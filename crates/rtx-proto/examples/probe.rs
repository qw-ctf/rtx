// SPDX-License-Identifier: AGPL-3.0-or-later

//! Connect to a real QuakeWorld server with nothing but this crate, walk the signon to the point
//! of entering the game, and report what the parser made of every byte.
//!
//! This is the codec's reality check. Unit tests prove the parser agrees with *my reading* of the
//! reference clients; only a live server proves it agrees with the reference clients themselves.
//! Point it at mvdsv, KTX or FTEQW and it either reaches `begin` cleanly or names the exact byte it
//! choked on.
//!
//! It is **not** the bot client — there's no world model, no movement, no brain. It connects as a
//! spectator, drains the stream, and prints a census. The real session lives in the netclient.
//!
//! ```sh
//! playground/mvdsv +exec rjtest.cfg &
//! cargo run -p rtx-proto --example probe -- --server 127.0.0.1:27500 --secs 20 --out fixtures/
//! ```
//!
//! With `--out` every datagram is written as a fixture, exactly as `udptap` does — so a single
//! probe run against a live server generates the corpus `tests/fixtures.rs` replays forever after.

use std::collections::BTreeMap;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use rtx_proto::info::{Info, UserinfoBuilder};
use rtx_proto::netchan::Netchan;
use rtx_proto::protocol::ProtoState;
use rtx_proto::svc::{self, SvcEvent};
use rtx_proto::{clc, oob};

struct Args {
    server: SocketAddr,
    secs: u64,
    out: Option<PathBuf>,
    basedir: Option<PathBuf>,
    name: String,
    verbose: bool,
}

fn usage() -> ! {
    eprintln!(
        "usage: probe --server <host:port> [--basedir <dir>] [--secs <n>] [--out <dir>]\n\
         \x20            [--name <s>] [-v]\n\n\
         Connects as a spectator and parses everything the server sends, then reports.\n\
         Exits non-zero if any datagram fails to parse.\n\n\
         \x20 --basedir  Quake dir holding the maps. Given one, the probe computes the real map\n\
         \x20            checksum and completes signon — which is the only way to check the\n\
         \x20            checksum against a live server's opinion of it."
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut a = Args {
        server: ([127, 0, 0, 1], 27500).into(),
        secs: 20,
        out: None,
        basedir: None,
        name: "rtxprobe".to_string(),
        verbose: false,
    };
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        let val = || argv.get(i + 1).cloned().unwrap_or_else(|| usage());
        match argv[i].as_str() {
            "-v" | "--verbose" => {
                a.verbose = true;
                i += 1;
                continue;
            }
            "--server" => {
                let s = val();
                let s = if s.contains(':') { s } else { format!("{s}:27500") };
                a.server = s
                    .to_socket_addrs()
                    .unwrap_or_else(|e| {
                        eprintln!("probe: resolve {s}: {e}");
                        std::process::exit(1)
                    })
                    .next()
                    .unwrap_or_else(|| usage());
            }
            "--secs" => a.secs = val().parse().unwrap_or_else(|_| usage()),
            "--out" => a.out = Some(PathBuf::from(val())),
            "--basedir" => a.basedir = Some(PathBuf::from(val())),
            "--name" => a.name = val(),
            _ => usage(),
        }
        i += 2;
    }
    a
}

/// How far through signon we've got. The probe stops at `Active` — it has proved what it set out
/// to prove by then.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum State {
    Challenge,
    Connect,
    Signon,
    Active,
}

fn main() -> std::io::Result<()> {
    let args = parse_args();
    if let Some(d) = &args.out {
        std::fs::create_dir_all(d)?;
    }

    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.set_read_timeout(Some(Duration::from_millis(20)))?;
    // A real qport is random; a fixed one keeps captures diffable.
    let mut chan = Netchan::new(0x4242);
    let mut proto = ProtoState::new();

    let mut state = State::Challenge;
    let mut servercount = 0i32;
    let mut gamedir = String::from("qw");
    let mut census: BTreeMap<&'static str, u32> = BTreeMap::new();
    let mut soundlist: Vec<String> = Vec::new();
    let mut modellist: Vec<String> = Vec::new();
    let mut serverinfo = Info::new();
    let mut delta_ack: Option<u8> = None;
    let mut delta_mismatch = 0u32;
    let mut nfix = 0u32;
    let mut buf = [0u8; 8192];

    eprintln!("probe: → {}", args.server);
    sock.send_to(&oob::getchallenge(), args.server)?;

    let start = Instant::now();
    let mut last_send = Instant::now();
    let deadline = Duration::from_secs(args.secs);

    while start.elapsed() < deadline {
        // --- receive ---
        while let Ok((len, _from)) = sock.recv_from(&mut buf) {
            let data = &buf[..len];
            if let Some(dir) = &args.out {
                nfix += 1;
                std::fs::write(dir.join(format!("{nfix:06}-s2c.bin")), data)?;
            }

            if oob::is_oob(data) {
                match oob::parse(data) {
                    Some(oob::Oob::Challenge { challenge, fte, fte2, mvd1 }) if state == State::Challenge => {
                        let n = oob::Negotiated::intersect(fte, fte2, mvd1);
                        eprintln!(
                            "probe: challenge {challenge}; server offers fte=0x{fte:x} fte2=0x{fte2:x} \
                             mvd1=0x{mvd1:x} → agreed fte=0x{:x} fte2=0x{:x} mvd1=0x{:x}",
                            n.fte, n.fte2, n.mvd1
                        );
                        let ui = UserinfoBuilder {
                            name: args.name.clone(),
                            spectator: true, // watch, don't play — this is a parser probe
                            ..Default::default()
                        };
                        let pkt = oob::connect(chan.qport, challenge, &ui.build().to_string(), &n);
                        sock.send_to(&pkt, args.server)?;
                        state = State::Connect;
                    }
                    Some(oob::Oob::Accepted) if state == State::Connect => {
                        eprintln!("probe: accepted; sending `new`");
                        chan.queue_reliable(&clc::write_stringcmd("new"));
                        state = State::Signon;
                    }
                    Some(oob::Oob::Print(t)) => eprintln!("probe: server says: {}", t.trim_end()),
                    other => {
                        if args.verbose {
                            eprintln!("probe: oob {other:?}");
                        }
                    }
                }
                continue;
            }

            let Some(payload) = chan.process(data) else { continue };
            let events = match svc::parse(&mut proto, payload) {
                Ok(e) => e,
                Err(e) => {
                    // The whole point of the probe: say exactly what broke, with the bytes.
                    eprintln!("\nprobe: PARSE FAILED after {:.1}s: {e}", start.elapsed().as_secs_f32());
                    eprintln!("probe: state={state:?} seq={} proto={proto:?}", chan.incoming_sequence);
                    eprintln!("{}", svc::hexdump(payload));
                    std::process::exit(1);
                }
            };

            for ev in events {
                *census.entry(name_of(&ev)).or_default() += 1;
                match ev {
                    SvcEvent::ServerData(sd) => {
                        eprintln!(
                            "probe: serverdata: gamedir={} map={:?} playernum={} spectator={} \
                             count={}\nprobe:   negotiated fte=0x{:x} fte2=0x{:x} mvd1=0x{:x} → coord={}B angle={}B\n\
                             probe:   movevars: gravity={} maxspeed={} accel={} airaccel={} friction={}",
                            sd.gamedir, sd.levelname, sd.playernum, sd.spectator, sd.servercount,
                            sd.fte, sd.fte2, sd.mvd1, proto.coord_bytes, proto.angle_bytes,
                            sd.movevars.gravity, sd.movevars.maxspeed, sd.movevars.accelerate,
                            sd.movevars.airaccelerate, sd.movevars.friction,
                        );
                        servercount = sd.servercount;
                        gamedir = sd.gamedir.clone();
                        soundlist.clear();
                        modellist.clear();
                        chan.queue_reliable(&clc::write_stringcmd(&format!("soundlist {servercount} 0")));
                    }
                    SvcEvent::SoundList(list) => {
                        soundlist.extend(list.names);
                        let next = list.next;
                        let cmd = if next != 0 {
                            format!("soundlist {servercount} {next}")
                        } else {
                            eprintln!("probe: {} sounds; asking for models", soundlist.len());
                            format!("modellist {servercount} 0")
                        };
                        chan.queue_reliable(&clc::write_stringcmd(&cmd));
                    }
                    SvcEvent::ModelList(list) => {
                        let start_idx = list.start as usize;
                        modellist.extend(list.names);
                        let next = list.next;
                        if next != 0 {
                            // The continuation index carries the high byte of the count — the low
                            // byte is what the server hands back.
                            let off = (modellist.len() + start_idx.saturating_sub(modellist.len())) & 0xff00;
                            chan.queue_reliable(&clc::write_stringcmd(&format!(
                                "modellist {servercount} {}",
                                off + next as usize
                            )));
                        } else {
                            // A client learns the map's filename from the model list — entry 0 is
                            // always `maps/<name>.bsp`. That's the only place it's named; the
                            // `levelname` in serverdata is the display title and can be anything.
                            let checksum = modellist
                                .first()
                                .and_then(|m| map_checksum(&args, &gamedir, m))
                                .unwrap_or(0);
                            eprintln!(
                                "probe: {} models; map={} checksum2=0x{:08x}{}",
                                modellist.len(),
                                modellist.first().map(|s| s.as_str()).unwrap_or("?"),
                                checksum as u32,
                                if checksum == 0 { "  (no basedir — server may refuse)" } else { "" }
                            );
                            chan.queue_reliable(&clc::write_stringcmd(&format!(
                                "prespawn {servercount} 0 {checksum}"
                            )));
                        }
                    }
                    SvcEvent::StuffText(t) => {
                        if args.verbose {
                            eprintln!("probe: stufftext: {:?}", t.trim_end());
                        }
                        for line in t.lines() {
                            let line = line.trim();
                            if let Some(rest) = line.strip_prefix("cmd ") {
                                chan.queue_reliable(&clc::write_stringcmd(rest));
                            } else if let Some(rest) = line.strip_prefix("fullserverinfo ") {
                                let s = rest.trim_matches('"');
                                serverinfo = Info::parse(s);
                                if let Some(z) = serverinfo.get_u32("*z_ext") {
                                    proto.z_ext = z;
                                    eprintln!("probe: serverinfo *z_ext = 0x{z:x}");
                                }
                            } else if line == "skins" {
                                chan.queue_reliable(&clc::write_stringcmd(&format!("begin {servercount}")));
                                state = State::Active;
                                eprintln!("probe: entered the game at {:.1}s", start.elapsed().as_secs_f32());
                            }
                        }
                    }
                    SvcEvent::ServerInfo { key, value } => {
                        if key == "*z_ext" {
                            if let Ok(z) = value.parse() {
                                proto.z_ext = z;
                            }
                        }
                        serverinfo.set(&key, &value);
                    }
                    SvcEvent::Print { text, .. } if args.verbose => {
                        eprint!("probe: print: {text}");
                    }
                    SvcEvent::PacketEntities(pe) => {
                        // Acknowledge the frame so the server delta-compresses the next one. Until
                        // a client does this, the server only ever sends full updates — so without
                        // it the delta path, which carries essentially all real gameplay traffic,
                        // is never exercised.
                        //
                        // A real client must also verify `pe.delta_from` against the frame it
                        // recorded, and drop the update if they disagree. The probe doesn't keep a
                        // frame ring; it just reports mismatches, since its job is to prove the
                        // bytes decode.
                        if let (Some(from), Some(want)) = (pe.delta_from, delta_ack) {
                            if from != want {
                                delta_mismatch += 1;
                            }
                        }
                        delta_ack = Some((chan.incoming_sequence & 0xff) as u8);
                    }
                    SvcEvent::Disconnect => {
                        eprintln!("probe: server disconnected us");
                        report(&census, &soundlist, &modellist, nfix, state, delta_mismatch);
                        return Ok(());
                    }
                    _ => {}
                }
            }
        }

        // --- send: keep the connection alive and the reliable queue moving ---
        if last_send.elapsed() >= Duration::from_millis(25) {
            last_send = Instant::now();
            let payload = if state == State::Active {
                // A spectator still has to send moves, or the server times us out.
                let cmd = clc::make_usercmd(25, glam::Vec3::ZERO, 0, 0, 0, 0, 0);
                clc::write_move(
                    &clc::Move { oldest: cmd, previous: cmd, current: cmd, loss: 0 },
                    chan.outgoing_sequence,
                    delta_ack,
                )
            } else {
                clc::write_nop()
            };
            let datagram = chan.transmit(&payload);
            if let Some(dir) = &args.out {
                nfix += 1;
                std::fs::write(dir.join(format!("{nfix:06}-c2s.bin")), &datagram)?;
            }
            sock.send_to(&datagram, args.server)?;
        }
    }

    report(&census, &soundlist, &modellist, nfix, state, delta_mismatch);
    if state != State::Active {
        eprintln!("probe: NOTE: never entered the game (stalled in {state:?})");
    }
    Ok(())
}

/// Find the map on disk and checksum it, the way a client must before `prespawn`.
///
/// The search order is a client's: the server's gamedir first, then `qw`, then `id1` — so a mod's
/// override of an id map wins, which is what the server will have loaded too.
fn map_checksum(args: &Args, gamedir: &str, model: &str) -> Option<i32> {
    let base = args.basedir.as_ref()?;
    let name = model.strip_prefix("maps/")?.strip_suffix(".bsp")?;
    for dir in [gamedir, "qw", "id1"] {
        let path = base.join(dir).join(model);
        if let Ok(bytes) = std::fs::read(&path) {
            match rtx_proto::checksum::map_checksum2(&bytes, name) {
                Ok(sum) => {
                    eprintln!("probe: {} ({} bytes)", path.display(), bytes.len());
                    return Some(sum);
                }
                Err(e) => eprintln!("probe: {}: {e}", path.display()),
            }
        }
    }
    eprintln!("probe: {model} not found under {}", base.display());
    None
}

fn report(
    census: &BTreeMap<&'static str, u32>,
    sounds: &[String],
    models: &[String],
    fixtures: u32,
    state: State,
    delta_mismatch: u32,
) {
    eprintln!("\nprobe: parsed cleanly. state={state:?}, {} sounds, {} models{}", sounds.len(), models.len(),
        if fixtures > 0 { format!(", {fixtures} fixtures written") } else { String::new() });
    if delta_mismatch > 0 {
        eprintln!("probe: {delta_mismatch} delta updates referenced a frame we hadn't acked");
    }
    eprintln!("probe: message census:");
    let mut rows: Vec<_> = census.iter().collect();
    rows.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
    for (name, n) in rows {
        eprintln!("  {n:8}  {name}");
    }
}

/// A stable name per event kind, for the census.
fn name_of(ev: &SvcEvent) -> &'static str {
    match ev {
        SvcEvent::Nop => "nop",
        SvcEvent::Disconnect => "disconnect",
        SvcEvent::Print { .. } => "print",
        SvcEvent::CenterPrint(_) => "centerprint",
        SvcEvent::StuffText(_) => "stufftext",
        SvcEvent::Damage { .. } => "damage",
        SvcEvent::ServerData(_) => "serverdata",
        SvcEvent::SetAngle { .. } => "setangle",
        SvcEvent::LightStyle { .. } => "lightstyle",
        SvcEvent::Sound { .. } => "sound",
        SvcEvent::StopSound { .. } => "stopsound",
        SvcEvent::UpdateFrags { .. } => "updatefrags",
        SvcEvent::UpdatePing { .. } => "updateping",
        SvcEvent::UpdatePl { .. } => "updatepl",
        SvcEvent::UpdateEnterTime { .. } => "updateentertime",
        SvcEvent::UpdateStat { .. } => "updatestat",
        SvcEvent::UpdateUserinfo { .. } => "updateuserinfo",
        SvcEvent::SetInfo { .. } => "setinfo",
        SvcEvent::ServerInfo { .. } => "serverinfo",
        SvcEvent::SpawnBaseline { .. } => "spawnbaseline",
        SvcEvent::SpawnStatic(_) => "spawnstatic",
        SvcEvent::SpawnBaselineDelta { .. } => "spawnbaseline2 (fte)",
        SvcEvent::SpawnStaticDelta(_) => "spawnstatic2 (fte)",
        SvcEvent::SpawnStaticSound { .. } => "spawnstaticsound",
        SvcEvent::TempEntity(_) => "temp_entity",
        SvcEvent::MuzzleFlash { .. } => "muzzleflash",
        SvcEvent::SmallKick => "smallkick",
        SvcEvent::BigKick => "bigkick",
        SvcEvent::ChokeCount(_) => "chokecount",
        SvcEvent::Intermission { .. } => "intermission",
        SvcEvent::Finale(_) => "finale",
        SvcEvent::CdTrack(_) => "cdtrack",
        SvcEvent::SellScreen => "sellscreen",
        SvcEvent::KilledMonster => "killedmonster",
        SvcEvent::FoundSecret => "foundsecret",
        SvcEvent::MaxSpeed(_) => "maxspeed",
        SvcEvent::EntGravity(_) => "entgravity",
        SvcEvent::SetPause(_) => "setpause",
        SvcEvent::Download(_) => "download",
        SvcEvent::PlayerInfo(_) => "playerinfo",
        // Worth splitting: a session that only ever saw full updates never exercised the delta
        // path, which is what carries essentially all real gameplay traffic.
        SvcEvent::PacketEntities(pe) if pe.delta_from.is_some() => "deltapacketentities",
        SvcEvent::PacketEntities(_) => "packetentities (full)",
        SvcEvent::Nails(_) => "nails",
        SvcEvent::ModelList(_) => "modellist",
        SvcEvent::SoundList(_) => "soundlist",
        SvcEvent::Voice => "voice",
        // NetQuake-only events; a QuakeWorld session never produces them.
        SvcEvent::Time(_) => "time (nq)",
        SvcEvent::SignonNum(_) => "signonnum (nq)",
        SvcEvent::NqServerData(_) => "serverinfo (nq)",
        SvcEvent::ClientData(_) => "clientdata (nq)",
        SvcEvent::UpdateName { .. } => "updatename (nq)",
        SvcEvent::UpdateColors { .. } => "updatecolors (nq)",
        SvcEvent::SetView(_) => "setview (nq)",
        SvcEvent::EntityUpdate(_) => "entityupdate (nq)",
        SvcEvent::Particle { .. } => "particle (nq)",
    }
}
