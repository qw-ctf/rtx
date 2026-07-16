// SPDX-License-Identifier: AGPL-3.0-or-later

//! Connect to a real **NetQuake** server with nothing but this crate, walk the signon into the game,
//! and report what the parser made of every byte. The NetQuake twin of `probe.rs`.
//!
//! Unit tests prove the parser agrees with *my reading* of QuakeSpasm-Spiked; only a live server
//! proves it agrees with the reference itself. Point it at FTEQW (`sv_listen_nq 1`), QuakeSpasm or a
//! classic ProQuake server and it either reaches `begin` cleanly or names the exact byte it choked
//! on.
//!
//! It is **not** the bot client — no world model, no movement, no brain. It connects, drains the
//! stream, and prints a census. The real session lives in the netclient.
//!
//! ```sh
//! playground/fteqw-macosx-sv +sv_listen_nq 1 +map dm4 &
//! cargo run -p rtx-proto --example probe_nq -- --server 127.0.0.1:26000 --secs 20 --out fixtures/
//! ```
//!
//! With `--out` every datagram is written as a fixture (`NNNNNN-{s2c,c2s}.bin`), the corpus
//! `tests/nq_fixtures.rs` replays forever after.

use std::collections::BTreeMap;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use rtx_proto::nq::chan::{Incoming, NqChan};
use rtx_proto::nq::connect::{self, Ccrep};
use rtx_proto::nq::protocol::{NqProtoState, PORT};
use rtx_proto::nq::{clc, svc};
use rtx_proto::svc::SvcEvent;

struct Args {
    server: SocketAddr,
    secs: u64,
    out: Option<PathBuf>,
    name: String,
    verbose: bool,
}

fn usage() -> ! {
    eprintln!(
        "usage: probe_nq --server <host:port> [--secs <n>] [--out <dir>] [--name <s>] [-v]\n\n\
         Connects to a NetQuake server, walks the signon, parses everything, then reports.\n\
         Exits non-zero if any datagram fails to parse."
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut a = Args {
        server: ([127, 0, 0, 1], PORT).into(),
        secs: 20,
        out: None,
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
                let s = if s.contains(':') { s } else { format!("{s}:{PORT}") };
                a.server = s
                    .to_socket_addrs()
                    .unwrap_or_else(|e| {
                        eprintln!("probe_nq: resolve {s}: {e}");
                        std::process::exit(1)
                    })
                    .next()
                    .unwrap_or_else(|| usage());
            }
            "--secs" => a.secs = val().parse().unwrap_or_else(|_| usage()),
            "--out" => a.out = Some(PathBuf::from(val())),
            "--name" => a.name = val(),
            _ => usage(),
        }
        i += 2;
    }
    a
}

/// How far through the handshake we've got. The probe stops at `Active`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum State {
    Connecting,
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
    let mut server = args.server;
    let mut chan = NqChan::new();
    let mut proto = NqProtoState::new();

    let mut state = State::Connecting;
    let mut census: BTreeMap<&'static str, u32> = BTreeMap::new();
    let mut last_svc_time = 0.0f32;
    let mut nfix = 0u32;
    let mut buf = [0u8; 65536];

    eprintln!("probe_nq: → {server} (CCREQ_CONNECT)");
    sock.send_to(&connect::connect_request(), server)?;

    let start = Instant::now();
    let mut last_send = Instant::now();
    let mut last_reliable = Instant::now();
    let deadline = Duration::from_secs(args.secs);

    while start.elapsed() < deadline {
        // --- receive (bounded per tick, so the send phase still gets a turn under a fast stream) ---
        for _ in 0..64 {
            let Ok((len, _from)) = sock.recv_from(&mut buf) else { break };
            let data = &buf[..len];
            if let Some(dir) = &args.out {
                nfix += 1;
                std::fs::write(dir.join(format!("{nfix:06}-s2c.bin")), data)?;
            }

            // Control packets (the handshake) live outside the netchan.
            if connect::is_control(data) {
                match connect::parse_control(data) {
                    Some(Ccrep::Accept { port, proquake, ignore_port }) if state == State::Connecting => {
                        proto.proquake_angles = proquake;
                        if let Some(p) = (Ccrep::Accept { port, proquake, ignore_port }).switch_to() {
                            server.set_port(p);
                            eprintln!("probe_nq: accepted; switching to data port {p}");
                        } else {
                            eprintln!("probe_nq: accepted (proquake={proquake}, keeping port)");
                        }
                        // NAT-punch: one unreliable nop so the server's data port can reach us.
                        sock.send_to(&chan.transmit_unreliable(&clc::write_nop()), server)?;
                        state = State::Signon;
                    }
                    Some(Ccrep::Reject(why)) => {
                        eprintln!("probe_nq: rejected: {why}");
                        std::process::exit(1);
                    }
                    _ => {}
                }
                continue;
            }

            let (incoming, ack) = chan.process(data);
            if let Some(ack) = ack {
                sock.send_to(&ack, server)?;
            }
            let payload = match incoming {
                Incoming::Unreliable(p) | Incoming::Reliable(p) => p,
                Incoming::None => continue,
            };

            let events = match svc::parse(&mut proto, &payload) {
                Ok(e) => e,
                Err(e) => {
                    eprintln!("\nprobe_nq: PARSE FAILED after {:.1}s: {e}", start.elapsed().as_secs_f32());
                    eprintln!("probe_nq: state={state:?} proto={proto:?}");
                    eprintln!("{}", rtx_proto::svc::hexdump(&payload));
                    std::process::exit(1);
                }
            };

            for ev in events {
                *census.entry(name_of(&ev)).or_default() += 1;
                match ev {
                    SvcEvent::NqServerData(sd) => {
                        eprintln!(
                            "probe_nq: serverinfo: protocol={} flags=0x{:x} maxclients={} gametype={} \
                             map={:?}\nprobe_nq:   {} models, {} sounds",
                            sd.protocol, sd.flags, sd.maxclients, sd.gametype,
                            sd.models.get(1).map(|s| s.as_str()).unwrap_or("?"),
                            sd.models.len().saturating_sub(1), sd.sounds.len().saturating_sub(1),
                        );
                    }
                    SvcEvent::SignonNum(n) => {
                        eprintln!("probe_nq: signon {n}");
                        match n {
                            1 => {
                                chan.queue_reliable(&clc::write_stringcmd(&format!("name \"{}\"", args.name)));
                                chan.queue_reliable(&clc::write_stringcmd("prespawn"));
                            }
                            2 => {
                                chan.queue_reliable(&clc::write_stringcmd("color 0 0"));
                                chan.queue_reliable(&clc::write_stringcmd("spawn"));
                            }
                            3 => chan.queue_reliable(&clc::write_stringcmd("begin")),
                            _ => {}
                        }
                    }
                    SvcEvent::StuffText(t) => {
                        if args.verbose {
                            eprintln!("probe_nq: stufftext: {:?}", t.trim_end());
                        }
                        // Echo any `cmd X` back as a stringcmd — the server's signon lever. This
                        // covers FitzQuake/RMQ's `cmd prespawn` and `cmd pext` (whose bare `pext`
                        // reply declines every FTE extension). `//` lines are engine commands we
                        // ignore.
                        for line in t.lines() {
                            if let Some(rest) = line.trim().strip_prefix("cmd ") {
                                chan.queue_reliable(&clc::write_stringcmd(rest));
                            }
                        }
                    }
                    SvcEvent::Time(t) => last_svc_time = t,
                    SvcEvent::EntityUpdate(_) if state == State::Signon => {
                        state = State::Active;
                        eprintln!("probe_nq: entered the game at {:.1}s", start.elapsed().as_secs_f32());
                    }
                    SvcEvent::Print { text, .. } if args.verbose => eprint!("probe_nq: print: {text}"),
                    SvcEvent::Disconnect => {
                        eprintln!("probe_nq: server disconnected us");
                        report(&census, nfix, state);
                        return Ok(());
                    }
                    _ => {}
                }
            }
        }

        // --- send: drain the reliable queue, then keep the connection alive ---
        let mut sent_reliable = false;
        while let Some(frag) = chan.reliable_to_send() {
            record_c2s(&args, &mut nfix, &frag)?;
            sock.send_to(&frag, server)?;
            sent_reliable = true;
        }
        if sent_reliable {
            last_reliable = Instant::now();
        } else if chan.reliable_pending() && last_reliable.elapsed() >= Duration::from_secs(1) {
            if let Some(frag) = chan.reliable_resend() {
                sock.send_to(&frag, server)?;
                last_reliable = Instant::now();
            }
        }

        if last_send.elapsed() >= Duration::from_millis(50) {
            last_send = Instant::now();
            let payload = if state == State::Active {
                clc::write_move(&proto, last_svc_time, glam::Vec3::ZERO, 0, 0, 0, 0, 0)
            } else {
                clc::write_nop()
            };
            let datagram = chan.transmit_unreliable(&payload);
            record_c2s(&args, &mut nfix, &datagram)?;
            sock.send_to(&datagram, server)?;
        }
    }

    report(&census, nfix, state);
    if state != State::Active {
        eprintln!("probe_nq: NOTE: never entered the game (stalled in {state:?})");
        std::process::exit(1);
    }
    Ok(())
}

fn record_c2s(args: &Args, nfix: &mut u32, data: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = &args.out {
        *nfix += 1;
        std::fs::write(dir.join(format!("{nfix:06}-c2s.bin")), data)?;
    }
    Ok(())
}

fn report(census: &BTreeMap<&'static str, u32>, fixtures: u32, state: State) {
    eprintln!(
        "\nprobe_nq: parsed cleanly. state={state:?}{}",
        if fixtures > 0 { format!(", {fixtures} fixtures written") } else { String::new() }
    );
    eprintln!("probe_nq: message census:");
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
        SvcEvent::SetAngle { .. } => "setangle",
        SvcEvent::LightStyle { .. } => "lightstyle",
        SvcEvent::Sound { .. } => "sound",
        SvcEvent::StopSound { .. } => "stopsound",
        SvcEvent::UpdateFrags { .. } => "updatefrags",
        SvcEvent::UpdateStat { .. } => "updatestat",
        SvcEvent::SpawnBaseline { .. } => "spawnbaseline",
        SvcEvent::SpawnStatic(_) => "spawnstatic",
        SvcEvent::SpawnStaticSound { .. } => "spawnstaticsound",
        SvcEvent::TempEntity(_) => "temp_entity",
        SvcEvent::Intermission { .. } => "intermission",
        SvcEvent::Finale(_) => "finale",
        SvcEvent::CdTrack(_) => "cdtrack",
        SvcEvent::SellScreen => "sellscreen",
        SvcEvent::KilledMonster => "killedmonster",
        SvcEvent::FoundSecret => "foundsecret",
        SvcEvent::SetPause(_) => "setpause",
        SvcEvent::Time(_) => "time",
        SvcEvent::SignonNum(_) => "signonnum",
        SvcEvent::NqServerData(_) => "serverinfo",
        SvcEvent::ClientData(_) => "clientdata",
        SvcEvent::UpdateName { .. } => "updatename",
        SvcEvent::UpdateColors { .. } => "updatecolors",
        SvcEvent::SetView(_) => "setview",
        SvcEvent::EntityUpdate(_) => "entityupdate",
        SvcEvent::Particle { .. } => "particle",
        _ => "other",
    }
}
