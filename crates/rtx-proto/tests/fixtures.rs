// SPDX-License-Identifier: AGPL-3.0-or-later

//! Replay a captured QuakeWorld session through the parser.
//!
//! The unit tests prove the parser agrees with *our reading* of the reference clients. This proves
//! it agrees with a real server: point it at a directory of datagrams recorded from an actual
//! session and every one must decode, with no unknown opcodes and no desync. Between them, a
//! protocol mistake has nowhere to hide — the unit tests catch what we misunderstood, this catches
//! what we never thought to ask about.
//!
//! Captures live outside the repo (they're megabytes of binary, and each is specific to one
//! server's extension set), so this is opt-in via `RTX_TEST_QW_CAPTURE`, the same idiom as
//! `RTX_TEST_BSP` in rtx-nav. Generate one against a live server with either tool:
//!
//! ```sh
//! # Our own client, driving the codec end to end — also validates the map checksum:
//! cargo run -p rtx-proto --example probe -- \
//!     --server 127.0.0.1:27500 --basedir playground --secs 30 --out /tmp/qwfix
//!
//! # Or tap a real client, to capture bytes we didn't generate ourselves:
//! cargo run -p rtx-proto --example udptap -- --server 127.0.0.1:27500 --out /tmp/qwfix
//!
//! RTX_TEST_QW_CAPTURE=/tmp/qwfix cargo test -p rtx-proto --test fixtures -- --nocapture
//! ```
//!
//! Capture against each server family that matters — mvdsv, KTX, FTEQW — since the extension set
//! they negotiate is what decides which parser paths run.

use std::collections::BTreeMap;
use std::path::PathBuf;

use rtx_proto::info::Info;
use rtx_proto::netchan::Netchan;
use rtx_proto::protocol::ProtoState;
use rtx_proto::svc::{self, SvcEvent};
use rtx_proto::oob;

/// Every `*-s2c.bin` in the capture, in capture order — which is the order the netchan's sequence
/// numbers assume.
fn server_datagrams(dir: &PathBuf) -> Vec<(String, Vec<u8>)> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.ends_with("-s2c.bin")))
        .collect();
    files.sort(); // zero-padded capture index, so lexical order is capture order
    files
        .into_iter()
        .map(|p| {
            let name = p.file_name().unwrap().to_string_lossy().into_owned();
            (name, std::fs::read(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display())))
        })
        .collect()
}

/// Replay a whole captured session. Any datagram that fails to parse fails the test, naming the
/// file and dumping the bytes — which is the difference between "the bot desynced sometimes" and a
/// fixable bug report.
#[test]
fn replays_a_captured_session() {
    let Ok(dir) = std::env::var("RTX_TEST_QW_CAPTURE") else {
        eprintln!("RTX_TEST_QW_CAPTURE not set; skipping");
        return;
    };
    let dir = PathBuf::from(dir);
    let datagrams = server_datagrams(&dir);
    assert!(!datagrams.is_empty(), "no *-s2c.bin files in {}", dir.display());

    let mut chan = Netchan::new(0x4242);
    let mut proto = ProtoState::new();
    let mut census: BTreeMap<String, u32> = BTreeMap::new();
    let mut oob_seen = 0u32;
    let mut parsed = 0u32;
    let mut stale = 0u32;

    for (name, data) in &datagrams {
        // The handshake rides out of band, before there's a netchan.
        if oob::is_oob(data) {
            let ev = oob::parse(data).unwrap_or_else(|| panic!("{name}: unparseable OOB packet"));
            if let oob::Oob::Challenge { fte, fte2, mvd1, .. } = ev {
                // Only ever narrows — see `oob::Negotiated`.
                let n = oob::Negotiated::intersect(fte, fte2, mvd1);
                assert_eq!(n.fte & !rtx_proto::protocol::FTE, 0);
                assert_eq!(n.mvd1 & !rtx_proto::protocol::MVD1, 0);
            }
            oob_seen += 1;
            continue;
        }

        // A capture replays a real sequence stream, so stale/duplicate packets are expected and
        // must be dropped exactly as they were live.
        let Some(payload) = chan.process(data) else {
            stale += 1;
            continue;
        };

        let events = match svc::parse(&mut proto, payload) {
            Ok(e) => e,
            Err(e) => panic!(
                "{name}: parse failed: {e}\nnetchan seq={} proto={proto:?}\n{}",
                chan.incoming_sequence,
                svc::hexdump(payload)
            ),
        };
        parsed += 1;

        for ev in events {
            *census.entry(kind(&ev)).or_default() += 1;
            // The server's `*z_ext` echo gates how playerinfo decodes pm_type, and it arrives
            // inside a stufftext rather than as its own message.
            match &ev {
                SvcEvent::StuffText(t) => {
                    for line in t.lines() {
                        if let Some(rest) = line.trim().strip_prefix("fullserverinfo ") {
                            if let Some(z) = Info::parse(rest.trim_matches('"')).get_u32("*z_ext") {
                                proto.z_ext = z;
                            }
                        }
                    }
                }
                SvcEvent::ServerInfo { key, value } if key == "*z_ext" => {
                    if let Ok(z) = value.parse() {
                        proto.z_ext = z;
                    }
                }
                _ => {}
            }
        }
    }

    eprintln!(
        "replayed {} datagrams from {} ({oob_seen} out-of-band, {parsed} in-band, {stale} stale)",
        datagrams.len(),
        dir.display()
    );
    let mut rows: Vec<_> = census.iter().collect();
    rows.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
    for (k, n) in &rows {
        eprintln!("  {n:8}  {k}");
    }

    // A capture that never got past the handshake would pass every assertion above while proving
    // nothing, so require evidence that the interesting paths actually ran.
    assert!(parsed > 0, "no in-band packets — is this a real capture?");
    assert!(
        census.contains_key("serverdata"),
        "capture never reached serverdata; the extension-dependent paths were never exercised"
    );
}

/// A stable name per event kind.
fn kind(ev: &SvcEvent) -> String {
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
        SvcEvent::Download { .. } => "download",
        SvcEvent::PlayerInfo(_) => "playerinfo",
        SvcEvent::PacketEntities(pe) => {
            // Full and delta updates take different paths, and a capture that only ever saw full
            // updates hasn't tested the one that matters in a real game.
            return if pe.delta_from.is_some() { "deltapacketentities" } else { "packetentities" }.to_string();
        }
        SvcEvent::Nails(_) => "nails",
        SvcEvent::ModelList(_) => "modellist",
        SvcEvent::SoundList(_) => "soundlist",
        SvcEvent::Voice => "voice",
    }
    .to_string()
}
