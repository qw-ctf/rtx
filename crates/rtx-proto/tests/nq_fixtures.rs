// SPDX-License-Identifier: AGPL-3.0-or-later

//! Replay a captured NetQuake session through the parser — the NetQuake twin of `fixtures.rs`.
//!
//! The unit tests prove the parser agrees with *our reading* of QuakeSpasm-Spiked; this proves it
//! agrees with a real server. Point it at a directory of datagrams recorded from an actual session
//! and every one must decode, with no unknown opcodes and no desync.
//!
//! Captures live outside the repo (megabytes of binary, server-specific), so this is opt-in via
//! `RTX_TEST_NQ_CAPTURE`, the same idiom as `RTX_TEST_QW_CAPTURE`. Generate one against a live NQ
//! server:
//!
//! ```sh
//! playground/fteqw-macosx-sv +set sv_listen_nq 1 +set sv_port 26000 +map dm4 &
//! cargo run -p rtx-proto --example probe_nq -- --server 127.0.0.1:26000 --secs 20 --out /tmp/nqfix
//! RTX_TEST_NQ_CAPTURE=/tmp/nqfix cargo test -p rtx-proto --test nq_fixtures -- --nocapture
//! ```
//!
//! Capture against each protocol that matters (`sv_protocol 15`, `666`, `999`) since the version
//! decides which parser paths run.

use std::collections::BTreeMap;
use std::path::PathBuf;

use rtx_proto::nq::chan::{Incoming, NqChan};
use rtx_proto::nq::connect::{self, Ccrep};
use rtx_proto::nq::protocol::NqProtoState;
use rtx_proto::nq::svc;
use rtx_proto::svc::SvcEvent;

/// Every `*-s2c.bin` in the capture, in capture order (the order the Datagram sequence numbers
/// assume).
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

/// Replay a whole captured session. Any datagram that fails to parse fails the test, naming the file
/// and dumping the bytes.
#[test]
fn replays_a_captured_nq_session() {
    let Ok(dir) = std::env::var("RTX_TEST_NQ_CAPTURE") else {
        eprintln!("RTX_TEST_NQ_CAPTURE not set; skipping");
        return;
    };
    let dir = PathBuf::from(dir);
    let datagrams = server_datagrams(&dir);
    assert!(!datagrams.is_empty(), "no *-s2c.bin files in {}", dir.display());

    let mut chan = NqChan::new();
    let mut proto = NqProtoState::new();
    let mut census: BTreeMap<&'static str, u32> = BTreeMap::new();
    let mut control_seen = 0u32;
    let mut parsed = 0u32;
    let mut dropped = 0u32;
    let mut reached_serverinfo = false;
    let mut reached_active = false;

    for (name, data) in &datagrams {
        // The handshake rides in control packets, before the Datagram streams start.
        if connect::is_control(data) {
            match connect::parse_control(data) {
                Some(Ccrep::Accept { proquake, .. }) => proto.proquake_angles = proquake,
                Some(_) => {}
                None => panic!("{name}: malformed control packet"),
            }
            control_seen += 1;
            continue;
        }

        // A capture replays a real sequence stream, so stale/duplicate packets and bare acks carry
        // no payload — drop them exactly as the live channel would.
        let (incoming, _ack) = chan.process(data);
        let payload = match incoming {
            Incoming::Unreliable(p) | Incoming::Reliable(p) => p,
            Incoming::None => {
                dropped += 1;
                continue;
            }
        };

        let events = match svc::parse(&mut proto, &payload) {
            Ok(e) => e,
            Err(e) => panic!(
                "{name}: parse failed: {e}\nproto={proto:?}\n{}",
                rtx_proto::svc::hexdump(&payload)
            ),
        };
        parsed += 1;

        for ev in &events {
            *census.entry(kind(ev)).or_default() += 1;
            match ev {
                SvcEvent::NqServerData(_) => reached_serverinfo = true,
                SvcEvent::EntityUpdate(_) => reached_active = true,
                _ => {}
            }
        }
    }

    eprintln!(
        "nq_fixtures: {} datagrams — {control_seen} control, {parsed} parsed, {dropped} dropped",
        datagrams.len()
    );
    let mut rows: Vec<_> = census.iter().collect();
    rows.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
    for (k, n) in rows {
        eprintln!("  {n:8}  {k}");
    }

    // A capture that never reached serverinfo isn't testing the parser — it's testing an empty file.
    assert!(reached_serverinfo, "capture never delivered svc_serverinfo");
    assert!(reached_active, "capture never delivered an entity update (never entered the game)");
}

/// A stable name per event kind, for the census.
fn kind(ev: &SvcEvent) -> &'static str {
    match ev {
        SvcEvent::EntityUpdate(_) => "entityupdate",
        SvcEvent::ClientData(_) => "clientdata",
        SvcEvent::Time(_) => "time",
        SvcEvent::SpawnBaseline { .. } => "spawnbaseline",
        SvcEvent::SpawnStatic(_) => "spawnstatic",
        SvcEvent::LightStyle { .. } => "lightstyle",
        SvcEvent::Sound { .. } => "sound",
        SvcEvent::SpawnStaticSound { .. } => "spawnstaticsound",
        SvcEvent::TempEntity(_) => "temp_entity",
        SvcEvent::Particle { .. } => "particle",
        SvcEvent::UpdateName { .. } => "updatename",
        SvcEvent::UpdateFrags { .. } => "updatefrags",
        SvcEvent::UpdateColors { .. } => "updatecolors",
        SvcEvent::UpdateStat { .. } => "updatestat",
        SvcEvent::SignonNum(_) => "signonnum",
        SvcEvent::NqServerData(_) => "serverinfo",
        SvcEvent::SetView(_) => "setview",
        SvcEvent::SetAngle { .. } => "setangle",
        SvcEvent::Print { .. } => "print",
        SvcEvent::StuffText(_) => "stufftext",
        _ => "other",
    }
}
