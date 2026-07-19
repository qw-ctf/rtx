// SPDX-License-Identifier: AGPL-3.0-or-later

//! Dump a map's built navmesh as JSON for external viewers (qw-nav-viewer's
//! `?overlay=` real-graph layer — see route-lab
//! `docs/plans/2026-07-19-spawn7-orchestration.md`).
//!
//! Usage: `nav_dump <map.bsp> [--rjump] > graph.json`
//!
//! Builds the graph exactly like the live route server defaults (no hooks, no
//! double jump, speed jumps off, rocket jumps off unless `--rjump`) and emits:
//! `{"schema":"qw-nav-graph/1","map":<file stem>,"grid":32.0,
//!   "cells":[[x,y,z],...],"links":[[from,to,"Kind",cost],...]}`
//! Cells are origin positions; link indexes refer into `cells`.

use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::{build_navmesh, LinkKind, RocketJumpParams};

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: nav_dump <map.bsp> [--rjump]");
        std::process::exit(2);
    };
    let rjump = args.any(|a| a == "--rjump");
    let bytes = match std::fs::read(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("read {path}: {e}");
            std::process::exit(1);
        }
    };
    let rocket_jump = rjump.then_some(RocketJumpParams { gravity: 800.0, rj_extra: 0.0 });
    let Some(bsp) = Bsp::parse(&bytes) else {
        eprintln!("unsupported/malformed BSP: {path}");
        std::process::exit(1);
    };
    let graph = build_navmesh(&bsp, vec![], vec![], vec![], None, false, None, rocket_jump);
    let stem = std::path::Path::new(&path)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown");
    let mut out = String::with_capacity(graph.cells.len() * 24 + graph.links.len() * 24);
    out.push_str(&format!(
        "{{\"schema\":\"qw-nav-graph/1\",\"map\":\"{stem}\",\"grid\":32.0,\"cells\":["
    ));
    for (i, cell) in graph.cells.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        let o = cell.origin;
        out.push_str(&format!("[{},{},{}]", o.x, o.y, o.z));
    }
    out.push_str("],\"links\":[");
    for (i, link) in graph.links.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        let kind = match link.kind {
            LinkKind::Walk => "Walk",
            LinkKind::Step => "Step",
            LinkKind::Drop => "Drop",
            LinkKind::JumpGap => "JumpGap",
            LinkKind::DoubleJump => "DoubleJump",
            LinkKind::SpeedJump => "SpeedJump",
            LinkKind::Teleport => "Teleport",
            LinkKind::Plat => "Plat",
            LinkKind::Hook => "Hook",
            LinkKind::RocketJump => "RocketJump",
        };
        out.push_str(&format!("[{},{},\"{kind}\",{}]", link.from, link.to, link.cost));
    }
    out.push_str("]}");
    println!("{out}");
}
