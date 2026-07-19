// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shardable, deterministic ground-turn-curl candidate search.
//!
//! Standalone harness for the vmonster low-entry campaign (route-lab
//! `docs/plans/2026-07-18-vmonster-gt-curl-farm.md`): reuses the exact
//! certifier the live navmesh build uses
//! (`NavGraph::solve_chained_ground_turn_from`, see
//! `crates/rtx-nav/src/navmesh/jumps.rs`), just with a caller-supplied
//! carried-entry ladder and a stable modulo-N shard over the ledge-cell
//! space, so the search can be split across many machines/threads without
//! rewriting the certifier.
//!
//! No engine/game-state access, no wall clock, no OS rng anywhere in this
//! binary or in the certifier it calls: the whole pass is an exhaustive
//! bounded lattice scan over deterministic inputs, so the same BSP + the
//! same shard + the same args always produce byte-identical JSON. The
//! `seed` field in the output is provenance only (a fingerprint of the
//! invocation), never an RNG seed.
//!
//! Usage:
//! ```text
//! gt-search --bsp <path> --shard i/N --out <json> \
//!           [--entry-ladder 320,340,360,390,430] [--calib <json>] \
//!           [--near x,y,z,r]
//! ```
//!
//! `--shard i/N` selects ledge cells `id % N == i` (0-indexed, `i < N`).
//! `--entry-ladder` is the carried-entry-speed ladder sampled by the
//! certifier's canonical-scout gate and full lattice (default matches the
//! campaign's low-entry exploration: 320..430). `--calib` is optional and
//! purely informational here — its path and byte length are stamped into
//! the output metadata; it is NOT parsed or used to bias/filter the search
//! (that is reserved for a later fail-closed gate, not this generator).
//! `--near` optionally limits the selected shard to ledge-cell origins within
//! a 3D radius, avoiding unrelated certifier work in local microtests.

use std::fmt::Write as _;
use std::path::PathBuf;

use glam::Vec3;
use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::{build_navmesh, CellId, Link, SpeedJumpParams, SpeedJumpTraversal};

const GT_SEARCH_VERSION: &str = "gt-search/1";

struct Args {
    bsp: PathBuf,
    shard_i: u32,
    shard_n: u32,
    out: PathBuf,
    entry_ladder: Vec<f32>,
    calib: Option<PathBuf>,
    near: Option<(Vec3, f32)>,
    /// Use the additive ground-optimal single-sided sweep certifier
    /// (`NavGraph::solve_chained_ground_turn_optimal_curl`) instead of the
    /// default bearing-follow weave (`solve_chained_ground_turn_from`). Off by
    /// default, so the harness's canonical pass is byte-identical to before.
    optimal: bool,
}

fn usage() -> ! {
    eprintln!(
        "usage: gt-search --bsp <path> --shard i/N --out <json> \
         [--entry-ladder 320,340,360,390,430] [--calib <json>] \
         [--near x,y,z,r] [--optimal]"
    );
    std::process::exit(2);
}

fn parse_args() -> Args {
    let mut bsp = None;
    let mut shard = None;
    let mut out = None;
    let mut entry_ladder: Vec<f32> = vec![320.0, 340.0, 360.0, 390.0, 430.0];
    let mut calib = None;
    let mut near = None;
    let mut optimal = false;

    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--bsp" => bsp = Some(PathBuf::from(argv.next().unwrap_or_else(|| usage()))),
            "--shard" => shard = Some(argv.next().unwrap_or_else(|| usage())),
            "--out" => out = Some(PathBuf::from(argv.next().unwrap_or_else(|| usage()))),
            "--entry-ladder" => {
                let s = argv.next().unwrap_or_else(|| usage());
                entry_ladder = s
                    .split(',')
                    .map(|t| t.trim().parse::<f32>().unwrap_or_else(|_| {
                        eprintln!("--entry-ladder: bad number {t:?}");
                        std::process::exit(2);
                    }))
                    .collect();
            }
            "--calib" => calib = Some(PathBuf::from(argv.next().unwrap_or_else(|| usage()))),
            "--optimal" => optimal = true,
            "--near" => {
                let s = argv.next().unwrap_or_else(|| usage());
                let values: Vec<f32> = s
                    .split(',')
                    .map(|t| t.trim().parse::<f32>().unwrap_or_else(|_| {
                        eprintln!("--near: bad number {t:?}");
                        std::process::exit(2);
                    }))
                    .collect();
                if values.len() != 4 || values.iter().any(|v| !v.is_finite()) || values[3] < 0.0 {
                    eprintln!("--near: expected x,y,z,r with r >= 0, got {s:?}");
                    std::process::exit(2);
                }
                near = Some((Vec3::new(values[0], values[1], values[2]), values[3]));
            }
            "-h" | "--help" => usage(),
            other => {
                eprintln!("unknown arg: {other}");
                usage();
            }
        }
    }

    let Some(shard) = shard else {
        eprintln!("--shard i/N is required");
        usage();
    };
    let Some((i_s, n_s)) = shard.split_once('/') else {
        eprintln!("--shard must be i/N, got {shard:?}");
        usage();
    };
    let shard_i: u32 = i_s.parse().unwrap_or_else(|_| {
        eprintln!("--shard: bad i in {shard:?}");
        std::process::exit(2);
    });
    let shard_n: u32 = n_s.parse().unwrap_or_else(|_| {
        eprintln!("--shard: bad N in {shard:?}");
        std::process::exit(2);
    });
    if shard_n == 0 || shard_i >= shard_n {
        eprintln!("--shard: need 0 <= i < N, got i={shard_i} N={shard_n}");
        std::process::exit(2);
    }
    if entry_ladder.is_empty() {
        eprintln!("--entry-ladder: at least one speed is required");
        std::process::exit(2);
    }

    Args {
        bsp: bsp.unwrap_or_else(|| usage()),
        shard_i,
        shard_n,
        out: out.unwrap_or_else(|| usage()),
        entry_ladder,
        calib,
        near,
        optimal,
    }
}

/// FNV-1a — deterministic, dependency-free, and NOT used as an rng seed anywhere: the search
/// below is a pure exhaustive lattice scan. This hash only stamps a stable provenance
/// fingerprint of the invocation into the output metadata (`seed`), so two runs that used
/// identical args/inputs can be spotted at a glance without diffing the whole file.
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                write!(out, "\\u{:04x}", c as u32).unwrap();
            }
            c => out.push(c),
        }
    }
    out
}

fn main() {
    let args = parse_args();

    let bytes = std::fs::read(&args.bsp).unwrap_or_else(|e| {
        eprintln!("read {:?}: {e}", args.bsp);
        std::process::exit(1);
    });

    // Same physics knobs and double_jump=false (mirrors the live green config, rtx_doublejump 0)
    // as crates/rtx-nav/tests/dm3_ra_curl_coverage_probe.rs, and the same build_navmesh entry
    // point production/tests use — no bespoke loading path.
    let params = SpeedJumpParams { gravity: 800.0, accel: 10.0, maxspeed: 320.0, friction: 4.0, stopspeed: 100.0, curl: true };
    let Some(bsp) = Bsp::parse(&bytes) else {
        eprintln!("failed to parse BSP {:?}", args.bsp);
        std::process::exit(1);
    };
    let graph = build_navmesh(&bsp, vec![], vec![], vec![], None, false, Some(params), None);

    // Stable modulo-N shard over the ledge-cell space, ascending cell-id order.
    let n_cells = graph.cells.len() as CellId;
    let ledges: Vec<CellId> = (0..n_cells)
        .filter(|&id| id % args.shard_n == args.shard_i)
        .filter(|&id| {
            args.near
                .is_none_or(|(center, radius)| (graph.cells[id as usize].origin - center).length() <= radius)
        })
        .collect();

    let mut candidates: Vec<(Link, SpeedJumpTraversal)> = Vec::new();
    for &ledge in &ledges {
        let mut out = Vec::new();
        if args.optimal {
            graph.solve_chained_ground_turn_optimal_curl(&bsp, ledge, params, &args.entry_ladder, &mut out);
        } else {
            graph.solve_chained_ground_turn_from(&bsp, ledge, params, &args.entry_ladder, &mut out);
        }
        candidates.extend(out);
    }

    // Deterministic final order independent of the certifier's internal traversal order: sort by
    // (to, from, cost) with f32::total_cmp so no NaN/partial-order surprise can affect the sort.
    candidates.sort_by(|(la, _), (lb, _)| la.to.cmp(&lb.to).then(la.from.cmp(&lb.from)).then(la.cost.total_cmp(&lb.cost)));

    let seed = fnv1a(
        format!(
            "{}|{}/{}|{:?}|near={:?}|optimal={}|bsp_bytes={}",
            GT_SEARCH_VERSION,
            args.shard_i,
            args.shard_n,
            args.entry_ladder,
            args.near,
            args.optimal,
            bytes.len()
        )
        .as_bytes(),
    );

    let json = render_json(&args, &candidates, bytes.len(), &params, seed);
    std::fs::write(&args.out, &json).unwrap_or_else(|e| {
        eprintln!("write {:?}: {e}", args.out);
        std::process::exit(1);
    });

    eprintln!(
        "gt-search: shard {}/{} ledges={} candidates={} entry_ladder={:?} -> {}",
        args.shard_i,
        args.shard_n,
        ledges.len(),
        candidates.len(),
        args.entry_ladder,
        args.out.display()
    );
}

fn render_json(args: &Args, candidates: &[(Link, SpeedJumpTraversal)], bsp_bytes: usize, params: &SpeedJumpParams, seed: u64) -> String {
    let mut s = String::new();
    s.push_str("{\n");
    write!(s, "  \"version\": \"{}\",\n", GT_SEARCH_VERSION).unwrap();
    write!(s, "  \"shard\": \"{}/{}\",\n", args.shard_i, args.shard_n).unwrap();
    write!(s, "  \"optimal\": {},\n", args.optimal).unwrap();
    write!(s, "  \"seed\": {},\n", seed).unwrap();
    s.push_str("  \"params\": {\n");
    write!(s, "    \"gravity\": {},\n", params.gravity).unwrap();
    write!(s, "    \"accel\": {},\n", params.accel).unwrap();
    write!(s, "    \"maxspeed\": {},\n", params.maxspeed).unwrap();
    write!(s, "    \"friction\": {},\n", params.friction).unwrap();
    write!(s, "    \"stopspeed\": {},\n", params.stopspeed).unwrap();
    write!(s, "    \"curl\": {},\n", params.curl).unwrap();
    write!(s, "    \"double_jump\": false\n").unwrap();
    s.push_str("  },\n");
    write!(
        s,
        "  \"entry_ladder\": [{}],\n",
        args.entry_ladder.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ")
    )
    .unwrap();
    match args.near {
        Some((p, r)) => write!(s, "  \"near\": [{}, {}, {}, {}],\n", p.x, p.y, p.z, r).unwrap(),
        None => s.push_str("  \"near\": null,\n"),
    }
    write!(s, "  \"bsp_path\": \"{}\",\n", json_escape(&args.bsp.display().to_string())).unwrap();
    write!(s, "  \"bsp_bytes\": {},\n", bsp_bytes).unwrap();
    match &args.calib {
        Some(p) => write!(s, "  \"calib_path\": \"{}\",\n", json_escape(&p.display().to_string())).unwrap(),
        None => s.push_str("  \"calib_path\": null,\n"),
    }
    write!(s, "  \"candidate_count\": {},\n", candidates.len()).unwrap();
    s.push_str("  \"candidates\": [\n");
    for (idx, (link, tr)) in candidates.iter().enumerate() {
        s.push_str("    {\n");
        write!(s, "      \"from\": {},\n", link.from).unwrap();
        write!(s, "      \"to\": {},\n", link.to).unwrap();
        write!(s, "      \"link_kind\": \"{:?}\",\n", link.kind).unwrap();
        write!(s, "      \"link_cost\": {},\n", link.cost).unwrap();
        // For every chained ground-turn candidate, link.cost IS the certified worst-case elapsed
        // (see the comment at the emission site in jumps.rs: "no commit padding on top"); stamped
        // again under its own name per the campaign plan's output contract.
        write!(s, "      \"worst_elapsed\": {},\n", link.cost).unwrap();
        s.push_str("      \"traversal\": {\n");
        write!(s, "        \"takeoff\": [{}, {}, {}],\n", tr.takeoff.x, tr.takeoff.y, tr.takeoff.z).unwrap();
        write!(s, "        \"v_req\": {},\n", tr.v_req).unwrap();
        write!(s, "        \"airtime\": {},\n", tr.airtime).unwrap();
        write!(s, "        \"landing_speed_lo\": {},\n", tr.landing_speed_lo).unwrap();
        write!(s, "        \"chained\": {},\n", tr.chained).unwrap();
        write!(s, "        \"curl_gain\": {},\n", tr.curl_gain).unwrap();
        write!(s, "        \"curl_entry_aim\": [{}, {}, {}],\n", tr.curl_entry_aim.x, tr.curl_entry_aim.y, tr.curl_entry_aim.z).unwrap();
        write!(s, "        \"curl_switch_dist\": {},\n", tr.curl_switch_dist).unwrap();
        write!(
            s,
            "        \"curl_landing_aim\": [{}, {}, {}]\n",
            tr.curl_landing_aim.x, tr.curl_landing_aim.y, tr.curl_landing_aim.z
        )
        .unwrap();
        s.push_str("      },\n");
        match &tr.ground_turn {
            Some(gt) => {
                s.push_str("      \"ground_turn\": {\n");
                write!(s, "        \"version\": {},\n", gt.version).unwrap();
                write!(s, "        \"runway_aim\": [{}, {}, {}],\n", gt.runway_aim.x, gt.runway_aim.y, gt.runway_aim.z).unwrap();
                write!(s, "        \"blended_runway\": {},\n", gt.blended_runway).unwrap();
                write!(s, "        \"runway_yaw\": {},\n", gt.runway_yaw).unwrap();
                write!(s, "        \"lip_reach\": {},\n", gt.lip_reach).unwrap();
                write!(s, "        \"hold_speed\": {},\n", gt.hold_speed).unwrap();
                write!(s, "        \"turn_dist\": {},\n", gt.turn_dist).unwrap();
                write!(s, "        \"launch_yaw\": {},\n", gt.launch_yaw).unwrap();
                write!(s, "        \"yaw_min\": {},\n", gt.yaw_min).unwrap();
                write!(s, "        \"box_min\": [{}, {}, {}],\n", gt.box_min.x, gt.box_min.y, gt.box_min.z).unwrap();
                write!(s, "        \"box_max\": [{}, {}, {}],\n", gt.box_max.x, gt.box_max.y, gt.box_max.z).unwrap();
                write!(s, "        \"launch_gain\": {},\n", gt.launch_gain).unwrap();
                write!(s, "        \"hold_aim\": [{}, {}, {}],\n", gt.hold_aim.x, gt.hold_aim.y, gt.hold_aim.z).unwrap();
                write!(s, "        \"gate_point\": [{}, {}, {}],\n", gt.gate_point.x, gt.gate_point.y, gt.gate_point.z).unwrap();
                write!(s, "        \"gate_normal\": [{}, {}, {}],\n", gt.gate_normal.x, gt.gate_normal.y, gt.gate_normal.z).unwrap();
                write!(s, "        \"air_gain\": {},\n", gt.air_gain).unwrap();
                write!(s, "        \"landing_aim\": [{}, {}, {}],\n", gt.landing_aim.x, gt.landing_aim.y, gt.landing_aim.z).unwrap();
                write!(s, "        \"entry_speed_lo\": {},\n", gt.entry_speed_lo).unwrap();
                write!(s, "        \"entry_speed_hi\": {},\n", gt.entry_speed_hi).unwrap();
                write!(s, "        \"entry_yaw_lo\": {},\n", gt.entry_yaw_lo).unwrap();
                write!(s, "        \"entry_yaw_hi\": {},\n", gt.entry_yaw_hi).unwrap();
                write!(s, "        \"landing_speed_lo\": {},\n", gt.landing_speed_lo).unwrap();
                write!(s, "        \"landing_yaw\": {}\n", gt.landing_yaw).unwrap();
                s.push_str("      }\n");
            }
            None => s.push_str("      \"ground_turn\": null\n"),
        }
        s.push_str(if idx + 1 == candidates.len() { "    }\n" } else { "    },\n" });
    }
    s.push_str("  ]\n");
    s.push_str("}\n");
    s
}
