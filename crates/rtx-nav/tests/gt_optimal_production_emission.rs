// SPDX-License-Identifier: AGPL-3.0-or-later

//! Production emission + runtime-reproduction contract for the optimal-sweep
//! ground-turn curl (`GROUND_TURN_OPTIMAL_VERSION`, v3).
//!
//! Part A of the wiring change makes `add_speed_jumps` emit v3 contracts in the
//! *production* build path (no cvar gate on this branch). This test proves, on
//! `dm3`, that:
//!   1. the production build emits at least one v3 SpeedJump link;
//!   2. a specific certified link near the upper gap is present — the gt-search
//!      candidate from-cell ~(320..384, -576, ~56..72), landing near
//!      (160,-832,16), v_req 360, landing_speed_lo ~= 462;
//!   3. the public runtime steering functions (the ones steer.rs dispatches to
//!      on v3) reproduce the certified rollout: grounded optimal sweep ->
//!      `ground_turn_should_launch_optimal` -> launch along `launch_yaw` -> air
//!      curl, landing back on the contract's target cell.
//!
//! Assertion (3) is the point of the whole change: the flown law must equal the
//! proven law. It re-implements `ground_turn_rolls_optimal_tol`'s center-lattice
//! rollout (mid dt = 0.020, mid entry speed = v_req, mid entry yaw = the
//! corridor heading `runway_yaw`) using only the crate's public functions.

use glam::{Vec3, Vec3Swizzles};
use rtx_nav::navmesh::{
    build_navmesh, ground_turn_air_cmd, ground_turn_ground_cmd_optimal, ground_turn_launch_cmd,
    ground_turn_should_launch_optimal, CellId, GroundTurnCurl, LinkKind, NavGraph, SpeedJumpParams,
    GROUND_TURN_OPTIMAL_VERSION,
};
use rtx_nav::pmove::{pm_step_report, PmParams, PmState};

const BSP_PATH: &str = "/mnt/c/Users/benya/projects/quakeworld/mvd_analyzer/bsps/dm3.bsp";

fn prod_params() -> SpeedJumpParams {
    // Exactly the gt-search / green-config build parameters.
    SpeedJumpParams {
        gravity: 800.0,
        accel: 10.0,
        maxspeed: 320.0,
        friction: 4.0,
        stopspeed: 100.0,
        curl: true,
    }
}

/// Brute-force reimplementation of `NavGraph::nearest_within` (private): the
/// nearest cell to `p` within `horiz` XY and `vert` Z, or `None`.
fn nearest_within(graph: &NavGraph, p: Vec3, horiz: f32, vert: f32) -> Option<CellId> {
    graph
        .cells
        .iter()
        .enumerate()
        .filter(|(_, c)| (c.origin.xy() - p.xy()).length() <= horiz && (c.origin.z - p.z).abs() <= vert)
        .min_by(|a, b| {
            (a.1.origin - p)
                .length_squared()
                .total_cmp(&(b.1.origin - p).length_squared())
        })
        .map(|(id, _)| id as CellId)
}

/// Reproduce `ground_turn_rolls_optimal_tol`'s center rollout with the public
/// runtime steering functions steer.rs dispatches on for a v3 contract. Returns
/// the resolved landing cell (nearest within 24u XY / 2u Z of the touchdown), or
/// `None` if the trajectory scrapes a wall or never lands.
fn runtime_rollout(graph: &NavGraph, bsp: &rtx_nav::bsp::Bsp, from: CellId, gt: &GroundTurnCurl, v_req: f32) -> Option<CellId> {
    let p = PmParams {
        gravity: 800.0,
        accel: 10.0,
        friction: 4.0,
        stopspeed: 100.0,
        maxspeed: 320.0,
    };
    let dt = 0.020_f32; // GT_DT_CLASSES[1], the certification's mid class.
    // Entry = the solver's center lattice point: from-cell origin (+ the tiny z
    // nudge it uses), carried at v_req along the corridor heading runway_yaw.
    let (sy, cy) = gt.runway_yaw.to_radians().sin_cos();
    let entry_origin = graph.cells[from as usize].origin + Vec3::new(0.0, 0.0, 0.03125);
    let mut s = PmState {
        origin: entry_origin,
        vel: Vec3::new(v_req * cy, v_req * sy, 0.0),
        on_ground: true,
        jump_held: false,
    };
    // Grounded optimal sweep until the launch gate fires (setup tick cap 45).
    let mut setup = 0usize;
    loop {
        if ground_turn_should_launch_optimal(s.origin, s.vel.xy(), s.on_ground, gt) {
            break;
        }
        if setup >= 45 {
            return None;
        }
        let cmd = ground_turn_ground_cmd_optimal(s.vel.xy(), gt, p.accel, p.maxspeed, dt);
        let rep = pm_step_report(bsp, &mut s, &cmd, &p, dt);
        if rep.wall_contact {
            return None;
        }
        setup += 1;
        if s.jump_held {
            return None;
        }
    }
    // Launch tick: aim along launch_yaw (as steer.rs does for v3).
    let cmd = ground_turn_launch_cmd(s.vel.xy(), gt.launch_yaw, gt, p.accel, p.maxspeed, dt);
    let rep = pm_step_report(bsp, &mut s, &cmd, &p, dt);
    if rep.wall_contact || s.on_ground {
        return None;
    }
    // Air curl to first touchdown (flight tick cap 60).
    for _ in 0..60 {
        let cmd = ground_turn_air_cmd(s.origin, s.vel.xy(), gt, p.accel, p.maxspeed, dt);
        let rep = pm_step_report(bsp, &mut s, &cmd, &p, dt);
        if rep.wall_contact {
            return None;
        }
        if s.on_ground {
            return nearest_within(graph, s.origin, 24.0, 2.0);
        }
    }
    None
}

#[test]
fn gt_optimal_production_emission() {
    let bytes = std::fs::read(BSP_PATH).unwrap_or_else(|e| panic!("read dm3.bsp at {BSP_PATH}: {e}"));
    let build = build_navmesh(bytes, vec![], vec![], vec![], None, false, Some(prod_params()), None)
        .expect("build dm3 navmesh");
    let (bsp, graph) = (&build.0, &build.1);

    // (1) The production path emits v3 contracts.
    let mut v3_links: Vec<(usize, CellId, CellId, f32, GroundTurnCurl)> = Vec::new();
    for (li, link) in graph.links.iter().enumerate() {
        if link.kind != LinkKind::SpeedJump {
            continue;
        }
        let Some(tr) = graph.speed_jump_of_link(li as u32) else { continue };
        let Some(gt) = tr.ground_turn else { continue };
        if gt.version == GROUND_TURN_OPTIMAL_VERSION {
            v3_links.push((li, link.from, link.to, tr.v_req, gt));
        }
    }
    eprintln!("dm3 v3 (optimal-sweep) SpeedJump links emitted: {}", v3_links.len());
    assert!(
        !v3_links.is_empty(),
        "production build emitted no GROUND_TURN_OPTIMAL_VERSION links on dm3"
    );

    // (2) The certified upper-gap link: from ~(320..384,-576,~56..72), landing
    // near (160,-832,16), v_req 360, landing_speed_lo ~= 462 (the gt-search
    // candidate). Two v3 links land on that cell; land_lo ~= 462 (vs the other's
    // ~437) plus the from-cell pin it uniquely.
    let to_target = Vec3::new(160.0, -832.0, 16.0);
    let from_ref = Vec3::new(320.0, -576.0, 72.0);
    let upper = v3_links
        .iter()
        .filter(|(_, from, to, v_req, gt)| {
            let to_o = graph.cells[*to as usize].origin;
            let from_o = graph.cells[*from as usize].origin;
            (to_o.xy() - to_target.xy()).length() <= 24.0
                && (to_o.z - to_target.z).abs() <= 8.0
                && (*v_req - 360.0).abs() <= 0.5
                && (from_o - from_ref).length() <= 120.0
                && (gt.landing_speed_lo - 462.0).abs() <= 8.0
        })
        .min_by(|a, b| {
            let da = (graph.cells[a.1 as usize].origin - from_ref).length();
            let db = (graph.cells[b.1 as usize].origin - from_ref).length();
            da.total_cmp(&db)
        })
        .copied();
    let Some((li, from, to, v_req, gt)) = upper else {
        // Fail with the full v3 upper-region listing rather than weakening.
        for (li, from, to, v_req, gt) in &v3_links {
            let fo = graph.cells[*from as usize].origin;
            let to_o = graph.cells[*to as usize].origin;
            if (to_o.xy() - to_target.xy()).length() <= 64.0 {
                eprintln!(
                    "  v3 {from}->{to} from({:.0},{:.0},{:.0}) to({:.0},{:.0},{:.0}) v_req={v_req:.1} land_lo={:.1} li={li}",
                    fo.x, fo.y, fo.z, to_o.x, to_o.y, to_o.z, gt.landing_speed_lo
                );
            }
        }
        panic!("no certified upper-gap v3 link near {to_target:?} (v_req 360, land_lo ~=462) found");
    };
    let from_o = graph.cells[from as usize].origin;
    let to_o = graph.cells[to as usize].origin;
    eprintln!(
        "UPPER-GAP v3 link: {from}->{to} from({:.0},{:.0},{:.0}) to({:.0},{:.0},{:.0}) v_req={v_req:.1} \
         landing_speed_lo={:.1} cost={:.3} launch_yaw={:.1} runway_yaw={:.1} entry[{:.0}..{:.0}] li={li}",
        from_o.x, from_o.y, from_o.z, to_o.x, to_o.y, to_o.z, gt.landing_speed_lo,
        graph.links[li].cost, gt.launch_yaw, gt.runway_yaw, gt.entry_speed_lo, gt.entry_speed_hi
    );

    // (3) The public runtime steering path reproduces the certified rollout: it
    // must land back on the contract's own target cell.
    let landed = runtime_rollout(graph, bsp, from, &gt, v_req);
    eprintln!("runtime rollout landed on cell {landed:?} (contract target {to})");
    assert_eq!(
        landed,
        Some(to),
        "runtime steering rollout did not reproduce the certified landing on cell {to} \
         (from {from}, v_req {v_req}, launch_yaw {:.1})",
        gt.launch_yaw
    );
}
