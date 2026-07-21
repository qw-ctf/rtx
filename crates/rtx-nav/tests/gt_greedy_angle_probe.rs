// SPDX-License-Identifier: AGPL-3.0-or-later

//! Diagnostic C2 — is the certifier's angle scheme the strangler?
//!
//! Same rig as gt_ground_phase_calib.rs test B (upper platform, standing origin
//! at (256,-576), start yaw 177, 240 ms in 13 ms ticks, human-like max
//! forward+side wish) — but instead of the fixed linear bearing sweep
//! (177 -> 245, ~68 degrees), each tick **greedily** picks the wish bearing that
//! maximises next-tick horizontal speed. Candidates are `velocity_yaw + offset`
//! for `offset` in 0..=70 degrees in 1-degree steps (the same increasing-yaw
//! sweep direction as the humans). We clone the pmove state, step the clone with
//! each candidate, keep the fastest, and apply it to the real state.
//!
//! If greedy reaches roughly the human exits (~445 from 358) the physics can
//! build the speed and the linear bearing policy is the limiter (diagnosis b);
//! if greedy also saturates near ~400 the ceiling is in the ground physics
//! itself (diagnosis a). Human aggregates are reference constants only.
//!
//! Wish construction: a candidate bearing `B` is emitted as max forward+side
//! (`forward = side = MOVE_SPEED`) with `view_yaw = B + 45`, so `wishdir` points
//! exactly at `B` (with `side_sign = +1`, `wishdir_yaw = view_yaw − 45`), and
//! `wishspeed` clamps to sv_maxspeed = 320 — matching the calib's construction.

use glam::{Vec2, Vec3, Vec3Swizzles};
use rtx_nav::bsp::Bsp;
use rtx_nav::pmove::{pm_step_report, PmParams, PmState};
use rtx_nav::strafe::{Cmd, MOVE_SPEED};

const ENTRY_SPEEDS: [f32; 3] = [332.0, 358.0, 414.0];
const HUMAN_EXITS: [f32; 3] = [419.4103, 444.59644, 460.8709];
const START_YAW: f32 = 177.0;
const HUMAN_SWEEP_DEG: f32 = 68.0;
const DURATION_MS: u32 = 240;
const MSEC_CLASS: u32 = 13;
const MAX_OFFSET_DEG: u32 = 70;

fn standing_origin(bsp: &Bsp, xy: Vec2) -> Vec3 {
    let top = Vec3::new(xy.x, xy.y, 160.0);
    let bottom = Vec3::new(xy.x, xy.y, 80.0);
    let trace = bsp.hull1_trace(top, bottom);
    assert!(!trace.start_solid && !trace.all_solid, "upper-platform floor trace started solid");
    assert!(trace.fraction < 1.0, "no upper-platform floor below {xy:?}");
    assert!(trace.plane_normal.z >= 0.7, "trace hit a non-floor plane: {trace:?}");
    trace.endpos + Vec3::Z * 0.03125
}

fn yaw_deg(v: Vec2) -> f32 {
    v.y.atan2(v.x).to_degrees()
}

/// A max forward+side wish whose wish direction points at world bearing `B`.
fn cmd_at_bearing(bearing: f32) -> Cmd {
    Cmd {
        view_yaw: bearing + 45.0,
        forward: MOVE_SPEED,
        side: MOVE_SPEED,
        jump: false,
    }
}

struct Outcome {
    exit_speed: f32,
    total_heading_deg: f32,
    grounded_throughout: bool,
}

fn greedy_rollout(bsp: &Bsp, entry_speed: f32, verbose: bool) -> Outcome {
    let origin = standing_origin(bsp, Vec2::new(256.0, -576.0));
    let (sy, cy) = START_YAW.to_radians().sin_cos();
    let mut state = PmState {
        origin,
        vel: Vec3::new(entry_speed * cy, entry_speed * sy, 0.0),
        on_ground: true,
        jump_held: false,
    };
    let params = PmParams::default();
    let mut elapsed_ms = 0u32;
    let mut grounded_throughout = true;
    let mut cumulative_heading = 0.0f32;
    let mut prev_yaw = yaw_deg(state.vel.xy());
    let mut tick_idx = 0u32;

    if verbose {
        eprintln!("  t_ms\toffset\tspeed\theading\tgrounded");
    }

    while elapsed_ms < DURATION_MS {
        let tick_ms = MSEC_CLASS.min(DURATION_MS - elapsed_ms);
        let dt = tick_ms as f32 / 1000.0;
        let vel_yaw = yaw_deg(state.vel.xy());

        // Greedy: probe every candidate offset on a throwaway clone, keep fastest.
        let mut best_offset = 0u32;
        let mut best_speed = f32::NEG_INFINITY;
        for offset in 0..=MAX_OFFSET_DEG {
            let cmd = cmd_at_bearing(vel_yaw + offset as f32);
            let mut probe = state; // Copy
            let _ = pm_step_report(bsp, &mut probe, &cmd, &params, dt);
            let sp = probe.vel.xy().length();
            if sp > best_speed {
                best_speed = sp;
                best_offset = offset;
            }
        }

        // Apply the winning candidate to the real state.
        let cmd = cmd_at_bearing(vel_yaw + best_offset as f32);
        let _ = pm_step_report(bsp, &mut state, &cmd, &params, dt);
        grounded_throughout &= state.on_ground;

        let new_yaw = yaw_deg(state.vel.xy());
        // Accumulate signed heading change across the wrap-safe short arc.
        let mut d = new_yaw - prev_yaw;
        while d > 180.0 {
            d -= 360.0;
        }
        while d < -180.0 {
            d += 360.0;
        }
        cumulative_heading += d;
        prev_yaw = new_yaw;

        elapsed_ms += tick_ms;
        tick_idx += 1;
        if verbose && (tick_idx % 3 == 0 || elapsed_ms >= DURATION_MS) {
            eprintln!(
                "  {elapsed_ms}\t{best_offset}\t{:.3}\t{new_yaw:.2}\t{}",
                state.vel.xy().length(),
                state.on_ground
            );
        }
    }

    Outcome {
        exit_speed: state.vel.xy().length(),
        total_heading_deg: cumulative_heading,
        grounded_throughout,
    }
}

#[test]
fn greedy_angle_probe() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(path).expect("read BSP");
    let bsp = Bsp::parse(&bytes).expect("parse BSP");

    eprintln!(
        "Greedy per-tick wish-bearing probe (offset 0..={MAX_OFFSET_DEG} deg), \
         human linear sweep = {HUMAN_SWEEP_DEG} deg over {DURATION_MS} ms\n"
    );
    eprintln!("entry_u/s\texit_u/s\thuman_exit\tdelta_%\theading_chg_deg\tgrounded");
    for (&entry, &human_exit) in ENTRY_SPEEDS.iter().zip(HUMAN_EXITS.iter()) {
        eprintln!("-- entry {entry:.0} --");
        let out = greedy_rollout(&bsp, entry, true);
        let delta_pct = (out.exit_speed - human_exit) / human_exit * 100.0;
        eprintln!(
            "{entry:.0}\t\t{:.3}\t{human_exit:.3}\t{delta_pct:+.2}\t{:.2}\t\t{}\n",
            out.exit_speed, out.total_heading_deg, out.grounded_throughout
        );
        assert!(
            out.exit_speed > entry * 0.9,
            "greedy lost implausibly much speed: {entry:.1} -> {:.1}",
            out.exit_speed
        );
    }
}
