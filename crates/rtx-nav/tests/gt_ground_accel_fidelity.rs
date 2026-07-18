// SPDX-License-Identifier: AGPL-3.0-or-later

//! Diagnostic C1 — one-tick ground-physics fidelity probe.
//!
//! For a grid of horizontal speeds and wish angles it drives a single
//! [`pm_step_report`] tick on DM3's flat upper platform, then reconstructs the
//! expected QuakeWorld result with raw arithmetic (an independent hand-calc of
//! `PM_Friction` followed by `PM_Accelerate`) and reports the delta. This is a
//! *report*, not a gate: it asks "does the offline pmove faithfully reproduce
//! QW ground friction + accel, and are its params the KTX defaults?" so that the
//! upper-gap curl saturation can be attributed to (a) the physics or (b) the
//! angle policy. Human aggregates are compared against, never asserted on.
//!
//! Angle convention: velocity is laid along `VEL_YAW`, and the wish is expressed
//! as **pure forward** (`side = 0`) with `view_yaw = VEL_YAW + theta`, so the
//! wish direction sits exactly `theta` degrees off the velocity. With
//! `forward = MOVE_SPEED (800) > sv_maxspeed`, `wishspeed` clamps to 320 — the
//! same clamp the model applies — while `wishdir` stays the unit view heading.

use glam::{Vec2, Vec3, Vec3Swizzles};
use rtx_nav::bsp::Bsp;
use rtx_nav::pmove::{pm_step_report, PmParams, PmState};
use rtx_nav::strafe::{Cmd, MOVE_SPEED};

const SPEEDS: [f32; 5] = [300.0, 340.0, 380.0, 420.0, 450.0];
const ANGLES_DEG: [f32; 5] = [0.0, 20.0, 37.0, 45.0, 60.0];
const VEL_YAW: f32 = 177.0;
const DT: f32 = 0.013;

fn standing_origin(bsp: &Bsp, xy: Vec2) -> Vec3 {
    // Same pattern as gt_ground_phase_calib.rs: start just above the z=152
    // standing band so the hull-1 down-trace lands on the upper-platform floor.
    let top = Vec3::new(xy.x, xy.y, 160.0);
    let bottom = Vec3::new(xy.x, xy.y, 80.0);
    let trace = bsp.hull1_trace(top, bottom);
    assert!(!trace.start_solid && !trace.all_solid, "upper-platform floor trace started solid");
    assert!(trace.fraction < 1.0, "no upper-platform floor below {xy:?}");
    assert!(trace.plane_normal.z >= 0.7, "trace hit a non-floor plane: {trace:?}");
    trace.endpos + Vec3::Z * 0.03125
}

/// Independent raw-arithmetic reconstruction of one grounded pmove tick:
/// `PM_Friction` (drop `max(speed, stopspeed)·friction·dt`) then `PM_Accelerate`
/// (`addspeed = wishspeed − dot(v, wishdir)`, capped add of `accel·wishspeed·dt`).
/// Deliberately does NOT call the strafe helpers — it recomputes the formulas by
/// hand so the model's composition is checked, not merely re-invoked.
fn handcalc_speed_after(v_xy: Vec2, view_yaw: f32, forward: f32, side: f32, p: &PmParams, dt: f32) -> f32 {
    // Friction.
    let speed = v_xy.length();
    let v_af = if speed < 1.0 {
        Vec2::ZERO
    } else {
        let control = speed.max(p.stopspeed);
        let drop = control * p.friction * dt;
        let newspeed = (speed - drop).max(0.0);
        v_xy * (newspeed / speed)
    };
    // Wish direction / speed (wishvel = forward·(cos,sin) + side·(sin,−cos)).
    let (sy, cy) = view_yaw.to_radians().sin_cos();
    let wishvel = Vec2::new(cy, sy) * forward + Vec2::new(sy, -cy) * side;
    let wishdir = wishvel.normalize_or_zero();
    let wishspeed = wishvel.length().min(p.maxspeed);
    // Accelerate.
    let addspeed = wishspeed - v_af.dot(wishdir);
    let v_final = if addspeed > 0.0 {
        let accelspeed = (p.accel * wishspeed * dt).min(addspeed);
        v_af + wishdir * accelspeed
    } else {
        v_af
    };
    v_final.length()
}

#[test]
fn ground_accel_fidelity() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(path).expect("read BSP");
    let bsp = Bsp::parse(&bytes).expect("parse BSP");

    let params = PmParams::default();
    eprintln!(
        "PmParams::default(): accel={} friction={} stopspeed={} maxspeed={} gravity={}",
        params.accel, params.friction, params.stopspeed, params.maxspeed, params.gravity
    );
    eprintln!(
        "KTX defaults        : accel=10 friction=4 stopspeed=100 maxspeed=320 gravity=800  \
         (sv_accelerate/sv_friction/sv_stopspeed/sv_maxspeed/sv_gravity)"
    );
    let params_match = params.accel == 10.0
        && params.friction == 4.0
        && params.stopspeed == 100.0
        && params.maxspeed == 320.0
        && params.gravity == 800.0;
    eprintln!("params match KTX defaults: {params_match}\n");

    eprintln!("v_u/s\ttheta\tmodel_after\thandcalc_after\tdelta_u/s\tdelta_%");
    let origin = standing_origin(&bsp, Vec2::new(256.0, -576.0));
    let (svy, cvy) = VEL_YAW.to_radians().sin_cos();

    let mut worst_pct = 0.0f32;
    let mut mismatches = 0u32;
    for &v in SPEEDS.iter() {
        for &theta in ANGLES_DEG.iter() {
            let mut state = PmState {
                origin,
                vel: Vec3::new(v * cvy, v * svy, 0.0),
                on_ground: true,
                jump_held: false,
            };
            // Pure forward, view rotated theta degrees off the velocity.
            let cmd = Cmd {
                view_yaw: VEL_YAW + theta,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: false,
            };
            let v_before = state.vel.xy();
            let _ = pm_step_report(&bsp, &mut state, &cmd, &params, DT);
            let model_after = state.vel.xy().length();
            let hand_after = handcalc_speed_after(v_before, cmd.view_yaw, cmd.forward, cmd.side, &params, DT);
            let delta = model_after - hand_after;
            let delta_pct = if hand_after.abs() > 1e-6 { delta / hand_after * 100.0 } else { 0.0 };
            worst_pct = worst_pct.max(delta_pct.abs());
            let tag = if delta_pct.abs() >= 5.0 {
                mismatches += 1;
                "  <-- FIDELITY-MISMATCH"
            } else {
                ""
            };
            eprintln!(
                "{v:.0}\t{theta:.0}\t{model_after:.4}\t\t{hand_after:.4}\t\t{delta:+.4}\t\t{delta_pct:+.4}{tag}"
            );
            // Report, not gate: only guard against NaN/Inf so the run stays green.
            assert!(model_after.is_finite() && hand_after.is_finite(), "non-finite speed");
        }
    }
    eprintln!("\nworst |delta| = {worst_pct:.4}%  cells over 5% = {mismatches}/25");
}
