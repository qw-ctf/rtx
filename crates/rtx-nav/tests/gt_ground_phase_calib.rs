// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reporting calibration probe for the upper-gap ground curl. This deliberately
//! has only a broad smoke assertion: human aggregates are evidence to compare
//! against, not inputs that should hard-code bot physics behavior.

use glam::{Vec2, Vec3, Vec3Swizzles};
use rtx_nav::bsp::Bsp;
use rtx_nav::pmove::{pm_step_report, PmParams, PmState};
use rtx_nav::strafe::{ground_prestrafe, Cmd, MOVE_SPEED};

const ENTRY_SPEEDS: [f32; 3] = [332.0, 358.0, 414.0];
const HUMAN_EXITS: [f32; 3] = [419.4103, 444.59644, 460.8709];
const START_YAW: f32 = 177.0;
const SWEEP_DEG: f32 = 68.0;
const DURATION_MS: u32 = 240;
const MSEC_CLASS: u32 = 13;

fn standing_origin(bsp: &Bsp, xy: Vec2) -> Vec3 {
    // Start just above the known z=152 standing-origin band. A higher trace
    // can begin inside DM3's low ceiling after hull-1 player-box expansion.
    let top = Vec3::new(xy.x, xy.y, 160.0);
    let bottom = Vec3::new(xy.x, xy.y, 80.0);
    let trace = bsp.hull1_trace(top, bottom);
    assert!(!trace.start_solid && !trace.all_solid, "upper-platform floor trace started solid");
    assert!(trace.fraction < 1.0, "no upper-platform floor below {xy:?}");
    assert!(trace.plane_normal.z >= 0.7, "trace hit a non-floor plane: {trace:?}");
    trace.endpos + Vec3::Z * 0.03125
}

fn rollout(bsp: &Bsp, entry_speed: f32) -> (PmState, bool, u32) {
    // One runway length behind the nominal (160,-576,152) lip keeps the
    // complete 240 ms reporting phase on the flat upper platform.
    let origin = standing_origin(bsp, Vec2::new(256.0, -576.0));
    let (sy, cy) = START_YAW.to_radians().sin_cos();
    let mut state = PmState {
        origin,
        vel: Vec3::new(entry_speed * cy, entry_speed * sy, 0.0),
        on_ground: true,
        jump_held: false,
    };
    let params = PmParams::default();
    let mut sigma = 0.0;
    let mut elapsed_ms = 0;
    let mut grounded_throughout = true;
    let mut wall_contacts = 0;

    while elapsed_ms < DURATION_MS {
        let tick_ms = MSEC_CLASS.min(DURATION_MS - elapsed_ms);
        let dt = tick_ms as f32 / 1000.0;
        let progress = (elapsed_ms + tick_ms) as f32 / DURATION_MS as f32;
        let bearing = START_YAW + SWEEP_DEG * progress;

        // Exercise the same ground oracle used by the certifier, then submit
        // the calibration's human-like max forward+side command. With +side,
        // Quake's wish direction is view_yaw - 45 degrees, so offset the view
        // to make the actual wish direction sweep exactly 177 -> 245 degrees.
        let oracle = ground_prestrafe(
            state.vel.xy(),
            bearing,
            sigma,
            params.accel * params.maxspeed * dt,
            params.maxspeed,
        );
        sigma = oracle.sigma;
        let side_sign = if oracle.side < 0.0 { -1.0 } else { 1.0 };
        let cmd = Cmd {
            view_yaw: bearing + side_sign * 45.0,
            forward: MOVE_SPEED,
            side: side_sign * MOVE_SPEED,
            jump: false,
        };
        let report = pm_step_report(bsp, &mut state, &cmd, &params, dt);
        wall_contacts += u32::from(report.wall_contact);
        grounded_throughout &= state.on_ground;
        elapsed_ms += tick_ms;
    }
    (state, grounded_throughout, wall_contacts)
}

#[test]
fn upper_gap_ground_phase_calibration() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(path).expect("read BSP");
    let bsp = Bsp::parse(&bytes).expect("parse BSP");

    eprintln!("entry_u/s\tmodel_exit_u/s\thuman_exit_u/s\tdelta_%\tgrounded\twalls");
    for (&entry, &human_exit) in ENTRY_SPEEDS.iter().zip(HUMAN_EXITS.iter()) {
        let (state, grounded, walls) = rollout(&bsp, entry);
        let exit = state.vel.xy().length();
        let delta_pct = (exit - human_exit) / human_exit * 100.0;
        eprintln!("{entry:.0}\t\t{exit:.3}\t\t{human_exit:.3}\t\t{delta_pct:+.2}\t{grounded}\t\t{walls}");
        assert!(exit > entry * 0.9, "simulation lost implausibly much speed: {entry:.1} -> {exit:.1}");
    }
}
