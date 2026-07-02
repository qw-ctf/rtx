// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bunnyhop — the QuakeWorld air-strafe controller for bots. Chaining jumps (so ground friction
//! never bites) while **air-strafing** lets a player accelerate far past `sv_maxspeed`: QW's air
//! acceleration clamps the *projected* wish speed to a small cap (~30 ups), so when the wish
//! direction is held roughly perpendicular to the velocity — one strafe key, view swept to keep the
//! angle — the speed grows every frame without bound. This module is the pure math of that: pick the
//! optimal wish angle, turn it into a usercmd view/side, and weave the heading toward the waypoint.
//! The engine runs the actual `PM_PlayerMove`; the bot only emits the usercmd (`crate::bot`).

use glam::Vec2;

use crate::bot::wrap180;

/// The projected-wishspeed cap in QW air acceleration (`PM_AirAccelerate`) — an engine literal, not
/// a cvar. Only this much of the wish speed counts against the current velocity each tick, which is
/// exactly what lets a perpendicular strafe keep gaining.
const AIR_CAP: f32 = 30.0;
/// Usercmd move scale (as in `bot.rs`; pmove clamps wish speed to `sv_maxspeed`).
const MOVE_SPEED: f32 = 800.0;
/// Heading deadband (degrees): the strafe sign flips only once the bearing error crosses this far to
/// the *other* side, giving a gentle weave around the target bearing instead of per-frame flapping.
const HEADING_DEAD: f32 = 3.0;

/// The most air speed a single tick can add along the wish direction: `accel · maxspeed · dt`. At any
/// sane tickrate this exceeds [`AIR_CAP`], putting the optimum at a perpendicular strafe.
pub fn air_accel_max(accel: f32, maxspeed: f32, dt: f32) -> f32 {
    accel * maxspeed * dt
}

/// The wish angle off the velocity (degrees) that maximizes the per-tick speed gain, from the
/// air-accel geometry: the gain² is `2·u·a + a²` with `u = s·cosθ` and `a = min(a_max, cap − u)`.
/// When `a_max ≥ cap` (the usual case) the optimum is `u = 0` → **90°, perpendicular**; otherwise
/// it's `u = cap − a_max`. One formula covers both and degrades gracefully at coarse tickrates.
pub fn theta_star(speed: f32, a_max: f32) -> f32 {
    let u_star = (AIR_CAP - a_max.min(AIR_CAP)).max(0.0);
    (u_star / speed.max(1.0)).clamp(0.0, 1.0).acos().to_degrees()
}

/// The air-strafe usercmd for one frame.
#[derive(Clone, Copy, Debug)]
pub struct Strafe {
    /// View yaw to send (degrees).
    pub view_yaw: f32,
    /// `side` move component (± [`MOVE_SPEED`]); `forward` is always 0 in a single-key strafe.
    pub side: f32,
    /// The strafe sign chosen this frame (±1), to carry into the next as sticky state.
    pub sigma: f32,
}

/// Compute the air-strafe: aim the view so a single held strafe key puts the wish direction at the
/// speed-optimal angle off the current velocity, and choose the strafe side to weave the heading
/// toward `wp_bearing`. `prev_sigma` is last frame's side (`0` on the first frame).
///
/// The velocity curves toward the strafe side, so `sigma = sign(bearing error)`, held across the
/// deadband and flipped only once the error overshoots to the other side — an S-curve whose average
/// heading is the waypoint bearing. With `forward = 0` and `side = −sigma·MOVE`, the engine's
/// `right` vector places the wish direction at `view_yaw ± 90°`, so `view_yaw = vel_yaw + sigma·(θ*−90)`.
pub fn strafe(v_xy: Vec2, wp_bearing: f32, prev_sigma: f32, a_max: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = v_xy.y.atan2(v_xy.x).to_degrees();
    let err = wrap180(wp_bearing - vel_yaw);
    let sigma = if prev_sigma == 0.0 {
        if err >= 0.0 {
            1.0
        } else {
            -1.0
        }
    } else if err * prev_sigma < -HEADING_DEAD {
        -prev_sigma // overshot the other way — flip and weave back
    } else {
        prev_sigma // keep curving the same way
    };
    let theta = theta_star(speed, a_max);
    Strafe {
        view_yaw: wrap180(vel_yaw + sigma * (theta - 90.0)),
        side: -sigma * MOVE_SPEED,
        sigma,
    }
}

/// A faithful one-tick QuakeWorld `PM_AirAccelerate`: the wish speed's projection onto the velocity
/// is capped at [`AIR_CAP`], and `accel·wishspeed·dt` (uncapped `wishspeed`) is added along
/// `wishdir`. Used as the unit-test oracle for the controller (and to document the model the live
/// engine implements — the engine, not this module, applies it at runtime).
#[allow(dead_code)]
pub fn apply_airaccel(v: Vec2, wishdir: Vec2, wishspeed: f32, accel: f32, dt: f32) -> Vec2 {
    let addspeed = wishspeed.min(AIR_CAP) - v.dot(wishdir);
    if addspeed <= 0.0 {
        return v;
    }
    let accelspeed = (accel * wishspeed * dt).min(addspeed);
    v + wishdir * accelspeed
}

/// The wish direction the engine derives from a [`Strafe`]'s `view_yaw`/`side` (`forward = 0`): the
/// `right` vector `(sin yaw, −cos yaw)` scaled by `side`, normalized. Exposed for tests and to make
/// the view↔wishdir geometry explicit.
#[allow(dead_code)]
pub fn wishdir_of(view_yaw: f32, side: f32) -> Vec2 {
    let (sy, cy) = view_yaw.to_radians().sin_cos();
    (Vec2::new(sy, -cy) * side).normalize_or_zero()
}

#[cfg(test)]
mod tests {
    use super::*;

    const ACCEL: f32 = 10.0;
    const MAXSPEED: f32 = 320.0;

    #[test]
    fn theta_star_regimes() {
        // Coarse-enough tick → a_max ≥ cap → perpendicular optimum.
        let a = air_accel_max(ACCEL, MAXSPEED, 1.0 / 77.0);
        assert!(a >= AIR_CAP);
        assert!((theta_star(400.0, a) - 90.0).abs() < 0.01);
        // Tiny a_max → optimum wish angle bends forward (< 90°), and shrinks as a_max grows.
        let t_small = theta_star(400.0, 5.0);
        let t_big = theta_star(400.0, 20.0);
        assert!(t_small < 90.0 && t_big < 90.0);
        assert!(t_big > t_small, "θ* increases toward 90° as a_max grows");
    }

    #[test]
    fn strafe_output_strictly_gains_speed() {
        for &s in &[100.0f32, 320.0, 500.0, 800.0, 1500.0] {
            for &dt in &[1.0 / 77.0, 1.0 / 30.0, 1.0 / 13.0] {
                let a = air_accel_max(ACCEL, MAXSPEED, dt);
                let v = Vec2::new(s, 0.0);
                let cmd = strafe(v, 0.0, 1.0, a);
                let wd = wishdir_of(cmd.view_yaw, cmd.side);
                let v2 = apply_airaccel(v, wd, MAXSPEED, ACCEL, dt);
                assert!(
                    v2.length() > v.length(),
                    "no gain at s={s} dt={dt}: {} -> {}",
                    v.length(),
                    v2.length()
                );
            }
        }
    }

    #[test]
    fn chosen_angle_beats_offsets() {
        // The controller's yaw should give at least as much gain as small offsets from it.
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let v = Vec2::new(500.0, 0.0);
        let cmd = strafe(v, 0.0, 1.0, a);
        let best = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt).length();
        for off in [-10.0f32, -5.0, -2.0, 2.0, 5.0, 10.0] {
            let g = apply_airaccel(v, wishdir_of(cmd.view_yaw + off, cmd.side), MAXSPEED, ACCEL, dt).length();
            assert!(best + 1e-3 >= g, "offset {off} beat the chosen angle ({g} > {best})");
        }
    }

    #[test]
    fn ramps_far_past_maxspeed_and_tracks_bearing() {
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let mut v = Vec2::new(MAXSPEED, 0.0); // first hop, along +x; bearing also +x (0°)
        let mut sigma = 0.0;
        let mut flips = 0;
        for _ in 0..385 {
            let cmd = strafe(v, 0.0, sigma, a);
            if sigma != 0.0 && cmd.sigma != sigma {
                flips += 1;
            }
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt);
        }
        assert!(v.length() > 600.0, "only reached {} ups over 5s", v.length());
        let heading = v.y.atan2(v.x).to_degrees();
        assert!(heading.abs() < 8.0, "heading drifted to {heading}");
        assert!(flips > 5, "the weave should flip the strafe sign repeatedly ({flips})");
    }

    #[test]
    fn tracks_an_offset_bearing() {
        // Velocity along +x, but the waypoint is 30° to the left → heading should converge there.
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let mut v = Vec2::new(MAXSPEED, 0.0);
        let mut sigma = 0.0;
        for _ in 0..200 {
            let cmd = strafe(v, 30.0, sigma, a);
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt);
        }
        let heading = v.y.atan2(v.x).to_degrees();
        assert!((heading - 30.0).abs() < 8.0, "did not converge to 30°: {heading}");
    }
}
