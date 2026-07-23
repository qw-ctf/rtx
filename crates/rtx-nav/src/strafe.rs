// SPDX-License-Identifier: AGPL-3.0-or-later

//! The pure QuakeWorld air/ground movement oracles — one-tick `PM_AirAccelerate`/`PM_Accelerate`/
//! `PM_Friction`, the wish-direction geometry, and the air-strafe angle solvers ([`strafe_rate`],
//! [`air_correct`]). Free functions of their inputs, no engine or game state. They live in rtx-nav so
//! both the offline pmove sim ([`crate::pmove`]) and the navmesh build's curl-jump certifier can drive
//! the *same* physics the bot's bhop controller flies (which re-exports these). See `crate::qphys` for
//! the shared constants (`AIR_CAP`).

use glam::Vec2;

use crate::math::{wrap180, yaw_of};
use crate::qphys::AIR_CAP;

/// Bot move-component magnitude (`forward`/`side`), matching the game's `BOT_MOVE_SPEED`. The wish
/// direction is what matters; the magnitude only has to exceed `sv_maxspeed` so the wish never clamps.
pub const MOVE_SPEED: f32 = 800.0;

/// The air-curl proportional gain (°/s per ° of heading error) [`air_correct`] uses by default.
const AIR_CORRECT_GAIN: f32 = 6.0;
/// The default air-curl gain, exposed so callers passing "no override" use the tuned value.
pub const AIR_CORRECT_GAIN_DEFAULT: f32 = AIR_CORRECT_GAIN;

/// One frame's strafe usercmd, from an air-strafe or ground prestrafe solver.
#[derive(Clone, Copy, Debug)]
pub struct Strafe {
    /// View yaw to send (degrees).
    pub view_yaw: f32,
    /// `forward` move component: the velocity-aligned share of the wish (0 for a perpendicular
    /// max-gain strafe, positive for a gentle curl, negative for an emergency max-turn carve).
    pub forward: f32,
    /// `side` move component (± [`MOVE_SPEED`]).
    pub side: f32,
    /// The strafe sign chosen this frame (±1), to carry into the next as sticky state.
    pub sigma: f32,
}

/// The usercmd a controller (or an offline rollout) wants this frame.
#[derive(Clone, Copy, Debug)]
pub struct Cmd {
    /// View yaw to send (degrees); the caller supplies pitch.
    pub view_yaw: f32,
    /// `forward` move component.
    pub forward: f32,
    /// `side` move component.
    pub side: f32,
    /// Press `BUTTON_JUMP` this frame.
    pub jump: bool,
}

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

/// Heading turn rate (deg/s) of the **maximum-speed-gain** air strafe at `speed`: a perpendicular
/// wish adds `min(a_max, cap)` ups sideways per tick, rotating velocity by `atan(that/speed)`.
/// This is deliberately not the absolute maximum turn rate; [`max_turn_strafe`] spends a little
/// speed to turn harder when collision avoidance matters more than acceleration.
pub fn omega_gain_max(speed: f32, a_max: f32, dt: f32) -> f32 {
    let a = a_max.min(AIR_CAP);
    (a / speed.max(1.0)).atan().to_degrees() / dt.max(1e-4)
}

/// The maximum non-reversing heading-change air command. In the normal `a_max < speed` regime, with
/// wish projection `u` onto velocity and a full acceleration vector of length `a_max`, maximizing
/// the resultant angle gives `u = −a_max`, or `theta = acos(−a_max/speed)`: slightly behind
/// perpendicular. The resulting turn is `asin(a_max/speed)`, versus the max-gain
/// `atan(min(a_max, cap)/speed)`, while speed falls only to `sqrt(speed² − a_max²)`.
///
/// At unusually low speed/coarse ticks (`a_max >= speed`), solve the wish projection that leaves
/// zero forward velocity, then remain infinitesimally on its forward side. This approaches a 90°
/// carve without ever making emergency avoidance issue a reverse/U-turn command.
pub fn max_turn_strafe(v_xy: Vec2, sigma: f32, a_max: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = yaw_of(v_xy);
    let projection = if a_max < speed {
        -a_max
    } else {
        // In the full-acceleration region, `x' = s + A·u/s = 0` at `u = -s²/A`.
        // If that lies beyond the cap boundary, solve `s + (cap-u)·u/s = 0` instead.
        let full_stop = -(speed * speed) / a_max.max(1e-4);
        let cap_stop = (AIR_CAP - (AIR_CAP * AIR_CAP + 4.0 * speed * speed).sqrt()) * 0.5;
        let stopped = if full_stop <= AIR_CAP - a_max {
            full_stop
        } else {
            cap_stop
        };
        stopped + speed * 1e-5
    };
    let theta = (projection / speed).clamp(-1.0, 1.0).acos();
    Strafe {
        view_yaw: vel_yaw,
        forward: MOVE_SPEED * theta.cos(),
        side: -sigma * MOVE_SPEED * theta.sin(),
        sigma,
    }
}

/// An air-strafe that turns the velocity at a *chosen* rate `omega_deg` (deg/s) rather than the
/// speed-optimal maximum — the smooth-lobe primitive. The turn rate is set by how much sideways
/// speed the tick adds: `a_need = speed·tan(ω·dt)`. Angling the wish so its projection onto the
/// velocity is `cap − a_need` (i.e. `θ = acos((cap − a_need)/speed)`, forward of perpendicular)
/// delivers exactly that sideways add while the parallel component still grows the speed. When the
/// requested rate meets or exceeds what max-gain geometry can deliver, fall back to the max-gain
/// [`theta_star`] angle (perpendicular). `sigma` is the strafe side (±1).
///
/// The wish (world direction `vel_yaw + sigma·θ`) is expressed with the **view riding the velocity**
/// and the angle carried in `forward`/`side` (`MOVE·cosθ`, `−sigma·MOVE·sinθ`) rather than offsetting
/// the view by `sigma·(θ−90)` with a single strafe key. Same wishdir, but the strafe-side flip no
/// longer *jumps* the view yaw (the eyes just sweep with the velocity), so the gait doesn't twitch.
pub fn strafe_rate(v_xy: Vec2, sigma: f32, omega_deg: f32, a_max: f32, dt: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = yaw_of(v_xy);
    let cap = a_max.min(AIR_CAP);
    let a_need = speed * (omega_deg.to_radians() * dt).tan();
    let theta = if a_need >= cap {
        theta_star(speed, a_max)
    } else {
        ((AIR_CAP - a_need).max(0.0) / speed)
            .clamp(0.0, 1.0)
            .acos()
            .to_degrees()
    };
    let tr = theta.to_radians();
    Strafe {
        view_yaw: vel_yaw,
        forward: MOVE_SPEED * tr.cos(),
        side: -sigma * MOVE_SPEED * tr.sin(),
        sigma,
    }
}

/// Mid-air course correction toward a fixed `bearing` — for a gap jump, curl jump, or rocket-jump arc,
/// where there is no hop cycle, just an arc to steer onto the landing line. A single continuous strafe
/// whose turn rate is proportional to the heading error and eases to zero at alignment (at `err ≈ 0`
/// the wish projects exactly onto the [`AIR_CAP`] and adds nothing — a coast on the current heading).
/// No mode switch and no deadband, so the returned wish never snaps. The strafe *side* still flips as
/// `err` crosses zero, but there the turn rate is ~0 and the wish is inert, so the caller applies the
/// wish in **world space** and steers the eyes separately — the flip never moves the view.
pub fn air_correct(v_xy: Vec2, bearing: f32, a_max: f32, dt: f32, gain: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = yaw_of(v_xy);
    let err = wrap180(bearing - vel_yaw);
    let omega = (err.abs() * gain).min(omega_gain_max(speed, a_max, dt));
    strafe_rate(v_xy, err.signum(), omega, a_max, dt)
}

/// A faithful one-tick QuakeWorld `PM_AirAccelerate`: the wish speed's projection onto the velocity
/// is capped at [`AIR_CAP`], and `accel·wishspeed·dt` (uncapped `wishspeed`) is added along `wishdir`.
pub fn apply_airaccel(v: Vec2, wishdir: Vec2, wishspeed: f32, accel: f32, dt: f32) -> Vec2 {
    let addspeed = wishspeed.min(AIR_CAP) - v.dot(wishdir);
    if addspeed <= 0.0 {
        return v;
    }
    let accelspeed = (accel * wishspeed * dt).min(addspeed);
    v + wishdir * accelspeed
}

/// A faithful one-tick QuakeWorld `PM_Accelerate` (ground): as [`apply_airaccel`] but the projection
/// limit is the full `wishspeed` (≤ `sv_maxspeed`), not [`AIR_CAP`].
pub fn apply_groundaccel(v: Vec2, wishdir: Vec2, wishspeed: f32, accel: f32, dt: f32) -> Vec2 {
    let addspeed = wishspeed - v.dot(wishdir);
    if addspeed <= 0.0 {
        return v;
    }
    let accelspeed = (accel * wishspeed * dt).min(addspeed);
    v + wishdir * accelspeed
}

/// A faithful one-tick QuakeWorld `PM_Friction` on flat ground: drop `max(speed, stopspeed) ·
/// friction · dt`, floored at zero.
pub fn apply_friction(v: Vec2, friction: f32, stopspeed: f32, dt: f32) -> Vec2 {
    let speed = v.length();
    if speed < 1.0 {
        return Vec2::ZERO;
    }
    let drop = speed.max(stopspeed) * friction * dt;
    v * ((speed - drop).max(0.0) / speed)
}

/// The wish direction the engine derives from a view yaw and forward/side move components:
/// `wishvel = forward·(cos, sin) + side·(sin, −cos)`, normalized.
pub fn wishdir_fs(view_yaw: f32, forward: f32, side: f32) -> Vec2 {
    let (sy, cy) = view_yaw.to_radians().sin_cos();
    (Vec2::new(cy, sy) * forward + Vec2::new(sy, -cy) * side).normalize_or_zero()
}

/// [`wishdir_fs`] for the single-key air strafe (`forward = 0`).
pub fn wishdir_of(view_yaw: f32, side: f32) -> Vec2 {
    wishdir_fs(view_yaw, 0.0, side)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ACCEL: f32 = 10.0;
    const MAXSPEED: f32 = 320.0;
    const DT: f32 = 1.0 / 77.0;

    #[test]
    fn emergency_turn_is_the_one_tick_heading_maximum() {
        for speed in [320.0f32, 450.0, 500.0, 800.0] {
            let v = Vec2::new(speed, 0.0);
            let a_max = air_accel_max(ACCEL, MAXSPEED, DT);
            let cmd = max_turn_strafe(v, 1.0, a_max);
            let result = apply_airaccel(v, wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side), MAXSPEED, ACCEL, DT);
            let best_turn = yaw_of(result);
            let expected = (a_max / speed).asin().to_degrees();
            assert!(
                (best_turn - expected).abs() < 0.01,
                "speed {speed}: {best_turn}° != {expected}°"
            );

            // Exhaustively bracket the command in wish-angle space. No ordinary forward/back/side
            // combination may produce a larger positive heading change through the engine oracle.
            for tenth_degree in 0..=1800 {
                let theta = tenth_degree as f32 * 0.1;
                let radians = theta.to_radians();
                let wish = Vec2::new(radians.cos(), radians.sin());
                let candidate = apply_airaccel(v, wish, MAXSPEED, ACCEL, DT);
                assert!(
                    yaw_of(candidate) <= best_turn + 0.001,
                    "speed {speed}: wish {theta}° turned {}° > {best_turn}°",
                    yaw_of(candidate)
                );
            }

            let gain_turn = omega_gain_max(speed, a_max, DT) * DT;
            assert!(
                best_turn > gain_turn * 1.3,
                "speed {speed}: {best_turn}° vs gain {gain_turn}°"
            );
            assert!(
                result.length() > speed * 0.99,
                "speed {speed}: excessive loss to {}",
                result.length()
            );
        }
    }

    #[test]
    fn emergency_turn_never_reverses_at_low_speed_or_coarse_ticks() {
        for (speed, dt) in [(20.0f32, DT), (40.0, DT), (100.0, 0.05)] {
            let v = Vec2::new(speed, 0.0);
            let a_max = air_accel_max(ACCEL, MAXSPEED, dt);
            let cmd = max_turn_strafe(v, 1.0, a_max);
            let result = apply_airaccel(v, wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side), MAXSPEED, ACCEL, dt);
            let best_turn = yaw_of(result);
            assert!(
                result.dot(v) > 0.0,
                "speed {speed}, dt {dt}: emergency reversed to {result:?}"
            );
            assert!(
                (0.0..=90.0).contains(&best_turn),
                "speed {speed}, dt {dt}: turned {best_turn}°"
            );

            for tenth_degree in 0..=1800 {
                let theta = tenth_degree as f32 * 0.1;
                let radians = theta.to_radians();
                let candidate = apply_airaccel(v, Vec2::new(radians.cos(), radians.sin()), MAXSPEED, ACCEL, dt);
                if candidate.dot(v) > 0.0 {
                    assert!(
                        yaw_of(candidate) <= best_turn + 0.05,
                        "speed {speed}, dt {dt}: non-reversing wish {theta}° beat {best_turn}°"
                    );
                }
            }
        }
    }
}
