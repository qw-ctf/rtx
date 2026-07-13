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
    /// `forward` move component: 0 in the air (single-key strafe); the bearing-aligned share of the
    /// wish during a ground prestrafe.
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

/// The fastest heading turn rate (deg/s) an air-strafe can sustain at `speed`: a perpendicular
/// strafe adds `min(a_max, cap)` ups sideways per tick, rotating the velocity by `atan(that/speed)`.
/// This is the ceiling [`strafe_rate`] clamps a requested rate to (and where it degenerates to the
/// max-gain [`theta_star`] angle).
pub fn omega_max(speed: f32, a_max: f32, dt: f32) -> f32 {
    let a = a_max.min(AIR_CAP);
    (a / speed.max(1.0)).atan().to_degrees() / dt.max(1e-4)
}

/// An air-strafe that turns the velocity at a *chosen* rate `omega_deg` (deg/s) rather than the
/// speed-optimal maximum — the smooth-lobe primitive. The turn rate is set by how much sideways
/// speed the tick adds: `a_need = speed·tan(ω·dt)`. Angling the wish so its projection onto the
/// velocity is `cap − a_need` (i.e. `θ = acos((cap − a_need)/speed)`, forward of perpendicular)
/// delivers exactly that sideways add while the parallel component still grows the speed. When the
/// requested rate meets or exceeds what the tick can physically deliver, fall back to the max-rate
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
        ((AIR_CAP - a_need).max(0.0) / speed).clamp(0.0, 1.0).acos().to_degrees()
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
    let omega = (err.abs() * gain).min(omega_max(speed, a_max, dt));
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
