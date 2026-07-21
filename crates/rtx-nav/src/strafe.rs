// SPDX-License-Identifier: AGPL-3.0-or-later

//! The pure QuakeWorld air/ground movement oracles — one-tick `PM_AirAccelerate`/`PM_Accelerate`/
//! `PM_Friction`, the wish-direction geometry, and the air-strafe angle solvers ([`strafe_rate`],
//! [`air_correct`]). Free functions of their inputs, no engine or game state. They live in rtx-nav so
//! both the offline pmove sim ([`crate::pmove`]) and the navmesh build's curl-jump certifier can drive
//! the *same* physics the bot's bhop controller flies (which re-exports these). See `crate::qphys` for
//! the shared constants (`AIR_CAP`).

use glam::{Vec2, Vec3};

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

/// The command representation a `SetBotCMD` fake client actually submits after the bot's desired
/// world-space move has been reprojected onto its emitted (spring-smoothed) view and rounded to the
/// integer syscall fields. `msec` is the fake-bot `Sense` class; movement time is exactly
/// `msec / 1000`, not the unquantized server `frametime`.
///
/// This is deliberately fake-client provenance. The netclient host drops the brain's `msec` and its
/// packet driver stamps separately measured wall-clock time; it must not reuse this value.
#[derive(Clone, Copy, Debug)]
pub struct FakeBotMove {
    pub msec: u8,
    pub emitted_view: Vec3,
    pub forward: i32,
    pub side: i32,
    pub jump: bool,
}

impl FakeBotMove {
    /// Reproduce `bot::emit`: normalize the controller's wish into a world-space `MOVE_SPEED`
    /// vector, project it onto Quake's emitted forward/right view basis, then round to syscall ints.
    /// Returns `None` outside the fake-bot host's accepted 1..=100ms frame range.
    pub fn from_controller(cmd: Cmd, emitted_view: Vec3, msec: u8) -> Option<Self> {
        if !(1..=100).contains(&msec) || !emitted_view.is_finite() {
            return None;
        }
        let move_world = wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side) * MOVE_SPEED;
        let (sin_yaw, cos_yaw) = emitted_view.y.to_radians().sin_cos();
        let cos_pitch = emitted_view.x.to_radians().cos();
        let view_forward = Vec2::new(cos_pitch * cos_yaw, cos_pitch * sin_yaw);
        let view_right = Vec2::new(sin_yaw, -cos_yaw);
        Some(Self {
            msec,
            emitted_view,
            forward: view_forward.dot(move_world).round() as i32,
            side: view_right.dot(move_world).round() as i32,
            jump: cmd.jump,
        })
    }

    pub fn dt(self) -> f32 {
        self.msec as f32 / 1000.0
    }

    /// The pure pmove input represented by the integer fake-client command. Pitch is already
    /// reflected in the projected forward magnitude; planar pmove consumes the emitted yaw.
    pub fn pmove_cmd(self) -> Cmd {
        Cmd {
            view_yaw: self.emitted_view.y,
            forward: self.forward as f32,
            side: self.side as f32,
            jump: self.jump,
        }
    }
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

/// Ground circle-strafe command used by an offline run-up rollout.  This is the ground analogue of
/// [`air_correct`]: below the angling threshold it simply runs at `bearing`; above it, it holds the
/// ground-optimal wish angle and keeps a sticky weave sign until the velocity has crossed a broad
/// deadband around the requested bearing.  Returning the same [`Strafe`] shape as the air solver lets
/// a certifier drive [`crate::pmove::pm_step_report`] without importing game code.
pub fn ground_prestrafe(v_xy: Vec2, bearing: f32, prev_sigma: f32, a_ground: f32, maxspeed: f32) -> Strafe {
    let speed = v_xy.length();
    let u_star = (maxspeed - a_ground).max(0.0);
    if speed <= u_star.max(60.0) {
        return Strafe {
            view_yaw: bearing,
            forward: MOVE_SPEED,
            side: 0.0,
            sigma: prev_sigma,
        };
    }

    let vel_yaw = yaw_of(v_xy);
    let err = wrap180(bearing - vel_yaw);
    // Keep the ground-optimal weave tight enough for a one-cell stair/runway.  A broad free-field
    // circle strafe can gain the same speed but drifts 16+ units sideways before reversing — enough
    // to leave DM3's RA stairs without ever touching a wall.  A sub-one optimum-angle tick per
    // half-wave retains the gain while the path bearing continually recentres the run.
    let psi = (AIR_CAP / speed.max(1.0)).atan().to_degrees();
    let band = (psi * 0.75).clamp(3.0, 6.0);
    let sigma = if prev_sigma == 0.0 {
        if err >= 0.0 {
            1.0
        } else {
            -1.0
        }
    } else if err * prev_sigma < -band {
        -prev_sigma
    } else {
        prev_sigma
    };
    let theta = (u_star / speed).clamp(0.0, 1.0).acos();
    let (s, c) = theta.sin_cos();
    Strafe {
        view_yaw: vel_yaw,
        forward: MOVE_SPEED * c,
        side: -sigma * MOVE_SPEED * s,
        sigma,
    }
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
mod quantized_tests {
    use super::*;

    #[test]
    fn fake_bot_move_reprojects_rounds_and_derives_dt_from_msec() {
        // Controller +side convention: view 0 with side -800 wishes due north. Reprojecting that
        // world wish onto a 90-degree emitted view becomes pure +forward.
        let north = Cmd {
            view_yaw: 0.0,
            forward: 0.0,
            side: -800.0,
            jump: true,
        };
        let wire = FakeBotMove::from_controller(north, Vec3::new(0.0, 90.0, 0.0), 20).unwrap();
        assert_eq!((wire.forward, wire.side), (800, 0));
        assert!(wire.jump);
        assert_eq!(wire.dt().to_bits(), 0.020f32.to_bits());
        let pm = wire.pmove_cmd();
        assert!((wishdir_fs(pm.view_yaw, pm.forward, pm.side) - Vec2::Y).length() < 1e-6);

        // 800 projected onto a 45-degree basis is 565.685... in both components: the syscall sees
        // the exact round-to-i32 result, not the controller floats.
        let east = Cmd {
            view_yaw: 0.0,
            forward: 800.0,
            side: 0.0,
            jump: false,
        };
        let diagonal_view = FakeBotMove::from_controller(east, Vec3::new(0.0, 45.0, 0.0), 20).unwrap();
        assert_eq!((diagonal_view.forward, diagonal_view.side), (566, 566));
        assert!(
            (wishdir_fs(
                diagonal_view.emitted_view.y,
                diagonal_view.forward as f32,
                diagonal_view.side as f32,
            ) - Vec2::X)
                .length()
                < 1e-6
        );

        // `emit` projects with the full forward vector, so pitch attenuates forward before pmove.
        let pitched = FakeBotMove::from_controller(east, Vec3::new(60.0, 0.0, 0.0), 20).unwrap();
        assert_eq!((pitched.forward, pitched.side), (400, 0));
        assert!(FakeBotMove::from_controller(east, Vec3::ZERO, 0).is_none());
        assert!(FakeBotMove::from_controller(east, Vec3::ZERO, 101).is_none());
    }
}
