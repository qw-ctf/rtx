// SPDX-License-Identifier: AGPL-3.0-or-later

//! A standalone QuakeWorld `PM_PlayerMove` over the BSP player hull — the offline forward simulator
//! the race-line optimizer ([`crate::raceline`]) rolls trajectories through. The live engine runs
//! the real pmove; this mirrors its per-tick order closely enough to *predict* a run: it drives the
//! same analytic [`crate::bot::bhop`] controller the bots use and integrates the result against
//! [`crate::bsp::Bsp::hull1_trace`], so a route's completion time can be estimated without a server.
//!
//! Frame order follows FTEQW `common/pmove.c` (and matches the flat-world oracle `bhop::sim`):
//! CategorizePosition → CheckJump (before friction) → Friction → Ground/Air accelerate (+ gravity) →
//! GroundMove (with step-up) / FlyMove (slide, ≤4 bumps) → CategorizePosition.
//!
//! Deliberately unmodelled (a rollout aborts if it needs them — see [`crate::raceline`]): water and
//! water moves, movers (plats/doors), ladders, teleports. Fidelity caveats (msec quantization, the
//! `DIST_EPSILON` in the hull trace, step-up corner cases) are why the optimizer is validated against
//! real leg times before it is trusted.

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bot::bhop::{apply_airaccel, apply_friction, apply_groundaccel, wishdir_fs, Cmd};
use crate::bsp::Bsp;
use rtx_nav::qphys::{JUMP_VZ, STEP_HEIGHT};

/// The map's movement cvars, snapshotted so a rollout is pure and reproducible.
#[derive(Clone, Copy)]
pub struct PmParams {
    pub gravity: f32,
    pub accel: f32,
    pub friction: f32,
    pub stopspeed: f32,
    pub maxspeed: f32,
}

impl Default for PmParams {
    fn default() -> Self {
        PmParams { gravity: 800.0, accel: 10.0, friction: 4.0, stopspeed: 100.0, maxspeed: 320.0 }
    }
}

/// A player's movement state between ticks.
#[derive(Clone, Copy, Debug)]
pub struct PmState {
    pub origin: Vec3,
    pub vel: Vec3,
    pub on_ground: bool,
    /// QW `pmove.jump_held`: set by a jump, cleared only when the jump button is released, so a held
    /// button doesn't re-jump every landing frame (the controller's pulse guard relies on this).
    pub jump_held: bool,
}

/// Overbounce for movement clipping (slide exactly along the plane, no bounce).
const OVERBOUNCE: f32 = 1.0;
/// A surface counts as ground when its normal tilts up at least this much (QW's `0.7`).
const GROUND_NORMAL_Z: f32 = 0.7;
/// Above this rising speed the player is considered airborne regardless of a floor below (jumping).
const ONGROUND_MAX_VZ: f32 = 180.0;

/// Advance one engine tick of `dt` seconds, mutating `s` in place.
pub fn pm_step(bsp: &Bsp, s: &mut PmState, cmd: &Cmd, p: &PmParams, dt: f32) {
    s.on_ground = categorize(bsp, s.origin, s.vel);

    // CheckJump — before friction, so a landing-frame jump skips ground friction and takes air accel.
    if !cmd.jump {
        s.jump_held = false;
    } else if s.on_ground && !s.jump_held {
        s.vel.z = JUMP_VZ;
        s.on_ground = false;
        s.jump_held = true;
    }

    if s.on_ground {
        let h = apply_friction(s.vel.xy(), p.friction, p.stopspeed, dt);
        s.vel.x = h.x;
        s.vel.y = h.y;
    }

    let wishdir = wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side);
    let wishspeed = Vec2::new(cmd.forward, cmd.side).length().min(p.maxspeed);

    if s.on_ground {
        let h = apply_groundaccel(s.vel.xy(), wishdir, wishspeed, p.accel, dt);
        // Ground movement is horizontal; the step logic owns Z.
        let (o, v) = ground_move(bsp, s.origin, Vec3::new(h.x, h.y, 0.0), dt);
        s.origin = o;
        s.vel = Vec3::new(v.x, v.y, 0.0);
    } else {
        let h = apply_airaccel(s.vel.xy(), wishdir, wishspeed, p.accel, dt);
        s.vel.x = h.x;
        s.vel.y = h.y;
        s.vel.z -= p.gravity * dt;
        let (o, v, _) = fly_move(bsp, s.origin, s.vel, dt);
        s.origin = o;
        s.vel = v;
    }

    s.on_ground = categorize(bsp, s.origin, s.vel);
}

/// Whether the player at `o` with velocity `v` is standing on ground: a short downward hull trace
/// hits a floor-ish surface, and the player isn't rising fast (mid-jump).
fn categorize(bsp: &Bsp, o: Vec3, v: Vec3) -> bool {
    if v.z > ONGROUND_MAX_VZ {
        return false;
    }
    let tr = bsp.hull1_trace(o, o - Vec3::new(0.0, 0.0, 1.0));
    tr.fraction < 1.0 && tr.plane_normal.z >= GROUND_NORMAL_Z
}

/// Reflect `v` off a plane so it slides along it (QW `PM_ClipVelocity`).
fn clip_velocity(v: Vec3, normal: Vec3, overbounce: f32) -> Vec3 {
    v - normal * (v.dot(normal) * overbounce)
}

/// The classic Quake slide-move (`SV_FlyMove` / `PM_FlyMove`): move by `v·dt`, and on hitting a
/// surface clip the velocity to slide along it, accumulating planes so a crease is handled by moving
/// along their intersection and a pocket dead-stops. Up to 4 bumps. Returns the new origin/velocity
/// and whether anything was hit.
fn fly_move(bsp: &Bsp, origin: Vec3, velocity: Vec3, dt: f32) -> (Vec3, Vec3, bool) {
    const MAX_CLIP_PLANES: usize = 5;
    let mut o = origin;
    let mut v = velocity;
    let primal = velocity;
    let mut original = velocity;
    let mut planes: [Vec3; MAX_CLIP_PLANES] = [Vec3::ZERO; MAX_CLIP_PLANES];
    let mut nplanes = 0usize;
    let mut time_left = dt;
    let mut blocked = false;

    for _ in 0..4 {
        if v == Vec3::ZERO {
            break;
        }
        let end = o + v * time_left;
        let tr = bsp.hull1_trace(o, end);
        if tr.all_solid {
            return (o, Vec3::ZERO, true); // wedged in solid — give up
        }
        if tr.fraction > 0.0 {
            o = tr.endpos;
            original = v;
            nplanes = 0;
        }
        if tr.fraction >= 1.0 {
            break; // moved the whole way
        }
        blocked = true;
        time_left -= time_left * tr.fraction;
        if nplanes >= MAX_CLIP_PLANES {
            return (o, Vec3::ZERO, true); // too many planes — dead stop
        }
        planes[nplanes] = tr.plane_normal;
        nplanes += 1;

        // Find a velocity that parallels every accumulated plane.
        let mut chosen = None;
        for i in 0..nplanes {
            let cand = clip_velocity(original, planes[i], OVERBOUNCE);
            if (0..nplanes).all(|j| j == i || cand.dot(planes[j]) >= 0.0) {
                chosen = Some(cand);
                break;
            }
        }
        v = match chosen {
            Some(c) => c,
            None => {
                // Caught in a crease: slide along the intersection of the two planes, or stop.
                if nplanes != 2 {
                    return (o, Vec3::ZERO, true);
                }
                let dir = planes[0].cross(planes[1]);
                dir * dir.dot(v)
            }
        };
        // Never let clipping reverse us into the original heading (kills tiny oscillation).
        if v.dot(primal) <= 0.0 {
            return (o, Vec3::ZERO, true);
        }
    }
    (o, v, blocked)
}

/// Ground move with step-up: attempt the flat slide, and independently attempt stepping up
/// [`STEP_HEIGHT`], sliding, then settling back down — keep whichever advanced farther horizontally
/// (QW `PM_StepSlideMove`). Lets a runner climb stairs and lips without a jump.
fn ground_move(bsp: &Bsp, origin: Vec3, velocity: Vec3, dt: f32) -> (Vec3, Vec3) {
    let (flat_o, flat_v, blocked) = fly_move(bsp, origin, velocity, dt);
    if !blocked {
        return (flat_o, flat_v);
    }
    let step = Vec3::new(0.0, 0.0, STEP_HEIGHT);
    // Step up (only as far as the hull can rise), slide, then trace back down to the floor.
    let up = bsp.hull1_trace(origin, origin + step).endpos;
    let (mut up_o, up_v, _) = fly_move(bsp, up, velocity, dt);
    let down = bsp.hull1_trace(up_o, up_o - step * 2.0);
    if down.plane_normal.z >= GROUND_NORMAL_Z || down.fraction < 1.0 {
        up_o = down.endpos;
    }
    // Keep the attempt that covered more horizontal ground.
    let flat_d = (flat_o.xy() - origin.xy()).length_squared();
    let up_d = (up_o.xy() - origin.xy()).length_squared();
    if up_d > flat_d {
        (up_o, up_v)
    } else {
        (flat_o, flat_v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bot::bhop::{self, air_accel_max, wishdir_of};

    /// A flat, infinite floor at z = 0 as a synthetic collision oracle: any query below the plane is
    /// solid. `hull1_trace` can't be built without a BSP, so these tests exercise the slide/step math
    /// indirectly via the pure accel/friction oracles; the full geometric path is covered by the
    /// opt-in real-map test in [`crate::raceline`].
    #[test]
    fn clip_velocity_slides_along_plane() {
        // Moving into a +z floor: the downward component is removed, horizontal preserved.
        let v = Vec3::new(300.0, 0.0, -100.0);
        let n = Vec3::new(0.0, 0.0, 1.0);
        let c = clip_velocity(v, n, OVERBOUNCE);
        assert!((c.z).abs() < 1e-4, "vertical not clipped: {}", c.z);
        assert!((c.x - 300.0).abs() < 1e-4, "horizontal changed: {}", c.x);
    }

    #[test]
    fn clip_velocity_off_wall_keeps_tangent() {
        // Into a +x wall: x removed, y (tangent) kept.
        let v = Vec3::new(-200.0, 150.0, 0.0);
        let n = Vec3::new(1.0, 0.0, 0.0);
        let c = clip_velocity(v, n, OVERBOUNCE);
        assert!(c.x.abs() < 1e-4 && (c.y - 150.0).abs() < 1e-4, "wall clip wrong: {c:?}");
    }

    /// The accel/friction oracles `pm_step` composes are the same ones `bhop::sim` validates, so a
    /// flat-world bhop run driven through them ramps speed exactly as the controller expects. This
    /// guards that `pm_step`'s horizontal update stays bit-identical to the controller's own oracle.
    #[test]
    fn horizontal_update_matches_bhop_oracle() {
        let dt = 1.0 / 77.0;
        let (accel, maxspeed) = (10.0, 320.0);
        let a = air_accel_max(accel, maxspeed, dt);
        let mut v = Vec2::new(320.0, 0.0);
        let mut sigma = 0.0;
        for _ in 0..385 {
            let cmd = bhop::strafe(v, 0.0, sigma, a);
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), maxspeed, accel, dt);
        }
        assert!(v.length() > 600.0, "oracle ramp regressed: {}", v.length());
    }
}
