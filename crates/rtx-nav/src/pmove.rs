// SPDX-License-Identifier: AGPL-3.0-or-later

//! A standalone QuakeWorld `PM_PlayerMove` over the BSP player hull — the offline forward simulator
//! the race-line optimizer and the navmesh build's curl-jump certifier roll trajectories through. The
//! live engine runs the real pmove; this mirrors its per-tick order closely enough to *predict* a run:
//! it drives the same analytic [`crate::strafe`] oracles the bots use and integrates the result against
//! [`crate::bsp::Bsp::hull1_trace`], so a route's completion time (or a jump's landing) can be
//! estimated without a server.
//!
//! Frame order follows FTEQW `common/pmove.c` (and matches the flat-world bhop oracle):
//! CategorizePosition → CheckJump (before friction) → Friction → Ground/Air accelerate (+ gravity) →
//! GroundMove (with step-up) / FlyMove (slide, ≤4 bumps) → CategorizePosition.
//!
//! Deliberately unmodelled (a rollout aborts if it needs them): water and water moves, movers
//! (plats/doors), ladders, teleports. Fidelity caveats (msec quantization, the `DIST_EPSILON` in the
//! hull trace, step-up corner cases) are why the optimizer is validated against real leg times before
//! it is trusted.

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bsp::{Bsp, HullTrace};
use crate::qphys::{JUMP_VZ, STEP_HEIGHT};
use crate::strafe::{apply_airaccel, apply_friction, apply_groundaccel, wishdir_fs, Cmd};

/// The player clip hull a rollout traces against. [`Bsp`] is the live implementation (the map's
/// hull 1); a rollout is generic over this so a test can substitute a synthetic hull built from an
/// `is_solid` oracle ([`SampledHull`]) and simulate trajectories without a compiled map.
pub trait Hull {
    /// Trace the standing player hull from `a` to `b` — QW `hull1_trace` semantics (see [`HullTrace`]).
    fn trace(&self, a: Vec3, b: Vec3) -> HullTrace;
}

impl Hull for Bsp {
    fn trace(&self, a: Vec3, b: Vec3) -> HullTrace {
        self.hull1_trace(a, b)
    }
}

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
        PmParams {
            gravity: 800.0,
            accel: 10.0,
            friction: 4.0,
            stopspeed: 100.0,
            maxspeed: 320.0,
        }
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
pub fn pm_step(hull: &impl Hull, s: &mut PmState, cmd: &Cmd, p: &PmParams, dt: f32) {
    s.on_ground = categorize(hull, s.origin, s.vel);

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
        let (o, v) = ground_move(hull, s.origin, Vec3::new(h.x, h.y, 0.0), dt);
        s.origin = o;
        s.vel = Vec3::new(v.x, v.y, 0.0);
    } else {
        let h = apply_airaccel(s.vel.xy(), wishdir, wishspeed, p.accel, dt);
        s.vel.x = h.x;
        s.vel.y = h.y;
        s.vel.z -= p.gravity * dt;
        let (o, v, _) = fly_move(hull, s.origin, s.vel, dt);
        s.origin = o;
        s.vel = v;
    }

    s.on_ground = categorize(hull, s.origin, s.vel);
}

/// Whether the player at `o` with velocity `v` is standing on ground: a short downward hull trace
/// hits a floor-ish surface, and the player isn't rising fast (mid-jump).
fn categorize(hull: &impl Hull, o: Vec3, v: Vec3) -> bool {
    if v.z > ONGROUND_MAX_VZ {
        return false;
    }
    let tr = hull.trace(o, o - Vec3::new(0.0, 0.0, 1.0));
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
fn fly_move(hull: &impl Hull, origin: Vec3, velocity: Vec3, dt: f32) -> (Vec3, Vec3, bool) {
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
        let tr = hull.trace(o, end);
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
fn ground_move(hull: &impl Hull, origin: Vec3, velocity: Vec3, dt: f32) -> (Vec3, Vec3) {
    let (flat_o, flat_v, blocked) = fly_move(hull, origin, velocity, dt);
    if !blocked {
        return (flat_o, flat_v);
    }
    let step = Vec3::new(0.0, 0.0, STEP_HEIGHT);
    // Step up (only as far as the hull can rise), slide, then trace back down to the floor.
    let up = hull.trace(origin, origin + step).endpos;
    let (mut up_o, up_v, _) = fly_move(hull, up, velocity, dt);
    let down = hull.trace(up_o, up_o - step * 2.0);
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

/// A synthetic player-clip [`Hull`] for rollout tests: a height field. `floor(x, y)` gives the
/// walkable surface height at a column, or `None` for a bottomless void (a spiral's open core). A
/// standing origin is inside solid iff it sits *below* the surface there and rests *on* it. Traces
/// march the segment for first contact and estimate the normal from the local solid gradient — enough
/// to roll hop arcs, climb ramps/stairs, and fall through voids without a compiled map. Not a general
/// BSP: it models the walkable-surface worlds the movement tests need. Exposed (doc-hidden) so
/// dependent crates' tests can build worlds from an `is_solid`-style closure, the codebase idiom.
#[doc(hidden)]
pub struct HeightHull<F: Fn(f32, f32) -> Option<f32>> {
    pub floor: F,
}

impl<F: Fn(f32, f32) -> Option<f32>> HeightHull<F> {
    fn solid(&self, p: Vec3) -> bool {
        (self.floor)(p.x, p.y).is_some_and(|fz| p.z < fz)
    }

    /// Surface normal at `c`, from a 6-neighbour solid gradient pointing to the empty side: a flat
    /// floor gives `+z`, a ramp a tilted normal, a vertical riser a horizontal one.
    fn normal(&self, c: Vec3) -> Vec3 {
        let e = 1.0;
        let mut n = Vec3::ZERO;
        for d in [Vec3::X, Vec3::Y, Vec3::Z] {
            n += d * (i32::from(self.solid(c - d * e)) - i32::from(self.solid(c + d * e))) as f32;
        }
        n.normalize_or_zero()
    }
}

impl<F: Fn(f32, f32) -> Option<f32>> Hull for HeightHull<F> {
    fn trace(&self, a: Vec3, b: Vec3) -> HullTrace {
        let clear = HullTrace {
            fraction: 1.0,
            endpos: b,
            plane_normal: Vec3::ZERO,
            plane_dist: 0.0,
            start_solid: false,
            all_solid: false,
            in_open: true,
            in_water: false,
        };
        if self.solid(a) {
            return HullTrace {
                endpos: a,
                start_solid: true,
                all_solid: true,
                in_open: false,
                ..clear
            };
        }
        let len = (b - a).length();
        if len < 1e-6 {
            return clear;
        }
        let steps = len.ceil() as i32; // ~1u march
        let mut prev = a;
        for i in 1..=steps {
            let p = a.lerp(b, (i as f32 / steps as f32).min(1.0));
            if self.solid(p) {
                // Bisect [prev (empty), p (solid)] for the contact just outside the surface.
                let (mut lo, mut hi) = (prev, p);
                for _ in 0..8 {
                    let mid = (lo + hi) * 0.5;
                    if self.solid(mid) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return HullTrace {
                    fraction: ((lo - a).length() / len).clamp(0.0, 1.0),
                    endpos: lo,
                    plane_normal: self.normal(lo),
                    ..clear
                };
            }
            prev = p;
        }
        clear
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flat-floor hop from a standstill-ish run reaches the analytic apex and lands after the
    /// analytic airtime — validating that `HeightHull` traces a rollout faithfully enough to plan on.
    #[test]
    fn height_hull_rolls_a_faithful_hop_arc() {
        use crate::strafe::MOVE_SPEED;
        let hull = HeightHull {
            floor: |_, _| Some(0.0),
        };
        let p = PmParams::default();
        let dt = 1.0 / 77.0;
        let mut s = PmState {
            origin: Vec3::new(0.0, 0.0, 0.0),
            vel: Vec3::new(320.0, 0.0, 0.0),
            on_ground: true,
            jump_held: false,
        };
        let mut max_z = 0.0f32;
        let mut airtime = 0.0;
        let mut landed = false;
        for tick in 0..200 {
            let cmd = Cmd {
                view_yaw: 0.0,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: tick == 0,
            };
            pm_step(&hull, &mut s, &cmd, &p, dt);
            max_z = max_z.max(s.origin.z);
            if tick > 3 && s.on_ground {
                airtime = tick as f32 * dt;
                landed = true;
                break;
            }
        }
        assert!(landed, "the hop must land back on the floor");
        // Analytic: apex JUMP_VZ^2 / 2g = 270^2/1600 ≈ 45.6u, airtime 2·270/800 ≈ 0.675s.
        assert!((max_z - 45.6).abs() < 4.0, "apex off: {max_z}");
        assert!((airtime - 0.675).abs() < 0.05, "airtime off: {airtime}");
        assert!(
            (s.origin.z).abs() < 1.0,
            "should rest back on the floor: {}",
            s.origin.z
        );
    }

    /// A void column (`floor` returns `None`) is fallen through, not stood on — the spiral-core case.
    #[test]
    fn height_hull_falls_through_a_void() {
        let hull = HeightHull {
            floor: |x, _| (x < 32.0).then_some(0.0), // floor only for x < 32, void beyond
        };
        let p = PmParams::default();
        let dt = 1.0 / 77.0;
        let mut s = PmState {
            origin: Vec3::new(0.0, 0.0, 0.0),
            vel: Vec3::new(600.0, 0.0, 0.0), // fast enough to clear the floor edge into the void
            on_ground: true,
            jump_held: false,
        };
        for tick in 0..200 {
            let cmd = Cmd {
                view_yaw: 0.0,
                forward: 800.0,
                side: 0.0,
                jump: tick == 0,
            };
            pm_step(&hull, &mut s, &cmd, &p, dt);
            if s.origin.z < -200.0 {
                return; // fell into the void, as expected
            }
        }
        panic!("should have fallen into the void past the floor edge, z={}", s.origin.z);
    }

    /// Moving into a +z floor: the downward component is removed, horizontal preserved.
    #[test]
    fn clip_velocity_slides_along_plane() {
        let v = Vec3::new(300.0, 0.0, -100.0);
        let n = Vec3::new(0.0, 0.0, 1.0);
        let c = clip_velocity(v, n, OVERBOUNCE);
        assert!((c.z).abs() < 1e-4, "vertical not clipped: {}", c.z);
        assert!((c.x - 300.0).abs() < 1e-4, "horizontal changed: {}", c.x);
    }

    /// Into a +x wall: x removed, y (tangent) kept.
    #[test]
    fn clip_velocity_off_wall_keeps_tangent() {
        let v = Vec3::new(-200.0, 150.0, 0.0);
        let n = Vec3::new(1.0, 0.0, 0.0);
        let c = clip_velocity(v, n, OVERBOUNCE);
        assert!(c.x.abs() < 1e-4 && (c.y - 150.0).abs() < 1e-4, "wall clip wrong: {c:?}");
    }
}
