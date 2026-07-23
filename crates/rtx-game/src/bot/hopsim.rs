// SPDX-License-Identifier: AGPL-3.0-or-later

//! Predictive hop planning.
//!
//! A bunny-hop is a fixed ballistic: once the bot leaps, gravity and the air-strafe decide where it
//! lands, and on a curved walkway (a spiral staircase's inner edge) the straight chord sags *inward*
//! over the void by more than the walkway is wide — so any purely reactive "don't step off the edge"
//! test either brakes the bot to a crawl or lets its momentum carry it off. The only honest question
//! is **where will this hop land**, and that the deterministic pmove simulator can answer.
//!
//! [`plan_hop`] rolls the [`pmove`](crate::pmove_sim) forward one hop under the **guided** policy the
//! controller will actually fly (a steady `air_correct` pursuit toward an aim point, no slalom),
//! across a fan of aim points along the route, and returns the first whose predicted landing stays on
//! the route's walkable floor. The controller then flies exactly that aim — prediction and execution
//! run the same command policy, so the rollout is trustworthy. It is the offline curl-jump certifier
//! (`navmesh::jumps::curl_land_point`) lifted to runtime, per hop. `None` means no hop from here lands
//! on-route — the *predicted* boxed state, and the one case a fallback brake is for.

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::math::yaw_of;
use crate::pmove_sim::{pm_step, Hull, PmParams, PmState};
use rtx_nav::strafe::{air_accel_max, air_correct, Cmd, MOVE_SPEED};

/// Rollout tick length (77 Hz, matching the offline certifier).
const DT: f32 = 1.0 / 77.0;
/// Tick budget for one hop rollout (~1 s — a hop's airtime is ~0.68 s; this leaves margin without
/// paying for a runaway). Kept tight because a rollout traces the live BSP hull several times a tick.
const MAX_TICKS: usize = 80;
/// A descent past this below the takeoff height is a fall off an edge, not a step/hop down.
const MAX_FALL: f32 = 64.0;
/// Air-strafe pursuit gains tried, gentlest first (a gentle arc is preferred when it suffices).
const GAINS: [f32; 2] = [8.0, 14.0];
/// A landing must fall within this perpendicular distance of the route polyline to count as on-route.
const LAND_LATERAL_TOL: f32 = 40.0;
/// …and within this height of the nearest-in-z route segment (keeps stacked spiral levels distinct).
const LAND_Z_TOL: f32 = 40.0;

/// What a single guided hop does when rolled forward from a takeoff state.
#[derive(Clone, Copy, Debug)]
pub enum HopRollout {
    /// Settled back on the ground at this origin.
    Landed { origin: Vec3 },
    /// Dropped more than [`MAX_FALL`] below the takeoff — carried off an edge.
    Fell,
    /// Never settled within the tick budget (should not happen on a sane arc).
    Overran,
}

/// Roll one guided hop from `st` (a grounded/landing frame) aiming at `aim`: leap on tick 0 along the
/// current velocity heading (keeping the bot's live speed), then pursue `aim` with `air_correct` at
/// `gain` every tick until it lands, falls, or overruns. Mirrors `curl_land_point`, but the launch
/// keeps the bot's momentum rather than a fixed corridor heading.
pub fn roll_hop(hull: &impl Hull, mut st: PmState, aim: Vec3, gain: f32, p: &PmParams) -> HopRollout {
    let amax = air_accel_max(p.accel, p.maxspeed, DT);
    let takeoff_z = st.origin.z;
    for tick in 0..MAX_TICKS {
        // The controller's guided policy: leap straight down the aim bearing on tick 0, then pursue
        // it with `air_correct` — recomputed each tick as the origin advances, so prediction matches
        // what the live controller (fed `bearing = yaw(aim − origin)` per frame) will fly.
        let bearing = yaw_of(aim.xy() - st.origin.xy());
        let cmd = if tick == 0 {
            Cmd {
                view_yaw: bearing,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: true,
            }
        } else {
            let s = air_correct(st.vel.xy(), bearing, amax, DT, gain);
            Cmd {
                view_yaw: s.view_yaw,
                forward: s.forward,
                side: s.side,
                jump: false,
            }
        };
        pm_step(hull, &mut st, &cmd, p, DT);
        if st.origin.z < takeoff_z - MAX_FALL {
            return HopRollout::Fell;
        }
        if tick > 3 && st.on_ground {
            return HopRollout::Landed { origin: st.origin };
        }
    }
    HopRollout::Overran
}

/// How closely a point sits on a route polyline segment.
#[derive(Clone, Copy, Debug)]
pub struct Projection {
    /// Height of the point above (`+`) or below (`-`) the matched segment.
    pub dz: f32,
    /// Perpendicular XY distance from the point to the polyline.
    pub lateral: f32,
}

/// Project `p` onto polyline `pts`, choosing the segment that best matches in **3D** — lateral offset
/// plus a doubled height penalty — so two stacked spiral flights that overlap in XY stay distinct.
/// Reports the lateral and vertical offsets of the best match. `None` for a degenerate polyline.
pub fn route_project(pts: &[Vec3], p: Vec3) -> Option<Projection> {
    let mut best: Option<(f32, Projection)> = None;
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let seg = b - a;
        let seg_len = seg.xy().length().max(1e-3);
        let t = ((p.xy() - a.xy()).dot(seg.xy()) / (seg_len * seg_len)).clamp(0.0, 1.0);
        let foot = a.lerp(b, t);
        let proj = Projection {
            dz: p.z - foot.z,
            lateral: (p.xy() - foot.xy()).length(),
        };
        let score = proj.lateral + proj.dz.abs() * 2.0;
        if best.as_ref().is_none_or(|(bs, _)| score < *bs) {
            best = Some((score, proj));
        }
    }
    best.map(|(_, pr)| pr)
}

/// A certified hop: the aim point the controller pursues, and the air-strafe gain that rolls its
/// landing onto the route.
#[derive(Clone, Copy, Debug)]
pub struct HopPlan {
    pub aim: Vec3,
    pub gain: f32,
}

/// Plan the next hop from `st` (a grounded frame) so its predicted landing stays on `route_pts` — the
/// leg-target polyline **starting at the bot's own position** (so arc-distances measure from the
/// bot). Enumerate aim points along the route at speed-scaled distances, each with small lateral
/// offsets (how the planner discovers a human's outer-wall line on a curve), longest-first; return the
/// first that, under some [gain](GAINS), rolls to a landing that projects onto the route within
/// tolerance and isn't a hazard. `None` when nothing lands on-route — the predicted boxed state.
pub fn plan_hop(
    hull: &impl Hull,
    is_hazard: &impl Fn(Vec3) -> bool,
    route_pts: &[Vec3],
    st: PmState,
    p: &PmParams,
) -> Option<HopPlan> {
    if route_pts.len() < 2 {
        return None;
    }
    let speed = st.vel.xy().length().max(1.0);
    let hop = speed * (2.0 * rtx_nav::qphys::JUMP_VZ / p.gravity); // ~a hop's flat reach
    for &frac in &[1.0, 0.8, 0.6] {
        let d = hop * frac;
        let base = point_on(route_pts, d);
        let perp = {
            let dir = (point_on(route_pts, d + 16.0).xy() - base.xy()).normalize_or_zero();
            Vec2::new(-dir.y, dir.x)
        };
        for &off in &[0.0, 32.0, -32.0] {
            let aim = base + Vec3::new(perp.x * off, perp.y * off, 0.0);
            for &gain in &GAINS {
                let HopRollout::Landed { origin, .. } = roll_hop(hull, st, aim, gain, p) else {
                    continue;
                };
                let Some(proj) = route_project(route_pts, origin) else {
                    continue;
                };
                if proj.lateral <= LAND_LATERAL_TOL && proj.dz.abs() <= LAND_Z_TOL && !is_hazard(origin) {
                    return Some(HopPlan { aim, gain });
                }
            }
        }
    }
    None
}

/// The point at XY arc-distance `d` along polyline `pts` (clamped to its ends).
fn point_on(pts: &[Vec3], d: f32) -> Vec3 {
    let mut acc = 0.0;
    for w in pts.windows(2) {
        let seg = (w[1].xy() - w[0].xy()).length();
        if acc + seg >= d {
            let t = if seg > 1e-3 { (d - acc) / seg } else { 0.0 };
            return w[0].lerp(w[1], t);
        }
        acc += seg;
    }
    pts.last().copied().unwrap_or(Vec3::ZERO)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pmove_sim::HeightHull;

    fn no_hazard(_: Vec3) -> bool {
        false
    }

    /// The z-gated projection keeps two stacked flights (same XY, 100u apart) distinct: a point on the
    /// upper flight matches the upper segment, never the lower.
    #[test]
    fn route_project_distinguishes_stacked_levels() {
        // Lower flight along +x at z=0, upper flight doubling back at z=100 — overlapping in XY.
        let pts = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(200.0, 0.0, 0.0),
            Vec3::new(200.0, 0.0, 100.0),
            Vec3::new(0.0, 0.0, 100.0),
        ];
        let on_upper = Vec3::new(100.0, 0.0, 100.0);
        let pr = route_project(&pts, on_upper).unwrap();
        assert!(pr.dz.abs() < 1.0, "should match the upper flight in z, dz={}", pr.dz);
        assert!(pr.lateral < 1.0, "and sit on it laterally, lateral={}", pr.lateral);
        // Against the naive nearest-XY match it would snap to the lower flight (dz≈100); the z-gate
        // keeps the levels distinct.
        let ambiguous = route_project(&pts, Vec3::new(100.0, 0.0, 100.0)).unwrap();
        assert!(
            ambiguous.dz.abs() < 50.0,
            "z-gate must not snap to the far level: dz={}",
            ambiguous.dz
        );
    }

    /// On a flat annular walkway, a straight tangent hop would sag into the core; `plan_hop` finds an
    /// outward-aimed hop whose *predicted* landing stays on the ring.
    #[test]
    fn plan_hop_keeps_a_curved_hop_on_the_ring() {
        let (r_i, r_o) = (100.0f32, 300.0f32);
        let hull = HeightHull {
            floor: move |x, y| {
                let rho = (x * x + y * y).sqrt();
                (r_i..=r_o).contains(&rho).then_some(0.0)
            },
        };
        let p = PmParams::default();
        // Bot mid-ring at angle 0, moving tangentially (+y) at a controlled bhop speed.
        let rho = 0.5 * (r_i + r_o);
        let st = PmState {
            origin: Vec3::new(rho, 0.0, 0.0),
            vel: Vec3::new(0.0, 320.0, 0.0),
            on_ground: true,
            jump_held: false,
        };
        // Route polyline: points around the ring ahead of the bot, starting at the bot.
        let route: Vec<Vec3> = (0..=8)
            .map(|i| {
                let a = i as f32 / 24.0 * std::f32::consts::TAU; // 120° of ring ahead
                Vec3::new(rho * a.cos(), rho * a.sin(), 0.0)
            })
            .collect();
        let plan = plan_hop(&hull, &no_hazard, &route, st, &p).expect("a hop should land on the ring");
        // Flying the returned plan lands back on the ring (not sagged off the inner edge into the core).
        let HopRollout::Landed { origin } = roll_hop(&hull, st, plan.aim, plan.gain, &p) else {
            panic!("the planned hop should land");
        };
        let land_rho = origin.xy().length();
        assert!(
            (r_i..=r_o).contains(&land_rho),
            "planned landing must be on the ring: rho={land_rho}"
        );
    }

    /// A route running off a cliff returns `None` — the predicted boxed state. A hop leaps at the
    /// bot's live speed and can only steer, not brake, so from a short strip it always overshoots the
    /// lip into the void; nothing lands on-route, and that is exactly when a fallback brake is for.
    #[test]
    fn plan_hop_none_when_the_route_runs_off_a_cliff() {
        let hull = HeightHull {
            floor: |x, _| (x <= 60.0).then_some(0.0), // floor ends at x=60, void beyond
        };
        let p = PmParams::default();
        let st = PmState {
            origin: Vec3::new(30.0, 0.0, 0.0),
            vel: Vec3::new(400.0, 0.0, 0.0), // fast, straight at the lip
            on_ground: true,
            jump_held: false,
        };
        let route = [Vec3::new(30.0, 0.0, 0.0), Vec3::new(60.0, 0.0, 0.0)];
        assert!(plan_hop(&hull, &no_hazard, &route, st, &p).is_none());
    }

    /// A hop rolled onto open flat floor lands (a sanity check that `roll_hop` terminates in `Landed`).
    #[test]
    fn roll_hop_lands_on_flat_floor() {
        let hull = HeightHull {
            floor: |_, _| Some(0.0),
        };
        let p = PmParams::default();
        let st = PmState {
            origin: Vec3::ZERO,
            vel: Vec3::new(300.0, 0.0, 0.0),
            on_ground: true,
            jump_held: false,
        };
        let aim = Vec3::new(1000.0, 0.0, 0.0);
        assert!(matches!(roll_hop(&hull, st, aim, 10.0, &p), HopRollout::Landed { .. }));
    }
}
