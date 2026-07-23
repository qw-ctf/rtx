// SPDX-License-Identifier: AGPL-3.0-or-later

//! Offline racing-line optimization — automated TAS'ing for race maps. Given a navmesh route between
//! two race nodes, we resample it into a control polyline, then search for the lateral offsets that
//! let the [`crate::bot::bhop`] controller (driven through the standalone [`crate::pmove_sim`]) reach
//! the finish fastest. The controller is already a pure function of its inputs and the sim is pure
//! over the BSP, so a full run is a deterministic rollout and the completion time is the objective.
//!
//! This is "point 3" of the design: line choice / when-to-convert / jump-timing, which the low-level
//! controller can't decide on its own. The optimizer is a hand-rolled (1+1)-ES — no heavy deps, and
//! the controller absorbs most of the dimensionality, so a handful of lateral knobs per route go a
//! long way. The result is a per-route [`RaceLine`] the race bots track (see [`crate::mode::race`]);
//! it's advisory — a bot that strays falls back to plain navmesh following until it re-acquires.

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bot::bhop::{self, Bhop, Env};
use crate::bsp::Bsp;
use crate::math::yaw_of;
use crate::pmove_sim::{pm_step, PmParams, PmState};
use crate::race::{touching, RaceRouteNode};

/// A move technique tag on a line point, chosen by the optimizer / route kinds. Advisory: it biases
/// the controller inputs (zigzag vs hop vs a committed speed-jump leap), the physics decides the rest.
pub const TECH_RUN: u8 = 0;
/// Ground zigzag. Part of the technique vocabulary for completeness; the rollout currently infers
/// zigzag vs hop from the corridor geometry rather than a per-point tag, so nothing emits it yet.
#[allow(dead_code)]
pub const TECH_ZIGZAG: u8 = 1;
pub const TECH_HOP: u8 = 2;
pub const TECH_SPEEDJUMP: u8 = 3;

/// One control point of a racing line: where to be, how fast to want to be going there, and how.
#[derive(Clone, Copy, Debug)]
pub struct LinePoint {
    pub pos: Vec3,
    pub target_speed: f32,
    pub technique: u8,
}

/// A full racing line for one race leg — the polyline a bot tracks.
#[derive(Clone, Debug, Default)]
pub struct RaceLine {
    pub points: Vec<LinePoint>,
}

/// The optimizer genome: a lateral offset (perpendicular to the local path, clamped to the corridor)
/// per control point. Kept deliberately small — the controller handles the per-tick dimensionality.
#[derive(Clone, Debug, Default)]
pub struct LineParams {
    pub lateral: Vec<f32>,
}

/// The outcome of a rollout: completion time (or the time spent before giving up), whether the finish
/// was reached, whether the runner fell off, and how many route nodes it touched.
#[derive(Clone, Copy, Debug)]
pub struct RolloutResult {
    pub time: f32,
    pub finished: bool,
    pub fell: bool,
    pub reached: usize,
}

/// Engine tick the sim steps at (bots run ~77 Hz; the sim is msec-quantized — a named fidelity risk,
/// validated against live leg times before the optimizer is trusted).
const DT: f32 = 1.0 / 77.0;
/// A rollout that falls this far below the lowest line point has left the course.
const FELL_BELOW: f32 = 512.0;
/// Advance the line cursor once within this of a control point.
const ARRIVE_LINE: f32 = 48.0;
/// Straightness tolerance (degrees) for measuring the remaining straight runway ahead on the line.
const RUNWAY_BEND: f32 = 35.0;
/// Default per-point speed a bot should carry (just under the friction equilibrium); only consulted
/// as the `hold_jump` threshold on speed-jump points.
const DEFAULT_TARGET_SPEED: f32 = 450.0;

// --- deterministic PRNG (xorshift32) ---

/// A tiny deterministic PRNG so a given `(map, route)` seed always produces the same optimization —
/// no `Math.random`, reproducible across runs and the resume journal.
struct Prng(u32);

impl Prng {
    fn new(seed: u32) -> Self {
        // SplitMix32 finalizer so nearby seeds diverge; forced nonzero (xorshift fixed point at 0).
        let mut x = seed ^ 0x9e37_79b9;
        x ^= x >> 16;
        x = x.wrapping_mul(0x7feb_352d);
        x ^= x >> 15;
        x = x.wrapping_mul(0x846c_a68b);
        x ^= x >> 16;
        Prng(x.max(1))
    }
    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }
    /// Uniform in `[-1, 1)`.
    fn sym(&mut self) -> f32 {
        self.unit() * 2.0 - 1.0
    }
}

/// Resample a route polyline to roughly uniform arclength `spacing`, always keeping the endpoints.
pub fn resample(points: &[Vec3], spacing: f32) -> Vec<Vec3> {
    if points.len() < 2 || spacing <= 0.0 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    let mut carry = 0.0f32;
    for seg in points.windows(2) {
        let (a, b) = (seg[0], seg[1]);
        let len = (b - a).length();
        if len < 1e-3 {
            continue;
        }
        let dir = (b - a) / len;
        let mut d = spacing - carry;
        while d < len {
            out.push(a + dir * d);
            d += spacing;
        }
        carry = len - (d - spacing);
    }
    out.push(*points.last().unwrap());
    out
}

/// Build a [`RaceLine`] from a control polyline and a genome: shift each interior point sideways
/// (perpendicular to its local tangent, in XY) by its lateral gene, clamped to that point's corridor
/// half-width. Endpoints stay put so the line still starts/ends on the route. `half_width` is
/// per-point (from perpendicular BSP probes); a shorter slice clamps the rest to zero offset.
pub fn decode(polyline: &[Vec3], params: &LineParams, half_width: &[f32]) -> RaceLine {
    let n = polyline.len();
    let points = (0..n)
        .map(|i| {
            let mut pos = polyline[i];
            if i > 0 && i + 1 < n {
                let tangent = (polyline[i + 1] - polyline[i - 1]).xy().normalize_or_zero();
                let perp = Vec2::new(-tangent.y, tangent.x); // left-hand normal in XY
                let hw = half_width.get(i).copied().unwrap_or(0.0);
                let off = params.lateral.get(i).copied().unwrap_or(0.0).clamp(-hw, hw);
                let shift = perp * off;
                pos.x += shift.x;
                pos.y += shift.y;
            }
            LinePoint {
                pos,
                target_speed: DEFAULT_TARGET_SPEED,
                technique: TECH_RUN,
            }
        })
        .collect();
    RaceLine { points }
}

/// Roll a rollout of the [`Bhop`] controller tracking `line` through the pmove sim, from the start
/// node to the finish, touching checkpoints in order. Pure and deterministic. Aborts (with the time
/// spent) if the runner falls off or exceeds `timeout`.
pub fn rollout(bsp: &Bsp, line: &RaceLine, nodes: &[RaceRouteNode], pm: &PmParams, timeout: f32) -> RolloutResult {
    if line.points.len() < 2 || nodes.len() < 2 {
        return RolloutResult {
            time: 0.0,
            finished: false,
            fell: false,
            reached: 0,
        };
    }
    let floor = line.points.iter().map(|p| p.pos.z).fold(f32::INFINITY, f32::min);
    let env = Env {
        dt: DT,
        accel: pm.accel,
        maxspeed: pm.maxspeed,
        friction: pm.friction,
        stopspeed: pm.stopspeed,
        profile: crate::bot::human_profile::HumanMovementProfile::legacy(),
    };
    let start_dir = (line.points[1].pos - line.points[0].pos).xy().normalize_or_zero();
    let mut st = PmState {
        origin: line.points[0].pos,
        vel: Vec3::new(start_dir.x, start_dir.y, 0.0) * pm.maxspeed,
        on_ground: true,
        jump_held: false,
    };
    let mut bh = Bhop::default();
    let mut cursor = 0usize; // index of the line point we're heading toward
    let mut reached = 1usize; // the start node counts as touched at spawn
    let mut t = 0.0f32;

    let pts = &line.points;
    let last = pts.len() - 1;
    while t < timeout {
        // Advance the cursor past points we've reached or projected beyond.
        while cursor < last {
            let here = pts[cursor].pos;
            let rel = (st.origin - here).xy();
            let passed = {
                let seg = (pts[cursor + 1].pos - here).xy();
                seg.length_squared() > 1.0 && rel.dot(seg) >= seg.length_squared()
            };
            if passed || rel.length() < ARRIVE_LINE {
                cursor += 1;
            } else {
                break;
            }
        }
        // Steer toward a look-ahead point a couple of controls down the line.
        let target = pts[(cursor + 2).min(last)].pos;
        let dir = (target.xy() - st.origin.xy()).normalize_or_zero();
        let bearing = yaw_of(dir);
        let runway = line_runway(pts, cursor);
        let tech = pts[cursor.min(last)].technique;
        let input = bhop::Input {
            v_xy: st.vel.xy(),
            on_ground: st.on_ground,
            bearing,
            runway,
            eligible: runway >= bhop::RUNWAY_ENGAGE,
            zigzag: runway >= bhop::ZIGZAG_ENGAGE,
            sustain: true,
            veto: false,
            committed: tech == TECH_SPEEDJUMP,
            carry: tech == TECH_HOP || tech == TECH_SPEEDJUMP,
            hold_jump: tech == TECH_SPEEDJUMP && st.vel.xy().length() < pts[cursor.min(last)].target_speed,
            // Racing-line speed jumps are straight (curl_gain 0) and must NOT enter the curl takeoff
            // regime: `runway` here is remaining-line arclength, not distance-to-takeoff, so the
            // hold-to-lip branch would misfire. `hold_jump` above already gates the leap by target_speed.
            takeoff_speed: 0.0,
            curl_gain: 0.0, // racing-line speed jumps are straight; keep the slalom the line was tuned with
            clear: f32::INFINITY, // the offline line is already collision-clean; no runtime wall probe
            now: t,
        };
        let cmd = bh.step(&input, &env).unwrap_or(bhop::Cmd {
            view_yaw: bearing,
            forward: crate::defs::BOT_MOVE_SPEED,
            side: 0.0,
            jump: false,
        });
        pm_step(bsp, &mut st, &cmd, pm, DT);
        t += DT;

        // Touch the next route node in order (checkpoints then finish).
        while reached < nodes.len() && touching(st.origin, &nodes[reached]) {
            reached += 1;
        }
        if reached >= nodes.len() {
            return RolloutResult {
                time: t,
                finished: true,
                fell: false,
                reached,
            };
        }
        if st.origin.z < floor - FELL_BELOW {
            return RolloutResult {
                time: t,
                finished: false,
                fell: true,
                reached,
            };
        }
    }
    RolloutResult {
        time: t,
        finished: false,
        fell: false,
        reached,
    }
}

/// The remaining straight runway ahead of `cursor` on the line: sum segment lengths while the heading
/// stays within [`RUNWAY_BEND`] of the first segment (mirrors the navmesh `runway` measure the live
/// bhop eligibility uses).
fn line_runway(pts: &[LinePoint], cursor: usize) -> f32 {
    let last = pts.len() - 1;
    if cursor >= last {
        return 0.0;
    }
    let base = (pts[cursor + 1].pos - pts[cursor].pos).xy().normalize_or_zero();
    let mut total = 0.0;
    for i in cursor..last {
        let seg = (pts[i + 1].pos - pts[i].pos).xy();
        let len = seg.length();
        if len < 1e-3 {
            continue;
        }
        let cos = base.dot(seg / len).clamp(-1.0, 1.0);
        if cos.acos().to_degrees() > RUNWAY_BEND {
            break;
        }
        total += len;
    }
    total
}

/// The optimization objective: finish time when the run completes, else the timeout plus penalties
/// for each unreached node and for falling — so "got farther" and "didn't fall" both improve the
/// score even before a run first completes, giving the search a gradient to climb.
pub fn rollout_cost(r: &RolloutResult, nodes: usize, timeout: f32) -> f32 {
    if r.finished {
        r.time
    } else {
        timeout + (nodes.saturating_sub(r.reached)) as f32 * timeout + if r.fell { timeout } else { 0.0 }
    }
}

/// Hand-rolled (1+1)-ES: mutate the best genome by a per-gene uniform step, keep the mutant if it
/// scores better, and adapt the step (Rechenberg-style: grow on success, shrink on failure). Seeded
/// deterministically. `bound` clamps each gene (the corridor half-width scale). Returns the best
/// genome and its cost. Generic over the objective so it can be unit-tested without a BSP.
pub fn optimize<F: Fn(&LineParams) -> f32>(
    seed: u32,
    init: LineParams,
    iters: u32,
    bound: f32,
    eval: F,
) -> (LineParams, f32) {
    let mut rng = Prng::new(seed);
    let mut best = init;
    let mut best_cost = eval(&best);
    let mut sigma = bound * 0.5;
    for _ in 0..iters {
        let mut cand = best.clone();
        for g in cand.lateral.iter_mut() {
            *g = (*g + rng.sym() * sigma).clamp(-bound, bound);
        }
        let cost = eval(&cand);
        if cost < best_cost {
            best = cand;
            best_cost = cost;
            sigma = (sigma * 1.2).min(bound);
        } else {
            sigma = (sigma * 0.85).max(bound * 0.01);
        }
    }
    (best, best_cost)
}

/// Control-point spacing along a route when building a line to optimize.
const RESAMPLE_SPACING: f32 = 192.0;
/// Widest sideways offset the optimizer may probe / apply at a control point.
const MAX_HALF_WIDTH: f32 = 128.0;

/// The corridor half-width at each control point: how far the line may be shoved sideways before a
/// wall, from a perpendicular hull trace each way (the nearer wall wins). Endpoints are pinned (0).
fn probe_half_widths(bsp: &Bsp, pts: &[Vec3]) -> Vec<f32> {
    (0..pts.len())
        .map(|i| {
            if i == 0 || i + 1 >= pts.len() {
                return 0.0;
            }
            let tangent = (pts[i + 1] - pts[i - 1]).xy().normalize_or_zero();
            let perp = Vec3::new(-tangent.y, tangent.x, 0.0);
            let reach = |d: Vec3| bsp.hull1_trace(pts[i], pts[i] + d * MAX_HALF_WIDTH).fraction * MAX_HALF_WIDTH;
            reach(perp).min(reach(-perp)).max(0.0)
        })
        .collect()
}

/// Optimize a racing line for one route: resample the route polyline, probe its corridor, then run
/// the (1+1)-ES with the finish-time rollout as the objective. Pure over the BSP — this is the unit
/// of work a race-line optimization worker thread runs per route. Returns the best line found.
pub fn optimize_route(
    bsp: &Bsp,
    polyline: &[Vec3],
    nodes: &[RaceRouteNode],
    pm: &PmParams,
    timeout: f32,
    seed: u32,
    iters: u32,
) -> RaceLine {
    let resampled = resample(polyline, RESAMPLE_SPACING);
    let hw = probe_half_widths(bsp, &resampled);
    let n = resampled.len();
    let eval = |p: &LineParams| {
        let line = decode(&resampled, p, &hw);
        rollout_cost(&rollout(bsp, &line, nodes, pm, timeout), nodes.len(), timeout)
    };
    let bound = hw.iter().copied().fold(1.0f32, f32::max);
    let (best, _) = optimize(seed, LineParams { lateral: vec![0.0; n] }, iters, bound, eval);
    decode(&resampled, &best, &hw)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prng_deterministic_and_bounded() {
        let mut a = Prng::new(12345);
        let mut b = Prng::new(12345);
        for _ in 0..1000 {
            let x = a.unit();
            assert_eq!(x.to_bits(), b.unit().to_bits(), "same seed must reproduce the sequence");
            assert!((0.0..1.0).contains(&x), "unit out of range: {x}");
            let s = a.sym();
            b.sym();
            assert!((-1.0..1.0).contains(&s), "sym out of range: {s}");
        }
        // Different seeds diverge.
        assert_ne!(Prng::new(1).next_u32(), Prng::new(2).next_u32());
    }

    #[test]
    fn resample_is_uniform_and_keeps_endpoints() {
        let pts = [Vec3::ZERO, Vec3::new(1000.0, 0.0, 0.0)];
        let r = resample(&pts, 100.0);
        assert_eq!(r.first().unwrap().x, 0.0);
        assert!((r.last().unwrap().x - 1000.0).abs() < 1e-3);
        // ~11 points at 100u spacing over 1000u.
        assert!((9..=12).contains(&r.len()), "unexpected resample count {}", r.len());
    }

    #[test]
    fn decode_shifts_perpendicular_and_clamps() {
        // A straight line along +x: a positive lateral gene shifts interior points to +y (left-hand
        // normal of +x), clamped to the corridor half-width.
        let poly = vec![Vec3::ZERO, Vec3::new(100.0, 0.0, 0.0), Vec3::new(200.0, 0.0, 0.0)];
        let params = LineParams {
            lateral: vec![0.0, 999.0, 0.0],
        }; // huge → must clamp
        let line = decode(&poly, &params, &[0.0, 64.0, 0.0]);
        assert!(
            (line.points[1].pos.y - 64.0).abs() < 1e-3,
            "not clamped to half-width: {}",
            line.points[1].pos.y
        );
        assert!((line.points[1].pos.x - 100.0).abs() < 1e-3, "tangent position drifted");
        // Endpoints never move.
        assert_eq!(line.points[0].pos, Vec3::ZERO);
        assert_eq!(line.points[2].pos, Vec3::new(200.0, 0.0, 0.0));
    }

    #[test]
    fn optimize_minimizes_and_is_deterministic() {
        // A synthetic objective (no BSP): drive each gene toward a distinct target inside the bound.
        let target = [30.0f32, -50.0, 12.0, -8.0];
        let eval = |p: &LineParams| p.lateral.iter().zip(target).map(|(g, t)| (g - t).powi(2)).sum::<f32>();
        let init = LineParams { lateral: vec![0.0; 4] };
        let (best, cost) = optimize(42, init.clone(), 4000, 64.0, eval);
        assert!(cost < 5.0, "optimizer failed to converge: cost {cost}");
        for (g, t) in best.lateral.iter().zip(target) {
            assert!((g - t).abs() < 2.0, "gene {g} far from target {t}");
        }
        // Same seed → identical result.
        let (best2, cost2) = optimize(42, init, 4000, 64.0, eval);
        assert_eq!(cost.to_bits(), cost2.to_bits());
        assert_eq!(best.lateral, best2.lateral);
    }
}
