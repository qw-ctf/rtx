// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot grenade play — proactively **lobbing** grenades and chaining **lob→shoot combos**, using the
//! blast's knockback both for airburst damage and to **shove opponents into hazards** (lava, slime,
//! pits, ledges). It layers on top of [`crate::bot::combat`]: the combat overlay handles the direct
//! gunfight; this module decides when a grenade is the better play, aims the arc, then detonates it
//! at the moment its blast pushes the enemy where the bot wants them.
//!
//! Everything routes through the usercmd (`look`/`move_world`/`buttons`/`impulse`) like the rest of
//! the bot code — the module never fires a weapon directly. The ballistic and knockback math is
//! factored into pure functions (unit-tested) so the engine coupling is pinned down and verifiable
//! offline; the stateful combo driver reads the world and drives those.

use glam::{Vec3, Vec3Swizzles};

use crate::bot::{self, BotCmd};
use crate::bot::combat::{
    self, blast_self_damage, can_hit_grenade, hitscan_choice, shoot_grenade, teammate_in_blast, GRENADE_MIN_SHOOT,
    GRENADE_SHOOT_HEALTH_FRAC,
};
use crate::defs::{
    Bits, Content, Flags, Items, MoveType, Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK,
    VEC_VIEW_OFS,
};
use crate::bot::state::GrenadePhase;
use crate::entity::EntId;
use crate::game::GameState;

/// Impulse that selects the grenade launcher.
pub(crate) const GL_IMPULSE: i32 = 6;
/// Range band in which a lob combo makes sense (closer = fight; farther = out of GL range).
const LOB_MIN_RANGE: f32 = 200.0;
const LOB_MAX_RANGE: f32 = 500.0;
/// Accepted miss of the simulated lob arc vs. the target, for the clearance check.
pub(crate) const LOB_LAND_TOL: f32 = 48.0;
/// Blast offset behind the enemy (toward the near side, away from the hazard) so the outward
/// knockback drives them into it.
const SHOVE_OFFSET: f32 = 72.0;
/// Combo restart throttles: after a failed/aborted attempt, and after a completed one.
const COMBO_COOLDOWN: f32 = 4.0;
const COMBO_DONE_COOLDOWN: f32 = 1.5;
/// Bank-shot restart throttles — longer, so a blind lob that found no path isn't re-attempted every
/// few seconds.
const BANK_COOLDOWN: f32 = 6.0;
const BANK_DONE_COOLDOWN: f32 = 3.0;
/// Give up a windup / an uncaptured lob after these.
const WINDUP_TIMEOUT: f32 = 1.2;
const CAPTURE_TIMEOUT: f32 = 0.3;
/// Throw once the smoothed view is within this many degrees of the lob solution.
const LOB_AIM_TOL: f32 = 2.5;

// --- grenade launcher physics (mirrors `w_fire_grenade`, weapons.rs) ---

/// Grenade launch speed: `|(600 forward, 200 up)| = √(600² + 200²)`.
pub(crate) const GL_SPEED: f32 = 632.455_5;
/// Fixed loft of the launch above the view-forward direction: `atan2(200, 600)`.
const GL_LOFT_DEG: f32 = 18.434_95;
/// Grenade fuse (seconds) before it self-detonates.
pub(crate) const GL_FUSE: f32 = 2.5;
/// The whole arc must land in less than this, leaving time to switch to a detonator and shoot
/// (the GL's own 0.6 s cooldown swallows the switch impulse first).
const LOB_MAX_FLIGHT: f32 = GL_FUSE - 0.8;

/// The grenade launch velocity for a given view — `600·forward + 200·up`, exactly as
/// `w_fire_grenade` builds it (minus the ±10 u random spread). Fixed magnitude [`GL_SPEED`] at
/// [`GL_LOFT_DEG`] above the view-forward.
pub(crate) fn launch_velocity(view: Vec3) -> Vec3 {
    let (forward, _right, up) = crate::bot::angle_vectors(view);
    forward * 600.0 + up * 200.0
}

/// Solve the view angles that lob a grenade from `p0` (the player origin — grenades spawn there) to
/// land at `b`, given the fixed launch speed `s` and gravity `g`. Because the launch sits a fixed
/// [`GL_LOFT_DEG`] above the view-forward, the view pitch is `loft − φ` where `φ` is the ballistic
/// launch elevation. `high` picks the lofted arc (clears walls) over the flat one (faster, harder to
/// dodge). Returns the `(look, flight_time)`; `None` when out of range or the arc takes too long.
pub(crate) fn solve_lob(p0: Vec3, b: Vec3, s: f32, g: f32, high: bool) -> Option<(Vec3, f32)> {
    let to = b - p0;
    let r = to.xy().length();
    if r < 1.0 {
        return None; // straight up/down — not a lob
    }
    let dz = to.z;
    let s2 = s * s;
    // tanφ = (s² ± √(s⁴ − g(gR² + 2·dz·s²))) / (gR); minus = flat arc, plus = lofted.
    let disc = s2 * s2 - g * (g * r * r + 2.0 * dz * s2);
    if disc < 0.0 {
        return None; // out of range
    }
    let root = disc.sqrt();
    let tan_phi = if high {
        (s2 + root) / (g * r)
    } else {
        (s2 - root) / (g * r)
    };
    let phi = tan_phi.atan(); // launch elevation (radians)
    let flight = r / (s * phi.cos()).max(1.0);
    if flight > LOB_MAX_FLIGHT {
        return None;
    }
    let mut pitch = GL_LOFT_DEG - phi.to_degrees();
    if pitch == 0.0 {
        pitch = -0.01; // v_angle.x == 0 takes a different velocity branch in w_fire_grenade
    }
    let yaw = to.y.atan2(to.x).to_degrees();
    Some((Vec3::new(pitch.clamp(-80.0, 80.0), yaw, 0.0), flight))
}

/// Solve the view angles that intercept a **free-falling** enemy with a grenade. The trick: gravity
/// pulls the grenade and the airborne enemy down by the same `½g·t²`, so it *cancels* in relative
/// motion — the meet reduces to the straight-line intercept ([`combat::intercept_time`]) at the
/// fixed launch speed [`GL_SPEED`]. The required launch velocity is then `v_g = (e_pos − p0)/t +
/// e_vel` (its magnitude is `GL_SPEED` by that construction), and the view pitch removes the fixed
/// [`GL_LOFT_DEG`] loft just like [`solve_lob`] (`pitch = loft − elevation`). Grenades spawn at the
/// player origin, so `p0` is exact. Returns `(look, flight_time, meet_point)`; `None` when the
/// enemy outruns the grenade, the flight can't finish before the fuse, or the view pitch clamps.
///
/// The engine adds a ±10 u launch spread (`w_fire_grenade`) — about 0.9°, ~6 u at 400 u, inside the
/// ±16 u hull at airshot ranges — so the closed-form solve lands the touch explosion in practice.
pub(crate) fn solve_air_intercept(p0: Vec3, e_pos: Vec3, e_vel: Vec3, g: f32) -> Option<(Vec3, f32, Vec3)> {
    let t = crate::bot::combat::intercept_time(e_pos - p0, e_vel, GL_SPEED)?;
    // The grenade touch-explodes on the enemy, so only the fuse bounds the flight; leave a margin.
    if !(0.1..(GL_FUSE - 0.15)).contains(&t) {
        return None;
    }
    let v_g = (e_pos - p0) / t + e_vel;
    let horiz = v_g.xy().length();
    let elev = v_g.z.atan2(horiz.max(1.0)); // launch elevation above horizontal (radians)
    let mut pitch = GL_LOFT_DEG - elev.to_degrees();
    if pitch == 0.0 {
        pitch = -0.01; // v_angle.x == 0 takes a different velocity branch in w_fire_grenade
    }
    if !(-80.0..=80.0).contains(&pitch) {
        return None; // out of the view-pitch range; the loft would bend the solution
    }
    let yaw = v_g.y.atan2(v_g.x).to_degrees();
    // Where they actually meet in the world (both dropped by ½g·t²) — for aim memory and the gate.
    let meet = e_pos + e_vel * t - Vec3::new(0.0, 0.0, 0.5 * g * t * t);
    Some((Vec3::new(pitch, yaw, 0.0), t, meet))
}

/// Solve a flat lob onto a **grounded, moving** target's feet, leading its horizontal motion over
/// the flight: solve once to get the flight time, advance the feet by `vel_xy·flight`, re-solve. Two
/// rounds settle a strafing target well inside the blast. Returns `(look, flight_time, led_feet)`;
/// `None` when even the static lob is out of range. Used as the RL-less fallback in [`combat::engage`].
pub(crate) fn solve_ground_lead(p0: Vec3, feet: Vec3, vel_xy: Vec3, g: f32) -> Option<(Vec3, f32, Vec3)> {
    let (_, flight) = solve_lob(p0, feet, GL_SPEED, g, false)?;
    let led = feet + vel_xy * flight;
    let (look, flight) = solve_lob(p0, led, GL_SPEED, g, false)?;
    Some((look, flight, led))
}

/// Estimate whether the knockback from a blast at `b` actually carries the enemy across the hazard
/// edge: the shove impulse's horizontal reach (grounded slide ≈ `0.25·|v_xy|`, or the airborne carry
/// when launched) must exceed `edge_dist`, and it must point along `shove_dir`. This is what turns a
/// "push roughly toward the lava" into a committed "they land in it".
pub(crate) fn shove_reaches(e_center: Vec3, e_origin: Vec3, b: Vec3, shove_dir: Vec3, edge_dist: f32) -> bool {
    let imp = predict_shove(e_center, e_origin, b);
    let horiz = imp.xy();
    if horiz.normalize_or_zero().dot(shove_dir.xy().normalize_or_zero()) < 0.6 {
        return false;
    }
    // Airborne launch carries much farther than a ground slide; approximate both conservatively.
    let reach = if imp.z > 150.0 {
        horiz.length() * 0.5
    } else {
        horiz.length() * 0.25
    };
    reach > edge_dist + 16.0
}

// --- multi-bounce bank shots (indirect fire, no line of sight) ---

/// Backoff of a `MOVETYPE_BOUNCE` reflection — the stock QuakeWorld `ClipVelocity` factor. It's an
/// energy-losing over-reflection off the surface, **not** a 2.0 mirror.
const BOUNCE_BACKOFF: f32 = 1.5;
/// Below this speed on a floor (`n.z > 0.7`) the grenade comes to rest.
const BOUNCE_REST_SPEED: f32 = 60.0;
/// Cap the bounces we bother simulating.
const BOUNCE_MAX: u8 = 8;
/// A pass this close to the enemy centre counts as a touch (bbox ±16 xy, grenade small) — the
/// grenade would explode on them.
const BANK_TOUCH: f32 = 32.0;
/// Fuse/rest detonation within this of the enemy is a "good" bank (≈ half the 160 blast radius, so
/// the splash still bites through the sim's hull/spread slop).
const BANK_GOOD: f32 = 90.0;
/// The robustness sweep (launch jitter) must still land within this.
const BANK_TOL: f32 = 140.0;

/// Reflect a velocity off a surface normal with the grenade bounce backoff (`v − 1.5·(v·n)·n`).
fn bounce_velocity(v: Vec3, n: Vec3) -> Vec3 {
    v - BOUNCE_BACKOFF * v.dot(n) * n
}

/// Distance from point `p` to the segment `a→b`.
fn point_seg_dist(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 < 1e-6 {
        0.0
    } else {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    };
    (a + ab * t - p).length()
}

/// The outcome of simulating a bouncing grenade thrown at `v0` from `p0`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct BounceSim {
    /// Where and when it detonates (a touch, a rest, or the 2.5 s fuse).
    pub det_pos: Vec3,
    pub det_time: f32,
    /// It passed through the (moving) enemy — an early touch explosion.
    pub hit_enemy: bool,
    pub bounces: u8,
    /// Closest the path came back to the thrower's own origin after leaving (`INF` if it never did).
    pub self_return: f32,
    /// Distance of the first bounce from the thrower (`INF` if it never bounced).
    pub first_bounce: f32,
}

/// Simulate a `MOVETYPE_BOUNCE` grenade: integrate the parabola, bounce off world geometry via the
/// `trace` oracle (reflecting with [`bounce_velocity`]), and detonate on touching the (moving) enemy
/// or on the fuse/rest — modelling `SV_Physics_Bounce` + `grenade_touch`. Pure over the oracles, so
/// it's unit-testable with a synthetic world. `enemy_at(t)` gives the led enemy position at time `t`.
pub(crate) fn simulate_bounce(
    trace: &impl Fn(Vec3, Vec3) -> crate::bsp::HullTrace,
    p0: Vec3,
    v0: Vec3,
    gravity: f32,
    enemy_at: &impl Fn(f32) -> Vec3,
) -> BounceSim {
    let (mut p, mut v, mut t) = (p0, v0, 0.0f32);
    let mut bounces = 0u8;
    let mut self_return = f32::INFINITY;
    let mut first_bounce = f32::INFINITY;
    let mut det: Option<(Vec3, f32, bool)> = None;

    while t < GL_FUSE {
        let dt = (16.0 / v.length().max(1.0)).min(0.02);
        v.z -= gravity * dt; // SV_AddGravity before the move
        let target = p + v * dt;
        let tr = trace(p, target);
        let seg_end = if tr.fraction < 1.0 { tr.endpos } else { target };

        let e = enemy_at(t);
        if t > 0.3 {
            self_return = self_return.min(point_seg_dist(p0, p, seg_end));
        }
        if point_seg_dist(e, p, seg_end) < BANK_TOUCH {
            det = Some((seg_end, t, true)); // explodes on the enemy
            break;
        }

        if tr.fraction < 1.0 {
            if first_bounce.is_infinite() {
                first_bounce = (tr.endpos - p0).length();
            }
            p = tr.endpos + tr.plane_normal * 0.25; // nudge off the surface
            v = bounce_velocity(v, tr.plane_normal);
            bounces += 1;
            t += tr.fraction * dt;
            if tr.plane_normal.z > 0.7 && v.length() < BOUNCE_REST_SPEED {
                det = Some((p, GL_FUSE, false)); // rests on the floor; fuse blows it there
                break;
            }
            if bounces > BOUNCE_MAX {
                det = Some((p, t, false));
                break;
            }
        } else {
            p = target;
            t += dt;
        }
    }
    let (det_pos, det_time, hit_enemy) = det.unwrap_or((p, GL_FUSE, false));
    BounceSim {
        det_pos,
        det_time,
        hit_enemy,
        bounces,
        self_return,
        first_bounce,
    }
}

/// A solved bank shot: the view to lob along, and where it detonates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct BankShot {
    pub look: Vec3,
    pub det_pos: Vec3,
}

/// Search launch angles for a bouncing lob that reaches an out-of-sight enemy. Samples a fan of
/// yaws around the bearing and a few lofts, simulates each, and picks the one that detonates closest
/// to the (led) enemy — preferring a direct touch, then fewer bounces. A robustness sweep rejects
/// knife-edge angles the ±10 u launch spread would spoil. `None` if nothing gets close.
pub(crate) fn solve_bank(
    trace: &impl Fn(Vec3, Vec3) -> crate::bsp::HullTrace,
    p0: Vec3,
    e_org: Vec3,
    e_vel: Vec3,
    gravity: f32,
) -> Option<BankShot> {
    let bearing = (e_org - p0).xy();
    let base_yaw = bearing.y.atan2(bearing.x).to_degrees();
    let enemy_at = |t: f32| e_org + e_vel * t.min(1.0);

    let mut best: Option<(f32, BankShot)> = None;
    for dyaw in [0.0f32, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0] {
        for pitch in [-40.0f32, -25.0, -10.0, 5.0] {
            let view = Vec3::new(pitch, base_yaw + dyaw, 0.0);
            let sim = simulate_bounce(trace, p0, launch_velocity(view), gravity, &enemy_at);
            let det_dist = (enemy_at(sim.det_time) - sim.det_pos).length();
            if !(sim.hit_enemy || det_dist < BANK_GOOD) {
                continue;
            }
            // Don't bank into our own feet or ricochet back past ourselves.
            if sim.self_return < 160.0 || sim.first_bounce < 64.0 {
                continue;
            }
            let score = (if sim.hit_enemy { 0.0 } else { det_dist }) + sim.bounces as f32 * 20.0 + sim.det_time * 5.0;
            if best.is_none_or(|(bs, _)| score < bs) {
                best = Some((
                    score,
                    BankShot {
                        look: view,
                        det_pos: sim.det_pos,
                    },
                ));
            }
        }
    }
    let (_, shot) = best?;
    // Launch-jitter robustness: the winner must survive small angle perturbations.
    for (dp, dy) in [(1.5f32, 0.0f32), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5)] {
        let view = Vec3::new(shot.look.x + dp, shot.look.y + dy, 0.0);
        let sim = simulate_bounce(trace, p0, launch_velocity(view), gravity, &enemy_at);
        let dd = (enemy_at(sim.det_time) - sim.det_pos).length();
        if !(sim.hit_enemy || dd < BANK_TOL) {
            return None;
        }
    }
    Some(shot)
}

// --- blast knockback ---

/// The velocity impulse a grenade blast at `b` imparts to a victim (origin `v_origin`, bbox centre
/// `v_center`): `8 · max(0, 120 − 0.5·dist) · normalize(v_origin − b)` — purely outward from the
/// blast (`t_damage`'s knockback, combat.rs). A blast placed *below and behind* the victim (toward
/// the far side from a hazard) therefore pushes them up-and-over toward it.
pub(crate) fn predict_shove(v_center: Vec3, v_origin: Vec3, b: Vec3) -> Vec3 {
    let points = (120.0 - 0.5 * (b - v_center).length()).max(0.0);
    (v_origin - b).normalize_or_zero() * points * 8.0
}

// --- hazard detection (shove targets) ---

/// The eight compass directions probed around an enemy for a shoveable hazard.
const HAZARD_DIRS: [(f32, f32); 8] = [
    (1.0, 0.0),
    (0.707, 0.707),
    (0.0, 1.0),
    (-0.707, 0.707),
    (-1.0, 0.0),
    (-0.707, -0.707),
    (0.0, -1.0),
    (0.707, -0.707),
];
/// Distances out from the enemy sampled for a hazard edge.
const HAZARD_RADII: [f32; 3] = [48.0, 96.0, 144.0];
/// A downward drop past this counts as a lethal/harmful fall to shove someone off (2·SAFE_FALL).
const HAZARD_DROP: f32 = 176.0;
/// How far down to look for a floor before calling it a pit.
const HAZARD_PROBE_DEPTH: f32 = 320.0;

/// What kind of hazard a direction leads to. Ordered by how much a bot should prefer to shove there.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum HazardKind {
    Slime,
    Pit, // a lethal/harmful fall (ledge or bottomless)
    Lava,
}

impl HazardKind {
    fn rank(self) -> u8 {
        match self {
            HazardKind::Lava => 3,
            HazardKind::Pit => 2,
            HazardKind::Slime => 1,
        }
    }
}

/// A shove opportunity near an enemy: the horizontal direction to push them, how far the hazard edge
/// is, and what it is.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Hazard {
    pub dir: Vec3,
    pub edge_dist: f32,
    pub kind: HazardKind,
}

/// Classify what's below `p` by marching down: lava/slime (from `contents`) or a big drop / pit
/// (from `is_solid`). `None` if solid ground sits close below (no hazard).
fn hazard_below(is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> f32, p: Vec3) -> Option<HazardKind> {
    let mut d = 0.0;
    while d <= HAZARD_PROBE_DEPTH {
        let q = p - Vec3::new(0.0, 0.0, d);
        let c = contents(q);
        if c == Content::Lava.as_f32() {
            return Some(HazardKind::Lava);
        }
        if c == Content::Slime.as_f32() {
            return Some(HazardKind::Slime);
        }
        if is_solid(q) {
            return (d > HAZARD_DROP).then_some(HazardKind::Pit);
        }
        d += 24.0;
    }
    Some(HazardKind::Pit) // no floor within reach — bottomless
}

/// Find the best hazard to shove an enemy (at `e_feet`) into: probe a ring of directions/distances,
/// classify each by what lies below (liquids via `contents`, drops via `is_solid`), and require a
/// clear horizontal path to it (a railing/wall between blocks the shove). Pure over the two oracles.
pub(crate) fn find_hazard(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> f32,
    e_feet: Vec3,
) -> Option<Hazard> {
    let mut best: Option<Hazard> = None;
    for (dx, dy) in HAZARD_DIRS {
        let dir = Vec3::new(dx, dy, 0.0);
        for r in HAZARD_RADII {
            let p = e_feet + dir * r + Vec3::new(0.0, 0.0, 8.0);
            // Reachable? The horizontal lane from the enemy out to the sample must be clear.
            let start = e_feet + dir * 24.0 + Vec3::new(0.0, 0.0, 8.0);
            let steps = ((p - start).length() / 16.0).ceil().max(1.0) as i32;
            let clear = (0..=steps).all(|i| !is_solid(start.lerp(p, i as f32 / steps as f32)));
            if !clear {
                break; // walled off in this direction — try the next
            }
            if let Some(kind) = hazard_below(is_solid, contents, p) {
                let cand = Hazard {
                    dir,
                    edge_dist: r,
                    kind,
                };
                let better = best
                    .is_none_or(|b| kind.rank() > b.kind.rank() || (kind.rank() == b.kind.rank() && r < b.edge_dist));
                if better {
                    best = Some(cand);
                }
                break; // found the near edge in this direction
            }
        }
    }
    best
}

// --- the combo driver (stateful; reads the world) ---

/// Reset the combo and set a restart cooldown.
fn combo_reset(game: &mut GameState, e: EntId, next_try: f32) {
    let b = &mut game.entities[e].bot;
    b.grenade_phase = GrenadePhase::Idle;
    b.grenade_ent = 0;
    b.grenade_bank = false;
    b.grenade_next_try = next_try;
}

/// A player's bbox centre (where radius damage/knockback are measured).
fn player_center(game: &GameState, p: EntId) -> Vec3 {
    let v = &game.entities[p].v;
    v.origin + (v.mins + v.maxs) * 0.5
}

/// The best hazard to shove enemy `en` into, if any — the shared detection both the grenade lob and
/// the rocket shove use. Builds the solidity/liquid oracles over the live BSP + `pointcontents`.
fn enemy_hazard(game: &GameState, en: EntId) -> Option<Hazard> {
    let e_feet = game.entities[en].v.origin - Vec3::new(0.0, 0.0, 24.0);
    let bsp = game.nav.bsp.as_ref();
    let host = game.host();
    let is_solid = |p: Vec3| bsp.is_some_and(|b| b.is_solid(p));
    let contents = |p: Vec3| host.pointcontents(p);
    find_hazard(&is_solid, &contents, e_feet)
}

/// Clear line from our eye to `target`'s eye.
fn los_to(game: &mut GameState, e: EntId, target: EntId) -> bool {
    let from = game.entities[e].v.origin + VEC_VIEW_OFS;
    let to = game.entities[target].v.origin + VEC_VIEW_OFS;
    let tr = game.traceline(from, to, false, e);
    tr.ent == target || tr.fraction > 0.95
}

/// Whether a grenade entity is still live (in flight, not yet detonated).
fn grenade_live(game: &GameState, g: EntId) -> bool {
    let ent = &game.entities[g];
    ent.in_use && ent.classname() == Some("grenade") && ent.combat.voided == 0.0
}

/// Find the bot's own just-fired grenade — a live grenade it owns, near its origin.
fn own_live_grenade(game: &GameState, e: EntId, origin: Vec3) -> Option<EntId> {
    game.entities
        .iter()
        .enumerate()
        .filter(|(_, ent)| ent.classname() == Some("grenade") && ent.in_use && ent.combat.voided == 0.0)
        .map(|(i, _)| EntId(i as u32))
        .filter(|&g| game.entities[g].owner() == e && (game.entities[g].v.origin - origin).length() < 120.0)
        .min_by(|&a, &b| {
            let d = |g: EntId| (game.entities[g].v.origin - origin).length();
            d(a).total_cmp(&d(b))
        })
}

/// Angular error (deg) between two view angles (pitch/yaw), the larger axis.
fn aim_err(view: Vec3, want: Vec3) -> f32 {
    bot::wrap180(view.x - want.x)
        .abs()
        .max(bot::wrap180(view.y - want.y).abs())
}

/// Drive the grenade lob→shoot combo for one frame, overlaid after `engage` (and after the
/// defensive grenade check, which wins). Picks up a fight where a lobbed-and-detonated grenade beats
/// the gunfight — shoving a grounded enemy into a nearby hazard, or a plain airburst — and runs the
/// [`GrenadePhase`] machine to aim, fire, track, and detonate it. Everything is written back through
/// the frame's [`BotCmd`]; safety (self-splash, teammates, wrong-way shoves, walls) is enforced at
/// every step.
pub(crate) fn grenade_combo(
    game: &mut GameState,
    e: EntId,
    enemy: Option<EntId>,
    origin: Vec3,
    now: f32,
    cmd: &mut BotCmd,
) {
    let phase = game.entities[e].bot.grenade_phase;
    // Losing the enemy cancels any in-progress combo (but a lobbed grenade can still fuse-blow).
    let Some(en) = enemy else {
        if phase != GrenadePhase::Idle {
            combo_reset(game, e, now + COMBO_DONE_COOLDOWN);
        }
        return;
    };
    match phase {
        GrenadePhase::Idle => try_start(game, e, en, origin, now),
        GrenadePhase::Windup => windup(game, e, now, cmd),
        GrenadePhase::Lobbed => lobbed(game, e, origin, now, cmd),
        GrenadePhase::Detonate => detonate(game, e, en, origin, now, cmd),
    }
}

/// Decide whether to start a combo, and if so aim the lob (Idle → Windup).
fn try_start(game: &mut GameState, e: EntId, en: EntId, origin: Vec3, now: f32) {
    let host = *game.host();
    if now < game.entities[e].bot.grenade_next_try || !host.cvar_bool(c"rtx_shootable_grenades") {
        return;
    }
    let (items, ammo_rockets, health) = {
        let v = &game.entities[e].v;
        (v.items, v.ammo_rockets, v.health)
    };
    if !items.has(Items::GRENADE_LAUNCHER) || ammo_rockets < 1.0 || health < 50.0 {
        return;
    }
    let e_org = game.entities[en].v.origin;
    let dist = (e_org - origin).length();
    if !(LOB_MIN_RANGE..=LOB_MAX_RANGE).contains(&dist) {
        return;
    }
    // The lob shove / airburst wants a grounded, walking target (knockback needs `movetype == Walk`,
    // and the enemy must stay put for the arc).
    let en_grounded = game.entities[en].v.flags.has(Flags::ONGROUND);
    if game.entities[en].v.movetype != MoveType::Walk || !en_grounded {
        return;
    }
    // No line of sight → this is a job for an indirect **bank shot** (fuse-detonated), not the
    // LOS-required lob→shoot combo.
    if !los_to(game, e, en) {
        try_start_bank(game, e, en, origin, now);
        return;
    }
    if hitscan_choice(game, e).is_none() {
        return; // the LOS combo needs a gun to detonate with (a bank shot uses the fuse)
    }
    let my_team = game.entities[e].mode_p.team;
    let e_feet = e_org - Vec3::new(0.0, 0.0, 24.0);

    // Look for a hazard to shove the enemy into; else consider a plain airburst.
    let (target, shove_dir, shove_edge) = match enemy_hazard(game, en) {
        Some(h) => (e_feet - h.dir * SHOVE_OFFSET, h.dir, h.edge_dist),
        None => {
            // Plain airburst: throttled so it seasons fights rather than replacing the gun game
            // (skill-gated; low-skill bots don't bother). Land it at the enemy's feet.
            let skill = host.cvar(c"rtx_bot_skill");
            if skill < 3.0 || (now * 7.0 + e.0 as f32).sin() < 0.6 {
                return;
            }
            (e_feet, Vec3::ZERO, 0.0)
        }
    };

    // Safety on the blast point: outside our own splash, no teammate caught, a clear line to the
    // enemy (so the blast actually reaches them), and — for a shove — not driving them toward us.
    let self_splash = blast_self_damage((target - origin).length()) * 0.5;
    if self_splash > health * GRENADE_SHOOT_HEALTH_FRAC
        || (target - origin).length() < GRENADE_MIN_SHOOT
        || teammate_in_blast(game, e, my_team, target)
    {
        combo_reset(game, e, now + COMBO_COOLDOWN);
        return;
    }
    if shove_dir != Vec3::ZERO {
        let to_bot = (origin - e_org).xy().normalize_or_zero();
        let e_center = player_center(game, en);
        // Bail unless the hazard is away from us AND the blast would actually shove them across the
        // edge (not just nudge them toward it).
        if shove_dir.xy().normalize_or_zero().dot(to_bot) > 0.5
            || !shove_reaches(e_center, e_org, target, shove_dir, shove_edge)
        {
            combo_reset(game, e, now + COMBO_COOLDOWN);
            return;
        }
    }
    {
        let tr = game.traceline(target + Vec3::new(0.0, 0.0, 8.0), e_org, false, en);
        if !(tr.ent == en || tr.fraction > 0.9) {
            combo_reset(game, e, now + COMBO_COOLDOWN);
            return;
        }
    }

    // Solve the lob (flat arc first, lofted as a fallback), and verify it clears geometry.
    let gravity = host.cvar(c"sv_gravity").max(1.0);
    let solved = [false, true].into_iter().find_map(|high| {
        let (look, _flight) = solve_lob(origin, target, GL_SPEED, gravity, high)?;
        let v0 = launch_velocity(look);
        let clear = match game.nav.bsp.as_ref() {
            Some(bsp) => crate::navmesh::arc_land(bsp, origin, v0, gravity)
                .is_some_and(|(land, _, _)| (land.xy() - target.xy()).length() < LOB_LAND_TOL * 2.0),
            None => true,
        };
        clear.then_some(look)
    });
    let Some(look) = solved else {
        combo_reset(game, e, now + COMBO_COOLDOWN);
        return;
    };

    let b = &mut game.entities[e].bot;
    b.grenade_phase = GrenadePhase::Windup;
    b.grenade_started = now;
    b.grenade_target = target;
    b.grenade_look = look;
    b.grenade_shove_dir = shove_dir;
    b.grenade_shove_edge = shove_edge;
    b.grenade_ent = 0;
    b.grenade_bank = false;
}

/// Try to start an indirect **bank shot** at an out-of-sight enemy (Idle → Windup): search the
/// bouncing-grenade solver for a lob that banks off geometry to reach them, and if one exists, aim
/// it. Detonation is left to the fuse (the bot can't see the grenade to shoot it). Gated to keep it
/// honest and rare: a fresh belief in the enemy's position, a slow target, higher skill, and a real
/// solution — flag carriers bypass the frequency throttle as a persistent threat worth a blind lob.
fn try_start_bank(game: &mut GameState, e: EntId, en: EntId, origin: Vec3, now: f32) {
    let host = *game.host();
    if host.cvar(c"rtx_bot_skill") < 4.0 || game.nav.bsp.is_none() {
        return;
    }
    let e_org = game.entities[en].v.origin;
    let e_vel = game.entities[en].v.velocity;
    if e_vel.xy().length() > 120.0 {
        combo_reset(game, e, now + BANK_COOLDOWN); // too mobile to predict over a ~2s flight
        return;
    }
    // Only bank at a position we actually saw recently — don't lob blindly at a stale origin.
    let seen = game.entities[e].bot.enemy_seen_time;
    if seen <= 0.0 || now - seen > 3.0 {
        combo_reset(game, e, now + BANK_COOLDOWN);
        return;
    }
    // Season it in: an occasional attempt, unless the enemy carries a flag (worth a blind grenade).
    let carrier = game.entities[en].mode_p.ctf.carrying != 0;
    if !carrier && (now * 5.0 + e.0 as f32).sin() < 0.7 {
        return;
    }
    let gravity = host.cvar(c"sv_gravity").max(1.0);
    let shot = {
        let Some(bsp) = game.nav.bsp.as_ref() else {
            return;
        };
        let trace = |a: Vec3, b: Vec3| bsp.hull1_trace(a, b);
        solve_bank(&trace, origin, e_org, e_vel, gravity)
    };
    let Some(shot) = shot else {
        combo_reset(game, e, now + BANK_COOLDOWN);
        return;
    };
    let my_team = game.entities[e].mode_p.team;
    if teammate_in_blast(game, e, my_team, shot.det_pos) {
        combo_reset(game, e, now + BANK_COOLDOWN);
        return;
    }
    let b = &mut game.entities[e].bot;
    b.grenade_phase = GrenadePhase::Windup;
    b.grenade_started = now;
    b.grenade_target = shot.det_pos;
    b.grenade_look = shot.look;
    b.grenade_shove_dir = Vec3::ZERO;
    b.grenade_shove_edge = 0.0;
    b.grenade_ent = 0;
    b.grenade_bank = true;
}

/// Select the GL, aim the lob, and fire once the smoothed view is on it (Windup → Lobbed).
fn windup(game: &mut GameState, e: EntId, now: f32, cmd: &mut BotCmd) {
    if now - game.entities[e].bot.grenade_started > WINDUP_TIMEOUT {
        combo_reset(game, e, now + COMBO_COOLDOWN);
        return;
    }
    let want = game.entities[e].bot.grenade_look;
    cmd.look = want;
    cmd.move_world = Vec3::ZERO; // hold the firing stance
    cmd.buttons &= !BUTTON_ATTACK; // don't fire the current gun at lob pitch
    let (weapon, attack_finished) = {
        let ent = &game.entities[e];
        (ent.v.weapon, ent.combat.attack_finished)
    };
    if weapon != Weapon::GrenadeLauncher {
        cmd.impulse = GL_IMPULSE;
        return;
    }
    // Grenade launcher in hand: fire when the smoothed aim has settled and the GL is off cooldown.
    if now >= attack_finished && aim_err(game.entities[e].bot.aim, want) < LOB_AIM_TOL {
        cmd.buttons |= BUTTON_ATTACK;
        let b = &mut game.entities[e].bot;
        b.grenade_phase = GrenadePhase::Lobbed;
        b.grenade_started = now; // now the fuse clock
    }
}

/// Capture the fired grenade and switch to the detonator (Lobbed → Detonate).
fn lobbed(game: &mut GameState, e: EntId, origin: Vec3, now: f32, cmd: &mut BotCmd) {
    if game.entities[e].bot.grenade_ent == 0 {
        match own_live_grenade(game, e, origin) {
            Some(g) => game.entities[e].bot.grenade_ent = g.0,
            None => {
                if now - game.entities[e].bot.grenade_started > CAPTURE_TIMEOUT {
                    combo_reset(game, e, now + COMBO_COOLDOWN); // the shot never produced a grenade
                }
                return;
            }
        }
    }
    let g = EntId(game.entities[e].bot.grenade_ent);
    if !grenade_live(game, g) {
        combo_reset(game, e, now + COMBO_DONE_COOLDOWN); // already went off (touch/fuse) — done
        return;
    }
    // Bank shot: no line of sight to the grenade, so don't switch to a detonator or yank the view at
    // an unseen wall — just let the fuse blow it near the enemy. Navigation/combat keep the bot
    // moving (often back toward line of sight) while it burns. A backstop reset covers a lost track.
    if game.entities[e].bot.grenade_bank {
        if now - game.entities[e].bot.grenade_started > GL_FUSE + 0.5 {
            combo_reset(game, e, now + BANK_DONE_COOLDOWN);
        }
        return;
    }
    // Keep requesting the detonator every frame — the switch impulse is swallowed until the GL's
    // 0.6s cooldown ends, so a one-shot request would be lost.
    if let Some((imp, weapon)) = hitscan_choice(game, e) {
        if game.entities[e].v.weapon != weapon {
            cmd.impulse = imp;
        } else {
            game.entities[e].bot.grenade_phase = GrenadePhase::Detonate;
        }
    } else {
        combo_reset(game, e, now + COMBO_COOLDOWN);
        return;
    }
    let eye = origin + VEC_VIEW_OFS;
    let gpos = game.entities[g].v.origin;
    cmd.look = combat::angles_to(eye, gpos); // pre-aim the grenade
}

/// Detonate the grenade the instant its blast puts the enemy where we want them (Detonate → Idle).
fn detonate(game: &mut GameState, e: EntId, en: EntId, origin: Vec3, now: f32, cmd: &mut BotCmd) {
    let g = EntId(game.entities[e].bot.grenade_ent);
    if game.entities[e].bot.grenade_ent == 0 || !grenade_live(game, g) {
        combo_reset(game, e, now + COMBO_DONE_COOLDOWN); // detonated (by us or fuse) — done
        return;
    }
    // Fuse backstop: if we've held too long, stop and let it blow on its own.
    if now - game.entities[e].bot.grenade_started > GL_FUSE {
        combo_reset(game, e, now + COMBO_DONE_COOLDOWN);
        return;
    }
    let (health, my_team) = (game.entities[e].v.health, game.entities[e].mode_p.team);
    let gpos = game.entities[g].v.origin;
    let eye = origin + VEC_VIEW_OFS;
    cmd.look = combat::angles_to(eye, gpos);

    // Never blow it up in our own face, or on a teammate. Back away if it's drifted too close.
    let d_self = (gpos - origin).length();
    if blast_self_damage(d_self) * 0.5 > health * GRENADE_SHOOT_HEALTH_FRAC
        || d_self < GRENADE_MIN_SHOOT
        || teammate_in_blast(game, e, my_team, gpos)
    {
        if d_self < 200.0 {
            let away = Vec3::new(origin.x - gpos.x, origin.y - gpos.y, 0.0).normalize_or_zero();
            cmd.move_world = away * MOVE_SPEED;
        }
        return; // hold fire; the fuse is the backstop
    }

    // Is the geometry right? For a shove, the grenade must sit close to the enemy on the correct
    // side so the outward push drives them toward the hazard; for an airburst, just close.
    let shove_dir = game.entities[e].bot.grenade_shove_dir;
    let shove_edge = game.entities[e].bot.grenade_shove_edge;
    let e_center = player_center(game, en);
    let e_org = game.entities[en].v.origin;
    let fire_ok = if shove_dir != Vec3::ZERO {
        // Detonate against the grenade's *actual* position (it bounced): close to the enemy, and the
        // live blast geometry must still carry them across the hazard edge.
        (gpos - e_center).length() < 140.0 && shove_reaches(e_center, e_org, gpos, shove_dir, shove_edge)
    } else {
        (gpos - e_center).length() < 100.0
    };
    if fire_ok && can_hit_grenade(game, e, g) {
        shoot_grenade(game, e, g, cmd);
    }
}

/// The **generic hazard shove**, rocket-launcher variant. The knockback trick isn't grenade-specific
/// — any splash weapon shoves — and a rocket doesn't need a *direct hit*: one put on the **ground
/// just behind the enemy** (the same blast point `B` the grenade lob targets, on the far side from
/// the hazard) shoves them into it by splash alone, from wherever the bot has a clear line to that
/// spot. That's more flexible than a body shot (a static ground point is easy to hit, and the bot
/// needn't stand on any particular side) and just as strong. The lob is the fallback for when the
/// blast has to be *arced over* the enemy instead. Returns whether it took over the frame.
pub(crate) fn rocket_shove(
    game: &mut GameState,
    e: EntId,
    enemy: Option<EntId>,
    origin: Vec3,
    cmd: &mut BotCmd,
) -> bool {
    let Some(en) = enemy else {
        return false;
    };
    let (items, ammo, health, my_team) = {
        let ent = &game.entities[e];
        (ent.v.items, ent.v.ammo_rockets, ent.v.health, ent.mode_p.team)
    };
    if !items.has(Items::ROCKET_LAUNCHER) || ammo < 1.0 {
        return false;
    }
    let (e_org, e_vel, en_grounded, en_walk) = {
        let v = &game.entities[en].v;
        (
            v.origin,
            v.velocity,
            v.flags.has(Flags::ONGROUND),
            v.movetype == MoveType::Walk,
        )
    };
    let dist = (e_org - origin).length();
    // The target must be a grounded, walking, not-sprinting player (so the ground point stays valid
    // for the rocket's short flight); and it can't be point-blank (self-splash).
    if !(200.0..=700.0).contains(&dist) || !en_grounded || !en_walk || e_vel.xy().length() > 250.0 {
        return false;
    }
    let Some(h) = enemy_hazard(game, en) else {
        return false;
    };
    // The blast point: on the ground behind the enemy, away from the hazard, so the outward splash
    // drives them into it. Same `B` as the lob — only the delivery differs.
    let e_feet = e_org - Vec3::new(0.0, 0.0, 24.0);
    let b = e_feet - h.dir * SHOVE_OFFSET;
    let e_center = player_center(game, en);
    if !shove_reaches(e_center, e_org, b, h.dir, h.edge_dist) || teammate_in_blast(game, e, my_team, b) {
        return false;
    }
    // Don't splash ourselves (attacker damage is halved), and don't stand on top of the blast.
    let d_self = (b - origin).length();
    if blast_self_damage(d_self) * 0.5 > health * GRENADE_SHOOT_HEALTH_FRAC || d_self < GRENADE_MIN_SHOOT {
        return false;
    }
    // We need a clear straight shot to `B`: the rocket must reach that ground spot, not detonate on
    // the enemy in the way (that would blast the wrong point) or on a wall short of it. If it can't,
    // this is a job for the lob (which arcs over) — bail so the combo takes it.
    let eye = origin + VEC_VIEW_OFS;
    let aim = b + Vec3::new(0.0, 0.0, 2.0);
    let tr = game.traceline(eye, aim, false, e);
    if tr.ent == en {
        return false; // enemy blocks the shot
    }
    let hit = eye + (aim - eye) * tr.fraction;
    if (hit - b).length() > 48.0 {
        return false; // a wall stops the rocket short of B
    }
    // Aim the rocket at the ground point, select the launcher, and fire once the view is on it.
    cmd.look = combat::angles_to(eye, aim);
    if game.entities[e].v.weapon != Weapon::RocketLauncher {
        cmd.impulse = 7;
        cmd.buttons &= !BUTTON_ATTACK;
        return true;
    }
    if aim_err(game.entities[e].bot.aim, cmd.look) < 4.0 {
        cmd.buttons |= BUTTON_ATTACK;
    } else {
        cmd.buttons &= !BUTTON_ATTACK;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f32 = 800.0;

    // Oracle helpers: a flat floor at z ≤ 0 by default.
    fn floor(p: Vec3) -> bool {
        p.z <= 0.0
    }

    #[test]
    fn finds_lava_edge_and_direction() {
        // Lava fills x > 200 (as a liquid — not solid); solid floor at z ≤ 0 for x ≤ 200.
        let solid = |p: Vec3| p.z <= 0.0 && p.x <= 200.0;
        let contents = |p: Vec3| {
            if p.x > 200.0 && p.z < 0.0 {
                Content::Lava.as_f32()
            } else {
                Content::Empty.as_f32()
            }
        };
        let e_feet = Vec3::new(160.0, 0.0, 24.0); // enemy near the lava edge
        let h = find_hazard(&solid, &contents, e_feet).expect("lava found");
        assert_eq!(h.kind, HazardKind::Lava);
        assert!(h.dir.x > 0.5, "should push toward +x (the lava): {:?}", h.dir);
    }

    #[test]
    fn finds_pit() {
        // Floor at z ≤ 0 for x ≤ 200; bottomless past it.
        let solid = |p: Vec3| p.z <= 0.0 && p.x <= 200.0;
        let empty = |_: Vec3| Content::Empty.as_f32();
        let h = find_hazard(&solid, &empty, Vec3::new(170.0, 0.0, 24.0)).expect("pit found");
        assert_eq!(h.kind, HazardKind::Pit);
        assert!(h.dir.x > 0.5);
    }

    #[test]
    fn railing_blocks_the_shove() {
        // Lava past x > 200, but a wall spans 96 < x < 104 for all z between enemy and it.
        let solid = |p: Vec3| (p.z <= 0.0 && p.x <= 200.0) || (96.0..104.0).contains(&p.x);
        let contents = |p: Vec3| {
            if p.x > 200.0 && p.z < 0.0 {
                Content::Lava.as_f32()
            } else {
                Content::Empty.as_f32()
            }
        };
        assert!(find_hazard(&solid, &contents, Vec3::new(40.0, 0.0, 24.0)).is_none());
    }

    #[test]
    fn open_floor_has_no_hazard() {
        let empty = |_: Vec3| Content::Empty.as_f32();
        assert!(find_hazard(&floor, &empty, Vec3::new(0.0, 0.0, 24.0)).is_none());
    }

    /// The launch matches `w_fire_grenade`: fixed speed, and elevation = view-up angle + loft.
    #[test]
    fn launch_speed_and_loft() {
        for pitch in [0.0f32, -20.0, -45.0, 15.0, 30.0] {
            let v = launch_velocity(Vec3::new(pitch, 37.0, 0.0));
            assert!(
                (v.length() - GL_SPEED).abs() < 0.01,
                "speed off at pitch {pitch}: {}",
                v.length()
            );
            let elev = v.z.atan2(v.xy().length()).to_degrees();
            // view elevation is −pitch; launch sits loft above it.
            assert!(
                (elev - (-pitch + GL_LOFT_DEG)).abs() < 0.01,
                "elevation off at pitch {pitch}: {elev}"
            );
        }
    }

    /// Integrate the solved arc to the flight time and confirm it lands on the target.
    #[test]
    fn solve_lob_lands_on_target() {
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        for &(r, dz) in &[(300.0f32, 0.0f32), (250.0, 80.0), (400.0, -120.0), (150.0, 40.0)] {
            let b = Vec3::new(r, 0.0, dz);
            for high in [false, true] {
                let Some((solved, t)) = solve_lob(p0, b, GL_SPEED, G, high) else {
                    continue;
                };
                let v0 = launch_velocity(solved);
                let landed = p0 + v0 * t - Vec3::new(0.0, 0.0, 0.5 * G * t * t);
                assert!(
                    (landed - b).length() < 2.0,
                    "arc missed (r={r} dz={dz} high={high}): landed {landed:?} want {b:?}"
                );
            }
        }
    }

    /// Beyond the flat-ground range `s²/g` there is no solution.
    #[test]
    fn solve_lob_out_of_range() {
        let far = GL_SPEED * GL_SPEED / G + 100.0; // past max range
        assert!(solve_lob(Vec3::ZERO, Vec3::new(far, 0.0, 0.0), GL_SPEED, G, false).is_none());
        assert!(solve_lob(Vec3::ZERO, Vec3::new(far, 0.0, 0.0), GL_SPEED, G, true).is_none());
    }

    /// Knockback is outward from the blast, scaled by the splash points.
    #[test]
    fn shove_is_outward_and_scaled() {
        // Blast 60u to the −x side of the victim → push toward +x, magnitude 8·(120−0.5·60)=720.
        let v_center = Vec3::new(0.0, 0.0, 4.0);
        let v_origin = Vec3::ZERO;
        let b = Vec3::new(-60.0, 0.0, 4.0);
        let imp = predict_shove(v_center, v_origin, b);
        assert!(imp.x > 0.0 && imp.y.abs() < 1.0, "should push +x: {imp:?}");
        assert!((imp.length() - 720.0).abs() < 1.0, "magnitude off: {}", imp.length());
        // Outside the radius → no shove.
        assert_eq!(
            predict_shove(v_center, v_origin, Vec3::new(-400.0, 0.0, 4.0)).length(),
            0.0
        );
    }

    #[test]
    fn bounce_backoff_is_one_and_a_half() {
        // Straight down onto a floor (n = +z): vz reverses to −0.5·vz_in (the 1.5-not-2.0 pitfall).
        let out = bounce_velocity(Vec3::new(30.0, 0.0, -100.0), Vec3::new(0.0, 0.0, 1.0));
        assert!((out.z - 50.0).abs() < 0.01, "vz {}", out.z);
        assert!((out.x - 30.0).abs() < 0.01, "tangential x preserved"); // no change along the surface
    }

    // A trace oracle for an infinite floor at z = 0 (solid below), open above.
    fn floor_trace(a: Vec3, b: Vec3) -> crate::bsp::HullTrace {
        use crate::bsp::HullTrace;
        if b.z >= 0.0 {
            return HullTrace {
                fraction: 1.0,
                endpos: b,
                plane_normal: Vec3::ZERO,
                start_solid: false,
                all_solid: false,
            };
        }
        let f = if (a.z - b.z).abs() < 1e-6 {
            0.0
        } else {
            (a.z / (a.z - b.z)).clamp(0.0, 1.0)
        };
        HullTrace {
            fraction: f,
            endpos: a + (b - a) * f,
            plane_normal: Vec3::new(0.0, 0.0, 1.0),
            start_solid: a.z < 0.0,
            all_solid: false,
        }
    }

    #[test]
    fn bounce_fuse_in_open_matches_closed_form() {
        // No geometry → the grenade flies free and blows on the 2.5 s fuse at the parabola position.
        let open = |_: Vec3, b: Vec3| crate::bsp::HullTrace {
            fraction: 1.0,
            endpos: b,
            plane_normal: Vec3::ZERO,
            start_solid: false,
            all_solid: false,
        };
        let far = |_: f32| Vec3::new(100000.0, 0.0, 0.0); // enemy elsewhere, never touched
        let p0 = Vec3::ZERO;
        let v0 = launch_velocity(Vec3::new(-30.0, 0.0, 0.0));
        let sim = simulate_bounce(&open, p0, v0, G, &far);
        assert!(!sim.hit_enemy);
        assert!((sim.det_time - GL_FUSE).abs() < 0.03, "det_time {}", sim.det_time);
        let t = GL_FUSE;
        let want = p0 + v0 * t - Vec3::new(0.0, 0.0, 0.5 * G * t * t);
        // Semi-implicit Euler (matching the engine's SV_Physics_Toss) drifts ~20u from the
        // continuous parabola over the full fuse — inside the blast's slop, and the same scheme the
        // engine uses, so the sim tracks the real grenade rather than the ideal one.
        assert!(
            (sim.det_pos - want).length() < 40.0,
            "det_pos {:?} want {want:?}",
            sim.det_pos
        );
    }

    #[test]
    fn bounce_touches_enemy_on_the_path() {
        // Enemy standing in the flat throw's path → an early touch detonation before the fuse.
        let open = |_: Vec3, b: Vec3| crate::bsp::HullTrace {
            fraction: 1.0,
            endpos: b,
            plane_normal: Vec3::ZERO,
            start_solid: false,
            all_solid: false,
        };
        let v0 = launch_velocity(Vec3::new(18.435, 0.0, 0.0)); // level launch (elevation 0) along +x
                                                               // Enemy ~120u ahead, at the height the level throw has fallen to there → the path passes
                                                               // right through them.
        let enemy = |_: f32| Vec3::new(120.0, 0.0, 45.0);
        let sim = simulate_bounce(&open, Vec3::new(0.0, 0.0, 60.0), v0, G, &enemy);
        assert!(sim.hit_enemy, "should touch the enemy");
        assert!(sim.det_time < GL_FUSE);
    }

    #[test]
    fn solve_bank_open_space_and_out_of_range() {
        // In the open a "bank" degenerates to a direct arc onto the enemy → a solution exists.
        let sim = solve_bank(
            &floor_trace,
            Vec3::new(0.0, 0.0, 40.0),
            Vec3::new(300.0, 0.0, 40.0),
            Vec3::ZERO,
            G,
        );
        assert!(sim.is_some(), "should find a lob onto a reachable enemy");
        // Far out of grenade range → nothing lands close.
        let none = solve_bank(
            &floor_trace,
            Vec3::new(0.0, 0.0, 40.0),
            Vec3::new(1500.0, 0.0, 40.0),
            Vec3::ZERO,
            G,
        );
        assert!(none.is_none());
    }

    #[test]
    fn shove_reaches_near_edge_not_far() {
        // Blast 60u behind the enemy pushes +x hard (720ups → ~180u ground reach); dir = +x.
        let ec = Vec3::new(0.0, 0.0, 4.0);
        let eo = Vec3::ZERO;
        let b = Vec3::new(-60.0, 0.0, 4.0);
        let dir = Vec3::new(1.0, 0.0, 0.0);
        assert!(shove_reaches(ec, eo, b, dir, 100.0), "should clear a 100u edge");
        assert!(!shove_reaches(ec, eo, b, dir, 400.0), "shouldn't clear a 400u edge");
        // Wrong-direction hazard: pushing +x doesn't help a −x hazard.
        assert!(!shove_reaches(ec, eo, b, Vec3::new(-1.0, 0.0, 0.0), 50.0));
    }

    /// End-to-end proof of the gravity-cancellation trick: integrate the solved launch *and* the
    /// free-falling enemy under the same gravity to the solved flight time — they should meet.
    /// Repeated at g=800 and g=100 (e1m8) to show the solve is gravity-value-independent.
    #[test]
    fn air_intercept_meets_falling_enemy() {
        for &g in &[800.0f32, 100.0] {
            let p0 = Vec3::new(0.0, 0.0, 40.0);
            for &(e_pos, e_vel) in &[
                (Vec3::new(400.0, 0.0, 120.0), Vec3::new(0.0, 200.0, 250.0)), // rising + crossing
                (Vec3::new(300.0, 150.0, 200.0), Vec3::new(-100.0, 0.0, -150.0)), // falling
                (Vec3::new(250.0, -200.0, 90.0), Vec3::new(120.0, 60.0, 0.0)), // level dash
            ] {
                let (look, t, _meet) = solve_air_intercept(p0, e_pos, e_vel, g).expect("intercept exists");
                let drop = Vec3::new(0.0, 0.0, 0.5 * g * t * t);
                let grenade = p0 + launch_velocity(look) * t - drop;
                let enemy = e_pos + e_vel * t - drop;
                assert!(
                    (grenade - enemy).length() < 5.0,
                    "g={g}: grenade and enemy miss by {} at t={t}",
                    (grenade - enemy).length()
                );
            }
        }
    }

    /// The solved view really does launch along the required velocity (the loft subtraction, incl.
    /// the pitch-0 nudge, is exact).
    #[test]
    fn air_intercept_view_roundtrip() {
        let p0 = Vec3::new(0.0, 0.0, 40.0);
        let e_pos = Vec3::new(350.0, 120.0, 160.0);
        let e_vel = Vec3::new(-80.0, 40.0, 120.0);
        let (look, t, _) = solve_air_intercept(p0, e_pos, e_vel, G).unwrap();
        let want = (e_pos - p0) / t + e_vel; // required launch velocity
        let got = launch_velocity(look);
        assert!((got - want).length() < 1.0, "launch {got:?} vs required {want:?}");
    }

    #[test]
    fn air_intercept_rejects_unreachable() {
        let p0 = Vec3::ZERO;
        // Receding straight away faster than the grenade flies → no positive intercept.
        assert!(solve_air_intercept(p0, Vec3::new(400.0, 0.0, 0.0), Vec3::new(900.0, 0.0, 0.0), G).is_none());
        // Very far → the flight exceeds the fuse window.
        assert!(solve_air_intercept(p0, Vec3::new(4000.0, 0.0, 0.0), Vec3::ZERO, G).is_none());
    }

    /// The grounded lead-lob lands on the *led* point (where a strafing target will be), not where
    /// it started.
    #[test]
    fn lead_lob_lands_on_moving_target() {
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        let feet = Vec3::new(350.0, 0.0, 0.0);
        let vel = Vec3::new(0.0, 150.0, 0.0); // strafing sideways
        let (look, flight, led) = solve_ground_lead(p0, feet, vel, G).expect("solves");
        let landed = p0 + launch_velocity(look) * flight - Vec3::new(0.0, 0.0, 0.5 * G * flight * flight);
        assert!(
            (landed.xy() - led.xy()).length() < LOB_LAND_TOL,
            "landed {landed:?} vs led {led:?}"
        );
        assert!(led.y > feet.y, "must lead the +y strafe");
    }
}
