// SPDX-License-Identifier: AGPL-3.0-or-later

//! The pure aim/ballistics math behind the combat overlay: aim-spring stiffness and spread by skill,
//! the closed-form intercept-time solver, the fixed-`dt` fall/ballistic integrators and their
//! perturbed intercept search, the fire tolerance, and the direction->view-angle projection. All
//! stateless (glam only), so they unit-test without a `GameState`; `engage` and the grenade solvers
//! call them.

use glam::Vec3;

use crate::math::wrap180;

/// Aim-spring stiffness (1/s) for a given skill — the single source shared with the spring
/// integrator in `bot.rs`, so the feed-forward lag estimate here matches the actual spring.
impl crate::game::GameState {
    /// How far ahead of what we can see to aim, for a shot of this kind.
    ///
    /// Inside a server this is zero and every branch below folds away: the bot reads the enemy's
    /// position as the server has it, and its shot is judged the instant it's taken. As a network
    /// client both halves are late — we see where the enemy *was* half a trip ago, and the server
    /// judges our shot half a trip from now — so the enemy has a full round trip in which to move,
    /// on top of any flight time. Aiming at what you can see is aiming behind by exactly that.
    ///
    /// **Antilag** is why a projectile and a bullet want different answers. A server running it
    /// rewinds the players to where the shooter saw them before tracing an instant hit — the whole
    /// point being that a laggy player aims where they look — so a bullet must *not* be led, and one
    /// that is misses in front. It can't do the same for a rocket: that's a real object, spawned when
    /// the command lands and flying in the server's present, so the round trip counts for it either
    /// way.
    pub(crate) fn aim_lead(&self, projectile: bool) -> f32 {
        if projectile || !self.host.cvar_bool(c"sv_antilag") {
            self.client_lead
        } else {
            0.0
        }
    }
}

pub(crate) fn aim_omega(skill: f32) -> f32 {
    6.0 + skill * 2.0
}

/// Ceiling on the aim spring's angular speed (deg/s), skill-scaled. It sits *above* the spring's own
/// natural peak for an ordinary ≤90° correction (`omega·90/e` ≈ 200 at skill 0, 660 at skill 7), so
/// combat flicks are untouched — but it turns a ≥150° look-target reversal (a vigil-scan flip, a
/// goal re-pick, a flickering enemy) from an instant snap / unbounded spin into a fast *human* pan
/// (~360 deg/s at skill 0, ~990 at skill 7, in the range of a real fast flick). This is the missing
/// view turn-rate dial; `rtx_bot_turnrate > 0` overrides it for live tuning.
pub(crate) fn aim_rate_cap(skill: f32) -> f32 {
    360.0 + skill * 90.0
}

/// Multiplier on the base aim spread from three human tracking factors, all ≥ 1 so they only ever
/// *widen* the error: **convergence** (loose on first sight at `visible_for = 0`, tightening below 1
/// over ~1.5s of continuous line of sight), **own motion** (worse while running, up to +40% at
/// 320ups), and **target crossing** (worse the faster the target moves across the line of fire,
/// `perp_speed/dist` ≈ angular rate). Pure, so the clamps/monotonicity are unit-testable; skill's
/// contribution stays in the base spread (so skill 7 ⇒ base 0 ⇒ spread 0 regardless of this).
pub(super) fn spread_scale(visible_for: f32, own_speed: f32, perp_speed: f32, dist: f32) -> f32 {
    let converge = 1.6 + (0.7 - 1.6) * (visible_for / 1.5).clamp(0.0, 1.0);
    let move_factor = 1.0 + 0.4 * (own_speed / 320.0).min(1.0);
    let track_factor = 1.0 + 0.5 * (perp_speed / dist.max(1.0)).min(1.0);
    converge * move_factor * track_factor
}

/// Time for a projectile of speed `s` fired from the origin to meet a target at relative position
/// `r` moving at constant velocity `v`: the smallest positive root of `|r + v·t| = s·t`
/// (quadratic `(v·v − s²)t² + 2(r·v)t + r·r = 0`). This is what makes lead *geometry-aware*:
/// motion perpendicular to the line of fire lengthens the flight and shifts the aim a lot, motion
/// straight toward/away barely does. `None` when no positive intercept exists (target outrunning
/// the projectile). Shared with the grenade airborne solver (`grenade::solve_air_intercept`), where
/// gravity cancels between projectile and free-falling target so the meet reduces to this linear one.
pub(crate) fn intercept_time(r: Vec3, v: Vec3, s: f32) -> Option<f32> {
    let a = v.dot(v) - s * s;
    let b = 2.0 * r.dot(v);
    let c = r.dot(r);
    if a.abs() < 1e-3 {
        // Degenerate: target speed equals projectile speed — the quadratic collapses to linear.
        let t = -c / b;
        return (b.abs() > 1e-6 && t > 0.0).then_some(t);
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let mut best = f32::INFINITY;
    for t in [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)] {
        if t > 0.0 && t < best {
            best = t;
        }
    }
    best.is_finite().then_some(best)
}

/// Where and when a free-falling target's parabola first meets the world, within `horizon`
/// seconds — the landing spot for the aim clamp below. Integrated through the `trace` oracle
/// (hull1, the player hull) exactly like [`crate::bot::grenade::simulate_bounce`] so the discrete
/// step matches the engine's `SV_Physics_Toss`; a non-floor hit (wall/ceiling) just clips the
/// velocity and the fall continues. `None` when it's still airborne at the horizon. Pure over the
/// oracle, so it's unit-testable against a synthetic floor.
pub(crate) fn fall_land(
    trace: &impl Fn(Vec3, Vec3) -> crate::bsp::HullTrace,
    p0: Vec3,
    v0: Vec3,
    gravity: f32,
    horizon: f32,
) -> Option<(f32, Vec3)> {
    let (mut p, mut v, mut t) = (p0, v0, 0.0f32);
    // Step cap: a fall wedged into a corner can slide in tiny sub-steps that barely advance `t`;
    // bound the work and treat an unresolved fall as "still airborne" (no clamp), like the horizon.
    for _ in 0..512 {
        if t >= horizon {
            break;
        }
        let dt = (16.0 / v.length().max(1.0)).min(0.02);
        v.z -= gravity * dt; // SV_AddGravity before the move
        let target = p + v * dt;
        let tr = trace(p, target);
        if tr.fraction < 1.0 {
            if tr.plane_normal.z > 0.7 {
                return Some((t + tr.fraction * dt, tr.endpos)); // rests on a floor
            }
            // Wall/ceiling: slide along it (kill the into-surface component) and keep falling.
            p = tr.endpos + tr.plane_normal * 0.25;
            v -= v.dot(tr.plane_normal) * tr.plane_normal;
            t += tr.fraction * dt;
        } else {
            p = target;
            t += dt;
        }
    }
    None
}

/// A free-falling target's origin at time `t`: the parabola `p0 + v0·t − ½g·t²·ẑ` until it lands,
/// then held at the landing point drifting at the ground horizontal speed. The clamp is the whole
/// point — without it a rocket aimed at a jumping enemy near the floor is led *through* the floor,
/// to a point the enemy will never occupy because they'll be standing on the ground when it arrives.
pub(crate) fn ballistic_pos(p0: Vec3, v0: Vec3, gravity: f32, land: Option<(f32, Vec3)>, t: f32) -> Vec3 {
    if let Some((t_land, land_pos)) = land {
        if t >= t_land {
            return land_pos + Vec3::new(v0.x, v0.y, 0.0) * (t - t_land);
        }
    }
    p0 + v0 * t - Vec3::new(0.0, 0.0, 0.5 * gravity * t * t)
}

/// Flight time (and meet point) for a straight projectile of speed `s` from `from` to intercept a
/// target whose position at time `t` is `pos_at(t)` — used for rockets, whose flat flight can't be
/// folded into gravity the way a grenade's can. Fixed-point `t ← |pos_at(t) − from| / s` seeded by
/// the gravity-free linear intercept; five rounds converge to well under a unit at rocket speeds.
/// `None` if it doesn't settle onto the projectile sphere (a target falling away faster than the
/// rocket closes), so the caller can fall back to a plain linear lead.
pub(super) fn ballistic_intercept(from: Vec3, pos_at: &impl Fn(f32) -> Vec3, s: f32, seed: f32) -> Option<(f32, Vec3)> {
    let mut t = seed.max(0.0);
    for _ in 0..5 {
        t = (pos_at(t) - from).length() / s;
    }
    let meet = pos_at(t);
    let residual = (meet - from).length() - s * t;
    (t > 0.0 && residual.abs() < 1.0).then_some((t, meet))
}

/// Lateral world-space miss at `range` for the angular gap between the smoothed view and the clean
/// firing solution (`miss ≈ sin(Δ)·range`). Gating fire on *distance off the target* rather than a
/// fixed angular cone keeps a skill-7 solve honest: 4° of slack is ~28u at 400u — a whole player
/// width — while 16u is a guaranteed hull hit regardless of range. The angle is clamped to 90°
/// before the `sin`: past a quarter turn the crosshair is nowhere near the target and any real
/// tolerance is tens of units, so saturating at the full `range` keeps the gate monotone (an
/// unclamped `sin` dips back toward zero — even negative past 180° — and would wrongly pass a shot
/// aimed the opposite way, e.g. mid-flick onto an enemy that just appeared behind the bot).
pub(super) fn miss_distance(view: Vec3, clean: Vec3, range: f32) -> f32 {
    let dp = wrap180(view.x - clean.x).to_radians();
    let dy = wrap180(view.y - clean.y).to_radians();
    (dp * dp + dy * dy).sqrt().min(std::f32::consts::FRAC_PI_2).sin() * range
}

/// Fire-gate tolerance (world units) at the intercept range. A **direct** hit must land inside the
/// ±16u hull (16u at skill 7); a **splash** shot rides the 160u blast, so it fires far looser (40u
/// at skill 7). Both widen with `(7 − skill)` so a low-skill bot still fires — loose, and misses —
/// rather than freezing when its lagging aim never quite reaches the tight cone.
pub(super) fn fire_tolerance(skill: f32, direct: bool) -> f32 {
    let s = skill.clamp(0.0, 7.0);
    if direct {
        16.0 + (7.0 - s) * 18.0
    } else {
        40.0 + (7.0 - s) * 25.0
    }
}
