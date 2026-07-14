// SPDX-License-Identifier: AGPL-3.0-or-later

//! Rocket-jump ballistics used by the navmesh rocket-jump-link builder (`super::solve_rocket_jumps_from`).
//!
//! A rocket jump is a **two-phase** flight the hook solver's single-`v0` integrator can't model on
//! its own: the bot jumps straight up (`vz = 270`), rises for `fire_delay` seconds, then fires a
//! rocket at the floor/wall; the explosion adds a knockback impulse to the bot's *current* velocity,
//! and the resulting parabola carries it onto a high ledge. So the two solved ingredients — matching
//! the user's framing — are **when** to fire (the delay) and **which way** (the fire angles). This
//! module integrates the ascent closed-form, applies the blast impulse exactly as `combat.rs` does,
//! and hands the continuation to [`super::hook::simulate_arc`]. Pure geometry against the BSP
//! solidity oracle, like `hook.rs`.
//!
//! Everything is priced against **default-mode** damage numbers (`t_radius_damage` 120 / falloff
//! 160, self-splash halved, knockback `dir·points·8` added to velocity, unreduced by armor). Midair
//! mode rescales self-knockback (`rtx_midair_kb_air`) so the solved arcs fly short there — the
//! runtime's failed-link strikes stop a bot re-attempting a leg that keeps landing short.

use glam::{Vec3, Vec3Swizzles};

use super::hook::{march_to_solid, simulate_arc, ArcResult};
use super::{RocketJumpParams, FALL_DAMAGE_SPEED, HOOK_SIM_DT, RJ_LAND_XY, RJ_LAND_Z};
use crate::qphys::JUMP_VZ;

/// View pitches (degrees below horizontal) tried for the shot; a steeper pitch blasts the bot more
/// vertically. 80 ≈ the engine's view-pitch clamp — steeper than that isn't reachable in-game.
pub(super) const RJ_PITCHES: [f32; 4] = [55.0, 65.0, 75.0, 80.0];
/// Fire delays (seconds after the jump press) tried: earlier fires while still rising fast (higher
/// apex, blast closer to the floor); later fires flatter.
pub(super) const RJ_DELAYS: [f32; 4] = [0.05, 0.15, 0.25, 0.35];

/// How far the rocket may travel to its impact surface (the shot aims at nearby floor/wall).
const RJ_ROCKET_RANGE: f32 = 512.0;
/// Radius-damage falloff origin: the player-box centre, `(mins.z + maxs.z)/2 = (−24+32)/2`.
const PLAYER_CENTER_Z: f32 = 4.0;
/// Player-box top (`maxs.z`) — the head height sampled for ascent ceiling clearance.
const PLAYER_TOP_Z: f32 = 32.0;
/// `w_fire_rocket`'s muzzle offset: `origin + v_forward*MUZZLE_FWD + (0,0,MUZZLE_Z)`.
const MUZZLE_Z: f32 = 16.0;
const MUZZLE_FWD: f32 = 8.0;
/// Rocket projectile speed (`SV_FireRocket`).
const ROCKET_SPEED: f32 = 1000.0;
/// Fixed overhead per rocket-jump link: stance walk-in + RL switch (a swallowed impulse until the
/// weapon's cooldown ends) + aim settle. A touch above `HOOK_OVERHEAD` (1.2) for the weapon switch.
const RJ_OVERHEAD: f32 = 1.5;
/// Health→time conversion in the cost: `~50HP × 0.07 ≈ 3.5s`, so a typical RJ link costs ~6s and
/// only beats a genuinely long detour (≈ two 25-health pickups of walking) or an unreachable target
/// — the "high-value reason" gate, implemented by A* itself.
const RJ_HEALTH_SECS_PER_HP: f32 = 0.07;

/// Bot position/velocity `t` seconds into a stationary vertical jump from standing origin `a` — the
/// engine pmove sets `vz = JUMP_VZ` on the jump press; the bot stands still in Stance, so the ascent
/// is purely vertical (any horizontal drift is left to the in-flight air-strafe correction).
fn jump_state(a: Vec3, t: f32, gravity: f32) -> (Vec3, Vec3) {
    let vz = JUMP_VZ - gravity * t;
    let dz = JUMP_VZ * t - 0.5 * gravity * t * t;
    (a + Vec3::new(0.0, 0.0, dz), Vec3::new(0.0, 0.0, vz))
}

/// The view-forward unit vector for QW view `angles` (pitch positive-*down*, roll ignored) — the
/// direction `w_fire_rocket` sends the rocket (`aim_dir` = `v_forward`).
fn fire_dir(angles: Vec3) -> Vec3 {
    let (sp, cp) = angles.x.to_radians().sin_cos();
    let (sy, cy) = angles.y.to_radians().sin_cos();
    Vec3::new(cp * cy, cp * sy, -sp)
}

/// The outcome of one solved rocket jump.
pub(super) struct RjSolution {
    /// Where the continuation arc lands (standing spot just above the floor it hit).
    pub land: Vec3,
    /// Airtime of the post-blast parabola — the runtime's Ballistic-phase watchdog base.
    pub airtime: f32,
    /// Vertical speed at landing (for the hard-landing cost surcharge).
    pub vz_land: f32,
    /// Where the rocket explodes (debug/telemetry).
    pub blast: Vec3,
    /// Seconds from the jump press to the `+attack` that fires the rocket.
    pub t_blast: f32,
    /// Bot position at the blast — stored so the build/test can re-fly the continuation arc.
    pub pos_blast: Vec3,
    /// Bot velocity just after the blast (jump velocity + knockback impulse) — the continuation `v0`.
    pub v0: Vec3,
    /// Pre-armor self-damage points from the blast — the runtime health gate and the cost surcharge.
    pub self_damage: f32,
}

/// Two-phase integration of a rocket jump from standing origin `a`, firing `fire_angles` at
/// `fire_delay` seconds into the jump. Returns the flight outcome, or `None` if the shot finds no
/// surface, the ascent hits a ceiling, the blast is out of self-splash range, or the arc never lands.
///
/// Two solidity oracles, because a rocket and a player collide on different hulls: `player_solid` is
/// the player box (hull 1, used for the ascent-ceiling check and the post-blast landing arc), while
/// `rocket_solid` is a point (hull 0) — the rocket is a zero-size missile, so it detonates on the
/// *true* surface, ~24u below (16u nearer) the inflated player-hull floor. Marching the rocket on the
/// player hull would stop it too high, overestimating the blast (this was the undershoot bug).
pub(super) fn simulate_rocket_jump(
    player_solid: impl Fn(Vec3) -> bool + Copy,
    rocket_solid: impl Fn(Vec3) -> bool + Copy,
    a: Vec3,
    fire_angles: Vec3,
    fire_delay: f32,
    params: RocketJumpParams,
) -> Option<RjSolution> {
    let g = params.gravity;
    let dir = fire_dir(fire_angles);

    // Muzzle at the fire moment, then march the rocket (a point, on hull 0) to the surface it detonates on.
    let (pos_fire, _) = jump_state(a, fire_delay, g);
    let muzzle = pos_fire + dir * MUZZLE_FWD + Vec3::new(0.0, 0.0, MUZZLE_Z);
    let blast = march_to_solid(rocket_solid, muzzle, dir, RJ_ROCKET_RANGE)?;
    let t_blast = fire_delay + (blast - muzzle).length() / ROCKET_SPEED;

    // The rising bot must not clip a ceiling before the blast (its head at `PLAYER_TOP_Z`, hull 1).
    let mut t = 0.0;
    while t < t_blast {
        let (p, _) = jump_state(a, t, g);
        if player_solid(p + Vec3::new(0.0, 0.0, PLAYER_TOP_Z)) {
            return None;
        }
        t += HOOK_SIM_DT;
    }

    // Bot state at the blast; self-splash falloff from the player-box centre (per t_radius_damage).
    let (pos_b, vel_b) = jump_state(a, t_blast, g);
    let center = pos_b + Vec3::new(0.0, 0.0, PLAYER_CENTER_Z);
    let d = (blast - center).length();
    let points = (120.0 - 0.5 * d).max(0.0) * 0.5; // radius damage 120, self splash halved
    if points <= 0.0 {
        return None;
    }

    // Knockback: `velocity += normalize(origin − blast) · points · 8` (plus the `rj` cvar boost when
    // the server sets it > 1), added to the current jump velocity — exactly as `t_damage` does.
    let kdir = (pos_b - blast).normalize_or_zero();
    let mut dv = kdir * (points * 8.0);
    if params.rj_extra > 1.0 {
        dv += kdir * (points * params.rj_extra);
    }
    let v0 = vel_b + dv;

    match simulate_arc(player_solid, pos_b, v0, g) {
        ArcResult::Land { pos, airtime, vz } => Some(RjSolution {
            land: pos,
            airtime,
            vz_land: vz,
            blast,
            t_blast,
            pos_blast: pos_b,
            v0,
            self_damage: points,
        }),
        _ => None,
    }
}

/// Aim error (degrees, each fire axis) the robustness sweep below proves a candidate arc survives.
/// The runtime's fire-release tolerance must stay *inside* this — a bot that shoots while further off
/// than this is flying an arc nobody certified, and near a corner a fraction of a degree changes which
/// surface the rocket detonates on, flipping the knockback. `bot::RJ_AIM_TOL` is derived from this for
/// exactly that reason; don't set one without the other.
pub const RJ_CERT_AIM_DEG: f32 = 1.5;

/// Robustness sweep — the conservative core. A failed rocket jump wastes ~50HP, so a candidate is
/// accepted only if it still lands within **2×** the acceptance window of target `b` under every
/// perturbation a live bot introduces: ±25 ms of fire timing (≈ two bot frames of delay error),
/// ±[`RJ_CERT_AIM_DEG`] on each fire axis (aim-spring settle error), and ±16 u of launch-stance
/// error. This rejects fp-fragile grazing arcs whose landing swings wildly with a hair of input change.
pub(super) fn rj_perturb_ok(
    player_solid: impl Fn(Vec3) -> bool + Copy,
    rocket_solid: impl Fn(Vec3) -> bool + Copy,
    a: Vec3,
    angles: Vec3,
    delay: f32,
    params: RocketJumpParams,
    b: Vec3,
) -> bool {
    let lands = |la: Vec3, ang: Vec3, del: f32| {
        matches!(
            simulate_rocket_jump(player_solid, rocket_solid, la, ang, del, params),
            Some(s)
                if (s.land.xy() - b.xy()).length() <= RJ_LAND_XY * 2.0
                    && (s.land.z - b.z).abs() <= RJ_LAND_Z * 2.0
        )
    };
    let aim = RJ_CERT_AIM_DEG;
    lands(a, angles, (delay - 0.025).max(0.0))
        && lands(a, angles, delay + 0.025)
        && lands(a, angles + Vec3::new(aim, 0.0, 0.0), delay)
        && lands(a, angles + Vec3::new(-aim, 0.0, 0.0), delay)
        && lands(a, angles + Vec3::new(0.0, aim, 0.0), delay)
        && lands(a, angles + Vec3::new(0.0, -aim, 0.0), delay)
        && lands(a + Vec3::new(16.0, 0.0, 0.0), angles, delay)
        && lands(a + Vec3::new(-16.0, 0.0, 0.0), angles, delay)
}

/// Travel-time cost of a rocket-jump link: the real flight (rise-to-blast + arc airtime), the fixed
/// stance/switch/aim overhead, and the health surcharge that makes a rocket jump a deliberate last
/// resort. Plus a hard-landing surcharge mirroring `hook_cost` / `Drop`.
pub(super) fn rocket_jump_cost(t_blast: f32, airtime: f32, vz_land: f32, self_damage: f32) -> f32 {
    let mut c = t_blast + airtime + RJ_OVERHEAD + self_damage * RJ_HEALTH_SECS_PER_HP;
    if vz_land.abs() > FALL_DAMAGE_SPEED {
        c += 1.0;
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A straight-down shot early in a flat-ground jump: the rocket hits the floor ~0.10 s in, the
    /// self-blast is ~half the 120 radius damage, and the added impulse flings the bot decisively
    /// past the double-jump envelope (`DOUBLE_ARC_PEAK` = 100).
    #[test]
    fn rocket_jump_two_phase_solution() {
        // The player box rests on a floor at z=0 (its origin a hull-height above); the point rocket
        // reaches the true surface 24u lower (the player-hull floor is inflated by `-mins.z = 24`).
        let player_floor = |p: Vec3| p.z <= 0.0;
        let rocket_floor = |p: Vec3| p.z <= -24.0;
        let a = Vec3::new(0.0, 0.0, 24.0); // standing origin a hull-height above the floor
        let params = RocketJumpParams { gravity: 800.0, rj_extra: 0.0 };
        let s = simulate_rocket_jump(player_floor, rocket_floor, a, Vec3::new(80.0, 0.0, 0.0), 0.05, params)
            .expect("vertical rocket jump should solve over a flat floor");
        assert!((s.t_blast - 0.12).abs() < 0.04, "t_blast {} not ~0.12", s.t_blast);
        // Blast on the real floor (24u lower) sits further from the player box, so self-damage is a
        // touch below the old hull-1 figure (~50): the honest health cost.
        assert!((38.0..=52.0).contains(&s.self_damage), "self_damage {} not ~44", s.self_damage);
        // Post-blast apex above the launch, from the continuation velocity.
        let apex = s.pos_blast.z + s.v0.z * s.v0.z / (2.0 * params.gravity) - a.z;
        assert!((150.0..=340.0).contains(&apex), "apex {apex} outside the RJ envelope");
        assert!(apex > 100.0, "apex {apex} no better than a double jump");
    }

    /// The self-damage the solver reports equals the game's rocket radius damage recomputed from the
    /// blast geometry: `t_radius_damage` deals 120 with a 0.5/unit falloff from the player-box centre
    /// (weapons.rs fires it at 120; combat.rs's falloff `points = 120 − 0.5·dist`), self splash ×0.5.
    /// Pins those constants so the planned self-damage can't drift from the health the bot really loses.
    #[test]
    fn rj_self_damage_matches_combat() {
        let floor = |p: Vec3| p.z <= 0.0;
        let a = Vec3::new(0.0, 0.0, 24.0);
        let params = RocketJumpParams { gravity: 800.0, rj_extra: 0.0 };
        for delay in [0.05f32, 0.15, 0.25] {
            // The falloff→self_damage identity holds for any blast geometry, so a single floor for both
            // oracles suffices here (the hull distinction is exercised by rocket_jump_two_phase_solution).
            if let Some(s) = simulate_rocket_jump(floor, floor, a, Vec3::new(80.0, 0.0, 0.0), delay, params) {
                let d = (s.blast - (s.pos_blast + Vec3::new(0.0, 0.0, PLAYER_CENTER_Z))).length();
                let game = (120.0 - 0.5 * d).max(0.0) * 0.5;
                assert!((s.self_damage - game).abs() < 1e-3, "self_damage {} != game {game}", s.self_damage);
            }
        }
    }

    /// Perturb rejects a fragile arc: a floor with a pit just past the target — the nominal shot
    /// clears onto the ledge, but a +25 ms-later fire drops the bot into the pit, so it's rejected.
    #[test]
    fn rj_perturb_rejects_fragile() {
        // Floor everywhere except a pit in x ∈ (180, 260): nominal lands at ~200 on the ledge past
        // it only for a precise delay; a slightly later fire falls into the pit.
        let world = |p: Vec3| p.z <= 0.0 && !(180.0..260.0).contains(&p.x);
        let a = Vec3::new(0.0, 0.0, 24.0);
        let params = RocketJumpParams { gravity: 800.0, rj_extra: 0.0 };
        // Find any nominal solve, then confirm perturb is stricter than a bare solve near a pit edge.
        if let Some(s) = simulate_rocket_jump(world, world, a, Vec3::new(60.0, 0.0, 0.0), 0.15, params) {
            if (170.0..280.0).contains(&s.land.x) {
                assert!(
                    !rj_perturb_ok(world, world, a, Vec3::new(60.0, 0.0, 0.0), 0.15, params, s.land),
                    "perturb accepted an arc landing at a pit edge"
                );
            }
        }
    }
}
