// SPDX-License-Identifier: AGPL-3.0-or-later

//! Grappling-hook ballistics used by the navmesh hook-link builder (`super::solve_hooks_from`): the
//! offline parabola integration and the release/anchor solvers. Pure geometry against the BSP
//! solidity oracle — no graph state — so they live apart from the graph build.

use glam::{Vec3, Vec3Swizzles};

use super::{
    HookParams, FALL_DAMAGE_SPEED, HOOK_LAND_XY, HOOK_LAND_Z, HOOK_MAX_AIRTIME, HOOK_OVERHEAD, HOOK_SAMPLE, HOOK_SIM_DT,
};
use crate::bsp::Bsp;

/// Launch pitches (degrees above horizontal) tried when searching for a hook anchor.
pub(super) const HOOK_PITCHES: [f32; 4] = [20.0, 40.0, 60.0, 80.0];

/// Outcome of flying a release velocity from a point under gravity against a solidity oracle.
pub(super) enum ArcResult {
    /// The parabola descended onto solid: the standing position just above it, airtime, and the
    /// vertical speed at impact (for fall-damage pricing).
    Land { pos: Vec3, airtime: f32, vz: f32 },
    /// Ran into solid while level or ascending, or *struck a wall side-on* while descending — the
    /// arc is obstructed, not landed.
    Blocked,
    /// Never landed within the airtime cap.
    Timeout,
}

/// Integrate a ballistic arc from `r` with initial velocity `v0` under `gravity`, stepping so no
/// step advances more than `HOOK_SAMPLE`, until it hits solid (landing if descending onto a floor,
/// blocked otherwise) or the airtime cap. Pure: the world enters only through the `is_solid` oracle,
/// so this is unit-testable against the closed-form parabola with a synthetic floor.
///
/// A descending hit is only a *landing* if the blocker is underneath. Descending into the side of a
/// wall is not: the bot scrapes it and slides down to whatever is below, far from here. Accepting
/// those was the "certified" rocket jump that flies into a pillar face and falls back to the floor —
/// and because a wall face is perfectly stable under perturbation, the robustness sweep rated it the
/// *safest* link on the map. Classify by re-testing the step with the horizontal component removed:
/// solid on the pure-vertical move ⇒ floor beneath ⇒ a true touchdown.
pub(super) fn simulate_arc(is_solid: impl Fn(Vec3) -> bool, r: Vec3, v0: Vec3, gravity: f32) -> ArcResult {
    let mut p = r;
    let mut v = v0;
    let mut t = 0.0;
    while t < HOOK_MAX_AIRTIME {
        let dt = (HOOK_SAMPLE / v.length().max(1.0)).min(HOOK_SIM_DT);
        let next = p + v * dt;
        if is_solid(next) {
            let onto_floor = is_solid(Vec3::new(p.x, p.y, next.z));
            return if v.z < 0.0 && onto_floor {
                ArcResult::Land {
                    pos: p,
                    airtime: t,
                    vz: v.z,
                }
            } else {
                ArcResult::Blocked
            };
        }
        p = next;
        v.z -= gravity * dt;
        t += dt;
    }
    ArcResult::Timeout
}

/// Fly a release from `r` and report where it lands (descending onto floor), or `None` if it's
/// blocked or never lands. Also used by the bot grenade-lob solver to verify an arc's clearance.
pub fn arc_land(bsp: &Bsp, r: Vec3, v0: Vec3, gravity: f32) -> Option<(Vec3, f32, f32)> {
    match simulate_arc(|p| bsp.is_solid(p), r, v0, gravity) {
        ArcResult::Land { pos, airtime, vz } => Some((pos, airtime, vz)),
        _ => None,
    }
}

/// March a ray from `from` along unit `dir` until it strikes solid, returning the last empty point
/// (the surface the hook would stick to / the rocket would explode on), or `None` within `max`.
/// Bisected for a tight surface. Takes the solidity oracle as a closure so the rocket-jump solver
/// reuses it against a synthetic floor in tests, not just `&Bsp`.
pub(super) fn march_to_solid(is_solid: impl Fn(Vec3) -> bool, from: Vec3, dir: Vec3, max: f32) -> Option<Vec3> {
    let mut d = HOOK_SAMPLE;
    while d <= max {
        if is_solid(from + dir * d) {
            let (mut lo, mut hi) = (d - HOOK_SAMPLE, d);
            for _ in 0..4 {
                let mid = (lo + hi) * 0.5;
                if is_solid(from + dir * mid) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            return Some(from + dir * lo);
        }
        d += HOOK_SAMPLE;
    }
    None
}

/// Robustness sweep for a candidate hook (release at distance `d` along the rope from `launch`
/// toward the stick, target cell origin `b`): require the arc to still land near **`b`** under a
/// ±10% reel-speed error and a ±16u release-point error. Clustering the perturbed landings on the
/// target (not merely "somewhere standable") rejects fp-fragile grazing arcs whose landing swings
/// wildly with a hair of input change — which is exactly what keeps the runtime re-solve honest and
/// stops a bot being flung off-target when its reel timing is slightly off.
pub(super) fn perturb_ok(
    bsp: &Bsp,
    stick: Vec3,
    rdir: Vec3,
    release_dist: f32,
    rope: f32,
    params: HookParams,
    b: Vec3,
) -> bool {
    let variants = [
        (release_dist, params.pull * 0.9),
        (release_dist, params.pull * 1.1),
        ((release_dist - 16.0).max(HOOK_SAMPLE), params.pull),
        ((release_dist + 16.0).min(rope - HOOK_SAMPLE), params.pull),
    ];
    variants.iter().all(|&(rd, pull)| {
        let r = stick - rdir * rd;
        match arc_land(bsp, r, rdir * pull, params.gravity) {
            Some((land, _, _)) => {
                (land.xy() - b.xy()).length() <= HOOK_LAND_XY * 2.0 && (land.z - b.z).abs() <= HOOK_LAND_Z * 2.0
            }
            None => false,
        }
    })
}

/// Travel-time cost of a hook link: hook flight + reel to the release point + parabola airtime +
/// fixed overhead, plus a fall-damage surcharge on a hard landing (mirroring `Drop`).
pub(super) fn hook_cost(rope: f32, release_dist: f32, airtime: f32, vz_land: f32, params: HookParams) -> f32 {
    let throw = rope / params.throw;
    let reel = (rope - release_dist).max(0.0) / params.pull;
    let mut c = throw + reel + airtime + HOOK_OVERHEAD;
    if vz_land.abs() > FALL_DAMAGE_SPEED {
        c += 1.0;
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Descending into the *side* of a wall is not a landing — the bot scrapes the face and slides
    /// down to whatever is below. Only a blocker underneath is a touchdown. Certifying wall strikes
    /// is what minted stronghold's rocket jumps into the quad pillar: the solver read the contact
    /// point as a landing on top of the pillar, and since a wall face never moves under perturbation,
    /// the robustness sweep confirmed it as rock solid. The bot flew into the wall every single time.
    #[test]
    fn descending_into_a_wall_face_is_blocked_not_landed() {
        // Floor at z=0, and a wall filling x >= 64 (up to z=200).
        let solid = |p: Vec3| p.z <= 0.0 || (p.x >= 64.0 && p.z <= 200.0);
        // Launched up-and-toward the wall: apexes at x=25, then meets the face at z~91 descending.
        let arc = simulate_arc(solid, Vec3::new(0.0, 0.0, 100.0), Vec3::new(200.0, 0.0, 100.0), 800.0);
        assert!(
            matches!(arc, ArcResult::Blocked),
            "wall strike must not certify as a landing"
        );
    }

    /// The same descent with the wall removed lands on the floor beneath — the classifier rejects
    /// side-on hits without rejecting ordinary touchdowns.
    #[test]
    fn descending_onto_floor_still_lands() {
        let solid = |p: Vec3| p.z <= 0.0;
        let arc = simulate_arc(solid, Vec3::new(0.0, 0.0, 100.0), Vec3::new(200.0, 0.0, 100.0), 800.0);
        assert!(
            matches!(arc, ArcResult::Land { .. }),
            "a plain floor landing must still certify"
        );
    }
}
