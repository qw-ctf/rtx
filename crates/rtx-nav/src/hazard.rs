// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared hazard classification for the bots: where lava, slime, and lethal drops lie relative to
//! a point. Two consumers use it, which is why it lives in this neutral navigation-core crate:
//!
//!  * **offence** — [`find_hazard`] scans the ring around an enemy for a spot to *shove* them into
//!    (the grenade lob-and-shoot combo and the one-shot rocket shove in the game's `bot::grenade`);
//!  * **self-preservation** — [`hazard_ahead`] asks "does stepping this way walk me into lava?" for
//!    the game's combat-movement guard, and the navmesh bakes a routing surcharge onto links that
//!    skirt a liquid edge.
//!
//! Everything here is **pure** over two oracles — `is_solid` (the clip-hull point test,
//! [`crate::bsp::Bsp::is_solid`]) and `contents` (the render-hull `pointcontents`, which is the only
//! hull that carries liquid contents — the clip hull resolves to solid/empty only). Keeping them as
//! closures lets the same logic run against a live map, the engine, or a synthetic test fixture.

use glam::Vec3;

use crate::bsp::{CONTENTS_EMPTY, CONTENTS_LAVA, CONTENTS_SLIME, CONTENTS_SOLID, CONTENTS_WATER};
use crate::navmesh::GRID;
use crate::qphys::STEP_HEIGHT;

/// The eight compass directions probed around a point for a hazard.
pub(crate) const HAZARD_DIRS: [(f32, f32); 8] = [
    (1.0, 0.0),
    (0.707, 0.707),
    (0.0, 1.0),
    (-0.707, 0.707),
    (-1.0, 0.0),
    (-0.707, -0.707),
    (0.0, -1.0),
    (0.707, -0.707),
];
/// Distances out from an enemy sampled for a shoveable hazard edge.
const HAZARD_RADII: [f32; 3] = [48.0, 96.0, 144.0];
/// Distances ahead of the feet sampled by [`hazard_ahead`]: one within this movement step, one a
/// stride further for momentum margin (a running bot needs room to stop before the edge).
const HAZARD_AHEAD_DISTS: [f32; 2] = [40.0, 72.0];
/// A downward drop past this counts as a lethal/harmful fall to shove someone off (2·SAFE_FALL).
const HAZARD_DROP: f32 = 176.0;
/// How far down to look for a floor before calling it a pit.
const HAZARD_PROBE_DEPTH: f32 = 320.0;

/// What kind of hazard a direction leads to. Ordered by how much a bot should prefer to shove there.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HazardKind {
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
pub struct Hazard {
    pub dir: Vec3,
    pub edge_dist: f32,
    pub kind: HazardKind,
}

/// What the downward march from a point first meets: a liquid, solid ground at that fall distance,
/// or nothing within reach. The raw probe behind [`hazard_below`] and [`water_ahead`], so the two
/// classify the *same* geometry and only differ in which finding they care about.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Below {
    Lava,
    Slime,
    Water,
    Solid(f32), // ground this far below the probe point
    Bottomless, // no floor within HAZARD_PROBE_DEPTH
}

/// March straight down from `p`, reporting the first liquid or solid floor met (water short-circuits
/// like the others — it never lets the march run past its surface to the floor below).
///
/// The march is coarse (24u strides), but a *shallow* liquid film pooled on a floor is thinner than
/// one stride and can fall entirely between two samples — its surface below the last open sample,
/// yet above the solid the next sample hits. So when a stride lands in solid, we don't take that at
/// face value: [`solid_or_film`] refines the exact boundary and runs the engine's own waterlevel-1
/// test there, catching the film the coarse march stepped over. (The very first sample landing solid
/// has no open bracket above it to refine — that's a wall, reported as plain solid.)
fn probe_below(is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> i32, p: Vec3) -> Below {
    let mut d = 0.0;
    let mut open_above = None; // depth of the last sample that was neither solid nor liquid
    while d <= HAZARD_PROBE_DEPTH {
        let q = p - Vec3::new(0.0, 0.0, d);
        let c = contents(q);
        if c == CONTENTS_LAVA {
            return Below::Lava;
        }
        if c == CONTENTS_SLIME {
            return Below::Slime;
        }
        if c == CONTENTS_WATER {
            return Below::Water;
        }
        if is_solid(q) {
            return match open_above {
                Some(lo) => solid_or_film(is_solid, contents, p, lo, d),
                None => Below::Solid(0.0), // solid at the very first sample — a wall, no bracket to refine
            };
        }
        open_above = Some(d);
        d += 24.0;
    }
    Below::Bottomless
}

/// Refine the open→solid transition between depths `lo` (last open sample) and `hi` (first solid
/// sample), then run the engine's own waterlevel-1 test at the spot a player would rest there — the
/// crux of shallow-film detection.
///
/// `is_solid` reads the *clip hull* (hull 1), inflated by the player's bounding box, so a floor's
/// top face bevels up by `|mins.z|` = 24u: the boundary the binary search finds is where a standing
/// player's *origin* rests, not where its feet touch. The liquid film — carried only by the render
/// hull `contents` samples — sits on the true floor 24u below that. The engine's `SV_CheckWater`
/// sets waterlevel 1 (and the game's `apply_liquid_damage` burns you) from a probe at
/// `origin + mins.z + 1` = feet+1, so we sample `contents` at `boundary − 23`. A liquid there is
/// precisely a film that scalds a player standing at this boundary while being too thin for the
/// coarse march to land inside — the whole reason ankle-deep lava used to read as safe ground.
fn solid_or_film(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> i32,
    p: Vec3,
    lo: f32,
    hi: f32,
) -> Below {
    // Bisect the bracket (known-open `lo`, known-solid `hi`): five halvings pin the 24u stride to
    // under 1u. `hi` stays the shallowest depth proven solid — the boundary.
    let (mut lo, mut hi) = (lo, hi);
    for _ in 0..5 {
        let mid = 0.5 * (lo + hi);
        if is_solid(p - Vec3::new(0.0, 0.0, mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let c = contents(p - Vec3::new(0.0, 0.0, hi + 23.0));
    if c == CONTENTS_LAVA {
        Below::Lava
    } else if c == CONTENTS_SLIME {
        Below::Slime
    } else if c == CONTENTS_WATER {
        Below::Water
    } else {
        Below::Solid(hi) // refined depth — at worst 24u shallower than the coarse march's, a hair more accurate against HAZARD_DROP
    }
}

/// Classify what's below `p` by marching down: lava/slime (from `contents`) or a big drop / pit
/// (from `is_solid`). `None` if solid ground sits close below, or if *water* is below — swimmable
/// water breaks a fall and is harmless, so it is never a hazard (a deep pool would otherwise march
/// past its surface to the floor and misread as a pit).
pub(crate) fn hazard_below(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> i32,
    p: Vec3,
) -> Option<HazardKind> {
    match probe_below(is_solid, contents, p) {
        Below::Lava => Some(HazardKind::Lava),
        Below::Slime => Some(HazardKind::Slime),
        Below::Water => None, // swimmable — harmless, and not a pit no matter how deep
        Below::Solid(d) => (d > HAZARD_DROP).then_some(HazardKind::Pit),
        Below::Bottomless => Some(HazardKind::Pit), // no floor within reach
    }
}

/// Find the best hazard to shove an enemy (at `e_feet`) into: probe a ring of directions/distances,
/// classify each by what lies below (liquids via `contents`, drops via `is_solid`), and require a
/// clear horizontal path to it (a railing/wall between blocks the shove). Pure over the two oracles.
pub fn find_hazard(is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> i32, e_feet: Vec3) -> Option<Hazard> {
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

/// Whether stepping from `feet` along horizontal unit `dir` walks into a hazard within the next
/// stride or two: probe a near and a far point ahead (feet-plus-a-little height) and classify what
/// lies below each. A probe that lands *inside solid* is a wall, not a pit — normal collision
/// handles that — so it's skipped, not treated as unsafe. Pure over the two oracles, like
/// [`hazard_below`]; the game's combat movement guard uses it to veto a lethal step.
pub fn hazard_ahead(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> i32,
    feet: Vec3,
    dir: Vec3,
) -> Option<HazardKind> {
    for d in HAZARD_AHEAD_DISTS {
        let p = feet + dir * d + Vec3::new(0.0, 0.0, 8.0);
        if is_solid(p) {
            continue; // a wall ahead, not lava/a pit — leave it to normal movement/collision
        }
        if let Some(k) = hazard_below(is_solid, contents, p) {
            return Some(k);
        }
    }
    None
}

/// Whether stepping from `feet` along horizontal unit `dir` heads out over water within the next
/// stride or two — the same probe geometry as [`hazard_ahead`], but reporting swimmable water rather
/// than lethal hazards. The combat guard uses it to prefer dry footing: water is slow and exposed,
/// so a bot picks a dry candidate move over a wet one when both are safe.
pub fn water_ahead(is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> i32, feet: Vec3, dir: Vec3) -> bool {
    HAZARD_AHEAD_DISTS.iter().any(|&d| {
        let p = feet + dir * d + Vec3::new(0.0, 0.0, 8.0);
        // A wall ahead isn't water — normal collision handles it (mirrors `hazard_ahead`).
        !is_solid(p) && probe_below(is_solid, contents, p) == Below::Water
    })
}

/// Vertical stride when marching down to test for a walkable floor under a probe point.
const LEDGE_MARCH: f32 = 8.0;
/// How far to each side of the travel direction [`edge_bias`] probes for a drop — a body half-width
/// (16) plus a small margin, so a bot reacts before its hull actually reaches the lip.
const EDGE_PROBE_SIDE: f32 = 24.0;
/// Small forward offset of the side probes so a moving bot reads the edge it's heading onto, not the
/// floor it's already standing on.
const EDGE_PROBE_AHEAD: f32 = 12.0;
/// Marge below the side-probe point within which a floor still counts as continuous. With the probe's
/// 8u lift and the hull's 24u bevel baked into `is_solid`, this lands the effective side-drop
/// threshold at roughly two steps down from the feet plane — a single walkable step beside the path is
/// fine, an open shaft or a real ledge is steered away from.
const EDGE_SIDE_DROP: f32 = 16.0;

/// Whether the floor falls away by more than `allow` just below probe point `p` — a real drop, not a
/// walkable step. `p` already inside solid is a wall/step-up (not a fall), so `false`. Marches down in
/// `LEDGE_MARCH` strides looking for a floor within `allow` of `p`.
fn drop_below(is_solid: &impl Fn(Vec3) -> bool, p: Vec3, allow: f32) -> bool {
    if is_solid(p) {
        return false; // a wall or step-up here — normal collision handles it, not a fall
    }
    let mut z = 0.0;
    while z <= allow {
        if is_solid(p - Vec3::new(0.0, 0.0, z)) {
            return false;
        }
        z += LEDGE_MARCH;
    }
    true
}

/// Whether walking from `feet` along horizontal unit `dir` runs off a plain **ledge** within a stride
/// or two — the everyday-fall analogue of [`hazard_ahead`], which flags only lava/slime/lethal pits
/// and runs only in combat. The floor is *allowed* to descend as you walk: a staircase steps down one
/// [`STEP_HEIGHT`] per grid cell, so the tolerated drop grows with distance. This trips only when the
/// floor falls away faster than that walkable rate — a real edge the route didn't mean to leave. Pure
/// over `is_solid`; the game's path-follower uses it to brake a grounded bot before it fumbles off.
pub fn ledge_ahead(is_solid: &impl Fn(Vec3) -> bool, feet: Vec3, dir: Vec3) -> bool {
    HAZARD_AHEAD_DISTS.iter().any(|&d| {
        // The most the floor may have legally dropped by `d` and still be a walkable descent: the 8u
        // probe lift plus one step per grid cell covered.
        let p = feet + dir * d + Vec3::new(0.0, 0.0, 8.0);
        drop_below(is_solid, p, 8.0 + STEP_HEIGHT * (1.0 + d / GRID))
    })
}

/// A sideways steering nudge that keeps a walking bot off drop edges beside its path. Probes a
/// body-width to each side of `dir` (a touch ahead) for a drop; returns a horizontal unit vector
/// pushing *away* from a one-sided drop, or zero when both sides are safe (open floor — no nudge) or
/// both drop (a thin catwalk / balance beam — hold the centre line rather than veer off the far side).
/// This is what keeps a bot spiralling up an open-cored staircase, or crossing a narrow ledge, from
/// drifting off the inner edge while it steers for the next cell centre. Pure over `is_solid`.
pub fn edge_bias(is_solid: &impl Fn(Vec3) -> bool, feet: Vec3, dir: Vec3) -> Vec3 {
    let perp = Vec3::new(-dir.y, dir.x, 0.0);
    let base = feet + dir * EDGE_PROBE_AHEAD + Vec3::new(0.0, 0.0, 8.0);
    let side_drops = |s: Vec3| drop_below(is_solid, base + s * EDGE_PROBE_SIDE, EDGE_SIDE_DROP);
    match (side_drops(perp), side_drops(-perp)) {
        (true, false) => -perp, // drop on the left → steer right
        (false, true) => perp,  // drop on the right → steer left
        _ => Vec3::ZERO,        // both safe, or both drop (thin path) → hold the line
    }
}

/// Whether open air sits directly above `p` — a surface a submerged bot can swim up to by holding
/// jump. Marches up in strides: reaching `CONTENTS_EMPTY` means an open surface overhead (true);
/// hitting solid first means a roofed underwater tunnel (false) — there the bot must swim *out* to
/// a breathing spot rather than press uselessly into the ceiling. Pure over the render-hull oracle.
pub fn surface_above(contents: &impl Fn(Vec3) -> i32, p: Vec3) -> bool {
    let mut d = 0.0;
    while d <= HAZARD_PROBE_DEPTH {
        let c = contents(p + Vec3::new(0.0, 0.0, d));
        if c == CONTENTS_EMPTY {
            return true; // broke the surface into open air
        }
        if c == CONTENTS_SOLID {
            return false; // roofed — no surface directly overhead
        }
        d += 24.0;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsp::CONTENTS_EMPTY;

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
                CONTENTS_LAVA
            } else {
                CONTENTS_EMPTY
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
        let empty = |_: Vec3| CONTENTS_EMPTY;
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
                CONTENTS_LAVA
            } else {
                CONTENTS_EMPTY
            }
        };
        assert!(find_hazard(&solid, &contents, Vec3::new(40.0, 0.0, 24.0)).is_none());
    }

    #[test]
    fn open_floor_has_no_hazard() {
        let empty = |_: Vec3| CONTENTS_EMPTY;
        assert!(find_hazard(&floor, &empty, Vec3::new(0.0, 0.0, 24.0)).is_none());
    }

    #[test]
    fn deep_water_is_safe() {
        // A pool of water with no floor within reach: harmless, must not read as a pit.
        let never_solid = |_: Vec3| false;
        let water = |p: Vec3| {
            if p.z < 0.0 {
                CONTENTS_WATER
            } else {
                CONTENTS_EMPTY
            }
        };
        assert!(hazard_below(&never_solid, &water, Vec3::new(0.0, 0.0, 24.0)).is_none());
        // And a bot standing at its edge is not warned away from swimmable water.
        assert!(hazard_ahead(
            &never_solid,
            &water,
            Vec3::new(0.0, 0.0, 24.0),
            Vec3::new(1.0, 0.0, 0.0)
        )
        .is_none());
    }

    #[test]
    fn hazard_ahead_detects_lava_edge() {
        // Lava fills x > 60 below the floor; a bot at the origin stepping +x walks toward it.
        let solid = |p: Vec3| p.z <= 0.0 && p.x <= 60.0;
        let contents = |p: Vec3| {
            if p.x > 60.0 && p.z < 0.0 {
                CONTENTS_LAVA
            } else {
                CONTENTS_EMPTY
            }
        };
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert_eq!(
            hazard_ahead(&solid, &contents, feet, Vec3::new(1.0, 0.0, 0.0)),
            Some(HazardKind::Lava)
        );
        // Stepping the other way stays over floor — safe.
        assert!(hazard_ahead(&solid, &contents, feet, Vec3::new(-1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hazard_ahead_safe_on_open_floor() {
        let empty = |_: Vec3| CONTENTS_EMPTY;
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(hazard_ahead(&floor, &empty, feet, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hazard_ahead_wall_is_not_a_hazard() {
        // Solid wall filling all of x > 60 (including at knee height): a wall, not a pit.
        let solid = |p: Vec3| p.z <= 0.0 || p.x > 60.0;
        let empty = |_: Vec3| CONTENTS_EMPTY;
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(hazard_ahead(&solid, &empty, feet, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn water_ahead_detects_a_pool() {
        // Water fills x > 60 below the floor; a bot at the origin stepping +x heads into it.
        let solid = |p: Vec3| p.z <= 0.0 && p.x <= 60.0;
        let contents = |p: Vec3| {
            if p.x > 60.0 && p.z < 0.0 {
                CONTENTS_WATER
            } else {
                CONTENTS_EMPTY
            }
        };
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(water_ahead(&solid, &contents, feet, Vec3::new(1.0, 0.0, 0.0)));
        // Stepping the other way stays over dry floor.
        assert!(!water_ahead(&solid, &contents, feet, Vec3::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn water_ahead_dry_floor_and_wall_are_not_water() {
        let empty = |_: Vec3| CONTENTS_EMPTY;
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(!water_ahead(&floor, &empty, feet, Vec3::new(1.0, 0.0, 0.0)));
        // A wall ahead (solid at the probe) is not water.
        let wall = |p: Vec3| p.z <= 0.0 || p.x > 60.0;
        assert!(!water_ahead(&wall, &empty, feet, Vec3::new(1.0, 0.0, 0.0)));
    }

    // Shallow-liquid films: a thin sheet of lava/slime/water resting on a floor, too thin for the
    // coarse 24u down-march to land a sample inside. `is_solid` is the hull-1 clip test, so a floor
    // bevels up to the resting origin — these oracles model that bevel (walkway solid to z ≤ 24 with
    // feet at the visual floor z = 0, like the `ledge_ahead` tests below), with a short drop into a
    // basin beyond and a 4u-thick film hovering just above the basin floor. From feet+8 = z 8 the
    // march steps clean to z −16, over the film — this is the ankle-deep lava that used to read safe.
    // The film band (−32, −28) is where boundary−23 lands: basin solid begins ≈ z −8 (depth ≈ 16.5
    // below the probe), and 8 − (16.5 + 23) ≈ −31.5.

    #[test]
    fn hazard_ahead_detects_shallow_lava_film() {
        let solid = |p: Vec3| if p.x <= 60.0 { p.z <= 24.0 } else { p.z <= -8.0 };
        let film = |p: Vec3| {
            if p.x > 60.0 && (-32.0..-28.0).contains(&p.z) {
                CONTENTS_LAVA
            } else {
                CONTENTS_EMPTY
            }
        };
        assert_eq!(
            hazard_ahead(&solid, &film, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)),
            Some(HazardKind::Lava)
        );
    }

    #[test]
    fn shallow_dry_basin_is_not_a_hazard() {
        // Same geometry with the film removed: the refined boundary sample finds plain floor, so a
        // bot may step into the shallow basin — the guard that we didn't just start flagging drops.
        let solid = |p: Vec3| if p.x <= 60.0 { p.z <= 24.0 } else { p.z <= -8.0 };
        let empty = |_: Vec3| CONTENTS_EMPTY;
        assert!(hazard_ahead(&solid, &empty, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn water_ahead_detects_shallow_water_film() {
        // A puddle-thin water film reads as Wet (deprioritized) — never as a lethal hazard.
        let solid = |p: Vec3| if p.x <= 60.0 { p.z <= 24.0 } else { p.z <= -8.0 };
        let film = |p: Vec3| {
            if p.x > 60.0 && (-32.0..-28.0).contains(&p.z) {
                CONTENTS_WATER
            } else {
                CONTENTS_EMPTY
            }
        };
        assert!(water_ahead(&solid, &film, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)));
        assert!(hazard_ahead(&solid, &film, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hazard_below_sees_shallow_slime_film() {
        // The same thin-film refinement catches slime a stride out over the basin.
        let solid = |p: Vec3| if p.x <= 60.0 { p.z <= 24.0 } else { p.z <= -8.0 };
        let film = |p: Vec3| {
            if p.x > 60.0 && (-32.0..-28.0).contains(&p.z) {
                CONTENTS_SLIME
            } else {
                CONTENTS_EMPTY
            }
        };
        assert_eq!(
            hazard_below(&solid, &film, Vec3::new(72.0, 0.0, 8.0)),
            Some(HazardKind::Slime)
        );
    }

    #[test]
    fn surface_above_open_pool_vs_roofed_tunnel() {
        // Open pool: water below z = 0, open air above it — a surface to swim up to.
        let open = |p: Vec3| if p.z < 0.0 { CONTENTS_WATER } else { CONTENTS_EMPTY };
        assert!(surface_above(&open, Vec3::new(0.0, 0.0, -40.0)));
        // Roofed tunnel: water below z = 0, solid ceiling above — no surface overhead.
        let roofed = |p: Vec3| if p.z < 0.0 { CONTENTS_WATER } else { CONTENTS_SOLID };
        assert!(!surface_above(&roofed, Vec3::new(0.0, 0.0, -40.0)));
    }

    // `ledge_ahead` is called with `feet` at the visual floor (origin − 24) and the real hull-1
    // `is_solid`, which reads solid up to the resting origin — 24u above the visual floor. The oracles
    // below model that bevel: a flat floor at visual height 0 is solid for z ≤ 24.

    #[test]
    fn ledge_ahead_flat_floor_is_safe() {
        let solid = |p: Vec3| p.z <= 24.0;
        assert!(!ledge_ahead(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn ledge_ahead_detects_a_drop() {
        // Floor for x ≤ 60, bottomless beyond: stepping +x runs off the edge, −x stays on floor.
        let solid = |p: Vec3| p.z <= 24.0 && p.x <= 60.0;
        assert!(ledge_ahead(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)));
        assert!(!ledge_ahead(&solid, Vec3::ZERO, Vec3::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn ledge_ahead_staircase_descent_is_safe() {
        // Steps down one STEP_HEIGHT (18) per 32u grid cell — a walkable descent, not a ledge.
        let solid = |p: Vec3| p.z <= 24.0 - 18.0 * (p.x / 32.0).floor().max(0.0);
        assert!(!ledge_ahead(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn ledge_ahead_wall_is_safe() {
        // A wall filling x > 60 at every height: a step-up/obstacle for collision, not a fall.
        let solid = |p: Vec3| p.z <= 24.0 || p.x > 60.0;
        assert!(!ledge_ahead(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn edge_bias_open_floor_no_nudge() {
        // Flat floor everywhere: neither side drops, so nothing to steer away from.
        let solid = |p: Vec3| p.z <= 24.0;
        assert_eq!(edge_bias(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)), Vec3::ZERO);
    }

    #[test]
    fn edge_bias_pushes_off_a_one_sided_drop() {
        // Floor for y >= -20, open shaft below/beyond it. Walking +x, the shaft is on the right
        // (-y) — the nudge should point +y (left), away from it.
        let solid = |p: Vec3| p.z <= 24.0 && p.y >= -20.0;
        let n = edge_bias(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        assert!(n.y > 0.5, "should steer +y off the right-hand drop: {n:?}");
        // Mirror it: shaft on the left (+y) → nudge -y (right).
        let solid_l = |p: Vec3| p.z <= 24.0 && p.y <= 20.0;
        assert!(edge_bias(&solid_l, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)).y < -0.5);
    }

    #[test]
    fn edge_bias_thin_path_holds_the_line() {
        // A narrow beam of floor |y| <= 16 with drops both sides: the pushes cancel, so a bot
        // balances straight down the middle instead of veering off one side.
        let solid = |p: Vec3| p.z <= 24.0 && p.y.abs() <= 16.0;
        assert_eq!(edge_bias(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)), Vec3::ZERO);
    }

    #[test]
    fn edge_bias_step_down_is_not_an_edge() {
        // A single walkable step (18u) down on the right is fine — not steered away from.
        let solid = |p: Vec3| if p.y < -20.0 { p.z <= 24.0 - 18.0 } else { p.z <= 24.0 };
        assert_eq!(edge_bias(&solid, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)), Vec3::ZERO);
    }
}
