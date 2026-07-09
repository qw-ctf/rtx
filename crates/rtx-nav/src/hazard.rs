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

use crate::bsp::{CONTENTS_LAVA, CONTENTS_SLIME, CONTENTS_WATER};

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

/// Classify what's below `p` by marching down: lava/slime (from `contents`) or a big drop / pit
/// (from `is_solid`). `None` if solid ground sits close below, or if *water* is below — swimmable
/// water breaks a fall and is harmless, so it is never a hazard (a deep pool would otherwise march
/// past its surface to the floor and misread as a pit).
pub(crate) fn hazard_below(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> f32,
    p: Vec3,
) -> Option<HazardKind> {
    let mut d = 0.0;
    while d <= HAZARD_PROBE_DEPTH {
        let q = p - Vec3::new(0.0, 0.0, d);
        let c = contents(q);
        if c == CONTENTS_LAVA as f32 {
            return Some(HazardKind::Lava);
        }
        if c == CONTENTS_SLIME as f32 {
            return Some(HazardKind::Slime);
        }
        if c == CONTENTS_WATER as f32 {
            return None; // swimmable — harmless, and not a pit no matter how deep
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
pub fn find_hazard(
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

/// Whether stepping from `feet` along horizontal unit `dir` walks into a hazard within the next
/// stride or two: probe a near and a far point ahead (feet-plus-a-little height) and classify what
/// lies below each. A probe that lands *inside solid* is a wall, not a pit — normal collision
/// handles that — so it's skipped, not treated as unsafe. Pure over the two oracles, like
/// [`hazard_below`]; the game's combat movement guard uses it to veto a lethal step.
pub fn hazard_ahead(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> f32,
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
                CONTENTS_LAVA as f32
            } else {
                CONTENTS_EMPTY as f32
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
        let empty = |_: Vec3| CONTENTS_EMPTY as f32;
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
                CONTENTS_LAVA as f32
            } else {
                CONTENTS_EMPTY as f32
            }
        };
        assert!(find_hazard(&solid, &contents, Vec3::new(40.0, 0.0, 24.0)).is_none());
    }

    #[test]
    fn open_floor_has_no_hazard() {
        let empty = |_: Vec3| CONTENTS_EMPTY as f32;
        assert!(find_hazard(&floor, &empty, Vec3::new(0.0, 0.0, 24.0)).is_none());
    }

    #[test]
    fn deep_water_is_safe() {
        // A pool of water with no floor within reach: harmless, must not read as a pit.
        let never_solid = |_: Vec3| false;
        let water = |p: Vec3| {
            if p.z < 0.0 {
                CONTENTS_WATER as f32
            } else {
                CONTENTS_EMPTY as f32
            }
        };
        assert!(hazard_below(&never_solid, &water, Vec3::new(0.0, 0.0, 24.0)).is_none());
        // And a bot standing at its edge is not warned away from swimmable water.
        assert!(hazard_ahead(&never_solid, &water, Vec3::new(0.0, 0.0, 24.0), Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hazard_ahead_detects_lava_edge() {
        // Lava fills x > 60 below the floor; a bot at the origin stepping +x walks toward it.
        let solid = |p: Vec3| p.z <= 0.0 && p.x <= 60.0;
        let contents = |p: Vec3| {
            if p.x > 60.0 && p.z < 0.0 {
                CONTENTS_LAVA as f32
            } else {
                CONTENTS_EMPTY as f32
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
        let empty = |_: Vec3| CONTENTS_EMPTY as f32;
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(hazard_ahead(&floor, &empty, feet, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hazard_ahead_wall_is_not_a_hazard() {
        // Solid wall filling all of x > 60 (including at knee height): a wall, not a pit.
        let solid = |p: Vec3| p.z <= 0.0 || p.x > 60.0;
        let empty = |_: Vec3| CONTENTS_EMPTY as f32;
        let feet = Vec3::new(0.0, 0.0, 24.0);
        assert!(hazard_ahead(&solid, &empty, feet, Vec3::new(1.0, 0.0, 0.0)).is_none());
    }
}
