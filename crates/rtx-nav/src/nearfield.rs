// SPDX-License-Identifier: AGPL-3.0-or-later

//! A fine (8u) near-field clearance grid the bots build around themselves for the *last metre* of
//! steering — the resolution the 32u navmesh deliberately doesn't have. Routing stays on the coarse
//! mesh + LOD abstraction; this layer only reshapes the immediate wish so a grounded bot rounds
//! corners, threads doorways, and lines up on items without scraping walls or drifting off ledges.
//!
//! It is a **repulsion field**, not a mini-navmesh: no links, no A*. [`NearField::build`] floods the
//! walkable columns out from the bot's own footing (so a bridge over a tunnel resolves to the floor
//! *connected to the bot*, and stairs climb by construction), classifying every 8u column as
//! walkable / wall / drop. [`NearField::steer_push`] then reads that grid — a generalisation of
//! [`crate::hazard::edge_bias`] to eight directions with a continuous ramp — returning a horizontal
//! nudge that pushes off nearby walls and drop-edges and cancels to zero between symmetric ones
//! (doorway centring, thin-beam balance). [`NearField::chord_clear`] certifies a straight short-cut
//! stays on clear floor, for a look-ahead glide.
//!
//! Everything is **pure** over two oracles — `is_solid` (the clip-hull point test,
//! [`crate::bsp::Bsp::is_solid`], already inflated by the ±16 player box so "not solid at an origin"
//! means the standing hull fits there) and `is_hazard` (whether a floor point sits in lava/slime,
//! which the liquid-blind clip hull can't tell, so flush lava would otherwise read as plain floor) —
//! plus a caller-supplied list of blocked boxes (closed button-gated doors, which the world hull can't
//! see). Keeping them closures lets the same logic run against a live map, the engine, or a synthetic
//! test fixture, exactly like [`crate::hazard`].

use std::collections::VecDeque;

use glam::{Vec3, Vec3Swizzles};

use crate::navmesh::PLAYER_HALF_WIDTH;
use crate::qphys::STEP_HEIGHT;

/// Grid pitch — a quarter of the navmesh's 32u [`GRID`](crate::navmesh::GRID), fine enough that a
/// one-body doorway spans several columns.
pub const NEAR_RES: f32 = 8.0;
/// Columns per side. 48 · 8 = a 384u-wide footprint centred on the bot — a couple of rooms of
/// look-around, comfortably past the ~24u the repulsion actually reaches.
pub const NEAR_N: usize = 48;
/// Half the footprint width (`NEAR_N · NEAR_RES / 2`), for the caller's gate-intersection test.
pub const NEAR_HALF: f32 = NEAR_N as f32 * NEAR_RES / 2.0;
/// Rebuild once the bot strays this far (horizontally) from the field's centre — half the footprint,
/// so the bot is always deep inside a valid grid when steering off it.
pub const NEAR_RECENTER: f32 = 96.0;
/// Rebuild once the bot's height leaves the seed floor by this much (a lift ride, a tall stair run):
/// the field's per-column floors were flooded from the old level and go stale a couple of steps out.
pub const NEAR_Z_RECENTER: f32 = 48.0;

/// Vertical sweep step when probing a column for its floor (mirrors the navmesh carve's `SCAN_DZ`).
const SCAN_DZ: f32 = 8.0;

/// The furthest a wall/drop is felt (the repulsion ramps to zero here). One column past it the sample
/// contributes nothing, so this also bounds the outward march.
const REACH: f32 = 40.0;
/// How the repulsion reacts to a **drop** edge beside the path — at full weight. Falling is worse than
/// scraping, so a drop is felt harder than a wall (mirroring [`crate::hazard::edge_bias`], which reacts
/// only to drops).
const DROP_WEIGHT: f32 = 1.0;
/// How the repulsion reacts to a **wall** — gentler than a drop, so a bot rounds a corner and stops
/// scraping without being pinned to the far wall or stalling.
const WALL_WEIGHT: f32 = 0.6;

/// The flood classifies columns only within this radius of the bot. [`steer_push`](NearField::steer_push)
/// reads at most [`REACH`] from a bot that has strayed up to [`NEAR_RECENTER`] off-centre, so anything
/// past `NEAR_RECENTER + REACH` is never read — leaving the far corners of the 384u footprint `Unknown`
/// (never classified) skips those hull traces and roughly halves the per-build cost on open maps, which
/// was overrunning the frame when a rebuild met a repath. The glide, which looks further, degrades to
/// the raw waypoint out here (`chord_clear` reads `Unknown` as not-walkable — its safe fallback).
const FLOOD_REACH: f32 = NEAR_RECENTER + REACH;

/// The eight compass directions the repulsion probes, pre-normalised (diagonals at 0.707). Shared
/// shape with [`crate::hazard::HAZARD_DIRS`].
const DIRS8: [(f32, f32); 8] = [
    (1.0, 0.0),
    (0.707, 0.707),
    (0.0, 1.0),
    (-0.707, 0.707),
    (-1.0, 0.0),
    (-0.707, -0.707),
    (0.0, -1.0),
    (0.707, -0.707),
];

/// Classification of one 8u column, flooded out from the bot's footing.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Col {
    /// Not yet examined by the flood (interior beyond the walkable frontier). Sampled as a wall — it
    /// only ever sits well outside the ~24u the repulsion reaches, so the conservative label is inert.
    Unknown,
    /// A wall / obstacle / step too tall to walk onto, or a closed gate's volume. Repels.
    Wall,
    /// A ledge: the floor falls away by more than a step. Repels (harder than a wall).
    Drop,
    /// Walkable footing whose floor is lava/slime — the clip hull can't see it (liquid-blind), so it
    /// is classified from a caller-supplied contents oracle. Repels like a drop (falling in is fatal)
    /// and, like a drop/wall, does not extend the walkable frontier — the field ends at the shore.
    Hazard,
    /// Walkable, at this resting-origin floor height (reachable from the bot within step-sized moves).
    Walk(f32),
}

impl Col {
    /// Whether a sample lands on walkable floor (not a wall/drop/hazard/unexamined column).
    fn walkable(self) -> bool {
        matches!(self, Col::Walk(_))
    }
}

/// A bot's local clearance field. Built around the bot ([`build`](Self::build)), read per frame
/// ([`steer_push`](Self::steer_push)) until it drifts stale ([`valid_for`](Self::valid_for)).
pub struct NearField {
    /// Snapped grid centre (xy on the 8u lattice) carrying the seed floor height in z.
    center: Vec3,
    /// World-xy of the grid's `(0, 0)` corner (`center − NEAR_HALF`), so `col_at` is a plain divide.
    corner: Vec3,
    /// The closed-gate bitmask the field was built under (see the caller's `gate_key`); a change
    /// (a nearby door opened/shut) invalidates it.
    gate_key: u32,
    /// Row-major `NEAR_N × NEAR_N` classification, indexed `j · NEAR_N + i`.
    grid: Vec<Col>,
}

impl NearField {
    /// Whether this field still describes the bot's surroundings: same closed-gate state, and the bot
    /// hasn't strayed past the recenter radius horizontally or the seed floor vertically. No timers —
    /// the world is static, so these are the only honest triggers.
    pub fn valid_for(&self, origin: Vec3, gate_key: u32) -> bool {
        self.gate_key == gate_key
            && (origin.xy() - self.center.xy()).length() <= NEAR_RECENTER
            && (origin.z - self.center.z).abs() <= NEAR_Z_RECENTER
    }

    /// Build the field around `origin` (the bot's standing origin). `blocked` are closed-gate world
    /// AABBs the world hull can't see — their volume is stamped unwalkable so the field doesn't route
    /// a bot through a shut door. `is_hazard` reports whether a floor point sits in lava/slime — the
    /// clip hull is liquid-blind, so a flush lava pool would otherwise read as ordinary walkable floor;
    /// a walkable column whose floor is a hazard becomes [`Col::Hazard`] and repels. `gate_key` is
    /// stored for [`valid_for`](Self::valid_for).
    pub fn build(
        is_solid: &impl Fn(Vec3) -> bool,
        is_hazard: &impl Fn(Vec3) -> bool,
        origin: Vec3,
        blocked: &[(Vec3, Vec3)],
        gate_key: u32,
    ) -> NearField {
        let snap = |v: f32| (v / NEAR_RES).round() * NEAR_RES;
        let center = Vec3::new(snap(origin.x), snap(origin.y), origin.z);
        // Place the corner so the bot's column (`N/2`) is *centred* on the bot — a half-cell shift, so
        // the left/right (and up/down) samples land symmetrically and symmetric obstacles truly cancel.
        let corner = Vec3::new(center.x - NEAR_HALF - NEAR_RES * 0.5, center.y - NEAR_HALF - NEAR_RES * 0.5, 0.0);
        let mut grid = vec![Col::Unknown; NEAR_N * NEAR_N];

        // Centre of column (i, j) at the given height.
        let col_center = |i: usize, j: usize, z: f32| {
            Vec3::new(corner.x + (i as f32 + 0.5) * NEAR_RES, corner.y + (j as f32 + 0.5) * NEAR_RES, z)
        };

        // Seed the bot's own column with its actual footing and flood the walkable neighbourhood.
        let (si, sj) = (NEAR_N / 2, NEAR_N / 2);
        grid[sj * NEAR_N + si] = Col::Walk(origin.z);
        let mut queue = VecDeque::from([(si, sj)]);
        while let Some((i, j)) = queue.pop_front() {
            let Col::Walk(cz) = grid[j * NEAR_N + i] else { continue };
            for (di, dj) in [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)] {
                let (ni, nj) = (i as i32 + di, j as i32 + dj);
                if !(0..NEAR_N as i32).contains(&ni) || !(0..NEAR_N as i32).contains(&nj) {
                    continue;
                }
                let (ni, nj) = (ni as usize, nj as usize);
                let idx = nj * NEAR_N + ni;
                if grid[idx] != Col::Unknown {
                    continue; // already classified this frame
                }
                // Past the read radius: leave it Unknown and don't pay to classify or flood on.
                let (ddi, ddj) = ((ni as f32 - si as f32) * NEAR_RES, (nj as f32 - sj as f32) * NEAR_RES);
                if ddi * ddi + ddj * ddj > FLOOD_REACH * FLOOD_REACH {
                    continue;
                }
                let c = col_center(ni, nj, cz);
                let col = if blocks(c, cz, blocked) {
                    Col::Wall // a shut gate's volume — invisible to the world hull, stamped here
                } else {
                    // Walkable footing over lava/slime (the liquid-blind clip hull can't tell) becomes
                    // a repelling Hazard, tested only on the columns geometry already calls walkable.
                    match probe_column(is_solid, c.x, c.y, cz) {
                        Col::Walk(f) if is_hazard(Vec3::new(c.x, c.y, f)) => Col::Hazard,
                        other => other,
                    }
                };
                grid[idx] = col;
                if col.walkable() {
                    queue.push_back((ni, nj)); // only walkable columns extend the frontier
                }
            }
        }

        NearField { center, corner, gate_key, grid }
    }

    /// The column covering world point `p` (z ignored — the grid is 2.5D), or `None` if `p` is off
    /// the footprint.
    fn col_at(&self, p: Vec3) -> Option<Col> {
        let i = ((p.x - self.corner.x) / NEAR_RES).floor() as i32;
        let j = ((p.y - self.corner.y) / NEAR_RES).floor() as i32;
        ((0..NEAR_N as i32).contains(&i) && (0..NEAR_N as i32).contains(&j))
            .then(|| self.grid[j as usize * NEAR_N + i as usize])
    }

    /// A horizontal steering nudge at `p`: for each of eight directions, the nearest wall/drop within
    /// its margin contributes a push *inward* (away from it), ramped up as it nears. Symmetric
    /// obstacles cancel — a doorway or corridor centres, a thin beam with drops both sides holds the
    /// line — while a one-sided wall or ledge pushes off it, and open floor returns zero.
    ///
    /// `None` when `p` is off the footprint or on an unwalkable column (the bot was shoved/teleported
    /// off its own field) — the caller falls back to the live [`crate::hazard::edge_bias`] probe. The
    /// magnitude is clamped to 1 so it composes with the unit waypoint direction like `edge_bias`.
    pub fn steer_push(&self, p: Vec3) -> Option<Vec3> {
        if !self.col_at(p)?.walkable() {
            return None;
        }
        let mut push = Vec3::ZERO;
        for (dx, dy) in DIRS8 {
            let dir = Vec3::new(dx, dy, 0.0);
            // March outward; the first blocked column in this direction sets the contribution.
            let mut r = NEAR_RES;
            while r <= REACH {
                match self.col_at(p + dir * r) {
                    Some(Col::Walk(_)) => {} // clear here — keep looking outward
                    Some(Col::Drop) | Some(Col::Hazard) => {
                        // A ledge or a lava edge — both fatal to walk off; push off at full weight.
                        push -= dir * DROP_WEIGHT * ramp(r, REACH);
                        break;
                    }
                    Some(Col::Wall) | Some(Col::Unknown) => {
                        push -= dir * WALL_WEIGHT * ramp(r, REACH);
                        break;
                    }
                    None => break, // off the footprint — nothing to react to out here
                }
                r += NEAR_RES;
            }
        }
        Some(push.clamp_length_max(1.0))
    }

    /// Whether the straight chord `a`→`b` stays on walkable floor with at least `margin` clearance to
    /// any wall/drop the whole way — a pure grid read (no tracing), for certifying a look-ahead glide
    /// short-cut before steering onto it.
    pub fn chord_clear(&self, a: Vec3, b: Vec3, margin: f32) -> bool {
        let steps = ((b.xy() - a.xy()).length() / NEAR_RES).ceil().max(1.0) as i32;
        (0..=steps).all(|i| {
            let p = a.lerp(b, i as f32 / steps as f32);
            self.col_at(p).is_some_and(Col::walkable) && self.clear_by(p, margin)
        })
    }

    /// Whether no wall/drop column lies within `margin` of `p` (a small box scan around `p`).
    fn clear_by(&self, p: Vec3, margin: f32) -> bool {
        let cells = (margin / NEAR_RES).ceil() as i32;
        for dj in -cells..=cells {
            for di in -cells..=cells {
                let q = p + Vec3::new(di as f32 * NEAR_RES, dj as f32 * NEAR_RES, 0.0);
                let blocked = match self.col_at(q) {
                    Some(c) => !c.walkable(),
                    None => false, // off-footprint is unknown, not a hazard to brake for
                };
                if blocked && (q.xy() - p.xy()).length() <= margin {
                    return false;
                }
            }
        }
        true
    }
}

/// Linear ramp: 1 at `r == 0`, 0 at `r >= margin`.
fn ramp(r: f32, margin: f32) -> f32 {
    (1.0 - r / margin).clamp(0.0, 1.0)
}

/// Whether column centre `c` (at floor height `z`) lies inside any blocked box, grown by the player's
/// horizontal half-width (the standard agent-radius inflation the gate splice uses too, so a body
/// beside a shut door still reads as blocked). The z test is lenient — a door spanning head height
/// blocks a body whose feet sit anywhere in its span.
fn blocks(c: Vec3, z: f32, blocked: &[(Vec3, Vec3)]) -> bool {
    let m = PLAYER_HALF_WIDTH;
    blocked.iter().any(|&(lo, hi)| {
        c.x >= lo.x - m
            && c.x <= hi.x + m
            && c.y >= lo.y - m
            && c.y <= hi.y + m
            && z >= lo.z - STEP_HEIGHT
            && z <= hi.z
    })
}

/// Classify the column at `(x, y)` relative to the neighbouring floor height `z_ref`: walkable (with
/// its own bisected floor) when a floor sits within a [`STEP_HEIGHT`] of `z_ref`, a [`Col::Drop`] when
/// the floor falls away beyond that, a [`Col::Wall`] when solid fills the body's height (a pillar or a
/// too-tall step-up). Pure over `is_solid`.
fn probe_column(is_solid: &impl Fn(Vec3) -> bool, x: f32, y: f32, z_ref: f32) -> Col {
    let at = |z: f32| is_solid(Vec3::new(x, y, z));
    // Fast path: a flat / near-flat continuation — the body fits at `z_ref` and solid sits just below.
    // Covers the overwhelming majority of columns (open floor) in three samples plus the bisection.
    if !at(z_ref) && at(z_ref - SCAN_DZ) {
        return Col::Walk(bisect(is_solid, x, y, z_ref - SCAN_DZ, z_ref));
    }
    // Otherwise scan a step-sized window for the nearest floor (a step up or down within reach).
    let (lo, hi) = (z_ref - (STEP_HEIGHT + SCAN_DZ), z_ref + (STEP_HEIGHT + SCAN_DZ));
    let mut best: Option<f32> = None;
    let mut prev = at(lo);
    let mut z = lo;
    while z < hi {
        z += SCAN_DZ;
        let solid = at(z);
        if prev && !solid {
            let f = bisect(is_solid, x, y, z - SCAN_DZ, z);
            if best.is_none_or(|b| (f - z_ref).abs() < (b - z_ref).abs()) {
                best = Some(f);
            }
        }
        prev = solid;
    }
    match best {
        Some(f) if (f - z_ref).abs() <= STEP_HEIGHT => Col::Walk(f),
        Some(f) if f < z_ref => Col::Drop, // floor only below, beyond a step → a ledge
        Some(_) => Col::Wall,              // floor only above, beyond a step → an unmountable step-up
        None if at(z_ref) => Col::Wall,    // solid at body height → a wall / pillar
        None => Col::Drop,                 // open, no floor within reach → a drop / pit
    }
}

/// Bisect the resting-origin height between a solid sample below and an empty one above (four
/// halvings pin an 8u stride under 1u). Mirrors [`crate::navmesh`]'s build-time `bisect_floor`.
fn bisect(is_solid: &impl Fn(Vec3) -> bool, x: f32, y: f32, z_solid: f32, z_empty: f32) -> f32 {
    let (mut lo, mut hi) = (z_solid, z_empty);
    for _ in 0..4 {
        let mid = (lo + hi) * 0.5;
        if is_solid(Vec3::new(x, y, mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

#[cfg(test)]
mod tests {
    use super::*;

    // Oracles model the *already hull-inflated* clip test: "solid" means a standing origin placed
    // there is blocked. A plain floor is solid for z ≤ 0, so an origin rests at ≈ 0; the bot stands a
    // hair above it. Features are carved by x/y conditions, mirroring the `hazard::edge_bias` tests.

    const EYE: Vec3 = Vec3::new(0.0, 0.0, 1.0); // the bot's standing origin, just above a z ≤ 0 floor

    fn build(is_solid: &impl Fn(Vec3) -> bool) -> NearField {
        NearField::build(is_solid, &|_| false, EYE, &[], 0)
    }

    #[test]
    fn open_floor_no_push() {
        let solid = |p: Vec3| p.z <= 0.0;
        let nf = build(&solid);
        let push = nf.steer_push(EYE).expect("on walkable floor");
        assert!(push.length() < 1e-3, "flat floor should not nudge: {push:?}");
    }

    #[test]
    fn corridor_centres() {
        // A 32u-wide walkable band (walls solid at |y| > 16); floor at z ≤ 0 within it.
        let solid = |p: Vec3| p.y.abs() > 16.0 || p.z <= 0.0;
        let nf = build(&solid);
        // Centred: the two walls cancel.
        let mid = nf.steer_push(EYE).expect("walkable");
        assert!(mid.y.abs() < 0.2, "centre of the corridor should be balanced: {mid:?}");
        // Off-centre toward +y: pushed back toward the centre (−y).
        let off = nf.steer_push(Vec3::new(0.0, 8.0, 1.0)).expect("walkable");
        assert!(off.y < -0.2, "should steer back off the near (+y) wall: {off:?}");
    }

    #[test]
    fn pushes_off_a_one_sided_drop() {
        // Floor for y ≥ -16, an open drop beyond it (bottomless: never solid past the edge). Walking
        // the band, the drop is on the −y side → the nudge points +y, away from it. Mirrors
        // `hazard::edge_bias_pushes_off_a_one_sided_drop`.
        let solid = |p: Vec3| p.y >= -16.0 && p.z <= 0.0;
        let nf = build(&solid);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y > 0.3, "should steer +y off the −y drop: {push:?}");
    }

    #[test]
    fn thin_beam_holds_the_line() {
        // A narrow beam |y| ≤ 16 with drops both sides: the drop pushes cancel → balance straight.
        let solid = |p: Vec3| p.y.abs() <= 16.0 && p.z <= 0.0;
        let nf = build(&solid);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y.abs() < 0.2, "thin beam should hold the centre: {push:?}");
    }

    #[test]
    fn walkable_step_down_is_not_an_edge() {
        // A single walkable step (12u) down on the −y side is fine — not steered away from.
        let solid = |p: Vec3| if p.y < -16.0 { p.z <= -12.0 } else { p.z <= 0.0 };
        let nf = build(&solid);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y.abs() < 0.2, "a step down is not a ledge: {push:?}");
    }

    #[test]
    fn wall_on_one_side_pushes_off() {
        // Solid wall filling y > 16 at every height (a wall, not a drop): push off it, toward −y.
        let solid = |p: Vec3| p.y > 16.0 || p.z <= 0.0;
        let nf = build(&solid);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y < -0.2, "should steer −y off the +y wall: {push:?}");
    }

    #[test]
    fn flood_climbs_a_staircase() {
        // Steps up one STEP_HEIGHT (18) per 32u of +x — a walkable ascent. Columns up the stair must
        // flood as walkable, at rising floor heights, rather than reading as a wall of risers.
        let solid = |p: Vec3| p.z <= 18.0 * (p.x / 32.0).floor().max(0.0);
        let nf = build(&solid);
        // 96u along the stair (three steps up, ~54u) is still on walkable floor connected to the bot.
        let up = nf.col_at(Vec3::new(96.0, 0.0, 60.0)).expect("on footprint");
        assert!(up.walkable(), "staircase should flood walkable, got {up:?}");
    }

    #[test]
    fn doorway_gap_is_clear_but_flanks_block() {
        // A wall across x ≈ 40 with a 48u-wide doorway (|y| < 24). A chord straight through the gap
        // stays clear; one aimed a body-width to the side hits the doorframe.
        let solid = |p: Vec3| (p.x > 32.0 && p.x < 48.0 && p.y.abs() >= 24.0) || p.z <= 0.0;
        let nf = build(&solid);
        assert!(
            nf.chord_clear(EYE, Vec3::new(80.0, 0.0, 1.0), 0.0),
            "the doorway centre should be walkable through"
        );
        assert!(
            !nf.chord_clear(EYE, Vec3::new(60.0, 60.0, 1.0), 0.0),
            "a chord into the doorframe should be blocked"
        );
    }

    #[test]
    fn closed_gate_box_blocks_a_chord() {
        // Open floor, but a closed-door box straddles x ∈ [32, 48] across the whole width. The world
        // hull can't see it (floor is clear), so only the overlay makes the chord through it fail.
        let solid = |p: Vec3| p.z <= 0.0;
        let door = [(Vec3::new(32.0, -64.0, -8.0), Vec3::new(48.0, 64.0, 64.0))];
        let nf = NearField::build(&solid, &|_| false, EYE, &door, 1);
        assert!(
            !nf.chord_clear(EYE, Vec3::new(80.0, 0.0, 1.0), 0.0),
            "a shut gate's volume must block the chord the world hull can't see"
        );
        // Without the overlay the same geometry is wide open.
        let open = NearField::build(&solid, &|_| false, EYE, &[], 0);
        assert!(open.chord_clear(EYE, Vec3::new(80.0, 0.0, 1.0), 0.0));
    }

    #[test]
    fn pushes_off_a_one_sided_lava_edge() {
        // Flat floor everywhere (the clip hull sees no drop), but lava fills y < -16 — invisible to
        // is_solid, caught by is_hazard. Walking the band, the lava is on the −y side → push +y off it,
        // exactly like a geometric drop (`pushes_off_a_one_sided_drop`).
        let solid = |p: Vec3| p.z <= 0.0;
        let lava = |p: Vec3| p.y < -16.0;
        let nf = NearField::build(&solid, &lava, EYE, &[], 0);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y > 0.3, "should steer +y off the −y lava: {push:?}");
    }

    #[test]
    fn thin_walkway_between_lava_holds_the_line() {
        // A one-body walkway |y| ≤ 16 with lava both sides on otherwise-flat floor: the two hazard
        // pushes cancel → hold the centre (don't drift into either pool).
        let solid = |p: Vec3| p.z <= 0.0;
        let lava = |p: Vec3| p.y.abs() > 16.0;
        let nf = NearField::build(&solid, &lava, EYE, &[], 0);
        let push = nf.steer_push(EYE).expect("walkable");
        assert!(push.y.abs() < 0.2, "lava both sides should balance: {push:?}");
        // And a chord straight out along the walkway stays clear, but one angling into the lava fails.
        assert!(nf.chord_clear(EYE, Vec3::new(64.0, 0.0, 1.0), 0.0), "walkway centre is clear");
        assert!(!nf.chord_clear(EYE, Vec3::new(48.0, 48.0, 1.0), 0.0), "a chord into the lava is blocked");
    }

    #[test]
    fn off_field_returns_none() {
        // A point outside the 384u footprint has no data → None (caller falls back to a live probe).
        let solid = |p: Vec3| p.z <= 0.0;
        let nf = build(&solid);
        assert!(nf.steer_push(Vec3::new(10_000.0, 0.0, 1.0)).is_none());
    }

    #[test]
    fn valid_for_tracks_movement_and_gates() {
        let solid = |p: Vec3| p.z <= 0.0;
        let nf = NearField::build(&solid, &|_| false, EYE, &[], 0b10);
        assert!(nf.valid_for(EYE, 0b10), "fresh field is valid");
        assert!(nf.valid_for(Vec3::new(40.0, 0.0, 1.0), 0b10), "small move stays valid");
        assert!(!nf.valid_for(Vec3::new(200.0, 0.0, 1.0), 0b10), "past the recenter radius");
        assert!(!nf.valid_for(Vec3::new(0.0, 0.0, 200.0), 0b10), "a lift ride invalidates");
        assert!(!nf.valid_for(EYE, 0b11), "a nearby door changing state invalidates");
    }

    #[test]
    fn build_is_deterministic() {
        let solid = |p: Vec3| p.y.abs() > 16.0 || p.z <= 0.0;
        let a = build(&solid);
        let b = build(&solid);
        assert_eq!(a.grid, b.grid, "same inputs must yield the same field");
    }
}
