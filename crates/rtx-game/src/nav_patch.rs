//! Map-pinned navmesh patches, applied once when a build finishes.
//!
//! The column carve samples one column per `GRID` step of XY, so a walkable surface narrower than
//! that pitch and out of phase with it is invisible to the automatic build — see
//! [`NavGraph::plant_cell`]'s doc for the worked example, dm3's machinery shelf west of SNG. A bot
//! that ends up on such a surface localizes through `nearest` to a floor far below it, plans routes
//! that are fiction from where it actually stands, and wedges until the round ends.
//!
//! [`plant_cell`](NavGraph::plant_cell) / [`plant_drop`](NavGraph::plant_drop) exist for exactly
//! that failure class, but only as control-channel verbs — nothing applied them in production, so
//! every server restart forgot the shelf. This module is the missing wiring: a short, reviewable
//! table of hand-verified plants per map, applied right after the build finishes, gated by
//! `rtx_nav_patch` (default on).
//!
//! Fail-closed, and transactional: every mutation goes through the build's own validators
//! (`plant_cell` refuses non-standable spots, `plant_drop` accepts only a drop
//! `classify_grounded` would itself emit), each planted cell must land on the standing height
//! measured on the shipped BSP (`snap_z` ± [`SNAP_TOL`] — a *local geometry precondition*, not a
//! whole-BSP fingerprint: it catches the floor moving, and the link validators catch the
//! surroundings changing, but a map edit that keeps this exact floor height passes), and each
//! patch mutates a clone that only replaces the live graph when everything validated. The outcome
//! is one unambiguous console line per patch: `applied` / `skipped (...)` / `failed (...)`. A
//! skipped or failed patch leaves the graph bit-for-bit what the build produced.

use glam::Vec3;

use crate::bsp::Bsp;
use crate::navmesh::{LinkKind, NavGraph};

/// Tolerance around [`ShelfPatch::snap_z`] for the floor-snap fingerprint. Standing heights come
/// out of the hull trace at exact model coordinates, so a correct BSP matches to well under a unit;
/// anything past this is a different floor than the one the patch was measured on.
const SNAP_TOL: f32 = 0.5;

/// How close an existing cell must be (XY/Z) for a patch position to count as already meshed —
/// mirrors `plant_cell`'s own same-spot test, so the answer agrees with what planting would do.
const ALREADY_XY: f32 = 8.0;
const ALREADY_Z: f32 = 8.0;

/// One un-carved standable surface: the cells that give it honest positions and the drops that give
/// it a way off. Positions are aim points (a couple of units above the surface); `plant_cell` snaps
/// them to the actual floor.
pub struct ShelfPatch {
    /// `level.mapname` this patch is pinned to.
    pub map: &'static str,
    /// Short id for the console status line.
    pub name: &'static str,
    /// Cell aim points along the surface.
    pub cells: &'static [[f32; 3]],
    /// `(from, to)` aim points per drop; `from` must resolve to a patch cell, `to` to a carved cell.
    pub drops: &'static [([f32; 3], [f32; 3])],
    /// Standing height every planted cell must snap to on the shipped BSP.
    pub snap_z: f32,
}

/// The pinned patch table. One entry so far.
///
/// dm3 west shelf (`sng-t`/`lifts` boundary, x −920..−845, y −48): the machinery-top strip bots
/// climb onto in pairs during normal play and then never leave — measured on upstream main
/// (cc5fa8ea) at 13/0/0/20/94/53 stall firings per 600 s T2 across six runs, with per-bot
/// standstill rising to 27-33.5 s in the big-episode runs (10.6-19 s otherwise). The south face is solid (drops that way fail
/// `classify_grounded`); the open lip is north, so every drop lands on the y=0 floor row. Cell
/// spacing follows the measured wander range of trapped bots (x −847..−872 around each episode's
/// entry point) so localization never has to reach more than ~15 u.
pub const PATCHES: &[ShelfPatch] = &[ShelfPatch {
    map: "dm3",
    name: "west-shelf",
    cells: &[
        [-920.0, -48.0, 90.0],
        [-895.0, -48.0, 90.0],
        [-865.0, -48.0, 90.0],
        [-845.0, -48.0, 90.0],
    ],
    drops: &[
        ([-920.0, -48.0, 90.0], [-920.0, 0.0, -16.0]),
        ([-895.0, -48.0, 90.0], [-895.0, 0.0, -16.0]),
        ([-865.0, -48.0, 90.0], [-865.0, 0.0, -16.0]),
        ([-845.0, -48.0, 90.0], [-845.0, 0.0, -16.0]),
    ],
    snap_z: 88.03125,
}];

/// Endpoint resolution bounds for a drop's `to` point — same rationale and values as the control
/// channel's `PlanDrop`: a target with nothing near it must be an error, not a silent snap to
/// whatever cell is closest somewhere else on the map.
const REACH_XY: f32 = 48.0;
const REACH_Z: f32 = 48.0;

/// What applying one patch did. Rendered into the console status line by the caller.
pub enum Outcome {
    /// New topology went in (counts are the *new* cells/drops; pre-existing ones are not counted).
    Applied { cells: usize, drops: usize },
    /// Every cell **and every drop** the patch asks for is already in the graph — a future carve
    /// that genuinely sees the whole surface makes the patch a no-op rather than a conflict. A
    /// carve that finds the cells but still misses the way off does *not* qualify; the missing
    /// drops get planted and the patch reports `Applied`.
    AlreadyMeshed,
    /// A precondition or a validator said no. The candidate graph is discarded whole, so the
    /// published graph — derived tables included — is bit-for-bit the one the build produced.
    Failed(String),
}

/// Apply every patch pinned to `map`, in table order — transactionally: each patch mutates a
/// clone, which replaces `graph` (derived tables rebuilt) only when the whole patch validated.
/// A `Failed` patch therefore cannot leave partial topology or stale reachability/LOD behind.
pub fn apply_for_map(map: &str, bsp: &Bsp, graph: &mut NavGraph) -> Vec<(&'static str, Outcome)> {
    let mut out = Vec::new();
    for patch in PATCHES.iter().filter(|p| p.map == map) {
        let mut candidate = graph.clone();
        let outcome = apply_one(patch, bsp, &mut candidate);
        if let Outcome::Applied { .. } = outcome {
            candidate.rebuild_derived();
            *graph = candidate;
        }
        out.push((patch.name, outcome));
    }
    out
}

fn apply_one(patch: &ShelfPatch, bsp: &Bsp, graph: &mut NavGraph) -> Outcome {
    let v = |a: [f32; 3]| Vec3::new(a[0], a[1], a[2]);

    let mut new_cells = 0;
    for &c in patch.cells {
        // "New" is judged by what `plant_cell` actually did (its dedup runs against the *snapped*
        // position, which an aim point can sit further from than any pre-check here would use) —
        // the cell count grows exactly when a cell was genuinely planted.
        let cells_before = graph.cells.len();
        let Some((id, _)) = graph.plant_cell(bsp, v(c)) else {
            return Outcome::Failed(format!("no standable floor at {c:?}"));
        };
        let existed = graph.cells.len() == cells_before;
        let z = graph.cell_origin(id).z;
        if (z - patch.snap_z).abs() > SNAP_TOL {
            return Outcome::Failed(format!(
                "cell at {c:?} snapped to z={z}, expected {} ± {SNAP_TOL} — the floor here is not \
                 the one this patch was measured on",
                patch.snap_z
            ));
        }
        if !existed {
            new_cells += 1;
        }
    }

    let mut new_drops = 0;
    for &(from, to) in patch.drops {
        let Some(from_cell) = graph.cell_within(v(from), ALREADY_XY, ALREADY_Z) else {
            return Outcome::Failed(format!("drop from {from:?} resolves to no cell"));
        };
        let Some(to_cell) = graph.cell_within(v(to), REACH_XY, REACH_Z) else {
            return Outcome::Failed(format!("drop to {to:?} resolves to no cell"));
        };
        // `plant_drop` does not deduplicate; an equivalent drop already in the graph (an earlier
        // patch run, or a carve that learned the lip) is simply kept.
        if graph
            .links
            .iter()
            .any(|l| l.from == from_cell && l.to == to_cell && l.kind == LinkKind::Drop)
        {
            continue;
        }
        if graph.plant_drop(bsp, from_cell, to_cell).is_none() {
            return Outcome::Failed(format!("drop {from:?} -> {to:?} is not one the build would emit"));
        }
        new_drops += 1;
    }

    if new_cells == 0 && new_drops == 0 {
        return Outcome::AlreadyMeshed;
    }
    Outcome::Applied {
        cells: new_cells,
        drops: new_drops,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The table is data reviewed by eye; these hold the invariants the apply loop assumes.
    #[test]
    fn table_is_well_formed() {
        for p in PATCHES {
            assert!(!p.map.is_empty() && p.map == p.map.to_lowercase());
            assert!(!p.cells.is_empty(), "{}: a patch with no cells patches nothing", p.name);
            assert!(
                !p.drops.is_empty(),
                "{}: a shelf with no way off is still a trap",
                p.name
            );
            for (from, _) in p.drops {
                assert!(
                    p.cells.iter().any(|c| {
                        let dx = c[0] - from[0];
                        let dy = c[1] - from[1];
                        (dx * dx + dy * dy).sqrt() <= ALREADY_XY
                    }),
                    "{}: every drop must start on a patch cell",
                    p.name
                );
            }
        }
    }
}
