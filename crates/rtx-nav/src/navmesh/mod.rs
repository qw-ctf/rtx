// SPDX-License-Identifier: AGPL-3.0-or-later

//! Auto-generated navigation mesh, built once at map load from the BSP player clip hull
//! (see [`crate::bsp`]). Bots route over this instead of hand-authored waypoint files.
//!
//! The build is pure and offline (no engine syscalls): it asks the hull-1 solidity oracle
//! ([`Bsp::is_solid`], a point test that already accounts for the player box) where a player
//! can stand, drops a [`Cell`] at each floor, and classifies the moves between nearby cells
//! into [`Link`]s (walk / step / drop / jump). Costs are travel times, so A* over the graph
//! yields fast routes (P2).
//!
//! The static-hull cut emits `Walk`/`Step`/`Drop`/`JumpGap`; on top of it come `DoubleJump`,
//! `SpeedJump`, `Hook`, and `RocketJump` movement links (each solved offline against the same
//! solidity oracle), and the entity-derived `Plat`/`Teleport`/gate splices.

use std::collections::HashMap;

use glam::{Vec2, Vec3, Vec3Swizzles};
use rayon::prelude::*;

mod geom;
mod hook;
mod jumps;
mod physics;
mod query;
mod rocketjump;
mod sidetable;
mod splice;

pub use geom::arc_point;
use geom::*;
pub use hook::arc_land;
use hook::{hook_cost, march_to_solid, perturb_ok, HOOK_PITCHES};
#[cfg(test)]
use hook::{simulate_arc, ArcResult};
pub use physics::{
    attainable_speed, band_of, bhop_k, prestrafe_delivered_from, BAND_EDGES, BAND_FLOOR, BAND_V_MAX, BHOP_EFF,
    CURL_V_HOLD_TOL, DOUBLE_ARC_PEAK, JUMP_APEX, MAX_SPEED, NBANDS,
};
use physics::*;
use rocketjump::{rj_perturb_ok, rocket_jump_cost, simulate_rocket_jump, RJ_DELAYS, RJ_PITCHES};
use sidetable::SideTable;
pub use splice::{Gate, GateInfo, Plat, PlatInfo, TeleportInfo};

use crate::bsp::Bsp;
use crate::qphys::STEP_HEIGHT;

// --- grappling-hook traversal (see `add_hooks`) ---

/// Height above a cell's standing origin the hook launches from (`throw_grapple` spawns it at
/// `origin + 16z`; the small `v_forward*16` XY offset is absorbed by the range margin).
const HOOK_LAUNCH_Z: f32 = 16.0;
/// Reel-in speed at the `rtx_hook_pull ×1` default (`2.35 · 320`, from `grapple.rs`). The live
/// multiplier scales this; the build takes it as a [`HookParams`] field.
pub const HOOK_PULL_BASE: f32 = 2.35 * 320.0;
/// Hook throw (projectile) speed at `rtx_hook_speed ×1` (`2.5 · 320`). Only feeds the flight-time
/// term of the cost; the live multiplier is applied from [`HookParams`].
pub const HOOK_THROW_BASE: f32 = 2.5 * 320.0;
/// Longest rope we'll consider — caps the anchor ray-march and keeps costs bounded.
const HOOK_ROPE_MAX: f32 = 1024.0;
/// Max horizontal reach of a hook link (a fling can cross a wide gap), bounding the candidate scan.
const HOOK_RANGE_XY: f32 = 640.0;
/// Highest rise a hook link may climb (reel + fling reaches well past a plain jump).
const HOOK_MAX_RISE: f32 = 512.0;
/// Lowest a target may sit below the source and still be a hook link (a descending fling).
const HOOK_MIN_RISE: f32 = -128.0;
/// Ray-march / arc-clearance sampling step (matches `path_clear` granularity).
const HOOK_SAMPLE: f32 = 16.0;
/// Spacing of candidate release points sampled along the reel rope.
const HOOK_R_STEP: f32 = 24.0;
/// Fixed timestep for the offline parabola integration (~15 u/step at reel speed).
const HOOK_SIM_DT: f32 = 0.02;
/// Cap on simulated airtime before a candidate arc is abandoned.
const HOOK_MAX_AIRTIME: f32 = 2.5;
/// Landing acceptance: the descending arc must pass within this XY of the target cell…
const HOOK_LAND_XY: f32 = 24.0;
/// …and within this Z window above it.
const HOOK_LAND_Z: f32 = 48.0;
/// Fixed overhead charged to every hook link: aim-settle + weapon switch + throw/release latency.
const HOOK_OVERHEAD: f32 = 1.2;
/// At most this many hook links per source cell (post octant/elevation dedup), to bound explosion.
const HOOK_MAX_PER_CELL: usize = 4;

// --- rocket jumps (blast-launched leaps up to high ledges) ---

/// Max horizontal reach of a rocket-jump link. A floor-fired RJ is mostly vertical, so the reach is
/// tighter than a hook's — an RJ that also travels far is rare and fragile.
const RJ_RANGE_XY: f32 = 400.0;
/// Highest rise a rocket-jump link may climb — the realistic apex (~280u) plus landing slack.
const RJ_MAX_RISE: f32 = 320.0;
/// Lowest a target may sit above the source and still be worth a rocket jump (below this a jump or
/// double jump already reaches — see the useful-gate).
const RJ_MIN_RISE: f32 = 40.0;
/// Landing acceptance window: how far the solved touchdown may sit from a cell for the shot to count
/// as landing *on* it. Tight in Z — a rocket jump must put the bot squarely on the ledge, not a
/// player-height below it. A looser Z (was 48, a full hull) let `nearest_within` snap a landing that
/// fell ~42u short up onto the higher target cell, minting links that structurally undershoot (~24%
/// of them). 24 keeps the snap to at most a half-hull, so a generated link actually reaches its target.
const RJ_LAND_XY: f32 = 24.0;
const RJ_LAND_Z: f32 = 24.0;
/// At most this many rocket-jump links per source cell — kept small (each costs the bot ~50HP to
/// fly, so a map wants a handful of genuinely-useful ones, not a spray).
const RJ_MAX_PER_CELL: usize = 2;

// --- grid ---

/// XY sampling step. 32 = the player's full width: one column per body. Coarser than the
/// plan's 16 to keep the build cheap on big maps; thin ledges may be missed (revisit). Public so a
/// viewer can tile each cell's walkable footprint at the true grid pitch.
pub const GRID: f32 = 32.0;
/// Player hull half-width (the QW player box is ±16 in X/Y). Used to grow obstacles by the agent
/// radius so a bot doesn't clip geometry its path's centre-line technically clears.
const PLAYER_HALF_WIDTH: f32 = 16.0;
/// Vertical sweep step when scanning a column for floors (refined by bisection after).
const SCAN_DZ: f32 = 8.0;
/// Spacing of the floor-continuity samples along a grounded link (see [`geom::ground_along`]).
/// 8 = four per grid cell; a real support gap only exists where the opening exceeds the 32u player
/// box, so an 8u stride can't step over one.
const GROUND_SAMPLE: f32 = 8.0;
/// Slack added to [`STEP_HEIGHT`] when probing for floor under a grounded link: the interpolated
/// origin height can sit up to a step off the true resting height on a riser, plus plane noise.
const GROUND_SLACK: f32 = 4.0;

/// A standable spot: the player *origin* position when standing here (feet are
/// `ORIGIN_TO_FEET` below `origin.z`). Tagged with its grid column for neighbor lookup.
#[derive(Clone, Copy)]
pub struct Cell {
    pub origin: Vec3,
    pub gx: i32,
    pub gy: i32,
}

pub type CellId = u32;

/// XY spatial index: grid column `(gx, gy)` → the cells carved in that column.
type GridIndex = HashMap<(i32, i32), Vec<CellId>>;

/// How a bot traverses a directed link between two cells.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LinkKind {
    /// Effectively flat ground.
    Walk,
    /// Step up/down within `STEP_HEIGHT` — pmove handles it with no jump.
    Step,
    /// One-way fall off a ledge (down only).
    Drop,
    /// A run-jump across a gap or up to a ledge within reach.
    JumpGap,
    /// A **double jump** across a wider gap / up to a higher ledge than a single jump reaches — the
    /// bot ground-jumps, then air-jumps near the apex (rtx's `rtx_doublejump`). Only emitted when the
    /// map has double jump enabled.
    DoubleJump,
    /// A **speed jump**: a leap across a gap wider than any single/double jump, cleared by arriving
    /// at the takeoff with **bunnyhop-built speed**. The link's `from` is the *start of the runway*
    /// (not the ledge), so taking it means running the whole run-up — the bot is guaranteed the speed
    /// by construction. The [`SpeedJumpTraversal`] side table carries the takeoff point and required
    /// speed. Only emitted when bots bhop (`rtx_bot_bhop`).
    SpeedJump,
    /// Riding a `func_plat`: board it at the bottom and let it carry you to the top. The
    /// link's `from` cell is the standing spot on the plat (its centre), `to` the floor the
    /// plat delivers to. Bots stay centred and wait rather than steering off.
    Plat,
    /// Walking into a `trigger_teleport`: the engine warps you to the destination. No special
    /// traversal — the bot just routes onto an entrance cell and is teleported; it then detects
    /// the jump and re-paths from where it lands.
    Teleport,
    /// A grappling-hook swing: throw the hook at an anchor, reel toward it to build speed, then
    /// **release mid-reel** so the resulting velocity flings the bot along a gravity parabola onto
    /// the target ledge. The vertical pull-up (release at the anchor, near-zero speed, drop
    /// straight down) is just the degenerate case. The per-link [`HookTraversal`] (stored in a side
    /// table, see [`NavGraph::hook_of_link`]) carries the anchor and the release distance the bot
    /// needs to reproduce the arc. Only emitted when the map hands out the hook (`rtx_grapple`).
    Hook,
    /// A **rocket jump**: the bot jumps, and at a solved moment in the rise fires a rocket at the
    /// floor below it; the blast knockback flings it up onto a ledge no jump reaches. The two solved
    /// ingredients — the delay from jump to fire, and the fire direction — plus the self-damage live
    /// in the [`RocketJumpTraversal`] side table ([`NavGraph::rocket_jump_of_link`]). Costs the bot
    /// health, so it's planned only when it clearly beats the detour (a big cost surcharge) and a bot
    /// unfit to fly it (no RL / rocket / health) prices it away. Only emitted when `rtx_bot_rocketjump`.
    RocketJump,
}

/// A directed edge between two cells, with its traversal kind and travel-time cost.
#[derive(Clone, Copy)]
pub struct Link {
    pub from: CellId,
    pub to: CellId,
    pub kind: LinkKind,
    pub cost: f32,
}

/// The built navigation graph: cells, directed links, per-cell adjacency (indices into
/// `links`), and an XY spatial index for `nearest`/neighbor queries.
pub struct NavGraph {
    pub cells: Vec<Cell>,
    pub links: Vec<Link>,
    pub adjacency: Vec<Vec<u32>>,
    /// Per-cell "the standing origin is under water" flag (parallel to `cells`), so the planner can
    /// price swimming above walking and the runtime can tell a wet cell from a dry one. Empty until
    /// [`surcharge_water_links`](Self::surcharge_water_links) runs at graph-swap (it needs the
    /// engine's liquid-carrying `pointcontents`, unavailable on the pure worker build); an empty vec
    /// reads as "all dry" via [`cell_in_water`](Self::cell_in_water).
    water: Vec<bool>,
    /// Per-cell "a bot standing here can breathe" flag (parallel to `cells`): its eye point is out of
    /// the water, so it's a spot a drowning bot can path to for air. Filled alongside `water`; an
    /// empty vec reads as "all breathable" via [`cell_breathable`](Self::cell_breathable) — the safe
    /// default for an unmarked (dry) graph.
    breathable: Vec<bool>,
    grid: GridIndex,
    /// The closed door/movewall each link's segment passes through — the "navmesh aware of dynamic
    /// geometry" core, so pathfinding can price a link by its door's live state (see
    /// [`find_path`](Self::find_path)). Empty until [`add_gates`](Self::add_gates) runs.
    gates: SideTable<Gate>,
    /// How to fly each [`LinkKind::Hook`] link (its solved [`HookTraversal`]).
    hooks: SideTable<HookTraversal>,
    /// The takeoff point + required entry speed for each speed-jump link, for the bot executor.
    speed_jumps: SideTable<SpeedJumpTraversal>,
    /// The fire delay + angles + self-damage for each rocket-jump link, for the bot executor.
    rocket_jumps: SideTable<RocketJumpTraversal>,
    /// Spliced `func_plat` lifts (entity id + footprint), tagged onto the ride link and every
    /// jump-aboard link that boards each plat, so the runtime can find which lift a leg boards and
    /// hold a standoff while it's raised.
    plats: SideTable<Plat>,
    /// The bhop speed-gain constant `k` for this map's movement cvars ([`bhop_k`]), captured when
    /// speed jumps are spliced (or left at the stock default). The banded planner reads it to price
    /// speed carried between links; keeping it here avoids re-reading cvars at query time.
    sj_k: f32,
}

/// A solved speed jump: where the takeoff ledge is and the horizontal speed needed there, so the
/// runtime can refuse to leap if the bot somehow reaches the edge too slow to clear the gap.
#[derive(Clone, Copy)]
pub struct SpeedJumpTraversal {
    pub takeoff: Vec3,
    pub v_req: f32,
    /// Flight time of the leap (s), so the banded planner can price a hot entry (the runway term
    /// shrinks with carried speed while the airtime is fixed).
    pub airtime: f32,
    /// A **chained** speed jump: it has no self-contained runway (the `from` cell *is* the ledge),
    /// so it is only traversable when the planner proves the entry band already carries `v_req`
    /// (see [`NavGraph::find_path_banded`]). Unbanded queries price it away via [`NavGraph::link_extra`].
    pub chained: bool,
    /// Air-curl gain for a jump whose run-up and leap are **not collinear** (a curl jump — the bot runs
    /// down one corridor and turns in the air onto an offset landing). `0` = a straight speed jump: the
    /// runtime flies it with the hop slalom, unchanged. `> 0` = curl: once airborne the controller homes
    /// the velocity onto the landing with [`air_correct`](crate) at this rate — one smooth pursuit arc.
    pub curl_gain: f32,
}

/// Extra travel-time cost charged to a link whose gate is currently shut. Large enough that the
/// planner routes around a closed door whenever any open way exists, but finite so it still
/// crosses (and the bot then detours to the button) when there's no alternative — matching how a
/// game engine prices a disabled-but-openable NavMesh link rather than deleting it outright.
const CLOSED_GATE_PENALTY: f32 = 100_000.0;

/// Extra travel-time charged to every [`LinkKind::RocketJump`] link when the querying bot is unfit
/// to fly one (no rocket launcher, no rocket, too little health, or quad running — see
/// the game's `bot::rj::rocket_jump_extra`). Same magnitude as [`CLOSED_GATE_PENALTY`]: the planner
/// diverts around rocket jumps it can't make, yet — being finite — still takes one as a last resort
/// down a sole corridor rather than treating the graph as severed.
pub const RJ_UNFIT_PENALTY: f32 = 100_000.0;

/// Peak fraction of a link's own cost added as deterministic per-caller jitter when
/// [`LinkCosts::jitter_seed`] is set — enough to break ties between near-equal routes (so two bots
/// vary their paths) without ever reordering genuinely-cheaper alternatives.
const JITTER_FRAC: f32 = 0.10;

/// Per-query dynamic costs layered on each link's static cost: live gate state, plus an optional
/// caller-supplied surcharge (a bot's recently-failed links) and deterministic jitter (per-bot
/// route variety). **Every term is non-negative**, so A*'s straight-line heuristic stays an
/// admissible lower bound and routes stay optimal-or-diverted, never wrong. Cheap to pass by value.
#[derive(Default, Clone, Copy)]
pub struct LinkCosts<'a> {
    /// `gate_closed[i]` marks gate `i`'s door currently shut; a link through it is charged
    /// [`CLOSED_GATE_PENALTY`]. Empty slice ⇒ every door treated as open.
    pub gate_closed: &'a [bool],
    /// `(link idx, extra seconds)` surcharges — a bot's failed-link penalties. Tiny (≤8 entries),
    /// scanned linearly. Kept far below [`CLOSED_GATE_PENALTY`] so it diverts a route without ever
    /// forcing one through a shut door.
    pub penalties: &'a [(u32, f32)],
    /// Nonzero ⇒ add `hash(seed ^ link) → [0, JITTER_FRAC]·link.cost` per link, so bots with distinct
    /// seeds pick different near-equal corridors. Zero ⇒ no jitter (deterministic; tests, non-bots).
    pub jitter_seed: u32,
    /// Nonzero ⇒ charge every [`LinkKind::RocketJump`] link this many extra seconds — the per-bot
    /// capability gate. A bot currently unable to rocket-jump sets it to [`RJ_UNFIT_PENALTY`] so it
    /// plans around them; `0` (the default) leaves rocket jumps at their solved cost. The only price
    /// term that depends on *who* is asking (the others are world state), because unlike the grapple
    /// — a server-wide cvar — a bot's rockets and health vary moment to moment.
    pub rocket_jump_extra: f32,
}

/// A cheap integer hash (variant of the SplitMix/Murmur finalizer) for deterministic route jitter.
#[inline]
fn hash32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

impl NavGraph {
    /// Build the graph from a parsed BSP's player hull. Pure; safe to run at load time.
    pub fn build(bsp: &Bsp) -> NavGraph {
        let cells_grid = Self::carve_cells(bsp);
        let mut graph = NavGraph {
            adjacency: vec![Vec::new(); cells_grid.0.len()],
            cells: cells_grid.0,
            links: Vec::new(),
            water: Vec::new(),      // filled at graph-swap by surcharge_water_links
            breathable: Vec::new(), // (needs the engine's liquid-carrying pointcontents)
            grid: cells_grid.1,
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED), // stock default until add_speed_jumps captures live cvars
        };
        graph.link_cells(bsp);
        graph
    }

    /// Sweep every grid column for floors and emit one [`Cell`] at the bottom of each empty
    /// span (a surface the player can rest on). Returns the cells plus their XY spatial index.
    fn carve_cells(bsp: &Bsp) -> (Vec<Cell>, GridIndex) {
        let (gx0, gy0) = (floor_grid(bsp.mins.x), floor_grid(bsp.mins.y));
        let (gx1, gy1) = (floor_grid(bsp.maxs.x), floor_grid(bsp.maxs.y));

        // Scan columns for floors in parallel — the column sweep is a big share of the build and the
        // columns are independent geometry. One `(gx, gy, origin_z)` row per `gx`; the indexed
        // `collect` keeps rows in `gx` order (a `RangeInclusive<i32>` is an indexed parallel
        // iterator) and `gy` is sequential within a row, so the serial ID-assignment pass below
        // reproduces the single-threaded cell order — cell IDs and the grid index come out identical.
        let rows: Vec<Vec<(i32, i32, f32)>> = (gx0..=gx1)
            .into_par_iter()
            .map(|gx| {
                let mut row = Vec::new();
                for gy in gy0..=gy1 {
                    let (x, y) = (gx as f32 * GRID, gy as f32 * GRID);
                    Self::column_floors(bsp, x, y, |origin_z| row.push((gx, gy, origin_z)));
                }
                row
            })
            .collect();

        let mut cells = Vec::new();
        let mut grid: GridIndex = HashMap::new();
        for (gx, gy, origin_z) in rows.into_iter().flatten() {
            let id = cells.len() as CellId;
            cells.push(Cell {
                origin: Vec3::new(gx as f32 * GRID, gy as f32 * GRID, origin_z),
                gx,
                gy,
            });
            grid.entry((gx, gy)).or_default().push(id);
        }
        (cells, grid)
    }

    /// Scan one column bottom-to-top; for each solid→empty transition (a floor with headroom
    /// for the standing hull) call `emit(origin_z)` with the bisected resting origin height.
    fn column_floors(bsp: &Bsp, x: f32, y: f32, mut emit: impl FnMut(f32)) {
        let at = |z: f32| bsp.is_solid(Vec3::new(x, y, z));
        let mut z = bsp.mins.z;
        let mut prev_solid = true; // below the world is solid
        while z <= bsp.maxs.z {
            let solid = at(z);
            if prev_solid && !solid {
                // Resting origin is between the last solid sample and this empty one.
                emit(bisect_floor(bsp, x, y, z - SCAN_DZ, z));
            }
            prev_solid = solid;
            z += SCAN_DZ;
        }
    }

    /// Classify the moves out of every cell into directed links: grounded moves to the 8
    /// grid-adjacent columns, then jumps across gaps / up to ledges (windowed and deduped).
    fn link_cells(&mut self, bsp: &Bsp) {
        // Classify per cell in parallel (`classify_grounded`/`find_jumps` are read-only), collecting
        // each cell's links grounded-then-jumps, then splice serially. Indexed `collect` keeps cell
        // order and the per-cell order is preserved, so link indices are identical to a serial build.
        let this = &*self;
        let per_cell: Vec<Vec<Link>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let c = this.cells[from as usize];
                let mut out = Vec::new();
                for to in this.neighbors_within(c.gx, c.gy, 1) {
                    if to != from {
                        if let Some(link) = this.classify_grounded(bsp, from, to) {
                            out.push(link);
                        }
                    }
                }
                out.extend(this.find_jumps(bsp, from));
                out
            })
            .collect();
        for link in per_cell.into_iter().flatten() {
            self.push_link(link);
        }
    }

    fn push_link(&mut self, link: Link) {
        let idx = self.links.len() as u32;
        self.adjacency[link.from as usize].push(idx);
        self.links.push(link);
    }

    /// A grounded move (walk/step/drop) to a grid-adjacent cell, if the path is clear. An
    /// adjacent rise between step height and a standing jump's apex — a knee-high ledge or a
    /// slope too steep for a free step — is a short **hop up** (a `JumpGap`): pmove needs a jump
    /// to mount it, but it's basic movement (all modes), not a gap leap. Taller rises need the
    /// windowed ledge jumps in [`find_jumps`].
    fn classify_grounded(&self, bsp: &Bsp, from: CellId, to: CellId) -> Option<Link> {
        let (a, b) = (self.cells[from as usize], self.cells[to as usize]);
        let dz = b.origin.z - a.origin.z;
        if dz > STEP_HEIGHT && dz <= JUMP_APEX {
            // Hop up onto the adjacent higher footing; clear the standing-jump arc to it. Not from a
            // submerged cell, though — you can't jump when submerged (the jump input swims up).
            if bsp.is_liquid_at(a.origin) {
                return None;
            }
            return arc_clear(bsp, a.origin, b.origin).then(|| {
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                Link {
                    from,
                    to,
                    kind: LinkKind::JumpGap,
                    cost: link_cost(LinkKind::JumpGap, horiz, dz),
                }
            });
        }
        let kind = if dz.abs() <= WALK_DZ {
            LinkKind::Walk
        } else if dz.abs() <= STEP_HEIGHT {
            LinkKind::Step
        } else if (-MAX_DROP..-STEP_HEIGHT).contains(&dz) {
            // A drop is only real off a **ledge**: the column stepped toward the target must not
            // have walkable ground at our height (mirrors the ledge check in `find_jumps`). Without
            // this, a cell in the middle of a floor sprouts phantom drops onto lower cells beneath
            // it — `path_clear` only samples the top corridor, so it never sees the solid slab in
            // between (worst on thin floors, where a lower room sits directly under every cell). A
            // bot handed such a link freezes trying to "drop" straight through solid ground.
            let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                return None;
            }
            // And the hull must actually fit down into the target — not a slot too small for it.
            if !descent_clear(bsp, a.origin.z, b.origin) {
                return None;
            }
            LinkKind::Drop
        } else {
            return None; // up beyond a jump's apex — needs the windowed ledge jumps
        };
        if !path_clear(bsp, a.origin, b.origin) {
            return None;
        }
        // `path_clear` only rules out walls/ceilings along the head-height corridor; it can't see an
        // air gap *under* the segment. For a Walk/Step to a grid-diagonal neighbor around an L-shaped
        // ledge (a stair side, a balcony corner), the centre-to-centre line clips the corner's air —
        // the bot walks straight off. Require floor to continue beneath the whole segment. (Drop keeps
        // its own `has_ground_near` + `descent_clear`; JumpGap flew the `arc_clear` branch above.)
        if matches!(kind, LinkKind::Walk | LinkKind::Step) && !ground_along(&|p| bsp.is_solid(p), a.origin, b.origin) {
            return None;
        }
        let horiz = (b.origin.xy() - a.origin.xy()).length();
        Some(Link {
            from,
            to,
            kind,
            cost: link_cost(kind, horiz, dz),
        })
    }

    /// A cell in grid column `(gx, gy)` within `STEP_HEIGHT` of height `z`, if any.
    fn cell_near(&self, gx: i32, gy: i32, z: f32) -> Option<CellId> {
        self.grid.get(&(gx, gy)).and_then(|ids| {
            ids.iter()
                .copied()
                .find(|&id| (self.cells[id as usize].origin.z - z).abs() <= STEP_HEIGHT)
        })
    }

    /// Whether grid column `(gx, gy)` has a cell within `STEP_HEIGHT` of height `z` — i.e.
    /// walkable ground continues there (so a jump isn't warranted).
    fn has_ground_near(&self, gx: i32, gy: i32, z: f32) -> bool {
        self.grid.get(&(gx, gy)).is_some_and(|ids| {
            ids.iter()
                .any(|&id| (self.cells[id as usize].origin.z - z).abs() <= STEP_HEIGHT)
        })
    }

    /// Cell ids in grid columns within Chebyshev `radius` of `(gx, gy)`.
    fn neighbors_within(&self, gx: i32, gy: i32, radius: i32) -> Vec<CellId> {
        let mut out = Vec::new();
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                if let Some(ids) = self.grid.get(&(gx + dx, gy + dy)) {
                    out.extend_from_slice(ids);
                }
            }
        }
        out
    }

    /// Surcharge links that route a bot alongside a lava or slime pool, so the planner keeps its
    /// distance whenever a comparable safe route exists. For each cell we probe its *open* compass
    /// sides — those with no walkable neighbour, i.e. an edge — a stride out and down; a liquid
    /// below flags the cell, and every link *entering* a flagged cell pays [`HAZARD_LINK_EXTRA`].
    ///
    /// Pits are deliberately not flagged here (every balcony/ledge cell borders a drop — that would
    /// surcharge half the map); the runtime combat guard [`crate::hazard::hazard_ahead`] keeps bots
    /// from stepping off edges in a fight. Only *liquids* get a routing bias, and the pure
    /// worker-thread build can't see them — the clip hull it reads carries no liquid contents — so
    /// this takes the engine's `pointcontents` as `contents` and runs at graph-swap time (from the
    /// game's navmesh poll) rather than inside [`build_navmesh`].
    pub fn surcharge_hazard_links(&mut self, is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> f32) {
        let hazard: Vec<bool> = (0..self.cells.len() as CellId)
            .map(|id| self.cell_on_liquid_edge(id, is_solid, contents))
            .collect();
        for link in &mut self.links {
            if hazard[link.to as usize] {
                link.cost += HAZARD_LINK_EXTRA;
            }
        }
    }

    /// Whether cell `id` sits on the edge of a lava or slime pool: for each compass direction with
    /// no walkable neighbour (an open side), probe a stride out at foot height and check for a
    /// liquid below. A drop to lower ground, plain water, or a wall all read as safe.
    fn cell_on_liquid_edge(
        &self,
        id: CellId,
        is_solid: &impl Fn(Vec3) -> bool,
        contents: &impl Fn(Vec3) -> f32,
    ) -> bool {
        let c = self.cells[id as usize];
        // Standing player feet sit 24u below the origin (player `mins.z`); probe from there.
        let feet = c.origin - Vec3::new(0.0, 0.0, 24.0);
        let step = |v: f32| if v > 0.1 { 1 } else if v < -0.1 { -1 } else { 0 };
        crate::hazard::HAZARD_DIRS.iter().any(|&(dx, dy)| {
            if self.has_ground_near(c.gx + step(dx), c.gy + step(dy), c.origin.z) {
                return false; // walkable ground continues this way — not an edge
            }
            let p = feet + Vec3::new(dx, dy, 0.0) * HAZARD_PROBE_R + Vec3::new(0.0, 0.0, 8.0);
            matches!(
                crate::hazard::hazard_below(is_solid, contents, p),
                Some(crate::hazard::HazardKind::Lava | crate::hazard::HazardKind::Slime)
            )
        })
    }

    /// Flag every cell whose standing origin is under water, and price swimming above walking. Fills
    /// the parallel `water`/`breathable` vectors — an origin in water means a bot swims here; an eye
    /// point out of the water (`origin + 22`, pmove's waterlevel-3 sample) means a spot it can
    /// breathe — then multiplies the cost of every link *entering* a water cell by
    /// [`WATER_COST_MULT`]. Exit links (water → dry) are left untouched, so the surcharge forms a
    /// cost gradient the planner follows back to shore rather than a uniform pool tax.
    ///
    /// Like [`surcharge_hazard_links`](Self::surcharge_hazard_links) this reads liquid contents, which
    /// only the engine's render-hull `pointcontents` carries (the worker build's clip hull is
    /// liquid-blind), so it runs at graph-swap from the game's navmesh poll — not inside [`build`].
    pub fn surcharge_water_links(&mut self, contents: &impl Fn(Vec3) -> f32) {
        let is_water = |p: Vec3| contents(p) == crate::bsp::CONTENTS_WATER as f32;
        self.water = self.cells.iter().map(|c| is_water(c.origin)).collect();
        // Eye height for the breathe test: the standing view offset (pmove samples waterlevel 3 here).
        let eye = Vec3::new(0.0, 0.0, 22.0);
        self.breathable = self.cells.iter().map(|c| !is_water(c.origin + eye)).collect();
        for link in &mut self.links {
            if self.water[link.to as usize] {
                link.cost *= WATER_COST_MULT;
            }
        }
    }

    /// Extra A* cost for link `li` under `costs`: closed-gate penalty + this caller's per-link
    /// surcharge + optional deterministic jitter. All non-negative, keeping the A* heuristic
    /// admissible (see [`LinkCosts`]).
    #[inline]
    fn link_extra(&self, li: u32, costs: &LinkCosts) -> f32 {
        let mut extra = match self.gate_of_link(li) {
            Some(g) if costs.gate_closed.get(g).copied().unwrap_or(false) => CLOSED_GATE_PENALTY,
            _ => 0.0,
        };
        for &(l, sec) in costs.penalties {
            if l == li {
                extra += sec;
                break;
            }
        }
        if costs.jitter_seed != 0 {
            let h = hash32(costs.jitter_seed ^ li.wrapping_mul(0x9e37_79b1));
            extra += (h as f32 / u32::MAX as f32) * JITTER_FRAC * self.links[li as usize].cost;
        }
        if costs.rocket_jump_extra > 0.0 && self.links[li as usize].kind == LinkKind::RocketJump {
            extra += costs.rocket_jump_extra;
        }
        extra
    }

    /// The block a *speed-unaware* query (`find_path`, `costs_from`) must add to a chained speed
    /// jump: it has no self-contained runway, so a route that doesn't reason about carried speed can
    /// never take one. Large enough to sever it in practice, finite so it never poisons `g` beyond
    /// the existing [`CLOSED_GATE_PENALTY`] scale. The banded planner ([`Self::find_path_banded`])
    /// bypasses this and gates chained jumps on the entry band instead.
    pub(super) fn chained_block(&self, li: u32) -> f32 {
        match self.speed_jump_of_link(li) {
            Some(t) if t.chained => CLOSED_GATE_PENALTY,
            _ => 0.0,
        }
    }

    /// The banded transition for link `li` entered at speed band `entry`: its travel-time cost and
    /// the band the bot arrives in, or `None` if the leg is infeasible at this entry speed (a
    /// chained speed jump the carried speed can't satisfy). Conservative by construction — speeds
    /// are band *floors*, gains are derated ([`BHOP_EFF`]), and no leg demotes a carried band except
    /// where physics forces it (a hard fall, or a teleport/plat/hook/rocket that resets speed).
    /// Every cost is floored at `horiz / BAND_V_MAX`, keeping the banded heuristic admissible.
    pub fn banded_step(&self, li: u32, entry: u8) -> Option<(f32, u8)> {
        let link = self.links[li as usize];
        let from = self.cells[link.from as usize].origin;
        let to = self.cells[link.to as usize].origin;
        let horiz = (to.xy() - from.xy()).length();
        let dz = to.z - from.z;
        let floor_cost = horiz / BAND_V_MAX;
        let v_in = BAND_FLOOR[entry as usize].max(MAX_SPEED);
        Some(match link.kind {
            LinkKind::Walk | LinkKind::Step => {
                // Already moving (band ≥ 1): carry speed and climb. From a standstill spend a spin-up
                // runway before gains begin. But an ascending leg — a stair riser (`dz > WALK_DZ`) —
                // builds no bhop speed (a human runs up stairs), so it carries the band without gain
                // and without demotion (a lone step must not zero a chain's plan; the runtime runway
                // and carry gates handle real staircases). `.max(v_in)` never demotes on a short leg.
                let v_out = if dz > WALK_DZ {
                    v_in
                } else {
                    let usable = if entry >= 1 { horiz } else { (horiz - BAND_SPINUP).max(0.0) };
                    v_in.max(BHOP_EFF * attainable_speed(v_in, usable, self.sj_k))
                };
                let avg = ((v_in + v_out) * 0.5).max(MAX_SPEED);
                ((horiz / avg).max(floor_cost), band_of(v_out))
            }
            LinkKind::SpeedJump => {
                let (v_req, airtime, chained, curl_gain) = self
                    .speed_jump_of_link(li)
                    .map(|t| (t.v_req, t.airtime, t.chained, t.curl_gain))
                    .unwrap_or((MAX_SPEED, 0.0, false, 0.0));
                // A curl speed jump was certified end-to-end at build time — the rollout solver measured
                // a run-up that the ground circle-strafe genuinely delivers (which the conservative
                // air-strafe recompute below has no term for and badly under-credits). So trust its
                // stored cost. Exit at the certified takeoff *floor* (`v_req`), NOT the carried entry: the
                // takeoff regime grounds the whole run-up, so friction caps the takeoff at the prestrafe
                // equilibrium regardless of how fast the bot arrived — crediting `v_in` would plan a
                // downstream chained jump off a band the runtime can't actually carry through the curl.
                if curl_gain > 0.0 && !chained {
                    return Some((link.cost.max(floor_cost), band_of(v_req)));
                }
                // A chained jump has no runway: traversable only if the entry band already carries it.
                if chained && v_in < v_req * SJ_MARGIN {
                    return None;
                }
                // The runway run-up shrinks with carried speed (0 once at v_req); airtime is fixed.
                let runway_t = runway_time(v_req * SJ_MARGIN, v_in, self.sj_k);
                // Horizontal speed is conserved through the leap: the bot lands carrying whatever it
                // took off with — the carried entry (chained) or the runway-built v_req (stand start),
                // whichever is greater. So a chain of jumps sustains its band instead of decaying.
                let v_exit = v_in.max(v_req * SJ_MARGIN);
                ((runway_t + airtime + 1.0).max(floor_cost), band_of(v_exit))
            }
            // A hard fall stumbles to a standstill; a short drop keeps the band.
            LinkKind::Drop => (link.cost, if -dz <= SAFE_FALL { entry } else { 0 }),
            LinkKind::JumpGap => (link.cost, entry),
            LinkKind::DoubleJump => (link.cost, entry.saturating_sub(1)),
            // Teleport / plat ride / hook / rocket jump all deliver the bot at a standstill.
            LinkKind::Teleport | LinkKind::Plat | LinkKind::Hook | LinkKind::RocketJump => (link.cost, 0),
        })
    }

    // --- entity-independent: grappling-hook swing links ---

    /// Splice grappling-hook swing links into the graph. From each **ledge-edge** cell, fire probe
    /// rays out over the drop-off at a few pitches, find where the hook would anchor, then sample
    /// release points along the reel and simulate the resulting gravity parabola — whatever standable
    /// cell the arc lands on becomes the link's target. This discovers both vertical pull-ups and
    /// long horizontal flings from one mechanism. Only accepted when the arc (and perturbed variants)
    /// land safely, so a bot is never flung into a pit. Deduped per direction/elevation and capped.
    pub fn add_hooks(&mut self, bsp: &Bsp, params: HookParams) {
        // Solve per source cell in parallel (immutable borrow), then splice serially (push_hook needs
        // `&mut`). Indexed `collect` preserves cell order, so link indices match a sequential build.
        let this = &*self;
        let pending: Vec<Vec<(Link, HookTraversal)>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let mut out = Vec::new();
                this.solve_hooks_from(bsp, from, params, &mut out);
                out
            })
            .collect();
        for (link, tr) in pending.into_iter().flatten() {
            self.push_hook(link, tr);
        }
    }

    /// Solve the hook links leaving cell `from`, appending accepted `(Link, HookTraversal)` to `out`.
    fn solve_hooks_from(&self, bsp: &Bsp, from: CellId, params: HookParams, out: &mut Vec<(Link, HookTraversal)>) {
        let c = self.cells[from as usize];
        let a = c.origin;
        let launch = a + Vec3::new(0.0, 0.0, HOOK_LAUNCH_Z);
        // best per (compass octant, 128u elevation band): (cost, link, traversal)
        let mut best: HashMap<(usize, i32), (f32, Link, HookTraversal)> = HashMap::new();

        for (dgx, dgy) in COMPASS {
            // Only launch out over a ledge/gap in this direction — hooking toward continuing ground
            // is pointless (walk/step/jump already cover it).
            if self.has_ground_near(c.gx + dgx, c.gy + dgy, a.z) {
                continue;
            }
            let yaw = (dgy as f32).atan2(dgx as f32);
            for pitch_deg in HOOK_PITCHES {
                let pitch = pitch_deg.to_radians();
                let (sp, cp) = pitch.sin_cos();
                let (sy, cy) = yaw.sin_cos();
                let dir = Vec3::new(cp * cy, cp * sy, sp);
                let Some(stick) = march_to_solid(|p| bsp.is_solid(p), launch, dir, HOOK_ROPE_MAX) else {
                    continue;
                };
                let rope = (stick - launch).length();
                if rope < HOOK_SAMPLE {
                    continue;
                }
                let v0 = dir * params.pull;
                let rdir = v0.normalize_or_zero();
                // Sample release points along the reel by their distance from the anchor. Everything
                // keys off `stick` + `release_dist` (the reconstructable form the runtime re-solve and
                // the test use), so the stored arc reproduces bit-for-bit — no fp drift on a grazing
                // sample. Leave some rope reeled and some swing left.
                let mut release_dist = HOOK_SAMPLE;
                while release_dist < rope - HOOK_SAMPLE {
                    let r = stick - rdir * release_dist;
                    if let Some((land, airtime, vz)) = arc_land(bsp, r, v0, params.gravity) {
                        if let Some(to) = self.nearest_within(land, HOOK_LAND_XY, HOOK_LAND_Z) {
                            if to != from {
                                let b = self.cells[to as usize].origin;
                                let dz = b.z - a.z;
                                let horiz = (b.xy() - a.xy()).length();
                                let useful = dz > JUMP_APEX || horiz > JUMP_REACH;
                                let in_range = (HOOK_MIN_RISE..=HOOK_MAX_RISE).contains(&dz) && horiz <= HOOK_RANGE_XY;
                                if useful
                                    && in_range
                                    && !self.has_direct_link(from, to)
                                    && perturb_ok(bsp, stick, rdir, release_dist, rope, params, b)
                                {
                                    let cost = hook_cost(rope, release_dist, airtime, vz, params);
                                    let key = (dir_bucket(dgx, dgy), (dz / 128.0).floor() as i32);
                                    if best.get(&key).is_none_or(|(bc, _, _)| cost < *bc) {
                                        let link = Link {
                                            from,
                                            to,
                                            kind: LinkKind::Hook,
                                            cost,
                                        };
                                        let tr = HookTraversal {
                                            stick,
                                            release_dist,
                                            v0,
                                            airtime,
                                        };
                                        best.insert(key, (cost, link, tr));
                                    }
                                }
                            }
                        }
                    }
                    release_dist += HOOK_R_STEP;
                }
            }
        }

        // Keep the cheapest few per cell. Break cost ties by target cell then dedup key, so the
        // survivors don't depend on `HashMap` iteration order (randomized per instance — and under
        // parallel building a tie would otherwise resolve differently run to run).
        let mut chosen: Vec<_> = best.into_iter().collect();
        chosen.sort_by(|(ak, (ac, al, _)), (bk, (bc, bl, _))| {
            ac.total_cmp(bc).then(al.to.cmp(&bl.to)).then(ak.cmp(bk))
        });
        chosen.truncate(HOOK_MAX_PER_CELL);
        out.extend(chosen.into_iter().map(|(_, (_, link, tr))| (link, tr)));
    }

    /// Splice rocket-jump links: for each cell, fire a rocket at a solved delay/angle during a jump
    /// and keep the launches that land on a higher ledge no cheaper move reaches. `double_jump` gates
    /// the useful height. See [`super::rocketjump`] for the two-phase ballistics.
    pub fn add_rocket_jumps(&mut self, bsp: &Bsp, params: RocketJumpParams, double_jump: bool) {
        // Solve per source cell in parallel (immutable borrow), then splice serially (push needs
        // `&mut`). Indexed `collect` preserves cell order, so link indices match a sequential build.
        let this = &*self;
        let pending: Vec<Vec<(Link, RocketJumpTraversal)>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let mut out = Vec::new();
                this.solve_rocket_jumps_from(bsp, from, params, double_jump, &mut out);
                out
            })
            .collect();
        for (link, tr) in pending.into_iter().flatten() {
            self.push_rocket_jump(link, tr);
        }
    }

    /// Solve the rocket-jump links leaving cell `from`, appending accepted `(Link, RocketJumpTraversal)`
    /// to `out`. Unlike hooks there's no ledge-edge skip — the classic RJ launches from flat ground up
    /// a wall face — so all eight travel octants are tried, firing opposite the travel direction.
    fn solve_rocket_jumps_from(
        &self,
        bsp: &Bsp,
        from: CellId,
        params: RocketJumpParams,
        double_jump: bool,
        out: &mut Vec<(Link, RocketJumpTraversal)>,
    ) {
        let a = self.cells[from as usize].origin;
        if bsp.is_liquid_at(a) {
            return; // submerged takeoff: can't jump to start the rocket jump (the jump input swims up)
        }
        let is_solid = |p: Vec3| bsp.is_solid(p);
        // The rocket is a zero-size point, so it collides on the render hull (hull 0) and reaches the
        // true floor/wall — ~24u below the inflated player-hull surface `is_solid` reports. The solve
        // detonates the shot on this hull so the blast geometry matches what the engine produces.
        let rocket_solid = |p: Vec3| bsp.is_point_solid(p);
        // Height an RJ must clear to earn its health cost: past a plain (or double) jump's apex.
        let useful_apex = if double_jump { DOUBLE_JUMP_APEX } else { JUMP_APEX };
        // best per (compass octant, 128u elevation band): (cost, link, traversal)
        let mut best: HashMap<(usize, i32), (f32, Link, RocketJumpTraversal)> = HashMap::new();

        for (dgx, dgy) in COMPASS {
            // Fire opposite the travel direction: the blast lands behind-and-below, shoving the bot
            // up and toward the ledge.
            let fire_yaw = (dgy as f32).atan2(dgx as f32).to_degrees() + 180.0;
            for pitch in RJ_PITCHES {
                for delay in RJ_DELAYS {
                    let angles = Vec3::new(pitch, fire_yaw, 0.0);
                    let Some(s) = simulate_rocket_jump(is_solid, rocket_solid, a, angles, delay, params) else {
                        continue;
                    };
                    let Some(to) = self.nearest_within(s.land, RJ_LAND_XY, RJ_LAND_Z) else {
                        continue;
                    };
                    if to == from {
                        continue;
                    }
                    let b = self.cells[to as usize].origin;
                    let dz = b.z - a.z;
                    let horiz = (b.xy() - a.xy()).length();
                    let useful = dz > useful_apex; // height is the whole point of an RJ in v1
                    let in_range = (RJ_MIN_RISE..=RJ_MAX_RISE).contains(&dz) && horiz <= RJ_RANGE_XY;
                    if useful
                        && in_range
                        && !self.has_direct_link(from, to)
                        && rj_perturb_ok(is_solid, rocket_solid, a, angles, delay, params, b)
                    {
                        let cost = rocket_jump_cost(s.t_blast, s.airtime, s.vz_land, s.self_damage);
                        let key = (dir_bucket(dgx, dgy), (dz / 128.0).floor() as i32);
                        if best.get(&key).is_none_or(|(bc, _, _)| cost < *bc) {
                            let link = Link { from, to, kind: LinkKind::RocketJump, cost };
                            let tr = RocketJumpTraversal {
                                fire_angles: angles,
                                fire_delay: delay,
                                blast: s.blast,
                                pos_blast: s.pos_blast,
                                v0: s.v0,
                                land: s.land,
                                airtime: s.airtime,
                                self_damage: s.self_damage,
                            };
                            best.insert(key, (cost, link, tr));
                        }
                    }
                }
            }
        }

        // Keep the cheapest few per cell. Break cost ties by target cell then dedup key, so the
        // survivors don't depend on `HashMap` iteration order (randomized per instance — and under
        // parallel building a tie would otherwise resolve differently run to run).
        let mut chosen: Vec<_> = best.into_iter().collect();
        chosen.sort_by(|(ak, (ac, al, _)), (bk, (bc, bl, _))| {
            ac.total_cmp(bc).then(al.to.cmp(&bl.to)).then(ak.cmp(bk))
        });
        chosen.truncate(RJ_MAX_PER_CELL);
        out.extend(chosen.into_iter().map(|(_, (_, link, tr))| (link, tr)));
    }

    /// Whether `from` already has a direct (non-hook) link to `to` — such a target needs no hook.
    fn has_direct_link(&self, from: CellId, to: CellId) -> bool {
        self.adjacency[from as usize]
            .iter()
            .any(|&li| self.links[li as usize].to == to)
    }

    /// The solved traversal for hook link `li`, or `None` for a non-hook link.
    pub fn hook_of_link(&self, li: u32) -> Option<&HookTraversal> {
        self.hooks.of_link(li)
    }

    /// Push a hook link with its solved traversal, tagging the new link in the `hooks` side table.
    fn push_hook(&mut self, link: Link, traversal: HookTraversal) {
        let h = self.hooks.push(traversal);
        self.push_link(link);
        self.hooks.tag(self.links.len() - 1, h);
    }

    /// The solved traversal for speed-jump link `li`, or `None` for any other link.
    pub fn speed_jump_of_link(&self, li: u32) -> Option<&SpeedJumpTraversal> {
        self.speed_jumps.of_link(li)
    }

    /// Hand-plant a speed-jump link post-build (harness / bring-up): inject a `SpeedJump` from `from`
    /// to `to` with the given cost and solved traversal, returning its link index. Lets the control
    /// harness validate a takeoff/curl the generator doesn't yet emit — the runtime flies a planted
    /// link exactly like a generated one. Not used by the automatic build.
    pub fn plant_speed_jump(&mut self, from: CellId, to: CellId, cost: f32, traversal: SpeedJumpTraversal) -> u32 {
        self.push_speed_jump(Link { from, to, kind: LinkKind::SpeedJump, cost }, traversal);
        (self.links.len() - 1) as u32
    }

    /// Push a speed-jump link with its traversal, tagging the new link in the side table.
    fn push_speed_jump(&mut self, link: Link, traversal: SpeedJumpTraversal) {
        let s = self.speed_jumps.push(traversal);
        self.push_link(link);
        self.speed_jumps.tag(self.links.len() - 1, s);
    }

    /// The solved traversal for rocket-jump link `li`, or `None` for any other link.
    pub fn rocket_jump_of_link(&self, li: u32) -> Option<&RocketJumpTraversal> {
        self.rocket_jumps.of_link(li)
    }

    /// Push a rocket-jump link with its traversal, tagging the new link in the side table.
    fn push_rocket_jump(&mut self, link: Link, traversal: RocketJumpTraversal) {
        let r = self.rocket_jumps.push(traversal);
        self.push_link(link);
        self.rocket_jumps.tag(self.links.len() - 1, r);
    }

    /// Append a free-standing cell (not from the column carve) and index it. Used for plat
    /// surfaces, which don't exist in the static world hull.
    fn add_cell(&mut self, origin: Vec3) -> CellId {
        let id = self.cells.len() as CellId;
        let (gx, gy) = (floor_grid(origin.x), floor_grid(origin.y));
        self.cells.push(Cell { origin, gx, gy });
        self.adjacency.push(Vec::new());
        self.grid.entry((gx, gy)).or_default().push(id);
        id
    }

    /// Cells whose XY is within `radius` of `xy`.
    fn cells_near(&self, xy: Vec2, radius: f32) -> Vec<CellId> {
        let (gx, gy) = (floor_grid(xy.x), floor_grid(xy.y));
        let r = (radius / GRID).ceil() as i32;
        self.neighbors_within(gx, gy, r)
            .into_iter()
            .filter(|&c| (self.cells[c as usize].origin.xy() - xy).length() <= radius)
            .collect()
    }

    /// Nearest cell to `p` within `horiz` XY and `vert` Z of it, by 3D distance.
    fn nearest_within(&self, p: Vec3, horiz: f32, vert: f32) -> Option<CellId> {
        let (gx, gy) = (floor_grid(p.x), floor_grid(p.y));
        let r = (horiz / GRID).ceil() as i32;
        self.neighbors_within(gx, gy, r)
            .into_iter()
            .filter(|&c| {
                let o = self.cells[c as usize].origin;
                (o.xy() - p.xy()).length() <= horiz && (o.z - p.z).abs() <= vert
            })
            .min_by(|&a, &b| {
                let d = |c: CellId| (self.cells[c as usize].origin - p).length_squared();
                d(a).total_cmp(&d(b))
            })
    }
}

/// Live physics the hook-arc solver needs, gathered from cvars at build time (they can differ per
/// map: `sv_gravity` is 100 on e1m8, and `rtx_hook_pull`/`rtx_hook_speed` are tunable). Reeling
/// speed and gravity fix the parabola; the throw speed only prices the flight leg.
#[derive(Clone, Copy)]
pub struct HookParams {
    pub gravity: f32,
    pub pull: f32,  // HOOK_PULL_BASE × rtx_hook_pull
    pub throw: f32, // HOOK_THROW_BASE × rtx_hook_speed
}

/// Live physics the speed-jump solver needs: gravity (jump airtime) and the bhop acceleration
/// (`sv_accelerate`/`sv_maxspeed`) that converts a runway length into attainable speed.
#[derive(Clone, Copy)]
pub struct SpeedJumpParams {
    pub gravity: f32,
    pub accel: f32,
    pub maxspeed: f32,
    /// `sv_friction` / `sv_stopspeed` — needed by the curl certifier's ground-prestrafe rollout.
    pub friction: f32,
    pub stopspeed: f32,
    /// Generate curl jumps (run-up + air-turn onto an offset landing), certified by a pmove rollout.
    pub curl: bool,
}

/// A solved hook traversal, stored per hook link in a side table (parallel to `links`, like
/// `gated_links`). Carries what the bot can't cheaply re-derive: the anchor it should aim at, and
/// the distance-to-anchor at which to let go so the ensuing parabola lands on the target.
#[derive(Clone, Copy)]
pub struct HookTraversal {
    /// Where the hook is expected to stick (also the throw aim point).
    pub stick: Vec3,
    /// Distance from the anchor at which to release the reel (`0` = pull all the way in / drop).
    pub release_dist: f32,
    /// Release velocity used to solve the arc — the reel direction and speed at let-go. Read by the
    /// build/test to re-fly the stored arc (the runtime reels toward the live anchor instead).
    #[allow(dead_code)]
    pub v0: Vec3,
    /// Simulated airtime of the parabola — the runtime's Ballistic-phase watchdog base.
    pub airtime: f32,
}

/// Live physics the rocket-jump solver needs, gathered from cvars at build time: gravity (fixes both
/// the jump ascent and the post-blast parabola) and the `rj` self-boost cvar (off by default; when a
/// server sets it > 1, a self-rocket adds an extra `dir·points·rj` impulse — see `t_damage`).
#[derive(Clone, Copy)]
pub struct RocketJumpParams {
    pub gravity: f32,
    pub rj_extra: f32,
}

/// A solved rocket jump, stored per rocket-jump link in a side table (parallel to `links`, like
/// `hooks`). Carries the two ingredients the bot fires the shot by — the delay from the jump press
/// and the view angles — plus the self-damage (the runtime health gate) and the arc data.
///
/// The runtime driver (`crate::bot::rj`) reads `fire_angles`/`fire_delay`/`airtime`/`self_damage`;
/// `blast`/`pos_blast`/`v0` are build/test-only re-flight data.
#[derive(Clone, Copy)]
pub struct RocketJumpTraversal {
    /// View angles to fire at (QW pitch positive-down); the shot goes straight along `v_forward`.
    pub fire_angles: Vec3,
    /// Seconds from the jump press to the `+attack` that fires the rocket.
    pub fire_delay: f32,
    /// Where the rocket is expected to explode (telemetry / the runtime doesn't need it).
    #[allow(dead_code)]
    pub blast: Vec3,
    /// Bot position at the blast — stored so the build/test can re-fly the continuation arc.
    #[allow(dead_code)]
    pub pos_blast: Vec3,
    /// Continuation velocity just after the blast — re-flown by the build/test.
    #[allow(dead_code)]
    pub v0: Vec3,
    /// The solver's predicted landing position (where the post-blast arc touches down). Telemetry /
    /// test only — lets the harness tell a physics miss (runtime ≠ this) from an acceptance snap
    /// (this ≠ the target cell).
    #[allow(dead_code)]
    pub land: Vec3,
    /// Simulated airtime of the parabola after the blast — the runtime's Ballistic watchdog base.
    pub airtime: f32,
    /// Pre-armor self-damage points from the blast — the runtime's health gate.
    pub self_damage: f32,
}

#[derive(Default)]
pub struct LinkCounts {
    pub walk: u32,
    pub step: u32,
    pub drop: u32,
    pub jump: u32,
    pub double_jump: u32,
    pub speed_jump: u32,
    pub plat: u32,
    pub teleport: u32,
    pub hook: u32,
    pub rocket_jump: u32,
}

/// Radius out from a cell probed for an adjacent lava/slime surface — a stride (1.5 grid columns)
/// past the cell, so a bot skirting the edge is caught without flagging cells a safe walkway away.
const HAZARD_PROBE_R: f32 = 48.0;
/// Extra travel-time charged to every link *entering* a cell on a lava/slime edge (see
/// [`NavGraph::surcharge_hazard_links`]). Moderate — a few walk-links' worth — so the planner takes
/// a parallel safe corridor when one exists within a short detour, yet still crosses a lava bridge
/// that is the only way; finite, so it never disconnects the graph (like the gate/RJ penalties).
const HAZARD_LINK_EXTRA: f32 = 0.5;

/// Multiplier on the cost of every link *entering* an underwater cell (see
/// [`NavGraph::surcharge_water_links`]). Water is not lethal like lava, so it's a soft bias, not the
/// flat [`HAZARD_LINK_EXTRA`] surcharge: a flat add on each of a pool's many short swim links would
/// stack into an effective ban, whereas a multiplier keeps a crossing *proportional* to its length —
/// a dry detour up to ~twice as long wins, longer ones don't. `2.0` sits a touch above the honest
/// physical cost (pmove swims at 0.7× ground wishspeed ⇒ ≥1.43×) to price in the exposure and the
/// lost bunnyhop, so a bot travels through water only when it's genuinely the shorter way.
const WATER_COST_MULT: f32 = 2.0;

/// Travel-time cost of a link: horizontal distance / speed, plus risk/effort penalties so A*
/// prefers grounded routes and avoids damaging falls.
fn link_cost(kind: LinkKind, horiz: f32, dz: f32) -> f32 {
    let base = horiz.max(GRID) / MAX_SPEED;
    // A landing below the takeoff adds its real ballistic fall time (nominal gravity), plus a
    // beat past SAFE_FALL for the hard-landing tax (the flat 5 HP + the recovery stumble) — so
    // a 2000u plunge prices its ~2.2s honestly instead of looking free.
    let fall = if -dz > SAFE_FALL {
        (2.0 * -dz / 800.0).sqrt() + 0.4
    } else {
        0.0
    };
    match kind {
        LinkKind::Walk => base,
        LinkKind::Step => base * 1.1,
        LinkKind::Drop => base + 0.1 + fall,
        LinkKind::JumpGap => base + 0.3 + fall,
        // A double jump is a touch pricier — a harder maneuver (two timed jumps) than a single hop.
        LinkKind::DoubleJump => base + 0.6,
        // Speed-jump costs (runway run-up + flight + commitment) are computed at splice time.
        LinkKind::SpeedJump => base + 2.0,
        // Plat costs are computed at splice time (ride time + overhead), not here.
        LinkKind::Plat => base + 1.0,
        // Teleporting is near-instant; cost is set at splice time.
        LinkKind::Teleport => 0.2,
        // Hook costs (throw + reel + parabola airtime + overhead) are computed at splice time from
        // the solved trajectory; this fallback should never actually be priced.
        LinkKind::Hook => base + HOOK_OVERHEAD,
        // Rocket-jump costs (rise-to-blast + arc airtime + overhead + health surcharge) are computed
        // at splice time; this fallback should never actually be priced.
        LinkKind::RocketJump => base + 4.0,
    }
}

/// Per-map navigation state, reset each map load. Lives on `GameState`.
/// The product of a background navmesh build handed back to the main thread: the parsed BSP and
/// the finished graph, or `None` if the BSP couldn't be parsed. `Send` (plain data), so it crosses
/// the worker→main channel.
pub type NavBuild = Option<(Bsp, NavGraph)>;

#[derive(Default)]
pub struct NavState {
    /// The parsed clip-hull geometry the navmesh is derived from. `None` until a map's BSP
    /// has been successfully read and parsed.
    pub bsp: Option<Bsp>,
    /// The built navigation graph. `None` until [`NavGraph::build`] runs (bots stay disabled).
    pub graph: Option<NavGraph>,
    /// Whether a build has been kicked off for this map (so a failed BSP read doesn't retry every
    /// frame). Reset when a new map loads.
    pub attempted: bool,
    /// A background build in flight: the channel the worker thread delivers its finished graph on.
    /// The main thread polls it each frame and swaps the result into `graph`/`bsp` when ready
    /// (`None` when no build is running). Dropping it (on map change) discards a stale build.
    pub pending: Option<std::sync::mpsc::Receiver<NavBuild>>,
    /// Static catalog of item-goal pickups: `(entity index, nearest cell)`. Built once with the
    /// graph; items don't move, so their cell is fixed. Live availability and desire are read
    /// fresh at selection time (by the game's `bot::goals`).
    pub goals: Vec<(u32, CellId)>,
}

/// Build a navmesh off the main thread from pre-gathered, `Send` inputs: the raw BSP bytes plus the
/// entity-derived plat/teleport/gate info. Pure — no engine or game-state access — so it runs
/// safely on a worker thread whose result the main thread swaps in when ready.
#[allow(clippy::too_many_arguments)] // the per-map build knobs; a params struct would just relocate them
pub fn build_navmesh(
    bytes: Vec<u8>,
    plats: Vec<PlatInfo>,
    teleports: Vec<TeleportInfo>,
    gates: Vec<GateInfo>,
    hooks: Option<HookParams>,
    double_jump: bool,
    speed_jump: Option<SpeedJumpParams>,
    rocket_jump: Option<RocketJumpParams>,
) -> NavBuild {
    let run = move || -> NavBuild {
        let bsp = Bsp::parse(&bytes)?;
        let mut graph = NavGraph::build(&bsp);
        // Static-geometry jump/hook splices first (before the plat/gate splices): keeps plat surfaces
        // off their endpoints and lets `add_gates` tag any of these links that cross a door.
        if double_jump {
            graph.add_double_jumps(&bsp);
        }
        // Speed jumps after double jumps, so they only fill the gaps double jumps can't (they see the DJ
        // links via `has_direct_link`).
        if let Some(params) = speed_jump {
            graph.add_speed_jumps(&bsp, params, double_jump);
        }
        // Hooks first: they derive from the static hull, and going before the plat/gate splices keeps
        // plat surfaces off hook endpoints and lets `add_gates` tag any hook link crossing a door.
        if let Some(params) = hooks {
            graph.add_hooks(&bsp, params);
        }
        // Rocket jumps after hooks: `has_direct_link` then skips any ledge a (free, cheaper) hook already
        // reaches, so an RJ link is only spent where nothing else gets there.
        if let Some(params) = rocket_jump {
            graph.add_rocket_jumps(&bsp, params, double_jump);
        }
        graph.add_plats(&bsp, &plats);
        graph.add_teleports(&teleports);
        graph.add_gates(&gates);
        Some((bsp, graph))
    };
    // Run the (rayon-parallel) build on a transient pool sized to leave one core for the caller.
    // A transient pool rather than rayon's process-global one because this crate ships inside a
    // native game module (rtx.dll) the engine can unload: global-pool worker threads would outlive
    // the DLL and later run freed code. This pool's threads are joined when the call returns.
    // If parallelism can't be queried or the pool fails to build, fall back to running inline (which
    // uses rayon's global pool for the par_iters — fine in tests and the standalone viewer).
    let threads = std::thread::available_parallelism()
        .map(|n| n.get().saturating_sub(1).max(1))
        .unwrap_or(1);
    match rayon::ThreadPoolBuilder::new().num_threads(threads).build() {
        Ok(pool) => pool.install(run),
        Err(_) => run(),
    }
}

impl NavState {
    /// Whether a usable navmesh exists for the current map.
    pub fn is_loaded(&self) -> bool {
        self.graph.as_ref().is_some_and(|g| !g.cells.is_empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The parabola integrator matches the closed-form ballistic solution over a flat floor.
    #[test]
    fn hook_arc_matches_closed_form() {
        // Floor at z = 0 (solid at or below), open above.
        let floor = |p: Vec3| p.z <= 0.0;
        let r = Vec3::new(0.0, 0.0, 100.0);
        let v0 = Vec3::new(200.0, 0.0, 300.0);
        let g = 800.0;
        // Closed form: 100 + 300t - 400t^2 = 0 -> t = 1.0; x = 200.
        match simulate_arc(floor, r, v0, g) {
            ArcResult::Land { pos, airtime, vz } => {
                assert!((pos.x - 200.0).abs() < 20.0, "landing x {} != ~200", pos.x);
                assert!(pos.z.abs() < HOOK_SAMPLE, "landing z {} not near floor", pos.z);
                assert!((airtime - 1.0).abs() < 0.1, "airtime {airtime} != ~1.0");
                assert!(vz < 0.0, "must be descending at landing");
            }
            _ => panic!("arc did not land on the floor"),
        }
        // A ceiling just above the release point blocks the (ascending) arc.
        let boxed = |p: Vec3| p.z <= 0.0 || p.z >= 110.0;
        assert!(
            matches!(simulate_arc(boxed, r, v0, g), ArcResult::Blocked),
            "arc into a ceiling should be Blocked"
        );
    }

    /// `ground_along` keeps a link whose whole span has floor under it — flat floor, straight or
    /// grid-diagonal. The oracle is hull-1 solidity: solid at or below the resting origin z (24 here).
    #[test]
    fn ground_along_keeps_continuous_floor() {
        let floor = |p: Vec3| p.z <= 24.0;
        assert!(ground_along(&floor, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 0.0, 24.0)));
        assert!(ground_along(&floor, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 32.0, 24.0)));
    }

    /// A grid-diagonal link whose centre line crosses a hole in the floor (wider than the player box,
    /// so the hull-1 oracle reads air there) is severed — the stair-side L-corner fall.
    #[test]
    fn ground_along_severs_diagonal_over_hole() {
        // Flat floor except a 16u square hole straddling the diagonal midpoint; endpoints stay solid.
        let holed = |p: Vec3| p.z <= 24.0 && !((8.0..24.0).contains(&p.x) && (8.0..24.0).contains(&p.y));
        assert!(!ground_along(&holed, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 32.0, 24.0)));
    }

    /// Balancing along a thin walkable strip (a wall-top) survives: a link running *along* a 32u-wide
    /// crest keeps floor under every sample. The hull-1 oracle reports solid across a strip at least
    /// as wide as the player box — exactly what carried the cells there in the first place.
    #[test]
    fn ground_along_keeps_thin_strip() {
        let strip = |p: Vec3| p.z <= 24.0 && p.y.abs() <= 16.0;
        assert!(ground_along(&strip, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 0.0, 24.0)));
        assert!(ground_along(&strip, Vec3::new(0.0, 0.0, 24.0), Vec3::new(64.0, 0.0, 24.0)));
    }

    /// Build the navmesh from a real map (`RTX_TEST_BSP`) and sanity-check it: cells and links
    /// exist, and a healthy majority of cells land in one connected component (a fragmented
    /// graph means missing jump/step links). Reports per-kind counts. Skipped without the env.
    #[test]
    fn builds_navmesh() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let mut g = NavGraph::build(&bsp);
        assert!(!g.cells.is_empty(), "no cells carved");
        assert!(!g.links.is_empty(), "no links built");

        // Largest connected component via union-find over (undirected) links.
        let mut parent: Vec<u32> = (0..g.cells.len() as u32).collect();
        fn find(p: &mut [u32], mut x: u32) -> u32 {
            while p[x as usize] != x {
                p[x as usize] = p[p[x as usize] as usize];
                x = p[x as usize];
            }
            x
        }
        for l in &g.links {
            let (a, b) = (find(&mut parent, l.from), find(&mut parent, l.to));
            parent[a as usize] = b;
        }
        let mut sizes: HashMap<u32, u32> = HashMap::new();
        for i in 0..g.cells.len() as u32 {
            let r = find(&mut parent, i);
            *sizes.entry(r).or_default() += 1;
        }
        let largest = sizes.values().copied().max().unwrap_or(0);
        let frac = largest as f32 / g.cells.len() as f32;
        let c = g.summary();
        eprintln!(
            "{path}: {} cells, {} links (walk {} step {} drop {} jump {}); \
             largest component {largest}/{} = {:.0}%",
            g.cells.len(),
            g.links.len(),
            c.walk,
            c.step,
            c.drop,
            c.jump,
            g.cells.len(),
            frac * 100.0,
        );
        assert!(frac > 0.5, "navmesh too fragmented: {:.0}%", frac * 100.0);

        // Links are directed (drops/jumps are one-way), so directed reachability depends on the
        // start. Sample several starts and take the best — that models a bot spawned at a
        // player-start, which should be able to roam most of the map.
        let directed_reach = |start: CellId| -> Vec<CellId> {
            let mut seen = vec![false; g.cells.len()];
            let mut stack = vec![start];
            let mut order = vec![start];
            seen[start as usize] = true;
            while let Some(c) = stack.pop() {
                for &li in &g.adjacency[c as usize] {
                    let to = g.links[li as usize].to;
                    if !seen[to as usize] {
                        seen[to as usize] = true;
                        stack.push(to);
                        order.push(to);
                    }
                }
            }
            order
        };
        let step = (g.cells.len() / 32).max(1);
        let best = (0..g.cells.len() as u32)
            .step_by(step)
            .map(directed_reach)
            .max_by_key(|o| o.len())
            .unwrap();
        let (start, reached) = (best[0], best.len());
        let reach_frac = reached as f32 / g.cells.len() as f32;
        eprintln!(
            "best directed reach: {reached}/{} = {:.0}%",
            g.cells.len(),
            reach_frac * 100.0
        );

        // Assert A* returns a valid chain to the farthest reachable cell.
        let goal = *best.last().unwrap();
        let route = g
            .find_path(start, goal, &LinkCosts::default())
            .expect("A* found no route to a reachable cell");
        let mut cell = start;
        for &li in &route {
            assert_eq!(g.links[li as usize].from, cell, "route discontinuity");
            cell = g.links[li as usize].to;
        }
        assert_eq!(cell, goal, "route did not reach goal");
        eprintln!("A*: route to {goal} is {} links", route.len());
        assert!(
            reach_frac > 0.4,
            "best directed reach too low: {:.0}%",
            reach_frac * 100.0
        );

        // Plat splice: synthesize a lift whose board sits just above a well-connected cell and
        // whose exit is another reachable cell; confirm the ride + board cell wire in. (The
        // jump-aboard count depends on local geometry/headroom, so it's reported, not asserted.)
        let (links_before, cells_before) = (g.links.len(), g.cells.len());
        let board = g.cells[start as usize].origin + Vec3::Z * 24.0;
        let exit = g.cells[goal as usize].origin;
        g.add_plats(
            &bsp,
            &[PlatInfo {
                board,
                exit,
                entity: 7,
                fp_min: board.xy() - Vec2::splat(32.0),
                fp_max: board.xy() + Vec2::splat(32.0),
            }],
        );
        assert_eq!(g.summary().plat, 1, "plat ride not added");
        assert_eq!(g.cells.len(), cells_before + 1, "board cell not added");
        // The lift registered, and its ride link (the first link added) plus every jump-aboard link
        // carries the plat tag, while a pre-existing static link does not.
        assert_eq!(g.plat_count(), 1, "plat not registered");
        assert_eq!(g.plat(0).entity, 7, "plat entity id not stored");
        assert_eq!(g.plat_of_link(links_before as u32), Some(0), "ride link not tagged");
        let tagged = (links_before as u32..g.links.len() as u32)
            .filter(|&li| g.plat_of_link(li) == Some(0))
            .count();
        assert_eq!(tagged, g.links.len() - links_before, "all plat links tagged");
        assert_eq!(g.plat_of_link(0), None, "static link wrongly tagged");
        eprintln!("plat splice: {} jump-aboard links", g.links.len() - links_before - 1);

        // Teleport splice: a trigger box around a well-connected cell warping to another cell.
        // Every standable cell in the box should gain a Teleport link to the destination.
        let near = g.cells[start as usize].origin;
        let tmin = near - Vec3::new(40.0, 40.0, 8.0);
        let tmax = near + Vec3::new(40.0, 40.0, 56.0);
        g.add_teleports(&[TeleportInfo { tmin, tmax, dest: exit }]);
        let tele = g.summary().teleport;
        assert!(tele >= 1, "no teleport links added");
        eprintln!("teleport splice: {tele} entrance links");

        // Gate splice: a closed door box over a well-connected cell, with a button nearby. Links
        // whose segment crosses the box become gated, and the button resolves to an operating cell.
        let dcell = g.cells[start as usize].origin;
        let gate = GateInfo {
            obstruction: 0,
            closed_origin: dcell,
            closed_min: dcell - Vec3::new(32.0, 32.0, 8.0),
            closed_max: dcell + Vec3::new(32.0, 32.0, 56.0),
            activator: 0,
            button: g.cells[goal as usize].origin,
            shoot: false,
        };
        g.add_gates(&[gate]);
        assert_eq!(g.gate_count(), 1, "gate not registered");
        let gated_links = (0..g.links.len() as u32)
            .filter(|&li| g.gate_of_link(li).is_some())
            .count();
        assert!(gated_links > 0, "no link tagged by the gate");
        // The state-aware A* still resolves with the gate shut (routes around, or through with the
        // penalty when there's no other way).
        let shut = LinkCosts {
            gate_closed: &[true],
            ..Default::default()
        };
        assert!(g.find_path(start, goal, &shut).is_some(), "no route with gate shut");
        eprintln!(
            "gate splice: {gated_links} gated links, button cell {}",
            g.gate(0).button_cell
        );

        // Hook splice: build a fresh graph (the gate test mutated `g`) and run the hook pass with
        // stock physics. Hooks derive from real geometry, so — like reach — we report the count
        // rather than assert a floor (flat maps legitimately have none), but every emitted link
        // must satisfy its invariants and its stored arc must re-simulate onto the target corridor.
        let params = HookParams {
            gravity: 800.0,
            pull: HOOK_PULL_BASE,
            throw: HOOK_THROW_BASE,
        };
        let mut gh = NavGraph::build(&bsp);
        let reach_before = {
            let step = (gh.cells.len() / 32).max(1);
            (0..gh.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gh, s))
                .max()
                .unwrap_or(0)
        };
        gh.add_hooks(&bsp, params);
        let hooks = gh.summary().hook;
        let mut vertical = 0;
        let mut fling = 0;
        for li in 0..gh.links.len() as u32 {
            if gh.link_kind(li) != LinkKind::Hook {
                continue;
            }
            let tr = *gh.hook_of_link(li).expect("hook link missing its traversal");
            let a = gh.cell_origin(gh.link_source(li));
            let b = gh.cell_origin(gh.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                (HOOK_MIN_RISE..=HOOK_MAX_RISE).contains(&dz) && horiz <= HOOK_RANGE_XY,
                "hook link out of range: dz={dz} horiz={horiz}"
            );
            assert!(dz > JUMP_APEX || horiz > JUMP_REACH, "hook link no better than a jump");
            // Re-fly the stored arc: release point is `release_dist` back from the stick along v0.
            let dir = tr.v0.normalize_or_zero();
            let r = tr.stick - dir * tr.release_dist;
            match simulate_arc(|p| bsp.is_solid(p), r, tr.v0, params.gravity) {
                ArcResult::Land { pos, .. } => {
                    let d = (pos.xy() - b.xy()).length();
                    assert!(d <= HOOK_LAND_XY * 2.0, "stored arc lands {d} from target (li {li})");
                }
                _ => panic!("stored hook arc no longer lands (li {li})"),
            }
            if horiz > JUMP_REACH {
                fling += 1;
            } else {
                vertical += 1;
            }
        }
        // Determinism: a second identical build yields the same hook count (the runtime re-solve
        // and the offline build must agree).
        let mut gh2 = NavGraph::build(&bsp);
        gh2.add_hooks(&bsp, params);
        assert_eq!(gh2.summary().hook, hooks, "hook build not deterministic");

        let reach_after = {
            let step = (gh.cells.len() / 32).max(1);
            (0..gh.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gh, s))
                .max()
                .unwrap_or(0)
        };
        assert!(reach_after >= reach_before, "hooks reduced reachability");
        eprintln!(
            "hook splice: {hooks} links ({vertical} vertical, {fling} fling); \
             best directed reach {reach_before} -> {reach_after}"
        );

        // Double-jump splice: every emitted link must be beyond a single jump's reach (else a
        // JumpGap already covers it) but within the double-jump envelope. Report the count.
        let mut gd = NavGraph::build(&bsp);
        gd.add_double_jumps(&bsp);
        let djumps = gd.summary().double_jump;
        for li in 0..gd.links.len() as u32 {
            if gd.link_kind(li) != LinkKind::DoubleJump {
                continue;
            }
            let a = gd.cell_origin(gd.link_source(li));
            let b = gd.cell_origin(gd.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                horiz <= DOUBLE_JUMP_REACH && (-DJ_MAX_DROP..=DOUBLE_JUMP_APEX).contains(&dz),
                "double-jump link out of envelope: dz={dz} horiz={horiz}"
            );
            assert!(
                horiz > JUMP_REACH || dz > JUMP_APEX,
                "double-jump link a single jump could make"
            );
        }
        eprintln!("double-jump splice: {djumps} links");

        // Speed-jump splice: leaps beyond a single jump, cleared by bhop speed. Every emitted link's
        // takeoff must need more than maxspeed, its from-cell must sit a real runway back from the
        // takeoff, and the required speed must be within the runway's attainable cap.
        let params = SpeedJumpParams {
            gravity: 800.0,
            accel: 10.0,
            maxspeed: MAX_SPEED,
            friction: 4.0,
            stopspeed: 100.0,
            curl: false, // this test asserts straight-speed-jump invariants
        };
        let k = bhop_k(params.accel, params.maxspeed);
        let mut gs = NavGraph::build(&bsp);
        gs.add_speed_jumps(&bsp, params, false);
        let sjumps = gs.summary().speed_jump;
        for li in 0..gs.links.len() as u32 {
            if gs.link_kind(li) != LinkKind::SpeedJump {
                continue;
            }
            let tr = *gs
                .speed_jump_of_link(li)
                .expect("speed-jump link missing its traversal");
            let start = gs.cell_origin(gs.link_source(li));
            let land = gs.cell_origin(gs.link_target(li));
            let horiz = (land.xy() - tr.takeoff.xy()).length();
            // Beyond a single jump's *reach* (else a JumpGap covers it) — a wide flat gap, or a
            // downhill one whose extra airtime a JumpGap's flat 200u cap still missed.
            assert!(horiz > JUMP_REACH, "speed jump within single-jump reach: {horiz}");
            assert!(tr.v_req <= SPEED_JUMP_V_CAP + 1.0, "v_req over the cap: {}", tr.v_req);
            // A *chained* speed jump has no self-contained runway by design — its from-cell **is**
            // the ledge (it's only traversable when the planner proves the entry band already carries
            // `v_req`), so the runway-back invariant applies only to stand-start jumps.
            if !tr.chained {
                // The from-cell is the runway start: at least the runway needed to build the *extra*
                // speed over maxspeed (a gap crossable at ≤ maxspeed needs no runway → from = ledge).
                let need = runway_len_for(tr.v_req.max(MAX_SPEED), MAX_SPEED, k);
                let back = (start.xy() - tr.takeoff.xy()).length();
                assert!(back + GRID >= need, "runway too short: {back} < {need}");
            }
        }
        eprintln!("speed-jump splice: {sjumps} links");

        // Rocket-jump splice: blast-launched leaps up to high ledges. Every emitted link must clear
        // more than a single jump's apex (else a jump covers it), sit within the RJ envelope, and its
        // stored (pos_blast, v0) arc must re-simulate onto the target — the offline solve and the
        // runtime re-flight must agree. Default-mode physics (gravity 800, no `rj` boost).
        let rjp = RocketJumpParams { gravity: 800.0, rj_extra: 0.0 };
        let mut gr = NavGraph::build(&bsp);
        let reach_before = {
            let step = (gr.cells.len() / 32).max(1);
            (0..gr.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gr, s))
                .max()
                .unwrap_or(0)
        };
        gr.add_rocket_jumps(&bsp, rjp, false);
        let rjumps = gr.summary().rocket_jump;
        for li in 0..gr.links.len() as u32 {
            if gr.link_kind(li) != LinkKind::RocketJump {
                continue;
            }
            let tr = *gr.rocket_jump_of_link(li).expect("rocket-jump link missing its traversal");
            let a = gr.cell_origin(gr.link_source(li));
            let b = gr.cell_origin(gr.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                (RJ_MIN_RISE..=RJ_MAX_RISE).contains(&dz) && horiz <= RJ_RANGE_XY,
                "rocket-jump link out of envelope: dz={dz} horiz={horiz}"
            );
            assert!(dz > JUMP_APEX, "rocket-jump link a single jump could make: dz={dz}");
            assert!(tr.self_damage > 0.0, "rocket-jump link with no self-blast");
            // Re-fly the stored continuation arc onto the target corridor.
            match simulate_arc(|p| bsp.is_solid(p), tr.pos_blast, tr.v0, rjp.gravity) {
                ArcResult::Land { pos, .. } => {
                    let d = (pos.xy() - b.xy()).length();
                    assert!(d <= RJ_LAND_XY * 2.0, "stored RJ arc lands {d} from target (li {li})");
                }
                _ => panic!("stored rocket-jump arc no longer lands (li {li})"),
            }
        }
        // Determinism.
        let mut gr2 = NavGraph::build(&bsp);
        gr2.add_rocket_jumps(&bsp, rjp, false);
        assert_eq!(gr2.summary().rocket_jump, rjumps, "rocket-jump build not deterministic");
        let reach_after = {
            let step = (gr.cells.len() / 32).max(1);
            (0..gr.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gr, s))
                .max()
                .unwrap_or(0)
        };
        assert!(reach_after >= reach_before, "rocket jumps reduced reachability");
        eprintln!("rocket-jump splice: {rjumps} links; best directed reach {reach_before} -> {reach_after}");
    }

    /// Two full builds of the same map must produce a byte-identical graph — the guard that the
    /// rayon-parallel solvers stay deterministic across link indices, adjacency, and every
    /// side-table payload. Env-gated on `RTX_TEST_BSP` like [`builds_navmesh`]. Prints an FNV-1a
    /// fingerprint of the graph so a change that alters build output (versus an earlier commit) is
    /// visible run to run, not just detectable as a build-vs-build mismatch.
    #[test]
    fn build_deterministic() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let hooks = Some(HookParams {
            gravity: 800.0,
            pull: HOOK_PULL_BASE,
            throw: HOOK_THROW_BASE,
        });
        let speed = Some(SpeedJumpParams {
            gravity: 800.0,
            accel: 10.0,
            maxspeed: MAX_SPEED,
            friction: 4.0,
            stopspeed: 100.0,
            curl: true, // cover the curl-generation pass's determinism
        });
        let rj = Some(RocketJumpParams { gravity: 800.0, rj_extra: 0.0 });
        // All solvers on, no entity splices (they're serial and not the subject here).
        let build = || {
            build_navmesh(bytes.clone(), vec![], vec![], vec![], hooks, true, speed, rj)
                .expect("build produced no graph")
                .1
        };
        let a = build();
        let b = build();

        fn mix(h: &mut u64, x: u64) {
            *h ^= x;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3); // FNV-1a 64-bit prime
        }
        fn mix_vec3(h: &mut u64, v: Vec3) {
            mix(h, v.x.to_bits() as u64);
            mix(h, v.y.to_bits() as u64);
            mix(h, v.z.to_bits() as u64);
        }
        fn fingerprint(g: &NavGraph) -> u64 {
            let mut h: u64 = 0xcbf2_9ce4_8422_2325; // FNV-1a offset basis
            mix(&mut h, g.cells.len() as u64);
            for c in &g.cells {
                mix_vec3(&mut h, c.origin);
                mix(&mut h, c.gx as u32 as u64);
                mix(&mut h, c.gy as u32 as u64);
            }
            mix(&mut h, g.links.len() as u64);
            for l in &g.links {
                mix(&mut h, l.from as u64);
                mix(&mut h, l.to as u64);
                mix(&mut h, l.kind as u64);
                mix(&mut h, l.cost.to_bits() as u64);
            }
            for adj in &g.adjacency {
                mix(&mut h, adj.len() as u64);
                for &li in adj {
                    mix(&mut h, li as u64);
                }
            }
            for &x in g.hooks.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.hooks.items_raw() {
                mix_vec3(&mut h, t.stick);
                mix(&mut h, t.release_dist.to_bits() as u64);
                mix_vec3(&mut h, t.v0);
                mix(&mut h, t.airtime.to_bits() as u64);
            }
            for &x in g.speed_jumps.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.speed_jumps.items_raw() {
                mix_vec3(&mut h, t.takeoff);
                mix(&mut h, t.v_req.to_bits() as u64);
                mix(&mut h, t.airtime.to_bits() as u64);
                mix(&mut h, t.chained as u64);
                mix(&mut h, t.curl_gain.to_bits() as u64);
            }
            for &x in g.rocket_jumps.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.rocket_jumps.items_raw() {
                mix_vec3(&mut h, t.fire_angles);
                mix(&mut h, t.fire_delay.to_bits() as u64);
                mix_vec3(&mut h, t.blast);
                mix_vec3(&mut h, t.pos_blast);
                mix_vec3(&mut h, t.v0);
                mix(&mut h, t.airtime.to_bits() as u64);
                mix(&mut h, t.self_damage.to_bits() as u64);
            }
            h
        }

        assert_eq!(a.links.len(), b.links.len(), "link count not deterministic");
        assert_eq!(a.adjacency, b.adjacency, "adjacency not deterministic");
        let (fa, fb) = (fingerprint(&a), fingerprint(&b));
        assert_eq!(fa, fb, "navmesh build not deterministic (fingerprint mismatch)");
        let c = a.summary();
        eprintln!(
            "{path}: deterministic build fingerprint {fa:#018x} \
             ({} cells, {} links; hook {} speedjump {} doublejump {} rocketjump {})",
            a.cells.len(),
            a.links.len(),
            c.hook,
            c.speed_jump,
            c.double_jump,
            c.rocket_jump,
        );
    }

    /// The speed-jump ballistic + runway model, and its agreement with the real bhop controller.
    #[test]
    fn speed_jump_model() {
        // A jump at exactly maxspeed reaches ~JUMP_REACH: v_required(216, 0) ≈ 320.
        let t = jump_airtime(0.0, 800.0);
        assert!((t - 0.675).abs() < 0.01, "flat airtime {t}");
        assert!((v_required(MAX_SPEED * t, 0.0, 800.0) - MAX_SPEED).abs() < 1.0);
        // Rising shrinks airtime (needs more speed); dropping lengthens it.
        assert!(jump_airtime(45.0, 800.0) < t && jump_airtime(-200.0, 800.0) > t);

        // attainable_speed / runway_len_for are inverses.
        let k = bhop_k(10.0, MAX_SPEED);
        let v = attainable_speed(MAX_SPEED, 800.0, k);
        assert!(v > 450.0, "800u runway should build good speed, got {v}"); // ~480, ≈1.5× maxspeed
        assert!((runway_len_for(v, MAX_SPEED, k) - 800.0).abs() < 1.0);
        // The cross-check that this derated model is conservative vs the *actual* bhop controller
        // lives in the game crate (`bot::bhop`'s tests), where the controller sim is defined.
    }

    /// Count cells directly reachable from `start` over the (directed) graph — a small DFS helper
    /// shared by the reach-delta checks.
    fn directed_reach_len(g: &NavGraph, start: CellId) -> usize {
        let mut seen = vec![false; g.cells.len()];
        let mut stack = vec![start];
        seen[start as usize] = true;
        let mut n = 1;
        while let Some(c) = stack.pop() {
            for &li in &g.adjacency[c as usize] {
                let to = g.links[li as usize].to;
                if !seen[to as usize] {
                    seen[to as usize] = true;
                    n += 1;
                    stack.push(to);
                }
            }
        }
        n
    }

    /// A diamond: two routes from cell 0 to cell 3, one (via 1) slightly cheaper than the other (via
    /// 2). Enough to exercise per-link penalty diversion and jitter without a BSP.
    ///   links: 0=(0→1,1.0) 1=(1→3,1.0)  2=(0→2,1.1) 3=(2→3,1.1)
    fn diamond() -> NavGraph {
        let cell = |x: f32, y: f32| Cell {
            origin: Vec3::new(x, y, 0.0),
            gx: 0,
            gy: 0,
        };
        let link = |from: CellId, to: CellId, cost: f32| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost,
        };
        NavGraph {
            cells: vec![cell(0.0, 0.0), cell(100.0, 50.0), cell(100.0, -50.0), cell(200.0, 0.0)],
            links: vec![link(0, 1, 1.0), link(1, 3, 1.0), link(0, 2, 1.1), link(2, 3, 1.1)],
            adjacency: vec![vec![0, 2], vec![1], vec![3], vec![]],
            water: Vec::new(),
            breathable: Vec::new(),
            grid: GridIndex::default(),
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
        }
    }

    /// The banded planner credits bhop speed gains on a flat corridor but not up a staircase: an
    /// ascending Walk/Step leg carries the entry band without climbing it (a human runs up stairs;
    /// you can't air-strafe-build up risers), while a flat leg of equal length gains a band.
    #[test]
    fn banded_step_no_gain_up_stairs() {
        let cell = |x: f32, y: f32, z: f32| Cell {
            origin: Vec3::new(x, y, z),
            gx: 0,
            gy: 0,
        };
        let step = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Step,
            cost: 1.0,
        };
        let g = NavGraph {
            cells: vec![cell(0.0, 0.0, 0.0), cell(1500.0, 0.0, 0.0), cell(0.0, 1500.0, 100.0)],
            links: vec![step(0, 1), step(0, 2)], // link 0: 1500u flat; link 1: 1500u rising 100u
            adjacency: vec![vec![0, 1], vec![], vec![]],
            water: Vec::new(),
            breathable: Vec::new(),
            grid: GridIndex::default(),
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
        };
        let (_, flat_exit) = g.banded_step(0, 0).unwrap();
        let (_, up_exit) = g.banded_step(1, 0).unwrap();
        assert!(flat_exit >= 1, "a long flat corridor should climb a band, got {flat_exit}");
        assert_eq!(up_exit, 0, "an ascending leg must not gain a band, got {up_exit}");
    }

    /// A per-link penalty diverts A* onto the alternate route once it exceeds the route's cost
    /// margin, and the route reverts the moment the penalty is gone — the loop-free-nav core.
    #[test]
    fn penalty_diverts_then_reverts() {
        let g = diamond();
        // No penalty → the cheaper route via cell 1.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
        // A penalty smaller than the 0.2s route-cost gap doesn't flip it.
        let tiny = [(0u32, 0.05f32)];
        let costs = LinkCosts {
            penalties: &tiny,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![0, 1]);
        // A larger penalty on link 0 (0→1) diverts onto the route via cell 2.
        let big = [(0u32, 5.0f32)];
        let costs = LinkCosts {
            penalties: &big,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![2, 3]);
        // Penalty expired (absent from the slice) → back to the cheap route.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
    }

    /// The rocket-jump fitness gate surcharges *only* RocketJump links: a bot unfit to rocket-jump
    /// diverts around a cheap-branch RJ leg, and a fit bot (no surcharge) still takes it.
    #[test]
    fn rocket_jump_fitness_gate_diverts() {
        let mut g = diamond();
        g.links[0].kind = LinkKind::RocketJump; // make the cheap branch's first leg (0→1) an RJ
        // Fit bot: no surcharge → the cheap route via cell 1.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
        // Unfit bot: every RJ link costs RJ_UNFIT_PENALTY → diverts onto the route via cell 2.
        let costs = LinkCosts {
            rocket_jump_extra: RJ_UNFIT_PENALTY,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![2, 3]);
        // The surcharge hits only RJ links: a Walk-only diamond is unaffected by it.
        let g2 = diamond();
        assert_eq!(g2.find_path(0, 3, &costs).unwrap(), vec![0, 1]);
    }

    /// A finite penalty never disconnects the graph: if the only route runs through the penalized
    /// link, A* still returns it (finite cost, unlike a closed gate's near-infinite one).
    #[test]
    fn penalty_never_disconnects() {
        let g = diamond();
        let huge = [(0u32, 999.0f32), (2u32, 999.0f32)]; // penalize both first legs
        let costs = LinkCosts {
            penalties: &huge,
            ..Default::default()
        };
        assert!(g.find_path(0, 3, &costs).is_some(), "finite penalties must not sever the route");
    }

    /// Jitter is deterministic per (seed, link) and bounded to `[0, JITTER_FRAC·cost]`.
    #[test]
    fn jitter_bounded_and_deterministic() {
        let g = diamond();
        let costs = LinkCosts {
            jitter_seed: 7,
            ..Default::default()
        };
        for li in 0..g.links.len() as u32 {
            let a = g.link_extra(li, &costs);
            assert_eq!(a, g.link_extra(li, &costs), "jitter must be deterministic");
            assert!(a >= 0.0, "jitter is non-negative (keeps the heuristic admissible)");
            assert!(
                a <= JITTER_FRAC * g.links[li as usize].cost + 1e-6,
                "jitter {a} exceeds the {JITTER_FRAC} cost bound",
            );
        }
        // Zero seed disables jitter entirely.
        assert_eq!(g.link_extra(0, &LinkCosts::default()), 0.0);
    }

    /// `surcharge_hazard_links` flags a cell on a lava edge and bumps the cost of links *into* it,
    /// while leaving an interior link untouched — over synthetic solid/liquid oracles, no BSP.
    #[test]
    fn surcharge_flags_lava_edge_only() {
        // Two adjacent floor cells in a row; open lava sits past the +x cell (grid column 2 has no
        // ground, and lava lurks below the probe there). Solid floor a short step under both cells.
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut grid = GridIndex::default();
        grid.insert((0, 0), vec![0]);
        grid.insert((1, 0), vec![1]);
        let mut g = NavGraph {
            cells: vec![cell(0.0, 0), cell(32.0, 1)],
            links: vec![link(0, 1), link(1, 0)],
            adjacency: vec![vec![0], vec![1]],
            water: Vec::new(),
            breathable: Vec::new(),
            grid,
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
        };
        // Floor a short step (z ≤ −40) below the cells for x ≤ 60; lava fills x > 60 under z = 0.
        let is_solid = |p: Vec3| p.x <= 60.0 && p.z <= -40.0;
        let contents = |p: Vec3| {
            if p.x > 60.0 && p.z < 0.0 {
                crate::bsp::CONTENTS_LAVA as f32
            } else {
                crate::bsp::CONTENTS_EMPTY as f32
            }
        };
        g.surcharge_hazard_links(&is_solid, &contents);
        // Link 0→1 enters the lava-edge cell 1 → surcharged; link 1→0 enters interior cell 0 → not.
        assert!(
            (g.links[0].cost - (1.0 + HAZARD_LINK_EXTRA)).abs() < 1e-6,
            "into-edge link not surcharged: {}",
            g.links[0].cost
        );
        assert_eq!(g.links[1].cost, 1.0, "interior link must be unchanged");
    }

    /// `surcharge_water_links` flags the submerged cell, multiplies the cost of links *into* it while
    /// leaving the exit link alone, and reports the depth via `cell_in_water`/`cell_breathable`.
    #[test]
    fn surcharge_flags_water_cells_and_multiplies_into_links() {
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut g = NavGraph {
            cells: vec![cell(0.0, 0), cell(32.0, 1)],
            links: vec![link(0, 1), link(1, 0)],
            adjacency: vec![vec![0], vec![1]],
            water: Vec::new(),
            breathable: Vec::new(),
            grid: GridIndex::default(),
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
        };
        // Deep water under and around cell 1 (x = 32), up past its eye point; cell 0 (x = 0) is dry.
        let contents = |p: Vec3| {
            if p.x > 16.0 {
                crate::bsp::CONTENTS_WATER as f32
            } else {
                crate::bsp::CONTENTS_EMPTY as f32
            }
        };
        g.surcharge_water_links(&contents);
        assert!(!g.cell_in_water(0) && g.cell_breathable(0), "dry cell 0");
        assert!(g.cell_in_water(1) && !g.cell_breathable(1), "deep cell 1 submerged, no air");
        // Link 0→1 enters the water cell → ×WATER_COST_MULT; link 1→0 exits to dry → unchanged.
        assert!((g.links[0].cost - WATER_COST_MULT).abs() < 1e-6, "into-water link: {}", g.links[0].cost);
        assert_eq!(g.links[1].cost, 1.0, "exit link must stay cheap (the gradient toward shore)");
    }

    /// A shallow cell — origin submerged but the eye point above the surface — is both `in_water`
    /// (swim physics) and `breathable` (a safe spot for a drowning bot to path to for air).
    #[test]
    fn shallow_water_cell_is_breathable() {
        let mut g = diamond();
        // Water fills z < 10 only, so cell origins (z = 0) are wet but their eye points (z = 22) are dry.
        let contents = |p: Vec3| {
            if p.z < 10.0 {
                crate::bsp::CONTENTS_WATER as f32
            } else {
                crate::bsp::CONTENTS_EMPTY as f32
            }
        };
        g.surcharge_water_links(&contents);
        assert!(g.cell_in_water(0), "origin under the surface");
        assert!(g.cell_breathable(0), "eye above the surface — can breathe");
    }

    /// The surcharge diverts A* off a lava-edge route the same way a penalty does: flagging the
    /// cheap branch's middle cell (via a localized lava pool) flips the route onto the safe branch,
    /// and the graph is never severed. Reuses [`diamond`] (empty grid ⇒ every side is an edge, so
    /// the oracle alone decides), with `is_solid` false everywhere so non-lava probes read as a
    /// *pit* — which the surcharge deliberately ignores, leaving only the lava-flagged cell charged.
    #[test]
    fn surcharge_diverts_off_lava_route() {
        let mut g = diamond();
        // Baseline: cheaper route via cell 1.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
        // A lava pool localized around cell 1's origin (100, 50); nothing else is near it.
        let is_solid = |_: Vec3| false;
        let lava_at = |cx: f32, cy: f32| {
            move |p: Vec3| {
                if (p.x - cx).abs() < 40.0 && (p.y - cy).abs() < 40.0 && p.z < 0.0 {
                    crate::bsp::CONTENTS_LAVA as f32
                } else {
                    crate::bsp::CONTENTS_EMPTY as f32
                }
            }
        };
        g.surcharge_hazard_links(&is_solid, &lava_at(100.0, 50.0));
        // Cell 1 now sits on lava, so 0→1 costs more → A* takes the (now cheaper) route via cell 2.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![2, 3]);
        // Flag the other branch too: both first legs charged, but finite — a route still exists.
        g.surcharge_hazard_links(&is_solid, &lava_at(100.0, -50.0));
        assert!(
            g.find_path(0, 3, &LinkCosts::default()).is_some(),
            "the moderate surcharge must never sever the graph"
        );
    }

    // --- speed-band planning (Phase B) ---

    /// Build a synthetic graph for banded-planner tests: cells at the given origins, directed links
    /// `(from, to, kind, cost)`, and speed-jump side entries `(link index, v_req, airtime, chained)`.
    fn banded_graph(
        origins: &[Vec3],
        links: &[(CellId, CellId, LinkKind, f32)],
        sjs: &[(usize, f32, f32, bool, f32)],
    ) -> NavGraph {
        let cells = origins
            .iter()
            .map(|&o| Cell { origin: o, gx: 0, gy: 0 })
            .collect::<Vec<_>>();
        let mut adjacency = vec![Vec::new(); cells.len()];
        let links = links
            .iter()
            .enumerate()
            .map(|(li, &(from, to, kind, cost))| {
                adjacency[from as usize].push(li as u32);
                Link { from, to, kind, cost }
            })
            .collect::<Vec<_>>();
        let mut speed_jumps = SideTable::default();
        for &(li, v_req, airtime, chained, curl_gain) in sjs {
            let s = speed_jumps.push(SpeedJumpTraversal { takeoff: origins[links[li].from as usize], v_req, airtime, chained, curl_gain });
            speed_jumps.tag(li, s);
        }
        NavGraph {
            cells,
            links,
            adjacency,
            water: Vec::new(),
            breathable: Vec::new(),
            grid: GridIndex::default(),
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps,
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
        }
    }

    /// Every banded step costs at least `horiz / BAND_V_MAX` — the floor that keeps the banded
    /// heuristic (`dist / BAND_V_MAX`) admissible — across every link kind and entry band.
    #[test]
    fn banded_cost_is_admissible() {
        let origins = [
            Vec3::ZERO,
            Vec3::new(300.0, 0.0, 0.0),
            Vec3::new(600.0, 0.0, -200.0),
            Vec3::new(900.0, 0.0, 0.0),
        ];
        let kinds = [
            LinkKind::Walk,
            LinkKind::Step,
            LinkKind::Drop,
            LinkKind::JumpGap,
            LinkKind::DoubleJump,
            LinkKind::SpeedJump,
            LinkKind::Teleport,
        ];
        for kind in kinds {
            let g = banded_graph(
                &origins,
                &[(0, 1, kind, 3.0)],
                if kind == LinkKind::SpeedJump { &[(0, 350.0, 0.7, false, 0.0)] } else { &[] },
            );
            let horiz = (origins[1].xy() - origins[0].xy()).length();
            for band in 0..NBANDS as u8 {
                if let Some((cost, _)) = g.banded_step(0, band) {
                    assert!(
                        cost + 1e-4 >= horiz / BAND_V_MAX,
                        "{kind:?} band {band}: cost {cost} below the admissibility floor {}",
                        horiz / BAND_V_MAX,
                    );
                }
            }
        }
    }

    /// A longer straight Walk leg exits in a higher band, and a standing start (band 0) pays a
    /// spin-up runway before it gains — so a short leg from a standstill stays in band 0.
    #[test]
    fn banded_walk_exit_bands_monotone() {
        let g = banded_graph(
            &[Vec3::ZERO, Vec3::new(200.0, 0.0, 0.0), Vec3::new(2400.0, 0.0, 0.0)],
            &[(0, 1, LinkKind::Walk, 0.6), (0, 2, LinkKind::Walk, 7.5)],
            &[],
        );
        let (_, short) = g.banded_step(0, 0).unwrap(); // 200u from a standstill: below the spin-up
        let (_, long) = g.banded_step(1, 0).unwrap(); // 2400u from a standstill: builds real speed
        assert_eq!(short, 0, "a short standing-start leg should not leave band 0");
        assert!(long > short, "a long leg should exit a higher band than a short one ({long} vs {short})");
    }

    /// A chain of speed jumps with only a short platform between them: unroutable to a speed-unaware
    /// query (chained links priced away), routable to the banded planner when fed from a runway, and
    /// still unroutable from a standstill mid-chain (the carried band can't satisfy the next jump).
    #[test]
    fn banded_chain_needs_carried_speed() {
        // R --walk 2000--> A --chained SJ--> B --walk 200--> B2 --chained SJ--> C
        let g = banded_graph(
            &[
                Vec3::ZERO,                       // 0 R (runway start)
                Vec3::new(2000.0, 0.0, 0.0),      // 1 A (ledge)
                Vec3::new(2300.0, 0.0, 0.0),      // 2 B (landing)
                Vec3::new(2500.0, 0.0, 0.0),      // 3 B2 (short platform)
                Vec3::new(2800.0, 0.0, 0.0),      // 4 C (final landing)
            ],
            &[
                (0, 1, LinkKind::Walk, 6.25),
                (1, 2, LinkKind::SpeedJump, 1.7),
                (2, 3, LinkKind::Walk, 0.625),
                (3, 4, LinkKind::SpeedJump, 1.7),
            ],
            &[(1, 350.0, 0.7, true, 0.0), (3, 350.0, 0.7, true, 0.0)],
        );
        // Speed-unaware: the chained legs are priced away, so C is effectively unreachable.
        let flood = g.costs_from(0, &LinkCosts::default());
        assert!(flood[4] >= CLOSED_GATE_PENALTY, "unbanded query must treat the chain as blocked");
        // Banded, fed from the runway: the walk builds a band that carries both jumps.
        let route = g.find_path_banded(0, 4, MAX_SPEED, &LinkCosts::default()).expect("banded route exists");
        assert_eq!(route.links, vec![0, 1, 2, 3], "banded route should run the whole chain");
        // From a standstill mid-chain (on B2), the next chained jump is infeasible → no route.
        assert!(
            g.find_path_banded(3, 4, 0.0, &LinkCosts::default()).is_none(),
            "a standing start can't satisfy a chained speed jump"
        );
    }

    /// A certified curl speed jump is priced at its stored cost, so the planner takes it over a
    /// detour that beats the conservative per-`v_req` recompute a *straight* speed jump still gets.
    #[test]
    fn banded_prefers_certified_curl_over_detour() {
        // Direct curl R->C (cost 2.0) vs a two-JumpGap detour R->M->C (2 × 1.3 = 2.6). A straight
        // speed jump would be repriced from v_req (391 from a standstill ≈ 3.2s) and lose to the
        // detour; the curl's certified cost (2.0) wins. Same geometry, only the curl flag differs.
        let origins = [Vec3::ZERO, Vec3::new(600.0, 300.0, 0.0), Vec3::new(300.0, 150.0, 0.0)];
        let links = [
            (0, 1, LinkKind::SpeedJump, 2.0), // the curl (or straight) under test
            (0, 2, LinkKind::JumpGap, 1.3),   // detour leg 1
            (2, 1, LinkKind::JumpGap, 1.3),   // detour leg 2
        ];
        let curl = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 12.0)]);
        let route = curl.find_path_banded(0, 1, MAX_SPEED, &LinkCosts::default()).expect("route exists");
        assert_eq!(route.links, vec![0], "the certified curl must be taken over the detour");
        let straight = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 0.0)]);
        let route2 = straight.find_path_banded(0, 1, MAX_SPEED, &LinkCosts::default()).expect("route exists");
        assert_eq!(route2.links, vec![1, 2], "a straight speed jump is repriced high; the detour wins");
    }

    /// Carried speed only survives a corner within the heading cone: a straight approach reaches a
    /// chained speed jump feasibly, an L-shaped one arrives demoted to band 0 and can't take it.
    #[test]
    fn banded_corner_demotes_carry() {
        let long_walk = |to_x: f32, to_y: f32| {
            [Vec3::ZERO, Vec3::new(2000.0, 0.0, 0.0), Vec3::new(to_x, to_y, 0.0)]
        };
        let links = [(0, 1, LinkKind::Walk, 6.25), (1, 2, LinkKind::SpeedJump, 1.7)];
        let sj = [(1usize, 350.0, 0.7, true, 0.0)];
        // Straight: R→M→C all along +x — the carried band satisfies the chained jump.
        let straight = banded_graph(&long_walk(2300.0, 0.0), &links, &sj);
        assert!(
            straight.find_path_banded(0, 2, 0.0, &LinkCosts::default()).is_some(),
            "a straight approach should carry speed into the chained jump"
        );
        // Corner: R→M along +x, then the jump heads +y (90° turn) — carry is demoted, jump infeasible.
        let corner = banded_graph(&long_walk(2000.0, 300.0), &links, &sj);
        assert!(
            corner.find_path_banded(0, 2, 0.0, &LinkCosts::default()).is_none(),
            "a sharp corner should demote the carried band below the jump's requirement"
        );
    }
}
