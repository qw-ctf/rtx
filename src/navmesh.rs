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
//! Staged per the plan: this first cut emits `Walk`/`Step`/`Drop`/`JumpGap`. `CurveJump`,
//! `RocketJump`, water, and entity-derived (teleport/plat/door) links land iteratively on top.

use std::collections::HashMap;

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bsp::Bsp;

// --- player + movement constants (QuakeWorld pmove) ---

/// `STEPSIZE` — pmove climbs steps up to this for free, no jump needed.
const STEP_HEIGHT: f32 = 18.0;
/// Height delta treated as effectively flat ground (a `Walk`).
const WALK_DZ: f32 = 8.0;
/// Largest one-way drop we'll encode as a `Drop` link (taller falls are pruned, not least
/// because fall damage starts to bite — handled in cost).
const MAX_DROP: f32 = 240.0;
/// Fall height beyond which QW fall damage applies (`MAX_SAFE_FALL` ≈ when speed > 580).
const SAFE_FALL: f32 = 88.0;
/// Apex a standing jump adds: `jump_vel² / (2·gravity)` = `270² / 1600`.
const JUMP_APEX: f32 = 45.0;
/// Horizontal reach of a running jump (`maxspeed · air-time`), conservatively floored.
const JUMP_REACH: f32 = 200.0;
/// Extra reach/rise unlocked by rtx's mid-air **double jump** (`rtx_doublejump`): a second jump near
/// the apex restacks a ~45u arc, roughly doubling both. Conservatively floored so a bot with slightly
/// off air-jump timing still clears the linked gap.
const DOUBLE_JUMP_REACH: f32 = 300.0;
const DOUBLE_JUMP_APEX: f32 = 80.0;
/// Clearance envelope for a double jump — the real two-arc path peaks ~91u above the launch, so
/// sample the arc a touch higher to be safe.
const DOUBLE_ARC_PEAK: f32 = 100.0;
/// `sv_maxspeed` default — the cost denominator (travel time = distance / speed).
const MAX_SPEED: f32 = 320.0;

// --- speed jumps (bunnyhop-carried leaps across wide gaps) ---

/// Jump impulse (`velocity.z`) — fixed, so a jump's airtime/apex don't change with horizontal speed;
/// only the reach does (`speed · airtime`). That's what lets a fast bhopping bot clear a wide gap.
const JUMP_VZ: f32 = 270.0;
/// The QW `PM_AirAccelerate` projected-wishspeed cap (mirrors `bot_bhop::AIR_CAP`; cross-checked in a
/// test). Bhop speed builds at a rate set by this and the tickrate.
const SJ_AIR_CAP: f32 = 30.0;
/// Conservative server tickrate assumed for the bhop acceleration model.
const SJ_TICKRATE: f32 = 72.0;
/// Speed we'll plan bhop runways up to (reach ≈ `V·0.675` ≈ 600u); real runways bound it further.
const SPEED_JUMP_V_CAP: f32 = 900.0;
/// Derate the ideal bhop model to attainable speed (the S-weave + a friction frame per landing).
const BHOP_EFF: f32 = 0.8;
/// Longest runway we bother measuring.
const RUNWAY_MAX: f32 = 2048.0;
/// The measured runway must reach this multiple of the jump's required entry speed.
const SJ_MARGIN: f32 = 1.15;
/// Walkable floor must continue this far past the landing (the takeoff-phase window).
const SJ_LANDING_DEPTH: f32 = 96.0;
/// At most this many speed-jump links per source cell.
const SPEED_JUMP_MAX_PER_CELL: usize = 3;

/// Airtime of a jump reaching a target `dz` above (or below) the takeoff, at gravity `g`: the
/// descending root of `JUMP_VZ·t − ½g·t² = dz`. `0` if `dz` is unreachable (above the apex).
fn jump_airtime(dz: f32, gravity: f32) -> f32 {
    let disc = JUMP_VZ * JUMP_VZ - 2.0 * gravity * dz;
    if disc < 0.0 {
        return 0.0;
    }
    (JUMP_VZ + disc.sqrt()) / gravity
}

/// The horizontal entry speed needed to clear `horiz` while rising/falling `dz`, at gravity `g`.
fn v_required(horiz: f32, dz: f32, gravity: f32) -> f32 {
    let t = jump_airtime(dz, gravity);
    if t <= 0.0 {
        f32::INFINITY
    } else {
        horiz / t
    }
}

/// Bhop speed-gain constant `k`: velocity² grows at `2k` per second while air-strafing. Derived from
/// the perpendicular air-accel cap and the tickrate (`k = tick · a² / 2`, `a = min(accel·maxspeed/tick, cap)`).
fn bhop_k(accel: f32, maxspeed: f32) -> f32 {
    let a = (accel * maxspeed / SJ_TICKRATE).min(SJ_AIR_CAP);
    SJ_TICKRATE * a * a / 2.0
}

/// Speed reached after air-strafing `len` units from `v0`: `(v0³ + 3k·len)^⅓`.
fn attainable_speed(v0: f32, len: f32, k: f32) -> f32 {
    (v0.powi(3) + 3.0 * k * len.max(0.0)).cbrt()
}

/// Runway length needed to air-strafe from `v0` up to `v`: `(v³ − v0³) / 3k`.
fn runway_len_for(v: f32, v0: f32, k: f32) -> f32 {
    ((v.powi(3) - v0.powi(3)) / (3.0 * k)).max(0.0)
}

/// Time to air-strafe from `v0` up to `v`: `(v² − v0²) / 2k`.
fn runway_time(v: f32, v0: f32, k: f32) -> f32 {
    ((v * v - v0 * v0) / (2.0 * k)).max(0.0)
}

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

// --- grid ---

/// XY sampling step. 32 = the player's full width: one column per body. Coarser than the
/// plan's 16 to keep the build cheap on big maps; thin ledges may be missed (revisit).
const GRID: f32 = 32.0;
/// Player hull half-width (the QW player box is ±16 in X/Y). Used to grow obstacles by the agent
/// radius so a bot doesn't clip geometry its path's centre-line technically clears.
const PLAYER_HALF_WIDTH: f32 = 16.0;
/// Vertical sweep step when scanning a column for floors (refined by bisection after).
const SCAN_DZ: f32 = 8.0;

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
    grid: GridIndex,
    gates: Vec<Gate>,
    /// Per-link gate tag: the index of the gate whose *closed* door the link's segment passes
    /// through, or `-1` for an ungated link. This is the "navmesh aware of dynamic geometry" core
    /// — a link (graph edge) knows which door it depends on, so pathfinding can price it by the
    /// door's live state (see [`find_path`](Self::find_path)). Empty until [`add_gates`](Self::add_gates)
    /// runs; indexed by link index (parallel to `links`).
    gated_links: Vec<i32>,
    /// Per-link hook payload index: for a [`LinkKind::Hook`] link, the index into `hooks` of its
    /// solved [`HookTraversal`]; `-1` for every non-hook link. Parallel to `links` (the same
    /// side-table pattern as `gated_links`), so the bot can look up how to fly a hook leg.
    hook_links: Vec<i32>,
    hooks: Vec<HookTraversal>,
    /// Per-link speed-jump payload index (parallel to `links`, `-1` for non-speed-jump links) — the
    /// takeoff point + required entry speed the bot executor needs.
    speed_jump_links: Vec<i32>,
    speed_jumps: Vec<SpeedJumpTraversal>,
}

/// A solved speed jump: where the takeoff ledge is and the horizontal speed needed there, so the
/// runtime can refuse to leap if the bot somehow reaches the edge too slow to clear the gap.
#[derive(Clone, Copy)]
pub struct SpeedJumpTraversal {
    pub takeoff: Vec3,
    pub v_req: f32,
}

/// Extra travel-time cost charged to a link whose gate is currently shut. Large enough that the
/// planner routes around a closed door whenever any open way exists, but finite so it still
/// crosses (and the bot then detours to the button) when there's no alternative — matching how a
/// game engine prices a disabled-but-openable NavMesh link rather than deleting it outright.
const CLOSED_GATE_PENALTY: f32 = 100_000.0;

impl NavGraph {
    /// Build the graph from a parsed BSP's player hull. Pure; safe to run at load time.
    pub fn build(bsp: &Bsp) -> NavGraph {
        let cells_grid = Self::carve_cells(bsp);
        let mut graph = NavGraph {
            adjacency: vec![Vec::new(); cells_grid.0.len()],
            cells: cells_grid.0,
            links: Vec::new(),
            grid: cells_grid.1,
            gates: Vec::new(),
            gated_links: Vec::new(),
            hook_links: Vec::new(),
            hooks: Vec::new(),
            speed_jump_links: Vec::new(),
            speed_jumps: Vec::new(),
        };
        graph.link_cells(bsp);
        graph
    }

    /// Sweep every grid column for floors and emit one [`Cell`] at the bottom of each empty
    /// span (a surface the player can rest on). Returns the cells plus their XY spatial index.
    fn carve_cells(bsp: &Bsp) -> (Vec<Cell>, GridIndex) {
        let (gx0, gy0) = (floor_grid(bsp.mins.x), floor_grid(bsp.mins.y));
        let (gx1, gy1) = (floor_grid(bsp.maxs.x), floor_grid(bsp.maxs.y));
        let mut cells = Vec::new();
        let mut grid: GridIndex = HashMap::new();

        for gx in gx0..=gx1 {
            for gy in gy0..=gy1 {
                let (x, y) = (gx as f32 * GRID, gy as f32 * GRID);
                Self::column_floors(bsp, x, y, |origin_z| {
                    let id = cells.len() as CellId;
                    cells.push(Cell {
                        origin: Vec3::new(x, y, origin_z),
                        gx,
                        gy,
                    });
                    grid.entry((gx, gy)).or_default().push(id);
                });
            }
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
        for from in 0..self.cells.len() as CellId {
            let c = self.cells[from as usize];
            for to in self.neighbors_within(c.gx, c.gy, 1) {
                if to != from {
                    if let Some(link) = self.classify_grounded(bsp, from, to) {
                        self.push_link(link);
                    }
                }
            }
            for link in self.find_jumps(bsp, from) {
                self.push_link(link);
            }
        }
    }

    fn push_link(&mut self, link: Link) {
        let idx = self.links.len() as u32;
        self.adjacency[link.from as usize].push(idx);
        self.links.push(link);
    }

    /// A grounded move (walk/step/drop) to a grid-adjacent cell, if the path is clear.
    fn classify_grounded(&self, bsp: &Bsp, from: CellId, to: CellId) -> Option<Link> {
        let (a, b) = (self.cells[from as usize], self.cells[to as usize]);
        let dz = b.origin.z - a.origin.z;
        let kind = if dz.abs() <= WALK_DZ {
            LinkKind::Walk
        } else if dz.abs() <= STEP_HEIGHT {
            LinkKind::Step
        } else if (-MAX_DROP..-STEP_HEIGHT).contains(&dz) {
            LinkKind::Drop
        } else {
            return None; // up beyond step height — needs a jump, handled separately
        };
        if !path_clear(bsp, a.origin, b.origin) {
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

    /// Jump links out of `from`: only from a **ledge edge** (the adjacent column toward the
    /// target has no walkable ground, i.e. a gap/pit), within run-jump reach and apex, with a
    /// clear arc. Deduped to the single nearest target per compass octant so a ledge sprouts a
    /// handful of jumps, not hundreds of redundant parallel ones.
    fn find_jumps(&self, bsp: &Bsp, from: CellId) -> Vec<Link> {
        let a = self.cells[from as usize];
        // best (distance, link) per compass direction bucket (3×3, center unused)
        let mut best: [Option<(f32, Link)>; 9] = Default::default();
        for to in self.neighbors_within(a.gx, a.gy, jump_grid_radius()) {
            let b = self.cells[to as usize];
            let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
            if dgx.abs() <= 1 && dgy.abs() <= 1 {
                continue; // adjacent — a grounded link if anything
            }
            let dz = b.origin.z - a.origin.z;
            if !(-MAX_DROP..=JUMP_APEX).contains(&dz) {
                continue;
            }
            let horiz = (b.origin.xy() - a.origin.xy()).length();
            if horiz > JUMP_REACH {
                continue;
            }
            // Must take off from a ledge: the column one step toward B isn't walkable ground.
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                continue;
            }
            if !arc_clear(bsp, a.origin, b.origin) {
                continue;
            }
            let oct = dir_bucket(dgx, dgy);
            if best[oct].is_none_or(|(d, _)| horiz < d) {
                best[oct] = Some((
                    horiz,
                    Link {
                        from,
                        to,
                        kind: LinkKind::JumpGap,
                        cost: link_cost(LinkKind::JumpGap, horiz, dz),
                    },
                ));
            }
        }
        best.into_iter().flatten().map(|(_, l)| l).collect()
    }

    /// Splice **double-jump** links: gaps/ledges beyond a single jump's reach but within a double
    /// jump's, gated on `rtx_doublejump`. Same ledge-edge/octant-dedup shape as [`find_jumps`], but
    /// the wider reach/apex and the taller arc-clearance envelope — and only for targets a plain
    /// jump can't already make (else a `JumpGap` covers it). The bot air-jumps mid-flight to cross.
    pub fn add_double_jumps(&mut self, bsp: &Bsp) {
        let mut pending: Vec<Link> = Vec::new();
        for from in 0..self.cells.len() as CellId {
            let a = self.cells[from as usize];
            let mut best: [Option<(f32, Link)>; 9] = Default::default();
            for to in self.neighbors_within(a.gx, a.gy, double_jump_grid_radius()) {
                if to == from {
                    continue;
                }
                let b = self.cells[to as usize];
                let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
                if dgx.abs() <= 1 && dgy.abs() <= 1 {
                    continue;
                }
                let dz = b.origin.z - a.origin.z;
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                if !(-MAX_DROP..=DOUBLE_JUMP_APEX).contains(&dz) || horiz > DOUBLE_JUMP_REACH {
                    continue;
                }
                // Only worthwhile beyond a single jump — otherwise `find_jumps` already linked it.
                if horiz <= JUMP_REACH && dz <= JUMP_APEX {
                    continue;
                }
                // Take off from a ledge edge, clear the taller arc, and don't duplicate a route the
                // static graph already provides (walk/step/jump).
                if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z)
                    || !arc_clear_peak(bsp, a.origin, b.origin, DOUBLE_ARC_PEAK, 12)
                    || self.has_direct_link(from, to)
                {
                    continue;
                }
                let oct = dir_bucket(dgx, dgy);
                if best[oct].is_none_or(|(d, _)| horiz < d) {
                    best[oct] = Some((
                        horiz,
                        Link {
                            from,
                            to,
                            kind: LinkKind::DoubleJump,
                            cost: link_cost(LinkKind::DoubleJump, horiz, dz),
                        },
                    ));
                }
            }
            pending.extend(best.into_iter().flatten().map(|(_, l)| l));
        }
        for link in pending {
            self.push_link(link);
        }
    }

    /// Splice **speed-jump** links: leaps across gaps too wide for any single/double jump, cleared by
    /// arriving at the ledge with bunnyhop-built speed. For each ledge edge, measure the straight
    /// runway feeding it, cap the attainable speed to that, and link the widest reachable targets —
    /// but with `from` set to the *runway start* so A* commits the whole run-up (the bot is thus
    /// guaranteed the speed). Only where a plain/double jump can't already make it. Gated on bhop.
    pub fn add_speed_jumps(&mut self, bsp: &Bsp, params: SpeedJumpParams, double_jump: bool) {
        let k = bhop_k(params.accel, params.maxspeed);
        let mut pending: Vec<(Link, SpeedJumpTraversal)> = Vec::new();
        for ledge in 0..self.cells.len() as CellId {
            self.solve_speed_jumps_from(bsp, ledge, params, k, double_jump, &mut pending);
        }
        for (link, tr) in pending {
            self.push_speed_jump(link, tr);
        }
    }

    /// The speed-jump links leaving ledge cell `ledge` (the takeoff), appended to `out`.
    fn solve_speed_jumps_from(
        &self,
        bsp: &Bsp,
        ledge: CellId,
        params: SpeedJumpParams,
        k: f32,
        double_jump: bool,
        out: &mut Vec<(Link, SpeedJumpTraversal)>,
    ) {
        let a = self.cells[ledge as usize];
        let mut cands: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new(); // (v_req, link, traversal)
        for (dgx, dgy) in COMPASS {
            // Take off from a ledge edge, and only where a straight runway can feed the jump.
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                continue;
            }
            let runway = self.measure_runway(bsp, &a, dgx, dgy);
            let v_max = SPEED_JUMP_V_CAP.min(BHOP_EFF * attainable_speed(MAX_SPEED, runway, k));
            if v_max * jump_airtime(0.0, params.gravity) <= JUMP_REACH + 1.0 {
                continue; // this runway buys nothing past a normal jump
            }
            let reach_cap = v_max * jump_airtime(-MAX_DROP, params.gravity);
            let scan = ((reach_cap / GRID).ceil() as i32).max(1);
            let mut best: Option<(f32, Link, SpeedJumpTraversal)> = None;
            for to in self.neighbors_within(a.gx, a.gy, scan) {
                if to == ledge {
                    continue;
                }
                let b = self.cells[to as usize];
                let (bgx, bgy) = (b.gx - a.gx, b.gy - a.gy);
                if (bgx.abs() <= 1 && bgy.abs() <= 1) || dir_bucket(bgx, bgy) != dir_bucket(dgx, dgy) {
                    continue;
                }
                let dz = b.origin.z - a.origin.z;
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                if !(-MAX_DROP..=JUMP_APEX).contains(&dz) || horiz <= JUMP_REACH {
                    continue;
                }
                // Skip what a double jump already covers (when enabled), and any existing direct link.
                if (double_jump && horiz <= DOUBLE_JUMP_REACH && dz <= DOUBLE_JUMP_APEX)
                    || self.has_direct_link(ledge, to)
                {
                    continue;
                }
                let airtime = jump_airtime(dz, params.gravity);
                let v_req = v_required(horiz, dz, params.gravity);
                if airtime <= 0.0 || v_req * SJ_MARGIN > v_max {
                    continue;
                }
                // Flat-long arc clearance, a landing with room to slide out at speed, and a
                // runway-start cell to anchor from.
                let steps = ((horiz / 24.0).ceil() as i32).max(8);
                let depth_cols = (SJ_LANDING_DEPTH / GRID).ceil() as i32;
                let landing_ok = (1..=depth_cols)
                    .all(|i| self.has_ground_near(b.gx + dgx.signum() * i, b.gy + dgy.signum() * i, b.origin.z));
                if !arc_clear_peak(bsp, a.origin, b.origin, JUMP_APEX, steps) || !landing_ok {
                    continue;
                }
                let need = runway_len_for(v_req * SJ_MARGIN, MAX_SPEED, k);
                let dir = Vec3::new(dgx.signum() as f32, dgy.signum() as f32, 0.0).normalize_or_zero();
                let Some(start) = self.nearest_within(a.origin - dir * need, GRID * 1.5, STEP_HEIGHT * 3.0) else {
                    continue;
                };
                if start == to {
                    continue;
                }
                let cost = runway_time(v_req * SJ_MARGIN, MAX_SPEED, k) + airtime + 1.0;
                let link = Link {
                    from: start,
                    to,
                    kind: LinkKind::SpeedJump,
                    cost,
                };
                let tr = SpeedJumpTraversal {
                    takeoff: a.origin,
                    v_req,
                };
                if best.is_none_or(|(bv, _, _)| v_req < bv) {
                    best = Some((v_req, link, tr));
                }
            }
            if let Some(c) = best {
                cands.push(c);
            }
        }
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands.truncate(SPEED_JUMP_MAX_PER_CELL);
        out.extend(cands.into_iter().map(|(_, l, t)| (l, t)));
    }

    /// Measure the straight, flat, hop-wide runway feeding ledge cell `a` from behind (opposite the
    /// jump direction): walk grid columns back while each has a cell within `STEP_HEIGHT`, hop
    /// headroom, and ground in both perpendicular columns (so the air-strafe weave stays on floor).
    fn measure_runway(&self, bsp: &Bsp, a: &Cell, dgx: i32, dgy: i32) -> f32 {
        let (bx, by) = (-dgx.signum(), -dgy.signum());
        if bx == 0 && by == 0 {
            return 0.0;
        }
        let step_len = GRID * (((bx * bx + by * by) as f32).sqrt());
        let (px, py) = (-by, bx); // perpendicular grid direction
        let (mut gx, mut gy, mut z, mut len) = (a.gx, a.gy, a.origin.z, 0.0);
        while len < RUNWAY_MAX {
            let (ngx, ngy) = (gx + bx, gy + by);
            let Some(cid) = self.cell_near(ngx, ngy, z) else {
                break;
            };
            let c = self.cells[cid as usize].origin;
            if bsp.is_solid(c + Vec3::new(0.0, 0.0, JUMP_APEX))
                || self.cell_near(ngx + px, ngy + py, c.z).is_none()
                || self.cell_near(ngx - px, ngy - py, c.z).is_none()
            {
                break;
            }
            len += step_len;
            (gx, gy, z) = (ngx, ngy, c.z);
        }
        len
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

    /// Cell whose origin is nearest `pos` (searches outward from `pos`'s grid column). `None`
    /// if the graph is empty or nothing is found within a few columns.
    pub fn nearest(&self, pos: Vec3) -> Option<CellId> {
        let (gx, gy) = (floor_grid(pos.x), floor_grid(pos.y));
        let mut best: Option<(CellId, f32)> = None;
        for radius in 0..=4 {
            for id in self.neighbors_within(gx, gy, radius) {
                let d = (self.cells[id as usize].origin - pos).length_squared();
                if best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((id, d));
                }
            }
            if best.is_some() && radius >= 1 {
                break;
            }
        }
        best.map(|(id, _)| id)
    }

    /// A\* over the graph from `start` to `goal`, returning the route as a sequence of link
    /// indices (each link's `to` is the next cell, its `kind` how to get there). `None` if no
    /// route exists (different connected components). Heuristic = straight-line travel time, an
    /// admissible lower bound on cost, so the path is optimal.
    /// A* from `start` to `goal`. `gate_closed[i]` marks gate `i`'s door as currently shut; a link
    /// through a shut gate is charged [`CLOSED_GATE_PENALTY`], so the route bends around closed
    /// doors when it can and only crosses one (leaving the bot to open it) when there's no other
    /// way. Pass `&[]` to treat every door as open.
    pub fn find_path(&self, start: CellId, goal: CellId, gate_closed: &[bool]) -> Option<Vec<u32>> {
        use std::cmp::Ordering;
        use std::collections::BinaryHeap;

        if start == goal {
            return Some(Vec::new());
        }
        let h = |c: CellId| (self.cells[goal as usize].origin - self.cells[c as usize].origin).length() / MAX_SPEED;

        // Min-heap on f = g + h (Reverse via a custom ordering on a NaN-free f32 key).
        struct Node {
            f: f32,
            cell: CellId,
        }
        impl PartialEq for Node {
            fn eq(&self, o: &Self) -> bool {
                self.f == o.f
            }
        }
        impl Eq for Node {}
        impl PartialOrd for Node {
            fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
                Some(self.cmp(o))
            }
        }
        impl Ord for Node {
            fn cmp(&self, o: &Self) -> Ordering {
                // Reverse so BinaryHeap (a max-heap) pops the smallest f first.
                o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
            }
        }

        let n = self.cells.len();
        let mut g_cost = vec![f32::INFINITY; n];
        let mut came_from = vec![u32::MAX; n]; // link index used to reach this cell
        let mut heap = BinaryHeap::new();
        g_cost[start as usize] = 0.0;
        heap.push(Node {
            f: h(start),
            cell: start,
        });

        while let Some(Node { cell, .. }) = heap.pop() {
            if cell == goal {
                return Some(self.reconstruct(&came_from, start, goal));
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                let ng = g_cost[cell as usize] + link.cost + self.gate_penalty(li, gate_closed);
                if ng < g_cost[link.to as usize] {
                    g_cost[link.to as usize] = ng;
                    came_from[link.to as usize] = li;
                    heap.push(Node {
                        f: ng + h(link.to),
                        cell: link.to,
                    });
                }
            }
        }
        None
    }

    /// The reachable cell (per current door states) whose origin is closest to `goal`'s, when
    /// `goal` itself can't be reached. Lets a bot head as far toward an unreachable target as the
    /// graph allows — approaching a wall/door/connection to get line of sight — instead of homing
    /// straight into geometry. `None` only if nothing but `start` is reachable.
    pub fn nearest_reachable_to(&self, start: CellId, goal: CellId, gate_closed: &[bool]) -> Option<CellId> {
        let costs = self.costs_from(start, gate_closed);
        let goal_pos = self.cells[goal as usize].origin;
        (0..self.cells.len() as CellId)
            .filter(|&c| c != start && costs[c as usize].is_finite())
            .min_by(|&a, &b| {
                let d = |c: CellId| (self.cells[c as usize].origin - goal_pos).length_squared();
                d(a).total_cmp(&d(b))
            })
    }

    /// Dijkstra cost-flood from `start`: the travel-time cost to reach every cell (`INFINITY`
    /// for unreachable ones). One pass answers "how far is each item?" for goal selection, far
    /// cheaper than an A* per candidate. Indexed by [`CellId`].
    pub fn costs_from(&self, start: CellId, gate_closed: &[bool]) -> Vec<f32> {
        use std::cmp::Ordering;
        use std::collections::BinaryHeap;

        struct Node {
            g: f32,
            cell: CellId,
        }
        impl PartialEq for Node {
            fn eq(&self, o: &Self) -> bool {
                self.g == o.g
            }
        }
        impl Eq for Node {}
        impl PartialOrd for Node {
            fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
                Some(self.cmp(o))
            }
        }
        impl Ord for Node {
            fn cmp(&self, o: &Self) -> Ordering {
                o.g.partial_cmp(&self.g).unwrap_or(Ordering::Equal) // min-heap on g
            }
        }

        let mut cost = vec![f32::INFINITY; self.cells.len()];
        let mut heap = BinaryHeap::new();
        cost[start as usize] = 0.0;
        heap.push(Node { g: 0.0, cell: start });
        while let Some(Node { g, cell }) = heap.pop() {
            if g > cost[cell as usize] {
                continue; // a cheaper path already settled this cell
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                let ng = g + link.cost + self.gate_penalty(li, gate_closed);
                if ng < cost[link.to as usize] {
                    cost[link.to as usize] = ng;
                    heap.push(Node { g: ng, cell: link.to });
                }
            }
        }
        cost
    }

    /// Walk `came_from` link indices back from `goal` to `start` into a forward link route.
    fn reconstruct(&self, came_from: &[u32], start: CellId, goal: CellId) -> Vec<u32> {
        let mut route = Vec::new();
        let mut cell = goal;
        while cell != start {
            let li = came_from[cell as usize];
            route.push(li);
            cell = self.links[li as usize].from;
        }
        route.reverse();
        route
    }

    /// The cell a link points at (its destination).
    pub fn link_target(&self, link_idx: u32) -> CellId {
        self.links[link_idx as usize].to
    }

    /// The cell a link departs from.
    pub fn link_source(&self, link_idx: u32) -> CellId {
        self.links[link_idx as usize].from
    }

    /// How a link is traversed (walk/step/drop/jump).
    pub fn link_kind(&self, link_idx: u32) -> LinkKind {
        self.links[link_idx as usize].kind
    }

    /// The standing player-origin position of a cell (the point a bot steers toward).
    pub fn cell_origin(&self, cell: CellId) -> Vec3 {
        self.cells[cell as usize].origin
    }

    /// Counts per link kind, for the load-time debug line.
    pub fn summary(&self) -> LinkCounts {
        let mut c = LinkCounts::default();
        for l in &self.links {
            match l.kind {
                LinkKind::Walk => c.walk += 1,
                LinkKind::Step => c.step += 1,
                LinkKind::Drop => c.drop += 1,
                LinkKind::JumpGap => c.jump += 1,
                LinkKind::DoubleJump => c.double_jump += 1,
                LinkKind::SpeedJump => c.speed_jump += 1,
                LinkKind::Plat => c.plat += 1,
                LinkKind::Teleport => c.teleport += 1,
                LinkKind::Hook => c.hook += 1,
            }
        }
        c
    }

    // --- entity-derived links: func_plat (built after the static graph, from spawned ents) ---

    /// Splice `func_plat` lifts into the graph. For each plat we add a cell on its surface at
    /// the bottom (the board point), a [`LinkKind::Plat`] ride from there to the floor the plat
    /// delivers to at the top, and `JumpGap` "jump aboard" links from the nearby lower floor
    /// onto the plat — boarding by jumping is safer because the trigger that raises the plat is
    /// larger than the plat brush. Plats whose top doesn't reach any floor cell are skipped.
    pub fn add_plats(&mut self, bsp: &Bsp, plats: &[PlatInfo]) {
        for p in plats {
            // Where does the plat deliver you? Nearest floor cell to its raised surface.
            let Some(top) = self.nearest_within(p.exit, GRID * 3.0, STEP_HEIGHT * 2.0) else {
                continue;
            };
            let board = self.add_cell(p.board);
            let ride = (p.exit.z - p.board.z).max(0.0);
            self.push_link(Link {
                from: board,
                to: top,
                kind: LinkKind::Plat,
                cost: ride / MAX_SPEED + 1.0, // ride time + boarding/trigger overhead
            });
            // Jump-aboard links from the surrounding lower floor.
            for c in self.cells_near(p.board.xy(), GRID * 3.0) {
                if c == board {
                    continue;
                }
                let from = self.cells[c as usize].origin;
                let dz = p.board.z - from.z;
                if dz.abs() <= JUMP_APEX && arc_clear(bsp, from, p.board) {
                    let horiz = (p.board.xy() - from.xy()).length();
                    self.push_link(Link {
                        from: c,
                        to: board,
                        kind: LinkKind::JumpGap,
                        cost: link_cost(LinkKind::JumpGap, horiz, dz),
                    });
                }
            }
        }
    }

    /// Splice `trigger_teleport`s into the graph: every standable cell inside a teleporter's
    /// trigger box gets a [`LinkKind::Teleport`] link to the cell at its destination. The bot
    /// needs no special handling — routing onto an entrance cell walks it into the trigger and
    /// the engine warps it; a separate displacement check then re-paths from the landing spot.
    /// Teleporters whose destination doesn't reach any floor cell are skipped.
    pub fn add_teleports(&mut self, teles: &[TeleportInfo]) {
        for t in teles {
            let Some(dest) = self.nearest_within(t.dest, GRID * 3.0, 96.0) else {
                continue;
            };
            // Entrance cells: those whose footprint sits within the trigger box (loosened in Z
            // so a floor cell standing in a doorway-tall trigger still counts).
            let lo = Vec3::new(t.tmin.x, t.tmin.y, t.tmin.z - 32.0);
            let hi = Vec3::new(t.tmax.x, t.tmax.y, t.tmax.z + 24.0);
            for c in self.cells_in_box(lo, hi) {
                if c != dest {
                    self.push_link(Link {
                        from: c,
                        to: dest,
                        kind: LinkKind::Teleport,
                        cost: 0.2,
                    });
                }
            }
        }
    }

    /// Cells whose origin lies within the axis-aligned box `[min, max]`.
    fn cells_in_box(&self, min: Vec3, max: Vec3) -> Vec<CellId> {
        let mut out = Vec::new();
        for gx in floor_grid(min.x)..=floor_grid(max.x) {
            for gy in floor_grid(min.y)..=floor_grid(max.y) {
                if let Some(ids) = self.grid.get(&(gx, gy)) {
                    for &c in ids {
                        let o = self.cells[c as usize].origin;
                        if (min.x..=max.x).contains(&o.x)
                            && (min.y..=max.y).contains(&o.y)
                            && (min.z..=max.z).contains(&o.z)
                        {
                            out.push(c);
                        }
                    }
                }
            }
        }
        out
    }

    // --- entity-derived: button-gated doors ---

    /// Register button-gated doors. Each `func_door` with a targetname is a gate that stays shut
    /// until its `func_button` fires it; the static carve (hull 0, no door brushes) has links
    /// running straight through, so we tag every link whose *segment* passes through the door's
    /// *closed* volume with that gate and remember which button opens it. Tagging links (not cells)
    /// is what makes this robust for thin pillars — a link crossing a 14-unit door is caught even
    /// when no cell centre lands inside it. Pathfinding then prices those links by door state
    /// (see [`find_path`](Self::find_path)); bots detour to the button when a route must cross a
    /// shut one (see `bot.rs`). Gates whose closed door crosses no link, or whose button has no
    /// nearby cell to operate from, are skipped.
    pub fn add_gates(&mut self, gates: &[GateInfo]) {
        if self.gated_links.len() != self.links.len() {
            self.gated_links = vec![-1; self.links.len()];
        }
        for gi in gates {
            let Some(button_cell) = self.nearest_within(gi.button, GRID * 5.0, 160.0) else {
                continue;
            };
            // Inflate the door box by the player's horizontal half-width before testing links: a
            // link whose centre-line passes just *beside* the door still can't be walked (the
            // player's 32-wide body clips it), so it must be gated too — otherwise a bot takes the
            // "around" route onto that link and wedges against the pillar. This is the standard
            // navmesh trick of growing obstacles by the agent radius.
            let margin = Vec3::new(PLAYER_HALF_WIDTH, PLAYER_HALF_WIDTH, 0.0);
            let (lo, hi) = (gi.closed_min - margin, gi.closed_max + margin);
            let hit: Vec<usize> = (0..self.links.len())
                .filter(|&li| {
                    let link = self.links[li];
                    let p0 = self.cells[link.from as usize].origin;
                    let p1 = self.cells[link.to as usize].origin;
                    segment_aabb_intersect(p0, p1, lo, hi)
                })
                .collect();
            if hit.is_empty() {
                continue; // door crosses no link — not an obstruction the bots can hit
            }
            let idx = self.gates.len() as i32;
            for li in hit {
                self.gated_links[li] = idx;
            }
            self.gates.push(Gate {
                obstruction: gi.obstruction,
                closed_origin: gi.closed_origin,
                activator: gi.activator,
                button_cell,
                aim: gi.button,
                shoot: gi.shoot,
            });
        }
    }

    pub fn gate_count(&self) -> usize {
        self.gates.len()
    }

    pub fn gate(&self, i: usize) -> &Gate {
        &self.gates[i]
    }

    /// The gate (if any) whose shut door link `li` passes through.
    pub fn gate_of_link(&self, li: u32) -> Option<usize> {
        match self.gated_links.get(li as usize).copied().unwrap_or(-1) {
            g if g >= 0 => Some(g as usize),
            _ => None,
        }
    }

    /// Extra A* cost for link `li` given the current door states — [`CLOSED_GATE_PENALTY`] if its
    /// gate is shut, else nothing.
    #[inline]
    fn gate_penalty(&self, li: u32, gate_closed: &[bool]) -> f32 {
        match self.gate_of_link(li) {
            Some(g) if gate_closed.get(g).copied().unwrap_or(false) => CLOSED_GATE_PENALTY,
            _ => 0.0,
        }
    }

    // --- entity-independent: grappling-hook swing links ---

    /// Splice grappling-hook swing links into the graph. From each **ledge-edge** cell, fire probe
    /// rays out over the drop-off at a few pitches, find where the hook would anchor, then sample
    /// release points along the reel and simulate the resulting gravity parabola — whatever standable
    /// cell the arc lands on becomes the link's target. This discovers both vertical pull-ups and
    /// long horizontal flings from one mechanism. Only accepted when the arc (and perturbed variants)
    /// land safely, so a bot is never flung into a pit. Deduped per direction/elevation and capped.
    pub fn add_hooks(&mut self, bsp: &Bsp, params: HookParams) {
        if self.hook_links.len() != self.links.len() {
            self.hook_links.resize(self.links.len(), -1);
        }
        // Solve per source cell first (immutable borrow), then splice (push_hook needs `&mut`).
        let mut pending: Vec<(Link, HookTraversal)> = Vec::new();
        for from in 0..self.cells.len() as CellId {
            self.solve_hooks_from(bsp, from, params, &mut pending);
        }
        for (link, tr) in pending {
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
                let Some(stick) = march_to_solid(bsp, launch, dir, HOOK_ROPE_MAX) else {
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

        // Keep the cheapest few per cell.
        let mut chosen: Vec<(f32, Link, HookTraversal)> = best.into_values().collect();
        chosen.sort_by(|x, y| x.0.total_cmp(&y.0));
        chosen.truncate(HOOK_MAX_PER_CELL);
        out.extend(chosen.into_iter().map(|(_, link, tr)| (link, tr)));
    }

    /// Whether `from` already has a direct (non-hook) link to `to` — such a target needs no hook.
    fn has_direct_link(&self, from: CellId, to: CellId) -> bool {
        self.adjacency[from as usize]
            .iter()
            .any(|&li| self.links[li as usize].to == to)
    }

    /// The solved traversal for hook link `li`, or `None` for a non-hook link.
    pub fn hook_of_link(&self, li: u32) -> Option<&HookTraversal> {
        match self.hook_links.get(li as usize).copied().unwrap_or(-1) {
            h if h >= 0 => self.hooks.get(h as usize),
            _ => None,
        }
    }

    /// Push a hook link with its solved traversal, keeping the `hook_links` side table in step.
    fn push_hook(&mut self, link: Link, traversal: HookTraversal) {
        if self.hook_links.len() != self.links.len() {
            self.hook_links.resize(self.links.len(), -1);
        }
        let h = self.hooks.len() as i32;
        self.hooks.push(traversal);
        self.push_link(link);
        self.hook_links.push(h);
    }

    /// The solved traversal for speed-jump link `li`, or `None` for any other link.
    pub fn speed_jump_of_link(&self, li: u32) -> Option<&SpeedJumpTraversal> {
        match self.speed_jump_links.get(li as usize).copied().unwrap_or(-1) {
            s if s >= 0 => self.speed_jumps.get(s as usize),
            _ => None,
        }
    }

    /// Push a speed-jump link with its traversal, keeping the side table in step.
    fn push_speed_jump(&mut self, link: Link, traversal: SpeedJumpTraversal) {
        if self.speed_jump_links.len() != self.links.len() {
            self.speed_jump_links.resize(self.links.len(), -1);
        }
        let s = self.speed_jumps.len() as i32;
        self.speed_jumps.push(traversal);
        self.push_link(link);
        self.speed_jump_links.push(s);
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

/// The two standing positions a `func_plat` connects: the player-origin spot on the plat
/// surface at the bottom of travel (`board`) and at the top (`exit`).
pub struct PlatInfo {
    pub board: Vec3,
    pub exit: Vec3,
}

/// A `trigger_teleport`: its world-space trigger box (`tmin`/`tmax`) and the player-origin
/// arrival point at its destination (`dest`).
pub struct TeleportInfo {
    pub tmin: Vec3,
    pub tmax: Vec3,
    pub dest: Vec3,
}

/// A button-gated obstruction (a sliding `func_door` or a rotating `func_movewall`): the
/// obstructing entity (to read its current position), where it sits while blocking
/// (`closed_origin` — it's "open" once moved from here), where the bot operates the button from
/// (`button_cell`), the button centre to face/touch/shoot (`aim`), and whether it's shot.
pub struct Gate {
    pub obstruction: u32,
    pub closed_origin: Vec3,
    /// The activator entity (button or shootable trigger), to read its cooldown/`takedamage`
    /// state — a re-triggerable activator goes dead for a while after each use.
    pub activator: u32,
    pub button_cell: CellId,
    pub aim: Vec3,
    pub shoot: bool,
}

/// Inputs for [`NavGraph::add_gates`], gathered from spawned obstruction/activator entities: the
/// obstruction's closed-position origin and world box, the activator entity + its centre, and
/// whether it's shot rather than touched.
pub struct GateInfo {
    pub obstruction: u32,
    pub closed_origin: Vec3,
    pub closed_min: Vec3,
    pub closed_max: Vec3,
    pub activator: u32,
    pub button: Vec3,
    pub shoot: bool,
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
}

/// Travel-time cost of a link: horizontal distance / speed, plus risk/effort penalties so A*
/// prefers grounded routes and avoids damaging falls.
fn link_cost(kind: LinkKind, horiz: f32, dz: f32) -> f32 {
    let base = horiz.max(GRID) / MAX_SPEED;
    match kind {
        LinkKind::Walk => base,
        LinkKind::Step => base * 1.1,
        LinkKind::Drop => base + if -dz > SAFE_FALL { 1.0 } else { 0.1 },
        LinkKind::JumpGap => base + 0.3,
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
    }
}

/// Bisect the floor origin height between a solid sample below and an empty one above.
fn bisect_floor(bsp: &Bsp, x: f32, y: f32, z_solid: f32, z_empty: f32) -> f32 {
    let (mut lo, mut hi) = (z_solid, z_empty);
    for _ in 0..8 {
        let mid = (lo + hi) * 0.5;
        if bsp.is_solid(Vec3::new(x, y, mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// Whether the straight segment between two standing origins is free of solid (sampled at the
/// higher origin so a wall or low ceiling between the cells blocks the move).
fn path_clear(bsp: &Bsp, a: Vec3, b: Vec3) -> bool {
    let z = a.z.max(b.z);
    let steps = ((b.xy() - a.xy()).length() / 16.0).ceil().max(1.0) as i32;
    (0..=steps).all(|i| {
        let t = i as f32 / steps as f32;
        let p = a.lerp(b, t);
        !bsp.is_solid(Vec3::new(p.x, p.y, z))
    })
}

/// Whether a jump arc from `a` to `b` clears geometry: sample a parabola peaking `JUMP_APEX`
/// above the higher endpoint and require every point to be open.
fn arc_clear(bsp: &Bsp, a: Vec3, b: Vec3) -> bool {
    arc_clear_peak(bsp, a, b, JUMP_APEX, 8)
}

/// [`arc_clear`] with a caller-chosen apex height (for the taller double-jump arc) and step count.
fn arc_clear_peak(bsp: &Bsp, a: Vec3, b: Vec3, apex: f32, steps: i32) -> bool {
    let peak = a.z.max(b.z) + apex;
    (0..=steps).all(|i| {
        let t = i as f32 / steps as f32;
        let xy = a.xy().lerp(b.xy(), t);
        // Parabola through a.z (t=0) and b.z (t=1) peaking at `peak`.
        let z = a.z + (b.z - a.z) * t + 4.0 * (peak - a.z.max(b.z)) * t * (1.0 - t);
        !bsp.is_solid(Vec3::new(xy.x, xy.y, z))
    })
}

/// Grid column index for a world coordinate.
fn floor_grid(v: f32) -> i32 {
    (v / GRID).floor() as i32
}

/// The eight compass grid directions (used to find hook launch edges).
const COMPASS: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

/// Launch pitches (degrees above horizontal) tried when searching for a hook anchor.
const HOOK_PITCHES: [f32; 4] = [20.0, 40.0, 60.0, 80.0];

/// Outcome of flying a release velocity from a point under gravity against a solidity oracle.
enum ArcResult {
    /// The parabola descended onto solid: the standing position just above it, airtime, and the
    /// vertical speed at impact (for fall-damage pricing).
    Land { pos: Vec3, airtime: f32, vz: f32 },
    /// Ran into solid while level or ascending — a wall/ceiling blocks this arc.
    Blocked,
    /// Never landed within the airtime cap.
    Timeout,
}

/// Integrate a ballistic arc from `r` with initial velocity `v0` under `gravity`, stepping so no
/// step advances more than `HOOK_SAMPLE`, until it hits solid (landing if descending, blocked
/// otherwise) or the airtime cap. Pure: the world enters only through the `is_solid` oracle, so
/// this is unit-testable against the closed-form parabola with a synthetic floor.
fn simulate_arc(is_solid: impl Fn(Vec3) -> bool, r: Vec3, v0: Vec3, gravity: f32) -> ArcResult {
    let mut p = r;
    let mut v = v0;
    let mut t = 0.0;
    while t < HOOK_MAX_AIRTIME {
        let dt = (HOOK_SAMPLE / v.length().max(1.0)).min(HOOK_SIM_DT);
        let next = p + v * dt;
        if is_solid(next) {
            return if v.z < 0.0 {
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
pub(crate) fn arc_land(bsp: &Bsp, r: Vec3, v0: Vec3, gravity: f32) -> Option<(Vec3, f32, f32)> {
    match simulate_arc(|p| bsp.is_solid(p), r, v0, gravity) {
        ArcResult::Land { pos, airtime, vz } => Some((pos, airtime, vz)),
        _ => None,
    }
}

/// March a ray from `from` along unit `dir` until it strikes solid, returning the last empty point
/// (the surface the hook would stick to), or `None` within `max`. Bisected for a tight surface.
fn march_to_solid(bsp: &Bsp, from: Vec3, dir: Vec3, max: f32) -> Option<Vec3> {
    let mut d = HOOK_SAMPLE;
    while d <= max {
        if bsp.is_solid(from + dir * d) {
            let (mut lo, mut hi) = (d - HOOK_SAMPLE, d);
            for _ in 0..4 {
                let mid = (lo + hi) * 0.5;
                if bsp.is_solid(from + dir * mid) {
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
fn perturb_ok(bsp: &Bsp, stick: Vec3, rdir: Vec3, release_dist: f32, rope: f32, params: HookParams, b: Vec3) -> bool {
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
fn hook_cost(rope: f32, release_dist: f32, airtime: f32, vz_land: f32, params: HookParams) -> f32 {
    let throw = rope / params.throw;
    let reel = (rope - release_dist).max(0.0) / params.pull;
    let mut c = throw + reel + airtime + HOOK_OVERHEAD;
    if vz_land.abs() > 580.0 {
        c += 1.0;
    }
    c
}

/// Whether the segment `p0`→`p1` intersects the axis-aligned box `[min, max]` (slab method).
/// Used to decide which navmesh links a closed door's volume blocks.
fn segment_aabb_intersect(p0: Vec3, p1: Vec3, min: Vec3, max: Vec3) -> bool {
    let (o, d) = (p0.to_array(), (p1 - p0).to_array());
    let (lo, hi) = (min.to_array(), max.to_array());
    let (mut tmin, mut tmax) = (0.0f32, 1.0f32);
    for i in 0..3 {
        if d[i].abs() < 1e-6 {
            if o[i] < lo[i] || o[i] > hi[i] {
                return false; // parallel to this slab and outside it
            }
        } else {
            let inv = 1.0 / d[i];
            let mut t0 = (lo[i] - o[i]) * inv;
            let mut t1 = (hi[i] - o[i]) * inv;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            tmin = tmin.max(t0);
            tmax = tmax.min(t1);
            if tmin > tmax {
                return false;
            }
        }
    }
    true
}

/// How many grid columns a jump can span.
fn jump_grid_radius() -> i32 {
    (JUMP_REACH / GRID).ceil() as i32
}

/// How many grid columns a double jump can span.
fn double_jump_grid_radius() -> i32 {
    (DOUBLE_JUMP_REACH / GRID).ceil() as i32
}

/// Bucket a grid direction into a 3×3 compass cell (0..9, center index 4 unused), for jump
/// dedup. Distinct for all 8 surrounding directions — opposite directions never collide.
fn dir_bucket(dgx: i32, dgy: i32) -> usize {
    ((dgx.signum() + 1) + (dgy.signum() + 1) * 3) as usize
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
    /// fresh at selection time (see [`crate::bot_goals`]).
    pub goals: Vec<(u32, CellId)>,
}

/// Build a navmesh off the main thread from pre-gathered, `Send` inputs: the raw BSP bytes plus the
/// entity-derived plat/teleport/gate info. Pure — no engine or game-state access — so it runs
/// safely on a worker thread whose result the main thread swaps in when ready.
pub fn build_navmesh(
    bytes: Vec<u8>,
    plats: Vec<PlatInfo>,
    teleports: Vec<TeleportInfo>,
    gates: Vec<GateInfo>,
    hooks: Option<HookParams>,
    double_jump: bool,
    speed_jump: Option<SpeedJumpParams>,
) -> NavBuild {
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
    graph.add_plats(&bsp, &plats);
    graph.add_teleports(&teleports);
    graph.add_gates(&gates);
    Some((bsp, graph))
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
            .find_path(start, goal, &[])
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
        g.add_plats(&bsp, &[PlatInfo { board, exit }]);
        assert_eq!(g.summary().plat, 1, "plat ride not added");
        assert_eq!(g.cells.len(), cells_before + 1, "board cell not added");
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
        assert!(g.find_path(start, goal, &[true]).is_some(), "no route with gate shut");
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
                horiz <= DOUBLE_JUMP_REACH && (-MAX_DROP..=DOUBLE_JUMP_APEX).contains(&dz),
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
            // The from-cell is the runway start: at least the runway needed to build the *extra*
            // speed over maxspeed (a gap crossable at ≤ maxspeed needs no runway → from = ledge).
            let need = runway_len_for(tr.v_req.max(MAX_SPEED), MAX_SPEED, k);
            let back = (start.xy() - tr.takeoff.xy()).length();
            assert!(back + GRID >= need, "runway too short: {back} < {need}");
        }
        eprintln!("speed-jump splice: {sjumps} links");
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

        // The build-time model, derated, is conservative vs the actual bhop controller: simulate a
        // real air-strafe over the runway and confirm it reaches at least the planned speed.
        use crate::bot_bhop::{air_accel_max, apply_airaccel, strafe, wishdir_of};
        let dt = 1.0 / 72.0;
        let a_max = air_accel_max(10.0, MAX_SPEED, dt);
        let steps = (800.0 / (MAX_SPEED * dt)) as i32; // ~time to cover the runway, air frames only
        let mut vel = glam::Vec2::new(MAX_SPEED, 0.0);
        let mut sigma = 0.0;
        for _ in 0..steps {
            let s = strafe(vel, 0.0, sigma, a_max);
            sigma = s.sigma;
            vel = apply_airaccel(vel, wishdir_of(s.view_yaw, s.side), MAX_SPEED, 10.0, dt);
        }
        let planned = BHOP_EFF * attainable_speed(MAX_SPEED, 800.0, k);
        assert!(
            vel.length() >= planned,
            "controller {} slower than planned {planned}",
            vel.length()
        );
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
}
