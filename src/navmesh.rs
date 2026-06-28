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
/// `sv_maxspeed` default — the cost denominator (travel time = distance / speed).
const MAX_SPEED: f32 = 320.0;

// --- grid ---

/// XY sampling step. 32 = the player's full width: one column per body. Coarser than the
/// plan's 16 to keep the build cheap on big maps; thin ledges may be missed (revisit).
const GRID: f32 = 32.0;
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
    /// Riding a `func_plat`: board it at the bottom and let it carry you to the top. The
    /// link's `from` cell is the standing spot on the plat (its centre), `to` the floor the
    /// plat delivers to. Bots stay centred and wait rather than steering off.
    Plat,
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
}

impl NavGraph {
    /// Build the graph from a parsed BSP's player hull. Pure; safe to run at load time.
    pub fn build(bsp: &Bsp) -> NavGraph {
        let cells_grid = Self::carve_cells(bsp);
        let mut graph = NavGraph {
            adjacency: vec![Vec::new(); cells_grid.0.len()],
            cells: cells_grid.0,
            links: Vec::new(),
            grid: cells_grid.1,
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
                    cells.push(Cell { origin: Vec3::new(x, y, origin_z), gx, gy });
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
        Some(Link { from, to, kind, cost: link_cost(kind, horiz, dz) })
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
                best[oct] = Some((horiz, Link {
                    from,
                    to,
                    kind: LinkKind::JumpGap,
                    cost: link_cost(LinkKind::JumpGap, horiz, dz),
                }));
            }
        }
        best.into_iter().flatten().map(|(_, l)| l).collect()
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
    pub fn find_path(&self, start: CellId, goal: CellId) -> Option<Vec<u32>> {
        use std::cmp::Ordering;
        use std::collections::BinaryHeap;

        if start == goal {
            return Some(Vec::new());
        }
        let h = |c: CellId| {
            (self.cells[goal as usize].origin - self.cells[c as usize].origin).length() / MAX_SPEED
        };

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
        heap.push(Node { f: h(start), cell: start });

        while let Some(Node { cell, .. }) = heap.pop() {
            if cell == goal {
                return Some(self.reconstruct(&came_from, start, goal));
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                let ng = g_cost[cell as usize] + link.cost;
                if ng < g_cost[link.to as usize] {
                    g_cost[link.to as usize] = ng;
                    came_from[link.to as usize] = li;
                    heap.push(Node { f: ng + h(link.to), cell: link.to });
                }
            }
        }
        None
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
                LinkKind::Plat => c.plat += 1,
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

/// The two standing positions a `func_plat` connects: the player-origin spot on the plat
/// surface at the bottom of travel (`board`) and at the top (`exit`).
pub struct PlatInfo {
    pub board: Vec3,
    pub exit: Vec3,
}

#[derive(Default)]
pub struct LinkCounts {
    pub walk: u32,
    pub step: u32,
    pub drop: u32,
    pub jump: u32,
    pub plat: u32,
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
        // Plat costs are computed at splice time (ride time + overhead), not here.
        LinkKind::Plat => base + 1.0,
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
    let peak = a.z.max(b.z) + JUMP_APEX;
    let steps = 8;
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

/// How many grid columns a jump can span.
fn jump_grid_radius() -> i32 {
    (JUMP_REACH / GRID).ceil() as i32
}

/// Bucket a grid direction into a 3×3 compass cell (0..9, center index 4 unused), for jump
/// dedup. Distinct for all 8 surrounding directions — opposite directions never collide.
fn dir_bucket(dgx: i32, dgy: i32) -> usize {
    ((dgx.signum() + 1) + (dgy.signum() + 1) * 3) as usize
}

/// Per-map navigation state, reset each map load. Lives on `GameState`.
#[derive(Default)]
pub struct NavState {
    /// The parsed clip-hull geometry the navmesh is derived from. `None` until a map's BSP
    /// has been successfully read and parsed.
    pub bsp: Option<Bsp>,
    /// The built navigation graph. `None` until [`NavGraph::build`] runs (bots stay disabled).
    pub graph: Option<NavGraph>,
    /// Whether a build has been attempted for this map (so a failed BSP read doesn't retry
    /// every frame). Reset when a new map loads.
    pub attempted: bool,
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
            .map(|s| directed_reach(s))
            .max_by_key(|o| o.len())
            .unwrap();
        let (start, reached) = (best[0], best.len());
        let reach_frac = reached as f32 / g.cells.len() as f32;
        eprintln!("best directed reach: {reached}/{} = {:.0}%", g.cells.len(), reach_frac * 100.0);

        // Assert A* returns a valid chain to the farthest reachable cell.
        let goal = *best.last().unwrap();
        let route = g.find_path(start, goal).expect("A* found no route to a reachable cell");
        let mut cell = start;
        for &li in &route {
            assert_eq!(g.links[li as usize].from, cell, "route discontinuity");
            cell = g.links[li as usize].to;
        }
        assert_eq!(cell, goal, "route did not reach goal");
        eprintln!("A*: route to {goal} is {} links", route.len());
        assert!(reach_frac > 0.4, "best directed reach too low: {:.0}%", reach_frac * 100.0);

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
    }
}
