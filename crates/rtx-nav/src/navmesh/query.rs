// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh queries: nearest-cell lookup and the A* / Dijkstra pathfinding over the built graph
//! (`find_path`, `nearest_reachable_to`, `costs_from`), plus the small link/cell accessors bots read
//! routes through. Split out of the graph *build* (`super`) — this is the read-only side.

use glam::{Vec2, Vec3, Vec3Swizzles};

use super::{
    band_of, floor_grid, CellId, LinkCosts, LinkCounts, LinkKind, NavGraph, BAND_V_MAX, MAX_SPEED, NBANDS,
    SPEED_CONE_DEG,
};

/// A min-heap entry for the A*/Dijkstra searches below: a NaN-free f32 priority `key` carrying an
/// opaque `payload` (a cell, or a `(cell, band)` state). `Ord` reverses the key so `BinaryHeap` (a
/// max-heap) pops the smallest key first; the payload never participates in the comparison. One
/// shared shape replaces the three identical hand-written `Node` orderings.
struct MinCost<T> {
    key: f32,
    payload: T,
}
impl<T> PartialEq for MinCost<T> {
    fn eq(&self, o: &Self) -> bool {
        self.key == o.key
    }
}
impl<T> Eq for MinCost<T> {}
impl<T> PartialOrd for MinCost<T> {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl<T> Ord for MinCost<T> {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.key.partial_cmp(&self.key).unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// A route from the banded planner: link indices plus the planned *entry* speed band for each leg
/// (parallel to `links`), so the runtime knows where it is meant to arrive carrying speed.
pub struct BandedRoute {
    pub links: Vec<u32>,
    pub bands: Vec<u8>,
    /// Total banded travel-time cost of the route (seconds).
    pub cost: f32,
    /// The speed band the bot arrives in at the goal — the carry into whatever comes next (e.g. the
    /// next race leg, since a checkpoint touch doesn't stop the runner).
    pub end_band: u8,
}

impl NavGraph {
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
    /// A* from `start` to `goal`. `costs` supplies the live door state plus any per-bot surcharges
    /// and jitter (see [`LinkCosts`]): a link through a shut gate is charged `CLOSED_GATE_PENALTY`,
    /// so the route bends around closed doors when it can and only crosses one (leaving the bot to
    /// open it) when there's no other way. Pass [`LinkCosts::gated`] (or `default`) for gates-only.
    pub fn find_path(&self, start: CellId, goal: CellId, costs: &LinkCosts) -> Option<Vec<u32>> {
        use std::collections::BinaryHeap;

        if start == goal {
            return Some(Vec::new());
        }
        let h = |c: CellId| (self.cells[goal as usize].origin - self.cells[c as usize].origin).length() / MAX_SPEED;

        // Min-heap on f = g + h (see MinCost).
        let n = self.cells.len();
        let mut g_cost = vec![f32::INFINITY; n];
        let mut came_from = vec![u32::MAX; n]; // link index used to reach this cell
        let mut heap = BinaryHeap::new();
        g_cost[start as usize] = 0.0;
        heap.push(MinCost {
            key: h(start),
            payload: start,
        });

        while let Some(MinCost { payload: cell, .. }) = heap.pop() {
            if cell == goal {
                return Some(self.reconstruct(&came_from, start, goal));
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                let ng = g_cost[cell as usize] + link.cost + self.link_extra(li, costs) + self.chained_block(li);
                if ng < g_cost[link.to as usize] {
                    g_cost[link.to as usize] = ng;
                    came_from[link.to as usize] = li;
                    heap.push(MinCost {
                        key: ng + h(link.to),
                        payload: link.to,
                    });
                }
            }
        }
        None
    }

    /// Speed-band A\*: like [`find_path`](Self::find_path) but planning over `(cell, speed band)`
    /// states, so carried bhop speed changes both which legs are feasible (a chained speed jump
    /// needs a minimum entry band) and their cost (a fast band covers a Walk leg quicker). Returns
    /// the link route plus the planned entry band per leg. `start_speed` seeds the starting band, so
    /// a bot re-pathing mid-run keeps credit for a hop chain already in progress.
    ///
    /// The state space is `cells · NBANDS`; expansion iterates the existing per-cell adjacency and
    /// calls [`banded_step`](Self::banded_step) on the fly (no 4× graph is materialized), composing
    /// `link_extra` on top unchanged. Carried speed only survives a corner within [`SPEED_CONE_DEG`]
    /// of the incoming heading — so the recorded cost depends mildly on the predecessor and the
    /// search is not strictly optimal, but every approximation is *conservative* (it never credits
    /// speed it might not have), so routes stay feasible. The heuristic (`dist / BAND_V_MAX`) is
    /// smaller than [`find_path`]'s, so this never returns a *less* optimal route, only expands more.
    pub fn find_path_banded(
        &self,
        start: CellId,
        goal: CellId,
        start_speed: f32,
        costs: &LinkCosts,
    ) -> Option<BandedRoute> {
        use std::collections::BinaryHeap;

        if start == goal {
            return Some(BandedRoute { links: Vec::new(), bands: Vec::new(), cost: 0.0, end_band: band_of(start_speed) });
        }
        let nb = NBANDS as u32;
        let nstates = self.cells.len() * NBANDS;
        let h = |cell: CellId| {
            (self.cells[goal as usize].origin - self.cells[cell as usize].origin).length() / BAND_V_MAX
        };

        let mut g_cost = vec![f32::INFINITY; nstates];
        let mut came_link = vec![u32::MAX; nstates]; // link used to reach this state
        let mut came_state = vec![u32::MAX; nstates]; // predecessor state
        let mut heap = BinaryHeap::new();
        let s0 = start * nb + band_of(start_speed) as u32;
        g_cost[s0 as usize] = 0.0;
        heap.push(MinCost { key: h(start), payload: s0 });

        while let Some(MinCost { payload: state, .. }) = heap.pop() {
            let cell = state / nb;
            let band = (state % nb) as u8;
            if cell == goal {
                let mut route = self.reconstruct_banded(&came_link, &came_state, state);
                route.cost = g_cost[state as usize];
                route.end_band = band;
                return Some(route);
            }
            // The heading we arrived along, for the carry-around-corners test.
            let in_link = came_link[state as usize];
            let in_dir = (in_link != u32::MAX).then(|| self.link_dir(in_link));
            for &li in &self.adjacency[cell as usize] {
                // Carried speed only counts if the corridor continues within the cone.
                let entry = match in_dir {
                    Some(d) if d.length_squared() > 0.01 => {
                        let cos = d.dot(self.link_dir(li)).clamp(-1.0, 1.0);
                        if cos.acos().to_degrees() > SPEED_CONE_DEG {
                            0
                        } else {
                            band
                        }
                    }
                    _ => band,
                };
                let Some((step_cost, exit)) = self.banded_step(li, entry) else {
                    continue; // infeasible at this entry speed (a chained jump we can't satisfy)
                };
                let ng = g_cost[state as usize] + step_cost + self.link_extra(li, costs);
                let ns = self.links[li as usize].to * nb + exit as u32;
                if ng < g_cost[ns as usize] {
                    g_cost[ns as usize] = ng;
                    came_link[ns as usize] = li;
                    came_state[ns as usize] = state;
                    heap.push(MinCost { key: ng + h(self.links[li as usize].to), payload: ns });
                }
            }
        }
        None
    }

    /// Walk the banded `came_*` tables back from a goal state into a forward route with per-leg
    /// entry bands. Stops at the seeded start state (its `came_link` is `u32::MAX`).
    fn reconstruct_banded(&self, came_link: &[u32], came_state: &[u32], goal_state: u32) -> BandedRoute {
        let nb = NBANDS as u32;
        let mut links = Vec::new();
        let mut bands = Vec::new();
        let mut state = goal_state;
        while came_link[state as usize] != u32::MAX {
            let prev = came_state[state as usize];
            links.push(came_link[state as usize]);
            bands.push((prev % nb) as u8); // entry band = the band of the leg's source state
            state = prev;
        }
        links.reverse();
        bands.reverse();
        BandedRoute { links, bands, cost: 0.0, end_band: 0 } // cost/end_band filled by the caller
    }

    /// Unit horizontal heading of a link (source cell → target cell), or zero for a degenerate link.
    fn link_dir(&self, li: u32) -> Vec2 {
        let l = self.links[li as usize];
        (self.cells[l.to as usize].origin.xy() - self.cells[l.from as usize].origin.xy()).normalize_or_zero()
    }

    /// The reachable cell (per current door states) whose origin is closest to `goal`'s, when
    /// `goal` itself can't be reached. Lets a bot head as far toward an unreachable target as the
    /// graph allows — approaching a wall/door/connection to get line of sight — instead of homing
    /// straight into geometry. `None` only if nothing but `start` is reachable.
    pub fn nearest_reachable_to(&self, start: CellId, goal: CellId, costs: &LinkCosts) -> Option<CellId> {
        let flood = self.costs_from(start, costs);
        let goal_pos = self.cells[goal as usize].origin;
        (0..self.cells.len() as CellId)
            .filter(|&c| c != start && flood[c as usize].is_finite())
            .min_by(|&a, &b| {
                let d = |c: CellId| (self.cells[c as usize].origin - goal_pos).length_squared();
                d(a).total_cmp(&d(b))
            })
    }

    /// Dijkstra cost-flood from `start`: the travel-time cost to reach every cell (`INFINITY`
    /// for unreachable ones). One pass answers "how far is each item?" for goal selection, far
    /// cheaper than an A* per candidate. Indexed by [`CellId`].
    pub fn costs_from(&self, start: CellId, costs: &LinkCosts) -> Vec<f32> {
        use std::collections::BinaryHeap;

        let mut cost = vec![f32::INFINITY; self.cells.len()];
        let mut heap = BinaryHeap::new();
        cost[start as usize] = 0.0;
        heap.push(MinCost { key: 0.0, payload: start });
        while let Some(MinCost { key: g, payload: cell }) = heap.pop() {
            if g > cost[cell as usize] {
                continue; // a cheaper path already settled this cell
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                let ng = g + link.cost + self.link_extra(li, costs) + self.chained_block(li);
                if ng < cost[link.to as usize] {
                    cost[link.to as usize] = ng;
                    heap.push(MinCost { key: ng, payload: link.to });
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

    /// Whether a bot standing on this cell is under water (its origin is submerged, so pmove swims).
    /// Set by [`surcharge_water_links`](Self::surcharge_water_links); an unmarked graph reads as dry.
    pub fn cell_in_water(&self, cell: CellId) -> bool {
        self.water.get(cell as usize).copied().unwrap_or(false)
    }

    /// Whether a bot standing on this cell can breathe (its eye point is out of the water) — the
    /// destinations a drowning bot paths to for air. An unmarked graph reads as all-breathable (dry).
    pub fn cell_breathable(&self, cell: CellId) -> bool {
        self.breathable.get(cell as usize).copied().unwrap_or(true)
    }

    /// The liquid a bot standing on this cell is *in* — lava/slime at its feet, which the game burns
    /// it for — or `None` for safe footing. Set by [`surcharge_hazard_links`](Self::surcharge_hazard_links);
    /// an unmarked graph reads as all-safe.
    pub fn cell_hazard(&self, cell: CellId) -> Option<crate::hazard::HazardKind> {
        self.hazard.get(cell as usize).copied().flatten()
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
                LinkKind::RocketJump => c.rocket_jump += 1,
            }
        }
        c
    }
}
