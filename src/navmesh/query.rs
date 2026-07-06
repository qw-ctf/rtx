// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh queries: nearest-cell lookup and the A* / Dijkstra pathfinding over the built graph
//! (`find_path`, `nearest_reachable_to`, `costs_from`), plus the small link/cell accessors bots read
//! routes through. Split out of the graph *build* (`super`) — this is the read-only side.

use glam::Vec3;

use super::{floor_grid, CellId, LinkCosts, LinkCounts, LinkKind, NavGraph, MAX_SPEED};

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
                let ng = g_cost[cell as usize] + link.cost + self.link_extra(li, costs);
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
                let ng = g + link.cost + self.link_extra(li, costs);
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
}
