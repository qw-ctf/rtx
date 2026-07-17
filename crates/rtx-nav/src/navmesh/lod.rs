// SPDX-License-Identifier: AGPL-3.0-or-later

//! Level-of-detail hierarchy over the fine cell graph — the navigation analogue of mesh LOD. Cells
//! group into coarse **clusters** (a connected component of cells within a spatial block), and an
//! abstract graph of **portals** between clusters lets goal scoring and long-range steering reason
//! over hundreds of nodes instead of tens of thousands of cells. Near the bot the fine graph is still
//! queried exactly; only the far field goes coarse.
//!
//! Built once at the end of [`build_navmesh`](super::build_navmesh), after every link is spliced —
//! same slot and lifetime as [`super::reach`]. `lod: Option<Lod>` on `NavGraph`; `None` on a bare
//! (unbuilt) graph, where the public accessors fall back conservatively.
//!
//! This module is grown in steps: clustering first, the abstract portal graph and the coarse-cost
//! query on top of it.

use std::collections::{BinaryHeap, HashMap};

use super::{CellId, LinkCosts, LinkKind, NavGraph, CLOSED_GATE_PENALTY};

/// Grid columns per cluster-block edge, as a shift. `3` → `1<<3 = 8` columns → `8 · GRID = 256u`
/// blocks. Bigger blocks mean fewer, coarser clusters (cheaper abstract graph, blunter estimates).
const LOD_SHIFT: i32 = 3;

/// A min-heap entry with a NaN-free f32 key (mirrors `query::MinCost`): `BinaryHeap` is a max-heap, so
/// `Ord` reverses the key and the smallest pops first. `id` is a cell or an abstract-portal index.
struct MinNode {
    key: f32,
    id: u32,
}
impl PartialEq for MinNode {
    fn eq(&self, o: &Self) -> bool {
        self.key == o.key
    }
}
impl Eq for MinNode {}
impl PartialOrd for MinNode {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for MinNode {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.key.partial_cmp(&self.key).unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// An abstract-graph node: a cell where a link crosses a cluster boundary — where a coarse route
/// leaves one cluster or enters the next.
struct Portal {
    cell: CellId,
    cluster: u32,
}

/// A directed abstract edge: a boundary crossing (one cross-cluster link) or intra-cluster transit
/// (the shortest in-cluster hop between two of its portals). `base` is the static travel cost; the
/// dynamic terms are folded in per query from the metadata (never baked, so live door state stays
/// honest). Water/hazard costs are added at the graph-swap patch (they don't exist on the worker).
struct AbsEdge {
    to: u32,
    base: f32,
    /// OR of the gate-id bits (`1<<gi`) this edge crosses; gate counts are single-digit, so 32 bits
    /// suffice (a gate id ≥ 32 is simply not tracked — a rare, benign coarse underestimate).
    gates: u32,
    /// How many rocket-jump links this edge crosses — priced by the per-bot RJ-unfit surcharge.
    rj: u8,
    /// Crosses a chained speed jump: severed in scoring mode (parity with `chained_block`), allowed in
    /// corridor mode so a route can still lead through a chain the fine window then gates for feasibility.
    chained: bool,
}

/// Reaching a cell from one of the entry portals of its cluster: the final coarse hop. Same metadata
/// as an [`AbsEdge`], priced the same way.
struct PortalReach {
    portal: u32,
    dist: f32,
    gates: u32,
    rj: u8,
    chained: bool,
}

/// The level-of-detail tables: cluster assignment plus the abstract portal graph over it.
pub(super) struct Lod {
    /// Cluster id of each cell, parallel to `cells`, dense in `0..cluster_count`.
    cluster_of: Vec<u32>,
    cluster_count: u32,
    /// Abstract node index of each cell, or `-1` if the cell isn't a portal (parallel to `cells`).
    portal_of_cell: Vec<i32>,
    /// The abstract nodes.
    portals: Vec<Portal>,
    /// Outgoing abstract edges per portal node (crossings + intra-cluster transit).
    abs_adj: Vec<Vec<AbsEdge>>,
    /// Per cell, the entry portals of its cluster that reach it (intra-cluster), for the final coarse
    /// hop of [`CoarseCosts::cost_to`].
    cell_reach: Vec<Vec<PortalReach>>,
}

/// The block a cell sits in — its grid column shifted down by [`LOD_SHIFT`]. Two cells cluster
/// together only if they share a block *and* a link path within it.
#[inline]
fn block_of(gx: i32, gy: i32) -> (i32, i32) {
    (gx >> LOD_SHIFT, gy >> LOD_SHIFT)
}

/// Union-find with path-halving + union-by-rank, for the intra-block connected-components pass.
struct UnionFind {
    parent: Vec<u32>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self { parent: (0..n as u32).collect(), rank: vec![0; n] }
    }

    fn find(&mut self, x: u32) -> u32 {
        let mut r = x;
        while self.parent[r as usize] != r {
            r = self.parent[r as usize];
        }
        // Path-halving: point every node on the walk straight at the root.
        let mut c = x;
        while self.parent[c as usize] != r {
            let next = self.parent[c as usize];
            self.parent[c as usize] = r;
            c = next;
        }
        r
    }

    fn union(&mut self, a: u32, b: u32) {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        if self.rank[ra as usize] < self.rank[rb as usize] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb as usize] = ra;
        if self.rank[ra as usize] == self.rank[rb as usize] {
            self.rank[ra as usize] += 1;
        }
    }
}

/// The static (build-time) metadata a link contributes to an abstract edge: its gate bit, whether it
/// is a rocket jump, whether it is a chained speed jump.
fn link_meta(graph: &NavGraph, li: u32) -> (u32, u8, bool) {
    let gates = match graph.gate_of_link(li) {
        Some(gi) if gi < 32 => 1u32 << gi,
        _ => 0,
    };
    let rj = (graph.link_kind(li) == LinkKind::RocketJump) as u8;
    let chained = graph.speed_jump_of_link(li).is_some_and(|s| s.chained);
    (gates, rj, chained)
}

impl NavGraph {
    /// Build the LOD tables. Called once at the end of the navmesh build, after all splices, so it
    /// sees every link. Clustering is O(V+E); the abstract graph runs one intra-cluster Dijkstra per
    /// portal (clusters are small), all serial — cheap next to the parallel cell/link solve.
    pub(super) fn build_lod(&mut self) {
        let n = self.cells.len();
        if n == 0 {
            self.lod = None;
            return;
        }
        let (cluster_of, cluster_count) = self.cluster_cells();

        // Collapse each directed cluster pair's border to a single representative crossing — the
        // cheapest link between those clusters (lowest index on a tie, so the build is deterministic).
        // Without this a wide border becomes dozens of parallel portals and the intra-cluster all-pairs
        // transit explodes to a graph larger than the fine one; one portal per neighbour keeps it small.
        let mut rep: HashMap<(u32, u32), u32> = HashMap::new();
        for li in 0..self.links.len() as u32 {
            let link = self.links[li as usize];
            let key = (cluster_of[link.from as usize], cluster_of[link.to as usize]);
            if key.0 == key.1 {
                continue;
            }
            match rep.get(&key) {
                Some(&best) if self.links[best as usize].cost <= link.cost => {}
                _ => {
                    rep.insert(key, li);
                }
            }
        }

        // Portal cells: the endpoints of the representative crossings, assigned abstract-node indices
        // in cell order (deterministic).
        let mut is_portal = vec![false; n];
        for &li in rep.values() {
            let link = self.links[li as usize];
            is_portal[link.from as usize] = true;
            is_portal[link.to as usize] = true;
        }
        let mut portal_of_cell = vec![-1i32; n];
        let mut portals = Vec::new();
        for c in 0..n as u32 {
            if is_portal[c as usize] {
                portal_of_cell[c as usize] = portals.len() as i32;
                portals.push(Portal { cell: c, cluster: cluster_of[c as usize] });
            }
        }
        let mut abs_adj: Vec<Vec<AbsEdge>> = (0..portals.len()).map(|_| Vec::new()).collect();

        // Crossing edges: one per representative, built in link order (deterministic).
        for li in 0..self.links.len() as u32 {
            let link = self.links[li as usize];
            let key = (cluster_of[link.from as usize], cluster_of[link.to as usize]);
            if key.0 == key.1 || rep.get(&key) != Some(&li) {
                continue;
            }
            let (gates, rj, chained) = link_meta(self, li);
            let pf = portal_of_cell[link.from as usize] as u32;
            let pt = portal_of_cell[link.to as usize] as u32;
            abs_adj[pf as usize].push(AbsEdge { to: pt, base: link.cost, gates, rj, chained });
        }

        // Intra-cluster tables: from each portal, a Dijkstra restricted to its cluster gives the
        // transit edges (portal→portal) and the final-hop reach (portal→every cell in the cluster).
        let mut cell_reach: Vec<Vec<PortalReach>> = (0..n).map(|_| Vec::new()).collect();
        for pi in 0..portals.len() as u32 {
            let Portal { cell: src, cluster: cl } = portals[pi as usize];
            for (cell, dist, gates, rj, chained) in self.intra_reach(src, cl, &cluster_of) {
                let pc = portal_of_cell[cell as usize];
                if pc >= 0 && cell != src {
                    abs_adj[pi as usize].push(AbsEdge { to: pc as u32, base: dist, gates, rj, chained });
                }
                cell_reach[cell as usize].push(PortalReach { portal: pi, dist, gates, rj, chained });
            }
        }

        self.lod = Some(Lod { cluster_of, cluster_count, portal_of_cell, portals, abs_adj, cell_reach });
    }

    /// Weakly connect cells joined by a link that stays inside one block (grouping is undirected — a
    /// one-way drop still groups its endpoints; the directed intra distances carry the asymmetry).
    /// Returns the dense per-cell cluster id (first-appearance order → deterministic) and the count.
    fn cluster_cells(&self) -> (Vec<u32>, u32) {
        let n = self.cells.len();
        let mut uf = UnionFind::new(n);
        for link in &self.links {
            let a = &self.cells[link.from as usize];
            let b = &self.cells[link.to as usize];
            if block_of(a.gx, a.gy) == block_of(b.gx, b.gy) {
                uf.union(link.from, link.to);
            }
        }
        let mut cluster_of = vec![u32::MAX; n];
        let mut root_id: HashMap<u32, u32> = HashMap::new();
        let mut count = 0u32;
        for c in 0..n as u32 {
            let r = uf.find(c);
            let id = *root_id.entry(r).or_insert_with(|| {
                let id = count;
                count += 1;
                id
            });
            cluster_of[c as usize] = id;
        }
        (cluster_of, count)
    }

    /// Dijkstra from `src` restricted to the cells of cluster `cl`, over static link cost only, with a
    /// metadata accumulator (gates OR'd, rocket jumps counted, chained flag OR'd) along each cell's
    /// min-cost in-cluster path. One per portal at build time.
    fn intra_reach(&self, src: CellId, cl: u32, cluster_of: &[u32]) -> Vec<(CellId, f32, u32, u8, bool)> {
        let mut dist: HashMap<CellId, (f32, u32, u8, bool)> = HashMap::new();
        let mut heap = BinaryHeap::new();
        dist.insert(src, (0.0, 0, 0, false));
        heap.push(MinNode { key: 0.0, id: src });
        let mut out = Vec::new();
        while let Some(MinNode { key: g, id: cell }) = heap.pop() {
            let (d, gates, rj, chained) = dist[&cell];
            if g > d {
                continue;
            }
            out.push((cell, d, gates, rj, chained));
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                if cluster_of[link.to as usize] != cl {
                    continue; // stay inside the cluster
                }
                let (lg, lrj, lch) = link_meta(self, li);
                let ng = d + link.cost;
                if dist.get(&link.to).is_none_or(|&(od, ..)| ng < od) {
                    dist.insert(link.to, (ng, gates | lg, rj.saturating_add(lrj), chained || lch));
                    heap.push(MinNode { key: ng, id: link.to });
                }
            }
        }
        out
    }

    /// The cluster id of cell `c`, or `None` when the LOD layer isn't built (bare test graphs).
    pub fn cluster_of(&self, c: CellId) -> Option<u32> {
        self.lod.as_ref().map(|l| l.cluster_of[c as usize])
    }

    /// How many clusters the LOD layer has (0 when unbuilt).
    pub fn cluster_count(&self) -> usize {
        self.lod.as_ref().map_or(0, |l| l.cluster_count as usize)
    }

    /// `(clusters, portal nodes, abstract edges, cell-reach entries)` — for the build summary log, to
    /// watch the abstract graph's size against the fine graph's.
    pub fn lod_stats(&self) -> (usize, usize, usize, usize) {
        match &self.lod {
            Some(l) => (
                l.cluster_count as usize,
                l.portals.len(),
                l.abs_adj.iter().map(Vec::len).sum(),
                l.cell_reach.iter().map(Vec::len).sum(),
            ),
            None => (0, 0, 0, 0),
        }
    }

    /// Iterate `(cell, cluster_id)` for every cell — for the navview overlay. Empty when unbuilt.
    pub fn cluster_assignment(&self) -> impl Iterator<Item = (CellId, u32)> + '_ {
        self.lod
            .as_ref()
            .into_iter()
            .flat_map(|l| l.cluster_of.iter().enumerate().map(|(c, &id)| (c as CellId, id)))
    }

    /// Coarse travel costs from `from` under `costs`: exact within [`COARSE_FINE_CAP`] via a bounded
    /// fine flood, and an abstract-graph estimate (a bounded overestimate) beyond it. `sever_chained`
    /// mirrors `chained_block` — `true` for goal scoring (a chained speed jump is impassable to a
    /// speed-unaware estimate), `false` for the steer corridor (it may lead through one). Cheap: a
    /// capped flood plus a home-cluster seed and a Dijkstra over a few hundred portals. Falls back to
    /// an exact full flood on a bare graph (no LOD tables).
    pub fn coarse_costs<'a>(&'a self, from: CellId, costs: &'a LinkCosts, sever_chained: bool) -> CoarseCosts<'a> {
        let Some(lod) = self.lod.as_ref() else {
            // No hierarchy (bare test graph): an exact full flood, read directly.
            let full = Some(self.costs_from(from, costs));
            return CoarseCosts { graph: self, costs, sever_chained, full, home: HashMap::new(), abs_cost: Vec::new() };
        };
        // Exact home cluster, priced by an in-cluster flood; its cells answer `cost_to` directly and
        // its portals seed the abstract search. There is deliberately no separate near-field flood —
        // running one per `coarse_costs` call (up to nine a pick) was the very cost the hierarchy
        // exists to avoid. The abstract graph answers everything past the home cluster.
        let home = self.home_flood(from, lod.cluster_of[from as usize], lod, costs, sever_chained);
        let mut abs_cost = vec![f32::INFINITY; lod.portals.len()];
        let mut heap = BinaryHeap::new();
        for (&cell, &seed) in &home {
            let p = lod.portal_of_cell[cell as usize];
            if p >= 0 && seed < abs_cost[p as usize] {
                abs_cost[p as usize] = seed;
                heap.push(MinNode { key: seed, id: p as u32 });
            }
        }
        // Dijkstra over the abstract graph, pricing dynamic terms on each edge as it is relaxed.
        while let Some(MinNode { key: g, id: p }) = heap.pop() {
            if g > abs_cost[p as usize] {
                continue;
            }
            for e in &lod.abs_adj[p as usize] {
                let ng = g + e.base + self.price_meta(e.gates, e.rj, e.chained, costs, sever_chained);
                if ng < abs_cost[e.to as usize] {
                    abs_cost[e.to as usize] = ng;
                    heap.push(MinNode { key: ng, id: e.to });
                }
            }
        }
        CoarseCosts { graph: self, costs, sever_chained, full: None, home, abs_cost }
    }

    /// Priced Dijkstra from `from` restricted to cluster `cl`, returning the exact priced cost to every
    /// cell of the home cluster. Uses the full `link_extra` pricing (gates, hazard, …) so the home
    /// region matches the fine flood exactly; its portals seed [`coarse_costs`]'s abstract search and
    /// its cells answer [`CoarseCosts::cost_to`] directly for anything past the fine cap.
    fn home_flood(&self, from: CellId, cl: u32, lod: &Lod, costs: &LinkCosts, sever_chained: bool) -> HashMap<CellId, f32> {
        let mut dist: HashMap<CellId, f32> = HashMap::new();
        let mut heap = BinaryHeap::new();
        dist.insert(from, 0.0);
        heap.push(MinNode { key: 0.0, id: from });
        while let Some(MinNode { key: g, id: cell }) = heap.pop() {
            if dist.get(&cell).is_some_and(|&d| g > d) {
                continue;
            }
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                if lod.cluster_of[link.to as usize] != cl {
                    continue;
                }
                let chain = if sever_chained { self.chained_block(li) } else { 0.0 };
                let ng = g + link.cost + self.link_extra(li, costs) + chain;
                if dist.get(&link.to).is_none_or(|&d| ng < d) {
                    dist.insert(link.to, ng);
                    heap.push(MinNode { key: ng, id: link.to });
                }
            }
        }
        dist
    }

    /// Price the dynamic terms an abstract edge carries: closed gates (openable ones at the errand
    /// price), the per-bot rocket-jump-unfit surcharge, and — in scoring mode — a chained speed jump.
    /// Mirrors the terms `link_extra`/`chained_block` add per fine link.
    fn price_meta(&self, gates: u32, rj: u8, chained: bool, costs: &LinkCosts, sever_chained: bool) -> f32 {
        let mut extra = 0.0;
        if gates != 0 {
            for gi in 0..self.gate_count().min(32) {
                if gates & (1 << gi) != 0 && costs.gate_closed.get(gi).copied().unwrap_or(false) {
                    extra += if costs.openable_gates.get(gi).copied().unwrap_or(false) {
                        costs.open_gate_cost
                    } else {
                        CLOSED_GATE_PENALTY
                    };
                }
            }
        }
        if rj > 0 && costs.rocket_jump_extra > 0.0 {
            extra += rj as f32 * costs.rocket_jump_extra;
        }
        if sever_chained && chained {
            extra += CLOSED_GATE_PENALTY;
        }
        extra
    }
}

/// The result of [`NavGraph::coarse_costs`]: exact costs in the home cluster, an abstract-graph
/// estimate beyond. Borrows the graph and the pricing so [`cost_to`](Self::cost_to) can finish the
/// last coarse hop into the target's cluster.
pub struct CoarseCosts<'a> {
    graph: &'a NavGraph,
    costs: &'a LinkCosts<'a>,
    sever_chained: bool,
    /// Present only for a bare graph (no LOD): the exact full flood, read directly for every cell.
    full: Option<Vec<f32>>,
    /// Exact priced costs to every cell of the home cluster.
    home: HashMap<CellId, f32>,
    abs_cost: Vec<f32>,
}

impl CoarseCosts<'_> {
    /// Coarse travel cost from the source to `cell`: exact when `cell` is in the home cluster,
    /// otherwise the cheapest way into its cluster through the abstract graph plus the in-cluster hop
    /// to it. `INFINITY` if unreachable. A bounded overestimate beyond the home cluster — never an
    /// underestimate, so goal scoring can trust it not to call an item closer than it is.
    pub fn cost_to(&self, cell: CellId) -> f32 {
        if let Some(full) = &self.full {
            return full[cell as usize]; // bare graph: exact full flood
        }
        if let Some(&d) = self.home.get(&cell) {
            return d; // home cluster: exact
        }
        let Some(lod) = self.graph.lod.as_ref() else {
            return f32::INFINITY;
        };
        let mut best = f32::INFINITY;
        for r in &lod.cell_reach[cell as usize] {
            let via = self.abs_cost[r.portal as usize];
            if via.is_finite() {
                let c = via + r.dist + self.graph.price_meta(r.gates, r.rj, r.chained, self.costs, self.sever_chained);
                if c < best {
                    best = c;
                }
            }
        }
        best
    }
}
