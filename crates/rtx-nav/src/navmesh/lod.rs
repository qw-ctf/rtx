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

use super::{hazard_cost, CellId, LinkCosts, LinkKind, NavGraph, CLOSED_GATE_PENALTY};

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
/// (the shortest in-cluster hop between two of its portals). `base` is the static travel cost
/// *including* the water tax (baked at the graph-swap patch, since the liquid columns don't exist on
/// the worker); the rest of the dynamic terms are priced per query from the metadata, never baked, so
/// live door state and per-bot hazard nerve stay honest.
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
    /// Health lost to lava/slime along this edge — priced per bot via `hazard_cost` in [`price_meta`].
    hazard_hp: f32,
    /// For a crossing edge, the representative fine link, so the swap patch can re-read its liquid
    /// costs; `u32::MAX` for a transit edge (rebuilt wholesale at the patch instead).
    link: u32,
}

/// Reaching a cell from one of the entry portals of its cluster: the final coarse hop. Same metadata
/// as an [`AbsEdge`], priced the same way.
struct PortalReach {
    portal: u32,
    dist: f32,
    gates: u32,
    rj: u8,
    chained: bool,
    hazard_hp: f32,
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

/// Top bit of the gate bitmask, reserved to mean "crosses a gate whose id ≥ 31 that the 31 tracked
/// bits can't represent". Priced as an always-shut door (a safe overestimate) rather than dropped —
/// dropping it (the old `gi < 32` check) priced a 32nd gate at 0, a real underestimate that could make
/// a bot value an item actually sealed behind it. Real maps have single-digit gate counts, so this
/// never fires; it's the correctness backstop.
const SEALED_GATE: u32 = 1 << 31;

/// The static (build-time) metadata a link contributes to an abstract edge: its gate bit, whether it
/// is a rocket jump, whether it is a chained speed jump.
fn link_meta(graph: &NavGraph, li: u32) -> (u32, u8, bool) {
    let gates = match graph.gate_of_link(li) {
        Some(gi) if gi < 31 => 1u32 << gi,
        Some(_) => SEALED_GATE,
        None => 0,
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

        // Kept crossings — the cross-cluster links promoted to abstract edges. One representative per
        // directed cluster pair (a wide border must not become dozens of parallel portals, or the
        // intra-cluster all-pairs transit explodes to a graph larger than the fine one; the coverage
        // pass below adds back any crossing a rep fails to cover). Chosen by cheapest, then *gate-free*
        // (so the abstract graph carries an open route between clusters where one exists, and a shut
        // door doesn't wrongly seal the pair), then lowest index (deterministic).
        let mut rep: HashMap<(u32, u32), u32> = HashMap::new();
        for li in 0..self.links.len() as u32 {
            let link = self.links[li as usize];
            let key = (cluster_of[link.from as usize], cluster_of[link.to as usize]);
            if key.0 == key.1 {
                continue;
            }
            let better = match rep.get(&key) {
                None => true,
                Some(&best) => {
                    let best_cost = self.links[best as usize].cost;
                    if link.cost != best_cost {
                        link.cost < best_cost
                    } else if self.gate_of_link(li).is_some() != self.gate_of_link(best).is_some() {
                        self.gate_of_link(li).is_none() // prefer the gate-free crossing at equal cost
                    } else {
                        li < best
                    }
                }
            };
            if better {
                rep.insert(key, li);
            }
        }
        let mut kept: Vec<u32> = rep.values().copied().collect();
        kept.sort_unstable();

        let mut lod = self.build_lod_tables(&cluster_of, cluster_count, &kept);

        // Coverage: a cell reachable only through a *dropped* crossing's landing (two teleporter exits
        // into directed-disjoint pockets, a ledge only jumpable from the neighbour) would have an empty
        // `cell_reach` and read `cost_to = INFINITY` while the exact flood reaches it — a silent goal
        // drop. Promote every cross-cluster landing a rep didn't cover (its `cell_reach` is empty) and
        // rebuild once: those landings become portals, so `intra_reach` floods from them. One rebuild
        // suffices — adding crossings only adds coverage, and a promoted landing covers at least itself.
        let extra: Vec<u32> = (0..self.links.len() as u32)
            .filter(|&li| {
                let link = self.links[li as usize];
                cluster_of[link.from as usize] != cluster_of[link.to as usize] && lod.cell_reach[link.to as usize].is_empty()
            })
            .collect();
        if !extra.is_empty() {
            kept.extend(extra);
            kept.sort_unstable();
            kept.dedup();
            lod = self.build_lod_tables(&cluster_of, cluster_count, &kept);
        }
        self.lod = Some(lod);
    }

    /// Build the portal graph + intra-cluster tables from a fixed set of `kept` cross-cluster links
    /// (the abstract edges). Portal cells = the kept crossings' endpoints, in cell order; each kept
    /// crossing is a crossing edge; one intra-cluster Dijkstra per portal gives the transit edges
    /// (portal→portal) and the reach table (portal→every in-cluster cell). Callable twice — the second
    /// pass rebuilds with the coverage promotions folded in.
    fn build_lod_tables(&self, cluster_of: &[u32], cluster_count: u32, kept: &[u32]) -> Lod {
        let n = self.cells.len();
        let mut is_portal = vec![false; n];
        for &li in kept {
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
        for &li in kept {
            let link = self.links[li as usize];
            let (gates, rj, chained) = link_meta(self, li);
            let pf = portal_of_cell[link.from as usize] as u32;
            let pt = portal_of_cell[link.to as usize] as u32;
            let base = link.cost + self.link_water_extra(li);
            let hazard_hp = self.link_hazard_hp(li);
            abs_adj[pf as usize].push(AbsEdge { to: pt, base, gates, rj, chained, hazard_hp, link: li });
        }
        let mut cell_reach: Vec<Vec<PortalReach>> = (0..n).map(|_| Vec::new()).collect();
        for pi in 0..portals.len() as u32 {
            let (src, cl) = (portals[pi as usize].cell, portals[pi as usize].cluster);
            for (cell, dist, gates, rj, chained, hazard_hp) in self.intra_reach(src, cl, cluster_of) {
                let pc = portal_of_cell[cell as usize];
                if pc >= 0 && cell != src {
                    abs_adj[pi as usize].push(AbsEdge { to: pc as u32, base: dist, gates, rj, chained, hazard_hp, link: u32::MAX });
                }
                cell_reach[cell as usize].push(PortalReach { portal: pi, dist, gates, rj, chained, hazard_hp });
            }
        }
        Lod { cluster_of: cluster_of.to_vec(), cluster_count, portal_of_cell, portals, abs_adj, cell_reach }
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

    /// Dijkstra from `src` restricted to the cells of cluster `cl`, with a metadata accumulator (gates
    /// OR'd, rocket jumps counted, chained flag OR'd, hazard-hp summed) along each cell's min-cost
    /// in-cluster path. Cost includes the water tax (`link_water_extra`), so the min-cost path is the
    /// wettest-aware one. On the worker build the liquid columns are empty (both read 0); the swap
    /// patch re-runs this once they are filled — see [`patch_lod_liquids`](Self::patch_lod_liquids).
    fn intra_reach(&self, src: CellId, cl: u32, cluster_of: &[u32]) -> Vec<(CellId, f32, u32, u8, bool, f32)> {
        let mut dist: HashMap<CellId, (f32, u32, u8, bool, f32)> = HashMap::new();
        let mut heap = BinaryHeap::new();
        dist.insert(src, (0.0, 0, 0, false, 0.0));
        heap.push(MinNode { key: 0.0, id: src });
        let mut out = Vec::new();
        while let Some(MinNode { key: g, id: cell }) = heap.pop() {
            let (d, gates, rj, chained, haz) = dist[&cell];
            if g > d {
                continue;
            }
            out.push((cell, d, gates, rj, chained, haz));
            for &li in &self.adjacency[cell as usize] {
                let link = self.links[li as usize];
                if cluster_of[link.to as usize] != cl {
                    continue; // stay inside the cluster
                }
                let (lg, lrj, lch) = link_meta(self, li);
                let ng = d + link.cost + self.link_water_extra(li);
                if dist.get(&link.to).is_none_or(|&(od, ..)| ng < od) {
                    let nh = haz + self.link_hazard_hp(li);
                    dist.insert(link.to, (ng, gates | lg, rj.saturating_add(lrj), chained || lch, nh));
                    heap.push(MinNode { key: ng, id: link.to });
                }
            }
        }
        out
    }

    /// Fold water and hazard costs into the abstract graph, once the graph-swap has filled the liquid
    /// columns (they don't exist on the worker build — see `nav_build::poll_navmesh_build`). Re-reads
    /// each crossing's own water/hazard, and re-runs the intra-cluster tables for the clusters that
    /// contain a liquid link. Cheap: dry clusters — the overwhelming majority — keep their build-time
    /// costs untouched. A no-op on a bare or liquid-free graph.
    pub fn patch_lod_liquids(&mut self) {
        let Some(mut lod) = self.lod.take() else {
            return;
        };
        // Crossing edges: re-read each representative crossing's own liquid costs (idempotent for a
        // dry crossing, where both read 0).
        for edges in &mut lod.abs_adj {
            for e in edges.iter_mut() {
                if e.link != u32::MAX {
                    e.base = self.links[e.link as usize].cost + self.link_water_extra(e.link);
                    e.hazard_hp = self.link_hazard_hp(e.link);
                }
            }
        }
        // Clusters holding an intra liquid link need their transit + reach recomputed with liquids.
        let mut liquid = vec![false; lod.cluster_count as usize];
        for li in 0..self.links.len() as u32 {
            if self.link_water_extra(li) <= 0.0 && self.link_hazard_hp(li) <= 0.0 {
                continue;
            }
            let link = self.links[li as usize];
            let (cf, ct) = (lod.cluster_of[link.from as usize], lod.cluster_of[link.to as usize]);
            if cf == ct {
                liquid[cf as usize] = true;
            }
        }
        if liquid.iter().any(|&x| x) {
            for c in 0..self.cells.len() {
                if liquid[lod.cluster_of[c] as usize] {
                    lod.cell_reach[c].clear();
                }
            }
            let cluster_of = lod.cluster_of.clone();
            for pi in 0..lod.portals.len() as u32 {
                let (src, cl) = (lod.portals[pi as usize].cell, lod.portals[pi as usize].cluster);
                if !liquid[cl as usize] {
                    continue;
                }
                // Drop this portal's stale transit edges (identified by the sentinel link), keeping the
                // crossings, then re-add the liquid-aware transit + reach from a fresh intra flood.
                lod.abs_adj[pi as usize].retain(|e| e.link != u32::MAX);
                for (cell, dist, gates, rj, chained, hazard_hp) in self.intra_reach(src, cl, &cluster_of) {
                    let pc = lod.portal_of_cell[cell as usize];
                    if pc >= 0 && cell != src {
                        lod.abs_adj[pi as usize].push(AbsEdge { to: pc as u32, base: dist, gates, rj, chained, hazard_hp, link: u32::MAX });
                    }
                    lod.cell_reach[cell as usize].push(PortalReach { portal: pi, dist, gates, rj, chained, hazard_hp });
                }
            }
        }
        self.lod = Some(lod);
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
                let ng = g + e.base + self.price_meta(e.gates, e.rj, e.chained, e.hazard_hp, costs, sever_chained);
                if ng < abs_cost[e.to as usize] {
                    abs_cost[e.to as usize] = ng;
                    heap.push(MinNode { key: ng, id: e.to });
                }
            }
        }
        CoarseCosts { graph: self, costs, sever_chained, full: None, home, abs_cost }
    }

    /// The coarse corridor toward a far `goal`: an interim steer target plus the cluster window the fine
    /// search may stay inside and the gates the corridor crosses. The interim is the first portal at/past
    /// `horizon` along the abstract shortest path; steering the fine banded A* at it (restricted to the
    /// window) instead of the far goal bounds the search to a local neighbourhood — the abstract path is
    /// a real fine path through the window clusters, so a route always exists there. The next repath
    /// advances it as the bot moves. `None` when the goal is within `horizon` (steer it directly) or
    /// unreachable, or on a bare graph. `sever_chained` is `false` — a corridor may lead through a
    /// chained speed jump the fine window then gates for feasibility.
    pub fn corridor(&self, from: CellId, goal: CellId, costs: &LinkCosts, horizon: f32) -> Option<Corridor> {
        let lod = self.lod.as_ref()?;
        let from_cl = lod.cluster_of[from as usize];
        if from_cl == lod.cluster_of[goal as usize] {
            return None; // same cluster — the goal is near, steer straight at it
        }
        // Abstract Dijkstra from the home portals, tracking each portal's predecessor and the gate bits
        // accumulated along its min-cost path (home-cluster gates aren't tracked — they sit on the fine
        // route, so `route_blocking_gate` still catches them; the abstract edges carry the far ones).
        let home = self.home_flood(from, from_cl, lod, costs, false);
        let np = lod.portals.len();
        let mut abs_cost = vec![f32::INFINITY; np];
        let mut parent = vec![u32::MAX; np];
        let mut pgate = vec![0u32; np];
        let mut heap = BinaryHeap::new();
        for (&cell, &seed) in &home {
            let p = lod.portal_of_cell[cell as usize];
            if p >= 0 && seed < abs_cost[p as usize] {
                abs_cost[p as usize] = seed;
                heap.push(MinNode { key: seed, id: p as u32 });
            }
        }
        while let Some(MinNode { key: g, id: p }) = heap.pop() {
            if g > abs_cost[p as usize] {
                continue;
            }
            for e in &lod.abs_adj[p as usize] {
                let ng = g + e.base + self.price_meta(e.gates, e.rj, e.chained, e.hazard_hp, costs, false);
                if ng < abs_cost[e.to as usize] {
                    abs_cost[e.to as usize] = ng;
                    parent[e.to as usize] = p;
                    pgate[e.to as usize] = pgate[p as usize] | e.gates;
                    heap.push(MinNode { key: ng, id: e.to });
                }
            }
        }
        // The cheapest entry portal of the goal's cluster (coarse cost into the cluster + the in-cluster
        // hop to the goal). If the whole thing is within `horizon` the goal is near — steer it directly.
        let mut goal_portal = None;
        let mut goal_cost = f32::INFINITY;
        for r in &lod.cell_reach[goal as usize] {
            let via = abs_cost[r.portal as usize];
            if via.is_finite() && via + r.dist < goal_cost {
                goal_cost = via + r.dist;
                goal_portal = Some(r.portal);
            }
        }
        let goal_portal = goal_portal?;
        if goal_cost <= horizon {
            return None;
        }
        // Walk the abstract path home→goal, flagging every cluster it passes through (the fine search's
        // window) and stopping the interim at the first portal at/past the horizon.
        let mut chain = Vec::new();
        let mut p = goal_portal;
        while p != u32::MAX {
            chain.push(p);
            p = parent[p as usize];
        }
        let mut allowed = vec![false; lod.cluster_count as usize];
        allowed[from_cl as usize] = true;
        let mut interim = goal_portal;
        for &p in chain.iter().rev() {
            allowed[lod.portals[p as usize].cluster as usize] = true;
            if abs_cost[p as usize] >= horizon {
                interim = p;
                break;
            }
        }
        Some(Corridor { interim: lod.portals[interim as usize].cell, allowed, crossed_gates: pgate[goal_portal as usize] })
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
    fn price_meta(&self, gates: u32, rj: u8, chained: bool, hazard_hp: f32, costs: &LinkCosts, sever_chained: bool) -> f32 {
        let mut extra = 0.0;
        if gates != 0 {
            for gi in 0..self.gate_count().min(31) {
                if gates & (1 << gi) != 0 && costs.gate_closed.get(gi).copied().unwrap_or(false) {
                    extra += if costs.openable_gates.get(gi).copied().unwrap_or(false) {
                        costs.open_gate_cost
                    } else {
                        CLOSED_GATE_PENALTY
                    };
                }
            }
            // An untracked (≥31) gate is priced as always shut — never underestimate (see SEALED_GATE).
            if gates & SEALED_GATE != 0 {
                extra += CLOSED_GATE_PENALTY;
            }
        }
        if rj > 0 && costs.rocket_jump_extra > 0.0 {
            extra += rj as f32 * costs.rocket_jump_extra;
        }
        if sever_chained && chained {
            extra += CLOSED_GATE_PENALTY;
        }
        // Lava/slime crossed along this edge, priced against the querying bot's nerve — the lump
        // equivalent of the fine graph's per-link `hazard_cost` (`link_extra`). Only home-cluster
        // hazards are per-bot-exact (the fine home flood); this covers the far field.
        if hazard_hp > 0.0 {
            if let Some(price) = costs.hazard {
                extra += hazard_cost(hazard_hp, price);
            }
        }
        extra
    }
}

/// The result of [`NavGraph::corridor`]: where to steer next toward a far goal, and the bound.
pub struct Corridor {
    /// The interim steer target — the first corridor portal cell at/past the horizon.
    pub interim: CellId,
    /// Clusters the fine search may enter (home + the corridor up to the interim), indexed by cluster
    /// id. A route to the interim exists inside this window, so the restricted search always succeeds.
    pub allowed: Vec<bool>,
    /// Gate-id bits the corridor crosses reaching the true goal — for the far button-errand pre-arm
    /// (near/home gates already sit on the fine route, caught by `route_blocking_gate`).
    pub crossed_gates: u32,
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
                let c = via + r.dist + self.graph.price_meta(r.gates, r.rj, r.chained, r.hazard_hp, self.costs, self.sever_chained);
                if c < best {
                    best = c;
                }
            }
        }
        best
    }
}
