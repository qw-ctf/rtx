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
//! Clustering (`cluster_cells`) groups cells; the abstract graph (`build_lod_tables` + a coverage
//! pass) keeps a few representative crossings per cluster pair (one per border level and gatedness —
//! see [`rep_band`] and `build_lod`) plus any landing a rep misses; queries layer on top —
//! [`coarse_costs`](NavGraph::coarse_costs) for scoring, [`corridor`](NavGraph::corridor) for the
//! bounded steer window. The worker build flags the liquid link columns (`water_extra`/`hazard_hp`)
//! before this runs, so the abstract tables price water and hazard directly at build time.

use std::collections::{BinaryHeap, HashMap};

use super::{hazard_cost, CellId, LinkCosts, LinkKind, NavGraph, CLOSED_GATE_PENALTY};

/// Grid columns per cluster-block edge, as a shift. `3` → `1<<3 = 8` columns → `8 · GRID = 256u`
/// blocks. Bigger blocks mean fewer, coarser clusters (cheaper abstract graph, blunter estimates).
const LOD_SHIFT: i32 = 3;

/// Vertical extent of one cluster storey, in world units: cells more than half this apart in Z never
/// share a cluster, even inside one XY block. Without it the block union is Z-blind, so a one-way drop
/// welds a platform to the pit beneath it into one cluster spanning both heights — and then the single
/// cheapest crossing kept per cluster pair is a low walk that *evicts* the climb link onto the
/// platform, so the coarse route into the platform's cluster lands below it and the steer corridor
/// never enters the plateau (the bravado quad, unreachable under LOD). Banded by `round`, not `floor`,
/// exactly as [`super::geom::jump_elev_band`] bands a jump target: a storey is centred on its band, so
/// a floor plane sits mid-band and a full-storey drop lands a band away. A floor straddling a band edge
/// merely splits into two clusters (fewer cells each) — never a wrong weld.
const LOD_STOREY: f32 = 128.0;

/// A min-heap entry with a NaN-free f32 key (mirrors `query::MinCost`): `BinaryHeap` is a max-heap, so
/// `Ord` reverses the key and the smallest pops first. `id` is a cell or an abstract-portal index, and
/// it also breaks key ties so the settle order is deterministic — otherwise two abstract paths of equal
/// cost would resolve by heap-internal order (seeded from a `HashMap`'s per-process iteration), and the
/// corridor's `parent` chain (hence its interim/window/crossed gates) could differ between the server
/// and netclient brains that must drive the same run, or between consecutive identical-state repaths.
struct MinNode {
    key: f32,
    id: u32,
}
impl PartialEq for MinNode {
    fn eq(&self, o: &Self) -> bool {
        self.key == o.key && self.id == o.id
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
        o.key
            .partial_cmp(&self.key)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(o.id.cmp(&self.id))
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
/// *including* the water tax (the liquid columns are flagged on the worker before this table is
/// built, so it bakes in at build time); the rest of the dynamic terms are priced per query from the
/// metadata, never baked, so live door state and per-bot hazard nerve stay honest.
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
    /// For a crossing edge, the representative fine link, so [`price_meta`] can look up its transient
    /// per-link penalty; `u32::MAX` for a transit edge (a multi-link intra path with no single rep).
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

/// The block a cell sits in — its grid column shifted down by [`LOD_SHIFT`], plus a storey band from
/// its height ([`LOD_STOREY`]). Two cells cluster together only if they share a block *and* a storey
/// *and* a link path between them: the storey term keeps a plateau and the pit beneath it (same XY
/// block, joined by a one-way drop) in separate clusters.
#[inline]
fn block_of(gx: i32, gy: i32, z: f32) -> (i32, i32, i32) {
    (gx >> LOD_SHIFT, gy >> LOD_SHIFT, (z / LOD_STOREY).round() as i32)
}

/// Half-storey height band of a crossing endpoint — the vertical resolution at which `build_lod`
/// keeps crossing representatives between a cluster pair. One rep per pair is the classic HPA*
/// entrance-placement trap when the shared border spans height: a 128u storey holds two walkable
/// floors (dm3's base at z-16 and the red-armour spiral's first flight at z56 both round to band 0),
/// so a cluster pair can touch along *both*, and the single cheapest crossing — picked by cost, all
/// flat Walks tie — lands on whichever floor a tie-break favours. Every coarse route into the other
/// floor then detours through that entrance and pays the full intra-cluster climb (or, where the
/// floors don't connect inside the block at all, a many-cluster loop the fine graph never takes),
/// inflating the estimate ~9× and flipping route decisions against the exact flood. Banding the rep
/// slots by the endpoints' heights keeps one entrance per level the border actually touches, which
/// is bounded — a cluster spans at most one storey, so at most three half-storey bands per side —
/// unlike keying on XY extent, where a long flat seam would mint a portal per column for a detour
/// the block circumference already caps at a fraction of a second.
#[inline]
fn rep_band(z: f32) -> i32 {
    (z / (LOD_STOREY * 0.5)).round() as i32
}

/// Union-find with path-halving + union-by-rank, for the intra-block connected-components pass.
struct UnionFind {
    parent: Vec<u32>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n as u32).collect(),
            rank: vec![0; n],
        }
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

        // Kept crossings — the cross-cluster links promoted to abstract edges. Up to two representatives
        // per directed cluster pair *per height band of the crossing's endpoints* ([`rep_band`]): the
        // cheapest crossing, and — when that one is gated — the cheapest *gate-free* crossing, if the
        // pair has one. A wide border must not become dozens of parallel portals (the intra-cluster
        // all-pairs transit would explode to a graph larger than the fine one), but each of these two
        // alternates earns its slot by expressing a way between the clusters the plain cheapest crossing
        // can't: the gate-free rep keeps an *open* way when the cheapest crossing is a shut door, so
        // `cost_to` routes around a closed gate wherever a fine gate-free route exists and an openable
        // prize prices as an errand, not a sealed ~100k wall (preferring gate-free only *at equal cost*
        // wouldn't do it — a strictly-cheaper gated crossing would still seal the pair); the height
        // banding keeps an entrance *per level* when the border spans storeys stacked inside one band,
        // so a route to the border's upper floor crosses there instead of entering at the ground-floor
        // rep and paying a huge intra-cluster climb the exact flood never pays (dm3's red-armour
        // spiral: base and first flight share band 0, and the one kept crossing sat on the base, so the
        // coarse climb read ~18 against an exact ~1 and the bot routed the wrong way off the landing).
        // The coverage pass below adds back any landing no rep covers.
        let mut rep: HashMap<(u32, u32, i32, i32), u32> = HashMap::new();
        let mut rep_free: HashMap<(u32, u32, i32, i32), u32> = HashMap::new();
        for li in 0..self.links.len() as u32 {
            let link = self.links[li as usize];
            let (cf, ct) = (cluster_of[link.from as usize], cluster_of[link.to as usize]);
            if cf == ct {
                continue;
            }
            let key = (
                cf,
                ct,
                rep_band(self.cells[link.from as usize].origin.z),
                rep_band(self.cells[link.to as usize].origin.z),
            );
            // Cheapest, then lowest index (deterministic).
            let cheaper = |cur: Option<&u32>| match cur {
                None => true,
                Some(&best) => {
                    let best_cost = self.links[best as usize].cost;
                    if link.cost != best_cost {
                        link.cost < best_cost
                    } else {
                        li < best
                    }
                }
            };
            if cheaper(rep.get(&key)) {
                rep.insert(key, li);
            }
            if self.gate_of_link(li).is_none() && cheaper(rep_free.get(&key)) {
                rep_free.insert(key, li); // cheapest gate-free crossing of this slot (may equal `rep`)
            }
        }
        let mut kept: Vec<u32> = rep.values().chain(rep_free.values()).copied().collect();
        kept.sort_unstable();
        kept.dedup();

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
                cluster_of[link.from as usize] != cluster_of[link.to as usize]
                    && lod.cell_reach[link.to as usize].is_empty()
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
                portals.push(Portal {
                    cell: c,
                    cluster: cluster_of[c as usize],
                });
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
            abs_adj[pf as usize].push(AbsEdge {
                to: pt,
                base,
                gates,
                rj,
                chained,
                hazard_hp,
                link: li,
            });
        }
        let mut cell_reach: Vec<Vec<PortalReach>> = (0..n).map(|_| Vec::new()).collect();
        for pi in 0..portals.len() as u32 {
            let (src, cl) = (portals[pi as usize].cell, portals[pi as usize].cluster);
            for (cell, dist, gates, rj, chained, hazard_hp) in self.intra_reach(src, cl, cluster_of) {
                let pc = portal_of_cell[cell as usize];
                if pc >= 0 && cell != src {
                    abs_adj[pi as usize].push(AbsEdge {
                        to: pc as u32,
                        base: dist,
                        gates,
                        rj,
                        chained,
                        hazard_hp,
                        link: u32::MAX,
                    });
                }
                cell_reach[cell as usize].push(PortalReach {
                    portal: pi,
                    dist,
                    gates,
                    rj,
                    chained,
                    hazard_hp,
                });
            }
        }
        Lod {
            cluster_of: cluster_of.to_vec(),
            cluster_count,
            portal_of_cell,
            portals,
            abs_adj,
            cell_reach,
        }
    }

    /// Weakly connect cells joined by a link that stays inside one block *and one storey* (grouping is
    /// undirected — a one-way drop within a storey still groups its endpoints; the directed intra
    /// distances carry the asymmetry). A link spanning storeys (a drop off a plateau, a jump up a wall)
    /// is a cluster *crossing*, not an intra-cluster weld. Returns the dense per-cell cluster id
    /// (first-appearance order → deterministic) and the count.
    fn cluster_cells(&self) -> (Vec<u32>, u32) {
        let n = self.cells.len();
        let mut uf = UnionFind::new(n);
        for link in &self.links {
            let a = &self.cells[link.from as usize];
            let b = &self.cells[link.to as usize];
            if block_of(a.gx, a.gy, a.origin.z) == block_of(b.gx, b.gy, b.origin.z) {
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
    /// wettest-aware one — the worker build flags the liquid columns before this runs, so those costs
    /// are already in place.
    ///
    /// The metadata rides the min-*cost* path, chosen gate-blind (gates are metadata, not a cost term
    /// here). So a cell whose cheapest in-cluster path clips a gate carries that gate's bit even if a
    /// pricier *gate-free* in-cluster path also exists; when the gate is shut, `cost_to`/`price_meta`
    /// then charge it the closed-gate penalty — a bounded **overestimate**, never an underestimate, so
    /// the never-call-an-item-closer contract holds. A per-cell gate-free alternate distance (the
    /// `d_nogate` of the plan) would tighten it, but it is deliberately not built: a gate wholly inside
    /// one 256u cluster *with* a gate-free intra detour is rare (doors sit at chokepoints, i.e. cluster
    /// boundaries — handled by the gate-free crossing rep in `build_lod`), the error is safe-direction,
    /// and a second intra flood + parallel metadata on every edge is disproportionate for it. The one
    /// live symptom — a far pre-arm for such an intra gate the fine route would skirt — self-corrects
    /// via `button_reachable` + the errand give-up clock.
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

    /// A stable FNV-1a hash of the LOD tables (clusters, portals, edges, reach), for the
    /// build-determinism fingerprint test — a nondeterministic cluster/portal build would be caught.
    #[cfg(test)]
    pub(super) fn lod_fingerprint(&self) -> u64 {
        fn mix(h: &mut u64, v: u64) {
            *h ^= v;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        match &self.lod {
            None => mix(&mut h, 0),
            Some(l) => {
                mix(&mut h, l.cluster_count as u64);
                for &c in &l.cluster_of {
                    mix(&mut h, c as u64);
                }
                for p in &l.portals {
                    mix(&mut h, p.cell as u64);
                    mix(&mut h, p.cluster as u64);
                }
                for edges in &l.abs_adj {
                    mix(&mut h, edges.len() as u64);
                    for e in edges {
                        mix(&mut h, e.to as u64);
                        mix(&mut h, e.base.to_bits() as u64);
                        mix(&mut h, e.gates as u64);
                        mix(&mut h, e.link as u64);
                    }
                }
                for reach in &l.cell_reach {
                    mix(&mut h, reach.len() as u64);
                    for r in reach {
                        mix(&mut h, r.portal as u64);
                        mix(&mut h, r.dist.to_bits() as u64);
                    }
                }
            }
        }
        h
    }

    /// Coarse travel costs from `from` under `costs`: exact within the home cluster (an in-cluster
    /// flood), and an abstract-graph estimate (a bounded overestimate) beyond it. `sever_chained`
    /// mirrors `chained_block` — `true` for goal scoring (a chained speed jump is impassable to a
    /// speed-unaware estimate), `false` for the steer corridor (it may lead through one). Cheap: one
    /// home-cluster flood plus a Dijkstra over a few hundred portals. Falls back to an exact full flood
    /// on a bare graph (no LOD tables).
    pub fn coarse_costs<'a>(&'a self, from: CellId, costs: &'a LinkCosts, sever_chained: bool) -> CoarseCosts<'a> {
        let Some(lod) = self.lod.as_ref() else {
            // No hierarchy (bare test graph): an exact full flood, read directly.
            let full = Some(self.costs_from(from, costs));
            return CoarseCosts {
                graph: self,
                costs,
                sever_chained,
                full,
                home: HashMap::new(),
                abs_cost: Vec::new(),
            };
        };
        // Home cluster, priced by an in-cluster flood (exact for cells the shortest path reaches
        // without leaving the cluster; overestimate-safe for the rest); its cells answer `cost_to`
        // directly and its portals seed the abstract search. There is deliberately no near-field flood —
        // running one per `coarse_costs` call (up to nine a pick) was the very cost the hierarchy
        // exists to avoid. The abstract graph answers everything past the home cluster.
        let home = self.home_flood(from, lod.cluster_of[from as usize], lod, costs, sever_chained);
        let mut abs_cost = vec![f32::INFINITY; lod.portals.len()];
        let mut heap = BinaryHeap::new();
        for (&cell, &seed) in &home {
            let p = lod.portal_of_cell[cell as usize];
            if p >= 0 && seed < abs_cost[p as usize] {
                abs_cost[p as usize] = seed;
                heap.push(MinNode {
                    key: seed,
                    id: p as u32,
                });
            }
        }
        // Dijkstra over the abstract graph, pricing dynamic terms on each edge as it is relaxed.
        while let Some(MinNode { key: g, id: p }) = heap.pop() {
            if g > abs_cost[p as usize] {
                continue;
            }
            for e in &lod.abs_adj[p as usize] {
                let ng =
                    g + e.base + self.price_meta(e.gates, e.rj, e.chained, e.hazard_hp, costs, sever_chained, e.link);
                if ng < abs_cost[e.to as usize] {
                    abs_cost[e.to as usize] = ng;
                    heap.push(MinNode { key: ng, id: e.to });
                }
            }
        }
        CoarseCosts {
            graph: self,
            costs,
            sever_chained,
            full: None,
            home,
            abs_cost,
        }
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
                heap.push(MinNode {
                    key: seed,
                    id: p as u32,
                });
            }
        }
        // The goal cluster's entry portals: once every one is settled its coarse cost is final (Dijkstra
        // order), so the search can stop there instead of draining every portal on the map. Unreachable
        // entry portals never settle, so a truly unreachable goal falls through to a full drain (then
        // `goal_portal` is `None` below) — same result as before, only the reachable common case is cut.
        let mut goal_pending = vec![false; np];
        let mut remaining = 0u32;
        for r in &lod.cell_reach[goal as usize] {
            if !std::mem::replace(&mut goal_pending[r.portal as usize], true) {
                remaining += 1;
            }
        }
        while let Some(MinNode { key: g, id: p }) = heap.pop() {
            if g > abs_cost[p as usize] {
                continue;
            }
            if std::mem::replace(&mut goal_pending[p as usize], false) {
                remaining -= 1;
                if remaining == 0 {
                    break; // every entry portal of the goal cluster is settled — the rest can't help
                }
            }
            for e in &lod.abs_adj[p as usize] {
                let ng = g + e.base + self.price_meta(e.gates, e.rj, e.chained, e.hazard_hp, costs, false, e.link);
                if ng < abs_cost[e.to as usize] {
                    abs_cost[e.to as usize] = ng;
                    parent[e.to as usize] = p;
                    pgate[e.to as usize] = pgate[p as usize] | e.gates;
                    heap.push(MinNode { key: ng, id: e.to });
                }
            }
        }
        // The cheapest entry portal of the goal's cluster: coarse cost into the cluster plus the priced
        // in-cluster hop to the goal — the *same* sum `CoarseCosts::cost_to` forms, so the horizon test
        // agrees with goal scoring. (Pricing the final hop matters: a goal whose last in-cluster hop
        // crosses a shut gate must read far, not near, or the corridor returns `None` and steer runs an
        // unbounded whole-graph search at a 100k-penalized goal — the very spike the corridor bounds.)
        let mut goal_portal = None;
        let mut goal_cost = f32::INFINITY;
        let mut goal_gates = 0u32;
        for r in &lod.cell_reach[goal as usize] {
            let via = abs_cost[r.portal as usize];
            if via.is_finite() {
                let c = via + r.dist + self.price_meta(r.gates, r.rj, r.chained, r.hazard_hp, costs, false, u32::MAX);
                if c < goal_cost {
                    goal_cost = c;
                    goal_portal = Some(r.portal);
                    goal_gates = r.gates;
                }
            }
        }
        let goal_portal = goal_portal?;
        if goal_cost <= horizon {
            return None;
        }
        // Walk the abstract path home→goal: flag every cluster up to the interim (the fine search's
        // window), put the interim at the first portal at/past the horizon, and collect — in route
        // order, nearest first — the gate ids crossed *beyond* the interim. The truncated route can't
        // reveal those, so the far pre-arm works the first shut one; gates within the window sit on the
        // fine route and `route_blocking_gate` catches them. A sealed (≥31) gate is no specific gate.
        let mut chain = Vec::new();
        let mut p = goal_portal;
        while p != u32::MAX {
            chain.push(p);
            p = parent[p as usize];
        }
        let mut allowed = vec![false; lod.cluster_count as usize];
        allowed[from_cl as usize] = true;
        let mut interim = goal_portal;
        let mut far_gates: Vec<u32> = Vec::new();
        let mut past_interim = false;
        let mut prev_pgate = 0u32;
        for &p in chain.iter().rev() {
            if past_interim {
                let mut bits = (pgate[p as usize] & !prev_pgate) & !SEALED_GATE;
                while bits != 0 {
                    far_gates.push(bits.trailing_zeros());
                    bits &= bits - 1;
                }
            } else {
                allowed[lod.portals[p as usize].cluster as usize] = true;
                if abs_cost[p as usize] >= horizon {
                    interim = p;
                    past_interim = true;
                }
            }
            prev_pgate = pgate[p as usize];
        }
        // The final in-cluster hop into the goal cell is beyond whatever portal the route stops at.
        let mut bits = goal_gates & !SEALED_GATE;
        while bits != 0 {
            far_gates.push(bits.trailing_zeros());
            bits &= bits - 1;
        }
        Some(Corridor {
            interim: lod.portals[interim as usize].cell,
            allowed,
            far_gates,
        })
    }

    /// Priced Dijkstra from `from` restricted to cluster `cl`, returning the exact priced cost to every
    /// cell of the home cluster. Uses the full `link_extra` pricing (gates, hazard, …) so the home
    /// region matches the fine flood exactly; its portals seed [`coarse_costs`]'s abstract search and
    /// its cells answer [`CoarseCosts::cost_to`] directly for anything past the fine cap.
    fn home_flood(
        &self,
        from: CellId,
        cl: u32,
        lod: &Lod,
        costs: &LinkCosts,
        sever_chained: bool,
    ) -> HashMap<CellId, f32> {
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
    /// price), the per-bot rocket-jump-unfit surcharge, — in scoring mode — a chained speed jump, and
    /// (for a crossing edge, whose representative fine link is `link`) that link's transient per-bot
    /// surcharge. Mirrors the terms `link_extra`/`chained_block` add per fine link. `link` is `u32::MAX`
    /// for a transit/reach hop (a multi-link intra path with no single representative); its interior
    /// links' surcharges are the accepted near-field gap — those legs sit in or near the home cluster,
    /// which the exact `home_flood` already prices in full.
    fn price_meta(
        &self,
        gates: u32,
        rj: u8,
        chained: bool,
        hazard_hp: f32,
        costs: &LinkCosts,
        sever_chained: bool,
        link: u32,
    ) -> f32 {
        let mut extra = 0.0;
        // Iterate only the gate bits actually set (single-digit gate counts, usually one), not every
        // gate id. Bit 31 (`SEALED_GATE`) is a "gate id ≥ 31 we can't track" marker, priced as an
        // always-shut door — a safe overestimate, never an underestimate. See [`SEALED_GATE`].
        let mut bits = gates & !SEALED_GATE;
        while bits != 0 {
            let gi = bits.trailing_zeros() as usize;
            bits &= bits - 1;
            if costs.gate_closed.get(gi).copied().unwrap_or(false) {
                extra += if costs.openable_gates.get(gi).copied().unwrap_or(false) {
                    costs.open_gate_cost
                } else {
                    CLOSED_GATE_PENALTY
                };
            }
        }
        if gates & SEALED_GATE != 0 {
            extra += CLOSED_GATE_PENALTY;
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
        // This crossing's own transient surcharge (a recently-failed link, or a team-occupied
        // teleport exit — always a crossing). Without it the far field would price a penalized route
        // *cheaper* than the exact flood does, an underestimate that would call an item closer than it
        // is. Mirrors `link_extra`'s per-link `penalties` scan.
        if link != u32::MAX {
            for &(li, sec) in costs.penalties {
                if li == link {
                    extra += sec;
                    break;
                }
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
    /// Gate ids the corridor crosses **beyond** the interim, nearest first — for the far button-errand
    /// pre-arm (near/home gates already sit on the fine route, caught by `route_blocking_gate`). The
    /// far block works the first currently-shut one, matching exact mode's route-order detection.
    pub far_gates: Vec<u32>,
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
    /// Coarse travel cost from the source to `cell`: home-cluster cells come from the in-cluster flood
    /// (overestimate-safe — a cell whose true shortest path leaves and re-enters the home cluster is
    /// priced high, not exact), everything else is the cheapest way into its cluster through the
    /// abstract graph plus the in-cluster hop. `INFINITY` if unreachable (the coverage pass in
    /// `build_lod` guarantees reachable ⟺ finite). A bounded overestimate — never an underestimate (an
    /// abstract path is a real path; ≥31-gate crossings are sealed shut, not dropped) — so goal scoring
    /// can trust it not to call an item closer than it is.
    pub fn cost_to(&self, cell: CellId) -> f32 {
        if let Some(full) = &self.full {
            return full.get(cell as usize).copied().unwrap_or(f32::INFINITY); // bare graph: exact flood
        }
        if let Some(&d) = self.home.get(&cell) {
            return d; // home cluster: exact
        }
        // Out of range (a goal cell held stale across a navmesh rebuild) reads as unreachable, never a
        // panic; `None` LOD tables likewise.
        let reach = match self
            .graph
            .lod
            .as_ref()
            .and_then(|lod| lod.cell_reach.get(cell as usize))
        {
            Some(r) => r,
            None => return f32::INFINITY,
        };
        let mut best = f32::INFINITY;
        for r in reach {
            let via = self.abs_cost[r.portal as usize];
            if via.is_finite() {
                let c = via
                    + r.dist
                    + self.graph.price_meta(
                        r.gates,
                        r.rj,
                        r.chained,
                        r.hazard_hp,
                        self.costs,
                        self.sever_chained,
                        u32::MAX,
                    );
                if c < best {
                    best = c;
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::super::{build_navmesh, RocketJumpParams, SpeedJumpParams, TeleportInfo};
    use super::*;
    use crate::bsp::Bsp;
    use glam::Vec3;
    use std::collections::HashSet;

    /// The quad on bravado — same reference point as `bot::tests::bravado_quad_reachability`.
    const QUAD: Vec3 = Vec3::new(752.0, 24.0, 288.0);
    /// `STEER_LOD_HORIZON` (crates/rtx-game/src/bot/mod.rs) inlined — rtx-nav can't see the game crate.
    const HORIZON: f32 = 3.0;

    /// Build the bravado navmesh the way the rjtest live server does: `rtx_doublejump 0`, grapple off,
    /// bhop + rocket-jump on. So the LOD tables this probe dissects are the ones the live bot steers on
    /// (liquid columns included — `build_navmesh` flags them on the worker). `None` when the env isn't
    /// set to bravado (the test then no-ops, mirroring `bravado_quad_reachability`).
    fn build_bravado() -> Option<NavGraph> {
        let path = std::env::var("RTX_TEST_BSP").ok()?;
        if !path.to_lowercase().contains("bravado") {
            return None;
        }
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        // The two trigger_teleports from the entity lump — both far from the quad, included for fidelity.
        let teleports = vec![
            TeleportInfo {
                tmin: Vec3::new(-63.0, 1249.0, 1.0),
                tmax: Vec3::new(31.0, 1255.0, 111.0),
                dest: Vec3::new(256.0, 1256.0, 164.0),
            },
            TeleportInfo {
                tmin: Vec3::new(-167.0, 401.0, 1.0),
                tmax: Vec3::new(-161.0, 495.0, 111.0),
                dest: Vec3::new(-664.0, 480.0, 312.0),
            },
        ];
        let graph = build_navmesh(
            &bsp,
            vec![],
            teleports,
            vec![],
            None,
            false,
            Some(SpeedJumpParams {
                gravity: 800.0,
                accel: 10.0,
                maxspeed: 320.0,
                friction: 4.0,
                stopspeed: 100.0,
                curl: false,
            }),
            Some(RocketJumpParams {
                gravity: 800.0,
                rj_extra: 0.0,
            }),
        );
        Some(graph)
    }

    /// The walk/step-connected plateau containing the quad cell (jump links excluded) — the same
    /// "on the platform" flood `bravado_quad_reachability` uses.
    fn platform_of(graph: &NavGraph, quad: CellId) -> Vec<bool> {
        let mut on = vec![false; graph.cells.len()];
        let mut stack = vec![quad];
        on[quad as usize] = true;
        while let Some(c) = stack.pop() {
            for l in &graph.links {
                let walkish = matches!(l.kind, LinkKind::Walk | LinkKind::Step);
                for (a, b) in [(l.from, l.to), (l.to, l.from)] {
                    if walkish && a == c && !on[b as usize] {
                        on[b as usize] = true;
                        stack.push(b);
                    }
                }
            }
        }
        on
    }

    /// Root-cause probe *and* regression guard for the "bot can't reach the bravado quad under LOD"
    /// bug. Dumps clustering, crossing eviction, coverage, coarse-vs-exact cost, and corridor decisions
    /// (read the output to diagnose), and asserts the invariants the storey-band fix restores: the
    /// quad's cluster stays on the plateau (not welded to the pit), a fine-reachable quad has a finite
    /// coarse cost, and the steer corridor reaches the quad from every sampled source. Pre-fix the quad
    /// cluster spanned 208u (platform + pit), every climb crossing was evicted by a cheap low walk, and
    /// the platform read `cost_to = INFINITY` from the mid-level. Run:
    /// `RTX_TEST_BSP=…/bravado.bsp cargo test -p rtx-nav bravado_quad_lod_probe -- --nocapture`.
    #[test]
    fn bravado_quad_lod_probe() {
        let Some(graph) = build_bravado() else { return };
        let lod = graph.lod.as_ref().expect("lod tables built");
        let costs = LinkCosts::default();

        let (nc, np, ne, nr) = graph.lod_stats();
        eprintln!(
            "\n=== bravado LOD probe ===\n[stats] cells={} links={} clusters={} portals={} abs_edges={} reach_entries={}",
            graph.cells.len(),
            graph.links.len(),
            nc,
            np,
            ne,
            nr
        );

        // §1 goal cell
        let quad = graph.nearest_collectable(QUAD).expect("collectable cell near quad");
        let qo = graph.cells[quad as usize].origin;
        let qcl = lod.cluster_of[quad as usize];
        eprintln!(
            "[goal] nearest_collectable=cell {} origin=({:.0},{:.0},{:.0}) gx={} gy={} cluster={}  (nearest=cell {})",
            quad,
            qo.x,
            qo.y,
            qo.z,
            graph.cells[quad as usize].gx,
            graph.cells[quad as usize].gy,
            qcl,
            graph.nearest(QUAD).unwrap()
        );

        // §2 platform set
        let on_platform = platform_of(&graph, quad);
        let plat_cells: Vec<u32> = (0..graph.cells.len() as u32)
            .filter(|&c| on_platform[c as usize])
            .collect();
        eprintln!("[platform] {} walk/step-connected cells", plat_cells.len());

        // §3 cluster census (H-A)
        let members: Vec<u32> = (0..graph.cells.len() as u32)
            .filter(|&c| lod.cluster_of[c as usize] == qcl)
            .collect();
        let zmin = members
            .iter()
            .map(|&c| graph.cells[c as usize].origin.z)
            .fold(f32::INFINITY, f32::min);
        let zmax = members
            .iter()
            .map(|&c| graph.cells[c as usize].origin.z)
            .fold(f32::NEG_INFINITY, f32::max);
        let pit = members
            .iter()
            .filter(|&&c| graph.cells[c as usize].origin.z < qo.z - 100.0)
            .count();
        let on_plat_in_cluster = members.iter().filter(|&&c| on_platform[c as usize]).count();
        let off_plat_same_z = members
            .iter()
            .filter(|&&c| !on_platform[c as usize] && (graph.cells[c as usize].origin.z - qo.z).abs() <= 48.0)
            .count();
        let mut plat_clusters: Vec<u32> = plat_cells.iter().map(|&c| lod.cluster_of[c as usize]).collect();
        plat_clusters.sort_unstable();
        plat_clusters.dedup();
        eprintln!(
            "[cluster {}] members={} z=[{:.0}..{:.0}] on_platform={} pit(z<{:.0})={} off_platform_same_z={}",
            qcl,
            members.len(),
            zmin,
            zmax,
            on_plat_in_cluster,
            qo.z - 100.0,
            pit,
            off_plat_same_z
        );
        eprintln!(
            "[cluster] platform spans clusters {:?}  => H-A {}",
            plat_clusters,
            if pit > 0 || off_plat_same_z > 0 {
                "LIKELY (cluster spills off the plateau)"
            } else {
                "not indicated"
            }
        );
        // The storey band must keep the quad's cluster on the plateau, not welded to the pit below it.
        assert!(
            zmax - zmin <= LOD_STOREY + 1.0,
            "quad cluster spans {:.0}u in Z (> one {LOD_STOREY}u storey) — platform merged with the pit",
            zmax - zmin
        );
        assert_eq!(pit, 0, "quad cluster must hold no pit cells (z < platform − 100)");

        // §4 crossing audit (H-B): which fine climb links onto the platform survived as abstract edges?
        let mut kept: HashSet<u32> = HashSet::new();
        for edges in &lod.abs_adj {
            for e in edges {
                if e.link != u32::MAX {
                    kept.insert(e.link);
                }
            }
        }
        eprintln!("[inbound climbs] non-drop links landing on the platform from off it:");
        for (li, l) in graph.links.iter().enumerate() {
            let li = li as u32;
            if !(on_platform[l.to as usize] && !on_platform[l.from as usize] && !matches!(l.kind, LinkKind::Drop)) {
                continue;
            }
            let (fc, tc) = (lod.cluster_of[l.from as usize], lod.cluster_of[l.to as usize]);
            let (fo, to) = (graph.cells[l.from as usize].origin, graph.cells[l.to as usize].origin);
            let cross = fc != tc;
            eprintln!(
                "  link {} {:?} cost={:.1}  cell {} (cl {}, z {:.0}) -> cell {} (cl {}, z {:.0})  cross={} kept={}",
                li,
                l.kind,
                l.cost,
                l.from,
                fc,
                fo.z,
                l.to,
                tc,
                to.z,
                cross,
                kept.contains(&li)
            );
            if cross && !kept.contains(&li) {
                for (lj, m) in graph.links.iter().enumerate() {
                    let lj = lj as u32;
                    if kept.contains(&lj)
                        && lod.cluster_of[m.from as usize] == fc
                        && lod.cluster_of[m.to as usize] == tc
                    {
                        eprintln!(
                            "      >>> EVICTED — pair ({fc}->{tc}) rep is link {} {:?} cost={:.1} landing z {:.0}",
                            lj, m.kind, m.cost, graph.cells[m.to as usize].origin.z
                        );
                    }
                }
            }
        }

        // §5 cell_reach of the quad + each covering portal's inbound abstract degree
        eprintln!("[cell_reach quad] {} entries:", lod.cell_reach[quad as usize].len());
        for r in &lod.cell_reach[quad as usize] {
            let p = &lod.portals[r.portal as usize];
            let indeg = lod.abs_adj.iter().flatten().filter(|e| e.to == r.portal).count();
            eprintln!(
                "  portal {} = cell {} (cl {}, z {:.0}) dist={:.1} inbound_abs_edges={}",
                r.portal, p.cell, p.cluster, graph.cells[p.cell as usize].origin.z, r.dist, indeg
            );
        }

        // §6/§7 coarse-vs-exact + corridor + convergence, from sampled sources
        let mut sources: Vec<u32> = graph
            .links
            .iter()
            .filter(|l| {
                on_platform[l.to as usize] && !on_platform[l.from as usize] && !matches!(l.kind, LinkKind::Drop)
            })
            .map(|l| l.from)
            .collect();
        sources.sort_unstable();
        sources.dedup();
        // A few off-platform vantage points: west of the launch ledge, and the two teleport dests.
        for p in [
            Vec3::new(560.0, 24.0, 288.0),
            Vec3::new(-664.0, 480.0, 312.0),
            Vec3::new(256.0, 1256.0, 164.0),
        ] {
            if let Some(c) = graph.nearest(p) {
                if !sources.contains(&c) {
                    sources.push(c);
                }
            }
        }

        eprintln!("[costs + corridor per source]");
        for &s in &sources {
            let so = graph.cells[s as usize].origin;
            let reach = graph.reachable(s, quad);
            let exact = graph.costs_from(s, &costs)[quad as usize];
            let score = graph.coarse_costs(s, &costs, true);
            let cs = score.cost_to(quad);
            let cc = graph.coarse_costs(s, &costs, false).cost_to(quad);
            eprintln!(
                "  src cell {} ({:.0},{:.0},{:.0}) cl {}: reachable={} exact={:.1} coarse_score={:.1} coarse_corr={:.1}{}",
                s,
                so.x,
                so.y,
                so.z,
                lod.cluster_of[s as usize],
                reach,
                exact,
                cs,
                cc,
                if reach && !cs.is_finite() { "   <<< reachable but INF (H-C)" } else { "" }
            );
            // The invariant the fix restores: a fine-reachable quad has a finite coarse cost.
            assert_eq!(
                reach,
                cs.is_finite(),
                "src {s}: reachable={reach} but coarse-finite={} — reachable⟺finite broken",
                cs.is_finite()
            );
            for r in &lod.cell_reach[quad as usize] {
                eprintln!(
                    "      via portal {} abs_cost={:.1}",
                    r.portal, score.abs_cost[r.portal as usize]
                );
            }
            match graph.corridor(s, quad, &costs, HORIZON) {
                None => eprintln!("      corridor=None (steer directly, unrestricted)"),
                Some(c) => {
                    let win = c.allowed.iter().filter(|&&b| b).count();
                    let giw = c.allowed.get(qcl as usize).copied().unwrap_or(false);
                    eprintln!(
                        "      corridor: interim=cell {} (z {:.0}) window={} clusters goal_cluster_in_window={}{}",
                        c.interim,
                        graph.cells[c.interim as usize].origin.z,
                        win,
                        giw,
                        if !giw {
                            "   <<< quad cluster EXCLUDED from window (H-B)"
                        } else {
                            ""
                        }
                    );
                }
            }
        }

        // Convergence: replay steer's repath cadence (corridor → windowed_route → advance) from each
        // source and see whether the fine bot ever reaches the quad, orbits, or strands.
        eprintln!("[convergence] (corridor-follow, mirroring steer::windowed_route):");
        for &s in &sources {
            let mut cur = s;
            let mut seen = HashSet::new();
            let mut hops = 0u32;
            let verdict = loop {
                if cur == quad {
                    break format!("ARRIVED in {hops} hops");
                }
                match graph.corridor(cur, quad, &costs, HORIZON) {
                    None => {
                        let r = graph.find_path(cur, quad, &costs).unwrap_or_default();
                        break if r.is_empty() {
                            format!("DIRECT-EMPTY after {hops} hops (cell {cur})")
                        } else {
                            format!("CONVERGED-DIRECT after {hops} hops + {} fine legs", r.len())
                        };
                    }
                    Some(c) => {
                        let mut route = graph
                            .find_path_within(cur, c.interim, &costs, &c.allowed)
                            .unwrap_or_default();
                        if route.is_empty() {
                            route = graph.find_path(cur, c.interim, &costs).unwrap_or_default();
                        }
                        if route.is_empty() {
                            break format!("STRANDED at cell {cur} (interim {} unreachable)", c.interim);
                        }
                        cur = graph.links[*route.last().unwrap() as usize].to;
                        hops += 1;
                        if !seen.insert(cur) {
                            break format!("LOOP — revisited cell {cur} after {hops} hops");
                        }
                        if hops >= 40 {
                            break format!("NO-CONVERGE in 40 hops (last cell {cur})");
                        }
                    }
                }
            };
            eprintln!("  src cell {} -> {}", s, verdict);
            assert!(
                verdict.starts_with("ARRIVED") || verdict.starts_with("CONVERGED"),
                "src {s}: corridor-follow never reached the quad — {verdict}"
            );
        }
        eprintln!("=== end probe ===\n");
    }
}
