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

use super::{CellId, NavGraph};

/// Grid columns per cluster-block edge, as a shift. `3` → `1<<3 = 8` columns → `8 · GRID = 256u`
/// blocks. Bigger blocks mean fewer, coarser clusters (cheaper abstract graph, blunter estimates).
const LOD_SHIFT: i32 = 3;

/// The level-of-detail tables. Grown incrementally; today it holds the cluster assignment.
pub(super) struct Lod {
    /// Cluster id of each cell, parallel to `cells`, dense in `0..cluster_count`.
    cluster_of: Vec<u32>,
    cluster_count: u32,
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

impl NavGraph {
    /// Build the LOD tables. Called once at the end of the navmesh build, after all splices, so it
    /// sees every link. O(V + E). Serial — cheap next to the parallel cell/link solve.
    pub(super) fn build_lod(&mut self) {
        let n = self.cells.len();
        if n == 0 {
            self.lod = None;
            return;
        }
        // Weakly connect cells joined by a link that stays inside one block. A one-way drop still
        // unions its endpoints (grouping is undirected); the directed intra-cluster distances added
        // later carry the asymmetry. Cross-block links are portals, handled by the abstract graph.
        let mut uf = UnionFind::new(n);
        for link in &self.links {
            let a = &self.cells[link.from as usize];
            let b = &self.cells[link.to as usize];
            if block_of(a.gx, a.gy) == block_of(b.gx, b.gy) {
                uf.union(link.from, link.to);
            }
        }
        // Compact union roots to dense cluster ids in order of first appearance — deterministic, so
        // the build stays bit-reproducible (the `build_deterministic` fingerprint covers it).
        let mut cluster_of = vec![u32::MAX; n];
        let mut root_id: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
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
        self.lod = Some(Lod { cluster_of, cluster_count: count });
    }

    /// The cluster id of cell `c`, or `None` when the LOD layer isn't built (bare test graphs).
    pub fn cluster_of(&self, c: CellId) -> Option<u32> {
        self.lod.as_ref().map(|l| l.cluster_of[c as usize])
    }

    /// How many clusters the LOD layer has (0 when unbuilt).
    pub fn cluster_count(&self) -> usize {
        self.lod.as_ref().map_or(0, |l| l.cluster_count as usize)
    }
}
