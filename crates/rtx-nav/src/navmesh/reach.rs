// SPDX-License-Identifier: AGPL-3.0-or-later

//! Static reachability over the built graph: "can a bot at cell A ever get to cell B?"
//!
//! This is a *topological* question, not a routing one. Every dynamic cost term the queries layer on
//! ([`LinkCosts`](super::LinkCosts): closed gates, rocket-jump unfitness, failed-link surcharges,
//! hazard/water prices, jitter) is **finite** — a shut door is charged [`CLOSED_GATE_PENALTY`], not
//! infinity — so no live state ever *severs* a link. Whether B is reachable from A therefore never
//! changes at runtime and can be answered from the graph's fixed structure, in O(1), instead of by
//! exhausting an A* and watching it come back empty (see [`super::NavGraph::find_path`]) or by running a
//! whole-graph flood ([`super::NavGraph::costs_from`]) just to ask "did we reach it?".
//!
//! Cells collapse into strongly-connected components (an SCC is a maximal set of cells mutually
//! reachable from each other); the components form a DAG. We compute the components once (Tarjan) and
//! the forward transitive closure over that far-smaller DAG as a bitset per component — for real maps
//! the SCC count is in the hundreds (the main walkable mass is one giant component; one-way links —
//! drops, teleports, unfit rocket jumps — carve off the rest), so the closure is a few tens of KB.
//!
//! [`CLOSED_GATE_PENALTY`]: super::CLOSED_GATE_PENALTY

use super::{CellId, NavGraph};

/// Precomputed strongly-connected components plus the forward reachability closure over their
/// condensation DAG. Built once at the end of the navmesh build (all link splices done); immutable
/// thereafter, so it is safe to share `&Reach` across threads.
pub(super) struct Reach {
    /// The SCC id of each cell (parallel to `cells`), in `0..count`.
    scc: Vec<u32>,
    /// Row-major bitset: row `s` (a `stride`-word slice) has bit `t` set iff SCC `t` is reachable
    /// from SCC `s` (including `s` itself). `count × stride` words.
    closure: Vec<u64>,
    /// Words per closure row, `ceil(count / 64)`.
    stride: usize,
}

impl Reach {
    /// Whether cell `to` is reachable from cell `from` (an O(1) bitset test on their SCCs).
    #[inline]
    fn get(&self, from: CellId, to: CellId) -> bool {
        let (s, t) = (self.scc[from as usize] as usize, self.scc[to as usize] as usize);
        self.closure[s * self.stride + (t >> 6)] & (1u64 << (t & 63)) != 0
    }
}

impl NavGraph {
    /// Compute the reachability table (SCCs + forward closure). Called once by `build_navmesh` after
    /// every link splice has landed; O(V + E) plus the closure fill. Serial — cheap next to the
    /// parallel cell/link solve, and it needs the finished link set.
    pub(super) fn build_reachability(&mut self) {
        let n = self.cells.len();
        if n == 0 {
            self.reach = None;
            return;
        }
        let (scc, count) = self.tarjan_scc();
        // Condensation edges, deduplicated per source SCC. Tarjan numbers components in reverse
        // topological order, so every edge runs from a higher SCC id to a lower one — which lets the
        // closure below fill in a single increasing-id pass.
        let mut dag: Vec<Vec<u32>> = vec![Vec::new(); count];
        for u in 0..n {
            let su = scc[u];
            for &li in &self.adjacency[u] {
                let sv = scc[self.links[li as usize].to as usize];
                if sv != su {
                    dag[su as usize].push(sv);
                }
            }
        }
        for succ in &mut dag {
            succ.sort_unstable();
            succ.dedup();
        }
        // Forward closure: closure[s] = {s} ∪ ⋃ closure[t] for every DAG edge s→t. Because t < s for
        // every such edge, processing s in increasing order means each row it unions is already final.
        let stride = count.div_ceil(64);
        // The closure is count × stride words (≈ count²/8 bytes). A navmesh is dominated by two-way
        // Walk/Step links, so the walkable mass collapses into a handful of SCCs and this is tens of KB.
        // Guard the theoretical all-one-way-links case (count ≈ cells) so a pathological map degrades to
        // "no table" (conservative `reachable`, flood fallback) instead of a 100 MB allocation.
        const MAX_CLOSURE_WORDS: usize = 8 << 20; // 64 MiB
        if count.saturating_mul(stride) > MAX_CLOSURE_WORDS {
            self.reach = None;
            return;
        }
        let mut closure = vec![0u64; count * stride];
        for s in 0..count {
            let base = s * stride;
            closure[base + (s >> 6)] |= 1u64 << (s & 63);
            for &t in &dag[s] {
                let tbase = t as usize * stride;
                for w in 0..stride {
                    closure[base + w] |= closure[tbase + w];
                }
            }
        }
        self.reach = Some(Reach { scc, closure, stride });
    }

    /// Iterative Tarjan: assign each cell an SCC id in `0..count`, ids handed out in the order
    /// components are finalized (reverse topological order of the condensation). Iterative rather than
    /// recursive so a long one-way corridor can't blow the stack on a big map.
    fn tarjan_scc(&self) -> (Vec<u32>, usize) {
        let n = self.cells.len();
        const UNVISITED: u32 = u32::MAX;
        let mut index = vec![UNVISITED; n]; // DFS discovery order
        let mut low = vec![0u32; n]; // lowest index reachable
        let mut on_stack = vec![false; n];
        let mut comp = vec![UNVISITED; n]; // final SCC id
        let mut stack: Vec<u32> = Vec::new(); // Tarjan's component stack
        let mut next_index = 0u32;
        let mut next_comp = 0u32;

        // Explicit DFS stack of `(cell, next adjacency slot to visit)`.
        let mut dfs: Vec<(u32, usize)> = Vec::new();
        for start in 0..n as u32 {
            if index[start as usize] != UNVISITED {
                continue;
            }
            dfs.push((start, 0));
            while let Some(&(v, slot)) = dfs.last() {
                let vu = v as usize;
                if slot == 0 {
                    index[vu] = next_index;
                    low[vu] = next_index;
                    next_index += 1;
                    stack.push(v);
                    on_stack[vu] = true;
                }
                // Advance to the next unexplored successor of `v`.
                let adj = &self.adjacency[vu];
                if slot < adj.len() {
                    dfs.last_mut().unwrap().1 += 1;
                    let w = self.links[adj[slot] as usize].to;
                    if index[w as usize] == UNVISITED {
                        dfs.push((w, 0)); // recurse into w
                    } else if on_stack[w as usize] {
                        low[vu] = low[vu].min(index[w as usize]); // back/cross edge into the stack
                    }
                    continue;
                }
                // All successors explored: `v` closes. If it roots an SCC, pop it off the stack.
                dfs.pop();
                if let Some(&(parent, _)) = dfs.last() {
                    low[parent as usize] = low[parent as usize].min(low[vu]);
                }
                if low[vu] == index[vu] {
                    loop {
                        let w = stack.pop().unwrap();
                        on_stack[w as usize] = false;
                        comp[w as usize] = next_comp;
                        if w == v {
                            break;
                        }
                    }
                    next_comp += 1;
                }
            }
        }
        (comp, next_comp as usize)
    }

    /// Whether a bot at `from` can ever reach `to` over the graph, regardless of live door/hazard
    /// state (all of which is finitely priced, never blocking — see the module docs). O(1). A graph
    /// without a built table (bare test graphs) answers `true` for any in-range cell — the caller
    /// then falls back to a search, exactly as before this table existed.
    #[inline]
    pub fn reachable(&self, from: CellId, to: CellId) -> bool {
        match &self.reach {
            Some(r) => r.get(from, to),
            None => (to as usize) < self.cells.len() && (from as usize) < self.cells.len(),
        }
    }

    /// A stable FNV-1a hash of the reachability tables (SCC ids + closure bitset), for the
    /// build-determinism fingerprint test.
    #[cfg(test)]
    pub(super) fn reach_fingerprint(&self) -> u64 {
        fn mix(h: &mut u64, v: u64) {
            *h ^= v;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        match &self.reach {
            None => mix(&mut h, 0),
            Some(r) => {
                for &s in &r.scc {
                    mix(&mut h, s as u64);
                }
                for &w in &r.closure {
                    mix(&mut h, w);
                }
            }
        }
        h
    }
}
