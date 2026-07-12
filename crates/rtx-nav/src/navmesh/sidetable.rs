// SPDX-License-Identifier: AGPL-3.0-or-later

//! A sparse per-link payload column, index-parallel to [`NavGraph::links`](super::NavGraph). The
//! graph carries five of these — gates, hooks, speed jumps, rocket jumps, plats — each mapping a
//! link to the extra data its kind needs (the door it depends on, the arc to fly, the lift to
//! board). Before this they were five hand-kept `Vec<i32>` + `Vec<Payload>` pairs, each with its own
//! copy of the "resize the index vec to `links.len()` in step, push the payload, push its index"
//! dance — a standing invitation to a keep-in-step bug. Factored into one type.
//!
//! Two shapes of tag: **1:1** (a hook/speed-jump/rocket-jump link owns one payload — `push` then
//! `tag` the just-added link) and **n:1** (many links board the same plat, or pass through the same
//! closed gate — `push` once, `tag` each affected link with the shared index). Both read back the
//! same way. Append-only and `Vec`-backed, so iteration/index order is exactly link push order (the
//! serial splice) — determinism is structural, never hashed.

/// A payload column keyed by link index. `idx[li] == -1` marks an untagged link; a link index past
/// the end of `idx` also reads as untagged (the column need not extend to the full link count).
pub(super) struct SideTable<T> {
    idx: Vec<i32>,
    items: Vec<T>,
}

impl<T> Default for SideTable<T> {
    fn default() -> Self {
        SideTable {
            idx: Vec::new(),
            items: Vec::new(),
        }
    }
}

impl<T> SideTable<T> {
    /// Register a payload, returning its index — pass that to [`tag`](Self::tag) for each link that
    /// uses it.
    pub(super) fn push(&mut self, item: T) -> usize {
        self.items.push(item);
        self.items.len() - 1
    }

    /// Tag link `li` with payload index `item`, padding any intervening links as untagged.
    pub(super) fn tag(&mut self, li: usize, item: usize) {
        if self.idx.len() <= li {
            self.idx.resize(li + 1, -1);
        }
        self.idx[li] = item as i32;
    }

    /// The payload for link `li`, if tagged.
    pub(super) fn of_link(&self, li: u32) -> Option<&T> {
        self.index_of_link(li).and_then(|i| self.items.get(i))
    }

    /// The payload *index* for link `li`, if tagged (for callers that key other tables by it).
    pub(super) fn index_of_link(&self, li: u32) -> Option<usize> {
        match self.idx.get(li as usize).copied().unwrap_or(-1) {
            i if i >= 0 => Some(i as usize),
            _ => None,
        }
    }

    /// Number of registered payloads.
    pub(super) fn len(&self) -> usize {
        self.items.len()
    }

    /// The `i`-th payload (panics if out of range — callers hold indices from [`push`](Self::push)).
    pub(super) fn item(&self, i: usize) -> &T {
        &self.items[i]
    }

    /// The raw per-link index column — for the deterministic-build fingerprint only.
    #[cfg(test)]
    pub(super) fn idx_raw(&self) -> &[i32] {
        &self.idx
    }

    /// The raw payload slice — for the deterministic-build fingerprint only.
    #[cfg(test)]
    pub(super) fn items_raw(&self) -> &[T] {
        &self.items
    }
}
