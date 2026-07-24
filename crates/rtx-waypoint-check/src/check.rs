// SPDX-License-Identifier: AGPL-3.0-or-later

//! Building the navmesh offline and classifying each KTX rocket-jump / curl-jump path against it.
//!
//! The build recipe is the viewer's ([`rtx_nav_view`-equivalent](crate)): stock DM physics with
//! double-jump, bunnyhop speed-jumps (curl on), and rocket-jumps enabled. The one addition is
//! wiring teleporters from the entity lump ([`crate::ent::teleports`]) so teleport-riding routes
//! resolve; plats and button-gated doors still aren't spliced offline (their traversal needs the
//! live movers), which the report flags per map.
//!
//! For each authored path A→B we ask how well our mesh reproduces it, in descending strength:
//!
//! - **Matched** — a link of the *same kind* (rocket jump for an RJ path, curl speed-jump for a curl
//!   path) leaves near A and lands near B.
//! - **JumpConnected** — some *other* airborne link bridges the same endpoints; the mesh crosses the
//!   gap, just by different means.
//! - **RouteConnected** — no single link matches, but A and B snap to the mesh and a route exists.
//! - **Unreachable** — the blind spot: A and B snap, but the mesh can't get from one to the other.
//! - **Unsnapped** — an endpoint didn't land on any nav cell (a marker over a pedestal, water, or
//!   void), so no honest verdict is possible.

use glam::Vec3;
use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::{
    build_navmesh, LinkCosts, LinkKind, NavGraph, RocketJumpParams, SpeedJumpParams, CLOSED_GATE_PENALTY,
};

use crate::botfile::ResolvedPath;

/// Vertical window (units) within which two points count as "the same storey" for endpoint
/// matching. Both marker and cell origins hover ~24u over the floor with author-dependent slop, so
/// horizontal distance is what discriminates a match; a full storey must not blur into one.
const Z_TOL: f32 = 64.0;

/// Which family of authored path we're checking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    RocketJump,
    Curl,
}

/// A candidate nav link and how far its endpoints sit from the authored path's.
#[derive(Clone, Copy, Debug)]
pub struct Near {
    pub link: u32,
    pub kind: LinkKind,
    pub d_src: f32,
    pub d_tgt: f32,
}

/// How well the mesh reproduces one authored path.
#[derive(Clone, Copy, Debug)]
pub enum Verdict {
    Matched(Near),
    JumpConnected(Near),
    RouteConnected {
        cost: f32,
        legs: usize,
        jump_legs: usize,
        /// The only route is penalty-priced (a shut gate or a chained speed-jump plain A* blocks).
        degenerate: bool,
    },
    Unreachable {
        nearest_kindred: Option<Near>,
    },
    Unsnapped {
        end: &'static str,
        dist: f32,
    },
}

/// Build the navmesh the checker compares against — the viewer's stock-DM recipe plus teleports.
pub fn build(bsp: &Bsp) -> NavGraph {
    build_navmesh(
        bsp,
        Vec::new(),
        crate::ent::teleports(bsp),
        Vec::new(),
        None,
        true,
        Some(SpeedJumpParams {
            gravity: 800.0,
            accel: 10.0,
            maxspeed: 320.0,
            friction: 4.0,
            stopspeed: 100.0,
            curl: true,
        }),
        Some(RocketJumpParams {
            gravity: 800.0,
            rj_extra: 0.0,
        }),
    )
}

/// A navmesh plus the link indices grouped by family, so each path is checked without re-scanning.
pub struct Checker<'a> {
    pub graph: &'a NavGraph,
    radius: f32,
    rj_links: Vec<u32>,
    curl_links: Vec<u32>,
    /// Every airborne link (jump/double/speed/rocket) — the pool for the JumpConnected fallback.
    airborne: Vec<u32>,
}

impl<'a> Checker<'a> {
    pub fn new(graph: &'a NavGraph, radius: f32) -> Self {
        let mut rj_links = Vec::new();
        let mut curl_links = Vec::new();
        let mut airborne = Vec::new();
        for li in 0..graph.links.len() as u32 {
            match graph.link_kind(li) {
                LinkKind::RocketJump => {
                    rj_links.push(li);
                    airborne.push(li);
                }
                LinkKind::SpeedJump => {
                    airborne.push(li);
                    if graph.speed_jump_of_link(li).is_some_and(|s| s.curl_gain > 0.0) {
                        curl_links.push(li);
                    }
                }
                LinkKind::JumpGap | LinkKind::DoubleJump => airborne.push(li),
                _ => {}
            }
        }
        Checker {
            graph,
            radius,
            rj_links,
            curl_links,
            airborne,
        }
    }

    pub fn rj_link_count(&self) -> usize {
        self.rj_links.len()
    }
    pub fn curl_link_count(&self) -> usize {
        self.curl_links.len()
    }

    /// Classify one authored path within the given family.
    pub fn classify(&self, p: &ResolvedPath, fam: Family) -> Verdict {
        let g = self.graph;
        let r = self.radius;
        let a = p.from.pos();
        let b = p.to.pos();
        let nb = g.nearest(b);
        let kindred = match fam {
            Family::RocketJump => &self.rj_links,
            Family::Curl => &self.curl_links,
        };

        // Strongest: a same-kind link whose endpoints land within tolerance.
        let mut best: Option<(f32, Near)> = None;
        for &li in kindred {
            let ds = self.d_src_window(li, a);
            let dt = dist_window(g.cell_origin(g.link_target(li)), b);
            if !ds.is_finite() || !dt.is_finite() {
                continue;
            }
            let accepted =
                ds <= r && (dt <= r || (dt <= 3.0 * r && nb.is_some_and(|nb| self.same_shelf(g.link_target(li), nb))));
            if accepted {
                let score = ds.max(dt);
                let near = Near {
                    link: li,
                    kind: g.link_kind(li),
                    d_src: ds,
                    d_tgt: dt,
                };
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    best = Some((score, near));
                }
            }
        }
        if let Some((_, near)) = best {
            return Verdict::Matched(near);
        }

        // Next: some other airborne link bridges the same gap.
        if let Some(near) = self.jump_connected(fam, a, b, nb) {
            return Verdict::JumpConnected(near);
        }

        // Otherwise fall back to snapping + reachability.
        let Some(ca) = g.nearest(a) else {
            return Verdict::Unsnapped {
                end: "src",
                dist: f32::INFINITY,
            };
        };
        let Some(cb) = nb else {
            return Verdict::Unsnapped {
                end: "tgt",
                dist: f32::INFINITY,
            };
        };
        let snap_a = (g.cell_origin(ca) - a).length();
        let snap_b = (g.cell_origin(cb) - b).length();
        if snap_a > 2.0 * r {
            return Verdict::Unsnapped {
                end: "src",
                dist: snap_a,
            };
        }
        if snap_b > 2.0 * r {
            return Verdict::Unsnapped {
                end: "tgt",
                dist: snap_b,
            };
        }
        if g.reachable(ca, cb) {
            match g.find_path(ca, cb, &LinkCosts::default()) {
                Some(route) => {
                    let cost: f32 = route.iter().map(|&li| g.link_cost(li)).sum();
                    let jump_legs = route.iter().filter(|&&li| is_airborne(g.link_kind(li))).count();
                    Verdict::RouteConnected {
                        cost,
                        legs: route.len(),
                        jump_legs,
                        degenerate: cost >= CLOSED_GATE_PENALTY,
                    }
                }
                // Reachable per the closure, but plain A* priced the only route away (a chained
                // speed-jump it blocks): still connected, but degenerately.
                None => Verdict::RouteConnected {
                    cost: f32::INFINITY,
                    legs: 0,
                    jump_legs: 0,
                    degenerate: true,
                },
            }
        } else {
            Verdict::Unreachable {
                nearest_kindred: self.nearest_kindred_3d(kindred, a, b),
            }
        }
    }

    /// A source-anchor distance under the z-window metric: the nearer of the link's source cell and,
    /// for a speed jump, its takeoff ledge (a speed jump's `from` is the runway start, not the ledge).
    fn d_src_window(&self, li: u32, a: Vec3) -> f32 {
        let g = self.graph;
        let mut best = dist_window(g.cell_origin(g.link_source(li)), a);
        if let Some(sj) = g.speed_jump_of_link(li) {
            best = best.min(dist_window(sj.takeoff, a));
        }
        best
    }

    fn jump_connected(&self, fam: Family, a: Vec3, b: Vec3, nb: Option<u32>) -> Option<Near> {
        let g = self.graph;
        let r = self.radius;
        let mut best: Option<(f32, Near)> = None;
        for &li in &self.airborne {
            let is_kindred = match fam {
                Family::RocketJump => g.link_kind(li) == LinkKind::RocketJump,
                Family::Curl => g.speed_jump_of_link(li).is_some_and(|s| s.curl_gain > 0.0),
            };
            if is_kindred {
                continue;
            }
            let ds = self.d_src_window(li, a);
            let dt = dist_window(g.cell_origin(g.link_target(li)), b);
            if !ds.is_finite() || !dt.is_finite() {
                continue;
            }
            let accepted =
                ds <= r && (dt <= r || (dt <= 3.0 * r && nb.is_some_and(|nb| self.same_shelf(g.link_target(li), nb))));
            if accepted {
                let score = ds.max(dt);
                let near = Near {
                    link: li,
                    kind: g.link_kind(li),
                    d_src: ds,
                    d_tgt: dt,
                };
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    best = Some((score, near));
                }
            }
        }
        best.map(|(_, n)| n)
    }

    /// Whether `y` is reachable from `x` over flat ground (Walk/Step) within 6 hops — the same
    /// shelf. Absorbs our rocket-jump solver landing on the same ledge but an offset cell.
    fn same_shelf(&self, x: u32, y: u32) -> bool {
        if x == y {
            return true;
        }
        let g = self.graph;
        let mut seen = std::collections::HashSet::new();
        seen.insert(x);
        let mut frontier = vec![x];
        for _ in 0..6 {
            let mut next = Vec::new();
            for c in frontier {
                for &li in &g.adjacency[c as usize] {
                    if matches!(g.link_kind(li), LinkKind::Walk | LinkKind::Step) {
                        let t = g.link_target(li);
                        if t == y {
                            return true;
                        }
                        if seen.insert(t) {
                            next.push(t);
                        }
                    }
                }
            }
            if next.is_empty() {
                break;
            }
            frontier = next;
        }
        false
    }

    /// The nearest same-kind link by raw 3D endpoint distance (no z-window) — the number that tells
    /// whether an unreachable path is a radius-tuning miss or a genuine hole.
    fn nearest_kindred_3d(&self, kindred: &[u32], a: Vec3, b: Vec3) -> Option<Near> {
        let g = self.graph;
        kindred
            .iter()
            .map(|&li| {
                let mut ds = (g.cell_origin(g.link_source(li)) - a).length();
                if let Some(sj) = g.speed_jump_of_link(li) {
                    ds = ds.min((sj.takeoff - a).length());
                }
                let dt = (g.cell_origin(g.link_target(li)) - b).length();
                (
                    ds.max(dt),
                    Near {
                        link: li,
                        kind: g.link_kind(li),
                        d_src: ds,
                        d_tgt: dt,
                    },
                )
            })
            .min_by(|x, y| x.0.total_cmp(&y.0))
            .map(|(_, n)| n)
    }
}

/// Horizontal distance if the two points share a storey (`|Δz| ≤ Z_TOL`), else infinite.
fn dist_window(p: Vec3, q: Vec3) -> f32 {
    if (p.z - q.z).abs() <= Z_TOL {
        ((p.x - q.x).powi(2) + (p.y - q.y).powi(2)).sqrt()
    } else {
        f32::INFINITY
    }
}

fn is_airborne(k: LinkKind) -> bool {
    matches!(
        k,
        LinkKind::JumpGap | LinkKind::DoubleJump | LinkKind::SpeedJump | LinkKind::RocketJump | LinkKind::Hook
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_metric_respects_storeys() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let same = Vec3::new(30.0, 40.0, 40.0); // Δz 40 ≤ 64 → 3-4-5 horizontal = 50
        assert_eq!(dist_window(a, same), 50.0);
        let upstairs = Vec3::new(30.0, 40.0, 200.0); // Δz 200 > 64 → infinite
        assert!(dist_window(a, upstairs).is_infinite());
    }

    /// Full-pipeline check against a real install, gated on the same env idiom as the rtx-nav /
    /// rtx-game tests. `cargo test` runs with the crate dir as CWD, so pass **absolute** paths:
    ///   RTX_TEST_BASEDIR="$PWD/playground" RTX_TEST_WAYPOINTS="$PWD/waypoints" \
    ///     cargo test -p rtx-waypoint-check --release -- --nocapture ktx_dm4
    #[test]
    fn ktx_dm4_end_to_end() {
        let (Ok(base), Ok(wp)) = (std::env::var("RTX_TEST_BASEDIR"), std::env::var("RTX_TEST_WAYPOINTS")) else {
            eprintln!("RTX_TEST_BASEDIR / RTX_TEST_WAYPOINTS not set; skipping");
            return;
        };
        let text = std::fs::read_to_string(std::path::Path::new(&wp).join("dm4.bot")).expect("dm4.bot");
        let bytes = crate::pak::resolve_bsp(std::path::Path::new(&base), "dm4").expect("dm4.bsp");
        let bsp = Bsp::parse(&bytes).expect("parse dm4.bsp");

        let file = crate::botfile::parse(&text);
        let markers = crate::ent::marker_walk(&bsp);
        assert_eq!(markers.len(), 54, "entity-walk K");
        assert_eq!(file.implied_entity_markers(), 54, "file-implied K");

        let (paths, dropped) = crate::botfile::resolve(&file, &markers);
        assert_eq!(dropped, 0, "every path reference resolves");
        let rj = paths.iter().filter(|p| p.is_rj()).count();
        let curl = paths.iter().filter(|p| p.is_curl()).count();
        assert_eq!(rj, 23, "dm4 rocket-jump paths");
        assert_eq!(curl, 1, "dm4 curl paths");

        let graph = build(&bsp);
        let checker = Checker::new(&graph, 96.0);
        for p in paths.iter().filter(|p| p.is_rj()) {
            assert!(
                !matches!(checker.classify(p, Family::RocketJump), Verdict::Unsnapped { .. }),
                "rj {}->{} should snap",
                p.src,
                p.dst
            );
        }
    }
}
