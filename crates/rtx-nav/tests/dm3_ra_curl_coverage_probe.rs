// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only diagnostic: with curl-jump generation ON, does the DM3 navmesh
//! emit speed-jump/curl links covering the two offline-certified RA-chain
//! curls (gap-1 onto the z56/z72 mid floor, upper curl onto the RA stair
//! foot), and what route does the banded planner pick RA-spawn -> RA?
//! Env-gated on RTX_TEST_BSP; changes nothing.

use glam::Vec3;
use rtx_nav::navmesh::{build_navmesh, CellId, LinkCosts, LinkKind, NavGraph, SpeedJumpParams};

fn params(curl: bool) -> SpeedJumpParams {
    SpeedJumpParams { gravity: 800.0, accel: 10.0, maxspeed: 320.0, friction: 4.0, stopspeed: 100.0, curl }
}

fn exact_cell(graph: &NavGraph, p: Vec3) -> CellId {
    let (id, miss) = graph
        .cells
        .iter()
        .enumerate()
        .map(|(id, cell)| (id as CellId, (cell.origin - p).length()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("non-empty navmesh");
    assert!(miss < 0.1, "expected exact cell at {p:?}, nearest miss {miss}");
    id
}

#[test]
fn dm3_ra_curl_coverage() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(&path).expect("read bsp");

    // double_jump=false mirrors the live green config (rtx_doublejump 0).
    for curl in [false, true] {
        let build = build_navmesh(bytes.clone(), vec![], vec![], vec![], None, false, Some(params(curl)), None)
            .expect("build navmesh");
        let graph = &build.1;
        let summary = graph.summary();
        eprintln!(
            "curl={curl}: cells={} links={} speed_jump={}",
            graph.cells.len(),
            graph.links.len(),
            summary.speed_jump
        );

        // The certified geometries, by landing area.
        let gap1_targets: Vec<CellId> = [
            (352.0, -544.0, 56.0),
            (352.0, -576.0, 56.0),
            (384.0, -544.0, 56.0),
            (384.0, -576.0, 56.0),
            (320.0, -576.0, 72.0),
            (320.0, -544.0, 72.0),
        ]
        .iter()
        .map(|&(x, y, z)| exact_cell(graph, Vec3::new(x, y, z)))
        .collect();
        let upper_targets: Vec<CellId> = [
            (32.0, -864.0, 152.0),
            (32.0, -832.0, 152.0),
            (64.0, -864.0, 168.0),
            (64.0, -832.0, 168.0),
            (96.0, -864.0, 184.0),
            (96.0, -832.0, 184.0),
        ]
        .iter()
        .map(|&(x, y, z)| exact_cell(graph, Vec3::new(x, y, z)))
        .collect();

        for (li, link) in graph.links.iter().enumerate() {
            if link.kind != LinkKind::SpeedJump {
                continue;
            }
            let hit_gap1 = gap1_targets.contains(&link.to);
            let hit_upper = upper_targets.contains(&link.to);
            if !(hit_gap1 || hit_upper) {
                continue;
            }
            let from = graph.cells[link.from as usize].origin;
            let to = graph.cells[link.to as usize].origin;
            let tag = if hit_gap1 { "GAP1" } else { "UPPER" };
            let tr = graph.speed_jump_of_link(li as u32);
            let trs = tr
                .map(|t| {
                    format!(
                        "takeoff=({:.0},{:.0},{:.0}) v_req={:.1} airtime={:.3} chained={} curl_gain={:.1} switch={:.0}",
                        t.takeoff.x, t.takeoff.y, t.takeoff.z, t.v_req, t.airtime, t.chained, t.curl_gain, t.curl_switch_dist
                    )
                })
                .unwrap_or_else(|| "none".into());
            eprintln!(
                "{tag} sj link {}->{} ({:.0},{:.0},{:.0})->({:.0},{:.0},{:.0}) cost={:.3} {trs}",
                link.from, link.to, from.x, from.y, from.z, to.x, to.y, to.z, link.cost
            );
        }

        // Segment diagnosis (curl=true only): can the banded planner walk the
        // certified chain piecewise, and what does each piece cost?
        if curl {
            let seg = |label: &str, s: (f32, f32, f32), g: (f32, f32, f32), v0: f32| {
                let sc = exact_cell(graph, Vec3::new(s.0, s.1, s.2));
                let gc = exact_cell(graph, Vec3::new(g.0, g.1, g.2));
                match graph.find_path_banded(sc, gc, v0, &LinkCosts::default()) {
                    None => eprintln!("SEG {label}: NO ROUTE"),
                    Some(r) => {
                        let kinds: Vec<String> = r
                            .links
                            .iter()
                            .map(|&li| {
                                let l = graph.links[li as usize];
                                format!("{:?}:{}->{}", l.kind, l.from, l.to)
                            })
                            .filter(|s| !s.starts_with("Walk") && !s.starts_with("Step"))
                            .collect();
                        eprintln!(
                            "SEG {label}: cost={:.3} legs={} end_band={} special=[{}]",
                            r.cost,
                            r.links.len(),
                            r.end_band,
                            kinds.join(", ")
                        );
                        for (&li, &entry_band) in r.links.iter().zip(&r.bands) {
                            let l = graph.links[li as usize];
                            if !matches!(l.kind, LinkKind::Walk | LinkKind::Step) {
                                let tr = graph.speed_jump_of_link(li);
                                let exit = graph.banded_step(li, entry_band).map(|(_, b)| b);
                                eprintln!(
                                    "  SEGLEG {label}: {:?} {}->{} entry_band={} exit_band={:?} tr={}",
                                    l.kind,
                                    l.from,
                                    l.to,
                                    entry_band,
                                    exit,
                                    tr.map(|t| format!(
                                        "v_req={:.1} chained={} curl={:.1} gt={} land_lo={:.1}",
                                        t.v_req,
                                        t.chained,
                                        t.curl_gain,
                                        t.ground_turn.is_some(),
                                        t.landing_speed_lo,
                                    ))
                                    .unwrap_or_else(|| "none".into()),
                                );
                            }
                        }
                    }
                }
            };
            // The chain pieces, seeded with realistic entry speeds.
            seg("spawn->1364(sj-land)", (192.0, -224.0, -176.0), (256.0, -864.0, 32.0), 0.0);
            seg("spawn->1313(lower-gt-source)", (192.0, -224.0, -176.0), (224.0, -832.0, 24.0), 0.0);
            seg("860->1313 @320", (-160.0, -672.0, -16.0), (224.0, -832.0, 24.0), 320.0);
            seg("1364->stairfoot-1588 @493", (256.0, -864.0, 32.0), (384.0, -576.0, 56.0), 493.0);
            seg("1364->1682 @479", (256.0, -864.0, 32.0), (448.0, -800.0, 56.0), 479.0);
            seg("1682->1474 @479", (448.0, -800.0, 56.0), (320.0, -576.0, 72.0), 479.0);
            seg("1364->1474 @479", (256.0, -864.0, 32.0), (320.0, -576.0, 72.0), 479.0);
            seg("1474->1123 @493", (320.0, -576.0, 72.0), (64.0, -832.0, 168.0), 493.0);
            seg("1588->rastair-1123 @500", (384.0, -576.0, 56.0), (64.0, -832.0, 168.0), 500.0);
            seg("1123->ra @300", (64.0, -832.0, 168.0), (256.0, -704.0, 328.0), 300.0);
            seg("860->1364 @320", (-160.0, -672.0, -16.0), (256.0, -864.0, 32.0), 320.0);
            seg("spawn->1588 @0", (192.0, -224.0, -176.0), (384.0, -576.0, 56.0), 0.0);
            seg("spawn->1123 @0", (192.0, -224.0, -176.0), (64.0, -832.0, 168.0), 0.0);
            // spawn cell is 16u off-grid; the full-route query below covers it.

            // Regression contract: the certified chained ground-turn capability
            // must (a) emit at least one contract-carrying link into the RA
            // stair set, (b) make the SJ-landing -> stair-foot segment banded-
            // traversable fast at carried speed, (c) keep every emitted
            // contract internally sane (envelope, box, landing carry).
            let mut gt_links = 0usize;
            for (li, link) in graph.links.iter().enumerate() {
                let Some(tr) = graph.speed_jump_of_link(li as u32) else { continue };
                let Some(gt) = tr.ground_turn else { continue };
                gt_links += 1;
                assert!(gt.entry_speed_lo < gt.entry_speed_hi, "link {li} envelope");
                // v3 (optimal-sweep) contracts are certified from the carried
                // low-entry band (ladder 320/340/360, tol 2% => floor >= 313);
                // the >=420 floor is a v1/v2 (bearing-follow) plausibility bound.
                if gt.version == rtx_nav::navmesh::GROUND_TURN_OPTIMAL_VERSION {
                    assert!(gt.entry_speed_lo >= 313.0, "link {li} implausibly slow optimal entry floor");
                } else if tr.chained {
                    assert!(gt.entry_speed_lo >= 420.0, "link {li} implausibly slow chained entry floor");
                } else {
                    assert!(gt.entry_speed_lo >= 300.0, "link {li} implausibly slow runway entry floor");
                }
                assert!(gt.landing_speed_lo > 0.0, "link {li} landing carry not stamped");
                assert!(gt.box_min.x < gt.box_max.x && gt.box_min.y < gt.box_max.y, "link {li} box");
                if gt.version == rtx_nav::navmesh::GROUND_TURN_OPTIMAL_VERSION {
                    assert!(!gt.blended_runway, "link {li} v3 contract must not blend runway");
                } else {
                    assert_eq!(gt.version, if gt.blended_runway { 2 } else { 1 }, "link {li} contract version");
                }
                let _ = link;
            }
            assert!(gt_links > 0, "no chained ground-turn links generated at all");
            let upper_covered = upper_targets.iter().any(|&t| {
                graph.links.iter().enumerate().any(|(li, l)| {
                    l.to == t
                        && graph
                            .speed_jump_of_link(li as u32)
                            .is_some_and(|tr| tr.ground_turn.is_some())
                })
            });
            assert!(upper_covered, "no ground-turn link lands on the RA stair set");
            let sjland = exact_cell(graph, Vec3::new(256.0, -864.0, 32.0));
            let stairfoot = exact_cell(graph, Vec3::new(384.0, -576.0, 56.0));
            let r = graph
                .find_path_banded(sjland, stairfoot, 493.0, &LinkCosts::default())
                .expect("banded SJ-landing -> stair-foot");
            assert!(r.cost < 1.2, "stair-foot segment regressed: {:.3}", r.cost);
        }

        // Banded route: RA-tunnel spawn -> RA plateau cell at the item.
        let spawn = graph
            .cells
            .iter()
            .enumerate()
            .map(|(id, cell)| (id as CellId, (cell.origin - Vec3::new(192.0, -208.0, -176.0)).length()))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("spawn cell");
        let ra = exact_cell(graph, Vec3::new(256.0, -704.0, 328.0));
        eprintln!("spawn cell {} (miss {:.1}), ra cell {}", spawn.0, spawn.1, ra);
        let costs = LinkCosts::default();
        match graph.find_path_banded(spawn.0, ra, 0.0, &costs) {
            None => eprintln!("curl={curl}: NO banded route spawn->ra"),
            Some(route) => {
                eprintln!("curl={curl}: banded route cost={:.3}s legs={}", route.cost, route.links.len());
                for &li in &route.links {
                    let link = graph.links[li as usize];
                    let from = graph.cells[link.from as usize].origin;
                    let to = graph.cells[link.to as usize].origin;
                    eprintln!(
                        "  {:?} {}->{} ({:.0},{:.0},{:.0})->({:.0},{:.0},{:.0}) cost={:.3}",
                        link.kind, link.from, link.to, from.x, from.y, from.z, to.x, to.y, to.z, link.cost
                    );
                }
                for (&li, &entry_band) in route.links.iter().zip(&route.bands) {
                    let link = graph.links[li as usize];
                    if !matches!(link.kind, LinkKind::Walk | LinkKind::Step) {
                        let tr = graph.speed_jump_of_link(li);
                        let exit = graph.banded_step(li, entry_band).map(|(_, b)| b);
                        eprintln!(
                            "  ROUTE_SPECIAL {:?} {}->{} entry_band={} exit_band={:?} tr={}",
                            link.kind,
                            link.from,
                            link.to,
                            entry_band,
                            exit,
                            tr.map(|t| format!(
                                "v_req={:.1} chained={} curl={:.1} gt={} land_lo={:.1}",
                                t.v_req,
                                t.chained,
                                t.curl_gain,
                                t.ground_turn.is_some(),
                                t.landing_speed_lo,
                            ))
                            .unwrap_or_else(|| "none".into()),
                        );
                    }
                }
                if curl {
                    assert!(route.cost < 9.565, "full route misses owner target: {:.3}", route.cost);
                }
            }
        }
    }
}
