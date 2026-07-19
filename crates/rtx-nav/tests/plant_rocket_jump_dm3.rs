// SPDX-License-Identifier: AGPL-3.0-or-later

//! Env-gated (RTX_TEST_BSP) NEGATIVE-RESULT pin: a *static* single stock
//! rocket jump pentlift→window does NOT exist on DM3 — the spawn-7 drill
//! (docs/plans/2026-07-19-spawn7-orchestration.md in route-lab) must use the
//! rising pent lift (lift-jump + RJ), flown via `planrjraw` + live trial.
//!
//! Evidence (2026-07-19, full-circle yaw sweep, boosts up to `rj 2.0`): 264
//! clean simulations across both lift bases and the lift top; best landing
//! miss 384u. Two independent blockers: (1) rise from the pent floor is
//! ~376u where a plain RJ tops out near ~300u even boosted, and (2) from the
//! lift-rest cell (z −104) every wall the rocket can find pushes the bot
//! +x along y≈850 — no surface yields the −y component the window (y 640)
//! needs. If this test ever *finds* an arc, geometry or physics changed:
//! flip the drill back to a certified `planrj` and celebrate.

use glam::{Vec3, Vec3Swizzles};
use rtx_nav::navmesh::{build_navmesh, LinkKind, RocketJumpParams};

/// Pent lift base and window ledge, from the corpus-derived spawn-7 targets
/// (Claudette's blocker note): lift bases (608,880,≈−290) / (507,848,≈−290),
/// window ledge (1152,640,86).
const LIFT_BASE_A: Vec3 = Vec3::new(608.0, 880.0, -290.0);
const LIFT_BASE_B: Vec3 = Vec3::new(507.0, 848.0, -290.0);
/// The lifts landing above the pent lift (the `lifts` spawn ledge).
const LIFT_TOP: Vec3 = Vec3::new(512.0, 768.0, 216.0);
const WINDOW: Vec3 = Vec3::new(1152.0, 640.0, 86.0);

#[test]
fn dm3_pentlift_window_rj_plants() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(&path).expect("read bsp");

    // Mirror the live green config: no hooks, no double jump, no speed-jump
    // params needed for this drill, and — decisively — rocket_jump: None.
    let build = build_navmesh(bytes, vec![], vec![], vec![], None, false, None, None)
        .expect("build navmesh");
    let (bsp, mut graph) = (build.0, build.1);
    let rj_before = (0..graph.links.len() as u32)
        .filter(|&li| graph.link_kind(li) == LinkKind::RocketJump)
        .count();
    assert_eq!(rj_before, 0, "rjump-off build should carry zero generated RJ links");

    // Survey first (route-lab search-campaign rule): where do the anchor points actually resolve
    // in the carved graph, and what standable floor exists around the window ledge?
    for (n, p) in [
        ("lift-a", LIFT_BASE_A),
        ("lift-b", LIFT_BASE_B),
        ("lift-top", LIFT_TOP),
        ("window", WINDOW),
    ] {
        match graph.nearest(p) {
            Some(c) => {
                let o = graph.cell_origin(c);
                eprintln!("survey {n}: {p:?} -> cell {c} at {o:?} (miss {:.0}u)", (o - p).length());
            }
            None => eprintln!("survey {n}: {p:?} -> NO CELL"),
        }
    }
    for dz in [-64.0f32, -32.0, 0.0, 32.0, 64.0] {
        for (dx, dy) in [(-64.0f32, 0.0), (0.0, 0.0), (64.0, 0.0), (0.0, -64.0), (0.0, 64.0)] {
            let p = WINDOW + Vec3::new(dx, dy, dz);
            if let Some(c) = graph.nearest(p) {
                let o = graph.cell_origin(c);
                if (o - p).length() < 40.0 {
                    eprintln!("window-floor: cell {c} at {o:?}");
                }
            }
        }
    }
    // Coarse floor survey of the pent room / lift shaft region: which z-bands carry standable
    // cells? Answers where an RJ can actually launch from.
    let mut bands: std::collections::BTreeMap<i32, Vec<(f32, f32)>> = Default::default();
    for cell in &graph.cells {
        let o = cell.origin;
        if (500.0..=1250.0).contains(&o.x) && (550.0..=950.0).contains(&o.y) {
            bands.entry((o.z / 64.0).floor() as i32).or_default().push((o.x, o.y));
        }
    }
    for (band, cells) in &bands {
        let (xs, ys): (Vec<f32>, Vec<f32>) = cells.iter().copied().unzip();
        let minmax = |v: &[f32]| (v.iter().copied().fold(f32::MAX, f32::min), v.iter().copied().fold(f32::MIN, f32::max));
        let ((x0, x1), (y0, y1)) = (minmax(&xs), minmax(&ys));
        eprintln!(
            "floor band z≈{}..{}: {} cells, x {x0:.0}..{x1:.0}, y {y0:.0}..{y1:.0}",
            band * 64,
            (band + 1) * 64,
            cells.len()
        );
    }

    let mut planted = Vec::new();
    for (name, base, rj_extra) in [
        ("lift-a", LIFT_BASE_A, 0.0f32),
        ("lift-b", LIFT_BASE_B, 0.0),
        ("lift-top", LIFT_TOP, 0.0),
        ("lift-a rj1.5", LIFT_BASE_A, 1.5),
        ("lift-a rj2.0", LIFT_BASE_A, 2.0),
    ] {
        let params = RocketJumpParams { gravity: 800.0, rj_extra };
        match graph.plant_rocket_jump(&bsp, base, WINDOW, params) {
            Ok(li) => {
                assert_eq!(graph.link_kind(li), LinkKind::RocketJump);
                let tr = graph.rocket_jump_of_link(li).expect("traversal tagged");
                let src = graph.cell_origin(graph.link_source(li));
                let dst = graph.cell_origin(graph.link_target(li));
                eprintln!(
                    "{name}: link {li} {src:?} -> {dst:?} pitch {} yaw {} delay {} airtime {:.2} selfdmg {:.0} land {:?}",
                    tr.fire_angles.x, tr.fire_angles.y, tr.fire_delay, tr.airtime, tr.self_damage, tr.land,
                );
                planted.push((name, li, dst));
            }
            Err(e) => eprintln!("{name}: no arc — {e}"),
        }
    }
    // The negative-result pin (see module docs): no static launch point certifies. If an arc ever
    // appears, the drill should switch from `planrjraw` live-trial back to this certified plant.
    assert!(
        planted.is_empty(),
        "a static pentlift→window arc now certifies ({planted:?}) — geometry/physics changed; \
         move the spawn-7 drill to the certified planrj link"
    );

    // The live-trial primitive must be plantable on the rjump-off graph: lift-rest cell → window
    // ledge with nominal lift-jump params (tuned live by the drill loop).
    let li = graph
        .plant_rocket_jump_raw(LIFT_BASE_A, WINDOW, Vec3::new(65.0, 158.0, 0.0), 0.30, 1.5, 35.0)
        .expect("raw plant on rjump-off graph");
    assert_eq!(graph.link_kind(li), LinkKind::RocketJump);
    let tr = graph.rocket_jump_of_link(li).expect("raw traversal tagged");
    assert_eq!(tr.fire_delay, 0.30);
    let dst = graph.cell_origin(graph.link_target(li));
    assert!(
        (dst.xy() - WINDOW.xy()).length() <= 96.0 && (dst.z - WINDOW.z).abs() <= 64.0,
        "raw link landing cell {dst:?} is not the window ledge"
    );
}
