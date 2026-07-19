// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only diagnostic (env-gated on RTX_TEST_BSP): why is the DM3 YA
//! enclosure a graph island? Spawn-7 blocker 2 (route-lab
//! docs/plans/2026-07-19-spawn7-orchestration.md): the ground corridor at
//! z=-24 is severed at the x=1376 column strip, and the 88-walkway above the
//! pocket emits no Drop links into it. This probe answers, from the BSP:
//!
//!  1. What solidity does the x=1376 column actually carry (crate? pillar?
//!     thin-obstacle GRID miss?) — solid bands from z −300..100.
//!  2. Which specific check kills the walkway→pocket Drop candidates:
//!     the descent hull trace, or the head-height path corridor.
//!
//! Changes nothing; prints evidence for the fix decision.

use glam::{Vec3, Vec3Swizzles};
use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::build_navmesh;

#[test]
fn ya_pocket_probe() {
    let Ok(path) = std::env::var("RTX_TEST_BSP") else {
        eprintln!("skip: set RTX_TEST_BSP");
        return;
    };
    let bytes = std::fs::read(&path).expect("read bsp");
    let bsp = Bsp::parse(&bytes).expect("parse bsp");
    let graph = build_navmesh(&bsp, vec![], vec![], vec![], None, false, None, None);

    // 1. Solid bands in the severed strip and its neighbors.
    for x in [1344.0f32, 1376.0, 1408.0] {
        for y in [-928.0f32, -896.0, -864.0, -832.0] {
            let mut bands: Vec<(f32, f32)> = Vec::new();
            let mut cur: Option<f32> = None;
            let mut z = -300.0f32;
            while z <= 100.0 {
                let solid = bsp.is_solid(Vec3::new(x, y, z));
                match (solid, cur) {
                    (true, None) => cur = Some(z),
                    (false, Some(start)) => {
                        bands.push((start, z));
                        cur = None;
                    }
                    _ => {}
                }
                z += 4.0;
            }
            if let Some(start) = cur {
                bands.push((start, z));
            }
            eprintln!("column ({x},{y}): solid bands {bands:?}");
        }
    }

    // 2. The walkway→pocket Drop candidates: replicate the generator's descent + corridor checks.
    let candidates = [
        (Vec3::new(1248.0, -928.0, 88.0), Vec3::new(1216.0, -928.0, -24.0)),
        (Vec3::new(1248.0, -896.0, 88.0), Vec3::new(1216.0, -896.0, -24.0)),
        (Vec3::new(1344.0, -896.0, 88.0), Vec3::new(1344.0, -896.0, -24.0)),
    ];
    for (a, b) in candidates {
        // descent_clear equivalent: hull-1 trace straight down the target column.
        let tr = bsp.hull1_trace(Vec3::new(b.x, b.y, a.z), b);
        let descent_ok = !tr.start_solid && tr.fraction > 0.99;
        // path_clear equivalent: head-height corridor at the higher origin.
        let z = a.z.max(b.z);
        let steps = ((b.xy() - a.xy()).length() / 16.0).ceil().max(1.0) as i32;
        let path_ok = (0..=steps).all(|i| {
            let t = i as f32 / steps as f32;
            let p = a.lerp(b, t);
            !bsp.is_solid(Vec3::new(p.x, p.y, z))
        });
        eprintln!(
            "drop {:?} -> {:?}: descent_clear={descent_ok} (start_solid={}, fraction={:.3}) path_clear={path_ok}",
            a, b, tr.start_solid, tr.fraction
        );
    }

    // 3. Sanity: confirm the graph really has no cell in the strip at ground level.
    let strip: Vec<_> = graph
        .cells
        .iter()
        .filter(|c| c.origin.x == 1376.0 && (-960.0..=-820.0).contains(&c.origin.y) && c.origin.z < 50.0)
        .map(|c| c.origin)
        .collect();
    eprintln!("ground-level cells in x=1376 strip: {strip:?}");
}
