// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh mutation commands used by the external control harness.

use glam::Vec3;

use crate::game::GameState;

use super::{jnum, jvec3};

/// Hand-plant a rocket-jump link — see [`super::ControlCmd::PlanRj`]. Snapshots the live gravity and the
/// `rj` self-boost cvar exactly like the build does (`nav_build.rs`), so the offline blast solve
/// matches the knockback the runtime flight will get. The solve itself (targeted, cap-free but
/// fully certified) lives in [`NavGraph::plant_rocket_jump`](crate::navmesh); the reply reads the
/// inserted link back through the same accessors `links_json` uses.
pub(super) fn plant_rj_json(game: &mut GameState, from: Vec3, tgt: Vec3) -> Result<String, String> {
    let params = crate::navmesh::RocketJumpParams {
        gravity: {
            let g = game.host.cvar(c"sv_gravity");
            if g > 0.0 { g } else { 800.0 }
        },
        rj_extra: game.host.cvar(c"rj"),
    };
    let nav = &mut game.nav;
    let bsp = nav.bsp.as_ref().ok_or("no bsp loaded")?;
    let g = nav.graph.as_mut().ok_or("navmesh not ready")?;
    let li = g.plant_rocket_jump(bsp, from, tgt, params)?;
    let (src_cell, tgt_cell) = (g.link_source(li), g.link_target(li));
    let (src, dst) = (g.cell_origin(src_cell), g.cell_origin(tgt_cell));
    let tr = g.rocket_jump_of_link(li).ok_or("planted link lost its traversal")?;
    Ok(format!(
        "{{\"link\":{li},\"from_cell\":{src_cell},\"to_cell\":{tgt_cell},\"src\":{},\"tgt\":{},\
         \"fire_pitch\":{},\"fire_yaw\":{},\"fire_delay\":{},\"airtime\":{},\"self_damage\":{},\
         \"v0\":{},\"blast\":{},\"land\":{}}}",
        jvec3(src),
        jvec3(dst),
        jnum(tr.fire_angles.x),
        jnum(tr.fire_angles.y),
        jnum(tr.fire_delay),
        jnum(tr.airtime),
        jnum(tr.self_damage),
        jvec3(tr.v0),
        jvec3(tr.blast),
        jvec3(tr.land),
    ))
}

/// Hand-plant an uncertified rocket-jump link with explicit fire params — see
/// [`super::ControlCmd::PlanRjRaw`]. `self_damage` is a nominal full-blast estimate (the runtime fitness
/// gate and health cost accounting need an honest number; ~35 is a typical point-blank floor shot).
pub(super) fn plant_rj_raw_json(
    game: &mut GameState,
    from: Vec3,
    tgt: Vec3,
    pitch: f32,
    yaw: f32,
    delay: f32,
    airtime: f32,
) -> Result<String, String> {
    const RAW_SELF_DAMAGE: f32 = 35.0;
    let g = game.nav.graph.as_mut().ok_or("navmesh not ready")?;
    let li = g.plant_rocket_jump_raw(
        from,
        tgt,
        Vec3::new(pitch, yaw, 0.0),
        delay,
        airtime,
        RAW_SELF_DAMAGE,
    )?;
    let (src_cell, tgt_cell) = (g.link_source(li), g.link_target(li));
    let (src, dst) = (g.cell_origin(src_cell), g.cell_origin(tgt_cell));
    Ok(format!(
        "{{\"link\":{li},\"from_cell\":{src_cell},\"to_cell\":{tgt_cell},\"src\":{},\"tgt\":{},\
         \"fire_pitch\":{},\"fire_yaw\":{},\"fire_delay\":{},\"airtime\":{},\"certified\":false}}",
        jvec3(src),
        jvec3(dst),
        jnum(pitch),
        jnum(yaw),
        jnum(delay),
        jnum(airtime),
    ))
}

/// Curated link disable — see [`super::ControlCmd::Unlink`]. Adds a prohibitive surcharge (same magnitude
/// as the closed-gate/unfit penalties, which every search already treats as "never worth it") to
/// the link's stored cost, so all bots route around it until the next navmesh rebuild.
pub(super) fn unlink_json(game: &mut GameState, link: u32) -> Result<String, String> {
    const UNLINK_PENALTY: f32 = 100_000.0;
    let g = game.nav.graph.as_mut().ok_or("navmesh not ready")?;
    let n = g.links.len() as u32;
    if link >= n {
        return Err(format!("link {link} out of range (0..{n})"));
    }
    let old = g.links[link as usize].cost;
    if old >= UNLINK_PENALTY {
        return Err(format!("link {link} is already unlinked (cost {old})"));
    }
    g.links[link as usize].cost = old + UNLINK_PENALTY;
    let (src, dst) = (g.cell_origin(g.link_source(link)), g.cell_origin(g.link_target(link)));
    Ok(format!(
        "{{\"link\":{link},\"oldCost\":{},\"newCost\":{},\"src\":{},\"tgt\":{}}}",
        jnum(old),
        jnum(old + UNLINK_PENALTY),
        jvec3(src),
        jvec3(dst),
    ))
}
