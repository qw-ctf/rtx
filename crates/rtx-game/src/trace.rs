// SPDX-License-Identifier: AGPL-3.0-or-later

//! World tracing shared by both embodiments — a port of mvdsv's `SV_Trace` / `SV_ClipToLinks` /
//! `SV_ClipMoveToEntity` / `SV_HullForEntity` (sv_world.c), answering `traceline` from our own
//! parsed [`Bsp`](rtx_nav::bsp::Bsp) and entity array rather than an engine syscall. The point-hull
//! numeric core lives in [`rtx_nav::bsp`] (`CM_HullTrace`); this layer is the entity clip loop on
//! top of it: world first, then every solid entity whose box the move crosses, keeping the nearest
//! impact. One implementation, so the server (qwprogs) and the netclient answer identically.
//!
//! Scope: a **traceline** is a point move (`mins == maxs == 0`). The parity-relevant `SV_ClipToLinks`
//! rules are all here — `nomonsters` still clips brush-model doors/plats; triggers and `Solid::Not`
//! never interact; a sized `passedict` never hits a zero-size touch (points-never-interact); the
//! shooter and its own missiles / its owner are skipped (both directions, including the `owner == 0
//! → world` quirk); `startsolid` OR-merges across candidates and the nearest impact wins whole. The
//! `MOVE_LAGGED` (`sv_antilag 2`) path is deliberately not ported (mvdsv default is 0).

use glam::Vec3;

use crate::defs::Solid;
use crate::entity::EntId;
use crate::game::{GameState, TraceResult};

/// A cleared (nothing-hit) trace out to `end`, attributed to `ent`.
fn clear_trace(end: Vec3, ent: EntId) -> TraceResult {
    TraceResult {
        allsolid: false,
        startsolid: false,
        fraction: 1.0,
        endpos: end,
        plane_normal: Vec3::ZERO,
        plane_dist: 0.0,
        ent,
        in_open: false,
        in_water: false,
    }
}

/// The answer when there's no map: everything is solid (fail closed). A clear line would have the
/// caller believe it can see through an unloaded world; `droptofloor` would think every item is
/// floating and refuse to settle it. Only reachable in the netclient's transient pre-map window.
fn no_map(start: Vec3) -> TraceResult {
    TraceResult { allsolid: true, startsolid: true, fraction: 0.0, endpos: start, ..clear_trace(start, EntId::WORLD) }
}

/// A [`rtx_nav::bsp::HullTrace`] plus an owning entity, in `TraceResult` shape.
fn from_hull(ht: rtx_nav::bsp::HullTrace, ent: EntId) -> TraceResult {
    TraceResult {
        allsolid: ht.all_solid,
        startsolid: ht.start_solid,
        fraction: ht.fraction,
        endpos: ht.endpos,
        plane_normal: ht.plane_normal,
        plane_dist: ht.plane_dist,
        ent,
        in_open: ht.in_open,
        in_water: ht.in_water,
    }
}

/// The inline-submodel index of an entity's model string `"*N"`, or `None` for a non-submodel model
/// (a `.mdl`/`.bsp` path, or the worldmodel). The model **string** — not `modelindex`, which differs
/// between the server and the netclient (see `netclient::mirror`).
fn submodel_index(model: Option<&str>) -> Option<usize> {
    model?.strip_prefix('*')?.parse().ok()
}

/// The move box for a point trace: `SV_MoveBounds` with a zero move-box, i.e. the segment's own AABB
/// grown by a unit each way.
fn move_bounds(start: Vec3, end: Vec3) -> (Vec3, Vec3) {
    (start.min(end) - Vec3::ONE, start.max(end) + Vec3::ONE)
}

impl GameState {
    /// `SV_Trace` for a **point** move (`traceline`): clip `start → end` to the world hull, then to
    /// every solid entity whose box the move crosses, and return the nearest impact. `nomonsters`
    /// skips point/box entities but still clips brush-model doors and plats; `passedict` (the shooter)
    /// is never clipped. Pure over `self.nav.bsp` + `self.entities` — no engine call, either host.
    pub(crate) fn sv_trace(&self, start: Vec3, end: Vec3, nomonsters: bool, passedict: EntId) -> TraceResult {
        // 1. Clip to the world hull (hull 0 — the point hull QuakeC's traceline uses). With no map
        // bound, fail *closed* (all-solid at the start): a clear line would have callers believe they
        // can see through a world that isn't loaded, and `droptofloor` believe every item is floating.
        // The server always has a map here (parsed at load), so this only guards the netclient's
        // transient pre-map state — and there, refusing every trace is the safe answer.
        let Some(bsp) = self.nav.bsp.as_deref() else {
            return no_map(start);
        };
        let mut trace = from_hull(bsp.hull0_trace(start, end), EntId::WORLD);

        // 2. Bounds of the whole move, and the shooter's size for points-never-interact.
        let (boxmins, boxmaxs) = move_bounds(start, end);
        let pass_size = self.trace_size(passedict);

        // 3. Clip to entities, keeping the nearest impact (SV_ClipToLinks).
        for i in 0..self.entities.len() as u32 {
            if trace.allsolid {
                break; // wholly inside solid — nothing nearer can be found
            }
            let touch = EntId(i);
            if touch == passedict || touch == EntId::WORLD {
                continue; // never the shooter; the world was clipped in step 1
            }
            let te = &self.entities[touch];
            if !te.in_use {
                continue;
            }
            // Triggers and non-solids never block a trace (they touch, they don't clip).
            if matches!(te.v.solid, Solid::Not | Solid::Trigger) {
                continue;
            }
            // nomonsters clips brush models (doors/plats) only — point/box entities are skipped.
            if nomonsters && te.v.solid != Solid::Bsp {
                continue;
            }
            // Move-box vs entity-box reject (a loose superset of SV_LinkEdict's absmin/absmax, so it
            // can only over-include — the exact clip below is authoritative — never wrongly skip).
            let amin = te.v.origin + te.v.mins - Vec3::ONE;
            let amax = te.v.origin + te.v.maxs + Vec3::ONE;
            if amin.cmpgt(boxmaxs).any() || amax.cmplt(boxmins).any() {
                continue;
            }
            // Points never interact: a sized shooter never hits a zero-size touch.
            if pass_size != 0.0 && self.trace_size(touch) == 0.0 {
                continue;
            }
            // Don't clip against the shooter's own missiles, nor against the shooter's owner. Both
            // read a raw-0 owner as the world (`from_prog(0) == WORLD`), so an `ignore == WORLD`
            // trace skips every ownerless solid — the engine-authentic quirk.
            if te.owner() == passedict || self.entities[passedict].owner() == touch {
                continue;
            }
            let et = self.clip_move_to_entity(touch, start, end);
            // Keep a startsolid from any candidate; the nearest impact wins whole (SV_ClipToLinks).
            trace.startsolid |= et.startsolid;
            if et.allsolid || et.fraction < trace.fraction {
                let mut win = et;
                win.ent = touch;
                win.startsolid |= trace.startsolid;
                trace = win;
            }
        }
        trace
    }

    /// `SV_ClipMoveToEntity` + `SV_HullForEntity` for a point move: pick `ent`'s clip hull (a brush
    /// submodel's hull 0 for `Solid::Bsp`, else a box hull from its bounds), trace the origin-relative
    /// segment through it, and shift the result back by the entity's origin. No rotation (QW never
    /// implemented it).
    fn clip_move_to_entity(&self, ent: EntId, start: Vec3, end: Vec3) -> TraceResult {
        let e = &self.entities[ent];
        let origin = e.v.origin;
        let (s, t) = (start - origin, end - origin);
        let ht = if e.v.solid == Solid::Bsp {
            // A door/plat/trigger brush shaped like "*N", clipped through the shared hull-0 tree. A
            // non-"*N" Solid::Bsp (only the world, already excluded) traces as clear, never blocking.
            match (submodel_index(e.model.as_deref()), self.nav.bsp.as_deref()) {
                (Some(n), Some(bsp)) => bsp.submodel_hull0_trace(n, s, t),
                _ => return clear_trace(end, EntId::WORLD),
            }
        } else {
            // A sized point entity (player, item): a box hull from its own bounds.
            rtx_nav::bsp::box_hull(e.v.mins, e.v.maxs).trace(s, t)
        };
        let mut tr = from_hull(ht, EntId::WORLD);
        tr.endpos += origin; // fix the endpoint back up by the offset
        tr
    }

    /// An entity's trace size on the x axis (`maxs.x - mins.x`), the points-never-interact key.
    /// Derived from the bounds rather than the `.size` field, which the netclient mirror doesn't
    /// maintain; a zero here marks a point entity (a projectile, a gib).
    fn trace_size(&self, e: EntId) -> f32 {
        let v = &self.entities[e].v;
        v.maxs.x - v.mins.x
    }

    /// Mirror a finished trace into the engine-shared `trace_*` globals. Required even though
    /// `sv_trace` returns the result directly, because a touch callback (`wall_velocity`, blood
    /// spray) reads the **stale** `trace_plane_normal` from whatever trace ran last — mvdsv's
    /// `SV_Impact` doesn't refresh the globals, and the quirk is engine-authentic.
    pub(crate) fn write_trace_globals(&mut self, tr: &TraceResult) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        let g = &mut self.globals;
        g.trace_allsolid = b(tr.allsolid);
        g.trace_startsolid = b(tr.startsolid);
        g.trace_fraction = tr.fraction;
        g.trace_endpos = tr.endpos;
        g.trace_plane_normal = tr.plane_normal;
        g.trace_plane_dist = tr.plane_dist;
        g.trace_ent = tr.ent.to_prog();
        g.trace_inopen = b(tr.in_open);
        g.trace_inwater = b(tr.in_water);
    }
}

// `sv_trace` reads only `nav.bsp` + `entities` + `globals`, so it is embodiment-agnostic; the tests
// build a GameState through the (simplest) netclient fixture. The logic under test is the same one
// the server runs.
#[cfg(all(test, feature = "netclient"))]
mod tests {
    use std::path::PathBuf;
    use std::sync::Arc;

    use rtx_nav::bsp::{Bsp, ClipNode, Model, Plane, CONTENTS_EMPTY, CONTENTS_SOLID};

    use super::*;
    use crate::game::GameState;
    use crate::netclient::host::NetHost;

    fn game() -> GameState {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(PathBuf::from("/nonexistent"))));
        host.set("maxclients", "8");
        GameState::new_client(host)
    }

    /// A solid box entity (a mirrored player) at `at`, in slot `slot`.
    fn box_ent(g: &mut GameState, slot: u32, at: Vec3) -> EntId {
        let e = EntId(slot);
        let ent = &mut g.entities[e];
        ent.in_use = true;
        ent.v.solid = Solid::SlideBox;
        ent.v.origin = at;
        ent.v.mins = Vec3::new(-16.0, -16.0, -24.0);
        ent.v.maxs = Vec3::new(16.0, 16.0, 32.0);
        e
    }

    /// A stand-in for "the entity doing the trace" — a real (non-world) passedict, parked far off the
    /// move so it never clips. A trace must **not** ignore `world`: that triggers the `owner == 0 →
    /// world` skip and drops every ownerless solid from the clip (an engine-authentic quirk, tested
    /// separately in [`owner_both_ways_and_the_world_quirk`]).
    fn shooter(g: &mut GameState) -> EntId {
        box_ent(g, 20, Vec3::new(0.0, 0.0, -1000.0))
    }

    /// An empty world (a synthetic BSP whose world hull is one open leaf) so only entities clip.
    fn open_world(g: &mut GameState) {
        let bsp = Bsp::synthetic(Vec::new(), Vec::new(), CONTENTS_EMPTY, vec![world_model()]);
        g.nav.bsp = Some(Arc::new(bsp));
    }

    fn world_model() -> Model {
        Model { mins: Vec3::splat(-4096.0), maxs: Vec3::splat(4096.0), render_head: CONTENTS_EMPTY, clip1: CONTENTS_EMPTY }
    }

    const HORIZ: Vec3 = Vec3::new(200.0, 0.0, 0.0);

    #[test]
    fn clips_a_box_entity_and_names_it() {
        let mut g = game();
        open_world(&mut g);
        let ig = shooter(&mut g);
        let p = box_ent(&mut g, 2, Vec3::new(100.0, 0.0, 0.0));
        // Straight through a player box centred at x=100: enters its −x face at x=84.
        let tr = g.sv_trace(Vec3::ZERO, HORIZ, false, ig);
        assert!((tr.fraction - 84.0 / 200.0).abs() < 0.02, "fraction {}", tr.fraction);
        assert_eq!(tr.ent, p, "the box entity is named as the thing hit");

        // Ignoring that entity (it's the shooter) → clear.
        let clear = g.sv_trace(Vec3::ZERO, HORIZ, false, p);
        assert_eq!(clear.fraction, 1.0, "the passedict is never clipped");
        assert_eq!(clear.ent, EntId::WORLD);
    }

    #[test]
    fn skips_triggers_nonsolids_and_freed() {
        let mut g = game();
        open_world(&mut g);
        let ig = shooter(&mut g);
        let e = box_ent(&mut g, 2, Vec3::new(100.0, 0.0, 0.0));
        for solid in [Solid::Trigger, Solid::Not] {
            g.entities[e].v.solid = solid;
            assert_eq!(g.sv_trace(Vec3::ZERO, HORIZ, false, ig).fraction, 1.0, "{solid:?} must not clip");
        }
        // A freed (not in_use) solid box is likewise invisible to the trace.
        g.entities[e].v.solid = Solid::SlideBox;
        g.entities[e].in_use = false;
        assert_eq!(g.sv_trace(Vec3::ZERO, HORIZ, false, ig).fraction, 1.0, "a freed entity must not clip");
    }

    #[test]
    fn nomonsters_skips_boxes_but_clips_a_bsp_door() {
        let mut g = game();
        // World hull 0 is one node: a half-space wall solid at local x ≥ 0 (front), empty behind.
        let planes = vec![Plane { normal: Vec3::new(1.0, 0.0, 0.0), dist: 0.0, kind: 0 }];
        let nodes = vec![ClipNode { plane: 0, children: [CONTENTS_SOLID, CONTENTS_EMPTY] }];
        // models[0] world = open air; models[1] the door submodel = that wall, rooted at node 0.
        let door_model = Model { mins: Vec3::splat(-64.0), maxs: Vec3::splat(64.0), render_head: 0, clip1: 0 };
        let bsp = Bsp::synthetic(planes, nodes, CONTENTS_EMPTY, vec![world_model(), door_model]);
        g.nav.bsp = Some(Arc::new(bsp));

        let ig = shooter(&mut g);
        // A box player and a Bsp door "*1" both sit in the path (door plane at world x = 50).
        box_ent(&mut g, 2, Vec3::new(30.0, 0.0, 0.0));
        let door = EntId(3);
        g.entities[door].in_use = true;
        g.entities[door].v.solid = Solid::Bsp;
        g.entities[door].v.origin = Vec3::new(50.0, 0.0, 0.0);
        g.entities[door].v.mins = Vec3::splat(-64.0); // a brush door is sized (setmodel copies the
        g.entities[door].v.maxs = Vec3::splat(64.0); // submodel bounds), so it's not a "point"
        g.entities[door].model = Some("*1".into());

        // nomonsters skips the box player but still clips the door — at world x ≈ 50.
        let tr = g.sv_trace(Vec3::ZERO, HORIZ, true, ig);
        assert_eq!(tr.ent, door, "nomonsters keeps clipping a brush door");
        assert!((tr.endpos.x - 50.0).abs() < 0.5, "door impact offset by its origin: {:?}", tr.endpos);

        // Without nomonsters the nearer box player (front face at x = 14) wins instead.
        let tr = g.sv_trace(Vec3::ZERO, HORIZ, false, ig);
        assert_eq!(tr.ent, EntId(2), "the nearer box wins a normal trace");
    }

    #[test]
    fn points_never_interact() {
        let mut g = game();
        open_world(&mut g);
        // A sized shooter and a zero-size projectile in the path.
        let shooter = box_ent(&mut g, 1, Vec3::new(-50.0, 0.0, 0.0));
        let rocket = EntId(2);
        g.entities[rocket].in_use = true;
        g.entities[rocket].v.solid = Solid::BBox;
        g.entities[rocket].v.origin = Vec3::new(100.0, 0.0, 0.0);
        g.entities[rocket].v.mins = Vec3::ZERO;
        g.entities[rocket].v.maxs = Vec3::ZERO; // zero size → a point
        // The sized shooter's trace passes straight through the point projectile.
        let tr = g.sv_trace(Vec3::ZERO, HORIZ, false, shooter);
        assert_eq!(tr.fraction, 1.0, "a sized shooter never hits a zero-size touch");
    }

    #[test]
    fn owner_both_ways_and_the_world_quirk() {
        let mut g = game();
        open_world(&mut g);
        let shooter = box_ent(&mut g, 1, Vec3::new(-50.0, 0.0, 0.0));
        let missile = box_ent(&mut g, 2, Vec3::new(100.0, 0.0, 0.0));

        // The shooter's own missile doesn't clip it.
        g.entities[missile].set_owner(shooter);
        assert_eq!(g.sv_trace(Vec3::ZERO, HORIZ, false, shooter).fraction, 1.0, "own missile skipped");

        // The other direction: a trace by the missile doesn't clip its owner (the shooter) either —
        // put the shooter in the path to prove it.
        g.entities[shooter].v.origin = Vec3::new(100.0, 0.0, 0.0);
        g.entities[missile].v.origin = Vec3::new(-50.0, 0.0, 0.0);
        assert_eq!(g.sv_trace(Vec3::ZERO, HORIZ, false, missile).fraction, 1.0, "owner skipped");

        // The owner==0 → world quirk: an ownerless solid, traced with ignore=WORLD, is skipped.
        g.entities[shooter].set_owner(EntId::WORLD); // raw 0
        assert_eq!(g.sv_trace(Vec3::ZERO, HORIZ, false, EntId::WORLD).fraction, 1.0, "ownerless + ignore=world skipped");
    }

    #[test]
    fn startsolid_merges_and_nearest_wins() {
        let mut g = game();
        open_world(&mut g);
        let ig = shooter(&mut g);
        // A box the trace *starts* inside (start-in-solid → startsolid, but it exits so not allsolid),
        // and a second, nearer box further along that the trace actually stops at.
        box_ent(&mut g, 1, Vec3::ZERO); // start point (0,0,0) is inside this box
        let near = box_ent(&mut g, 2, Vec3::new(60.0, 0.0, 0.0));
        let tr = g.sv_trace(Vec3::ZERO, HORIZ, false, ig);
        assert!(tr.startsolid, "starting inside a box flags startsolid, kept across candidates");
        assert_eq!(tr.ent, near, "the nearest impact still wins the whole trace");
    }
}
