// SPDX-License-Identifier: AGPL-3.0-or-later

//! The engine's world traps, done ourselves: linking, `setmodel`, and `traceline`.
//!
//! These are the traps that write our own entity array. A [`ClientHost`](crate::host::ClientHost)
//! can't perform them — it would alias the `&mut GameState` the game is holding while it calls —
//! so they live here as `GameState` methods and the [`host`](crate::host) module's dispatch routes
//! to them. The point is that the game and bot code above never learns which host it's under.
//!
//! Ported from Quake's `sv_move.c` / `world.c` / `pr_cmds.c`, and faithful where fidelity is
//! load-bearing: the shadow world's plats and doors compute their travel from the bounds these set,
//! so a lazily-approximated box here becomes a navmesh link to the wrong floor.

use glam::Vec3;

use crate::defs::{Bits, Flags};
use crate::entity::{EntId, Think};
use crate::game::{GameState, TraceResult};
use crate::game_command::GameCommand;

/// How far `droptofloor` looks for a floor before giving up (`SV_DropToFloor`). A mapper's item
/// left floating further than this than is a map bug, and reporting it is how they find out.
const DROP_DISTANCE: f32 = 256.0;

/// The half-extents an item's box is grown by when linked, so it can be picked up from a step away
/// or off a shelf (`SV_LinkEdict`). Horizontal only — id was specific about that.
const ITEM_GRAB: f32 = 15.0;

/// Everything else grows by a unit, because movement is clipped an epsilon short of a real edge and
/// touching boxes must still register as touching.
const LINK_EPSILON: f32 = 1.0;

impl GameState {
    /// Spawn the map's own entities — the **shadow world**.
    ///
    /// A server hands its game module a world already populated: items on their pedestals, doors on
    /// their hinges, teleporters wired to their destinations. A client is handed none of that. It
    /// gets told where things *are*, once they move, and nothing about what they are or where they
    /// belong.
    ///
    /// So we spawn them ourselves, from the map's own entity string, by running the module's real
    /// spawn code — the same `load_entities` the server drives. That's the whole trick: the navmesh
    /// builder, the item catalogue, the plat and door and teleporter collectors all keep working
    /// unmodified, because the world they read was built by the code they were written against.
    /// The network then overlays live truth on top: the shadow says what exists and where it
    /// belongs, the wire says where it is now.
    ///
    /// Only the *spawn* runs. Nothing thinks, nothing touches, nothing moves — those are the
    /// server's job, and doing them here would be a second simulation quietly disagreeing with the
    /// real one.
    pub(crate) fn spawn_shadow_world(&mut self) {
        self.dispatch(GameCommand::LoadEntities, 0, 0, 0);
        self.settle_items();
    }

    /// Drop every item to the floor, once.
    ///
    /// Mappers place items at eye height and let `PlaceItem` settle them at load; the server runs
    /// that think a heartbeat after spawn. Ours never runs — we don't dispatch thinks — so without
    /// this the whole map's items float at the height they were typed at, and the navmesh attaches
    /// each to whatever cell is level with it rather than the one you stand in to pick it up.
    ///
    /// This is the one think worth running by hand, because it's not really behaviour: it's the
    /// finishing step of spawning.
    fn settle_items(&mut self) {
        for i in 0..self.entities.len() as u32 {
            let e = EntId(i);
            if self.entities[e].in_use && self.entities[e].think == Think::PlaceItem {
                self.run_think_now(e, Think::PlaceItem);
            }
        }
    }

    /// `SV_LinkEdict`'s absolute-box half: recompute `absmin`/`absmax` from the origin and bounds.
    ///
    /// The server does this on every `setorigin`/`setsize`, and plenty of game code reads the
    /// result — aim points come from the box's midpoint, the lightning gun's muzzle from
    /// `absmin.z + size.z * 0.7`. There's no PVS or area grid to maintain here: our "link" is only
    /// about keeping those derived fields true.
    pub(crate) fn link_edict(&mut self, e: EntId) {
        let v = &mut self.entities[e].v;
        v.absmin = v.origin + v.mins;
        v.absmax = v.origin + v.maxs;

        // An item is grown horizontally so it's easy to grab; everything else by an epsilon on all
        // three axes. Reproducing the asymmetry matters: a bot's pickup logic is tuned against the
        // box the server actually uses.
        if Flags::from_bits_truncate(v.flags as u32).contains(Flags::ITEM) {
            v.absmin.x -= ITEM_GRAB;
            v.absmin.y -= ITEM_GRAB;
            v.absmax.x += ITEM_GRAB;
            v.absmax.y += ITEM_GRAB;
        } else {
            v.absmin -= Vec3::splat(LINK_EPSILON);
            v.absmax += Vec3::splat(LINK_EPSILON);
        }
    }

    /// `SV_SetModel`, client side: adopt a model's name and, for the map's own brushwork, its shape.
    ///
    /// An inline submodel (`"*3"`) is a piece of the level — a door, a plat, a trigger — and its
    /// bounds come from the BSP. Note they're *world* coordinates and are copied verbatim, qbsp's
    /// one-unit shrink and all: a paper-thin door legitimately has an inside-out box, and
    /// "correcting" it here would put its travel a unit off.
    pub(crate) fn client_set_model(&mut self, e: EntId, name: &core::ffi::CStr) {
        let name = name.to_string_lossy().into_owned();

        // A submodel index is the whole of what a client needs to identify brushwork on the wire:
        // the mirror matches a networked entity to its shadow twin by this name.
        if let Some(n) = name.strip_prefix('*').and_then(|n| n.parse::<usize>().ok()) {
            if let Some((mins, maxs)) = self.host.submodel_bounds(n) {
                self.entities[e].v.modelindex = n as f32;
                self.set_size(e, mins, maxs);
                return;
            }
        }

        // An external model (a rocket, a backpack) has bounds the game sets itself via `setsize`;
        // all `setmodel` owes it is a non-zero `modelindex`, which is what `is_alive`-style checks
        // and the mirror's "does this exist" tests read. The real index would come from the model
        // list, and nothing client-side needs it to agree with the server's.
        self.entities[e].v.modelindex = 1.0;
    }

    /// `SV_DropToFloor`, client side: settle an entity onto the floor beneath it.
    ///
    /// This is what puts the map's items on the ground at load — and it is not optional. An item
    /// whose drop fails is **deleted** ("bonus item fell out of level"), so a client that gets this
    /// wrong doesn't spawn a slightly-off world, it spawns a world with no items in it.
    ///
    /// The subtlety is that Quake traces the item's *own box*, not its origin. `SV_HullForEntity`
    /// picks a precompiled hull by the box's size and then shifts the trace by
    /// `hull.clip_mins - ent.mins`, which maps the entity's box onto the hull the BSP was compiled
    /// with. An item is typically `(0,0,0)..(32,32,56)`, so it traces 16 across and 24 up from its
    /// origin — trace the bare origin instead and it starts inside the floor, `allsolid`, and dies.
    pub(crate) fn client_droptofloor(&mut self, e: EntId) -> bool {
        let (origin, mins, maxs) = {
            let v = &self.entities[e].v;
            (v.origin, v.mins, v.maxs)
        };
        let offset = hull_offset(mins, maxs);

        let start = origin - offset;
        let tr = self.host.world_trace(start, start - Vec3::new(0.0, 0.0, DROP_DISTANCE));
        if tr.all_solid || tr.fraction == 1.0 {
            return false; // buried in a wall, or nothing under it within reach
        }

        self.set_origin(e, tr.endpos + offset);
        let v = &mut self.entities[e].v;
        v.flags = v.flags.with(Flags::ONGROUND);
        v.groundentity = EntId::WORLD.to_prog();
        true
    }

    /// `SV_Move`, client side: trace a line against the map and the players in it.
    ///
    /// Enough for what the brain asks of it — "can I see you", "is there a wall between us", "what
    /// is under my feet". It knows about the world hull and player boxes, and deliberately not
    /// about the rest: a client's entity picture is PVS-culled anyway, so a trace here is already a
    /// question about what we can see.
    pub(crate) fn client_traceline(&mut self, start: Vec3, end: Vec3, ignore: EntId) -> TraceResult {
        // The map lives on the host, not in `nav` — the navmesh is built *later*, from the world
        // this very trap helps spawn. Reading it from `nav.bsp` would make every trace during the
        // spawn hit a map that isn't loaded yet, and `droptofloor` would delete the map's items as
        // having "fallen out of the level".
        let world = self.host.world_trace(start, end);

        let mut best = TraceResult {
            allsolid: world.all_solid,
            startsolid: world.start_solid,
            fraction: world.fraction,
            endpos: world.endpos,
            plane_normal: world.plane_normal,
            ent: EntId::WORLD,
            in_open: !world.start_solid,
            in_water: false,
        };

        // Players are the only entities worth blocking a trace here: they're what "can I see you"
        // is about, and a rocket in flight doesn't stop a line of sight.
        let maxclients = self.host.cvar(c"maxclients") as u32;
        for i in 1..=maxclients {
            let p = EntId(i);
            if p == ignore || !self.entities[p].in_use || !self.entities[p].is_alive() {
                continue;
            }
            let v = &self.entities[p].v;
            let Some(hit) = ray_box(start, end, v.absmin, v.absmax) else { continue };
            if hit < best.fraction {
                best.fraction = hit;
                best.endpos = start + (end - start) * hit;
                best.ent = p;
                best.plane_normal = Vec3::ZERO;
                best.allsolid = false;
            }
        }
        best
    }
}

/// How far to shift a trace so an entity's box lines up with the hull the BSP was compiled with
/// (`SV_HullForEntity`).
///
/// Quake doesn't trace boxes; it traces *points* through hulls whose planes were pushed out by a
/// box's size at compile time. To move a box of your own, you offset into the hull that matches it.
/// Which hull that is depends on the box's width — under 3 units is a point (hull 0), up to 32 the
/// player hull (1), larger the crouch hull (2) — and we only have hull 1's clipnodes, which covers
/// items and players. A wider entity traces a little small, which for a client that never moves one
/// is a detail; it would matter to a server.
fn hull_offset(mins: Vec3, maxs: Vec3) -> Vec3 {
    let _ = maxs; // hull choice is by width alone, but the box is what's being mapped
    crate::defs::VEC_HULL_MIN - mins
}

/// Slab-method ray/box intersection: the fraction along `start`→`end` at which the segment first
/// enters the box, or `None` if it misses or only touches behind the start.
///
/// A start *inside* the box yields `0.0` — you can't see past someone you're standing in.
fn ray_box(start: Vec3, end: Vec3, mins: Vec3, maxs: Vec3) -> Option<f32> {
    let dir = end - start;
    let (mut tmin, mut tmax) = (0.0f32, 1.0f32);

    for i in 0..3 {
        if dir[i].abs() < 1e-6 {
            // Parallel to this slab: a miss unless we're already between its faces.
            if start[i] < mins[i] || start[i] > maxs[i] {
                return None;
            }
            continue;
        }
        let inv = 1.0 / dir[i];
        let (mut t1, mut t2) = ((mins[i] - start[i]) * inv, (maxs[i] - start[i]) * inv);
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return None;
        }
    }
    Some(tmin)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The slab test, including the cases that decide whether a bot thinks it can see someone.
    #[test]
    fn ray_box_hits_and_misses() {
        let (mins, maxs) = (Vec3::new(-16.0, -16.0, -24.0), Vec3::new(16.0, 16.0, 32.0));
        let at = |o: Vec3| (mins + o, maxs + o);

        // Straight through a box 100 units away: enters at 84/200 of the way.
        let (bmin, bmax) = at(Vec3::new(100.0, 0.0, 0.0));
        let f = ray_box(Vec3::ZERO, Vec3::new(200.0, 0.0, 0.0), bmin, bmax).expect("hit");
        assert!((f - 84.0 / 200.0).abs() < 1e-4, "{f}");

        // Beside it, and short of it.
        assert!(ray_box(Vec3::new(0.0, 100.0, 0.0), Vec3::new(200.0, 100.0, 0.0), bmin, bmax).is_none());
        assert!(ray_box(Vec3::ZERO, Vec3::new(50.0, 0.0, 0.0), bmin, bmax).is_none(), "stops short");

        // Behind us doesn't count — a trace is a segment, not a line.
        assert!(ray_box(Vec3::ZERO, Vec3::new(-200.0, 0.0, 0.0), bmin, bmax).is_none());

        // Standing inside: fraction 0, because you can't see past what you're in.
        let (bmin, bmax) = at(Vec3::ZERO);
        assert_eq!(ray_box(Vec3::ZERO, Vec3::new(200.0, 0.0, 0.0), bmin, bmax), Some(0.0));
    }

    /// A ray exactly parallel to a slab must not divide by zero into a false hit or a false miss.
    #[test]
    fn ray_box_handles_parallel_rays() {
        let (mins, maxs) = (Vec3::new(-16.0, -16.0, -24.0), Vec3::new(16.0, 16.0, 32.0));
        // Parallel to X, inside the Y/Z slabs → hits.
        assert!(ray_box(Vec3::new(-100.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 0.0), mins, maxs).is_some());
        // Parallel to X, outside the Y slab → misses, rather than dividing by zero.
        assert!(ray_box(Vec3::new(-100.0, 99.0, 0.0), Vec3::new(100.0, 99.0, 0.0), mins, maxs).is_none());
        // A zero-length trace at a point inside the box.
        assert_eq!(ray_box(Vec3::ZERO, Vec3::ZERO, mins, maxs), Some(0.0));
    }
}
