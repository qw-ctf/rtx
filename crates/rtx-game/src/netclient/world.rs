// SPDX-License-Identifier: AGPL-3.0-or-later

//! The engine's world traps, done ourselves: shadow-world spawn, linking, `setmodel`, and
//! `droptofloor`. (Line tracing is no longer here or host-specific: both embodiments answer
//! `traceline` from the parsed BSP + entity array through [`GameState::sv_trace`](crate::game).)
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
use crate::game::GameState;
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
        // Our own parsed BSP (hull 1 — "would a player fit"). Populated at map load
        // (`load_map_bsp`, inside `load_entities`), which `spawn_shadow_world` runs before it settles
        // items, so the map is here by now. No map → leave the item where it is.
        let Some(bsp) = self.nav.bsp.as_deref() else {
            return false;
        };
        let tr = bsp.hull1_trace(start, start - Vec3::new(0.0, 0.0, DROP_DISTANCE));
        if tr.all_solid || tr.fraction == 1.0 {
            return false; // buried in a wall, or nothing under it within reach
        }

        self.set_origin(e, tr.endpos + offset);
        let v = &mut self.entities[e].v;
        v.flags = v.flags.with(Flags::ONGROUND);
        v.groundentity = EntId::WORLD.to_prog();
        true
    }

}

/// How far to shift a trace so an entity's box lines up with the hull the BSP was compiled with
/// (`SV_HullForEntity`).
///
/// Quake doesn't trace boxes; it traces *points* through hulls whose planes were pushed out by a
/// box's size at compile time. To move a box of your own, you offset into the hull that matches it.
/// qbsp compiles four, chosen by the box's width:
///
/// | hull | box | what uses it |
/// |---|---|---|
/// | 0 | a point | shooting, sight — anything with no width |
/// | 1 | 32×32×56 | players, and everything player-sized: items, packs |
/// | 2 | 64×64×88 | the big monsters (a shambler) |
/// | 3 | — | unused by Quake |
///
/// We carry hulls 0 and 1, which is every question a deathmatch client asks: nothing here is
/// shambler-sized, and hull 3 doesn't exist. A wider entity would trace a little small — a detail
/// for a client that never moves one, and a bug for a server.
fn hull_offset(mins: Vec3, maxs: Vec3) -> Vec3 {
    let _ = maxs; // the hull is chosen by width, but it's the box's mins the offset maps
    crate::defs::VEC_HULL_MIN - mins
}

