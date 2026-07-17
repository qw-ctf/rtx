// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh build orchestration — the on-demand, off-the-load-path construction of the bot
//! navigation mesh and the map-entity scans that feed it (item goals, plats, teleporters, gates).
//! Split out of `game.rs`: this is bot infrastructure that happens to hang off `GameState`, not
//! core game plumbing.

use glam::Vec3;

use crate::bot::goals as bot_goals;
use crate::cvars::{default_of, CvarSeed};
use crate::defs;
use crate::entity::EntId;
use crate::game::{cstring, GameState};
use crate::navmesh;

/// Live status of a navmesh plat, for plat-aware boarding: whether it currently rests at its
/// bottom (safe to walk aboard) and the Z of its top surface (to tell an approaching bot from one
/// already riding). A freed plat reports as down so a bot never waits on a lift that's gone.
pub(crate) struct PlatStatus {
    pub down: bool,
    pub surface_z: f32,
}

impl GameState {
    /// Read an rtx bool cvar the navmesh build depends on, falling back to its **registered default**
    /// when the cvar isn't set yet. `GAME_INIT`'s `cvar_default` seeds each rtx cvar through a *queued*
    /// `set` (it can't flush during init — `pr_global_struct` is still null), and that queue is not
    /// guaranteed to have run before the first-frame navmesh build reads the cvar. An unset live read
    /// returns `0`/false, which would silently gate a build input off — the first-boot bug where the
    /// first map builds with no rocket jumps (`rjump 0`) until a later `map` rebuilds correctly. This
    /// returns the value the build *would* see once flushed: the live value if set, else the default.
    pub(crate) fn rtx_cvar_bool(&self, name: &str) -> bool {
        if self.host.cvar_is_set(name) {
            self.host.cvar_bool(&cstring(name))
        } else {
            matches!(default_of(name), Some(CvarSeed::Bool(true)))
        }
    }

    /// Float counterpart of [`rtx_cvar_bool`](Self::rtx_cvar_bool): the live value if the cvar is set,
    /// else its registered default (or `0.0` for a cvar with no float default). Same first-flush
    /// robustness for the numeric build inputs (hook speeds, …).
    pub(crate) fn rtx_cvar_f32(&self, name: &str) -> f32 {
        if self.host.cvar_is_set(name) {
            self.host.cvar(&cstring(name))
        } else if let Some(CvarSeed::Float(f)) = default_of(name) {
            f
        } else {
            0.0
        }
    }

    /// Build the bot navmesh for the current map on demand — the first time bots are wanted —
    /// then cache it. Attempted at most once per map (a failed read won't retry every frame).
    /// Best-effort: if the BSP can't be read or parsed the navmesh stays empty and bots simply
    /// don't spawn — never fatal. Deferring this off the load path means a bot-less server
    /// pays neither the build time nor the memory.
    /// Live door states for gate-aware pathfinding: `[i]` is true while gate `i`'s door is shut
    /// (present at its closed origin). A door that slid open, or one a button *removed* (freed, so
    /// `in_use` is cleared), reads as open. Empty when there's no navmesh or no gates.
    pub(crate) fn gate_closed_flags(&self) -> Vec<bool> {
        let Some(graph) = self.nav.graph.as_ref() else {
            return Vec::new();
        };
        (0..graph.gate_count())
            .map(|gi| {
                let g = graph.gate(gi);
                let obs = &self.entities[EntId(g.obstruction)];
                obs.in_use && (obs.v.origin - g.closed_origin).length() < 8.0
            })
            .collect()
    }

    /// Live status for every navmesh plat, in `plat_of_link` index order — the runtime uses it to
    /// hold a standoff outside a raised lift's inner trigger (which resets its lower-timer while any
    /// live player stands inside) and to board only once it's down. A freed lift reads as down.
    pub(crate) fn plat_statuses(&self) -> Vec<PlatStatus> {
        let Some(graph) = self.nav.graph.as_ref() else {
            return Vec::new();
        };
        (0..graph.plat_count())
            .map(|pi| {
                let p = &self.entities[EntId(graph.plat(pi).entity)];
                PlatStatus {
                    down: !p.in_use || p.mover.state == crate::entity::MoverPhase::Bottom,
                    surface_z: p.v.origin.z + p.v.maxs.z,
                }
            })
            .collect()
    }

    /// Parse the current map's BSP once and cache it on `nav.bsp` as a shared `Arc`. Called at
    /// entity load (see `load_entities`), before anything queries the world, and independent of
    /// whether bots are wanted: the `pointcontents` and world-trace paths read this in both
    /// embodiments, even on a bot-free server. The worker navmesh build then shares the same `Arc`.
    /// Best-effort — a read or parse failure leaves `nav.bsp` as `None`, so world queries answer as
    /// open air and bots stay disabled; never fatal.
    pub(crate) fn load_map_bsp(&mut self) {
        let path = cstring(&format!("maps/{}.bsp", self.level.mapname));
        let Some(bytes) = self.host.read_file(&path) else {
            self.host.dprint(c"rtx: navmesh: could not read map BSP\n");
            return;
        };
        match crate::bsp::Bsp::parse(&bytes) {
            Some(bsp) => self.nav.bsp = Some(std::sync::Arc::new(bsp)),
            None => self.host.dprint(c"rtx: navmesh: unsupported/malformed BSP\n"),
        }
    }

    /// Ensure the map's navmesh is (being) built. The heavy graph construction runs on a worker
    /// thread from `Send` inputs gathered here (the parsed BSP + entity-derived plats/teleports/gates);
    /// the result is polled each frame and swapped in atomically when ready, so a big map never
    /// hitches the server frame. Bots stay disabled until the swap lands.
    pub(crate) fn ensure_navmesh(&mut self) {
        if self.nav.graph.is_some() {
            return; // already built
        }
        if self.nav.pending.is_some() {
            self.poll_navmesh_build(); // a build is in flight — install it once ready
            return;
        }
        if self.nav.attempted {
            return; // a prior read/parse failed; don't retry until the next map
        }
        self.nav.attempted = true;

        // The BSP was parsed once at map load (`load_map_bsp`); share it (`Arc`) into the worker.
        // A missing parse means the map couldn't be read — bots simply stay disabled.
        let Some(bsp) = self.nav.bsp.clone() else {
            self.host
                .dprint(c"rtx: navmesh: map BSP not parsed; bots disabled\n");
            return;
        };
        // Gather the entity-derived inputs on the main thread (they read the spawned entities),
        // then hand everything to a worker thread for the pure, potentially-slow graph build.
        let plats = self.collect_plats();
        let teleports = self.collect_teleports();
        let gates = self.collect_gates();
        // A stock-movement mode (race) bans grapple / double-jump / rocket-jump traversal, so
        // their links must not exist in the graph at all — bots then can't plan them, and a
        // failed pathfind (the race routability check) is a truthful "not traversable".
        let stock = self.mode.stock_movement_only();
        // Hook links are only worth building when the map hands out the grapple. Snapshot the live
        // physics (gravity is 100 on e1m8; the hook speeds are tunable) so the arc solver on the
        // worker thread matches how the hook will actually fly in-game.
        // Every rtx cvar that gates a build input is read through `rtx_cvar_*` so an unflushed default
        // on a fresh boot doesn't silently disable the links (the first-boot `rjump 0` bug); engine
        // cvars (sv_*) are always set, so they read live.
        let hooks = (!stock && self.rtx_cvar_bool("rtx_grapple")).then(|| navmesh::HookParams {
            gravity: self.host.cvar(c"sv_gravity").max(1.0),
            pull: navmesh::HOOK_PULL_BASE * self.rtx_cvar_f32("rtx_hook_pull"),
            throw: navmesh::HOOK_THROW_BASE * self.rtx_cvar_f32("rtx_hook_speed"),
        });
        // Double-jump links: only when the map allows the mid-air jump, so bots plan the wider gaps.
        let double_jump = !stock && self.rtx_cvar_bool("rtx_doublejump");
        // Speed-jump links (bhop-carried leaps): only when bots bunnyhop, with the physics that turn
        // a runway length into attainable speed.
        let speed_jump = self.rtx_cvar_bool("rtx_bot_bhop").then(|| navmesh::SpeedJumpParams {
            gravity: self.host.cvar(c"sv_gravity").max(1.0),
            accel: {
                let a = self.host.cvar(c"sv_accelerate");
                if a > 0.0 {
                    a
                } else {
                    10.0
                }
            },
            maxspeed: {
                let m = self.host.cvar(c"sv_maxspeed");
                if m > 0.0 {
                    m
                } else {
                    320.0
                }
            },
            friction: {
                let f = self.host.cvar(c"sv_friction");
                if f > 0.0 {
                    f
                } else {
                    4.0
                }
            },
            stopspeed: {
                let s = self.host.cvar(c"sv_stopspeed");
                if s > 0.0 {
                    s
                } else {
                    100.0
                }
            },
            // Curl jumps (run-up + air-turn onto an offset landing), certified by a pmove rollout in
            // the build. Sub-toggle of bhop; on by default so bots take the human curl routes. Disabled
            // on a genuinely slick server (`sv_friction` set below ~1): the ground prestrafe never
            // reaches an equilibrium there, so the certifier's takeoff-speed model — and every curl it
            // mints — would be wrong (the bot arrives far over the certified envelope and overshoots).
            curl: self.rtx_cvar_bool("rtx_bot_curljump") && self.host.cvar(c"sv_friction") >= 1.0,
        });
        // Rocket-jump links: only when bots may rocket-jump. Snapshot gravity and the `rj` self-boost
        // cvar (off by default) so the offline blast solve matches the live knockback.
        let rocket_jump = (!stock && self.rtx_cvar_bool("rtx_bot_rocketjump")).then(|| navmesh::RocketJumpParams {
            gravity: self.host.cvar(c"sv_gravity").max(1.0),
            rj_extra: self.host.cvar(c"rj"),
        });
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(navmesh::build_navmesh(
                &bsp,
                plats,
                teleports,
                gates,
                hooks,
                double_jump,
                speed_jump,
                rocket_jump,
            ));
        });
        self.nav.pending = Some(rx);
        self.host.dprint(c"rtx: navmesh: building in background...\n");
    }

    /// Poll the in-flight background build; when it delivers, compute item goals and swap the graph
    /// in. The worker now returns a fully-priced graph — liquid flags and LOD are baked on the worker
    /// (it holds the parsed BSP), so the swap frame does only goal collection + the summary log, no
    /// main-thread pointcontents pass. A dead worker just clears the pending build.
    fn poll_navmesh_build(&mut self) {
        let Some(rx) = self.nav.pending.as_ref() else {
            return;
        };
        let graph = match rx.try_recv() {
            Ok(graph) => graph,
            Err(std::sync::mpsc::TryRecvError::Empty) => return, // still building
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.nav.pending = None;
                return;
            }
        };
        self.nav.pending = None;
        let counts = graph.summary();
        let goals = self.collect_goals(&graph);
        let (lclusters, lportals, ledges, lreach) = graph.lod_stats();
        let (planes, clipnodes) = self
            .nav
            .bsp
            .as_ref()
            .map_or((0, 0), |b| (b.planes.len(), b.clipnodes.len()));
        let msg = cstring(&format!(
            "rtx: navmesh: {} planes, {} clipnodes -> {} cells, {} links \
             (walk {} step {} drop {} jump {} djump {} sjump {} plat {} tele {} hook {} rjump {}), {} gates, {} item goals; \
             lod {} clusters {} portals {} edges {} reach\n",
            planes,
            clipnodes,
            graph.cells.len(),
            graph.links.len(),
            counts.walk,
            counts.step,
            counts.drop,
            counts.jump,
            counts.double_jump,
            counts.speed_jump,
            counts.plat,
            counts.teleport,
            counts.hook,
            counts.rocket_jump,
            graph.gate_count(),
            goals.len(),
            lclusters,
            lportals,
            ledges,
            lreach,
        ));
        self.host.dprint(&msg);
        self.nav.graph = Some(graph);
        self.nav.goals = goals;
    }

    /// Build the static item-goal catalog: every spawned pickup (weapons, health, armor, ammo,
    /// powerups) paired with every navmesh cell whose standing player origin overlaps its pickup
    /// trigger. Items don't move, so this is computed once with the navmesh;
    /// [`GameState::select_item_goal`] reads live availability per query.
    fn collect_goals(&self, graph: &navmesh::NavGraph) -> Vec<(u32, navmesh::CellId)> {
        let cells = || {
            graph
                .cells
                .iter()
                .enumerate()
                .map(|(cell, c)| (cell as navmesh::CellId, c.origin))
        };
        let mut goals = Vec::new();
        for (i, ent) in self.entities.iter().enumerate() {
            let Some(cn) = ent.classname() else {
                continue;
            };
            // `in_use` matters: a freed slot keeps its classname until something reuses it, and an
            // item that failed `droptofloor` at load was deleted for having fallen out of the level.
            // Cataloguing it anyway would send bots to stand forever where an item isn't.
            if i == 0 || !ent.in_use || !bot_goals::is_goal_classname(cn) {
                continue;
            }
            goals.extend(
                collect_touch_terminals(cells(), ent)
                    .into_iter()
                    .map(|cell| (i as u32, cell)),
            );
        }
        goals
    }

    /// Gather the [`PlatInfo`](crate::navmesh::PlatInfo) for every spawned `func_plat`: the
    /// player-origin standing spots on the plat surface at the bottom and top of its travel.
    /// The plat moves only in Z (`pos2`→`pos1`); its top surface is `maxs.z` above the origin,
    /// and a standing player origin sits 24 (`-mins.z`) above that surface.
    fn collect_plats(&self) -> Vec<navmesh::PlatInfo> {
        self.find_by_classname("plat")
            .map(|e| {
                let ent = &self.entities[e];
                let (pos1, pos2) = (ent.mover.pos1, ent.mover.pos2);
                let (mins, maxs) = (ent.v.mins, ent.v.maxs);
                let cx = pos1.x + (mins.x + maxs.x) * 0.5;
                let cy = pos1.y + (mins.y + maxs.y) * 0.5;
                navmesh::PlatInfo {
                    board: Vec3::new(cx, cy, pos2.z + maxs.z + 24.0),
                    exit: Vec3::new(cx, cy, pos1.z + maxs.z + 24.0),
                    entity: e.0,
                    // World-XY footprint of the brush (XY is the same at pos1/pos2 — travel is Z-only).
                    fp_min: glam::Vec2::new(pos1.x + mins.x, pos1.y + mins.y),
                    fp_max: glam::Vec2::new(pos1.x + maxs.x, pos1.y + maxs.y),
                    // The brush's bottom face with the lift at rest — the shaft floor the swept volume
                    // starts at (cells below it are under that floor, not in the lift's way).
                    bottom: pos2.z + mins.z,
                }
            })
            .collect()
    }

    /// Gather the [`TeleportInfo`](crate::navmesh::TeleportInfo) for every `trigger_teleport`:
    /// its world-space trigger box and the destination's arrival origin (resolved through the
    /// `target` → `targetname` link, exactly as `teleport_touch` does at runtime).
    fn collect_teleports(&self) -> Vec<navmesh::TeleportInfo> {
        self.find_by_classname("trigger_teleport")
            .filter_map(|e| {
                let ent = &self.entities[e];
                let target = ent.target.clone()?;
                let dest = self.find_by_targetname(&target).next()?;
                let origin = ent.v.origin;
                Some(navmesh::TeleportInfo {
                    tmin: origin + ent.v.mins,
                    tmax: origin + ent.v.maxs,
                    dest: self.entities[dest].v.origin,
                })
            })
            .collect()
    }

    /// Gather the [`GateInfo`](crate::navmesh::GateInfo) for every button-gated obstruction: a
    /// targeted `func_door` (slides; rests closed at `pos1`) or a blocking `func_movewall`
    /// (rotates, driven by a rotator; rests at its current origin). Each is paired with the
    /// `func_button` that ultimately opens it, found by following `target` → `targetname` back
    /// from the obstruction — directly for a door, or through the rotator for a movewall.
    fn collect_gates(&self) -> Vec<navmesh::GateInfo> {
        // (obstruction, closed origin) for every entity that blocks a path until triggered.
        let mut obstructions: Vec<(EntId, Vec3)> = Vec::new();
        for d in self.find_by_classname("door") {
            if self.entities[d].targetname.is_some() {
                obstructions.push((d, self.entities[d].mover.pos1));
            }
        }
        for w in self.find_by_classname("func_movewall") {
            let e = &self.entities[w];
            if e.targetname.is_some() && e.v.solid == defs::Solid::Bsp {
                obstructions.push((w, e.v.origin));
            }
        }

        let mut gates = Vec::new();
        for (obs, closed_origin) in obstructions {
            let tn = self.entities[obs].targetname.clone().unwrap();
            let Some((activator, shoot)) = self.find_activator(&tn, 0) else {
                self.host.dprint(&cstring(&format!(
                    "rtx: navmesh: gate '{tn}' has no reachable button/trigger — skipped\n"
                )));
                continue;
            };
            let oent = &self.entities[obs];
            let bent = &self.entities[activator];
            gates.push(navmesh::GateInfo {
                obstruction: obs.0,
                closed_origin,
                closed_min: closed_origin + oent.v.mins,
                closed_max: closed_origin + oent.v.maxs,
                activator: activator.0,
                button: bent.v.origin + (bent.v.mins + bent.v.maxs) * 0.5,
                shoot,
            });
        }
        gates
    }

    /// Follow `target`/`killtarget` → `targetname` from `tn` back to the player-facing activator
    /// that fires the chain: a `func_button` (door gates), or a shootable/touchable trigger
    /// (`trigger_multiple`/`trigger_once`) reached through rotators and relays (movewall gates).
    /// A button that *removes* the obstruction does so via `killtarget` (the door is deleted
    /// rather than slid), so both keys are followed. Returns the activator and whether it's shot
    /// (`health > 0`). Depth-bounded against cycles.
    fn find_activator(&self, tn: &str, depth: u32) -> Option<(EntId, bool)> {
        if depth > 5 {
            return None;
        }
        for (i, e) in self.entities.iter().enumerate() {
            if !e.in_use || (e.target.as_deref() != Some(tn) && e.killtarget.as_deref() != Some(tn)) {
                continue;
            }
            // An intermediate (rotator, relay) is itself triggered — follow the chain back.
            if let Some(next) = e.targetname.clone() {
                if let Some(found) = self.find_activator(&next, depth + 1) {
                    return Some(found);
                }
                continue;
            }
            // Leaf: the thing a player shoots or touches to fire the chain.
            if matches!(e.classname(), Some("func_button" | "trigger_multiple" | "trigger_once")) {
                return Some((EntId(i as u32), e.v.health > 0.0));
            }
        }
        None
    }
}

/// Player-origin nav cells that already overlap an item's pickup trigger. A route ending on one of
/// these cells completes the pickup by standing there; it never relies on a final beeline through
/// nearby geometry to the entity origin.
pub(crate) fn collect_touch_terminals(
    cells: impl IntoIterator<Item = (navmesh::CellId, Vec3)>,
    item: &crate::entity::Entity,
) -> Vec<navmesh::CellId> {
    cells
        .into_iter()
        .filter_map(|(cell, origin)| crate::bot::item_terminal_touches(origin, item).then_some(cell))
        .collect()
}

#[cfg(all(test, feature = "netclient"))]
mod tests {
    use super::*;
    use crate::bsp::Bsp;
    use crate::defs::{Bits, Items, Solid};
    use crate::entity::{Entity, Touch};
    use crate::netclient::host::NetHost;
    use crate::navmesh::NavGraph;
    use std::path::PathBuf;

    fn armor_entity(classname: &str, origin: Vec3) -> Entity {
        let mut armor = Entity::default();
        armor.in_use = true;
        armor.classname = Some(classname.into());
        armor.v.origin = origin;
        armor.v.mins = Vec3::new(-16.0, -16.0, 0.0);
        armor.v.maxs = Vec3::new(16.0, 16.0, 56.0);
        armor
    }

    fn assert_armor_take(classname: &str, item_origin: Vec3, terminal: Vec3) {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(PathBuf::from("/nonexistent"))));
        host.set("maxclients", "8");
        let mut game = GameState::new_client(host);
        let (player, armor) = (EntId(1), EntId(32));
        {
            let ent = &mut game.entities[player];
            ent.in_use = true;
            ent.classname = Some("player".into());
            ent.v.health = 100.0;
            ent.v.origin = terminal;
        }
        let mut armor_ent = armor_entity(classname, item_origin);
        armor_ent.v.solid = Solid::Trigger;
        armor_ent.set_touch(Touch::ItemArmor);
        game.entities[armor] = armor_ent;

        // This is the engine's touch-dispatch gate: only overlapping linked trigger/player hulls
        // dispatch GAME_EDICT_TOUCH. The dispatch itself is the production server-side armor path.
        assert!(crate::bot::item_terminal_touches(terminal, &game.entities[armor]));
        game.run_touch(armor, player);

        assert!(game.entities[player].v.armorvalue > 0.0, "terminal arrival must execute an armor take");
        assert!(
            game.entities[player].v.items.has(Items::ARMOR2 | Items::ARMOR3),
            "the take must change armor inventory, not merely satisfy a selector"
        );
        assert_eq!(game.entities[armor].v.solid, Solid::Not, "the server-side pickup handler consumed it");
    }

    fn armor_take_from_terminal(classname: &str, item_origin: Vec3, bad_endpoint: Vec3) {
        let valid_endpoint = item_origin + Vec3::new(0.0, 0.0, 24.0);
        let cells = [(7, bad_endpoint), (8, valid_endpoint)];
        let armor = armor_entity(classname, item_origin);
        let terminals = collect_touch_terminals(cells, &armor);

        assert_eq!(terminals, vec![8], "the observed stall cell must not catalogue as a pickup terminal");
        assert!(
            !crate::bot::item_terminal_touches(bad_endpoint, &armor),
            "the observed endpoint must fail the same hull-overlap gate that dispatches server touch"
        );
        assert_armor_take(classname, item_origin, valid_endpoint);
    }

    #[test]
    fn dm3_ra_wrong_side_endpoint_rebinds_to_a_real_take() {
        armor_take_from_terminal(
            "item_armorInv",
            Vec3::new(256.0, -704.0, 304.0),
            Vec3::new(360.0, -677.0, 264.0),
        );
    }

    #[test]
    fn dm3_ya_upper_floor_endpoint_rebinds_to_a_real_take() {
        armor_take_from_terminal(
            "item_armor2",
            Vec3::new(1232.0, -904.0, -48.0),
            Vec3::new(1239.0, -887.0, 88.0),
        );
    }

    /// Optional real-map check used by the DM3 bench/ref loop. The always-on endpoint tests above
    /// exercise catalog filtering plus the real armor handler without shipping id's BSP in-tree;
    /// setting `RTX_TEST_BSP=.../dm3.bsp` additionally proves the generated DM3 graph exposes
    /// touch-valid terminals for both armor entities and that those exact cell origins take armor.
    #[test]
    fn dm3_real_navmesh_exposes_takeable_ra_and_ya_terminals() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        if !path.to_ascii_lowercase().contains("dm3") {
            return;
        }
        let bytes = std::fs::read(path).expect("read dm3 bsp");
        let bsp = Bsp::parse(&bytes).expect("parse dm3 bsp");
        let graph = NavGraph::build(&bsp);

        for (classname, item_origin, bad_endpoint) in [
            (
                "item_armorInv",
                Vec3::new(256.0, -704.0, 304.0),
                Vec3::new(360.0, -677.0, 264.0),
            ),
            (
                "item_armor2",
                Vec3::new(1232.0, -904.0, -48.0),
                Vec3::new(1239.0, -887.0, 88.0),
            ),
        ] {
            let armor = armor_entity(classname, item_origin);
            assert!(
                !crate::bot::item_terminal_touches(bad_endpoint, &armor),
                "observed endpoint is not a take"
            );
            let terminals = collect_touch_terminals(
                graph
                    .cells
                    .iter()
                    .enumerate()
                    .map(|(cell, c)| (cell as navmesh::CellId, c.origin)),
                &armor,
            );
            assert!(!terminals.is_empty(), "{classname} has no touch-valid DM3 terminal");
            let terminal = graph.cell_origin(terminals[0]);
            assert!(crate::bot::item_terminal_touches(terminal, &armor));
            assert_armor_take(classname, item_origin, terminal);
        }
    }
}
