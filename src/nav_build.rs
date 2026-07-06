// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh build orchestration — the on-demand, off-the-load-path construction of the bot
//! navigation mesh and the map-entity scans that feed it (item goals, plats, teleporters, gates).
//! Split out of `game.rs`: this is bot infrastructure that happens to hang off `GameState`, not
//! core game plumbing.

use glam::Vec3;

use crate::bot::goals as bot_goals;
use crate::defs;
use crate::entity::EntId;
use crate::game::{cstring, GameState};
use crate::navmesh;

impl GameState {
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

    /// Ensure the map's navmesh is (being) built. The heavy graph construction runs on a worker
    /// thread from `Send` inputs gathered here (BSP bytes + entity-derived plats/teleports/gates);
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

        let path = cstring(&format!("maps/{}.bsp", self.level.mapname));
        let Some(bytes) = self.host.read_file(&path) else {
            self.host
                .dprint(c"rtx: navmesh: could not read map BSP; bots disabled\n");
            return;
        };
        // Gather the entity-derived inputs on the main thread (they read the spawned entities),
        // then hand everything to a worker thread for the pure, potentially-slow graph build.
        let plats = self.collect_plats();
        let teleports = self.collect_teleports();
        let gates = self.collect_gates();
        // Hook links are only worth building when the map hands out the grapple. Snapshot the live
        // physics (gravity is 100 on e1m8; the hook speeds are tunable) so the arc solver on the
        // worker thread matches how the hook will actually fly in-game.
        let hooks = self.host.cvar_bool(c"rtx_grapple").then(|| navmesh::HookParams {
            gravity: self.host.cvar(c"sv_gravity").max(1.0),
            pull: navmesh::HOOK_PULL_BASE * self.host.cvar(c"rtx_hook_pull"),
            throw: navmesh::HOOK_THROW_BASE * self.host.cvar(c"rtx_hook_speed"),
        });
        // Double-jump links: only when the map allows the mid-air jump, so bots plan the wider gaps.
        let double_jump = self.host.cvar_bool(c"rtx_doublejump");
        // Speed-jump links (bhop-carried leaps): only when bots bunnyhop, with the physics that turn
        // a runway length into attainable speed.
        let speed_jump = self.host.cvar_bool(c"rtx_bot_bhop").then(|| navmesh::SpeedJumpParams {
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
        });
        // Rocket-jump links: only when bots may rocket-jump. Snapshot gravity and the `rj` self-boost
        // cvar (off by default) so the offline blast solve matches the live knockback.
        let rocket_jump = self.host.cvar_bool(c"rtx_bot_rocketjump").then(|| navmesh::RocketJumpParams {
            gravity: self.host.cvar(c"sv_gravity").max(1.0),
            rj_extra: self.host.cvar(c"rj"),
        });
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(navmesh::build_navmesh(
                bytes,
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
    /// in. A `None` result (unparseable BSP) or a dead worker just clears the pending build.
    fn poll_navmesh_build(&mut self) {
        let Some(rx) = self.nav.pending.as_ref() else {
            return;
        };
        let built = match rx.try_recv() {
            Ok(built) => built,
            Err(std::sync::mpsc::TryRecvError::Empty) => return, // still building
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.nav.pending = None;
                return;
            }
        };
        self.nav.pending = None;
        let Some((bsp, graph)) = built else {
            self.host
                .dprint(c"rtx: navmesh: unsupported/malformed BSP; bots disabled\n");
            return;
        };
        let counts = graph.summary();
        let goals = self.collect_goals(&graph);
        let msg = cstring(&format!(
            "rtx: navmesh: {} planes, {} clipnodes -> {} cells, {} links \
             (walk {} step {} drop {} jump {} djump {} sjump {} plat {} tele {} hook {} rjump {}), {} gates, {} item goals\n",
            bsp.planes.len(),
            bsp.clipnodes.len(),
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
        ));
        self.host.dprint(&msg);
        self.nav.bsp = Some(bsp);
        self.nav.graph = Some(graph);
        self.nav.goals = goals;
    }

    /// Build the static item-goal catalog: every spawned pickup (weapons, health, armor, ammo,
    /// powerups) paired with the navmesh cell nearest it. Items don't move, so this is computed
    /// once with the navmesh; [`GameState::select_item_goal`] reads live availability per query.
    fn collect_goals(&self, graph: &navmesh::NavGraph) -> Vec<(u32, navmesh::CellId)> {
        self.entities
            .iter()
            .enumerate()
            .filter_map(|(i, ent)| {
                let cn = ent.classname()?;
                if i == 0 || !bot_goals::is_goal_classname(cn) {
                    return None;
                }
                Some((i as u32, graph.nearest(ent.v.origin)?))
            })
            .collect()
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
