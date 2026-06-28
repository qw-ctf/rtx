// SPDX-License-Identifier: AGPL-3.0-or-later

//! The single owner of all game state.
//!
//! `GameState` lives behind the one global (`OnceLock<Game>` in `lib.rs`) and owns the
//! entity array, the engine-shared globals, the field table, the returned [`GameData`],
//! and the host handle. The engine receives raw pointers into the heap `Box`es here at
//! `GAME_INIT` and keeps them for the process lifetime — those buffers are never
//! reallocated, so the pointers stay valid.

use core::ffi::CStr;
use std::ffi::CString;

use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::abi::{Field, GameData, GlobalVars, STRING_REF_COUNT, STRING_REF_WEAPONMODEL};
use crate::{defs, ext_field};
use crate::entity::{EntId, Entities, Entity};
use crate::game_command::GameCommand;
use crate::host::{HostApi, SyscallFn};
use crate::world;

/// Matches `MAX_EDICTS` in `ktx/include/q_shared.h`.
pub const MAX_EDICTS: usize = 2048;
/// Matches `GAME_API_VERSION` in `ktx/include/g_public.h`.
pub const GAME_API_VERSION: i32 = 16;

/// A single entity's parsed key/value pairs from the map's entity string.
type SpawnFields = Vec<(String, String)>;
type SpawnFn = fn(&mut GameState, EntId) -> bool;

#[derive(Clone, Copy)]
enum SpawnAction {
    Keep,
    Drop,
    Spawn(SpawnFn),
    Armor(f32),
    Weapon,
    Ammo {
        weapon_code: f32,
        netname: &'static str,
        small: Model,
        small_amt: f32,
        big: Model,
        big_amt: f32,
    },
    Powerup {
        model: Model,
        noise: Sound,
        netname: &'static str,
        item_bit: defs::Items,
        effect: defs::Effects,
    },
    Flame {
        model: Model,
        skin: f32,
        frame: bool,
    },
    Explobox {
        model: Model,
        size: Vec3,
    },
}

/// The result of a [`GameState::traceline`], read out of the engine's `trace_*` globals.
/// (Some fields are not yet consumed by the ported subset but complete the trace contract.)
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub struct TraceResult {
    pub allsolid: bool,
    pub startsolid: bool,
    pub fraction: f32,
    pub endpos: Vec3,
    pub plane_normal: Vec3,
    pub ent: EntId,
    pub in_open: bool,
    pub in_water: bool,
}

/// Match/level-wide state refreshed each frame from cvars (qw-qc globals).
#[derive(Default)]
pub struct Level {
    pub framecount: i32,
    pub deathmatch: i32,
    pub teamplay: i32,
    pub timelimit: i32,
    pub fraglimit: i32,
    pub skill: i32,
    /// Captured model indices for the player and the invisibility "eyes" model.
    pub modelindex_player: f32,
    pub modelindex_eyes: f32,
    /// `world.worldtype` (0 medieval / 1 runic / 2 base) — selects key-door message text.
    pub worldtype: f32,
    /// Current map name (without `maps/` and `.bsp`) and the queued next map.
    pub mapname: String,
    pub nextmap: String,
}

pub struct GameState {
    pub(crate) host: HostApi,
    /// The shared entity array. Heap-allocated once, never resized.
    pub(crate) entities: Entities,
    /// The shared globals block. Owned here to keep the buffer alive at a stable address;
    /// the engine also accesses it through `game_data.global`.
    pub(crate) globals: Box<GlobalVars>,
    /// Custom-field table handed to the engine (terminator-only for now). Owned for
    /// lifetime; accessed by the engine through `game_data.fields`.
    #[allow(dead_code)]
    fields: Box<[Field]>,
    /// The handshake reply; its `ents`/`global`/`fields` point into the boxes above.
    game_data: GameData,
    /// Match-wide state.
    pub level: Level,
    /// QuakeC transient globals threaded through target-firing and damage (`activator`,
    /// `damage_attacker`, `damage_inflictor`). Set at the top of the relevant callbacks.
    pub activator: EntId,
    pub damage_attacker: EntId,
    pub damage_inflictor: EntId,
    /// Intermission / level-exit state (server.qc).
    pub intermission_running: bool,
    pub intermission_exit_time: f32,
    /// PRNG state (QuakeC `random()` is a VM builtin with no native syscall, so we roll
    /// our own, seeded from `GAME_INIT`'s random seed).
    rng: u32,
    /// Resolved map-extension field references (entity `alpha`, …), looked up once per server.
    /// See [`Self::set_alpha`].
    ext_fields: ext_field::ExtFields,
    /// Auto-generated navmesh for the current map (bot navigation). Rebuilt each map load.
    pub(crate) nav: crate::navmesh::NavState,
    /// Escape hatch for string-declared sounds (precache-and-intern at load time). Empty for the
    /// current port — every sound is a registry handle — but here so a runtime path is registered
    /// through the same precache-guaranteeing door. Use `dyn_assets.sound(&host, path)`.
    #[allow(dead_code)]
    pub(crate) dyn_assets: crate::assets::DynAssets,
}

impl GameState {
    pub fn new(syscall: SyscallFn) -> Self {
        let mut entities = Entities::new(MAX_EDICTS);
        let host = HostApi::new(syscall, entities.as_mut_ptr());
        let mut globals = Box::new(GlobalVars::default());
        let fields: Box<[Field]> = Box::new([Field::TERMINATOR]);

        // Point the handshake struct at the stable heap buffers. These pointers survive
        // `self` being moved into the OnceLock because the Box *contents* don't move.
        let game_data = GameData {
            ents: entities.as_mut_ptr() as *mut u8,
            sizeofent: core::mem::size_of::<Entity>() as i32,
            global: globals.as_mut() as *mut GlobalVars,
            fields: fields.as_ptr(),
            api_version: GAME_API_VERSION,
            maxentities: MAX_EDICTS as i32,
        };

        GameState {
            host,
            entities,
            globals,
            fields,
            game_data,
            level: Level::default(),
            activator: EntId::WORLD,
            damage_attacker: EntId::WORLD,
            damage_inflictor: EntId::WORLD,
            intermission_running: false,
            intermission_exit_time: 0.0,
            rng: 0x2545_f491, // nonzero default; reseeded in GAME_INIT
            ext_fields: ext_field::ExtFields::default(),
            nav: crate::navmesh::NavState::default(),
            dyn_assets: crate::assets::DynAssets::default(),
        }
    }

    /// Handle one engine command. Returns the engine-expected `intptr_t`. Unknown raw
    /// command ids are filtered out before this is called (see `vmMain`).
    pub fn dispatch(&mut self, cmd: GameCommand, arg0: i32, arg1: i32, _arg2: i32) -> isize {
        let player = self.self_ent();
        let is_spectator = arg0 != 0;
        match cmd {
            GameCommand::Init => self.init(arg0, arg1),
            GameCommand::LoadEntities => self.load_entities(),
            GameCommand::StartFrame => self.start_frame(arg0, arg1),
            GameCommand::Shutdown => 0,
            GameCommand::ClientConnect if !is_spectator => {
                self.client_connect(player);
                1
            }
            GameCommand::PutClientInServer if !is_spectator => {
                self.put_client_in_server(player);
                1
            }
            GameCommand::ClientDisconnect if !is_spectator => {
                self.client_disconnect(player);
                1
            }
            GameCommand::ClientPreThink if !is_spectator => {
                self.player_pre_think(player);
                1
            }
            GameCommand::ClientPostThink if !is_spectator => {
                self.player_post_think(player);
                1
            }
            GameCommand::SetNewParams => {
                self.set_new_parms();
                1
            }
            GameCommand::SetChangeParams => {
                self.set_change_parms(player);
                1
            }
            GameCommand::EdictThink => {
                // `player` here is just the engine's current `self` entity.
                self.run_think(player);
                1
            }
            GameCommand::EdictTouch => {
                let other = self.other_ent();
                self.run_touch(player, other);
                1
            }
            GameCommand::EdictBlocked => {
                let other = self.other_ent();
                self.run_blocked(player, other);
                1
            }
            // Spectator paths (reached when `is_spectator`, since the player arms above are
            // guarded by `!is_spectator`).
            GameCommand::ClientConnect => {
                self.spectator_connect(player);
                1
            }
            GameCommand::PutClientInServer => {
                self.put_spectator_in_server(player);
                1
            }
            GameCommand::ClientDisconnect => {
                self.spectator_disconnect(player);
                1
            }
            GameCommand::ClientPostThink => {
                self.spectator_think(player);
                1
            }
            GameCommand::ClientPreThink | GameCommand::ClientThink => 1,
            GameCommand::ClientCommand => self.client_command(player),
            GameCommand::ClientSay => 1,
            GameCommand::ClearEdict => {
                // The engine has just zeroed this edict; re-establish its native string-field
                // indirection (mirrors ktx's `initialise_spawned_ent`).
                self.setup_string_refs(player);
                0
            }
            GameCommand::ClientUserInfoChanged | GameCommand::ConsoleCommand | GameCommand::PausedTic => 0,
        }
    }

    /// Re-establish the native string-field indirection for one entity. The engine zeroes an
    /// edict's memory immediately before `GAME_CLEAR_EDICT`, wiping both the `.string` slots
    /// and their backing cells, so this has to run for every cleared or freshly spawned slot.
    /// See [`EntVars::link_string_refs`] for the ABI; mirrors ktx's `initialise_spawned_ent`.
    fn setup_string_refs(&mut self, id: EntId) {
        // Byte offset, within the whole entity array, of this entity's first scratch cell.
        let base = id.index() * core::mem::size_of::<Entity>()
            + core::mem::offset_of!(Entity, string_refs);

        let ent = &mut self.entities[id];
        ent.string_refs = [0; STRING_REF_COUNT];
        ent.v
            .link_string_refs(|i| (base + i * core::mem::size_of::<u64>()) as i32);
    }

    /// Set the engine-visible `weaponmodel` string (the first-person viewmodel the engine
    /// resolves in `SV_WriteClientdata`). Unlike `model` there is no `setmodel` trap for it —
    /// QuakeC just assigns the string — so we write the `'static` `char*` straight into its
    /// indirection scratch cell (native `VM_MemoryBase` is 0, so a raw pointer is correct).
    /// `None` clears the viewmodel.
    pub(crate) fn set_weaponmodel(&mut self, e: EntId, model: Option<crate::assets::Model>) {
        let ptr = model.map_or(0, |m| m.path().as_ptr() as u64);
        self.entities[e].string_refs[STRING_REF_WEAPONMODEL] = ptr;
    }

    /// The entity the engine has made "current" (`self`). `globals.self_` is a *byte
    /// offset* into the entity array (QuakeC `PROG_TO_EDICT` convention), so divide by the
    /// per-entity stride to recover the index.
    fn self_ent(&self) -> EntId {
        EntId::from_prog(self.globals.self_)
    }

    /// The engine's current `other` entity (touch/use second party).
    pub(crate) fn other_ent(&self) -> EntId {
        EntId::from_prog(self.globals.other)
    }

    /// Current level time.
    #[inline]
    pub(crate) fn time(&self) -> f32 {
        self.globals.time
    }

    /// Allocate a fresh entity from the engine and wire up its string indirection.
    pub(crate) fn spawn(&mut self) -> EntId {
        let id = EntId(self.host.spawn() as u32);
        self.entities[id].reset();
        self.setup_string_refs(id);
        id
    }

    /// `traceline` — trace from `start` to `end` and read the result out of the engine
    /// globals into a value (so callers don't juggle the shared `trace_*` block).
    pub(crate) fn traceline(
        &mut self,
        start: Vec3,
        end: Vec3,
        nomonsters: bool,
        ignore: EntId,
    ) -> TraceResult {
        // The traceline builtin takes the ignore entity as an edict *index* (it runs
        // `EdictNum(arg)`), unlike entvars `.entity` fields which store byte offsets.
        self.host
            .traceline(start, end, nomonsters, ignore);
        let g = &self.globals;
        TraceResult {
            allsolid: g.trace_allsolid != 0.0,
            startsolid: g.trace_startsolid != 0.0,
            fraction: g.trace_fraction,
            endpos: g.trace_endpos,
            plane_normal: g.trace_plane_normal,
            ent: EntId::from_prog(g.trace_ent),
            in_open: g.trace_inopen != 0.0,
            in_water: g.trace_inwater != 0.0,
        }
    }

    /// `setmodel(self, self.model)` for a brush model (`*N`). The owned `CString` is parked in
    /// the entity so the pointer the engine keeps stays valid for the entity's lifetime.
    pub(crate) fn set_brush_model(&mut self, e: EntId) {
        let Some(m) = self.entities[e].model.clone() else {
            return;
        };
        self.entities[e].model_cs = Some(cstring(&m));
        let host = self.host;
        let ptr = self.entities[e].model_cs.as_deref().unwrap();
        host.set_model_brush(e, ptr);
    }

    /// The entity's `message` string as an owned `CString`, for `centerprint`.
    pub(crate) fn message_cstring(&self, e: EntId) -> Option<CString> {
        self.entities[e].message.as_deref().map(cstring)
    }

    /// Play the entity's `.noise` sound on `chan` at full volume (no-op if unset).
    pub(crate) fn play_noise(&self, e: EntId, chan: defs::Channel) {
        if let Some(noise) = self.entities[e].noise {
            self.host
                .sound(e, chan, noise, 1.0, defs::Attenuation::Norm);
        }
    }

    /// `findradius` — every solid entity whose bounding sphere is within `rad` of `org`.
    /// Implemented directly over our entity array (QuakeC's builtin links via `.chain`;
    /// returning a `Vec` is cleaner and avoids mutating shared state).
    pub(crate) fn find_radius(&self, org: Vec3, rad: f32) -> Vec<EntId> {
        let mut out = Vec::new();
        for (i, e) in self.entities.iter().enumerate() {
            if !e.in_use || e.v.solid == defs::Solid::Not {
                continue;
            }
            let center = e.v.origin + (e.v.mins + e.v.maxs) * 0.5;
            if (org - center).length() <= rad {
                out.push(EntId(i as u32));
            }
        }
        out
    }

    /// `GAME_INIT` — version handshake; returns a pointer to our [`GameData`].
    fn init(&mut self, _level_time: i32, random_seed: i32) -> isize {
        if random_seed != 0 {
            self.rng = (random_seed as u32) | 1;
        }
        let api = self.host.api_version();
        if api < GAME_API_VERSION {
            self.host.error(cstr(b"rtx: server API too old\0"));
            return 0;
        }
        // mvdsv: declare that we use 64-bit string references.
        self.host
            .cvar_set_float(cstr(b"sv_pr2references\0"), 1.0);

        // The engine wipes its extension-trap table on every map load (and the field-reference
        // cookie is regenerated), so the registration and resolved field refs from a previous map
        // are stale. `GAME_INIT` runs per map, so re-resolve them lazily by dropping the cache —
        // matching how ktx re-runs `G_InitExtensions` each `GAME_INIT`.
        self.ext_fields = ext_field::ExtFields::default();

        // Mid-air double jump, on by default (set `rtx_doublejump 0` to disable).
        self.host
            .cvar_set_float(cstr(b"rtx_doublejump\0"), 1.0);

        // Wall jump (kick off a wall you jump into), on by default (`rtx_walljump 0` to disable).
        self.host
            .cvar_set_float(cstr(b"rtx_walljump\0"), 1.0);

        // Elevator jump: a rising lift boosts your jump by `lift_speed * rtx_elevator_jump`.
        // It's a multiplier (0 disables, 1 = add the lift's true speed, 2 = double it, …).
        self.host
            .cvar_set_float(cstr(b"rtx_elevator_jump\0"), 2.0);

        // Shoot live grenades to detonate them early, on by default (`rtx_shootable_grenades 0`
        // to restore classic non-shootable grenades).
        self.host
            .cvar_set_float(cstr(b"rtx_shootable_grenades\0"), 1.0);

        // Grappling hook (purectf port), on by default — every player spawns with it (impulse 22
        // to select). `rtx_grapple 0` to disable.
        self.host.cvar_set_float(cstr(b"rtx_grapple\0"), 1.0);
        // Hook throw / reel-in speed multipliers (purectf's `localinfo hookspeed`/`hookpull`),
        // each scaling its base `× sv_maxspeed`. Defaults match purectf's shipped server.cfg.
        self.host.cvar_set_float(cstr(b"rtx_hook_speed\0"), 1.25);
        self.host.cvar_set_float(cstr(b"rtx_hook_pull\0"), 1.0);

        // Navmesh bots: how many to keep on the server (0 = none), and their skill (reserved
        // for combat tuning later). Bots only spawn once a map's navmesh is built.
        self.host.cvar_set_float(cstr(b"rtx_bots\0"), 0.0);
        self.host.cvar_set_float(cstr(b"rtx_bot_skill\0"), 3.0);

        self.host.dprint(cstr(b"rtx: QuakeWorld game module loaded\0"));

        // `self.game_data` lives inside the OnceLock-pinned GameState, so its address is
        // stable for the process — safe to hand to the engine.
        &self.game_data as *const GameData as isize
    }

    /// `GAME_LOADENTS` — parse the map's entity string and spawn entities.
    ///
    /// The first block configures the `world` entity and runs `worldspawn`; each remaining
    /// block becomes an entity, filtered by deathmatch/skill spawnflags and dispatched to a
    /// spawn function by classname. Parsing (pure) is separated from spawning (mutating).
    fn load_entities(&mut self) -> isize {
        self.level.deathmatch = self.host.cvar(c"deathmatch") as i32;
        self.level.skill = self.host.cvar(c"skill") as i32;
        // Fresh map: drop any prior navmesh so it's rebuilt lazily when bots are next wanted.
        self.nav = crate::navmesh::NavState::default();

        // The worldspawn block configures `world` and runs the global precaches.
        let Some(world_fields) = self.parse_block() else {
            self.host.error(c"SpawnEntities: no entities");
            return 0;
        };
        self.entities[EntId::WORLD].in_use = true;
        self.setup_string_refs(EntId::WORLD);
        self.apply_fields(EntId::WORLD, &world_fields);
        world::worldspawn(self);
        self.host.flush_signon();

        // Parse and spawn each remaining entity one at a time, flushing the signon after each
        // (ktx does the same) so the precache/baseline data for every entity reaches clients.
        while let Some(fields) = self.parse_block() {
            self.spawn_entity(&fields);
            self.host.flush_signon();
        }
        1
    }

    /// Build the bot navmesh for the current map on demand — the first time bots are wanted —
    /// then cache it. Attempted at most once per map (a failed read won't retry every frame).
    /// Best-effort: if the BSP can't be read or parsed the navmesh stays empty and bots simply
    /// don't spawn — never fatal. Deferring this off the load path means a bot-less server
    /// pays neither the build time nor the memory.
    pub(crate) fn ensure_navmesh(&mut self) {
        if self.nav.attempted {
            return;
        }
        self.nav.attempted = true;
        let path = cstring(&format!("maps/{}.bsp", self.level.mapname));
        let Some(bytes) = self.host.read_file(&path) else {
            self.host.dprint(c"rtx: navmesh: could not read map BSP; bots disabled\n");
            return;
        };
        let Some(bsp) = crate::bsp::Bsp::parse(&bytes) else {
            self.host.dprint(c"rtx: navmesh: unsupported/!malformed BSP; bots disabled\n");
            return;
        };
        // Build the cell/link graph from the player clip hull, then splice in the spawned
        // entity-derived links (func_plat lifts, trigger_teleport warps), which need the
        // entities to exist.
        let mut graph = crate::navmesh::NavGraph::build(&bsp);
        graph.add_plats(&bsp, &self.collect_plats());
        graph.add_teleports(&self.collect_teleports());
        graph.add_gates(&self.collect_gates());
        let counts = graph.summary();
        let msg = cstring(&format!(
            "rtx: navmesh: {} planes, {} clipnodes -> {} cells, {} links \
             (walk {} step {} drop {} jump {} plat {} tele {}), {} gates\n",
            bsp.planes.len(),
            bsp.clipnodes.len(),
            graph.cells.len(),
            graph.links.len(),
            counts.walk,
            counts.step,
            counts.drop,
            counts.jump,
            counts.plat,
            counts.teleport,
            graph.gate_count(),
        ));
        self.host.dprint(&msg);
        self.nav.bsp = Some(bsp);
        self.nav.graph = Some(graph);
    }

    /// Gather the [`PlatInfo`](crate::navmesh::PlatInfo) for every spawned `func_plat`: the
    /// player-origin standing spots on the plat surface at the bottom and top of its travel.
    /// The plat moves only in Z (`pos2`→`pos1`); its top surface is `maxs.z` above the origin,
    /// and a standing player origin sits 24 (`-mins.z`) above that surface.
    fn collect_plats(&self) -> Vec<crate::navmesh::PlatInfo> {
        self.find_by_classname("plat")
            .map(|e| {
                let ent = &self.entities[e];
                let (pos1, pos2) = (ent.mover.pos1, ent.mover.pos2);
                let (mins, maxs) = (ent.v.mins, ent.v.maxs);
                let cx = pos1.x + (mins.x + maxs.x) * 0.5;
                let cy = pos1.y + (mins.y + maxs.y) * 0.5;
                crate::navmesh::PlatInfo {
                    board: Vec3::new(cx, cy, pos2.z + maxs.z + 24.0),
                    exit: Vec3::new(cx, cy, pos1.z + maxs.z + 24.0),
                }
            })
            .collect()
    }

    /// Gather the [`TeleportInfo`](crate::navmesh::TeleportInfo) for every `trigger_teleport`:
    /// its world-space trigger box and the destination's arrival origin (resolved through the
    /// `target` → `targetname` link, exactly as `teleport_touch` does at runtime).
    fn collect_teleports(&self) -> Vec<crate::navmesh::TeleportInfo> {
        self.find_by_classname("trigger_teleport")
            .filter_map(|e| {
                let ent = &self.entities[e];
                let target = ent.target.clone()?;
                let dest = self.find_by_targetname(&target).next()?;
                let origin = ent.v.origin;
                Some(crate::navmesh::TeleportInfo {
                    tmin: origin + ent.v.mins,
                    tmax: origin + ent.v.maxs,
                    dest: self.entities[dest].v.origin,
                })
            })
            .collect()
    }

    /// Gather the [`GateInfo`](crate::navmesh::GateInfo) for every button-gated door: a
    /// `func_door` with a targetname (it has no auto-trigger, so it stays shut) paired with the
    /// `func_button` whose `target` fires it. Records the door's closed-position world box (from
    /// `pos1`), the button centre, and whether the button is shot (`health > 0`) or touched.
    fn collect_gates(&self) -> Vec<crate::navmesh::GateInfo> {
        let doors: Vec<EntId> = self
            .find_by_classname("door")
            .filter(|&d| self.entities[d].targetname.is_some())
            .collect();
        let mut gates = Vec::new();
        for door in doors {
            let tn = self.entities[door].targetname.clone().unwrap();
            let Some(button) = self
                .find_by_classname("func_button")
                .find(|&b| self.entities[b].target.as_deref() == Some(tn.as_ref()))
            else {
                continue;
            };
            let bent = &self.entities[button];
            let dent = &self.entities[door];
            let closed = dent.mover.pos1; // door rests closed at pos1
            gates.push(crate::navmesh::GateInfo {
                door: door.0,
                closed_min: closed + dent.v.mins,
                closed_max: closed + dent.v.maxs,
                button: bent.v.origin + (bent.v.mins + bent.v.maxs) * 0.5,
                shoot: bent.v.health > 0.0,
            });
        }
        gates
    }

    /// `GAME_START_FRAME` — once per server frame. `is_bot_frame` runs only bot logic.
    fn start_frame(&mut self, _level_time: i32, is_bot_frame: i32) -> isize {
        if is_bot_frame == 0 {
            world::start_frame(self);
            crate::bot::manage_population(self);
        } else {
            crate::bot::run_bots(self);
        }
        1
    }

    // --- entity-string parsing (reads tokens via the engine, no entity mutation) ---

    /// Parse one `{ "key" "value" ... }` block, or `None` at end of the entity string.
    fn parse_block(&self) -> Option<SpawnFields> {
        let mut buf = [0u8; 1024];
        if self.next_token(&mut buf)? != "{" {
            self.host.error(c"ParseEntity: expected '{'");
            return None;
        }
        let mut fields = SpawnFields::new();
        loop {
            let key = self.next_token(&mut buf)?;
            if key == "}" {
                return Some(fields);
            }
            let value = self.next_token(&mut buf)?;
            fields.push((key, value));
        }
    }

    /// Fetch the next entity-string token as an owned `String`, or `None` at the end.
    fn next_token(&self, buf: &mut [u8]) -> Option<String> {
        let (more, token) = self.host.get_entity_token(buf);
        more.then(|| token.to_owned())
    }

    // --- spawning (mutating) ---

    /// Allocate an engine entity slot, apply its fields, filter it, and dispatch its spawn
    /// function. Unspawnable entities are freed.
    fn spawn_entity(&mut self, fields: &SpawnFields) {
        let id = self.spawn();
        self.apply_fields(id, fields);

        if !self.passes_spawn_filter(id) || !self.call_spawn(id) {
            self.free(id);
        }
    }

    /// Whether an entity survives deathmatch/skill spawnflag filtering.
    fn passes_spawn_filter(&self, id: EntId) -> bool {
        use defs::Bits;
        let flags = self.entities[id].v.spawnflags;
        if self.level.deathmatch != 0 {
            return !flags.has(defs::SpawnFilter::NOT_DEATHMATCH);
        }
        !flags.has(match self.level.skill {
            0 => defs::SpawnFilter::NOT_EASY,
            1 => defs::SpawnFilter::NOT_MEDIUM,
            _ => defs::SpawnFilter::NOT_HARD,
        })
    }

    /// Dispatch a class-specific spawn function. Returns `false` when the entity has no
    /// spawn behaviour (and should be discarded).
    fn call_spawn(&mut self, id: EntId) -> bool {
        // Classname is owned, so clone the few bytes to avoid borrowing `self` across the
        // spawn call.
        let class = match self.entities[id].classname() {
            Some(c) => c.to_owned(),
            None => return false,
        };
        let action = Self::resolve_spawn_class(class.as_str());
        self.execute_spawn_action(id, class.as_str(), action)
    }

    fn execute_spawn_action(&mut self, id: EntId, class: &str, action: SpawnAction) -> bool {
        match action {
            SpawnAction::Keep => true,
            SpawnAction::Drop => false,
            SpawnAction::Spawn(f) => f(self, id),
            SpawnAction::Armor(skin) => self.spawn_item_armor(id, skin),
            SpawnAction::Weapon => self.spawn_weapon_by_classname(id, class),
            SpawnAction::Ammo {
                weapon_code,
                netname,
                small,
                small_amt,
                big,
                big_amt,
            } => self.spawn_ammo(id, weapon_code, netname, small, small_amt, big, big_amt),
            SpawnAction::Powerup {
                model,
                noise,
                netname,
                item_bit,
                effect,
            } => self.spawn_powerup(id, model, noise, netname, item_bit, effect),
            SpawnAction::Flame { model, skin, frame } => {
                self.spawn_flame(id, model, skin, frame)
            }
            SpawnAction::Explobox { model, size } => self.spawn_misc_explobox(id, model, size),
        }
    }

    /// Resolve a map classname into a spawn action. The returned action is executed after the
    /// classname borrow has ended, keeping the dispatch table separate from mutation.
    fn resolve_spawn_class(class: &str) -> SpawnAction {
        match class {
            // Positional markers scanned later (spawn points, teleport/intermission
            // destinations): kept in place with no behaviour of their own.
            "info_player_start" | "info_player_start2" | "info_player_deathmatch"
            | "info_player_coop" | "info_player_team1" | "info_player_team2"
            | "info_player_team3" | "info_player_team4" | "info_intermission"
            | "info_notnull" => SpawnAction::Keep,

            // items.qc
            "item_health" => SpawnAction::Spawn(GameState::spawn_item_health),
            "item_armor1" => SpawnAction::Armor(0.0),
            "item_armor2" => SpawnAction::Armor(1.0),
            "item_armorInv" => SpawnAction::Armor(2.0),
            "weapon_supershotgun" | "weapon_nailgun" | "weapon_supernailgun"
            | "weapon_grenadelauncher" | "weapon_rocketlauncher" | "weapon_lightning" => {
                SpawnAction::Weapon
            }
            "item_shells" => SpawnAction::Ammo {
                weapon_code: 1.0,
                netname: "shells",
                small: Model::MAPS_B_SHELL0,
                small_amt: 20.0,
                big: Model::MAPS_B_SHELL1,
                big_amt: 40.0,
            },
            "item_spikes" => SpawnAction::Ammo {
                weapon_code: 2.0,
                netname: "nails",
                small: Model::MAPS_B_NAIL0,
                small_amt: 25.0,
                big: Model::MAPS_B_NAIL1,
                big_amt: 50.0,
            },
            "item_rockets" => SpawnAction::Ammo {
                weapon_code: 3.0,
                netname: "rockets",
                small: Model::MAPS_B_ROCK0,
                small_amt: 5.0,
                big: Model::MAPS_B_ROCK1,
                big_amt: 10.0,
            },
            "item_cells" => SpawnAction::Ammo {
                weapon_code: 4.0,
                netname: "cells",
                small: Model::MAPS_B_BATT0,
                small_amt: 6.0,
                big: Model::MAPS_B_BATT1,
                big_amt: 12.0,
            },
            "item_artifact_invulnerability" => SpawnAction::Powerup {
                model: Model::PROGS_INVULNER,
                noise: Sound::ITEMS_PROTECT,
                netname: "Pentagram of Protection",
                item_bit: defs::Items::INVULNERABILITY,
                effect: defs::Effects::RED,
            },
            "item_artifact_envirosuit" => SpawnAction::Powerup {
                model: Model::PROGS_SUIT,
                noise: Sound::ITEMS_SUIT,
                netname: "Biosuit",
                item_bit: defs::Items::SUIT,
                effect: defs::Effects::empty(),
            },
            "item_artifact_invisibility" => SpawnAction::Powerup {
                model: Model::PROGS_INVISIBL,
                noise: Sound::ITEMS_INV1,
                netname: "Ring of Shadows",
                item_bit: defs::Items::INVISIBILITY,
                effect: defs::Effects::empty(),
            },
            "item_artifact_super_damage" => SpawnAction::Powerup {
                model: Model::PROGS_QUADDAMA,
                noise: Sound::ITEMS_DAMAGE,
                netname: "Quad Damage",
                item_bit: defs::Items::QUAD,
                effect: defs::Effects::BLUE,
            },

            // triggers.qc
            "trigger_multiple" => SpawnAction::Spawn(GameState::spawn_trigger_multiple),
            "trigger_once" => SpawnAction::Spawn(GameState::spawn_trigger_once),
            "trigger_relay" => SpawnAction::Spawn(GameState::spawn_trigger_relay),
            "trigger_secret" => SpawnAction::Spawn(GameState::spawn_trigger_secret),
            "trigger_counter" => SpawnAction::Spawn(GameState::spawn_trigger_counter),
            "trigger_teleport" => SpawnAction::Spawn(GameState::spawn_trigger_teleport),
            "trigger_hurt" => SpawnAction::Spawn(GameState::spawn_trigger_hurt),
            "trigger_push" => SpawnAction::Spawn(GameState::spawn_trigger_push),
            "trigger_monsterjump" => SpawnAction::Spawn(GameState::spawn_trigger_monsterjump),
            "trigger_changelevel" => SpawnAction::Spawn(GameState::spawn_trigger_changelevel),
            "info_teleport_destination" => {
                SpawnAction::Spawn(GameState::spawn_info_teleport_destination)
            }
            // setskill/onlyregistered are start-map only: drop them.
            "trigger_setskill" => SpawnAction::Drop,

            // buttons.qc
            "func_button" => SpawnAction::Spawn(GameState::spawn_func_button),

            // doors.qc
            "func_door" => SpawnAction::Spawn(GameState::spawn_func_door),
            "func_door_secret" => SpawnAction::Spawn(GameState::spawn_func_door_secret),

            // plats.qc
            "func_plat" => SpawnAction::Spawn(GameState::spawn_func_plat),
            "func_train" => SpawnAction::Spawn(GameState::spawn_func_train),
            // path_corner waypoints are inert markers used by trains.
            "path_corner" => SpawnAction::Keep,

            // rotate.rs — Hipnotic rotating brushes
            "func_rotate_entity" => SpawnAction::Spawn(GameState::spawn_func_rotate_entity),
            "func_rotate_train" => SpawnAction::Spawn(GameState::spawn_func_rotate_train),
            "func_rotate_door" => SpawnAction::Spawn(GameState::spawn_func_rotate_door),
            "func_movewall" => SpawnAction::Spawn(GameState::spawn_func_movewall),
            "rotate_object" => SpawnAction::Spawn(GameState::spawn_rotate_object),
            "path_rotate" => SpawnAction::Spawn(GameState::spawn_path_rotate),
            "info_rotate" => SpawnAction::Spawn(GameState::spawn_info_rotate),
            "func_bob" => SpawnAction::Spawn(GameState::spawn_func_bob),

            // misc.qc
            "info_null" => SpawnAction::Spawn(GameState::spawn_info_null),
            "light" => SpawnAction::Spawn(GameState::spawn_light),
            "light_fluoro" => SpawnAction::Spawn(GameState::spawn_light_fluoro),
            "light_fluorospark" => SpawnAction::Spawn(GameState::spawn_light_fluorospark),
            "light_globe" => SpawnAction::Flame {
                model: Model::PROGS_S_LIGHT,
                skin: 0.0,
                frame: false,
            },
            "light_torch_small_walltorch" => SpawnAction::Flame {
                model: Model::PROGS_FLAME,
                skin: 0.0,
                frame: true,
            },
            "light_flame_large_yellow" => SpawnAction::Flame {
                model: Model::PROGS_FLAME2,
                skin: 1.0,
                frame: true,
            },
            "light_flame_small_yellow" | "light_flame_small_white" => {
                SpawnAction::Flame {
                    model: Model::PROGS_FLAME2,
                    skin: 0.0,
                    frame: true,
                }
            }
            "func_wall" | "func_episodegate" | "func_bossgate" => {
                SpawnAction::Spawn(GameState::spawn_func_wall)
            }
            "func_illusionary" => SpawnAction::Spawn(GameState::spawn_func_illusionary),
            "misc_explobox" => SpawnAction::Explobox {
                model: Model::MAPS_B_EXPLOB,
                size: Vec3::new(32.0, 32.0, 64.0),
            },
            "misc_explobox2" => SpawnAction::Explobox {
                model: Model::MAPS_B_EXBOX2,
                size: Vec3::new(32.0, 32.0, 32.0),
            },

            // Other classes (doors/plats/buttons/misc) get spawn functions below; until then
            // they are discarded, matching QuakeC's "no spawn function" path.
            _ => SpawnAction::Drop,
        }
    }

    /// Apply a block of parsed key/value pairs to an entity.
    fn apply_fields(&mut self, id: EntId, fields: &SpawnFields) {
        for (key, value) in fields {
            self.set_field(id, key, value);
        }
    }

    /// Set a single map field on an entity. Unknown keys are ignored.
    fn set_field(&mut self, id: EntId, key: &str, value: &str) {
        // Map-extension fields (`alpha`, `colormod`) aren't part of our entvars — they live in
        // the engine's parallel block, set through a trap. Handle them before borrowing `ent`.
        match key {
            "alpha" => return self.set_alpha(id, parse_f32(value)),
            "colormod" => return self.set_colormod(id, parse_vec3(value)),
            // Map-declared sound paths (used by path_rotate / func_rotate_train). The typed
            // `Sound` handle implies a precache, so these go through the runtime escape hatch.
            "noise" => return self.set_noise_field(id, 0, value),
            "noise1" => return self.set_noise_field(id, 1, value),
            "noise2" => return self.set_noise_field(id, 2, value),
            "noise3" => return self.set_noise_field(id, 3, value),
            _ => {}
        }
        let ent = &mut self.entities[id];
        match key {
            "classname" => ent.classname = Some(value.into()),
            "model" => ent.model = Some(value.into()),
            "target" => ent.target = Some(value.into()),
            "targetname" => ent.targetname = Some(value.into()),
            "killtarget" => ent.killtarget = Some(value.into()),
            "map" => ent.map = Some(value.into()),
            "message" => ent.message = Some(value.into()),
            "netname" => ent.netname = Some(value.into()),
            "origin" => ent.v.origin = parse_vec3(value),
            "angles" => ent.v.angles = parse_vec3(value),
            "angle" => ent.v.angles = Vec3::new(0.0, parse_f32(value), 0.0), // anglehack
            "spawnflags" => ent.v.spawnflags = parse_f32(value),
            // worldtype only matters on `world`; park it in its (unused) `skin`.
            "worldtype" => ent.v.skin = parse_f32(value),
            "health" => ent.v.health = parse_f32(value),
            "frags" => ent.v.frags = parse_f32(value),
            "team" => ent.v.team = parse_f32(value),
            "items" => ent.v.items = parse_f32(value),
            "sounds" => ent.v.sounds = parse_f32(value),
            // Mover/trigger tuning keys (doors, plats, trains, trigger_push, trigger_hurt, …).
            // Without these, movers fall back to their spawn defaults — e.g. trigger_push's
            // default speed of 1000 launches at 10000 ups (`speed * movedir * 10`).
            "speed" => ent.mover.speed = parse_f32(value),
            "wait" => ent.mover.wait = parse_f32(value),
            "delay" => ent.mover.delay = parse_f32(value),
            "lip" => ent.mover.lip = parse_f32(value),
            "height" => ent.mover.height = parse_f32(value),
            "dmg" => ent.mover.dmg = parse_f32(value),
            "count" => ent.mover.count = parse_f32(value),
            // func_bob easing knobs (speed-up / slow-down factors).
            "waitmin" => ent.bob.waitmin = parse_f32(value),
            "waitmin2" => ent.bob.waitmin2 = parse_f32(value),
            // rotate.rs keys (Hipnotic rotating brushes).
            "rotate" => ent.rot.rotate = parse_vec3(value),
            "path" => ent.path = Some(value.into()),
            "event" => ent.event = Some(value.into()),
            "group" => ent.group = Some(value.into()),
            _ => {}
        }
    }

    /// Store a map-declared sound path in one of the entity's `noise` slots, precaching it
    /// through [`DynAssets`](crate::assets::DynAssets) so the typed [`Sound`] handle still
    /// guarantees a precache. Empty values are ignored.
    fn set_noise_field(&mut self, id: EntId, slot: u8, value: &str) {
        if value.is_empty() {
            return;
        }
        let Ok(path) = CString::new(value) else {
            return;
        };
        let sound = self.dyn_assets.sound(&self.host, &path);
        let ent = &mut self.entities[id];
        match slot {
            0 => ent.noise = Some(sound),
            1 => ent.noise1 = Some(sound),
            2 => ent.noise2 = Some(sound),
            _ => ent.noise3 = Some(sound),
        }
    }

    /// `ExtFieldSetAlpha` (ktx) — set an entity's transparency via the engine's `alpha`
    /// map-extension field: `0` = invisible, `1` = fully opaque. No-op if the server lacks the
    /// extension (older engines). The 0..1 bound is alpha's own rule, so it lives here.
    pub(crate) fn set_alpha(&mut self, id: EntId, alpha: f32) {
        self.ext_fields
            .set::<ext_field::Alpha>(&self.host, id, alpha.clamp(0.0, 1.0));
    }

    /// Set an entity's per-channel RGB colour modulation via the engine's `colormod`
    /// map-extension field (each component a multiplier around `1.0`). No-op if the server
    /// lacks the extension.
    pub(crate) fn set_colormod(&mut self, id: EntId, color: Vec3) {
        self.ext_fields
            .set::<ext_field::ColorMod>(&self.host, id, color.to_array());
    }

    /// Free an entity slot, both on our side and in the engine.
    pub(crate) fn free(&mut self, id: EntId) {
        self.entities[id].in_use = false;
        self.host.remove(id);
    }

    // --- entity access (index handles only; no references escape) ---

    #[inline]
    #[allow(dead_code)]
    pub fn ent(&self, id: EntId) -> &Entity {
        &self.entities[id]
    }

    #[inline]
    #[allow(dead_code)]
    pub fn ent_mut(&mut self, id: EntId) -> &mut Entity {
        &mut self.entities[id]
    }

    #[inline]
    pub fn host(&self) -> &HostApi {
        &self.host
    }

    /// QuakeC `random()` — a float in `[0, 1)` from our xorshift PRNG.
    pub(crate) fn random(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Live entities whose classname matches `name` (the QuakeC `find(... classname ...)`).
    pub(crate) fn find_by_classname<'a>(
        &'a self,
        name: &'a str,
    ) -> impl Iterator<Item = EntId> + 'a {
        self.entities
            .iter()
            .enumerate()
            .filter(move |(_, e)| e.in_use && e.classname() == Some(name))
            .map(|(i, _)| EntId(i as u32))
    }
}

/// Build an owned C string for a host trap, dropping any interior NUL (QuakeC strings
/// never contain one). Used wherever we pass an owned model/message/sound string.
pub(crate) fn cstring(s: &str) -> CString {
    CString::new(s.replace('\0', "")).unwrap_or_default()
}

/// Parse a float from a map value, defaulting to `0.0` on garbage.
fn parse_f32(s: &str) -> f32 {
    s.trim().parse().unwrap_or(0.0)
}

/// Parse a `"x y z"` map value into a [`Vec3`], with missing/garbage components as `0.0`.
fn parse_vec3(s: &str) -> Vec3 {
    let mut parts = s.split_whitespace().map(|p| p.parse().unwrap_or(0.0));
    Vec3::new(
        parts.next().unwrap_or(0.0),
        parts.next().unwrap_or(0.0),
        parts.next().unwrap_or(0.0),
    )
}

/// Build a `&CStr` from a NUL-terminated byte literal at compile-checked call sites.
#[inline]
fn cstr(bytes: &'static [u8]) -> &'static CStr {
    CStr::from_bytes_with_nul(bytes).expect("literal must be NUL-terminated")
}
