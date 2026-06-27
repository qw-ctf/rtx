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

use crate::abi::{Field, GameData, GlobalVars, STRING_REF_COUNT, STRING_REF_WEAPONMODEL};
use crate::defs;
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
}

impl GameState {
    pub fn new(syscall: SyscallFn) -> Self {
        let host = HostApi::new(syscall);
        let mut entities = Entities::new(MAX_EDICTS);
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
    pub(crate) fn set_weaponmodel(&mut self, e: EntId, model: Option<&'static CStr>) {
        let ptr = model.map_or(0, |m| m.as_ptr() as u64);
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
            .traceline(start, end, nomonsters, ignore.0 as i32);
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
        host.set_model(e.0 as i32, ptr);
    }

    /// The entity's `message` string as an owned `CString`, for `centerprint`.
    pub(crate) fn message_cstring(&self, e: EntId) -> Option<CString> {
        self.entities[e].message.as_deref().map(cstring)
    }

    /// Play the entity's `.noise` sound on `chan` at full volume (no-op if unset).
    pub(crate) fn play_noise(&self, e: EntId, chan: defs::Channel) {
        if let Some(noise) = self.entities[e].noise.as_deref() {
            self.host
                .sound(e.0 as i32, chan, &cstring(noise), 1.0, defs::Attenuation::Norm);
        }
    }

    /// `findradius` — every solid entity whose bounding sphere is within `rad` of `org`.
    /// Implemented directly over our entity array (QuakeC's builtin links via `.chain`;
    /// returning a `Vec` is cleaner and avoids mutating shared state).
    pub(crate) fn find_radius(&self, org: Vec3, rad: f32) -> Vec<EntId> {
        use defs::FieldEq;
        let mut out = Vec::new();
        for (i, e) in self.entities.iter().enumerate() {
            if !e.in_use || e.v.solid.is(defs::Solid::Not) {
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

        // Mid-air double jump, on by default (set `rtx_doublejump 0` to disable).
        self.host
            .cvar_set_float(cstr(b"rtx_doublejump\0"), 1.0);

        // Wall jump (kick off a wall you jump into), on by default (`rtx_walljump 0` to disable).
        self.host
            .cvar_set_float(cstr(b"rtx_walljump\0"), 1.0);

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

    /// `GAME_START_FRAME` — once per server frame. `is_bot_frame` runs only bot logic.
    fn start_frame(&mut self, _level_time: i32, is_bot_frame: i32) -> isize {
        if is_bot_frame == 0 {
            world::start_frame(self);
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
        match class.as_str() {
            // Positional markers scanned later (spawn points, teleport/intermission
            // destinations): kept in place with no behaviour of their own.
            "info_player_start" | "info_player_start2" | "info_player_deathmatch"
            | "info_player_coop" | "info_player_team1" | "info_player_team2"
            | "info_player_team3" | "info_player_team4" | "info_intermission"
            | "info_notnull" => true,

            // items.qc
            "item_health" => self.spawn_item_health(id),
            "item_armor1" => self.spawn_item_armor(id, 0.0),
            "item_armor2" => self.spawn_item_armor(id, 1.0),
            "item_armorInv" => self.spawn_item_armor(id, 2.0),
            "weapon_supershotgun" => {
                self.spawn_weapon(id, c"progs/g_shot.mdl", "Double-barrelled Shotgun")
            }
            "weapon_nailgun" => self.spawn_weapon(id, c"progs/g_nail.mdl", "nailgun"),
            "weapon_supernailgun" => self.spawn_weapon(id, c"progs/g_nail2.mdl", "Super Nailgun"),
            "weapon_grenadelauncher" => {
                self.spawn_weapon(id, c"progs/g_rock.mdl", "Grenade Launcher")
            }
            "weapon_rocketlauncher" => {
                self.spawn_weapon(id, c"progs/g_rock2.mdl", "Rocket Launcher")
            }
            "weapon_lightning" => self.spawn_weapon(id, c"progs/g_light.mdl", "Thunderbolt"),
            "item_shells" => self.spawn_ammo(
                id, 1.0, "shells", c"maps/b_shell0.bsp", 20.0, c"maps/b_shell1.bsp", 40.0,
            ),
            "item_spikes" => self.spawn_ammo(
                id, 2.0, "nails", c"maps/b_nail0.bsp", 25.0, c"maps/b_nail1.bsp", 50.0,
            ),
            "item_rockets" => self.spawn_ammo(
                id, 3.0, "rockets", c"maps/b_rock0.bsp", 5.0, c"maps/b_rock1.bsp", 10.0,
            ),
            "item_cells" => self.spawn_ammo(
                id, 4.0, "cells", c"maps/b_batt0.bsp", 6.0, c"maps/b_batt1.bsp", 12.0,
            ),
            "item_artifact_invulnerability" => self.spawn_powerup(
                id, c"progs/invulner.mdl", c"items/protect.wav", "Pentagram of Protection",
                defs::Items::INVULNERABILITY, defs::Effects::RED,
            ),
            "item_artifact_envirosuit" => self.spawn_powerup(
                id, c"progs/suit.mdl", c"items/suit.wav", "Biosuit", defs::Items::SUIT, defs::Effects::empty(),
            ),
            "item_artifact_invisibility" => self.spawn_powerup(
                id, c"progs/invisibl.mdl", c"items/inv1.wav", "Ring of Shadows",
                defs::Items::INVISIBILITY, defs::Effects::empty(),
            ),
            "item_artifact_super_damage" => self.spawn_powerup(
                id, c"progs/quaddama.mdl", c"items/damage.wav", "Quad Damage", defs::Items::QUAD,
                defs::Effects::BLUE,
            ),

            // triggers.qc
            "trigger_multiple" => self.spawn_trigger_multiple(id),
            "trigger_once" => self.spawn_trigger_once(id),
            "trigger_relay" => self.spawn_trigger_relay(id),
            "trigger_secret" => self.spawn_trigger_secret(id),
            "trigger_counter" => self.spawn_trigger_counter(id),
            "trigger_teleport" => self.spawn_trigger_teleport(id),
            "trigger_hurt" => self.spawn_trigger_hurt(id),
            "trigger_push" => self.spawn_trigger_push(id),
            "trigger_monsterjump" => self.spawn_trigger_monsterjump(id),
            "trigger_changelevel" => self.spawn_trigger_changelevel(id),
            "info_teleport_destination" => self.spawn_info_teleport_destination(id),
            // setskill/onlyregistered are start-map only: drop them.
            "trigger_setskill" => false,

            // buttons.qc
            "func_button" => self.spawn_func_button(id),

            // doors.qc
            "func_door" => self.spawn_func_door(id),
            "func_door_secret" => self.spawn_func_door_secret(id),

            // plats.qc
            "func_plat" => self.spawn_func_plat(id),
            "func_train" => self.spawn_func_train(id),
            // path_corner waypoints are inert markers used by trains.
            "path_corner" => true,

            // misc.qc
            "info_null" => self.spawn_info_null(id),
            "light" => self.spawn_light(id),
            "light_fluoro" => self.spawn_light_fluoro(id),
            "light_fluorospark" => self.spawn_light_fluorospark(id),
            "light_globe" => self.spawn_flame(id, c"progs/s_light.spr", 0.0, false),
            "light_torch_small_walltorch" => self.spawn_flame(id, c"progs/flame.mdl", 0.0, true),
            "light_flame_large_yellow" => self.spawn_flame(id, c"progs/flame2.mdl", 1.0, true),
            "light_flame_small_yellow" | "light_flame_small_white" => {
                self.spawn_flame(id, c"progs/flame2.mdl", 0.0, true)
            }
            "func_wall" | "func_episodegate" | "func_bossgate" => self.spawn_func_wall(id),
            "func_illusionary" => self.spawn_func_illusionary(id),
            "misc_explobox" => {
                self.spawn_misc_explobox(id, c"maps/b_explob.bsp", Vec3::new(32.0, 32.0, 64.0))
            }
            "misc_explobox2" => {
                self.spawn_misc_explobox(id, c"maps/b_exbox2.bsp", Vec3::new(32.0, 32.0, 32.0))
            }

            // Other classes (doors/plats/buttons/misc) get spawn functions below; until then
            // they are discarded, matching QuakeC's "no spawn function" path.
            _ => false,
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
            _ => {}
        }
    }

    /// Free an entity slot, both on our side and in the engine.
    pub(crate) fn free(&mut self, id: EntId) {
        self.entities[id].in_use = false;
        self.host.remove(id.0 as i32);
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
