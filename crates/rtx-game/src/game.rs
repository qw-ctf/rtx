// SPDX-License-Identifier: AGPL-3.0-or-later

//! The single owner of all game state.
//!
//! `GameState` lives behind the one global (`OnceLock<Game>` in `lib.rs`) and owns the
//! entity array, the engine-shared globals, the field table, the returned [`GameData`],
//! and the host handle. The engine receives raw pointers into the heap `Box`es here at
//! `GAME_INIT` and keeps them for the process lifetime — those buffers are never
//! reallocated, so the pointers stay valid.

use std::ffi::CString;
use std::sync::atomic;

use glam::Vec3;

use crate::abi::{
    Field, FieldType, GameData, GlobalVars, STRING_REF_COUNT, STRING_REF_NETNAME, STRING_REF_WEAPONMODEL,
};
use crate::assets::{DynAssets, Model};
use crate::entity::{EntId, Entities, Entity};
use crate::game_command::GameCommand;
use crate::host::{HostApi, SyscallFn};
use crate::mode::{self, ArenaState, GameMode};
use crate::world;
use crate::{bot, defs, ext_field, navmesh, race};

/// Matches `MAX_EDICTS` in `ktx/include/q_shared.h`.
pub const MAX_EDICTS: usize = 2048;
/// Matches `GAME_API_VERSION` in `ktx/include/g_public.h`.
pub const GAME_API_VERSION: i32 = 16;

/// A single entity's parsed key/value pairs from the map's entity string.
pub(crate) type SpawnFields = Vec<(String, String)>;
pub(crate) type SpawnFn = fn(&mut GameState, EntId) -> bool;

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
    /// Custom-field table handed to the engine (the extended `maxspeed` field, plus the
    /// terminator). Owned for lifetime; accessed by the engine through `game_data.fields`.
    #[allow(dead_code)]
    fields: Box<[Field]>,
    /// The handshake reply; its `ents`/`global`/`fields` point into the boxes above.
    game_data: GameData,
    /// Match-wide state.
    pub level: Level,
    /// The active game mode (`rtx_mode`) — a stateless behavior descriptor. Reselected each map
    /// load in `worldspawn`; defaults to deathmatch. See [`crate::mode`].
    pub mode: &'static dyn GameMode,
    /// The raw `rtx_mode` / `rtx_match` cvar strings as last read, so `refresh_mode` can detect a
    /// *raw* change (and fire its one-shot console hints) even when the value resolves to the same
    /// descriptor/config — e.g. an unknown mode that falls back to `dm`. See [`GameState::refresh_mode`].
    pub mode_cvar: String,
    pub match_cvar: String,
    /// Rocket-Arena round-state machine. Only meaningful while `mode` is the arena; otherwise
    /// left at its default. See [`crate::mode::ArenaState`].
    pub arena: ArenaState,
    /// Match-composition state (resolved `rtx_match` config + lifecycle + locked roster). Meaningful
    /// while a team composition is active (`teams >= 2`). Lives here (not on the mode descriptor) so
    /// it survives the match-start map reload. See [`crate::mode::MatchState`].
    pub team_match: crate::mode::MatchState,
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
    pub(crate) nav: navmesh::NavState,
    /// The map's KTX race routes (from `race/routes/*.route` and/or embedded
    /// `race_route_*` entities), loaded at the end of `load_entities`. See [`crate::race`].
    pub(crate) race: race::RaceState,
    /// Shared, observation-gated opponent hypotheses (per-side strength/arsenal estimates). Reset to
    /// the mode's spawn kit each map load in `mode::on_worldspawn`. See [`crate::bot::model`].
    pub(crate) opponents: bot::model::OpponentModel,
    /// Set by each engine **bot frame** (`GAME_START_FRAME` with `is_bot_frame`) and cleared by the
    /// next normal frame. The engine only issues bot frames once a real client is connected, so on an
    /// empty (bots-only) server this stays clear and the normal frame drives the bots itself. See
    /// `start_frame`. Frame-based, not time-based, so it doesn't depend on the clock advancing.
    bot_frame_seen: bool,
    /// Whether the current map has finished `GAME_LOADENTS` (worldspawn precaches + entity spawn).
    /// Cleared at `GAME_INIT`, set at the end of `load_entities`. `start_frame` no-ops until it's set
    /// so nothing spawns a player/bot — whose `PutClientInServer` does `setmodel("progs/eyes.mdl")` —
    /// before worldspawn has precached that model, which the engine rejects fatally ("no precache").
    map_spawned: bool,
    /// One-shot: whether we've logged that the normal frame is driving the bots (diagnostic).
    normal_bot_drive_logged: bool,
    /// A population change the bot manager wants applied this frame (add or remove one bot),
    /// deferred out of the frame. `add_bot`/`remove_bot` make the engine run our
    /// `ClientConnect`/`PutClientInServer`/`ClientDisconnect` *synchronously and re-entrantly*; if
    /// we called them while a `&mut GameState` borrow is live (as `manage_population` does), the
    /// re-entered `vmMain` would create a second, aliasing `&mut GameState` — undefined behavior.
    /// `vmMain` drains this after the frame's borrow is released. See [`bot::drain_roster`].
    pub(crate) pending_roster: Option<bot::RosterOp>,
    /// Escape hatch for string-declared sounds (precache-and-intern at load time). Empty for the
    /// current port — every sound is a registry handle — but here so a runtime path is registered
    /// through the same precache-guaranteeing door. Use `dyn_assets.sound(&host, path)`.
    #[allow(dead_code)]
    pub(crate) dyn_assets: DynAssets,
}

impl GameState {
    pub fn new(syscall: SyscallFn) -> Self {
        let mut entities = Entities::new(MAX_EDICTS);
        let host = HostApi::new(syscall, entities.as_mut_ptr());
        let mut globals = Box::new(GlobalVars::default());
        // Declare the extended `maxspeed` field so the engine can seed each client's move-speed
        // cap from it (required for bots to walk under mvdsv, which never initializes a bot's cap
        // otherwise; set on spawn in `put_client_in_server`). `ofs` is the byte offset from the
        // entity base.
        let fields: Box<[Field]> = Box::new([
            Field::new(
                c"maxspeed",
                core::mem::offset_of!(Entity, maxspeed) as i32,
                FieldType::Float,
            ),
            Field::TERMINATOR,
        ]);

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
            mode: mode::default_mode(),
            mode_cvar: String::new(),
            match_cvar: String::new(),
            arena: ArenaState::default(),
            team_match: crate::mode::MatchState::default(),
            activator: EntId::WORLD,
            damage_attacker: EntId::WORLD,
            damage_inflictor: EntId::WORLD,
            intermission_running: false,
            intermission_exit_time: 0.0,
            rng: 0x2545_f491, // nonzero default; reseeded in GAME_INIT
            ext_fields: ext_field::ExtFields::default(),
            nav: navmesh::NavState::default(),
            race: race::RaceState::default(),
            opponents: bot::model::OpponentModel::default(),
            bot_frame_seen: false,
            map_spawned: false,
            normal_bot_drive_logged: false,
            pending_roster: None,
            dyn_assets: DynAssets::default(),
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
                // The engine drives this for bots on a bot frame whenever the server isn't empty (a
                // human is watching), bypassing the round former's deferral. Honor the mode's spawn
                // gate for bots so it can't drop a bot onto an occupied arena spot the pre-round
                // telefrag can't clear — leave it never-spawned and let `run_bot` / the arena's
                // Forming retry place it once the area frees. Humans can't be deferred (no retry
                // channel), so they always spawn here.
                let mode = self.mode;
                if self.entities[player].bot.is_bot && !mode.spawn_area_clear(self, player) {
                    return 1;
                }
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
                // Zero the slot ourselves — the engine only memsets an edict on `ED_Alloc`, NOT on
                // the map-load `PR2_ClearEdict` sweep that also drives us here. Our entity box is
                // process-global (adopted verbatim as `sv.game_edicts`) and outlives the map, so
                // without this a slot the new map hasn't re-allocated keeps the previous map's
                // `Entity` at an index `>= sv.num_edicts`; passing it to a `NUM_FOR_EDICT`-routed
                // builtin (sound, unicast) then aborts with "NUM_FOR_EDICT: bad pointer". `default()`
                // (not `reset()`) leaves `in_use == false`, so the cleared slot is inert to scans.
                // Our own `spawn()` suppresses this callback and resets the slot itself, so the only
                // callers reaching here are the map-load sweep and client (re)spawns — exactly where
                // a fresh slot is wanted. Then re-establish the native string-field indirection the
                // clear just wiped (mirrors ktx's `initialise_spawned_ent`).
                self.entities[player] = Entity::default();
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
        let base = id.index() * core::mem::size_of::<Entity>() + core::mem::offset_of!(Entity, string_refs);

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
    pub(crate) fn set_weaponmodel(&mut self, e: EntId, model: Option<Model>) {
        let ptr = model.map_or(0, |m| m.path().as_ptr() as u64);
        self.entities[e].string_refs[STRING_REF_WEAPONMODEL] = ptr;
    }

    /// Set the engine-visible `netname` string. The engine (FTEQW) syncs a connected client's
    /// name *from* `v.netname` every frame, so a client whose edict was cleared with an empty
    /// netname — notably a bot — gets renamed to an empty string (and vanishes from the
    /// scoreboard) unless we write it here. Backed by a persistent `CString` on the entity so the
    /// raw pointer the engine keeps stays valid for the client's lifetime.
    pub(crate) fn set_netname(&mut self, e: EntId, name: &str) {
        let ent = &mut self.entities[e];
        ent.netname_cs = Some(cstring(name));
        let ptr = ent.netname_cs.as_deref().map_or(0, |c| c.as_ptr() as u64);
        ent.string_refs[STRING_REF_NETNAME] = ptr;
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
        // `host.spawn()` -> the engine's `ED_Alloc` -> `ED_ClearEdict` re-enters this module with
        // `GAME_CLEAR_EDICT` synchronously, while this `&mut self` is live — which would alias the
        // `&mut GameState` the re-entered `vmMain` derives (UB). We re-establish the edict's string
        // refs ourselves just below, so that callback is redundant here: suppress it across the trap
        // so `vmMain` skips it before taking a borrow. Save/restore rather than clear-to-false so a
        // nested `spawn()` (e.g. from a re-entrant `PutClientInServer`) stays suppressed too.
        let prev = crate::SUPPRESS_CLEAR_EDICT.swap(true, atomic::Ordering::Relaxed);
        let raw = self.host.spawn();
        crate::SUPPRESS_CLEAR_EDICT.store(prev, atomic::Ordering::Relaxed);
        let id = EntId(raw as u32);
        self.entities[id].reset();
        self.setup_string_refs(id);
        id
    }

    /// `traceline` — trace from `start` to `end` and read the result out of the engine
    /// globals into a value (so callers don't juggle the shared `trace_*` block).
    pub(crate) fn traceline(&mut self, start: Vec3, end: Vec3, nomonsters: bool, ignore: EntId) -> TraceResult {
        // The traceline builtin takes the ignore entity as an edict *index* (it runs
        // `EdictNum(arg)`), unlike entvars `.entity` fields which store byte offsets.
        self.host.traceline(start, end, nomonsters, ignore);
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
        let host = self.host;
        let ptr = self.entities[e].model_cs.insert(cstring(&m));
        host.set_model_brush(e, ptr);
    }

    /// The entity's `message` string as an owned `CString`, for `centerprint`.
    pub(crate) fn message_cstring(&self, e: EntId) -> Option<CString> {
        self.entities[e].message.as_deref().map(cstring)
    }

    /// Play the entity's `.noise` sound on `chan` at full volume (no-op if unset).
    pub(crate) fn play_noise(&self, e: EntId, chan: defs::Channel) {
        if let Some(noise) = self.entities[e].noise {
            self.host.sound(e, chan, noise, 1.0, defs::Attenuation::Norm);
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
            self.host.error(c"rtx: server API too old");
            return 0;
        }
        // mvdsv: declare that we use 64-bit string references.
        self.host.cvar_set_float(c"sv_pr2references", 1.0);

        // The engine wipes its extension-trap table on every map load (and the field-reference
        // cookie is regenerated), so the registration and resolved field refs from a previous map
        // are stale. `GAME_INIT` runs per map, so re-resolve them lazily by dropping the cache —
        // matching how ktx re-runs `G_InitExtensions` each `GAME_INIT`.
        self.ext_fields = ext_field::ExtFields::default();

        // Not spawned until `load_entities` (worldspawn) has re-issued this map's precaches. Guards
        // `start_frame` from spawning any player/bot — whose `setmodel("progs/eyes.mdl")` would hit
        // the engine's fatal "no precache" — before the model is precached on the new map.
        self.map_spawned = false;

        // Retire the previous map's bots. The map change that precedes this `GAME_INIT` runs the
        // engine's `SV_SpawnServer`, which *frees every bot client* (clearing its `isBot`) but — by
        // design — does **not** run our `ClientDisconnect` for them. Our `GameState` is process-global
        // and outlives the map, so those slots would otherwise stay `is_bot == true` here with no bot
        // behind them: `run_bots` would drive freed slots (the engine rejects each command — "tried
        // to set cmd a non-botclient N") and `manage_population`, counting the phantoms as present,
        // would never re-add real bots. Clear them so the roster rebuilds fresh on the new map.
        // Human clients are never `is_bot`, so they're untouched — the engine keeps them across the
        // change and re-runs their spawn.
        let maxclients = self.host.cvar(c"maxclients").max(0.0) as u32;
        for id in (1..=maxclients).map(EntId) {
            if self.entities[id].bot.is_bot {
                self.retire_slot(id);
            }
        }

        // Seed the rtx tunables from the registry (see `crate::cvars` for each cvar's meaning).
        for &(name, seed) in crate::cvars::RTX_CVAR_DEFAULTS {
            self.host.cvar_default(name, seed);
        }

        // conprint (not dprint) so it shows without `developer 1` — lets you confirm at a glance
        // that the freshly built module is the one actually loaded.
        self.host
            .conprint(c"rtx: QuakeWorld game module loaded (bot-goals build)\n");

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
        // Fresh map: drop any prior navmesh so it's rebuilt lazily when bots are next wanted,
        // and the previous map's race routes with it.
        self.nav = navmesh::NavState::default();
        self.race = race::RaceState::default();

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
        // Fold any race-route data (route file and/or embedded marker entities) into
        // `self.race` — mirrors ktx loading routes at the end of G_SpawnEntitiesFromString.
        self.load_race_routes();
        // Precaches issued and entities spawned — frames may now spawn players/bots safely.
        self.map_spawned = true;
        1
    }

    /// `GAME_START_FRAME` — once per server frame. `is_bot_frame` runs only bot logic.
    fn start_frame(&mut self, _level_time: i32, is_bot_frame: i32) -> isize {
        // A frame before `load_entities` has run (worldspawn precaches) must not spawn anything — the
        // arena round machine and bot population both reach `PutClientInServer` → `setmodel(eyes)`,
        // which the engine rejects fatally until eyes is precached on this map. Wait for spawn.
        if !self.map_spawned {
            return 1;
        }
        if is_bot_frame == 0 {
            world::start_frame(self);
            // Auto-advance a configured map rotation past the intermission scoreboard.
            self.map_queue_frame();
            // Drive the active mode's per-frame state machine (Rocket Arena's round countdown/fight/
            // reset), then the composition layer's match lifecycle (a no-op unless a team match is
            // active — team DM / CTF share it, keyed off `rtx_match`, not the mode).
            let mode = self.mode;
            mode.tick(self);
            crate::mode::team::tick_lifecycle(self);
            bot::manage_population(self);
            // The engine only issues bot frames (below) once a real client is connected. On an empty
            // bots-only server those never arrive, so if no bot frame ran since the last normal frame,
            // drive the bots from this normal frame instead — otherwise they'd spawn and just stand
            // there. (When a real client is present the bot frame drives them and this stays idle.)
            if !self.bot_frame_seen {
                if !self.normal_bot_drive_logged && self.nav.is_loaded() {
                    self.host
                        .dprint(c"rtx: no engine bot frame — driving bots from the normal frame\n");
                    self.normal_bot_drive_logged = true;
                }
                bot::run_bots(self);
            }
            self.bot_frame_seen = false;
        } else {
            self.bot_frame_seen = true;
            bot::run_bots(self);
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

    /// The mask of weapons enabled by `rtx_weapons`. Read live; a weapon absent from it has no map
    /// pickups (dropped at map load) and is stripped from every spawn kit (so it can't be fired).
    pub(crate) fn enabled_weapon_mask(&self) -> defs::Items {
        let mut buf = [0u8; 128];
        let list = self.host.cvar_string(c"rtx_weapons", &mut buf);
        crate::arsenal::enabled_weapons(list)
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

    /// Live entities satisfying `pred` — the shared body of the `find_by_*` selectors (the QuakeC
    /// `find(...)` scan). `pred` sees only entities already known to be `in_use`.
    pub(crate) fn find_where<'a>(&'a self, pred: impl Fn(&Entity) -> bool + 'a) -> impl Iterator<Item = EntId> + 'a {
        self.entities
            .iter()
            .enumerate()
            .filter(move |(_, e)| e.in_use && pred(e))
            .map(|(i, _)| EntId(i as u32))
    }

    /// Live entities whose classname matches `name` (the QuakeC `find(... classname ...)`).
    pub(crate) fn find_by_classname<'a>(&'a self, name: &'a str) -> impl Iterator<Item = EntId> + 'a {
        self.find_where(move |e| e.classname() == Some(name))
    }
}

/// Build an owned C string for a host trap, dropping any interior NUL (QuakeC strings
/// never contain one). Used wherever we pass an owned model/message/sound string.
pub(crate) fn cstring(s: &str) -> CString {
    CString::new(s.replace('\0', "")).unwrap_or_default()
}

