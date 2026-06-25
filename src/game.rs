//! The single owner of all game state.
//!
//! `GameState` lives behind the one global (`OnceLock<Game>` in `lib.rs`) and owns the
//! entity array, the engine-shared globals, the field table, the returned [`GameData`],
//! and the host handle. The engine receives raw pointers into the heap `Box`es here at
//! `GAME_INIT` and keeps them for the process lifetime — those buffers are never
//! reallocated, so the pointers stay valid.

use core::ffi::CStr;

use glam::Vec3;

use crate::abi::{Field, GameData, GlobalVars};
use crate::defs;
use crate::entity::{EntId, Entity};
use crate::game_command::GameCommand;
use crate::host::{HostApi, SyscallFn};
use crate::world;

/// Matches `MAX_EDICTS` in `ktx/include/q_shared.h`.
pub const MAX_EDICTS: usize = 2048;
/// Matches `GAME_API_VERSION` in `ktx/include/g_public.h`.
pub const GAME_API_VERSION: i32 = 16;

/// A single entity's parsed key/value pairs from the map's entity string.
type SpawnFields = Vec<(String, String)>;

/// Match/level-wide state refreshed each frame from cvars (qw-qc globals).
#[derive(Default)]
pub struct Level {
    pub framecount: i32,
    pub deathmatch: i32,
    pub teamplay: i32,
    pub timelimit: i32,
    pub fraglimit: i32,
    pub skill: i32,
}

pub struct GameState {
    pub(crate) host: HostApi,
    /// The shared entity array. Heap-allocated once, never resized.
    pub(crate) entities: Box<[Entity]>,
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
    /// PRNG state (QuakeC `random()` is a VM builtin with no native syscall, so we roll
    /// our own, seeded from `GAME_INIT`'s random seed).
    rng: u32,
}

impl GameState {
    pub fn new(syscall: SyscallFn) -> Self {
        let host = HostApi::new(syscall);
        let mut entities: Box<[Entity]> =
            (0..MAX_EDICTS).map(|_| Entity::default()).collect();
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
            rng: 0x2545_f491, // nonzero default; reseeded in GAME_INIT
        }
    }

    /// Handle one engine command. Returns the engine-expected `intptr_t`. Unknown raw
    /// command ids are filtered out before this is called (see `vmMain`).
    pub fn dispatch(&mut self, cmd: GameCommand, arg0: i32, arg1: i32, _arg2: i32) -> isize {
        let player = self.self_ent();
        let is_spectator = arg0 != 0;

        // TEMP M2 diagnostics: trace the client lifecycle on the server console.
        if matches!(
            cmd,
            GameCommand::ClientConnect
                | GameCommand::PutClientInServer
                | GameCommand::ClientDisconnect
                | GameCommand::SetNewParams
                | GameCommand::SetChangeParams
        ) {
            self.dlog(&format!(
                "[rtx] {cmd:?} arg0={arg0} self_off={} self_idx={} sizeofent={}",
                self.globals.self_,
                player.index(),
                core::mem::size_of::<Entity>(),
            ));
        }
        if matches!(cmd, GameCommand::ClientConnect) {
            let e = player.0 as i32;
            let (mut b1, mut b2, mut b3, mut b4) = ([0u8; 64], [0u8; 64], [0u8; 64], [0u8; 64]);
            let name = self.host.infokey(e, c"name", &mut b1).to_owned();
            let spec = self.host.infokey(e, c"spectator", &mut b2).to_owned();
            let sspec = self.host.infokey(e, c"*spectator", &mut b3).to_owned();
            let zext = self.host.infokey(e, c"*z_ext", &mut b4).to_owned();
            // Z_EXT_JOIN_OBSERVE = 1<<5 = 32.
            self.dlog(&format!(
                "[rtx] connect userinfo: name='{name}' spectator='{spec}' *spectator='{sspec}' *z_ext='{zext}' (join/observe bit32)"
            ));
        }

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
            // Spectator paths and remaining callbacks land in later milestones.
            GameCommand::ClientConnect
            | GameCommand::PutClientInServer
            | GameCommand::ClientDisconnect
            | GameCommand::ClientPreThink
            | GameCommand::ClientThink
            | GameCommand::ClientPostThink
            | GameCommand::EdictTouch
            | GameCommand::EdictThink
            | GameCommand::EdictBlocked
            | GameCommand::ClientSay => 1,
            GameCommand::ClientUserInfoChanged
            | GameCommand::ClientCommand
            | GameCommand::ConsoleCommand
            | GameCommand::PausedTic
            | GameCommand::ClearEdict => 0,
        }
    }

    /// The entity the engine has made "current" (`self`). `globals.self_` is a *byte
    /// offset* into the entity array (QuakeC `PROG_TO_EDICT` convention), so divide by the
    /// per-entity stride to recover the index.
    fn self_ent(&self) -> EntId {
        EntId((self.globals.self_ as usize / core::mem::size_of::<Entity>()) as u32)
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

        let mut blocks = self.read_entity_blocks().into_iter();

        let Some(world_fields) = blocks.next() else {
            self.host.error(c"SpawnEntities: no entities");
            return 0;
        };
        self.entities[EntId::WORLD.index()].in_use = true;
        self.apply_fields(EntId::WORLD, &world_fields);
        world::worldspawn(self);

        for fields in blocks {
            self.spawn_entity(&fields);
        }

        // TEMP M2 diagnostics.
        let live = self.entities.iter().filter(|e| e.in_use).count();
        let dm_spots = self.find_by_classname("info_player_deathmatch").count();
        let starts = self.find_by_classname("info_player_start").count();
        self.dlog(&format!(
            "[rtx] load_entities: live={live} dm_spawns={dm_spots} starts={starts} dm_cvar={}",
            self.level.deathmatch,
        ));
        1
    }

    /// `GAME_START_FRAME` — once per server frame. `is_bot_frame` runs only bot logic.
    fn start_frame(&mut self, _level_time: i32, is_bot_frame: i32) -> isize {
        if is_bot_frame == 0 {
            world::start_frame(self);
        }
        1
    }

    // --- entity-string parsing (pure: reads tokens, no mutation) ---

    /// Read every `{ ... }` block from the engine's entity string.
    fn read_entity_blocks(&self) -> Vec<SpawnFields> {
        let mut blocks = Vec::new();
        while let Some(block) = self.parse_block() {
            blocks.push(block);
        }
        blocks
    }

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
        let id = EntId(self.host.spawn() as u32);
        self.entities[id.index()].reset();
        self.apply_fields(id, fields);

        if !self.passes_spawn_filter(id) || !self.call_spawn(id) {
            self.free(id);
        }
    }

    /// Whether an entity survives deathmatch/skill spawnflag filtering.
    fn passes_spawn_filter(&self, id: EntId) -> bool {
        let flags = self.entities[id.index()].v.spawnflags as i32;
        if self.level.deathmatch != 0 {
            return flags & defs::SPAWNFLAG_NOT_DEATHMATCH == 0;
        }
        let blocked = match self.level.skill {
            0 => flags & defs::SPAWNFLAG_NOT_EASY,
            1 => flags & defs::SPAWNFLAG_NOT_MEDIUM,
            _ => flags & defs::SPAWNFLAG_NOT_HARD,
        };
        blocked == 0
    }

    /// Dispatch a class-specific spawn function. Returns `false` when the entity has no
    /// spawn behaviour (and should be discarded).
    fn call_spawn(&mut self, id: EntId) -> bool {
        match self.entities[id.index()].classname() {
            // Positional markers scanned later (spawn points, teleport/intermission
            // destinations): kept in place with no behaviour of their own.
            Some(
                "info_player_start" | "info_player_start2" | "info_player_deathmatch"
                | "info_player_coop" | "info_player_team1" | "info_player_team2"
                | "info_player_team3" | "info_player_team4" | "info_intermission"
                | "info_teleport_destination" | "info_notnull",
            ) => true,
            // Other classes get spawn functions in later milestones; until then they are
            // discarded, matching QuakeC's "no spawn function" path.
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
        let ent = &mut self.entities[id.index()];
        match key {
            "classname" => ent.classname = Some(value.into()),
            "model" => ent.model = Some(value.into()),
            "target" => ent.target = Some(value.into()),
            "targetname" => ent.targetname = Some(value.into()),
            "killtarget" => ent.killtarget = Some(value.into()),
            "message" => ent.message = Some(value.into()),
            "netname" => ent.netname = Some(value.into()),
            "origin" => ent.v.origin = parse_vec3(value),
            "angles" => ent.v.angles = parse_vec3(value),
            "angle" => ent.v.angles = Vec3::new(0.0, parse_f32(value), 0.0), // anglehack
            "spawnflags" => ent.v.spawnflags = parse_f32(value),
            "health" => ent.v.health = parse_f32(value),
            "frags" => ent.v.frags = parse_f32(value),
            "team" => ent.v.team = parse_f32(value),
            "items" => ent.v.items = parse_f32(value),
            "sounds" => ent.v.sounds = parse_f32(value),
            _ => {}
        }
    }

    /// Free an entity slot, both on our side and in the engine.
    fn free(&mut self, id: EntId) {
        self.entities[id.index()].in_use = false;
        self.host.remove(id.0 as i32);
    }

    // --- entity access (index handles only; no references escape) ---

    #[inline]
    #[allow(dead_code)]
    pub fn ent(&self, id: EntId) -> &Entity {
        &self.entities[id.index()]
    }

    #[inline]
    #[allow(dead_code)]
    pub fn ent_mut(&mut self, id: EntId) -> &mut Entity {
        &mut self.entities[id.index()]
    }

    #[inline]
    pub fn host(&self) -> &HostApi {
        &self.host
    }

    /// Print a formatted line to the server console (`G_conprint`, unconditional —
    /// unlike `dprint`, which is developer-gated).
    pub(crate) fn dlog(&self, msg: &str) {
        if let Ok(c) = std::ffi::CString::new(format!("{msg}\n")) {
            self.host.conprint(&c);
        }
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
