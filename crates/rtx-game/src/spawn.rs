// SPDX-License-Identifier: AGPL-3.0-or-later

//! The map entity-string spawn system: the classname->action dispatch table
//! ([`GameState::resolve_spawn_class`]) and the field application that turns a parsed
//! `{ key value }` block into a live entity. Split out of `game.rs`, which keeps the entity-string
//! *reader* (`load_entities`) and drives it through [`GameState::spawn_entity`] here.

use std::ffi::CString;

use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::defs;
use crate::entity::EntId;
use crate::game::{GameState, SpawnFields, SpawnFn};

/// A parsed map entity's spawn action, resolved from its classname before any mutation.
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

impl GameState {
    // --- spawning (mutating) ---

    /// Allocate an engine entity slot, apply its fields, filter it, and dispatch its spawn
    /// function. Unspawnable entities are freed.
    pub(crate) fn spawn_entity(&mut self, fields: &SpawnFields) {
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
            SpawnAction::Flame { model, skin, frame } => self.spawn_flame(id, model, skin, frame),
            SpawnAction::Explobox { model, size } => self.spawn_misc_explobox(id, model, size),
        }
    }

    /// Resolve a map classname into a spawn action. The returned action is executed after the
    /// classname borrow has ended, keeping the dispatch table separate from mutation.
    fn resolve_spawn_class(class: &str) -> SpawnAction {
        match class {
            // Positional markers scanned later (spawn points, teleport/intermission
            // destinations): kept in place with no behaviour of their own.
            "info_player_start"
            | "info_player_start2"
            | "info_player_deathmatch"
            | "info_player_coop"
            | "info_player_team1"
            | "info_player_team2"
            | "info_player_team3"
            | "info_player_team4"
            | "info_intermission"
            | "info_notnull" => SpawnAction::Keep,

            // items.qc
            "item_health" => SpawnAction::Spawn(GameState::spawn_item_health),
            "item_armor1" => SpawnAction::Armor(0.0),
            "item_armor2" => SpawnAction::Armor(1.0),
            "item_armorInv" => SpawnAction::Armor(2.0),
            "weapon_supershotgun"
            | "weapon_nailgun"
            | "weapon_supernailgun"
            | "weapon_grenadelauncher"
            | "weapon_rocketlauncher"
            | "weapon_lightning" => SpawnAction::Weapon,
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

            // ctf.rs — CTF flags (spawn only in the ctf mode; harmlessly removed otherwise)
            "item_flag_team1" => SpawnAction::Spawn(GameState::spawn_flag_team1),
            "item_flag_team2" => SpawnAction::Spawn(GameState::spawn_flag_team2),

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
            "info_teleport_destination" => SpawnAction::Spawn(GameState::spawn_info_teleport_destination),
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

            // race.rs — embedded race-route data carriers, folded into `GameState.race` (and
            // freed) by `load_race_routes` at the end of entity spawn.
            "race_route_start" | "race_route_marker" => SpawnAction::Keep,

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
            "light_flame_small_yellow" | "light_flame_small_white" => SpawnAction::Flame {
                model: Model::PROGS_FLAME2,
                skin: 0.0,
                frame: true,
            },
            "func_wall" | "func_episodegate" | "func_bossgate" => SpawnAction::Spawn(GameState::spawn_func_wall),
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
    pub(crate) fn apply_fields(&mut self, id: EntId, fields: &SpawnFields) {
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
            // race.rs keys (race_route_start / race_route_marker data carriers).
            "race_route_name" => ent.race.name = Some(value.into()),
            "race_route_description" => ent.race.desc = Some(value.into()),
            "race_route_timeout" => ent.race.timeout = parse_f32(value),
            "race_route_weapon_mode" => ent.race.weapon_mode = parse_f32(value),
            "race_route_falsestart_mode" => ent.race.falsestart_mode = parse_f32(value),
            "race_route_start_yaw" => ent.race.start_yaw = parse_f32(value),
            // ktx's field table aliases pitch onto yaw (g_spawn.c:150) — a latent bug we
            // deliberately don't replicate; every shipped map uses 0/0 anyway.
            "race_route_start_pitch" => ent.race.start_pitch = parse_f32(value),
            "race_flags" => ent.race.flags = parse_f32(value),
            "size" => ent.race.size = parse_vec3(value),
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
