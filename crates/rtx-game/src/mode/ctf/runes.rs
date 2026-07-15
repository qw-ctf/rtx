// SPDX-License-Identifier: AGPL-3.0-or-later

//! CTF runes (purectf): one held per player, dropped or tossed on death/impulse, four game runes
//! (Resistance / Strength / Haste / Regeneration) spawned at random deathmatch points when a match
//! goes live. These are `GameState` methods dispatched from the spawn/touch/think/impulse seams.

use glam::Vec3;

use super::RUNE_RESPAWN_TIME;
use crate::assets::{Model, Sound};
use crate::defs::{
    Attenuation, Channel, MoveType, Solid, RUNE_HASTE, RUNE_MASK, RUNE_REGEN, RUNE_RESISTANCE, RUNE_STRENGTH,
};
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;
use crate::mode::players;

impl GameState {
    /// Clear any runes and spawn the four game runes at random deathmatch spawns. Called when a CTF
    /// match goes live; a no-op if `rtx_runes` is off.
    pub(crate) fn spawn_runes(&mut self) {
        let existing: Vec<EntId> = self.find_by_classname("item_rune").collect();
        for r in existing {
            self.free(r);
        }
        for p in players(self) {
            self.entities[p].mode_p.ctf.runes = 0;
            self.refresh_haste_speed(p);
        }
        if !self.mode.uses_ctf_objects() || self.host.cvar(c"rtx_runes") as i32 == 1 {
            return; // 1 = runes off
        }
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            let spot = self.select_spawn_point(None); // item placement — no spawn memory
            if spot != EntId::WORLD {
                let org = self.entities[spot].v.origin;
                self.do_spawn_rune(org, bit);
            }
        }
    }

    /// Create one rune item (`item_rune`) near `origin`.
    fn do_spawn_rune(&mut self, origin: Vec3, bit: u8) -> EntId {
        let e = self.spawn();
        let (model, msg) = rune_asset(bit);
        {
            let ent = &mut self.entities[e];
            ent.classname = Some("item_rune".into());
            ent.item.rune_bit = bit; // which rune this item is
            ent.netname = Some(msg.into());
            ent.v.movetype = MoveType::Toss;
            ent.v.solid = Solid::Trigger;
        }
        self.set_model(e, model);
        self
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        let jx = -500.0 + self.random() * 1000.0;
        let jy = -500.0 + self.random() * 1000.0;
        let time = self.time();
        {
            let ent = &mut self.entities[e];
            ent.v.velocity = Vec3::new(jx, jy, 300.0);
            ent.think = Think::RuneRespawn;
            ent.v.nextthink = time + RUNE_RESPAWN_TIME;
        }
        self.entities[e].set_touch(Touch::Rune);
        self.set_origin(e, origin + Vec3::new(0.0, 0.0, 4.0));
        e
    }

    /// `Touch::Rune` — pick up a rune (one per player; a held rune blocks the pickup).
    pub(crate) fn rune_touch(&mut self, rune: EntId, other: EntId) {
        if other == self.entities[rune].owner()
            || !self.entities[other].is_player()
            || !self.entities[other].is_alive()
        {
            return;
        }
        if self.entities[other].mode_p.ctf.runes & RUNE_MASK != 0 {
            self.sprint_to(other, c"You already have a rune (tossrune to drop).\n");
            return;
        }
        let bit = self.entities[rune].item.rune_bit;
        self.entities[other].mode_p.ctf.runes |= bit;
        self.refresh_haste_speed(other);
        self.host
            .sound(other, Channel::Item, Sound::WEAPONS_LOCK4, 1.0, Attenuation::Norm);
        self.sprint_to(other, c"You got a rune!\n");
        self.free(rune);
    }

    /// `Think::RuneRespawn` — an untouched rune relocates to a fresh spawn.
    pub(crate) fn rune_respawn(&mut self, rune: EntId) {
        let bit = self.entities[rune].item.rune_bit;
        let spot = self.select_spawn_point(None); // item placement — no spawn memory
        let org = if spot != EntId::WORLD {
            self.entities[spot].v.origin
        } else {
            self.entities[rune].v.origin
        };
        self.free(rune);
        self.do_spawn_rune(org, bit);
    }

    /// Drop the runes a player holds where they are (called on death; owner-locked briefly).
    pub(crate) fn drop_runes(&mut self, player: EntId) {
        let runes = self.entities[player].mode_p.ctf.runes & RUNE_MASK;
        if runes == 0 {
            return;
        }
        let origin = self.entities[player].v.origin;
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            if runes & bit != 0 {
                self.do_spawn_rune(origin, bit);
            }
        }
        self.entities[player].mode_p.ctf.runes = 0;
        self.refresh_haste_speed(player);
    }

    /// Impulse 24 — toss your held rune(s) along your aim (gated by `rtx_ctf_tossrune`).
    pub(crate) fn toss_rune(&mut self, player: EntId) {
        // Reached only via `Ctf::handle_impulse`; just the cvar gate remains.
        if !self.host.cvar_bool(c"rtx_ctf_tossrune") {
            return;
        }
        let runes = self.entities[player].mode_p.ctf.runes & RUNE_MASK;
        if runes == 0 {
            return;
        }
        let origin = self.entities[player].v.origin;
        let dir = self.aim_dir(player);
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            if runes & bit != 0 {
                let r = self.do_spawn_rune(origin, bit);
                self.entities[r].v.velocity = dir * 300.0 + Vec3::new(0.0, 0.0, 200.0);
            }
        }
        self.entities[player].mode_p.ctf.runes = 0;
        self.refresh_haste_speed(player);
    }

    /// Recompute a player's move-speed cap for the Haste rune (`×1.25` when held, `rtx_runes 0`).
    pub(crate) fn refresh_haste_speed(&mut self, e: EntId) {
        let base = {
            let v = self.host.cvar(c"sv_maxspeed");
            if v > 0.0 {
                v
            } else {
                320.0
            }
        };
        // Only the "pure" mode (`rtx_runes 0`) grants the speed boost; `2` is haste-attack only.
        let pure = self.host.cvar(c"rtx_runes") as i32 == 0;
        let haste = pure && self.entities[e].mode_p.ctf.runes & RUNE_HASTE != 0;
        self.entities[e].maxspeed = base * if haste { 1.25 } else { 1.0 };
    }

    /// Regeneration rune: heal health/armor toward 150 while held (called each frame in prethink).
    pub(crate) fn ctf_rune_regen(&mut self, e: EntId) {
        if self.entities[e].mode_p.ctf.runes & RUNE_REGEN == 0 {
            return;
        }
        let dt = self.globals.frametime;
        let v = &mut self.entities[e].v;
        if v.health > 0.0 && v.health < 150.0 {
            v.health = (v.health + 10.0 * dt).min(150.0);
        }
        if v.armortype > 0.0 && v.armorvalue < 150.0 {
            v.armorvalue = (v.armorvalue + 10.0 * dt).min(150.0);
        }
    }
}

/// The `(model, pickup message)` for a rune bit.
fn rune_asset(bit: u8) -> (Model, &'static str) {
    match bit {
        RUNE_RESISTANCE => (Model::PROGS_END1, "Resistance"),
        RUNE_STRENGTH => (Model::PROGS_END2, "Strength"),
        RUNE_HASTE => (Model::PROGS_END3, "Haste"),
        _ => (Model::PROGS_END4, "Regeneration"),
    }
}
