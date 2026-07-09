// SPDX-License-Identifier: AGPL-3.0-or-later

//! Damage and death, ported from `qw-qc/combat.qc` (plus a compact `ClientObituary`).
//!
//! `T_Damage` is the single chokepoint that reduces health; it handles armor absorption,
//! quad multipliers, knockback, god/pent protection, and teamplay before applying damage
//! and routing to `th_pain` or `Killed`.

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{EntId, Touch};
use crate::game::{self, GameState};
use crate::obituary::DeathType;

impl GameState {
    /// `CanDamage` — can `inflictor` reach `targ` with a clear trace (corners are tried for
    /// some slack)? `ignore` is excluded from the trace (the inflictor itself).
    pub(crate) fn can_damage(&mut self, targ: EntId, inflictor: EntId, ignore: EntId) -> bool {
        let inflictor_org = self.entities[inflictor].v.origin;
        let (targ_org, absmin, absmax, movetype) = {
            let v = &self.entities[targ].v;
            (v.origin, v.absmin, v.absmax, v.movetype)
        };

        if movetype == MoveType::Push {
            let mid = 0.5 * (absmin + absmax);
            let tr = self.traceline(inflictor_org, mid, true, ignore);
            return tr.fraction == 1.0 || tr.ent == targ;
        }

        let corners = [
            Vec3::ZERO,
            Vec3::new(15.0, 15.0, 0.0),
            Vec3::new(-15.0, -15.0, 0.0),
            Vec3::new(-15.0, 15.0, 0.0),
            Vec3::new(15.0, -15.0, 0.0),
        ];
        for c in corners {
            let tr = self.traceline(inflictor_org, targ_org + c, true, ignore);
            if tr.fraction == 1.0 {
                return true;
            }
        }
        false
    }

    /// `Killed` — `targ` has reached <= 0 health; run its death behaviour.
    pub(crate) fn killed(&mut self, targ: EntId, attacker: EntId) {
        {
            let t = &mut self.entities[targ];
            if t.v.health < -99.0 {
                t.v.health = -99.0;
            }
        }
        let movetype = self.entities[targ].v.movetype;
        if movetype == MoveType::Push || movetype == MoveType::None {
            // doors, triggers, etc.: their th_die does the work directly.
            self.run_die(targ);
            return;
        }

        self.entities[targ].set_enemy(attacker);

        // Opponent modeling: death is public (the frag is broadcast), so every side's hypothesis of
        // this player resets to the spawn kit — they'll respawn fresh. No-op unless it's a player.
        self.model_reset_target(targ);

        // The mode may print its own scoring/announcement (Midair's airshot tiers) in place of the
        // stock obituary; otherwise the default obituary runs.
        let mode = self.mode;
        if !mode.announce_death(self, targ, attacker) {
            self.client_obituary(targ, attacker);
        }

        // Notify the mode (round-based modes track eliminations / round end here).
        mode.on_death(self, targ, attacker);

        {
            let t = &mut self.entities[targ];
            t.v.takedamage = TakeDamage::No.as_f32();
            t.set_touch(Touch::None);
            t.v.effects = 0.0;
        }
        self.run_die(targ);
    }

    /// `T_Damage` — the only function that reduces health.
    pub(crate) fn t_damage(&mut self, targ: EntId, inflictor: EntId, attacker: EntId, mut damage: f32) {
        if self.entities[targ].v.takedamage == 0.0 {
            return;
        }
        if self.is_grenade(targ) {
            if self.is_live_shootable_grenade(targ) {
                self.grenade_explode(targ);
            }
            return;
        }

        let time = self.time();
        self.damage_attacker = attacker;

        // Quad damage on the attacker (but not for crushing doors).
        let inflictor_is_door = self.entities[inflictor].classname() == Some("door");
        if self.entities[attacker].combat.super_damage_finished > time && !inflictor_is_door {
            damage *= if self.level.deathmatch == 4 { 8.0 } else { 4.0 };
        }

        // Benched spectators (a structured match's non-roster late-joiners) neither deal nor take
        // player damage — the same hard gate Rocket Arena applies to its audience. Guard the
        // attacker check to player attackers so world hits (fall/lava/drowning) aren't blocked.
        if self.entities[targ].classname() == Some("player") {
            let att_benched = self.entities[attacker].classname() == Some("player")
                && crate::mode::team::benched(self, attacker);
            if att_benched || crate::mode::team::benched(self, targ) {
                return;
            }
        }

        // Mode damage ruleset (Rocket Arena countdown/audience protection; Midair airborne-only
        // kills + launch knockback; CTF rune Strength/Resistance scaling + the carrier-defense
        // window), applied after quad and before armor. A fully-blocked hit — no health *and* no
        // knockback — short-circuits exactly like the old boolean damage gate.
        let mode = self.mode;
        let outcome = mode.player_damage(self, targ, attacker, inflictor, damage);
        if outcome.health <= 0.0 && outcome.knockback <= 0.0 {
            return;
        }
        damage = outcome.health;
        let knockback = outcome.knockback;

        // Armor absorption.
        let take;
        {
            let t = &mut self.entities[targ];
            let mut save = (t.v.armortype * damage).ceil();
            if save >= t.v.armorvalue {
                save = t.v.armorvalue;
                t.v.armortype = 0.0;
                t.v.items = t.v.items.without(Items::ARMOR1 | Items::ARMOR2 | Items::ARMOR3);
            }
            t.v.armorvalue -= save;
            take = (damage - save).ceil();

            if t.v.flags.has(Flags::CLIENT) {
                t.v.dmg_take += take;
                t.v.dmg_save += save;
                t.v.dmg_inflictor = inflictor.to_prog();
            }
        }
        self.damage_inflictor = inflictor;

        // Knockback.
        let inflictor_org = {
            let v = &self.entities[inflictor].v;
            (v.absmin + v.absmax) * 0.5
        };
        if inflictor != EntId::WORLD && self.entities[targ].v.movetype == MoveType::Walk {
            let dir = (self.entities[targ].v.origin - inflictor_org).normalize_or_zero();
            self.entities[targ].v.velocity += dir * knockback * 8.0;

            let rj = self.host.cvar(c"rj");
            if rj > 1.0 && self.same_player_netname(attacker, targ) {
                self.entities[targ].v.velocity += dir * knockback * rj;
            }
        }

        // God mode / pentagram protection.
        if self.entities[targ].v.flags.has(Flags::GODMODE) {
            return;
        }
        if self.entities[targ].combat.invincible_finished >= time {
            if self.entities[targ].combat.invincible_sound < time {
                self.host
                    .sound(targ, Channel::Item, Sound::ITEMS_PROTECT3, 1.0, Attenuation::Norm);
                self.entities[targ].combat.invincible_sound = time + 2.0;
            }
            return;
        }

        // Teamplay damage avoidance (QW uses the "team" userinfo key).
        if self.teamplay_protects(targ, attacker, inflictor) {
            return;
        }

        // Apply.
        self.entities[targ].v.health -= take;

        // Opponent modeling: the attacker's side learns `targ` lost roughly `damage` off its stack
        // (they saw the hit land — run the delivered, pre-armor amount through the estimate's own
        // armor). And an identifiable projectile proves the attacker owns that weapon, which the
        // victim's side felt. Ambiguous inflictors (spikes: nailgun vs super-nailgun) are left to the
        // fire-heard hook. Both are no-ops when opponent modeling is off.
        self.model_note_damage(attacker, targ, damage);
        let inflictor_weapon = match self.entities[inflictor].classname() {
            Some("rocket") => Some(Items::ROCKET_LAUNCHER),
            Some("grenade") => Some(Items::GRENADE_LAUNCHER),
            _ => None,
        };
        if let Some(bit) = inflictor_weapon {
            self.model_note_weapon_of_attacker(targ, attacker, bit);
        }

        // Perception "feel": a bot that just took damage instantly registers its attacker (you turn
        // toward the hit when shot in the back), bypassing the view-cone/reaction gate. Like sound,
        // this reveals only a *direction* — the bot hunts a hypothesised point along the bearing the
        // hit came from, not the shooter's exact spot (only sight pins that). Perception then reads
        // `known_enemy`/`known_until` next frame. Skipped for self-damage and world hazards.
        if self.entities[targ].bot.is_bot && attacker != targ && attacker != EntId::WORLD {
            let targ_org = self.entities[targ].v.origin;
            let atk_org = self.entities[attacker].v.origin;
            let (r_lat, r_dist) = (self.random(), self.random());
            let pt = crate::bot::perception::heard_hypothesis(targ_org, atk_org, r_lat, r_dist);
            let b = &mut self.entities[targ].bot;
            b.known_enemy = attacker.0;
            b.known_until = time + crate::bot::perception::MEMORY;
            b.percept_last_seen = pt; // felt the hit's direction, not the shooter's exact position
        }

        if self.entities[targ].v.health <= 0.0 {
            self.killed(targ, attacker);
            return;
        }

        self.run_pain(targ, attacker, take);
    }

    /// `T_RadiusDamage` — splash damage to everything near `inflictor`, falling off linearly.
    pub(crate) fn t_radius_damage(
        &mut self,
        inflictor: EntId,
        attacker: EntId,
        damage: f32,
        ignore: EntId,
        dtype: DeathType,
    ) {
        let org = self.entities[inflictor].v.origin;
        for head in self.find_radius(org, damage + 40.0) {
            if head == ignore || self.entities[head].v.takedamage == 0.0 {
                continue;
            }
            let henter = {
                let v = &self.entities[head].v;
                v.origin + (v.mins + v.maxs) * 0.5
            };
            let mut points = damage - 0.5 * (org - henter).length();
            if points < 0.0 {
                points = 0.0;
            }
            if head == attacker {
                points *= 0.5;
            }
            if points > 0.0 && self.can_damage(head, inflictor, inflictor) {
                self.entities[head].deathtype = dtype;
                self.t_damage(head, inflictor, attacker, points);
            }
        }
    }

    // --- small helpers ---

    /// Whether `a` and `b` are both players sharing a netname (rocket-jump self-boost case).
    fn same_player_netname(&self, a: EntId, b: EntId) -> bool {
        self.entities[a].classname() == Some("player")
            && self.entities[b].classname() == Some("player")
            && self.netname_of(a) == self.netname_of(b)
    }

    /// Whether teamplay rules make `attacker`'s damage to `targ` a no-op.
    fn teamplay_protects(&self, targ: EntId, attacker: EntId, inflictor: EntId) -> bool {
        // A telefrag is never blocked — a protected teammate would otherwise be stuck inside you
        // forever (KTX `!TELEDEATH(targ)`, combat.c:752).
        if self.entities[targ].deathtype.is_telefrag() {
            return false;
        }
        let tp = self.level.teamplay;
        if tp != 1 && tp != 3 {
            return false;
        }
        if self.entities[attacker].classname() != Some("player") {
            return false;
        }
        if self.entities[inflictor].classname() == Some("door") {
            return false;
        }
        let at = self.team_of(attacker);
        let tt = self.team_of(targ);
        if at.is_empty() || at != tt {
            return false;
        }
        // tp 1: no team damage at all; tp 3: no team damage except to yourself.
        tp == 1 || targ != attacker
    }

    /// The "team" userinfo value for a client.
    pub(crate) fn team_of(&self, ent: EntId) -> String {
        let mut buf = [0u8; 32];
        self.host.infokey(ent, c"team", &mut buf).to_owned()
    }

    /// A display name for an entity (its stored netname, else its classname).
    pub(crate) fn netname_of(&self, ent: EntId) -> String {
        let e = &self.entities[ent];
        e.netname.as_deref().or_else(|| e.classname()).unwrap_or("").to_owned()
    }
}

impl GameState {
    /// `bprint` of a dynamic message to every client (shared by combat/items/etc).
    pub(crate) fn broadcast(&self, level: PrintLevel, message: &str) {
        let c = game::cstring(message);
        self.host.bprint(level, &c);
    }
}
