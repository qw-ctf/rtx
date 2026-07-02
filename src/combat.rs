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
use crate::game::GameState;

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

        self.client_obituary(targ, attacker);

        // Notify the mode (round-based modes track eliminations / round end here).
        let mode = self.mode;
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
        // Mode-level damage gate (Rocket Arena countdown protection / harmless audience). Only
        // affects players; world objects (doors, grenades) always pass.
        let mode = self.mode;
        if !mode.damage_allowed(self, targ) {
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
            self.entities[targ].v.velocity += dir * damage * 8.0;

            let rj = self.host.cvar(c"rj");
            if rj > 1.0 && self.same_player_netname(attacker, targ) {
                self.entities[targ].v.velocity += dir * damage * rj;
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
        dtype: &str,
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
                self.entities[head].deathtype = Some(dtype.into());
                self.t_damage(head, inflictor, attacker, points);
            }
        }
    }

    /// `T_BeamDamage` — like radius damage but always centred on (and crediting) `attacker`.
    /// (A faithful port of the boss shockwave; no ported entity currently calls it.)
    #[allow(dead_code)]
    pub(crate) fn t_beam_damage(&mut self, attacker: EntId, damage: f32) {
        let org = self.entities[attacker].v.origin;
        for head in self.find_radius(org, damage + 40.0) {
            if self.entities[head].v.takedamage == 0.0 {
                continue;
            }
            let mut points = damage - 0.5 * (org - self.entities[head].v.origin).length();
            if points < 0.0 {
                points = 0.0;
            }
            if head == attacker {
                points *= 0.5;
            }
            if points > 0.0 && self.can_damage(head, attacker, attacker) {
                self.t_damage(head, attacker, attacker, points);
            }
        }
    }

    /// `ClientObituary` (compact) — deathmatch frag scoring, logging and a broadcast. The
    /// full per-weapon flavour text lands with the client.qc port; this keeps scores correct.
    pub(crate) fn client_obituary(&mut self, targ: EntId, attacker: EntId) {
        if self.entities[targ].classname() != Some("player") {
            return;
        }
        let targ_name = self.netname_of(targ);

        if self.entities[attacker].classname() == Some("player") {
            if targ == attacker {
                self.entities[targ].v.frags -= 1.0;
                self.broadcast(PrintLevel::Medium, &format!("{targ_name} suicides\n"));
                self.host.logfrag(targ, targ);
                return;
            }
            let attacker_name = self.netname_of(attacker);
            self.entities[attacker].v.frags += 1.0;
            self.host.logfrag(attacker, targ);
            self.broadcast(
                PrintLevel::Medium,
                &format!("{targ_name} was killed by {attacker_name}\n"),
            );
            return;
        }

        // Environmental / world death.
        self.entities[targ].v.frags -= 1.0;
        self.host.logfrag(targ, targ);
        self.broadcast(PrintLevel::Medium, &format!("{targ_name} died\n"));
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
    fn team_of(&self, ent: EntId) -> String {
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
        let c = crate::game::cstring(message);
        self.host.bprint(level, &c);
    }
}
