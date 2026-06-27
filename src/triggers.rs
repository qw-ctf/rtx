//! Trigger volumes, ported from `qw-qc/triggers.qc`: multiple/once/relay/secret/counter,
//! teleporters (with telefrag), hurt, push (jump pads), and monsterjump.

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{Die, EntId, Think, Touch, Use};
use crate::game::GameState;


impl GameState {
    // --- trigger_multiple / once / secret / counter / relay ---

    /// `multi_wait` — re-arm a damageable trigger.
    pub(crate) fn multi_wait(&mut self, e: EntId) {
        let ent = &mut self.entities[e];
        if ent.v.max_health != 0.0 {
            ent.v.health = ent.v.max_health;
            ent.v.takedamage = TakeDamage::Yes.as_f32();
            ent.v.solid = Solid::BBox.as_f32();
        }
    }

    /// `multi_trigger` — fire the trigger's targets, then re-arm or remove.
    fn multi_trigger(&mut self, e: EntId) {
        if self.entities[e].v.nextthink > self.time() {
            return;
        }
        if self.entities[e].classname() == Some("trigger_secret") {
            if self.entities[self.entities[e].enemy()].classname() != Some("player")
            {
                return;
            }
            self.globals.found_secrets += 1.0;
            self.host.write_svc(MsgDest::All, Svc::FoundSecret);
        }
        self.play_noise(e, Channel::Voice);
        self.entities[e].v.takedamage = TakeDamage::No.as_f32();
        self.activator = self.entities[e].enemy();
        self.sub_use_targets(e);

        let wait = self.entities[e].mover.wait;
        let time = self.time();
        let ent = &mut self.entities[e];
        if wait > 0.0 {
            ent.think = Think::MultiWait;
            ent.v.nextthink = time + wait;
        } else {
            ent.set_touch(Touch::None);
            ent.v.nextthink = time + 0.1;
            ent.think = Think::SubRemove;
        }
    }

    /// `multi_killed` (`th_die`).
    pub(crate) fn multi_killed(&mut self, e: EntId) {
        let attacker = self.damage_attacker;
        self.entities[e].set_enemy(attacker);
        self.multi_trigger(e);
    }

    /// `multi_use` (`use`).
    pub(crate) fn multi_use(&mut self, e: EntId) {
        let act = self.activator;
        self.entities[e].set_enemy(act);
        self.multi_trigger(e);
    }

    /// `multi_touch` (`touch`).
    pub(crate) fn multi_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].classname() != Some("player") {
            return;
        }
        let movedir = self.entities[e].v.movedir;
        if movedir != Vec3::ZERO {
            let angles = self.entities[other].v.angles;
            self.host.make_vectors(angles);
            if self.globals.v_forward.dot(movedir) < 0.0 {
                return;
            }
        }
        self.entities[e].set_enemy(other);
        self.multi_trigger(e);
    }

    /// `counter_use` — count down, then fire on completion.
    pub(crate) fn counter_use(&mut self, e: EntId) {
        let activator = self.activator;
        let (count, spawnflags) = {
            let ent = &self.entities[e];
            (ent.mover.count - 1.0, ent.v.spawnflags)
        };
        self.entities[e].mover.count = count;
        if count < 0.0 {
            return;
        }
        let is_player = self.entities[activator].classname() == Some("player");
        let show = is_player && !spawnflags.has(TriggerFlags::NOMESSAGE);
        if count != 0.0 {
            if show {
                let msg = if count >= 4.0 {
                    c"There are more to go..."
                } else if count == 3.0 {
                    c"Only 3 more to go..."
                } else if count == 2.0 {
                    c"Only 2 more to go..."
                } else {
                    c"Only 1 more to go..."
                };
                self.host.centerprint(activator.0 as i32, msg);
            }
            return;
        }
        if show {
            self.host
                .centerprint(activator.0 as i32, c"Sequence completed!");
        }
        self.entities[e].set_enemy(activator);
        self.multi_trigger(e);
    }

    /// `trigger_multiple` spawn (also backs `trigger_once`/`trigger_secret`).
    pub(crate) fn spawn_trigger_multiple(&mut self, e: EntId) -> bool {
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.entities[e].noise = Some(Sound::MISC_SECRET);
            }
            2 => {
                self.entities[e].noise = Some(Sound::MISC_TALK);
            }
            3 => {
                self.entities[e].noise = Some(Sound::MISC_TRIGGER1);
            }
            _ => {}
        }
        if self.entities[e].mover.wait == 0.0 {
            self.entities[e].mover.wait = 0.2;
        }
        self.entities[e].use_ = Use::MultiUse;
        self.init_trigger(e);

        if self.entities[e].v.health != 0.0 {
            let ent = &mut self.entities[e];
            ent.v.max_health = ent.v.health;
            ent.th_die = Die::TriggerKilled;
            ent.v.takedamage = TakeDamage::Yes.as_f32();
            ent.v.solid = Solid::BBox.as_f32();
            let origin = ent.v.origin;
            self.host.set_origin(e.0 as i32, origin);
        } else if !self.entities[e].v.spawnflags.has(TriggerFlags::NOTOUCH) {
            self.entities[e].set_touch(Touch::Multi);
        }
        true
    }

    pub(crate) fn spawn_trigger_once(&mut self, e: EntId) -> bool {
        self.entities[e].mover.wait = -1.0;
        self.spawn_trigger_multiple(e)
    }

    pub(crate) fn spawn_trigger_relay(&mut self, e: EntId) -> bool {
        self.entities[e].use_ = Use::TriggerRelay;
        true
    }

    pub(crate) fn spawn_trigger_secret(&mut self, e: EntId) -> bool {
        self.globals.total_secrets += 1.0;
        {
            let ent = &mut self.entities[e];
            ent.mover.wait = -1.0;
            if ent.message.is_none() {
                ent.message = Some("You found a secret area!".into());
            }
            if ent.v.sounds == 0.0 {
                ent.v.sounds = 1.0;
            }
        }
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.entities[e].noise = Some(Sound::MISC_SECRET);
            }
            2 => {
                self.entities[e].noise = Some(Sound::MISC_TALK);
            }
            _ => {}
        }
        self.spawn_trigger_multiple(e)
    }

    pub(crate) fn spawn_trigger_counter(&mut self, e: EntId) -> bool {
        let ent = &mut self.entities[e];
        ent.mover.wait = -1.0;
        if ent.mover.count == 0.0 {
            ent.mover.count = 2.0;
        }
        ent.use_ = Use::CounterUse;
        true
    }

    // --- teleporters ---

    /// `play_teleport` — random teleport whoosh, then remove the fog entity.
    pub(crate) fn play_teleport(&mut self, e: EntId) {
        let v = (self.random() * 5.0) as i32;
        let s = match v {
            0 => Sound::MISC_R_TELE1,
            1 => Sound::MISC_R_TELE2,
            2 => Sound::MISC_R_TELE3,
            3 => Sound::MISC_R_TELE4,
            _ => Sound::MISC_R_TELE5,
        };
        self.host.sound(e.0 as i32, Channel::Voice, s, 1.0, Attenuation::Norm);
        self.free(e);
    }

    /// `spawn_tfog` — teleport flash + delayed whoosh.
    fn spawn_tfog(&mut self, org: Vec3) {
        let time = self.time();
        let s = self.spawn();
        {
            let ent = &mut self.entities[s];
            ent.v.origin = org;
            ent.v.nextthink = time + 0.2;
            ent.think = Think::PlayTeleport;
        }
        self.host.write_te(MsgDest::Multicast, Te::Teleport);
        self.host.write_coord(MsgDest::Multicast, org.x);
        self.host.write_coord(MsgDest::Multicast, org.y);
        self.host.write_coord(MsgDest::Multicast, org.z);
        self.host.multicast(org, Multicast::Phs);
    }

    /// `tdeath_touch` — telefrag whoever is at the destination.
    pub(crate) fn tdeath_touch(&mut self, e: EntId, other: EntId) {
        let owner = self.entities[e].owner();
        if other == owner {
            return;
        }
        let time = self.time();
        if self.entities[other].classname() == Some("player") {
            let other_inv = self.entities[other].combat.invincible_finished > time;
            let owner_inv = self.entities[owner].combat.invincible_finished > time;
            if other_inv && owner_inv {
                self.entities[e].classname = Some("teledeath3".into());
                self.entities[other].combat.invincible_finished = 0.0;
                self.entities[owner].combat.invincible_finished = 0.0;
                self.t_damage(other, e, e, 50000.0);
                self.entities[e].set_owner(other);
                self.t_damage(owner, e, e, 50000.0);
                return;
            }
            if other_inv {
                self.entities[e].classname = Some("teledeath2".into());
                self.t_damage(owner, e, e, 50000.0);
                return;
            }
        }
        if self.entities[other].v.health != 0.0 {
            self.t_damage(other, e, e, 50000.0);
        }
    }

    /// `spawn_tdeath` — temporary telefrag volume at a spawn/teleport destination.
    pub(crate) fn spawn_tdeath(&mut self, org: Vec3, death_owner: EntId) {
        let time = self.time();
        let (mins, maxs) = {
            let v = &self.entities[death_owner].v;
            (v.mins, v.maxs)
        };
        let d = self.spawn();
        {
            let ent = &mut self.entities[d];
            ent.classname = Some("teledeath".into());
            ent.v.movetype = MoveType::None.as_f32();
            ent.v.solid = Solid::Trigger.as_f32();
            ent.v.angles = Vec3::ZERO;
            ent.set_touch(Touch::Tdeath);
            ent.v.nextthink = time + 0.2;
            ent.think = Think::SubRemove;
            ent.set_owner(death_owner);
        }
        self.host.set_size(
            d.0 as i32,
            mins - Vec3::ONE,
            maxs + Vec3::ONE,
        );
        self.host.set_origin(d.0 as i32, org);
        self.globals.force_retouch = 2.0;
    }

    /// `teleport_touch`.
    pub(crate) fn teleport_touch(&mut self, e: EntId, other: EntId) {
        let time = self.time();
        if self.entities[e].targetname.is_some()
            && self.entities[e].v.nextthink < time
        {
            return;
        }
        if self.entities[e].v.spawnflags.has(TeleportFlags::PLAYER_ONLY)
            && self.entities[other].classname() != Some("player")
        {
            return;
        }
        {
            let v = &self.entities[other].v;
            if v.health <= 0.0 || !v.solid.is(Solid::SlideBox) {
                return;
            }
        }
        self.activator = other;
        self.sub_use_targets(e);

        let other_org = self.entities[other].v.origin;
        self.spawn_tfog(other_org);

        let target = match self.entities[e].target.clone() {
            Some(t) => t,
            None => return,
        };
        let dest = match self.find_by_targetname(&target).next() {
            Some(d) => d,
            None => return,
        };
        let (t_org, t_mangle) = {
            let v = &self.entities[dest];
            (v.v.origin, v.mover.mangle)
        };
        self.host.make_vectors(t_mangle);
        let v_forward = self.globals.v_forward;
        self.spawn_tfog(t_org + v_forward * 32.0);
        self.spawn_tdeath(t_org, other);

        self.host.set_origin(other.0 as i32, t_org);
        {
            let o = &mut self.entities[other].v;
            o.origin = t_org;
            o.angles = t_mangle;
        }
        if self.entities[other].classname() == Some("player") {
            let o = &mut self.entities[other].v;
            o.fixangle = 1.0;
            o.teleport_time = time + 0.7;
            o.flags = o.flags.without(Flags::ONGROUND);
            o.velocity = v_forward * 300.0;
        }
    }

    /// `info_teleport_destination` spawn.
    pub(crate) fn spawn_info_teleport_destination(&mut self, e: EntId) -> bool {
        let ent = &mut self.entities[e];
        ent.mover.mangle = ent.v.angles;
        ent.v.angles = Vec3::ZERO;
        ent.model = None;
        ent.v.origin.z += 27.0;
        true
    }

    /// `teleport_use`.
    pub(crate) fn teleport_use(&mut self, e: EntId) {
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.nextthink = time + 0.2;
        ent.think = Think::None;
        self.globals.force_retouch = 2.0;
    }

    /// `trigger_teleport` spawn.
    pub(crate) fn spawn_trigger_teleport(&mut self, e: EntId) -> bool {
        self.init_trigger(e);
        self.entities[e].set_touch(Touch::Teleport);
        if self.entities[e].target.is_none() {
            return false;
        }
        self.entities[e].use_ = Use::TeleportUse;
        if !self.entities[e].v.spawnflags.has(TeleportFlags::SILENT) {
            let o = {
                let v = &self.entities[e].v;
                (v.mins + v.maxs) * 0.5
            };
            self.host
                .ambient_sound(o, Sound::AMBIENCE_HUM1, 0.5, Attenuation::Static);
        }
        true
    }

    // --- hurt ---

    /// `hurt_on` — re-enable a hurt trigger after its cooldown.
    pub(crate) fn hurt_on(&mut self, e: EntId) {
        let ent = &mut self.entities[e];
        ent.v.solid = Solid::Trigger.as_f32();
        ent.v.nextthink = -1.0;
    }

    /// `hurt_touch`.
    pub(crate) fn hurt_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].v.takedamage != 0.0 {
            let dmg = self.entities[e].mover.dmg;
            self.entities[e].v.solid = Solid::Not.as_f32();
            self.t_damage(other, e, e, dmg);
            let time = self.time();
            let ent = &mut self.entities[e];
            ent.think = Think::HurtOn;
            ent.v.nextthink = time + 1.0;
        }
    }

    /// `trigger_hurt` spawn.
    pub(crate) fn spawn_trigger_hurt(&mut self, e: EntId) -> bool {
        self.init_trigger(e);
        self.entities[e].set_touch(Touch::Hurt);
        if self.entities[e].mover.dmg == 0.0 {
            self.entities[e].mover.dmg = 5.0;
        }
        true
    }

    // --- push (jump pads / wind) ---

    /// `trigger_push_touch`.
    pub(crate) fn trigger_push_touch(&mut self, e: EntId, other: EntId) {
        let (speed, movedir, spawnflags) = {
            let ent = &self.entities[e];
            (ent.mover.speed, ent.v.movedir, ent.v.spawnflags)
        };
        let push = speed * movedir * 10.0;
        let is_grenade = self.entities[other].classname() == Some("grenade");
        if is_grenade {
            self.entities[other].v.velocity = push;
        } else if self.entities[other].v.health > 0.0 {
            self.entities[other].v.velocity = push;
            if self.entities[other].classname() == Some("player") {
                let time = self.time();
                if self.entities[other].combat.fly_sound < time {
                    self.entities[other].combat.fly_sound = time + 1.5;
                    self.host
                        .sound(other.0 as i32, Channel::Auto, Sound::AMBIENCE_WINDFLY, 1.0, Attenuation::Norm);
                }
            }
        }
        if spawnflags.has(PushFlags::ONCE) {
            self.free(e);
        }
    }

    /// `trigger_push` spawn.
    pub(crate) fn spawn_trigger_push(&mut self, e: EntId) -> bool {
        self.init_trigger(e);
        self.entities[e].set_touch(Touch::Push);
        if self.entities[e].mover.speed == 0.0 {
            self.entities[e].mover.speed = 1000.0;
        }
        true
    }

    /// `trigger_monsterjump` spawn (kept inert-but-present; no monsters in this subset).
    pub(crate) fn spawn_trigger_monsterjump(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            if ent.mover.speed == 0.0 {
                ent.mover.speed = 200.0;
            }
            if ent.mover.height == 0.0 {
                ent.mover.height = 200.0;
            }
            if ent.v.angles == Vec3::ZERO {
                ent.v.angles = Vec3::new(0.0, 360.0, 0.0);
            }
        }
        self.init_trigger(e);
        self.entities[e].set_touch(Touch::TriggerMonsterjump);
        true
    }
}
