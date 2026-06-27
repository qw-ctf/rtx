// SPDX-License-Identifier: AGPL-3.0-or-later

//! Player animation, ported from `qw-qc/player.qc`.
//!
//! QuakeC drives player animation as a think-chained state machine: each frame function
//! sets `self.frame`, schedules `nextthink = time + 0.1`, and points `self.think` at the
//! next function. We model the same loop with the [`Think`] enum and the engine's
//! `GAME_EDICT_THINK` callback (the engine ignores the entvars `think` funcref for native
//! modules and re-enters us whenever `nextthink` elapses).


use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::anim::{Anim, frames};
use crate::defs::*;
use crate::entity::{EntId, Think};
use crate::game::GameState;

// `player.mdl` frames and the animations built on them. The `frames!` machinery and the `Anim`
// type live in `crate::anim`; this is just `player.mdl`'s table — QuakeC's two constructs, where
// `number { … }` is the flat `$frame` numbering and each `anim NAME = FIRST ..= LAST;` is a
// `player_*` frame-chain (e.g. the axe swing `player_axe1..4` stops at `$axatt4`, so it spans
// only four of the six declared `$axatt` frames; `$axatt5`/`$axatt6` are numbered but unplayed).
frames! {
    number {
        // running
        AXRUN1 AXRUN2 AXRUN3 AXRUN4 AXRUN5 AXRUN6
        ROCKRUN1 ROCKRUN2 ROCKRUN3 ROCKRUN4 ROCKRUN5 ROCKRUN6
        // standing
        STAND1 STAND2 STAND3 STAND4 STAND5
        AXSTND1 AXSTND2 AXSTND3 AXSTND4 AXSTND5 AXSTND6
        AXSTND7 AXSTND8 AXSTND9 AXSTND10 AXSTND11 AXSTND12
        // pain
        AXPAIN1 AXPAIN2 AXPAIN3 AXPAIN4 AXPAIN5 AXPAIN6
        PAIN1 PAIN2 PAIN3 PAIN4 PAIN5 PAIN6
        // dying
        AXDETH1 AXDETH2 AXDETH3 AXDETH4 AXDETH5 AXDETH6 AXDETH7 AXDETH8 AXDETH9
        DEATHA1 DEATHA2 DEATHA3 DEATHA4 DEATHA5 DEATHA6 DEATHA7 DEATHA8
        DEATHA9 DEATHA10 DEATHA11
        DEATHB1 DEATHB2 DEATHB3 DEATHB4 DEATHB5 DEATHB6 DEATHB7 DEATHB8 DEATHB9
        DEATHC1 DEATHC2 DEATHC3 DEATHC4 DEATHC5 DEATHC6 DEATHC7 DEATHC8
        DEATHC9 DEATHC10 DEATHC11 DEATHC12 DEATHC13 DEATHC14 DEATHC15
        DEATHD1 DEATHD2 DEATHD3 DEATHD4 DEATHD5 DEATHD6 DEATHD7 DEATHD8 DEATHD9
        DEATHE1 DEATHE2 DEATHE3 DEATHE4 DEATHE5 DEATHE6 DEATHE7 DEATHE8 DEATHE9
        // attacking
        NAILATT1 NAILATT2
        LIGHT1 LIGHT2
        ROCKATT1 ROCKATT2 ROCKATT3 ROCKATT4 ROCKATT5 ROCKATT6
        SHOTATT1 SHOTATT2 SHOTATT3 SHOTATT4 SHOTATT5 SHOTATT6
        AXATT1 AXATT2 AXATT3 AXATT4 AXATT5 AXATT6
        AXATTB1 AXATTB2 AXATTB3 AXATTB4 AXATTB5 AXATTB6
        AXATTC1 AXATTC2 AXATTC3 AXATTC4 AXATTC5 AXATTC6
        AXATTD1 AXATTD2 AXATTD3 AXATTD4 AXATTD5 AXATTD6
    }

    // locomotion (looping)
    anim AXRUN   = AXRUN1   ..= AXRUN6;
    anim ROCKRUN = ROCKRUN1 ..= ROCKRUN6;
    anim STAND   = STAND1   ..= STAND5;
    anim AXSTND  = AXSTND1  ..= AXSTND12;
    // pain / death (one-shot)
    anim AXPAIN  = AXPAIN1  ..= AXPAIN6;
    anim PAIN    = PAIN1    ..= PAIN6;
    anim AXDETH  = AXDETH1  ..= AXDETH9;
    anim DEATHA  = DEATHA1  ..= DEATHA11;
    anim DEATHB  = DEATHB1  ..= DEATHB9;
    anim DEATHC  = DEATHC1  ..= DEATHC15;
    anim DEATHD  = DEATHD1  ..= DEATHD9;
    anim DEATHE  = DEATHE1  ..= DEATHE9;
    // weapon fire
    anim NAILATT = NAILATT1 ..= NAILATT2;
    anim LIGHT   = LIGHT1   ..= LIGHT2;
    anim ROCKATT = ROCKATT1 ..= ROCKATT6;
    anim SHOTATT = SHOTATT1 ..= SHOTATT6;
    anim AXATT   = AXATT1   ..= AXATT4;   // axe plays 4 of its 6 declared frames
    anim AXATTB  = AXATTB1  ..= AXATTB4;
    anim AXATTC  = AXATTC1  ..= AXATTC4;
    anim AXATTD  = AXATTD1  ..= AXATTD4;
}

impl GameState {
    /// Re-arm an animation loop: schedule the next think 0.1s out and record which loop.
    fn schedule_anim(&mut self, e: EntId, think: Think) {
        let next = self.globals.time + 0.1;
        let ent = &mut self.entities[e];
        ent.think = think;
        ent.v.nextthink = next;
    }

    /// `player_stand1` — idle loop; transitions to the run loop while moving.
    pub(crate) fn player_stand1(&mut self, e: EntId) {
        self.schedule_anim(e, Think::PlayerStand);

        let moving = {
            let ent = &mut self.entities[e];
            ent.v.weaponframe = 0.0;
            ent.v.velocity.x != 0.0 || ent.v.velocity.y != 0.0
        };
        if moving {
            self.entities[e].anim.walkframe = 0;
            self.player_run(e);
            return;
        }

        let ent = &mut self.entities[e];
        let anim = if ent.v.weapon.is(Items::AXE) { AXSTND } else { STAND };
        if ent.anim.walkframe >= anim.len {
            ent.anim.walkframe = 0;
        }
        ent.v.frame = anim.frame(ent.anim.walkframe);
        ent.anim.walkframe += 1;
    }

    /// `player_run` — running loop; transitions back to idle when stopped.
    pub(crate) fn player_run(&mut self, e: EntId) {
        self.schedule_anim(e, Think::PlayerRun);

        let stopped = {
            let ent = &mut self.entities[e];
            ent.v.weaponframe = 0.0;
            ent.v.velocity.x == 0.0 && ent.v.velocity.y == 0.0
        };
        if stopped {
            self.entities[e].anim.walkframe = 0;
            self.player_stand1(e);
            return;
        }

        let ent = &mut self.entities[e];
        let anim = if ent.v.weapon.is(Items::AXE) { AXRUN } else { ROCKRUN };
        if ent.anim.walkframe >= anim.len {
            ent.anim.walkframe = 0;
        }
        ent.v.frame = anim.frame(ent.anim.walkframe);
        ent.anim.walkframe += 1;
    }

    // --- weapon firing animations (driven by W_Attack) ---

    /// Begin a cosmetic weapon animation: `walkframe` is the cursor, `anim` supplies the body
    /// frames (and their count), and `wf_base`/`fire`/`muzzle` are the per-weapon events.
    fn start_weapon_anim(&mut self, e: EntId, anim: Anim, wf_base: i32, fire: i32, muzzle: i32) {
        {
            let ent = &mut self.entities[e];
            ent.anim.walkframe = 0;
            ent.anim.anim_base = anim.first;
            ent.anim.anim_wf_base = wf_base;
            ent.anim.anim_len = anim.len;
            ent.anim.anim_fire = fire;
            ent.anim.anim_muzzle = muzzle;
            ent.think = Think::PlayerWeaponAnim;
        }
        // Run the first frame immediately (as the QuakeC `player_*1` body does).
        self.player_weapon_anim(e);
    }

    pub(crate) fn start_shot_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, SHOTATT, 1, -1, 0);
    }

    pub(crate) fn start_rocket_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, ROCKATT, 1, -1, 0);
    }

    pub(crate) fn start_axe_anim(&mut self, e: EntId) {
        // Four cosmetic variants; all fire the axe on the third frame.
        let (anim, wf_base) = match (self.random() * 4.0) as i32 {
            0 => (AXATT, 1),
            1 => (AXATTB, 5),
            2 => (AXATTC, 1),
            _ => (AXATTD, 5),
        };
        self.start_weapon_anim(e, anim, wf_base, 2, -1);
    }

    /// `PlayerWeaponAnim` think — advance one cosmetic weapon frame.
    pub(crate) fn player_weapon_anim(&mut self, e: EntId) {
        let (wf, base, wf_base, len, fire, muzzle) = {
            let ent = &self.entities[e];
            (ent.anim.walkframe, ent.anim.anim_base, ent.anim.anim_wf_base, ent.anim.anim_len, ent.anim.anim_fire, ent.anim.anim_muzzle)
        };
        if wf == muzzle {
            self.muzzleflash(e);
        }
        {
            let ent = &mut self.entities[e];
            ent.v.frame = (base + wf) as f32;
            ent.v.weaponframe = (wf_base + wf) as f32;
        }
        if wf == fire {
            self.w_fire_axe(e);
        }
        if wf >= len - 1 {
            self.player_run(e);
            return;
        }
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.anim.walkframe = wf + 1;
        ent.think = Think::PlayerWeaponAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// Begin the looping nailgun fire.
    pub(crate) fn start_nail(&mut self, e: EntId) {
        self.entities[e].anim.walkframe = 0;
        self.player_nail(e);
    }

    /// `player_nail1`/`player_nail2` think — fire alternating spikes while attack held.
    pub(crate) fn player_nail(&mut self, e: EntId) {
        self.muzzleflash(e);
        let (button0, impulse) = {
            let v = &self.entities[e].v;
            (v.button0, v.impulse)
        };
        if button0 == 0.0 || self.intermission_running || impulse != 0.0 {
            self.player_run(e);
            return;
        }
        {
            let ent = &mut self.entities[e];
            ent.v.weaponframe += 1.0;
            if ent.v.weaponframe == 9.0 {
                ent.v.weaponframe = 1.0;
            }
        }
        self.super_damage_sound(e);
        let parity = self.entities[e].anim.walkframe & 1;
        let dir = if parity == 0 { 4.0 } else { -4.0 };
        self.w_fire_spikes(e, dir);
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.combat.attack_finished = time + 0.2;
        ent.v.frame = NAILATT.frame(parity);
        ent.anim.walkframe = parity ^ 1;
        ent.think = Think::PlayerNail;
        ent.v.nextthink = time + 0.1;
    }

    /// Begin the looping lightning fire.
    pub(crate) fn start_light(&mut self, e: EntId) {
        self.entities[e].anim.walkframe = 0;
        self.player_light(e);
    }

    /// `player_light1`/`player_light2` think — fire lightning while attack held.
    pub(crate) fn player_light(&mut self, e: EntId) {
        self.muzzleflash(e);
        let button0 = self.entities[e].v.button0;
        if button0 == 0.0 || self.intermission_running {
            self.player_run(e);
            return;
        }
        {
            let ent = &mut self.entities[e];
            ent.v.weaponframe += 1.0;
            if ent.v.weaponframe == 5.0 {
                ent.v.weaponframe = 1.0;
            }
        }
        self.super_damage_sound(e);
        self.w_fire_lightning(e);
        let parity = self.entities[e].anim.walkframe & 1;
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.combat.attack_finished = time + 0.2;
        ent.v.frame = LIGHT.frame(parity);
        ent.anim.walkframe = parity ^ 1;
        ent.think = Think::PlayerLight;
        ent.v.nextthink = time + 0.1;
    }

    // --- pain & death animations ---

    /// Start a one-shot body `anim`, then run `after` once it reaches its final frame.
    fn start_body_anim(&mut self, e: EntId, anim: Anim, after: Think) {
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.frame = anim.frame(0);
        ent.anim.anim_end = anim.last();
        ent.anim.anim_after = after;
        ent.think = Think::PlayerAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// `PlayerAnim` think — advance a one-shot body animation one frame.
    pub(crate) fn player_anim_tick(&mut self, e: EntId) {
        let (frame, end, after) = {
            let ent = &self.entities[e];
            (ent.v.frame as i32, ent.anim.anim_end, ent.anim.anim_after)
        };
        if frame >= end {
            self.run_think_now(e, after);
            return;
        }
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.frame = (frame + 1) as f32;
        ent.think = Think::PlayerAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// `PlayerDead` — freeze at the final death frame and mark respawnable.
    pub(crate) fn player_dead(&mut self, e: EntId) {
        let ent = &mut self.entities[e];
        ent.v.nextthink = -1.0;
        ent.v.deadflag = DeadFlag::Dead.as_f32();
    }

    /// `player_pain` (`th_pain`) — play a pain sequence if not mid-attack.
    pub(crate) fn player_pain(&mut self, e: EntId, _attacker: EntId, _damage: f32) {
        let time = self.time();
        let (weaponframe, invisible, weapon) = {
            let ent = &self.entities[e];
            (ent.v.weaponframe, ent.combat.invisible_finished, ent.v.weapon)
        };
        if weaponframe != 0.0 || invisible > time {
            return;
        }
        self.entities[e].v.weaponframe = 0.0;
        self.pain_sound(e);
        let anim = if weapon.is(Items::AXE) { AXPAIN } else { PAIN };
        self.start_body_anim(e, anim, Think::PlayerRun);
    }

    /// `PainSound` — context-sensitive pain/drown/burn vocalisation.
    fn pain_sound(&mut self, e: EntId) {
        let time = self.time();
        let (health, watertype, waterlevel, pain_finished, axhitme) = {
            let ent = &self.entities[e];
            (ent.v.health, ent.v.watertype, ent.v.waterlevel, ent.combat.pain_finished, ent.combat.axhitme)
        };
        if health < 0.0 {
            return;
        }
        if self.entities[self.damage_attacker].classname() == Some("teledeath") {
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_TELEDTH1, 1.0, Attenuation::None);
            return;
        }
        if watertype.is(Content::Water) && waterlevel == 3.0 {
            self.death_bubbles(e, 1.0);
            let s = if self.random() > 0.5 { Sound::PLAYER_DROWN1 } else { Sound::PLAYER_DROWN2 };
            self.host.sound(e, Channel::Voice, s, 1.0, Attenuation::Norm);
            return;
        }
        if watertype.is(Content::Slime) || watertype.is(Content::Lava) {
            let s = if self.random() > 0.5 { Sound::PLAYER_LBURN1 } else { Sound::PLAYER_LBURN2 };
            self.host.sound(e, Channel::Voice, s, 1.0, Attenuation::Norm);
            return;
        }
        if pain_finished > time {
            self.entities[e].combat.axhitme = 0.0;
            return;
        }
        self.entities[e].combat.pain_finished = time + 0.5;
        if axhitme == 1.0 {
            self.entities[e].combat.axhitme = 0.0;
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_AXHIT1, 1.0, Attenuation::Norm);
            return;
        }
        let rs = (self.random() * 5.0).round() as i32 + 1;
        let noise = match rs {
            1 => Sound::PLAYER_PAIN1,
            2 => Sound::PLAYER_PAIN2,
            3 => Sound::PLAYER_PAIN3,
            4 => Sound::PLAYER_PAIN4,
            5 => Sound::PLAYER_PAIN5,
            _ => Sound::PLAYER_PAIN6,
        };
        self.host.sound(e, Channel::Voice, noise, 1.0, Attenuation::Norm);
    }

    /// `DeathSound`.
    fn death_sound(&mut self, e: EntId) {
        if self.entities[e].v.waterlevel == 3.0 {
            self.death_bubbles(e, 5.0);
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_H2ODEATH, 1.0, Attenuation::None);
            return;
        }
        let rs = (self.random() * 4.0).round() as i32 + 1;
        let noise = match rs {
            1 => Sound::PLAYER_DEATH1,
            2 => Sound::PLAYER_DEATH2,
            3 => Sound::PLAYER_DEATH3,
            4 => Sound::PLAYER_DEATH4,
            _ => Sound::PLAYER_DEATH5,
        };
        self.host.sound(e, Channel::Voice, noise, 1.0, Attenuation::None);
    }

    /// `PlayerDie` (`th_die`) — drop loot, start the death animation or gib.
    pub(crate) fn player_die(&mut self, e: EntId) {
        let time = self.time();
        {
            let ent = &mut self.entities[e];
            ent.v.items = ent.v.items.without(Items::INVISIBILITY);
            ent.combat.invisible_finished = 0.0;
            ent.combat.invincible_finished = 0.0;
            ent.combat.super_damage_finished = 0.0;
            ent.combat.radsuit_finished = 0.0;
        }
        self.drop_backpack(e);
        let vz = self.entities[e].v.velocity.z;
        let zboost = if vz < 10.0 { self.rng_unit() * 300.0 } else { 0.0 };
        {
            let ent = &mut self.entities[e];
            ent.weaponmodel = None;
            ent.v.view_ofs = Vec3::new(0.0, 0.0, -8.0);
            ent.v.deadflag = DeadFlag::Dying.as_f32();
            ent.v.solid = Solid::Not.as_f32();
            ent.v.flags = ent.v.flags.without(Flags::ONGROUND);
            ent.v.movetype = MoveType::Toss.as_f32();
            ent.v.velocity.z += zboost;
        }
        self.set_weaponmodel(e, None); // clear the networked viewmodel

        let health = self.entities[e].v.health;
        if health < -40.0 {
            self.gib_player(e);
            return;
        }
        self.death_sound(e);
        {
            let ent = &mut self.entities[e];
            ent.v.angles.x = 0.0;
            ent.v.angles.z = 0.0;
        }
        if self.entities[e].v.weapon.is(Items::AXE) {
            self.start_body_anim(e, AXDETH, Think::PlayerDead);
            return;
        }
        let _ = time;
        let anim = match 1 + (self.random() * 6.0).floor() as i32 {
            1 => DEATHA,
            2 => DEATHB,
            3 => DEATHC,
            4 => DEATHD,
            _ => DEATHE,
        };
        self.start_body_anim(e, anim, Think::PlayerDead);
    }

    /// `set_suicide_frame` — freeze a fresh corpse (kill/disconnect), unless already gibbed.
    pub(crate) fn set_suicide_frame(&mut self, e: EntId) {
        if self.entities[e].model.as_deref() != Some("progs/player.mdl") {
            return;
        }
        let ent = &mut self.entities[e];
        ent.v.frame = DEATHA.last() as f32;
        ent.v.solid = Solid::Not.as_f32();
        ent.v.movetype = MoveType::Toss.as_f32();
        ent.v.deadflag = DeadFlag::Dead.as_f32();
        ent.v.nextthink = -1.0;
    }

    // --- gibs ---

    /// `VelocityForDamage`.
    fn velocity_for_damage(&mut self, e: EntId, dm: f32) -> Vec3 {
        let infl = self.damage_inflictor;
        let infl_vel = self.entities[infl].v.velocity;
        let origin = self.entities[e].v.origin;
        let mut v;
        if infl_vel.length() > 0.0 {
            v = 0.5 * infl_vel;
            let infl_org = self.entities[infl].v.origin;
            v += 25.0 * (origin - infl_org).normalize_or_zero();
            v.z = 100.0 + 240.0 * self.random();
            v.x += 200.0 * self.rng_unit();
            v.y += 200.0 * self.rng_unit();
        } else {
            v = Vec3::new(100.0 * self.rng_unit(), 100.0 * self.rng_unit(), 200.0 + 100.0 * self.random());
        }
        v * if dm > -50.0 { 0.7 } else if dm > -200.0 { 2.0 } else { 10.0 }
    }

    /// `ThrowGib` — spawn a tumbling gib model.
    fn throw_gib(&mut self, e: EntId, gibname: Model, dm: f32) {
        let origin = self.entities[e].v.origin;
        let vel = self.velocity_for_damage(e, dm);
        let time = self.time();
        let avel = Vec3::new(self.random() * 600.0, self.random() * 600.0, self.random() * 600.0);
        let nextthink = time + 10.0 + self.random() * 10.0;
        let g = self.spawn();
        {
            let gib = &mut self.entities[g];
            gib.v.origin = origin;
            gib.v.velocity = vel;
            gib.v.movetype = MoveType::Bounce.as_f32();
            gib.v.solid = Solid::Not.as_f32();
            gib.v.avelocity = avel;
            gib.think = Think::SubRemove;
            gib.v.ltime = time;
            gib.v.nextthink = nextthink;
            gib.v.frame = 0.0;
            gib.v.flags = 0.0;
        }
        self.host.set_model(g, gibname);
        self.host.set_size(g, Vec3::ZERO, Vec3::ZERO);
        self.host.set_origin(g, origin);
    }

    /// `ThrowHead` — turn the player entity itself into a flying head gib.
    fn throw_head(&mut self, e: EntId, gibname: Model, dm: f32) {
        let vel = self.velocity_for_damage(e, dm);
        let avel = self.rng_unit() * Vec3::new(0.0, 600.0, 0.0);
        self.host.set_model(e, gibname);
        {
            let ent = &mut self.entities[e];
            ent.v.frame = 0.0;
            ent.v.nextthink = -1.0;
            ent.v.movetype = MoveType::Bounce.as_f32();
            ent.v.takedamage = TakeDamage::No.as_f32();
            ent.v.solid = Solid::Not.as_f32();
            ent.v.view_ofs = Vec3::new(0.0, 0.0, 8.0);
            ent.v.velocity = vel;
            ent.v.flags = ent.v.flags.without(Flags::ONGROUND);
            ent.v.avelocity = avel;
        }
        self.host.set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        let mut origin = self.entities[e].v.origin;
        origin.z -= 24.0;
        self.host.set_origin(e, origin);
        self.entities[e].v.origin = origin;
    }

    /// `GibPlayer`.
    fn gib_player(&mut self, e: EntId) {
        let health = self.entities[e].v.health;
        self.throw_head(e, Model::PROGS_H_PLAYER, health);
        self.throw_gib(e, Model::PROGS_GIB1, health);
        self.throw_gib(e, Model::PROGS_GIB2, health);
        self.throw_gib(e, Model::PROGS_GIB3, health);
        self.entities[e].v.deadflag = DeadFlag::Dead.as_f32();
        if self.entities[self.damage_attacker].classname() == Some("teledeath") {
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_TELEDTH1, 1.0, Attenuation::None);
            return;
        }
        let s = if self.random() < 0.5 { Sound::PLAYER_GIB } else { Sound::PLAYER_UDEATH };
        self.host.sound(e, Channel::Voice, s, 1.0, Attenuation::None);
    }

    /// A random value in `[-1, 1)` (QuakeC `crandom`).
    fn rng_unit(&mut self) -> f32 {
        2.0 * (self.random() - 0.5)
    }

    /// `DeathBubbles` — air bubbles when dying underwater. Cosmetic; the bubble-spawner
    /// chain is omitted for now (the death/drown sounds still play).
    fn death_bubbles(&mut self, _e: EntId, _count: f32) {}
}
