//! Player animation, ported from `qw-qc/player.qc`.
//!
//! QuakeC drives player animation as a think-chained state machine: each frame function
//! sets `self.frame`, schedules `nextthink = time + 0.1`, and points `self.think` at the
//! next function. We model the same loop with the [`Think`] enum and the engine's
//! `GAME_EDICT_THINK` callback (the engine ignores the entvars `think` funcref for native
//! modules and re-enters us whenever `nextthink` elapses).

use core::ffi::CStr;

use glam::Vec3;

use crate::defs::*;
use crate::entity::{EntId, Think};
use crate::game::GameState;

// player.mdl frame indices (sequential across player.qc's `$frame` declarations).
const AXRUN1: i32 = 0;
const ROCKRUN1: i32 = 6;
const STAND1: i32 = 12;
const AXSTND1: i32 = 17;
const AXPAIN1: i32 = 29;
const PAIN1: i32 = 35;
const AXDETH1: i32 = 41;
const DEATHA1: i32 = 50;
const DEATHB1: i32 = 61;
const DEATHC1: i32 = 70;
const DEATHD1: i32 = 85;
const DEATHE1: i32 = 94;
const DEATHA11: i32 = 60;
const NAILATT1: i32 = 103;
const LIGHT1: i32 = 105;
const ROCKATT1: i32 = 107;
const SHOTATT1: i32 = 113;
const AXATT1: i32 = 119;
const AXATTB1: i32 = 125;
const AXATTC1: i32 = 131;
const AXATTD1: i32 = 137;

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
            self.entities[e].walkframe = 0;
            self.player_run(e);
            return;
        }

        let ent = &mut self.entities[e];
        if ent.v.weapon == Items::AXE.as_f32() {
            if ent.walkframe >= 12 {
                ent.walkframe = 0;
            }
            ent.v.frame = (AXSTND1 + ent.walkframe) as f32;
        } else {
            if ent.walkframe >= 5 {
                ent.walkframe = 0;
            }
            ent.v.frame = (STAND1 + ent.walkframe) as f32;
        }
        ent.walkframe += 1;
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
            self.entities[e].walkframe = 0;
            self.player_stand1(e);
            return;
        }

        let ent = &mut self.entities[e];
        let base = if ent.v.weapon == Items::AXE.as_f32() { AXRUN1 } else { ROCKRUN1 };
        if ent.walkframe == 6 {
            ent.walkframe = 0;
        }
        ent.v.frame = (base + ent.walkframe) as f32;
        ent.walkframe += 1;
    }

    // --- weapon firing animations (driven by W_Attack) ---

    /// Begin a cosmetic weapon animation: `walkframe` is the cursor, `anim_*` the parameters.
    fn start_weapon_anim(&mut self, e: EntId, base: i32, wf_base: i32, len: i32, fire: i32, muzzle: i32) {
        {
            let ent = &mut self.entities[e];
            ent.walkframe = 0;
            ent.anim_base = base;
            ent.anim_wf_base = wf_base;
            ent.anim_len = len;
            ent.anim_fire = fire;
            ent.anim_muzzle = muzzle;
            ent.think = Think::PlayerWeaponAnim;
        }
        // Run the first frame immediately (as the QuakeC `player_*1` body does).
        self.player_weapon_anim(e);
    }

    pub(crate) fn start_shot_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, SHOTATT1, 1, 6, -1, 0);
    }

    pub(crate) fn start_rocket_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, ROCKATT1, 1, 6, -1, 0);
    }

    pub(crate) fn start_axe_anim(&mut self, e: EntId) {
        // Four cosmetic variants; all fire the axe on the third frame.
        let (base, wf_base) = match (self.random() * 4.0) as i32 {
            0 => (AXATT1, 1),
            1 => (AXATTB1, 5),
            2 => (AXATTC1, 1),
            _ => (AXATTD1, 5),
        };
        self.start_weapon_anim(e, base, wf_base, 4, 2, -1);
    }

    /// `PlayerWeaponAnim` think — advance one cosmetic weapon frame.
    pub(crate) fn player_weapon_anim(&mut self, e: EntId) {
        let (wf, base, wf_base, len, fire, muzzle) = {
            let ent = &self.entities[e];
            (ent.walkframe, ent.anim_base, ent.anim_wf_base, ent.anim_len, ent.anim_fire, ent.anim_muzzle)
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
        ent.walkframe = wf + 1;
        ent.think = Think::PlayerWeaponAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// Begin the looping nailgun fire.
    pub(crate) fn start_nail(&mut self, e: EntId) {
        self.entities[e].walkframe = 0;
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
        let parity = self.entities[e].walkframe & 1;
        let dir = if parity == 0 { 4.0 } else { -4.0 };
        self.w_fire_spikes(e, dir);
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.attack_finished = time + 0.2;
        ent.v.frame = (NAILATT1 + parity) as f32;
        ent.walkframe = parity ^ 1;
        ent.think = Think::PlayerNail;
        ent.v.nextthink = time + 0.1;
    }

    /// Begin the looping lightning fire.
    pub(crate) fn start_light(&mut self, e: EntId) {
        self.entities[e].walkframe = 0;
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
        let parity = self.entities[e].walkframe & 1;
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.attack_finished = time + 0.2;
        ent.v.frame = (LIGHT1 + parity) as f32;
        ent.walkframe = parity ^ 1;
        ent.think = Think::PlayerLight;
        ent.v.nextthink = time + 0.1;
    }

    // --- pain & death animations ---

    /// Start a one-shot body animation from `first` to `last`, then run `after`.
    fn start_body_anim(&mut self, e: EntId, first: i32, last: i32, after: Think) {
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.frame = first as f32;
        ent.anim_end = last;
        ent.anim_after = after;
        ent.think = Think::PlayerAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// `PlayerAnim` think — advance a one-shot body animation one frame.
    pub(crate) fn player_anim_tick(&mut self, e: EntId) {
        let (frame, end, after) = {
            let ent = &self.entities[e];
            (ent.v.frame as i32, ent.anim_end, ent.anim_after)
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
            (ent.v.weaponframe, ent.invisible_finished, ent.v.weapon)
        };
        if weaponframe != 0.0 || invisible > time {
            return;
        }
        self.entities[e].v.weaponframe = 0.0;
        self.pain_sound(e);
        if weapon == Items::AXE.as_f32() {
            self.start_body_anim(e, AXPAIN1, AXPAIN1 + 5, Think::PlayerRun);
        } else {
            self.start_body_anim(e, PAIN1, PAIN1 + 5, Think::PlayerRun);
        }
    }

    /// `PainSound` — context-sensitive pain/drown/burn vocalisation.
    fn pain_sound(&mut self, e: EntId) {
        let time = self.time();
        let (health, watertype, waterlevel, pain_finished, axhitme) = {
            let ent = &self.entities[e];
            (ent.v.health, ent.v.watertype, ent.v.waterlevel, ent.pain_finished, ent.axhitme)
        };
        if health < 0.0 {
            return;
        }
        if self.entities[self.damage_attacker].classname() == Some("teledeath") {
            self.host
                .sound(e.0 as i32, Channel::Voice, c"player/teledth1.wav", 1.0, Attenuation::None);
            return;
        }
        if watertype == Content::Water.as_f32() && waterlevel == 3.0 {
            self.death_bubbles(e, 1.0);
            let s = if self.random() > 0.5 { c"player/drown1.wav" } else { c"player/drown2.wav" };
            self.host.sound(e.0 as i32, Channel::Voice, s, 1.0, Attenuation::Norm);
            return;
        }
        if watertype == Content::Slime.as_f32() || watertype == Content::Lava.as_f32() {
            let s = if self.random() > 0.5 { c"player/lburn1.wav" } else { c"player/lburn2.wav" };
            self.host.sound(e.0 as i32, Channel::Voice, s, 1.0, Attenuation::Norm);
            return;
        }
        if pain_finished > time {
            self.entities[e].axhitme = 0.0;
            return;
        }
        self.entities[e].pain_finished = time + 0.5;
        if axhitme == 1.0 {
            self.entities[e].axhitme = 0.0;
            self.host
                .sound(e.0 as i32, Channel::Voice, c"player/axhit1.wav", 1.0, Attenuation::Norm);
            return;
        }
        let rs = (self.random() * 5.0).round() as i32 + 1;
        let noise = match rs {
            1 => c"player/pain1.wav",
            2 => c"player/pain2.wav",
            3 => c"player/pain3.wav",
            4 => c"player/pain4.wav",
            5 => c"player/pain5.wav",
            _ => c"player/pain6.wav",
        };
        self.host.sound(e.0 as i32, Channel::Voice, noise, 1.0, Attenuation::Norm);
    }

    /// `DeathSound`.
    fn death_sound(&mut self, e: EntId) {
        if self.entities[e].v.waterlevel == 3.0 {
            self.death_bubbles(e, 5.0);
            self.host
                .sound(e.0 as i32, Channel::Voice, c"player/h2odeath.wav", 1.0, Attenuation::None);
            return;
        }
        let rs = (self.random() * 4.0).round() as i32 + 1;
        let noise = match rs {
            1 => c"player/death1.wav",
            2 => c"player/death2.wav",
            3 => c"player/death3.wav",
            4 => c"player/death4.wav",
            _ => c"player/death5.wav",
        };
        self.host.sound(e.0 as i32, Channel::Voice, noise, 1.0, Attenuation::None);
    }

    /// `PlayerDie` (`th_die`) — drop loot, start the death animation or gib.
    pub(crate) fn player_die(&mut self, e: EntId) {
        let time = self.time();
        {
            let ent = &mut self.entities[e];
            ent.v.items = ent.v.items.without(Items::INVISIBILITY);
            ent.invisible_finished = 0.0;
            ent.invincible_finished = 0.0;
            ent.super_damage_finished = 0.0;
            ent.radsuit_finished = 0.0;
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
        if self.entities[e].v.weapon == Items::AXE.as_f32() {
            self.start_body_anim(e, AXDETH1, AXDETH1 + 8, Think::PlayerDead);
            return;
        }
        let _ = time;
        let pick = 1 + (self.random() * 6.0).floor() as i32;
        let (first, count) = match pick {
            1 => (DEATHA1, 11),
            2 => (DEATHB1, 9),
            3 => (DEATHC1, 15),
            4 => (DEATHD1, 9),
            _ => (DEATHE1, 9),
        };
        self.start_body_anim(e, first, first + count - 1, Think::PlayerDead);
    }

    /// `set_suicide_frame` — freeze a fresh corpse (kill/disconnect), unless already gibbed.
    pub(crate) fn set_suicide_frame(&mut self, e: EntId) {
        if self.entities[e].model.as_deref() != Some("progs/player.mdl") {
            return;
        }
        let ent = &mut self.entities[e];
        ent.v.frame = DEATHA11 as f32;
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
    fn throw_gib(&mut self, e: EntId, gibname: &CStr, dm: f32) {
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
        self.host.set_model(g.0 as i32, gibname);
        self.host.set_size(g.0 as i32, Vec3::ZERO, Vec3::ZERO);
        self.host.set_origin(g.0 as i32, origin);
    }

    /// `ThrowHead` — turn the player entity itself into a flying head gib.
    fn throw_head(&mut self, e: EntId, gibname: &CStr, dm: f32) {
        let vel = self.velocity_for_damage(e, dm);
        let avel = self.rng_unit() * Vec3::new(0.0, 600.0, 0.0);
        self.host.set_model(e.0 as i32, gibname);
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
        self.host.set_size(e.0 as i32, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        let mut origin = self.entities[e].v.origin;
        origin.z -= 24.0;
        self.host.set_origin(e.0 as i32, origin);
        self.entities[e].v.origin = origin;
    }

    /// `GibPlayer`.
    fn gib_player(&mut self, e: EntId) {
        let health = self.entities[e].v.health;
        self.throw_head(e, c"progs/h_player.mdl", health);
        self.throw_gib(e, c"progs/gib1.mdl", health);
        self.throw_gib(e, c"progs/gib2.mdl", health);
        self.throw_gib(e, c"progs/gib3.mdl", health);
        self.entities[e].v.deadflag = DeadFlag::Dead.as_f32();
        if self.entities[self.damage_attacker].classname() == Some("teledeath") {
            self.host
                .sound(e.0 as i32, Channel::Voice, c"player/teledth1.wav", 1.0, Attenuation::None);
            return;
        }
        let s = if self.random() < 0.5 { c"player/gib.wav" } else { c"player/udeath.wav" };
        self.host.sound(e.0 as i32, Channel::Voice, s, 1.0, Attenuation::None);
    }

    /// A random value in `[-1, 1)` (QuakeC `crandom`).
    fn rng_unit(&mut self) -> f32 {
        2.0 * (self.random() - 0.5)
    }

    /// `DeathBubbles` — air bubbles when dying underwater. Cosmetic; the bubble-spawner
    /// chain is omitted for now (the death/drown sounds still play).
    fn death_bubbles(&mut self, _e: EntId, _count: f32) {}
}
