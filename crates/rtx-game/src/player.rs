// SPDX-License-Identifier: AGPL-3.0-or-later

//! Player animation, ported from `qw-qc/player.qc`.
//!
//! QuakeC drives player animation as a think-chained state machine: each frame function
//! sets `self.frame`, schedules `nextthink = time + 0.1`, and points `self.think` at the
//! next function. We model the same loop with the [`Think`] enum and the engine's
//! `GAME_EDICT_THINK` callback (the engine ignores the entvars `think` funcref for native
//! modules and re-enters us whenever `nextthink` elapses).

use glam::Vec3;

use crate::anim::{frames, seq, Anim};
use crate::assets::{Model, Sound};
use crate::defs::*;
use crate::entity::{CombatState, EntId, Think};
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

// Weapon viewmodel (`v_*.mdl`) firing animations — one per distinct viewmodel sequence. Each
// viewmodel is its own model with its own frame numbering (independent of the player.mdl body
// frames above), and we only need each sequence's bounds, so these are plain [`Anim`]s rather than
// a `frames!` table that would name viewmodel frames nothing else refers to. Weapons that fire with
// the same sequence (only the model differs) share an entry, so this covers all eight weapons. The
// looped guns (`.cycle()`) repeat while fire is held; the rest play once via `start_weapon_anim`.
const VAXE_FIRE_A: Anim = seq(1, 4); // v_axe.mdl swing A — axe (random body variants 1 & 3)
const VAXE_FIRE_B: Anim = seq(5, 8); // v_axe.mdl swing B — axe (random body variants 2 & 4)
const VSHOT_FIRE: Anim = seq(1, 6); // v_shot.mdl / v_shot2.mdl — shotgun, super shotgun
const VNAIL_FIRE: Anim = seq(1, 8); // v_nail.mdl / v_nail2.mdl — nailgun, super nailgun (looped)
const VROCK_FIRE: Anim = seq(1, 6); // v_rock.mdl / v_rock2.mdl — grenade & rocket launchers
const VLIGHT_FIRE: Anim = seq(1, 4); // v_light.mdl — lightning gun (looped)

// v_star.mdl grapple viewmodel poses (not a cycle — a throw windup, then a held pose chosen by
// reel speed). Frame 0 is idle; the player_hook chain steps weaponframe through these.
const VSTAR_THROW: f32 = 2.0; // throwing the hook
const VSTAR_OUT: f32 = 3.0; // hook in flight / reeling slowly
const VSTAR_PULL: f32 = 4.0; // reeling at full speed

impl GameState {
    /// Re-arm an animation loop: schedule the next think 0.1s out and record which loop.
    fn schedule_anim(&mut self, e: EntId, think: Think) {
        let next = self.globals.time + 0.1;
        let ent = &mut self.entities[e];
        ent.think = think;
        ent.v.nextthink = next;
    }

    /// DEV (`rtx_wedge_debug`): catch the frame an *alive* animation loop (`player_run` /
    /// `player_stand1`) is first installed on a *dead* entity. That corruption leaves a dead bot
    /// cycling a stand/run frame with `deadflag` stuck below `Dead`, so `player_death_think` never
    /// runs and — rtx having no autospawn — it never respawns again. Logged once per wedge: only on
    /// the transition (a live entity animating is normal; once the think is already an alive loop the
    /// dispatch re-enters here every 0.1s — those are skipped so this can't spam). `#[track_caller]`
    /// on the two callers makes `caller` name the exact call site that installed the alive loop.
    /// On its own cvar (not `rtx_bot_debug`) so it can be enabled without the per-spawn spam.
    fn dbg_wedge_edge(&self, e: EntId, caller: &'static std::panic::Location<'static>) {
        // Hot-path bail before any cvar read: this runs on every stand/run tick of every live player.
        if self.entities[e].v.deadflag == DeadFlag::No {
            return;
        }
        let prev = self.entities[e].think;
        if matches!(prev, Think::PlayerRun | Think::PlayerStand) {
            return; // continuation, not the originating transition
        }
        if !self.host.cvar_bool(c"rtx_wedge_debug") {
            return;
        }
        let msg = crate::game::cstring(&format!(
            "rtx: WEDGE e{} bot={} deadflag={} health={:.0} frame={} prev_think={:?} installer={}:{}\n",
            e.0,
            self.entities[e].bot.is_bot as i32,
            self.entities[e].v.deadflag.as_f32() as i32,
            self.entities[e].v.health,
            self.entities[e].v.frame as i32,
            prev,
            caller.file(),
            caller.line(),
        ));
        self.host.dprint(&msg);
    }

    /// Recover from the wedge `dbg_wedge_edge` reports: a stale *alive* animation think (a weapon /
    /// nail / light / grapple hand-off) reached a *dead* corpse and is about to install the
    /// self-perpetuating `player_run`/`player_stand1` loop. That loop re-arms forever and never
    /// reaches `player_dead`, so `deadflag` stays pinned at `Dying` — and with no autospawn the bot
    /// never respawns. The death animation is already lost (the alive think clobbered it), so finalize
    /// the corpse straight to the death terminus: `player_pre_think` then routes to `player_death_think`
    /// and the respawn press is honoured. Returns whether the caller should bail out of the alive loop.
    fn finalize_if_dead(&mut self, e: EntId) -> bool {
        if self.entities[e].v.deadflag == DeadFlag::No {
            return false;
        }
        self.entities[e].v.frame = DEATHA.last() as f32; // a corpse pose, not the frozen mid-anim frame
        self.player_dead(e);
        true
    }

    /// `player_stand1` — idle loop; transitions to the run loop while moving.
    #[track_caller]
    pub(crate) fn player_stand1(&mut self, e: EntId) {
        self.dbg_wedge_edge(e, std::panic::Location::caller());
        if self.finalize_if_dead(e) {
            return;
        }
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
        let anim = if ent.v.weapon == Weapon::Axe { AXSTND } else { STAND };
        if ent.anim.walkframe >= anim.len {
            ent.anim.walkframe = 0;
        }
        ent.v.frame = anim.frame(ent.anim.walkframe);
        ent.anim.walkframe += 1;
    }

    /// `player_run` — running loop; transitions back to idle when stopped.
    #[track_caller]
    pub(crate) fn player_run(&mut self, e: EntId) {
        self.dbg_wedge_edge(e, std::panic::Location::caller());
        if self.finalize_if_dead(e) {
            return;
        }
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
        let anim = if ent.v.weapon == Weapon::Axe { AXRUN } else { ROCKRUN };
        if ent.anim.walkframe >= anim.len {
            ent.anim.walkframe = 0;
        }
        ent.v.frame = anim.frame(ent.anim.walkframe);
        ent.anim.walkframe += 1;
    }

    // --- weapon firing animations (driven by W_Attack) ---

    /// Begin a play-once weapon animation: `walkframe` is the cursor, `body` supplies the player
    /// frames (and the shared frame count), `vwep` the matching viewmodel frames, and
    /// `fire`/`muzzle` are the cursor indices at which the weapon fires / shows a muzzle flash.
    fn start_weapon_anim(&mut self, e: EntId, body: Anim, vwep: Anim, fire: i32, muzzle: i32) {
        debug_assert_eq!(
            body.len, vwep.len,
            "body and viewmodel fire animations must run the same number of frames"
        );
        {
            let ent = &mut self.entities[e];
            ent.anim.walkframe = 0;
            ent.anim.anim_base = body.first;
            ent.anim.anim_wf_base = vwep.first;
            ent.anim.anim_len = body.len;
            ent.anim.anim_fire = fire;
            ent.anim.anim_muzzle = muzzle;
            ent.think = Think::PlayerWeaponAnim;
        }
        // Run the first frame immediately (as the QuakeC `player_*1` body does).
        self.player_weapon_anim(e);
    }

    pub(crate) fn start_shot_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, SHOTATT, VSHOT_FIRE, -1, 0);
    }

    pub(crate) fn start_rocket_anim(&mut self, e: EntId) {
        self.start_weapon_anim(e, ROCKATT, VROCK_FIRE, -1, 0);
    }

    pub(crate) fn start_axe_anim(&mut self, e: EntId) {
        // Four cosmetic body variants over two viewmodel swings; all fire the axe on the third frame.
        let (body, vwep) = match (self.random() * 4.0) as i32 {
            0 => (AXATT, VAXE_FIRE_A),
            1 => (AXATTB, VAXE_FIRE_B),
            2 => (AXATTC, VAXE_FIRE_A),
            _ => (AXATTD, VAXE_FIRE_B),
        };
        self.start_weapon_anim(e, body, vwep, 2, -1);
    }

    /// `PlayerWeaponAnim` think — advance one cosmetic weapon frame.
    pub(crate) fn player_weapon_anim(&mut self, e: EntId) {
        let (wf, base, wf_base, len, fire, muzzle) = {
            let ent = &self.entities[e];
            (
                ent.anim.walkframe,
                ent.anim.anim_base,
                ent.anim.anim_wf_base,
                ent.anim.anim_len,
                ent.anim.anim_fire,
                ent.anim.anim_muzzle,
            )
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
            ent.v.weaponframe = VNAIL_FIRE.cycle(ent.v.weaponframe);
        }
        self.super_damage_sound(e);
        let parity = self.entities[e].anim.walkframe & 1;
        let dir = if parity == 0 { 4.0 } else { -4.0 };
        self.w_fire_spikes(e, dir);
        let time = self.time();
        let cd = crate::arsenal::cooldown_of(self.entities[e].v.weapon.item());
        let ent = &mut self.entities[e];
        ent.combat.attack_finished = time + cd;
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
            ent.v.weaponframe = VLIGHT_FIRE.cycle(ent.v.weaponframe);
        }
        self.super_damage_sound(e);
        self.w_fire_lightning(e);
        let parity = self.entities[e].anim.walkframe & 1;
        let time = self.time();
        let cd = crate::arsenal::cooldown_of(self.entities[e].v.weapon.item());
        let ent = &mut self.entities[e];
        ent.combat.attack_finished = time + cd;
        ent.v.frame = LIGHT.frame(parity);
        ent.anim.walkframe = parity ^ 1;
        ent.think = Think::PlayerLight;
        ent.v.nextthink = time + 0.1;
    }

    /// `player_hook1` — fire the grapple and start its viewmodel animation. A no-op while a hook is
    /// already out (the hold loop is running then), so re-fires from the held button don't restart it.
    pub(crate) fn start_grapple_throw(&mut self, e: EntId) {
        if self.entities[e].grapple.hook_out {
            return;
        }
        self.throw_grapple(e);
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.frame = AXATTD.frame(0); // body: start of the throw
        ent.v.weaponframe = VSTAR_THROW;
        ent.think = Think::GrappleAnim;
        ent.v.nextthink = time + 0.1;
    }

    /// `player_chain3`/`player_chain4` — hold the grapple pose while the hook is out, swapping the
    /// `v_star.mdl` frame by reel speed; return to the run/stand loop once the hook is gone.
    pub(crate) fn grapple_anim(&mut self, e: EntId) {
        if !self.entities[e].grapple.hook_out {
            self.player_run(e); // also clears weaponframe
            return;
        }
        let time = self.time();
        let fast = self.entities[e].v.velocity.length() >= 750.0;
        let ent = &mut self.entities[e];
        ent.v.frame = AXATTD.frame(2); // body: hold pose (axattd3)
        ent.v.weaponframe = if fast { VSTAR_PULL } else { VSTAR_OUT };
        ent.think = Think::GrappleAnim;
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
        ent.v.deadflag = DeadFlag::Dead;
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
        let anim = if weapon == Weapon::Axe { AXPAIN } else { PAIN };
        self.start_body_anim(e, anim, Think::PlayerRun);
    }

    /// `PainSound` — context-sensitive pain/drown/burn vocalisation.
    fn pain_sound(&mut self, e: EntId) {
        let time = self.time();
        let (health, watertype, waterlevel, pain_finished, axhitme) = {
            let ent = &self.entities[e];
            (
                ent.v.health,
                ent.v.watertype,
                ent.v.waterlevel,
                ent.combat.pain_finished,
                ent.combat.axhitme,
            )
        };
        if health < 0.0 {
            return;
        }
        if self.entities[e].deathtype.is_telefrag() {
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_TELEDTH1, 1.0, Attenuation::None);
            return;
        }
        if watertype.is(Content::Water) && waterlevel == 3.0 {
            self.death_bubbles(e, 1.0);
            let s = if self.random() > 0.5 {
                Sound::PLAYER_DROWN1
            } else {
                Sound::PLAYER_DROWN2
            };
            self.host.sound(e, Channel::Voice, s, 1.0, Attenuation::Norm);
            return;
        }
        if watertype.is(Content::Slime) || watertype.is(Content::Lava) {
            let s = if self.random() > 0.5 {
                Sound::PLAYER_LBURN1
            } else {
                Sound::PLAYER_LBURN2
            };
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
        let noise = pain_noise((self.random() * 5.0).round() as i32 + 1);
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
        let noise = death_noise((self.random() * 4.0).round() as i32 + 1);
        self.host.sound(e, Channel::Voice, noise, 1.0, Attenuation::None);
    }

    /// `PlayerDie` (`th_die`) — drop loot, start the death animation or gib.
    pub(crate) fn player_die(&mut self, e: EntId) {
        let time = self.time();
        {
            let ent = &mut self.entities[e];
            ent.v.items = ent.v.items.without(Items::INVISIBILITY);
            // Death ends this life's combat state: clear all of it (powerup timers and effects,
            // cooldowns, the air-jump latch, …). Nothing below reads it, and respawn re-inits it.
            ent.combat = CombatState::default();
        }
        // Drop the grappling hook if one is out (grapple state lives outside CombatState).
        if self.entities[e].grapple.hook_out {
            let hook = EntId(self.entities[e].grapple.hook);
            self.reset_grapple(hook);
        }
        self.drop_backpack(e);
        // Let the mode react to the death (CTF drops the carried flag + held runes here).
        let mode = self.mode;
        mode.player_died(self, e);
        let vz = self.entities[e].v.velocity.z;
        let zboost = if vz < 10.0 { self.rng_unit() * 300.0 } else { 0.0 };
        {
            let ent = &mut self.entities[e];
            ent.weaponmodel = None;
            ent.v.view_ofs = Vec3::new(0.0, 0.0, -8.0);
            ent.v.deadflag = DeadFlag::Dying;
            ent.v.solid = Solid::Not;
            ent.v.flags = ent.v.flags.without(Flags::ONGROUND);
            ent.v.movetype = MoveType::Toss;
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
        if self.entities[e].v.weapon == Weapon::Axe {
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
        ent.v.solid = Solid::Not;
        ent.v.movetype = MoveType::Toss;
        ent.v.deadflag = DeadFlag::Dead;
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
            v = Vec3::new(
                100.0 * self.rng_unit(),
                100.0 * self.rng_unit(),
                200.0 + 100.0 * self.random(),
            );
        }
        v * if dm > -50.0 {
            0.7
        } else if dm > -200.0 {
            2.0
        } else {
            10.0
        }
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
            gib.v.movetype = MoveType::Bounce;
            gib.v.solid = Solid::Not;
            gib.v.avelocity = avel;
            gib.think = Think::SubRemove;
            gib.v.ltime = time;
            gib.v.nextthink = nextthink;
            gib.v.frame = 0.0;
            gib.v.flags = 0.0;
        }
        self.set_model(g, gibname);
        self.set_size(g, Vec3::ZERO, Vec3::ZERO);
        self.set_origin(g, origin);
    }

    /// `ThrowHead` — turn the player entity itself into a flying head gib.
    fn throw_head(&mut self, e: EntId, gibname: Model, dm: f32) {
        let vel = self.velocity_for_damage(e, dm);
        let avel = self.rng_unit() * Vec3::new(0.0, 600.0, 0.0);
        self.set_model(e, gibname);
        {
            let ent = &mut self.entities[e];
            ent.v.frame = 0.0;
            ent.v.nextthink = -1.0;
            ent.v.movetype = MoveType::Bounce;
            ent.v.takedamage = TakeDamage::No;
            ent.v.solid = Solid::Not;
            ent.v.view_ofs = Vec3::new(0.0, 0.0, 8.0);
            ent.v.velocity = vel;
            ent.v.flags = ent.v.flags.without(Flags::ONGROUND);
            ent.v.avelocity = avel;
        }
        self.set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        let mut origin = self.entities[e].v.origin;
        origin.z -= 24.0;
        self.set_origin(e, origin);
        self.entities[e].v.origin = origin;
    }

    /// `GibPlayer`.
    fn gib_player(&mut self, e: EntId) {
        let health = self.entities[e].v.health;
        self.throw_head(e, Model::PROGS_H_PLAYER, health);
        self.throw_gib(e, Model::PROGS_GIB1, health);
        self.throw_gib(e, Model::PROGS_GIB2, health);
        self.throw_gib(e, Model::PROGS_GIB3, health);
        self.entities[e].v.deadflag = DeadFlag::Dead;
        if self.entities[e].deathtype.is_telefrag() {
            self.host
                .sound(e, Channel::Voice, Sound::PLAYER_TELEDTH1, 1.0, Attenuation::None);
            return;
        }
        let s = if self.random() < 0.5 {
            Sound::PLAYER_GIB
        } else {
            Sound::PLAYER_UDEATH
        };
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

/// The player death vocalization for a `1..=5` roll — id's `DeathSound` table (`player/death1-5`).
/// Pure so the roll→sound mapping is pinned by test, the way the obituary strings are: clients
/// select the wav by index, so a silent repermutation must not slip through a refactor.
fn death_noise(roll: i32) -> Sound {
    match roll {
        1 => Sound::PLAYER_DEATH1,
        2 => Sound::PLAYER_DEATH2,
        3 => Sound::PLAYER_DEATH3,
        4 => Sound::PLAYER_DEATH4,
        _ => Sound::PLAYER_DEATH5,
    }
}

/// The player pain vocalization for a `1..=6` roll — id's `PainSound` table (`player/pain1-6`).
/// Pure for the same reason as [`death_noise`].
fn pain_noise(roll: i32) -> Sound {
    match roll {
        1 => Sound::PLAYER_PAIN1,
        2 => Sound::PLAYER_PAIN2,
        3 => Sound::PLAYER_PAIN3,
        4 => Sound::PLAYER_PAIN4,
        5 => Sound::PLAYER_PAIN5,
        _ => Sound::PLAYER_PAIN6,
    }
}

#[cfg(test)]
mod tests {
    use super::{death_noise, pain_noise};
    use crate::assets::Sound;

    // `Sound` is an opaque precache handle (no PartialEq) — compare on its wire path.
    fn same(a: Sound, b: Sound) -> bool {
        a.path() == b.path()
    }

    // The live rolls are `(random()*4).round()+1` -> 1..=5 and `(random()*5).round()+1` -> 1..=6;
    // pin the whole table (including the out-of-range guards) so the wav indices never drift.
    #[test]
    fn death_noise_table_is_stock() {
        assert!(same(death_noise(1), Sound::PLAYER_DEATH1));
        assert!(same(death_noise(2), Sound::PLAYER_DEATH2));
        assert!(same(death_noise(3), Sound::PLAYER_DEATH3));
        assert!(same(death_noise(4), Sound::PLAYER_DEATH4));
        assert!(same(death_noise(5), Sound::PLAYER_DEATH5));
        assert!(same(death_noise(0), Sound::PLAYER_DEATH5));
    }

    #[test]
    fn pain_noise_table_is_stock() {
        assert!(same(pain_noise(1), Sound::PLAYER_PAIN1));
        assert!(same(pain_noise(2), Sound::PLAYER_PAIN2));
        assert!(same(pain_noise(3), Sound::PLAYER_PAIN3));
        assert!(same(pain_noise(4), Sound::PLAYER_PAIN4));
        assert!(same(pain_noise(5), Sound::PLAYER_PAIN5));
        assert!(same(pain_noise(6), Sound::PLAYER_PAIN6));
        assert!(same(pain_noise(0), Sound::PLAYER_PAIN6));
    }
}
