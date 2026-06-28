// SPDX-License-Identifier: AGPL-3.0-or-later

//! `func_plat` and `func_train`, ported from `qw-qc/plats.qc`. Plats raise when a player
//! stands on their inner trigger; trains follow `path_corner` waypoints.

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{
    Blocked, EntId, Think, Touch, Use, STATE_BOTTOM, STATE_DOWN, STATE_TOP, STATE_UP,
};
use crate::game::GameState;

impl GameState {
    // --- plats ---

    fn plat_sound(&mut self, e: EntId, no_phs: bool, which: i32) {
        let noise = if which == 0 {
            self.entities[e].noise
        } else {
            self.entities[e].noise1
        };
        if let Some(noise) = noise {
            if no_phs {
                self.host.sound_no_phs(e, Channel::Voice, noise, 1.0, Attenuation::Norm);
            } else {
                self.host.sound(e, Channel::Voice, noise, 1.0, Attenuation::Norm);
            }
        }
    }

    /// `plat_hit_top`.
    pub(crate) fn plat_hit_top(&mut self, e: EntId) {
        self.plat_sound(e, true, 1);
        let ltime = self.entities[e].v.ltime;
        let ent = &mut self.entities[e];
        ent.mover.state = STATE_TOP;
        ent.think = Think::PlatGoDown;
        ent.v.nextthink = ltime + 3.0;
    }

    /// `plat_hit_bottom`.
    pub(crate) fn plat_hit_bottom(&mut self, e: EntId) {
        self.plat_sound(e, true, 1);
        self.entities[e].mover.state = STATE_BOTTOM;
    }

    /// `plat_go_down`.
    pub(crate) fn plat_go_down(&mut self, e: EntId) {
        self.plat_sound(e, false, 0);
        self.entities[e].mover.state = STATE_DOWN;
        let (pos2, speed) = {
            let v = &self.entities[e];
            (v.mover.pos2, v.mover.speed)
        };
        self.sub_calc_move(e, pos2, speed, Think::PlatHitBottom);
    }

    /// `plat_go_up`.
    pub(crate) fn plat_go_up(&mut self, e: EntId) {
        self.plat_sound(e, false, 0);
        self.entities[e].mover.state = STATE_UP;
        let (pos1, speed) = {
            let v = &self.entities[e];
            (v.mover.pos1, v.mover.speed)
        };
        self.sub_calc_move(e, pos1, speed, Think::PlatHitTop);
    }

    /// `plat_center_touch` — a player on the inner trigger raises/holds the plat.
    pub(crate) fn plat_center_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].classname() != Some("player")
            || self.entities[other].v.health <= 0.0
        {
            return;
        }
        let plat = self.entities[e].enemy();
        let state = self.entities[plat].mover.state;
        if state == STATE_BOTTOM {
            self.plat_go_up(plat);
        } else if state == STATE_TOP {
            let ltime = self.entities[plat].v.ltime;
            self.entities[plat].v.nextthink = ltime + 1.0;
        }
    }

    /// `plat_trigger_use` — external trigger lowers an idle plat.
    pub(crate) fn plat_trigger_use(&mut self, e: EntId) {
        if self.entities[e].think != Think::None {
            return; // already activated
        }
        self.plat_go_down(e);
    }

    /// `plat_use` — targeted plat: drop on first use.
    pub(crate) fn plat_use(&mut self, e: EntId) {
        self.entities[e].use_ = Use::None;
        self.plat_go_down(e);
    }

    /// `plat_crush` (`blocked`).
    pub(crate) fn plat_crush(&mut self, e: EntId, other: EntId) {
        self.entities[other].deathtype = Some("squish".into());
        self.t_damage(other, e, e, 1.0);
        let state = self.entities[e].mover.state;
        if state == STATE_UP {
            self.plat_go_down(e);
        } else if state == STATE_DOWN {
            self.plat_go_up(e);
        }
    }

    /// `plat_spawn_inside_trigger`.
    fn plat_spawn_inside_trigger(&mut self, plat: EntId) {
        let (mins, maxs, size, pos1, pos2, spawnflags) = {
            let v = &self.entities[plat];
            (v.v.mins, v.v.maxs, v.v.size, v.mover.pos1, v.mover.pos2, v.v.spawnflags)
        };
        let t = self.spawn();
        {
            let trig = &mut self.entities[t];
            trig.set_touch(Touch::PlatCenter);
            trig.v.movetype = MoveType::None;
            trig.v.solid = Solid::Trigger;
            trig.set_enemy(plat);
        }
        let mut tmin = mins + Vec3::new(25.0, 25.0, 0.0);
        let mut tmax = maxs - Vec3::new(25.0, 25.0, -8.0);
        tmin.z = tmax.z - (pos1.z - pos2.z + 8.0);
        if spawnflags.has(PlatFlags::LOW_TRIGGER) {
            tmax.z = tmin.z + 8.0;
        }
        if size.x <= 50.0 {
            tmin.x = (mins.x + maxs.x) / 2.0;
            tmax.x = tmin.x + 1.0;
        }
        if size.y <= 50.0 {
            tmin.y = (mins.y + maxs.y) / 2.0;
            tmax.y = tmin.y + 1.0;
        }
        self.host.set_size(t, tmin, tmax);
    }

    /// `func_plat` spawn.
    pub(crate) fn spawn_func_plat(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            if ent.mover.t_length == 0.0 {
                ent.mover.t_length = 80.0;
            }
            if ent.mover.t_width == 0.0 {
                ent.mover.t_width = 10.0;
            }
            if ent.v.sounds == 0.0 {
                ent.v.sounds = 2.0;
            }
        }
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.entities[e].noise = Some(Sound::PLATS_PLAT1);
                self.entities[e].noise1 = Some(Sound::PLATS_PLAT2);
            }
            _ => {
                self.entities[e].noise = Some(Sound::PLATS_MEDPLAT1);
                self.entities[e].noise1 = Some(Sound::PLATS_MEDPLAT2);
            }
        }

        {
            let ent = &mut self.entities[e];
            ent.mover.mangle = ent.v.angles;
            ent.v.angles = Vec3::ZERO;
            ent.classname = Some("plat".into());
            ent.v.solid = Solid::Bsp;
            ent.v.movetype = MoveType::Push;
        }
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e, origin);
        self.set_brush_model(e);
        let (mins, maxs) = {
            let v = &self.entities[e].v;
            (v.mins, v.maxs)
        };
        self.host.set_size(e, mins, maxs);

        self.entities[e].set_blocked(Blocked::PlatBlocked);
        {
            let ent = &mut self.entities[e];
            if ent.mover.speed == 0.0 {
                ent.mover.speed = 150.0;
            }
            ent.mover.pos1 = ent.v.origin;
            ent.mover.pos2 = ent.v.origin;
            if ent.mover.height != 0.0 {
                ent.mover.pos2.z = ent.v.origin.z - ent.mover.height;
            } else {
                ent.mover.pos2.z = ent.v.origin.z - ent.v.size.z + 8.0;
            }
            ent.use_ = Use::PlatTrigger;
        }

        self.plat_spawn_inside_trigger(e);

        if self.entities[e].targetname.is_some() {
            let ent = &mut self.entities[e];
            ent.mover.state = STATE_UP;
            ent.use_ = Use::PlatUse;
        } else {
            let pos2 = self.entities[e].mover.pos2;
            self.host.set_origin(e, pos2);
            let ent = &mut self.entities[e];
            ent.v.origin = pos2;
            ent.mover.state = STATE_BOTTOM;
        }
        true
    }

    // --- trains ---

    /// `train_blocked`.
    pub(crate) fn train_blocked(&mut self, e: EntId, other: EntId) {
        let time = self.time();
        if time < self.entities[e].combat.attack_finished {
            return;
        }
        self.entities[e].combat.attack_finished = time + 0.5;
        let dmg = self.entities[e].mover.dmg;
        self.entities[other].deathtype = Some("squish".into());
        self.t_damage(other, e, e, dmg);
    }

    /// `train_use`.
    pub(crate) fn train_use(&mut self, e: EntId) {
        if self.entities[e].think != Think::FuncTrainFind {
            return;
        }
        self.train_next(e);
    }

    /// `train_wait`.
    pub(crate) fn train_wait(&mut self, e: EntId) {
        let (wait, ltime) = {
            let v = &self.entities[e];
            (v.mover.wait, v.v.ltime)
        };
        if wait != 0.0 {
            self.entities[e].v.nextthink = ltime + wait;
            self.plat_sound(e, true, 0);
        } else {
            self.entities[e].v.nextthink = ltime + 0.1;
        }
        self.entities[e].think = Think::TrainNext;
    }

    /// `train_next`.
    pub(crate) fn train_next(&mut self, e: EntId) {
        let target = match self.entities[e].target.clone() {
            Some(t) => t,
            None => return,
        };
        let targ = match self.find_by_targetname(&target).next() {
            Some(t) => t,
            None => return,
        };
        let (next_target, targ_wait, targ_origin) = {
            let t = &self.entities[targ];
            (t.target.clone(), t.mover.wait, t.v.origin)
        };
        self.entities[e].target = next_target;
        self.entities[e].mover.wait = targ_wait; // 0 if none
        self.plat_sound(e, false, 1);
        let (mins, speed) = {
            let v = &self.entities[e];
            (v.v.mins, v.mover.speed)
        };
        self.sub_calc_move(e, targ_origin - mins, speed, Think::TrainWait);
    }

    /// `func_train_find`.
    pub(crate) fn func_train_find(&mut self, e: EntId) {
        let target = match self.entities[e].target.clone() {
            Some(t) => t,
            None => return,
        };
        let targ = match self.find_by_targetname(&target).next() {
            Some(t) => t,
            None => return,
        };
        let (next_target, targ_origin) = {
            let t = &self.entities[targ];
            (t.target.clone(), t.v.origin)
        };
        self.entities[e].target = next_target;
        let mins = self.entities[e].v.mins;
        self.host.set_origin(e, targ_origin - mins);
        self.entities[e].v.origin = targ_origin - mins;
        if self.entities[e].targetname.is_none() {
            let ltime = self.entities[e].v.ltime;
            let ent = &mut self.entities[e];
            ent.v.nextthink = ltime + 0.1;
            ent.think = Think::TrainNext;
        }
    }

    /// `func_train` spawn.
    pub(crate) fn spawn_func_train(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            if ent.mover.speed == 0.0 {
                ent.mover.speed = 100.0;
            }
            if ent.mover.dmg == 0.0 {
                ent.mover.dmg = 2.0;
            }
        }
        if self.entities[e].target.is_none() {
            return false;
        }
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.entities[e].noise = Some(Sound::PLATS_TRAIN2);
                self.entities[e].noise1 = Some(Sound::PLATS_TRAIN1);
            }
            _ => {
                self.entities[e].noise = Some(Sound::MISC_NULL);
                self.entities[e].noise1 = Some(Sound::MISC_NULL);
            }
        }
        {
            let ent = &mut self.entities[e];
            ent.mover.cnt = 1.0;
            ent.v.solid = Solid::Bsp;
            ent.v.movetype = MoveType::Push;
            ent.set_blocked(Blocked::TrainBlocked);
            ent.use_ = Use::TrainUse;
            ent.classname = Some("train".into());
        }
        self.set_brush_model(e);
        let (mins, maxs, origin) = {
            let v = &self.entities[e].v;
            (v.mins, v.maxs, v.origin)
        };
        self.host.set_size(e, mins, maxs);
        self.host.set_origin(e, origin);
        let ltime = self.entities[e].v.ltime;
        let ent = &mut self.entities[e];
        ent.v.nextthink = ltime + 0.1;
        ent.think = Think::FuncTrainFind;
        true
    }
}
