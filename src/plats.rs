//! `func_plat` and `func_train`, ported from `qw-qc/plats.qc`. Plats raise when a player
//! stands on their inner trigger; trains follow `path_corner` waypoints.

use glam::Vec3;

use crate::defs::*;
use crate::entity::{
    Blocked, EntId, Think, Touch, Use, STATE_BOTTOM, STATE_DOWN, STATE_TOP, STATE_UP,
};
use crate::game::GameState;

impl GameState {
    // --- plats ---

    fn plat_sound(&mut self, e: EntId, no_phs: bool, which: i32) {
        let noise = if which == 0 {
            self.entities[e].noise.clone()
        } else {
            self.entities[e].noise1.clone()
        };
        if let Some(noise) = noise {
            let c = crate::game::cstring(&noise);
            let ent = e.0 as i32;
            if no_phs {
                self.host.sound_no_phs(ent, Channel::Voice, &c, 1.0, Attenuation::Norm);
            } else {
                self.host.sound(ent, Channel::Voice, &c, 1.0, Attenuation::Norm);
            }
        }
    }

    /// `plat_hit_top`.
    pub(crate) fn plat_hit_top(&mut self, e: EntId) {
        self.plat_sound(e, true, 1);
        let ltime = self.entities[e].v.ltime;
        let ent = &mut self.entities[e];
        ent.state = STATE_TOP;
        ent.think = Think::PlatGoDown;
        ent.v.nextthink = ltime + 3.0;
    }

    /// `plat_hit_bottom`.
    pub(crate) fn plat_hit_bottom(&mut self, e: EntId) {
        self.plat_sound(e, true, 1);
        self.entities[e].state = STATE_BOTTOM;
    }

    /// `plat_go_down`.
    pub(crate) fn plat_go_down(&mut self, e: EntId) {
        self.plat_sound(e, false, 0);
        self.entities[e].state = STATE_DOWN;
        let (pos2, speed) = {
            let v = &self.entities[e];
            (v.pos2, v.speed)
        };
        self.sub_calc_move(e, pos2, speed, Think::PlatHitBottom);
    }

    /// `plat_go_up`.
    pub(crate) fn plat_go_up(&mut self, e: EntId) {
        self.plat_sound(e, false, 0);
        self.entities[e].state = STATE_UP;
        let (pos1, speed) = {
            let v = &self.entities[e];
            (v.pos1, v.speed)
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
        let state = self.entities[plat].state;
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
        let state = self.entities[e].state;
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
            (v.v.mins, v.v.maxs, v.v.size, v.pos1, v.pos2, v.v.spawnflags)
        };
        let t = self.spawn();
        {
            let trig = &mut self.entities[t];
            trig.touch = Touch::PlatCenter;
            trig.v.movetype = MoveType::None.as_f32();
            trig.v.solid = Solid::Trigger.as_f32();
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
        self.host.set_size(t.0 as i32, tmin, tmax);
    }

    /// `func_plat` spawn.
    pub(crate) fn spawn_func_plat(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            if ent.t_length == 0.0 {
                ent.t_length = 80.0;
            }
            if ent.t_width == 0.0 {
                ent.t_width = 10.0;
            }
            if ent.v.sounds == 0.0 {
                ent.v.sounds = 2.0;
            }
        }
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.host.precache_sound(c"plats/plat1.wav");
                self.host.precache_sound(c"plats/plat2.wav");
                self.entities[e].noise = Some("plats/plat1.wav".into());
                self.entities[e].noise1 = Some("plats/plat2.wav".into());
            }
            _ => {
                self.host.precache_sound(c"plats/medplat1.wav");
                self.host.precache_sound(c"plats/medplat2.wav");
                self.entities[e].noise = Some("plats/medplat1.wav".into());
                self.entities[e].noise1 = Some("plats/medplat2.wav".into());
            }
        }

        {
            let ent = &mut self.entities[e];
            ent.mangle = ent.v.angles;
            ent.v.angles = Vec3::ZERO;
            ent.classname = Some("plat".into());
            ent.v.solid = Solid::Bsp.as_f32();
            ent.v.movetype = MoveType::Push.as_f32();
        }
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e.0 as i32, origin);
        self.set_brush_model(e);
        let (mins, maxs) = {
            let v = &self.entities[e].v;
            (v.mins, v.maxs)
        };
        self.host.set_size(e.0 as i32, mins, maxs);

        self.entities[e].blocked = Blocked::PlatBlocked;
        {
            let ent = &mut self.entities[e];
            if ent.speed == 0.0 {
                ent.speed = 150.0;
            }
            ent.pos1 = ent.v.origin;
            ent.pos2 = ent.v.origin;
            if ent.height != 0.0 {
                ent.pos2.z = ent.v.origin.z - ent.height;
            } else {
                ent.pos2.z = ent.v.origin.z - ent.v.size.z + 8.0;
            }
            ent.use_ = Use::PlatTrigger;
        }

        self.plat_spawn_inside_trigger(e);

        if self.entities[e].targetname.is_some() {
            let ent = &mut self.entities[e];
            ent.state = STATE_UP;
            ent.use_ = Use::PlatUse;
        } else {
            let pos2 = self.entities[e].pos2;
            self.host.set_origin(e.0 as i32, pos2);
            let ent = &mut self.entities[e];
            ent.v.origin = pos2;
            ent.state = STATE_BOTTOM;
        }
        true
    }

    // --- trains ---

    /// `train_blocked`.
    pub(crate) fn train_blocked(&mut self, e: EntId, other: EntId) {
        let time = self.time();
        if time < self.entities[e].attack_finished {
            return;
        }
        self.entities[e].attack_finished = time + 0.5;
        let dmg = self.entities[e].dmg;
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
            (v.wait, v.v.ltime)
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
            (t.target.clone(), t.wait, t.v.origin)
        };
        self.entities[e].target = next_target;
        self.entities[e].wait = targ_wait; // 0 if none
        self.plat_sound(e, false, 1);
        let (mins, speed) = {
            let v = &self.entities[e];
            (v.v.mins, v.speed)
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
        self.host.set_origin(e.0 as i32, targ_origin - mins);
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
            if ent.speed == 0.0 {
                ent.speed = 100.0;
            }
            if ent.dmg == 0.0 {
                ent.dmg = 2.0;
            }
        }
        if self.entities[e].target.is_none() {
            return false;
        }
        match self.entities[e].v.sounds as i32 {
            1 => {
                self.host.precache_sound(c"plats/train2.wav");
                self.host.precache_sound(c"plats/train1.wav");
                self.entities[e].noise = Some("plats/train2.wav".into());
                self.entities[e].noise1 = Some("plats/train1.wav".into());
            }
            _ => {
                self.host.precache_sound(c"misc/null.wav");
                self.entities[e].noise = Some("misc/null.wav".into());
                self.entities[e].noise1 = Some("misc/null.wav".into());
            }
        }
        {
            let ent = &mut self.entities[e];
            ent.cnt = 1.0;
            ent.v.solid = Solid::Bsp.as_f32();
            ent.v.movetype = MoveType::Push.as_f32();
            ent.blocked = Blocked::TrainBlocked;
            ent.use_ = Use::TrainUse;
            ent.classname = Some("train".into());
        }
        self.set_brush_model(e);
        let (mins, maxs, origin) = {
            let v = &self.entities[e].v;
            (v.mins, v.maxs, v.origin)
        };
        self.host.set_size(e.0 as i32, mins, maxs);
        self.host.set_origin(e.0 as i32, origin);
        let ltime = self.entities[e].v.ltime;
        let ent = &mut self.entities[e];
        ent.v.nextthink = ltime + 0.1;
        ent.think = Think::FuncTrainFind;
        true
    }
}
