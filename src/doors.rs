//! `func_door`, ported from `qw-qc/doors.qc`. Touching doors auto-link into a group with a
//! shared trigger field; the group opens and closes together. (Secret doors are spawned as
//! static brushes — they keep the wall in place without the full slide sequence.)

use core::ffi::CStr;

use glam::Vec3;

use crate::defs::*;
use crate::entity::{
    Blocked, Die, EntId, Think, Touch, Use, STATE_BOTTOM, STATE_DOWN, STATE_TOP, STATE_UP,
};
use crate::game::GameState;

impl GameState {
    // --- think functions ---

    /// `door_blocked`.
    pub(crate) fn door_blocked(&mut self, e: EntId, other: EntId) {
        let (goal, dmg, wait, state) = {
            let v = &self.entities[e];
            (v.goalentity(), v.mover.dmg, v.mover.wait, v.mover.state)
        };
        self.entities[other].deathtype = Some("squish".into());
        self.t_damage(other, e, goal, dmg);
        if wait >= 0.0 {
            if state == STATE_DOWN {
                self.door_go_up(e);
            } else {
                self.door_go_down(e);
            }
        }
    }

    /// `door_hit_top`.
    pub(crate) fn door_hit_top(&mut self, e: EntId) {
        self.play_door(e, true, 1);
        self.entities[e].mover.state = STATE_TOP;
        if self.entities[e].v.spawnflags.has(DoorFlags::TOGGLE) {
            return;
        }
        let ltime = self.entities[e].v.ltime;
        let wait = self.entities[e].mover.wait;
        let ent = &mut self.entities[e];
        ent.think = Think::DoorGoDown;
        ent.v.nextthink = ltime + wait;
    }

    /// `door_hit_bottom`.
    pub(crate) fn door_hit_bottom(&mut self, e: EntId) {
        self.play_door(e, true, 1);
        self.entities[e].mover.state = STATE_BOTTOM;
    }

    /// `door_go_down`.
    pub(crate) fn door_go_down(&mut self, e: EntId) {
        self.play_door(e, false, 2);
        {
            let ent = &mut self.entities[e];
            if ent.v.max_health != 0.0 {
                ent.v.takedamage = TakeDamage::Yes.as_f32();
                ent.v.health = ent.v.max_health;
            }
            ent.mover.state = STATE_DOWN;
        }
        let (pos1, speed) = {
            let v = &self.entities[e];
            (v.mover.pos1, v.mover.speed)
        };
        self.sub_calc_move(e, pos1, speed, Think::DoorHitBottom);
    }

    /// `door_go_up`.
    pub(crate) fn door_go_up(&mut self, e: EntId) {
        let (state, ltime, wait) = {
            let v = &self.entities[e];
            (v.mover.state, v.v.ltime, v.mover.wait)
        };
        if state == STATE_UP {
            return;
        }
        if state == STATE_TOP {
            self.entities[e].v.nextthink = ltime + wait;
            return;
        }
        self.play_door(e, false, 2);
        self.entities[e].mover.state = STATE_UP;
        let (pos2, speed) = {
            let v = &self.entities[e];
            (v.mover.pos2, v.mover.speed)
        };
        self.sub_calc_move(e, pos2, speed, Think::DoorHitTop);
        self.sub_use_targets(e);
    }

    // --- activation ---

    /// `door_fire` — open (or, when toggled, close) every door in the linked group.
    fn door_fire(&mut self, master: EntId) {
        if self.entities[master].v.items != 0.0 {
            self.play_door(master, false, 4);
        }
        self.entities[master].message = None;

        let toggled = self.entities[master].v.spawnflags.has(DoorFlags::TOGGLE);
        if toggled {
            let state = self.entities[master].mover.state;
            if state == STATE_UP || state == STATE_TOP {
                let mut cur = master;
                loop {
                    self.door_go_down(cur);
                    cur = self.entities[cur].enemy();
                    if cur == master || cur == EntId::WORLD {
                        break;
                    }
                }
                return;
            }
        }

        let activator = self.activator;
        let mut cur = master;
        loop {
            self.entities[cur].set_goalentity(activator);
            self.door_go_up(cur);
            cur = self.entities[cur].enemy();
            if cur == master || cur == EntId::WORLD {
                break;
            }
        }
    }

    /// `door_use`.
    pub(crate) fn door_use(&mut self, e: EntId) {
        self.entities[e].message = None;
        let owner = self.entities[e].owner();
        self.entities[owner].message = None;
        let enemy = self.entities[e].enemy();
        if enemy.is_some() {
            self.entities[enemy].message = None;
        }
        self.door_fire(owner);
    }

    /// `door_trigger_touch`.
    pub(crate) fn door_trigger_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].v.health <= 0.0 {
            return;
        }
        let time = self.time();
        if time < self.entities[e].combat.attack_finished {
            return;
        }
        self.entities[e].combat.attack_finished = time + 1.0;
        self.activator = other;
        let owner = self.entities[e].owner();
        self.door_use(owner);
    }

    /// `door_killed` (`th_die`).
    pub(crate) fn door_killed(&mut self, e: EntId) {
        let owner = self.entities[e].owner();
        {
            let o = &mut self.entities[owner];
            o.v.health = o.v.max_health;
            o.v.takedamage = TakeDamage::No.as_f32();
        }
        self.door_use(owner);
    }

    /// `door_touch` — print messages / open key doors.
    pub(crate) fn door_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].classname() != Some("player") {
            return;
        }
        let owner = self.entities[e].owner();
        let time = self.time();
        if self.entities[owner].combat.attack_finished > time {
            return;
        }
        self.entities[owner].combat.attack_finished = time + 2.0;

        if self.entities[owner].message.is_some() {
            if let Some(msg) = self.message_cstring(owner) {
                self.host.centerprint(other.0 as i32, &msg);
            }
            self.host
                .sound(other.0 as i32, Channel::Voice, c"misc/talk.wav", 1.0, Attenuation::Norm);
        }

        let items = self.entities[e].v.items;
        if items == 0.0 {
            return;
        }
        let other_items = self.entities[other].v.items;
        if !other_items.has_all(items) {
            // Missing key: nag and bail.
            let owner_items = self.entities[owner].v.items;
            let wt = self.level.worldtype;
            let msg = if owner_items.is(Items::KEY1) {
                match wt as i32 {
                    2 => c"You need the silver keycard",
                    1 => c"You need the silver runekey",
                    _ => c"You need the silver key",
                }
            } else {
                match wt as i32 {
                    2 => c"You need the gold keycard",
                    1 => c"You need the gold runekey",
                    _ => c"You need the gold key",
                }
            };
            self.host.centerprint(other.0 as i32, msg);
            self.play_door(e, false, 3);
            return;
        }

        self.entities[other].v.items = other_items.without(items);
        self.entities[e].set_touch(Touch::None);
        let enemy = self.entities[e].enemy();
        if enemy.is_some() {
            self.entities[enemy].set_touch(Touch::None);
        }
        self.door_use(e);
    }

    // --- spawning ---

    /// `spawn_field` — the fat auto-open trigger volume around a door group.
    fn spawn_field(&mut self, master: EntId, fmins: Vec3, fmaxs: Vec3) -> EntId {
        let t = self.spawn();
        {
            let trig = &mut self.entities[t];
            trig.v.movetype = MoveType::None.as_f32();
            trig.v.solid = Solid::Trigger.as_f32();
            trig.set_touch(Touch::DoorTriggerField);
            trig.set_owner(master);
        }
        let margin = Vec3::new(60.0, 60.0, 8.0);
        self.host.set_size(t.0 as i32, fmins - margin, fmaxs + margin);
        t
    }

    fn entities_touching(&self, a: EntId, b: EntId) -> bool {
        let (amin, amax) = {
            let v = &self.entities[a].v;
            (v.mins, v.maxs)
        };
        let (bmin, bmax) = {
            let v = &self.entities[b].v;
            (v.mins, v.maxs)
        };
        amin.x <= bmax.x
            && amin.y <= bmax.y
            && amin.z <= bmax.z
            && amax.x >= bmin.x
            && amax.y >= bmin.y
            && amax.z >= bmin.z
    }

    /// First door (classname `"door"`) at an index greater than `after`.
    fn next_door(&self, after: EntId) -> Option<EntId> {
        (after.index() + 1..self.entities.len())
            .find(|&i| self.entities[i].in_use && self.entities[i].classname() == Some("door"))
            .map(|i| EntId(i as u32))
    }

    /// `LinkDoors` — group touching doors and (for the master) spawn the trigger field.
    pub(crate) fn door_link(&mut self, e: EntId) {
        if self.entities[e].enemy().is_some() {
            return;
        }
        if self.entities[e].v.spawnflags.has(DoorFlags::DONT_LINK) {
            self.entities[e].set_owner(e);
            self.entities[e].set_enemy(e);
            return;
        }

        let mut cmins = self.entities[e].v.mins;
        let mut cmaxs = self.entities[e].v.maxs;
        let master = e;
        let mut cur = e;
        let mut t = e;

        loop {
            self.entities[cur].set_owner(master);
            // Promote group-wide properties onto the master.
            {
                let (h, tn, msg) = {
                    let c = &self.entities[cur];
                    (c.v.health, c.targetname.clone(), c.message.clone())
                };
                let m = &mut self.entities[master];
                if h != 0.0 {
                    m.v.health = h;
                }
                if tn.is_some() {
                    m.targetname = tn;
                }
                if msg.is_some() {
                    m.message = msg;
                }
            }

            match self.next_door(t) {
                None => {
                    self.entities[cur].set_enemy(master);
                    let m = &self.entities[master];
                    if m.v.health != 0.0 || m.targetname.is_some() || m.v.items != 0.0 {
                        return;
                    }
                    let field = self.spawn_field(master, cmins, cmaxs);
                    self.entities[master].refs.trigger_field = field.0;
                    return;
                }
                Some(next) => {
                    t = next;
                    if self.entities_touching(cur, t) {
                        self.entities[cur].set_enemy(t);
                        cur = t;
                        let (tmin, tmax) = {
                            let v = &self.entities[t].v;
                            (v.mins, v.maxs)
                        };
                        cmins = cmins.min(tmin);
                        cmaxs = cmaxs.max(tmax);
                    }
                }
            }
        }
    }

    /// `func_door` spawn.
    pub(crate) fn spawn_func_door(&mut self, e: EntId) -> bool {
        self.setup_door_sounds(e);
        self.set_movedir(e);

        {
            let ent = &mut self.entities[e];
            ent.v.max_health = ent.v.health;
            ent.v.solid = Solid::Bsp.as_f32();
            ent.v.movetype = MoveType::Push.as_f32();
        }
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e.0 as i32, origin);
        self.set_brush_model(e);
        self.entities[e].classname = Some("door".into());
        self.entities[e].use_ = Use::DoorUse;
        self.entities[e].set_blocked(Blocked::DoorBlocked);

        {
            let ent = &mut self.entities[e];
            let sf = ent.v.spawnflags;
            if sf.has(DoorFlags::SILVER_KEY) {
                ent.v.items = Items::KEY1.as_f32();
            }
            if sf.has(DoorFlags::GOLD_KEY) {
                ent.v.items = Items::KEY2.as_f32();
            }
            if ent.mover.speed == 0.0 {
                ent.mover.speed = 100.0;
            }
            if ent.mover.wait == 0.0 {
                ent.mover.wait = 3.0;
            }
            if ent.mover.lip == 0.0 {
                ent.mover.lip = 8.0;
            }
            if ent.mover.dmg == 0.0 {
                ent.mover.dmg = 2.0;
            }
            ent.mover.pos1 = ent.v.origin;
            let movedir = ent.v.movedir;
            let size = ent.v.size;
            ent.mover.pos2 = ent.mover.pos1 + movedir * ((movedir.dot(size)).abs() - ent.mover.lip);
        }

        if self.entities[e].v.spawnflags.has(DoorFlags::START_OPEN) {
            let pos2 = self.entities[e].mover.pos2;
            self.host.set_origin(e.0 as i32, pos2);
            let ent = &mut self.entities[e];
            ent.v.origin = pos2;
            ent.mover.pos2 = ent.mover.pos1;
            ent.mover.pos1 = pos2;
        }

        {
            let ent = &mut self.entities[e];
            ent.mover.state = STATE_BOTTOM;
            if ent.v.health != 0.0 {
                ent.v.takedamage = TakeDamage::Yes.as_f32();
                ent.th_die = Die::DoorKilled;
            }
            if ent.v.items != 0.0 {
                ent.mover.wait = -1.0;
            }
            ent.set_touch(Touch::DoorTouch);
            // Link once all doors have spawned.
            ent.think = Think::DoorLink;
            ent.v.nextthink = ent.v.ltime + 0.1;
        }
        true
    }

    /// `func_door_secret` — spawned as a static brush (no slide sequence in this subset).
    pub(crate) fn spawn_func_door_secret(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.solid = Solid::Bsp.as_f32();
            ent.v.movetype = MoveType::Push.as_f32();
        }
        self.set_brush_model(e);
        true
    }

    // --- helpers ---

    /// Set the door's `noise1..4` from its `sounds`/worldtype, precaching as we go.
    fn setup_door_sounds(&mut self, e: EntId) {
        let wt = self.level.worldtype as i32;
        let (try_s, use_s) = match wt {
            1 => (c"doors/runetry.wav", c"doors/runeuse.wav"),
            2 => (c"doors/basetry.wav", c"doors/baseuse.wav"),
            _ => (c"doors/medtry.wav", c"doors/meduse.wav"),
        };
        self.host.precache_sound(try_s);
        self.host.precache_sound(use_s);
        self.entities[e].noise3 = Some(try_s.to_str().unwrap().into());
        self.entities[e].noise4 = Some(use_s.to_str().unwrap().into());

        let (n1, n2): (&'static CStr, &'static CStr) = match self.entities[e].v.sounds as i32
        {
            1 => (c"doors/drclos4.wav", c"doors/doormv1.wav"),
            2 => (c"doors/hydro2.wav", c"doors/hydro1.wav"),
            3 => (c"doors/stndr2.wav", c"doors/stndr1.wav"),
            4 => (c"doors/ddoor2.wav", c"doors/ddoor1.wav"),
            _ => (c"misc/null.wav", c"misc/null.wav"),
        };
        self.host.precache_sound(n1);
        self.host.precache_sound(n2);
        self.entities[e].noise1 = Some(n1.to_str().unwrap().into());
        self.entities[e].noise2 = Some(n2.to_str().unwrap().into());
    }

    /// Play the door's `noiseN` (1..4) on `chan`.
    fn play_door(&mut self, e: EntId, no_phs: bool, which: i32) {
        let noise = match which {
            1 => self.entities[e].noise1.clone(),
            2 => self.entities[e].noise2.clone(),
            3 => self.entities[e].noise3.clone(),
            _ => self.entities[e].noise4.clone(),
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
}
