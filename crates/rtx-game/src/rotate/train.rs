// SPDX-License-Identifier: AGPL-3.0-or-later

//! `func_rotate_train` — a brush that rides a chain of `path_rotate` corners, rotating as it travels
//! (the Hipnotic rotating train). Spawn + the corner-to-corner think/find/wait/stop/next state
//! machine. Rides on the shared rotator engine (`rotate_targets` etc.) in the parent module.

use glam::Vec3;

use crate::math::normalize_angles;
use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{EntId, RotPhase, Think, Use};
use crate::game::GameState;

impl GameState {
    pub(crate) fn spawn_func_rotate_train(&mut self, e: EntId) -> bool {
        if self.entities[e].target.is_none() {
            return false; // a rotate_train needs a centre target
        }
        if self.entities[e].mover.speed == 0.0 {
            self.entities[e].mover.speed = 100.0;
        }
        // Default move/stop sounds from `sounds`, unless the map supplied its own.
        let quiet = self.entities[e].v.sounds as i32 != 1;
        if self.entities[e].noise.is_none() {
            self.entities[e].noise = Some(if quiet { Sound::MISC_NULL } else { Sound::PLATS_TRAIN2 });
        }
        if self.entities[e].noise1.is_none() {
            self.entities[e].noise1 = Some(if quiet { Sound::MISC_NULL } else { Sound::PLATS_TRAIN1 });
        }
        {
            let ent = &mut self.entities[e];
            ent.v.solid = Solid::Not;
            ent.v.movetype = MoveType::Step;
            ent.use_ = Use::RotateTrainUse;
        }
        self.link_brush(e);
        let origin = self.entities[e].v.origin;
        self.set_origin(e, origin);

        // Start on the second frame, once the corners have spawned.
        let t = self.time();
        let ent = &mut self.entities[e];
        ent.v.ltime = t;
        ent.v.nextthink = t + 0.1;
        ent.rot.endtime = t + 0.1;
        ent.think = Think::RotateTrainThink;
        ent.think1 = Think::RotateTrainFind;
        ent.rot.phase = RotPhase::None;
        ent.rot.duration = 1.0;
        ent.mover.cnt = 0.1; // interpolation start time
        ent.mover.dest2 = Vec3::ZERO; // per-segment delta
        ent.mover.dest1 = origin; // per-segment start
        ent.v.flags = ent.v.flags.with(Flags::ONGROUND); // STEP entity must not fall
        true
    }

    pub(crate) fn rotate_train_think(&mut self, e: EntId) {
        let now = self.time();
        let (ltime, endtime, phase, cnt, duration, dest1, dest2, finaldest, think1) = {
            let v = &self.entities[e];
            (
                v.v.ltime,
                v.rot.endtime,
                v.rot.phase,
                v.mover.cnt,
                v.rot.duration,
                v.mover.dest1,
                v.mover.dest2,
                v.mover.finaldest,
                v.think1,
            )
        };
        let t = now - ltime;
        self.entities[e].v.ltime = now;

        if endtime != 0.0 && now >= endtime {
            self.entities[e].rot.endtime = 0.0;
            if phase == RotPhase::Moving {
                self.place(e, finaldest);
                self.entities[e].v.velocity = Vec3::ZERO;
            }
            if think1 != Think::None {
                self.run_think_now(e, think1);
            }
        } else {
            // Interpolate along the current segment (clamped at the destination).
            let frac = ((now - cnt) * duration).min(1.0);
            self.place(e, dest1 + dest2 * frac);
        }

        // `think1` above may have set a new rotation rate, so read it fresh.
        let rotate = self.entities[e].rot.rotate;
        self.entities[e].v.angles = normalize_angles(self.entities[e].v.angles + rotate * t);
        self.rotate_targets(e);
        self.entities[e].v.nextthink = now + 0.02;
    }

    pub(crate) fn rotate_train_use(&mut self, e: EntId) {
        let (think1, moving) = {
            let v = &self.entities[e];
            (v.think1, v.v.velocity.length() != 0.0)
        };
        // Before the train has found its path it auto-starts; afterwards a use kicks it on, but
        // only if it isn't already in motion.
        if think1 != Think::RotateTrainFind && !moving && think1 != Think::None {
            self.run_think_now(e, think1);
        }
    }

    pub(crate) fn rotate_train_find(&mut self, e: EntId) {
        self.entities[e].rot.phase = RotPhase::None;
        self.link_rotate_targets(e);

        let Some(targ) = self.next_path_corner(e) else {
            return;
        };
        self.entities[e].set_goalentity(targ);

        let (angles_flag, targ_angles, targ_target, targ_origin) = {
            let tv = &self.entities[targ];
            (
                tv.v.spawnflags.has(PathRotateFlags::ANGLES),
                tv.v.angles,
                tv.target.clone(),
                tv.v.origin,
            )
        };
        if angles_flag {
            let na = normalize_angles(targ_angles);
            self.entities[e].v.angles = targ_angles;
            self.entities[targ].v.angles = na;
            self.entities[e].mover.finalangle = na;
        }
        self.entities[e].path = targ_target;
        self.place(e, targ_origin);
        self.set_target_origin(e);
        self.rotate_targets_final(e);

        let t = self.time();
        let (has_name, ltime) = {
            let v = &self.entities[e];
            (v.targetname.is_some(), v.v.ltime)
        };
        let ent = &mut self.entities[e];
        ent.think1 = Think::RotateTrainNext;
        // Untriggered trains start immediately; triggered ones wait for a use.
        ent.rot.endtime = if has_name { 0.0 } else { ltime + 0.1 };
        ent.rot.duration = 1.0;
        ent.mover.cnt = t;
        ent.mover.dest2 = Vec3::ZERO;
        ent.mover.dest1 = targ_origin;
    }

    pub(crate) fn rotate_train_wait(&mut self, e: EntId) {
        let goal = self.entities[e].goalentity();
        self.rotate_train_settle(e, goal);
        let endtime = self.entities[e].v.ltime + self.entities[goal].mover.wait;
        self.entities[e].rot.endtime = endtime;
    }

    pub(crate) fn rotate_train_stop(&mut self, e: EntId) {
        let goal = self.entities[e].goalentity();
        self.rotate_train_settle(e, goal);
        self.entities[e].mover.dmg = 0.0;
        // No `endtime` is set, so the train rests here until used again.
    }

    /// Shared arrival handling for [`rotate_train_wait`](Self::rotate_train_wait) and
    /// [`rotate_train_stop`](Self::rotate_train_stop): announce the stop, honour the corner's
    /// `ANGLES`/`NO_ROTATE` flags (snap to the turned-toward angle and/or halt the spin), and
    /// queue the next corner. The callers add only the wait time / damage reset that differs.
    fn rotate_train_settle(&mut self, e: EntId, goal: EntId) {
        self.entities[e].rot.phase = RotPhase::None;
        self.rotate_train_play_noise(e, goal);

        let (angles_flag, no_rotate) = {
            let f = &self.entities[goal].v.spawnflags;
            (f.has(PathRotateFlags::ANGLES), f.has(PathRotateFlags::NO_ROTATE))
        };
        if angles_flag {
            let finalangle = self.entities[e].mover.finalangle;
            self.entities[e].v.angles = finalangle;
        }
        if angles_flag || no_rotate {
            self.entities[e].rot.rotate = Vec3::ZERO;
        }
        self.entities[e].think1 = Think::RotateTrainNext;
    }

    pub(crate) fn rotate_train_next(&mut self, e: EntId) {
        self.entities[e].rot.phase = RotPhase::None;

        // The corner we're leaving (`current`) drives this segment; `targ` is where we head next.
        let current = self.entities[e].goalentity();
        let Some(targ) = self.next_path_corner(e) else {
            return;
        };

        // The leaving corner can override the move sound.
        if let Some(n1) = self.entities[current].noise1 {
            self.entities[e].noise1 = Some(n1);
        }
        if let Some(n) = self.entities[e].noise1 {
            self.host.sound(e, Channel::Voice, n, 1.0, Attenuation::Norm);
        }

        self.entities[e].set_goalentity(targ);
        let next_path = self.entities[targ].target.clone();
        if next_path.is_none() {
            self.host.dprint(c"rotate_train_next: corner has no next target\n");
            return;
        }
        self.entities[e].path = next_path;

        // Decide what to do on arrival at `targ`.
        let (targ_stop, targ_wait) = {
            let tv = &self.entities[targ];
            (tv.v.spawnflags.has(PathRotateFlags::STOP), tv.mover.wait)
        };
        self.entities[e].think1 = if targ_stop {
            Think::RotateTrainStop
        } else if targ_wait != 0.0 {
            Think::RotateTrainWait
        } else {
            Think::RotateTrainNext
        };

        self.fire_corner_event(e, current);

        let (cur_speed, movetime_flag) = self.apply_leaving_corner(e, current);
        self.move_to_corner(e, targ, cur_speed, movetime_flag);
    }

    /// Apply the modifiers the *leaving* corner sets for this segment — an angle snap, a new rotation
    /// vector, a damage value, and damage-on-targets — and report the corner's `(speed, movetime)`
    /// for [`Self::move_to_corner`] to solve the travel with.
    fn apply_leaving_corner(&mut self, e: EntId, current: EntId) -> (f32, bool) {
        let (angles_flag, rotation_flag, damage_flag, set_damage_flag, movetime_flag, cur_speed, cur_rotate, cur_dmg) = {
            let c = &self.entities[current];
            (
                c.v.spawnflags.has(PathRotateFlags::ANGLES),
                c.v.spawnflags.has(PathRotateFlags::ROTATION),
                c.v.spawnflags.has(PathRotateFlags::DAMAGE),
                c.v.spawnflags.has(PathRotateFlags::SET_DAMAGE),
                c.v.spawnflags.has(PathRotateFlags::MOVETIME),
                c.mover.speed,
                c.rot.rotate,
                c.mover.dmg,
            )
        };
        let finalangle = self.entities[e].mover.finalangle;
        {
            let ent = &mut self.entities[e];
            if angles_flag {
                ent.rot.rotate = Vec3::ZERO;
                ent.v.angles = finalangle;
            }
            if rotation_flag {
                ent.rot.rotate = cur_rotate;
            }
            if damage_flag {
                ent.mover.dmg = cur_dmg;
            }
        }
        if set_damage_flag {
            self.set_damage_on_targets(e, cur_dmg);
        }
        (cur_speed, movetime_flag)
    }

    /// Drive the train from its current origin toward corner `targ`: a `speed == -1` warp, an
    /// already-there idle tick, or a timed slide (a positive corner speed becomes the new cruising
    /// speed). Sets the mover dest/velocity/endtime that `rotate_train_think` steps.
    fn move_to_corner(&mut self, e: EntId, targ: EntId, cur_speed: f32, movetime_flag: bool) {
        let t = self.time();
        let (ltime, origin) = {
            let v = &self.entities[e];
            (v.v.ltime, v.v.origin)
        };
        let (targ_origin, targ_angles_flag, targ_angles) = {
            let tv = &self.entities[targ];
            (tv.v.origin, tv.v.spawnflags.has(PathRotateFlags::ANGLES), tv.v.angles)
        };

        // `speed == -1` warps straight to the next corner after the wait.
        if cur_speed == -1.0 {
            self.place(e, targ_origin);
            self.entities[e].rot.endtime = ltime + 0.01;
            self.set_target_origin(e);
            if targ_angles_flag {
                self.entities[e].v.angles = targ_angles;
            }
            let ent = &mut self.entities[e];
            ent.rot.duration = 1.0;
            ent.mover.cnt = t;
            ent.mover.dest2 = Vec3::ZERO;
            ent.mover.dest1 = targ_origin;
            ent.mover.finaldest = targ_origin;
            return;
        }

        // Otherwise travel to it.
        self.entities[e].rot.phase = RotPhase::Moving;
        self.entities[e].mover.finaldest = targ_origin;
        if targ_origin == origin {
            // Already there: idle one tick.
            let ent = &mut self.entities[e];
            ent.v.velocity = Vec3::ZERO;
            ent.rot.endtime = ltime + 0.1;
            ent.rot.duration = 1.0;
            ent.mover.cnt = t;
            ent.mover.dest2 = Vec3::ZERO;
            ent.mover.dest1 = origin;
            return;
        }

        let delta = targ_origin - origin;
        let traveltime = if movetime_flag {
            cur_speed
        } else {
            // A positive corner speed becomes the train's new cruising speed.
            let mut speed = self.entities[e].mover.speed;
            if cur_speed > 0.0 {
                speed = cur_speed;
                self.entities[e].mover.speed = cur_speed;
            }
            if speed == 0.0 {
                self.host.dprint(c"rotate_train_next: no speed defined\n");
                return;
            }
            delta.length() / speed
        };
        if traveltime < 0.1 {
            let ent = &mut self.entities[e];
            ent.v.velocity = Vec3::ZERO;
            ent.rot.endtime = ltime + 0.1;
            if targ_angles_flag {
                ent.v.angles = targ_angles;
            }
            return;
        }

        let div = 1.0 / traveltime;
        if targ_angles_flag {
            let angles = self.entities[e].v.angles;
            let ent = &mut self.entities[e];
            ent.mover.finalangle = normalize_angles(targ_angles);
            ent.rot.rotate = (targ_angles - angles) * div;
        }
        let ent = &mut self.entities[e];
        ent.rot.endtime = ltime + traveltime;
        ent.v.velocity = delta * div;
        ent.rot.duration = div;
        ent.mover.cnt = t;
        ent.mover.dest2 = delta;
        ent.mover.dest1 = origin;
    }

    /// Fire a corner's `event` as the train departs it: temporarily borrow the train's `target`
    /// and `message` to run `SUB_UseTargets` against the corner's event, then restore them. The
    /// train is the activator. No-op if the corner has no event.
    fn fire_corner_event(&mut self, e: EntId, corner: EntId) {
        let Some(event) = self.entities[corner].event.clone() else {
            return;
        };
        let saved_target = self.entities[e].target.clone();
        let message = self.entities[corner].message.clone();
        self.entities[e].target = Some(event);
        self.entities[e].message = message;
        self.activator = e;
        self.sub_use_targets(e);
        self.entities[e].target = saved_target;
        self.entities[e].message = None;
    }

    /// Resolve the train's current `path` to the next `path_rotate` corner, logging and returning
    /// `None` on a malformed chain (rather than aborting the server as the QC did).
    fn next_path_corner(&self, e: EntId) -> Option<EntId> {
        let path = self.entities[e].path.clone()?;
        let targ = self.find_by_targetname(&path).next()?;
        if self.entities[targ].classname() == Some("path_rotate") {
            Some(targ)
        } else {
            self.host.dprint(c"rotate_train: next target is not a path_rotate\n");
            None
        }
    }

    /// Play a corner's stop sound, falling back to the train's own.
    fn rotate_train_play_noise(&mut self, e: EntId, goal: EntId) {
        if let Some(n) = self.entities[goal].noise.or(self.entities[e].noise) {
            self.host.sound(e, Channel::Voice, n, 1.0, Attenuation::Norm);
        }
    }
}
