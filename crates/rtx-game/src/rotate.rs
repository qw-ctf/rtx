// SPDX-License-Identifier: AGPL-3.0-or-later

//! The Hipnotic mission-pack rotating brushes (`hiprot.qc`): continually-spinning brushes,
//! rotating trains that ride `path_rotate` corners, and rotating doors.
//!
//! QuakeWorld BSP brush models can't truly rotate (their clip hull is axis-aligned), so this
//! system fakes it: a *rotator* (`func_rotate_entity` / `func_rotate_train` / `func_rotate_door`)
//! spins, and every tick recomputes the origin of each brush it *targets* so they orbit the
//! rotator's origin. Targets follow in one of three ways ([`RotateType`]):
//!
//! * **[`RotateType::Rotate`]** (`rotate_object`) — a display brush; it tracks the orbit *and*
//!   copies the rotator's angles, so it visibly turns.
//! * **[`RotateType::Movewall`]** (`func_movewall`) — a `MoveType::Push` clip brush driven by
//!   velocity, so it actually pushes and crushes players (the collision the display brush lacks).
//! * **[`RotateType::SetOrigin`]** — any other target, carried along by `setorigin`.
//!
//! The port is a thorough cleanup of the original (which overloaded one integer `state` with
//! three unrelated state machines and reached for direct `think1()` function-pointer calls): the
//! states are named [`RotPhase`] variants, the "what to run next" continuations are ordinary
//! [`Think`] values dispatched through the central table, and the scratch lives in
//! [`RotState`](crate::entity::RotState) / reused [`MoverState`](crate::entity::MoverState).

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{Blocked, EntId, RotPhase, RotateType, Think, Touch, Use};
use crate::game::GameState;

/// `SUB_NormalizeAngles` — wrap each angle into `(-360, 360)` (QuakeC's `fmod(a, 360)`).
fn normalize_angles(a: Vec3) -> Vec3 {
    Vec3::new(a.x % 360.0, a.y % 360.0, a.z % 360.0)
}

impl GameState {
    // ----------------------------------------------------------------------------------------
    // Shared rotation engine: move/spin the brushes a rotator targets around its own origin.
    // ----------------------------------------------------------------------------------------

    /// The live brushes a rotator targets (its `target` → their `targetname`). Collected into a
    /// `Vec` so the find-borrow is released before the caller mutates each target; empty if the
    /// rotator has no `target`.
    fn rotator_targets(&self, e: EntId) -> Vec<EntId> {
        match self.entities[e].target.as_deref() {
            Some(target) => self.find_by_targetname(target).collect(),
            None => Vec::new(),
        }
    }

    /// `RotateTargets` — for the current rotator angles/origin, place every targeted brush at its
    /// orbited position this tick. Movewalls are driven by velocity (so the engine's pusher
    /// physics carries riders); the rest are placed with `setorigin`.
    fn rotate_targets(&mut self, e: EntId) {
        let targets = self.rotator_targets(e);
        if targets.is_empty() {
            return;
        }
        let (angles, origin, oldorigin) = {
            let v = &self.entities[e].v;
            (v.angles, v.origin, v.oldorigin)
        };
        self.host.make_vectors(angles);
        let (vf, vr, vu) = {
            let g = &self.globals;
            (g.v_forward, g.v_right, g.v_up)
        };

        for t in targets {
            let (rel, kind, t_origin) = {
                let te = &self.entities[t];
                (te.v.oldorigin, te.rot.kind, te.v.origin)
            };
            // The target's offset from the centre, rotated into the rotator's frame.
            let orbit = vf * rel.x - vr * rel.y + vu * rel.z;
            if kind == RotateType::Movewall {
                // Driven by velocity so the pusher carries/crushes riders; the new position folds
                // in how far the rotator itself has shifted since linking.
                let neworigin = (origin - oldorigin) + orbit - rel;
                self.entities[t].rot.neworigin = neworigin;
                self.entities[t].v.velocity = (neworigin - t_origin) * 25.0;
            } else {
                self.entities[t].rot.neworigin = orbit;
                if kind == RotateType::Rotate {
                    self.entities[t].v.angles = angles; // a display brush also turns
                }
                self.place(t, orbit + origin);
            }
        }
    }

    /// `RotateTargetsFinal` — stop every targeted brush and snap rotators' display brushes to the
    /// rotator's final angles (called when a rotation segment ends).
    fn rotate_targets_final(&mut self, e: EntId) {
        let angles = self.entities[e].v.angles;
        for t in self.rotator_targets(e) {
            self.entities[t].v.velocity = Vec3::ZERO;
            if self.entities[t].rot.kind == RotateType::Rotate {
                self.entities[t].v.angles = angles;
            }
        }
    }

    /// `SetTargetOrigin` — place every targeted brush at its orbited position directly (used after
    /// a warp/teleport of the rotator, where there is no per-tick interpolation).
    fn set_target_origin(&mut self, e: EntId) {
        let (origin, oldorigin) = {
            let v = &self.entities[e].v;
            (v.origin, v.oldorigin)
        };
        for t in self.rotator_targets(e) {
            let (kind, neworigin, rel) = {
                let te = &self.entities[t];
                (te.rot.kind, te.rot.neworigin, te.v.oldorigin)
            };
            let pos = if kind == RotateType::Movewall {
                (origin - oldorigin) + neworigin - rel
            } else {
                neworigin + origin
            };
            self.place(t, pos);
        }
    }

    /// `LinkRotateTargets` — one-time setup: record the rotator's origin as the centre, then for
    /// each targeted brush record its offset from that centre and classify how it follows.
    fn link_rotate_targets(&mut self, e: EntId) {
        let origin = self.entities[e].v.origin;
        self.entities[e].v.oldorigin = origin; // centre of rotation
        for t in self.rotator_targets(e) {
            let kind = match self.entities[t].classname() {
                Some("rotate_object") => RotateType::Rotate,
                Some("func_movewall") => RotateType::Movewall,
                _ => RotateType::SetOrigin,
            };
            // Movewalls orbit around their bbox centre; everything else around its origin.
            let rel = if kind == RotateType::Movewall {
                let v = &self.entities[t].v;
                (v.absmin + v.absmax) * 0.5 - origin
            } else {
                self.entities[t].v.origin - origin
            };
            let te = &mut self.entities[t];
            te.rot.kind = kind;
            te.v.oldorigin = rel;
            te.rot.neworigin = rel;
            // Display/clip brushes need to know their rotator for touch/blocked damage.
            if kind != RotateType::SetOrigin {
                te.set_owner(e);
            }
        }
    }

    /// `SetDamageOnTargets` — arm/disarm the damage of a rotator's `trigger_hurt` and `func_movewall`
    /// targets (path_rotate's `SET_DAMAGE`).
    fn set_damage_on_targets(&mut self, e: EntId, amount: f32) {
        for t in self.rotator_targets(e) {
            if self.entities[t].classname() == Some("trigger_hurt") {
                let te = &mut self.entities[t];
                te.mover.dmg = amount;
                te.v.solid = if amount == 0.0 { Solid::Not } else { Solid::Trigger };
                te.v.nextthink = -1.0;
            } else if self.entities[t].classname() == Some("func_movewall") {
                self.entities[t].mover.dmg = amount;
            }
        }
    }

    /// `setorigin(e, pos)` keeping our shadowed `v.origin` in sync (the engine writes the shared
    /// field too, but later same-tick reads go through our copy).
    fn place(&mut self, e: EntId, pos: Vec3) {
        self.host.set_origin(e, pos);
        self.entities[e].v.origin = pos;
    }

    /// `setmodel` + `setsize`: link a brush entity's model and its bounding box.
    pub(crate) fn link_brush(&mut self, e: EntId) {
        self.set_brush_model(e);
        let (mins, maxs) = {
            let v = &self.entities[e].v;
            (v.mins, v.maxs)
        };
        self.host.set_size(e, mins, maxs);
    }

    // ----------------------------------------------------------------------------------------
    // info_rotate / path_rotate / rotate_object — markers and followers.
    // ----------------------------------------------------------------------------------------

    /// `info_rotate` — a centre-of-rotation marker; removes itself shortly after spawn, once the
    /// entities that target it have had a chance to link.
    pub(crate) fn spawn_info_rotate(&mut self, e: EntId) -> bool {
        let t = self.time();
        let ent = &mut self.entities[e];
        ent.v.nextthink = t + 2.0;
        ent.think = Think::SubRemove;
        true
    }

    /// `path_rotate` — an inert corner for `func_rotate_train`. Its `noise`/`noise1` sound paths
    /// are precached as the map is parsed (see `set_noise_field`), so nothing else is needed here.
    pub(crate) fn spawn_path_rotate(&mut self, _e: EntId) -> bool {
        true
    }

    /// `rotate_object` — a display brush carried (and turned) by a rotator that targets it.
    pub(crate) fn spawn_rotate_object(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.classname = Some("rotate_object".into());
            ent.v.solid = Solid::Not;
            ent.v.movetype = MoveType::None;
        }
        self.link_brush(e);
        self.entities[e].think = Think::None;
        true
    }

    // ----------------------------------------------------------------------------------------
    // func_rotate_entity — a brush that continually spins, optionally toggled on/off.
    // ----------------------------------------------------------------------------------------

    pub(crate) fn spawn_func_rotate_entity(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.solid = Solid::Not;
            ent.v.movetype = MoveType::None;
        }
        self.link_brush(e);
        let (t, speed) = (self.time(), self.entities[e].mover.speed);
        let ent = &mut self.entities[e];
        // `speed` is the spin-up/down time; `cnt` is its reciprocal (rate the fraction ramps).
        if speed != 0.0 {
            ent.mover.cnt = 1.0 / speed;
        }
        // Defer linking targets one frame so they've all spawned.
        ent.think = Think::RotateEntityFirstThink;
        ent.v.nextthink = t + 0.1;
        ent.v.ltime = t;
        true
    }

    pub(crate) fn rotate_entity_firstthink(&mut self, e: EntId) {
        self.link_rotate_targets(e);
        let t = self.time();
        let start_on = self.entities[e].v.spawnflags.has(RotateEntityFlags::START_ON);
        let ent = &mut self.entities[e];
        if start_on {
            ent.rot.phase = RotPhase::Active;
            ent.think = Think::RotateEntityThink;
            ent.v.nextthink = t + 0.02;
            ent.v.ltime = t;
        } else {
            ent.rot.phase = RotPhase::Inactive;
            ent.think = Think::None;
        }
        ent.use_ = Use::RotateEntityUse;
    }

    pub(crate) fn rotate_entity_think(&mut self, e: EntId) {
        let now = self.time();
        let (phase, ltime, mut count, cnt) = {
            let v = &self.entities[e];
            (v.rot.phase, v.v.ltime, v.mover.count, v.mover.cnt)
        };
        let mut t = now - ltime;
        self.entities[e].v.ltime = now;

        // While spinning up or down, scale the rotation by a 0..1 fraction that ramps at `cnt`.
        match phase {
            RotPhase::SpeedingUp => {
                count = (count + cnt * t).min(1.0);
                self.entities[e].mover.count = count;
                t *= count;
            }
            RotPhase::SlowingDown => {
                count -= cnt * t;
                if count < 0.0 {
                    self.rotate_targets_final(e);
                    let ent = &mut self.entities[e];
                    ent.rot.phase = RotPhase::Inactive;
                    ent.think = Think::None;
                    return;
                }
                self.entities[e].mover.count = count;
                t *= count;
            }
            _ => {}
        }

        let delta = self.entities[e].rot.rotate * t;
        self.entities[e].v.angles = normalize_angles(self.entities[e].v.angles + delta);
        self.rotate_targets(e);
        self.entities[e].v.nextthink = now + 0.02;
    }

    pub(crate) fn rotate_entity_use(&mut self, e: EntId) {
        let t = self.time();
        let (phase, toggle, speed) = {
            let v = &self.entities[e];
            (
                v.rot.phase,
                v.v.spawnflags.has(RotateEntityFlags::TOGGLE),
                v.mover.speed,
            )
        };
        let ent = &mut self.entities[e];
        ent.v.frame = 1.0 - ent.v.frame; // alternate textures
        match phase {
            RotPhase::Active => {
                if toggle {
                    if speed != 0.0 {
                        ent.mover.count = 1.0;
                        ent.rot.phase = RotPhase::SlowingDown;
                    } else {
                        ent.rot.phase = RotPhase::Inactive;
                        ent.think = Think::None;
                    }
                }
            }
            RotPhase::Inactive => {
                ent.think = Think::RotateEntityThink;
                ent.v.nextthink = t + 0.02;
                ent.v.ltime = t;
                if speed != 0.0 {
                    ent.mover.count = 0.0;
                    ent.rot.phase = RotPhase::SpeedingUp;
                } else {
                    ent.rot.phase = RotPhase::Active;
                }
            }
            RotPhase::SpeedingUp => {
                if toggle {
                    ent.rot.phase = RotPhase::SlowingDown;
                }
            }
            // SlowingDown (or any other): reverse back to spinning up.
            _ => ent.rot.phase = RotPhase::SpeedingUp,
        }
    }

    // ----------------------------------------------------------------------------------------
    // func_movewall — the clip/collision brush that gives a rotating object solidity.
    // ----------------------------------------------------------------------------------------

    pub(crate) fn spawn_func_movewall(&mut self, e: EntId) -> bool {
        let t = self.time();
        let spawnflags = self.entities[e].v.spawnflags;
        {
            let ent = &mut self.entities[e];
            ent.v.angles = Vec3::ZERO;
            ent.v.movetype = MoveType::Push;
            if spawnflags.has(MovewallFlags::NONBLOCKING) {
                ent.v.solid = Solid::Not;
            } else {
                ent.v.solid = Solid::Bsp;
                ent.set_blocked(Blocked::MovewallBlocked);
            }
            if spawnflags.has(MovewallFlags::TOUCH) {
                ent.set_touch(Touch::Movewall);
            }
        }
        self.set_brush_model(e);
        // A movewall is normally an invisible collision proxy for the object it shadows. Like
        // ktx's `self->model = NULL`, null the engine-visible model *string* (not the modelindex):
        // the server skips entities whose model string is empty (`sv_ents.c`), so it stops being
        // drawn, while `modelindex` + `SOLID_BSP` keep its clip hull for collision. `VISIBLE`
        // leaves it drawn.
        if !spawnflags.has(MovewallFlags::VISIBLE) {
            self.entities[e].v.model = 0;
        }
        let ent = &mut self.entities[e];
        ent.think = Think::MovewallThink;
        ent.v.nextthink = t + 0.02;
        ent.v.ltime = t;
        true
    }

    /// Keep-alive tick: a pusher needs its `ltime` advanced each frame even though the rotator
    /// (not the wall itself) decides the velocity.
    pub(crate) fn movewall_think(&mut self, e: EntId) {
        let t = self.time();
        let ent = &mut self.entities[e];
        ent.v.ltime = t;
        ent.v.nextthink = t + 0.02;
    }

    pub(crate) fn movewall_touch(&mut self, e: EntId, other: EntId) {
        let owner = self.entities[e].owner();
        let t = self.time();
        if t < self.entities[owner].combat.attack_finished {
            return;
        }
        self.movewall_apply_damage(e, other, owner, t);
    }

    pub(crate) fn movewall_blocked(&mut self, e: EntId, other: EntId) {
        let owner = self.entities[e].owner();
        let t = self.time();
        if t < self.entities[owner].combat.attack_finished {
            return;
        }
        self.entities[owner].combat.attack_finished = t + 0.5;
        // A blocked rotating door bounces its whole group back the other way.
        if self.entities[owner].classname() == Some("func_rotate_door") {
            self.rotate_door_group_reversedirection(owner);
        }
        self.movewall_apply_damage(e, other, owner, t);
    }

    /// Shared touch/blocked damage: the wall's own `dmg` takes precedence over its rotator's.
    fn movewall_apply_damage(&mut self, e: EntId, other: EntId, owner: EntId, t: f32) {
        let dmg = {
            let self_dmg = self.entities[e].mover.dmg;
            if self_dmg != 0.0 {
                self_dmg
            } else {
                self.entities[owner].mover.dmg
            }
        };
        if dmg != 0.0 {
            self.t_damage(other, e, owner, dmg);
            self.entities[owner].combat.attack_finished = t + 0.5;
        }
    }

    // ----------------------------------------------------------------------------------------
    // func_rotate_door — a brush group that swings between two angles when triggered.
    // ----------------------------------------------------------------------------------------

    pub(crate) fn spawn_func_rotate_door(&mut self, e: EntId) -> bool {
        if self.entities[e].target.is_none() {
            return false; // a rotate_door is useless without targets to swing
        }
        let angles = self.entities[e].v.angles;
        {
            let ent = &mut self.entities[e];
            ent.mover.dest1 = Vec3::ZERO; // closed angles
            ent.mover.dest2 = angles; // open angles (the mapped angles)
            ent.v.angles = Vec3::ZERO;
            if ent.mover.speed == 0.0 {
                ent.mover.speed = 2.0;
            }
            ent.mover.cnt = 0.0; // "targets linked" latch
            if ent.mover.dmg == 0.0 {
                ent.mover.dmg = 2.0;
            } else if ent.mover.dmg < 0.0 {
                ent.mover.dmg = 0.0;
            }
            if ent.v.sounds == 0.0 {
                ent.v.sounds = 1.0;
            }
        }
        // Sound set: noise1 = move-start latch, noise2 = swing loop, noise3 = arrival.
        let (n1, n2, n3) = match self.entities[e].v.sounds as i32 {
            2 => (Sound::DOORS_AIRDOOR2, Sound::DOORS_AIRDOOR1, Sound::DOORS_AIRDOOR2),
            3 => (Sound::DOORS_BASESEC2, Sound::DOORS_BASESEC1, Sound::DOORS_BASESEC2),
            _ => (Sound::DOORS_LATCH2, Sound::DOORS_WINCH2, Sound::DOORS_DRCLOS4),
        };
        {
            let ent = &mut self.entities[e];
            ent.noise1 = Some(n1);
            ent.noise2 = Some(n2);
            ent.noise3 = Some(n3);
            ent.v.solid = Solid::Not;
            ent.v.movetype = MoveType::None;
        }
        self.link_brush(e);
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e, origin);
        let ent = &mut self.entities[e];
        ent.rot.phase = RotPhase::Closed;
        ent.use_ = Use::RotateDoorUse;
        ent.think = Think::None;
        true
    }

    pub(crate) fn rotate_door_use(&mut self, e: EntId) {
        let phase = self.entities[e].rot.phase;
        if phase != RotPhase::Open && phase != RotPhase::Closed {
            return; // mid-swing; ignore
        }
        // Link targets the first time the door is used.
        if self.entities[e].mover.cnt == 0.0 {
            self.entities[e].mover.cnt = 1.0;
            self.link_rotate_targets(e);
        }
        self.entities[e].v.frame = 1.0 - self.entities[e].v.frame;
        let t = self.time();
        self.rotate_door_begin_swing(e, phase == RotPhase::Closed, t + self.entities[e].mover.speed);
    }

    /// `rotate_door_reversedirection` — flip a door mid-swing (used when blocked or on `STAYOPEN`),
    /// preserving how far it had already turned.
    fn rotate_door_reversedirection(&mut self, e: EntId) {
        let (closing, speed, endtime) = {
            let v = &self.entities[e];
            (v.rot.phase == RotPhase::Closing, v.mover.speed, v.rot.endtime)
        };
        self.entities[e].v.frame = 1.0 - self.entities[e].v.frame;
        let t = self.time();
        // Remaining-time mirror: a swing that was `endtime - t` from finishing now needs
        // `speed - (endtime - t)` to return.
        self.rotate_door_begin_swing(e, closing, t + speed - (endtime - t));
    }

    /// `rotate_door_group_reversedirection` — bounce a whole grouped door back, or just this one.
    fn rotate_door_group_reversedirection(&mut self, e: EntId) {
        if let Some(group) = self.entities[e].group.clone() {
            for m in self.find_by_group(&group).collect::<Vec<_>>() {
                self.rotate_door_reversedirection(m);
            }
        } else {
            self.rotate_door_reversedirection(e);
        }
    }

    /// Common swing setup for [`rotate_door_use`] and [`rotate_door_reversedirection`]: head for the
    /// open angles when `from_closed`, else the closed angles, arriving at `endtime`.
    fn rotate_door_begin_swing(&mut self, e: EntId, from_closed: bool, endtime: f32) {
        let (dest1, dest2, speed, noise2) = {
            let v = &self.entities[e];
            (v.mover.dest1, v.mover.dest2, v.mover.speed, v.noise2)
        };
        let (start, dest, phase) = if from_closed {
            (dest1, dest2, RotPhase::Opening)
        } else {
            (dest2, dest1, RotPhase::Closing)
        };
        if let Some(n) = noise2 {
            self.host.sound(e, Channel::Voice, n, 1.0, Attenuation::Norm);
        }
        let t = self.time();
        let ent = &mut self.entities[e];
        ent.mover.dest = dest;
        ent.rot.phase = phase;
        ent.rot.rotate = (dest - start) * (1.0 / speed);
        ent.think = Think::RotateDoorThink;
        ent.v.nextthink = t + 0.01;
        ent.rot.endtime = endtime;
        ent.v.ltime = t;
    }

    pub(crate) fn rotate_door_think(&mut self, e: EntId) {
        let now = self.time();
        let (ltime, endtime, rotate) = {
            let v = &self.entities[e];
            (v.v.ltime, v.rot.endtime, v.rot.rotate)
        };
        let t = now - ltime;
        self.entities[e].v.ltime = now;
        if now < endtime {
            self.entities[e].v.angles += rotate * t;
            self.rotate_targets(e);
        } else {
            let dest = self.entities[e].mover.dest;
            self.entities[e].v.angles = dest;
            self.rotate_targets(e);
            self.entities[e].think = Think::RotateDoorThink2;
        }
        self.entities[e].v.nextthink = now + 0.01;
    }

    /// Arrival: settle at the destination angles, then either rest open/closed or (with `STAYOPEN`)
    /// immediately swing back.
    pub(crate) fn rotate_door_think2(&mut self, e: EntId) {
        let t = self.time();
        let (phase, stayopen, dest, noise3) = {
            let v = &self.entities[e];
            (
                v.rot.phase,
                v.v.spawnflags.has(RotateDoorFlags::STAYOPEN),
                v.mover.dest,
                v.noise3,
            )
        };
        {
            let ent = &mut self.entities[e];
            ent.v.ltime = t;
            ent.v.frame = 1.0 - ent.v.frame;
            ent.v.angles = dest;
        }
        if phase == RotPhase::Opening {
            self.entities[e].rot.phase = RotPhase::Open;
        } else {
            if stayopen {
                self.rotate_door_group_reversedirection(e);
                return;
            }
            self.entities[e].rot.phase = RotPhase::Closed;
        }
        if let Some(n) = noise3 {
            self.host.sound(e, Channel::Voice, n, 1.0, Attenuation::Norm);
        }
        self.entities[e].think = Think::None;
        self.rotate_targets_final(e);
    }

    // ----------------------------------------------------------------------------------------
    // func_rotate_train — a brush that rides path_rotate corners, rotating as it goes.
    // ----------------------------------------------------------------------------------------

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
        self.host.set_origin(e, origin);

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
