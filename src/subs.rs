//! Movement and target-firing helpers, ported from `qw-qc/subs.qc`.
//!
//! QuakeC's `SUB_CalcMove`/`SUB_CalcAngleMove` schedule a constant-velocity move and a
//! follow-up `think1` to run on arrival. We keep that structure: the mover's `think` is set
//! to [`Think::SubCalcMoveDone`], and `think1` holds the callback to run once it lands.

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{EntId, Think, Use};
use crate::game::GameState;

impl GameState {
    /// `SetMovedir` — QuakeEd writes a single yaw float for door/button move direction;
    /// the magic angles `0 -1 0` / `0 -2 0` mean straight up / down.
    pub(crate) fn set_movedir(&mut self, e: EntId) {
        let angles = self.entities[e].v.angles;
        let movedir = if angles == Vec3::new(0.0, -1.0, 0.0) {
            Vec3::new(0.0, 0.0, 1.0)
        } else if angles == Vec3::new(0.0, -2.0, 0.0) {
            Vec3::new(0.0, 0.0, -1.0)
        } else {
            self.host.make_vectors(angles);
            self.globals.v_forward
        };
        let ent = &mut self.entities[e];
        ent.v.movedir = movedir;
        ent.v.angles = Vec3::ZERO;
    }

    /// `InitTrigger` — shared setup for trigger volumes: trigger solidity, link the brush
    /// model for its bounds, then hide the model.
    pub(crate) fn init_trigger(&mut self, e: EntId) {
        if self.entities[e].v.angles != Vec3::ZERO {
            self.set_movedir(e);
        }
        self.entities[e].v.solid = Solid::Trigger.as_f32();
        self.set_brush_model(e);
        let ent = &mut self.entities[e];
        ent.v.movetype = MoveType::None.as_f32();
        ent.v.modelindex = 0.0;
        ent.model = None;
    }

    /// `SUB_CalcMove` — move `e` to `tdest` at `tspeed`, then run `func`.
    pub(crate) fn sub_calc_move(&mut self, e: EntId, tdest: Vec3, tspeed: f32, func: Think) {
        debug_assert!(tspeed != 0.0, "SUB_CalcMove: no speed");
        let (origin, ltime) = {
            let v = &self.entities[e].v;
            (v.origin, v.ltime)
        };

        {
            let ent = &mut self.entities[e];
            ent.think1 = func;
            ent.mover.finaldest = tdest;
            ent.think = Think::SubCalcMoveDone;
        }

        if tdest == origin {
            let ent = &mut self.entities[e];
            ent.v.velocity = Vec3::ZERO;
            ent.v.nextthink = ltime + 0.1;
            return;
        }

        let vdestdelta = tdest - origin;
        let len = vdestdelta.length();
        let traveltime = (len / tspeed).max(0.03);

        let ent = &mut self.entities[e];
        ent.v.nextthink = ltime + traveltime;
        ent.v.velocity = vdestdelta * (1.0 / traveltime);
    }

    /// `SUB_CalcMoveDone` — snap to the exact destination and fire `think1`.
    pub(crate) fn sub_calc_move_done(&mut self, e: EntId) {
        let dest = self.entities[e].mover.finaldest;
        self.host.set_origin(e.0 as i32, dest);
        {
            let ent = &mut self.entities[e];
            ent.v.origin = dest;
            ent.v.velocity = Vec3::ZERO;
            ent.v.nextthink = -1.0;
        }
        let next = self.entities[e].think1;
        if next != Think::None {
            self.run_think_now(e, next);
        }
    }

    /// `SUB_Remove` — free this entity.
    pub(crate) fn sub_remove(&mut self, e: EntId) {
        self.free(e);
    }

    /// `DelayThink` — the deferred body of a delayed [`Self::sub_use_targets`].
    pub(crate) fn delayed_use(&mut self, e: EntId) {
        self.activator = self.entities[e].enemy();
        self.sub_use_targets(e);
        self.free(e);
    }

    /// `SUB_UseTargets` — the core trigger mechanism: optionally delay, center-print the
    /// message to the activator, remove `killtarget`s, then `.use` every entity whose
    /// `targetname` matches our `target`. `self.activator` must be set by the caller.
    pub(crate) fn sub_use_targets(&mut self, e: EntId) {
        // Delayed fire: spawn a temp entity that re-runs us after `delay` seconds.
        let delay = self.entities[e].mover.delay;
        if delay != 0.0 {
            let t = self.spawn();
            let time = self.time();
            let (message, killtarget, target) = {
                let s = &self.entities[e];
                (s.message.clone(), s.killtarget.clone(), s.target.clone())
            };
            let activator = self.activator;
            let td = &mut self.entities[t];
            td.classname = Some("DelayedUse".into());
            td.v.nextthink = time + delay;
            td.think = Think::DelayedUse;
            td.set_enemy(activator);
            td.message = message;
            td.killtarget = killtarget;
            td.target = target;
            return;
        }

        // Center-print our message to the activator (if a player).
        let activator = self.activator;
        let has_message = self.entities[e].message.is_some();
        if has_message && self.entities[activator].classname() == Some("player") {
            if let Some(msg) = self.message_cstring(e) {
                self.host.centerprint(activator.0 as i32, &msg);
            }
            if self.entities[e].noise.is_none() {
                self.host
                    .sound(activator.0 as i32, Channel::Voice, Sound::MISC_TALK, 1.0, Attenuation::Norm);
            }
        }

        // Remove killtargets.
        if let Some(killtarget) = self.entities[e].killtarget.clone() {
            let victims: Vec<EntId> = self.find_by_targetname(&killtarget).collect();
            for v in victims {
                self.free(v);
            }
        }

        // Fire targets.
        if let Some(target) = self.entities[e].target.clone() {
            let targets: Vec<EntId> = self.find_by_targetname(&target).collect();
            for t in targets {
                let use_ = self.entities[t].use_;
                if use_ != Use::None {
                    // QuakeC sets self=t, other=stemp before .use(); our use handlers take
                    // the target id and read `self.activator` directly.
                    self.run_use(t);
                }
            }
        }
    }

    /// Live entities whose `targetname` matches `name`.
    pub(crate) fn find_by_targetname<'a>(
        &'a self,
        name: &'a str,
    ) -> impl Iterator<Item = EntId> + 'a {
        self.entities
            .iter()
            .enumerate()
            .filter(move |(_, e)| e.in_use && e.targetname.as_deref() == Some(name))
            .map(|(i, _)| EntId(i as u32))
    }

}
