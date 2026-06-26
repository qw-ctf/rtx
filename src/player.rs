//! Player animation, ported from `qw-qc/player.qc`.
//!
//! QuakeC drives player animation as a think-chained state machine: each frame function
//! sets `self.frame`, schedules `nextthink = time + 0.1`, and points `self.think` at the
//! next function. We model the same loop with the [`Think`] enum and the engine's
//! `GAME_EDICT_THINK` callback (the engine ignores the entvars `think` funcref for native
//! modules and re-enters us whenever `nextthink` elapses).

use crate::defs::IT_AXE;
use crate::entity::{EntId, Think};
use crate::game::GameState;

// player.mdl frame indices (sequential across player.qc's `$frame` declarations).
const AXRUN1: i32 = 0;
const ROCKRUN1: i32 = 6;
const STAND1: i32 = 12;
const AXSTND1: i32 = 17;

impl GameState {
    /// `GAME_EDICT_THINK` entry: run the current entity's scheduled think.
    pub(crate) fn run_think(&mut self, e: EntId) {
        match self.entities[e.index()].think {
            Think::None => {}
            Think::PlayerStand => self.player_stand1(e),
            Think::PlayerRun => self.player_run(e),
        }
    }

    /// Re-arm an animation loop: schedule the next think 0.1s out and record which loop.
    fn schedule_anim(&mut self, e: EntId, think: Think) {
        let next = self.globals.time + 0.1;
        let ent = &mut self.entities[e.index()];
        ent.think = think;
        ent.v.nextthink = next;
    }

    /// `player_stand1` — idle loop; transitions to the run loop while moving.
    pub(crate) fn player_stand1(&mut self, e: EntId) {
        self.schedule_anim(e, Think::PlayerStand);

        let moving = {
            let ent = &mut self.entities[e.index()];
            ent.v.weaponframe = 0.0;
            ent.v.velocity.x != 0.0 || ent.v.velocity.y != 0.0
        };
        if moving {
            self.entities[e.index()].walkframe = 0;
            self.player_run(e);
            return;
        }

        let ent = &mut self.entities[e.index()];
        if ent.v.weapon == IT_AXE {
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
            let ent = &mut self.entities[e.index()];
            ent.v.weaponframe = 0.0;
            ent.v.velocity.x == 0.0 && ent.v.velocity.y == 0.0
        };
        if stopped {
            self.entities[e.index()].walkframe = 0;
            self.player_stand1(e);
            return;
        }

        let ent = &mut self.entities[e.index()];
        let base = if ent.v.weapon == IT_AXE { AXRUN1 } else { ROCKRUN1 };
        if ent.walkframe == 6 {
            ent.walkframe = 0;
        }
        ent.v.frame = (base + ent.walkframe) as f32;
        ent.walkframe += 1;
    }
}
