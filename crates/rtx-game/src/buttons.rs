// SPDX-License-Identifier: AGPL-3.0-or-later

//! `func_button`, ported from `qw-qc/buttons.qc`. A button slides to `pos2` when touched,
//! shot, or used, fires its targets, waits, then slides back to `pos1`.

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{Die, EntId, Think, Touch, Use, STATE_BOTTOM, STATE_DOWN, STATE_TOP, STATE_UP};
use crate::game::GameState;

impl GameState {
    /// `button_wait` — reached the top: fire targets, hold, then schedule the return.
    pub(crate) fn button_wait(&mut self, e: EntId) {
        let (wait, ltime) = {
            let v = &self.entities[e];
            (v.mover.wait, v.v.ltime)
        };
        {
            let ent = &mut self.entities[e];
            ent.mover.state = STATE_TOP;
            ent.v.nextthink = ltime + wait;
            ent.think = Think::ButtonReturn;
            ent.v.frame = 1.0;
        }
        self.activator = self.entities[e].enemy();
        self.sub_use_targets(e);
    }

    /// `button_done`.
    pub(crate) fn button_done(&mut self, e: EntId) {
        self.entities[e].mover.state = STATE_BOTTOM;
    }

    /// `button_return`.
    pub(crate) fn button_return(&mut self, e: EntId) {
        let (pos1, speed, health) = {
            let v = &self.entities[e];
            (v.mover.pos1, v.mover.speed, v.v.health)
        };
        self.entities[e].mover.state = STATE_DOWN;
        self.sub_calc_move(e, pos1, speed, Think::ButtonDone);
        self.entities[e].v.frame = 0.0;
        if health != 0.0 {
            self.entities[e].v.takedamage = TakeDamage::Yes;
        }
    }

    /// `button_fire` — start the upward stroke.
    fn button_fire(&mut self, e: EntId) {
        let (state, pos2, speed) = {
            let v = &self.entities[e];
            (v.mover.state, v.mover.pos2, v.mover.speed)
        };
        if state == STATE_UP || state == STATE_TOP {
            return;
        }
        self.play_noise(e, Channel::Voice);
        self.entities[e].mover.state = STATE_UP;
        self.sub_calc_move(e, pos2, speed, Think::ButtonWait);
    }

    /// `button_use`.
    pub(crate) fn button_use(&mut self, e: EntId) {
        let act = self.activator;
        self.entities[e].set_enemy(act);
        self.button_fire(e);
    }

    /// `button_touch`.
    pub(crate) fn button_touch(&mut self, e: EntId, other: EntId) {
        if !self.entities[other].is_player() {
            return;
        }
        self.entities[e].set_enemy(other);
        self.button_fire(e);
    }

    /// `button_killed` (`th_die`).
    pub(crate) fn button_killed(&mut self, e: EntId) {
        let attacker = self.damage_attacker;
        {
            let ent = &mut self.entities[e];
            ent.set_enemy(attacker);
            ent.v.health = ent.v.max_health;
            ent.v.takedamage = TakeDamage::No;
        }
        self.button_fire(e);
    }

    /// `func_button` spawn.
    pub(crate) fn spawn_func_button(&mut self, e: EntId) -> bool {
        self.entities[e].noise = Some(match self.entities[e].v.sounds as i32 {
            0 => Sound::BUTTONS_AIRBUT1,
            1 => Sound::BUTTONS_SWITCH21,
            2 => Sound::BUTTONS_SWITCH02,
            _ => Sound::BUTTONS_SWITCH04,
        });

        self.set_movedir(e);
        {
            let ent = &mut self.entities[e];
            ent.v.movetype = MoveType::Push;
            ent.v.solid = Solid::Bsp;
        }
        self.set_brush_model(e);
        self.entities[e].use_ = Use::ButtonUse;

        if self.entities[e].v.health != 0.0 {
            let ent = &mut self.entities[e];
            ent.v.max_health = ent.v.health;
            ent.th_die = Die::ButtonKilled;
            ent.v.takedamage = TakeDamage::Yes;
        } else {
            self.entities[e].set_touch(Touch::ButtonTouch);
        }

        let ent = &mut self.entities[e];
        if ent.mover.speed == 0.0 {
            ent.mover.speed = 40.0;
        }
        if ent.mover.wait == 0.0 {
            ent.mover.wait = 1.0;
        }
        if ent.mover.lip == 0.0 {
            ent.mover.lip = 4.0;
        }
        ent.mover.state = STATE_BOTTOM;
        ent.mover.pos1 = ent.v.origin;
        ent.mover.pos2 = crate::subs::mover_pos2(ent.mover.pos1, ent.v.movedir, ent.v.size, ent.mover.lip);
        true
    }
}
