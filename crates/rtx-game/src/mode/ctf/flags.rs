// SPDX-License-Identifier: AGPL-3.0-or-later

//! CTF flags: the two team flags' spawn / grab / capture / drop / return lifecycle. These are
//! `GameState` methods dispatched from the flag entity's spawn/touch/think seams (and from the mode's
//! impulse handler for a voluntary toss).

use glam::Vec3;

use super::team;
use super::{
    CAPTURE_BONUS, EF_FLAG1, EF_FLAG2, FLAG_RETURN_TIME, FRAG_CARRIER_ASSIST, FRAG_CARRIER_ASSIST_BONUS,
    RETURN_ASSIST, RETURN_ASSIST_BONUS, RETURN_BONUS, TEAM_CAPTURE_BONUS,
};
use crate::assets::{Model, Sound};
use crate::defs::{Attenuation, Channel, MoveType, PrintLevel, Solid};
use crate::entity::{EntId, FlagPhase, Think, Touch};
use crate::game::GameState;
use crate::mode::players;

impl GameState {
    /// Spawn the red (team 1) flag.
    pub(crate) fn spawn_flag_team1(&mut self, e: EntId) -> bool {
        self.spawn_flag(e, 1, 0.0)
    }

    /// Spawn the blue (team 2) flag.
    pub(crate) fn spawn_flag_team2(&mut self, e: EntId) -> bool {
        self.spawn_flag(e, 2, 1.0)
    }

    /// Place a flag at its base. Only in CTF — otherwise the entity is removed (purectf's guard).
    fn spawn_flag(&mut self, e: EntId, team: u8, skin: f32) -> bool {
        if !self.mode.uses_ctf_objects() {
            return false;
        }
        self.entities[e].classname = Some("flag".into());
        {
            let ent = &mut self.entities[e];
            ent.flag.team = team;
            ent.flag.phase = FlagPhase::Home;
            ent.flag.carrier = EntId::WORLD;
            ent.v.skin = skin;
            ent.v.movetype = MoveType::Toss;
            ent.v.solid = Solid::Trigger;
        }
        self.set_model(e, Model::PROGS_FLAG);
        self
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 74.0));
        // Settle to the floor (best effort) and remember the base position for returns.
        self.entities[e].v.origin.z += 6.0;
        let _ = self.droptofloor(e);
        let home = self.entities[e].v.origin;
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.flag.home = home;
        ent.think = Think::FlagReturn;
        ent.v.nextthink = time + 0.5;
        ent.set_touch(Touch::Flag);
        true
    }

    /// `Touch::Flag` — a player touched a flag: grab (enemy), return (own, dropped), or capture
    /// (own base while carrying the enemy flag).
    pub(crate) fn flag_touch(&mut self, flag: EntId, other: EntId) {
        if !self.entities[other].is_player() || !self.entities[other].is_alive() {
            return;
        }
        if self.entities[flag].flag.phase == FlagPhase::Carried {
            return;
        }
        // The tosser can't re-grab a flag they just tossed until the lock expires.
        if self.entities[flag].flag.phase == FlagPhase::Tossed && other == self.entities[flag].flag.carrier {
            return;
        }
        let flag_team = self.entities[flag].flag.team;
        let player_team = self.entities[other].mode_p.team;
        if flag_team == 0 || player_team == 0 {
            return;
        }
        if player_team == flag_team {
            if self.entities[flag].flag.phase == FlagPhase::Home {
                // Own flag home: capture if we're carrying the enemy flag.
                if self.entities[other].mode_p.ctf.carrying != 0 {
                    self.ctf_capture(other);
                }
            } else {
                // Own flag dropped in the field: return it.
                self.ctf_return_flag(flag, other);
            }
        } else {
            self.ctf_grab(flag, other);
        }
    }

    /// `Think::FlagReturn` — idle flag tick: promote a tossed flag to dropped once its re-grab lock
    /// expires, and auto-return a dropped flag once its timeout elapses.
    pub(crate) fn flag_return_think(&mut self, flag: EntId) {
        let time = self.time();
        self.entities[flag].v.nextthink = time + 0.5;
        let phase = self.entities[flag].flag.phase;
        let due = time >= self.entities[flag].flag.return_at;
        if phase == FlagPhase::Tossed && due {
            // Lock over: a normal dropped flag now, with the full auto-return timer.
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Dropped;
            f.flag.carrier = EntId::WORLD;
            f.flag.return_at = time + FLAG_RETURN_TIME;
        } else if phase == FlagPhase::Dropped && due {
            self.flag_send_home(flag);
            let (name, _) = team::team_identity(self.entities[flag].flag.team);
            self.broadcast(PrintLevel::High, &format!("The {name} flag returned to base.\n"));
        }
    }

    /// Impulse 26 — toss the enemy flag you carry (gated by `rtx_ctf_tossflag`). It flies along your
    /// aim and you can't re-grab it for 2 s.
    pub(crate) fn toss_flag(&mut self, player: EntId) {
        // Reached only via `Ctf::handle_impulse`, so the mode is already CTF; just the cvar gate + a
        // carry check remain.
        if !self.host.cvar_bool(c"rtx_ctf_tossflag") || self.entities[player].mode_p.ctf.carrying == 0 {
            return;
        }
        let Some(flag) = self.carried_flag(player) else {
            self.entities[player].mode_p.ctf.carrying = 0;
            return;
        };
        let team = self.entities[flag].flag.team;
        let dir = self.aim_dir(player);
        let origin = self.entities[player].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let time = self.time();
        self.clear_carrier(flag);
        {
            // Like `place_flag`, but keeps the tosser as `carrier` (a "who threw it" reference the
            // grab logic reads), so this sets the fields itself rather than un-carrying via place_flag.
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Tossed;
            f.flag.carrier = player; // lock reference (the tosser)
            f.flag.return_at = time + 2.0;
            f.v.solid = Solid::Trigger;
            f.v.movetype = MoveType::Toss;
            f.v.velocity = dir * 300.0;
        }
        self.set_model(flag, Model::PROGS_FLAG);
        self.set_origin(flag, origin);
        let (tname, _) = team::team_identity(team);
        let name = self.netname_of(player);
        self.broadcast(PrintLevel::High, &format!("{name} tossed the {tname} flag!\n"));
    }

    /// Drop the flag a player is carrying (called on their death/disconnect), leaving it in the
    /// field to auto-return, be recaptured, or be returned.
    pub(crate) fn drop_flag_if_carrying(&mut self, player: EntId) {
        if self.entities[player].mode_p.ctf.carrying == 0 {
            return;
        }
        match self.carried_flag(player) {
            Some(f) => self.flag_drop(f),
            None => self.entities[player].mode_p.ctf.carrying = 0,
        }
    }

    fn ctf_grab(&mut self, flag: EntId, other: EntId) {
        let team = self.entities[flag].flag.team;
        {
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Carried;
            f.flag.carrier = other;
            f.v.solid = Solid::Not;
            f.v.modelindex = 0.0;
        }
        self.entities[other].mode_p.ctf.carrying = team;
        self.entities[other].mode_p.ctf.flag_since = self.time();
        self.set_flag_effect(other, team, true);
        let name = self.netname_of(other);
        let (tname, _) = team::team_identity(team);
        self.broadcast(PrintLevel::High, &format!("{name} got the {tname} flag!\n"));
        self.host
            .sound(other, Channel::Item, Sound::DOORS_RUNETRY, 1.0, Attenuation::Norm);
    }

    fn ctf_capture(&mut self, carrier: EntId) {
        let team = self.entities[carrier].mode_p.team;
        let idx = team as usize - 1;
        if idx < self.team_match.scores.len() {
            self.team_match.scores[idx] += 1;
        }
        self.entities[carrier].v.frags += CAPTURE_BONUS;
        // Teammates share the capture (+10 each) and cash in any recent return / carrier-frag as an
        // assist; enemies lose their carrier-hurt window (purectf's LoopThroughPlayersAfterCapture).
        let now = self.time();
        for p in players(self) {
            if p == carrier {
                continue;
            }
            if self.entities[p].mode_p.team == team {
                self.entities[p].v.frags += TEAM_CAPTURE_BONUS;
                if self.entities[p].mode_p.ctf.last_returned_flag + RETURN_ASSIST > now {
                    self.entities[p].v.frags += RETURN_ASSIST_BONUS;
                }
                if self.entities[p].mode_p.ctf.last_fragged_carrier + FRAG_CARRIER_ASSIST > now {
                    self.entities[p].v.frags += FRAG_CARRIER_ASSIST_BONUS;
                }
            } else {
                self.entities[p].mode_p.ctf.last_hurt_carrier = -5.0;
            }
        }
        // Send the carried enemy flag home and clear the carry.
        if let Some(f) = self.carried_flag(carrier) {
            self.flag_send_home(f);
        }
        self.entities[carrier].mode_p.ctf.carrying = 0;
        let name = self.netname_of(carrier);
        let (tname, _) = team::team_identity(team);
        let score = self.team_match.scores.get(idx).copied().unwrap_or(0);
        self.broadcast(
            PrintLevel::High,
            &format!("{name} captured the flag! {tname} team: {score}\n"),
        );
        self.host
            .sound(carrier, Channel::Voice, Sound::ITEMS_ITEMBK2, 1.0, Attenuation::None);
    }

    fn ctf_return_flag(&mut self, flag: EntId, returner: EntId) {
        self.flag_send_home(flag);
        self.entities[returner].v.frags += RETURN_BONUS;
        self.entities[returner].mode_p.ctf.last_returned_flag = self.time();
        let name = self.netname_of(returner);
        let (tname, _) = team::team_identity(self.entities[flag].flag.team);
        self.broadcast(PrintLevel::High, &format!("{name} returned the {tname} flag!\n"));
        self.host
            .sound(returner, Channel::Item, Sound::DOORS_RUNETRY, 1.0, Attenuation::Norm);
    }

    /// Clear a flag's carry state (if it has a carrier): drop the carrier's carry bit and its carry
    /// glow. No-op for a flag already at rest.
    fn clear_carrier(&mut self, flag: EntId) {
        let carrier = self.entities[flag].flag.carrier;
        if carrier != EntId::WORLD {
            let team = self.entities[flag].flag.team;
            self.entities[carrier].mode_p.ctf.carrying = 0;
            self.set_flag_effect(carrier, team, false);
        }
    }

    /// Put `flag` down at `pos` in `phase` with launch `velocity`, un-carried: the "touchable Toss
    /// brush showing the flag model" tail shared by drop and send-home.
    fn place_flag(&mut self, flag: EntId, phase: FlagPhase, pos: Vec3, velocity: Vec3) {
        {
            let f = &mut self.entities[flag];
            f.flag.phase = phase;
            f.flag.carrier = EntId::WORLD;
            f.v.solid = Solid::Trigger;
            f.v.movetype = MoveType::Toss;
            f.v.velocity = velocity;
        }
        self.set_model(flag, Model::PROGS_FLAG);
        self.set_origin(flag, pos);
    }

    /// Drop a carried flag where its carrier is, starting the auto-return countdown.
    fn flag_drop(&mut self, flag: EntId) {
        let carrier = self.entities[flag].flag.carrier;
        let origin = if carrier != EntId::WORLD {
            self.entities[carrier].v.origin
        } else {
            self.entities[flag].v.origin
        };
        let team = self.entities[flag].flag.team;
        self.clear_carrier(flag);
        let time = self.time();
        self.place_flag(flag, FlagPhase::Dropped, origin - Vec3::new(0.0, 0.0, 24.0), Vec3::new(0.0, 0.0, 300.0));
        self.entities[flag].flag.return_at = time + FLAG_RETURN_TIME;
        let (tname, _) = team::team_identity(team);
        self.broadcast(PrintLevel::High, &format!("The {tname} flag was dropped!\n"));
    }

    /// Return a flag to its base (from a return, capture, auto-return, or match reset).
    fn flag_send_home(&mut self, flag: EntId) {
        self.clear_carrier(flag);
        let home = self.entities[flag].flag.home;
        self.place_flag(flag, FlagPhase::Home, home, Vec3::ZERO);
    }

    /// Send both flags home and clear every carry — used when a match goes live.
    pub(super) fn reset_flags(&mut self) {
        let flags: Vec<EntId> = self.find_by_classname("flag").collect();
        for f in flags {
            self.flag_send_home(f);
        }
    }

    /// Toggle the `EF_FLAG*` "flag on the back" effect on a carrier.
    fn set_flag_effect(&mut self, carrier: EntId, team: u8, on: bool) {
        let bit = if team == 1 { EF_FLAG1 } else { EF_FLAG2 };
        let e = &mut self.entities[carrier].v.effects;
        let cur = *e as i32;
        *e = (if on { cur | bit } else { cur & !bit }) as f32;
    }

    /// The flag entity carried by `player`, if any.
    fn carried_flag(&self, player: EntId) -> Option<EntId> {
        self.find_by_classname("flag")
            .find(|&f| self.entities[f].flag.carrier == player)
    }
}
