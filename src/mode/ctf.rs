// SPDX-License-Identifier: AGPL-3.0-or-later

//! Capture the Flag (`rtx_mode ctf`) — a purectf-modeled flag game built on the generic team
//! layer ([`super::team`]).
//!
//! Two teams (red / blue) each own a flag at a base. Grab the **enemy** flag, carry it to **your**
//! base while **your** flag is home, and it's a **capture** (+1 to your team's capture count, +15
//! frags to the carrier). Touch your **own** flag where it lies in the field to **return** it (+1);
//! a dropped flag also **auto-returns** after 40 s, and a killed carrier **drops** it where they
//! died. Teams win at `rtx_capturelimit`. Friendly fire follows `teamplay`.
//!
//! CTF reuses the team layer's lifecycle wholesale: the warmup → **`start`** (map reload +
//! countdown) → live → results machine, the locked roster + reconnect-reattach, team assignment,
//! colours and `info_player_teamN` spawns all come from [`super::team`] via [`is_match_mode`]. Only
//! the win condition (captures, not frags) and the flag entities are CTF-specific.
//!
//! Deferred vs purectf (kept simple for a first cut): runes, the proximity defense/assist bonuses,
//! the voluntary flag toss, and the grapple-only extras (the grapple itself is already handed out by
//! `put_client_in_server` when `rtx_grapple` is on).

use glam::Vec3;

use super::team;
use super::{BotIntent, GameMode, MatchPhase};
use crate::assets::{Model, Sound};
use crate::defs::{Attenuation, Channel, MoveType, PrintLevel, Solid};
use crate::entity::{EntId, FlagPhase, Think, Touch};
use crate::game::GameState;

/// Carrier's bonus frags for a capture (purectf `TEAM_CAPTURE_CAPTURE_BONUS`).
const CAPTURE_BONUS: f32 = 15.0;
/// Bonus frags for returning your own flag (`TEAM_CAPTURE_RECOVERY_BONUS`).
const RETURN_BONUS: f32 = 1.0;
/// Seconds a dropped flag waits before auto-returning (`TEAM_CAPTURE_FLAG_RETURN_TIME`).
const FLAG_RETURN_TIME: f32 = 40.0;
/// Results-screen pause before returning to warmup / rotating.
const END_PAUSE: f32 = 5.0;
/// `EF_FLAG1`/`EF_FLAG2` — the client-side "flag on the carrier's back" effect bits.
const EF_FLAG1: i32 = 16;
const EF_FLAG2: i32 = 32;

/// The CTF mode descriptor. Stateless — the match lifecycle lives in [`super::MatchState`] (shared
/// with team DM), the flag state on the flag entities.
pub(crate) struct Ctf;

impl GameMode for Ctf {
    fn name(&self) -> &'static str {
        "ctf"
    }

    fn tick(&self, g: &mut GameState) {
        let now = g.time();
        match g.team_match.phase {
            MatchPhase::Warmup => {} // playable; team assignment happens on spawn (apply_loadout)
            MatchPhase::Countdown { until } => {
                let remaining = (until - now).ceil() as i32;
                if remaining != g.team_match.last_count {
                    g.team_match.last_count = remaining;
                    if remaining > 0 {
                        team::centerprint_all(g, &format!("{remaining}"));
                    }
                }
                if now >= until {
                    // Go live on a clean slate: reset capture scores and both flags, arm timelimit.
                    g.team_match.scores = vec![0; 2];
                    let tl = g.level.timelimit;
                    g.team_match.live_until = if tl > 0 { now + tl as f32 } else { 0.0 };
                    reset_flags(g);
                    g.team_match.phase = MatchPhase::Live;
                    team::centerprint_all(g, "FIGHT!");
                }
            }
            MatchPhase::Live => {
                // Capture scores are updated on capture events (`ctf_capture`); here we only check
                // the end conditions.
                let cap = g.host().cvar(c"rtx_capturelimit").max(0.0) as i32;
                let hit = cap > 0 && g.team_match.scores.iter().any(|&s| s >= cap);
                let time_up = g.team_match.live_until > 0.0 && now >= g.team_match.live_until;
                if hit || time_up {
                    self.end_match(g, now);
                }
            }
            MatchPhase::Ended { until } => {
                if now >= until {
                    if g.queued_next_map().is_some() {
                        g.next_level();
                    } else {
                        g.team_match.phase = MatchPhase::Warmup;
                        g.team_match.roster.clear();
                    }
                }
            }
        }
    }

    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        let team = g.entities[e].arena.team;
        if team >= 1 {
            let spot = g.select_spawn_point_of(&format!("info_player_team{team}"));
            if spot != EntId::WORLD {
                return spot;
            }
        }
        g.select_spawn_point()
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        // Team assignment + colours; weapons stay the decoded DM parms (+ the grapple handout,
        // which is CTF's signature movement tool).
        team::assign_team(g, e);
    }

    fn weapons_hot(&self, g: &GameState) -> bool {
        !matches!(g.team_match.phase, MatchPhase::Countdown { .. })
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        ctf_bot_intent(g, bot)
    }
}

impl Ctf {
    fn end_match(&self, g: &mut GameState, now: f32) {
        let red = g.team_match.scores.first().copied().unwrap_or(0);
        let blue = g.team_match.scores.get(1).copied().unwrap_or(0);
        g.broadcast(PrintLevel::High, &format!("Match over — red {red} : {blue} blue\n"));
        g.team_match.phase = MatchPhase::Ended { until: now + END_PAUSE };
    }
}

/// The team-aware CTF bot brain: run a captured flag home; fight a close enemy; retrieve a stolen
/// own flag; otherwise go grab the enemy flag. All navigation is the generic `Move`/`Fight` seam.
fn ctf_bot_intent(g: &mut GameState, bot: EntId) -> Option<BotIntent> {
    let team = g.entities[bot].arena.team;
    if team == 0 {
        return None;
    }
    let origin = g.entities[bot].v.origin;

    // Carrying the enemy flag → run to our base to capture (don't stop to fight).
    if g.entities[bot].arena.carrying != 0 {
        if let Some(hf) = base_flag(g, team) {
            return Some(BotIntent::Move(g.entities[hf].flag.home));
        }
    }
    // A close enemy in the way → fight it.
    if let Some(en) = team::nearest_enemy(g, bot) {
        if (g.entities[en].v.origin - origin).length_squared() < 500.0 * 500.0 {
            return Some(BotIntent::Fight(en));
        }
    }
    // Our flag stolen → hunt the carrier.
    if let Some(of) = base_flag(g, team) {
        let carrier = g.entities[of].flag.carrier;
        if carrier != EntId::WORLD {
            return Some(BotIntent::Fight(carrier));
        }
    }
    // Otherwise head for the enemy flag to grab it.
    if let Some(ef) = enemy_flag(g, team) {
        return Some(BotIntent::Move(g.entities[ef].v.origin));
    }
    team::nearest_enemy(g, bot).map(BotIntent::Fight)
}

// --- flag entities + lifecycle (GameState methods; dispatched from spawn/touch/think) ---

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
        if self.mode.name() != "ctf" {
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
        self.host.set_model(e, Model::PROGS_FLAG);
        self.host
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 74.0));
        // Settle to the floor (best effort) and remember the base position for returns.
        self.entities[e].v.origin.z += 6.0;
        let _ = self.host.droptofloor(e);
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
        if self.entities[other].classname() != Some("player")
            || self.entities[other].v.health <= 0.0
            || self.entities[other].v.deadflag != 0.0
        {
            return;
        }
        if self.entities[flag].flag.phase == FlagPhase::Carried {
            return;
        }
        let flag_team = self.entities[flag].flag.team;
        let player_team = self.entities[other].arena.team;
        if flag_team == 0 || player_team == 0 {
            return;
        }
        if player_team == flag_team {
            if self.entities[flag].flag.phase == FlagPhase::Home {
                // Own flag home: capture if we're carrying the enemy flag.
                if self.entities[other].arena.carrying != 0 {
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

    /// `Think::FlagReturn` — idle flag tick: auto-return a dropped flag once its timeout elapses.
    pub(crate) fn flag_return_think(&mut self, flag: EntId) {
        let time = self.time();
        self.entities[flag].v.nextthink = time + 0.5;
        if self.entities[flag].flag.phase == FlagPhase::Dropped && time >= self.entities[flag].flag.return_at {
            self.flag_send_home(flag);
            let (name, _) = team::team_identity(self.entities[flag].flag.team);
            self.broadcast(PrintLevel::High, &format!("The {name} flag returned to base.\n"));
        }
    }

    /// Drop the flag a player is carrying (called on their death/disconnect), leaving it in the
    /// field to auto-return, be recaptured, or be returned.
    pub(crate) fn drop_flag_if_carrying(&mut self, player: EntId) {
        if self.entities[player].arena.carrying == 0 {
            return;
        }
        match self.carried_flag(player) {
            Some(f) => self.flag_drop(f),
            None => self.entities[player].arena.carrying = 0,
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
        self.entities[other].arena.carrying = team;
        self.set_flag_effect(other, team, true);
        let name = self.netname_of(other);
        let (tname, _) = team::team_identity(team);
        self.broadcast(PrintLevel::High, &format!("{name} got the {tname} flag!\n"));
        self.host
            .sound(other, Channel::Item, Sound::DOORS_RUNETRY, 1.0, Attenuation::Norm);
    }

    fn ctf_capture(&mut self, carrier: EntId) {
        let team = self.entities[carrier].arena.team;
        let idx = team as usize - 1;
        if idx < self.team_match.scores.len() {
            self.team_match.scores[idx] += 1;
        }
        self.entities[carrier].v.frags += CAPTURE_BONUS;
        // Send the carried enemy flag home and clear the carry.
        if let Some(f) = self.carried_flag(carrier) {
            self.flag_send_home(f);
        }
        self.entities[carrier].arena.carrying = 0;
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
        let name = self.netname_of(returner);
        let (tname, _) = team::team_identity(self.entities[flag].flag.team);
        self.broadcast(PrintLevel::High, &format!("{name} returned the {tname} flag!\n"));
        self.host
            .sound(returner, Channel::Item, Sound::DOORS_RUNETRY, 1.0, Attenuation::Norm);
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
        if carrier != EntId::WORLD {
            self.entities[carrier].arena.carrying = 0;
            self.set_flag_effect(carrier, team, false);
        }
        let time = self.time();
        let pos = origin - Vec3::new(0.0, 0.0, 24.0);
        {
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Dropped;
            f.flag.carrier = EntId::WORLD;
            f.flag.return_at = time + FLAG_RETURN_TIME;
            f.v.solid = Solid::Trigger;
            f.v.movetype = MoveType::Toss;
            f.v.velocity = Vec3::new(0.0, 0.0, 300.0);
        }
        self.host.set_model(flag, Model::PROGS_FLAG);
        self.host.set_origin(flag, pos);
        let (tname, _) = team::team_identity(team);
        self.broadcast(PrintLevel::High, &format!("The {tname} flag was dropped!\n"));
    }

    /// Return a flag to its base (from a return, capture, auto-return, or match reset).
    fn flag_send_home(&mut self, flag: EntId) {
        let carrier = self.entities[flag].flag.carrier;
        let team = self.entities[flag].flag.team;
        if carrier != EntId::WORLD {
            self.entities[carrier].arena.carrying = 0;
            self.set_flag_effect(carrier, team, false);
        }
        let home = self.entities[flag].flag.home;
        {
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Home;
            f.flag.carrier = EntId::WORLD;
            f.v.solid = Solid::Trigger;
            f.v.movetype = MoveType::Toss;
            f.v.velocity = Vec3::ZERO;
        }
        self.host.set_model(flag, Model::PROGS_FLAG);
        self.host.set_origin(flag, home);
    }

    /// Send both flags home and clear every carry — used when a match goes live.
    fn reset_flags_impl(&mut self) {
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

/// Team `team`'s own flag entity.
fn base_flag(g: &GameState, team: u8) -> Option<EntId> {
    g.find_by_classname("flag").find(|&f| g.entities[f].flag.team == team)
}

/// The enemy flag (the one not owned by `team`).
fn enemy_flag(g: &GameState, team: u8) -> Option<EntId> {
    g.find_by_classname("flag").find(|&f| {
        let t = g.entities[f].flag.team;
        t != 0 && t != team
    })
}

/// Send both flags home (free function so `Ctf::tick` can call it without a `self` borrow clash).
fn reset_flags(g: &mut GameState) {
    g.reset_flags_impl();
}
