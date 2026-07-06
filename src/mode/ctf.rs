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
//! On top of the base flag game this also ports the purectf extras: the four runes
//! (resistance / strength / haste / regen), the defense/assist frag bonuses, and the voluntary flag
//! and rune toss (impulses 24 / 26, gated by `rtx_ctf_tossflag` / `rtx_ctf_tossrune`). The grapple
//! is handed out by `put_client_in_server` when `rtx_grapple` is on.

use glam::Vec3;

use super::team::{self, match_weapons_hot, team_spawn, tick_lifecycle, MatchMode};
use super::{players, BotIntent, DamageOutcome, GameMode};
use crate::assets::{Model, Sound};
use crate::defs::{
    Attenuation, Channel, MoveType, PrintLevel, Solid, RUNE_HASTE, RUNE_MASK, RUNE_REGEN, RUNE_RESISTANCE,
    RUNE_STRENGTH,
};
use crate::entity::{EntId, FlagPhase, Think, Touch};
use crate::game::GameState;

/// Carrier's bonus frags for a capture (purectf `TEAM_CAPTURE_CAPTURE_BONUS`).
const CAPTURE_BONUS: f32 = 15.0;
/// Bonus frags for returning your own flag (`TEAM_CAPTURE_RECOVERY_BONUS`).
const RETURN_BONUS: f32 = 1.0;
/// Seconds a dropped flag waits before auto-returning (`TEAM_CAPTURE_FLAG_RETURN_TIME`).
const FLAG_RETURN_TIME: f32 = 40.0;
/// Seconds a rune waits, untouched, before relocating to a fresh spawn.
const RUNE_RESPAWN_TIME: f32 = 120.0;
/// `EF_FLAG1`/`EF_FLAG2` — the client-side "flag on the carrier's back" effect bits.
const EF_FLAG1: i32 = 16;
const EF_FLAG2: i32 = 32;

// --- purectf frag bonuses + assist windows (seconds / frags). ---
/// Frag an enemy flag carrier (blocked for the first `CARRIER_FLAG_SINCE` after they grabbed).
const FRAG_CARRIER_BONUS: f32 = 2.0;
const CARRIER_FLAG_SINCE: f32 = 2.0;
/// Frag someone who hurt your carrier within `CARRIER_DANGER` seconds.
const CARRIER_DANGER_PROTECT_BONUS: f32 = 2.0;
const CARRIER_DANGER: f32 = 4.0;
/// Frag near your own carrier / your own flag (within `PROTECT_RADIUS`).
const CARRIER_PROTECT_BONUS: f32 = 1.0;
const FLAG_DEFENSE_BONUS: f32 = 1.0;
const PROTECT_RADIUS: f32 = 400.0;
/// Each teammate's share of a capture, and the assist windows a capture pays out on.
const TEAM_CAPTURE_BONUS: f32 = 10.0;
const RETURN_ASSIST_BONUS: f32 = 1.0;
const RETURN_ASSIST: f32 = 4.0;
const FRAG_CARRIER_ASSIST_BONUS: f32 = 2.0;
const FRAG_CARRIER_ASSIST: f32 = 6.0;

/// The CTF mode descriptor. Stateless — the match lifecycle lives in [`super::MatchState`] (shared
/// with team DM), the flag state on the flag entities.
pub(crate) struct Ctf;

impl GameMode for Ctf {
    fn name(&self) -> &'static str {
        "ctf"
    }

    fn tick(&self, g: &mut GameState) {
        tick_lifecycle(self, g);
    }

    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        team_spawn(g, e)
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        // Team assignment + colours; weapons stay the decoded DM parms (+ the grapple handout,
        // which is CTF's signature movement tool).
        team::assign_team(g, e);
    }

    fn weapons_hot(&self, g: &GameState) -> bool {
        match_weapons_hot(g)
    }

    fn player_damage(
        &self,
        g: &mut GameState,
        targ: EntId,
        attacker: EntId,
        _inflictor: EntId,
        incoming: f32,
    ) -> DamageOutcome {
        // Rune damage: Strength doubles the attacker's outgoing damage (after quad); Resistance
        // halves the target's incoming. Also mark a carrier-defense window when an enemy hurts a
        // flag carrier (see `ctf_frag_bonuses`). Runes exist only in CTF, so this whole rule lives
        // here rather than inline in `t_damage`.
        let mut damage = incoming;
        if g.entities[attacker].mode_p.ctf.runes & RUNE_STRENGTH != 0 {
            damage *= 2.0;
        }
        if g.entities[targ].mode_p.ctf.runes & RUNE_RESISTANCE != 0 {
            damage *= 0.5;
        }
        if g.entities[targ].mode_p.ctf.carrying != 0
            && attacker != targ
            && g.entities[attacker].classname() == Some("player")
            && g.entities[attacker].mode_p.team != g.entities[targ].mode_p.team
        {
            g.entities[attacker].mode_p.ctf.last_hurt_carrier = g.time();
        }
        DamageOutcome::pass(damage)
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        ctf_bot_intent(g, bot)
    }

    fn on_death(&self, g: &mut GameState, victim: EntId, attacker: EntId) {
        ctf_frag_bonuses(g, victim, attacker);
    }

    fn on_client_disconnect(&self, g: &mut GameState, e: EntId) {
        // Drop a carried flag before the slot is retired (its carry marker is about to be cleared).
        // Runes are intentionally *not* dropped on disconnect (only on death).
        g.drop_flag_if_carrying(e);
    }

    fn player_prethink(&self, g: &mut GameState, e: EntId) {
        g.ctf_rune_regen(e); // Regeneration rune's periodic heal
    }

    fn player_died(&self, g: &mut GameState, e: EntId) {
        // A killed carrier drops the flag where they fell; a rune-holder drops their rune.
        g.drop_flag_if_carrying(e);
        g.drop_runes(e);
    }

    fn handle_impulse(&self, g: &mut GameState, e: EntId, impulse: i32) -> bool {
        match impulse {
            24 => {
                g.toss_rune(e); // drop your held rune
                true
            }
            26 => {
                g.toss_flag(e); // toss the enemy flag you carry
                true
            }
            _ => false,
        }
    }

    fn handle_command(&self, g: &mut GameState, _e: EntId, cmd: &str) -> bool {
        team::match_handle_command(g, cmd)
    }

    fn attack_cooldown_scale(&self, g: &GameState, e: EntId) -> f32 {
        // Haste rune: fire ~2× as fast.
        if g.entities[e].mode_p.ctf.runes & RUNE_HASTE != 0 {
            0.5
        } else {
            1.0
        }
    }
}

impl MatchMode for Ctf {
    fn on_go_live(&self, g: &mut GameState) {
        // Fresh slate: zero the two capture scores, send both flags home, and (re)spawn the runes.
        g.team_match.scores = vec![0; 2];
        g.reset_flags();
        g.spawn_runes();
    }

    fn limit_reached(&self, g: &mut GameState) -> bool {
        // Capture scores are updated on capture events (`ctf_capture`); here we only test the limit.
        let cap = g.host().cvar(c"rtx_capturelimit").max(0.0) as i32;
        cap > 0 && g.team_match.scores.iter().any(|&s| s >= cap)
    }

    fn announce_result(&self, g: &mut GameState) {
        self.end_match(g);
    }
}

/// purectf's kill-time CTF bonuses (obituary side): fragging the enemy carrier, protecting your own
/// carrier / flag by fragging attackers near them. Runs after the stock obituary credited the +1.
fn ctf_frag_bonuses(g: &mut GameState, victim: EntId, attacker: EntId) {
    if attacker == victim
        || g.entities[attacker].classname() != Some("player")
        || g.entities[victim].classname() != Some("player")
    {
        return;
    }
    let now = g.time();
    let a_team = g.entities[attacker].mode_p.team;
    let v_team = g.entities[victim].mode_p.team;
    if a_team == 0 {
        return;
    }
    let mut protected_carrier = false;

    // Fragged the enemy flag carrier.
    if g.entities[victim].mode_p.ctf.carrying != 0 && a_team != v_team {
        g.entities[attacker].mode_p.ctf.last_fragged_carrier = now;
        if g.entities[victim].mode_p.ctf.flag_since + CARRIER_FLAG_SINCE <= now {
            g.entities[attacker].v.frags += FRAG_CARRIER_BONUS;
        }
    }
    // Fragged someone who recently hurt your carrier (and you aren't the carrier yourself).
    if g.entities[victim].mode_p.ctf.last_hurt_carrier + CARRIER_DANGER > now && g.entities[attacker].mode_p.ctf.carrying == 0 {
        g.entities[attacker].v.frags += CARRIER_DANGER_PROTECT_BONUS;
        protected_carrier = true;
    }

    // Proximity scans around the attacker and the victim: a teammate carrier nearby (carrier
    // protect, once) and your own flag nearby (flag defense, once).
    let mut flag_defended = false;
    let centers = [g.entities[attacker].v.origin, g.entities[victim].v.origin];
    for center in centers {
        for head in g.find_radius(center, PROTECT_RADIUS) {
            if !protected_carrier
                && head != attacker
                && g.entities[head].classname() == Some("player")
                && g.entities[head].mode_p.team == a_team
                && g.entities[head].mode_p.ctf.carrying != 0
            {
                g.entities[attacker].v.frags += CARRIER_PROTECT_BONUS;
                protected_carrier = true;
            }
            if !flag_defended
                && g.entities[head].flag.team == a_team
                && matches!(g.entities[head].flag.phase, FlagPhase::Home)
            {
                g.entities[attacker].v.frags += FLAG_DEFENSE_BONUS;
                flag_defended = true;
            }
        }
    }
}

impl Ctf {
    /// Broadcast the CTF result (red : blue captures). The post-match pause is entered by the shared
    /// [`tick_lifecycle`] machine.
    fn end_match(&self, g: &mut GameState) {
        let red = g.team_match.scores.first().copied().unwrap_or(0);
        let blue = g.team_match.scores.get(1).copied().unwrap_or(0);
        g.broadcast(PrintLevel::High, &format!("Match over — red {red} : {blue} blue\n"));
    }
}

/// How close an enemy must be for an attacker to break off and fight instead of pushing the flag.
const ATTACK_ENGAGE: f32 = 500.0;
/// How close to our base an enemy must be for a defender to leave the flag and engage.
const DEFEND_RADIUS: f32 = 700.0;

/// Split a bot's job on its team. Roles are what turn "every bot rushes the same flag" into a team:
/// most bots [`Attack`](CtfRole::Attack) (grab and run the enemy flag), a minority
/// [`Defend`](CtfRole::Defend) (hold the base, retrieve the flag, intercept attackers).
#[derive(Clone, Copy, PartialEq, Eq)]
enum CtfRole {
    Attack,
    Defend,
}

/// Assign `bot` a stable role among its team's bots: sort the team's bots by edict id and make the
/// lowest ~third (at least one, once a team has two) defenders, the rest attackers. Deterministic, so
/// a bot keeps its role frame to frame as long as the roster holds; a lone bot always attacks.
fn ctf_role(g: &GameState, bot: EntId, team: u8) -> CtfRole {
    let mut mates: Vec<EntId> = players(g)
        .into_iter()
        .filter(|&e| g.entities[e].mode_p.team == team && g.entities[e].bot.is_bot)
        .collect();
    mates.sort_unstable_by_key(|e| e.0);
    let defenders = if mates.len() <= 1 { 0 } else { (mates.len() / 3).max(1) };
    let rank = mates.iter().position(|&e| e == bot).unwrap_or(0);
    if rank < defenders {
        CtfRole::Defend
    } else {
        CtfRole::Attack
    }
}

/// A living teammate (other than `bot`) carrying the enemy flag — the runner an attacker escorts.
fn team_carrier(g: &GameState, team: u8, bot: EntId) -> Option<EntId> {
    players(g).into_iter().find(|&e| {
        let ent = &g.entities[e];
        e != bot && ent.mode_p.team == team && ent.v.health > 0.0 && ent.mode_p.ctf.carrying != 0
    })
}

/// A per-bot hold post ~150u around the flag (one of three, by id), so defenders spread to cover
/// approaches instead of stacking on the exact flag point.
fn defender_offset(bot: EntId) -> Vec3 {
    let a = (bot.0 % 3) as f32 * std::f32::consts::TAU / 3.0;
    Vec3::new(a.cos(), a.sin(), 0.0) * 150.0
}

/// The team-aware CTF bot brain. Carrying the flag always overrides to a run home; otherwise the
/// bot's [`CtfRole`] picks between pushing the enemy flag and holding our own base. All navigation is
/// the generic `Move`/`Fight` seam.
fn ctf_bot_intent(g: &mut GameState, bot: EntId) -> Option<BotIntent> {
    let team = g.entities[bot].mode_p.team;
    if team == 0 {
        return None;
    }
    let origin = g.entities[bot].v.origin;
    let teamwork = g.host().cvar_bool(c"rtx_bot_teamwork");

    // Carrying the enemy flag → run to our base to capture (all roles; don't stop to fight).
    if g.entities[bot].mode_p.ctf.carrying != 0 {
        if let Some(hf) = base_flag(g, team) {
            return Some(BotIntent::Move(g.entities[hf].flag.home));
        }
    }

    match ctf_role(g, bot, team) {
        CtfRole::Defend => {
            if let Some(of) = base_flag(g, team) {
                let (carrier, phase, flag_pos, home) = {
                    let f = &g.entities[of].flag;
                    (f.carrier, f.phase, g.entities[of].v.origin, f.home)
                };
                // Our flag stolen → hunt down the carrier to bring it back.
                if carrier != EntId::WORLD {
                    return Some(BotIntent::Fight(carrier));
                }
                // An enemy pushing our base → intercept it before it reaches the flag.
                if let Some(en) = team::nearest_enemy_to(g, team, home) {
                    if (g.entities[en].v.origin - home).length_squared() < DEFEND_RADIUS * DEFEND_RADIUS {
                        return Some(BotIntent::Fight(en));
                    }
                }
                // Flag knocked out into the field → go stand on it to return it; else hold at home.
                // When holding, stagger defenders to per-bot posts around the flag so they cover
                // approaches instead of stacking on the exact spot.
                let target = if matches!(phase, FlagPhase::Home) {
                    if teamwork {
                        home + defender_offset(bot)
                    } else {
                        home
                    }
                } else {
                    flag_pos
                };
                return Some(BotIntent::Move(target));
            }
        }
        CtfRole::Attack => {
            // A close enemy in the way → fight it (escorts included), otherwise move.
            if let Some(en) = team::nearest_enemy(g, bot) {
                if (g.entities[en].v.origin - origin).length_squared() < ATTACK_ENGAGE * ATTACK_ENGAGE {
                    return Some(BotIntent::Fight(en));
                }
            }
            // No close enemy: half the attackers (by id parity) escort a teammate flag carrier home
            // — trailing between the runner and the enemy base as a rearguard — while the rest keep
            // pushing the enemy flag. Splits the team into capture pressure *and* carrier protection.
            if teamwork && bot.0.is_multiple_of(2) {
                if let Some(carrier) = team_carrier(g, team, bot) {
                    let carrier_org = g.entities[carrier].v.origin;
                    let home = base_flag(g, team).map_or(carrier_org, |f| g.entities[f].flag.home);
                    let back = (carrier_org - home).normalize_or_zero();
                    return Some(BotIntent::Move(carrier_org + back * 150.0));
                }
            }
            if let Some(ef) = enemy_flag(g, team) {
                return Some(BotIntent::Move(g.entities[ef].v.origin));
            }
        }
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
        self.entities[player].mode_p.ctf.carrying = 0;
        self.set_flag_effect(player, team, false);
        {
            let f = &mut self.entities[flag];
            f.flag.phase = FlagPhase::Tossed;
            f.flag.carrier = player; // lock reference (the tosser)
            f.flag.return_at = time + 2.0;
            f.v.solid = Solid::Trigger;
            f.v.movetype = MoveType::Toss;
            f.v.velocity = dir * 300.0;
        }
        self.host.set_model(flag, Model::PROGS_FLAG);
        self.host.set_origin(flag, origin);
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
            self.entities[carrier].mode_p.ctf.carrying = 0;
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
            self.entities[carrier].mode_p.ctf.carrying = 0;
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
    fn reset_flags(&mut self) {
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

    // --- runes (purectf): one held per player, dropped/tossable, four game runes. ---

    /// Clear any runes and spawn the four game runes at random deathmatch spawns. Called when a CTF
    /// match goes live; a no-op if `rtx_runes` is off.
    pub(crate) fn spawn_runes(&mut self) {
        let existing: Vec<EntId> = self.find_by_classname("item_rune").collect();
        for r in existing {
            self.free(r);
        }
        for p in players(self) {
            self.entities[p].mode_p.ctf.runes = 0;
            self.refresh_haste_speed(p);
        }
        if self.mode.name() != "ctf" || self.host.cvar(c"rtx_runes") as i32 == 1 {
            return; // 1 = runes off
        }
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            let spot = self.select_spawn_point();
            if spot != EntId::WORLD {
                let org = self.entities[spot].v.origin;
                self.do_spawn_rune(org, bit);
            }
        }
    }

    /// Create one rune item (`item_rune`) near `origin`.
    fn do_spawn_rune(&mut self, origin: Vec3, bit: u8) -> EntId {
        let e = self.spawn();
        let (model, msg) = rune_asset(bit);
        {
            let ent = &mut self.entities[e];
            ent.classname = Some("item_rune".into());
            ent.item.rune_bit = bit; // which rune this item is
            ent.netname = Some(msg.into());
            ent.v.movetype = MoveType::Toss;
            ent.v.solid = Solid::Trigger;
        }
        self.host.set_model(e, model);
        self.host
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        let jx = -500.0 + self.random() * 1000.0;
        let jy = -500.0 + self.random() * 1000.0;
        let time = self.time();
        {
            let ent = &mut self.entities[e];
            ent.v.velocity = Vec3::new(jx, jy, 300.0);
            ent.think = Think::RuneRespawn;
            ent.v.nextthink = time + RUNE_RESPAWN_TIME;
        }
        self.entities[e].set_touch(Touch::Rune);
        self.host.set_origin(e, origin + Vec3::new(0.0, 0.0, 4.0));
        e
    }

    /// `Touch::Rune` — pick up a rune (one per player; a held rune blocks the pickup).
    pub(crate) fn rune_touch(&mut self, rune: EntId, other: EntId) {
        if other == self.entities[rune].owner()
            || self.entities[other].classname() != Some("player")
            || self.entities[other].v.health <= 0.0
            || self.entities[other].v.deadflag != 0.0
        {
            return;
        }
        if self.entities[other].mode_p.ctf.runes & RUNE_MASK != 0 {
            self.sprint_to(other, c"You already have a rune (tossrune to drop).\n");
            return;
        }
        let bit = self.entities[rune].item.rune_bit;
        self.entities[other].mode_p.ctf.runes |= bit;
        self.refresh_haste_speed(other);
        self.host
            .sound(other, Channel::Item, Sound::WEAPONS_LOCK4, 1.0, Attenuation::Norm);
        self.sprint_to(other, c"You got a rune!\n");
        self.free(rune);
    }

    /// `Think::RuneRespawn` — an untouched rune relocates to a fresh spawn.
    pub(crate) fn rune_respawn(&mut self, rune: EntId) {
        let bit = self.entities[rune].item.rune_bit;
        let spot = self.select_spawn_point();
        let org = if spot != EntId::WORLD {
            self.entities[spot].v.origin
        } else {
            self.entities[rune].v.origin
        };
        self.free(rune);
        self.do_spawn_rune(org, bit);
    }

    /// Drop the runes a player holds where they are (called on death; owner-locked briefly).
    pub(crate) fn drop_runes(&mut self, player: EntId) {
        let runes = self.entities[player].mode_p.ctf.runes & RUNE_MASK;
        if runes == 0 {
            return;
        }
        let origin = self.entities[player].v.origin;
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            if runes & bit != 0 {
                self.do_spawn_rune(origin, bit);
            }
        }
        self.entities[player].mode_p.ctf.runes = 0;
        self.refresh_haste_speed(player);
    }

    /// Impulse 24 — toss your held rune(s) along your aim (gated by `rtx_ctf_tossrune`).
    pub(crate) fn toss_rune(&mut self, player: EntId) {
        // Reached only via `Ctf::handle_impulse`; just the cvar gate remains.
        if !self.host.cvar_bool(c"rtx_ctf_tossrune") {
            return;
        }
        let runes = self.entities[player].mode_p.ctf.runes & RUNE_MASK;
        if runes == 0 {
            return;
        }
        let origin = self.entities[player].v.origin;
        let dir = self.aim_dir(player);
        for bit in [RUNE_RESISTANCE, RUNE_STRENGTH, RUNE_HASTE, RUNE_REGEN] {
            if runes & bit != 0 {
                let r = self.do_spawn_rune(origin, bit);
                self.entities[r].v.velocity = dir * 300.0 + Vec3::new(0.0, 0.0, 200.0);
            }
        }
        self.entities[player].mode_p.ctf.runes = 0;
        self.refresh_haste_speed(player);
    }

    /// Recompute a player's move-speed cap for the Haste rune (`×1.25` when held, `rtx_runes 0`).
    pub(crate) fn refresh_haste_speed(&mut self, e: EntId) {
        let base = {
            let v = self.host.cvar(c"sv_maxspeed");
            if v > 0.0 {
                v
            } else {
                320.0
            }
        };
        // Only the "pure" mode (`rtx_runes 0`) grants the speed boost; `2` is haste-attack only.
        let pure = self.host.cvar(c"rtx_runes") as i32 == 0;
        let haste = pure && self.entities[e].mode_p.ctf.runes & RUNE_HASTE != 0;
        self.entities[e].maxspeed = base * if haste { 1.25 } else { 1.0 };
    }

    /// Regeneration rune: heal health/armor toward 150 while held (called each frame in prethink).
    pub(crate) fn ctf_rune_regen(&mut self, e: EntId) {
        if self.entities[e].mode_p.ctf.runes & RUNE_REGEN == 0 {
            return;
        }
        let dt = self.globals.frametime;
        let v = &mut self.entities[e].v;
        if v.health > 0.0 && v.health < 150.0 {
            v.health = (v.health + 10.0 * dt).min(150.0);
        }
        if v.armortype > 0.0 && v.armorvalue < 150.0 {
            v.armorvalue = (v.armorvalue + 10.0 * dt).min(150.0);
        }
    }
}

/// The `(model, pickup message)` for a rune bit.
fn rune_asset(bit: u8) -> (Model, &'static str) {
    match bit {
        RUNE_RESISTANCE => (Model::PROGS_END1, "Resistance"),
        RUNE_STRENGTH => (Model::PROGS_END2, "Strength"),
        RUNE_HASTE => (Model::PROGS_END3, "Haste"),
        _ => (Model::PROGS_END4, "Regeneration"),
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
