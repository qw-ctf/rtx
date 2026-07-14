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
//! CTF reuses the composition layer's lifecycle wholesale: the warmup → **`start`** (map reload +
//! countdown) → live → results machine, the locked roster + reconnect-reattach, team assignment,
//! colours and `info_player_teamN` spawns all come from [`super::team`] (CTF resolves to a 2-team
//! composition). Only the win condition (captures, not frags — the `on_match_*` hooks) and the flag
//! entities are CTF-specific.
//!
//! On top of the base flag game this also ports the purectf extras: the four runes
//! (resistance / strength / haste / regen), the defense/assist frag bonuses, and the voluntary flag
//! and rune toss (impulses 24 / 26, gated by `rtx_ctf_tossflag` / `rtx_ctf_tossrune`). The grapple
//! is handed out by `put_client_in_server` when `rtx_grapple` is on.

use glam::Vec3;

use super::team;
use super::{players, BotIntent, DamageOutcome, GameMode};
use crate::defs::{PrintLevel, Solid, RUNE_HASTE, RUNE_RESISTANCE, RUNE_STRENGTH};
use crate::entity::{EntId, FlagPhase};
use crate::game::GameState;

mod flags;
mod runes;

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

    fn uses_ctf_objects(&self) -> bool {
        true
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
            && g.entities[attacker].is_player()
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

    fn attack_cooldown_scale(&self, g: &GameState, e: EntId) -> f32 {
        // Haste rune: fire ~2× as fast.
        if g.entities[e].mode_p.ctf.runes & RUNE_HASTE != 0 {
            0.5
        } else {
            1.0
        }
    }

    fn on_match_go_live(&self, g: &mut GameState) {
        // Fresh slate: zero the two capture scores, send both flags home, and (re)spawn the runes.
        g.team_match.scores = vec![0; 2];
        g.reset_flags();
        g.spawn_runes();
    }

    fn match_limit_reached(&self, g: &mut GameState) -> bool {
        // Capture scores are updated on capture events (`ctf_capture`); here we only test the limit.
        let cap = g.host().cvar(c"rtx_capturelimit").max(0.0) as i32;
        cap > 0 && g.team_match.scores.iter().any(|&s| s >= cap)
    }

    fn announce_match_result(&self, g: &mut GameState) {
        self.end_match(g);
    }
}

/// purectf's kill-time CTF bonuses (obituary side): fragging the enemy carrier, protecting your own
/// carrier / flag by fragging attackers near them. Runs after the stock obituary credited the +1.
fn ctf_frag_bonuses(g: &mut GameState, victim: EntId, attacker: EntId) {
    if attacker == victim || !g.entities[attacker].is_player() || !g.entities[victim].is_player() {
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
                && g.entities[head].is_player()
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
/// Midfielders contest enemies crossing the central powerup/rune lane inside this radius.
const MIDFIELD_ENGAGE: f32 = 650.0;

/// Split a bot's job on its team. Roles are what turn "every bot rushes the same flag" into a team:
/// most bots [`Attack`](CtfRole::Attack) (grab and run the enemy flag), a minority
/// [`Defend`](CtfRole::Defend) (hold the base, retrieve the flag, intercept attackers).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CtfRole {
    Attack,
    Midfield,
    Defend,
}

/// Stable role distribution by sorted team-bot rank. A solo bot attacks; two split attack/defense;
/// larger teams keep roughly one defender per three players and add one midfielder to contest
/// powerups, runes, and crossings; everyone else attacks.
fn role_for_rank(size: usize, rank: usize) -> CtfRole {
    if size <= 1 {
        return CtfRole::Attack;
    }
    let defenders = (size / 3).max(1);
    if rank < defenders {
        CtfRole::Defend
    } else if size >= 3 && rank == defenders {
        CtfRole::Midfield
    } else {
        CtfRole::Attack
    }
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
    let rank = mates.iter().position(|&e| e == bot).unwrap_or(0);
    role_for_rank(mates.len(), rank)
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

/// Midpoint between both flag homes (or the available base), used as the soft midfield anchor.
fn midfield_point(g: &GameState, team: u8, fallback: Vec3) -> Vec3 {
    match (base_flag(g, team), enemy_flag(g, team)) {
        (Some(ours), Some(theirs)) => (g.entities[ours].flag.home + g.entities[theirs].flag.home) * 0.5,
        (Some(ours), None) => g.entities[ours].flag.home,
        (None, Some(theirs)) => g.entities[theirs].flag.home,
        (None, None) => fallback,
    }
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

    // Carrying the enemy flag → run to our base to capture (all roles; don't stop to fight).
    if g.entities[bot].mode_p.ctf.carrying != 0 {
        if let Some(hf) = base_flag(g, team) {
            return Some(BotIntent::Move(g.entities[hf].flag.home));
        }
    }

    // A real, recent teammate damage call may pull the nearest responder off their normal lane.
    // Carrier calls get two responders; ordinary calls only one (see team::help_target).
    if let Some(attacker) = team::help_target(g, bot) {
        return Some(BotIntent::Fight(attacker));
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
                    home + defender_offset(bot)
                } else {
                    flag_pos
                };
                return Some(if matches!(phase, FlagPhase::Home) {
                    BotIntent::Advance(target)
                } else {
                    BotIntent::Move(target)
                });
            }
        }
        CtfRole::Midfield => {
            let mid = midfield_point(g, team, origin);
            // Help the carrier through the central lane before looking for a fresh contest.
            if let Some(carrier) = team_carrier(g, team, bot) {
                let carrier_org = g.entities[carrier].v.origin;
                let home = base_flag(g, team).map_or(carrier_org, |f| g.entities[f].flag.home);
                let rear = (carrier_org - home).normalize_or_zero();
                return Some(BotIntent::Advance(carrier_org + rear * 180.0));
            }
            if let Some(en) = team::nearest_enemy_to(g, team, mid) {
                if (g.entities[en].v.origin - mid).length_squared() < MIDFIELD_ENGAGE * MIDFIELD_ENGAGE {
                    return Some(BotIntent::Fight(en));
                }
            }
            // Advance is deliberately soft: the shared item planner may insert quad/pent/runes or
            // useful stack on the way, then resumes this central anchor.
            return Some(BotIntent::Advance(mid));
        }
        CtfRole::Attack => {
            // Once the flag is within the final few steps, finish the touch before accepting a
            // nearby duel. This is the flag equivalent of the shared critical-pickup commitment.
            if let Some(ef) = enemy_flag(g, team) {
                let flag_org = g.entities[ef].v.origin;
                if g.entities[ef].v.solid == Solid::Trigger
                    && (flag_org - origin).length_squared() < 256.0 * 256.0
                {
                    return Some(BotIntent::Move(flag_org));
                }
            }
            // A close enemy in the way → fight it (escorts included), otherwise move.
            if let Some(en) = team::nearest_enemy(g, bot) {
                if (g.entities[en].v.origin - origin).length_squared() < ATTACK_ENGAGE * ATTACK_ENGAGE {
                    return Some(BotIntent::Fight(en));
                }
            }
            // No close enemy: half the attackers (by id parity) escort a teammate flag carrier home
            // — trailing between the runner and the enemy base as a rearguard — while the rest keep
            // pushing the enemy flag. Splits the team into capture pressure *and* carrier protection.
            if bot.0.is_multiple_of(2) {
                if let Some(carrier) = team_carrier(g, team, bot) {
                    let carrier_org = g.entities[carrier].v.origin;
                    let home = base_flag(g, team).map_or(carrier_org, |f| g.entities[f].flag.home);
                    let back = (carrier_org - home).normalize_or_zero();
                    return Some(BotIntent::Advance(carrier_org + back * 150.0));
                }
            }
            if let Some(ef) = enemy_flag(g, team) {
                return Some(BotIntent::Advance(g.entities[ef].v.origin));
            }
        }
    }
    team::nearest_enemy(g, bot).map(BotIntent::Fight)
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

#[cfg(test)]
mod tests {
    use super::{role_for_rank, CtfRole};

    #[test]
    fn ctf_roles_add_midfield_without_abandoning_attack() {
        assert_eq!(role_for_rank(1, 0), CtfRole::Attack);
        assert_eq!(role_for_rank(2, 0), CtfRole::Defend);
        assert_eq!(role_for_rank(2, 1), CtfRole::Attack);
        assert_eq!(role_for_rank(3, 0), CtfRole::Defend);
        assert_eq!(role_for_rank(3, 1), CtfRole::Midfield);
        assert_eq!(role_for_rank(3, 2), CtfRole::Attack);
        assert_eq!(role_for_rank(6, 1), CtfRole::Defend);
        assert_eq!(role_for_rank(6, 2), CtfRole::Midfield);
        assert_eq!(role_for_rank(6, 5), CtfRole::Attack);
    }
}
