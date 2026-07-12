// SPDX-License-Identifier: AGPL-3.0-or-later

//! The **match-composition layer** — the second axis, orthogonal to the game mode (`rtx_mode`).
//!
//! `rtx_mode` picks the *ruleset* (dm / ra / midair / ctf); `rtx_match` picks how the match is
//! *organized*, resolved by [`resolve_composition`] into a [`MatchConfig`] of **N teams of size M**:
//!
//! - **Open** (`{teams: 0}`) — no teams, no lifecycle: plain free-for-all (dm/midair default).
//! - **Open team pickup** (`{teams: 2, size: 0}`) — teams + lifecycle, unbounded roster,
//!   auto-balanced joiners (CTF's default; public team play).
//! - **Structured** (`{teams: N, size: M ≥ 1}`) — a locked N×M match: `1on1`/`duel`, `2on2`,
//!   `2on2on2`, any `NonMon…`. Overflow / late joiners are **benched** (see [`benched`]).
//!
//! Any composition with `teams ≥ 2` ([`lifecycle_active`]) drives the KTX-inspired match lifecycle,
//! shared by team-DM (the trait defaults) and CTF alike: **Warmup** (playable; joiners
//! auto-balanced) → an explicit **`start`** command (strict roster: [`pick_roster`]) **reloads the
//! map** and runs a **countdown** → **Live** (frag / capture limit) → **Ended** (results) → Warmup.
//! The roster locks at start; players who drop and reconnect are **reattached by netname**, and the
//! whole match state survives the start-reload because it lives on the process-lifetime
//! [`GameState`] (guarded in `worldspawn`). The mode supplies only the go-live slate, win condition,
//! and result line via the three `on_match_*` [`GameMode`](super::GameMode) hooks.

use glam::Vec3;

use super::{centerprint_all, nearest_player_where, players};
use crate::defs::{DeadFlag, PrintLevel};
use crate::entity::EntId;
use crate::game::{cstring, GameState};

/// Upper bound on teams (sizes the colour/spawn tables; more teams cycle the palette and fall back
/// to DM spawns). Covers every practical `NonMon…` format.
const MAX_TEAMS: usize = 8;

/// Team identity: `(name, QW color index)`. The name goes into each member's `"team"` userinfo so
/// the existing `teamplay_protects` friendly-fire and the engine scoreboard group by team; the
/// colour goes into `top/bottomcolor` so teammates share a shirt colour. Cycled for teams > 8.
const TEAM_IDENTITY: [(&str, &str); MAX_TEAMS] = [
    ("red", "4"),
    ("blue", "13"),
    ("green", "2"),
    ("yellow", "12"),
    ("pink", "11"),
    ("orange", "6"),
    ("purple", "9"),
    ("white", "0"),
];

/// Seconds the results screen lingers before dropping back to warmup.
const END_PAUSE: f32 = 5.0;

/// A team format: `teams` sides of `size` players each (`2on2on2` = 3 teams of 2).
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub(crate) struct MatchConfig {
    pub teams: usize,
    pub size: usize,
}

/// Where the match is in its lifecycle.
#[derive(Clone, Copy, Default)]
pub(crate) enum MatchPhase {
    /// Playable team DM; joiners auto-balanced. Waiting for a `start`.
    #[default]
    Warmup,
    /// Post-reload spawn protection, counting down to "FIGHT". `until` is the world time it ends.
    Countdown { until: f32 },
    /// Team frags count toward the limit.
    Live,
    /// A results pause before returning to warmup. `until` is when it ends.
    Ended { until: f32 },
}

/// Mutable team-match state, owned by [`GameState`]. Lives on the process-lifetime game state so it
/// **survives the match-start map reload** (guarded against `worldspawn`'s reset). Meaningful only
/// while a team `rtx_mode` alias is active.
#[derive(Default)]
pub(crate) struct MatchState {
    pub config: MatchConfig,
    pub phase: MatchPhase,
    /// Last countdown second announced (per-second centerprint throttle, as in Arena).
    pub last_count: i32,
    /// Team scores this match (index `team-1`), recomputed each Live frame as Σ member frags.
    pub scores: Vec<i32>,
    /// The roster locked at match start: `(netname, team)`. Used to reattach a reconnecting player
    /// (and to restore teams after the start-reload clears per-entity state).
    pub roster: Vec<(String, u8)>,
    /// Set by the `start` command right before the reload; consumed once in `worldspawn` to arm the
    /// countdown. Distinguishes a match-start reload (preserve state) from any other map change
    /// (fresh warmup).
    pub resuming: bool,
    /// World time the match ends on `timelimit` (`0` = no time limit). Set when the round goes Live.
    pub live_until: f32,
}

/// Parse an `rtx_mode` alias into a team format, or `None` if it isn't one. `duel`/`1on1` → 2×1;
/// otherwise a `NonMon…` chain of equal sizes (`2on2on2` → 3×2). Ragged sizes and zero are rejected.
pub(crate) fn parse_match_alias(s: &str) -> Option<MatchConfig> {
    if s == "duel" || s == "1on1" {
        return Some(MatchConfig { teams: 2, size: 1 });
    }
    let parts: Vec<&str> = s.split("on").collect();
    if parts.len() < 2 {
        return None;
    }
    let sizes: Vec<usize> = parts.iter().map(|p| p.parse().ok()).collect::<Option<_>>()?;
    if sizes.contains(&0) || sizes.iter().any(|&n| n != sizes[0]) {
        return None;
    }
    Some(MatchConfig {
        teams: sizes.len(),
        size: sizes[0],
    })
}

/// Resolve the two cvars (`rtx_mode` ruleset + `rtx_match` alias) into a composition. This is the one
/// place the two axes meet:
///
/// - **ra** ignores `rtx_match` entirely — its 1v1 round queue *is* its composition (always Open).
/// - `""` (auto) picks the mode's natural composition: CTF → open 2-team pickup, midair → a 1on1
///   duel, everything else → Open free-for-all.
/// - `ffa` forces open play (CTF stays 2-team so its flags have owners; everyone else Open).
/// - a parsed `NonMon…` is taken as-is, except CTF clamps it to 2 teams (flags are red/blue).
/// - anything unparseable falls back to the mode's auto default (the caller hints once).
pub(crate) fn resolve_composition(mode: &str, alias: &str) -> MatchConfig {
    const OPEN: MatchConfig = MatchConfig { teams: 0, size: 0 };
    const CTF_PICKUP: MatchConfig = MatchConfig { teams: 2, size: 0 };
    // ra's 1v1 round queue is its composition; race is per-runner timed play — neither has a
    // team lifecycle, ever.
    if mode == "ra" || mode == "race" {
        return OPEN;
    }
    match alias {
        "" => match mode {
            "ctf" => CTF_PICKUP,
            "midair" => MatchConfig { teams: 2, size: 1 },
            _ => OPEN,
        },
        "ffa" => {
            if mode == "ctf" {
                CTF_PICKUP
            } else {
                OPEN
            }
        }
        _ => match parse_match_alias(alias) {
            Some(cfg) if mode == "ctf" => MatchConfig { teams: 2, size: cfg.size },
            Some(cfg) => cfg,
            None => resolve_composition(mode, ""),
        },
    }
}

/// Whether the resolved composition runs the team match lifecycle (teams + warmup→live machine).
/// True for structured matches *and* open team pickup (CTF); false for plain free-for-all.
pub(crate) fn lifecycle_active(g: &GameState) -> bool {
    g.team_match.config.teams >= 2
}

/// Whether the composition is a **structured** N×M match — a locked roster with a fixed seat count,
/// as opposed to open team pickup (unbounded roster). Only structured matches bench overflow players.
pub(crate) fn structured(g: &GameState) -> bool {
    let c = g.team_match.config;
    c.teams >= 2 && c.size >= 1
}

/// A short human-readable label for a composition (`duel`, `2on2`, `2on2on2`, or `open`), for the
/// console format line and the `start`-refused message.
pub(crate) fn format_label(cfg: MatchConfig) -> String {
    if cfg.size == 0 {
        "open".to_string()
    } else if cfg.teams == 2 && cfg.size == 1 {
        "duel".to_string()
    } else {
        vec![cfg.size.to_string(); cfg.teams].join("on")
    }
}

/// Whether player `e` is currently **benched** — a structured match is under way (past warmup) and
/// they aren't on the locked roster (a late joiner, or an overflow beyond teams×size). Benched
/// players sit out as harmless spectators until the next warmup. Derived from the roster (not a
/// stored flag), so it survives the match-start reload for free. Cheap in open/warmup play (the
/// `structured` gate short-circuits before the netname lookup allocates).
pub(crate) fn benched(g: &GameState, e: EntId) -> bool {
    structured(g)
        && !matches!(g.team_match.phase, MatchPhase::Warmup)
        && {
            let name = g.netname_of(e);
            !g.team_match.roster.iter().any(|(rn, _)| *rn == name)
        }
}

/// Seat the warmup membership into exactly `teams × size` slots for a structured `start`. Humans are
/// seated before bots; a player keeps their current team while it has a free seat, otherwise they're
/// moved to the lowest-numbered team with room (there's no join-team command, so rebalancing here —
/// rather than refusing an imbalanced warmup — is what lets a match start after disconnects skew the
/// sides). `Err(n)` means the warmup is `n` players short of a full format. Pure and unit-tested.
pub(crate) fn pick_roster(members: &[(EntId, u8, bool)], cfg: MatchConfig) -> Result<Vec<(EntId, u8)>, usize> {
    let seats = cfg.teams * cfg.size;
    // Humans first (index by is_bot == false), then bots, each in input order.
    let ordered: Vec<&(EntId, u8, bool)> = members
        .iter()
        .filter(|m| !m.2)
        .chain(members.iter().filter(|m| m.2))
        .collect();
    if ordered.len() < seats {
        return Err(seats - ordered.len());
    }
    let mut counts = vec![0usize; cfg.teams];
    let mut out: Vec<(EntId, u8)> = Vec::with_capacity(seats);
    for &&(e, cur, _) in &ordered {
        if out.len() == seats {
            break;
        }
        let keep = cur >= 1 && (cur as usize) <= cfg.teams && counts[cur as usize - 1] < cfg.size;
        let team = if keep {
            cur as usize - 1
        } else {
            match counts.iter().position(|&c| c < cfg.size) {
                Some(i) => i,
                None => break, // unreachable: total capacity == seats, and out.len() < seats here
            }
        };
        counts[team] += 1;
        out.push((e, (team + 1) as u8));
    }
    Ok(out)
}

/// The shared match lifecycle, advanced once per server frame from `start_frame` (a no-op unless
/// [`lifecycle_active`]). Drives the countdown announce throttle, the go-live handoff (mode slate →
/// timelimit → FIGHT), the per-frame win check, and the results pause → map-rotate/warmup loop. The
/// three per-mode variation points are [`GameMode`](super::GameMode) hooks on the active mode
/// (`on_match_go_live` / `match_limit_reached` / `announce_match_result`), whose defaults reproduce
/// team deathmatch (see the free fns below); CTF overrides all three.
pub(crate) fn tick_lifecycle(g: &mut GameState) {
    if !lifecycle_active(g) {
        return;
    }
    let now = g.time();
    match g.team_match.phase {
        MatchPhase::Warmup => {} // playable; team assignment happens on spawn (put_client_in_server)
        MatchPhase::Countdown { until } => {
            let remaining = (until - now).ceil() as i32;
            if remaining != g.team_match.last_count {
                g.team_match.last_count = remaining;
                if remaining > 0 {
                    centerprint_all(g, &format!("{remaining}"));
                }
            }
            if now >= until {
                // Go live: the mode's own slate reset, then arm the time limit (`timelimit` is in
                // seconds; 0 = none) and flip to Live.
                let mode = g.mode;
                mode.on_match_go_live(g);
                let tl = g.level.timelimit;
                g.team_match.live_until = if tl > 0 { now + tl as f32 } else { 0.0 };
                g.team_match.phase = MatchPhase::Live;
                centerprint_all(g, "FIGHT!");
            }
        }
        MatchPhase::Live => {
            let time_up = g.team_match.live_until > 0.0 && now >= g.team_match.live_until;
            let mode = g.mode;
            if mode.match_limit_reached(g) || time_up {
                mode.announce_match_result(g);
                g.team_match.phase = MatchPhase::Ended { until: now + END_PAUSE };
            }
        }
        MatchPhase::Ended { until } => {
            if now >= until {
                // Rotate to the next map in the queue if one is configured; otherwise loop back to
                // warmup on the same map for another `start`.
                if g.queued_next_map().is_some() {
                    g.next_level();
                } else {
                    // Re-admit any benched spectators: snapshot them *before* clearing the roster
                    // (bench is derived from it), drop to warmup, then respawn each so they rejoin
                    // with a real loadout/team.
                    let benched_players: Vec<EntId> = if structured(g) {
                        players(g).into_iter().filter(|&e| benched(g, e)).collect()
                    } else {
                        Vec::new()
                    };
                    g.team_match.phase = MatchPhase::Warmup;
                    g.team_match.roster.clear();
                    for e in benched_players {
                        g.put_client_in_server(e);
                    }
                }
            }
        }
    }
}

/// Default `on_match_go_live` (continuous team DM): zero every player's frags and (re)size the
/// per-team score tally. The shared machine then arms the timelimit and flips to Live.
pub(crate) fn default_go_live(g: &mut GameState) {
    for e in players(g) {
        g.entities[e].v.frags = 0.0;
    }
    g.team_match.scores = vec![0; g.team_match.config.teams];
}

/// Default `match_limit_reached` (team DM): team scores are Σ member frags, recomputed every Live
/// frame for the scoreboard/result; the match ends when any team reaches `fraglimit`.
pub(crate) fn frag_limit_reached(g: &mut GameState) -> bool {
    let teams = g.team_match.config.teams;
    let mut scores = vec![0i32; teams];
    for e in players(g) {
        let t = g.entities[e].mode_p.team as usize;
        if t >= 1 && t <= teams {
            scores[t - 1] += g.entities[e].v.frags as i32;
        }
    }
    g.team_match.scores = scores.clone();
    let fl = g.level.fraglimit;
    fl != 0 && scores.iter().any(|&s| s >= fl)
}

/// Default `announce_match_result` (team DM): a duel scoreline, or a per-team tally. The post-match
/// pause is entered by the shared [`tick_lifecycle`] machine.
pub(crate) fn announce_team_result(g: &mut GameState) {
    let scores = g.team_match.scores.clone();
    let config = g.team_match.config;
    if config.teams == 2 && config.size == 1 {
        // Duel: name each duelist from the *locked roster* (not `players()` order, which now
        // includes any benched spectators), paired to their team's score.
        let name_of = |t: u8| {
            g.team_match
                .roster
                .iter()
                .find(|(_, tt)| *tt == t)
                .map(|(n, _)| n.clone())
                .unwrap_or_default()
        };
        g.broadcast(
            PrintLevel::High,
            &format!(
                "Duel over — {} {} : {} {}\n",
                name_of(1),
                scores.first().unwrap_or(&0),
                scores.get(1).unwrap_or(&0),
                name_of(2)
            ),
        );
    } else {
        let mut line = String::from("Match over —");
        for (i, s) in scores.iter().enumerate() {
            let (name, _) = TEAM_IDENTITY[i % MAX_TEAMS];
            line.push_str(&format!(" {name} {s}"));
        }
        line.push('\n');
        g.broadcast(PrintLevel::High, &line);
    }
}

/// Team spawn selection shared by every match mode: prefer this team's dedicated spawns
/// (`info_player_teamN`), else fall back to the deathmatch spawns.
pub(crate) fn team_spawn(g: &mut GameState, e: EntId) -> EntId {
    let team = g.entities[e].mode_p.team;
    if team >= 1 {
        let spot = g.select_spawn_point_of(&format!("info_player_team{team}"), Some(e));
        if spot != EntId::WORLD {
            return spot;
        }
    }
    g.select_spawn_point(Some(e))
}

/// Weapons are hot in every match phase except the pre-fight countdown.
pub(crate) fn match_weapons_hot(g: &GameState) -> bool {
    !matches!(g.team_match.phase, MatchPhase::Countdown { .. })
}

/// Reset (or resume) team-match state on a map (re)load, called from [`super::on_worldspawn`]. A
/// match-start reload (`resuming`) preserves the locked roster and **arms the countdown**; any other
/// load — a fresh map, or a switch to a composition with no team lifecycle — starts a fresh warmup
/// (keeping the resolved format). Runs after `refresh_mode` (which owns the alias→config tracking).
pub(crate) fn on_worldspawn(g: &mut GameState) {
    if !lifecycle_active(g) {
        g.team_match = MatchState::default();
        return;
    }
    if g.team_match.resuming {
        g.team_match.resuming = false;
        let cd = g.host().cvar(c"rtx_match_countdown").max(0.0);
        g.team_match.phase = MatchPhase::Countdown { until: g.time() + cd };
        g.team_match.last_count = -1;
    } else {
        let cfg = g.team_match.config;
        g.team_match = MatchState {
            config: cfg,
            ..Default::default()
        };
    }
}

/// Begin a match: lock the roster and reload the map. Dispatched from `client_command` on the `start`
/// console command; a no-op unless the lifecycle is active and we're in warmup. A **structured**
/// match seats exactly teams×size via [`pick_roster`] (refusing, with a message, if the warmup is
/// short) and re-teams any moved players; an **open** team pickup (CTF) locks everyone currently in.
/// The countdown is armed in [`on_worldspawn`] after the reload (via `resuming`).
pub(crate) fn start_match(g: &mut GameState) {
    if !lifecycle_active(g) || !matches!(g.team_match.phase, MatchPhase::Warmup) {
        return;
    }
    let cfg = g.team_match.config;
    let roster: Vec<(String, u8)> = if structured(g) {
        let members: Vec<(EntId, u8, bool)> = players(g)
            .into_iter()
            .map(|e| (e, g.entities[e].mode_p.team, g.entities[e].bot.is_bot))
            .collect();
        match pick_roster(&members, cfg) {
            Ok(seated) => {
                // Re-team any player the seating moved, then lock the roster by netname.
                for &(e, team) in &seated {
                    if g.entities[e].mode_p.team != team {
                        g.entities[e].mode_p.team = team;
                        apply_team_identity(g, e);
                    }
                }
                seated.iter().map(|&(e, team)| (g.netname_of(e), team)).collect()
            }
            Err(short) => {
                g.broadcast(
                    PrintLevel::High,
                    &format!("start: {} needs {short} more player(s)\n", format_label(cfg)),
                );
                return;
            }
        }
    } else {
        let r: Vec<(String, u8)> = players(g)
            .into_iter()
            .map(|e| (g.netname_of(e), g.entities[e].mode_p.team))
            .collect();
        if r.is_empty() {
            return;
        }
        r
    };
    g.team_match.roster = roster;
    g.team_match.resuming = true;
    g.broadcast(PrintLevel::High, "Match starting — reloading map…\n");
    let map = cstring(&g.level.mapname);
    g.host().changelevel(&map);
}

/// Assign player `e` a team the first time they're placed (team `0`): reattach a reconnecting player
/// from the locked roster by netname, else auto-balance onto the smallest team; then apply the team
/// colours/userinfo. Shared by team DM and CTF (called from their `apply_loadout`).
pub(crate) fn assign_team(g: &mut GameState, e: EntId) {
    if g.entities[e].mode_p.team == 0 {
        let name = g.netname_of(e);
        let team = g
            .team_match
            .roster
            .iter()
            .find(|(n, _)| *n == name)
            .map(|&(_, t)| t)
            .filter(|&t| t >= 1)
            .unwrap_or_else(|| smallest_team(g));
        g.entities[e].mode_p.team = team;
    }
    apply_team_identity(g, e);
}

/// The `(name, color)` identity for team `t` (1-based), cycling the palette past [`MAX_TEAMS`].
pub(crate) fn team_identity(t: u8) -> (&'static str, &'static str) {
    TEAM_IDENTITY[(t as usize).saturating_sub(1) % MAX_TEAMS]
}

/// Write player `e`'s team name + shirt/pant colour into userinfo, so friendly fire
/// (`teamplay_protects`) and the engine scoreboard follow the team, and teammates share a colour.
pub(crate) fn apply_team_identity(g: &mut GameState, e: EntId) {
    let (name, color) = team_identity(g.entities[e].mode_p.team);
    let is_bot = g.entities[e].bot.is_bot;
    let host = *g.host();
    let client = e.0 as i32;
    let name_c = cstring(name);
    let color_c = cstring(color);
    let set = |key: &core::ffi::CStr, val: &core::ffi::CStr| {
        if is_bot {
            host.set_bot_userinfo(client, key, val, 0);
        } else {
            host.set_userinfo(client, key, val, 0);
        }
    };
    set(c"team", &name_c);
    set(c"topcolor", &color_c);
    set(c"bottomcolor", &color_c);
}

/// The team id (1-based) with the fewest current members — ties break to the lowest id.
pub(crate) fn smallest_team(g: &GameState) -> u8 {
    let teams = g.team_match.config.teams.max(1);
    let mut counts = vec![0u32; teams];
    for e in players(g) {
        let t = g.entities[e].mode_p.team as usize;
        if t >= 1 && t <= teams {
            counts[t - 1] += 1;
        }
    }
    let idx = counts
        .iter()
        .enumerate()
        .min_by_key(|(_, &c)| c)
        .map(|(i, _)| i)
        .unwrap_or(0);
    (idx + 1) as u8
}

/// Beyond this many teammate bots already committed to an enemy, others spread to a fresher target
/// instead of piling on — so a team splits its attention across the opposing side.
const MAX_ATTACKERS: u32 = 2;

/// The enemy `bot` should engage. With `rtx_bot_teamwork` (the default) it deconflicts: prefer the
/// nearest enemy that fewer than [`MAX_ATTACKERS`] teammates are already on, so bots don't dogpile
/// whoever's closest; if every enemy is saturated it falls back to nearest, so no bot idles. With
/// teamwork off it's plain nearest-enemy — the team-aware picker the FFA/Arena/Midair ones lack.
pub(crate) fn nearest_enemy(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    let my_team = g.entities[bot].mode_p.team;
    if !g.host().cvar_bool(c"rtx_bot_teamwork") {
        return nearest_enemy_to(g, my_team, origin);
    }
    // Opponent modeling nudges the choice toward weaker / better-armed enemies (a finishable frag,
    // and in deathmatch 1 a kill that resets an armed player's kit). The bias is a distance²
    // multiplier that returns 1.0 when modeling is off, so this stays plain nearest then.
    let now = g.time();
    let weapons_stay = matches!(g.level.deathmatch, 2 | 3 | 5);
    let candidates: Vec<(EntId, f32, u32)> = crate::mode::players(g)
        .into_iter()
        .filter(|&en| {
            let e = &g.entities[en];
            e.v.health > 0.0 && e.v.deadflag == DeadFlag::No && e.mode_p.team != my_team
        })
        .map(|en| {
            let d = (g.entities[en].v.origin - origin).length_squared()
                * g.target_dist_bias(bot, en, now, weapons_stay);
            (en, d, teammate_attackers(g, bot, my_team, en))
        })
        .collect();
    assign_target(&candidates)
}

/// How many *other* living teammate bots are currently committed to `enemy` (their perception's
/// `known_enemy`). The deconfliction signal — who the team is already fighting.
fn teammate_attackers(g: &GameState, bot: EntId, my_team: u8, enemy: EntId) -> u32 {
    crate::mode::players(g)
        .into_iter()
        .filter(|&t| {
            let e = &g.entities[t];
            t != bot && e.bot.is_bot && e.v.health > 0.0 && e.mode_p.team == my_team && e.bot.known_enemy == enemy.0
        })
        .count() as u32
}

/// Pick from `(enemy, dist², attacker_count)`: the nearest enemy under the [`MAX_ATTACKERS`] cap,
/// else (all saturated) the nearest overall — never `None` when any candidate exists. Pure.
fn assign_target(candidates: &[(EntId, f32, u32)]) -> Option<EntId> {
    let nearest = |set: &mut dyn Iterator<Item = &(EntId, f32, u32)>| set.min_by(|a, b| a.1.total_cmp(&b.1)).map(|&(e, _, _)| e);
    nearest(&mut candidates.iter().filter(|&&(_, _, atk)| atk < MAX_ATTACKERS)).or_else(|| nearest(&mut candidates.iter()))
}

/// The nearest living player not on `my_team` to an arbitrary `point` — used to pick a target near a
/// base to defend, not just near the bot itself.
pub(crate) fn nearest_enemy_to(g: &GameState, my_team: u8, point: Vec3) -> Option<EntId> {
    nearest_player_where(g, point, EntId::WORLD, |g, e| g.entities[e].mode_p.team != my_team)
}

#[cfg(test)]
mod tests {
    use super::{parse_match_alias, MatchConfig};

    fn cfg(teams: usize, size: usize) -> Option<MatchConfig> {
        Some(MatchConfig { teams, size })
    }

    #[test]
    fn resolves_composition_per_mode() {
        use super::resolve_composition as r;
        let open = MatchConfig { teams: 0, size: 0 };
        let ctf_pickup = MatchConfig { teams: 2, size: 0 };
        // Auto ("") picks each mode's natural composition.
        assert_eq!(r("dm", ""), open);
        assert_eq!(r("ctf", ""), ctf_pickup);
        assert_eq!(r("midair", ""), MatchConfig { teams: 2, size: 1 });
        // ra ignores rtx_match entirely.
        assert_eq!(r("ra", ""), open);
        assert_eq!(r("ra", "2on2"), open);
        assert_eq!(r("ra", "ffa"), open);
        // "ffa" forces open play; CTF stays 2-team so its flags have owners.
        assert_eq!(r("dm", "ffa"), open);
        assert_eq!(r("ctf", "ffa"), ctf_pickup);
        // Parsed formats pass through; CTF clamps to 2 teams keeping the size.
        assert_eq!(r("dm", "2on2"), MatchConfig { teams: 2, size: 2 });
        assert_eq!(r("midair", "2on2"), MatchConfig { teams: 2, size: 2 });
        assert_eq!(r("ctf", "2on2on2"), MatchConfig { teams: 2, size: 2 });
        assert_eq!(r("dm", "duel"), MatchConfig { teams: 2, size: 1 });
        // Unparseable falls back to the mode's auto default.
        assert_eq!(r("dm", "garbage"), open);
        assert_eq!(r("ctf", "garbage"), ctf_pickup);
    }

    #[test]
    fn pick_roster_seats_humans_first_and_rebalances() {
        use super::pick_roster;
        use crate::entity::EntId;
        let two_by_two = MatchConfig { teams: 2, size: 2 };
        // Humans (false) seated before bots (true), regardless of input order.
        let members = [
            (EntId(1), 0u8, true),  // bot
            (EntId(2), 0u8, false), // human
            (EntId(3), 0u8, true),  // bot
            (EntId(4), 0u8, false), // human
        ];
        let seated = pick_roster(&members, two_by_two).unwrap();
        assert_eq!(seated.len(), 4);
        // Humans are seated before bots (so a human is never benched while a bot plays); unassigned
        // players then fill team 1 before team 2, so the two humans (2, 4) land ahead of the bots.
        let team_of = |e: EntId| seated.iter().find(|&&(x, _)| x == e).unwrap().1;
        assert!(seated.iter().take(2).all(|&(e, _)| e == EntId(2) || e == EntId(4)), "humans seated first");
        assert_eq!(team_of(EntId(2)), 1);
        assert_eq!(team_of(EntId(4)), 1);
        assert_eq!(team_of(EntId(1)), 2);
        assert_eq!(team_of(EntId(3)), 2);
        // A player keeps their current team while it has room, else moves to a team with a free seat.
        let skewed = [
            (EntId(1), 1, false),
            (EntId(2), 1, false),
            (EntId(3), 1, false), // team 1 is full after two → this one rebalances to team 2
            (EntId(4), 2, false),
        ];
        let seated = pick_roster(&skewed, two_by_two).unwrap();
        let team_of = |e: EntId| seated.iter().find(|&&(x, _)| x == e).unwrap().1;
        assert_eq!(team_of(EntId(1)), 1);
        assert_eq!(team_of(EntId(2)), 1);
        assert_eq!(team_of(EntId(3)), 2, "overflow of team 1 rebalances to team 2");
        assert_eq!(team_of(EntId(4)), 2);
        // Short warmup → Err(n) with how many more are needed.
        let short = [(EntId(1), 0, false), (EntId(2), 0, false)];
        assert_eq!(pick_roster(&short, two_by_two), Err(2));
    }

    #[test]
    fn parses_team_aliases() {
        assert_eq!(parse_match_alias("duel"), cfg(2, 1));
        assert_eq!(parse_match_alias("1on1"), cfg(2, 1));
        assert_eq!(parse_match_alias("2on2"), cfg(2, 2));
        assert_eq!(parse_match_alias("4on4"), cfg(2, 4));
        assert_eq!(parse_match_alias("2on2on2"), cfg(3, 2));
        assert_eq!(parse_match_alias("3on3on3on3"), cfg(4, 3));
    }

    #[test]
    fn rejects_non_team_aliases() {
        for s in ["ffa", "ra", "midair", "2", "", "onon", "2on3", "0on0", "2on0"] {
            assert_eq!(parse_match_alias(s), None, "{s} should not parse");
        }
    }

    #[test]
    fn assign_target_deconflicts_then_falls_back() {
        use super::{assign_target, MAX_ATTACKERS};
        use crate::entity::EntId;
        assert_eq!(MAX_ATTACKERS, 2); // the cases below assume this cap
        assert_eq!(assign_target(&[]), None, "no candidates → no target");
        // Nearest (A) is saturated → pick the nearest *unsaturated* (C at 150 beats B at 200).
        let mixed = [(EntId(3), 100.0, 2), (EntId(4), 200.0, 0), (EntId(5), 150.0, 0)];
        assert_eq!(assign_target(&mixed), Some(EntId(5)));
        // Every enemy saturated → fall back to nearest overall, never idle.
        let saturated = [(EntId(3), 300.0, 2), (EntId(4), 100.0, 3)];
        assert_eq!(assign_target(&saturated), Some(EntId(4)));
        // One enemy, unsaturated → it, trivially.
        assert_eq!(assign_target(&[(EntId(9), 500.0, 1)]), Some(EntId(9)));
    }
}
