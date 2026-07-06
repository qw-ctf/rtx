// SPDX-License-Identifier: AGPL-3.0-or-later

//! Generic team match coordination (`rtx_mode 1on1|duel|2on2|2on2on2|NonM…`).
//!
//! A *reusable* team layer, not a one-off mode: it manages **N teams of size M** (an arbitrary
//! number of teams, uniform size) selected by an alias — `duel`/`1on1` (2 teams of 1), `2on2`
//! (2 of 2), `2on2on2` (3 of 2), or any `NonMon…`. The first consumer is a **continuous team
//! deathmatch**: teams frag to the fraglimit, friendly fire follows `teamplay`, and the team
//! primitives (roster, team-aware targeting, colours) are here for future team modes to reuse.
//!
//! Match lifecycle (KTX-inspired): **Warmup** (playable; joiners auto-balanced onto the smallest
//! team) → an explicit **`start`** command **reloads the map** (fresh entities) and runs a
//! **countdown** → **Live** (team frags to the limit) → **Ended** (results) → Warmup. The roster
//! locks at start; players who drop and reconnect are **reattached to their team by netname**, and
//! the whole match state survives the start-reload because it lives on the process-lifetime
//! [`GameState`] (guarded in `worldspawn`).

use glam::Vec3;

use super::{centerprint_all, nearest_player_where, players, BotIntent, GameMode};
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

/// The team-match mode descriptor. Stateless — the config/lifecycle live in [`MatchState`].
pub(crate) struct TeamMatch;

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

/// The per-mode variation points of the shared match lifecycle. Team DM and CTF run the identical
/// warmup → countdown → live → ended machine ([`tick_lifecycle`]); they differ only in the go-live
/// slate reset, the win condition, and the results line. Implementing this (plus the thin
/// `GameMode` shims that delegate to the shared helpers below) is all a new match mode needs.
///
/// Not a `GameMode` supertrait: a blanket `impl<T: MatchMode> GameMode for T` would collide with the
/// direct `impl GameMode` on `Ffa`/`Arena`/`Midair` under Rust's coherence rules, so each match mode
/// keeps a small explicit `GameMode` impl that forwards to `tick_lifecycle`/`team_spawn`/etc.
pub(crate) trait MatchMode {
    /// Reset the mode-specific slate when the countdown expires (frags/scores, flags, runes). The
    /// shared machine then arms the timelimit and flips to Live — so this must *not* touch either.
    fn on_go_live(&self, g: &mut GameState);
    /// One Live frame: refresh scores if the mode tallies them here, and report whether the
    /// frag/capture limit is reached. (The shared machine ends the match on the timelimit itself.)
    fn limit_reached(&self, g: &mut GameState) -> bool;
    /// Broadcast the results line as the match ends (the phase transition is handled by the machine).
    fn announce_result(&self, g: &mut GameState);
}

/// The shared match lifecycle, advanced once per server frame. Drives the countdown announce
/// throttle, the go-live handoff (mode slate → timelimit → FIGHT), the per-frame win check, and the
/// results pause → map-rotate/warmup loop. The three per-mode variation points come from `mode`.
pub(crate) fn tick_lifecycle<M: MatchMode>(mode: &M, g: &mut GameState) {
    let now = g.time();
    match g.team_match.phase {
        MatchPhase::Warmup => {} // playable; team assignment happens on spawn (apply_loadout)
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
                mode.on_go_live(g);
                let tl = g.level.timelimit;
                g.team_match.live_until = if tl > 0 { now + tl as f32 } else { 0.0 };
                g.team_match.phase = MatchPhase::Live;
                centerprint_all(g, "FIGHT!");
            }
        }
        MatchPhase::Live => {
            let time_up = g.team_match.live_until > 0.0 && now >= g.team_match.live_until;
            if mode.limit_reached(g) || time_up {
                mode.announce_result(g);
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
                    g.team_match.phase = MatchPhase::Warmup;
                    g.team_match.roster.clear();
                }
            }
        }
    }
}

/// Team spawn selection shared by every match mode: prefer this team's dedicated spawns
/// (`info_player_teamN`), else fall back to the deathmatch spawns.
pub(crate) fn team_spawn(g: &mut GameState, e: EntId) -> EntId {
    let team = g.entities[e].mode_p.team;
    if team >= 1 {
        let spot = g.select_spawn_point_of(&format!("info_player_team{team}"));
        if spot != EntId::WORLD {
            return spot;
        }
    }
    g.select_spawn_point()
}

/// Weapons are hot in every match phase except the pre-fight countdown.
pub(crate) fn match_weapons_hot(g: &GameState) -> bool {
    !matches!(g.team_match.phase, MatchPhase::Countdown { .. })
}

impl GameMode for TeamMatch {
    fn name(&self) -> &'static str {
        // Constant (not the alias) so `select_mode` resolves every `NonM…` to this one descriptor;
        // `refresh_mode` tracks the actual config separately. See `crate::mode::refresh_mode`.
        "team"
    }

    fn tick(&self, g: &mut GameState) {
        tick_lifecycle(self, g);
    }

    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        team_spawn(g, e)
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        // Assign/reattach the team + colours; weapons/ammo stay the decoded DM parms (shotgun + axe,
        // plus the grapple handout) — a standard team-DM start.
        assign_team(g, e);
    }

    fn weapons_hot(&self, g: &GameState) -> bool {
        match_weapons_hot(g)
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // Hunt the nearest living *enemy* (different team). Teammates are skipped, so bots don't
        // waste shots on allies. Falls back to the generic roam when no enemy is in play.
        nearest_enemy(g, bot).map(BotIntent::Fight)
    }

    fn handle_command(&self, g: &mut GameState, _e: EntId, cmd: &str) -> bool {
        match_handle_command(g, cmd)
    }
}

impl MatchMode for TeamMatch {
    fn on_go_live(&self, g: &mut GameState) {
        // Continuous team DM: zero every player's frags and (re)size the team-score tally.
        for e in players(g) {
            g.entities[e].v.frags = 0.0;
        }
        g.team_match.scores = vec![0; g.team_match.config.teams];
    }

    fn limit_reached(&self, g: &mut GameState) -> bool {
        // Team scores are Σ member frags, recomputed every Live frame for the scoreboard/result.
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

    fn announce_result(&self, g: &mut GameState) {
        self.end_match(g);
    }
}

/// Reset (or resume) team-match state on a map (re)load, called from [`super::on_worldspawn`]. A
/// match-start reload (`resuming`) preserves the locked roster and **arms the countdown**; any other
/// load — a fresh map or a switch away from a match mode — starts a fresh warmup (keeping the parsed
/// format). Runs after `refresh_mode` (which owns the alias→config tracking). Shared by team DM and
/// CTF, since both live on the same [`MatchState`].
pub(crate) fn on_worldspawn(g: &mut GameState) {
    if !is_match_mode(g.mode.name()) {
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

/// Begin a match: lock the current roster and reload the map. Dispatched from a mode's
/// [`GameMode::handle_command`] on the `start` command. The countdown is armed in
/// [`on_worldspawn`] after the reload (via `resuming`).
pub(crate) fn start_match(g: &mut GameState) {
    if !is_match_mode(g.mode.name()) || !matches!(g.team_match.phase, MatchPhase::Warmup) {
        return;
    }
    let roster: Vec<(String, u8)> = players(g)
        .into_iter()
        .map(|e| (g.netname_of(e), g.entities[e].mode_p.team))
        .collect();
    if roster.is_empty() {
        return;
    }
    g.team_match.roster = roster;
    g.team_match.resuming = true;
    g.broadcast(crate::defs::PrintLevel::High, "Match starting — reloading map…\n");
    let map = cstring(&g.level.mapname);
    g.host().changelevel(&map);
}

/// A match mode's console-command hook: consume `start`. Shared by team DM and CTF.
pub(crate) fn match_handle_command(g: &mut GameState, cmd: &str) -> bool {
    if cmd == "start" {
        start_match(g);
        true
    } else {
        false
    }
}

impl TeamMatch {
    /// Broadcast the match result (duel scoreline, or per-team tally). The post-match pause is
    /// entered by the shared [`tick_lifecycle`] machine.
    fn end_match(&self, g: &mut GameState) {
        let scores = g.team_match.scores.clone();
        let config = g.team_match.config;
        if config.teams == 2 && config.size == 1 {
            // Duel: name the two duelists on one line.
            let names: Vec<String> = players(g).into_iter().map(|e| g.netname_of(e)).collect();
            let a = names.first().cloned().unwrap_or_default();
            let b = names.get(1).cloned().unwrap_or_default();
            g.broadcast(
                crate::defs::PrintLevel::High,
                &format!(
                    "Duel over — {a} {} : {} {b}\n",
                    scores.first().unwrap_or(&0),
                    scores.get(1).unwrap_or(&0)
                ),
            );
        } else {
            let mut line = String::from("Match over —");
            for (i, s) in scores.iter().enumerate() {
                let (name, _) = TEAM_IDENTITY[i % MAX_TEAMS];
                line.push_str(&format!(" {name} {s}"));
            }
            line.push('\n');
            g.broadcast(crate::defs::PrintLevel::High, &line);
        }
    }
}

/// Whether `name` is one of the match-lifecycle modes (team DM or CTF) that share `MatchState`, the
/// warmup→start→countdown→live→ended machine, the locked roster, and the team-coordination helpers.
pub(crate) fn is_match_mode(name: &str) -> bool {
    name == "team" || name == "ctf"
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
    let candidates: Vec<(EntId, f32, u32)> = crate::mode::players(g)
        .into_iter()
        .filter(|&en| {
            let e = &g.entities[en];
            e.v.health > 0.0 && e.v.deadflag == 0.0 && e.mode_p.team != my_team
        })
        .map(|en| {
            let d = (g.entities[en].v.origin - origin).length_squared();
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
