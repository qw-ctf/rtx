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

use super::{BotIntent, GameMode};
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

impl GameMode for TeamMatch {
    fn name(&self) -> &'static str {
        // Constant (not the alias) so `select_mode` resolves every `NonM…` to this one descriptor;
        // `refresh_mode` tracks the actual config separately. See `crate::mode::refresh_mode`.
        "team"
    }

    fn tick(&self, g: &mut GameState) {
        let now = g.time();
        match g.team_match.phase {
            MatchPhase::Warmup => {} // playable; assignment happens on spawn (apply_loadout)
            MatchPhase::Countdown { until } => {
                let remaining = (until - now).ceil() as i32;
                if remaining != g.team_match.last_count {
                    g.team_match.last_count = remaining;
                    if remaining > 0 {
                        centerprint_all(g, &format!("{remaining}"));
                    }
                }
                if now >= until {
                    // Go live on a clean slate: zero every player's frags and the team scores, and
                    // arm the time limit (`timelimit` is in seconds; 0 = none).
                    for e in players(g) {
                        g.entities[e].v.frags = 0.0;
                    }
                    g.team_match.scores = vec![0; g.team_match.config.teams];
                    let tl = g.level.timelimit;
                    g.team_match.live_until = if tl > 0 { now + tl as f32 } else { 0.0 };
                    g.team_match.phase = MatchPhase::Live;
                    centerprint_all(g, "FIGHT!");
                }
            }
            MatchPhase::Live => {
                let teams = g.team_match.config.teams;
                let mut scores = vec![0i32; teams];
                for e in players(g) {
                    let t = g.entities[e].arena.team as usize;
                    if t >= 1 && t <= teams {
                        scores[t - 1] += g.entities[e].v.frags as i32;
                    }
                }
                g.team_match.scores = scores.clone();
                let fl = g.level.fraglimit;
                let time_up = g.team_match.live_until > 0.0 && now >= g.team_match.live_until;
                if (fl != 0 && scores.iter().any(|&s| s >= fl)) || time_up {
                    self.end_match(g, now);
                }
            }
            MatchPhase::Ended { until } => {
                if now >= until {
                    // Rotate to the next map in the queue if one is configured; otherwise loop back
                    // to warmup on the same map for another `start`.
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
        // Prefer this team's dedicated spawns (`info_player_teamN`), else the deathmatch spawns.
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
        // Assign/reattach the team + colours; weapons/ammo stay the decoded DM parms (shotgun + axe,
        // plus the grapple handout) — a standard team-DM start.
        assign_team(g, e);
    }

    fn weapons_hot(&self, g: &GameState) -> bool {
        // Everything is hot except the pre-fight countdown.
        !matches!(g.team_match.phase, MatchPhase::Countdown { .. })
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // Hunt the nearest living *enemy* (different team). Teammates are skipped, so bots don't
        // waste shots on allies. Falls back to the generic roam when no enemy is in play.
        nearest_enemy(g, bot).map(BotIntent::Fight)
    }
}

impl TeamMatch {
    /// Reset (or resume) team-match state on a map (re)load, called from `worldspawn`. A match-start
    /// reload (`resuming`) preserves the locked roster and **arms the countdown**; any other load —
    /// a fresh map or a switch away from team mode — starts a fresh warmup (keeping the parsed
    /// format). Runs after `refresh_mode` (which owns the alias→config tracking).
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

    /// Begin a match: lock the current roster and reload the map. Called from the `start` command.
    /// The countdown is armed in `on_worldspawn` after the reload (via `resuming`).
    pub(crate) fn start(g: &mut GameState) {
        if !is_match_mode(g.mode.name()) || !matches!(g.team_match.phase, MatchPhase::Warmup) {
            return;
        }
        let roster: Vec<(String, u8)> = players(g)
            .into_iter()
            .map(|e| (g.netname_of(e), g.entities[e].arena.team))
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

    /// Announce the result and enter the post-match pause.
    fn end_match(&self, g: &mut GameState, now: f32) {
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
        g.team_match.phase = MatchPhase::Ended { until: now + END_PAUSE };
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
    if g.entities[e].arena.team == 0 {
        let name = g.netname_of(e);
        let team = g
            .team_match
            .roster
            .iter()
            .find(|(n, _)| *n == name)
            .map(|&(_, t)| t)
            .filter(|&t| t >= 1)
            .unwrap_or_else(|| smallest_team(g));
        g.entities[e].arena.team = team;
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
    let (name, color) = team_identity(g.entities[e].arena.team);
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
        let t = g.entities[e].arena.team as usize;
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

/// Every connected player edict (humans and bots, `1..=maxclients`).
pub(crate) fn players(g: &GameState) -> Vec<EntId> {
    let maxclients = g.host().cvar(c"maxclients") as i32;
    (1..=maxclients as u32)
        .map(EntId)
        .filter(|&e| g.entities[e].in_use && g.entities[e].classname() == Some("player"))
        .collect()
}

/// The nearest living player on a *different* team to `bot` — the team-aware enemy picker (the
/// teammate filter the FFA/Arena/Midair pickers deliberately lack).
pub(crate) fn nearest_enemy(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    let my_team = g.entities[bot].arena.team;
    nearest_enemy_to(g, my_team, origin)
}

/// The nearest living player not on `my_team` to an arbitrary `point` — used to pick a target near a
/// base to defend, not just near the bot itself.
pub(crate) fn nearest_enemy_to(g: &GameState, my_team: u8, point: Vec3) -> Option<EntId> {
    let mut best: Option<(EntId, f32)> = None;
    for e in players(g) {
        let ent = &g.entities[e];
        if ent.arena.team == my_team {
            continue;
        }
        if ent.v.health <= 0.0 || ent.v.deadflag != 0.0 {
            continue;
        }
        let d = (ent.v.origin - point).length_squared();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

/// Center-print to every connected human (bots are fake clients with no connection — a unicast to
/// one makes the engine warn "msg_entity: not a client").
pub(crate) fn centerprint_all(g: &GameState, msg: &str) {
    let host = *g.host();
    let cmsg = cstring(msg);
    for e in players(g) {
        if g.entities[e].bot.is_bot {
            continue;
        }
        host.centerprint(e, &cmsg);
    }
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
}
