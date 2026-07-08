// SPDX-License-Identifier: AGPL-3.0-or-later

//! Rocket Arena (`rtx_mode ra`) — a round-based duel mode on the [`Dm`](super::Dm) baseline. Its 1v1
//! round queue *is* its composition, so it ignores `rtx_match` (see [`resolve_composition`]).
//!
//! [`resolve_composition`]: super::team::resolve_composition
//!
//! This follows the "rocket arena part" of the reference Frogbot-Rocket-Arena QuakeC
//! (`fbra/src/arena.qc`): round-based play where everyone spawns with a **full loadout**,
//! fighters duel **inside the arena** while eliminated/waiting players roam the **audience**,
//! damage is off during a short spawn-protected countdown, and the last player standing wins the
//! round. We deliberately drop FBRA's clan-arena machinery (color/shirt teams, team menus,
//! series-to-N-wins) — structured team play is the separate `rtx_match` composition axis.
//!
//! The arena/audience split reuses FBRA's spawn-point trick (`arena.qc:set_a_spawn`): fighters
//! spawn at `info_teleport_destination` (inside the arena on a Rocket-Arena map), audience at
//! `info_player_deathmatch` (the stands). On a plain deathmatch map without teleport
//! destinations it falls back to DM spawns so `ra` still runs.

use glam::Vec3;

use super::{centerprint_all, nearest_player_where, players, ArenaRole, BotIntent, DamageOutcome, GameMode};
use crate::defs::{Items, PrintLevel, Weapon, VEC_VIEW_OFS};
use crate::entity::EntId;
use crate::game::GameState;

/// Where the round is in its lifecycle. The whole mode is this four-state machine, advanced once
/// per server frame by [`Arena::tick`] and on each kill by [`Arena::on_death`].
#[derive(Default, Clone, Copy)]
pub(crate) enum RoundState {
    /// Waiting for enough players to start a round.
    #[default]
    Warmup,
    /// Fighters spawned and protected; counting down to "FIGHT". `until` is the world time the
    /// countdown ends.
    Countdown { until: f32 },
    /// Combat live — damage enabled, eliminations tracked.
    Live,
    /// A winner was found; a brief results pause before the next round. `until` is when it ends.
    Ended { until: f32 },
}

/// Mutable Rocket-Arena match state, owned by [`GameState`]. Meaningful only while `rtx_mode ra`.
#[derive(Default)]
pub(crate) struct ArenaState {
    pub round: RoundState,
    /// Last countdown second announced, so the "3/2/1" center-print fires once per second rather
    /// than every frame.
    pub last_count: i32,
    /// Monotonic stamp handed out to audience members to order the challenger queue.
    pub serial: u32,
}

/// The Rocket Arena mode descriptor.
pub(crate) struct Arena;

/// Seconds of results pause after a round before the next one forms.
const ROUND_END_PAUSE: f32 = 3.0;
/// Players in the arena at once — Rocket Arena is a 1v1 duel; everyone else waits in the audience.
const FIGHTER_SLOTS: usize = 2;

/// Audience spectating: how many candidate stands spawns to line-of-sight test when re-picking a
/// vantage point (from a random start offset), capping the tracelines per re-pick — mirrors vigil's
/// `MAX_POST_TRIES`. With up to two fighters that's ≤12 traces, only every few seconds.
const VANTAGE_TRIES: usize = 6;
/// Hold a chosen fighter this long (+jitter) before re-picking, so the gaze doesn't ping-pong
/// between the two duelists — mirrors vigil's scan-hold cadence.
const WATCH_HOLD: f32 = 1.2;
const WATCH_JITTER: f32 = 0.8;
/// When no fighter is visible, wait this long before re-testing — so a spectator behind an occluder
/// doesn't pay a traceline per fighter every frame just to keep coming up empty.
const WATCH_RETRY: f32 = 0.4;

impl GameMode for Arena {
    fn name(&self) -> &'static str {
        "ra"
    }

    fn tick(&self, g: &mut GameState) {
        let now = g.time();
        // Queue every waiting player the moment they're in the audience, so the challenger order
        // reflects *when they started waiting* (see `stamp_audience`).
        self.stamp_audience(g);
        match g.arena.round {
            RoundState::Warmup => {
                if count_players(g) >= 2 {
                    self.form_round(g, now);
                }
            }
            RoundState::Countdown { until } => {
                let remaining = (until - now).ceil() as i32;
                if remaining != g.arena.last_count {
                    g.arena.last_count = remaining;
                    if remaining > 0 {
                        centerprint_all(g, &format!("{remaining}"));
                    }
                }
                if now >= until {
                    g.arena.round = RoundState::Live;
                    centerprint_all(g, "FIGHT!");
                }
            }
            RoundState::Live => {
                // A round can also end here if it started under-full or a fighter left; the
                // per-kill path in on_death handles the common case.
                if live_fighters(g).len() <= 1 {
                    self.end_round(g, now);
                }
            }
            RoundState::Ended { until } => {
                if now >= until {
                    // Winner stays a fighter; form the next round (pulls a fresh challenger from
                    // the audience queue), or drop to warmup if there aren't enough players.
                    if count_players(g) >= 2 {
                        self.form_round(g, now);
                    } else {
                        g.arena.round = RoundState::Warmup;
                    }
                }
            }
        }
    }

    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        if g.entities[e].mode_p.arena.role == ArenaRole::Fighter {
            // Fighters spawn inside the arena. On maps without teleport destinations, fall back
            // to the deathmatch spawns so the mode still functions.
            let spot = g.select_spawn_point_of("info_teleport_destination");
            if spot != EntId::WORLD {
                return spot;
            }
        }
        // Audience (and the fallback) use the deathmatch spawns — the stands on an arena map.
        g.select_spawn_point()
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        if g.entities[e].mode_p.arena.role != ArenaRole::Fighter {
            // Audience: the shared harmless-spectator loadout (axe only, no ammo, damage refused).
            super::audience_loadout(g, e);
            return;
        }
        // A rockets-first arena: the full arsenal minus the lightning gun (no cells), mirroring
        // arena.qc:a_newitems defaults. Weapons the server disables via `rtx_weapons` are stripped
        // afterward in put_client_in_server, so this is only the arena's own baseline.
        let arsenal = Items::AXE
            | Items::SHOTGUN
            | Items::SUPER_SHOTGUN
            | Items::NAILGUN
            | Items::SUPER_NAILGUN
            | Items::GRENADE_LAUNCHER
            | Items::ROCKET_LAUNCHER
            | Items::ARMOR3;
        let v = &mut g.entities[e].v;
        v.items = arsenal.as_f32();
        v.health = 100.0;
        v.max_health = 100.0;
        v.armorvalue = 200.0;
        v.armortype = 0.8; // red armor
        v.ammo_shells = 250.0;
        v.ammo_nails = 250.0;
        v.ammo_rockets = 200.0;
        v.ammo_cells = 0.0;
        v.weapon = Weapon::RocketLauncher;
    }

    fn player_damage(
        &self,
        g: &mut GameState,
        targ: EntId,
        _attacker: EntId,
        _inflictor: EntId,
        incoming: f32,
    ) -> DamageOutcome {
        // Only gate players — doors, buttons, grenades, etc. must stay damageable (bots shoot
        // gate buttons to open them).
        let t = &g.entities[targ];
        if t.classname() != Some("player") {
            return DamageOutcome::pass(incoming);
        }
        // Audience is untouchable; fighters are protected until the round goes live. A blocked hit
        // deals nothing and imparts no knockback (spawn protection), as before.
        if t.mode_p.arena.role == ArenaRole::Audience || !matches!(g.arena.round, RoundState::Live) {
            return DamageOutcome::none();
        }
        DamageOutcome::pass(incoming)
    }

    fn weapons_hot(&self, g: &GameState) -> bool {
        // No firing until "FIGHT" — locked out through warmup, countdown, and the post-round pause.
        matches!(g.arena.round, RoundState::Live)
    }

    fn on_death(&self, g: &mut GameState, victim: EntId, _attacker: EntId) {
        if !matches!(g.arena.round, RoundState::Live) {
            return;
        }
        // Eliminated: drop to the audience. The corpse still runs the normal death-think, so on
        // its next respawn `select_spawn`/`apply_loadout` place it in the stands, not the arena.
        g.entities[victim].mode_p.arena.role = ArenaRole::Audience;
        if live_fighters(g).len() <= 1 {
            let now = g.time();
            self.end_round(g, now);
        }
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        match g.entities[bot].mode_p.arena.role {
            ArenaRole::Fighter => {
                // Before the round goes live (countdown), roam the arena to take up a position,
                // rather than standing on the spawn (which looks dead and trips the stuck-jumper).
                if !matches!(g.arena.round, RoundState::Live) {
                    return Some(BotIntent::Move(super::wander_point(g, bot, "info_teleport_destination", |_| None)));
                }
                Some(match nearest_enemy(g, bot) {
                    Some(enemy) => BotIntent::Fight(enemy),
                    None => BotIntent::Move(super::wander_point(g, bot, "info_teleport_destination", |_| None)),
                })
            }
            // Eliminated / waiting: mill around the audience (the stands) but *watch the duel* —
            // stroll to a spot that overlooks the arena and keep the eyes on a live fighter. Only
            // once there's something to watch (a formed round with fighters still up); Warmup keeps
            // the plain wander.
            ArenaRole::Audience => {
                let watching = matches!(
                    g.arena.round,
                    RoundState::Countdown { .. } | RoundState::Live | RoundState::Ended { .. }
                );
                let fighters: Vec<(EntId, Vec3)> = if watching {
                    live_fighters(g)
                        .into_iter()
                        .map(|f| (f, g.entities[f].v.origin + VEC_VIEW_OFS))
                        .collect()
                } else {
                    Vec::new()
                };
                let goal = super::wander_point(g, bot, "info_player_deathmatch", |g| {
                    Self::vantage_spot(g, "info_player_deathmatch", &fighters)
                });
                Some(match self.watch_target(g, bot, &fighters) {
                    Some(w) => BotIntent::Spectate { goal, watch: w },
                    None => BotIntent::Move(goal),
                })
            }
        }
    }
}

impl Arena {
    /// Give any audience member that isn't queued yet a fresh monotonic stamp, so the challenger
    /// queue orders players by *when they entered the audience* — fresh joiners and just-eliminated
    /// fighters alike line up behind everyone already waiting. Run every frame (from [`Self::tick`])
    /// rather than lazily at round formation: a player eliminated mid-match and the players who
    /// joined *while that match was running* all reach the next [`Self::form_round`] with
    /// `queue == 0`, and stamping them there in edict order lets a low-edict eliminated player jump
    /// the queue — the "I lose the round but still play the next one" bug. Stamping as they enter
    /// preserves the true waiting order. A promoted challenger's stamp is cleared to 0 (it's a
    /// fighter now, not waiting); it re-stamps only once eliminated back into the audience.
    fn stamp_audience(&self, g: &mut GameState) {
        for e in players(g) {
            if g.entities[e].mode_p.arena.role == ArenaRole::Audience && g.entities[e].mode_p.arena.queue == 0 {
                g.arena.serial += 1;
                g.entities[e].mode_p.arena.queue = g.arena.serial;
            }
        }
    }

    /// Form the next round: keep any current fighter (the previous winner stays), fill the
    /// remaining arena slots from the front of the audience queue, respawn all fighters into the
    /// arena with a full loadout, and begin the spawn-protected countdown. Everyone else stays in
    /// the audience. This is the "winner stays, loser goes to the back of the queue" duel model —
    /// the arena never holds more than `rtx_ra_fighters` players.
    fn form_round(&self, g: &mut GameState, now: f32) {
        // Everyone waiting is already queued by `stamp_audience` (run every frame), so their order
        // reflects when they started waiting — no lazy stamping here.

        // Fighters carried over from the previous round (the survivor(s)) — snapshot *before* we
        // promote challengers, so we can tell a carried-over winner apart from a fresh challenger
        // (audience members are alive too, so "alive" alone isn't enough).
        let carried: Vec<EntId> = players(g)
            .into_iter()
            .filter(|&e| g.entities[e].mode_p.arena.role == ArenaRole::Fighter)
            .collect();

        // Fill empty fighter slots with the longest-waiting audience members.
        let mut fighters = fighter_count(g);
        while fighters < FIGHTER_SLOTS {
            let next = players(g)
                .into_iter()
                .filter(|&e| g.entities[e].mode_p.arena.role == ArenaRole::Audience && g.entities[e].mode_p.arena.queue != 0)
                .min_by_key(|&e| g.entities[e].mode_p.arena.queue);
            let Some(e) = next else { break }; // not enough players to fill the arena
            g.entities[e].mode_p.arena.role = ArenaRole::Fighter;
            g.entities[e].mode_p.arena.queue = 0;
            fighters += 1;
        }

        // Bring fighters up for the new round. A carried-over winner still alive *stays where they
        // are* and is just topped back up to full — no teleport. Challengers promoted from the
        // audience (and any dead carried-over slot) get a fresh spawn inside the arena.
        for e in players(g) {
            if g.entities[e].mode_p.arena.role != ArenaRole::Fighter {
                continue;
            }
            let winner_stays = carried.contains(&e) && g.entities[e].v.health > 0.0 && g.entities[e].v.deadflag == 0.0;
            if winner_stays {
                self.apply_loadout(g, e);
                // The winner is re-equipped here, not through `put_client_in_server`, so apply the
                // `rtx_weapons` filter ourselves — otherwise a disabled weapon (e.g. the RL) would
                // leak back into the carried-over arsenal each round.
                g.filter_disabled_weapons(e);
                g.w_set_current_ammo(e);
            } else {
                g.put_client_in_server(e);
            }
        }

        let countdown = g.host().cvar(c"rtx_ra_countdown").max(0.0);
        g.arena.round = RoundState::Countdown { until: now + countdown };
        g.arena.last_count = -1;
        g.broadcast(PrintLevel::High, "Rocket Arena: round starting\n");
    }

    /// End the round: credit the survivor (if any) and pause briefly before the next.
    fn end_round(&self, g: &mut GameState, now: f32) {
        if let Some(w) = live_fighters(g).first().copied() {
            g.entities[w].mode_p.arena.round_wins += 1;
            let name = g.netname_of(w);
            let wins = g.entities[w].mode_p.arena.round_wins;
            g.broadcast(PrintLevel::High, &format!("{name} wins the round! ({wins} total)\n"));
        } else {
            g.broadcast(PrintLevel::High, "Round over — no survivor\n");
        }
        g.arena.round = RoundState::Ended {
            until: now + ROUND_END_PAUSE,
        };
    }

    /// A `classname` spawn whose eye position has line of sight to one of the `targets` (live
    /// fighters, `(entity, eye)`), so an audience bot strolls to a spot overlooking the duel. Tries
    /// up to [`VANTAGE_TRIES`] spots from a random offset; `None` (no target, or none with a clear
    /// view) leaves the caller on its plain random wander. Traces run only at re-pick time.
    fn vantage_spot(g: &mut GameState, classname: &str, targets: &[(EntId, Vec3)]) -> Option<Vec3> {
        if targets.is_empty() {
            return None;
        }
        // Snapshot the spawn origins before tracing — `find_by_classname` borrows `&g` immutably,
        // while `traceline` needs `&mut g`.
        let spots: Vec<Vec3> = g.find_by_classname(classname).map(|s| g.entities[s].v.origin).collect();
        if spots.is_empty() {
            return None;
        }
        let start = (g.random() * spots.len() as f32) as usize;
        pick_vantage(&spots, start, VANTAGE_TRIES, |eye| {
            targets.iter().any(|&(f, feye)| Self::sees(g, eye, f, feye))
        })
    }

    /// Choose which live fighter an audience bot's eyes track this frame. Keep the currently held
    /// fighter while it's still live and visible and the hold hasn't lapsed; otherwise re-pick the
    /// nearest *visible* fighter. `None` (no fighter in sight) drops the watch — the eyes fall back
    /// to the walk corridor rather than tracking a duelist through walls — and throttles the next
    /// re-test by [`WATCH_RETRY`] so a blocked view doesn't cost a trace per fighter every frame.
    fn watch_target(&self, g: &mut GameState, bot: EntId, fighters: &[(EntId, Vec3)]) -> Option<EntId> {
        let now = g.time();
        let (held, watch_time) = {
            let b = &g.entities[bot].bot;
            (b.watch_ent, b.watch_time)
        };
        let eye = g.entities[bot].v.origin + VEC_VIEW_OFS;

        // Keep the held fighter if it's still a live target, still in sight, and the hold is unexpired.
        let held_ok = held != 0
            && now < watch_time
            && fighters.iter().any(|&(f, feye)| f.0 == held && Self::sees(g, eye, f, feye));
        if held_ok {
            return Some(EntId(held));
        }

        // Re-pick: nearest fighter we can actually see.
        let candidates: Vec<(EntId, f32, bool)> = fighters
            .iter()
            .map(|&(f, feye)| {
                let dist = (g.entities[f].v.origin - g.entities[bot].v.origin).length();
                (f, dist, Self::sees(g, eye, f, feye))
            })
            .collect();
        let pick = pick_watch(&candidates);
        let jitter = g.random();
        let b = &mut g.entities[bot].bot;
        match pick {
            Some(w) => {
                b.watch_ent = w.0;
                b.watch_time = now + WATCH_HOLD + jitter * WATCH_JITTER;
                Some(w)
            }
            None => {
                b.watch_ent = 0;
                b.watch_time = now + WATCH_RETRY;
                None
            }
        }
    }

    /// Clear line of sight from `eye` to fighter `f`'s eye — the perception LOS test (a trace that
    /// stops on the fighter, or gets ≥95% of the way).
    fn sees(g: &mut GameState, eye: Vec3, f: EntId, feye: Vec3) -> bool {
        let tr = g.traceline(eye, feye, false, EntId::WORLD);
        tr.ent == f || tr.fraction > 0.95
    }
}

/// First of `spots` (scanning up to `tries` from `start`, wrapping) whose eye point satisfies
/// `sees`. Eye = spot + [`VEC_VIEW_OFS`]. `None` if none in the window has a view. Pure but for the
/// caller's `sees` closure, so the try-window / wrap / offset logic is unit-testable.
fn pick_vantage(spots: &[Vec3], start: usize, tries: usize, mut sees: impl FnMut(Vec3) -> bool) -> Option<Vec3> {
    if spots.is_empty() {
        return None;
    }
    let start = start % spots.len();
    for i in 0..tries.min(spots.len()) {
        let spot = spots[(start + i) % spots.len()];
        if sees(spot + VEC_VIEW_OFS) {
            return Some(spot);
        }
    }
    None
}

/// The nearest *visible* fighter among `(entity, distance, visible)` candidates — the one the eyes
/// should track. `None` when none is visible (drop the watch).
fn pick_watch(cands: &[(EntId, f32, bool)]) -> Option<EntId> {
    cands
        .iter()
        .filter(|&&(_, _, vis)| vis)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|&(f, _, _)| f)
}

/// Number of connected players (fighter-eligible).
fn count_players(g: &GameState) -> usize {
    players(g).len()
}

/// How many players currently hold a fighter slot (alive or dead this round).
fn fighter_count(g: &GameState) -> usize {
    players(g)
        .into_iter()
        .filter(|&e| g.entities[e].mode_p.arena.role == ArenaRole::Fighter)
        .count()
}

/// Fighters still alive this round.
fn live_fighters(g: &GameState) -> Vec<EntId> {
    players(g)
        .into_iter()
        .filter(|&e| {
            let ent = &g.entities[e];
            ent.mode_p.arena.role == ArenaRole::Fighter && ent.v.health > 0.0 && ent.v.deadflag == 0.0
        })
        .collect()
}

/// The nearest living opposing fighter to `bot` (everyone else is an enemy — this is a duel /
/// last-man-standing round, no teams).
fn nearest_enemy(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    nearest_player_where(g, origin, bot, |g, e| g.entities[e].mode_p.arena.role == ArenaRole::Fighter)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vantage_returns_first_visible_from_offset() {
        // Spots 0 and 2 have a clear view; from start=1 the scan hits spot 2 first.
        let spots = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 0.0), Vec3::new(200.0, 0.0, 0.0)];
        let sees = |eye: Vec3| eye.x < 50.0 || eye.x > 150.0;
        let pick = pick_vantage(&spots, 1, VANTAGE_TRIES, sees);
        assert_eq!(pick, Some(spots[2]), "wraps from the offset and takes the first visible");
        // The eye offset is applied, not the raw origin.
        let mut seen_eye = Vec3::ZERO;
        pick_vantage(&spots, 0, 1, |eye| {
            seen_eye = eye;
            true
        });
        assert_eq!(seen_eye, spots[0] + VEC_VIEW_OFS, "tests eye height, not the floor");
    }

    #[test]
    fn vantage_none_when_all_blind_and_respects_try_budget() {
        let spots = [Vec3::ZERO, Vec3::new(100.0, 0.0, 0.0), Vec3::new(200.0, 0.0, 0.0)];
        assert_eq!(pick_vantage(&spots, 0, VANTAGE_TRIES, |_| false), None, "no view anywhere → None");
        // Only spot 2 is visible; a 2-try budget from start 0 never reaches it.
        let sees = |eye: Vec3| eye.x > 150.0;
        assert_eq!(pick_vantage(&spots, 0, 2, sees), None, "try budget caps the scan window");
        assert_eq!(pick_vantage(&spots, 0, 3, sees), Some(spots[2]), "a wider budget finds it");
        assert_eq!(pick_vantage(&[], 0, VANTAGE_TRIES, |_| true), None, "no spots → None");
    }

    #[test]
    fn watch_prefers_nearest_visible() {
        let a = EntId(3);
        let b = EntId(4);
        let c = EntId(5);
        // b is nearer than a but hidden; a (visible, farther) wins over the hidden nearer one.
        let cands = [(a, 300.0, true), (b, 100.0, false), (c, 500.0, true)];
        assert_eq!(pick_watch(&cands), Some(a), "nearest *visible*, skipping the hidden nearer fighter");
        assert_eq!(pick_watch(&[(a, 100.0, false), (b, 200.0, false)]), None, "all hidden → drop the watch");
        assert_eq!(pick_watch(&[]), None, "no fighters → None");
    }
}
