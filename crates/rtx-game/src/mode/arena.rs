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

/// Where the round is in its lifecycle. The whole mode is this five-state machine, advanced once
/// per server frame by [`Arena::tick`] and on each kill by [`Arena::on_death`].
#[derive(Default, Clone, Copy)]
pub(crate) enum RoundState {
    /// Waiting for enough players to start a round.
    #[default]
    Warmup,
    /// Fighters assigned, but at least one couldn't be placed yet because its spawn area wasn't
    /// clear (a single-spawn map, an occupied arena). [`Arena::tick`] keeps retrying the placement
    /// and arms the countdown once every fighter is in. `since` is when forming began. Treated as
    /// "not live" everywhere (damage gated, weapons cold), exactly like `Countdown`.
    Forming { since: f32 },
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
    /// than every frame. Reset to -1 by `arm_countdown`.
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
/// How long to wait for a pending fighter's spawn area to clear before forcing the placement (which
/// slides it a hull-width off any occupant via [`spread_spawn`]). Bounds the single-spawn stall so
/// the round always starts.
const FORMING_TIMEOUT: f32 = 3.0;

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
        // Publish the round phase in KTX's `status` vocabulary — a round is Rocket Arena's match, so
        // it drives the same key a team match does, and a KTX-aware HUD shows the arena countdown for
        // free. Deduped, so this only touches the wire on a phase change.
        let status = match g.arena.round {
            RoundState::Countdown { .. } => "Countdown",
            RoundState::Live => "in progress",
            _ => "Standby", // Warmup / Forming / Ended
        };
        g.publish_serverinfo("status", status);
        match g.arena.round {
            RoundState::Warmup => {
                if count_players(g) >= 2 {
                    self.form_round(g, now);
                }
            }
            RoundState::Forming { since } => {
                if count_players(g) < 2 {
                    // Lost a player before the round could start — abandon it and clear the flags.
                    for e in players(g) {
                        g.entities[e].mode_p.arena.pending_spawn = false;
                    }
                    g.arena.round = RoundState::Warmup;
                } else if fighter_count(g) < FIGHTER_SLOTS {
                    // A pending fighter left mid-forming but others still wait — re-form to pull a
                    // fresh challenger and re-arm cleanly.
                    self.form_round(g, now);
                } else if now - since >= FORMING_TIMEOUT {
                    // The area never cleared (a single-spawn map, an occupant parked on the spot) —
                    // force the remaining fighters in. `spread_spawn` (in `adjust_spawn_origin`)
                    // slides each a hull-width off the occupant, so they spawn apart, not stacked.
                    self.force_place_pending(g);
                    self.arm_countdown(g, now);
                } else {
                    // Keep retrying the clean placement; the timeout above is the backstop.
                    self.place_pending(g);
                    if !any_pending(g) {
                        self.arm_countdown(g, now);
                    }
                }
            }
            RoundState::Countdown { until } => {
                let (last, print) = super::countdown_announce(until, now, g.arena.last_count);
                g.arena.last_count = last;
                if let Some(n) = print {
                    centerprint_all(g, &format!("{n}"));
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
        // Any spawn (this one is about to happen) settles a pending placement — clear it here, the
        // one point every game-driven spawn path funnels through, so no path can double-place.
        g.entities[e].mode_p.arena.pending_spawn = false;
        if g.entities[e].mode_p.arena.role == ArenaRole::Fighter {
            // Fighters spawn inside the arena. On maps without teleport destinations, fall back
            // to the deathmatch spawns so the mode still functions.
            let spot = g.select_spawn_point_of("info_teleport_destination", Some(e));
            if spot != EntId::WORLD {
                return spot;
            }
        }
        // Audience (and the fallback) use the deathmatch spawns — the stands on an arena map.
        g.select_spawn_point(Some(e))
    }

    fn spawn_rules_live(&self, g: &GameState) -> bool {
        // The arena's "actually playing" state is a live round, not the team-match lifecycle.
        matches!(g.arena.round, RoundState::Live)
    }

    fn spawn_area_clear(&self, g: &GameState, e: EntId) -> bool {
        // The pool `select_spawn` will actually draw from must have a free spot. A degenerate map
        // with no pool (info_player_start only) can't wedge meaningfully — treat it as clear.
        spawn_pool(g, e).is_none_or(|c| g.has_free_spawn_of(c, e))
    }

    fn adjust_spawn_origin(&self, g: &mut GameState, e: EntId, origin: Vec3) -> Vec3 {
        // The pre-round damage gate nullifies the spawn telefrag, so a spot shared with another
        // live player would wedge both forever. Slide off to a free adjacent point instead — this
        // is what lets a single-spawn arena still run (fighters spawn a hull-width apart).
        spread_spawn(g, e, origin)
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
        super::Loadout {
            items: arsenal,
            health: 100.0,
            max_health: Some(100.0),
            armorvalue: 200.0,
            armortype: 0.8, // red armor
            shells: 250.0,
            nails: 250.0,
            rockets: 200.0,
            cells: 0.0,
            weapon: Weapon::RocketLauncher,
        }
        .apply(g, e);
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
        if !t.is_player() {
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

    fn untouchable_bystander(&self, g: &GameState, e: EntId) -> bool {
        // The audience is solid but damage-refused, so a spawn telefrag can't clear them.
        g.entities[e].mode_p.arena.role == ArenaRole::Audience
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

    fn allow_respawn(&self, g: &GameState, e: EntId) -> bool {
        // Hold a death-think respawn (into the stands, or the arena) until it won't wedge into
        // another player the pre-round telefrag can't clear. A dead bot keeps pulsing +attack and a
        // human keeps pressing, so the respawn fires as soon as the area frees.
        self.spawn_area_clear(g, e)
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        match g.entities[bot].mode_p.arena.role {
            ArenaRole::Fighter => {
                // Before the round goes live (countdown), roam to take up a position rather than
                // standing on the spawn (which looks dead and trips the stuck-jumper). A promoted-
                // but-not-yet-placed fighter is still physically in the stands, so it roams the
                // audience pool until placed; a placed fighter roams its own arena pool. Using the
                // resolved pool (not a hardcoded classname) keeps a teleport-dest-less map — where
                // fighters fall back to DM spawns — from degenerating to a stand-still wander.
                if !matches!(g.arena.round, RoundState::Live) {
                    let pool = if g.entities[bot].mode_p.arena.pending_spawn {
                        "info_player_deathmatch"
                    } else {
                        spawn_pool(g, bot).unwrap_or("info_player_deathmatch")
                    };
                    return Some(BotIntent::Move(super::wander_point(g, bot, pool, |_| None)));
                }
                let pool = spawn_pool(g, bot).unwrap_or("info_player_deathmatch");
                Some(match nearest_enemy(g, bot) {
                    Some(enemy) => BotIntent::Fight(enemy),
                    None => BotIntent::Move(super::wander_point(g, bot, pool, |_| None)),
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

    fn bot_idle_roam(&self, g: &mut GameState, bot: EntId) -> Option<Vec3> {
        // No opponent in sight and no goal to fetch: roam our *own* space, never the whole map.
        // `spawn_pool` already splits by role — a fighter's arena (its teleport-destination ring), an
        // audience member's deathmatch stands — so a fighter picked to duel never sets a roam target
        // among the unreachable stands cells and jams itself into the wall below the audience trying
        // to reach it, and the audience keep to the stands. Mirrors the Fighter arm's own no-enemy
        // fallback (`bot_intent` above), which is why they read the same pool.
        let pool = spawn_pool(g, bot).unwrap_or("info_player_deathmatch");
        Some(super::wander_point(g, bot, pool, |_| None))
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
    /// remaining arena slots from the front of the audience queue, spawn the fresh fighters into the
    /// arena with a full loadout, and begin the spawn-protected countdown. Everyone else stays in
    /// the audience. This is the "winner stays, loser goes to the back of the queue" duel model —
    /// the arena never holds more than `rtx_ra_fighters` players.
    ///
    /// A fighter whose arena spot isn't clear yet is left `pending_spawn` and placed later by
    /// [`Self::tick`]'s [`RoundState::Forming`] arm; the countdown is armed only once every fighter
    /// is actually in the arena.
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

        self.promote_challengers(g);

        // Bring fighters up for the new round. A carried-over winner still alive *stays where they
        // are* and is just topped back up to full — no teleport, never pending. Everyone else
        // (promoted challengers, dead carried-over slots) needs a fresh arena spawn: mark them
        // pending, then place any whose spot is already clear.
        for e in players(g) {
            if g.entities[e].mode_p.arena.role != ArenaRole::Fighter {
                continue;
            }
            let winner_stays = carried.contains(&e) && g.entities[e].is_alive();
            if winner_stays {
                g.entities[e].mode_p.arena.pending_spawn = false;
                self.apply_loadout(g, e);
                // The winner is re-equipped here, not through `put_client_in_server`, so apply the
                // `rtx_weapons` filter ourselves — otherwise a disabled weapon (e.g. the RL) would
                // leak back into the carried-over arsenal each round.
                g.filter_disabled_weapons(e);
                g.w_set_current_ammo(e);
                // Same reason the opponent model must be reset by hand: the winner never crosses the
                // respawn path (`grant_spawn_loadout`) that resets every other spawn, so without this
                // the shared belief keeps last round's low health — and the next challenger reads a
                // full-stack survivor as "nearly dead" and shotgun-rushes them. Restore the RA
                // spawn baseline (100 hp / 200 red), matching the loadout just applied.
                g.model_reset_target(e);
            } else {
                g.entities[e].mode_p.arena.pending_spawn = true;
            }
        }
        self.place_pending(g);

        if any_pending(g) {
            // At least one fighter couldn't be placed — hold in Forming until the area frees or the
            // timeout forces a spread placement.
            g.arena.round = RoundState::Forming { since: now };
            g.broadcast(PrintLevel::High, "Rocket Arena: waiting for the spawn area to clear\n");
        } else {
            self.arm_countdown(g, now);
        }
    }

    /// Fill empty fighter slots with the longest-waiting audience members (lowest queue stamp).
    /// Promotion only changes the role/queue; the actual arena placement is done by [`Self::form_round`]
    /// / [`Self::place_pending`].
    fn promote_challengers(&self, g: &mut GameState) {
        let mut fighters = fighter_count(g);
        while fighters < FIGHTER_SLOTS {
            let next = players(g)
                .into_iter()
                .filter(|&e| {
                    g.entities[e].mode_p.arena.role == ArenaRole::Audience && g.entities[e].mode_p.arena.queue != 0
                })
                .min_by_key(|&e| g.entities[e].mode_p.arena.queue);
            let Some(e) = next else { break }; // not enough players to fill the arena
            g.entities[e].mode_p.arena.role = ArenaRole::Fighter;
            g.entities[e].mode_p.arena.queue = 0;
            fighters += 1;
        }
    }

    /// Place every `pending_spawn` fighter whose spawn area is now clear (`put_client_in_server`
    /// self-clears the flag via `select_spawn`). Sequential, so each placement is seen by the next
    /// check — two fighters won't both claim the last free spot in one pass.
    fn place_pending(&self, g: &mut GameState) {
        for e in players(g) {
            let pending = {
                let a = &g.entities[e].mode_p.arena;
                a.role == ArenaRole::Fighter && a.pending_spawn
            };
            if pending && self.spawn_area_clear(g, e) {
                g.put_client_in_server(e);
            }
        }
    }

    /// Place every remaining `pending_spawn` fighter unconditionally — the [`FORMING_TIMEOUT`]
    /// backstop when the area never clears. `put_client_in_server` spreads each off any occupant
    /// (via [`Arena::adjust_spawn_origin`]), so this un-stalls the round without stacking players.
    fn force_place_pending(&self, g: &mut GameState) {
        for e in players(g) {
            let pending = {
                let a = &g.entities[e].mode_p.arena;
                a.role == ArenaRole::Fighter && a.pending_spawn
            };
            if pending {
                g.put_client_in_server(e);
            }
        }
    }

    /// Arm the spawn-protected countdown once all fighters are in the arena.
    fn arm_countdown(&self, g: &mut GameState, now: f32) {
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
            (b.watch.ent, b.watch.time)
        };
        let eye = g.entities[bot].v.origin + VEC_VIEW_OFS;

        // Keep the held fighter if it's still a live target, still in sight, and the hold is unexpired.
        let held_ok = held != 0
            && now < watch_time
            && fighters
                .iter()
                .any(|&(f, feye)| f.0 == held && Self::sees(g, eye, f, feye));
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
                b.watch.ent = w.0;
                b.watch.time = now + WATCH_HOLD + jitter * WATCH_JITTER;
                Some(w)
            }
            None => {
                b.watch.ent = 0;
                b.watch.time = now + WATCH_RETRY;
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

/// Is any fighter still waiting to be placed in the arena this round?
fn any_pending(g: &GameState) -> bool {
    players(g).into_iter().any(|e| {
        let a = &g.entities[e].mode_p.arena;
        a.role == ArenaRole::Fighter && a.pending_spawn
    })
}

/// Just over the player hull half-width (16) doubled — the closest two hulls can sit before they
/// overlap. Spawn origins nearer than this share space and wedge.
const SPREAD_MIN: f32 = 34.0;
/// Ring radii tried when sliding a spawn off an occupant, in [`SPREAD_MIN`]-ish steps.
const SPREAD_RINGS: [f32; 2] = [40.0, 80.0];

/// Slide `origin` off any other live player, so no two players are relinked onto the same point (a
/// pre-round arena can't telefrag them apart). Returns `origin` untouched when it's already clear —
/// the common case, so a normal spawn pays only one proximity scan. Otherwise the first free point
/// on a ring around it that also has clear line from `origin` (not through a wall); `origin` as the
/// last resort if the spot is fully boxed in.
fn spread_spawn(g: &mut GameState, spawning: EntId, origin: Vec3) -> Vec3 {
    if !origin_crowded(g, origin, spawning) {
        return origin;
    }
    for &ring in &SPREAD_RINGS {
        for i in 0..8 {
            let a = i as f32 * std::f32::consts::FRAC_PI_4;
            let cand = origin + Vec3::new(ring * a.cos(), ring * a.sin(), 0.0);
            if origin_crowded(g, cand, spawning) {
                continue;
            }
            // Reject a candidate a wall away from the spot (would land outside the arena / in geo).
            if g.traceline(origin, cand, false, spawning).fraction > 0.99 {
                return cand;
            }
        }
    }
    origin
}

/// Any live player other than `who` within [`SPREAD_MIN`] of point `p`.
fn origin_crowded(g: &GameState, p: Vec3, who: EntId) -> bool {
    players(g)
        .into_iter()
        .any(|e| e != who && g.entities[e].is_alive() && (g.entities[e].v.origin - p).length() < SPREAD_MIN)
}

/// The spawn-point classname `select_spawn` will actually draw from for `e` — the single source of
/// truth shared by the spawn picker and the [`Arena::spawn_area_clear`] gate, so the two never
/// disagree. Fighters spawn at `info_teleport_destination` when the map has any (even all-occupied —
/// `select_spawn` returns one regardless), else fall back to the deathmatch spawns; audience always
/// uses the deathmatch spawns. `None` on a degenerate map with neither (an `info_player_start`-only
/// map), where there's no pool to gate on.
fn spawn_pool(g: &GameState, e: EntId) -> Option<&'static str> {
    let fighter = g.entities[e].mode_p.arena.role == ArenaRole::Fighter;
    let has_tele = g.find_by_classname("info_teleport_destination").next().is_some();
    let has_dm = g.find_by_classname("info_player_deathmatch").next().is_some();
    spawn_pool_class(fighter, has_tele, has_dm)
}

/// The spawn-pool decision, extracted pure: a fighter takes the arena's teleport destinations when
/// the map has any (never falling back to DM while they exist), otherwise the deathmatch spawns;
/// audience always takes the deathmatch spawns. `None` when the relevant pool is absent.
fn spawn_pool_class(fighter: bool, has_tele: bool, has_dm: bool) -> Option<&'static str> {
    if fighter && has_tele {
        Some("info_teleport_destination")
    } else if has_dm {
        Some("info_player_deathmatch")
    } else {
        None
    }
}

/// Fighters still alive this round.
fn live_fighters(g: &GameState) -> Vec<EntId> {
    players(g)
        .into_iter()
        .filter(|&e| {
            let ent = &g.entities[e];
            ent.mode_p.arena.role == ArenaRole::Fighter && ent.is_alive()
        })
        .collect()
}

/// The nearest living opposing fighter to `bot` (everyone else is an enemy — this is a duel /
/// last-man-standing round, no teams).
fn nearest_enemy(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    nearest_player_where(g, origin, bot, |g, e| {
        g.entities[e].mode_p.arena.role == ArenaRole::Fighter
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawn_pool_prefers_arena_for_fighters() {
        // On a real arena map, fighters take the teleport destinations even though DM spawns exist.
        assert_eq!(spawn_pool_class(true, true, true), Some("info_teleport_destination"));
        // A fighter on a teleport-dest-less map falls back to the DM spawns (mode still runs).
        assert_eq!(spawn_pool_class(true, false, true), Some("info_player_deathmatch"));
        // Audience always uses the DM spawns (the stands), teleport dests or not.
        assert_eq!(spawn_pool_class(false, true, true), Some("info_player_deathmatch"));
        assert_eq!(spawn_pool_class(false, false, true), Some("info_player_deathmatch"));
        // Degenerate map (info_player_start only): no pool to gate on.
        assert_eq!(spawn_pool_class(true, false, false), None);
        assert_eq!(spawn_pool_class(false, false, false), None);
    }

    #[test]
    fn vantage_returns_first_visible_from_offset() {
        // Spots 0 and 2 have a clear view; from start=1 the scan hits spot 2 first.
        let spots = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(100.0, 0.0, 0.0),
            Vec3::new(200.0, 0.0, 0.0),
        ];
        let sees = |eye: Vec3| eye.x < 50.0 || eye.x > 150.0;
        let pick = pick_vantage(&spots, 1, VANTAGE_TRIES, sees);
        assert_eq!(
            pick,
            Some(spots[2]),
            "wraps from the offset and takes the first visible"
        );
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
        assert_eq!(
            pick_vantage(&spots, 0, VANTAGE_TRIES, |_| false),
            None,
            "no view anywhere → None"
        );
        // Only spot 2 is visible; a 2-try budget from start 0 never reaches it.
        let sees = |eye: Vec3| eye.x > 150.0;
        assert_eq!(
            pick_vantage(&spots, 0, 2, sees),
            None,
            "try budget caps the scan window"
        );
        assert_eq!(
            pick_vantage(&spots, 0, 3, sees),
            Some(spots[2]),
            "a wider budget finds it"
        );
        assert_eq!(pick_vantage(&[], 0, VANTAGE_TRIES, |_| true), None, "no spots → None");
    }

    #[test]
    fn watch_prefers_nearest_visible() {
        let a = EntId(3);
        let b = EntId(4);
        let c = EntId(5);
        // b is nearer than a but hidden; a (visible, farther) wins over the hidden nearer one.
        let cands = [(a, 300.0, true), (b, 100.0, false), (c, 500.0, true)];
        assert_eq!(
            pick_watch(&cands),
            Some(a),
            "nearest *visible*, skipping the hidden nearer fighter"
        );
        assert_eq!(
            pick_watch(&[(a, 100.0, false), (b, 200.0, false)]),
            None,
            "all hidden → drop the watch"
        );
        assert_eq!(pick_watch(&[]), None, "no fighters → None");
    }
}
