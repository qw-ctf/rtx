// SPDX-License-Identifier: AGPL-3.0-or-later

//! Rocket Arena (`rtx_mode ra`) — the first mode layered on the [`Ffa`](super::Ffa) baseline.
//!
//! This follows the "rocket arena part" of the reference Frogbot-Rocket-Arena QuakeC
//! (`fbra/src/arena.qc`): round-based play where everyone spawns with a **full loadout**,
//! fighters duel **inside the arena** while eliminated/waiting players roam the **audience**,
//! damage is off during a short spawn-protected countdown, and the last player standing wins the
//! round. We deliberately drop FBRA's clan-arena machinery (color/shirt teams, team menus,
//! series-to-N-wins) — that granularity belongs to the future 1on1/2on2/4on4 modes.
//!
//! The arena/audience split reuses FBRA's spawn-point trick (`arena.qc:set_a_spawn`): fighters
//! spawn at `info_teleport_destination` (inside the arena on a Rocket-Arena map), audience at
//! `info_player_deathmatch` (the stands). On a plain deathmatch map without teleport
//! destinations it falls back to DM spawns so `ra` still runs.

use glam::Vec3;

use super::{ArenaRole, BotIntent, GameMode};
use crate::defs::{Items, PrintLevel, Weapon};
use crate::entity::EntId;
use crate::game::{cstring, GameState};

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

impl GameMode for Arena {
    fn name(&self) -> &'static str {
        "ra"
    }

    fn tick(&self, g: &mut GameState) {
        let now = g.time();
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
        if g.entities[e].arena.role == ArenaRole::Fighter {
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
        let fighter = g.entities[e].arena.role == ArenaRole::Fighter;
        // The lightning gun is off by default (rtx_ra_lightning_gun 0) — a rockets-first arena.
        let lightning = g.host().cvar(c"rtx_ra_lightning_gun") != 0.0;
        let v = &mut g.entities[e].v;
        if fighter {
            // Full arsenal + full ammo + red armor, mirroring arena.qc:a_newitems defaults.
            let arsenal = Items::AXE
                | Items::SHOTGUN
                | Items::SUPER_SHOTGUN
                | Items::NAILGUN
                | Items::SUPER_NAILGUN
                | Items::GRENADE_LAUNCHER
                | Items::ROCKET_LAUNCHER
                | Items::ARMOR3;
            let arsenal = if lightning { arsenal | Items::LIGHTNING } else { arsenal };
            v.items = arsenal.as_f32();
            v.health = 100.0;
            v.max_health = 100.0;
            v.armorvalue = 200.0;
            v.armortype = 0.8; // red armor
            v.ammo_shells = 250.0;
            v.ammo_nails = 250.0;
            v.ammo_rockets = 200.0;
            v.ammo_cells = if lightning { 200.0 } else { 0.0 };
            v.weapon = Weapon::RocketLauncher;
        } else {
            // Audience: axe only, no ammo — harmless spectators wandering the stands (damage to
            // them is refused, see `damage_allowed`). Health/armor must stay positive: a client
            // (and the bot AI) treats 0 health as dead and locks movement, so they'd freeze.
            v.items = Items::AXE.as_f32();
            v.health = 100.0;
            v.armorvalue = 100.0;
            v.armortype = 0.8;
            v.ammo_shells = 0.0;
            v.ammo_nails = 0.0;
            v.ammo_rockets = 0.0;
            v.ammo_cells = 0.0;
            v.weapon = Weapon::Axe;
        }
    }

    fn damage_allowed(&self, g: &GameState, targ: EntId) -> bool {
        // Only gate players — doors, buttons, grenades, etc. must stay damageable (bots shoot
        // gate buttons to open them).
        let t = &g.entities[targ];
        if t.classname() != Some("player") {
            return true;
        }
        // Audience is untouchable; fighters are protected until the round goes live.
        if t.arena.role == ArenaRole::Audience {
            return false;
        }
        matches!(g.arena.round, RoundState::Live)
    }

    fn on_death(&self, g: &mut GameState, victim: EntId, _attacker: EntId) {
        if !matches!(g.arena.round, RoundState::Live) {
            return;
        }
        // Eliminated: drop to the audience. The corpse still runs the normal death-think, so on
        // its next respawn `select_spawn`/`apply_loadout` place it in the stands, not the arena.
        g.entities[victim].arena.role = ArenaRole::Audience;
        if live_fighters(g).len() <= 1 {
            let now = g.time();
            self.end_round(g, now);
        }
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        match g.entities[bot].arena.role {
            ArenaRole::Fighter => {
                // Before the round goes live (countdown), and if somehow no enemy is left, hold
                // position on the spawn spot rather than run off into the map.
                if !matches!(g.arena.round, RoundState::Live) {
                    return Some(BotIntent::Move(g.entities[bot].v.origin));
                }
                Some(match nearest_enemy(g, bot) {
                    Some(enemy) => BotIntent::Fight(enemy),
                    None => BotIntent::Move(g.entities[bot].v.origin),
                })
            }
            // Eliminated / waiting: mill around the audience (the deathmatch spawns / stands).
            ArenaRole::Audience => Some(BotIntent::Move(self.wander_point(g, bot))),
        }
    }
}

impl Arena {
    /// Form the next round: keep any current fighter (the previous winner stays), fill the
    /// remaining arena slots from the front of the audience queue, respawn all fighters into the
    /// arena with a full loadout, and begin the spawn-protected countdown. Everyone else stays in
    /// the audience. This is the "winner stays, loser goes to the back of the queue" duel model —
    /// the arena never holds more than `rtx_ra_fighters` players.
    fn form_round(&self, g: &mut GameState, now: f32) {
        // Stamp any audience member that isn't queued yet (fresh joiners, and players just
        // eliminated — whose stamp was cleared when they last became a fighter), so they line up
        // behind those already waiting.
        for e in players(g) {
            if g.entities[e].arena.role == ArenaRole::Audience && g.entities[e].arena.queue == 0 {
                g.arena.serial += 1;
                g.entities[e].arena.queue = g.arena.serial;
            }
        }

        // Fill empty fighter slots with the longest-waiting audience members.
        let mut fighters = fighter_count(g);
        while fighters < FIGHTER_SLOTS {
            let next = players(g)
                .into_iter()
                .filter(|&e| {
                    g.entities[e].arena.role == ArenaRole::Audience && g.entities[e].arena.queue != 0
                })
                .min_by_key(|&e| g.entities[e].arena.queue);
            let Some(e) = next else { break }; // not enough players to fill the arena
            g.entities[e].arena.role = ArenaRole::Fighter;
            g.entities[e].arena.queue = 0;
            fighters += 1;
        }

        // Respawn every fighter into the arena, fresh.
        for e in players(g) {
            if g.entities[e].arena.role == ArenaRole::Fighter {
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
            g.entities[w].arena.round_wins += 1;
            let name = g.netname_of(w);
            let wins = g.entities[w].arena.round_wins;
            g.broadcast(
                PrintLevel::High,
                &format!("{name} wins the round! ({wins} total)\n"),
            );
        } else {
            g.broadcast(PrintLevel::High, "Round over — no survivor\n");
        }
        g.arena.round = RoundState::Ended {
            until: now + ROUND_END_PAUSE,
        };
    }

    /// The audience destination for a waiting bot: a deathmatch spawn (the stands), re-picked on a
    /// staggered timer so it strolls between vantage points instead of freezing. This is the
    /// arena's whole audience-roaming brain — kept here, out of the generic bot code.
    fn wander_point(&self, g: &mut GameState, bot: EntId) -> Vec3 {
        let now = g.time();
        let need = g.entities[bot].bot.wander_time <= now
            || g.entities[bot].bot.wander_target == Vec3::ZERO;
        if need {
            let spot = g.select_spawn_point_of("info_player_deathmatch");
            let origin = g.entities[bot].v.origin;
            let target = if spot != EntId::WORLD {
                g.entities[spot].v.origin
            } else {
                origin
            };
            let jitter = g.random();
            let b = &mut g.entities[bot].bot;
            b.wander_target = target;
            b.wander_time = now + 3.0 + jitter * 3.0;
        }
        g.entities[bot].bot.wander_target
    }
}

/// Every connected player edict (humans and bots occupy `1..=maxclients`).
fn players(g: &GameState) -> Vec<EntId> {
    let maxclients = g.host().cvar(c"maxclients") as i32;
    (1..=maxclients as u32)
        .map(EntId)
        .filter(|&e| g.entities[e].in_use && g.entities[e].classname() == Some("player"))
        .collect()
}

/// Number of connected players (fighter-eligible).
fn count_players(g: &GameState) -> usize {
    players(g).len()
}

/// How many players currently hold a fighter slot (alive or dead this round).
fn fighter_count(g: &GameState) -> usize {
    players(g)
        .into_iter()
        .filter(|&e| g.entities[e].arena.role == ArenaRole::Fighter)
        .count()
}

/// Fighters still alive this round.
fn live_fighters(g: &GameState) -> Vec<EntId> {
    players(g)
        .into_iter()
        .filter(|&e| {
            let ent = &g.entities[e];
            ent.arena.role == ArenaRole::Fighter && ent.v.health > 0.0 && ent.v.deadflag == 0.0
        })
        .collect()
}

/// The nearest living opposing fighter to `bot` (everyone else is an enemy — this is a duel /
/// last-man-standing round, no teams).
fn nearest_enemy(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    live_fighters(g)
        .into_iter()
        .filter(|&e| e != bot)
        .min_by(|&a, &b| {
            let da = (g.entities[a].v.origin - origin).length_squared();
            let db = (g.entities[b].v.origin - origin).length_squared();
            da.total_cmp(&db)
        })
}

/// Center-print a message to every connected human. Bots are fake clients with no connection, so
/// a unicast to one makes the engine complain ("msg_entity: not a client") — skip them.
fn centerprint_all(g: &GameState, msg: &str) {
    let host = *g.host();
    let cmsg = cstring(msg);
    for e in players(g) {
        if g.entities[e].bot.is_bot {
            continue;
        }
        host.centerprint(e, &cmsg);
    }
}
