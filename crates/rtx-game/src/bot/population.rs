// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot population management: reconcile the live bot count to `rtx_bot_count` (one add/remove per
//! call), and the deferred roster (`RosterOp` / `drain_roster`) that applies bot add/remove *outside*
//! the `vmMain` borrow so the re-entrant spawn those host traps trigger can't alias a live `&mut`.

use std::ffi::CString;

use super::state::BotState;
use crate::entity::EntId;
use crate::game::GameState;

/// Reconcile the live bot count to `rtx_bot_count`, one add/remove per call (called each normal
/// server frame). No-ops until a navmesh exists for the map, so bots never spawn blind.
pub fn manage_population(game: &mut GameState) {
    let host = *game.host();
    let maxclients = host.cvar(c"maxclients") as i32;

    // Tally humans and bots in one pass.
    let (mut humans, mut count, mut last_bot) = (0, 0, None);
    for i in 1..=maxclients as u32 {
        let ent = &game.entities[EntId(i)];
        if ent.bot.is_bot {
            count += 1;
            last_bot = Some(EntId(i));
        } else if ent.in_use && ent.is_player() {
            humans += 1;
        }
    }

    // Field bots while at least one human is in the game — an empty server (or one whose last human
    // just left) wants none, so the trim path below removes them. `rtx_bot_alone` overrides that,
    // keeping bots on even with no humans (a demo/idle server that plays itself).
    let want = if humans >= 1 || host.cvar_bool(c"rtx_bot_alone") {
        host.cvar(c"rtx_bot_count").max(0.0) as i32
    } else {
        0
    };

    // In a structured match, cap the fill to the empty seats during warmup and freeze the roster once
    // the match is under way — see `bot_target`.
    let in_warmup = matches!(game.team_match.phase, crate::mode::MatchPhase::Warmup);
    if game.team_match.config.teams >= 2 && game.team_match.config.size >= 1 && !in_warmup {
        let active: Vec<String> = (1..=maxclients as u32)
            .map(EntId)
            .filter(|&e| game.entities[e].bot.is_bot)
            .map(|e| game.netname_of(e))
            .collect();
        let missing = game
            .team_match
            .bot_roster
            .iter()
            .enumerate()
            .find(|(_, name)| !active.contains(name))
            .map(|(index, name)| (index as i32, name.clone()));
        if let Some((index, name)) = missing {
            game.ensure_navmesh();
            if game.nav.is_loaded() {
                queue_add_named_bot(game, &name, index);
            }
        }
        return;
    }
    let want = match bot_target(want, humans, game.team_match.config, in_warmup) {
        Some(w) => w,
        None => return, // structured match live — don't add or trim (would bench noise / drop a rostered bot)
    };

    // Build the navmesh on demand the first time bots are actually wanted.
    if want > 0 {
        game.ensure_navmesh();
        if !game.nav.is_loaded() {
            return;
        }
    }

    // Queue at most one population change per frame; `vmMain` applies it via `drain_roster` once
    // this frame's `&mut GameState` is released, so the trap's re-entrant client callbacks hold the
    // sole borrow instead of aliasing ours. `add_bot`/`remove_bot` must not be called here directly.
    if count < want {
        queue_add_bot(game, count);
    } else if count > want {
        if let Some(e) = last_bot {
            let client = game.entities[e].bot.client;
            game.pending_roster = Some(RosterOp::Remove { client, slot: e });
        }
    }
}

/// A population change [`manage_population`] wants applied, deferred to the `vmMain` boundary so the
/// re-entrant engine trap doesn't fire while a `&mut GameState` borrow is live. See
/// [`drain_roster`] and [`crate::game::GameState::pending_roster`].
pub(crate) enum RosterOp {
    /// Add a fake client with this name/colours (skin is always `"base"`).
    Add { name: CString, bottom: i32, top: i32 },
    /// Remove the fake client at edict `slot` (its engine client id is `client`).
    Remove { client: i32, slot: EntId },
}

/// Apply the queued [`RosterOp`], if any. `add_bot`/`remove_bot` run the module's
/// `ClientConnect`/`PutClientInServer`/`ClientDisconnect` synchronously and re-entrantly, so this
/// is called from `vmMain` with **no** `&mut GameState` borrow live: the re-entered `vmMain` then
/// holds the sole borrow (soundly nested) instead of aliasing one of ours.
///
/// # Safety
/// `game` must point at the live `GameState`, and no other reference into it may be live for the
/// duration of this call (guaranteed by calling it from `vmMain` after the frame's borrow drops).
/// The engine is single-threaded, so the re-entrant callbacks the trap fires are the only nested
/// access, and each borrow below is created briefly and dropped before the next trap.
pub(crate) unsafe fn drain_roster(game: *mut GameState) {
    // Take the op out (and copy the `Copy` host handle) under a short borrow that ends here — so a
    // re-entrant `vmMain` -> `drain_roster` during the trap finds `None`, and no borrow spans the
    // trap. Every `&mut *game` below is an explicit, scoped reborrow, never live across a trap.
    let (op, host) = {
        let g = &mut *game;
        (g.pending_roster.take(), g.host)
    };
    let Some(op) = op else {
        return;
    };
    match op {
        // `add_bot` sets the bot's name in userinfo and broadcasts it — don't re-set "name"
        // afterwards (that renamed the bot to an empty string and kept it off the scoreboard). Tag
        // the edict as bot-driven only after the trap returns.
        RosterOp::Add { name, bottom, top } => {
            let client = host.add_bot(&name, bottom, top, c"base");
            if client > 0 {
                let g = &mut *game;
                g.entities[EntId(client as u32)].bot = BotState {
                    is_bot: true,
                    client,
                    ..Default::default() // goal_cell None, route empty, etc. — a fresh blackboard
                };
            }
        }
        RosterOp::Remove { client, slot } => {
            host.remove_bot(client);
            let g = &mut *game;
            g.retire_slot(slot); // fully retire the slot (bot state + in_use/classname/arena)
        }
    }
}

/// How many bots to field this frame, given the raw `cvar_want`, the human count, the resolved
/// composition, and whether a team match is in warmup. Open play (and CTF pickup) passes `cvar_want`
/// through. A **structured** match caps the fill to the empty seats during warmup — so bots exactly
/// top up teams×size around the humans — and returns `None` (freeze: don't add or trim) once the
/// match is live, since a fresh bot would only be benched and a trim could drop a rostered one. Pure.
pub(super) fn bot_target(
    cvar_want: i32,
    humans: i32,
    cfg: crate::mode::team::MatchConfig,
    in_warmup: bool,
) -> Option<i32> {
    let structured = cfg.teams >= 2 && cfg.size >= 1;
    if !structured {
        return Some(cvar_want);
    }
    if !in_warmup {
        return None;
    }
    let seats = (cfg.teams * cfg.size) as i32;
    Some(cvar_want.min((seats - humans).max(0)))
}

/// Queue the add of one bot (name/colours by rotating `index`). The actual `add_bot` trap — which
/// re-enters the module — is fired later by [`drain_roster`] at the `vmMain` boundary, not here,
/// so it can't alias the population manager's `&mut GameState`.
fn queue_add_bot(game: &mut GameState, index: i32) {
    // Build the name from latin-1 bytes, not `cstring` (which is UTF-8): `bot_display_name` carries
    // high-half conchars, and the engine stores the `CString`'s bytes verbatim into the netname.
    let display = bot_display_name(bot_name(index));
    let name = CString::new(crate::text::latin1_bytes(&display)).unwrap_or_default();
    let (bottom, top) = bot_colors(index);
    game.pending_roster = Some(RosterOp::Add { name, bottom, top });
}

/// Recreate a fake client dropped by mvdsv's match-start map reload with its exact locked name, so
/// team assignment recognizes it as a roster member rather than benching a newly numbered bot.
fn queue_add_named_bot(game: &mut GameState, display: &str, index: i32) {
    let name = CString::new(crate::text::latin1_bytes(display)).unwrap_or_default();
    let (bottom, top) = bot_colors(index);
    game.pending_roster = Some(RosterOp::Add { name, bottom, top });
}

/// A bot's on-scoreboard name: a coloured `bot` tag, the coloured dot, then `label` in plain white
/// — e.g. `bot•Grunt`. Shared by both embodiments (the qwprogs roster and the netclient userinfo).
pub(crate) fn bot_display_name(label: &str) -> String {
    crate::text::Conchars::default()
        .coloured("bot")
        .ch(crate::text::DOT)
        .plain(label)
        .build()
}

/// A rotating set of bot names.
pub(crate) fn bot_name(index: i32) -> &'static str {
    const NAMES: [&str; 32] = [
        "Grunt",
        "Ranger",
        "Visor",
        "Sarge",
        "Bitterman",
        "Hossman",
        "Daemia",
        "Klesk",
        "Anarki",
        "Angel",
        "Biker",
        "Bones",
        "Cadaver",
        "Crash",
        "Doom",
        "Gorre",
        "Hunter",
        "Keel",
        "Lucy",
        "Major",
        "Mynx",
        "Orbb",
        "Patriot",
        "Phobos",
        "Razor",
        "Slash",
        "Sorlag",
        "Stripe",
        "TankJr",
        "Uriel",
        "Wrack",
        "Xaero",
    ];
    NAMES[(index as usize) % NAMES.len()]
}

/// Distinct shirt/pants colors per bot (QW palette 0–13).
fn bot_colors(index: i32) -> (i32, i32) {
    let c = [4, 11, 12, 13, 2, 6, 3, 10];
    (c[(index as usize) % c.len()], c[(index as usize * 3 + 1) % c.len()])
}
