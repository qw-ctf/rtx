// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh-driven bots. First deliverable (P4): multiple fake clients that **follow the
//! nearest human around the map** — full navigation over the auto-generated [navmesh], no
//! shooting. Goals and combat layer on later.
//!
//! A bot is a real client edict (`1..=maxclients`) spawned via `add_bot`; the engine runs its
//! [`BotState`]-driven usercmd through the same `SV_RunCmd`/`PM_PlayerMove` as a human, so
//! gravity, friction, stepping, and jumps come for free. Each server bot-frame
//! ([`GameState::start_frame`] with `is_bot_frame`) we recompute intent and emit one
//! `set_bot_cmd` per bot — the engine zeroes the cmd after running it, so it must be re-sent
//! every frame.
//!
//! [navmesh]: crate::navmesh

use glam::{Vec3, Vec3Swizzles};

pub(crate) mod bhop;
mod combat;
pub(crate) mod goals;
mod grenade;
mod hook;
pub(crate) mod state;

use crate::bot::state::{BotState, GrenadePhase, HookPhase};
use crate::defs::{
    Bits, Flags, Items, Solid, Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK, BUTTON_JUMP,
};
use crate::entity::{EntId, Entity, Touch};
use crate::game::{cstring, GameState};
use crate::mode::BotIntent;
use crate::navmesh::{CellId, LinkKind, NavGraph};

/// Impulse to select the shotgun (for shooting a health-gated button).
const IMPULSE_SHOTGUN: i32 = 2;
/// Impulse to select the grappling hook (for flying a hook leg).
const IMPULSE_GRAPPLE: i32 = 22;

// --- grappling-hook leg execution ---

/// Within this of the hook leg's source cell counts as "in stance" to throw from.
const HOOK_STANCE: f32 = 24.0;
/// Throw once the smoothed view is within this many degrees of the anchor.
const HOOK_AIM_TOL: f32 = 2.0;
/// Give up aiming/throwing if the hook hasn't bitten within these windows.
const HOOK_AIM_TIMEOUT: f32 = 1.5;
const HOOK_FLIGHT_TIMEOUT: f32 = 1.0;
/// Abort a reel that runs this long (snagged, or a moving/none anchor) without releasing.
const HOOK_REEL_TIMEOUT: f32 = 3.0;
/// Abort if the live anchor sits this far from where the build expected it (hooked a player, a
/// moving door, or a sky brush the runtime rejected) — the solved arc no longer applies.
const HOOK_ANCHOR_DRIFT: f32 = 48.0;
/// Ballistic watchdog slack added to the solved airtime before we give up waiting to land.
const HOOK_BALLISTIC_SLACK: f32 = 1.0;

/// Advance to the next route leg once within this of the current waypoint (≈ ¾ of a grid).
const ARRIVE_RADIUS: f32 = 24.0;
/// Stop closing once this near the followed human, so bots tail rather than shove.
const POLITE_DIST: f32 = 64.0;
/// Minimum seconds between A* re-paths (the human keeps moving).
const REPATH_INTERVAL: f32 = 0.4;
/// Stuck detector: if we move less than this over `STUCK_TIME`, jump and re-path.
const STUCK_MOVE: f32 = 16.0;
const STUCK_TIME: f32 = 0.7;
/// A plat ride is "done" once we've risen to within this of the exit-floor height.
const PLAT_RISE_TOL: f32 = 18.0;
/// Minimum seconds between item-goal re-selections (so a bot commits to a pickup rather than
/// flip-flopping between two of similar worth each frame).
const GOAL_SELECT_INTERVAL: f32 = 1.5;
/// How long after the last line-of-sight frame the nav view may still point at a Fight enemy's
/// live origin. Set equal to combat's `HOLD_ANGLE_TIME` (the 2s corner-hold in `combat::engage`):
/// while that hold owns the view it overrides `look` anyway, so the handoff is seamless — hold the
/// corner for 2s, then look where we're travelling instead of tracking the enemy through geometry.
const LOOK_LOS_GRACE: f32 = 2.0;
/// Goal watchdog: if a bot has been chasing the *same* item this long (and isn't detouring to open
/// a gate) without ever collecting it, that item is effectively unreachable for it — abandon it and
/// avoid it for `GOAL_AVOID_TIME`, then retry. Time-based (not distance) so a legitimate route that
/// detours away — riding an elevator, walking to a teleporter — isn't mistaken for being stuck.
const GOAL_GIVEUP_TIME: f32 = 10.0;
const GOAL_AVOID_TIME: f32 = 12.0;
/// Gate errand give-up: if a bot goes this long without getting any closer to a gate's button, the
/// button is out of reach (or unusable) — abandon and avoid the gate for `GATE_AVOID_TIME`. Keyed
/// on lack of progress, not elapsed time, so a button that's just far away is still pursued.
const GATE_GIVEUP_TIME: f32 = 4.0;
const GATE_AVOID_TIME: f32 = 6.0;

/// One bot's accumulated frame command: what navigation proposes and the combat/grenade overlays
/// mutate in turn, before the aim spring and view projection in `run_bot` turn it into the final
/// `set_bot_cmd`. `look` (desired view angles, pre-spring) and `move_world` (desired world-space
/// velocity) are deliberately decoupled — the bot can run one way while looking another.
pub(crate) struct BotCmd {
    pub look: Vec3,
    pub move_world: Vec3,
    pub buttons: i32,
    pub impulse: i32,
}

// --- population management (P3) ---

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
        } else if ent.in_use && ent.classname() == Some("player") {
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

    // Build the navmesh on demand the first time bots are actually wanted.
    if want > 0 {
        game.ensure_navmesh();
        if !game.nav.is_loaded() {
            return;
        }
    }

    if count < want {
        add_one_bot(game, count);
    } else if count > want {
        if let Some(e) = last_bot {
            host.remove_bot(game.entities[e].bot.client);
            game.retire_slot(e); // fully retire the slot (bot state + in_use/classname/arena)
        }
    }
}

/// Spawn one bot. `add_bot` runs the module's ClientConnect + PutClientInServer for the new
/// edict synchronously, then we tag that edict as bot-driven. No-op if the server is full.
fn add_one_bot(game: &mut GameState, index: i32) {
    let host = *game.host();
    let name = cstring(&format!("[rtx]{}", bot_name(index)));
    let (bottom, top) = bot_colors(index);
    // `add_bot` already sets the bot's name in its userinfo and broadcasts it (the
    // "[rtx]Grunt entered the game" line) — don't re-set "name" afterwards: doing so renamed the
    // bot to an empty string and is what kept bots off the scoreboard.
    let client = host.add_bot(&name, bottom, top, c"base");
    if client > 0 {
        game.entities[EntId(client as u32)].bot = BotState {
            is_bot: true,
            client,
            goal_cell: u32::MAX,
            ..Default::default()
        };
    }
}

/// A rotating set of bot names.
fn bot_name(index: i32) -> &'static str {
    const NAMES: [&str; 8] = [
        "Grunt",
        "Ranger",
        "Visor",
        "Sarge",
        "Bitterman",
        "Hossman",
        "Daemia",
        "Klesk",
    ];
    NAMES[(index as usize) % NAMES.len()]
}

/// Distinct shirt/pants colors per bot (QW palette 0–13).
fn bot_colors(index: i32) -> (i32, i32) {
    let c = [4, 11, 12, 13, 2, 6, 3, 10];
    (c[(index as usize) % c.len()], c[(index as usize * 3 + 1) % c.len()])
}

// --- per-frame driving (P2 follow + P4 behavior) ---

/// Drive every bot for this server bot-frame: pick the nearest human, path to them, and emit
/// each bot's usercmd. Skipped entirely without a navmesh.
pub fn run_bots(game: &mut GameState) {
    if !game.nav.is_loaded() {
        return;
    }
    let maxclients = game.host().cvar(c"maxclients") as i32;
    for i in 1..=maxclients as u32 {
        let e = EntId(i);
        if game.entities[e].bot.is_bot && game.entities[e].in_use {
            bot_pickup_items(game, e);
            run_bot(game, e);
        }
    }
}

/// Generous fixed pickup half-extents (the player hull ±16 plus the item trigger ±16, with a
/// little slack). We use a fixed box rather than the item's engine-side `mins`/`maxs` because
/// `set_size` may not sync those to our shadow `EntVars`, which would make the test a degenerate
/// point and almost never fire.
const PICKUP_XY: f32 = 40.0;
const PICKUP_Z: f32 = 48.0;

/// Manually collect any item the bot is standing on. The engine doesn't run the trigger-touch
/// phase for `SetBotCMD` fake clients the way it does for human `SV_RunCmd`, so a bot would walk
/// onto a pickup and never actually take it — it'd just keep wanting it and circle. We replicate
/// the touch here, guarded by `solid == Trigger` (a respawning item that's already been taken is
/// non-solid → skipped) so this can't double-grant even if an engine *does* fire the touch.
fn bot_pickup_items(game: &mut GameState, e: EntId) {
    if game.entities[e].v.health <= 0.0 || game.entities[e].v.deadflag != 0.0 {
        return;
    }
    let origin = game.entities[e].v.origin;
    // Gather first (immutable borrow of nav/entities), then fire touches (needs `&mut game`).
    // Catalog items (weapons/ammo/health/armor/powerups)...
    let mut hits: Vec<EntId> = game
        .nav
        .goals
        .iter()
        .filter_map(|&(idx, _)| {
            let item = EntId(idx);
            let it = &game.entities[item];
            (it.v.solid == Solid::Trigger && on_item(origin, it.v.origin)).then_some(item)
        })
        .collect();
    // ...plus any dropped backpack we're standing on. Backpacks aren't in the static catalog (they
    // spawn on death / a teammate's toss), so without this a bot would walk over one and never take
    // it — the engine skips the trigger-touch phase for `SetBotCMD` fake clients.
    hits.extend(game.entities.iter().enumerate().filter_map(|(i, it)| {
        (it.touch == Touch::Backpack && it.v.solid == Solid::Trigger && on_item(origin, it.v.origin))
            .then_some(EntId(i as u32))
    }));
    for item in hits {
        if game.entities[item].v.solid == Solid::Trigger {
            game.run_touch(item, e);
        }
    }
}

/// Whether a bot at `bot_origin` is close enough to an item at `item_origin` to collect it.
fn on_item(bot_origin: Vec3, item_origin: Vec3) -> bool {
    let d = item_origin - bot_origin;
    d.x.abs() <= PICKUP_XY && d.y.abs() <= PICKUP_XY && d.z.abs() <= PICKUP_Z
}

fn run_bot(game: &mut GameState, e: EntId) {
    let host = *game.host();
    let now = game.time();
    let frametime = game.globals.frametime;
    let msec = ((frametime * 1000.0) as i32).clamp(1, 100);

    let origin = game.entities[e].v.origin;
    let v_angle = game.entities[e].v.v_angle;
    let client = game.entities[e].bot.client;
    let weapon = game.entities[e].v.weapon;
    let on_ground = game.entities[e].v.flags.has(Flags::ONGROUND);
    let alive = game.entities[e].v.health > 0.0 && game.entities[e].v.deadflag == 0.0;
    // Vertical speed and whether the once-per-air-travel double jump is still available — snapshot
    // now, since the `&mut bot` binding below blocks reading the edict during the move logic.
    let vz = game.entities[e].v.velocity.z;
    let air_jumped = game.entities[e].combat.air_jumped;
    // When combat last had line of sight (see `combat::engage`) — snapshot for the bhop veto.
    let enemy_seen_time = game.entities[e].bot.enemy_seen_time;
    let v_xy = game.entities[e].v.velocity.xy();
    let speed = v_xy.length();
    // Snapshot grapple state up front (it's set in the previous frame's PlayerPreThink, so it's
    // stable across this bot frame) — lets the hook driver read it without re-borrowing the edict
    // while the `&mut bot` binding is live. `anchor` is meaningful only once `on_hook`.
    let grapple_hook = EntId(game.entities[e].grapple.hook);
    let has_grapple = game.entities[e].v.items.has(Items::GRAPPLE);
    let hook_out = game.entities[e].grapple.hook_out;
    let on_hook = game.entities[e].grapple.on_hook;
    let anchor = if hook_out {
        game.entities[grapple_hook].v.origin
    } else {
        Vec3::ZERO
    };
    // Live reel speed, for the release-crossing prediction (half a frame of lookahead).
    let reel_half_step = crate::navmesh::HOOK_PULL_BASE * host.cvar(c"rtx_hook_pull") * game.globals.frametime * 0.5;
    // Flip the per-frame pulse used for press/release-edge buttons.
    let pulse = {
        let b = &mut game.entities[e].bot;
        b.pulse = !b.pulse;
        b.pulse
    };

    // Connected but never spawned (health 0, not dead): the engine defers `PutClientInServer` — the
    // full spawn that sets health/loadout — to the bot's spawn on a *bot frame*, which an empty
    // (bots-only) server never runs. So the bot sits at 0 health forever, and the respawn pulse below
    // can't help it (`death_think` only runs for `deadflag >= Dead`). Spawn it ourselves; next frame
    // it's alive and plays normally.
    if !alive && game.entities[e].v.deadflag == 0.0 {
        game.put_client_in_server(e);
        return;
    }
    // Genuinely dead (fragged): pulse +attack to respawn. rtx's death-think needs all buttons
    // *released* (Dead → Respawnable) and then *pressed* again — so the button must be pulsed.
    if !alive {
        let buttons = if pulse { BUTTON_ATTACK } else { 0 };
        host.set_bot_cmd(client, msec, v_angle, 0, 0, 0, buttons, 0);
        return;
    }

    let idle = |angles: Vec3| host.set_bot_cmd(client, msec, angles, 0, 0, 0, 0, 0);

    // Hook invariant net: if we're mid-hook but no longer hold the grapple (a mode loadout stripped
    // it, e.g. Rocket Arena), abandon the traversal cleanly — release any live hook and reset the
    // phase. Runs before the nav borrow, where `&mut game` is free. Other aborts (leg changed, hook
    // vanished, timeouts) are handled inside the hook driver below.
    if game.entities[e].bot.hook_phase != HookPhase::Idle && !game.entities[e].v.items.has(Items::GRAPPLE) {
        if game.entities[e].grapple.hook_out {
            let hook = EntId(game.entities[e].grapple.hook);
            game.reset_grapple(hook);
        }
        game.entities[e].bot.hook_phase = HookPhase::Idle;
        game.entities[e].bot.hook_fails = 0;
    }
    let hooking = game.entities[e].bot.hook_phase != HookPhase::Idle;
    // On a speed-jump leg the route must be frozen: the link's `from` is the runway start, now behind
    // the bot, so a repath would drop the link and turn the bot around at speed. Treated like `hooking`.
    let on_sj = game.entities[e].bot.sj_leg.is_some();

    // Ask the active mode for this bot's intent. A round mode (Rocket Arena) returns Fight/Move to
    // drive combat or audience-roaming; FFA hunts the nearest player. Every mode-specific bot
    // adaptation lives behind this one hook — the rest of run_bot stays mode-agnostic and reusable.
    let mode = game.mode;
    let intent = if host.cvar_bool(c"rtx_bot_pacifist") {
        // Global override, any mode: don't fight — just tail the nearest human around the map.
        nearest_human(game, e).map(|h| BotIntent::Move(game.entities[h].v.origin))
    } else {
        mode.bot_intent(game, e)
    };
    // Item goal (P5): re-pick the best reachable pickup on a slow cadence, and drop a chosen item
    // once it's been grabbed (no longer available/respawning soon) so the bot moves on.
    //
    // Three cases feed the same `goal_item` slot:
    //  - No mode intent → the full item brain (idle pickup, or follow-a-human fallback below).
    //  - A **Fight** intent with `rtx_bot_greed` → a *combat detour*: still pick a compelling item
    //    (`select_combat_item` bars trivial ones) so the bot can break off to grab the quad / a
    //    needed weapon / big health, ktx-style, without abandoning combat — `enemy` stays set, so
    //    the combat overlay keeps aiming and firing whenever it has line of sight (see below).
    //  - Any other mode intent (a **Move** objective, or Fight with greed off) → no item chase.
    let greedy = matches!(intent, Some(BotIntent::Fight(_))) && host.cvar_bool(c"rtx_bot_greed");
    if intent.is_none() || greedy {
        if now >= game.entities[e].bot.goal_select_time {
            let pick = if greedy {
                game.select_combat_item(e)
            } else {
                game.select_item_goal(e)
            };
            let (new_item, new_cell) = pick.map_or((0, 0), |(it, c)| (it.0, c));
            let b = &mut game.entities[e].bot;
            if new_item != b.goal_item {
                b.goal_started = now; // restart the watchdog for a new goal
            }
            (b.goal_item, b.goal_item_cell) = (new_item, new_cell);
            b.goal_select_time = now + GOAL_SELECT_INTERVAL;
        }
        if game.entities[e].bot.goal_item != 0 && !game.item_goal_valid(e, EntId(game.entities[e].bot.goal_item), now) {
            let b = &mut game.entities[e].bot;
            b.goal_item = 0;
            b.goal_select_time = now; // re-pick next frame
        }
    } else {
        game.entities[e].bot.goal_item = 0; // a Move objective supersedes any item chase
    }

    // Opt-in diagnostics (`rtx_bot_debug 1`): one throttled line per bot — what it wants, how far,
    // whether it's standing on that item, and whether it owns the LG. Pinpoints pickup-vs-desire.
    if host.cvar_bool(c"rtx_bot_debug") && now >= game.entities[e].bot.repath_time {
        let gi = game.entities[e].bot.goal_item;
        let (goal, dist, overlap) = if gi != 0 {
            let it = &game.entities[EntId(gi)];
            let on = it.v.solid == Solid::Trigger && on_item(origin, it.v.origin);
            let name = it
                .classname()
                .unwrap_or(if it.touch == Touch::Backpack { "backpack" } else { "?" });
            (name, (it.v.origin - origin).length(), on)
        } else {
            ("human", 0.0, false)
        };
        let own_lg = game.entities[e].v.items.has(Items::LIGHTNING) as i32;
        let msg = cstring(&format!(
            "rtx bot{client}: want={goal} dist={dist:.0} on_item={overlap} ownLG={own_lg} cells={:.0}\n",
            game.entities[e].v.ammo_cells,
        ));
        host.conprint(&msg); // conprint always shows; dprint needs `developer 1`
    }

    // The mode's intent (fight an enemy / roam to a spot), if any, and whether we're on an item.
    let enemy = if let Some(BotIntent::Fight(en)) = intent {
        Some(en)
    } else {
        None
    };
    // A committed item goal drives *movement* — set by the idle brain, or by the greedy combat
    // detour (Fight + `rtx_bot_greed`). Under a Fight intent the enemy is still tracked above, so
    // the combat overlay keeps aiming/firing on line of sight (and its range-keeping owns movement
    // then); the detour only steers navigation while the enemy is *out* of sight, when navigation
    // would otherwise just beeline the enemy. This is ktx's "the enemy is one more goal" in effect.
    let chasing = game.entities[e].bot.goal_item != 0;
    let goal_item_org = {
        let it = EntId(game.entities[e].bot.goal_item);
        (game.entities[it].v.origin, Some(game.entities[e].bot.goal_item_cell))
    };
    // Where we're headed: the detour item, the mode's target, the chosen item, or the nearest human.
    let (target_origin, item_cell) = match intent {
        Some(BotIntent::Fight(_)) if chasing => goal_item_org,
        Some(BotIntent::Fight(en)) => (game.entities[en].v.origin, None),
        Some(BotIntent::Move(pos)) => (pos, None),
        None if chasing => goal_item_org,
        None => {
            if let Some(h) = nearest_human(game, e) {
                (game.entities[h].v.origin, None)
            } else {
                // Nothing to chase and no human to follow: **roam** to a random reachable spot
                // instead of freezing, so a human-less server (e.g. bots-only FFA) keeps its bots
                // moving — and finding items and each other — rather than standing on spawn.
                (roam_target(game, e, origin, now), None)
            }
        }
    };

    // Goal watchdog: while chasing an item and *not* already detouring to open a gate, give up on
    // one we've chased too long without collecting (behind an elevator/button/movewall/teleporter
    // chain the router can't thread) so we stop circling and go fetch something reachable instead.
    if chasing && game.entities[e].bot.gate.is_none() && now - game.entities[e].bot.goal_started > GOAL_GIVEUP_TIME {
        let b = &mut game.entities[e].bot;
        b.avoid_item = b.goal_item;
        b.avoid_until = now + GOAL_AVOID_TIME;
        b.goal_item = 0;
        b.goal_select_time = now; // re-pick (skipping the abandoned item) next frame
    }

    // Current door states, for gate-aware pathfinding. A shut gate makes its links expensive, so
    // `find_path` bends the route around a closed door when any open way exists and only crosses
    // one (leaving the bot to detour to the button) when there's no alternative. Computed before
    // the nav borrow (it reads the obstruction edicts).
    let gate_closed = game.gate_closed_flags();

    // Graph queries (borrows game.nav) and bot-state updates (borrows game.entities) are on
    // disjoint fields, so they coexist; `host` is a Copy, no game borrow held across the send.
    let graph = game.nav.graph.as_ref().unwrap();
    let Some(bot_cell) = graph.nearest(origin) else {
        idle(v_angle);
        return;
    };
    let Some(goal_cell) = item_cell.or_else(|| graph.nearest(target_origin)) else {
        idle(v_angle);
        return;
    };

    // Whether each gate's activator can be triggered right now: a shoot activator is "ready" only
    // while it takes damage — re-triggerable triggers go dead during their cooldown.
    let gate_ready: Vec<bool> = (0..graph.gate_count())
        .map(|gi| {
            let g = graph.gate(gi);
            !g.shoot || game.entities[EntId(g.activator)].v.takedamage != 0.0
        })
        .collect();

    let bot = &mut game.entities[e].bot;

    // A teleport (or any large instant displacement) invalidates the planned route — drop it
    // and re-path from where we landed. ~200u in one frame is far beyond running/falling. Skipped
    // mid-hook: the reel and the parabola move fast on purpose and must not clear the hook route.
    if !hooking && !on_sj && bot.last_origin != Vec3::ZERO && (origin - bot.last_origin).length() > 200.0 {
        bot.route.clear();
        bot.repath_time = now;
    }
    bot.last_origin = origin;

    // Gate errand: drop it once the gate's door has opened — or give up if we stop making progress
    // toward its button (stuck at a door whose button we can't actually reach), so we don't camp
    // there. Progress-based, not a flat timeout: a button that's simply far across the map (e.g.
    // when we spawned right next to the door) still gets reached. Suspended mid-hook.
    if !hooking && !on_sj && bot.gate.is_some() {
        let gi = bot.gate.unwrap();
        let give_up = |bot: &mut BotState| {
            bot.avoid_gate = gi as i32;
            bot.avoid_gate_until = now + GATE_AVOID_TIME;
            bot.gate = None;
            bot.route.clear();
            bot.repath_time = now;
        };
        if gate_closed.get(gi).copied() != Some(true) {
            bot.gate = None; // door opened — done
            bot.route.clear();
            bot.repath_time = now;
        } else if !button_reachable(graph, bot_cell, gi, &gate_closed) {
            give_up(bot); // button is walled off behind this very gate — route around instead
        } else {
            let d = (graph.cell_origin(graph.gate(gi).button_cell).xy() - origin.xy()).length();
            if d < bot.gate_best_dist - 4.0 {
                bot.gate_best_dist = d; // got closer — reset the give-up clock
                bot.gate_since = now;
            } else if now - bot.gate_since > GATE_GIVEUP_TIME {
                give_up(bot); // no progress toward a reachable button — stuck; try elsewhere
            }
        }
    }

    // Effective goal: the human, or — while opening a gate — that gate's button.
    let goal = match bot.gate {
        Some(gi) => graph.gate(gi).button_cell,
        None => goal_cell,
    };

    // Re-path when the route is empty, the goal changed, or the timer elapsed. Frozen mid-hook so
    // the traversal keeps the route that put it on the hook leg.
    if !hooking && !on_sj && (bot.route.is_empty() || bot.goal_cell != goal || now >= bot.repath_time) {
        let mut route = graph.find_path(bot_cell, goal, &gate_closed).unwrap_or_default();
        // Goal unreachable from here (behind a shut door with no way around from this spot, or a
        // disconnected pocket)? Don't home straight into a wall — head to the reachable cell
        // nearest the goal, approaching as far as the graph allows (often enough for line of sight
        // or to find a connection). Better than freezing until the target wanders into view.
        if route.is_empty() && bot_cell != goal {
            if let Some(near) = graph.nearest_reachable_to(bot_cell, goal, &gate_closed) {
                route = graph.find_path(bot_cell, near, &gate_closed).unwrap_or_default();
            }
        }
        bot.route = route;
        bot.route_pos = 0;
        bot.goal_cell = goal;
        bot.repath_time = now + REPATH_INTERVAL;
    }
    // If we've fallen off the planned route (missed a jump, got shoved), re-localize next.
    if !hooking && !on_sj && bot.route_pos >= bot.route.len() && bot_cell != goal && now >= bot.repath_time {
        bot.repath_time = now; // force a fresh path next frame
    }

    // Not on an errand yet? `find_path` already routes *around* a shut gate when it can (its links
    // are priced high), so if the chosen route still crosses one, there's no other way in — divert
    // to that gate's button. Skip a gate we recently gave up on (its button was unreachable) so we
    // don't immediately re-camp on it.
    if !hooking && !on_sj && bot.gate.is_none() {
        let avoid = if now < bot.avoid_gate_until { bot.avoid_gate } else { -1 };
        let block =
            route_blocking_gate(graph, &bot.route, bot.route_pos, &gate_closed).filter(|&gi| gi as i32 != avoid);
        if let Some(gi) = block {
            if button_reachable(graph, bot_cell, gi, &gate_closed) {
                let button_cell = graph.gate(gi).button_cell;
                bot.gate = Some(gi);
                bot.gate_since = now;
                bot.gate_best_dist = f32::INFINITY; // first frame records the starting distance
                bot.route = graph.find_path(bot_cell, button_cell, &gate_closed).unwrap_or_default();
                bot.route_pos = 0;
                bot.goal_cell = button_cell;
                bot.repath_time = now + REPATH_INTERVAL;
            } else {
                // Button is walled off behind this gate — don't chase it; avoid the gate so
                // route_blocking_gate stops re-selecting it and find_path routes around the pillar.
                bot.avoid_gate = gi as i32;
                bot.avoid_gate_until = now + GATE_AVOID_TIME;
            }
        }
    }

    // Advance past route legs we've already reached. A plat leg completes when we've *risen*
    // to the exit height (Z), not on XY arrival — we're standing still on the lift while it
    // carries us up, so XY barely changes.
    // A bunnyhopping bot covers ground fast enough to orbit a 24u waypoint, so widen the arrival gate
    // with speed and also advance once a waypoint slips *behind* the velocity.
    let arrive_r = if bot.bhop.phase != bhop::Phase::Off || bot.sj_leg.is_some() {
        ARRIVE_RADIUS.max(2.0 * speed * frametime)
    } else {
        ARRIVE_RADIUS
    };
    while bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let target = graph.cell_origin(graph.link_target(leg));
        let arrived = match graph.link_kind(leg) {
            LinkKind::Plat => origin.z >= target.z - PLAT_RISE_TOL,
            // A hook leg never auto-advances on XY: a near-vertical pull-up passes the XY test while
            // still at the *bottom* of the swing. The hook driver advances it only once the parabola
            // has landed (see below).
            LinkKind::Hook => false,
            _ => {
                let to = target.xy() - origin.xy();
                let fast = bot.bhop.phase != bhop::Phase::Off || bot.sj_leg.is_some();
                to.length() <= arrive_r || (fast && to.dot(v_xy) < 0.0 && to.length() <= 64.0)
            }
        };
        if arrived {
            bot.route_pos += 1;
        } else {
            break;
        }
    }

    // Current waypoint + how to traverse to it. Past the route's end, home straight in on the
    // human (final approach). While riding a plat, steer toward the plat *centre* (the leg's
    // source cell) to stay aboard as it rises, instead of toward the far exit ledge.
    let (waypoint, kind, final_leg, cur_leg) = if bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let k = graph.link_kind(leg);
        let aim = if k == LinkKind::Plat {
            graph.cell_origin(graph.link_source(leg))
        } else {
            graph.cell_origin(graph.link_target(leg))
        };
        (aim, Some(k), false, Some(leg))
    } else {
        (target_origin, None, true, None)
    };
    let hook_active = matches!(kind, Some(LinkKind::Hook)) || hooking;
    // Where the *eyes* go while navigating: a couple of legs ahead of the feet (or the final
    // target when the route is short), so the view sweeps down the corridor instead of snapping
    // to every 32u grid cell the bot steps through. Steering still uses `waypoint`.
    //
    // But a Fight target we're *not* detouring on sets `target_origin` to the enemy's LIVE origin,
    // so aiming the eyes there while we can't see the enemy tracks it through walls — an aimbot
    // look. Once combat's 2s corner-hold lapses (or if we never saw them), look where we're
    // *travelling* instead. Non-combat targets — a human we follow, a committed item goal, or a
    // greedy detour (`chasing`) — are exactly where we want to look, so they keep `target_origin`.
    let combat_blind =
        enemy.is_some() && !chasing && (enemy_seen_time <= 0.0 || now - enemy_seen_time > LOOK_LOS_GRACE);
    let look_point = if bot.route_pos + 2 < bot.route.len() {
        graph.cell_origin(graph.link_target(bot.route[bot.route_pos + 2]))
    } else if combat_blind {
        // Past the route's end `waypoint` *is* `target_origin` (the enemy), so there fall through
        // to our actual travel heading rather than re-pointing the eyes at the hidden enemy.
        if final_leg && speed > 20.0 {
            origin + Vec3::new(v_xy.x, v_xy.y, 0.0)
        } else {
            waypoint
        }
    } else {
        target_origin
    };

    // Stuck detection. Suppressed mid-hook: standing in the throw stance, reeling, and riding the
    // parabola all look "stuck" to it, and a force-jump/repath there would wreck the traversal — the
    // hook driver's own per-phase timeouts are its stuck detection.
    let mut force_jump = false;
    if hook_active || (origin - bot.stuck_origin).length() > STUCK_MOVE {
        bot.stuck_origin = origin;
        bot.stuck_since = now;
    } else if now - bot.stuck_since > STUCK_TIME {
        force_jump = true;
        bot.repath_time = now; // re-path next frame
        bot.stuck_since = now;
    }

    // Bunnyhop policy verdicts — everything that needs game state is judged here; *when* each
    // verdict may apply in the hop cycle (engage hysteresis, mid-hop commitment, landing-only
    // disengage) is `bhop::Bhop::step`'s job. The entry runway bar is deliberately fixed:
    // the old `speed·0.9` bar rose as the bot gained speed and cut runs short mid-air.
    let goal_dist = (target_origin.xy() - origin.xy()).length();
    let runway_dist = runway(graph, &bot.route, bot.route_pos, origin);
    // Combat only vetoes bhop while it *owns the view* — the enemy is in sight (or lost a moment
    // ago), when the eyes must aim, not sweep a strafe. A mere Fight target being chased across
    // the map is navigation, and navigation bunnyhops; in FFA every bot always has a target, so
    // gating on target existence kept bhop permanently off. The grace here is deliberately much
    // shorter than combat's 2s corner-hold: on a small open FFA map sight contact is frequent,
    // and a long window suppresses hopping almost everywhere.
    const BHOP_COMBAT_GRACE: f32 = 0.5;
    let combat_view = enemy.is_some() && enemy_seen_time > 0.0 && now - enemy_seen_time < BHOP_COMBAT_GRACE;
    let bhop_veto = !host.cvar_bool(c"rtx_bot_bhop")
        || combat_view
        || hook_active
        || bot.gate.is_some()
        || bot.grenade_phase != GrenadePhase::Idle;
    let bhop_entry = !final_leg
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && goal_dist > 300.0
        && runway_dist >= bhop::RUNWAY_ENGAGE;
    // Lenient continuation gate for taking *another* hop from a landing: leg kinds churn as the
    // route advances, and a run in progress shouldn't be dumped by the stricter entry conditions.
    let bhop_sustain = matches!(kind, Some(LinkKind::Walk | LinkKind::Step)) && goal_dist > 150.0;
    // A speed-jump leg is a *committed* bhop run-up + leap: engage bhop unconditionally (the link is a
    // pre-verified runway) and track it so the route stays frozen. Latch/clear `sj_leg` on the leg.
    let mut sj_active = matches!(kind, Some(LinkKind::SpeedJump)) && host.cvar_bool(c"rtx_bot_bhop") && !hook_active;
    if sj_active {
        if bot.sj_leg != cur_leg {
            bot.sj_leg = cur_leg;
            bot.sj_started = now;
        }
        // Watchdog: the route is frozen mid-leg, so if the run-up stalls (blocked, shoved, never
        // built speed) abandon it and re-path rather than wedging on the runway forever.
        if now - bot.sj_started > 4.0 {
            bot.sj_leg = None;
            bot.route.clear();
            bot.repath_time = now;
            sj_active = false;
        }
    } else if bot.sj_leg.is_some() {
        bot.sj_leg = None;
    }
    // "Don't leap to your death": if we somehow reach the takeoff edge too slow to clear the gap,
    // hold the jump (keep accelerating) rather than launching short into it.
    let sj_takeoff = cur_leg
        .and_then(|l| graph.speed_jump_of_link(l))
        .map(|tr| (tr.takeoff, tr.v_req));
    let sj_hold = sj_active && {
        match sj_takeoff {
            Some((takeoff, v_req)) => {
                let to_edge = takeoff.xy() - origin.xy();
                (to_edge.length() < 48.0 || to_edge.dot(v_xy) < 0.0) && speed < v_req * 0.9
            }
            None => false,
        }
    };

    // Drive the hop-cycle controller (see `bhop::Bhop`). On a speed jump the runway is the
    // run-up to the takeoff edge and the bearing aims straight at the landing so the leap goes
    // across the gap; otherwise steer toward the look-ahead corridor point (smoother than the 32u
    // next cell) with as much straight-ish corridor as the route offers.
    let bhop_cmd = {
        let dt = frametime.clamp(0.001, 0.05);
        let accel = host.cvar(c"sv_accelerate");
        let maxspeed = host.cvar(c"sv_maxspeed");
        let env = bhop::Env {
            dt,
            accel: if accel > 0.0 { accel } else { 10.0 },
            maxspeed: if maxspeed > 0.0 { maxspeed } else { 320.0 },
        };
        let ahead = if sj_active { waypoint.xy() } else { look_point.xy() } - origin.xy();
        let to_wp = waypoint.xy() - origin.xy();
        let dir = if ahead.length() > 8.0 { ahead } else { to_wp };
        let bearing = dir.y.atan2(dir.x).to_degrees();
        let bhop_runway = match sj_takeoff {
            Some((takeoff, _)) if sj_active => (takeoff.xy() - origin.xy()).length(),
            _ => runway_dist,
        };
        let phase_was = bot.bhop.phase;
        let cmd = bot.bhop.step(
            &bhop::Input {
                v_xy,
                on_ground,
                bearing,
                runway: bhop_runway,
                eligible: bhop_entry,
                sustain: bhop_sustain,
                veto: bhop_veto,
                committed: sj_active,
                hold_jump: sj_hold,
                now,
            },
            &env,
        );
        // A phase transition is the interesting diagnostic moment — why a run started or ended.
        if bot.bhop.phase != phase_was && host.cvar_bool(c"rtx_bot_debug") {
            let why = if bot.bhop.phase == bhop::Phase::Off {
                format!(" ({})", bot.bhop.off_reason)
            } else {
                String::new()
            };
            host.conprint(&cstring(&format!(
                "rtx bot{client}: bhop {phase_was:?}->{:?}{why} spd={speed:.0} runway={bhop_runway:.0}\n",
                bot.bhop.phase,
            )));
        }
        cmd
    };
    let bhop_active = bhop_cmd.is_some();

    // Steering: face the waypoint and run toward it.
    let to_wp = waypoint.xy() - origin.xy();
    let dist = to_wp.length();
    let yaw = to_wp.y.atan2(to_wp.x).to_degrees();
    let mut angles = Vec3::new(0.0, yaw, 0.0);

    // Nav look target: eyes on the look-ahead point down the corridor (combat/gate may override
    // below). Falls back to the steering yaw when the look point is on top of us.
    let eye = origin + Vec3::new(0.0, 0.0, 22.0);
    let to_look = look_point - eye;
    let mut look = if to_look.xy().length() > 8.0 {
        Vec3::new(
            -to_look.z.atan2(to_look.xy().length()).to_degrees(),
            to_look.y.atan2(to_look.x).to_degrees(),
            0.0,
        )
    } else {
        angles
    };

    // Grappling-hook leg driver: fly a LinkKind::Hook leg (select the grapple, settle the view on
    // the anchor, throw, reel to build speed, release into a parabola onto the target ledge). Its
    // whole state machine lives in `hook::drive_hook`; here we just feed it the frame snapshot and
    // apply the HookDrive it returns. The deferred `reset` (needs `&mut game`) is flushed later.
    let hook = hook::drive_hook(
        graph,
        bot,
        hook::HookCtx {
            hook_active,
            cur_leg,
            enemy,
            hook_out,
            on_hook,
            grapple_hook,
            has_grapple,
            now,
            weapon,
            origin,
            on_ground,
            anchor,
            reel_half_step,
            chasing,
        },
    );
    // Whether the hook is actively steering this frame (survives the abort branches above).
    let hook_engaged = bot.hook_phase != HookPhase::Idle;
    let hook_lock = matches!(
        bot.hook_phase,
        HookPhase::Flight | HookPhase::Reel | HookPhase::Ballistic
    );
    if let Some(t) = hook.look_target {
        let d = t - eye;
        if d.xy().length() > 1.0 {
            look = Vec3::new(
                -d.z.atan2(d.xy().length()).to_degrees(),
                d.y.atan2(d.x).to_degrees(),
                0.0,
            );
        }
    }

    let (mut forward, mut side, mut buttons, mut impulse) = (0, 0, 0, 0);
    // Politely stop short only when tailing a human; when fetching an item, walk right onto it so
    // the pickup's touch fires — and when hunting an enemy, never stop short (otherwise the bot
    // halts 64u away and just stands there, e.g. right at a door between it and its target; the
    // combat layer manages the actual fighting distance once it has line of sight).
    let close_enough = final_leg && !chasing && enemy.is_none() && dist <= POLITE_DIST;
    if !close_enough {
        let (fwd, right, _) = angle_vectors(angles);
        let dir = Vec3::new(to_wp.x, to_wp.y, 0.0).normalize_or_zero();
        forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
        side = (right.dot(dir) * MOVE_SPEED) as i32;
    }
    // Jump only while on the ground: QW pmove jumps once per press and needs the button
    // released (airborne) before it'll fire again. Gating on ground state pulses it correctly,
    // so a jump that falls short is retried on the next landing instead of the bot getting
    // stuck holding +jump against a ledge.
    if on_ground
        && (force_jump
            || bhop_cmd.is_some_and(|c| c.jump)
            || matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump)))
    {
        buttons |= BUTTON_JUMP;
    }
    // Mid-air (double) jump: rtx grants one air jump per air travel. On a double-jump leg, spend it
    // near the apex (`vz` small) to restack the arc and clear the wider gap; on a plain jump leg,
    // spend it as a *recovery* only when we're descending short of a higher target (an undershoot).
    // `air_jumped` gates re-pressing it, and the engine ignores it when the floor's close (landing).
    if !on_ground && !air_jumped && vz <= 30.0 {
        let air_jump = match kind {
            Some(LinkKind::DoubleJump) => true,
            Some(LinkKind::JumpGap) => vz < 0.0 && waypoint.z > origin.z + 20.0,
            _ => false,
        };
        if air_jump {
            buttons |= BUTTON_JUMP;
        }
    }

    // Opening a gate's button: once at it, face it and push (walk in) or shoot it.
    if let Some(gi) = bot.gate {
        let g = graph.gate(gi);
        let at_button =
            bot.route_pos >= bot.route.len() || (origin.xy() - graph.cell_origin(g.button_cell).xy()).length() < 40.0;
        if at_button {
            let d = g.aim - eye;
            let yaw = d.y.atan2(d.x).to_degrees();
            let pitch = -d.z.atan2(d.xy().length()).to_degrees();
            angles = Vec3::new(pitch, yaw, 0.0);
            look = angles; // the button needs a precise aim; the spring settles on it while parked
            buttons &= !BUTTON_JUMP;
            if g.shoot {
                // Switch to the shotgun and fire at the activator. If it's so high above us that
                // aiming would exceed the view-pitch limit (the shot lands under it), back
                // straight away first for a shallower angle — ground movement stays horizontal
                // regardless of look pitch, so we can keep aiming up while backpedalling. Only
                // fire while the activator is ready (not in its post-trigger cooldown).
                impulse = IMPULSE_SHOTGUN;
                if pitch < -68.0 {
                    forward = (-MOVE_SPEED) as i32;
                    side = 0;
                } else {
                    (forward, side) = (0, 0);
                    if weapon == Weapon::Shotgun && gate_ready[gi] {
                        buttons |= BUTTON_ATTACK;
                    }
                }
            } else {
                // Walk into the button to push it.
                let (fwd, right, _) = angle_vectors(Vec3::new(0.0, yaw, 0.0));
                let dir = (g.aim - origin).normalize_or_zero();
                forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
                side = (right.dot(dir) * MOVE_SPEED) as i32;
            }
        }
    }

    // The frame's movement as a world-space velocity, decoupled from the view: smoothing the eyes
    // below can't change where the bot goes, and combat can steer independently of its aim.
    let (nf, nr, _) = angle_vectors(angles);
    let mut move_world = nf * forward as f32 + nr * side as f32;

    // Hook override: stand still while reeling/flying (the pull owns velocity; ground input would
    // fight it or, airborne, break the frictionless arc), or walk toward the throw stance in Aim.
    if hook_engaged {
        move_world = match hook.approach {
            _ if hook.stand => Vec3::ZERO,
            Some(src) => Vec3::new(src.x - origin.x, src.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED,
            None => Vec3::ZERO,
        };
        buttons &= !BUTTON_JUMP;
        if hook.select {
            impulse = IMPULSE_GRAPPLE;
        }
    }

    // Bundle the frame's decisions into one command for the combat/grenade overlays to mutate.
    let mut cmd = BotCmd { look, move_world, buttons, impulse };

    // Combat overlay: with an enemy in sight, the combat layer picks the look (live aim with a
    // drifting error) and its own movement; having *just lost* sight it holds the angle where the
    // enemy vanished while navigation keeps driving; otherwise navigation's look/move stand. Hooks
    // in flight/reel/ballistic lock out combat — an impulse or a dropped +attack there would break
    // the reel/release (the grapple must stay selected and fire held until the release point).
    if let Some(en) = enemy.filter(|_| !hook_lock) {
        combat::engage(game, e, en, origin, now, &mut cmd);
    }

    // Splash-weapon overlays, run after `engage` (they override its aim/movement) and only when not
    // flying a hook leg. Priority: (1) defensive/opportunistic reaction to live grenades — if it
    // handled the frame it wins and any stale combo is dropped; (2) finish an in-progress grenade
    // lob→shoot combo; (3) a one-shot rocket **hazard shove** when the bot is already positioned for
    // it (cheaper than a lob); (4) otherwise start a grenade combo (hazard shove via a lobbed arc, or
    // a plain airburst). The hazard shove is the generic strategy — the knockback shoves regardless
    // of which splash weapon delivers the blast. Skipped while bunnyhopping (no enemy, view is busy
    // air-strafing).
    if !hook_engaged && !bhop_active {
        let handled = combat::grenade_tactics(game, e, enemy, origin, &mut cmd);
        if handled {
            game.entities[e].bot.grenade_phase = GrenadePhase::Idle;
        // defence drops a stale combo
        } else {
            // A one-shot rocket shove takes priority over *starting* a grenade combo, but never
            // interrupts one already in progress (the short-circuit keeps a running combo going).
            let running = game.entities[e].bot.grenade_phase != GrenadePhase::Idle;
            if running || !grenade::rocket_shove(game, e, enemy, origin, &mut cmd) {
                grenade::grenade_combo(game, e, enemy, origin, now, &mut cmd);
            }
        }
    }

    // Overlays done — unpack the command back into the frame's locals for the aim spring / emit.
    // `buttons` is still touched by the post-spring hook fire below; the rest are read-only now.
    let BotCmd { look, move_world, mut buttons, impulse } = cmd;

    // View + move for the frame. Bunnyhopping bypasses the aim spring and the world-move reprojection
    // entirely — an air-strafe needs the view yaw swept *independently* of the travel direction, with
    // `forward = 0` and one strafe key held, which the reprojection can't express. The controller
    // already decided the whole cmd (air strafe, landing strafe+jump, or ground prestrafe); consume
    // it here. Otherwise the normal aim spring smooths the view and the world move is projected onto it.
    let (view, forward, side) = if let Some(c) = bhop_cmd {
        let dt = frametime.clamp(0.001, 0.05);
        let view = Vec3::new(look.x, c.view_yaw, 0.0);
        // Seed the aim spring so combat re-acquisition continues from the real view with a plausible
        // turn rate (a human-like flick) instead of snapping.
        let b = &mut game.entities[e].bot;
        let yv = if b.bhop_prev_yaw == 0.0 {
            0.0
        } else {
            (wrap180(view.y - b.bhop_prev_yaw) / dt).clamp(-720.0, 720.0)
        };
        b.bhop_prev_yaw = view.y;
        b.aim = view;
        b.aim_vel = Vec3::new(0.0, yv, 0.0);
        (view, c.forward.round() as i32, c.side.round() as i32)
    } else {
        game.entities[e].bot.bhop_prev_yaw = 0.0; // forget the bhop yaw so the next engage seeds clean
        let dt = frametime.clamp(0.001, 0.05);
        let skill = host.cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
        // Spring stiffness (1/s): sluggish → pro-snappy. Shared with the combat feed-forward,
        // whose lag compensation assumes exactly this spring.
        let omega = combat::aim_omega(skill);
        let b = &mut game.entities[e].bot;
        if b.aim == Vec3::ZERO {
            b.aim = v_angle; // seed from the real view so the first frame doesn't snap from zero
        }
        let spring = |a: f32, v: f32, target: f32| {
            let d = wrap180(target - a);
            let v = v + (omega * omega * d - 2.0 * omega * v) * dt;
            (wrap180(a + v * dt), v)
        };
        let (pitch, pv) = spring(b.aim.x, b.aim_vel.x, look.x);
        let (yaw, yv) = spring(b.aim.y, b.aim_vel.y, look.y);
        b.aim = Vec3::new(pitch, yaw, 0.0);
        b.aim_vel = Vec3::new(pv, yv, 0.0);
        let view = b.aim;
        let (vf, vr, _) = angle_vectors(view);
        (
            view,
            vf.dot(move_world).round() as i32,
            vr.dot(move_world).round() as i32,
        )
    };

    // Hook fire, decided *after* the spring: the hook flies along `v_angle` (the smoothed view we're
    // about to send), so the throw must wait until that view has actually settled onto the anchor —
    // gating on the raw target would launch the hook while the aim is still swinging. Once thrown we
    // hold +attack every frame through the reel (the engine zeroes the cmd each tick, and a single
    // released frame would drop the hook at impact or unhook mid-reel).
    if hook.fire_ready {
        let err = wrap180(view.x - look.x).abs().max(wrap180(view.y - look.y).abs());
        if err < HOOK_AIM_TOL {
            buttons |= BUTTON_ATTACK;
            let b = &mut game.entities[e].bot;
            b.hook_phase = HookPhase::Flight;
            b.hook_started = now;
        }
    } else if hook.hold_fire {
        buttons |= BUTTON_ATTACK;
    }
    // Flush any deferred hook release now the graph/bot borrows are done.
    if let Some(h) = hook.reset {
        game.reset_grapple(h);
    }

    // Combat/gate diagnostics: what the bot is chasing and whether it's stuck at a gate. Enable
    // with `rtx_bot_debug 1` (conprint shows without `developer`).
    if host.cvar_bool(c"rtx_bot_debug") {
        let gate = game.entities[e].bot.gate;
        let route = game.entities[e].bot.route.len();
        let hph = game.entities[e].bot.hook_phase;
        let bh = &game.entities[e].bot.bhop;
        host.conprint(&cstring(&format!(
            "rtx bot{client}: enemy={} gate={gate:?} hook={hph:?} bhop={:?} hops={} flips={} peak={:.0} \
             spd={speed:.0} route={route} fwd={forward} side={side} atk={}\n",
            enemy.is_some(),
            bh.phase,
            bh.hops,
            bh.flips,
            bh.peak,
            (buttons & BUTTON_ATTACK) != 0,
        )));
    }

    host.set_bot_cmd(client, msec, view, forward, side, 0, buttons, impulse);
}

/// Wrap an angle into (-180, 180].
pub(crate) fn wrap180(a: f32) -> f32 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a < -180.0 {
        a += 360.0;
    }
    a
}

/// The first shut gate whose blocked cell lies on the remaining route, if any.
fn route_blocking_gate(graph: &NavGraph, route: &[u32], from: usize, closed: &[bool]) -> Option<usize> {
    route.get(from..)?.iter().find_map(|&leg| {
        let gi = graph.gate_of_link(leg)?;
        (*closed.get(gi)?).then_some(gi)
    })
}

/// Whether gate `gi`'s button can be reached from `from` *without* crossing gate `gi`'s own shut
/// door. False for the chicken-and-egg case (e.g. arenazap's central plate, which opens all four
/// pillars but sits behind them): a bot outside can't reach it, so committing to that gate is
/// futile — it should route around the pillar instead. A `None` path counts as unreachable.
fn button_reachable(graph: &NavGraph, from: CellId, gi: usize, gate_closed: &[bool]) -> bool {
    match graph.find_path(from, graph.gate(gi).button_cell, gate_closed) {
        Some(route) => !route.iter().any(|&leg| graph.gate_of_link(leg) == Some(gi)),
        None => false,
    }
}

/// How far a bunnyhop runway extends from `route_pos`: sum the lengths of the leading `Walk`/`Step`
/// legs while the corridor keeps roughly its heading, stopping at the first sharp turn, non-ground
/// leg, or route end. Each segment may bend up to ~30° from the *previous* segment — measuring
/// against the neighbour rather than the initial heading lets a gently curving corridor accumulate
/// (the weave tracks such bends fine); the old initial-heading test cut real-map runways so short
/// that bhop barely ever engaged.
fn runway(graph: &NavGraph, route: &[u32], route_pos: usize, origin: Vec3) -> f32 {
    // Judge bends on ~chord-length spans, not per 32u leg: grid-quantized cell centres zigzag on
    // any heading between grid axes, which a per-segment angle test misreads as constant turning.
    const CHORD: f32 = 96.0;
    const MAX_BEND: f32 = 35.0;
    let (mut dist, mut prev) = (0.0, origin.xy());
    let (mut anchor, mut anchor_dist) = (origin.xy(), 0.0);
    let mut chord_yaw = None::<f32>;
    for &leg in route.get(route_pos..).unwrap_or_default() {
        if !matches!(graph.link_kind(leg), LinkKind::Walk | LinkKind::Step) {
            break;
        }
        let tgt = graph.cell_origin(graph.link_target(leg)).xy();
        dist += (tgt - prev).length();
        prev = tgt;
        if dist - anchor_dist >= CHORD {
            let c = tgt - anchor;
            let yaw = c.y.atan2(c.x).to_degrees();
            if chord_yaw.is_some_and(|p| wrap180(yaw - p).abs() > MAX_BEND) {
                return anchor_dist; // the corridor turned somewhere in this chord — stop before it
            }
            chord_yaw = Some(yaw);
            anchor = tgt;
            anchor_dist = dist;
        }
    }
    dist
}

/// A wander destination for an idle bot with nothing to chase: a random reachable navmesh cell,
/// refreshed on arrival or every few seconds. Keeps bots moving on a human-less server instead of
/// freezing on the spawn (the "bots stand still with no human" case).
fn roam_target(game: &mut GameState, e: EntId, origin: Vec3, now: f32) -> Vec3 {
    let (wt, wtime) = {
        let b = &game.entities[e].bot;
        (b.wander_target, b.wander_time)
    };
    let reached = wt != Vec3::ZERO && (wt.xy() - origin.xy()).length() < 64.0;
    if wt != Vec3::ZERO && !reached && now < wtime {
        return wt;
    }
    let r = game.random(); // before borrowing the graph (needs &mut game)
    let Some(g) = game.nav.graph.as_ref() else {
        return origin;
    };
    if g.cells.is_empty() {
        return origin;
    }
    let idx = ((r * g.cells.len() as f32) as usize).min(g.cells.len() - 1);
    let cell = g.cell_origin(idx as u32);
    game.entities[e].bot.wander_target = cell; // disjoint field from game.nav — coexists with `g`
    game.entities[e].bot.wander_time = now + 5.0;
    cell
}

/// The nearest living human player to bot `bot_e` (skips bots, spectators, and the dead).
fn nearest_human(game: &GameState, bot_e: EntId) -> Option<EntId> {
    let maxclients = game.host().cvar(c"maxclients") as i32;
    let origin = game.entities[bot_e].v.origin;
    let mut best: Option<(EntId, f32)> = None;
    for i in 1..=maxclients as u32 {
        let e = EntId(i);
        if e == bot_e {
            continue;
        }
        let ent = &game.entities[e];
        if !ent.in_use || ent.bot.is_bot || ent.classname() != Some("player") {
            continue;
        }
        if ent.v.health <= 0.0 || ent.v.deadflag != 0.0 {
            continue;
        }
        let d = (ent.v.origin - origin).length_squared();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

/// QuakeWorld `AngleVectors` (roll assumed 0): the view's forward, right, and up unit vectors —
/// exactly what the engine's `makevectors` produces and `w_fire_grenade` orients the launch by.
pub(crate) fn angle_vectors(angles: Vec3) -> (Vec3, Vec3, Vec3) {
    let (sy, cy) = angles.y.to_radians().sin_cos();
    let (sp, cp) = angles.x.to_radians().sin_cos();
    let forward = Vec3::new(cp * cy, cp * sy, -sp);
    let right = Vec3::new(sy, -cy, 0.0);
    let up = Vec3::new(sp * cy, sp * sy, cp);
    (forward, right, up)
}

/// Drop bot bookkeeping when a bot client disconnects (kicked, or removed by the manager), so
/// a slot reused by a future human isn't mistaken for a bot.
pub fn on_disconnect(ent: &mut Entity) {
    if ent.bot.is_bot {
        ent.bot = BotState::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsp::Bsp;
    use crate::navmesh::NavGraph;

    /// On a real map, traversal routes must expose usable bhop runways. Regression for the grid
    /// staircase: cell centres sit on a 32u grid, so any route heading between grid axes zigzags
    /// (segments alternating 0°/45°), and a naive per-segment bend test reads that as a constant
    /// sharp turn — truncating every runway to nothing and keeping bhop permanently off in play.
    /// Run with `RTX_TEST_BSP=…/dm6.bsp`; skipped (vacuously green) when unset.
    #[test]
    fn real_map_routes_have_runways() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let graph = NavGraph::build(&bsp);
        // Sample routes between far-apart cell pairs and record the best runway a bot would see
        // anywhere along each route.
        let mut best = 0.0f32;
        let (mut positions, mut kind_ok, mut run_ok, mut entry_ok) = (0u32, 0u32, 0u32, 0u32);
        let stride = (graph.cells.len() / 64).max(1);
        let sample: Vec<u32> = (0..graph.cells.len() as u32).step_by(stride).collect();
        let mut routes = 0;
        for (i, &a) in sample.iter().enumerate() {
            for &b in &sample[i + 1..] {
                let d = (graph.cell_origin(a).xy() - graph.cell_origin(b).xy()).length();
                if d < 600.0 {
                    continue;
                }
                let Some(route) = graph.find_path(a, b, &[]) else {
                    continue;
                };
                routes += 1;
                for pos in 0..route.len() {
                    let origin = graph.cell_origin(graph.link_source(route[pos]));
                    let r = runway(&graph, &route, pos, origin);
                    best = best.max(r);
                    let walkish = matches!(graph.link_kind(route[pos]), LinkKind::Walk | LinkKind::Step);
                    let goal_far = (graph.cell_origin(b).xy() - origin.xy()).length() > 300.0;
                    positions += 1;
                    kind_ok += walkish as u32;
                    run_ok += (r >= bhop::RUNWAY_ENGAGE) as u32;
                    entry_ok += (walkish && goal_far && r >= bhop::RUNWAY_ENGAGE) as u32;
                }
                if routes >= 40 {
                    break;
                }
            }
            if routes >= 40 {
                break;
            }
        }
        assert!(routes > 0, "no long routes found to sample");
        eprintln!(
            "runway stats over {routes} routes, {positions} positions: best={best:.0}u \
             kind_ok={:.0}% runway_ok={:.0}% entry_ok={:.0}%",
            100.0 * kind_ok as f32 / positions as f32,
            100.0 * run_ok as f32 / positions as f32,
            100.0 * entry_ok as f32 / positions as f32,
        );
        assert!(
            best >= 400.0,
            "best runway across {routes} long routes is only {best:.0}u — real-map corridors \
             never clear the {:.0}u engage bar",
            bhop::RUNWAY_ENGAGE
        );
    }
}
