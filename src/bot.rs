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

use crate::bot_bhop;
use crate::bot_combat;
use crate::bot_grenade;
use crate::defs::{Bits, Flags, Items, Solid, Weapon};
use crate::entity::{BotState, EntId, Entity, GrenadePhase, HookPhase};
use crate::game::{cstring, GameState};
use crate::mode::BotIntent;
use crate::navmesh::{CellId, LinkKind, NavGraph};

// usercmd button bits.
const BUTTON_ATTACK: i32 = 1;
const BUTTON_JUMP: i32 = 2;
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

/// Move-component scale (matches ktx: project the desired world direction onto the view
/// vectors and scale by 800; pmove clamps to `sv_maxspeed`).
const MOVE_SPEED: f32 = 800.0;
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

// --- population management (P3) ---

/// Reconcile the live bot count to `rtx_bots`, one add/remove per call (called each normal
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

    // Only field bots while at least one human is in the game — an empty server (or one whose
    // last human just left) wants none, so the trim path below removes them.
    let want = if humans >= 1 {
        host.cvar(c"rtx_bots").max(0.0) as i32
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
    let hits: Vec<EntId> = game
        .nav
        .goals
        .iter()
        .filter_map(|&(idx, _)| {
            let item = EntId(idx);
            let it = &game.entities[item];
            (it.v.solid == Solid::Trigger && on_item(origin, it.v.origin)).then_some(item)
        })
        .collect();
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

    // Dead: pulse +attack to respawn. rtx's death-think needs all buttons *released* (Dead →
    // Respawnable) and then *pressed* again — so the button must be pulsed, not held.
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

    // Ask the active mode for this bot's intent. A round mode (Rocket Arena) returns Fight/Move to
    // drive combat or audience-roaming; FFA returns None, leaving the generic item/human brain
    // below in charge. Every mode-specific bot adaptation lives behind this one hook — the rest of
    // run_bot stays mode-agnostic and reusable.
    let mode = game.mode;
    let intent = mode.bot_intent(game, e);
    if intent.is_some() {
        game.entities[e].bot.goal_item = 0; // a mode target supersedes any item chase
    }

    // Item goal (P5): re-pick the best reachable pickup on a slow cadence, and drop a chosen item
    // once it's been grabbed (no longer available/respawning soon) so the bot moves on. With no
    // worthwhile item, fall back to following the nearest human. Skipped when a mode supplies an
    // intent (it chooses its own target below).
    if intent.is_none() {
        if now >= game.entities[e].bot.goal_select_time {
            let pick = game.select_item_goal(e);
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
    }

    // Opt-in diagnostics (`rtx_bot_debug 1`): one throttled line per bot — what it wants, how far,
    // whether it's standing on that item, and whether it owns the LG. Pinpoints pickup-vs-desire.
    if host.cvar_bool(c"rtx_bot_debug") && now >= game.entities[e].bot.repath_time {
        let gi = game.entities[e].bot.goal_item;
        let (goal, dist, overlap) = if gi != 0 {
            let it = &game.entities[EntId(gi)];
            let on = it.v.solid == Solid::Trigger && on_item(origin, it.v.origin);
            (it.classname().unwrap_or("?"), (it.v.origin - origin).length(), on)
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
    let chasing = intent.is_none() && game.entities[e].bot.goal_item != 0;
    // Where we're headed: the mode's target, the chosen item, or the nearest human. None → idle.
    let (target_origin, item_cell) = match intent {
        Some(BotIntent::Fight(en)) => (game.entities[en].v.origin, None),
        Some(BotIntent::Move(pos)) => (pos, None),
        None if chasing => {
            let it = EntId(game.entities[e].bot.goal_item);
            (game.entities[it].v.origin, Some(game.entities[e].bot.goal_item_cell))
        }
        None => {
            if let Some(h) = nearest_human(game, e) {
                (game.entities[h].v.origin, None)
            } else {
                idle(v_angle);
                return;
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
    if !hooking && bot.last_origin != Vec3::ZERO && (origin - bot.last_origin).length() > 200.0 {
        bot.route.clear();
        bot.repath_time = now;
    }
    bot.last_origin = origin;

    // Gate errand: drop it once the gate's door has opened — or give up if we stop making progress
    // toward its button (stuck at a door whose button we can't actually reach), so we don't camp
    // there. Progress-based, not a flat timeout: a button that's simply far across the map (e.g.
    // when we spawned right next to the door) still gets reached. Suspended mid-hook.
    if !hooking && bot.gate.is_some() {
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
    if !hooking && (bot.route.is_empty() || bot.goal_cell != goal || now >= bot.repath_time) {
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
    if !hooking && bot.route_pos >= bot.route.len() && bot_cell != goal && now >= bot.repath_time {
        bot.repath_time = now; // force a fresh path next frame
    }

    // Not on an errand yet? `find_path` already routes *around* a shut gate when it can (its links
    // are priced high), so if the chosen route still crosses one, there's no other way in — divert
    // to that gate's button. Skip a gate we recently gave up on (its button was unreachable) so we
    // don't immediately re-camp on it.
    if !hooking && bot.gate.is_none() {
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
    let arrive_r = if bot.bhop {
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
                to.length() <= arrive_r || (bot.bhop && to.dot(v_xy) < 0.0 && to.length() <= 64.0)
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
    let look_point = if bot.route_pos + 2 < bot.route.len() {
        graph.cell_origin(graph.link_target(bot.route[bot.route_pos + 2]))
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

    // Bunnyhop policy: air-strafe to build speed only on an open, roughly-straight ground stretch
    // toward a far waypoint, with no combat / hook / gate / grenade demanding the view or movement.
    // Eligibility must hold briefly before engaging (so a momentary straightaway doesn't stutter the
    // gait); disengaging is instant. See `bot_bhop`.
    let goal_dist = (target_origin.xy() - origin.xy()).length();
    let (runway_dist, runway_straight) = runway(graph, &bot.route, bot.route_pos, origin);
    let bhop_eligible = host.cvar_bool(c"rtx_bot_bhop")
        && enemy.is_none()
        && !hook_active
        && bot.gate.is_none()
        && bot.grenade_phase == GrenadePhase::Idle
        && !final_leg
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && goal_dist > 300.0
        && runway_straight
        && runway_dist >= 256.0f32.max(speed * 0.9);
    if bhop_eligible {
        if bot.bhop_since == 0.0 {
            bot.bhop_since = now;
        }
        bot.bhop = now - bot.bhop_since >= 0.3;
    } else {
        bot.bhop_since = 0.0;
        bot.bhop = false;
        bot.bhop_dir = 0.0;
    }
    let bhop_active = bot.bhop;

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

    // --- grappling-hook leg driver -------------------------------------------------------------
    // Fly a LinkKind::Hook leg: select the grapple, settle the view on the anchor, throw, reel to
    // build speed, then release into a gravity parabola that lands on the target ledge. All physics
    // reads come from the `anchor`/`hook_out`/`on_hook` snapshot; graph reads use `cur_leg`'s stored
    // HookTraversal. `reset_grapple` (needs `&mut game`) is deferred to `hook_reset` and flushed
    // after the graph borrow ends. Per-phase timeouts are the traversal's own stuck detection.
    let mut hook_look_target: Option<Vec3> = None; // view override → the anchor / landing
    let mut hook_stand = false; // hold ground still (reel/parabola own the velocity)
    let mut hook_approach: Option<Vec3> = None; // Aim: walk toward the throw stance (source cell)
    let mut hook_select = false; // need to switch to the grapple
    let mut hook_fire_ready = false; // Aim: eligible to throw once the smoothed view settles
    let mut hook_hold_fire = false; // Flight/Reel: keep +attack held
    let mut hook_reset: Option<EntId> = None; // deferred reset_grapple target
    let mut hook_failed = false;
    if hook_active {
        if let Some((leg, tr)) = cur_leg.and_then(|l| graph.hook_of_link(l).copied().map(|t| (l, t))) {
            let src = graph.cell_origin(graph.link_source(leg));
            let tgt = graph.cell_origin(graph.link_target(leg));
            // An enemy in an early phase → let combat win; abort a hook we haven't committed to.
            if enemy.is_some() && matches!(bot.hook_phase, HookPhase::Idle | HookPhase::Aim) {
                if hook_out {
                    hook_reset = Some(grapple_hook);
                }
                bot.hook_phase = HookPhase::Idle;
            } else {
                if bot.hook_phase == HookPhase::Idle {
                    if !has_grapple {
                        hook_failed = true; // no hook to fly this leg (a mode stripped it)
                    } else {
                        bot.hook_phase = HookPhase::Aim;
                        bot.hook_link = leg;
                        bot.hook_started = now;
                        bot.hook_release_dist = tr.release_dist;
                    }
                }
                match bot.hook_phase {
                    HookPhase::Aim => {
                        hook_look_target = Some(tr.stick);
                        if weapon != Weapon::Grapple {
                            hook_select = true;
                        }
                        if (origin.xy() - src.xy()).length() <= HOOK_STANCE {
                            hook_stand = true;
                            if weapon == Weapon::Grapple && on_ground && !hook_out {
                                hook_fire_ready = true; // the throw fires post-spring, once aimed
                            }
                        } else {
                            hook_approach = Some(src);
                        }
                        if now - bot.hook_started > HOOK_AIM_TIMEOUT {
                            hook_failed = true;
                        }
                    }
                    HookPhase::Flight => {
                        hook_look_target = Some(tr.stick);
                        hook_stand = true;
                        hook_hold_fire = true;
                        if on_hook {
                            if (anchor - tr.stick).length() > HOOK_ANCHOR_DRIFT {
                                hook_failed = true; // stuck somewhere the solve didn't predict
                            } else {
                                bot.hook_phase = HookPhase::Reel;
                                bot.hook_started = now;
                                bot.hook_prev_dist = (origin - anchor).length();
                            }
                        } else if !hook_out || now - bot.hook_started > HOOK_FLIGHT_TIMEOUT {
                            hook_failed = true; // throw missed / hit sky (server reset it)
                        }
                    }
                    HookPhase::Reel => {
                        hook_look_target = Some(tgt); // pre-aim the landing
                        hook_stand = true;
                        hook_hold_fire = true;
                        let d = (origin - anchor).length();
                        if !on_hook || !hook_out || (anchor - tr.stick).length() > HOOK_ANCHOR_DRIFT {
                            hook_failed = true; // hook lost or the anchor moved (door/plat/player)
                        } else if d - reel_half_step <= bot.hook_release_dist {
                            // Release: drop +attack so `service_grapple` lets go next PreThink.
                            hook_hold_fire = false;
                            bot.hook_phase = HookPhase::Ballistic;
                            bot.hook_started = now;
                        } else if d > bot.hook_prev_dist - 1.0 && now - bot.hook_started > HOOK_REEL_TIMEOUT {
                            hook_failed = true; // reel stalled against a lip
                        } else {
                            bot.hook_prev_dist = d.min(bot.hook_prev_dist);
                        }
                    }
                    HookPhase::Ballistic => {
                        hook_look_target = Some(tgt);
                        hook_stand = true; // zero input: the frictionless arc must match the solve
                        if on_ground && now - bot.hook_started > 0.1 {
                            if (origin.xy() - tgt.xy()).length() <= ARRIVE_RADIUS * 2.0 {
                                bot.hook_fails = 0;
                            }
                            bot.route_pos += 1; // clear the hook leg; repath from the landing
                            bot.hook_phase = HookPhase::Idle;
                            bot.repath_time = now;
                        } else if now - bot.hook_started > tr.airtime + HOOK_BALLISTIC_SLACK {
                            bot.hook_phase = HookPhase::Idle; // never landed cleanly — just repath
                            bot.repath_time = now;
                        }
                    }
                    HookPhase::Idle => {}
                }
            }
        } else {
            // hooking but the current leg isn't a solvable hook (route changed under us) — abort.
            if hook_out {
                hook_reset = Some(grapple_hook);
            }
            bot.hook_phase = HookPhase::Idle;
        }
    }
    if hook_failed {
        if hook_out {
            hook_reset = Some(grapple_hook);
        }
        bot.hook_phase = HookPhase::Idle;
        bot.hook_fails = bot.hook_fails.saturating_add(1);
        bot.repath_time = now;
        if bot.hook_fails >= 2 {
            bot.hook_fails = 0;
            bot.route.clear();
            if chasing {
                bot.avoid_item = bot.goal_item;
                bot.avoid_until = now + GOAL_AVOID_TIME;
                bot.goal_item = 0;
                bot.goal_select_time = now;
            }
        }
    }
    // Whether the hook is actively steering this frame (survives the abort branches above).
    let hook_engaged = bot.hook_phase != HookPhase::Idle;
    let hook_lock = matches!(
        bot.hook_phase,
        HookPhase::Flight | HookPhase::Reel | HookPhase::Ballistic
    );
    if let Some(t) = hook_look_target {
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
        let (fwd, right) = angle_vectors(angles);
        let dir = Vec3::new(to_wp.x, to_wp.y, 0.0).normalize_or_zero();
        forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
        side = (right.dot(dir) * MOVE_SPEED) as i32;
    }
    // Jump only while on the ground: QW pmove jumps once per press and needs the button
    // released (airborne) before it'll fire again. Gating on ground state pulses it correctly,
    // so a jump that falls short is retried on the next landing instead of the bot getting
    // stuck holding +jump against a ledge.
    if on_ground && (force_jump || bhop_active || matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump))) {
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
                let (fwd, right) = angle_vectors(Vec3::new(0.0, yaw, 0.0));
                let dir = (g.aim - origin).normalize_or_zero();
                forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
                side = (right.dot(dir) * MOVE_SPEED) as i32;
            }
        }
    }

    // The frame's movement as a world-space velocity, decoupled from the view: smoothing the eyes
    // below can't change where the bot goes, and combat can steer independently of its aim.
    let (nf, nr) = angle_vectors(angles);
    let mut move_world = nf * forward as f32 + nr * side as f32;

    // Hook override: stand still while reeling/flying (the pull owns velocity; ground input would
    // fight it or, airborne, break the frictionless arc), or walk toward the throw stance in Aim.
    if hook_engaged {
        move_world = match hook_approach {
            _ if hook_stand => Vec3::ZERO,
            Some(src) => Vec3::new(src.x - origin.x, src.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED,
            None => Vec3::ZERO,
        };
        buttons &= !BUTTON_JUMP;
        if hook_select {
            impulse = IMPULSE_GRAPPLE;
        }
    }

    // Combat overlay: with an enemy in sight, the combat layer picks the look (live aim with a
    // drifting error) and its own movement; having *just lost* sight it holds the angle where the
    // enemy vanished while navigation keeps driving; otherwise navigation's look/move stand. Hooks
    // in flight/reel/ballistic lock out combat — an impulse or a dropped +attack there would break
    // the reel/release (the grapple must stay selected and fire held until the release point).
    if let Some(en) = enemy.filter(|_| !hook_lock) {
        bot_combat::engage(
            game,
            e,
            en,
            origin,
            now,
            &mut look,
            &mut move_world,
            &mut buttons,
            &mut impulse,
        );
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
        let handled = bot_combat::grenade_tactics(
            game,
            e,
            enemy,
            origin,
            &mut look,
            &mut move_world,
            &mut buttons,
            &mut impulse,
        );
        if handled {
            game.entities[e].bot.grenade_phase = crate::entity::GrenadePhase::Idle;
        // defence drops a stale combo
        } else {
            // A one-shot rocket shove takes priority over *starting* a grenade combo, but never
            // interrupts one already in progress (the short-circuit keeps a running combo going).
            let running = game.entities[e].bot.grenade_phase != crate::entity::GrenadePhase::Idle;
            if running || !bot_grenade::rocket_shove(game, e, enemy, origin, &mut look, &mut buttons, &mut impulse) {
                bot_grenade::grenade_combo(
                    game,
                    e,
                    enemy,
                    origin,
                    now,
                    &mut look,
                    &mut move_world,
                    &mut buttons,
                    &mut impulse,
                );
            }
        }
    }

    // View + move for the frame. Bunnyhopping bypasses the aim spring and the world-move reprojection
    // entirely — an air-strafe needs the view yaw swept *independently* of the travel direction, with
    // `forward = 0` and one strafe key held, which the reprojection can't express. Otherwise the
    // normal aim spring smooths the view and the world move is projected onto it.
    let (view, forward, side) = if bhop_active {
        let dt = frametime.clamp(0.001, 0.05);
        let accel = {
            let a = host.cvar(c"sv_accelerate");
            if a > 0.0 {
                a
            } else {
                10.0
            }
        };
        let maxspeed = {
            let m = host.cvar(c"sv_maxspeed");
            if m > 0.0 {
                m
            } else {
                320.0
            }
        };
        let a_max = bot_bhop::air_accel_max(accel, maxspeed, dt);
        // Steer toward the look-ahead point (smoother than the 32u next cell).
        let ahead = look_point.xy() - origin.xy();
        let bearing = if ahead.length() > 8.0 {
            ahead.y.atan2(ahead.x).to_degrees()
        } else {
            yaw
        };
        let (view_yaw, fwd, sd) = if on_ground {
            (bearing, MOVE_SPEED as i32, 0) // ground frame between hops: run straight; jump pressed above
        } else {
            let prev_sigma = game.entities[e].bot.bhop_dir;
            let s = bot_bhop::strafe(v_xy, bearing, prev_sigma, a_max);
            game.entities[e].bot.bhop_dir = s.sigma;
            (s.view_yaw, 0, s.side.round() as i32)
        };
        let view = Vec3::new(look.x, view_yaw, 0.0);
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
        (view, fwd, sd)
    } else {
        game.entities[e].bot.bhop_prev_yaw = 0.0; // forget the bhop yaw so the next engage seeds clean
        let dt = frametime.clamp(0.001, 0.05);
        let skill = host.cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
        // Spring stiffness (1/s): sluggish → pro-snappy. Shared with the combat feed-forward,
        // whose lag compensation assumes exactly this spring.
        let omega = bot_combat::aim_omega(skill);
        let b = &mut game.entities[e].bot;
        if b.aim == Vec3::ZERO {
            b.aim = v_angle; // seed from the real view so the first frame doesn't snap from zero
        }
        let spring = |a: f32, v: f32, target: f32| {
            let mut d = (target - a) % 360.0;
            if d > 180.0 {
                d -= 360.0;
            } else if d < -180.0 {
                d += 360.0;
            }
            let v = v + (omega * omega * d - 2.0 * omega * v) * dt;
            (wrap180(a + v * dt), v)
        };
        let (pitch, pv) = spring(b.aim.x, b.aim_vel.x, look.x);
        let (yaw, yv) = spring(b.aim.y, b.aim_vel.y, look.y);
        b.aim = Vec3::new(pitch, yaw, 0.0);
        b.aim_vel = Vec3::new(pv, yv, 0.0);
        let view = b.aim;
        let (vf, vr) = angle_vectors(view);
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
    if hook_fire_ready {
        let err = wrap180(view.x - look.x).abs().max(wrap180(view.y - look.y).abs());
        if err < HOOK_AIM_TOL {
            buttons |= BUTTON_ATTACK;
            let b = &mut game.entities[e].bot;
            b.hook_phase = HookPhase::Flight;
            b.hook_started = now;
        }
    } else if hook_hold_fire {
        buttons |= BUTTON_ATTACK;
    }
    // Flush any deferred hook release now the graph/bot borrows are done.
    if let Some(h) = hook_reset {
        game.reset_grapple(h);
    }

    // Combat/gate diagnostics: what the bot is chasing and whether it's stuck at a gate. Enable
    // with `rtx_bot_debug 1` (conprint shows without `developer`).
    if host.cvar_bool(c"rtx_bot_debug") {
        let gate = game.entities[e].bot.gate;
        let route = game.entities[e].bot.route.len();
        let hph = game.entities[e].bot.hook_phase;
        host.conprint(&cstring(&format!(
            "rtx bot{client}: enemy={} gate={gate:?} hook={hph:?} bhop={bhop_active} spd={speed:.0} \
             route={route} fwd={forward} side={side} atk={}\n",
            enemy.is_some(),
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

/// How far, and whether, a bunnyhop runway extends from `route_pos`: sum the lengths of the leading
/// `Walk`/`Step` legs while each stays within ~20° of the initial heading, stopping at the first
/// turn, non-ground leg, or route end. Returns `(distance, has_a_straight_leg)`.
fn runway(graph: &NavGraph, route: &[u32], route_pos: usize, origin: Vec3) -> (f32, bool) {
    let Some(&first) = route.get(route_pos) else {
        return (0.0, false);
    };
    let base = graph.cell_origin(graph.link_target(first)).xy() - origin.xy();
    if base.length() < 1.0 {
        return (0.0, false);
    }
    let base_yaw = base.y.atan2(base.x).to_degrees();
    let (mut dist, mut prev, mut any) = (0.0, origin.xy(), false);
    for &leg in &route[route_pos..] {
        if !matches!(graph.link_kind(leg), LinkKind::Walk | LinkKind::Step) {
            break;
        }
        let tgt = graph.cell_origin(graph.link_target(leg)).xy();
        let seg = tgt - prev;
        if seg.length() > 1.0 && wrap180(seg.y.atan2(seg.x).to_degrees() - base_yaw).abs() > 20.0 {
            break; // the corridor turned — end of the straightaway
        }
        dist += seg.length();
        prev = tgt;
        any = true;
    }
    (dist, any)
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

/// QuakeWorld `AngleVectors` (roll assumed 0): the view's forward and right unit vectors.
fn angle_vectors(angles: Vec3) -> (Vec3, Vec3) {
    let (sy, cy) = angles.y.to_radians().sin_cos();
    let (sp, cp) = angles.x.to_radians().sin_cos();
    let forward = Vec3::new(cp * cy, cp * sy, -sp);
    let right = Vec3::new(sy, -cy, 0.0);
    (forward, right)
}

/// Drop bot bookkeeping when a bot client disconnects (kicked, or removed by the manager), so
/// a slot reused by a future human isn't mistaken for a bot.
pub fn on_disconnect(ent: &mut Entity) {
    if ent.bot.is_bot {
        ent.bot = BotState::default();
    }
}
