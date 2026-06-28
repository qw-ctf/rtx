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

use crate::defs::{Bits, Flags, Items};
use crate::entity::{BotState, EntId};
use crate::game::{cstring, GameState};
use crate::navmesh::{LinkKind, NavGraph};

// usercmd button bits.
const BUTTON_ATTACK: i32 = 1;
const BUTTON_JUMP: i32 = 2;
/// Impulse to select the shotgun (for shooting a health-gated button).
const IMPULSE_SHOTGUN: i32 = 2;

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
    let want = if humans >= 1 { host.cvar(c"rtx_bots").max(0.0) as i32 } else { 0 };

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
            game.entities[e].bot = BotState::default();
        }
    }
}

/// Spawn one bot. `add_bot` runs the module's ClientConnect + PutClientInServer for the new
/// edict synchronously, then we tag that edict as bot-driven. No-op if the server is full.
fn add_one_bot(game: &mut GameState, index: i32) {
    let host = *game.host();
    let name = cstring(&format!("[rtx]{}", bot_name(index)));
    let (bottom, top) = bot_colors(index);
    let client = host.add_bot(&name, bottom, top, c"base");
    if client > 0 {
        // Re-broadcast the bot's name: `set_bot_userinfo` emits `svc_setinfo` to every client,
        // which is what lands the bot on their scoreboard (ktx likewise pokes userinfo after
        // its `add_bot`). Without it the name set inside `add_bot` doesn't reliably show.
        host.set_bot_userinfo(client, c"name", &name, 0);
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
    const NAMES: [&str; 8] =
        ["Grunt", "Ranger", "Visor", "Sarge", "Bitterman", "Hossman", "Daemia", "Klesk"];
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
            run_bot(game, e);
        }
    }
}

fn run_bot(game: &mut GameState, e: EntId) {
    let host = *game.host();
    let now = game.time();
    let msec = ((game.globals.frametime * 1000.0) as i32).clamp(1, 100);

    let origin = game.entities[e].v.origin;
    let v_angle = game.entities[e].v.v_angle;
    let client = game.entities[e].bot.client;
    let weapon = game.entities[e].v.weapon;
    let on_ground = game.entities[e].v.flags.has(Flags::ONGROUND);
    let alive = game.entities[e].v.health > 0.0 && game.entities[e].v.deadflag == 0.0;
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

    let Some(target) = nearest_human(game, e) else {
        idle(v_angle);
        return;
    };
    let target_origin = game.entities[target].v.origin;

    // Graph queries (borrows game.nav) and bot-state updates (borrows game.entities) are on
    // disjoint fields, so they coexist; `host` is a Copy, no game borrow held across the send.
    let graph = game.nav.graph.as_ref().unwrap();
    let (Some(bot_cell), Some(goal_cell)) = (graph.nearest(origin), graph.nearest(target_origin))
    else {
        idle(v_angle);
        return;
    };

    // Which gates are currently shut, and whether each activator can be triggered right now?
    // Read before borrowing the bot mutably (these touch the door/movewall + activator edicts).
    // A gate is shut while its obstruction still sits at its closed origin; a shoot activator is
    // "ready" only while it takes damage — re-triggerable triggers go dead during their cooldown.
    let gate_closed: Vec<bool> = (0..graph.gate_count())
        .map(|gi| {
            let g = graph.gate(gi);
            (game.entities[EntId(g.obstruction)].v.origin - g.closed_origin).length() < 8.0
        })
        .collect();
    let gate_ready: Vec<bool> = (0..graph.gate_count())
        .map(|gi| {
            let g = graph.gate(gi);
            !g.shoot || game.entities[EntId(g.activator)].v.takedamage != 0.0
        })
        .collect();

    let bot = &mut game.entities[e].bot;

    // A teleport (or any large instant displacement) invalidates the planned route — drop it
    // and re-path from where we landed. ~200u in one frame is far beyond running/falling.
    if bot.last_origin != Vec3::ZERO && (origin - bot.last_origin).length() > 200.0 {
        bot.route.clear();
        bot.repath_time = now;
    }
    bot.last_origin = origin;

    // Gate errand: drop it once the gate's door has opened.
    if let Some(gi) = bot.gate {
        if gate_closed.get(gi).copied() != Some(true) {
            bot.gate = None;
            bot.route.clear();
            bot.repath_time = now;
        }
    }

    // Effective goal: the human, or — while opening a gate — that gate's button.
    let goal = match bot.gate {
        Some(gi) => graph.gate(gi).button_cell,
        None => goal_cell,
    };

    // Re-path when the route is empty, the goal changed, or the timer elapsed.
    if bot.route.is_empty() || bot.goal_cell != goal || now >= bot.repath_time {
        bot.route = graph.find_path(bot_cell, goal).unwrap_or_default();
        bot.route_pos = 0;
        bot.goal_cell = goal;
        bot.repath_time = now + REPATH_INTERVAL;
    }
    // If we've fallen off the planned route (missed a jump, got shoved), re-localize next.
    if bot.route_pos >= bot.route.len() && bot_cell != goal && now >= bot.repath_time {
        bot.repath_time = now; // force a fresh path next frame
    }

    // Not on an errand yet? If the route to the human crosses a shut gate, divert to its button.
    if bot.gate.is_none() {
        if let Some(gi) = route_blocking_gate(graph, &bot.route, bot.route_pos, &gate_closed) {
            let button_cell = graph.gate(gi).button_cell;
            bot.gate = Some(gi);
            bot.route = graph.find_path(bot_cell, button_cell).unwrap_or_default();
            bot.route_pos = 0;
            bot.goal_cell = button_cell;
            bot.repath_time = now + REPATH_INTERVAL;
        }
    }

    // Advance past route legs we've already reached. A plat leg completes when we've *risen*
    // to the exit height (Z), not on XY arrival — we're standing still on the lift while it
    // carries us up, so XY barely changes.
    while bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let target = graph.cell_origin(graph.link_target(leg));
        let arrived = if graph.link_kind(leg) == LinkKind::Plat {
            origin.z >= target.z - PLAT_RISE_TOL
        } else {
            (target.xy() - origin.xy()).length() <= ARRIVE_RADIUS
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
    let (waypoint, kind, final_leg) = if bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let k = graph.link_kind(leg);
        let aim = if k == LinkKind::Plat {
            graph.cell_origin(graph.link_source(leg))
        } else {
            graph.cell_origin(graph.link_target(leg))
        };
        (aim, Some(k), false)
    } else {
        (target_origin, None, true)
    };

    // Stuck detection.
    let mut force_jump = false;
    if (origin - bot.stuck_origin).length() > STUCK_MOVE {
        bot.stuck_origin = origin;
        bot.stuck_since = now;
    } else if now - bot.stuck_since > STUCK_TIME {
        force_jump = true;
        bot.repath_time = now; // re-path next frame
        bot.stuck_since = now;
    }

    // Steering: face the waypoint and run toward it.
    let to_wp = waypoint.xy() - origin.xy();
    let dist = to_wp.length();
    let yaw = to_wp.y.atan2(to_wp.x).to_degrees();
    let mut angles = Vec3::new(0.0, yaw, 0.0);

    let (mut forward, mut side, mut buttons, mut impulse) = (0, 0, 0, 0);
    let close_enough = final_leg && dist <= POLITE_DIST;
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
    if on_ground && (force_jump || matches!(kind, Some(LinkKind::JumpGap))) {
        buttons |= BUTTON_JUMP;
    }

    // Opening a gate's button: once at it, face it and push (walk in) or shoot it.
    if let Some(gi) = bot.gate {
        let g = graph.gate(gi);
        let at_button = bot.route_pos >= bot.route.len()
            || (origin.xy() - graph.cell_origin(g.button_cell).xy()).length() < 40.0;
        if at_button {
            let eye = origin + Vec3::new(0.0, 0.0, 22.0);
            let d = g.aim - eye;
            let yaw = d.y.atan2(d.x).to_degrees();
            let pitch = -d.z.atan2(d.xy().length()).to_degrees();
            angles = Vec3::new(pitch, yaw, 0.0);
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
                    if weapon == Items::SHOTGUN.as_f32() && gate_ready[gi] {
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

    host.set_bot_cmd(client, msec, angles, forward, side, 0, buttons, impulse);
}

/// The first shut gate whose blocked cell lies on the remaining route, if any.
fn route_blocking_gate(graph: &NavGraph, route: &[u32], from: usize, closed: &[bool]) -> Option<usize> {
    route.get(from..)?.iter().find_map(|&leg| {
        let gi = graph.gate_of_cell(graph.link_target(leg))?;
        (*closed.get(gi)?).then_some(gi)
    })
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
pub fn on_disconnect(ent: &mut crate::entity::Entity) {
    if ent.bot.is_bot {
        ent.bot = BotState::default();
    }
}
