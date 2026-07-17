// SPDX-License-Identifier: AGPL-3.0-or-later

//! `run_bot`'s route-steering core: the disjoint `&nav` / `&mut bot` region.
//!
//! Everything from "we have a bot cell and a goal" through "we have a movement command" — teleport
//! invalidation, gate errands, the repath / banded-A*, leg advancement, the plat standoff, the
//! stuck/progress watchdogs, the bunnyhop policy verdicts + controller, the hook/rocket-jump leg
//! drivers, and the final steering/look/move composition. It runs entirely on `graph` (an immutable
//! `&NavGraph`) plus `&mut BotState` plus the all-`Copy` frame snapshot in [`SteerCtx`] — never
//! `&mut GameState`. That is exactly what lets `run_bot` hold the two disjoint borrows here and then
//! resume the `&mut game` spine (combat/grenade overlays, `emit`) once [`steer`] returns.

use glam::{Vec2, Vec3, Vec3Swizzles};

use super::*;
use crate::bsp::Bsp;
use rtx_nav::qphys::ORIGIN_TO_FEET;
use crate::bot::state::{AirCommit, Commit, GateErrand, PlatWait, TerminalArrival};
use crate::math::{angle_vectors, angles_to, yaw_of};
use crate::defs::{Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK, BUTTON_JUMP};
use crate::game::cstring;
use crate::nav_build::PlatStatus;
use crate::navmesh::{CellId, Corridor, LinkCosts, LinkKind, NavGraph};
use crate::nearfield;

/// How long a real network client waits at a reached pickup terminal for the authoritative server
/// touch/stat update before trying another terminal or abandoning the item.
pub(super) const TERMINAL_TAKE_GRACE: f32 = 0.35;

/// The all-`Copy` frame snapshot `steer` reads: the [`Sense`] and [`Objective`] this frame, the
/// per-bot A* costs, and the live gate/plat state gathered before the borrow (see `run_bot`).
pub(super) struct SteerCtx<'a> {
    pub s: Sense,
    pub o: Objective,
    pub costs: LinkCosts<'a>,
    pub plat_status: &'a [PlatStatus],
    pub gate_ready: &'a [bool],
    pub bot_cell: CellId,
    pub goal_cell: CellId,
    pub race_line_ahead: Option<Vec3>,
    pub weapons_hot: bool,
    /// The collision hull for the live forward wall probe (bhop wall-avoidance). `None` = no BSP
    /// (degenerate/test map) → the probe reports open, same as off the live path.
    pub bsp: Option<&'a Bsp>,
}

/// What `steer` hands back to the spine: the frame's command (which the combat/grenade overlays then
/// mutate), the bhop/hook/rocket-jump driver outputs `emit` applies, and the two gates that decide
/// whether the overlays run.
pub(super) struct SteerOut {
    pub cmd: BotCmd,
    pub bhop_cmd: Option<bhop::Cmd>,
    pub hook: hook::HookDrive,
    pub rj: rj::RjDrive,
    /// Traversal-critical leg (hook/rj lock, airborne, or a gap/double/speed jump) — combat is
    /// locked out (`engage` owns movement and clears +jump).
    pub traversal_lock: bool,
    /// The grenade/rocket overlays may run (not hooking/rj/bhop-ing and not traversal-locked).
    pub overlays_ok: bool,
}

/// The LOD steer corridor toward `goal` — the interim search target plus its cluster window — or
/// `None` when lod is off or the goal is near enough to steer at directly. Shared by the main repath
/// and the one-shot gate-errand route so both bound a far target the same way.
fn corridor_to(graph: &NavGraph, from: CellId, goal: CellId, costs: &LinkCosts, lod: bool) -> Option<Corridor> {
    lod.then(|| graph.corridor(from, goal, costs, STEER_LOD_HORIZON)).flatten()
}

/// A plain (unbanded) route from `from` to `target`, restricted to the corridor `window` when present.
/// The abstract corridor is a real in-window fine path, so the restricted search normally succeeds; if
/// it somehow comes up empty it falls back to an unrestricted search to the (near) interim, bounded by
/// its proximity. This is the one fallback ladder the main repath and the gate errand both use.
fn windowed_route(graph: &NavGraph, from: CellId, target: CellId, costs: &LinkCosts, window: Option<&[bool]>) -> Vec<u32> {
    let plain = match window {
        Some(w) => graph.find_path_within(from, target, costs, w),
        None => graph.find_path(from, target, costs),
    };
    let r = plain.unwrap_or_default();
    if r.is_empty() && window.is_some() {
        return graph.find_path(from, target, costs).unwrap_or_default();
    }
    r
}

/// The closed gates whose shut volume sits near `origin` — the near-field's invalidation key (each a
/// bit) and, when it rebuilds, the door boxes it stamps unwalkable. A door opening/shutting nearby
/// flips a bit and forces a rebuild; the radius carries a recenter's slack so a bit stays stable under
/// sub-recenter movement (no per-frame churn). Gate ids are single-digit, so `1 << gi` fits a `u32`.
fn nearfield_gates<'a>(graph: &'a NavGraph, gate_closed: &'a [bool], origin: Vec3) -> impl Iterator<Item = usize> + 'a {
    let reach = nearfield::NEAR_HALF + nearfield::NEAR_RECENTER;
    (0..graph.gate_count()).filter(move |&gi| {
        gate_closed.get(gi).copied().unwrap_or(false) && {
            let g = graph.gate(gi);
            let nearest = origin.xy().clamp(g.closed_min.xy(), g.closed_max.xy());
            (nearest - origin.xy()).length() <= reach
        }
    })
}

/// The lip is "right here" — inside this distance the takeoff jump must fire *now* or the bot wedges
/// against the step face; beyond it the run-up gate applies.
const JUMP_NOW_DIST: f32 = 40.0;

/// Whether a plain jump leg (`JumpGap`/`DoubleJump`) may fire its takeoff jump this frame. Applying
/// forward *after* the leap barely helps in QW air physics, so the speed must already be carried
/// *toward the waypoint* before jumping — gate on the velocity component along `to_wp` reaching
/// `frac · maxspeed`. Two escapes keep a bot from wedging: the lip is within [`JUMP_NOW_DIST`] (jump
/// now), or the gate is off (`frac <= 0`). `frac` is `rtx_jump_runup`.
fn jump_runup_ok(v_xy: Vec2, to_wp: Vec2, dist: f32, frac: f32, maxspeed: f32) -> bool {
    if frac <= 0.0 || dist < JUMP_NOW_DIST {
        return true;
    }
    v_xy.dot(to_wp.normalize_or_zero()) >= frac * maxspeed
}

fn abandon_terminal_item(bot: &mut BotState, item: u32, now: f32) {
    debug_assert_eq!(bot.goal.item, item);
    bot.abandon_item_goal(now, now + GOAL_AVOID_TIME);
    bot.route.clear();
    bot.goal_cell = None;
    bot.repath_time = now;
}

fn consume_terminal_retry(bot: &mut BotState, item: u32, alternate: CellId, now: f32) {
    bot.goal.item_cell = alternate;
    bot.goal.terminal_retried_item = Some(EntId(item));
    bot.goal.terminal_arrival = None;
    // The ordinary 1.5 s selector must not immediately rebind the just-failed primary terminal.
    bot.goal.next_pick = now + GOAL_SELECT_INTERVAL;
    bot.route.clear();
    bot.goal_cell = None;
    bot.repath_time = now;
}

fn update_item_terminal(
    bot: &mut BotState,
    at_item_terminal: bool,
    item_solid: Solid,
    goal_cell: CellId,
    alternate_item_cell: Option<CellId>,
    now: f32,
) -> bool {
    if at_item_terminal && item_solid == Solid::Trigger {
        let item = bot.goal.item;
        let arrived_at = match bot.goal.terminal_arrival {
            Some(arrival) if arrival.item == EntId(item) && arrival.cell == goal_cell => arrival.at,
            _ => {
                bot.goal.terminal_arrival = Some(TerminalArrival {
                    item: EntId(item),
                    cell: goal_cell,
                    at: now,
                });
                now
            }
        };
        if now - arrived_at >= TERMINAL_TAKE_GRACE {
            if bot.goal.terminal_retried_item != Some(EntId(item)) {
                if let Some(alternate) = alternate_item_cell {
                    consume_terminal_retry(bot, item, alternate, now);
                    return true;
                }
                abandon_terminal_item(bot, item, now);
                return true;
            }
            abandon_terminal_item(bot, item, now);
            return true;
        }
    } else {
        bot.goal.terminal_arrival = None;
    }
    false
}

pub(super) fn steer(graph: &NavGraph, bot: &mut BotState, ctx: SteerCtx) -> SteerOut {
    let SteerCtx { s, o, costs, plat_status, gate_ready, bot_cell, goal_cell, race_line_ahead, weapons_hot, bsp } =
        ctx;
    let Sense {
        host, now, frametime, origin, v_angle, client, weapon, on_ground, in_water, vz, air_jumped,
        enemy_seen_time, v_xy, speed, grapple_hook, has_grapple, hook_out, on_hook, anchor, reel_half_step,
        attack_finished, has_rl, ammo_rockets, health, armortype, armorvalue, quad, ..
    } = s;
    let Objective {
        hooking,
        on_sj,
        on_rj,
        enemy,
        chasing,
        polite,
        vigil,
        item_solid,
        target_origin,
        item_cell,
        alternate_item_cell,
        watch_point,
        ..
    } = o;
    let gate_closed = costs.gate_closed;

    // Plain-jump commitment is normally pre-armed before objective resolution. Remember the first
    // physical airborne frame here; route kind/position is intentionally irrelevant to release.
    if !on_ground {
        if let Some(c) = bot.air.as_mut() {
            c.airborne = true;
        }
    }
    // Puppet rocket-jump order (test harness, see [`crate::control`]): pin the route to the single
    // ordered link so the repath / leg-advance / errand logic below can't clobber the one-leg route
    // the rocket-jump driver flies. Folded into `route_frozen` below, so every `!route_frozen` guard
    // also respects the pin. A RocketJump link never auto-advances (its driver advances on landing),
    // so the leg stays put until the attempt finishes and the control poller lifts the order. Goto/Hold
    // orders leave `order_link` None and route normally. Rebuilds only when the route isn't already it.
    let pinned = o.order_link.is_some();
    if let Some(link) = o.order_link {
        if bot.route.len() != 1 || bot.route.first() != Some(&link) {
            bot.route = vec![link];
            bot.route_bands = vec![0];
            bot.route_pos = 0;
            bot.goal_cell = Some(graph.link_target(link));
        }
    }
    // Incoming commitment (reads the route state *before* this frame's displacement handler): a >200u
    // jump while hooking, on a speed/rocket jump, riding a plain-jump arc, or pinned is that traversal
    // moving fast on purpose — not a teleport — so the handler below must leave the route alone.
    let frozen_pre = hooking || on_sj || on_rj || bot.air.is_some() || pinned;

    // A teleport (or any large instant displacement) invalidates the planned route — drop it and
    // re-path from where we landed. ~200u in one frame is far beyond running/falling. Skipped mid-hook:
    // the reel and the parabola move fast on purpose and must not clear the hook route.
    //
    // Exception — a *launch* teleporter: it flings you out airborne carrying the exit velocity, and the
    // ballistic arc lands on the far ledge the navmesh linked as the leg's target. Re-pathing from
    // mid-air instead localizes to whatever floor cell sits under the apex and air-steers off the ledge,
    // so the bot sails past the destination. When the leg we were walking into is a Teleport and we came
    // out airborne, commit to that target as an air arc (released on landing, like a jump leg) so the
    // air-strafe below curves us onto it. A teleport that drops you standing (`on_ground`) still clears
    // and re-paths, exactly as before.
    if !frozen_pre && bot.watchdog.last_origin != Vec3::ZERO && (origin - bot.watchdog.last_origin).length() > 200.0 {
        let launch = bot
            .route
            .get(bot.route_pos)
            .filter(|&&l| graph.link_kind(l) == LinkKind::Teleport && !on_ground)
            .map(|&l| (l, graph.link_target(l)));
        if let Some((leg, target)) = launch {
            bot.air = Some(AirCommit { leg, target, since: now, airborne: true });
        } else {
            // A grounded teleport exit clears the route here. Stamp the just-ridden pad (and the reverse
            // pad by the exit) into the re-entry surcharge ring so a shuttle re-prices itself, and — under
            // debug — log it (two `tele` lines from one bot within a few seconds is a round-trip).
            if let Some(&leg) = bot.route.get(bot.route_pos) {
                if graph.link_kind(leg) == LinkKind::Teleport {
                    stamp_tele_reuse(bot, graph, leg, origin, now);
                    if host.cvar_bool(c"rtx_bot_debug") {
                        host.conprint(&cstring(&format!("rtx bot{client}: tele leg={leg}\n")));
                    }
                }
            }
            bot.route.clear();
            bot.repath_time = now;
        }
    }
    bot.watchdog.last_origin = origin;

    // Settle the commitment view for the rest of the frame. `on_air`/`route_frozen` now include a
    // launch-teleport arc just latched above, so the repath / gate / leg-advance logic all treat it as a
    // committed airborne traversal and won't yank the route out from under it. (A goal flip mid-arc must
    // not replace the route and turn the bot around.) Plain jumps used to be a collection of separate
    // `!on_air` guards, leaving holes such as gate errands; one ownership bit closes those seams.
    let on_air = bot.air.is_some();
    let route_frozen = hooking || on_sj || on_rj || on_air || pinned;
    // Read the LOD toggle once for the whole steer pass (errand reachability bound, repath corridor,
    // far gate pre-arm, errand route) — one engine cvar lookup, one consistent value across the frame.
    let lod = host.cvar_bool(c"rtx_bot_lod");

    // Gate errand: drop it once the gate's door has opened — or give up if we stop making progress
    // toward its button (stuck at a door whose button we can't actually reach), so we don't camp
    // there. Progress-based, not a flat timeout: a button that's simply far across the map (e.g.
    // when we spawned right next to the door) still gets reached. Suspended mid-hook.
    if !route_frozen {
        if let Some(errand) = bot.gate.errand {
            let gi = errand.index;
            let give_up = |bot: &mut BotState| {
                bot.gate.avoid = Some((gi, now + GATE_AVOID_TIME));
                bot.gate.errand = None;
                bot.route.clear();
                bot.repath_time = now;
            };
            if gate_closed.get(gi).copied() != Some(true) {
                bot.gate.errand = None; // door opened — done
                bot.route.clear();
                bot.repath_time = now;
            } else if now >= bot.repath_time && !button_reachable(graph, bot_cell, gi, &costs, lod) {
                // Re-verify the button is reachable without crossing this very gate (the arenazap
                // chicken-and-egg case) only at the repath cadence, not every frame: `button_reachable`
                // runs a whole-graph search, and the far pre-arm now routinely aims errands at distant
                // buttons. A door that opens is caught above every frame, and a bot that stops making
                // progress toward a now-unreachable button is caught by the give-up clock below, so a
                // ≤`REPATH_INTERVAL` lag on the topology-flip case is harmless.
                give_up(bot); // button is walled off behind this very gate — route around instead
            } else {
                let d = (graph.cell_origin(graph.gate(gi).button_cell).xy() - origin.xy()).length();
                if d < errand.best_dist - 4.0 {
                    let e = bot.gate.errand.as_mut().unwrap();
                    e.best_dist = d; // got closer — reset the give-up clock
                    e.since = now;
                } else if now - errand.since > GATE_GIVEUP_TIME {
                    give_up(bot); // no progress toward a reachable button — stuck; try elsewhere
                }
            }
        }
    }

    // Effective goal: the human, or — while opening a gate — that gate's button.
    let goal = match bot.gate.errand {
        Some(errand) => graph.gate(errand.index).button_cell,
        None => goal_cell,
    };

    // Re-path when the route is empty, the goal changed, or the timer elapsed. Frozen mid-hook, on a
    // speed/rocket jump, or committed to a plain jump arc, so the traversal keeps the route that put
    // it on that leg (a goal flip mid-air must not replace the route and turn the bot around).
    if !route_frozen && !on_air && (bot.route.is_empty() || bot.goal_cell != Some(goal) || now >= bot.repath_time) {
        // Speed-band planning credits the speed a bot carries between legs (chained speed jumps,
        // cheaper hot Walk legs) — gated on bhop being on (no speed-jump links otherwise) plus its
        // own escape-hatch cvar. `speed` seeds the start band, so a mid-run re-path keeps a hop
        // chain alive. Falls back to the plain cell A* (bands all-zero) when off.
        let use_bands = host.cvar_bool(c"rtx_bot_bhop") && host.cvar_bool(c"rtx_bot_bandplan");
        // Where can we actually head? Unreachability is pure topology (every dynamic cost term is
        // finite — see `navmesh::reach`), so resolve the target *before* searching instead of
        // discovering a dead goal by watching a whole-graph search exhaust and then flooding to find
        // the nearest reachable cell. A goal behind a shut door with no way around, or in a
        // disconnected pocket, redirects to the reachable cell nearest it — the bot heads as far
        // toward the target as the graph allows (often enough for line of sight) rather than homing
        // into a wall.
        let target = if graph.reachable(bot_cell, goal) {
            goal
        } else {
            graph.nearest_reachable_to(bot_cell, goal).unwrap_or(goal)
        };
        // LOD steer corridor: for a far target, aim the fine search at an interim portal ~a few seconds
        // along the coarse route *and* restrict expansion to the corridor's clusters — so even a
        // band-infeasible exhaustion stays a local neighbourhood instead of draining the whole
        // cells×NBANDS space. The abstract corridor is a real fine path through those clusters, so a
        // route to the interim always exists in the window. The next repath advances it; `bot.goal_cell`
        // stays the true `goal`, so change detection and the gate/give-up logic are untouched. Off →
        // today's exact whole-graph search to the target.
        let corridor = corridor_to(graph, bot_cell, target, &costs, lod);
        let (search_target, window) = match &corridor {
            Some(c) => (c.interim, Some(c.allowed.as_slice())),
            None => (target, None),
        };
        let banded = use_bands
            .then(|| match window {
                Some(w) => graph.find_path_banded_within(bot_cell, search_target, speed, &costs, w),
                None => graph.find_path_banded(bot_cell, search_target, speed, &costs),
            })
            .flatten();
        let (route, mut bands) = match banded {
            Some(r) => (r.links, r.bands),
            // Banded came back empty ⇒ band-infeasible (a route that only exists through a speed-jump
            // chain the carried speed can't satisfy) or bands off; the plain A* ignores bands and finds
            // the reachable target (windowed, with the unrestricted fallback — see [`windowed_route`]).
            None => (windowed_route(graph, bot_cell, search_target, &costs, window), Vec::new()),
        };
        // Keep `route_bands` parallel to `route`: zero-fill when unbanded (or on any length mismatch).
        if bands.len() != route.len() {
            bands = vec![0u8; route.len()];
        }
        // Empty-route telemetry (C6): a resolved repath that came back with no legs while the bot isn't
        // already at its goal — the "parked in place" signature. `corr` shows whether a corridor was in
        // play (a same-cluster/near None vs a windowed search that found nothing).
        if host.cvar_bool(c"rtx_bot_debug") && route.is_empty() && bot_cell != target {
            host.conprint(&cstring(&format!(
                "rtx bot{client}: route=0 corr={} tgt_eq_goal={}\n",
                corridor.is_some(),
                target == goal
            )));
        }
        bot.route = route;
        bot.route_bands = bands;
        bot.route_pos = 0;
        bot.goal_cell = Some(goal);
        // Remember the gates the corridor crosses beyond the interim window (nearest first), so the
        // gate-errand block can pre-arm for a far shut door the truncated route won't reveal (see
        // [`GateState`]). Empty when there's no corridor, which also clears any previous list.
        bot.gate.corridor_gates = corridor.as_ref().map_or_else(Vec::new, |c| c.far_gates.clone());
        bot.repath_time = now + REPATH_INTERVAL;
        // Restart the progress watchdog against the new route (INFINITY ⇒ the first frame records the
        // real starting distance rather than reading as an instant stall on an old baseline).
        bot.watchdog.progress_best = f32::INFINITY;
        bot.watchdog.progress_since = now;
    }
    // If we've fallen off the planned route (missed a jump, got shoved), re-localize next.
    if !route_frozen && !on_air && bot.route_pos >= bot.route.len() && bot_cell != goal && now >= bot.repath_time {
        bot.repath_time = now; // force a fresh path next frame
    }

    // Not on an errand yet? `find_path` already routes *around* a shut gate when it can (its links
    // are priced high), so if the chosen route still crosses one, there's no other way in — divert
    // to that gate's button. Skip a gate we recently gave up on (its button was unreachable) so we
    // don't immediately re-camp on it.
    if !route_frozen && !on_air && bot.gate.errand.is_none() {
        // Skip a gate we recently gave up on, while its avoid window is still open.
        let avoid = bot.gate.avoid.filter(|&(_, until)| now < until).map(|(gi, _)| gi);
        // A shut gate on the current route, or — under LOD, where the route stops at the interim — the
        // first still-shut gate the corridor crosses further along, in route order (the far pre-arm,
        // matching exact mode's full-route detection). `route_blocking_gate` takes priority (the more
        // precise near case). The far list is consulted only while lod is on, so a mid-game
        // `rtx_bot_lod 0` can't act on a stale corridor.
        let far = lod
            .then(|| {
                bot.gate.corridor_gates.iter().copied().find(|&gi| {
                    gate_closed.get(gi as usize).copied().unwrap_or(false) && Some(gi as usize) != avoid
                })
            })
            .flatten()
            .map(|gi| gi as usize);
        let block = route_blocking_gate(graph, &bot.route, bot.route_pos, gate_closed)
            .filter(|&gi| Some(gi) != avoid)
            .or(far);
        if let Some(gi) = block {
            if button_reachable(graph, bot_cell, gi, &costs, lod) {
                let button_cell = graph.gate(gi).button_cell;
                // first frame records the starting distance (best_dist starts at +inf)
                bot.gate.errand = Some(GateErrand { index: gi, best_dist: f32::INFINITY, since: now });
                // Bound this one-shot errand route the same way the main repath does (subsequent
                // repaths target the button as `goal` and corridor-bound it); a far button on a big
                // map would otherwise be one unbounded whole-graph A*.
                let corridor = corridor_to(graph, bot_cell, button_cell, &costs, lod);
                let (search_target, window) = match &corridor {
                    Some(c) => (c.interim, Some(c.allowed.as_slice())),
                    None => (button_cell, None),
                };
                bot.route = windowed_route(graph, bot_cell, search_target, &costs, window);
                bot.route_bands = vec![0u8; bot.route.len()]; // a walking errand, no carried speed
                bot.route_pos = 0;
                bot.goal_cell = Some(button_cell);
                bot.repath_time = now + REPATH_INTERVAL;
            } else {
                // Button is walled off behind this gate — don't chase it; avoid the gate so
                // route_blocking_gate stops re-selecting it and find_path routes around the pillar.
                bot.gate.avoid = Some((gi, now + GATE_AVOID_TIME));
            }
        }
    }

    // Advance past route legs we've already reached. A plat leg completes when we've *risen*
    // to the exit height (Z), not on XY arrival — we're standing still on the lift while it
    // carries us up, so XY barely changes.
    // A bunnyhopping bot covers ground fast enough to orbit a 24u waypoint, so widen the arrival gate
    // with speed and also advance once a waypoint slips *behind* the velocity.
    let arrive_r = if bot.bhop.phase != bhop::Phase::Off || bot.sj.is_some() {
        ARRIVE_RADIUS.max(2.0 * speed * frametime)
    } else {
        ARRIVE_RADIUS
    };
    // While committed to a plain jump arc and still airborne, don't advance the leg: keep `kind` and
    // the waypoint pinned to the jump so steering stays on the landing point and the air-jump
    // undershoot recovery keeps firing (the leg advances naturally once we land). Like Hook/RocketJump,
    // whose drivers advance on landing, not on passing the target XY.
    while (on_ground || (!on_air && !on_sj)) && bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let target = graph.cell_origin(graph.link_target(leg));
        let arrived = match graph.link_kind(leg) {
            LinkKind::Plat => origin.z >= target.z - PLAT_RISE_TOL,
            // A hook leg never auto-advances on XY: a near-vertical pull-up passes the XY test while
            // still at the *bottom* of the swing. The hook driver advances it only once the parabola
            // has landed (see below).
            LinkKind::Hook => false,
            // Same for a rocket jump — its driver advances on landing, not on passing the target XY.
            LinkKind::RocketJump => false,
            _ => {
                let to = target.xy() - origin.xy();
                let fast = bot.bhop.phase != bhop::Phase::Off || bot.sj.is_some();
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
    // human (final approach). A Plat and a *grounded* Teleport both aim at the leg's *source* cell
    // rather than its target, for the same reason from opposite ends: you don't walk toward where the
    // leg *sends* you, you stay in the thing that does the sending. A plat's exit ledge is across a gap
    // you can't reach until it lifts you; a teleporter's exit is across the map, often through a wall —
    // steer at it and the bot walks *out* of the trigger it needs to stand in, reaches nothing, and
    // turns around. Aim at the source and it walks *into* the trigger; touching it teleports.
    //
    // Once airborne on a launch teleporter's arc (the displacement handler above latched a teleport
    // AirCommit), the roles flip: the source is now across the map behind us and the *target* ledge is
    // where the arc must land, so aim there and let the air-strafe curve us onto it.
    let (waypoint, kind, final_leg, cur_leg) = if bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let k = graph.link_kind(leg);
        let aim_source = matches!(k, LinkKind::Plat) || (k == LinkKind::Teleport && on_ground);
        let aim = if aim_source {
            graph.cell_origin(graph.link_source(leg))
        } else {
            graph.cell_origin(graph.link_target(leg))
        };
        (aim, Some(k), false, Some(leg))
    } else {
        (target_origin, None, true, None)
    };

    // Reaching a catalogued pickup terminal while the item remains active means the authoritative
    // touch has not arrived. Give a network update a short grace, then immediately retry one other
    // touch-valid terminal. A second miss (or no alternate) avoids the item now — never spend the
    // ordinary/major goal watchdog pushing into nearby geometry.
    let at_item_terminal = chasing
        && item_cell == Some(goal_cell)
        && final_leg
        && bot_cell == goal_cell
        && (target_origin - origin).length() <= ARRIVE_RADIUS;
    let terminal_changed =
        update_item_terminal(bot, at_item_terminal, item_solid, goal_cell, alternate_item_cell, now);

    // Plat standoff. If an upcoming leg boards/rides a func_plat that isn't at its bottom, and we're
    // not already aboard it, walking to the board point would put us inside the lift's inner trigger
    // (the footprint shrunk 25u, spanning the full travel height) — and `plat_center_touch` resets
    // the lower-timer for any live player inside, so a bot waiting there would hold the lift raised
    // forever (and can wedge under a non-solid one). Instead hold a standoff outside the footprint
    // until it descends. The board leg itself may be a couple of Walk legs ahead, so scan a small
    // window and gate on proximity — the walk-in cells sit inside the full-height trigger too.
    let plat_hold: Option<usize> = bot
        .route
        .get(bot.route_pos..)
        .into_iter()
        .flatten()
        .take(PLAT_LOOKAHEAD)
        .find_map(|&l| graph.plat_of_link(l))
        .filter(|&pi| {
            let st = &plat_status[pi];
            let p = graph.plat(pi);
            let riding =
                origin.z > st.surface_z + 8.0 && in_footprint(origin.xy(), p.fp_min, p.fp_max, 0.0);
            !st.down && !riding && in_footprint(origin.xy(), p.fp_min, p.fp_max, PLAT_ENGAGE)
        });
    // Note there is deliberately no "the bot is loitering under a raised lift, walk it out" reflex here.
    // Standing in a shaft is not a state to detect and recover from on a timer — it is a state nothing
    // should ever choose. Every spot a bot comes to rest on is picked by a chooser that now refuses
    // shaft cells (`roam_target`, `vigil::pick_post`) and combat's footing demotes a dodge into one, so
    // a bot only ever *crosses* a shaft — and a crossing needs no permission to end. Anything else that
    // leaves a bot standing there is a bug in the chooser, and belongs fixed there rather than papered
    // over by a grace period that would, by construction, still stand under the lift for its duration.
    // While holding, steer to the standoff point and borrow the Plat leg's driver treatment (no
    // jump-press, no bhop entry, no air-latch, progress-watchdog exempt) by presenting `kind` as Plat.
    let (waypoint, kind) = if terminal_changed {
        (origin, None)
    } else {
        match plat_hold {
        Some(pi) => {
            let p = graph.plat(pi);
            (plat_standoff(origin, p.fp_min, p.fp_max), Some(LinkKind::Plat))
        }
        None => (waypoint, kind),
        }
    };

    // Waypoint magnetism: `resolve_objective` picked a desirable up item near the route; if it lies on
    // this leg's corridor, bend the immediate waypoint through it so the hull actually crosses the
    // trigger (a network-client bot has no generous pickup box — only the tight server-side overlap).
    // Only on a plain walk/step leg or the final approach (`None`) and never while airborne, bhopping,
    // holding off a plat, or running a gate errand — those own the feet and a side-step would wreck the
    // traversal. The bend is a lateral nudge of at most `MAGNET_LATERAL`; leg advancement still keys on
    // cell centers (untouched above), so this can't trip the progress watchdog. Left active under a
    // powerup commit on purpose: a ≤48u step costs far under the bridge slack, and grabbing armour on
    // the quad walk is the whole point.
    let waypoint = match o.magnet {
        Some(item)
            if matches!(kind, Some(LinkKind::Walk | LinkKind::Step) | None)
                && !on_air
                && plat_hold.is_none()
                && bot.gate.errand.is_none()
                && bot.bhop.phase == bhop::Phase::Off
                && magnet_on_corridor(origin.xy(), waypoint.xy(), item.xy()) =>
        {
            item
        }
        _ => waypoint,
    };
    // Plat-wait timeout: keyed on the plat index (not the leg, which the 0.4s repath churn rebuilds),
    // give up on a lift that never descends — a camped one, or a targeted plat only its own trigger
    // lowers — by striking its ride link so this bot's A* diverts, then re-path.
    match plat_hold {
        Some(pi) => {
            if bot.plat_wait.map(|w| w.plat) != Some(pi) {
                bot.plat_wait = Some(PlatWait { plat: pi, since: now });
            } else if bot.plat_wait.is_some_and(|w| now - w.since > PLAT_WAIT_TIMEOUT) {
                let ride = bot.route[bot.route_pos..]
                    .iter()
                    .copied()
                    .find(|&l| graph.link_kind(l) == LinkKind::Plat && graph.plat_of_link(l) == Some(pi));
                if let Some(ride) = ride {
                    penalize_link(bot, ride, now);
                }
                bot.plat_wait = None;
                bot.route.clear();
                bot.repath_time = now;
            }
        }
        None => bot.plat_wait = None,
    }

    let hook_active = matches!(kind, Some(LinkKind::Hook)) || hooking;
    // Same for a rocket-jump leg: standing in stance and riding the blast arc must be exempt from the
    // stuck/progress watchdogs and the bhop veto, exactly like a hook leg.
    let rj_active = matches!(kind, Some(LinkKind::RocketJump)) || on_rj;
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
    let look_point = if vigil && bot.vigil.scan_point != Vec3::ZERO {
        // Standing vigil: sweep the eyes across the room (the scan point the aim spring pans to).
        // This drives the perception cone too (perception reads `bot.aim.angles`), so it's real scouting;
        // combat's `engage` still overrides the moment a target comes into sight.
        bot.vigil.scan_point
    } else if let Some(pi) = plat_hold {
        // Holding off a raised lift: watch it, so we notice it descend (and combat's `engage` still
        // overrides the instant a target comes into sight).
        let p = graph.plat(pi);
        Vec3::new(
            (p.fp_min.x + p.fp_max.x) * 0.5,
            (p.fp_min.y + p.fp_max.y) * 0.5,
            plat_status[pi].surface_z + 24.0,
        )
    } else if bot.route_pos + 2 < bot.route.len() {
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

    let goal_dist = (target_origin.xy() - origin.xy()).length();

    // Stuck detection. Suppressed mid-hook: standing in the throw stance, reeling, and riding the
    // parabola all look "stuck" to it, and a force-jump/repath there would wreck the traversal — the
    // hook driver's own per-phase timeouts are its stuck detection.
    let mut force_jump = false;
    if hook_active
        || rj_active
        || on_air
        || vigil
        || plat_hold.is_some()
        || (origin - bot.watchdog.stuck_origin).length() > STUCK_MOVE
    {
        bot.watchdog.stuck_origin = origin;
        bot.watchdog.stuck_since = now;
    } else if now - bot.watchdog.stuck_since > STUCK_TIME {
        // Force a jump to unwedge — but NOT toward a fatal edge. A bot stuck at a lava/pit lip (e.g.
        // wedged against a surcharged jump the router refuses to take) would otherwise force-jump
        // straight off it and burn. When the near-field sees a drop/hazard within a hop toward the
        // waypoint, hold the jump and let the penalize+repath below divert the route instead.
        let toward_edge = bot.near.as_ref().is_some_and(|nf| {
            let d = (waypoint.xy() - origin.xy()).normalize_or_zero();
            d.length_squared() > 0.5 && nf.edge_ahead(origin, Vec3::new(d.x, d.y, 0.0), STUCK_JUMP_LOOK) < STUCK_JUMP_LOOK
        });
        force_jump = !toward_edge;
        // Penalize the leg we're wedged on so the forced re-path actually *diverts* — without this
        // the deterministic A* hands back the identical route and the bot re-wedges every 0.7s.
        penalize_leg(bot, cur_leg, kind, now);
        bot.repath_time = now; // re-path next frame
        bot.watchdog.stuck_since = now;
    }

    // Path-progress watchdog: catches a bot that *is* moving (so the displacement detector above
    // stays satisfied) yet makes no headway toward the goal — orbiting a pillar, sliding along a
    // wall, riding a mis-linked jump back and forth. If the straight-line distance to the goal hasn't
    // improved by `PROGRESS_EPS` for `PROGRESS_STALL_TIME`, treat the current leg as failing: penalize
    // it and re-path. Suspended while hooking / on a committed speed-jump / riding a plat (all of which
    // legitimately hold or reverse XY progress for a while).
    let plat_leg = matches!(kind, Some(LinkKind::Plat));
    if !hook_active && !rj_active && !on_sj && !on_air && !plat_leg && !vigil {
        if progress_stalled(bot.watchdog.progress_best, bot.watchdog.progress_since, goal_dist, now) {
            penalize_leg(bot, cur_leg, kind, now);
            bot.route.clear();
            bot.repath_time = now;
            bot.watchdog.progress_best = goal_dist;
            bot.watchdog.progress_since = now;
        } else if goal_dist < bot.watchdog.progress_best - PROGRESS_EPS {
            bot.watchdog.progress_best = goal_dist;
            bot.watchdog.progress_since = now;
        }
    } else {
        // Keep the baseline current so a stall isn't falsely flagged the instant we resume.
        bot.watchdog.progress_best = goal_dist;
        bot.watchdog.progress_since = now;
    }

    // Bunnyhop policy verdicts — everything that needs game state is judged here; *when* each
    // verdict may apply in the hop cycle (engage hysteresis, mid-hop commitment, landing-only
    // disengage) is `bhop::Bhop::step`'s job. The entry runway bar is deliberately fixed:
    // the old `speed·0.9` bar rose as the bot gained speed and cut runs short mid-air.
    let runway_dist = runway(graph, &bot.route, bot.route_pos, origin);
    // Combat only vetoes bhop while it *owns the view* — the enemy is in sight (or lost a moment
    // ago), when the eyes must aim, not sweep a strafe. A mere Fight target being chased across
    // the map is navigation, and navigation bunnyhops; in FFA every bot always has a target, so
    // gating on target existence kept bhop permanently off. The grace here is deliberately much
    // shorter than combat's 2s corner-hold: on a small open FFA map sight contact is frequent,
    // and a long window suppresses hopping almost everywhere.
    const BHOP_COMBAT_GRACE: f32 = 0.5;
    let combat_view = enemy.is_some() && enemy_seen_time > 0.0 && now - enemy_seen_time < BHOP_COMBAT_GRACE;
    // On a navmesh cell flagged beside a fatal drop (a wall-hugging walkway over an open pit — a spiral
    // staircase's inner edge) drop to the walk: a fast bot carries off the inner edge at the corners,
    // where no bend holds it. The flag is precomputed per cell, so unlike the near-field it stays valid
    // while the bot is airborne mid-hop. Exempt a jump run-up (the leg at hand, or the next, is a jump)
    // — those need bhop speed to clear the gap; the drop there is the jump's landing, not a fall.
    let is_jump = |l: u32| matches!(graph.link_kind(l), LinkKind::JumpGap | LinkKind::DoubleJump | LinkKind::SpeedJump);
    let jump_at_hand = cur_leg.is_some_and(&is_jump) || bot.route.get(bot.route_pos + 1).is_some_and(|&l| is_jump(l));
    let on_ledge = graph.is_ledge(bot_cell) && !jump_at_hand;
    let bhop_veto = !host.cvar_bool(c"rtx_bot_bhop")
        || combat_view
        || in_water // can't hop while swimming — the engine's pmove turns jumps into swim strokes
        || hook_active
        || rj_active
        // Spectating: a bhop cmd would overwrite the view yaw in `emit` and clobber the watch —
        // and a spectator strolling the stands shouldn't be bunnyhopping anyway.
        || watch_point.is_some()
        || bot.gate.errand.is_some()
        || bot.grenade.phase != GrenadePhase::Idle
        || on_ledge;
    // The banded planner's intent for this run: a band ≥ 1 on the current or next leg means the
    // route was planned to carry speed here, so admit bhop even on a short leg (the goal-distance
    // gates below exist to avoid hopping on trivial approaches — the plan overrides that judgment)
    // and tell the controller to hold the chain across the waypoint rather than disengage per leg.
    let planned_band = bot.route_bands.get(bot.route_pos).copied().unwrap_or(0);
    // An ascending Walk/Step leg (target more than a walk's worth above the source, i.e. a stair
    // riser) just ahead: a human runs up stairs, so don't let a planned carry hold the hop chain up
    // them — `runway`'s climb stop keeps *entry* off stairs, and this keeps *carry* from overriding it.
    let leg_ascends = |leg: u32| {
        matches!(graph.link_kind(leg), LinkKind::Walk | LinkKind::Step)
            && graph.cell_origin(graph.link_target(leg)).z - graph.cell_origin(graph.link_source(leg)).z > 8.0
    };
    let ascent_ahead =
        cur_leg.is_some_and(&leg_ascends) || bot.route.get(bot.route_pos + 1).is_some_and(|&l| leg_ascends(l));
    let carry = (planned_band >= 1 || bot.route_bands.get(bot.route_pos + 1).copied().unwrap_or(0) >= 1)
        && !ascent_ahead;
    let bhop_entry = !final_leg
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && (goal_dist > 300.0 || planned_band >= 1)
        && runway_dist >= bhop::RUNWAY_ENGAGE
        // Run up first: don't start the hop cycle from a standstill — accelerate on the ground until
        // we're actually moving, then leap into the circle-jump (a human never hops from a stop).
        && speed >= bhop::RUN_UP_SPEED;
    // Lenient continuation gate for taking *another* hop from a landing: leg kinds churn as the
    // route advances, and a run in progress shouldn't be dumped by the stricter entry conditions.
    // But never sustain the chain up an ascending stair run — `runway`'s climb stop keeps *entry* off
    // stairs, yet a chain carried onto a stairway (a wall-hugging spiral, say) would otherwise keep
    // hopping and weave off the treads. Drop to the walk, whose near-field glide tracks the steps.
    let bhop_sustain = matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && (goal_dist > 150.0 || planned_band >= 1)
        && !ascent_ahead;
    // Ground zigzag: a corridor too short for a hop ([`bhop::RUNWAY_ENGAGE`]) but straight and long
    // enough ([`bhop::ZIGZAG_ENGAGE`]) to gain speed from the circle-strafe alone. The controller
    // hands off to the hop cycle if `bhop_entry` opens up mid-run, and `bhop_veto` (which includes
    // `!rtx_bot_bhop`) still gates it, so this is purely a sub-toggle on the same controller.
    let zigzag_ok = host.cvar_bool(c"rtx_bot_zigzag")
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && !final_leg
        && goal_dist > 150.0
        && runway_dist >= bhop::ZIGZAG_ENGAGE
        && !on_ledge;
    // A speed-jump leg is a *committed* bhop run-up + leap: engage bhop unconditionally (the link is a
    // pre-verified runway) and track it so the route stays frozen. Latch/clear `sj_leg` on the leg.
    let mut sj_active =
        matches!(kind, Some(LinkKind::SpeedJump)) && host.cvar_bool(c"rtx_bot_bhop") && !hook_active && !rj_active;
    if sj_active {
        if bot.sj.map(|c| c.leg) != cur_leg {
            bot.sj = cur_leg.map(|leg| Commit { leg, since: now });
        }
        // Watchdog: the route is frozen mid-leg, so if the run-up stalls (blocked, shoved, never
        // built speed) abandon it and re-path rather than wedging on the runway forever. Penalize the
        // leg so the deterministic A* actually diverts instead of handing back the same run-up.
        if bot.sj.is_some_and(|c| now - c.since > 4.0) {
            penalize_leg(bot, cur_leg, kind, now);
            bot.sj = None;
            bot.route.clear();
            bot.repath_time = now;
            sj_active = false;
        }
    } else if bot.sj.is_some() {
        bot.sj = None;
    }
    // Fallback latch for a jump leg created by this frame's repath. Ordinarily `prearm_traversal`
    // installed it before objective resolution; this closes the first-frame route-build case.
    let on_jump_leg = matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump));
    if on_jump_leg && bot.air.map(|c| c.leg) != cur_leg {
        bot.air = cur_leg.map(|leg| AirCommit {
            leg,
            target: graph.link_target(leg),
            since: now,
            airborne: !on_ground,
        });
    }
    if let Some(committed) = bot.air {
        match air_commit_decision(on_ground, committed.airborne, now - committed.since) {
            AirRelease::Keep => {}
            AirRelease::Land => {
                let target = graph.cell_origin(committed.target);
                let on_target = (origin.xy() - target.xy()).length() <= 2.0 * ARRIVE_RADIUS;
                if !on_target {
                    penalize_leg(bot, Some(committed.leg), Some(graph.link_kind(committed.leg)), now);
                    bot.route.clear();
                    bot.repath_time = now;
                }
                bot.air = None;
            }
            AirRelease::Timeout => {
                penalize_leg(bot, Some(committed.leg), Some(graph.link_kind(committed.leg)), now);
                bot.air = None;
                bot.route.clear();
                bot.repath_time = now;
            }
        }
    }
    // "Don't leap to your death": if we somehow reach the takeoff edge too slow to clear the gap,
    // hold the jump (keep accelerating) rather than launching short into it.
    let sj_takeoff = cur_leg
        .and_then(|l| graph.speed_jump_of_link(l))
        .map(|tr| (tr.takeoff, tr.v_req));
    // A curl speed jump carries a nonzero air-curl gain; a straight one carries 0 (keeps the slalom).
    let sj_curl_gain = cur_leg.and_then(|l| graph.speed_jump_of_link(l)).map(|tr| tr.curl_gain).unwrap_or(0.0);
    let sj_curl = sj_active && sj_curl_gain > 0.0;
    // Signed along-corridor distance from the bot to a curl's takeoff (>0 behind the lip, <0 past it):
    // the run-up direction is the link's `from`→takeoff line. Used to trigger the leap on crossing the
    // takeoff *line* (not a radial ball the weave can skirt into a U-turn) and to gate the run-up aim.
    let sj_progress: Option<f32> = if sj_curl {
        if let (Some((takeoff, _)), Some(leg)) = (sj_takeoff, cur_leg) {
            let dir = (takeoff.xy() - graph.cell_origin(graph.link_source(leg)).xy()).normalize_or_zero();
            Some((takeoff.xy() - origin.xy()).dot(dir))
        } else {
            None
        }
    } else {
        None
    };
    // Curl too-slow abort: the bhop takeoff regime leaps a curl *unconditionally* at the lip, so if the
    // bot won't build `v_req` by the lip from where it is now (shoved, blocked, or dropped onto the leg
    // slow by a repath), bail the leg here rather than leap short into the pit. Predict the lip speed
    // from the current state via the ground-prestrafe oracle; abort (penalize + repath) when it falls
    // well short. Edge-avoidance — restored the moment `sj_active` clears — then keeps the bot off the
    // ledge. Left running (the run-up recovers a low *early* speed over the remaining distance).
    if let (true, Some((_, v_req)), Some(progress)) = (sj_curl, sj_takeoff, sj_progress) {
        let cv = |n: &std::ffi::CStr, d: f32| {
            let x = host.cvar(n);
            if x > 0.0 { x } else { d }
        };
        let predicted = crate::navmesh::prestrafe_delivered_from(
            speed,
            progress.max(0.0),
            cv(c"sv_accelerate", 10.0),
            cv(c"sv_maxspeed", 320.0),
            cv(c"sv_friction", 4.0),
            cv(c"sv_stopspeed", 100.0),
        );
        if predicted < v_req * 0.85 {
            penalize_leg(bot, cur_leg, kind, now);
            bot.sj = None;
            bot.route.clear();
            bot.repath_time = now;
            sj_active = false;
        }
    }
    let sj_hold = sj_active && {
        match sj_takeoff {
            Some((takeoff, v_req)) => {
                let to_edge = takeoff.xy() - origin.xy();
                (to_edge.length() < 48.0 || to_edge.dot(v_xy) < 0.0) && speed < v_req * 0.9
            }
            None => false,
        }
    };

    // Near-field ensure (see [`crate::nearfield`]): build/refresh the fine 8u clearance grid whenever
    // the bot is doing GROUNDED locomotion on a walk/step/approach leg — walking, zigzagging, OR
    // bhopping over ground — so the *fast-movement* hop bearing below can be steered clear of drop
    // edges and walls, not just the slow walk. Excluded on a committed ballistic arc (speed/rocket
    // jump, hook, launched air-commit), where the flight must commit to its landing and must not be
    // nudged. Built on grounded frames (the seed needs footing); the cache survives the brief airborne
    // phase of a hop (which rises well under the recenter height), so the bearing stays aware mid-hop.
    // The slow-walk edge margin below re-reads this same grid.
    let nf_locomotion = !on_air
        && !sj_active
        && !hook_active
        && !rj_active
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step) | None);
    let nf_active = nf_locomotion && host.cvar_bool(c"rtx_bot_nearfield");
    if nf_active && on_ground {
        if let Some(bsp) = bsp {
            let key = nearfield_gates(graph, gate_closed, origin).fold(0u32, |k, gi| k | (1u32 << gi.min(31)));
            if bot.near.as_ref().is_none_or(|nf| !nf.valid_for(origin, key)) {
                let boxes: Vec<(Vec3, Vec3)> = nearfield_gates(graph, gate_closed, origin)
                    .map(|gi| { let g = graph.gate(gi); (g.closed_min, g.closed_max) })
                    .collect();
                // Liquid oracle: flush lava/slime is invisible to the clip hull, so classify it from the
                // render hull's `pointcontents` (our own parsed BSP — no syscall). Gated on the map having
                // any hazard cell at all, so the dry maps (the norm) pay nothing. A walkable column over
                // lava becomes a repelling `Col::Hazard`, so the walk margin and hop bearing steer off it.
                let has_haz = graph.has_hazards();
                let (lava, slime) = (crate::bsp::CONTENTS_LAVA, crate::bsp::CONTENTS_SLIME);
                let is_hazard = |p: Vec3| has_haz && { let c = bsp.pointcontents(p); c == lava || c == slime };
                bot.near = Some(nearfield::NearField::build(&|p| bsp.is_solid(p), &is_hazard, origin, &boxes, key));
            }
        }
    }

    // Drive the hop-cycle controller (see `bhop::Bhop`). On a speed jump the runway is the
    // run-up to the takeoff edge and the bearing aims straight at the landing so the leap goes
    // across the gap; otherwise steer toward the look-ahead corridor point (smoother than the 32u
    // next cell) with as much straight-ish corridor as the route offers. `mut` so the hazard-edge
    // brake below can null the hop and drive a reverse wish through `emit` instead.
    let mut bhop_cmd = {
        let dt = frametime.clamp(0.001, 0.05);
        let accel = host.cvar(c"sv_accelerate");
        let maxspeed = host.cvar(c"sv_maxspeed");
        let env = bhop::Env {
            dt,
            accel: if accel > 0.0 { accel } else { 10.0 },
            maxspeed: if maxspeed > 0.0 { maxspeed } else { 320.0 },
        };
        // A committed speed jump aims at its gap; otherwise steer toward the racing-line look-ahead
        // (race mode, when a line exists) or a *speed-scaled* corridor look-ahead — ~0.6 s of travel
        // ahead (clamped 96–448u) so a fast bot's bearing anticipates the corridor far enough to
        // start curving, rather than chasing the fixed ~2-legs `look_point` it has already overrun.
        let bhop_look = corridor_point(graph, &bot.route, bot.route_pos, origin, (speed * 0.6).clamp(96.0, 448.0));
        let to_wp = waypoint.xy() - origin.xy();
        let ahead = match race_line_ahead {
            Some(lp) if !sj_active => lp.xy() - origin.xy(),
            // On a speed jump the run-up aims at the *takeoff* (follow the corridor to the lip), and
            // only once airborne does the bearing swing to the *landing* — so a curl jump (run-up and
            // leap not collinear) tracks its corridor instead of cutting across it and off the edge.
            // For a straight speed jump takeoff and target are collinear, so this is a no-op.
            _ if sj_active => {
                let aim = match (sj_takeoff, sj_progress) {
                    // Curl run-up: aim at the takeoff (follow the corridor) while still behind the lip —
                    // grounded *or* briefly airborne (a bumped or carried-airborne entry) — so it never
                    // curls toward the offset landing while still over the run-up and pulls off the edge.
                    (Some((takeoff, _)), Some(p)) if p > bhop::LIP_REACH => takeoff,
                    // Straight speed jump on the ground: aim at the takeoff (collinear → no-op vs landing).
                    (Some((takeoff, _)), None) if on_ground => takeoff,
                    _ => waypoint,
                };
                aim.xy() - origin.xy()
            }
            // Normal corridor follow: aim at the speed-scaled look-ahead — but only while the straight
            // line to it stays on near-field-clear floor. On a wall-hugging spiral staircase the far
            // look-ahead wraps around the inner curve, so the chord to it cuts across the open centre;
            // certifying it (as the slow walk-glide path does) and otherwise dropping back to the near
            // waypoint keeps the fast bot on its current flight instead of steering off the inner edge.
            // Past the grid the chord passes — the route out there is trusted; the veto fires only on a
            // drop the near-field actually sees, so open corridors keep the full anticipatory look-ahead.
            _ => {
                let look_clear = bot
                    .near
                    .as_ref()
                    .filter(|_| nf_active)
                    .is_none_or(|nf| nf.chord_open(origin, bhop_look));
                if look_clear { bhop_look.xy() - origin.xy() } else { to_wp }
            }
        };
        let dir = if ahead.length() > 8.0 { ahead } else { to_wp };
        // Near-field-aware hop bearing: bend the heading off nearby drop edges and walls so a fast bot
        // (bhop or zigzag) holds the walkable line — e.g. tracking up a staircase — instead of weaving
        // straight off the edge toward the raw xy goal. `nf_active` already excludes committed jumps,
        // and a speed jump takes the `sj_active` branch above (so a gap leap still commits to its
        // landing). Inert on open ground, where the near-field push is zero.
        let dir = match bot.near.as_ref().filter(|_| nf_active).and_then(|nf| nf.steer_push(origin)) {
            Some(push) => {
                let bent = dir.normalize_or_zero() + push.xy() * NEARFIELD_BHOP_WEIGHT;
                if bent.length() > 1e-3 { bent * dir.length() } else { dir }
            }
            None => dir,
        };
        let bearing = yaw_of(dir);
        let bhop_runway = match (sj_takeoff, sj_progress) {
            // Curl: signed along-corridor distance to the takeoff (past-lip goes negative → leap).
            (_, Some(p)) => p,
            // Straight speed jump: radial distance to the takeoff edge (collinear run-up).
            (Some((takeoff, _)), None) if sj_active => (takeoff.xy() - origin.xy()).length(),
            _ => runway_dist,
        };
        // Forward wall probe: how far the bot can fly straight ahead before a wall — one hull trace
        // along the velocity out to a hop's flight. Feeds the controller's "don't leap at a wall,
        // carve when flying at one" logic. `INFINITY` (open) when there's no BSP, we're barely moving,
        // or the hop cycle isn't engaged/about to engage — so idle and plain-walking bots never trace.
        let clear = match bsp {
            Some(bsp) if speed > 1.0 && (bot.bhop.phase != bhop::Phase::Off || bhop_entry) => {
                let d = (speed * bhop::T_HOP).max(64.0);
                let end = origin + (v_xy.normalize_or_zero() * d).extend(0.0);
                let wall = bsp.hull1_trace(origin, end).fraction * d;
                // The hull trace sees only walls: a bot flying at the open centre of a wall-hugging
                // walkway (a spiral staircase's inner edge) traces clear and hops over the void. The
                // near-field sees the drop — cap `clear` at the lip so the controller carves/brakes on
                // the ground at the edge (turning far faster than in the air) instead of leaping off it.
                let edge = bot.near.as_ref().filter(|_| nf_active).map_or(d, |nf| nf.edge_ahead(origin, v_xy.extend(0.0), d));
                wall.min(edge)
            }
            _ => f32::INFINITY,
        };
        let phase_was = bot.bhop.phase;
        let cmd = bot.bhop.step(
            &bhop::Input {
                v_xy,
                on_ground,
                bearing,
                runway: bhop_runway,
                eligible: bhop_entry,
                zigzag: zigzag_ok,
                sustain: bhop_sustain,
                veto: bhop_veto,
                committed: sj_active,
                carry,
                hold_jump: sj_hold,
                // The takeoff regime (hold ground prestrafe to the lip, leap once) is only for *curl*
                // jumps, which need a run-up the ground circle-strafe builds. A straight speed jump keeps
                // the pre-existing hop-chain takeoff — its air-strafe runway can exceed the ~490 prestrafe
                // ceiling, which the hold-to-lip regime would cap it below. So gate on the curl flag.
                takeoff_speed: match sj_takeoff {
                    Some((_, v_req)) if sj_active && sj_curl_gain > 0.0 => v_req,
                    _ => 0.0,
                },
                // Curl only jumps flagged as curls (straight speed jumps keep the slalom untouched). The
                // cvar, when set, overrides the link's baked gain for live tuning of the curl arc.
                curl_gain: if sj_active && sj_curl_gain > 0.0 {
                    let cv = host.cvar(c"rtx_jump_curl_gain");
                    if cv > 0.0 { cv } else { sj_curl_gain }
                } else {
                    0.0
                },
                clear,
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
    let yaw = yaw_of(to_wp);
    let mut angles = Vec3::new(0.0, yaw, 0.0);

    // Nav look target: eyes on the look-ahead point down the corridor (combat/gate may override
    // below). When the look point is basically on top of us (standing on the goal/waypoint), both it
    // and the steering yaw degenerate — `atan2` on a near-zero vector jitters frame to frame, which is
    // the source of the on-the-spot twitch — so hold the current smoothed view instead of chasing
    // noise. 48u guard (not 8) so a bot idling at a pickup doesn't re-solve a garbage angle.
    let eye = origin + Vec3::new(0.0, 0.0, 22.0);
    let to_look = look_point - eye;
    let mut look = if to_look.xy().length() > 48.0 {
        angles_to(eye, look_point)
    } else if dist > 8.0 {
        angles // steering yaw is still meaningful — look where we're walking
    } else if bot.aim.angles != Vec3::ZERO {
        bot.aim.angles // standing still on the point — hold the current view, don't snap to yaw 0
    } else {
        v_angle
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
    let hook_engaged = bot.hook.phase != HookPhase::Idle;
    let hook_lock = matches!(
        bot.hook.phase,
        HookPhase::Flight | HookPhase::Reel | HookPhase::Ballistic
    );

    // Rocket-jump leg driver: walk to the launch cell with the RL out, settle the aim on the solved
    // fire angles, jump, fire after the solved delay, ride the blast arc onto the ledge. Same shape as
    // the hook driver — a snapshot in, an `RjDrive` out that the code below applies.
    let rj = rj::drive_rj(
        graph,
        bot,
        rj::RjCtx {
            rj_active,
            cur_leg,
            enemy,
            chasing,
            now,
            weapon,
            origin,
            on_ground,
            attack_finished,
            weapons_hot,
            has_rl,
            ammo_rockets,
            health,
            armortype,
            armorvalue,
            quad,
            knobs: s.rj_knobs,
        },
    );
    let rj_engaged = bot.rj.phase != RjPhase::Idle;
    let rj_lock = matches!(bot.rj.phase, RjPhase::Rise | RjPhase::Ballistic);

    if let Some(t) = hook.look_target {
        if (t - eye).xy().length() > 1.0 {
            look = angles_to(eye, t);
        }
    }
    // Rocket-jump look: Stance/Rise hold the solved fire *angles* directly (the shot flies along the
    // view); Ballistic looks at the landing *point* (reprojected like the hook's).
    if let Some(a) = rj.look_target_angles {
        look = a;
    } else if let Some(t) = rj.look_target {
        if (t - eye).xy().length() > 1.0 {
            look = angles_to(eye, t);
        }
    }
    // Audience watch (arena Spectate): eyes on the fighter the mode chose — already LOS-validated
    // there and held ~1-2s. Post-hoc like the hook/rj overrides, so bhop steering and the route
    // look-ahead stay untouched; the aim spring in `emit` turns it into a human pan and perception
    // follows through `bot.aim.angles`. Same 48u degenerate-angle guard as the nav look. Audience bots
    // have no grapple/RL, so the hook/rj guard is belt-and-braces.
    if !hook_engaged && !rj_engaged {
        if let Some(t) = watch_point {
            if (t - eye).xy().length() > 48.0 {
                look = angles_to(eye, t);
            }
        }
    }

    let (mut forward, mut side, mut buttons, mut impulse) = (0, 0, 0, 0);
    // Politely stop short only when tailing a human or roaming (`Objective::polite`). Everything
    // else walks all the way in: an item pickup needs its touch to fire, a race checkpoint is a
    // hull-sized touch box, and when hunting an enemy stopping short would halt the bot 64u out
    // — e.g. right at a door between it and its target (the combat layer manages the actual
    // fighting distance once it has line of sight). `polite` is never set alongside a chase or
    // a Fight intent, so it alone decides.
    // Arrival slowdown: when a grounded Walk/Step leg is about to hand off to a sharply-turning next
    // leg and continuing straight past the waypoint would run off a ledge, ease the wish down as we
    // close in so we arrive slow enough to make the turn instead of overshooting the lip. Double-gated
    // — a sharp turn AND a real drop straight ahead — so flat corners and the grid's 45° zigzag keep
    // full speed, and a thin balance path (no turn, or floor continuing past the waypoint) is untouched.
    let wish_scale = {
        let eligible = on_ground
            && !bhop_active
            && !sj_active
            && !hook_engaged
            && !rj_engaged
            && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
            && dist < TURN_SLOW_RADIUS;
        let cur_dir = to_wp.normalize_or_zero();
        let next_dir = bot
            .route
            .get(bot.route_pos + 1)
            .map(|&nl| (graph.cell_origin(graph.link_target(nl)).xy() - waypoint.xy()).normalize_or_zero());
        let sharp = cur_dir != Vec2::ZERO
            && next_dir.is_some_and(|nd| nd != Vec2::ZERO && cur_dir.dot(nd) < TURN_SLOW_COS);
        let over_ledge = eligible
            && sharp
            && bsp.is_some_and(|bsp| {
                let feet = waypoint - Vec3::new(0.0, 0.0, ORIGIN_TO_FEET);
                crate::hazard::ledge_ahead(&|p| bsp.is_solid(p), feet, Vec3::new(cur_dir.x, cur_dir.y, 0.0))
            });
        let turn = if over_ledge { (dist / TURN_SLOW_RADIUS).clamp(TURN_SLOW_MIN, 1.0) } else { 1.0 };
        // Careful-ledge cap: on a flagged ledge cell (an open-cored spiral's inner edge) hold the walk
        // to `rtx_bot_ledgecap` for the *whole* run — the turn slowdown above only bites inside the 96u
        // approach, too late to bleed a full-speed straight's momentum before the corner's lip. `on_ledge`
        // already excludes jump run-ups. The wish is `MOVE_SPEED` (800, 2.5× maxspeed) so a scale of
        // cap/800 caps the post-clamp ground speed at `cap` u/s. `min` so a sharp ledge corner slows further.
        let ledge = if on_ledge {
            let cap = host.cvar(c"rtx_bot_ledgecap");
            if cap > 0.0 { (cap / MOVE_SPEED).min(1.0) } else { 1.0 }
        } else {
            1.0
        };
        turn.min(ledge)
    };
    // Edge margin: on a grounded walk/step (or final-approach) leg while NOT bhopping, steer the
    // slow-walk wish away from a wall or drop beside the line of travel — the inner edge of an
    // open-cored spiral, a catwalk lip, a doorframe — instead of drifting into it while homing on the
    // next cell centre (which sits on the grid, up to a hull-width from the true edge). Bhop's own
    // bearing was made near-field-aware above, so this stays gated to the non-bhop walk.
    //
    // With `rtx_bot_nearfield` on, this reads the fine (8u) near-field clearance grid ensured before
    // the bhop block (see `nearfield`): wall-aware, self-cancelling through doorways/thin beams. When
    // the grid is off, absent, or the bot's been shoved off its own field, it falls back to the
    // drop-only `hazard::edge_bias` probe (Walk/Step only, as before — a final-approach `None` leg got
    // no push in that path historically).
    let nf_ground = on_ground
        && !on_air
        && bot.bhop.phase == bhop::Phase::Off
        && !bhop_active
        && !sj_active
        && !hook_engaged
        && !rj_engaged
        && dist > ARRIVE_RADIUS;
    let edge_push = if nf_ground {
        let near_push = nf_active.then(|| bot.near.as_ref()?.steer_push(origin)).flatten();
        near_push.unwrap_or_else(|| {
            // No field → today's drop-only probe, which only runs on a real Walk/Step leg.
            if matches!(kind, Some(LinkKind::Walk | LinkKind::Step)) {
                bsp.map_or(Vec3::ZERO, |bsp| {
                    let feet = origin - Vec3::new(0.0, 0.0, ORIGIN_TO_FEET);
                    let travel = if speed > 40.0 { v_xy.normalize_or_zero() } else { to_wp.normalize_or_zero() };
                    crate::hazard::edge_bias(&|p| bsp.is_solid(p), feet, Vec3::new(travel.x, travel.y, 0.0))
                })
            } else {
                Vec3::ZERO
            }
        })
    } else {
        Vec3::ZERO
    };

    // Glide look-ahead: on a grounded walk/step leg, if the near-field certifies a straight chord to a
    // point ~96u down the corridor stays on clear floor, aim the feet at *that* instead of the raw 32u
    // next cell — straightening the grid's constant 45° zigzag. Everything else still keys on the raw
    // waypoint (leg advancement, `wish_scale`, the magnet, the watchdogs): the chord follows the leg
    // polyline, so it passes within `ARRIVE_RADIUS` of each cell centre, exactly the magnet's argument.
    // Off on the final approach (home straight on the target) and whenever the chord isn't clear.
    let heading = {
        let want_glide = nf_ground
            && nf_active
            && host.cvar_bool(c"rtx_bot_glide")
            && matches!(kind, Some(LinkKind::Walk | LinkKind::Step));
        let glide = want_glide.then_some(bot.near.as_ref()).flatten().and_then(|nf| {
            let g = corridor_point(graph, &bot.route, bot.route_pos, origin, NEAR_GLIDE_AHEAD);
            nf.chord_clear(origin, g, NEAR_GLIDE_MARGIN).then_some(g)
        });
        glide.map_or(to_wp, |g| g.xy() - origin.xy())
    };

    let close_enough = final_leg && polite && dist <= POLITE_DIST;
    if !close_enough {
        let (fwd, right, _) = angle_vectors(angles);
        let dir = (Vec3::new(heading.x, heading.y, 0.0).normalize_or_zero() + edge_push * EDGE_BIAS_WEIGHT).normalize_or_zero();
        forward = (fwd.dot(dir) * MOVE_SPEED * wish_scale) as i32;
        side = (right.dot(dir) * MOVE_SPEED * wish_scale) as i32;
    }
    // Jump only while on the ground: QW pmove jumps once per press and needs the button
    // released (airborne) before it'll fire again. Gating on ground state pulses it correctly,
    // so a jump that falls short is retried on the next landing instead of the bot getting
    // stuck holding +jump against a ledge.
    // Curl-jump knobs for plain jump legs (see cvars): a run-up speed gate on the takeoff, plus the
    // in-air curl hold-fraction and gain applied below. All default to today's behavior.
    let jump_maxspeed = {
        let m = host.cvar(c"sv_maxspeed");
        if m > 0.0 { m } else { 320.0 }
    };
    let jump_runup = host.cvar(c"rtx_jump_runup").max(0.0);
    let curl_hold = host.cvar(c"rtx_jump_curl_hold").clamp(0.0, 0.95);
    let curl_gain = {
        let g = host.cvar(c"rtx_jump_curl_gain");
        if g > 0.0 { g } else { bhop::AIR_CORRECT_GAIN_DEFAULT }
    };
    // Run-up gate: on a plain jump leg, hold the takeoff jump until the bot carries speed *toward the
    // waypoint* (`jump_runup · maxspeed`), so it leaves the lip moving instead of jumping from a
    // standstill and air-accelerating into a stub arc. Escapes at the lip and when disabled keep it from
    // wedging; `force_jump` (the stuck detector) and the bhop controller bypass it too.
    let runup_ok = jump_runup_ok(v_xy, to_wp, dist, jump_runup, jump_maxspeed);
    if on_ground
        && (force_jump
            || bhop_cmd.is_some_and(|c| c.jump)
            || (matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump)) && runup_ok))
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
    if let Some(errand) = bot.gate.errand {
        let gi = errand.index;
        let g = graph.gate(gi);
        let at_button =
            bot.route_pos >= bot.route.len() || (origin.xy() - graph.cell_origin(g.button_cell).xy()).length() < 40.0;
        if at_button {
            angles = angles_to(eye, g.aim);
            let (pitch, yaw) = (angles.x, angles.y);
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

    // Unified air steering (always on): a yaw-synced air-strafe wish toward a landing point, in
    // **world space** so the wish actually turns the velocity — a straight wish the 30-ups air-accel
    // cap all but ignores — while the eyes keep smoothing toward the target through the normal aim
    // spring (no raw-view channel, so the strafe never twitches the view). `None` when we're basically
    // on top of the target (keep whatever wish we had). See [`bhop::air_correct`].
    let air_wish = |target: Vec3, gain: f32| -> Option<Vec3> {
        let to = target.xy() - origin.xy();
        (to.length() > 24.0).then(|| {
            let dt = frametime.clamp(0.001, 0.05);
            let accel = host.cvar(c"sv_accelerate");
            let maxspeed = host.cvar(c"sv_maxspeed");
            let a_max = bhop::air_accel_max(
                if accel > 0.0 { accel } else { 10.0 },
                if maxspeed > 0.0 { maxspeed } else { 320.0 },
                dt,
            );
            let s = bhop::air_correct(v_xy, yaw_of(to), a_max, dt, gain);
            let w = bhop::wishdir_fs(s.view_yaw, s.forward, s.side);
            Vec3::new(w.x, w.y, 0.0) * MOVE_SPEED
        })
    };
    // Airborne on a plain jump leg: ride the arc toward the landing (the pinned waypoint — the
    // `on_air` gate keeps it on the link target) with the air-strafe wish. `look` stays as steered
    // above, so the eyes pan smoothly toward the landing while the strafe curves the trajectory.
    // Curl-hold: a jump link certifies only the straight source→target center line, but the bot took
    // off offset and homing back onto the target can sweep the arc into an edge wall. For the first
    // `curl_hold` fraction of the gap, hold the takeoff heading (steer along our own velocity — an
    // inert coast) so the near wall is cleared, then curl onto the target at `curl_gain`.
    if on_air && !on_ground {
        let held = curl_hold > 0.0
            && cur_leg.is_some_and(|leg| {
                let src = graph.cell_origin(graph.link_source(leg)).xy();
                let tgt = graph.cell_origin(graph.link_target(leg)).xy();
                let done = 1.0 - (tgt - origin.xy()).length() / (tgt - src).length().max(1.0);
                done < curl_hold
            });
        let wish = if held {
            air_wish(origin + Vec3::new(v_xy.x, v_xy.y, 0.0), curl_gain)
        } else {
            air_wish(waypoint, curl_gain)
        };
        if let Some(w) = wish {
            move_world = w;
        }
    }

    // Hazard-edge brake: if the near-field sees a fatal drop or lava edge close ahead along the
    // *velocity* — nearer than the bot can bleed its speed before the lip — hard-brake to a stop rather
    // than sliding or hopping into it. Unlike the geometric ledge brake below this is hazard-aware
    // (catches flush lava the clip hull can't see) and fires while *bhopping* too: a fast bot's
    // momentum, not its wish direction, is what carries it off, so the near-field bearing bend and the
    // leap-suppression above don't stop it — the speed itself must go. The hop is nulled for the frame
    // so the reverse wish actually drives (`emit` ignores `move_world` while a hop is). Off during a
    // jump run-up (`jump_at_hand`) — the bot needs that speed to clear the gap — and inert on open
    // ground, where `edge_ahead` finds no edge. `nf_active` already limits it to grounded walk/step/
    // approach legs (a Drop leg descends; a jump leg leaps), and the near-field grid is built there.
    if nf_active && on_ground && !jump_at_hand && speed > LEDGE_MIN_SPEED {
        if let Some(nf) = bot.near.as_ref() {
            let vdir = v_xy.normalize_or_zero();
            let stop = (speed * BRAKE_REACT + speed * speed / (2.0 * BRAKE_DECEL)).clamp(nearfield::NEAR_RES, BRAKE_MAX_LOOK);
            let dir3 = Vec3::new(vdir.x, vdir.y, 0.0);
            if nf.edge_ahead(origin, dir3, stop + nearfield::NEAR_RES) <= stop {
                move_world = -dir3 * MOVE_SPEED;
                bhop_cmd = None;
            }
        }
    }

    // Ledge brake: a grounded bot on a Walk/Step leg whose *velocity* has drifted well off the corridor
    // to its waypoint (an overshot corner — e.g. run straight at a stair side) and is one stride from
    // running off the floor: kill the wish and thrust backward to stop before the lip. After the
    // navmesh's `ground_along` fix an *aligned* Walk/Step leg always has floor under it, so a drop
    // along velocity is unintended; and balancing along a thin wall-top keeps velocity aligned to the
    // waypoints, so the misalignment gate keeps this dead there. Dead too while airborne, bhopping,
    // speed-/rocket-jumping, or hooking — those own their motion (and the hook/rj overrides below win).
    if let Some(bsp) = bsp {
        let braking = on_ground
            && !on_air
            && bot.bhop.phase == bhop::Phase::Off
            && !bhop_active
            && !sj_active
            && !hook_engaged
            && !rj_engaged
            && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
            && speed > LEDGE_MIN_SPEED;
        if braking {
            let vdir = v_xy.normalize_or_zero();
            let aligned = vdir.dot(to_wp.normalize_or_zero()) >= LEDGE_ALIGN_COS;
            let vdir3 = Vec3::new(vdir.x, vdir.y, 0.0);
            let feet = origin - Vec3::new(0.0, 0.0, ORIGIN_TO_FEET);
            if !aligned && crate::hazard::ledge_ahead(&|p| bsp.is_solid(p), feet, vdir3) {
                move_world = -vdir3 * MOVE_SPEED;
            }
        }
    }

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

    // Rocket-jump override: walk to the launch cell (Stance), stand and hold the aim (Rise), or ride
    // the arc with the world-space air-strafe wish toward the landing (Ballistic — the same in-flight
    // correction as a plain jump leg, curving the blast arc onto the target). The jump itself is
    // pressed post-spring in `emit` (via `rj.jump_ready`); the rocket fires on the driver's `rj.fire`.
    if rj_engaged {
        move_world = match rj.approach {
            _ if rj.stand => Vec3::ZERO,
            Some(src) => Vec3::new(src.x - origin.x, src.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED,
            None => rj
                .air_correct
                .and_then(|t| air_wish(t, bhop::AIR_CORRECT_GAIN_DEFAULT))
                .unwrap_or(Vec3::ZERO),
        };
        buttons &= !BUTTON_JUMP; // the launch jump is pressed only via `emit`'s post-spring gate
        if rj.select {
            impulse = IMPULSE_ROCKET;
        }
        if rj.fire {
            buttons |= BUTTON_ATTACK;
        }
    }

    // Bundle the frame's decisions into one command for the combat/grenade overlays to mutate.
    let cmd = BotCmd { look, move_world, buttons, impulse, shot: None };

    // Traversal-critical legs lock out the combat/grenade overlays: `engage` owns movement and
    // clears +jump, which cancels the planner's route if done mid gap/double/speed jump.
    let traversal_lock = hook_lock
        || rj_lock
        || on_air
        || matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump | LinkKind::SpeedJump));
    let overlays_ok = !hook_engaged && !rj_engaged && !bhop_active && !traversal_lock;
    SteerOut { cmd, bhop_cmd, hook, rj, traversal_lock, overlays_ok }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jump_runup_gate_wants_speed_toward_the_waypoint() {
        let fwd = Vec2::new(1.0, 0.0);
        // Standstill, far from the lip → blocked (no useless pogo).
        assert!(!jump_runup_ok(Vec2::ZERO, fwd, 200.0, 0.5, 320.0));
        // At the lip (< JUMP_NOW_DIST) → must jump now, whatever the speed.
        assert!(jump_runup_ok(Vec2::ZERO, fwd, 39.0, 0.5, 320.0));
        // Running toward the waypoint at 200 ups (> 0.5·320 = 160) → allowed.
        assert!(jump_runup_ok(Vec2::new(200.0, 0.0), fwd, 200.0, 0.5, 320.0));
        // Fast but perpendicular (no toward-component) → blocked.
        assert!(!jump_runup_ok(Vec2::new(0.0, 300.0), fwd, 200.0, 0.5, 320.0));
        // Gate disabled → always allowed (today's behavior).
        assert!(jump_runup_ok(Vec2::ZERO, fwd, 200.0, 0.0, 320.0));
    }

    #[test]
    fn terminal_retry_blocks_periodic_repick_until_the_alternate_leg_starts() {
        let mut bot = BotState::default();
        bot.goal.set_item(17);
        bot.goal.item_cell = 4;
        bot.goal.next_pick = 1.0;
        bot.goal.terminal_arrival = Some(TerminalArrival {
            item: EntId(17),
            cell: 4,
            at: 2.0,
        });
        bot.route = vec![1, 2];
        bot.goal_cell = Some(4);

        consume_terminal_retry(&mut bot, 17, 9, 10.0);

        assert_eq!(bot.goal.item_cell, 9);
        assert_eq!(bot.goal.terminal_retried_item, Some(EntId(17)));
        assert!(bot.goal.terminal_arrival.is_none());
        assert_eq!(bot.goal.next_pick, 10.0 + GOAL_SELECT_INTERVAL);
        assert!(bot.route.is_empty());
        assert_eq!(bot.goal_cell, None);
        assert_eq!(bot.repath_time, 10.0);
    }

    #[test]
    fn timed_return_arrival_preserves_unspawned_item_goal_after_grace() {
        let mut bot = BotState::default();
        let item = EntId(17);
        let terminal = 4;
        bot.goal.set_item(item.0);
        bot.goal.item_cell = terminal;
        bot.goal.terminal_arrival = Some(TerminalArrival { item, cell: terminal, at: 1.0 });

        let now = 1.0 + TERMINAL_TAKE_GRACE;
        let changed = update_item_terminal(&mut bot, true, Solid::Not, terminal, Some(9), now);

        assert!(!changed, "an unspawned item is a wait, not a failed take");
        assert_eq!(bot.goal.item, item.0);
        assert_eq!(bot.goal.item_cell, terminal);
        assert_eq!(bot.goal.terminal_retried_item, None);
        assert!(!bot.is_avoided(item.0, now));
    }
}
