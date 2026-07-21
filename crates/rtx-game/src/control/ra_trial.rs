// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic DM3 red-armor acceptance trial for the external control harness.

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bot::state::{
    BotState, ControlOrder, GoalCommit, ItemTrial, ItemTrialMoveFrame, ItemTrialSample,
};
use crate::defs::{Bits, Flags, Items, Solid, Weapon};
use crate::entity::{EntId, Think};
use crate::game::GameState;
use crate::navmesh::LinkKind;

use super::{jnum, jstr, jvec3, route_legs_json, send, valid_bot};

// DM3 → red-armor acceptance anchors. `local` is the upper lip immediately before the currently
// failing JumpGap (internal regression only). `ra_spawn` is the exact stock deathmatch spawn in
// RA.tunnel, verified against the BSP entity lump; `ring` is the corpus major-zone centre.
const DM3_RA_LOCAL_START: Vec3 = Vec3::new(360.0, -677.0, 264.0);
const DM3_RA_SPAWN: Vec3 = Vec3::new(192.0, -208.0, -176.0);
const DM3_RA_RING_HINT: Vec3 = Vec3::new(240.0, -32.0, 56.0);
const DM3_RA_ORIGIN: Vec3 = Vec3::new(256.0, -704.0, 304.0);
pub(super) const RA_TRIAL_LOCAL_DEFAULT_SECS: f32 = 2.435_059;
// MVD exact-spawn → authoritative RA-taken calibration, restricted to one life where RA was
// active at spawn and this runner made the first subsequent RA take: n=86, 77 demos, p50=12.6255 s.
pub(super) const RA_TRIAL_SPAWN_DEFAULT_SECS: f32 = 12.6255;
pub(super) const RA_TRIAL_RING_DEFAULT_SECS: f32 = 9.604_003;
const RA_TRIAL_FALL_DEPTH: f32 = 56.0;
const RA_TRIAL_LOCAL_FLOOR_SLOP: f32 = 8.0;
const RA_TRIAL_STALL_SECS: f32 = 1.0;
const RA_TRIAL_WALL_SECS: f32 = 0.25;
const RA_TRIAL_MOVE_EPS: f32 = 16.0;
const RA_TRIAL_TOUCH_GRACE: f32 = 0.35;
const RA_TRIAL_WALL_PROBE: f32 = 24.0;
const RA_TRIAL_CONTACT_SLOP: f32 = 0.75;
/// Telemetry is sampled once per server frame. Budget for a 100 Hz server plus two boundary frames;
/// the protocol caps deadlines at 30 s, and this hard ceiling prevents hostile future callers from
/// turning an acceptance run into unbounded memory growth.
const RA_TRIAL_SAMPLE_HZ: f32 = 100.0;
const RA_TRIAL_SAMPLE_SLACK: usize = 2;
const RA_TRIAL_SAMPLE_LIMIT_MAX: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RaTrialStart {
    Local,
    RaSpawn,
    Ring,
}

impl RaTrialStart {
    fn name(self) -> &'static str {
        match self {
            Self::Local => "local",
            Self::RaSpawn => "ra_spawn",
            Self::Ring => "ring",
        }
    }

    pub(super) fn default_secs(self) -> f32 {
        match self {
            Self::Local => RA_TRIAL_LOCAL_DEFAULT_SECS,
            Self::RaSpawn => RA_TRIAL_SPAWN_DEFAULT_SECS,
            Self::Ring => RA_TRIAL_RING_DEFAULT_SECS,
        }
    }
}

/// Physical placement for an acceptance run. A semantic spawn must preserve the production spawn
/// entity's exact XY; its nearest nav cell is only the planner start. `+1 Z` matches
/// `client::place_at_spawn` and avoids starting embedded in the floor.
fn ra_trial_start_origin(start: RaTrialStart, hint: Vec3, planner_cell_origin: Vec3) -> Vec3 {
    let base = match start {
        RaTrialStart::Local | RaTrialStart::RaSpawn => hint,
        RaTrialStart::Ring => planner_cell_origin,
    };
    base + Vec3::new(0.0, 0.0, 1.0)
}

fn ra_trial_sample_limit(max_secs: f32) -> usize {
    ((max_secs.max(0.0) * RA_TRIAL_SAMPLE_HZ).ceil() as usize)
        .saturating_add(RA_TRIAL_SAMPLE_SLACK)
        .min(RA_TRIAL_SAMPLE_LIMIT_MAX)
}

pub(super) fn any_item_trial_active(game: &GameState) -> bool {
    game.entities.iter().any(|ent| ent.bot.puppet.item_trial.is_some())
}

fn item_trial_idle(active: bool, bot: u32) -> Result<(), String> {
    if active {
        Err(format!(
            "bot {bot} item trial busy; wait for ra_trial_result or timeout"
        ))
    } else {
        Ok(())
    }
}

pub(super) fn ensure_item_trial_idle(game: &GameState, e: EntId) -> Result<(), String> {
    item_trial_idle(game.entities[e].bot.puppet.item_trial.is_some(), e.0)
}

pub(super) fn ensure_global_item_trial_idle(game: &GameState) -> Result<(), String> {
    global_item_trial_idle(any_item_trial_active(game))
}

fn global_item_trial_idle(active: bool) -> Result<(), String> {
    if active {
        Err("another RA item trial is active; wait for ra_trial_result or timeout".into())
    } else {
        Ok(())
    }
}

/// Wipe all bot-side state that can carry movement or intent across trial attempts while retaining
/// only the fake-client identity needed to issue commands. This is intentionally broader than the
/// ordinary teleport reset: an acceptance attempt represents a new life, not a repositioned one.
fn reset_trial_bot_state(bot: &mut BotState, at: Vec3, now: f32) {
    let (is_bot, client) = (bot.is_bot, bot.client);
    *bot = BotState::default();
    bot.is_bot = is_bot;
    bot.client = client;
    bot.was_alive = true;
    bot.last_health = 100.0;
    bot.last_armor_value = 0.0;
    bot.watchdog.last_origin = at;
    bot.watchdog.stuck_origin = at;
    bot.watchdog.stuck_since = now;
    bot.repath_time = now;
}

/// Start a deterministic DM3 → red-armor acceptance run through the *real* item-goal
/// machinery. The harness controls only the initial world state and competing intent; route choice,
/// traversal drivers, terminal retry and the authoritative armor touch remain production behavior.
pub(super) fn do_ra_trial(
    game: &mut GameState,
    request_id: i64,
    bot: u32,
    start: RaTrialStart,
    max_secs: f32,
) -> Result<String, String> {
    if game.host().is_client() {
        return Err("ra_trial requires the authoritative server backend".into());
    }
    // RA is shared world state, so even trials on different bots must be single-flight.
    ensure_global_item_trial_idle(game)?;
    let e = valid_bot(game, bot)?;
    if !game.level.mapname.eq_ignore_ascii_case("dm3") {
        return Err(format!("ra_trial requires dm3 (current map {})", game.level.mapname));
    }
    let now = game.time();

    // Select the stock DM3 red armor by both classname and map coordinate. The coordinate guard
    // prevents a custom entity set from silently turning this map-specialized acceptance test into
    // a different item run.
    let ra = game
        .find_by_classname("item_armorInv")
        .min_by(|&a, &b| {
            (game.entities[a].v.origin - DM3_RA_ORIGIN)
                .length_squared()
                .total_cmp(&(game.entities[b].v.origin - DM3_RA_ORIGIN).length_squared())
        })
        .ok_or("dm3 red armor entity not found")?;
    if (game.entities[ra].v.origin - DM3_RA_ORIGIN).length() > 8.0 {
        return Err(format!("dm3 red armor moved to {:?}", game.entities[ra].v.origin));
    }

    // `ra_spawn` is a semantic map contract, not merely a convenient coordinate. Refuse to run it
    // if a custom entity set removed or moved the stock RA-tunnel deathmatch spawn.
    let ra_spawn = if start == RaTrialStart::RaSpawn {
        Some(
            game.find_by_classname("info_player_deathmatch")
                .find(|&spawn| {
                    (game.entities[spawn].v.origin - DM3_RA_SPAWN).length() <= 0.125
                })
                .ok_or_else(|| {
                    format!(
                        "dm3 RA-tunnel info_player_deathmatch missing at {:?}",
                        DM3_RA_SPAWN
                    )
                })?,
        )
    } else {
        None
    };

    let hint = match start {
        RaTrialStart::Local => DM3_RA_LOCAL_START,
        RaTrialStart::RaSpawn => DM3_RA_SPAWN,
        RaTrialStart::Ring => DM3_RA_RING_HINT,
    };
    let start_cell = game
        .nav
        .graph
        .as_ref()
        .ok_or("navmesh not ready")?
        .nearest(hint)
        .ok_or("no navmesh cell at trial start")?;
    let planner_origin = game.nav.graph.as_ref().unwrap().cell_origin(start_cell);
    let snapped = match start {
        // Gate A intentionally starts at the exact historical reproduction point. Gate B's corpus
        // location is a semantic hint, so snap it to a valid standing cell. The exact BSP spawn is
        // already a physical placement contract and must not inherit the planner cell's 16u Y skew.
        RaTrialStart::Local | RaTrialStart::RaSpawn => hint,
        RaTrialStart::Ring => planner_origin,
    };
    let at = ra_trial_start_origin(start, hint, planner_origin);
    debug_assert_eq!(at, snapped + Vec3::new(0.0, 0.0, 1.0));
    // A real map spawn owns both body and view angles. Synthetic Ring/local starts use an explicit
    // zero pose rather than inheriting whichever direction the previous attempt happened to face.
    let start_angles = ra_spawn.map_or(Vec3::ZERO, |spawn| game.entities[spawn].v.angles);

    // Choose among every touch-valid RA terminal using the exact live bot pricing. This is the same
    // item-goal representation production selection uses; importantly, the target is a terminal cell
    // whose standing hull overlaps RA, not RA's coordinate.
    let pricing = game.bot_item_trial_link_pricing(e, now);
    let (terminal, planned_route) = {
        let g = game.nav.graph.as_ref().unwrap();
        let travel = g.costs_from(start_cell, &pricing.costs(0));
        let terminal = game
            .nav
            .goals
            .iter()
            .filter_map(|&(item, cell)| (item == ra.0).then_some(cell))
            .filter(|&cell| crate::bot::item_terminal_touches(g.cell_origin(cell), &game.entities[ra]))
            .filter(|&cell| travel[cell as usize].is_finite())
            .min_by(|&a, &b| travel[a as usize].total_cmp(&travel[b as usize]))
            .ok_or("red armor has no reachable touch-valid terminal")?;
        let route_costs = pricing.costs(e.0);
        let use_bands = game.host.cvar_bool(c"rtx_bot_bhop")
            && game.host.cvar_bool(c"rtx_bot_bandplan");
        let route = if use_bands {
            g.find_path_banded(start_cell, terminal, 0.0, &route_costs)
                .map(|route| route.links)
        } else {
            g.find_path(start_cell, terminal, &route_costs)
        }
        .ok_or("red armor terminal became unreachable under production planner")?;
        (terminal, route)
    };

    // Everything above is read-only and fallible. Only after the item, start, touch terminal and
    // route have all been validated do we change authoritative world state, making a rejected
    // command atomic from the caller's point of view.
    if game.entities[ra].v.solid != Solid::Trigger {
        game.sub_regen(ra);
    }
    game.entities[ra].think = Think::None;
    game.entities[ra].v.nextthink = 0.0;

    // Fresh-spawn stock removes loadout-dependent routing and makes the armor touch itself the sole
    // success signal. Reuse production's body/pose helpers (including dead-body revival, hull,
    // view, water/ground/buttons, combat and grapple cleanup), but deliberately skip spawn selection,
    // telefragging and mode lifecycle: this trial already validated its exact source above.
    game.configure_fresh_player_body(e);
    game.place_fresh_player_body_at(e, at, start_angles);
    reset_trial_bot_state(&mut game.entities[e].bot, at, now);
    {
        let ent = &mut game.entities[e];
        ent.v.armorvalue = 0.0;
        ent.v.armortype = 0.0;
        ent.v.items = (Items::AXE | Items::SHOTGUN).as_f32();
        ent.v.weapon = Weapon::Shotgun;
        ent.v.ammo_shells = 25.0;
        ent.v.ammo_nails = 0.0;
        ent.v.ammo_rockets = 0.0;
        ent.v.ammo_cells = 0.0;
    }
    game.w_set_current_ammo(e);

    let deadline = now + max_secs;
    let sample_limit = ra_trial_sample_limit(max_secs);
    {
        let b = &mut game.entities[e].bot;
        b.goal.set_item(ra.0);
        b.goal.item_cell = terminal;
        b.goal.commit = GoalCommit::Pickup;
        b.goal.since = now;
        b.goal.next_pick = deadline + 1.0;
        b.goal.magnet_item = 0;
        b.puppet.item_trial = Some(ItemTrial {
            request_id,
            item: ra.0,
            terminal,
            scenario: start.name(),
            start_hint: hint,
            started: now,
            deadline,
            start_origin: at,
            initial_armor: 0.0,
            wish: Vec3::ZERO,
            pending_wish: Vec3::ZERO,
            buttons: 0,
            pending_buttons: 0,
            move_frame: ItemTrialMoveFrame {
                route_pos: 0,
                link: u32::MAX,
                terminal,
            },
            last_origin: at,
            last_velocity: Vec3::ZERO,
            last_t: now,
            motion_anchor: at,
            motion_since: now,
            wall_run: 0.0,
            wall_max: 0.0,
            wall_contacts: 0,
            wall_normal: Vec3::ZERO,
            ground_z: at.z,
            terminal_touch_since: None,
            goal_lost: false,
            min_z: at.z,
            peak_speed: 0.0,
            initial_route: planned_route.clone(),
            route_captured: false,
            sample_limit,
            samples: Vec::with_capacity(sample_limit),
            samples_truncated: false,
        });
    }

    let g = game.nav.graph.as_ref().unwrap();
    Ok(format!(
        "{{\"bot\":{bot},\"scenario\":{},\"start_hint\":{},\"start\":{},\"start_cell\":{start_cell},\
         \"item\":{},\"item_origin\":{},\"terminal\":{terminal},\"terminal_origin\":{},\
         \"max_secs\":{},\"planned_route\":{}}}",
        jstr(start.name()),
        jvec3(hint),
        jvec3(at),
        ra.0,
        jvec3(game.entities[ra].v.origin),
        jvec3(g.cell_origin(terminal)),
        jnum(max_secs),
        route_legs_json(g, &planned_route),
    ))
}

// --- per-frame puppet pollers (emit lifecycle events) ---

/// The authoritative RA pickup signal. All three facts are required: inventory/armor prove *this*
/// bot changed, while the trigger disappearing proves the server completed the item touch.
fn ra_pickup_complete(armor: f32, items: f32, ra_solid: Solid) -> bool {
    armor >= 199.0 && items.has(Items::ARMOR3) && ra_solid != Solid::Trigger
}

/// Whether this frame physically reached the static-BSP surface found by the forward hull probe.
/// Comparing impact distance with realized travel prevents a merely predictive wall within the 24u
/// probe from counting as contact. `drive` may be either the submitted wish or the velocity carried
/// into that movement step; both can push the body into a plane.
fn physical_wall_contact_frame(
    drive: Vec3,
    delta: Vec3,
    trace_fraction: f32,
    plane_normal: Vec3,
) -> bool {
    let dir = drive.xy().normalize_or_zero();
    let normal = plane_normal.xy().normalize_or_zero();
    let inward_drive = -drive.xy().dot(normal);
    let impact_distance = trace_fraction * RA_TRIAL_WALL_PROBE;
    // If this frame reached the probed impact, subtracting the trace-to-impact vector leaves only
    // tangent/into-plane motion; its outward-normal clearance is at most the numerical slop. A wall
    // that is merely nearby leaves positive clearance and is not a physical contact yet.
    let clearance = (delta.xy() - dir * impact_distance).dot(normal);
    normal != Vec2::ZERO
        && inward_drive >= 64.0
        && trace_fraction < 0.99
        && clearance <= RA_TRIAL_CONTACT_SLOP
}

/// A continuous wall push is physical contact plus negligible progress into the collision plane.
/// Measuring the inward normal component (not total drive-axis displacement) correctly catches a
/// bot sliding quickly along a wall while wish or retained momentum keeps driving into it.
fn blocked_drive_frame(drive: Vec3, delta: Vec3, trace_fraction: f32, plane_normal: Vec3) -> bool {
    let normal = plane_normal.xy().normalize_or_zero();
    physical_wall_contact_frame(drive, delta, trace_fraction, plane_normal)
        && -delta.xy().dot(normal) < 0.5
}

#[derive(Default)]
struct StaticWallFrame {
    contact: bool,
    push: bool,
    normal: Vec3,
}

impl StaticWallFrame {
    /// Fold one traced movement drive into a single per-frame result. Multiple drives are deliberate:
    /// the wish can turn south while westward velocity from the preceding step still hits an X wall.
    fn observe_probe(
        &mut self,
        drive: Vec3,
        delta: Vec3,
        trace_fraction: f32,
        plane_normal: Vec3,
        ascending_step_riser: bool,
    ) {
        let contact = physical_wall_contact_frame(drive, delta, trace_fraction, plane_normal)
            && !ascending_step_riser;
        if !contact {
            return;
        }
        let push = blocked_drive_frame(drive, delta, trace_fraction, plane_normal);
        // Prefer the plane responsible for a sustained push when two probes hit in one frame.
        if !self.contact || (push && !self.push) {
            self.normal = plane_normal;
        }
        self.contact = true;
        self.push |= push;
    }
}

/// Pmove climbs a stair by first tracing into its vertical face, then stepping upward. The impact can
/// therefore occur one producer frame before the Z rise. Exempt only the plane that faces opposite
/// an ascending Walk/Step link; a lateral wall on that same link remains a strict contact. DM3's
/// lower-RA +8u ramp cells are Walk links even though pmove crosses their faces by stepping.
fn ascending_step_riser_plane(
    kind: Option<LinkKind>,
    source: Vec3,
    target: Vec3,
    plane_normal: Vec3,
) -> bool {
    let travel = (target.xy() - source.xy()).normalize_or_zero();
    let normal = plane_normal.xy().normalize_or_zero();
    matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && target.z > source.z + 0.5
        && travel != Vec2::ZERO
        && normal != Vec2::ZERO
        && -normal.dot(travel) >= 0.7
}

/// Resolve the stair exemption from the link snapshot that produced the observed command. The live
/// route may already have advanced (notably rising ordinary → JumpGap) while preparing the next command.
fn ascending_step_riser_for_link(
    producer_link: u32,
    plane_normal: Vec3,
    link_frame: impl FnOnce(u32) -> Option<(LinkKind, Vec3, Vec3)>,
) -> bool {
    link_frame(producer_link).is_some_and(|(kind, source, target)| {
        ascending_step_riser_plane(Some(kind), source, target, plane_normal)
    })
}

/// A Drop is structural only while the trial item still owns the bot's active route. On the
/// authoritative pickup frame normal goal selection may already install a post-pickup roam route;
/// that replacement must never retroactively invalidate the completed RA route.
fn trial_route_has_planned_drop(goal_item: u32, trial_item: u32, remaining_has_drop: bool) -> bool {
    goal_item == trial_item && remaining_has_drop
}

fn fell_from_ground(ground_z: f32, z: f32) -> bool {
    z < ground_z - RA_TRIAL_FALL_DEPTH
}

#[derive(Default)]
struct RaTrialFacts {
    pickup: bool,
    planned_drop: bool,
    alive: bool,
    fell: bool,
    wall_secs: f32,
    touch_secs: Option<f32>,
    item_taken_elsewhere: bool,
    goal_lost: bool,
    stalled: bool,
    timed_out: bool,
}

/// Mutually-exclusive trial classifier. Ordering is intentionally strict: every structural or
/// deadline violation beats a simultaneous pickup, so a bot cannot pass by touching RA while
/// falling, pushing a wall, after losing its goal, or on/after the deadline frame.
fn ra_trial_outcome(f: &RaTrialFacts) -> Option<(bool, &'static str)> {
    if f.planned_drop {
        Some((false, "planned_drop"))
    } else if !f.alive {
        Some((false, "dead"))
    } else if f.fell {
        Some((false, "fall"))
    } else if f.wall_secs >= RA_TRIAL_WALL_SECS {
        Some((false, "wall_push"))
    } else if f.touch_secs.is_some_and(|secs| secs >= RA_TRIAL_TOUCH_GRACE) {
        Some((false, "no_pickup"))
    } else if f.item_taken_elsewhere {
        Some((false, "item_taken_elsewhere"))
    } else if f.goal_lost {
        Some((false, "goal_lost"))
    } else if f.stalled {
        Some((false, "stall"))
    } else if f.timed_out {
        Some((false, "timeout"))
    } else if f.pickup {
        Some((true, "pickup"))
    } else {
        None
    }
}

/// Observe one frame of a real item-goal run and either retain the trial or emit one terminal event.
/// Taking the state out while evaluating avoids interleaving mutable bot telemetry with immutable
/// BSP/item snapshots; unfinished state is put back at the end.
pub(super) fn poll_ra_trial(game: &mut GameState, e: EntId, bot: u32, now: f32) {
    let Some(mut trial) = game.entities[e].bot.puppet.item_trial.take() else {
        return;
    };
    let origin = game.entities[e].v.origin;
    let velocity = game.entities[e].v.velocity;
    let on_ground = game.entities[e].v.flags.has(Flags::ONGROUND);
    let alive = game.entities[e].is_alive();
    let armor = game.entities[e].v.armorvalue;
    let items = game.entities[e].v.items;
    let ra_solid = game.entities[EntId(trial.item)].v.solid;
    let touching = crate::bot::item_terminal_touches(origin, &game.entities[EntId(trial.item)]);
    let (goal_item, selected_terminal, route_pos, current_link) = {
        let b = &game.entities[e].bot;
        (
            b.goal.item,
            b.goal.item_cell,
            b.route_pos,
            b.route.get(b.route_pos).copied().unwrap_or(u32::MAX),
        )
    };
    let dt = (now - trial.last_t).clamp(0.0, 0.1);
    let delta = origin - trial.last_origin;
    trial.min_z = trial.min_z.min(origin.z);
    trial.peak_speed = trial.peak_speed.max(velocity.xy().length());

    let mut wall_frame = StaticWallFrame::default();
    let realized_velocity = if dt > f32::EPSILON { delta / dt } else { Vec3::ZERO };
    // A new wish does not erase Quake momentum. Probe all physical drives that could have carried
    // this hull into static BSP, then collapse them to one strict per-frame contact result.
    for drive in [trial.wish, trial.last_velocity, realized_velocity] {
        if drive.xy().length() < 64.0 {
            continue;
        }
        let dir = drive.xy().normalize_or_zero();
        let end = trial.last_origin + Vec3::new(dir.x, dir.y, 0.0) * RA_TRIAL_WALL_PROBE;
        if let Some(tr) = game
            .nav
            .bsp
            .as_ref()
            .map(|bsp| bsp.hull1_trace(trial.last_origin, end))
        {
            let ascending_step_riser = game.nav.graph.as_ref().is_some_and(|g| {
                ascending_step_riser_for_link(trial.move_frame.link, tr.plane_normal, |link| {
                    ((link as usize) < g.links.len()).then(|| {
                        (
                            g.link_kind(link),
                            g.cell_origin(g.link_source(link)),
                            g.cell_origin(g.link_target(link)),
                        )
                    })
                })
            });
            wall_frame.observe_probe(
                drive,
                delta,
                tr.fraction,
                tr.plane_normal,
                ascending_step_riser,
            );
        }
    }
    let (wall_contact, wall_push, wall_normal) =
        (wall_frame.contact, wall_frame.push, wall_frame.normal);
    if wall_contact {
        trial.wall_contacts = trial.wall_contacts.saturating_add(1);
        trial.wall_normal = wall_normal;
    }
    if wall_push {
        trial.wall_run += dt;
        trial.wall_max = trial.wall_max.max(trial.wall_run);
    } else {
        trial.wall_run = 0.0;
    }

    if (origin - trial.motion_anchor).length() >= RA_TRIAL_MOVE_EPS {
        trial.motion_anchor = origin;
        trial.motion_since = now;
    }
    if touching {
        trial.terminal_touch_since.get_or_insert(now);
    } else {
        trial.terminal_touch_since = None;
    }

    // Replace the acknowledgement's prevalidated production-planner route with the route steering
    // actually installed. Hard structural checks below inspect only this executed route.
    if goal_item == trial.item && !trial.route_captured && !game.entities[e].bot.route.is_empty() {
        trial.initial_route = game.entities[e].bot.route.clone();
        trial.route_captured = true;
    }

    if trial.samples.len() < trial.sample_limit {
        trial.samples.push(ItemTrialSample {
            t: now,
            origin,
            velocity,
            wish: trial.wish,
            buttons: trial.buttons,
            on_ground,
            wall: wall_contact,
            route_pos: trial.move_frame.route_pos,
            link: trial.move_frame.link,
            terminal: trial.move_frame.terminal,
        });
    } else {
        trial.samples_truncated = true;
    }

    let remaining_has_drop = game.nav.graph.as_ref().is_some_and(|g| {
        game.entities[e]
            .bot
            .route
            .iter()
            .skip(route_pos)
            .any(|&link| g.link_kind(link) == LinkKind::Drop)
    });
    let planned_drop = trial_route_has_planned_drop(goal_item, trial.item, remaining_has_drop);
    // Compare against the previous grounded height before accepting a new grounded anchor, so a
    // landing on the floor below is classified as a fall rather than silently resetting the datum.
    let fell = fell_from_ground(trial.ground_z, origin.z)
        || (trial.scenario == "local"
            && origin.z < trial.start_origin.z - RA_TRIAL_LOCAL_FLOOR_SLOP);
    if on_ground && !fell {
        trial.ground_z = origin.z;
    }

    let pickup = ra_pickup_complete(armor, items, ra_solid);
    // A normal armor touch clears the item goal in the same authoritative frame. Latch goal loss
    // only when it disappears without that success signal; once latched, a later pickup cannot pass.
    if goal_item != trial.item && !pickup {
        trial.goal_lost = true;
    }

    let outcome = ra_trial_outcome(&RaTrialFacts {
        pickup,
        planned_drop,
        alive,
        fell,
        wall_secs: trial.wall_run,
        touch_secs: trial.terminal_touch_since.map(|since| now - since),
        item_taken_elsewhere: ra_solid != Solid::Trigger && !pickup,
        goal_lost: trial.goal_lost,
        stalled: trial.wish.xy().length() >= 64.0
            && now - trial.motion_since >= RA_TRIAL_STALL_SECS,
        timed_out: now >= trial.deadline,
    });

    if let Some((ok, reason)) = outcome {
        finish_ra_trial(game, e, bot, now, trial, ok, reason);
        return;
    }

    trial.last_origin = origin;
    trial.last_velocity = velocity;
    trial.last_t = now;
    trial.wish = trial.pending_wish;
    trial.buttons = trial.pending_buttons;
    trial.move_frame = ItemTrialMoveFrame {
        route_pos,
        link: current_link,
        terminal: selected_terminal,
    };
    game.entities[e].bot.puppet.item_trial = Some(trial);
}

fn trial_samples_json(samples: &[ItemTrialSample]) -> String {
    let mut out = String::new();
    for s in samples {
        if !out.is_empty() {
            out.push(',');
        }
        let link = if s.link == u32::MAX { "null".to_string() } else { s.link.to_string() };
        out.push_str(&format!(
            "[{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}]",
            jnum(s.t),
            jnum(s.origin.x),
            jnum(s.origin.y),
            jnum(s.origin.z),
            jnum(s.velocity.x),
            jnum(s.velocity.y),
            jnum(s.velocity.z),
            jnum(s.wish.x),
            jnum(s.wish.y),
            jnum(s.wish.z),
            s.buttons,
            s.on_ground,
            s.wall,
            s.route_pos,
            link,
            s.terminal,
            // Reserved final column keeps the row schema extensible without changing old indices.
            0,
        ));
    }
    out
}

fn finish_ra_trial(
    game: &mut GameState,
    e: EntId,
    bot: u32,
    now: f32,
    trial: ItemTrial,
    ok: bool,
    reason: &str,
) {
    let ent = &game.entities[e];
    let origin = ent.v.origin;
    let velocity = ent.v.velocity;
    let armor = ent.v.armorvalue;
    let armortype = ent.v.armortype;
    let items = ent.v.items;
    let on_ground = ent.v.flags.has(Flags::ONGROUND);
    let selected_item = ent.bot.goal.item;
    let selected_terminal = ent.bot.goal.item_cell;
    let route_pos = ent.bot.route_pos;
    let current_link = ent.bot.route.get(route_pos).copied();
    let client = ent.bot.client;
    let ra = &game.entities[EntId(trial.item)];
    let ra_solid = format!("{:?}", ra.v.solid);
    let initial_route = game
        .nav
        .graph
        .as_ref()
        .map(|g| route_legs_json(g, &trial.initial_route))
        .unwrap_or_else(|| "[]".to_string());
    let terminal_origin = game
        .nav
        .graph
        .as_ref()
        .map(|g| jvec3(g.cell_origin(trial.terminal)))
        .unwrap_or_else(|| "null".to_string());
    let current_link = current_link.map_or_else(|| "null".to_string(), |link| link.to_string());
    let samples = trial_samples_json(&trial.samples);
    let event = format!(
        "{{\"ev\":\"ra_trial_result\",\"request_id\":{},\"map\":{},\"bot\":{bot},\"client\":{client},\
         \"ok\":{ok},\"reason\":{},\"scenario\":{},\"forced_item_goal\":true,\
         \"started\":{},\"ended\":{},\"elapsed\":{},\"max_secs\":{},\
         \"start_hint\":{},\"start\":{},\"origin\":{},\"velocity\":{},\"wish\":{},\
         \"buttons\":{},\"on_ground\":{on_ground},\"item\":{},\"item_origin\":{},\
         \"terminal\":{},\"terminal_origin\":{},\"selected_item\":{selected_item},\
         \"selected_terminal\":{selected_terminal},\"route_pos\":{route_pos},\"current_link\":{current_link},\
         \"armor_before\":{},\"armor\":{},\"armortype\":{},\"items\":{},\"ra_solid\":{},\
         \"min_z\":{},\"peak_speed\":{},\"wall_secs\":{},\"wall_contacts\":{},\"wall_normal\":{},\
         \"wall_contact_scope\":\"static_bsp\",\"route_captured\":{},\"samples_truncated\":{},\
         \"sample_schema\":[\"t\",\"x\",\"y\",\"z\",\"vx\",\"vy\",\"vz\",\"wish_x\",\"wish_y\",\"wish_z\",\"buttons\",\"on_ground\",\"wall\",\"route_pos\",\"link\",\"terminal\",\"reserved\"],\
         \"initial_route\":{initial_route},\"samples\":[{samples}]}}",
        trial.request_id,
        jstr(&game.level.mapname),
        jstr(reason),
        jstr(trial.scenario),
        jnum(trial.started),
        jnum(now),
        jnum(now - trial.started),
        jnum(trial.deadline - trial.started),
        jvec3(trial.start_hint),
        jvec3(trial.start_origin),
        jvec3(origin),
        jvec3(velocity),
        jvec3(trial.wish),
        trial.buttons,
        trial.item,
        jvec3(ra.v.origin),
        trial.terminal,
        terminal_origin,
        jnum(trial.initial_armor),
        jnum(armor),
        jnum(armortype),
        jnum(items),
        jstr(&ra_solid),
        jnum(trial.min_z),
        jnum(trial.peak_speed),
        jnum(trial.wall_max),
        trial.wall_contacts,
        jvec3(trial.wall_normal),
        trial.route_captured,
        trial.samples_truncated,
    );

    let b = &mut game.entities[e].bot;
    b.puppet.order = Some(ControlOrder::Hold);
    b.route.clear();
    b.route_bands.clear();
    b.route_pos = 0;
    b.goal.set_item(0);
    b.goal.commit = GoalCommit::None;
    send(game, event);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bot::state::{AirCommit, Commit, HookPhase, RjPhase};

    #[test]
    fn ra_spawn_uses_exact_production_spawn_placement_not_nav_cell_xy() {
        let planner_cell = Vec3::new(192.0, -224.0, -176.0);
        assert_eq!(
            ra_trial_start_origin(RaTrialStart::RaSpawn, DM3_RA_SPAWN, planner_cell),
            Vec3::new(192.0, -208.0, -175.0)
        );
        assert_eq!(
            ra_trial_start_origin(RaTrialStart::Ring, DM3_RA_RING_HINT, planner_cell),
            Vec3::new(192.0, -224.0, -175.0)
        );
    }

    #[test]
    fn trial_sample_budget_covers_the_full_deadline_at_100_hz() {
        assert_eq!(ra_trial_sample_limit(RA_TRIAL_SPAWN_DEFAULT_SECS), 1265);
        assert!(
            ra_trial_sample_limit(RA_TRIAL_SPAWN_DEFAULT_SECS) > 1024,
            "the corpus-median ra_spawn deadline must not inherit the old fixed cap"
        );
        assert_eq!(ra_trial_sample_limit(30.0), 3002);
        assert_eq!(ra_trial_sample_limit(f32::MAX), RA_TRIAL_SAMPLE_LIMIT_MAX);
    }

    #[test]
    fn ra_success_requires_armor_bit_and_hidden_trigger() {
        let armor3 = Items::ARMOR3.as_f32();
        assert!(ra_pickup_complete(200.0, armor3, Solid::Not));
        assert!(!ra_pickup_complete(198.0, armor3, Solid::Not));
        assert!(!ra_pickup_complete(200.0, 0.0, Solid::Not));
        assert!(!ra_pickup_complete(200.0, armor3, Solid::Trigger));
    }

    #[test]
    fn wall_and_fall_frame_predicates_are_strict() {
        let wish = Vec3::new(320.0, 0.0, 0.0);
        let normal = Vec3::new(-1.0, 0.0, 0.0);
        assert!(physical_wall_contact_frame(wish, Vec3::ZERO, 0.0, normal));
        assert!(blocked_drive_frame(wish, Vec3::ZERO, 0.0, normal));
        assert!(!physical_wall_contact_frame(wish, Vec3::new(2.0, 0.0, 0.0), 0.5, normal));
        assert!(!blocked_drive_frame(wish, Vec3::ZERO, 1.0, normal));

        // A fast tangent slide used to look like progress because total delta·wish was positive.
        // The inward normal component correctly identifies the continuing diagonal wall push.
        let diagonal = Vec3::new(320.0, 320.0, 0.0);
        let slide = Vec3::new(0.0, 6.0, 0.0);
        assert!(physical_wall_contact_frame(diagonal, slide, 0.0, normal));
        assert!(blocked_drive_frame(diagonal, slide, 0.0, normal));
        assert!(
            !physical_wall_contact_frame(diagonal, slide, 0.2, normal),
            "tangent motion near a predictive wall is not contact until it reaches the plane"
        );

        // Contact and sustained push are separate metrics: reaching the plane counts even when the
        // frame made measurable inward progress before impact.
        let impact = Vec3::new(4.0, 0.0, 0.0);
        assert!(physical_wall_contact_frame(wish, impact, 4.0 / RA_TRIAL_WALL_PROBE, normal));
        assert!(!blocked_drive_frame(wish, impact, 4.0 / RA_TRIAL_WALL_PROBE, normal));
        assert!(!fell_from_ground(300.0, 244.0));
        assert!(fell_from_ground(300.0, 243.9));

        let step_source = Vec3::new(0.0, 0.0, 280.0);
        let step_target = Vec3::new(32.0, 0.0, 296.0);
        let riser_normal = Vec3::NEG_X;
        assert!(
            ascending_step_riser_plane(Some(LinkKind::Step), step_source, step_target, riser_normal),
            "the expected riser is exempt even on its pre-rise producer frame"
        );
        assert!(
            !ascending_step_riser_plane(Some(LinkKind::Step), step_source, step_target, Vec3::NEG_Y),
            "a lateral block while following an ascending Step remains a wall contact"
        );
        assert!(!ascending_step_riser_plane(
            None,
            step_source,
            step_target,
            riser_normal,
        ));
        assert!(
            ascending_step_riser_plane(Some(LinkKind::Walk), step_source, step_target, riser_normal),
            "DM3's +8u lower-RA ramp is represented by rising Walk links"
        );
        assert!(!ascending_step_riser_plane(
            Some(LinkKind::Step),
            step_target,
            step_source,
            Vec3::X,
        ));
    }

    #[test]
    fn wall_probe_catches_westward_momentum_after_wish_turns_south() {
        let south_wish = Vec3::new(0.0, -320.0, 0.0);
        let west_momentum = Vec3::new(-320.0, 0.0, 0.0);
        let south_slide = Vec3::new(0.0, -3.0, 0.0);
        let x_wall_normal = Vec3::X;
        let mut frame = StaticWallFrame::default();

        // Looking only along the new wish sees no wall. Retained westward momentum is already at
        // the X plane, and the realized frame slides south instead of progressing through it.
        frame.observe_probe(south_wish, south_slide, 1.0, x_wall_normal, false);
        assert!(!frame.contact);
        frame.observe_probe(west_momentum, south_slide, 0.0, x_wall_normal, false);

        assert!(frame.contact);
        assert!(frame.push);
        assert_eq!(frame.normal, x_wall_normal);
    }

    #[test]
    fn step_to_jumpgap_uses_command_producer_link_for_riser_exemption() {
        const STEP_LINK: u32 = 41;
        const JUMP_LINK: u32 = 42;
        let link_frame = |link| match link {
            STEP_LINK => Some((
                LinkKind::Step,
                Vec3::new(0.0, 0.0, 280.0),
                Vec3::new(32.0, 0.0, 296.0),
            )),
            JUMP_LINK => Some((
                LinkKind::JumpGap,
                Vec3::new(32.0, 0.0, 296.0),
                Vec3::new(32.0, -96.0, 328.0),
            )),
            _ => None,
        };
        let producer = ItemTrialMoveFrame {
            route_pos: 7,
            link: STEP_LINK,
            terminal: 1320,
        };
        let live_current_link = JUMP_LINK;

        assert!(ascending_step_riser_for_link(producer.link, Vec3::NEG_X, link_frame));
        assert!(
            !ascending_step_riser_for_link(live_current_link, Vec3::NEG_X, link_frame),
            "the already-advanced live JumpGap must not be attributed to the Step command"
        );

        // The inverse transition is equally important: a JumpGap command never inherits a Step
        // exemption merely because the live route was replaced before its displacement was sampled.
        let jump_producer = ItemTrialMoveFrame { link: JUMP_LINK, ..producer };
        let live_replanned_step = STEP_LINK;
        assert!(!ascending_step_riser_for_link(
            jump_producer.link,
            Vec3::NEG_X,
            link_frame
        ));
        assert!(ascending_step_riser_for_link(
            live_replanned_step,
            Vec3::NEG_X,
            link_frame
        ));
    }

    #[test]
    fn post_pickup_route_cannot_create_a_planned_drop_failure() {
        assert!(trial_route_has_planned_drop(116, 116, true));
        assert!(!trial_route_has_planned_drop(127, 116, true));
        assert!(!trial_route_has_planned_drop(116, 116, false));
    }

    #[test]
    fn active_item_trial_rejects_competing_control() {
        assert!(item_trial_idle(false, 1).is_ok());
        let err = item_trial_idle(true, 1).unwrap_err();
        assert!(err.contains("bot 1 item trial busy"));
        assert!(err.contains("ra_trial_result"));
    }

    #[test]
    fn active_trial_blocks_plan_link_global_mutation_gate() {
        assert!(global_item_trial_idle(false).is_ok());
        let err = global_item_trial_idle(true).unwrap_err();
        assert!(err.contains("another RA item trial is active"));
        assert!(err.contains("ra_trial_result"));
    }

    fn poison_trial_bot(bot: &mut BotState, n: u32) {
        bot.was_alive = false;
        bot.spawn_exit = true;
        bot.spawn_exit_until = 99.0;
        bot.last_health = -40.0;
        bot.last_armor_value = 200.0;
        bot.route = vec![n, n + 1];
        bot.route_bands = vec![2, 3];
        bot.route_pos = 1;
        bot.goal_cell = Some(n + 2);
        bot.goal.set_item(n + 3);
        bot.goal.item_cell = n + 4;
        bot.goal.commit = GoalCommit::Pickup;
        bot.failed_links[0] = (n + 5, 100.0, 3);
        bot.repath_time = 777.0;
        bot.watchdog.last_origin = Vec3::splat(1.0);
        bot.watchdog.stuck_origin = Vec3::splat(2.0);
        bot.watchdog.stuck_since = 3.0;
        bot.watchdog.progress_best = 4.0;
        bot.watchdog.progress_since = 5.0;
        bot.pulse = true;
        bot.hook.phase = HookPhase::Reel;
        bot.hook.link = n + 6;
        bot.rj.phase = RjPhase::Ballistic;
        bot.rj.link = n + 7;
        bot.sj = Some(Commit::new(n + 8, 8.0, false));
        bot.air = Some(AirCommit {
            leg: n + 9,
            target: n + 10,
            since: 9.0,
            airborne: true,
        });
        bot.bhop.hops = n + 11;
        bot.bhop.flips = n + 12;
        bot.bhop.peak = 999.0;
        bot.puppet.order = Some(ControlOrder::Goto { target: Vec3::splat(n as f32) });
        bot.puppet.traj.push((10.0, Vec3::ONE, Vec3::splat(320.0)));
        bot.puppet.fly_airborne = true;
        bot.puppet.fly_takeoff_speed = 500.0;
    }

    #[test]
    fn repeated_trial_bot_reset_does_not_inherit_traversal_or_intent() {
        let mut bot = BotState::default();
        bot.is_bot = true;
        bot.client = 7;
        let at = Vec3::new(192.0, -208.0, -175.0);
        let now = 42.25;

        for attempt in 1..=2 {
            poison_trial_bot(&mut bot, attempt);
            reset_trial_bot_state(&mut bot, at, now);

            assert!(bot.is_bot);
            assert_eq!(bot.client, 7);
            assert!(bot.was_alive);
            assert!(!bot.spawn_exit);
            assert_eq!(bot.last_health, 100.0);
            assert_eq!(bot.last_armor_value, 0.0);
            assert!(bot.route.is_empty());
            assert!(bot.route_bands.is_empty());
            assert_eq!(bot.route_pos, 0);
            assert_eq!(bot.goal_cell, None);
            assert_eq!(bot.goal.item, 0);
            assert_eq!(bot.goal.commit, GoalCommit::None);
            assert!(bot.failed_links.iter().all(|entry| *entry == (0, 0.0, 0)));
            assert_eq!(bot.repath_time, now);
            assert_eq!(bot.watchdog.last_origin, at);
            assert_eq!(bot.watchdog.stuck_origin, at);
            assert_eq!(bot.watchdog.stuck_since, now);
            assert_eq!(bot.watchdog.progress_best, 0.0);
            assert_eq!(bot.watchdog.progress_since, 0.0);
            assert!(!bot.pulse);
            assert_eq!(bot.hook.phase, HookPhase::Idle);
            assert_eq!(bot.rj.phase, RjPhase::Idle);
            assert!(bot.sj.is_none());
            assert!(bot.air.is_none());
            assert_eq!(bot.bhop.hops, 0);
            assert_eq!(bot.bhop.flips, 0);
            assert_eq!(bot.bhop.peak, 0.0);
            assert!(bot.puppet.order.is_none());
            assert!(bot.puppet.item_trial.is_none());
            assert!(bot.puppet.traj.is_empty());
            assert!(!bot.puppet.fly_airborne);
            assert_eq!(bot.puppet.fly_takeoff_speed, 0.0);
        }
    }

    fn running_trial_facts() -> RaTrialFacts {
        RaTrialFacts {
            alive: true,
            ..Default::default()
        }
    }

    #[test]
    fn hard_trial_outcomes_are_distinct_and_prioritized() {
        let mut f = running_trial_facts();
        f.planned_drop = true;
        f.timed_out = true;
        assert_eq!(ra_trial_outcome(&f), Some((false, "planned_drop")));

        let mut f = running_trial_facts();
        f.fell = true;
        assert_eq!(ra_trial_outcome(&f), Some((false, "fall")));

        let mut f = running_trial_facts();
        f.wall_secs = RA_TRIAL_WALL_SECS;
        assert_eq!(ra_trial_outcome(&f), Some((false, "wall_push")));
        f.wall_secs = RA_TRIAL_WALL_SECS - 0.001;
        assert_eq!(ra_trial_outcome(&f), None, "a transient contact is not a wall failure");

        let mut f = running_trial_facts();
        f.touch_secs = Some(RA_TRIAL_TOUCH_GRACE);
        assert_eq!(ra_trial_outcome(&f), Some((false, "no_pickup")));

        let mut f = running_trial_facts();
        f.stalled = true;
        f.timed_out = true;
        assert_eq!(ra_trial_outcome(&f), Some((false, "stall")));

        let mut f = running_trial_facts();
        f.pickup = true;
        assert_eq!(ra_trial_outcome(&f), Some((true, "pickup")));

        for (reason, set_hard_fact) in [
            ("planned_drop", 0_u8),
            ("fall", 1),
            ("goal_lost", 2),
            ("timeout", 3),
        ] {
            let mut f = running_trial_facts();
            f.pickup = true;
            match set_hard_fact {
                0 => f.planned_drop = true,
                1 => f.fell = true,
                2 => f.goal_lost = true,
                _ => f.timed_out = true,
            }
            assert_eq!(ra_trial_outcome(&f), Some((false, reason)), "{reason} must beat pickup");
        }
    }
}
