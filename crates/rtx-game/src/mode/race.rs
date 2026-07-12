// SPDX-License-Identifier: AGPL-3.0-or-later

//! Race (`rtx_mode race`) — run a KTX race route (see [`crate::race`]) from its start pad
//! through its checkpoints to the finish, timed per runner. Built first as a **bot sanity
//! harness**: most race routes are unfinishable without bunnyhop-accumulated speed, so bots
//! finishing (or timing out on a named leg) is a live regression check on the speed-jump /
//! bunnyhop machinery. The KTX ceremony (ready-up, countdown, false starts, score tables) is
//! deliberately absent; runs are per-runner and asynchronous.
//!
//! Race maps are authored for **stock movement plus bunnyhop** — no rocket jumps, no grapple,
//! no double jump, no wall jump — so the mode reports
//! [`stock_movement_only`](GameMode::stock_movement_only), which switches those mechanics off
//! live and keeps their links out of the navmesh (see `ensure_navmesh`). With the graph thus
//! restricted, [`report_routability`](Race::report_routability) can honestly answer the question
//! the harness exists to ask: *can every leg of every route be traversed with bhop and speed
//! jumps alone?* One console line per route, `PASS` or `FAIL` at the first broken leg.
//!
//! Everyone spawns on the active route's start pad (a synthetic spawn marker; `rtx_race_route`
//! selects among a map's routes live). A runner's clock starts when they leave the start box;
//! touching the nodes in order advances them; the finish broadcasts their time and respawns
//! them; `route.timeout` without finishing resets the run with a "timed out on leg k" line —
//! the harness's negative signal.

use glam::Vec3;

use super::{BotIntent, DamageOutcome, GameMode};
use crate::bsp::Bsp;
use crate::defs::{Items, PrintLevel, Weapon};
use crate::entity::EntId;
use crate::game::{cstring, GameState};
use crate::navmesh::{LinkCosts, LinkKind, BAND_FLOOR};
use crate::pmove_sim::PmParams;
use crate::race::{touching, RaceNodeType, RaceRoute, RaceRouteNode};
use crate::raceline::{self, RaceLine};

/// The Race mode descriptor.
pub(crate) struct Race;

/// Classname of the synthetic spawn marker placed on the active route's start pad.
const RACE_SPAWN_CLASS: &str = "race_start_spawn";

/// A runner's progress: the next route node to touch, the running clock, and their best time.
/// Lives in [`super::ModePlayer`]; reset explicitly on finish/death/route-change (a plain
/// respawn deliberately keeps it — the run resumes where it was interrupted only in the sense
/// that death resets it via `on_death`, so a respawned runner always starts over).
#[derive(Default)]
pub(crate) struct RaceSlot {
    /// Index of the next node to touch. `0` means "just spawned" and is normalized to `1`
    /// (node 0 is the start pad the runner is standing on).
    pub next: usize,
    /// World time the run clock started (the runner left the start box); `0` = not yet running.
    pub run_start: f32,
    /// Best finish this map (seconds); `0` = none yet.
    pub best: f32,
}

impl GameMode for Race {
    fn name(&self) -> &'static str {
        "race"
    }

    /// Race maps assume stock QW movement plus bunnyhop; see the module docs.
    fn stock_movement_only(&self) -> bool {
        true
    }

    fn tick(&self, g: &mut GameState) {
        // Build the navmesh even on a bot-less server — the routability report is this mode's
        // whole point and must not depend on rtx_bot_count. Idempotent once built.
        g.ensure_navmesh();
        let route_changed = self.sync_active_route(g);
        self.ensure_start_spawn(g); // (re)place the start pad before anyone respawns onto it
        if route_changed {
            for e in super::players(g) {
                g.entities[e].mode_p.race = RaceSlot::default();
                g.put_client_in_server(e); // re-runs select_spawn onto the moved start pad
            }
        }
        self.report_routability(g);
        self.maybe_optimize_lines(g);
        self.run_touch_machine(g);
    }

    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        // Everyone spawns on the start pad. On a map with no routes there's no marker, so
        // degrade to the stock deathmatch spawns and let the mode idle.
        let spot = g.select_spawn_point_of(RACE_SPAWN_CLASS, Some(e));
        if spot != EntId::WORLD {
            return spot;
        }
        g.select_spawn_point(Some(e))
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        // KTX raceWeaponNo: axe only, nothing else.
        super::Loadout {
            items: Items::AXE,
            health: 100.0,
            max_health: Some(100.0),
            armorvalue: 0.0,
            armortype: 0.0,
            shells: 0.0,
            nails: 0.0,
            rockets: 0.0,
            cells: 0.0,
            weapon: Weapon::Axe,
        }
        .apply(g, e);
    }

    /// Nobody fires in a race (KTX raceWeaponNo; the axe-only loadout makes this belt-and-braces).
    fn weapons_hot(&self, _g: &GameState) -> bool {
        false
    }

    fn player_damage(
        &self,
        g: &mut GameState,
        targ: EntId,
        attacker: EntId,
        inflictor: EntId,
        incoming: f32,
    ) -> DamageOutcome {
        // Only gate players — anything else (doors, buttons) takes normal damage.
        if !g.entities[targ].is_player() {
            return DamageOutcome::pass(incoming);
        }
        // Spawn telefrags must still connect: everyone shares one start pad, and a blocked
        // telefrag would wedge two runners inside each other.
        if g.entities[inflictor].classname() == Some("teledeath") {
            return DamageOutcome::pass(incoming);
        }
        // No player-vs-player. World damage stays — a run that ends in the lava must read as
        // a death (which resets the run via on_death), not a free pass.
        if attacker != targ && g.entities[attacker].is_player() {
            return DamageOutcome::none();
        }
        DamageOutcome::pass(incoming)
    }

    fn on_death(&self, g: &mut GameState, victim: EntId, _attacker: EntId) {
        // Fell / lava / crushed: the next respawn (input-driven, as stock) starts the route over.
        reset_run(g, victim);
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        let Some(target) = self.next_node_origin(g, bot) else {
            // No routes on this map: wander the DM spawns so the bot doesn't freeze in place
            // (a frozen bot trips the stuck-jumper).
            return Some(BotIntent::Move(super::wander_point(g, bot, "info_player_deathmatch", |_| None)));
        };
        Some(BotIntent::Move(target))
    }
}

impl Race {
    /// The active route, if the map has any.
    fn route(g: &GameState) -> Option<&RaceRoute> {
        g.race.routes.get(g.race.active)
    }

    /// Where `bot` should head this frame: its next uncrossed node on the active route.
    fn next_node_origin(&self, g: &GameState, bot: EntId) -> Option<Vec3> {
        let route = Self::route(g)?;
        let next = g.entities[bot].mode_p.race.next.max(1);
        Some(route.nodes.get(next)?.origin)
    }

    /// Track `rtx_race_route` (read live, clamped to the map's route count). On a change,
    /// announce it and report `true` — the caller resets and respawns the runners once the
    /// start pad has moved.
    fn sync_active_route(&self, g: &mut GameState) -> bool {
        if g.race.routes.is_empty() {
            return false;
        }
        let want = (g.host.cvar(c"rtx_race_route").max(0.0) as usize).min(g.race.routes.len() - 1);
        if want == g.race.active {
            return false;
        }
        g.race.active = want;
        let route = &g.race.routes[want];
        g.broadcast(
            PrintLevel::High,
            &format!("rtx: race: route {want} \"{}\" ({})\n", route.name, route.desc),
        );
        true
    }

    /// Keep a synthetic spawn marker sitting on the active route's start node, carrying the
    /// route's spawn view angles. Created on demand (the CTF rune-spawn idiom: a bare
    /// positional entity needs nothing but classname/origin/angles); origin is refreshed every
    /// tick so a route change just slides it.
    fn ensure_start_spawn(&self, g: &mut GameState) {
        let Some(route) = Self::route(g) else {
            return;
        };
        let Some(start) = route.nodes.first() else {
            return;
        };
        let (origin, angles) = (start.origin, Vec3::new(start.pitch, start.yaw, 0.0));
        // Bind the lookup before the match — the iterator borrows `g`, and the create arm
        // needs `&mut g`.
        let existing = g.find_by_classname(RACE_SPAWN_CLASS).next();
        let spot = match existing {
            Some(spot) => spot,
            None => {
                let e = g.spawn();
                g.entities[e].classname = Some(RACE_SPAWN_CLASS.into());
                e
            }
        };
        let v = &mut g.entities[spot].v;
        v.origin = origin;
        v.angles = angles;
    }

    /// Advance every live runner through the route: start their clock as they leave the start
    /// box, step `next` on each node touch, broadcast + respawn on the finish, and reset with a
    /// named leg on timeout — the harness's negative signal.
    fn run_touch_machine(&self, g: &mut GameState) {
        // Clone the active route (tiny: ≤20 nodes) so the loop below is free to mutate
        // runner state and respawn players without fighting the `g.race` borrow.
        let Some(route) = Self::route(g).cloned() else {
            return;
        };
        if route.nodes.len() < 2 {
            return;
        }
        let now = g.time();
        let last = route.nodes.len() - 1;
        for e in super::players(g) {
            let ent = &g.entities[e];
            if !ent.is_alive() {
                continue;
            }
            let origin = ent.v.origin;
            let slot = &g.entities[e].mode_p.race;
            let next = slot.next.max(1).min(last);
            let run_start = slot.run_start;

            // The clock starts the moment the runner leaves the start box — same predicate as
            // the checkpoint touches, and it excludes spawn churn from the measured time.
            if run_start == 0.0 {
                if !touching(origin, &route.nodes[0]) {
                    g.entities[e].mode_p.race.run_start = now;
                    g.entities[e].mode_p.race.next = next;
                }
                continue;
            }

            if touching(origin, &route.nodes[next]) {
                let name = g.netname_of(e);
                let elapsed = now - run_start;
                if next == last {
                    let best = g.entities[e].mode_p.race.best;
                    let note = if best == 0.0 || elapsed < best {
                        g.entities[e].mode_p.race.best = elapsed;
                        " (personal best)"
                    } else {
                        ""
                    };
                    g.broadcast(
                        PrintLevel::High,
                        &format!("{name} finished \"{}\" in {elapsed:.2}s{note}\n", route.name),
                    );
                    reset_run(g, e);
                    g.put_client_in_server(e);
                } else {
                    let total = route.nodes.len();
                    g.broadcast(
                        PrintLevel::Medium,
                        &format!("{name} checkpoint {next}/{} ({elapsed:.1}s)\n", total - 2),
                    );
                    g.entities[e].mode_p.race.next = next + 1;
                }
                continue;
            }

            // Timeout: the route's own budget (clamped 1..=999 at load) is the map author's
            // intended bound; a bot that can't fly a leg surfaces here with the leg named.
            if now - run_start > route.timeout {
                let name = g.netname_of(e);
                g.broadcast(
                    PrintLevel::High,
                    &format!("{name} timed out on leg {next} of \"{}\" ({:.0}s)\n", route.name, route.timeout),
                );
                reset_run(g, e);
                g.put_client_in_server(e);
            }
        }
    }

    /// Once per map, as soon as the navmesh lands: can every leg of every route be traversed
    /// with race-legal movement? The graph was built under
    /// [`stock_movement_only`](GameMode::stock_movement_only), so hook / double-jump /
    /// rocket-jump links don't exist and `find_path == None` is a truthful FAIL — but a graph
    /// built *before* a mid-map switch to race still carries them, so paths are also scanned
    /// for banned kinds (the fix is a map restart). One conprint line per route.
    fn report_routability(&self, g: &mut GameState) {
        if g.race.routability_reported || g.race.routes.is_empty() {
            return;
        }
        if g.nav.graph.is_none() {
            return; // still building; try again next tick
        }
        g.race.routability_reported = true;
        let graph = g.nav.graph.as_ref().unwrap();
        let host = g.host;
        if !host.cvar_bool(c"rtx_bot_bhop") {
            host.conprint(&cstring(
                "rtx: race: warning — rtx_bot_bhop 0: the navmesh has no speed-jump links, routes will likely FAIL\n",
            ));
        }
        // A route node floats at its author's placement; snapping much farther than a couple of
        // grid columns means it's off the walkable mesh entirely.
        const OFF_MESH: f32 = 80.0;
        for (ri, route) in g.race.routes.iter().enumerate() {
            let label = format!("route {ri} \"{}\"", route.name);
            let mut est = 0.0f32;
            // Speed carried between legs: a checkpoint touch doesn't stop the runner, so a leg's
            // exit band feeds the next leg's entry speed. The first leg starts at run speed.
            let mut carry = BAND_FLOOR[0];
            let mut verdict: Option<String> = None; // None = passing so far
            for (i, pair) in route.nodes.windows(2).enumerate() {
                let leg = format!("leg {} ({} -> {})", i + 1, node_label(&pair[0], i), node_label(&pair[1], i + 1));
                let ends: Vec<_> = pair
                    .iter()
                    .map(|n| graph.nearest(n.origin).filter(|&c| (graph.cell_origin(c) - n.origin).length() <= OFF_MESH))
                    .collect();
                let (Some(a), Some(b)) = (ends[0], ends[1]) else {
                    verdict = Some(format!("FAIL at {leg}: node off the navmesh"));
                    break;
                };
                // Banded so a leg reachable only by carrying speed from the previous one (a chained
                // speed jump) is credited — the whole point race maps exercise.
                let Some(route) = graph.find_path_banded(a, b, carry, &LinkCosts::default()) else {
                    verdict = Some(format!("FAIL at {leg}: no path"));
                    break;
                };
                if route
                    .links
                    .iter()
                    .any(|&li| matches!(graph.link_kind(li), LinkKind::Hook | LinkKind::RocketJump | LinkKind::DoubleJump))
                {
                    verdict = Some(format!(
                        "FAIL at {leg}: path needs a non-race move — restart the map to rebuild the navmesh for race"
                    ));
                    break;
                }
                est += route.cost;
                carry = BAND_FLOOR[route.end_band as usize];
            }
            let line = match verdict {
                None => {
                    let slow = if est > route.timeout {
                        format!(" — estimate exceeds the {:.0}s timeout", route.timeout)
                    } else {
                        String::new()
                    };
                    format!("rtx: race: {label}: PASS ({} legs, est {est:.1}s{slow})\n", route.nodes.len() - 1)
                }
                Some(v) => format!("rtx: race: {label}: {v}\n"),
            };
            host.conprint(&cstring(&line));
        }
    }

    /// Offline racing-line optimization (`rtx_race_optimize` iterations, in thousands): once per map,
    /// after the navmesh and routability report land, TAS a line for each route on a worker thread
    /// (same background pattern as the navmesh build) and poll it in. Default off (`0`), so this is
    /// inert unless an admin opts in. Lines live in memory for the map's lifetime; a disk cache
    /// (`race/lines/{map}.line`) is deferred pending a verified module file-write ABI.
    fn maybe_optimize_lines(&self, g: &mut GameState) {
        // Poll an in-flight optimization.
        if let Some(rx) = g.race.opt_pending.as_ref() {
            match rx.try_recv() {
                Ok(lines) => {
                    g.race.opt_pending = None;
                    let mut out = vec![RaceLine::default(); g.race.routes.len()];
                    let mut done = 0;
                    for (ri, line) in lines {
                        if ri < out.len() && line.points.len() >= 2 {
                            out[ri] = line;
                            done += 1;
                        }
                    }
                    g.race.lines = out;
                    g.host
                        .conprint(&cstring(&format!("rtx: race: optimized {done} racing line(s)\n")));
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
                Err(std::sync::mpsc::TryRecvError::Disconnected) => g.race.opt_pending = None,
            }
            return;
        }
        // Kick once, after the routability report, when opted in and the graph is ready.
        let iters = (g.host.cvar(c"rtx_race_optimize").max(0.0) * 1000.0) as u32;
        if g.race.opt_started || iters == 0 || !g.race.routability_reported || g.nav.graph.is_none() {
            return;
        }
        g.race.opt_started = true;

        // Build each route's control polyline from the banded path (main thread — needs the graph),
        // threading each leg's exit band into the next leg's entry speed like the routability report.
        let graph = g.nav.graph.as_ref().unwrap();
        let mut jobs: Vec<OptJob> = Vec::new();
        for (ri, route) in g.race.routes.iter().enumerate() {
            let mut poly = route.nodes.first().map(|n| vec![n.origin]).unwrap_or_default();
            let mut carry = BAND_FLOOR[0];
            let mut ok = !route.nodes.is_empty();
            for pair in route.nodes.windows(2) {
                let snap = |n: &RaceRouteNode| {
                    graph.nearest(n.origin).filter(|&c| (graph.cell_origin(c) - n.origin).length() <= 80.0)
                };
                let (Some(a), Some(b)) = (snap(&pair[0]), snap(&pair[1])) else {
                    ok = false;
                    break;
                };
                let Some(r) = graph.find_path_banded(a, b, carry, &LinkCosts::default()) else {
                    ok = false;
                    break;
                };
                poly.extend(r.links.iter().map(|&li| graph.cell_origin(graph.link_target(li))));
                poly.push(pair[1].origin);
                carry = BAND_FLOOR[r.end_band as usize];
            }
            if ok && poly.len() >= 2 {
                // FNV-1a over the map name, mixed with the route index → deterministic per-map seed.
                let seed = route
                    .name
                    .bytes()
                    .chain(g.level.mapname.bytes())
                    .fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619))
                    ^ ri as u32;
                jobs.push((ri, poly, route.nodes.clone(), route.timeout, seed));
            }
        }
        if jobs.is_empty() {
            return;
        }

        let pm = PmParams {
            gravity: g.host.cvar(c"sv_gravity").max(1.0),
            accel: pos_or(g.host.cvar(c"sv_accelerate"), 10.0),
            friction: pos_or(g.host.cvar(c"sv_friction"), 4.0),
            stopspeed: pos_or(g.host.cvar(c"sv_stopspeed"), 100.0),
            maxspeed: pos_or(g.host.cvar(c"sv_maxspeed"), 320.0),
        };
        let Some(bytes) = g.host.read_file(&cstring(&format!("maps/{}.bsp", g.level.mapname))) else {
            return;
        };
        let njobs = jobs.len();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let lines = match Bsp::parse(&bytes) {
                Some(bsp) => jobs
                    .iter()
                    .map(|(ri, poly, nodes, timeout, seed)| {
                        (*ri, raceline::optimize_route(&bsp, poly, nodes, &pm, *timeout, *seed, iters))
                    })
                    .collect(),
                None => Vec::new(),
            };
            let _ = tx.send(lines);
        });
        g.race.opt_pending = Some(rx);
        g.host
            .dprint(&cstring(&format!("rtx: race: optimizing {njobs} racing line(s) in background...\n")));
    }
}

/// A positive cvar value, or a fallback when it's unset/nonpositive.
fn pos_or(v: f32, fallback: f32) -> f32 {
    if v > 0.0 {
        v
    } else {
        fallback
    }
}

/// How far a bot may drift from the optimized line before it's abandoned for navmesh recovery.
const RACE_LINE_STRAY: f32 = 160.0;

/// One racing-line optimization job handed to the worker: `(route index, control polyline, route
/// nodes, timeout seconds, deterministic seed)`.
type OptJob = (usize, Vec<Vec3>, Vec<RaceRouteNode>, f32, u32);

impl GameState {
    /// The look-ahead point on the active route's optimized racing line for a bot at `origin`, or
    /// `None` when there's no line, the feature is off, or the bot has strayed too far (recover on
    /// the navmesh instead). Consulted by `run_bot` to bias the bhop bearing in race mode. Inert
    /// unless `rtx_race_optimize` produced a line — the data's presence is the gate, so this stays
    /// `None` in every non-race mode (their `race.lines` is empty).
    pub(crate) fn race_line_lookahead(&self, origin: Vec3) -> Option<Vec3> {
        if !self.host.cvar_bool(c"rtx_race_line") {
            return None;
        }
        let line = self.race.lines.get(self.race.active)?;
        if line.points.len() < 2 {
            return None;
        }
        let (mut bi, mut bd) = (0usize, f32::INFINITY);
        for (i, p) in line.points.iter().enumerate() {
            let d = (p.pos - origin).length_squared();
            if d < bd {
                bd = d;
                bi = i;
            }
        }
        if bd > RACE_LINE_STRAY * RACE_LINE_STRAY {
            return None; // off the line — let the navmesh route recover it
        }
        Some(line.points[(bi + 2).min(line.points.len() - 1)].pos)
    }
}

/// Reset a runner's progress to "standing on the start pad, clock not running". Their best
/// time survives (it's a per-map stat, not per-run).
fn reset_run(g: &mut GameState, e: EntId) {
    let slot = &mut g.entities[e].mode_p.race;
    slot.next = 1;
    slot.run_start = 0.0;
}

/// A human-readable node name for the routability report ("start", "checkpoint 2", "finish").
fn node_label(node: &RaceRouteNode, index: usize) -> String {
    match node.kind {
        RaceNodeType::Start => "start".into(),
        RaceNodeType::End => "finish".into(),
        RaceNodeType::Checkpoint => format!("checkpoint {index}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::race::node_box;

    fn node(origin: Vec3, size: Vec3) -> RaceRouteNode {
        RaceRouteNode {
            kind: RaceNodeType::Checkpoint,
            origin,
            pitch: 0.0,
            yaw: 0.0,
            size,
        }
    }

    #[test]
    fn default_node_box_is_player_hull() {
        let n = node(Vec3::new(100.0, 200.0, 300.0), Vec3::ZERO);
        let (min, max) = node_box(&n);
        assert_eq!(min, Vec3::new(84.0, 184.0, 276.0));
        assert_eq!(max, Vec3::new(116.0, 216.0, 332.0));
    }

    #[test]
    fn sized_node_box_is_centered_extent() {
        // race17's checkpoint: "size 128 8 64" — a thin gate plane.
        let n = node(Vec3::ZERO, Vec3::new(128.0, 8.0, 64.0));
        let (min, max) = node_box(&n);
        assert_eq!(min, Vec3::new(-64.0, -4.0, -32.0));
        assert_eq!(max, Vec3::new(64.0, 4.0, 32.0));
    }

    #[test]
    fn touching_is_hull_overlap_not_center_distance() {
        let n = node(Vec3::ZERO, Vec3::ZERO);
        // Player hull (±16 xy) meets node hull (±16 xy): overlap reaches out to 32 in x.
        assert!(touching(Vec3::new(31.0, 0.0, 0.0), &n));
        assert!(!touching(Vec3::new(33.0, 0.0, 0.0), &n));
        // Vertically the boxes span -24..32 each: overlap holds to a 56u offset.
        assert!(touching(Vec3::new(0.0, 0.0, 55.0), &n));
        assert!(!touching(Vec3::new(0.0, 0.0, 57.0), &n));
        // A thin sized gate is touchable from the side the hull reaches across.
        let gate = node(Vec3::ZERO, Vec3::new(128.0, 8.0, 64.0));
        assert!(touching(Vec3::new(0.0, 19.0, 0.0), &gate));
        assert!(!touching(Vec3::new(0.0, 21.0, 0.0), &gate));
    }
}
