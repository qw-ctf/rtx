// SPDX-License-Identifier: AGPL-3.0-or-later

//! KTX race routes — the start→checkpoints→finish courses raced on `race*`/`ztricks*` maps.
//!
//! A route arrives from one of two sources, mirroring ktx (`race.c`):
//! - an external command file `race/routes/{mapname}.route` ([`parse_route_file`]), the format
//!   ktx ships for maps without embedded data (race1–10, ztricks, ztricks2);
//! - `race_route_start`/`race_route_marker` entities embedded in the map's entity string
//!   ([`routes_from_markers`]), chained `target` → `targetname` (race11–20, race32c).
//!
//! Both sources are parsed into the same [`RaceRoute`] model at map load
//! ([`GameState::load_race_routes`]) regardless of the active mode — like the navmesh, this is
//! pure per-map derived data, and `rtx_mode race` may be switched on live after the load. The
//! parsers are pure and unit-tested; the `GameState` glue at the bottom does the I/O and logging.

use glam::Vec3;

use crate::defs;
use crate::entity::EntId;
use crate::game::{cstring, GameState};

/// KTX parity caps (`MAX_ROUTES` / `MAX_ROUTE_NODES` in ktx `progs.h`).
pub const MAX_ROUTES: usize = 20;
pub const MAX_ROUTE_NODES: usize = 20;

/// Whether weapons may be used on a route (`raceWeaponNo`/`Allowed`/`2s`; ktx ints 1/2/3).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RaceWeaponMode {
    No,
    Allowed,
    After2s,
}

/// Whether a racer may move before "GO" (`raceFalseStartNo`/`Yes`; ktx ints 1/2).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RaceFalseStartMode {
    No,
    Yes,
}

/// A node's role in the route: first = start pad, last = finish, middle = checkpoints.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RaceNodeType {
    Start,
    Checkpoint,
    End,
}

/// `race_flags` a route file may stamp onto named teleporters: touching one fails or
/// finishes the run. Recorded for a future full race ruleset; not yet applied to entities.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RaceTeleportFlag {
    Fail,
    End,
}

/// One route node: where it is, the spawn view angles (meaningful on the start node), and its
/// touch-box size (`ZERO` = ktx's default player-hull box; else a full extent, box = `origin ± size/2`).
#[derive(Clone, Debug)]
pub struct RaceRouteNode {
    pub kind: RaceNodeType,
    pub origin: Vec3,
    pub pitch: f32,
    pub yaw: f32,
    pub size: Vec3,
}

/// A node's touch box in world space: KTX gives sized nodes a box of `origin ± size/2` and
/// default (zero-size) nodes the player hull box (ktx race.c:1705-1712). Shared by the live touch
/// machine ([`crate::mode::race`]) and the offline race-line rollout ([`crate::raceline`]).
pub fn node_box(node: &RaceRouteNode) -> (Vec3, Vec3) {
    if node.size == Vec3::ZERO {
        (node.origin + defs::VEC_HULL_MIN, node.origin + defs::VEC_HULL_MAX)
    } else {
        (node.origin - node.size * 0.5, node.origin + node.size * 0.5)
    }
}

/// Whether a player standing at `origin` touches `node` — the player hull box overlapping the
/// node's box, the same predicate the engine's trigger touch would use.
pub fn touching(origin: Vec3, node: &RaceRouteNode) -> bool {
    let (pmin, pmax) = (origin + defs::VEC_HULL_MIN, origin + defs::VEC_HULL_MAX);
    let (nmin, nmax) = node_box(node);
    pmin.x <= nmax.x && pmax.x >= nmin.x && pmin.y <= nmax.y && pmax.y >= nmin.y && pmin.z <= nmax.z && pmax.z >= nmin.z
}

/// One start→finish course. `timeout` is seconds allowed for a run (ktx clamps to 1..=999).
#[derive(Clone, Debug)]
pub struct RaceRoute {
    pub name: String,
    pub desc: String,
    pub timeout: f32,
    pub weapon: RaceWeaponMode,
    pub falsestart: RaceFalseStartMode,
    pub nodes: Vec<RaceRouteNode>,
}

impl Default for RaceRoute {
    /// The ktx per-route defaults seeded by `race_route_add_start` (race.c:560-562).
    fn default() -> Self {
        RaceRoute {
            name: String::new(),
            desc: String::new(),
            timeout: 20.0,
            weapon: RaceWeaponMode::Allowed,
            falsestart: RaceFalseStartMode::No,
            nodes: Vec::new(),
        }
    }
}

/// The map's loaded race data, reset and rebuilt each map load (like the navmesh).
#[derive(Default)]
pub struct RaceState {
    pub routes: Vec<RaceRoute>,
    /// Teleport flag rules from the route file, keyed by the teleporter's `target` name.
    pub teleport_flags: Vec<(String, RaceTeleportFlag)>,
    /// The resolved `rtx_race_route` index the race mode is currently running.
    pub active: usize,
    /// One-shot latch for the race mode's navmesh routability report.
    pub routability_reported: bool,
    /// Offline-optimized racing lines, parallel to `routes` (empty until the optimizer runs — only
    /// when `rtx_race_optimize > 0`). Bots track their active route's line when `rtx_race_line` is on.
    pub lines: Vec<crate::raceline::RaceLine>,
    /// In-flight racing-line optimization on a worker thread (`(route index, line)` per route), and a
    /// one-shot latch so it's kicked at most once per map. See [`crate::mode::race`].
    pub opt_pending: Option<std::sync::mpsc::Receiver<Vec<(usize, crate::raceline::RaceLine)>>>,
    pub opt_started: bool,
}

// --- route file parsing (`race/routes/{mapname}.route`) ---

/// A parse error with the 1-based line it occurred on. Per ktx semantics, any error voids the
/// whole file (every file route is discarded; embedded entity routes are unaffected).
#[derive(Debug, PartialEq)]
pub struct RouteFileError {
    pub line: usize,
    pub msg: String,
}

/// A successfully parsed route file. `warnings` are non-fatal notes (e.g. the route cap).
#[derive(Default, Debug)]
pub struct RouteFile {
    pub routes: Vec<RaceRoute>,
    pub teleport_flags: Vec<(String, RaceTeleportFlag)>,
    pub warnings: Vec<String>,
}

/// Parse the ktx route-command format (race.c:3839-4193): one command per line, `//` comments,
/// quoted strings as single tokens. Routes open with `race_route_add_start`, gain nodes and
/// settings, and commit on `race_route_add_end`; a file ending mid-definition drops the
/// uncommitted route (as ktx does, with a warning here).
pub fn parse_route_file(text: &str) -> Result<RouteFile, RouteFileError> {
    let mut out = RouteFile::default();
    // The route under construction; `Some` while between add_start and add_end.
    let mut current: Option<RaceRoute> = None;
    let err = |line: usize, msg: String| Err(RouteFileError { line, msg });

    for (idx, raw) in text.lines().enumerate() {
        let line = idx + 1;
        let args = tokenize(raw);
        let Some(cmd) = args.first() else {
            continue;
        };
        // Commands valid only inside a route definition borrow `current` mutably here; the
        // "outside a definition" error text mirrors ktx's.
        match cmd.as_str() {
            "race_route_add_start" => {
                if current.is_some() {
                    return err(line, "race_route_add_start in route definition".into());
                }
                if out.routes.len() >= MAX_ROUTES {
                    // Not an error in ktx: earlier routes stay valid, the rest are ignored.
                    out.warnings
                        .push(format!("#{line}: routes ignored, limit is {MAX_ROUTES} routes/map"));
                    break;
                }
                current = Some(RaceRoute::default());
            }
            "race_add_route_node" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_add_route_node outside of route definition".into());
                };
                if args.len() != 6 {
                    return err(
                        line,
                        format!("race_add_route_node should have 5 arguments, found {}", args.len() - 1),
                    );
                }
                if route.nodes.len() >= MAX_ROUTE_NODES {
                    // ktx drops the node silently (race_add_route_node returns NULL when full);
                    // surface it, since a truncated route is almost certainly a mistake.
                    out.warnings
                        .push(format!("#{line}: node ignored, limit is {MAX_ROUTE_NODES} nodes/route"));
                    continue;
                }
                let n: Vec<f32> = args[1..6].iter().map(|a| parse_f32(a)).collect();
                // First node is the start; every later node lands as the end, demoting the
                // previous non-start node to a checkpoint (race.c:3911-3923).
                let kind = if route.nodes.is_empty() {
                    RaceNodeType::Start
                } else {
                    if route.nodes.len() > 1 {
                        route.nodes.last_mut().unwrap().kind = RaceNodeType::Checkpoint;
                    }
                    RaceNodeType::End
                };
                route.nodes.push(RaceRouteNode {
                    kind,
                    origin: Vec3::new(n[0], n[1], n[2]),
                    pitch: n[3],
                    yaw: n[4],
                    size: Vec3::ZERO,
                });
            }
            "race_set_route_name" => {
                if args.len() != 3 {
                    return err(
                        line,
                        format!("race_set_route_name should have 2 arguments, found {}", args.len() - 1),
                    );
                }
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_name outside of route definition".into());
                };
                route.name = args[1].clone();
                route.desc = unescape_desc(&args[2]);
            }
            "race_set_route_timeout" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_timeout outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_timeout: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                let t = parse_f32(&args[1]);
                if t > 0.0 {
                    route.timeout = t.clamp(1.0, 999.0);
                }
            }
            "race_set_route_weapon_mode" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_weapon_mode outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_weapon_mode: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                route.weapon = match args[1].as_str() {
                    "raceWeaponNo" => RaceWeaponMode::No,
                    "raceWeaponAllowed" => RaceWeaponMode::Allowed,
                    "raceWeapon2s" => RaceWeaponMode::After2s,
                    other => {
                        return err(line, format!("race_set_route_weapon_mode: invalid argument {other}"));
                    }
                };
            }
            "race_set_route_falsestart_mode" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_falsestart_mode outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_falsestart_mode: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                route.falsestart = match args[1].as_str() {
                    "raceFalseStartNo" => RaceFalseStartMode::No,
                    "raceFalseStartYes" => RaceFalseStartMode::Yes,
                    other => {
                        return err(line, format!("race_set_route_falsestart_mode: invalid argument {other}"));
                    }
                };
            }
            "race_set_node_size" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_node_size outside of route definition".into());
                };
                if args.len() != 4 {
                    return err(
                        line,
                        format!("race_set_node_size: expected 3 arguments, found {}", args.len() - 1),
                    );
                }
                let Some(node) = route.nodes.last_mut() else {
                    return err(line, "race_set_node_size: no node to amend".into());
                };
                node.size = Vec3::new(parse_f32(&args[1]), parse_f32(&args[2]), parse_f32(&args[3]));
            }
            "race_set_teleport_flags_by_name" => {
                if current.is_some() {
                    return err(line, "race_set_teleport_flags_by_name inside route definition".into());
                }
                if args.len() != 3 {
                    return err(
                        line,
                        format!("race_set_teleport_flags_by_name: expected 2 arguments, found {}", args.len() - 1),
                    );
                }
                // Unknown flag names are silently ignored, as in ktx.
                let flag = match args[2].as_str() {
                    "RACEFLAG_TOUCH_RACEFAIL" => Some(RaceTeleportFlag::Fail),
                    "RACEFLAG_TOUCH_RACEEND" => Some(RaceTeleportFlag::End),
                    _ => None,
                };
                if let Some(flag) = flag {
                    out.teleport_flags.push((args[1].clone(), flag));
                }
            }
            "race_route_add_end" => {
                let Some(route) = current.take() else {
                    return err(line, "race_route_add_end outside of route definition".into());
                };
                out.routes.push(route);
            }
            other => {
                return err(line, format!("unknown route instruction {other}"));
            }
        }
    }
    if current.is_some() {
        out.warnings
            .push("file ended inside a route definition; last route dropped".into());
    }
    Ok(out)
}

/// Split a route-file line into tokens: whitespace-separated, `"quoted strings"` as one token
/// (quotes stripped), `//` starting a comment outside quotes.
fn tokenize(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = line.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '"' {
            chars.next();
            let mut tok = String::new();
            for c in chars.by_ref() {
                if c == '"' {
                    break;
                }
                tok.push(c);
            }
            out.push(tok);
        } else {
            let mut tok = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_whitespace() || c == '"' {
                    break;
                }
                tok.push(c);
                chars.next();
            }
            if let Some(rest) = tok.find("//") {
                tok.truncate(rest);
                if !tok.is_empty() {
                    out.push(tok);
                }
                return out;
            }
            out.push(tok);
        }
    }
    out
}

/// Unescape a route description: `\\` → `\`, and ktx's colored-text escape `\abc` (three
/// digits) → the byte `16a + 8b + c` (race.c:3954-3972), kept as a Latin-1 codepoint.
fn unescape_desc(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::new();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'\\' && i + 1 < b.len() && b[i + 1] == b'\\' {
            out.push('\\');
            i += 2;
        } else if b[i] == b'\\'
            && i + 3 < b.len()
            && b[i + 1].is_ascii_digit()
            && b[i + 2].is_ascii_digit()
            && b[i + 3].is_ascii_digit()
        {
            let v = 16 * (b[i + 1] - b'0') as u32 + 8 * (b[i + 2] - b'0') as u32 + (b[i + 3] - b'0') as u32;
            out.push(char::from_u32(v).unwrap_or('?'));
            i += 4;
        } else {
            out.push(b[i] as char);
            i += 1;
        }
    }
    out
}

/// Parse a float the way ktx's `atof` does: garbage → `0.0`.
fn parse_f32(s: &str) -> f32 {
    s.trim().parse().unwrap_or(0.0)
}

// --- embedded-entity routes (`race_route_start` → `race_route_marker` chains) ---

/// One `race_route_start` or `race_route_marker` entity, reduced to the fields the chain
/// builder needs (gathered on the main thread, then processed purely — the `collect_plats`
/// pattern). `pitch`/`yaw` are the entity's `race_route_start_pitch`/`_yaw` keys (zero on markers).
pub struct RaceMarkerInfo {
    pub origin: Vec3,
    pub size: Vec3,
    pub target: Option<Box<str>>,
    pub targetname: Option<Box<str>>,
    pub pitch: f32,
    pub yaw: f32,
}

/// A `race_route_start` entity: the chain head plus the route-wide settings it carries.
pub struct RaceStartInfo {
    pub marker: RaceMarkerInfo,
    pub name: String,
    pub desc: String,
    pub timeout: f32,
    pub weapon_mode: i32,
    pub falsestart_mode: i32,
}

/// Build routes from embedded map entities, mirroring ktx `race_route_create`
/// (race.c:3652-3792): each `race_route_start` heads one route; the chain follows
/// `target` → `targetname` through the markers until a target-less node. Invalid chains skip
/// that route with a warning (never fatal). One deliberate fix over ktx: each node's own
/// pitch/yaw is used (ktx's field table aliases `race_route_start_pitch` onto yaw —
/// g_spawn.c:150 — a latent bug no shipped map exercises; all use 0/0).
pub fn routes_from_markers(starts: &[RaceStartInfo], markers: &[RaceMarkerInfo]) -> (Vec<RaceRoute>, Vec<String>) {
    let mut routes = Vec::new();
    let mut warnings = Vec::new();

    for start in starts {
        let label = if start.name.is_empty() { "(unnamed)" } else { &start.name };
        if start.name.is_empty() || start.desc.is_empty() {
            warnings.push("route name/description not specified".into());
            continue;
        }
        let weapon = match start.weapon_mode {
            1 => RaceWeaponMode::No,
            2 => RaceWeaponMode::Allowed,
            3 => RaceWeaponMode::After2s,
            _ => {
                warnings.push(format!("route \"{label}\": weapon mode not valid"));
                continue;
            }
        };
        let falsestart = match start.falsestart_mode {
            1 => RaceFalseStartMode::No,
            2 => RaceFalseStartMode::Yes,
            _ => {
                warnings.push(format!("route \"{label}\": falsestart mode not valid"));
                continue;
            }
        };

        // Walk the marker chain. `chain` holds indices into `markers` (usize::MAX = the start
        // entity itself) purely for cycle detection.
        let mut nodes: Vec<&RaceMarkerInfo> = vec![&start.marker];
        let mut seen: Vec<usize> = vec![usize::MAX];
        let mut cursor = &start.marker;
        let mut valid = true;
        while let Some(target) = cursor.target.as_deref() {
            if nodes.len() >= MAX_ROUTE_NODES {
                warnings.push(format!("route \"{label}\": route too long"));
                valid = false;
                break;
            }
            let Some(mi) = markers
                .iter()
                .position(|m| m.targetname.as_deref() == Some(target))
            else {
                // ktx distinguishes a dangling target (chain just ends, route kept) from a
                // non-marker target (route skipped); having gathered only markers we can't
                // tell them apart, so end the chain and let the ≥2-node check decide.
                warnings.push(format!("route \"{label}\": target '{target}' matches no race_route_marker; route ends there"));
                break;
            };
            if seen.contains(&mi) {
                warnings.push(format!("route \"{label}\": circular route detected"));
                valid = false;
                break;
            }
            seen.push(mi);
            nodes.push(&markers[mi]);
            cursor = &markers[mi];
        }
        if !valid {
            continue;
        }
        if nodes.len() < 2 {
            warnings.push(format!("route \"{label}\": route too short ({} nodes)", nodes.len()));
            continue;
        }

        let last = nodes.len() - 1;
        routes.push(RaceRoute {
            name: start.name.clone(),
            desc: start.desc.clone(),
            // ktx applies the 1..=999 clamp unconditionally on this path (an absent key
            // becomes a 1s timeout); every shipped map sets one.
            timeout: start.timeout.clamp(1.0, 999.0),
            weapon,
            falsestart,
            nodes: nodes
                .iter()
                .enumerate()
                .map(|(i, m)| RaceRouteNode {
                    kind: match i {
                        0 => RaceNodeType::Start,
                        i if i == last => RaceNodeType::End,
                        _ => RaceNodeType::Checkpoint,
                    },
                    origin: m.origin,
                    pitch: m.pitch,
                    yaw: m.yaw,
                    size: m.size,
                })
                .collect(),
        });
    }
    (routes, warnings)
}

// --- GameState glue: load, validate and log the map's routes ---

impl GameState {
    /// Load the map's race routes from both sources, called at the end of `load_entities`
    /// (mirroring ktx, which loads the file at the end of `G_SpawnEntitiesFromString` and
    /// builds entity routes a frame later). File routes come first, then entity routes append,
    /// both capped at [`MAX_ROUTES`] total. Always runs — the data is per-map derived state
    /// like the navmesh, and `rtx_mode race` can be switched on after load.
    pub(crate) fn load_race_routes(&mut self) {
        let path = format!("race/routes/{}.route", self.level.mapname);
        let mut from_file = 0;
        if let Some(bytes) = self.host.read_file(&cstring(&path)) {
            match parse_route_file(&String::from_utf8_lossy(&bytes)) {
                Ok(file) => {
                    for w in &file.warnings {
                        self.host.conprint(&cstring(&format!("rtx: race: {path}: {w}\n")));
                    }
                    from_file = file.routes.len();
                    self.race.routes = file.routes;
                    self.race.teleport_flags = file.teleport_flags;
                }
                Err(e) => {
                    // ktx semantics: one bad line voids every file route.
                    self.host.conprint(&cstring(&format!(
                        "rtx: race: {path}:{}: {} — ignoring all file routes\n",
                        e.line, e.msg
                    )));
                }
            }
        }

        let (starts, markers) = self.collect_race_markers();
        let (routes, warnings) = routes_from_markers(&starts, &markers);
        for w in &warnings {
            self.host.conprint(&cstring(&format!("rtx: race: {}: {w}\n", self.level.mapname)));
        }
        let from_entities = routes.len().min(MAX_ROUTES.saturating_sub(self.race.routes.len()));
        if from_entities < routes.len() {
            self.host.conprint(&cstring(&format!(
                "rtx: race: routes ignored, limit is {MAX_ROUTES} routes/map\n"
            )));
        }
        self.race.routes.extend(routes.into_iter().take(from_entities));

        if self.race.routes.is_empty() {
            return; // the overwhelmingly common non-race map: stay silent
        }
        self.host.conprint(&cstring(&format!(
            "rtx: race: {} route(s) loaded for {} ({from_file} from {path}, {from_entities} from map entities)\n",
            self.race.routes.len(),
            self.level.mapname,
        )));
        self.validate_routes();
    }

    /// Gather the embedded `race_route_start`/`race_route_marker` entities into plain info
    /// structs for [`routes_from_markers`], then free them — they are inert data carriers,
    /// consumed here (ktx schedules them for removal the frame after route creation).
    fn collect_race_markers(&mut self) -> (Vec<RaceStartInfo>, Vec<RaceMarkerInfo>) {
        let info = |e: &crate::entity::Entity| RaceMarkerInfo {
            origin: e.v.origin,
            size: e.race.size,
            target: e.target.clone(),
            targetname: e.targetname.clone(),
            pitch: e.race.start_pitch,
            yaw: e.race.start_yaw,
        };
        let start_ids: Vec<EntId> = self.find_by_classname("race_route_start").collect();
        let marker_ids: Vec<EntId> = self.find_by_classname("race_route_marker").collect();
        let starts = start_ids
            .iter()
            .map(|&id| {
                let e = &self.entities[id];
                RaceStartInfo {
                    marker: info(e),
                    name: e.race.name.as_deref().unwrap_or("").to_owned(),
                    desc: e.race.desc.as_deref().unwrap_or("").to_owned(),
                    timeout: e.race.timeout,
                    weapon_mode: e.race.weapon_mode as i32,
                    falsestart_mode: e.race.falsestart_mode as i32,
                }
            })
            .collect();
        let markers = marker_ids.iter().map(|&id| info(&self.entities[id])).collect();
        for id in start_ids.into_iter().chain(marker_ids) {
            self.free(id);
        }
        (starts, markers)
    }

    /// Sanity-check the loaded routes once at load: shape (a start, an end, ≥2 nodes — the
    /// file grammar can commit degenerate routes ktx never validates) and node placement
    /// (an origin inside solid or lava is a map/route bug a route author needs to see).
    /// The navmesh routability check is separate — it belongs to the race mode, which knows
    /// which link kinds are race-legal.
    fn validate_routes(&mut self) {
        for ri in 0..self.race.routes.len() {
            let route = &self.race.routes[ri];
            let label = format!("route {ri} \"{}\"", route.name);
            let shape_ok = route.nodes.len() >= 2
                && route.nodes.first().is_some_and(|n| n.kind == RaceNodeType::Start)
                && route.nodes.last().is_some_and(|n| n.kind == RaceNodeType::End);
            let mut messages: Vec<String> = Vec::new();
            if !shape_ok {
                messages.push(format!(
                    "rtx: race: warning: {label} lacks a proper start/finish ({} nodes)\n",
                    route.nodes.len()
                ));
            }
            for (ni, node) in route.nodes.iter().enumerate() {
                let contents = self.host.pointcontents(node.origin);
                let what = if contents == defs::Content::Solid.as_f32() {
                    "inside solid"
                } else if contents == defs::Content::Lava.as_f32() {
                    "in lava"
                } else {
                    continue; // water/slime are legitimate on race maps
                };
                let o = node.origin;
                messages.push(format!(
                    "rtx: race: warning: {label} node {ni} origin ({} {} {}) is {what}\n",
                    o.x, o.y, o.z
                ));
            }
            for m in messages {
                self.host.conprint(&cstring(&m));
            }
            let route = &self.race.routes[ri];
            self.host.dprint(&cstring(&format!(
                "rtx: race:   [{ri}] \"{}\" ({}): {} nodes, timeout {:.0}s, weapons {}, falsestart {}\n",
                route.name,
                route.desc,
                route.nodes.len(),
                route.timeout,
                match route.weapon {
                    RaceWeaponMode::No => "no",
                    RaceWeaponMode::Allowed => "allowed",
                    RaceWeaponMode::After2s => "after 2s",
                },
                match route.falsestart {
                    RaceFalseStartMode::No => "no",
                    RaceFalseStartMode::Yes => "yes",
                },
            )));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped ktx race1.route, verbatim shape: one 2-node route.
    const RACE1: &str = r#"race_route_add_start

    race_add_route_node    0   0   -40 0 0
    race_add_route_node 1376 382 -3700 0 0

    race_set_route_name "Pillars" "start\215finish"
    race_set_route_timeout 40
    race_set_route_weapon_mode raceWeaponNo
    race_set_route_falsestart_mode raceFalseStartNo

race_route_add_end
"#;

    #[test]
    fn parses_simple_two_node_file() {
        let f = parse_route_file(RACE1).unwrap();
        assert!(f.warnings.is_empty());
        assert_eq!(f.routes.len(), 1);
        let r = &f.routes[0];
        assert_eq!(r.name, "Pillars");
        assert_eq!(r.desc, "start-finish"); // \215 -> 16*2+8*1+5 = 45 = '-'
        assert_eq!(r.timeout, 40.0);
        assert_eq!(r.weapon, RaceWeaponMode::No);
        assert_eq!(r.falsestart, RaceFalseStartMode::No);
        assert_eq!(r.nodes.len(), 2);
        assert_eq!(r.nodes[0].kind, RaceNodeType::Start);
        assert_eq!(r.nodes[0].origin, Vec3::new(0.0, 0.0, -40.0));
        assert_eq!(r.nodes[1].kind, RaceNodeType::End);
        assert_eq!(r.nodes[1].origin, Vec3::new(1376.0, 382.0, -3700.0));
    }

    #[test]
    fn multi_route_file_and_node_demotion() {
        // Two routes; the first has 4 nodes so the middle two must demote to checkpoints.
        let text = "\
race_route_add_start
race_add_route_node 0 0 0 0 90
race_add_route_node 1 0 0 0 0
race_add_route_node 2 0 0 0 0
race_add_route_node 3 0 0 0 0
race_set_route_name \"A\" \"a\"
race_route_add_end
race_route_add_start
race_add_route_node 0 0 0 0 0
race_add_route_node 5 0 0 0 0
race_set_route_name \"B\" \"b\"
race_route_add_end
";
        let f = parse_route_file(text).unwrap();
        assert_eq!(f.routes.len(), 2);
        let kinds: Vec<_> = f.routes[0].nodes.iter().map(|n| n.kind).collect();
        assert_eq!(
            kinds,
            vec![
                RaceNodeType::Start,
                RaceNodeType::Checkpoint,
                RaceNodeType::Checkpoint,
                RaceNodeType::End
            ]
        );
        assert_eq!(f.routes[0].nodes[0].yaw, 90.0);
        // Defaults where the file sets nothing: 20s timeout, weapons allowed.
        assert_eq!(f.routes[1].timeout, 20.0);
        assert_eq!(f.routes[1].weapon, RaceWeaponMode::Allowed);
    }

    #[test]
    fn node_size_amends_last_node() {
        let text = "\
race_route_add_start
race_add_route_node 0 0 0 0 0
race_add_route_node 1 1 1 0 0
race_set_node_size 128 8 64
race_set_route_name \"A\" \"a\"
race_route_add_end
";
        let f = parse_route_file(text).unwrap();
        assert_eq!(f.routes[0].nodes[0].size, Vec3::ZERO);
        assert_eq!(f.routes[0].nodes[1].size, Vec3::new(128.0, 8.0, 64.0));
    }

    #[test]
    fn timeout_zero_is_ignored_and_valid_clamped() {
        let text = "\
race_route_add_start
race_set_route_timeout 0
race_route_add_end
race_route_add_start
race_set_route_timeout 5000
race_route_add_end
";
        let f = parse_route_file(text).unwrap();
        assert_eq!(f.routes[0].timeout, 20.0); // 0 leaves the default
        assert_eq!(f.routes[1].timeout, 999.0);
    }

    #[test]
    fn teleport_flags_and_comments() {
        let text = "\
// full-line comment
race_set_teleport_flags_by_name exit RACEFLAG_TOUCH_RACEEND
race_set_teleport_flags_by_name pit RACEFLAG_TOUCH_RACEFAIL
race_set_teleport_flags_by_name odd RACEFLAG_BOGUS
";
        let f = parse_route_file(text).unwrap();
        assert_eq!(
            f.teleport_flags,
            vec![
                ("exit".to_string(), RaceTeleportFlag::End),
                ("pit".to_string(), RaceTeleportFlag::Fail)
            ]
        );
    }

    #[test]
    fn error_cases() {
        let line_of = |text: &str| parse_route_file(text).unwrap_err().line;
        // Unknown instruction.
        assert_eq!(line_of("race_bogus_command\n"), 1);
        // Node outside a definition.
        assert_eq!(line_of("race_add_route_node 0 0 0 0 0\n"), 1);
        // Wrong arg count.
        assert_eq!(line_of("race_route_add_start\nrace_add_route_node 0 0 0\n"), 2);
        // Nested add_start.
        assert_eq!(line_of("race_route_add_start\nrace_route_add_start\n"), 2);
        // add_end outside a definition.
        assert_eq!(line_of("race_route_add_end\n"), 1);
        // node_size with nothing to amend.
        assert_eq!(line_of("race_route_add_start\nrace_set_node_size 1 2 3\n"), 2);
        // teleport flags inside a definition.
        assert_eq!(
            line_of("race_route_add_start\nrace_set_teleport_flags_by_name a RACEFLAG_TOUCH_RACEEND\n"),
            2
        );
        // Bad weapon mode value.
        assert_eq!(line_of("race_route_add_start\nrace_set_route_weapon_mode raceWeaponMaybe\n"), 2);
    }

    #[test]
    fn unterminated_route_is_dropped_with_warning() {
        let f = parse_route_file("race_route_add_start\nrace_add_route_node 0 0 0 0 0\n").unwrap();
        assert!(f.routes.is_empty());
        assert_eq!(f.warnings.len(), 1);
    }

    // --- embedded-entity chains ---

    fn marker(x: f32, target: Option<&str>, targetname: Option<&str>) -> RaceMarkerInfo {
        RaceMarkerInfo {
            origin: Vec3::new(x, 0.0, 0.0),
            size: Vec3::ZERO,
            target: target.map(Into::into),
            targetname: targetname.map(Into::into),
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    fn start(name: &str, target: Option<&str>) -> RaceStartInfo {
        RaceStartInfo {
            marker: marker(0.0, target, None),
            name: name.into(),
            desc: "desc".into(),
            timeout: 40.0,
            weapon_mode: 1,
            falsestart_mode: 1,
        }
    }

    #[test]
    fn race12_shaped_chain() {
        // One start chaining checkpoint1 -> ... -> checkpoint5 (the race12 layout): 6 nodes,
        // four mid checkpoints, the last marker the end.
        let starts = [start("Just in time", Some("checkpoint1"))];
        let markers: Vec<_> = (1..=5)
            .map(|i| {
                marker(
                    i as f32 * 100.0,
                    (i < 5).then(|| format!("checkpoint{}", i + 1)).as_deref(),
                    Some(&format!("checkpoint{i}")),
                )
            })
            .collect();
        let (routes, warnings) = routes_from_markers(&starts, &markers);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(routes.len(), 1);
        let r = &routes[0];
        assert_eq!(r.nodes.len(), 6);
        assert_eq!(r.nodes[0].kind, RaceNodeType::Start);
        assert!(r.nodes[1..5].iter().all(|n| n.kind == RaceNodeType::Checkpoint));
        assert_eq!(r.nodes[5].kind, RaceNodeType::End);
        assert_eq!(r.nodes[5].origin.x, 500.0);
        assert_eq!(r.weapon, RaceWeaponMode::No);
        assert_eq!(r.falsestart, RaceFalseStartMode::No);
        assert_eq!(r.timeout, 40.0);
    }

    #[test]
    fn race14_shaped_multiple_starts() {
        // Three starts from one origin, distinct chains (the race14 layout).
        let starts = [
            start("Main Route", Some("m1")),
            start("Advanced Bunnyhopper", Some("a1")),
            start("Freestyle", Some("f1")),
        ];
        let markers = [
            marker(1.0, None, Some("m1")),
            marker(2.0, None, Some("a1")),
            marker(3.0, None, Some("f1")),
        ];
        let (routes, warnings) = routes_from_markers(&starts, &markers);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(routes.len(), 3);
        assert!(routes.iter().all(|r| r.nodes.len() == 2));
    }

    #[test]
    fn circular_chain_rejected() {
        let starts = [start("Loop", Some("a"))];
        let markers = [marker(1.0, Some("b"), Some("a")), marker(2.0, Some("a"), Some("b"))];
        let (routes, warnings) = routes_from_markers(&starts, &markers);
        assert!(routes.is_empty());
        assert!(warnings.iter().any(|w| w.contains("circular")));
    }

    #[test]
    fn missing_name_or_bad_modes_rejected() {
        let mut unnamed = start("", Some("a"));
        unnamed.desc = String::new();
        let mut bad_weapon = start("W", Some("a"));
        bad_weapon.weapon_mode = 0;
        let mut bad_fs = start("F", Some("a"));
        bad_fs.falsestart_mode = 9;
        let markers = [marker(1.0, None, Some("a"))];
        let (routes, warnings) = routes_from_markers(&[unnamed, bad_weapon, bad_fs], &markers);
        assert!(routes.is_empty());
        assert_eq!(warnings.len(), 3);
    }

    #[test]
    fn short_chain_rejected() {
        // A start with no target is a 1-node chain: too short.
        let (routes, warnings) = routes_from_markers(&[start("Solo", None)], &[]);
        assert!(routes.is_empty());
        assert!(warnings.iter().any(|w| w.contains("too short")));
    }

    /// Every shipped ktx example route file must parse clean. Run with
    /// `RTX_TEST_ROUTES=…/ktx/resources/example-configs/ktx/race/routes`; skipped
    /// (vacuously green) when unset — the same opt-in idiom as `RTX_TEST_BSP`.
    #[test]
    fn parses_shipped_route_files() {
        let Ok(dir) = std::env::var("RTX_TEST_ROUTES") else {
            return;
        };
        let mut files = 0;
        for entry in std::fs::read_dir(dir).expect("read routes dir") {
            let path = entry.expect("dir entry").path();
            if path.extension().is_none_or(|e| e != "route") {
                continue;
            }
            files += 1;
            let text = std::fs::read_to_string(&path).expect("read route file");
            let f = parse_route_file(&text).unwrap_or_else(|e| panic!("{path:?}:{}: {}", e.line, e.msg));
            assert!(!f.routes.is_empty(), "{path:?}: no routes");
            for (ri, r) in f.routes.iter().enumerate() {
                assert!(!r.name.is_empty(), "{path:?} route {ri}: unnamed");
                assert!(r.nodes.len() >= 2, "{path:?} route {ri}: {} nodes", r.nodes.len());
                assert_eq!(r.nodes[0].kind, RaceNodeType::Start, "{path:?} route {ri}");
                assert_eq!(r.nodes.last().unwrap().kind, RaceNodeType::End, "{path:?} route {ri}");
            }
        }
        assert!(files > 0, "no .route files found");
    }

    #[test]
    fn dangling_target_ends_chain() {
        // Chain: start -> a -> (dangling "ghost"): route keeps its 2 nodes, with a warning.
        let starts = [start("Dangle", Some("a"))];
        let markers = [marker(1.0, Some("ghost"), Some("a"))];
        let (routes, warnings) = routes_from_markers(&starts, &markers);
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].nodes.len(), 2);
        assert!(warnings.iter().any(|w| w.contains("ghost")));
    }

    // --- offline routability harness -------------------------------------------------------------
    //
    // The closest thing to running the server: build the real navmesh from the real BSPs and run
    // the race mode's per-leg routability question over every route, from both sources. Same
    // opt-in idiom as `RTX_TEST_BSP`; run with
    //   RTX_TEST_MAPS=…/qw/maps RTX_TEST_ROUTES=…/ktx/race/routes \
    //     cargo test --release offline_routability -- --nocapture
    // Approximation vs the live server: plats and button-gated doors aren't wired offline (their
    // inputs come from spawned entities), so a leg that genuinely rides a lift reports FAIL here.
    // Teleporters *are* wired (trigger box from the submodel bounds, destination +27z as spawned).

    /// A lump's byte range from a BSP header (works for BSP29 and BSP2 — same directory layout).
    fn bsp_lump(bytes: &[u8], idx: usize) -> Option<&[u8]> {
        let at = 4 + idx * 8;
        let ofs = i32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?) as usize;
        let len = i32::from_le_bytes(bytes.get(at + 4..at + 8)?.try_into().ok()?) as usize;
        bytes.get(ofs..ofs + len)
    }

    /// Parse the entity lump's `{ "key" "value" … }` blocks.
    fn parse_ent_blocks(text: &str) -> Vec<Vec<(String, String)>> {
        let mut blocks = Vec::new();
        let mut cur: Option<Vec<(String, String)>> = None;
        let mut chars = text.chars().peekable();
        let mut strings: Vec<String> = Vec::new();
        while let Some(c) = chars.next() {
            match c {
                '{' => cur = Some(Vec::new()),
                '}' => {
                    if let Some(b) = cur.take() {
                        blocks.push(b);
                    }
                    strings.clear();
                }
                '"' => {
                    let mut s = String::new();
                    for c in chars.by_ref() {
                        if c == '"' {
                            break;
                        }
                        s.push(c);
                    }
                    strings.push(s);
                    if strings.len() == 2 {
                        if let Some(b) = cur.as_mut() {
                            b.push((strings[0].clone(), strings[1].clone()));
                        }
                        strings.clear();
                    }
                }
                _ => {}
            }
        }
        blocks
    }

    fn field<'a>(b: &'a [(String, String)], key: &str) -> Option<&'a str> {
        b.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    fn vec3_of(s: &str) -> Vec3 {
        let mut p = s.split_whitespace().map(|t| t.parse().unwrap_or(0.0));
        Vec3::new(p.next().unwrap_or(0.0), p.next().unwrap_or(0.0), p.next().unwrap_or(0.0))
    }

    /// Submodel `n`'s world-space bounds from the models lump (64-byte records, bounds first).
    fn submodel_bounds(models: &[u8], n: usize) -> Option<(Vec3, Vec3)> {
        let rec = models.get(n * 64..n * 64 + 24)?;
        let f = |i: usize| f32::from_le_bytes(rec[i * 4..i * 4 + 4].try_into().unwrap());
        Some((Vec3::new(f(0), f(1), f(2)), Vec3::new(f(3), f(4), f(5))))
    }

    /// The map's routes from both sources, as the live loader would assemble them.
    fn routes_of(blocks: &[Vec<(String, String)>], route_file: Option<&str>) -> Vec<RaceRoute> {
        let mut routes = Vec::new();
        if let Some(text) = route_file {
            if let Ok(f) = parse_route_file(text) {
                routes.extend(f.routes);
            }
        }
        let info = |b: &Vec<(String, String)>| RaceMarkerInfo {
            origin: field(b, "origin").map(vec3_of).unwrap_or_default(),
            size: field(b, "size").map(vec3_of).unwrap_or_default(),
            target: field(b, "target").map(Into::into),
            targetname: field(b, "targetname").map(Into::into),
            pitch: field(b, "race_route_start_pitch").and_then(|v| v.parse().ok()).unwrap_or(0.0),
            yaw: field(b, "race_route_start_yaw").and_then(|v| v.parse().ok()).unwrap_or(0.0),
        };
        let starts: Vec<RaceStartInfo> = blocks
            .iter()
            .filter(|b| field(b, "classname") == Some("race_route_start"))
            .map(|b| RaceStartInfo {
                marker: info(b),
                name: field(b, "race_route_name").unwrap_or("").to_owned(),
                desc: field(b, "race_route_description").unwrap_or("").to_owned(),
                timeout: field(b, "race_route_timeout").and_then(|v| v.parse().ok()).unwrap_or(0.0),
                weapon_mode: field(b, "race_route_weapon_mode").and_then(|v| v.parse().ok()).unwrap_or(0),
                falsestart_mode: field(b, "race_route_falsestart_mode").and_then(|v| v.parse().ok()).unwrap_or(0),
            })
            .collect();
        let markers: Vec<RaceMarkerInfo> = blocks
            .iter()
            .filter(|b| field(b, "classname") == Some("race_route_marker"))
            .map(info)
            .collect();
        let (ent_routes, _) = routes_from_markers(&starts, &markers);
        routes.extend(ent_routes);
        routes
    }

    /// Teleporters as `collect_teleports` would gather them: trigger box from the brush
    /// submodel's bounds, destination = the targeted entity's origin (+27z, as its spawn does).
    fn teleports_of(blocks: &[Vec<(String, String)>], models: &[u8]) -> Vec<crate::navmesh::TeleportInfo> {
        blocks
            .iter()
            .filter(|b| field(b, "classname") == Some("trigger_teleport"))
            .filter_map(|b| {
                let n: usize = field(b, "model")?.strip_prefix('*')?.parse().ok()?;
                let (tmin, tmax) = submodel_bounds(models, n)?;
                let target = field(b, "target")?;
                let dest = blocks.iter().find(|d| field(d, "targetname") == Some(target))?;
                let mut origin = field(dest, "origin").map(vec3_of)?;
                if field(dest, "classname") == Some("info_teleport_destination") {
                    origin.z += 27.0;
                }
                Some(crate::navmesh::TeleportInfo { tmin, tmax, dest: origin })
            })
            .collect()
    }

    #[test]
    fn offline_routability_report() {
        let Ok(maps_dir) = std::env::var("RTX_TEST_MAPS") else {
            return;
        };
        let routes_dir = std::env::var("RTX_TEST_ROUTES").ok();
        let sj = crate::navmesh::SpeedJumpParams { gravity: 800.0, accel: 10.0, maxspeed: 320.0 };
        let mut names: Vec<String> = std::fs::read_dir(&maps_dir)
            .expect("read maps dir")
            .filter_map(|e| {
                let p = e.ok()?.path();
                let stem = p.file_stem()?.to_str()?.to_owned();
                (p.extension().is_some_and(|x| x == "bsp") && (stem.starts_with("race") || stem.starts_with("ztrick")))
                    .then_some(stem)
            })
            .collect();
        names.sort();
        assert!(!names.is_empty(), "no race*/ztrick* maps in {maps_dir}");

        let (mut maps_with_routes, mut legs_pass, mut legs_fail) = (0, 0, 0);
        for name in names {
            let bytes = std::fs::read(format!("{maps_dir}/{name}.bsp")).expect("read bsp");
            let Some(ents) = bsp_lump(&bytes, 0) else {
                eprintln!("{name}: unreadable entity lump");
                continue;
            };
            let blocks = parse_ent_blocks(&String::from_utf8_lossy(ents));
            let route_file = routes_dir
                .as_ref()
                .and_then(|d| std::fs::read_to_string(format!("{d}/{name}.route")).ok());
            let routes = routes_of(&blocks, route_file.as_deref());
            if routes.is_empty() {
                eprintln!("{name}: no routes (unsupported)");
                continue;
            }
            maps_with_routes += 1;
            let models = bsp_lump(&bytes, 14).unwrap_or(&[]);
            let teleports = teleports_of(&blocks, models);
            let Some((_bsp, graph)) = crate::navmesh::build_navmesh(
                bytes.clone(),
                Vec::new(),
                teleports,
                Vec::new(),
                None,
                false, // race-legal: no double-jump links
                Some(sj),
                None,
            ) else {
                eprintln!("{name}: BSP failed to build a navmesh");
                continue;
            };
            for (ri, route) in routes.iter().enumerate() {
                let costs = crate::navmesh::LinkCosts::default();
                // Two verdicts side by side: the plain cell A* vs the speed-band planner (which
                // carries speed between legs — the exit band of one leg feeds the next). Banded
                // should route a superset of the legs the unbanded search can (never fewer).
                let mut unbanded = format!("PASS ({} legs)", route.nodes.len() - 1);
                let mut banded = unbanded.clone();
                let mut carry = crate::navmesh::BAND_FLOOR[0];
                for (i, pair) in route.nodes.windows(2).enumerate() {
                    let snap = |n: &RaceRouteNode| {
                        graph
                            .nearest(n.origin)
                            .filter(|&c| (graph.cell_origin(c) - n.origin).length() <= 80.0)
                    };
                    let (Some(a), Some(b)) = (snap(&pair[0]), snap(&pair[1])) else {
                        let msg = format!("FAIL at leg {} (node off the navmesh)", i + 1);
                        (unbanded, banded) = (msg.clone(), msg);
                        break;
                    };
                    if unbanded.starts_with("PASS") && graph.find_path(a, b, &costs).is_none() {
                        // Diagnose the break: how close does reachability get to the goal, and
                        // what does the remaining hop look like (distance and height delta)?
                        let frontier = graph
                            .nearest_reachable_to(a, b, &costs)
                            .map(|c| {
                                let (fo, go) = (graph.cell_origin(c), graph.cell_origin(b));
                                let dxy = glam::Vec2::new(go.x - fo.x, go.y - fo.y).length();
                                format!(
                                    "reached to {:.0}u short of the goal (dxy {dxy:.0}, dz {:+.0})",
                                    (go - fo).length(),
                                    go.z - fo.z
                                )
                            })
                            .unwrap_or_else(|| "nothing reachable from the start at all".into());
                        unbanded = format!("FAIL at leg {} (no path; {frontier})", i + 1);
                    }
                    match graph.find_path_banded(a, b, carry, &costs) {
                        Some(r) => carry = crate::navmesh::BAND_FLOOR[r.end_band as usize],
                        None if banded.starts_with("PASS") => banded = format!("FAIL at leg {} (no banded path)", i + 1),
                        None => {}
                    }
                }
                let (up, bp) = (unbanded.starts_with("PASS"), banded.starts_with("PASS"));
                if bp {
                    legs_pass += 1;
                } else {
                    legs_fail += 1;
                }
                assert!(!up || bp, "{name} route {ri}: unbanded PASS but banded FAIL — banding lost a route");
                eprintln!("{name}: route {ri} \"{}\": unbanded {unbanded} | banded {banded}", route.name);
            }
        }
        eprintln!("--- {maps_with_routes} maps with routes: {legs_pass} routes PASS (banded), {legs_fail} FAIL ---");
        assert!(maps_with_routes > 0, "no routes found on any map — wrong RTX_TEST_MAPS/RTX_TEST_ROUTES?");
    }
}
