// SPDX-License-Identifier: AGPL-3.0-or-later

//! External bot-control channel — the rocket-jump tuning harness.
//!
//! A cvar-gated (`rtx_control_port`) localhost TCP server an external driver connects to in order to
//! *puppet* a bot: teleport it to a rocket-jump link's launch cell, order it to fly that specific
//! link, and read back per-attempt telemetry (stance offset, aim error, fire-timing error, landing
//! miss). The point is a scripted tuning loop — sweep every RJ link a map generates, see which land,
//! turn the `rtx_rj_*` knobs, re-run — without hand-flying bots in a live server.
//!
//! ## Threading
//! The engine drives this module single-threaded from the frame calls, so every `GameState` mutation
//! stays on that thread. The socket work is pushed to background threads that only shuttle raw
//! `String` lines through `mpsc` channels — the exact shape as the navmesh build worker
//! ([`crate::nav_build`]): a listener thread accepts connections, a per-connection reader thread feeds
//! inbound lines to [`ControlState::lines_rx`], and a writer thread drains outbound JSON to the
//! current client. Commands are parsed and executed, and events emitted, entirely inside
//! [`frame_begin`]/[`frame_end`] under the frame's `&mut GameState`. No lock is ever held over game
//! state; the only shared state between threads is the raw socket and the channels.
//!
//! ## Protocol
//! Inbound: newline-delimited text, `<id> <verb> [args…]`, `id` a caller-chosen integer echoed back.
//! Outbound: newline-delimited hand-emitted JSON (the game crate stays dependency-free). A reply is
//! `{"id":N,"ok":true,"data":{…}}` / `{"id":N,"ok":false,"error":"…"}`; an unsolicited lifecycle
//! event is `{"ev":"arrived"|"goto_stall"|"rj_result",…}`. A single outbound channel gives total
//! ordering; the client demuxes on the presence of `id` vs `ev`.

use std::io::{BufRead, BufReader, Write};
use std::net::{Ipv4Addr, TcpListener, TcpStream};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};

use glam::{Vec3, Vec3Swizzles};

use crate::bot::state::{ControlOrder, HookState, RjOutcome, RjState, RjTelemetry};
use crate::defs::{Bits, Flags, Items, Weapon};
use crate::entity::EntId;
use crate::game::{cstring, GameState, MAX_EDICTS};
use crate::math::wrap180;
use crate::navmesh::LinkKind;

/// A goto is "arrived" once within this XY radius of the target (matches the bot's own arrival gate)
/// and within [`GOTO_ARRIVE_Z`] in Z. A radius policy, not a cell match — cell borders flap.
const GOTO_ARRIVE_XY: f32 = 24.0;
const GOTO_ARRIVE_Z: f32 = 48.0;
/// Goto stall: if the straight-line XY distance to the target hasn't improved by [`STALL_EPS`] for
/// [`STALL_SECS`], the source is (currently) inaccessible. The window sits above the bot's own 2.5 s
/// progress watchdog, so it gets one penalize-and-divert attempt first — a stall then means
/// "unreachable even after diverting", the signal a rocket-jump *source* cell can't be stood on.
const STALL_EPS: f32 = 16.0;
const STALL_SECS: f32 = 4.0;

/// The control channel's live state, carried on [`GameState`]. Persists across map loads (the socket
/// binds once); `started` guards against re-binding. All fields stay untouched — the whole harness is
/// inert — until `rtx_control_port` is set to a real port and the first frame binds the listener.
pub(crate) struct ControlState {
    /// Whether the listener has been (attempted to be) bound. Set once, so a bind is tried at most once.
    started: bool,
    /// Inbound command lines from the current connection (drained each frame in [`frame_begin`]).
    lines_rx: Option<Receiver<String>>,
    /// Outbound JSON lines (replies + events). The writer thread owns the receiving half.
    events_tx: Option<Sender<String>>,
}

impl Default for ControlState {
    fn default() -> Self {
        ControlState { started: false, lines_rx: None, events_tx: None }
    }
}

/// Frame prologue: lazily bind the listener once the port cvar is set, then drain and execute every
/// inbound command on the engine thread. Runs before `run_bots` so a `goto`/`rj`/`teleport` issued
/// this frame takes effect this frame.
pub(crate) fn frame_begin(game: &mut GameState) {
    if !game.control.started {
        let p = game.host.cvar(c"rtx_control_port") as i64;
        if (1..=65535).contains(&p) {
            start_listener(game, p as u16);
        }
    }
    let lines: Vec<String> = match game.control.lines_rx.as_ref() {
        Some(rx) => rx.try_iter().collect(),
        None => return,
    };
    for line in lines {
        exec_line(game, &line);
    }
}

/// Frame epilogue: observe every puppeted bot and emit the lifecycle events its order produced this
/// frame (arrival / stall for a goto, the terminal telemetry for a rocket jump). Runs after
/// `run_bots` so it sees the post-frame bot state the driver just wrote.
pub(crate) fn frame_end(game: &mut GameState) {
    if game.control.events_tx.is_none() {
        return; // channel never came up — nothing to emit
    }
    let now = game.time();
    let maxclients = game.host.cvar(c"maxclients").max(0.0) as u32;
    for i in 1..=maxclients {
        let e = EntId(i);
        if !game.entities[e].bot.is_bot || !game.entities[e].in_use {
            continue;
        }
        match game.entities[e].bot.puppet.order {
            None | Some(ControlOrder::Hold) => {}
            Some(ControlOrder::Goto { target }) => poll_goto(game, e, i, target, now),
            Some(ControlOrder::RocketJump { link }) => {
                // Trace the flight: sample this frame's post-move origin/velocity before checking for a
                // result, so the trajectory in `rj_result` runs from the stance through the landing.
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let traj = &mut game.entities[e].bot.puppet.traj;
                if traj.len() < 400 {
                    traj.push((now, origin, vel));
                }
                poll_rj(game, e, i, link, now);
            }
        }
    }
}

/// Bind the localhost listener and spawn the socket threads (see the module docs). Called once; on
/// bind failure it logs and leaves the channel down (`events_tx` stays `None`, so `frame_end` no-ops).
fn start_listener(game: &mut GameState, port: u16) {
    game.control.started = true; // one attempt only, success or not
    let listener = match TcpListener::bind((Ipv4Addr::LOCALHOST, port)) {
        Ok(l) => l,
        Err(err) => {
            game.host
                .conprint(&cstring(&format!("rtx: control: bind 127.0.0.1:{port} failed: {err}\n")));
            return;
        }
    };
    let (lines_tx, lines_rx) = std::sync::mpsc::channel::<String>();
    let (events_tx, events_rx) = std::sync::mpsc::channel::<String>();
    // The single write-half slot the writer thread drains and the listener thread replaces on each new
    // connection. `Option` so a dropped client leaves it empty and outbound lines are simply discarded.
    let slot: Arc<Mutex<Option<TcpStream>>> = Arc::new(Mutex::new(None));
    let wslot = slot.clone();
    std::thread::spawn(move || writer_loop(events_rx, wslot));
    std::thread::spawn(move || listener_loop(listener, lines_tx, slot));
    game.control.lines_rx = Some(lines_rx);
    game.control.events_tx = Some(events_tx);
    game.host
        .conprint(&cstring(&format!("rtx: control: listening on 127.0.0.1:{port}\n")));
}

/// Accept loop: each new connection replaces the write-half slot (one client at a time — a fresh
/// connection supersedes a stale one) and gets its own reader thread feeding inbound lines.
fn listener_loop(listener: TcpListener, lines_tx: Sender<String>, slot: Arc<Mutex<Option<TcpStream>>>) {
    for stream in listener.incoming().flatten() {
        let _ = stream.set_nodelay(true);
        if let Ok(wr) = stream.try_clone() {
            if let Ok(mut g) = slot.lock() {
                *g = Some(wr);
            }
        }
        let tx = lines_tx.clone();
        std::thread::spawn(move || {
            for line in BufReader::new(stream).lines() {
                match line {
                    Ok(l) => {
                        if tx.send(l).is_err() {
                            break; // game side gone
                        }
                    }
                    Err(_) => break, // connection dropped
                }
            }
        });
    }
}

/// Writer loop: drain outbound JSON lines to the current client. With no client connected the line is
/// dropped (the backpressure policy — events are low-rate and a reconnecting client resyncs via
/// `status`); a write error clears the slot until a new connection lands.
fn writer_loop(events_rx: Receiver<String>, slot: Arc<Mutex<Option<TcpStream>>>) {
    while let Ok(line) = events_rx.recv() {
        let Ok(mut g) = slot.lock() else { continue };
        if let Some(stream) = g.as_mut() {
            if stream.write_all(line.as_bytes()).is_err() || stream.write_all(b"\n").is_err() {
                *g = None;
            } else {
                let _ = stream.flush();
            }
        }
    }
}

/// Queue one outbound JSON line (reply or event). A no-op when the channel is down.
fn send(game: &GameState, line: String) {
    if let Some(tx) = game.control.events_tx.as_ref() {
        let _ = tx.send(line);
    }
}

// --- inbound command grammar (pure parse, unit-tested) ---

/// One parsed control command (see the module protocol docs). The wire `id` is threaded separately.
#[derive(Debug, PartialEq)]
enum ControlCmd {
    Status,
    Links,
    Prep { bot: u32, health: f32, rockets: f32 },
    Teleport { bot: u32, pos: Vec3 },
    Goto { bot: u32, pos: Vec3 },
    Rj { bot: u32, link: u32 },
    Hold { bot: u32 },
    Stop { bot: u32 },
    Set { name: String, value: String },
    Get { name: String },
    Cmd { raw: String },
}

/// Split the first whitespace-delimited token off `s`, returning `(token, rest)` with `rest` trimmed
/// of leading whitespace. `("", "")` for an all-whitespace input.
fn split_first(s: &str) -> (&str, &str) {
    let s = s.trim_start();
    match s.find(char::is_whitespace) {
        Some(i) => (&s[..i], s[i..].trim_start()),
        None => (s, ""),
    }
}

fn parse_u32(tok: Option<&str>, what: &str) -> Result<u32, String> {
    tok.ok_or_else(|| format!("missing {what}"))?
        .parse::<u32>()
        .map_err(|_| format!("bad {what}"))
}

fn parse_f32(tok: Option<&str>, what: &str) -> Result<f32, String> {
    tok.ok_or_else(|| format!("missing {what}"))?
        .parse::<f32>()
        .map_err(|_| format!("bad {what}"))
}

fn parse_bot_vec3(rest: &str) -> Result<(u32, Vec3), String> {
    let mut t = rest.split_whitespace();
    let bot = parse_u32(t.next(), "bot")?;
    let x = parse_f32(t.next(), "x")?;
    let y = parse_f32(t.next(), "y")?;
    let z = parse_f32(t.next(), "z")?;
    Ok((bot, Vec3::new(x, y, z)))
}

/// Parse one inbound line into `(id, command)`. Pure — no game state — so it unit-tests standalone.
fn parse_line(line: &str) -> Result<(i64, ControlCmd), String> {
    let (id_tok, r1) = split_first(line.trim());
    if id_tok.is_empty() {
        return Err("empty line".into());
    }
    let id: i64 = id_tok.parse().map_err(|_| format!("bad id '{id_tok}'"))?;
    let (verb, rest) = split_first(r1);
    let cmd = match verb {
        "status" => ControlCmd::Status,
        "links" => ControlCmd::Links,
        "prep" => {
            let mut t = rest.split_whitespace();
            let bot = parse_u32(t.next(), "bot")?;
            let health = t.next().map(|s| s.parse::<f32>()).transpose().map_err(|_| "bad health")?.unwrap_or(100.0);
            let rockets = t.next().map(|s| s.parse::<f32>()).transpose().map_err(|_| "bad rockets")?.unwrap_or(10.0);
            ControlCmd::Prep { bot, health, rockets }
        }
        "teleport" => {
            let (bot, pos) = parse_bot_vec3(rest)?;
            ControlCmd::Teleport { bot, pos }
        }
        "goto" => {
            let (bot, pos) = parse_bot_vec3(rest)?;
            ControlCmd::Goto { bot, pos }
        }
        "rj" => {
            let mut t = rest.split_whitespace();
            let bot = parse_u32(t.next(), "bot")?;
            let link = parse_u32(t.next(), "link")?;
            ControlCmd::Rj { bot, link }
        }
        "hold" => ControlCmd::Hold { bot: parse_u32(rest.split_whitespace().next(), "bot")? },
        "stop" => ControlCmd::Stop { bot: parse_u32(rest.split_whitespace().next(), "bot")? },
        "set" => {
            let (name, value) = split_first(rest);
            if name.is_empty() {
                return Err("set: missing cvar".into());
            }
            ControlCmd::Set { name: name.to_string(), value: value.to_string() }
        }
        "get" => {
            let (name, _) = split_first(rest);
            if name.is_empty() {
                return Err("get: missing cvar".into());
            }
            ControlCmd::Get { name: name.to_string() }
        }
        "cmd" => {
            if rest.is_empty() {
                return Err("cmd: missing command".into());
            }
            ControlCmd::Cmd { raw: rest.to_string() }
        }
        other => return Err(format!("unknown verb '{other}'")),
    };
    Ok((id, cmd))
}

/// Whether a cvar name is safe to splice into a `set` localcmd (guards the console tokenizer): a
/// non-empty run of `[A-Za-z0-9_]`. rtx cvars are all of this form.
fn valid_cvar_name(name: &str) -> bool {
    !name.is_empty() && name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
}

// --- command execution (engine thread, &mut GameState) ---

fn exec_line(game: &mut GameState, line: &str) {
    match parse_line(line) {
        Ok((id, cmd)) => exec_cmd(game, id, cmd),
        Err(e) => reply_err(game, 0, &e),
    }
}

fn exec_cmd(game: &mut GameState, id: i64, cmd: ControlCmd) {
    let result: Result<String, String> = match cmd {
        ControlCmd::Status => Ok(status_json(game)),
        ControlCmd::Links => links_json(game),
        ControlCmd::Prep { bot, health, rockets } => do_prep(game, bot, health, rockets),
        ControlCmd::Teleport { bot, pos } => do_teleport(game, bot, pos),
        ControlCmd::Goto { bot, pos } => do_goto(game, bot, pos),
        ControlCmd::Rj { bot, link } => do_rj(game, bot, link),
        ControlCmd::Hold { bot } => do_order(game, bot, ControlOrder::Hold),
        ControlCmd::Stop { bot } => do_stop(game, bot),
        ControlCmd::Set { name, value } => do_set(game, &name, &value),
        ControlCmd::Get { name } => do_get(game, &name),
        ControlCmd::Cmd { raw } => {
            game.host.localcmd(&raw);
            Ok("{\"queued\":true}".to_string())
        }
    };
    match result {
        Ok(data) => reply_ok(game, id, &data),
        Err(e) => reply_err(game, id, &e),
    }
}

fn reply_ok(game: &GameState, id: i64, data: &str) {
    send(game, format!("{{\"id\":{id},\"ok\":true,\"data\":{data}}}"));
}

fn reply_err(game: &GameState, id: i64, msg: &str) {
    send(game, format!("{{\"id\":{id},\"ok\":false,\"error\":{}}}", jstr(msg)));
}

/// Validate that `bot` names a live rtx bot's client slot.
fn valid_bot(game: &GameState, bot: u32) -> Result<EntId, String> {
    if bot == 0 || bot as usize >= MAX_EDICTS {
        return Err(format!("bad bot {bot}"));
    }
    let ent = &game.entities[EntId(bot)];
    if !ent.bot.is_bot || !ent.in_use {
        return Err(format!("no such bot {bot}"));
    }
    Ok(EntId(bot))
}

/// Make a bot fit to rocket-jump: full-ish health, the RL selected with rockets, no quad, off cooldown.
/// Writing the entvars directly is the established way to set a loadout (mirrors the mode spawn kits).
fn do_prep(game: &mut GameState, bot: u32, health: f32, rockets: f32) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    if !game.entities[e].is_alive() {
        return Err(format!("bot {bot} not alive"));
    }
    {
        let v = &mut game.entities[e].v;
        v.health = health;
        v.items = v.items.with(Items::ROCKET_LAUNCHER);
        v.ammo_rockets = rockets;
        v.weapon = Weapon::RocketLauncher;
    }
    game.entities[e].combat.super_damage_finished = 0.0; // clear quad — a self-rocket under quad is lethal
    game.entities[e].combat.attack_finished = 0.0; // off cooldown, so the fire isn't swallowed
    game.w_set_current_ammo(e); // sync currentammo/ammo-type bits to the RL
    Ok(format!(
        "{{\"bot\":{bot},\"health\":{},\"rockets\":{}}}",
        jnum(health),
        jnum(rockets)
    ))
}

/// Place a bot at `pos` (feet on the ground it names), zero its momentum, and reset all navigation
/// commitments so nothing stale (a mid-flight route/jump) survives the jump. `+1z` avoids startsolid.
fn do_teleport(game: &mut GameState, bot: u32, pos: Vec3) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let at = pos + Vec3::new(0.0, 0.0, 1.0);
    game.entities[e].v.velocity = Vec3::ZERO;
    game.host.set_origin(e, at);
    reset_nav_state(&mut game.entities[e].bot, at, now);
    // Park the bot after placing it — otherwise, with no order, it would roam autonomously and arrive
    // at a subsequent rocket jump with residual velocity, contaminating the standstill measurement.
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
    Ok(format!("{{\"bot\":{bot},\"origin\":{}}}", jvec3(game.entities[e].v.origin)))
}

/// Clear every route/traversal commitment and seed the watchdogs at `at` (so the 200u teleport
/// detector doesn't trip on the jump). Shared by teleport and the goto/rj order setup.
fn reset_nav_state(bot: &mut crate::bot::state::BotState, at: Vec3, now: f32) {
    bot.route.clear();
    bot.route_bands.clear();
    bot.route_pos = 0;
    bot.rj = RjState::default();
    bot.hook = HookState::default();
    bot.sj = None;
    bot.air = None;
    bot.bhop = Default::default();
    bot.watchdog.last_origin = at;
    bot.watchdog.stuck_origin = at;
    bot.watchdog.stuck_since = now;
    bot.repath_time = now;
}

fn do_goto(game: &mut GameState, bot: u32, pos: Vec3) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let b = &mut game.entities[e].bot;
    b.rj = RjState::default();
    b.route.clear();
    b.repath_time = now;
    b.puppet.order = Some(ControlOrder::Goto { target: pos });
    b.puppet.best_dist = f32::INFINITY;
    b.puppet.best_since = now;
    Ok(format!("{{\"bot\":{bot},\"target\":{}}}", jvec3(pos)))
}

fn do_rj(game: &mut GameState, bot: u32, link: u32) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    {
        let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
        if link as usize >= g.links.len() {
            return Err(format!("link {link} out of range (0..{})", g.links.len()));
        }
        if g.link_kind(link) != LinkKind::RocketJump {
            return Err(format!("link {link} is not a rocket jump"));
        }
        if g.rocket_jump_of_link(link).is_none() {
            return Err(format!("link {link} has no solved rocket jump"));
        }
    }
    let b = &mut game.entities[e].bot;
    b.rj = RjState::default(); // fresh attempt (clears telemetry)
    b.rj.telem.link = link;
    b.route.clear();
    b.repath_time = now;
    b.puppet.traj.clear(); // fresh flight trace
    b.puppet.order = Some(ControlOrder::RocketJump { link });
    Ok(format!("{{\"bot\":{bot},\"link\":{link}}}"))
}

fn do_order(game: &mut GameState, bot: u32, order: ControlOrder) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    game.entities[e].bot.puppet.order = Some(order);
    Ok(format!("{{\"bot\":{bot}}}"))
}

fn do_stop(game: &mut GameState, bot: u32) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let b = &mut game.entities[e].bot;
    b.puppet.order = None;
    b.rj = RjState::default();
    b.route.clear();
    b.repath_time = now;
    Ok(format!("{{\"bot\":{bot}}}"))
}

fn do_set(game: &mut GameState, name: &str, value: &str) -> Result<String, String> {
    if !valid_cvar_name(name) {
        return Err(format!("bad cvar name '{name}'"));
    }
    let cname = cstring(name);
    // A cvar that already exists (all rtx_* knobs do, seeded at init) takes the value immediately via
    // the set builtin; an unknown one must be created through the `set` console command (mvdsv's
    // Cvar_Set refuses to create), which takes effect on the next Cbuf flush.
    if game.host.cvar_is_set(name) {
        game.host.cvar_set(&cname, &cstring(value));
    } else {
        game.host.localcmd(&format!("set {name} \"{value}\""));
    }
    Ok(format!("{{\"name\":{},\"value\":{}}}", jstr(name), jstr(value)))
}

fn do_get(game: &mut GameState, name: &str) -> Result<String, String> {
    if !valid_cvar_name(name) {
        return Err(format!("bad cvar name '{name}'"));
    }
    let cname = cstring(name);
    let mut buf = [0u8; 128];
    let s = game.host.cvar_string(&cname, &mut buf).to_string();
    let f = game.host.cvar(&cname);
    Ok(format!(
        "{{\"name\":{},\"string\":{},\"value\":{}}}",
        jstr(name),
        jstr(&s),
        jnum(f)
    ))
}

// --- status / links snapshots ---

fn status_json(game: &GameState) -> String {
    let (navmesh, cells, links, rj_links) = match game.nav.graph.as_ref() {
        Some(g) => ("ready", g.cells.len(), g.links.len(), g.summary().rocket_jump),
        None if game.nav.pending.is_some() => ("building", 0, 0, 0),
        None => ("none", 0, 0, 0),
    };
    let maxclients = game.host.cvar(c"maxclients").max(0.0) as u32;
    let mut bots = String::new();
    for i in 1..=maxclients {
        let ent = &game.entities[EntId(i)];
        if !ent.bot.is_bot || !ent.in_use {
            continue;
        }
        if !bots.is_empty() {
            bots.push(',');
        }
        bots.push_str(&format!(
            "{{\"ent\":{i},\"client\":{},\"origin\":{},\"health\":{},\"on_ground\":{},\"alive\":{},\"order\":{},\"rj_phase\":{}}}",
            ent.bot.client,
            jvec3(ent.v.origin),
            jnum(ent.v.health),
            ent.v.flags.has(Flags::ONGROUND),
            ent.is_alive(),
            jstr(order_name(ent.bot.puppet.order)),
            jstr(&format!("{:?}", ent.bot.rj.phase)),
        ));
    }
    format!(
        "{{\"map\":{},\"time\":{},\"navmesh\":{},\"cells\":{cells},\"links\":{links},\"rj_links\":{rj_links},\"bots\":[{bots}]}}",
        jstr(&game.level.mapname),
        jnum(game.time()),
        jstr(navmesh),
    )
}

fn links_json(game: &GameState) -> Result<String, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let mut items = String::new();
    for li in 0..g.links.len() as u32 {
        if g.link_kind(li) != LinkKind::RocketJump {
            continue;
        }
        let Some(tr) = g.rocket_jump_of_link(li) else { continue };
        let src = g.cell_origin(g.link_source(li));
        let tgt = g.cell_origin(g.link_target(li));
        if !items.is_empty() {
            items.push(',');
        }
        items.push_str(&format!(
            "{{\"link\":{li},\"src\":{},\"tgt\":{},\"fire_pitch\":{},\"fire_yaw\":{},\"fire_delay\":{},\"airtime\":{},\"self_damage\":{},\"v0\":{},\"blast\":{}}}",
            jvec3(src),
            jvec3(tgt),
            jnum(tr.fire_angles.x),
            jnum(tr.fire_angles.y),
            jnum(tr.fire_delay),
            jnum(tr.airtime),
            jnum(tr.self_damage),
            jvec3(tr.v0),
            jvec3(tr.blast),
        ));
    }
    Ok(format!("{{\"links\":[{items}]}}"))
}

fn order_name(o: Option<ControlOrder>) -> &'static str {
    match o {
        None => "none",
        Some(ControlOrder::Hold) => "hold",
        Some(ControlOrder::Goto { .. }) => "goto",
        Some(ControlOrder::RocketJump { .. }) => "rj",
    }
}

// --- per-frame puppet pollers (emit lifecycle events) ---

fn poll_goto(game: &mut GameState, e: EntId, bot: u32, target: Vec3, now: f32) {
    let origin = game.entities[e].v.origin;
    let dxy = (origin.xy() - target.xy()).length();
    let dz = (origin.z - target.z).abs();
    if dxy <= GOTO_ARRIVE_XY && dz <= GOTO_ARRIVE_Z {
        game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
        send(game, format!(
            "{{\"ev\":\"arrived\",\"bot\":{bot},\"t\":{},\"origin\":{},\"target\":{},\"dist\":{}}}",
            jnum(now), jvec3(origin), jvec3(target), jnum(dxy),
        ));
        return;
    }
    let (best_dist, best_since) = {
        let p = &game.entities[e].bot.puppet;
        (p.best_dist, p.best_since)
    };
    if dxy < best_dist - STALL_EPS {
        let p = &mut game.entities[e].bot.puppet;
        p.best_dist = dxy;
        p.best_since = now;
    } else if now - best_since > STALL_SECS {
        game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
        send(game, format!(
            "{{\"ev\":\"goto_stall\",\"bot\":{bot},\"t\":{},\"origin\":{},\"target\":{},\"dist\":{},\"best\":{},\"secs\":{}}}",
            jnum(now), jvec3(origin), jvec3(target), jnum(dxy), jnum(best_dist), jnum(STALL_SECS),
        ));
    }
}

fn poll_rj(game: &mut GameState, e: EntId, bot: u32, link: u32, now: f32) {
    let Some(outcome) = game.entities[e].bot.rj.telem.outcome.take() else {
        return; // attempt still in flight
    };
    let telem = game.entities[e].bot.rj.telem.clone();
    // The solver's predicted post-blast velocity and blast geometry for this link, to compare against
    // the actual flight trace: what the offline model *expected* vs what the engine produced.
    let (v0, blast, pos_blast) = game
        .nav
        .graph
        .as_ref()
        .and_then(|g| g.rocket_jump_of_link(link))
        .map(|t| (t.v0, t.blast, t.pos_blast))
        .unwrap_or((Vec3::ZERO, Vec3::ZERO, Vec3::ZERO));
    let traj = std::mem::take(&mut game.entities[e].bot.puppet.traj);
    // Reset the fail counter so a harness attempt doesn't leak strikes into later autonomous play, and
    // park the bot (Hold) between tests for clean, still telemetry. `now` reserved for symmetry with
    // poll_goto; the outcome carries its own timestamps.
    let _ = now;
    game.entities[e].bot.rj.fails = 0;
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
    let json = rj_result_json(bot, link, &telem, outcome, v0, blast, pos_blast, &traj);
    send(game, json);
}

#[allow(clippy::too_many_arguments)] // one JSON event's worth of measured + solved fields
fn rj_result_json(
    bot: u32,
    link: u32,
    t: &RjTelemetry,
    outcome: RjOutcome,
    v0: Vec3,
    blast: Vec3,
    pos_blast: Vec3,
    traj: &[(f32, Vec3, Vec3)],
) -> String {
    // Terminal name + (for a touchdown/overrun) the landing measurement vs the target cell.
    let (name, land) = match outcome {
        RjOutcome::Landed { on_target, origin, t: ft } => {
            (if on_target { "landed" } else { "landed_off" }, Some((origin, ft)))
        }
        RjOutcome::Overran { origin, t: ft } => ("overran", Some((origin, ft))),
        RjOutcome::StanceTimeout => ("stance_timeout", None),
        RjOutcome::LiftoffTimeout => ("liftoff_timeout", None),
        RjOutcome::Unfit => ("unfit", None),
        RjOutcome::EnemyAbort => ("enemy_abort", None),
        RjOutcome::LegVanished => ("leg_vanished", None),
    };
    let press = match t.press {
        Some(p) => format!(
            "{{\"t\":{},\"origin\":{},\"view\":[{},{}],\"aim_err\":{},\"stance_off_xy\":{}}}",
            jnum(p.t),
            jvec3(p.origin),
            jnum(p.view.x),
            jnum(p.view.y),
            jnum(p.aim_err),
            jnum((p.origin.xy() - t.src.xy()).length()),
        ),
        None => "null".to_string(),
    };
    let fire = match t.fire {
        Some(f) => format!(
            "{{\"t\":{},\"delay\":{},\"origin\":{},\"view\":[{},{}],\"pitch_err\":{},\"yaw_err\":{}}}",
            jnum(f.t),
            jnum(f.actual_delay),
            jvec3(f.origin),
            jnum(f.view.x),
            jnum(f.view.y),
            jnum(f.view.x - (t.solved_angles.x + t.pitch_bias)),
            jnum(wrap180(f.view.y - t.solved_angles.y)),
        ),
        None => "null".to_string(),
    };
    let land = match land {
        Some((o, ft)) => format!(
            "{{\"t\":{},\"origin\":{},\"miss_xy\":{},\"miss_z\":{}}}",
            jnum(ft),
            jvec3(o),
            jnum((o.xy() - t.tgt.xy()).length()),
            jnum((o.z - t.tgt.z).abs()),
        ),
        None => "null".to_string(),
    };
    // The flight trace: one [t, x,y,z, vx,vy,vz] per frame from stance through landing.
    let mut trace = String::new();
    for (ts, o, v) in traj {
        if !trace.is_empty() {
            trace.push(',');
        }
        trace.push_str(&format!(
            "[{},{},{},{},{},{},{}]",
            jnum(*ts), jnum(o.x), jnum(o.y), jnum(o.z), jnum(v.x), jnum(v.y), jnum(v.z)
        ));
    }
    format!(
        "{{\"ev\":\"rj_result\",\"bot\":{bot},\"link\":{link},\"outcome\":{},\"src\":{},\"tgt\":{},\
         \"solved\":{{\"pitch\":{},\"yaw\":{},\"delay\":{},\"airtime\":{},\"self_damage\":{},\
         \"v0\":{},\"blast\":{},\"pos_blast\":{}}},\
         \"bias\":{{\"delay\":{},\"pitch\":{}}},\"press\":{press},\"fire\":{fire},\"land\":{land},\
         \"traj\":[{trace}]}}",
        jstr(name),
        jvec3(t.src),
        jvec3(t.tgt),
        jnum(t.solved_angles.x),
        jnum(t.solved_angles.y),
        jnum(t.solved_delay),
        jnum(t.airtime),
        jnum(t.self_damage),
        jvec3(v0),
        jvec3(blast),
        jvec3(pos_blast),
        jnum(t.delay_bias),
        jnum(t.pitch_bias),
    )
}

// --- tiny JSON emitters (no serde: the game crate stays dependency-free) ---

/// A finite `f32` as a JSON number (shortest round-trip form); non-finite → `null`.
fn jnum(x: f32) -> String {
    if x.is_finite() {
        format!("{x}")
    } else {
        "null".to_string()
    }
}

/// A `Vec3` as a JSON `[x,y,z]` array.
fn jvec3(v: Vec3) -> String {
    format!("[{},{},{}]", jnum(v.x), jnum(v.y), jnum(v.z))
}

/// A `&str` as a quoted, escaped JSON string.
fn jstr(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_verbs() {
        assert_eq!(parse_line("7 status").unwrap(), (7, ControlCmd::Status));
        assert_eq!(parse_line("  12   links  ").unwrap(), (12, ControlCmd::Links));
        assert_eq!(parse_line("3 hold 1").unwrap(), (3, ControlCmd::Hold { bot: 1 }));
        assert_eq!(parse_line("4 stop 2").unwrap(), (4, ControlCmd::Stop { bot: 2 }));
    }

    #[test]
    fn parses_vectors_and_links() {
        assert_eq!(
            parse_line("1 teleport 1 10 -20.5 300").unwrap(),
            (1, ControlCmd::Teleport { bot: 1, pos: Vec3::new(10.0, -20.5, 300.0) })
        );
        assert_eq!(
            parse_line("2 goto 1 0 0 0").unwrap(),
            (2, ControlCmd::Goto { bot: 1, pos: Vec3::ZERO })
        );
        assert_eq!(parse_line("9 rj 1 412").unwrap(), (9, ControlCmd::Rj { bot: 1, link: 412 }));
    }

    #[test]
    fn parses_prep_defaults_and_overrides() {
        assert_eq!(
            parse_line("1 prep 1").unwrap(),
            (1, ControlCmd::Prep { bot: 1, health: 100.0, rockets: 10.0 })
        );
        assert_eq!(
            parse_line("1 prep 1 50 3").unwrap(),
            (1, ControlCmd::Prep { bot: 1, health: 50.0, rockets: 3.0 })
        );
    }

    #[test]
    fn set_and_cmd_take_rest_of_line() {
        assert_eq!(
            parse_line("5 set rtx_rj_delay_bias 0.05").unwrap(),
            (5, ControlCmd::Set { name: "rtx_rj_delay_bias".into(), value: "0.05".into() })
        );
        assert_eq!(
            parse_line("6 get rtx_rj_stance").unwrap(),
            (6, ControlCmd::Get { name: "rtx_rj_stance".into() })
        );
        assert_eq!(
            parse_line("8 cmd map bravado").unwrap(),
            (8, ControlCmd::Cmd { raw: "map bravado".into() })
        );
    }

    #[test]
    fn rejects_malformed() {
        assert!(parse_line("").is_err());
        assert!(parse_line("notanumber status").is_err());
        assert!(parse_line("1 bogusverb").is_err());
        assert!(parse_line("1 rj 1").is_err()); // missing link
        assert!(parse_line("1 teleport 1 0 0").is_err()); // missing z
        assert!(parse_line("1 set").is_err()); // missing cvar
    }

    #[test]
    fn cvar_name_guard() {
        assert!(valid_cvar_name("rtx_rj_stance"));
        assert!(!valid_cvar_name("rtx; quit"));
        assert!(!valid_cvar_name(""));
        assert!(!valid_cvar_name("foo bar"));
    }

    #[test]
    fn json_helpers_escape_and_format() {
        assert_eq!(jnum(16.0), "16");
        assert_eq!(jnum(f32::NAN), "null");
        assert_eq!(jvec3(Vec3::new(1.0, 2.0, 3.0)), "[1,2,3]");
        assert_eq!(jstr("a\"b\\c"), "\"a\\\"b\\\\c\"");
    }
}
