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
/// or after a bounded finish-plane crossing, and within [`GOTO_ARRIVE_Z`] in Z. This stays independent
/// of navmesh cell borders, which flap at high speed.
const GOTO_ARRIVE_XY: f32 = 24.0;
const GOTO_ARRIVE_Z: f32 = 48.0;
/// A fast directed run can cross the target plane between samples while one slalom lobe is outside
/// the radial arrival ball. Accept that crossing inside the same bounded corridor used for fast
/// route waypoints, so the control order stops at the finish instead of commanding a recovery turn.
const GOTO_FINISH_CORRIDOR: f32 = 96.0;
/// Goto stall: if the straight-line XY distance to the target hasn't improved by [`STALL_EPS`] for
/// [`STALL_SECS`], the source is (currently) inaccessible. The window sits above the bot's own 2.5 s
/// progress watchdog, so it gets one penalize-and-divert attempt first — a stall then means
/// "unreachable even after diverting", the signal a rocket-jump *source* cell can't be stood on.
const STALL_EPS: f32 = 16.0;
const STALL_SECS: f32 = 4.0;
/// A FlyLink attempt gives up after this long with no touchdown (see `poll_fly`).
const FLY_TIMEOUT: f32 = 8.0;

/// The control channel's live state, carried on [`GameState`]. Persists across map loads (the socket
/// binds once); `started` guards against re-binding. All fields stay untouched — the whole harness is
/// inert — until `rtx_control_port` is set to a real port and the first frame binds the listener.
#[derive(Default)]
pub(crate) struct ControlState {
    /// Whether the listener has been (attempted to be) bound. Set once, so a bind is tried at most once.
    started: bool,
    /// Inbound command lines from the current connection (drained each frame in [`frame_begin`]).
    lines_rx: Option<Receiver<String>>,
    /// Outbound JSON lines (replies + events). The writer thread owns the receiving half.
    events_tx: Option<Sender<String>>,
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
            Some(ControlOrder::Goto { target }) => {
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let traj = &mut game.entities[e].bot.puppet.traj;
                // Long flat-corridor benchmarks need roughly 7–10 seconds to expose the 800+ ups
                // regime. Keep their complete velocity trace; the old 400-row cap truncated the
                // final acceleration and made the reported peak systematically too low.
                if traj.len() < 1200 {
                    traj.push((now, origin, vel));
                }
                poll_goto(game, e, i, target, now);
            }
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
            Some(ControlOrder::FlyLink { link }) => {
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let traj = &mut game.entities[e].bot.puppet.traj;
                if traj.len() < 400 {
                    traj.push((now, origin, vel));
                }
                poll_fly(game, e, i, link, now);
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
            game.host.conprint(&cstring(&format!(
                "rtx: control: bind 127.0.0.1:{port} failed: {err}\n"
            )));
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
    MatchStart,
    Links,
    Prep {
        bot: u32,
        health: f32,
        rockets: f32,
    },
    Teleport {
        bot: u32,
        pos: Vec3,
    },
    Goto {
        bot: u32,
        pos: Vec3,
    },
    Rj {
        bot: u32,
        link: u32,
    },
    /// Fly a non-RJ link (e.g. a planted speed/curl jump) via the normal steer/bhop path. Reports a
    /// `fly_result` with the takeoff speed and landing measurement.
    Fly {
        bot: u32,
        link: u32,
    },
    Hold {
        bot: u32,
    },
    Stop {
        bot: u32,
    },
    Set {
        name: String,
        value: String,
    },
    Get {
        name: String,
    },
    Cmd {
        raw: String,
    },
    /// Inspect the navmesh cell nearest a world point: its origin and every link in/out, by kind.
    Cell {
        pos: Vec3,
    },
    /// Dump a bot's current A* route: each leg's index, kind, source and target.
    Route {
        bot: u32,
    },
    /// List every generated curl link (a SpeedJump with `curl_gain > 0`): index, from, takeoff, target,
    /// v_req, gain — for verifying which gaps the build's curl certifier covered.
    Curls,
    /// Probe the build-time curl certifier from `takeoff` along `psi0`° with the speed `runway` delivers,
    /// onto `tgt`: reports predicted takeoff speed, whether the envelope certifies, and per-gain landings.
    Probe {
        takeoff: Vec3,
        tgt: Vec3,
        psi0: f32,
        runway: f32,
    },
    /// Search (offline pmove sim, live BSP) for a speed-curl jump from a source to a target world
    /// point — the M2 curl-jump solver, validated live. Returns the best (v0, launch heading, gain).
    Curl {
        src: Vec3,
        tgt: Vec3,
    },
    /// Hand-plant a `SpeedJump` link (harness bring-up): a self-contained speed jump from the cell
    /// nearest `from` (the run-up start), taking off at `takeoff` (the lip), to the cell nearest `tgt`,
    /// requiring `v_req` ups at the lip. Lets us fly the takeoff regime before the generator emits it.
    PlanLink {
        from: Vec3,
        takeoff: Vec3,
        tgt: Vec3,
        v_req: f32,
    },
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
        "match_start" => ControlCmd::MatchStart,
        "links" => ControlCmd::Links,
        "prep" => {
            let mut t = rest.split_whitespace();
            let bot = parse_u32(t.next(), "bot")?;
            let health = t
                .next()
                .map(|s| s.parse::<f32>())
                .transpose()
                .map_err(|_| "bad health")?
                .unwrap_or(100.0);
            let rockets = t
                .next()
                .map(|s| s.parse::<f32>())
                .transpose()
                .map_err(|_| "bad rockets")?
                .unwrap_or(10.0);
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
        "fly" => {
            let mut t = rest.split_whitespace();
            let bot = parse_u32(t.next(), "bot")?;
            let link = parse_u32(t.next(), "link")?;
            ControlCmd::Fly { bot, link }
        }
        "hold" => ControlCmd::Hold {
            bot: parse_u32(rest.split_whitespace().next(), "bot")?,
        },
        "stop" => ControlCmd::Stop {
            bot: parse_u32(rest.split_whitespace().next(), "bot")?,
        },
        "set" => {
            let (name, value) = split_first(rest);
            if name.is_empty() {
                return Err("set: missing cvar".into());
            }
            ControlCmd::Set {
                name: name.to_string(),
                value: value.to_string(),
            }
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
        "cell" => {
            let mut t = rest.split_whitespace();
            let x = parse_f32(t.next(), "x")?;
            let y = parse_f32(t.next(), "y")?;
            let z = parse_f32(t.next(), "z")?;
            ControlCmd::Cell {
                pos: Vec3::new(x, y, z),
            }
        }
        "route" => ControlCmd::Route {
            bot: parse_u32(rest.split_whitespace().next(), "bot")?,
        },
        "curls" => ControlCmd::Curls,
        "probe" => {
            let mut t = rest.split_whitespace();
            let takeoff = Vec3::new(
                parse_f32(t.next(), "ox")?,
                parse_f32(t.next(), "oy")?,
                parse_f32(t.next(), "oz")?,
            );
            let tgt = Vec3::new(
                parse_f32(t.next(), "tx")?,
                parse_f32(t.next(), "ty")?,
                parse_f32(t.next(), "tz")?,
            );
            let psi0 = parse_f32(t.next(), "psi0")?;
            let runway = parse_f32(t.next(), "runway")?;
            ControlCmd::Probe {
                takeoff,
                tgt,
                psi0,
                runway,
            }
        }
        "curl" => {
            let mut t = rest.split_whitespace();
            let src = Vec3::new(
                parse_f32(t.next(), "sx")?,
                parse_f32(t.next(), "sy")?,
                parse_f32(t.next(), "sz")?,
            );
            let tgt = Vec3::new(
                parse_f32(t.next(), "tx")?,
                parse_f32(t.next(), "ty")?,
                parse_f32(t.next(), "tz")?,
            );
            ControlCmd::Curl { src, tgt }
        }
        "planlink" => {
            let mut t = rest.split_whitespace();
            let from = Vec3::new(
                parse_f32(t.next(), "fx")?,
                parse_f32(t.next(), "fy")?,
                parse_f32(t.next(), "fz")?,
            );
            let takeoff = Vec3::new(
                parse_f32(t.next(), "ox")?,
                parse_f32(t.next(), "oy")?,
                parse_f32(t.next(), "oz")?,
            );
            let tgt = Vec3::new(
                parse_f32(t.next(), "tx")?,
                parse_f32(t.next(), "ty")?,
                parse_f32(t.next(), "tz")?,
            );
            let v_req = parse_f32(t.next(), "v_req")?;
            ControlCmd::PlanLink {
                from,
                takeoff,
                tgt,
                v_req,
            }
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
        ControlCmd::MatchStart => {
            crate::mode::team::start_match(game);
            Ok("{\"queued\":true}".to_string())
        }
        ControlCmd::Links => links_json(game),
        ControlCmd::Prep { bot, health, rockets } => do_prep(game, bot, health, rockets),
        ControlCmd::Teleport { bot, pos } => do_teleport(game, bot, pos),
        ControlCmd::Goto { bot, pos } => do_goto(game, bot, pos),
        ControlCmd::Rj { bot, link } => do_rj(game, bot, link),
        ControlCmd::Fly { bot, link } => do_fly(game, bot, link),
        ControlCmd::Hold { bot } => do_order(game, bot, ControlOrder::Hold),
        ControlCmd::Stop { bot } => do_stop(game, bot),
        ControlCmd::Set { name, value } => do_set(game, &name, &value),
        ControlCmd::Get { name } => do_get(game, &name),
        ControlCmd::Cmd { raw } => {
            game.host.localcmd(&raw);
            Ok("{\"queued\":true}".to_string())
        }
        ControlCmd::Cell { pos } => cell_json(game, pos),
        ControlCmd::Route { bot } => route_json(game, bot),
        ControlCmd::Curls => curls_json(game),
        ControlCmd::Probe {
            takeoff,
            tgt,
            psi0,
            runway,
        } => probe_json(game, takeoff, tgt, psi0, runway),
        ControlCmd::Curl { src, tgt } => curl_json(game, src, tgt),
        ControlCmd::PlanLink {
            from,
            takeoff,
            tgt,
            v_req,
        } => plant_link_json(game, from, takeoff, tgt, v_req),
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
    game.set_origin(e, at);
    reset_nav_state(&mut game.entities[e].bot, at, now);
    // Park the bot after placing it — otherwise, with no order, it would roam autonomously and arrive
    // at a subsequent rocket jump with residual velocity, contaminating the standstill measurement.
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
    Ok(format!(
        "{{\"bot\":{bot},\"origin\":{}}}",
        jvec3(game.entities[e].v.origin)
    ))
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
    b.puppet.traj.clear();
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

fn do_fly(game: &mut GameState, bot: u32, link: u32) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    {
        let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
        if link as usize >= g.links.len() {
            return Err(format!("link {link} out of range (0..{})", g.links.len()));
        }
        if g.link_kind(link) == LinkKind::RocketJump {
            return Err(format!("link {link} is a rocket jump — use `rj`"));
        }
    }
    let b = &mut game.entities[e].bot;
    b.route.clear();
    b.repath_time = now;
    b.puppet.traj.clear(); // fresh flight trace
    b.puppet.fly_airborne = false;
    b.puppet.fly_takeoff_speed = 0.0;
    b.puppet.best_since = now; // FlyLink stall clock (poll_fly gives up after FLY_TIMEOUT)
    b.puppet.order = Some(ControlOrder::FlyLink { link });
    Ok(format!("{{\"bot\":{bot},\"link\":{link}}}"))
}

fn do_order(game: &mut GameState, bot: u32, order: ControlOrder) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    if order == ControlOrder::Hold {
        let at = game.entities[e].v.origin;
        let now = game.time();
        game.entities[e].v.velocity = Vec3::ZERO;
        reset_nav_state(&mut game.entities[e].bot, at, now);
    }
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

fn match_phase_name(phase: crate::mode::MatchPhase) -> &'static str {
    match phase {
        crate::mode::MatchPhase::Warmup => "warmup",
        crate::mode::MatchPhase::Countdown { .. } => "countdown",
        crate::mode::MatchPhase::Live => "live",
        crate::mode::MatchPhase::Ended { .. } => "ended",
    }
}

/// A compact reference to a live entity carried by strategy telemetry. Item goals need the
/// classname + location; enemy/teammate references also benefit from the display name. Keeping the
/// reference nullable makes the `0` sentinel explicit to MCP clients instead of exposing a fake
/// world entity.
fn entity_ref_json(game: &GameState, id: u32) -> String {
    if id == 0 {
        return "null".to_string();
    }
    let Some(ent) = game.entities.get(id as usize).filter(|e| e.in_use) else {
        return "null".to_string();
    };
    format!(
        "{{\"ent\":{id},\"name\":{},\"classname\":{},\"origin\":{},\"solid\":{}}}",
        jstr(&game.netname_of(EntId(id))),
        jstr(ent.classname().unwrap_or("")),
        jvec3(ent.v.origin),
        jstr(&format!("{:?}", ent.v.solid)),
    )
}

fn route_head_json(game: &GameState, e: EntId) -> String {
    let b = &game.entities[e].bot;
    let Some(g) = game.nav.graph.as_ref() else {
        return format!("{{\"pos\":{},\"len\":{},\"next\":null}}", b.route_pos, b.route.len());
    };
    let next = b.route.get(b.route_pos).map_or_else(
        || "null".to_string(),
        |&link| {
            format!(
                "{{\"link\":{link},\"kind\":{},\"target\":{}}}",
                jstr(kind_name(g.link_kind(link))),
                jvec3(g.cell_origin(g.link_target(link))),
            )
        },
    );
    format!("{{\"pos\":{},\"len\":{},\"next\":{next}}}", b.route_pos, b.route.len())
}

fn match_json(game: &GameState) -> String {
    let cfg = game.team_match.config;
    let mut scores = Vec::with_capacity(cfg.teams);
    for team in 1..=cfg.teams {
        let score = game
            .entities
            .iter()
            .filter(|e| e.is_player() && e.in_use && e.mode_p.team as usize == team)
            .map(|e| e.v.frags as i32)
            .sum::<i32>();
        scores.push(score.to_string());
    }
    let mut roster = String::new();
    for (name, team) in &game.team_match.roster {
        if !roster.is_empty() {
            roster.push(',');
        }
        roster.push_str(&format!("{{\"name\":{},\"team\":{team}}}", jstr(name)));
    }
    format!(
        "{{\"mode\":{},\"format\":{},\"phase\":{},\"teams\":{},\"size\":{},\"teamplay\":{},\"timelimit\":{},\"fraglimit\":{},\"live_until\":{},\"scores\":[{}],\"roster\":[{roster}]}}",
        jstr(game.mode.name()),
        jstr(&crate::mode::team::format_label(cfg)),
        jstr(match_phase_name(game.team_match.phase)),
        cfg.teams,
        cfg.size,
        game.level.teamplay,
        game.level.timelimit,
        game.level.fraglimit,
        jnum(game.team_match.live_until),
        scores.join(","),
    )
}

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
        let b = &ent.bot;
        bots.push_str(&format!(
            "{{\"ent\":{i},\"client\":{},\"name\":{},\"team\":{},\"team_name\":{},\"frags\":{},\"origin\":{},\"health\":{},\"armor\":{},\"armor_type\":{},\"weapon\":{},\"items\":{},\"ammo\":{{\"shells\":{},\"nails\":{},\"rockets\":{},\"cells\":{}}},\"on_ground\":{},\"alive\":{},\"order\":{},\"posture\":{},\"known_enemy\":{},\"goal\":{{\"item\":{},\"commit\":{},\"since\":{},\"next_item\":{},\"hold_item\":{},\"hold_for\":{}}},\"route\":{},\"rj_phase\":{},\"speed\":{},\"bhop\":{},\"bhop_peak\":{}}}",
            b.client,
            jstr(&game.netname_of(EntId(i))),
            ent.mode_p.team,
            jstr(&game.team_of(EntId(i))),
            jnum(ent.v.frags),
            jvec3(ent.v.origin),
            jnum(ent.v.health),
            jnum(ent.v.armorvalue),
            jnum(ent.v.armortype),
            jstr(&format!("{:?}", ent.v.weapon)),
            jstr(&format!("{:?}", Items::from_f32(ent.v.items))),
            jnum(ent.v.ammo_shells),
            jnum(ent.v.ammo_nails),
            jnum(ent.v.ammo_rockets),
            jnum(ent.v.ammo_cells),
            ent.v.flags.has(Flags::ONGROUND),
            ent.is_alive(),
            jstr(order_name(b.puppet.order)),
            jstr(&format!("{:?}", b.posture)),
            entity_ref_json(game, b.percept.known_enemy),
            entity_ref_json(game, b.goal.item),
            jstr(&format!("{:?}", b.goal.commit)),
            jnum(b.goal.since),
            entity_ref_json(game, b.goal.next_item),
            entity_ref_json(game, b.goal.hold_item),
            entity_ref_json(game, b.goal.hold_for),
            route_head_json(game, EntId(i)),
            jstr(&format!("{:?}", b.rj.phase)),
            jnum(ent.v.velocity.xy().length()),
            jstr(&format!("{:?}", b.bhop.phase)),
            jnum(b.bhop.peak),
        ));
    }
    let oracle = oracle_json(game);
    format!(
        "{{\"map\":{},\"time\":{},\"navmesh\":{},\"cells\":{cells},\"links\":{links},\"rj_links\":{rj_links},\"match\":{},\"oracle\":{oracle},\"bots\":[{bots}]}}",
        jstr(&game.level.mapname),
        jnum(game.time()),
        jstr(navmesh),
        match_json(game),
    )
}

fn oracle_json(game: &GameState) -> String {
    let eval = game.oracle.eval_summary();
    let episode_eval = game.oracle.eval_episode_summary();
    let comms = game.oracle.communication_summary();
    let mut by_kind = String::new();
    let mut episodes_by_kind = String::new();
    for kind in crate::bot::oracle::NUGGET_KINDS {
        if !by_kind.is_empty() {
            by_kind.push(',');
        }
        let summary = game.oracle.eval_summary_for(kind);
        by_kind.push_str(&format!(
            "{}:{{\"treated\":{},\"treated_success\":{},\"controls\":{},\"control_success\":{},\"applied\":{},\"invalidated\":{},\"pending\":{}}}",
            jstr(&format!("{:?}", kind)),
            summary.treated,
            summary.treated_success,
            summary.controls,
            summary.control_success,
            summary.applied,
            summary.invalidated,
            summary.pending,
        ));
        if !episodes_by_kind.is_empty() {
            episodes_by_kind.push(',');
        }
        let summary = game.oracle.eval_episode_summary_for(kind);
        episodes_by_kind.push_str(&format!(
            "{}:{{\"treated\":{},\"treated_success\":{},\"controls\":{},\"control_success\":{},\"applied\":{},\"invalidated\":{},\"pending\":{}}}",
            jstr(&format!("{:?}", kind)),
            summary.treated,
            summary.treated_success,
            summary.controls,
            summary.control_success,
            summary.applied,
            summary.invalidated,
            summary.pending,
        ));
    }
    let episode_eval_json = format!(
        "{{\"treated\":{},\"treated_success\":{},\"controls\":{},\"control_success\":{},\"applied\":{},\"invalidated\":{},\"pending\":{},\"by_kind\":{{{episodes_by_kind}}}}}",
        episode_eval.treated,
        episode_eval.treated_success,
        episode_eval.controls,
        episode_eval.control_success,
        episode_eval.applied,
        episode_eval.invalidated,
        episode_eval.pending,
    );
    let eval_json = format!(
        "{{\"treated\":{},\"treated_success\":{},\"controls\":{},\"control_success\":{},\"applied\":{},\"invalidated\":{},\"pending\":{},\"by_kind\":{{{by_kind}}},\"episodes\":{episode_eval_json}}}",
        eval.treated,
        eval.treated_success,
        eval.controls,
        eval.control_success,
        eval.applied,
        eval.invalidated,
        eval.pending,
    );
    let plan = game.oracle.last_plan().map_or_else(
        || "null".to_string(),
        |plan| {
            let mut teams = String::new();
            for team in &plan.teams {
                if !teams.is_empty() {
                    teams.push(',');
                }
                let mut nuggets = String::new();
                for nugget in &team.nuggets {
                    if !nuggets.is_empty() {
                        nuggets.push(',');
                    }
                    nuggets.push_str(&format!(
                        "{{\"recipient\":{},\"kind\":{},\"target_cell\":{},\"subject\":{},\"confidence\":{},\"decision_at\":{},\"evidence_at\":{},\"expires_at\":{}}}",
                        nugget.recipient,
                        jstr(&format!("{:?}", nugget.kind)),
                        nugget.target_cell,
                        nugget.subject,
                        jnum(nugget.confidence),
                        jnum(nugget.decision_at),
                        jnum(nugget.evidence_at),
                        jnum(nugget.expires_at),
                    ));
                }
                teams.push_str(&format!(
                    "{{\"team\":{},\"mode\":{},\"control\":{},\"nuggets\":[{nuggets}]}}",
                    team.team,
                    jstr(&format!("{:?}", team.mode)),
                    jstr(&format!("{:?}", team.control)),
                ));
            }
            format!(
                "{{\"generation\":{},\"at\":{},\"teams\":[{teams}]}}",
                plan.generation,
                jnum(plan.at),
            )
        },
    );
    format!(
        "{{\"running\":{},\"epoch\":{},\"last_output\":{},\"plan\":{plan},\"communication\":{{\"proposed\":{},\"communicated\":{},\"refreshed\":{},\"suppressed\":{},\"superseded\":{},\"arm_clears\":{}}},\"eval\":{eval_json}}}",
        game.oracle.running(),
        game.oracle.epoch(),
        jnum(game.oracle.last_output()),
        comms.proposed,
        comms.communicated,
        comms.refreshed,
        comms.suppressed,
        comms.superseded,
        comms.arm_clears,
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
            "{{\"link\":{li},\"src\":{},\"tgt\":{},\"fire_pitch\":{},\"fire_yaw\":{},\"fire_delay\":{},\"airtime\":{},\"self_damage\":{},\"v0\":{},\"blast\":{},\"pos_blast\":{},\"land\":{}}}",
            jvec3(src),
            jvec3(tgt),
            jnum(tr.fire_angles.x),
            jnum(tr.fire_angles.y),
            jnum(tr.fire_delay),
            jnum(tr.airtime),
            jnum(tr.self_damage),
            jvec3(tr.v0),
            jvec3(tr.blast),
            jvec3(tr.pos_blast),
            jvec3(tr.land),
        ));
    }
    Ok(format!("{{\"links\":[{items}]}}"))
}

/// Human-readable name for a link kind, for the `cell` inspector.
fn kind_name(k: LinkKind) -> &'static str {
    match k {
        LinkKind::Walk => "walk",
        LinkKind::Step => "step",
        LinkKind::Drop => "drop",
        LinkKind::JumpGap => "jump",
        LinkKind::DoubleJump => "doublejump",
        LinkKind::SpeedJump => "speedjump",
        LinkKind::Plat => "plat",
        LinkKind::Teleport => "teleport",
        LinkKind::Hook => "hook",
        LinkKind::RocketJump => "rocketjump",
    }
}

/// Inspect the navmesh cell nearest `pos`: its origin plus every link leaving and entering it (index,
/// kind, other endpoint). The diagnostic for "why can't the bot reach here" — an unreachable ledge
/// has no incoming jump/speed-jump link.
fn cell_json(game: &GameState, pos: Vec3) -> Result<String, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let cell = g.nearest(pos).ok_or("no navmesh cell near that point")?;
    let mut out = String::new();
    let mut inc = String::new();
    for li in 0..g.links.len() as u32 {
        if g.link_source(li) == cell {
            let e = if out.is_empty() { "" } else { "," };
            // `cost` is the static travel time only. What a hazard link *really* costs the planner is
            // `hazard_hp` valued against the asking bot's strength, so report the health and let the
            // caller price it — reporting seconds here would mean picking a bot to price it for.
            out.push_str(&format!(
                "{e}{{\"link\":{li},\"kind\":{},\"to\":{},\"cost\":{:.2},\"tgt_hazard\":{},\"hazard_hp\":{:.2},\"water_extra\":{:.2}}}",
                jstr(kind_name(g.link_kind(li))),
                jvec3(g.cell_origin(g.link_target(li))),
                g.link_cost(li),
                jstr(&format!("{:?}", g.cell_hazard(g.link_target(li)))),
                g.link_hazard_hp(li),
                g.link_water_extra(li),
            ));
        }
        if g.link_target(li) == cell {
            let e = if inc.is_empty() { "" } else { "," };
            inc.push_str(&format!(
                "{e}{{\"link\":{li},\"kind\":{},\"from\":{}}}",
                jstr(kind_name(g.link_kind(li))),
                jvec3(g.cell_origin(g.link_source(li)))
            ));
        }
    }
    Ok(format!(
        "{{\"cell\":{},\"origin\":{},\"hazard\":{},\"out\":[{out}],\"in\":[{inc}]}}",
        cell,
        jvec3(g.cell_origin(cell)),
        jstr(&format!("{:?}", g.cell_hazard(cell)))
    ))
}

/// Dump a bot's current route: `route_pos` and each leg (index, kind, source→target).
fn route_json(game: &GameState, bot: u32) -> Result<String, String> {
    let e = valid_bot(game, bot)?;
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let b = &game.entities[e].bot;
    let mut legs = String::new();
    for (i, &leg) in b.route.iter().enumerate() {
        if !legs.is_empty() {
            legs.push(',');
        }
        legs.push_str(&format!(
            "{{\"i\":{i},\"link\":{leg},\"kind\":{},\"src\":{},\"tgt\":{}}}",
            jstr(kind_name(g.link_kind(leg))),
            jvec3(g.cell_origin(g.link_source(leg))),
            jvec3(g.cell_origin(g.link_target(leg))),
        ));
    }
    Ok(format!(
        "{{\"bot\":{bot},\"route_pos\":{},\"origin\":{},\"legs\":[{legs}]}}",
        b.route_pos,
        jvec3(game.entities[e].v.origin),
    ))
}

/// Search the offline pmove sim (against the live BSP) for a speed-curl jump from `src` to `tgt`: a
/// held-strafe air-curl from a run-up-built takeoff speed. Grid-searches takeoff speed `v0`, launch
/// heading `psi0`, and turn gain, returning the lowest-speed curl that lands within tolerance — the
/// M2 solver, exercised live. Mirrors the human demo (build speed, one leap, gentle held-strafe sweep).
fn curl_json(game: &GameState, src: Vec3, tgt: Vec3) -> Result<String, String> {
    use crate::bot::bhop;
    use crate::math::{wrap180, yaw_of};
    use crate::pmove_sim::{pm_step, PmParams, PmState};
    let bsp = game.nav.bsp.as_ref().ok_or("no bsp loaded")?;
    let cv = |name: &std::ffi::CStr, d: f32| {
        let v = game.host.cvar(name);
        if v > 0.0 {
            v
        } else {
            d
        }
    };
    let p = PmParams {
        gravity: cv(c"sv_gravity", 800.0),
        accel: cv(c"sv_accelerate", 10.0),
        friction: cv(c"sv_friction", 4.0),
        stopspeed: 100.0,
        maxspeed: cv(c"sv_maxspeed", 320.0),
    };
    let dt = 0.013_f32;
    let amax = bhop::air_accel_max(p.accel, p.maxspeed, dt);
    let rollout = |v0: f32, psi0: f32, gain: f32| -> Option<Vec3> {
        let mut s = PmState {
            origin: src,
            vel: Vec3::new(v0 * psi0.to_radians().cos(), v0 * psi0.to_radians().sin(), 0.0),
            on_ground: true,
            jump_held: false,
        };
        let sigma = wrap180(yaw_of(tgt.xy() - src.xy()) - psi0).signum();
        for tick in 0..100 {
            let cmd = if tick == 0 {
                bhop::Cmd {
                    view_yaw: psi0,
                    forward: 400.0,
                    side: 0.0,
                    jump: true,
                }
            } else {
                let v_xy = s.vel.xy();
                let err = wrap180(yaw_of(tgt.xy() - s.origin.xy()) - yaw_of(v_xy));
                let omega = (err.abs() * gain).min(bhop::omega_gain_max(v_xy.length().max(1.0), amax, dt));
                let st = bhop::strafe_rate(v_xy, sigma, omega, amax, dt);
                bhop::Cmd {
                    view_yaw: st.view_yaw,
                    forward: st.forward,
                    side: st.side,
                    jump: false,
                }
            };
            pm_step(bsp, &mut s, &cmd, &p, dt);
            if tick > 3 && s.on_ground {
                return Some(s.origin);
            }
        }
        None
    };
    let chord = yaw_of(tgt.xy() - src.xy());
    let mut best: Option<(f32, f32, f32, f32, Vec3)> = None;
    for vi in 0..10 {
        let v0 = 340.0 + vi as f32 * 15.0;
        for pi in 0..24 {
            let psi0 = chord - 60.0 + pi as f32 * 4.0;
            for gi in 0..8 {
                let gain = 1.0 + gi as f32 * 0.4;
                if let Some(land) = rollout(v0, psi0, gain) {
                    let miss = (land.xy() - tgt.xy()).length();
                    if (land.z - tgt.z).abs() < 40.0 && best.is_none_or(|b| miss < b.3) {
                        best = Some((v0, psi0, gain, miss, land));
                    }
                }
            }
        }
    }
    match best {
        Some((v0, psi0, gain, miss, land)) => Ok(format!(
            "{{\"found\":true,\"v0\":{},\"psi0\":{},\"chord\":{},\"gain\":{},\"miss_xy\":{},\"land\":{}}}",
            jnum(v0),
            jnum(psi0),
            jnum(chord),
            jnum(gain),
            jnum(miss),
            jvec3(land)
        )),
        None => Ok(format!("{{\"found\":false,\"chord\":{}}}", jnum(chord))),
    }
}

/// Hand-plant a self-contained `SpeedJump` link into the live graph for takeoff-regime bring-up: the
/// run-up starts at the cell nearest `from`, the leap is at `takeoff` (the lip), and it lands on the
/// cell nearest `tgt`, requiring `v_req` ups at the lip. The runtime flies a planted link exactly like
/// a generated one, so a subsequent `goto <tgt>` exercises the committed-prestrafe takeoff on the real
/// corridor. Returns the new link index and the resolved cell origins so the caller can verify routing.
fn plant_link_json(game: &mut GameState, from: Vec3, takeoff: Vec3, tgt: Vec3, v_req: f32) -> Result<String, String> {
    use crate::navmesh::SpeedJumpTraversal;
    let gravity = {
        let g = game.host.cvar(c"sv_gravity");
        if g > 0.0 {
            g
        } else {
            800.0
        }
    };
    let graph = game.nav.graph.as_mut().ok_or("navmesh not ready")?;
    let g = std::sync::Arc::get_mut(graph).ok_or("navmesh is shared with the team oracle")?;
    let from_cell = g.nearest(from).ok_or("no cell near from")?;
    let to_cell = g.nearest(tgt).ok_or("no cell near tgt")?;
    let dz = g.cell_origin(to_cell).z - takeoff.z;
    // Ballistic flight time to fall back through `dz` after a jump (vz0 = JUMP_VZ): the later root of
    // dz = JUMP_VZ·t − ½·g·t². Only used for the planner's hot-entry pricing; the flight itself is
    // driven by v_req + takeoff at runtime.
    let vz0 = rtx_nav::qphys::JUMP_VZ;
    let disc = (vz0 * vz0 - 2.0 * gravity * dz).max(0.0);
    let airtime = (vz0 + disc.sqrt()) / gravity;
    // A hand-planted link is a curl by default (it's what we plant for the curl bring-up); the runtime
    // reads this gain to pick `air_correct` over the slalom. A fast run-up overshoots a gentle curl, so
    // the bring-up default is a firm gain that bleeds the excess onto the landing (see the harness gain
    // sweep — ~12 lands the bravado LG dead-on). The cvar overrides it for tuning; step 4's solver will
    // compute a per-link gain from the certified takeoff speed.
    let curl_gain = {
        let g = game.host.cvar(c"rtx_jump_curl_gain");
        if g > 0.0 {
            g
        } else {
            12.0
        }
    };
    // Curl-link cost the banded planner now trusts (see `banded_step`): the honest run-up travel +
    // flight + a JumpGap-grade commitment (a rollout-certified envelope carries less risk than the
    // +1.0 charged to a modeled speed jump). Run-up is the `from`→lip distance at the mean build speed.
    let runup = (takeoff.xy() - g.cell_origin(from_cell).xy()).length();
    let cost = runup / 400.0 + airtime + 0.3;
    let tr = SpeedJumpTraversal {
        takeoff,
        v_req,
        airtime,
        chained: false,
        curl_gain,
    };
    let li = g.plant_speed_jump(from_cell, to_cell, cost, tr);
    // Refresh the reachability + LOD tables so the new link is visible to steer's O(1) reachable()
    // gate and the coarse router — otherwise a `goto` across the plant redirects to the nearest cell
    // reachable on the pre-plant graph instead of pathing over it.
    g.rebuild_derived();
    let (fo, to) = (g.cell_origin(from_cell), g.cell_origin(to_cell));
    Ok(format!(
        "{{\"link\":{li},\"from_cell\":{from_cell},\"to_cell\":{to_cell},\"from\":{},\"tgt\":{},\"takeoff\":{},\"v_req\":{},\"airtime\":{},\"cost\":{}}}",
        jvec3(fo), jvec3(to), jvec3(takeoff), jnum(v_req), jnum(airtime), jnum(cost),
    ))
}

/// Probe the build-time curl certifier — see `ControlCmd::Probe`.
fn probe_json(game: &GameState, takeoff: Vec3, tgt: Vec3, psi0: f32, runway: f32) -> Result<String, String> {
    let bsp = game.nav.bsp.as_ref().ok_or("no bsp")?;
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let cv = |n: &std::ffi::CStr, d: f32| {
        let v = game.host.cvar(n);
        if v > 0.0 {
            v
        } else {
            d
        }
    };
    let params = crate::navmesh::SpeedJumpParams {
        gravity: cv(c"sv_gravity", 800.0),
        accel: cv(c"sv_accelerate", 10.0),
        maxspeed: cv(c"sv_maxspeed", 320.0),
        friction: cv(c"sv_friction", 4.0),
        stopspeed: cv(c"sv_stopspeed", 100.0),
        curl: true,
    };
    let probe = g.curl_probe(bsp, takeoff, tgt, psi0, runway, params);
    let mut d = String::new();
    for (gain, land) in probe.landings {
        if !d.is_empty() {
            d.push(',');
        }
        let miss = (land.truncate() - tgt.truncate()).length();
        d.push_str(&format!(
            "{{\"gain\":{},\"land\":{},\"miss_xy\":{},\"miss_z\":{}}}",
            jnum(gain),
            jvec3(land),
            jnum(miss),
            jnum((land.z - tgt.z).abs())
        ));
    }
    let cert_s = match probe.certified {
        Some((v_req, gain)) => format!("{{\"v_req\":{},\"gain\":{}}}", jnum(v_req), jnum(gain)),
        None => "null".to_string(),
    };
    Ok(format!(
        "{{\"v_deliver\":{},\"certified\":{cert_s},\"gains\":[{d}]}}",
        jnum(probe.v_deliver)
    ))
}

/// List every generated curl link (SpeedJump with `curl_gain > 0`).
fn curls_json(game: &GameState) -> Result<String, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let mut items = String::new();
    for li in 0..g.links.len() as u32 {
        if g.link_kind(li) != LinkKind::SpeedJump {
            continue;
        }
        let Some(tr) = g.speed_jump_of_link(li) else { continue };
        if tr.curl_gain <= 0.0 {
            continue;
        }
        if !items.is_empty() {
            items.push(',');
        }
        items.push_str(&format!(
            "{{\"link\":{li},\"from\":{},\"takeoff\":{},\"tgt\":{},\"v_req\":{},\"gain\":{}}}",
            jvec3(g.cell_origin(g.link_source(li))),
            jvec3(tr.takeoff),
            jvec3(g.cell_origin(g.link_target(li))),
            jnum(tr.v_req),
            jnum(tr.curl_gain),
        ));
    }
    Ok(format!("{{\"curls\":[{items}]}}"))
}

fn order_name(o: Option<ControlOrder>) -> &'static str {
    match o {
        None => "none",
        Some(ControlOrder::Hold) => "hold",
        Some(ControlOrder::Goto { .. }) => "goto",
        Some(ControlOrder::RocketJump { .. }) => "rj",
        Some(ControlOrder::FlyLink { .. }) => "fly",
    }
}

// --- per-frame puppet pollers (emit lifecycle events) ---

fn poll_goto(game: &mut GameState, e: EntId, bot: u32, target: Vec3, now: f32) {
    let origin = game.entities[e].v.origin;
    let dxy = (origin.xy() - target.xy()).length();
    let dz = (origin.z - target.z).abs();
    let crossed_finish = goto_crossed_finish(&game.entities[e].bot.puppet.traj, origin, target);
    if (dxy <= GOTO_ARRIVE_XY || crossed_finish) && dz <= GOTO_ARRIVE_Z {
        let traj = traj_json(&std::mem::take(&mut game.entities[e].bot.puppet.traj));
        // A goto commonly ends while the bot is airborne and carrying several hundred ups. Merely
        // swapping the order to Hold leaves the active hop controller, route, and momentum intact for
        // another frame; on a finish-line target that is enough to cross trigger_changelevel, and on
        // an ordinary target it can produce a sharp stale-route turn after the reported arrival.
        // Finish the puppet order atomically: stop the body and discard every navigation commitment
        // before the next bot frame observes Hold.
        finish_goto_hold(game, e, origin, now);
        send(
            game,
            format!(
            "{{\"ev\":\"arrived\",\"bot\":{bot},\"t\":{},\"origin\":{},\"target\":{},\"dist\":{},\"traj\":[{traj}]}}",
            jnum(now), jvec3(origin), jvec3(target), jnum(dxy),
        ),
        );
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
        let traj = traj_json(&std::mem::take(&mut game.entities[e].bot.puppet.traj));
        finish_goto_hold(game, e, origin, now);
        send(game, format!(
            "{{\"ev\":\"goto_stall\",\"bot\":{bot},\"t\":{},\"origin\":{},\"target\":{},\"dist\":{},\"best\":{},\"secs\":{},\"traj\":[{traj}]}}",
            jnum(now), jvec3(origin), jvec3(target), jnum(dxy), jnum(best_dist), jnum(STALL_SECS),
        ));
    }
}

fn goto_crossed_finish(traj: &[(f32, Vec3, Vec3)], origin: Vec3, target: Vec3) -> bool {
    let Some((_, start, _)) = traj.first() else {
        return false;
    };
    let along = (target.xy() - start.xy()).normalize_or_zero();
    if along == glam::Vec2::ZERO {
        return false;
    }
    let past = origin.xy() - target.xy();
    if past.dot(along) < 0.0 {
        return false;
    }
    (past - along * past.dot(along)).length() <= GOTO_FINISH_CORRIDOR
}

/// Stop a completed puppet goto without letting its route or bhop state leak into the Hold order.
fn finish_goto_hold(game: &mut GameState, e: EntId, at: Vec3, now: f32) {
    game.entities[e].v.velocity = Vec3::ZERO;
    reset_nav_state(&mut game.entities[e].bot, at, now);
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
}

/// Serialize a flight/goto trace as `[t, x,y,z, vx,vy,vz]` rows.
fn traj_json(traj: &[(f32, Vec3, Vec3)]) -> String {
    let mut s = String::new();
    for (ts, o, v) in traj {
        if !s.is_empty() {
            s.push(',');
        }
        s.push_str(&format!(
            "[{},{},{},{},{},{},{}]",
            jnum(*ts),
            jnum(o.x),
            jnum(o.y),
            jnum(o.z),
            jnum(v.x),
            jnum(v.y),
            jnum(v.z)
        ));
    }
    s
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

/// Watch a FlyLink attempt: capture the horizontal speed at the speed-jump takeoff (the first airborne
/// frame past the lip, so corridor hops on the run-up don't count), and on the next touchdown emit a
/// `fly_result` with the landing measurement vs the target cell. Then park the bot (Hold).
fn poll_fly(game: &mut GameState, e: EntId, bot: u32, link: u32, now: f32) {
    // Stall timeout: a FlyLink that never gets airborne past the lip (blocked run-up, aborted-and-
    // repathed, or fell) would otherwise pin the order forever and hang the harness. Give up after
    // FLY_TIMEOUT with a `timeout` result so a fly-rate sweep always advances.
    if now - game.entities[e].bot.puppet.best_since > FLY_TIMEOUT {
        let origin = game.entities[e].v.origin;
        game.entities[e].bot.puppet.fly_airborne = false;
        game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
        game.entities[e].bot.rj.fails = 0;
        let _ = std::mem::take(&mut game.entities[e].bot.puppet.traj);
        send(
            game,
            format!(
                "{{\"ev\":\"fly_result\",\"bot\":{bot},\"link\":{link},\"on_target\":false,\"timeout\":true,\
             \"land\":{},\"miss_xy\":9999,\"miss_z\":9999,\"takeoff_speed\":0,\"peak\":0,\"traj\":[]}}",
                jvec3(origin),
            ),
        );
        return;
    }
    let og = game.entities[e].v.flags.has(Flags::ONGROUND);
    let origin = game.entities[e].v.origin;
    let speed = game.entities[e].v.velocity.xy().length();
    let Some(g) = game.nav.graph.as_ref() else { return };
    let takeoff = g
        .speed_jump_of_link(link)
        .map(|t| t.takeoff)
        .unwrap_or_else(|| g.cell_origin(g.link_source(link)));
    let target = g.cell_origin(g.link_target(link));
    // "Past the lip" = progress along takeoff→target is positive, so the run-up (behind the lip) and its
    // corridor hops never register as the jump's flight.
    let past_lip = (origin.xy() - takeoff.xy()).dot(target.xy() - takeoff.xy()) > 0.0;
    if !game.entities[e].bot.puppet.fly_airborne {
        if !og && past_lip {
            game.entities[e].bot.puppet.fly_airborne = true;
            game.entities[e].bot.puppet.fly_takeoff_speed = speed;
        }
        return;
    }
    if !og {
        return; // still in flight
    }
    // Touchdown after the leap — measure vs the target cell and report.
    let miss_xy = (origin.xy() - target.xy()).length();
    let miss_z = (origin.z - target.z).abs();
    let on_target = miss_xy <= 32.0 && miss_z <= 32.0;
    let takeoff_speed = game.entities[e].bot.puppet.fly_takeoff_speed;
    let peak = game.entities[e].bot.bhop.peak;
    let traj = std::mem::take(&mut game.entities[e].bot.puppet.traj);
    game.entities[e].bot.puppet.fly_airborne = false;
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
    game.entities[e].bot.rj.fails = 0;
    let _ = now;
    send(
        game,
        format!(
            "{{\"ev\":\"fly_result\",\"bot\":{bot},\"link\":{link},\"on_target\":{on_target},\"land\":{},\
         \"target\":{},\"miss_xy\":{},\"miss_z\":{},\"takeoff_speed\":{},\"peak\":{},\"traj\":[{}]}}",
            jvec3(origin),
            jvec3(target),
            jnum(miss_xy),
            jnum(miss_z),
            jnum(takeoff_speed),
            jnum(peak),
            traj_json(&traj),
        ),
    );
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
        RjOutcome::Landed {
            on_target,
            origin,
            t: ft,
        } => (if on_target { "landed" } else { "landed_off" }, Some((origin, ft))),
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
            jnum(*ts),
            jnum(o.x),
            jnum(o.y),
            jnum(o.z),
            jnum(v.x),
            jnum(v.y),
            jnum(v.z)
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
    fn fast_goto_crossing_stops_inside_bounded_finish_corridor() {
        let traj = vec![(0.0, Vec3::new(224.0, 1440.0, 24.0), Vec3::ZERO)];
        let target = Vec3::new(224.0, 2992.0, 24.0);
        assert!(goto_crossed_finish(&traj, Vec3::new(280.0, 3008.0, 48.0), target));
        assert!(!goto_crossed_finish(&traj, Vec3::new(330.0, 3008.0, 48.0), target));
        assert!(!goto_crossed_finish(&traj, Vec3::new(224.0, 2970.0, 48.0), target));
    }

    #[test]
    fn parses_simple_verbs() {
        assert_eq!(parse_line("7 status").unwrap(), (7, ControlCmd::Status));
        assert_eq!(parse_line("8 match_start").unwrap(), (8, ControlCmd::MatchStart));
        assert_eq!(parse_line("  12   links  ").unwrap(), (12, ControlCmd::Links));
        assert_eq!(parse_line("3 hold 1").unwrap(), (3, ControlCmd::Hold { bot: 1 }));
        assert_eq!(parse_line("4 stop 2").unwrap(), (4, ControlCmd::Stop { bot: 2 }));
    }

    #[test]
    fn parses_vectors_and_links() {
        assert_eq!(
            parse_line("1 teleport 1 10 -20.5 300").unwrap(),
            (
                1,
                ControlCmd::Teleport {
                    bot: 1,
                    pos: Vec3::new(10.0, -20.5, 300.0)
                }
            )
        );
        assert_eq!(
            parse_line("2 goto 1 0 0 0").unwrap(),
            (
                2,
                ControlCmd::Goto {
                    bot: 1,
                    pos: Vec3::ZERO
                }
            )
        );
        assert_eq!(
            parse_line("9 rj 1 412").unwrap(),
            (9, ControlCmd::Rj { bot: 1, link: 412 })
        );
    }

    #[test]
    fn parses_prep_defaults_and_overrides() {
        assert_eq!(
            parse_line("1 prep 1").unwrap(),
            (
                1,
                ControlCmd::Prep {
                    bot: 1,
                    health: 100.0,
                    rockets: 10.0
                }
            )
        );
        assert_eq!(
            parse_line("1 prep 1 50 3").unwrap(),
            (
                1,
                ControlCmd::Prep {
                    bot: 1,
                    health: 50.0,
                    rockets: 3.0
                }
            )
        );
    }

    #[test]
    fn set_and_cmd_take_rest_of_line() {
        assert_eq!(
            parse_line("5 set rtx_rj_delay_bias 0.05").unwrap(),
            (
                5,
                ControlCmd::Set {
                    name: "rtx_rj_delay_bias".into(),
                    value: "0.05".into()
                }
            )
        );
        assert_eq!(
            parse_line("6 get rtx_rj_stance").unwrap(),
            (
                6,
                ControlCmd::Get {
                    name: "rtx_rj_stance".into()
                }
            )
        );
        assert_eq!(
            parse_line("8 cmd map bravado").unwrap(),
            (
                8,
                ControlCmd::Cmd {
                    raw: "map bravado".into()
                }
            )
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
