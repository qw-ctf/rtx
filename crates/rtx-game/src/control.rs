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
//! stays on that thread. The socket work is pushed to background threads that only shuttle raw wire
//! frames through `mpsc` channels — the exact shape as the navmesh build worker
//! ([`crate::nav_build`]): a listener thread accepts connections, a per-connection reader thread feeds
//! inbound request frames (tagged with a connection id) to [`ControlState::requests_rx`], and a writer
//! thread drains outbound frames to their targets — a reply to the client that asked, an event to all
//! connected clients (so the MCP bridge and the navview viewer can attach at once). Requests are
//! decoded and executed, and events emitted, entirely inside
//! [`frame_begin`]/[`frame_end`] under the frame's `&mut GameState`. No lock is ever held over game
//! state; the only shared state between threads is the raw socket and the channels.
//!
//! ## Protocol
//! Framed [msgpack] of the typed [`rtx_ctlproto`] schema (`[u32 LE len][payload]`). Inbound is a
//! [`Request`] (`id` + [`Cmd`]); outbound is a [`Msg`] — a `Reply { id, Result<Resp, String> }`
//! correlated to the request, or an async [`Event`] (`arrived` / `goto_stall` / `rj_result` /
//! `fly_result`). A single outbound channel gives total ordering; the client demuxes on the `Msg`
//! variant. The MCP re-serialises the typed values as JSON for Claude.
//!
//! [msgpack]: https://msgpack.org/

use std::collections::HashMap;
use std::io::Write;
use std::net::{Ipv4Addr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};

use glam::{Vec3, Vec3Swizzles};
use rtx_ctlproto::{self as proto, Cmd, Event, Msg, Request, Resp};

use crate::bot::goals::is_goal_classname;
use crate::bot::state::{ControlOrder, HookState, RjOutcome, RjState, RjTelemetry};
use crate::defs::{Bits, Flags, Items, Solid, Weapon};
use crate::entity::EntId;
use crate::game::{cstring, GameState, MAX_EDICTS};
use crate::math::wrap180;
use crate::navmesh::{LinkKind, NavGraph};

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
/// Altitude gain that counts as goto progress (resets the stall clock) — a spiral climbs toward a
/// target above while its XY distance plateaus. Mirrors the bot's own route watchdog `CLIMB_EPS`.
const GOTO_CLIMB_EPS: f32 = 8.0;
/// A FlyLink attempt gives up after this long with no touchdown (see `poll_fly`).
const FLY_TIMEOUT: f32 = 8.0;

/// The control channel's live state, carried on [`GameState`].
///
/// **This does not survive a map change.** The engine unloads and reloads the game module on every
/// level change, which resets the module's statics and builds a fresh [`GameState`] — so `started`
/// comes back `false` and the listener binds again. What does *not* reset is the threads the previous
/// image spawned: an orphaned accept loop keeps winning connections and posting them to a channel
/// whose receiver died with the old state, so every client is accepted and then never answered, for
/// good. [`shutdown`] is what prevents that, by taking the sockets and threads down while the code
/// they run is still mapped.
#[derive(Default)]
pub(crate) struct ControlState {
    /// Whether the listener has been (attempted to be) bound. Set once, so a bind is tried at most once.
    started: bool,
    /// Inbound raw request frames tagged with the connection id they arrived on (decoded and drained
    /// each frame in [`frame_begin`]). Kept as bytes, not decoded [`Request`]s, so a malformed frame is
    /// answered with an error reply on the engine thread rather than silently dropped by the reader
    /// thread. The connection id routes the reply back to the client that asked.
    requests_rx: Option<Receiver<(u64, Vec<u8>)>>,
    /// Outbound encoded [`Msg`] frames plus their delivery target. The writer thread owns the receiving
    /// half and the client table.
    out_tx: Option<Sender<(Target, Vec<u8>)>>,
    /// Frames queued on `out_tx` that the writer has not yet picked up. Events stop being queued once
    /// this passes [`EVENT_BACKLOG_MAX`] — one wedged client must not grow the queue without bound.
    /// Replies are always queued: a request/response pair is bounded by the requester itself.
    out_backlog: Arc<AtomicUsize>,
    /// Raised by [`shutdown`] to tell the accept loop to stop.
    stop: Option<Arc<AtomicBool>>,
    /// The port the listener bound, so [`shutdown`] can poke a blocked `accept` awake.
    port: u16,
    /// The live client table, shared with the writer thread — [`shutdown`] closes these sockets so the
    /// reader threads fall out of `read_frame`.
    clients: Option<Arc<Mutex<HashMap<u64, TcpStream>>>>,
    /// Accept and writer loops, joined by [`shutdown`] before the module image goes away.
    threads: Vec<std::thread::JoinHandle<()>>,
    /// One clone per live reader thread; [`shutdown`] waits for the count to fall back to its own.
    readers: Option<Arc<()>>,
}

/// Take the control channel down and wait for its threads to leave our code — called from
/// `GAME_SHUTDOWN`, just before the engine unloads the module on a level change.
///
/// Skipping this is what breaks the channel permanently after a `map` command: the replacement image
/// binds a second listener while the previous one is still queued on the same port, and connections
/// that land on the stale one are never served. Joining here also keeps the threads from running code
/// that has been unmapped underneath them.
pub(crate) fn shutdown(game: &mut GameState) {
    let Some(stop) = game.control.stop.take() else {
        return; // never started
    };
    stop.store(true, Ordering::SeqCst);
    // Close every client socket so the reader threads leave their blocking `read_frame`.
    if let Some(clients) = game.control.clients.take() {
        if let Ok(m) = clients.lock() {
            for s in m.values() {
                let _ = s.shutdown(std::net::Shutdown::Both);
            }
        }
    }
    // Dropping these ends the writer loop (`recv` fails) and makes the readers' sends fail.
    game.control.out_tx = None;
    game.control.requests_rx = None;
    // `accept` blocks until a connection arrives, so knock on the door to let it see `stop`.
    let _ = TcpStream::connect((Ipv4Addr::LOCALHOST, game.control.port));
    for h in std::mem::take(&mut game.control.threads) {
        let _ = h.join();
    }
    // Reader threads aren't individually tracked; wait for the last clone of the token to drop.
    if let Some(token) = game.control.readers.take() {
        for _ in 0..200 {
            if Arc::strong_count(&token) == 1 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }
    game.control.started = false;
}

/// Where an outbound frame goes: a reply to the one client that asked, or an event broadcast to all.
enum Target {
    One(u64),
    All,
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
    let frames: Vec<(u64, Vec<u8>)> = match game.control.requests_rx.as_ref() {
        Some(rx) => rx.try_iter().collect(),
        None => return,
    };
    for (conn, frame) in frames {
        match proto::decode::<Request>(&frame) {
            Ok(req) => exec_request(game, conn, req),
            Err(e) => reply(game, conn, 0, Err(format!("bad request frame: {e}"))),
        }
    }
}

/// Frame epilogue: observe every puppeted bot and emit the lifecycle events its order produced this
/// frame (arrival / stall for a goto, the terminal telemetry for a rocket jump). Runs after
/// `run_bots` so it sees the post-frame bot state the driver just wrote.
pub(crate) fn frame_end(game: &mut GameState) {
    if game.control.out_tx.is_none() {
        return; // channel never came up — nothing to emit
    }
    let now = game.time();
    let maxclients = game.host.cvar(c"maxclients").max(0.0) as u32;
    // One switch for the whole new observability surface (`Pmove`, `BotStall`, the client heartbeat):
    // pre-branch typed consumers cannot decode an unknown enum variant — the deployed nav viewer
    // treats that as a dead connection — so nothing new goes on the wire until the operator opts
    // this server in with `rtx_telemetry 1`.
    let telemetry = game.host.cvar(c"rtx_telemetry") > 0.0;
    if telemetry {
        send_event(game, Event::Pmove(pmove_event(game, now, maxclients)));
    }
    for i in 1..=maxclients {
        let e = EntId(i);
        if !game.entities[e].bot.is_bot || !game.entities[e].in_use {
            continue;
        }
        // Steering-watchdog firings the bot parked this frame (see `bot::state::StallRecord`).
        // Drained ahead of the puppet-order match below, and for *every* bot: a stall is a fact
        // about autonomous play, which by definition runs without an order. `pop_front` — not
        // `mem::take` — so the deque keeps its capacity and the watchdog path never re-allocates.
        // Behind the telemetry switch like everything else; undrained records self-bound at
        // [`crate::bot::state::STALL_BUF_CAP`].
        while telemetry {
            let Some(rec) = game.entities[e].bot.stall_events.pop_front() else {
                break;
            };
            send_event(
                game,
                Event::BotStall {
                    bot: i,
                    t: rec.t,
                    reason: rec.reason.to_string(),
                    origin: a3(rec.origin),
                    cell: rec.cell,
                    goal_cell: rec.goal_cell,
                    goal_dist: rec.goal_dist,
                    link: rec.link.unwrap_or(u32::MAX),
                    kind: rec.kind.map(|k| format!("{k:?}")).unwrap_or_default(),
                    speed: rec.speed,
                    route_len: rec.route_len,
                    route_pos: rec.route_pos,
                    action: rec.action.to_string(),
                },
            );
        }
        match game.entities[e].bot.puppet.order {
            None | Some(ControlOrder::Hold) => {}
            Some(ControlOrder::Goto { target }) => {
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let phase = game.entities[e].bot.bhop.phase as u8;
                let traj = &mut game.entities[e].bot.puppet.traj;
                // Long flat-corridor benchmarks need roughly 7–10 seconds to expose the 800+ ups
                // regime. Keep their complete velocity trace; the old 400-row cap truncated the
                // final acceleration and made the reported peak systematically too low.
                if traj.len() < 1200 {
                    traj.push((now, origin, vel, phase));
                }
                poll_goto(game, e, i, target, now);
            }
            Some(ControlOrder::RocketJump { link }) => {
                // Trace the flight: sample this frame's post-move origin/velocity before checking for a
                // result, so the trajectory in `rj_result` runs from the stance through the landing.
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let phase = game.entities[e].bot.bhop.phase as u8;
                let traj = &mut game.entities[e].bot.puppet.traj;
                if traj.len() < 400 {
                    traj.push((now, origin, vel, phase));
                }
                poll_rj(game, e, i, link, now);
            }
            Some(ControlOrder::FlyLink { link }) => {
                let (origin, vel) = (game.entities[e].v.origin, game.entities[e].v.velocity);
                let phase = game.entities[e].bot.bhop.phase as u8;
                let traj = &mut game.entities[e].bot.puppet.traj;
                if traj.len() < 400 {
                    traj.push((now, origin, vel, phase));
                }
                poll_fly(game, e, i, link, now);
            }
        }
    }
}

/// How long a single outbound write may stall before the client is treated as gone. See where it's
/// applied in [`listener_loop`].
const WRITE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(15);

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
    let stop = Arc::new(AtomicBool::new(false));
    let readers = Arc::new(());
    let (requests_tx, requests_rx) = std::sync::mpsc::channel::<(u64, Vec<u8>)>();
    let (out_tx, out_rx) = std::sync::mpsc::channel::<(Target, Vec<u8>)>();
    // The live client table (write halves keyed by connection id), shared by the writer thread and the
    // per-connection reader threads. Multiple clients attach at once — e.g. the MCP bridge and the
    // navview viewer — with replies routed by id and events broadcast to all.
    let clients: Arc<Mutex<HashMap<u64, TcpStream>>> = Arc::new(Mutex::new(HashMap::new()));
    let wclients = clients.clone();
    let backlog = Arc::new(AtomicUsize::new(0));
    let wbacklog = backlog.clone();
    let writer = std::thread::spawn(move || writer_loop(out_rx, wclients, wbacklog));
    let lstop = stop.clone();
    let lreaders = readers.clone();
    let lclients = clients.clone();
    let accept = std::thread::spawn(move || listener_loop(listener, requests_tx, lclients, lstop, lreaders));
    game.control.requests_rx = Some(requests_rx);
    game.control.out_tx = Some(out_tx);
    game.control.out_backlog = backlog;
    game.control.stop = Some(stop);
    game.control.port = port;
    game.control.clients = Some(clients);
    game.control.threads = vec![accept, writer];
    game.control.readers = Some(readers);
    game.host
        .conprint(&cstring(&format!("rtx: control: listening on 127.0.0.1:{port}\n")));
}

/// Accept loop: each connection gets a unique id, a write half in the shared client table, and its own
/// reader thread tagging inbound frames with that id. The reader removes the client on disconnect.
fn listener_loop(
    listener: TcpListener,
    requests_tx: Sender<(u64, Vec<u8>)>,
    clients: Arc<Mutex<HashMap<u64, TcpStream>>>,
    stop: Arc<AtomicBool>,
    readers: Arc<()>,
) {
    let mut next_id: u64 = 0;
    for stream in listener.incoming().flatten() {
        if stop.load(Ordering::SeqCst) {
            break; // shutting down: drop the listener so the port is free for the next module image
        }
        let _ = stream.set_nodelay(true);
        let id = next_id;
        next_id += 1;
        if let Ok(wr) = stream.try_clone() {
            // A client that stops draining must not stall the writer indefinitely: past this, the
            // write fails and the connection is dropped, which is the recoverable outcome. Generous,
            // because a legitimately slow reader taking a multi-megabyte BSP is not a dead one.
            let _ = wr.set_write_timeout(Some(WRITE_TIMEOUT));
            if let Ok(mut m) = clients.lock() {
                m.insert(id, wr);
            }
        }
        let tx = requests_tx.clone();
        let clients = clients.clone();
        let alive = readers.clone(); // dropped when this reader returns; `shutdown` waits on the count
        std::thread::spawn(move || {
            let _alive = alive;
            let mut stream = stream;
            loop {
                match proto::read_frame(&mut stream) {
                    Ok(Some(frame)) => {
                        if tx.send((id, frame)).is_err() {
                            break; // game side gone
                        }
                    }
                    Ok(None) | Err(_) => break, // clean EOF or connection dropped
                }
            }
            if let Ok(mut m) = clients.lock() {
                m.remove(&id); // drop this client's write half
            }
        });
    }
}

/// Writer loop: drain outbound msgpack frames to their targets. A reply goes to the one client that
/// asked; an event broadcasts to every connected client. A write error drops that client from the
/// table (a reconnecting client resyncs via `status`).
fn writer_loop(
    out_rx: Receiver<(Target, Vec<u8>)>,
    clients: Arc<Mutex<HashMap<u64, TcpStream>>>,
    backlog: Arc<AtomicUsize>,
) {
    while let Ok((target, frame)) = out_rx.recv() {
        backlog.fetch_sub(1, Ordering::Relaxed);
        // Pick the recipients under the lock, then **release it before writing**. `write_all` blocks
        // once the peer stops draining — a multi-megabyte BSP overruns a socket buffer several times
        // over — and holding the table across that wedges the whole channel: `listener_loop` needs
        // the same lock to register a connection, so even a freshly restarted client would sit there
        // accepted but never served, looking for all the world like the server had died.
        let targets: Vec<(u64, TcpStream)> = {
            let Ok(m) = clients.lock() else { continue };
            match target {
                Target::One(id) => m
                    .get(&id)
                    .and_then(|s| s.try_clone().ok())
                    .map(|s| (id, s))
                    .into_iter()
                    .collect(),
                // `try_clone` dups the handle, so the clone writes to the same connection.
                Target::All => m
                    .iter()
                    .filter_map(|(&id, s)| Some((id, s.try_clone().ok()?)))
                    .collect(),
            }
        };
        let mut dead = Vec::new();
        for (id, mut stream) in targets {
            if stream.write_all(&frame).and_then(|()| stream.flush()).is_err() {
                dead.push(id);
            }
        }
        if !dead.is_empty() {
            if let Ok(mut m) = clients.lock() {
                for id in dead {
                    m.remove(&id);
                }
            }
        }
    }
}

/// Queue one outbound [`Msg`] to a target, encoded to a wire frame. A no-op when the channel is down.
fn send_to(game: &GameState, target: Target, msg: Msg) {
    if let Some(tx) = game.control.out_tx.as_ref() {
        game.control.out_backlog.fetch_add(1, Ordering::Relaxed);
        let _ = tx.send((target, proto::to_frame(&msg)));
    }
}

/// Ceiling on queued-but-unwritten frames before *events* stop being queued (replies always are).
/// ~6 s of per-frame telemetry at 77 Hz — far above any healthy writer's backlog, small enough that
/// a wedged client caps the queue at a few hundred kilobytes instead of the whole heap.
const EVENT_BACKLOG_MAX: usize = 512;

/// Queue one async lifecycle [`Event`] to every connected client — unless the writer is drowning
/// (see [`EVENT_BACKLOG_MAX`]). An event is a broadcast fact about a moment already gone; stale
/// facts are droppable in a way replies never are.
fn send_event(game: &GameState, ev: Event) {
    if game.control.out_backlog.load(Ordering::Relaxed) >= EVENT_BACKLOG_MAX {
        return;
    }
    send_to(game, Target::All, Msg::Event(ev));
}

/// Broadcast an event raised outside the frame hooks — the client's per-seat heartbeat (see
/// `crate::netclient`). Silently drops it when the channel never came up, the same way
/// [`frame_end`] declines to do any work at all in that case.
///
/// Gated on the feature that owns its only caller: a `qwprogs` build has no seats to report.
#[cfg(feature = "netclient")]
pub(crate) fn emit_event(game: &GameState, ev: Event) {
    if game.control.out_tx.is_none() {
        return;
    }
    send_event(game, ev);
}

/// Whether a cvar name is safe to splice into a `set` localcmd (guards the console tokenizer): a
/// non-empty run of `[A-Za-z0-9_]`. rtx cvars are all of this form.
fn valid_cvar_name(name: &str) -> bool {
    !name.is_empty() && name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
}

// --- command execution (engine thread, &mut GameState) ---

/// A wire position (`[x, y, z]`) as a `glam::Vec3`.
fn v3(a: proto::Vec3) -> Vec3 {
    Vec3::from_array(a)
}

/// A `glam::Vec3` as a wire position (`[x, y, z]`).
fn a3(v: Vec3) -> proto::Vec3 {
    v.to_array()
}

/// Execute one decoded request on the engine thread and send its typed reply back to connection `conn`.
fn exec_request(game: &mut GameState, conn: u64, req: Request) {
    let Request { id, cmd } = req;
    let result: Result<Resp, String> = match cmd {
        Cmd::Status => Ok(Resp::Status(Box::new(status_resp(game)))),
        // Report a refusal rather than acknowledging one. A caller that cannot tell "accepted" from
        // "one bot short" has no choice but to poll until a timeout, which is exactly what the MCP
        // was doing for ninety seconds a call.
        Cmd::MatchStart => crate::mode::team::start_match(game).map(|()| Resp::Queued),
        Cmd::Links => links_resp(game).map(Resp::Links),
        Cmd::Items => items_resp(game).map(Resp::Items),
        Cmd::Prep { bot, health, rockets } => do_prep(game, bot, health, rockets),
        Cmd::Teleport { bot, pos, vel } => do_teleport(game, bot, v3(pos), v3(vel)),
        Cmd::Goto { bot, pos } => do_goto(game, bot, v3(pos)),
        Cmd::Rj { bot, link } => do_rj(game, bot, link),
        Cmd::Fly { bot, link } => do_fly(game, bot, link),
        Cmd::Hold { bot } => do_order(game, bot, ControlOrder::Hold),
        Cmd::Stop { bot } => do_stop(game, bot),
        Cmd::Set { name, value } => do_set(game, &name, &value),
        Cmd::Get { name } => do_get(game, &name),
        Cmd::RunCmd { raw } => {
            game.host.localcmd(&raw);
            Ok(Resp::Queued)
        }
        Cmd::Cell { pos } => cell_resp(game, v3(pos)).map(Resp::Cell),
        Cmd::CellById { cell } => cell_by_id_resp(game, cell).map(Resp::Cell),
        Cmd::Route { bot } => route_resp(game, bot).map(Resp::Route),
        Cmd::Audit { bot, lines } => audit_resp(game, bot, lines as usize).map(Resp::Audit),
        Cmd::Curls => curls_resp(game).map(Resp::Curls),
        Cmd::Bsp => bsp_resp(game).map(|b| Resp::Bsp(Box::new(b))),
        Cmd::Maps => Ok(Resp::Maps(maps_resp(game))),
        Cmd::Probe {
            takeoff,
            tgt,
            psi0,
            runway,
        } => probe_resp(game, v3(takeoff), v3(tgt), psi0, runway).map(Resp::Probe),
        Cmd::Curl { src, tgt } => curl_resp(game, v3(src), v3(tgt)).map(Resp::Curl),
        Cmd::PlanLink {
            from,
            takeoff,
            tgt,
            v_req,
            gain,
        } => plant_link_resp(game, v3(from), v3(takeoff), v3(tgt), v_req, gain).map(Resp::PlanLink),
        Cmd::PlanCell { pos } => plant_cell_resp(game, v3(pos)).map(Resp::PlanCell),
        Cmd::PlanDrop { from, to } => plant_drop_resp(game, v3(from), v3(to)).map(Resp::PlanDrop),
    };
    reply(game, conn, id, result);
}

/// Send the typed reply for request `id` back to the connection that issued it.
fn reply(game: &GameState, conn: u64, id: i64, result: Result<Resp, String>) {
    send_to(game, Target::One(conn), Msg::Reply { id, result });
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
fn do_prep(game: &mut GameState, bot: u32, health: f32, rockets: f32) -> Result<Resp, String> {
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
    Ok(Resp::Prep { bot, health, rockets })
}

/// Place a bot at `pos` (feet on the ground it names) carrying `vel`, and reset all navigation
/// commitments so nothing stale (a mid-flight route/jump) survives the jump. `+1z` avoids startsolid.
///
/// `vel` is normally zero — a rocket-jump test has to start from a standstill or its launch
/// measurement is contaminated. It is non-zero when the caller is reproducing a starting *condition*
/// rather than just a position, which is what scoring against a human demo line needs: the human
/// entered that movement at speed, and a bot dropped there at rest is measured on its standing start
/// instead of on the movement.
fn do_teleport(game: &mut GameState, bot: u32, pos: Vec3, vel: Vec3) -> Result<Resp, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let at = pos + Vec3::new(0.0, 0.0, 1.0);
    game.entities[e].v.velocity = vel;
    game.set_origin(e, at);
    reset_nav_state(&mut game.entities[e].bot, at, now);
    // Park the bot after placing it — otherwise, with no order, it would roam autonomously and arrive
    // at a subsequent rocket jump with residual velocity, contaminating the standstill measurement.
    game.entities[e].bot.puppet.order = Some(ControlOrder::Hold);
    Ok(Resp::Teleport {
        bot,
        origin: a3(game.entities[e].v.origin),
    })
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
    bot.walk = None; // the certified line was proven over route the bot is no longer on
    bot.bhop = Default::default();
    bot.watchdog.last_origin = at;
    bot.watchdog.stuck_origin = at;
    bot.watchdog.stuck_since = now;
    bot.repath_time = now;
}

fn do_goto(game: &mut GameState, bot: u32, pos: Vec3) -> Result<Resp, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let start = game.entities[e].v.origin;
    let b = &mut game.entities[e].bot;
    b.rj = RjState::default();
    b.route.clear();
    b.repath_time = now;
    b.puppet.traj.clear();
    b.puppet.order = Some(ControlOrder::Goto { target: pos });
    b.puppet.best_dist = f32::INFINITY;
    b.puppet.best_z = f32::NEG_INFINITY;
    b.puppet.best_since = now;
    b.puppet.anchor = start;
    Ok(Resp::Goto { bot, target: a3(pos) })
}

fn do_rj(game: &mut GameState, bot: u32, link: u32) -> Result<Resp, String> {
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
    Ok(Resp::Rj { bot, link })
}

fn do_fly(game: &mut GameState, bot: u32, link: u32) -> Result<Resp, String> {
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
    Ok(Resp::Fly { bot, link })
}

fn do_order(game: &mut GameState, bot: u32, order: ControlOrder) -> Result<Resp, String> {
    let e = valid_bot(game, bot)?;
    if order == ControlOrder::Hold {
        let at = game.entities[e].v.origin;
        let now = game.time();
        game.entities[e].v.velocity = Vec3::ZERO;
        reset_nav_state(&mut game.entities[e].bot, at, now);
    }
    game.entities[e].bot.puppet.order = Some(order);
    Ok(Resp::Ack { bot })
}

fn do_stop(game: &mut GameState, bot: u32) -> Result<Resp, String> {
    let e = valid_bot(game, bot)?;
    let now = game.time();
    let b = &mut game.entities[e].bot;
    b.puppet.order = None;
    b.rj = RjState::default();
    b.route.clear();
    b.repath_time = now;
    Ok(Resp::Ack { bot })
}

fn do_set(game: &mut GameState, name: &str, value: &str) -> Result<Resp, String> {
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
    Ok(Resp::Set {
        name: name.to_string(),
        value: value.to_string(),
    })
}

fn do_get(game: &mut GameState, name: &str) -> Result<Resp, String> {
    if !valid_cvar_name(name) {
        return Err(format!("bad cvar name '{name}'"));
    }
    let cname = cstring(name);
    let mut buf = [0u8; 128];
    let s = game.host.cvar_string(&cname, &mut buf).to_string();
    let f = game.host.cvar(&cname);
    Ok(Resp::Get {
        name: name.to_string(),
        string: s,
        value: f,
    })
}

// --- status / links snapshots ---

use crate::mode::match_phase_name;

/// A compact reference to a live entity carried by strategy telemetry. Item goals need the
/// classname + location; enemy/teammate references also benefit from the display name. Keeping the
/// reference nullable makes the `0` sentinel explicit to MCP clients instead of exposing a fake
/// world entity.
fn ent_ref(game: &GameState, id: u32) -> Option<proto::EntRef> {
    if id == 0 {
        return None;
    }
    let ent = game.entities.get(id as usize).filter(|e| e.in_use)?;
    Some(proto::EntRef {
        ent: id,
        name: game.netname_of(EntId(id)),
        classname: ent.classname().unwrap_or("").to_string(),
        origin: a3(ent.v.origin),
        solid: format!("{:?}", ent.v.solid),
    })
}

fn route_head(game: &GameState, e: EntId) -> proto::RouteHead {
    let b = &game.entities[e].bot;
    let pos = b.route_pos as u32;
    let len = b.route.len() as u32;
    let next = game.nav.graph.as_ref().and_then(|g| {
        b.route.get(b.route_pos).map(|&link| proto::RouteNext {
            link,
            kind: kind_name(g.link_kind(link)).to_string(),
            target: a3(g.cell_origin(g.link_target(link))),
        })
    });
    proto::RouteHead { pos, len, next }
}

fn match_info(game: &GameState) -> proto::MatchInfo {
    let cfg = game.team_match.config;
    let mut scores = Vec::with_capacity(cfg.teams);
    for team in 1..=cfg.teams {
        let score = game
            .entities
            .iter()
            .filter(|e| e.is_player() && e.in_use && e.mode_p.team as usize == team)
            .map(|e| e.v.frags as i32)
            .sum::<i32>();
        scores.push(score);
    }
    let roster = game
        .team_match
        .roster
        .iter()
        .map(|(name, team)| proto::RosterEntry {
            name: name.clone(),
            team: *team as u32,
        })
        .collect();
    proto::MatchInfo {
        mode: game.mode.name().to_string(),
        format: crate::mode::team::format_label(cfg),
        phase: match_phase_name(game.team_match.phase).to_string(),
        teams: cfg.teams as u32,
        size: cfg.size as u32,
        teamplay: game.level.teamplay as i32,
        timelimit: game.level.timelimit as f32,
        fraglimit: game.level.fraglimit as f32,
        live_until: game.team_match.live_until,
        scores,
        roster,
    }
}

/// One frame of authoritative human movement for [`Event::Pmove`].
///
/// Bots are excluded: their state already travels in `status.bots` every frame a harness asks
/// for it, and the lab is about what a *person* did. An empty player list is still emitted by
/// the caller — see the event's own documentation for why the heartbeat matters.
fn pmove_event(game: &GameState, now: f32, maxclients: u32) -> proto::PmoveEvent {
    let mut players = Vec::new();
    for i in 1..=maxclients {
        let ent = &game.entities[EntId(i)];
        if !ent.in_use || !ent.is_player() || ent.bot.is_bot {
            continue;
        }
        // groundentity is a raw prog reference: only trust it if it round-trips to a real edict.
        let ground_ent = if ent.v.groundentity < 0 {
            -1
        } else {
            let ground = EntId::from_prog(ent.v.groundentity);
            if ground.index() < game.entities.len() && ground.to_prog() == ent.v.groundentity {
                ground.0 as i32
            } else {
                -1
            }
        };
        players.push(proto::PmovePlayer {
            ent: i,
            origin: a3(ent.v.origin),
            vel: a3(ent.v.velocity),
            on_ground: ent.v.flags.has(Flags::ONGROUND),
            ground_ent,
        });
    }
    proto::PmoveEvent { t: now, players }
}

fn status_resp(game: &GameState) -> proto::StatusResp {
    let (navmesh, cells, links, rj_links) = match game.nav.graph.as_ref() {
        Some(g) => (
            "ready",
            g.cells.len() as u32,
            g.links.len() as u32,
            g.summary().rocket_jump as u32,
        ),
        None if game.nav.pending.is_some() => ("building", 0, 0, 0),
        None => ("none", 0, 0, 0),
    };
    // Scan the full QW client-entity range, not just `maxclients`: as a net
    // client, the local cvar mirrors the *advertised* maxclients (KTX caps it
    // with k_maxclients), while our own seats can sit on higher slots when
    // earlier connections hold the low ones — a bot on such a slot would
    // silently vanish from Status. The is_bot/in_use filter already skips
    // whatever else lives in 1..=32.
    let maxclients = (game.host.cvar(c"maxclients").max(0.0) as u32).max(32);
    let mut bots = Vec::new();
    for i in 1..=maxclients {
        let ent = &game.entities[EntId(i)];
        if !ent.bot.is_bot || !ent.in_use {
            continue;
        }
        let b = &ent.bot;
        bots.push(proto::BotStatus {
            ent: i,
            client: b.client,
            name: game.netname_of(EntId(i)),
            team: ent.mode_p.team as i32,
            team_name: game.team_of(EntId(i)),
            frags: ent.v.frags as i32,
            origin: a3(ent.v.origin),
            health: ent.v.health,
            armor: ent.v.armorvalue,
            armor_type: ent.v.armortype,
            weapon: format!("{:?}", ent.v.weapon),
            items: format!("{:?}", Items::from_f32(ent.v.items)),
            ammo: proto::Ammo {
                shells: ent.v.ammo_shells as i32,
                nails: ent.v.ammo_nails as i32,
                rockets: ent.v.ammo_rockets as i32,
                cells: ent.v.ammo_cells as i32,
            },
            on_ground: ent.v.flags.has(Flags::ONGROUND),
            alive: ent.is_alive(),
            order: order_name(b.puppet.order).to_string(),
            posture: format!("{:?}", b.posture),
            known_enemy: ent_ref(game, b.percept.known_enemy),
            goal: proto::BotGoal {
                switches: b.goal.switches,
                item: ent_ref(game, b.goal.item),
                commit: format!("{:?}", b.goal.commit),
                since: b.goal.since,
                next_item: ent_ref(game, b.goal.next_item),
                hold_item: ent_ref(game, b.goal.hold_item),
                hold_for: ent_ref(game, b.goal.hold_for),
            },
            route: route_head(game, EntId(i)),
            rj_phase: format!("{:?}", b.rj.phase),
            speed: ent.v.velocity.xy().length(),
            bhop: format!("{:?}", b.bhop.phase),
            bhop_peak: b.bhop.peak,
            packs: proto::PackStats {
                sg_swaps: b.packs.sg_swaps,
                hinted: b.packs.hinted,
                secured: b.packs.secured,
                fed: b.packs.fed,
            },
        });
    }
    // Human clients, for movement-lab monitoring. A separate array from `bots` on purpose:
    // every existing consumer that iterates `bots` keeps its bots-only contract.
    let mut players = Vec::new();
    for i in 1..=maxclients {
        let ent = &game.entities[EntId(i)];
        if ent.bot.is_bot || !ent.in_use || !ent.is_player() {
            continue;
        }
        players.push(proto::PlayerStatus {
            ent: i,
            name: game.netname_of(EntId(i)),
            origin: a3(ent.v.origin),
            health: ent.v.health,
            on_ground: ent.v.flags.has(Flags::ONGROUND),
            alive: ent.is_alive(),
            speed: ent.v.velocity.xy().length(),
        });
    }
    proto::StatusResp {
        map: game.level.mapname.clone(),
        time: game.time(),
        navmesh: navmesh.to_string(),
        cells,
        links,
        rj_links,
        match_: match_info(game),
        oracle: oracle_info(game),
        bots,
        players,
    }
}

/// Map an oracle [`crate::bot::oracle::EvalSummary`] to the wire counts (identical fields).
fn eval_counts(s: crate::bot::oracle::EvalSummary) -> proto::EvalCounts {
    proto::EvalCounts {
        treated: s.treated,
        treated_success: s.treated_success,
        controls: s.controls,
        control_success: s.control_success,
        applied: s.applied,
        invalidated: s.invalidated,
        pending: s.pending,
    }
}

fn oracle_info(game: &GameState) -> proto::OracleInfo {
    let mut by_kind = Vec::new();
    let mut ep_by_kind = Vec::new();
    for kind in crate::bot::oracle::NUGGET_KINDS {
        let label = format!("{:?}", kind);
        by_kind.push((label.clone(), eval_counts(game.oracle.eval_summary_for(kind))));
        ep_by_kind.push((label, eval_counts(game.oracle.eval_episode_summary_for(kind))));
    }
    let eval = proto::Eval {
        counts: eval_counts(game.oracle.eval_summary()),
        by_kind,
        episodes: proto::EpisodeEval {
            counts: eval_counts(game.oracle.eval_episode_summary()),
            by_kind: ep_by_kind,
        },
    };
    let comms = game.oracle.communication_summary();
    let communication = proto::Communication {
        proposed: comms.proposed,
        communicated: comms.communicated,
        refreshed: comms.refreshed,
        suppressed: comms.suppressed,
        superseded: comms.superseded,
        arm_clears: comms.arm_clears,
    };
    let plan = game.oracle.last_plan().map(|plan| proto::Plan {
        generation: plan.generation as u64,
        at: plan.at,
        teams: plan
            .teams
            .iter()
            .map(|team| proto::PlanTeam {
                team: team.team as u32,
                mode: format!("{:?}", team.mode),
                control: format!("{:?}", team.control),
                power_gap: team.power_gap,
                nuggets: team
                    .nuggets
                    .iter()
                    .map(|n| proto::Nugget {
                        recipient: n.recipient as i32,
                        kind: format!("{:?}", n.kind),
                        target_cell: n.target_cell as u32,
                        subject: n.subject as i32,
                        confidence: n.confidence,
                        decision_at: n.decision_at,
                        evidence_at: n.evidence_at,
                        expires_at: n.expires_at,
                    })
                    .collect(),
            })
            .collect(),
    });
    proto::OracleInfo {
        running: game.oracle.running(),
        epoch: game.oracle.epoch() as u64,
        last_output: game.oracle.last_output(),
        plan,
        communication,
        eval,
    }
}

fn links_resp(game: &GameState) -> Result<Vec<proto::RjLink>, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let mut links = Vec::new();
    for li in 0..g.links.len() as u32 {
        if g.link_kind(li) != LinkKind::RocketJump {
            continue;
        }
        let Some(tr) = g.rocket_jump_of_link(li) else { continue };
        links.push(proto::RjLink {
            link: li,
            src: a3(g.cell_origin(g.link_source(li))),
            tgt: a3(g.cell_origin(g.link_target(li))),
            fire_pitch: tr.fire_angles.x,
            fire_yaw: tr.fire_angles.y,
            fire_delay: tr.fire_delay,
            airtime: tr.airtime,
            self_damage: tr.self_damage,
            v0: a3(tr.v0),
            blast: a3(tr.blast),
            pos_blast: a3(tr.pos_blast),
            land: a3(tr.land),
        });
    }
    Ok(links)
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
        LinkKind::Swim => "swim",
    }
}

/// Inspect the navmesh cell nearest `pos`: its origin plus every link leaving and entering it (index,
/// kind, other endpoint). The diagnostic for "why can't the bot reach here" — an unreachable ledge
/// has no incoming jump/speed-jump link.
fn cell_resp(game: &GameState, pos: Vec3) -> Result<proto::CellResp, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let cell = g.nearest(pos).ok_or("no navmesh cell near that point")?;
    Ok(describe_cell(g, cell))
}

/// [`cell_resp`] by cell id instead of by point — the direction a route, a link listing or an earlier
/// reply hands you, none of which give coordinates you could feed back to `nearest`.
fn cell_by_id_resp(game: &GameState, cell: u32) -> Result<proto::CellResp, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    if cell as usize >= g.cells.len() {
        return Err(format!("no such cell {cell} (the mesh has {})", g.cells.len()));
    }
    Ok(describe_cell(g, cell))
}

/// A cell's origin, hazard and both link directions.
fn describe_cell(g: &NavGraph, cell: u32) -> proto::CellResp {
    let mut out = Vec::new();
    let mut incoming = Vec::new();
    // Outgoing comes from the adjacency, not a scan of every link: a link can exist in the array and
    // still be untraversable (a teleport trigger's cell keeps only its teleport exit — walking out of
    // one is not a thing a player can do). Reporting the array would show exits nothing can take.
    for &li in &g.adjacency[cell as usize] {
        // `cost` is the static travel time only. What a hazard link *really* costs the planner is
        // `hazard_hp` valued against the asking bot's strength, so report the health and let the
        // caller price it — reporting seconds here would mean picking a bot to price it for.
        out.push(proto::CellLinkOut {
            link: li,
            kind: kind_name(g.link_kind(li)).to_string(),
            to_cell: g.link_target(li),
            to: a3(g.cell_origin(g.link_target(li))),
            cost: g.link_cost(li),
            tgt_hazard: format!("{:?}", g.cell_hazard(g.link_target(li))),
            hazard_hp: g.link_hazard_hp(li),
            water_extra: g.link_water_extra(li),
        });
    }
    for li in 0..g.links.len() as u32 {
        if g.link_target(li) == cell {
            incoming.push(proto::CellLinkIn {
                link: li,
                kind: kind_name(g.link_kind(li)).to_string(),
                from_cell: g.link_source(li),
                from: a3(g.cell_origin(g.link_source(li))),
            });
        }
    }
    proto::CellResp {
        cell,
        origin: a3(g.cell_origin(cell)),
        hazard: format!("{:?}", g.cell_hazard(cell)),
        ledge: g.is_ledge(cell),
        out,
        incoming,
    }
}

/// List the map's bot-goal items (armor, health, weapons, ammo, powerups), so a caller can find a
/// pickup without spelunking the bsp entity lump. Each item reports its entity origin, whether it's
/// currently on the floor to be taken (`available`), and the nearest navmesh cell — the standable
/// point to `goto`, since the entity origin itself floats above the floor and isn't a nav cell.
fn items_resp(game: &GameState) -> Result<Vec<proto::ItemInfo>, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let mut out = Vec::new();
    for (id, ent) in game.entities.live() {
        let Some(classname) = ent.classname() else {
            continue;
        };
        if !is_goal_classname(classname) {
            continue;
        }
        let nav = g.nearest(ent.v.origin).map(|cell| proto::NavCell {
            cell,
            origin: a3(g.cell_origin(cell)),
        });
        out.push(proto::ItemInfo {
            ent: id.0,
            classname: classname.to_string(),
            origin: a3(ent.v.origin),
            available: ent.v.solid == Solid::Trigger,
            nav,
        });
    }
    Ok(out)
}

/// Dump a bot's current route: `route_pos` and each leg (index, kind, source→target).
fn route_resp(game: &GameState, bot: u32) -> Result<proto::RouteResp, String> {
    let e = valid_bot(game, bot)?;
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let b = &game.entities[e].bot;
    let legs = b
        .route
        .iter()
        .enumerate()
        .map(|(i, &leg)| proto::RouteLeg {
            i: i as u32,
            link: leg,
            kind: kind_name(g.link_kind(leg)).to_string(),
            src_cell: g.link_source(leg),
            tgt_cell: g.link_target(leg),
            src: a3(g.cell_origin(g.link_source(leg))),
            tgt: a3(g.cell_origin(g.link_target(leg))),
        })
        .collect();
    Ok(proto::RouteResp {
        bot,
        route_pos: b.route_pos as u32,
        origin: a3(game.entities[e].v.origin),
        legs,
    })
}

/// Dump a bot's `rtx_bot_debug` audit ring: the last `lines` per-frame sensor snapshots, oldest-first.
/// The frames are already the wire schema, so this just tails the ring. Empty when `rtx_bot_debug`
/// has been off (nothing was captured).
fn audit_resp(game: &GameState, bot: u32, lines: usize) -> Result<proto::AuditResp, String> {
    let e = valid_bot(game, bot)?;
    let frames = game.entities[e].bot.audit.tail(lines);
    Ok(proto::AuditResp {
        bot,
        count: frames.len() as u32,
        frames,
    })
}

/// Search the offline pmove sim (against the live BSP) for a speed-curl jump from `src` to `tgt`: a
/// held-strafe air-curl from a run-up-built takeoff speed. Grid-searches takeoff speed `v0`, launch
/// heading `psi0`, and turn gain, returning the lowest-speed curl that lands within tolerance — the
/// M2 solver, exercised live. Mirrors the human demo (build speed, one leap, gentle held-strafe sweep).
fn curl_resp(game: &GameState, src: Vec3, tgt: Vec3) -> Result<proto::CurlResp, String> {
    use crate::bot::bhop;
    use crate::math::{wrap180, yaw_of};
    use crate::pmove_sim::{pm_step, PmParams, PmState};
    let bsp = game.nav.bsp.as_deref().ok_or("no bsp loaded")?;
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
    Ok(match best {
        Some((v0, psi0, gain, miss, land)) => proto::CurlResp {
            found: true,
            chord,
            v0,
            psi0,
            gain,
            miss_xy: miss,
            land: a3(land),
        },
        None => proto::CurlResp {
            found: false,
            chord,
            v0: 0.0,
            psi0: 0.0,
            gain: 0.0,
            miss_xy: 0.0,
            land: [0.0; 3],
        },
    })
}

/// Hand-plant a standing cell at `pos`: index a walkable surface the column carve's XY pitch cannot
/// sample, so a bot standing there resolves to *it* instead of to whatever floor happens to be nearest
/// in 3D. Additive and inert on its own — nothing links into a planted cell unless the caller plants
/// that too — so this cannot change any route a bot already takes. Pair it with `PlanDrop` to give the
/// surface a way off.
fn plant_cell_resp(game: &mut GameState, pos: Vec3) -> Result<proto::PlanCellResp, String> {
    let bsp = game.nav.bsp.clone().ok_or("no bsp")?;
    let graph = game.nav.graph.as_mut().ok_or("navmesh not ready")?;
    let g = std::sync::Arc::get_mut(graph).ok_or("navmesh is shared with the team oracle")?;
    let (cell, links_created) = g
        .plant_cell(&bsp, pos)
        .ok_or("cell position is not standable dry floor")?;
    // Refresh reachability + LOD for the same reason `PlanLink` does: without it the O(1) `reachable`
    // gate and the coarse router keep answering for the pre-plant graph.
    g.rebuild_derived();
    Ok(proto::PlanCellResp {
        cell,
        origin: a3(g.cell_origin(cell)),
        links_created: links_created as u32,
    })
}

/// Hand-plant a `Drop` from the cell nearest `from` to the cell nearest `to`. Resolution goes through
/// `nearest`, so plant the shelf cell *first* — otherwise `from` resolves to the floor below the shelf
/// and the link is a no-op between two floor cells. The reply carries both resolved origins so the
/// caller can assert it attached where it meant to.
fn plant_drop_resp(game: &mut GameState, from: Vec3, to: Vec3) -> Result<proto::PlanDropResp, String> {
    /// Endpoint resolution is bounded, unlike bare `nearest`: a position with nothing near it must be
    /// an error, not a silent snap to whatever cell happens to be closest somewhere else on the map.
    const REACH_XY: f32 = 48.0;
    const REACH_Z: f32 = 48.0;
    let bsp = game.nav.bsp.clone().ok_or("no bsp")?;
    let graph = game.nav.graph.as_mut().ok_or("navmesh not ready")?;
    let g = std::sync::Arc::get_mut(graph).ok_or("navmesh is shared with the team oracle")?;
    let resolve = |g: &rtx_nav::navmesh::NavGraph, p: Vec3, what: &str| {
        g.cell_within(p, REACH_XY, REACH_Z)
            .ok_or_else(|| format!("no cell within {REACH_XY}/{REACH_Z} of {what} {p:?}"))
    };
    let from_cell = resolve(g, from, "from")?;
    let to_cell = resolve(g, to, "to")?;
    if from_cell == to_cell {
        return Err("from and to resolved to the same cell".into());
    }
    let link = g
        .plant_drop(&bsp, from_cell, to_cell)
        .ok_or("not a drop the build would emit (needs a descent off a lip, hull-clear, within MAX_DROP)")?;
    g.rebuild_derived();
    let (fo, to_o) = (g.cell_origin(from_cell), g.cell_origin(to_cell));
    Ok(proto::PlanDropResp {
        link,
        from_cell,
        to_cell,
        from: a3(fo),
        tgt: a3(to_o),
        cost: g.link_cost(link),
    })
}

/// Hand-plant a self-contained `SpeedJump` link into the live graph for takeoff-regime bring-up: the
/// run-up starts at the cell nearest `from`, the leap is at `takeoff` (the lip), and it lands on the
/// cell nearest `tgt`, requiring `v_req` ups at the lip. The runtime flies a planted link exactly like
/// a generated one, so a subsequent `goto <tgt>` exercises the committed-prestrafe takeoff on the real
/// corridor. Returns the new link index and the resolved cell origins so the caller can verify routing.
fn plant_link_resp(
    game: &mut GameState,
    from: Vec3,
    takeoff: Vec3,
    tgt: Vec3,
    v_req: f32,
    gain: Option<f32>,
) -> Result<proto::PlanLinkResp, String> {
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
    // sweep — ~12 lands the bravado LG dead-on). An explicit `gain` on the command wins (a side-jump
    // sweep varies it per plant, and a server-wide cvar can't express that); the cvar is the fallback.
    let curl_gain = gain.filter(|g| *g > 0.0).unwrap_or_else(|| {
        let g = game.host.cvar(c"rtx_jump_curl_gain");
        if g > 0.0 {
            g
        } else {
            12.0
        }
    });
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
        // A hand-planted link makes no claim about its run-up's width, so it keeps the uncapped weave
        // — the harness measures the excursion rather than constraining it.
        ..Default::default()
    };
    let li = g.plant_speed_jump(from_cell, to_cell, cost, tr);
    // Refresh the reachability + LOD tables so the new link is visible to steer's O(1) reachable()
    // gate and the coarse router — otherwise a `goto` across the plant redirects to the nearest cell
    // reachable on the pre-plant graph instead of pathing over it.
    g.rebuild_derived();
    let (fo, to) = (g.cell_origin(from_cell), g.cell_origin(to_cell));
    Ok(proto::PlanLinkResp {
        link: li,
        from_cell,
        to_cell,
        from: a3(fo),
        tgt: a3(to),
        takeoff: a3(takeoff),
        v_req,
        airtime,
        cost,
    })
}

/// Probe the build-time curl certifier — see [`Cmd::Probe`].
fn probe_resp(game: &GameState, takeoff: Vec3, tgt: Vec3, psi0: f32, runway: f32) -> Result<proto::ProbeResp, String> {
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
    let gains = probe
        .landings
        .iter()
        .map(|&(gain, land)| proto::ProbeGain {
            gain,
            land: a3(land),
            miss_xy: (land.truncate() - tgt.truncate()).length(),
            miss_z: (land.z - tgt.z).abs(),
        })
        .collect();
    let certified = probe.certified.map(|(v_req, gain)| proto::Cert { v_req, gain });
    Ok(proto::ProbeResp {
        v_deliver: probe.v_deliver,
        certified,
        gains,
    })
}

/// The current map's raw BSP file, re-read on demand so a viewer can render the world without a local
/// copy of the map. Mirrors `load_map_bsp`'s read path.
fn bsp_resp(game: &GameState) -> Result<proto::BspResp, String> {
    let path = cstring(&format!("maps/{}.bsp", game.level.mapname));
    let bytes = game.host.read_file(&path).ok_or("could not read map BSP")?;
    Ok(proto::BspResp {
        map: game.level.mapname.clone(),
        bytes,
    })
}

/// Every map this server could load, lowercased, deduped and sorted.
///
/// The engine exposes no directory listing to a game module (`G_FSOpenFile` opens a *named* file), so
/// this walks the filesystem itself. That's sound because the module runs inside the server process:
/// its working directory is the server's, and the gamedirs are the ones the engine is searching. The
/// order mirrors `FS_AddPathHandle` — the active gamedir plus `id1` underneath it — and each is
/// checked for both loose `maps/*.bsp` and `maps/*.bsp` inside its `.pak`s, since a stock install
/// keeps every map in `pak0.pak` and has no `maps/` directory at all.
///
/// Best-effort by design: an unreadable directory or a damaged pak contributes nothing rather than
/// failing the listing, so a client always gets a usable (if shorter) picker.
fn maps_resp(game: &GameState) -> Vec<String> {
    let mut buf = [0u8; 64];
    // mvdsv publishes the active gamedir as the `*gamedir` serverinfo key; `qw` is the stock default.
    let gamedir = match game.host.infokey(EntId::WORLD, c"*gamedir", &mut buf) {
        "" => "qw".to_string(),
        g => g.to_string(),
    };
    let mut names: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for dir in [gamedir.as_str(), "id1"] {
        let dir = std::path::Path::new(dir);
        if let Ok(entries) = std::fs::read_dir(dir.join("maps")) {
            let stems = entries.flatten().map(|e| e.path()).filter(|p| {
                p.extension()
                    .is_some_and(|x| x.eq_ignore_ascii_case(std::ffi::OsStr::new("bsp")))
            });
            names.extend(stems.filter_map(|p| Some(p.file_stem()?.to_str()?.to_ascii_lowercase())));
        }
        for pak in crate::pak::paks_in(dir) {
            names.extend(pak.map_names().map(str::to_string));
        }
    }
    names.into_iter().collect()
}

/// List every generated curl link (SpeedJump with `curl_gain > 0`).
fn curls_resp(game: &GameState) -> Result<Vec<proto::CurlLink>, String> {
    let g = game.nav.graph.as_ref().ok_or("navmesh not ready")?;
    let mut curls = Vec::new();
    for li in 0..g.links.len() as u32 {
        if g.link_kind(li) != LinkKind::SpeedJump {
            continue;
        }
        let Some(tr) = g.speed_jump_of_link(li) else { continue };
        // The whole SpeedJump family, curls (`gain > 0`) and straight/chained alike. Filtering to
        // curls left the straight family listed by nothing, so it could not be fly-tested by id.
        curls.push(proto::CurlLink {
            link: li,
            from: a3(g.cell_origin(g.link_source(li))),
            takeoff: a3(tr.takeoff),
            tgt: a3(g.cell_origin(g.link_target(li))),
            v_req: tr.v_req,
            gain: tr.curl_gain,
            chained: tr.chained,
        });
    }
    Ok(curls)
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
        let traj = traj_rows(&std::mem::take(&mut game.entities[e].bot.puppet.traj));
        // A goto commonly ends while the bot is airborne and carrying several hundred ups. Merely
        // swapping the order to Hold leaves the active hop controller, route, and momentum intact for
        // another frame; on a finish-line target that is enough to cross trigger_changelevel, and on
        // an ordinary target it can produce a sharp stale-route turn after the reported arrival.
        // Finish the puppet order atomically: stop the body and discard every navigation commitment
        // before the next bot frame observes Hold.
        finish_goto_hold(game, e, origin, now);
        send_event(
            game,
            Event::Arrived {
                bot,
                t: now,
                origin: a3(origin),
                target: a3(target),
                dist: dxy,
                traj,
            },
        );
        return;
    }
    let (best_dist, best_since, best_z, anchor) = {
        let p = &game.entities[e].bot.puppet;
        (p.best_dist, p.best_since, p.best_z, p.anchor)
    };
    // A climb toward a target above (a spiral staircase) holds XY distance near-constant while
    // ascending correctly, so gaining altitude counts as progress and keeps the stall clock from
    // false-tripping on the only way up — mirrors the bot's own route watchdog.
    let climbed = origin.z > best_z + GOTO_CLIMB_EPS;
    // Progress is *movement*, not a new record closest approach. The old rule compared against a
    // high-water mark that only ever fell, so once the bot had been close, every later frame was
    // judged against its best ever — and any route that must first go the wrong way (around an
    // obstacle, back up after a fall, the long way round a wall) burned the clock while running at
    // full speed. A bot that is genuinely stuck does not move at all, which is what this asks.
    let travelled = (origin - anchor).length();
    if travelled > STALL_EPS || climbed {
        let p = &mut game.entities[e].bot.puppet;
        p.best_dist = dxy.min(p.best_dist); // still reported, as the closest it ever got
        p.best_z = p.best_z.max(origin.z);
        p.anchor = origin;
        p.best_since = now;
    } else if now - best_since > STALL_SECS {
        let traj = traj_rows(&std::mem::take(&mut game.entities[e].bot.puppet.traj));
        finish_goto_hold(game, e, origin, now);
        send_event(
            game,
            Event::GotoStall {
                bot,
                t: now,
                origin: a3(origin),
                target: a3(target),
                dist: dxy,
                best: best_dist,
                secs: STALL_SECS,
                traj,
            },
        );
    }
}

fn goto_crossed_finish(traj: &[(f32, Vec3, Vec3, u8)], origin: Vec3, target: Vec3) -> bool {
    let Some((_, start, _, _)) = traj.first() else {
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

/// A flight/goto trace as `[t, x, y, z, vx, vy, vz]` rows.
fn traj_rows(traj: &[(f32, Vec3, Vec3, u8)]) -> Vec<proto::TrajRow> {
    traj.iter()
        .map(|&(t, o, v, ph)| [t, o.x, o.y, o.z, v.x, v.y, v.z, ph as f32])
        .collect()
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
    let result = rj_result(bot, link, &telem, outcome, v0, blast, pos_blast, &traj);
    send_event(game, Event::RjResult(Box::new(result)));
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
        send_event(
            game,
            Event::FlyResult(proto::FlyResult {
                bot,
                link,
                on_target: false,
                timeout: true,
                land: a3(origin),
                target: [0.0; 3],
                miss_xy: 9999.0,
                miss_z: 9999.0,
                takeoff_speed: 0.0,
                peak: 0.0,
                traj: Vec::new(),
            }),
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
    send_event(
        game,
        Event::FlyResult(proto::FlyResult {
            bot,
            link,
            on_target,
            timeout: false,
            land: a3(origin),
            target: a3(target),
            miss_xy,
            miss_z,
            takeoff_speed,
            peak,
            traj: traj_rows(&traj),
        }),
    );
}

#[allow(clippy::too_many_arguments)] // one event's worth of measured + solved fields
fn rj_result(
    bot: u32,
    link: u32,
    t: &RjTelemetry,
    outcome: RjOutcome,
    v0: Vec3,
    blast: Vec3,
    pos_blast: Vec3,
    traj: &[(f32, Vec3, Vec3, u8)],
) -> proto::RjResult {
    // Terminal name + (for a touchdown/overrun) the landing measurement vs the target cell.
    let (name, land_pt) = match outcome {
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
    let press = t.press.map(|p| proto::RjPress {
        t: p.t,
        origin: a3(p.origin),
        view: [p.view.x, p.view.y],
        aim_err: p.aim_err,
        stance_off_xy: (p.origin.xy() - t.src.xy()).length(),
    });
    let fire = t.fire.map(|f| proto::RjFire {
        t: f.t,
        delay: f.actual_delay,
        origin: a3(f.origin),
        view: [f.view.x, f.view.y],
        pitch_err: f.view.x - (t.solved_angles.x + t.pitch_bias),
        yaw_err: wrap180(f.view.y - t.solved_angles.y),
    });
    let land = land_pt.map(|(o, ft)| proto::RjLand {
        t: ft,
        origin: a3(o),
        miss_xy: (o.xy() - t.tgt.xy()).length(),
        miss_z: (o.z - t.tgt.z).abs(),
    });
    proto::RjResult {
        bot,
        link,
        outcome: name.to_string(),
        src: a3(t.src),
        tgt: a3(t.tgt),
        solved: proto::RjSolved {
            pitch: t.solved_angles.x,
            yaw: t.solved_angles.y,
            delay: t.solved_delay,
            airtime: t.airtime,
            self_damage: t.self_damage,
            v0: a3(v0),
            blast: a3(blast),
            pos_blast: a3(pos_blast),
        },
        bias: proto::RjBias {
            delay: t.delay_bias,
            pitch: t.pitch_bias,
        },
        press,
        fire,
        land,
        traj: traj_rows(traj),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "netclient")]
    #[test]
    fn movement_report_accepts_high_shadow_ground_entities() {
        let host = Box::leak(Box::new(crate::netclient::host::NetHost::new("/nonexistent".into())));
        let mut game = GameState::new_client(host);
        let player = &mut game.entities[EntId(1)];
        player.in_use = true;
        player.classname = Some("player".into());
        player.v.groundentity = EntId(32767).to_prog();
        assert_eq!(pmove_event(&game, 1.0, 1).players[0].ground_ent, 32767);
        game.entities[EntId(1)].v.groundentity = EntId(32768).to_prog();
        assert_eq!(pmove_event(&game, 1.0, 1).players[0].ground_ent, -1);
    }

    #[test]
    fn fast_goto_crossing_stops_inside_bounded_finish_corridor() {
        let traj = vec![(0.0f32, Vec3::new(224.0, 1440.0, 24.0), Vec3::ZERO, 0u8)];
        let target = Vec3::new(224.0, 2992.0, 24.0);
        assert!(goto_crossed_finish(&traj, Vec3::new(280.0, 3008.0, 48.0), target));
        assert!(!goto_crossed_finish(&traj, Vec3::new(330.0, 3008.0, 48.0), target));
        assert!(!goto_crossed_finish(&traj, Vec3::new(224.0, 2970.0, 48.0), target));
    }

    #[test]
    fn cvar_name_guard() {
        assert!(valid_cvar_name("rtx_rj_stance"));
        assert!(!valid_cvar_name("rtx; quit"));
        assert!(!valid_cvar_name(""));
        assert!(!valid_cvar_name("foo bar"));
    }
}
