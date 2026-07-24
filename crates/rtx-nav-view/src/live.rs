// SPDX-License-Identifier: AGPL-3.0-or-later

//! Background poller for the live overlay: connect to a running game's control channel (framed
//! msgpack of the [`rtx_ctlproto`] schema), resolve the first live bot, and stream its route to the
//! viewer's event loop ~10×/s. It reconnects forever, so it can be started before the server is up —
//! and note the control server is single-client (a new connection supersedes the last), so attaching
//! navview takes the channel over from any other client (e.g. the MCP bridge).

use std::io::{self, Write};
use std::net::TcpStream;
use std::time::Duration;

use rtx_ctlproto::{self as proto, Cmd, Msg, Request, Resp};
use winit::event_loop::EventLoopProxy;

use crate::UserEvent;

/// Reconnect interval after a dropped connection or a failed connect attempt.
const RECONNECT: Duration = Duration::from_secs(5);

/// Spawn the poller thread. `port` is the game's `rtx_control_port` (default 27950).
pub fn spawn(proxy: EventLoopProxy<UserEvent>, port: u16) {
    std::thread::spawn(move || run(&proxy, port));
}

fn run(proxy: &EventLoopProxy<UserEvent>, port: u16) {
    let mut next_id: i64 = 1;
    loop {
        if let Ok(mut stream) = TcpStream::connect(("127.0.0.1", port)) {
            let _ = stream.set_nodelay(true);
            // `send_event` errors only once the event loop (the app) has shut down — stop the poller.
            if proxy.send_event(UserEvent::LiveConnected(true)).is_err() {
                return;
            }
            // Poll until any I/O error (the game closed / was restarted), then drop out to reconnect.
            let _ = session(proxy, &mut stream, &mut next_id);
            if proxy.send_event(UserEvent::LiveConnected(false)).is_err() {
                return;
            }
        }
        // Retry every RECONNECT until we reconnect or the app terminates (the thread dies with the
        // process, and the send_event guards above stop it as soon as the loop is gone).
        std::thread::sleep(RECONNECT);
    }
}

/// Poll one connection: resolve the first bot from `status`, then stream its `route` until an I/O
/// error ends the session (a bad-bot reply just re-resolves — the bot may have died/respawned).
fn session(proxy: &EventLoopProxy<UserEvent>, stream: &mut TcpStream, next_id: &mut i64) -> io::Result<()> {
    // Fetch the map BSP once up front so the viewer renders the exact map the game is running —
    // no local `.bsp` needed, and it works even for maps that live only inside a `.pak` (the game
    // serves it through the engine filesystem). A game-side error just leaves the viewer mapless.
    if let Ok(Resp::Bsp(b)) = request(stream, next_id, Cmd::Bsp)? {
        let _ = proxy.send_event(UserEvent::Bsp(b));
    }
    let mut bot: Option<u32> = None;
    loop {
        if bot.is_none() {
            if let Ok(Resp::Status(s)) = request(stream, next_id, Cmd::Status)? {
                bot = s.bots.first().map(|b| b.ent);
            }
            if bot.is_none() {
                std::thread::sleep(Duration::from_millis(300));
                continue;
            }
        }
        match request(stream, next_id, Cmd::Route { bot: bot.unwrap() })? {
            Ok(Resp::Route(r)) => {
                let _ = proxy.send_event(UserEvent::Live(Box::new(r)));
            }
            Ok(_) => {}
            Err(_) => bot = None, // e.g. the bot id went stale — re-resolve next loop
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Send one request and return its typed reply, skipping any async events that arrive first. Returns
/// `Err` only on I/O trouble; a game-side error surfaces as `Ok(Err(msg))`.
fn request(stream: &mut TcpStream, next_id: &mut i64, cmd: Cmd) -> io::Result<Result<Resp, String>> {
    let id = *next_id;
    *next_id += 1;
    stream.write_all(&proto::to_frame(&Request { id, cmd }))?;
    stream.flush()?;
    loop {
        let Some(frame) = proto::read_frame(stream)? else {
            return Err(io::Error::from(io::ErrorKind::UnexpectedEof));
        };
        match proto::decode::<Msg>(&frame).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))? {
            Msg::Reply { id: rid, result } if rid == id => return Ok(result),
            _ => continue, // an event, or a reply we're not waiting on — keep reading
        }
    }
}
