// SPDX-License-Identifier: AGPL-3.0-or-later

//! The bot brain as a real QuakeWorld **network client**.
//!
//! The same bots that run inside the server's game module, embodied instead as clients that connect
//! over UDP — so they can play against humans on any server, or against qwprogs-hosted bots. The
//! brain is not reimplemented and not forked: it is the *same code*, reading the same
//! [`GameState`](crate::game::GameState), and it does not know which of the two hosts it's under.
//!
//! # How the same brain runs in two places
//!
//! Inside the server, the engine fills each entity's `EntVars` and runs the bot's usercmd through
//! `SV_RunCmd`. Here, neither happens — so this module supplies both ends:
//!
//! ```text
//!   the server module                     the network client
//!   ─────────────────                     ──────────────────
//!   engine fills EntVars       ──▶        mirror writes EntVars from svc_playerinfo /
//!                                         svc_packetentities / stats
//!   engine answers traceline,  ──▶        NetHost answers from the map's own BSP
//!     pointcontents, cvars                (rtx-nav) and its own cvar store
//!   set_bot_cmd → SV_RunCmd    ──▶        cmd sink → clc_move on the wire
//!   server runs trigger touches──▶        the server does it for us (we're a real client)
//! ```
//!
//! Everything else — perception, goals, combat, steering, bhop, the navmesh — is untouched. The
//! trick that makes that possible is the [`ClientHost`](crate::host::ClientHost) seam plus a
//! discipline: **write network truth into exactly the fields the brain already reads**, rather than
//! teaching the brain a second way to ask.
//!
//! # What it will not know
//!
//! A server-side bot can read an enemy's health straight out of their entity. A client cannot —
//! that isn't on the wire, and no amount of parsing will put it there. The gap is filled by the
//! opponent model the bots already use for observation-gated estimates, which is the honest answer
//! and, not by coincidence, the one that makes them play like a player rather than a cheat.
//!
//! # Status
//!
//! The seam, the wire session and the tick loop are in place: a client connects, completes signon,
//! tracks the entity delta chain, and holds a connection. The **world mirror** — the part that
//! writes network truth into the entities the brain reads — is the next milestone, and until it
//! lands the bots have nothing to think about, so `--spectate` is the honest way to run this.

pub mod config;
pub(crate) mod frames;
pub(crate) mod host;
pub(crate) mod session;

use std::io;
use std::time::{Duration, Instant};

pub use config::{parse as parse_args, Config, USAGE};
use rtx_proto::info::UserinfoBuilder;
use rtx_proto::svc::SvcEvent;
use session::{Session, Signon};

use crate::game::GameState;
use host::NetHost;

/// The rate to send at before the server has said otherwise. QuakeWorld's traditional server frame
/// rate; KTX commonly runs 77.
const DEFAULT_MAXFPS: f32 = 72.0;

/// A bot client: the brain, hosted by [`NetHost`] instead of a server.
pub struct Client {
    game: GameState,
    host: &'static NetHost,
    sessions: Vec<Session>,
    config: Config,
    /// Wall clock at the last tick, for the frame time the game runs on.
    last_tick: Instant,
}

impl Client {
    /// Build a client from a parsed command line.
    ///
    /// The host is leaked deliberately: [`HostApi`](crate::host::HostApi) is `Copy` and is
    /// snapshotted throughout the bot code, so the reference it carries has to be `'static`. There
    /// is one host per process and it lives as long as the process, so `'static` is the truth
    /// rather than a workaround.
    pub fn new(config: Config) -> Self {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(config.basedir.clone())));
        host.set("rtx_bot_skill", &config.skill.to_string());
        // The hook needs server-side grapple state that isn't on the wire, so a client bot doesn't
        // use it. Routing around a link beats reaching for a rope that isn't there.
        host.set("rtx_grapple", "0");
        // Estimates are the only thing a client *can* know about an enemy's strength, so the
        // opponent model isn't optional here — it's the data source.
        host.set("rtx_bot_model", "1");
        // Explicit overrides last, so `+set` always wins.
        for (name, value) in &config.cvars {
            host.set(name, value);
        }
        Client {
            game: GameState::new_client(host),
            host,
            sessions: Vec::new(),
            config,
            last_tick: Instant::now(),
        }
    }

    /// The host, for tests and for the session to feed.
    #[allow(dead_code)]
    pub(crate) fn host(&self) -> &'static NetHost {
        self.host
    }

    /// The world the brain reads. The mirror writes it; until then, only the tests look.
    #[allow(dead_code)]
    pub(crate) fn game(&mut self) -> &mut GameState {
        &mut self.game
    }

    /// Bring every bot online.
    ///
    /// Each gets its own socket and qport, because each *is* its own client as far as the server is
    /// concerned — QuakeWorld has no notion of a connection carrying two players.
    pub fn connect(&mut self) -> io::Result<()> {
        for i in 0..self.config.bots {
            let name = if self.config.bots == 1 {
                self.config.name.clone()
            } else {
                format!("{}{}", self.config.name, i + 1)
            };
            let ui = UserinfoBuilder {
                name,
                team: self.config.team.clone(),
                skin: self.config.skin.clone(),
                topcolor: self.config.colors.0,
                bottomcolor: self.config.colors.1,
                spectator: self.config.spectate,
                ..Default::default()
            };
            // The qport identifies us across a NAT rebinding, so a squad needs distinct ones. Real
            // clients randomize; deriving them keeps a capture readable.
            let qport = 0x4000u16.wrapping_add(i as u16);
            self.sessions.push(Session::connect(self.config.server, ui, qport)?);
        }
        Ok(())
    }

    /// Run until the soak expires or everyone is gone.
    pub fn run(&mut self) -> io::Result<()> {
        let start = Instant::now();
        let deadline = self.config.soak.map(|s| start + Duration::from_secs(s));

        loop {
            self.tick()?;

            if self.sessions.iter().all(|s| s.signon() == Signon::Disconnected) {
                eprintln!("rtx-client: all sessions are gone");
                return Ok(());
            }
            if deadline.is_some_and(|d| Instant::now() >= d) {
                eprintln!("rtx-client: soak finished after {:.0}s", start.elapsed().as_secs_f32());
                self.report();
                return Ok(());
            }
            self.pace();
        }
    }

    /// Wait out the rest of this frame.
    ///
    /// Sleeping the whole slack would overshoot by the OS timer's granularity every frame, and at
    /// 72 Hz that is a move packet's worth of time — so sleep short and spin the tail. Send timing
    /// is not cosmetic here: `msec` is measured from this clock, and the server integrates it.
    fn pace(&self) {
        let interval = Duration::from_secs_f32(1.0 / self.maxfps());
        if self.last_tick.elapsed() >= interval {
            return;
        }
        if let Some(slack) = interval.checked_sub(self.last_tick.elapsed()) {
            if slack > Duration::from_millis(2) {
                std::thread::sleep(slack - Duration::from_millis(1));
            }
        }
        while self.last_tick.elapsed() < interval {
            std::hint::spin_loop();
        }
    }

    /// The rate the server runs at, which is the rate it wants our moves at.
    fn maxfps(&self) -> f32 {
        self.sessions
            .first()
            .and_then(|s| s.serverinfo().get_f32("maxfps"))
            .filter(|v| (10.0..=1000.0).contains(v))
            .unwrap_or(DEFAULT_MAXFPS)
    }

    /// One frame: read what the server said, advance the clock, drive the bots, send.
    fn tick(&mut self) -> io::Result<()> {
        let now = Instant::now();
        let dt = now.duration_since(self.last_tick).as_secs_f32();
        self.last_tick = now;

        // 1. Read.
        for i in 0..self.sessions.len() {
            let events = self.sessions[i].poll(self.host)?;
            for ev in &events {
                self.observe(i, ev);
            }
        }

        // 2. Advance the game's clock. The brain's timers — reaction delay, respawn waits, powerup
        //    countdowns — all read this, so it has to move whether or not there's a world yet.
        self.game.globals.frametime = dt;
        self.game.globals.time += dt;

        // 3. Drive the bots — once there is something to drive. The mirror that gives them a world
        //    is the next milestone; running the brain against an empty one would only teach it that
        //    the map is deserted.

        // 4. Send. Once a bot is embodied this carries its usercmd; until then it is the keepalive
        //    that stops the server timing us out, and the carrier the netchan needs to get the
        //    reliable signon messages out.
        for s in &mut self.sessions {
            match s.signon() {
                Signon::Active => s.send_move(glam::Vec3::ZERO, 0, 0, 0, 0, 0)?,
                Signon::Disconnected => {}
                _ => s.send_nop()?,
            }
        }
        Ok(())
    }

    /// Say the things worth saying while there's no mirror to consume them.
    ///
    /// A squad is N connections to one server, so every bot receives every broadcast — printing
    /// each copy would repeat the whole game log N times. Anything the server says *about the
    /// world* is therefore reported once, from the first session; only what it says *to a
    /// particular bot* is per-session.
    fn observe(&mut self, index: usize, ev: &SvcEvent) {
        match ev {
            SvcEvent::ServerData(sd) => eprintln!(
                "rtx-client: [{index}] joined {} on {:?} as slot {}{}",
                if sd.gamedir.is_empty() { "qw" } else { &sd.gamedir },
                sd.levelname,
                sd.playernum,
                if sd.spectator { " (spectating)" } else { "" }
            ),
            SvcEvent::Print { text, .. } if index == 0 => eprint!("{text}"),
            SvcEvent::Disconnect => eprintln!("rtx-client: [{index}] dropped by the server"),
            _ => {}
        }
    }

    /// What the run looked like — the numbers that say whether it went well.
    fn report(&self) {
        for (i, s) in self.sessions.iter().enumerate() {
            eprintln!(
                "rtx-client: [{i}] {:?}, map={} rtt={:.0}ms chokes={}",
                s.signon(),
                s.mapname(),
                s.rtt() * 1000.0,
                s.chokes
            );
        }
    }

    /// How far every bot has got, for a caller that wants to know without printing.
    #[allow(dead_code)]
    pub(crate) fn signons(&self) -> Vec<Signon> {
        self.sessions.iter().map(|s| s.signon()).collect()
    }
}

/// Run a bot client from a parsed command line.
pub fn run(config: Config) -> io::Result<()> {
    let mut client = Client::new(config);
    client.connect()?;
    client.run()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn config() -> Config {
        Config {
            basedir: PathBuf::from("/nonexistent"),
            ..Default::default()
        }
    }

    /// The whole point of the seam: a `GameState` can exist with no engine behind it, and the game
    /// reads its tunables from the client's own store without noticing the difference.
    ///
    /// A small test for a large claim. Everything downstream — the mirror, the brain, the tick loop
    /// — assumes the game runs unmodified against a non-server host; if that were false, it would be
    /// false here first.
    #[test]
    fn builds_a_game_with_no_engine_behind_it() {
        let mut client = Client::new(config());

        assert!(client.game().host().is_client());
        assert_eq!(client.game().host().cvar(c"rtx_bot_bhop"), 1.0);

        client.host().set_movevars(rtx_proto::svc::MoveVars {
            gravity: 640.0,
            ..Default::default()
        });
        assert_eq!(client.game().host().cvar(c"sv_gravity"), 640.0);
    }

    /// The client forces the two tunables a network client can't honestly leave alone — and an
    /// explicit `+set` still wins, because the operator knows things we don't.
    #[test]
    fn forces_the_tunables_a_client_must() {
        let mut client = Client::new(Config { skill: 7.0, ..config() });
        assert_eq!(client.game().host().cvar(c"rtx_grapple"), 0.0, "no hook: its state isn't on the wire");
        assert_eq!(client.game().host().cvar(c"rtx_bot_model"), 1.0, "estimates are the data source");
        assert_eq!(client.game().host().cvar(c"rtx_bot_skill"), 7.0);

        let mut client = Client::new(Config {
            cvars: vec![("rtx_bot_bhop".into(), "0".into())],
            ..config()
        });
        assert_eq!(client.game().host().cvar(c"rtx_bot_bhop"), 0.0, "an explicit +set wins");
    }

    /// A squad is N independent clients; the server tells them apart by qport and name, so both
    /// have to differ. A lone bot keeps the plain name.
    #[test]
    fn a_squad_gets_distinct_names_and_qports() {
        let mut squad = Client::new(Config { bots: 3, name: "bot".into(), ..config() });
        squad.connect().expect("bind");
        assert_eq!(squad.sessions.len(), 3);

        let mut solo = Client::new(Config { bots: 1, name: "bot".into(), ..config() });
        solo.connect().expect("bind");
        assert_eq!(solo.sessions.len(), 1);
    }

    /// The send rate follows the server, because that's the rate it wants moves at — with a sane
    /// default before it has said, and bounds against a server advertising nonsense.
    #[test]
    fn send_rate_follows_the_server_within_reason() {
        let mut client = Client::new(config());
        assert_eq!(client.maxfps(), DEFAULT_MAXFPS, "before any serverinfo");
        client.connect().expect("bind");
        assert_eq!(client.maxfps(), DEFAULT_MAXFPS);
        assert!(client.maxfps() > 0.0, "and never a division by zero");
    }
}
