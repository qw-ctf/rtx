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

pub(crate) mod adapters;
pub mod config;
pub(crate) mod frames;
pub(crate) mod host;
pub(crate) mod mirror;
pub(crate) mod session;
pub(crate) mod world;

use std::io;
use std::time::{Duration, Instant};

pub use config::{parse as parse_args, Config, USAGE};
use rtx_proto::info::UserinfoBuilder;
use rtx_proto::svc::SvcEvent;
use crate::entity::EntId;
use frames::EntityState;
use mirror::Mirror;
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
    /// One mirror per connection: each bot's stats are its own, and each knows which slot it is.
    /// The *world* they write into is shared — that's the squad, and it's what the bots have inside
    /// qwprogs too.
    mirrors: Vec<Mirror>,
    config: Config,
    /// Wall clock at the last tick, for the frame time the game runs on.
    last_tick: Instant,
    /// The map the shadow world was built for, so a level change is noticed.
    world_map: String,
    /// How far each bot has actually travelled, and where it last was. A bot that connects, spawns
    /// and then stands still looks identical to a working one in every other line of output — this
    /// is the number that tells them apart.
    travelled: Vec<(f32, glam::Vec3)>,
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
            mirrors: Vec::new(),
            config,
            last_tick: Instant::now(),
            world_map: String::new(),
            travelled: Vec::new(),
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
            self.mirrors.push(Mirror::default());
            self.travelled.push((0.0, glam::Vec3::ZERO));
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

        // 1. Read, and write what was said into the world.
        for i in 0..self.sessions.len() {
            let events = self.sessions[i].poll(self.host)?;
            for ev in &events {
                self.observe(i, ev);
                self.mirrors[i].apply(&mut self.game, ev);
            }
        }

        // 2. Build the world, if the map changed under us.
        self.rebuild_world_if_map_changed();

        // 3. Advance the game's clock. The brain's timers — reaction delay, respawn waits, powerup
        //    countdowns — all read this, so it has to move whether or not there's a world yet.
        self.game.globals.frametime = dt;
        self.game.globals.time += dt;

        // 4. Build the navmesh, off-thread, as the server does — but only once the world exists.
        //    `ensure_navmesh` gives up permanently if it can't read the map, so calling it before
        //    we know which map that is would disable the bots for the whole connection.
        if !self.world_map.is_empty() {
            self.game.ensure_navmesh();
        }

        // 5. Fold this frame's entities in — the rockets in the air, the doors that moved, which
        //    items are actually there.
        self.mirror_entities();

        // 6. Fold in what we believe about everyone, then drive the bots. The same `run_bots` the
        //    server calls, over the same world — it has no idea it isn't one.
        for m in &mut self.mirrors {
            m.write_estimates(&mut self.game);
        }
        crate::bot::run_bots(&mut self.game);
        self.measure_travel();

        // 7. Send. Once a bot is embodied this carries its usercmd; until then it is the keepalive
        //    that stops the server timing us out, and the carrier the netchan needs to get the
        //    reliable signon messages out.
        //    Whatever the brain emitted for each bot this frame becomes that bot's move. A bot that
        //    emitted nothing (no navmesh yet, or still connecting) still sends: the server needs a
        //    packet from us to not time us out, and the netchan needs one to carry signon replies.
        let cmds = self.host.take_cmds();
        let mut fired: Vec<EntId> = Vec::new();
        for (i, s) in self.sessions.iter_mut().enumerate() {
            if s.signon() == Signon::Disconnected {
                continue;
            }
            if s.signon() != Signon::Active {
                s.send_nop()?;
                continue;
            }
            let client = s.playernum() as i32 + 1;
            match cmds.iter().find(|c| c.client == client) {
                Some(c) => {
                    // Nothing tells a client when its own gun is ready — the server owns
                    // `attack_finished` and never sends it. But we know what we fired and when we
                    // pressed, and the delay is the table the server fires by.
                    if c.buttons & rtx_proto::clc::button::ATTACK as i32 != 0 {
                        fired.push(EntId(client as u32));
                    }
                    s.send_move(c.angles, c.forward, c.side, c.up, c.buttons as u8, c.impulse as u8)?
                }
                None => s.send_move(glam::Vec3::ZERO, 0, 0, 0, 0, 0)?,
            }
            let _ = i;
        }
        for e in fired {
            self.game.client_note_own_fire(e);
        }
        Ok(())
    }

    /// Spawn the shadow world when the session binds a map, and again when the map changes.
    ///
    /// Keyed off the session having read the map, which happens at `prespawn` — by which point the
    /// host has the BSP and the entity string, and the whole of the module's spawn code can run
    /// against them exactly as it would on a server.
    fn rebuild_world_if_map_changed(&mut self) {
        let Some(map) = self.sessions.first().map(|s| s.mapname().to_string()) else {
            return;
        };
        if map.is_empty() || map == self.world_map {
            return;
        }
        self.world_map = map.clone();

        // A new level voids every entity: the shadow furniture belongs to the old map, and the
        // network numbers are about to be reassigned. A server module gets this done for it by the
        // engine's edict clear; here it's ours to do. Note `reset()` marks a slot *spawned* — what's
        // wanted is a free one, so this is `default()` rather than a reset.
        for i in 0..self.game.entities.len() as u32 {
            self.game.entities[crate::entity::EntId(i)] = crate::entity::Entity::default();
        }
        self.game.spawn_shadow_world();
        // The items are what the mirror reasons about the *absence* of, so it has to know where
        // they all are before the first frame lands.
        for m in &mut self.mirrors {
            m.index_items(&self.game);
        }

        // The counts are the shadow world's proof of life, and worth printing rather than trusting:
        // if `droptofloor` misfires, items are *deleted* as having fallen out of the level, and a
        // world with no items looks exactly like a working one until a bot has nothing to collect.
        let items = self
            .game
            .entities
            .live()
            .filter(|(_, e)| e.classname().is_some_and(crate::bot::goals::is_goal_classname))
            .count();
        let spawns = self
            .game
            .entities
            .live()
            .filter(|(_, e)| e.classname() == Some("info_player_deathmatch"))
            .count();
        eprintln!(
            "rtx-client: world: {map} — {} entities, {items} items, {spawns} spawn points",
            self.game.entities.live().count(),
        );
    }

    /// Fold every bot's view of this frame into the one shared world.
    ///
    /// A squad is several clients watching the same game from different places, so each sees a
    /// different subset — the server culls what it sends by what you could see. The **union** is
    /// what the team collectively knows, which is exactly what the bots share inside qwprogs, and
    /// taking any one bot's view alone would have the others forget everything they can't see.
    fn mirror_entities(&mut self) {
        let mut seen: Vec<EntityState> = Vec::new();
        for s in &self.sessions {
            for e in s.frames.current() {
                if !seen.iter().any(|x| x.number == e.number) {
                    seen.push(*e);
                }
            }
        }
        // The model list names what each entity is; it's per map, and identical across a squad.
        let models: Vec<String> = self.sessions.first().map(|s| s.models().to_vec()).unwrap_or_default();
        if models.is_empty() {
            return;
        }
        // One mirror does the shared world (they'd otherwise fight over it); the rest keep only
        // their own body and stats, which they already did on the way in.
        if let Some(m) = self.mirrors.first_mut() {
            m.apply_frame(&mut self.game, &seen, &models);
        }
    }

    /// Track how far each bot has moved.
    ///
    /// A teleport isn't travel, and neither is a respawn across the map, so a single frame's jump
    /// is discarded rather than counted — otherwise a bot that dies repeatedly would look like a
    /// marathon runner.
    fn measure_travel(&mut self) {
        for (i, m) in self.mirrors.iter().enumerate() {
            let e = m.own();
            if !self.game.entities[e].in_use {
                continue;
            }
            let origin = self.game.entities[e].v.origin;
            let (dist, last) = &mut self.travelled[i];
            if *last != glam::Vec3::ZERO {
                let step = origin.distance(*last);
                if step < 200.0 {
                    *dist += step;
                }
            }
            *last = origin;
        }
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
            let e = self.mirrors[i].own();
            let ent = &self.game.entities[e];
            eprintln!(
                "rtx-client: [{i}] {:?} map={} rtt={:.0}ms chokes={}",
                s.signon(),
                s.mapname(),
                s.rtt() * 1000.0,
                s.chokes
            );
            // Travel is the honest measure of "is it playing": everything else can look right while
            // the bot stands on its spawn.
            eprintln!(
                "rtx-client:      travelled {:.0}u, at {:.0?}, health {} armor {} frags {}",
                self.travelled[i].0,
                ent.v.origin,
                ent.v.health,
                ent.v.armorvalue,
                ent.v.frags,
            );
            let (up, waiting, tracked) = self.mirrors[i].census(&self.game);
            eprintln!("rtx-client:      items: {up} up, {waiting} timed; {tracked} tracked now");
            eprintln!(
                "rtx-client:      projectiles: {} seen in flight (peak {} at once)",
                self.mirrors[i].projectiles_seen, self.mirrors[i].projectiles_peak,
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
