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
//! # A squad is one world
//!
//! One process hosts N connections and **one** [`GameState`]. That isn't a shortcut: it's what the
//! bots already have inside qwprogs, where teammates share an opponent model and a set of item
//! timers because they talk to each other. Each bot's *body* and *stats* are its own connection's
//! and nobody else's; the world they write into is shared, and what any one of them can see, all of
//! them know.
//!
//! That sentence is the whole of the module's shape, and it's worth reading as a layout:
//!
//! ```text
//!   Client
//!     game     one GameState — the brain reads it and cannot tell which host it's under
//!     world    one WorldMirror — the map's items, its doors, everything in flight
//!     bots     one Bot each ─┬─ Session  the wire: netchan, signon, the move stream
//!                            └─ Mirror   this body: its slot, its stats, what it can see
//! ```
//!
//! Which half a thing belongs in is not a matter of taste, and getting it wrong has produced every
//! squad bug so far. Two rules keep it straight:
//!
//! **Each connection receives everything** — including messages about our own other bots. A mirror
//! can tell its own body from the rest, but not a squadmate's from a stranger's, so the rules that
//! need that distinction take a [`Squad`](mirror::Squad) and ask. It's a per-tick snapshot rather
//! than a live view because you cannot lend one mirror its siblings while mutating it.
//!
//! **The world outlives any connection to it.** Anything shared that were kept per connection would
//! need one connection elected to write it — and elections have losers: the survivor inherits none of
//! what its predecessor was tracking, and a rocket in the air at the handover hangs there forever.
//! Hence one [`WorldMirror`], and [`lead`](Client::lead) rather than "bot 0" wherever a single
//! connection has to answer for the squad.

pub mod config;
pub(crate) mod download;
pub(crate) mod frames;
pub(crate) mod host;
pub(crate) mod mirror;
pub(crate) mod modes;
pub(crate) mod pak;
pub(crate) mod senses;
pub(crate) mod session;
pub(crate) mod world;

use std::io;
use std::time::{Duration, Instant};

pub use config::{parse as parse_args, Config, USAGE};
use rtx_proto::info::UserinfoBuilder;
use rtx_proto::svc::SvcEvent;
use frames::EntityState;
use mirror::{Mirror, Squad, WorldMirror};
use session::{Session, Signon};

use crate::entity::EntId;
use crate::game::GameState;
use host::NetHost;

/// The rate to send at before the server has said otherwise. QuakeWorld's traditional server frame
/// rate; KTX commonly runs 77.
const DEFAULT_MAXFPS: f32 = 72.0;

/// The most latency worth aiming across. A quarter-second is already a bad transatlantic link; past
/// it the bot is guessing, and a guess that lands 100 units in front of a strafing player is worse
/// than aiming where it can see. Also a NaN/garbage stop, since this multiplies a velocity.
const MAX_LEAD: f32 = 0.25;

/// One bot: its wire, its view of the world, and the odometer that proves it's playing.
///
/// The two halves are kept as separate types on purpose. A [`Session`] knows the wire and nothing
/// about the game — which is what lets it be tested against a recorded conversation with no bot in
/// sight — and a [`Mirror`] knows the game and nothing about sockets. This is the seam between them,
/// and there is exactly one of these per connection because QuakeWorld has no notion of a connection
/// carrying two players.
struct Bot {
    session: Session,
    mirror: Mirror,
    /// How far it has actually travelled. A bot that connects, spawns and then stands still looks
    /// identical to a working one in every other line of output; this is the number that tells them
    /// apart.
    travelled: f32,
    /// Where it was last frame, once it has a body to be anywhere. A teleport or a respawn across
    /// the map is a single frame's jump and is discarded rather than counted, or a bot that died a
    /// lot would look like a marathon runner.
    last_at: Option<glam::Vec3>,
}

impl Bot {
    /// Fold one message into the world.
    fn apply(&mut self, game: &mut GameState, world: &mut WorldMirror, squad: &Squad, ev: &SvcEvent) {
        self.mirror.apply(game, world, squad, ev, self.session.sounds());
    }

    /// Note how far it moved this frame.
    fn measure_travel(&mut self, game: &GameState) {
        let e = self.mirror.own();
        if !game.entities[e].in_use {
            return;
        }
        let origin = game.entities[e].v.origin;
        if let Some(last) = self.last_at {
            let step = origin.distance(last);
            if step < TELEPORT_STEP {
                self.travelled += step;
            }
        }
        self.last_at = Some(origin);
    }
}

/// The players a client can reason about: in the game, and not a corpse.
///
/// The scan every part of this module opens with, named once. A client's entity picture is
/// PVS-culled before it arrives, so "the live players" is already "the live players we can see" —
/// which is the whole population a client is entitled to have an opinion about. Lazy on purpose:
/// these run per item, per projectile, per frame, and the callers all stop early.
pub(crate) fn live_players(game: &GameState) -> impl Iterator<Item = EntId> + '_ {
    let maxclients = game.host().cvar(c"maxclients") as u32;
    (1..=maxclients)
        .map(EntId)
        .filter(move |&p| game.entities[p].in_use && game.entities[p].is_alive())
}

/// A step no bot takes by walking. Anything this big in one frame was the server moving us — a
/// teleport, or a respawn — and counting it would make a bot that dies a lot look well travelled.
const TELEPORT_STEP: f32 = 200.0;

/// Apply a cvar cfg to the host, following `exec` chains. Silent if the file isn't there — a default
/// `rtx.cfg` that doesn't exist is not an error, it just means "no cfg".
///
/// `depth` bounds the `exec` recursion so a cfg that execs itself (or a cycle of them) can't loop
/// forever. An `exec` path is resolved relative to the cfg that named it, the way a console does.
fn exec_cfg(host: &NetHost, path: &std::path::Path, depth: u32) {
    const MAX_DEPTH: u32 = 8;
    if depth > MAX_DEPTH {
        eprintln!("rtx-client: cfg: too many nested exec (cycle?) at {}", path.display());
        return;
    }
    let Ok(text) = std::fs::read_to_string(path) else {
        return; // absent cfg — fine
    };
    if depth == 0 {
        eprintln!("rtx-client: reading {}", path.display());
    }
    for line in config::parse_cfg(&text) {
        match line {
            config::CfgLine::Set(name, value) => host.set(&name, &value),
            config::CfgLine::Exec(file) => {
                let child = path.parent().unwrap_or(std::path::Path::new(".")).join(file);
                exec_cfg(host, &child, depth + 1);
            }
        }
    }
}

/// A bot client: the brain, hosted by [`NetHost`] instead of a server.
pub struct Client {
    game: GameState,
    host: &'static NetHost,
    /// The squad. Each bot's body and stats are its own; the world below is shared.
    bots: Vec<Bot>,
    /// Everything that belongs to nobody: the map's items, its moving brushwork, and whatever is in
    /// flight. One copy, because there is one world — the bots merely see different parts of it.
    world: WorldMirror,
    config: Config,
    /// Wall clock at the last tick, for the frame time the game runs on.
    last_tick: Instant,
    /// The map the shadow world was built for, and which incarnation of the server built it — so a
    /// level change is noticed, and so is a restart onto the same level.
    world_map: (String, i32),
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
        // The control channel binds off a cvar, so it needs no client-specific plumbing — the same
        // harness that drives a server's bots drives these.
        if let Some(port) = config.control_port {
            host.set("rtx_control_port", &port.to_string());
        }
        // A cfg of cvar settings — the client's `server.cfg`. `--config` names one; otherwise
        // `<basedir>/rtx.cfg` if it's there, so a bot can be tuned by dropping a file next to its
        // maps, the way the server is tuned by its own cfg.
        let cfg = config.config_file.clone().unwrap_or_else(|| config.basedir.join("rtx.cfg"));
        exec_cfg(host, &cfg, 0);
        // Explicit overrides last, so a command-line `+set` always wins — over the cfg and the rest.
        for (name, value) in &config.cvars {
            host.set(name, value);
        }
        Client {
            game: GameState::new_client(host),
            host,
            bots: Vec::new(),
            world: WorldMirror::default(),
            config,
            last_tick: Instant::now(),
            world_map: (String::new(), 0),
        }
    }

    /// The host the brain asks about the map and the tunables. Only the tests reach for it from
    /// outside — everything else already holds one.
    #[cfg(test)]
    pub(crate) fn host(&self) -> &'static NetHost {
        self.host
    }

    /// The world the brain reads. Likewise: the tick loop owns it directly.
    #[cfg(test)]
    pub(crate) fn game(&mut self) -> &mut GameState {
        &mut self.game
    }

    /// Bring every bot online.
    ///
    /// Each gets its own socket and qport, because each *is* its own client as far as the server is
    /// concerned — QuakeWorld has no notion of a connection carrying two players.
    pub fn connect(&mut self) -> io::Result<()> {
        for i in 0..self.config.bots {
            // The label after the `bot•` tag: a name from the list per bot by default, or the
            // operator's `--name` (a squad appending a number). Then wrap it in the coloured tag so
            // every rtx bot reads `bot•<label>` on the scoreboard.
            let label = match &self.config.name {
                None => crate::bot::bot_name(i as i32).to_string(),
                Some(base) if self.config.bots == 1 => base.clone(),
                Some(base) => format!("{base}{}", i + 1),
            };
            let ui = UserinfoBuilder {
                name: crate::bot::bot_display_name(&label),
                team: self.config.team.clone(),
                skin: self.config.skin.clone(),
                topcolor: self.config.colors.0,
                bottomcolor: self.config.colors.1,
                spectator: self.config.spectate,
                bot: true, // announce ourselves — a bot on a human server should say so
                ..Default::default()
            };
            // The qport identifies us across a NAT rebinding, so a squad needs distinct ones. Real
            // clients randomize; deriving them keeps a capture readable.
            let qport = 0x4000u16.wrapping_add(i as u16);
            self.bots.push(Bot {
                session: Session::connect(
                    self.config.server,
                    ui,
                    qport,
                    self.config.wiretap.as_deref(),
                    self.config.download,
                )?,
                mirror: Mirror::default(),
                travelled: 0.0,
                last_at: None,
            });
        }
        Ok(())
    }

    /// Run until the soak expires or everyone is gone.
    pub fn run(&mut self) -> io::Result<()> {
        let start = Instant::now();
        let deadline = self.config.soak.map(|s| start + Duration::from_secs(s));

        loop {
            self.tick()?;

            if self.lead().is_none() {
                eprintln!("rtx-client: all sessions are gone");
                self.report();
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

    /// The connection that speaks for the squad.
    ///
    /// Several questions have one answer for the whole squad — which map we're on, what the model
    /// list says, how fast the server wants our moves — and any connection that's still up can
    /// answer them. It has to be *any*, though, rather than the first: a squad outlives its bots, and
    /// a dropped connection's answers freeze at the moment it died. Asking a corpse which map we're
    /// on is how the rest of the squad ends up playing a new one against the old one's navmesh.
    ///
    /// Not `Active`, note — `Loading` counts. The shadow world is built at `prespawn`, before anyone
    /// is in the game, and it's built from the answers to exactly these questions.
    fn lead(&self) -> Option<&Bot> {
        self.bots.iter().find(|b| b.session.signon() != Signon::Disconnected)
    }

    /// The rate the server runs at, which is the rate it wants our moves at.
    fn maxfps(&self) -> f32 {
        self.lead()
            .and_then(|b| b.session.serverinfo().get_f32("maxfps"))
            .filter(|v| (10.0..=1000.0).contains(v))
            .unwrap_or(DEFAULT_MAXFPS)
    }

    /// One frame: read what the server said, advance the clock, drive the bots, send.
    fn tick(&mut self) -> io::Result<()> {
        let now = Instant::now();
        let dt = now.duration_since(self.last_tick).as_secs_f32();
        self.last_tick = now;

        // 1. Read everything every connection has to say, before writing any of it. The order is
        //    load-bearing for a squad: the shape of the squad — which slots are our own bots' — is
        //    established by these reads, and the writes below need it settled before the first one.
        let mut inbox: Vec<Vec<SvcEvent>> = Vec::with_capacity(self.bots.len());
        for b in &mut self.bots {
            inbox.push(b.session.poll(self.host)?);
        }
        let squad = self.squad();
        for (i, events) in inbox.iter().enumerate() {
            for ev in events {
                self.observe(i, ev);
                self.bots[i].apply(&mut self.game, &mut self.world, &squad, ev);
            }
        }

        // 2. Build the world, if the map changed under us.
        self.rebuild_world_if_map_changed();

        // 3. Advance the game's clock. The brain's timers — reaction delay, respawn waits, powerup
        //    countdowns — all read this, so it has to move whether or not there's a world yet.
        //
        //    It's our own clock, not the server's, and that's deliberate. `STAT_TIME` is on the wire
        //    (we ask for it in `*z_ext`) and nothing reads it, because nothing needs to: every timer
        //    here is one we set ourselves, from this clock, and compared against this clock — an item
        //    is due 20 seconds after *we* saw it go. Adopting the server's would buy no accuracy the
        //    two clocks don't already share, and would cost the one property all of those timers
        //    quietly depend on: a level change resets the server's clock to zero, and every deadline
        //    in the future would land in the distant past at once.
        self.game.globals.frametime = dt;
        self.game.globals.time += dt;

        // 4. Build the navmesh, off-thread, as the server does — but only once the world exists.
        //    `ensure_navmesh` gives up permanently if it can't read the map, so calling it before
        //    we know which map that is would disable the bots for the whole connection.
        if !self.world_map.0.is_empty() {
            self.game.ensure_navmesh();
        }

        // 5. Fold this frame's entities in — the rockets in the air, the doors that moved, which
        //    items are actually there. The squad is rebuilt: the bots have bodies now, and where
        //    those bodies are looking from is what decides whether an item's absence means anything.
        let squad = self.squad();
        self.mirror_entities(&squad);

        // 6. How far behind the world we're shooting, what phase the match is in (so the RJ gate
        //    respects a countdown), and then drive the bots. The same `run_bots` the server calls,
        //    over the same world — it has no idea it isn't one. The control channel brackets it
        //    exactly as it does on a server: a `goto` issued this frame should take effect this frame.
        self.game.client_lead = self.latency();
        self.feed_phase();
        crate::control::frame_begin(&mut self.game);
        crate::bot::run_bots(&mut self.game);
        for b in &mut self.bots {
            b.measure_travel(&self.game);
        }
        crate::control::frame_end(&mut self.game);

        // 7. Send. Once a bot is embodied this carries its usercmd; until then it is the keepalive
        //    that stops the server timing us out, and the carrier the netchan needs to get the
        //    reliable signon messages out.
        self.send_moves()
    }

    /// Put this frame's decisions on the wire, one packet per connection.
    ///
    /// Every connection sends, every frame, whatever the brain had to say — including nothing. A bot
    /// still connecting, or waiting on a navmesh, has no move to make and must send anyway: the
    /// server times out a client that goes quiet, and the netchan needs a packet of ours to carry the
    /// signon replies out on.
    fn send_moves(&mut self) -> io::Result<()> {
        self.flush_console();
        let cmds = self.host.take_cmds();
        let auto_ready = self.config.auto_ready && !self.config.spectate;

        for bot in &mut self.bots {
            match bot.session.signon() {
                Signon::Disconnected => continue,
                // Not in the game yet: the packet is pure carrier.
                Signon::Active => {}
                _ => {
                    bot.session.send_nop()?;
                    continue;
                }
            }
            // Say we'll play, if the server is the kind that waits to be told. Once per level, and
            // not while the last one's scoreboard is up — KTX ignores it there, and the reload that
            // ends the scoreboard re-arms it.
            if auto_ready && !bot.session.at_intermission() {
                bot.session.ready_up();
            }
            // At a scoreboard, hold +attack and nothing else. The map won't advance without it: the
            // stock intermission (`server.rs` `intermission_think`) waits for a *held* button once
            // its own five-second minimum has passed, and a bots-only server with no `rtx_maplist`
            // will otherwise sit on the scoreboard forever. We still suppress the brain's own cmd —
            // the bot mustn't try to *play* the scoreboard, it can't see that the game is over — so
            // this is a fixed press, not the emitted move. KTX advances by its own clock and ignores
            // the button harmlessly.
            if bot.session.at_intermission() {
                let attack = rtx_proto::clc::button::ATTACK;
                bot.session.send_move(glam::Vec3::ZERO, 0, 0, 0, attack, 0)?;
                continue;
            }

            let e = bot.mirror.own();
            match cmds.iter().find(|c| c.client == e.0 as i32) {
                Some(c) => {
                    // Nothing tells a client when its own gun is ready — the server owns
                    // `attack_finished` and never sends it. But we know what we fired and when we
                    // pressed, and the delay is the table the server fires by.
                    if c.buttons & rtx_proto::clc::button::ATTACK as i32 != 0 {
                        self.game.client_note_own_fire(e);
                    }
                    bot.session.send_move(c.angles, c.forward, c.side, c.up, c.buttons as u8, c.impulse as u8)?;
                }
                None => bot.session.send_idle()?,
            }
        }
        Ok(())
    }

    /// Spawn the shadow world when the session binds a map, and again whenever the server starts a
    /// fresh one — including a restart onto the map we're already on.
    ///
    /// Keyed off the session having read the map, which happens at `prespawn` — by which point the
    /// host has the BSP and the entity string, and the whole of the module's spawn code can run
    /// against them exactly as it would on a server.
    ///
    /// The name alone won't do, because the case that matters most doesn't change it: KTX ends a
    /// match by reloading the same map, and every item on it goes back to being up. Keyed on the name
    /// we'd keep the finished match's world — items marked taken, timers due, lifts wherever they
    /// stopped — and hand it to the bots as the state of a game that has just started. `servercount`
    /// is the server's own incarnation number and changes every time.
    fn rebuild_world_if_map_changed(&mut self) {
        let Some(here) = self.lead().map(|b| (b.session.mapname().to_string(), b.session.servercount())) else {
            return;
        };
        // Not just "is the name set" but "is the map actually loaded": the session names the map at
        // prespawn, which can be *before* the file is on disk when it's still downloading. Spawning
        // now would build the world against no geometry and then mark it done, so the real map, once
        // it lands, would be skipped as already-built.
        if here.0.is_empty() || here == self.world_map || !self.host.has_map() {
            return;
        }
        let map = here.0.clone();
        self.world_map = here;

        // Work out what game this is *before* the world spawns, because the spawn reads it: the mode
        // decides which entities exist (CTF's flags, a mode's item set) and `refresh_mode` runs inside
        // `worldspawn`. The server described itself in serverinfo, which arrived during signon — long
        // before this point.
        self.select_mode();
        // And match the rtx movement features the server actually provides, before the navmesh build
        // reads them — so we don't plan a double-jump the server won't grant.
        self.sync_movement();

        // A new level voids every entity: the shadow furniture belongs to the old map, and the
        // network numbers are about to be reassigned. A server module gets this done for it by the
        // engine's edict clear; here it's ours to do. Note `reset()` marks a slot *spawned* — what's
        // wanted is a free one, so this is `default()` rather than a reset.
        for i in 0..self.game.entities.len() as u32 {
            self.game.entities[crate::entity::EntId(i)] = crate::entity::Entity::default();
        }
        self.game.spawn_shadow_world();
        // The items are what the world reasons about the *absence* of, so it has to know where they
        // all are before the first frame lands. Everything else it remembered — which shadow entity
        // was which door, what was in the air — named slots in a map that no longer exists.
        self.world.index_items(&mut self.game);
        // And the nail pools were allocated out of the old map's entity range, which
        // `spawn_shadow_world` has just handed back.
        for b in &mut self.bots {
            b.mirror.forget_map();
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

    /// How far behind the world we are, in seconds.
    ///
    /// The round trip: we see the enemy where they were half a trip ago, and the server judges our
    /// shot half a trip from now. One number for the squad, because every connection here runs from
    /// one machine to one server — they don't have meaningfully different latencies, and a bot that
    /// believed it did would only be modelling jitter. Sanity-bounded, because an early estimate
    /// is built from very few samples and a bot leading a second and a half would simply never hit.
    fn latency(&self) -> f32 {
        self.bots
            .iter()
            .filter(|b| b.session.signon() == Signon::Active)
            .map(|b| b.session.rtt())
            .fold(0.0f32, f32::max) // nobody connected yet ⇒ 0: nothing to be behind
            .clamp(0.0, MAX_LEAD)
    }

    /// Send on anything the game asked a console to run.
    ///
    /// A server module's `localcmd` reaches the server's own console; a client hasn't got one, so
    /// the honest equivalent is what a real client's console does with a word it doesn't recognise —
    /// send it to the server as a `clc_stringcmd`. `set` never arrives here ([`NetHost::localcmd`]
    /// interprets those itself, since there's no console to interpret them for it), which leaves the
    /// control channel's `console` verb: an operator asking the server something down a bot's
    /// connection. It goes out on the first live one, because it's the *server* being addressed and
    /// which bot carries the question doesn't matter.
    fn flush_console(&mut self) {
        let queued = self.host.take_pending_cmds();
        if queued.is_empty() {
            return;
        }
        let Some(bot) = self.bots.iter_mut().find(|b| b.session.signon() == Signon::Active) else {
            return; // nobody in the game to say it through; the operator can ask again
        };
        for cmd in queued {
            bot.session.stringcmd(&cmd);
        }
    }

    /// Select the mode from what the server said about itself, unless the operator pinned it.
    ///
    /// Writes the two cvars `refresh_mode` resolves from (`rtx_mode`/`rtx_match`); the shadow-world
    /// spawn that follows reads them. A `+set rtx_mode`/`rtx_match` on the command line is the last
    /// word — the operator knows things the wire can't tell us — so a key that appears in
    /// `config.cvars` is left exactly as they set it.
    fn select_mode(&self) {
        let pinned = |key: &str| self.config.cvars.iter().any(|(k, _)| k == key);
        let Some(info) = self.lead().map(|b| b.session.serverinfo()) else {
            return;
        };
        let choice = modes::select_mode(info);
        if !pinned("rtx_mode") {
            self.host.set("rtx_mode", choice.mode);
        }
        if !pinned("rtx_match") {
            self.host.set("rtx_match", &choice.composition);
        }
        eprintln!(
            "rtx-client: mode: {} {} (from serverinfo \"{}\")",
            choice.mode,
            choice.composition,
            info.get("mode").unwrap_or(""),
        );
    }

    /// Match the rtx-specific movement features to what the server actually provides.
    ///
    /// These run *server-side* (a double jump, a wall dodge, the shootable-grenade combo), so they
    /// exist only on an rtx server — and the sharp one, the double jump, decides whether the navmesh
    /// plans routes across gaps that need it (`nav_build.rs`'s `djump` links). On a KTX or vanilla
    /// server that second jump is never granted, so a bot planning around it would leap into a pit.
    ///
    /// So: not an rtx server → force every one off, and the navmesh never mints a link the server
    /// won't honour. An rtx server → mirror exactly what it advertised in serverinfo (it publishes
    /// each, `mode/mod.rs::publish_movement`), so a server running with, say, double jump off is
    /// matched rather than assumed on. A key the operator pinned with `+set` is left alone — they know
    /// something we don't. Run before the world spawns, since the navmesh build reads these.
    fn sync_movement(&self) {
        let pinned = |key: &str| self.config.cvars.iter().any(|(k, _)| k == key);
        let Some(info) = self.lead().map(|b| b.session.serverinfo()) else {
            return;
        };
        for (cv, value) in modes::movement_overrides(info, pinned) {
            self.host.set(cv, &value);
        }
    }

    /// Feed the server's match phase into the game, so the bot's rocket-jump gate respects a
    /// countdown. Cheap and per-tick: `match_phase` is a serverinfo lookup, and only a change to
    /// `team_match.phase` matters downstream.
    fn feed_phase(&mut self) {
        let Some(phase) = self.lead().map(|b| modes::match_phase(b.session.serverinfo())) else {
            return;
        };
        // Map onto the lifecycle's own phase. The client never *runs* the lifecycle (no
        // `tick_lifecycle`), so this is the only thing that moves `phase` here, and the only reader
        // that matters client-side is `match_weapons_hot`.
        use crate::mode::team::MatchPhase;
        let now = self.game.time();
        self.game.team_match.phase = match phase {
            modes::Phase::Warmup => MatchPhase::Warmup,
            // The `until` only has to be in the future — nothing client-side counts it down, it just
            // has to read as "a countdown is running" for the weapons gate.
            modes::Phase::Countdown => MatchPhase::Countdown { until: now + 1.0 },
            modes::Phase::Live => MatchPhase::Live,
        };
    }

    /// Which slots are our own bots', and where each of them is looking from.
    ///
    /// Rebuilt every tick rather than kept, because both halves move: a bot's slot is reassigned on
    /// reconnect and on a map change, and its eyes are wherever it walked to since the last frame.
    fn squad(&self) -> Squad {
        let mut slots = [false; mirror::MAX_CLIENTS];
        let mut eyes = Vec::new();
        for b in self.bots.iter().filter(|b| b.session.signon() == Signon::Active) {
            if let Some(slot) = slots.get_mut(b.session.playernum() as usize) {
                *slot = !b.mirror.spectating();
            }
            eyes.extend(b.mirror.eyes(&self.game));
        }
        Squad::new(slots, eyes)
    }

    /// Fold every bot's view of this frame into the one shared world.
    ///
    /// A squad is several clients watching the same game from different places, so each sees a
    /// different subset — the server culls what it sends by what you could see. The **union** is
    /// what the team collectively knows, which is exactly what the bots share inside qwprogs, and
    /// taking any one bot's view alone would have the others forget everything they can't see.
    ///
    /// Where two views disagree about the same entity, the newer one wins. That matters less for the
    /// frame or so that separates two healthy connections than for an unhealthy one: a session that
    /// stops receiving keeps its last snapshot indefinitely, and without a freshness rule its frozen
    /// rockets would hang in the air over every live view of them for the rest of the run.
    fn mirror_entities(&mut self, squad: &Squad) {
        let mut seen: Vec<(EntityState, Instant)> = Vec::new();
        let playing = || self.bots.iter().filter(|b| b.session.signon() == Signon::Active);
        for b in playing() {
            let at = b.session.frames_at();
            for e in b.session.frames.current() {
                match seen.iter_mut().find(|(x, _)| x.number == e.number) {
                    Some(held) if held.1 < at => *held = (*e, at),
                    Some(_) => {}
                    None => seen.push((*e, at)),
                }
            }
        }
        let seen: Vec<EntityState> = seen.into_iter().map(|(e, _)| e).collect();

        // The model list names what each entity is. It's per map and identical across a squad, but
        // it has to come from the same population the frames did: a connection that dropped still
        // holds a perfectly good list for the map it died on.
        let Some(models) = playing().next().map(|b| b.session.models().to_vec()) else {
            return;
        };
        if models.is_empty() {
            return; // still in signon; the list arrives before the first frame does
        }
        self.world.apply_frame(&mut self.game, squad, &seen, &models);
    }

    /// Say the things worth saying while there's no mirror to consume them.
    ///
    /// A squad is N connections to one server, so every bot receives every broadcast — printing each
    /// copy would repeat the whole game log N times. Anything the server says *about the world* is
    /// therefore reported once, by whichever connection speaks for the squad; only what it says *to a
    /// particular bot* is per-bot. Following the lead rather than bot 0 keeps the log running when
    /// bot 0 is the one that got dropped.
    fn observe(&mut self, index: usize, ev: &SvcEvent) {
        let lead = self.bots.iter().position(|b| b.session.signon() != Signon::Disconnected);
        match ev {
            SvcEvent::ServerData(sd) => eprintln!(
                "rtx-client: [{index}] joined {} on {:?} as slot {}{}",
                if sd.gamedir.is_empty() { "qw" } else { &sd.gamedir },
                crate::text::readable(&sd.levelname),
                sd.playernum,
                if sd.spectator { " (spectating)" } else { "" }
            ),
            SvcEvent::Print { text, .. } if lead == Some(index) => eprint!("{}", crate::text::readable(text)),
            SvcEvent::Disconnect => eprintln!("rtx-client: [{index}] dropped by the server"),
            _ => {}
        }
    }

    /// What the run looked like — the numbers that say whether it went well.
    ///
    /// Split the way the state is: what a bot did is per bot, what the world did is said once. The
    /// item and projectile counts used to be printed per bot from each bot's own copy, which was a
    /// tidy way of printing the same numbers N times and claiming they were different.
    fn report(&self) {
        for (i, bot) in self.bots.iter().enumerate() {
            let s = &bot.session;
            eprintln!(
                "rtx-client: [{i}] {:?} map={} rtt={:.0}ms chokes={}",
                s.signon(),
                s.mapname(),
                s.rtt() * 1000.0,
                s.chokes
            );
            // Travel is the honest measure of "is it playing": everything else can look right while
            // the bot stands on its spawn.
            let ent = &self.game.entities[bot.mirror.own()];
            eprintln!(
                "rtx-client:      travelled {:.0}u, at {:.0?}, health {} armor {} frags {}",
                bot.travelled, ent.v.origin, ent.v.health, ent.v.armorvalue, ent.v.frags,
            );
        }
        let (up, waiting, tracked) = self.world.census(&self.game);
        eprintln!("rtx-client: world: items {up} up, {waiting} timed; {tracked} tracked now");
        eprintln!(
            "rtx-client: world: projectiles: {} seen in flight (peak {} at once)",
            self.world.projectiles_seen, self.world.projectiles_peak,
        );
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

    /// A squad is N independent clients; the server tells them apart by name, so they have to
    /// differ. Default naming draws a distinct list name per bot, each under the coloured `bot•`
    /// tag (`\u{e2}\u{ef}\u{f4}\u{85}` = coloured `bot` + the `0x85` dot). An explicit `--name`
    /// numbers a squad and stays wrapped; a lone bot keeps a single label.
    #[test]
    fn a_squad_gets_distinct_names() {
        const TAG: &str = "\u{e2}\u{ef}\u{f4}\u{85}"; // coloured `bot` + dot

        let mut squad = Client::new(Config { bots: 3, ..config() });
        squad.connect().expect("bind");
        let names: Vec<&str> = squad.bots.iter().map(|b| b.session.name()).collect();
        assert!(names.iter().all(|n| n.starts_with(TAG)), "each carries the bot• tag: {names:?}");
        assert_eq!(names.iter().collect::<std::collections::HashSet<_>>().len(), 3, "distinct: {names:?}");

        let mut named = Client::new(Config { bots: 2, name: Some("botto".into()), ..config() });
        named.connect().expect("bind");
        assert_eq!(named.bots[0].session.name(), format!("{TAG}botto1"));
        assert_eq!(named.bots[1].session.name(), format!("{TAG}botto2"));

        let mut solo = Client::new(Config { bots: 1, name: Some("botto".into()), ..config() });
        solo.connect().expect("bind");
        assert_eq!(solo.bots[0].session.name(), format!("{TAG}botto"));
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
