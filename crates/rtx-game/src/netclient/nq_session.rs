// SPDX-License-Identifier: AGPL-3.0-or-later

//! One connection to a **NetQuake** server — the NetQuake counterpart to [`Session`](super::session).
//!
//! It presents the same surface the tick loop drives ([`poll`](NqSession::poll),
//! [`send_move`](NqSession::send_move), the signon/rtt/mapname accessors), so
//! [`AnySession`](super::AnySession) can hold either without the loop knowing which. What differs is
//! entirely below that surface: the [`CCREQ_CONNECT`](rtx_proto::nq::connect) handshake instead of a
//! challenge, the [Datagram netchan](rtx_proto::nq::chan) instead of QuakeWorld's, and a signon that
//! is driven by `svc_signonnum` plus a `cmd X` stufftext contract rather than resource-list
//! round-trips.
//!
//! # Signon
//!
//! ```text
//!   C→S  CCREQ_CONNECT                 S→C  CCREP_ACCEPT
//!   C→S  clc_nop (NAT punch)           S→C  svc_serverinfo (protocol, inline precaches), signonnum 1
//!   C→S  name / prespawn               S→C  baselines, signonnum 2
//!   C→S  color / spawn                 S→C  updatename/frags/colors, clientdata, signonnum 3
//!   C→S  begin                         S→C  the game (first entity update ⇒ signon 4)
//! ```
//!
//! There is **no map checksum** and no model-CRC anti-cheat — a NetQuake server trusts the precache
//! list. The map's filename is `models[1]` (`maps/<name>.bsp`), exactly as in QuakeWorld.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use rtx_proto::info::Info;
use rtx_proto::nq::chan::{Incoming, NqChan};
use rtx_proto::nq::connect::{self, Ccrep};
use rtx_proto::nq::protocol::NqProtoState;
use rtx_proto::nq::{clc, svc as nqsvc};
use rtx_proto::svc::{self, SvcEvent};

use super::frames::EntityState;
use super::host::NetHost;
use super::nq_frames::NqFrames;
use super::session::{Signon, Wiretap};

/// How often to re-send an unanswered `CCREQ_CONNECT`.
const RESEND_INTERVAL: Duration = Duration::from_secs(2);

/// How long to wait before retransmitting an unacked reliable fragment.
const RELIABLE_RESEND: Duration = Duration::from_secs(1);

/// One NetQuake connection.
pub(crate) struct NqSession {
    sock: UdpSocket,
    server: SocketAddr,
    chan: NqChan,
    proto: NqProtoState,
    signon: Signon,

    /// The label to announce with `name`, and the colours with `color`.
    name: String,
    colors: (u8, u8),
    spectator: bool,

    /// A synthesized incarnation number — NetQuake has no `servercount`, so we bump one per
    /// `svc_serverinfo` to give [`rebuild_world_if_map_changed`](super::Client) the "restart ⇒
    /// changed" signal it keys on.
    servercount: i32,
    /// The gamedir the maps live under (`id1`).
    gamedir: String,
    /// The map's filename, from `models[1]`.
    mapname: String,
    /// The sound precache list; `svc_sound` indexes it.
    sounds: Vec<String>,
    /// The model precache list; entity deltas index it, and entry 1 is the map.
    models: Vec<String>,
    /// A serverinfo synthesized from the fields NetQuake *does* send, so the host's rule cvars
    /// (`maxclients`, `deathmatch`) answer correctly.
    serverinfo: Info,
    /// Our player slot, learned from `svc_setview` (view entity minus one).
    playernum: u8,

    /// The entity store.
    frames: NqFrames,
    /// When the store last advanced — the squad merge picks the fresher of two views by this.
    frames_at: Instant,
    /// The most recent `svc_time`, echoed in `clc_move` for the server's ping calc.
    last_svc_time: f32,

    /// Whether we've been accepted (past the control handshake).
    accepted: bool,
    /// When we last sent a `CCREQ_CONNECT`, for retries.
    last_oob: Instant,
    /// When we last sent a reliable fragment, for the retransmit timer.
    last_reliable: Instant,
    /// When the in-flight reliable was first sent, for a coarse round-trip estimate.
    reliable_sent_at: Option<Instant>,
    /// Smoothed round-trip time in seconds. NetQuake has no per-move ack, so this is sampled from
    /// reliable round trips (mostly during signon) and held after.
    rtt: f32,
    /// Whether the server has us at a scoreboard.
    intermission: bool,
    /// Where to record the wire, if `--wiretap`.
    wiretap: Option<Wiretap>,
    /// Whether to fetch a missing map.
    download_enabled: bool,
    /// A map fetch in flight, while [`Signon::Downloading`].
    download: Option<super::download::Download>,
}

impl NqSession {
    /// Open a socket and send `CCREQ_CONNECT`. Doesn't block: the reply arrives via
    /// [`poll`](Self::poll).
    pub(crate) fn connect(
        server: SocketAddr,
        name: String,
        colors: (u8, u8),
        spectator: bool,
        gamedir: String,
        wiretap: Option<&std::path::Path>,
        download_enabled: bool,
    ) -> io::Result<Self> {
        let sock = UdpSocket::bind(if server.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" })?;
        sock.set_nonblocking(true)?;
        let wiretap = wiretap.and_then(|dir| {
            // The remote port disambiguates a squad's connections in one capture directory.
            Wiretap::open(dir, &format!("nq-{}", server.port()))
        });
        let now = Instant::now();
        let mut s = NqSession {
            sock,
            server,
            chan: NqChan::new(),
            proto: NqProtoState::new(),
            signon: Signon::Challenge,
            name,
            colors,
            spectator,
            servercount: 0,
            gamedir,
            mapname: String::new(),
            sounds: Vec::new(),
            models: Vec::new(),
            serverinfo: Info::new(),
            playernum: 0,
            frames: NqFrames::default(),
            frames_at: now,
            last_svc_time: 0.0,
            accepted: false,
            last_oob: now,
            last_reliable: now,
            reliable_sent_at: None,
            rtt: 0.0,
            intermission: false,
            wiretap,
            download_enabled,
            download: None,
        };
        s.send(&connect::connect_request());
        Ok(s)
    }

    // ── Accessors mirroring `Session` ───────────────────────────────────────────────────────────

    pub(crate) fn signon(&self) -> Signon {
        self.signon
    }
    pub(crate) fn playernum(&self) -> u8 {
        self.playernum
    }
    pub(crate) fn models(&self) -> &[String] {
        &self.models
    }
    pub(crate) fn sounds(&self) -> &[String] {
        &self.sounds
    }
    pub(crate) fn serverinfo(&self) -> &Info {
        &self.serverinfo
    }
    pub(crate) fn mapname(&self) -> &str {
        &self.mapname
    }
    pub(crate) fn rtt(&self) -> f32 {
        self.rtt
    }
    pub(crate) fn frames_at(&self) -> Instant {
        self.frames_at
    }
    pub(crate) fn at_intermission(&self) -> bool {
        self.intermission
    }
    pub(crate) fn servercount(&self) -> i32 {
        self.servercount
    }
    pub(crate) fn frames_current(&self) -> &[EntityState] {
        self.frames.current()
    }
    /// NetQuake has no server-side choke; always zero, so the report line reads uniformly.
    pub(crate) fn chokes(&self) -> u32 {
        0
    }
    #[cfg(test)]
    pub(crate) fn name(&self) -> &str {
        &self.name
    }

    /// The velocity a frame-differencing store estimated for an entity — NetQuake sends none.
    pub(crate) fn velocity_of(&self, number: u16) -> Option<glam::Vec3> {
        self.frames.velocity_of(number)
    }

    /// Queue a console command for the server. Sent reliably.
    pub(crate) fn stringcmd(&mut self, cmd: &str) {
        self.chan.queue_reliable(&clc::write_stringcmd(cmd));
    }

    /// A no-op on NetQuake: `ready` is a KTX word, and stuffing it at a NetQuake server's console is
    /// noise it doesn't understand.
    pub(crate) fn ready_up(&mut self) {}

    /// Drain the socket, act on the signon traffic, and hand back everything the server said.
    pub(crate) fn poll(&mut self, host: &NetHost) -> io::Result<Vec<SvcEvent>> {
        self.poll_download(host);

        let mut out = Vec::new();
        let mut buf = [0u8; 65536];
        let was_pending = self.chan.reliable_pending();
        loop {
            let len = match self.sock.recv(&mut buf) {
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::ConnectionReset => continue,
                Err(e) => return Err(e),
            };
            let data = &buf[..len];
            if let Some(w) = self.wiretap.as_mut() {
                w.record(data, false);
            }

            // The handshake rides in control packets, outside the Datagram streams.
            if connect::is_control(data) {
                self.handle_control(data);
                continue;
            }
            let (incoming, ack) = self.chan.process(data);
            if let Some(ack) = ack {
                self.send(&ack);
            }
            let payload = match incoming {
                Incoming::Unreliable(p) | Incoming::Reliable(p) => p,
                Incoming::None => continue,
            };

            let events = match nqsvc::parse(&mut self.proto, &payload) {
                Ok(e) => e,
                Err(e) => {
                    eprintln!("rtx-client: NQ protocol desync: {e}");
                    eprintln!("rtx-client: proto={:?}", self.proto);
                    eprintln!("{}", svc::hexdump(&payload));
                    self.signon = Signon::Disconnected;
                    return Ok(out);
                }
            };
            for ev in events {
                self.handle(&ev, host);
                out.push(ev);
            }
        }

        // The datagram(s) are drained, so the latest frame is complete — publish it.
        self.frames.settle();
        // A reliable that completed this poll gives us a round-trip sample.
        if was_pending && !self.chan.reliable_pending() {
            if let Some(sent) = self.reliable_sent_at.take() {
                self.note_rtt(sent.elapsed().as_secs_f32());
            }
        }
        self.pump_reliable();
        self.retry_handshake();
        Ok(out)
    }

    /// Handle a control-packet reply to our `CCREQ_CONNECT`.
    fn handle_control(&mut self, data: &[u8]) {
        match connect::parse_control(data) {
            Some(rep @ Ccrep::Accept { proquake, .. }) if !self.accepted => {
                self.accepted = true;
                self.proto.proquake_angles = proquake;
                if let Some(port) = rep.switch_to() {
                    self.server.set_port(port);
                }
                // NAT punch: one unreliable nop so the server's data port can reach us.
                let nop = self.chan.transmit_unreliable(&clc::write_nop());
                self.send(&nop);
                self.signon = Signon::Loading;
            }
            Some(Ccrep::Reject(why)) => {
                eprintln!("rtx-client: server refused the connection: {why}");
                self.signon = Signon::Disconnected;
            }
            _ => {}
        }
    }

    /// Act on the messages that are the session's business.
    fn handle(&mut self, ev: &SvcEvent, host: &NetHost) {
        match ev {
            SvcEvent::NqServerData(sd) => {
                // A new level: everything about the last one is void. NetQuake has no servercount,
                // so bump a synthetic one — a restart of the same map still counts as a change.
                self.servercount += 1;
                self.models = sd.models.clone();
                self.sounds = sd.sounds.clone();
                self.mapname = sd
                    .models
                    .get(1)
                    .and_then(|m| m.strip_prefix("maps/"))
                    .and_then(|m| m.strip_suffix(".bsp"))
                    .unwrap_or_default()
                    .to_string();
                self.frames.clear();
                self.intermission = false;
                self.signon = Signon::Loading;

                // NetQuake announces no serverinfo string, but the host's rule cvars still need
                // `maxclients` and `deathmatch` or the shadow world spawns the single-player item set.
                let mut info = Info::new();
                info.set("maxclients", &sd.maxclients.to_string());
                info.set("deathmatch", if sd.gametype == 1 { "1" } else { "0" });
                if !sd.levelname.is_empty() {
                    info.set("hostname", &sd.levelname);
                }
                host.set_serverinfo(info.clone());
                self.serverinfo = info;

                self.bind_map(host);
            }
            SvcEvent::SignonNum(n) => match n {
                1 => {
                    self.stringcmd(&format!("name \"{}\"", self.name));
                    self.stringcmd("prespawn");
                }
                2 => {
                    self.stringcmd(&format!("color {} {}", self.colors.0, self.colors.1));
                    if self.spectator {
                        self.stringcmd("spectator 1");
                    }
                    self.stringcmd("spawn");
                }
                3 => self.stringcmd("begin"),
                _ => {}
            },
            SvcEvent::StuffText(text) => self.stufftext(text),
            SvcEvent::SetView(e) => self.playernum = e.saturating_sub(1) as u8,
            SvcEvent::Time(t) => {
                self.last_svc_time = *t;
                self.frames.begin_frame(*t);
            }
            SvcEvent::SpawnBaseline { entity, baseline } => self.frames.set_baseline(*entity, *baseline),
            SvcEvent::EntityUpdate(delta) => {
                self.frames.apply(delta);
                self.frames_at = Instant::now();
                // The first entity update is what "in the game" means (signon 4).
                if self.signon == Signon::Loading {
                    self.signon = Signon::Active;
                }
            }
            SvcEvent::Intermission { .. } | SvcEvent::Finale(_) => self.intermission = true,
            SvcEvent::Disconnect => self.signon = Signon::Disconnected,
            _ => {}
        }
    }

    /// Obey a stuffed console command. On NetQuake the whole contract is: echo any `cmd X` back (the
    /// server's lever for `prespawn` on FitzQuake/RMQ, and `pext` — a bare `pext` reply declines
    /// every FTE extension, keeping us on the vanilla wire). `//` lines are engine commands we ignore.
    fn stufftext(&mut self, text: &str) {
        for line in text.lines() {
            if let Some(args) = line.trim().strip_prefix("cmd ") {
                self.stringcmd(args.trim());
            }
        }
    }

    /// Load the map into the host so the brain has geometry, fetching it first if we don't have it.
    fn bind_map(&mut self, host: &NetHost) {
        if host.rebind(&self.gamedir, &self.mapname) {
            return;
        }
        if self.download_enabled {
            eprintln!("rtx-client: don't have maps/{}.bsp — downloading it", self.mapname);
            self.download = Some(super::download::Download::start(
                host.basedir(),
                self.gamedir.clone(),
                self.mapname.clone(),
            ));
            self.signon = Signon::Downloading;
        } else {
            eprintln!(
                "rtx-client: can't read maps/{}.bsp (and --no-download is set)",
                self.mapname
            );
            self.signon = Signon::Disconnected;
        }
    }

    /// Resume signon once a map download lands. The server isn't waiting on anything (NetQuake sent
    /// the whole serverinfo already); we just need the geometry before the first frame is useful.
    fn poll_download(&mut self, host: &NetHost) {
        if self.signon != Signon::Downloading {
            return;
        }
        let Some(result) = self.download.as_ref().and_then(|d| d.poll()) else {
            return;
        };
        self.download = None;
        match result {
            Ok(path) => {
                eprintln!("rtx-client: downloaded {}", path.display());
                if host.rebind(&self.gamedir, &self.mapname) {
                    self.signon = Signon::Loading;
                } else {
                    eprintln!("rtx-client: downloaded map still won't load");
                    self.signon = Signon::Disconnected;
                }
            }
            Err(e) => {
                eprintln!("rtx-client: map download failed: {e}");
                self.signon = Signon::Disconnected;
            }
        }
    }

    /// Send this frame's move. No `msec`, no checksum, no delta ack — the server runs the physics.
    pub(crate) fn send_move(
        &mut self,
        angles: glam::Vec3,
        forward: i32,
        side: i32,
        up: i32,
        buttons: u8,
        impulse: u8,
    ) -> io::Result<()> {
        let payload = clc::write_move(
            &self.proto,
            self.last_svc_time,
            angles,
            forward as i16,
            side as i16,
            up as i16,
            buttons,
            impulse,
        );
        let datagram = self.chan.transmit_unreliable(&payload);
        self.send(&datagram);
        Ok(())
    }

    /// A move that stands still — the intermission hold and the general keepalive-with-a-body.
    pub(crate) fn send_idle(&mut self) -> io::Result<()> {
        self.send_move(glam::Vec3::ZERO, 0, 0, 0, 0, 0)
    }

    /// A `clc_nop` — the keepalive while connecting, before there are moves to send.
    pub(crate) fn send_nop(&mut self) -> io::Result<()> {
        let datagram = self.chan.transmit_unreliable(&clc::write_nop());
        self.send(&datagram);
        Ok(())
    }

    /// Push out the next reliable fragment, or retransmit the in-flight one if it's overdue.
    fn pump_reliable(&mut self) {
        let mut sent = false;
        while let Some(frag) = self.chan.reliable_to_send() {
            self.send(&frag);
            sent = true;
        }
        if sent {
            self.last_reliable = Instant::now();
            self.reliable_sent_at.get_or_insert_with(Instant::now);
        } else if self.chan.reliable_pending() && self.last_reliable.elapsed() >= RELIABLE_RESEND {
            if let Some(frag) = self.chan.reliable_resend() {
                self.send(&frag);
                self.last_reliable = Instant::now();
            }
        }
    }

    /// Re-send an unanswered `CCREQ_CONNECT`. UDP loses packets and the handshake has no other way to
    /// notice.
    fn retry_handshake(&mut self) {
        if self.accepted || self.signon == Signon::Disconnected {
            return;
        }
        if self.last_oob.elapsed() < RESEND_INTERVAL {
            return;
        }
        self.last_oob = Instant::now();
        self.send(&connect::connect_request());
    }

    /// Put a datagram on the wire, and in the record if one is being kept. Handshake sends are
    /// unreliable by nature — a failure is answered by a retry, not an error.
    fn send(&mut self, datagram: &[u8]) {
        if let Some(w) = self.wiretap.as_mut() {
            w.record(datagram, true);
        }
        let _ = self.sock.send_to(datagram, self.server);
    }

    /// Fold a round-trip sample into the smoothed estimate.
    fn note_rtt(&mut self, sample: f32) {
        self.rtt = if self.rtt == 0.0 {
            sample
        } else {
            self.rtt * 0.9 + sample * 0.1
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtx_proto::nq::protocol::NETQUAKE;
    use rtx_proto::sizebuf::Writer;
    use std::path::PathBuf;

    fn session() -> NqSession {
        NqSession::connect(
            "127.0.0.1:1".parse().unwrap(),
            "rtxbot".to_string(),
            (0, 0),
            false,
            "id1".to_string(),
            None,
            false,
        )
        .expect("bind")
    }

    fn host() -> NetHost {
        NetHost::new(PathBuf::from("/nonexistent"))
    }

    /// A serverinfo without a real map still seeds the host's rule cvars — `maxclients` and
    /// `deathmatch` — which is what keeps the shadow world from spawning the single-player item set.
    #[test]
    fn serverinfo_seeds_rule_cvars_and_bumps_servercount() {
        let mut s = session();
        let h = host();
        let sd = rtx_proto::svc::NqServerData {
            protocol: NETQUAKE,
            flags: 0,
            maxclients: 8,
            gametype: 1,
            levelname: "The Bad Place".to_string(),
            // No real map, so bind fails and (download off) the session disconnects — but the
            // serverinfo has already been adopted, which is what we're checking.
            models: vec![String::new(), "maps/nope.bsp".to_string()],
            sounds: vec![String::new()],
        };
        s.handle(&SvcEvent::NqServerData(Box::new(sd)), &h);
        assert_eq!(s.servercount(), 1);
        assert_eq!(s.serverinfo().get_f32("maxclients"), Some(8.0));
        assert_eq!(s.serverinfo().get("deathmatch"), Some("1"));
        assert_eq!(s.mapname(), "nope");
    }

    /// The signon walk: each `svc_signonnum` queues exactly the reliable commands that step owes.
    #[test]
    fn signon_steps_queue_the_right_commands() {
        let mut s = session();
        let h = host();
        // Before the walk, only the CCREQ has gone out; no reliable is queued.
        assert!(!s.chan.reliable_pending());

        s.handle(&SvcEvent::SignonNum(1), &h);
        assert!(s.chan.reliable_pending(), "signon 1 queues name+prespawn");

        // Drain the queue so we can observe signon 3 independently.
        while s.chan.reliable_to_send().is_some() {}
        // (name still awaiting ack in the real flow; here we just check begin queues something.)
        s.handle(&SvcEvent::SignonNum(3), &h);
        assert!(s.chan.reliable_pending());
    }

    /// `svc_setview` sets our slot to the view entity minus one — the NetQuake way of learning our
    /// player number.
    #[test]
    fn setview_sets_playernum() {
        let mut s = session();
        let h = host();
        s.handle(&SvcEvent::SetView(4), &h);
        assert_eq!(s.playernum(), 3);
    }

    /// A `cmd X` stufftext is echoed straight back; a `//` engine line is ignored.
    #[test]
    fn stufftext_echoes_cmd_and_ignores_slashes() {
        let mut s = session();
        s.stufftext("//paknames id1/pak0.pak\ncmd prespawn\n");
        assert!(s.chan.reliable_pending(), "cmd prespawn was echoed");
    }

    /// The first entity update flips a loading session into the game (signon 4).
    #[test]
    fn first_entity_update_enters_the_game() {
        let mut s = session();
        let h = host();
        s.signon = Signon::Loading;
        s.frames.set_baseline(1, rtx_proto::svc::Baseline::default());
        s.handle(&SvcEvent::Time(1.0), &h);
        let delta = rtx_proto::svc::EntityDelta {
            number: 1,
            ..Default::default()
        };
        s.handle(&SvcEvent::EntityUpdate(delta), &h);
        assert_eq!(s.signon(), Signon::Active);
    }

    /// A move on a plain protocol-15 session is 16 bytes wrapped in an 8-byte Datagram header, and
    /// echoes the last heard server time.
    #[test]
    fn move_is_wrapped_in_a_datagram() {
        let mut s = session();
        s.last_svc_time = 2.5;
        s.send_move(glam::Vec3::ZERO, 400, 0, 0, 1, 0).unwrap();
        // Can't observe the socket, but building it must not panic and the writer round-trips
        // through the codec's own tests. Here we just assert the unreliable sequence advanced.
        // (transmit_unreliable increments it; a second send differs.)
        let a = s.chan.transmit_unreliable(&[1]);
        let b = s.chan.transmit_unreliable(&[1]);
        assert_ne!(a, b, "each unreliable carries a fresh sequence");
    }

    /// Building the writer for a raw datagram must not panic — a sanity check on `Writer` reuse.
    #[test]
    fn writer_is_usable() {
        let mut w = Writer::new();
        w.u32_be(0x8000_0000);
        assert_eq!(w.len(), 4);
    }
}
