// SPDX-License-Identifier: AGPL-3.0-or-later

//! One connection to a QuakeWorld server: the handshake, the signon, and the move stream.
//!
//! # Signon is a conversation, not a sequence
//!
//! A client doesn't march through connecting; it answers. The server drives most of it by stuffing
//! console commands at us, and a client that ignores those never gets in:
//!
//! ```text
//!   C→S  getchallenge                    S→C  c<challenge> + extension offers
//!   C→S  connect 28 <qport> …            S→C  j
//!   C→S  new                             S→C  svc_serverdata, stufftext `fullserverinfo …`
//!   C→S  soundlist <count> 0             S→C  svc_soundlist … (chunked)
//!   C→S  modellist <count> 0             S→C  svc_modellist … (chunked)
//!   C→S  prespawn <count> 0 <checksum>   S→C  baselines, stufftext `cmd prespawn <count> <n>` …
//!   C→S  spawn <count> 0                 S→C  stats, players, stufftext `skins`
//!   C→S  begin <count>                   S→C  the game
//! ```
//!
//! The map checksum in `prespawn` is the one step that can't be faked: get it wrong and the server
//! silently drops the connection mid-signon. The map's *filename* isn't in `svc_serverdata` either
//! — a client learns it from `modellist[0]`, because `levelname` is the display title and is often
//! empty.
//!
//! # What this owns, and what it doesn't
//!
//! It owns the wire: the netchan, the negotiated protocol, the signon state, the entity snapshot
//! ring, and the last three usercmds. It does **not** own the world — parsed events go up to the
//! caller, which decides what they mean. That split is why this module can be tested against a
//! recorded session with no bot in sight.
//!
//! One thing it is strict about: `msec`. See [`Session::send_move`].

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use rtx_proto::info::{Info, UserinfoBuilder};
use rtx_proto::netchan::Netchan;
use rtx_proto::protocol::{self, magic, ProtoState};
use rtx_proto::svc::{self, DownloadMessage, SvcEvent, Usercmd};
use rtx_proto::{checksum, clc, oob};

use super::frames::{Applied, EntityState, Frames};
use super::host::NetHost;
use crate::host::ClientHost;

/// How far through connecting we are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Signon {
    /// Asked for a challenge.
    Challenge,
    /// Sent `connect`, waiting to be let in.
    Connecting,
    /// In, and working through the signon exchange.
    Loading,
    /// Signon is paused while we fetch a map the machine doesn't have. The netchan stays alive on
    /// the tick loop's nops; when the download lands, signon resumes at `prespawn`.
    Downloading,
    /// Playing.
    Active,
    /// The map is changing; everything we know is stale until the next `svc_serverdata`.
    Changing,
    /// The server dropped us.
    Disconnected,
}

/// How often to retry an unanswered `getchallenge` / `connect`.
const RESEND_INTERVAL: Duration = Duration::from_secs(5);

/// What accepting one server download message requires once the mutable transfer borrow is over.
enum DownloadOutcome {
    Waiting,
    NextLegacyBlock,
    Complete {
        path: std::path::PathBuf,
        chunked: bool,
        cookie: u32,
    },
    Failed(String),
}

/// The CRCs of stock `progs/player.mdl` and `progs/eyes.mdl`.
///
/// A server asks a client to prove it hasn't swapped the player model for something easier to see —
/// an old anti-cheat, and KTX still warns without an answer. The models normally live inside `pak0`,
/// which this client doesn't read, so these are hardcoded. They're not folklore: both were computed
/// with [`crc::block`](rtx_proto::crc::block) over the files from two separate id installs, and
/// qualia hardcodes the same 33168 — it forges the stock value when it loads its MD5 replacement
/// player model, precisely so the server still sees a vanilla client.
const STOCK_PMODEL_CRC: u16 = 33168;
const STOCK_EMODEL_CRC: u16 = 6967;

/// A record of every datagram a connection saw, written as it goes.
///
/// The point is fixtures. `rtx-proto`'s parser tests replay a capture directory and demand that every
/// server datagram parses with no unknown opcode, and the captures they've had until now came from a
/// proxy sat in front of *ezquake* — a real client, but not this one. A bot's own session is the
/// traffic that actually matters, and the interesting packets (a rocket in flight, a lift moving, a
/// nail volley) only exist while a bot is playing.
///
/// Named to `rtx-proto`'s contract — `NNNNNN-{c2s,s2c}.bin` — so a directory written here is one
/// `RTX_TEST_QW_CAPTURE` can be pointed straight at.
pub(crate) struct Wiretap {
    dir: std::path::PathBuf,
    n: usize,
}

impl Wiretap {
    /// Open a wiretap on a per-connection subdirectory, so a squad's interleaved datagrams don't
    /// mix into one unreplayable capture. `None` if the directory can't be made.
    pub(crate) fn open(base: &std::path::Path, tag: &str) -> Option<Wiretap> {
        let dir = base.join(tag);
        std::fs::create_dir_all(&dir)
            .map_err(|e| eprintln!("rtx-client: can't record to {}: {e}", dir.display()))
            .ok()
            .map(|()| Wiretap { dir, n: 0 })
    }

    /// Record one datagram. Failures are dropped on purpose: a full disk is not a reason to break
    /// off a game, and the capture is a diagnostic, not the point of the run.
    pub(crate) fn record(&mut self, data: &[u8], to_server: bool) {
        let name = format!("{:06}-{}.bin", self.n, if to_server { "c2s" } else { "s2c" });
        self.n += 1;
        let _ = std::fs::write(self.dir.join(name), data);
    }
}

/// One connection.
pub(crate) struct Session {
    sock: UdpSocket,
    server: SocketAddr,
    chan: Netchan,
    proto: ProtoState,
    signon: Signon,
    userinfo: UserinfoBuilder,

    /// Echoed back in every signon command so the server can tell a stale signon from a current one.
    servercount: i32,
    /// The gamedir the server is running.
    gamedir: String,
    /// The map's filename, learned from `modellist[0]`.
    mapname: String,
    /// The sound list, for turning a `svc_sound` index into "who fired what".
    sounds: Vec<String>,
    /// The model list; entry 0 is the map.
    models: Vec<String>,
    /// The server's info string.
    serverinfo: Info,
    /// Our player slot.
    playernum: u8,

    /// The entity snapshot ring, and the delta chain.
    pub(crate) frames: Frames,
    /// When the snapshot last advanced. A squad merges several views of one world, and this is what
    /// says which of two disagreeing views is the current one — a connection that has gone quiet
    /// still has a perfectly well-formed snapshot of a world that has since moved on.
    frames_at: Instant,
    /// The last three usercmds, oldest first — every move packet carries all three.
    cmds: [Usercmd; 3],

    /// The challenge we're answering.
    challenge: i32,
    /// When we last sent an out-of-band packet, for retries.
    last_oob: Instant,
    /// When we last sent a move, for the `msec` the next one claims.
    last_move: Instant,
    /// When each recent packet went out, keyed by its sequence. The server acks a sequence; the
    /// gap between then and now is the round trip.
    sent_at: Vec<Instant>,
    /// Smoothed round-trip time in seconds — how far behind the world we are.
    rtt: f32,
    /// Frames the server withheld to stay inside our rate.
    pub(crate) chokes: u32,
    /// Whether we've told this level's server we're ready to play. See [`Session::ready_up`].
    readied: bool,
    /// Whether the server has us at a scoreboard rather than in the game. Set by `svc_intermission`
    /// and cleared by the next `svc_serverdata` — which is the whole of its lifecycle, because
    /// there is no "intermission over" message: the level reload *is* the end of it.
    intermission: bool,
    /// Where to record what crosses the wire, if anyone asked (`--wiretap`).
    wiretap: Option<Wiretap>,
    /// Whether to fetch a missing map rather than give up on it. Off (`--no-download`) restores the
    /// old behaviour: a map we don't have is a connection we can't make.
    download_enabled: bool,
    /// The first attempt: an HTTP worker, while [`Signon::Downloading`].
    download: Option<super::download::Download>,
    /// The fallback transfer over this connection, after HTTP failed.
    server_download: Option<super::download::ServerDownload>,
    /// Non-zero identifier echoed by FTE's out-of-band chunks. Incremented per server transfer so
    /// a late block from the last map cannot satisfy the next one.
    download_cookie: u32,
}

// A session answers two callers: the tick loop, which drives it, and the world mirror, which asks
// what it saw.
impl Session {
    /// Open a socket and ask for a challenge. Doesn't block: the reply arrives via
    /// [`poll`](Self::poll).
    pub(crate) fn connect(
        server: SocketAddr,
        userinfo: UserinfoBuilder,
        qport: u16,
        wiretap: Option<&std::path::Path>,
        download_enabled: bool,
    ) -> io::Result<Self> {
        let sock = UdpSocket::bind(if server.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" })?;
        sock.set_nonblocking(true)?;
        // Each connection gets its own directory: a squad's datagrams interleave, and a capture that
        // mixes two netchans' sequences replays as gibberish.
        let wiretap = wiretap.map(|dir| dir.join(format!("qport-{qport:04x}"))).and_then(|dir| {
            std::fs::create_dir_all(&dir)
                .map_err(|e| eprintln!("rtx-client: can't record to {}: {e}", dir.display()))
                .ok()
                .map(|()| Wiretap { dir, n: 0 })
        });
        let now = Instant::now();
        let s = Session {
            sock,
            server,
            chan: Netchan::new(qport),
            proto: ProtoState::new(),
            signon: Signon::Challenge,
            userinfo,
            servercount: 0,
            gamedir: "qw".to_string(),
            mapname: String::new(),
            sounds: Vec::new(),
            models: Vec::new(),
            serverinfo: Info::new(),
            playernum: 0,
            frames: Frames::default(),
            frames_at: now,
            cmds: [Usercmd::default(); 3],
            sent_at: vec![now; super::frames::UPDATE_BACKUP],
            challenge: 0,
            last_oob: now - RESEND_INTERVAL, // fire immediately
            last_move: now,
            rtt: 0.0,
            chokes: 0,
            readied: false,
            intermission: false,
            wiretap,
            download_enabled,
            download: None,
            server_download: None,
            download_cookie: 0,
        };
        Ok(s)
    }

    /// How far through connecting we are.
    pub(crate) fn signon(&self) -> Signon {
        self.signon
    }

    /// Our player slot, once the server has told us.
    pub(crate) fn playernum(&self) -> u8 {
        self.playernum
    }

    /// The model list — entity deltas carry an index into this.
    pub(crate) fn models(&self) -> &[String] {
        &self.models
    }

    /// The sound list — `svc_sound` carries an index into this, and the name behind it is what says
    /// whether the bot just heard a rocket launcher or a footstep.
    pub(crate) fn sounds(&self) -> &[String] {
        &self.sounds
    }

    /// A move that stands still: the keepalive, with a body.
    ///
    /// A connected client that stops sending is a client the server times out, so there is no such
    /// thing as "nothing to send" — only nothing to say.
    pub(crate) fn send_idle(&mut self) -> io::Result<()> {
        self.send_move(glam::Vec3::ZERO, 0, 0, 0, 0, 0)
    }

    /// The server's info string.
    pub(crate) fn serverinfo(&self) -> &Info {
        &self.serverinfo
    }

    /// The map's filename, without `maps/` or `.bsp`.
    pub(crate) fn mapname(&self) -> &str {
        &self.mapname
    }

    /// The name this bot connects under — conchars and all.
    #[cfg(test)]
    pub(crate) fn name(&self) -> &str {
        &self.userinfo.name
    }

    /// Smoothed round-trip time, in seconds.
    pub(crate) fn rtt(&self) -> f32 {
        self.rtt
    }

    /// When this connection's entity snapshot last advanced.
    pub(crate) fn frames_at(&self) -> Instant {
        self.frames_at
    }

    /// The entities of the current frame — the slice the world mirror consumes. An accessor rather
    /// than a public field so [`AnySession`](super::AnySession) can front both protocols uniformly.
    pub(crate) fn frames_current(&self) -> &[EntityState] {
        self.frames.current()
    }

    /// Frames the server withheld to stay inside our rate.
    pub(crate) fn chokes(&self) -> u32 {
        self.chokes
    }

    /// Whether the server has us at a scoreboard rather than in the game.
    pub(crate) fn at_intermission(&self) -> bool {
        self.intermission
    }

    /// The server's incarnation number, bumped every time it (re)starts a level.
    ///
    /// It's what distinguishes a *restart* from nothing having happened: KTX ends a match by
    /// reloading the same map, so the name is no evidence at all and this is the only thing that
    /// changes.
    pub(crate) fn servercount(&self) -> i32 {
        self.servercount
    }

    /// Queue a console command for the server (`say`, `kill`, `ready`, …).
    pub(crate) fn stringcmd(&mut self, cmd: &str) {
        self.chan.queue_reliable(&clc::write_stringcmd(cmd));
    }

    /// If the HTTP attempt is running, see whether it finished. Any failure starts an in-protocol
    /// QuakeWorld transfer on the same connection; the server messages drive that second phase.
    ///
    /// The server is still holding at `prespawn`, waiting for us; the nops the tick loop sends while
    /// downloading keep the connection alive and carry chunk requests once metadata arrives.
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
                eprintln!("rtx-client: downloaded {} over HTTP", path.display());
                self.resume_with_downloaded_map(host);
            }
            Err(e) => {
                eprintln!("rtx-client: HTTP map download failed ({e}); asking the server");
                self.start_server_download(host);
            }
        }
    }

    fn start_server_download(&mut self, host: &NetHost) {
        self.server_download = None;
        self.download_cookie = self.download_cookie.wrapping_add(1);
        if self.download_cookie == 0 {
            self.download_cookie = 1;
        }
        let transfer = match super::download::ServerDownload::new(
            host.basedir(),
            &self.gamedir,
            &self.mapname,
            self.download_cookie,
        ) {
            Ok(transfer) => transfer,
            Err(e) => {
                self.fail_server_download(e);
                return;
            }
        };
        let name = transfer.expected_name().to_string();
        eprintln!(
            "rtx-client: requesting {name} from the server ({})",
            if self.proto.has_fte(protocol::fte::CHUNKEDDOWNLOADS) {
                "FTE chunked"
            } else {
                "regular QuakeWorld"
            }
        );
        self.server_download = Some(transfer);
        self.stringcmd(&format!("download \"{name}\""));
    }

    fn handle_server_download(&mut self, message: &DownloadMessage, host: &NetHost) {
        if self.signon != Signon::Downloading {
            return;
        }
        let Some(transfer) = self.server_download.as_mut() else { return };
        let outcome = match message {
            DownloadMessage::LegacyBlock { percent, data } => {
                if !transfer.legacy_reply_ready(self.chan.incoming_acknowledged, self.chan.can_reliable()) {
                    DownloadOutcome::Waiting
                } else {
                    let cookie = transfer.cookie();
                    match transfer.receive_legacy(data, *percent) {
                        Ok(Some(path)) => DownloadOutcome::Complete { path, chunked: false, cookie },
                        Ok(None) => DownloadOutcome::NextLegacyBlock,
                        Err(e) => DownloadOutcome::Failed(e),
                    }
                }
            }
            DownloadMessage::LegacyError(code) => {
                DownloadOutcome::Failed(format!("server rejected the regular download (error {code})"))
            }
            DownloadMessage::ChunkedStart { name, size } => match size {
                Ok(size) => match transfer.begin_chunked(name, *size, Instant::now()) {
                    Ok(()) => DownloadOutcome::Waiting,
                    Err(e) => DownloadOutcome::Failed(e),
                },
                Err(code) => DownloadOutcome::Failed(format!(
                    "server rejected the FTE chunked download of {name:?} (error {code})"
                )),
            },
            DownloadMessage::ChunkedBlock { chunk, data } => {
                let cookie = transfer.cookie();
                match transfer.receive_chunk(*chunk, data.as_ref()) {
                    Ok(Some(path)) => DownloadOutcome::Complete { path, chunked: true, cookie },
                    Ok(None) => DownloadOutcome::Waiting,
                    Err(e) => DownloadOutcome::Failed(e),
                }
            }
        };
        self.apply_download_outcome(outcome, host);
    }

    /// Accept an OOB chunk only for the active transfer. The source address was checked in `poll`;
    /// the cookie separates this file from late packets belonging to an earlier one.
    fn receive_server_chunk(
        &mut self,
        cookie: u32,
        chunk: u32,
        data: &[u8; svc::DOWNLOAD_CHUNK_SIZE],
        host: &NetHost,
    ) {
        let Some(transfer) = self.server_download.as_mut() else { return };
        if transfer.cookie() != cookie || !transfer.is_chunked() {
            return;
        }
        let outcome = match transfer.receive_chunk(chunk, data) {
            Ok(Some(path)) => DownloadOutcome::Complete { path, chunked: true, cookie },
            Ok(None) => DownloadOutcome::Waiting,
            Err(e) => DownloadOutcome::Failed(e),
        };
        self.apply_download_outcome(outcome, host);
    }

    fn apply_download_outcome(&mut self, outcome: DownloadOutcome, host: &NetHost) {
        match outcome {
            DownloadOutcome::Waiting => {}
            DownloadOutcome::NextLegacyBlock => {
                self.stringcmd("nextdl");
                if let Some(transfer) = self.server_download.as_mut() {
                    transfer.mark_legacy_request_queued();
                }
            }
            DownloadOutcome::Complete { path, chunked, cookie } => {
                // FTE and mvdsv use this as their compatible end-of-transfer cleanup command.
                if chunked {
                    self.stringcmd(&format!("nextdl -1 100 {cookie}"));
                }
                self.server_download = None;
                eprintln!(
                    "rtx-client: downloaded {} from the server ({})",
                    path.display(),
                    if chunked { "FTE chunked" } else { "regular QuakeWorld" }
                );
                self.resume_with_downloaded_map(host);
            }
            DownloadOutcome::Failed(error) => self.fail_server_download(error),
        }
    }

    fn fail_server_download(&mut self, error: String) {
        eprintln!("rtx-client: server map download failed: {error}");
        self.download = None;
        self.server_download = None;
        self.signon = Signon::Disconnected;
    }

    /// Tell the server we're willing to play, once per level.
    ///
    /// KTX won't start a match until everyone has said so, so a squad that never says it is a squad
    /// standing in a warmup forever — which is most of what "play on a KTX server" means. There's
    /// nothing to detect and nothing to parse: `ready` is **idempotent** in KTX (`PlayerReady`
    /// answers an already-ready player with "type break to unready yourself" and changes nothing),
    /// and a server that has never heard of it says so and carries on. So the honest implementation
    /// is to say it once and not be clever.
    ///
    /// Once per *level*, though, not once per connection: KTX ignores a `ready` sent during
    /// intermission, and ends a match by reloading the map — which bumps `servercount` and re-arms
    /// this, so the next match gets its own.
    pub(crate) fn ready_up(&mut self) {
        if self.readied || self.signon != Signon::Active {
            return;
        }
        self.readied = true;
        self.stringcmd("ready");
    }

    /// Drain the socket and hand back everything the server said.
    ///
    /// Signon traffic is acted on here — it's this module's business, and the caller shouldn't have
    /// to know that `skins` means "say `begin`". Everything is still passed up, because the world
    /// mirror needs `svc_serverdata` too.
    pub(crate) fn poll(&mut self, host: &NetHost) -> io::Result<Vec<SvcEvent>> {
        self.poll_download(host);

        let mut out = Vec::new();
        let mut buf = [0u8; 8192];
        loop {
            let (len, from) = match self.sock.recv_from(&mut buf) {
                Ok(received) => received,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                // A closed port answers with ICMP, which surfaces here on some platforms. It isn't
                // fatal — the server may simply not be up yet, and we're about to retry anyway.
                Err(e) if e.kind() == io::ErrorKind::ConnectionReset => continue,
                Err(e) => return Err(e),
            };
            // The socket is intentionally unconnected so the same send path can handle the OOB
            // handshake. Once packets arrive, though, only the selected server is authoritative —
            // especially for OOB chunk replies, whose cookie is correlation rather than security.
            if from != self.server {
                continue;
            }
            let data = &buf[..len];
            if let Some(w) = self.wiretap.as_mut() {
                w.record(data, false);
            }

            if oob::is_oob(data) {
                self.handle_oob(data, host);
                continue;
            }
            let Some(payload) = self.chan.process(data) else { continue };
            self.measure_rtt();

            let events = match svc::parse(&mut self.proto, payload) {
                Ok(e) => e,
                Err(e) => {
                    // Losing our place in the byte stream is not survivable: everything after the
                    // misread byte is fiction. Say exactly what happened, with the bytes, and drop
                    // the connection rather than play on a hallucinated world.
                    eprintln!("rtx-client: protocol desync: {e}");
                    eprintln!("rtx-client: seq={} proto={:?}", self.chan.incoming_sequence, self.proto);
                    eprintln!("{}", svc::hexdump(payload));
                    self.download = None;
                    self.server_download = None;
                    self.signon = Signon::Disconnected;
                    return Ok(out);
                }
            };
            for ev in events {
                self.handle(&ev, host);
                out.push(ev);
            }
        }
        self.retry_handshake();
        Ok(out)
    }

    /// Re-send an unanswered `getchallenge`/`connect`. UDP loses packets, and the handshake has no
    /// other way to notice.
    fn retry_handshake(&mut self) {
        if !matches!(self.signon, Signon::Challenge | Signon::Connecting) {
            return;
        }
        if self.last_oob.elapsed() < RESEND_INTERVAL {
            return;
        }
        self.last_oob = Instant::now();
        let pkt = match self.signon {
            Signon::Challenge => oob::getchallenge(),
            _ => {
                let n = self.negotiated();
                oob::connect(self.chan.qport, self.challenge, &self.userinfo.build().to_string(), &n)
            }
        };
        self.send_oob(&pkt);
    }

    /// The extension masks we're asking for.
    fn negotiated(&self) -> oob::Negotiated {
        oob::Negotiated {
            fte: self.proto.fte,
            fte2: self.proto.fte2,
            mvd1: self.proto.mvd1,
        }
    }

    fn handle_oob(&mut self, data: &[u8], host: &NetHost) {
        match oob::parse(data) {
            Some(oob::Oob::Challenge { challenge, fte, fte2, mvd1 }) if self.signon == Signon::Challenge => {
                self.challenge = challenge;
                // Narrow the server's offer to what we can parse. The server gets the last word in
                // `svc_serverdata` — it may enable less than we ask for — so this is a request, not
                // a conclusion.
                let n = oob::Negotiated::intersect(fte, fte2, mvd1);
                (self.proto.fte, self.proto.fte2, self.proto.mvd1) = (n.fte, n.fte2, n.mvd1);
                let pkt = oob::connect(self.chan.qport, challenge, &self.userinfo.build().to_string(), &n);
                self.send_oob(&pkt);
                self.signon = Signon::Connecting;
                self.last_oob = Instant::now();
            }
            Some(oob::Oob::Accepted) if self.signon == Signon::Connecting => {
                self.signon = Signon::Loading;
                self.stringcmd("new");
            }
            Some(oob::Oob::DownloadChunk { cookie, chunk, data })
                if self.signon == Signon::Downloading =>
            {
                self.receive_server_chunk(cookie, chunk, data.as_ref(), host);
            }
            Some(oob::Oob::Print(text)) => {
                eprintln!("rtx-client: server: {}", crate::text::readable(text.trim_end()))
            }
            Some(oob::Oob::Ping) => {
                self.send_oob(&oob::ack());
            }
            _ => {}
        }
    }

    /// Act on the messages that are this module's business.
    fn handle(&mut self, ev: &SvcEvent, host: &NetHost) {
        match ev {
            SvcEvent::ServerData(sd) => {
                // A new level: everything we knew about the last one is void.
                self.servercount = sd.servercount;
                self.gamedir = sd.gamedir.clone();
                self.playernum = sd.playernum;
                self.mapname.clear();
                self.sounds.clear();
                self.models.clear();
                self.frames.clear();
                self.download = None;
                self.server_download = None;
                self.signon = Signon::Loading;
                // A fresh level is a fresh match to say yes to, and the only way out of an
                // intermission — there is no message that ends one.
                self.readied = false;
                self.intermission = false;

                host.set_movevars(sd.movevars);
                self.stringcmd(&format!("soundlist {} 0", self.servercount));
            }
            // The lists, and the replies that walk through them, belong to the signon — so they're
            // only answered while we're in one.
            //
            // Under loss this is the difference between a bot with ears and one without. A chunk can
            // arrive after we're already playing, because QuakeWorld duplicates reliables: a lost ack
            // makes the sender resend, and the receiver's `incoming_reliable_sequence ^= 1` is
            // unconditional, so the payload is processed twice. Everyone lives with it — it's why
            // mvdsv answers a second `soundlist` with a shrug ("not valid -- already spawned") rather
            // than an error, and why you'll see that line in a lossy soak from any client, this one
            // included.
            //
            // What we mustn't do is *absorb* the late copy. `extend` truncates to the chunk's start
            // before appending, so a late first chunk would cut the finished list back to one entry
            // and leave the bot resolving every shot it hears against an empty table.
            SvcEvent::SoundList(list) if self.signon == Signon::Loading => {
                self.extend(true, list.start as usize, &list.names);
                if list.next != 0 {
                    let next = self.continuation(self.sounds.len(), list.next);
                    self.stringcmd(&format!("soundlist {} {next}", self.servercount));
                } else {
                    self.stringcmd(&format!("modellist {} 0", self.servercount));
                }
            }
            SvcEvent::ModelList(list) if self.signon == Signon::Loading => {
                self.extend(false, list.start as usize, &list.names);
                if list.next != 0 {
                    let next = self.continuation(self.models.len(), list.next);
                    self.stringcmd(&format!("modellist {} {next}", self.servercount));
                } else {
                    // The model list names the map, which is everything the client has been waiting
                    // for: it can now read the map, prove it has the same one, and build its world.
                    self.prespawn(host);
                }
            }
            SvcEvent::Download(message) if self.signon == Signon::Downloading => {
                self.handle_server_download(message, host);
            }
            SvcEvent::SpawnBaseline { entity, baseline } => self.frames.set_baseline(*entity, *baseline),
            SvcEvent::SpawnBaselineDelta { entity, delta } => self.frames.set_baseline_delta(*entity, delta),
            SvcEvent::PacketEntities(pe) => {
                let (seq, outgoing) = (self.chan.incoming_sequence, self.chan.outgoing_sequence);
                if self.frames.apply(seq, outgoing, pe.delta_from, &pe.updates) == Applied::Stale {
                    // The server built this on a frame we don't have — our clc_delta was probably
                    // lost. Everything in it is unusable; asking for a full update is the only way
                    // back, and that happens by simply not sending clc_delta next time.
                    eprintln!("rtx-client: entity delta from a frame we lack; requesting a full update");
                } else {
                    self.frames_at = Instant::now();
                }
                // The first entity frame is what "in the game" actually means.
                if self.signon == Signon::Loading {
                    self.signon = Signon::Active;
                }
            }
            SvcEvent::ServerInfo { key, value } => {
                self.serverinfo.set(key, value);
                self.adopt_serverinfo(host);
            }
            SvcEvent::StuffText(text) => self.stufftext(text, host),
            // The scoreboard. The game is over and our body is a camera on a pole; whatever the
            // brain thinks it's doing, it isn't playing, and it certainly isn't shooting.
            SvcEvent::Intermission { .. } | SvcEvent::Finale(_) => self.intermission = true,
            SvcEvent::ChokeCount(n) => self.chokes += *n as u32,
            SvcEvent::Disconnect => {
                self.download = None;
                self.server_download = None;
                self.signon = Signon::Disconnected;
            }
            _ => {}
        }
    }

    /// Append a chunk of a resource list at `start`.
    fn extend(&mut self, sounds: bool, start: usize, names: &[String]) {
        let list = if sounds { &mut self.sounds } else { &mut self.models };
        // Index 0 of each list is a placeholder the server never sends, so the first chunk starts
        // at 1 and the vector has to be padded to match — an off-by-one here would misname every
        // sound and model.
        if list.is_empty() {
            list.push(String::new());
        }
        list.resize(list.len().max(start.max(1)), String::new());
        list.truncate(start.max(1));
        list.extend_from_slice(names);
    }

    /// The offset to ask for the next chunk from.
    ///
    /// The server hands back a single byte, but a list can exceed 255 entries — so the client
    /// re-attaches the high bits from its own count. Without this, a long model list loops forever
    /// on entry 256.
    fn continuation(&self, have: usize, next: u8) -> usize {
        (have & 0xff00) + next as usize
    }

    /// Ask to spawn, proving we have the same map the server does.
    fn prespawn(&mut self, host: &NetHost) {
        // The map's filename is in the model list, not in serverdata.
        self.mapname = self
            .models
            .get(1)
            .and_then(|m| m.strip_prefix("maps/"))
            .and_then(|m| m.strip_suffix(".bsp"))
            .unwrap_or_default()
            .to_string();
        self.bind_map_then_prespawn(host);
    }

    /// Load the map and send `prespawn` — or, if we haven't got the map, start fetching it and pause
    /// the signon at [`Signon::Downloading`]. Called again once the download lands (`poll`), when the
    /// rebind succeeds and the signon resumes.
    fn bind_map_then_prespawn(&mut self, host: &NetHost) {
        if !host.rebind(&self.gamedir, &self.mapname) {
            if self.download_enabled {
                eprintln!("rtx-client: don't have maps/{}.bsp — downloading it", self.mapname);
                self.server_download = None;
                self.download = Some(super::download::Download::start(
                    host.basedir(),
                    self.gamedir.clone(),
                    self.mapname.clone(),
                ));
                self.signon = Signon::Downloading;
            } else {
                eprintln!(
                    "rtx-client: can't read maps/{}.bsp — the bot needs the map to see or move (and --no-download is set)",
                    self.mapname
                );
                self.signon = Signon::Disconnected;
            }
            return;
        }

        self.send_prespawn(host);
    }

    /// Re-index the freshly installed file and resume exactly where the missing map paused signon.
    fn resume_with_downloaded_map(&mut self, host: &NetHost) {
        if !host.rebind(&self.gamedir, &self.mapname) {
            eprintln!(
                "rtx-client: downloaded maps/{}.bsp, but it could not be loaded",
                self.mapname
            );
            self.download = None;
            self.server_download = None;
            self.signon = Signon::Disconnected;
            return;
        }
        self.signon = Signon::Loading;
        self.send_prespawn(host);
    }

    fn send_prespawn(&mut self, host: &NetHost) {
        let checksum = host
            .read_file(&std::ffi::CString::new(format!("maps/{}.bsp", self.mapname)).unwrap_or_default())
            .and_then(|bytes| checksum::map_checksum2(&bytes, &self.mapname).ok())
            .unwrap_or(0);

        // Prefer a real file when the gamedir has one loose — a mod may ship its own player model,
        // and then the stock answer is the wrong one — but always send something, because the models
        // are usually in a pak we don't read and a server that asks will warn without an answer.
        for (key, model, stock) in [
            ("pmodel", "progs/player.mdl", STOCK_PMODEL_CRC),
            ("emodel", "progs/eyes.mdl", STOCK_EMODEL_CRC),
        ] {
            let crc = host
                .read_file(&std::ffi::CString::new(model).unwrap_or_default())
                .map(|bytes| rtx_proto::crc::block(&bytes))
                .unwrap_or(stock);
            self.stringcmd(&format!("setinfo {key} {crc}"));
        }

        self.stringcmd(&format!("prespawn {} 0 {checksum}", self.servercount));
    }

    /// Adopt the serverinfo keys that change how we parse or play.
    fn adopt_serverinfo(&mut self, host: &NetHost) {
        // `*z_ext` decides how svc_playerinfo encodes pm_type and whether PF_ONGROUND is valid for
        // other players — i.e. whether a bot can tell that an enemy is airborne.
        if let Some(z) = self.serverinfo.get_u32("*z_ext") {
            self.proto.z_ext = z;
        }
        host.set_serverinfo(self.serverinfo.clone());
    }

    /// Obey a console command the server stuffed at us.
    ///
    /// This is a contract, not a courtesy: `cmd spawn`, `skins` and `changing` *are* the signon and
    /// the map cycle. A client that ignores them connects once and then hangs forever at the next
    /// map change.
    fn stufftext(&mut self, text: &str, host: &NetHost) {
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            // The server asking which extensions we speak, mid-session. Answer with all three
            // families — unlike `connect`, this form always sends every one.
            if line == "cmd pext" {
                let reply = format!(
                    "pext 0x{:x} 0x{:x} 0x{:x} 0x{:x} 0x{:x} 0x{:x}",
                    magic::FTE,
                    if self.proto.fte != 0 { self.proto.fte } else { protocol::FTE },
                    magic::FTE2,
                    if self.proto.fte2 != 0 { self.proto.fte2 } else { protocol::FTE2 },
                    magic::MVD1,
                    if self.proto.mvd1 != 0 { self.proto.mvd1 } else { protocol::MVD1 },
                );
                self.stringcmd(&reply);
                continue;
            }

            // `cmd <args>` forwards <args> straight back — this is what drives prespawn/spawn.
            if let Some(args) = line.strip_prefix("cmd ") {
                self.stringcmd(args.trim());
                continue;
            }

            if let Some(rest) = line.strip_prefix("fullserverinfo ") {
                self.serverinfo = Info::parse(rest.trim().trim_matches('"'));
                self.adopt_serverinfo(host);
                continue;
            }

            match line {
                // The map is changing: hold everything until the next serverdata.
                "changing" => {
                    self.download = None;
                    self.server_download = None;
                    self.signon = Signon::Changing;
                    self.frames.clear();
                }
                // Restart the signon on the same connection.
                "reconnect" => {
                    self.download = None;
                    self.server_download = None;
                    self.signon = Signon::Loading;
                    self.stringcmd("new");
                }
                // A real client would load skins here. We have no skins — but the server is waiting
                // for the `begin` that follows, so this is the cue to enter the game.
                "skins" => self.stringcmd(&format!("begin {}", self.servercount)),
                _ => {
                    // Everything else is either a server-side info line (`//ktx`, `//tinfo`, …) or a
                    // console command for a client with a console. Neither is ours; ignoring them is
                    // correct, and noisy logging here would drown the session.
                }
            }
        }
    }

    /// Send this frame's move.
    ///
    /// **`msec` is measured, never invented.** It's how much time the move covers, and the server
    /// integrates it: a client whose `msec` outruns the wall clock has moved further than real time
    /// allows, which is exactly what a speed cheat looks like — and mvdsv kicks for it. So the
    /// duration comes from the clock here, not from the brain's idea of a frame.
    ///
    /// The packet carries the last three moves. Two of them the server has already run; re-sending
    /// them costs a few bytes and makes a lost packet invisible instead of a hitch.
    pub(crate) fn send_move(&mut self, angles: glam::Vec3, forward: i32, side: i32, up: i32, buttons: u8, impulse: u8) -> io::Result<()> {
        let elapsed = self.last_move.elapsed();
        self.last_move = Instant::now();
        let msec = (elapsed.as_millis() as u32).min(clc::MAX_MSEC as u32) as u8;

        let cmd = clc::make_usercmd(msec, angles, forward as i16, side as i16, up as i16, buttons, impulse);
        self.cmds = [self.cmds[1], self.cmds[2], cmd];

        let sequence = self.chan.outgoing_sequence;
        let payload = clc::write_move(
            &clc::Move {
                oldest: self.cmds[0],
                previous: self.cmds[1],
                current: self.cmds[2],
                loss: 0,
            },
            sequence,
            // Naming a frame we have lets the server compress the next update against it. After a
            // stale delta — or once the server's own ring would have rolled past our base — we name
            // nothing, which is how a client asks for a full update.
            self.frames.delta_sequence(sequence),
        );
        self.transmit(&payload)
    }

    /// Send a packet with nothing in it — keeps the sequence advancing and the reliable queue
    /// moving while we're still connecting. During an FTE download it also carries a wide batch of
    /// unreliable `nextdl` commands: loss is handled by the chunk scheduler, so one missing request
    /// never stalls the whole transfer behind QuakeWorld's one-message reliable channel.
    pub(crate) fn send_nop(&mut self) -> io::Result<()> {
        let mut payload = clc::write_nop();
        self.append_download_requests(&mut payload);
        self.transmit(&payload)
    }

    fn append_download_requests(&mut self, payload: &mut Vec<u8>) {
        if self.signon != Signon::Downloading || self.chan.reliable_pending() {
            return;
        }
        let Some(transfer) = self.server_download.as_mut() else { return };
        let now = Instant::now();
        let sequence = self.chan.outgoing_sequence;
        let budget = transfer.request_budget(now, self.chan.incoming_acknowledged, self.rtt);
        let max_payload = protocol::MAX_MSGLEN - rtx_proto::netchan::HEADER_BYTES;

        for _ in 0..budget {
            let Some((chunk, percent, cookie)) = self.server_download.as_ref().and_then(|transfer| {
                transfer.next_missing_chunk().map(|chunk| (chunk, transfer.percent(), transfer.cookie()))
            }) else {
                break;
            };
            let command = clc::write_stringcmd(&format!("nextdl {chunk} {percent} {cookie}"));
            if payload.len() + command.len() > max_payload {
                break;
            }
            let Some(transfer) = self.server_download.as_mut() else { break };
            if transfer.mark_requested(chunk, sequence, now) {
                payload.extend_from_slice(&command);
            }
        }
    }

    fn transmit(&mut self, payload: &[u8]) -> io::Result<()> {
        let slot = self.chan.outgoing_sequence as usize % self.sent_at.len();
        let sequence = self.chan.outgoing_sequence;
        let promotes_reliable = self.chan.can_reliable() && self.chan.reliable_pending();
        let datagram = self.chan.transmit(payload);
        self.sent_at[slot] = Instant::now();
        self.send(&datagram)?;
        if promotes_reliable {
            if let Some(transfer) = self.server_download.as_mut() {
                transfer.mark_legacy_request_sent(sequence);
            }
        }
        Ok(())
    }

    /// Put a datagram on the wire, and in the record if one is being kept.
    ///
    /// Every byte this client sends goes through here, which is the point: a wiretap that missed the
    /// handshake would record a conversation starting in the middle.
    fn send(&mut self, datagram: &[u8]) -> io::Result<()> {
        if let Some(w) = self.wiretap.as_mut() {
            w.record(datagram, true);
        }
        self.sock.send_to(datagram, self.server)?;
        Ok(())
    }

    /// Put an out-of-band packet on the wire. The handshake is unreliable by nature — there's no
    /// netchan yet to carry it — so a failure here is answered by [`retry_handshake`](Self::retry_handshake),
    /// not by an error.
    fn send_oob(&mut self, pkt: &[u8]) {
        let _ = self.send(pkt);
    }

    /// Time the round trip from the packet the server just acknowledged.
    ///
    /// The ack names one of our sequences; we know when it left. There's no timestamp on the wire
    /// and none is needed — the sequence *is* the timestamp, if you kept one.
    fn measure_rtt(&mut self) {
        let acked = self.chan.incoming_acknowledged;
        // Sequence 0 is the "nothing acked yet" state, and an ack older than the ring has been
        // overwritten by a newer packet's send time — either would measure nonsense.
        if acked == 0 || self.chan.outgoing_sequence.wrapping_sub(acked) >= self.sent_at.len() as u32 {
            return;
        }
        let sent = self.sent_at[acked as usize % self.sent_at.len()];
        self.note_rtt(sent.elapsed().as_secs_f32());
    }

    /// Fold a fresh round-trip measurement into the smoothed estimate.
    ///
    /// A client acts on a world that is already `rtt/2` old and whose reaction to us is another
    /// `rtt/2` away — so this isn't diagnostics, it's how far ahead a bot must aim.
    fn note_rtt(&mut self, sample: f32) {
        self.rtt = if self.rtt == 0.0 { sample } else { self.rtt * 0.9 + sample * 0.1 };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// A session pointed at a socket nobody is listening on — enough to exercise everything that
    /// doesn't need an answer.
    fn session() -> Session {
        let ui = UserinfoBuilder {
            name: "rtxbot".to_string(),
            ..Default::default()
        };
        Session::connect("127.0.0.1:1".parse().unwrap(), ui, 0x1234, None, true).expect("bind")
    }

    fn host() -> NetHost {
        NetHost::new(PathBuf::from("/nonexistent"))
    }

    /// Smallest BSP our client-side reader accepts: a v29 header with empty lumps and one zeroed
    /// world model. It is enough to prove a downloaded file is rebound before `prespawn` resumes.
    fn minimal_bsp() -> Vec<u8> {
        const LUMPS: usize = 15;
        const HEADER: usize = 4 + LUMPS * 8;
        const MODEL_SIZE: usize = 64;
        let mut bytes = vec![0; HEADER + MODEL_SIZE];
        bytes[..4].copy_from_slice(&29u32.to_le_bytes());
        for lump in 0..LUMPS {
            let at = 4 + lump * 8;
            bytes[at..at + 4].copy_from_slice(&(HEADER as u32).to_le_bytes());
        }
        let models = 4 + 14 * 8;
        bytes[models + 4..models + 8].copy_from_slice(&(MODEL_SIZE as u32).to_le_bytes());
        bytes
    }

    fn temporary_host(tag: &str) -> (PathBuf, NetHost) {
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("rtx-session-{tag}-{}-{nonce}", std::process::id()));
        (root.clone(), NetHost::new(root))
    }

    #[test]
    fn an_http_failure_falls_back_to_the_connected_server() {
        let (root, host) = temporary_host("fallback");
        let mut s = session();
        s.signon = Signon::Downloading;
        s.gamedir = "qw".to_string();
        s.mapname = "missing".to_string();
        s.download = Some(super::super::download::Download::completed(Err("HTTP 404".to_string())));

        s.poll_download(&host);

        assert!(s.server_download.is_some());
        assert_eq!(s.signon, Signon::Downloading);
        let packet = s.chan.transmit(b"");
        let body = &packet[rtx_proto::netchan::HEADER_BYTES..];
        assert_eq!(body[0], clc::op::STRINGCMD);
        assert_eq!(
            rtx_proto::sizebuf::Reader::new(&body[1..]).string().unwrap(),
            "download \"maps/missing.bsp\""
        );

        drop(s);
        drop(host);
        let _ = std::fs::remove_dir_all(root);
    }

    /// Chunk requests belong to the unreliable half of the packet, and an OOB response must carry
    /// this transfer's cookie. Completion installs, rebinds, and continues at prespawn.
    #[test]
    fn fte_download_requests_are_unreliable_and_resume_prespawn() {
        let (root, host) = temporary_host("chunked");
        let bytes = minimal_bsp();
        let cookie = 7;
        let mut transfer = super::super::download::ServerDownload::new(root.clone(), "qw", "tiny", cookie).unwrap();
        transfer
            .begin_chunked("maps/tiny.bsp", bytes.len() as u64, Instant::now())
            .unwrap();

        let mut s = session();
        s.signon = Signon::Downloading;
        s.gamedir = "qw".to_string();
        s.mapname = "tiny".to_string();
        s.server_download = Some(transfer);

        let mut payload = clc::write_nop();
        s.append_download_requests(&mut payload);
        assert_eq!(payload[0], clc::op::NOP);
        assert_eq!(payload[1], clc::op::STRINGCMD);
        assert_eq!(
            rtx_proto::sizebuf::Reader::new(&payload[2..]).string().unwrap(),
            "nextdl 0 0 7"
        );
        assert!(!s.chan.reliable_pending(), "chunk requests must not enter the reliable queue");

        let mut chunk = [0; svc::DOWNLOAD_CHUNK_SIZE];
        chunk[..bytes.len()].copy_from_slice(&bytes);
        s.receive_server_chunk(cookie + 1, 0, &chunk, &host);
        assert_eq!(s.signon, Signon::Downloading, "a stale cookie cannot complete this file");
        assert!(s.server_download.is_some());
        assert!(!root.join("qw/maps/tiny.bsp").exists());

        s.receive_server_chunk(cookie, 0, &chunk, &host);
        assert!(s.server_download.is_none());
        assert_eq!(s.signon, Signon::Loading);
        assert!(host.has_map(), "the installed BSP is rebound before signon resumes");
        assert_eq!(std::fs::read(root.join("qw/maps/tiny.bsp")).unwrap(), bytes);
        assert!(s.chan.reliable_pending(), "cleanup and prespawn are queued reliably");

        drop(s);
        drop(host);
        let _ = std::fs::remove_dir_all(root);
    }

    /// A lost acknowledgement can make QuakeWorld replay a reliable server payload under a fresh
    /// packet sequence. Do not append that old regular block again while our `nextdl` is unsent or
    /// unacknowledged; accept the next block once the server proves it saw the request.
    #[test]
    fn regular_download_ignores_reliable_replays() {
        let (root, host) = temporary_host("regular-replay");
        let mut bytes = minimal_bsp();
        bytes.resize(1500, 0x5a);
        let mut s = session();
        s.signon = Signon::Downloading;
        s.gamedir = "qw".to_string();
        s.mapname = "regular".to_string();
        s.server_download = Some(
            super::super::download::ServerDownload::new(root.clone(), "qw", "regular", 1).unwrap(),
        );

        let first = DownloadMessage::LegacyBlock { percent: 51, data: bytes[..768].to_vec() };
        s.handle_server_download(&first, &host);
        s.handle_server_download(&first, &host); // replay before queued nextdl has even left

        let request_sequence = s.chan.outgoing_sequence;
        s.transmit(&clc::write_nop()).unwrap();
        let mut ack = Vec::new();
        ack.extend_from_slice(&1u32.to_le_bytes());
        ack.extend_from_slice(&(request_sequence | (1 << 31)).to_le_bytes());
        assert!(s.chan.process(&ack).is_some());
        s.handle_server_download(
            &DownloadMessage::LegacyBlock { percent: 100, data: bytes[768..].to_vec() },
            &host,
        );

        assert_eq!(std::fs::read(root.join("qw/maps/regular.bsp")).unwrap(), bytes);
        assert_eq!(s.signon, Signon::Loading);
        assert!(host.has_map());

        drop(s);
        drop(host);
        let _ = std::fs::remove_dir_all(root);
    }

    /// Resource lists are 1-based: index 0 is a placeholder the server never sends. An off-by-one
    /// here renames every sound and model — `svc_sound 7` would fetch the wrong weapon, and the
    /// bot's ears would lie.
    #[test]
    fn resource_lists_are_one_based() {
        let mut s = session();
        s.extend(true, 0, &["a.wav".to_string(), "b.wav".to_string()]);
        assert_eq!(s.sounds, vec!["", "a.wav", "b.wav"]);

        // A continuation chunk appends where the last left off.
        s.extend(true, 3, &["c.wav".to_string()]);
        assert_eq!(s.sounds, vec!["", "a.wav", "b.wav", "c.wav"]);
    }

    /// A resource list belongs to the signon, and a late chunk of one is history.
    ///
    /// Found by playing through 20% packet loss, where it turns out to be routine: QuakeWorld
    /// duplicates reliables (a lost ack makes the sender resend, and the receiver toggles its
    /// reliable bit unconditionally), so a chunk of the list lands again long after the bot is in the
    /// game. The damage is in absorbing it: `extend` truncates to the chunk's start before appending,
    /// so a late *first* chunk cuts the finished list back to one entry — and the sound list is the
    /// bot's ears. It would spend the rest of the map resolving every shot it hears against an empty
    /// table.
    #[test]
    fn a_late_list_chunk_cannot_cost_the_bot_its_ears() {
        let mut s = session();
        let host = host();
        s.signon = Signon::Loading;
        let list = |start: u16, names: &[&str]| rtx_proto::svc::ResourceList {
            start,
            names: names.iter().map(|n| n.to_string()).collect(),
            next: 0,
        };
        s.handle(&SvcEvent::SoundList(list(0, &["a.wav", "b.wav", "c.wav"])), &host);
        assert_eq!(s.sounds, vec!["", "a.wav", "b.wav", "c.wav"]);

        // In the game now — and the first chunk turns up again, late.
        s.signon = Signon::Active;
        s.handle(&SvcEvent::SoundList(list(0, &["a.wav"])), &host);
        assert_eq!(s.sounds, vec!["", "a.wav", "b.wav", "c.wav"], "the list we spent the signon building");
    }

    /// The continuation offset re-attaches the high bits the server's single byte can't carry.
    /// Without it, a list longer than 255 loops on entry 256 forever.
    #[test]
    fn list_continuation_carries_high_bits() {
        let s = session();
        assert_eq!(s.continuation(100, 100), 100);
        assert_eq!(s.continuation(300, 44), 256 + 44);
        assert_eq!(s.continuation(600, 90), 512 + 90);
    }

    /// The signon contract. Each of these is a step the server is waiting on; ignoring any one of
    /// them hangs the connection.
    #[test]
    fn stufftext_drives_the_signon() {
        let mut s = session();
        let h = host();
        s.servercount = 42;

        // `cmd <args>` forwards the args verbatim — this is what carries prespawn and spawn.
        s.stufftext("cmd spawn 42 0\n", &h);
        assert!(s.chan.reliable_pending());

        // `skins` is the cue to enter the game.
        let mut s = session();
        s.servercount = 42;
        s.stufftext("skins\n", &h);
        assert!(s.chan.reliable_pending(), "skins must be answered with `begin`");

        // `changing` voids the world until the next serverdata.
        let mut s = session();
        s.frames.apply(1, 1, None, &[]);
        s.stufftext("changing\n", &h);
        assert_eq!(s.signon, Signon::Changing);
        assert_eq!(s.frames.delta_sequence(2), None, "the old level's frames are void");
    }

    /// `fullserverinfo` arrives as a stufftext, and carries `*z_ext` — which decides whether we can
    /// tell that an enemy is airborne. Missing it silently degrades what the bot can perceive.
    #[test]
    fn fullserverinfo_adopts_z_ext() {
        let mut s = session();
        let h = host();
        s.stufftext("fullserverinfo \"\\*z_ext\\511\\teamplay\\2\\maxfps\\77\"\n", &h);

        assert_eq!(s.proto.z_ext, 511);
        assert!(s.proto.has_z_ext(rtx_proto::protocol::z_ext::PF_ONGROUND));
        assert_eq!(s.serverinfo().get("teamplay"), Some("2"));
        assert_eq!(s.serverinfo().get_f32("maxfps"), Some(77.0));
    }

    /// The server can ask mid-session which extensions we speak. Unlike `connect`, the reply names
    /// every family — and reports what was *negotiated*, not what we'd have liked.
    #[test]
    fn cmd_pext_is_answered_with_all_three_families() {
        let mut s = session();
        let h = host();
        s.proto.apply(rtx_proto::protocol::fte::TRANS, 0, 0);
        s.stufftext("cmd pext\n", &h);

        // Dig the queued stringcmd back out of the netchan.
        let pkt = s.chan.transmit(b"");
        let body = &pkt[rtx_proto::netchan::HEADER_BYTES..];
        assert_eq!(body[0], clc::op::STRINGCMD);
        let sent = rtx_proto::sizebuf::Reader::new(&body[1..]).string().unwrap();
        assert_eq!(
            sent,
            format!("pext 0x{:x} 0x8 0x{:x} 0x0 0x{:x} 0x{:x}",
                magic::FTE, magic::FTE2, magic::MVD1, protocol::MVD1),
            "negotiated fte (0x8) is reported, and every family appears"
        );
    }

    /// A line the server stuffs that isn't ours — an info line, or a command for a client with a
    /// console — must be ignored rather than forwarded or logged into oblivion.
    #[test]
    fn unknown_stufftext_is_ignored() {
        let mut s = session();
        let h = host();
        for line in ["//ktx 1\n", "//tinfo 1 2 3\n", "r_skyname foo\n", "alias _cs \"say hi\"\n", "\n"] {
            s.stufftext(line, &h);
        }
        assert!(!s.chan.reliable_pending(), "nothing should have been sent back");
        assert_eq!(s.signon, Signon::Challenge, "and nothing should have changed state");
    }

    /// The handshake retries: UDP drops packets and the challenge exchange has no other way to
    /// notice. A client that sends `getchallenge` once and waits forever never connects on a lossy
    /// link.
    #[test]
    fn handshake_retries_when_unanswered() {
        let mut s = session();
        assert_eq!(s.signon, Signon::Challenge);

        s.last_oob = Instant::now();
        s.retry_handshake(); // too soon
        s.last_oob = Instant::now() - RESEND_INTERVAL - Duration::from_millis(1);
        s.retry_handshake(); // fires
        assert!(s.last_oob.elapsed() < Duration::from_secs(1), "the retry re-stamped the clock");

        // Once we're in, retries stop: `new` is reliable and the netchan resends it for us.
        s.signon = Signon::Loading;
        s.last_oob = Instant::now() - RESEND_INTERVAL * 2;
        let before = s.last_oob;
        s.retry_handshake();
        assert_eq!(s.last_oob, before, "no out-of-band retries once connected");
    }

    /// `msec` must be measured, not invented — the server integrates it, and a client whose msec
    /// outruns the clock looks exactly like a speed cheat. It's also clamped, because the server
    /// clamps it too.
    #[test]
    fn move_msec_is_measured_and_clamped() {
        let mut s = session();
        s.last_move = Instant::now() - Duration::from_millis(13);
        s.send_move(glam::Vec3::ZERO, 400, 0, 0, 0, 0).ok();
        let msec = s.cmds[2].msec;
        assert!((12..=15).contains(&msec), "msec {msec} should track the ~13ms that really passed");

        // A long stall is clamped rather than claimed.
        s.last_move = Instant::now() - Duration::from_secs(30);
        s.send_move(glam::Vec3::ZERO, 0, 0, 0, 0, 0).ok();
        assert_eq!(s.cmds[2].msec, clc::MAX_MSEC);
    }

    /// Every move packet carries the last three, so a single lost packet costs nothing.
    #[test]
    fn move_packet_carries_the_last_three_commands() {
        let mut s = session();
        for fwd in [100, 200, 300, 400] {
            s.send_move(glam::Vec3::ZERO, fwd, 0, 0, 0, 0).ok();
        }
        assert_eq!(
            [s.cmds[0].forward, s.cmds[1].forward, s.cmds[2].forward],
            [200, 300, 400],
            "the window slides, oldest first"
        );
    }

    /// A move names the frame we want the next update built on — and after a stale delta it names
    /// nothing, which is how a client asks for a full update.
    #[test]
    fn move_requests_deltas_only_against_frames_we_have() {
        let mut s = session();
        assert_eq!(s.frames.delta_sequence(1), None, "nothing to delta against yet");

        s.frames.apply(7, 1, None, &[]);
        assert_eq!(s.frames.delta_sequence(2), Some(7));

        // A delta from a frame we lack invalidates the chain.
        assert_eq!(s.frames.apply(8, 2, Some(200), &[]), Applied::Stale);
        assert_eq!(s.frames.delta_sequence(3), None, "so the next move asks for a full update");
    }

    /// A new level voids everything: baselines, snapshots and the delta chain all belonged to the
    /// map that just ended, and the model indices are about to be reassigned.
    #[test]
    fn serverdata_resets_the_level() {
        let mut s = session();
        let h = host();
        s.sounds = vec!["".into(), "old.wav".into()];
        s.models = vec!["".into(), "maps/old.bsp".into()];
        s.frames.apply(5, 1, None, &[]);

        let sd = rtx_proto::svc::ServerData {
            servercount: 99,
            gamedir: "ktx".to_string(),
            playernum: 3,
            movevars: rtx_proto::svc::MoveVars { gravity: 640.0, ..Default::default() },
            ..Default::default()
        };
        s.handle(&SvcEvent::ServerData(Box::new(sd)), &h);

        assert_eq!(s.servercount, 99);
        assert_eq!(s.gamedir, "ktx");
        assert_eq!(s.playernum(), 3);
        assert!(s.sounds.is_empty(), "last level's sound list is void");
        assert!(s.models.is_empty());
        assert_eq!(s.frames.delta_sequence(2), None);
        assert_eq!(h.cvar(c"sv_gravity"), 640.0, "the server's physics are adopted");
        assert!(s.chan.reliable_pending(), "and the soundlist request goes out");
    }
}
