// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-demo-tool` — read and analyze QuakeWorld demos, in both containers.
//!
//! A **`.qwd`** is what a client records: a flat log of timestamped records holding its own
//! `dem_cmd` inputs and the `dem_read` packets it received. A **`.mvd`** is what a *server* records
//! — the whole game, every player, addressed blocks, delta timestamps, and no single point of view
//! (see [`mvd`], which is where all the container differences live).
//!
//! Either way the framing is the *only* thing this crate parses: a block's payload is a recorded
//! message stream, decoded by [`rtx_proto::svc`], the same codec the live client speaks. So the
//! split is clean — the containers are read here, the wire is read there.
//!
//! [`parse_demo`] picks the container by content and hands back [`Frame`]s: one normalised player
//! state per update, whichever format it came from. The [`analysis`] module turns those into
//! per-player motion [`Track`](analysis::Track)s and the things worth asking of them — speed
//! percentiles, the jumps, turn rates. The `qwd` binary is the CLI: `players` lists a demo's
//! roster, `dump` emits CSV, `analyze` prints a movement report.
//!
//! What the two formats can and cannot tell you differs, and [`Frame`]'s `Option`s are where that
//! shows: classic MVD playerinfo carries no velocity or ground flag, while a `.qwd` can carry
//! both for the players its recorder could see. Replacement-delta MVDs and QWDs retain entity
//! state across packets and carry prediction velocity, movement, view angles and ground contact.

use std::io::{Cursor, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use binrw::{BinRead, BinReaderExt};
use glam::Vec3;
use rtx_proto::protocol::ProtoState;
use rtx_proto::svc::{self, MoveVars, SvcEvent, Usercmd};

pub mod analysis;
pub mod line;
pub mod mvd;
pub mod teamplay;

pub use analysis::{Motion, Summary, Track};

/// The fixed `usercmd_t` embedded in a `dem_cmd` record — the local player's raw input, not a
/// delta. Byte-for-byte the engine's C layout: `msec`, 3 alignment bytes, `vec3_t` view angles,
/// three move shorts, then the button and impulse bytes.
#[derive(BinRead)]
#[br(little)]
struct RawUsercmd {
    msec: u8,
    #[br(pad_before = 3)]
    angles: [f32; 3],
    forward: i16,
    side: i16,
    up: i16,
    buttons: u8,
    impulse: u8,
}

impl RawUsercmd {
    fn into_usercmd(self) -> Usercmd {
        Usercmd {
            msec: self.msec,
            angles: Vec3::from_array(self.angles),
            forward: self.forward,
            side: self.side,
            up: self.up,
            buttons: self.buttons,
            impulse: self.impulse,
        }
    }
}

/// Each record's 5-byte header: a float demo time and the `dem_*` kind byte.
#[derive(BinRead)]
#[br(little)]
struct RecordHeader {
    time: f32,
    kind: u8,
}

/// The bytes of a QuakeWorld netchan sequence header (`incoming`/`incoming_acknowledged` longs)
/// that lead every recorded `dem_read` packet ahead of the svc message stream.
const NETCHAN_HEADER: usize = 8;

/// Which demo container a [`Demo`] was read from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// A single client's recording: absolute timestamps, its own inputs, wire velocities.
    Qwd,
    /// A server-side multi-view recording: delta timestamps and every player. Classic playerinfo
    /// lacks velocity; replacement deltas carry it.
    Mvd,
}

impl Format {
    pub fn as_str(self) -> &'static str {
        match self {
            Format::Qwd => "qwd",
            Format::Mvd => "mvd",
        }
    }
}

/// The local client's own input for one frame, from a `dem_cmd` record.
#[derive(Clone, Copy, Debug)]
pub struct DemoCmd {
    /// Demo timestamp of the record.
    pub time: f32,
    /// The decoded command — view angles, moves, buttons, msec.
    pub cmd: Usercmd,
}

/// One player's state at one instant — the common currency of both containers.
///
/// A `.qwd` and a `.mvd` say very different things on the wire (see [`mvd`]), and normalising here
/// is what lets [`analysis`] and the CLI treat them alike. The cost is that a field absent from one
/// format is `Option` for both: classic MVD playerinfo carries no velocity, while replacement
/// deltas carry prediction state in either container. A `.qwd` covers its recorder's visible players.
#[derive(Clone, Debug)]
pub struct Frame {
    /// Demo timestamp in seconds from the start of the file.
    pub time: f32,
    /// Player slot, 0..32. Stable across the file; [`Demo::players`] names it.
    pub player: u8,
    pub origin: Vec3,
    /// View angles in degrees. Classic MVD pitch is decoded from its recorder scaling; RD
    /// model angles are converted using the prediction basis before publication.
    pub angles: Vec3,
    /// Velocity **as the wire stated it**, in game units/s. `None` for classic MVD playerinfo,
    /// where analysis must derive it from successive origins ([`analysis::Track`] does).
    pub velocity: Option<Vec3>,
    /// The player's own input, when the format carries it: `PF_COMMAND` in a `.qwd`, or a hidden
    /// `usercmd` block in an MVD. RD prediction hints alone do not supply a recorded command.
    pub command: Option<Usercmd>,
    /// Whether the recorder marked this player dead this frame.
    pub dead: bool,
    /// Ground contact, when carried by RD or `Z_EXT_PF_ONGROUND`. Absent in classic MVD playerinfo.
    pub on_ground: Option<bool>,
    /// Animation frame of the player's weapon — a usable proxy for firing.
    pub weaponframe: Option<u16>,
}

/// A player's score changing, from `svc_updatefrags`.
///
/// This is the scoreboard the server itself maintained, which makes it the one unarguable measure of
/// who won — obituary text can be parsed and miscounted, and a bot's own frag counter is the thing
/// under test. Recorded as a series rather than a total so the score can be read *over time*: a team
/// that wins the first two minutes and loses the next eight is a different finding from one that
/// trails evenly throughout.
#[derive(Clone, Copy, Debug)]
pub struct FragUpdate {
    pub time: f32,
    pub player: u8,
    pub frags: i16,
}

/// One `svc_print` line, which for a deathmatch server is mostly obituaries.
#[derive(Clone, Debug)]
pub struct PrintLine {
    pub time: f32,
    /// Print level; obituaries arrive at `PRINT_MEDIUM` (1).
    pub level: u8,
    pub text: String,
}

/// What a demo knows about one player slot.
#[derive(Clone, Debug, Default)]
pub struct PlayerSlot {
    /// `\name\` from the slot's userinfo.
    pub name: String,
    /// `\team\` from the slot's userinfo.
    pub team: String,
    /// Whether the slot joined as a spectator (`\*spectator\`).
    pub spectator: bool,
    /// The raw `\k\v\` userinfo string, last seen.
    pub userinfo: String,
    /// How many frames this slot appeared in — the cheap way to tell players from empty slots.
    pub frames: usize,
}

impl PlayerSlot {
    /// Whether this slot ever carried a player. Empty slots exist in every demo; a 4-on-4 fills
    /// eight of the thirty-two.
    pub fn present(&self) -> bool {
        self.frames > 0 || !self.name.is_empty()
    }

    /// The slot's display name, falling back to its number when userinfo never arrived.
    pub fn label(&self, slot: u8) -> String {
        if self.name.is_empty() {
            format!("#{slot}")
        } else {
            self.name.clone()
        }
    }
}

/// Pull `key` out of a `\k\v\k\v` userinfo string.
fn info_value(userinfo: &str, key: &str) -> String {
    let mut it = userinfo.split('\\').skip(1);
    while let (Some(k), Some(v)) = (it.next(), it.next()) {
        if k == key {
            return v.to_string();
        }
    }
    String::new()
}

/// Everything one demo yields: the framing-derived context plus the two event streams.
#[derive(Clone, Debug)]
pub struct Demo {
    /// The file this came from.
    pub path: PathBuf,
    /// The negotiated protocol state at end of file (coord/angle widths, extension masks).
    pub proto: ProtoState,
    /// Which container this came from — the two carry different information, and a consumer that
    /// silently assumes velocity or usercmds exist will read zeros off an MVD.
    pub format: Format,
    /// The recording client's own player slot, from the last `svc_serverdata`. `None` for an MVD,
    /// which records the whole game rather than one client's view.
    pub local_player: Option<u8>,
    /// What each of the 32 slots was — name, team, spectator flag.
    pub players: Vec<PlayerSlot>,
    /// The map, from `svc_serverdata`.
    pub levelname: String,
    /// The server's physics constants, from the last `svc_serverdata`.
    pub movevars: Option<MoveVars>,
    /// The local player's own `dem_cmd` inputs, in file order (ascending time).
    pub demo_cmds: Vec<DemoCmd>,
    /// Every player update, in file order. Each accepted RD frame samples all retained players.
    pub frames: Vec<Frame>,
    /// Every `svc_updatefrags`, in file order — the server's own scoreboard as it moved.
    pub frag_updates: Vec<FragUpdate>,
    /// Every `svc_print`, in file order. Obituaries live here.
    pub prints: Vec<PrintLine>,
    /// Non-fatal per-packet decode failures (offset + reason). A malformed tail packet lands here
    /// rather than aborting the whole file.
    pub warnings: Vec<String>,
}

/// Why a demo couldn't be framed. Failures *inside* a packet are collected as
/// [`Demo::warnings`] instead; these are the container-level errors that stop parsing.
#[derive(Debug)]
pub enum Error {
    /// Reading the file failed.
    Io(std::io::Error),
    /// A record ran off the end of the file.
    Truncated {
        /// What was being read.
        what: &'static str,
        /// Byte offset it started at.
        at: u64,
    },
    /// A `dem_read` declared a length past the end of the file.
    LengthOverflow {
        /// Byte offset of the length field.
        at: u64,
    },
    /// A record tag that isn't `dem_cmd`/`dem_read`/`dem_set`.
    UnknownRecord {
        /// The tag byte.
        kind: u8,
        /// Byte offset it appeared at.
        at: u64,
    },
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Truncated { what, at } => write!(f, "truncated {what} at offset {at}"),
            Error::LengthOverflow { at } => write!(f, "dem_read length exceeds file at offset {at}"),
            Error::UnknownRecord { kind, at } => write!(f, "unknown demo record type {kind} at offset {at}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

type Result<T> = std::result::Result<T, Error>;

/// Accumulates decoded messages into a [`Demo`], for either container.
///
/// Both formats reduce to "here is a message stream, and here is when it happened", so everything
/// downstream of the framing lives here. The MVD-only state is the per-player carry-forward: those
/// `svc_playerinfo` deltas are relative to that player's *last* update, so the resolver has to hold
/// one absolute state per slot and patch it.
struct Feed {
    proto: ProtoState,
    format: Format,
    local_player: Option<u8>,
    movevars: Option<MoveVars>,
    levelname: String,
    players: Vec<PlayerSlot>,
    frames: Vec<Frame>,
    frag_updates: Vec<FragUpdate>,
    prints: Vec<PrintLine>,
    warnings: Vec<String>,
    /// Persistent RD state, separate from classic per-player MVD deltas.
    rd: rtx_proto::rd_state::RetainedEntities,
    /// Per-slot absolute state, for resolving MVD deltas. Origin and angles only — those are the
    /// fields the format deltas.
    last: Vec<(Vec3, Vec3)>,
    /// Hidden-channel usercmds, keyed by slot, awaiting the frame they belong to.
    pending_cmds: Vec<Option<Usercmd>>,
}

/// A recorder quirk: MVD pitch is written pre-multiplied by −3 (it packs a wider useful range into
/// the byte), so it has to be undone to mean degrees again. Roll is always written as zero.
const MVD_PITCH_SCALE: f32 = -3.0;

impl Feed {
    fn new(format: Format) -> Self {
        Feed {
            proto: if format == Format::Mvd {
                ProtoState::new_mvd()
            } else {
                ProtoState::new()
            },
            format,
            local_player: None,
            movevars: None,
            levelname: String::new(),
            players: vec![PlayerSlot::default(); 32],
            frames: Vec::new(),
            frag_updates: Vec::new(),
            prints: Vec::new(),
            warnings: Vec::new(),
            rd: rtx_proto::rd_state::RetainedEntities::default(),
            last: vec![(Vec3::ZERO, Vec3::ZERO); 32],
            pending_cmds: vec![None; 32],
        }
    }

    /// Decode one svc message stream and fold what it says into the demo.
    fn packet(&mut self, time: f32, msg: &[u8], at: usize) {
        // A malformed tail rejects the entire recorded packet, including earlier serverdata.
        let mut proto = self.proto.clone();
        let events = match svc::parse(&mut proto, msg) {
            Ok(evs) => evs,
            Err(e) => {
                self.rd.invalidate();
                self.warnings.push(format!("packet at offset {at}: {e}"));
                return;
            }
        };
        let mut replacement = self.proto.fte2 & rtx_proto::protocol::fte2::REPLACEMENTDELTAS != 0;
        self.proto = proto;
        for ev in events {
            match ev {
                SvcEvent::PlayerInfo(info) => self.push_player(time, *info, false),
                SvcEvent::SpawnReplacementBaseline { entity, delta } => {
                    if let Err(reason) = self.rd.set_baseline(entity, delta.apply(None, None)) {
                        self.rd.invalidate();
                        self.warnings.push(format!("packet at offset {at}: {reason}"));
                    }
                }
                SvcEvent::SpawnBaseline { entity, baseline } if replacement => {
                    let delta = rtx_proto::rd::Delta {
                        origin: baseline.origin.to_array().map(Some),
                        angles: baseline.angles.to_array().map(Some),
                        modelindex: Some(baseline.modelindex),
                        frame: Some(baseline.frame),
                        colormap: Some(baseline.colormap),
                        skinnum: Some(baseline.skinnum as i16),
                        ..Default::default()
                    };
                    if let Err(reason) = self.rd.set_baseline(entity, delta.apply(None, None)) {
                        self.rd.invalidate();
                        self.warnings.push(format!("packet at offset {at}: {reason}"));
                    }
                }
                SvcEvent::ReplacementEntities(frame) => match self.rd.apply(&frame) {
                    Ok(rtx_proto::rd_state::Applied::Published) => {
                        // Omitted records leave players present. Sample the retained world rather
                        // than mistaking the sparse wire changes for a full player snapshot.
                        for entity in 1..=32 {
                            if let Some(info) = self
                                .rd
                                .get(entity)
                                .and_then(|state| rtx_proto::rd::player_info(entity, state))
                            {
                                self.push_player(time, info, true);
                            }
                        }
                    }
                    Ok(rtx_proto::rd_state::Applied::AwaitingReset) => {}
                    Err(reason) => self.warnings.push(format!("packet at offset {at}: {reason}")),
                },
                SvcEvent::MvdPlayerInfo(info) => {
                    let slot = info.player as usize;
                    if slot >= self.last.len() {
                        continue;
                    }
                    // Resolve the delta: a field the recorder omitted is unchanged, not zero.
                    let (origin, angles) = &mut self.last[slot];
                    for i in 0..3 {
                        if let Some(v) = info.origin[i] {
                            origin[i] = v;
                        }
                        if let Some(a) = info.angles[i] {
                            angles[i] = if i == 0 { a / MVD_PITCH_SCALE } else { a };
                        }
                    }
                    let (origin, angles) = (*origin, *angles);
                    self.players[slot].frames += 1;
                    self.frames.push(Frame {
                        time,
                        player: info.player,
                        origin,
                        angles,
                        velocity: None, // classic MVD playerinfo has no velocity
                        command: self.pending_cmds[slot].take(),
                        dead: info.dead(),
                        on_ground: None,
                        weaponframe: info.weaponframe.map(u16::from),
                    });
                }
                SvcEvent::ServerData(sd) => {
                    replacement = sd.fte2 & rtx_proto::protocol::fte2::REPLACEMENTDELTAS != 0;
                    self.rd.clear();
                    self.last.fill((Vec3::ZERO, Vec3::ZERO));
                    self.pending_cmds.fill(None);
                    if self.format == Format::Qwd {
                        self.local_player = Some(sd.playernum);
                    }
                    self.movevars = Some(sd.movevars);
                    self.levelname = sd.levelname.clone();
                }
                SvcEvent::UpdateFrags { player, frags } => {
                    self.frag_updates.push(FragUpdate { time, player, frags });
                }
                SvcEvent::Print { level, text } => {
                    self.prints.push(PrintLine { time, level, text });
                }
                SvcEvent::UpdateUserinfo { player, userinfo, .. } => self.set_userinfo(player, userinfo),
                SvcEvent::SetInfo { player, key, value } => {
                    if let Some(p) = self.players.get_mut(player as usize) {
                        match key.as_str() {
                            "name" => p.name = value,
                            "team" => p.team = value,
                            "*spectator" => p.spectator = !value.is_empty() && value != "0",
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }

    fn push_player(&mut self, time: f32, info: svc::PlayerInfo, replacement: bool) {
        let velocity_present = (0..3).any(|i| info.flags & (svc::pf::VELOCITY1 << i) != 0);
        let angles = info.command.map_or(Vec3::ZERO, |command| command.angles);
        // RD's projected usercmd carries prediction/view state, not recorded buttons/impulse.
        // MVD supplies real input on its hidden channel; QWD resolves local dem_cmd separately.
        let command = if replacement {
            self.pending_cmds[info.player as usize].take()
        } else {
            info.command
        };
        if let Some(player) = self.players.get_mut(info.player as usize) {
            player.frames += 1;
        }
        self.frames.push(Frame {
            time,
            player: info.player,
            origin: info.origin,
            angles,
            velocity: velocity_present.then_some(info.velocity),
            command,
            dead: info.dead(),
            on_ground: (replacement || self.proto.has_z_ext(rtx_proto::protocol::z_ext::PF_ONGROUND))
                .then(|| info.on_ground()),
            weaponframe: info.weaponframe,
        });
    }

    fn set_userinfo(&mut self, player: u8, userinfo: String) {
        let Some(p) = self.players.get_mut(player as usize) else {
            return;
        };
        p.name = info_value(&userinfo, "name");
        p.team = info_value(&userinfo, "team");
        p.spectator = !info_value(&userinfo, "*spectator").is_empty();
        p.userinfo = userinfo;
    }

    fn finish(self, path: PathBuf, demo_cmds: Vec<DemoCmd>) -> Demo {
        Demo {
            path,
            proto: self.proto,
            format: self.format,
            local_player: self.local_player,
            players: self.players,
            levelname: self.levelname,
            movevars: self.movevars,
            demo_cmds,
            frames: self.frames,
            frag_updates: self.frag_updates,
            prints: self.prints,
            warnings: self.warnings,
        }
    }
}

/// Parse a demo file, choosing the container by content.
///
/// Neither format has a magic number, so the choice is structural — see [`mvd::looks_like_mvd`].
/// Content beats the extension deliberately: demos get renamed, and misreading the container is
/// silent rather than loud.
pub fn parse_demo(path: impl AsRef<Path>) -> Result<Demo> {
    let path = path.as_ref().to_path_buf();
    let data = std::fs::read(&path)?;
    if mvd::looks_like_mvd(&data) {
        Ok(parse_mvd_bytes(&data, path))
    } else {
        parse_qwd_bytes(&data, path)
    }
}

/// Parse multi-view demo bytes. Framing problems end the file rather than failing it — a demo cut
/// short by a crashed server is normal, and everything before the tear is good.
pub fn parse_mvd_bytes(data: &[u8], path: PathBuf) -> Demo {
    let mut feed = Feed::new(Format::Mvd);
    let mut r = mvd::Reader::new(data);
    let mut cmds = Vec::new();
    while let Some(b) = r.next_block() {
        match b.kind {
            mvd::dem::SET => {}
            // Not an svc stream — the hidden channel, which is where the real usercmds live.
            _ if b.to.is_hidden_channel() => {
                cmds.clear();
                mvd::hidden_usercmds(b.body, &mut cmds);
                for c in cmds.drain(..) {
                    if let Some(slot) = feed.pending_cmds.get_mut(c.player as usize) {
                        *slot = Some(Usercmd {
                            msec: c.msec,
                            angles: Vec3::from_array(c.angles),
                            forward: c.forward,
                            side: c.side,
                            up: c.up,
                            buttons: c.buttons,
                            impulse: c.impulse,
                        });
                    }
                }
            }
            _ => feed.packet(b.time, b.body, b.at),
        }
    }
    feed.finish(path, Vec::new())
}

/// Parse `.qwd` bytes: a single client's recording.
///
/// Framing errors (a truncated record, a bad length, an unknown tag) return `Err` — unlike an MVD,
/// a `.qwd` is not a joinable stream, so a broken record means the file is wrong rather than cut.
/// A packet whose svc stream fails to decode is recorded in [`Demo::warnings`] and skipped, so one
/// corrupt frame near the end doesn't cost you the whole demo.
pub fn parse_qwd_bytes(data: &[u8], path: PathBuf) -> Result<Demo> {
    let total = data.len() as u64;
    let mut cur = Cursor::new(data);
    let mut feed = Feed::new(Format::Qwd);
    let mut demo_cmds = Vec::new();

    // Require `n` more bytes before a read, so a truncated record is a clear error rather than a
    // binrw underflow with no context.
    let require = |pos: u64, n: u64, what: &'static str| -> Result<()> {
        if total - pos < n {
            Err(Error::Truncated { what, at: pos })
        } else {
            Ok(())
        }
    };

    while cur.position() < total {
        let rec_at = cur.position();
        require(rec_at, 5, "record header")?;
        let header: RecordHeader = cur.read_le().map_err(|_| Error::Truncated {
            what: "record header",
            at: rec_at,
        })?;

        match header.kind {
            mvd::dem::CMD => {
                let body_at = cur.position();
                require(body_at, (USERCMD_BYTES + VIEWANGLES_BYTES) as u64, "dem_cmd")?;
                let raw: RawUsercmd = cur.read_le().map_err(|_| Error::Truncated {
                    what: "dem_cmd",
                    at: body_at,
                })?;
                cur.seek(SeekFrom::Current(VIEWANGLES_BYTES as i64))?; // skip the smoothed view angles
                demo_cmds.push(DemoCmd {
                    time: header.time,
                    cmd: raw.into_usercmd(),
                });
            }
            mvd::dem::READ => {
                let len_at = cur.position();
                require(len_at, 4, "dem_read length")?;
                let length: u32 = cur.read_le().map_err(|_| Error::Truncated {
                    what: "dem_read length",
                    at: len_at,
                })?;
                let start = cur.position() as usize;
                let end = start + length as usize;
                if end > data.len() {
                    return Err(Error::LengthOverflow { at: len_at });
                }
                let packet = &data[start..end];
                cur.set_position(end as u64);
                // A connectionless (`0xffffffff`) packet is out-of-band, not a message stream — the
                // trailing "EndOfDemo" marker is one. It carries no frame data, so skip it rather
                // than feed its bytes to the svc parser as if they were opcodes.
                if packet.starts_with(&rtx_proto::protocol::CONNECTIONLESS) {
                    continue;
                }
                // A `.qwd` records whole datagrams, so the svc stream starts after the netchan
                // sequence header. (An MVD records only the stream — no header to skip.)
                let msg = &packet[NETCHAN_HEADER.min(packet.len())..];
                feed.packet(header.time, msg, start);
            }
            mvd::dem::SET => {
                require(cur.position(), 8, "dem_set")?;
                cur.seek(SeekFrom::Current(8))?;
            }
            kind => return Err(Error::UnknownRecord { kind, at: rec_at }),
        }
    }

    Ok(feed.finish(path, demo_cmds))
}

/// Size of the embedded `usercmd_t` in a `dem_cmd` record.
const USERCMD_BYTES: usize = 24;
/// Size of the view-angle vector that trails it, which we skip.
const VIEWANGLES_BYTES: usize = 12;

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-build a minimal demo (one `dem_cmd`, one `dem_set`) and check the framing and the
    /// fixed `usercmd_t` decode — the parts this crate owns, exercised without a demo file on disk.
    #[test]
    fn frames_a_dem_cmd_and_skips_a_dem_set() {
        let mut buf = Vec::new();
        // dem_cmd record: time=1.5, kind=0, then the 24-byte usercmd + 12 view-angle bytes.
        buf.extend_from_slice(&1.5f32.to_le_bytes());
        buf.push(mvd::dem::CMD);
        buf.push(13); // msec
        buf.extend_from_slice(&[0, 0, 0]); // alignment padding
        buf.extend_from_slice(&10.0f32.to_le_bytes()); // pitch
        buf.extend_from_slice(&(-20.0f32).to_le_bytes()); // yaw
        buf.extend_from_slice(&0.0f32.to_le_bytes()); // roll
        buf.extend_from_slice(&800i16.to_le_bytes()); // forward
        buf.extend_from_slice(&(-400i16).to_le_bytes()); // side
        buf.extend_from_slice(&0i16.to_le_bytes()); // up
        buf.push(0b11); // buttons: attack | jump
        buf.push(7); // impulse
        buf.extend_from_slice(&[0u8; VIEWANGLES_BYTES]); // trailing view angles, skipped
                                                         // dem_set record: time=1.5, kind=2, 8 payload bytes.
        buf.extend_from_slice(&1.5f32.to_le_bytes());
        buf.push(mvd::dem::SET);
        buf.extend_from_slice(&[0u8; 8]);

        let dir = std::env::temp_dir();
        let path = dir.join("rtx_qwd_parse_frames_test.qwd");
        std::fs::write(&path, &buf).unwrap();
        let demo = parse_demo(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(demo.demo_cmds.len(), 1);
        assert!(demo.frames.is_empty());
        let c = demo.demo_cmds[0];
        assert_eq!(c.time, 1.5);
        assert_eq!(c.cmd.msec, 13);
        assert_eq!(c.cmd.forward, 800);
        assert_eq!(c.cmd.side, -400);
        assert_eq!(c.cmd.buttons, 0b11);
        assert_eq!(c.cmd.impulse, 7);
        assert_eq!(c.cmd.angles.x, 10.0);
        assert_eq!(c.cmd.angles.y, -20.0);
    }

    /// A record tag that isn't one of the three demo kinds is a hard framing error.
    #[test]
    fn rejects_an_unknown_record_tag() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&0.0f32.to_le_bytes());
        buf.push(9); // not dem_cmd/read/set
        let dir = std::env::temp_dir();
        let path = dir.join("rtx_qwd_parse_badtag_test.qwd");
        std::fs::write(&path, &buf).unwrap();
        let err = parse_demo(&path).unwrap_err();
        std::fs::remove_file(&path).ok();
        assert!(matches!(err, Error::UnknownRecord { kind: 9, .. }));
    }
}

#[cfg(test)]
mod replacement_tests {
    use super::*;
    use rtx_proto::protocol::fte2;
    use rtx_proto::svc::op;

    fn flags(mut value: u32) -> Vec<u8> {
        if value & 0xff00_0000 != 0 {
            value |= 1 << 23;
        }
        if value & 0xffff_0000 != 0 {
            value |= 1 << 15;
        }
        if value & 0xffff_ff00 != 0 {
            value |= 1 << 7;
        }
        let len = if value & (1 << 23) != 0 {
            4
        } else if value & (1 << 15) != 0 {
            3
        } else if value & (1 << 7) != 0 {
            2
        } else {
            1
        };
        value.to_le_bytes()[..len].to_vec()
    }

    fn frame(time: f32, records: &[u8]) -> Vec<u8> {
        let mut packet = vec![86];
        packet.extend(time.to_le_bytes());
        packet.extend(records);
        packet.extend([0, 0]);
        packet
    }

    fn feed(format: Format) -> Feed {
        let mut feed = Feed::new(format);
        feed.proto.apply(0, fte2::REPLACEMENTDELTAS, 0);
        feed
    }

    #[test]
    fn rd_demo_retains_player_state_without_fabricating_recorded_input() {
        let mut feed = feed(Format::Mvd);
        // Independent pinned-FTE payload includes frame300, weapon200, signedskin, predicted
        // movement/velocity and angle16. Player32 maps to slot31; entity16383 stays nonplayer.
        let fixture = include_bytes!("../../rtx-proto/tests/fixtures/replacement_deltas/fte-base-2.bin");
        let mut records = vec![0, 128, 32, 0];
        records.extend(fixture);
        records.extend([255, 63, 0]);
        feed.packet(1., &frame(10., &records), 0);
        feed.packet(2., &frame(11., &[]), 100);
        assert!(feed.warnings.is_empty(), "{:?}", feed.warnings);
        assert_eq!(feed.frames.len(), 2);
        assert_eq!(feed.players[31].frames, 2);
        for frame in &feed.frames {
            assert_eq!(frame.player, 31);
            assert_eq!(frame.origin, Vec3::new(1.25, -2.5, 3.75));
            assert_eq!(frame.weaponframe, Some(200));
            assert_eq!(frame.velocity, Some(Vec3::new(100., -200., 300.)));
            assert!((frame.angles.x + 90.).abs() < 0.02);
            assert!(frame.command.is_none(), "prediction fields are not a recorded usercmd");
            assert_eq!(frame.on_ground, Some(false));
        }
        assert_eq!((feed.frames[0].time, feed.frames[1].time), (1., 2.));
    }

    /// Append an MVD block; a zero audience mask carries hidden data rather than svc messages.
    fn mvd_block(bytes: &mut Vec<u8>, elapsed: u8, hidden: bool, body: &[u8]) {
        bytes.extend([elapsed, if hidden { mvd::dem::MULTIPLE } else { mvd::dem::ALL }]);
        if hidden {
            bytes.extend(0u32.to_le_bytes());
        }
        bytes.extend((body.len() as u32).to_le_bytes());
        bytes.extend(body);
    }

    #[test]
    fn rd_mvd_uses_recorded_hidden_input_once_including_zero_commands() {
        let mut bytes = Vec::new();
        mvd_block(&mut bytes, 0, false, &serverdata(Format::Mvd, true));
        let mut cmd = vec![31, 0, 23]; // player, dropnum, msec
        for angle in [10.0f32, 200.0, -15.0] {
            cmd.extend(angle.to_le_bytes());
        }
        for movement in [400i16, -40, 12] {
            cmd.extend(movement.to_le_bytes());
        }
        cmd.extend([3, 7]); // attack+jump, impulse
        let mut hidden = (cmd.len() as u32).to_le_bytes().to_vec();
        hidden.extend(mvd::hidden::USERCMD.to_le_bytes());
        hidden.extend(&cmd);
        mvd_block(&mut bytes, 0, true, &hidden);

        let fixture = include_bytes!("../../rtx-proto/tests/fixtures/replacement_deltas/fte-base-2.bin");
        let mut records = vec![0, 128, 32, 0];
        records.extend(fixture);
        mvd_block(&mut bytes, 100, false, &frame(10., &records));
        mvd_block(&mut bytes, 100, false, &frame(11., &[]));

        // An explicitly recorded all-zero command is present input, not an absent command.
        let mut zero = vec![23, 0, 0, 0, 1, 0, 31]; // hidden length, type, player
        zero.extend([0; 22]);
        mvd_block(&mut bytes, 0, true, &zero);
        mvd_block(&mut bytes, 100, false, &frame(12., &[]));
        let demo = parse_mvd_bytes(&bytes, PathBuf::from("rd-hidden-input.mvd"));
        assert!(demo.warnings.is_empty(), "{:?}", demo.warnings);
        assert_eq!(demo.frames.len(), 3);
        let first = &demo.frames[0];
        let cmd = first
            .command
            .expect("hidden input must replace the synthetic prediction command");
        assert_eq!(cmd.msec, 23);
        assert_eq!(cmd.angles, Vec3::new(10.0, 200.0, -15.0));
        assert_eq!(
            (cmd.forward, cmd.side, cmd.up, cmd.buttons, cmd.impulse),
            (400, -40, 12, 3, 7)
        );
        assert!((first.angles.x + 90.0).abs() < 0.02, "state angles still come from RD");
        assert_eq!(first.velocity, Some(Vec3::new(100.0, -200.0, 300.0)));
        assert!(
            demo.frames[1].command.is_none(),
            "one input must not be replayed on later retained frames"
        );
        let zero = demo.frames[2].command.expect("zero input is still present");
        assert_eq!(
            (zero.msec, zero.forward, zero.side, zero.up, zero.buttons, zero.impulse),
            (0, 0, 0, 0, 0, 0)
        );
        assert_eq!(zero.angles, Vec3::ZERO);
    }

    #[test]
    fn rd_demo_rejects_malformed_tail_and_waits_for_reset_recovery() {
        for format in [Format::Mvd, Format::Qwd] {
            let mut feed = feed(format);
            feed.packet(1., &frame(10., &[0, 128, 1, 0, 0]), 0);
            assert_eq!(feed.frames.len(), 1);
            let mut bad = frame(11., &[1, 128]);
            bad.push(255);
            feed.packet(2., &bad, 100);
            feed.packet(3., &frame(12., &[]), 200);
            assert_eq!(feed.frames.len(), 1, "failed packet must invalidate retained decoding");
            assert_eq!(feed.warnings.len(), 1);
            feed.packet(4., &frame(13., &[0, 128, 2, 0, 0]), 300);
            assert_eq!(feed.frames.len(), 2);
            assert_eq!(feed.frames[1].player, 1);
            assert_eq!(feed.frames[1].time, 4.);
            feed.packet(5., &frame(14., &[2, 128]), 400);
            feed.packet(6., &frame(15., &[]), 500);
            assert_eq!(feed.frames.len(), 2, "removed player must stay removed");
        }
    }

    fn serverdata(format: Format, replacement: bool) -> Vec<u8> {
        let mut packet = vec![op::SERVERDATA];
        if replacement {
            packet.extend(rtx_proto::protocol::magic::FTE2.to_le_bytes());
            packet.extend(fte2::REPLACEMENTDELTAS.to_le_bytes());
        }
        packet.extend(rtx_proto::protocol::VERSION.to_le_bytes());
        packet.extend(2i32.to_le_bytes());
        packet.extend(b"qw\0");
        if format == Format::Mvd {
            packet.extend(0f32.to_le_bytes());
        } else {
            packet.push(31);
        }
        packet.extend(b"next-map\0");
        for _ in 0..10 {
            packet.extend(0f32.to_le_bytes());
        }
        packet
    }

    #[test]
    fn rd_baselines_survive_world_reset_and_reuse_but_are_cleared_on_map_change() {
        for format in [Format::Mvd, Format::Qwd] {
            let mut feed = feed(format);
            let mut baseline = vec![op::FTE_SPAWNBASELINE2, 1, 0];
            baseline.extend(flags(
                (1 << 0) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 29) | (1 << 5),
            ));
            baseline.extend(65535u16.to_le_bytes());
            baseline.extend(0x1234abcd_u32.to_le_bytes());
            baseline.extend(511u16.to_le_bytes());
            baseline.extend((-200i16).to_le_bytes());
            feed.packet(0., &baseline, 0);
            let mut records = vec![0, 128, 1, 0];
            records.extend(flags((1 << 8) | (1 << 6)));
            records.extend([8 | 128, 3 | 128, 255, 255]); // move ground; weapon32767.
            records.extend([255, 63, 0]);
            feed.packet(1., &frame(10., &records), 100);
            assert_eq!(feed.frames.len(), 1);
            assert_eq!(feed.frames[0].weaponframe, Some(32767));
            assert_eq!(feed.frames[0].on_ground, Some(true));
            let state = feed.rd.get(1).unwrap();
            assert_eq!(
                (state.frame, state.modelindex, state.skinnum, state.effects),
                (65535, 511, -200, 0x1234abcd)
            );
            assert!(feed.rd.get(16383).is_some());
            feed.packet(2., &frame(11., &[1, 128, 255, 191]), 200);
            assert!(feed.rd.get(1).is_none() && feed.rd.get(16383).is_none());
            feed.packet(3., &frame(12., &[0, 128, 1, 0, 128, 1]), 300);
            assert_eq!(feed.rd.get(1).unwrap().frame, 65535);
            assert_eq!(feed.frames[1].weaponframe, Some(0));
            feed.packet(4., &serverdata(format, true), 400);
            feed.packet(5., &frame(13., &[0, 128, 1, 0, 128, 1]), 500);
            assert_eq!(feed.rd.get(1).unwrap().frame, 0);
            assert_eq!(feed.frames.len(), 3);
            assert!(feed.warnings.is_empty(), "{:?}", feed.warnings);
        }
    }

    #[test]
    fn malformed_serverdata_does_not_switch_demo_protocol_or_clear_last_good_world() {
        let mut feed = feed(Format::Mvd);
        feed.packet(1., &frame(10., &[0, 128, 1, 0, 0]), 0);
        let mut malformed = serverdata(Format::Mvd, false);
        malformed.push(255);
        feed.packet(2., &malformed, 100);
        assert_eq!(feed.proto.fte2, fte2::REPLACEMENTDELTAS);
        assert!(feed.rd.get(1).is_some());
        assert!(!feed.rd.can_delta());
        assert_eq!(feed.frames.len(), 1);
        feed.packet(3., &frame(12., &[0, 128, 2, 0, 0]), 200);
        assert_eq!(feed.frames.len(), 2);
        assert_eq!(feed.frames[1].player, 1);
    }

    #[test]
    fn classic_mvd_still_carries_player_deltas_without_prediction_fields() {
        let mut feed = Feed::new(Format::Mvd);
        let mut first = vec![op::PLAYERINFO, 0];
        first.extend((svc::df::ORIGIN | svc::df::ANGLES | svc::df::WEAPONFRAME | svc::df::DEAD).to_le_bytes());
        first.push(2); // classic animation frame
        first.extend(80i16.to_le_bytes());
        first.extend(16384i16.to_le_bytes());
        first.push(255);
        feed.packet(1., &first, 0);
        feed.packet(2., &[op::PLAYERINFO, 0, 0, 0, 3], 100);
        assert_eq!(feed.frames.len(), 2);
        assert_eq!(feed.frames[0].origin.x, 10.);
        assert_eq!(feed.frames[1].origin.x, 10.);
        assert_eq!(feed.frames[0].angles.x, -30.);
        assert_eq!(feed.frames[1].angles.x, -30.);
        assert_eq!(feed.frames[0].weaponframe, Some(255));
        assert_eq!(feed.frames[1].weaponframe, None);
        assert!(feed.frames[0].dead && !feed.frames[1].dead);
        for frame in feed.frames {
            assert!(frame.velocity.is_none() && frame.on_ground.is_none());
        }
        assert!(feed.warnings.is_empty(), "{:?}", feed.warnings);
    }

    #[test]
    fn both_demo_containers_deliver_rd_frames_through_public_parsers() {
        for format in [Format::Mvd, Format::Qwd] {
            let packets = [
                serverdata(format, true),
                frame(10., &[0, 128, 1, 0, 0]),
                frame(11., &[]),
            ];
            let mut bytes = Vec::new();
            for (index, packet) in packets.iter().enumerate() {
                if format == Format::Mvd {
                    bytes.extend([100, mvd::dem::ALL]);
                    bytes.extend((packet.len() as u32).to_le_bytes());
                } else {
                    bytes.extend((index as f32 * 0.1).to_le_bytes());
                    bytes.push(mvd::dem::READ);
                    bytes.extend(((packet.len() + NETCHAN_HEADER) as u32).to_le_bytes());
                    bytes.extend((index as u32 + 1).to_le_bytes());
                    bytes.extend(0u32.to_le_bytes());
                }
                bytes.extend(packet);
            }
            let path = PathBuf::from("synthetic-rd-demo");
            let demo = if format == Format::Mvd {
                parse_mvd_bytes(&bytes, path)
            } else {
                parse_qwd_bytes(&bytes, path).unwrap()
            };
            assert!(demo.warnings.is_empty(), "{:?}", demo.warnings);
            assert_eq!(demo.frames.len(), 2);
            assert_eq!(demo.frames[0].player, 0);
            assert!(demo.frames.iter().all(|frame| frame.command.is_none()));
            assert_eq!(demo.players[0].frames, 2);
            assert_eq!(demo.local_player, if format == Format::Qwd { Some(31) } else { None });
        }
    }

    #[test]
    fn invalid_rd_classic_baseline_rejects_earlier_frame_in_same_packet() {
        let mut feed = feed(Format::Mvd);
        feed.packet(1., &frame(10., &[0, 128, 1, 0, 0]), 0);
        let mut packet = frame(11., &[1, 128]);
        packet.extend([op::SPAWNBASELINE, 0, 64]); // legal ordinary ref, exceeds application slots.
        packet.extend([0; 13]);
        feed.packet(2., &packet, 100);
        assert!(feed.rd.get(1).is_some());
        assert!(!feed.rd.can_delta());
        assert_eq!(feed.frames.len(), 1);
        assert_eq!(feed.warnings.len(), 1);
    }
}
