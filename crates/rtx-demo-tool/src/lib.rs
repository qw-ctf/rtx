// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-demo-tool` — read and analyze QuakeWorld demo files (`.qwd` today).
//!
//! A `.qwd` is a flat log of timestamped records: the local client's own `dem_cmd` inputs, the
//! `dem_read` server→client packets it received, and `dem_set` bookkeeping. The records are the
//! *only* thing this crate parses — a `dem_read` block is a recorded network packet, and the
//! messages inside it are decoded by [`rtx_proto::svc`], the same codec the live client speaks.
//! So the split is clean: [`binrw`] reads the fixed demo framing here, `rtx-proto` reads the wire.
//!
//! [`parse_demo`] hands back the per-frame [`Frame`]s and the local player's [`DemoCmd`]s; the
//! [`analysis`] module turns those into per-player motion [`Track`](analysis::Track)s (position and
//! speed over time) with summary stats. The `qwd` binary is the CLI over both: `qwd dump` emits the
//! CSV the old `qwd_dump.py` did, `qwd analyze` prints a movement report.

use std::io::{Cursor, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use binrw::{BinRead, BinReaderExt};
use glam::Vec3;
use rtx_proto::protocol::ProtoState;
use rtx_proto::svc::{self, MoveVars, PlayerInfo, SvcEvent, Usercmd};

pub mod analysis;

pub use analysis::{Motion, Summary, Track};

/// Demo record kinds (`dem_*`), the one-byte tag after each record's float timestamp.
mod dem {
    /// The local client's own `usercmd` for this frame (a fixed 24-byte struct + 12 view-angle bytes).
    pub const CMD: u8 = 0;
    /// A recorded server→client packet: a `u32` length then that many bytes of message stream.
    pub const READ: u8 = 1;
    /// A camera/viewangle set; 8 bytes we skip.
    pub const SET: u8 = 2;
}

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

/// The local client's own input for one frame, from a `dem_cmd` record.
#[derive(Clone, Copy, Debug)]
pub struct DemoCmd {
    /// Demo timestamp of the record.
    pub time: f32,
    /// The decoded command — view angles, moves, buttons, msec.
    pub cmd: Usercmd,
}

/// One `svc_playerinfo` occurrence, tagged with the demo time of the packet that carried it.
#[derive(Clone, Debug)]
pub struct Frame {
    /// Demo timestamp of the enclosing `dem_read` record.
    pub time: f32,
    /// The player state, as decoded by [`rtx_proto::svc`].
    pub info: PlayerInfo,
}

/// Everything one demo yields: the framing-derived context plus the two event streams.
#[derive(Clone, Debug)]
pub struct Demo {
    /// The file this came from.
    pub path: PathBuf,
    /// The negotiated protocol state at end of file (coord/angle widths, extension masks).
    pub proto: ProtoState,
    /// The recording client's own player slot, from the last `svc_serverdata`.
    pub local_player: Option<u8>,
    /// The server's physics constants, from the last `svc_serverdata`.
    pub movevars: Option<MoveVars>,
    /// The local player's own `dem_cmd` inputs, in file order (ascending time).
    pub demo_cmds: Vec<DemoCmd>,
    /// Every `svc_playerinfo` seen, in file order.
    pub frames: Vec<Frame>,
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

/// Parse a `.qwd` file into its movement data.
///
/// Framing errors (a truncated record, a bad length, an unknown tag) return `Err`. A packet whose
/// svc stream fails to decode is recorded in [`Demo::warnings`] and skipped, so one corrupt frame
/// near the end doesn't cost you the whole demo.
pub fn parse_demo(path: impl AsRef<Path>) -> Result<Demo> {
    let path = path.as_ref().to_path_buf();
    let data = std::fs::read(&path)?;
    let total = data.len() as u64;
    let mut cur = Cursor::new(data.as_slice());

    let mut proto = ProtoState::new();
    let mut local_player = None;
    let mut movevars = None;
    let mut demo_cmds = Vec::new();
    let mut frames = Vec::new();
    let mut warnings = Vec::new();

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
            dem::CMD => {
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
            dem::READ => {
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
                // A normal packet leads with the netchan sequence header; the svc stream, which is
                // what rtx-proto decodes, starts after it.
                let msg = &packet[NETCHAN_HEADER.min(packet.len())..];
                match svc::parse(&mut proto, msg) {
                    Ok(events) => {
                        for ev in events {
                            match ev {
                                SvcEvent::PlayerInfo(info) => frames.push(Frame {
                                    time: header.time,
                                    info: *info,
                                }),
                                SvcEvent::ServerData(sd) => {
                                    local_player = Some(sd.playernum);
                                    movevars = Some(sd.movevars);
                                }
                                _ => {}
                            }
                        }
                    }
                    Err(e) => warnings.push(format!("dem_read at offset {start}: {e}")),
                }
            }
            dem::SET => {
                require(cur.position(), 8, "dem_set")?;
                cur.seek(SeekFrom::Current(8))?;
            }
            kind => return Err(Error::UnknownRecord { kind, at: rec_at }),
        }
    }

    Ok(Demo {
        path,
        proto,
        local_player,
        movevars,
        demo_cmds,
        frames,
        warnings,
    })
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
        buf.push(dem::CMD);
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
        buf.push(dem::SET);
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
