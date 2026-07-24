// SPDX-License-Identifier: AGPL-3.0-or-later

//! The typed MCP<->game control protocol, spoken as length-framed [msgpack].
//!
//! The bot-control channel used to be newline-delimited text with hand-built JSON replies. This crate
//! replaces that with a compact, typed wire: the MCP sends a [`Request`] (an `id` plus a [`Cmd`]) and
//! the game answers with a [`Msg`] — either a `Reply` correlated by `id` carrying a typed [`Resp`] (or
//! an error string), or an async [`Event`] emitted by a puppet order as it plays out. Both crates
//! depend on this schema, so it is single-sourced; the MCP decodes typed values and re-serializes them
//! as JSON for Claude, and the game builds typed values instead of formatting JSON strings.
//!
//! Frames are `[u32 little-endian byte length][msgpack payload]` (see [`to_frame`] / [`read_frame`]).
//! World positions are `[x, y, z]`; flight traces are `[t, x, y, z, vx, vy, vz]` rows. Descriptive
//! enum labels (weapon, link kind, hazard, oracle mode, …) travel as strings — they are display
//! labels, so the schema stays typed at the structural level without mirroring a dozen game enums.
//!
//! [msgpack]: https://msgpack.org/

use std::io::{self, Read};

use rtx_auditlog::AuditFrame;
use serde::{Deserialize, Serialize};

/// A 3D world position, `[x, y, z]`.
pub type Vec3 = [f32; 3];
/// One flight-trace sample: `[t, x, y, z, vx, vy, vz]`.
pub type TrajRow = [f32; 7];

// ---------------------------------------------------------------------------------------------------
// Framing codec
// ---------------------------------------------------------------------------------------------------

/// Encode `v` as msgpack behind a 4-byte little-endian length prefix — one wire frame.
pub fn to_frame<T: Serialize>(v: &T) -> Vec<u8> {
    let body = rmp_serde::to_vec_named(v).expect("msgpack encode");
    let mut frame = Vec::with_capacity(body.len() + 4);
    frame.extend_from_slice(&(body.len() as u32).to_le_bytes());
    frame.extend_from_slice(&body);
    frame
}

/// Largest frame payload we will allocate for. A guard against a corrupted or garbage length prefix
/// (a peer that died mid-frame, a desynced stream) blowing up into a multi-gigabyte allocation and
/// crashing the reader. Real frames — even a full status or a long trajectory — are far under this.
pub const MAX_FRAME: usize = 64 * 1024 * 1024;

/// Read one length-prefixed frame's payload bytes, or `Ok(None)` at a clean end of stream. A length
/// past [`MAX_FRAME`] is treated as a protocol error (so the caller reconnects) rather than allocated.
pub fn read_frame<R: Read>(r: &mut R) -> io::Result<Option<Vec<u8>>> {
    let mut len = [0u8; 4];
    match r.read_exact(&mut len) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let n = u32::from_le_bytes(len) as usize;
    if n > MAX_FRAME {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "control frame length too large",
        ));
    }
    let mut body = vec![0u8; n];
    r.read_exact(&mut body)?;
    Ok(Some(body))
}

/// Decode a frame payload as `T`.
pub fn decode<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, rmp_serde::decode::Error> {
    rmp_serde::from_slice(bytes)
}

// ---------------------------------------------------------------------------------------------------
// Requests (MCP -> game)
// ---------------------------------------------------------------------------------------------------

/// A request frame: a caller-chosen `id` echoed back on the matching [`Msg::Reply`], and the command.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Request {
    pub id: i64,
    pub cmd: Cmd,
}

/// Every bot-control verb.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Cmd {
    /// Server + strategy status (map, navmesh, match, oracle, per-bot state).
    Status,
    /// Queue the team-match start.
    MatchStart,
    /// Every rocket-jump link the navmesh generated.
    Links,
    /// The map's bot-goal items.
    Items,
    /// Make a bot fit for a rocket-jump test.
    Prep { bot: u32, health: f32, rockets: f32 },
    /// Teleport a bot to a world point.
    Teleport { bot: u32, pos: Vec3 },
    /// Order a bot to run to a world point (emits `Arrived` / `GotoStall`).
    Goto { bot: u32, pos: Vec3 },
    /// Order a bot to fly a rocket-jump link (emits `RjResult`).
    Rj { bot: u32, link: u32 },
    /// Order a bot to fly a non-RJ link (emits `FlyResult`).
    Fly { bot: u32, link: u32 },
    /// Park a bot (clear any order).
    Hold { bot: u32 },
    /// Stop a bot and clear its puppet state.
    Stop { bot: u32 },
    /// Set a live server cvar.
    Set { name: String, value: String },
    /// Read a cvar's string and float value.
    Get { name: String },
    /// Run a raw console command.
    RunCmd { raw: String },
    /// Inspect the navmesh cell nearest a world point.
    Cell { pos: Vec3 },
    /// Dump a bot's current A* route.
    Route { bot: u32 },
    /// Dump the tail of a bot's `rtx_bot_debug` audit ring.
    Audit { bot: u32, lines: u32 },
    /// List generated curl links.
    Curls,
    /// Fetch the current map's raw BSP file, so a viewer can render the world without a local copy.
    Bsp,
    /// Probe the build-time curl certifier.
    Probe {
        takeoff: Vec3,
        tgt: Vec3,
        psi0: f32,
        runway: f32,
    },
    /// Search the offline sim for a speed-curl jump.
    Curl { src: Vec3, tgt: Vec3 },
    /// Hand-plant a SpeedJump link into the live graph.
    PlanLink {
        from: Vec3,
        takeoff: Vec3,
        tgt: Vec3,
        v_req: f32,
    },
}

// ---------------------------------------------------------------------------------------------------
// Messages (game -> MCP): replies and async events
// ---------------------------------------------------------------------------------------------------

/// One outbound frame from the game: a reply to a request, or an async puppet event.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Msg {
    /// Answer to the request with this `id`. `Err` carries the failure message.
    Reply { id: i64, result: Result<Resp, String> },
    /// A puppet order's lifecycle event (not correlated to a request).
    Event(Event),
}

/// A typed command result. `Ack`/`Queued` cover the verbs whose reply is just a confirmation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Resp {
    Status(Box<StatusResp>),
    /// A verb that only queues work (`MatchStart`, `RunCmd`).
    Queued,
    Links(Vec<RjLink>),
    Items(Vec<ItemInfo>),
    Prep {
        bot: u32,
        health: f32,
        rockets: f32,
    },
    Teleport {
        bot: u32,
        origin: Vec3,
    },
    Goto {
        bot: u32,
        target: Vec3,
    },
    Rj {
        bot: u32,
        link: u32,
    },
    Fly {
        bot: u32,
        link: u32,
    },
    /// `Hold` / `Stop` — just the bot id.
    Ack {
        bot: u32,
    },
    Set {
        name: String,
        value: String,
    },
    Get {
        name: String,
        string: String,
        value: f32,
    },
    Cell(CellResp),
    Route(RouteResp),
    Audit(AuditResp),
    Curls(Vec<CurlLink>),
    Probe(ProbeResp),
    Curl(CurlResp),
    PlanLink(PlanLinkResp),
    Bsp(Box<BspResp>),
}

/// The current map's raw BSP file plus its name, so a viewer can parse the render lumps and draw the
/// world without needing a local copy of the map. `bytes` travels as a msgpack `bin` (not an int
/// array) via `serde_bytes`, so a multi-MB map stays compact on the wire.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BspResp {
    pub map: String,
    #[serde(with = "serde_bytes")]
    pub bytes: Vec<u8>,
}

// --- status -----------------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StatusResp {
    pub map: String,
    pub time: f32,
    /// `"ready"`, `"building"`, or `"none"`.
    pub navmesh: String,
    pub cells: u32,
    pub links: u32,
    pub rj_links: u32,
    pub match_: MatchInfo,
    pub oracle: OracleInfo,
    pub bots: Vec<BotStatus>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MatchInfo {
    pub mode: String,
    pub format: String,
    pub phase: String,
    pub teams: u32,
    pub size: u32,
    pub teamplay: i32,
    pub timelimit: f32,
    pub fraglimit: f32,
    pub live_until: f32,
    pub scores: Vec<i32>,
    pub roster: Vec<RosterEntry>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RosterEntry {
    pub name: String,
    pub team: u32,
}

/// A referenced entity (enemy, goal item), or absent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EntRef {
    pub ent: u32,
    pub name: String,
    pub classname: String,
    pub origin: Vec3,
    pub solid: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ammo {
    pub shells: i32,
    pub nails: i32,
    pub rockets: i32,
    pub cells: i32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BotGoal {
    pub item: Option<EntRef>,
    pub commit: String,
    pub since: f32,
    pub next_item: Option<EntRef>,
    pub hold_item: Option<EntRef>,
    pub hold_for: Option<EntRef>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteHead {
    pub pos: u32,
    pub len: u32,
    pub next: Option<RouteNext>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteNext {
    pub link: u32,
    pub kind: String,
    pub target: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BotStatus {
    pub ent: u32,
    pub client: i32,
    pub name: String,
    pub team: i32,
    pub team_name: String,
    pub frags: i32,
    pub origin: Vec3,
    pub health: f32,
    pub armor: f32,
    pub armor_type: f32,
    pub weapon: String,
    pub items: String,
    pub ammo: Ammo,
    pub on_ground: bool,
    pub alive: bool,
    pub order: String,
    pub posture: String,
    pub known_enemy: Option<EntRef>,
    pub goal: BotGoal,
    pub route: RouteHead,
    pub rj_phase: String,
    pub speed: f32,
    pub bhop: String,
    pub bhop_peak: f32,
}

// --- oracle -----------------------------------------------------------------------------------------

/// The seven evaluation counters, shared by the top-level summary and each nugget-kind breakdown.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvalCounts {
    pub treated: u32,
    pub treated_success: u32,
    pub controls: u32,
    pub control_success: u32,
    pub applied: u32,
    pub invalidated: u32,
    pub pending: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EpisodeEval {
    pub counts: EvalCounts,
    pub by_kind: Vec<(String, EvalCounts)>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Eval {
    pub counts: EvalCounts,
    pub by_kind: Vec<(String, EvalCounts)>,
    pub episodes: EpisodeEval,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Communication {
    pub proposed: u32,
    pub communicated: u32,
    pub refreshed: u32,
    pub suppressed: u32,
    pub superseded: u32,
    pub arm_clears: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Nugget {
    pub recipient: i32,
    pub kind: String,
    pub target_cell: u32,
    pub subject: i32,
    pub confidence: f32,
    pub decision_at: f32,
    pub evidence_at: f32,
    pub expires_at: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanTeam {
    pub team: u32,
    pub mode: String,
    pub control: String,
    pub nuggets: Vec<Nugget>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    pub generation: u64,
    pub at: f32,
    pub teams: Vec<PlanTeam>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OracleInfo {
    pub running: bool,
    pub epoch: u64,
    pub last_output: f32,
    pub plan: Option<Plan>,
    pub communication: Communication,
    pub eval: Eval,
}

// --- rj / items / cell / route / audit / curl -------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjLink {
    pub link: u32,
    pub src: Vec3,
    pub tgt: Vec3,
    pub fire_pitch: f32,
    pub fire_yaw: f32,
    pub fire_delay: f32,
    pub airtime: f32,
    pub self_damage: f32,
    pub v0: Vec3,
    pub blast: Vec3,
    pub pos_blast: Vec3,
    pub land: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NavCell {
    pub cell: u32,
    pub origin: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ItemInfo {
    pub ent: u32,
    pub classname: String,
    pub origin: Vec3,
    pub available: bool,
    pub nav: Option<NavCell>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellLinkOut {
    pub link: u32,
    pub kind: String,
    pub to: Vec3,
    pub cost: f32,
    pub tgt_hazard: String,
    pub hazard_hp: f32,
    pub water_extra: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellLinkIn {
    pub link: u32,
    pub kind: String,
    pub from: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellResp {
    pub cell: u32,
    pub origin: Vec3,
    pub hazard: String,
    pub out: Vec<CellLinkOut>,
    pub incoming: Vec<CellLinkIn>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteLeg {
    pub i: u32,
    pub link: u32,
    pub kind: String,
    pub src: Vec3,
    pub tgt: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteResp {
    pub bot: u32,
    pub route_pos: u32,
    pub origin: Vec3,
    pub legs: Vec<RouteLeg>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AuditResp {
    pub bot: u32,
    pub count: u32,
    pub frames: Vec<AuditFrame>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CurlLink {
    pub link: u32,
    pub from: Vec3,
    pub takeoff: Vec3,
    pub tgt: Vec3,
    pub v_req: f32,
    pub gain: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cert {
    pub v_req: f32,
    pub gain: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProbeGain {
    pub gain: f32,
    pub land: Vec3,
    pub miss_xy: f32,
    pub miss_z: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProbeResp {
    pub v_deliver: f32,
    pub certified: Option<Cert>,
    pub gains: Vec<ProbeGain>,
}

/// A curl-jump search result. When `found` is false only `chord` is meaningful.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CurlResp {
    pub found: bool,
    pub chord: f32,
    pub v0: f32,
    pub psi0: f32,
    pub gain: f32,
    pub miss_xy: f32,
    pub land: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanLinkResp {
    pub link: u32,
    pub from_cell: u32,
    pub to_cell: u32,
    pub from: Vec3,
    pub tgt: Vec3,
    pub takeoff: Vec3,
    pub v_req: f32,
    pub airtime: f32,
    pub cost: f32,
}

// ---------------------------------------------------------------------------------------------------
// Events (game -> MCP, async)
// ---------------------------------------------------------------------------------------------------

/// A puppet order's lifecycle event, emitted as the order plays out over frames.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Event {
    /// A `Goto` reached its target.
    Arrived {
        bot: u32,
        t: f32,
        origin: Vec3,
        target: Vec3,
        dist: f32,
        traj: Vec<TrajRow>,
    },
    /// A `Goto` stalled (no progress) — the source is (currently) inaccessible.
    GotoStall {
        bot: u32,
        t: f32,
        origin: Vec3,
        target: Vec3,
        dist: f32,
        best: f32,
        secs: f32,
        traj: Vec<TrajRow>,
    },
    /// A rocket-jump attempt finished (any terminal outcome).
    RjResult(Box<RjResult>),
    /// A fly-link attempt finished (landed, timed out, …).
    FlyResult(FlyResult),
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjSolved {
    pub pitch: f32,
    pub yaw: f32,
    pub delay: f32,
    pub airtime: f32,
    pub self_damage: f32,
    pub v0: Vec3,
    pub blast: Vec3,
    pub pos_blast: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjBias {
    pub delay: f32,
    pub pitch: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjPress {
    pub t: f32,
    pub origin: Vec3,
    pub view: [f32; 2],
    pub aim_err: f32,
    pub stance_off_xy: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjFire {
    pub t: f32,
    pub delay: f32,
    pub origin: Vec3,
    pub view: [f32; 2],
    pub pitch_err: f32,
    pub yaw_err: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjLand {
    pub t: f32,
    pub origin: Vec3,
    pub miss_xy: f32,
    pub miss_z: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RjResult {
    pub bot: u32,
    pub link: u32,
    /// Terminal outcome label (`landed`, `landed_off`, `overran`, `stance_timeout`, …).
    pub outcome: String,
    pub src: Vec3,
    pub tgt: Vec3,
    pub solved: RjSolved,
    pub bias: RjBias,
    pub press: Option<RjPress>,
    pub fire: Option<RjFire>,
    pub land: Option<RjLand>,
    pub traj: Vec<TrajRow>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlyResult {
    pub bot: u32,
    pub link: u32,
    pub on_target: bool,
    pub timeout: bool,
    pub land: Vec3,
    pub target: Vec3,
    pub miss_xy: f32,
    pub miss_z: f32,
    pub takeoff_speed: f32,
    pub peak: f32,
    pub traj: Vec<TrajRow>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<T>(v: &T) -> T
    where
        T: Serialize + for<'de> Deserialize<'de>,
    {
        let frame = to_frame(v);
        // The length prefix matches the payload, and the payload decodes back to an equal value.
        let n = u32::from_le_bytes(frame[..4].try_into().unwrap()) as usize;
        assert_eq!(n, frame.len() - 4);
        decode(&frame[4..]).unwrap()
    }

    #[test]
    fn request_roundtrips() {
        let r = Request {
            id: 7,
            cmd: Cmd::Goto {
                bot: 2,
                pos: [1.0, 2.0, 3.0],
            },
        };
        assert_eq!(roundtrip(&r), r);
    }

    #[test]
    fn reply_ok_and_err_roundtrip() {
        let ok = Msg::Reply {
            id: 3,
            result: Ok(Resp::Ack { bot: 2 }),
        };
        assert_eq!(roundtrip(&ok), ok);
        let err = Msg::Reply {
            id: 4,
            result: Err("no such bot 9".to_string()),
        };
        assert_eq!(roundtrip(&err), err);
    }

    #[test]
    fn audit_reply_roundtrips_frames() {
        let mut f = AuditFrame::default();
        f.speed = 812.0;
        f.bhop = rtx_auditlog::Bhop::Hop;
        let msg = Msg::Reply {
            id: 1,
            result: Ok(Resp::Audit(AuditResp {
                bot: 2,
                count: 1,
                frames: vec![f],
            })),
        };
        let back = roundtrip(&msg);
        assert_eq!(back, msg);
    }

    #[test]
    fn bsp_reply_roundtrips_bytes() {
        // The BSP payload rides as raw `serde_bytes` — verify a non-UTF-8 blob survives intact.
        let msg = Msg::Reply {
            id: 8,
            result: Ok(Resp::Bsp(Box::new(BspResp {
                map: "dm3".to_string(),
                bytes: vec![0x1d, 0x00, 0xff, 0x80, 0x01, 0x02, 0x03],
            }))),
        };
        assert_eq!(roundtrip(&msg), msg);
    }

    #[test]
    fn event_roundtrips() {
        let ev = Msg::Event(Event::FlyResult(FlyResult {
            bot: 2,
            link: 5,
            on_target: true,
            timeout: false,
            land: [10.0, 20.0, 30.0],
            target: [11.0, 21.0, 31.0],
            miss_xy: 1.4,
            miss_z: 1.0,
            takeoff_speed: 500.0,
            peak: 812.0,
            traj: vec![[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]],
        }));
        assert_eq!(roundtrip(&ev), ev);
    }

    #[test]
    fn two_frames_read_back_in_order() {
        let a = to_frame(&Request {
            id: 1,
            cmd: Cmd::Status,
        });
        let b = to_frame(&Request {
            id: 2,
            cmd: Cmd::Audit { bot: 2, lines: 50 },
        });
        let mut stream: Vec<u8> = Vec::new();
        stream.extend_from_slice(&a);
        stream.extend_from_slice(&b);
        let mut cur = std::io::Cursor::new(stream);
        let f1 = read_frame(&mut cur).unwrap().unwrap();
        let f2 = read_frame(&mut cur).unwrap().unwrap();
        assert!(read_frame(&mut cur).unwrap().is_none());
        assert_eq!(decode::<Request>(&f1).unwrap().id, 1);
        assert_eq!(decode::<Request>(&f2).unwrap().cmd, Cmd::Audit { bot: 2, lines: 50 });
    }
}
