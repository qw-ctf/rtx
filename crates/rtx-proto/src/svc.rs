// SPDX-License-Identifier: AGPL-3.0-or-later

//! The server→client message stream: `svc_*` opcodes in, typed [`SvcEvent`]s out.
//!
//! One packet carries many messages back to back, each a one-byte opcode followed by a payload
//! whose shape depends on the opcode — and, for several of them, on the negotiated extensions. The
//! stream is **self-delimiting only if you understand every message**: there are no lengths and no
//! framing, so mis-sizing one payload shifts everything after it. That's why an unknown opcode is a
//! hard [`ParseError`] rather than something to skip: by the time we see it we've already lost our
//! place, and the honest thing is to say so with a hexdump instead of inventing entities from
//! misaligned bytes.
//!
//! The parser is **world-stateless**. It decodes bytes into values and does not know what an entity
//! is, which player we are, or what a sound means. Entity deltas come out as
//! [`EntityDelta`]s — per-field `Option`s saying "this changed to that" — and the caller applies
//! them against whatever it keeps. The one exception is [`ProtoState`], which the parser must both
//! read and write, because `svc_serverdata` renegotiates the coord width *mid-packet* and every
//! subsequent read in the same datagram depends on it.
//!
//! Ported from ezQuake's `src/cl_parse.c` and `src/cl_ents.c`.

use glam::Vec3;

use crate::protocol::{fte, mvd1, z_ext, ProtoState};
use crate::sizebuf::{Reader, Underflow};

/// FTE chunked downloads always transfer this many bytes. The final chunk is zero-padded on the
/// wire and truncated to the advertised file size by the receiver.
pub const DOWNLOAD_CHUNK_SIZE: usize = 1024;

/// `svc_*` opcodes (ezQuake `qwprot/src/protocol.h`). Only the ones a QuakeWorld server sends;
/// the NetQuake-legacy numbers in the same range are deliberately absent, and land as
/// [`ParseError::UnknownSvc`] if a server ever sends one.
pub mod op {
    pub const BAD: u8 = 0;
    pub const NOP: u8 = 1;
    pub const DISCONNECT: u8 = 2;
    pub const UPDATESTAT: u8 = 3;
    pub const SOUND: u8 = 6;
    pub const PRINT: u8 = 8;
    pub const STUFFTEXT: u8 = 9;
    pub const SETANGLE: u8 = 10;
    pub const SERVERDATA: u8 = 11;
    pub const LIGHTSTYLE: u8 = 12;
    pub const UPDATEFRAGS: u8 = 14;
    pub const STOPSOUND: u8 = 16;
    pub const DAMAGE: u8 = 19;
    pub const SPAWNSTATIC: u8 = 20;
    pub const FTE_SPAWNSTATIC2: u8 = 21;
    pub const SPAWNBASELINE: u8 = 22;
    pub const TEMP_ENTITY: u8 = 23;
    pub const SETPAUSE: u8 = 24;
    pub const CENTERPRINT: u8 = 26;
    pub const KILLEDMONSTER: u8 = 27;
    pub const FOUNDSECRET: u8 = 28;
    pub const SPAWNSTATICSOUND: u8 = 29;
    pub const INTERMISSION: u8 = 30;
    pub const FINALE: u8 = 31;
    pub const CDTRACK: u8 = 32;
    pub const SELLSCREEN: u8 = 33;
    pub const SMALLKICK: u8 = 34;
    pub const BIGKICK: u8 = 35;
    pub const UPDATEPING: u8 = 36;
    pub const UPDATEENTERTIME: u8 = 37;
    pub const UPDATESTATLONG: u8 = 38;
    pub const MUZZLEFLASH: u8 = 39;
    pub const UPDATEUSERINFO: u8 = 40;
    pub const DOWNLOAD: u8 = 41;
    pub const PLAYERINFO: u8 = 42;
    pub const NAILS: u8 = 43;
    pub const CHOKECOUNT: u8 = 44;
    pub const MODELLIST: u8 = 45;
    pub const SOUNDLIST: u8 = 46;
    pub const PACKETENTITIES: u8 = 47;
    pub const DELTAPACKETENTITIES: u8 = 48;
    pub const MAXSPEED: u8 = 49;
    pub const ENTGRAVITY: u8 = 50;
    pub const SETINFO: u8 = 51;
    pub const SERVERINFO: u8 = 52;
    pub const UPDATEPL: u8 = 53;
    pub const NAILS2: u8 = 54;
    pub const FTE_MODELLISTSHORT: u8 = 60;
    pub const FTE_SPAWNBASELINE2: u8 = 66;
    pub const QIZMOVOICE: u8 = 83;
    pub const FTE_VOICECHAT: u8 = 84;
}

/// `svc_playerinfo` flags (`PF_*`).
pub mod pf {
    pub const MSEC: u32 = 1 << 0;
    pub const COMMAND: u32 = 1 << 1;
    pub const VELOCITY1: u32 = 1 << 2;
    pub const MODEL: u32 = 1 << 5;
    pub const SKINNUM: u32 = 1 << 6;
    pub const EFFECTS: u32 = 1 << 7;
    pub const WEAPONFRAME: u32 = 1 << 8;
    pub const DEAD: u32 = 1 << 9;
    pub const GIB: u32 = 1 << 10;
    /// Player move code occupies bits 11..13.
    pub const PMC_SHIFT: u32 = 11;
    pub const PMC_MASK: u32 = 7;
    /// With `FTE_PEXT_TRANS`: a third flags byte follows.
    pub const EXTRA_PFS: u32 = 1 << 15;
    /// With `FTE_PEXT_TRANS`: an alpha byte follows.
    pub const TRANS_Z: u32 = 1 << 17;
    /// Post-remap position (see [`PlayerInfo`]).
    pub const ONGROUND: u32 = 1 << 22;
    /// Post-remap position (see [`PlayerInfo`]).
    pub const SOLID: u32 = 1 << 23;
}

/// Entity delta bits (`U_*`), as they appear in the leading 16-bit word.
mod u {
    pub const ORIGIN1: u32 = 1 << 9;
    pub const ORIGIN2: u32 = 1 << 10;
    pub const ORIGIN3: u32 = 1 << 11;
    pub const ANGLE2: u32 = 1 << 12;
    pub const FRAME: u32 = 1 << 13;
    pub const REMOVE: u32 = 1 << 14;
    pub const MOREBITS: u32 = 1 << 15;

    // Read from the extra byte when MOREBITS is set.
    pub const ANGLE1: u32 = 1 << 0;
    pub const ANGLE3: u32 = 1 << 1;
    pub const MODEL: u32 = 1 << 2;
    pub const COLORMAP: u32 = 1 << 3;
    pub const SKIN: u32 = 1 << 4;
    pub const EFFECTS: u32 = 1 << 5;
    pub const SOLID: u32 = 1 << 6;
    pub const FTE_EVENMORE: u32 = 1 << 7;

    // Read from the FTE extension byte(s) when FTE_EVENMORE is set.
    pub const FTE_TRANS: u32 = 1 << 1;
    pub const FTE_MODELDBL: u32 = 1 << 3;
    pub const FTE_ENTITYDBL: u32 = 1 << 5;
    pub const FTE_ENTITYDBL2: u32 = 1 << 6;
    pub const FTE_YETMORE: u32 = 1 << 7;
    pub const FTE_COLOURMOD: u32 = 1 << 10;
}

/// `svc_sound` channel-word flags.
mod snd {
    pub const VOLUME: u16 = 1 << 15;
    pub const ATTENUATION: u16 = 1 << 14;
}

/// Usercmd delta bits (`CM_*`). Shared with [`clc`](crate::clc), which writes them.
pub mod cm {
    pub const ANGLE1: u8 = 1 << 0;
    pub const ANGLE3: u8 = 1 << 1;
    pub const FORWARD: u8 = 1 << 2;
    pub const SIDE: u8 = 1 << 3;
    pub const UP: u8 = 1 << 4;
    pub const BUTTONS: u8 = 1 << 5;
    pub const IMPULSE: u8 = 1 << 6;
    pub const ANGLE2: u8 = 1 << 7;
}

/// The default sound volume when `svc_sound` doesn't carry one.
const DEFAULT_SOUND_VOLUME: u8 = 255;
/// The default sound attenuation when `svc_sound` doesn't carry one.
const DEFAULT_SOUND_ATTENUATION: f32 = 1.0;

/// Why a message stream couldn't be parsed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// An opcode we don't implement. Fatal by design — see the module docs.
    UnknownSvc {
        /// The opcode byte.
        svc: u8,
        /// Where it appeared.
        offset: usize,
    },
    /// A message ran off the end of the packet.
    Underflow(Underflow),
    /// `svc_bad` — the server itself signalled a broken stream.
    Bad {
        /// Where it appeared.
        offset: usize,
    },
}

impl From<Underflow> for ParseError {
    fn from(u: Underflow) -> Self {
        ParseError::Underflow(u)
    }
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::UnknownSvc { svc, offset } => {
                write!(f, "unknown svc {svc} ({svc:#04x}) at offset {offset}")
            }
            ParseError::Underflow(u) => write!(f, "{u}"),
            ParseError::Bad { offset } => write!(f, "svc_bad at offset {offset}"),
        }
    }
}

impl std::error::Error for ParseError {}

/// A hexdump of a packet, for the log line that accompanies a [`ParseError`]. Without the bytes,
/// a desync report is unactionable.
pub fn hexdump(data: &[u8]) -> String {
    let mut out = String::new();
    for (i, chunk) in data.chunks(16).enumerate() {
        out.push_str(&format!("{:04x}  ", i * 16));
        for b in chunk {
            out.push_str(&format!("{b:02x} "));
        }
        out.push('\n');
    }
    out
}

/// A usercmd as it appears inside `svc_playerinfo` — what a player's client last told the server
/// to do. Notably it carries their **view angles**, which is how a bot can know where an opponent
/// is looking.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Usercmd {
    /// Milliseconds this command covers.
    pub msec: u8,
    /// View angles in degrees: pitch, yaw, roll.
    pub angles: Vec3,
    /// Forward move, in units/sec.
    pub forward: i16,
    /// Strafe move, in units/sec.
    pub side: i16,
    /// Vertical move (swim/jump in water), in units/sec.
    pub up: i16,
    /// Button bits: attack 1, jump 2, use 4.
    pub buttons: u8,
    /// Impulse (weapon selection etc.), 0 for none.
    pub impulse: u8,
}

/// Read a delta-encoded usercmd against `from`. Protocol 28 only (27+ semantics: angles are all
/// optional and msec is always sent).
pub fn read_delta_usercmd(r: &mut Reader, from: &Usercmd) -> Result<Usercmd, Underflow> {
    let mut m = *from;
    let bits = r.u8()?;

    if bits & cm::ANGLE1 != 0 {
        m.angles.x = r.angle16()?;
    }
    if bits & cm::ANGLE2 != 0 {
        m.angles.y = r.angle16()?;
    }
    if bits & cm::ANGLE3 != 0 {
        m.angles.z = r.angle16()?;
    }
    if bits & cm::FORWARD != 0 {
        m.forward = r.i16()?;
    }
    if bits & cm::SIDE != 0 {
        m.side = r.i16()?;
    }
    if bits & cm::UP != 0 {
        m.up = r.i16()?;
    }
    if bits & cm::BUTTONS != 0 {
        m.buttons = r.u8()?;
    }
    if bits & cm::IMPULSE != 0 {
        m.impulse = r.u8()?;
    }
    m.msec = r.u8()?; // always sent
    Ok(m)
}

/// How a player is allowed to move (`pm_type` / `PMC_*`), decoded from the playerinfo flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PmType {
    /// Ordinary ground movement.
    Normal,
    /// Dead — no longer blocks movement.
    Dead,
    /// Spectator that passes through walls, QuakeWorld-compatible.
    OldSpectator,
    /// Spectator that passes through walls.
    Spectator,
    /// Flying, but collides with walls.
    Fly,
    /// Frozen (intermission, countdown).
    None,
    /// The server owns the view angles.
    Lock,
}

/// One player's state, from `svc_playerinfo`. Sent every frame for every player in our PVS,
/// including ourselves.
#[derive(Clone, Debug, PartialEq)]
pub struct PlayerInfo {
    /// Player slot (0-based).
    pub player: u8,
    /// Raw `PF_*` flags, already normalised (see below).
    pub flags: u32,
    /// Position.
    pub origin: Vec3,
    /// Animation frame.
    pub frame: u8,
    /// Age of this state in milliseconds, when the server sent one.
    pub msec: Option<u8>,
    /// The player's last usercmd — including **their view angles**.
    pub command: Option<Usercmd>,
    /// Velocity. Components the server omitted read as zero, per id.
    pub velocity: Vec3,
    /// Model index, if overridden (else the default player model).
    pub modelindex: Option<u16>,
    /// Skin number.
    pub skinnum: Option<u8>,
    /// Effect bits (quad/pent glow).
    pub effects: Option<u8>,
    /// Weapon animation frame — only sent for our own player.
    pub weaponframe: Option<u8>,
    /// Alpha, under `FTE_PEXT_TRANS`.
    pub alpha: Option<u8>,
    /// Decoded move type, when `Z_EXT_PM_TYPE` is negotiated.
    pub pm_type: Option<PmType>,
    /// Whether the jump button was held — part of the `PMC_NORMAL_JUMP_HELD` code.
    pub jump_held: bool,
}

impl PlayerInfo {
    /// Whether the player is dead. Authoritative — unlike health, which we only estimate for
    /// other players, this bit is on the wire.
    pub fn dead(&self) -> bool {
        self.flags & pf::DEAD != 0
    }

    /// Whether the player was gibbed.
    pub fn gib(&self) -> bool {
        self.flags & pf::GIB != 0
    }

    /// Whether the player is standing on the ground.
    ///
    /// Only meaningful for players other than ourselves when `Z_EXT_PF_ONGROUND` is negotiated —
    /// which is why we advertise it: without it a bot cannot tell whether an enemy is airborne,
    /// and airborne enemies are the ones worth rocketing.
    pub fn on_ground(&self) -> bool {
        self.flags & pf::ONGROUND != 0
    }

    /// Whether the player is solid.
    pub fn solid(&self) -> bool {
        self.flags & pf::SOLID != 0
    }
}

/// One entity's changed fields, from a `svc_packetentities` word.
///
/// Every field is an `Option` meaning "the server sent a new value" — absent means "unchanged from
/// whatever you had". The parser doesn't hold the baseline, so it can't resolve that itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EntityDelta {
    /// Entity number, already widened by the `ENTITYDBL` bits.
    pub number: u16,
    /// The entity is gone.
    pub remove: bool,
    /// Model index.
    pub model: Option<u16>,
    /// Animation frame. 16-bit to carry NetQuake 666's `U_FRAME2` high byte; QuakeWorld only ever
    /// fills the low 8.
    pub frame: Option<u16>,
    /// Colormap.
    pub colormap: Option<u8>,
    /// Skin.
    pub skin: Option<u8>,
    /// Effects bits.
    pub effects: Option<u8>,
    /// Per-axis origin.
    pub origin: [Option<f32>; 3],
    /// Per-axis angles, in degrees.
    pub angles: [Option<f32>; 3],
    /// The entity should be solid for prediction. Carries no payload — the bit *is* the value.
    pub solid: bool,
    /// Alpha, under `FTE_PEXT_TRANS`.
    pub trans: Option<u8>,
    /// Colour modulation, under `FTE_PEXT_COLOURMOD`.
    pub colourmod: Option<[u8; 3]>,
}

/// A `svc_packetentities` or `svc_deltapacketentities` message.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PacketEntities {
    /// For a delta update, the sequence this is relative to; `None` for a full update.
    ///
    /// The caller must check this against the delta_sequence it recorded for that frame: if they
    /// disagree, the server delta'd against a frame we don't have and the whole update is garbage.
    pub delta_from: Option<u8>,
    /// The entity deltas, in ascending entity-number order (the server guarantees this, and the
    /// merge algorithm depends on it).
    pub updates: Vec<EntityDelta>,
}

/// A flying nail, from `svc_nails` / `svc_nails2`.
///
/// Position is quantized to 2-unit steps in a ±4096 box, and angles to 22.5°/1.4° — cheap enough
/// to send dozens per frame, which is the point. There's no velocity: a bot has to difference
/// successive frames to get one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Nail {
    /// Server-assigned id, under `svc_nails2`. Without it, nails are anonymous and must be
    /// re-associated frame to frame by proximity.
    pub number: Option<u8>,
    /// Position.
    pub origin: Vec3,
    /// Pitch and yaw, in degrees; roll is not sent.
    pub pitch: f32,
    /// Yaw, in degrees.
    pub yaw: f32,
}

/// An entity baseline (`svc_spawnbaseline` / `svc_spawnstatic`) in its classic form.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Baseline {
    /// Model index.
    pub modelindex: u16,
    /// Animation frame. 16-bit for NetQuake 666's `B_LARGEFRAME` baselines; QuakeWorld fills the
    /// low 8 only.
    pub frame: u16,
    /// Colormap.
    pub colormap: u8,
    /// Skin.
    pub skinnum: u8,
    /// Position.
    pub origin: Vec3,
    /// Orientation.
    pub angles: Vec3,
}

/// The kinds of `svc_temp_entity`. Values are id's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TempEntityKind {
    Spike = 0,
    SuperSpike = 1,
    Gunshot = 2,
    Explosion = 3,
    TarExplosion = 4,
    Lightning1 = 5,
    Lightning2 = 6,
    WizSpike = 7,
    KnightSpike = 8,
    Lightning3 = 9,
    LavaSplash = 10,
    Teleport = 11,
    Blood = 12,
    LightningBlood = 13,
    /// NetQuake `TE_EXPLOSION2` (wire byte 12): a colour-mapped explosion. QuakeWorld puts `Blood`
    /// at 12, so this takes a distinct tag — the wire→kind mapping is each parser's job.
    Explosion2 = 14,
    /// NetQuake `TE_BEAM` (wire byte 13): the grapple beam.
    GrappleBeam = 15,
}

/// A temp entity — the one-shot visual effects. A bot reads them as evidence: an explosion is a
/// rocket that landed, a lightning beam is someone firing a shaft right now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TempEntity {
    /// A point effect (spikes, explosions, teleport, lava splash).
    Point {
        /// Which effect.
        kind: TempEntityKind,
        /// Where.
        origin: Vec3,
    },
    /// A blood or gunshot puff, with a particle count that scales with the damage dealt.
    Puff {
        /// Which effect.
        kind: TempEntityKind,
        /// Particle count.
        count: u8,
        /// Where.
        origin: Vec3,
    },
    /// A lightning beam — the lightning gun, firing, from `entity` along `start`→`end`.
    Beam {
        /// Which beam.
        kind: TempEntityKind,
        /// The entity holding the gun.
        entity: u16,
        /// Beam start.
        start: Vec3,
        /// Beam end.
        end: Vec3,
    },
}

/// `svc_serverdata` — the map is changing; everything before this is stale.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ServerData {
    /// The server's FTE extension echo.
    pub fte: u32,
    /// The server's FTE2 extension echo.
    pub fte2: u32,
    /// The server's MVDSV extension echo.
    pub mvd1: u32,
    /// Spawn count — echoed back in `prespawn`/`spawn`/`begin` so the server can tell a stale
    /// signon from a current one.
    pub servercount: i32,
    /// Game directory (`qw`, `ktx`, …).
    pub gamedir: String,
    /// Our player slot.
    pub playernum: u8,
    /// Whether we were admitted as a spectator.
    pub spectator: bool,
    /// The map's display name (not its filename).
    pub levelname: String,
    /// Physics constants — see [`MoveVars`].
    pub movevars: MoveVars,
}

/// The server's physics constants, from `svc_serverdata`.
///
/// These are what the bot's movement model must run on: they're per-server, not per-map, and a
/// server running non-default gravity or accel makes every jump arc we precomputed wrong.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoveVars {
    pub gravity: f32,
    pub stopspeed: f32,
    pub maxspeed: f32,
    pub spectatormaxspeed: f32,
    pub accelerate: f32,
    pub airaccelerate: f32,
    pub wateraccelerate: f32,
    pub friction: f32,
    pub waterfriction: f32,
    pub entgravity: f32,
}

impl Default for MoveVars {
    /// Stock QuakeWorld values, in case we ever read them before a serverdata.
    fn default() -> Self {
        MoveVars {
            gravity: 800.0,
            stopspeed: 100.0,
            maxspeed: 320.0,
            spectatormaxspeed: 500.0,
            accelerate: 10.0,
            airaccelerate: 0.7,
            wateraccelerate: 10.0,
            friction: 4.0,
            waterfriction: 4.0,
            entgravity: 1.0,
        }
    }
}

/// `svc_serverinfo` — NetQuake's map-change message, the counterpart to QuakeWorld's [`ServerData`].
///
/// Unlike QuakeWorld, the precache lists ride *inside* this one message rather than in a separate
/// `modellist`/`soundlist` exchange, so there is no signon round trip to fetch them. Both lists are
/// 1-indexed on the wire (index 0 is a reserved empty slot); [`models`](Self::models) and
/// [`sounds`](Self::sounds) keep that convention with an empty string at index 0, so a wire index
/// dereferences them directly.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NqServerData {
    /// Protocol version: 15, 666 or 999.
    pub protocol: u32,
    /// `protocolflags` (999 only), for the coord/angle widths.
    pub flags: u32,
    /// Maximum player slots — the scoreboard size and the player-entity range `1..=maxclients`.
    pub maxclients: u8,
    /// Game type: 0 = coop, 1 = deathmatch (`GAME_*`).
    pub gametype: u8,
    /// The map's display name (not its filename — that's `models[1]`).
    pub levelname: String,
    /// The model precache list, 1-indexed (index 0 empty). `models[1]` is `maps/<name>.bsp`.
    pub models: Vec<String>,
    /// The sound precache list, 1-indexed (index 0 empty).
    pub sounds: Vec<String>,
}

/// `svc_clientdata` — the whole of our own player state in one bitfielded message, sent every frame.
///
/// QuakeWorld dribbles this out as individual `svc_updatestat`s plus playerinfo; NetQuake packs it
/// into one message. Fields absent from the bitfield default to zero (or, for viewheight, 22).
/// `active_weapon` is the `IT_*` weapon bit under standard Quake rules (id1), matching what the
/// `STAT_ACTIVEWEAPON` consumer expects.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClientData {
    /// Eye height above the origin (`STAT_VIEWHEIGHT`, default 22).
    pub viewheight: i16,
    /// Server's suggested pitch for auto-aim assist (`STAT_IDEALPITCH`).
    pub ideal_pitch: i8,
    /// View punch from recent damage, per axis in degrees.
    pub punch: Vec3,
    /// Our velocity, per axis (`char * 16`).
    pub velocity: Vec3,
    /// Carried items and powerups (`IT_*` / `STAT_ITEMS`).
    pub items: u32,
    /// Standing on the ground this frame.
    pub on_ground: bool,
    /// Standing in water this frame.
    pub in_water: bool,
    /// Weapon-model animation frame.
    pub weaponframe: u16,
    /// Armour points.
    pub armor: u16,
    /// The viewmodel's model index (`STAT_WEAPON`).
    pub weapon_model: u16,
    /// Health; non-positive means dead.
    pub health: i16,
    /// Ammo for the current weapon (`STAT_AMMO`).
    pub ammo: u16,
    /// Shells.
    pub shells: u16,
    /// Nails.
    pub nails: u16,
    /// Rockets.
    pub rockets: u16,
    /// Cells.
    pub cells: u16,
    /// The active weapon's `IT_*` bit (`STAT_ACTIVEWEAPON`).
    pub active_weapon: u8,
}

/// A soundlist or modellist chunk. The list is sent in batches because it won't fit one packet.
#[derive(Clone, Debug, PartialEq)]
pub struct ResourceList {
    /// Index of the first name in this chunk.
    pub start: u16,
    /// The names.
    pub names: Vec<String>,
    /// The next index to ask for, or 0 when the list is complete.
    pub next: u8,
}

/// One `svc_download` message. Its wire shape changes completely when
/// `FTE_PEXT_CHUNKEDDOWNLOADS` is negotiated.
#[derive(Clone, Debug, PartialEq)]
pub enum DownloadMessage {
    /// A sequential QuakeWorld block. `percent == 100` completes the file.
    LegacyBlock { percent: u8, data: Vec<u8> },
    /// A sequential download failed. FTEQW uses `-1` for a missing or denied file.
    LegacyError(i16),
    /// Metadata beginning a random-access transfer. Negative results are FTE's `DLERR_*` values.
    ChunkedStart { name: String, size: Result<u64, i32> },
    /// One fixed-size random-access block. This form can also arrive out of band.
    ChunkedBlock {
        chunk: u32,
        data: Box<[u8; DOWNLOAD_CHUNK_SIZE]>,
    },
}

/// One decoded server message.
#[derive(Clone, Debug, PartialEq)]
pub enum SvcEvent {
    /// Keepalive.
    Nop,
    /// The server is dropping us.
    Disconnect,
    /// Console text. `level` is `PRINT_LOW`/`MEDIUM`/`HIGH`/`CHAT` — obituaries arrive here.
    Print { level: u8, text: String },
    /// Text for the centre of the screen (KTX countdowns, "FIGHT").
    CenterPrint(String),
    /// A console command the server wants us to run. See the session's stufftext contract.
    StuffText(String),
    /// We took damage. `from` is where it came from — the only "someone shot me" signal that
    /// works when the shooter is out of sight.
    Damage {
        /// Damage absorbed by armour.
        armor: u8,
        /// Damage taken as health.
        blood: u8,
        /// Where the hit came from.
        from: Vec3,
    },
    /// New map, new everything.
    ServerData(Box<ServerData>),
    /// The server is setting our view angles (a teleport, or a respawn).
    SetAngle {
        /// Under `MVD_PEXT1_HIGHLAGTELEPORT`: 1 = teleport, 2 = respawn.
        kind: Option<u8>,
        /// The angles to adopt.
        angles: Vec3,
    },
    /// A light style's animation string.
    LightStyle {
        /// Style index.
        index: u8,
        /// Brightness pattern (`"a"`–`"z"`).
        pattern: String,
    },
    /// A sound started. Heard through walls (the server sends by PHS, not PVS), which makes this
    /// a bot's ears.
    Sound {
        /// The entity making it.
        entity: u16,
        /// Channel 0–7.
        channel: u8,
        /// Index into the soundlist. 16-bit for NetQuake 666's `SND_LARGESOUND`; QuakeWorld fits 8.
        sound: u16,
        /// Volume 0–255.
        volume: u8,
        /// Attenuation — how fast it fades with distance.
        attenuation: f32,
        /// Where it came from.
        origin: Vec3,
    },
    /// A looping sound stopped.
    StopSound {
        /// The entity.
        entity: u16,
        /// Channel 0–7.
        channel: u8,
    },
    /// A player's frag count changed.
    UpdateFrags { player: u8, frags: i16 },
    /// A player's ping changed.
    UpdatePing { player: u8, ping: i16 },
    /// A player's packet loss changed.
    UpdatePl { player: u8, pl: u8 },
    /// How long ago a player joined.
    UpdateEnterTime { player: u8, secs: f32 },
    /// One of our own stats changed (health, ammo, items…). See `STAT_*`.
    UpdateStat { stat: u8, value: i32 },
    /// A player's userinfo (name, team, colours).
    UpdateUserinfo { player: u8, userid: i32, userinfo: String },
    /// One key of a player's userinfo changed.
    SetInfo { player: u8, key: String, value: String },
    /// One key of the serverinfo changed.
    ServerInfo { key: String, value: String },
    /// An entity's baseline — the state deltas are relative to before it's ever updated.
    SpawnBaseline { entity: u16, baseline: Baseline },
    /// A static entity (torches, scenery): drawn, never updated.
    SpawnStatic(Baseline),
    /// A baseline in entity-delta form, under `FTE_PEXT_SPAWNSTATIC2`.
    SpawnBaselineDelta { entity: u16, delta: EntityDelta },
    /// A static entity in delta form, under `FTE_PEXT_SPAWNSTATIC2`.
    SpawnStaticDelta(EntityDelta),
    /// A looping ambient sound.
    SpawnStaticSound {
        origin: Vec3,
        /// Index into the soundlist. 16-bit for NetQuake's `svc_spawnstaticsound2`.
        sound: u16,
        volume: u8,
        attenuation: u8,
    },
    /// A one-shot visual effect.
    TempEntity(TempEntity),
    /// An entity fired something — a muzzle flash.
    MuzzleFlash { entity: u16 },
    /// Small view kick (we were lightly hit).
    SmallKick,
    /// Large view kick.
    BigKick,
    /// The server withheld this many frames to stay within our rate.
    ChokeCount(u8),
    /// The level ended; here's the camera position.
    Intermission { origin: Vec3, angles: Vec3 },
    /// End-of-episode text.
    Finale(String),
    /// CD track number.
    CdTrack(u8),
    /// Registered-version nag screen. Ignored.
    SellScreen,
    /// A monster died (single-player stat).
    KilledMonster,
    /// A secret was found (single-player stat).
    FoundSecret,
    /// Our max speed changed — prediction input.
    MaxSpeed(f32),
    /// Our gravity changed — prediction input.
    EntGravity(f32),
    /// The game was paused or unpaused.
    SetPause(bool),
    /// A legacy or negotiated-FTE file download message.
    Download(DownloadMessage),
    /// A player's per-frame state.
    PlayerInfo(Box<PlayerInfo>),
    /// Entity updates for this frame.
    PacketEntities(PacketEntities),
    /// Flying nails.
    Nails(Vec<Nail>),
    /// A chunk of the model list.
    ModelList(ResourceList),
    /// A chunk of the sound list.
    SoundList(ResourceList),
    /// Voice chat, skipped.
    Voice,

    // ── NetQuake-only messages ──────────────────────────────────────────────────────────────────
    // QuakeWorld folds these into other messages (own state into stats, frame time into the netchan
    // sequence); NetQuake sends them explicitly, so they need their own events. The mirror consumes
    // them through the same match as the shared ones above.
    /// `svc_time` — the server's frame clock. Delimits an entity frame and is echoed in `clc_move`
    /// for the server's ping calculation.
    Time(f32),
    /// `svc_signonnum` — advance the signon handshake to this step.
    SignonNum(u8),
    /// `svc_serverinfo` — NetQuake's map change (the counterpart to [`ServerData`]).
    NqServerData(Box<NqServerData>),
    /// `svc_clientdata` — our own player state for this frame.
    ClientData(Box<ClientData>),
    /// `svc_updatename` — a player slot's name (NetQuake has no userinfo string).
    UpdateName { player: u8, name: String },
    /// `svc_updatecolors` — a player slot's colours, packed `(top << 4) | bottom`.
    UpdateColors { player: u8, colors: u8 },
    /// `svc_setview` — the entity we're viewing from; our own player number is this minus one.
    SetView(u16),
    /// A NetQuake fast entity update, delta'd from the entity's **baseline** (not the previous
    /// frame). The store resolves absent fields against the baseline.
    EntityUpdate(EntityDelta),
    /// `svc_particle` — a particle burst. Carried for completeness; the bot ignores it.
    Particle {
        origin: Vec3,
        dir: Vec3,
        count: u8,
        color: u8,
    },
}

/// Parse every message in one packet payload.
///
/// `proto` is updated in place when `svc_serverdata` renegotiates the extensions — and must be,
/// because the coord width it sets applies to the rest of *this* packet.
///
/// On error, the events parsed so far are discarded: a stream that desynced halfway is not
/// half-trustworthy, it's untrustworthy from the first misread byte.
pub fn parse(proto: &mut ProtoState, data: &[u8]) -> Result<Vec<SvcEvent>, ParseError> {
    let mut r = Reader::with_widths(data, proto);
    let mut out = Vec::new();
    while !r.at_end() {
        let offset = r.pos();
        let svc = r.u8()?;
        match parse_one(proto, &mut r, svc, offset)? {
            Some(ev) => out.push(ev),
            None => continue,
        }
    }
    Ok(out)
}

/// Parse one message, having already read its opcode.
fn parse_one(proto: &mut ProtoState, r: &mut Reader, svc: u8, offset: usize) -> Result<Option<SvcEvent>, ParseError> {
    let ev = match svc {
        op::BAD => return Err(ParseError::Bad { offset }),
        op::NOP => SvcEvent::Nop,
        op::DISCONNECT => SvcEvent::Disconnect,
        op::UPDATESTAT => SvcEvent::UpdateStat {
            stat: r.u8()?,
            value: r.u8()? as i32,
        },
        op::UPDATESTATLONG => SvcEvent::UpdateStat {
            stat: r.u8()?,
            value: r.i32()?,
        },
        op::SOUND => {
            let channel = r.u16()?;
            let volume = if channel & snd::VOLUME != 0 {
                r.u8()?
            } else {
                DEFAULT_SOUND_VOLUME
            };
            let attenuation = if channel & snd::ATTENUATION != 0 {
                r.u8()? as f32 / 64.0
            } else {
                DEFAULT_SOUND_ATTENUATION
            };
            SvcEvent::Sound {
                sound: r.u8()? as u16,
                origin: r.coord3()?,
                entity: (channel >> 3) & 1023,
                channel: (channel & 7) as u8,
                volume,
                attenuation,
            }
        }
        op::STOPSOUND => {
            let w = r.u16()?;
            SvcEvent::StopSound {
                entity: w >> 3,
                channel: (w & 7) as u8,
            }
        }
        op::PRINT => SvcEvent::Print {
            level: r.u8()?,
            text: r.string()?,
        },
        op::STUFFTEXT => SvcEvent::StuffText(r.string()?),
        op::SETANGLE => {
            // Under HIGHLAGTELEPORT the server says *why*, so a client can fix up the moves
            // already in flight instead of walking the wrong way for a round-trip.
            let kind = if proto.has_mvd1(mvd1::HIGHLAGTELEPORT) {
                Some(r.u8()?)
            } else {
                None
            };
            SvcEvent::SetAngle {
                kind,
                angles: r.angle3()?,
            }
        }
        op::SERVERDATA => return parse_serverdata(proto, r).map(Some),
        op::LIGHTSTYLE => SvcEvent::LightStyle {
            index: r.u8()?,
            pattern: r.string()?,
        },
        op::UPDATEFRAGS => SvcEvent::UpdateFrags {
            player: r.u8()?,
            frags: r.i16()?,
        },
        op::DAMAGE => SvcEvent::Damage {
            armor: r.u8()?,
            blood: r.u8()?,
            from: r.coord3()?,
        },
        op::SPAWNSTATIC => SvcEvent::SpawnStatic(read_baseline(r)?),
        op::FTE_SPAWNSTATIC2 => SvcEvent::SpawnStaticDelta(read_delta_entity(proto, r)?),
        op::SPAWNBASELINE => SvcEvent::SpawnBaseline {
            entity: r.u16()?,
            baseline: read_baseline(r)?,
        },
        op::FTE_SPAWNBASELINE2 => {
            let delta = read_delta_entity(proto, r)?;
            SvcEvent::SpawnBaselineDelta {
                entity: delta.number,
                delta,
            }
        }
        op::TEMP_ENTITY => SvcEvent::TempEntity(read_temp_entity(r)?),
        op::SETPAUSE => SvcEvent::SetPause(r.u8()? != 0),
        op::CENTERPRINT => SvcEvent::CenterPrint(r.string()?),
        op::KILLEDMONSTER => SvcEvent::KilledMonster,
        op::FOUNDSECRET => SvcEvent::FoundSecret,
        op::SPAWNSTATICSOUND => SvcEvent::SpawnStaticSound {
            origin: r.coord3()?,
            sound: r.u8()? as u16,
            volume: r.u8()?,
            attenuation: r.u8()?,
        },
        op::INTERMISSION => SvcEvent::Intermission {
            origin: r.coord3()?,
            angles: r.angle3()?,
        },
        op::FINALE => SvcEvent::Finale(r.string()?),
        op::CDTRACK => SvcEvent::CdTrack(r.u8()?),
        op::SELLSCREEN => SvcEvent::SellScreen,
        op::SMALLKICK => SvcEvent::SmallKick,
        op::BIGKICK => SvcEvent::BigKick,
        op::UPDATEPING => SvcEvent::UpdatePing {
            player: r.u8()?,
            ping: r.i16()?,
        },
        op::UPDATEENTERTIME => SvcEvent::UpdateEnterTime {
            player: r.u8()?,
            secs: r.f32()?,
        },
        op::MUZZLEFLASH => SvcEvent::MuzzleFlash { entity: r.u16()? },
        op::UPDATEUSERINFO => SvcEvent::UpdateUserinfo {
            player: r.u8()?,
            userid: r.i32()?,
            userinfo: r.string()?,
        },
        op::DOWNLOAD => {
            if proto.has_fte(fte::CHUNKEDDOWNLOADS) {
                let chunk = r.i32()?;
                if chunk == -1 {
                    let flag = r.i32()?;
                    let size = if flag == i32::MIN {
                        let low = r.u32()? as u64;
                        let high = r.u32()? as u64;
                        Ok(low | high << 32)
                    } else if flag < 0 {
                        Err(flag)
                    } else {
                        Ok(flag as u64)
                    };
                    SvcEvent::Download(DownloadMessage::ChunkedStart {
                        name: r.string()?,
                        size,
                    })
                } else {
                    let data: [u8; DOWNLOAD_CHUNK_SIZE] = r.bytes(DOWNLOAD_CHUNK_SIZE)?.try_into().unwrap();
                    SvcEvent::Download(DownloadMessage::ChunkedBlock {
                        chunk: chunk as u32,
                        data: Box::new(data),
                    })
                }
            } else {
                let size = r.i16()?;
                let percent = r.u8()?;
                if size < 0 {
                    SvcEvent::Download(DownloadMessage::LegacyError(size))
                } else {
                    let data = r.bytes(size as usize)?.to_vec();
                    SvcEvent::Download(DownloadMessage::LegacyBlock { percent, data })
                }
            }
        }
        op::PLAYERINFO => SvcEvent::PlayerInfo(Box::new(read_playerinfo(proto, r)?)),
        op::NAILS => SvcEvent::Nails(read_nails(r, false)?),
        op::NAILS2 => SvcEvent::Nails(read_nails(r, true)?),
        op::CHOKECOUNT => SvcEvent::ChokeCount(r.u8()?),
        op::MODELLIST => SvcEvent::ModelList(read_resource_list(r, false)?),
        op::FTE_MODELLISTSHORT => SvcEvent::ModelList(read_resource_list(r, true)?),
        op::SOUNDLIST => SvcEvent::SoundList(read_resource_list(r, false)?),
        op::PACKETENTITIES => SvcEvent::PacketEntities(read_packet_entities(proto, r, false)?),
        op::DELTAPACKETENTITIES => SvcEvent::PacketEntities(read_packet_entities(proto, r, true)?),
        op::MAXSPEED => SvcEvent::MaxSpeed(r.f32()?),
        op::ENTGRAVITY => SvcEvent::EntGravity(r.f32()?),
        op::SETINFO => SvcEvent::SetInfo {
            player: r.u8()?,
            key: r.string()?,
            value: r.string()?,
        },
        op::SERVERINFO => SvcEvent::ServerInfo {
            key: r.string()?,
            value: r.string()?,
        },
        op::UPDATEPL => SvcEvent::UpdatePl {
            player: r.u8()?,
            pl: r.u8()?,
        },
        // We negotiate neither voice extension, so a server should never send these. If one does,
        // we can't skip it (no length prefix) — so treat it as the desync it is.
        op::QIZMOVOICE | op::FTE_VOICECHAT => return Err(ParseError::UnknownSvc { svc, offset }),
        _ => return Err(ParseError::UnknownSvc { svc, offset }),
    };
    Ok(Some(ev))
}

/// `svc_serverdata`: a leading run of `(magic, mask)` extension pairs, then the base protocol
/// version, then the map's parameters.
fn parse_serverdata(proto: &mut ProtoState, r: &mut Reader) -> Result<SvcEvent, ParseError> {
    use crate::protocol::magic;

    let (mut fte, mut fte2, mut mvd1) = (0, 0, 0);
    loop {
        let ver = r.u32()?;
        match ver {
            magic::FTE => fte = r.u32()?,
            magic::FTE2 => fte2 = r.u32()?,
            magic::MVD1 => mvd1 = r.u32()?,
            v if v == crate::protocol::VERSION => break,
            // An unknown tag here is unrecoverable: we can't tell whether a mask follows it.
            _ => {
                return Err(ParseError::UnknownSvc {
                    svc: op::SERVERDATA,
                    offset: r.pos(),
                })
            }
        }
    }

    // Adopt the negotiated set *now*: the coord width changes with it, and the very next field in
    // this same message may be read at the new width.
    proto.apply(fte, fte2, mvd1);
    r.coord_bytes = proto.coord_bytes;
    r.angle_bytes = proto.angle_bytes;

    let servercount = r.i32()?;
    let gamedir = r.string()?;
    let pnum = r.u8()?;
    let levelname = r.string()?;
    let movevars = MoveVars {
        gravity: r.f32()?,
        stopspeed: r.f32()?,
        maxspeed: r.f32()?,
        spectatormaxspeed: r.f32()?,
        accelerate: r.f32()?,
        airaccelerate: r.f32()?,
        wateraccelerate: r.f32()?,
        friction: r.f32()?,
        waterfriction: r.f32()?,
        entgravity: r.f32()?,
    };

    Ok(SvcEvent::ServerData(Box::new(ServerData {
        fte,
        fte2,
        mvd1,
        servercount,
        gamedir,
        // The high bit means "you're a spectator" — the slot is in the low seven.
        playernum: pnum & 0x7f,
        spectator: pnum & 0x80 != 0,
        levelname,
        movevars,
    })))
}

/// The classic baseline body, shared by `svc_spawnbaseline` and `svc_spawnstatic`.
fn read_baseline(r: &mut Reader) -> Result<Baseline, Underflow> {
    let modelindex = r.u8()? as u16;
    let frame = r.u8()? as u16;
    let colormap = r.u8()?;
    let skinnum = r.u8()?;
    // Origin and angle interleave, one axis at a time.
    let mut origin = Vec3::ZERO;
    let mut angles = Vec3::ZERO;
    for i in 0..3 {
        origin[i] = r.coord()?;
        angles[i] = r.angle()?;
    }
    Ok(Baseline {
        modelindex,
        frame,
        colormap,
        skinnum,
        origin,
        angles,
    })
}

/// `svc_temp_entity`. The payload shape depends entirely on the kind, and there's no length —
/// mis-sizing one desyncs the packet.
fn read_temp_entity(r: &mut Reader) -> Result<TempEntity, ParseError> {
    use TempEntityKind::*;
    let raw = r.u8()?;
    let kind = match raw {
        0 => Spike,
        1 => SuperSpike,
        2 => Gunshot,
        3 => Explosion,
        4 => TarExplosion,
        5 => Lightning1,
        6 => Lightning2,
        7 => WizSpike,
        8 => KnightSpike,
        9 => Lightning3,
        10 => LavaSplash,
        11 => Teleport,
        12 => Blood,
        13 => LightningBlood,
        _ => {
            return Err(ParseError::UnknownSvc {
                svc: op::TEMP_ENTITY,
                offset: r.pos() - 1,
            })
        }
    };
    Ok(match kind {
        Lightning1 | Lightning2 | Lightning3 => TempEntity::Beam {
            kind,
            entity: r.u16()?,
            start: r.coord3()?,
            end: r.coord3()?,
        },
        Gunshot | Blood => TempEntity::Puff {
            kind,
            count: r.u8()?,
            origin: r.coord3()?,
        },
        _ => TempEntity::Point {
            kind,
            origin: r.coord3()?,
        },
    })
}

/// `svc_nails` / `svc_nails2` — nails packed six bytes each: 12 bits per axis (2-unit steps,
/// biased by 4096), 4 bits of pitch, 8 of yaw.
///
/// These bytes are **immune to coord-width negotiation** — the packing is fixed regardless of
/// `FLOATCOORDS`, which makes them the one place the width must *not* be consulted.
fn read_nails(r: &mut Reader, indexed: bool) -> Result<Vec<Nail>, Underflow> {
    let count = r.u8()?;
    let mut out = Vec::with_capacity(count as usize);
    for _ in 0..count {
        let number = if indexed { Some(r.u8()?) } else { None };
        let b = r.bytes(6)?;
        let (b0, b1, b2, b3, b4, b5) = (
            b[0] as i32,
            b[1] as i32,
            b[2] as i32,
            b[3] as i32,
            b[4] as i32,
            b[5] as i32,
        );
        out.push(Nail {
            number,
            origin: Vec3::new(
                (((b0 + ((b1 & 15) << 8)) << 1) - 4096) as f32,
                ((((b1 >> 4) + (b2 << 4)) << 1) - 4096) as f32,
                (((b3 + ((b4 & 15) << 8)) << 1) - 4096) as f32,
            ),
            pitch: (360 * (b4 >> 4) / 16) as f32,
            yaw: (360 * b5 / 256) as f32,
        });
    }
    Ok(out)
}

/// A soundlist/modellist chunk. `short_start` is the `FTE_PEXT_MODELDBL` form, whose index is a
/// short because there can be more than 255 models.
fn read_resource_list(r: &mut Reader, short_start: bool) -> Result<ResourceList, Underflow> {
    let start = if short_start { r.u16()? } else { r.u8()? as u16 };
    let mut names = Vec::new();
    loop {
        let s = r.string()?;
        if s.is_empty() {
            break;
        }
        names.push(s);
    }
    Ok(ResourceList {
        start,
        names,
        next: r.u8()?,
    })
}

/// `svc_playerinfo`.
fn read_playerinfo(proto: &ProtoState, r: &mut Reader) -> Result<PlayerInfo, Underflow> {
    let player = r.u8()?;
    let mut flags = r.u16()? as u32;

    // The flags field is 16 bits on the wire but 24 in meaning. With TRANS the server may send a
    // third byte; without it, ONGROUND/SOLID sit at bits 14/15 and have to be shifted up to where
    // the rest of the code expects them. Getting this backwards silently reports every player as
    // airborne.
    if proto.has_fte(fte::TRANS) {
        if flags & pf::EXTRA_PFS != 0 {
            flags |= (r.u8()? as u32) << 16;
        }
    } else {
        flags = (flags & 0x3fff) | (flags & 0xc000) << 8;
    }

    let origin = if proto.has_mvd1(mvd1::FLOATCOORDS) {
        Vec3::new(r.float_coord()?, r.float_coord()?, r.float_coord()?)
    } else {
        r.coord3()?
    };
    let frame = r.u8()?;
    let msec = if flags & pf::MSEC != 0 { Some(r.u8()?) } else { None };
    let command = if flags & pf::COMMAND != 0 {
        Some(read_delta_usercmd(r, &Usercmd::default())?)
    } else {
        None
    };

    // Omitted velocity components read as zero, not "unchanged" — id is explicit about this.
    let mut velocity = Vec3::ZERO;
    for i in 0..3 {
        if flags & (pf::VELOCITY1 << i) != 0 {
            velocity[i] = r.i16()? as f32;
        }
    }

    let mut modelindex = if flags & pf::MODEL != 0 {
        Some(r.u8()? as u16)
    } else {
        None
    };
    let skinnum = if flags & pf::SKINNUM != 0 {
        let mut skin = r.u8()?;
        // The skin's top bit is stolen as a 9th model bit — but only when a model was sent.
        if skin & (1 << 7) != 0 && flags & pf::MODEL != 0 {
            modelindex = modelindex.map(|m| m + 256);
            skin -= 1 << 7;
        }
        Some(skin)
    } else {
        None
    };
    let effects = if flags & pf::EFFECTS != 0 { Some(r.u8()?) } else { None };
    let weaponframe = if flags & pf::WEAPONFRAME != 0 {
        Some(r.u8()?)
    } else {
        None
    };
    let alpha = if flags & pf::TRANS_Z != 0 && proto.has_fte(fte::TRANS) {
        Some(r.u8()?)
    } else {
        None
    };

    // pm_type is encoded in bits 11..13, but only meaningful if the server agreed to Z_EXT_PM_TYPE.
    let mut jump_held = false;
    let pm_type = if proto.has_z_ext(z_ext::PM_TYPE) {
        let code = (flags >> pf::PMC_SHIFT) & pf::PMC_MASK;
        Some(match code {
            0 | 1 => {
                if flags & pf::DEAD != 0 {
                    PmType::Dead
                } else {
                    jump_held = code == 1;
                    PmType::Normal
                }
            }
            2 => PmType::OldSpectator,
            3 if proto.has_z_ext(z_ext::PM_TYPE_NEW) => PmType::Spectator,
            4 if proto.has_z_ext(z_ext::PM_TYPE_NEW) => PmType::Fly,
            5 if proto.has_z_ext(z_ext::PM_TYPE_NEW) => PmType::None,
            6 if proto.has_z_ext(z_ext::PM_TYPE_NEW) => PmType::Lock,
            // A code from a future extension: fall back to what the flags tell us.
            _ if flags & pf::DEAD != 0 => PmType::Dead,
            _ => PmType::Normal,
        })
    } else {
        None
    };

    Ok(PlayerInfo {
        player,
        flags,
        origin,
        frame,
        msec,
        command,
        velocity,
        modelindex,
        skinnum,
        effects,
        weaponframe,
        alpha,
        pm_type,
        jump_held,
    })
}

/// One entity delta, given its already-read leading word.
fn read_delta_entity_bits(proto: &ProtoState, r: &mut Reader, word: u32) -> Result<EntityDelta, Underflow> {
    let mut bits = word;
    let mut d = EntityDelta {
        number: (bits & 511) as u16,
        remove: bits & u::REMOVE != 0,
        ..Default::default()
    };
    bits &= !511;

    if bits & u::MOREBITS != 0 {
        bits |= r.u8()? as u32;
    }
    let mut morebits = 0u32;
    if bits & u::FTE_EVENMORE != 0 && proto.fte != 0 {
        morebits = r.u8()? as u32;
        if morebits & u::FTE_YETMORE != 0 {
            morebits |= (r.u8()? as u32) << 8;
        }
    }

    if bits & u::MODEL != 0 {
        d.model = Some(r.u8()? as u16 + if morebits & u::FTE_MODELDBL != 0 { 256 } else { 0 });
    } else if morebits & u::FTE_MODELDBL != 0 {
        // MODELDBL without U_MODEL means the index didn't fit a byte at all: it's a full short.
        d.model = Some(r.u16()?);
    }
    if bits & u::FRAME != 0 {
        d.frame = Some(r.u8()? as u16);
    }
    if bits & u::COLORMAP != 0 {
        d.colormap = Some(r.u8()?);
    }
    if bits & u::SKIN != 0 {
        d.skin = Some(r.u8()?);
    }
    if bits & u::EFFECTS != 0 {
        d.effects = Some(r.u8()?);
    }

    // Origins and angles interleave per axis, and each origin honours the MVDSV float form.
    let float_origin = proto.has_mvd1(mvd1::FLOATCOORDS);
    let coord = |r: &mut Reader| if float_origin { r.float_coord() } else { r.coord() };
    if bits & u::ORIGIN1 != 0 {
        d.origin[0] = Some(coord(r)?);
    }
    if bits & u::ANGLE1 != 0 {
        d.angles[0] = Some(r.angle()?);
    }
    if bits & u::ORIGIN2 != 0 {
        d.origin[1] = Some(coord(r)?);
    }
    if bits & u::ANGLE2 != 0 {
        d.angles[1] = Some(r.angle()?);
    }
    if bits & u::ORIGIN3 != 0 {
        d.origin[2] = Some(coord(r)?);
    }
    if bits & u::ANGLE3 != 0 {
        d.angles[2] = Some(r.angle()?);
    }
    d.solid = bits & u::SOLID != 0;

    if morebits & u::FTE_TRANS != 0 && proto.has_fte(fte::TRANS) {
        d.trans = Some(r.u8()?);
    }
    if morebits & u::FTE_COLOURMOD != 0 && proto.has_fte(fte::COLOURMOD) {
        d.colourmod = Some([r.u8()?, r.u8()?, r.u8()?]);
    }
    // The entity-number extension bits live at the *end*, which is why the packetentities loop has
    // to peek ahead to learn the real number before it can merge (see `read_packet_entities`).
    if morebits & u::FTE_ENTITYDBL != 0 {
        d.number += 512;
    }
    if morebits & u::FTE_ENTITYDBL2 != 0 {
        d.number += 1024;
    }
    Ok(d)
}

/// An entity delta that carries its own leading word (the `svc_fte_spawnstatic2` form).
fn read_delta_entity(proto: &ProtoState, r: &mut Reader) -> Result<EntityDelta, Underflow> {
    let word = r.u16()? as u32;
    read_delta_entity_bits(proto, r, word)
}

/// `svc_packetentities` / `svc_deltapacketentities`.
fn read_packet_entities(proto: &ProtoState, r: &mut Reader, delta: bool) -> Result<PacketEntities, Underflow> {
    let delta_from = if delta { Some(r.u8()?) } else { None };
    let mut updates = Vec::new();
    loop {
        let word = r.u16()? as u32;
        if word == 0 {
            break;
        }
        updates.push(read_delta_entity_bits(proto, r, word)?);
    }
    Ok(PacketEntities { delta_from, updates })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sizebuf::Writer;

    /// Write a vanilla (1-byte) angle. Only tests need this: a real client writes angles solely
    /// inside usercmds, which are always 16-bit, so it isn't part of [`Writer`]'s surface.
    trait WriteVanillaAngle {
        fn i8_as_angle(&mut self, degrees: f32);
    }

    impl WriteVanillaAngle for Writer {
        fn i8_as_angle(&mut self, degrees: f32) {
            self.u8(((degrees * (256.0 / 360.0)).round() as i32 & 255) as u8);
        }
    }

    fn vanilla() -> ProtoState {
        ProtoState::new()
    }

    fn with(fte_bits: u32, mvd1_bits: u32, z: u32) -> ProtoState {
        let mut p = ProtoState::new();
        p.apply(fte_bits, 0, mvd1_bits);
        p.z_ext = z;
        p
    }

    /// An opcode we don't implement means we've already lost our place in the byte stream — the
    /// only honest response is to fail loudly, with enough context to debug it.
    #[test]
    fn unknown_svc_is_fatal_and_located() {
        let mut p = vanilla();
        let err = parse(&mut p, &[op::NOP, 200]).unwrap_err();
        assert_eq!(err, ParseError::UnknownSvc { svc: 200, offset: 1 });
        assert!(err.to_string().contains("200"));

        // The NetQuake-legacy opcodes a QuakeWorld server never sends are in this category too.
        assert!(matches!(
            parse(&mut p, &[7]).unwrap_err(),
            ParseError::UnknownSvc { svc: 7, .. }
        ));
        // As is svc_bad, which the server sends when *it* knows the stream is broken.
        assert!(matches!(parse(&mut p, &[op::BAD]), Err(ParseError::Bad { offset: 0 })));
    }

    /// A truncated packet is reported, not panicked on, and names where it ran out.
    #[test]
    fn truncated_message_underflows_cleanly() {
        let mut p = vanilla();
        let err = parse(&mut p, &[op::UPDATEFRAGS, 3]).unwrap_err();
        assert!(matches!(err, ParseError::Underflow(_)), "{err:?}");
    }

    /// Many messages ride in one packet; all of them must come out, in order.
    #[test]
    fn parses_a_multi_message_packet() {
        let mut w = Writer::new();
        w.u8(op::NOP);
        w.u8(op::UPDATEFRAGS);
        w.u8(2);
        w.i16(15);
        w.u8(op::PRINT);
        w.u8(3);
        w.string("hello");
        w.u8(op::CHOKECOUNT);
        w.u8(4);

        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::Nop,
                SvcEvent::UpdateFrags { player: 2, frags: 15 },
                SvcEvent::Print {
                    level: 3,
                    text: "hello".into()
                },
                SvcEvent::ChokeCount(4),
            ]
        );
    }

    /// `svc_serverdata` renegotiates the coord width **mid-packet**: everything after it in the
    /// same datagram must be read at the new width. This is the single most consequential ordering
    /// rule in the parser — get it wrong and the first map you join desyncs.
    #[test]
    fn serverdata_widens_coords_for_the_rest_of_the_packet() {
        use crate::protocol::magic;
        let mut w = Writer::new();
        w.u8(op::SERVERDATA);
        w.u32(magic::FTE);
        w.u32(fte::FLOATCOORDS);
        w.u32(crate::protocol::VERSION);
        w.i32(1234); // servercount
        w.string("ktx");
        w.u8(0x80 | 3); // playernum 3, spectator bit
        w.string("The Bad Place");
        for v in [800.0f32, 100.0, 320.0, 500.0, 10.0, 0.7, 10.0, 4.0, 4.0, 1.0] {
            w.i32(v.to_bits() as i32);
        }
        // A float-coord message riding in the same packet as the serverdata that enabled it.
        w.u8(op::INTERMISSION);
        for v in [1.5f32, 2.5, 3.5] {
            w.i32(v.to_bits() as i32);
        }
        w.i16(0);
        w.i16(0);
        w.i16(0);

        let mut p = vanilla();
        let evs = parse(&mut p, &w.into_vec()).unwrap();

        let SvcEvent::ServerData(sd) = &evs[0] else {
            panic!("expected serverdata: {evs:?}")
        };
        assert_eq!(sd.servercount, 1234);
        assert_eq!(sd.gamedir, "ktx");
        assert_eq!(sd.playernum, 3);
        assert!(sd.spectator, "the high bit of playernum means spectator");
        assert_eq!(sd.levelname, "The Bad Place");
        assert_eq!(sd.movevars.gravity, 800.0);
        assert_eq!(sd.movevars.airaccelerate, 0.7);

        // The state was adopted, and the intermission coords were read as floats.
        assert_eq!((p.coord_bytes, p.angle_bytes), (4, 2));
        assert_eq!(
            evs[1],
            SvcEvent::Intermission {
                origin: Vec3::new(1.5, 2.5, 3.5),
                angles: Vec3::ZERO
            }
        );
    }

    /// A vanilla server sends no extension pairs at all — just the version — and the widths stay
    /// narrow.
    #[test]
    fn serverdata_without_extensions() {
        let mut w = Writer::new();
        w.u8(op::SERVERDATA);
        w.u32(crate::protocol::VERSION);
        w.i32(7);
        w.string("qw");
        w.u8(0);
        w.string("dm4");
        for _ in 0..10 {
            w.i32(0);
        }

        let mut p = ProtoState::new();
        p.apply(fte::FLOATCOORDS, 0, 0); // stale state from a previous map
        let evs = parse(&mut p, &w.into_vec()).unwrap();

        let SvcEvent::ServerData(sd) = &evs[0] else { panic!() };
        assert!(!sd.spectator);
        assert_eq!(sd.playernum, 0);
        assert_eq!((p.coord_bytes, p.angle_bytes), (2, 1), "must reset, not inherit");
    }

    /// `svc_sound` packs the entity and channel into one word, and makes volume/attenuation
    /// optional with sensible defaults.
    #[test]
    fn parses_sound_with_and_without_optional_fields() {
        // Bare: no volume, no attenuation.
        let mut w = Writer::new();
        w.u8(op::SOUND);
        w.u16(37 << 3 | 2); // entity 37, channel 2
        w.u8(9);
        w.i16(8 * 8);
        w.i16(0);
        w.i16(0);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs[0],
            SvcEvent::Sound {
                entity: 37,
                channel: 2,
                sound: 9,
                volume: 255,
                attenuation: 1.0,
                origin: Vec3::new(8.0, 0.0, 0.0),
            }
        );

        // With both.
        let mut w = Writer::new();
        w.u8(op::SOUND);
        w.u16(snd::VOLUME | snd::ATTENUATION | (5 << 3) | 1);
        w.u8(128); // volume
        w.u8(64); // attenuation / 64 == 1.0
        w.u8(3);
        w.i16(0);
        w.i16(0);
        w.i16(0);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs[0],
            SvcEvent::Sound {
                entity: 5,
                channel: 1,
                sound: 3,
                volume: 128,
                attenuation: 1.0,
                origin: Vec3::ZERO,
            }
        );
    }

    /// Each temp-entity kind has its own payload size. A wrong one doesn't fail — it silently eats
    /// the next message — so every shape is pinned here.
    #[test]
    fn parses_every_temp_entity_shape() {
        // Point: coords only.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(TempEntityKind::Explosion as u8);
        w.i16(16 * 8);
        w.i16(0);
        w.i16(0);
        w.u8(op::NOP); // proves the payload was sized right
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::TempEntity(TempEntity::Point {
                    kind: TempEntityKind::Explosion,
                    origin: Vec3::new(16.0, 0.0, 0.0)
                }),
                SvcEvent::Nop
            ]
        );

        // Puff: a count byte, then coords.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(TempEntityKind::Blood as u8);
        w.u8(20);
        w.i16(0);
        w.i16(0);
        w.i16(0);
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::TempEntity(TempEntity::Puff {
                    kind: TempEntityKind::Blood,
                    count: 20,
                    origin: Vec3::ZERO
                }),
                SvcEvent::Nop
            ]
        );

        // Beam: an entity short, then two coord triples.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(TempEntityKind::Lightning2 as u8);
        w.u16(6);
        for _ in 0..6 {
            w.i16(0);
        }
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::TempEntity(TempEntity::Beam {
                    kind: TempEntityKind::Lightning2,
                    entity: 6,
                    start: Vec3::ZERO,
                    end: Vec3::ZERO
                }),
                SvcEvent::Nop
            ]
        );

        // An unknown kind can't be sized, so it's fatal like an unknown opcode.
        let evs = parse(&mut vanilla(), &[op::TEMP_ENTITY, 99]);
        assert!(matches!(evs, Err(ParseError::UnknownSvc { .. })));
    }

    /// Nails are bit-packed six bytes each and are **not** affected by coord-width negotiation —
    /// the one place where honouring FLOATCOORDS would be a bug.
    #[test]
    fn parses_nails_at_both_coord_widths() {
        let mut w = Writer::new();
        w.u8(op::NAILS);
        w.u8(1);
        // origin (2048, 2048, 2048) → each axis (2048+4096)/2 = 3072 = 0xC00
        w.bytes(&[0x00, 0x0c, 0xc0, 0x00, 0x0c, 0x80]);
        w.u8(op::NOP);
        let bytes = w.into_vec();

        for mut p in [vanilla(), with(fte::FLOATCOORDS, 0, 0)] {
            let evs = parse(&mut p, &bytes).unwrap();
            let SvcEvent::Nails(nails) = &evs[0] else {
                panic!("{evs:?}")
            };
            assert_eq!(nails.len(), 1);
            assert_eq!(nails[0].origin, Vec3::new(2048.0, 2048.0, 2048.0));
            assert_eq!(nails[0].number, None);
            assert_eq!(nails[0].yaw, 180.0);
            assert_eq!(evs[1], SvcEvent::Nop, "six bytes per nail regardless of width");
        }
    }

    /// `svc_nails2` prefixes each nail with a server-assigned id — the thing that makes tracking a
    /// nail across frames possible rather than a guess.
    #[test]
    fn parses_indexed_nails() {
        let mut w = Writer::new();
        w.u8(op::NAILS2);
        w.u8(2);
        w.u8(17);
        w.bytes(&[0; 6]);
        w.u8(18);
        w.bytes(&[0; 6]);
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::Nails(nails) = &evs[0] else { panic!() };
        assert_eq!(
            nails.iter().map(|n| n.number).collect::<Vec<_>>(),
            vec![Some(17), Some(18)]
        );
        assert_eq!(evs[1], SvcEvent::Nop);
    }

    /// Without `FTE_PEXT_TRANS` the ONGROUND/SOLID flags arrive at bits 14/15 and must be shifted
    /// to 22/23; with it, they're already there and a third flags byte may follow. Getting this
    /// backwards makes every enemy look permanently airborne.
    #[test]
    fn playerinfo_flag_remap_without_trans() {
        let mut w = Writer::new();
        w.u8(op::PLAYERINFO);
        w.u8(1);
        w.u16(1 << 14); // PF_ONGROUND in its pre-remap position
        w.i16(0);
        w.i16(0);
        w.i16(0);
        w.u8(0); // frame

        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else {
            panic!("{evs:?}")
        };
        assert!(pi.on_ground(), "bit 14 should have been remapped to bit 22");
        assert!(!pi.solid());
    }

    /// With TRANS, the flags are 24 bits and `PF_EXTRA_PFS` pulls in the third byte holding
    /// ONGROUND/SOLID.
    #[test]
    fn playerinfo_extra_flags_with_trans() {
        let mut w = Writer::new();
        w.u8(op::PLAYERINFO);
        w.u8(2);
        w.u16((pf::EXTRA_PFS | pf::DEAD) as u16);
        w.u8(((pf::ONGROUND | pf::SOLID) >> 16) as u8);
        w.i16(0);
        w.i16(0);
        w.i16(0);
        w.u8(0);

        let mut p = with(fte::TRANS, 0, 0);
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else {
            panic!("{evs:?}")
        };
        assert!(pi.on_ground());
        assert!(pi.solid());
        assert!(pi.dead(), "death is on the wire — never an estimate");
    }

    /// The full playerinfo field order, which is the easiest thing in the protocol to get subtly
    /// wrong: velocity comes *after* the usercmd, and the skin's top bit steals a model bit.
    #[test]
    fn playerinfo_field_order_and_skin_model_bit() {
        let flags =
            pf::MSEC | pf::COMMAND | pf::VELOCITY1 | (pf::VELOCITY1 << 2) | pf::MODEL | pf::SKINNUM | pf::EFFECTS;
        let mut w = Writer::new();
        w.u8(op::PLAYERINFO);
        w.u8(5);
        w.u16(flags as u16);
        w.i16(64 * 8); // origin x = 64
        w.i16(0);
        w.i16(0);
        w.u8(3); // frame
        w.u8(13); // msec
                  // usercmd: yaw + forward, then msec
        w.u8(cm::ANGLE2 | cm::FORWARD);
        w.angle16(90.0);
        w.i16(400);
        w.u8(13);
        w.i16(200); // velocity x (VELOCITY1)
        w.i16(-50); // velocity z (VELOCITY3)
        w.u8(40); // modelindex
        w.u8(1 << 7 | 5); // skin 5, plus the 9th model bit
        w.u8(8); // effects
        w.u8(op::NOP);

        let mut p = with(fte::TRANS, 0, 0);
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else {
            panic!("{evs:?}")
        };

        assert_eq!(pi.player, 5);
        assert_eq!(pi.origin, Vec3::new(64.0, 0.0, 0.0));
        assert_eq!(pi.frame, 3);
        assert_eq!(pi.msec, Some(13));
        let cmd = pi.command.expect("usercmd");
        assert!((cmd.angles.y - 90.0).abs() < 0.01, "an opponent's view yaw is knowable");
        assert_eq!(cmd.forward, 400);
        assert_eq!(cmd.msec, 13);
        assert_eq!(
            pi.velocity,
            Vec3::new(200.0, 0.0, -50.0),
            "omitted components read as zero"
        );
        assert_eq!(pi.modelindex, Some(40 + 256), "skin bit 7 carries the 9th model bit");
        assert_eq!(pi.skinnum, Some(5), "and is stripped from the skin");
        assert_eq!(pi.effects, Some(8));
        assert_eq!(evs[1], SvcEvent::Nop, "payload sized exactly");
    }

    /// pm_type only decodes when the server agreed to `Z_EXT_PM_TYPE`; the jump-held code is how
    /// we know whether a bunnyhopping opponent is holding jump.
    #[test]
    fn playerinfo_pm_type_needs_z_ext() {
        let build = |code: u32, extra: u32| {
            let mut w = Writer::new();
            w.u8(op::PLAYERINFO);
            w.u8(0);
            w.u16(((code << pf::PMC_SHIFT) | extra) as u16);
            w.i16(0);
            w.i16(0);
            w.i16(0);
            w.u8(0);
            w.into_vec()
        };

        // Without the extension, no claim is made.
        let evs = parse(&mut with(fte::TRANS, 0, 0), &build(1, 0)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert_eq!(pi.pm_type, None);

        // With it: jump-held is distinguishable from plain normal.
        let mut p = with(fte::TRANS, 0, z_ext::PM_TYPE | z_ext::PM_TYPE_NEW);
        let evs = parse(&mut p, &build(1, 0)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert_eq!(pi.pm_type, Some(PmType::Normal));
        assert!(pi.jump_held);

        let evs = parse(&mut p, &build(0, 0)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert!(!pi.jump_held);

        // Dead outranks the move code.
        let evs = parse(&mut p, &build(0, pf::DEAD)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert_eq!(pi.pm_type, Some(PmType::Dead));

        let evs = parse(&mut p, &build(4, 0)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert_eq!(pi.pm_type, Some(PmType::Fly));

        // PM_TYPE without PM_TYPE_NEW: the newer codes fall back rather than misreport.
        let mut p = with(fte::TRANS, 0, z_ext::PM_TYPE);
        let evs = parse(&mut p, &build(4, 0)).unwrap();
        let SvcEvent::PlayerInfo(pi) = &evs[0] else { panic!() };
        assert_eq!(pi.pm_type, Some(PmType::Normal));
    }

    /// A full update has no `from` byte; a delta update does, and the caller needs it to decide
    /// whether the delta is against a frame it still has.
    #[test]
    fn packetentities_full_vs_delta() {
        let mut w = Writer::new();
        w.u8(op::PACKETENTITIES);
        w.u16(u::ORIGIN1 as u16 | 12); // entity 12, origin x
        w.i16(32 * 8);
        w.u16(0); // end of list
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!("{evs:?}")
        };
        assert_eq!(pe.delta_from, None);
        assert_eq!(pe.updates.len(), 1);
        assert_eq!(pe.updates[0].number, 12);
        assert_eq!(pe.updates[0].origin[0], Some(32.0));
        assert_eq!(pe.updates[0].origin[1], None, "untouched axes stay unchanged");

        let mut w = Writer::new();
        w.u8(op::DELTAPACKETENTITIES);
        w.u8(42); // delta from sequence 42
        w.u16(u::REMOVE as u16 | 7);
        w.u16(0);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!()
        };
        assert_eq!(pe.delta_from, Some(42));
        assert_eq!(pe.updates[0].number, 7);
        assert!(pe.updates[0].remove);
    }

    /// The delta's field order — model, frame, colormap, skin, effects, then origin and angles
    /// *interleaved per axis*. The interleave is the trap: reading all three origins then all
    /// three angles parses byte-for-byte plausibly and produces garbage.
    #[test]
    fn entity_delta_field_order() {
        let mut w = Writer::new();
        w.u8(op::PACKETENTITIES);
        let bits = u::MOREBITS | u::FRAME | u::ORIGIN1 | u::ORIGIN2 | u::ORIGIN3 | u::ANGLE2;
        w.u16(bits as u16 | 3);
        w.u8((u::MODEL | u::SKIN | u::ANGLE1) as u8);
        w.u8(11); // model
        w.u8(2); // frame
        w.u8(4); // skin
        w.i16(8); // origin x = 1.0
        w.i8_as_angle(90.0); // angle x
        w.i16(2 * 8); // origin y
        w.i8_as_angle(45.0); // angle y
        w.i16(3 * 8); // origin z
        w.u16(0);

        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!("{evs:?}")
        };
        let d = &pe.updates[0];
        assert_eq!(d.number, 3);
        assert_eq!(d.model, Some(11));
        assert_eq!(d.frame, Some(2));
        assert_eq!(d.skin, Some(4));
        assert_eq!(d.colormap, None);
        assert_eq!(d.origin, [Some(1.0), Some(2.0), Some(3.0)]);
        assert_eq!(d.angles[0].map(|a| a.round()), Some(90.0));
        assert_eq!(d.angles[1].map(|a| a.round()), Some(45.0));
        assert_eq!(d.angles[2], None);
    }

    /// `FTE_PEXT_ENTITYDBL` widens entity numbers using bits stored *after* the number — and
    /// `MODELDBL` without `U_MODEL` means the model index is a short rather than a byte.
    #[test]
    fn entity_delta_fte_extensions() {
        let mut p = with(fte::ENTITYDBL | fte::ENTITYDBL2 | fte::MODELDBL | fte::TRANS, 0, 0);

        let mut w = Writer::new();
        w.u8(op::PACKETENTITIES);
        w.u16((u::MOREBITS | 5) as u16);
        w.u8(u::FTE_EVENMORE as u8);
        w.u8((u::FTE_ENTITYDBL | u::FTE_ENTITYDBL2 | u::FTE_MODELDBL | u::FTE_TRANS) as u8);
        w.u16(400); // model as a short, since U_MODEL is clear
        w.u8(128); // trans
        w.u16(0);

        let evs = parse(&mut p, &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!("{evs:?}")
        };
        let d = &pe.updates[0];
        assert_eq!(d.number, 5 + 512 + 1024, "both doubling bits apply");
        assert_eq!(d.model, Some(400));
        assert_eq!(d.trans, Some(128));

        // With U_MODEL set, MODELDBL instead means "+256 to the byte you just read".
        let mut w = Writer::new();
        w.u8(op::PACKETENTITIES);
        w.u16((u::MOREBITS | 1) as u16);
        w.u8((u::MODEL | u::FTE_EVENMORE) as u8);
        w.u8(u::FTE_MODELDBL as u8);
        w.u8(7);
        w.u16(0);
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!()
        };
        assert_eq!(pe.updates[0].model, Some(7 + 256));
    }

    /// `U_FTE_YETMORE` pulls in a second extension byte, where COLOURMOD lives.
    #[test]
    fn entity_delta_yetmore_byte() {
        let mut p = with(fte::COLOURMOD, 0, 0);
        let mut w = Writer::new();
        w.u8(op::PACKETENTITIES);
        w.u16((u::MOREBITS | 9) as u16);
        w.u8(u::FTE_EVENMORE as u8);
        w.u8(u::FTE_YETMORE as u8); // first extension byte: "another follows"
        w.u8((u::FTE_COLOURMOD >> 8) as u8); // second byte holds COLOURMOD
        w.u8(10);
        w.u8(20);
        w.u8(30);
        w.u16(0);

        let evs = parse(&mut p, &w.into_vec()).unwrap();
        let SvcEvent::PacketEntities(pe) = &evs[0] else {
            panic!("{evs:?}")
        };
        assert_eq!(pe.updates[0].colourmod, Some([10, 20, 30]));
    }

    /// Legacy download data is retained for the transfer layer and consumes exactly its length, or
    /// everything after it in the packet is garbage.
    #[test]
    fn parses_legacy_download_payload_exactly() {
        let mut w = Writer::new();
        w.u8(op::DOWNLOAD);
        w.i16(4);
        w.u8(50); // percent
        w.bytes(&[0xde, 0xad, 0xbe, 0xef]);
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::Download(DownloadMessage::LegacyBlock {
                    percent: 50,
                    data: vec![0xde, 0xad, 0xbe, 0xef],
                }),
                SvcEvent::Nop,
            ]
        );

        // -1 means "no such file" and carries no payload at all.
        let mut w = Writer::new();
        w.u8(op::DOWNLOAD);
        w.i16(-1);
        w.u8(0);
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![SvcEvent::Download(DownloadMessage::LegacyError(-1)), SvcEvent::Nop]
        );
    }

    /// FTE replaces the legacy short/percent body with either file metadata or a fixed-size random
    /// access chunk. Both the ordinary and 64-bit size forms are on the wire in deployed servers.
    #[test]
    fn parses_fte_chunked_download_messages() {
        let mut p = with(fte::CHUNKEDDOWNLOADS, 0, 0);
        let mut w = Writer::new();
        w.u8(op::DOWNLOAD);
        w.i32(-1);
        w.i32(4097);
        w.string("maps/test.bsp");
        w.u8(op::DOWNLOAD);
        w.i32(2);
        w.bytes(&[0x5a; DOWNLOAD_CHUNK_SIZE]);
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::Download(DownloadMessage::ChunkedStart {
                    name: "maps/test.bsp".into(),
                    size: Ok(4097),
                }),
                SvcEvent::Download(DownloadMessage::ChunkedBlock {
                    chunk: 2,
                    data: Box::new([0x5a; DOWNLOAD_CHUNK_SIZE]),
                }),
            ]
        );

        let mut w = Writer::new();
        w.u8(op::DOWNLOAD);
        w.i32(-1);
        w.i32(i32::MIN);
        w.u32(7);
        w.u32(1);
        w.string("maps/huge.bsp");
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![SvcEvent::Download(DownloadMessage::ChunkedStart {
                name: "maps/huge.bsp".into(),
                size: Ok((1u64 << 32) | 7),
            })]
        );

        let mut w = Writer::new();
        w.u8(op::DOWNLOAD);
        w.i32(-1);
        w.i32(-3);
        w.string("maps/missing.bsp");
        let evs = parse(&mut p, &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![SvcEvent::Download(DownloadMessage::ChunkedStart {
                name: "maps/missing.bsp".into(),
                size: Err(-3),
            })]
        );
    }

    /// Resource lists arrive in chunks with a continuation index; `next == 0` ends the list.
    #[test]
    fn parses_resource_lists() {
        let mut w = Writer::new();
        w.u8(op::SOUNDLIST);
        w.u8(0);
        w.string("weapons/rocket1i.wav");
        w.string("items/damage.wav");
        w.string(""); // list terminator
        w.u8(0); // no continuation
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs[0],
            SvcEvent::SoundList(ResourceList {
                start: 0,
                names: vec!["weapons/rocket1i.wav".into(), "items/damage.wav".into()],
                next: 0,
            })
        );

        // The MODELDBL form indexes with a short, because there can be more than 255 models.
        let mut w = Writer::new();
        w.u8(op::FTE_MODELLISTSHORT);
        w.u16(300);
        w.string("progs/player.mdl");
        w.string("");
        w.u8(44); // ask for more from here
        let evs = parse(&mut with(fte::MODELDBL, 0, 0), &w.into_vec()).unwrap();
        assert_eq!(
            evs[0],
            SvcEvent::ModelList(ResourceList {
                start: 300,
                names: vec!["progs/player.mdl".into()],
                next: 44,
            })
        );
    }

    /// `svc_setangle` grows a leading type byte under HIGHLAGTELEPORT — a one-byte difference that
    /// shifts every following message if mis-parsed.
    #[test]
    fn setangle_type_byte_follows_negotiation() {
        let mut w = Writer::new();
        w.u8(op::SETANGLE);
        w.i8_as_angle(0.0);
        w.i8_as_angle(90.0);
        w.i8_as_angle(0.0);
        w.u8(op::NOP);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert!(matches!(evs[0], SvcEvent::SetAngle { kind: None, .. }));
        assert_eq!(evs[1], SvcEvent::Nop);

        let mut w = Writer::new();
        w.u8(op::SETANGLE);
        w.u8(1); // teleport
        w.i8_as_angle(0.0);
        w.i8_as_angle(90.0);
        w.i8_as_angle(0.0);
        w.u8(op::NOP);
        let evs = parse(&mut with(0, mvd1::HIGHLAGTELEPORT, 0), &w.into_vec()).unwrap();
        assert!(matches!(evs[0], SvcEvent::SetAngle { kind: Some(1), .. }));
        assert_eq!(evs[1], SvcEvent::Nop);
    }

    /// Stats come in a byte form and a long form; both must land as the same event, since a stat
    /// doesn't care how it was encoded.
    #[test]
    fn parses_both_stat_forms() {
        let mut w = Writer::new();
        w.u8(op::UPDATESTAT);
        w.u8(0);
        w.u8(100);
        w.u8(op::UPDATESTATLONG);
        w.u8(17);
        w.i32(123456);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::UpdateStat { stat: 0, value: 100 },
                SvcEvent::UpdateStat {
                    stat: 17,
                    value: 123456
                },
            ]
        );
    }

    /// The messages a bot reads as evidence rather than decoration: who hurt us and from where,
    /// who fired, and what the server is telling us to do.
    #[test]
    fn parses_the_evidence_messages() {
        let mut w = Writer::new();
        w.u8(op::DAMAGE);
        w.u8(10); // armour
        w.u8(25); // health
        w.i16(100 * 8);
        w.i16(0);
        w.i16(0);
        w.u8(op::MUZZLEFLASH);
        w.u16(4);
        w.u8(op::STUFFTEXT);
        w.string("cmd spawn 5\n");
        w.u8(op::CENTERPRINT);
        w.string("FIGHT");
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        assert_eq!(
            evs,
            vec![
                SvcEvent::Damage {
                    armor: 10,
                    blood: 25,
                    from: Vec3::new(100.0, 0.0, 0.0)
                },
                SvcEvent::MuzzleFlash { entity: 4 },
                SvcEvent::StuffText("cmd spawn 5\n".into()),
                SvcEvent::CenterPrint("FIGHT".into()),
            ]
        );
    }

    /// Baselines in both forms: the classic body, and the entity-delta form FTE uses.
    #[test]
    fn parses_baselines() {
        let mut w = Writer::new();
        w.u8(op::SPAWNBASELINE);
        w.u16(23);
        w.u8(5); // modelindex
        w.u8(1); // frame
        w.u8(0); // colormap
        w.u8(2); // skin
        w.i16(8 * 8);
        w.i8_as_angle(0.0);
        w.i16(16 * 8);
        w.i8_as_angle(90.0);
        w.i16(24 * 8);
        w.i8_as_angle(0.0);
        let evs = parse(&mut vanilla(), &w.into_vec()).unwrap();
        let SvcEvent::SpawnBaseline { entity, baseline } = &evs[0] else {
            panic!("{evs:?}")
        };
        assert_eq!(*entity, 23);
        assert_eq!(baseline.modelindex, 5);
        assert_eq!(baseline.skinnum, 2);
        assert_eq!(baseline.origin, Vec3::new(8.0, 16.0, 24.0));
        assert_eq!(baseline.angles[1].round(), 90.0);

        // FTE's delta form carries the entity number in the delta word itself.
        let mut w = Writer::new();
        w.u8(op::FTE_SPAWNBASELINE2);
        w.u16(u::ORIGIN1 as u16 | 31);
        w.i16(64 * 8);
        let evs = parse(&mut with(fte::SPAWNSTATIC2, 0, 0), &w.into_vec()).unwrap();
        let SvcEvent::SpawnBaselineDelta { entity, delta } = &evs[0] else {
            panic!("{evs:?}")
        };
        assert_eq!(*entity, 31);
        assert_eq!(delta.origin[0], Some(64.0));
    }
}
