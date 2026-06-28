// SPDX-License-Identifier: AGPL-3.0-or-later

//! Constants ported from `qw-qc/defs.qc`. Only the subset needed so far is defined;
//! more arrives with the gameplay milestones.

#![allow(dead_code)]

use glam::Vec3;

/// Bitwise helpers for QuakeC's `f32`-encoded flag fields (`.items`, `.flags`, `.effects`,
/// `.spawnflags`, …). Lets call sites read `v.flags.has(Flags::ONGROUND)` and
/// `v.items = v.items.with(Items::SHELLS)` instead of the `as i32 & … as i32` dance, while
/// preserving the exact integer-truncation semantics QuakeC relies on. Accepts any of the
/// [`flag_bits!`] types (via their `impl Into<f32>`) as the `bits` argument.
pub trait Bits {
    /// Whether any of `bits` is set.
    fn has(self, bits: impl Into<f32>) -> bool;
    /// Whether *all* of `bits` are set (subset test, e.g. key-door checks).
    fn has_all(self, bits: impl Into<f32>) -> bool;
    /// This value with `bits` set.
    fn with(self, bits: impl Into<f32>) -> Self;
    /// This value with `bits` cleared.
    fn without(self, bits: impl Into<f32>) -> Self;
}

impl Bits for f32 {
    #[inline]
    fn has(self, bits: impl Into<f32>) -> bool {
        self as i32 & bits.into() as i32 != 0
    }
    #[inline]
    fn has_all(self, bits: impl Into<f32>) -> bool {
        let b = bits.into() as i32;
        self as i32 & b == b
    }
    #[inline]
    fn with(self, bits: impl Into<f32>) -> f32 {
        (self as i32 | bits.into() as i32) as f32
    }
    #[inline]
    fn without(self, bits: impl Into<f32>) -> f32 {
        (self as i32 & !(bits.into() as i32)) as f32
    }
}

/// Equality test for QuakeC's `f32`-encoded *enum* fields (`.weapon`, `.solid`, `.movetype`,
/// `.deadflag`, `.watertype`, …): `v.weapon.is(Items::AXE)` instead of the noisier
/// `v.weapon == Items::AXE.as_f32()`. The single-value sibling of [`Bits::has`]; accepts any
/// [`flag_bits!`] / [`float_enum!`] type via its `impl Into<f32>`.
pub trait FieldEq {
    /// Whether this field equals `value`.
    fn is(self, value: impl Into<f32>) -> bool;
}

impl FieldEq for f32 {
    #[inline]
    fn is(self, value: impl Into<f32>) -> bool {
        self == value.into()
    }
}

/// Declare a `bitflags` type for one of QuakeC's `f32`-encoded bit fields, with the trivial
/// `f32` bridge the engine-shared fields and the [`Bits`] helpers use. Bit values are written
/// `1 << N` for readability.
macro_rules! flag_bits {
    ($(#[$meta:meta])* $name:ident { $($flag:ident = $val:expr;)* }) => {
        bitflags::bitflags! {
            $(#[$meta])*
            #[derive(Clone, Copy, PartialEq, Eq, Debug)]
            pub struct $name: u32 {
                $(const $flag = $val;)*
            }
        }
        impl $name {
            /// This flag set as the `f32` the engine-shared field stores.
            #[inline]
            pub const fn as_f32(self) -> f32 {
                self.bits() as f32
            }
            /// Decode an engine-shared `f32` field back into flags.
            #[inline]
            pub fn from_f32(v: f32) -> Self {
                Self::from_bits_truncate(v as u32)
            }
        }
        impl From<$name> for f32 {
            #[inline]
            fn from(v: $name) -> f32 {
                v.as_f32()
            }
        }
    };
}

flag_bits! {
    /// QuakeC's `.items` bitfield (weapons, ammo, armor, keys, powerups). Single source of
    /// truth for the bit values; the engine-shared `items`/`weapon` fields store them as `f32`.
    Items {
        SHOTGUN          = 1 << 0;
        SUPER_SHOTGUN    = 1 << 1;
        NAILGUN          = 1 << 2;
        SUPER_NAILGUN    = 1 << 3;
        GRENADE_LAUNCHER = 1 << 4;
        ROCKET_LAUNCHER  = 1 << 5;
        LIGHTNING        = 1 << 6;
        SHELLS           = 1 << 8;
        NAILS            = 1 << 9;
        ROCKETS          = 1 << 10;
        CELLS            = 1 << 11;
        AXE              = 1 << 12;
        ARMOR1           = 1 << 13;
        ARMOR2           = 1 << 14;
        ARMOR3           = 1 << 15;
        SUPERHEALTH      = 1 << 16;
        KEY1             = 1 << 17;
        KEY2             = 1 << 18;
        INVISIBILITY     = 1 << 19;
        INVULNERABILITY  = 1 << 20;
        SUIT             = 1 << 21;
        QUAD             = 1 << 22;
    }
}

flag_bits! {
    /// QuakeC `.flags` (`FL_*`).
    Flags {
        FLY           = 1 << 0;
        SWIM          = 1 << 1;
        CLIENT        = 1 << 3;
        INWATER       = 1 << 4;
        MONSTER       = 1 << 5;
        GODMODE       = 1 << 6;
        NOTARGET      = 1 << 7;
        ITEM          = 1 << 8;
        ONGROUND      = 1 << 9;
        PARTIALGROUND = 1 << 10;
        WATERJUMP     = 1 << 11;
        JUMPRELEASED  = 1 << 12;
    }
}

flag_bits! {
    /// QuakeC `.effects` (`EF_*`).
    Effects {
        BRIGHTFIELD = 1 << 0;
        MUZZLEFLASH = 1 << 1;
        BRIGHTLIGHT = 1 << 2;
        DIMLIGHT    = 1 << 3;
        FLAG1       = 1 << 4;
        FLAG2       = 1 << 5;
        BLUE        = 1 << 6;
        RED         = 1 << 7;
    }
}

flag_bits! {
    /// `.spawnflags` skill/deathmatch filtering (engine `ED_LoadFromFile` equivalent).
    SpawnFilter {
        NOT_EASY       = 1 << 8;
        NOT_MEDIUM     = 1 << 9;
        NOT_HARD       = 1 << 10;
        NOT_DEATHMATCH = 1 << 11;
    }
}

flag_bits! {
    /// `func_door`/`func_door_secret` `.spawnflags`.
    DoorFlags {
        START_OPEN = 1 << 0;
        DONT_LINK  = 1 << 2;
        GOLD_KEY   = 1 << 3;
        SILVER_KEY = 1 << 4;
        TOGGLE     = 1 << 5;
    }
}

flag_bits! {
    /// `func_plat` `.spawnflags`.
    PlatFlags {
        LOW_TRIGGER = 1 << 0;
    }
}

flag_bits! {
    /// `trigger_teleport` `.spawnflags`.
    TeleportFlags {
        PLAYER_ONLY = 1 << 0;
        SILENT      = 1 << 1;
    }
}

flag_bits! {
    /// Shared trigger `.spawnflags` (`trigger_multiple` no-touch, `trigger_counter` no-message).
    TriggerFlags {
        NOTOUCH   = 1 << 0;
        NOMESSAGE = 1 << 0;
    }
}

flag_bits! {
    /// `item_health` `.spawnflags`.
    HealthFlags {
        ROTTEN = 1 << 0;
        MEGA   = 1 << 1;
    }
}

flag_bits! {
    /// Ammo box `.spawnflags` (big box).
    AmmoFlags {
        BIG = 1 << 0;
    }
}

flag_bits! {
    /// `light` `.spawnflags`.
    LightFlags {
        START_OFF = 1 << 0;
    }
}

flag_bits! {
    /// `trigger_push` `.spawnflags`.
    PushFlags {
        ONCE = 1 << 0;
    }
}

flag_bits! {
    /// `func_rotate_entity` `.spawnflags` (Hipnotic continual rotator): `TOGGLE` lets a trigger
    /// turn the spin on/off; `START_ON` spins from spawn.
    RotateEntityFlags {
        TOGGLE   = 1 << 0;
        START_ON = 1 << 1;
    }
}

flag_bits! {
    /// `path_rotate` `.spawnflags` — per-corner behaviour for `func_rotate_train`: `ROTATION`
    /// spins at the corner's `rotate` rate, `ANGLES` turns to its `angles` (clearing rotation),
    /// `STOP` waits for a retrigger, `NO_ROTATE` stops spinning while waiting, `DAMAGE` causes
    /// `dmg` along the segment, `MOVETIME` reads `speed` as travel time, `SET_DAMAGE` applies
    /// `dmg` to all of the train's targets.
    PathRotateFlags {
        ROTATION   = 1 << 0;
        ANGLES     = 1 << 1;
        STOP       = 1 << 2;
        NO_ROTATE  = 1 << 3;
        DAMAGE     = 1 << 4;
        MOVETIME   = 1 << 5;
        SET_DAMAGE = 1 << 6;
    }
}

flag_bits! {
    /// `func_movewall` `.spawnflags` — collision proxy for a rotating object: `VISIBLE` draws the
    /// brush (otherwise it's an invisible clip), `TOUCH` damages the player on contact,
    /// `NONBLOCKING` makes it non-solid.
    MovewallFlags {
        VISIBLE     = 1 << 0;
        TOUCH       = 1 << 1;
        NONBLOCKING = 1 << 2;
    }
}

flag_bits! {
    /// `func_rotate_door` `.spawnflags`: `STAYOPEN` reopens after closing, so a trigger-once door
    /// can't jam shut when blocked.
    RotateDoorFlags {
        STAYOPEN = 1 << 0;
    }
}

flag_bits! {
    /// `func_bob` `.spawnflags`. `MG_NONSOLID` (Dimension of the Machine) and `NONSOLID`
    /// (progsdump / Arcane Dimensions) both mean a non-solid bob, which isn't supported.
    /// `COLLISION` is the progsdump collision bit, kept to document the map-format bit.
    FuncBobFlags {
        MG_NONSOLID = 1 << 0;
        COLLISION   = 1 << 1;
        NONSOLID    = 1 << 2;
    }
}

// --- player bounding box & view offset ---
pub const VEC_HULL_MIN: Vec3 = Vec3::new(-16.0, -16.0, -24.0);
pub const VEC_HULL_MAX: Vec3 = Vec3::new(16.0, 16.0, 32.0);
pub const VEC_VIEW_OFS: Vec3 = Vec3::new(0.0, 0.0, 22.0);

/// Declare a `#[repr(i32)]` enum for one of QuakeC's discrete-valued `f32` entity fields
/// (`.solid`, `.movetype`, `.deadflag`, …), with the `as_f32`/`from_f32` bridge those
/// engine-shared fields need.
macro_rules! float_enum {
    ($(#[$meta:meta])* $name:ident { $($variant:ident = $val:expr,)* }) => {
        $(#[$meta])*
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[repr(i32)]
        pub enum $name {
            $($variant = $val,)*
        }
        impl $name {
            /// This value as the `f32` the engine-shared field stores.
            #[inline]
            pub const fn as_f32(self) -> f32 {
                self as i32 as f32
            }
            /// This value as an `i32` (for builtin arguments).
            #[allow(dead_code)]
            #[inline]
            pub const fn as_i32(self) -> i32 {
                self as i32
            }
            /// Decode an engine-shared `f32` field, or `None` if it matches no variant.
            #[allow(dead_code)]
            #[inline]
            pub fn from_f32(v: f32) -> Option<Self> {
                match v as i32 {
                    $(x if x == $name::$variant as i32 => Some($name::$variant),)*
                    _ => None,
                }
            }
        }
        impl From<$name> for f32 {
            #[inline]
            fn from(v: $name) -> f32 {
                v.as_f32()
            }
        }
    };
}

/// Declare a `#[repr(i32)]` enum for a network-protocol opcode set (`MSG_*`, `MULTICAST_*`,
/// `svc_*`, `TE_*`) — pure `i32` values written to the wire, with only the `as_i32` bridge
/// the `write_*`/`multicast` builtins need (no `f32` projection — these never touch entity fields).
macro_rules! int_enum {
    ($(#[$meta:meta])* $name:ident { $($variant:ident = $val:expr,)* }) => {
        $(#[$meta])*
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[repr(i32)]
        pub enum $name {
            $($variant = $val,)*
        }
        impl $name {
            /// This opcode's wire value.
            #[inline]
            pub const fn as_i32(self) -> i32 {
                self as i32
            }
        }
    };
}

float_enum! {
    /// `.solid` — how an entity interacts with others.
    Solid {
        Not = 0,      // no interaction
        Trigger = 1,  // touch on edge, not blocking
        BBox = 2,     // touch on edge, block
        SlideBox = 3, // touch on edge, not an onground
        Bsp = 4,      // bsp clip, touch on edge, block
    }
}

float_enum! {
    /// `.movetype` — how the engine moves an entity.
    MoveType {
        None = 0, // never moves
        AngleNoclip = 1,
        AngleClip = 2,
        Walk = 3, // players only
        Step = 4, // discrete, not real time unless falling
        Fly = 5,
        Toss = 6, // gravity
        Push = 7, // no clip to world, push and crush
        Noclip = 8,
        FlyMissile = 9,
        Bounce = 10,
    }
}

float_enum! {
    /// `pointcontents` / `.watertype` values.
    Content {
        Empty = -1,
        Solid = -2,
        Water = -3,
        Slime = -4,
        Lava = -5,
        Sky = -6,
    }
}

float_enum! {
    /// `.deadflag`.
    DeadFlag {
        No = 0,
        Dying = 1,
        Dead = 2,
        Respawnable = 3,
    }
}

float_enum! {
    /// `.takedamage`.
    TakeDamage {
        No = 0,
        Yes = 1,
        Aim = 2,
    }
}

float_enum! {
    /// `bprint`/`sprint` message level.
    PrintLevel {
        Low = 0,    // pickup messages
        Medium = 1, // death messages
        High = 2,   // critical messages
        Chat = 3,   // chat messages
    }
}

float_enum! {
    /// Sound attenuation.
    Attenuation {
        None = 0,
        Norm = 1,
        Idle = 2,
        Static = 3,
    }
}

int_enum! {
    /// Sound channel for `host.sound`. (The protocol's `CHAN_NO_PHS_ADD` bit-3 modifier is not
    /// a channel — it lives in [`host.sound_no_phs`](crate::host::HostApi::sound_no_phs).)
    Channel {
        Auto = 0,
        Weapon = 1,
        Voice = 2,
        Item = 3,
        Body = 4,
    }
}

int_enum! {
    /// `WriteByte`/`WriteCoord`/… destination — the `to` of every `host.write_*` call.
    MsgDest {
        One = 1,       // reliable, to `msg_entity`
        All = 2,       // reliable, to all clients
        Init = 3,      // the signon buffer
        Multicast = 4, // unreliable, to the multicast recipient set
    }
}

int_enum! {
    /// Recipient set for `host.multicast`.
    Multicast {
        All = 0,
        Phs = 1,
        Pvs = 2,
        AllR = 3,
        PhsR = 4,
        PvsR = 5,
    }
}

int_enum! {
    /// Server-to-client message opcode (`svc_*`), written via `host.write_svc`.
    Svc {
        UpdateFrags = 14,
        TempEntity = 23,
        SetPause = 24,
        CenterPrint = 26,
        KilledMonster = 27,
        FoundSecret = 28,
        Intermission = 30,
        Finale = 31,
        CdTrack = 32,
        SellScreen = 33,
        SmallKick = 34,
        BigKick = 35,
        MuzzleFlash = 39,
    }
}

int_enum! {
    /// Temp-entity effect, written via `host.write_te` (which emits the `svc_temp_entity`
    /// header itself, then this byte).
    Te {
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
    }
}

// --- collision epsilon ---
pub const STOP_EPSILON: f32 = 0.1;

// --- the standard QuakeC big-number "infinity" for traces ---
pub const WEAPON_BIG_RANGE: f32 = 100000.0;
