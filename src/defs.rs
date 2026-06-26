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

// --- player bounding box & view offset ---
pub const VEC_HULL_MIN: Vec3 = Vec3::new(-16.0, -16.0, -24.0);
pub const VEC_HULL_MAX: Vec3 = Vec3::new(16.0, 16.0, 32.0);
pub const VEC_VIEW_OFS: Vec3 = Vec3::new(0.0, 0.0, 22.0);

// --- edict.solid ---
pub const SOLID_NOT: f32 = 0.0; // no interaction with other objects
pub const SOLID_TRIGGER: f32 = 1.0; // touch on edge, but not blocking
pub const SOLID_BBOX: f32 = 2.0; // touch on edge, block
pub const SOLID_SLIDEBOX: f32 = 3.0; // touch on edge, but not an onground
pub const SOLID_BSP: f32 = 4.0; // bsp clip, touch on edge, block

// --- edict.movetype ---
pub const MOVETYPE_NONE: f32 = 0.0; // never moves
pub const MOVETYPE_ANGLENOCLIP: f32 = 1.0;
pub const MOVETYPE_ANGLECLIP: f32 = 2.0;
pub const MOVETYPE_WALK: f32 = 3.0; // players only
pub const MOVETYPE_STEP: f32 = 4.0; // discrete, not real time unless fall
pub const MOVETYPE_FLY: f32 = 5.0;
pub const MOVETYPE_TOSS: f32 = 6.0; // gravity
pub const MOVETYPE_PUSH: f32 = 7.0; // no clip to world, push and crush
pub const MOVETYPE_NOCLIP: f32 = 8.0;
pub const MOVETYPE_FLYMISSILE: f32 = 9.0; // extra size to monsters
pub const MOVETYPE_BOUNCE: f32 = 10.0;

// --- point content values (pointcontents / watertype) ---
pub const CONTENT_EMPTY: f32 = -1.0;
pub const CONTENT_SOLID: f32 = -2.0;
pub const CONTENT_WATER: f32 = -3.0;
pub const CONTENT_SLIME: f32 = -4.0;
pub const CONTENT_LAVA: f32 = -5.0;
pub const CONTENT_SKY: f32 = -6.0;

// --- deadflag ---
pub const DEAD_NO: f32 = 0.0;
pub const DEAD_DYING: f32 = 1.0;
pub const DEAD_DEAD: f32 = 2.0;
pub const DEAD_RESPAWNABLE: f32 = 3.0;

// --- takedamage ---
pub const DAMAGE_NO: f32 = 0.0;
pub const DAMAGE_YES: f32 = 1.0;
pub const DAMAGE_AIM: f32 = 2.0;

// --- print levels (bprint / sprint) ---
pub const PRINT_LOW: i32 = 0; // pickup messages
pub const PRINT_MEDIUM: i32 = 1; // death messages
pub const PRINT_HIGH: i32 = 2; // critical messages
pub const PRINT_CHAT: i32 = 3; // chat messages

// --- sound channels ---
/// Added to a channel to skip the PHS check (door movement sounds).
pub const CHAN_NO_PHS_ADD: i32 = 8;
pub const CHAN_AUTO: i32 = 0;
pub const CHAN_WEAPON: i32 = 1;
pub const CHAN_VOICE: i32 = 2;
pub const CHAN_ITEM: i32 = 3;
pub const CHAN_BODY: i32 = 4;

// --- sound attenuation ---
pub const ATTN_NONE: f32 = 0.0;
pub const ATTN_NORM: f32 = 1.0;
pub const ATTN_IDLE: f32 = 2.0;
pub const ATTN_STATIC: f32 = 3.0;

// --- network message destinations (multicast / WriteByte `to`) ---
pub const MSG_ONE: i32 = 1; // reliable to msg_entity
pub const MSG_ALL: i32 = 2; // reliable to all
pub const MSG_INIT: i32 = 3;
pub const MSG_MULTICAST: i32 = 4; // unreliable, to the multicast set

// --- multicast destinations (the `to` of host.multicast) ---
pub const MULTICAST_ALL: i32 = 0;
pub const MULTICAST_PHS: i32 = 1;
pub const MULTICAST_PVS: i32 = 2;
pub const MULTICAST_ALL_R: i32 = 3;
pub const MULTICAST_PHS_R: i32 = 4;
pub const MULTICAST_PVS_R: i32 = 5;

// --- server-to-client message types (used with WriteByte MSG_*) ---
pub const SVC_UPDATEFRAGS: i32 = 14;
pub const SVC_TEMPENTITY: i32 = 23;
pub const SVC_SETPAUSE: i32 = 24;
pub const SVC_CENTERPRINT: i32 = 26;
pub const SVC_KILLEDMONSTER: i32 = 27;
pub const SVC_FOUNDSECRET: i32 = 28;
pub const SVC_INTERMISSION: i32 = 30;
pub const SVC_FINALE: i32 = 31;
pub const SVC_CDTRACK: i32 = 32;
pub const SVC_SELLSCREEN: i32 = 33;
pub const SVC_SMALLKICK: i32 = 34;
pub const SVC_BIGKICK: i32 = 35;
pub const SVC_MUZZLEFLASH: i32 = 39;

// --- temp-entity effects (WriteByte TE_* after SVC_TEMPENTITY) ---
pub const TE_SPIKE: i32 = 0;
pub const TE_SUPERSPIKE: i32 = 1;
pub const TE_GUNSHOT: i32 = 2;
pub const TE_EXPLOSION: i32 = 3;
pub const TE_TAREXPLOSION: i32 = 4;
pub const TE_LIGHTNING1: i32 = 5;
pub const TE_LIGHTNING2: i32 = 6;
pub const TE_WIZSPIKE: i32 = 7;
pub const TE_KNIGHTSPIKE: i32 = 8;
pub const TE_LIGHTNING3: i32 = 9;
pub const TE_LAVASPLASH: i32 = 10;
pub const TE_TELEPORT: i32 = 11;
pub const TE_BLOOD: i32 = 12;
pub const TE_LIGHTNINGBLOOD: i32 = 13;

// --- collision epsilon ---
pub const STOP_EPSILON: f32 = 0.1;

// --- the standard QuakeC big-number "infinity" for traces ---
pub const WEAPON_BIG_RANGE: f32 = 100000.0;
