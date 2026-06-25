//! Constants ported from `qw-qc/defs.qc`. Only the subset needed so far is defined;
//! more arrives with the gameplay milestones.

#![allow(dead_code)]

use glam::Vec3;

// --- items / weapons bitflags (.items) ---
pub const IT_SHOTGUN: f32 = 1.0;
pub const IT_SUPER_SHOTGUN: f32 = 2.0;
pub const IT_NAILGUN: f32 = 4.0;
pub const IT_SUPER_NAILGUN: f32 = 8.0;
pub const IT_GRENADE_LAUNCHER: f32 = 16.0;
pub const IT_ROCKET_LAUNCHER: f32 = 32.0;
pub const IT_LIGHTNING: f32 = 64.0;
pub const IT_AXE: f32 = 4096.0;
pub const IT_SHELLS: f32 = 256.0;
pub const IT_NAILS: f32 = 512.0;
pub const IT_ROCKETS: f32 = 1024.0;
pub const IT_CELLS: f32 = 2048.0;
pub const IT_ARMOR1: f32 = 8192.0;
pub const IT_ARMOR2: f32 = 16384.0;
pub const IT_ARMOR3: f32 = 32768.0;
pub const IT_KEY1: f32 = 131072.0;
pub const IT_KEY2: f32 = 262144.0;
pub const IT_INVISIBILITY: f32 = 524288.0;
pub const IT_INVULNERABILITY: f32 = 1048576.0;
pub const IT_SUIT: f32 = 2097152.0;
pub const IT_QUAD: f32 = 4194304.0;

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

// --- edict.flags ---
pub const FL_FLY: f32 = 1.0;
pub const FL_SWIM: f32 = 2.0;
pub const FL_CLIENT: f32 = 8.0; // set for all client edicts
pub const FL_INWATER: f32 = 16.0;
pub const FL_MONSTER: f32 = 32.0;
pub const FL_GODMODE: f32 = 64.0;
pub const FL_NOTARGET: f32 = 128.0;
pub const FL_ITEM: f32 = 256.0; // extra wide size for bonus items
pub const FL_ONGROUND: f32 = 512.0; // standing on something
pub const FL_PARTIALGROUND: f32 = 1024.0;
pub const FL_WATERJUMP: f32 = 2048.0;
pub const FL_JUMPRELEASED: f32 = 4096.0;

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

// --- spawnflags filtering (engine ED_LoadFromFile equivalent) ---
pub const SPAWNFLAG_NOT_EASY: i32 = 256;
pub const SPAWNFLAG_NOT_MEDIUM: i32 = 512;
pub const SPAWNFLAG_NOT_HARD: i32 = 1024;
pub const SPAWNFLAG_NOT_DEATHMATCH: i32 = 2048;

// --- print levels (bprint / sprint) ---
pub const PRINT_LOW: i32 = 0; // pickup messages
pub const PRINT_MEDIUM: i32 = 1; // death messages
pub const PRINT_HIGH: i32 = 2; // critical messages
pub const PRINT_CHAT: i32 = 3; // chat messages

// --- sound channels ---
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
