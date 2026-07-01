// SPDX-License-Identifier: AGPL-3.0-or-later

//! Raw, `#[repr(C)]` structures shared with the host engine (mvdsv, `pr2` native
//! game-module API, `GAME_API_VERSION 16`).
//!
//! These layouts must match the engine's expectation *exactly* — the engine reads
//! and writes the [`EntVars`] prefix of every entity, and the [`GlobalVars`] block,
//! directly through the pointers we hand it in [`GameData`].
//!
//! Hard rule: every field of [`EntVars`] and [`GlobalVars`] is an all-bit-patterns-valid
//! POD type (`f32`, `i32`, or `glam::Vec3`). The engine may overwrite these bytes between
//! our calls, so no `bool`, `enum`, `NonZero`, or reference may appear here. Rust-side
//! state lives in the private tail of `Entity` (see `entity.rs`), which the engine never
//! addresses.
//!
//! `glam::Vec3` is `#[repr(C)]` and is exactly three `f32` (12 bytes), byte-compatible
//! with the C `vec3_t` (`float[3]`). It must be `Vec3`, never `Vec3A` (16-byte aligned).

use core::ffi::c_char;
use glam::Vec3;

use crate::defs::{MoveType, Solid, Weapon};

/// QuakeC string reference. With `sv_pr2references 1` these are engine-managed `int`
/// handles; the mod communicates real strings to the engine via trap calls (e.g.
/// `setmodel`'s `char*`), so for our purposes these are opaque placeholders.
pub type StringRef = i32;
/// QuakeC function reference (`int`). Unused by the native module — the engine drives
/// callbacks through `vmMain(GAME_EDICT_*)`, not these fields.
pub type FuncRef = i32;

/// The engine-shared entity fields — the `entvars_t` from `ktx/include/progdefs.h`.
/// Field order and types are load-bearing; do not reorder.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct EntVars {
    pub modelindex: f32,
    pub absmin: Vec3,
    pub absmax: Vec3,
    pub ltime: f32,
    pub lastruntime: f32,
    pub movetype: MoveType,
    pub solid: Solid,
    pub origin: Vec3,
    pub oldorigin: Vec3,
    pub velocity: Vec3,
    pub angles: Vec3,
    pub avelocity: Vec3,
    pub classname: StringRef,
    pub model: StringRef,
    pub frame: f32,
    pub skin: f32,
    pub effects: f32,
    pub mins: Vec3,
    pub maxs: Vec3,
    pub size: Vec3,
    pub touch: FuncRef,
    pub use_: FuncRef,
    pub think: FuncRef,
    pub blocked: FuncRef,
    pub nextthink: f32,
    pub groundentity: i32,
    pub health: f32,
    pub frags: f32,
    pub weapon: Weapon,
    pub weaponmodel: StringRef,
    pub weaponframe: f32,
    pub currentammo: f32,
    pub ammo_shells: f32,
    pub ammo_nails: f32,
    pub ammo_rockets: f32,
    pub ammo_cells: f32,
    pub items: f32,
    pub takedamage: f32,
    pub chain: i32,
    pub deadflag: f32,
    pub view_ofs: Vec3,
    pub button0: f32,
    pub button1: f32,
    pub button2: f32,
    pub impulse: f32,
    pub fixangle: f32,
    pub v_angle: Vec3,
    pub netname: StringRef,
    pub enemy: i32,
    pub flags: f32,
    pub colormap: f32,
    pub team: f32,
    pub max_health: f32,
    pub teleport_time: f32,
    pub armortype: f32,
    pub armorvalue: f32,
    pub waterlevel: f32,
    pub watertype: f32,
    pub ideal_yaw: f32,
    pub yaw_speed: f32,
    pub aiment: i32,
    pub goalentity: i32,
    pub spawnflags: f32,
    pub target: StringRef,
    pub targetname: StringRef,
    pub dmg_take: f32,
    pub dmg_save: f32,
    pub dmg_inflictor: i32,
    pub owner: i32,
    pub movedir: Vec3,
    pub message: StringRef,
    pub sounds: f32,
    pub noise: StringRef,
    pub noise1: StringRef,
    pub noise2: StringRef,
    pub noise3: StringRef,
}

/// The number of engine-visible `.string` fields in [`EntVars`] — one native-ABI scratch
/// slot is needed per field (see [`EntVars::link_string_refs`] and `Entity::string_refs`).
pub const STRING_REF_COUNT: usize = 11;

/// Scratch-cell index of `weaponmodel` (its position in [`EntVars::link_string_refs`]).
/// `weaponmodel` has no `setmodel`-style trap — QuakeC sets it by plain string assignment —
/// so we write the resolved `char*` into this cell ourselves (see `GameState::set_weaponmodel`).
pub const STRING_REF_WEAPONMODEL: usize = 2;

/// Scratch-cell index of `netname` (its position in [`EntVars::link_string_refs`]). The engine
/// (notably FTEQW) syncs a connected client's name *from* `v.netname` every frame, so a client —
/// especially a bot, whose edict is cleared with an empty netname — must have this written or it
/// gets renamed to an empty string (and disappears from the scoreboard). See `set_netname`.
pub const STRING_REF_NETNAME: usize = 3;

impl EntVars {
    /// Wire up the native 64-bit string ABI (fteqw API ≥ 15, mvdsv `sv_pr2references`).
    ///
    /// In that ABI the engine does not read a `.string` slot as the string itself: the 4-byte
    /// slot holds a byte offset into the entity array pointing at an 8-byte cell, and the
    /// engine writes the resolved native `char*` there (e.g. on `setmodel`). This points each
    /// string field at its scratch cell; `slot(i)` returns the byte offset of the `i`-th cell,
    /// which the caller owns (`Entity::string_refs`). A field left at `0` makes the engine
    /// silently drop the string, so this must run for every edict the engine clears.
    ///
    /// The array length is checked against [`STRING_REF_COUNT`] at compile time, so adding a
    /// string field to `EntVars` without updating the count (or vice versa) fails to build.
    pub fn link_string_refs(&mut self, slot: impl Fn(usize) -> StringRef) {
        let fields: [&mut StringRef; STRING_REF_COUNT] = [
            &mut self.classname,
            &mut self.model,
            &mut self.weaponmodel,
            &mut self.netname,
            &mut self.target,
            &mut self.targetname,
            &mut self.message,
            &mut self.noise,
            &mut self.noise1,
            &mut self.noise2,
            &mut self.noise3,
        ];
        for (i, field) in fields.into_iter().enumerate() {
            *field = slot(i);
        }
    }
}

/// The engine-shared globals — `globalvars_t` from `ktx/include/progdefs.h`.
/// The engine populates `self`/`other`/`time`/`frametime` and the `trace_*`/`v_*`
/// blocks; we read them. The trailing `*_cb` function-reference fields are legacy QC
/// slots the native module never uses (kept for exact layout).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GlobalVars {
    pub pad: [i32; 28],
    pub self_: i32,
    pub other: i32,
    pub world: i32,
    pub time: f32,
    pub frametime: f32,
    pub newmis: i32,
    pub force_retouch: f32,
    pub mapname: StringRef,
    pub serverflags: f32,
    pub total_secrets: f32,
    pub total_monsters: f32,
    pub found_secrets: f32,
    pub killed_monsters: f32,
    pub parm: [f32; 16],
    pub v_forward: Vec3,
    pub v_up: Vec3,
    pub v_right: Vec3,
    pub trace_allsolid: f32,
    pub trace_startsolid: f32,
    pub trace_fraction: f32,
    pub trace_endpos: Vec3,
    pub trace_plane_normal: Vec3,
    pub trace_plane_dist: f32,
    pub trace_ent: i32,
    pub trace_inopen: f32,
    pub trace_inwater: f32,
    pub msg_entity: i32,
    pub main_cb: FuncRef,
    pub start_frame_cb: FuncRef,
    pub player_pre_think_cb: FuncRef,
    pub player_post_think_cb: FuncRef,
    pub client_kill_cb: FuncRef,
    pub client_connect_cb: FuncRef,
    pub put_client_in_server_cb: FuncRef,
    pub client_disconnect_cb: FuncRef,
    pub set_new_parms_cb: FuncRef,
    pub set_change_parms_cb: FuncRef,
}

/// `fieldtype_t` from `g_public.h` — describes how the engine's entity-string parser
/// should interpret a custom field registered in the [`Field`] table.
#[repr(i32)]
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub enum FieldType {
    Int = 0,
    Float = 1,
    LString = 2,
    Vector = 3,
    AngleHack = 4,
    Ignore = 5,
}

/// `field_t` from `g_public.h`. A null `name` terminates the table.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Field {
    pub name: *const c_char,
    pub ofs: i32,
    pub type_: i32,
}

impl Field {
    /// The terminating sentinel entry.
    pub const TERMINATOR: Field = Field {
        name: core::ptr::null(),
        ofs: 0,
        type_: 0,
    };

    /// Declare an extended entity field by name, byte offset from the entity base, and type —
    /// so the engine writes/reads it at our chosen location (e.g. the player `maxspeed` cap).
    pub fn new(name: &'static core::ffi::CStr, ofs: i32, ty: FieldType) -> Field {
        Field { name: name.as_ptr(), ofs, type_: ty as i32 }
    }
}

/// `gameData_t` from `g_public.h` — returned (by pointer) from `GAME_INIT`. The engine
/// keeps and dereferences `ents`/`global`/`fields` for the whole process lifetime, so
/// they must point at storage that never moves (we point them at heap `Box` buffers
/// owned by `GameState`).
#[repr(C)]
pub struct GameData {
    /// Base of the entity array. Typed as a byte pointer because the engine strides it
    /// by `sizeofent`, reading the [`EntVars`] prefix at each step.
    pub ents: *mut u8,
    pub sizeofent: i32,
    pub global: *mut GlobalVars,
    pub fields: *const Field,
    pub api_version: i32,
    pub maxentities: i32,
}

// --- Layout guards: catch a stray `Vec3A`, reordered field, or wrong-width type at
// compile time rather than as silent memory corruption at runtime. ---
const _: () = assert!(core::mem::size_of::<EntVars>() == 408);
const _: () = assert!(core::mem::align_of::<EntVars>() == 4);
const _: () = assert!(core::mem::size_of::<GlobalVars>() == 360);
const _: () = assert!(core::mem::align_of::<GlobalVars>() == 4);
const _: () = assert!(core::mem::size_of::<Vec3>() == 12);
