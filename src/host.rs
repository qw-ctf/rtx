//! The FFI boundary: safe Rust wrappers over the engine's `syscall` function pointer.
//!
//! This is the one module that performs `unsafe` FFI calls. Everything above it speaks
//! ordinary Rust (`i32` entity indices, `f32`, `glam::Vec3`, `&CStr`). Floats are passed
//! to the variadic syscall as their IEEE-754 bit pattern, zero-extended to `isize`
//! (matching ktx's `PASSFLOAT` union trick); the engine reads the low 32 bits back as a
//! `float`.

use core::ffi::CStr;
use glam::Vec3;

/// The host-provided dispatcher. Variadic, C ABI; calling it is `unsafe`.
pub type SyscallFn = unsafe extern "C" fn(arg: isize, ...) -> isize;

/// `gameImport_t` from `g_public.h` — builtin/syscall numbers. Order is load-bearing.
#[repr(isize)]
#[allow(dead_code)]
enum B {
    GetApiVersion,
    DPrint,
    Error,
    GetEntityToken,
    SpawnEnt,
    RemoveEnt,
    PrecacheSound,
    PrecacheModel,
    LightStyle,
    SetOrigin,
    SetSize,
    SetModel,
    BPrint,
    SPrint,
    CenterPrint,
    AmbientSound,
    Sound,
    TraceLine,
    CheckClient,
    StuffCmd,
    LocalCmd,
    Cvar,
    CvarSet,
    FindRadius,
    WalkMove,
    DropToFloor,
    CheckBottom,
    PointContents,
    NextEntity,
    Aim,
    MakeStatic,
    SetSpawnParams,
    ChangeLevel,
    LogFrag,
    GetInfoKey,
    Multicast,
    DisableUpdates,
    WriteByte,
    WriteChar,
    WriteShort,
    WriteLong,
    WriteAngle,
    WriteCoord,
    WriteString,
    WriteEntity,
    FlushSignon,
    MemSet,
    MemCpy,
    StrnCpy,
    Sin,
    Cos,
    Atan2,
    Sqrt,
    Floor,
    Ceil,
    Acos,
    CmdArgc,
    CmdArgv,
    TraceCapsule,
    FsOpenFile,
    FsCloseFile,
    FsReadFile,
    FsWriteFile,
    FsSeekFile,
    FsTellFile,
    FsGetFileList,
    CvarSetFloat,
    CvarString,
    MapExtension,
    StrCmp,
    StrnCmp,
    StriCmp,
    StrniCmp,
    Find,
    ExecuteCommand,
    ConPrint,
    ReadCmd,
    RedirectCmd,
    AddBot,
    RemoveBot,
    SetBotUserInfo,
    SetBotCmd,
    QvmStrftime,
    CmdArgs,
    CmdTokenize,
    StrlCpy,
    StrlCat,
    MakeVectors,
    NextClient,
    PrecacheVwepModel,
    SetPause,
    SetUserInfo,
    MoveToGoal,
    VisibleTo,
}

/// Pack an `f32` as the engine expects it on the variadic syscall: its bit pattern,
/// zero-extended into an `isize`. The engine ignores the upper 32 bits.
#[inline]
fn pf(x: f32) -> isize {
    x.to_bits() as isize
}

/// Unpack an `f32` from a syscall return value (low 32 bits).
#[inline]
fn rf(v: isize) -> f32 {
    f32::from_bits(v as u32)
}

/// Thin, `Copy` handle wrapping the engine's syscall pointer.
#[derive(Clone, Copy)]
pub struct HostApi {
    syscall: SyscallFn,
}

// Several wrappers are foundation for M1/M2 and not yet called.
#[allow(dead_code)]
impl HostApi {
    pub fn new(syscall: SyscallFn) -> Self {
        Self { syscall }
    }

    /// `G_GETAPIVERSION` — the engine's supported API version.
    pub fn api_version(&self) -> i32 {
        unsafe { (self.syscall)(B::GetApiVersion as isize) as i32 }
    }

    /// `G_DPRINT` — print to the server console.
    pub fn dprint(&self, msg: &CStr) {
        unsafe { (self.syscall)(B::DPrint as isize, msg.as_ptr() as isize) };
    }

    /// `G_conprint` — console print (no log redirection).
    pub fn conprint(&self, msg: &CStr) {
        unsafe { (self.syscall)(B::ConPrint as isize, msg.as_ptr() as isize) };
    }

    /// `G_ERROR` — abort the game with a message. The engine does not return.
    pub fn error(&self, msg: &CStr) {
        unsafe { (self.syscall)(B::Error as isize, msg.as_ptr() as isize) };
    }

    /// `G_BPRINT` — broadcast print to all clients at `level` (PRINT_LOW..PRINT_CHAT).
    pub fn bprint(&self, level: i32, msg: &CStr) {
        unsafe { (self.syscall)(B::BPrint as isize, level as isize, msg.as_ptr() as isize, 0) };
    }

    /// `G_SPRINT` — print to a single client at `level`.
    pub fn sprint(&self, ent: i32, level: i32, msg: &CStr) {
        unsafe {
            (self.syscall)(
                B::SPrint as isize,
                ent as isize,
                level as isize,
                msg.as_ptr() as isize,
                0,
            )
        };
    }

    /// `G_CVAR` — read a cvar's float value.
    pub fn cvar(&self, name: &CStr) -> f32 {
        unsafe { rf((self.syscall)(B::Cvar as isize, name.as_ptr() as isize)) }
    }

    /// `G_CVAR_SET` — set a cvar from a string.
    pub fn cvar_set(&self, name: &CStr, value: &CStr) {
        unsafe {
            (self.syscall)(B::CvarSet as isize, name.as_ptr() as isize, value.as_ptr() as isize)
        };
    }

    /// `G_CVAR_SET_FLOAT` — set a cvar from a float.
    pub fn cvar_set_float(&self, name: &CStr, value: f32) {
        unsafe { (self.syscall)(B::CvarSetFloat as isize, name.as_ptr() as isize, pf(value)) };
    }

    /// `G_PRECACHE_MODEL`.
    pub fn precache_model(&self, name: &CStr) {
        unsafe { (self.syscall)(B::PrecacheModel as isize, name.as_ptr() as isize) };
    }

    /// `G_PRECACHE_SOUND`.
    pub fn precache_sound(&self, name: &CStr) {
        unsafe { (self.syscall)(B::PrecacheSound as isize, name.as_ptr() as isize) };
    }

    /// `G_LIGHTSTYLE` — assign an animation string ("a".."z") to a light style slot.
    pub fn lightstyle(&self, style: i32, value: &CStr) {
        unsafe { (self.syscall)(B::LightStyle as isize, style as isize, value.as_ptr() as isize) };
    }

    /// `G_SPAWN_ENT` — allocate an entity, returning its index.
    pub fn spawn(&self) -> i32 {
        unsafe { (self.syscall)(B::SpawnEnt as isize) as i32 }
    }

    /// `G_REMOVE_ENT` — free an entity by index.
    pub fn remove(&self, ent: i32) {
        unsafe { (self.syscall)(B::RemoveEnt as isize, ent as isize) };
    }

    /// `G_SETMODEL` — assign a model (also sets `modelindex`, `mins`/`maxs` for bmodels).
    pub fn set_model(&self, ent: i32, model: &CStr) {
        unsafe { (self.syscall)(B::SetModel as isize, ent as isize, model.as_ptr() as isize) };
    }

    /// `G_SETORIGIN` — move an entity and relink it.
    pub fn set_origin(&self, ent: i32, origin: Vec3) {
        unsafe {
            (self.syscall)(
                B::SetOrigin as isize,
                ent as isize,
                pf(origin.x),
                pf(origin.y),
                pf(origin.z),
            )
        };
    }

    /// `G_SETSIZE` — set the bounding box and relink.
    pub fn set_size(&self, ent: i32, min: Vec3, max: Vec3) {
        unsafe {
            (self.syscall)(
                B::SetSize as isize,
                ent as isize,
                pf(min.x),
                pf(min.y),
                pf(min.z),
                pf(max.x),
                pf(max.y),
                pf(max.z),
            )
        };
    }

    /// `G_SOUND` — play `sample` from `ent` on `channel` (see `defs::CHAN_*`).
    pub fn sound(&self, ent: i32, channel: i32, sample: &CStr, volume: f32, attenuation: f32) {
        unsafe {
            (self.syscall)(
                B::Sound as isize,
                ent as isize,
                channel as isize,
                sample.as_ptr() as isize,
                pf(volume),
                pf(attenuation),
            )
        };
    }

    /// `G_MAKEVECTORS` — compute `v_forward`/`v_right`/`v_up` from `angles` into globals.
    pub fn make_vectors(&self, angles: Vec3) {
        let v = [angles.x, angles.y, angles.z];
        unsafe { (self.syscall)(B::MakeVectors as isize, v.as_ptr() as isize) };
    }

    /// `G_GETINFOKEY` — read a userinfo/serverinfo key into `buf`, returning the value
    /// as a borrowed `&str` (up to the first NUL, lossily decoded). `ent` 0 = serverinfo.
    pub fn infokey<'b>(&self, ent: i32, key: &CStr, buf: &'b mut [u8]) -> &'b str {
        unsafe {
            (self.syscall)(
                B::GetInfoKey as isize,
                ent as isize,
                key.as_ptr() as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        };
        cstr_from_buf(buf)
    }

    /// `G_GetEntityToken` — fetch the next token from the map's entity string into `buf`.
    /// Returns `false` when the entity string is exhausted.
    pub fn get_entity_token<'b>(&self, buf: &'b mut [u8]) -> (bool, &'b str) {
        let more = unsafe {
            (self.syscall)(
                B::GetEntityToken as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        } != 0;
        (more, cstr_from_buf(buf))
    }
}

/// Interpret a NUL-terminated (or full) byte buffer as `&str`, lossily.
#[allow(dead_code)]
fn cstr_from_buf(buf: &[u8]) -> &str {
    let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    core::str::from_utf8(&buf[..end]).unwrap_or("")
}
