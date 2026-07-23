// SPDX-License-Identifier: AGPL-3.0-or-later

//! The host boundary: safe Rust wrappers over whatever is hosting the game logic.
//!
//! This is the one module that performs `unsafe` FFI calls. Everything above it speaks
//! ordinary Rust ([`EntId`] handles, `f32`, `glam::Vec3`, `&CStr`); this layer does the
//! index/bit juggling at the boundary (e.g. `ent.0 as isize`). Floats are passed
//! to the variadic syscall as their IEEE-754 bit pattern, zero-extended to `isize`
//! (matching ktx's `PASSFLOAT` union trick); the engine reads the low 32 bits back as a
//! `float`.
//!
//! # Two hosts
//!
//! Normally the host is a QuakeWorld **server** that loaded us as its game module, reached through
//! its `syscall` pointer ([`Backend::Pr2`]). With the `netclient` feature there's a second: the bot
//! brain running inside a **network client**, where there is no server to ask — the answers come
//! from a [`ClientHost`] that reads the map off disk, keeps its own cvars, and turns the bot's
//! usercmd into a packet rather than a `SV_RunCmd`.
//!
//! The split lives here, in [`Backend`], so that not one line of game or bot code has to know which
//! host it's running under. Three groups of traps:
//!
//! - **Both hosts** — cvars, file reads, the entity-token cursor, prints, and the usercmd sink. These
//!   `match` on the backend. (`pointcontents` used to be here; it now answers from our own parsed
//!   BSP via `GameState::pointcontents`, identically in both hosts — no trap.)
//! - **Server only** — everything that broadcasts, spawns fake clients, or writes the signon. A
//!   real client can't do these and never asks; they reach for [`syscall`](HostApi::syscall), which
//!   says so if the client mode ever does.
//! - **Traps that mutate our own entity array** (`set_origin`, `set_size`, `set_model`,
//!   `droptofloor`, `make_vectors`) — the engine writes those through raw pointers while we hold
//!   `&mut GameState`, which is legal only because the engine isn't Rust. A [`ClientHost`] doing
//!   the same would be Rust-on-Rust aliasing UB, so in client mode they become safe `&mut self`
//!   methods on `GameState` instead. They land with the shadow world that needs them; until then
//!   the client arm says so rather than pretending.

use core::ffi::CStr;
use std::ffi::CString;

use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::defs::{Attenuation, Channel, MsgDest, Multicast, PrintLevel, Svc, Te};
use crate::entity::{EntId, Entity};

/// The host-provided dispatcher. Variadic, C ABI; calling it is `unsafe`.
pub type SyscallFn = unsafe extern "C" fn(arg: isize, ...) -> isize;

/// `gameImport_t` from `g_public.h` — builtin/syscall numbers. Order is load-bearing.
#[repr(isize)]
#[allow(dead_code, clippy::enum_variant_names)]
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

/// Extension syscall numbers (`g_local.h`, `G_EXTENSIONS_FIRST = 256`). Unlike the [`B`]
/// builtins, these are *opt-in*: the module must claim each trap at its number via
/// `Map_Extension` (a `B::MapExtension` call) before the engine routes `syscall(n)` to its
/// handler. The numbers must match the ones the module passes to `Map_Extension`.
#[repr(isize)]
enum Ext {
    MapExtFieldPtr = 265,
    SetExtFieldPtr = 266,
}

/// Why the entity-mutating traps have no client arm yet.
///
/// In server mode the engine writes our entity array through raw pointers while we hold
/// `&mut GameState` — sound only because the engine isn't Rust and its writes sit outside Rust's
/// aliasing model. A [`ClientHost`] doing the same from Rust would alias that `&mut`, which is UB
/// however carefully it's written. The fix isn't a cleverer trap: it's for these to become safe
/// `&mut self` methods on `GameState`, which is where they'll land alongside the shadow world that
/// first needs them. Until then, saying so beats a silent no-op that would corrupt the shadow world
/// somewhere nobody would think to look.
#[cfg(feature = "netclient")]
const MUTATING_TRAP: &str = "entity-mutating trap has no client arm yet — it becomes a GameState \
     method with the shadow world; see the host module docs";

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

/// What a bot brain needs from the world when there's no server to ask.
///
/// Deliberately small: it covers only the traps the *client* actually reaches, which is far less
/// than the server module uses. Everything that broadcasts to other players, spawns fake clients,
/// or writes the signon is the server's job and is absent here by design.
///
/// All methods take `&self` and use interior mutability, because [`HostApi`] is `Copy` and gets
/// snapshotted all over the bot code. Crucially, an implementation **must not touch the entity
/// array or the globals** — the game holds `&mut GameState` while these are called, so writing that
/// memory from here would alias it. Implementations keep their own state (cvars, a `Bsp`, a cmd
/// sink) and nothing else.
///
/// Not `Sync`, deliberately: a client runs its sockets, mirror and brain on one thread, so plain
/// `RefCell` is the right tool and asking for `Sync` would buy only the obligation to explain it.
#[cfg(feature = "netclient")]
pub(crate) trait ClientHost {
    /// A cvar's float value; 0.0 if unset.
    fn cvar(&self, name: &CStr) -> f32;
    /// A cvar's string value, written into `buf`; empty if unset.
    fn cvar_string<'b>(&self, name: &CStr, buf: &'b mut [u8]) -> &'b str;
    /// Set a cvar from a string.
    fn cvar_set(&self, name: &CStr, value: &CStr);
    /// A serverinfo key (and the pseudo-key `"modelname"`).
    fn infokey<'b>(&self, ent: EntId, key: &CStr, buf: &'b mut [u8]) -> &'b str;
    /// The bounds of inline submodel `n` — the shape of a door, plat or trigger.
    fn submodel_bounds(&self, n: usize) -> Option<(Vec3, Vec3)>;
    /// Read a whole file, searching the gamedir then the base game.
    fn read_file(&self, name: &CStr) -> Option<Vec<u8>>;
    /// The next token of the map's entity string; `false` when exhausted.
    fn entity_token<'b>(&self, buf: &'b mut [u8]) -> (bool, &'b str);
    /// Allocate an entity slot for the shadow world.
    fn alloc_ent(&self) -> i32;
    /// Note a model as precached, returning its index.
    fn precache_model(&self, name: &CStr);
    /// Note a sound as precached.
    fn precache_sound(&self, name: &CStr);
    /// Take a bot's usercmd for this frame — the client turns it into a `clc_move`.
    #[allow(clippy::too_many_arguments)]
    fn set_bot_cmd(
        &self,
        client: i32,
        msec: i32,
        angles: Vec3,
        forward: i32,
        side: i32,
        up: i32,
        buttons: i32,
        impulse: i32,
    );
    /// Queue a console command.
    fn localcmd(&self, cmd: &str);
    /// Print a line.
    fn print(&self, msg: &CStr);
}

/// Where a [`HostApi`] sends its questions. See the module docs.
///
/// `Copy`, because `HostApi` is copied pervasively (into `Sense`, into locals) and making it
/// anything else would ripple through the bot code — which is exactly what this seam exists to
/// avoid.
#[derive(Clone, Copy)]
pub(crate) enum Backend {
    /// A QuakeWorld server that loaded us as its game module.
    Pr2(SyscallFn),
    /// A network client running the brain with no server to ask. Leaked once per process, so the
    /// `'static` costs nothing and keeps `HostApi` `Copy`.
    #[cfg(feature = "netclient")]
    Client(&'static dyn ClientHost),
}

/// Thin, `Copy` handle over the host.
#[derive(Clone, Copy)]
pub struct HostApi {
    backend: Backend,
    /// Base of the shared entity array, for the few traps that take an entity by *address*
    /// rather than by index (the map-extension field traps). Stable for the program's life.
    ents: *const Entity,
}

// Several wrappers are foundation for M1/M2 and not yet called.
#[allow(dead_code)]
impl HostApi {
    pub fn new(syscall: SyscallFn, ents: *const Entity) -> Self {
        Self {
            backend: Backend::Pr2(syscall),
            ents,
        }
    }

    /// A handle over a [`ClientHost`], for the brain running inside a network client.
    #[cfg(feature = "netclient")]
    pub(crate) fn new_client(host: &'static dyn ClientHost, ents: *const Entity) -> Self {
        Self {
            backend: Backend::Client(host),
            ents,
        }
    }

    /// An inline submodel's bounds, client-side. See [`ClientHost::submodel_bounds`].
    #[cfg(feature = "netclient")]
    pub(crate) fn submodel_bounds(&self, n: usize) -> Option<(Vec3, Vec3)> {
        match self.backend {
            Backend::Client(c) => c.submodel_bounds(n),
            Backend::Pr2(_) => unreachable!("client-only: a server's setmodel does this itself"),
        }
    }

    /// Whether we're running inside a network client rather than as a server's game module.
    ///
    /// The brain shouldn't branch on this — the whole point of the seam is that it doesn't have
    /// to. It exists for the handful of places where the *server* does something on a bot's behalf
    /// that a real client gets from the server instead (running trigger touches, most notably).
    pub(crate) fn is_client(&self) -> bool {
        match self.backend {
            Backend::Pr2(_) => false,
            #[cfg(feature = "netclient")]
            Backend::Client(_) => true,
        }
    }

    /// The engine's syscall.
    ///
    /// Reaching this in client mode means a server-side path ran that shouldn't have. The traps a
    /// client legitimately meets — the ones the module's own spawn code reaches for on its way past
    /// — carry an explicit [`told_nobody`](Self::told_nobody) arm instead, each saying why doing
    /// nothing is the right answer. This is for the rest, and it's deliberately loud: silently
    /// returning zero from, say, `add_bot` would leave a bug to be found in a live match.
    #[inline]
    fn syscall(&self) -> SyscallFn {
        match self.backend {
            Backend::Pr2(f) => f,
            #[cfg(feature = "netclient")]
            Backend::Client(_) => unreachable!("server-only trap reached from the network client"),
        }
    }

    /// Whether this trap has nobody to talk to, and should quietly do nothing.
    ///
    /// True in client mode, for the traps whose whole purpose is telling *other* players something:
    /// a lightstyle, a sound, an obituary, a temp entity. The server does all of that; we're one of
    /// the players being told. They're reached because the shadow world runs the module's real spawn
    /// code — `worldspawn` assigns lightstyles, an item spawn plays no sound but precaches one — and
    /// running that real code is the entire point, so the traps it passes through must be harmless
    /// rather than fatal.
    #[inline]
    fn told_nobody(&self) -> bool {
        self.is_client()
    }

    /// `G_GETAPIVERSION` — the engine's supported API version.
    pub fn api_version(&self) -> i32 {
        unsafe { (self.syscall())(B::GetApiVersion as isize) as i32 }
    }

    /// `G_DPRINT` — print to the server console.
    pub fn dprint(&self, msg: &CStr) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.print(msg);
        }
        unsafe { (self.syscall())(B::DPrint as isize, msg.as_ptr() as isize) };
    }

    /// `G_conprint` — console print (no log redirection).
    pub fn conprint(&self, msg: &CStr) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.print(msg);
        }
        unsafe { (self.syscall())(B::ConPrint as isize, msg.as_ptr() as isize) };
    }

    /// `G_ERROR` — abort the game with a message. The engine does not return.
    pub fn error(&self, msg: &CStr) {
        unsafe { (self.syscall())(B::Error as isize, msg.as_ptr() as isize) };
    }

    /// `G_BPRINT` — broadcast print to all clients at `level` (PrintLevel::Low..PrintLevel::Chat).
    pub fn bprint(&self, level: PrintLevel, msg: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::BPrint as isize, level.as_i32() as isize, msg.as_ptr() as isize, 0) };
    }

    /// `G_SPRINT` — print to a single client at `level`.
    pub fn sprint(&self, ent: EntId, level: PrintLevel, msg: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe {
            (self.syscall())(
                B::SPrint as isize,
                ent.0 as isize,
                level.as_i32() as isize,
                msg.as_ptr() as isize,
                0,
            )
        };
    }

    /// `G_CVAR` — read a cvar's float value.
    pub fn cvar(&self, name: &CStr) -> f32 {
        match self.backend {
            Backend::Pr2(f) => unsafe { rf(f(B::Cvar as isize, name.as_ptr() as isize)) },
            #[cfg(feature = "netclient")]
            Backend::Client(c) => c.cvar(name),
        }
    }

    /// Read a cvar as a boolean toggle: `> 0.0` is true, `0.0` (or negative) is false.
    pub fn cvar_bool(&self, name: &CStr) -> bool {
        self.cvar(name) > 0.0
    }

    /// `G_CVAR_SET` — set a cvar from a string.
    pub fn cvar_set(&self, name: &CStr, value: &CStr) {
        match self.backend {
            Backend::Pr2(f) => unsafe {
                f(B::CvarSet as isize, name.as_ptr() as isize, value.as_ptr() as isize);
            },
            #[cfg(feature = "netclient")]
            Backend::Client(c) => c.cvar_set(name, value),
        }
    }

    /// `G_CVAR_SET_FLOAT` — set a cvar from a float.
    pub fn cvar_set_float(&self, name: &CStr, value: f32) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            let v = CString::new(value.to_string()).unwrap_or_default();
            return c.cvar_set(name, &v);
        }
        unsafe { (self.syscall())(B::CvarSetFloat as isize, name.as_ptr() as isize, pf(value)) };
    }

    /// Register a default: set the cvar only if it isn't already set. `GAME_INIT` runs on every
    /// map load, so a plain `cvar_set` there would overwrite a value the user put in `server.cfg`
    /// (or a `set` before `map`) each time — this preserves an existing value and only seeds the
    /// default when the cvar is unset (empty string). Generic over the value type so string,
    /// numeric, and boolean defaults read the same, e.g. `cvar_default("rtx_mode", "dm")`,
    /// `cvar_default("rtx_bot_count", 0.0)`, and `cvar_default("rtx_grapple", true)`.
    pub fn cvar_default<V: CvarValue>(&self, name: &str, default: V) {
        // Preserve any existing value (server.cfg, or a prior map) — only seed when unset.
        if self.cvar_is_set(name) {
            return;
        }
        // Seed through the `set` console command, not the `G_CVAR_SET*` builtins: mvdsv's cvar-set
        // builtins are a no-op on a cvar that doesn't exist yet — they refuse to create it
        // ("Cvar_Set: variable ... not found") — so a code default would silently never take (it
        // reads back as 0/""). `set` creates the cvar; fteqw honours it identically. The value is
        // quoted so string defaults survive the console tokenizer.
        //
        // The queued `set` isn't flushed here on purpose: `cvar_default` runs during `GAME_INIT`,
        // before mvdsv sets `pr_global_struct`, so a `G_executecmd` flush would dereference NULL and
        // crash. The buffer flushes on its own before the first game frame — long before any of
        // these cvars is read (the earliest, `rtx_ra_countdown`, only once a round forms).
        self.localcmd(&format!("set {name} \"{}\"", default.cvar_token()));
    }

    /// Whether a cvar currently has a non-empty value (set in server.cfg, by a prior map, or a
    /// default we already seeded). Used by [`cvar_default`](Self::cvar_default) to avoid clobbering,
    /// and by [`crate::control`] to pick the immediate `cvar_set` over a create-via-`set` localcmd.
    pub(crate) fn cvar_is_set(&self, name: &str) -> bool {
        let cname = CString::new(name).unwrap_or_default();
        let mut buf = [0u8; 64];
        !self.cvar_string(&cname, &mut buf).is_empty()
    }

    /// `G_PRECACHE_MODEL`.
    pub fn precache_model(&self, name: &CStr) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.precache_model(name);
        }
        unsafe { (self.syscall())(B::PrecacheModel as isize, name.as_ptr() as isize) };
    }

    /// `G_PRECACHE_SOUND`.
    pub fn precache_sound(&self, name: &CStr) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.precache_sound(name);
        }
        unsafe { (self.syscall())(B::PrecacheSound as isize, name.as_ptr() as isize) };
    }

    /// `G_LIGHTSTYLE` — assign an animation string ("a".."z") to a light style slot.
    pub fn lightstyle(&self, style: i32, value: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::LightStyle as isize, style as isize, value.as_ptr() as isize) };
    }

    /// `G_FlushSignon` — commit the current signon block (precaches + baselines) and start a
    /// new one. Must be called per entity during `GAME_LOADENTS` or the signon overflows and
    /// later entities never reach clients (matches ktx's spawn loop).
    pub fn flush_signon(&self) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::FlushSignon as isize) };
    }

    /// `G_SPAWN_ENT` — allocate an entity, returning its index.
    pub fn spawn(&self) -> i32 {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.alloc_ent();
        }
        unsafe { (self.syscall())(B::SpawnEnt as isize) as i32 }
    }

    /// `G_REMOVE_ENT` — free an entity by index.
    pub fn remove(&self, ent: EntId) {
        unsafe { (self.syscall())(B::RemoveEnt as isize, ent.0 as isize) };
    }

    /// `G_SETMODEL` — assign an external [`Model`] (also sets `modelindex`, `mins`/`maxs`). Takes
    /// a handle, which only comes from a precached source — so the model is provably precached.
    pub fn set_model(&self, ent: EntId, model: Model) {
        self.set_model_raw(ent, model.path());
    }

    /// `G_SETMODEL` for an inline brush submodel (`*N`) supplied by the map. These are part of
    /// the level BSP and engine-managed (no precache), so they take a raw name, not a [`Model`].
    pub fn set_model_brush(&self, ent: EntId, name: &CStr) {
        self.set_model_raw(ent, name);
    }

    fn set_model_raw(&self, ent: EntId, model: &CStr) {
        #[cfg(feature = "netclient")]
        debug_assert!(!self.is_client(), "{}", MUTATING_TRAP);
        unsafe { (self.syscall())(B::SetModel as isize, ent.0 as isize, model.as_ptr() as isize) };
    }

    /// `G_SETORIGIN` — move an entity and relink it.
    pub fn set_origin(&self, ent: EntId, origin: Vec3) {
        #[cfg(feature = "netclient")]
        debug_assert!(!self.is_client(), "{}", MUTATING_TRAP);
        unsafe {
            (self.syscall())(
                B::SetOrigin as isize,
                ent.0 as isize,
                pf(origin.x),
                pf(origin.y),
                pf(origin.z),
            )
        };
    }

    /// `G_SETSIZE` — set the bounding box and relink.
    pub fn set_size(&self, ent: EntId, min: Vec3, max: Vec3) {
        #[cfg(feature = "netclient")]
        debug_assert!(!self.is_client(), "{}", MUTATING_TRAP);
        unsafe {
            (self.syscall())(
                B::SetSize as isize,
                ent.0 as isize,
                pf(min.x),
                pf(min.y),
                pf(min.z),
                pf(max.x),
                pf(max.y),
                pf(max.z),
            )
        };
    }

    /// `G_VISIBLETO` — fill `buf` (one byte per entity in `[first, first+count)`) with
    /// whether each is in `viewer`'s PVS. Returns the count visible.
    pub fn visible_to(&self, viewer: EntId, first: i32, count: i32, buf: &mut [u8]) -> i32 {
        unsafe {
            (self.syscall())(
                B::VisibleTo as isize,
                viewer.0 as isize,
                first as isize,
                count as isize,
                buf.as_mut_ptr() as isize,
            ) as i32
        }
    }

    /// `G_SOUND` — play `sample` from `ent` on `channel`. Takes a [`Sound`] handle, which can
    /// only come from a precached source — so playing an unprecached sound is unrepresentable.
    pub fn sound(&self, ent: EntId, channel: Channel, sample: Sound, volume: f32, attenuation: Attenuation) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        self.sound_raw(ent, channel.as_i32(), sample.path(), volume, attenuation);
    }

    /// As [`sound`](Self::sound), but with the `CHAN_NO_PHS_ADD` modifier (channel bit 3) set so
    /// the sound bypasses the PHS cull — used for door/plat movement, audible through walls.
    pub fn sound_no_phs(&self, ent: EntId, channel: Channel, sample: Sound, volume: f32, attenuation: Attenuation) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        self.sound_raw(ent, channel.as_i32() | 8, sample.path(), volume, attenuation);
    }

    fn sound_raw(&self, ent: EntId, channel: i32, sample: &CStr, volume: f32, attenuation: Attenuation) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe {
            (self.syscall())(
                B::Sound as isize,
                ent.0 as isize,
                channel as isize,
                sample.as_ptr() as isize,
                pf(volume),
                pf(attenuation.as_f32()),
            )
        };
    }

    /// `G_MAKEVECTORS` — compute `v_forward`/`v_right`/`v_up` from `angles` into globals.
    pub fn make_vectors(&self, angles: Vec3) {
        #[cfg(feature = "netclient")]
        debug_assert!(!self.is_client(), "{}", MUTATING_TRAP);
        let v = [angles.x, angles.y, angles.z];
        unsafe { (self.syscall())(B::MakeVectors as isize, v.as_ptr() as isize) };
    }

    /// `G_CMD_ARGC` — number of tokens in the current client/console command.
    pub fn cmd_argc(&self) -> i32 {
        unsafe { (self.syscall())(B::CmdArgc as isize) as i32 }
    }

    /// `G_CMD_ARGV` — token `n` of the current command, into `buf`, as a borrowed `&str`.
    pub fn cmd_argv<'b>(&self, n: i32, buf: &'b mut [u8]) -> &'b str {
        unsafe {
            (self.syscall())(
                B::CmdArgv as isize,
                n as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        };
        cstr_from_buf(buf)
    }

    /// `G_CVAR_STRING` — a cvar's string value into `buf`, as a borrowed `&str`.
    pub fn cvar_string<'b>(&self, name: &CStr, buf: &'b mut [u8]) -> &'b str {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.cvar_string(name, buf);
        }
        unsafe {
            (self.syscall())(
                B::CvarString as isize,
                name.as_ptr() as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        };
        cstr_from_buf(buf)
    }

    /// `G_GETINFOKEY` — read a userinfo/serverinfo key into `buf`, returning the value
    /// as a borrowed `&str` (up to the first NUL, lossily decoded). `ent` 0 = serverinfo.
    pub fn infokey<'b>(&self, ent: EntId, key: &CStr, buf: &'b mut [u8]) -> &'b str {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.infokey(ent, key, buf);
        }
        unsafe {
            (self.syscall())(
                B::GetInfoKey as isize,
                ent.0 as isize,
                key.as_ptr() as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        };
        cstr_from_buf(buf)
    }

    /// `G_DROPTOFLOOR` — drop an entity straight down onto the floor; returns whether it
    /// landed on a valid surface.
    pub fn droptofloor(&self, ent: EntId) -> bool {
        #[cfg(feature = "netclient")]
        debug_assert!(!self.is_client(), "{}", MUTATING_TRAP);
        unsafe { (self.syscall())(B::DropToFloor as isize, ent.0 as isize) != 0 }
    }

    /// `G_CENTERPRINT` — center-screen message to one client.
    pub fn centerprint(&self, ent: EntId, msg: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::CenterPrint as isize, ent.0 as isize, msg.as_ptr() as isize) };
    }

    /// `G_CHANGELEVEL` — request a map change.
    pub fn changelevel(&self, name: &CStr) {
        unsafe { (self.syscall())(B::ChangeLevel as isize, name.as_ptr() as isize, 0) };
    }

    /// `G_SETSPAWNPARAMS` — persist a client's spawn parameters.
    pub fn set_spawn_params(&self, ent: EntId) {
        unsafe { (self.syscall())(B::SetSpawnParams as isize, ent.0 as isize) };
    }

    /// `G_LOGFRAG` — record a frag for stats/MVD.
    pub fn logfrag(&self, killer: EntId, killee: EntId) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::LogFrag as isize, killer.0 as isize, killee.0 as isize) };
    }

    /// `G_STUFFCMD` — send a command to a client's console.
    pub fn stuffcmd(&self, ent: EntId, cmd: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::StuffCmd as isize, ent.0 as isize, cmd.as_ptr() as isize, 0) };
    }

    /// `G_LOCALCMD` — append a console command to the server's command buffer (run at the next
    /// flush). A terminating newline is added, so pass the command without one.
    pub fn localcmd(&self, cmd: &str) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.localcmd(cmd);
        }
        if let Ok(cmd) = CString::new(format!("{cmd}\n")) {
            unsafe { (self.syscall())(B::LocalCmd as isize, cmd.as_ptr() as isize) };
        }
    }

    /// `G_MAKESTATIC` — turn an entity into a static (client-side only) entity and remove it.
    pub fn makestatic(&self, ent: EntId) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::MakeStatic as isize, ent.0 as isize) };
    }

    /// `G_SETPAUSE`.
    pub fn set_pause(&self, paused: bool) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::SetPause as isize, paused as isize) };
    }

    /// `G_AMBIENTSOUND` — attach a looping ambient sound at a point.
    pub fn ambient_sound(&self, pos: Vec3, sample: Sound, volume: f32, attenuation: Attenuation) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe {
            (self.syscall())(
                B::AmbientSound as isize,
                pf(pos.x),
                pf(pos.y),
                pf(pos.z),
                sample.path().as_ptr() as isize,
                pf(volume),
                pf(attenuation.as_f32()),
            )
        };
    }

    // --- network message writing (multicast / temp entities / kicks) ---

    /// `G_MULTICAST` — send the buffered `write_*` message to a recipient set.
    pub fn multicast(&self, origin: Vec3, to: Multicast) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe {
            (self.syscall())(
                B::Multicast as isize,
                pf(origin.x),
                pf(origin.y),
                pf(origin.z),
                to.as_i32() as isize,
            )
        };
    }

    pub fn write_byte(&self, to: MsgDest, v: i32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteByte as isize, to.as_i32() as isize, v as isize) };
    }
    pub fn write_char(&self, to: MsgDest, v: i32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteChar as isize, to.as_i32() as isize, v as isize) };
    }
    pub fn write_short(&self, to: MsgDest, v: i32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteShort as isize, to.as_i32() as isize, v as isize) };
    }
    pub fn write_long(&self, to: MsgDest, v: i32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteLong as isize, to.as_i32() as isize, v as isize) };
    }
    pub fn write_coord(&self, to: MsgDest, v: f32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteCoord as isize, to.as_i32() as isize, pf(v)) };
    }
    pub fn write_angle(&self, to: MsgDest, v: f32) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteAngle as isize, to.as_i32() as isize, pf(v)) };
    }
    pub fn write_string(&self, to: MsgDest, s: &CStr) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteString as isize, to.as_i32() as isize, s.as_ptr() as isize) };
    }
    pub fn write_entity(&self, to: MsgDest, ent: EntId) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        unsafe { (self.syscall())(B::WriteEntity as isize, to.as_i32() as isize, ent.0 as isize) };
    }

    /// Write a server-to-client opcode byte (`svc_*`).
    pub fn write_svc(&self, to: MsgDest, svc: Svc) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        self.write_byte(to, svc.as_i32());
    }

    /// Begin a temp-entity message: emit the `svc_temp_entity` header and the effect byte.
    /// The caller follows with the effect's payload (coords / entity / count) and a
    /// [`multicast`](Self::multicast).
    pub fn write_te(&self, to: MsgDest, te: Te) {
        #[cfg(feature = "netclient")]
        if self.told_nobody() {
            return;
        }
        self.write_byte(to, Svc::TempEntity.as_i32());
        self.write_byte(to, te.as_i32());
    }

    // --- map-extension fields (alpha, colormod, …) ---

    /// `G_Map_Extension` — claim extension `name` at syscall number `mapto`, so subsequent
    /// `syscall(mapto)` calls route to it. Returns `mapto` on success, negative if the server
    /// doesn't provide the extension. Must be called before invoking the trap.
    fn map_extension(&self, name: &CStr, mapto: isize) -> isize {
        unsafe { (self.syscall())(B::MapExtension as isize, name.as_ptr() as isize, mapto) }
    }

    /// Claim the `MapExtFieldPtr`/`SetExtFieldPtr` traps (used for the `alpha` field) at the
    /// numbers ktx uses. Returns whether both are available on this server.
    ///
    /// A network client has no engine to claim them from, and no need to: `alpha`/`colormod` are
    /// cosmetic, and a headless bot renders nothing. Reporting "unsupported" makes every
    /// [`ExtFields::set`](crate::ext_field::ExtFields::set) a no-op — which is the same best-effort
    /// path a server that lacks the extension takes — instead of reaching a server-only trap when a
    /// map (a CTF map, say) carries an `alpha` field on some entity.
    pub fn register_ext_fields(&self) -> bool {
        if self.is_client() {
            return false;
        }
        self.map_extension(c"MapExtFieldPtr", Ext::MapExtFieldPtr as isize) >= 0
            && self.map_extension(c"SetExtFieldPtr", Ext::SetExtFieldPtr as isize) >= 0
    }

    /// `MapExtFieldPtr` — resolve a named map-extension field (e.g. `"alpha"`) to an opaque
    /// field reference: a byte offset into the engine's `ext_entvars_t`, tagged with a
    /// validation cookie. Returns 0 if the field is unknown. Cache the result.
    pub fn map_ext_field_ptr(&self, name: &CStr) -> u32 {
        unsafe { (self.syscall())(Ext::MapExtFieldPtr as isize, name.as_ptr() as isize) as u32 }
    }

    /// `SetExtFieldPtr` — write `value` into entity `ent`'s extension field `field_ref` (from
    /// [`map_ext_field_ptr`](Self::map_ext_field_ptr)). Generic over the field's value type, so
    /// the byte size the trap needs is derived from `T` — an `f32` for `alpha`, a `[f32; 3]` for
    /// `colormod`, and so on. Unlike the index-based builtins, this trap takes the entity by
    /// *address* (the engine derives the edict from it); that — and the value pointer — stay
    /// inside this layer rather than leaking to callers.
    pub fn set_ext_field<T: Copy>(&self, ent: EntId, field_ref: u32, value: &T) {
        unsafe {
            (self.syscall())(
                Ext::SetExtFieldPtr as isize,
                self.ent_ptr(ent) as isize,
                field_ref as isize,
                value as *const T as isize,
                core::mem::size_of::<T>() as isize,
            )
        };
    }

    /// Address of entity `ent` in the shared array — what the map-extension field traps expect
    /// in place of an index.
    fn ent_ptr(&self, ent: EntId) -> *const Entity {
        self.ents.wrapping_add(ent.0 as usize)
    }

    // --- filesystem (reading the map's BSP for the navmesh) ---

    /// `G_FSOpenFile` for reading — opens `name` (under the gamedir/paks; `name` must pass the
    /// engine's `FS_UnsafeFilename` check) in binary read mode. Returns `(handle, len)`, or
    /// `None` if the file can't be opened. The engine writes the handle through a pointer arg
    /// and returns the byte length (`-1` on failure).
    fn fs_open_read(&self, name: &CStr) -> Option<(i32, i32)> {
        // fsMode_t::FS_READ_BIN == 0 (mvdsv `g_public.h`).
        const FS_READ_BIN: isize = 0;
        let mut handle: i32 = 0;
        let len = unsafe {
            (self.syscall())(
                B::FsOpenFile as isize,
                name.as_ptr() as isize,
                &mut handle as *mut i32 as isize,
                FS_READ_BIN,
            ) as i32
        };
        (len >= 0 && handle != 0).then_some((handle, len))
    }

    /// `G_FSReadFile` — read up to `buf.len()` bytes from `handle`. Returns bytes read (`<0` on
    /// error).
    fn fs_read(&self, buf: &mut [u8], handle: i32) -> i32 {
        unsafe {
            (self.syscall())(
                B::FsReadFile as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
                handle as isize,
            ) as i32
        }
    }

    /// `G_FSCloseFile` — release a file handle.
    fn fs_close(&self, handle: i32) {
        unsafe { (self.syscall())(B::FsCloseFile as isize, handle as isize) };
    }

    /// Read an entire file into a `Vec<u8>` (open + read + close). `None` if it can't be opened
    /// or read. Used to slurp `maps/<name>.bsp` for the navmesh build.
    pub fn read_file(&self, name: &CStr) -> Option<Vec<u8>> {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.read_file(name);
        }
        let (handle, len) = self.fs_open_read(name)?;
        let mut buf = vec![0u8; len.max(0) as usize];
        let n = self.fs_read(&mut buf, handle);
        self.fs_close(handle);
        if n < 0 {
            return None;
        }
        buf.truncate(n as usize);
        Some(buf)
    }

    // --- bots (fake clients driven by the module) ---

    /// `G_Add_Bot` — spawn a fake client (runs the module's ClientConnect + PutClientInServer).
    /// `bottom`/`top` are the lower/upper shirt colors (0–13). Returns the 1-based client number,
    /// or 0 if the server is full.
    pub fn add_bot(&self, name: &CStr, bottom: i32, top: i32, skin: &CStr) -> i32 {
        unsafe {
            (self.syscall())(
                B::AddBot as isize,
                name.as_ptr() as isize,
                bottom as isize,
                top as isize,
                skin.as_ptr() as isize,
            ) as i32
        }
    }

    /// `G_Remove_Bot` — disconnect a bot by its 1-based client number.
    pub fn remove_bot(&self, client: i32) {
        unsafe { (self.syscall())(B::RemoveBot as isize, client as isize) };
    }

    /// `G_SETUSERINFO` — set a userinfo key on any client server-side (`flags` 0 for normal
    /// userinfo). Used to force a human player's `"team"`/colours in a team match; bots use
    /// [`set_bot_userinfo`](Self::set_bot_userinfo).
    pub fn set_userinfo(&self, client: i32, key: &CStr, value: &CStr, flags: i32) {
        unsafe {
            (self.syscall())(
                B::SetUserInfo as isize,
                client as isize,
                key.as_ptr() as isize,
                value.as_ptr() as isize,
                flags as isize,
            )
        };
    }

    /// `G_SetBotUserInfo` — set a userinfo key on a bot client (`flags` 0 for normal userinfo).
    pub fn set_bot_userinfo(&self, client: i32, key: &CStr, value: &CStr, flags: i32) {
        unsafe {
            (self.syscall())(
                B::SetBotUserInfo as isize,
                client as isize,
                key.as_ptr() as isize,
                value.as_ptr() as isize,
                flags as isize,
            )
        };
    }

    /// `G_SetBotCMD` — feed a bot its usercmd for this frame. `angles` is `(pitch, yaw, roll)`;
    /// `forward`/`side`/`up` are the integer move components; `buttons`/`impulse` as usual. The
    /// engine runs this through the same `SV_RunCmd`/`PM_PlayerMove` as a human client. Must be
    /// re-sent every frame — the engine reuses the last cmd otherwise.
    #[allow(clippy::too_many_arguments)]
    pub fn set_bot_cmd(
        &self,
        client: i32,
        msec: i32,
        angles: Vec3,
        forward: i32,
        side: i32,
        up: i32,
        buttons: i32,
        impulse: i32,
    ) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.set_bot_cmd(client, msec, angles, forward, side, up, buttons, impulse);
        }
        unsafe {
            (self.syscall())(
                B::SetBotCmd as isize,
                client as isize,
                msec as isize,
                pf(angles.x),
                pf(angles.y),
                pf(angles.z),
                forward as isize,
                side as isize,
                up as isize,
                buttons as isize,
                impulse as isize,
            )
        };
    }

    /// `G_TraceCapsule` — like [`traceline`](Self::traceline) but sweeps a box (`mins`/`maxs`)
    /// from `start` to `end`, writing results into the engine's `trace_*` globals. Used to
    /// verify jump/drop arcs clear geometry. `nomonsters` follows QuakeC.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_capsule(&self, start: Vec3, end: Vec3, nomonsters: bool, ignore: EntId, mins: Vec3, maxs: Vec3) {
        unsafe {
            (self.syscall())(
                B::TraceCapsule as isize,
                pf(start.x),
                pf(start.y),
                pf(start.z),
                pf(end.x),
                pf(end.y),
                pf(end.z),
                nomonsters as isize,
                ignore.0 as isize,
                pf(mins.x),
                pf(mins.y),
                pf(mins.z),
                pf(maxs.x),
                pf(maxs.y),
                pf(maxs.z),
            )
        };
    }

    /// `G_GetEntityToken` — fetch the next token from the map's entity string into `buf`.
    /// Returns `false` when the entity string is exhausted.
    pub fn get_entity_token<'b>(&self, buf: &'b mut [u8]) -> (bool, &'b str) {
        #[cfg(feature = "netclient")]
        if let Backend::Client(c) = self.backend {
            return c.entity_token(buf);
        }
        let more = unsafe {
            (self.syscall())(
                B::GetEntityToken as isize,
                buf.as_mut_ptr() as isize,
                buf.len() as isize,
            )
        } != 0;
        (more, cstr_from_buf(buf))
    }
}

/// A value that can seed a cvar default via [`HostApi::cvar_default`] — implemented for `f32`
/// (numeric cvars), `bool` (0/1 toggles), and `&str` (string cvars), so one `cvar_default` handles
/// all three. The value is rendered as the argument to a `set` console command.
pub trait CvarValue {
    fn cvar_token(&self) -> String;
}

impl CvarValue for f32 {
    fn cvar_token(&self) -> String {
        format!("{self}")
    }
}

impl CvarValue for bool {
    fn cvar_token(&self) -> String {
        // `1`/`0` so the same value reads back as truthy/falsy via `HostApi::cvar_bool`.
        if *self { "1" } else { "0" }.to_string()
    }
}

impl CvarValue for &str {
    fn cvar_token(&self) -> String {
        (*self).to_string()
    }
}

/// Interpret a NUL-terminated (or full) byte buffer as `&str`, lossily.
#[allow(dead_code)]
fn cstr_from_buf(buf: &[u8]) -> &str {
    let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    core::str::from_utf8(&buf[..end]).unwrap_or("")
}
