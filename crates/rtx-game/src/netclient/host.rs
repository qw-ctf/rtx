// SPDX-License-Identifier: AGPL-3.0-or-later

//! [`NetHost`] — what answers the game's questions when there's no server to ask.
//!
//! Everything the module normally gets from the engine, sourced locally instead: cvars from our own
//! store rather than the server's, `pointcontents` from the map's own BSP rather than
//! `SV_PointContents`, files from disk rather than the server's filesystem, and the bot's usercmd
//! into a sink rather than into `SV_RunCmd`.
//!
//! Two rules shape the design:
//!
//! **It never touches the entity array or the globals.** The game holds `&mut GameState` while
//! calling these, so writing that memory from here would alias it — see the [`host`](crate::host)
//! module docs. Everything here is `NetHost`'s own state.
//!
//! **Some cvars aren't ours to set.** `sv_gravity` and friends are *the server's* physics, arriving
//! in `svc_serverdata`, and the bot's movement model must use the server's numbers or every jump arc
//! it computes is wrong. So the movevars are routed to whatever the server last told us and are
//! read-only: `worldspawn` tries to `cvar_set("sv_gravity", …)` on map load (`world.rs`), and here
//! that has to be ignored rather than obeyed.

use std::cell::RefCell;
use std::collections::HashMap;
use std::ffi::CStr;
use std::path::{Path, PathBuf};

use glam::Vec3;
use rtx_nav::bsp::Bsp;
use rtx_proto::svc::MoveVars;

use crate::cvars::RTX_CVAR_DEFAULTS;
use crate::entity::EntId;
use crate::host::ClientHost;

/// A usercmd the brain emitted this frame, waiting to be packed into a `clc_move`.
///
/// Read by the session that packs it — the next milestone.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct EmittedCmd {
    /// Which of our connections it belongs to (1-based client number).
    pub client: i32,
    /// View angles: pitch, yaw, roll.
    pub angles: Vec3,
    /// Forward move.
    pub forward: i32,
    /// Strafe move.
    pub side: i32,
    /// Vertical move.
    pub up: i32,
    /// Button bits.
    pub buttons: i32,
    /// Impulse, 0 for none.
    pub impulse: i32,
}

/// The client's stand-in for the engine.
///
/// One per process, leaked so [`HostApi`](crate::host::HostApi) can stay `Copy`, and rebound per
/// map via [`rebind`](Self::rebind).
///
/// Several fields are read only by the session that feeds them — the next milestone — but they're
/// exercised by this module's tests today.
#[allow(dead_code)]
pub(crate) struct NetHost {
    /// Where the game's files live (the directory holding `qw/`, `id1/`, …).
    basedir: PathBuf,
    /// The gamedir the server told us to use.
    gamedir: RefCell<String>,
    /// Our cvars. The server's `sv_*` physics are *not* in here — see [`movevars`](Self::movevars).
    cvars: RefCell<HashMap<String, String>>,
    /// The server's physics, from `svc_serverdata`. What the bot's movement model must run on.
    movevars: RefCell<MoveVars>,
    /// The server's serverinfo, answering [`infokey`](ClientHost::infokey).
    serverinfo: RefCell<rtx_proto::info::Info>,
    /// The current map, for `pointcontents`. `None` until a map is bound.
    bsp: RefCell<Option<Bsp>>,
    /// Usercmds the brain emitted this frame.
    cmds: RefCell<Vec<EmittedCmd>>,
    /// Console commands the game queued via `localcmd` that weren't `set`.
    pending_cmds: RefCell<Vec<String>>,
}

/// Picks one physics constant out of the set the server sent.
type MoveVarField = fn(&MoveVars) -> f32;

/// The server's physics cvars, and the [`MoveVars`] field each reads from.
///
/// These are read-only here: they're the server's, and a write would be the module trying to
/// change physics it doesn't own.
const MOVEVAR_CVARS: &[(&str, MoveVarField)] = &[
    ("sv_gravity", |m| m.gravity),
    ("sv_stopspeed", |m| m.stopspeed),
    ("sv_maxspeed", |m| m.maxspeed),
    ("sv_spectatormaxspeed", |m| m.spectatormaxspeed),
    ("sv_accelerate", |m| m.accelerate),
    ("sv_airaccelerate", |m| m.airaccelerate),
    ("sv_wateraccelerate", |m| m.wateraccelerate),
    ("sv_friction", |m| m.friction),
    ("sv_waterfriction", |m| m.waterfriction),
    ("sv_entgravity", |m| m.entgravity),
];

#[allow(dead_code)]
impl NetHost {
    /// A host rooted at `basedir`, with the rtx tunables seeded to their defaults.
    pub(crate) fn new(basedir: PathBuf) -> Self {
        let host = NetHost {
            basedir,
            gamedir: RefCell::new("qw".to_string()),
            cvars: RefCell::new(HashMap::new()),
            movevars: RefCell::new(MoveVars::default()),
            serverinfo: RefCell::new(rtx_proto::info::Info::new()),
            bsp: RefCell::new(None),
            cmds: RefCell::new(Vec::new()),
            pending_cmds: RefCell::new(Vec::new()),
        };
        // Seed the same defaults the server module registers in GAME_INIT, so a client bot is
        // tuned identically to a qwprogs one unless told otherwise.
        for (name, seed) in RTX_CVAR_DEFAULTS {
            host.set(name, &crate::host::CvarValue::cvar_token(seed));
        }
        host
    }

    /// Set a cvar, unless it's one of the server's (see [`MOVEVAR_CVARS`]).
    pub(crate) fn set(&self, name: &str, value: &str) {
        if MOVEVAR_CVARS.iter().any(|(n, _)| *n == name) {
            return; // the server's physics; ours to read, not to write
        }
        self.cvars.borrow_mut().insert(name.to_string(), value.to_string());
    }

    /// Read a cvar as a string, resolving the server-owned ones from the live movevars.
    fn get(&self, name: &str) -> Option<String> {
        if let Some((_, field)) = MOVEVAR_CVARS.iter().find(|(n, _)| *n == name) {
            return Some(field(&self.movevars.borrow()).to_string());
        }
        self.cvars.borrow().get(name).cloned()
    }

    /// Adopt the physics the server sent in `svc_serverdata`.
    pub(crate) fn set_movevars(&self, m: MoveVars) {
        *self.movevars.borrow_mut() = m;
    }

    /// Adopt the server's serverinfo.
    pub(crate) fn set_serverinfo(&self, info: rtx_proto::info::Info) {
        *self.serverinfo.borrow_mut() = info;
    }

    /// Point at a new map: load its BSP for `pointcontents`, and note the gamedir to search.
    ///
    /// Returns whether the map was found and parsed. A client that can't read the map can't play —
    /// there's no navmesh and no `pointcontents` — so the caller must not proceed without it.
    pub(crate) fn rebind(&self, gamedir: &str, mapname: &str) -> bool {
        *self.gamedir.borrow_mut() = gamedir.to_string();
        self.cmds.borrow_mut().clear();
        self.pending_cmds.borrow_mut().clear();

        let path = format!("maps/{mapname}.bsp");
        let Some(bytes) = self.find(&path) else {
            *self.bsp.borrow_mut() = None;
            return false;
        };
        let parsed = Bsp::parse(&bytes);
        let ok = parsed.is_some();
        *self.bsp.borrow_mut() = parsed;
        ok
    }

    /// Find a game file, searching as a client does: the server's gamedir first (so a mod's
    /// override wins, as it will have for the server), then `qw`, then `id1`.
    fn find(&self, rel: &str) -> Option<Vec<u8>> {
        let gamedir = self.gamedir.borrow().clone();
        let mut seen: Vec<&str> = Vec::new();
        for dir in [gamedir.as_str(), "qw", "id1"] {
            if seen.contains(&dir) {
                continue;
            }
            seen.push(dir);
            let p: PathBuf = Path::new(&self.basedir).join(dir).join(rel);
            if let Ok(bytes) = std::fs::read(&p) {
                return Some(bytes);
            }
        }
        None
    }

    /// Take the usercmds the brain emitted this frame.
    pub(crate) fn take_cmds(&self) -> Vec<EmittedCmd> {
        std::mem::take(&mut self.cmds.borrow_mut())
    }

    /// Take any console commands the game queued that weren't `set`.
    pub(crate) fn take_pending_cmds(&self) -> Vec<String> {
        std::mem::take(&mut self.pending_cmds.borrow_mut())
    }
}

/// Copy a `&str` into a caller's buffer as a NUL-terminated string, and hand back the borrowed
/// `&str` — the shape the engine's buffer-filling traps have, which the game code already expects.
fn fill<'b>(buf: &'b mut [u8], value: &str) -> &'b str {
    let n = value.len().min(buf.len().saturating_sub(1));
    buf[..n].copy_from_slice(&value.as_bytes()[..n]);
    if n < buf.len() {
        buf[n] = 0;
    }
    std::str::from_utf8(&buf[..n]).unwrap_or("")
}

impl ClientHost for NetHost {
    fn cvar(&self, name: &CStr) -> f32 {
        self.get(&name.to_string_lossy())
            .and_then(|v| v.trim().parse().ok())
            .unwrap_or(0.0)
    }

    fn cvar_string<'b>(&self, name: &CStr, buf: &'b mut [u8]) -> &'b str {
        let v = self.get(&name.to_string_lossy()).unwrap_or_default();
        fill(buf, &v)
    }

    fn cvar_set(&self, name: &CStr, value: &CStr) {
        self.set(&name.to_string_lossy(), &value.to_string_lossy());
    }

    fn infokey<'b>(&self, ent: EntId, key: &CStr, buf: &'b mut [u8]) -> &'b str {
        // Only serverinfo (entity 0) is answerable here. Per-player userinfo lives in the mirror,
        // which owns the entities this must not touch.
        if ent != EntId::WORLD {
            return fill(buf, "");
        }
        let key = key.to_string_lossy();
        let info = self.serverinfo.borrow();
        fill(buf, info.get(&key).unwrap_or(""))
    }

    fn pointcontents(&self, p: Vec3) -> f32 {
        // The render hull, not the clip hull: liquids only exist in the former, and this trap's one
        // job for the bots is telling lava from water from air.
        match self.bsp.borrow().as_ref() {
            Some(bsp) => bsp.pointcontents(p) as f32,
            None => rtx_nav::bsp::CONTENTS_EMPTY as f32,
        }
    }

    fn read_file(&self, name: &CStr) -> Option<Vec<u8>> {
        self.find(&name.to_string_lossy())
    }

    fn entity_token<'b>(&self, buf: &'b mut [u8]) -> (bool, &'b str) {
        // The shadow world's entity-lump cursor lands with the shadow world itself.
        (false, fill(buf, ""))
    }

    fn alloc_ent(&self) -> i32 {
        unimplemented!("the shadow world's entity allocator lands with the shadow world")
    }

    fn precache_model(&self, _name: &CStr) {}

    fn precache_sound(&self, _name: &CStr) {}

    fn set_bot_cmd(
        &self,
        client: i32,
        _msec: i32,
        angles: Vec3,
        forward: i32,
        side: i32,
        up: i32,
        buttons: i32,
        impulse: i32,
    ) {
        // `msec` is deliberately dropped. The brain's notion of a frame isn't what the server
        // integrates — it integrates wall-clock time, and a client whose msec runs ahead of the
        // clock is moving faster than real time, which is what a speed cheat looks like. The tick
        // loop stamps the real elapsed time when it builds the packet.
        self.cmds.borrow_mut().push(EmittedCmd {
            client,
            angles,
            forward,
            side,
            up,
            buttons,
            impulse,
        });
    }

    fn localcmd(&self, cmd: &str) {
        // `cvar_default` seeds through `set` because mvdsv's cvar-set builtins refuse to create a
        // cvar that doesn't exist. There's no console here, so interpret `set` ourselves and keep
        // anything else for the caller.
        let mut it = cmd.trim().splitn(3, char::is_whitespace);
        match (it.next(), it.next(), it.next()) {
            (Some("set"), Some(name), value) => {
                self.set(name, value.unwrap_or("").trim().trim_matches('"'));
            }
            _ => self.pending_cmds.borrow_mut().push(cmd.to_string()),
        }
    }

    fn print(&self, msg: &CStr) {
        eprint!("{}", msg.to_string_lossy());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn host() -> NetHost {
        NetHost::new(PathBuf::from("/nonexistent"))
    }

    /// The rtx tunables must arrive already seeded, or a client bot would run with every knob at
    /// zero — no bhop, no skill, no reaction time — and behave nothing like a qwprogs one.
    #[test]
    fn seeds_the_rtx_tunables() {
        let h = host();
        assert_eq!(h.cvar(c"rtx_bot_bhop"), 1.0);
        assert_eq!(h.cvar(c"rtx_bot_skill"), 3.0);
        assert_eq!(h.cvar(c"rtx_bot_fov"), 120.0);

        // A string tunable reads back as itself, and an unknown cvar is 0/"" rather than a panic.
        let mut buf = [0u8; 64];
        assert_eq!(h.cvar_string(c"rtx_mode", &mut buf), "dm");
        assert_eq!(h.cvar(c"nonexistent_cvar"), 0.0);
        assert_eq!(h.cvar_string(c"nonexistent_cvar", &mut [0u8; 8]), "");
    }

    /// The server's physics are read-only: they come from `svc_serverdata`, and `worldspawn` tries
    /// to write `sv_gravity` on every map load. Obeying that would silently desync the bot's
    /// movement model from the server's actual physics.
    #[test]
    fn movevars_come_from_the_server_and_ignore_writes() {
        let h = host();
        assert_eq!(h.cvar(c"sv_gravity"), 800.0, "stock default before any serverdata");

        h.set_movevars(MoveVars {
            gravity: 640.0,
            maxspeed: 400.0,
            ..MoveVars::default()
        });
        assert_eq!(h.cvar(c"sv_gravity"), 640.0);
        assert_eq!(h.cvar(c"sv_maxspeed"), 400.0);

        // What worldspawn does on every map load — and what must not take effect.
        h.cvar_set(c"sv_gravity", c"1234");
        h.localcmd("set sv_gravity 1234");
        assert_eq!(h.cvar(c"sv_gravity"), 640.0, "the server owns gravity, not us");
    }

    /// `cvar_default` seeds through `set` (mvdsv's cvar-set builtins won't create a cvar), so the
    /// localcmd path has to understand `set` or every default would vanish.
    #[test]
    fn localcmd_interprets_set_and_keeps_the_rest() {
        let h = host();
        h.localcmd("set rtx_bot_count 4");
        assert_eq!(h.cvar(c"rtx_bot_count"), 4.0);

        // Quoted values (how string defaults survive a console tokenizer) unwrap.
        h.localcmd("set rtx_mode \"ctf\"");
        assert_eq!(h.cvar_string(c"rtx_mode", &mut [0u8; 16]), "ctf");

        // Anything that isn't a `set` is the caller's problem, not silently dropped.
        h.localcmd("say hello");
        assert_eq!(h.take_pending_cmds(), vec!["say hello".to_string()]);
        assert!(h.take_pending_cmds().is_empty(), "taking drains");
    }

    /// The brain's one output. `msec` is dropped on purpose — see `set_bot_cmd`.
    #[test]
    fn collects_emitted_usercmds() {
        let h = host();
        assert!(h.take_cmds().is_empty());

        h.set_bot_cmd(2, 13, Vec3::new(0.0, 90.0, 0.0), 400, -200, 0, 3, 7);
        let cmds = h.take_cmds();
        assert_eq!(cmds.len(), 1);
        assert_eq!(cmds[0].client, 2);
        assert_eq!(cmds[0].angles.y, 90.0);
        assert_eq!((cmds[0].forward, cmds[0].side), (400, -200));
        assert_eq!((cmds[0].buttons, cmds[0].impulse), (3, 7));
        assert!(h.take_cmds().is_empty(), "taking drains");
    }

    /// With no map bound, `pointcontents` must answer "empty" rather than panic — the brain asks
    /// before a map is loaded, and a crash there would be a crash on connect.
    #[test]
    fn pointcontents_without_a_map_is_empty() {
        let h = host();
        assert_eq!(h.pointcontents(Vec3::ZERO), rtx_nav::bsp::CONTENTS_EMPTY as f32);
        assert!(!h.rebind("qw", "nosuchmap"), "a missing map must report failure");
    }

    /// Serverinfo answers `infokey` for the world; per-player userinfo isn't ours to serve, because
    /// it lives on entities this host must never touch.
    #[test]
    fn infokey_serves_serverinfo_only() {
        let h = host();
        h.set_serverinfo(rtx_proto::info::Info::parse("\\teamplay\\2\\maxfps\\77"));
        assert_eq!(h.infokey(EntId::WORLD, c"teamplay", &mut [0u8; 16]), "2");
        assert_eq!(h.infokey(EntId::WORLD, c"absent", &mut [0u8; 16]), "");
        assert_eq!(h.infokey(EntId(1), c"teamplay", &mut [0u8; 16]), "");
    }

    /// A buffer-filling trap must never overrun the caller's buffer, however long the value.
    #[test]
    fn fill_truncates_rather_than_overruns() {
        let mut buf = [0xffu8; 4];
        assert_eq!(fill(&mut buf, "abcdefgh"), "abc");
        assert_eq!(buf[3], 0, "NUL-terminated");
        assert_eq!(fill(&mut [], "abc"), "", "a zero-length buffer is not a panic");
    }
}
