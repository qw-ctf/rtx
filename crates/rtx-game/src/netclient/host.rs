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

use std::cell::{Cell, RefCell};
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
    /// The current map's bare name. `worldspawn` asks for it as the `"modelname"` infokey, which is
    /// how `level.mapname` — and therefore the navmesh's idea of which BSP to load — gets set.
    mapname: RefCell<String>,
    /// The map's entity string and how far through it the spawner has read. The server hands a
    /// module its entities one token at a time; with no server, this is where they come from.
    entities: RefCell<(String, usize)>,
    /// The next entity slot to hand out, counting **down**. See [`alloc_ent`](Self::alloc_ent).
    next_ent: Cell<i32>,
    /// Usercmds the brain emitted this frame.
    cmds: RefCell<Vec<EmittedCmd>>,
    /// Console commands the game queued via `localcmd` that weren't `set`.
    pending_cmds: RefCell<Vec<String>>,
}

/// Picks one physics constant out of the set the server sent.
type MoveVarField = fn(&MoveVars) -> f32;

/// Where the shadow world's entities start, counting down.
///
/// The network hands us entities by *the server's* numbers, and those are small — players are 1..N
/// and everything else follows. Our own spawned copies of the map's furniture can't collide with
/// them, so they're allocated from the far end of the array and grow toward the middle. The two
/// ranges meeting would mean a map with ~2000 entities *and* a server using every slot, which is
/// louder to detect than it is likely.
const SHADOW_TOP: i32 = crate::game::MAX_EDICTS as i32 - 1;

/// The cvars that are the *server's rules*, not ours, and are answered from its serverinfo.
///
/// These decide which entities a map even has. `load_entities` filters spawns on `deathmatch` and
/// `skill`, so a client that reads `deathmatch` as 0 — which an empty cvar store does — spawns the
/// single-player version of a deathmatch map: no weapons on the floor, half the items missing, and
/// a navmesh whose goals point at things the server never placed. `maxclients` is the same kind of
/// load-bearing: it's the loop bound for every scan over players, so at 0 a bot sees an empty
/// server.
///
/// Answered from serverinfo when the server publishes them, because the server is the authority on
/// its own rules; a local value is only the fallback for before it has told us.
const SERVER_RULE_CVARS: &[&str] = &[
    "maxclients",
    "maxspectators",
    "deathmatch",
    "teamplay",
    "skill",
    "timelimit",
    "fraglimit",
    "samelevel",
    "maxfps",
    "watervis",
];

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
            mapname: RefCell::new(String::new()),
            entities: RefCell::new((String::new(), 0)),
            next_ent: Cell::new(SHADOW_TOP),
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

    /// Read a cvar as a string, resolving the ones the server owns from what the server said.
    fn get(&self, name: &str) -> Option<String> {
        if let Some((_, field)) = MOVEVAR_CVARS.iter().find(|(n, _)| *n == name) {
            return Some(field(&self.movevars.borrow()).to_string());
        }
        if SERVER_RULE_CVARS.contains(&name) {
            if let Some(v) = self.serverinfo.borrow().get(name) {
                return Some(v.to_string());
            }
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
        *self.mapname.borrow_mut() = mapname.to_string();
        self.cmds.borrow_mut().clear();
        self.pending_cmds.borrow_mut().clear();

        let path = format!("maps/{mapname}.bsp");
        let Some(bytes) = self.find(&path) else {
            *self.bsp.borrow_mut() = None;
            return false;
        };
        let parsed = Bsp::parse(&bytes);
        let ok = parsed.is_some();
        // A new map is a new entity string, read from the top, and a fresh set of slots.
        *self.entities.borrow_mut() = (parsed.as_ref().map(|b| b.entities.clone()).unwrap_or_default(), 0);
        self.next_ent.set(SHADOW_TOP);
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

/// The answer when there's no map: everything is solid.
///
/// Fail closed, deliberately. A clear line would have the caller believe it can see through the
/// world, and `droptofloor` believe every item in the map is floating in space — which deletes them.
pub(crate) fn no_map(start: Vec3) -> rtx_nav::bsp::HullTrace {
    rtx_nav::bsp::HullTrace {
        all_solid: true,
        start_solid: true,
        fraction: 0.0,
        endpos: start,
        plane_normal: Vec3::ZERO,
    }
}

/// The characters that are a token all by themselves, even with no space around them
/// (`COM_Parse`). `{` and `}` are the ones that matter — they delimit every entity block.
const PUNCTUATION: [char; 6] = ['{', '}', '(', ')', '\'', ':'];

/// Pull the next token out of the entity string, advancing `pos`; `None` at the end.
///
/// id's `COM_Parse`, which is fussier than it looks. A quoted string is one token with the quotes
/// stripped (that's how `"classname" "info_player_deathmatch"` becomes two tokens); braces are
/// tokens on their own with no space needed; `//` runs to end of line. Get any of it wrong and the
/// spawner sees a different map than the server did.
fn next_token(text: &str, pos: &mut usize) -> Option<String> {
    let b = text.as_bytes();
    loop {
        // Skip whitespace, then comments, then whatever whitespace followed the comment.
        while *pos < b.len() && b[*pos] <= b' ' {
            *pos += 1;
        }
        if *pos >= b.len() {
            return None;
        }
        if b[*pos] == b'/' && b.get(*pos + 1) == Some(&b'/') {
            while *pos < b.len() && b[*pos] != b'\n' {
                *pos += 1;
            }
            continue;
        }
        break;
    }

    let c = b[*pos] as char;

    if c == '"' {
        *pos += 1;
        let start = *pos;
        while *pos < b.len() && b[*pos] != b'"' {
            *pos += 1;
        }
        let tok = text[start..*pos].to_string();
        if *pos < b.len() {
            *pos += 1; // consume the closing quote
        }
        // An empty quoted value is a real token (`"targetname" ""`), so this can't fold into the
        // `None` that means end-of-string.
        return Some(tok);
    }

    if PUNCTUATION.contains(&c) {
        *pos += 1;
        return Some(c.to_string());
    }

    let start = *pos;
    while *pos < b.len() && b[*pos] > b' ' && !PUNCTUATION.contains(&(b[*pos] as char)) {
        *pos += 1;
    }
    Some(text[start..*pos].to_string())
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
        // `modelname` isn't a serverinfo key at all — it's how a server tells the module which map
        // it's running, and `worldspawn` reads it to set `level.mapname`. Everything downstream that
        // needs to find the map on disk (the navmesh, most of all) follows from this one answer.
        if key == "modelname" {
            let m = self.mapname.borrow();
            return if m.is_empty() { fill(buf, "") } else { fill(buf, &format!("maps/{m}.bsp")) };
        }
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

    fn world_trace(&self, start: Vec3, end: Vec3) -> rtx_nav::bsp::HullTrace {
        // Hull 1 is the standing-player hull, beveled by the player box at compile time — so a
        // *point* traced through it answers "would a player fit".
        match self.bsp.borrow().as_ref() {
            Some(bsp) => bsp.hull1_trace(start, end),
            None => no_map(start),
        }
    }

    fn world_trace_point(&self, start: Vec3, end: Vec3) -> rtx_nav::bsp::HullTrace {
        // Hull 0 is the real surfaces — what a bullet or a sightline meets.
        match self.bsp.borrow().as_ref() {
            Some(bsp) => bsp.hull0_trace(start, end),
            None => no_map(start),
        }
    }

    fn submodel_bounds(&self, n: usize) -> Option<(Vec3, Vec3)> {
        let bsp = self.bsp.borrow();
        let m = bsp.as_ref()?.submodel(n)?;
        Some((m.mins, m.maxs))
    }

    fn read_file(&self, name: &CStr) -> Option<Vec<u8>> {
        self.find(&name.to_string_lossy())
    }

    fn entity_token<'b>(&self, buf: &'b mut [u8]) -> (bool, &'b str) {
        let mut cursor = self.entities.borrow_mut();
        let (text, pos) = &mut *cursor;
        match next_token(text, pos) {
            Some(tok) => (true, fill(buf, &tok)),
            None => (false, fill(buf, "")),
        }
    }

    fn alloc_ent(&self) -> i32 {
        // Down from the top, away from the server's numbers — see `SHADOW_TOP`.
        let n = self.next_ent.get();
        if n <= 0 {
            return 0; // out of slots; the caller gets the world entity, as a full server would give
        }
        self.next_ent.set(n - 1);
        n
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

    /// The tokenizer is what turns a map file into a world, so every shape the entity string uses
    /// has to come out right: quoted values (with the quotes gone), braces as tokens without
    /// spaces, and comments skipped.
    #[test]
    fn tokenizes_an_entity_block() {
        let text = "\
{
\"classname\" \"info_player_deathmatch\"
\"origin\" \"544 288 32\"
}
// a comment, ignored
{\"classname\" \"item_health\"}
";
        let mut pos = 0;
        let mut toks = Vec::new();
        while let Some(t) = next_token(text, &mut pos) {
            toks.push(t);
        }
        assert_eq!(
            toks,
            vec![
                "{", "classname", "info_player_deathmatch", "origin", "544 288 32", "}",
                "{", "classname", "item_health", "}",
            ]
        );
    }

    /// The awkward cases, each of which appears in real maps and each of which would silently
    /// mis-spawn something if mishandled.
    #[test]
    fn tokenizer_handles_the_awkward_cases() {
        let tok = |text: &str| {
            let (mut pos, mut out) = (0, Vec::new());
            while let Some(t) = next_token(text, &mut pos) {
                out.push(t);
            }
            out
        };

        // An empty quoted value is a token, not the end of the string.
        assert_eq!(tok("\"targetname\" \"\" \"x\" \"1\""), vec!["targetname", "", "x", "1"]);
        // A quoted value may contain spaces and braces without becoming several tokens.
        assert_eq!(tok("\"message\" \"a { b } c\""), vec!["message", "a { b } c"]);
        // Braces need no whitespace around them.
        assert_eq!(tok("{}{}"), vec!["{", "}", "{", "}"]);
        // A comment at the very end, and a comment with no trailing newline.
        assert_eq!(tok("a // b\nc"), vec!["a", "c"]);
        assert_eq!(tok("a // trailing"), vec!["a"]);
        // Nothing at all.
        assert!(tok("").is_empty());
        assert!(tok("   \n\t  ").is_empty());
        assert!(tok("// only a comment").is_empty());
        // An unterminated quote takes the rest rather than looping or panicking.
        assert_eq!(tok("\"unterminated"), vec!["unterminated"]);
    }

    /// The cursor is per map: a new map restarts the string, and the old one's tokens are gone.
    #[test]
    fn entity_cursor_walks_once_and_resets_per_map() {
        let h = host();
        *h.entities.borrow_mut() = ("{ \"a\" \"1\" }".to_string(), 0);

        let mut buf = [0u8; 64];
        let mut seen = Vec::new();
        loop {
            let (more, tok) = h.entity_token(&mut buf);
            if !more {
                break;
            }
            seen.push(tok.to_string());
        }
        assert_eq!(seen, vec!["{", "a", "1", "}"]);

        // Exhausted, and it stays exhausted rather than looping.
        assert_eq!(h.entity_token(&mut buf), (false, ""));
        assert_eq!(h.entity_token(&mut buf), (false, ""));
    }

    /// Shadow entities are allocated from the top down, away from the small numbers the *server*
    /// uses for players and projectiles — the two must never be confused for one another.
    #[test]
    fn allocates_shadow_entities_from_the_top_down() {
        let h = host();
        assert_eq!(h.alloc_ent(), SHADOW_TOP);
        assert_eq!(h.alloc_ent(), SHADOW_TOP - 1);
        assert_eq!(h.alloc_ent(), SHADOW_TOP - 2);
        const { assert!(SHADOW_TOP > 1000, "shadow slots must stay clear of server entity numbers") };

        // Running out yields the world entity rather than an out-of-range slot — the same answer a
        // full server gives.
        h.next_ent.set(0);
        assert_eq!(h.alloc_ent(), 0);
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
