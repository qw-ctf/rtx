// SPDX-License-Identifier: AGPL-3.0-or-later

//! Ported from `qw-qc/world.qc` — world setup (`worldspawn`) and per-frame `StartFrame`.

use crate::game::GameState;

/// `worldspawn` — global level setup: gravity, precaches, light-style animation tables.
/// (The QuakeC body's `InitBodyQue`/body-queue setup is deferred to a later milestone.)
pub fn worldspawn(game: &mut GameState) {
    let host = *game.host();

    // worldtype was parked in the world entity's skin during field parsing.
    game.level.worldtype = game.ent(crate::entity::EntId::WORLD).v.skin;

    // Custom map gravity (qw-qc special-cases e1m8).
    let mut buf = [0u8; 64];
    let modelname = host.infokey(crate::entity::EntId::WORLD, c"modelname", &mut buf);
    // Strip "maps/" and ".bsp" to recover the bare map name.
    game.level.mapname = modelname
        .strip_prefix("maps/")
        .unwrap_or(modelname)
        .strip_suffix(".bsp")
        .unwrap_or(modelname)
        .to_owned();
    let is_e1m8 = modelname == "maps/e1m8.bsp";
    host.cvar_set(c"sv_gravity", if is_e1m8 { c"100" } else { c"800" });

    // Every sound, from the single registry (`assets.rs`): the set of nameable sounds *is* the
    // set of precached sounds, so a missing precache is unrepresentable.
    crate::assets::precache_sounds(&host);
    crate::assets::precache_models(&host);

    // Models.








    // Light animation tables. 'a' is total darkness, 'z' is maxbright.
    host.lightstyle(0, c"m"); // 0 normal
    host.lightstyle(1, c"mmnmmommommnonmmonqnmmo"); // 1 FLICKER (first variety)
    host.lightstyle(2, c"abcdefghijklmnopqrstuvwxyzyxwvutsrqponmlkjihgfedcba"); // 2 SLOW STRONG PULSE
    host.lightstyle(3, c"mmmmmaaaaammmmmaaaaaabcdefgabcdefg"); // 3 CANDLE (first variety)
    host.lightstyle(4, c"mamamamamama"); // 4 FAST STROBE
    host.lightstyle(5, c"jklmnopqrstuvwxyzyxwvutsrqponmlkj"); // 5 GENTLE PULSE 1
    host.lightstyle(6, c"nmonqnmomnmomomno"); // 6 FLICKER (second variety)
    host.lightstyle(7, c"mmmaaaabcdefgmmmmaaaammmaamm"); // 7 CANDLE (second variety)
    host.lightstyle(8, c"mmmaaammmaaammmabcdefaaaammmmabcdefmmmaaaa"); // 8 CANDLE (third variety)
    host.lightstyle(9, c"aaaaaaaazzzzzzzz"); // 9 SLOW STROBE (fourth variety)
    host.lightstyle(10, c"mmamammmmammamamaaamammma"); // 10 FLUORESCENT FLICKER
    host.lightstyle(11, c"abcdefghijklmnopqrrqponmlkjihgfedcba"); // 11 SLOW PULSE NOT FADE TO BLACK
    host.lightstyle(63, c"a"); // 63 testing
}

/// `StartFrame` — runs once per server frame. Refreshes match cvars and the frame counter.
pub fn start_frame(game: &mut GameState) {
    let host = *game.host();
    game.level.timelimit = (host.cvar(c"timelimit") * 60.0) as i32;
    game.level.fraglimit = host.cvar(c"fraglimit") as i32;
    game.level.teamplay = host.cvar(c"teamplay") as i32;
    game.level.deathmatch = host.cvar(c"deathmatch") as i32;
    game.level.framecount += 1;
}
