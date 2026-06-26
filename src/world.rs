//! Ported from `qw-qc/world.qc` — world setup (`worldspawn`) and per-frame `StartFrame`.

use crate::game::GameState;
use crate::host::HostApi;

/// `worldspawn` — global level setup: gravity, precaches, light-style animation tables.
/// (The QuakeC body's `InitBodyQue`/body-queue setup is deferred to a later milestone.)
pub fn worldspawn(game: &mut GameState) {
    let host = *game.host();

    // worldtype was parked in the world entity's skin during field parsing.
    game.level.worldtype = game.ent(crate::entity::EntId::WORLD).v.skin;

    // Custom map gravity (qw-qc special-cases e1m8).
    let mut buf = [0u8; 64];
    let modelname = host.infokey(0, c"modelname", &mut buf);
    // Strip "maps/" and ".bsp" to recover the bare map name.
    game.level.mapname = modelname
        .strip_prefix("maps/")
        .unwrap_or(modelname)
        .strip_suffix(".bsp")
        .unwrap_or(modelname)
        .to_owned();
    let is_e1m8 = modelname == "maps/e1m8.bsp";
    host.cvar_set(c"sv_gravity", if is_e1m8 { c"100" } else { c"800" });

    // Weapon precaches.
    w_precache(&host);

    // Sounds used from the C physics code.
    host.precache_sound(c"demon/dland2.wav"); // landing thud
    host.precache_sound(c"misc/h2ohit1.wav"); // landing splash

    // Setup precaches always needed.
    host.precache_sound(c"items/itembk2.wav"); // item respawn sound
    host.precache_sound(c"player/plyrjmp8.wav"); // player jump
    host.precache_sound(c"player/land.wav"); // player landing
    host.precache_sound(c"player/land2.wav"); // player hurt landing
    host.precache_sound(c"player/drown1.wav"); // drowning pain
    host.precache_sound(c"player/drown2.wav"); // drowning pain
    host.precache_sound(c"player/gasp1.wav"); // gasping for air
    host.precache_sound(c"player/gasp2.wav"); // taking breath
    host.precache_sound(c"player/h2odeath.wav"); // drowning death

    host.precache_sound(c"misc/talk.wav"); // talk
    host.precache_sound(c"player/teledth1.wav"); // telefrag
    host.precache_sound(c"misc/r_tele1.wav"); // teleport sounds
    host.precache_sound(c"misc/r_tele2.wav");
    host.precache_sound(c"misc/r_tele3.wav");
    host.precache_sound(c"misc/r_tele4.wav");
    host.precache_sound(c"misc/r_tele5.wav");
    host.precache_sound(c"weapons/lock4.wav"); // ammo pick up
    host.precache_sound(c"weapons/pkup.wav"); // weapon up
    host.precache_sound(c"items/armor1.wav"); // armor up
    host.precache_sound(c"weapons/lhit.wav"); // lightning
    host.precache_sound(c"weapons/lstart.wav"); // lightning start
    host.precache_sound(c"items/damage3.wav");

    host.precache_sound(c"misc/power.wav"); // lightning for boss

    // Player gib sounds.
    host.precache_sound(c"player/gib.wav");
    host.precache_sound(c"player/udeath.wav");
    host.precache_sound(c"player/tornoff2.wav");

    // Player pain sounds.
    host.precache_sound(c"player/pain1.wav");
    host.precache_sound(c"player/pain2.wav");
    host.precache_sound(c"player/pain3.wav");
    host.precache_sound(c"player/pain4.wav");
    host.precache_sound(c"player/pain5.wav");
    host.precache_sound(c"player/pain6.wav");

    // Player death sounds.
    host.precache_sound(c"player/death1.wav");
    host.precache_sound(c"player/death2.wav");
    host.precache_sound(c"player/death3.wav");
    host.precache_sound(c"player/death4.wav");
    host.precache_sound(c"player/death5.wav");

    host.precache_sound(c"boss1/sight1.wav");

    // Axe sounds.
    host.precache_sound(c"weapons/ax1.wav"); // ax swoosh
    host.precache_sound(c"player/axhit1.wav"); // ax hit meat
    host.precache_sound(c"player/axhit2.wav"); // ax hit world

    host.precache_sound(c"player/h2ojump.wav"); // player jumping into water
    host.precache_sound(c"player/slimbrn2.wav"); // player enter slime
    host.precache_sound(c"player/inh2o.wav"); // player enter water
    host.precache_sound(c"player/inlava.wav"); // player enter lava
    host.precache_sound(c"misc/outwater.wav"); // leaving water sound

    host.precache_sound(c"player/lburn1.wav"); // lava burn
    host.precache_sound(c"player/lburn2.wav"); // lava burn

    host.precache_sound(c"misc/water1.wav"); // swimming
    host.precache_sound(c"misc/water2.wav"); // swimming

    // Invulnerability sounds.
    host.precache_sound(c"items/protect.wav");
    host.precache_sound(c"items/protect2.wav");
    host.precache_sound(c"items/protect3.wav");

    // Models.
    host.precache_model(c"progs/player.mdl");
    host.precache_model(c"progs/eyes.mdl");
    host.precache_model(c"progs/h_player.mdl");
    host.precache_model(c"progs/gib1.mdl");
    host.precache_model(c"progs/gib2.mdl");
    host.precache_model(c"progs/gib3.mdl");

    host.precache_model(c"progs/s_bubble.spr"); // drowning bubbles
    host.precache_model(c"progs/s_explod.spr"); // sprite explosion

    host.precache_model(c"progs/v_axe.mdl");
    host.precache_model(c"progs/v_shot.mdl");
    host.precache_model(c"progs/v_nail.mdl");
    host.precache_model(c"progs/v_rock.mdl");
    host.precache_model(c"progs/v_shot2.mdl");
    host.precache_model(c"progs/v_nail2.mdl");
    host.precache_model(c"progs/v_rock2.mdl");

    host.precache_model(c"progs/bolt.mdl"); // lightning gun
    host.precache_model(c"progs/bolt2.mdl"); // lightning gun
    host.precache_model(c"progs/bolt3.mdl"); // boss shock
    host.precache_model(c"progs/lavaball.mdl");

    host.precache_model(c"progs/missile.mdl");
    host.precache_model(c"progs/grenade.mdl");
    host.precache_model(c"progs/spike.mdl");
    host.precache_model(c"progs/s_spike.mdl");

    host.precache_model(c"progs/backpack.mdl");

    host.precache_model(c"progs/zom_gib.mdl");

    host.precache_model(c"progs/v_light.mdl");

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

/// `W_Precache` from `qw-qc/weapons.qc` — weapon sound precaches needed at world setup.
fn w_precache(host: &HostApi) {
    host.precache_sound(c"weapons/r_exp3.wav"); // new rocket explosion
    host.precache_sound(c"weapons/rocket1i.wav"); // spike gun
    host.precache_sound(c"weapons/sgun1.wav");
    host.precache_sound(c"weapons/guncock.wav"); // player shotgun
    host.precache_sound(c"weapons/ric1.wav"); // ricochet (c code)
    host.precache_sound(c"weapons/ric2.wav"); // ricochet (c code)
    host.precache_sound(c"weapons/ric3.wav"); // ricochet (c code)
    host.precache_sound(c"weapons/spike2.wav"); // super spikes
    host.precache_sound(c"weapons/tink1.wav"); // spikes tink (c code)
    host.precache_sound(c"weapons/grenade.wav"); // grenade launcher
    host.precache_sound(c"weapons/bounce.wav"); // grenade bounce
    host.precache_sound(c"weapons/shotgn2.wav"); // super shotgun
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
