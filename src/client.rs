// SPDX-License-Identifier: AGPL-3.0-or-later

//! Player lifecycle, ported from `qw-qc/client.qc`: connect/disconnect, spawn parameters,
//! spawn-point selection, and `PutClientInServer`. Movement itself is the engine's job
//! (QuakeWorld player physics) once the player entity is set up; combat, water, weapon
//! frames and powerups arrive in later milestones.

use core::ffi::CStr;

use glam::Vec3;

use crate::abi::EntVars;
use crate::assets::{Model, Sound};
use crate::bot;
use crate::defs::*;
use crate::entity::{CombatState, Die, EntId, Pain};
use crate::game::GameState;
use crate::mode::ArenaPlayer;

#[derive(Clone, Copy)]
struct PlayerParms {
    items: f32,
    health: f32,
    armorvalue: f32,
    shells: f32,
    nails: f32,
    rockets: f32,
    cells: f32,
    weapon: f32,
    armortype: f32,
}

impl PlayerParms {
    fn fresh() -> Self {
        Self {
            items: (Items::SHOTGUN | Items::AXE).as_f32(),
            health: 100.0,
            armorvalue: 0.0,
            shells: 25.0,
            nails: 0.0,
            rockets: 0.0,
            cells: 0.0,
            weapon: Items::SHOTGUN.as_f32(),
            armortype: 0.0,
        }
    }

    fn from_survivor(v: &EntVars) -> Self {
        Self {
            items: v.items.without(
                Items::KEY1 | Items::KEY2 | Items::INVISIBILITY | Items::INVULNERABILITY | Items::SUIT | Items::QUAD,
            ),
            health: v.health.clamp(50.0, 100.0),
            armorvalue: v.armorvalue,
            shells: v.ammo_shells.max(25.0),
            nails: v.ammo_nails,
            rockets: v.ammo_rockets,
            cells: v.ammo_cells,
            weapon: v.weapon.as_f32(),
            armortype: v.armortype,
        }
    }

    fn read(parm: [f32; 16]) -> Self {
        Self {
            items: parm[0],
            health: parm[1],
            armorvalue: parm[2],
            shells: parm[3],
            nails: parm[4],
            rockets: parm[5],
            cells: parm[6],
            weapon: parm[7],
            armortype: parm[8] * 0.01,
        }
    }

    fn write(self, parm: &mut [f32; 16]) {
        parm[0] = self.items;
        parm[1] = self.health;
        parm[2] = self.armorvalue;
        parm[3] = self.shells;
        parm[4] = self.nails;
        parm[5] = self.rockets;
        parm[6] = self.cells;
        parm[7] = self.weapon;
        parm[8] = self.armortype * 100.0;
    }

    fn apply_to(self, v: &mut EntVars) {
        v.items = self.items;
        v.health = self.health;
        v.armorvalue = self.armorvalue;
        v.ammo_shells = self.shells;
        v.ammo_nails = self.nails;
        v.ammo_rockets = self.rockets;
        v.ammo_cells = self.cells;
        v.weapon = Weapon::from_f32(self.weapon);
        v.armortype = self.armortype;
    }
}

#[derive(Clone, Copy)]
enum PowerupKind {
    Invisibility,
    Invulnerability,
    Quad,
    Biosuit,
}

impl GameState {
    /// `ClientConnect` — announce a joining player and record their name.
    pub(crate) fn client_connect(&mut self, player: EntId) {
        let name = self.read_netname(player);
        let ent = &mut self.entities[player];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.netname = Some(name.as_str().into());
        // Mirror the name into the engine-visible `v.netname` StringRef. The engine (FTEQW) syncs
        // the client name from it each frame; a bot's edict is cleared with an empty netname, so
        // without this it gets renamed to "" and disappears from the scoreboard.
        self.set_netname(player, &name);
        self.broadcast(PrintLevel::High, &format!("{name} entered the game\n"));
    }

    /// `ClientDisconnect` — announce a leaving player.
    pub(crate) fn client_disconnect(&mut self, player: EntId) {
        let ent = &self.entities[player];
        let name = ent.netname.as_deref().unwrap_or("");
        let frags = ent.v.frags as i32;
        let message = format!("{name} left the game with {frags} frags\n");
        self.broadcast(PrintLevel::High, &message);
        self.host
            .sound(player, Channel::Body, Sound::PLAYER_TORNOFF2, 1.0, Attenuation::None);
        // Drop a carried flag before `retire_slot` clears the carry marker (no-op outside CTF).
        self.drop_flag_if_carrying(player);
        self.retire_slot(player);
    }

    /// Retire a client slot from our private shadow state when it leaves — a human disconnecting,
    /// or a bot trimmed by the population manager. The engine frees the edict, but `in_use`,
    /// `classname`, the per-player arena role/queue, and bot bookkeeping are all *our* fields it
    /// never touches. Left stale, the departed player still counts as a live player and fighter:
    /// bots stop being trimmed (they think a human is still here), the Rocket Arena queue jams (a
    /// phantom fighter fills a slot), and a bot can even lock onto the freed edict and fire at its
    /// zeroed origin.
    pub(crate) fn retire_slot(&mut self, e: EntId) {
        bot::on_disconnect(&mut self.entities[e]); // resets bot state if it was a bot
        let ent = &mut self.entities[e];
        ent.in_use = false;
        ent.classname = None;
        ent.arena = ArenaPlayer::default();
    }

    /// `SetNewParms` — default spawn parameters for a fresh player.
    pub(crate) fn set_new_parms(&mut self) {
        PlayerParms::fresh().write(&mut self.globals.parm);
    }

    /// `SetChangeParms` — persist a surviving player's state across a level change.
    pub(crate) fn set_change_parms(&mut self, player: EntId) {
        let v = &self.entities[player].v;
        if v.health <= 0.0 {
            self.set_new_parms();
            return;
        }

        PlayerParms::from_survivor(v).write(&mut self.globals.parm);
    }

    /// `DecodeLevelParms` — load a player's fields from the spawn parameters.
    fn decode_level_parms(&mut self, player: EntId) {
        let parms = PlayerParms::read(self.globals.parm);
        let v = &mut self.entities[player].v;
        parms.apply_to(v);
    }

    /// `PutClientInServer` — set up (or respawn) the player entity at a spawn point.
    pub(crate) fn put_client_in_server(&mut self, player: EntId) {
        let time = self.globals.time;
        // The move-speed cap the engine seeds each client's pmove clamp from (via the declared
        // `maxspeed` field). Standard `sv_maxspeed` is 320; fall back to it if the cvar reads as
        // unset so a client is never accidentally pinned to a zero cap.
        let maxspeed = {
            let v = self.host.cvar(c"sv_maxspeed");
            if v > 0.0 {
                v
            } else {
                320.0
            }
        };
        {
            let ent = &mut self.entities[player];
            ent.in_use = true;
            ent.maxspeed = maxspeed;
            ent.classname = Some("player".into());
            ent.v.health = 100.0;
            ent.v.takedamage = TakeDamage::Aim.as_f32();
            ent.v.solid = Solid::SlideBox;
            ent.v.movetype = MoveType::Walk;
            ent.v.max_health = 100.0;
            ent.v.flags = Flags::CLIENT.as_f32();
            ent.v.effects = 0.0;
            ent.v.deadflag = DeadFlag::No.as_f32();
            // Respawn: a fresh player, so wipe all per-life combat state (powerup timers, pain
            // and attack cooldowns, the air-jump latch, …) and set only the two time-based timers.
            ent.combat = CombatState::default();
            ent.combat.air_finished = time + 12.0;
            ent.combat.attack_finished = time;
            ent.mover.dmg = 2.0; // initial water damage
            ent.mover.pausetime = 0.0;
            ent.th_pain = Pain::Player;
            ent.th_die = Die::Player;
        }

        self.decode_level_parms(player);
        // The grappling hook is handed out at spawn (also selectable via impulse 22 or a double-tap
        // of impulse 1), gated by a cvar like the other rtx movement features. It carries no ammo,
        // so it's just an extra item bit — and we spawn holding it by default.
        if self.host.cvar_bool(c"rtx_grapple") {
            let ent = &mut self.entities[player];
            ent.v.items = ent.v.items.with(Items::GRAPPLE);
            ent.v.weapon = Weapon::Grapple;
        }
        // The active mode has the final say on the loadout (e.g. Rocket Arena's fixed arsenal —
        // rocket launcher active, no grapple / an audience player's empty hands), so it runs after
        // the grapple handout and overrides it. FFA leaves the decoded parms + grapple as-is.
        let mode = self.mode;
        mode.apply_loadout(self, player);
        self.w_set_current_ammo(player);

        // The mode chooses the spawn point (arena vs. audience in Rocket Arena; a plain DM spawn
        // otherwise).
        let spot = mode.select_spawn(self, player);
        let origin = self.entities[spot].v.origin + Vec3::new(0.0, 0.0, 1.0);
        let angles = self.entities[spot].v.angles;
        {
            let ent = &mut self.entities[player];
            ent.v.origin = origin;
            ent.v.angles = angles;
            ent.v.fixangle = 1.0; // snap the client's view immediately
            ent.v.view_ofs = VEC_VIEW_OFS;
            ent.v.velocity = Vec3::ZERO;
        }

        // Assign the player model and bounding box, then explicitly relink at the spawn
        // origin via setorigin (the engine warns that a direct origin write does not fix
        // internal links — this is what makes the player visible/collidable to others).
        // The "eyes" model is set first purely to capture its modelindex (QuakeC's hack for
        // the Ring of Shadows), then the real player model.
        self.host.set_model(player, Model::PROGS_EYES);
        self.level.modelindex_eyes = self.entities[player].v.modelindex;
        self.host.set_model(player, Model::PROGS_PLAYER);
        self.level.modelindex_player = self.entities[player].v.modelindex;
        self.host.set_size(player, VEC_HULL_MIN, VEC_HULL_MAX);
        self.host.set_origin(player, origin);

        // Telefrag anyone already standing here, then kick off the idle animation loop.
        self.spawn_tdeath(origin, player);
        self.player_stand1(player);
    }

    /// `PlayerPreThink` — runs before engine physics: rules, water, death/respawn, jump.
    pub(crate) fn player_pre_think(&mut self, e: EntId) {
        if self.intermission_running {
            self.intermission_think();
            return;
        }
        if self.entities[e].v.view_ofs == Vec3::ZERO {
            return; // intermission or finale
        }
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        self.entities[e].deathtype = None;

        self.check_rules(e);
        self.water_move(e);

        let deadflag = self.entities[e].v.deadflag;
        if deadflag >= DeadFlag::Dead.as_f32() {
            self.player_death_think(e);
            return;
        }
        if deadflag.is(DeadFlag::Dying) {
            return;
        }

        if self.entities[e].v.flags.has(Flags::ONGROUND) {
            self.entities[e].combat.air_jumped = false; // rearm the mid-air jump for the next air travel
        }
        self.track_lift(e); // remember a rising lift for the elevator-jump grace window
        if self.entities[e].v.button2 != 0.0 {
            self.player_jump(e);
        } else {
            let ent = &mut self.entities[e];
            ent.v.flags = ent.v.flags.with(Flags::JUMPRELEASED);
        }

        // Reel in the grappling hook before the engine's player move consumes the velocity.
        if self.entities[e].grapple.on_hook {
            self.service_grapple(e);
        }

        // Teleporters can force a pause.
        if self.time() < self.entities[e].mover.pausetime {
            self.entities[e].v.velocity = Vec3::ZERO;
        }

        let v = &self.entities[e].v;
        if self.time() > self.entities[e].combat.attack_finished
            && v.currentammo == 0.0
            && v.weapon != Weapon::Axe
            && v.weapon != Weapon::Grapple
        // the grapple uses no ammo — don't auto-switch off it
        {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best;
            self.w_set_current_ammo(e);
        }
    }

    /// `PlayerPostThink` — runs after engine physics: landing damage, powerups, weapon loop.
    pub(crate) fn player_post_think(&mut self, e: EntId) {
        if self.entities[e].v.view_ofs == Vec3::ZERO || self.entities[e].v.deadflag != 0.0 {
            return;
        }

        // Landing sound / falling damage.
        let (jump_flag, on_ground, watertype) = {
            let ent = &self.entities[e];
            (ent.combat.jump_flag, ent.v.flags.has(Flags::ONGROUND), ent.v.watertype)
        };
        if jump_flag < -300.0 && on_ground {
            if watertype.is(Content::Water) {
                self.host
                    .sound(e, Channel::Body, Sound::PLAYER_H2OJUMP, 1.0, Attenuation::Norm);
            } else if jump_flag < -650.0 {
                self.entities[e].deathtype = Some("falling".into());
                self.t_damage(e, EntId::WORLD, EntId::WORLD, 5.0);
                self.host
                    .sound(e, Channel::Voice, Sound::PLAYER_LAND2, 1.0, Attenuation::Norm);
            } else {
                self.host
                    .sound(e, Channel::Voice, Sound::PLAYER_LAND, 1.0, Attenuation::Norm);
            }
        }
        self.entities[e].combat.jump_flag = self.entities[e].v.velocity.z;

        self.check_powerups(e);
        self.w_weapon_frame(e);

        // Bots: publish the full view angles through `v.angles`. When the engine networks a
        // player it sends their `lastcmd` view angles (what a tracking spectator's camera and
        // other clients' body-pitch rendering use) — but for bot clients FTE nulls `lastcmd`
        // (sv_ents.c `if (isbot) clst.lastcmd = NULL`) and falls back to `v.angles`, which
        // SV_RunCmd re-derives every tick as the model's -pitch/3 — so a spectated bot looked
        // nearly level while its rockets flew up/down. PostThink runs after that derivation and
        // before the frame is sent, so writing the real view here makes the fallback carry
        // exactly what a human's lastcmd would (remote clients re-derive body pitch themselves).
        if self.entities[e].bot.is_bot {
            let v = &mut self.entities[e].v;
            v.angles = Vec3::new(v.v_angle.x, v.v_angle.y, 0.0);
        }
    }

    /// `GAME_CLIENT_COMMAND` — handle a client console command; returns whether we consumed
    /// it (the engine runs its own handler otherwise).
    pub(crate) fn client_command(&mut self, e: EntId) -> isize {
        let mut buf = [0u8; 64];
        let cmd = self.host.cmd_argv(0, &mut buf).to_owned();
        match cmd.as_str() {
            "kill" => {
                self.client_kill(e);
                1
            }
            // Start a team match: lock the roster, reload the map, run the countdown (no-op unless a
            // team `rtx_mode` is active and we're in warmup). See `crate::mode::team`.
            "start" => {
                crate::mode::TeamMatch::start(self);
                1
            }
            _ => 0,
        }
    }

    /// `ClientKill` — the `kill` suicide command.
    fn client_kill(&mut self, e: EntId) {
        let name = self.netname_of(e);
        self.broadcast(PrintLevel::Medium, &format!("{name} suicides\n"));
        self.set_suicide_frame(e);
        self.entities[e].v.modelindex = self.level.modelindex_player;
        self.host.logfrag(e, e);
        self.entities[e].v.frags -= 2.0;
        self.respawn(e);
    }

    /// `respawn` — copy the corpse, reset parms, re-enter the server.
    pub(crate) fn respawn(&mut self, e: EntId) {
        self.copy_to_body_que(e);
        self.set_new_parms();
        self.put_client_in_server(e);
    }

    /// `CheckRules` — end the level on time/frag limit.
    fn check_rules(&mut self, e: EntId) {
        // Team/CTF matches own their limits (team score / capture limit, in the mode's `tick`) —
        // don't let an individual player's frags trip the stock intermission path.
        if crate::mode::is_match_mode(self.mode.name()) {
            return;
        }
        let frags = self.entities[e].v.frags;
        let tl = self.level.timelimit;
        let fl = self.level.fraglimit;
        if (tl != 0 && self.time() as i32 >= tl) || (fl != 0 && frags as i32 >= fl) {
            self.next_level();
        }
    }

    /// `PlayerDeathThink` — slow the corpse, then respawn on a button press.
    fn player_death_think(&mut self, e: EntId) {
        if self.entities[e].v.flags.has(Flags::ONGROUND) {
            let vel = self.entities[e].v.velocity;
            let forward = vel.length() - 20.0;
            self.entities[e].v.velocity = if forward <= 0.0 {
                Vec3::ZERO
            } else {
                forward * vel.normalize_or_zero()
            };
        }

        let (deadflag, b0, b1, b2) = {
            let v = &self.entities[e].v;
            (self.entities[e].v.deadflag, v.button0, v.button1, v.button2)
        };
        if deadflag.is(DeadFlag::Dead) {
            if b0 != 0.0 || b1 != 0.0 || b2 != 0.0 {
                return;
            }
            self.entities[e].v.deadflag = DeadFlag::Respawnable.as_f32();
            return;
        }
        if b0 == 0.0 && b1 == 0.0 && b2 == 0.0 {
            return;
        }
        {
            let v = &mut self.entities[e].v;
            v.button0 = 0.0;
            v.button1 = 0.0;
            v.button2 = 0.0;
        }
        // A mode may drive respawns itself (e.g. hold a dead player until the next round).
        let mode = self.mode;
        if !mode.allow_respawn(self, e) {
            return;
        }
        self.respawn(e);
    }

    /// `PlayerJump`.
    fn player_jump(&mut self, e: EntId) {
        let time = self.time();
        let (flags, waterlevel, swim_flag) = {
            let ent = &self.entities[e];
            (ent.v.flags, ent.v.waterlevel, ent.combat.swim_flag)
        };
        if flags.has(Flags::WATERJUMP) {
            return;
        }
        if waterlevel >= 2.0 {
            if swim_flag < time {
                self.entities[e].combat.swim_flag = time + 1.0;
                let s = if self.random() < 0.5 {
                    Sound::MISC_WATER1
                } else {
                    Sound::MISC_WATER2
                };
                self.host.sound(e, Channel::Body, s, 1.0, Attenuation::Norm);
            }
            return;
        }
        // Must release jump between jumps — stops auto-bouncing and holding into a double jump.
        if !flags.has(Flags::JUMPRELEASED) {
            return;
        }
        // An elevator jump (riding a rising lift) takes priority and works whether the lift left
        // us flagged on-ground or bounced us airborne. Failing that, the mid-air jumps.
        if !self.try_elevator_jump(e) && !flags.has(Flags::ONGROUND) {
            // Mid-air. A wall jump (kicking off a wall we're moving into) takes priority and is
            // limited only by geometry; failing that, the once-per-air-travel double jump. The
            // ground jump's impulse is applied by the engine's pmove, but nothing lifts us
            // mid-air, so both set velocity themselves.
            if !self.try_wall_jump(e) {
                if self.entities[e].combat.air_jumped || !self.host.cvar_bool(c"rtx_doublejump") {
                    return;
                }
                // Don't double-jump when about to land (or just after takeoff) — preserves bunny
                // hopping (jump-on-landing). Trace straight down from the feet; if the floor is
                // close, bail without consuming the air jump so the intent carries to the landing.
                const CLEARANCE: f32 = 24.0; // units of air required below the feet
                let mut feet = self.entities[e].v.origin;
                feet.z += self.entities[e].v.mins.z; // mins.z = -24 (VEC_HULL_MIN): origin -> feet
                let tr = self.traceline(feet, feet - Vec3::new(0.0, 0.0, CLEARANCE), true, e);
                if tr.fraction < 1.0 {
                    return; // floor close: the landing ground jump happens instead
                }
                self.entities[e].combat.air_jumped = true;
                self.entities[e].v.velocity.z = 270.0;
            }
        }
        {
            let v = &mut self.entities[e].v;
            v.flags = flags.without(Flags::JUMPRELEASED);
            v.button2 = 0.0;
        }
        self.host
            .sound(e, Channel::Body, Sound::PLAYER_PLYRJMP8, 1.0, Attenuation::Norm);
    }

    /// The upward speed of the rising `MoveType::Push` lift the player is standing on, or `0`.
    /// The engine's ground pick is unreliable for a pusher rider (it often reports the world), so
    /// we find it geometrically: the fastest-rising mover whose top is at/just below the feet and
    /// whose horizontal footprint (its half-width margin) contains the player.
    fn lift_under_player(&self, e: EntId) -> f32 {
        let (origin, mins) = {
            let v = &self.entities[e].v;
            (v.origin, v.mins)
        };
        let feet_z = origin.z + mins.z;
        let mut best = 0.0;
        for i in 1..self.entities.len() {
            let id = EntId(i as u32);
            if id == e || !self.entities[id].in_use {
                continue;
            }
            let m = &self.entities[id].v;
            if m.movetype != MoveType::Push || m.velocity.z <= best {
                continue;
            }
            let on_top = (-24.0..=48.0).contains(&(feet_z - m.absmax.z));
            let over = origin.x >= m.absmin.x - 16.0
                && origin.x <= m.absmax.x + 16.0
                && origin.y >= m.absmin.y - 16.0
                && origin.y <= m.absmax.y + 16.0;
            if on_top && over {
                best = m.velocity.z;
            }
        }
        best
    }

    /// Record the lift the player is riding each frame, so `elevator_boost` has a recent value to
    /// use even if the lift has just stopped at the moment of the jump.
    pub(crate) fn track_lift(&mut self, e: EntId) {
        let vz = self.lift_under_player(e);
        if vz > 0.0 {
            let now = self.time();
            let c = &mut self.entities[e].combat;
            c.lift_vz = vz;
            c.lift_time = now;
        }
    }

    /// `rtx_elevator_jump` — if the player jumped while (recently) riding a rising lift, launch
    /// with the whole jump applied ourselves: `lift·mult + 270`. That always exceeds the engine's
    /// `maxgroundspeed` (180), so pmove treats us as airborne and skips its own `+270` (no
    /// double-count, full boost regardless of lift speed). Deliberately decoupled from the
    /// on-ground flag and the exact jump frame — a pusher rider's on-ground state and feet
    /// alignment both flicker — using the lift speed `track_lift` remembers each frame plus a
    /// grace window. Consumes that memory so it gives one boost per ride. Returns whether it fired.
    fn try_elevator_jump(&mut self, e: EntId) -> bool {
        /// How long after last riding a rising lift a jump still gets the boost.
        const GRACE: f32 = 0.4;
        let mult = self.host.cvar(c"rtx_elevator_jump");
        if mult == 0.0 {
            return false;
        }
        let now = self.time();
        let (vz, when) = {
            let c = &self.entities[e].combat;
            (c.lift_vz, c.lift_time)
        };
        if vz <= 0.0 || now - when > GRACE {
            return false;
        }
        self.entities[e].combat.lift_time = 0.0; // consume — one boost per ride
        self.entities[e].v.velocity.z = vz * mult + 270.0;
        true
    }

    /// `rtx_walljump` — kick off a nearby wall mid-air. Because the engine clips a player's
    /// velocity *parallel* to a wall on contact, by the time you're at the wall you're sliding
    /// along it — so we probe ahead **and to both sides** to find it. The kick keeps the
    /// along-wall momentum, launches out along the wall normal (reflecting an into-wall approach,
    /// but always at least `KICK` so a parallel slide still pushes off), and jumps up — so you
    /// leave at an outward angle set by your slide speed vs. the kick. Returns whether it fired.
    fn try_wall_jump(&mut self, e: EntId) -> bool {
        /// Trace reach from the player's center — half-width (16) plus a margin.
        const REACH: f32 = 32.0;
        /// Minimum horizontal speed to orient the probes (and to count as a moving wall jump).
        const MIN_SPEED: f32 = 30.0;
        /// `|normal.z|` above this is a floor/ceiling, not a wall (Quake's walkable cutoff is 0.7).
        const MAX_FLOORNESS: f32 = 0.7;
        /// Minimum outward launch speed off the wall (a parallel slide gets at least this).
        const KICK: f32 = 270.0;
        /// Upward velocity imparted by the kick.
        const UP: f32 = 270.0;

        if !self.host.cvar_bool(c"rtx_walljump") {
            return false;
        }
        let (origin, vel) = {
            let v = &self.entities[e].v;
            (v.origin, v.velocity)
        };
        let horiz = Vec3::new(vel.x, vel.y, 0.0);
        let speed = horiz.length();
        if speed < MIN_SPEED {
            return false;
        }
        let fwd = horiz / speed;
        let side = Vec3::new(-fwd.y, fwd.x, 0.0); // horizontal perpendicular

        // Nearest near-vertical surface among forward / left / right.
        let mut wall = None;
        let mut best = 1.0;
        for d in [fwd, side, -side] {
            let tr = self.traceline(origin, origin + d * REACH, true, e);
            if tr.fraction < best && tr.plane_normal.z.abs() <= MAX_FLOORNESS {
                best = tr.fraction;
                wall = Some(tr.plane_normal);
            }
        }
        let Some(pn) = wall else {
            return false;
        };
        let n = Vec3::new(pn.x, pn.y, 0.0).normalize_or_zero();
        if n == Vec3::ZERO {
            return false;
        }

        // Keep along-wall momentum, launch out along the normal, jump up.
        let into = horiz.dot(n); // < 0 moving into the wall, ~0 sliding along it
        let tangential = horiz - into * n;
        let outward = (-into).max(KICK);
        let launch = tangential + outward * n;
        self.entities[e].v.velocity = Vec3::new(launch.x, launch.y, UP);
        true
    }

    /// `WaterMove` — drowning and lava/slime damage and enter/leave sounds.
    fn water_move(&mut self, e: EntId) {
        let time = self.time();
        let (movetype, health, waterlevel, watertype, air_finished) = {
            let ent = &self.entities[e];
            (
                ent.v.movetype,
                ent.v.health,
                ent.v.waterlevel,
                ent.v.watertype,
                ent.combat.air_finished,
            )
        };
        if movetype == MoveType::Noclip || health < 0.0 {
            return;
        }

        if waterlevel != 3.0 {
            if air_finished < time {
                self.host
                    .sound(e, Channel::Voice, Sound::PLAYER_GASP2, 1.0, Attenuation::Norm);
            } else if air_finished < time + 9.0 {
                self.host
                    .sound(e, Channel::Voice, Sound::PLAYER_GASP1, 1.0, Attenuation::Norm);
            }
            let ent = &mut self.entities[e];
            ent.combat.air_finished = time + 12.0;
            ent.mover.dmg = 2.0;
        } else if air_finished < time && self.entities[e].combat.pain_finished < time {
            let ent = &mut self.entities[e];
            ent.mover.dmg += 2.0;
            if ent.mover.dmg > 15.0 {
                ent.mover.dmg = 10.0;
            }
            let dmg = ent.mover.dmg;
            ent.combat.pain_finished = time + 1.0;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, dmg);
        }

        if waterlevel == 0.0 {
            if self.entities[e].v.flags.has(Flags::INWATER) {
                self.host
                    .sound(e, Channel::Body, Sound::MISC_OUTWATER, 1.0, Attenuation::Norm);
                let ent = &mut self.entities[e];
                ent.v.flags = ent.v.flags.without(Flags::INWATER);
            }
            return;
        }

        // Lava/slime contact damage.
        let (dmgtime, radsuit) = {
            let ent = &self.entities[e];
            (ent.combat.dmgtime, ent.combat.radsuit_finished)
        };
        if watertype.is(Content::Lava) && dmgtime < time {
            self.entities[e].combat.dmgtime = if radsuit > time { time + 1.0 } else { time + 0.2 };
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 10.0 * waterlevel);
        } else if watertype.is(Content::Slime) && dmgtime < time && radsuit < time {
            self.entities[e].combat.dmgtime = time + 1.0;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 4.0 * waterlevel);
        }

        if !self.entities[e].v.flags.has(Flags::INWATER) {
            let s = match watertype {
                w if w.is(Content::Lava) => Some(Sound::PLAYER_INLAVA),
                w if w.is(Content::Water) => Some(Sound::PLAYER_INH2O),
                w if w.is(Content::Slime) => Some(Sound::PLAYER_SLIMBRN2),
                _ => None,
            };
            if let Some(s) = s {
                self.host.sound(e, Channel::Body, s, 1.0, Attenuation::Norm);
            }
            let ent = &mut self.entities[e];
            ent.v.flags = ent.v.flags.with(Flags::INWATER);
            ent.combat.dmgtime = 0.0;
        }
    }

    /// `CheckPowerups` — expire powerups, flash warnings, and drive their lighting effects.
    fn check_powerups(&mut self, e: EntId) {
        let time = self.time();
        if self.entities[e].v.health <= 0.0 {
            return;
        }

        // Invisibility (Ring of Shadows) — swap to the eyes model.
        if self.entities[e].combat.invisible_finished != 0.0 {
            if self.entities[e].combat.invisible_sound < time {
                self.host
                    .sound(e, Channel::Auto, Sound::ITEMS_INV3, 0.5, Attenuation::Idle);
                let r = (self.random() * 3.0) + 1.0;
                self.entities[e].combat.invisible_sound = time + r;
            }
            if self.entities[e].combat.invisible_finished < time + 3.0 {
                self.powerup_warn(
                    e,
                    PowerupKind::Invisibility,
                    c"Ring of Shadows magic is fading\n",
                    Sound::ITEMS_INV2,
                );
            }
            if self.entities[e].combat.invisible_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::INVISIBILITY);
                ent.combat.invisible_finished = 0.0;
                ent.combat.invisible_time = 0.0;
            }
            let eyes = self.level.modelindex_eyes;
            let ent = &mut self.entities[e];
            ent.v.frame = 0.0;
            ent.v.modelindex = eyes;
        } else {
            self.entities[e].v.modelindex = self.level.modelindex_player;
        }

        // Invincibility (Pentagram) — red glow.
        if self.entities[e].combat.invincible_finished != 0.0 {
            if self.entities[e].combat.invincible_finished < time + 3.0 {
                self.powerup_warn(
                    e,
                    PowerupKind::Invulnerability,
                    c"Protection is almost burned out\n",
                    Sound::ITEMS_PROTECT2,
                );
            }
            if self.entities[e].combat.invincible_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::INVULNERABILITY);
                ent.combat.invincible_time = 0.0;
                ent.combat.invincible_finished = 0.0;
            }
            self.set_powerup_glow(e, self.entities[e].combat.invincible_finished > time, Effects::RED);
        }

        // Quad Damage — blue glow.
        if self.entities[e].combat.super_damage_finished != 0.0 {
            if self.entities[e].combat.super_damage_finished < time + 3.0 {
                let msg = if self.level.deathmatch == 4 {
                    c"OctaPower is wearing off\n"
                } else {
                    c"Quad Damage is wearing off\n"
                };
                self.powerup_warn(e, PowerupKind::Quad, msg, Sound::ITEMS_DAMAGE2);
            }
            if self.entities[e].combat.super_damage_finished < time {
                let dm4 = self.level.deathmatch == 4;
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::QUAD);
                if dm4 {
                    ent.v.ammo_cells = 255.0;
                    ent.v.armorvalue = 1.0;
                    ent.v.armortype = 0.8;
                    ent.v.health = 100.0;
                }
                ent.combat.super_damage_finished = 0.0;
                ent.combat.super_time = 0.0;
            }
            self.set_powerup_glow(e, self.entities[e].combat.super_damage_finished > time, Effects::BLUE);
        }

        // Biosuit — refresh air, expire quietly.
        if self.entities[e].combat.radsuit_finished != 0.0 {
            self.entities[e].combat.air_finished = time + 12.0;
            if self.entities[e].combat.radsuit_finished < time + 3.0 {
                self.powerup_warn(
                    e,
                    PowerupKind::Biosuit,
                    c"Air supply in Biosuit expiring\n",
                    Sound::ITEMS_SUIT2,
                );
            }
            if self.entities[e].combat.radsuit_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::SUIT);
                ent.combat.rad_time = 0.0;
                ent.combat.radsuit_finished = 0.0;
            }
        }
    }

    /// Shared "powerup almost out" flash/sound bookkeeping for [`Self::check_powerups`].
    /// `kind` selects the per-powerup `*_time` latch.
    fn powerup_warn(&mut self, e: EntId, kind: PowerupKind, msg: &CStr, sound: Sound) {
        let time = self.time();
        let latch = match kind {
            PowerupKind::Invisibility => self.entities[e].combat.invisible_time,
            PowerupKind::Invulnerability => self.entities[e].combat.invincible_time,
            PowerupKind::Quad => self.entities[e].combat.super_time,
            PowerupKind::Biosuit => self.entities[e].combat.rad_time,
        };
        if latch == 1.0 {
            self.host.sprint(e, PrintLevel::High, msg);
            self.host.stuffcmd(e, c"bf\n");
            self.host.sound(e, Channel::Auto, sound, 1.0, Attenuation::Norm);
            self.set_powerup_time(e, kind, time + 1.0);
        } else if latch < time {
            self.set_powerup_time(e, kind, time + 1.0);
            self.host.stuffcmd(e, c"bf\n");
        }
    }

    fn set_powerup_time(&mut self, e: EntId, kind: PowerupKind, t: f32) {
        let ent = &mut self.entities[e];
        match kind {
            PowerupKind::Invisibility => ent.combat.invisible_time = t,
            PowerupKind::Invulnerability => ent.combat.invincible_time = t,
            PowerupKind::Quad => ent.combat.super_time = t,
            PowerupKind::Biosuit => ent.combat.rad_time = t,
        }
    }

    /// Toggle a powerup's dim-light + colour glow effect bits.
    fn set_powerup_glow(&mut self, e: EntId, on: bool, color: Effects) {
        let glow = Effects::DIMLIGHT | color;
        let fx = self.entities[e].v.effects;
        self.entities[e].v.effects = if on { fx.with(glow) } else { fx.without(glow) };
    }

    /// `CopyToBodyQue` — leave a corpse copy behind on respawn. (Cosmetic body queue is
    /// deferred; the live entity is simply re-used.)
    fn copy_to_body_que(&mut self, _e: EntId) {}

    /// `W_SetCurrentAmmo` — sync `currentammo`/`weaponmodel`/ammo item bits to the active
    /// weapon, and network the first-person viewmodel.
    pub(crate) fn w_set_current_ammo(&mut self, player: EntId) {
        self.player_run(player); // get out of any weapon-firing animation state

        let (ammo, model, ammo_bit) = self.current_weapon_ammo_state(player);

        {
            let ent = &mut self.entities[player];
            ent.v.items = ent
                .v
                .items
                .without(Items::SHELLS | Items::NAILS | Items::ROCKETS | Items::CELLS)
                .with(ammo_bit);
            ent.v.currentammo = ammo;
            ent.v.weaponframe = 0.0;
            ent.weaponmodel = model.and_then(|m| m.path().to_str().ok()).map(Into::into);
        }
        self.set_weaponmodel(player, model);
    }

    /// `SelectSpawnPoint` — pick a deathmatch spawn (preferring unoccupied ones), falling
    /// back to the single-player start.
    /// Standard deathmatch spawn: a free `info_player_deathmatch`, falling back to
    /// `info_player_start` when a map has none.
    pub(crate) fn select_spawn_point(&mut self) -> EntId {
        let spot = self.pick_spawn_of("info_player_deathmatch");
        if spot != EntId::WORLD {
            return spot;
        }
        self.find_by_classname("info_player_start")
            .next()
            .unwrap_or(EntId::WORLD)
    }

    /// Pick a spawn point of a specific classname (e.g. `info_teleport_destination` for arena
    /// fighters), preferring an unoccupied one. `EntId::WORLD` if the map has none — the caller
    /// decides the fallback.
    pub(crate) fn select_spawn_point_of(&mut self, classname: &str) -> EntId {
        self.pick_spawn_of(classname)
    }

    /// Shared spawn-point picker: a random unoccupied entity of `classname` (any of them if all
    /// are occupied), or `EntId::WORLD` if none exist.
    fn pick_spawn_of(&mut self, classname: &str) -> EntId {
        let spots: Vec<EntId> = self.find_by_classname(classname).collect();
        if spots.is_empty() {
            return EntId::WORLD;
        }
        let free: Vec<EntId> = spots.iter().copied().filter(|&s| !self.spot_occupied(s)).collect();
        let pool = if free.is_empty() { &spots } else { &free };

        let pick = (self.random() * pool.len() as f32) as usize;
        pool[pick.min(pool.len() - 1)]
    }

    /// Whether any player stands within 84 units of a spawn point.
    fn spot_occupied(&self, spot: EntId) -> bool {
        let origin = self.entities[spot].v.origin;
        self.find_by_classname("player")
            .any(|p| (self.entities[p].v.origin - origin).length() < 84.0)
    }

    // --- small helpers ---

    /// Read the player's `name` userinfo key.
    pub(crate) fn read_netname(&self, player: EntId) -> String {
        let mut buf = [0u8; 64];
        self.host.infokey(player, c"name", &mut buf).to_owned()
    }
}
