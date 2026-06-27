//! Player lifecycle, ported from `qw-qc/client.qc`: connect/disconnect, spawn parameters,
//! spawn-point selection, and `PutClientInServer`. Movement itself is the engine's job
//! (QuakeWorld player physics) once the player entity is set up; combat, water, weapon
//! frames and powerups arrive in later milestones.

use core::ffi::CStr;

use glam::Vec3;

use crate::defs::*;
use crate::entity::{Die, EntId, Pain};
use crate::game::GameState;

impl GameState {
    /// `ClientConnect` — announce a joining player and record their name.
    pub(crate) fn client_connect(&mut self, player: EntId) {
        let name = self.read_netname(player);
        let ent = &mut self.entities[player];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.netname = Some(name.as_str().into());
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
            .sound(player.0 as i32, Channel::Body, c"player/tornoff2.wav", 1.0, Attenuation::None);
    }

    /// `SetNewParms` — default spawn parameters for a fresh player.
    pub(crate) fn set_new_parms(&mut self) {
        let p = &mut self.globals.parm;
        p[0] = (Items::SHOTGUN | Items::AXE).as_f32(); // items
        p[1] = 100.0; // health
        p[2] = 0.0; // armor value
        p[3] = 25.0; // shells
        p[4] = 0.0; // nails
        p[5] = 0.0; // rockets
        p[6] = 0.0; // cells
        p[7] = 1.0; // weapon = Items::SHOTGUN.as_f32()
        p[8] = 0.0; // armor type
    }

    /// `SetChangeParms` — persist a surviving player's state across a level change.
    pub(crate) fn set_change_parms(&mut self, player: EntId) {
        let v = &self.entities[player].v;
        if v.health <= 0.0 {
            self.set_new_parms();
            return;
        }

        let items = v.items
            .without(Items::KEY1 | Items::KEY2 | Items::INVISIBILITY | Items::INVULNERABILITY | Items::SUIT | Items::QUAD);
        let health = v.health.clamp(50.0, 100.0);
        let armorvalue = v.armorvalue;
        let shells = v.ammo_shells.max(25.0);
        let nails = v.ammo_nails;
        let rockets = v.ammo_rockets;
        let cells = v.ammo_cells;
        let weapon = v.weapon;
        let armortype = v.armortype;

        let p = &mut self.globals.parm;
        p[0] = items;
        p[1] = health;
        p[2] = armorvalue;
        p[3] = shells;
        p[4] = nails;
        p[5] = rockets;
        p[6] = cells;
        p[7] = weapon;
        p[8] = armortype * 100.0;
    }

    /// `DecodeLevelParms` — load a player's fields from the spawn parameters.
    fn decode_level_parms(&mut self, player: EntId) {
        let p = self.globals.parm;
        let v = &mut self.entities[player].v;
        v.items = p[0];
        v.health = p[1];
        v.armorvalue = p[2];
        v.ammo_shells = p[3];
        v.ammo_nails = p[4];
        v.ammo_rockets = p[5];
        v.ammo_cells = p[6];
        v.weapon = p[7];
        v.armortype = p[8] * 0.01;
    }

    /// `PutClientInServer` — set up (or respawn) the player entity at a spawn point.
    pub(crate) fn put_client_in_server(&mut self, player: EntId) {
        let time = self.globals.time;
        {
            let ent = &mut self.entities[player];
            ent.in_use = true;
            ent.classname = Some("player".into());
            ent.v.health = 100.0;
            ent.v.takedamage = TakeDamage::Aim.as_f32();
            ent.v.solid = Solid::SlideBox.as_f32();
            ent.v.movetype = MoveType::Walk.as_f32();
            ent.v.max_health = 100.0;
            ent.v.flags = Flags::CLIENT.as_f32();
            ent.v.effects = 0.0;
            ent.v.deadflag = DeadFlag::No.as_f32();
            ent.show_hostile = 0.0;
            ent.air_finished = time + 12.0;
            ent.dmg = 2.0; // initial water damage
            ent.super_damage_finished = 0.0;
            ent.radsuit_finished = 0.0;
            ent.invisible_finished = 0.0;
            ent.invincible_finished = 0.0;
            ent.invincible_time = 0.0;
            ent.pausetime = 0.0;
            ent.attack_finished = time;
            ent.th_pain = Pain::Player;
            ent.th_die = Die::Player;
        }

        self.decode_level_parms(player);
        self.w_set_current_ammo(player);



        let spot = self.select_spawn_point();
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
        self.host.set_model(player.0 as i32, c"progs/eyes.mdl");
        self.level.modelindex_eyes = self.entities[player].v.modelindex;
        self.host.set_model(player.0 as i32, c"progs/player.mdl");
        self.level.modelindex_player = self.entities[player].v.modelindex;
        self.host
            .set_size(player.0 as i32, VEC_HULL_MIN, VEC_HULL_MAX);
        self.host.set_origin(player.0 as i32, origin);

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

        if self.entities[e].v.button2 != 0.0 {
            self.player_jump(e);
        } else {
            let ent = &mut self.entities[e];
            ent.v.flags = ent.v.flags.with(Flags::JUMPRELEASED);
        }

        // Teleporters can force a pause.
        if self.time() < self.entities[e].pausetime {
            self.entities[e].v.velocity = Vec3::ZERO;
        }

        let v = &self.entities[e].v;
        if self.time() > self.entities[e].attack_finished
            && v.currentammo == 0.0
            && !v.weapon.is(Items::AXE)
        {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best.as_f32();
            self.w_set_current_ammo(e);
        }
    }

    /// `PlayerPostThink` — runs after engine physics: landing damage, powerups, weapon loop.
    pub(crate) fn player_post_think(&mut self, e: EntId) {
        if self.entities[e].v.view_ofs == Vec3::ZERO || self.entities[e].v.deadflag != 0.0
        {
            return;
        }

        // Landing sound / falling damage.
        let (jump_flag, on_ground, watertype) = {
            let ent = &self.entities[e];
            (
                ent.jump_flag,
                ent.v.flags.has(Flags::ONGROUND),
                ent.v.watertype,
            )
        };
        if jump_flag < -300.0 && on_ground {
            if watertype.is(Content::Water) {
                self.host
                    .sound(e.0 as i32, Channel::Body, c"player/h2ojump.wav", 1.0, Attenuation::Norm);
            } else if jump_flag < -650.0 {
                self.entities[e].deathtype = Some("falling".into());
                self.t_damage(e, EntId::WORLD, EntId::WORLD, 5.0);
                self.host
                    .sound(e.0 as i32, Channel::Voice, c"player/land2.wav", 1.0, Attenuation::Norm);
            } else {
                self.host
                    .sound(e.0 as i32, Channel::Voice, c"player/land.wav", 1.0, Attenuation::Norm);
            }
        }
        self.entities[e].jump_flag = self.entities[e].v.velocity.z;

        self.check_powerups(e);
        self.w_weapon_frame(e);
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
            _ => 0,
        }
    }

    /// `ClientKill` — the `kill` suicide command.
    fn client_kill(&mut self, e: EntId) {
        let name = self.netname_of(e);
        self.broadcast(PrintLevel::Medium, &format!("{name} suicides\n"));
        self.set_suicide_frame(e);
        self.entities[e].v.modelindex = self.level.modelindex_player;
        self.host.logfrag(e.0 as i32, e.0 as i32);
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
            (
                self.entities[e].v.deadflag,
                v.button0,
                v.button1,
                v.button2,
            )
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
        self.respawn(e);
    }

    /// `PlayerJump`.
    fn player_jump(&mut self, e: EntId) {
        let time = self.time();
        let (flags, waterlevel, swim_flag) = {
            let ent = &self.entities[e];
            (ent.v.flags, ent.v.waterlevel, ent.swim_flag)
        };
        if flags.has(Flags::WATERJUMP) {
            return;
        }
        if waterlevel >= 2.0 {
            if swim_flag < time {
                self.entities[e].swim_flag = time + 1.0;
                let s = if self.random() < 0.5 { c"misc/water1.wav" } else { c"misc/water2.wav" };
                self.host.sound(e.0 as i32, Channel::Body, s, 1.0, Attenuation::Norm);
            }
            return;
        }
        if !flags.has(Flags::ONGROUND) || !flags.has(Flags::JUMPRELEASED) {
            return;
        }
        {
            let v = &mut self.entities[e].v;
            v.flags = flags.without(Flags::JUMPRELEASED);
            v.button2 = 0.0;
        }
        self.host
            .sound(e.0 as i32, Channel::Body, c"player/plyrjmp8.wav", 1.0, Attenuation::Norm);
    }

    /// `WaterMove` — drowning and lava/slime damage and enter/leave sounds.
    fn water_move(&mut self, e: EntId) {
        let time = self.time();
        let (movetype, health, waterlevel, watertype, air_finished) = {
            let ent = &self.entities[e];
            (ent.v.movetype, ent.v.health, ent.v.waterlevel, ent.v.watertype, ent.air_finished)
        };
        if movetype.is(MoveType::Noclip) || health < 0.0 {
            return;
        }

        if waterlevel != 3.0 {
            if air_finished < time {
                self.host
                    .sound(e.0 as i32, Channel::Voice, c"player/gasp2.wav", 1.0, Attenuation::Norm);
            } else if air_finished < time + 9.0 {
                self.host
                    .sound(e.0 as i32, Channel::Voice, c"player/gasp1.wav", 1.0, Attenuation::Norm);
            }
            let ent = &mut self.entities[e];
            ent.air_finished = time + 12.0;
            ent.dmg = 2.0;
        } else if air_finished < time && self.entities[e].pain_finished < time {
            let ent = &mut self.entities[e];
            ent.dmg += 2.0;
            if ent.dmg > 15.0 {
                ent.dmg = 10.0;
            }
            let dmg = ent.dmg;
            ent.pain_finished = time + 1.0;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, dmg);
        }

        if waterlevel == 0.0 {
            if self.entities[e].v.flags.has(Flags::INWATER) {
                self.host
                    .sound(e.0 as i32, Channel::Body, c"misc/outwater.wav", 1.0, Attenuation::Norm);
                let ent = &mut self.entities[e];
                ent.v.flags = ent.v.flags.without(Flags::INWATER);
            }
            return;
        }

        // Lava/slime contact damage.
        let (dmgtime, radsuit) = {
            let ent = &self.entities[e];
            (ent.dmgtime, ent.radsuit_finished)
        };
        if watertype.is(Content::Lava) && dmgtime < time {
            self.entities[e].dmgtime = if radsuit > time { time + 1.0 } else { time + 0.2 };
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 10.0 * waterlevel);
        } else if watertype.is(Content::Slime) && dmgtime < time && radsuit < time {
            self.entities[e].dmgtime = time + 1.0;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 4.0 * waterlevel);
        }

        if !self.entities[e].v.flags.has(Flags::INWATER) {
            let s = match watertype {
                w if w.is(Content::Lava) => Some(c"player/inlava.wav"),
                w if w.is(Content::Water) => Some(c"player/inh2o.wav"),
                w if w.is(Content::Slime) => Some(c"player/slimbrn2.wav"),
                _ => None,
            };
            if let Some(s) = s {
                self.host.sound(e.0 as i32, Channel::Body, s, 1.0, Attenuation::Norm);
            }
            let ent = &mut self.entities[e];
            ent.v.flags = ent.v.flags.with(Flags::INWATER);
            ent.dmgtime = 0.0;
        }
    }

    /// `CheckPowerups` — expire powerups, flash warnings, and drive their lighting effects.
    fn check_powerups(&mut self, e: EntId) {
        let time = self.time();
        if self.entities[e].v.health <= 0.0 {
            return;
        }

        // Invisibility (Ring of Shadows) — swap to the eyes model.
        if self.entities[e].invisible_finished != 0.0 {
            if self.entities[e].invisible_sound < time {
                self.host
                    .sound(e.0 as i32, Channel::Auto, c"items/inv3.wav", 0.5, Attenuation::Idle);
                let r = (self.random() * 3.0) + 1.0;
                self.entities[e].invisible_sound = time + r;
            }
            if self.entities[e].invisible_finished < time + 3.0 {
                self.powerup_warn(e, "invisible", c"Ring of Shadows magic is fading\n", c"items/inv2.wav");
            }
            if self.entities[e].invisible_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::INVISIBILITY);
                ent.invisible_finished = 0.0;
                ent.invisible_time = 0.0;
            }
            let eyes = self.level.modelindex_eyes;
            let ent = &mut self.entities[e];
            ent.v.frame = 0.0;
            ent.v.modelindex = eyes;
        } else {
            self.entities[e].v.modelindex = self.level.modelindex_player;
        }

        // Invincibility (Pentagram) — red glow.
        if self.entities[e].invincible_finished != 0.0 {
            if self.entities[e].invincible_finished < time + 3.0 {
                self.powerup_warn(e, "invincible", c"Protection is almost burned out\n", c"items/protect2.wav");
            }
            if self.entities[e].invincible_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::INVULNERABILITY);
                ent.invincible_time = 0.0;
                ent.invincible_finished = 0.0;
            }
            self.set_powerup_glow(e, self.entities[e].invincible_finished > time, Effects::RED);
        }

        // Quad Damage — blue glow.
        if self.entities[e].super_damage_finished != 0.0 {
            if self.entities[e].super_damage_finished < time + 3.0 {
                let msg = if self.level.deathmatch == 4 {
                    c"OctaPower is wearing off\n"
                } else {
                    c"Quad Damage is wearing off\n"
                };
                self.powerup_warn(e, "super", msg, c"items/damage2.wav");
            }
            if self.entities[e].super_damage_finished < time {
                let dm4 = self.level.deathmatch == 4;
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::QUAD);
                if dm4 {
                    ent.v.ammo_cells = 255.0;
                    ent.v.armorvalue = 1.0;
                    ent.v.armortype = 0.8;
                    ent.v.health = 100.0;
                }
                ent.super_damage_finished = 0.0;
                ent.super_time = 0.0;
            }
            self.set_powerup_glow(e, self.entities[e].super_damage_finished > time, Effects::BLUE);
        }

        // Biosuit — refresh air, expire quietly.
        if self.entities[e].radsuit_finished != 0.0 {
            self.entities[e].air_finished = time + 12.0;
            if self.entities[e].radsuit_finished < time + 3.0 {
                self.powerup_warn(e, "rad", c"Air supply in Biosuit expiring\n", c"items/suit2.wav");
            }
            if self.entities[e].radsuit_finished < time {
                let ent = &mut self.entities[e];
                ent.v.items = ent.v.items.without(Items::SUIT);
                ent.rad_time = 0.0;
                ent.radsuit_finished = 0.0;
            }
        }
    }

    /// Shared "powerup almost out" flash/sound bookkeeping for [`Self::check_powerups`].
    /// `which` selects the per-powerup `*_time` latch.
    fn powerup_warn(&mut self, e: EntId, which: &str, msg: &CStr, sound: &CStr) {
        let time = self.time();
        let latch = match which {
            "invisible" => self.entities[e].invisible_time,
            "invincible" => self.entities[e].invincible_time,
            "super" => self.entities[e].super_time,
            _ => self.entities[e].rad_time,
        };
        if latch == 1.0 {
            self.host.sprint(e.0 as i32, PrintLevel::High, msg);
            self.host.stuffcmd(e.0 as i32, c"bf\n");
            self.host.sound(e.0 as i32, Channel::Auto, sound, 1.0, Attenuation::Norm);
            self.set_powerup_time(e, which, time + 1.0);
        } else if latch < time {
            self.set_powerup_time(e, which, time + 1.0);
            self.host.stuffcmd(e.0 as i32, c"bf\n");
        }
    }

    fn set_powerup_time(&mut self, e: EntId, which: &str, t: f32) {
        let ent = &mut self.entities[e];
        match which {
            "invisible" => ent.invisible_time = t,
            "invincible" => ent.invincible_time = t,
            "super" => ent.super_time = t,
            _ => ent.rad_time = t,
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

        let (ammo, model, ammo_bit): (f32, Option<&'static CStr>, Items) = {
            let v = &self.entities[player].v;
            match Items::from_f32(v.weapon) {
                w if w == Items::AXE => (0.0, Some(c"progs/v_axe.mdl"), Items::empty()),
                w if w == Items::SHOTGUN => (v.ammo_shells, Some(c"progs/v_shot.mdl"), Items::SHELLS),
                w if w == Items::SUPER_SHOTGUN => {
                    (v.ammo_shells, Some(c"progs/v_shot2.mdl"), Items::SHELLS)
                }
                w if w == Items::NAILGUN => (v.ammo_nails, Some(c"progs/v_nail.mdl"), Items::NAILS),
                w if w == Items::SUPER_NAILGUN => {
                    (v.ammo_nails, Some(c"progs/v_nail2.mdl"), Items::NAILS)
                }
                w if w == Items::GRENADE_LAUNCHER => {
                    (v.ammo_rockets, Some(c"progs/v_rock.mdl"), Items::ROCKETS)
                }
                w if w == Items::ROCKET_LAUNCHER => {
                    (v.ammo_rockets, Some(c"progs/v_rock2.mdl"), Items::ROCKETS)
                }
                w if w == Items::LIGHTNING => (v.ammo_cells, Some(c"progs/v_light.mdl"), Items::CELLS),
                _ => (0.0, None, Items::empty()),
            }
        };

        {
            let ent = &mut self.entities[player];
            ent.v.items = ent
                .v
                .items
                .without(Items::SHELLS | Items::NAILS | Items::ROCKETS | Items::CELLS)
                .with(ammo_bit);
            ent.v.currentammo = ammo;
            ent.v.weaponframe = 0.0;
            ent.weaponmodel = model.and_then(|m| m.to_str().ok()).map(Into::into);
        }
        self.set_weaponmodel(player, model);
    }

    /// `SelectSpawnPoint` — pick a deathmatch spawn (preferring unoccupied ones), falling
    /// back to the single-player start.
    fn select_spawn_point(&mut self) -> EntId {
        let spots: Vec<EntId> = self.find_by_classname("info_player_deathmatch").collect();
        if spots.is_empty() {
            return self
                .find_by_classname("info_player_start")
                .next()
                .unwrap_or(EntId::WORLD);
        }

        let free: Vec<EntId> = spots
            .iter()
            .copied()
            .filter(|&s| !self.spot_occupied(s))
            .collect();
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
        self.host
            .infokey(player.0 as i32, c"name", &mut buf)
            .to_owned()
    }

}
