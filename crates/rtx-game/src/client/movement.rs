// SPDX-License-Identifier: AGPL-3.0-or-later

//! Player movement extras and powerups, split out of `client/mod.rs`: the rtx jump features
//! (elevator/wall/double jump over the engine's own jump), water physics (drowning, lava/slime
//! damage, enter/leave sounds), and the powerup lifecycle (invisibility/invulnerability/quad/biosuit
//! warn-and-expire). These are the per-frame physics the pre/post-think loop calls into.

use glam::Vec3;

use super::*;

impl GameState {
    /// `PlayerJump`.
    pub(super) fn player_jump(&mut self, e: EntId) {
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
        // us flagged on-ground or bounced us airborne. Failing that, the mid-air jumps: a wall jump
        // (kicking off a wall we're moving into) takes priority over the once-per-air-travel double
        // jump. If neither mid-air jump fires, bail without the jump-release/sound below (the ground
        // jump's own impulse is the engine's pmove job).
        if !self.try_elevator_jump(e) && !flags.has(Flags::ONGROUND) && !self.try_wall_jump(e) && !self.try_double_jump(e)
        {
            return;
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
        for (id, ent) in self.entities.live() {
            if id == e {
                continue;
            }
            let m = &ent.v;
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
        if mult == 0.0 || self.mode.stock_movement_only() {
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

        if !self.host.cvar_bool(c"rtx_walljump") || self.mode.stock_movement_only() {
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

    /// The rtx once-per-air-travel mid-air double jump: near the apex, restack a jump to clear a
    /// wider gap. Off in stock-movement modes (race maps aren't authored for it; bots'
    /// undershoot-recovery air jump flows through here too). Suppressed when about to land or just
    /// after takeoff — a nearby floor bails without consuming the air jump, so the intent carries to
    /// the landing and bunny hopping (jump-on-landing) is preserved. Returns whether it fired.
    fn try_double_jump(&mut self, e: EntId) -> bool {
        if self.entities[e].combat.air_jumped
            || !self.host.cvar_bool(c"rtx_doublejump")
            || self.mode.stock_movement_only()
        {
            return false;
        }
        const CLEARANCE: f32 = 24.0; // units of air required below the feet
        let mut feet = self.entities[e].v.origin;
        feet.z += self.entities[e].v.mins.z; // mins.z = -24 (VEC_HULL_MIN): origin -> feet
        let tr = self.traceline(feet, feet - Vec3::new(0.0, 0.0, CLEARANCE), true, e);
        if tr.fraction < 1.0 {
            return false; // floor close: the landing ground jump happens instead
        }
        self.entities[e].combat.air_jumped = true;
        self.entities[e].v.velocity.z = 270.0;
        true
    }

    /// `WaterMove` — drowning and lava/slime damage and enter/leave sounds.
    pub(super) fn water_move(&mut self, e: EntId) {
        let (movetype, health, waterlevel) = {
            let ent = &self.entities[e];
            (ent.v.movetype, ent.v.health, ent.v.waterlevel)
        };
        if movetype == MoveType::Noclip || health < 0.0 {
            return;
        }
        self.update_air_supply(e);
        if waterlevel == 0.0 {
            // Just left the water: play the out-of-water splash and clear the flag.
            if self.entities[e].v.flags.has(Flags::INWATER) {
                self.host
                    .sound(e, Channel::Body, Sound::MISC_OUTWATER, 1.0, Attenuation::Norm);
                let ent = &mut self.entities[e];
                ent.v.flags = ent.v.flags.without(Flags::INWATER);
            }
            return;
        }
        self.apply_liquid_damage(e);
        self.update_water_sounds(e);
    }

    /// Air/drowning accounting: out of (or only partly in) the water refreshes the air timer and
    /// gasps as it runs low; fully submerged past the timer drowns, crediting the liquid.
    fn update_air_supply(&mut self, e: EntId) {
        let time = self.time();
        let (waterlevel, watertype, air_finished) = {
            let ent = &self.entities[e];
            (ent.v.waterlevel, ent.v.watertype, ent.combat.air_finished)
        };
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
            // You can drown in lava or slime too — credit the liquid (KTX client.c:2795).
            self.entities[e].deathtype = if watertype.is(Content::Lava) {
                DeathType::Lava
            } else if watertype.is(Content::Slime) {
                DeathType::Slime
            } else {
                DeathType::Water
            };
            self.t_damage(e, EntId::WORLD, EntId::WORLD, dmg);
        }
    }

    /// Lava / slime contact damage while standing in it (throttled by `dmgtime`; the biosuit exempts
    /// slime and slows lava).
    fn apply_liquid_damage(&mut self, e: EntId) {
        let time = self.time();
        let (waterlevel, watertype, dmgtime, radsuit) = {
            let ent = &self.entities[e];
            (ent.v.waterlevel, ent.v.watertype, ent.combat.dmgtime, ent.combat.radsuit_finished)
        };
        if watertype.is(Content::Lava) && dmgtime < time {
            self.entities[e].combat.dmgtime = if radsuit > time { time + 1.0 } else { time + 0.2 };
            self.entities[e].deathtype = DeathType::Lava;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 10.0 * waterlevel);
        } else if watertype.is(Content::Slime) && dmgtime < time && radsuit < time {
            self.entities[e].combat.dmgtime = time + 1.0;
            self.entities[e].deathtype = DeathType::Slime;
            self.t_damage(e, EntId::WORLD, EntId::WORLD, 4.0 * waterlevel);
        }
    }

    /// The enter-water splash: on first entering, play the per-liquid sound, set the `INWATER` flag,
    /// and reset the contact-damage clock.
    fn update_water_sounds(&mut self, e: EntId) {
        if self.entities[e].v.flags.has(Flags::INWATER) {
            return;
        }
        let watertype = self.entities[e].v.watertype;
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

    /// `CheckPowerups` — expire powerups, flash warnings, and drive their lighting effects.
    pub(super) fn check_powerups(&mut self, e: EntId) {
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
                self.powerup_expire(e, PowerupKind::Invisibility, Items::INVISIBILITY);
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
                self.powerup_expire(e, PowerupKind::Invulnerability, Items::INVULNERABILITY);
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
                self.powerup_expire(e, PowerupKind::Quad, Items::QUAD);
                // dm4 (OctaPower): expiry hands back a fresh cells/armor/health kit.
                if self.level.deathmatch == 4 {
                    let ent = &mut self.entities[e];
                    ent.v.ammo_cells = 255.0;
                    ent.v.armorvalue = 1.0;
                    ent.v.armortype = 0.8;
                    ent.v.health = 100.0;
                }
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
                self.powerup_expire(e, PowerupKind::Biosuit, Items::SUIT);
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
            self.sprint_to(e, msg);
            self.screen_flash(e);
            self.host.sound(e, Channel::Auto, sound, 1.0, Attenuation::Norm);
            self.set_powerup_time(e, kind, time + 1.0);
        } else if latch < time {
            self.set_powerup_time(e, kind, time + 1.0);
            self.screen_flash(e);
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

    /// Expire a powerup: strip its item bit and zero both of its `*_finished`/`*_time` latches.
    /// Per-powerup expiry extras (Quad's dm4 kit refill) stay at the call site — they touch
    /// disjoint fields, so they compose in either order.
    fn powerup_expire(&mut self, e: EntId, kind: PowerupKind, item: Items) {
        let ent = &mut self.entities[e];
        ent.v.items = ent.v.items.without(item);
        match kind {
            PowerupKind::Invisibility => {
                ent.combat.invisible_finished = 0.0;
                ent.combat.invisible_time = 0.0;
            }
            PowerupKind::Invulnerability => {
                ent.combat.invincible_finished = 0.0;
                ent.combat.invincible_time = 0.0;
            }
            PowerupKind::Quad => {
                ent.combat.super_damage_finished = 0.0;
                ent.combat.super_time = 0.0;
            }
            PowerupKind::Biosuit => {
                ent.combat.radsuit_finished = 0.0;
                ent.combat.rad_time = 0.0;
            }
        }
    }

    /// Toggle a powerup's dim-light + colour glow effect bits.
    fn set_powerup_glow(&mut self, e: EntId, on: bool, color: Effects) {
        let glow = Effects::DIMLIGHT | color;
        let fx = self.entities[e].v.effects;
        self.entities[e].v.effects = if on { fx.with(glow) } else { fx.without(glow) };
    }
}
