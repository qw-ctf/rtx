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
use crate::entity::{CombatState, Die, EntId, Pain, SpawnState};
use crate::game::GameState;
use crate::mode::ModePlayer;
use crate::obituary::DeathType;

pub(crate) mod movement;
mod spawn_select;

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
        // Let the mode react before `retire_slot` clears its per-player state (CTF drops a carried
        // flag here).
        let mode = self.mode;
        mode.on_client_disconnect(self, player);
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
        ent.mode_p = ModePlayer::default();
        ent.spawn = SpawnState::default();
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

    /// Strip weapons disabled by `rtx_weapons` from a just-equipped player, re-picking the held
    /// weapon if it was one of them. Only weapon bits are touched; ammo/armor/powerups are left
    /// alone. Shared by every loadout path — the spawn in [`Self::put_client_in_server`] and Rocket
    /// Arena's winner-stays re-equip — so a disabled weapon can't slip through any of them.
    pub(crate) fn filter_disabled_weapons(&mut self, e: EntId) {
        let disabled = crate::arsenal::all_weapon_bits().difference(self.enabled_weapon_mask());
        self.entities[e].v.items = self.entities[e].v.items.without(disabled);
        let held = self.entities[e].v.weapon;
        if held != Weapon::None && !self.entities[e].v.items.has(held.item()) {
            // W_BestWeapon never auto-selects the explosives, so a player left with only a GL or RL
            // would otherwise fall back to the axe. Prefer a fireable RL, then GL, before that.
            let v = &self.entities[e].v;
            let best = if v.items.has(Items::ROCKET_LAUNCHER) && v.ammo_rockets >= 1.0 {
                Weapon::RocketLauncher
            } else if v.items.has(Items::GRENADE_LAUNCHER) && v.ammo_rockets >= 1.0 {
                Weapon::GrenadeLauncher
            } else {
                self.w_best_weapon(e)
            };
            self.entities[e].v.weapon = best;
        }
    }

    /// `PutClientInServer` — set up (or respawn) the player entity at a spawn point.
    pub(crate) fn put_client_in_server(&mut self, player: EntId) {
        self.configure_fresh_player_body(player);
        // Granting the loadout is also where bench-vs-active is decided; the spawn placement reuses
        // that verdict (benched spectators go to the stands, not the mode's spawn logic).
        let benched = self.grant_spawn_loadout(player);
        self.place_at_spawn(player, benched);
    }

    /// Reset `player`'s edict to a fresh, living body: solid slidebox, walk movetype, full health,
    /// the client flag, a wiped per-life `CombatState`, and the player think/pain/die callbacks.
    fn configure_fresh_player_body(&mut self, player: EntId) {
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
        let ent = &mut self.entities[player];
        ent.in_use = true;
        ent.maxspeed = maxspeed;
        ent.classname = Some("player".into());
        ent.v.health = 100.0;
        ent.v.takedamage = TakeDamage::Aim;
        ent.v.solid = Solid::SlideBox;
        ent.v.movetype = MoveType::Walk;
        ent.v.max_health = 100.0;
        ent.v.flags = Flags::CLIENT.as_f32();
        ent.v.effects = 0.0;
        ent.v.deadflag = DeadFlag::No;
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

    /// Grant `player` their spawn kit and return whether they were benched. Decoded level parms,
    /// then the optional grapple grant, then either the audience loadout (a benched late joiner:
    /// axe only, damage refused) or team assignment plus the mode's kit. Disabled weapons are
    /// stripped from whatever was granted, current ammo is set, and every side's opponent-model
    /// hypothesis of this player resets to the spawn baseline.
    fn grant_spawn_loadout(&mut self, player: EntId) -> bool {
        self.decode_level_parms(player);
        // The grappling hook is handed out at spawn (also selectable via impulse 22 or a double-tap
        // of impulse 1), gated by a cvar like the other rtx movement features. It carries no ammo,
        // so it's just an extra item bit — and we spawn holding it by default.
        if self.host.cvar_bool(c"rtx_grapple") {
            let ent = &mut self.entities[player];
            ent.v.items = ent.v.items.with(Items::GRAPPLE);
            ent.v.weapon = Weapon::Grapple;
        }
        // In a structured match, a late joiner not on the locked roster sits out as a harmless
        // spectator (axe only, damage refused) until the next warmup. Everyone else gets their team
        // assignment first (in any team composition), then the mode's loadout on top — so the mode's
        // fixed kit (Rocket Arena's arsenal, Midair's RL) composes with teams. Deathmatch leaves the
        // decoded parms + grapple as-is.
        let mode = self.mode;
        let benched = crate::mode::team::benched(self, player);
        if benched {
            crate::mode::audience_loadout(self, player);
        } else {
            if crate::mode::team::lifecycle_active(self) {
                crate::mode::team::assign_team(self, player);
            }
            mode.apply_loadout(self, player);
        }
        // rtx_weapons: strip any disabled weapon from the granted kit (decoded parms, the grapple
        // grant, and the mode loadout, all applied above).
        self.filter_disabled_weapons(player);
        self.w_set_current_ammo(player);

        // Opponent modeling: a (re)spawn hands out a fresh loadout, so reset every side's hypothesis
        // of this player to the mode's spawn-kit baseline. Covers connect and mode kit re-grants
        // without relying on the death path. No-op when modeling is off.
        self.model_reset_target(player);
        benched
    }

    /// Choose `player`'s spawn point (mode-driven; benched spectators use plain DM spawns), nudge
    /// the origin off any other live player, snap the view there, then relink via `set_origin` (a
    /// raw origin write doesn't fix the engine's internal links). Finishes by telefragging anyone
    /// already standing there, starting the idle animation, and telling a benched human why they're
    /// a spectator.
    fn place_at_spawn(&mut self, player: EntId, benched: bool) {
        let time = self.globals.time;
        let mode = self.mode;
        // The mode chooses the spawn point (arena vs. audience in Rocket Arena; a plain DM spawn
        // otherwise). Benched spectators go to the stands (plain DM spawns), not the mode's logic.
        let spot = if benched {
            self.select_spawn_point(Some(player))
        } else {
            mode.select_spawn(self, player)
        };
        // Let the mode nudge the origin off any other live player (Rocket Arena) so no two players
        // are relinked onto the same point — the last funnel every spawn path passes through.
        let origin = self.entities[spot].v.origin + Vec3::new(0.0, 0.0, 1.0);
        let origin = mode.adjust_spawn_origin(self, player, origin);
        let angles = self.entities[spot].v.angles;
        {
            let ent = &mut self.entities[player];
            ent.v.origin = origin;
            ent.v.angles = angles;
            ent.v.fixangle = 1.0; // snap the client's view immediately
            ent.v.view_ofs = VEC_VIEW_OFS;
            ent.v.velocity = Vec3::ZERO;
            // Freshly spawned players fence nearby spawn spots for a moment (KTX's k_1spawn),
            // so two respawns in quick succession don't land on adjacent spots.
            ent.spawn.grace_until = time + 2.6;
        }

        // Assign the player model and bounding box, then explicitly relink at the spawn
        // origin via setorigin (the engine warns that a direct origin write does not fix
        // internal links — this is what makes the player visible/collidable to others).
        // The "eyes" model is set first purely to capture its modelindex (QuakeC's hack for
        // the Ring of Shadows), then the real player model.
        self.set_model(player, Model::PROGS_EYES);
        self.level.modelindex_eyes = self.entities[player].v.modelindex;
        self.set_model(player, Model::PROGS_PLAYER);
        self.level.modelindex_player = self.entities[player].v.modelindex;
        self.set_size(player, VEC_HULL_MIN, VEC_HULL_MAX);
        self.set_origin(player, origin);

        // Telefrag anyone already standing here, then kick off the idle animation loop.
        self.spawn_tdeath(origin, player);
        self.player_stand1(player);

        // Tell a benched human why they're spectating (bots have no connection to print to).
        if benched && !self.entities[player].bot.is_bot {
            self.sprint_to(player, c"Match in progress — you'll join at the next warmup.\n");
        }
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
        self.make_vectors(v_angle);
        self.entities[e].deathtype = DeathType::None;

        self.check_rules(e);
        self.water_move(e);
        let mode = self.mode;
        mode.player_prethink(self, e); // CTF Regeneration rune tick

        let deadflag = self.entities[e].v.deadflag;
        if deadflag >= DeadFlag::Dead {
            self.player_death_think(e);
            return;
        }
        if deadflag == DeadFlag::Dying {
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
        if self.entities[e].v.view_ofs == Vec3::ZERO || self.entities[e].v.deadflag != DeadFlag::No {
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
                self.entities[e].deathtype = DeathType::Fall;
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
            // The match-lifecycle "start" command begins a team match (a no-op in open/non-team
            // play). Consume the token regardless so a stray "start" isn't handed to the engine.
            "start" => {
                crate::mode::team::start_match(self);
                1
            }
            _ => 0,
        }
    }

    /// `ClientKill` — the `kill` suicide command. We can't route KTX's self-`T_Damage` here (the
    /// mode damage hooks — Midair, Arena — would swallow it), so drive the respawn directly but let
    /// the obituary own the "{name} suicides" message and the -2 frag penalty.
    fn client_kill(&mut self, e: EntId) {
        self.entities[e].deathtype = DeathType::Suicide;
        self.client_obituary(e, e);
        self.set_suicide_frame(e);
        self.entities[e].v.modelindex = self.level.modelindex_player;
        // `kill` respawns directly rather than through the death-think, so honor the mode's spawn
        // gate here too — otherwise a suicide could re-stack onto an occupied arena spot. When the
        // area isn't clear the corpse (now `Dead`) just falls into the normal death-think, which
        // retries the gated respawn on the next button press.
        let mode = self.mode;
        if mode.allow_respawn(self, e) {
            self.respawn(e);
        }
    }

    /// `respawn` — copy the corpse, reset parms, re-enter the server.
    pub(crate) fn respawn(&mut self, e: EntId) {
        self.copy_to_body_que(e);
        self.set_new_parms();
        self.put_client_in_server(e);
    }

    /// `CheckRules` — end the level on time/frag limit.
    fn check_rules(&mut self, e: EntId) {
        // Team matches (team DM / CTF) own their limits (team score / capture limit, in the shared
        // lifecycle) — don't let an individual player's frags trip the stock intermission path.
        if crate::mode::team::lifecycle_active(self) {
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
        if deadflag == DeadFlag::Dead {
            if b0 != 0.0 || b1 != 0.0 || b2 != 0.0 {
                return;
            }
            self.entities[e].v.deadflag = DeadFlag::Respawnable;
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
}
