//! Player lifecycle, ported from `qw-qc/client.qc`: connect/disconnect, spawn parameters,
//! spawn-point selection, and `PutClientInServer`. Movement itself is the engine's job
//! (QuakeWorld player physics) once the player entity is set up; combat, water, weapon
//! frames and powerups arrive in later milestones.

use std::ffi::CString;

use glam::Vec3;

use crate::defs::*;
use crate::entity::EntId;
use crate::game::GameState;

impl GameState {
    /// `ClientConnect` — announce a joining player and record their name.
    pub(crate) fn client_connect(&mut self, player: EntId) {
        let name = self.read_netname(player);
        let ent = &mut self.entities[player.index()];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.netname = Some(name.as_str().into());
        self.broadcast(PRINT_HIGH, &format!("{name} entered the game\n"));
    }

    /// `ClientDisconnect` — announce a leaving player.
    pub(crate) fn client_disconnect(&mut self, player: EntId) {
        let ent = &self.entities[player.index()];
        let name = ent.netname.as_deref().unwrap_or("");
        let frags = ent.v.frags as i32;
        let message = format!("{name} left the game with {frags} frags\n");
        self.broadcast(PRINT_HIGH, &message);
        self.host
            .sound(player.0 as i32, CHAN_BODY, c"player/tornoff2.wav", 1.0, ATTN_NONE);
    }

    /// `SetNewParms` — default spawn parameters for a fresh player.
    pub(crate) fn set_new_parms(&mut self) {
        let p = &mut self.globals.parm;
        p[0] = IT_SHOTGUN + IT_AXE; // items
        p[1] = 100.0; // health
        p[2] = 0.0; // armor value
        p[3] = 25.0; // shells
        p[4] = 0.0; // nails
        p[5] = 0.0; // rockets
        p[6] = 0.0; // cells
        p[7] = 1.0; // weapon = IT_SHOTGUN
        p[8] = 0.0; // armor type
    }

    /// `SetChangeParms` — persist a surviving player's state across a level change.
    pub(crate) fn set_change_parms(&mut self, player: EntId) {
        let v = &self.entities[player.index()].v;
        if v.health <= 0.0 {
            self.set_new_parms();
            return;
        }

        let keep_mask =
            (IT_KEY1 + IT_KEY2 + IT_INVISIBILITY + IT_INVULNERABILITY + IT_SUIT + IT_QUAD) as i32;
        let items = (v.items as i32 & !keep_mask) as f32;
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
        let v = &mut self.entities[player.index()].v;
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
        {
            let ent = &mut self.entities[player.index()];
            ent.in_use = true;
            ent.classname = Some("player".into());
            ent.v.health = 100.0;
            ent.v.takedamage = DAMAGE_AIM;
            ent.v.solid = SOLID_SLIDEBOX;
            ent.v.movetype = MOVETYPE_WALK;
            ent.v.max_health = 100.0;
            ent.v.flags = FL_CLIENT;
            ent.v.effects = 0.0;
            ent.v.deadflag = DEAD_NO;
        }

        self.decode_level_parms(player);
        self.w_set_current_ammo(player);

        let spot = self.select_spawn_point();
        let origin = self.entities[spot.index()].v.origin + Vec3::new(0.0, 0.0, 1.0);
        let angles = self.entities[spot.index()].v.angles;
        {
            let ent = &mut self.entities[player.index()];
            ent.v.origin = origin;
            ent.v.angles = angles;
            ent.v.fixangle = 1.0; // snap the client's view immediately
            ent.v.view_ofs = VEC_VIEW_OFS;
            ent.v.velocity = Vec3::ZERO;
        }

        // Assign the player model and bounding box; both relink the entity in the engine
        // at its current origin.
        self.host.set_model(player.0 as i32, c"progs/player.mdl");
        self.host
            .set_size(player.0 as i32, VEC_HULL_MIN, VEC_HULL_MAX);

        // Kick off the idle/run animation think loop.
        self.player_stand1(player);
    }

    /// `PlayerPreThink` — runs before engine physics. Minimal for now (movement is engine
    /// side); combat/water/jump/weapon logic lands in M3.
    pub(crate) fn player_pre_think(&mut self, player: EntId) {
        let v_angle = self.entities[player.index()].v.v_angle;
        self.host.make_vectors(v_angle);
    }

    /// `PlayerPostThink` — runs after engine physics. Minimal for now.
    pub(crate) fn player_post_think(&mut self, _player: EntId) {}

    /// `W_SetCurrentAmmo` — sync `currentammo`/`weaponmodel`/ammo item bits to the active
    /// weapon. (The weapon view model is recorded but not yet networked — vwep is M3.)
    fn w_set_current_ammo(&mut self, player: EntId) {
        let ent = &mut self.entities[player.index()];
        let mut items = ent.v.items as i32 & !((IT_SHELLS + IT_NAILS + IT_ROCKETS + IT_CELLS) as i32);

        let (ammo, model, ammo_bit): (f32, Option<&str>, f32) = match ent.v.weapon {
            w if w == IT_AXE => (0.0, Some("progs/v_axe.mdl"), 0.0),
            w if w == IT_SHOTGUN => (ent.v.ammo_shells, Some("progs/v_shot.mdl"), IT_SHELLS),
            w if w == IT_SUPER_SHOTGUN => (ent.v.ammo_shells, Some("progs/v_shot2.mdl"), IT_SHELLS),
            w if w == IT_NAILGUN => (ent.v.ammo_nails, Some("progs/v_nail.mdl"), IT_NAILS),
            w if w == IT_SUPER_NAILGUN => (ent.v.ammo_nails, Some("progs/v_nail2.mdl"), IT_NAILS),
            w if w == IT_GRENADE_LAUNCHER => {
                (ent.v.ammo_rockets, Some("progs/v_rock.mdl"), IT_ROCKETS)
            }
            w if w == IT_ROCKET_LAUNCHER => {
                (ent.v.ammo_rockets, Some("progs/v_rock2.mdl"), IT_ROCKETS)
            }
            w if w == IT_LIGHTNING => (ent.v.ammo_cells, Some("progs/v_light.mdl"), IT_CELLS),
            _ => (0.0, None, 0.0),
        };

        if ammo_bit != 0.0 {
            items |= ammo_bit as i32;
        }
        ent.v.currentammo = ammo;
        ent.v.weaponframe = 0.0;
        ent.v.items = items as f32;
        ent.weaponmodel = model.map(Into::into);
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
        let origin = self.entities[spot.index()].v.origin;
        self.find_by_classname("player")
            .any(|p| (self.entities[p.index()].v.origin - origin).length() < 84.0)
    }

    // --- small helpers ---

    /// Read the player's `name` userinfo key.
    fn read_netname(&self, player: EntId) -> String {
        let mut buf = [0u8; 64];
        self.host
            .infokey(player.0 as i32, c"name", &mut buf)
            .to_owned()
    }

    /// `bprint` of a dynamic message to every client.
    fn broadcast(&self, level: i32, message: &str) {
        if let Ok(c) = CString::new(message) {
            self.host.bprint(level, &c);
        }
    }
}
