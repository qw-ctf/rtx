//! Miscellaneous map entities, ported from `qw-qc/misc.qc`: info markers, lights, solid and
//! illusionary walls, and exploding boxes. (Cosmetic static decorations are networked as
//! ordinary still entities rather than via `makestatic`, which keeps edict bookkeeping
//! simple; rarely-used set pieces are spawned inert.)


use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::defs::*;
use crate::entity::{Die, EntId, Use};
use crate::game::GameState;


impl GameState {
    /// `info_null` — a target placeholder that removes itself.
    pub(crate) fn spawn_info_null(&mut self, _e: EntId) -> bool {
        false
    }

    /// `light` — toggleable named lights stay; inert lights are removed.
    pub(crate) fn spawn_light(&mut self, e: EntId) -> bool {
        if self.entities[e].targetname.is_none() {
            return false; // inert light
        }
        if self.entities[e].v.skin >= 32.0 {
            // `style` rides on skin for lights (we don't parse a separate style field).
            self.entities[e].use_ = Use::LightUse;
            let style = self.entities[e].v.skin as i32;
            let on = !self.entities[e].v.spawnflags.has(LightFlags::START_OFF);
            self.host.lightstyle(style, if on { c"m" } else { c"a" });
        }
        true
    }

    /// `light_use` — toggle a switchable light style.
    pub(crate) fn light_use(&mut self, e: EntId) {
        let style = self.entities[e].v.skin as i32;
        let sf = self.entities[e].v.spawnflags;
        if sf.has(LightFlags::START_OFF) {
            self.host.lightstyle(style, c"m");
            self.entities[e].v.spawnflags = sf.without(LightFlags::START_OFF);
        } else {
            self.host.lightstyle(style, c"a");
            self.entities[e].v.spawnflags = sf.with(LightFlags::START_OFF);
        }
    }

    /// A flame/torch/globe decoration: precache, show the model, attach ambient fire.
    pub(crate) fn spawn_flame(&mut self, e: EntId, model: Model, frame: f32, fire: bool) -> bool {
        self.entities[e].model_cstr = Some(model);
        self.host.set_model(e.0 as i32, model);
        self.entities[e].v.frame = frame;
        if fire {
            let origin = self.entities[e].v.origin;
            self.host
                .ambient_sound(origin, Sound::AMBIENCE_FIRE1, 0.5, Attenuation::Static);
        }
        true
    }

    /// `light_fluoro` — humming fluorescent light.
    pub(crate) fn spawn_light_fluoro(&mut self, e: EntId) -> bool {
        if self.entities[e].v.skin >= 32.0 {
            self.entities[e].use_ = Use::LightUse;
            let style = self.entities[e].v.skin as i32;
            let on = !self.entities[e].v.spawnflags.has(LightFlags::START_OFF);
            self.host.lightstyle(style, if on { c"m" } else { c"a" });
        }
        let origin = self.entities[e].v.origin;
        self.host
            .ambient_sound(origin, Sound::AMBIENCE_FL_HUM1, 0.5, Attenuation::Static);
        true
    }

    /// `light_fluorospark` — sparking broken light.
    pub(crate) fn spawn_light_fluorospark(&mut self, e: EntId) -> bool {
        if self.entities[e].v.skin == 0.0 {
            self.entities[e].v.skin = 10.0;
        }
        let origin = self.entities[e].v.origin;
        self.host
            .ambient_sound(origin, Sound::AMBIENCE_BUZZ1, 0.5, Attenuation::Static);
        true
    }

    // --- walls ---

    /// `func_wall_use` — flip to the alternate texture set.
    pub(crate) fn func_wall_use(&mut self, e: EntId) {
        let f = self.entities[e].v.frame;
        self.entities[e].v.frame = 1.0 - f;
    }

    /// `func_wall` — a solid brush wall (optionally texture-toggled by a trigger).
    pub(crate) fn spawn_func_wall(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.angles = Vec3::ZERO;
            ent.v.movetype = MoveType::Push.as_f32();
            ent.v.solid = Solid::Bsp.as_f32();
            ent.use_ = Use::FuncWallUse;
        }
        self.set_brush_model(e);
        true
    }

    /// `func_illusionary` — looks solid, but is walk-through.
    pub(crate) fn spawn_func_illusionary(&mut self, e: EntId) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.angles = Vec3::ZERO;
            ent.v.movetype = MoveType::None.as_f32();
            ent.v.solid = Solid::Not.as_f32();
        }
        self.set_brush_model(e);
        true
    }

    // --- exploding boxes ---

    /// `barrel_explode` (`th_die`).
    pub(crate) fn barrel_explode(&mut self, e: EntId) {
        {
            let ent = &mut self.entities[e];
            ent.v.takedamage = TakeDamage::No.as_f32();
            ent.classname = Some("explo_box".into());
        }
        self.t_radius_damage(e, e, 160.0, EntId::WORLD, "");
        let mut origin = self.entities[e].v.origin;
        origin.z += 32.0;
        self.host.write_te(MsgDest::Multicast, Te::Explosion);
        self.host.write_coord(MsgDest::Multicast, origin.x);
        self.host.write_coord(MsgDest::Multicast, origin.y);
        self.host.write_coord(MsgDest::Multicast, origin.z);
        let center = self.entities[e].v.origin;
        self.host.multicast(center, Multicast::Phs);
        self.free(e);
    }

    /// `misc_explobox` / `misc_explobox2` — a shootable barrel.
    pub(crate) fn spawn_misc_explobox(&mut self, e: EntId, model: Model, size: Vec3) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.solid = Solid::BBox.as_f32();
            ent.v.movetype = MoveType::None.as_f32();
        }
        self.entities[e].model_cstr = Some(model);
        self.host.set_model(e.0 as i32, model);
        self.host.set_size(e.0 as i32, Vec3::ZERO, size);
        {
            let ent = &mut self.entities[e];
            ent.v.health = 20.0;
            ent.th_die = Die::ExploBoxDie;
            ent.v.takedamage = TakeDamage::Aim.as_f32();
            ent.v.origin.z += 2.0;
        }
        if !self.host.droptofloor(e.0 as i32) {
            // Left as-is if it can't settle; matches the QuakeC tolerance.
        }
        true
    }
}
