//! Miscellaneous map entities, ported from `qw-qc/misc.qc`: info markers, lights, solid and
//! illusionary walls, and exploding boxes. (Cosmetic static decorations are networked as
//! ordinary still entities rather than via `makestatic`, which keeps edict bookkeeping
//! simple; rarely-used set pieces are spawned inert.)

use core::ffi::CStr;

use glam::Vec3;

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
    pub(crate) fn spawn_flame(&mut self, e: EntId, model: &'static CStr, frame: f32, fire: bool) -> bool {
        self.host.precache_model(model);
        self.entities[e].model_cstr = Some(model);
        self.host.set_model(e.0 as i32, model);
        self.entities[e].v.frame = frame;
        if fire {
            self.host.precache_sound(c"ambience/fire1.wav");
            let origin = self.entities[e].v.origin;
            self.host
                .ambient_sound(origin, c"ambience/fire1.wav", 0.5, ATTN_STATIC);
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
        self.host.precache_sound(c"ambience/fl_hum1.wav");
        let origin = self.entities[e].v.origin;
        self.host
            .ambient_sound(origin, c"ambience/fl_hum1.wav", 0.5, ATTN_STATIC);
        true
    }

    /// `light_fluorospark` — sparking broken light.
    pub(crate) fn spawn_light_fluorospark(&mut self, e: EntId) -> bool {
        if self.entities[e].v.skin == 0.0 {
            self.entities[e].v.skin = 10.0;
        }
        self.host.precache_sound(c"ambience/buzz1.wav");
        let origin = self.entities[e].v.origin;
        self.host
            .ambient_sound(origin, c"ambience/buzz1.wav", 0.5, ATTN_STATIC);
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
            ent.v.movetype = MOVETYPE_PUSH;
            ent.v.solid = SOLID_BSP;
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
            ent.v.movetype = MOVETYPE_NONE;
            ent.v.solid = SOLID_NOT;
        }
        self.set_brush_model(e);
        true
    }

    // --- exploding boxes ---

    /// `barrel_explode` (`th_die`).
    pub(crate) fn barrel_explode(&mut self, e: EntId) {
        {
            let ent = &mut self.entities[e];
            ent.v.takedamage = DAMAGE_NO;
            ent.classname = Some("explo_box".into());
        }
        self.t_radius_damage(e, e, 160.0, EntId::WORLD, "");
        let mut origin = self.entities[e].v.origin;
        origin.z += 32.0;
        self.host.write_byte(MSG_MULTICAST, SVC_TEMPENTITY);
        self.host.write_byte(MSG_MULTICAST, TE_EXPLOSION);
        self.host.write_coord(MSG_MULTICAST, origin.x);
        self.host.write_coord(MSG_MULTICAST, origin.y);
        self.host.write_coord(MSG_MULTICAST, origin.z);
        let center = self.entities[e].v.origin;
        self.host.multicast(center, MULTICAST_PHS);
        self.free(e);
    }

    /// `misc_explobox` / `misc_explobox2` — a shootable barrel.
    pub(crate) fn spawn_misc_explobox(&mut self, e: EntId, model: &'static CStr, size: Vec3) -> bool {
        {
            let ent = &mut self.entities[e];
            ent.v.solid = SOLID_BBOX;
            ent.v.movetype = MOVETYPE_NONE;
        }
        self.host.precache_model(model);
        self.entities[e].model_cstr = Some(model);
        self.host.set_model(e.0 as i32, model);
        self.host.set_size(e.0 as i32, Vec3::ZERO, size);
        self.host.precache_sound(c"weapons/r_exp3.wav");
        {
            let ent = &mut self.entities[e];
            ent.v.health = 20.0;
            ent.th_die = Die::ExploBoxDie;
            ent.v.takedamage = DAMAGE_AIM;
            ent.v.origin.z += 2.0;
        }
        if !self.host.droptofloor(e.0 as i32) {
            // Left as-is if it can't settle; matches the QuakeC tolerance.
        }
        true
    }
}
