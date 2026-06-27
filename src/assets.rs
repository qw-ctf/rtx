//! Type-safe asset handles, so a missing precache is unrepresentable.
//!
//! A [`Sound`]/[`Model`] can only be obtained from something that precached it: the registries
//! below (every const is precached by `precache_sounds`/`precache_models`) or the runtime escape
//! hatch [`DynAssets`] (which precaches a path before handing back a handle). Since `host.sound`
//! and `host.set_model` accept only these handles, you cannot use an asset that was never
//! precached. (Inline brush models — map `*N` submodels — are engine-managed and bypass this via
//! `host.set_model_brush`; they need no precache.)

use std::collections::HashMap;
use std::ffi::{CStr, CString};

use crate::host::HostApi;

/// Declare an asset-handle type `$ty` (a `'static` path), its `Ty::NAME` registry consts, and
/// a `$precache_all` that precaches every one via `host.$host_precache`. The single source of
/// truth: one line both names an asset and guarantees its precache.
macro_rules! asset_registry {
    ($ty:ident, $precache_all:ident, $host_precache:ident, { $($name:ident = $path:expr;)* }) => {
        /// A precached asset handle. Mint one only via the `'static` registry consts or
        /// [`DynAssets`] — never the private constructor; that is what ties "usable" to "precached".
        #[derive(Clone, Copy)]
        pub struct $ty(&'static CStr);

        impl $ty {
            /// The wire path, for the host layer to hand to the engine.
            pub(crate) const fn path(self) -> &'static CStr {
                self.0
            }
            $(pub const $name: $ty = $ty($path);)*
        }

        /// Precache every registered asset of this kind. Called once at worldspawn.
        pub fn $precache_all(host: &HostApi) {
            $(host.$host_precache($ty::$name.path());)*
        }
    };
}

asset_registry!(Sound, precache_sounds, precache_sound, {
    // ambience
    AMBIENCE_BUZZ1 = c"ambience/buzz1.wav";
    AMBIENCE_FIRE1 = c"ambience/fire1.wav";
    AMBIENCE_FL_HUM1 = c"ambience/fl_hum1.wav";
    AMBIENCE_HUM1 = c"ambience/hum1.wav";
    AMBIENCE_WINDFLY = c"ambience/windfly.wav";
    // boss1
    BOSS1_SIGHT1 = c"boss1/sight1.wav";
    // buttons
    BUTTONS_AIRBUT1 = c"buttons/airbut1.wav";
    BUTTONS_SWITCH02 = c"buttons/switch02.wav";
    BUTTONS_SWITCH04 = c"buttons/switch04.wav";
    BUTTONS_SWITCH21 = c"buttons/switch21.wav";
    // demon
    DEMON_DLAND2 = c"demon/dland2.wav";
    // doors
    DOORS_BASETRY = c"doors/basetry.wav";
    DOORS_BASEUSE = c"doors/baseuse.wav";
    DOORS_DDOOR1 = c"doors/ddoor1.wav";
    DOORS_DDOOR2 = c"doors/ddoor2.wav";
    DOORS_DOORMV1 = c"doors/doormv1.wav";
    DOORS_DRCLOS4 = c"doors/drclos4.wav";
    DOORS_HYDRO1 = c"doors/hydro1.wav";
    DOORS_HYDRO2 = c"doors/hydro2.wav";
    DOORS_MEDTRY = c"doors/medtry.wav";
    DOORS_MEDUSE = c"doors/meduse.wav";
    DOORS_RUNETRY = c"doors/runetry.wav";
    DOORS_RUNEUSE = c"doors/runeuse.wav";
    DOORS_STNDR1 = c"doors/stndr1.wav";
    DOORS_STNDR2 = c"doors/stndr2.wav";
    // items
    ITEMS_ARMOR1 = c"items/armor1.wav";
    ITEMS_DAMAGE = c"items/damage.wav";
    ITEMS_DAMAGE2 = c"items/damage2.wav";
    ITEMS_DAMAGE3 = c"items/damage3.wav";
    ITEMS_HEALTH1 = c"items/health1.wav";
    ITEMS_INV1 = c"items/inv1.wav";
    ITEMS_INV2 = c"items/inv2.wav";
    ITEMS_INV3 = c"items/inv3.wav";
    ITEMS_ITEMBK2 = c"items/itembk2.wav";
    ITEMS_PROTECT = c"items/protect.wav";
    ITEMS_PROTECT2 = c"items/protect2.wav";
    ITEMS_PROTECT3 = c"items/protect3.wav";
    ITEMS_R_ITEM1 = c"items/r_item1.wav";
    ITEMS_R_ITEM2 = c"items/r_item2.wav";
    ITEMS_SUIT = c"items/suit.wav";
    ITEMS_SUIT2 = c"items/suit2.wav";
    // misc
    MISC_H2OHIT1 = c"misc/h2ohit1.wav";
    MISC_NULL = c"misc/null.wav";
    MISC_OUTWATER = c"misc/outwater.wav";
    MISC_POWER = c"misc/power.wav";
    MISC_R_TELE1 = c"misc/r_tele1.wav";
    MISC_R_TELE2 = c"misc/r_tele2.wav";
    MISC_R_TELE3 = c"misc/r_tele3.wav";
    MISC_R_TELE4 = c"misc/r_tele4.wav";
    MISC_R_TELE5 = c"misc/r_tele5.wav";
    MISC_SECRET = c"misc/secret.wav";
    MISC_TALK = c"misc/talk.wav";
    MISC_TRIGGER1 = c"misc/trigger1.wav";
    MISC_WATER1 = c"misc/water1.wav";
    MISC_WATER2 = c"misc/water2.wav";
    // plats
    PLATS_MEDPLAT1 = c"plats/medplat1.wav";
    PLATS_MEDPLAT2 = c"plats/medplat2.wav";
    PLATS_PLAT1 = c"plats/plat1.wav";
    PLATS_PLAT2 = c"plats/plat2.wav";
    PLATS_TRAIN1 = c"plats/train1.wav";
    PLATS_TRAIN2 = c"plats/train2.wav";
    // player
    PLAYER_AXHIT1 = c"player/axhit1.wav";
    PLAYER_AXHIT2 = c"player/axhit2.wav";
    PLAYER_DEATH1 = c"player/death1.wav";
    PLAYER_DEATH2 = c"player/death2.wav";
    PLAYER_DEATH3 = c"player/death3.wav";
    PLAYER_DEATH4 = c"player/death4.wav";
    PLAYER_DEATH5 = c"player/death5.wav";
    PLAYER_DROWN1 = c"player/drown1.wav";
    PLAYER_DROWN2 = c"player/drown2.wav";
    PLAYER_GASP1 = c"player/gasp1.wav";
    PLAYER_GASP2 = c"player/gasp2.wav";
    PLAYER_GIB = c"player/gib.wav";
    PLAYER_H2ODEATH = c"player/h2odeath.wav";
    PLAYER_H2OJUMP = c"player/h2ojump.wav";
    PLAYER_INH2O = c"player/inh2o.wav";
    PLAYER_INLAVA = c"player/inlava.wav";
    PLAYER_LAND = c"player/land.wav";
    PLAYER_LAND2 = c"player/land2.wav";
    PLAYER_LBURN1 = c"player/lburn1.wav";
    PLAYER_LBURN2 = c"player/lburn2.wav";
    PLAYER_PAIN1 = c"player/pain1.wav";
    PLAYER_PAIN2 = c"player/pain2.wav";
    PLAYER_PAIN3 = c"player/pain3.wav";
    PLAYER_PAIN4 = c"player/pain4.wav";
    PLAYER_PAIN5 = c"player/pain5.wav";
    PLAYER_PAIN6 = c"player/pain6.wav";
    PLAYER_PLYRJMP8 = c"player/plyrjmp8.wav";
    PLAYER_SLIMBRN2 = c"player/slimbrn2.wav";
    PLAYER_TELEDTH1 = c"player/teledth1.wav";
    PLAYER_TORNOFF2 = c"player/tornoff2.wav";
    PLAYER_UDEATH = c"player/udeath.wav";
    // weapons
    WEAPONS_AX1 = c"weapons/ax1.wav";
    WEAPONS_BOUNCE = c"weapons/bounce.wav";
    WEAPONS_GRENADE = c"weapons/grenade.wav";
    WEAPONS_GUNCOCK = c"weapons/guncock.wav";
    WEAPONS_LHIT = c"weapons/lhit.wav";
    WEAPONS_LOCK4 = c"weapons/lock4.wav";
    WEAPONS_LSTART = c"weapons/lstart.wav";
    WEAPONS_PKUP = c"weapons/pkup.wav";
    WEAPONS_R_EXP3 = c"weapons/r_exp3.wav";
    WEAPONS_RIC1 = c"weapons/ric1.wav";
    WEAPONS_RIC2 = c"weapons/ric2.wav";
    WEAPONS_RIC3 = c"weapons/ric3.wav";
    WEAPONS_ROCKET1I = c"weapons/rocket1i.wav";
    WEAPONS_SGUN1 = c"weapons/sgun1.wav";
    WEAPONS_SHOTGN2 = c"weapons/shotgn2.wav";
    WEAPONS_SPIKE2 = c"weapons/spike2.wav";
    WEAPONS_TINK1 = c"weapons/tink1.wav";
});

asset_registry!(Model, precache_models, precache_model, {
    // maps
    MAPS_B_BATT0 = c"maps/b_batt0.bsp";
    MAPS_B_BATT1 = c"maps/b_batt1.bsp";
    MAPS_B_BH10 = c"maps/b_bh10.bsp";
    MAPS_B_BH100 = c"maps/b_bh100.bsp";
    MAPS_B_BH25 = c"maps/b_bh25.bsp";
    MAPS_B_EXBOX2 = c"maps/b_exbox2.bsp";
    MAPS_B_EXPLOB = c"maps/b_explob.bsp";
    MAPS_B_NAIL0 = c"maps/b_nail0.bsp";
    MAPS_B_NAIL1 = c"maps/b_nail1.bsp";
    MAPS_B_ROCK0 = c"maps/b_rock0.bsp";
    MAPS_B_ROCK1 = c"maps/b_rock1.bsp";
    MAPS_B_SHELL0 = c"maps/b_shell0.bsp";
    MAPS_B_SHELL1 = c"maps/b_shell1.bsp";
    // progs
    PROGS_ARMOR = c"progs/armor.mdl";
    PROGS_BACKPACK = c"progs/backpack.mdl";
    PROGS_BOLT = c"progs/bolt.mdl";
    PROGS_BOLT2 = c"progs/bolt2.mdl";
    PROGS_BOLT3 = c"progs/bolt3.mdl";
    PROGS_EYES = c"progs/eyes.mdl";
    PROGS_FLAME = c"progs/flame.mdl";
    PROGS_FLAME2 = c"progs/flame2.mdl";
    PROGS_G_LIGHT = c"progs/g_light.mdl";
    PROGS_G_NAIL = c"progs/g_nail.mdl";
    PROGS_G_NAIL2 = c"progs/g_nail2.mdl";
    PROGS_G_ROCK = c"progs/g_rock.mdl";
    PROGS_G_ROCK2 = c"progs/g_rock2.mdl";
    PROGS_G_SHOT = c"progs/g_shot.mdl";
    PROGS_GIB1 = c"progs/gib1.mdl";
    PROGS_GIB2 = c"progs/gib2.mdl";
    PROGS_GIB3 = c"progs/gib3.mdl";
    PROGS_GRENADE = c"progs/grenade.mdl";
    PROGS_H_PLAYER = c"progs/h_player.mdl";
    PROGS_INVISIBL = c"progs/invisibl.mdl";
    PROGS_INVULNER = c"progs/invulner.mdl";
    PROGS_LAVABALL = c"progs/lavaball.mdl";
    PROGS_MISSILE = c"progs/missile.mdl";
    PROGS_PLAYER = c"progs/player.mdl";
    PROGS_QUADDAMA = c"progs/quaddama.mdl";
    PROGS_S_BUBBLE = c"progs/s_bubble.spr";
    PROGS_S_EXPLOD = c"progs/s_explod.spr";
    PROGS_S_LIGHT = c"progs/s_light.spr";
    PROGS_S_SPIKE = c"progs/s_spike.mdl";
    PROGS_SPIKE = c"progs/spike.mdl";
    PROGS_SUIT = c"progs/suit.mdl";
    PROGS_V_AXE = c"progs/v_axe.mdl";
    PROGS_V_LIGHT = c"progs/v_light.mdl";
    PROGS_V_NAIL = c"progs/v_nail.mdl";
    PROGS_V_NAIL2 = c"progs/v_nail2.mdl";
    PROGS_V_ROCK = c"progs/v_rock.mdl";
    PROGS_V_ROCK2 = c"progs/v_rock2.mdl";
    PROGS_V_SHOT = c"progs/v_shot.mdl";
    PROGS_V_SHOT2 = c"progs/v_shot2.mdl";
    PROGS_ZOM_GIB = c"progs/zom_gib.mdl";
});

/// Escape hatch for string-declared (runtime / map-supplied) assets absent from the registries:
/// precaches each on first sight and interns it, keeping it `'static` and deduping repeats — so
/// "usable implies precached" still holds. Lives in `GameState`; registration must happen at load
/// time. (No string-declared assets in the port yet; here for when one appears.)
#[derive(Default)]
pub struct DynAssets {
    sounds: HashMap<CString, &'static CStr>,
    models: HashMap<CString, &'static CStr>,
}

impl DynAssets {
    /// Precache a runtime sound path (idempotently) and return its handle.
    #[allow(dead_code)]
    pub fn sound(&mut self, host: &HostApi, path: &CStr) -> Sound {
        Sound(intern(&mut self.sounds, path, || host.precache_sound(path)))
    }
    /// Precache a runtime model path (idempotently) and return its handle.
    #[allow(dead_code)]
    pub fn model(&mut self, host: &HostApi, path: &CStr) -> Model {
        Model(intern(&mut self.models, path, || host.precache_model(path)))
    }
}

/// Return the interned `'static` copy of `path`, precaching (via `precache`) on first insertion.
fn intern(
    map: &mut HashMap<CString, &'static CStr>,
    path: &CStr,
    precache: impl FnOnce(),
) -> &'static CStr {
    if let Some(&p) = map.get(path) {
        return p;
    }
    precache();
    let leaked: &'static CStr = Box::leak(path.to_owned().into_boxed_c_str());
    map.insert(path.to_owned(), leaked);
    leaked
}
