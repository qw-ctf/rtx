// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot combat — the shooting/dodging layer the navmesh bots gained for Rocket Arena (the
//! README's deferred "combat is planned next"). It is mode-agnostic: [`crate::bot::run_bot`]
//! calls [`engage`] whenever the active mode hands it an enemy via
//! [`GameMode::bot_enemy`](crate::mode::GameMode::bot_enemy), so any future mode (instagib, CTF,
//! …) reuses it for free.
//!
//! Bots run their usercmd through the engine's player-move + weapon code just like humans, so
//! "combat" here is purely a matter of choosing the view angles, weapon (an `impulse`), the
//! attack button, and evasive movement — never a direct weapon-fire call. [`engage`] overlays
//! those onto the movement the navmesh already produced: while the bot has no line of sight it
//! keeps navigating toward the enemy untouched; once it can see them it aims (leading the target
//! for projectiles), picks a weapon by range, strafes/retreats, and fires.

use glam::{Vec3, Vec3Swizzles};

mod aim;
pub(crate) use aim::*;

use crate::abi::EntVars;
use crate::arsenal::{self, AmmoKind};
use crate::bot::state::GrenadePhase;
use crate::bot::{grenade, BotCmd};
use crate::defs::{
    Bits, Content, FieldEq, Flags, Items, Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK,
    BUTTON_JUMP, RUNE_RESISTANCE, RUNE_STRENGTH, VEC_HULL_MAX, VEC_VIEW_OFS,
};
use crate::entity::{EntId, Touch};
use crate::game::GameState;
use crate::math::{angle_vectors, angles_to, wrap180};

/// Rocket/grenade projectile speed (QuakeWorld `SV_FireRocket`), for target leading.
const ROCKET_SPEED: f32 = 1000.0;
/// Nail (spike) projectile speed (`launch_spike`, weapons.rs) — nailguns fire straight, no gravity.
const NAIL_SPEED: f32 = 1000.0;
/// Preferred fighting distance for the rocket launcher — close enough to hit, far enough to dodge
/// the reply and not splash ourselves.
const PREFERRED_RANGE: f32 = 400.0;
/// Below this we're in self-splash territory for the RL — switch to the super shotgun.
const SPLASH_RANGE: f32 = 140.0;
/// The lightning gun's beam reaches this far (`w_fire_lightning` traces `v_forward * 600`). The normal
/// mid-range pick caps LG at a conservative `PREFERRED_RANGE + 150` (550); the *finishing* pick uses
/// the true reach, so a believed-low enemy in the 550–600 band still draws the bolt over the rocket.
const LG_RANGE: f32 = 600.0;
/// KTX's `AvoidQuadBore` treats an enemy inside 250 units as a quad-explosive danger even though
/// the physical radius is 160: quad turns the normally-small edge splash into a fight-ending hit,
/// and the projectile strikes the near face of the target rather than its reported origin.
const QUAD_SPLASH_CAUTION_RANGE: f32 = 250.0;
/// A direct rocket aimed at a player detonates on the near side of their hull, not at the aim point.
/// Subtract this conservative hull allowance before projecting our own splash damage.
const EXPLOSIVE_IMPACT_MARGIN: f32 = 24.0;
/// How far short of the aim point a projectile may land and still count as a clear shot. A wall
/// that stops the rocket more than this before `aim` means the muzzle→aim path is blocked (corner
/// self-splash; blast radius is 160 and attacker self-damage is only halved). Matches the slack in
/// [`crate::bot::grenade::rocket_shove`].
const LINE_OF_FIRE_SLACK: f32 = 48.0;
/// How long *before* an airborne enemy touches down a rocket may still arrive and be better put into
/// the ground under them than into the body ([`aim_solution`]). The blast reaches 160 units, so a
/// target this close to landing — tens of units up, and dropping into it — eats the floor splash
/// anyway; and eating it airborne is the point, since the blast throws them further up, where the
/// next rocket has a body on a clean parabola to solve. Past this they're properly in the air, with
/// room to be anywhere by the time the shot lands, and the hull is the better target.
const FLOOR_SPLASH_LEEWAY: f32 = 0.15;
/// Retreat when hurt below this.
const LOW_HEALTH: f32 = 40.0;
/// Upper range on the single-barrel *shotgun* finish (the lightning finish keeps its full
/// [`LG_RANGE`] beam). The pattern holds its full ~24-dmg six pellets only while its cone half-width
/// (`dist · 0.04`, the [`w_fire_shotgun`](crate::weapons) spread) stays under the ~16u target
/// half-width — about 400 units; two shots then close out a [`FINISH_STACK`] enemy. Past this the
/// cone opens up and it needs 3+ hits, so a believed-low enemy is better finished with the rocket's
/// splash than a switch to a shotgun that can't reach — the "swapped too early" the user saw.
const FINISH_SHOTGUN_RANGE: f32 = 450.0;

/// Opponent-model "press the advantage" thresholds. When the current enemy is believed to be on a
/// finishable stack (below [`FINISH_STACK`] — under one rocket even through green armor), the belief
/// is fresh (younger than [`FINISH_FRESH`]; a stale estimate has drifted and shouldn't buy risk), and
/// the bot itself isn't critical (health ≥ [`PRESS_FLOOR`]), the bot lowers its own retreat threshold
/// and closes in to finish the kill instead of holding range — the deathmatch-1 "he's low, go get
/// him" read the user asked for.
const FINISH_STACK: f32 = 35.0;
const FINISH_FRESH: f32 = 4.0;
const PRESS_FLOOR: f32 = 20.0;

/// Whether to press a finishable kill rather than retreat — see the threshold constants above. Pure.
fn press_advantage(own_health: f32, enemy_stack: f32, est_age: f32) -> bool {
    enemy_stack < FINISH_STACK && est_age < FINISH_FRESH && own_health >= PRESS_FLOOR
}

/// A weapon choice for the current range: the impulse that selects it, the [`Weapon`] it yields
/// (to avoid re-selecting what we already hold), and its projectile speed (`0` = hitscan, so no
/// target leading). `grenade_arc` marks a *lobbed* grenade solution (airborne intercept or grounded
/// lead-lob) whose view angles come pre-solved from the ballistic solver — the aim code fires along
/// them directly instead of pointing straight at the target.
#[derive(Clone, Copy)]
struct WeaponChoice {
    impulse: i32,
    weapon: Weapon,
    projectile_speed: f32,
    grenade_arc: bool,
}

impl WeaponChoice {
    /// The direct-fire choice for a weapon: its select impulse and (for projectile guns) the shot
    /// speed the aim lead uses; hitscan is `0.0`. One table in place of the per-arm struct literals.
    fn of(weapon: Weapon) -> WeaponChoice {
        let (impulse, projectile_speed) = match weapon {
            Weapon::SuperShotgun => (3, 0.0),
            Weapon::Shotgun => (2, 0.0),
            Weapon::Lightning => (8, 0.0),
            Weapon::RocketLauncher => (7, ROCKET_SPEED),
            Weapon::SuperNailgun => (5, NAIL_SPEED),
            Weapon::Nailgun => (4, NAIL_SPEED),
            _ => (1, 0.0), // axe — the hard fallback
        };
        WeaponChoice { impulse, weapon, projectile_speed, grenade_arc: false }
    }

    /// The grenade-launcher arc shot (a validated lob/intercept solution — `engage` owns the aim).
    fn grenade() -> WeaponChoice {
        WeaponChoice {
            impulse: grenade::GL_IMPULSE,
            weapon: Weapon::GrenadeLauncher,
            projectile_speed: grenade::GL_SPEED,
            grenade_arc: true,
        }
    }
}

/// A bot's weapon inventory and ammo pools — the pure inputs to weapon selection, so [`choose_weapon`]
/// and [`Loadout::gl_primary`] can be unit-tested without a live [`GameState`].
#[derive(Clone, Copy)]
struct Loadout {
    items: Items,
    shells: f32,
    nails: f32,
    rockets: f32,
    cells: f32,
}

impl Loadout {
    fn of(v: &EntVars) -> Loadout {
        Loadout {
            items: Items::from_f32(v.items),
            shells: v.ammo_shells,
            nails: v.ammo_nails,
            rockets: v.ammo_rockets,
            cells: v.ammo_cells,
        }
    }

    fn has(&self, bit: Items) -> bool {
        self.items.contains(bit)
    }

    /// Amount held in an ammo pool.
    fn ammo(&self, kind: AmmoKind) -> f32 {
        match kind {
            AmmoKind::Shells => self.shells,
            AmmoKind::Nails => self.nails,
            AmmoKind::Rockets => self.rockets,
            AmmoKind::Cells => self.cells,
        }
    }

    /// Owns `w` and has the ammo to fire it — the arsenal's `min_ammo` gate (the axe/grapple, which
    /// draw no ammo, are fed whenever owned).
    fn fed(&self, w: Weapon) -> bool {
        let item = w.item();
        self.has(item)
            && arsenal::weapon_spec(item).is_some_and(|s| s.ammo_kind.is_none_or(|k| self.ammo(k) >= s.min_ammo))
    }

    /// The direct (non-explosive, before-axe) guns [`choose_weapon`] can pick, best first.
    const DIRECT_GUNS: [Weapon; 5] = [
        Weapon::SuperShotgun,
        Weapon::Lightning,
        Weapon::SuperNailgun,
        Weapon::Nailgun,
        Weapon::Shotgun,
    ];

    /// A fireable super-shotgun / lightning / super-nailgun / nailgun / shotgun — any hitscan or nail
    /// gun [`choose_weapon`] can pick before the axe. Used to decide whether the GL is the bot's
    /// *only* real weapon.
    fn has_direct_gun(&self) -> bool {
        Self::DIRECT_GUNS.iter().any(|&w| self.fed(w))
    }

    /// The grenade launcher is the bot's only offensive gun: a fireable GL, no RL, and no direct gun.
    /// (RL and GL share the rocket pool, so a fireable GL + no RL owned ⇒ no fireable RL.) When true,
    /// [`engage`] solves the grenade arc itself even with shootable grenades on — the lob→shoot combo
    /// can't help (it has no hitscan detonator), so the bot would otherwise be stuck on the axe.
    fn gl_primary(&self) -> bool {
        self.has(Items::GRENADE_LAUNCHER)
            && self.rockets >= 1.0
            && !self.has(Items::ROCKET_LAUNCHER)
            && !self.has_direct_gun()
    }
}

/// Pick a weapon for `dist`, given what the bot owns and has ammo for. `gl_air`/`gl_ground` are set
/// by [`engage`] once it has a *validated* grenade-arc solution (an airborne intercept, or a
/// grounded lead-lob) — when either holds, the grenade launcher wins. `underwater` (own `waterlevel
/// > 1`, the discharge condition in `w_fire_lightning`) bars the lightning gun: firing it submerged
/// dumps all cells as a self-lethal discharge. The server's auto-pick guard (`w_best_weapon`) can't
/// help — the bot forces its weapon by impulse, which bypasses it — so the ban lives here (and, as a
/// belt-and-suspenders fire gate, in [`engage`]).
///
/// `finishable` — the opponent model believes this enemy is on a finishable stack (set by [`engage`]
/// from [`est_strength`](crate::bot::model::est_strength) against [`FINISH_STACK`], the same read the
/// movement `press` uses). When set, a mid-range pick that would otherwise be a dodgeable projectile
/// becomes a hitscan direct hit — the lightning gun in beam range, else the tight single-barrel
/// shotgun within [`FINISH_SHOTGUN_RANGE`] — so the near-kill lands the instant it fires instead of
/// being strafed clear of a rocket's ~0.4 s flight. Past the shotgun's finishing range (and out of
/// beam range) the rocket's splash is the better closer, so the finish leaves the pick alone.
fn choose_weapon(inv: Loadout, dist: f32, gl_air: bool, gl_ground: bool, underwater: bool, finishable: bool) -> WeaponChoice {
    // A solved airborne grenade intercept takes precedence: it's the shot we came here to take.
    if gl_air {
        return WeaponChoice::grenade();
    }
    // Point blank: the super shotgun then shotgun (hitscan, no self-splash).
    if dist < SPLASH_RANGE {
        if inv.fed(Weapon::SuperShotgun) {
            return WeaponChoice::of(Weapon::SuperShotgun);
        }
        if inv.fed(Weapon::Shotgun) {
            return WeaponChoice::of(Weapon::Shotgun);
        }
    }
    // Mid range: the lightning gun (fast, high DPS) when fed — never submerged (it would discharge).
    if dist < PREFERRED_RANGE + 150.0 && inv.fed(Weapon::Lightning) && !underwater {
        return WeaponChoice::of(Weapon::Lightning);
    }
    // A finishable enemy past point blank: a hitscan direct hit lands the kill the instant it fires,
    // where the rocket's flight lets a near-dead target strafe clear. Lightning if fed and in beam
    // range (its true 600, past the conservative 550 the branch above stops at), else the tight
    // single-barrel shotgun — not the wide SSG, which patterns worse at this distance — but only
    // within FINISH_SHOTGUN_RANGE, where two barrels still close the enemy out; beyond it the pattern
    // can't finish and we keep the rocket's splash. Only reached when the branches above didn't
    // already pick a hitscan gun, i.e. exactly the RL/projectile case.
    if finishable && dist >= SPLASH_RANGE {
        if inv.fed(Weapon::Lightning) && dist < LG_RANGE && !underwater {
            return WeaponChoice::of(Weapon::Lightning);
        }
        if inv.fed(Weapon::Shotgun) && dist < FINISH_SHOTGUN_RANGE {
            return WeaponChoice::of(Weapon::Shotgun);
        }
    }
    // Default: the rocket launcher (projectile, lead the target).
    if inv.fed(Weapon::RocketLauncher) {
        return WeaponChoice::of(Weapon::RocketLauncher);
    }
    // No rocket launcher but a solved grenade lob: prefer the arc over the shotgun at range (the GL
    // reaches where the SSG can't). `engage` only sets this when a lob actually solves, so we never
    // pick it hopelessly.
    if gl_ground {
        return WeaponChoice::grenade();
    }
    // Ammo-starved fallbacks: the best owned+fed gun before the axe. This is also the *only* branch a
    // stock-loadout (shotgun + axe) bot reaches at range, so without these it would roam throwing the
    // axe at distant enemies; the nailguns sit here too (never preferred over RL/SSG/LG), so a bot
    // restricted to a nailgun via `rtx_weapons` still fights with it. The axe is the hard fallback.
    Loadout::DIRECT_GUNS
        .into_iter()
        .filter(|&w| !(underwater && w == Weapon::Lightning)) // an LG-only bot discharges otherwise
        .find(|&w| inv.fed(w))
        .map_or_else(|| WeaponChoice::of(Weapon::Axe), WeaponChoice::of)
}

/// Best non-explosive fallback when a teammate occupies the intended splash area. This keeps a bot
/// fighting instead of merely holding an unsafe rocket: close range favors shotguns, medium range
/// lightning, then nails and the remaining fed hitscan guns. `None` means explosives/axe are all it
/// has, in which case the final fire gate simply withholds the unsafe shot.
fn safe_direct_choice(inv: Loadout, dist: f32, underwater: bool) -> Option<WeaponChoice> {
    let ordered = if dist < SPLASH_RANGE {
        [
            Weapon::SuperShotgun,
            Weapon::Shotgun,
            Weapon::Lightning,
            Weapon::SuperNailgun,
            Weapon::Nailgun,
        ]
    } else {
        [
            Weapon::Lightning,
            Weapon::SuperNailgun,
            Weapon::Nailgun,
            Weapon::SuperShotgun,
            Weapon::Shotgun,
        ]
    };
    ordered
        .into_iter()
        .filter(|&w| !(underwater && w == Weapon::Lightning))
        .find(|&w| inv.fed(w))
        .map(WeaponChoice::of)
}

/// The pieces of a bot's current stack that determine health lost to its own explosive. Kept as a
/// pure snapshot so quad/rune/armor arithmetic and policy can be tested without a live game.
#[derive(Clone, Copy)]
struct OwnSplashState {
    health: f32,
    armor_value: f32,
    armor_type: f32,
    damage_scale: f32,
    quad: bool,
    immune: bool,
}

impl OwnSplashState {
    fn of(game: &GameState, e: EntId) -> Self {
        let now = game.time();
        let ent = &game.entities[e];
        let quad = ent.combat.super_damage_finished > now;
        let mut damage_scale = if quad {
            if game.level.deathmatch == 4 { 8.0 } else { 4.0 }
        } else {
            1.0
        };
        // CTF's mode hook applies these after quad and before armor, including to self-splash.
        if game.mode.name() == "ctf" {
            if ent.mode_p.ctf.runes & RUNE_STRENGTH != 0 {
                damage_scale *= 2.0;
            }
            if ent.mode_p.ctf.runes & RUNE_RESISTANCE != 0 {
                damage_scale *= 0.5;
            }
        }
        Self {
            health: ent.v.health,
            armor_value: ent.v.armorvalue,
            armor_type: ent.v.armortype,
            damage_scale,
            quad,
            // `t_damage` protects health under pent/god; Midair's mode hook makes every self-shot
            // damage-free while preserving rocket knockback. KTX likewise exempts pent/Midair.
            immune: ent.v.flags.has(Flags::GODMODE)
                || ent.combat.invincible_finished >= now
                || game.mode.name() == "midair",
        }
    }
}

/// Health that gets through armor from our own rocket/grenade at an exact blast-centre distance.
/// Mirrors `t_radius_damage`'s attacker half-damage, `t_damage`'s quad/mode multiplier, and
/// `apply_armor`'s ceil/clamp arithmetic. Pent/god/Midair return zero health damage.
fn own_splash_health_damage(state: OwnSplashState, distance: f32) -> f32 {
    if state.immune {
        return 0.0;
    }
    let damage = blast_self_damage(distance) * 0.5 * state.damage_scale;
    let save = (state.armor_type * damage).ceil().min(state.armor_value);
    (damage - save).ceil().max(0.0)
}

/// Safety policy for deliberately creating our own explosion. Quad gets KTX's enlarged 250-unit
/// caution zone; otherwise (and beyond it) projected post-armor health loss may consume at most the
/// same half-health budget used by grenade tactics. This is substantially stricter than merely
/// avoiding a lethal shot and leaves room for damage received while the projectile is in flight.
fn own_splash_safe(state: OwnSplashState, distance: f32, impact_margin: f32) -> bool {
    if state.immune {
        return true;
    }
    if state.quad && distance <= QUAD_SPLASH_CAUTION_RANGE {
        return false;
    }
    let blast_distance = (distance - impact_margin).max(0.0);
    own_splash_health_damage(state, blast_distance)
        <= state.health.max(1.0) * GRENADE_SHOOT_HEALTH_FRAC
}

/// Exact-centre variant used by the owned-grenade/rocket tactics module.
pub(crate) fn own_explosion_safe_at(game: &GameState, e: EntId, distance: f32) -> bool {
    own_splash_safe(OwnSplashState::of(game, e), distance, 0.0)
}

/// Intended-aim variant used for a rocket/GL shot: account for impact on the near face of a hull.
fn own_explosive_aim_safe(game: &GameState, e: EntId, distance: f32) -> bool {
    own_splash_safe(
        OwnSplashState::of(game, e),
        distance,
        EXPLOSIVE_IMPACT_MARGIN,
    )
}

/// One player caught inside a would-be discharge blast, as the bot's belief sees them: distance from
/// the blast centre, estimated effective HP, and whether they're believed to hold quad.
struct DischargeVictim {
    dist: f32,
    strength: f32,
    quad: bool,
}

/// Whether dumping `cells` as an underwater discharge is a worthwhile sacrifice. The blast deals
/// `35·cells` at the centre with `−0.5/unit` falloff (`combat.rs::t_radius_damage`), and the firer
/// eats `17.5·cells` (halved, but never excluded from its own blast) — so it's usually self-lethal,
/// justified only when it kills a believed **quad** carrier or **≥2** enemies at once. A plain 1v1
/// never qualifies. A victim counts as killed when the blast damage at their range meets their
/// estimated strength.
fn discharge_worth_it(cells: f32, victims: &[DischargeVictim]) -> bool {
    let kills = |v: &DischargeVictim| 35.0 * cells - 0.5 * v.dist >= v.strength;
    let quad_kill = victims.iter().any(|v| kills(v) && v.quad);
    let multi_kill = victims.iter().filter(|&v| kills(v)).count() >= 2;
    quad_kill || multi_kill
}

/// The players a discharge from `e` (dumping its cells at `origin`) would catch: living non-teammate
/// clients inside the blast radius with a clear line to the centre, each tagged with the bot's belief
/// about their strength and quad. Empty when `rtx_bot_model` is off (`opponent_est` returns `None`),
/// so a modeling-disabled bot can never talk itself into a discharge — it just never fires the LG
/// underwater.
fn discharge_victims(game: &mut GameState, e: EntId, origin: Vec3, now: f32) -> Vec<DischargeVictim> {
    let cells = game.entities[e].v.ammo_cells;
    let radius = 35.0 * cells + 40.0; // t_radius_damage blast radius (damage + 40)
    let my_team = game.entities[e].mode_p.team;
    let maxclients = game.host().cvar(c"maxclients") as i32;
    let mut victims = Vec::new();
    for p in (1..=maxclients as u32).map(EntId) {
        if p == e {
            continue;
        }
        let (ok, center) = {
            let ent = &game.entities[p];
            let teammate = my_team != 0 && ent.mode_p.team == my_team;
            let ok = ent.in_use && ent.is_player() && ent.v.health > 0.0 && !teammate;
            (ok, ent.v.origin + (ent.v.mins + ent.v.maxs) * 0.5)
        };
        if !ok || (center - origin).length() > radius {
            continue;
        }
        // A wall between the firer and the victim stops radius damage (t_radius_damage's can_damage).
        let tr = game.traceline(origin, center, false, e);
        if tr.ent != p && tr.fraction <= 0.95 {
            continue;
        }
        let Some(est) = game.opponent_est(e, p, now) else {
            continue; // modeling off / no pool → no belief → don't count them
        };
        victims.push(DischargeVictim {
            dist: (center - origin).length(),
            strength: crate::bot::model::est_strength(&est, now),
            quad: est.quad_until > now,
        });
    }
    victims
}

/// How long after losing sight of the enemy the bot keeps *holding the angle* where they vanished
/// (like a player holding a corner) before its eyes fall back to the navigation view.
const HOLD_ANGLE_TIME: f32 = 2.0;

/// How a candidate move's footing rates for a fighting bot: dry ground (best), swimmable water (slow
/// and exposed — accept only if nothing dry works), or a lethal hazard (lava/slime/a pit — never step
/// there). Ordered best-to-worst.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Footing {
    Dry,
    Wet,
    Hazard,
}

/// Choose the best combat move near a hazard or water. Candidates are tried in priority order — the
/// wanted move, the wanted move with the strafe flipped, each strafe alone, then the forward/backpedal
/// component alone. Two passes: a candidate that lands on dry ground wins outright; failing that, the
/// first that isn't a lethal hazard (wading shallow water beats freezing at a lava edge); if every
/// option is a hazard the bot holds ground rather than walking off. Backpedal is dropped before the
/// strafe, so a bot pinned at a lava edge sidesteps rather than backing in. `footing` classifies where
/// a horizontal move lands. Pure over the oracle, so the priority order is unit-testable.
///
/// The one exception is `burning` — the bot is *already standing in* lava/slime, not merely beside it.
/// Then holding ground means cooking, so the all-hazard fallback takes the wanted move (candidate 0)
/// and walks off the damage instead of freezing on it.
///
/// That fallback is a last resort, not the plan: it moves the bot *somewhere*, but somewhere is chosen
/// by where the enemy is, not where the bank is. This used to be the whole answer, and the bot duly
/// circle-strafed its opponent inside the pool until it died — the comment here claimed the routing
/// gradient would steer it ashore, but a fight overwrites `move_world`, so the route never reached the
/// feet. [`combat_move`] now heads for the shore itself before consulting this at all; by the time the
/// `burning` arm below fires, the bot has no fix on a bank to walk to.
fn safe_combat_move(
    footing: &impl Fn(Vec3) -> Footing,
    dir: Vec3,
    perp: Vec3,
    want_forward: f32,
    strafe_sign: f32,
    burning: bool,
) -> Vec3 {
    let fwd = dir * want_forward;
    let strafe = perp * (strafe_sign * MOVE_SPEED);
    let candidates = [fwd + strafe, fwd - strafe, strafe, -strafe, fwd];
    if let Some(&mv) = candidates.iter().find(|&&mv| footing(mv) == Footing::Dry) {
        return mv; // dry footing preferred — bias the dodge toward the bank
    }
    // Candidate 0 (the wanted move + strafe) is always nonzero — the strafe term is ±MOVE_SPEED — so a
    // burning bot moves off rather than settling for Vec3::ZERO on the coals.
    let hold = if burning { candidates[0] } else { Vec3::ZERO };
    candidates.into_iter().find(|&mv| footing(mv) != Footing::Hazard).unwrap_or(hold)
}

/// How far ahead the footing oracles probe for a lift shaft — the same one-stride reach the hazard/water
/// probes use, so a dodge is judged by where it actually puts the body, not where it starts.
const PLAT_PROBE_AHEAD: f32 = 48.0;

/// The XY boxes of every lift currently raised *above* `origin`, grown by the player half-width (a body
/// that far outside the brush still overlaps it). Stepping into one parks the bot where the lift wants to
/// come down — it blocks the descent and resets the lower-timer — so the footing oracles demote such a
/// move exactly like water: dodge elsewhere when anything else is open, take it only if the alternative
/// is worse. The `origin.z` guard keeps a fight *on top of* the raised plat, or on the floor it delivers
/// to, unaffected. Mirrors `plat_statuses` (see `nav_build`); a map has a handful of plats, so the scan is
/// cheap enough to run per combat frame.
fn raised_plat_boxes(game: &GameState, origin: Vec3) -> Vec<(glam::Vec2, glam::Vec2)> {
    let Some(graph) = game.nav.graph.as_ref() else {
        return Vec::new();
    };
    let m = VEC_HULL_MAX.xy(); // the player's own half-width — the reach of the body we mustn't park there
    (0..graph.plat_count())
        .filter_map(|pi| {
            let p = graph.plat(pi);
            let ent = &game.entities[EntId(p.entity)];
            let raised = ent.in_use && ent.mover.state != crate::entity::MoverPhase::Bottom;
            let below = origin.z < ent.v.origin.z + ent.v.maxs.z;
            (raised && below).then(|| (p.fp_min - m, p.fp_max + m))
        })
        .collect()
}

/// Whether a step `d` from `feet` lands inside one of `plats`' footprints (see [`raised_plat_boxes`]).
fn steps_under_plat(plats: &[(glam::Vec2, glam::Vec2)], feet: Vec3, d: Vec3) -> bool {
    let p = (feet + d * PLAT_PROBE_AHEAD).xy();
    plats
        .iter()
        .any(|&(lo, hi)| crate::bot::in_footprint(p, lo, hi, 0.0))
}

/// Classify where a horizontal step `mv` from `feet` lands, for the combat/flee/dodge hazard ladders.
/// A zero move is Dry — holding ground is never a step into anything. A raised lift's shaft rates Wet,
/// not Hazard: somewhere to leave, not something that kills — move to open ground when there is any,
/// step in only if every other way is worse. Pure over the oracles (clip-hull solidity plus the
/// engine's `pointcontents`, the only hull that reports liquids).
fn step_footing(
    is_solid: &impl Fn(Vec3) -> bool,
    contents: &impl Fn(Vec3) -> f32,
    plats: &[(glam::Vec2, glam::Vec2)],
    feet: Vec3,
    mv: Vec3,
) -> Footing {
    let d = Vec3::new(mv.x, mv.y, 0.0).normalize_or_zero();
    if d == Vec3::ZERO {
        Footing::Dry
    } else if crate::hazard::hazard_ahead(is_solid, contents, feet, d).is_some() {
        Footing::Hazard
    } else if crate::hazard::water_ahead(is_solid, contents, feet, d) || steps_under_plat(plats, feet, d) {
        Footing::Wet
    } else {
        Footing::Dry
    }
}

/// A fleeing bot's move away from a threat at heading `away` (a horizontal unit vector), kept off
/// hazards: straight away when that's clear, else the safer perpendicular, else hold ground. The
/// returned bool is whether the bot may still hop to clear the blast — suppressed when every escape
/// is unsafe, since a jump surrenders the ground control needed to stop at an edge. Used by the
/// grenade-flee in [`grenade::grenade_tactics`].
fn safe_flee_move(game: &GameState, e: EntId, origin: Vec3, away: Vec3) -> (Vec3, bool) {
    let grounded = game.entities[e].v.flags.has(Flags::ONGROUND);
    let Some(bsp) = game.nav.bsp.as_ref() else {
        return (away * MOVE_SPEED, grounded);
    };
    let plats = raised_plat_boxes(game, origin);
    let host = game.host();
    let is_solid = |p: Vec3| bsp.is_solid(p);
    let contents = |p: Vec3| host.pointcontents(p);
    let feet = origin - Vec3::new(0.0, 0.0, 24.0);
    let footing = |d: Vec3| step_footing(&is_solid, &contents, &plats, feet, d);
    safe_flee_choice(&footing, away, grounded, is_burning(&game.entities[e].v))
}

/// The flee decision over a footing oracle, split out of [`safe_flee_move`] so the hazard ladder —
/// and the burning override in particular — is unit-testable. Straight away is the best escape vector,
/// taken unless it's a lethal hazard even through water (fleeing a blast beats staying dry); only when
/// that's blocked do we sidestep, preferring a dry perpendicular over a wet one. The final fallback is
/// the burning split: on safe ground, hazards on every side mean *hold* — a jump surrenders the ground
/// control needed to stop at an edge. But a bot already standing in the liquid has no safe edge to
/// protect (its footing is the damage), so it flees straight away regardless and keeps the hop, which
/// preserves flee momentum out of the blast and can clear a shallow basin lip.
fn safe_flee_choice(footing: &impl Fn(Vec3) -> Footing, away: Vec3, grounded: bool, burning: bool) -> (Vec3, bool) {
    if footing(away) != Footing::Hazard {
        return (away * MOVE_SPEED, grounded);
    }
    let perp = Vec3::new(-away.y, away.x, 0.0);
    let sides = [perp, -perp];
    if let Some(&d) = sides.iter().find(|&&d| footing(d) == Footing::Dry) {
        return (d * MOVE_SPEED, grounded);
    }
    if let Some(&d) = sides.iter().find(|&&d| footing(d) != Footing::Hazard) {
        return (d * MOVE_SPEED, grounded);
    }
    if burning {
        (away * MOVE_SPEED, grounded) // every side burns anyway — flee the blast, keep the hop
    } else {
        (Vec3::ZERO, false) // hazards on every side — hold, don't hop off the edge
    }
}

/// A dodging bot's move across an incoming projectile's travel line: `dodge` is the chosen lateral
/// heading (a horizontal unit vector, see [`dodge_side_sign`]), `away` the radial escape from the
/// blast point kept as a last resort for when the lane is walled off by hazards on both sides. Same
/// two-pass ladder as [`safe_combat_move`] — a candidate on dry ground wins outright, else the first
/// that isn't lethal (wading beats eating a rocket) — then the burning split from [`safe_flee_choice`]:
/// on safe ground, hazards every way mean hold rather than dodge off a ledge; already cooking in lava,
/// take the dodge anyway. Zero candidates are filtered because `footing(ZERO)` is Dry — a degenerate
/// radial would otherwise "win" the dry pass as a stand-still. The returned bool is whether footing
/// permits a hop; the caller gates it further on established lateral speed. Pure over the oracle.
fn safe_dodge_choice(
    footing: &impl Fn(Vec3) -> Footing,
    dodge: Vec3,
    away: Vec3,
    grounded: bool,
    burning: bool,
) -> (Vec3, bool) {
    let candidates = [dodge, -dodge, away];
    let mut open = candidates.into_iter().filter(|&d| d != Vec3::ZERO);
    if let Some(d) = open.clone().find(|&d| footing(d) == Footing::Dry) {
        return (d * MOVE_SPEED, grounded);
    }
    if let Some(d) = open.find(|&d| footing(d) != Footing::Hazard) {
        return (d * MOVE_SPEED, grounded);
    }
    if burning {
        (dodge * MOVE_SPEED, grounded) // every side burns anyway — cross the line, keep moving
    } else {
        (Vec3::ZERO, false) // hazards on every side — hold, don't dodge off the edge
    }
}

/// [`safe_dodge_choice`] over the live world: the same clip-hull/`pointcontents` oracles and raised-lift
/// footprints the combat and flee moves use. Mapless, the dodge goes through unfiltered.
fn safe_dodge_move(game: &GameState, e: EntId, origin: Vec3, dodge: Vec3, away: Vec3) -> (Vec3, bool) {
    let grounded = game.entities[e].v.flags.has(Flags::ONGROUND);
    let Some(bsp) = game.nav.bsp.as_ref() else {
        return (dodge * MOVE_SPEED, grounded);
    };
    let plats = raised_plat_boxes(game, origin);
    let host = game.host();
    let is_solid = |p: Vec3| bsp.is_solid(p);
    let contents = |p: Vec3| host.pointcontents(p);
    let feet = origin - Vec3::new(0.0, 0.0, 24.0);
    let footing = |d: Vec3| step_footing(&is_solid, &contents, &plats, feet, d);
    safe_dodge_choice(&footing, dodge, away, grounded, is_burning(&game.entities[e].v))
}

/// Bias a combat strafe away from a visible enemy's current horizontal aim line. `dodge_perp` is
/// the bot's normal lateral combat axis; once the bot is already to one side of a line passing close
/// by, keep moving farther to that side. Outside the danger tube (or behind the aim origin), retain
/// the ordinary time-varying sign.
fn aim_line_escape_sign(
    line_origin: Vec3,
    line_forward: Vec3,
    bot_origin: Vec3,
    dodge_perp: Vec3,
    default: f32,
) -> f32 {
    let forward = Vec3::new(line_forward.x, line_forward.y, 0.0).normalize_or_zero();
    if forward == Vec3::ZERO {
        return default;
    }
    let rel = Vec3::new(bot_origin.x - line_origin.x, bot_origin.y - line_origin.y, 0.0);
    let along = rel.dot(forward);
    if along <= 0.0 {
        return default;
    }
    let off = rel - forward * along;
    if off.length() >= 96.0 {
        return default;
    }
    let side = off.dot(dodge_perp);
    if side.abs() < 1.0 {
        default
    } else {
        side.signum()
    }
}

/// Whether this entity is standing in lava or slime deep enough to take damage. The game's
/// `apply_liquid_damage` burns at `waterlevel >= 1` (feet touching) for a lava/slime `watertype`, with
/// no deeper gate — so this is the exact condition under which a bot must keep moving out of the pool
/// rather than hold ground on it. Reads the engine-populated ABI fields, like [`sense`].
pub(crate) fn is_burning(v: &EntVars) -> bool {
    v.waterlevel >= 1.0 && (v.watertype.is(Content::Lava) || v.watertype.is(Content::Slime))
}

/// Verdict on one line-of-fire trace for a splash projectile. Clear when the shot reached the enemy,
/// or flew *unobstructed* (an open shot — an intended miss beside a strafer is fine), or stopped
/// essentially at the aim point (near-target splash still lands). Blocked only when it strikes a wall
/// short of the target *and inside our own blast radius* — the corner self-splash this whole gate
/// exists to stop (the rocket blast is [`GRENADE_BLAST_RADIUS`]; attacker self-damage is merely
/// halved). `impact` is where the trace stopped; `obstructed` is whether it hit anything at all.
fn lof_verdict(origin: Vec3, aim: Vec3, impact: Vec3, hit_enemy: bool, obstructed: bool) -> bool {
    if hit_enemy || !obstructed {
        return true;
    }
    (impact - aim).length() <= LINE_OF_FIRE_SLACK && (impact - origin).length() >= GRENADE_BLAST_RADIUS
}

/// The enemy's kinematics for one engagement frame — snapshotted once so the ballistics and aim
/// helpers share a single read of the target's motion state. A grounded target is led horizontally;
/// a swimmer moves freely but does *not* free-fall (no gravity term); an airborne one is on a
/// gravity parabola solved exactly.
struct Target {
    org: Vec3, // entity origin
    eye: Vec3, // origin + VEC_VIEW_OFS
    vel: Vec3,
    dist: f32,      // eye-to-eye separation, clamped ≥ 1
    swimming: bool, // waterlevel ≥ 2 — led in full 3D, no gravity term
    airborne: bool, // neither grounded nor swimming — on a gravity parabola
}

/// A validated grenade-arc solution: the pre-solved launch view angles and the meet point. Produced
/// by [`plan_ballistics`] (airborne intercept or grounded lead-lob), consumed by the aim path.
#[derive(Clone, Copy)]
struct GrenadeSol {
    look: Vec3,
    meet: Vec3,
}

/// Projectile planning computed inside one immutable BSP borrow, handed back as plain data.
struct BallisticPlan {
    /// Where an airborne enemy would touch down (time, point), so the rocket lead clamps at the
    /// floor instead of aiming through it — and, once they're down by the time a rocket could reach
    /// them, the floor the shot is put into rather than the body (see [`aim_solution`]).
    land: Option<(f32, Vec3)>,
    /// A *validated* airborne grenade intercept: still airborne at the meet, far enough that the
    /// blast doesn't catch us, and a real bounce sim confirms the arc reaches them.
    air_gl: Option<GrenadeSol>,
    /// The RL-less grounded lead-lob, arc-cleared like the combo's `try_start`.
    gl_ground: Option<GrenadeSol>,
}

/// The resolved shot for the fire gate: which weapon, where it aims, the clean angles to there, the
/// projectile spawn height, and whether it must land a direct hull hit (vs. leaning on splash).
/// Self-contained — it carries the target and the frame's origin too, so the gate can be judged
/// later in `emit` against the settled view without re-deriving the frame's combat context.
#[derive(Clone, Copy)]
pub(crate) struct Shot {
    choice: WeaponChoice,
    aim: Vec3,
    clean: Vec3,
    muzzle_base: Vec3,
    gate_direct: bool,
    /// The enemy this shot is solved against — the trace-hit identity the line-of-fire rays accept.
    enemy: EntId,
    /// The bot's origin at solve time: the muzzle/launch point and the self-splash range come off it.
    origin: Vec3,
}

/// Solve the geometry-aware projectile options against the real BSP hull, all inside one immutable
/// navmesh borrow (see the field docs on [`BallisticPlan`]). The grenade arcs are only bothered with
/// when they could actually be chosen — the RL is the better weapon whenever it's in hand, so a
/// grenade intercept is a *fallback* (no RL, GL stocked). `idle`/`combos_on` gate the grounded lob
/// the way [`engage`] did: the lob→shoot combo owns grounded grenade offence when shootable grenades
/// are on, so engage only ground-lobs as the fallback when the combo is off — unless the GL is our
/// only weapon (`gl_primary`), where the combo bails and engage is the sole driver.
fn plan_ballistics(
    game: &GameState,
    origin: Vec3,
    tgt: &Target,
    gravity: f32,
    inv: Loadout,
    idle: bool,
    combos_on: bool,
) -> BallisticPlan {
    // RL and GL share the rocket ammo pool.
    let has_rl = inv.has(Items::ROCKET_LAUNCHER) && inv.rockets >= 1.0;
    let has_gl = inv.has(Items::GRENADE_LAUNCHER) && inv.rockets >= 1.0;
    let gl_primary = inv.gl_primary();
    let mut plan = BallisticPlan { land: None, air_gl: None, gl_ground: None };
    let Some(bsp) = game.nav.bsp.as_ref() else {
        return plan;
    };
    let trace = |a: Vec3, b: Vec3| bsp.hull1_trace(a, b);
    if tgt.airborne {
        let land = fall_land(&trace, tgt.org, tgt.vel, gravity, grenade::GL_FUSE);
        plan.land = land;
        // Only solve the grenade arc when it could actually be chosen — a fallback for when the RL
        // is gone (`has_gl && !has_rl`).
        if has_gl && !has_rl && idle {
            if let Some((look, t, meet)) = grenade::solve_air_intercept(origin, tgt.org, tgt.vel, gravity) {
                let airborne_at_meet = land.is_none_or(|(t_land, _)| t < t_land);
                // Keep the blast off ourselves: the meet must sit a full blast radius away.
                let safe_range = (meet - origin).length() >= GRENADE_BLAST_RADIUS;
                let enemy_at =
                    |tt: f32| ballistic_pos(tgt.org, tgt.vel, gravity, land, tt) + Vec3::new(0.0, 0.0, 4.0);
                let sim = grenade::simulate_bounce(&trace, origin, grenade::launch_velocity(look), gravity, &enemy_at);
                if airborne_at_meet && safe_range && sim.hit_enemy {
                    plan.air_gl = Some(GrenadeSol { look, meet });
                }
            }
        }
    } else if !tgt.swimming && has_gl && !has_rl && idle && (!combos_on || gl_primary) {
        // Grounded lead-lob (RL gone, GL stocked, combo off *or* GL is our only weapon): two-round
        // lead so a strafer stays in the blast, then verify the arc actually clears geometry onto the
        // led point — a purely ballistic solve would happily hurl the grenade into a low ceiling and
        // bounce it back onto us.
        let feet = tgt.org - Vec3::new(0.0, 0.0, 24.0);
        let vel_xy = Vec3::new(tgt.vel.x, tgt.vel.y, 0.0);
        if let Some((look, _flight, led)) = grenade::solve_ground_lead(origin, feet, vel_xy, gravity) {
            let clear = crate::navmesh::arc_land(bsp, origin, grenade::launch_velocity(look), gravity)
                .is_some_and(|(land_pt, _, _)| (land_pt.xy() - led.xy()).length() < grenade::LOB_LAND_TOL);
            if clear {
                plan.gl_ground = Some(GrenadeSol { look, meet: led });
            }
        }
    }
    plan
}

/// The aim point, the clean firing angles to it, and whether the shot needs a direct hull hit
/// (`gate_direct`, vs. leaning on splash). Pure ballistics. A lobbed grenade fires along its
/// pre-solved launch view (the meet point is only for aim memory and the fire gate). Other
/// projectiles solve the true intercept — where the enemy *will be* when the shot arrives; airborne
/// targets ride the gravity-displaced, floor-clamped parabola from the muzzle (falling back to a
/// linear lead if the fixed point can't settle), aimed at the hull centre (+4) for the most
/// direct-hit margin — *unless* a rocket's target is down, or all but down, by the time it could get
/// there, in which case it aims at the floor under them and leans on the splash. Grounded/swimming
/// targets get a linear lead from the eye (a grounded RL strafer gets the same shin-drop so a near
/// miss becomes floor splash, but nailguns need a direct hit). Hitscan aims straight at the eye, led
/// only by `lead`.
///
/// `lead` is the latency the shot is fired across — zero inside a server, and a network client's
/// round trip otherwise (see [`GameState::aim_lead`]). It's simply added to the flight time, because
/// that is exactly what it is: time in which the target moves and we cannot see it. That makes it a
/// hitscan's *whole* lead, which is why the last branch has one at all.
fn aim_solution(
    choice: WeaponChoice,
    tgt: &Target,
    my_eye: Vec3,
    muzzle_base: Vec3,
    gravity: f32,
    plan: &BallisticPlan,
    lead: f32,
) -> (Vec3, Vec3, bool) {
    let s = choice.projectile_speed;
    if choice.grenade_arc {
        // Exactly one of air_gl/gl_ground is set when `grenade_arc` holds, and it aligns with
        // `airborne` (air intercept ⇒ airborne). Fire straight along the solved view.
        let sol = plan.air_gl.or(plan.gl_ground).expect("grenade_arc ⇒ a grenade solution was validated");
        (sol.meet, sol.look, tgt.airborne)
    } else if s > 0.0 {
        if tgt.airborne {
            let seed =
                intercept_time(tgt.org - muzzle_base, tgt.vel, s).unwrap_or((tgt.org - muzzle_base).length() / s);
            // The parabola is evaluated `lead` further along than the flight alone: an airborne
            // target keeps falling through our latency, so the meet point drops with it.
            let pos_at = |t: f32| ballistic_pos(tgt.org, tgt.vel, gravity, plan.land, t + lead);
            // Fallback (fixed point didn't settle — a target falling away near projectile speed):
            // the linear-seed flight time evaluated on the *clamped* `pos_at`, so a target that lands
            // mid-flight still resolves to the landing spot rather than a point below the floor.
            let (t, meet) = ballistic_intercept(muzzle_base, &pos_at, s, seed).unwrap_or((seed, pos_at(seed)));
            // Down, or as good as, before the rocket can get there ([`FLOOR_SPLASH_LEEWAY`]): put the
            // shot in the floor under them rather than into the body. Against an enemy who dodges what
            // is aimed *at* them, the ground is the better target — stand on it and eat the splash, or
            // jump off it and get catapulted, which only hands us the shot we want next. The blast is
            // instantaneous, so it goes under where they'll *be* when it arrives (`meet`, which past
            // touchdown is the landing spot with the run-on already in it), at the landing floor:
            // `land_pos` is the resting *origin*, so −16 is the shin — the grounded strafer's
            // floor-splash convention below, keeping 8u of clearance so a shallow shot doesn't graze
            // the ground short of the spot. Rocket only: a nail in the floor is a wasted shot.
            let floor_splash = plan
                .land
                .filter(|_| choice.weapon == Weapon::RocketLauncher)
                .filter(|&(t_land, _)| t + lead >= t_land - FLOOR_SPLASH_LEEWAY)
                .map(|(_, land_pos)| Vec3::new(meet.x, meet.y, land_pos.z - 16.0));
            let aim = floor_splash.unwrap_or(meet + Vec3::new(0.0, 0.0, 4.0));
            (aim, angles_to(muzzle_base, aim), floor_splash.is_none())
        } else {
            // A swimmer is led in full 3D with no gravity term (water isn't free-fall).
            let pred_vel = if tgt.swimming {
                tgt.vel
            } else {
                Vec3::new(tgt.vel.x, tgt.vel.y, 0.0)
            };
            let t = intercept_time(tgt.eye - my_eye, pred_vel, s).unwrap_or(tgt.dist / s);
            let mut aim = tgt.eye + pred_vel * (t + lead);
            if !tgt.swimming && choice.weapon == Weapon::RocketLauncher && pred_vel.xy().length() > 150.0 {
                aim.z -= 38.0; // eye (+22 over origin) → shin (−16)
            }
            (aim, angles_to(my_eye, aim), choice.weapon != Weapon::RocketLauncher)
        }
    } else {
        // Hitscan: no flight time, so `lead` is the whole of it — nothing inside a server, and the
        // round trip on a server that won't rewind for us.
        let aim = tgt.eye + Vec3::new(tgt.vel.x, tgt.vel.y, 0.0) * lead;
        (aim, angles_to(my_eye, aim), false)
    }
}

/// The skill-scaled *drifting* aim error added to the view this frame, and the "last seen" memory
/// update. The error wanders smoothly toward a periodically resampled offset (never fresh white
/// noise per frame — that reads as view jitter), so misses sweep past the target and drift back like
/// human tracking error; pitch error is kept smaller than yaw (vertical mouse control is steadier);
/// skill 7 ⇒ error ≈ 0. The base half-range shrinks with skill, then widens with three human
/// tracking factors so snap-shots are looser than a settled duel: convergence (loose on first sight,
/// tightening over ~1.5s of continuous LoS), own motion, and target crossing speed.
fn aim_error(game: &mut GameState, e: EntId, now: f32, skill: f32, aim: Vec3, my_eye: Vec3, tgt: &Target) -> Vec3 {
    let base_spread = (7.0 - skill).max(0.0);
    let visible_for = {
        let vs = game.entities[e].bot.percept.vis_since;
        if vs > 0.0 {
            now - vs
        } else {
            0.0
        }
    };
    let own_speed = game.entities[e].v.velocity.xy().length();
    let to_enemy = tgt.eye - my_eye;
    let perp_speed = {
        let los_dir = to_enemy / tgt.dist;
        (tgt.vel - los_dir * tgt.vel.dot(los_dir)).length() // target motion across the line of fire
    };
    let spread = base_spread * spread_scale(visible_for, own_speed, perp_speed, tgt.dist);
    let frametime = game.globals.frametime;
    if now >= game.entities[e].bot.aim.err_until {
        let (r1, r2, r3) = (game.random(), game.random(), game.random());
        let b = &mut game.entities[e].bot;
        b.aim.err_target = Vec3::new((r1 - 0.5) * spread, (r2 - 0.5) * 2.0 * spread, 0.0);
        b.aim.err_until = now + 0.3 + r3 * 0.3;
    }
    let b = &mut game.entities[e].bot;
    let t = (4.0 * frametime).min(1.0);
    b.aim.err = b.aim.err + (b.aim.err_target - b.aim.err) * t;
    // Remember where the enemy is while we can see them, for the hold-the-angle behavior.
    b.seen.at = aim;
    b.seen.time = now;
    b.aim.err
}

/// How fast the smoothed rate estimate chases the raw one (1/s), giving a time constant near 0.1s —
/// long enough to average out the per-frame noise in a finite-difference rate, short enough that the
/// lead is already there when a strafer reverses.
const LEAD_RATE_SMOOTH: f32 = 10.0;

/// Feed-forward lead for the aim spring. The spring tracks a moving solution with a steady-state lag
/// of 2·rate/ω, so on a constant strafer the crosshair would trail forever. Estimate how fast the
/// solution is moving (from last frame's clean angles) and aim ahead by the expected lag —
/// skill-scaled, so skill 7 locks onto strafers while low skill keeps trailing them. A jump too fast
/// for human tracking is treated as a discontinuity (target/weapon switch or teleport), not motion,
/// so no phantom slew is fed forward. The estimate is smoothed before it reaches the view
/// ([`LEAD_RATE_SMOOTH`]): the raw per-frame rate is noisy enough that feeding it straight through
/// shakes the crosshair, which is what a spectator sees as the bot's aim buzzing left and right.
fn feed_forward(game: &mut GameState, e: EntId, now: f32, skill: f32, clean: Vec3) -> Vec3 {
    let frametime = game.globals.frametime;
    let b = &mut game.entities[e].bot;
    let dt = now - b.aim.look_prev_time;
    let raw = if b.aim.look_prev_time > 0.0 && dt > 1e-3 && dt < 0.25 {
        Vec3::new(wrap180(clean.x - b.aim.look_prev.x) / dt, wrap180(clean.y - b.aim.look_prev.y) / dt, 0.0)
    } else {
        Vec3::ZERO // stale/first sample (just acquired the target) — no estimate yet
    };
    // Genuine crossing tops out near 230°/s even up close, well under this 360°/s discontinuity cut.
    let rate = if raw.x.abs() > 360.0 || raw.y.abs() > 360.0 {
        Vec3::ZERO
    } else {
        Vec3::new(raw.x.clamp(-180.0, 180.0), raw.y.clamp(-180.0, 180.0), 0.0)
    };
    b.aim.look_prev = clean;
    b.aim.look_prev_time = now;
    // Chase the estimate rather than take it raw. A one-frame difference of the solution angle is the
    // tracking motion plus noise — in a close duel it swings by well over a hundred deg/s between
    // frames, and a sample rejected as a discontinuity drops it to zero outright. The lead scales all
    // of that by 2/ω (±36° at skill 7), so feeding it straight to the view snaps the crosshair frame
    // to frame and the aim visibly shakes — worst airborne, where a duel's angular rates peak.
    // Smoothing over ~0.1s still tracks a strafer's swing while averaging the noise away.
    let t = (LEAD_RATE_SMOOTH * frametime).min(1.0);
    b.aim.rate += (rate - b.aim.rate) * t;
    b.aim.rate * (2.0 / aim_omega(skill)) * (skill / 7.0)
}

/// The world-space combat movement: hold a preferred range and strafe to dodge, retreating when
/// hurt. Opponent modeling can flip this to *press* — if the enemy is believed finishable (belief
/// fresh, and we're not ourselves critical) the bot closes to finish rather than holding range;
/// `press` is false when modeling is off, leaving the range logic unchanged. Grounded near a hazard
/// the move is filtered so the bot won't strafe or backpedal into lava/slime or off a ledge (the
/// probes reuse the offensive-shove oracles — clip-hull solidity plus the engine's `pointcontents`,
/// the only hull that reports liquids), nor orbit a fight into a raised lift's shaft, where its body
/// would hold the lift up; airborne or map-less it's the original blind composition.
fn combat_move(game: &mut GameState, e: EntId, enemy: EntId, now: f32, origin: Vec3, to_enemy: Vec3) -> Vec3 {
    let health = game.entities[e].v.health;
    let default_strafe = if ((now * 0.9) + e.0 as f32).sin() >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let press = game.opponent_est(e, enemy, now).is_some_and(|est| {
        press_advantage(health, crate::bot::model::est_strength(&est, now), now - est.last_update)
    });
    let retreat_health = if press { LOW_HEALTH / 2.0 } else { LOW_HEALTH };
    let dist = to_enemy.length().max(1.0);
    let want_forward = if health < retreat_health || dist < PREFERRED_RANGE - 100.0 {
        -MOVE_SPEED // back off (too hurt, or inside self-splash range)
    } else if dist > PREFERRED_RANGE + 100.0 || press {
        MOVE_SPEED // close in — normally only when far, but also to finish a pressed kill
    } else {
        0.0 // hold and strafe
    };
    let dir = Vec3::new(to_enemy.x, to_enemy.y, 0.0).normalize_or_zero();
    let perp = Vec3::new(-dir.y, dir.x, 0.0);
    let enemy_eye = game.entities[enemy].v.origin + VEC_VIEW_OFS;
    let enemy_forward = angle_vectors(game.entities[enemy].v.v_angle).0;
    let strafe_sign = aim_line_escape_sign(enemy_eye, enemy_forward, origin, perp, default_strafe);
    let grounded_self = game.entities[e].v.flags.has(Flags::ONGROUND);
    let burning = is_burning(&game.entities[e].v);
    // Standing in the fire outranks the duel — for the feet. The aim and the trigger above are
    // untouched, so the bot fights its way out rather than breaking off; it just stops choosing where
    // to stand by where the enemy is.
    //
    // This is the whole of the "bots throw their lives away in lava for no reason" case. The guard
    // below picks between *combat's* candidates by probing 40-72u around the body, and deep in a pool
    // every one of those reads lava — so it falls through to the wanted move and the bot circle-strafes
    // its opponent, in the lava, until it dies. Its route knows where the bank is, but a fight
    // overwrites `move_world` outright, so the routing gradient never reaches the feet. The burn goal
    // has already fixed the nearest shore (`run_bot`); walk at it.
    if burning {
        let shore = game.entities[e].bot.burn.target;
        let out = Vec3::new(shore.x - origin.x, shore.y - origin.y, 0.0).normalize_or_zero();
        if shore != Vec3::ZERO && out != Vec3::ZERO {
            return out * MOVE_SPEED;
        }
    }
    match (grounded_self, game.nav.bsp.as_ref()) {
        (true, Some(bsp)) => {
            let plats = raised_plat_boxes(game, origin);
            let host = game.host();
            let is_solid = |p: Vec3| bsp.is_solid(p);
            let contents = |p: Vec3| host.pointcontents(p);
            let feet = origin - Vec3::new(0.0, 0.0, 24.0);
            let footing = |mv: Vec3| step_footing(&is_solid, &contents, &plats, feet, mv);
            safe_combat_move(&footing, dir, perp, want_forward, strafe_sign, burning)
        }
        _ => dir * want_forward + perp * (strafe_sign * MOVE_SPEED),
    }
}

/// Whether `view` puts the shot on the spot: projectiles by the predicted *miss distance* at intercept
/// range (a direct-hit shot needs the hull, a splash shot leans on the blast), hitscan by a per-weapon
/// cone — the lightning beam tight, the shotguns/axe looser — plus low-skill leniency. Pure over the
/// resolved shot, so the tolerance model is unit-testable without a live frame.
fn shot_on_target(view: Vec3, shot: &Shot, skill: f32) -> bool {
    let Shot { choice, aim, clean, muzzle_base, gate_direct, origin, .. } = *shot;
    if choice.projectile_speed > 0.0 {
        let launch = if choice.grenade_arc { origin } else { muzzle_base };
        let range = (aim - launch).length().max(1.0);
        miss_distance(view, clean, range) <= fire_tolerance(skill, gate_direct)
    } else {
        let base_cone = if choice.weapon == Weapon::Lightning { 2.5 } else { 5.0 };
        let cone = base_cone + (7.0 - skill);
        let dp = wrap180(view.x - clean.x);
        let dy = wrap180(view.y - clean.y);
        (dp * dp + dy * dy).sqrt() <= cone
    }
}

/// Whether to pull the trigger. Fire only when the crosshair is on the spot *and* the line of fire is
/// clear. `view` is the *settled* view this frame's usercmd carries — the exact angles the shot flies
/// along — so the gate judges the shot it is actually about to take (see [`fire_pending`], which runs
/// this from `emit` after the aim spring). A rocket also traces its real muzzle→aim line *twice* (the
/// steady `clean` ray and the ray it will actually fly) and needs both clear — the corner self-splash
/// fix. Fire is held while a switch to the GL is still pending, so the held gun doesn't loose along
/// the ~18°-high grenade-loft view.
fn fire_gate(game: &mut GameState, e: EntId, skill: f32, view: Vec3, shot: &Shot) -> bool {
    let Shot { choice, aim, clean, gate_direct: _, enemy, origin, .. } = *shot;
    let s = choice.projectile_speed;
    let on_target = shot_on_target(view, shot, skill);
    let switching_to_gl = choice.grenade_arc && game.entities[e].v.weapon != Weapon::GrenadeLauncher;
    // Muzzle matches `w_fire_rocket` (origin + forward·8 + 16 up), taken from each ray's own forward.
    // A grenade arc keeps its own geometry check (bounce sim / arc_land) and skips this straight-line
    // trace, which its lofted path would spuriously fail.
    let mut ray_clear = |ang: Vec3| {
        let fwd = angle_vectors(ang).0;
        let muzzle = crate::weapons::rocket_muzzle(origin, fwd);
        let end = muzzle + fwd * (aim - muzzle).length();
        let tr = game.traceline(muzzle, end, false, e);
        let impact = muzzle + (end - muzzle) * tr.fraction;
        lof_verdict(origin, aim, impact, tr.ent == enemy, tr.fraction < 1.0)
    };
    let lof_clear = if choice.grenade_arc {
        true
    } else if s > 0.0 {
        ray_clear(clean) && ray_clear(view)
    } else {
        true // hitscan: the eye-ray LoS above already governs the shot
    };
    let explosive = matches!(choice.weapon, Weapon::RocketLauncher | Weapon::GrenadeLauncher);
    let my_team = game.entities[e].mode_p.team;
    let friendly_clear = !explosive || !teammate_in_blast(game, e, my_team, aim);
    let self_clear = !explosive || own_explosive_aim_safe(game, e, (aim - origin).length());
    // If the safety fallback selected a direct gun while an explosive is still physically in hand,
    // wait for that switch to land. Otherwise a swallowed impulse could loose the old quad rocket
    // along angles intended for the replacement gun.
    let held = game.entities[e].v.weapon;
    let switching_from_explosive = held != choice.weapon
        && matches!(held, Weapon::RocketLauncher | Weapon::GrenadeLauncher);
    on_target
        && lof_clear
        && friendly_clear
        && self_clear
        && !switching_to_gl
        && !switching_from_explosive
}

/// The combat fire decision, run from `emit` once the aim spring has settled — `view` is the very
/// angle the projectile will fly along this frame, so the gate approves the shot it actually takes
/// rather than the one last frame's view would have taken. Judging it a frame early let a marginal
/// aim pass and the shot leave wide: the spring keeps moving after the gate looks.
///
/// Deliberately *no* `attack_finished` check. Holding +attack under cooldown is free — the engine's
/// `w_weapon_frame` swallows the press and paces refire — and the LG beam and the nail streams need
/// `button0` held continuously to sustain, so dropping the button on cooldown frames would stutter
/// them. Every frame is a fresh decision regardless, so a held button is never a stale one.
pub(crate) fn fire_pending(game: &mut GameState, e: EntId, skill: f32, view: Vec3, shot: &Shot) -> bool {
    fire_gate(game, e, skill, view, shot)
}

/// Overlay combat onto the frame's decisions. `look` is the desired view (smoothed downstream by
/// the aim spring in `bot.rs`); `move_world` is the desired world-space velocity — the two are
/// independent, so the bot can run one way while looking another. With line of sight it aims
/// (leading the target, plus a smoothly drifting skill-scaled error), fights for range, and fires;
/// having *recently* lost sight it holds the angle where the enemy vanished while navigation keeps
/// it moving; otherwise it leaves the navigation view/movement untouched.
pub(crate) fn engage(
    game: &mut GameState,
    e: EntId,
    enemy: EntId,
    origin: Vec3,
    now: f32,
    cmd: &mut BotCmd,
) {
    let my_eye = origin + VEC_VIEW_OFS;
    let enemy_org = game.entities[enemy].v.origin;
    let enemy_eye = enemy_org + VEC_VIEW_OFS;
    let enemy_vel = game.entities[enemy].v.velocity;

    // Line of sight? Trace to the enemy's eyes, ignoring ourselves. Clear if we hit the enemy or
    // nothing at all — unless the enemy is a stale network shadow (teleported or ducked out of PVS,
    // frozen where we last saw it), in which case that clear line is to a ghost, not a target, and we
    // must not fire down it. Same guard perception uses, so aim and trigger agree on who's really there.
    let tr = game.traceline(my_eye, enemy_eye, false, e);
    let los = (tr.ent == enemy || tr.fraction > 0.95) && !game.net_shadow_stale(enemy, now);
    if !los {
        // No shooting through walls; navigation keeps driving the movement. But if we saw the
        // enemy moments ago, hold the angle where they disappeared instead of snapping the view
        // back to the route — the human "holding the corner" look, and it kills the nav↔enemy
        // view flip-flop while line of sight flickers at an edge.
        let b = &game.entities[e].bot;
        if b.seen.time > 0.0 && now - b.seen.time < HOLD_ANGLE_TIME {
            cmd.look = angles_to(my_eye, b.seen.at);
        }
        return;
    }

    let to_enemy = enemy_eye - my_eye;
    let dist = to_enemy.length().max(1.0);
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let gravity = game.host().cvar(c"sv_gravity");

    // Target motion state, snapshotted once for the ballistics/aim helpers. A grounded target is led
    // horizontally; a swimmer moves freely but does *not* free-fall; an airborne one rides a parabola.
    let grounded = game.entities[enemy].v.flags.has(Flags::ONGROUND);
    let swimming = game.entities[enemy].v.waterlevel >= 2.0;
    let tgt = Target {
        org: enemy_org,
        eye: enemy_eye,
        vel: enemy_vel,
        dist,
        swimming,
        airborne: !grounded && !swimming,
    };

    // Weapon inventory relevant to the projectile choice (RL and GL share the rocket ammo pool).
    let inv = Loadout::of(&game.entities[e].v);
    // The lob→shoot combo (`grenade::grenade_combo`, run after us) owns grounded grenade offence when
    // shootable grenades are enabled; `idle`/`combos_on` tell `plan_ballistics` whether engage is the
    // driver (see its doc). The airborne intercept is engage-exclusive.
    let idle = game.entities[e].bot.grenade.phase == GrenadePhase::Idle;
    let combos_on = game.host().cvar_bool(c"rtx_shootable_grenades");

    // Geometry-aware projectile planning, all inside one immutable BSP borrow, handed back as data.
    let plan = plan_ballistics(game, origin, &tgt, gravity, inv, idle, combos_on);

    // Underwater the lightning gun discharges — dumping every cell as a self-lethal radius blast
    // (`w_fire_lightning`, at `waterlevel > 1`). A bot only does that on purpose when it trades for a
    // believed quad carrier or ≥2 enemies (`discharge_worth_it`); otherwise the pick steers clear of
    // the LG entirely, and the fire gate below refuses to shoot one still in hand.
    let underwater = game.entities[e].v.waterlevel > 1.0;
    let discharge = underwater
        && inv.fed(Weapon::Lightning)
        && discharge_worth_it(inv.cells, &discharge_victims(game, e, origin, now));

    // Weapon for the range. The grenade choice is keyed solely on inventory (a validated arc, RL
    // unavailable) — not a clock or geometry threshold — so it can't flip mid-jump and re-slew the
    // aim off the shot; RL/GL share ammo, so the only transition is the pool running dry, grounding
    // both at once. Midair's RL-only loadout never reaches the grenade path.
    // The opponent model's finish read: is this enemy believed to be on a finishable stack, freshly?
    // The same belief the movement `press` uses (`combat_move`), minus its own-health gate — a hurt
    // bot still wants the *reliable* finishing weapon. When set, `choose_weapon` swaps a dodgeable
    // rocket at range for a hitscan direct hit. `None` (modeling off, or a client with no health
    // belief yet) ⇒ `false` ⇒ the pick is unchanged.
    let finishable = game.opponent_est(e, enemy, now).is_some_and(|est| {
        now - est.last_update < FINISH_FRESH && crate::bot::model::est_strength(&est, now) < FINISH_STACK
    });
    let mut choice = if discharge {
        WeaponChoice::of(Weapon::Lightning)
    } else {
        choose_weapon(inv, dist, plan.air_gl.is_some(), plan.gl_ground.is_some(), underwater, finishable)
    };
    // Do not even select an explosive weapon when a teammate occupies the target blast or its own
    // projected splash is too costly (especially KTX's enlarged quad caution zone). The fire gate
    // repeats both checks at the fully led aim point to catch motion between planning and firing;
    // this earlier branch gives the bot a useful non-splash fallback instead of silence.
    let explosive = matches!(choice.weapon, Weapon::RocketLauncher | Weapon::GrenadeLauncher);
    let my_team = game.entities[e].mode_p.team;
    if explosive
        && (teammate_in_blast(game, e, my_team, tgt.org)
            || !own_explosive_aim_safe(game, e, dist))
    {
        if let Some(direct) = safe_direct_choice(inv, dist, underwater) {
            choice = direct;
        }
    }
    // Switch weapon only when we don't already hold the desired one (setting `impulse` re-runs
    // W_ChangeWeapon each frame otherwise).
    if game.entities[e].v.weapon != choice.weapon {
        cmd.impulse = choice.impulse;
        // Make the finish read observable live (`rtx_bot_debug`): log only when it actually diverts the
        // pick from what range + inventory alone would choose — the "he's low, hit him with something
        // that lands" swap. Throttled by the switch itself, and off the hot path unless debugging.
        //
        // The belief prints *beside the truth it is a guess at*: the failure this catches is never the
        // gate arithmetic, it's the model quietly disagreeing with the world (a stale baseline, a missed
        // reset, over-modelled damage), and a believed-strength number alone cannot show that. Both
        // sides come from the frame that took the decision, so they're directly comparable.
        if finishable && game.host().cvar_bool(c"rtx_bot_debug") {
            let plain = choose_weapon(inv, dist, plan.air_gl.is_some(), plan.gl_ground.is_some(), underwater, false);
            if plain.weapon != choice.weapon {
                let (bh, ba, be, age) = game.opponent_est(e, enemy, now).map_or((-1.0, -1.0, -1.0, -1.0), |est| {
                    (est.health, est.armor_value, crate::bot::model::est_strength(&est, now), now - est.last_update)
                });
                let v = &game.entities[enemy].v;
                let (rh, ra, rt) = (v.health, v.armorvalue, v.armortype);
                let real = crate::bot::goals::total_strength(rh, ra, rt);
                game.host().conprint(&crate::game::cstring(&format!(
                    "rtx bot{}: finishing with {:?} (range pick was {:?}) at {dist:.0}u — \
                     belief H{bh:.0}/A{ba:.0} E{be:.0} age {age:.1}s | real H{rh:.0}/A{ra:.0}@{rt:.1} E{real:.0}\n",
                    e.0, choice.weapon, plain.weapon,
                )));
            }
        }
    }

    // Aim point and clean firing angles (pure ballistics). `gate_direct` marks a shot that needs a
    // direct hull hit vs. one that can lean on splash.
    let muzzle_base = origin + Vec3::new(0.0, 0.0, 16.0); // rocket/grenade spawn height (w_fire_rocket)
    let lead = game.aim_lead(choice.projectile_speed > 0.0);
    let (aim, clean, gate_direct) = aim_solution(choice, &tgt, my_eye, muzzle_base, gravity, &plan, lead);

    // Compose the view: clean angles + feed-forward lead + drifting skill error. `aim_error` also
    // records the "last seen" spot/time for the hold-the-angle behavior.
    let err = aim_error(game, e, now, skill, aim, my_eye, &tgt);
    let ff = feed_forward(game, e, now, skill, clean);
    cmd.look = Vec3::new(clean.x + ff.x + err.x, clean.y + ff.y + err.y, 0.0);

    // Movement (world-space) then the fire decision.
    cmd.move_world = combat_move(game, e, enemy, now, origin, to_enemy);
    cmd.buttons &= !BUTTON_JUMP; // don't bunny-hop while dueling

    let holding_lg = game.entities[e].v.weapon == Weapon::Lightning;
    if discharge {
        // Deliberate discharge: radius damage needs no aim, so it never goes through the gate — but
        // wait until the LG is actually in hand so the still-held weapon doesn't loose first (the
        // switch takes a frame).
        if holding_lg {
            cmd.buttons |= BUTTON_ATTACK;
        }
    } else if underwater && holding_lg {
        // Still holding the LG underwater with no worthwhile discharge (a mid-fight dive, or the
        // switch to another gun hasn't landed yet): never pull the trigger — it would discharge.
    } else {
        // Arm the shot; `emit` gates it after the aim spring, against the view it will fly along.
        cmd.shot = Some(Shot { choice, aim, clean, muzzle_base, gate_direct, enemy, origin });
    }
}

// --- live projectile survival ------------------------------------------------------------------

/// Predicted lateral miss (units) below which the closest-approach offset carries no side — about half
/// the player hull's width. A well-aimed rocket lands inside this, where the offset is aim noise that
/// flips sign frame to frame rather than a side worth widening.
const DODGE_MISS_EPS: f32 = 16.0;

/// Lateral speed (units/s) that counts as a strafe already under way and worth reinforcing. Above the
/// ~30u/s an airborne bot can wish itself sideways and above ground friction jitter, well under a run.
const DODGE_VEL_EPS: f32 = 50.0;

/// Speed (units/s) along the chosen dodge before a hop may extend it. Ground runs cap at `sv_maxspeed`
/// (320), so this is a committed side-dodge carrying real ground speed into the air — a QuakeWorld
/// side-jump. Below it a jump is a standing pogo: it trades ground acceleration for the ~30u/s air-wish
/// cap and builds no separation at all, which is exactly the bug this gate exists to kill. Absolute, not
/// a fraction of [`MOVE_SPEED`] — that is a wish magnitude (800), not a speed the bot can ever reach.
const DODGE_HOP_SPEED: f32 = 200.0;

/// Which side of a linear projectile's travel line to dodge toward, as a sign on the lateral axis.
/// A predicted miss wide enough to be real is widened — the shot is already going to one side, so
/// commit to that side. Failing that, an established strafe carries on: reading the bot's own lateral
/// velocity makes the choice self-reinforcing, holding a direction across frames without new state.
/// Failing both (a dead-on shot at a standing bot), a stable per-bot parity, so adjacent teammates
/// split rather than pile into one another.
fn dodge_side_sign(off: f32, lateral_vel: f32, even: bool) -> f32 {
    if off.abs() > DODGE_MISS_EPS {
        off.signum()
    } else if lateral_vel.abs() > DODGE_VEL_EPS {
        lateral_vel.signum()
    } else if even {
        1.0
    } else {
        -1.0
    }
}

/// Exact closest approach for a constant-velocity projectile relative to a moving bot.
fn linear_closest_approach(relative: Vec3, relative_velocity: Vec3, horizon: f32) -> (f32, Vec3) {
    let speed2 = relative_velocity.length_squared();
    let t = if speed2 > 1e-6 {
        (-relative.dot(relative_velocity) / speed2).clamp(0.0, horizon.max(0.0))
    } else {
        0.0
    };
    (t, relative + relative_velocity * t)
}

/// Short-horizon grenade approach under gravity. Live bounce state is already reflected in the
/// entity's current velocity; sampling the next second catches the incoming/fuse threat without
/// pretending to know future collision normals. Twelve samples are cheap (projectile counts are
/// tiny) and keep the maximum temporal gap below a normal human reaction beat.
fn ballistic_closest_approach(relative: Vec3, relative_velocity: Vec3, horizon: f32, gravity: f32) -> (f32, Vec3) {
    const SAMPLES: usize = 12;
    let mut best = (0.0, relative);
    let mut best_dist = relative.length_squared();
    for i in 1..=SAMPLES {
        let t = horizon.max(0.0) * i as f32 / SAMPLES as f32;
        let at = relative + relative_velocity * t - Vec3::new(0.0, 0.0, 0.5 * gravity * t * t);
        let d = at.length_squared();
        if d < best_dist {
            best = (t, at);
            best_dist = d;
        }
    }
    best
}

/// Dodge any visible live rocket, grenade, or nail whose predicted closest approach enters its
/// damage tube. Unlike [`grenade_tactics`], this works when grenades are not shootable and covers
/// every projectile weapon. Skill controls only how early the threat is noticed; every skill level
/// understands the same geometry. Returns true when survival claimed movement this frame.
///
/// A flying projectile is escaped *sideways*, across its travel line ([`dodge_side_sign`] picks which
/// side, [`safe_dodge_move`] keeps it off lava and ledges) — the one direction that opens distance from
/// something far faster than the bot. A grenade, whose blast is centred on a landing point rather than
/// strung along a line, is still fled radially. Either way a jump only ever extends a dodge already
/// moving ([`DODGE_HOP_SPEED`]): a standing hop builds no separation and just hangs the bot in the air.
pub(crate) fn projectile_dodge(
    game: &mut GameState,
    e: EntId,
    origin: Vec3,
    now: f32,
    cmd: &mut BotCmd,
) -> bool {
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let horizon = 0.35 + 0.1 * skill;
    let awareness = 400.0 + 60.0 * skill;
    let gravity = game.host().cvar(c"sv_gravity");
    let bot_velocity = game.entities[e].v.velocity;
    let my_team = game.entities[e].mode_p.team;
    let shootable_grenades = game.host().cvar_bool(c"rtx_shootable_grenades");
    let eye = origin + VEC_VIEW_OFS;
    let live: Vec<_> = game
        .entities
        .iter()
        .enumerate()
        .filter_map(|(i, ent)| {
            matches!(ent.touch, Touch::Missile | Touch::Grenade | Touch::Spike | Touch::SuperSpike)
                .then_some((
                    EntId(i as u32),
                    ent.touch,
                    ent.v.origin,
                    ent.v.velocity,
                    ent.owner(),
                    ent.v.nextthink,
                    ent.in_use && ent.combat.voided == 0.0,
                ))
        })
        // Own rockets/nails cannot turn back; own grenades can bounce back into their thrower and
        // remain a real threat once their velocity reverses.
        .filter(|&(_, touch, _, _, owner, _, live)| live && (owner != e || touch == Touch::Grenade))
        .collect();

    // (score, time of closest approach, projectile position at danger, kind, velocity) — lower score
    // is more urgent.
    let mut best: Option<(f32, f32, Vec3, Touch, Vec3)> = None;
    for (projectile, touch, pos, velocity, owner, nextthink, _) in live {
        let enemy_owned = owner.is_some()
            && owner != e
            && (my_team == 0 || game.entities[owner].mode_p.team != my_team);
        if touch == Touch::Grenade && shootable_grenades && enemy_owned {
            continue; // grenade_tactics gets first choice to shoot this one down safely
        }
        if (pos - origin).length() > awareness {
            continue;
        }
        // A wall blocks both sight and the current projectile path. Very close open projectiles are
        // still seen even when their point-sized entity is not the trace hit.
        let sight = game.traceline(eye, pos, false, e);
        if sight.ent != projectile && sight.fraction < 0.95 {
            continue;
        }
        let fuse = (nextthink - now).max(0.0);
        let lookahead = if touch == Touch::Grenade {
            horizon.min(fuse)
        } else {
            horizon
        };
        let relative = pos - origin;
        let relative_velocity = velocity - bot_velocity;
        let (mut t, _) = if touch == Touch::Grenade {
            ballistic_closest_approach(relative, relative_velocity, lookahead, gravity)
        } else {
            linear_closest_approach(relative, relative_velocity, lookahead)
        };
        // A projectile already moving away is no longer a threat merely because it is momentarily
        // close. A grenade with an imminent fuse remains dangerous even while rolling away.
        if t <= 0.01 && relative.dot(relative_velocity) >= 0.0 && !(touch == Touch::Grenade && fuse <= 0.25) {
            continue;
        }

        let ballistic_drop = if touch == Touch::Grenade {
            Vec3::new(0.0, 0.0, 0.5 * gravity * t * t)
        } else {
            Vec3::ZERO
        };
        let mut danger = pos + velocity * t - ballistic_drop;
        // Respect an imminent world/entity collision. Rockets explode there; nails stop there;
        // grenades bounce, but the contact point is still the conservative near-term danger point.
        if t > 0.0 {
            let path = game.traceline(pos, danger, false, projectile);
            if path.fraction < 1.0 {
                if matches!(touch, Touch::Spike | Touch::SuperSpike) && path.ent != e {
                    continue; // the nail is stopped before reaching this bot
                }
                t *= path.fraction;
                danger = path.endpos;
            }
        }
        let bot_at = origin + bot_velocity * t;
        let dist = (danger - bot_at).length();
        let radius = match touch {
            Touch::Missile => GRENADE_BLAST_RADIUS + 24.0,
            Touch::Grenade => GRENADE_BLAST_RADIUS + 16.0,
            Touch::Spike | Touch::SuperSpike => 40.0,
            _ => unreachable!(),
        };
        if dist > radius {
            continue;
        }
        let score = t + 0.2 * dist / radius;
        if best.is_none_or(|(old, _, _, _, _)| score < old) {
            best = Some((score, t, danger, touch, velocity));
        }
    }

    let Some((_, t, danger, touch, velocity)) = best else {
        return false;
    };
    let axis = Vec3::new(velocity.x, velocity.y, 0.0).normalize_or_zero();
    let radial = Vec3::new(origin.x - danger.x, origin.y - danger.y, 0.0).normalize_or_zero();
    let (mv, may_hop) = if touch == Touch::Grenade || axis == Vec3::ZERO {
        // A grenade's blast is centred on where it comes to rest, not strung along a line — and a
        // projectile dropping dead vertical has no travel line to cross. Radial escape is the geometry.
        let mut away = radial;
        if away == Vec3::ZERO {
            // A direct intercept has no radial side yet; step perpendicular to its travel, with a stable
            // per-bot side so adjacent teammates do not all dodge into one another.
            away = Vec3::new(-velocity.y, velocity.x, 0.0).normalize_or_zero();
            if e.0.is_multiple_of(2) {
                away = -away;
            }
        }
        if away == Vec3::ZERO {
            // A grenade falling vertically onto the bot has no horizontal travel axis. Pick a stable
            // orthogonal escape rather than accepting the direct overhead blast.
            away = if e.0.is_multiple_of(2) { Vec3::X } else { Vec3::Y };
        }
        safe_flee_move(game, e, origin, away)
    } else {
        // A rocket or nail is dodged *across* its travel line, never along it. The radial escape is
        // useless here: for a well-aimed shot the danger point sits on the bot, so `origin - danger`
        // is noise that flips every frame, and head-on it degenerates into a backpedal — no bot
        // outruns a rocket. The projectile's own velocity gives a lateral axis that stays put.
        let perp = Vec3::new(-axis.y, axis.x, 0.0);
        let off = (origin + bot_velocity * t - danger).dot(perp); // the predicted miss a dodge widens
        let sign = dodge_side_sign(off, bot_velocity.dot(perp), e.0.is_multiple_of(2));
        safe_dodge_move(game, e, origin, perp * sign, radial)
    };
    cmd.move_world = mv;
    // A hop may only *extend* a dodge already carrying speed — that is a side-jump, and it keeps the
    // ground velocity it took off with. Hopping from a standstill is the pogo this gate exists to
    // prevent: it surrenders ground acceleration for the air-wish cap and moves the bot nowhere.
    if may_hop && bot_velocity.dot(mv.normalize_or_zero()) > DODGE_HOP_SPEED {
        cmd.buttons |= BUTTON_JUMP;
    }
    true
}

// --- shootable-grenade tactics -----------------------------------------------------------------

/// Grenade blast: damage at the centre and the radius over which it falls off (`grenade_explode`
/// deals 120 over `damage + 40` units; see `combat.rs::t_radius_damage`).
const GRENADE_BLAST_DAMAGE: f32 = 120.0;
pub(crate) const GRENADE_BLAST_RADIUS: f32 = 160.0;
/// How near a grenade must be for a bot to notice it at all.
const GRENADE_AWARE: f32 = 320.0;
/// Never shoot a grenade closer than this — detonating it point-blank is worse than the threat.
pub(crate) const GRENADE_MIN_SHOOT: f32 = 100.0;
/// Only shoot a grenade to disarm/airburst it if the splash we'd eat is at most this share of our
/// health — a low-health bot only detonates ones already outside its own blast, a healthy one will
/// trade a little splash for the disarm (the "far enough vs health" call).
pub(crate) const GRENADE_SHOOT_HEALTH_FRAC: f32 = 0.5;

/// Splash a blast at `dist` from its centre would deal to a player (linear falloff to the radius).
pub(crate) fn blast_self_damage(dist: f32) -> f32 {
    if dist < GRENADE_BLAST_RADIUS {
        (GRENADE_BLAST_DAMAGE - 0.5 * dist).max(0.0)
    } else {
        0.0
    }
}

/// The best hitscan weapon the bot owns and can feed, for detonating a grenade precisely: the
/// lightning beam first (a continuous line, most reliable on the 8u hit radius), then the shotguns.
/// Underwater the beam is skipped — firing it there is a discharge, not a detonation — so the bot
/// falls to a shotgun (or, with only the LG, doesn't shoot the grenade down).
pub(crate) fn hitscan_choice(g: &GameState, e: EntId) -> Option<(i32, Weapon)> {
    let v = &g.entities[e].v;
    if v.items.has(Items::LIGHTNING) && v.ammo_cells >= 1.0 && v.waterlevel <= 1.0 {
        Some((8, Weapon::Lightning))
    } else if v.items.has(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
        Some((3, Weapon::SuperShotgun))
    } else if v.items.has(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
        Some((2, Weapon::Shotgun))
    } else {
        None
    }
}

/// Whether a living teammate (other than `e`) stands within a blast at `pos` — so we don't detonate
/// a grenade on our own side. Always false in non-team play (`my_team == 0`).
pub(crate) fn teammate_in_blast(g: &GameState, e: EntId, my_team: u8, pos: Vec3) -> bool {
    if my_team == 0 {
        return false;
    }
    let maxclients = g.host().cvar(c"maxclients") as i32;
    (1..=maxclients as u32).map(EntId).any(|p| {
        let ent = &g.entities[p];
        p != e
            && ent.in_use
            && ent.is_player()
            && ent.v.health > 0.0
            && ent.mode_p.team == my_team
            && (ent.v.origin - pos).length() < GRENADE_BLAST_RADIUS
    })
}

/// Whether the bot has a clear line to the grenade's centre (so a shot could actually reach it).
pub(crate) fn can_hit_grenade(game: &mut GameState, e: EntId, grenade: EntId) -> bool {
    let from = game.entities[e].v.origin + VEC_VIEW_OFS;
    let to = game.entities[grenade].v.origin;
    let tr = game.traceline(from, to, false, e);
    tr.fraction > 0.9 || tr.ent == grenade
}

/// Aim at a grenade and fire a hitscan shot to detonate it; select the gun first if needed. Returns
/// `false` (didn't commit) if the bot has no usable hitscan weapon. The shot leaves along the
/// *smoothed* view, so it fires only once that view has swung onto the grenade (and the gun is in
/// hand), matching how `engage` gates its shots.
pub(crate) fn shoot_grenade(game: &mut GameState, e: EntId, grenade: EntId, cmd: &mut BotCmd) -> bool {
    let Some((imp, weapon)) = hitscan_choice(game, e) else {
        return false;
    };
    let eye = game.entities[e].v.origin + VEC_VIEW_OFS;
    let gpos = game.entities[grenade].v.origin;
    cmd.look = angles_to(eye, gpos);
    // The grenade shot owns the trigger: engage's shot is solved against the *enemy*, and the view is
    // now swinging onto the grenade instead, so leaving it armed lets `emit` loose a round at the enemy
    // on its own tolerance while this code is still lining up. One trigger, one owner.
    cmd.shot = None;
    if game.entities[e].v.weapon != weapon {
        cmd.impulse = imp; // switching takes a frame; fire once we hold it
        return true;
    }
    // The detonation line check accepts a shot passing within 8u of the grenade — convert that to an
    // angular tolerance at this range so the bot fires as soon as its aim is close enough to connect.
    let dist = (gpos - eye).length().max(1.0);
    let cone = (8.0 / dist).atan().to_degrees().clamp(1.5, 5.0);
    let view = game.entities[e].bot.aim.angles;
    let dp = wrap180(view.x - cmd.look.x);
    let dy = wrap180(view.y - cmd.look.y);
    if view == Vec3::ZERO || (dp * dp + dy * dy).sqrt() <= cone {
        cmd.buttons |= BUTTON_ATTACK;
    }
    true
}

/// React to live shootable grenades, overlaid *after* [`engage`]. Two uses of the blast's area of
/// effect, both weighed against our own splash so we never blow ourselves up:
///
/// - **Defensive** — the nearest enemy grenade that's within (or heading into) our blast. If we can
///   detonate it at a safe distance (the splash we'd take is a small share of our health) we shoot
///   it down; if it's too close for that, we **run and hop away** instead.
/// - **Offensive** — a grenade sitting on the current enemy (and clear of our teammates): a free
///   airburst, shot from outside its blast.
///
/// Everything routes through the frame's [`BotCmd`]; the shot detonates the grenade through the
/// engine's `shootable_grenade_on_line`/`t_damage` path.
pub(crate) fn grenade_tactics(
    game: &mut GameState,
    e: EntId,
    enemy: Option<EntId>,
    origin: Vec3,
    cmd: &mut BotCmd,
) -> bool {
    if !game.host().cvar_bool(c"rtx_shootable_grenades") {
        return false; // grenades aren't hittable (and are point-size), so there's nothing to exploit
    }
    let live: Vec<EntId> = game
        .entities
        .iter()
        .enumerate()
        .filter(|(_, ent)| ent.classname() == Some("grenade") && ent.in_use && ent.combat.voided == 0.0)
        .map(|(i, _)| EntId(i as u32))
        .collect();
    if live.is_empty() {
        return false;
    }
    let my_team = game.entities[e].mode_p.team;
    let health = game.entities[e].v.health.max(1.0);
    // A grenade this bot is running as a lob→shoot combo (see `super::grenade`): don't let the
    // opportunistic offence below detonate it early — that would blow it *short* of the enemy and
    // shove them the wrong way. The combo driver detonates it at the right moment itself.
    let combo_grenade = game.entities[e].bot.grenade.ent;

    // Nearest threatening enemy grenade (defence), and the nearest grenade sitting on our enemy that
    // we can safely detonate (offence).
    let mut threat: Option<(EntId, f32)> = None; // (grenade, dist to us)
    let mut offense: Option<(EntId, f32)> = None;
    for grenade in live {
        let gpos = game.entities[grenade].v.origin;
        let gvel = game.entities[grenade].v.velocity;
        let d = (gpos - origin).length();
        if d > GRENADE_AWARE {
            continue;
        }
        let owner = game.entities[grenade].owner();
        let ally = owner == e || (my_team != 0 && owner.is_some() && game.entities[owner].mode_p.team == my_team);
        if !ally {
            // A threat if it's already within our splash, or approaching us from range.
            let approaching = (origin - gpos).dot(gvel) > 0.0;
            if (d < GRENADE_BLAST_RADIUS + 40.0 || approaching) && threat.is_none_or(|(_, bd)| d < bd) {
                threat = Some((grenade, d));
            }
        }
        if let Some(en) = enemy {
            let on_enemy = (game.entities[en].v.origin - gpos).length() < GRENADE_BLAST_RADIUS;
            if grenade.0 != combo_grenade
                && on_enemy
                && blast_self_damage(d) <= health * GRENADE_SHOOT_HEALTH_FRAC
                && !teammate_in_blast(game, e, my_team, gpos)
                && offense.is_none_or(|(_, bd)| d < bd)
            {
                offense = Some((grenade, d));
            }
        }
    }

    // Defence takes priority — survival first.
    if let Some((grenade, d)) = threat {
        let safe_to_shoot = d >= GRENADE_MIN_SHOOT && blast_self_damage(d) <= health * GRENADE_SHOOT_HEALTH_FRAC;
        if safe_to_shoot && can_hit_grenade(game, e, grenade) && shoot_grenade(game, e, grenade, cmd) {
            return true;
        }
        // Too close (or no clear shot / no hitscan gun): run away and hop off the ground to put
        // distance between us and the blast — but not into lava or off a ledge. Sidestep along a
        // hazard, and hold (no hop) when every escape is unsafe rather than leaping into the pit.
        let gpos = game.entities[grenade].v.origin;
        let away = Vec3::new(origin.x - gpos.x, origin.y - gpos.y, 0.0).normalize_or_zero();
        let (mv, may_hop) = safe_flee_move(game, e, origin, away);
        cmd.move_world = mv;
        if may_hop {
            cmd.buttons |= BUTTON_JUMP;
        }
        return true;
    }

    // Offence: airburst a grenade sitting on the enemy.
    if let Some((grenade, _)) = offense {
        if can_hit_grenade(game, e, grenade) {
            shoot_grenade(game, e, grenade, cmd);
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    // Combat-move geometry for the hazard-guard tests: enemy along +x, so `dir = +x`, `perp = +y`.
    const DIR: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const PERP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

    /// An enemy 400 units away along +x, running sideways at full speed.
    fn strafing_target() -> Target {
        let org = Vec3::new(400.0, 0.0, 0.0);
        Target {
            org,
            eye: org + Vec3::new(0.0, 0.0, 22.0),
            vel: Vec3::new(0.0, 320.0, 0.0),
            dist: 400.0,
            swimming: false,
            airborne: false,
        }
    }

    fn solve(weapon: Weapon, tgt: &Target, lead: f32) -> Vec3 {
        let my_eye = Vec3::new(0.0, 0.0, 22.0);
        let plan = BallisticPlan { land: None, air_gl: None, gl_ground: None };
        let (aim, _, _) =
            aim_solution(WeaponChoice::of(weapon), tgt, my_eye, Vec3::new(0.0, 0.0, 16.0), 800.0, &plan, lead);
        aim
    }

    /// Latency is time in which the target moves and we cannot see it, so it adds to the flight time
    /// — and for a hitscan, where there is no flight time, it *is* the lead.
    ///
    /// Zero inside a server, where the bot is the world: every one of these folds back to aiming at
    /// what it can see, which is why the same code serves both hosts.
    #[test]
    fn latency_leads_a_shot_by_exactly_the_time_we_cannot_see() {
        let tgt = strafing_target();

        // A server-side bot: no lead beyond the shot's own flight.
        let bullet = solve(Weapon::Lightning, &tgt, 0.0);
        assert_eq!(bullet, tgt.eye, "hitscan aims where it looks");

        // A client 100ms behind: the target has had 100ms to run since we saw it here.
        let led = solve(Weapon::Lightning, &tgt, 0.1);
        assert!((led.y - (tgt.eye.y + 32.0)).abs() < 0.01, "320u/s for 0.1s = 32u: {led:?}");

        // A rocket is led by its flight *and* the latency, so more than by flight alone.
        let rocket_now = solve(Weapon::RocketLauncher, &tgt, 0.0);
        let rocket_late = solve(Weapon::RocketLauncher, &tgt, 0.1);
        assert!(rocket_now.y > tgt.eye.y, "flight time alone already leads");
        assert!(
            (rocket_late.y - rocket_now.y - 32.0).abs() < 0.5,
            "and latency adds its own 32u on top: {rocket_now:?} vs {rocket_late:?}",
        );

        // A target standing still is where it is, however far behind we are.
        let still = Target { vel: Vec3::ZERO, ..strafing_target() };
        assert_eq!(solve(Weapon::Lightning, &still, 0.25), still.eye);
    }

    #[test]
    fn safe_combat_move_passes_through_when_clear() {
        // All dry → the exact original composition (dir·forward + perp·strafe·speed).
        let dry = |_: Vec3| Footing::Dry;
        let mv = safe_combat_move(&dry, DIR, PERP, MOVE_SPEED, 1.0, false);
        assert_eq!(mv, DIR * MOVE_SPEED + PERP * MOVE_SPEED);
    }

    #[test]
    fn safe_combat_move_prefers_flip() {
        // Holding range (no forward), strafing +y is a hazard → flip to −y, same speed.
        let hazard_plus_y = |mv: Vec3| if mv.y > 0.0 { Footing::Hazard } else { Footing::Dry };
        let mv = safe_combat_move(&hazard_plus_y, DIR, PERP, 0.0, 1.0, false);
        assert_eq!(mv, PERP * -MOVE_SPEED);
    }

    #[test]
    fn safe_combat_move_drops_backpedal_before_strafe() {
        // Retreating (−x) is a hazard but the strafe is fine → take the strafe alone, no backpedal.
        let hazard_back = |mv: Vec3| if mv.x < 0.0 { Footing::Hazard } else { Footing::Dry };
        let mv = safe_combat_move(&hazard_back, DIR, PERP, -MOVE_SPEED, 1.0, false);
        assert_eq!(mv, PERP * MOVE_SPEED); // strafe only, no −x component
        assert_eq!(mv.x, 0.0);
    }

    #[test]
    fn safe_combat_move_holds_ground_when_surrounded() {
        // Every real move steps toward a hazard → hold ground rather than walk into it.
        let all_hazard = |mv: Vec3| if mv == Vec3::ZERO { Footing::Dry } else { Footing::Hazard };
        let mv = safe_combat_move(&all_hazard, DIR, PERP, -MOVE_SPEED, 1.0, false);
        assert_eq!(mv, Vec3::ZERO);
    }

    #[test]
    fn safe_combat_move_prefers_dry_over_wet() {
        // Holding range: strafing +y is into water, −y is dry → pick the dry side even though the
        // wet one is a valid (non-hazard) move and comes first in the candidate order.
        let wet_plus_y = |mv: Vec3| if mv.y > 0.0 { Footing::Wet } else { Footing::Dry };
        let mv = safe_combat_move(&wet_plus_y, DIR, PERP, 0.0, 1.0, false);
        assert_eq!(mv, PERP * -MOVE_SPEED);
    }

    #[test]
    fn safe_combat_move_wades_when_no_dry_option() {
        // Every real move heads into water (none dry, none a hazard) → still move: take the first
        // wet candidate rather than freezing (wading out beats treading water).
        let all_wet = |mv: Vec3| if mv == Vec3::ZERO { Footing::Dry } else { Footing::Wet };
        let mv = safe_combat_move(&all_wet, DIR, PERP, MOVE_SPEED, 1.0, false);
        assert_eq!(mv, DIR * MOVE_SPEED + PERP * MOVE_SPEED); // the first candidate, fwd+strafe
    }

    #[test]
    fn safe_combat_move_burning_never_holds() {
        // Standing *in* lava with every real move a hazard: a non-burning bot would hold (Vec3::ZERO),
        // but a burning one must keep moving — it takes the wanted move (fwd+strafe) and walks off.
        let all_hazard = |mv: Vec3| if mv == Vec3::ZERO { Footing::Dry } else { Footing::Hazard };
        let mv = safe_combat_move(&all_hazard, DIR, PERP, -MOVE_SPEED, 1.0, true);
        assert_ne!(mv, Vec3::ZERO, "a burning bot must not freeze on the coals");
        assert_eq!(mv, DIR * -MOVE_SPEED + PERP * MOVE_SPEED, "the wanted move (candidate 0)");
    }

    #[test]
    fn safe_combat_move_burning_still_prefers_dry() {
        // Burning doesn't abandon the priority order: a dry candidate still wins outright, so the bot
        // dodges toward the bank rather than blindly taking the wanted (hazardous) move.
        let hazard_plus_y = |mv: Vec3| if mv.y > 0.0 { Footing::Hazard } else { Footing::Dry };
        let mv = safe_combat_move(&hazard_plus_y, DIR, PERP, 0.0, 1.0, true);
        assert_eq!(mv, PERP * -MOVE_SPEED, "still flips to the dry −y side");
    }

    // Flee geometry: threat behind, so the escape heading `AWAY` is +x; the perpendiculars are ±y.
    const AWAY: Vec3 = Vec3::new(1.0, 0.0, 0.0);

    #[test]
    fn safe_flee_choice_burning_flees_and_keeps_hop() {
        // Every side a hazard: a non-burning grounded bot holds and suppresses the hop; a burning one
        // flees straight away and keeps the hop (its footing is the damage — no edge worth guarding).
        let all_hazard = |_: Vec3| Footing::Hazard;
        assert_eq!(safe_flee_choice(&all_hazard, AWAY, true, false), (Vec3::ZERO, false));
        assert_eq!(safe_flee_choice(&all_hazard, AWAY, true, true), (AWAY * MOVE_SPEED, true));
    }

    #[test]
    fn safe_flee_choice_burning_still_takes_dry_perp() {
        // Away is a hazard but one perpendicular (−y) is dry: burning or not, take the dry side — the
        // burning bot exits the pool and dodges in one move rather than fleeing straight into more.
        let away_hazard_dry_minus_y =
            |d: Vec3| if d.x > 0.0 { Footing::Hazard } else if d.y < 0.0 { Footing::Dry } else { Footing::Hazard };
        assert_eq!(
            safe_flee_choice(&away_hazard_dry_minus_y, AWAY, true, true),
            (Vec3::new(0.0, -1.0, 0.0) * MOVE_SPEED, true)
        );
    }

    // Dodge geometry: the projectile flies along +x, so the lateral escape axis is ±y and the chosen
    // side is +y; `RADIAL` (−x, straight back down the travel line) is the last-resort escape.
    const DODGE: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    const RADIAL: Vec3 = Vec3::new(-1.0, 0.0, 0.0);

    #[test]
    fn safe_dodge_choice_takes_preferred_side() {
        // Open ground → cross the travel line at full speed on the chosen side. The hop rides on
        // footing alone here; the caller's speed gate is what decides a side-jump.
        let dry = |_: Vec3| Footing::Dry;
        assert_eq!(safe_dodge_choice(&dry, DODGE, RADIAL, true, false), (DODGE * MOVE_SPEED, true));
        assert!(!safe_dodge_choice(&dry, DODGE, RADIAL, false, false).1, "airborne never hops");
    }

    #[test]
    fn safe_dodge_choice_flips_before_radial() {
        // The chosen side is a hazard: cross the line the other way rather than fall back to the
        // radial — a sidestep still opens distance from the projectile, backing off does not.
        let hazard_plus_y = |d: Vec3| if d.y > 0.0 { Footing::Hazard } else { Footing::Dry };
        assert_eq!(
            safe_dodge_choice(&hazard_plus_y, DODGE, RADIAL, true, false),
            (-DODGE * MOVE_SPEED, true)
        );
    }

    #[test]
    fn safe_dodge_choice_prefers_dry_flip_over_wet_preferred() {
        // The chosen side wades, the other is dry → take the dry side, even though the wet one is a
        // legal (non-hazard) move and comes first in the candidate order.
        let wet_plus_y = |d: Vec3| if d.y > 0.0 { Footing::Wet } else { Footing::Dry };
        assert_eq!(safe_dodge_choice(&wet_plus_y, DODGE, RADIAL, true, false), (-DODGE * MOVE_SPEED, true));
    }

    #[test]
    fn safe_dodge_choice_radial_when_both_sides_hazard() {
        // A catwalk along the rocket's line: neither sidestep survives, so back off radially rather
        // than stand still and eat it.
        let hazard_lateral = |d: Vec3| if d.y != 0.0 { Footing::Hazard } else { Footing::Dry };
        assert_eq!(
            safe_dodge_choice(&hazard_lateral, DODGE, RADIAL, true, false),
            (RADIAL * MOVE_SPEED, true)
        );
    }

    #[test]
    fn safe_dodge_choice_holds_when_surrounded_unless_burning() {
        // Hazards every way: a grounded bot holds and drops the hop rather than dodge off the edge —
        // but one already standing in lava has no footing worth saving, so it dodges and keeps the hop.
        let all_hazard = |_: Vec3| Footing::Hazard;
        assert_eq!(safe_dodge_choice(&all_hazard, DODGE, RADIAL, true, false), (Vec3::ZERO, false));
        assert_eq!(safe_dodge_choice(&all_hazard, DODGE, RADIAL, true, true), (DODGE * MOVE_SPEED, true));
    }

    #[test]
    fn safe_dodge_choice_skips_zero_radial() {
        // A dead-on shot leaves no radial (`origin - danger` collapses). Zero footing rates Dry, so an
        // unfiltered candidate list would hand back a stand-still *with the hop kept* — the pogo. Both
        // sides blocked must mean hold, hop suppressed.
        let hazard_lateral = |d: Vec3| if d.y != 0.0 { Footing::Hazard } else { Footing::Dry };
        assert_eq!(
            safe_dodge_choice(&hazard_lateral, DODGE, Vec3::ZERO, true, false),
            (Vec3::ZERO, false)
        );
    }

    #[test]
    fn dodge_side_sign_prefers_miss_then_velocity_then_parity() {
        // A real predicted miss picks the side, outranking a strafe running the other way.
        assert_eq!(dodge_side_sign(-40.0, 300.0, true), -1.0);
        // Miss inside the noise band → carry on with the strafe already under way.
        assert_eq!(dodge_side_sign(4.0, -150.0, true), -1.0);
        // Neither: a dead-on shot at a near-stationary bot splits on parity.
        assert_eq!(dodge_side_sign(4.0, 10.0, true), 1.0);
        assert_eq!(dodge_side_sign(4.0, 10.0, false), -1.0);
    }

    #[test]
    fn live_projectile_closest_approach_detects_crossings_and_departures() {
        let (t, at) = linear_closest_approach(
            Vec3::new(500.0, 0.0, 0.0),
            Vec3::new(-1000.0, 0.0, 0.0),
            1.0,
        );
        assert!((t - 0.5).abs() < 1e-5);
        assert!(at.length() < 1e-4);

        let (t, at) = linear_closest_approach(
            Vec3::new(100.0, 100.0, 0.0),
            Vec3::new(-100.0, 0.0, 0.0),
            2.0,
        );
        assert!((t - 1.0).abs() < 1e-5);
        assert!((at.length() - 100.0).abs() < 1e-4);
        assert_eq!(
            linear_closest_approach(Vec3::X * 100.0, Vec3::X * 100.0, 1.0).0,
            0.0,
            "a projectile already departing has no future approach",
        );
    }

    #[test]
    fn grenade_approach_includes_gravity() {
        let (t, at) = ballistic_closest_approach(
            Vec3::new(0.0, 0.0, 100.0),
            Vec3::ZERO,
            0.5,
            800.0,
        );
        assert!((t - 0.5).abs() < 1e-5);
        assert!(at.length() < 1e-4);
    }

    #[test]
    fn aim_line_strafe_keeps_moving_away_from_the_line() {
        let line = Vec3::X;
        let dodge = Vec3::Y;
        assert_eq!(aim_line_escape_sign(Vec3::ZERO, line, Vec3::new(100.0, 20.0, 0.0), dodge, -1.0), 1.0);
        assert_eq!(aim_line_escape_sign(Vec3::ZERO, line, Vec3::new(100.0, -20.0, 0.0), dodge, 1.0), -1.0);
        assert_eq!(
            aim_line_escape_sign(Vec3::ZERO, line, Vec3::new(100.0, 120.0, 0.0), dodge, -1.0),
            -1.0,
            "outside the danger tube the ordinary strafe schedule remains",
        );
    }

    // Line-of-fire verdict: bot at the origin, aim 400u downrange along +x unless noted.
    const AIM: Vec3 = Vec3::new(400.0, 0.0, 0.0);

    #[test]
    fn lof_verdict_hits_enemy_always_clear() {
        // A hull hit is clear even point-blank (the super-shotgun-fallback close shot).
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(60.0, 0.0, 0.0), true, true));
    }

    #[test]
    fn lof_verdict_open_shot_is_clear() {
        // Flew unobstructed but 80u wide of the target — an intended low-skill miss, not a corner.
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(400.0, 80.0, 0.0), false, false));
    }

    #[test]
    fn lof_verdict_wall_near_aim_is_clear() {
        // Wall 40u short of a distant aim point: the blast still lands on the target, safe for us.
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(360.0, 0.0, 0.0), false, true));
    }

    #[test]
    fn lof_verdict_corner_blocks() {
        // Wall 300u short of the target — a corner the rocket would detonate on, missing the enemy.
        assert!(!lof_verdict(Vec3::ZERO, AIM, Vec3::new(100.0, 0.0, 0.0), false, true));
    }

    #[test]
    fn lof_verdict_self_splash_blocks() {
        // Impact within 48u of a *close* aim (150u) but only 120u from us — inside our own blast
        // radius. Cleared under the old aim-point-only slack; the self-splash guard now blocks it.
        let aim = Vec3::new(150.0, 0.0, 0.0);
        assert!(!lof_verdict(Vec3::ZERO, aim, Vec3::new(120.0, 0.0, 0.0), false, true));
    }

    /// Blast falloff: full damage at the centre, tapering to zero at the radius, matching
    /// `t_radius_damage`'s `120 - 0.5·dist`.
    #[test]
    fn grenade_blast_falloff() {
        assert_eq!(blast_self_damage(0.0), 120.0);
        assert_eq!(blast_self_damage(160.0), 0.0); // past the radius
        assert!((blast_self_damage(140.0) - 50.0).abs() < 0.01);
        assert_eq!(blast_self_damage(400.0), 0.0);
    }

    #[test]
    fn quad_self_splash_scales_before_armor() {
        let bare = OwnSplashState {
            health: 100.0,
            armor_value: 0.0,
            armor_type: 0.0,
            damage_scale: 1.0,
            quad: false,
            immune: false,
        };
        assert_eq!(own_splash_health_damage(bare, 140.0), 25.0);
        let quad = OwnSplashState { damage_scale: 4.0, quad: true, ..bare };
        assert_eq!(own_splash_health_damage(quad, 140.0), 100.0);
        let yellow = OwnSplashState {
            armor_value: 150.0,
            armor_type: 0.6,
            ..quad
        };
        // f32 `0.6 * 100` sits just above 60, so the engine's ceil saves 61 and passes 39.
        assert_eq!(own_splash_health_damage(yellow, 140.0), 39.0);
        assert_eq!(own_splash_health_damage(OwnSplashState { immune: true, ..quad }, 0.0), 0.0);
    }

    #[test]
    fn quad_explosives_use_the_enlarged_caution_zone() {
        let normal = OwnSplashState {
            health: 100.0,
            armor_value: 0.0,
            armor_type: 0.0,
            damage_scale: 1.0,
            quad: false,
            immune: false,
        };
        let quad = OwnSplashState { damage_scale: 4.0, quad: true, ..normal };
        // No physical splash reaches 200u, but quad still uses KTX's conservative bore guard.
        assert!(own_splash_safe(normal, 200.0, 0.0));
        assert!(!own_splash_safe(quad, 200.0, 0.0));
        assert!(!own_splash_safe(quad, QUAD_SPLASH_CAUTION_RANGE, 0.0));
        assert!(own_splash_safe(quad, QUAD_SPLASH_CAUTION_RANGE + 0.01, 0.0));
        assert!(own_splash_safe(OwnSplashState { immune: true, ..quad }, 1.0, 0.0));
        // Even without quad, a 20-health bot refuses a 25-health self-hit.
        assert!(!own_splash_safe(OwnSplashState { health: 20.0, ..normal }, 140.0, 0.0));
    }

    #[test]
    fn intercept_leads_perpendicular_motion() {
        // Strafer at 400u moving 320 ups perpendicular to the line of fire, rocket at 1000 ups:
        // the true intercept takes longer than the naive dist/speed = 0.4s, and the solution must
        // sit exactly on the projectile sphere |r + v·t| = s·t.
        let r = Vec3::new(400.0, 0.0, 0.0);
        let v = Vec3::new(0.0, 320.0, 0.0);
        let s = 1000.0;
        let t = intercept_time(r, v, s).expect("intercept exists");
        assert!(
            t > 0.4,
            "perpendicular motion must lengthen the flight (naive 0.4), got {t}"
        );
        let miss = ((r + v * t).length() - s * t).abs();
        assert!(miss < 0.1, "intercept not on the projectile sphere: off by {miss}");

        // Radial motion is the near-no-op case: running straight away at 320 ups gives the exact
        // closing-speed time 400/(1000-320).
        let t2 = intercept_time(r, Vec3::new(320.0, 0.0, 0.0), s).unwrap();
        assert!((t2 - 400.0 / 680.0).abs() < 1e-3, "radial case wrong: {t2}");

        // Outrunnable target: no positive intercept.
        assert!(intercept_time(r, Vec3::new(1100.0, 0.0, 0.0), 1000.0).is_none());
    }

    #[test]
    fn press_advantage_bounds() {
        // Fresh belief of a finishable stack, healthy bot → press.
        assert!(press_advantage(100.0, 20.0, 1.0));
        // Stale belief → don't buy risk on a drifted estimate.
        assert!(!press_advantage(100.0, 20.0, 5.0));
        // Enemy not actually low → no press.
        assert!(!press_advantage(100.0, 60.0, 1.0));
        // Bot itself critical → don't press even a finishable kill.
        assert!(!press_advantage(15.0, 20.0, 1.0));
    }

    #[test]
    fn spread_scale_converges_and_widens() {
        let dist = 500.0;
        // First glimpse (visible_for 0), still, target not crossing → the loosest convergence, 1.6×.
        let first = spread_scale(0.0, 0.0, 0.0, dist);
        assert!((first - 1.6).abs() < 1e-6);
        // After sustained sight the convergence bottoms out at 0.7× (tighter than a fresh glimpse).
        let settled = spread_scale(2.0, 0.0, 0.0, dist);
        assert!((settled - 0.7).abs() < 1e-6);
        assert!(settled < first, "sustained sight must tighten aim");
        // Own motion and target crossing only ever widen the spread, each within its cap.
        assert!(spread_scale(2.0, 320.0, 0.0, dist) > settled);
        assert!(spread_scale(2.0, 0.0, 800.0, dist) > settled);
        let capped = spread_scale(0.0, 9999.0, 9_999_999.0, dist);
        assert!(capped <= 1.6 * 1.4 * 1.5 + 1e-4, "factors must stay within their caps");
    }

    #[test]
    fn spread_scale_never_below_settled_floor() {
        // The multiplier is bounded below by the fully-converged, still, non-crossing case (0.7),
        // so a high-skill bot's zero base spread stays zero and a low-skill bot never over-tightens.
        for &(v, own, perp) in &[(0.0, 0.0, 0.0), (1.5, 100.0, 200.0), (5.0, 320.0, 1000.0)] {
            assert!(spread_scale(v, own, perp, 400.0) >= 0.7 - 1e-6);
        }
    }

    #[test]
    fn ballistic_intercept_hits_falling_target() {
        // A crossing, rising target that then falls under gravity; rocket at 1000 ups from below.
        let from = Vec3::new(0.0, 0.0, 40.0);
        let g = 800.0;
        let p0 = Vec3::new(500.0, 0.0, 300.0);
        let v0 = Vec3::new(0.0, 300.0, 100.0);
        let s = 1000.0;
        let pos_at = |t: f32| ballistic_pos(p0, v0, g, None, t);
        let seed = intercept_time(p0 - from, v0, s).unwrap();
        let (t, meet) = ballistic_intercept(from, &pos_at, s, seed).expect("intercept");
        // The meet sits exactly on the projectile sphere |meet − from| = s·t.
        let residual = ((meet - from).length() - s * t).abs();
        assert!(residual < 0.1, "off the projectile sphere by {residual}");
        assert!(t > 0.0);
    }

    #[test]
    fn ballistic_pos_lands_and_runs() {
        let g = 800.0;
        let p0 = Vec3::new(0.0, 0.0, 100.0);
        let v0 = Vec3::new(200.0, 0.0, 0.0); // horizontal, falls to the floor at t = 0.5 (x = 100)
        let land = Some((0.5, Vec3::new(100.0, 0.0, 0.0)));
        // Before landing: on the parabola, between launch height and the floor.
        let mid = ballistic_pos(p0, v0, g, land, 0.25);
        assert!(mid.z > 0.0 && mid.z < 100.0, "mid z {}", mid.z);
        // After landing: clamped to the floor height, drifting on at the ground speed.
        let after = ballistic_pos(p0, v0, g, land, 1.0);
        assert_eq!(after.z, 0.0);
        assert!((after.x - (100.0 + 200.0 * 0.5)).abs() < 1e-3, "x {}", after.x);
        // Never predicted below the floor — the whole point of the clamp.
        assert!(ballistic_pos(p0, v0, g, land, 5.0).z >= 0.0);
    }

    /// An airborne target 600u downrange, solved from a bot at the origin — a rocket's flight over
    /// that range is ~0.6s, which is what the touchdown times below are placed either side of.
    /// The floor is at z = 0 throughout, so a landed player's origin rests at z = 24 and the shin
    /// the floor-splash aims at is z = 8.
    fn solve_air(weapon: Weapon, tgt: &Target, land: Option<(f32, Vec3)>, lead: f32) -> (Vec3, bool) {
        let plan = BallisticPlan { land, air_gl: None, gl_ground: None };
        let (aim, _, gate_direct) = aim_solution(
            WeaponChoice::of(weapon),
            tgt,
            Vec3::new(0.0, 0.0, 22.0),
            Vec3::new(0.0, 0.0, 16.0),
            800.0,
            &plan,
            lead,
        );
        (aim, gate_direct)
    }

    /// A target 600u downrange at height `z`, in the air. `land` times below are the real solution of
    /// this parabola against the floor, as `fall_land` would trace them.
    fn falling_target(z: f32, vel: Vec3) -> Target {
        let org = Vec3::new(600.0, 0.0, z);
        Target {
            org,
            eye: org + Vec3::new(0.0, 0.0, 22.0),
            vel,
            dist: (org - Vec3::new(0.0, 0.0, 22.0)).length(),
            swimming: false,
            airborne: true,
        }
    }

    /// The dodge answer: an enemy who'll be standing when the rocket gets there is shot at the ground
    /// they'll be standing on, not at the body — so the splash lands whether they hold or jump.
    #[test]
    fn rocket_floor_splashes_a_target_that_lands_first() {
        // Dropping from 60 at 200ups: down at t = 0.14, long before a ~0.6s rocket arrives.
        let tgt = falling_target(60.0, Vec3::new(0.0, 0.0, -200.0));
        let land = Some((0.14, Vec3::new(600.0, 0.0, 24.0)));
        let (aim, gate_direct) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.0);
        assert_eq!(aim, Vec3::new(600.0, 0.0, 8.0), "the shin of the spot they're standing on");
        assert!(!gate_direct, "a floor shot rides the splash tolerance, not the hull");
    }

    /// The leeway: they're still airborne when it arrives, but only just — the floor blast reaches
    /// them anyway, and catches them off the ground, which is where we want them.
    #[test]
    fn rocket_floor_splashes_a_target_about_to_land() {
        // Falling from an apex at 200: down at t = 0.66, a shade after the ~0.60s flight — so the
        // rocket meets them ~30u up, inside the leeway and well inside the blast.
        let tgt = falling_target(200.0, Vec3::ZERO);
        let land = Some((0.66, Vec3::new(600.0, 0.0, 24.0)));
        let (aim, gate_direct) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.0);
        assert_eq!(aim, Vec3::new(600.0, 0.0, 8.0), "the floor under where they'll be");
        assert!(!gate_direct);
    }

    /// The mid-air intercept is still the right answer while they're a body on a parabola.
    #[test]
    fn rocket_keeps_midair_intercept_when_it_arrives_first() {
        // Rising at 100ups from 300: not down until t = 0.97, and the rocket is there at ~0.63.
        let tgt = falling_target(300.0, Vec3::new(0.0, 0.0, 100.0));
        let land = Some((0.97, Vec3::new(600.0, 0.0, 24.0)));
        let (aim, gate_direct) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.0);
        assert!(aim.z > 100.0, "hull centre up on the parabola, nowhere near the floor: {aim:?}");
        assert!(gate_direct, "a body shot needs the hull");
    }

    /// Nails don't splash, so the floor is never a target for them — the aim still rides the clamp to
    /// the landing spot, but at the hull it has to hit.
    #[test]
    fn nailgun_never_aims_at_the_floor() {
        let tgt = falling_target(60.0, Vec3::new(0.0, 0.0, -200.0));
        let land = Some((0.14, Vec3::new(600.0, 0.0, 24.0)));
        let (aim, gate_direct) = solve_air(Weapon::SuperNailgun, &tgt, land, 0.0);
        assert_eq!(aim, Vec3::new(600.0, 0.0, 28.0), "hull centre at the landing spot (24 + 4)");
        assert!(gate_direct);
    }

    /// The blast goes under where they'll *be*, not where they touched down: a target that lands
    /// running keeps running, and the shot follows.
    #[test]
    fn floor_splash_follows_the_post_landing_run() {
        // Same 0.14s drop, running +y at 320ups: they touch down 45u along and keep going.
        let tgt = falling_target(60.0, Vec3::new(0.0, 320.0, -200.0));
        let land = Some((0.14, Vec3::new(600.0, 44.8, 24.0)));
        let (aim, gate_direct) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.0);
        assert_eq!(aim.z, 8.0, "still the shin of the floor");
        assert!(aim.y > 44.8, "and past the touchdown point at their ground speed: {aim:?}");
        assert!(!gate_direct);
    }

    /// Latency is time the target keeps falling before our shot even exists, so it counts toward
    /// their landing exactly as it counts toward the flight.
    #[test]
    fn latency_counts_toward_the_landing() {
        // Falling from an apex at 300: down at t = 0.83, with the rocket there at ~0.61.
        let tgt = falling_target(300.0, Vec3::ZERO);
        let land = Some((0.83, Vec3::new(600.0, 0.0, 24.0)));
        let (_, direct_now) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.0);
        // A client 150ms behind: they've had that long to keep falling before the shot even exists.
        let (aim_late, direct_late) = solve_air(Weapon::RocketLauncher, &tgt, land, 0.15);
        assert!(direct_now, "in the server's present they're still well up when it arrives");
        assert!(!direct_late, "seen 150ms late, they're on the floor by then — shoot the floor");
        assert_eq!(aim_late.z, 8.0, "{aim_late:?}");
    }

    /// A shot solved 400u downrange along +x, clean angles dead ahead — the geometry the on-target
    /// tests vary a view against. `speed`/`arc`/`direct` pick which gate branch is exercised.
    fn test_shot(weapon: Weapon, speed: f32, arc: bool, direct: bool) -> Shot {
        Shot {
            choice: WeaponChoice { impulse: 0, weapon, projectile_speed: speed, grenade_arc: arc },
            aim: Vec3::new(400.0, 0.0, 0.0),
            clean: Vec3::ZERO,
            muzzle_base: Vec3::ZERO,
            gate_direct: direct,
            enemy: EntId(2),
            origin: Vec3::ZERO,
        }
    }

    #[test]
    fn shot_on_target_projectile_tolerance_boundary() {
        // Skill 7 direct rocket at 400u: the gate is the ±16u hull, i.e. ~2.3° of slack.
        let shot = test_shot(Weapon::RocketLauncher, 1000.0, false, true);
        assert!(shot_on_target(Vec3::new(0.0, 2.0, 0.0), &shot, 7.0), "2° ≈ 14u — inside the hull");
        assert!(!shot_on_target(Vec3::new(0.0, 3.0, 0.0), &shot, 7.0), "3° ≈ 21u — past it");
    }

    #[test]
    fn shot_on_target_splash_looser_than_direct() {
        // The same geometry riding the blast instead of the hull: 40u at skill 7 passes what the
        // direct gate rejects — this is why a grounded rocket fires sooner than a mid-air one.
        let splash = test_shot(Weapon::RocketLauncher, 1000.0, false, false);
        let direct = test_shot(Weapon::RocketLauncher, 1000.0, false, true);
        assert!(shot_on_target(Vec3::new(0.0, 5.0, 0.0), &splash, 7.0), "5° ≈ 35u — inside the blast");
        assert!(!shot_on_target(Vec3::new(0.0, 5.0, 0.0), &direct, 7.0));
    }

    #[test]
    fn shot_on_target_hitscan_cone() {
        // Hitscan ignores range and gates on the per-weapon cone: the beam is tight, the pellets wide.
        let lg = test_shot(Weapon::Lightning, 0.0, false, true);
        assert!(shot_on_target(Vec3::new(0.0, 2.0, 0.0), &lg, 7.0));
        assert!(!shot_on_target(Vec3::new(0.0, 3.0, 0.0), &lg, 7.0));
        let sg = test_shot(Weapon::SuperShotgun, 0.0, false, true);
        assert!(shot_on_target(Vec3::new(0.0, 4.0, 0.0), &sg, 7.0));
        assert!(!shot_on_target(Vec3::new(0.0, 6.0, 0.0), &sg, 7.0));
    }

    #[test]
    fn shot_on_target_low_skill_fires_looser() {
        // A low-skill bot must still shoot — loose, and miss — rather than freeze waiting on a cone
        // its lagging aim never reaches.
        let shot = test_shot(Weapon::RocketLauncher, 1000.0, false, true);
        assert!(!shot_on_target(Vec3::new(0.0, 6.0, 0.0), &shot, 7.0));
        assert!(shot_on_target(Vec3::new(0.0, 6.0, 0.0), &shot, 0.0));
    }

    #[test]
    fn shot_on_target_grenade_arc_ranges_from_origin() {
        // A lobbed grenade leaves the body, not the raised muzzle, so its range comes off `origin`.
        // Push `muzzle_base` far downrange: only the arc branch is unmoved by it.
        let mut arc = test_shot(Weapon::GrenadeLauncher, 600.0, true, false);
        let mut straight = test_shot(Weapon::GrenadeLauncher, 600.0, false, false);
        arc.muzzle_base = Vec3::new(396.0, 0.0, 0.0); // a mere 4u of range if it were the launch point
        straight.muzzle_base = Vec3::new(396.0, 0.0, 0.0);
        // 8° is ~56u wide at 400u (past the 40u splash gate) but ~0.6u at 4u (trivially on target),
        // so the two branches disagree only because they measure range from different points.
        let view = Vec3::new(0.0, 8.0, 0.0);
        assert!(!shot_on_target(view, &arc, 7.0), "arc ranges from origin: 8° at 400u ≈ 56u > 40u");
        assert!(shot_on_target(view, &straight, 7.0), "straight ranges from muzzle_base: 8° at 4u ≈ 0.6u");
    }

    #[test]
    fn one_spring_step_closes_the_gap() {
        // The defect in miniature. A view 3° off a 400u direct solve fails the gate — but one
        // critically-damped spring step toward `clean` brings it inside. Gating on the *pre*-step
        // view (as `engage` used to) approves a shot that then leaves along the *post*-step view;
        // only judging the settled view describes the shot actually taken.
        let shot = test_shot(Weapon::RocketLauncher, 1000.0, false, true);
        let (omega, dt) = (aim_omega(7.0), 1.0 / 72.0);
        let (mut a, mut v) = (3.0_f32, 0.0_f32);
        for _ in 0..8 {
            v += (omega * omega * (0.0 - a) - 2.0 * omega * v) * dt;
            a += v * dt;
        }
        assert!(!shot_on_target(Vec3::new(0.0, 3.0, 0.0), &shot, 7.0), "pre-step: wide");
        assert!(shot_on_target(Vec3::new(0.0, a, 0.0), &shot, 7.0), "settled at {a}°: on target");
    }

    #[test]
    fn fire_tolerance_monotonic_and_tight_at_seven() {
        assert!((fire_tolerance(7.0, true) - 16.0).abs() < 1e-6); // ±hull at skill 7
        assert!((fire_tolerance(7.0, false) - 40.0).abs() < 1e-6);
        assert!(fire_tolerance(0.0, true) > fire_tolerance(7.0, true)); // widens as skill drops
        assert!(fire_tolerance(3.0, true) < fire_tolerance(3.0, false)); // direct tighter than splash
    }

    #[test]
    fn miss_distance_small_angle() {
        // 1° of yaw at 400u ≈ sin(1°)·400 ≈ 6.98u.
        let m = miss_distance(Vec3::new(0.0, 1.0, 0.0), Vec3::ZERO, 400.0);
        assert!((m - 6.98).abs() < 0.05, "miss {m}");
        assert_eq!(miss_distance(Vec3::ZERO, Vec3::ZERO, 400.0), 0.0); // on target
    }

    #[test]
    fn miss_distance_saturates_past_quarter_turn() {
        // The bug this guards: an unclamped sin dips back toward zero (and goes negative past 180°),
        // so a view aimed away from the solution would read as a *hit*. Any gap ≥90° must saturate to
        // the full range so the fire gate can never pass a back-to-front shot.
        for &(dp, dy) in &[(0.0, 175.0), (0.0, 180.0), (60.0, 170.0), (90.0, 90.0)] {
            let m = miss_distance(Vec3::new(dp, dy, 0.0), Vec3::ZERO, 400.0);
            assert!((m - 400.0).abs() < 1e-3, "off by {dp},{dy} should saturate to range, got {m}");
        }
    }

    /// A loadout with the given weapon bits and every ammo pool full — the ammo count is what most
    /// `choose_weapon` branches gate on, so tests that want a weapon *fireable* start here and drain
    /// what they mean to.
    fn armed(items: Items) -> Loadout {
        Loadout { items, shells: 100.0, nails: 100.0, rockets: 100.0, cells: 100.0 }
    }

    #[test]
    fn choose_weapon_falls_back_to_nailguns() {
        // A bot restricted to a nailgun (via rtx_weapons) must fire it, not the axe. Super first.
        let sng = choose_weapon(armed(Items::SUPER_NAILGUN), 400.0, false, false, false, false);
        assert_eq!(sng.weapon, Weapon::SuperNailgun);
        assert_eq!(sng.projectile_speed, NAIL_SPEED);
        let ng = choose_weapon(armed(Items::NAILGUN), 400.0, false, false, false, false);
        assert_eq!(ng.weapon, Weapon::Nailgun);
        // Out of nails → nothing else to fire → the axe.
        let dry = Loadout { nails: 0.0, ..armed(Items::SUPER_NAILGUN | Items::NAILGUN) };
        assert_eq!(choose_weapon(dry, 400.0, false, false, false, false).weapon, Weapon::Axe);
    }

    #[test]
    fn choose_weapon_gl_needs_a_solution() {
        // A GL-only bot only fires the GL when engage supplies an arc (gl_ground/gl_air); with no
        // solution there's nothing else to fire, so it holds the axe (never lobs blindly).
        let gl = armed(Items::GRENADE_LAUNCHER);
        assert_eq!(choose_weapon(gl, 400.0, false, false, false, false).weapon, Weapon::Axe);
        assert_eq!(choose_weapon(gl, 400.0, false, true, false, false).weapon, Weapon::GrenadeLauncher);
        assert_eq!(choose_weapon(gl, 400.0, true, false, false, false).weapon, Weapon::GrenadeLauncher);
    }

    #[test]
    fn choose_weapon_full_arsenal_unchanged() {
        // Regression guard: with everything (and dry) and no finish read, the range order is still SSG
        // (point-blank <140) / LG (mid <550) / RL (beyond) — the nailgun fallbacks never pre-empt it.
        let all = armed(Items::all());
        assert_eq!(choose_weapon(all, 100.0, false, false, false, false).weapon, Weapon::SuperShotgun);
        assert_eq!(choose_weapon(all, 400.0, false, false, false, false).weapon, Weapon::Lightning);
        assert_eq!(choose_weapon(all, 600.0, false, false, false, false).weapon, Weapon::RocketLauncher);
    }

    #[test]
    fn choose_weapon_never_lg_underwater() {
        // Underwater the lightning gun is barred (it would discharge). With a full arsenal the
        // mid-range pick falls through to the rocket launcher instead of the LG...
        let all = armed(Items::all());
        assert_eq!(choose_weapon(all, 400.0, false, false, true, false).weapon, Weapon::RocketLauncher);
        // ...and a bot whose only fed gun is the LG drops to the axe rather than discharging.
        let lg_only = armed(Items::LIGHTNING);
        assert_eq!(choose_weapon(lg_only, 400.0, false, false, true, false).weapon, Weapon::Axe);
        // Dry, that same LG-only bot fires the LG as usual (regression guard on the gate).
        assert_eq!(choose_weapon(lg_only, 400.0, false, false, false, false).weapon, Weapon::Lightning);
    }

    #[test]
    fn choose_weapon_finishes_a_low_enemy_with_a_hitscan_hit() {
        // A believed-low enemy past point blank: the pick swaps the dodgeable rocket for a hitscan
        // direct hit so the near-kill lands before the target can strafe clear of the flight.
        let all = armed(Items::all());
        // In the 550–600 band (past the normal LG cap of PREFERRED_RANGE+150) it's the rocket without
        // a finish read; with one, the lightning gun's true 600-unit reach takes the guaranteed hit.
        assert_eq!(choose_weapon(all, 580.0, false, false, false, false).weapon, Weapon::RocketLauncher);
        assert_eq!(choose_weapon(all, 580.0, false, false, false, true).weapon, Weapon::Lightning);
        // No lightning gun, inside the shotgun's finishing range: the tight single-barrel shotgun —
        // not the rocket, and not the wider SSG, which patterns worse at this distance. Contrast the
        // no-finish rocket.
        let no_lg = armed(Items::all() & !Items::LIGHTNING);
        assert_eq!(choose_weapon(no_lg, 400.0, false, false, false, false).weapon, Weapon::RocketLauncher);
        assert_eq!(choose_weapon(no_lg, 400.0, false, false, false, true).weapon, Weapon::Shotgun);
        // Past FINISH_SHOTGUN_RANGE the single-barrel can't close the kill out, so the finish keeps
        // the rocket's splash rather than swapping too early — even without a lightning gun.
        assert_eq!(choose_weapon(no_lg, 500.0, false, false, false, true).weapon, Weapon::RocketLauncher);
        // And past the lightning gun's reach the shotgun no longer serves either — the rocket stays.
        assert_eq!(choose_weapon(all, 700.0, false, false, false, true).weapon, Weapon::RocketLauncher);
        // Point blank is untouched — the super shotgun already one-shots a low enemy up close.
        assert_eq!(choose_weapon(all, 100.0, false, false, false, true).weapon, Weapon::SuperShotgun);
        // Underwater bars even the finish lightning gun; the shotgun still serves.
        assert_eq!(choose_weapon(all, 400.0, false, false, true, true).weapon, Weapon::Shotgun);
    }

    #[test]
    fn friendly_splash_fallback_keeps_fighting_with_a_direct_gun() {
        let all = armed(
            Items::ROCKET_LAUNCHER | Items::LIGHTNING | Items::SUPER_SHOTGUN | Items::SHOTGUN,
        );
        assert_eq!(safe_direct_choice(all, 500.0, false).unwrap().weapon, Weapon::Lightning);
        assert_eq!(safe_direct_choice(all, 100.0, false).unwrap().weapon, Weapon::SuperShotgun);
        assert_eq!(safe_direct_choice(all, 500.0, true).unwrap().weapon, Weapon::SuperShotgun);
        assert!(safe_direct_choice(armed(Items::ROCKET_LAUNCHER), 500.0, false).is_none());
    }

    #[test]
    fn discharge_worth_it_only_trades_for_quad_or_multi() {
        let v = |dist: f32, strength: f32, quad: bool| DischargeVictim { dist, strength, quad };
        // 100 cells → 3500 at the centre, −0.5/unit falloff. A single plain victim is never worth it.
        assert!(!discharge_worth_it(100.0, &[v(50.0, 100.0, false)]), "1v1 must not discharge");
        // A believed quad carrier in lethal range → worth the trade.
        assert!(discharge_worth_it(100.0, &[v(50.0, 100.0, true)]), "quad kill is worth it");
        // Two plain enemies both in lethal range → a 2-for-1 is worth it.
        assert!(discharge_worth_it(100.0, &[v(50.0, 100.0, false), v(80.0, 100.0, false)]));
        // A quad carrier too far to actually kill (blast has fallen below their HP) → not worth it.
        assert!(!discharge_worth_it(2.0, &[v(200.0, 100.0, true)]), "no kill, no discharge");
        // Nobody in the blast → never.
        assert!(!discharge_worth_it(100.0, &[]));
    }

    #[test]
    fn gl_primary_only_when_gl_is_the_only_gun() {
        // GL alone (with rockets) → primary; out of rockets → not (can't fire it).
        assert!(armed(Items::GRENADE_LAUNCHER).gl_primary());
        assert!(!Loadout { rockets: 0.0, ..armed(Items::GRENADE_LAUNCHER) }.gl_primary());
        // Any other fireable gun, or the RL, disqualifies it (the combo / the RL take over).
        assert!(!armed(Items::GRENADE_LAUNCHER | Items::SHOTGUN).gl_primary());
        assert!(!armed(Items::GRENADE_LAUNCHER | Items::ROCKET_LAUNCHER).gl_primary());
        // A GL + an *empty* shotgun is still GL-primary (the shotgun can't fire).
        assert!(Loadout { shells: 0.0, ..armed(Items::GRENADE_LAUNCHER | Items::SHOTGUN) }.gl_primary());
    }
}
