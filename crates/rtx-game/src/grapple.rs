// SPDX-License-Identifier: AGPL-3.0-or-later

//! The grappling hook — a faithful port of the Wedge (Steve Bond) QuakeWorld grapple from
//! purectf's `grapple.qc`, minus its CTF-specific team/standby checks. The *feel* is purectf's;
//! only the **sound** follows ktx, which made it less intrusive by dropping the looping chain
//! rattle (`weapons/chain{1,2,3}.wav` + the `tink1` channel-clear) for one-shots: `weapons/ax1.wav`
//! on throw, `player/axhit{1,2}.wav` on impact, `weapons/bounce2.wav` on release.
//!
//! It's a weapon ([`Items::GRAPPLE`], handed out at spawn behind the `rtx_grapple` cvar and
//! selected by impulse): firing throws a fast hook projectile ([`Self::throw_grapple`]); when it
//! strikes something ([`Self::anchor_grapple`]) it anchors and the player reels toward it each
//! frame ([`Self::service_grapple`], driven from `PlayerPreThink`). A hooked *player* is tracked
//! and lightly damaged ([`Self::grapple_track`]); a moving anchor's velocity is copied so you ride
//! platforms. Three spinning chain links trail the rope ([`Self::build_chain`] /
//! [`Self::update_chain`]). Per-player state lives in [`GrappleState`](crate::entity::GrappleState);
//! the hook entity stores its target in `enemy` and the chain head in `goalentity`.
//!
//! One deliberate omission: purectf freezes the player's `.gravity` while reeling in close (a
//! jitter fix). rtx's engine-shared entvars don't expose a per-entity gravity field, but the pull
//! overwrites `velocity` every frame anyway, so gravity is effectively cancelled — the freeze
//! isn't needed.

use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::defs::*;
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;
use crate::obituary::DeathType;

/// QuakeWorld default player speed, the base the hook speeds scale off (purectf's `self.maxspeed`).
const MAXSPEED: f32 = 320.0;
/// Hook throw speed before the `rtx_hook_speed` multiplier (`2.5 * maxspeed` = ~800 ups at ×1).
const HOOK_THROW_SPEED: f32 = 2.5 * MAXSPEED;
/// Reel-in speed before the `rtx_hook_pull` multiplier (`2.35 * maxspeed` = ~750 ups at ×1).
const HOOK_PULL_SPEED: f32 = 2.35 * MAXSPEED;
/// Distance under which the rope is taut enough to ditch the now-redundant chain links.
const HOOK_CLOSE: f32 = 100.0;

impl GameState {
    /// `Throw_Grapple` — fire the hook. Rejects a second throw while one is already out.
    pub(crate) fn throw_grapple(&mut self, player: EntId) {
        if self.entities[player].grapple.hook_out {
            return;
        }
        self.small_kick(player);
        // ktx made the hook less intrusive by dropping the looping chain rattle for a single
        // one-shot swing on throw (and no attach/clear loops below).
        self.host
            .sound(player, Channel::Weapon, Sound::WEAPONS_AX1, 1.0, Attenuation::Norm);

        let (origin, v_angle) = {
            let v = &self.entities[player].v;
            (v.origin, v.v_angle)
        };
        self.make_vectors(v_angle);
        let v_forward = self.globals.v_forward;
        let now = self.globals.time;
        let throw_speed = HOOK_THROW_SPEED * self.host.cvar(c"rtx_hook_speed");

        let m = self.spawn();
        {
            let hook = &mut self.entities[m];
            hook.v.movetype = MoveType::FlyMissile;
            hook.v.solid = Solid::BBox;
            hook.set_owner(player);
            hook.classname = Some("hook".into());
            hook.v.velocity = v_forward * throw_speed;
            hook.v.avelocity = Vec3::new(0.0, 0.0, -500.0);
            hook.set_touch(Touch::Hook);
            hook.think = Think::BuildChain; // defer the links a frame, as the QC does
            hook.v.nextthink = now + 0.1;
        }
        self.set_model(m, Model::PROGS_STAR);
        self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self
            .set_origin(m, origin + v_forward * 16.0 + Vec3::new(0.0, 0.0, 16.0));

        let p = &mut self.entities[player];
        p.grapple.hook = m.0;
        p.grapple.hook_out = true;
    }

    /// `Anchor_Grapple` — the hook's touch: stick to whatever it strikes (or bail on a projectile,
    /// the sky, or a released fire button).
    pub(crate) fn anchor_grapple(&mut self, hook: EntId, other: EntId) {
        let owner = self.entities[hook].owner();
        if other == owner {
            return;
        }
        // Never hook onto a projectile (including another hook or its chain).
        if let Some(cn) = self.entities[other].classname() {
            if matches!(cn, "missile" | "rocket" | "grenade" | "spike" | "hook" | "hook_link") {
                return;
            }
        }
        // Never hook the sky.
        let hook_org = self.entities[hook].v.origin;
        if self.pointcontents(hook_org) == crate::bsp::CONTENTS_SKY {
            self.reset_grapple(hook);
            return;
        }

        if self.entities[other].is_player() {
            self.host
                .sound(hook, Channel::Weapon, Sound::PLAYER_AXHIT1, 1.0, Attenuation::Norm);
            self.entities[other].deathtype = DeathType::Hook;
            self.t_damage(other, hook, owner, 10.0);
            // Hide the hook — we pull straight at the hooked player rather than match its velocity.
            self.entities[hook].v.model = 0;
        } else {
            self.host
                .sound(hook, Channel::Weapon, Sound::PLAYER_AXHIT2, 1.0, Attenuation::Norm);
            // One hit on impact; further damage only goes to players (so doors/triggers fire once).
            if self.entities[other].v.takedamage != TakeDamage::No {
                self.t_damage(other, hook, owner, 1.0);
            }
            let h = &mut self.entities[hook];
            h.v.velocity = Vec3::ZERO;
            h.v.avelocity = Vec3::ZERO;
        }

        if self.entities[owner].v.button0 == 0.0 {
            self.reset_grapple(hook);
            return;
        }

        {
            let p = &mut self.entities[owner];
            p.v.flags = p.v.flags.without(Flags::ONGROUND); // lift off so the reel can pull
            p.grapple.on_hook = true;
            p.grapple.lefty = true; // chain still up; the reel ditches it once it's close
        }

        let now = self.time();
        let h = &mut self.entities[hook];
        h.set_enemy(other);
        h.think = Think::GrappleTrack;
        h.v.nextthink = now;
        h.v.solid = Solid::Not;
        h.set_touch(Touch::None);
    }

    /// `Grapple_Track` — the anchored hook's think: follow a hooked player (damaging them) or ride
    /// a moving anchor, and drop the hook once the owner dies or releases.
    pub(crate) fn grapple_track(&mut self, hook: EntId) {
        let owner = self.entities[hook].owner();
        let enemy = self.entities[hook].enemy();
        let enemy_is_player = self.entities[enemy].is_player();

        // Release a hooked player once they die.
        if enemy_is_player && self.entities[enemy].v.health <= 0.0 {
            self.entities[owner].grapple.on_hook = false;
        }
        // Drop the hook if the owner died or let go.
        if !self.entities[owner].grapple.on_hook || self.entities[owner].v.health <= 0.0 {
            self.reset_grapple(hook);
            return;
        }

        if enemy_is_player {
            // Lost line of sight — unlock.
            if !self.can_damage(enemy, owner, EntId::WORLD) {
                self.reset_grapple(hook);
                return;
            }
            let epos = self.entities[enemy].v.origin;
            self.set_origin(hook, epos);
            self.entities[hook].v.origin = epos;
            self.t_damage(enemy, hook, owner, 1.0);
            self.spawn_blood(epos, 1);
        } else {
            // Ride a moving anchor (platform/door); velocity copying only works for non-players.
            let vel = self.entities[enemy].v.velocity;
            self.entities[hook].v.velocity = vel;
        }
        self.entities[hook].v.nextthink = self.time() + 0.1;
    }

    /// `Service_Grapple` — per-frame reel-in, driven from `PlayerPreThink` while `on_hook`. Pulls
    /// the player straight toward the anchor (a hooked player is tracked directly), and once the
    /// rope is taut ditches the now-redundant chain links. (purectf behaviour; only its looping
    /// chain sound is gone — see the module header.)
    pub(crate) fn service_grapple(&mut self, player: EntId) {
        let hook = EntId(self.entities[player].grapple.hook);

        // Let go only if fire is released *and* the grapple is still the active weapon — so you can
        // swing while shooting another gun.
        let (button0, is_grapple) = {
            let v = &self.entities[player].v;
            (v.button0, v.weapon == Weapon::Grapple)
        };
        if button0 == 0.0 && is_grapple {
            self.reset_grapple(hook);
            return;
        }

        let enemy = self.entities[hook].enemy();
        let target = if self.entities[enemy].is_player() {
            self.entities[enemy].v.origin
        } else {
            self.entities[hook].v.origin
        };
        let dir = target - self.entities[player].v.origin;
        let len = dir.length();
        let pull_speed = HOOK_PULL_SPEED * self.host.cvar(c"rtx_hook_pull");
        self.entities[player].v.velocity = dir.normalize_or_zero() * pull_speed;

        if len <= HOOK_CLOSE && self.entities[player].grapple.lefty {
            // Close enough — drop the chain links now, once (ktx played CHAIN3 here; we don't).
            let l1 = self.entities[hook].goalentity();
            if l1.is_some() {
                let now = self.time();
                let lead = &mut self.entities[l1];
                lead.think = Think::RemoveChain;
                lead.v.nextthink = now;
            }
            self.entities[player].grapple.lefty = false;
        }
    }

    /// `Reset_Grapple` — release the hook and restore the owner. The chain links self-remove once
    /// they see `hook_out` clear (see [`Self::update_chain`]).
    pub(crate) fn reset_grapple(&mut self, hook: EntId) {
        let owner = self.entities[hook].owner();
        if !owner.is_some() {
            return; // owner is world — nothing hooked
        }
        self.host
            .sound_no_phs(owner, Channel::Weapon, Sound::WEAPONS_BOUNCE2, 1.0, Attenuation::Norm);
        {
            let p = &mut self.entities[owner];
            p.grapple.on_hook = false;
            p.grapple.hook_out = false;
            p.v.weaponframe = 0.0;
        }
        let now = self.time();
        let h = &mut self.entities[hook];
        h.think = Think::SubRemove;
        h.v.nextthink = now;
    }

    // --- the cosmetic chain (three trailing links) ---

    /// `Build_Chain` — the hook's deferred think: spawn the three chain links and start the lead
    /// link repositioning them. Linked hook → l1 → l2 → l3 via `goalentity`.
    pub(crate) fn build_chain(&mut self, hook: EntId) {
        let owner = self.entities[hook].owner();
        let l1 = self.make_link(hook);
        let l2 = self.make_link(hook);
        let l3 = self.make_link(hook);
        self.entities[hook].set_goalentity(l1);
        self.entities[l1].set_goalentity(l2);
        self.entities[l2].set_goalentity(l3);

        let now = self.globals.time;
        let lead = &mut self.entities[l1];
        lead.set_owner(owner); // the lead link tracks the player (and via it, the hook)
        lead.think = Think::UpdateChain;
        lead.v.nextthink = now + 0.1;
    }

    /// `MakeLink` — one spinning chain-link entity, spawned at the hook.
    fn make_link(&mut self, hook: EntId) -> EntId {
        let origin = self.entities[hook].v.origin;
        let m = self.spawn();
        {
            let link = &mut self.entities[m];
            link.v.movetype = MoveType::FlyMissile;
            link.v.solid = Solid::Not;
            link.set_owner(hook);
            link.v.avelocity = Vec3::new(200.0, 200.0, 200.0);
            link.classname = Some("hook_link".into());
        }
        self.set_model(m, Model::PROGS_BIT);
        self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.set_origin(m, origin);
        m
    }

    /// `Update_Chain` — the lead link spaces all three links evenly along the rope each frame, or
    /// tears the chain down once the hook is gone.
    pub(crate) fn update_chain(&mut self, l1: EntId) {
        let owner = self.entities[l1].owner();
        if !self.entities[owner].grapple.hook_out {
            let now = self.globals.time;
            let lead = &mut self.entities[l1];
            lead.think = Think::RemoveChain;
            lead.v.nextthink = now;
            return;
        }
        let hook = EntId(self.entities[owner].grapple.hook);
        let owner_org = self.entities[owner].v.origin;
        let rope = self.entities[hook].v.origin - owner_org;
        let l2 = self.entities[l1].goalentity();
        let l3 = self.entities[l2].goalentity();
        self.move_link(l1, owner_org + rope * 0.25);
        self.move_link(l2, owner_org + rope * 0.5);
        self.move_link(l3, owner_org + rope * 0.75);
        self.entities[l1].v.nextthink = self.time() + 0.1;
    }

    /// `Remove_Chain` — free all three links (the lead link plus its two followers).
    pub(crate) fn remove_chain(&mut self, l1: EntId) {
        let l2 = self.entities[l1].goalentity();
        if l2.is_some() {
            let l3 = self.entities[l2].goalentity();
            if l3.is_some() {
                self.free(l3);
            }
            self.free(l2);
        }
        self.free(l1);
    }

    /// `setorigin` for a chain link, keeping our shadowed `v.origin` in sync.
    fn move_link(&mut self, link: EntId, pos: Vec3) {
        self.set_origin(link, pos);
        self.entities[link].v.origin = pos;
    }
}
