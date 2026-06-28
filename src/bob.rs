// SPDX-License-Identifier: AGPL-3.0-or-later

//! `func_bob` — a brush model that bobs back and forth along a direction (dumptruckDS /
//! progsdump / Arcane Dimensions), ported from ktx's `func_bob.c`.
//!
//! ## What the motion is
//!
//! Each cycle the brush accelerates from rest along `movedir` for the first half (an ease-in: the
//! per-tick velocity *ramp* `height` is scaled up by `waitmin` every tick) then decays toward the
//! far pivot for the second half (speed scaled by `waitmin2`), and the direction flips every
//! cycle. So `height` is a velocity intensity, not a distance — the visible amplitude emerges from
//! integrating the ramp, which is why a `height` of `0.9` still yields a sizeable bob.
//!
//! ## Avoiding ktx's drift bug
//!
//! ktx integrates the position from velocity and only ever zeroes the *velocity* (never the
//! *position*) at the pivot. Worse, it checks the cycle boundaries against `g_globalvars.time`
//! while scheduling off `ltime`, and `count` need not be a whole number of 0.1s ticks — so
//! successive cycles span different tick counts, the alternating displacements don't cancel, and
//! the brush wanders.
//!
//! This port keeps ktx's velocity ramp exactly (so the feel/amplitude match), but:
//! * times the cycle off an internal clock ([`BobState::cycle_t`]) reset to 0 at each pivot, so
//!   every cycle spans the same number of ticks and the per-cycle displacement is identical in
//!   magnitude, opposite in sign — they cancel over a full period;
//! * integrates that velocity into an `offset` from the spawn origin and drives the brush to
//!   `pos1 + offset·movedir` via velocity (so the pusher still carries riders), anchoring it so
//!   no positional error can accumulate.

use crate::defs::*;
use crate::entity::{EntId, Think};
use crate::game::GameState;

/// Think cadence (ktx's bob runs at 10 Hz).
const TICK: f32 = 0.1;

impl GameState {
    /// `func_bob` spawn. A solid bobbing brush; non-solid variants aren't supported, so they're
    /// dropped rather than left blocking the player.
    pub(crate) fn spawn_func_bob(&mut self, e: EntId) -> bool {
        let spawnflags = self.entities[e].v.spawnflags;
        if spawnflags.has(FuncBobFlags::NONSOLID) || spawnflags.has(FuncBobFlags::MG_NONSOLID) {
            return false;
        }

        {
            let ent = &mut self.entities[e];
            ent.v.movetype = MoveType::Push.as_f32();
            ent.v.solid = Solid::Bsp.as_f32();
        }
        self.link_brush(e);
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e, origin); // relink at the spawn position
        self.set_movedir(e); // bob axis from the `angle`/`angles` key

        // `delay` staggers nearby bobs by delaying the first tick; a negative value randomises it.
        let delay = self.entities[e].mover.delay;
        let delay = if delay < 0.0 {
            self.random() + self.random() + self.random()
        } else {
            delay
        };
        let now = self.time();
        let movedir = self.entities[e].v.movedir.normalize_or_zero();

        let ent = &mut self.entities[e];
        ent.v.movedir = movedir;
        // Defaults mirror ktx.
        if ent.mover.height <= 0.0 {
            ent.mover.height = 8.0; // velocity ramp intensity
        }
        if ent.mover.count < 1.0 {
            ent.mover.count = 2.0; // seconds for one direction (half a full bob)
        }
        if ent.bob.waitmin <= 0.0 {
            ent.bob.waitmin = 1.0; // speed up
        }
        if ent.bob.waitmin2 <= 0.0 {
            ent.bob.waitmin2 = 0.75; // slow down
        }
        ent.mover.pos1 = ent.v.origin; // bob anchor — keeps it drift-free
        ent.bob.offset = 0.0;
        ent.bob.vel = 0.0;
        // Begin ready to start a cycle; a positive ramp makes the first flip go negative (ktx's
        // `lefty` 0→1).
        ent.bob.cycle_t = ent.mover.count;
        ent.bob.t_length = ent.mover.height;
        ent.v.ltime = now;
        ent.v.nextthink = now + TICK + delay;
        ent.think = Think::FuncBobThink;
        true
    }

    pub(crate) fn func_bob_think(&mut self, e: EntId) {
        let now = self.time();
        let (origin, pos1, movedir, height, count) = {
            let v = &self.entities[e];
            (v.v.origin, v.mover.pos1, v.v.movedir, v.mover.height, v.mover.count)
        };

        let ent = &mut self.entities[e];
        ent.bob.cycle_t += TICK;
        if ent.bob.cycle_t >= count {
            // New cycle: flip direction, restart the ramp at full height, rest at the pivot.
            ent.bob.cycle_t = 0.0;
            ent.bob.t_length = if ent.bob.t_length >= 0.0 { -height } else { height };
            ent.bob.vel = 0.0;
        }
        if ent.bob.cycle_t < count * 0.5 {
            // First half: ease in — the ramp itself grows, and adds to the speed.
            ent.bob.t_length *= ent.bob.waitmin;
            ent.bob.vel += ent.bob.t_length;
        } else {
            // Second half: ease out toward the far pivot.
            ent.bob.vel *= ent.bob.waitmin2;
        }
        ent.bob.offset += ent.bob.vel * TICK;

        // Anchor to pos1 + offset·movedir and reach it via velocity (so riders are carried).
        let target = pos1 + movedir * ent.bob.offset;
        ent.v.velocity = (target - origin) * (1.0 / TICK);
        ent.v.ltime = now;
        ent.v.nextthink = now + TICK;
    }
}
