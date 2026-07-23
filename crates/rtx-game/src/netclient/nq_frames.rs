// SPDX-License-Identifier: AGPL-3.0-or-later

//! The NetQuake entity store — the counterpart to [`Frames`](super::frames::Frames), and a much
//! simpler thing.
//!
//! QuakeWorld deltas each frame from an *acked previous frame*, so its store is a 64-deep ring with
//! a delta chain. NetQuake deltas each entity from its **baseline**: every update fully respecifies
//! the entity relative to the state it spawned with, and a field the update omits falls back to the
//! baseline, not to last frame. So there is no ring and no chain — just the last state per entity and
//! a note of which server frame last touched it.
//!
//! A NetQuake server writes *every* in-PVS entity every frame (delta-from-baseline, but all of them),
//! so "the entities stamped at the latest `svc_time`" is the same population QuakeWorld's
//! `packetentities` carries: what's visible now. An entity that leaves PVS simply stops being
//! stamped and drops out of [`current`](NqFrames::current), exactly as a QuakeWorld entity absent
//! from a frame does — which is what the world mirror already expects.
//!
//! Losing a datagram costs one frame: `time` doesn't advance, so `current` keeps holding the last
//! complete frame rather than flickering to a partial one.

use glam::Vec3;
use rtx_proto::svc::{Baseline, EntityDelta};

use super::frames::EntityState;

/// Entity-number ceiling. The wire can name a `u16` well past any real entity count, so updates
/// beyond this are dropped rather than trusted to size a Vec.
const MAX_ENTITIES: usize = 2048;

/// "This entity has never been updated." Server frame times are non-negative, so this can't collide.
const NEVER: f32 = -1.0;

/// The last-known state of every entity, indexed by entity number.
pub(crate) struct NqFrames {
    /// The baseline each entity deltas from (`svc_spawnbaseline`).
    baselines: Vec<Baseline>,
    /// The last reconstructed state per entity.
    states: Vec<EntityState>,
    /// The `svc_time` of the frame that last updated each entity ([`NEVER`] if untouched).
    seen_at: Vec<f32>,
    /// The last velocity estimate per entity, from differencing successive origins.
    vel: Vec<Vec3>,
    /// The current frame's `svc_time`.
    time: f32,
    /// The entities of the last settled frame, ascending by number — what [`current`](Self::current)
    /// returns.
    current: Vec<EntityState>,
}

impl Default for NqFrames {
    fn default() -> Self {
        NqFrames {
            baselines: vec![Baseline::default(); MAX_ENTITIES],
            states: vec![EntityState::default(); MAX_ENTITIES],
            seen_at: vec![NEVER; MAX_ENTITIES],
            vel: vec![Vec3::ZERO; MAX_ENTITIES],
            time: NEVER,
            current: Vec::new(),
        }
    }
}

impl NqFrames {
    /// Forget everything — a map change invalidates every baseline and every position.
    pub(crate) fn clear(&mut self) {
        self.baselines.iter_mut().for_each(|b| *b = Baseline::default());
        self.states.iter_mut().for_each(|s| *s = EntityState::default());
        self.seen_at.iter_mut().for_each(|t| *t = NEVER);
        self.vel.iter_mut().for_each(|v| *v = Vec3::ZERO);
        self.time = NEVER;
        self.current.clear();
    }

    /// Record an entity's baseline. Later updates for it delta from here.
    pub(crate) fn set_baseline(&mut self, number: u16, b: Baseline) {
        if let Some(slot) = self.baselines.get_mut(number as usize) {
            *slot = b;
        }
    }

    /// Open a new server frame (`svc_time`). Entities updated after this are stamped with it.
    pub(crate) fn begin_frame(&mut self, time: f32) {
        self.time = time;
    }

    /// Apply one entity update: rebuild from the baseline, overlay the fields the update carried, and
    /// stamp it at the current frame time. Absent fields fall back to the baseline (the NetQuake
    /// rule), not to the previous frame.
    pub(crate) fn apply(&mut self, d: &EntityDelta) {
        let n = d.number as usize;
        if n >= MAX_ENTITIES {
            return; // a wire number past any real entity — don't trust it to index
        }
        let mut st = EntityState::from_baseline(d.number, &self.baselines[n]);
        st.apply(d);
        // Estimate velocity by differencing this origin against the last, over the elapsed frames.
        if self.seen_at[n] >= 0.0 && self.time > self.seen_at[n] {
            self.vel[n] = (st.origin - self.states[n].origin) / (self.time - self.seen_at[n]);
        }
        self.states[n] = st;
        self.seen_at[n] = self.time;
    }

    /// Publish the current frame: the entities stamped at the latest `svc_time`. Called once the
    /// datagram (a whole frame) has been drained, so a half-received frame never shows.
    pub(crate) fn settle(&mut self) {
        self.current.clear();
        if self.time < 0.0 {
            return;
        }
        for n in 0..MAX_ENTITIES {
            if self.seen_at[n] == self.time {
                self.current.push(self.states[n]);
            }
        }
    }

    /// The entities visible this frame, ascending by number — the slice the world mirror consumes.
    pub(crate) fn current(&self) -> &[EntityState] {
        &self.current
    }

    /// The estimated velocity of an entity, or `None` if it's never been seen. NetQuake sends no
    /// velocity for other entities, so a bot leading a moving target derives it here. An entity seen
    /// only once reads as zero (nothing to difference yet) — a reasonable lead on a first sighting.
    pub(crate) fn velocity_of(&self, number: u16) -> Option<Vec3> {
        let n = number as usize;
        if n < MAX_ENTITIES && self.seen_at[n] >= 0.0 {
            Some(self.vel[n])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline_at(origin: Vec3, model: u16) -> Baseline {
        Baseline {
            modelindex: model,
            origin,
            ..Default::default()
        }
    }

    fn delta(number: u16, origin: [Option<f32>; 3]) -> EntityDelta {
        EntityDelta {
            number,
            origin,
            ..Default::default()
        }
    }

    /// An update carrying only origin.x leaves y/z at the baseline — the NetQuake delta-from-baseline
    /// rule, the inverse of QuakeWorld's carry-forward. Getting this backwards would freeze doors at
    /// their spawn or un-move compressed entities.
    #[test]
    fn absent_fields_fall_back_to_baseline() {
        let mut f = NqFrames::default();
        f.set_baseline(5, baseline_at(Vec3::new(10.0, 20.0, 30.0), 7));
        f.begin_frame(1.0);
        f.apply(&delta(5, [Some(99.0), None, None]));
        f.settle();

        let e = f.current().iter().find(|e| e.number == 5).unwrap();
        assert_eq!(e.origin, Vec3::new(99.0, 20.0, 30.0)); // y/z from baseline
        assert_eq!(e.model, 7); // model absent → baseline
    }

    /// `current` holds the entities of the latest frame; an entity that stops being updated (leaves
    /// PVS) drops out, and a fresh frame supersedes the old one.
    #[test]
    fn current_is_the_latest_frame_only() {
        let mut f = NqFrames::default();
        f.set_baseline(1, baseline_at(Vec3::ZERO, 1));
        f.set_baseline(2, baseline_at(Vec3::ZERO, 1));

        f.begin_frame(1.0);
        f.apply(&delta(1, [Some(1.0), None, None]));
        f.apply(&delta(2, [Some(2.0), None, None]));
        f.settle();
        assert_eq!(f.current().len(), 2);

        // Next frame mentions only entity 1; entity 2 left PVS and drops out.
        f.begin_frame(2.0);
        f.apply(&delta(1, [Some(5.0), None, None]));
        f.settle();
        let cur = f.current();
        assert_eq!(cur.len(), 1);
        assert_eq!(cur[0].number, 1);
        assert_eq!(cur[0].origin.x, 5.0);
    }

    /// Velocity is differenced from successive origins over the elapsed server time.
    #[test]
    fn velocity_from_successive_origins() {
        let mut f = NqFrames::default();
        f.set_baseline(3, baseline_at(Vec3::ZERO, 1));
        f.begin_frame(1.0);
        f.apply(&delta(3, [Some(0.0), Some(0.0), Some(0.0)]));
        f.begin_frame(1.5); // half a second later
        f.apply(&delta(3, [Some(50.0), Some(0.0), Some(0.0)]));
        // 50 units in 0.5s = 100 u/s.
        assert_eq!(f.velocity_of(3), Some(Vec3::new(100.0, 0.0, 0.0)));
        assert_eq!(f.velocity_of(99), None); // never seen
    }

    /// A wire entity number past the ceiling is dropped, not a panic and not an allocation.
    #[test]
    fn oversize_entity_number_is_ignored() {
        let mut f = NqFrames::default();
        f.begin_frame(1.0);
        f.apply(&delta(30000, [Some(1.0), None, None]));
        f.settle();
        assert!(f.current().is_empty());
        assert_eq!(f.velocity_of(30000), None);
    }
}
