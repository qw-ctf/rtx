// SPDX-License-Identifier: AGPL-3.0-or-later

//! Entity snapshots, and the delta chain that reconstructs them.
//!
//! The server almost never sends the world; it sends the *difference* from a frame it believes we
//! already have. So a client that wants to know where anything is must keep the last 64 frames and
//! be able to say, of any one of them, "yes, I have that". Lose track and the deltas decode into
//! plausible nonsense — entities drifting, sliding through walls — rather than failing outright.
//!
//! Three rules make that work:
//!
//! **Every update names its base.** `svc_deltapacketentities` carries the sequence it was built
//! from. If we don't have that frame, the update is unusable — not "mostly usable" — and the only
//! correct response is to stop asking for deltas until the server sends a full update.
//!
//! **Absence means unchanged.** An entity the update doesn't mention still exists, exactly as it
//! was. That's why [`Snapshot`]s are kept whole rather than as a list of changes.
//!
//! **A new entity deltas from its baseline, not from nothing.** Baselines arrive once, at signon,
//! and the server assumes we kept them — an entity appearing mid-game sends only what differs from
//! the baseline it was spawned with.
//!
//! # Why age, and not just presence
//!
//! We key snapshots by the **server's** sequence and store the full 32-bit value, rather than
//! ezQuake's `cl.frames[]`, which indexes by the client's outgoing sequence on send and the
//! server's incoming sequence on receive — two different things that agree only because both ends
//! send one packet per frame.
//!
//! But keying better is not enough, because **having a frame is not the same as it being usable**.
//! The `from` byte is 8 bits, so it names a frame only modulo 256 — `1` and `257` are the same
//! byte, and land in the same slot. Which one it means is settled by *age*, not by the byte: the
//! server can only delta from a base we advertised, and we always advertise the newest frame we
//! have, so the young candidate is the one meant. Received sequences gap freely under loss, so age
//! is something to check rather than assume.
//!
//! Hence [`UPDATE_BACKUP`] appears twice as an **age** check, mirroring ezQuake's two: once when
//! resolving a base (is this candidate young enough to be the frame that byte refers to, or is it
//! something ancient that merely matches?), and once when advertising one (has the server's own
//! 64-deep ring rolled past what we're about to ask for?). Those checks are the safety net; the
//! low-byte comparison alone never was.
//!
//! One deliberate divergence: when an update turns out to be unusable we keep the **last good
//! snapshot** and stop requesting deltas, rather than blanking the world. ezQuake does the same on
//! its "too old" path (`cl_ents.c`: *"Don't clear cl.validsequence, so that frames can still be
//! rendered"*), and for a bot it matters more than for a renderer: entities a second stale are a
//! bot shooting slightly behind an enemy, whereas no entities at all is a bot concluding the map
//! is empty and wandering off.

use glam::Vec3;
use rtx_proto::svc::{Baseline, EntityDelta};

/// How many frames the server may delta against (`UPDATE_BACKUP`). Power of two.
pub(crate) const UPDATE_BACKUP: usize = 64;
const UPDATE_MASK: usize = UPDATE_BACKUP - 1;

/// The largest entity number the negotiated extensions allow (`ENTITYDBL` + `ENTITYDBL2`).
const MAX_ENTITIES: usize = 2048;

/// One entity's full state, as reconstructed from a baseline plus a chain of deltas.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct EntityState {
    /// Server entity number.
    pub number: u16,
    /// Index into the model list.
    pub model: u16,
    /// Animation frame. 16-bit to match [`Baseline`]/[`EntityDelta`] (NetQuake 666 large frames).
    pub frame: u16,
    /// Colormap.
    pub colormap: u8,
    /// Skin.
    pub skin: u8,
    /// Effect bits.
    pub effects: u8,
    /// Position.
    pub origin: Vec3,
    /// Orientation.
    pub angles: Vec3,
}

impl EntityState {
    /// The state an entity starts from when the server first mentions it.
    pub(crate) fn from_baseline(number: u16, b: &Baseline) -> Self {
        EntityState {
            number,
            model: b.modelindex,
            frame: b.frame,
            colormap: b.colormap,
            skin: b.skinnum,
            effects: 0,
            origin: b.origin,
            angles: b.angles,
        }
    }

    /// Apply the fields an update actually carried, leaving the rest alone.
    pub(crate) fn apply(&mut self, d: &EntityDelta) {
        self.number = d.number;
        if let Some(v) = d.model {
            self.model = v;
        }
        if let Some(v) = d.frame {
            self.frame = v;
        }
        if let Some(v) = d.colormap {
            self.colormap = v;
        }
        if let Some(v) = d.skin {
            self.skin = v;
        }
        if let Some(v) = d.effects {
            self.effects = v;
        }
        for i in 0..3 {
            if let Some(v) = d.origin[i] {
                self.origin[i] = v;
            }
            if let Some(v) = d.angles[i] {
                self.angles[i] = v;
            }
        }
    }
}

/// Every entity the server told us about in one frame, ordered by entity number (the server sends
/// them that way, and the merge relies on it).
#[derive(Clone, Debug, Default)]
pub(crate) struct Snapshot {
    /// The server sequence this is the state at.
    pub sequence: u32,
    /// The entities, ascending by number.
    pub entities: Vec<EntityState>,
}

/// The frame we're asking the server to build the next update from.
#[derive(Clone, Copy, Debug)]
struct DeltaBase {
    /// The server sequence we're naming.
    sequence: u32,
    /// Our outgoing sequence when we adopted it — the clock against which it ages out of the
    /// *server's* ring, which is the same 64 frames deep as ours.
    outgoing: u32,
}

/// The last [`UPDATE_BACKUP`] snapshots, plus the baselines new entities start from.
pub(crate) struct Frames {
    ring: Vec<Option<Snapshot>>,
    baselines: Vec<Baseline>,
    /// The newest good snapshot — what [`current`](Frames::current) reads. Survives an unusable
    /// update: stale entities beat no entities.
    valid: Option<u32>,
    /// What to ask the server to delta against, if anything. Separate from `valid` because losing
    /// the ability to *request* a delta is not the same as losing the ability to *see* — ezQuake
    /// keeps the same two apart as `cl.delta_sequence` and `cl.validsequence`.
    delta_base: Option<DeltaBase>,
}

impl Default for Frames {
    fn default() -> Self {
        Frames {
            ring: vec![None; UPDATE_BACKUP],
            baselines: vec![Baseline::default(); MAX_ENTITIES],
            valid: None,
            delta_base: None,
        }
    }
}

/// What happened when an update was applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Applied {
    /// Applied; this is the new current frame.
    Ok,
    /// The update named a base we don't have, or one too old to still mean what it says. Nothing in
    /// it is usable — so we keep the last good snapshot and stop requesting deltas, which is how a
    /// client asks for a full update.
    Stale,
}

impl Frames {
    /// Forget everything. Called on a new map: baselines, snapshots and sequences all belong to the
    /// level that just ended.
    pub(crate) fn clear(&mut self) {
        self.ring.iter_mut().for_each(|s| *s = None);
        self.baselines.iter_mut().for_each(|b| *b = Baseline::default());
        self.valid = None;
        self.delta_base = None;
    }

    /// Record a baseline from signon.
    pub(crate) fn set_baseline(&mut self, number: u16, b: Baseline) {
        if let Some(slot) = self.baselines.get_mut(number as usize) {
            *slot = b;
        }
    }

    /// Remember a baseline that arrived as an entity *delta* rather than a fixed record.
    ///
    /// `svc_spawnbaseline2` is the FTE form, and it reuses the entity-delta encoding: it says only
    /// what differs from nothing. Folding it onto an empty baseline gets the same thing the vanilla
    /// message states outright — which is the point of the extension, since most baselines differ
    /// from nothing in three or four fields and paying for all nine per entity is what it exists to
    /// avoid.
    pub(crate) fn set_baseline_delta(&mut self, number: u16, delta: &rtx_proto::svc::EntityDelta) {
        let mut b = Baseline::default();
        if let Some(v) = delta.model {
            b.modelindex = v;
        }
        if let Some(v) = delta.frame {
            b.frame = v;
        }
        if let Some(v) = delta.colormap {
            b.colormap = v;
        }
        if let Some(v) = delta.skin {
            b.skinnum = v;
        }
        for i in 0..3 {
            if let Some(v) = delta.origin[i] {
                b.origin[i] = v;
            }
            if let Some(v) = delta.angles[i] {
                b.angles[i] = v;
            }
        }
        self.set_baseline(number, b);
    }

    /// The frame to ask the server to delta against, given the sequence of the packet we're about
    /// to send. `None` asks for a full update.
    ///
    /// It takes `outgoing` because a base expires: the server's ring is [`UPDATE_BACKUP`] deep too,
    /// so once we've sent that many packets since adopting a base, the server can no longer have it
    /// — and asking anyway makes it delta from whatever now occupies that slot. This is ezQuake's
    /// `outgoing_sequence - validsequence >= UPDATE_BACKUP - 1` check.
    pub(crate) fn delta_sequence(&self, outgoing: u32) -> Option<u8> {
        let d = self.delta_base?;
        let age = outgoing.wrapping_sub(d.outgoing);
        (age < (UPDATE_BACKUP - 1) as u32).then_some(d.sequence as u8)
    }

    /// The current entity set, or empty before the first update lands.
    pub(crate) fn current(&self) -> &[EntityState] {
        self.valid
            .and_then(|s| self.ring[s as usize & UPDATE_MASK].as_ref())
            .map(|s| s.entities.as_slice())
            .unwrap_or(&[])
    }

    /// Fold one `svc_packetentities` / `svc_deltapacketentities` into a new snapshot at `sequence`.
    ///
    /// `delta_from` is the sequence byte the server said it built the update from (`None` for a
    /// full update), and `outgoing` is our netchan's next outgoing sequence, which dates the base
    /// we adopt so [`delta_sequence`](Self::delta_sequence) can retire it.
    pub(crate) fn apply(
        &mut self,
        sequence: u32,
        outgoing: u32,
        delta_from: Option<u8>,
        updates: &[EntityDelta],
    ) -> Applied {
        let base: &[EntityState] = match delta_from {
            None => &[],
            Some(from) => {
                // Two conditions, and the second is the one that matters. The low byte narrows the
                // ring to one candidate — but `1` and `257` share both a slot and a low byte, so a
                // candidate is only *the* frame if it's also young enough to be nameable. Received
                // sequences gap under loss, so age can't be assumed from presence.
                match self.ring[from as usize & UPDATE_MASK].as_ref() {
                    Some(s)
                        if s.sequence as u8 == from
                            && sequence.wrapping_sub(s.sequence) < (UPDATE_BACKUP - 1) as u32 =>
                    {
                        &s.entities
                    }
                    _ => {
                        // Keep `valid`: our last snapshot is still a true picture, just an old one.
                        self.delta_base = None;
                        return Applied::Stale;
                    }
                }
            }
        };

        let entities = merge(base, updates, &self.baselines);
        self.ring[sequence as usize & UPDATE_MASK] = Some(Snapshot { sequence, entities });
        self.valid = Some(sequence);
        self.delta_base = Some(DeltaBase { sequence, outgoing });
        Applied::Ok
    }
}

/// Merge one frame's updates over the previous frame's entities.
///
/// Both are ascending by entity number, so this is a merge join: an update either revises an entity
/// the base had, introduces one it didn't (which deltas from its baseline), or removes one. Any
/// entity the updates skip past is carried forward untouched — that's what "absence means
/// unchanged" means, and it's why the base has to be a whole snapshot.
fn merge(base: &[EntityState], updates: &[EntityDelta], baselines: &[Baseline]) -> Vec<EntityState> {
    let mut out: Vec<EntityState> = Vec::with_capacity(base.len().max(updates.len()));
    let mut oi = 0;

    for d in updates {
        // Carry forward everything the server skipped over.
        while oi < base.len() && base[oi].number < d.number {
            out.push(base[oi]);
            oi += 1;
        }

        let existing = if oi < base.len() && base[oi].number == d.number {
            let e = base[oi];
            oi += 1;
            Some(e)
        } else {
            None
        };

        if d.remove {
            continue; // dropped from the new snapshot, and its base entry consumed above
        }

        // An entity the base doesn't have is new, and deltas from the baseline it spawned with —
        // not from zero, which would put it at the origin with no model.
        let mut e = existing.unwrap_or_else(|| {
            let b = baselines.get(d.number as usize).copied().unwrap_or_default();
            EntityState::from_baseline(d.number, &b)
        });
        e.apply(d);
        out.push(e);
    }

    out.extend_from_slice(&base[oi..]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn delta(number: u16) -> EntityDelta {
        EntityDelta {
            number,
            ..Default::default()
        }
    }

    fn at(number: u16, x: f32) -> EntityDelta {
        let mut d = delta(number);
        d.origin[0] = Some(x);
        d
    }

    fn origins(f: &Frames) -> Vec<(u16, f32)> {
        f.current().iter().map(|e| (e.number, e.origin.x)).collect()
    }

    /// A full update needs no base and establishes one.
    #[test]
    fn full_update_starts_the_chain() {
        let mut f = Frames::default();
        assert_eq!(f.delta_sequence(1), None, "nothing to delta against yet");
        assert!(f.current().is_empty());

        assert_eq!(f.apply(10, 10, None, &[at(3, 100.0), at(7, 200.0)]), Applied::Ok);
        assert_eq!(origins(&f), vec![(3, 100.0), (7, 200.0)]);
        assert_eq!(f.delta_sequence(11), Some(10));
    }

    /// The rule the whole module exists for: an entity the update doesn't mention is unchanged,
    /// not gone. Getting this wrong makes the world flicker out of existence between frames.
    #[test]
    fn unmentioned_entities_are_carried_forward() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 100.0), at(7, 200.0), at(9, 300.0)]);

        // Only entity 7 moved.
        assert_eq!(f.apply(2, 2, Some(1), &[at(7, 250.0)]), Applied::Ok);
        assert_eq!(origins(&f), vec![(3, 100.0), (7, 250.0), (9, 300.0)]);
    }

    /// A delta touches only the fields it carries; everything else survives from the base.
    #[test]
    fn delta_revises_only_what_it_carries() {
        let mut f = Frames::default();
        let mut d = delta(5);
        d.model = Some(11);
        d.frame = Some(2);
        d.origin = [Some(1.0), Some(2.0), Some(3.0)];
        d.angles = [None, Some(90.0), None];
        f.apply(1, 1, None, &[d]);

        // Move it on one axis only.
        let mut d = delta(5);
        d.origin[0] = Some(50.0);
        f.apply(2, 2, Some(1), &[d]);

        let e = f.current()[0];
        assert_eq!(e.origin, Vec3::new(50.0, 2.0, 3.0), "untouched axes persist");
        assert_eq!(e.angles.y, 90.0);
        assert_eq!(e.model, 11, "model persists across a delta that omits it");
        assert_eq!(e.frame, 2);
    }

    /// An entity the base doesn't have is new, and starts from the baseline it was spawned with —
    /// otherwise it would appear at the world origin wearing no model.
    #[test]
    fn new_entity_deltas_from_its_baseline() {
        let mut f = Frames::default();
        f.set_baseline(
            42,
            Baseline {
                modelindex: 17,
                skinnum: 3,
                origin: Vec3::new(10.0, 20.0, 30.0),
                ..Default::default()
            },
        );
        f.apply(1, 1, None, &[]);

        // The update mentions 42 for the first time, carrying only a new x.
        f.apply(2, 2, Some(1), &[at(42, 99.0)]);
        let e = f.current()[0];
        assert_eq!(e.number, 42);
        assert_eq!(e.model, 17, "from the baseline");
        assert_eq!(e.skin, 3);
        assert_eq!(
            e.origin,
            Vec3::new(99.0, 20.0, 30.0),
            "x from the delta, y/z from the baseline"
        );
    }

    /// Removal takes the entity out of the new snapshot and doesn't disturb its neighbours.
    #[test]
    fn remove_drops_the_entity() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0), at(7, 2.0), at(9, 3.0)]);

        let mut gone = delta(7);
        gone.remove = true;
        f.apply(2, 2, Some(1), &[gone]);
        assert_eq!(origins(&f), vec![(3, 1.0), (9, 3.0)]);

        // And it stays gone on the next delta.
        f.apply(3, 3, Some(2), &[]);
        assert_eq!(origins(&f), vec![(3, 1.0), (9, 3.0)]);
    }

    /// Removing something that isn't there is a no-op, not a corruption — it happens when our
    /// picture and the server's briefly disagree.
    #[test]
    fn removing_an_absent_entity_is_harmless() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0)]);
        let mut gone = delta(99);
        gone.remove = true;
        f.apply(2, 2, Some(1), &[gone]);
        assert_eq!(origins(&f), vec![(3, 1.0)]);
    }

    /// An update built on a frame we don't have is unusable — applying it anyway would decode into
    /// plausible nonsense. But the *last good snapshot* is still a true picture, just an old one, so
    /// it survives: a bot with second-old entities shoots slightly behind; a bot with no entities
    /// concludes the map is empty and wanders off.
    #[test]
    fn delta_from_a_frame_we_lack_is_stale_but_the_world_survives() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0)]);

        assert_eq!(f.apply(2, 2, Some(200), &[at(3, 2.0)]), Applied::Stale);
        assert_eq!(f.delta_sequence(3), None, "stop asking for deltas until a full update");
        assert_eq!(origins(&f), vec![(3, 1.0)], "but keep the last frame we did understand");

        // A full update re-establishes the chain.
        assert_eq!(f.apply(3, 3, None, &[at(3, 5.0)]), Applied::Ok);
        assert_eq!(f.delta_sequence(4), Some(3));
        assert_eq!(origins(&f), vec![(3, 5.0)]);
    }

    /// A base older than the ring must be refused however it's named. Presence isn't recency:
    /// received sequences gap under loss, so a slot can hold something ancient.
    ///
    /// Regression: this accepted a 399-frame-old base and called it fresh.
    #[test]
    fn an_ancient_base_is_refused() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0)]);
        assert_eq!(
            f.apply(400, 400, Some(1), &[]),
            Applied::Stale,
            "frame 1 is 399 frames old"
        );
        assert_eq!(origins(&f), vec![(3, 1.0)]);

        // The boundary: 62 frames back is still nameable, 63 is not (UPDATE_BACKUP - 1).
        let mut f = Frames::default();
        f.apply(100, 100, None, &[at(3, 1.0)]);
        assert_eq!(f.apply(162, 162, Some(100), &[]), Applied::Ok);
        let mut f = Frames::default();
        f.apply(100, 100, None, &[at(3, 1.0)]);
        assert_eq!(f.apply(163, 163, Some(100), &[]), Applied::Stale);
    }

    /// Frames 256 apart share a slot *and* a low byte — `1` and `257` are indistinguishable in the
    /// `from` byte. The right resolution is the **young** one, and that isn't a lucky guess: the
    /// server can only delta from a base we advertised, and we always advertise the newest frame we
    /// have. Once 257 arrives, `from=1` *means* 257, because 257 is what we'd have asked for.
    ///
    /// This is the case that makes the age check necessary rather than the low byte sufficient: age
    /// is what says which of the two a byte refers to.
    #[test]
    fn an_ambiguous_byte_resolves_to_the_frame_we_would_have_asked_for() {
        assert_eq!(257 & UPDATE_MASK, 1 & UPDATE_MASK, "same slot");
        assert_eq!(257u32 as u8, 1u32 as u8, "and the same low byte");

        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0)]);
        f.apply(257, 257, None, &[at(3, 257.0)]);

        // What we'd advertise is 257 — whose byte is 1. So `from=1` names 257, and resolving to it
        // is correct.
        assert_eq!(f.delta_sequence(258), Some(1), "we ask for 257, spelled `1`");
        assert_eq!(f.apply(258, 258, Some(1), &[]), Applied::Ok);
        assert_eq!(
            origins(&f),
            vec![(3, 257.0)],
            "resolved to 257, not to the frame it replaced"
        );
    }

    /// A base expires on the send side too: the server's ring is the same 64 frames deep, so once
    /// we've sent that many packets since adopting one, the server can't have it either — and
    /// asking anyway makes it delta from whatever now occupies the slot.
    #[test]
    fn a_base_expires_once_the_servers_ring_would_have_rolled_past_it() {
        let mut f = Frames::default();
        f.apply(10, 100, None, &[at(3, 1.0)]);

        assert_eq!(f.delta_sequence(101), Some(10), "fresh");
        assert_eq!(f.delta_sequence(100 + 62), Some(10), "still inside the server's ring");
        assert_eq!(f.delta_sequence(100 + 63), None, "the server's ring has rolled past it");
        assert_eq!(f.delta_sequence(100 + 1000), None);

        // Seeing a fresh frame re-dates the base.
        f.apply(11, 200, None, &[at(3, 2.0)]);
        assert_eq!(f.delta_sequence(201), Some(11));
    }

    /// A frame older than the ring is gone, and the low-byte match must not mistake the newer frame
    /// now occupying its slot for it.
    ///
    /// Worth spelling out why the match is exact rather than lucky. A slot always holds the newest
    /// sequence `S` with `S & 63 == slot`, so `S` is within 64 of the present. The server can only
    /// name a frame modulo 256 (the `from` byte), and it only ever deltas against a recent one — so
    /// the candidate and the wanted frame both live in a window narrower than 256, where agreeing
    /// modulo 256 means being the same frame. That's the same guarantee ezQuake gets from comparing
    /// `from & UPDATE_MASK`, without depending on its two sequence spaces lining up.
    #[test]
    fn wrapped_frames_are_not_mistaken_for_fresh_ones() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 1.0)]);
        // Fill the ring right past the wrap: sequence 65 now occupies slot 1, where 1 used to be.
        for seq in 2..=70 {
            assert_eq!(f.apply(seq, seq, None, &[at(3, seq as f32)]), Applied::Ok);
        }
        assert_eq!(
            f.apply(71, 71, Some(1), &[]),
            Applied::Stale,
            "1 was evicted by 65 — same slot"
        );

        // 65 itself is still there and still usable: eviction is per slot, not wholesale.
        f.apply(72, 72, None, &[at(3, 72.0)]);
        assert_eq!(f.apply(73, 73, Some(65), &[]), Applied::Ok);

        // Every sequence still in the ring resolves to itself, and nothing evicted resolves at all.
        for seq in 74..=200u32 {
            f.apply(seq, seq, None, &[at(3, seq as f32)]);
            assert_eq!(
                f.apply(seq + 1, seq + 1, Some(seq as u8), &[]),
                Applied::Ok,
                "{seq} should resolve"
            );
            // 64 back is the oldest live frame; 65 back has been evicted by the wrap.
            let evicted = (seq - 65) as u8;
            assert_eq!(
                f.apply(seq + 2, seq + 2, Some(evicted), &[]),
                Applied::Stale,
                "{evicted} is gone"
            );
            f.apply(seq + 3, seq + 3, None, &[at(3, seq as f32)]); // re-establish after the stale
        }
    }

    /// Deltas chain: each frame builds on the last, over many frames, without drift.
    #[test]
    fn long_delta_chain_holds() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(3, 0.0), at(4, 1000.0)]);
        for seq in 2..200u32 {
            // Entity 3 moves every frame; entity 4 is never mentioned again.
            assert_eq!(
                f.apply(seq, seq, Some((seq - 1) as u8), &[at(3, seq as f32)]),
                Applied::Ok
            );
        }
        assert_eq!(origins(&f), vec![(3, 199.0), (4, 1000.0)]);
    }

    /// A new map invalidates everything: baselines, snapshots and the delta chain all belonged to
    /// the level that ended.
    #[test]
    fn clear_forgets_the_level() {
        let mut f = Frames::default();
        f.set_baseline(
            5,
            Baseline {
                modelindex: 9,
                ..Default::default()
            },
        );
        f.apply(1, 1, None, &[at(5, 1.0)]);

        f.clear();
        assert_eq!(f.delta_sequence(2), None);
        assert!(f.current().is_empty());

        // The baseline is gone too — model indices are per-map.
        f.apply(1, 1, None, &[at(5, 1.0)]);
        assert_eq!(f.current()[0].model, 0);
    }

    /// Entities stay sorted through inserts and removes, since the merge is a merge join and would
    /// quietly misbehave on unsorted input.
    #[test]
    fn snapshots_stay_sorted() {
        let mut f = Frames::default();
        f.apply(1, 1, None, &[at(2, 0.0), at(50, 0.0)]);
        f.apply(2, 2, Some(1), &[at(1, 0.0), at(10, 0.0), at(99, 0.0)]);
        let nums: Vec<u16> = f.current().iter().map(|e| e.number).collect();
        assert_eq!(nums, vec![1, 2, 10, 50, 99]);
        assert!(nums.windows(2).all(|w| w[0] < w[1]));
    }
}
