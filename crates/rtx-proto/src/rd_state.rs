// SPDX-License-Identifier: AGPL-3.0-or-later

//! Caller-owned replacement-delta retention shared by live clients and offline readers.

use std::collections::BTreeMap;

use crate::rd::{self, Frame, State, Update};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Applied {
    Published,
    AwaitingReset,
}

/// A sparse current world and its signon baselines, owned by one protocol consumer.
/// Sequence ordering and transaction boundaries around multiple messages belong to the caller.
#[derive(Default)]
pub struct RetainedEntities {
    baselines: BTreeMap<u32, State>,
    entities: BTreeMap<u32, (State, f32)>,
    valid: bool,
    time: f32,
}

impl RetainedEntities {
    /// Replace a baseline after validating it. A bad baseline leaves all retained state unchanged.
    pub fn set_baseline(&mut self, entity: u32, state: State) -> Result<(), &'static str> {
        if entity >= rd::MAX_ENTITIES {
            return Err("replacement baseline entity out of range");
        }
        if !state.origin.is_finite() || !state.angles.is_finite() || !state.viewangles.is_finite() {
            return Err("nonfinite replacement baseline");
        }
        self.baselines.insert(entity, state);
        Ok(())
    }

    /// Forget the entire map, including baselines and recovery state.
    pub fn clear(&mut self) {
        self.baselines.clear();
        self.entities.clear();
        self.valid = false;
        self.time = 0.0;
    }

    /// Request a world reset while keeping the last good world visible.
    pub fn invalidate(&mut self) {
        self.valid = false;
    }

    pub fn can_delta(&self) -> bool {
        self.valid
    }

    /// Server time of the last published frame, including during reset recovery.
    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn get(&self, entity: u32) -> Option<&State> {
        self.entities.get(&entity).map(|(state, _)| state)
    }

    /// Server time of the most recent change to this entity. Omission does not refresh it.
    pub fn updated_at(&self, entity: u32) -> Option<f32> {
        self.entities.get(&entity).map(|(_, time)| *time)
    }

    /// The last good world in ascending entity order, including during reset recovery.
    pub fn current(&self) -> impl Iterator<Item = (u32, &State)> {
        self.entities.iter().map(|(&entity, (state, _))| (entity, state))
    }

    /// Validate the whole frame before publishing any changes. An invalid frame requests a reset
    /// and leaves the previous visible world and its timestamp intact.
    pub fn apply(&mut self, frame: &Frame) -> Result<Applied, &'static str> {
        let reset = match preflight(frame) {
            Ok(reset) => reset,
            Err(reason) => {
                self.invalidate();
                return Err(reason);
            }
        };
        if !self.valid && !reset {
            return Ok(Applied::AwaitingReset);
        }
        if reset {
            self.entities.clear();
        }
        for update in &frame.updates {
            match update {
                Update::ResetWorld => {}
                Update::Remove(entity) => {
                    self.entities.remove(entity);
                }
                Update::Change { entity, delta } => {
                    let state = delta.apply(self.get(*entity), self.baselines.get(entity));
                    self.entities.insert(*entity, (state, frame.time));
                }
            }
        }
        self.time = frame.time;
        self.valid = true;
        Ok(Applied::Published)
    }
}

/// Return whether the frame starts with a reset. No mutation or full-world copy is needed.
fn preflight(frame: &Frame) -> Result<bool, &'static str> {
    if !frame.time.is_finite() {
        return Err("nonfinite replacement time");
    }
    let mut reset = false;
    let mut previous = 0;
    for (index, update) in frame.updates.iter().enumerate() {
        let entity = match update {
            Update::ResetWorld => {
                if index != 0 {
                    return Err("replacement world reset must be first");
                }
                reset = true;
                continue;
            }
            Update::Remove(entity) => *entity,
            Update::Change { entity, delta } => {
                if delta
                    .origin
                    .iter()
                    .chain(delta.angles.iter())
                    .flatten()
                    .any(|value| !value.is_finite())
                {
                    return Err("nonfinite replacement entity change");
                }
                *entity
            }
        };
        if entity == 0 || entity >= rd::MAX_ENTITIES {
            return Err("replacement entity out of range");
        }
        if entity <= previous {
            return Err("replacement entities must be strictly increasing");
        }
        previous = entity;
    }
    Ok(reset)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rd::{Delta, Update};
    use glam::Vec3;

    fn frame(time: f32, updates: Vec<Update>) -> Frame {
        Frame { time, updates }
    }

    fn change(entity: u32, delta: Delta) -> Update {
        Update::Change { entity, delta }
    }

    fn published() -> RetainedEntities {
        let mut retained = RetainedEntities::default();
        retained
            .apply(&frame(1.0, vec![Update::ResetWorld, change(1, Delta::default())]))
            .unwrap();
        retained
    }

    #[test]
    fn requires_initial_reset_before_publishing() {
        let mut retained = RetainedEntities::default();
        assert!(!retained.can_delta());
        assert_eq!(
            retained.apply(&frame(1.0, vec![change(1, Delta::default())])),
            Ok(Applied::AwaitingReset)
        );
        assert!(retained.get(1).is_none());
        assert_eq!(retained.time(), 0.0);
        assert_eq!(
            retained.apply(&frame(2.0, vec![Update::ResetWorld, change(1, Delta::default())])),
            Ok(Applied::Published)
        );
        assert!(retained.can_delta());
        assert_eq!(retained.time(), 2.0);
    }

    #[test]
    fn omitted_entities_persist_and_changed_entities_reset_transients() {
        let mut retained = RetainedEntities::default();
        let delta = Delta {
            origin: [Some(12.0), None, None],
            movement: [1, 2, 3],
            velocity: [8, 16, 24],
            msec: 20,
            ..Default::default()
        };
        retained
            .apply(&frame(1.0, vec![Update::ResetWorld, change(16383, delta)]))
            .unwrap();
        let first = retained.get(16383).unwrap().clone();
        retained.apply(&frame(2.0, vec![])).unwrap();
        assert_eq!(retained.get(16383), Some(&first));
        retained
            .apply(&frame(3.0, vec![change(16383, Delta::default())]))
            .unwrap();
        let updated = retained.get(16383).unwrap();
        assert_eq!(updated.origin, Vec3::new(12.0, 0.0, 0.0));
        assert_eq!(updated.movement, [0; 3]);
        assert_eq!(updated.velocity, [0; 3]);
        assert_eq!(updated.msec, 0);
    }

    #[test]
    fn remove_and_reuse_restore_baseline_and_sorted_iteration() {
        let mut retained = RetainedEntities::default();
        retained
            .set_baseline(
                16383,
                State {
                    modelindex: 9,
                    ..Default::default()
                },
            )
            .unwrap();
        retained
            .apply(&frame(
                1.0,
                vec![
                    Update::ResetWorld,
                    change(1, Delta::default()),
                    change(
                        16383,
                        Delta {
                            reset: true,
                            modelindex: Some(12),
                            ..Default::default()
                        },
                    ),
                ],
            ))
            .unwrap();
        assert_eq!(retained.current().map(|(id, _)| id).collect::<Vec<_>>(), [1, 16383]);
        retained
            .apply(&frame(2.0, vec![Update::Remove(999), Update::Remove(16383)]))
            .unwrap();
        assert!(retained.get(16383).is_none());
        retained
            .apply(&frame(
                3.0,
                vec![change(
                    16383,
                    Delta {
                        reset: true,
                        ..Default::default()
                    },
                )],
            ))
            .unwrap();
        assert_eq!(retained.get(16383).unwrap().modelindex, 9);
    }

    #[test]
    fn invalidation_keeps_visible_world_until_a_reset_arrives() {
        let mut retained = published();
        retained.invalidate();
        assert!(!retained.can_delta());
        assert!(retained.get(1).is_some());
        assert_eq!(
            retained.apply(&frame(2.0, vec![Update::Remove(1)])),
            Ok(Applied::AwaitingReset)
        );
        assert!(retained.get(1).is_some());
        assert_eq!(retained.time(), 1.0);
        assert_eq!(
            retained.apply(&frame(3.0, vec![Update::ResetWorld])),
            Ok(Applied::Published)
        );
        assert!(retained.can_delta());
        assert!(retained.current().next().is_none());
    }

    #[test]
    fn reset_keeps_baselines_but_clear_discards_them() {
        let mut retained = published();
        retained
            .set_baseline(
                2,
                State {
                    modelindex: 7,
                    ..Default::default()
                },
            )
            .unwrap();
        let reset = || {
            vec![
                Update::ResetWorld,
                change(
                    2,
                    Delta {
                        reset: true,
                        ..Default::default()
                    },
                ),
            ]
        };
        retained.apply(&frame(2.0, reset())).unwrap();
        assert!(retained.get(1).is_none());
        assert_eq!(retained.get(2).unwrap().modelindex, 7);
        retained.clear();
        assert!(!retained.can_delta());
        assert_eq!(retained.time(), 0.0);
        assert!(retained.current().next().is_none());
        retained.apply(&frame(1.0, reset())).unwrap();
        assert_eq!(retained.get(2).unwrap().modelindex, 0);
    }

    #[test]
    fn malformed_frame_never_publishes_its_valid_prefix_or_reset() {
        for invalid in [
            vec![change(2, Delta::default()), Update::Remove(16384)],
            vec![
                Update::ResetWorld,
                change(2, Delta::default()),
                change(
                    3,
                    Delta {
                        origin: [None, Some(f32::NAN), None],
                        ..Default::default()
                    },
                ),
            ],
        ] {
            let mut retained = published();
            assert!(retained.apply(&frame(2.0, invalid)).is_err());
            assert!(!retained.can_delta());
            assert_eq!(retained.time(), 1.0);
            assert!(retained.get(1).is_some());
            assert!(retained.get(2).is_none());
        }
    }

    #[test]
    fn rejects_zero_oversized_duplicate_unsorted_and_misplaced_reset_updates() {
        for updates in [
            vec![change(0, Delta::default())],
            vec![Update::Remove(0)],
            vec![change(16384, Delta::default())],
            vec![Update::Remove(u32::MAX)],
            vec![change(2, Delta::default()), Update::Remove(2)],
            vec![Update::Remove(3), change(2, Delta::default())],
            vec![change(2, Delta::default()), Update::ResetWorld],
            vec![Update::ResetWorld, Update::ResetWorld],
        ] {
            let mut retained = published();
            assert!(retained.apply(&frame(2.0, updates)).is_err());
            assert!(retained.get(1).is_some());
            assert!(!retained.can_delta());
        }
    }

    #[test]
    fn rejects_nonfinite_changed_angles_and_invalid_frame_times() {
        for time in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut retained = published();
            assert!(retained.apply(&frame(time, vec![])).is_err());
            assert_eq!(retained.time(), 1.0);
            assert!(!retained.can_delta());
        }
        let mut retained = published();
        assert!(retained
            .apply(&frame(
                2.0,
                vec![change(
                    1,
                    Delta {
                        angles: [Some(f32::NEG_INFINITY), None, None],
                        ..Default::default()
                    }
                )]
            ))
            .is_err());
        assert_eq!(retained.get(1).unwrap().angles, Vec3::ZERO);
    }

    #[test]
    fn finite_time_policy_belongs_to_the_caller() {
        let mut retained = published();
        assert_eq!(retained.apply(&frame(-1.0, vec![])), Ok(Applied::Published));
        assert_eq!(retained.time(), -1.0);
        assert_eq!(retained.apply(&frame(-1.0, vec![])), Ok(Applied::Published));
    }

    #[test]
    fn invalid_baseline_does_not_replace_a_valid_one() {
        let mut retained = RetainedEntities::default();
        retained
            .set_baseline(
                16383,
                State {
                    modelindex: 5,
                    ..Default::default()
                },
            )
            .unwrap();
        assert!(retained.set_baseline(16384, State::default()).is_err());
        for state in [
            State {
                origin: Vec3::splat(f32::NAN),
                ..Default::default()
            },
            State {
                angles: Vec3::splat(f32::INFINITY),
                ..Default::default()
            },
            State {
                viewangles: Vec3::splat(f32::NEG_INFINITY),
                ..Default::default()
            },
        ] {
            assert!(retained.set_baseline(16383, state).is_err());
        }
        retained
            .apply(&frame(
                1.0,
                vec![
                    Update::ResetWorld,
                    change(
                        16383,
                        Delta {
                            reset: true,
                            ..Default::default()
                        },
                    ),
                ],
            ))
            .unwrap();
        assert_eq!(retained.get(16383).unwrap().modelindex, 5);
    }

    #[test]
    fn validates_malformed_frames_even_while_awaiting_reset() {
        let mut retained = published();
        retained.invalidate();
        assert!(retained.apply(&frame(2.0, vec![Update::Remove(16384)])).is_err());
        assert!(retained.get(1).is_some());
        assert_eq!(retained.time(), 1.0);
        assert!(!retained.can_delta());
    }

    #[test]
    fn ordinary_attachment_references_are_distinct_from_world_indices() {
        let mut retained = RetainedEntities::default();
        retained.set_baseline(0, State::default()).unwrap();
        retained
            .apply(&frame(
                1.0,
                vec![
                    Update::ResetWorld,
                    change(
                        1,
                        Delta {
                            tagentity: Some(1 << 22),
                            ..Default::default()
                        },
                    ),
                ],
            ))
            .unwrap();
        assert_eq!(retained.get(1).unwrap().tagentity, 1 << 22);
        assert!(retained.get(0).is_none());
    }

    #[test]
    fn entity_update_times_survive_omission_and_recovery_but_not_removal_or_reset() {
        let mut retained = published();
        assert_eq!(retained.updated_at(1), Some(1.0));
        retained.apply(&frame(2.0, vec![])).unwrap();
        assert_eq!(retained.updated_at(1), Some(1.0));
        retained.apply(&frame(3.0, vec![change(1, Delta::default())])).unwrap();
        assert_eq!(retained.updated_at(1), Some(3.0));
        retained.invalidate();
        assert_eq!(retained.updated_at(1), Some(3.0));
        retained
            .apply(&frame(4.0, vec![Update::ResetWorld, change(2, Delta::default())]))
            .unwrap();
        assert_eq!(retained.updated_at(1), None);
        assert_eq!(retained.updated_at(2), Some(4.0));
        retained.apply(&frame(5.0, vec![Update::Remove(2)])).unwrap();
        assert_eq!(retained.updated_at(2), None);
        retained.apply(&frame(6.0, vec![change(2, Delta::default())])).unwrap();
        assert_eq!(retained.updated_at(2), Some(6.0));
        retained.clear();
        assert_eq!(retained.updated_at(2), None);
    }
}
