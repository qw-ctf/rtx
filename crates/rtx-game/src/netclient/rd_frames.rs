// SPDX-License-Identifier: AGPL-3.0-or-later

//! The live client's replacement-delta view. Retention is shared with offline readers; this
//! wrapper adds a reusable mirror projection and the accepted sequence used by `clc_delta`.

use rtx_proto::rd::{Delta, Frame};
use rtx_proto::rd_state::{Applied, RetainedEntities};
use rtx_proto::svc::Baseline;

use super::frames::EntityState;

#[derive(Default)]
pub(crate) struct RdFrames {
    retained: RetainedEntities,
    current: Vec<EntityState>,
    sequence: Option<u32>,
}

impl RdFrames {
    pub(crate) fn set_baseline(&mut self, entity: u32, baseline: Baseline) -> Result<(), &'static str> {
        let delta = Delta {
            modelindex: Some(baseline.modelindex),
            frame: Some(baseline.frame),
            skinnum: Some(baseline.skinnum as i16),
            colormap: Some(baseline.colormap),
            origin: baseline.origin.to_array().map(Some),
            angles: baseline.angles.to_array().map(Some),
            ..Default::default()
        };
        self.set_replacement_baseline(entity, &delta)
    }

    pub(crate) fn set_replacement_baseline(&mut self, entity: u32, delta: &Delta) -> Result<(), &'static str> {
        self.retained.set_baseline(entity, delta.apply(None, None))
    }

    pub(crate) fn apply(&mut self, sequence: u32, frame: &Frame) -> Result<Applied, &'static str> {
        let applied = match self.retained.apply(frame) {
            Ok(applied) => applied,
            Err(reason) => {
                self.sequence = None;
                return Err(reason);
            }
        };
        if applied == Applied::AwaitingReset {
            self.sequence = None;
            return Ok(applied);
        }
        self.current.clear();
        self.current
            .extend(self.retained.current().filter_map(|(entity, state)| {
                // Players are projected through player_info instead, where prediction and evidence
                // consume their full RD state. They must not also become world-mirror entities.
                (entity > 32).then_some(EntityState {
                    number: entity as u16,
                    model: state.modelindex,
                    frame: state.frame,
                    colormap: state.colormap,
                    skin: state.skinnum,
                    effects: state.effects,
                    origin: state.origin,
                    angles: state.angles,
                })
            }));
        self.sequence = Some(sequence);
        Ok(applied)
    }

    pub(crate) fn current(&self) -> &[EntityState] {
        &self.current
    }

    pub(crate) fn retained(&self) -> &RetainedEntities {
        &self.retained
    }

    pub(crate) fn delta_request(&self) -> Option<u8> {
        self.sequence
            .filter(|_| self.retained.can_delta())
            .map(|sequence| sequence as u8)
    }

    pub(crate) fn clear(&mut self) {
        self.retained.clear();
        self.current.clear();
        self.sequence = None;
    }

    pub(crate) fn invalidate(&mut self) {
        self.retained.invalidate();
        self.sequence = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use rtx_proto::rd::Update;

    fn frame(time: f32, updates: Vec<Update>) -> Frame {
        Frame { time, updates }
    }

    fn change(entity: u32, delta: Delta) -> Update {
        Update::Change { entity, delta }
    }

    #[test]
    fn projects_high_entities_without_losing_signed_skin_or_effect_bits() {
        let mut frames = RdFrames::default();
        assert_eq!(frames.delta_request(), None);
        assert_eq!(
            frames.apply(
                0x1234,
                &frame(
                    1.0,
                    vec![
                        Update::ResetWorld,
                        change(1, Delta::default()),
                        change(32, Delta::default()),
                        change(33, Delta::default()),
                        change(
                            16383,
                            Delta {
                                modelindex: Some(511),
                                frame: Some(300),
                                skinnum: Some(-3),
                                effects: Some(0x12345678),
                                origin: [Some(40.0), None, None],
                                ..Default::default()
                            }
                        )
                    ]
                )
            ),
            Ok(Applied::Published)
        );
        assert_eq!(frames.delta_request(), Some(0x34));
        assert_eq!(
            frames.current().iter().map(|e| e.number).collect::<Vec<_>>(),
            [33, 16383]
        );
        let entity = frames.current()[1];
        assert_eq!(entity.model, 511);
        assert_eq!(entity.frame, 300);
        assert_eq!(entity.skin, -3);
        assert_eq!(entity.effects, 0x12345678);
        assert_eq!(entity.origin, Vec3::new(40.0, 0.0, 0.0));
        assert!(
            frames.retained().get(1).is_some(),
            "players remain available for prediction"
        );
        assert_eq!(frames.retained().updated_at(1), Some(1.0));
    }

    #[test]
    fn omitted_entities_persist_until_remove_and_reuse_restores_rd_baseline() {
        let mut frames = RdFrames::default();
        frames
            .set_replacement_baseline(
                16383,
                &Delta {
                    modelindex: Some(9),
                    ..Default::default()
                },
            )
            .unwrap();
        frames
            .apply(
                1,
                &frame(
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
                ),
            )
            .unwrap();
        frames.apply(2, &frame(2.0, vec![])).unwrap();
        assert_eq!(frames.current()[0].model, 9);
        assert_eq!(frames.retained().updated_at(16383), Some(1.0));
        frames.apply(3, &frame(3.0, vec![Update::Remove(16383)])).unwrap();
        assert!(frames.current().is_empty());
        assert_eq!(frames.retained().updated_at(16383), None);
        frames
            .apply(
                4,
                &frame(
                    4.0,
                    vec![change(
                        16383,
                        Delta {
                            reset: true,
                            ..Default::default()
                        },
                    )],
                ),
            )
            .unwrap();
        assert_eq!(frames.current()[0].model, 9);
        assert_eq!(frames.retained().updated_at(16383), Some(4.0));
    }

    #[test]
    fn invalid_frames_preserve_projection_and_request_reset() {
        let mut frames = RdFrames::default();
        frames
            .apply(
                5,
                &frame(1.0, vec![Update::ResetWorld, change(16383, Delta::default())]),
            )
            .unwrap();
        let previous = frames.current().to_vec();
        assert!(frames
            .apply(
                6,
                &frame(2.0, vec![Update::ResetWorld, change(16384, Delta::default())])
            )
            .is_err());
        assert_eq!(frames.current(), previous);
        assert_eq!(frames.delta_request(), None);
        assert_eq!(
            frames.apply(7, &frame(3.0, vec![Update::Remove(16383)])),
            Ok(Applied::AwaitingReset)
        );
        assert_eq!(frames.current(), previous);
        frames.apply(8, &frame(4.0, vec![Update::ResetWorld])).unwrap();
        assert!(frames.current().is_empty());
        assert_eq!(frames.delta_request(), Some(8));
    }

    #[test]
    fn classic_baselines_and_map_clear_keep_expected_defaults() {
        let mut frames = RdFrames::default();
        frames
            .set_baseline(
                16383,
                Baseline {
                    modelindex: 7,
                    frame: 6,
                    skinnum: 255,
                    colormap: 2,
                    origin: Vec3::new(1.0, 2.0, 3.0),
                    ..Default::default()
                },
            )
            .unwrap();
        let reset = || {
            frame(
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
            )
        };
        frames.apply(1, &reset()).unwrap();
        assert_eq!(frames.current()[0].model, 7);
        assert_eq!(frames.current()[0].skin, 255);
        assert_eq!(frames.current()[0].origin, Vec3::new(1.0, 2.0, 3.0));
        frames.invalidate();
        assert_eq!(frames.delta_request(), None);
        assert_eq!(frames.current()[0].model, 7);
        frames.clear();
        assert!(frames.current().is_empty());
        assert_eq!(frames.retained().time(), 0.0);
        assert_eq!(frames.delta_request(), None);
        frames.apply(1, &reset()).unwrap();
        assert_eq!(frames.current()[0].model, 0);
    }
}
