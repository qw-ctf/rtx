// SPDX-License-Identifier: AGPL-3.0-or-later

//! Movement analysis over a parsed [`Demo`](crate::Demo).
//!
//! A demo's `svc_playerinfo` frames are positions sampled every server frame; the interesting
//! quantities for movement work — how fast a player was going, how much height a climb gained, the
//! shape of a route — are differences between them. [`track`] turns one player's frames into a
//! [`Track`] of [`Motion`]s (position plus the speed to reach it), and [`Track::summary`] reduces
//! that to the numbers you'd otherwise recompute by hand for every demo.

use glam::Vec3;

use crate::Demo;

/// One player's motion at a single frame: where they were, and how fast they got there from the
/// previous frame (the first frame of a track reads zero).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    /// Demo timestamp of the frame.
    pub time: f32,
    /// Position.
    pub origin: Vec3,
    /// Horizontal (xy-plane) speed in units/sec — the one that matters for bhop/run pace.
    pub horizontal_speed: f32,
    /// Vertical (z) speed in units/sec; positive is upward.
    pub vertical_speed: f32,
}

/// One player's motion across a whole demo, in ascending time.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    /// The player slot this track follows.
    pub player: u8,
    /// Per-frame motion, in file (time) order.
    pub motions: Vec<Motion>,
}

/// Reduced stats for a [`Track`] — the headline numbers a movement report shows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Summary {
    /// The player slot.
    pub player: u8,
    /// Number of frames in the track.
    pub frames: usize,
    /// Wall-clock span from first to last frame, in seconds.
    pub duration: f32,
    /// First position.
    pub start: Vec3,
    /// Last position.
    pub end: Vec3,
    /// Fastest horizontal speed reached, in units/sec.
    pub peak_speed: f32,
    /// Horizontal path length divided by duration, in units/sec.
    pub mean_speed: f32,
    /// Total horizontal distance travelled along the path, in units.
    pub path_length: f32,
    /// Lowest z reached.
    pub min_z: f32,
    /// Highest z reached.
    pub max_z: f32,
    /// Net height change, `end.z - start.z` (negative for a descent).
    pub height_gain: f32,
}

/// Build the motion track for `player`: their `svc_playerinfo` origins in time order, each carrying
/// the horizontal and vertical speed to reach it from the previous frame.
pub fn track(demo: &Demo, player: u8) -> Track {
    let mut motions = Vec::new();
    let mut prev: Option<(f32, Vec3)> = None;
    for frame in demo.frames.iter().filter(|f| f.info.player == player) {
        let origin = frame.info.origin;
        let (horizontal_speed, vertical_speed) = match prev {
            Some((pt, po)) if frame.time > pt => {
                let dt = frame.time - pt;
                let d = origin - po;
                (d.truncate().length() / dt, d.z / dt)
            }
            _ => (0.0, 0.0),
        };
        motions.push(Motion {
            time: frame.time,
            origin,
            horizontal_speed,
            vertical_speed,
        });
        prev = Some((frame.time, origin));
    }
    Track { player, motions }
}

/// Every player slot that appears in the demo's frames, ascending.
pub fn players(demo: &Demo) -> Vec<u8> {
    let mut ps: Vec<u8> = demo.frames.iter().map(|f| f.info.player).collect();
    ps.sort_unstable();
    ps.dedup();
    ps
}

impl Track {
    /// Reduce the track to its [`Summary`] stats.
    pub fn summary(&self) -> Summary {
        let first = self.motions.first();
        let last = self.motions.last();
        let start = first.map_or(Vec3::ZERO, |m| m.origin);
        let end = last.map_or(Vec3::ZERO, |m| m.origin);
        let duration = match (first, last) {
            (Some(f), Some(l)) => l.time - f.time,
            _ => 0.0,
        };
        // Path length sums the horizontal step distances (speed * dt is exactly that step).
        let mut path_length = 0.0;
        let mut peak_speed = 0.0f32;
        let mut min_z = start.z;
        let mut max_z = start.z;
        for (i, m) in self.motions.iter().enumerate() {
            peak_speed = peak_speed.max(m.horizontal_speed);
            min_z = min_z.min(m.origin.z);
            max_z = max_z.max(m.origin.z);
            if i > 0 {
                path_length += (m.origin - self.motions[i - 1].origin).truncate().length();
            }
        }
        let mean_speed = if duration > 0.0 { path_length / duration } else { 0.0 };
        Summary {
            player: self.player,
            frames: self.motions.len(),
            duration,
            start,
            end,
            peak_speed,
            mean_speed,
            path_length,
            min_z,
            max_z,
            height_gain: end.z - start.z,
        }
    }

    /// Down-sample the track to at most `n` roughly evenly spaced waypoints, always keeping the
    /// first and last frame. `n < 2` (or a shorter track) returns every frame unchanged.
    pub fn waypoints(&self, n: usize) -> Vec<Motion> {
        let len = self.motions.len();
        if n < 2 || len <= n {
            return self.motions.clone();
        }
        (0..n).map(|i| self.motions[i * (len - 1) / (n - 1)]).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Demo, Frame};
    use rtx_proto::protocol::ProtoState;
    use rtx_proto::svc::PlayerInfo;

    fn frame(time: f32, player: u8, origin: Vec3) -> Frame {
        Frame {
            time,
            info: PlayerInfo {
                player,
                flags: 0,
                origin,
                frame: 0,
                msec: None,
                command: None,
                velocity: Vec3::ZERO,
                modelindex: None,
                skinnum: None,
                effects: None,
                weaponframe: None,
                alpha: None,
                pm_type: None,
                jump_held: false,
            },
        }
    }

    fn demo(frames: Vec<Frame>) -> Demo {
        Demo {
            path: "test.qwd".into(),
            proto: ProtoState::new(),
            local_player: Some(0),
            movevars: None,
            demo_cmds: Vec::new(),
            frames,
            warnings: Vec::new(),
        }
    }

    /// A straight 200-unit horizontal hop over 0.1s is 2000 ups, and a climb shows as height gain —
    /// with a second player's frames interleaved but not mixed into the track.
    #[test]
    fn track_speed_and_climb() {
        let d = demo(vec![
            frame(0.0, 0, Vec3::new(0.0, 0.0, 0.0)),
            frame(0.05, 1, Vec3::new(999.0, 0.0, 0.0)), // other player, ignored
            frame(0.1, 0, Vec3::new(200.0, 0.0, 16.0)),
            frame(0.2, 0, Vec3::new(200.0, 0.0, 48.0)), // straight up: no horizontal move
        ]);
        let t = track(&d, 0);
        assert_eq!(t.motions.len(), 3);
        assert!((t.motions[1].horizontal_speed - 2000.0).abs() < 0.5);
        assert!((t.motions[1].vertical_speed - 160.0).abs() < 0.5);
        assert!((t.motions[2].horizontal_speed - 0.0).abs() < 0.5);

        let s = t.summary();
        assert_eq!(s.frames, 3);
        assert!((s.duration - 0.2).abs() < 1e-6);
        assert!((s.peak_speed - 2000.0).abs() < 0.5);
        assert!((s.height_gain - 48.0).abs() < 1e-4);
        assert!((s.max_z - 48.0).abs() < 1e-4);
        assert_eq!(players(&d), vec![0, 1]);
    }

    /// Down-sampling keeps the endpoints and the requested count.
    #[test]
    fn waypoints_keep_endpoints() {
        let frames: Vec<Frame> = (0..10)
            .map(|i| frame(i as f32, 0, Vec3::new(i as f32, 0.0, 0.0)))
            .collect();
        let t = track(&demo(frames), 0);
        let wp = t.waypoints(4);
        assert_eq!(wp.len(), 4);
        assert_eq!(wp.first().unwrap().origin.x, 0.0);
        assert_eq!(wp.last().unwrap().origin.x, 9.0);
    }
}
