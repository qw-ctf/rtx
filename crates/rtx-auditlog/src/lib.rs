// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-bot flight recorder: a fixed, once-allocated ring of compact [`AuditFrame`] sensor snapshots.
//!
//! `rtx_bot_debug` used to `conprint` a formatted line every bot frame — which floods the server
//! console and drops network packets. Instead each frame is captured here as a small `Copy` struct
//! (no allocation, no formatting on the hot path) into a ring buffer sized by `rtx_bot_auditlog` (MB,
//! default 10). The MCP pulls a tail of frames on demand over the control channel — the frames travel
//! as msgpack (this crate is shared by producer and consumer, so the schema is single-sourced) and the
//! MCP renders them for inspection. Because the frame is a stable serde schema, the whole trace stays
//! "raw compact data" end to end, decoded only where a human looks at it.
//!
//! The five phase/posture/commit fields mirror the game's internal enums; the game maps its enums to
//! these on capture (a compile-checked `match`), and the enums render to readable names via `Debug`,
//! so the MCP needs no code tables.

use serde::{Deserialize, Serialize};

/// Bhop controller phase (mirror of the game's `bot::bhop::Phase`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Bhop {
    #[default]
    Off,
    Prestrafe,
    Hop,
    Zigzag,
}

/// Grapple-hook traversal phase (mirror of the game's `bot::state::HookPhase`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hook {
    #[default]
    Idle,
    Aim,
    Flight,
    Reel,
    Ballistic,
}

/// Rocket-jump traversal phase (mirror of the game's `bot::state::RjPhase`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Rj {
    #[default]
    Idle,
    Stance,
    Rise,
    Ballistic,
}

/// Strategic combat posture (mirror of the game's `bot::state::CombatPosture`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Posture {
    Recover,
    #[default]
    Hold,
    Press,
}

/// Item-goal commitment (mirror of the game's `bot::state::GoalCommit`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Commit {
    #[default]
    None,
    Pickup,
    Powerup,
}

/// A fixed-capacity inline string (no heap allocation), used for the short `&'static str` reason tags
/// the game carries (e.g. why a bhop run ended). Captured by value into every frame; renders back to
/// `&str` on inspection. Truncates silently past [`Tag::CAP`].
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct Tag {
    len: u8,
    buf: [u8; Tag::CAP],
}

impl Tag {
    pub const CAP: usize = 23;

    /// Capture up to [`Tag::CAP`] bytes of `s` (UTF-8 boundary-safe truncation).
    pub fn new(s: &str) -> Self {
        let mut n = s.len().min(Self::CAP);
        while n > 0 && !s.is_char_boundary(n) {
            n -= 1;
        }
        let mut buf = [0u8; Self::CAP];
        buf[..n].copy_from_slice(&s.as_bytes()[..n]);
        Tag { len: n as u8, buf }
    }

    /// The captured text.
    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.buf[..self.len as usize]).unwrap_or("")
    }
}

impl Default for Tag {
    fn default() -> Self {
        Tag {
            len: 0,
            buf: [0u8; Self::CAP],
        }
    }
}

impl std::fmt::Debug for Tag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

/// One bot frame's diagnostic snapshot — the unit stored in the [`Audit`] ring and shipped to the MCP.
///
/// All fields are `Copy` primitives (or the small mirror enums / [`Tag`]), so capturing a frame is a
/// handful of field writes with no allocation and no string formatting. Boolean sensor bits are packed
/// into `flags` (see the `flags::*` masks). The record is a stable serde schema: adding fields is
/// fine, but reorder/rename with the MCP decoder in mind.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct AuditFrame {
    /// Game time of the frame (s).
    pub t: f32,
    /// Bot origin at frame end.
    pub origin: [f32; 3],
    /// Bot velocity at frame end.
    pub vel: [f32; 3],
    /// Horizontal speed (u/s) — `hypot(vel.x, vel.y)`, the number movement work watches.
    pub speed: f32,
    /// Peak horizontal speed reached in the current bhop run.
    pub peak: f32,
    /// Packed sensor bits — see the [`flags`] module.
    pub flags: u16,
    /// Bhop controller phase and the run's hop/flip counters.
    pub bhop: Bhop,
    pub hops: u16,
    pub flips: u16,
    /// Why the bhop run last ended (empty until the first `Hop -> Off`).
    pub off_reason: Tag,
    /// Grapple-hook and rocket-jump driver phases.
    pub hook: Hook,
    pub rj: Rj,
    /// Route cursor: total legs and the leg being flown, plus the route band at the cursor.
    pub route_len: u16,
    pub route_pos: u16,
    pub band: i16,
    /// The frame's movement command (what the brain actually pressed).
    pub forward: i16,
    pub side: i16,
    /// Strategic state.
    pub posture: Posture,
    pub commit: Commit,
    /// Item goal: entity id (0 = none), its resolved nav cell, straight-line distance, and the cell's
    /// height under the item (a large negative `goal_dz` means the goal aliased to floor below a
    /// pedestal item — uncollectable from there).
    pub goal_ent: u32,
    pub goal_cell: i32,
    pub goal_dist: f32,
    pub goal_dz: f32,
    /// Gate errand index this bot is servicing (-1 = none).
    pub gate: i32,
    /// Perception: the entity currently believed to be an enemy (0 = none).
    pub known_enemy: u32,
    /// Count of live failed-link penalties (loop-free-nav pressure).
    pub pen: u16,
    /// Item magnet / spectate-watch entity ids (0 = none).
    pub magnet: u32,
    pub watch: u32,
}

/// Bit masks for [`AuditFrame::flags`].
pub mod flags {
    pub const ENEMY: u16 = 1 << 0;
    pub const ON_GROUND: u16 = 1 << 1;
    pub const IN_WATER: u16 = 1 << 2;
    pub const ON_ITEM: u16 = 1 << 3;
    pub const OWN_LG: u16 = 1 << 4;
    pub const AWARE: u16 = 1 << 5;
    pub const ATTACK: u16 = 1 << 6;
    pub const JUMP: u16 = 1 << 7;
}

/// A per-bot ring buffer of [`AuditFrame`]s. A single fixed `Vec<AuditFrame>` is allocated once to the
/// `rtx_bot_auditlog` budget (bytes / frame size) with a write head that wraps — nothing is allocated
/// per frame, so full-rate capture adds no allocator traffic. If the budget cvar changes the ring is
/// reallocated once (and starts empty). Nothing is allocated until the first frame is pushed.
#[derive(Default)]
pub struct Audit {
    frames: Vec<AuditFrame>,
    /// Next write index (`0..cap`).
    head: usize,
    /// The head has lapped at least once (the ring is full).
    wrapped: bool,
}

impl Audit {
    /// Push one frame, sizing the ring to `budget_bytes` on the first call or after a budget change.
    /// `budget_bytes == 0` disables and frees the ring.
    pub fn push(&mut self, frame: AuditFrame, budget_bytes: usize) {
        let cap = budget_bytes / std::mem::size_of::<AuditFrame>();
        if cap == 0 {
            self.frames = Vec::new();
            self.head = 0;
            self.wrapped = false;
            return;
        }
        if self.frames.len() != cap {
            self.frames = vec![AuditFrame::default(); cap];
            self.head = 0;
            self.wrapped = false;
        }
        self.frames[self.head] = frame;
        self.head += 1;
        if self.head == cap {
            self.head = 0;
            self.wrapped = true;
        }
    }

    /// The last `max` frames, oldest-first (fewer if the ring holds fewer).
    pub fn tail(&self, max: usize) -> Vec<AuditFrame> {
        let cap = self.frames.len();
        if cap == 0 {
            return Vec::new();
        }
        let held = if self.wrapped { cap } else { self.head };
        let take = held.min(max);
        let start = (self.head + cap - take) % cap;
        (0..take).map(|i| self.frames[(start + i) % cap]).collect()
    }

    /// Number of frames currently held.
    pub fn len(&self) -> usize {
        if self.wrapped {
            self.frames.len()
        } else {
            self.head
        }
    }

    /// Whether any frame has been captured yet.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tail_roundtrips_without_wrapping() {
        let mut a = Audit::default();
        for i in 0..3u32 {
            let mut f = AuditFrame::default();
            f.goal_ent = i;
            a.push(f, 1 << 20);
        }
        let got: Vec<u32> = a.tail(10).iter().map(|f| f.goal_ent).collect();
        assert_eq!(got, vec![0, 1, 2]);
        assert_eq!(a.len(), 3);
    }

    #[test]
    fn tail_limits_to_the_newest_frames() {
        let mut a = Audit::default();
        for i in 0..100u32 {
            let mut f = AuditFrame::default();
            f.goal_ent = i;
            a.push(f, 1 << 20);
        }
        let got: Vec<u32> = a.tail(3).iter().map(|f| f.goal_ent).collect();
        assert_eq!(got, vec![97, 98, 99]);
    }

    #[test]
    fn ring_wraps_keeping_the_newest_frames() {
        let mut a = Audit::default();
        // A tiny budget: only a few frames fit; push far more.
        let budget = std::mem::size_of::<AuditFrame>() * 4;
        for i in 0..1000u32 {
            let mut f = AuditFrame::default();
            f.goal_ent = i;
            a.push(f, budget);
        }
        let got: Vec<u32> = a.tail(1000).iter().map(|f| f.goal_ent).collect();
        assert_eq!(got, vec![996, 997, 998, 999]);
        assert_eq!(a.len(), 4);
    }

    #[test]
    fn realloc_on_budget_change_starts_empty() {
        let mut a = Audit::default();
        a.push(AuditFrame::default(), 1 << 20);
        a.push(AuditFrame::default(), 1 << 21); // different budget -> realloc, drops history
        assert_eq!(a.len(), 1);
    }

    #[test]
    fn tag_truncates_on_char_boundary_and_roundtrips() {
        let t = Tag::new("runway");
        assert_eq!(t.as_str(), "runway");
        let long = "x".repeat(100);
        assert_eq!(Tag::new(&long).as_str().len(), Tag::CAP);
    }

    #[test]
    fn frame_msgpack_roundtrips() {
        let mut f = AuditFrame::default();
        f.t = 12.5;
        f.speed = 812.0;
        f.bhop = Bhop::Hop;
        f.off_reason = Tag::new("runway");
        f.flags = flags::ON_GROUND | flags::ENEMY;
        let bytes = rmp_serde::to_vec(&f).unwrap();
        let back: AuditFrame = rmp_serde::from_slice(&bytes).unwrap();
        assert_eq!(back.t, 12.5);
        assert_eq!(back.bhop, Bhop::Hop);
        assert_eq!(back.off_reason.as_str(), "runway");
        assert_eq!(back.flags, flags::ON_GROUND | flags::ENEMY);
    }
}
