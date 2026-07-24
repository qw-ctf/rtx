// SPDX-License-Identifier: AGPL-3.0-or-later

//! Parsing a KTX/Frogbot `.bot` waypoint file, and resolving its 1-based marker ids to world
//! positions.
//!
//! The format is plain ASCII, one console command per line, tokenized on whitespace. Blank lines
//! and `//` comments are skipped; malformed or unknown lines are dropped (and counted). This mirrors
//! KTX's `LoadBotRoutingFromFile` (`ktx/src/marker_load.c`) closely enough that the graph it builds
//! is the graph the game would load.
//!
//! # Marker numbering
//!
//! The subtle part is that marker ids are **not** just the `CreateMarker` order. KTX's `LoadMap`
//! runs `CreateItemMarkers()` *before* reading the file: every qualifying entity in the BSP becomes
//! a marker first (in entity-lump order), claiming ids `1..=K`, and the file's `CreateMarker`s
//! append after them starting at `K + 1`. So resolving an id needs the entity walk too — see
//! [`resolve`] and [`crate::ent::marker_walk`]. `K` is discovered from the entity lump and
//! cross-checked against [`BotFile::max_marker_id`] minus the created count.

use glam::Vec3;
use std::collections::BTreeMap;

use crate::ent::EntityMarker;

/// `ROCKET_JUMP` path flag: this path is crossed by rocket-jumping.
pub const ROCKET_JUMP: u32 = 1 << 9; // 512
/// `BOTPATH_CURLJUMP_HINT`: an air-control (curl) jump. In practice curls are identified by a
/// nonzero [`PathSlot::angle_hint`], which is what sets this bit; kept for completeness.
pub const CURLJUMP_HINT: u32 = 1 << 23; // 8_388_608
/// The most markers KTX will track (`NUMBER_MARKERS`); ids past it are dropped by the loader.
const NUMBER_MARKERS: u32 = 300;
/// KTX's default rocket-jump pitch, seeded when a path is flagged `r` without explicit fields.
const RJ_DEFAULT_PITCH: f32 = 78.25;

/// The rocket-jump parameters KTX distilled from a human's recorded jump: where to aim and how long
/// to wait before firing. `yaw` of `-1` (or `0`) means "keep the current yaw, aim straight down the
/// travel direction".
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RjFields {
    pub pitch: f32,
    pub yaw: f32,
    pub delay: f32,
}

/// One of a marker's eight outbound path slots.
#[derive(Clone, Copy, Debug, Default)]
pub struct PathSlot {
    /// Destination marker id (1-based, combined numbering). `0` = slot never wired to a marker.
    pub dst: u32,
    pub flags: u32,
    /// Curl air-control yaw offset in degrees (`+` = anti-clockwise). Nonzero ⇒ a curl jump.
    pub angle_hint: i32,
    pub rj_pitch: f32,
    pub rj_yaw: f32,
    pub rj_delay: f32,
}

impl PathSlot {
    /// The rocket-jump fields, if this path is flagged as a rocket jump.
    pub fn rj(&self) -> Option<RjFields> {
        (self.flags & ROCKET_JUMP != 0).then_some(RjFields {
            pitch: self.rj_pitch,
            yaw: self.rj_yaw,
            delay: self.rj_delay,
        })
    }
}

/// A parsed `.bot` file, id-agnostic: the created markers, the wired path slots, and the goal/zone
/// tags kept for report context. Resolution to world positions happens in [`resolve`].
#[derive(Default)]
pub struct BotFile {
    /// `CreateMarker` origins, in file order. These take ids `K + 1 ..` after the `K` entity markers.
    pub created: Vec<Vec3>,
    /// `(src marker id, slot 0..=7)` → the slot. `BTreeMap` so iteration is stable for reports.
    pub paths: BTreeMap<(u32, u8), PathSlot>,
    pub goals: Vec<(u32, i32)>,
    pub zones: Vec<(u32, i32)>,
    /// Non-blank, non-comment lines that were not applied (unknown command, wrong arg count,
    /// out-of-range id/slot, or unparseable number) — a corruption signal for the report.
    pub ignored_lines: u32,
}

impl BotFile {
    /// The highest marker id referenced anywhere in the file (as a path source/destination, goal, or
    /// zone). `implied K = max_marker_id − created.len()` should equal the entity-lump walk length.
    pub fn max_marker_id(&self) -> u32 {
        let mut m = 0;
        for (&(src, _), slot) in &self.paths {
            m = m.max(src).max(slot.dst);
        }
        for &(id, _) in self.goals.iter().chain(&self.zones) {
            m = m.max(id);
        }
        m
    }

    /// The entity-marker count the file's numbering implies: everything below the created markers.
    pub fn implied_entity_markers(&self) -> u32 {
        self.max_marker_id().saturating_sub(self.created.len() as u32)
    }
}

/// Parse a `.bot` file's text into a [`BotFile`]. Never fails — malformed lines are counted in
/// [`BotFile::ignored_lines`], matching KTX's silent skip.
pub fn parse(text: &str) -> BotFile {
    let mut f = BotFile::default();
    for line in text.lines() {
        let tok: Vec<&str> = line.split_whitespace().collect();
        let Some(&cmd) = tok.first() else { continue }; // blank line
        if cmd.starts_with("//") {
            continue; // comment
        }
        if !apply(&mut f, cmd, &tok) {
            f.ignored_lines += 1;
        }
    }
    f
}

/// Apply one tokenized command. Returns `false` if the line should be counted as ignored.
fn apply(f: &mut BotFile, cmd: &str, tok: &[&str]) -> bool {
    // Fetch the slot for (marker, slot), creating it on first touch. Enforces KTX's id/slot bounds.
    fn slot(f: &mut BotFile, marker: i64, s: i64) -> Option<&mut PathSlot> {
        if !(1..=NUMBER_MARKERS as i64).contains(&marker) || !(0..=7).contains(&s) {
            return None;
        }
        Some(f.paths.entry((marker as u32, s as u8)).or_default())
    }

    match cmd {
        "CreateMarker" => {
            let (Some(x), Some(y), Some(z)) = (fnum(tok, 1), fnum(tok, 2), fnum(tok, 3)) else {
                return false;
            };
            f.created.push(Vec3::new(x, y, z));
            true
        }
        "SetMarkerPath" => {
            let (Some(src), Some(s), Some(dst)) = (inum(tok, 1), inum(tok, 2), inum(tok, 3)) else {
                return false;
            };
            match slot(f, src, s) {
                Some(p) => {
                    p.dst = dst.clamp(0, NUMBER_MARKERS as i64) as u32;
                    true
                }
                None => false,
            }
        }
        "SetMarkerPathFlags" => {
            let (Some(src), Some(s)) = (inum(tok, 1), inum(tok, 2)) else {
                return false;
            };
            let Some(chars) = tok.get(3) else { return false };
            let flags = decode_path_flags(chars);
            match slot(f, src, s) {
                Some(p) => {
                    p.flags = flags; // replaces, as KTX does
                    if flags & ROCKET_JUMP != 0 {
                        // Seed KTX's rocket-jump defaults; SetRocketJumpPathFields overwrites later.
                        p.rj_pitch = RJ_DEFAULT_PITCH;
                        p.rj_yaw = -1.0;
                    }
                    true
                }
                None => false,
            }
        }
        "SetRocketJumpPathFields" => {
            let (Some(src), Some(s)) = (inum(tok, 1), inum(tok, 2)) else {
                return false;
            };
            let (Some(pitch), Some(yaw), Some(delay)) = (fnum(tok, 3), fnum(tok, 4), fnum(tok, 5)) else {
                return false;
            };
            match slot(f, src, s) {
                Some(p) => {
                    p.rj_pitch = pitch;
                    p.rj_yaw = yaw;
                    p.rj_delay = delay;
                    true
                }
                None => false,
            }
        }
        "SetMarkerPathAngleHint" => {
            let (Some(src), Some(s), Some(hint)) = (inum(tok, 1), inum(tok, 2), inum(tok, 3)) else {
                return false;
            };
            match slot(f, src, s) {
                Some(p) => {
                    p.angle_hint = hint as i32;
                    if hint != 0 {
                        p.flags |= CURLJUMP_HINT;
                    } else {
                        p.flags &= !CURLJUMP_HINT;
                    }
                    true
                }
                None => false,
            }
        }
        "SetGoal" => match (inum(tok, 1), inum(tok, 2)) {
            (Some(m), Some(g)) => {
                f.goals.push((m as u32, g as i32));
                true
            }
            _ => false,
        },
        "SetZone" => match (inum(tok, 1), inum(tok, 2)) {
            (Some(m), Some(z)) => {
                f.zones.push((m as u32, z as i32));
                true
            }
            _ => false,
        },
        // Recognized but not needed for RJ/curl coverage — accepted (not counted) when well-formed.
        "SetMarkerFlag" => tok.len() >= 3,
        "SetMarkerViewOfs" => tok.len() >= 3,
        "SetMapDeathHeight" => tok.len() >= 2,
        _ => false, // unknown command
    }
}

/// Decode a KTX path-flag string (`w`/`6`/`r`/`j`/`v`/`a`) to its bitfield. Unknown chars ignored.
pub fn decode_path_flags(s: &str) -> u32 {
    let mut flags = 0;
    for c in s.chars() {
        flags |= match c {
            'w' => 1 << 1,        // WATERJUMP_
            '6' => 1 << 8,        // DM6_DOOR
            'r' => ROCKET_JUMP,   // 1 << 9
            'j' => 1 << 10,       // JUMP_LEDGE
            'v' => 1 << 11,       // VERTICAL_PLATFORM
            'a' => CURLJUMP_HINT, // 1 << 23
            _ => 0,
        };
    }
    flags
}

fn inum(tok: &[&str], i: usize) -> Option<i64> {
    tok.get(i).and_then(|t| t.parse::<i64>().ok())
}

fn fnum(tok: &[&str], i: usize) -> Option<f32> {
    tok.get(i).and_then(|t| t.parse::<f32>().ok())
}

/// Where a resolved marker sits: an entity from the BSP (with its classname, and whether the
/// position is a brush-bounds approximation) or a `CreateMarker` from the file.
#[derive(Clone, Debug)]
pub enum MarkerPos {
    Entity { classname: String, pos: Vec3, brush: bool },
    Created(Vec3),
}

impl MarkerPos {
    pub fn pos(&self) -> Vec3 {
        match self {
            MarkerPos::Entity { pos, .. } => *pos,
            MarkerPos::Created(p) => *p,
        }
    }
}

/// A path with both endpoints resolved to world positions, ready to check against the navmesh.
#[derive(Clone, Debug)]
pub struct ResolvedPath {
    pub src: u32,
    pub dst: u32,
    pub from: MarkerPos,
    pub to: MarkerPos,
    pub flags: u32,
    pub rj: Option<RjFields>,
    pub angle_hint: i32,
}

impl ResolvedPath {
    pub fn is_rj(&self) -> bool {
        self.flags & ROCKET_JUMP != 0
    }
    pub fn is_curl(&self) -> bool {
        self.angle_hint != 0
    }
}

/// Resolve every wired path to world positions. Ids `1..=K` map onto `entity_markers` (in order),
/// ids above onto the file's `CreateMarker`s. Paths referencing an unresolvable id (or with no
/// destination wired) are dropped; the drop count is returned as the second element.
pub fn resolve(file: &BotFile, entity_markers: &[EntityMarker]) -> (Vec<ResolvedPath>, u32) {
    let k = entity_markers.len() as u32;
    let pos = |id: u32| -> Option<MarkerPos> {
        if id == 0 {
            return None;
        }
        if id <= k {
            let em = &entity_markers[(id - 1) as usize];
            Some(MarkerPos::Entity {
                classname: em.classname.clone(),
                pos: em.pos,
                brush: em.brush,
            })
        } else {
            file.created.get((id - k - 1) as usize).map(|&p| MarkerPos::Created(p))
        }
    };

    let mut out = Vec::new();
    let mut dropped = 0;
    for (&(src, _slot), ps) in &file.paths {
        if ps.dst == 0 {
            continue; // slot flagged/hinted but never wired to a destination
        }
        let (Some(from), Some(to)) = (pos(src), pos(ps.dst)) else {
            dropped += 1;
            continue;
        };
        out.push(ResolvedPath {
            src,
            dst: ps.dst,
            from,
            to,
            flags: ps.flags,
            rj: ps.rj(),
            angle_hint: ps.angle_hint,
        });
    }
    (out, dropped)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_rocket_jump_path_end_to_end() {
        let f = parse(
            "CreateMarker 1615 263 -213\n\
             CreateMarker 1970 428 -88\n\
             SetMarkerPath 1 2 2\n\
             SetMarkerPathFlags 1 2 r\n\
             SetRocketJumpPathFields 1 2 80.0 350.0 3\n",
        );
        assert_eq!(f.created.len(), 2);
        let p = f.paths[&(1, 2)];
        assert_eq!(p.dst, 2);
        assert!(p.flags & ROCKET_JUMP != 0);
        assert_eq!(
            p.rj().unwrap(),
            RjFields {
                pitch: 80.0,
                yaw: 350.0,
                delay: 3.0
            }
        );
        assert_eq!(f.ignored_lines, 0);
    }

    #[test]
    fn rocket_flag_without_fields_seeds_ktx_defaults_and_replaces_flags() {
        let f = parse(
            "CreateMarker 0 0 0\nCreateMarker 1 1 1\n\
             SetMarkerPathFlags 1 0 j\n\
             SetMarkerPath 1 0 2\n\
             SetMarkerPathFlags 1 0 r\n", // replaces the earlier `j`
        );
        let p = f.paths[&(1, 0)];
        assert_eq!(p.flags & ROCKET_JUMP, ROCKET_JUMP);
        assert_eq!(p.flags & (1 << 10), 0, "the earlier jump flag was replaced, not OR'd");
        assert_eq!(
            p.rj().unwrap(),
            RjFields {
                pitch: 78.25,
                yaw: -1.0,
                delay: 0.0
            }
        );
    }

    #[test]
    fn angle_hint_marks_and_clears_curl() {
        let f = parse(
            "CreateMarker 0 0 0\nCreateMarker 1 1 1\n\
             SetMarkerPath 1 0 2\n\
             SetMarkerPathAngleHint 1 0 45\n\
             SetMarkerPath 1 1 2\n\
             SetMarkerPathFlags 1 1 r\n\
             SetMarkerPathAngleHint 1 1 20\n\
             SetMarkerPath 1 2 2\n\
             SetMarkerPathAngleHint 1 2 0\n",
        );
        assert_eq!(f.paths[&(1, 0)].angle_hint, 45);
        assert!(f.paths[&(1, 0)].flags & CURLJUMP_HINT != 0);
        // Slot 1 is both a rocket jump and a curl — it belongs to both extracts.
        assert!(f.paths[&(1, 1)].flags & ROCKET_JUMP != 0);
        assert_eq!(f.paths[&(1, 1)].angle_hint, 20);
        // Slot 2 hinted 0 → curl bit cleared.
        assert_eq!(f.paths[&(1, 2)].angle_hint, 0);
        assert_eq!(f.paths[&(1, 2)].flags & CURLJUMP_HINT, 0);
    }

    #[test]
    fn last_write_wins_per_slot() {
        let f = parse("SetMarkerPath 5 3 10\nSetMarkerPath 5 3 20\n");
        assert_eq!(f.paths[&(5, 3)].dst, 20);
    }

    #[test]
    fn silently_skips_malformed_and_counts_them() {
        let f = parse(
            "\n\
             // a comment\n\
             CreateMarker 1 2 3\n\
             SetMarkerPath 1 8 2\n\
             SetMarkerPath 1\n\
             NonsenseCommand foo bar\n\
             SetMarkerPathFlags 0 0 r\n",
        );
        assert_eq!(f.created.len(), 1);
        // slot 8 (out of range), truncated SetMarkerPath, unknown command, marker id 0 → 4 ignored.
        // Blank and comment lines are not counted.
        assert_eq!(f.ignored_lines, 4);
    }

    #[test]
    fn implied_entity_marker_count() {
        // dm4 shape: created markers referenced up to 116, 62 created ⇒ 54 entity markers.
        let mut text = String::new();
        for _ in 0..62 {
            text.push_str("CreateMarker 0 0 0\n");
        }
        text.push_str("SetMarkerPath 116 0 8\n");
        let f = parse(&text);
        assert_eq!(f.max_marker_id(), 116);
        assert_eq!(f.implied_entity_markers(), 54);
    }

    #[test]
    fn resolves_entity_and_created_ids_and_drops_out_of_range() {
        let f = parse(
            "CreateMarker 100 0 0\n\
             SetMarkerPath 1 0 3\n\
             SetMarkerPath 3 0 1\n\
             SetMarkerPath 1 1 9\n", // 9 is beyond entity(2) + created(1) = 3 ⇒ dropped
        );
        let ents = vec![
            EntityMarker {
                classname: "item_health".into(),
                pos: Vec3::new(10.0, 0.0, 0.0),
                brush: false,
            },
            EntityMarker {
                classname: "door".into(),
                pos: Vec3::new(20.0, 0.0, 0.0),
                brush: true,
            },
        ];
        let (paths, dropped) = resolve(&f, &ents);
        assert_eq!(dropped, 1);
        // id 1 = first entity marker; id 3 = created marker 0 (3 - 2 - 1 = 0).
        let p = paths.iter().find(|p| p.src == 1 && p.dst == 3).unwrap();
        assert!(matches!(p.from, MarkerPos::Entity { .. }));
        assert!(matches!(p.to, MarkerPos::Created(_)));
        assert_eq!(p.to.pos(), Vec3::new(100.0, 0.0, 0.0));
    }
}
