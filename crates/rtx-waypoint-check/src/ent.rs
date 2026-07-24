// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reproducing KTX's `CreateItemMarkers` offline from the BSP entity lump.
//!
//! KTX turns a set of map entities into bot markers *before* it reads the `.bot` file, claiming the
//! low marker ids in entity-lump order (see [`crate::botfile`]). To resolve the file's path
//! references we have to walk the same entities in the same order and place a marker for each.
//!
//! The classname set and the position rules are lifted from `ktx/src/bot_loadmap.c` and
//! `bot_items.c`: point entities (items, spawns, teleport destinations) sit at their lump `origin`;
//! brush entities (doors, plats, triggers, buttons) sit at the centre of their submodel bounds with
//! `z = min_z + 24`, exactly as `fb_spawn_door` computes it. `func_door*`/`func_plat`/`func_train`
//! are renamed to `door`/`plat`/`train` at spawn, so we rename them here too. Entities flagged
//! `SPAWNFLAG_NOT_DEATHMATCH` are removed before the walk, so we skip them.
//!
//! Brush positions are an approximation (KTX works off live bboxes we don't rebuild), so those
//! markers are tagged `brush` and shown with a `~` in reports.
//!
//! The block parser and teleport wiring mirror the offline-routability test in
//! `rtx-game`'s `race` module — same shapes, duplicated here so the checker needs no game internals.

use glam::Vec3;
use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::TeleportInfo;

/// A map entity that KTX would have turned into a marker, in lump order.
#[derive(Clone, Debug)]
pub struct EntityMarker {
    /// Canonical (post-rename) classname, e.g. `door` for a `func_door`.
    pub classname: String,
    pub pos: Vec3,
    /// True when `pos` is a brush-bounds approximation rather than an exact point origin.
    pub brush: bool,
}

/// `SPAWNFLAG_NOT_DEATHMATCH` — such entities are removed in deathmatch before markers are made.
const NOT_DEATHMATCH: i64 = 2048;

/// Point-entity marker classnames (canonical): placed at the lump `origin`.
const POINT_MARKERS: &[&str] = &[
    "item_armor1",
    "item_armor2",
    "item_armorInv",
    "item_artifact_invulnerability",
    "item_artifact_envirosuit",
    "item_artifact_invisibility",
    "item_artifact_super_damage",
    "item_cells",
    "item_health",
    "item_rockets",
    "item_shells",
    "item_spikes",
    "item_weapon",
    "weapon_supershotgun",
    "weapon_nailgun",
    "weapon_supernailgun",
    "weapon_grenadelauncher",
    "weapon_rocketlauncher",
    "weapon_lightning",
    "info_player_deathmatch",
    "info_teleport_destination",
];

/// Brush-entity marker classnames (canonical): placed at submodel-bounds centre, `z = min_z + 24`.
const BRUSH_MARKERS: &[&str] = &[
    "door",
    "func_button",
    "plat",
    "train",
    "trigger_changelevel",
    "trigger_counter",
    "trigger_hurt",
    "trigger_multiple",
    "trigger_once",
    "trigger_onlyregistered",
    "trigger_push",
    "trigger_secret",
    "trigger_setskill",
    "trigger_teleport",
];

/// The spawn-time classname renames KTX applies before markers are counted.
fn rename(cn: &str) -> &str {
    match cn {
        "func_door" | "func_door_secret" => "door",
        "func_plat" => "plat",
        "func_train" => "train",
        other => other,
    }
}

/// A brush marker's origin: the centre of its submodel bounds, floored to `min_z + 24` — KTX's
/// `fb_spawn_door` formula. Falls back to the entity origin when the bounds are unavailable, so the
/// marker still consumes its id (keeping the numbering aligned) even if its position is unknown.
fn brush_pos(bounds: Option<(Vec3, Vec3)>, origin: Vec3) -> Vec3 {
    match bounds {
        Some((mins, maxs)) => {
            let c = (mins + maxs) * 0.5;
            Vec3::new(c.x, c.y, mins.z.min(maxs.z) + 24.0)
        }
        None => origin,
    }
}

/// The ordered marker list KTX would build from this map's entities.
pub fn marker_walk(bsp: &Bsp) -> Vec<EntityMarker> {
    let blocks = parse_blocks(&bsp.entities);
    walk_blocks(&blocks, |n| bsp.submodel(n).map(|m| (m.mins, m.maxs)))
}

/// Count of `func_plat` lifts — reported as a caveat, since plats aren't spliced into the offline
/// navmesh (their traversal needs the live mover).
pub fn plat_count(bsp: &Bsp) -> usize {
    parse_blocks(&bsp.entities)
        .iter()
        .filter(|b| field(b, "classname") == Some("func_plat"))
        .count()
}

/// Teleporters, as `collect_teleports` would gather them: the trigger box from the brush submodel's
/// bounds, the destination from the targeted entity's origin (`+27z` for an
/// `info_teleport_destination`, matching its spawn). Wired into the navmesh so teleport-riding paths
/// resolve offline.
pub fn teleports(bsp: &Bsp) -> Vec<TeleportInfo> {
    let blocks = parse_blocks(&bsp.entities);
    teleports_from(&blocks, |n| bsp.submodel(n).map(|m| (m.mins, m.maxs)))
}

/// [`marker_walk`] over pre-parsed blocks with an injectable submodel-bounds lookup (for testing).
fn walk_blocks(
    blocks: &[Vec<(String, String)>],
    bounds_of: impl Fn(usize) -> Option<(Vec3, Vec3)>,
) -> Vec<EntityMarker> {
    let mut out = Vec::new();
    for b in blocks {
        let Some(raw) = field(b, "classname") else { continue };
        let cn = rename(raw);
        let flags = field(b, "spawnflags")
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(0.0) as i64;
        if flags & NOT_DEATHMATCH != 0 {
            continue;
        }
        let origin = field(b, "origin").map(vec3_of).unwrap_or_default();
        if POINT_MARKERS.contains(&cn) {
            out.push(EntityMarker {
                classname: cn.to_string(),
                pos: origin,
                brush: false,
            });
        } else if BRUSH_MARKERS.contains(&cn) {
            let bounds = submodel_index(b).and_then(&bounds_of);
            out.push(EntityMarker {
                classname: cn.to_string(),
                pos: brush_pos(bounds, origin),
                brush: true,
            });
        }
    }
    out
}

/// [`teleports`] over pre-parsed blocks with an injectable submodel-bounds lookup (for testing).
fn teleports_from(
    blocks: &[Vec<(String, String)>],
    bounds_of: impl Fn(usize) -> Option<(Vec3, Vec3)>,
) -> Vec<TeleportInfo> {
    blocks
        .iter()
        .filter(|b| field(b, "classname") == Some("trigger_teleport"))
        .filter_map(|b| {
            let (tmin, tmax) = submodel_index(b).and_then(&bounds_of)?;
            let target = field(b, "target")?;
            let dest = blocks.iter().find(|d| field(d, "targetname") == Some(target))?;
            let mut origin = field(dest, "origin").map(vec3_of)?;
            if field(dest, "classname") == Some("info_teleport_destination") {
                origin.z += 27.0;
            }
            Some(TeleportInfo {
                tmin,
                tmax,
                dest: origin,
            })
        })
        .collect()
}

/// The `*N` submodel index from a brush entity's `model` field.
fn submodel_index(b: &[(String, String)]) -> Option<usize> {
    field(b, "model")?.strip_prefix('*')?.parse().ok()
}

/// Parse the entity lump's `{ "key" "value" … }` blocks.
fn parse_blocks(text: &str) -> Vec<Vec<(String, String)>> {
    let mut blocks = Vec::new();
    let mut cur: Option<Vec<(String, String)>> = None;
    let mut chars = text.chars().peekable();
    let mut strings: Vec<String> = Vec::new();
    while let Some(c) = chars.next() {
        match c {
            '{' => cur = Some(Vec::new()),
            '}' => {
                if let Some(b) = cur.take() {
                    blocks.push(b);
                }
                strings.clear();
            }
            '"' => {
                let mut s = String::new();
                for c in chars.by_ref() {
                    if c == '"' {
                        break;
                    }
                    s.push(c);
                }
                strings.push(s);
                if strings.len() == 2 {
                    if let Some(b) = cur.as_mut() {
                        b.push((strings[0].clone(), strings[1].clone()));
                    }
                    strings.clear();
                }
            }
            _ => {}
        }
    }
    blocks
}

fn field<'a>(b: &'a [(String, String)], key: &str) -> Option<&'a str> {
    b.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

fn vec3_of(s: &str) -> Vec3 {
    let mut p = s.split_whitespace().map(|t| t.parse().unwrap_or(0.0));
    Vec3::new(
        p.next().unwrap_or(0.0),
        p.next().unwrap_or(0.0),
        p.next().unwrap_or(0.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_blocks_with_quoted_values() {
        let blocks = parse_blocks(
            "{\n\"classname\" \"worldspawn\"\n\"message\" \"The Abandoned Base\"\n}\n\
             {\n\"classname\" \"item_health\"\n\"origin\" \"1 2 3\"\n}\n",
        );
        assert_eq!(blocks.len(), 2);
        assert_eq!(field(&blocks[1], "classname"), Some("item_health"));
        assert_eq!(field(&blocks[0], "message"), Some("The Abandoned Base"));
    }

    #[test]
    fn walk_preserves_order_renames_and_skips_not_deathmatch() {
        let blocks = parse_blocks(
            "{ \"classname\" \"worldspawn\" }\n\
             { \"classname\" \"item_rockets\" \"origin\" \"10 20 30\" }\n\
             { \"classname\" \"item_health\" \"origin\" \"0 0 0\" \"spawnflags\" \"2048\" }\n\
             { \"classname\" \"func_door\" \"model\" \"*1\" }\n\
             { \"classname\" \"info_player_deathmatch\" \"origin\" \"5 5 5\" }\n\
             { \"classname\" \"light\" \"origin\" \"9 9 9\" }\n",
        );
        // func_door's submodel *1 spans a box centred at (100,100) from z=40..60.
        let markers = walk_blocks(&blocks, |n| {
            (n == 1).then_some((Vec3::new(64.0, 64.0, 40.0), Vec3::new(136.0, 136.0, 60.0)))
        });
        // worldspawn skipped, item_health skipped (NOT_DEATHMATCH), light skipped.
        assert_eq!(markers.len(), 3);
        assert_eq!(markers[0].classname, "item_rockets");
        assert!(!markers[0].brush);
        assert_eq!(markers[0].pos, Vec3::new(10.0, 20.0, 30.0));
        // func_door renamed to door, positioned at bounds centre with z = min_z + 24.
        assert_eq!(markers[1].classname, "door");
        assert!(markers[1].brush);
        assert_eq!(markers[1].pos, Vec3::new(100.0, 100.0, 64.0));
        assert_eq!(markers[2].classname, "info_player_deathmatch");
    }

    #[test]
    fn brush_pos_matches_ktx_door_formula() {
        let p = brush_pos(
            Some((Vec3::new(-10.0, -20.0, 8.0), Vec3::new(30.0, 40.0, 72.0))),
            Vec3::ZERO,
        );
        assert_eq!(p, Vec3::new(10.0, 10.0, 32.0));
        // Missing bounds ⇒ fall back to the entity origin (still consumes the id).
        assert_eq!(brush_pos(None, Vec3::new(1.0, 2.0, 3.0)), Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn teleport_wiring_adds_27z_to_destination() {
        let blocks = parse_blocks(
            "{ \"classname\" \"trigger_teleport\" \"model\" \"*3\" \"target\" \"t1\" }\n\
             { \"classname\" \"info_teleport_destination\" \"targetname\" \"t1\" \"origin\" \"100 200 8\" }\n",
        );
        let tp = teleports_from(&blocks, |n| {
            (n == 3).then_some((Vec3::new(0.0, 0.0, 0.0), Vec3::new(32.0, 32.0, 64.0)))
        });
        assert_eq!(tp.len(), 1);
        assert_eq!(tp[0].dest, Vec3::new(100.0, 200.0, 35.0));
        assert_eq!(tp[0].tmax, Vec3::new(32.0, 32.0, 64.0));
    }
}
