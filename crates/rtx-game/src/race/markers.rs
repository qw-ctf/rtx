// SPDX-License-Identifier: AGPL-3.0-or-later

//! Building routes from the map's `race_route_start` / `race_route_marker` entities: the plain-info
//! structs gathered on the main thread (`RaceStartInfo` / `RaceMarkerInfo`) and `routes_from_markers`,
//! which walks each start's `target`->`targetname` chain into a validated route (the `collect_plats`
//! pattern). The alternative to a `.route` file (see the sibling `routefile` parser).

use glam::Vec3;

use super::{RaceFalseStartMode, RaceNodeType, RaceRoute, RaceRouteNode, RaceWeaponMode, MAX_ROUTE_NODES};

/// One `race_route_start` or `race_route_marker` entity, reduced to the fields the chain
/// builder needs (gathered on the main thread, then processed purely — the `collect_plats`
/// pattern). `pitch`/`yaw` are the entity's `race_route_start_pitch`/`_yaw` keys (zero on markers).
pub struct RaceMarkerInfo {
    pub origin: Vec3,
    pub size: Vec3,
    pub target: Option<Box<str>>,
    pub targetname: Option<Box<str>>,
    pub pitch: f32,
    pub yaw: f32,
}

/// A `race_route_start` entity: the chain head plus the route-wide settings it carries.
pub struct RaceStartInfo {
    pub marker: RaceMarkerInfo,
    pub name: String,
    pub desc: String,
    pub timeout: f32,
    pub weapon_mode: i32,
    pub falsestart_mode: i32,
}

/// Build routes from embedded map entities, mirroring ktx `race_route_create`
/// (race.c:3652-3792): each `race_route_start` heads one route; the chain follows
/// `target` → `targetname` through the markers until a target-less node. Invalid chains skip
/// that route with a warning (never fatal). One deliberate fix over ktx: each node's own
/// pitch/yaw is used (ktx's field table aliases `race_route_start_pitch` onto yaw —
/// g_spawn.c:150 — a latent bug no shipped map exercises; all use 0/0).
pub fn routes_from_markers(starts: &[RaceStartInfo], markers: &[RaceMarkerInfo]) -> (Vec<RaceRoute>, Vec<String>) {
    let mut routes = Vec::new();
    let mut warnings = Vec::new();

    for start in starts {
        let label = if start.name.is_empty() { "(unnamed)" } else { &start.name };
        if start.name.is_empty() || start.desc.is_empty() {
            warnings.push("route name/description not specified".into());
            continue;
        }
        let weapon = match start.weapon_mode {
            1 => RaceWeaponMode::No,
            2 => RaceWeaponMode::Allowed,
            3 => RaceWeaponMode::After2s,
            _ => {
                warnings.push(format!("route \"{label}\": weapon mode not valid"));
                continue;
            }
        };
        let falsestart = match start.falsestart_mode {
            1 => RaceFalseStartMode::No,
            2 => RaceFalseStartMode::Yes,
            _ => {
                warnings.push(format!("route \"{label}\": falsestart mode not valid"));
                continue;
            }
        };

        // Walk the marker chain. `chain` holds indices into `markers` (usize::MAX = the start
        // entity itself) purely for cycle detection.
        let mut nodes: Vec<&RaceMarkerInfo> = vec![&start.marker];
        let mut seen: Vec<usize> = vec![usize::MAX];
        let mut cursor = &start.marker;
        let mut valid = true;
        while let Some(target) = cursor.target.as_deref() {
            if nodes.len() >= MAX_ROUTE_NODES {
                warnings.push(format!("route \"{label}\": route too long"));
                valid = false;
                break;
            }
            let Some(mi) = markers
                .iter()
                .position(|m| m.targetname.as_deref() == Some(target))
            else {
                // ktx distinguishes a dangling target (chain just ends, route kept) from a
                // non-marker target (route skipped); having gathered only markers we can't
                // tell them apart, so end the chain and let the ≥2-node check decide.
                warnings.push(format!("route \"{label}\": target '{target}' matches no race_route_marker; route ends there"));
                break;
            };
            if seen.contains(&mi) {
                warnings.push(format!("route \"{label}\": circular route detected"));
                valid = false;
                break;
            }
            seen.push(mi);
            nodes.push(&markers[mi]);
            cursor = &markers[mi];
        }
        if !valid {
            continue;
        }
        if nodes.len() < 2 {
            warnings.push(format!("route \"{label}\": route too short ({} nodes)", nodes.len()));
            continue;
        }

        let last = nodes.len() - 1;
        routes.push(RaceRoute {
            name: start.name.clone(),
            desc: start.desc.clone(),
            // ktx applies the 1..=999 clamp unconditionally on this path (an absent key
            // becomes a 1s timeout); every shipped map sets one.
            timeout: start.timeout.clamp(1.0, 999.0),
            weapon,
            falsestart,
            nodes: nodes
                .iter()
                .enumerate()
                .map(|(i, m)| RaceRouteNode {
                    kind: match i {
                        0 => RaceNodeType::Start,
                        i if i == last => RaceNodeType::End,
                        _ => RaceNodeType::Checkpoint,
                    },
                    origin: m.origin,
                    pitch: m.pitch,
                    yaw: m.yaw,
                    size: m.size,
                })
                .collect(),
        });
    }
    (routes, warnings)
}
