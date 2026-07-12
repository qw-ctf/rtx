// SPDX-License-Identifier: AGPL-3.0-or-later

//! Item vigil — what a bot does while *waiting* on a goal item that isn't collectable yet: a pickup
//! mid-respawn, or a weapon it's reserving for a teammate (a handoff hold). Without this the bot
//! stands on the spawn point and, with the route exhausted and the look target on top of its own
//! feet, twitches its view on floating-point noise while the stuck/progress watchdogs fire on the
//! zero movement. Instead it **cruises** a short walk from the pickup (scouting approaches, staying
//! close enough to grab it on spawn) and **scans** the room with a slow bearing sweep that the aim
//! spring turns into a smooth human pan. Because perception reads `bot.aim.angles`, the sweep genuinely
//! changes what the bot can see — the scouting is real, not cosmetic.
//!
//! One [`maybe`] call per bot per frame from [`resolve_objective`](super::resolve_objective) decides
//! whether a vigil is warranted and, if so, returns the (overridden) navigation target; the watchdog
//! exemptions and the scan look-point live in `run_bot`. Pure helpers (post/scan/return math) are
//! unit-tested; the driver just wires them to the world with `roam_target`'s borrow order.

use glam::{Vec3, Vec3Swizzles};

use crate::defs::{Solid, VEC_VIEW_OFS};
use crate::entity::{EntId, Think};
use crate::game::GameState;
use crate::navmesh::{CellId, LinkCosts, LinkKind, NavGraph};

/// A goal item this close (XY) and on roughly the same floor is one we're *waiting at*, not still
/// travelling to — so vigil takes over. A bot further out keeps navigating normally.
const VIGIL_NEAR: f32 = 250.0;
const VIGIL_DZ: f32 = 96.0;

/// Cruise-post ring around the pickup: far enough not to idle inside the pickup box (`PICKUP_XY` 40),
/// close enough to scout it and get back on spawn. A handoff hold uses the tight max so the
/// reservation stays defensible against the 400u contest range.
const POST_MIN: f32 = 64.0;
const POST_MAX: f32 = 224.0;
const HOLD_POST_MAX: f32 = 96.0;
const POST_DZ: f32 = 96.0;
/// Consider a post reached within this XY, and re-pick one every `POST_HOLD`+jitter seconds.
const POST_ARRIVE: f32 = 48.0;
const POST_HOLD: f32 = 3.0;
const POST_JITTER: f32 = 2.0;
/// Cap the validation `find_path`s per post pick (candidates are tried from a random offset).
const MAX_POST_TRIES: usize = 8;

/// Timed return: head back to the exact spawn point when the respawn is within travelling range —
/// `dist·CORRIDOR/RETURN_SPEED + MARGIN` seconds out — so the bot is standing on it as it appears.
/// `RETURN_SPEED` is real map travel (not `BOT_MOVE_SPEED`, a wishspeed); `CORRIDOR` pads for a
/// non-straight walk back.
const RETURN_SPEED: f32 = 240.0;
const RETURN_CORRIDOR: f32 = 1.3;
const RETURN_MARGIN: f32 = 0.75;

/// Hold a scan bearing this long (plus jitter) before sweeping to the next; the aim spring pans
/// smoothly across the gap. The golden angle spreads successive looks so the sweep covers the room
/// without settling into a repeating cycle.
const SCAN_HOLD_MIN: f32 = 1.2;
const SCAN_HOLD_JITTER: f32 = 0.8;
const SCAN_DIST: f32 = 384.0;
const GOLDEN_DEG: f32 = 137.508;

/// Decide whether bot `e` is standing vigil over its (uncollectable-right-now) goal item, and if so
/// advance the cruise/scan state and return the navigation target `(world point, its cell)` to steer
/// toward this frame. `None` ⇒ not a vigil (still travelling, or the item is collectable / gone) and
/// the caller keeps its normal target. `holding` is [`update_handoff_hold`]'s verdict.
pub(crate) fn maybe(game: &mut GameState, e: EntId, origin: Vec3, holding: bool, now: f32) -> Option<(Vec3, Option<CellId>)> {
    let item = EntId(game.entities[e].bot.goal.item);
    if item.0 == 0 {
        return None;
    }
    let (item_org, solid, think, nextthink) = {
        let it = &game.entities[item];
        (it.v.origin, it.v.solid, it.think, it.v.nextthink)
    };
    let near = (item_org.xy() - origin.xy()).length() < VIGIL_NEAR && (item_org.z - origin.z).abs() < VIGIL_DZ;
    if !near {
        return None;
    }
    // A hold is an open-ended watch (no respawn clock); a respawn wait has a known return time.
    let respawn_at = if holding {
        None
    } else if solid != Solid::Trigger && matches!(think, Think::SubRegen) && nextthink > now {
        Some(nextthink)
    } else {
        return None; // collectable now (bot_pickup_items will grab it) or truly gone — not a wait
    };
    Some(update(game, e, origin, item_org, holding, respawn_at, now))
}

/// Advance the cruise post and scan bearing and return the frame's navigation target. Mirrors
/// [`roam_target`](super::roam_target)'s borrow order — draw randoms first, then borrow the graph,
/// then write the (disjoint) bot fields.
fn update(game: &mut GameState, e: EntId, origin: Vec3, item_org: Vec3, holding: bool, respawn_at: Option<f32>, now: f32) -> (Vec3, Option<CellId>) {
    // Waiting near a known respawn *is* making progress toward the goal — keep the give-up watchdog
    // (super::resolve_objective's GOAL_GIVEUP_TIME) from abandoning a legitimate wait. A hold is
    // bounded by its own HOLD_MAX deadline, so refreshing this is safe there too.
    game.entities[e].bot.goal.since = now;

    let (post, post_until, scan, scan_until) = {
        let b = &game.entities[e].bot;
        (b.vigil_post, b.vigil_post_until, b.scan_point, b.scan_until)
    };
    let eye = origin + VEC_VIEW_OFS;
    let dist_to_item = (item_org.xy() - origin.xy()).length();
    let returning = timed_return(dist_to_item, respawn_at, now);

    // Randoms up front (need &mut game; the graph borrow below forbids it).
    let (r_post, r_hold, r_scan, r_scanhold) = (game.random(), game.random(), game.random(), game.random());

    let Some(g) = game.nav.graph.as_ref() else {
        return (item_org, None);
    };

    // Scan: sweep to a fresh bearing when the hold lapses (or on first use), else keep the last point
    // so the view settles there.
    let (new_scan, new_scan_until) = if scan == Vec3::ZERO || scan_due(scan_until, now) {
        (pick_scan(eye, scan, r_scan), now + SCAN_HOLD_MIN + r_scanhold * SCAN_HOLD_JITTER)
    } else {
        (scan, scan_until)
    };

    // Cruise post: on a timed return, steer to the spawn itself; otherwise maintain a post in the
    // ring, re-picking on arrival / expiry / when unset. Fall back to the item if nothing validates.
    let max_r = if holding { HOLD_POST_MAX } else { POST_MAX };
    let (target, target_cell, out_post, out_until) = if returning {
        (item_org, g.nearest(item_org), Vec3::ZERO, post_until)
    } else if post != Vec3::ZERO && !post_due(post, post_until, origin, now) {
        (post, g.nearest(post), post, post_until) // keep heading to the current post
    } else if let Some((cell, p)) = g.nearest(origin).and_then(|from| pick_post(g, from, item_org, POST_MIN, max_r, r_post)) {
        (p, Some(cell), p, now + POST_HOLD + r_hold * POST_JITTER)
    } else {
        (item_org, g.nearest(item_org), Vec3::ZERO, post_until) // no trivial post — sit on the item
    };

    let b = &mut game.entities[e].bot;
    b.vigil_post = out_post;
    b.vigil_post_until = out_until;
    b.scan_point = new_scan;
    b.scan_until = new_scan_until;
    (target, target_cell)
}

/// Whether the current cruise post needs replacing: unset, arrived at, or its hold expired.
fn post_due(post: Vec3, until: f32, origin: Vec3, now: f32) -> bool {
    post == Vec3::ZERO || now >= until || (post.xy() - origin.xy()).length() < POST_ARRIVE
}

/// Whether to abandon cruising and head back to the spawn point now, so the bot arrives as the item
/// respawns. `None` (a handoff hold, no respawn clock) never times out.
fn timed_return(dist_xy: f32, respawn_at: Option<f32>, now: f32) -> bool {
    match respawn_at {
        Some(t) => t - now < dist_xy * RETURN_CORRIDOR / RETURN_SPEED + RETURN_MARGIN,
        None => false,
    }
}

/// Whether the scan bearing hold has lapsed.
fn scan_due(until: f32, now: f32) -> bool {
    now >= until
}

/// The next scan look point: a level point `SCAN_DIST` out along a bearing that steps the golden
/// angle past the previous look (or a random start on first use), so the sweep covers the room.
fn pick_scan(eye: Vec3, prev: Vec3, r: f32) -> Vec3 {
    let bearing = if prev != Vec3::ZERO {
        let d = prev - eye;
        d.y.atan2(d.x).to_degrees() + GOLDEN_DEG
    } else {
        r * 360.0
    };
    let rad = bearing.to_radians();
    eye + Vec3::new(rad.cos(), rad.sin(), 0.0) * SCAN_DIST
}

/// A reachable cell in the `[min, max]` XY ring around `item` on roughly its floor, whose route from
/// `from` is trivial (only `Walk`/`Step` legs, no gate) — so the cruise leg is a short stroll and can
/// never turn into a gate errand. Candidates are tried from a random offset; `None` if none validate.
fn pick_post(g: &NavGraph, from: CellId, item: Vec3, min: f32, max: f32, r: f32) -> Option<(CellId, Vec3)> {
    let ring: Vec<CellId> = (0..g.cells.len() as CellId)
        .filter(|&c| {
            let o = g.cells[c as usize].origin;
            let d = (o.xy() - item.xy()).length();
            d >= min && d <= max && (o.z - item.z).abs() < POST_DZ
        })
        .collect();
    if ring.is_empty() {
        return None;
    }
    let start = ((r * ring.len() as f32) as usize).min(ring.len() - 1);
    for i in 0..ring.len().min(MAX_POST_TRIES) {
        let cell = ring[(start + i) % ring.len()];
        if cell == from {
            continue; // a post at our own cell is no cruise — keep the bot actually moving
        }
        if let Some(route) = g.find_path(from, cell, &LinkCosts::default()) {
            if !route.is_empty() && route_is_trivial(g, &route) {
                return Some((cell, g.cell_origin(cell)));
            }
        }
    }
    None
}

/// Whether a route is a plain walk — only `Walk`/`Step` legs and no gated link. An empty route (the
/// bot is already at the cell) is trivially fine.
fn route_is_trivial(g: &NavGraph, route: &[u32]) -> bool {
    route
        .iter()
        .all(|&li| matches!(g.link_kind(li), LinkKind::Walk | LinkKind::Step) && g.gate_of_link(li).is_none())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timed_return_fires_within_travel_window() {
        // now = 0 throughout; 240u away ⇒ ~1.3s travel + 0.75 margin ≈ 2.05s lead.
        assert!(!timed_return(240.0, Some(5.0), 0.0), "far respawn → keep cruising");
        assert!(timed_return(240.0, Some(2.0), 0.0), "within travel+margin → head back");
        assert!(!timed_return(240.0, None, 0.0), "a hold never times out");
        // Boundary: at exactly the threshold it is not yet due (strict <).
        let thresh = 240.0 * RETURN_CORRIDOR / RETURN_SPEED + RETURN_MARGIN;
        assert!(!timed_return(240.0, Some(thresh), 0.0));
        assert!(timed_return(240.0, Some(thresh - 0.01), 0.0));
    }

    #[test]
    fn post_due_on_unset_arrival_and_expiry() {
        let origin = Vec3::new(0.0, 0.0, 0.0);
        assert!(post_due(Vec3::ZERO, 100.0, origin, 0.0), "unset → due");
        let far = Vec3::new(200.0, 0.0, 0.0);
        assert!(!post_due(far, 100.0, origin, 0.0), "far, unexpired → keep");
        assert!(post_due(far, 100.0, origin, 101.0), "expired → due");
        let here = Vec3::new(POST_ARRIVE - 1.0, 0.0, 0.0);
        assert!(post_due(here, 100.0, origin, 0.0), "arrived → due");
    }

    #[test]
    fn scan_sweeps_by_golden_angle_and_stays_distant() {
        let eye = Vec3::new(0.0, 0.0, 22.0);
        let first = pick_scan(eye, Vec3::ZERO, 0.25); // random start
        assert!((first - eye).xy().length() > SCAN_DIST - 1.0, "look point is far off");
        assert!((first.z - eye.z).abs() < 1e-3, "level scan");
        // Next pick steps the golden angle from the previous bearing.
        let b0 = { let d = first - eye; d.y.atan2(d.x).to_degrees() };
        let second = pick_scan(eye, first, 0.0);
        let b1 = { let d = second - eye; d.y.atan2(d.x).to_degrees() };
        let step = (b1 - b0).rem_euclid(360.0);
        assert!((step - GOLDEN_DEG).abs() < 0.5, "bearing advanced ~137.5°, got {step}");
    }

    #[test]
    fn scan_due_respects_hold() {
        assert!(!scan_due(2.0, 1.9));
        assert!(scan_due(2.0, 2.0));
    }
}
