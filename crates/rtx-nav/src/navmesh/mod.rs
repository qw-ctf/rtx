// SPDX-License-Identifier: AGPL-3.0-or-later

//! Auto-generated navigation mesh, built once at map load from the BSP player clip hull
//! (see [`crate::bsp`]). Bots route over this instead of hand-authored waypoint files.
//!
//! The build is pure and offline (no engine syscalls): it asks the hull-1 solidity oracle
//! ([`Bsp::is_solid`], a point test that already accounts for the player box) where a player
//! can stand, drops a [`Cell`] at each floor, and classifies the moves between nearby cells
//! into [`Link`]s (walk / step / drop / jump). Costs are travel times, so A* over the graph
//! yields fast routes (P2).
//!
//! The static-hull cut emits `Walk`/`Step`/`Drop`/`JumpGap`; on top of it come `DoubleJump`,
//! `SpeedJump`, `Hook`, and `RocketJump` movement links (each solved offline against the same
//! solidity oracle), and the entity-derived `Plat`/`Teleport`/gate splices.

use std::collections::HashMap;

use glam::{Vec2, Vec3, Vec3Swizzles};
use rayon::prelude::*;

mod geom;
mod hook;
mod jumps;
mod lod;
mod physics;
mod query;
mod reach;
mod rocketjump;
mod sidetable;
mod splice;

pub use geom::arc_point;
pub use jumps::{
    ground_turn_air_aim, ground_turn_air_cmd, ground_turn_entry_adjust_cmd, ground_turn_entry_ok,
    ground_turn_ground_aim, ground_turn_ground_cmd, ground_turn_ground_cmd_optimal, ground_turn_launch_cmd,
    ground_turn_should_launch, ground_turn_should_launch_optimal, yaw360_of, GroundTurnLiveRollout,
    GroundTurnSetupClock, GroundTurnSetupContinuation, GROUND_TURN_OPTIMAL_VERSION,
    GROUND_TURN_SETUP_AIRBORNE_TICK_CAP, GROUND_TURN_VERSION, RUNWAY_TURN_VERSION,
};
use geom::*;
pub use hook::arc_land;
use hook::{hook_cost, march_to_solid, perturb_ok, HOOK_PITCHES};
#[cfg(test)]
use hook::{simulate_arc, ArcResult};
pub use physics::{
    attainable_speed, band_of, bhop_k, prestrafe_delivered_from, BAND_EDGES, BAND_FLOOR, BAND_V_MAX, BHOP_EFF,
    CURL_PSI_TOL, CURL_V_HOLD_TOL, DOUBLE_ARC_PEAK, JUMP_APEX, MAX_SPEED, NBANDS,
};
pub use lod::{CoarseCosts, Corridor};
use lod::Lod;
use physics::*;
use reach::Reach;
pub use rocketjump::RJ_CERT_AIM_DEG;
use rocketjump::{rj_perturb_ok, rocket_jump_cost, simulate_rocket_jump, RJ_DELAYS, RJ_PITCHES};
use sidetable::SideTable;
pub use splice::{Gate, GateInfo, Plat, PlatInfo, TeleportInfo};

use std::sync::Arc;

use crate::bsp::Bsp;
use crate::qphys::STEP_HEIGHT;

// --- grappling-hook traversal (see `add_hooks`) ---

/// Height above a cell's standing origin the hook launches from (`throw_grapple` spawns it at
/// `origin + 16z`; the small `v_forward*16` XY offset is absorbed by the range margin).
const HOOK_LAUNCH_Z: f32 = 16.0;
/// Reel-in speed at the `rtx_hook_pull ×1` default (`2.35 · 320`, from `grapple.rs`). The live
/// multiplier scales this; the build takes it as a [`HookParams`] field.
pub const HOOK_PULL_BASE: f32 = 2.35 * 320.0;
/// Hook throw (projectile) speed at `rtx_hook_speed ×1` (`2.5 · 320`). Only feeds the flight-time
/// term of the cost; the live multiplier is applied from [`HookParams`].
pub const HOOK_THROW_BASE: f32 = 2.5 * 320.0;
/// Longest rope we'll consider — caps the anchor ray-march and keeps costs bounded.
const HOOK_ROPE_MAX: f32 = 1024.0;
/// Max horizontal reach of a hook link (a fling can cross a wide gap), bounding the candidate scan.
const HOOK_RANGE_XY: f32 = 640.0;
/// Highest rise a hook link may climb (reel + fling reaches well past a plain jump).
const HOOK_MAX_RISE: f32 = 512.0;
/// Lowest a target may sit below the source and still be a hook link (a descending fling).
const HOOK_MIN_RISE: f32 = -128.0;
/// Ray-march / arc-clearance sampling step (matches `path_clear` granularity).
const HOOK_SAMPLE: f32 = 16.0;
/// Spacing of candidate release points sampled along the reel rope.
const HOOK_R_STEP: f32 = 24.0;
/// Fixed timestep for the offline parabola integration (~15 u/step at reel speed).
const HOOK_SIM_DT: f32 = 0.02;
/// Cap on simulated airtime before a candidate arc is abandoned.
const HOOK_MAX_AIRTIME: f32 = 2.5;
/// Landing acceptance: the descending arc must pass within this XY of the target cell…
const HOOK_LAND_XY: f32 = 24.0;
/// …and within this Z window above it.
const HOOK_LAND_Z: f32 = 48.0;
/// Fixed overhead charged to every hook link: aim-settle + weapon switch + throw/release latency.
const HOOK_OVERHEAD: f32 = 1.2;
/// At most this many hook links per source cell (post octant/elevation dedup), to bound explosion.
const HOOK_MAX_PER_CELL: usize = 4;

// --- rocket jumps (blast-launched leaps up to high ledges) ---

/// Max horizontal reach of a rocket-jump link. A floor-fired RJ is mostly vertical, so the reach is
/// tighter than a hook's — an RJ that also travels far is rare and fragile.
const RJ_RANGE_XY: f32 = 400.0;
/// Highest rise a rocket-jump link may climb — the realistic apex (~280u) plus landing slack.
const RJ_MAX_RISE: f32 = 320.0;
/// Lowest a target may sit above the source and still be worth a rocket jump (below this a jump or
/// double jump already reaches — see the useful-gate).
const RJ_MIN_RISE: f32 = 40.0;
/// Landing acceptance window: how far the solved touchdown may sit from a cell for the shot to count
/// as landing *on* it. Tight in Z — a rocket jump must put the bot squarely on the ledge, not a
/// player-height below it. A looser Z (was 48, a full hull) let `nearest_within` snap a landing that
/// fell ~42u short up onto the higher target cell, minting links that structurally undershoot (~24%
/// of them). 24 keeps the snap to at most a half-hull, so a generated link actually reaches its target.
const RJ_LAND_XY: f32 = 24.0;
const RJ_LAND_Z: f32 = 24.0;
/// How far the arc must apex *above* its target to be worth minting. A rocket jump that peaks level
/// with the ledge has to thread the lip exactly; a human instead overshoots and settles down onto the
/// platform, and that spare height is what absorbs aim and timing error. Without this gate the
/// cheapest-arc rule below actively selects *against* margin — `rocket_jump_cost` charges airtime, so
/// the flattest arc that still scrapes the edge always outbids the safe one over it.
const RJ_APEX_MARGIN: f32 = 32.0;
/// At most this many rocket-jump links per source cell — kept small (each costs the bot ~50HP to
/// fly, so a map wants a handful of genuinely-useful ones, not a spray).
const RJ_MAX_PER_CELL: usize = 2;

// --- grid ---

/// XY sampling step. 32 = the player's full width: one column per body. Coarser than the
/// plan's 16 to keep the build cheap on big maps; thin ledges may be missed (revisit). Public so a
/// viewer can tile each cell's walkable footprint at the true grid pitch.
pub const GRID: f32 = 32.0;
/// Player hull half-width (the QW player box is ±16 in X/Y). Used to grow obstacles by the agent
/// radius so a bot doesn't clip geometry its path's centre-line technically clears.
pub(crate) const PLAYER_HALF_WIDTH: f32 = 16.0;
/// Vertical sweep step when scanning a column for floors (refined by bisection after).
const SCAN_DZ: f32 = 8.0;
/// Spacing of the floor-continuity samples along a grounded link (see [`geom::ground_along`]).
/// 8 = four per grid cell; a real support gap only exists where the opening exceeds the 32u player
/// box, so an 8u stride can't step over one.
const GROUND_SAMPLE: f32 = 8.0;
/// Slack added to [`STEP_HEIGHT`] when probing for floor under a grounded link: the interpolated
/// origin height can sit up to a step off the true resting height on a riser, plus plane noise.
const GROUND_SLACK: f32 = 4.0;

/// A standable spot: the player *origin* position when standing here (feet are
/// `ORIGIN_TO_FEET` below `origin.z`). Tagged with its grid column for neighbor lookup.
#[derive(Clone, Copy)]
pub struct Cell {
    pub origin: Vec3,
    pub gx: i32,
    pub gy: i32,
}

pub type CellId = u32;

/// XY spatial index: grid column `(gx, gy)` → the cells carved in that column.
type GridIndex = HashMap<(i32, i32), Vec<CellId>>;

/// How a bot traverses a directed link between two cells.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LinkKind {
    /// Effectively flat ground.
    Walk,
    /// Step up/down within `STEP_HEIGHT` — pmove handles it with no jump.
    Step,
    /// One-way fall off a ledge (down only).
    Drop,
    /// A run-jump across a gap or up to a ledge within reach.
    JumpGap,
    /// A **double jump** across a wider gap / up to a higher ledge than a single jump reaches — the
    /// bot ground-jumps, then air-jumps near the apex (rtx's `rtx_doublejump`). Only emitted when the
    /// map has double jump enabled.
    DoubleJump,
    /// A **speed jump**: a leap across a gap wider than any single/double jump, cleared by arriving
    /// at the takeoff with **bunnyhop-built speed**. The link's `from` is the *start of the runway*
    /// (not the ledge), so taking it means running the whole run-up — the bot is guaranteed the speed
    /// by construction. The [`SpeedJumpTraversal`] side table carries the takeoff point and required
    /// speed. Only emitted when bots bhop (`rtx_bot_bhop`).
    SpeedJump,
    /// Riding a `func_plat`: board it at the bottom and let it carry you to the top. The
    /// link's `from` cell is the standing spot on the plat (its centre), `to` the floor the
    /// plat delivers to. Bots stay centred and wait rather than steering off.
    Plat,
    /// Walking into a `trigger_teleport`: the engine warps you to the destination. No special
    /// traversal — the bot just routes onto an entrance cell and is teleported; it then detects
    /// the jump and re-paths from where it lands.
    Teleport,
    /// A grappling-hook swing: throw the hook at an anchor, reel toward it to build speed, then
    /// **release mid-reel** so the resulting velocity flings the bot along a gravity parabola onto
    /// the target ledge. The vertical pull-up (release at the anchor, near-zero speed, drop
    /// straight down) is just the degenerate case. The per-link [`HookTraversal`] (stored in a side
    /// table, see [`NavGraph::hook_of_link`]) carries the anchor and the release distance the bot
    /// needs to reproduce the arc. Only emitted when the map hands out the hook (`rtx_grapple`).
    Hook,
    /// A **rocket jump**: the bot jumps, and at a solved moment in the rise fires a rocket at the
    /// floor below it; the blast knockback flings it up onto a ledge no jump reaches. The two solved
    /// ingredients — the delay from jump to fire, and the fire direction — plus the self-damage live
    /// in the [`RocketJumpTraversal`] side table ([`NavGraph::rocket_jump_of_link`]). Costs the bot
    /// health, so it's planned only when it clearly beats the detour (a big cost surcharge) and a bot
    /// unfit to fly it (no RL / rocket / health) prices it away. Only emitted when `rtx_bot_rocketjump`.
    RocketJump,
}

/// A directed edge between two cells, with its traversal kind and travel-time cost.
#[derive(Clone, Copy)]
pub struct Link {
    pub from: CellId,
    pub to: CellId,
    pub kind: LinkKind,
    pub cost: f32,
}

/// The built navigation graph: cells, directed links, per-cell adjacency (indices into
/// `links`), and an XY spatial index for `nearest`/neighbor queries.
pub struct NavGraph {
    pub cells: Vec<Cell>,
    pub links: Vec<Link>,
    pub adjacency: Vec<Vec<u32>>,
    /// Per-cell "the standing origin is under water" flag (parallel to `cells`), so the planner can
    /// price swimming above walking and the runtime can tell a wet cell from a dry one. Empty until
    /// [`flag_water`](Self::flag_water) runs on the worker build (from the render hull's liquid-carrying
    /// `pointcontents`); an empty vec reads as "all dry" via [`cell_in_water`](Self::cell_in_water).
    water: Vec<bool>,
    /// Per-cell "a bot standing here can breathe" flag (parallel to `cells`): its eye point is out of
    /// the water, so it's a spot a drowning bot can path to for air. Filled alongside `water`; an
    /// empty vec reads as "all breathable" via [`cell_breathable`](Self::cell_breathable) — the safe
    /// default for an unmarked (dry) graph.
    breathable: Vec<bool>,
    /// Per-cell "a bot standing here is *in* lava or slime" (parallel to `cells`): the engine's
    /// waterlevel-1 sample (feet+1) reads that liquid, so the game burns anyone standing on the cell —
    /// including interior cells of a shallow film the liquid-edge probe can't see. Never `Pit`. Filled
    /// alongside `water` by [`flag_hazards`](Self::flag_hazards); an empty vec reads as "no cell burns"
    /// via [`cell_hazard`](Self::cell_hazard).
    hazard: Vec<Option<crate::hazard::HazardKind>>,
    /// Health a bot expects to lose *taking* each link (parallel to `links`), `0.0` on the
    /// overwhelming majority. Filled by [`flag_hazards`](Self::flag_hazards) from the map's real
    /// damage model — tick size × waterlevel × dwell, plus a risk premium on links onto a pool's
    /// edge. Kept as **health, not seconds**, because what a point of health is worth depends on who
    /// is asking: [`link_extra`](Self::link_extra) converts it per query against the bot's own
    /// strength. An empty vec reads as "nothing hurts".
    hazard_hp: Vec<f32>,
    /// Extra seconds each link costs for entering water (parallel to `links`): swimming is slower
    /// than running and carries no bunnyhop. Filled by [`flag_water`](Self::flag_water); an empty vec
    /// reads as "no water tax". Unlike `hazard_hp` this is a flat time price, not a health one —
    /// water doesn't burn, it just slows you down.
    water_extra: Vec<f32>,
    /// Per-cell "this spot is inside a `func_plat`'s swept volume" (parallel to `cells`), holding the
    /// lift's index in `plats`. A body resting here blocks the lift's descent *and* keeps resetting
    /// its inner trigger's lower-timer, so a raised plat never comes down — hence these cells are
    /// transit-only: a bot may cross one or grab an item on it, but must never park there. Unlike
    /// `water`/`hazard` this needs no `pointcontents` lookup at all (pure clip-hull geometry), and
    /// [`add_plats`](Self::add_plats) fills it on the worker build; an empty vec reads as "no cell is
    /// under a lift" via [`cell_under_plat`](Self::cell_under_plat).
    under_plat: Vec<Option<u16>>,
    /// Per-cell "beside a fatal drop" flag (parallel to `cells`): the ground falls away by more than a
    /// survivable step within a stride of the cell — a wall-hugging walkway over an open pit, a spiral
    /// staircase's inner edge. Pure build-side geometry (no engine callback), filled by
    /// [`flag_ledges`](Self::flag_ledges); lets the runtime bhop policy drop to a walk there (a fast bot
    /// carries off the inner edge at a corner) without the near-field's airborne staleness. An empty vec
    /// reads as "no ledge" via [`is_ledge`](Self::is_ledge) — the safe default for a bare graph.
    ledge: Vec<bool>,
    grid: GridIndex,
    /// The closed door/movewall each link's segment passes through — the "navmesh aware of dynamic
    /// geometry" core, so pathfinding can price a link by its door's live state (see
    /// [`find_path`](Self::find_path)). Empty until [`add_gates`](Self::add_gates) runs.
    gates: SideTable<Gate>,
    /// How to fly each [`LinkKind::Hook`] link (its solved [`HookTraversal`]).
    hooks: SideTable<HookTraversal>,
    /// The takeoff point + required entry speed for each speed-jump link, for the bot executor.
    speed_jumps: SideTable<SpeedJumpTraversal>,
    /// The fire delay + angles + self-damage for each rocket-jump link, for the bot executor.
    rocket_jumps: SideTable<RocketJumpTraversal>,
    /// Spliced `func_plat` lifts (entity id + footprint), tagged onto the ride link and every
    /// jump-aboard link that boards each plat, so the runtime can find which lift a leg boards and
    /// hold a standoff while it's raised.
    plats: SideTable<Plat>,
    /// The bhop speed-gain constant `k` for this map's movement cvars ([`bhop_k`]), captured when
    /// speed jumps are spliced (or left at the stock default). The banded planner reads it to price
    /// speed carried between links; keeping it here avoids re-reading cvars at query time.
    sj_k: f32,
    /// Static reachability (SCCs + forward closure), filled by [`build_reachability`](Self::build_reachability)
    /// at the end of the build so [`reachable`](Self::reachable) answers "can A ever get to B?" in O(1)
    /// instead of a failed whole-graph search. `None` on a bare (unbuilt) graph — see [`reach`].
    reach: Option<Reach>,
    /// Level-of-detail hierarchy (cluster assignment + abstract portal graph), filled by
    /// [`build_lod`](Self::build_lod) at the end of the build. Coarse far-field navigation reads it;
    /// `None` on a bare (unbuilt) graph — see [`lod`].
    lod: Option<Lod>,
}

/// A solved speed jump: where the takeoff ledge is and the horizontal speed needed there, so the
/// runtime can refuse to leap if the bot somehow reaches the edge too slow to clear the gap.
#[derive(Clone, Copy)]
pub struct SpeedJumpTraversal {
    pub takeoff: Vec3,
    pub v_req: f32,
    /// Flight time of the leap (s), so the banded planner can price a hot entry (the runway term
    /// shrinks with carried speed while the airtime is fixed).
    pub airtime: f32,
    /// Minimum certified horizontal landing speed. Populated by curl
    /// traversals whose full cadence/contact envelope was rolled; zero means
    /// that no landing-speed claim exists and the planner must use `v_req`.
    pub landing_speed_lo: f32,
    /// A **chained** speed jump: it has no self-contained runway (the `from` cell *is* the ledge),
    /// so it is only traversable when the planner proves the entry band already carries `v_req`
    /// (see [`NavGraph::find_path_banded`]). Unbanded queries price it away via [`NavGraph::link_extra`].
    pub chained: bool,
    /// Air-curl gain for a jump whose run-up and leap are **not collinear** (a curl jump — the bot runs
    /// down one corridor and turns in the air onto an offset landing). `0` = a straight speed jump: the
    /// runtime flies it with the hop slalom, unchanged. `> 0` = curl: once airborne the controller homes
    /// the velocity onto the landing with [`air_correct`](crate) at this rate — one smooth pursuit arc.
    pub curl_gain: f32,
    /// Optional first pursuit point for a two-phase curl. `curl_switch_dist == 0` disables the
    /// profile and preserves the original single-target curl. A signed distance permits a profile
    /// to use the entry aim only for the launch frame and switch immediately once airborne. When enabled, the runtime pursues
    /// this point until it has travelled `curl_switch_dist` units from `takeoff` along the
    /// takeoff→entry-aim axis, then pursues `curl_landing_aim` for the remainder of the flight.
    /// Keeping the solved points on the traversal makes the BSP certificate and live executor use
    /// the same geometry without map-specific branches in the controller.
    pub curl_entry_aim: Vec3,
    pub curl_switch_dist: f32,
    pub curl_landing_aim: Vec3,
    /// A **chained ground-turn curl**: the leap cannot be flown from a local run-up at all — it
    /// needs carried entry speed (the link is `chained`) *and* a grounded rotation before the jump
    /// (the launch heading is not the corridor heading, and rotating in the air after a lip launch
    /// provably cannot close the flight-time budget). `Some` carries the complete certified
    /// controller contract; `None` = every other speed jump, unchanged. See [`GroundTurnCurl`].
    pub ground_turn: Option<GroundTurnCurl>,
}

/// The self-contained, versioned contract for one chained ground-turn curl (see
/// [`SpeedJumpTraversal::ground_turn`]). Both the build-time certifier and the live executor drive
/// the same version-selected controller functions from exactly these numbers, so what was proven
/// offline is what flies. Fail-closed: the executor
/// checks the live entry state against the stored envelope and abandons the leg (replans) when
/// outside it, instead of improvising over the lip.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundTurnCurl {
    /// Contract version; bump on any semantic change to the controller or envelope fields.
    pub version: u16,
    /// Ground steering waypoint held while farther than `turn_dist` from the takeoff point: keeps
    /// the run (a stepped runway is fine — step-ups resolve as `stepped`, not wall contact) on the
    /// certified corridor line.
    pub runway_aim: Vec3,
    /// Self-contained runway-turn policy. When true, ground steering blends
    /// continuously from `runway_yaw` to `launch_yaw`, holds
    /// `hold_speed`, and launches at `lip_reach`; false preserves the
    /// original chained switch-at-distance controller.
    pub blended_runway: bool,
    pub runway_yaw: f32,
    pub lip_reach: f32,
    pub hold_speed: f32,
    /// Distance from the takeoff point at which ground steering switches from `runway_aim` to
    /// rotating the carried velocity toward `launch_yaw`.
    pub turn_dist: f32,
    /// Launch heading in degrees, [0,360) (atan2 convention, west = 180).
    pub launch_yaw: f32,
    /// Grounded velocity-yaw gate in degrees, [0,360): the jump fires on the first grounded tick
    /// inside the takeoff box at/above this yaw and at/above `box_min.z`.
    pub yaw_min: f32,
    /// Takeoff box: the jump may fire anywhere inside this XY AABB once the yaw gate is met;
    /// `box_min.z` doubles as the minimum launch height (must be on the takeoff platform).
    pub box_min: Vec3,
    pub box_max: Vec3,
    /// Launch-frame air-steer gain toward `hold_aim`.
    pub launch_gain: f32,
    /// Air phase A pursues `hold_aim` until the flight crosses the gate plane
    /// (`dot(origin - gate_point, gate_normal) < 0`), then pursues `landing_aim` at `air_gain`.
    /// A gate plane placed at the takeoff makes the curl immediate (phase A never runs).
    pub hold_aim: Vec3,
    pub gate_point: Vec3,
    pub gate_normal: Vec3,
    pub air_gain: f32,
    pub landing_aim: Vec3,
    /// Certified entry envelope at the link source (grounded arrival): horizontal speed and
    /// velocity yaw360 ranges. Outside ⇒ the executor must NOT commit the leg.
    pub entry_speed_lo: f32,
    pub entry_speed_hi: f32,
    pub entry_yaw_lo: f32,
    pub entry_yaw_hi: f32,
    /// Minimum horizontal landing speed observed across the certified envelope — the carry the
    /// banded planner may credit downstream of this link.
    pub landing_speed_lo: f32,
    /// Certified landing heading (degrees, [0,360), centre envelope corner) — the direction the
    /// carried speed actually points after the curl, which the banded planner's corner cone must
    /// use instead of the from→to chord (the flight rotates far off the chord by design).
    pub landing_yaw: f32,
}

/// Is `yaw` (degrees, [0,360)) inside the wrap-aware envelope `[lo, hi]` (also [0,360); `lo > hi`
/// means the envelope crosses 0)?
pub fn yaw_in_envelope(yaw: f32, lo: f32, hi: f32) -> bool {
    if lo <= hi {
        yaw >= lo && yaw <= hi
    } else {
        yaw >= lo || yaw <= hi
    }
}

/// Extra travel-time cost charged to a link whose gate is currently shut. Large enough that the
/// planner routes around a closed door whenever any open way exists, but finite so it still
/// crosses (and the bot then detours to the button) when there's no alternative — matching how a
/// game engine prices a disabled-but-openable NavMesh link rather than deleting it outright.
pub const CLOSED_GATE_PENALTY: f32 = 100_000.0;

/// Extra travel-time charged to every [`LinkKind::RocketJump`] link when the querying bot is unfit
/// to fly one (no rocket launcher, no rocket, too little health, or quad running — see
/// the game's `bot::rj::rocket_jump_extra`). Same magnitude as [`CLOSED_GATE_PENALTY`]: the planner
/// diverts around rocket jumps it can't make, yet — being finite — still takes one as a last resort
/// down a sole corridor rather than treating the graph as severed.
pub const RJ_UNFIT_PENALTY: f32 = 100_000.0;

/// Peak fraction of a link's own cost added as deterministic per-caller jitter when
/// [`LinkCosts::jitter_seed`] is set — enough to break ties between near-equal routes (so two bots
/// vary their paths) without ever reordering genuinely-cheaper alternatives.
const JITTER_FRAC: f32 = 0.10;

/// Per-query dynamic costs layered on each link's static cost: live gate state, plus an optional
/// caller-supplied surcharge (a bot's recently-failed links) and deterministic jitter (per-bot
/// route variety). **Every term is non-negative**, so A*'s straight-line heuristic stays an
/// admissible lower bound and routes stay optimal-or-diverted, never wrong. Cheap to pass by value.
#[derive(Default, Clone, Copy)]
pub struct LinkCosts<'a> {
    /// `gate_closed[i]` marks gate `i`'s door currently shut; a link through it is charged
    /// [`CLOSED_GATE_PENALTY`]. Empty slice ⇒ every door treated as open.
    pub gate_closed: &'a [bool],
    /// `openable_gates[i]` marks closed gate `i` as one this querying bot can open right now — its
    /// button is reachable without crossing the gate — so a link through it is charged the modest
    /// [`Self::open_gate_cost`] (a button-detour errand) instead of the full [`CLOSED_GATE_PENALTY`].
    /// Empty (the default) ⇒ every closed gate keeps the full penalty, which is what *path* planning
    /// always wants. This is the one term that differs between "how good is this goal" and "how do I
    /// get there": a prize behind a door the bot knows how to open is a fine goal to *choose*, but the
    /// route to it should still prefer an open way when one exists. Only goal *valuation* sets it.
    pub openable_gates: &'a [bool],
    /// Seconds charged for crossing an [`Self::openable_gates`] gate — the button-detour overhead the
    /// gate errand actually pays, not the route-around penalty. Unread when `openable_gates` is empty.
    pub open_gate_cost: f32,
    /// `(link idx, extra seconds)` surcharges — a bot's failed-link penalties. Tiny (≤8 entries),
    /// scanned linearly. Kept far below [`CLOSED_GATE_PENALTY`] so it diverts a route without ever
    /// forcing one through a shut door.
    pub penalties: &'a [(u32, f32)],
    /// Nonzero ⇒ add `hash(seed ^ link) → [0, JITTER_FRAC]·link.cost` per link, so bots with distinct
    /// seeds pick different near-equal corridors. Zero ⇒ no jitter (deterministic; tests, non-bots).
    pub jitter_seed: u32,
    /// Nonzero ⇒ charge every [`LinkKind::RocketJump`] link this many extra seconds — the per-bot
    /// capability gate. A bot currently unable to rocket-jump sets it to [`RJ_UNFIT_PENALTY`] so it
    /// plans around them; `0` (the default) leaves rocket jumps at their solved cost. One of the two
    /// price terms that depend on *who* is asking (the rest are world state), because unlike the
    /// grapple — a server-wide cvar — a bot's rockets and health vary moment to moment.
    pub rocket_jump_extra: f32,
    /// `Some(_)` ⇒ price lava/slime links for this bot, converting each link's `hazard_hp` to seconds
    /// via [`hazard_cost`]. The weaker the bot, the longer the detour it will accept to stay out of
    /// the pool; with armor and full health it barely notices a clipped corner.
    ///
    /// `None` (the default) ⇒ hazards unpriced, for callers measuring pure traversal time: race-line
    /// verification and the offline optimizer, whose authored lines deliberately cross lava and whose
    /// estimates are compared against a timeout. Deliberately not `0.0`-means-inert like the terms
    /// above: strength zero means *dead*, which has to mean maximum avoidance, not none.
    pub hazard: Option<HazardPrice>,
}

/// What a hazard is worth to the bot asking — the whole per-bot half of [`hazard_cost`].
#[derive(Clone, Copy)]
pub struct HazardPrice {
    /// The bot's effective hit points (`total_strength`: health scaled by armor absorption, which
    /// lava damage really does go through). The denominator of its nerve.
    pub strength: f32,
    /// Seconds of detour accepted per unit of fraction-of-strength lost ([`HAZARD_TIME_K`]); higher is
    /// more timid. Carried per query rather than read as a constant so the tuning harness can sweep it
    /// live (`rtx_bot_hazard_k`) without a rebuild.
    pub k: f32,
}

impl HazardPrice {
    /// Price hazards for a bot of effective `strength` at the stock timidity.
    pub fn new(strength: f32) -> Self {
        Self { strength, k: HAZARD_TIME_K }
    }
}

/// A cheap integer hash (variant of the SplitMix/Murmur finalizer) for deterministic route jitter.
#[inline]
fn hash32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

/// Ledge probe ([`ledge_beside`]): how far off a cell centre to sample, how far the ground must fall
/// there to count as a fatal edge, and the vertical sweep step. `REACH` catches a cell on a
/// one-to-two-grid-cell walkway beside a pit; a step-down whose floor sits within `FATAL_DROP`, or a
/// wall, leaves the cell unflagged. `√½` gives the diagonal samples a unit stride.
const LEDGE_REACH: f32 = 44.0;
const LEDGE_FATAL_DROP: f32 = 64.0;
const LEDGE_SCAN_DZ: f32 = 8.0;
const SQRT_HALF: f32 = 0.707_106_77;

/// Whether a fatal drop sits within [`LEDGE_REACH`] of `origin` in any of eight directions — a
/// wall-hugging walkway over an open pit, a spiral staircase's inner edge. In each direction the sampled
/// column must be open air from a step above the floor down past [`LEDGE_FATAL_DROP`]: solid anywhere
/// between is a wall (pillar / step-up) or a catch-floor (a mere step-down, survivable), so that
/// direction is not an edge. Pure over `is_solid` (the hull-1 point test), so it is unit-testable and
/// runs on the worker build; see [`NavGraph::flag_ledges`].
fn ledge_beside(is_solid: &impl Fn(Vec3) -> bool, origin: Vec3) -> bool {
    const DIRS: [(f32, f32); 8] = [
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
        (SQRT_HALF, SQRT_HALF),
        (SQRT_HALF, -SQRT_HALF),
        (-SQRT_HALF, SQRT_HALF),
        (-SQRT_HALF, -SQRT_HALF),
    ];
    DIRS.iter().any(|&(dx, dy)| {
        let (x, y) = (origin.x + dx * LEDGE_REACH, origin.y + dy * LEDGE_REACH);
        let mut z = origin.z + STEP_HEIGHT;
        while z > origin.z - LEDGE_FATAL_DROP {
            if is_solid(Vec3::new(x, y, z)) {
                return false;
            }
            z -= LEDGE_SCAN_DZ;
        }
        true
    })
}

impl NavGraph {
    /// Build the graph from a parsed BSP's player hull. Pure; safe to run at load time.
    pub fn build(bsp: &Bsp) -> NavGraph {
        let cells_grid = Self::carve_cells(bsp);
        let mut graph = NavGraph {
            adjacency: vec![Vec::new(); cells_grid.0.len()],
            cells: cells_grid.0,
            links: Vec::new(),
            water: Vec::new(),       // filled on the worker by flag_water
            breathable: Vec::new(),  // (from the render hull's liquid-carrying pointcontents)
            water_extra: Vec::new(), // (same)
            hazard: Vec::new(),      // filled on the worker by flag_hazards (same reason)
            hazard_hp: Vec::new(),   // (same)
            under_plat: Vec::new(),  // filled by add_plats (pure geometry — no pointcontents needed)
            ledge: Vec::new(),       // filled by flag_ledges below (pure geometry too)
            grid: cells_grid.1,
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED), // stock default until add_speed_jumps captures live cvars
            reach: None,                   // filled by build_reachability once all links are spliced
            lod: None,                     // filled by build_lod once all links are spliced
        };
        graph.link_cells(bsp);
        graph.flag_ledges(bsp);
        graph
    }

    /// A bare graph for the unit tests: `cells` + `links`, adjacency derived from the links, every
    /// optional column empty (so it reads as dry, unhazardous and gate-free) and stock bhop physics.
    /// Exists so that adding a column doesn't mean editing eight field-by-field literals — the friction
    /// that kept the hazard pricing out of the tests' sight while it silently did nothing in play.
    #[cfg(test)]
    pub(super) fn test_graph(cells: Vec<Cell>, links: Vec<Link>) -> NavGraph {
        let mut adjacency = vec![Vec::new(); cells.len()];
        for (i, l) in links.iter().enumerate() {
            adjacency[l.from as usize].push(i as u32);
        }
        NavGraph {
            cells,
            links,
            adjacency,
            water: Vec::new(),
            breathable: Vec::new(),
            water_extra: Vec::new(),
            hazard: Vec::new(),
            hazard_hp: Vec::new(),
            under_plat: Vec::new(),
            ledge: Vec::new(),
            grid: GridIndex::default(),
            gates: SideTable::default(),
            hooks: SideTable::default(),
            speed_jumps: SideTable::default(),
            rocket_jumps: SideTable::default(),
            plats: SideTable::default(),
            sj_k: bhop_k(10.0, MAX_SPEED),
            reach: None,
            lod: None,
        }
    }

    /// Flag every cell beside a fatal drop (see [`ledge_beside`]) — a wall-hugging walkway over an open
    /// pit, a spiral staircase's inner edge. Pure geometry over the hull-1 point test, so it runs on the
    /// worker build; a lava/slime-flanked cell is left to the `hazard` flag instead. The runtime bhop
    /// policy reads [`is_ledge`](Self::is_ledge) to walk here rather than carry speed off the inner edge
    /// at a corner (the near-field can't — it goes stale while the bot is airborne mid-hop).
    fn flag_ledges(&mut self, bsp: &Bsp) {
        self.ledge = self.cells.par_iter().map(|cell| ledge_beside(&|p| bsp.is_solid(p), cell.origin)).collect();
    }

    /// Whether a cell sits beside a fatal drop (see [`flag_ledges`](Self::flag_ledges)). Empty flags —
    /// a bare graph, or one built without the pass — read as `false` (no ledge).
    pub fn is_ledge(&self, c: CellId) -> bool {
        self.ledge.get(c as usize).copied().unwrap_or(false)
    }

    /// Sweep every grid column for floors and emit one [`Cell`] at the bottom of each empty
    /// span (a surface the player can rest on). Returns the cells plus their XY spatial index.
    fn carve_cells(bsp: &Bsp) -> (Vec<Cell>, GridIndex) {
        let (gx0, gy0) = (floor_grid(bsp.mins.x), floor_grid(bsp.mins.y));
        let (gx1, gy1) = (floor_grid(bsp.maxs.x), floor_grid(bsp.maxs.y));

        // Scan columns for floors in parallel — the column sweep is a big share of the build and the
        // columns are independent geometry. One `(gx, gy, origin_z)` row per `gx`; the indexed
        // `collect` keeps rows in `gx` order (a `RangeInclusive<i32>` is an indexed parallel
        // iterator) and `gy` is sequential within a row, so the serial ID-assignment pass below
        // reproduces the single-threaded cell order — cell IDs and the grid index come out identical.
        let rows: Vec<Vec<(i32, i32, f32)>> = (gx0..=gx1)
            .into_par_iter()
            .map(|gx| {
                let mut row = Vec::new();
                for gy in gy0..=gy1 {
                    let (x, y) = (gx as f32 * GRID, gy as f32 * GRID);
                    Self::column_floors(bsp, x, y, |origin_z| row.push((gx, gy, origin_z)));
                }
                row
            })
            .collect();

        let mut cells = Vec::new();
        let mut grid: GridIndex = HashMap::new();
        for (gx, gy, origin_z) in rows.into_iter().flatten() {
            let id = cells.len() as CellId;
            cells.push(Cell {
                origin: Vec3::new(gx as f32 * GRID, gy as f32 * GRID, origin_z),
                gx,
                gy,
            });
            grid.entry((gx, gy)).or_default().push(id);
        }
        (cells, grid)
    }

    /// Scan one column bottom-to-top; for each solid→empty transition (a floor with headroom
    /// for the standing hull) call `emit(origin_z)` with the bisected resting origin height.
    fn column_floors(bsp: &Bsp, x: f32, y: f32, mut emit: impl FnMut(f32)) {
        let at = |z: f32| bsp.is_solid(Vec3::new(x, y, z));
        let mut z = bsp.mins.z;
        let mut prev_solid = true; // below the world is solid
        while z <= bsp.maxs.z {
            let solid = at(z);
            if prev_solid && !solid {
                // Resting origin is between the last solid sample and this empty one.
                emit(bisect_floor(bsp, x, y, z - SCAN_DZ, z));
            }
            prev_solid = solid;
            z += SCAN_DZ;
        }
    }

    /// Classify the moves out of every cell into directed links: grounded moves to the 8
    /// grid-adjacent columns, then jumps across gaps / up to ledges (windowed and deduped).
    fn link_cells(&mut self, bsp: &Bsp) {
        // Classify per cell in parallel (`classify_grounded`/`find_jumps` are read-only), collecting
        // each cell's links grounded-then-jumps, then splice serially. Indexed `collect` keeps cell
        // order and the per-cell order is preserved, so link indices are identical to a serial build.
        let this = &*self;
        let per_cell: Vec<Vec<Link>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let c = this.cells[from as usize];
                let mut out = Vec::new();
                for to in this.neighbors_within(c.gx, c.gy, 1) {
                    if to != from {
                        if let Some(link) = this.classify_grounded(bsp, from, to) {
                            out.push(link);
                        }
                    }
                }
                out.extend(this.find_jumps(bsp, from));
                out
            })
            .collect();
        // A short near-apex rise can have a very narrow *slow* speed window: geometrically
        // reachable, but an ordinary runner reaches its front face before gaining the target
        // height. Preserve such links when they are the only way onto a ledge (the runtime will
        // eventually carry their solved speed envelope), but prune them when this same build has
        // already found a max-speed-safe takeoff farther back to the exact target cell. This removes
        // a wall-hit shortcut without disconnecting deliberate slow hop-ups such as DM3 MH.
        let mut has_hot_jump_to = vec![false; self.cells.len()];
        for link in per_cell.iter().flatten().filter(|l| l.kind == LinkKind::JumpGap) {
            let a = self.cells[link.from as usize].origin;
            let b = self.cells[link.to as usize].origin;
            if b.z > a.z && ballistic_clear_at_speed(bsp, a, b, MAX_SPEED) {
                has_hot_jump_to[link.to as usize] = true;
            }
        }
        for link in per_cell.into_iter().flatten() {
            let a = self.cells[link.from as usize].origin;
            let b = self.cells[link.to as usize].origin;
            let redundant_knife_edge = link.kind == LinkKind::JumpGap
                && b.z > a.z
                && has_hot_jump_to[link.to as usize]
                && !ballistic_clear_at_speed(bsp, a, b, MAX_SPEED);
            if !redundant_knife_edge {
                self.push_link(link);
            }
        }
    }

    fn push_link(&mut self, link: Link) {
        let idx = self.links.len() as u32;
        self.adjacency[link.from as usize].push(idx);
        self.links.push(link);
    }

    /// A grounded move (walk/step/drop) to a grid-adjacent cell, if the path is clear. An
    /// adjacent rise between step height and a standing jump's apex — a knee-high ledge or a
    /// slope too steep for a free step — is a short **hop up** (a `JumpGap`): pmove needs a jump
    /// to mount it, but it's basic movement (all modes), not a gap leap. Taller rises need the
    /// windowed ledge jumps in [`find_jumps`].
    fn classify_grounded(&self, bsp: &Bsp, from: CellId, to: CellId) -> Option<Link> {
        let (a, b) = (self.cells[from as usize], self.cells[to as usize]);
        let dz = b.origin.z - a.origin.z;
        if dz > STEP_HEIGHT && dz <= JUMP_APEX {
            // A rise in the jump band that is really a **walkable staircase** — two shallow risers
            // caught inside one 32u grid span — climbs by stepping, not jumping. Emit a `Step` (the same
            // path the ≤STEP_HEIGHT case takes) so the bot walks it instead of pogoing each riser, and
            // so a bhop/glide can treat the flight as a runway. A genuine knee-high ledge (one tall
            // riser) fails `steppable_rise` and stays a JumpGap below.
            if steppable_rise(&|p| bsp.is_solid(p), a.origin, b.origin)
                && path_clear(bsp, a.origin, b.origin)
                && ground_along(&|p| bsp.is_solid(p), a.origin, b.origin)
            {
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                return Some(Link { from, to, kind: LinkKind::Step, cost: link_cost(LinkKind::Step, horiz, dz) });
            }
            // Hop up onto the adjacent higher footing; clear the standing-jump arc to it. Not from a
            // submerged cell, though — you can't jump when submerged (the jump input swims up).
            if bsp.is_liquid_at(a.origin) {
                return None;
            }
            return arc_clear(bsp, a.origin, b.origin).then(|| {
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                Link {
                    from,
                    to,
                    kind: LinkKind::JumpGap,
                    cost: link_cost(LinkKind::JumpGap, horiz, dz),
                }
            });
        }
        let kind = if dz.abs() <= WALK_DZ {
            LinkKind::Walk
        } else if dz.abs() <= STEP_HEIGHT {
            LinkKind::Step
        } else if (-MAX_DROP..-STEP_HEIGHT).contains(&dz) {
            // A drop is only real off a **ledge**: the column stepped toward the target must not
            // have walkable ground at our height (mirrors the ledge check in `find_jumps`). Without
            // this, a cell in the middle of a floor sprouts phantom drops onto lower cells beneath
            // it — `path_clear` only samples the top corridor, so it never sees the solid slab in
            // between (worst on thin floors, where a lower room sits directly under every cell). A
            // bot handed such a link freezes trying to "drop" straight through solid ground.
            let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                return None;
            }
            // And the hull must actually fit down into the target — not a slot too small for it.
            if !descent_clear(bsp, a.origin.z, b.origin) {
                return None;
            }
            LinkKind::Drop
        } else {
            return None; // up beyond a jump's apex — needs the windowed ledge jumps
        };
        if !path_clear(bsp, a.origin, b.origin) {
            return None;
        }
        // `path_clear` only rules out walls/ceilings along the head-height corridor; it can't see an
        // air gap *under* the segment. For a Walk/Step to a grid-diagonal neighbor around an L-shaped
        // ledge (a stair side, a balcony corner), the centre-to-centre line clips the corner's air —
        // the bot walks straight off. Require floor to continue beneath the whole segment. (Drop keeps
        // its own `has_ground_near` + `descent_clear`; JumpGap flew the `arc_clear` branch above.)
        if matches!(kind, LinkKind::Walk | LinkKind::Step) && !ground_along(&|p| bsp.is_solid(p), a.origin, b.origin) {
            return None;
        }
        let horiz = (b.origin.xy() - a.origin.xy()).length();
        Some(Link {
            from,
            to,
            kind,
            cost: link_cost(kind, horiz, dz),
        })
    }

    /// A cell in grid column `(gx, gy)` within `STEP_HEIGHT` of height `z`, if any.
    fn cell_near(&self, gx: i32, gy: i32, z: f32) -> Option<CellId> {
        self.grid.get(&(gx, gy)).and_then(|ids| {
            ids.iter()
                .copied()
                .find(|&id| (self.cells[id as usize].origin.z - z).abs() <= STEP_HEIGHT)
        })
    }

    /// Whether grid column `(gx, gy)` has a cell within `STEP_HEIGHT` of height `z` — i.e.
    /// walkable ground continues there (so a jump isn't warranted).
    fn has_ground_near(&self, gx: i32, gy: i32, z: f32) -> bool {
        self.grid.get(&(gx, gy)).is_some_and(|ids| {
            ids.iter()
                .any(|&id| (self.cells[id as usize].origin.z - z).abs() <= STEP_HEIGHT)
        })
    }

    /// Cell ids in grid columns within Chebyshev `radius` of `(gx, gy)`.
    fn neighbors_within(&self, gx: i32, gy: i32, radius: i32) -> Vec<CellId> {
        let mut out = Vec::new();
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                if let Some(ids) = self.grid.get(&(gx + dx, gy + dy)) {
                    out.extend_from_slice(ids);
                }
            }
        }
        out
    }

    /// Flag every cell's liquid footing and price each link by the **health** crossing it costs, so
    /// the planner keeps its distance whenever a comparable safe route exists. Two kinds of cell pay:
    ///
    ///  * **standing *in* the liquid** — the cell's own footing (feet+1, the engine's waterlevel-1
    ///    sample) is lava/slime, so the game burns a bot parked there. Persisted in `self.hazard`,
    ///    priced per entering link from the real damage model. This catches shallow films *and*
    ///    interior cells of a walkable-bottom pool, which the edge probe below cannot see.
    ///  * **on the liquid's *edge*** — an open side a stride out drops onto lava/slime (a bank a bot
    ///    walks along). Priced by the smaller [`HAZARD_EDGE_HP`]. An in-liquid cell doesn't also pay
    ///    the edge tax (it's already the worse case).
    ///
    /// The price lands in `hazard_hp` (health), **not** in `link.cost` (seconds). It used to be baked
    /// into `link.cost`, and that quietly did nothing: `banded_step`'s Walk/Step arm derives its cost
    /// from speed and never reads `link.cost`, and the banded planner is the live one — so every bot
    /// walked into lava priced as bare floor. `link.cost` was the wrong home for risk anyway (it is a
    /// *distance/speed* quantity), and health can't be converted to seconds here regardless: the
    /// exchange rate depends on the health of whoever is asking. [`link_extra`](Self::link_extra) —
    /// which all three searches do honor — does the conversion per query.
    ///
    /// Pits are deliberately not flagged here (every balcony/ledge cell borders a drop — that would
    /// surcharge half the map); the runtime combat guard [`crate::hazard::hazard_ahead`] keeps bots
    /// from stepping off edges in a fight. Only *liquids* get a routing bias. `contents` is a
    /// render-hull `pointcontents` (the parsed BSP's [`crate::bsp::Bsp::pointcontents`], which carries
    /// liquid contents the clip hull does not); [`build_navmesh`] runs this on the worker with the BSP
    /// in hand, before the reachability/LOD passes so their tables price the liquid at birth.
    pub fn flag_hazards(&mut self, is_solid: &impl Fn(Vec3) -> bool, contents: &impl Fn(Vec3) -> i32) {
        let liquid = |p: Vec3| match contents(p) {
            crate::bsp::CONTENTS_LAVA => Some(crate::hazard::HazardKind::Lava),
            crate::bsp::CONTENTS_SLIME => Some(crate::hazard::HazardKind::Slime),
            _ => None,
        };
        // Each cell's liquid footing and how deep a bot standing there wades. The engine deals
        // `tick · waterlevel`, so depth is a 3× damage spread between clipping a corner (wl 1) and
        // swimming a pit (wl 3) — flattening it would price those the same. These are pmove's own
        // three sample heights: feet+1 (`mins.z + 1`, which is also SV_CheckWater's burn sample, so
        // the kind matches the damage exactly), the box mid-point, and the view offset.
        let depth: Vec<(Option<crate::hazard::HazardKind>, f32)> = self
            .cells
            .iter()
            .map(|c| {
                let kind = liquid(c.origin - Vec3::new(0.0, 0.0, 23.0));
                let wl = if kind.is_none() {
                    0.0
                } else if liquid(c.origin + Vec3::new(0.0, 0.0, 4.0)).is_none() {
                    1.0
                } else if liquid(c.origin + Vec3::new(0.0, 0.0, 22.0)).is_none() {
                    2.0
                } else {
                    3.0
                };
                (kind, wl)
            })
            .collect();
        self.hazard = depth.iter().map(|&(kind, _)| kind).collect();
        let edge: Vec<bool> = (0..self.cells.len() as CellId)
            .map(|id| self.cell_on_liquid_edge(id, is_solid, contents))
            .collect();
        self.hazard_hp = self
            .links
            .iter()
            .map(|link| {
                let (tick_hp, tick_secs) = match depth[link.to as usize].0 {
                    Some(crate::hazard::HazardKind::Lava) => (LAVA_TICK_HP, LAVA_TICK_SECS),
                    Some(crate::hazard::HazardKind::Slime) => (SLIME_TICK_HP, SLIME_TICK_SECS),
                    // Pit never comes off a footing sample; an edge cell pays the risk premium, and a
                    // link *leaving* a pool is free — that's the gradient that pulls a bot to shore.
                    _ => return if edge[link.to as usize] { HAZARD_EDGE_HP } else { 0.0 },
                };
                let horiz =
                    (self.cells[link.to as usize].origin.xy() - self.cells[link.from as usize].origin.xy()).length();
                // Conservative dwell: walking pace, not the bhop the bot may actually carry.
                let ticks = (horiz.max(GRID) / MAX_SPEED) / tick_secs;
                // `dmgtime` starts in the past, so stepping in costs a whole tick before any dwell
                // accrues — but only on the way *in*. Charging that entry tick per cell would price an
                // N-cell wade N times over.
                let ticks = if depth[link.from as usize].0.is_some() { ticks } else { ticks.max(1.0) };
                tick_hp * depth[link.to as usize].1 * ticks
            })
            .collect();

        // Jump-over-lava surcharge. The pricing above reads only the *target* cell's footing — for a
        // jump that's the safe far platform, so a leap over a lava pool reads as free and the router
        // sends bots sailing over the coals. But an undershoot (arriving a hair slow at the takeoff)
        // drops into the pool, not onto the platform — a near-certain kill. Price the span itself: if
        // any point of the fall-short zone between the two footings sits over lava/slime, charge the
        // fatal [`JUMP_LAVA_HP`] so the router prefers the walk-around. A sole-route lava jump still
        // costs finite (the [`HAZARD_COST_MAX`] cap), so a bot with no other way across still attempts
        // it. Computed as a separate pass (a fresh immutable borrow of the links/cells) after the map.
        let over_lava: Vec<bool> = self
            .links
            .iter()
            .map(|link| {
                matches!(
                    link.kind,
                    LinkKind::JumpGap | LinkKind::DoubleJump | LinkKind::SpeedJump | LinkKind::RocketJump
                ) && {
                    let a = self.cells[link.from as usize].origin;
                    let b = self.cells[link.to as usize].origin;
                    // Sample the interior of the platform-to-platform span (endpoints are safe footing).
                    (1..=3).any(|i| {
                        let p = a.lerp(b, i as f32 / 4.0);
                        matches!(
                            crate::hazard::hazard_below(is_solid, contents, p),
                            Some(crate::hazard::HazardKind::Lava | crate::hazard::HazardKind::Slime)
                        )
                    })
                }
            })
            .collect();
        for (hp, &over) in self.hazard_hp.iter_mut().zip(over_lava.iter()) {
            if over {
                *hp = hp.max(JUMP_LAVA_HP);
            }
        }
    }

    /// Whether cell `id` sits on the edge of a lava or slime pool: for each compass direction with
    /// no walkable neighbour (an open side), probe a stride out at foot height and check for a
    /// liquid below. A drop to lower ground, plain water, or a wall all read as safe.
    fn cell_on_liquid_edge(
        &self,
        id: CellId,
        is_solid: &impl Fn(Vec3) -> bool,
        contents: &impl Fn(Vec3) -> i32,
    ) -> bool {
        let c = self.cells[id as usize];
        // Standing player feet sit 24u below the origin (player `mins.z`); probe from there.
        let feet = c.origin - Vec3::new(0.0, 0.0, 24.0);
        let step = |v: f32| if v > 0.1 { 1 } else if v < -0.1 { -1 } else { 0 };
        crate::hazard::HAZARD_DIRS.iter().any(|&(dx, dy)| {
            if self.has_ground_near(c.gx + step(dx), c.gy + step(dy), c.origin.z) {
                return false; // walkable ground continues this way — not an edge
            }
            let p = feet + Vec3::new(dx, dy, 0.0) * HAZARD_PROBE_R + Vec3::new(0.0, 0.0, 8.0);
            matches!(
                crate::hazard::hazard_below(is_solid, contents, p),
                Some(crate::hazard::HazardKind::Lava | crate::hazard::HazardKind::Slime)
            )
        })
    }

    /// Flag every cell whose standing origin is under water, and price swimming above walking. Fills
    /// the parallel `water`/`breathable` vectors — an origin in water means a bot swims here; an eye
    /// point out of the water (`origin + 22`, pmove's waterlevel-3 sample) means a spot it can
    /// breathe — then charges every link *entering* a water cell the [`WATER_COST_MULT`] premium in
    /// `water_extra`. Exit links (water → dry) are left free, so the price forms a cost gradient the
    /// planner follows back to shore rather than a uniform pool tax.
    ///
    /// Like [`flag_hazards`](Self::flag_hazards) the premium lives in its own column rather than
    /// baked into `link.cost`, where `banded_step` never read it (see that method for the full story).
    /// Expressed as the equivalent additive delta, `(mult − 1) · cost`: identical to the old multiply
    /// for the searches that do read `link.cost`, and conservative for the banded one.
    ///
    /// Also like [`flag_hazards`](Self::flag_hazards) this reads liquid contents from a render-hull
    /// `pointcontents` (the clip hull is liquid-blind), and [`build_navmesh`] runs it on the worker
    /// before the reachability/LOD passes so the pool tax is baked into the abstract graph.
    pub fn flag_water(&mut self, contents: &impl Fn(Vec3) -> i32) {
        let is_water = |p: Vec3| contents(p) == crate::bsp::CONTENTS_WATER;
        self.water = self.cells.iter().map(|c| is_water(c.origin)).collect();
        // Eye height for the breathe test: the standing view offset (pmove samples waterlevel 3 here).
        let eye = Vec3::new(0.0, 0.0, 22.0);
        self.breathable = self.cells.iter().map(|c| !is_water(c.origin + eye)).collect();
        // The extra seconds the slower stroke costs over the same ground, straight from the geometry
        // — not `(mult − 1) · link.cost`, which would only be the swim time for links whose cost *is*
        // `horiz / MAX_SPEED`. A `Drop` into a pool pays its fall and then the swim, not a fifth more
        // falling.
        self.water_extra = self
            .links
            .iter()
            .map(|l| {
                if !self.water[l.to as usize] {
                    return 0.0;
                }
                let horiz = (self.cells[l.to as usize].origin.xy() - self.cells[l.from as usize].origin.xy()).length();
                horiz.max(GRID) / MAX_SPEED * (WATER_COST_MULT - 1.0)
            })
            .collect();
    }

    /// Extra A* cost for link `li` under `costs`: closed-gate penalty + this caller's per-link
    /// surcharge + optional deterministic jitter + the water and lava/slime prices. All non-negative,
    /// keeping the A* heuristic admissible (see [`LinkCosts`]).
    ///
    /// This is the one addend every search shares — [`find_path`](Self::find_path),
    /// [`find_path_banded`](Self::find_path_banded) and [`costs_from`](Self::costs_from) all route
    /// through here — which is why the liquid prices live here rather than baked into `link.cost`,
    /// where the banded planner never saw them.
    #[inline]
    fn link_extra(&self, li: u32, costs: &LinkCosts) -> f32 {
        let mut extra = match self.gate_of_link(li) {
            Some(g) if costs.gate_closed.get(g).copied().unwrap_or(false) => {
                // A closed gate this caller flagged openable (its button is reachable on our side) is
                // an errand, not a wall — price it so, but only for the query that asked. Every other
                // caller leaves `openable_gates` empty and pays the full route-around penalty.
                if costs.openable_gates.get(g).copied().unwrap_or(false) {
                    costs.open_gate_cost
                } else {
                    CLOSED_GATE_PENALTY
                }
            }
            _ => 0.0,
        };
        for &(l, sec) in costs.penalties {
            if l == li {
                extra += sec;
                break;
            }
        }
        if costs.jitter_seed != 0 {
            let h = hash32(costs.jitter_seed ^ li.wrapping_mul(0x9e37_79b1));
            extra += (h as f32 / u32::MAX as f32) * JITTER_FRAC * self.links[li as usize].cost;
        }
        if costs.rocket_jump_extra > 0.0 && self.links[li as usize].kind == LinkKind::RocketJump {
            extra += costs.rocket_jump_extra;
        }
        extra += self.water_extra.get(li as usize).copied().unwrap_or(0.0);
        // Health is only convertible to seconds against a particular bot's health, so an unpriced
        // query (`hazard_strength: None`) skips it — and the vast majority of links cost no health at
        // all, so check that first.
        if let Some(price) = costs.hazard {
            let hp = self.hazard_hp.get(li as usize).copied().unwrap_or(0.0);
            if hp > 0.0 {
                extra += hazard_cost(hp, price);
            }
        }
        extra
    }

    /// The block a *speed-unaware* query (`find_path`, `costs_from`) must add to a chained speed
    /// jump: it has no self-contained runway, so a route that doesn't reason about carried speed can
    /// never take one. Large enough to sever it in practice, finite so it never poisons `g` beyond
    /// the existing [`CLOSED_GATE_PENALTY`] scale. The banded planner ([`Self::find_path_banded`])
    /// bypasses this and gates chained jumps on the entry band instead.
    pub(super) fn chained_block(&self, li: u32) -> f32 {
        match self.speed_jump_of_link(li) {
            Some(t) if t.chained => CLOSED_GATE_PENALTY,
            _ => 0.0,
        }
    }

    /// The banded transition for link `li` entered at speed band `entry`: its travel-time cost and
    /// the band the bot arrives in, or `None` if the leg is infeasible at this entry speed (a
    /// chained speed jump the carried speed can't satisfy). Conservative by construction — speeds
    /// are band *floors*, gains are derated ([`BHOP_EFF`]), and no leg demotes a carried band except
    /// where physics forces it (a hard fall, or a teleport/plat/hook/rocket that resets speed).
    /// Every cost is floored at `horiz / BAND_V_MAX`, keeping the banded heuristic admissible.
    pub fn banded_step(&self, li: u32, entry: u8) -> Option<(f32, u8)> {
        let link = self.links[li as usize];
        let from = self.cells[link.from as usize].origin;
        let to = self.cells[link.to as usize].origin;
        let horiz = (to.xy() - from.xy()).length();
        let dz = to.z - from.z;
        let floor_cost = horiz / BAND_V_MAX;
        let v_in = BAND_FLOOR[entry as usize].max(MAX_SPEED);
        Some(match link.kind {
            LinkKind::Walk | LinkKind::Step => {
                // Swimming isn't walking: pmove drives a submerged bot at [`SWIM_SPEED`] and it can't
                // bunnyhop at all, so a leg into water neither climbs a band nor keeps one. Crediting
                // it the dry-corridor gain (as this arm did, water being invisible here) plans a
                // downstream chained jump off speed no bot can carry out of a pool. The *time* the
                // slower stroke costs isn't added here — `water_extra` charges it, once, for every
                // search alike.
                if self.cell_in_water(link.to) {
                    return Some(((horiz / MAX_SPEED).max(floor_cost), band_of(SWIM_SPEED)));
                }
                // Already moving (band ≥ 1): carry speed and climb. From a standstill spend a spin-up
                // runway before gains begin. But an ascending leg — a stair riser (`dz > WALK_DZ`) —
                // builds no bhop speed (a human runs up stairs), so it carries the band without gain
                // and without demotion (a lone step must not zero a chain's plan; the runtime runway
                // and carry gates handle real staircases). `.max(v_in)` never demotes on a short leg.
                let v_out = if dz > WALK_DZ {
                    v_in
                } else {
                    let usable = if entry >= 1 { horiz } else { (horiz - BAND_SPINUP).max(0.0) };
                    v_in.max(BHOP_EFF * attainable_speed(v_in, usable, self.sj_k))
                };
                let avg = ((v_in + v_out) * 0.5).max(MAX_SPEED);
                ((horiz / avg).max(floor_cost), band_of(v_out))
            }
            LinkKind::SpeedJump => {
                let (v_req, airtime, chained, curl_gain) = self
                    .speed_jump_of_link(li)
                    .map(|t| (t.v_req, t.airtime, t.chained, t.curl_gain))
                    .unwrap_or((MAX_SPEED, 0.0, false, 0.0));
                // A chained ground-turn curl carries its own certified entry envelope — the proof
                // the generic `SJ_MARGIN` exists to approximate — and its stored cost covers the
                // whole leg (run-up, rotation, flight) end-to-end. Gate on the envelope floor and
                // exit at the certified minimum landing speed; the runtime executor re-checks the
                // same envelope fail-closed before committing.
                if let Some(gt) = self.speed_jump_of_link(li).and_then(|t| t.ground_turn) {
                    if v_in < gt.entry_speed_lo {
                        return None;
                    }
                    return Some((link.cost.max(floor_cost), band_of(gt.landing_speed_lo)));
                }
                // A curl speed jump was certified end-to-end at build time — the rollout solver measured
                // a run-up that the ground circle-strafe genuinely delivers (which the conservative
                // air-strafe recompute below has no term for and badly under-credits). So trust its
                // stored cost. Exit at the certified takeoff *floor* (`v_req`), NOT the carried entry: the
                // takeoff regime grounds the whole run-up, so friction caps the takeoff at the prestrafe
                // equilibrium regardless of how fast the bot arrived — crediting `v_in` would plan a
                // downstream chained jump off a band the runtime can't actually carry through the curl.
                if curl_gain > 0.0 && !chained {
                    let landing = self
                        .speed_jump_of_link(li)
                        .map(|t| t.landing_speed_lo)
                        .filter(|&v| v.is_finite() && v > 0.0)
                        .unwrap_or(v_req);
                    return Some((link.cost.max(floor_cost), band_of(landing)));
                }
                // A chained jump has no runway: traversable only if the entry band already carries it.
                if chained && v_in < v_req * SJ_MARGIN {
                    return None;
                }
                // The runway run-up shrinks with carried speed (0 once at v_req); airtime is fixed.
                let runway_t = runway_time(v_req * SJ_MARGIN, v_in, self.sj_k);
                // Horizontal speed is conserved through the leap: the bot lands carrying whatever it
                // took off with — the carried entry (chained) or the runway-built v_req (stand start),
                // whichever is greater. So a chain of jumps sustains its band instead of decaying.
                let v_exit = v_in.max(v_req * SJ_MARGIN);
                ((runway_t + airtime + 1.0).max(floor_cost), band_of(v_exit))
            }
            // A hard fall stumbles to a standstill; a short drop keeps the band.
            LinkKind::Drop => (link.cost, if -dz <= SAFE_FALL { entry } else { 0 }),
            LinkKind::JumpGap => (link.cost, entry),
            LinkKind::DoubleJump => (link.cost, entry.saturating_sub(1)),
            // Teleport / plat ride / hook / rocket jump all deliver the bot at a standstill.
            LinkKind::Teleport | LinkKind::Plat | LinkKind::Hook | LinkKind::RocketJump => (link.cost, 0),
        })
    }

    // --- entity-independent: grappling-hook swing links ---

    /// Splice grappling-hook swing links into the graph. From each **ledge-edge** cell, fire probe
    /// rays out over the drop-off at a few pitches, find where the hook would anchor, then sample
    /// release points along the reel and simulate the resulting gravity parabola — whatever standable
    /// cell the arc lands on becomes the link's target. This discovers both vertical pull-ups and
    /// long horizontal flings from one mechanism. Only accepted when the arc (and perturbed variants)
    /// land safely, so a bot is never flung into a pit. Deduped per direction/elevation and capped.
    pub fn add_hooks(&mut self, bsp: &Bsp, params: HookParams) {
        // Solve per source cell in parallel (immutable borrow), then splice serially (push_hook needs
        // `&mut`). Indexed `collect` preserves cell order, so link indices match a sequential build.
        let this = &*self;
        let pending: Vec<Vec<(Link, HookTraversal)>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let mut out = Vec::new();
                this.solve_hooks_from(bsp, from, params, &mut out);
                out
            })
            .collect();
        for (link, tr) in pending.into_iter().flatten() {
            self.push_hook(link, tr);
        }
    }

    /// Solve the hook links leaving cell `from`, appending accepted `(Link, HookTraversal)` to `out`.
    fn solve_hooks_from(&self, bsp: &Bsp, from: CellId, params: HookParams, out: &mut Vec<(Link, HookTraversal)>) {
        let c = self.cells[from as usize];
        let a = c.origin;
        let launch = a + Vec3::new(0.0, 0.0, HOOK_LAUNCH_Z);
        // best per (compass octant, 128u elevation band): (cost, link, traversal)
        let mut best: HashMap<(usize, i32), (f32, Link, HookTraversal)> = HashMap::new();

        for (dgx, dgy) in COMPASS {
            // Only launch out over a ledge/gap in this direction — hooking toward continuing ground
            // is pointless (walk/step/jump already cover it).
            if self.has_ground_near(c.gx + dgx, c.gy + dgy, a.z) {
                continue;
            }
            let yaw = (dgy as f32).atan2(dgx as f32);
            for pitch_deg in HOOK_PITCHES {
                let pitch = pitch_deg.to_radians();
                let (sp, cp) = pitch.sin_cos();
                let (sy, cy) = yaw.sin_cos();
                let dir = Vec3::new(cp * cy, cp * sy, sp);
                let Some(stick) = march_to_solid(|p| bsp.is_solid(p), launch, dir, HOOK_ROPE_MAX) else {
                    continue;
                };
                let rope = (stick - launch).length();
                if rope < HOOK_SAMPLE {
                    continue;
                }
                let v0 = dir * params.pull;
                let rdir = v0.normalize_or_zero();
                // Sample release points along the reel by their distance from the anchor. Everything
                // keys off `stick` + `release_dist` (the reconstructable form the runtime re-solve and
                // the test use), so the stored arc reproduces bit-for-bit — no fp drift on a grazing
                // sample. Leave some rope reeled and some swing left.
                let mut release_dist = HOOK_SAMPLE;
                while release_dist < rope - HOOK_SAMPLE {
                    let r = stick - rdir * release_dist;
                    if let Some((land, airtime, vz)) = arc_land(bsp, r, v0, params.gravity) {
                        if let Some(to) = self.nearest_within(land, HOOK_LAND_XY, HOOK_LAND_Z) {
                            if to != from {
                                let b = self.cells[to as usize].origin;
                                let dz = b.z - a.z;
                                let horiz = (b.xy() - a.xy()).length();
                                let useful = dz > JUMP_APEX || horiz > JUMP_REACH;
                                let in_range = (HOOK_MIN_RISE..=HOOK_MAX_RISE).contains(&dz) && horiz <= HOOK_RANGE_XY;
                                if useful
                                    && in_range
                                    && !self.has_direct_link(from, to)
                                    && perturb_ok(bsp, stick, rdir, release_dist, rope, params, b)
                                {
                                    let cost = hook_cost(rope, release_dist, airtime, vz, params);
                                    let key = (dir_bucket(dgx, dgy), (dz / 128.0).floor() as i32);
                                    if best.get(&key).is_none_or(|(bc, _, _)| cost < *bc) {
                                        let link = Link {
                                            from,
                                            to,
                                            kind: LinkKind::Hook,
                                            cost,
                                        };
                                        let tr = HookTraversal {
                                            stick,
                                            release_dist,
                                            v0,
                                            airtime,
                                        };
                                        best.insert(key, (cost, link, tr));
                                    }
                                }
                            }
                        }
                    }
                    release_dist += HOOK_R_STEP;
                }
            }
        }

        // Keep the cheapest few per cell. Break cost ties by target cell then dedup key, so the
        // survivors don't depend on `HashMap` iteration order (randomized per instance — and under
        // parallel building a tie would otherwise resolve differently run to run).
        let mut chosen: Vec<_> = best.into_iter().collect();
        chosen.sort_by(|(ak, (ac, al, _)), (bk, (bc, bl, _))| {
            ac.total_cmp(bc).then(al.to.cmp(&bl.to)).then(ak.cmp(bk))
        });
        chosen.truncate(HOOK_MAX_PER_CELL);
        out.extend(chosen.into_iter().map(|(_, (_, link, tr))| (link, tr)));
    }

    /// Splice rocket-jump links: for each cell, fire a rocket at a solved delay/angle during a jump
    /// and keep the launches that land on a higher ledge no cheaper move reaches. `double_jump` gates
    /// the useful height. See [`super::rocketjump`] for the two-phase ballistics.
    pub fn add_rocket_jumps(&mut self, bsp: &Bsp, params: RocketJumpParams, double_jump: bool) {
        // Solve per source cell in parallel (immutable borrow), then splice serially (push needs
        // `&mut`). Indexed `collect` preserves cell order, so link indices match a sequential build.
        let this = &*self;
        let pending: Vec<Vec<(Link, RocketJumpTraversal)>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let mut out = Vec::new();
                this.solve_rocket_jumps_from(bsp, from, params, double_jump, &mut out);
                out
            })
            .collect();
        for (link, tr) in pending.into_iter().flatten() {
            self.push_rocket_jump(link, tr);
        }
    }

    /// Solve the rocket-jump links leaving cell `from`, appending accepted `(Link, RocketJumpTraversal)`
    /// to `out`. Unlike hooks there's no ledge-edge skip — the classic RJ launches from flat ground up
    /// a wall face — so all eight travel octants are tried, firing opposite the travel direction.
    fn solve_rocket_jumps_from(
        &self,
        bsp: &Bsp,
        from: CellId,
        params: RocketJumpParams,
        double_jump: bool,
        out: &mut Vec<(Link, RocketJumpTraversal)>,
    ) {
        let a = self.cells[from as usize].origin;
        if bsp.is_liquid_at(a) {
            return; // submerged takeoff: can't jump to start the rocket jump (the jump input swims up)
        }
        let is_solid = |p: Vec3| bsp.is_solid(p);
        // The rocket is a zero-size point, so it collides on the render hull (hull 0) and reaches the
        // true floor/wall — ~24u below the inflated player-hull surface `is_solid` reports. The solve
        // detonates the shot on this hull so the blast geometry matches what the engine produces.
        let rocket_solid = |p: Vec3| bsp.is_point_solid(p);
        // Height an RJ must clear to earn its health cost: past a plain (or double) jump's apex.
        let useful_apex = if double_jump { DOUBLE_JUMP_APEX } else { JUMP_APEX };
        // best per (compass octant, 128u elevation band): (cost, link, traversal)
        let mut best: HashMap<(usize, i32), (f32, Link, RocketJumpTraversal)> = HashMap::new();

        for (dgx, dgy) in COMPASS {
            // Fire opposite the travel direction: the blast lands behind-and-below, shoving the bot
            // up and toward the ledge.
            let fire_yaw = (dgy as f32).atan2(dgx as f32).to_degrees() + 180.0;
            for pitch in RJ_PITCHES {
                for delay in RJ_DELAYS {
                    let angles = Vec3::new(pitch, fire_yaw, 0.0);
                    let Some(s) = simulate_rocket_jump(is_solid, rocket_solid, a, angles, delay, params) else {
                        continue;
                    };
                    let Some(to) = self.nearest_within(s.land, RJ_LAND_XY, RJ_LAND_Z) else {
                        continue;
                    };
                    if to == from {
                        continue;
                    }
                    let b = self.cells[to as usize].origin;
                    let dz = b.z - a.z;
                    let horiz = (b.xy() - a.xy()).length();
                    let useful = dz > useful_apex; // height is the whole point of an RJ in v1
                    let in_range = (RJ_MIN_RISE..=RJ_MAX_RISE).contains(&dz) && horiz <= RJ_RANGE_XY;
                    // Peak of the post-blast parabola. Closed-form is exact here: `simulate_rocket_jump`
                    // only returns a solution when the arc flew unobstructed to a floor, so nothing
                    // clipped the rise.
                    let apex = s.pos_blast.z + s.v0.z.max(0.0).powi(2) / (2.0 * params.gravity);
                    if useful
                        && in_range
                        && apex >= b.z + RJ_APEX_MARGIN
                        && !self.has_direct_link(from, to)
                        && rj_perturb_ok(is_solid, rocket_solid, a, angles, delay, params, b)
                    {
                        let cost = rocket_jump_cost(s.t_blast, s.airtime, s.vz_land, s.self_damage);
                        let key = (dir_bucket(dgx, dgy), (dz / 128.0).floor() as i32);
                        if best.get(&key).is_none_or(|(bc, _, _)| cost < *bc) {
                            let link = Link { from, to, kind: LinkKind::RocketJump, cost };
                            let tr = RocketJumpTraversal {
                                fire_angles: angles,
                                fire_delay: delay,
                                blast: s.blast,
                                pos_blast: s.pos_blast,
                                v0: s.v0,
                                land: s.land,
                                airtime: s.airtime,
                                self_damage: s.self_damage,
                            };
                            best.insert(key, (cost, link, tr));
                        }
                    }
                }
            }
        }

        // Keep the cheapest few per cell. Break cost ties by target cell then dedup key, so the
        // survivors don't depend on `HashMap` iteration order (randomized per instance — and under
        // parallel building a tie would otherwise resolve differently run to run).
        let mut chosen: Vec<_> = best.into_iter().collect();
        chosen.sort_by(|(ak, (ac, al, _)), (bk, (bc, bl, _))| {
            ac.total_cmp(bc).then(al.to.cmp(&bl.to)).then(ak.cmp(bk))
        });
        chosen.truncate(RJ_MAX_PER_CELL);
        out.extend(chosen.into_iter().map(|(_, (_, link, tr))| (link, tr)));
    }

    /// Whether `from` already has a direct (non-hook) link to `to` — such a target needs no hook.
    fn has_direct_link(&self, from: CellId, to: CellId) -> bool {
        self.adjacency[from as usize]
            .iter()
            .any(|&li| self.links[li as usize].to == to)
    }

    /// The solved traversal for hook link `li`, or `None` for a non-hook link.
    pub fn hook_of_link(&self, li: u32) -> Option<&HookTraversal> {
        self.hooks.of_link(li)
    }

    /// Push a hook link with its solved traversal, tagging the new link in the `hooks` side table.
    fn push_hook(&mut self, link: Link, traversal: HookTraversal) {
        let h = self.hooks.push(traversal);
        self.push_link(link);
        self.hooks.tag(self.links.len() - 1, h);
    }

    /// The solved traversal for speed-jump link `li`, or `None` for any other link.
    pub fn speed_jump_of_link(&self, li: u32) -> Option<&SpeedJumpTraversal> {
        self.speed_jumps.of_link(li)
    }

    /// Hand-plant a speed-jump link post-build (harness / bring-up): inject a `SpeedJump` from `from`
    /// to `to` with the given cost and solved traversal, returning its link index. Lets the control
    /// harness validate a takeoff/curl the generator doesn't yet emit — the runtime flies a planted
    /// link exactly like a generated one. Not used by the automatic build.
    pub fn plant_speed_jump(&mut self, from: CellId, to: CellId, cost: f32, traversal: SpeedJumpTraversal) -> u32 {
        self.push_speed_jump(Link { from, to, kind: LinkKind::SpeedJump, cost }, traversal);
        (self.links.len() - 1) as u32
    }

    /// Rebuild the derived tables — the reachability closure and the LOD hierarchy — after a post-build
    /// topology mutation such as [`plant_speed_jump`](Self::plant_speed_jump). The automatic build runs
    /// these once at the end over the final link set; a hand-planted link (harness bring-up) must
    /// refresh them, or the O(1) [`reachable`](Self::reachable) gate and the coarse router keep
    /// answering for the *pre-plant* graph — e.g. a `goto` to a ledge reachable only across the planted
    /// jump would see `reachable` return false and redirect to the nearest old-reachable cell instead
    /// of pathing over it. The live graph's liquid columns are already filled, so `build_lod` folds
    /// them in directly (no separate liquid patch needed).
    pub fn rebuild_derived(&mut self) {
        self.build_reachability();
        self.build_lod();
    }

    /// Push a speed-jump link with its traversal, tagging the new link in the side table.
    fn push_speed_jump(&mut self, link: Link, traversal: SpeedJumpTraversal) {
        let s = self.speed_jumps.push(traversal);
        self.push_link(link);
        self.speed_jumps.tag(self.links.len() - 1, s);
    }

    /// The solved traversal for rocket-jump link `li`, or `None` for any other link.
    pub fn rocket_jump_of_link(&self, li: u32) -> Option<&RocketJumpTraversal> {
        self.rocket_jumps.of_link(li)
    }

    /// Push a rocket-jump link with its traversal, tagging the new link in the side table.
    fn push_rocket_jump(&mut self, link: Link, traversal: RocketJumpTraversal) {
        let r = self.rocket_jumps.push(traversal);
        self.push_link(link);
        self.rocket_jumps.tag(self.links.len() - 1, r);
    }

    /// Hand-plant a rocket-jump link post-build (harness / bring-up): run the real two-phase RJ
    /// ballistics from the cell nearest `from` onto a landing cell near `tgt`, unconstrained by the
    /// automatic generator's search caps — `RJ_RANGE_XY` / `RJ_MIN_RISE..=RJ_MAX_RISE` and the
    /// useful-apex rule bound the map-wide candidate *spray*, not physics, and a curated plant has
    /// already decided the jump is wanted. Certification is NOT relaxed: the shot must simulate
    /// clean on both hulls, resolve to a standable landing cell within the standard tolerance
    /// (`RJ_LAND_XY`/`RJ_LAND_Z`), apex past it by `RJ_APEX_MARGIN`, and survive the same
    /// perturbation check as a generated link. The yaw sweeps ±20° around the straight-line
    /// bearing so an off-axis opening (e.g. a window) the 8-octant search never aims at can still
    /// be threaded. The cheapest surviving arc is inserted; the runtime flies a planted link
    /// exactly like a generated one. Works on a graph built with rocket-jump generation off.
    /// Not used by the automatic build.
    pub fn plant_rocket_jump(
        &mut self,
        bsp: &Bsp,
        from: Vec3,
        tgt: Vec3,
        params: RocketJumpParams,
    ) -> Result<u32, String> {
        /// How far the resolved landing cell may sit from the requested `tgt` and still count as
        /// "the target": a few grid columns of slack, since the caller aims at a ledge, not a cell.
        const TGT_XY: f32 = 96.0;
        const TGT_Z: f32 = 64.0;
        let from_cell = self.nearest(from).ok_or("no cell near from")?;
        let a = self.cells[from_cell as usize].origin;
        if bsp.is_liquid_at(a) {
            return Err("submerged takeoff: can't jump to start the rocket jump".into());
        }
        let is_solid = |p: Vec3| bsp.is_solid(p);
        let rocket_solid = |p: Vec3| bsp.is_point_solid(p);
        // Fire opposite the travel direction, like the generator.
        let to_xy = tgt.xy() - a.xy();
        if to_xy.length() < 1.0 {
            return Err("tgt is directly above from; no bearing to fire against".into());
        }
        let base_yaw = to_xy.y.atan2(to_xy.x).to_degrees() + 180.0;
        let mut best: Option<(f32, Link, RocketJumpTraversal)> = None;
        let mut clean = 0u32;
        let mut best_miss = f32::INFINITY;
        // Full-circle yaw sweep, 15° steps: a curated plant is a one-shot solve, so unlike the
        // map-wide generator we can afford to try every wall — the blast's push direction is set by
        // which surface the rocket finds, and the geometry that reaches an off-axis target is often
        // a wall far from the straight-line bearing.
        for yaw_step in 0..24 {
            let yaw_off = yaw_step as f32 * 15.0 - 180.0;
            for pitch in RJ_PITCHES {
                for delay in RJ_DELAYS {
                    let angles = Vec3::new(pitch, base_yaw + yaw_off, 0.0);
                    let Some(s) =
                        simulate_rocket_jump(is_solid, rocket_solid, a, angles, delay, params)
                    else {
                        continue;
                    };
                    clean += 1;
                    best_miss = best_miss.min((s.land - tgt).length());
                    // Harness diagnostics for a failing plant: where does every clean arc land?
                    if std::env::var_os("RTX_PLANT_RJ_DEBUG").is_some() {
                        eprintln!(
                            "planrj arc: yaw_off {yaw_off} pitch {pitch} delay {delay} -> land \
                             {:.0} {:.0} {:.0} (miss {:.0}u, apex {:.0})",
                            s.land.x,
                            s.land.y,
                            s.land.z,
                            (s.land - tgt).length(),
                            s.pos_blast.z + s.v0.z.max(0.0).powi(2) / (2.0 * params.gravity),
                        );
                    }
                    let Some(to) = self.nearest_within(s.land, RJ_LAND_XY, RJ_LAND_Z) else {
                        continue;
                    };
                    if to == from_cell {
                        continue;
                    }
                    let b = self.cells[to as usize].origin;
                    if (b.xy() - tgt.xy()).length() > TGT_XY || (b.z - tgt.z).abs() > TGT_Z {
                        continue;
                    }
                    let apex = s.pos_blast.z + s.v0.z.max(0.0).powi(2) / (2.0 * params.gravity);
                    if apex >= b.z + RJ_APEX_MARGIN
                        && rj_perturb_ok(is_solid, rocket_solid, a, angles, delay, params, b)
                    {
                        let cost = rocket_jump_cost(s.t_blast, s.airtime, s.vz_land, s.self_damage);
                        if best.as_ref().is_none_or(|(bc, _, _)| cost < *bc) {
                            let link = Link { from: from_cell, to, kind: LinkKind::RocketJump, cost };
                            let tr = RocketJumpTraversal {
                                fire_angles: angles,
                                fire_delay: delay,
                                blast: s.blast,
                                pos_blast: s.pos_blast,
                                v0: s.v0,
                                land: s.land,
                                airtime: s.airtime,
                                self_damage: s.self_damage,
                            };
                            best = Some((cost, link, tr));
                        }
                    }
                }
            }
        }
        let Some((_, link, tr)) = best else {
            return Err(format!(
                "no certifiable arc: {clean} clean simulations, best landing miss {best_miss:.0}u \
                 from tgt {:.0} {:.0} {:.0}",
                tgt.x, tgt.y, tgt.z
            ));
        };
        self.push_rocket_jump(link, tr);
        Ok((self.links.len() - 1) as u32)
    }

    /// Hand-plant a rocket-jump link with caller-supplied fire parameters and NO offline
    /// certification — the bring-up primitive for lift-assisted rocket jumps, which the static
    /// solver cannot certify: a rising `func_plat` adds launch velocity that exists only at
    /// runtime (see the pentlift→window refutation in `tests/plant_rocket_jump_dm3.rs`). The
    /// traversal is synthesized — the runtime flies jump+fire with exactly these angles and delay
    /// and reports the real outcome through the standard rj telemetry, so certification happens
    /// live, by trial. Harness-only; never emitted by the automatic build.
    pub fn plant_rocket_jump_raw(
        &mut self,
        from: Vec3,
        tgt: Vec3,
        fire_angles: Vec3,
        fire_delay: f32,
        airtime: f32,
        self_damage: f32,
    ) -> Result<u32, String> {
        let from_cell = self.nearest(from).ok_or("no cell near from")?;
        let to_cell = self.nearest(tgt).ok_or("no cell near tgt")?;
        if from_cell == to_cell {
            return Err("from and tgt resolve to the same cell".into());
        }
        let a = self.cells[from_cell as usize].origin;
        let b = self.cells[to_cell as usize].origin;
        // Price the stated flight like a certified link (exact pricing is irrelevant for a
        // puppet-flown drill leg; it just has to be finite and honest about the health cost).
        let cost = rocket_jump_cost(fire_delay, airtime, 0.0, self_damage);
        let tr = RocketJumpTraversal {
            fire_angles,
            fire_delay,
            blast: a,
            pos_blast: a,
            v0: Vec3::ZERO,
            land: b,
            airtime,
            self_damage,
        };
        self.push_rocket_jump(
            Link { from: from_cell, to: to_cell, kind: LinkKind::RocketJump, cost },
            tr,
        );
        Ok((self.links.len() - 1) as u32)
    }

    /// Append a free-standing cell (not from the column carve) and index it. Used for plat
    /// surfaces, which don't exist in the static world hull.
    fn add_cell(&mut self, origin: Vec3) -> CellId {
        let id = self.cells.len() as CellId;
        let (gx, gy) = (floor_grid(origin.x), floor_grid(origin.y));
        self.cells.push(Cell { origin, gx, gy });
        self.adjacency.push(Vec::new());
        self.grid.entry((gx, gy)).or_default().push(id);
        id
    }

    /// Cells whose XY is within `radius` of `xy`.
    fn cells_near(&self, xy: Vec2, radius: f32) -> Vec<CellId> {
        let (gx, gy) = (floor_grid(xy.x), floor_grid(xy.y));
        let r = (radius / GRID).ceil() as i32;
        self.neighbors_within(gx, gy, r)
            .into_iter()
            .filter(|&c| (self.cells[c as usize].origin.xy() - xy).length() <= radius)
            .collect()
    }

    /// The nearest cell an item at `p` is actually **collectable from**: within a wide XY reach but only
    /// `48` vertically — mirroring the game's `on_item` pickup Z gate — so a thin ledge or pedestal item
    /// resolves to a cell *on* its shelf, never a floor cell a storey below it. Item goals resolve
    /// through this (the caller falls back to [`nearest`](Self::nearest) if nothing is close enough), so
    /// a bot never parks under an item it can't reach waiting for it — and the true route (a rocket jump
    /// up, say) gets its real pricing.
    pub fn nearest_collectable(&self, p: Vec3) -> Option<CellId> {
        self.nearest_within(p, GRID * 5.0, 48.0)
    }

    /// Nearest cell to `p` within `horiz` XY and `vert` Z of it, by 3D distance.
    fn nearest_within(&self, p: Vec3, horiz: f32, vert: f32) -> Option<CellId> {
        let (gx, gy) = (floor_grid(p.x), floor_grid(p.y));
        let r = (horiz / GRID).ceil() as i32;
        self.neighbors_within(gx, gy, r)
            .into_iter()
            .filter(|&c| {
                let o = self.cells[c as usize].origin;
                (o.xy() - p.xy()).length() <= horiz && (o.z - p.z).abs() <= vert
            })
            .min_by(|&a, &b| {
                let d = |c: CellId| (self.cells[c as usize].origin - p).length_squared();
                d(a).total_cmp(&d(b))
            })
    }
}

/// Live physics the hook-arc solver needs, gathered from cvars at build time (they can differ per
/// map: `sv_gravity` is 100 on e1m8, and `rtx_hook_pull`/`rtx_hook_speed` are tunable). Reeling
/// speed and gravity fix the parabola; the throw speed only prices the flight leg.
#[derive(Clone, Copy)]
pub struct HookParams {
    pub gravity: f32,
    pub pull: f32,  // HOOK_PULL_BASE × rtx_hook_pull
    pub throw: f32, // HOOK_THROW_BASE × rtx_hook_speed
}

/// Live physics the speed-jump solver needs: gravity (jump airtime) and the bhop acceleration
/// (`sv_accelerate`/`sv_maxspeed`) that converts a runway length into attainable speed.
#[derive(Clone, Copy)]
pub struct SpeedJumpParams {
    pub gravity: f32,
    pub accel: f32,
    pub maxspeed: f32,
    /// `sv_friction` / `sv_stopspeed` — needed by the curl certifier's ground-prestrafe rollout.
    pub friction: f32,
    pub stopspeed: f32,
    /// Generate curl jumps (run-up + air-turn onto an offset landing), certified by a pmove rollout.
    pub curl: bool,
}

/// A solved hook traversal, stored per hook link in a side table (parallel to `links`, like
/// `gated_links`). Carries what the bot can't cheaply re-derive: the anchor it should aim at, and
/// the distance-to-anchor at which to let go so the ensuing parabola lands on the target.
#[derive(Clone, Copy)]
pub struct HookTraversal {
    /// Where the hook is expected to stick (also the throw aim point).
    pub stick: Vec3,
    /// Distance from the anchor at which to release the reel (`0` = pull all the way in / drop).
    pub release_dist: f32,
    /// Release velocity used to solve the arc — the reel direction and speed at let-go. Read by the
    /// build/test to re-fly the stored arc (the runtime reels toward the live anchor instead).
    #[allow(dead_code)]
    pub v0: Vec3,
    /// Simulated airtime of the parabola — the runtime's Ballistic-phase watchdog base.
    pub airtime: f32,
}

/// Live physics the rocket-jump solver needs, gathered from cvars at build time: gravity (fixes both
/// the jump ascent and the post-blast parabola) and the `rj` self-boost cvar (off by default; when a
/// server sets it > 1, a self-rocket adds an extra `dir·points·rj` impulse — see `t_damage`).
#[derive(Clone, Copy)]
pub struct RocketJumpParams {
    pub gravity: f32,
    pub rj_extra: f32,
}

/// A solved rocket jump, stored per rocket-jump link in a side table (parallel to `links`, like
/// `hooks`). Carries the two ingredients the bot fires the shot by — the delay from the jump press
/// and the view angles — plus the self-damage (the runtime health gate) and the arc data.
///
/// The runtime driver (`crate::bot::rj`) reads `fire_angles`/`fire_delay`/`airtime`/`self_damage`;
/// `blast`/`pos_blast`/`v0` are build/test-only re-flight data.
#[derive(Clone, Copy)]
pub struct RocketJumpTraversal {
    /// View angles to fire at (QW pitch positive-down); the shot goes straight along `v_forward`.
    pub fire_angles: Vec3,
    /// Seconds from the jump press to the `+attack` that fires the rocket.
    pub fire_delay: f32,
    /// Where the rocket is expected to explode (telemetry / the runtime doesn't need it).
    #[allow(dead_code)]
    pub blast: Vec3,
    /// Bot position at the blast — stored so the build/test can re-fly the continuation arc.
    #[allow(dead_code)]
    pub pos_blast: Vec3,
    /// Continuation velocity just after the blast — re-flown by the build/test.
    #[allow(dead_code)]
    pub v0: Vec3,
    /// The solver's predicted landing position (where the post-blast arc touches down). Telemetry /
    /// test only — lets the harness tell a physics miss (runtime ≠ this) from an acceptance snap
    /// (this ≠ the target cell).
    #[allow(dead_code)]
    pub land: Vec3,
    /// Simulated airtime of the parabola after the blast — the runtime's Ballistic watchdog base.
    pub airtime: f32,
    /// Pre-armor self-damage points from the blast — the runtime's health gate.
    pub self_damage: f32,
}

#[derive(Default)]
pub struct LinkCounts {
    pub walk: u32,
    pub step: u32,
    pub drop: u32,
    pub jump: u32,
    pub double_jump: u32,
    pub speed_jump: u32,
    pub plat: u32,
    pub teleport: u32,
    pub hook: u32,
    pub rocket_jump: u32,
}

/// Radius out from a cell probed for an adjacent lava/slime surface — a stride (1.5 grid columns)
/// past the cell, so a bot skirting the edge is caught without flagging cells a safe walkway away.
const HAZARD_PROBE_R: f32 = 48.0;
/// Health charged to a link *entering* a cell on a lava/slime edge — a risk premium, not certain
/// damage: the bot means to walk past the pool, and only sometimes clips it. ~15% odds of a stumble
/// costing a ~20HP dip. Deliberately not stacked on top of the in-pool price below: a cell you're
/// already burning on doesn't also charge for being *near* the burn.
const HAZARD_EDGE_HP: f32 = 3.0;

/// The engine's liquid contact damage, which [`NavGraph::flag_hazards`] prices links against — the
/// exact numbers `apply_liquid_damage` deals: lava burns `10 · waterlevel` every 0.2s, slime
/// `4 · waterlevel` every 1.0s. Kept as the raw damage model rather than pre-converted seconds
/// because what a point of health is *worth* is a per-bot question (see [`hazard_cost`]); this is
/// just physics.
const LAVA_TICK_HP: f32 = 10.0;
const LAVA_TICK_SECS: f32 = 0.2;
const SLIME_TICK_HP: f32 = 4.0;
const SLIME_TICK_SECS: f32 = 1.0;

/// Health charged to a jump link whose fall-short zone is a lava/slime pool (see [`flag_hazards`]).
/// A jump/speed-jump over lava lands on a *safe* platform — so the per-cell pricing reads it free —
/// but an undershoot (the bot arriving a hair slow at the takeoff) drops into the pool, and that is a
/// near-certain kill. Priced past a bare bot's health so [`hazard_cost`] treats it as fatal (~the cost
/// cap): the router takes any walk-around, and only a sole-route lava jump is still attempted.
const JUMP_LAVA_HP: f32 = 200.0;

/// Seconds of detour a bot accepts per unit of "fraction of its surviving strength" a hazard eats
/// (see [`hazard_cost`]). Calibrated so a bare 100-health bot prices a waterlevel-1 lava cell at
/// ~1.7s — within a hair of the hand-tuned 2.0s this replaced. The median bot therefore behaves as
/// it always did; only the tails move, which is the whole point of the change.
pub const HAZARD_TIME_K: f32 = 15.0;
/// Floor on [`hazard_cost`]'s divisor: the least strength a bot is credited with having left after a
/// crossing. Small on purpose — the price *should* run away as the damage closes on the health it
/// would take, because that is the difference between a wound and a death, and a bot one tick from
/// dying has no business weighing a shortcut at all. It exists only to keep the division finite once
/// the crossing is fatal (strength ≤ hp), where the true answer is "never".
const HAZARD_STRENGTH_FLOOR: f32 = 1.0;
/// Sanity bound on a single hazard link's price, not a policy knob: the runaway above already reaches
/// ~15× the health at stake, far past any detour a Quake map can offer, so this only stops a
/// pathologically long in-pool link from swamping A*'s arithmetic. Finite, and orders below
/// [`CLOSED_GATE_PENALTY`], so a lava-only route is still taken — a bot with no other way through
/// wades rather than freezing.
const HAZARD_COST_MAX: f32 = 600.0;

/// What crossing a hazard link costs the bot in `price`, given the `hp` that link is expected to
/// burn.
///
/// The price is the damage as a **fraction of the strength you'd have left**, not a flat rate per
/// point: a fixed reserve would charge every hazard the same multiple of its size, so a badly hurt
/// bot would refuse a 4HP slime film (a scratch) as hard as a 20HP lava wade (most of its life). The
/// ratio separates them — which is the behaviour asked for: at full strength, with armor, clipping a
/// corner is nearly free; at 30 health it is not worth much of a shortcut.
///
/// A gradient the whole way down, and the shape matters at the bottom: the divisor is the strength
/// the crossing *leaves* you, so as the damage closes on the health it would take, the price runs
/// away on its own — no threshold to tune, and none to be wrong. Once the wade is fatal
/// (`strength ≤ hp`) it prices at ~15× the health at stake, hundreds of seconds, which no detour on
/// any real map beats: refusal, arrived at by arithmetic rather than declared.
///
/// | strength (10HP lava cell) | 300 | 100 | 50 | 30 | 20 | 15 | 12 | ≤10 (fatal) |
/// |---|---|---|---|---|---|---|---|---|
/// | seconds of detour accepted | 0.5 | 1.7 | 3.8 | 7.5 | 15 | 30 | 75 | 150 |
pub fn hazard_cost(hp: f32, price: HazardPrice) -> f32 {
    (price.k * hp / (price.strength - hp).max(HAZARD_STRENGTH_FLOOR)).min(HAZARD_COST_MAX)
}

/// Extra travel-time on every non-plat link *entering* a cell under a `func_plat`'s swept volume (see
/// [`NavGraph::surcharge_under_plat_links`]). Standing in a lift's shaft blocks its descent and resets
/// its lower-timer, so the shaft is a corridor to cross, not one to choose. Moderate by design: a shaft
/// spans 2–4 cells, so cutting through costs ~1.5–3s and any parallel corridor within that detour wins,
/// while an item under the lift stays worth a couple of seconds against its desirability and is still
/// fetched. Finite, like every other extra here, so a shaft that is the only way through is still taken.
/// This only *biases* routing — the runtime linger latch (the game's `bot::steer`) is what guarantees a
/// bot never parks there.
const UNDER_PLAT_EXTRA: f32 = 0.75;

/// Multiplier on the cost of every link *entering* an underwater cell — the ratio of swimming to
/// running the same distance, since pmove drives a submerged bot at [`SWIM_SPEED`] (0.7× wishspeed).
/// [`NavGraph::flag_water`] charges the difference in `water_extra`, so a crossing stays proportional
/// to its length and a bot swims exactly when the water really is the shorter way.
///
/// This *is* the whole cost of water. It was 2.0 — the honest 1.43 plus a premium for "the exposure
/// and the lost bunnyhop" — and neither half survives contact:
///
///  * the **lost bunnyhop** is now modelled where it belongs, in `banded_step`'s band (a leg into
///    water exits at swim speed), so charging it here as well would price it twice;
///  * there is no **exposure** to charge. Water isn't lava: it does no damage, and there is no wet
///    state to carry out of the pool. Being submerged costs you speed, and it costs you air —
///    and air is `breathable`'s job, not the route's.
const WATER_COST_MULT: f32 = MAX_SPEED / SWIM_SPEED;

/// Travel-time cost of a link: horizontal distance / speed, plus risk/effort penalties so A*
/// prefers grounded routes and avoids damaging falls.
fn link_cost(kind: LinkKind, horiz: f32, dz: f32) -> f32 {
    let base = horiz.max(GRID) / MAX_SPEED;
    // A landing below the takeoff adds its real ballistic fall time (nominal gravity), plus a
    // beat past SAFE_FALL for the hard-landing tax (the flat 5 HP + the recovery stumble) — so
    // a 2000u plunge prices its ~2.2s honestly instead of looking free.
    let fall = if -dz > SAFE_FALL {
        (2.0 * -dz / 800.0).sqrt() + 0.4
    } else {
        0.0
    };
    match kind {
        LinkKind::Walk => base,
        LinkKind::Step => base * 1.1,
        LinkKind::Drop => base + 0.1 + fall,
        LinkKind::JumpGap => base + 0.3 + fall,
        // A double jump is a touch pricier — a harder maneuver (two timed jumps) than a single hop.
        LinkKind::DoubleJump => base + 0.6,
        // Speed-jump costs (runway run-up + flight + commitment) are computed at splice time.
        LinkKind::SpeedJump => base + 2.0,
        // Plat costs are computed at splice time (ride time + overhead), not here.
        LinkKind::Plat => base + 1.0,
        // Teleporting is near-instant; cost is set at splice time.
        LinkKind::Teleport => 0.2,
        // Hook costs (throw + reel + parabola airtime + overhead) are computed at splice time from
        // the solved trajectory; this fallback should never actually be priced.
        LinkKind::Hook => base + HOOK_OVERHEAD,
        // Rocket-jump costs (rise-to-blast + arc airtime + overhead + health surcharge) are computed
        // at splice time; this fallback should never actually be priced.
        LinkKind::RocketJump => base + 4.0,
    }
}

/// Per-map navigation state, reset each map load. Lives on `GameState`.
#[derive(Default)]
pub struct NavState {
    /// The parsed BSP the navmesh is derived from, shared (`Arc`) with the background build worker
    /// and every `pointcontents`/trace query. `None` until a map's BSP has been read and parsed
    /// (`GameState::load_map_bsp`, at entity load); populated even on a bot-free server so world
    /// queries (sky/liquid tests, world traces) have geometry to read.
    pub bsp: Option<Arc<Bsp>>,
    /// The built navigation graph. `None` until [`NavGraph::build`] runs (bots stay disabled).
    pub graph: Option<NavGraph>,
    /// Whether a build has been kicked off for this map (so a failed BSP read doesn't retry every
    /// frame). Reset when a new map loads.
    pub attempted: bool,
    /// A background build in flight: the channel the worker thread delivers its finished graph on.
    /// The main thread polls it each frame and swaps the result into `graph` when ready (`None` when
    /// no build is running). Dropping it (on map change) discards a stale build.
    pub pending: Option<std::sync::mpsc::Receiver<NavGraph>>,
    /// Static catalog of item-goal pickups: `(entity index, nearest cell)`. Built once with the
    /// graph; items don't move, so their cell is fixed. Live availability and desire are read
    /// fresh at selection time (by the game's `bot::goals`).
    pub goals: Vec<(u32, CellId)>,
}

/// Build a navmesh off the main thread from the parsed BSP plus the entity-derived
/// plat/teleport/gate info. Pure — no engine or game-state access — so it runs safely on a worker
/// thread whose finished graph the main thread swaps in when ready. The BSP is parsed once at map
/// load (see `GameState::load_map_bsp`) and shared here by reference.
#[allow(clippy::too_many_arguments)] // the per-map build knobs; a params struct would just relocate them
pub fn build_navmesh(
    bsp: &Bsp,
    plats: Vec<PlatInfo>,
    teleports: Vec<TeleportInfo>,
    gates: Vec<GateInfo>,
    hooks: Option<HookParams>,
    double_jump: bool,
    speed_jump: Option<SpeedJumpParams>,
    rocket_jump: Option<RocketJumpParams>,
) -> NavGraph {
    let run = || -> NavGraph {
        let mut graph = NavGraph::build(bsp);
        // Static-geometry jump/hook splices first (before the plat/gate splices): keeps plat surfaces
        // off their endpoints and lets `add_gates` tag any of these links that cross a door.
        if double_jump {
            graph.add_double_jumps(bsp);
        }
        // Speed jumps after double jumps, so they only fill the gaps double jumps can't (they see the DJ
        // links via `has_direct_link`).
        if let Some(params) = speed_jump {
            graph.add_speed_jumps(bsp, params, double_jump);
        }
        // Hooks first: they derive from the static hull, and going before the plat/gate splices keeps
        // plat surfaces off hook endpoints and lets `add_gates` tag any hook link crossing a door.
        if let Some(params) = hooks {
            graph.add_hooks(bsp, params);
        }
        // Rocket jumps after hooks: `has_direct_link` then skips any ledge a (free, cheaper) hook already
        // reaches, so an RJ link is only spent where nothing else gets there.
        if let Some(params) = rocket_jump {
            graph.add_rocket_jumps(bsp, params, double_jump);
        }
        graph.add_plats(bsp, &plats);
        graph.add_teleports(bsp, &teleports);
        graph.add_gates(&gates);
        // Last: prices links entering a lift shaft, so it must see every link the splices above added
        // (a teleport that lands under a plat, a jump-aboard from the shaft floor).
        graph.surcharge_under_plat_links();
        // Liquid flags run here, on the worker: with the whole BSP in hand we read the render hull's
        // contents directly (`bsp.pointcontents`), so there's no longer a main-thread graph-swap pass.
        // They must precede `build_reachability`/`build_lod` so the LOD tables price water/hazard at
        // birth (the tables read `link.cost + water_extra` and carry `hazard_hp` — see
        // `build_lod_tables`/`intra_reach`; that's why there is no post-swap `patch_lod_liquids`).
        graph.flag_hazards(&|p| bsp.is_solid(p), &|p| bsp.pointcontents(p));
        graph.flag_water(&|p| bsp.pointcontents(p));
        // Now that every link is in place and priced, precompute static reachability and the LOD hierarchy.
        graph.build_reachability();
        graph.build_lod();
        graph
    };
    // Run the (rayon-parallel) build on a transient pool sized to leave one core for the caller.
    // A transient pool rather than rayon's process-global one because this crate ships inside a
    // native game module (rtx.dll) the engine can unload: global-pool worker threads would outlive
    // the DLL and later run freed code. This pool's threads are joined when the call returns.
    // If parallelism can't be queried or the pool fails to build, fall back to running inline (which
    // uses rayon's global pool for the par_iters — fine in tests and the standalone viewer).
    let threads = std::thread::available_parallelism()
        .map(|n| n.get().saturating_sub(1).max(1))
        .unwrap_or(1);
    match rayon::ThreadPoolBuilder::new().num_threads(threads).build() {
        Ok(pool) => pool.install(run),
        Err(_) => run(),
    }
}

impl NavState {
    /// Whether a usable navmesh exists for the current map.
    pub fn is_loaded(&self) -> bool {
        self.graph.as_ref().is_some_and(|g| !g.cells.is_empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `ledge_beside` flags a cell only when the floor genuinely falls away past a survivable step
    /// within a stride — an open pit, not flat ground, a wall, or a small step-down.
    #[test]
    fn ledge_beside_flags_a_pit_edge_not_flat_ground_or_a_step() {
        // Flat floor everywhere (solid below z=0): nowhere is a ledge.
        let flat = |p: Vec3| p.z < 0.0;
        assert!(!ledge_beside(&flat, Vec3::ZERO), "flat ground must not flag");

        // A walkway that ends at x=20 with open air (a deep pit) beyond it.
        let cliff = |p: Vec3| p.x < 20.0 && p.z < 0.0;
        // A cell on the lip (its +x probe at x=44 lands in the pit) is a ledge.
        assert!(ledge_beside(&cliff, Vec3::ZERO), "cell beside the pit must flag");
        // A cell well back from the lip (every probe still over floor) is not.
        assert!(!ledge_beside(&cliff, Vec3::new(-100.0, 0.0, 0.0)), "cell away from the pit must not flag");

        // A mere step-down (a catch-floor within FATAL_DROP past the lip) is survivable, not a ledge:
        // floor at z=0 for x<20, a lower floor at z=-40 beyond it.
        let step_down = |p: Vec3| if p.x < 20.0 { p.z < 0.0 } else { p.z < -40.0 };
        assert!(!ledge_beside(&step_down, Vec3::ZERO), "a survivable step-down must not flag");

        // But a drop past FATAL_DROP with no catch-floor within reach is a ledge (catch-floor at -100).
        let deep = |p: Vec3| if p.x < 20.0 { p.z < 0.0 } else { p.z < -100.0 };
        assert!(ledge_beside(&deep, Vec3::ZERO), "a drop past the fatal threshold must flag");
    }

    /// The parabola integrator matches the closed-form ballistic solution over a flat floor.
    #[test]
    fn hook_arc_matches_closed_form() {
        // Floor at z = 0 (solid at or below), open above.
        let floor = |p: Vec3| p.z <= 0.0;
        let r = Vec3::new(0.0, 0.0, 100.0);
        let v0 = Vec3::new(200.0, 0.0, 300.0);
        let g = 800.0;
        // Closed form: 100 + 300t - 400t^2 = 0 -> t = 1.0; x = 200.
        match simulate_arc(floor, r, v0, g) {
            ArcResult::Land { pos, airtime, vz } => {
                assert!((pos.x - 200.0).abs() < 20.0, "landing x {} != ~200", pos.x);
                assert!(pos.z.abs() < HOOK_SAMPLE, "landing z {} not near floor", pos.z);
                assert!((airtime - 1.0).abs() < 0.1, "airtime {airtime} != ~1.0");
                assert!(vz < 0.0, "must be descending at landing");
            }
            _ => panic!("arc did not land on the floor"),
        }
        // A ceiling just above the release point blocks the (ascending) arc.
        let boxed = |p: Vec3| p.z <= 0.0 || p.z >= 110.0;
        assert!(
            matches!(simulate_arc(boxed, r, v0, g), ArcResult::Blocked),
            "arc into a ceiling should be Blocked"
        );
    }

    /// `ground_along` keeps a link whose whole span has floor under it — flat floor, straight or
    /// grid-diagonal. The oracle is hull-1 solidity: solid at or below the resting origin z (24 here).
    #[test]
    fn ground_along_keeps_continuous_floor() {
        let floor = |p: Vec3| p.z <= 24.0;
        assert!(ground_along(&floor, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 0.0, 24.0)));
        assert!(ground_along(&floor, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 32.0, 24.0)));
    }

    /// A grid-diagonal link whose centre line crosses a hole in the floor (wider than the player box,
    /// so the hull-1 oracle reads air there) is severed — the stair-side L-corner fall.
    #[test]
    fn ground_along_severs_diagonal_over_hole() {
        // Flat floor except a 16u square hole straddling the diagonal midpoint; endpoints stay solid.
        let holed = |p: Vec3| p.z <= 24.0 && !((8.0..24.0).contains(&p.x) && (8.0..24.0).contains(&p.y));
        assert!(!ground_along(&holed, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 32.0, 24.0)));
    }

    /// Balancing along a thin walkable strip (a wall-top) survives: a link running *along* a 32u-wide
    /// crest keeps floor under every sample. The hull-1 oracle reports solid across a strip at least
    /// as wide as the player box — exactly what carried the cells there in the first place.
    #[test]
    fn ground_along_keeps_thin_strip() {
        let strip = |p: Vec3| p.z <= 24.0 && p.y.abs() <= 16.0;
        assert!(ground_along(&strip, Vec3::new(0.0, 0.0, 24.0), Vec3::new(32.0, 0.0, 24.0)));
        assert!(ground_along(&strip, Vec3::new(0.0, 0.0, 24.0), Vec3::new(64.0, 0.0, 24.0)));
    }

    /// Build the navmesh from a real map (`RTX_TEST_BSP`) and sanity-check it: cells and links
    /// exist, and a healthy majority of cells land in one connected component (a fragmented
    /// graph means missing jump/step links). Reports per-kind counts. Skipped without the env.
    #[test]
    fn builds_navmesh() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let mut g = NavGraph::build(&bsp);
        assert!(!g.cells.is_empty(), "no cells carved");
        assert!(!g.links.is_empty(), "no links built");

        // DM3 regression: the two adjacent +40u hop-ups at the lower-RA lip are geometrically
        // reachable only in unusually narrow slow-speed windows. Because the same target has a
        // max-speed-safe takeoff farther back, those redundant wall-hit choices are pruned. A slow
        // hop-up that is the only route to MH must remain, as must the final jump onto RA.
        if std::path::Path::new(&path).file_stem().is_some_and(|s| s.eq_ignore_ascii_case("dm3")) {
            let direct_jump = |from: Vec3, to: Vec3| {
                let Some(a) = g.nearest(from) else { return false };
                let Some(b) = g.nearest(to) else { return false };
                (g.cell_origin(a) - from).length() < 0.1
                    && (g.cell_origin(b) - to).length() < 0.1
                    && g.adjacency[a as usize]
                        .iter()
                        .any(|&li| g.link_target(li) == b && g.link_kind(li) == LinkKind::JumpGap)
            };
            assert!(
                !direct_jump(Vec3::new(192.0, -800.0, -16.0), Vec3::new(224.0, -832.0, 24.0)),
                "unsafe diagonal lower-RA hop-up survived hot-entry certification"
            );
            assert!(
                !direct_jump(Vec3::new(224.0, -800.0, -16.0), Vec3::new(224.0, -832.0, 24.0)),
                "unsafe straight lower-RA hop-up survived hot-entry certification"
            );
            assert!(
                direct_jump(Vec3::new(192.0, -704.0, -16.0), Vec3::new(224.0, -832.0, 24.0)),
                "safe lower-RA run-jump was removed"
            );
            assert!(
                direct_jump(Vec3::new(96.0, -576.0, 296.0), Vec3::new(128.0, -672.0, 328.0)),
                "final jump onto RA was removed"
            );
            assert!(
                direct_jump(Vec3::new(-512.0, 288.0, 144.0), Vec3::new(-576.0, 256.0, 176.0)),
                "non-redundant slow MH ascent was removed"
            );
        }

        // Largest connected component via union-find over (undirected) links.
        let mut parent: Vec<u32> = (0..g.cells.len() as u32).collect();
        fn find(p: &mut [u32], mut x: u32) -> u32 {
            while p[x as usize] != x {
                p[x as usize] = p[p[x as usize] as usize];
                x = p[x as usize];
            }
            x
        }
        for l in &g.links {
            let (a, b) = (find(&mut parent, l.from), find(&mut parent, l.to));
            parent[a as usize] = b;
        }
        let mut sizes: HashMap<u32, u32> = HashMap::new();
        for i in 0..g.cells.len() as u32 {
            let r = find(&mut parent, i);
            *sizes.entry(r).or_default() += 1;
        }
        let largest = sizes.values().copied().max().unwrap_or(0);
        let frac = largest as f32 / g.cells.len() as f32;
        let c = g.summary();
        eprintln!(
            "{path}: {} cells, {} links (walk {} step {} drop {} jump {}); \
             largest component {largest}/{} = {:.0}%",
            g.cells.len(),
            g.links.len(),
            c.walk,
            c.step,
            c.drop,
            c.jump,
            g.cells.len(),
            frac * 100.0,
        );
        assert!(frac > 0.5, "navmesh too fragmented: {:.0}%", frac * 100.0);

        // Links are directed (drops/jumps are one-way), so directed reachability depends on the
        // start. Sample several starts and take the best — that models a bot spawned at a
        // player-start, which should be able to roam most of the map.
        let directed_reach = |start: CellId| -> Vec<CellId> {
            let mut seen = vec![false; g.cells.len()];
            let mut stack = vec![start];
            let mut order = vec![start];
            seen[start as usize] = true;
            while let Some(c) = stack.pop() {
                for &li in &g.adjacency[c as usize] {
                    let to = g.links[li as usize].to;
                    if !seen[to as usize] {
                        seen[to as usize] = true;
                        stack.push(to);
                        order.push(to);
                    }
                }
            }
            order
        };
        let step = (g.cells.len() / 32).max(1);
        let best = (0..g.cells.len() as u32)
            .step_by(step)
            .map(directed_reach)
            .max_by_key(|o| o.len())
            .unwrap();
        let (start, reached) = (best[0], best.len());
        let reach_frac = reached as f32 / g.cells.len() as f32;
        eprintln!(
            "best directed reach: {reached}/{} = {:.0}%",
            g.cells.len(),
            reach_frac * 100.0
        );

        // Assert A* returns a valid chain to the farthest reachable cell.
        let goal = *best.last().unwrap();
        let route = g
            .find_path(start, goal, &LinkCosts::default())
            .expect("A* found no route to a reachable cell");
        let mut cell = start;
        for &li in &route {
            assert_eq!(g.links[li as usize].from, cell, "route discontinuity");
            cell = g.links[li as usize].to;
        }
        assert_eq!(cell, goal, "route did not reach goal");
        eprintln!("A*: route to {goal} is {} links", route.len());
        assert!(
            reach_frac > 0.4,
            "best directed reach too low: {:.0}%",
            reach_frac * 100.0
        );

        // Plat splice: synthesize a lift whose board sits just above a well-connected cell and
        // whose exit is another reachable cell; confirm the ride + board cell wire in. (The
        // jump-aboard count depends on local geometry/headroom, so it's reported, not asserted.)
        let (links_before, cells_before) = (g.links.len(), g.cells.len());
        let board = g.cells[start as usize].origin + Vec3::Z * 24.0;
        let exit = g.cells[goal as usize].origin;
        g.add_plats(
            &bsp,
            &[PlatInfo {
                board,
                exit,
                entity: 7,
                fp_min: board.xy() - Vec2::splat(32.0),
                fp_max: board.xy() + Vec2::splat(32.0),
                bottom: board.z - 48.0,
            }],
        );
        assert_eq!(g.summary().plat, 1, "plat ride not added");
        assert_eq!(g.cells.len(), cells_before + 1, "board cell not added");
        // The lift registered, and its ride link (the first link added) plus every jump-aboard link
        // carries the plat tag, while a pre-existing static link does not.
        assert_eq!(g.plat_count(), 1, "plat not registered");
        assert_eq!(g.plat(0).entity, 7, "plat entity id not stored");
        assert_eq!(g.plat_of_link(links_before as u32), Some(0), "ride link not tagged");
        let tagged = (links_before as u32..g.links.len() as u32)
            .filter(|&li| g.plat_of_link(li) == Some(0))
            .count();
        assert_eq!(tagged, g.links.len() - links_before, "all plat links tagged");
        assert_eq!(g.plat_of_link(0), None, "static link wrongly tagged");
        eprintln!("plat splice: {} jump-aboard links", g.links.len() - links_before - 1);

        // Teleport splice: a trigger box around a well-connected cell warping to another cell.
        // Every standable cell in the box should gain a Teleport link to the destination.
        let near = g.cells[start as usize].origin;
        let tmin = near - Vec3::new(40.0, 40.0, 8.0);
        let tmax = near + Vec3::new(40.0, 40.0, 56.0);
        g.add_teleports(&bsp, &[TeleportInfo { tmin, tmax, dest: exit }]);
        let tele = g.summary().teleport;
        assert!(tele >= 1, "no teleport links added");
        eprintln!("teleport splice: {tele} entrance links");

        // Gate splice: a closed door box over a well-connected cell, with a button nearby. Links
        // whose segment crosses the box become gated, and the button resolves to an operating cell.
        let dcell = g.cells[start as usize].origin;
        let gate = GateInfo {
            obstruction: 0,
            closed_origin: dcell,
            closed_min: dcell - Vec3::new(32.0, 32.0, 8.0),
            closed_max: dcell + Vec3::new(32.0, 32.0, 56.0),
            activator: 0,
            button: g.cells[goal as usize].origin,
            shoot: false,
        };
        g.add_gates(&[gate]);
        assert_eq!(g.gate_count(), 1, "gate not registered");
        let gated_links = (0..g.links.len() as u32)
            .filter(|&li| g.gate_of_link(li).is_some())
            .count();
        assert!(gated_links > 0, "no link tagged by the gate");
        // The state-aware A* still resolves with the gate shut (routes around, or through with the
        // penalty when there's no other way).
        let shut = LinkCosts {
            gate_closed: &[true],
            ..Default::default()
        };
        assert!(g.find_path(start, goal, &shut).is_some(), "no route with gate shut");
        eprintln!(
            "gate splice: {gated_links} gated links, button cell {}",
            g.gate(0).button_cell
        );

        // Hook splice: build a fresh graph (the gate test mutated `g`) and run the hook pass with
        // stock physics. Hooks derive from real geometry, so — like reach — we report the count
        // rather than assert a floor (flat maps legitimately have none), but every emitted link
        // must satisfy its invariants and its stored arc must re-simulate onto the target corridor.
        let params = HookParams {
            gravity: 800.0,
            pull: HOOK_PULL_BASE,
            throw: HOOK_THROW_BASE,
        };
        let mut gh = NavGraph::build(&bsp);
        let reach_before = {
            let step = (gh.cells.len() / 32).max(1);
            (0..gh.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gh, s))
                .max()
                .unwrap_or(0)
        };
        gh.add_hooks(&bsp, params);
        let hooks = gh.summary().hook;
        let mut vertical = 0;
        let mut fling = 0;
        for li in 0..gh.links.len() as u32 {
            if gh.link_kind(li) != LinkKind::Hook {
                continue;
            }
            let tr = *gh.hook_of_link(li).expect("hook link missing its traversal");
            let a = gh.cell_origin(gh.link_source(li));
            let b = gh.cell_origin(gh.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                (HOOK_MIN_RISE..=HOOK_MAX_RISE).contains(&dz) && horiz <= HOOK_RANGE_XY,
                "hook link out of range: dz={dz} horiz={horiz}"
            );
            assert!(dz > JUMP_APEX || horiz > JUMP_REACH, "hook link no better than a jump");
            // Re-fly the stored arc: release point is `release_dist` back from the stick along v0.
            let dir = tr.v0.normalize_or_zero();
            let r = tr.stick - dir * tr.release_dist;
            match simulate_arc(|p| bsp.is_solid(p), r, tr.v0, params.gravity) {
                ArcResult::Land { pos, .. } => {
                    let d = (pos.xy() - b.xy()).length();
                    assert!(d <= HOOK_LAND_XY * 2.0, "stored arc lands {d} from target (li {li})");
                }
                _ => panic!("stored hook arc no longer lands (li {li})"),
            }
            if horiz > JUMP_REACH {
                fling += 1;
            } else {
                vertical += 1;
            }
        }
        // Determinism: a second identical build yields the same hook count (the runtime re-solve
        // and the offline build must agree).
        let mut gh2 = NavGraph::build(&bsp);
        gh2.add_hooks(&bsp, params);
        assert_eq!(gh2.summary().hook, hooks, "hook build not deterministic");

        let reach_after = {
            let step = (gh.cells.len() / 32).max(1);
            (0..gh.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gh, s))
                .max()
                .unwrap_or(0)
        };
        assert!(reach_after >= reach_before, "hooks reduced reachability");
        eprintln!(
            "hook splice: {hooks} links ({vertical} vertical, {fling} fling); \
             best directed reach {reach_before} -> {reach_after}"
        );

        // Double-jump splice: every emitted link must be beyond a single jump's reach (else a
        // JumpGap already covers it) but within the double-jump envelope. Report the count.
        let mut gd = NavGraph::build(&bsp);
        gd.add_double_jumps(&bsp);
        let djumps = gd.summary().double_jump;
        for li in 0..gd.links.len() as u32 {
            if gd.link_kind(li) != LinkKind::DoubleJump {
                continue;
            }
            let a = gd.cell_origin(gd.link_source(li));
            let b = gd.cell_origin(gd.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                horiz <= DOUBLE_JUMP_REACH && (-DJ_MAX_DROP..=DOUBLE_JUMP_APEX).contains(&dz),
                "double-jump link out of envelope: dz={dz} horiz={horiz}"
            );
            assert!(
                horiz > JUMP_REACH || dz > JUMP_APEX,
                "double-jump link a single jump could make"
            );
        }
        eprintln!("double-jump splice: {djumps} links");

        // Speed-jump splice: leaps beyond a single jump, cleared by bhop speed. Every emitted link's
        // takeoff must need more than maxspeed, its from-cell must sit a real runway back from the
        // takeoff, and the required speed must be within the runway's attainable cap.
        let params = SpeedJumpParams {
            gravity: 800.0,
            accel: 10.0,
            maxspeed: MAX_SPEED,
            friction: 4.0,
            stopspeed: 100.0,
            curl: false, // this test asserts straight-speed-jump invariants
        };
        let k = bhop_k(params.accel, params.maxspeed);
        let mut gs = NavGraph::build(&bsp);
        gs.add_speed_jumps(&bsp, params, false);
        let sjumps = gs.summary().speed_jump;
        for li in 0..gs.links.len() as u32 {
            if gs.link_kind(li) != LinkKind::SpeedJump {
                continue;
            }
            let tr = *gs
                .speed_jump_of_link(li)
                .expect("speed-jump link missing its traversal");
            let start = gs.cell_origin(gs.link_source(li));
            let land = gs.cell_origin(gs.link_target(li));
            let horiz = (land.xy() - tr.takeoff.xy()).length();
            // Beyond a single jump's *reach* (else a JumpGap covers it) — a wide flat gap, or a
            // downhill one whose extra airtime a JumpGap's flat 200u cap still missed.
            assert!(horiz > JUMP_REACH, "speed jump within single-jump reach: {horiz}");
            assert!(tr.v_req <= SPEED_JUMP_V_CAP + 1.0, "v_req over the cap: {}", tr.v_req);
            // A *chained* speed jump has no self-contained runway by design — its from-cell **is**
            // the ledge (it's only traversable when the planner proves the entry band already carries
            // `v_req`), so the runway-back invariant applies only to stand-start jumps.
            if !tr.chained {
                // The from-cell is the runway start: at least the runway needed to build the *extra*
                // speed over maxspeed (a gap crossable at ≤ maxspeed needs no runway → from = ledge).
                let need = runway_len_for(tr.v_req.max(MAX_SPEED), MAX_SPEED, k);
                let back = (start.xy() - tr.takeoff.xy()).length();
                assert!(back + GRID >= need, "runway too short: {back} < {need}");
            }
        }
        eprintln!("speed-jump splice: {sjumps} links");

        // Rocket-jump splice: blast-launched leaps up to high ledges. Every emitted link must clear
        // more than a single jump's apex (else a jump covers it), sit within the RJ envelope, and its
        // stored (pos_blast, v0) arc must re-simulate onto the target — the offline solve and the
        // runtime re-flight must agree. Default-mode physics (gravity 800, no `rj` boost).
        let rjp = RocketJumpParams { gravity: 800.0, rj_extra: 0.0 };
        let mut gr = NavGraph::build(&bsp);
        let reach_before = {
            let step = (gr.cells.len() / 32).max(1);
            (0..gr.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gr, s))
                .max()
                .unwrap_or(0)
        };
        gr.add_rocket_jumps(&bsp, rjp, false);
        let rjumps = gr.summary().rocket_jump;
        for li in 0..gr.links.len() as u32 {
            if gr.link_kind(li) != LinkKind::RocketJump {
                continue;
            }
            let tr = *gr.rocket_jump_of_link(li).expect("rocket-jump link missing its traversal");
            let a = gr.cell_origin(gr.link_source(li));
            let b = gr.cell_origin(gr.link_target(li));
            let dz = b.z - a.z;
            let horiz = (b.xy() - a.xy()).length();
            assert!(
                (RJ_MIN_RISE..=RJ_MAX_RISE).contains(&dz) && horiz <= RJ_RANGE_XY,
                "rocket-jump link out of envelope: dz={dz} horiz={horiz}"
            );
            assert!(dz > JUMP_APEX, "rocket-jump link a single jump could make: dz={dz}");
            assert!(tr.self_damage > 0.0, "rocket-jump link with no self-blast");
            // Re-fly the stored continuation arc onto the target corridor.
            match simulate_arc(|p| bsp.is_solid(p), tr.pos_blast, tr.v0, rjp.gravity) {
                ArcResult::Land { pos, .. } => {
                    let d = (pos.xy() - b.xy()).length();
                    assert!(d <= RJ_LAND_XY * 2.0, "stored RJ arc lands {d} from target (li {li})");
                }
                _ => panic!("stored rocket-jump arc no longer lands (li {li})"),
            }
        }
        // Determinism.
        let mut gr2 = NavGraph::build(&bsp);
        gr2.add_rocket_jumps(&bsp, rjp, false);
        assert_eq!(gr2.summary().rocket_jump, rjumps, "rocket-jump build not deterministic");
        let reach_after = {
            let step = (gr.cells.len() / 32).max(1);
            (0..gr.cells.len() as u32)
                .step_by(step)
                .map(|s| directed_reach_len(&gr, s))
                .max()
                .unwrap_or(0)
        };
        assert!(reach_after >= reach_before, "rocket jumps reduced reachability");
        eprintln!("rocket-jump splice: {rjumps} links; best directed reach {reach_before} -> {reach_after}");
    }

    /// Two full builds of the same map must produce a byte-identical graph — the guard that the
    /// rayon-parallel solvers stay deterministic across link indices, adjacency, and every
    /// side-table payload. Env-gated on `RTX_TEST_BSP` like [`builds_navmesh`]. Prints an FNV-1a
    /// fingerprint of the graph so a change that alters build output (versus an earlier commit) is
    /// visible run to run, not just detectable as a build-vs-build mismatch.
    #[test]
    fn build_deterministic() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let hooks = Some(HookParams {
            gravity: 800.0,
            pull: HOOK_PULL_BASE,
            throw: HOOK_THROW_BASE,
        });
        let speed = Some(SpeedJumpParams {
            gravity: 800.0,
            accel: 10.0,
            maxspeed: MAX_SPEED,
            friction: 4.0,
            stopspeed: 100.0,
            curl: true, // cover the curl-generation pass's determinism
        });
        let rj = Some(RocketJumpParams { gravity: 800.0, rj_extra: 0.0 });
        // All solvers on, no entity splices (they're serial and not the subject here).
        let build = || build_navmesh(&bsp, vec![], vec![], vec![], hooks, true, speed, rj);
        let a = build();
        let b = build();

        fn mix(h: &mut u64, x: u64) {
            *h ^= x;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3); // FNV-1a 64-bit prime
        }
        fn mix_vec3(h: &mut u64, v: Vec3) {
            mix(h, v.x.to_bits() as u64);
            mix(h, v.y.to_bits() as u64);
            mix(h, v.z.to_bits() as u64);
        }
        fn fingerprint(g: &NavGraph) -> u64 {
            let mut h: u64 = 0xcbf2_9ce4_8422_2325; // FNV-1a offset basis
            mix(&mut h, g.cells.len() as u64);
            for c in &g.cells {
                mix_vec3(&mut h, c.origin);
                mix(&mut h, c.gx as u32 as u64);
                mix(&mut h, c.gy as u32 as u64);
            }
            mix(&mut h, g.links.len() as u64);
            for l in &g.links {
                mix(&mut h, l.from as u64);
                mix(&mut h, l.to as u64);
                mix(&mut h, l.kind as u64);
                mix(&mut h, l.cost.to_bits() as u64);
            }
            for adj in &g.adjacency {
                mix(&mut h, adj.len() as u64);
                for &li in adj {
                    mix(&mut h, li as u64);
                }
            }
            for &x in g.hooks.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.hooks.items_raw() {
                mix_vec3(&mut h, t.stick);
                mix(&mut h, t.release_dist.to_bits() as u64);
                mix_vec3(&mut h, t.v0);
                mix(&mut h, t.airtime.to_bits() as u64);
            }
            for &x in g.speed_jumps.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.speed_jumps.items_raw() {
                mix_vec3(&mut h, t.takeoff);
                mix(&mut h, t.v_req.to_bits() as u64);
                mix(&mut h, t.airtime.to_bits() as u64);
                mix(&mut h, t.chained as u64);
                mix(&mut h, t.curl_gain.to_bits() as u64);
                mix_vec3(&mut h, t.curl_entry_aim);
                mix(&mut h, t.curl_switch_dist.to_bits() as u64);
                mix_vec3(&mut h, t.curl_landing_aim);
            }
            for &x in g.rocket_jumps.idx_raw() {
                mix(&mut h, x as u32 as u64);
            }
            for t in g.rocket_jumps.items_raw() {
                mix_vec3(&mut h, t.fire_angles);
                mix(&mut h, t.fire_delay.to_bits() as u64);
                mix_vec3(&mut h, t.blast);
                mix_vec3(&mut h, t.pos_blast);
                mix_vec3(&mut h, t.v0);
                mix(&mut h, t.airtime.to_bits() as u64);
                mix(&mut h, t.self_damage.to_bits() as u64);
            }
            // Reachability (SCC + closure) and the LOD tables (clusters, portals, edges, reach) —
            // a nondeterministic cluster/portal build would otherwise slip past the fingerprint.
            mix(&mut h, g.reach_fingerprint());
            mix(&mut h, g.lod_fingerprint());
            h
        }

        assert_eq!(a.links.len(), b.links.len(), "link count not deterministic");
        assert_eq!(a.adjacency, b.adjacency, "adjacency not deterministic");
        let (fa, fb) = (fingerprint(&a), fingerprint(&b));
        assert_eq!(fa, fb, "navmesh build not deterministic (fingerprint mismatch)");
        let c = a.summary();
        eprintln!(
            "{path}: deterministic build fingerprint {fa:#018x} \
             ({} cells, {} links; hook {} speedjump {} doublejump {} rocketjump {})",
            a.cells.len(),
            a.links.len(),
            c.hook,
            c.speed_jump,
            c.double_jump,
            c.rocket_jump,
        );
    }

    /// The speed-jump ballistic + runway model, and its agreement with the real bhop controller.
    #[test]
    fn speed_jump_model() {
        // A jump at exactly maxspeed reaches ~JUMP_REACH: v_required(216, 0) ≈ 320.
        let t = jump_airtime(0.0, 800.0);
        assert!((t - 0.675).abs() < 0.01, "flat airtime {t}");
        assert!((v_required(MAX_SPEED * t, 0.0, 800.0) - MAX_SPEED).abs() < 1.0);
        // Rising shrinks airtime (needs more speed); dropping lengthens it.
        assert!(jump_airtime(45.0, 800.0) < t && jump_airtime(-200.0, 800.0) > t);

        // attainable_speed / runway_len_for are inverses.
        let k = bhop_k(10.0, MAX_SPEED);
        let v = attainable_speed(MAX_SPEED, 800.0, k);
        assert!(v > 450.0, "800u runway should build good speed, got {v}"); // ~480, ≈1.5× maxspeed
        assert!((runway_len_for(v, MAX_SPEED, k) - 800.0).abs() < 1.0);
        // The cross-check that this derated model is conservative vs the *actual* bhop controller
        // lives in the game crate (`bot::bhop`'s tests), where the controller sim is defined.
    }

    /// Count cells directly reachable from `start` over the (directed) graph — a small DFS helper
    /// shared by the reach-delta checks.
    fn directed_reach_len(g: &NavGraph, start: CellId) -> usize {
        let mut seen = vec![false; g.cells.len()];
        let mut stack = vec![start];
        seen[start as usize] = true;
        let mut n = 1;
        while let Some(c) = stack.pop() {
            for &li in &g.adjacency[c as usize] {
                let to = g.links[li as usize].to;
                if !seen[to as usize] {
                    seen[to as usize] = true;
                    n += 1;
                    stack.push(to);
                }
            }
        }
        n
    }

    /// A diamond: two routes from cell 0 to cell 3, one (via 1) slightly cheaper than the other (via
    /// 2). Enough to exercise per-link penalty diversion and jitter without a BSP.
    ///   links: 0=(0→1,1.0) 1=(1→3,1.0)  2=(0→2,1.1) 3=(2→3,1.1)
    fn diamond() -> NavGraph {
        let cell = |x: f32, y: f32| Cell {
            origin: Vec3::new(x, y, 0.0),
            gx: 0,
            gy: 0,
        };
        let link = |from: CellId, to: CellId, cost: f32| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost,
        };
        NavGraph::test_graph(
            vec![cell(0.0, 0.0), cell(100.0, 50.0), cell(100.0, -50.0), cell(200.0, 0.0)],
            vec![link(0, 1, 1.0), link(1, 3, 1.0), link(0, 2, 1.1), link(2, 3, 1.1)],
        )
    }

    /// The banded planner credits bhop speed gains on a flat corridor but not up a staircase: an
    /// ascending Walk/Step leg carries the entry band without climbing it (a human runs up stairs;
    /// you can't air-strafe-build up risers), while a flat leg of equal length gains a band.
    #[test]
    fn banded_step_no_gain_up_stairs() {
        let cell = |x: f32, y: f32, z: f32| Cell {
            origin: Vec3::new(x, y, z),
            gx: 0,
            gy: 0,
        };
        let step = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Step,
            cost: 1.0,
        };
        let g = NavGraph::test_graph(
            vec![cell(0.0, 0.0, 0.0), cell(1500.0, 0.0, 0.0), cell(0.0, 1500.0, 100.0)],
            vec![step(0, 1), step(0, 2)], // link 0: 1500u flat; link 1: 1500u rising 100u
        );
        let (_, flat_exit) = g.banded_step(0, 0).unwrap();
        let (_, up_exit) = g.banded_step(1, 0).unwrap();
        assert!(flat_exit >= 1, "a long flat corridor should climb a band, got {flat_exit}");
        assert_eq!(up_exit, 0, "an ascending leg must not gain a band, got {up_exit}");
    }

    /// A per-link penalty diverts A* onto the alternate route once it exceeds the route's cost
    /// margin, and the route reverts the moment the penalty is gone — the loop-free-nav core.
    #[test]
    fn penalty_diverts_then_reverts() {
        let g = diamond();
        // No penalty → the cheaper route via cell 1.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
        // A penalty smaller than the 0.2s route-cost gap doesn't flip it.
        let tiny = [(0u32, 0.05f32)];
        let costs = LinkCosts {
            penalties: &tiny,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![0, 1]);
        // A larger penalty on link 0 (0→1) diverts onto the route via cell 2.
        let big = [(0u32, 5.0f32)];
        let costs = LinkCosts {
            penalties: &big,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![2, 3]);
        // Penalty expired (absent from the slice) → back to the cheap route.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
    }

    /// The rocket-jump fitness gate surcharges *only* RocketJump links: a bot unfit to rocket-jump
    /// diverts around a cheap-branch RJ leg, and a fit bot (no surcharge) still takes it.
    #[test]
    fn rocket_jump_fitness_gate_diverts() {
        let mut g = diamond();
        g.links[0].kind = LinkKind::RocketJump; // make the cheap branch's first leg (0→1) an RJ
        // Fit bot: no surcharge → the cheap route via cell 1.
        assert_eq!(g.find_path(0, 3, &LinkCosts::default()).unwrap(), vec![0, 1]);
        // Unfit bot: every RJ link costs RJ_UNFIT_PENALTY → diverts onto the route via cell 2.
        let costs = LinkCosts {
            rocket_jump_extra: RJ_UNFIT_PENALTY,
            ..Default::default()
        };
        assert_eq!(g.find_path(0, 3, &costs).unwrap(), vec![2, 3]);
        // The surcharge hits only RJ links: a Walk-only diamond is unaffected by it.
        let g2 = diamond();
        assert_eq!(g2.find_path(0, 3, &costs).unwrap(), vec![0, 1]);
    }

    /// A finite penalty never disconnects the graph: if the only route runs through the penalized
    /// link, A* still returns it (finite cost, unlike a closed gate's near-infinite one).
    #[test]
    fn penalty_never_disconnects() {
        let g = diamond();
        let huge = [(0u32, 999.0f32), (2u32, 999.0f32)]; // penalize both first legs
        let costs = LinkCosts {
            penalties: &huge,
            ..Default::default()
        };
        assert!(g.find_path(0, 3, &costs).is_some(), "finite penalties must not sever the route");
    }

    /// Jitter is deterministic per (seed, link) and bounded to `[0, JITTER_FRAC·cost]`.
    #[test]
    fn jitter_bounded_and_deterministic() {
        let g = diamond();
        let costs = LinkCosts {
            jitter_seed: 7,
            ..Default::default()
        };
        for li in 0..g.links.len() as u32 {
            let a = g.link_extra(li, &costs);
            assert_eq!(a, g.link_extra(li, &costs), "jitter must be deterministic");
            assert!(a >= 0.0, "jitter is non-negative (keeps the heuristic admissible)");
            assert!(
                a <= JITTER_FRAC * g.links[li as usize].cost + 1e-6,
                "jitter {a} exceeds the {JITTER_FRAC} cost bound",
            );
        }
        // Zero seed disables jitter entirely.
        assert_eq!(g.link_extra(0, &LinkCosts::default()), 0.0);
    }

    /// `flag_hazards` flags a cell on a lava edge and bumps the cost of links *into* it, while
    /// leaving an interior link untouched — over synthetic solid/liquid oracles, no BSP.
    #[test]
    fn surcharge_flags_lava_edge_only() {
        // Two adjacent floor cells in a row; open lava sits past the +x cell (grid column 2 has no
        // ground, and lava lurks below the probe there). Solid floor a short step under both cells.
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut grid = GridIndex::default();
        grid.insert((0, 0), vec![0]);
        grid.insert((1, 0), vec![1]);
        let mut g = NavGraph::test_graph(vec![cell(0.0, 0), cell(32.0, 1)], vec![link(0, 1), link(1, 0)]);
        g.grid = grid;
        // Floor a short step (z ≤ −40) below the cells for x ≤ 60; lava fills x > 60 under z = 0.
        let is_solid = |p: Vec3| p.x <= 60.0 && p.z <= -40.0;
        let contents = |p: Vec3| {
            if p.x > 60.0 && p.z < 0.0 {
                crate::bsp::CONTENTS_LAVA
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_hazards(&is_solid, &contents);
        // Link 0→1 enters the lava-edge cell 1 → charged the edge premium; link 1→0 enters interior
        // cell 0 → free. Asserted through `link_extra` (what the searches actually pay), not through
        // `link.cost` — the old bake this replaced looked right there while the live planner, which
        // never reads it for a walk link, sailed bots straight into the pool.
        let costs = LinkCosts {
            hazard: Some(HazardPrice::new(100.0)),
            ..Default::default()
        };
        assert_eq!(g.hazard_hp[0], HAZARD_EDGE_HP, "into-edge link not charged");
        assert!(g.link_extra(0, &costs) > 0.0, "edge premium must reach the searches");
        assert_eq!(g.hazard_hp[1], 0.0, "interior link must be free");
        assert_eq!(g.link_extra(1, &costs), 0.0, "interior link must cost the searches nothing");
    }

    /// `flag_water` flags the submerged cell, charges links *into* it the swim premium while leaving
    /// the exit link alone, and reports the depth via `cell_in_water`/`cell_breathable`.
    #[test]
    fn flag_water_flags_cells_and_prices_into_links() {
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut g = NavGraph::test_graph(vec![cell(0.0, 0), cell(32.0, 1)], vec![link(0, 1), link(1, 0)]);
        // Deep water under and around cell 1 (x = 32), up past its eye point; cell 0 (x = 0) is dry.
        let contents = |p: Vec3| {
            if p.x > 16.0 {
                crate::bsp::CONTENTS_WATER
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_water(&contents);
        assert!(!g.cell_in_water(0) && g.cell_breathable(0), "dry cell 0");
        assert!(g.cell_in_water(1) && !g.cell_breathable(1), "deep cell 1 submerged, no air");
        // Link 0→1 enters the water cell → charged the swim premium; link 1→0 exits to dry → free.
        let costs = LinkCosts::default();
        assert_eq!(g.link_extra(1, &costs), 0.0, "exit link must stay free (the gradient toward shore)");
        // The premium is exactly what the slower stroke costs over this distance and no more: water
        // does no damage, and there is no lingering wet state — being under just means 0.7× wishspeed
        // (and air, which `breathable` above tracks). The banded planner is the one that used to see
        // none of this, and must now land on the same honest swim time as the plain search.
        let swim_secs = GRID / SWIM_SPEED;
        let (banded_step_cost, exit_band) = g.banded_step(0, 0).unwrap();
        let banded = banded_step_cost + g.link_extra(0, &costs);
        assert!((banded - swim_secs).abs() < 1e-4, "banded prices the swim at {banded}, not {swim_secs}");
        assert!(
            (g.link_extra(0, &costs) - (swim_secs - GRID / MAX_SPEED)).abs() < 1e-6,
            "the premium must be the swim/run difference, got {}",
            g.link_extra(0, &costs)
        );
        assert_eq!(exit_band, band_of(SWIM_SPEED), "you cannot bunnyhop out of a pool");
    }

    /// A shallow cell — origin submerged but the eye point above the surface — is both `in_water`
    /// (swim physics) and `breathable` (a safe spot for a drowning bot to path to for air).
    #[test]
    fn shallow_water_cell_is_breathable() {
        let mut g = diamond();
        // Water fills z < 10 only, so cell origins (z = 0) are wet but their eye points (z = 22) are dry.
        let contents = |p: Vec3| {
            if p.z < 10.0 {
                crate::bsp::CONTENTS_WATER
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_water(&contents);
        assert!(g.cell_in_water(0), "origin under the surface");
        assert!(g.cell_breathable(0), "eye above the surface — can breathe");
    }

    /// A lava pool localized on `diamond`'s cheap branch, for the routing tests below. `is_solid` is
    /// false everywhere so non-lava probes read as a *pit* — which the flagging deliberately ignores,
    /// leaving only the lava-flagged cell charged. (Empty grid ⇒ every side is an edge, so the oracle
    /// alone decides.)
    fn diamond_with_lava_on_branch_1() -> NavGraph {
        let mut g = diamond();
        let is_solid = |_: Vec3| false;
        let lava_at = |cx: f32, cy: f32| {
            move |p: Vec3| {
                if (p.x - cx).abs() < 40.0 && (p.y - cy).abs() < 40.0 && p.z < 0.0 {
                    crate::bsp::CONTENTS_LAVA
                } else {
                    crate::bsp::CONTENTS_EMPTY
                }
            }
        };
        g.flag_hazards(&is_solid, &lava_at(100.0, 50.0));
        g
    }

    /// The hazard price diverts A* off a lava route the same way a penalty does: flagging the cheap
    /// branch's middle cell flips the route onto the safe branch, and the graph is never severed.
    #[test]
    fn hazard_price_diverts_off_lava_route() {
        let g = diamond();
        let costs = |s: f32| LinkCosts {
            hazard: Some(HazardPrice::new(s)),
            ..Default::default()
        };
        // Baseline: cheaper route via cell 1.
        assert_eq!(g.find_path(0, 3, &costs(100.0)).unwrap(), vec![0, 1]);
        let g = diamond_with_lava_on_branch_1();
        // Cell 1 now sits on lava, so 0→1 costs more → A* takes the (now cheaper) route via cell 2.
        assert_eq!(g.find_path(0, 3, &costs(100.0)).unwrap(), vec![2, 3]);
        // With *both* branches lava the price is still finite, so a route survives — a bot walled in
        // by lava wades rather than freezing.
        let mut g = diamond();
        let is_solid = |_: Vec3| false;
        let lava_everywhere = |p: Vec3| {
            if p.z < 0.0 {
                crate::bsp::CONTENTS_LAVA
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_hazards(&is_solid, &lava_everywhere);
        assert!(
            g.find_path(0, 3, &costs(1.0)).is_some(),
            "even at death's door the price must never sever the graph"
        );
    }

    /// A 200u route straight through a lava pool (cell 1), and an ~820u detour around it (cell 2) —
    /// so wading is a real temptation, not a formality. Unlike [`diamond`] the branches differ in
    /// *geometry* rather than stored cost, which is what the banded planner actually reasons about: it
    /// derives a walk leg's cost from distance and speed and never reads `link.cost`.
    fn lava_shortcut() -> NavGraph {
        let cell = |x: f32, y: f32| Cell {
            origin: Vec3::new(x, y, 0.0),
            gx: 0,
            gy: 0,
        };
        let origin = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(100.0, 0.0, 0.0),
            Vec3::new(100.0, 400.0, 0.0),
            Vec3::new(200.0, 0.0, 0.0),
        ];
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: link_cost(
                LinkKind::Walk,
                (origin[to as usize].xy() - origin[from as usize].xy()).length(),
                0.0,
            ),
        };
        let mut g = NavGraph::test_graph(
            vec![cell(0.0, 0.0), cell(100.0, 0.0), cell(100.0, 400.0), cell(200.0, 0.0)],
            vec![link(0, 1), link(1, 3), link(0, 2), link(2, 3)],
        );
        let is_solid = |_: Vec3| false;
        let lava = |p: Vec3| {
            if (p.x - 100.0).abs() < 40.0 && p.y.abs() < 40.0 && p.z < 0.0 {
                crate::bsp::CONTENTS_LAVA
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_hazards(&is_solid, &lava);
        g
    }

    /// **The regression this whole change exists for.** The *banded* planner is the live one
    /// (`rtx_bot_bhop` + `rtx_bot_bandplan`, both default on), and its Walk/Step arm derives cost from
    /// speed and never reads `link.cost` — so the lava surcharge that used to be baked there was
    /// silently discarded, and every bot walked into lava priced as bare floor. Pricing now lives in
    /// `link_extra`, which all three searches honor: here lava is the *short* way and the banded
    /// planner must still walk around.
    #[test]
    fn banded_planner_diverts_off_lava_route() {
        let g = lava_shortcut();
        let costs = LinkCosts {
            hazard: Some(HazardPrice::new(100.0)),
            ..Default::default()
        };
        let route = g.find_path_banded(0, 3, 0.0, &costs).expect("a route exists");
        assert_eq!(route.links, vec![2, 3], "the banded planner must avoid the lava branch too");
    }

    /// The feature: a bot's health weights how willing it is to shortcut through a hazard. Same map,
    /// same pool — a bot with armor and full health clips it, a hurt one takes the long way round.
    #[test]
    fn health_weights_willingness_to_cross_lava() {
        let g = lava_shortcut();
        let route = |s: f32| {
            g.find_path_banded(
                0,
                3,
                0.0,
                &LinkCosts {
                    hazard: Some(HazardPrice::new(s)),
                    ..Default::default()
                },
            )
            .expect("a route exists")
            .links
        };
        assert_eq!(route(300.0), vec![0, 1], "100 health behind red armor should take the shortcut");
        assert_eq!(route(30.0), vec![2, 3], "a bot at 30 strength should walk around");
    }

    /// Race-line verification and the offline optimizer measure pure traversal time over authored
    /// lines that deliberately cross lava, and compare the estimate against a timeout — so the default
    /// (`hazard: None`) must leave routing untouched by the flagging. This is why `None` means
    /// *unpriced* rather than a neutral rate.
    #[test]
    fn default_costs_are_hazard_unpriced() {
        let plain = diamond();
        let flagged = diamond_with_lava_on_branch_1();
        let costs = LinkCosts::default();
        assert_eq!(
            plain.find_path_banded(0, 3, 0.0, &costs).unwrap().cost,
            flagged.find_path_banded(0, 3, 0.0, &costs).unwrap().cost,
            "flagging hazards must not move an unpriced query's cost by a hair"
        );
        assert_eq!(flagged.find_path(0, 3, &costs).unwrap(), vec![0, 1], "nor its route");
    }

    /// The price curve: never rises with strength, always positive, always capped, and finite for
    /// every strength a dying (or already dead) bot can present — including a hazard bigger than the
    /// bot itself, where the naive `k·hp/(S−hp)` would divide by zero or go negative.
    #[test]
    fn hazard_cost_curve_is_well_behaved() {
        let hp = 10.0;
        for s in [f32::MIN, -5.0, 0.0, hp, 25.0, 50.0, 100.0, 300.0, f32::MAX] {
            let c = hazard_cost(hp, HazardPrice::new(s));
            assert!(c.is_finite() && c > 0.0 && c <= HAZARD_COST_MAX, "strength {s} → {c}");
        }
        let steps = [0.0, 25.0, 50.0, 100.0, 200.0, 300.0];
        for w in steps.windows(2) {
            assert!(
                hazard_cost(hp, HazardPrice::new(w[0])) >= hazard_cost(hp, HazardPrice::new(w[1])),
                "a stronger bot must never fear a hazard more: {} vs {}",
                w[0],
                w[1]
            );
        }
        // A scratch and a near-lethal wade must not price alike for a hurt bot — the whole reason the
        // price is a ratio rather than a flat rate per point of health.
        let hurt = HazardPrice::new(30.0);
        assert!(
            hazard_cost(20.0, hurt) > 4.0 * hazard_cost(4.0, hurt),
            "a 20HP wade must cost a hurt bot far more than a 4HP slime film"
        );
        // The bottom of the curve is the point of it: a wade that would *kill* has to price past any
        // detour a map can offer, and get there by running away smoothly rather than by a threshold.
        // (It stays finite, so a bot walled in by lava still wades rather than freezing.)
        let fatal = hazard_cost(hp, HazardPrice::new(hp - 1.0));
        assert!(fatal > 100.0, "a fatal wade priced at only {fatal}s — a long way round would beat it");
        for w in [(12.0, 15.0), (15.0, 20.0), (20.0, 30.0)] {
            let (weaker, stronger) = (hazard_cost(hp, HazardPrice::new(w.0)), hazard_cost(hp, HazardPrice::new(w.1)));
            assert!(weaker > stronger * 1.5, "the price must climb steeply as death nears: {weaker} vs {stronger}");
        }
    }

    /// A cell whose *own footing* is lava (a bot standing on it burns) is flagged in `cell_hazard` and
    /// pays the big per-link surcharge on entry, while the exit link stays cheap — the shore gradient
    /// that pulls a bot back out, mirroring water. This is the shallow-film / interior-pool case the
    /// edge probe misses.
    #[test]
    fn flags_cell_standing_in_liquid_and_surcharges_entry() {
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut grid = GridIndex::default();
        grid.insert((0, 0), vec![0]);
        grid.insert((1, 0), vec![1]);
        let mut g = NavGraph::test_graph(vec![cell(0.0, 0), cell(32.0, 1)], vec![link(0, 1), link(1, 0)]);
        g.grid = grid;
        // Floor well below the cells; a tight lava pool sits on it directly under cell 1 (x = 32), so
        // cell 1's feet+1 sample (origin − 23) reads lava while cell 0 (x = 0) keeps dry footing.
        let is_solid = |p: Vec3| p.z <= -40.0;
        let contents = |p: Vec3| {
            if (p.x - 32.0).abs() < 8.0 && p.y.abs() < 8.0 && (-40.0..0.0).contains(&p.z) {
                crate::bsp::CONTENTS_LAVA
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_hazards(&is_solid, &contents);
        assert_eq!(g.cell_hazard(1), Some(crate::hazard::HazardKind::Lava), "cell 1 stands in lava");
        assert_eq!(g.cell_hazard(0), None, "cell 0 is dry footing");
        // Stepping into the pool costs a whole 10HP tick — `dmgtime` starts in the past, so the burn
        // lands before any dwell accrues — at waterlevel 1, the film being ankle-deep. Leaving is free:
        // that asymmetry is the gradient back to dry ground.
        assert_eq!(g.hazard_hp[0], LAVA_TICK_HP, "into-lava link must cost the entry tick");
        assert_eq!(g.hazard_hp[1], 0.0, "exit link must stay free (the gradient back to dry ground)");
        let hurt = LinkCosts {
            hazard: Some(HazardPrice::new(30.0)),
            ..Default::default()
        };
        assert!(g.link_extra(0, &hurt) > g.link_extra(0, &LinkCosts {
            hazard: Some(HazardPrice::new(300.0)),
            ..Default::default()
        }), "a hurt bot must fear the same pool more than a fit one");
        assert!(g.find_path(0, 1, &hurt).is_some(), "the finite price never severs");
    }

    /// The interior cell of a shallow film — every compass neighbour walkable, so the edge probe never
    /// fires, and here `is_solid` reads solid right at the probe height, blinding it further — is still
    /// caught by the per-cell own-footing check, which reads `contents` at feet+1 directly. A slime
    /// film prices entry by [`SLIME_CELL_EXTRA`].
    #[test]
    fn flags_interior_pool_cell_the_edge_probe_cannot_see() {
        let cell = |x: f32, gx: i32| Cell {
            origin: Vec3::new(x, 0.0, 0.0),
            gx,
            gy: 0,
        };
        let link = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        let mut grid = GridIndex::default();
        grid.insert((0, 0), vec![0]);
        grid.insert((1, 0), vec![1]);
        let mut g = NavGraph::test_graph(vec![cell(0.0, 0), cell(32.0, 1)], vec![link(0, 1), link(1, 0)]);
        g.grid = grid;
        // Solid right at the probe height everywhere blinds the edge probe (it reads a wall a stride
        // out, never the liquid below); a thin slime film at the feet+1 sample depth covers both cells.
        let is_solid = |p: Vec3| p.z <= 0.0;
        let contents = |p: Vec3| {
            if (-24.0..-20.0).contains(&p.z) {
                crate::bsp::CONTENTS_SLIME
            } else {
                crate::bsp::CONTENTS_EMPTY
            }
        };
        g.flag_hazards(&is_solid, &contents);
        assert_eq!(g.cell_hazard(0), Some(crate::hazard::HazardKind::Slime), "interior cell 0 in slime");
        assert_eq!(g.cell_hazard(1), Some(crate::hazard::HazardKind::Slime), "interior cell 1 in slime");
        // Both cells already stand in the film, so neither link pays the entry tick — only the drip
        // for the ~0.1s a grid pitch takes. Slime deals 4 HP a *second* against lava's 10 every fifth
        // of one, so wading it really is nearly free, and the price now says so instead of guessing.
        let drip = SLIME_TICK_HP * ((GRID / MAX_SPEED) / SLIME_TICK_SECS);
        assert!(
            (g.hazard_hp[0] - drip).abs() < 1e-4,
            "interior slime link should cost the drip {drip}, got {}",
            g.hazard_hp[0]
        );
    }

    // --- speed-band planning (Phase B) ---

    /// Build a synthetic graph for banded-planner tests: cells at the given origins, directed links
    /// `(from, to, kind, cost)`, and speed-jump side entries `(link index, v_req, airtime, chained)`.
    fn banded_graph(
        origins: &[Vec3],
        links: &[(CellId, CellId, LinkKind, f32)],
        sjs: &[(usize, f32, f32, bool, f32, f32)],
    ) -> NavGraph {
        let cells = origins
            .iter()
            .map(|&o| Cell { origin: o, gx: 0, gy: 0 })
            .collect::<Vec<_>>();
        let mut adjacency = vec![Vec::new(); cells.len()];
        let links = links
            .iter()
            .enumerate()
            .map(|(li, &(from, to, kind, cost))| {
                adjacency[from as usize].push(li as u32);
                Link { from, to, kind, cost }
            })
            .collect::<Vec<_>>();
        let mut speed_jumps = SideTable::default();
        for &(li, v_req, airtime, chained, curl_gain, landing_speed_lo) in sjs {
            let s = speed_jumps.push(SpeedJumpTraversal {
                takeoff: origins[links[li].from as usize],
                v_req,
                airtime,
                landing_speed_lo,
                chained,
                curl_gain,
                curl_entry_aim: Vec3::ZERO,
                curl_switch_dist: 0.0,
                curl_landing_aim: Vec3::ZERO,
                ground_turn: None,
            });
            speed_jumps.tag(li, s);
        }
        let mut g = NavGraph::test_graph(cells, links);
        g.adjacency = adjacency;
        g.speed_jumps = speed_jumps;
        g
    }

    /// Every banded step costs at least `horiz / BAND_V_MAX` — the floor that keeps the banded
    /// heuristic (`dist / BAND_V_MAX`) admissible — across every link kind and entry band.
    #[test]
    fn banded_cost_is_admissible() {
        let origins = [
            Vec3::ZERO,
            Vec3::new(300.0, 0.0, 0.0),
            Vec3::new(600.0, 0.0, -200.0),
            Vec3::new(900.0, 0.0, 0.0),
        ];
        let kinds = [
            LinkKind::Walk,
            LinkKind::Step,
            LinkKind::Drop,
            LinkKind::JumpGap,
            LinkKind::DoubleJump,
            LinkKind::SpeedJump,
            LinkKind::Teleport,
        ];
        for kind in kinds {
            let g = banded_graph(
                &origins,
                &[(0, 1, kind, 3.0)],
                if kind == LinkKind::SpeedJump { &[(0, 350.0, 0.7, false, 0.0, 0.0)] } else { &[] },
            );
            let horiz = (origins[1].xy() - origins[0].xy()).length();
            for band in 0..NBANDS as u8 {
                if let Some((cost, _)) = g.banded_step(0, band) {
                    assert!(
                        cost + 1e-4 >= horiz / BAND_V_MAX,
                        "{kind:?} band {band}: cost {cost} below the admissibility floor {}",
                        horiz / BAND_V_MAX,
                    );
                }
            }
        }
    }

    /// A longer straight Walk leg exits in a higher band, and a standing start (band 0) pays a
    /// spin-up runway before it gains — so a short leg from a standstill stays in band 0.
    #[test]
    fn banded_walk_exit_bands_monotone() {
        let g = banded_graph(
            &[Vec3::ZERO, Vec3::new(200.0, 0.0, 0.0), Vec3::new(2400.0, 0.0, 0.0)],
            &[(0, 1, LinkKind::Walk, 0.6), (0, 2, LinkKind::Walk, 7.5)],
            &[],
        );
        let (_, short) = g.banded_step(0, 0).unwrap(); // 200u from a standstill: below the spin-up
        let (_, long) = g.banded_step(1, 0).unwrap(); // 2400u from a standstill: builds real speed
        assert_eq!(short, 0, "a short standing-start leg should not leave band 0");
        assert!(long > short, "a long leg should exit a higher band than a short one ({long} vs {short})");
    }

    /// A chain of speed jumps with only a short platform between them: unroutable to a speed-unaware
    /// query (chained links priced away), routable to the banded planner when fed from a runway, and
    /// still unroutable from a standstill mid-chain (the carried band can't satisfy the next jump).
    #[test]
    fn banded_chain_needs_carried_speed() {
        // R --walk 2000--> A --chained SJ--> B --walk 200--> B2 --chained SJ--> C
        let g = banded_graph(
            &[
                Vec3::ZERO,                       // 0 R (runway start)
                Vec3::new(2000.0, 0.0, 0.0),      // 1 A (ledge)
                Vec3::new(2300.0, 0.0, 0.0),      // 2 B (landing)
                Vec3::new(2500.0, 0.0, 0.0),      // 3 B2 (short platform)
                Vec3::new(2800.0, 0.0, 0.0),      // 4 C (final landing)
            ],
            &[
                (0, 1, LinkKind::Walk, 6.25),
                (1, 2, LinkKind::SpeedJump, 1.7),
                (2, 3, LinkKind::Walk, 0.625),
                (3, 4, LinkKind::SpeedJump, 1.7),
            ],
            &[(1, 350.0, 0.7, true, 0.0, 0.0), (3, 350.0, 0.7, true, 0.0, 0.0)],
        );
        // Speed-unaware: the chained legs are priced away, so C is effectively unreachable.
        let flood = g.costs_from(0, &LinkCosts::default());
        assert!(flood[4] >= CLOSED_GATE_PENALTY, "unbanded query must treat the chain as blocked");
        // Banded, fed from the runway: the walk builds a band that carries both jumps.
        let route = g.find_path_banded(0, 4, MAX_SPEED, &LinkCosts::default()).expect("banded route exists");
        assert_eq!(route.links, vec![0, 1, 2, 3], "banded route should run the whole chain");
        // From a standstill mid-chain (on B2), the next chained jump is infeasible → no route.
        assert!(
            g.find_path_banded(3, 4, 0.0, &LinkCosts::default()).is_none(),
            "a standing start can't satisfy a chained speed jump"
        );
    }

    /// A certified curl speed jump is priced at its stored cost, so the planner takes it over a
    /// detour that beats the conservative per-`v_req` recompute a *straight* speed jump still gets.
    #[test]
    fn banded_prefers_certified_curl_over_detour() {
        // Direct curl R->C (cost 2.0) vs a two-JumpGap detour R->M->C (2 × 1.3 = 2.6). A straight
        // speed jump would be repriced from v_req (391 from a standstill ≈ 3.2s) and lose to the
        // detour; the curl's certified cost (2.0) wins. Same geometry, only the curl flag differs.
        let origins = [Vec3::ZERO, Vec3::new(600.0, 300.0, 0.0), Vec3::new(300.0, 150.0, 0.0)];
        let links = [
            (0, 1, LinkKind::SpeedJump, 2.0), // the curl (or straight) under test
            (0, 2, LinkKind::JumpGap, 1.3),   // detour leg 1
            (2, 1, LinkKind::JumpGap, 1.3),   // detour leg 2
        ];
        let curl = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 12.0, 0.0)]);
        let route = curl.find_path_banded(0, 1, MAX_SPEED, &LinkCosts::default()).expect("route exists");
        assert_eq!(route.links, vec![0], "the certified curl must be taken over the detour");
        let straight = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 0.0, 0.0)]);
        let route2 = straight.find_path_banded(0, 1, MAX_SPEED, &LinkCosts::default()).expect("route exists");
        assert_eq!(route2.links, vec![1, 2], "a straight speed jump is repriced high; the detour wins");
    }

    #[test]
    fn banded_plain_curl_credits_only_certified_landing_floor() {
        let origins = [Vec3::ZERO, Vec3::new(416.0, -192.0, 48.0)];
        let links = [(0, 1, LinkKind::SpeedJump, 1.5)];
        let certified = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 8.0, 479.4)]);
        let (_, certified_exit) = certified.banded_step(0, 0).expect("certified curl");
        assert_eq!(certified_exit, band_of(479.4));

        let unproven = banded_graph(&origins, &links, &[(0, 391.0, 0.68, false, 8.0, 0.0)]);
        let (_, fallback_exit) = unproven.banded_step(0, 0).expect("legacy curl fallback");
        assert_eq!(fallback_exit, band_of(391.0));
        assert!(certified_exit > fallback_exit, "unproven carry must not be credited");
    }

    /// Carried speed only survives a corner within the heading cone: a straight approach reaches a
    /// chained speed jump feasibly, an L-shaped one arrives demoted to band 0 and can't take it.
    #[test]
    fn banded_corner_demotes_carry() {
        let long_walk = |to_x: f32, to_y: f32| {
            [Vec3::ZERO, Vec3::new(2000.0, 0.0, 0.0), Vec3::new(to_x, to_y, 0.0)]
        };
        let links = [(0, 1, LinkKind::Walk, 6.25), (1, 2, LinkKind::SpeedJump, 1.7)];
        let sj = [(1usize, 350.0, 0.7, true, 0.0, 0.0)];
        // Straight: R→M→C all along +x — the carried band satisfies the chained jump.
        let straight = banded_graph(&long_walk(2300.0, 0.0), &links, &sj);
        assert!(
            straight.find_path_banded(0, 2, 0.0, &LinkCosts::default()).is_some(),
            "a straight approach should carry speed into the chained jump"
        );
        // Corner: R→M along +x, then the jump heads +y (90° turn) — carry is demoted, jump infeasible.
        let corner = banded_graph(&long_walk(2000.0, 300.0), &links, &sj);
        assert!(
            corner.find_path_banded(0, 2, 0.0, &LinkCosts::default()).is_none(),
            "a sharp corner should demote the carried band below the jump's requirement"
        );
    }

    // --- static reachability (see `reach`) ---

    fn reach_cell(x: f32) -> Cell {
        Cell { origin: Vec3::new(x, 0.0, 0.0), gx: 0, gy: 0 }
    }
    fn reach_link(from: CellId, to: CellId) -> Link {
        Link { from, to, kind: LinkKind::Walk, cost: 1.0 }
    }

    /// A one-way drop severs backward reachability but not forward: after a two-way pair a bot drops
    /// into a pocket it can't climb back out of. The SCC closure must reflect that asymmetry.
    #[test]
    fn reachability_respects_one_way_links() {
        // 0 <-> 1 (two-way), then a one-way chain 1 → 2 → 3 (a drop into a pocket, no way back up).
        let mut g = NavGraph::test_graph(
            vec![reach_cell(0.0), reach_cell(32.0), reach_cell(64.0), reach_cell(96.0)],
            vec![reach_link(0, 1), reach_link(1, 0), reach_link(1, 2), reach_link(2, 3)],
        );
        g.build_reachability();
        // Forward across the drop: everything downstream is reachable.
        assert!(g.reachable(0, 3), "0 should reach the pocket bottom");
        assert!(g.reachable(1, 2));
        // Backward across the drop: severed.
        assert!(!g.reachable(2, 1), "no way back up the drop");
        assert!(!g.reachable(3, 0), "the pocket bottom can't return");
        // The two-way pair is mutually reachable, and every cell reaches itself.
        assert!(g.reachable(0, 1) && g.reachable(1, 0));
        assert!(g.reachable(2, 2) && g.reachable(3, 3));
    }

    /// The goal-selection fan-out (`bot::par::flood_batch`) runs `costs_from` for many sources on a
    /// worker pool. That is only sound if `costs_from` is a pure function of `(graph, source, costs)` —
    /// no shared mutable state — so a batch computed on a rayon pool is **bit-identical** to the serial
    /// one. Guard that here (the pool-side determinism proof for Step 5), comparing raw f32 bits.
    #[test]
    fn costs_from_batch_is_thread_invariant() {
        use rayon::prelude::*;
        // A ring plus spokes: several distinct sources, each with a non-trivial flood.
        let cells: Vec<Cell> = (0..12).map(|i| reach_cell(i as f32 * 40.0)).collect();
        let mut links = Vec::new();
        for i in 0..12u32 {
            let j = (i + 1) % 12;
            links.push(reach_link(i, j));
            links.push(reach_link(j, i));
        }
        let g = NavGraph::test_graph(cells, links);
        let sources: Vec<CellId> = (0..12).collect();
        let costs = LinkCosts::default();

        let serial: Vec<Vec<f32>> = sources.iter().map(|&s| g.costs_from(s, &costs)).collect();
        let pool = rayon::ThreadPoolBuilder::new().num_threads(4).build().unwrap();
        let parallel: Vec<Vec<f32>> =
            pool.install(|| sources.par_iter().map(|&s| g.costs_from(s, &costs)).collect());

        assert_eq!(serial.len(), parallel.len());
        for (s, (a, b)) in sources.iter().zip(serial.iter().zip(&parallel)) {
            let ab: Vec<u32> = a.iter().map(|x| x.to_bits()).collect();
            let bb: Vec<u32> = b.iter().map(|x| x.to_bits()).collect();
            assert_eq!(ab, bb, "source {s}: parallel flood differs from serial bit-for-bit");
        }
    }

    /// `costs_from_within(t_max)` is the whole-graph flood restricted to the ≤ `t_max` ball: every
    /// cell within the bound reads its exact full-flood cost, and `settled` is exactly those cells in
    /// nondecreasing-cost order. This is the exactness guarantee the local escape/pickup floods lean on.
    #[test]
    fn costs_from_within_matches_bounded_full_flood() {
        // Chain 0↔1↔…↔9, unit walk links, so cell k settles at cost k from cell 0.
        let cells: Vec<Cell> = (0..10).map(|i| reach_cell(i as f32 * 40.0)).collect();
        let mut links = Vec::new();
        for i in 0..9u32 {
            links.push(reach_link(i, i + 1));
            links.push(reach_link(i + 1, i));
        }
        let g = NavGraph::test_graph(cells, links);
        let costs = LinkCosts::default();
        let full = g.costs_from(0, &costs);
        for &t_max in &[0.0_f32, 2.5, 4.0, 100.0] {
            let (bounded, settled) = g.costs_from_within(0, &costs, t_max);
            for c in 0..10usize {
                if full[c] <= t_max {
                    assert_eq!(bounded[c].to_bits(), full[c].to_bits(), "cell {c} within {t_max} must be exact");
                }
            }
            let expected: Vec<u32> = (0..10u32).filter(|&c| full[c as usize] <= t_max).collect();
            let mut got = settled.clone();
            got.sort_unstable();
            assert_eq!(got, expected, "settled set at t_max={t_max}");
            for w in settled.windows(2) {
                assert!(bounded[w[0] as usize] <= bounded[w[1] as usize], "settled out of cost order at {t_max}");
            }
        }
    }

    /// `nearest_reachable_to` picks the reachable cell physically closest to an unreachable goal, and
    /// the O(1)-table path agrees cell-for-cell with the Dijkstra-flood fallback (bare graph).
    #[test]
    fn nearest_reachable_matches_flood() {
        // A connected run 0→1→2 (x = 0, 100, 200) plus an isolated goal cell 3 at x = 250 with no
        // links. From 0 the reachable set is {0,1,2}; the closest of those to the goal is cell 2.
        let cells = vec![reach_cell(0.0), reach_cell(100.0), reach_cell(200.0), reach_cell(250.0)];
        let links = vec![reach_link(0, 1), reach_link(1, 0), reach_link(1, 2), reach_link(2, 1)];

        // Flood path: no reachability table built (reach == None), so it falls back to costs_from.
        let bare = NavGraph::test_graph(cells.clone(), links.clone());
        assert_eq!(bare.nearest_reachable_to(0, 3), Some(2), "flood fallback picks the closest reachable cell");

        // Table path: identical graph with the table built — must give the same answer.
        let mut built = NavGraph::test_graph(cells, links);
        built.build_reachability();
        assert_eq!(built.nearest_reachable_to(0, 3), Some(2), "table path must agree with the flood");
    }

    // --- LOD hierarchy (see `lod`) ---

    /// Clustering groups cells that share a spatial block *and* a link path inside it; an unconnected
    /// same-block cell and a cell in another block each get their own cluster. `LOD_SHIFT=3` → blocks
    /// are 8 grid columns wide, so gx 0..7 share a block and gx 8 starts the next.
    #[test]
    fn clusters_split_disconnected_blocks() {
        let cell = |gx: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, 0.0, 0.0), gx, gy: 0 };
        // cells 0,1,2 in block 0 (gx 0/1/2); cell 3 in block 1 (gx 8). 0↔1 linked; 2 isolated.
        let mut g = NavGraph::test_graph(
            vec![cell(0), cell(1), cell(2), cell(8)],
            vec![reach_link(0, 1), reach_link(1, 0)],
        );
        g.build_lod();
        let cl = |c: u32| g.cluster_of(c).unwrap();
        assert_eq!(cl(0), cl(1), "connected same-block cells share a cluster");
        assert_ne!(cl(0), cl(2), "an unconnected same-block cell is its own cluster");
        assert_ne!(cl(0), cl(3), "a cell in another block is a different cluster");
        assert_ne!(cl(2), cl(3), "distinct singletons stay distinct");
        assert_eq!(g.cluster_count(), 3, "0/1, 2, and 3 → three clusters");
        // A one-way link still groups its endpoints (undirected clustering).
        let mut one_way = NavGraph::test_graph(vec![cell(0), cell(1)], vec![reach_link(0, 1)]);
        one_way.build_lod();
        assert_eq!(one_way.cluster_of(0), one_way.cluster_of(1), "a one-way drop still clusters together");
    }

    /// On a straight corridor spanning several cluster blocks the abstract portal path *is* the only
    /// path, so the coarse estimate equals the exact flood at every cell — near cells from the fine
    /// flood, far cells reconstructed through portals and intra-cluster transit.
    #[test]
    fn coarse_costs_match_exact_on_a_chain() {
        let cell = |gx: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, 0.0, 0.0), gx, gy: 0 };
        let cells: Vec<Cell> = (0..25).map(|i| cell(i)).collect();
        let mut links = Vec::new();
        for i in 0..24u32 {
            links.push(reach_link(i, i + 1));
            links.push(reach_link(i + 1, i));
        }
        let mut g = NavGraph::test_graph(cells, links);
        g.build_lod();
        assert!(g.cluster_count() >= 4, "a 25-cell chain spans four 8-column blocks, got {}", g.cluster_count());

        let costs = LinkCosts::default();
        let exact = g.costs_from(0, &costs);
        let coarse = g.coarse_costs(0, &costs, true);
        for c in 0..25u32 {
            assert_eq!(
                coarse.cost_to(c).to_bits(),
                exact[c as usize].to_bits(),
                "cell {c}: coarse {} must equal exact {} on a linear chain",
                coarse.cost_to(c),
                exact[c as usize],
            );
        }

        // Bare graph (no LOD built): coarse falls back to the exact full flood.
        let bare = NavGraph::test_graph((0..3).map(cell).collect(), vec![reach_link(0, 1), reach_link(1, 2)]);
        let bare_exact = bare.costs_from(0, &costs);
        let bare_coarse = bare.coarse_costs(0, &costs, true);
        for c in 0..3u32 {
            assert_eq!(bare_coarse.cost_to(c).to_bits(), bare_exact[c as usize].to_bits(), "bare-graph fallback cell {c}");
        }
    }

    /// The safety property goal scoring relies on: over a 2-D grid spanning several clusters, the
    /// coarse estimate never *underestimates* the exact cost (an abstract path is a real path, so its
    /// cost ≥ the shortest) and agrees on reachability. Underestimating would let a bot think an item
    /// is closer than it is.
    #[test]
    fn coarse_never_underestimates_on_a_grid() {
        let (w, h) = (18i32, 6i32);
        let idx = |gx: i32, gy: i32| (gy * w + gx) as u32;
        let mut cells = Vec::new();
        for gy in 0..h {
            for gx in 0..w {
                cells.push(Cell { origin: Vec3::new(gx as f32 * 32.0, gy as f32 * 32.0, 0.0), gx, gy });
            }
        }
        let mut links = Vec::new();
        for gy in 0..h {
            for gx in 0..w {
                if gx + 1 < w {
                    links.push(reach_link(idx(gx, gy), idx(gx + 1, gy)));
                    links.push(reach_link(idx(gx + 1, gy), idx(gx, gy)));
                }
                if gy + 1 < h {
                    links.push(reach_link(idx(gx, gy), idx(gx, gy + 1)));
                    links.push(reach_link(idx(gx, gy + 1), idx(gx, gy)));
                }
            }
        }
        let mut g = NavGraph::test_graph(cells, links);
        g.build_lod();
        assert!(g.cluster_count() >= 3, "18-wide grid spans three gx-blocks, got {}", g.cluster_count());

        let costs = LinkCosts::default();
        let exact = g.costs_from(0, &costs);
        let coarse = g.coarse_costs(0, &costs, true);
        for c in 0..(w * h) as u32 {
            let (e, co) = (exact[c as usize], coarse.cost_to(c));
            assert_eq!(e.is_finite(), co.is_finite(), "cell {c}: reachability must agree");
            if e.is_finite() {
                assert!(co >= e - 1e-3, "cell {c}: coarse {co} underestimates exact {e}");
            }
        }
    }

    /// The coverage-pass guardrail: a cell reachable only through a crossing that the cheapest-per-pair
    /// rep dropped must still get a finite coarse cost (it silently read INFINITY before). Cluster A =
    /// {0,1}, cluster B = {2,3}; two A→B crossings (cheap Walk 1→2 = rep, pricier Drop 1→3), and inside
    /// B only one-way 3→2 — so the rep's landing (2) can't reach 3.
    #[test]
    fn coarse_covers_cells_reachable_only_via_a_dropped_crossing() {
        let cell = |gx: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, 0.0, 0.0), gx, gy: 0 };
        let cells = vec![cell(0), cell(1), cell(8), cell(9)];
        let links = vec![
            reach_link(0, 1),
            reach_link(1, 0),
            reach_link(1, 2), // cheap Walk cross → rep
            Link { from: 1, to: 3, kind: LinkKind::Drop, cost: 2.0 }, // pricier cross to cell 3
            reach_link(3, 2), // one-way inside B
        ];
        let mut g = NavGraph::test_graph(cells, links);
        g.build_reachability();
        g.build_lod();
        let costs = LinkCosts::default();
        let coarse = g.coarse_costs(0, &costs, false);
        assert!(g.reachable(0, 3), "cell 3 is reachable via the drop");
        assert!(coarse.cost_to(3).is_finite(), "coverage pass must give reachable cell 3 a finite coarse cost");
        for c in 0..4u32 {
            assert_eq!(g.reachable(0, c), coarse.cost_to(c).is_finite(), "reachability/finiteness must agree at cell {c}");
        }
    }

    /// Storey-banded clustering: a platform and the pit directly beneath it (same 256u XY block, joined
    /// only by a one-way drop) must land in *different* clusters, so the cheap drop into the pit can't
    /// evict the climb onto the platform as the block pair's single representative crossing. Z-blind,
    /// the two merged into one cluster spanning both heights; the cheap drop won the rep slot, the
    /// abstract route into the block landed in the pit (which can't climb back up), and the platform
    /// read `cost_to = INFINITY` while the fine graph reached it — the bravado quad, unreachable under
    /// LOD. Block A (gx&lt;8) is a launch walkway at platform height; block B (gx≥8) is the platform
    /// (z 256) over a pit (z 0). The platform's outbound jump makes it a self-covering portal, so the
    /// coverage pass can't paper over the eviction — exactly the bravado shape.
    #[test]
    fn coarse_reaches_a_platform_over_the_pit_below_it() {
        let at = |x: f32, y: f32, z: f32, gx: i32, gy: i32| Cell { origin: Vec3::new(x, y, z), gx, gy };
        let cells = vec![
            at(224.0, 0.0, 256.0, 7, 0),  // 0 W  — launch walkway (block A, storey 2)
            at(224.0, 32.0, 256.0, 7, 1), // 1 W2 — walkway neighbour (gives the platform an outbound portal)
            at(256.0, 0.0, 256.0, 8, 0),  // 2 P0 — platform edge (block B, storey 2)
            at(288.0, 0.0, 256.0, 9, 0),  // 3 P1 — platform interior — the "quad"
            at(256.0, 0.0, 0.0, 8, 0),    // 4 D0 — pit below the platform (block B, storey 0)
            at(288.0, 0.0, 0.0, 9, 0),    // 5 D1 — pit
        ];
        let jump = |from, to| Link { from, to, kind: LinkKind::JumpGap, cost: 1.0 };
        let drop = |from, to| Link { from, to, kind: LinkKind::Drop, cost: 0.3 };
        let links = vec![
            reach_link(0, 1),
            reach_link(1, 0), // walkway intra (block A)
            reach_link(2, 3),
            reach_link(3, 2), // platform intra (block B, storey 2)
            reach_link(4, 5),
            reach_link(5, 4), // pit intra (block B, storey 0)
            jump(0, 2),       // climb: walkway → platform edge (the crossing that must survive)
            drop(0, 4),       // cheaper drop: walkway → pit (the evictor — same block B pre-banding)
            drop(2, 4),       // one-way drop platform → pit (merges the two pre-banding)
            jump(3, 1),       // platform → walkway: makes the platform a self-covering takeoff portal
        ];
        let mut g = NavGraph::test_graph(cells, links);
        g.build_reachability();
        g.build_lod();

        // The storey band keeps the platform (cell 3, z 256) out of the pit's cluster (cell 4, z 0).
        assert_ne!(g.cluster_of(3), g.cluster_of(4), "platform and the pit beneath it must not share a cluster");

        let costs = LinkCosts::default();
        let coarse = g.coarse_costs(0, &costs, false);
        let exact = g.costs_from(0, &costs);
        assert!(g.reachable(0, 3), "the platform is reachable via the climb");
        assert!(coarse.cost_to(3).is_finite(), "coarse must reach the platform — the climb wasn't evicted into the pit");
        for c in 0..6u32 {
            assert_eq!(g.reachable(0, c), coarse.cost_to(c).is_finite(), "reachability/finiteness must agree at cell {c}");
            if exact[c as usize].is_finite() {
                assert!(coarse.cost_to(c) >= exact[c as usize] - 1e-3, "cell {c}: coarse {} underestimates exact {}", coarse.cost_to(c), exact[c as usize]);
            }
        }
    }

    /// A directed cluster pair whose *cheapest* crossing is a shut gate but which also has a pricier
    /// gate-free crossing: keeping a gate-free representative lets the coarse cost route around the shut
    /// door, so a prize past the gate-free crossing reads reachable (below the closed-gate wall) exactly
    /// as the exact flood does — the very input the openable-gate valuation reads. Without the second
    /// rep the strictly-cheaper gated crossing evicts the gate-free one and the prize reads sealed.
    #[test]
    fn coarse_routes_around_a_shut_gate_via_a_gate_free_crossing() {
        let at = |gx: i32, gy: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, gy as f32 * 32.0, 0.0), gx, gy };
        // Cluster A = block (0,0): cells 0,1. Cluster B = block (1,0): cells 2,3.
        let cells = vec![at(0, 0), at(1, 0), at(8, 0), at(15, 7)];
        let links = vec![
            reach_link(0, 1),
            reach_link(1, 0),
            reach_link(2, 3), // B intra
            reach_link(3, 2),
            Link { from: 1, to: 2, kind: LinkKind::Walk, cost: 1.0 }, // cheap crossing (gated below)
            Link { from: 1, to: 3, kind: LinkKind::Walk, cost: 3.0 }, // pricier gate-free crossing
        ];
        let mut g = NavGraph::test_graph(cells, links);
        // Tag the cheap 1→2 crossing (link 4) as gated directly — `add_gates` needs a grid index the
        // bare test graph lacks, and this test exercises the coarse router, not the geometry splice.
        let gate = g.gates.push(Gate {
            obstruction: 0,
            closed_origin: g.cells[2].origin,
            closed_min: Vec3::ZERO,
            closed_max: Vec3::ZERO,
            activator: 0,
            button_cell: 0,
            aim: g.cells[0].origin,
            shoot: false,
        });
        g.gates.tag(4, gate);
        assert_eq!(g.gate_count(), 1, "gate registered");
        assert_eq!(g.gate_of_link(4), Some(0), "the cheap 1→2 crossing is gated");
        assert_eq!(g.gate_of_link(5), None, "the pricier 1→3 crossing is gate-free");
        g.build_reachability();
        g.build_lod();

        let shut = LinkCosts { gate_closed: &[true], ..Default::default() };
        // Cell 3 is reachable gate-free (0→1→3, cost 1+3): coarse must price it below the closed wall,
        // not seal it at ~100k behind the cheaper gated crossing.
        let c3 = g.coarse_costs(0, &shut, true).cost_to(3);
        assert!(c3 < CLOSED_GATE_PENALTY, "cell 3 must route around the shut gate, got {c3}");
        assert!((c3 - 4.0).abs() < 1e-3, "cell 3's gate-free coarse cost should be 4, got {c3}");
        // With the gate open the cheaper crossing (0→1→2→3 = 1+1+1) wins again — both reps are present,
        // so the gate-free alternate never costs when the door is open.
        let c3_open = g.coarse_costs(0, &LinkCosts::default(), true).cost_to(3);
        assert!((c3_open - 3.0).abs() < 1e-3, "gate open: cheapest route to cell 3 is 3, got {c3_open}");
    }

    /// The LOD steer corridor plants its interim short of a far goal (bounding the fine search) but
    /// steers a near goal directly; its window contains a route the restricted search actually finds.
    #[test]
    fn corridor_bounds_a_far_goal() {
        let cell = |gx: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, 0.0, 0.0), gx, gy: 0 };
        let cells: Vec<Cell> = (0..25).map(|i| cell(i)).collect();
        let mut links = Vec::new();
        for i in 0..24u32 {
            links.push(reach_link(i, i + 1));
            links.push(reach_link(i + 1, i));
        }
        let mut g = NavGraph::test_graph(cells, links);
        g.build_lod();
        let costs = LinkCosts::default();

        // Far goal (cell 24): the interim is short of it, at/past the horizon, in the window.
        let c = g.corridor(0, 24, &costs, 4.0).expect("a far goal has a corridor");
        assert!(c.interim < 24, "interim {} should fall short of the far goal", c.interim);
        assert!(g.coarse_costs(0, &costs, false).cost_to(c.interim) >= 4.0, "interim at/past the horizon");
        assert!(c.allowed[g.cluster_of(0).unwrap() as usize], "the home cluster is in the window");
        assert!(c.allowed[g.cluster_of(c.interim).unwrap() as usize], "the interim's cluster is in the window");
        // The restricted search finds a route to the interim (the corridor is a real in-window path)…
        assert!(
            !g.find_path_within(0, c.interim, &costs, &c.allowed).unwrap_or_default().is_empty(),
            "restricted search must find the corridor route"
        );
        // …and it truly bounds: a cell outside the window is unreachable to the restricted search.
        let outside = (0..g.cluster_count() as u32).find(|&cl| !c.allowed[cl as usize]);
        if let Some(cl) = outside {
            let far = (0..25u32).find(|&x| g.cluster_of(x) == Some(cl)).unwrap();
            assert!(g.find_path_within(0, far, &costs, &c.allowed).is_none(), "window must exclude cell {far}");
        }
        // Same-cluster goal, and a goal within the horizon: steer directly (no corridor).
        assert!(g.corridor(0, 5, &costs, 4.0).is_none(), "a same-cluster goal steers directly");
        assert!(g.corridor(0, 10, &costs, 100.0).is_none(), "a goal within the horizon steers directly");
    }

    /// The LOD build folds the water tax (bot-independent) and hazard hp (priced per bot) of a far
    /// intra-cluster liquid link into the coarse estimate — because the liquid columns are now flagged
    /// on the worker *before* `build_lod`, which prices them at birth (no post-swap patch).
    #[test]
    fn lod_prices_water_and_hazard() {
        let cell = |gx: i32| Cell { origin: Vec3::new(gx as f32 * 32.0, 0.0, 0.0), gx, gy: 0 };
        let cells: Vec<Cell> = (0..15).map(|i| cell(i)).collect();
        let mut links = Vec::new();
        for i in 0..14u32 {
            links.push(reach_link(i, i + 1)); // link 2i = (i,i+1)
            links.push(reach_link(i + 1, i));
        }
        let mut g = NavGraph::test_graph(cells, links);
        g.build_lod();
        let costs = LinkCosts::default();
        let dry = g.coarse_costs(0, &costs, false).cost_to(12);

        // Flag link 10→11 (index 20) as a water + lava crossing and rebuild the LOD as the worker does
        // (liquid columns filled before `build_lod`, which then prices them into the abstract graph).
        let nlinks = g.links.len();
        g.water_extra = vec![0.0; nlinks];
        g.hazard_hp = vec![0.0; nlinks];
        g.water_extra[20] = 3.0;
        g.hazard_hp[20] = 25.0;
        g.build_lod();

        // Cell 12 is reached across 10→11, so its coarse cost grows by exactly the water tax.
        let wet = g.coarse_costs(0, &costs, false).cost_to(12);
        assert!((wet - (dry + 3.0)).abs() < 1e-3, "water tax: wet {wet} != dry+3 {}", dry + 3.0);
        // With a hazard price set, the lava hp adds cost on top, scaled to the bot's nerve.
        let hazcosts = LinkCosts { hazard: Some(HazardPrice::new(100.0)), ..LinkCosts::default() };
        let hurt = g.coarse_costs(0, &hazcosts, false).cost_to(12);
        assert!(hurt > wet, "hazard pricing should add cost: hurt {hurt} !> wet {wet}");
    }
}
