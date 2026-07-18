// SPDX-License-Identifier: AGPL-3.0-or-later

//! Jump-link generation: the run-jump (`find_jumps`), rtx double-jump (`add_double_jumps`), and
//! bhop-carried speed-jump (`add_speed_jumps`) passes plus their per-cell solvers and the runway
//! measurer. Each pass floods candidates off ledge edges, dedups them per compass octant, arc-tests
//! clearance, and splices the survivors into the graph. Runs on the parallel build's worker cells.

use glam::{Vec2, Vec3, Vec3Swizzles};

use super::geom::*;
use super::physics::*;
use super::*;
use crate::bsp::Bsp;
use crate::math::{wrap180, yaw_of};
use crate::pmove::{pm_step, PmParams, PmState};
use crate::strafe::{air_accel_max, air_correct, apply_friction, apply_groundaccel, Cmd, MOVE_SPEED};

impl NavGraph {
    /// Jump links out of `from`: only from a **ledge edge** (the adjacent column toward the
    /// target has no walkable ground, i.e. a gap/pit), within run-jump reach and apex, with a
    /// clear arc. Deduped to the single nearest target per (compass octant, elevation band) so a
    /// ledge sprouts a handful of jumps, not hundreds of redundant parallel ones — banded by
    /// elevation because targets a storey apart are distinct destinations: without the band, a
    /// short descending jump into the pit under a gap shadows the level jump *across* it onto a
    /// separate ledge, and the pit floor doesn't lead back up to that ledge.
    pub(super) fn find_jumps(&self, bsp: &Bsp, from: CellId) -> Vec<Link> {
        let a = self.cells[from as usize];
        if bsp.is_liquid_at(a.origin) {
            return Vec::new(); // submerged: the jump input swims up, so a jump takeoff here is a no-op
        }
        // best (distance, link) per compass direction bucket (3×3, center unused) × elevation band
        let mut best = [[None::<(f32, Link)>; JUMP_ELEV_BANDS]; 9];
        for to in self.neighbors_within(a.gx, a.gy, jump_grid_radius()) {
            let b = self.cells[to as usize];
            let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
            if dgx.abs() <= 1 && dgy.abs() <= 1 {
                continue; // adjacent — a grounded link if anything
            }
            let dz = b.origin.z - a.origin.z;
            if !(-MAX_DROP..=JUMP_APEX).contains(&dz) {
                continue;
            }
            let horiz = (b.origin.xy() - a.origin.xy()).length();
            if horiz > JUMP_REACH {
                continue;
            }
            // Must take off from a ledge: the column one step toward B isn't walkable ground.
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                continue;
            }
            // Shallow crossings check the symmetric hop parabola; a deep plunge flies a very
            // different path (out at run speed, then mostly straight down), so sample that.
            let clear = if dz < -JUMP_ELEV_SPAN {
                ballistic_clear(bsp, a.origin, b.origin)
            } else {
                arc_clear(bsp, a.origin, b.origin)
            };
            if !clear {
                continue;
            }
            // A jump *down* must land in a spot the hull can descend into — arc sampling can skip a
            // thin floor lip (a slot too small for the hull) that the vertical hull trace catches.
            if dz < 0.0 && !descent_clear(bsp, a.origin.z, b.origin) {
                continue;
            }
            let slot = &mut best[dir_bucket(dgx, dgy)][jump_elev_band(dz)];
            if slot.is_none_or(|(d, _)| horiz < d) {
                *slot = Some((
                    horiz,
                    Link {
                        from,
                        to,
                        kind: LinkKind::JumpGap,
                        cost: link_cost(LinkKind::JumpGap, horiz, dz),
                    },
                ));
            }
        }
        best.into_iter().flatten().flatten().map(|(_, l)| l).collect()
    }

    /// Splice **double-jump** links: gaps/ledges beyond a single jump's reach but within a double
    /// jump's, gated on `rtx_doublejump`. Same ledge-edge/octant-dedup shape as [`find_jumps`], but
    /// the wider reach/apex and the taller arc-clearance envelope — and only for targets a plain
    /// jump can't already make (else a `JumpGap` covers it). The bot air-jumps mid-flight to cross.
    pub fn add_double_jumps(&mut self, bsp: &Bsp) {
        // Solve per source cell in parallel (read-only borrow), then splice serially. The indexed
        // `collect` returns per-cell results in cell order, so the splice — and thus link indices —
        // are identical to a sequential build. The solvers never observe each other's pending links
        // (same as the sequential drain), so within-stage parallelism is sound.
        let this = &*self;
        let pending: Vec<Vec<Link>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|from| {
                let mut out = Vec::new();
                this.solve_double_jumps_from(bsp, from, &mut out);
                out
            })
            .collect();
        for link in pending.into_iter().flatten() {
            self.push_link(link);
        }
    }

    /// The double-jump links leaving cell `from`, appended to `out`.
    fn solve_double_jumps_from(&self, bsp: &Bsp, from: CellId, out: &mut Vec<Link>) {
        let a = self.cells[from as usize];
        if bsp.is_liquid_at(a.origin) {
            return; // submerged takeoff: can't jump (the jump input swims up)
        }
        let mut best: [Option<(f32, Link)>; 9] = Default::default();
        for to in self.neighbors_within(a.gx, a.gy, double_jump_grid_radius()) {
            if to == from {
                continue;
            }
            let b = self.cells[to as usize];
            let (dgx, dgy) = (b.gx - a.gx, b.gy - a.gy);
            if dgx.abs() <= 1 && dgy.abs() <= 1 {
                continue;
            }
            let dz = b.origin.z - a.origin.z;
            let horiz = (b.origin.xy() - a.origin.xy()).length();
            if !(-DJ_MAX_DROP..=DOUBLE_JUMP_APEX).contains(&dz) || horiz > DOUBLE_JUMP_REACH {
                continue;
            }
            // Only worthwhile beyond a single jump — otherwise `find_jumps` already linked it.
            if horiz <= JUMP_REACH && dz <= JUMP_APEX {
                continue;
            }
            // Take off from a ledge edge, clear the taller arc, and don't duplicate a route the
            // static graph already provides (walk/step/jump).
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z)
                || !arc_clear_peak(bsp, a.origin, b.origin, DOUBLE_ARC_PEAK, 12)
                || (dz < 0.0 && !descent_clear(bsp, a.origin.z, b.origin))
                || self.has_direct_link(from, to)
            {
                continue;
            }
            let oct = dir_bucket(dgx, dgy);
            if best[oct].is_none_or(|(d, _)| horiz < d) {
                best[oct] = Some((
                    horiz,
                    Link {
                        from,
                        to,
                        kind: LinkKind::DoubleJump,
                        cost: link_cost(LinkKind::DoubleJump, horiz, dz),
                    },
                ));
            }
        }
        out.extend(best.into_iter().flatten().map(|(_, l)| l));
    }

    /// Splice **speed-jump** links: leaps across gaps too wide for any single/double jump, cleared by
    /// arriving at the ledge with bunnyhop-built speed. For each ledge edge, measure the straight
    /// runway feeding it, cap the attainable speed to that, and link the widest reachable targets —
    /// but with `from` set to the *runway start* so A* commits the whole run-up (the bot is thus
    /// guaranteed the speed). Only where a plain/double jump can't already make it. Gated on bhop.
    ///
    /// A jump with no self-contained runway also emits a **chained** variant (`from` = the ledge
    /// itself) for the case a human relies on: a chain of gaps with only a short platform between
    /// them, where speed carried from the previous jump's landing clears the next. These have no
    /// runway budget of their own, so they are traversable only by the speed-band planner
    /// ([`Self::find_path_banded`]), which proves the entry band carries `v_req`; the speed-unaware
    /// `find_path`/`costs_from` price them away ([`Self::chained_block`]) since a standing start
    /// can't make them. Chained candidates use a separate small per-cell cap so they never evict a
    /// self-contained jump.
    pub fn add_speed_jumps(&mut self, bsp: &Bsp, params: SpeedJumpParams, double_jump: bool) {
        let k = bhop_k(params.accel, params.maxspeed);
        self.sj_k = k; // the banded planner prices carried speed with this map's k
                       // Solve per ledge in parallel (read-only borrow); indexed `collect` keeps cell order, so the
                       // serial splice below reproduces the sequential build's link indices exactly.
        let this = &*self;
        let pending: Vec<Vec<(Link, SpeedJumpTraversal)>> = (0..this.cells.len() as CellId)
            .into_par_iter()
            .map(|ledge| {
                let mut out = Vec::new();
                this.solve_speed_jumps_from(bsp, ledge, params, k, double_jump, &mut out);
                out
            })
            .collect();
        for (link, tr) in pending.into_iter().flatten() {
            self.push_speed_jump(link, tr);
        }
        // Curl jumps second (after the straight speed jumps are spliced): a separate certified pass for
        // gaps that need a run-up *and* an air-turn.
        if params.curl {
            let this = &*self;
            let curls: Vec<Vec<(Link, SpeedJumpTraversal)>> = (0..this.cells.len() as CellId)
                .into_par_iter()
                .map(|ledge| {
                    let mut out = Vec::new();
                    this.solve_curl_jumps_from(bsp, ledge, params, k, &mut out);
                    out
                })
                .collect();
            // Global dedup by target cell: many source ledges certify a curl onto the same platform, so
            // keep only the cheapest few per target (the same landing from a dozen corridors is noise the
            // planner never needs). Deterministic: iterate the indexed collect in cell order, and among
            // equal-cost keep the earliest. `CURL_TARGET_MAX` distinct sources per target land here.
            let mut per_target: std::collections::HashMap<CellId, Vec<(Link, SpeedJumpTraversal)>> =
                std::collections::HashMap::new();
            for (link, tr) in curls.into_iter().flatten() {
                let slot = per_target.entry(link.to).or_default();
                slot.push((link, tr));
            }
            // Stable target order (grid/cell id) so the splice is deterministic across builds.
            let mut targets: Vec<CellId> = per_target.keys().copied().collect();
            targets.sort_unstable();
            for tgt in targets {
                let mut v = per_target.remove(&tgt).unwrap();
                v.sort_by(|a, b| a.0.cost.total_cmp(&b.0.cost).then(a.0.from.cmp(&b.0.from)));
                v.truncate(CURL_TARGET_MAX);
                for (link, tr) in v {
                    self.push_speed_jump(link, tr);
                }
            }
            // Chained ground-turn curls third: leaps no local run-up can deliver
            // at all (carried entry speed + grounded pre-launch rotation). Same
            // dedup discipline, own per-cell budget.
            let this = &*self;
            let gts: Vec<Vec<(Link, SpeedJumpTraversal)>> = (0..this.cells.len() as CellId)
                .into_par_iter()
                .map(|ledge| {
                    let mut out = Vec::new();
                    this.solve_chained_ground_turn_from(bsp, ledge, params, &GT_ENTRY_SPEEDS, &mut out);
                    // Additive low-entry sibling: the ground-optimal single-sided sweep
                    // (GROUND_TURN_OPTIMAL_VERSION contracts) from the carried ~320..360
                    // band. Always on for this branch (no cvar gate). It appends into the
                    // same `out`; the per-target dedup below (cheapest-cost-wins,
                    // one-per-source-envelope, seen_from) mixes v1 and v3 candidates
                    // deterministically — a v3 link only survives if it is cheaper than
                    // the v1 covering the same target, exactly as within-version.
                    this.solve_chained_ground_turn_optimal_curl(bsp, ledge, params, &GT_OPT_ENTRY_SPEEDS, &mut out);
                    out
                })
                .collect();
            let mut per_target: std::collections::HashMap<CellId, Vec<(Link, SpeedJumpTraversal)>> =
                std::collections::HashMap::new();
            for (link, tr) in gts.into_iter().flatten() {
                per_target.entry(link.to).or_default().push((link, tr));
            }
            let mut targets: Vec<CellId> = per_target.keys().copied().collect();
            targets.sort_unstable();
            for tgt in targets {
                let mut v = per_target.remove(&tgt).unwrap();
                v.sort_by(|a, b| a.0.cost.total_cmp(&b.0.cost).then(a.0.from.cmp(&b.0.from)));
                // Chained links are only traversable where the entry band is
                // provable, so source diversity matters more than for plain
                // curls: a cheap link from an unreachable-at-speed corridor
                // must not evict the one the certified chain arrives through.
                // One link per source and exact certified entry envelope. Cold, mid and hot
                // arrivals — and distinct yaws at the same speed — are disjoint executable
                // contracts. Planner bands are too coarse a dedup key: 354 and 380 ups share one.
                let mut seen_from_contract = std::collections::HashSet::new();
                v.retain(|(link, tr)| {
                    let envelope = tr.ground_turn.map_or((0, 0, 0, 0), |gt| {
                        (
                            gt.entry_speed_lo.to_bits(),
                            gt.entry_speed_hi.to_bits(),
                            gt.entry_yaw_lo.to_bits(),
                            gt.entry_yaw_hi.to_bits(),
                        )
                    });
                    seen_from_contract.insert((link.from, envelope))
                });
                v.truncate(GT_TARGET_MAX);
                for (link, tr) in v {
                    self.push_speed_jump(link, tr);
                }
            }
        }
    }

    /// The curl-jump links leaving ledge cell `ledge`: targets offset off the run-up heading that a
    /// straight speed jump can't own (too fast for the air-strafe credit, or its arc is blocked), each
    /// certified by a `pm_step` rollout of the game's takeoff regime (ground prestrafe to the lip, leap
    /// along the corridor, `air_correct`-curl onto the landing). Emitted as a self-contained SpeedJump
    /// carrying its certified `curl_gain`, so the banded planner prices it by its stored cost and the
    /// runtime flies it with the curl controller. Its own per-cell budget, so it never evicts a straight
    /// jump.
    fn solve_curl_jumps_from(
        &self,
        bsp: &Bsp,
        ledge: CellId,
        params: SpeedJumpParams,
        k: f32,
        out: &mut Vec<(Link, SpeedJumpTraversal)>,
    ) {
        let a = self.cells[ledge as usize];
        if bsp.is_liquid_at(a.origin) {
            return; // submerged takeoff: can't jump
        }
        // On a low-gravity server even a flat leap hangs longer than the rollout tick cap, so no curl
        // could ever certify — skip the whole (otherwise enormous) scan rather than roll futilely.
        if jump_airtime(0.0, params.gravity) > CURL_MAX_TICKS as f32 * CURL_DT {
            return;
        }
        let p = PmParams {
            gravity: params.gravity,
            accel: params.accel,
            friction: params.friction,
            stopspeed: params.stopspeed,
            maxspeed: params.maxspeed,
        };
        let mut cands: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new(); // (horiz, link, tr)
        for (dgx, dgy) in COMPASS {
            // Leap into a gap (no ground the leap way); measure the corridor run-up behind the lip.
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                continue;
            }
            let runway = self.measure_runway(bsp, &a, dgx, dgy);
            if runway < CURL_MIN_RUNWAY {
                continue; // too little run-up for the ground prestrafe to build curl speed
            }
            // The takeoff speed is the ground-prestrafe equilibrium (saturates well inside CURL_RUNUP_CAP),
            // so it's the *committed* run-up — not the full measured corridor — that a curl builds over.
            let v_deliver = prestrafe_delivered(
                runway.min(CURL_RUNUP_CAP),
                params.accel,
                params.maxspeed,
                params.friction,
                params.stopspeed,
            );
            let v_max_straight = SPEED_JUMP_V_CAP.min(BHOP_EFF * attainable_speed(MAX_SPEED, runway, k));
            let psi0 = yaw_of(Vec2::new(dgx as f32, dgy as f32)); // corridor / takeoff heading
                                                                  // A rollout can only certify a landing it reaches inside the tick cap, so bound the target
                                                                  // scan (and the per-target airtime) by that flight time — not the full SJ_MAX_DROP fall, which
                                                                  // on low-gravity servers is many seconds of futile scan-and-rollout.
            let fly_cap = CURL_MAX_TICKS as f32 * CURL_DT;
            let reach = v_deliver * fly_cap;
            let scan = ((reach / GRID).ceil() as i32).max(1);
            for to in self.neighbors_within(a.gx, a.gy, scan) {
                if to == ledge {
                    continue;
                }
                let b = self.cells[to as usize];
                let dz = b.origin.z - a.origin.z;
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                if !(-SJ_MAX_DROP..=JUMP_APEX).contains(&dz) || horiz <= JUMP_REACH {
                    continue;
                }
                // The target must sit off the corridor by [LO, HI]° — a genuine curl, not a straight leap.
                let off = wrap180(yaw_of(b.origin.xy() - a.origin.xy()) - psi0).abs();
                if !(CURL_ANGLE_LO..=CURL_ANGLE_HI).contains(&off) {
                    continue;
                }
                if self.has_direct_link(ledge, to) {
                    continue; // a plain jump / existing link already leaves the ledge for here
                }
                let airtime = jump_airtime(dz, params.gravity);
                if airtime <= 0.0 || airtime > fly_cap {
                    continue; // unreachable, or a drop too deep to land within the rollout tick cap
                }
                // Only curl what the straight pass could NOT own: too fast for its air-strafe credit, or
                // an arc it can't fly through. (A target the straight pass covers needs no curl.)
                let steps = ((horiz / 24.0).ceil() as i32).max(8);
                let arc_ok = arc_clear_peak(bsp, a.origin, b.origin, JUMP_APEX, steps);
                let v_req_straight = v_required(horiz, dz, params.gravity);
                if arc_ok && v_req_straight * SJ_MARGIN <= v_max_straight {
                    continue;
                }
                // (No separate slide-out check: `certify_curl` below requires an actual on-ground
                // touchdown resolving to the target cell within tolerance, which is the landing proof.)
                // The expensive step, reached only by the survivors: certify a curl by rollout. Search
                // the takeoff *back* along the run-up — a fast run-up overshoots a leap right at the pit
                // edge, so the leap point slides back (over the near ground, which the arc clears) until
                // the delivered speed matches the distance. First (latest) leap that certifies wins.
                let t_max = (runway - CURL_MIN_RUNWAY).clamp(0.0, CURL_TAKEOFF_BACKOFF);
                let mut solved: Option<(Vec3, Vec3, f32, f32, f32, f32)> = None;
                // (takeoff, from_pt, v_req, gain, landing_speed_lo, cost)
                // The runtime takes off along the from→takeoff line, so that heading is ours to choose —
                // and certification is sharply sensitive to it (a real lip's approach is rarely exactly on
                // a compass axis; the dm3 curl_mid certifies at 6° off but not at 0°). Sample a few
                // headings around the corridor axis and place the from-cell along whichever certifies, so
                // the bot flies precisely the line that was proven.
                'psi: for dpsi in CURL_PSI_SAMPLES {
                    let psi = psi0 + dpsi;
                    let (sp, cp) = psi.to_radians().sin_cos();
                    let dir = Vec3::new(cp, sp, 0.0);
                    let mut t = 0.0;
                    loop {
                        // Snap the leap point to an actual cell: correct z on a stepped run-up, and steps the
                        // search over the grid so a narrow certify window isn't jumped past.
                        if let Some(cell) = self.nearest_within(a.origin - dir * t, GRID * 0.75, STEP_HEIGHT * 2.0) {
                            let takeoff = self.cells[cell as usize].origin;
                            let back = (takeoff.xy() - a.origin.xy()).length();
                            // The committed run-up is capped (CURL_RUNUP_CAP) but must fit behind this takeoff.
                            let runup_len = (runway - back).min(CURL_RUNUP_CAP);
                            let v_del = prestrafe_delivered(
                                runup_len,
                                params.accel,
                                params.maxspeed,
                                params.friction,
                                params.stopspeed,
                            );
                            // Cheap scout first — one mid-gain rollout with a generous tolerance — so the full
                            // envelope certify only runs where a landing is already near the target (else the
                            // pass is ~50× slower).
                            let scout_ok =
                                curl_land_point(bsp, takeoff, b.origin, v_del, psi, 10.0, &p).is_some_and(|land| {
                                    (land.xy() - b.origin.xy()).length() <= CURL_MISS_TOL * 2.5
                                        && (land.z - b.origin.z).abs() <= CURL_Z_TOL * 2.0
                                });
                            if scout_ok {
                                if let Some((v_req, gain, landing_speed_lo)) =
                                    certify_curl(bsp, takeoff, b.origin, psi, v_del, &p)
                                {
                                    // From-cell one committed run-up back *along the certified heading*, so the
                                    // runtime's run-up line is the one that was proven. Honest cost at the solved
                                    // takeoff speed the runtime will hold (not the equilibrium).
                                    let from_pt = takeoff - dir * runup_len;
                                    let cost = runup_len / ((MAX_SPEED + v_req) * 0.5) + airtime + CURL_COMMIT;
                                    solved = Some((takeoff, from_pt, v_req, gain, landing_speed_lo, cost));
                                    break 'psi;
                                }
                            }
                        }
                        t += GRID;
                        if t > t_max {
                            break;
                        }
                    }
                }
                let Some((takeoff, from_pt, v_req, gain, landing_speed_lo, cost)) = solved else {
                    continue;
                };
                let Some(start) = self.nearest_within(from_pt, GRID * 1.5, STEP_HEIGHT * 3.0) else {
                    continue;
                };
                if start == to || self.has_direct_link(start, to) {
                    continue;
                }
                let link = Link {
                    from: start,
                    to,
                    kind: LinkKind::SpeedJump,
                    cost,
                };
                let tr = SpeedJumpTraversal {
                    takeoff,
                    v_req,
                    airtime,
                    landing_speed_lo,
                    chained: false,
                    curl_gain: gain,
                    curl_entry_aim: Vec3::ZERO,
                    curl_switch_dist: 0.0,
                    curl_landing_aim: Vec3::ZERO,
                    ground_turn: None,
                };
                cands.push((horiz, link, tr)); // every certified curl; the per-cell cap trims below
            }
        }
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands.truncate(SPEED_JUMP_CURL_MAX_PER_CELL);
        out.extend(cands.into_iter().map(|(_, l, t)| (l, t)));
    }

    /// The speed-jump links leaving ledge cell `ledge` (the takeoff), appended to `out`.
    fn solve_speed_jumps_from(
        &self,
        bsp: &Bsp,
        ledge: CellId,
        params: SpeedJumpParams,
        k: f32,
        double_jump: bool,
        out: &mut Vec<(Link, SpeedJumpTraversal)>,
    ) {
        let a = self.cells[ledge as usize];
        if bsp.is_liquid_at(a.origin) {
            return; // submerged takeoff: can't jump (the jump input swims up)
        }
        let mut cands: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new(); // stand-start (v_req, link, tr)
        let mut cands_chained: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new(); // chained
                                                                                  // The most speed a chained entry can ever carry into a jump (the top band's floor); a jump
                                                                                  // needing more than this is unroutable even chained, so it bounds the chained target scan.
        let v_chain_max = BAND_FLOOR[NBANDS - 1] / SJ_MARGIN;
        for (dgx, dgy) in COMPASS {
            // Take off from a ledge edge (a runway only *helps* — a chained jump needs none).
            if self.has_ground_near(a.gx + dgx.signum(), a.gy + dgy.signum(), a.origin.z) {
                continue;
            }
            let runway = self.measure_runway(bsp, &a, dgx, dgy);
            let v_max = SPEED_JUMP_V_CAP.min(BHOP_EFF * attainable_speed(MAX_SPEED, runway, k));
            // Scan out to whatever the better of a self-contained runway or a carried entry reaches.
            let v_scan = v_max.max(v_chain_max);
            if v_scan * jump_airtime(0.0, params.gravity) <= JUMP_REACH + 1.0 {
                continue; // neither a runway nor a carried entry buys anything past a normal jump
            }
            let reach_cap = v_scan * jump_airtime(-SJ_MAX_DROP, params.gravity);
            let scan = ((reach_cap / GRID).ceil() as i32).max(1);
            let mut best: Option<(f32, Link, SpeedJumpTraversal)> = None; // stand-start
            let mut best_chained: Option<(f32, Link, SpeedJumpTraversal)> = None;
            for to in self.neighbors_within(a.gx, a.gy, scan) {
                if to == ledge {
                    continue;
                }
                let b = self.cells[to as usize];
                let (bgx, bgy) = (b.gx - a.gx, b.gy - a.gy);
                if (bgx.abs() <= 1 && bgy.abs() <= 1) || dir_bucket(bgx, bgy) != dir_bucket(dgx, dgy) {
                    continue;
                }
                let dz = b.origin.z - a.origin.z;
                let horiz = (b.origin.xy() - a.origin.xy()).length();
                if !(-SJ_MAX_DROP..=JUMP_APEX).contains(&dz) || horiz <= JUMP_REACH {
                    continue;
                }
                // Skip what a double jump already covers (when enabled), and any existing direct link.
                if (double_jump && horiz <= DOUBLE_JUMP_REACH && dz <= DOUBLE_JUMP_APEX)
                    || self.has_direct_link(ledge, to)
                {
                    continue;
                }
                let airtime = jump_airtime(dz, params.gravity);
                let v_req = v_required(horiz, dz, params.gravity);
                if airtime <= 0.0 || v_req * SJ_MARGIN > v_scan {
                    continue; // beyond even a carried entry
                }
                // Arc clearance and a landing with room to slide out — required for either form.
                let steps = ((horiz / 24.0).ceil() as i32).max(8);
                let depth_cols = (SJ_LANDING_DEPTH / GRID).ceil() as i32;
                let landing_ok = (1..=depth_cols)
                    .all(|i| self.has_ground_near(b.gx + dgx.signum() * i, b.gy + dgy.signum() * i, b.origin.z));
                if !arc_clear_peak(bsp, a.origin, b.origin, JUMP_APEX, steps) || !landing_ok {
                    continue;
                }
                // Stand-start form: a runway long enough behind the ledge to build v_req from a walk.
                if v_req * SJ_MARGIN <= v_max {
                    let need = runway_len_for(v_req * SJ_MARGIN, MAX_SPEED, k);
                    let dir = Vec3::new(dgx.signum() as f32, dgy.signum() as f32, 0.0).normalize_or_zero();
                    if let Some(start) = self.nearest_within(a.origin - dir * need, GRID * 1.5, STEP_HEIGHT * 3.0) {
                        if start != to {
                            let cost = runway_time(v_req * SJ_MARGIN, MAX_SPEED, k) + airtime + 1.0;
                            let link = Link {
                                from: start,
                                to,
                                kind: LinkKind::SpeedJump,
                                cost,
                            };
                            let tr = SpeedJumpTraversal {
                                takeoff: a.origin,
                                v_req,
                                airtime,
                                landing_speed_lo: 0.0,
                                chained: false,
                                curl_gain: 0.0,
                                curl_entry_aim: Vec3::ZERO,
                                curl_switch_dist: 0.0,
                                curl_landing_aim: Vec3::ZERO,
                                ground_turn: None,
                            };
                            if best.is_none_or(|(bv, _, _)| v_req < bv) {
                                best = Some((v_req, link, tr));
                            }
                            continue; // a self-contained jump covers this target; no chained dup
                        }
                    }
                }
                // Chained form: no runway of its own — take off from the ledge itself, feasible only
                // when a prior jump delivers ≥ v_req (the banded planner proves it; unbanded queries
                // price it away). Bounded to what the top band can carry.
                if v_req * SJ_MARGIN <= v_chain_max {
                    let cost = airtime + 1.0;
                    let link = Link {
                        from: ledge,
                        to,
                        kind: LinkKind::SpeedJump,
                        cost,
                    };
                    let tr = SpeedJumpTraversal {
                        takeoff: a.origin,
                        v_req,
                        airtime,
                        landing_speed_lo: 0.0,
                        chained: true,
                        curl_gain: 0.0,
                        curl_entry_aim: Vec3::ZERO,
                        curl_switch_dist: 0.0,
                        curl_landing_aim: Vec3::ZERO,
                        ground_turn: None,
                    };
                    if best_chained.is_none_or(|(bv, _, _)| v_req < bv) {
                        best_chained = Some((v_req, link, tr));
                    }
                }
            }
            if let Some(c) = best {
                cands.push(c);
            }
            if let Some(c) = best_chained {
                cands_chained.push(c);
            }
        }
        // Keep the cheapest-entry candidates in each pool (they never evict each other — separate
        // budgets), then splice link + traversal into the shared output.
        let mut keep_cheapest = |mut cs: Vec<(f32, Link, SpeedJumpTraversal)>, cap: usize| {
            cs.sort_by(|x, y| x.0.total_cmp(&y.0));
            cs.truncate(cap);
            out.extend(cs.into_iter().map(|(_, l, t)| (l, t)));
        };
        keep_cheapest(cands, SPEED_JUMP_MAX_PER_CELL);
        keep_cheapest(cands_chained, SPEED_JUMP_CHAINED_MAX_PER_CELL);
    }

    /// Measure the straight, flat, hop-wide runway feeding ledge cell `a` from behind (opposite the
    /// jump direction): walk grid columns back while each has a cell within `STEP_HEIGHT`, hop
    /// headroom, and ground in both perpendicular columns (so the air-strafe weave stays on floor).
    fn measure_runway(&self, bsp: &Bsp, a: &Cell, dgx: i32, dgy: i32) -> f32 {
        let (bx, by) = (-dgx.signum(), -dgy.signum());
        if bx == 0 && by == 0 {
            return 0.0;
        }
        let step_len = GRID * (((bx * bx + by * by) as f32).sqrt());
        let (px, py) = (-by, bx); // perpendicular grid direction
        let (mut gx, mut gy, mut z, mut len) = (a.gx, a.gy, a.origin.z, 0.0);
        while len < RUNWAY_MAX {
            let (ngx, ngy) = (gx + bx, gy + by);
            let Some(cid) = self.cell_near(ngx, ngy, z) else {
                break;
            };
            let c = self.cells[cid as usize].origin;
            if bsp.is_solid(c + Vec3::new(0.0, 0.0, JUMP_APEX))
                || self.cell_near(ngx + px, ngy + py, c.z).is_none()
                || self.cell_near(ngx - px, ngy - py, c.z).is_none()
            {
                break;
            }
            len += step_len;
            (gx, gy, z) = (ngx, ngy, c.z);
        }
        len
    }
}

/// What a curl probe saw. Every field is an *answer to a question the harness asked*, which is why
/// they're named rather than positional: a bare `(f32, Option<(f32, f32)>, Vec<(f32, Vec3)>)` needs
/// this comment read before it can be indexed at all.
pub struct CurlProbe {
    /// The takeoff speed the run-up actually delivers.
    pub v_deliver: f32,
    /// The certified envelope, if one lands: the gentlest gain that works, and the low corner of the
    /// speed envelope — what the runtime must at least deliver. `None` when nothing certifies, which
    /// is the case the harness is usually asking about.
    pub certified: Option<(f32, f32, f32)>,
    /// Where the centre corner lands, per gain tried. The miss distances are the *why* behind a
    /// `certified: None`.
    pub landings: Vec<(f32, Vec3)>,
}

impl NavGraph {
    /// Debug probe (harness): from `takeoff` along `psi0` (degrees) with the speed a `runway` delivers,
    /// report the predicted takeoff speed, whether the full envelope certifies, and per-gain the
    /// center-corner landing point — so the harness can see *why* a curl candidate is/ isn't emitted.
    pub fn curl_probe(
        &self,
        bsp: &Bsp,
        takeoff: Vec3,
        target: Vec3,
        psi0: f32,
        runway: f32,
        params: SpeedJumpParams,
    ) -> CurlProbe {
        let p = PmParams {
            gravity: params.gravity,
            accel: params.accel,
            friction: params.friction,
            stopspeed: params.stopspeed,
            maxspeed: params.maxspeed,
        };
        let v_deliver = prestrafe_delivered(runway, params.accel, params.maxspeed, params.friction, params.stopspeed);
        CurlProbe {
            v_deliver,
            certified: certify_curl(bsp, takeoff, target, psi0, v_deliver, &p),
            landings: CURL_GAINS
                .iter()
                .map(|&gain| {
                    (
                        gain,
                        curl_land_point(bsp, takeoff, target, v_deliver, psi0, gain, &p).unwrap_or(Vec3::ZERO),
                    )
                })
                .collect(),
        }
    }
}

/// Roll a curl and return the landing origin (or `None` if it never touched down after the leap) — the
/// probe variant of [`curl_lands`], without the accept tolerances.
fn curl_land_point(bsp: &Bsp, takeoff: Vec3, target: Vec3, v0: f32, psi: f32, gain: f32, p: &PmParams) -> Option<Vec3> {
    let dt = CURL_DT;
    let amax = air_accel_max(p.accel, p.maxspeed, dt);
    let (s0, c0) = psi.to_radians().sin_cos();
    let mut s = PmState {
        origin: takeoff,
        vel: Vec3::new(v0 * c0, v0 * s0, 0.0),
        on_ground: true,
        jump_held: false,
    };
    for tick in 0..CURL_MAX_TICKS {
        let cmd = if tick == 0 {
            Cmd {
                view_yaw: psi,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: true,
            }
        } else {
            let v_xy = s.vel.xy();
            let st = air_correct(v_xy, yaw_of(target.xy() - s.origin.xy()), amax, dt, gain);
            Cmd {
                view_yaw: st.view_yaw,
                forward: st.forward,
                side: st.side,
                jump: false,
            }
        };
        pm_step(bsp, &mut s, &cmd, p, dt);
        if tick > 3 && s.on_ground {
            return Some(s.origin);
        }
    }
    None
}

/// Certify a curl from `takeoff` onto `target`: the run-up delivers ~`v_deliver` ups along `psi0` (the
/// corridor heading, degrees); find the gentlest [`CURL_GAINS`] gain whose `air_correct` arc lands the
/// target cell across the whole delivered-speed × launch-heading × cadence
/// envelope. Returns `(v_req, gain, landing_speed_lo)` — `v_req` is the
/// takeoff envelope's low corner and `landing_speed_lo` is the minimum clean
/// touchdown carry the planner may credit — or `None`.
fn certify_curl(
    bsp: &Bsp,
    takeoff: Vec3,
    target: Vec3,
    psi0: f32,
    v_deliver: f32,
    p: &PmParams,
) -> Option<(f32, f32, f32)> {
    let (s0, c0) = psi0.to_radians().sin_cos();
    // The runtime leaps on crossing the takeoff *line*, up to a lip-reach *before* this point (the frame
    // progress < LIP_REACH, at ~6u/tick), so every corner is proven from both leap points.
    let early = takeoff - Vec3::new(c0, s0, 0.0) * CURL_LIP_REACH;
    // Solve the takeoff *speed*. Certifying only at what the run-up maxes out to (the ~484 prestrafe
    // equilibrium, 327u of flat reach) makes every moderate gap uncertifiable — it overshoots. A human
    // holds a controlled speed instead (396-416 across the recorded demos), so scan a ladder from the
    // ballistic floor up to what the run-up can deliver and take the *lowest* speed whose whole envelope
    // lands; the runtime's takeoff regime then holds exactly this (see `bhop`'s hold band).
    let horiz = (target.xy() - takeoff.xy()).length();
    let dz = target.z - takeoff.z;
    let v_floor = v_required(horiz, dz, p.gravity);
    let v_ceil = v_deliver * CURL_V_LO_FRAC;
    if !v_floor.is_finite() || v_floor > v_ceil {
        return None;
    }
    let steps = (((v_ceil - v_floor) / CURL_V_STEP).ceil() as i32).clamp(1, 24);
    for i in 0..=steps {
        let v = (v_floor + i as f32 * CURL_V_STEP).min(v_ceil);
        // Cheap scout at this speed before the full envelope (keeps rejected candidates ~1 rollout each).
        let scout = curl_land_point(bsp, takeoff, target, v, psi0, 10.0, p).is_some_and(|land| {
            (land.xy() - target.xy()).length() <= CURL_MISS_TOL * 2.5 && (land.z - target.z).abs() <= CURL_Z_TOL * 2.0
        });
        if !scout {
            continue;
        }
        // Envelope: both leap points × the speed band the runtime holds × a ±heading guard.
        let (lo, hi) = (v * (1.0 - CURL_V_HOLD_TOL), v * (1.0 + CURL_V_HOLD_TOL));
        let corners = [
            (takeoff, hi, 0.0),
            (takeoff, lo, 0.0),
            (early, hi, 0.0),
            (early, lo, 0.0),
            (takeoff, v, CURL_PSI_TOL),
            (early, v, -CURL_PSI_TOL),
        ];
        for &gain in &CURL_GAINS {
            let mut landing_speed_lo = f32::INFINITY;
            let mut passed = true;
            for &dt in &GT_DT_CLASSES {
                for &(tk, v0, dp) in &corners {
                    let Some(land) = curl_lands(bsp, tk, target, v0, psi0 + dp, gain, p, dt) else {
                        passed = false;
                        break;
                    };
                    landing_speed_lo = landing_speed_lo.min(land.vel.xy().length());
                }
                if !passed {
                    break;
                }
            }
            if passed {
                return Some((v, gain, landing_speed_lo));
            }
        }
    }
    None
}

/// Roll one curl and test whether it lands on the target cell: `pm_step` from `takeoff` seeded at
/// (`v0`, `psi` degrees), leap on tick 0, then per-tick `air_correct` toward the target at `gain` — the
/// exact runtime air policy. Accepts the first touchdown after the leap that resolves to the target
/// within tolerance; rejects a heading that crosses the target bearing mid-flight (an overshoot the
/// held-sign air-strafe diverges from) or an arc that falls well below / flies past the target.
fn curl_lands(
    bsp: &Bsp,
    takeoff: Vec3,
    target: Vec3,
    v0: f32,
    psi: f32,
    gain: f32,
    p: &PmParams,
    dt: f32,
) -> Option<PmState> {
    use crate::pmove::pm_step_report;
    let amax = air_accel_max(p.accel, p.maxspeed, dt);
    let (s0, c0) = psi.to_radians().sin_cos();
    let mut s = PmState {
        origin: takeoff,
        vel: Vec3::new(v0 * c0, v0 * s0, 0.0),
        on_ground: true,
        jump_held: false,
    };
    let mut prev_sign = 0.0f32;
    for tick in 0..CURL_MAX_TICKS {
        let cmd = if tick == 0 {
            Cmd {
                view_yaw: psi,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: true,
            }
        } else {
            let v_xy = s.vel.xy();
            let bearing = yaw_of(target.xy() - s.origin.xy());
            let err = wrap180(bearing - yaw_of(v_xy));
            // A mid-flight bearing-sign flip is a real overshoot the runtime would diverge from — but
            // once abeam of a target it's about to land on, the bearing swings fast and flips benignly,
            // so only treat it as divergence while still well short of the target.
            let far = (s.origin.xy() - target.xy()).length() > CURL_MISS_TOL * 1.5;
            if far && prev_sign != 0.0 && err.signum() != prev_sign && err.abs() > 2.0 {
                return None;
            }
            prev_sign = err.signum();
            let st = air_correct(v_xy, bearing, amax, dt, gain);
            Cmd {
                view_yaw: st.view_yaw,
                forward: st.forward,
                side: st.side,
                jump: false,
            }
        };
        let cmd = Cmd {
            forward: cmd.forward.round(),
            side: cmd.side.round(),
            ..cmd
        };
        let before = bsp.hull1_trace(s.origin, s.origin);
        if before.start_solid || before.all_solid {
            return None;
        }
        let report = pm_step_report(bsp, &mut s, &cmd, p, dt);
        let after = bsp.hull1_trace(s.origin, s.origin);
        if report.wall_contact || after.start_solid || after.all_solid {
            return None;
        }
        if s.vel.z < 0.0 && s.origin.z < target.z - 100.0 {
            return None; // fell past the target's level — undershoot
        }
        if tick > 3 && s.on_ground {
            return ((s.origin.xy() - target.xy()).length() <= CURL_MISS_TOL
                && (s.origin.z - target.z).abs() <= CURL_Z_TOL)
                .then_some(s);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Chained ground-turn curls
// ---------------------------------------------------------------------------
//
// A leap whose flight-time budget closes ONLY with carried entry speed (no
// local run-up delivers it — `prestrafe_delivered` saturates near 430 while
// the flight needs ~470) and whose launch heading is not the corridor
// heading (the grounded rotation happens in the final `turn_dist` before the
// jump; rotating after a lip launch provably cannot close the budget: air
// accel gives ~3-4 deg/tick at these speeds and gravity caps the flight at
// ~30 ticks). Both the certifier below and the live executor drive
// [`ground_turn_ground_aim`]/[`ground_turn_should_launch`]/
// [`ground_turn_air_aim`] from the same stored [`GroundTurnCurl`], so the
// proven contract is the flown contract. Fail closed: the executor checks
// the live entry state against the stored envelope and replans when outside.

/// Contract version stamped into every generated [`GroundTurnCurl`].
pub const GROUND_TURN_VERSION: u16 = 1;
/// Self-contained blended-runway contract; distinct from the original
/// carried-speed switch-at-distance controller.
pub const RUNWAY_TURN_VERSION: u16 = 2;

/// The fake-bot integer-millisecond cadence classes the live artifact
/// accepts (see the live20 provenance): every envelope corner must certify
/// at all three.
const GT_DT_CLASSES: [f32; 3] = [0.019, 0.020, 0.021];
/// Minimum measured (stepped, lax) runway behind the takeoff.
const GT_MIN_RUNWAY: f32 = 64.0;
/// Committed run-up lengths: the link's from-cell sits this far back along
/// the corridor (or as far as the runway allows); both a long and a short
/// variant are searched (see the loop at the ledge scan).
const GT_RUNUP_CAP: f32 = 320.0;
const GT_RUNUP_SHORT: f32 = 64.0;
/// Carried entry speeds searched, low first — v_req is the lowest that
/// certifies its whole envelope. Chosen so the envelope FLOOR
/// (`v * (1 - GT_ENTRY_V_TOL)`) lands exactly on a planner band floor
/// (430, 490): a certified envelope the banded planner can actually prove
/// (a floor 1 u/s above a band edge is a link no route can ever take).
const GT_ENTRY_SPEEDS: [f32; 2] = [430.0 / (1.0 - GT_ENTRY_V_TOL), 490.0 / (1.0 - GT_ENTRY_V_TOL)];
/// Entry-speed envelope half-width (fraction) certified around v_req.
const GT_ENTRY_V_TOL: f32 = 0.02;
/// Entry-yaw envelope half-width (degrees) certified around the corridor.
const GT_ENTRY_YAW_TOL: f32 = 12.0;
/// Launch-heading offsets (degrees off the corridor axis, signed toward the
/// target side) sampled by the generator.
const GT_LAUNCH_OFFSETS: [f32; 6] = [8.0, 16.0, 24.0, 35.0, 42.0, 50.0];
/// Ground-rotation window distances sampled.
const GT_TURN_DISTS: [f32; 4] = [48.0, 64.0, 96.0, 224.0];
/// Yaw slack: the jump may fire this many degrees short of `launch_yaw`.
const GT_YAW_SLACKS: [f32; 3] = [3.0, 8.0, 18.0];
/// Launch-frame and air-curl gains sampled.
const GT_LAUNCH_GAINS: [f32; 3] = [8.0, 16.0, 32.0];
const GT_AIR_GAINS: [f32; 3] = [8.0, 64.0, 256.0];
/// Lateral runway-aim offsets (perpendicular to the corridor) sampled.
const GT_RUNWAY_LATERALS: [f32; 3] = [0.0, -8.0, 8.0];
/// Target offset band off the corridor axis (degrees) — beyond the plain
/// curl pass's reach, a genuine turn-then-leap. The high edge admits the
/// DM3 gap-1 class (~111 deg off the eastbound corridor).
const GT_OFF_LO: f32 = 40.0;
const GT_OFF_HI: f32 = 115.0;
/// Hold-phase gate distances sampled along the launch heading before the
/// curl engages (0 = immediate curl, no hold phase). A hold lets the
/// flight clear a near wall corner before turning onto the target.
const GT_GATE_DISTS: [f32; 3] = [0.0, 96.0, 160.0];
/// Tick caps for the grounded approach and the flight.
const GT_SETUP_TICK_CAP: usize = 45;
const GT_FLIGHT_TICK_CAP: usize = 60;
/// Per-ledge emission cap (cheapest-cost first), own budget like the plain
/// curl pass — never evicts a straight jump.
const GT_MAX_PER_CELL: usize = 16;
/// Global per-target cap after dedup (see the source-diversity note at the
/// splice site).
const GT_TARGET_MAX: usize = 16;
/// Candidate targets examined per (ledge, corridor direction). Effectively
/// uncapped: every candidate first passes the cheap canonical-scout gate
/// below, so an infeasible one costs a handful of rollouts, not a full
/// lattice — the cap only bounds pathological geometry.
const GT_TARGETS_PER_DIR: usize = 96;
/// Takeoff box half-widths around the ledge-cell origin.
const GT_BOX_HALF: f32 = 28.0;

/// Velocity yaw mapped to [0,360): the ground rotation crosses +-180 in the
/// middle of the turn, so gates compare in a wrap-free domain.
pub fn yaw360_of(v: Vec2) -> f32 {
    yaw_of(v).rem_euclid(360.0)
}

/// Grounded steering waypoint for the current position: the runway line
/// while farther than `turn_dist` from the takeoff, else a far point along
/// the launch heading (rotating the carried velocity before the jump).
pub fn ground_turn_ground_aim(origin: Vec3, takeoff: Vec3, gt: &GroundTurnCurl) -> Vec2 {
    if gt.blended_runway {
        let (rs, rc) = gt.runway_yaw.to_radians().sin_cos();
        let runway_dir = Vec2::new(rc, rs);
        let remaining = (takeoff.xy() - origin.xy()).dot(runway_dir);
        let u = ((gt.turn_dist - remaining) / gt.turn_dist.max(1.0)).clamp(0.0, 1.0);
        let smooth = u * u * (3.0 - 2.0 * u);
        let bearing = gt.runway_yaw + wrap180(gt.launch_yaw - gt.runway_yaw) * smooth;
        let (s, c) = bearing.to_radians().sin_cos();
        return origin.xy() + Vec2::new(c, s) * 512.0;
    }
    if (origin.xy() - takeoff.xy()).length() > gt.turn_dist {
        gt.runway_aim.xy()
    } else {
        let (s, c) = gt.launch_yaw.to_radians().sin_cos();
        takeoff.xy() + Vec2::new(c, s) * 512.0
    }
}

/// Exact grounded command for a certified ground-turn traversal.  The build-time rollout and the
/// live executor call this same function so the last-moment sigma reset, speed hold and rounded
/// usercmd cannot drift apart.
#[allow(clippy::too_many_arguments)]
pub fn ground_turn_ground_cmd(
    origin: Vec3,
    vel_xy: Vec2,
    takeoff: Vec3,
    gt: &GroundTurnCurl,
    sigma: &mut f32,
    accel: f32,
    maxspeed: f32,
    dt: f32,
) -> Cmd {
    use crate::strafe::ground_prestrafe;

    let aim = ground_turn_ground_aim(origin, takeoff, gt);
    let bearing = yaw_of(aim - origin.xy());
    let st = if gt.blended_runway {
        let (rs, rc) = gt.runway_yaw.to_radians().sin_cos();
        let remaining = (takeoff.xy() - origin.xy()).dot(Vec2::new(rc, rs));
        if remaining <= 40.0 {
            ground_prestrafe(vel_xy, bearing, 0.0, accel * maxspeed * dt, maxspeed)
        } else if vel_xy.length() > gt.hold_speed * 1.03 {
            return Cmd {
                view_yaw: bearing,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: false,
            };
        } else {
            let st = ground_prestrafe(vel_xy, bearing, *sigma, accel * maxspeed * dt, maxspeed);
            *sigma = st.sigma;
            st
        }
    } else {
        let st = ground_prestrafe(vel_xy, bearing, *sigma, accel * maxspeed * dt, maxspeed);
        *sigma = st.sigma;
        st
    };
    Cmd {
        view_yaw: st.view_yaw,
        forward: st.forward.round(),
        side: st.side.round(),
        jump: false,
    }
}

/// Exact launch tick for a certified ground-turn traversal.
pub fn ground_turn_launch_cmd(
    vel_xy: Vec2,
    bearing: f32,
    gt: &GroundTurnCurl,
    accel: f32,
    maxspeed: f32,
    dt: f32,
) -> Cmd {
    let st = air_correct(vel_xy, bearing, air_accel_max(accel, maxspeed, dt), dt, gt.launch_gain);
    Cmd {
        view_yaw: st.view_yaw,
        forward: st.forward.round(),
        side: st.side.round(),
        jump: true,
    }
}

/// Exact airborne tick for a certified ground-turn traversal.
pub fn ground_turn_air_cmd(origin: Vec3, vel_xy: Vec2, gt: &GroundTurnCurl, accel: f32, maxspeed: f32, dt: f32) -> Cmd {
    let (aim, gain) = ground_turn_air_aim(origin, gt);
    let st = air_correct(
        vel_xy,
        yaw_of(aim - origin.xy()),
        air_accel_max(accel, maxspeed, dt),
        dt,
        gain,
    );
    Cmd {
        view_yaw: st.view_yaw,
        forward: st.forward.round(),
        side: st.side.round(),
        jump: false,
    }
}

/// Fire the jump? First grounded tick inside the takeoff box, at platform
/// height, with the carried velocity rotated at least to `yaw_min`.
pub fn ground_turn_should_launch(origin: Vec3, vel_xy: Vec2, on_ground: bool, gt: &GroundTurnCurl) -> bool {
    if gt.blended_runway {
        let takeoff = (gt.box_min + gt.box_max) * 0.5;
        let (rs, rc) = gt.runway_yaw.to_radians().sin_cos();
        let remaining = (takeoff.xy() - origin.xy()).dot(Vec2::new(rc, rs));
        let speed = vel_xy.length();
        return on_ground
            && remaining <= gt.lip_reach
            && (takeoff.xy() - origin.xy()).length() <= 48.0
            && (origin.z - gt.box_max.z).abs() <= 2.0
            && (speed - gt.hold_speed).abs() <= gt.hold_speed * 0.06
            && wrap180(yaw_of(vel_xy) - gt.launch_yaw).abs() <= 35.0;
    }
    on_ground
        && origin.x >= gt.box_min.x
        && origin.x <= gt.box_max.x
        && origin.y >= gt.box_min.y
        && origin.y <= gt.box_max.y
        && origin.z >= gt.box_min.z
        && yaw360_of(vel_xy) >= gt.yaw_min
}

/// Airborne pursuit point: `hold_aim` while still on the near side of the
/// gate plane, else the landing aim. A zero gate normal disables the hold
/// phase entirely (immediate curl).
pub fn ground_turn_air_aim(origin: Vec3, gt: &GroundTurnCurl) -> (Vec2, f32) {
    let held = gt.gate_normal != Vec3::ZERO && (origin - gt.gate_point).dot(gt.gate_normal) > 0.0;
    if held {
        (gt.hold_aim.xy(), gt.air_gain)
    } else {
        (gt.landing_aim.xy(), gt.air_gain)
    }
}

/// Live entry check (fail closed): grounded arrival at the link source with
/// speed and yaw inside the certified envelope.
pub fn ground_turn_entry_ok(speed: f32, vel_xy: Vec2, on_ground: bool, gt: &GroundTurnCurl) -> bool {
    let yaw = yaw360_of(vel_xy);
    on_ground
        && speed >= gt.entry_speed_lo
        && speed <= gt.entry_speed_hi
        && if gt.entry_yaw_lo <= gt.entry_yaw_hi {
            yaw >= gt.entry_yaw_lo && yaw <= gt.entry_yaw_hi
        } else {
            yaw >= gt.entry_yaw_lo || yaw <= gt.entry_yaw_hi
        }
}

/// One grounded corrective command that moves a nearby entry state into this contract's certified
/// speed/yaw envelope after normal QW friction. Returns `None` when the envelope cannot be reached
/// in one tick at stock-style ground acceleration. The target is inset slightly from every boundary
/// so fake-client integer command quantization cannot turn a valid adjustment into an envelope miss.
pub fn ground_turn_entry_adjust_cmd(
    vel_xy: Vec2,
    on_ground: bool,
    gt: &GroundTurnCurl,
    friction: f32,
    stopspeed: f32,
    accel: f32,
    maxspeed: f32,
    dt: f32,
) -> Option<Cmd> {
    if !on_ground || dt <= 0.0 || accel <= 0.0 || maxspeed <= 0.0 {
        return None;
    }
    let after_friction = apply_friction(vel_xy, friction, stopspeed, dt);
    let speed_margin = ((gt.entry_speed_hi - gt.entry_speed_lo) * 0.05).clamp(0.25, 1.0);
    let speed_lo = gt.entry_speed_lo + speed_margin;
    let speed_hi = gt.entry_speed_hi - speed_margin;
    if speed_lo > speed_hi {
        return None;
    }
    let target_speed = after_friction.length().clamp(speed_lo, speed_hi);

    let yaw = yaw360_of(after_friction);
    let yaw_margin = 0.25;
    let lo = (gt.entry_yaw_lo + yaw_margin).rem_euclid(360.0);
    let hi = (gt.entry_yaw_hi - yaw_margin).rem_euclid(360.0);
    let inside = if lo <= hi {
        yaw >= lo && yaw <= hi
    } else {
        yaw >= lo || yaw <= hi
    };
    let target_yaw = if inside {
        yaw
    } else if wrap180(lo - yaw).abs() <= wrap180(hi - yaw).abs() {
        lo
    } else {
        hi
    };
    let (sy, cy) = target_yaw.to_radians().sin_cos();
    let target = Vec2::new(cy, sy) * target_speed;
    let delta = target - after_friction;
    let delta_len = delta.length();
    if delta_len < 0.01 {
        return None;
    }
    let wishspeed = delta_len / (accel * dt);
    if wishspeed > maxspeed {
        return None;
    }
    let wishdir = delta / delta_len;
    let predicted = apply_groundaccel(after_friction, wishdir, wishspeed, accel, dt);
    ground_turn_entry_ok(predicted.length(), predicted, true, gt).then_some(Cmd {
        view_yaw: yaw_of(wishdir),
        forward: wishspeed,
        side: 0.0,
        jump: false,
    })
}

/// One certified rollout: grounded weave from `entry` up the runway,
/// rotation, launch, curl, first touchdown resolving to `to` — zero wall
/// contact, zero start-solid, no fall. Returns (elapsed, landing state).
fn ground_turn_rolls(
    graph: &NavGraph,
    bsp: &Bsp,
    entry: PmState,
    dt: f32,
    takeoff: Vec3,
    gt: &GroundTurnCurl,
    to: CellId,
    p: &PmParams,
) -> Option<(f32, PmState)> {
    ground_turn_rolls_tol(graph, bsp, entry, dt, takeoff, gt, to, p, 0.0)
}

/// [`ground_turn_rolls`] with a landing tolerance: `accept_near > 0` also
/// accepts a touchdown within that XY distance of the target cell (the
/// canonical-scout feasibility gate; certification always uses exact).
#[allow(clippy::too_many_arguments)]
fn ground_turn_rolls_tol(
    graph: &NavGraph,
    bsp: &Bsp,
    entry: PmState,
    dt: f32,
    takeoff: Vec3,
    gt: &GroundTurnCurl,
    to: CellId,
    p: &PmParams,
    accept_near: f32,
) -> Option<(f32, PmState)> {
    use crate::pmove::pm_step_report;
    let solid = |o: Vec3| {
        let tr = bsp.hull1_trace(o, o);
        tr.start_solid || tr.all_solid
    };
    let mut s = entry;
    let mut sigma = 0.0;
    let mut elapsed = 0.0;
    let mut airborne_streak = 0usize;
    let floor_z = entry.origin.z.min(takeoff.z) - 80.0;
    let mut setup = 0usize;
    loop {
        if ground_turn_should_launch(s.origin, s.vel.xy(), s.on_ground, gt) {
            break;
        }
        if setup >= GT_SETUP_TICK_CAP {
            return None;
        }
        let cmd = ground_turn_ground_cmd(s.origin, s.vel.xy(), takeoff, gt, &mut sigma, p.accel, p.maxspeed, dt);
        if solid(s.origin) {
            return None;
        }
        let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
        if rep.wall_contact || solid(s.origin) || s.origin.z < floor_z {
            return None;
        }
        elapsed += dt;
        setup += 1;
        if s.on_ground {
            airborne_streak = 0;
        } else {
            airborne_streak += 1;
            if airborne_streak >= 3 {
                return None; // left the runway floor
            }
        }
        if s.jump_held {
            return None;
        }
    }
    // Launch tick.
    let aim = ground_turn_ground_aim(s.origin, takeoff, gt);
    let cmd = ground_turn_launch_cmd(s.vel.xy(), yaw_of(aim - s.origin.xy()), gt, p.accel, p.maxspeed, dt);
    if solid(s.origin) {
        return None;
    }
    let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
    if rep.wall_contact || solid(s.origin) || s.on_ground {
        return None;
    }
    elapsed += dt;
    for _ in 0..GT_FLIGHT_TICK_CAP {
        let cmd = ground_turn_air_cmd(s.origin, s.vel.xy(), gt, p.accel, p.maxspeed, dt);
        if solid(s.origin) {
            return None;
        }
        let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
        if rep.wall_contact || solid(s.origin) || s.origin.z < floor_z {
            return None;
        }
        elapsed += dt;
        if s.on_ground {
            let land = graph.nearest_within(s.origin, 24.0, 2.0);
            let target = graph.cells[to as usize].origin;
            let near = accept_near > 0.0
                && (s.origin.xy() - target.xy()).length() <= accept_near
                && (s.origin.z - target.z).abs() <= CURL_Z_TOL;
            return if land == Some(to) || near {
                Some((elapsed, s))
            } else {
                None
            };
        }
    }
    None
}

impl NavGraph {
    /// Stepped, hop-wide-relaxed runway measure: walk grid columns back from
    /// `a` opposite `(dgx,dgy)` while a cell exists within step height and
    /// jump headroom — no side-column requirement (the certified weave holds
    /// the corridor line; certification, not corridor width, is the proof).
    fn measure_runway_lax(&self, bsp: &Bsp, a: &Cell, dgx: i32, dgy: i32) -> f32 {
        let (bx, by) = (-dgx.signum(), -dgy.signum());
        if bx == 0 && by == 0 {
            return 0.0;
        }
        let step_len = GRID * (((bx * bx + by * by) as f32).sqrt());
        let (mut gx, mut gy, mut z, mut len) = (a.gx, a.gy, a.origin.z, 0.0);
        while len < RUNWAY_MAX {
            let (ngx, ngy) = (gx + bx, gy + by);
            let Some(cid) = self.cell_near(ngx, ngy, z) else {
                break;
            };
            let c = self.cells[cid as usize].origin;
            if bsp.is_solid(c + Vec3::new(0.0, 0.0, JUMP_APEX)) {
                break;
            }
            len += step_len;
            (gx, gy, z) = (ngx, ngy, c.z);
        }
        len
    }

    /// Certify one ground-turn profile across its whole envelope: entry
    /// speeds {lo, mid, hi}, entry yaws {-tol, 0, +tol}, all three cadence
    /// classes. Returns the worst (largest) certified elapsed and the
    /// minimum horizontal landing speed across the envelope.
    fn certify_ground_turn(
        &self,
        bsp: &Bsp,
        entry_origin: Vec3,
        entry_yaw0: f32,
        v_req: f32,
        entry_v_tol: f32,
        entry_yaw_tol: f32,
        takeoff: Vec3,
        gt: &GroundTurnCurl,
        to: CellId,
        p: &PmParams,
    ) -> Option<(f32, f32, f32)> {
        let mut worst = 0.0f32;
        let mut land_lo = f32::INFINITY;
        let mut land_yaw = 0.0f32;
        let speeds = [v_req * (1.0 - entry_v_tol), v_req, v_req * (1.0 + entry_v_tol)];
        let yaws = [entry_yaw0 - entry_yaw_tol, entry_yaw0, entry_yaw0 + entry_yaw_tol];
        for &dt in &GT_DT_CLASSES {
            for &v in &speeds {
                for &yaw in &yaws {
                    let (sy, cy) = yaw.to_radians().sin_cos();
                    let entry = PmState {
                        origin: entry_origin,
                        vel: Vec3::new(v * cy, v * sy, 0.0),
                        on_ground: true,
                        jump_held: false,
                    };
                    let (elapsed, land) = ground_turn_rolls(self, bsp, entry, dt, takeoff, gt, to, p)?;
                    worst = worst.max(elapsed);
                    land_lo = land_lo.min(land.vel.xy().length());
                    if dt == GT_DT_CLASSES[1] && v == v_req && yaw == entry_yaw0 {
                        land_yaw = yaw360_of(land.vel.xy());
                    }
                }
            }
        }
        Some((worst, land_lo, land_yaw))
    }

    /// The chained ground-turn curl links leaving ledge cell `ledge`:
    /// candidate targets sit far off the corridor axis (beyond the plain
    /// curl pass), the flight needs carried speed no local run-up delivers,
    /// and the launch needs a grounded rotation. Bounded lattice per
    /// candidate; emitted `chained` so the banded planner only takes the
    /// link when the entry band carries `v_req`. Own per-cell budget.
    ///
    /// `entry_speeds` is the carried-entry ladder sampled by both the
    /// canonical-scout gate and the full lattice search (low-to-high order
    /// not required; the reach/floor bounds below use the max of the
    /// slice). The production call site passes [`GT_ENTRY_SPEEDS`]
    /// unchanged; a search harness (e.g. `bin/gt_search.rs`) may pass a
    /// different ladder — e.g. a lower-entry exploration — without
    /// otherwise touching this solver. Note [`GT_ENTRY_SPEEDS`] itself is
    /// pre-divided by `(1.0 - GT_ENTRY_V_TOL)` so its envelope's low corner
    /// lands exactly on a planner band floor; callers passing raw target
    /// entry speeds (not band-floor-aligned) get a correctly-certified but
    /// not necessarily banded-plannable link — fine for search/calibration,
    /// not for direct production splicing.
    pub fn solve_chained_ground_turn_from(
        &self,
        bsp: &Bsp,
        ledge: CellId,
        params: SpeedJumpParams,
        entry_speeds: &[f32],
        out: &mut Vec<(Link, SpeedJumpTraversal)>,
    ) {
        let a = self.cells[ledge as usize];
        if bsp.is_liquid_at(a.origin) {
            return;
        }
        // Build-time diagnostics for one ledge: RTX_GT_DEBUG_LEDGE=<cell id>.
        let dbg = std::env::var("RTX_GT_DEBUG_LEDGE")
            .ok()
            .and_then(|s| s.parse::<CellId>().ok())
            == Some(ledge);
        let p = PmParams {
            gravity: params.gravity,
            accel: params.accel,
            friction: params.friction,
            stopspeed: params.stopspeed,
            maxspeed: params.maxspeed,
        };
        let mut cands: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new();
        for (dgx, dgy) in COMPASS {
            // The corridor RUN direction is (dgx,dgy); measure the runway feeding
            // the ledge from behind it. Diagonal grid corridors are real runways
            // too; the certifier, rather than an axis-only prefilter, decides
            // whether their stepped approach and rotation are safe.
            let runway = self.measure_runway_lax(bsp, &a, dgx, dgy);
            if dbg {
                eprintln!("GTDBG ledge={ledge} dir=({dgx},{dgy}) runway={runway:.0}");
            }
            if runway < GT_MIN_RUNWAY {
                continue;
            }
            let psi0 = yaw_of(Vec2::new(dgx as f32, dgy as f32));
            let run_dir = Vec3::new(dgx as f32, dgy as f32, 0.0).normalize();
            // Two committed run-up lengths: the full cap, and a short variant
            // whose from-cell sits closer to the lip — a chained link is only
            // traversable where the entry band is provable, and the proving
            // jump may land PAST the long variant's entry cell.
            for runup in [runway.min(GT_RUNUP_CAP), runway.min(GT_RUNUP_SHORT)] {
                let entry_pt = a.origin - run_dir * runup;
                let Some(from) = self.nearest_within(entry_pt, GRID * 1.5, STEP_HEIGHT * (runup / GRID + 1.0)) else {
                    continue;
                };
                if from == ledge {
                    continue;
                }
                let entry_origin = self.cells[from as usize].origin + Vec3::new(0.0, 0.0, 0.03125);
                let fly_cap = GT_FLIGHT_TICK_CAP as f32 * 0.021;
                let entry_speed_max = entry_speeds.iter().copied().fold(f32::NEG_INFINITY, f32::max);
                let reach = entry_speed_max * fly_cap;
                let scan = ((reach / GRID).ceil() as i32).max(1);
                // Collect candidate targets first, keep only the few most
                // feasible (lowest ballistic floor): the lattice below is the
                // expensive part and an all-fail candidate costs the full grid.
                let mut targets: Vec<(f32, CellId, f32, f32, f32, bool)> = Vec::new(); // (v_floor, to, off, horiz, dz, lip)
                for to in self.neighbors_within(a.gx, a.gy, scan) {
                    if to == ledge || to == from {
                        continue;
                    }
                    let b = self.cells[to as usize];
                    let dz = b.origin.z - a.origin.z;
                    let horiz = (b.origin.xy() - a.origin.xy()).length();
                    if !(-SJ_MAX_DROP..=JUMP_APEX).contains(&dz) || horiz <= JUMP_REACH {
                        continue;
                    }
                    let off = wrap180(yaw_of(b.origin.xy() - a.origin.xy()) - psi0);
                    if !(GT_OFF_LO..=GT_OFF_HI).contains(&off.abs()) {
                        continue;
                    }
                    if self.has_direct_link(ledge, to) || self.has_direct_link(from, to) {
                        if dbg {
                            eprintln!("GTDBG ledge={ledge} to={to} SKIP direct-link");
                        }
                        continue;
                    }
                    // A strict plain curl already owns every direct link that
                    // certified above. Keep the remaining ballistic candidates
                    // here even when a local runway can build their speed: a
                    // self-contained runway-turn may succeed where the plain
                    // fixed-heading curl failed. The expensive lattice remains
                    // behind the physical lip and center-rollout scouts below.
                    let v_floor = v_required(horiz, dz, params.gravity);
                    let v_del_local = prestrafe_delivered(
                        runway.min(CURL_RUNUP_CAP),
                        params.accel,
                        params.maxspeed,
                        params.friction,
                        params.stopspeed,
                    );
                    if !v_floor.is_finite() || v_floor > entry_speed_max * (1.0 + GT_ENTRY_V_TOL) {
                        if dbg {
                            eprintln!(
                            "GTDBG ledge={ledge} to={to} SKIP speed-filter v_floor={v_floor:.0} v_del_local={v_del_local:.0}"
                        );
                        }
                        continue;
                    }
                    // A genuine lip: the flight direction must leave the floor within
                    // two grid columns of the takeoff (an interior floor cell would
                    // otherwise pay for a full all-fail lattice).
                    let bearing = yaw_of(b.origin.xy() - a.origin.xy()).to_radians();
                    let (sx, sy) = (bearing.cos().round() as i32, bearing.sin().round() as i32);
                    let lip = !self.has_ground_near(a.gx + sx, a.gy + sy, a.origin.z)
                        || !self.has_ground_near(a.gx + 2 * sx, a.gy + 2 * sy, a.origin.z);
                    if dbg {
                        eprintln!(
                        "GTDBG ledge={ledge} to={to} CANDIDATE from={from} off={off:.0} horiz={horiz:.0} dz={dz:.0} v_floor={v_floor:.0} lip={lip}"
                    );
                    }
                    targets.push((v_floor, to, off, horiz, dz, lip));
                }
                targets.sort_by(|x, y| x.0.total_cmp(&y.0));
                targets.truncate(GT_TARGETS_PER_DIR);
                for (v_floor, to, off, _horiz, dz, lip) in targets {
                    let b = self.cells[to as usize];
                    let side = off.signum();
                    // Generic self-contained runway-turn scout. A real ordinary
                    // predecessor supplies the entry heading; the traversal then
                    // owns the complete connected runway, grounded rotation and
                    // flight. This is deliberately derived only from graph/BSP
                    // geometry -- no map name, cell id or coordinate oracle.
                    let mut self_contained = Vec::new();
                    for predecessor in self
                        .links
                        .iter()
                        .filter(|link| link.to == from && matches!(link.kind, LinkKind::Walk | LinkKind::Step))
                    {
                        let pred_origin = self.cells[predecessor.from as usize].origin;
                        let incoming = self.cells[from as usize].origin.xy() - pred_origin.xy();
                        if incoming.length_squared() < 1.0 {
                            continue;
                        }
                        let entry_yaw_base = yaw_of(incoming);
                        let acquisition_turn = wrap180(entry_yaw_base - psi0).abs();
                        if !(30.0..=150.0).contains(&acquisition_turn) {
                            continue;
                        }
                        // Keep ordinary and hot ground arrivals as separate certificates. A single
                        // convex envelope spanning both could contain an unsafe middle state. The
                        // turned yaw is derived from graph geometry rather than a map coordinate.
                        let turn_away = wrap180(entry_yaw_base - psi0);
                        let turned_yaw = (entry_yaw_base + turn_away * 0.40).rem_euclid(360.0);
                        // A graph diagonal is not necessarily traversable by the expanded player hull
                        // around a BSP corner. Only the straight predecessor profile depends on that
                        // chord; fail it closed if the hull clips before the handoff cell.
                        let profiles = [
                            (params.maxspeed, 0.04, entry_yaw_base, 2.0),
                            (params.maxspeed, 0.04, turned_yaw, 4.0),
                            (params.maxspeed * 1.1875, 0.04, turned_yaw, 4.0),
                        ];
                        for (v, entry_v_tol, entry_yaw0, entry_yaw_tol) in profiles {
                            let launch_yaw = (psi0 + off * 0.5).rem_euclid(360.0);
                            let (ls, lc) = launch_yaw.to_radians().sin_cos();
                            let launch_dir = Vec3::new(lc, ls, 0.0);
                            let gt = GroundTurnCurl {
                                version: RUNWAY_TURN_VERSION,
                                runway_aim: a.origin + run_dir * 12.0,
                                blended_runway: true,
                                runway_yaw: psi0,
                                lip_reach: 28.0,
                                hold_speed: v_floor + 48.0,
                                turn_dist: runup.min(224.0),
                                launch_yaw,
                                yaw_min: (launch_yaw - 35.0).rem_euclid(360.0),
                                box_min: a.origin - Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 1.0),
                                box_max: a.origin + Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 0.0),
                                launch_gain: 8.0,
                                hold_aim: a.origin + launch_dir * 512.0,
                                gate_point: a.origin,
                                gate_normal: Vec3::ZERO,
                                air_gain: 8.0,
                                landing_aim: b.origin,
                                entry_speed_lo: v * (1.0 - entry_v_tol),
                                entry_speed_hi: v * (1.0 + entry_v_tol),
                                entry_yaw_lo: (entry_yaw0 - entry_yaw_tol).rem_euclid(360.0),
                                entry_yaw_hi: (entry_yaw0 + entry_yaw_tol).rem_euclid(360.0),
                                landing_speed_lo: 0.0,
                                landing_yaw: 0.0,
                            };
                            let (sy, cy) = entry_yaw0.to_radians().sin_cos();
                            let entry = PmState {
                                origin: entry_origin,
                                vel: Vec3::new(v * cy, v * sy, 0.0),
                                on_ground: true,
                                jump_held: false,
                            };
                            if ground_turn_rolls(self, bsp, entry, 0.020, a.origin, &gt, to, &p).is_none() {
                                continue;
                            }
                            if let Some((worst, land_lo, land_yaw)) = self.certify_ground_turn(
                                bsp,
                                entry_origin,
                                entry_yaw0,
                                v,
                                entry_v_tol,
                                entry_yaw_tol,
                                a.origin,
                                &gt,
                                to,
                                &p,
                            ) {
                                let mut gt = gt;
                                gt.landing_speed_lo = land_lo;
                                gt.landing_yaw = land_yaw;
                                self_contained.push((worst, v, gt));
                            }
                        }
                    }
                    if !self_contained.is_empty() {
                        for (worst, v_req, gt) in self_contained {
                            if dbg {
                                eprintln!(
                            "GTDBG ledge={ledge} to={to} SELF-CERT from={from} cost={worst:.3} entry={v_req:.1} land={:.1}",
                            gt.landing_speed_lo,
                        );
                            }
                            let airtime = jump_airtime(dz, params.gravity);
                            let link = Link {
                                from,
                                to,
                                kind: LinkKind::SpeedJump,
                                cost: worst,
                            };
                            let tr = SpeedJumpTraversal {
                                takeoff: a.origin,
                                v_req,
                                airtime,
                                landing_speed_lo: gt.landing_speed_lo,
                                chained: false,
                                curl_gain: gt.air_gain,
                                curl_entry_aim: Vec3::ZERO,
                                curl_switch_dist: 0.0,
                                curl_landing_aim: gt.landing_aim,
                                ground_turn: Some(gt),
                            };
                            cands.push((worst, link, tr));
                        }
                        continue;
                    }
                    if !lip {
                        continue;
                    }
                    // The legacy carried-speed lattice remains scoped to gaps
                    // whose ballistic floor exceeds the locally deliverable
                    // plain-curl regime. The self-contained scout above is the
                    // only new work for candidates below this boundary.
                    let v_del_local = prestrafe_delivered(
                        runway.min(CURL_RUNUP_CAP),
                        params.accel,
                        params.maxspeed,
                        params.friction,
                        params.stopspeed,
                    );
                    if v_floor * SJ_MARGIN <= v_del_local * CURL_V_LO_FRAC {
                        continue;
                    }
                    let entry_yaw0 = psi0;
                    // Canonical-scout gate: a handful of representative profiles;
                    // only a candidate at least one of them lands pays for the
                    // full lattice. (The lattice re-searches from scratch, so a
                    // canonical near-miss still gets its neighbors tried.)
                    let canonical_ok = {
                        let (sy, cy) = entry_yaw0.to_radians().sin_cos();
                        let mut ok = false;
                        'canon: for &v in entry_speeds {
                            if v < v_floor {
                                continue;
                            }
                            for &(gate_dist, off_l, lat) in &[
                                (0.0f32, 8.0f32, 0.0f32),
                                (0.0, 16.0, 0.0),
                                (0.0, 24.0, 0.0),
                                (0.0f32, 42.0f32, 0.0f32),
                                (0.0, 50.0, 0.0),
                                (0.0, 42.0, -8.0),
                                (0.0, 50.0, -8.0),
                                (0.0, 42.0, 8.0),
                                (0.0, 50.0, 8.0),
                                (96.0, 50.0, 0.0),
                                (160.0, 42.0, 0.0),
                            ] {
                                let launch_yaw = (psi0 + side * off_l).rem_euclid(360.0);
                                let (ls, lc) = launch_yaw.to_radians().sin_cos();
                                let launch_dir = Vec3::new(lc, ls, 0.0);
                                let gt = GroundTurnCurl {
                                    version: GROUND_TURN_VERSION,
                                    runway_aim: a.origin + run_dir * 12.0 + Vec3::new(-run_dir.y, run_dir.x, 0.0) * lat,
                                    blended_runway: false,
                                    runway_yaw: 0.0,
                                    lip_reach: 0.0,
                                    hold_speed: 0.0,
                                    turn_dist: 64.0,
                                    launch_yaw,
                                    yaw_min: (launch_yaw - 18.0).rem_euclid(360.0),
                                    box_min: a.origin - Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 1.0),
                                    box_max: a.origin + Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 0.0),
                                    launch_gain: 32.0,
                                    hold_aim: a.origin + launch_dir * 512.0,
                                    gate_point: a.origin + launch_dir * gate_dist,
                                    gate_normal: if gate_dist > 0.0 { -launch_dir } else { Vec3::ZERO },
                                    air_gain: 256.0,
                                    landing_aim: b.origin,
                                    entry_speed_lo: v * (1.0 - GT_ENTRY_V_TOL),
                                    entry_speed_hi: v * (1.0 + GT_ENTRY_V_TOL),
                                    entry_yaw_lo: (entry_yaw0 - GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                    entry_yaw_hi: (entry_yaw0 + GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                    landing_speed_lo: 0.0,
                                    landing_yaw: 0.0,
                                };
                                let entry = PmState {
                                    origin: entry_origin,
                                    vel: Vec3::new(v * cy, v * sy, 0.0),
                                    on_ground: true,
                                    jump_held: false,
                                };
                                if ground_turn_rolls_tol(self, bsp, entry, 0.020, a.origin, &gt, to, &p, 64.0).is_some()
                                {
                                    ok = true;
                                    break 'canon;
                                }
                            }
                        }
                        ok
                    };
                    if !canonical_ok {
                        if dbg {
                            eprintln!("GTDBG ledge={ledge} to={to} canonical scouts all missed");
                        }
                        continue;
                    }
                    let mut scouts_ok = 0usize;
                    let mut solved: Option<(f32, f32, GroundTurnCurl)> = None; // (worst_elapsed, v_req, contract)
                    'search: for &v in entry_speeds {
                        if v < v_floor {
                            continue;
                        }
                        for &gate_dist in &GT_GATE_DISTS {
                            for &lat in &GT_RUNWAY_LATERALS {
                                for &turn_dist in &GT_TURN_DISTS {
                                    for &off_l in &GT_LAUNCH_OFFSETS {
                                        for &slack in &GT_YAW_SLACKS {
                                            for &lgain in &GT_LAUNCH_GAINS {
                                                for &again in &GT_AIR_GAINS {
                                                    let launch_yaw = (psi0 + side * off_l).rem_euclid(360.0);
                                                    let (ls, lc) = launch_yaw.to_radians().sin_cos();
                                                    let launch_dir = Vec3::new(lc, ls, 0.0);
                                                    let lateral = Vec3::new(-run_dir.y, run_dir.x, 0.0) * lat;
                                                    let gt = GroundTurnCurl {
                                                        version: GROUND_TURN_VERSION,
                                                        runway_aim: a.origin + run_dir * 12.0 + lateral,
                                                        blended_runway: false,
                                                        runway_yaw: 0.0,
                                                        lip_reach: 0.0,
                                                        hold_speed: 0.0,
                                                        turn_dist,
                                                        launch_yaw,
                                                        yaw_min: (launch_yaw - slack).rem_euclid(360.0),
                                                        box_min: a.origin - Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 1.0),
                                                        box_max: a.origin + Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 0.0),
                                                        launch_gain: lgain,
                                                        hold_aim: a.origin + launch_dir * 512.0,
                                                        gate_point: a.origin + launch_dir * gate_dist,
                                                        gate_normal: if gate_dist > 0.0 {
                                                            -launch_dir
                                                        } else {
                                                            Vec3::ZERO
                                                        },
                                                        air_gain: again,
                                                        landing_aim: b.origin,
                                                        entry_speed_lo: v * (1.0 - GT_ENTRY_V_TOL),
                                                        entry_speed_hi: v * (1.0 + GT_ENTRY_V_TOL),
                                                        entry_yaw_lo: (entry_yaw0 - GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                                        entry_yaw_hi: (entry_yaw0 + GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                                        landing_speed_lo: 0.0, // stamped after certification
                                                        landing_yaw: 0.0,      // stamped after certification
                                                    };
                                                    // Cheap scout: one center rollout at 20 ms.
                                                    let (sy, cy) = entry_yaw0.to_radians().sin_cos();
                                                    let entry = PmState {
                                                        origin: entry_origin,
                                                        vel: Vec3::new(v * cy, v * sy, 0.0),
                                                        on_ground: true,
                                                        jump_held: false,
                                                    };
                                                    if ground_turn_rolls(self, bsp, entry, 0.020, a.origin, &gt, to, &p)
                                                        .is_none()
                                                    {
                                                        continue;
                                                    }
                                                    scouts_ok += 1;
                                                    if let Some((worst, land_lo, land_yaw)) = self.certify_ground_turn(
                                                        bsp,
                                                        entry_origin,
                                                        entry_yaw0,
                                                        v,
                                                        GT_ENTRY_V_TOL,
                                                        GT_ENTRY_YAW_TOL,
                                                        a.origin,
                                                        &gt,
                                                        to,
                                                        &p,
                                                    ) {
                                                        let mut gt = gt;
                                                        gt.landing_speed_lo = land_lo;
                                                        gt.landing_yaw = land_yaw;
                                                        solved = Some((worst, v, gt));
                                                        break 'search;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if dbg {
                        eprintln!(
                            "GTDBG ledge={ledge} to={to} scouts_ok={scouts_ok} certified={}",
                            solved.is_some()
                        );
                    }
                    let Some((worst, v_req, gt)) = solved else {
                        continue;
                    };
                    let airtime = jump_airtime(dz, params.gravity);
                    // The certified worst elapsed IS the whole leg (run-up,
                    // rotation, flight) — no commit padding on top; padding it
                    // would double-count and misrank the chain against routes
                    // whose walk legs are priced optimistically.
                    let cost = worst;
                    let link = Link {
                        from,
                        to,
                        kind: LinkKind::SpeedJump,
                        cost,
                    };
                    let tr = SpeedJumpTraversal {
                        takeoff: a.origin,
                        v_req,
                        airtime,
                        landing_speed_lo: gt.landing_speed_lo,
                        chained: true,
                        curl_gain: gt.air_gain,
                        curl_entry_aim: Vec3::ZERO,
                        curl_switch_dist: 0.0,
                        curl_landing_aim: gt.landing_aim,
                        ground_turn: Some(gt),
                    };
                    cands.push((cost, link, tr));
                }
            }
        }
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands.truncate(GT_MAX_PER_CELL);
        out.extend(cands.into_iter().map(|(_, l, t)| (l, t)));
    }

    /// Certify one **optimal-sweep** ground-turn profile across its whole
    /// envelope (entry speeds {lo, mid, hi} x entry yaws {-tol, 0, +tol} x the
    /// three cadence classes), exactly like [`Self::certify_ground_turn`] but
    /// flying the grounded approach with [`ground_turn_ground_cmd_optimal`]
    /// (see [`Self::solve_chained_ground_turn_optimal_curl`]). Returns the
    /// worst (largest) certified elapsed, the minimum horizontal landing speed,
    /// and the centre-corner landing yaw.
    #[allow(clippy::too_many_arguments)]
    fn certify_ground_turn_optimal(
        &self,
        bsp: &Bsp,
        entry_origin: Vec3,
        entry_yaw0: f32,
        v_req: f32,
        entry_v_tol: f32,
        entry_yaw_tol: f32,
        takeoff: Vec3,
        gt: &GroundTurnCurl,
        to: CellId,
        p: &PmParams,
    ) -> Option<(f32, f32, f32)> {
        let mut worst = 0.0f32;
        let mut land_lo = f32::INFINITY;
        let mut land_yaw = 0.0f32;
        let speeds = [v_req * (1.0 - entry_v_tol), v_req, v_req * (1.0 + entry_v_tol)];
        let yaws = [entry_yaw0 - entry_yaw_tol, entry_yaw0, entry_yaw0 + entry_yaw_tol];
        for &dt in &GT_DT_CLASSES {
            for &v in &speeds {
                for &yaw in &yaws {
                    let (sy, cy) = yaw.to_radians().sin_cos();
                    let entry = PmState {
                        origin: entry_origin,
                        vel: Vec3::new(v * cy, v * sy, 0.0),
                        on_ground: true,
                        jump_held: false,
                    };
                    let (elapsed, land) = ground_turn_rolls_optimal_tol(self, bsp, entry, dt, takeoff, gt, to, p, 0.0)?;
                    worst = worst.max(elapsed);
                    land_lo = land_lo.min(land.vel.xy().length());
                    if dt == GT_DT_CLASSES[1] && v == v_req && yaw == entry_yaw0 {
                        land_yaw = yaw360_of(land.vel.xy());
                    }
                }
            }
        }
        Some((worst, land_lo, land_yaw))
    }

    /// Additive low-entry sibling of [`Self::solve_chained_ground_turn_from`]:
    /// identical target discovery and identical certification matrix, but the
    /// grounded approach is flown with the **ground-optimal single-sided sweep**
    /// ([`ground_turn_ground_cmd_optimal`]) instead of the default bearing-follow
    /// weave. That law holds the per-tick speed-maximising wish offset
    /// (theta = acos(u*/speed)) off the *current* velocity while rotating onto
    /// `launch_yaw`, so a low carried-entry runway (~320..360) can build the exit
    /// speed the flight budget needs -- the regime the default weave saturates
    /// below (C2 diagnosis; cf. `crates/rtx-nav/tests/gt_greedy_angle_probe.rs`,
    /// where the same physics reaches ~452 u/s from a 358 entry). Because the
    /// exit speed is *built*, the ballistic-floor prefilter is relaxed to
    /// [`GT_OPT_BUILD_FRAC`] x the entry ladder (the default solver assumes
    /// launch speed ~= entry speed, which is exactly the assumption this law
    /// breaks). Emits contracts tagged [`GROUND_TURN_OPTIMAL_VERSION`] so the
    /// flown law equals the proven law.
    ///
    /// Production is unaffected: no live build path calls this; it is driven only
    /// by the `gt-search` harness (`--optimal`). Deterministic: a pure bounded
    /// lattice over the same inputs, no wall clock and no rng.
    pub fn solve_chained_ground_turn_optimal_curl(
        &self,
        bsp: &Bsp,
        ledge: CellId,
        params: SpeedJumpParams,
        entry_speeds: &[f32],
        out: &mut Vec<(Link, SpeedJumpTraversal)>,
    ) {
        let a = self.cells[ledge as usize];
        if bsp.is_liquid_at(a.origin) {
            return;
        }
        let p = PmParams {
            gravity: params.gravity,
            accel: params.accel,
            friction: params.friction,
            stopspeed: params.stopspeed,
            maxspeed: params.maxspeed,
        };
        let entry_speed_max = entry_speeds.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        // The optimal law BUILDS exit speed above entry, so a reachable target's
        // ballistic floor may exceed the entry ladder; scan/prefilter to the
        // built ceiling, not the entry ceiling.
        let built_ceiling = entry_speed_max * GT_OPT_BUILD_FRAC;
        let mut cands: Vec<(f32, Link, SpeedJumpTraversal)> = Vec::new();
        for (dgx, dgy) in COMPASS {
            let runway = self.measure_runway_lax(bsp, &a, dgx, dgy);
            if runway < GT_MIN_RUNWAY {
                continue;
            }
            let psi0 = yaw_of(Vec2::new(dgx as f32, dgy as f32));
            let run_dir = Vec3::new(dgx as f32, dgy as f32, 0.0).normalize();
            for runup in [runway.min(GT_RUNUP_CAP), runway.min(GT_RUNUP_SHORT)] {
                let entry_pt = a.origin - run_dir * runup;
                let Some(from) = self.nearest_within(entry_pt, GRID * 1.5, STEP_HEIGHT * (runup / GRID + 1.0)) else {
                    continue;
                };
                if from == ledge {
                    continue;
                }
                let entry_origin = self.cells[from as usize].origin + Vec3::new(0.0, 0.0, 0.03125);
                let fly_cap = GT_FLIGHT_TICK_CAP as f32 * 0.021;
                let reach = built_ceiling * fly_cap;
                let scan = ((reach / GRID).ceil() as i32).max(1);
                let mut targets: Vec<(f32, CellId, f32, f32)> = Vec::new(); // (v_floor, to, off, dz)
                for to in self.neighbors_within(a.gx, a.gy, scan) {
                    if to == ledge || to == from {
                        continue;
                    }
                    let b = self.cells[to as usize];
                    let dz = b.origin.z - a.origin.z;
                    let horiz = (b.origin.xy() - a.origin.xy()).length();
                    if !(-SJ_MAX_DROP..=JUMP_APEX).contains(&dz) || horiz <= JUMP_REACH {
                        continue;
                    }
                    let off = wrap180(yaw_of(b.origin.xy() - a.origin.xy()) - psi0);
                    if !(GT_OFF_LO..=GT_OFF_HI).contains(&off.abs()) {
                        continue;
                    }
                    if self.has_direct_link(ledge, to) || self.has_direct_link(from, to) {
                        continue;
                    }
                    let v_floor = v_required(horiz, dz, params.gravity);
                    if !v_floor.is_finite() || v_floor > built_ceiling * (1.0 + GT_ENTRY_V_TOL) {
                        continue;
                    }
                    // A genuine lip: the flight direction must leave the floor
                    // within two grid columns of the takeoff.
                    let bearing = yaw_of(b.origin.xy() - a.origin.xy()).to_radians();
                    let (sx, sy) = (bearing.cos().round() as i32, bearing.sin().round() as i32);
                    let lip = !self.has_ground_near(a.gx + sx, a.gy + sy, a.origin.z)
                        || !self.has_ground_near(a.gx + 2 * sx, a.gy + 2 * sy, a.origin.z);
                    if !lip {
                        continue;
                    }
                    targets.push((v_floor, to, off, dz));
                }
                targets.sort_by(|x, y| x.0.total_cmp(&y.0));
                targets.truncate(GT_TARGETS_PER_DIR);
                for (v_floor, to, off, dz) in targets {
                    let b = self.cells[to as usize];
                    let side = off.signum();
                    let entry_yaw0 = psi0;
                    let mut solved: Option<(f32, f32, GroundTurnCurl)> = None; // (worst, v_req, contract)
                    'search: for &v in entry_speeds {
                        for &gate_dist in &GT_GATE_DISTS {
                            for &off_l in &GT_OPT_LAUNCH_OFFSETS {
                                for &again in &GT_AIR_GAINS {
                                    let launch_yaw = (psi0 + side * off_l).rem_euclid(360.0);
                                    let (ls, lc) = launch_yaw.to_radians().sin_cos();
                                    let launch_dir = Vec3::new(lc, ls, 0.0);
                                    let gt = GroundTurnCurl {
                                        version: GROUND_TURN_OPTIMAL_VERSION,
                                        runway_aim: a.origin + run_dir * 12.0,
                                        blended_runway: false,
                                        runway_yaw: psi0,
                                        lip_reach: 0.0,
                                        hold_speed: 0.0,
                                        // Unused by the optimal law (it sweeps from entry, no
                                        // runway->rotation switch); stamped inert for provenance.
                                        turn_dist: 0.0,
                                        launch_yaw,
                                        yaw_min: (launch_yaw - side * GT_OPT_LAUNCH_SLACK).rem_euclid(360.0),
                                        box_min: a.origin - Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 1.0),
                                        box_max: a.origin + Vec3::new(GT_BOX_HALF, GT_BOX_HALF, 0.0),
                                        launch_gain: 32.0,
                                        hold_aim: a.origin + launch_dir * 512.0,
                                        gate_point: a.origin + launch_dir * gate_dist,
                                        gate_normal: if gate_dist > 0.0 { -launch_dir } else { Vec3::ZERO },
                                        air_gain: again,
                                        landing_aim: b.origin,
                                        entry_speed_lo: v * (1.0 - GT_ENTRY_V_TOL),
                                        entry_speed_hi: v * (1.0 + GT_ENTRY_V_TOL),
                                        entry_yaw_lo: (entry_yaw0 - GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                        entry_yaw_hi: (entry_yaw0 + GT_ENTRY_YAW_TOL).rem_euclid(360.0),
                                        landing_speed_lo: 0.0,
                                        landing_yaw: 0.0,
                                    };
                                    // Cheap scout: one centre rollout at 20 ms.
                                    let (sy, cy) = entry_yaw0.to_radians().sin_cos();
                                    let entry = PmState {
                                        origin: entry_origin,
                                        vel: Vec3::new(v * cy, v * sy, 0.0),
                                        on_ground: true,
                                        jump_held: false,
                                    };
                                    if ground_turn_rolls_optimal_tol(self, bsp, entry, 0.020, a.origin, &gt, to, &p, 0.0)
                                        .is_none()
                                    {
                                        continue;
                                    }
                                    if let Some((worst, land_lo, land_yaw)) = self.certify_ground_turn_optimal(
                                        bsp,
                                        entry_origin,
                                        entry_yaw0,
                                        v,
                                        GT_ENTRY_V_TOL,
                                        GT_ENTRY_YAW_TOL,
                                        a.origin,
                                        &gt,
                                        to,
                                        &p,
                                    ) {
                                        let mut gt = gt;
                                        gt.landing_speed_lo = land_lo;
                                        gt.landing_yaw = land_yaw;
                                        solved = Some((worst, v, gt));
                                        break 'search;
                                    }
                                }
                            }
                        }
                    }
                    let Some((worst, v_req, gt)) = solved else {
                        continue;
                    };
                    let _ = v_floor;
                    let airtime = jump_airtime(dz, params.gravity);
                    let link = Link {
                        from,
                        to,
                        kind: LinkKind::SpeedJump,
                        cost: worst,
                    };
                    let tr = SpeedJumpTraversal {
                        takeoff: a.origin,
                        v_req,
                        airtime,
                        landing_speed_lo: gt.landing_speed_lo,
                        chained: true,
                        curl_gain: gt.air_gain,
                        curl_entry_aim: Vec3::ZERO,
                        curl_switch_dist: 0.0,
                        curl_landing_aim: gt.landing_aim,
                        ground_turn: Some(gt),
                    };
                    cands.push((worst, link, tr));
                }
            }
        }
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands.truncate(GT_MAX_PER_CELL);
        out.extend(cands.into_iter().map(|(_, l, t)| (l, t)));
    }
}

/// Contract version tag for [`NavGraph::solve_chained_ground_turn_optimal_curl`]
/// candidates: the grounded approach is the ground-optimal single-sided sweep
/// ([`ground_turn_ground_cmd_optimal`]), distinct from the bearing-follow weave
/// of [`GROUND_TURN_VERSION`]/[`RUNWAY_TURN_VERSION`]. A runtime executor MUST
/// dispatch on this tag to reproduce the proven trajectory.
pub const GROUND_TURN_OPTIMAL_VERSION: u16 = 3;
/// Launch fires when the carried velocity is within this many degrees of
/// `launch_yaw` (approach side) and inside the takeoff box — part of the
/// optimal-sweep styrlag; a runtime reproduces it verbatim.
const GT_OPT_LAUNCH_SLACK: f32 = 8.0;
/// Launch-heading offsets (deg off the corridor axis, toward the target side)
/// sampled by the optimal-sweep solver. Wider than [`GT_LAUNCH_OFFSETS`]: the
/// single-sided sweep can rotate far while still accelerating.
const GT_OPT_LAUNCH_OFFSETS: [f32; 7] = [16.0, 24.0, 35.0, 42.0, 50.0, 60.0, 68.0];
/// Fraction of the entry ladder the optimal sweep is assumed to be able to
/// build the exit speed up to (calibrated near the C2 greedy result 452/358
/// ~= 1.26; a small margin above). Used only to widen the reach/ballistic-floor
/// prefilter so high-`v_req` targets are not pruned before certification proves
/// or rejects them.
const GT_OPT_BUILD_FRAC: f32 = 1.3;
/// Low-entry ladder for the production emission of the optimal-sweep solver.
/// From the corpus calibration the carried DM3 entry band clusters at
/// p10/p50 = 332/358 u/s; this ladder brackets that (320/340/360) so the
/// built-exit law is certified from the speeds a chained route actually
/// delivers, distinct from the high-entry `GT_ENTRY_SPEEDS` (~439/500) the
/// default weave assumes.
const GT_OPT_ENTRY_SPEEDS: [f32; 3] = [320.0, 340.0, 360.0];

/// The deterministic ground-optimal single-sided sweep -- the "optimal curl"
/// grounded styrlag (see [`NavGraph::solve_chained_ground_turn_optimal_curl`]).
///
/// The default grounded steering ([`ground_turn_ground_cmd`]) follows a
/// *position-scheduled* world bearing (`runway_yaw` -> `launch_yaw`) and weaves
/// its strafe sign to recentre the run onto that bearing; that recentre caps the
/// exit speed a low carried-entry runway can build. This law instead holds the
/// **ground-optimal wish offset off the current velocity**:
/// `theta = acos(u*/speed)` with `u* = maxspeed - accel*maxspeed*dt`, which is
/// the closed form of the per-tick speed-maximising angle above `sv_maxspeed`
/// (deriving it: velocity gain squared is `2*speed*A*cos(theta) + A^2` with
/// `A = min(accel*maxspeed*dt, maxspeed - speed*cos(theta))`, maximised at
/// `cos(theta) = u*/speed`; a greedy 1-degree probe over 0..70 converges to it).
/// The strafe side is taken **toward `launch_yaw`** and recomputed each tick, so
/// the carried velocity rotates monotonically onto the launch heading and then
/// holds there (a tight weave about `launch_yaw`) while still accelerating, until
/// the launch gate fires. Below the angling threshold (`speed <= u*`, never
/// reached by the >=320 entry ladder) it simply runs forward at `launch_yaw`.
///
/// Runtime reproduction: this is a pure function of
/// `(velocity, launch_yaw, accel, maxspeed, dt)`; a live executor reproduces the
/// exact grounded trajectory by calling it every grounded setup tick until the
/// launch gate (`|wrap180(launch_yaw - vel_yaw)| <= GT_OPT_LAUNCH_SLACK` while
/// inside the takeoff box) fires, then flies the stored launch/air curl.
pub fn ground_turn_ground_cmd_optimal(vel_xy: Vec2, gt: &GroundTurnCurl, accel: f32, maxspeed: f32, dt: f32) -> Cmd {
    let speed = vel_xy.length();
    let a_ground = accel * maxspeed * dt;
    let u_star = (maxspeed - a_ground).max(0.0);
    if speed <= u_star.max(60.0) {
        // No over-cap speed to preserve: spend the runway running onto the
        // launch heading. (Unreached by the certified >=320 entry ladder.)
        return Cmd {
            view_yaw: gt.launch_yaw,
            forward: MOVE_SPEED,
            side: 0.0,
            jump: false,
        };
    }
    let vel_yaw = yaw_of(vel_xy);
    // Fixed-per-tick side toward launch_yaw (no recentre onto a runway bearing):
    // monotonic rotation onto the launch heading, then a tight hold about it.
    let side_sign = if wrap180(gt.launch_yaw - vel_yaw) >= 0.0 { 1.0 } else { -1.0 };
    let theta = (u_star / speed).clamp(0.0, 1.0).acos();
    let (s, c) = theta.sin_cos();
    // Emit with the view riding the velocity and the angle in forward/side
    // (wishdir = vel_yaw + side_sign*theta), rounded like every other emitted cmd.
    Cmd {
        view_yaw: vel_yaw,
        forward: (MOVE_SPEED * c).round(),
        side: (-side_sign * MOVE_SPEED * s).round(),
        jump: false,
    }
}

/// Launch gate for an optimal-sweep ([`GROUND_TURN_OPTIMAL_VERSION`]) contract:
/// the runtime analogue of the `launch_now` closure inside
/// [`ground_turn_rolls_optimal_tol`]. Fire on the first grounded tick inside the
/// takeoff box whose carried velocity has rotated to within
/// [`GT_OPT_LAUNCH_SLACK`] of `launch_yaw` (approach side).
///
/// Deviation from the rollout: the rollout fixes `sweep_side` from the *entry*
/// heading and tests `wrap180(launch_yaw - vel_yaw) * sweep_side <= SLACK`, while
/// here `sweep_side = sign(wrap180(launch_yaw - vel_yaw))` is recomputed from the
/// *current* velocity — matching [`ground_turn_ground_cmd_optimal`], which also
/// derives its strafe side from the live velocity. The two are equivalent for the
/// launch decision: the sweep rotates the velocity monotonically toward
/// `launch_yaw`, so `sign(wrap180(launch_yaw - vel_yaw))` is invariant until the
/// velocity actually reaches the launch heading; on every pre-fire tick both
/// forms therefore agree, and `x * sign(x) = |x|` so the test reduces to
/// `|wrap180(launch_yaw - vel_yaw)| <= SLACK`, firing on exactly the same first
/// tick the entry-fixed form fires.
pub fn ground_turn_should_launch_optimal(origin: Vec3, vel_xy: Vec2, on_ground: bool, gt: &GroundTurnCurl) -> bool {
    let in_box = origin.x >= gt.box_min.x
        && origin.x <= gt.box_max.x
        && origin.y >= gt.box_min.y
        && origin.y <= gt.box_max.y
        && origin.z >= gt.box_min.z;
    let delta = wrap180(gt.launch_yaw - yaw_of(vel_xy));
    let sweep_side = if delta >= 0.0 { 1.0 } else { -1.0 };
    on_ground && in_box && delta * sweep_side <= GT_OPT_LAUNCH_SLACK
}

/// One certified **optimal-sweep** rollout (the [`ground_turn_ground_cmd_optimal`]
/// analogue of [`ground_turn_rolls`]): grounded ground-optimal sweep from `entry`
/// onto `launch_yaw`, launch on the first grounded tick within
/// [`GT_OPT_LAUNCH_SLACK`] of `launch_yaw` inside the takeoff box, then the stored
/// launch/air curl to first touchdown resolving to `to` -- zero wall contact,
/// zero start-solid, no fall. `accept_near > 0` also accepts a touchdown within
/// that XY distance of the target cell (the feasibility scout; certification uses
/// exact, `accept_near = 0`). Returns (elapsed, landing state).
#[allow(clippy::too_many_arguments)]
fn ground_turn_rolls_optimal_tol(
    graph: &NavGraph,
    bsp: &Bsp,
    entry: PmState,
    dt: f32,
    takeoff: Vec3,
    gt: &GroundTurnCurl,
    to: CellId,
    p: &PmParams,
    accept_near: f32,
) -> Option<(f32, PmState)> {
    use crate::pmove::pm_step_report;
    let solid = |o: Vec3| {
        let tr = bsp.hull1_trace(o, o);
        tr.start_solid || tr.all_solid
    };
    let mut s = entry;
    let mut elapsed = 0.0;
    let mut airborne_streak = 0usize;
    let floor_z = entry.origin.z.min(takeoff.z) - 80.0;
    // Sweep side fixed from the entry heading; the launch gate measures signed
    // progress toward launch_yaw on that side (fires from first reaching it).
    let sweep_side = if wrap180(gt.launch_yaw - yaw_of(entry.vel.xy())) >= 0.0 { 1.0 } else { -1.0 };
    let in_box = |o: Vec3| {
        o.x >= gt.box_min.x && o.x <= gt.box_max.x && o.y >= gt.box_min.y && o.y <= gt.box_max.y && o.z >= gt.box_min.z
    };
    let launch_now = |o: Vec3, v: Vec2, og: bool| {
        og && in_box(o) && wrap180(gt.launch_yaw - yaw_of(v)) * sweep_side <= GT_OPT_LAUNCH_SLACK
    };
    let mut setup = 0usize;
    loop {
        if launch_now(s.origin, s.vel.xy(), s.on_ground) {
            break;
        }
        if setup >= GT_SETUP_TICK_CAP {
            return None;
        }
        let cmd = ground_turn_ground_cmd_optimal(s.vel.xy(), gt, p.accel, p.maxspeed, dt);
        if solid(s.origin) {
            return None;
        }
        let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
        if rep.wall_contact || solid(s.origin) || s.origin.z < floor_z {
            return None;
        }
        elapsed += dt;
        setup += 1;
        if s.on_ground {
            airborne_streak = 0;
        } else {
            airborne_streak += 1;
            if airborne_streak >= 3 {
                return None; // left the runway floor
            }
        }
        if s.jump_held {
            return None;
        }
    }
    // Launch tick: fire the jump aiming along launch_yaw.
    let (ls, lc) = gt.launch_yaw.to_radians().sin_cos();
    let aim = s.origin.xy() + Vec2::new(lc, ls) * 512.0;
    let cmd = ground_turn_launch_cmd(s.vel.xy(), yaw_of(aim - s.origin.xy()), gt, p.accel, p.maxspeed, dt);
    if solid(s.origin) {
        return None;
    }
    let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
    if rep.wall_contact || solid(s.origin) || s.on_ground {
        return None;
    }
    elapsed += dt;
    for _ in 0..GT_FLIGHT_TICK_CAP {
        let cmd = ground_turn_air_cmd(s.origin, s.vel.xy(), gt, p.accel, p.maxspeed, dt);
        if solid(s.origin) {
            return None;
        }
        let rep = pm_step_report(bsp, &mut s, &cmd, p, dt);
        if rep.wall_contact || solid(s.origin) || s.origin.z < floor_z {
            return None;
        }
        elapsed += dt;
        if s.on_ground {
            let land = graph.nearest_within(s.origin, 24.0, 2.0);
            let target = graph.cells[to as usize].origin;
            let near = accept_near > 0.0
                && (s.origin.xy() - target.xy()).length() <= accept_near
                && (s.origin.z - target.z).abs() <= CURL_Z_TOL;
            return if land == Some(to) || near {
                Some((elapsed, s))
            } else {
                None
            };
        }
    }
    None
}

#[cfg(test)]
mod ground_turn_tests {
    use super::*;

    fn contract() -> GroundTurnCurl {
        GroundTurnCurl {
            version: GROUND_TURN_VERSION,
            runway_aim: Vec3::new(148.0, -576.0, 152.0),
            blended_runway: false,
            runway_yaw: 0.0,
            lip_reach: 0.0,
            hold_speed: 0.0,
            turn_dist: 64.0,
            launch_yaw: 222.0,
            yaw_min: 204.0,
            box_min: Vec3::new(132.0, -604.0, 151.0),
            box_max: Vec3::new(188.0, -548.0, 152.0),
            launch_gain: 32.0,
            hold_aim: Vec3::new(-220.0, -920.0, 152.0),
            gate_point: Vec3::new(160.0, -576.0, 152.0),
            gate_normal: Vec3::ZERO,
            air_gain: 256.0,
            landing_aim: Vec3::new(64.0, -832.0, 168.0),
            entry_speed_lo: 490.0,
            entry_speed_hi: 510.0,
            entry_yaw_lo: 168.0,
            entry_yaw_hi: 192.0,
            landing_speed_lo: 493.0,
            landing_yaw: 258.0,
        }
    }

    #[test]
    fn yaw360_is_wrap_free_over_the_turn_band() {
        // atan2 wraps at +-180 exactly mid-turn; the [0,360) domain must not.
        let west = Vec2::new(-1.0, 0.0);
        let wsw = Vec2::new(-0.74, -0.67); // ~222 deg
        assert!((yaw360_of(west) - 180.0).abs() < 0.01);
        assert!((yaw360_of(wsw) - 222.15).abs() < 0.5);
        assert!(yaw360_of(wsw) > yaw360_of(west));
    }

    #[test]
    fn ground_aim_switches_from_runway_to_rotation_inside_turn_dist() {
        let gt = contract();
        let takeoff = Vec3::new(160.0, -576.0, 152.0);
        // Far out on the runway: the aim is the runway line.
        let far = Vec3::new(340.0, -560.0, 56.0);
        assert_eq!(ground_turn_ground_aim(far, takeoff, &gt), gt.runway_aim.xy());
        // Inside the turn window: the aim rotates toward the launch heading.
        let near = Vec3::new(200.0, -572.0, 152.0);
        let aim = ground_turn_ground_aim(near, takeoff, &gt);
        let (s, c) = gt.launch_yaw.to_radians().sin_cos();
        let expect = takeoff.xy() + Vec2::new(c, s) * 512.0;
        assert!((aim - expect).length() < 0.01);
    }

    #[test]
    fn launch_gate_needs_box_ground_and_rotated_yaw() {
        let gt = contract();
        let inside = Vec3::new(170.0, -576.0, 152.0);
        let rotated = Vec2::new(-350.0, -320.0); // ~222 deg
        let unrotated = Vec2::new(-500.0, 30.0); // ~177 deg
        assert!(ground_turn_should_launch(inside, rotated, true, &gt));
        assert!(
            !ground_turn_should_launch(inside, rotated, false, &gt),
            "airborne must not fire"
        );
        assert!(!ground_turn_should_launch(inside, unrotated, true, &gt), "yaw gate");
        let outside = Vec3::new(220.0, -576.0, 152.0);
        assert!(
            !ground_turn_should_launch(outside, rotated, true, &gt),
            "outside the box"
        );
        let below = Vec3::new(170.0, -576.0, 120.0);
        assert!(
            !ground_turn_should_launch(below, rotated, true, &gt),
            "below the platform"
        );
    }

    #[test]
    fn air_aim_holds_until_the_gate_plane_then_lands() {
        let mut gt = contract();
        // Immediate curl (zero normal): always the landing aim.
        assert_eq!(
            ground_turn_air_aim(Vec3::new(150.0, -600.0, 160.0), &gt).0,
            gt.landing_aim.xy()
        );
        // A hold gate 96u along the launch heading: held before, landing after.
        let (s, c) = gt.launch_yaw.to_radians().sin_cos();
        let dir = Vec3::new(c, s, 0.0);
        gt.gate_point = Vec3::new(160.0, -576.0, 152.0) + dir * 96.0;
        gt.gate_normal = -dir;
        let before = Vec3::new(160.0, -576.0, 152.0) + dir * 20.0;
        let after = Vec3::new(160.0, -576.0, 152.0) + dir * 140.0;
        assert_eq!(ground_turn_air_aim(before, &gt).0, gt.hold_aim.xy());
        assert_eq!(ground_turn_air_aim(after, &gt).0, gt.landing_aim.xy());
    }

    #[test]
    fn entry_envelope_fails_closed() {
        let gt = contract();
        let ok_vel = Vec2::new(-500.0, 0.0); // 180 deg, 500 ups
        assert!(ground_turn_entry_ok(500.0, ok_vel, true, &gt));
        assert!(!ground_turn_entry_ok(500.0, ok_vel, false, &gt), "airborne entry");
        assert!(!ground_turn_entry_ok(480.0, ok_vel, true, &gt), "too slow");
        assert!(
            !ground_turn_entry_ok(520.0, ok_vel, true, &gt),
            "too fast is ALSO uncertified"
        );
        let off_yaw = Vec2::new(-350.0, 350.0); // 135 deg
        assert!(
            !ground_turn_entry_ok(500.0, off_yaw, true, &gt),
            "outside the yaw envelope"
        );
    }
}
