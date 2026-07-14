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
use crate::strafe::{air_accel_max, air_correct, Cmd, MOVE_SPEED};

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
        // Curl jumps second (after the straight speed jumps are spliced, so the per-target dedup can
        // see them): a separate certified pass for gaps that need a run-up *and* an air-turn.
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
            for (link, tr) in curls.into_iter().flatten() {
                self.push_speed_jump(link, tr);
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
    fn solve_curl_jumps_from(&self, bsp: &Bsp, ledge: CellId, params: SpeedJumpParams, k: f32, out: &mut Vec<(Link, SpeedJumpTraversal)>) {
        let a = self.cells[ledge as usize];
        if bsp.is_liquid_at(a.origin) {
            return; // submerged takeoff: can't jump
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
            let v_deliver = prestrafe_delivered(runway, params.accel, params.maxspeed, params.friction, params.stopspeed);
            let v_max_straight = SPEED_JUMP_V_CAP.min(BHOP_EFF * attainable_speed(MAX_SPEED, runway, k));
            let psi0 = yaw_of(Vec2::new(dgx as f32, dgy as f32)); // corridor / takeoff heading
            let reach = v_deliver * jump_airtime(-SJ_MAX_DROP, params.gravity);
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
                if airtime <= 0.0 {
                    continue;
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
                let dir = Vec3::new(dgx.signum() as f32, dgy.signum() as f32, 0.0).normalize_or_zero();
                let t_max = (runway - CURL_MIN_RUNWAY).clamp(0.0, CURL_TAKEOFF_BACKOFF);
                let mut solved: Option<(Vec3, f32, f32)> = None; // (takeoff, v_req, gain)
                let mut t = 0.0;
                loop {
                    let takeoff = a.origin - dir * t;
                    if self.nearest_within(takeoff, GRID * 0.75, STEP_HEIGHT * 2.0).is_some() {
                        // Cheap scout first — one mid-gain center rollout — so the full 48-corner certify
                        // only runs where a landing is already near the target (else this pass is ~50× slower).
                        let near = curl_land_point(bsp, takeoff, b.origin, v_deliver, psi0, 10.0, &p).is_some_and(|land| {
                            (land.xy() - b.origin.xy()).length() <= CURL_MISS_TOL * 2.0 && (land.z - b.origin.z).abs() <= CURL_Z_TOL
                        });
                        if near {
                            if let Some((v_req, gain)) = certify_curl(bsp, takeoff, b.origin, psi0, v_deliver, &p) {
                                solved = Some((takeoff, v_req, gain));
                                break;
                            }
                        }
                    }
                    t += GRID * 1.5;
                    if t > t_max {
                        break;
                    }
                }
                let Some((takeoff, v_req, gain)) = solved else {
                    continue;
                };
                // Place the run-up start behind the (backed-off) takeoff along the corridor.
                let back = (takeoff.xy() - a.origin.xy()).length();
                let Some(start) = self.nearest_within(takeoff - dir * (runway - back), GRID * 1.5, STEP_HEIGHT * 3.0) else {
                    continue;
                };
                if start == to || self.has_direct_link(start, to) {
                    continue;
                }
                let cost = runway / ((MAX_SPEED + v_deliver) * 0.5) + airtime + CURL_COMMIT;
                let link = Link { from: start, to, kind: LinkKind::SpeedJump, cost };
                let tr = SpeedJumpTraversal { takeoff, v_req, airtime, chained: false, curl_gain: gain };
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
                            let link = Link { from: start, to, kind: LinkKind::SpeedJump, cost };
                            let tr = SpeedJumpTraversal { takeoff: a.origin, v_req, airtime, chained: false, curl_gain: 0.0 };
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
                    let link = Link { from: ledge, to, kind: LinkKind::SpeedJump, cost };
                    let tr = SpeedJumpTraversal { takeoff: a.origin, v_req, airtime, chained: true, curl_gain: 0.0 };
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

impl NavGraph {
    /// Debug probe (harness): from `takeoff` along `psi0` (degrees) with the speed a `runway` delivers,
    /// report the predicted takeoff speed, whether the full envelope certifies, and per-gain the
    /// center-corner landing point — so the harness can see *why* a curl candidate is/ isn't emitted.
    pub fn curl_probe(&self, bsp: &Bsp, takeoff: Vec3, target: Vec3, psi0: f32, runway: f32, params: SpeedJumpParams) -> (f32, Option<(f32, f32)>, Vec<(f32, Vec3)>) {
        let p = PmParams {
            gravity: params.gravity,
            accel: params.accel,
            friction: params.friction,
            stopspeed: params.stopspeed,
            maxspeed: params.maxspeed,
        };
        let v_deliver = prestrafe_delivered(runway, params.accel, params.maxspeed, params.friction, params.stopspeed);
        let detail: Vec<(f32, Vec3)> = CURL_GAINS
            .iter()
            .map(|&gain| (gain, curl_land_point(bsp, takeoff, target, v_deliver, psi0, gain, &p).unwrap_or(Vec3::ZERO)))
            .collect();
        (v_deliver, certify_curl(bsp, takeoff, target, psi0, v_deliver, &p), detail)
    }
}

/// Roll a curl and return the landing origin (or `None` if it never touched down after the leap) — the
/// probe variant of [`curl_lands`], without the accept tolerances.
fn curl_land_point(bsp: &Bsp, takeoff: Vec3, target: Vec3, v0: f32, psi: f32, gain: f32, p: &PmParams) -> Option<Vec3> {
    let dt = CURL_DT;
    let amax = air_accel_max(p.accel, p.maxspeed, dt);
    let (s0, c0) = psi.to_radians().sin_cos();
    let mut s = PmState { origin: takeoff, vel: Vec3::new(v0 * c0, v0 * s0, 0.0), on_ground: true, jump_held: false };
    for tick in 0..CURL_MAX_TICKS {
        let cmd = if tick == 0 {
            Cmd { view_yaw: psi, forward: MOVE_SPEED, side: 0.0, jump: true }
        } else {
            let v_xy = s.vel.xy();
            let st = air_correct(v_xy, yaw_of(target.xy() - s.origin.xy()), amax, dt, gain);
            Cmd { view_yaw: st.view_yaw, forward: st.forward, side: st.side, jump: false }
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
/// target cell across the whole delivered-speed × launch-heading envelope. Returns `(v_req, gain)` —
/// `v_req` the envelope's low corner (what the runtime must at least deliver) — or `None`.
fn certify_curl(bsp: &Bsp, takeoff: Vec3, target: Vec3, psi0: f32, v_deliver: f32, p: &PmParams) -> Option<(f32, f32)> {
    let v_lo = v_deliver * CURL_V_LO_FRAC;
    // Envelope corners: {v_lo, v_deliver} × {psi0−tol, psi0, psi0+tol}.
    let corners = [(v_lo, -CURL_PSI_TOL), (v_lo, 0.0), (v_lo, CURL_PSI_TOL), (v_deliver, -CURL_PSI_TOL), (v_deliver, 0.0), (v_deliver, CURL_PSI_TOL)];
    for &gain in &CURL_GAINS {
        if corners.iter().all(|&(v0, dp)| curl_lands(bsp, takeoff, target, v0, psi0 + dp, gain, p)) {
            return Some((v_lo, gain));
        }
    }
    None
}

/// Roll one curl and test whether it lands on the target cell: `pm_step` from `takeoff` seeded at
/// (`v0`, `psi` degrees), leap on tick 0, then per-tick `air_correct` toward the target at `gain` — the
/// exact runtime air policy. Accepts the first touchdown after the leap that resolves to the target
/// within tolerance; rejects a heading that crosses the target bearing mid-flight (an overshoot the
/// held-sign air-strafe diverges from) or an arc that falls well below / flies past the target.
fn curl_lands(bsp: &Bsp, takeoff: Vec3, target: Vec3, v0: f32, psi: f32, gain: f32, p: &PmParams) -> bool {
    let dt = CURL_DT;
    let amax = air_accel_max(p.accel, p.maxspeed, dt);
    let (s0, c0) = psi.to_radians().sin_cos();
    let mut s = PmState { origin: takeoff, vel: Vec3::new(v0 * c0, v0 * s0, 0.0), on_ground: true, jump_held: false };
    let mut prev_sign = 0.0f32;
    for tick in 0..CURL_MAX_TICKS {
        let cmd = if tick == 0 {
            Cmd { view_yaw: psi, forward: MOVE_SPEED, side: 0.0, jump: true }
        } else {
            let v_xy = s.vel.xy();
            let bearing = yaw_of(target.xy() - s.origin.xy());
            let err = wrap180(bearing - yaw_of(v_xy));
            if prev_sign != 0.0 && err.signum() != prev_sign && err.abs() > 2.0 {
                return false; // overshot the target bearing — the runtime curl would diverge here
            }
            prev_sign = err.signum();
            let st = air_correct(v_xy, bearing, amax, dt, gain);
            Cmd { view_yaw: st.view_yaw, forward: st.forward, side: st.side, jump: false }
        };
        pm_step(bsp, &mut s, &cmd, p, dt);
        if s.vel.z < 0.0 && s.origin.z < target.z - 100.0 {
            return false; // fell past the target's level — undershoot
        }
        if tick > 3 && s.on_ground {
            return (s.origin.xy() - target.xy()).length() <= CURL_MISS_TOL && (s.origin.z - target.z).abs() <= CURL_Z_TOL;
        }
    }
    false
}
