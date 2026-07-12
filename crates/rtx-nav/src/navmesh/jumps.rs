// SPDX-License-Identifier: AGPL-3.0-or-later

//! Jump-link generation: the run-jump (`find_jumps`), rtx double-jump (`add_double_jumps`), and
//! bhop-carried speed-jump (`add_speed_jumps`) passes plus their per-cell solvers and the runway
//! measurer. Each pass floods candidates off ledge edges, dedups them per compass octant, arc-tests
//! clearance, and splices the survivors into the graph. Runs on the parallel build's worker cells.

use glam::{Vec3, Vec3Swizzles};

use super::geom::*;
use super::physics::*;
use super::*;
use crate::bsp::Bsp;

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
                            let tr = SpeedJumpTraversal { takeoff: a.origin, v_req, airtime, chained: false };
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
                    let tr = SpeedJumpTraversal { takeoff: a.origin, v_req, airtime, chained: true };
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
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands.truncate(SPEED_JUMP_MAX_PER_CELL);
        cands_chained.sort_by(|x, y| x.0.total_cmp(&y.0));
        cands_chained.truncate(SPEED_JUMP_CHAINED_MAX_PER_CELL);
        out.extend(cands.into_iter().map(|(_, l, t)| (l, t)));
        out.extend(cands_chained.into_iter().map(|(_, l, t)| (l, t)));
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
