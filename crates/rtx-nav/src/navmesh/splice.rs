// SPDX-License-Identifier: AGPL-3.0-or-later

//! Entity-derived graph splices: the `func_plat` lifts, `trigger_teleport` pairs, and
//! button-gated door/movewall obstructions the game layer discovers and hands to the build. Each
//! `add_*` pass folds a batch of plain-data `*Info` records into the graph as `Plat`/`Teleport`/gate
//! links, and the accessors expose the resulting side-tables to the runtime. See `nav_build.rs` in
//! the game crate for where the `*Info` come from.

use glam::Vec3;

use super::geom::*;
use super::*;
use crate::bsp::Bsp;
use crate::qphys::ORIGIN_TO_FEET;

/// The two standing positions a `func_plat` connects: the player-origin spot on the plat
/// surface at the bottom of travel (`board`) and at the top (`exit`), plus the edict id and
/// the plat brush's world-XY footprint so the runtime can read the lift's live state and hold
/// a standoff outside its inner trigger (see [`Plat`]).
pub struct PlatInfo {
    pub board: Vec3,
    pub exit: Vec3,
    /// The `func_plat` edict, to read its live mover state at runtime.
    pub entity: u32,
    /// World-XY footprint of the plat brush (XY is travel-invariant), for the standoff box.
    pub fp_min: Vec2,
    pub fp_max: Vec2,
    /// World-z of the plat body's bottom face at rest — the shaft floor, and the lower bound of the
    /// under-plat cell stamp (anything below it is beneath the floor the lift rests on, not in the shaft).
    pub bottom: f32,
}

/// A `trigger_teleport`: its world-space trigger box (`tmin`/`tmax`) and the player-origin
/// arrival point at its destination (`dest`).
pub struct TeleportInfo {
    pub tmin: Vec3,
    pub tmax: Vec3,
    pub dest: Vec3,
}

/// A button-gated obstruction (a sliding `func_door` or a rotating `func_movewall`): the
/// obstructing entity (to read its current position), where it sits while blocking
/// (`closed_origin` — it's "open" once moved from here), where the bot operates the button from
/// (`button_cell`), the button centre to face/touch/shoot (`aim`), and whether it's shot.
pub struct Gate {
    pub obstruction: u32,
    pub closed_origin: Vec3,
    /// The activator entity (button or shootable trigger), to read its cooldown/`takedamage`
    /// state — a re-triggerable activator goes dead for a while after each use.
    pub activator: u32,
    pub button_cell: CellId,
    pub aim: Vec3,
    pub shoot: bool,
}

/// A spliced `func_plat`: the edict whose live mover state gates boarding, and the plat brush's
/// world-XY footprint. The inner trigger is this footprint shrunk 25u in XY, spanning the full
/// travel height, so a live player standing on the ground *under* a raised plat is inside it and
/// keeps resetting its lower-timer — hence the bot must hold a standoff outside this box until the
/// lift is down (see the plat-hold logic in `bot::run_bot`).
pub struct Plat {
    pub entity: u32,
    pub fp_min: Vec2,
    pub fp_max: Vec2,
}

/// Inputs for [`NavGraph::add_gates`], gathered from spawned obstruction/activator entities: the
/// obstruction's closed-position origin and world box, the activator entity + its centre, and
/// whether it's shot rather than touched.
pub struct GateInfo {
    pub obstruction: u32,
    pub closed_origin: Vec3,
    pub closed_min: Vec3,
    pub closed_max: Vec3,
    pub activator: u32,
    pub button: Vec3,
    pub shoot: bool,
}

impl NavGraph {
    /// Splice `func_plat` lifts into the graph. For each plat we add a cell on its surface at
    /// the bottom (the board point), a [`LinkKind::Plat`] ride from there to the floor the plat
    /// delivers to at the top, and `JumpGap` "jump aboard" links from the nearby lower floor
    /// onto the plat — boarding by jumping is safer because the trigger that raises the plat is
    /// larger than the plat brush. Plats whose top doesn't reach any floor cell are skipped.
    pub fn add_plats(&mut self, bsp: &Bsp, plats: &[PlatInfo]) {
        for p in plats {
            // Where does the plat deliver you? Nearest floor cell to its raised surface.
            let Some(top) = self.nearest_within(p.exit, GRID * 3.0, STEP_HEIGHT * 2.0) else {
                continue;
            };
            // Register the plat only once its top wired in (skipped plats never register — same as
            // gates), so `plat_of_link` indices stay dense and match `plats`.
            let pi = self.plats.push(Plat {
                entity: p.entity,
                fp_min: p.fp_min,
                fp_max: p.fp_max,
            });
            let board = self.add_cell(p.board);
            let ride = (p.exit.z - p.board.z).max(0.0);
            self.push_plat_link(
                Link {
                    from: board,
                    to: top,
                    kind: LinkKind::Plat,
                    cost: ride / MAX_SPEED + 1.0, // ride time + boarding/trigger overhead
                },
                pi,
            );
            // Jump-aboard links from the surrounding lower floor.
            for c in self.cells_near(p.board.xy(), GRID * 3.0) {
                if c == board {
                    continue;
                }
                let from = self.cells[c as usize].origin;
                let dz = p.board.z - from.z;
                if dz.abs() <= JUMP_APEX && arc_clear(bsp, from, p.board) {
                    let horiz = (p.board.xy() - from.xy()).length();
                    self.push_plat_link(
                        Link {
                            from: c,
                            to: board,
                            kind: LinkKind::JumpGap,
                            cost: link_cost(LinkKind::JumpGap, horiz, dz),
                        },
                        pi,
                    );
                }
            }
            self.stamp_under_plat(p, pi);
        }
    }

    /// Mark every cell inside plat `p`'s swept volume with its index, so the planner can price the shaft
    /// as transit-only and the runtime can tell a bot it is standing where the lift wants to land (see
    /// the `under_plat` column). The box is the footprint grown by the player half-width — a body that
    /// far outside the brush still overlaps it and blocks the descent — spanning from the shaft floor up
    /// to just under the raised surface, so the floor cells the lift *delivers* to (origin ≈ `exit.z`)
    /// stay open ground. The board cell is stamped on purpose: the lift surface is no place to camp
    /// either, and boarding is unaffected because every link entering it is plat-tagged and so exempt
    /// from [`surcharge_under_plat_links`](Self::surcharge_under_plat_links).
    fn stamp_under_plat(&mut self, p: &PlatInfo, pi: usize) {
        let m = PLAYER_HALF_WIDTH;
        let lo = Vec3::new(p.fp_min.x - m, p.fp_min.y - m, p.bottom);
        let hi = Vec3::new(p.fp_max.x + m, p.fp_max.y + m, p.exit.z - ORIGIN_TO_FEET - 1.0);
        // Sized here rather than once up front: `add_cell` (the board cell above) appends as we go, and
        // a short column would silently read as all-clear.
        self.under_plat.resize(self.cells.len(), None);
        for c in self.cells_in_box(lo, hi) {
            self.under_plat[c as usize] = Some(pi as u16);
        }
    }

    /// Charge [`UNDER_PLAT_EXTRA`] on every link *entering* an under-plat cell, so routes prefer any
    /// comparable way around a lift shaft. Plat-tagged links — the ride and the jump-aboards — keep their
    /// solved cost: reaching those cells is the whole point of boarding. Pure (the stamp is build-side
    /// geometry), so unlike the water/hazard passes this runs inside the worker build.
    pub fn surcharge_under_plat_links(&mut self) {
        if self.under_plat.is_empty() {
            return; // no plats spliced — nothing to price
        }
        for li in 0..self.links.len() {
            if self.plats.index_of_link(li as u32).is_some() {
                continue;
            }
            let to = self.links[li].to as usize;
            if self.under_plat.get(to).copied().flatten().is_some() {
                self.links[li].cost += UNDER_PLAT_EXTRA;
            }
        }
    }

    /// Push a plat-related link (the ride or a jump-aboard), tagging it with plat index `pi` so the
    /// runtime can look the lift up via [`plat_of_link`](Self::plat_of_link), tagging the new link
    /// in the `plats` side table (mirroring [`push_hook`](Self::push_hook)).
    fn push_plat_link(&mut self, link: Link, pi: usize) {
        self.push_link(link);
        self.plats.tag(self.links.len() - 1, pi);
    }

    pub fn plat_count(&self) -> usize {
        self.plats.len()
    }

    pub fn plat(&self, i: usize) -> &Plat {
        self.plats.item(i)
    }

    /// The plat (if any) that link `li` boards or rides.
    pub fn plat_of_link(&self, li: u32) -> Option<usize> {
        self.plats.index_of_link(li)
    }

    /// Splice `trigger_teleport`s into the graph: every standable cell inside a teleporter's
    /// trigger box gets a [`LinkKind::Teleport`] link to the cell at its destination. The bot
    /// needs no special handling — routing onto an entrance cell walks it into the trigger and
    /// the engine warps it; a separate displacement check then re-paths from the landing spot.
    /// Teleporters whose destination doesn't reach any floor cell are skipped.
    pub fn add_teleports(&mut self, teles: &[TeleportInfo]) {
        for t in teles {
            let Some(dest) = self.nearest_within(t.dest, GRID * 3.0, 96.0) else {
                continue;
            };
            // Entrance cells: those whose footprint sits within the trigger box (loosened in Z
            // so a floor cell standing in a doorway-tall trigger still counts).
            let lo = Vec3::new(t.tmin.x, t.tmin.y, t.tmin.z - 32.0);
            let hi = Vec3::new(t.tmax.x, t.tmax.y, t.tmax.z + 24.0);
            for c in self.cells_in_box(lo, hi) {
                if c != dest {
                    self.push_link(Link {
                        from: c,
                        to: dest,
                        kind: LinkKind::Teleport,
                        cost: 0.2,
                    });
                }
            }
        }
    }

    /// Cells whose origin lies within the axis-aligned box `[min, max]`.
    fn cells_in_box(&self, min: Vec3, max: Vec3) -> Vec<CellId> {
        let mut out = Vec::new();
        for gx in floor_grid(min.x)..=floor_grid(max.x) {
            for gy in floor_grid(min.y)..=floor_grid(max.y) {
                if let Some(ids) = self.grid.get(&(gx, gy)) {
                    for &c in ids {
                        let o = self.cells[c as usize].origin;
                        if (min.x..=max.x).contains(&o.x)
                            && (min.y..=max.y).contains(&o.y)
                            && (min.z..=max.z).contains(&o.z)
                        {
                            out.push(c);
                        }
                    }
                }
            }
        }
        out
    }

    // --- entity-derived: button-gated doors ---

    /// Register button-gated doors. Each `func_door` with a targetname is a gate that stays shut
    /// until its `func_button` fires it; the static carve (hull 0, no door brushes) has links
    /// running straight through, so we tag every link whose *segment* passes through the door's
    /// *closed* volume with that gate and remember which button opens it. Tagging links (not cells)
    /// is what makes this robust for thin pillars — a link crossing a 14-unit door is caught even
    /// when no cell centre lands inside it. Pathfinding then prices those links by door state
    /// (see [`find_path`](Self::find_path)); bots detour to the button when a route must cross a
    /// shut one (see `bot.rs`). Gates whose closed door crosses no link, or whose button has no
    /// nearby cell to operate from, are skipped.
    pub fn add_gates(&mut self, gates: &[GateInfo]) {
        for gi in gates {
            let Some(button_cell) = self.nearest_within(gi.button, GRID * 5.0, 160.0) else {
                continue;
            };
            // Inflate the door box by the player's horizontal half-width before testing links: a
            // link whose centre-line passes just *beside* the door still can't be walked (the
            // player's 32-wide body clips it), so it must be gated too — otherwise a bot takes the
            // "around" route onto that link and wedges against the pillar. This is the standard
            // navmesh trick of growing obstacles by the agent radius.
            let margin = Vec3::new(PLAYER_HALF_WIDTH, PLAYER_HALF_WIDTH, 0.0);
            let (lo, hi) = (gi.closed_min - margin, gi.closed_max + margin);
            let hit: Vec<usize> = (0..self.links.len())
                .filter(|&li| {
                    let link = self.links[li];
                    let p0 = self.cells[link.from as usize].origin;
                    let p1 = self.cells[link.to as usize].origin;
                    segment_aabb_intersect(p0, p1, lo, hi)
                })
                .collect();
            if hit.is_empty() {
                continue; // door crosses no link — not an obstruction the bots can hit
            }
            let idx = self.gates.push(Gate {
                obstruction: gi.obstruction,
                closed_origin: gi.closed_origin,
                activator: gi.activator,
                button_cell,
                aim: gi.button,
                shoot: gi.shoot,
            });
            for li in hit {
                self.gates.tag(li, idx);
            }
        }
    }

    pub fn gate_count(&self) -> usize {
        self.gates.len()
    }

    pub fn gate(&self, i: usize) -> &Gate {
        self.gates.item(i)
    }

    /// The gate (if any) whose shut door link `li` passes through.
    pub fn gate_of_link(&self, li: u32) -> Option<usize> {
        self.gates.index_of_link(li)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A graph with no geometry, to hang synthetic cells off (`add_cell` fills the grid `cells_in_box`
    /// reads). Building the real thing needs a BSP; the stamp only needs cells and a plat box.
    fn bare_graph() -> NavGraph {
        NavGraph::test_graph(Vec::new(), Vec::new())
    }

    /// A lift resting at `z = -24` whose raised surface delivers to a floor at `z = 200`, footprint
    /// ±32 about the origin. The stamp box is that footprint grown by the 16u half-width, spanning
    /// `[-24, 175]` in z — so `exit.z` (200) sits above it.
    fn plat() -> PlatInfo {
        PlatInfo {
            board: Vec3::new(0.0, 0.0, 0.0),
            exit: Vec3::new(0.0, 0.0, 200.0),
            entity: 7,
            fp_min: Vec2::splat(-32.0),
            fp_max: Vec2::splat(32.0),
            bottom: -24.0,
        }
    }

    /// The stamp covers the shaft a body would block — including the sliver just outside the brush that
    /// a 32u-wide player still overlaps — and stops short of the floor the lift delivers to, which is
    /// ordinary ground a bot must be free to stand on.
    #[test]
    fn stamp_covers_the_shaft_but_not_the_delivery_floor() {
        let mut g = bare_graph();
        let shaft = g.add_cell(Vec3::new(0.0, 0.0, 0.0)); // on the lift at rest
        let lip = g.add_cell(Vec3::new(40.0, 0.0, 0.0)); // 8u outside the brush — still in the way
        let away = g.add_cell(Vec3::new(200.0, 0.0, 0.0)); // open floor across the room
        let top = g.add_cell(Vec3::new(0.0, 0.0, 200.0)); // the floor the raised lift delivers to
        let cellar = g.add_cell(Vec3::new(0.0, 0.0, -100.0)); // under the shaft floor, not in it
        g.stamp_under_plat(&plat(), 0);

        assert_eq!(g.cell_under_plat(shaft), Some(0), "the lift's own resting spot is under it");
        assert_eq!(g.cell_under_plat(lip), Some(0), "a body 8u outside the brush still blocks it");
        assert_eq!(g.cell_under_plat(away), None, "open floor wrongly stamped");
        assert_eq!(g.cell_under_plat(top), None, "the delivery floor must stay open ground");
        assert_eq!(g.cell_under_plat(cellar), None, "a cell below the shaft floor is not in the way");
    }

    /// Routing pays to *enter* the shaft, but boarding the lift is untouched — the ride and jump-aboard
    /// links are what the stamped cells exist for.
    #[test]
    fn surcharge_prices_entry_but_spares_boarding_and_exits() {
        let mut g = bare_graph();
        let shaft = g.add_cell(Vec3::new(0.0, 0.0, 0.0));
        let away = g.add_cell(Vec3::new(200.0, 0.0, 0.0));
        let walk = |from: CellId, to: CellId| Link {
            from,
            to,
            kind: LinkKind::Walk,
            cost: 1.0,
        };
        g.push_link(walk(away, shaft)); // 0: into the shaft — priced
        g.push_link(walk(shaft, away)); // 1: back out — free, so the way out is never taxed
        let pi = g.plats.push(Plat {
            entity: 7,
            fp_min: Vec2::splat(-32.0),
            fp_max: Vec2::splat(32.0),
        });
        g.push_plat_link(walk(away, shaft), pi); // 2: a jump-aboard onto the same cell — exempt
        g.stamp_under_plat(&plat(), pi);
        g.surcharge_under_plat_links();

        assert_eq!(g.links[0].cost, 1.0 + UNDER_PLAT_EXTRA, "entering the shaft not priced");
        assert_eq!(g.links[1].cost, 1.0, "leaving the shaft wrongly priced");
        assert_eq!(g.links[2].cost, 1.0, "boarding the lift must keep its solved cost");
    }

    /// A map with no plats leaves every link at its solved cost (and the column empty — the all-clear
    /// default every `cell_under_plat` caller relies on).
    #[test]
    fn no_plats_leaves_costs_and_column_untouched() {
        let mut g = bare_graph();
        let a = g.add_cell(Vec3::ZERO);
        let b = g.add_cell(Vec3::new(32.0, 0.0, 0.0));
        g.push_link(Link {
            from: a,
            to: b,
            kind: LinkKind::Walk,
            cost: 1.0,
        });
        g.surcharge_under_plat_links();

        assert_eq!(g.links[0].cost, 1.0);
        assert_eq!(g.cell_under_plat(a), None);
        assert_eq!(g.cell_under_plat(b), None);
    }
}
