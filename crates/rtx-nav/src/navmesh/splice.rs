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
