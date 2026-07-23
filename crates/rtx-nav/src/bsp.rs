// SPDX-License-Identifier: AGPL-3.0-or-later

//! Minimal BSP reader — the lumps the navmesh and the world queries need.
//!
//! Declarative `binrw` parsing in the style of the `bsp` crate at
//! `/Users/daniel/Development/home/bsp`, pared down to the lumps we use and extended with the one
//! that crate doesn't expose: `clipnodes`. The header reads `planes`, `clipnodes`, and `models`,
//! plus the render tree's `nodes` + `leaves` (for `pointcontents`); v29/HL clipnodes (`i16`
//! children) normalize to the BSP2 shape (`i32`) via a `From` conversion — same approach the crate
//! uses for nodes/leaves.
//!
//! Two hulls matter here. **Hull 1** is Quake's *standing player* collision hull: its clip planes
//! were already beveled by the player box at compile time, so a single **point** test against
//! hull 1 answers "would the player box collide here?" (classic `SV_HullPointContents`) — that's
//! what the navmesh reachability walks. **Hull 0** is the render tree, the only hull that carries
//! liquid/sky leaf contents; [`Bsp::pointcontents`] walks it (≡ mvdsv `SV_PointContents`) so the
//! game can answer `pointcontents`/world traces without an engine syscall, in either embodiment.
//! Everything else in the file (faces, lightmaps, textures, vis) is irrelevant to us.

use std::io::{Cursor, Seek, SeekFrom};

use binrw::{BinRead, BinReaderExt, BinResult};
use glam::Vec3;

/// `CONTENTS_SOLID`. The clip hulls (1/2) resolve to either `SOLID` or `CONTENTS_EMPTY` (`-1`);
/// the render hull (0) carries the liquids and sky below as well.
pub const CONTENTS_SOLID: i32 = -2;

/// The Quake point-contents values, as returned by [`Bsp::pointcontents`] (the render-hull walk,
/// bit-identical to the engine's `pointcontents`). Single-sourced here so the hazard classifier,
/// the world queries, and their tests all agree with the engine.
pub const CONTENTS_EMPTY: i32 = -1;
pub const CONTENTS_WATER: i32 = -3;
pub const CONTENTS_SLIME: i32 = -4;
pub const CONTENTS_LAVA: i32 = -5;
pub const CONTENTS_SKY: i32 = -6;

/// `DIST_EPSILON` — the crossing point is placed this far onto the near side of a plane during a
/// hull trace, so a bounce restart doesn't immediately re-collide with the surface it left.
const DIST_EPSILON: f32 = 0.03125;

/// A lump directory entry (`offset`, `size`).
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct Lump {
    offset: u32,
    size: u32,
}

/// BSP format magic. v29 (Quake) and v30 (Half-Life) store clipnode children as `i16`; BSP2
/// uses `i32`.
#[derive(BinRead, PartialEq)]
#[br(little)]
enum Version {
    #[br(magic(29u32))]
    Bsp29,
    #[br(magic(30u32))]
    BspHl,
    #[br(magic(0x3250_5342u32))] // "BSP2"
    Bsp2,
}

/// The lump directory, reading only the lumps the navmesh needs and skipping the rest with
/// `pad_before`. Besides the clip hull (`planes`, `clipnodes`, `models`) it also reads the render
/// tree (`nodes` + `leafs`, lumps 5 and 10) so `pointcontents` can answer which *liquid* a point is
/// in — the clip hull only resolves solid/empty.
#[derive(BinRead)]
#[br(little)]
struct Header {
    version: Version,
    entities: Lump, // lump 0 — the map's entity string
    planes: Lump,   // lump 1
    #[br(pad_before = 24)]
    nodes: Lump, // lump 5 (render tree) — skip textures/vertexes/vis
    #[br(pad_before = 24)]
    clipnodes: Lump, // lump 9 — skip texinfo/faces/lighting
    leafs: Lump,    // lump 10 (render leaf contents)
    #[br(pad_before = 24)]
    models: Lump, // lump 14 — skip marksurfaces/edges/surfedges
}

/// A BSP plane (`dplane_t`): `normal·p - dist`. `kind` is the axial type — `0/1/2` for an
/// axis-aligned plane (test just that coordinate), `>=3` for a general plane (dot product).
#[derive(BinRead, Clone, Copy)]
#[br(little)]
pub struct Plane {
    #[br(map = Vec3::from_array)]
    pub normal: Vec3,
    pub dist: f32,
    pub kind: i32,
}

/// `dclipnode_t` as stored in v29/HL (`i16` children).
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct ClipNodeV1 {
    plane: u32,
    children: [i16; 2],
}

/// A clip-hull BSP node, normalized to the BSP2 shape. `children[0]` is the front side
/// (`d >= 0`), `children[1]` the back; a negative child is a `CONTENTS_*` leaf, not a node index.
#[derive(BinRead, Clone, Copy)]
#[br(little)]
pub struct ClipNode {
    pub plane: u32,
    pub children: [i32; 2],
}

impl From<ClipNodeV1> for ClipNode {
    fn from(v: ClipNodeV1) -> Self {
        ClipNode {
            plane: v.plane,
            children: [v.children[0] as i32, v.children[1] as i32],
        }
    }
}

/// A brush model (`dmodel_t`): its bounding box, the render-tree headnode (`headnode[0]`, for
/// `pointcontents`), and the hull-1 headnode (`headnode[1]`); the trailing fields aren't read.
///
/// `models[0]` is the world. The rest are the map's **inline submodels** — the shapes its doors,
/// plats, buttons and triggers are made of, which entities claim by name as `"*1"`, `"*2"`, …
#[derive(BinRead, Clone, Copy)]
#[br(little)]
pub struct Model {
    /// Bounding box, in world coordinates.
    #[br(map = Vec3::from_array)]
    pub mins: Vec3,
    #[br(map = Vec3::from_array)]
    pub maxs: Vec3,
    /// `headnode[0]` — render (hull 0) tree root.
    #[br(pad_before = 12)] // skip origin (12)
    pub render_head: i32,
    /// `headnode[1]` — hull-1 (player clip) tree root. The `pad_after` completes `dmodel_t`'s 64
    /// bytes (`headnode[2..4]`, `visleafs`, `firstface`, `numfaces`) — load-bearing, because the
    /// models lump is read as a strided run and each record must consume its whole stride or every
    /// model after the first is read from the middle of its predecessor.
    #[br(pad_after = 20)]
    pub clip1: i32,
}

/// `sizeof(dmodel_t)` — the stride of the models lump. Unchanged between v29/HL and BSP2, which
/// widen the node/leaf/clipnode records but leave this one alone.
const MODEL_SIZE: usize = 64;

/// `dnode_t` render node (v29/HL): `i16` children. Only the split plane and children are needed for
/// a point-contents descent; the bbox and face range are skipped.
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct NodeV1 {
    plane: u32,
    #[br(pad_after = 16)] // skip mins[3]i16 + maxs[3]i16 + firstface u16 + numfaces u16
    children: [i16; 2],
}

/// `dnode_t` render node (BSP2): `i32` children, `f32` bbox.
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct NodeV2 {
    plane: u32,
    #[br(pad_after = 32)] // skip mins[3]f32 + maxs[3]f32 + firstface u32 + numfaces u32
    children: [i32; 2],
}

/// A render-tree node normalized to `i32` children. A non-negative child is a node index; a negative
/// child is a leaf, index `-1 - child`.
#[derive(Clone, Copy)]
struct RenderNode {
    plane: u32,
    children: [i32; 2],
}

impl From<NodeV1> for RenderNode {
    fn from(n: NodeV1) -> Self {
        RenderNode {
            plane: n.plane,
            children: [n.children[0] as i32, n.children[1] as i32],
        }
    }
}
impl From<NodeV2> for RenderNode {
    fn from(n: NodeV2) -> Self {
        RenderNode {
            plane: n.plane,
            children: n.children,
        }
    }
}

/// `dleaf_t` — only the leading `contents` (`CONTENTS_*`) is needed. v29/HL is 28 bytes, BSP2 44.
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct LeafV1 {
    #[br(pad_after = 24)]
    contents: i32,
}
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct LeafV2 {
    #[br(pad_after = 40)]
    contents: i32,
}

/// A render-node child as a clipnode child: a leaf becomes its contents, a node stays an index
/// (`Mod_MakeHull0`).
fn leaf_or_node(child: i32, leaf_contents: &[i32]) -> i32 {
    if child < 0 {
        leaf_contents
            .get((-1 - child) as usize)
            .copied()
            .unwrap_or(CONTENTS_SOLID)
    } else {
        child
    }
}

/// The subset of a parsed BSP the navmesh consumes.
pub struct Bsp {
    pub planes: Vec<Plane>,
    /// Hull 1's tree — the *standing player* hull, whose planes were beveled by the player box at
    /// compile time.
    pub clipnodes: Vec<ClipNode>,
    /// Hull 0's tree: the render nodes, rewritten into clipnode form (`Mod_MakeHull0`).
    ///
    /// Hull 0 is the **point** hull — no bevel, real surfaces — and it's the one to ask about
    /// shooting: QuakeC's `traceline` passes a zero-size box, which `SV_HullForEntity` reads as hull
    /// 0. Hull 1 is for *moving*. Answering sight from hull 1 has a bot believe every gap narrower
    /// than itself is a wall; measured over real maps, that hides 39–84% of the sightlines that
    /// genuinely exist (see `hull0_sees_further_than_hull1`).
    ///
    /// qbsp compiles four hulls: 0 a point, 1 the 32×32×56 player, 2 the 64×64×88 big monsters, 3
    /// unused. Navigation and a deathmatch client need the first two, so those are what's here.
    hull0_clipnodes: Vec<ClipNode>,
    /// `models[0].headnode[1]` — the world's hull-1 (player) clipnode tree root.
    pub hull1_headnode: i32,
    /// World model bounding box (float coords), the volume the navmesh voxelizes.
    pub mins: Vec3,
    pub maxs: Vec3,
    /// The map's entity string: the `{ "key" "value" … }` blocks a server spawns its items, doors,
    /// spawn points and triggers from. The navmesh doesn't read it — a server hands it entities
    /// already spawned — but a *client* has no server to do that, so it spawns them from here.
    pub entities: String,
    /// Every brush model in the map, world first. `models[N]` is what an entity naming itself
    /// `"*N"` is shaped like — how a door or a plat gets its bounds, and therefore how big a lift
    /// the navmesh thinks it is.
    pub models: Vec<Model>,
    /// The render (hull 0) tree — nodes + per-leaf contents + root — used only by `pointcontents`
    /// to tell which liquid (if any) a point is in. Private: callers go through `pointcontents`.
    render_nodes: Vec<RenderNode>,
    leaf_contents: Vec<i32>,
    render_headnode: i32,
}

impl Bsp {
    /// Parse the lumps the navmesh needs from a whole-file byte buffer. Returns `None` on an
    /// unsupported version or a malformed/truncated lump.
    pub fn parse(bytes: &[u8]) -> Option<Bsp> {
        let mut c = Cursor::new(bytes);
        let header: Header = c.read_le().ok()?;

        let planes = read_lump::<Plane>(&mut c, &header.planes).ok()?;
        let bsp2 = header.version == Version::Bsp2;
        let clipnodes = if bsp2 {
            read_lump::<ClipNode>(&mut c, &header.clipnodes).ok()?
        } else {
            read_lump_into::<ClipNodeV1, ClipNode>(&mut c, &header.clipnodes).ok()?
        };
        // Render tree (for liquid point-contents). v29/HL nodes are 24 B / leafs 28 B; BSP2 44 / 44.
        let render_nodes = if bsp2 {
            read_lump_stride::<NodeV2, RenderNode>(&mut c, &header.nodes, 44).ok()?
        } else {
            read_lump_stride::<NodeV1, RenderNode>(&mut c, &header.nodes, 24).ok()?
        };
        let leaf_contents: Vec<i32> = if bsp2 {
            read_lump_stride::<LeafV2, LeafV2>(&mut c, &header.leafs, 44)
                .ok()?
                .iter()
                .map(|l| l.contents)
                .collect()
        } else {
            read_lump_stride::<LeafV1, LeafV1>(&mut c, &header.leafs, 28)
                .ok()?
                .iter()
                .map(|l| l.contents)
                .collect()
        };

        let mut models = read_lump_stride::<Model, Model>(&mut c, &header.models, MODEL_SIZE).ok()?;
        // "Spread the mins / maxs by a pixel" — Quake's `Mod_LoadSubmodels`, verbatim, and not
        // cosmetic. qbsp *shrinks* every model's bounds by a unit per axis on the way out (it
        // removes the padding it added while compiling), and every engine expands them by a unit on
        // the way in. Skip it and a paper-thin brush — a teleport trigger, a flat door — arrives
        // inside-out (`mins.y > maxs.y`), so nothing is ever inside it: teleporters that teleport
        // nobody, doors with no extent.
        for m in &mut models {
            m.mins -= Vec3::ONE;
            m.maxs += Vec3::ONE;
        }
        let world = *models.first()?;

        // `Mod_MakeHull0`: the render tree, rewritten as clipnodes. A node child stays a node index;
        // a leaf child becomes that leaf's *contents*, because that's what a clipnode leaf is.
        let hull0_clipnodes = render_nodes
            .iter()
            .map(|n| ClipNode {
                plane: n.plane,
                children: [
                    leaf_or_node(n.children[0], &leaf_contents),
                    leaf_or_node(n.children[1], &leaf_contents),
                ],
            })
            .collect();

        // Latin-1, not UTF-8: the entity string is bytes, and a mapper's name with a high-bit
        // character in it shouldn't cost us the whole map.
        let (eo, es) = (header.entities.offset as usize, header.entities.size as usize);
        let entities = bytes
            .get(eo..eo.checked_add(es)?)?
            .iter()
            .take_while(|&&b| b != 0)
            .map(|&b| b as char)
            .collect();

        Some(Bsp {
            planes,
            clipnodes,
            hull1_headnode: world.clip1,
            mins: world.mins,
            maxs: world.maxs,
            entities,
            models,
            hull0_clipnodes,
            render_nodes,
            leaf_contents,
            render_headnode: world.render_head,
        })
    }

    /// A hand-built BSP for fixtures: `planes`, the hull-0 point-clip tree `nodes` rooted at
    /// `headnode`, and the `models` (`models[0]` is the world, traced from `headnode`; a submodel's
    /// `render_head` indexes into the same `nodes`). Hull 1 aliases hull 0, and the render
    /// point-contents tree is left empty (so `pointcontents` answers `CONTENTS_SOLID`). Lets a
    /// consuming crate build trace / submodel test worlds without a real `.bsp` on disk.
    pub fn synthetic(planes: Vec<Plane>, nodes: Vec<ClipNode>, headnode: i32, models: Vec<Model>) -> Bsp {
        Bsp {
            clipnodes: nodes.clone(),
            hull0_clipnodes: nodes,
            planes,
            hull1_headnode: headnode,
            render_headnode: headnode,
            mins: Vec3::splat(-4096.0),
            maxs: Vec3::splat(4096.0),
            entities: String::new(),
            models,
            render_nodes: Vec::new(),
            leaf_contents: Vec::new(),
        }
    }

    /// The bounds of inline submodel `n` (the shape of an entity whose model is `"*n"`), or `None`
    /// if the map has no such submodel.
    ///
    /// Note these are **world** coordinates, not an origin-relative box: a `func_door`'s brushes
    /// are modelled where the mapper drew them. Quake's `SV_SetModel` copies them straight into
    /// `mins`/`maxs` and leaves the entity at origin zero, which is why brush entities move by
    /// changing `origin` from a base of nothing.
    pub fn submodel(&self, n: usize) -> Option<Model> {
        self.models.get(n).copied()
    }

    /// The `CONTENTS_*` value at `p` in the render hull (hull 0) — the one that carries liquids
    /// (`SV_PointContents`). Descends `models[0]`'s node tree to a leaf and returns its contents.
    /// Out-of-range indices in a malformed file resolve to `CONTENTS_SOLID` and never panic.
    pub fn pointcontents(&self, p: Vec3) -> i32 {
        let mut num = self.render_headnode;
        while num >= 0 {
            let Some(node) = self.render_nodes.get(num as usize) else {
                return CONTENTS_SOLID;
            };
            let Some(plane) = self.planes.get(node.plane as usize) else {
                return CONTENTS_SOLID;
            };
            let d = if plane.kind < 3 {
                p[plane.kind as usize] - plane.dist
            } else {
                plane.normal.dot(p) - plane.dist
            };
            num = node.children[usize::from(d < 0.0)];
        }
        // A negative child is leaf `-1 - num`.
        self.leaf_contents
            .get((-1 - num) as usize)
            .copied()
            .unwrap_or(CONTENTS_SOLID)
    }

    /// Whether `p` is inside a liquid volume (water / slime / lava) per the render hull. Used by the
    /// navmesh to reject jump links whose takeoff is submerged — a submerged player can't jump (the
    /// jump input swims up).
    pub fn is_liquid_at(&self, p: Vec3) -> bool {
        matches!(self.pointcontents(p), CONTENTS_WATER | CONTENTS_SLIME | CONTENTS_LAVA)
    }

    /// Walk the hull rooted at `headnode`, returning the `CONTENTS_*` value at `p`
    /// (`SV_HullPointContents`). Out-of-range indices in a malformed file resolve to
    /// `CONTENTS_SOLID` — conservative, and never panics (this runs inside the engine).
    pub fn hull_contents(&self, headnode: i32, p: Vec3) -> i32 {
        self.contents_in(&self.clipnodes, headnode, p)
    }

    /// [`hull_contents`](Self::hull_contents) against an explicit tree.
    fn contents_in(&self, nodes: &[ClipNode], headnode: i32, p: Vec3) -> i32 {
        let mut num = headnode;
        while num >= 0 {
            let Some(node) = nodes.get(num as usize) else {
                return CONTENTS_SOLID;
            };
            let Some(plane) = self.planes.get(node.plane as usize) else {
                return CONTENTS_SOLID;
            };
            let d = if plane.kind < 3 {
                p[plane.kind as usize] - plane.dist
            } else {
                plane.normal.dot(p) - plane.dist
            };
            num = node.children[usize::from(d < 0.0)];
        }
        num
    }

    /// `CONTENTS_*` at `p` in the world's player hull (hull 1).
    pub fn hull1_contents(&self, p: Vec3) -> i32 {
        self.hull_contents(self.hull1_headnode, p)
    }

    /// Whether the player box centered at `p` would collide with world geometry.
    pub fn is_solid(&self, p: Vec3) -> bool {
        self.hull1_contents(p) == CONTENTS_SOLID
    }

    /// Whether a **point** at `p` is inside solid world geometry, tested against the render hull
    /// (hull 0) rather than the inflated player hull. A zero-size projectile (the rocket, spawned with
    /// `setsize 0 0`) collides on this hull, so it reaches the *true* floor/wall — ~24u below (16u
    /// nearer) than [`is_solid`](Self::is_solid)'s player-box surface. Used by the rocket-jump solve to
    /// detonate the shot where the engine actually would.
    pub fn is_point_solid(&self, p: Vec3) -> bool {
        self.pointcontents(p) == CONTENTS_SOLID
    }

    /// Trace the segment `p1 → p2` through the world's player hull (hull 1) — "would a player fit".
    /// Returns where it first hits solid (`fraction`/`endpos`) and the **surface normal** of the
    /// plane it struck (`plane_normal`, oriented against the segment), so a bouncing projectile can
    /// reflect off it. `fraction == 1` means the whole segment is clear. `start_solid` means `p1` was
    /// already inside solid. Pure over `planes`/`clipnodes`, no syscall. See [`trace_nodes`].
    pub fn hull1_trace(&self, p1: Vec3, p2: Vec3) -> HullTrace {
        trace_nodes(&self.planes, &self.clipnodes, self.hull1_headnode, p1, p2)
    }

    /// Trace the segment `p1 → p2` through the world's **point** hull (hull 0).
    ///
    /// This is what QuakeC's `traceline` does — it passes a zero-size box, and `SV_HullForEntity`
    /// reads that as hull 0 — so it's the right question for line of sight, for "can I shoot from
    /// here to there", and for anything else about a thing with no width. [`hull1_trace`] answers a
    /// different question: would a *player* fit.
    pub fn hull0_trace(&self, p1: Vec3, p2: Vec3) -> HullTrace {
        trace_nodes(&self.planes, &self.hull0_clipnodes, self.render_headnode, p1, p2)
    }

    /// Trace the segment `p1 → p2` through inline submodel `n`'s **point** hull (hull 0) — the door,
    /// plat or trigger shaped like `"*n"`. `Mod_MakeHull0` rewrote the whole render-node array into
    /// clipnodes once, and every submodel's `headnode[0]` indexes into that same array, so the trace
    /// shares [`hull0_clipnodes`](Self::hull0_clipnodes). Coordinates are the submodel's own frame
    /// (world brushes at their compiled position); the caller offsets by the entity's live origin. A
    /// missing submodel traces as open air (clear), never blocking.
    pub fn submodel_hull0_trace(&self, n: usize, p1: Vec3, p2: Vec3) -> HullTrace {
        let head = self.models.get(n).map_or(CONTENTS_EMPTY, |m| m.render_head);
        trace_nodes(&self.planes, &self.hull0_clipnodes, head, p1, p2)
    }
}

/// Which of the three trace states a subtree resolved to. Mirrors mvdsv's `RecursiveHullTrace`
/// (`TR_EMPTY`/`TR_SOLID`/`TR_BLOCKED`) — the FTE-style rewrite the engine actually runs, so our
/// own trace answers bit-for-bit what a server-side `traceline` would.
#[derive(PartialEq, Eq, Clone, Copy)]
enum Tr {
    Empty,
    Solid,
    Blocked,
}

/// Trace `p1 → p2` through a clip tree `(planes, nodes, headnode)` — the shared body of every hull
/// trace (world hull 0/1, an inline submodel, or a [`BoxHull`]). A port of mvdsv `CM_HullTrace` +
/// `RecursiveHullTrace` (cmodel.c): a segment that is solid the whole way reports
/// `start_solid == all_solid == true` with `endpos == p1` and **`fraction == 1`** (the id quirk the
/// engine preserves, not `0`); an impact places `endpos` exactly `DIST_EPSILON` shy of the plane
/// with no back-out drift. `plane_dist` is the struck plane's signed distance (oriented with
/// `plane_normal`), and `in_open`/`in_water` record whether the segment passed through empty / a
/// liquid leaf.
fn trace_nodes(planes: &[Plane], nodes: &[ClipNode], headnode: i32, p1: Vec3, p2: Vec3) -> HullTrace {
    let mut trace = HullTrace {
        fraction: 1.0,
        endpos: p2,
        plane_normal: Vec3::ZERO,
        plane_dist: 0.0,
        start_solid: false,
        all_solid: false,
        in_open: false,
        in_water: false,
    };
    let mut leafcount = 0;
    let check = recursive_hull_trace(planes, nodes, headnode, 0.0, 1.0, p1, p2, &mut trace, &mut leafcount);
    if check == Tr::Solid {
        // Whole path was solid. Match the engine: flag both, snap the endpoint back to the start,
        // and leave `fraction` at 1 (id left it there; mvdsv emulates it, so we do too).
        trace.start_solid = true;
        trace.all_solid = true;
        trace.endpos = p1;
    }
    trace
}

/// The recursion behind [`trace_nodes`]. Returns the subtree's [`Tr`] state; writes the impact into
/// `trace` on the transition from an empty near side to a solid far side.
#[allow(clippy::too_many_arguments)]
fn recursive_hull_trace(
    planes: &[Plane],
    nodes: &[ClipNode],
    num: i32,
    p1f: f32,
    p2f: f32,
    p1: Vec3,
    p2: Vec3,
    trace: &mut HullTrace,
    leafcount: &mut i32,
) -> Tr {
    // Leaf: a negative `num` is a CONTENTS_* value, not a node index.
    if num < 0 {
        *leafcount += 1;
        if num == CONTENTS_SOLID {
            // startsolid is the *first* leaf only — a segment that starts in open air and later
            // re-enters solid is an impact, not a start-in-solid.
            if *leafcount == 1 {
                trace.start_solid = true;
            }
            return Tr::Solid;
        }
        if num == CONTENTS_EMPTY {
            trace.in_open = true;
        } else {
            trace.in_water = true;
        }
        return Tr::Empty;
    }
    // Out-of-range clipnode index (a malformed file): treat as solid — conservative, never panics.
    let Some(node) = nodes.get(num as usize) else {
        return Tr::Solid;
    };
    let Some(plane) = planes.get(node.plane as usize) else {
        return Tr::Solid;
    };
    let (t1, t2) = if plane.kind < 3 {
        let k = plane.kind as usize;
        (p1[k] - plane.dist, p2[k] - plane.dist)
    } else {
        (plane.normal.dot(p1) - plane.dist, plane.normal.dot(p2) - plane.dist)
    };
    if t1 >= 0.0 && t2 >= 0.0 {
        return recursive_hull_trace(planes, nodes, node.children[0], p1f, p2f, p1, p2, trace, leafcount);
    }
    if t1 < 0.0 && t2 < 0.0 {
        return recursive_hull_trace(planes, nodes, node.children[1], p1f, p2f, p1, p2, trace, leafcount);
    }
    // The segment crosses this plane. Split at the *exact* intersection to recurse each half.
    let frac = (t1 / (t1 - t2)).clamp(0.0, 1.0);
    let midf = p1f + (p2f - p1f) * frac;
    let mid = p1 + (p2 - p1) * frac;
    let nearside = usize::from(t1 < t2); // the side `p1` lies on
    let check = recursive_hull_trace(
        planes,
        nodes,
        node.children[nearside],
        p1f,
        midf,
        p1,
        mid,
        trace,
        leafcount,
    );
    if check == Tr::Blocked {
        return check;
    }
    // Started in solid but have since reached open/water: stop, don't drive deeper.
    if check == Tr::Solid && (trace.in_open || trace.in_water) {
        return check;
    }
    let oldcheck = check;
    let check = recursive_hull_trace(
        planes,
        nodes,
        node.children[1 - nearside],
        midf,
        p2f,
        mid,
        p2,
        trace,
        leafcount,
    );
    if check == Tr::Empty || check == Tr::Blocked {
        return check;
    }
    if oldcheck != Tr::Empty {
        return check; // still in solid
    }
    // Near side empty, far side solid: this plane is the impact. Record the segment-facing normal.
    if nearside == 0 {
        trace.plane_normal = plane.normal;
        trace.plane_dist = plane.dist;
    } else {
        trace.plane_normal = -plane.normal;
        trace.plane_dist = -plane.dist;
    }
    // Put the final point DIST_EPSILON onto the near side (single split, no back-out iteration).
    let frac = if t1 < t2 {
        (t1 + DIST_EPSILON) / (t1 - t2)
    } else {
        (t1 - DIST_EPSILON) / (t1 - t2)
    }
    .clamp(0.0, 1.0);
    trace.fraction = p1f + (p2f - p1f) * frac;
    trace.endpos = p1 + (p2 - p1) * frac;
    Tr::Blocked
}

/// A bounding box turned into a six-plane clip tree — mvdsv `CM_InitBoxHull` + `CM_HullForBox`. A
/// sized entity with no BSP model (a `Solid::BBox`/`SlideBox` item or player) clips through one of
/// these, so a world trace treats every entity uniformly as a hull, exactly as the engine does.
pub struct BoxHull {
    planes: [Plane; 6],
    nodes: [ClipNode; 6],
}

/// Build the [`BoxHull`] for the box `mins..maxs` (both in the same frame the trace endpoints use).
pub fn box_hull(mins: Vec3, maxs: Vec3) -> BoxHull {
    let mut planes = [Plane {
        normal: Vec3::ZERO,
        dist: 0.0,
        kind: 0,
    }; 6];
    let mut nodes = [ClipNode {
        plane: 0,
        children: [0, 0],
    }; 6];
    for i in 0..6 {
        let side = i & 1;
        nodes[i].plane = i as u32;
        nodes[i].children[side] = CONTENTS_EMPTY;
        nodes[i].children[side ^ 1] = if i != 5 { (i + 1) as i32 } else { CONTENTS_SOLID };
        let mut n = [0.0f32; 3];
        n[i >> 1] = 1.0;
        planes[i].normal = Vec3::from_array(n);
        planes[i].kind = (i >> 1) as i32;
    }
    planes[0].dist = maxs.x;
    planes[1].dist = mins.x;
    planes[2].dist = maxs.y;
    planes[3].dist = mins.y;
    planes[4].dist = maxs.z;
    planes[5].dist = mins.z;
    BoxHull { planes, nodes }
}

impl BoxHull {
    /// Trace `p1 → p2` against this box (`0` is its root clipnode). Same semantics as [`trace_nodes`].
    pub fn trace(&self, p1: Vec3, p2: Vec3) -> HullTrace {
        trace_nodes(&self.planes, &self.nodes, 0, p1, p2)
    }
}

/// The result of a hull segment trace ([`Bsp::hull1_trace`]).
#[derive(Clone, Copy, Debug)]
pub struct HullTrace {
    /// Fraction of the segment traversed before impact (`1.0` = clear, and also `1.0` when the whole
    /// segment is solid — the id quirk, see [`trace_nodes`]).
    pub fraction: f32,
    /// The impact point (`p2` if clear, `p1` if wholly solid).
    pub endpos: Vec3,
    /// Surface normal at the impact, oriented against the incoming segment (`ZERO` if clear).
    pub plane_normal: Vec3,
    /// Signed distance of the struck plane, oriented with `plane_normal` (`0.0` if clear).
    pub plane_dist: f32,
    /// `p1` started inside solid.
    pub start_solid: bool,
    /// The whole segment was inside solid.
    pub all_solid: bool,
    /// The segment passed through an empty (`CONTENTS_EMPTY`) leaf at some point.
    pub in_open: bool,
    /// The segment passed through a liquid leaf (water/slime/lava) at some point.
    pub in_water: bool,
}

/// Read a lump as a `Vec<T>` (count derived from the lump size and `T`'s on-disk size).
fn read_lump<T>(c: &mut Cursor<&[u8]>, lump: &Lump) -> BinResult<Vec<T>>
where
    T: BinRead + for<'a> BinRead<Args<'a> = ()>,
{
    read_lump_into::<T, T>(c, lump)
}

/// Read a lump of `T` records, converting each `Into` the normalized `U` (used to widen v29
/// clipnodes to the BSP2 shape).
fn read_lump_into<T, U>(c: &mut Cursor<&[u8]>, lump: &Lump) -> BinResult<Vec<U>>
where
    T: BinRead + for<'a> BinRead<Args<'a> = ()>,
    U: From<T>,
{
    c.seek(SeekFrom::Start(lump.offset as u64))?;
    let count = lump.size as usize / std::mem::size_of::<T>();
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        out.push(T::read_le(c)?.into());
    }
    Ok(out)
}

/// Like [`read_lump_into`] but with an explicit on-disk record `stride`, for records whose Rust
/// `size_of` doesn't match the file layout because trailing fields are skipped with `pad_after`.
fn read_lump_stride<T, U>(c: &mut Cursor<&[u8]>, lump: &Lump, stride: usize) -> BinResult<Vec<U>>
where
    T: BinRead + for<'a> BinRead<Args<'a> = ()>,
    U: From<T>,
{
    c.seek(SeekFrom::Start(lump.offset as u64))?;
    let count = lump.size as usize / stride;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        out.push(T::read_le(c)?.into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Lump directory indices / element sizes, for the independent cross-check below.
    const LUMP_PLANES: usize = 1;
    const LUMP_CLIPNODES: usize = 9;
    const PLANE_SIZE: usize = 20;

    /// A hand-built one-plane hull: solid fills `x > 100`, empty behind it. Enough to exercise the
    /// segment trace's crossing math, impact normal, clear pass, and start-in-solid.
    fn wall_at_x100() -> Bsp {
        Bsp {
            planes: vec![Plane {
                normal: Vec3::new(1.0, 0.0, 0.0),
                dist: 100.0,
                kind: 0,
            }],
            // children[0] = front (x ≥ 100) = SOLID; children[1] = back (x < 100) = EMPTY (-1).
            clipnodes: vec![ClipNode {
                plane: 0,
                children: [CONTENTS_SOLID, -1],
            }],
            hull1_headnode: 0,
            mins: Vec3::splat(-256.0),
            maxs: Vec3::splat(256.0),
            entities: String::new(),
            hull0_clipnodes: Vec::new(),
            // One model (the world) and no submodels: this hull has no doors to be shaped like.
            models: vec![Model {
                mins: Vec3::splat(-256.0),
                maxs: Vec3::splat(256.0),
                render_head: 0,
                clip1: 0,
            }],
            // No render tree in this hand-built hull — pointcontents isn't exercised here.
            render_nodes: Vec::new(),
            leaf_contents: Vec::new(),
            render_headnode: 0,
        }
    }

    #[test]
    fn hull_trace_hits_wall_with_normal() {
        let bsp = wall_at_x100();
        // Into the +x wall from the empty side: stops at x≈100, normal faces back (−x).
        let tr = bsp.hull1_trace(Vec3::new(0.0, 0.0, 0.0), Vec3::new(200.0, 0.0, 0.0));
        assert!((tr.fraction - 0.5).abs() < 0.01, "fraction {}", tr.fraction);
        assert!((tr.endpos.x - 100.0).abs() < 0.5, "endpos {:?}", tr.endpos);
        assert!(
            (tr.plane_normal - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-4,
            "normal {:?}",
            tr.plane_normal
        );
        assert!(!tr.start_solid);

        // Fully in the empty half → clear.
        let clear = bsp.hull1_trace(Vec3::new(0.0, 0.0, 0.0), Vec3::new(50.0, 0.0, 0.0));
        assert_eq!(clear.fraction, 1.0);
        assert!(!clear.start_solid);

        // Starting inside the solid half is flagged.
        let inside = bsp.hull1_trace(Vec3::new(150.0, 0.0, 0.0), Vec3::new(160.0, 0.0, 0.0));
        assert!(inside.start_solid);
    }

    /// The mvdsv (`CM_HullTrace`) semantics the engine actually runs, so a server-side `traceline`
    /// cutover is a no-op: a wholly-solid segment reports start+all solid with `endpos == start` and
    /// `fraction == 1` (the id quirk, not `0`); an impact sits exactly `DIST_EPSILON` shy of the
    /// plane with no `frac -= 0.1` back-out drift; a leaf's `in_open`/`in_water` are tracked.
    #[test]
    fn hull_trace_mvdsv_semantics() {
        let bsp = wall_at_x100();

        // Wholly inside solid (both ends x > 100): start+all solid, endpos snaps to start, fraction 1.
        let (a, b) = (Vec3::new(150.0, 0.0, 0.0), Vec3::new(160.0, 0.0, 0.0));
        let solid = bsp.hull1_trace(a, b);
        assert!(solid.start_solid && solid.all_solid, "wholly-solid trace flags both");
        assert_eq!(solid.fraction, 1.0, "id quirk: fraction stays 1 on all-solid, not 0");
        assert_eq!(solid.endpos, a, "endpos snaps back to the start point");

        // Impact sits DIST_EPSILON shy on the near side — a single split, no iterative back-out.
        let hit = bsp.hull1_trace(Vec3::new(0.0, 0.0, 0.0), Vec3::new(200.0, 0.0, 0.0));
        assert!(
            hit.endpos.x < 100.0 && hit.endpos.x > 99.9,
            "endpos {} not just shy of 100",
            hit.endpos.x
        );
        // The signed plane distance is oriented with the (negated) normal: normal −x, dist −100.
        assert!((hit.plane_dist - -100.0).abs() < 1e-3, "plane_dist {}", hit.plane_dist);

        // A clear pass through the empty half records in_open (CONTENTS_EMPTY leaf), not in_water.
        let clear = bsp.hull1_trace(Vec3::new(0.0, 0.0, 0.0), Vec3::new(50.0, 0.0, 0.0));
        assert!(clear.in_open && !clear.in_water, "empty leaf → in_open");
        assert!(!clear.start_solid && !clear.all_solid);
    }

    /// A one-plane hull whose back side is a **liquid** leaf (not empty): a trace through it is clear
    /// (liquid isn't solid) but flags `in_water`, the field the mirror's waterlevel logic keys on.
    #[test]
    fn hull_trace_tracks_water_leaf() {
        let bsp = Bsp {
            planes: vec![Plane {
                normal: Vec3::new(1.0, 0.0, 0.0),
                dist: 100.0,
                kind: 0,
            }],
            // front (x ≥ 100) SOLID; back (x < 100) WATER (-3).
            clipnodes: vec![ClipNode {
                plane: 0,
                children: [CONTENTS_SOLID, CONTENTS_WATER],
            }],
            hull1_headnode: 0,
            mins: Vec3::splat(-256.0),
            maxs: Vec3::splat(256.0),
            entities: String::new(),
            hull0_clipnodes: Vec::new(),
            models: vec![Model {
                mins: Vec3::splat(-256.0),
                maxs: Vec3::splat(256.0),
                render_head: 0,
                clip1: 0,
            }],
            render_nodes: Vec::new(),
            leaf_contents: Vec::new(),
            render_headnode: 0,
        };
        let tr = bsp.hull1_trace(Vec3::new(0.0, 0.0, 0.0), Vec3::new(50.0, 0.0, 0.0));
        assert_eq!(tr.fraction, 1.0, "liquid is not solid — the segment is clear");
        assert!(tr.in_water && !tr.in_open, "the back leaf is water");
    }

    /// A `BoxHull` clips like the engine's `CM_HullForBox`: a segment entering the box stops at the
    /// face with the segment-facing normal; one passing outside is clear; a point inside is solid.
    #[test]
    fn box_hull_clips_a_bounding_box() {
        let hull = box_hull(Vec3::splat(-16.0), Vec3::splat(16.0));

        // Straight through the middle along +x: hits the −x face (x = −16), normal faces −x.
        let hit = hull.trace(Vec3::new(-100.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        // Enter at x ≈ −16 over a 200u span from x = −100 → fraction ≈ (−16 − −100)/200 = 0.42.
        assert!((hit.fraction - 0.42).abs() < 0.01, "fraction {}", hit.fraction);
        assert!(
            (hit.endpos.x - -16.0).abs() < 0.5 && hit.endpos.x > -16.5,
            "endpos {:?}",
            hit.endpos
        );
        assert!(
            (hit.plane_normal - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-4,
            "normal {:?}",
            hit.plane_normal
        );

        // Parallel but 50u to the side (|y| > 16): misses the box entirely.
        let miss = hull.trace(Vec3::new(-100.0, 50.0, 0.0), Vec3::new(100.0, 50.0, 0.0));
        assert_eq!(miss.fraction, 1.0, "a segment outside the box is clear");

        // A segment wholly inside the box is solid the whole way (start+all solid, endpos = start).
        let inside = hull.trace(Vec3::new(-4.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0));
        assert!(inside.start_solid && inside.all_solid, "inside the box is solid");
        assert_eq!(inside.endpos, Vec3::new(-4.0, 0.0, 0.0));
    }

    /// Hull 0 and hull 1 answer different questions, and a real map is the only place to see it.
    ///
    /// Hull 1's planes were pushed out by the player box at compile time, so a *point* traced through
    /// it is really asking "would a player fit". Hull 0 is the true surfaces. Anything about sight or
    /// shooting is a hull-0 question — QuakeC's `traceline` passes a zero-size box, which
    /// `SV_HullForEntity` reads as hull 0 — and answering it from hull 1 makes a bot believe every
    /// gap narrower than a player is a wall.
    #[test]
    fn hull0_sees_further_than_hull1() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            eprintln!("RTX_TEST_BSP not set; skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        assert!(!bsp.hull0_clipnodes.is_empty(), "hull 0 must have a tree of its own");

        // Sample sightlines across the map and count how many each hull calls clear. Both are
        // tracing the same segments; only the geometry differs.
        let (mut clear0, mut clear1, mut sampled) = (0, 0, 0);
        let span = bsp.maxs - bsp.mins;
        let mut seed = 12345u32;
        let mut rand = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 8) as f32 / (1 << 24) as f32
        };
        for _ in 0..40_000 {
            let a = bsp.mins + Vec3::new(rand(), rand(), rand()) * span;
            let b = bsp.mins + Vec3::new(rand(), rand(), rand()) * span;
            // Only sample from open air — a point inside rock says nothing about sightlines.
            if bsp.hull_contents(bsp.hull1_headnode, a) == CONTENTS_SOLID {
                continue;
            }
            sampled += 1;
            clear0 += u32::from(bsp.hull0_trace(a, b).fraction >= 1.0);
            clear1 += u32::from(bsp.hull1_trace(a, b).fraction >= 1.0);
        }
        eprintln!(
            "{path}: of {sampled} sightlines, hull0 clear {clear0}, hull1 clear {clear1} \
             (hull1 misses {:.0}%)",
            100.0 * (clear0.saturating_sub(clear1)) as f32 / clear0.max(1) as f32
        );

        assert!(sampled > 100, "not enough open space sampled to conclude anything");
        // The inflated hull can only ever block *more*: every one of its planes sits further into
        // the open than the surface it came from.
        assert!(
            clear0 >= clear1,
            "hull 0 must never see less than hull 1 — the player hull is the inflated one"
        );
        // And the difference is not academic: on a real map hull 1 refuses a large share of the
        // sightlines that genuinely exist — half of them on catalyst — which is a bot declining
        // shots it could take. Only assert that where enough sightlines exist to mean anything; a
        // small, boxy map sampled this way yields a handful and proves nothing either way.
        if clear0 > 10 {
            assert!(clear0 > clear1, "the two hulls should visibly disagree on an open map");
        }
    }

    /// The entity string is what a client spawns its shadow world from — no entities, no items, no
    /// navmesh goals, no bots. Check it's real text with the blocks a spawner expects.
    #[test]
    fn reads_the_entity_lump_of_a_real_bsp() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            eprintln!("RTX_TEST_BSP not set; skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");

        let opens = bsp.entities.matches('{').count();
        let closes = bsp.entities.matches('}').count();
        eprintln!("{path}: {} bytes of entities, {opens} blocks", bsp.entities.len());

        assert!(opens > 1, "a real map has a worldspawn and then some");
        assert_eq!(opens, closes, "every block closes");
        assert!(
            bsp.entities.contains("\"classname\" \"worldspawn\""),
            "worldspawn comes first"
        );
        // A deathmatch map has somewhere to spawn. This is the field the shadow world lives or dies
        // on: no spawn points means no bots.
        assert!(
            bsp.entities.contains("info_player_deathmatch") || bsp.entities.contains("info_player_start"),
            "no spawn points in the entity string — is the lump offset right?"
        );
        // The lump is NUL-terminated in the file; the terminator must not survive into the string,
        // or a tokenizer would trip on it.
        assert!(!bsp.entities.contains('\0'));
    }

    /// A map's inline submodels are the shapes of its doors, plats and triggers. An entity with no
    /// bounds is an entity the navmesh sizes wrong — a plat whose `pos2` is computed from its own
    /// height would land at the wrong floor — so parsing them is not cosmetic.
    #[test]
    fn reads_inline_submodels_of_a_real_bsp() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            eprintln!("RTX_TEST_BSP not set; skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");

        // Cross-check the count against an independent header read.
        let lump_models = 14;
        let base = 4 + lump_models * 8;
        let size = u32::from_le_bytes(bytes[base + 4..base + 8].try_into().unwrap()) as usize;
        assert_eq!(bsp.models.len(), size / MODEL_SIZE);
        assert!(!bsp.models.is_empty(), "every map has at least the world");

        // models[0] is the world, and is what the top-level fields were taken from.
        assert_eq!(bsp.submodel(0).map(|m| m.mins), Some(bsp.mins));
        assert_eq!(bsp.submodel(0).map(|m| m.clip1), Some(bsp.hull1_headnode));
        assert!(
            bsp.submodel(bsp.models.len()).is_none(),
            "and asking past the end is None"
        );

        // Every box must come out the right way round. It's the `Mod_LoadSubmodels` spread that
        // makes that true: on disk, qbsp's shrink leaves a paper-thin brush inside-out (catalyst's
        // `*1` is y 515..514, a 1-unit-thin teleport trigger), and only the +1 expansion turns it
        // back into the unit of extent the mapper drew. Without it nothing is ever inside such a
        // trigger. A stride bug looks different again — denormal garbage (`1e-42`) — which the
        // world-bounds check below catches.
        eprintln!("{}: {} models", path, bsp.models.len());
        for (i, m) in bsp.models.iter().enumerate() {
            assert!(
                m.mins.is_finite() && m.maxs.is_finite(),
                "submodel *{i}: {:?}..{:?}",
                m.mins,
                m.maxs
            );
            assert!(
                (m.maxs - m.mins).min_element() > 0.0,
                "submodel *{i} is inside-out: {:?}..{:?} — was the load-time spread applied?",
                m.mins,
                m.maxs
            );
            // Inside Quake's map limit. Note this deliberately isn't "inside the world's box":
            // `models[0]` bounds only the *world* brushes, so a submodel can legitimately sit
            // outside them (dm1's `*8` is 128 units past the world's y). What it can't do is land
            // outside the coordinate system, which is what misread bytes look like.
            assert!(
                m.mins.cmpge(Vec3::splat(-4096.0)).all() && m.maxs.cmple(Vec3::splat(4096.0)).all(),
                "submodel *{i} {:?}..{:?} is outside the ±4096 map limit — wrong stride?",
                m.mins,
                m.maxs
            );
        }
    }

    /// Parse a real map (path from `RTX_TEST_BSP`, e.g. a Quake `dm2.bsp`) and check the parser
    /// holds together: lump counts match an independent header read, the hull-1 root is a real
    /// node, the contents walk always terminates on a valid leaf, and a grid sample finds *both*
    /// solid and open space (a smoke test that plane dists / normals aren't garbage — a sign/scale
    /// bug makes everything one or the other). Skipped when the env var isn't set, so CI is green
    /// without a map checked in.
    #[test]
    fn parses_real_bsp() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            eprintln!("RTX_TEST_BSP not set; skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");

        // Independent header read: lump sizes / element size must equal the parsed counts.
        // Clipnode width depends on the format (v29/HL = 8 bytes, BSP2 = 12).
        let version = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let clipnode_size = if version == 0x3250_5342 { 12 } else { 8 };
        let lump = |i: usize| {
            let base = 4 + i * 8;
            let off = u32::from_le_bytes(bytes[base..base + 4].try_into().unwrap()) as usize;
            let size = u32::from_le_bytes(bytes[base + 4..base + 8].try_into().unwrap()) as usize;
            (off, size)
        };
        assert_eq!(bsp.planes.len(), lump(LUMP_PLANES).1 / PLANE_SIZE);
        assert_eq!(bsp.clipnodes.len(), lump(LUMP_CLIPNODES).1 / clipnode_size);
        assert!(!bsp.planes.is_empty());

        // A real map has a clip hull rooted at an actual clipnode; a degenerate test map
        // (q1_cube, 0 clipnodes) has none — nothing more to check there.
        if bsp.clipnodes.is_empty() {
            eprintln!("{path}: no clipnodes (degenerate map); skipping hull checks");
            return;
        }
        assert!(bsp.hull1_headnode >= 0);
        assert!((bsp.hull1_headnode as usize) < bsp.clipnodes.len());

        // Grid-sample the world bbox; every walk must end on a valid CONTENTS_* leaf.
        let (mut solid, mut open) = (0u32, 0u32);
        let n = 24;
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    let t = |i: i32| (i as f32 + 0.5) / n as f32;
                    let p = bsp.mins + (bsp.maxs - bsp.mins) * Vec3::new(t(ix), t(iy), t(iz));
                    let c = bsp.hull1_contents(p);
                    // Clip hull leaves are only ever SOLID (-2) or EMPTY (-1).
                    assert!((-2..=-1).contains(&c), "bad contents {c}");
                    match c {
                        CONTENTS_SOLID => solid += 1,
                        _ => open += 1,
                    }
                }
            }
        }
        assert!(solid > 0 && open > 0, "degenerate hull: solid={solid} open={open}");
        eprintln!(
            "{path}: {} planes, {} clipnodes, sample solid={solid} open={open}",
            bsp.planes.len(),
            bsp.clipnodes.len()
        );
    }
}
