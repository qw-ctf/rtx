// SPDX-License-Identifier: AGPL-3.0-or-later

//! Minimal BSP reader — only the lumps the navmesh needs from the **player clip hull**.
//!
//! Declarative `binrw` parsing in the style of the `bsp` crate at
//! `/Users/daniel/Development/home/bsp`, pared down to the three lumps navigation needs and
//! extended with the one that crate doesn't expose: `clipnodes`. The header skips straight to
//! `planes`, `clipnodes`, and `models` with `pad_before`, and v29/HL clipnodes (`i16` children)
//! normalize to the BSP2 shape (`i32`) via a `From` conversion — same approach the crate uses
//! for nodes/leaves.
//!
//! Hull 1 is Quake's *standing player* collision hull: its clip planes were already beveled by
//! the player box at compile time, so a single **point** test against hull 1 answers "would the
//! player box collide here?" (classic `SV_HullPointContents`). Everything else in the file
//! (rendering BSP tree, faces, lightmaps, textures, vis) is irrelevant to navigation.

use std::io::{Cursor, Seek, SeekFrom};

use binrw::{BinRead, BinReaderExt, BinResult};
use glam::Vec3;

/// `CONTENTS_SOLID` — the only clip-hull leaf value we test against. Clip hulls (1/2) resolve
/// to either `SOLID` or `CONTENTS_EMPTY` (`-1`); water/lava/sky live in the render hull (0),
/// which this minimal parser doesn't read.
pub const CONTENTS_SOLID: i32 = -2;

/// The Quake liquid/empty point-contents values (as returned by the engine's `pointcontents`).
/// The clip hull this parser reads never yields them — they come from a caller-supplied `contents`
/// oracle — but they're single-sourced here so the hazard classifier and its tests agree with the
/// engine. (`SOLID` above is the render-hull `-2`; `SKY` `-6` is unused by navigation.)
pub const CONTENTS_EMPTY: i32 = -1;
pub const CONTENTS_WATER: i32 = -3;
pub const CONTENTS_SLIME: i32 = -4;
pub const CONTENTS_LAVA: i32 = -5;

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
    #[br(pad_before = 8)]
    planes: Lump, // lump 1
    #[br(pad_before = 24)]
    nodes: Lump, // lump 5 (render tree) — skip textures/vertexes/vis
    #[br(pad_before = 24)]
    clipnodes: Lump, // lump 9 — skip texinfo/faces/lighting
    leafs: Lump, // lump 10 (render leaf contents)
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

/// The world model (`models[0]`): its bounding box, the render-tree headnode (`headnode[0]`, for
/// `pointcontents`), and the hull-1 headnode (`headnode[1]`); the trailing fields aren't read.
#[derive(BinRead)]
#[br(little)]
struct Model {
    #[br(map = Vec3::from_array)]
    mins: Vec3,
    #[br(map = Vec3::from_array)]
    maxs: Vec3,
    #[br(pad_before = 12)] // skip origin (12)
    render_head: i32,      // headnode[0] — render (hull 0) tree root
    clip1: i32,            // headnode[1] — hull-1 (player clip) tree root
}

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
        RenderNode { plane: n.plane, children: [n.children[0] as i32, n.children[1] as i32] }
    }
}
impl From<NodeV2> for RenderNode {
    fn from(n: NodeV2) -> Self {
        RenderNode { plane: n.plane, children: n.children }
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

/// The subset of a parsed BSP the navmesh consumes.
pub struct Bsp {
    pub planes: Vec<Plane>,
    pub clipnodes: Vec<ClipNode>,
    /// `models[0].headnode[1]` — the world's hull-1 (player) clipnode tree root.
    pub hull1_headnode: i32,
    /// World model bounding box (float coords), the volume the navmesh voxelizes.
    pub mins: Vec3,
    pub maxs: Vec3,
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
            read_lump_stride::<LeafV2, LeafV2>(&mut c, &header.leafs, 44).ok()?.iter().map(|l| l.contents).collect()
        } else {
            read_lump_stride::<LeafV1, LeafV1>(&mut c, &header.leafs, 28).ok()?.iter().map(|l| l.contents).collect()
        };

        c.seek(SeekFrom::Start(header.models.offset as u64)).ok()?;
        let model: Model = c.read_le().ok()?;

        Some(Bsp {
            planes,
            clipnodes,
            hull1_headnode: model.clip1,
            mins: model.mins,
            maxs: model.maxs,
            render_nodes,
            leaf_contents,
            render_headnode: model.render_head,
        })
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
        self.leaf_contents.get((-1 - num) as usize).copied().unwrap_or(CONTENTS_SOLID)
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
        let mut num = headnode;
        while num >= 0 {
            let Some(node) = self.clipnodes.get(num as usize) else {
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

    /// Trace the segment `p1 → p2` through the world's player hull (hull 1) — a port of
    /// `SV_RecursiveHullCheck`. Returns where it first hits solid (`fraction`/`endpos`) and the
    /// **surface normal** of the plane it struck (`plane_normal`, oriented against the segment), so a
    /// bouncing projectile can reflect off it. `fraction == 1` means the whole segment is clear.
    /// `start_solid` means `p1` was already inside solid. Pure over `planes`/`clipnodes`, no syscall.
    pub fn hull1_trace(&self, p1: Vec3, p2: Vec3) -> HullTrace {
        let mut trace = HullTrace {
            fraction: 1.0,
            endpos: p2,
            plane_normal: Vec3::ZERO,
            start_solid: false,
            all_solid: true,
        };
        self.recursive_hull_check(self.hull1_headnode, 0.0, 1.0, p1, p2, &mut trace);
        trace
    }

    /// The recursion behind [`hull1_trace`] (`SV_RecursiveHullCheck`). Returns `true` while the
    /// segment stays out of solid; `false` once it records an impact.
    fn recursive_hull_check(&self, num: i32, p1f: f32, p2f: f32, p1: Vec3, p2: Vec3, trace: &mut HullTrace) -> bool {
        // Leaf: negative `num` is a CONTENTS_* value, not a node index.
        if num < 0 {
            if num != CONTENTS_SOLID {
                trace.all_solid = false;
            } else {
                trace.start_solid = true;
            }
            return true;
        }
        let Some(node) = self.clipnodes.get(num as usize) else {
            trace.start_solid = true;
            return true;
        };
        let Some(plane) = self.planes.get(node.plane as usize) else {
            trace.start_solid = true;
            return true;
        };
        let (t1, t2) = if plane.kind < 3 {
            let k = plane.kind as usize;
            (p1[k] - plane.dist, p2[k] - plane.dist)
        } else {
            (plane.normal.dot(p1) - plane.dist, plane.normal.dot(p2) - plane.dist)
        };
        if t1 >= 0.0 && t2 >= 0.0 {
            return self.recursive_hull_check(node.children[0], p1f, p2f, p1, p2, trace);
        }
        if t1 < 0.0 && t2 < 0.0 {
            return self.recursive_hull_check(node.children[1], p1f, p2f, p1, p2, trace);
        }
        // The segment crosses this plane — split it `DIST_EPSILON` onto the near side.
        let mut frac = if t1 < 0.0 {
            (t1 + DIST_EPSILON) / (t1 - t2)
        } else {
            (t1 - DIST_EPSILON) / (t1 - t2)
        }
        .clamp(0.0, 1.0);
        let mut midf = p1f + (p2f - p1f) * frac;
        let mut mid = p1 + (p2 - p1) * frac;
        let side = usize::from(t1 < 0.0);
        // Walk the near side first.
        if !self.recursive_hull_check(node.children[side], p1f, midf, p1, mid, trace) {
            return false;
        }
        // If the far side isn't solid at the crossing, keep going into it.
        if self.hull_contents(node.children[side ^ 1], mid) != CONTENTS_SOLID {
            return self.recursive_hull_check(node.children[side ^ 1], midf, p2f, mid, p2, trace);
        }
        if trace.all_solid {
            return false; // never got out of the solid area
        }
        // Impact: the far side is solid. Record the (segment-facing) plane normal.
        trace.plane_normal = if side == 0 { plane.normal } else { -plane.normal };
        // Back the impact point out of solid if the epsilon split left it just inside.
        while self.hull_contents(self.hull1_headnode, mid) == CONTENTS_SOLID {
            frac -= 0.1;
            if frac < 0.0 {
                trace.fraction = midf;
                trace.endpos = mid;
                return false;
            }
            midf = p1f + (p2f - p1f) * frac;
            mid = p1 + (p2 - p1) * frac;
        }
        trace.fraction = midf;
        trace.endpos = mid;
        false
    }
}

/// The result of a hull segment trace ([`Bsp::hull1_trace`]).
#[derive(Clone, Copy, Debug)]
pub struct HullTrace {
    /// Fraction of the segment traversed before impact (`1.0` = clear).
    pub fraction: f32,
    /// The impact point (or `p2` if clear).
    pub endpos: Vec3,
    /// Surface normal at the impact, oriented against the incoming segment (`ZERO` if clear).
    pub plane_normal: Vec3,
    /// `p1` started inside solid.
    pub start_solid: bool,
    /// The whole segment was inside solid.
    pub all_solid: bool,
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
