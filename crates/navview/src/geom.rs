// SPDX-License-Identifier: AGPL-3.0-or-later

//! Two pure geometry builders for the viewer, both producing flat vertex buffers ready to upload:
//!
//!  * [`parse_render_mesh`] reads the *render* lumps of a Quake BSP (vertexes/faces/edges/surfedges)
//!    that `rtx_nav::bsp` deliberately skips, and fan-triangulates the world model into grey
//!    triangles. Self-contained — it re-reads the lump directory rather than touching the nav parser.
//!  * [`nav_lines`] walks a built [`NavGraph`] and turns each link into a colored line (a true
//!    ballistic arc for the jump/rocket kinds), plus a short tick under every cell.
//!
//! Both output `#[repr(C)]` `bytemuck::Pod` vertices so `gpu.rs` can `cast_slice` them straight into
//! a vertex buffer.

use std::io::{Cursor, Seek, SeekFrom};

use binrw::{BinRead, BinReaderExt};
use glam::Vec3;

use rtx_nav::navmesh::{arc_point, LinkKind, NavGraph, DOUBLE_ARC_PEAK, GRID, JUMP_APEX};

/// Quake gravity constant (`sv_gravity` default) — the parabola coefficient for the ballistic link
/// arcs. Matches the value the navmesh solved the arcs with.
const GRAVITY: f32 = 800.0;

// --- GPU vertex formats ------------------------------------------------------------------------

/// A world-model triangle vertex: position + face normal (flat shading, non-indexed).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
}

/// A navmesh line vertex: position + RGB color (one draw as `LineList`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LineVertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

/// The triangulated world geometry plus its bounds (for framing the camera).
pub struct RenderMesh {
    pub vertices: Vec<MeshVertex>,
    pub mins: Vec3,
    pub maxs: Vec3,
}

// --- BSP render-lump parsing -------------------------------------------------------------------

/// The 124-byte BSP lump directory: a version word followed by 15 `{offset, size}` entries. Common
/// to BSP29 (`29`), Half-Life BSP30 (`30`), and BSP2 (`"BSP2"`); only the per-record widths of the
/// face/edge lumps differ between them (handled below).
#[derive(BinRead)]
#[br(little)]
struct DirHeader {
    version: u32,
    lumps: [Lump; 15],
}

#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct Lump {
    offset: u32,
    size: u32,
}

// Lump indices (identical across the three versions).
const LUMP_VERTEXES: usize = 3;
const LUMP_FACES: usize = 7;
const LUMP_EDGES: usize = 12;
const LUMP_SURFEDGES: usize = 13;
const LUMP_MODELS: usize = 14;

const BSP2_MAGIC: u32 = u32::from_le_bytes(*b"BSP2");

// On-disk record sizes (bytes). binrw reads packed, so these are exact.
const VERTEX_SIZE: usize = 12; // [f32; 3]
const SURFEDGE_SIZE: usize = 4; // i32
const EDGE_SIZE_V1: usize = 4; // [u16; 2]
const EDGE_SIZE_V2: usize = 8; // [u32; 2]
const FACE_SIZE_V1: usize = 20;
const FACE_SIZE_V2: usize = 28;
const MODEL_SIZE: usize = 64;

/// BSP29 / BSP30 face (`dface_t`): 16-bit plane/side/numedges/texinfo.
#[derive(BinRead)]
#[br(little)]
struct FaceV1 {
    _plane: u16,
    _side: u16,
    first_edge: i32,
    num_edges: u16,
    _texinfo: u16,
    _styles: [u8; 4],
    _light_ofs: i32,
}

/// BSP2 face (`bsp2_dface_t`): every index field widened to 32-bit.
#[derive(BinRead)]
#[br(little)]
struct FaceV2 {
    _plane: u32,
    _side: u32,
    first_edge: i32,
    num_edges: u32,
    _texinfo: u32,
    _styles: [u8; 4],
    _light_ofs: i32,
}

/// The one field pair we need from every face: where its edge loop starts and how long it is.
#[derive(Clone, Copy)]
struct Face {
    first_edge: i32,
    num_edges: u32,
}

impl From<FaceV1> for Face {
    fn from(f: FaceV1) -> Self {
        Face { first_edge: f.first_edge, num_edges: f.num_edges as u32 }
    }
}
impl From<FaceV2> for Face {
    fn from(f: FaceV2) -> Self {
        Face { first_edge: f.first_edge, num_edges: f.num_edges }
    }
}

/// The world model (`models[0]`) record — only its face range and bbox are needed.
#[derive(BinRead)]
#[br(little)]
struct ModelRec {
    _mins: [f32; 3],
    _maxs: [f32; 3],
    #[br(pad_before = 32)] // skip origin[3] (12) + headnode[4] (16) + visleafs (4)
    first_face: i32,
    num_faces: i32,
}

/// Read `size / rec_size` fixed records of `T` from `bytes[lump.offset..]`. Returns `None` if the
/// lump size isn't a whole number of records (a version/format mismatch) or runs off the buffer.
fn read_records<T>(bytes: &[u8], lump: Lump, rec_size: usize, what: &str) -> Option<Vec<T>>
where
    T: BinRead,
    for<'a> <T as BinRead>::Args<'a>: Default,
{
    if lump.size as usize % rec_size != 0 {
        eprintln!("navview: {what} lump size {} is not a multiple of {rec_size} — not this BSP version", lump.size);
        return None;
    }
    let count = lump.size as usize / rec_size;
    let mut cur = Cursor::new(bytes);
    cur.seek(SeekFrom::Start(lump.offset as u64)).ok()?;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        out.push(cur.read_le::<T>().ok()?);
    }
    Some(out)
}

/// Parse a Quake BSP (v29 / v30 / BSP2) and fan-triangulate its **world model** into grey triangles.
/// Returns `None` on an unsupported version or malformed lump — never panics on bad input, so a bad
/// drag&drop just reports and is ignored. Submodels (doors/plats/triggers) are intentionally skipped:
/// without texinfo we can't tell trigger volumes from doors, and floating trigger boxes are noise.
pub fn parse_render_mesh(bytes: &[u8]) -> Option<RenderMesh> {
    let header: DirHeader = Cursor::new(bytes).read_le().ok()?;
    let (edge_size, face_size, bsp2) = match header.version {
        29 | 30 => (EDGE_SIZE_V1, FACE_SIZE_V1, false),
        BSP2_MAGIC => (EDGE_SIZE_V2, FACE_SIZE_V2, true),
        v if v == u32::from_le_bytes(*b"2PSB") => {
            eprintln!("navview: '2PSB' (old BSP2 variant) is not supported");
            return None;
        }
        v => {
            eprintln!("navview: unrecognized BSP version {v:#x}");
            return None;
        }
    };

    let vertexes: Vec<[f32; 3]> = read_records(bytes, header.lumps[LUMP_VERTEXES], VERTEX_SIZE, "vertexes")?;
    let surfedges: Vec<i32> = read_records(bytes, header.lumps[LUMP_SURFEDGES], SURFEDGE_SIZE, "surfedges")?;
    let models: Vec<ModelRec> = read_records(bytes, header.lumps[LUMP_MODELS], MODEL_SIZE, "models")?;

    // Edges and faces widen between v1 and BSP2 — read the right shape, normalize.
    let edges: Vec<[u32; 2]> = if bsp2 {
        read_records::<[u32; 2]>(bytes, header.lumps[LUMP_EDGES], edge_size, "edges")?
    } else {
        read_records::<[u16; 2]>(bytes, header.lumps[LUMP_EDGES], edge_size, "edges")?
            .into_iter()
            .map(|e| [e[0] as u32, e[1] as u32])
            .collect()
    };
    let faces: Vec<Face> = if bsp2 {
        read_records::<FaceV2>(bytes, header.lumps[LUMP_FACES], face_size, "faces")?
            .into_iter()
            .map(Face::from)
            .collect()
    } else {
        read_records::<FaceV1>(bytes, header.lumps[LUMP_FACES], face_size, "faces")?
            .into_iter()
            .map(Face::from)
            .collect()
    };

    let world = models.first()?;
    let face_lo = world.first_face.max(0) as usize;
    let face_hi = (world.first_face + world.num_faces).max(0) as usize;

    let vertex_at = |vi: u32| -> Option<Vec3> {
        let v = vertexes.get(vi as usize)?;
        Some(Vec3::new(v[0], v[1], v[2]))
    };

    let mut vertices: Vec<MeshVertex> = Vec::new();
    let (mut mins, mut maxs) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));

    'faces: for face in faces.get(face_lo..face_hi.min(faces.len()))?.iter() {
        if face.num_edges < 3 {
            continue;
        }
        // Resolve the face's ordered vertex loop through the surfedge indirection.
        let mut loop_verts: Vec<Vec3> = Vec::with_capacity(face.num_edges as usize);
        for e in 0..face.num_edges {
            let Some(&se) = surfedges.get(face.first_edge as usize + e as usize) else {
                continue 'faces; // corrupt face — skip it, don't abort the whole map
            };
            // A positive surfedge uses edge[0]→edge[1]; a negative one runs the edge backwards.
            let vi = if se >= 0 {
                edges.get(se as usize).map(|edge| edge[0])
            } else {
                edges.get((-se) as usize).map(|edge| edge[1])
            };
            let Some(v) = vi.and_then(vertex_at) else {
                continue 'faces;
            };
            loop_verts.push(v);
        }
        if loop_verts.len() < 3 {
            continue;
        }
        let normal = newell_normal(&loop_verts);
        for v in &loop_verts {
            mins = mins.min(*v);
            maxs = maxs.max(*v);
        }
        // Fan-triangulate: (v0, v[i], v[i+1]).
        let n = [normal.x, normal.y, normal.z];
        for i in 1..loop_verts.len() - 1 {
            for &v in &[loop_verts[0], loop_verts[i], loop_verts[i + 1]] {
                vertices.push(MeshVertex { pos: [v.x, v.y, v.z], normal: n });
            }
        }
    }

    if vertices.is_empty() {
        eprintln!("navview: BSP parsed but produced no world faces");
        return None;
    }
    Some(RenderMesh { vertices, mins, maxs })
}

/// Robust planar-polygon normal via Newell's method — independent of triangulation and stable for
/// near-degenerate first triangles (unlike a single edge cross product). Returns a unit vector, or
/// `+Z` for a degenerate loop (harmless: shading uses `|n·l|`).
fn newell_normal(verts: &[Vec3]) -> Vec3 {
    let mut n = Vec3::ZERO;
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    n.normalize_or(Vec3::Z)
}

// --- Navmesh overlay ---------------------------------------------------------------------------

/// The number of [`LinkKind`] variants — the width of the viewer's path-type toggle set.
pub const NUM_LINK_KINDS: usize = 10;

/// Every link kind in a stable display order (the row order of the path-type toggles). The compiler
/// checks the length against [`NUM_LINK_KINDS`], and [`link_color`]'s exhaustive match guards the set.
pub const LINK_KINDS: [LinkKind; NUM_LINK_KINDS] = [
    LinkKind::Walk,
    LinkKind::Step,
    LinkKind::Drop,
    LinkKind::JumpGap,
    LinkKind::DoubleJump,
    LinkKind::SpeedJump,
    LinkKind::Plat,
    LinkKind::Teleport,
    LinkKind::Hook,
    LinkKind::RocketJump,
];

/// Index of a kind within [`LINK_KINDS`] — the slot of its visibility flag.
pub fn kind_index(kind: LinkKind) -> usize {
    LINK_KINDS.iter().position(|&k| k == kind).expect("every LinkKind is in LINK_KINDS")
}

/// Human-readable label for a link kind (the checkbox text).
pub fn kind_label(kind: LinkKind) -> &'static str {
    match kind {
        LinkKind::Walk => "Walk (surface)",
        LinkKind::Step => "Step",
        LinkKind::Drop => "Drop",
        LinkKind::JumpGap => "Jump",
        LinkKind::DoubleJump => "Double jump",
        LinkKind::SpeedJump => "Speed jump",
        LinkKind::Plat => "Plat",
        LinkKind::Teleport => "Teleport",
        LinkKind::Hook => "Hook",
        LinkKind::RocketJump => "Rocket jump",
    }
}

// The per-LinkKind colors. Kept in one exhaustive match so adding a LinkKind forces a color choice.
pub fn link_color(kind: LinkKind) -> [f32; 3] {
    match kind {
        LinkKind::Walk => [0.25, 0.85, 0.25],       // green
        LinkKind::Step => [0.85, 0.85, 0.20],       // yellow
        LinkKind::Drop => [0.95, 0.55, 0.15],       // orange
        LinkKind::JumpGap => [0.95, 0.20, 0.20],    // red
        LinkKind::DoubleJump => [0.90, 0.25, 0.90], // magenta
        LinkKind::SpeedJump => [1.00, 0.45, 0.75],  // pink
        LinkKind::Plat => [0.20, 0.90, 0.90],       // cyan (needs live entities; not built offline)
        LinkKind::Teleport => [0.25, 0.45, 1.00],   // blue (needs live entities)
        LinkKind::Hook => [0.60, 0.30, 1.00],       // purple
        LinkKind::RocketJump => [1.00, 1.00, 1.00],  // white
    }
}

/// Number of straight segments used to approximate each ballistic arc.
const ARC_SEGMENTS: usize = 16;

/// Standing feet sit this far below the cell origin (player `mins.z`) — the floor height.
const FEET_DROP: f32 = 24.0;

/// Build the navmesh **line** overlay: one colored polyline per non-`Walk` link (a true parabola for
/// the ballistic kinds, a straight segment otherwise), emitted as `LineList` pairs. `Walk` links are
/// the flat-ground connectivity and are shown as the filled surface ([`nav_surface`]) instead. Links
/// whose kind isn't in `visible` are skipped, so a viewer can toggle path types.
pub fn nav_lines(graph: &NavGraph, visible: &[bool; NUM_LINK_KINDS]) -> Vec<LineVertex> {
    let mut out: Vec<LineVertex> = Vec::new();

    for (li, link) in graph.links.iter().enumerate() {
        if link.kind == LinkKind::Walk || !visible[kind_index(link.kind)] {
            continue;
        }
        let li = li as u32;
        let a = graph.cell_origin(link.from);
        let b = graph.cell_origin(link.to);
        let color = link_color(link.kind);
        match link.kind {
            LinkKind::JumpGap => push_arc(&mut out, arc_pts(a, b, JUMP_APEX), color),
            LinkKind::DoubleJump => push_arc(&mut out, arc_pts(a, b, DOUBLE_ARC_PEAK), color),
            LinkKind::SpeedJump => {
                // The `from` cell is the runway start; the real leap begins at the solved takeoff.
                if let Some(sj) = graph.speed_jump_of_link(li) {
                    push_seg(&mut out, a, sj.takeoff, color);
                    push_arc(&mut out, ballistic_pts(sj.takeoff, b, sj.airtime), color);
                } else {
                    push_seg(&mut out, a, b, color);
                }
            }
            LinkKind::RocketJump => {
                // `from` → blast position is the jump+aim; then the post-blast parabola to the target.
                if let Some(rj) = graph.rocket_jump_of_link(li) {
                    push_seg(&mut out, a, rj.pos_blast, color);
                    push_arc(&mut out, launch_pts(rj.pos_blast, rj.v0, rj.airtime), color);
                } else {
                    push_seg(&mut out, a, b, color);
                }
            }
            _ => push_seg(&mut out, a, b, color),
        }
    }
    out
}

/// Build the filled **walkable surface**: a flat green quad (two triangles) on the floor under each
/// cell, sized to the nav grid so adjacent cells tile edge-to-edge. Lifted 1u off the floor to avoid
/// z-fighting the world mesh. Returns `TriangleList` vertices (drawn translucent).
pub fn nav_surface(graph: &NavGraph) -> Vec<LineVertex> {
    let color = link_color(LinkKind::Walk);
    let h = GRID * 0.5;
    let mut out: Vec<LineVertex> = Vec::with_capacity(graph.cells.len() * 6);
    for cell in &graph.cells {
        let o = cell.origin;
        let z = o.z - FEET_DROP + 1.0;
        let corner = |dx: f32, dy: f32| LineVertex { pos: [o.x + dx * h, o.y + dy * h, z], color };
        let (a, b, c, d) = (corner(-1.0, -1.0), corner(1.0, -1.0), corner(1.0, 1.0), corner(-1.0, 1.0));
        out.extend_from_slice(&[a, b, c, a, c, d]);
    }
    out
}

fn push_seg(out: &mut Vec<LineVertex>, a: Vec3, b: Vec3, color: [f32; 3]) {
    out.push(LineVertex { pos: a.to_array(), color });
    out.push(LineVertex { pos: b.to_array(), color });
}

/// Emit a polyline (a slice of points) as consecutive `LineList` segment pairs.
fn push_arc(out: &mut Vec<LineVertex>, pts: Vec<Vec3>, color: [f32; 3]) {
    for w in pts.windows(2) {
        push_seg(out, w[0], w[1], color);
    }
}

/// Sample the shared jump parabola (`rtx_nav::navmesh::arc_point`) — the exact curve the build
/// cleared — into `ARC_SEGMENTS + 1` points.
fn arc_pts(a: Vec3, b: Vec3, apex: f32) -> Vec<Vec3> {
    (0..=ARC_SEGMENTS).map(|i| arc_point(a, b, apex, i as f32 / ARC_SEGMENTS as f32)).collect()
}

/// A gravity parabola from `a` to `b` over airtime `t_land`: xy is linear, z fits both endpoints with
/// the initial vertical speed `vz0 = (Δz + ½·g·T²)/T`. Used for the speed-jump leap.
fn ballistic_pts(a: Vec3, b: Vec3, t_land: f32) -> Vec<Vec3> {
    if t_land <= 0.0 {
        return vec![a, b];
    }
    let vz0 = (b.z - a.z + 0.5 * GRAVITY * t_land * t_land) / t_land;
    (0..=ARC_SEGMENTS)
        .map(|i| {
            let f = i as f32 / ARC_SEGMENTS as f32;
            let t = t_land * f;
            let xy = a.truncate().lerp(b.truncate(), f);
            Vec3::new(xy.x, xy.y, a.z + vz0 * t - 0.5 * GRAVITY * t * t)
        })
        .collect()
}

/// A ballistic path launched from `p0` with velocity `v0` under gravity for `t_land` seconds. Used
/// for the rocket-jump continuation after the blast.
fn launch_pts(p0: Vec3, v0: Vec3, t_land: f32) -> Vec<Vec3> {
    if t_land <= 0.0 {
        return vec![p0];
    }
    (0..=ARC_SEGMENTS)
        .map(|i| {
            let t = t_land * (i as f32 / ARC_SEGMENTS as f32);
            p0 + v0 * t - Vec3::new(0.0, 0.0, 0.5 * GRAVITY * t * t)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parse a real BSP end-to-end when `NAVVIEW_TEST_BSP` points at one: assert the render lumps
    /// triangulate into a whole number of triangles with finite bounds. Skipped when unset.
    #[test]
    fn parses_real_bsp_render_lumps() {
        let Ok(path) = std::env::var("NAVVIEW_TEST_BSP") else {
            eprintln!("NAVVIEW_TEST_BSP unset — skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let mesh = parse_render_mesh(&bytes).expect("parse render mesh");
        assert!(mesh.vertices.len() >= 3, "no triangles");
        assert_eq!(mesh.vertices.len() % 3, 0, "vertices not a whole number of triangles");
        assert!(mesh.mins.is_finite() && mesh.maxs.is_finite(), "bad bounds");
        assert!(mesh.maxs.cmpge(mesh.mins).all(), "inverted bounds");
        eprintln!(
            "{}: {} triangles, bounds {:?}..{:?}",
            path,
            mesh.vertices.len() / 3,
            mesh.mins,
            mesh.maxs
        );
    }

    /// Build a real navmesh and check both overlay builders: the surface is exactly six vertices
    /// (two triangles) per cell, and the lines are non-empty and exclude `Walk` (shown as surface).
    /// Skipped when `NAVVIEW_TEST_BSP` is unset.
    #[test]
    fn builds_nav_overlay() {
        use rtx_nav::navmesh::{build_navmesh, RocketJumpParams, SpeedJumpParams};
        let Ok(path) = std::env::var("NAVVIEW_TEST_BSP") else {
            eprintln!("NAVVIEW_TEST_BSP unset — skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let (_bsp, graph) = build_navmesh(
            bytes,
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            true,
            Some(SpeedJumpParams { gravity: 800.0, accel: 10.0, maxspeed: 320.0 }),
            Some(RocketJumpParams { gravity: 800.0, rj_extra: 0.0 }),
        )
        .expect("build navmesh");

        let surface = nav_surface(&graph);
        assert_eq!(surface.len(), graph.cells.len() * 6, "surface should be 2 triangles per cell");
        assert!(surface.iter().all(|v| v.color == link_color(LinkKind::Walk)), "surface tiles are Walk-green");

        let all_visible = [true; NUM_LINK_KINDS];
        let lines = nav_lines(&graph, &all_visible);
        assert!(lines.len() % 2 == 0, "lines are LineList pairs");
        assert!(!lines.is_empty(), "a real map should have non-Walk links (steps/jumps/drops)");
        assert!(
            !lines.iter().any(|v| v.color == link_color(LinkKind::Walk)),
            "Walk links must be excluded from the line overlay (they are the surface)"
        );

        // Hiding a kind removes exactly its lines; hiding all leaves nothing.
        let none_visible = [false; NUM_LINK_KINDS];
        assert!(nav_lines(&graph, &none_visible).is_empty(), "no kinds visible → no lines");
        eprintln!("{}: {} cells → {} surface verts, {} line verts", path, graph.cells.len(), surface.len(), lines.len());
    }
}
