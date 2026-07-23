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

use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::{arc_point, LinkKind, NavGraph, DOUBLE_ARC_PEAK, GRID, JUMP_APEX};
use rtx_nav::qphys::STEP_HEIGHT;

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

/// A flat colored triangle/line vertex: position + RGB color. Used for nav lines (`LineList`) and
/// for the translucent liquid surfaces (`TriangleList`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LineVertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

/// The triangulated world geometry plus its bounds (for framing the camera). Opaque solid surfaces
/// go in `vertices` (backface-culled grey); liquid surfaces go in `water` (drawn additive), sky is
/// dropped entirely.
pub struct RenderMesh {
    pub vertices: Vec<MeshVertex>,
    pub water: Vec<LineVertex>,
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
const LUMP_PLANES: usize = 1;
const LUMP_TEXTURES: usize = 2;
const LUMP_VERTEXES: usize = 3;
const LUMP_TEXINFO: usize = 6;
const LUMP_FACES: usize = 7;
const LUMP_EDGES: usize = 12;
const LUMP_SURFEDGES: usize = 13;
const LUMP_MODELS: usize = 14;

const BSP2_MAGIC: u32 = u32::from_le_bytes(*b"BSP2");

// On-disk record sizes (bytes). binrw reads packed, so these are exact. Planes and texinfo keep the
// same layout across BSP29/30/BSP2 (only faces/edges widen).
const PLANE_SIZE: usize = 20; // normal[3] f32 + dist f32 + type i32
const TEXINFO_SIZE: usize = 40; // vecs[2][4] f32 (32) + miptex i32 + flags i32
const VERTEX_SIZE: usize = 12; // [f32; 3]
const SURFEDGE_SIZE: usize = 4; // i32
const EDGE_SIZE_V1: usize = 4; // [u16; 2]
const EDGE_SIZE_V2: usize = 8; // [u32; 2]
const FACE_SIZE_V1: usize = 20;
const FACE_SIZE_V2: usize = 28;
const MODEL_SIZE: usize = 64;

/// A BSP plane — only its normal is needed (to orient faces for backface culling and lighting).
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct PlaneRec {
    normal: [f32; 3],
    _dist: f32,
    _kind: i32,
}

/// A `texinfo` record — only its `miptex` index (into the textures lump) is needed, to look up the
/// surface's texture name and classify it (sky / liquid / solid).
#[derive(BinRead, Clone, Copy)]
#[br(little)]
struct TexInfo {
    #[br(pad_before = 32)] // skip vecs[2][4]
    miptex: i32,
    _flags: i32,
}

/// How a surface should be drawn, decided by its texture name (Quake convention: `sky*` is the
/// skybox, `*name` is a turbulent liquid).
#[derive(PartialEq, Clone, Copy)]
enum SurfKind {
    Solid,
    Sky,
    Water,
    Lava,
    Slime,
}

fn classify(name: &str) -> SurfKind {
    if name.starts_with("sky") {
        SurfKind::Sky
    } else if let Some(rest) = name.strip_prefix('*') {
        if rest.contains("lava") {
            SurfKind::Lava
        } else if rest.contains("slime") {
            SurfKind::Slime
        } else {
            SurfKind::Water
        }
    } else {
        SurfKind::Solid
    }
}

/// Additive tint for a liquid surface (the color the water/lava/slime adds over the grey scene).
fn liquid_tint(kind: SurfKind) -> [f32; 3] {
    match kind {
        SurfKind::Lava => [0.85, 0.30, 0.08],
        SurfKind::Slime => [0.35, 0.70, 0.18],
        _ => [0.18, 0.42, 0.90], // water
    }
}

/// Read the texture-name table: the miptex lump is a count, then per-texture offsets, then each
/// `dmiptex_t` whose first 16 bytes are the null-terminated name. Missing entries (`ofs < 0`, animated
/// placeholders) become empty strings. Best-effort — a malformed table just yields fewer names.
fn read_texture_names(bytes: &[u8], lump: Lump) -> Vec<String> {
    let mut cur = Cursor::new(bytes);
    if cur.seek(SeekFrom::Start(lump.offset as u64)).is_err() {
        return Vec::new();
    }
    let count = match cur.read_le::<i32>() {
        Ok(n) if n >= 0 => n as usize,
        _ => return Vec::new(),
    };
    let mut offsets = Vec::with_capacity(count);
    for _ in 0..count {
        match cur.read_le::<i32>() {
            Ok(o) => offsets.push(o),
            Err(_) => break,
        }
    }
    offsets
        .into_iter()
        .map(|o| {
            if o < 0 {
                return String::new();
            }
            let start = lump.offset as usize + o as usize;
            bytes
                .get(start..start + 16)
                .map(|b| {
                    let end = b.iter().position(|&c| c == 0).unwrap_or(16);
                    String::from_utf8_lossy(&b[..end]).to_ascii_lowercase()
                })
                .unwrap_or_default()
        })
        .collect()
}

/// BSP29 / BSP30 face (`dface_t`): 16-bit plane/side/numedges/texinfo.
#[derive(BinRead)]
#[br(little)]
struct FaceV1 {
    plane: u16,
    side: u16,
    first_edge: i32,
    num_edges: u16,
    texinfo: u16,
    _styles: [u8; 4],
    _light_ofs: i32,
}

/// BSP2 face (`bsp2_dface_t`): every index field widened to 32-bit.
#[derive(BinRead)]
#[br(little)]
struct FaceV2 {
    plane: u32,
    side: u32,
    first_edge: i32,
    num_edges: u32,
    texinfo: u32,
    _styles: [u8; 4],
    _light_ofs: i32,
}

/// The fields we need from every face: its edge loop, the plane it lies on (+ which side, for the
/// outward normal), and its texinfo (to look up the texture name).
#[derive(Clone, Copy)]
struct Face {
    plane: u32,
    side: u32,
    first_edge: i32,
    num_edges: u32,
    texinfo: u32,
}

impl From<FaceV1> for Face {
    fn from(f: FaceV1) -> Self {
        Face {
            plane: f.plane as u32,
            side: f.side as u32,
            first_edge: f.first_edge,
            num_edges: f.num_edges as u32,
            texinfo: f.texinfo as u32,
        }
    }
}
impl From<FaceV2> for Face {
    fn from(f: FaceV2) -> Self {
        Face {
            plane: f.plane,
            side: f.side,
            first_edge: f.first_edge,
            num_edges: f.num_edges,
            texinfo: f.texinfo,
        }
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
    if !(lump.size as usize).is_multiple_of(rec_size) {
        eprintln!(
            "navview: {what} lump size {} is not a multiple of {rec_size} — not this BSP version",
            lump.size
        );
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
    let planes: Vec<PlaneRec> = read_records(bytes, header.lumps[LUMP_PLANES], PLANE_SIZE, "planes")?;
    let texinfos: Vec<TexInfo> = read_records(bytes, header.lumps[LUMP_TEXINFO], TEXINFO_SIZE, "texinfo")?;
    let tex_names = read_texture_names(bytes, header.lumps[LUMP_TEXTURES]);

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
    // The texture name behind a face (via texinfo → miptex), for sky/liquid classification.
    let tex_of = |face: &Face| -> &str {
        texinfos
            .get(face.texinfo as usize)
            .and_then(|t| usize::try_from(t.miptex).ok())
            .and_then(|mi| tex_names.get(mi))
            .map(String::as_str)
            .unwrap_or("")
    };

    let mut vertices: Vec<MeshVertex> = Vec::new();
    let mut water: Vec<LineVertex> = Vec::new();
    let (mut mins, mut maxs) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));

    'faces: for face in faces.get(face_lo..face_hi.min(faces.len()))?.iter() {
        if face.num_edges < 3 {
            continue;
        }
        let kind = classify(tex_of(face));
        if kind == SurfKind::Sky {
            continue; // sky isn't a surface a viewer wants — drop it
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
        for v in &loop_verts {
            mins = mins.min(*v);
            maxs = maxs.max(*v);
        }

        if kind != SurfKind::Solid {
            // Liquid: emit flat tinted triangles into the water buffer (drawn additive, double-sided
            // — no winding fixup needed).
            let color = liquid_tint(kind);
            fan(&loop_verts, |v| {
                water.push(LineVertex {
                    pos: v.to_array(),
                    color,
                })
            });
            continue;
        }

        // Solid: orient by the face's plane so backface culling works. The front (visible) normal
        // is the plane normal, flipped when the face is on the plane's back side. Wind the fan CCW
        // about it (reverse the loop if the surfedge order came out the other way).
        let front_n = planes
            .get(face.plane as usize)
            .map(|p| {
                let n = Vec3::from_array(p.normal);
                if face.side != 0 {
                    -n
                } else {
                    n
                }
            })
            .map(|n| n.normalize_or(Vec3::Z))
            .unwrap_or_else(|| newell_normal(&loop_verts));
        if newell_normal(&loop_verts).dot(front_n) < 0.0 {
            loop_verts.reverse();
        }
        let n = front_n.to_array();
        fan(&loop_verts, |v| {
            vertices.push(MeshVertex {
                pos: v.to_array(),
                normal: n,
            })
        });
    }

    if vertices.is_empty() && water.is_empty() {
        eprintln!("navview: BSP parsed but produced no world faces");
        return None;
    }
    Some(RenderMesh {
        vertices,
        water,
        mins,
        maxs,
    })
}

/// Fan-triangulate a vertex loop, calling `emit` for each of the `(v0, v[i], v[i+1])` corners.
fn fan(loop_verts: &[Vec3], mut emit: impl FnMut(Vec3)) {
    for i in 1..loop_verts.len() - 1 {
        emit(loop_verts[0]);
        emit(loop_verts[i]);
        emit(loop_verts[i + 1]);
    }
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
    LINK_KINDS
        .iter()
        .position(|&k| k == kind)
        .expect("every LinkKind is in LINK_KINDS")
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
        LinkKind::RocketJump => [1.00, 1.00, 1.00], // white
    }
}

/// Number of straight segments used to approximate each ballistic arc.
const ARC_SEGMENTS: usize = 16;

/// Standing feet sit this far below the cell origin (player `mins.z`) — the floor height.
const FEET_DROP: f32 = 24.0;

/// Brightness of a link's color at its `from` end; it ramps to full at the `to` end. This makes each
/// directed link read as a dim→bright flow in its direction of travel — the path's direction cue.
const DIR_DIM: f32 = 0.25;

/// Build the navmesh **line** overlay: one colored polyline per non-`Walk` link (a true parabola for
/// the ballistic kinds, a straight segment otherwise), emitted as `LineList` pairs shaded dim→bright
/// from `from` to `to` so the travel direction is visible. `Walk` links are the flat-ground
/// connectivity and are shown as the filled surface ([`nav_surface`]) instead. Links whose kind isn't
/// in `visible` are skipped, so a viewer can toggle path types.
pub fn nav_lines(graph: &NavGraph, visible: &[bool; NUM_LINK_KINDS]) -> Vec<LineVertex> {
    let mut out: Vec<LineVertex> = Vec::new();

    for (li, link) in graph.links.iter().enumerate() {
        if link.kind == LinkKind::Walk || !visible[kind_index(link.kind)] {
            continue;
        }
        let li = li as u32;
        let a = graph.cell_origin(link.from);
        let b = graph.cell_origin(link.to);
        // The ordered points from `from` to `to` — a true arc for the ballistic kinds, prefixed by
        // the runway/jump-up straight for the two-phase speed- and rocket-jumps.
        let path: Vec<Vec3> = match link.kind {
            LinkKind::JumpGap => arc_pts(a, b, JUMP_APEX),
            LinkKind::DoubleJump => arc_pts(a, b, DOUBLE_ARC_PEAK),
            LinkKind::SpeedJump => match graph.speed_jump_of_link(li) {
                Some(sj) => std::iter::once(a)
                    .chain(ballistic_pts(sj.takeoff, b, sj.airtime))
                    .collect(),
                None => vec![a, b],
            },
            LinkKind::RocketJump => match graph.rocket_jump_of_link(li) {
                Some(rj) => std::iter::once(a)
                    .chain(launch_pts(rj.pos_blast, rj.v0, rj.airtime))
                    .collect(),
                None => vec![a, b],
            },
            _ => vec![a, b],
        };
        push_gradient(&mut out, &path, link_color(link.kind));
    }
    out
}

/// Number of sub-quads per axis a cell tile is divided into when trimming it to the supported
/// footprint — 4 = 8u sub-quads at the 32u grid pitch.
const SURF_SUB: i32 = 4;

/// Build the filled **walkable surface**: green quads on the floor under each cell, lifted 1u to avoid
/// z-fighting the world mesh. Each 32u cell tile is subdivided into `SURF_SUB`² sub-quads and only the
/// sub-quads whose centre is genuinely standable are emitted — floor within a step below the origin
/// (hull-1 solid) and open at origin height (not buried in a wall/riser). This is the same physical
/// test the build's `ground_along` enforces, so the surface stops at the real hull footprint (≤16u of
/// honest overhang past a visual ledge) instead of padding out a full grid tile. `TriangleList`,
/// translucent.
pub fn nav_surface(graph: &NavGraph, bsp: &Bsp) -> Vec<LineVertex> {
    let color = link_color(LinkKind::Walk);
    let full = GRID * 0.5; // tile half-extent
    let sub = GRID / SURF_SUB as f32; // sub-quad side
    let hs = sub * 0.5; // sub-quad half-extent
    let mut out: Vec<LineVertex> = Vec::with_capacity(graph.cells.len() * 6);
    for cell in &graph.cells {
        let o = cell.origin;
        let z = o.z - FEET_DROP + 1.0;
        for iy in 0..SURF_SUB {
            for ix in 0..SURF_SUB {
                let cx = o.x - full + hs + ix as f32 * sub;
                let cy = o.y - full + hs + iy as f32 * sub;
                let supported = bsp.is_solid(Vec3::new(cx, cy, o.z - (STEP_HEIGHT + 4.0)))
                    && !bsp.is_solid(Vec3::new(cx, cy, o.z + 1.0));
                if !supported {
                    continue;
                }
                let corner = |dx: f32, dy: f32| LineVertex {
                    pos: [cx + dx * hs, cy + dy * hs, z],
                    color,
                };
                let (a, b, c, d) = (
                    corner(-1.0, -1.0),
                    corner(1.0, -1.0),
                    corner(1.0, 1.0),
                    corner(-1.0, 1.0),
                );
                out.extend_from_slice(&[a, b, c, a, c, d]);
            }
        }
    }
    out
}

// --- live overlay (the running game's current route + bot, via the control channel) ------------

/// Bright red for the bot's current path.
const PATH_COLOR: [f32; 3] = [1.0, 0.15, 0.12];
/// Yellow opaque faces for the live bot cube.
const BOT_COLOR: [f32; 3] = [0.95, 0.78, 0.08];
/// Dark outline for the bot cube's wireframe edges (contrast against the yellow faces).
const BOT_EDGE: [f32; 3] = [0.12, 0.08, 0.0];

/// Filled red tiles marking the bot's current route — one 32u quad per route cell, straight from the
/// game's leg origins, lifted 3u so they sit just over the green walkable surface (which is at +1).
pub fn path_tiles(origins: &[Vec3]) -> Vec<LineVertex> {
    let hs = GRID * 0.5;
    let mut out = Vec::with_capacity(origins.len() * 6);
    for o in origins {
        let z = o.z - FEET_DROP + 3.0;
        let corner = |dx: f32, dy: f32| LineVertex {
            pos: [o.x + dx * hs, o.y + dy * hs, z],
            color: PATH_COLOR,
        };
        let (a, b, c, d) = (
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        );
        out.extend_from_slice(&[a, b, c, a, c, d]);
    }
    out
}

/// Thick red ballistic arcs for the route's rocket-/speed-jump legs — an approximate parabola from
/// takeoff to landing (the game reports only endpoints), drawn as several offset polylines so it reads
/// as a fat line, since wgpu can't widen a `LineList`.
pub fn path_arcs(legs: &[(Vec3, Vec3)]) -> Vec<LineVertex> {
    let mut out = Vec::new();
    for &(a, b) in legs {
        let apex = a.z.max(b.z) + JUMP_APEX; // a plausible leap height above the higher endpoint
        let arc = arc_pts(a, b, apex);
        // A "plus" cross-section of offset copies — center plus ±3u in x/y and +3u up — to fake width.
        for off in [
            Vec3::ZERO,
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(-3.0, 0.0, 0.0),
            Vec3::new(0.0, 3.0, 0.0),
            Vec3::new(0.0, -3.0, 0.0),
            Vec3::new(0.0, 0.0, 3.0),
        ] {
            for w in arc.windows(2) {
                out.push(LineVertex {
                    pos: (w[0] + off).to_array(),
                    color: PATH_COLOR,
                });
                out.push(LineVertex {
                    pos: (w[1] + off).to_array(),
                    color: PATH_COLOR,
                });
            }
        }
    }
    out
}

/// The 8 corners of the QW player hull (`mins -16,-16,-24` / `maxs 16,16,32`) centred on `origin`,
/// indexed by bit 0=x, 1=y, 2=z picking lo/hi per axis.
fn bot_corners(origin: Vec3) -> [Vec3; 8] {
    let lo = origin + Vec3::new(-16.0, -16.0, -24.0);
    let hi = origin + Vec3::new(16.0, 16.0, 32.0);
    let c = |i: usize| {
        Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        )
    };
    [c(0), c(1), c(2), c(3), c(4), c(5), c(6), c(7)]
}

/// The live bot as an **opaque** box the size of the QW player hull, centred on `origin` — 6 faces × 2
/// triangles, solid yellow. Pair with [`bot_box`] for the wireframe edges over it. `TriangleList`.
pub fn bot_faces(origin: Vec3) -> Vec<LineVertex> {
    let v = bot_corners(origin);
    // Each face is 4 corner indices in ring order; drawn double-sided so winding doesn't matter.
    const FACES: [[usize; 4]; 6] = [
        [0, 1, 3, 2], // z-lo
        [4, 6, 7, 5], // z-hi
        [0, 4, 5, 1], // y-lo
        [2, 3, 7, 6], // y-hi
        [0, 2, 6, 4], // x-lo
        [1, 5, 7, 3], // x-hi
    ];
    let mut out = Vec::with_capacity(36);
    for f in FACES {
        let q = [v[f[0]], v[f[1]], v[f[2]], v[f[3]]];
        for idx in [0, 1, 2, 0, 2, 3] {
            out.push(LineVertex {
                pos: q[idx].to_array(),
                color: BOT_COLOR,
            });
        }
    }
    out
}

/// The live bot's 12 wireframe edges (dark, drawn over the opaque [`bot_faces`]). `LineList` pairs.
pub fn bot_box(origin: Vec3) -> Vec<LineVertex> {
    let v = bot_corners(origin);
    const EDGES: [(usize, usize); 12] = [
        (0, 1),
        (1, 3),
        (3, 2),
        (2, 0), // bottom
        (4, 5),
        (5, 7),
        (7, 6),
        (6, 4), // top
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7), // verticals
    ];
    let mut out = Vec::with_capacity(24);
    for (i, j) in EDGES {
        for p in [v[i], v[j]] {
            out.push(LineVertex {
                pos: p.to_array(),
                color: BOT_EDGE,
            });
        }
    }
    out
}

/// A wireframe outline of every navmesh cell — the 32u tile border under each cell, just over the
/// filled walkable surface — so the individual cells read as a grid on top of the flat fill. `LineList`
/// pairs, dim grey-green.
pub fn nav_cell_wire(graph: &NavGraph) -> Vec<LineVertex> {
    const WIRE: [f32; 3] = [0.10, 0.30, 0.12];
    let hs = GRID * 0.5;
    let mut out = Vec::with_capacity(graph.cells.len() * 8);
    for cell in &graph.cells {
        let o = cell.origin;
        let z = o.z - FEET_DROP + 2.0; // just above the filled surface (which sits at +1)
        let corner = |dx: f32, dy: f32| Vec3::new(o.x + dx * hs, o.y + dy * hs, z);
        let ring = [
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ];
        for k in 0..4 {
            out.push(LineVertex {
                pos: ring[k].to_array(),
                color: WIRE,
            });
            out.push(LineVertex {
                pos: ring[(k + 1) % 4].to_array(),
                color: WIRE,
            });
        }
    }
    out
}

/// A distinct-ish color per LOD cluster id (a hash to RGB, floored so every cluster stays visible),
/// so adjacent clusters read as different tiles in the [`nav_clusters`] overlay.
fn cluster_color(id: u32) -> [f32; 3] {
    let mut x = id.wrapping_mul(0x9e37_79b1);
    x ^= x >> 15;
    x = x.wrapping_mul(0x85eb_ca6b);
    x ^= x >> 13;
    let chan = |shift: u32| 0.35 + 0.6 * (((x >> shift) & 0xff) as f32 / 255.0);
    [chan(0), chan(8), chan(16)]
}

/// The LOD-overlay surface: the same walkable tiles as [`nav_surface`], but each cell tinted by its
/// coarse cluster ([`NavGraph::cluster_of`]) so the hierarchy's block/connectivity partition is
/// visible. Falls back to a flat grey where the LOD layer isn't built.
pub fn nav_clusters(graph: &NavGraph, bsp: &Bsp) -> Vec<LineVertex> {
    let full = GRID * 0.5;
    let sub = GRID / SURF_SUB as f32;
    let hs = sub * 0.5;
    let mut out: Vec<LineVertex> = Vec::with_capacity(graph.cells.len() * 6);
    for (i, cell) in graph.cells.iter().enumerate() {
        let color = graph.cluster_of(i as u32).map_or([0.3, 0.3, 0.3], cluster_color);
        let o = cell.origin;
        let z = o.z - FEET_DROP + 1.0;
        for iy in 0..SURF_SUB {
            for ix in 0..SURF_SUB {
                let cx = o.x - full + hs + ix as f32 * sub;
                let cy = o.y - full + hs + iy as f32 * sub;
                let supported = bsp.is_solid(Vec3::new(cx, cy, o.z - (STEP_HEIGHT + 4.0)))
                    && !bsp.is_solid(Vec3::new(cx, cy, o.z + 1.0));
                if !supported {
                    continue;
                }
                let corner = |dx: f32, dy: f32| LineVertex {
                    pos: [cx + dx * hs, cy + dy * hs, z],
                    color,
                };
                let (a, b, c, d) = (
                    corner(-1.0, -1.0),
                    corner(1.0, -1.0),
                    corner(1.0, 1.0),
                    corner(-1.0, 1.0),
                );
                out.extend_from_slice(&[a, b, c, a, c, d]);
            }
        }
    }
    out
}

/// Emit a polyline as `LineList` pairs, shading each vertex from `DIR_DIM`·color at the start to full
/// color at the end (by fraction of arc length) so the line reads as a directional flow.
fn push_gradient(out: &mut Vec<LineVertex>, path: &[Vec3], color: [f32; 3]) {
    if path.len() < 2 {
        return;
    }
    let total = path.windows(2).map(|w| w[0].distance(w[1])).sum::<f32>().max(1e-3);
    let shade = |frac: f32| {
        let s = DIR_DIM + (1.0 - DIR_DIM) * frac;
        [color[0] * s, color[1] * s, color[2] * s]
    };
    let mut acc = 0.0;
    for w in path.windows(2) {
        let c0 = shade(acc / total);
        acc += w[0].distance(w[1]);
        let c1 = shade(acc / total);
        out.push(LineVertex {
            pos: w[0].to_array(),
            color: c0,
        });
        out.push(LineVertex {
            pos: w[1].to_array(),
            color: c1,
        });
    }
}

/// Sample the shared jump parabola (`rtx_nav::navmesh::arc_point`) — the exact curve the build
/// cleared — into `ARC_SEGMENTS + 1` points.
fn arc_pts(a: Vec3, b: Vec3, apex: f32) -> Vec<Vec3> {
    (0..=ARC_SEGMENTS)
        .map(|i| arc_point(a, b, apex, i as f32 / ARC_SEGMENTS as f32))
        .collect()
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
            "{}: {} solid tris, {} water tris, bounds {:?}..{:?}",
            path,
            mesh.vertices.len() / 3,
            mesh.water.len() / 3,
            mesh.mins,
            mesh.maxs
        );
    }

    /// Build a real navmesh and check both overlay builders: the surface is exactly six vertices
    /// (two triangles) per cell, and the lines are non-empty and exclude `Walk` (shown as surface).
    /// Skipped when `NAVVIEW_TEST_BSP` is unset.
    #[test]
    fn builds_nav_overlay() {
        use rtx_nav::bsp::Bsp;
        use rtx_nav::navmesh::{build_navmesh, RocketJumpParams, SpeedJumpParams};
        let Ok(path) = std::env::var("NAVVIEW_TEST_BSP") else {
            eprintln!("NAVVIEW_TEST_BSP unset — skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let graph = build_navmesh(
            &bsp,
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            true,
            Some(SpeedJumpParams {
                gravity: 800.0,
                accel: 10.0,
                maxspeed: 320.0,
                friction: 4.0,
                stopspeed: 100.0,
                curl: true,
            }),
            Some(RocketJumpParams {
                gravity: 800.0,
                rj_extra: 0.0,
            }),
        );

        // No jump-type link may take off from a submerged cell — you can't jump underwater.
        let jump_kinds = [
            LinkKind::JumpGap,
            LinkKind::DoubleJump,
            LinkKind::SpeedJump,
            LinkKind::RocketJump,
        ];
        let submerged = graph.cells.iter().filter(|c| bsp.is_liquid_at(c.origin)).count();
        for link in &graph.links {
            if jump_kinds.contains(&link.kind) {
                assert!(
                    !bsp.is_liquid_at(graph.cell_origin(link.from)),
                    "{:?} link takes off from a submerged cell",
                    link.kind
                );
            }
        }
        eprintln!("{path}: {submerged}/{} cells submerged", graph.cells.len());

        // No down-link (drop / down-jump) may land where the hull can't descend — a floor slot too
        // small for the hull. Trace the hull straight down the column above each landing.
        for link in &graph.links {
            let a = graph.cell_origin(link.from);
            let b = graph.cell_origin(link.to);
            let dz = b.z - a.z;
            let down = matches!(link.kind, LinkKind::Drop | LinkKind::JumpGap | LinkKind::DoubleJump);
            if down && dz < -18.0 {
                let tr = bsp.hull1_trace(glam::Vec3::new(b.x, b.y, a.z), b);
                assert!(
                    !tr.start_solid && tr.fraction > 0.99,
                    "{:?} {a:?}->{b:?} lands where the hull can't descend (frac {:.2})",
                    link.kind,
                    tr.fraction
                );
            }
        }

        let surface = nav_surface(&graph, &bsp);
        // Each cell emits 0..=SURF_SUB² supported sub-quads, 6 verts (2 triangles) each.
        assert!(
            surface.len().is_multiple_of(6),
            "surface verts are 2-triangle sub-quads"
        );
        assert!(
            !surface.is_empty(),
            "a real map has standable footprint under its cells"
        );
        let max = graph.cells.len() * (SURF_SUB * SURF_SUB) as usize * 6;
        assert!(
            surface.len() <= max,
            "surface can't exceed a full SURF_SUB² tiling per cell"
        );
        assert!(
            surface.iter().all(|v| v.color == link_color(LinkKind::Walk)),
            "surface tiles are Walk-green"
        );

        let all_visible = [true; NUM_LINK_KINDS];
        let lines = nav_lines(&graph, &all_visible);
        assert!(lines.len().is_multiple_of(2), "lines are LineList pairs");
        assert!(
            !lines.is_empty(),
            "a real map should have non-Walk links (steps/jumps/drops)"
        );
        assert!(
            !lines.iter().any(|v| v.color == link_color(LinkKind::Walk)),
            "Walk links must be excluded from the line overlay (they are the surface)"
        );

        // Hiding a kind removes exactly its lines; hiding all leaves nothing.
        let none_visible = [false; NUM_LINK_KINDS];
        assert!(
            nav_lines(&graph, &none_visible).is_empty(),
            "no kinds visible → no lines"
        );
        eprintln!(
            "{}: {} cells → {} surface verts, {} line verts",
            path,
            graph.cells.len(),
            surface.len(),
            lines.len()
        );
    }
}
