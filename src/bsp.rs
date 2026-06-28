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

/// The lump directory, reading only the three lumps the navmesh needs and skipping the rest
/// (`pad_before` jumps over the unused entries: 1 before planes, 7 before clipnodes, 4 before
/// models).
#[derive(BinRead)]
#[br(little)]
struct Header {
    version: Version,
    #[br(pad_before = 8)]
    planes: Lump,
    #[br(pad_before = 56)]
    clipnodes: Lump,
    #[br(pad_before = 32)]
    models: Lump,
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

/// The world model (`models[0]`): we only need its bounding box and the hull-1 headnode, so
/// `pad_before` skips `origin` and `headnode[0]` and the trailing fields aren't read at all.
#[derive(BinRead)]
#[br(little)]
struct Model {
    #[br(map = Vec3::from_array)]
    mins: Vec3,
    #[br(map = Vec3::from_array)]
    maxs: Vec3,
    #[br(pad_before = 16)] // skip origin (12) + headnode[0] (4)
    clip1: i32,
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
}

impl Bsp {
    /// Parse the lumps the navmesh needs from a whole-file byte buffer. Returns `None` on an
    /// unsupported version or a malformed/truncated lump.
    pub fn parse(bytes: &[u8]) -> Option<Bsp> {
        let mut c = Cursor::new(bytes);
        let header: Header = c.read_le().ok()?;

        let planes = read_lump::<Plane>(&mut c, &header.planes).ok()?;
        let clipnodes = if header.version == Version::Bsp2 {
            read_lump::<ClipNode>(&mut c, &header.clipnodes).ok()?
        } else {
            read_lump_into::<ClipNodeV1, ClipNode>(&mut c, &header.clipnodes).ok()?
        };

        c.seek(SeekFrom::Start(header.models.offset as u64)).ok()?;
        let model: Model = c.read_le().ok()?;

        Some(Bsp {
            planes,
            clipnodes,
            hull1_headnode: model.clip1,
            mins: model.mins,
            maxs: model.maxs,
        })
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

#[cfg(test)]
mod tests {
    use super::*;

    // Lump directory indices / element sizes, for the independent cross-check below.
    const LUMP_PLANES: usize = 1;
    const LUMP_CLIPNODES: usize = 9;
    const PLANE_SIZE: usize = 20;

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
