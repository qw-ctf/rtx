// SPDX-License-Identifier: AGPL-3.0-or-later

//! Minimal BSP reader — only the lumps the navmesh needs from the **player clip hull**.
//!
//! Inspired by the layout of the `bsp` crate at `/Users/daniel/Development/home/bsp`, but
//! pared down to plain little-endian cursor reads over the raw file bytes (no `binrw`, no
//! extra crate deps) and extended with the one lump that crate doesn't expose: `clipnodes`.
//!
//! We read `planes`, `clipnodes`, and the world model (`models[0]`) — the world's hull-1
//! headnode plus its bounding box. Hull 1 is Quake's *standing player* collision hull: its
//! clip planes were already beveled by the player box at BSP-compile time, so a single
//! **point** test against hull 1 answers "would the player box collide here?" — no box trace
//! needed (classic `SV_HullPointContents`). Everything else in the file (rendering BSP tree,
//! faces, lightmaps, textures, vis) is irrelevant to navigation and skipped.

use glam::Vec3;

/// `CONTENTS_SOLID` — the only clip-hull leaf value we test against. Clip hulls (1/2) resolve
/// to either `SOLID` or `CONTENTS_EMPTY` (`-1`); water/lava/sky live in the render hull (0),
/// which this minimal parser doesn't read.
pub const CONTENTS_SOLID: i32 = -2;

/// A BSP plane (`dplane_t`): `normal·p - dist`. `kind` is the axial type — `0/1/2` for an
/// axis-aligned plane (test just that coordinate), `>=3` for a general plane (dot product).
#[derive(Clone, Copy)]
pub struct Plane {
    pub normal: Vec3,
    pub dist: f32,
    pub kind: i32,
}

/// A clip-hull BSP node (`dclipnode_t`). `children[0]` is the front side (`d >= 0`),
/// `children[1]` the back; a negative child is a `CONTENTS_*` leaf rather than a node index.
/// (v29/HL store the children as `i16`; we sign-extend to `i32` so BSP2 fits the same shape.)
#[derive(Clone, Copy)]
pub struct ClipNode {
    pub plane: u32,
    pub children: [i32; 2],
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

// Lump indices into the 15-entry directory (Quake `lump_t` order).
const LUMP_PLANES: usize = 1;
const LUMP_CLIPNODES: usize = 9;
const LUMP_MODELS: usize = 14;

const PLANE_SIZE: usize = 20; // normal(12) + dist(4) + type(4)
const MODEL_SIZE: usize = 64; // mins(12)+maxs(12)+origin(12)+headnode[4](16)+visleafs(4)+face(8)

impl Bsp {
    /// Parse the lumps the navmesh needs from a whole-file byte buffer. Returns `None` if the
    /// file is too short, has an unsupported version, or a lump is malformed.
    pub fn parse(bytes: &[u8]) -> Option<Bsp> {
        let r = Reader::new(bytes);
        let version = r.at(0)?.u32()?;
        // v29 (Quake) and v30 (Half-Life) store clipnode children as i16; BSP2 uses i32.
        let bsp2 = match version {
            29 | 30 => false,
            0x3250_5342 => true, // "BSP2"
            _ => return None,
        };

        let planes = {
            let (off, size) = r.lump(LUMP_PLANES)?;
            let count = size / PLANE_SIZE;
            let mut v = Vec::with_capacity(count);
            let mut p = r.at(off)?;
            for _ in 0..count {
                v.push(Plane {
                    normal: p.vec3()?,
                    dist: p.f32()?,
                    kind: p.i32()?,
                });
            }
            v
        };

        let clipnodes = {
            let (off, size) = r.lump(LUMP_CLIPNODES)?;
            let elem = if bsp2 { 12 } else { 8 };
            let count = size / elem;
            let mut v = Vec::with_capacity(count);
            let mut p = r.at(off)?;
            for _ in 0..count {
                let plane = p.u32()?;
                let children = if bsp2 {
                    [p.i32()?, p.i32()?]
                } else {
                    [p.i16()? as i32, p.i16()? as i32]
                };
                v.push(ClipNode { plane, children });
            }
            v
        };

        // World model is models[0]; we only need its bbox and hull-1 headnode.
        let (moff, msize) = r.lump(LUMP_MODELS)?;
        if msize < MODEL_SIZE {
            return None;
        }
        let mut m = r.at(moff)?;
        let mins = m.vec3()?;
        let maxs = m.vec3()?;
        let _origin = m.vec3()?;
        let _headnode0 = m.i32()?;
        let hull1_headnode = m.i32()?;

        Some(Bsp {
            planes,
            clipnodes,
            hull1_headnode,
            mins,
            maxs,
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

/// A tiny forward-only little-endian byte reader over the file buffer.
struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Reader { bytes, pos: 0 }
    }

    /// A fresh reader positioned at absolute byte offset `pos`.
    fn at(&self, pos: usize) -> Option<Reader<'a>> {
        (pos <= self.bytes.len()).then_some(Reader { bytes: self.bytes, pos })
    }

    /// Resolve lump `index` (0..15) to its `(offset, size)`. The directory is 15 entries of
    /// `{u32 offset, u32 size}` starting right after the 4-byte version.
    fn lump(&self, index: usize) -> Option<(usize, usize)> {
        let mut e = self.at(4 + index * 8)?;
        let off = e.u32()? as usize;
        let size = e.u32()? as usize;
        (off.checked_add(size)? <= self.bytes.len()).then_some((off, size))
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.pos.checked_add(n)?;
        let s = self.bytes.get(self.pos..end)?;
        self.pos = end;
        Some(s)
    }

    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn i32(&mut self) -> Option<i32> {
        Some(i32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn i16(&mut self) -> Option<i16> {
        Some(i16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn vec3(&mut self) -> Option<Vec3> {
        Some(Vec3::new(self.f32()?, self.f32()?, self.f32()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
