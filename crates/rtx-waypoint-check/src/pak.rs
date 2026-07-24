// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reading a `.pak` and resolving a map's BSP off a stock install.
//!
//! A trimmed copy of `rtx-game`'s `netclient::pak` — that one is `pub(crate)` behind the game's
//! `netclient` feature, so it can't be reached from an independent bin, and a checker isn't worth
//! turning it public for. Same format (header + a flat table of fixed-size entries) and the same
//! newest-pak-wins search order, with two changes the offline tool needs:
//!
//! - **Case-insensitive discovery.** A real id install ships `pak0.pak` and `PAK1.PAK` — one of each
//!   case. The game's reader probes the literal `pak{i}.pak`, which happens to work on Windows but
//!   would miss the uppercase one on a case-sensitive filesystem, so here we scan the directory once
//!   and match names case-folded.
//! - **[`resolve_bsp`]**, which walks the engine's gamedir-before-`id1` order so the tool finds a map
//!   wherever it lives (loose in `qw/maps`, or inside either pak).

use std::collections::HashMap;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

const MAGIC: &[u8; 4] = b"PACK";
const ENTRY_SIZE: usize = 64;
const NAME_SIZE: usize = 56;

/// A pak's directory: lowercased name → (offset, length). The file itself is reopened per read
/// rather than held — a checker reads one BSP out of a pak and is done.
pub struct Pak {
    path: PathBuf,
    entries: HashMap<String, (u32, u32)>,
}

impl Pak {
    /// Read a pak's directory. `None` if the file isn't one, or is too damaged to trust.
    pub fn open(path: &Path) -> Option<Pak> {
        let mut f = std::fs::File::open(path).ok()?;
        let mut head = [0u8; 12];
        f.read_exact(&mut head).ok()?;
        if &head[0..4] != MAGIC {
            return None;
        }
        let dir_offset = i32::from_le_bytes(head[4..8].try_into().ok()?);
        let dir_len = i32::from_le_bytes(head[8..12].try_into().ok()?);
        if dir_offset < 0 || dir_len < 0 || !(dir_len as usize).is_multiple_of(ENTRY_SIZE) {
            return None;
        }

        let mut dir = vec![0u8; dir_len as usize];
        f.seek(SeekFrom::Start(dir_offset as u64)).ok()?;
        f.read_exact(&mut dir).ok()?;

        let mut entries = HashMap::new();
        for e in dir.chunks_exact(ENTRY_SIZE) {
            let name = &e[..NAME_SIZE];
            let name = name.iter().position(|&b| b == 0).map_or(name, |n| &name[..n]);
            let Ok(name) = std::str::from_utf8(name) else { continue };
            let offset = i32::from_le_bytes(e[NAME_SIZE..NAME_SIZE + 4].try_into().ok()?);
            let length = i32::from_le_bytes(e[NAME_SIZE + 4..].try_into().ok()?);
            if offset < 0 || length < 0 {
                continue;
            }
            entries.insert(name.to_ascii_lowercase(), (offset as u32, length as u32));
        }
        Some(Pak {
            path: path.to_path_buf(),
            entries,
        })
    }

    /// Read one file out of the pak. Names are case-insensitive (`maps/DM4.bsp` == `maps/dm4.bsp`).
    pub fn read(&self, name: &str) -> Option<Vec<u8>> {
        let &(offset, length) = self.entries.get(&name.to_ascii_lowercase())?;
        let mut f = std::fs::File::open(&self.path).ok()?;
        f.seek(SeekFrom::Start(offset as u64)).ok()?;
        let mut out = vec![0u8; length as usize];
        f.read_exact(&mut out).ok()?;
        Some(out)
    }
}

/// Every pak in a directory, in the order the engine searches them: `pak0`, `pak1`, … form a
/// contiguous run and stop at the first gap, and the *last* one found is searched first. Discovery
/// is case-insensitive so a mixed-case install (`pak0.pak` + `PAK1.PAK`) loads both.
pub fn paks_in(dir: &Path) -> Vec<Pak> {
    let mut by_name: HashMap<String, PathBuf> = HashMap::new();
    if let Ok(rd) = std::fs::read_dir(dir) {
        for entry in rd.flatten() {
            if let Some(name) = entry.file_name().to_str() {
                by_name.insert(name.to_ascii_lowercase(), entry.path());
            }
        }
    }
    let mut out = Vec::new();
    for i in 0.. {
        let Some(path) = by_name.get(&format!("pak{i}.pak")) else {
            break; // the run is contiguous: pak3 without pak2 is not loaded by the engine either
        };
        match Pak::open(path) {
            Some(p) => out.push(p),
            None => break,
        }
    }
    out.reverse();
    out
}

/// Resolve `maps/<map>.bsp` off an install rooted at `basedir`, following the engine's search
/// order: the `qw` gamedir before `id1`, and within a directory the paks (newest first) before any
/// loose file. Returns the BSP bytes, or `None` if the map is nowhere to be found.
pub fn resolve_bsp(basedir: &Path, map: &str) -> Option<Vec<u8>> {
    let name = format!("maps/{map}.bsp");
    for sub in ["qw", "id1"] {
        let dir = basedir.join(sub);
        for pak in paks_in(&dir) {
            if let Some(bytes) = pak.read(&name) {
                return Some(bytes);
            }
        }
        if let Ok(bytes) = std::fs::read(dir.join("maps").join(format!("{map}.bsp"))) {
            return Some(bytes);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a pak in memory, so the reader is tested against the format rather than against
    /// whatever happens to be installed.
    fn make_pak(files: &[(&str, &[u8])]) -> Vec<u8> {
        let mut body = Vec::new();
        let mut dir = Vec::new();
        for (name, data) in files {
            let offset = 12 + body.len();
            body.extend_from_slice(data);
            let mut entry = [0u8; ENTRY_SIZE];
            entry[..name.len()].copy_from_slice(name.as_bytes());
            entry[NAME_SIZE..NAME_SIZE + 4].copy_from_slice(&(offset as i32).to_le_bytes());
            entry[NAME_SIZE + 4..].copy_from_slice(&(data.len() as i32).to_le_bytes());
            dir.extend_from_slice(&entry);
        }
        let mut out = Vec::from(*MAGIC);
        out.extend_from_slice(&((12 + body.len()) as i32).to_le_bytes());
        out.extend_from_slice(&(dir.len() as i32).to_le_bytes());
        out.extend_from_slice(&body);
        out.extend_from_slice(&dir);
        out
    }

    fn write(dir: &Path, name: &str, bytes: &[u8]) -> PathBuf {
        let p = dir.join(name);
        std::fs::write(&p, bytes).expect("write");
        p
    }

    fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("rtx-wpc-pak-{tag}"));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).expect("mkdir");
        d
    }

    #[test]
    fn reads_a_paks_directory_and_its_files() {
        let d = tmp("read");
        let p = write(
            &d,
            "pak0.pak",
            &make_pak(&[("maps/dm4.bsp", b"the bad place"), ("progs/player.mdl", b"IDPO")]),
        );
        let pak = Pak::open(&p).expect("a pak");
        assert_eq!(pak.read("maps/dm4.bsp").as_deref(), Some(&b"the bad place"[..]));
        assert_eq!(pak.read("MAPS/DM4.BSP").as_deref(), Some(&b"the bad place"[..]));
        assert_eq!(pak.read("maps/dm6.bsp"), None);
    }

    #[test]
    fn declines_what_it_cannot_read() {
        let d = tmp("bad");
        assert!(Pak::open(&write(&d, "notapak.pak", b"nope")).is_none());
        assert!(Pak::open(&write(&d, "empty.pak", b"")).is_none());
        let mut truncated = make_pak(&[("a", b"x")]);
        truncated.truncate(14);
        assert!(Pak::open(&write(&d, "short.pak", &truncated)).is_none());
    }

    #[test]
    fn paks_are_searched_highest_first_and_stop_at_a_gap() {
        let d = tmp("order");
        write(&d, "pak0.pak", &make_pak(&[("maps/dm4.bsp", b"original")]));
        write(&d, "pak1.pak", &make_pak(&[("maps/dm4.bsp", b"replaced")]));
        write(&d, "pak3.pak", &make_pak(&[("maps/dm4.bsp", b"never seen")]));
        let paks = paks_in(&d);
        assert_eq!(paks.len(), 2, "the run stops at the gap");
        let found = paks.iter().find_map(|p| p.read("maps/dm4.bsp")).expect("found");
        assert_eq!(found.as_slice(), b"replaced", "the highest-numbered pak wins");
        assert!(paks_in(&d.join("nothing-here")).is_empty());
    }

    /// A stock id install has `pak0.pak` and `PAK1.PAK` — one of each case. Both must load, and
    /// `resolve_bsp` must pull a map out of the uppercase one.
    #[test]
    fn discovers_mixed_case_paks_and_resolves() {
        let base = tmp("mixed");
        let id1 = base.join("id1");
        std::fs::create_dir_all(id1.join("maps")).expect("mkdir id1");
        std::fs::create_dir_all(base.join("qw").join("maps")).expect("mkdir qw");
        write(&id1, "pak0.pak", &make_pak(&[("maps/e1m2.bsp", b"e1m2-bytes")]));
        write(&id1, "PAK1.PAK", &make_pak(&[("maps/dm3.bsp", b"dm3-bytes")]));
        assert_eq!(paks_in(&id1).len(), 2, "both cases discovered");
        assert_eq!(resolve_bsp(&base, "dm3").as_deref(), Some(&b"dm3-bytes"[..]));
        assert_eq!(resolve_bsp(&base, "e1m2").as_deref(), Some(&b"e1m2-bytes"[..]));
        // Loose in the gamedir, and preferred over id1.
        write(&base.join("qw").join("maps"), "bravado.bsp", b"loose-bravado");
        assert_eq!(resolve_bsp(&base, "bravado").as_deref(), Some(&b"loose-bravado"[..]));
        assert_eq!(resolve_bsp(&base, "nomap"), None);
    }
}
