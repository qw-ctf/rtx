// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reading a `.pak` — Quake's archive, and for most installs the only place the maps are.
//!
//! A client that can't open one can't find `maps/dm4.bsp` on a stock install at all, because a stock
//! install doesn't have a `maps/` directory: it has `pak0.pak` and `pak1.pak`, and everything id
//! shipped is inside them. The format is from 1996 and shows it, which is the good news — a header
//! and a flat table of fixed-size entries, with no compression to speak of and nothing to negotiate.
//!
//! ```text
//!   header      "PACK", i32 directory offset, i32 directory length
//!   directory   N × 64 bytes: [56-byte NUL-padded name][i32 offset][i32 length]
//! ```
//!
//! # Order is not a detail
//!
//! Which copy of a file you get has to match what the *server* got, or the map checksum we send at
//! `prespawn` is a checksum of a different file and the connection is dropped without a word. So the
//! search order here is the engine's, and it is not the obvious one: `FS_AddPathHandle` prepends each
//! path as it's added and then adds that directory's paks, each prepended in turn — so a gamedir ends
//! up searched **pakN … pak1, pak0, then the loose directory**. Paks win over loose files, and the
//! highest-numbered pak wins over the rest. See [`super::host::NetHost::find`].

use std::collections::HashMap;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// The magic every pak starts with.
const MAGIC: &[u8; 4] = b"PACK";

/// One directory entry: a 56-byte name, then the offset and length as `i32`s.
const ENTRY_SIZE: usize = 64;
const NAME_SIZE: usize = 56;

/// A pak's directory: what's in it, and where each one lives.
///
/// The file itself is not held open — a pak is read a handful of times per map (the BSP, and not much
/// else), so the cost of reopening it is nothing next to the cost of an open handle that has to be
/// kept valid across a level change.
pub(crate) struct Pak {
    path: PathBuf,
    /// Lowercased name → (offset, length). Quake's names are case-insensitive; a map referenced as
    /// `maps/DM4.bsp` is the same file.
    entries: HashMap<String, (u32, u32)>,
}

impl Pak {
    /// Read a pak's directory. `None` if the file isn't one, or is too damaged to trust.
    pub(crate) fn open(path: &Path) -> Option<Pak> {
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
            // A later entry with the same name shadows an earlier one, as the engine's own linear
            // scan from the end would find it.
            entries.insert(name.to_ascii_lowercase(), (offset as u32, length as u32));
        }
        Some(Pak {
            path: path.to_path_buf(),
            entries,
        })
    }

    /// Read one file out of the pak.
    pub(crate) fn read(&self, name: &str) -> Option<Vec<u8>> {
        let &(offset, length) = self.entries.get(&name.to_ascii_lowercase())?;
        let mut f = std::fs::File::open(&self.path).ok()?;
        f.seek(SeekFrom::Start(offset as u64)).ok()?;
        let mut out = vec![0u8; length as usize];
        f.read_exact(&mut out).ok()?;
        Some(out)
    }
}

/// Every pak in a directory, in the order the engine searches them: `pak0`, `pak1`, … exist as a
/// contiguous run and stop at the first gap, and the *last* one found is searched first.
///
/// Returned highest-first, so a caller can simply iterate.
pub(crate) fn paks_in(dir: &Path) -> Vec<Pak> {
    let mut out = Vec::new();
    for i in 0.. {
        let path = dir.join(format!("pak{i}.pak"));
        if !path.exists() {
            break; // the run is contiguous: pak3 without pak2 is not loaded by the engine either
        }
        match Pak::open(&path) {
            Some(p) => out.push(p),
            None => break,
        }
    }
    out.reverse();
    out
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
        let d = std::env::temp_dir().join(format!("rtx-pak-{tag}"));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).expect("mkdir");
        d
    }

    /// The whole of the format, on a pak we built ourselves.
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
        assert_eq!(pak.read("progs/player.mdl").as_deref(), Some(&b"IDPO"[..]));
        assert_eq!(pak.read("maps/dm6.bsp"), None, "not in this one");

        // Quake's names are case-insensitive, and a map is referenced both ways in the wild.
        assert_eq!(pak.read("MAPS/DM4.BSP").as_deref(), Some(&b"the bad place"[..]));
    }

    /// Anything that isn't a pak is not a pak. `find` walks over whatever is in the directory, so
    /// this is asked about ordinary files as a matter of course.
    #[test]
    fn declines_what_it_cannot_read() {
        let d = tmp("bad");
        assert!(Pak::open(&write(&d, "notapak.pak", b"nope")).is_none());
        assert!(Pak::open(&write(&d, "empty.pak", b"")).is_none());
        assert!(Pak::open(&d.join("absent.pak")).is_none());

        // A header that claims a directory past the end of the file.
        let mut truncated = make_pak(&[("a", b"x")]);
        truncated.truncate(14);
        assert!(Pak::open(&write(&d, "short.pak", &truncated)).is_none());
    }

    /// Reading a *real* install, and proving the bytes are the ones a server will agree with.
    ///
    /// `RTX_TEST_BASEDIR` points at a directory holding `id1/`, `qw/`, … — the same opt-in idiom as
    /// the rtx-nav BSP tests, so CI stays green with no copyrighted data checked in. Point it at
    /// `playground` and the whole path runs: find the pak, read the directory, pull `maps/dm4.bsp`
    /// out of it, and checksum it.
    ///
    /// The checksum is what makes this worth writing. It's the number we send at `prespawn`, and a
    /// server that disagrees drops us mid-signon without a word — so "did the pak reader return the
    /// right bytes" and "will this connection work" are the same question. And it has a free oracle:
    /// ezQuake's authors hardcode the id originals' values, computed independently, and one byte
    /// wrong anywhere in the pak reader moves the number.
    #[test]
    fn reads_a_real_install_well_enough_for_a_server_to_accept() {
        let Ok(base) = std::env::var("RTX_TEST_BASEDIR") else {
            eprintln!("RTX_TEST_BASEDIR not set; skipping");
            return;
        };
        let host = crate::netclient::host::NetHost::new(PathBuf::from(base));
        let mut found = 0;
        // The id originals, from `rtx_proto::checksum`'s translate table — which is ezQuake's.
        for (map, want) in [("dm4", 0x9c6f_e4bfu32), ("dm6", 0x5208_da2b)] {
            let name = std::ffi::CString::new(format!("maps/{map}.bsp")).expect("a name");
            let Some(bytes) = crate::host::ClientHost::read_file(&host, &name) else {
                continue; // this install hasn't got it; the point is the ones it has
            };
            let sum = rtx_proto::checksum::map_checksum2(&bytes, map).expect("a bsp");
            assert_eq!(sum as u32, want, "{map}: the bytes a server would refuse");
            found += 1;
        }
        assert!(found > 0, "RTX_TEST_BASEDIR reached no id map at all");
    }

    /// The run of paks is contiguous and searched newest-first — the engine's rule, and the reason a
    /// mission pack's `pak1` overrides the base `pak0` rather than the other way about.
    #[test]
    fn paks_are_searched_highest_first_and_stop_at_a_gap() {
        let d = tmp("order");
        write(&d, "pak0.pak", &make_pak(&[("maps/dm4.bsp", b"original")]));
        write(&d, "pak1.pak", &make_pak(&[("maps/dm4.bsp", b"replaced")]));
        // A gap: pak3 exists but pak2 doesn't, so the engine never reaches it. Neither do we.
        write(&d, "pak3.pak", &make_pak(&[("maps/dm4.bsp", b"never seen")]));

        let paks = paks_in(&d);
        assert_eq!(paks.len(), 2, "the run stops at the gap");
        let found = paks.iter().find_map(|p| p.read("maps/dm4.bsp")).expect("found");
        assert_eq!(found.as_slice(), b"replaced", "the highest-numbered pak wins");

        assert!(paks_in(&d.join("nothing-here")).is_empty());
    }
}
