// SPDX-License-Identifier: AGPL-3.0-or-later

//! `qwd-dump` — dump position, velocity, and usercmd fields from QuakeWorld `.qwd` demos as CSV.
//!
//! A Rust port of the repo's old `qwd_dump.py`, on top of [`rtx_qwd_parse`]. Two modes:
//! *combined* (default) emits one row per `svc_playerinfo`, pairing it with the matching usercmd —
//! the frame's own if the server echoed one, else the nearest `dem_cmd` for the local player;
//! *raw* (`--raw`) emits the `dem_cmd` and `svc_playerinfo` events interleaved by time, plus a
//! leading `movevars` row.

use std::borrow::Cow;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use rtx_proto::svc::{self, MoveVars, Usercmd};
use rtx_qwd_parse::{parse_demo, Demo, DemoCmd, Frame};

/// The combined-mode columns (also the body of every raw row).
const HEADER: &[&str] = &[
    "file",
    "time",
    "player",
    "x",
    "y",
    "z",
    "velocity_present",
    "vx",
    "vy",
    "vz",
    "cmd_source",
    "cmd_time",
    "msec",
    "forwardmove",
    "sidemove",
    "upmove",
    "buttons",
    "pitch",
    "yaw",
];

/// The ten movevars, named to match the old tool's `mv_*` raw-CSV tail so both stay diffable.
const MOVEVAR_FIELDS: &[&str] = &[
    "mv_gravity",
    "mv_stopspeed",
    "mv_maxspeed",
    "mv_spectatormaxspeed",
    "mv_accelerate",
    "mv_airaccelerate",
    "mv_wateraccelerate",
    "mv_friction",
    "mv_waterfriction",
    "mv_entgravity",
];

/// Which stream a resolved usercmd came from — the `cmd_source` column.
#[derive(Clone, Copy)]
enum CmdSource {
    /// The usercmd the server echoed inside the `svc_playerinfo`.
    PlayerInfo,
    /// The local player's nearest `dem_cmd`, matched by time.
    DemCmd,
}

impl CmdSource {
    fn as_str(self) -> &'static str {
        match self {
            CmdSource::PlayerInfo => "playerinfo",
            CmdSource::DemCmd => "dem_cmd",
        }
    }
}

/// A usercmd resolved for a row, carrying which stream it came from and the time to report.
struct ResolvedCmd {
    source: CmdSource,
    time: f32,
    cmd: Usercmd,
}

fn main() -> ExitCode {
    let args = match Args::parse(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(msg) => {
            eprintln!("qwd-dump: {msg}");
            return ExitCode::from(2);
        }
    };
    if args.help {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }

    let paths = match args.files.is_empty() {
        true => default_paths(),
        false => args.files.clone(),
    };
    if paths.is_empty() {
        eprintln!("qwd-dump: no .qwd files found");
        return ExitCode::FAILURE;
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut had_error = false;

    if !args.no_header {
        let header: Vec<&str> = if args.raw {
            std::iter::once("event")
                .chain(HEADER.iter().copied())
                .chain(MOVEVAR_FIELDS.iter().copied())
                .collect()
        } else {
            HEADER.to_vec()
        };
        if write_row(&mut out, &header).is_err() {
            return ExitCode::SUCCESS; // downstream closed the pipe
        }
    }

    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(demo) => demo,
            Err(e) => {
                eprintln!("qwd-dump: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        for w in &demo.warnings {
            eprintln!("qwd-dump: {}: {w}", path.display());
            had_error = true;
        }

        let rows = if args.raw {
            raw_rows(&demo, args.player, args.all_players)
        } else {
            combined_rows(&demo, args.player, args.all_players)
        };
        for row in rows {
            let cells: Vec<&str> = row.iter().map(String::as_str).collect();
            if write_row(&mut out, &cells).is_err() {
                return if had_error {
                    ExitCode::FAILURE
                } else {
                    ExitCode::SUCCESS
                };
            }
        }
    }

    let _ = out.flush();
    if had_error {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

/// Which player's frames to emit: an explicit slot, the local player, or all of them (`None`).
fn selected_player(demo: &Demo, explicit: Option<u8>, all: bool) -> Option<u8> {
    if all {
        None
    } else if let Some(p) = explicit {
        Some(p)
    } else {
        Some(demo.local_player.unwrap_or(0))
    }
}

/// The nearest `dem_cmd` to `time`, by absolute time difference — the local player's input for a
/// frame the server didn't echo a command for.
fn nearest_demo_cmd(cmds: &[DemoCmd], time: f32) -> Option<&DemoCmd> {
    if cmds.is_empty() {
        return None;
    }
    let idx = cmds.partition_point(|c| c.time < time);
    let mut best: Option<&DemoCmd> = None;
    for &i in &[idx, idx.wrapping_sub(1)] {
        if let Some(c) = cmds.get(i) {
            if best.is_none_or(|b| (c.time - time).abs() < (b.time - time).abs()) {
                best = Some(c);
            }
        }
    }
    best
}

/// The usercmd to report for one frame: the echoed command if present, else the nearest `dem_cmd`
/// — but only for the local player (or every player when the demo never told us who that is).
fn command_for_frame(demo: &Demo, frame: &Frame) -> Option<ResolvedCmd> {
    if let Some(cmd) = frame.info.command {
        return Some(ResolvedCmd {
            source: CmdSource::PlayerInfo,
            time: frame.time,
            cmd,
        });
    }
    let use_demo = match demo.local_player {
        None => true,
        Some(local) => frame.info.player == local,
    };
    if use_demo {
        nearest_demo_cmd(&demo.demo_cmds, frame.time).map(|c| ResolvedCmd {
            source: CmdSource::DemCmd,
            time: c.time,
            cmd: c.cmd,
        })
    } else {
        None
    }
}

fn combined_rows(demo: &Demo, explicit: Option<u8>, all: bool) -> Vec<Vec<String>> {
    let player = selected_player(demo, explicit, all);
    demo.frames
        .iter()
        .filter(|f| player.is_none_or(|p| f.info.player == p))
        .map(|f| row_for_frame(demo, f, command_for_frame(demo, f).as_ref()))
        .collect()
}

fn raw_rows(demo: &Demo, explicit: Option<u8>, all: bool) -> Vec<Vec<String>> {
    let player = selected_player(demo, explicit, all);
    let pad = vec![String::new(); MOVEVAR_FIELDS.len()];
    let mut rows: Vec<Vec<String>> = Vec::new();

    // Movevars first, so a reader has the physics params before any frame.
    if let Some(mv) = demo.movevars {
        let mut row = vec!["movevars".to_string()];
        row.extend(std::iter::repeat_n(String::new(), HEADER.len()));
        row.extend(movevar_cells(&mv));
        rows.push(row);
    }

    // dem_cmd (key i*2+1) and playerinfo (key j*2) events, ordered by (time, key) — the key breaks
    // ties deterministically and interleaves the two independent index spaces exactly as before.
    let mut events: Vec<(f32, usize, Vec<String>)> = Vec::new();
    for (i, c) in demo.demo_cmds.iter().enumerate() {
        let mut row = vec!["dem_cmd".to_string()];
        row.extend(row_for_cmd(&demo.path, c));
        row.extend(pad.iter().cloned());
        events.push((c.time, i * 2 + 1, row));
    }
    for (j, f) in demo.frames.iter().enumerate() {
        if player.is_some_and(|p| f.info.player != p) {
            continue;
        }
        let resolved = f.info.command.map(|cmd| ResolvedCmd {
            source: CmdSource::PlayerInfo,
            time: f.time,
            cmd,
        });
        let mut row = vec!["playerinfo".to_string()];
        row.extend(row_for_frame(demo, f, resolved.as_ref()));
        row.extend(pad.iter().cloned());
        events.push((f.time, j * 2, row));
    }
    events.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.1.cmp(&b.1))
    });
    rows.extend(events.into_iter().map(|(_, _, row)| row));
    rows
}

/// A combined/playerinfo row: the frame's position and velocity, plus a resolved usercmd's fields.
fn row_for_frame(demo: &Demo, frame: &Frame, cmd: Option<&ResolvedCmd>) -> Vec<String> {
    let info = &frame.info;
    let velocity_present = (0..3).any(|i| info.flags & (svc::pf::VELOCITY1 << i) != 0);
    vec![
        basename(&demo.path),
        fmt_float(frame.time),
        info.player.to_string(),
        fmt_float(info.origin.x),
        fmt_float(info.origin.y),
        fmt_float(info.origin.z),
        if velocity_present { "1" } else { "0" }.to_string(),
        opt_int(velocity_present.then_some(info.velocity.x as i64)),
        opt_int(velocity_present.then_some(info.velocity.y as i64)),
        opt_int(velocity_present.then_some(info.velocity.z as i64)),
        cmd.map_or(String::new(), |c| c.source.as_str().to_string()),
        cmd.map_or(String::new(), |c| fmt_float(c.time)),
        cmd.map_or(String::new(), |c| c.cmd.msec.to_string()),
        cmd.map_or(String::new(), |c| c.cmd.forward.to_string()),
        cmd.map_or(String::new(), |c| c.cmd.side.to_string()),
        cmd.map_or(String::new(), |c| c.cmd.up.to_string()),
        cmd.map_or(String::new(), |c| c.cmd.buttons.to_string()),
        cmd.map_or(String::new(), |c| fmt_float(c.cmd.angles.x)),
        cmd.map_or(String::new(), |c| fmt_float(c.cmd.angles.y)),
    ]
}

/// A raw `dem_cmd` row: no position/velocity, just the local input.
fn row_for_cmd(path: &Path, c: &DemoCmd) -> Vec<String> {
    vec![
        basename(path),
        fmt_float(c.time),
        String::new(),
        String::new(),
        String::new(),
        String::new(),
        "0".to_string(),
        String::new(),
        String::new(),
        String::new(),
        CmdSource::DemCmd.as_str().to_string(),
        fmt_float(c.time),
        c.cmd.msec.to_string(),
        c.cmd.forward.to_string(),
        c.cmd.side.to_string(),
        c.cmd.up.to_string(),
        c.cmd.buttons.to_string(),
        fmt_float(c.cmd.angles.x),
        fmt_float(c.cmd.angles.y),
    ]
}

fn movevar_cells(mv: &MoveVars) -> Vec<String> {
    [
        mv.gravity,
        mv.stopspeed,
        mv.maxspeed,
        mv.spectatormaxspeed,
        mv.accelerate,
        mv.airaccelerate,
        mv.wateraccelerate,
        mv.friction,
        mv.waterfriction,
        mv.entgravity,
    ]
    .iter()
    .map(|v| fmt_float(*v))
    .collect()
}

fn basename(path: &Path) -> String {
    path.file_name()
        .map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned())
}

/// Format a float to at most six decimals, trailing zeros (and a bare point) trimmed — matching the
/// old tool's `fmt_float`, so the CSVs stay comparable.
fn fmt_float(v: f32) -> String {
    let s = format!("{v:.6}");
    let trimmed = s.trim_end_matches('0').trim_end_matches('.');
    if trimmed.is_empty() {
        "0".to_string()
    } else {
        trimmed.to_string()
    }
}

fn opt_int(v: Option<i64>) -> String {
    v.map_or(String::new(), |n| n.to_string())
}

/// Write one CSV row (`,`-separated, `\n`-terminated, minimal quoting).
fn write_row(out: &mut impl Write, cells: &[&str]) -> io::Result<()> {
    for (i, cell) in cells.iter().enumerate() {
        if i > 0 {
            out.write_all(b",")?;
        }
        out.write_all(csv_escape(cell).as_bytes())?;
    }
    out.write_all(b"\n")
}

/// Quote a field only if it contains a comma, quote, or newline (RFC 4180 minimal quoting).
fn csv_escape(s: &str) -> Cow<'_, str> {
    if s.contains([',', '"', '\n', '\r']) {
        Cow::Owned(format!("\"{}\"", s.replace('"', "\"\"")))
    } else {
        Cow::Borrowed(s)
    }
}

/// `*.qwd` in the current directory, sorted — the default when no files are named.
fn default_paths() -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(".")
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("qwd")))
        .collect();
    paths.sort();
    paths
}

const USAGE: &str = "\
usage: qwd-dump [--raw] [--player N | --all-players] [--no-header] [FILE...]

Dump position, velocity, and usercmd fields from QuakeWorld .qwd demos as CSV.
With no FILE, reads *.qwd in the current directory.

  --raw           emit raw dem_cmd and playerinfo events (plus a movevars row) instead
                  of combined rows
  --player N      restrict output to one player slot; defaults to the local player
  --all-players   include every svc_playerinfo player
  --no-header     omit the CSV header
  -h, --help      show this help
";

/// Parsed command line.
struct Args {
    files: Vec<PathBuf>,
    raw: bool,
    player: Option<u8>,
    all_players: bool,
    no_header: bool,
    help: bool,
}

impl Args {
    fn parse(argv: impl Iterator<Item = String>) -> std::result::Result<Args, String> {
        let mut args = Args {
            files: Vec::new(),
            raw: false,
            player: None,
            all_players: false,
            no_header: false,
            help: false,
        };
        let mut it = argv.peekable();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--raw" => args.raw = true,
                "--all-players" => args.all_players = true,
                "--no-header" => args.no_header = true,
                "-h" | "--help" => args.help = true,
                "--player" => {
                    let v = it.next().ok_or("--player needs a value")?;
                    args.player = Some(parse_player(&v)?);
                }
                _ if arg.starts_with("--player=") => {
                    args.player = Some(parse_player(&arg["--player=".len()..])?);
                }
                "--" => args.files.extend(it.by_ref().map(PathBuf::from)),
                _ if arg.starts_with('-') && arg != "-" => return Err(format!("unknown option {arg}")),
                _ => args.files.push(PathBuf::from(arg)),
            }
        }
        Ok(args)
    }
}

fn parse_player(s: &str) -> std::result::Result<u8, String> {
    s.parse::<u8>().map_err(|_| format!("invalid --player value {s:?}"))
}
