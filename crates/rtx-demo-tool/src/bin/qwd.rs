// SPDX-License-Identifier: AGPL-3.0-or-later

//! `qwd` — inspect QuakeWorld demos, single-client `.qwd` and multi-view `.mvd` alike (the
//! container is chosen by content, since neither has a magic number and demos get renamed).
//!
//! - `qwd players` lists a demo's roster. The first thing to run on a file you haven't seen: a
//!   multi-view recording holds a whole team game across thirty-two slots, and the other
//!   subcommands need a slot or a name to talk about.
//! - `qwd dump` writes position, velocity, and usercmd fields as CSV (the old `qwd_dump.py`
//!   output): *combined* rows (one per player-state update, paired with its usercmd) by default,
//!   or `--raw` events interleaved by time with a leading `movevars` row.
//! - `qwd analyze` prints a per-player movement report — duration, speed percentiles, climb, path
//!   length, the jumps it found, and an optional down-sampled waypoint table.
//!
//! The name is historical: it predates MVD support and is kept because scripts call it.

use std::borrow::Cow;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use glam::Vec3;
use rtx_demo_tool::analysis::{self, Motion};
use rtx_demo_tool::{parse_demo, Demo, DemoCmd, Frame};
use rtx_proto::svc::{MoveVars, Usercmd};

const USAGE: &str = "\
usage: qwd <command> [options] [FILE...]

Inspect QuakeWorld demos: single-client .qwd and server-side multi-view .mvd, told
apart by content. With no FILE, reads *.qwd and *.mvd in the current directory.

A .qwd records one client — its own inputs and the velocities the server sent it.
An .mvd records the whole game: every player, no velocities (speeds are derived
from successive positions), and inputs only where the recorder embedded them.

commands:
  players   list the players in a demo
  segments  cut all human movement into distinct segments (the coverage suite)
  route     find every human run between two points, and how long it took
  dump      write position/velocity/usercmd fields as CSV
  analyze   print a per-player movement report
  teamplay  print a per-team report: score, deaths, room control, idle time

dump options:
  --raw           emit raw dem_cmd and playerinfo events (plus a movevars row)
  --player P      one player, by slot number or (substring of) name; defaults to
                  the recording client, or to everyone in an .mvd
  --all-players   include every player
  --no-header     omit the CSV header

segments options:
  --min-path N    ignore segments shorter than N units of path (default 192)
  --max-path N    cut a segment every N units travelled (default 640). Players
                  in a real match almost never stop, so this — not the stop
                  rule — is what makes segments comparable and dedupable.
  --max-secs N    backstop for slow movement: split after N seconds (default 4)
  --bucket N      two segments are the same movement when both endpoints fall
                  in the same N-unit cell (default 128)
  --limit N       how many to print, longest first (default 20)

route options (usage: qwd route FROM_X FROM_Y FROM_Z TO_X TO_Y TO_Z [FILE...]):
  --radius N      how close counts as reaching a point (default 64)
  --max-secs N    longest run to consider one traversal (default 20)
  --max-detour N  reject runs whose path exceeds N x the straight-line distance
                  (default 2). Without it, being at A and later at B counts as
                  a traversal even when the player never went near the route.

analyze options:
  --player P      one player, by slot number or (substring of) name
  --all-players   report every player
  --waypoints N   also print N evenly-spaced trajectory waypoints (default: none)

teamplay options (server-recorded .mvd; reports per team, for A/B reading):
  --zones dm3     use the dm3 control rooms the stack measurement priced
  --zone NAME,X,Y,Z[,RADIUS[,HALFHEIGHT]]
                  add one zone (repeatable); radius defaults to 320, half-height
                  to 96, which is room-sized rather than pickup-sized
  --players       also print the per-player table
  --split N       also report frags per N equal time slices. Read these BEFORE
                  the total: a split-team effect has been seen to reverse
                  mid-match, and one window is not the result.

  -h, --help      show this help
";

fn main() -> ExitCode {
    let mut argv = std::env::args().skip(1);
    match argv.next().as_deref() {
        Some("players") => run_players(argv),
        Some("segments") => run_segments(argv),
        Some("route") => run_route(argv),
        Some("dump") => run_dump(argv),
        Some("analyze") => run_analyze(argv),
        Some("teamplay") => run_teamplay(argv),
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("qwd: unknown command {other:?} (try `qwd --help`)");
            ExitCode::from(2)
        }
    }
}

/// Resolve the file list: the given paths, or `*.qwd` in the current directory.
fn resolve_paths(files: Vec<PathBuf>) -> Vec<PathBuf> {
    if !files.is_empty() {
        return files;
    }
    let mut paths: Vec<PathBuf> = std::fs::read_dir(".")
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.extension()
                .is_some_and(|x| x.eq_ignore_ascii_case("qwd") || x.eq_ignore_ascii_case("mvd"))
        })
        .collect();
    paths.sort();
    paths
}

// ── segments ──────────────────────────────────────────────────────────────────────────────────

/// Cut every player's movement into segments, collapse the duplicates, and print what is left.
///
/// This is the coverage suite: not a curated list of routes, but every distinct piece of movement
/// the map and the game actually produced. No route is privileged — the bot should be able to
/// execute all of them, and how it executes each is what the score is about.
fn run_segments(argv: impl Iterator<Item = String>) -> ExitCode {
    let mut files = Vec::new();
    let (mut min_path, mut max_path, mut bucket, mut limit) = (192.0f32, 640.0f32, 128.0f32, 20usize);
    let mut max_secs = 4.0f32;
    let mut it = argv;
    while let Some(arg) = it.next() {
        let num = |v: Option<String>, what: &str| v.and_then(|v| v.parse::<f32>().ok()).ok_or(what.to_string());
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            "--min-path" => match num(it.next(), "--min-path needs a number") {
                Ok(v) => min_path = v,
                Err(e) => return usage_err(&e),
            },
            "--max-path" => match num(it.next(), "--max-path needs a number") {
                Ok(v) => max_path = v,
                Err(e) => return usage_err(&e),
            },
            "--max-secs" => match num(it.next(), "--max-secs needs a number") {
                Ok(v) => max_secs = v,
                Err(e) => return usage_err(&e),
            },
            "--bucket" => match num(it.next(), "--bucket needs a number") {
                Ok(v) => bucket = v,
                Err(e) => return usage_err(&e),
            },
            "--limit" => match num(it.next(), "--limit needs a number") {
                Ok(v) => limit = v as usize,
                Err(e) => return usage_err(&e),
            },
            _ if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
            _ => files.push(PathBuf::from(arg)),
        }
    }
    let paths = resolve_paths(files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }
    let mut had_error = false;
    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        let raw: usize = analysis::players(&demo)
            .into_iter()
            .map(|p| analysis::track(&demo, p).segments(min_path, max_path, max_secs).len())
            .sum();
        let suite = rtx_demo_tool::line::suite(&demo, min_path, max_path, max_secs, bucket);
        println!(
            "{}: {} movement segments -> {} distinct (bucket {:.0}u)",
            basename(&demo.path),
            raw,
            suite.len(),
            bucket
        );
        let total: f32 = suite.iter().map(|s| s.path_length()).sum();
        let median = |mut v: Vec<f32>| {
            v.sort_by(f32::total_cmp);
            v.get(v.len() / 2).copied().unwrap_or(0.0)
        };
        // What good movement reads as on the steering metrics. Scoring each human run against its own
        // line gives exactly that, and it is the only meaningful target: peak turn rate and wall
        // contacts have no good absolute value, and a strafe jump is a hard continuous turn.
        let self_scored: Vec<rtx_demo_tool::line::LineScore> = suite
            .iter()
            .map(|s| rtx_demo_tool::line::LineScore::score(&s.samples(), &s.reference("self")))
            .collect();
        let pctl = |mut v: Vec<f32>, q: f32| {
            v.sort_by(f32::total_cmp);
            v.get((((v.len().max(1) - 1) as f32) * q) as usize)
                .copied()
                .unwrap_or(0.0)
        };
        // Split by pace, because the two halves are different activities and averaging them says
        // nothing. A player crossing the map holds a line; one fighting in a room is dodging, and
        // *should* be reversing constantly. Only the travelling half is a target for a bot's
        // route-following, and mixing them hides a factor of five.
        for (label, lo, hi) in [
            ("travelling (>=500 ups)", 500.0, f32::MAX),
            ("manoeuvring (<300 ups)", 0.0, 300.0),
        ] {
            let sel: Vec<&rtx_demo_tool::line::LineScore> = suite
                .iter()
                .zip(&self_scored)
                .filter(|(s, _)| s.mean_speed() >= lo && s.mean_speed() < hi)
                .map(|(_, sc)| sc)
                .collect();
            if sel.is_empty() {
                continue;
            }
            println!(
                "  human {label}: {} segs | turn rate p50 {:.0} deg/s | turn flips p50 {:.1} p90 {:.1} /s | wall p50 {:.0}",
                sel.len(),
                pctl(sel.iter().map(|s| s.yaw_jitter_p95).collect(), 0.5),
                pctl(sel.iter().map(|s| s.yaw_reversals).collect(), 0.5),
                pctl(sel.iter().map(|s| s.yaw_reversals).collect(), 0.9),
                pctl(sel.iter().map(|s| s.wall_events as f32).collect(), 0.5),
            );
        }
        println!(
            "  covering {:.0}u of distinct movement; median {:.2}s / {:.0}u / {:.0} ups per segment",
            total,
            median(suite.iter().map(|s| s.duration()).collect()),
            median(suite.iter().map(|s| s.path_length()).collect()),
            median(suite.iter().map(|s| s.mean_speed()).collect()),
        );
        // The suite is already fastest first, and that order *is* the segment's name: a live run
        // scores "segment 7 of this demo", so the index printed here has to be the index the
        // harness will index by. Path length is near-constant by construction (segments are cut by
        // distance travelled), so speed is what separates them — and the fast ones are where a bot
        // that cannot hold a line shows it.
        for (i, s) in suite.iter().enumerate().take(limit) {
            let (a, b) = (s.motions[0].origin, s.motions[s.motions.len() - 1].origin);
            print!("  {i:>4} ");
            println!(
                "  {:.2}s  {:>5.0}u  {:>4.0} ups  ({:>6.0},{:>6.0},{:>5.0}) -> ({:>6.0},{:>6.0},{:>5.0})  {}",
                s.duration(),
                s.path_length(),
                s.mean_speed(),
                a.x,
                a.y,
                a.z,
                b.x,
                b.y,
                b.z,
                demo.players
                    .get(s.player as usize)
                    .map_or_else(|| format!("#{}", s.player), |p| p.label(s.player)),
            );
        }
    }
    exit_code(had_error)
}

// ── route ─────────────────────────────────────────────────────────────────────────────────────

/// Find every time any player travelled from A to B, and report how they did it.
///
/// This is how a bot route gets a target. "The bot takes 10 s to get from the stairs to the red
/// armour" means nothing on its own; "and eight humans did it 40 times, best 3.1 s, median 4.0 s"
/// turns it into a gap with a size. The spread across runs matters as much as the best one — it
/// separates a route the bot is merely slower on from one it is doing wrong.
fn run_route(argv: impl Iterator<Item = String>) -> ExitCode {
    let (mut files, mut nums) = (Vec::new(), Vec::new());
    let (mut radius, mut max_secs, mut max_detour) = (64.0f32, 20.0f32, 2.0f32);
    let mut it = argv;
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            "--radius" => match it.next().and_then(|v| v.parse().ok()) {
                Some(r) => radius = r,
                None => return usage_err("--radius needs a number"),
            },
            "--max-secs" => match it.next().and_then(|v| v.parse().ok()) {
                Some(s) => max_secs = s,
                None => return usage_err("--max-secs needs a number"),
            },
            "--max-detour" => match it.next().and_then(|v| v.parse().ok()) {
                Some(d) => max_detour = d,
                None => return usage_err("--max-detour needs a number"),
            },
            // Coordinates are negative half the time, so a leading `-` cannot mean "option" here:
            // try to read every bare argument as a number first.
            _ => match arg.parse::<f32>() {
                Ok(n) => nums.push(n),
                Err(_) if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
                Err(_) => files.push(PathBuf::from(arg)),
            },
        }
    }
    if nums.len() != 6 {
        return usage_err("route needs six numbers: FROM_X FROM_Y FROM_Z TO_X TO_Y TO_Z");
    }
    let from = Vec3::new(nums[0], nums[1], nums[2]);
    let to = Vec3::new(nums[3], nums[4], nums[5]);

    let paths = resolve_paths(files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }
    let mut had_error = false;
    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        let mut runs: Vec<(u8, rtx_demo_tool::line::Traversal)> = Vec::new();
        for p in analysis::players(&demo) {
            for t in analysis::track(&demo, p).traversals(from, to, radius, max_secs, max_detour) {
                runs.push((p, t));
            }
        }
        println!(
            "{}: {} run(s) from ({:.0},{:.0},{:.0}) to ({:.0},{:.0},{:.0}) within {radius:.0}u",
            basename(&demo.path),
            runs.len(),
            from.x,
            from.y,
            from.z,
            to.x,
            to.y,
            to.z
        );
        if runs.is_empty() {
            continue;
        }
        runs.sort_by(|a, b| a.1.duration().total_cmp(&b.1.duration()));
        let times: Vec<f32> = runs.iter().map(|(_, t)| t.duration()).collect();
        let median = times[times.len() / 2];
        println!(
            "  best {:.2}s   median {:.2}s   worst {:.2}s",
            times[0],
            median,
            times[times.len() - 1]
        );
        let reachable = ((to - from).length() - 2.0 * radius).max(1.0);
        for (p, t) in runs.iter().take(5) {
            println!(
                "    {:.2}s  {:<16} path {:.0}u ({:.1}x straight)  mean {:.0} ups  t={:.1}s",
                t.duration(),
                demo.players
                    .get(*p as usize)
                    .map_or_else(|| format!("#{p}"), |s| s.label(*p)),
                t.path_length(),
                t.path_length() / reachable,
                t.mean_speed(),
                t.start_time,
            );
        }
    }
    exit_code(had_error)
}

// ── players ───────────────────────────────────────────────────────────────────────────────────

/// List who is in a demo. The first thing you want from a file you haven't seen: a multi-view
/// recording of a 4-on-4 holds eight players plus spectators across thirty-two slots, and every
/// other subcommand needs a slot or a name to talk about.
/// `teamplay` — the per-team report an A/B is read from.
fn run_teamplay(argv: impl Iterator<Item = String>) -> ExitCode {
    use rtx_demo_tool::teamplay::{self, Zone};

    let mut files = Vec::new();
    let mut zones: Vec<Zone> = Vec::new();
    let mut show_players = false;
    let mut splits = 1usize;
    let mut argv = argv.peekable();
    while let Some(arg) = argv.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            "--players" => show_players = true,
            "--split" => match argv.next().as_deref().and_then(|s| s.parse::<usize>().ok()) {
                Some(n) if n >= 1 => splits = n,
                _ => return usage_err("--split needs a positive count"),
            },
            "--zones" => match argv.next().as_deref() {
                Some("dm3") => zones.extend(teamplay::dm3_zones()),
                Some(other) => return usage_err(&format!("unknown zone set {other:?}")),
                None => return usage_err("--zones needs a name"),
            },
            "--zone" => {
                let Some(spec) = argv.next() else {
                    return usage_err("--zone needs NAME,X,Y,Z[,RADIUS[,HALFHEIGHT]]");
                };
                let parts: Vec<&str> = spec.split(',').collect();
                if parts.len() < 4 {
                    return usage_err("--zone needs at least NAME,X,Y,Z");
                }
                let num = |s: &str| s.trim().parse::<f32>().ok();
                let (Some(x), Some(y), Some(z)) = (num(parts[1]), num(parts[2]), num(parts[3])) else {
                    return usage_err(&format!("bad coordinates in --zone {spec:?}"));
                };
                zones.push(Zone {
                    name: parts[0].to_string(),
                    center: glam::Vec3::new(x, y, z),
                    radius: parts.get(4).and_then(|s| num(s)).unwrap_or(320.0),
                    half_height: parts.get(5).and_then(|s| num(s)).unwrap_or(96.0),
                });
            }
            _ if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
            _ => files.push(PathBuf::from(arg)),
        }
    }
    let paths = resolve_paths(files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }

    let mut had_error = false;
    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        let zones = if zones.is_empty() && demo.levelname.contains("dm3") {
            teamplay::dm3_zones()
        } else {
            zones.clone()
        };
        let r = teamplay::report(&demo, zones);
        println!(
            "{} [{}] {} — {:.0}s",
            basename(&demo.path),
            demo.format.as_str(),
            if demo.levelname.is_empty() {
                "?"
            } else {
                &demo.levelname
            },
            r.duration
        );
        if demo.format != rtx_demo_tool::Format::Mvd {
            println!("  note: not a server recording — only the players the recorder saw are present");
        }
        for w in &r.warnings {
            println!("\n  !! CONTAMINATED: {w}");
        }
        if !r.warnings.is_empty() {
            println!("  !! Do not read the score below as a result.");
        }

        // Slices first, because a single window is not the result: on one split-team match the first
        // 22 minutes read 1.49:1 to the treated side and the next 9 read 0.58:1. If these disagree,
        // the total below is an average over a changing effect, not a measurement of a fixed one.
        if splits > 1 && r.duration > 0.0 {
            println!("\n  by time slice (frags scored within each):");
            let width = r.duration / splits as f32;
            for i in 0..splits {
                let (from, to) = (i as f32 * width, (i + 1) as f32 * width);
                let part = teamplay::report(&teamplay::slice(&demo, from, to), r.zones.clone());
                let line: Vec<String> = part
                    .teams
                    .iter()
                    .map(|t| format!("{} {}", if t.team.is_empty() { "?" } else { &t.team }, t.frags))
                    .collect();
                println!("    {:>5.0}–{:<5.0}s   {}", from, to, line.join("   "));
            }
        }

        println!("\n  team          frags  deaths   k/d   idle%  mean_ups  spread  longest_stay");
        for t in &r.teams {
            let idle_pct = if t.alive_secs > 0.0 {
                100.0 * t.idle_secs / t.alive_secs
            } else {
                0.0
            };
            let kd = if t.deaths > 0 {
                t.frags as f32 / t.deaths as f32
            } else {
                t.frags as f32
            };
            let stay = match t.longest_stay_zone {
                Some(zi) => format!("{:.0}s in {}", t.longest_stay, r.zones[zi].name),
                None => "-".to_string(),
            };
            println!(
                "  {:<12} {:>5}  {:>6}  {:>4.2}  {:>5.1}  {:>8.0}  {:>6.0}  {}",
                if t.team.is_empty() { "(none)" } else { &t.team },
                t.frags,
                t.deaths,
                kd,
                idle_pct,
                t.mean_speed,
                t.mean_spread,
                stay
            );
        }

        if !r.zones.is_empty() {
            println!("\n  room control (seconds held / of which standing still / deaths inside)");
            print!("  {:<12}", "team");
            for z in &r.zones {
                print!("  {:>18}", z.name);
            }
            println!();
            for t in &r.teams {
                print!("  {:<12}", if t.team.is_empty() { "(none)" } else { &t.team });
                for zi in 0..r.zones.len() {
                    print!(
                        "  {:>6.0} {:>5.0} {:>4}",
                        t.zone_secs[zi], t.zone_idle_secs[zi], t.zone_deaths[zi]
                    );
                }
                println!();
            }
        }

        if show_players {
            println!("\n  player            team    frags  deaths  idle%  mean_ups  longest_stay");
            let mut ps: Vec<_> = r.players.iter().collect();
            ps.sort_by(|a, b| b.frags.cmp(&a.frags));
            for p in ps {
                let idle_pct = if p.alive_secs > 0.0 {
                    100.0 * p.idle_secs / p.alive_secs
                } else {
                    0.0
                };
                let stay = match p.longest_stay_zone {
                    Some(zi) => format!("{:.0}s in {}", p.longest_stay, r.zones[zi].name),
                    None => "-".to_string(),
                };
                println!(
                    "  {:<16}  {:<6} {:>5}  {:>6}  {:>5.1}  {:>8.0}  {}",
                    p.name, p.team, p.frags, p.deaths, idle_pct, p.mean_speed, stay
                );
            }
        }
        println!();
    }
    if had_error {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

fn run_players(argv: impl Iterator<Item = String>) -> ExitCode {
    let mut files = Vec::new();
    for arg in argv {
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            _ if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
            _ => files.push(PathBuf::from(arg)),
        }
    }
    let paths = resolve_paths(files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }
    let mut had_error = false;
    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        let span = demo
            .frames
            .last()
            .zip(demo.frames.first())
            .map_or(0.0, |(l, f)| l.time - f.time);
        println!(
            "{} [{}] {} — {:.1}s, {} frames",
            basename(&demo.path),
            demo.format.as_str(),
            if demo.levelname.is_empty() {
                "?"
            } else {
                &demo.levelname
            },
            span,
            demo.frames.len()
        );
        for (slot, p) in demo.players.iter().enumerate() {
            if !p.present() {
                continue;
            }
            let mut tags = Vec::new();
            if p.spectator {
                tags.push("spectator".to_string());
            }
            if !p.team.is_empty() {
                tags.push(format!("team {}", p.team));
            }
            if demo.local_player == Some(slot as u8) {
                tags.push("recorder".to_string());
            }
            println!(
                "  {slot:>2}  {:<20} {:>7} frames{}{}",
                p.label(slot as u8),
                p.frames,
                if tags.is_empty() { "" } else { "  " },
                tags.join(", ")
            );
        }
    }
    exit_code(had_error)
}

// ── dump ──────────────────────────────────────────────────────────────────────────────────────

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

/// Parsed `qwd dump` options.
struct DumpArgs {
    files: Vec<PathBuf>,
    raw: bool,
    player: Option<String>,
    all_players: bool,
    no_header: bool,
    help: bool,
}

fn run_dump(argv: impl Iterator<Item = String>) -> ExitCode {
    let mut a = DumpArgs {
        files: Vec::new(),
        raw: false,
        player: None,
        all_players: false,
        no_header: false,
        help: false,
    };
    let mut it = argv;
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--raw" => a.raw = true,
            "--all-players" => a.all_players = true,
            "--no-header" => a.no_header = true,
            "-h" | "--help" => a.help = true,
            "--player" => match it.next() {
                Some(v) => a.player = Some(v),
                None => return usage_err("--player needs a slot number or name"),
            },
            _ if arg.starts_with("--player=") => a.player = Some(arg["--player=".len()..].to_string()),
            "--" => a.files.extend(it.by_ref().map(PathBuf::from)),
            _ if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
            _ => a.files.push(PathBuf::from(arg)),
        }
    }
    if a.help {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }

    let paths = resolve_paths(a.files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut had_error = false;

    if !a.no_header {
        let header: Vec<&str> = if a.raw {
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
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        for w in &demo.warnings {
            eprintln!("qwd: {}: {w}", path.display());
            had_error = true;
        }

        let slot = match a.player.as_deref().map(|v| resolve_player(&demo, v)) {
            Some(Ok(s)) => Some(s),
            Some(Err(e)) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
            None => None,
        };
        let rows = if a.raw {
            raw_rows(&demo, slot, a.all_players)
        } else {
            combined_rows(&demo, slot, a.all_players)
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
    exit_code(had_error)
}

/// Which player's frames to emit: an explicit slot, the local player, or all of them (`None`).
///
/// A `.qwd` has an obvious default — the client that recorded it. A multi-view demo does not: it is
/// a recording of everybody, so defaulting to a slot would silently report one of eight players as
/// if it were *the* player. There, no selection means all of them.
fn selected_player(demo: &Demo, explicit: Option<u8>, all: bool) -> Option<u8> {
    if all {
        None
    } else if explicit.is_some() {
        explicit
    } else {
        demo.local_player
    }
}

/// Resolve a `--player` argument that may name a slot *or* a player. Names are matched
/// case-insensitively and by substring, because QuakeWorld names are full of punctuation and
/// colour codes that are miserable to type exactly.
fn resolve_player(demo: &Demo, arg: &str) -> Result<u8, String> {
    if let Ok(slot) = arg.parse::<u8>() {
        return Ok(slot);
    }
    let needle = arg.to_ascii_lowercase();
    let hits: Vec<u8> = demo
        .players
        .iter()
        .enumerate()
        .filter(|(_, p)| p.present() && p.name.to_ascii_lowercase().contains(&needle))
        .map(|(i, _)| i as u8)
        .collect();
    match hits.as_slice() {
        [one] => Ok(*one),
        [] => Err(format!("no player matching {arg:?}")),
        many => {
            let names: Vec<String> = many.iter().map(|&s| demo.players[s as usize].label(s)).collect();
            Err(format!("{arg:?} matches several players: {}", names.join(", ")))
        }
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
    if let Some(cmd) = frame.command {
        return Some(ResolvedCmd {
            source: CmdSource::PlayerInfo,
            time: frame.time,
            cmd,
        });
    }
    let use_demo = match demo.local_player {
        None => true,
        Some(local) => frame.player == local,
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
        .filter(|f| player.is_none_or(|p| f.player == p))
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
        if player.is_some_and(|p| f.player != p) {
            continue;
        }
        let resolved = f.command.map(|cmd| ResolvedCmd {
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
    // `velocity_present` says whether the *wire* carried a velocity, not whether one can be had:
    // an MVD never does, and its speeds come from differencing successive origins instead.
    let v = frame.velocity;
    vec![
        basename(&demo.path),
        fmt_float(frame.time),
        frame.player.to_string(),
        fmt_float(frame.origin.x),
        fmt_float(frame.origin.y),
        fmt_float(frame.origin.z),
        if v.is_some() { "1" } else { "0" }.to_string(),
        opt_int(v.map(|v| v.x as i64)),
        opt_int(v.map(|v| v.y as i64)),
        opt_int(v.map(|v| v.z as i64)),
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

// ── analyze ───────────────────────────────────────────────────────────────────────────────────

/// Parsed `qwd analyze` options.
struct AnalyzeArgs {
    files: Vec<PathBuf>,
    player: Option<String>,
    all_players: bool,
    waypoints: usize,
    help: bool,
}

fn run_analyze(argv: impl Iterator<Item = String>) -> ExitCode {
    let mut a = AnalyzeArgs {
        files: Vec::new(),
        player: None,
        all_players: false,
        waypoints: 0,
        help: false,
    };
    let mut it = argv;
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--all-players" => a.all_players = true,
            "-h" | "--help" => a.help = true,
            "--player" => match it.next() {
                Some(v) => a.player = Some(v),
                None => return usage_err("--player needs a slot number or name"),
            },
            _ if arg.starts_with("--player=") => a.player = Some(arg["--player=".len()..].to_string()),
            "--waypoints" => match it.next().and_then(|v| v.parse().ok()) {
                Some(n) => a.waypoints = n,
                None => return usage_err("--waypoints needs a count"),
            },
            _ if arg.starts_with("--waypoints=") => match arg["--waypoints=".len()..].parse() {
                Ok(n) => a.waypoints = n,
                Err(_) => return usage_err("--waypoints needs a count"),
            },
            "--" => a.files.extend(it.by_ref().map(PathBuf::from)),
            _ if arg.starts_with('-') && arg != "-" => return usage_err(&format!("unknown option {arg}")),
            _ => a.files.push(PathBuf::from(arg)),
        }
    }
    if a.help {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }

    let paths = resolve_paths(a.files);
    if paths.is_empty() {
        eprintln!("qwd: no .qwd or .mvd files found");
        return ExitCode::FAILURE;
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut had_error = false;

    for path in &paths {
        let demo = match parse_demo(path) {
            Ok(demo) => demo,
            Err(e) => {
                eprintln!("qwd: {}: {e}", path.display());
                had_error = true;
                continue;
            }
        };
        for w in &demo.warnings {
            eprintln!("qwd: {}: {w}", path.display());
            had_error = true;
        }

        // With no `--player`, report the recording client — or, in a multi-view demo that has no
        // such thing, everyone. Picking a slot arbitrarily there would report one of eight players
        // as though the file were about them.
        let players = match (a.all_players, a.player.as_deref()) {
            (true, _) => analysis::players(&demo),
            (_, Some(v)) => match resolve_player(&demo, v) {
                Ok(s) => vec![s],
                Err(e) => {
                    eprintln!("qwd: {}: {e}", path.display());
                    had_error = true;
                    continue;
                }
            },
            (_, None) => match demo.local_player {
                Some(p) => vec![p],
                None => analysis::players(&demo),
            },
        };
        let _ = writeln!(out, "{}", basename(&demo.path));
        for p in players {
            if report_player(&mut out, &demo, p, a.waypoints).is_err() {
                return exit_code(had_error);
            }
        }
    }

    let _ = out.flush();
    exit_code(had_error)
}

/// Print one player's movement report: the summary block, then an optional waypoint table.
fn report_player(out: &mut impl Write, demo: &Demo, player: u8, waypoints: usize) -> io::Result<()> {
    let track = analysis::track(demo, player);
    let s = track.summary();
    if s.frames == 0 {
        return writeln!(out, "  player {player}: no frames");
    }
    let label = demo
        .players
        .get(player as usize)
        .map_or_else(|| format!("#{player}"), |p| p.label(player));
    writeln!(
        out,
        "  player {player} ({label}): {} frames over {:.2}s",
        s.frames, s.duration
    )?;
    writeln!(
        out,
        "    start ({:.0}, {:.0}, {:.0})  ->  end ({:.0}, {:.0}, {:.0})   climb {:+.0}",
        s.start.x, s.start.y, s.start.z, s.end.x, s.end.y, s.end.z, s.height_gain,
    )?;
    writeln!(
        out,
        "    horizontal speed: peak {:.0}  mean {:.0} ups   path {:.0}u   z {:.0}..{:.0}",
        s.peak_speed, s.mean_speed, s.path_length, s.min_z, s.max_z,
    )?;
    // The distribution, over live frames. `share_above` is the honest headline: a percentile alone
    // invites the reader to imagine a lot of idling that isn't there.
    let [_, p50, p90, p99, max] = track.speed_percentiles();
    writeln!(
        out,
        "    speed: p50 {p50:.0}  p90 {p90:.0}  p99 {p99:.0}  max {max:.0} ups   \
         {:.0}% of the time above 200, {:.0}% above 400   dead {:.0}%",
        100.0 * track.share_above(200.0),
        100.0 * track.share_above(400.0),
        100.0 * track.dead_share(),
    )?;
    let jumps = track.jumps();
    if !jumps.is_empty() {
        let gained = jumps.iter().filter(|j| j.peak_speed > j.takeoff_speed + 5.0).count();
        let longest = jumps.iter().fold(None::<&analysis::Jump>, |b, j| {
            Some(match b {
                Some(b) if b.distance >= j.distance => b,
                _ => j,
            })
        });
        writeln!(
            out,
            "    jumps: {} ({} gained speed in the air)   mean {:.0}u / {:.2}s",
            jumps.len(),
            gained,
            jumps.iter().map(|j| j.distance).sum::<f32>() / jumps.len() as f32,
            jumps.iter().map(|j| j.airtime).sum::<f32>() / jumps.len() as f32,
        )?;
        if let Some(j) = longest {
            writeln!(
                out,
                "      longest {:.0}u at t={:.1}s: ({:.0}, {:.0}, {:.0}) -> ({:.0}, {:.0}, {:.0})  \
                 takeoff {:.0} peak {:.0} ups, apex {:+.0}, {:.2}s",
                j.distance,
                j.takeoff_time,
                j.takeoff.x,
                j.takeoff.y,
                j.takeoff.z,
                j.landing.x,
                j.landing.y,
                j.landing.z,
                j.takeoff_speed,
                j.peak_speed,
                j.apex,
                j.airtime,
            )?;
        }
    }
    if waypoints >= 2 {
        writeln!(out, "    waypoints (t  x  y  z  hspeed  vspeed):")?;
        let t0 = track.motions.first().map_or(0.0, |m| m.time);
        for Motion {
            time,
            origin,
            horizontal_speed,
            vertical_speed,
            ..
        } in track.waypoints(waypoints)
        {
            writeln!(
                out,
                "      {:6.2}  {:6.0} {:6.0} {:6.0}   {:5.0}  {:+5.0}",
                time - t0,
                origin.x,
                origin.y,
                origin.z,
                horizontal_speed,
                vertical_speed,
            )?;
        }
    }
    Ok(())
}

// ── shared ────────────────────────────────────────────────────────────────────────────────────

fn basename(path: &Path) -> String {
    path.file_name()
        .map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned())
}

fn usage_err(msg: &str) -> ExitCode {
    eprintln!("qwd: {msg}");
    ExitCode::from(2)
}

fn exit_code(had_error: bool) -> ExitCode {
    if had_error {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qwd_input_fallback_is_local_and_preserves_present_zero_input() {
        let mut demo = rtx_demo_tool::parse_qwd_bytes(&[], PathBuf::from("input.qwd")).unwrap();
        demo.local_player = Some(0);
        demo.demo_cmds.push(DemoCmd {
            time: 1.0,
            cmd: Usercmd {
                msec: 14,
                forward: 100,
                buttons: 3,
                impulse: 7,
                angles: Vec3::new(10.0, 20.0, 0.0),
                ..Default::default()
            },
        });
        let mut frame = Frame {
            time: 1.01,
            player: 0,
            origin: Vec3::ZERO,
            angles: Vec3::ZERO,
            velocity: Some(Vec3::ZERO),
            command: None,
            dead: false,
            on_ground: Some(true),
            weaponframe: Some(0),
        };
        let resolved = command_for_frame(&demo, &frame).expect("local input fallback");
        assert!(matches!(resolved.source, CmdSource::DemCmd));
        assert_eq!(resolved.time, 1.0);
        assert_eq!(resolved.cmd.angles, Vec3::new(10.0, 20.0, 0.0));
        assert_eq!(
            (
                resolved.cmd.msec,
                resolved.cmd.forward,
                resolved.cmd.buttons,
                resolved.cmd.impulse
            ),
            (14, 100, 3, 7)
        );

        frame.player = 1;
        assert!(
            command_for_frame(&demo, &frame).is_none(),
            "local input must not be attributed to remote players"
        );
        frame.player = 0;
        frame.command = Some(Usercmd::default());
        let resolved = command_for_frame(&demo, &frame).expect("present zero input");
        assert!(matches!(resolved.source, CmdSource::PlayerInfo));
        assert_eq!(
            (
                resolved.cmd.msec,
                resolved.cmd.forward,
                resolved.cmd.buttons,
                resolved.cmd.impulse
            ),
            (0, 0, 0, 0)
        );

        frame.command = None;
        demo.demo_cmds.clear();
        assert!(command_for_frame(&demo, &frame).is_none(), "absent input stays absent");
    }
}
