// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-waypoint-check` — do our generated navmesh's rocket-jump and curl-jump links cover the
//! connections a human authored in KTX's hand-crafted waypoint files?
//!
//! KTX's `.bot` files encode a human's routing knowledge, including rocket jumps and air-control
//! (curl) jumps distilled from recorded play. This tool parses a map's `.bot` file, rebuilds our
//! navmesh from the same BSP the game would load, and reports — per authored RJ/curl path — whether
//! our mesh reproduces it, crosses the gap another way, merely routes around it, or can't connect
//! the endpoints at all. The last case is a blind spot in our link generation worth chasing.
//!
//! It runs fully offline over a stock install (`playground/` by default): no server, no network.
//!
//! ```sh
//! cargo run --release -p rtx-waypoint-check -- dm3 dm4 dm6 e1m2 bravado
//! cargo run --release -p rtx-waypoint-check            # every waypoints/*.bot whose BSP resolves
//! ```

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use rtx_nav::bsp::Bsp;

mod botfile;
mod check;
mod ent;
mod pak;
mod report;

use check::{Checker, Family};
use report::Tally;

const USAGE: &str = "\
rtx-waypoint-check — check KTX waypoint RJ/curl coverage against the generated navmesh

usage: rtx-waypoint-check [options] [map ...]

  --basedir <dir>    Quake dir holding qw/ and id1/ (default: playground)
  --waypoints <dir>  directory of KTX .bot files       (default: waypoints)
  --radius <units>   endpoint match radius             (default: 96)
  -h, --help         show this help

With no maps named, every <waypoints>/*.bot whose BSP can be resolved is checked.
Exit: 0 all paths at least route-connected; 1 some endpoints unreachable/off-mesh; 2 usage/IO error.
";

struct Config {
    basedir: PathBuf,
    waypoints: PathBuf,
    radius: f32,
    maps: Vec<String>,
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let config = match parse_args(&argv) {
        Ok(c) => c,
        Err(msg) => {
            eprint!("{msg}");
            return ExitCode::from(2);
        }
    };
    run(&config)
}

fn parse_args(argv: &[String]) -> Result<Config, String> {
    let mut basedir = PathBuf::from("playground");
    let mut waypoints = PathBuf::from("waypoints");
    let mut radius = 96.0f32;
    let mut maps = Vec::new();

    let mut it = argv.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "-h" | "--help" => return Err(USAGE.to_string()),
            "--basedir" => basedir = it.next().ok_or("--basedir needs a value\n")?.into(),
            "--waypoints" => waypoints = it.next().ok_or("--waypoints needs a value\n")?.into(),
            "--radius" => {
                radius = it
                    .next()
                    .ok_or("--radius needs a value\n")?
                    .parse()
                    .map_err(|_| "--radius must be a number\n".to_string())?;
            }
            other if other.starts_with('-') => return Err(format!("unknown option: {other}\n{USAGE}")),
            other => maps.push(other.to_string()),
        }
    }
    Ok(Config {
        basedir,
        waypoints,
        radius,
        maps,
    })
}

fn run(config: &Config) -> ExitCode {
    let explicit = !config.maps.is_empty();
    let maps = if explicit {
        config.maps.clone()
    } else {
        match list_waypoint_maps(&config.waypoints) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::from(2);
            }
        }
    };

    let mut grand_rj = Tally::default();
    let mut grand_curl = Tally::default();
    let mut any_hole = false;
    let mut fatal = false;
    let mut skipped: Vec<String> = Vec::new();
    let mut processed = 0;

    for map in &maps {
        let text = match std::fs::read_to_string(config.waypoints.join(format!("{map}.bot"))) {
            Ok(t) => t,
            Err(_) => {
                note_missing(explicit, &mut fatal, &mut skipped, map, "no .bot file");
                continue;
            }
        };
        let bytes = match pak::resolve_bsp(&config.basedir, map) {
            Some(b) => b,
            None => {
                note_missing(explicit, &mut fatal, &mut skipped, map, "no BSP");
                continue;
            }
        };
        let Some(bsp) = Bsp::parse(&bytes) else {
            note_missing(explicit, &mut fatal, &mut skipped, map, "unparseable BSP");
            continue;
        };

        let (rj, curl) = process_map(map, &text, &bsp, config.radius);
        any_hole |= rj.holes() + curl.holes() > 0;
        grand_rj.merge(&rj);
        grand_curl.merge(&curl);
        processed += 1;
    }

    if !skipped.is_empty() {
        println!("\nskipped: {}", skipped.join(", "));
    }
    if processed > 1 {
        println!(
            "\n== totals ({processed} maps): rj {} | curl {}",
            grand_rj.line(),
            grand_curl.line()
        );
    }

    if fatal {
        ExitCode::from(2)
    } else if any_hole {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

/// Parse, build, classify, and print one map. Returns its (rocket-jump, curl) tallies.
fn process_map(map: &str, text: &str, bsp: &Bsp, radius: f32) -> (Tally, Tally) {
    let file = botfile::parse(text);
    let markers = ent::marker_walk(bsp);
    let k = markers.len() as u32;
    let implied = file.implied_entity_markers();
    let (paths, dropped) = botfile::resolve(&file, &markers);

    let rj_paths: Vec<_> = paths.iter().filter(|p| p.is_rj()).collect();
    let curl_paths: Vec<_> = paths.iter().filter(|p| p.is_curl()).collect();

    eprintln!("{map}: building navmesh…");
    let graph = check::build(bsp);
    let checker = Checker::new(&graph, radius);

    // Header.
    let k_note = if k == implied {
        "K ok".to_string()
    } else {
        format!("K MISMATCH walk={k} file-implies={implied}")
    };
    let drop_note = if dropped > 0 {
        format!("; {dropped} refs dropped")
    } else {
        String::new()
    };
    println!(
        "\n== {map}: {} created + {k} entity markers ({k_note}); {} rj, {} curl paths{drop_note}",
        file.created.len(),
        rj_paths.len(),
        curl_paths.len(),
    );
    println!(
        "   mesh: {} cells, {} links ({} rj, {} curl); {} func_plat (plats not spliced offline)",
        graph.cells.len(),
        graph.links.len(),
        checker.rj_link_count(),
        checker.curl_link_count(),
        ent::plat_count(bsp),
    );
    if k != implied {
        eprintln!("{map}: WARN marker numbering mismatch — entity-marker positions may be unreliable");
    }

    let mut rj_tally = Tally::default();
    for p in &rj_paths {
        let v = checker.classify(p, Family::RocketJump);
        println!("{}", report::path_line(Family::RocketJump, p, &v));
        rj_tally.add(&v);
    }
    let mut curl_tally = Tally::default();
    for p in &curl_paths {
        let v = checker.classify(p, Family::Curl);
        println!("{}", report::path_line(Family::Curl, p, &v));
        curl_tally.add(&v);
    }

    println!("-- {map}: rj {} | curl {}", rj_tally.line(), curl_tally.line());
    (rj_tally, curl_tally)
}

fn note_missing(explicit: bool, fatal: &mut bool, skipped: &mut Vec<String>, map: &str, why: &str) {
    if explicit {
        eprintln!("error: {map}: {why}");
        *fatal = true;
    } else {
        skipped.push(format!("{map} ({why})"));
    }
}

/// Every `<dir>/*.bot`'s map stem, sorted.
fn list_waypoint_maps(dir: &Path) -> Result<Vec<String>, String> {
    let rd = std::fs::read_dir(dir).map_err(|e| format!("reading {}: {e}", dir.display()))?;
    let mut maps: Vec<String> = rd
        .flatten()
        .filter_map(|e| {
            let p = e.path();
            (p.extension()?.eq_ignore_ascii_case("bot"))
                .then(|| p.file_stem()?.to_str().map(str::to_string))
                .flatten()
        })
        .collect();
    maps.sort();
    Ok(maps)
}
