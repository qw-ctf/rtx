// SPDX-License-Identifier: AGPL-3.0-or-later

//! The KTX `.route` file parser: `parse_route_file` turns the text command grammar
//! (`race_route_add_start` / `race_add_route_node` / `race_set_route_*` / …) into the route data
//! model in the parent module, plus its private tokenizer / `\\`-unescape / float helpers. Any
//! error discards every route in the file (KTX memset semantics).

use glam::Vec3;

use super::{
    RaceFalseStartMode, RaceNodeType, RaceRoute, RaceRouteNode, RaceTeleportFlag, RaceWeaponMode, RouteFile,
    RouteFileError, MAX_ROUTES, MAX_ROUTE_NODES,
};

/// Parse the ktx route-command format (race.c:3839-4193): one command per line, `//` comments,
/// quoted strings as single tokens. Routes open with `race_route_add_start`, gain nodes and
/// settings, and commit on `race_route_add_end`; a file ending mid-definition drops the
/// uncommitted route (as ktx does, with a warning here).
pub fn parse_route_file(text: &str) -> Result<RouteFile, RouteFileError> {
    let mut out = RouteFile::default();
    // The route under construction; `Some` while between add_start and add_end.
    let mut current: Option<RaceRoute> = None;
    let err = |line: usize, msg: String| Err(RouteFileError { line, msg });

    for (idx, raw) in text.lines().enumerate() {
        let line = idx + 1;
        let args = tokenize(raw);
        let Some(cmd) = args.first() else {
            continue;
        };
        // Commands valid only inside a route definition borrow `current` mutably here; the
        // "outside a definition" error text mirrors ktx's.
        match cmd.as_str() {
            "race_route_add_start" => {
                if current.is_some() {
                    return err(line, "race_route_add_start in route definition".into());
                }
                if out.routes.len() >= MAX_ROUTES {
                    // Not an error in ktx: earlier routes stay valid, the rest are ignored.
                    out.warnings
                        .push(format!("#{line}: routes ignored, limit is {MAX_ROUTES} routes/map"));
                    break;
                }
                current = Some(RaceRoute::default());
            }
            "race_add_route_node" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_add_route_node outside of route definition".into());
                };
                if args.len() != 6 {
                    return err(
                        line,
                        format!("race_add_route_node should have 5 arguments, found {}", args.len() - 1),
                    );
                }
                if route.nodes.len() >= MAX_ROUTE_NODES {
                    // ktx drops the node silently (race_add_route_node returns NULL when full);
                    // surface it, since a truncated route is almost certainly a mistake.
                    out.warnings
                        .push(format!("#{line}: node ignored, limit is {MAX_ROUTE_NODES} nodes/route"));
                    continue;
                }
                let n: Vec<f32> = args[1..6].iter().map(|a| parse_f32(a)).collect();
                // First node is the start; every later node lands as the end, demoting the
                // previous non-start node to a checkpoint (race.c:3911-3923).
                let kind = if route.nodes.is_empty() {
                    RaceNodeType::Start
                } else {
                    if route.nodes.len() > 1 {
                        route.nodes.last_mut().unwrap().kind = RaceNodeType::Checkpoint;
                    }
                    RaceNodeType::End
                };
                route.nodes.push(RaceRouteNode {
                    kind,
                    origin: Vec3::new(n[0], n[1], n[2]),
                    pitch: n[3],
                    yaw: n[4],
                    size: Vec3::ZERO,
                });
            }
            "race_set_route_name" => {
                if args.len() != 3 {
                    return err(
                        line,
                        format!("race_set_route_name should have 2 arguments, found {}", args.len() - 1),
                    );
                }
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_name outside of route definition".into());
                };
                route.name = args[1].clone();
                route.desc = unescape_desc(&args[2]);
            }
            "race_set_route_timeout" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_timeout outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_timeout: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                let t = parse_f32(&args[1]);
                if t > 0.0 {
                    route.timeout = t.clamp(1.0, 999.0);
                }
            }
            "race_set_route_weapon_mode" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_weapon_mode outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_weapon_mode: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                route.weapon = match args[1].as_str() {
                    "raceWeaponNo" => RaceWeaponMode::No,
                    "raceWeaponAllowed" => RaceWeaponMode::Allowed,
                    "raceWeapon2s" => RaceWeaponMode::After2s,
                    other => {
                        return err(line, format!("race_set_route_weapon_mode: invalid argument {other}"));
                    }
                };
            }
            "race_set_route_falsestart_mode" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_route_falsestart_mode outside of route definition".into());
                };
                if args.len() != 2 {
                    return err(
                        line,
                        format!("race_set_route_falsestart_mode: expected 1 argument, found {}", args.len() - 1),
                    );
                }
                route.falsestart = match args[1].as_str() {
                    "raceFalseStartNo" => RaceFalseStartMode::No,
                    "raceFalseStartYes" => RaceFalseStartMode::Yes,
                    other => {
                        return err(line, format!("race_set_route_falsestart_mode: invalid argument {other}"));
                    }
                };
            }
            "race_set_node_size" => {
                let Some(route) = current.as_mut() else {
                    return err(line, "race_set_node_size outside of route definition".into());
                };
                if args.len() != 4 {
                    return err(
                        line,
                        format!("race_set_node_size: expected 3 arguments, found {}", args.len() - 1),
                    );
                }
                let Some(node) = route.nodes.last_mut() else {
                    return err(line, "race_set_node_size: no node to amend".into());
                };
                node.size = Vec3::new(parse_f32(&args[1]), parse_f32(&args[2]), parse_f32(&args[3]));
            }
            "race_set_teleport_flags_by_name" => {
                if current.is_some() {
                    return err(line, "race_set_teleport_flags_by_name inside route definition".into());
                }
                if args.len() != 3 {
                    return err(
                        line,
                        format!("race_set_teleport_flags_by_name: expected 2 arguments, found {}", args.len() - 1),
                    );
                }
                // Unknown flag names are silently ignored, as in ktx.
                let flag = match args[2].as_str() {
                    "RACEFLAG_TOUCH_RACEFAIL" => Some(RaceTeleportFlag::Fail),
                    "RACEFLAG_TOUCH_RACEEND" => Some(RaceTeleportFlag::End),
                    _ => None,
                };
                if let Some(flag) = flag {
                    out.teleport_flags.push((args[1].clone(), flag));
                }
            }
            "race_route_add_end" => {
                let Some(route) = current.take() else {
                    return err(line, "race_route_add_end outside of route definition".into());
                };
                out.routes.push(route);
            }
            other => {
                return err(line, format!("unknown route instruction {other}"));
            }
        }
    }
    if current.is_some() {
        out.warnings
            .push("file ended inside a route definition; last route dropped".into());
    }
    Ok(out)
}

/// Split a route-file line into tokens: whitespace-separated, `"quoted strings"` as one token
/// (quotes stripped), `//` starting a comment outside quotes.
fn tokenize(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = line.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '"' {
            chars.next();
            let mut tok = String::new();
            for c in chars.by_ref() {
                if c == '"' {
                    break;
                }
                tok.push(c);
            }
            out.push(tok);
        } else {
            let mut tok = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_whitespace() || c == '"' {
                    break;
                }
                tok.push(c);
                chars.next();
            }
            if let Some(rest) = tok.find("//") {
                tok.truncate(rest);
                if !tok.is_empty() {
                    out.push(tok);
                }
                return out;
            }
            out.push(tok);
        }
    }
    out
}

/// Unescape a route description: `\\` → `\`, and ktx's colored-text escape `\abc` (three
/// digits) → the byte `16a + 8b + c` (race.c:3954-3972), kept as a Latin-1 codepoint.
fn unescape_desc(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::new();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'\\' && i + 1 < b.len() && b[i + 1] == b'\\' {
            out.push('\\');
            i += 2;
        } else if b[i] == b'\\'
            && i + 3 < b.len()
            && b[i + 1].is_ascii_digit()
            && b[i + 2].is_ascii_digit()
            && b[i + 3].is_ascii_digit()
        {
            let v = 16 * (b[i + 1] - b'0') as u32 + 8 * (b[i + 2] - b'0') as u32 + (b[i + 3] - b'0') as u32;
            out.push(char::from_u32(v).unwrap_or('?'));
            i += 4;
        } else {
            out.push(b[i] as char);
            i += 1;
        }
    }
    out
}

/// Parse a float the way ktx's `atof` does: garbage → `0.0`.
fn parse_f32(s: &str) -> f32 {
    s.trim().parse().unwrap_or(0.0)
}
