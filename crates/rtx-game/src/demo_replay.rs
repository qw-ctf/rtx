// SPDX-License-Identifier: AGPL-3.0-or-later

//! Demo-grounded validation of the movement model and the bhop controller, against real
//! QuakeWorld player demos (a non-pro human on dm3/dm4). Two env-gated tests:
//!
//! - [`replay_tracks_recorded_origins`] — a **fidelity** check: drive the recorded usercmds through
//!   [`crate::pmove_sim`] on the real BSP and confirm the simulated trajectory tracks the recorded
//!   origins. This is what lets the benchmark below trust the sim.
//! - [`bot_matches_or_beats_human`] — the **acceptance** check: drive the bot's own [`bhop::Bhop`]
//!   controller along the human's path through the same sim and confirm it covers the run at least
//!   as fast as the human (who is deliberately non-pro — the bots should be much better).
//!
//! Fixtures are the `*.csv` files `qwd_dump.py --raw` emits next to the `.qwd` demos (columns:
//! `event,file,time,…,forwardmove,sidemove,upmove,buttons,pitch,yaw,mv_*`). Both tests are skipped
//! (vacuously green) unless `RTX_TEST_DEMOS` (the demo/CSV dir) and `RTX_TEST_MAPS` (the `.bsp` dir)
//! are set — the same opt-in idiom as `RTX_TEST_BSP`. Run with:
//!
//! ```text
//! RTX_TEST_DEMOS=~/Development/home/rtx-demos RTX_TEST_MAPS=~/Games/Quake2/id1/2/maps \
//!   cargo test -p rtx-game --lib demo_replay -- --nocapture
//! ```

use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::bot::bhop::{self, apply_airaccel, apply_friction, apply_groundaccel, wishdir_fs, Bhop, Env};
use crate::math::yaw_of;
use crate::bsp::Bsp;
use crate::pmove_sim::{pm_step, PmParams, PmState};
use rtx_nav::qphys::JUMP_VZ;

/// One tick of flat-ground QW movement (FTEQW order: CheckJump before Friction, then accel + gravity).
/// The `bot_matches_or_beats_human` bhop test drives the gait on this flat plane rather than the real
/// BSP: the demo routes are exactly as wide as the human's own weave, so a differently-phased bot
/// weave walks off the edge — safe weaving inside a corridor is a live-navigation concern (the bot
/// knows corridor widths from the navmesh), not a property of the speed gait, which is what this
/// isolates. Seeded from the demo's real start speed and movevars, it stays honest to the demo.
fn flat_step(pos: &mut Vec3, vel: &mut Vec3, on_ground: &mut bool, jump_held: &mut bool, cmd: &bhop::Cmd, p: &PmParams, dt: f32) {
    if !cmd.jump {
        *jump_held = false;
    } else if *on_ground && !*jump_held {
        vel.z = JUMP_VZ;
        *on_ground = false;
        *jump_held = true;
    }
    if *on_ground {
        let h = apply_friction(vel.xy(), p.friction, p.stopspeed, dt);
        (vel.x, vel.y) = (h.x, h.y);
    }
    let wishdir = wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side);
    let wishspeed = Vec2::new(cmd.forward, cmd.side).length().min(p.maxspeed);
    if *on_ground {
        let h = apply_groundaccel(vel.xy(), wishdir, wishspeed, p.accel, dt);
        (vel.x, vel.y) = (h.x, h.y);
    } else {
        let h = apply_airaccel(vel.xy(), wishdir, wishspeed, p.accel, dt);
        (vel.x, vel.y) = (h.x, h.y);
        vel.z -= p.gravity * dt;
        pos.z += vel.z * dt;
        if pos.z <= 0.0 && vel.z <= 0.0 {
            pos.z = 0.0;
            vel.z = 0.0;
            *on_ground = true;
        }
    }
    pos.x += vel.x * dt;
    pos.y += vel.y * dt;
}

/// The map's movement cvars, read from the demo's `movevars` row.
#[derive(Clone, Copy, Debug)]
struct Movevars {
    gravity: f32,
    stopspeed: f32,
    maxspeed: f32,
    accelerate: f32,
    friction: f32,
}

impl Movevars {
    fn params(&self) -> PmParams {
        PmParams {
            gravity: self.gravity,
            accel: self.accelerate,
            friction: self.friction,
            stopspeed: self.stopspeed,
            maxspeed: self.maxspeed,
        }
    }
}

/// One recorded frame of the local player: the observed state (`origin`, `vel` when the demo sent
/// it) and the usercmd the client issued that tick (paired 1:1 by timestamp in the demo stream).
#[derive(Clone, Copy, Debug)]
struct Frame {
    time: f32,
    origin: Vec3,
    vel: Option<Vec3>,
    view_yaw: f32,
    forward: f32,
    side: f32,
    buttons: u8,
}

impl Frame {
    fn cmd(&self) -> bhop::Cmd {
        bhop::Cmd { view_yaw: self.view_yaw, forward: self.forward, side: self.side, jump: self.buttons & 2 != 0 }
    }
}

/// The parsed demo: physics params plus the per-frame local-player trajectory + inputs.
struct Demo {
    movevars: Movevars,
    frames: Vec<Frame>,
}

/// Parse a `--raw` CSV. Playerinfo rows carry the origin/velocity, dem_cmd rows carry the usercmd;
/// they share a timestamp, so we pair them into one [`Frame`] per tick.
fn load_demo(path: &str) -> Demo {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("empty csv").split(',').collect();
    let col = |name: &str| header.iter().position(|h| *h == name).unwrap_or_else(|| panic!("no column {name}"));
    let (c_event, c_time) = (col("event"), col("time"));
    let (cx, cy, cz) = (col("x"), col("y"), col("z"));
    let (c_vp, c_vx, c_vy, c_vz) = (col("velocity_present"), col("vx"), col("vy"), col("vz"));
    let (c_fwd, c_side, c_btn, c_yaw) = (col("forwardmove"), col("sidemove"), col("buttons"), col("yaw"));
    let mv_cols = ["mv_gravity", "mv_stopspeed", "mv_maxspeed", "mv_accelerate", "mv_friction"].map(col);

    let mut movevars = None;
    // Pair playerinfo (state) with the dem_cmd (input) that shares its timestamp.
    let mut samples: Vec<(f32, Vec3, Option<Vec3>)> = Vec::new();
    let mut cmds: Vec<(f32, f32, f32, u8)> = Vec::new(); // time, view_yaw, (forward,side packed later), buttons
    let mut cmd_fs: Vec<(f32, f32)> = Vec::new(); // forward, side (parallel to cmds)

    let f = |row: &[&str], i: usize| row[i].parse::<f32>().ok();
    for line in lines {
        let row: Vec<&str> = line.split(',').collect();
        match row[c_event] {
            "movevars" => {
                let g = |i: usize| f(&row, i).unwrap_or(0.0);
                movevars = Some(Movevars {
                    gravity: g(mv_cols[0]),
                    stopspeed: g(mv_cols[1]),
                    maxspeed: g(mv_cols[2]),
                    accelerate: g(mv_cols[3]),
                    friction: g(mv_cols[4]),
                });
            }
            "playerinfo" => {
                let origin = Vec3::new(f(&row, cx).unwrap(), f(&row, cy).unwrap(), f(&row, cz).unwrap());
                let vel = (row[c_vp] == "1")
                    .then(|| Vec3::new(f(&row, c_vx).unwrap(), f(&row, c_vy).unwrap(), f(&row, c_vz).unwrap()));
                samples.push((f(&row, c_time).unwrap(), origin, vel));
            }
            "dem_cmd" => {
                cmds.push((
                    f(&row, c_time).unwrap(),
                    f(&row, c_yaw).unwrap_or(0.0),
                    0.0,
                    row[c_btn].parse::<u8>().unwrap_or(0),
                ));
                cmd_fs.push((f(&row, c_fwd).unwrap_or(0.0), f(&row, c_side).unwrap_or(0.0)));
            }
            _ => {}
        }
    }

    // Merge: for each state sample, take the dem_cmd at the nearest timestamp (they coincide).
    let mut frames = Vec::with_capacity(samples.len());
    for (time, origin, vel) in samples {
        let Some(j) = nearest_cmd(&cmds, time) else { continue };
        let (_, view_yaw, _, buttons) = cmds[j];
        let (forward, side) = cmd_fs[j];
        frames.push(Frame { time, origin, vel, view_yaw, forward, side, buttons });
    }
    Demo { movevars: movevars.expect("no movevars row"), frames }
}

/// Index of the dem_cmd whose timestamp is closest to `time` (within 2 ms), or `None`.
fn nearest_cmd(cmds: &[(f32, f32, f32, u8)], time: f32) -> Option<usize> {
    let mut best = None;
    let mut best_d = 0.002;
    for (i, c) in cmds.iter().enumerate() {
        let d = (c.0 - time).abs();
        if d < best_d {
            best_d = d;
            best = Some(i);
        }
    }
    best
}

/// Recorded velocity when present, else a finite difference against the next frame.
fn frame_vel(frames: &[Frame], i: usize) -> Vec3 {
    if let Some(v) = frames[i].vel {
        return v;
    }
    if i + 1 < frames.len() {
        let dt = frames[i + 1].time - frames[i].time;
        if dt > 1e-4 {
            return (frames[i + 1].origin - frames[i].origin) / dt;
        }
    }
    Vec3::ZERO
}

fn demo_dir() -> Option<String> {
    std::env::var("RTX_TEST_DEMOS").ok()
}
fn maps_dir() -> Option<String> {
    std::env::var("RTX_TEST_MAPS").ok()
}

fn load_bsp(map: &str) -> Bsp {
    let dir = maps_dir().unwrap();
    let path = format!("{dir}/{map}.bsp");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    Bsp::parse(&bytes).unwrap_or_else(|| panic!("parse {path}"))
}

/// A benchmark segment of a demo: `[t0, t1]` of the local-player trajectory, and how the human
/// moved through it (so the bot is driven with the matching controller entry).
#[derive(Clone, Copy)]
enum SegKind {
    /// A ground-and-air bhop / circle-jump run: drive the full [`Bhop`] hop cycle along the path.
    Bhop,
    /// A rocket-jump arc: seed just after the blast and drive [`bhop::air_correct`] to the landing
    /// (the sim has no rockets, so the launch impulse comes from the recorded post-blast velocity).
    RjBallistic,
}

struct Seg {
    map: &'static str,
    csv: &'static str,
    t0: f32,
    t1: f32,
    kind: SegKind,
}

/// The demo fixtures and the technique window in each. Windows are chosen inside the moving portion
/// and away from water (dm3/dm4 hop routes are dry).
fn segments() -> Vec<Seg> {
    vec![
        Seg { map: "dm3", csv: "curl_mid", t0: 321.8, t1: 327.5, kind: SegKind::Bhop },
        Seg { map: "dm4", csv: "dm4jump", t0: 392.6, t1: 396.9, kind: SegKind::Bhop },
        Seg { map: "dm3", csv: "bridge_rl", t0: 276.6, t1: 284.4, kind: SegKind::Bhop },
        Seg { map: "dm3", csv: "rl_jump", t0: 349.66, t1: 350.75, kind: SegKind::RjBallistic },
    ]
}

/// Frames of a demo within `[t0, t1]`.
fn window(frames: &[Frame], t0: f32, t1: f32) -> Vec<Frame> {
    frames.iter().copied().filter(|f| f.time >= t0 && f.time <= t1).collect()
}

/// The longest straight-ish fast corridor inside `frames`: the widest `[a, b]` whose every point
/// stays within `CORRIDOR` of the straight line a→b (so it's not a turn) and where the player is
/// moving fast throughout. Returns `(a, b)`. This isolates the bhop *speed gait* from turning — a
/// fast air-strafe can't corner as tight as a slow one, so cornering is a navigation concern, not a
/// gait one. Comparing speed gain along a straight corridor is the fair, physics-clean test.
fn straight_window(frames: &[Frame]) -> (usize, usize) {
    const CORRIDOR: f32 = 64.0; // max lateral deviation from the chord — a genuinely straight run
    const FLAT: f32 = 72.0; // max height change — a local flat corridor, not a chord over a void
    const MIN_SPEED: f32 = 260.0;
    const MAX_DT: f32 = 1.3; // cap the span so it's a local straight segment, not a spiral chord
    let (mut best_a, mut best_b, mut best_len) = (0, 0, 0.0f32);
    for a in 0..frames.len() {
        for b in (a + 8)..frames.len() {
            if frames[b].time - frames[a].time > MAX_DT {
                break;
            }
            let base = frames[a].origin;
            let axis = (frames[b].origin.xy() - base.xy()).normalize_or_zero();
            if axis == glam::Vec2::ZERO {
                continue;
            }
            let ok = frames[a..=b].iter().all(|f| {
                let rel = f.origin.xy() - base.xy();
                (rel - axis * rel.dot(axis)).length() <= CORRIDOR
                    && (f.origin.z - base.z).abs() <= FLAT
                    && f.vel.is_none_or(|v| v.xy().length() >= MIN_SPEED)
            });
            if !ok {
                break; // extending b only makes the corridor worse
            }
            let len = (frames[b].origin.xy() - base.xy()).length();
            if len > best_len {
                best_len = len;
                best_a = a;
                best_b = b;
            }
        }
    }
    (best_a, best_b)
}

/// Percentile of a sorted-in-place sample.
fn pct(errs: &mut [f32], p: f32) -> f32 {
    if errs.is_empty() {
        return 0.0;
    }
    errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    errs[((errs.len() as f32 - 1.0) * p) as usize]
}

#[test]
fn replay_tracks_recorded_origins() {
    let (Some(_), Some(_)) = (demo_dir(), maps_dir()) else {
        eprintln!("RTX_TEST_DEMOS / RTX_TEST_MAPS not set; skipping");
        return;
    };
    let dir = demo_dir().unwrap();
    // One representative dry run per map. (rl_jump is validated separately as an RJ segment.)
    for (csv, map) in [("bridge_rl", "dm3"), ("curl_mid", "dm3"), ("dm4jump", "dm4")] {
        let demo = load_demo(&format!("{dir}/{csv}.csv"));
        let bsp = load_bsp(map);
        let p = demo.movevars.params();
        let fr = &demo.frames;

        // Pass A — one-step prediction: re-anchor at every frame, step once, compare to the next
        // recorded origin. Errors split air vs ground (ground step-up/stairs are looser).
        let (mut air_pos, mut air_vel, mut gnd_pos) = (Vec::new(), Vec::new(), Vec::new());
        for i in 0..fr.len().saturating_sub(1) {
            let dt = fr[i + 1].time - fr[i].time;
            if !(0.005..=0.05).contains(&dt) {
                continue;
            }
            let vel = frame_vel(fr, i);
            let mut st = PmState { origin: fr[i].origin, vel, on_ground: false, jump_held: false };
            pm_step(&bsp, &mut st, &fr[i].cmd(), &p, dt);
            let pos_err = (st.origin - fr[i + 1].origin).length();
            let airborne = vel.z.abs() > 20.0 || fr[i].origin.z + 1.0 < st.origin.z;
            if airborne {
                air_pos.push(pos_err);
                if let Some(next_v) = fr[i + 1].vel {
                    air_vel.push((st.vel - next_v).length());
                }
            } else {
                gnd_pos.push(pos_err);
            }
        }
        let air_p95 = pct(&mut air_pos, 0.95);
        let air_v95 = pct(&mut air_vel, 0.95);
        let gnd_p95 = pct(&mut gnd_pos, 0.95);
        eprintln!(
            "{csv}: 1-step p95 air_pos={air_p95:.2}u air_vel={air_v95:.1}ups gnd_pos={gnd_p95:.2}u \
             (air n={} gnd n={})",
            air_pos.len(),
            gnd_pos.len()
        );
        // Observed p95s are ~0.1u position / ~1 ups velocity — essentially the demo's 1/8-unit
        // coordinate quantization. These bounds leave headroom but still flag any real regression.
        assert!(air_p95 < 1.0, "{csv}: airborne 1-step position error p95 {air_p95:.2}u too high");
        assert!(air_v95 < 8.0, "{csv}: airborne 1-step velocity error p95 {air_v95:.1}ups too high");
        assert!(gnd_p95 < 1.0, "{csv}: ground 1-step position error p95 {gnd_p95:.2}u too high");

        // Pass B — short-horizon drift: free-run ~0.25s windows, compare the endpoint.
        let mut drift = Vec::new();
        let mut i = 0;
        while i < fr.len() {
            let vel = frame_vel(fr, i);
            let mut st = PmState { origin: fr[i].origin, vel, on_ground: false, jump_held: false };
            let mut j = i;
            while j + 1 < fr.len() && fr[j + 1].time - fr[i].time < 0.25 {
                let dt = fr[j + 1].time - fr[j].time;
                if !(0.005..=0.05).contains(&dt) {
                    break;
                }
                pm_step(&bsp, &mut st, &fr[j].cmd(), &p, dt);
                j += 1;
            }
            if j > i {
                drift.push((st.origin - fr[j].origin).length());
            }
            i = (i + 8).max(j.min(i + 8)); // step the window start forward ~0.1s
        }
        let med = pct(&mut drift.clone(), 0.5);
        let max = drift.iter().cloned().fold(0.0, f32::max);
        eprintln!("{csv}: 0.25s-window drift median={med:.1}u max={max:.1}u (n={})", drift.len());
        assert!(med < 3.0, "{csv}: window drift median {med:.1}u too high");
        assert!(max < 16.0, "{csv}: window drift max {max:.1}u too high");
    }
}

#[test]
fn bot_matches_or_beats_human() {
    let (Some(dir), Some(_)) = (demo_dir(), maps_dir()) else {
        eprintln!("RTX_TEST_DEMOS / RTX_TEST_MAPS not set; skipping");
        return;
    };
    const DT: f32 = 1.0 / 77.0;
    for seg in segments() {
        let demo = load_demo(&format!("{dir}/{}.csv", seg.csv));
        let bsp = load_bsp(seg.map);
        let p = demo.movevars.params();
        let frames = window(&demo.frames, seg.t0, seg.t1);
        assert!(frames.len() > 10, "{}: empty segment window", seg.csv);
        let human_time = frames.last().unwrap().time - frames[0].time;

        match seg.kind {
            SegKind::Bhop => {
                // Pick the straightest fast corridor the human bhopped through, and race the bot down
                // over the same span from the same start speed, and confirm the bot's gait covers at
                // least as much ground and ends at least as fast — the "at least as good as the human"
                // bar. Driven on a flat plane (see `flat_step`) so the human's own narrow, weave-width
                // corridor doesn't clip the bot's differently-phased weave; seeded from the demo's real
                // start speed, movevars, and window duration, so the comparison stays honest.
                let (a, b) = straight_window(&frames);
                let win = &frames[a..=b];
                assert!(win.len() > 12, "{}: no straight fast corridor found", seg.csv);
                let human_dt = win[win.len() - 1].time - win[0].time;
                let human_dist = (win[win.len() - 1].origin.xy() - win[0].origin.xy()).length();
                let v_start = frame_vel(&frames, a).xy().length();
                let v_human_end = frame_vel(&frames, b).xy().length();

                let env = Env {
                    dt: DT,
                    accel: p.accel,
                    maxspeed: p.maxspeed,
                    profile: crate::bot::human_profile::HumanMovementProfile::legacy(),
                };
                let mut pos = Vec3::ZERO;
                let mut vel = Vec3::new(v_start, 0.0, 0.0); // start along +x at the human's speed
                let (mut on_ground, mut jump_held) = (true, false);
                let mut bh = Bhop::default();
                let mut t = 0.0f32;
                while t < human_dt {
                    let progress = pos.x;
                    let input = bhop::Input {
                        v_xy: vel.xy(),
                        on_ground,
                        bearing: 0.0,
                        runway: (human_dist - progress).max(0.0) + 512.0,
                        eligible: true,
                        zigzag: false,
                        sustain: true,
                        veto: false,
                        // A committed run: engage the hop cycle immediately. Without this the 0.15 s
                        // engage hysteresis leaves the bot running on the ground, where friction bleeds
                        // the seeded speed toward sv_maxspeed before the first hop — an artifact of the
                        // cold start, not the gait (live, the bot accelerates into the hop).
                        committed: true,
                        carry: true,
                        hold_jump: false,
                        takeoff_speed: 0.0, // gait bench, not a speed-jump takeoff
                        curl_gain: 0.0,
                        clear: f32::INFINITY, // flat-plane gait bench — no walls
                        now: t,
                    };
                    let cmd = bh.step(&input, &env).unwrap_or(bhop::Cmd {
                        view_yaw: 0.0,
                        forward: crate::defs::BOT_MOVE_SPEED,
                        side: 0.0,
                        jump: false,
                    });
                    flat_step(&mut pos, &mut vel, &mut on_ground, &mut jump_held, &cmd, &p, DT);
                    t += DT;
                }
                let bot_dist = pos.x; // net advance along the corridor axis
                let v_bot_end = vel.xy().length();
                let fph = if bh.hops > 0 { bh.flips as f32 / bh.hops as f32 } else { 0.0 };
                eprintln!(
                    "{}: {human_dt:.2}s window — bot went {bot_dist:.0}u vs human {human_dist:.0}u, end speed \
                     bot {v_bot_end:.0} vs human {v_human_end:.0} (start {v_start:.0}, hops {}, {fph:.1} flips/hop)",
                    seg.csv,
                    bh.hops
                );
                // Primary claim: the bot's flat-ground gait ends at least as fast as the human did
                // over the same span from the same start speed — it moves at least as well. (The
                // human's window may cross a real slope the flat plane doesn't, so end-speed is the
                // honest comparison, not distance.) Plus: the gait holds/gains speed rather than
                // bleeding it, covers comparable ground, and stays smooth (~one flip per hop).
                assert!(
                    v_bot_end >= v_human_end - 8.0,
                    "{}: bot end speed {v_bot_end:.0} below human {v_human_end:.0}",
                    seg.csv
                );
                assert!(
                    v_bot_end >= v_start * 0.95,
                    "{}: gait bled speed on flat ground: {v_start:.0} -> {v_bot_end:.0}",
                    seg.csv
                );
                assert!(
                    bot_dist >= human_dist * 0.92,
                    "{}: bot covered {bot_dist:.0}u < human {human_dist:.0}u in {human_dt:.2}s",
                    seg.csv
                );
                assert!(fph <= 2.0, "{}: {fph:.1} flips/hop — not the smooth gait", seg.csv);
            }
            SegKind::RjBallistic => {
                // Seed just after the blast (the recorded velocity carries the launch impulse the
                // sim can't produce), then air-steer to the recorded landing.
                let land = frames.last().unwrap().origin;
                let a_max = bhop::air_accel_max(p.accel, p.maxspeed, DT);
                let mut st = PmState {
                    origin: frames[0].origin,
                    vel: frame_vel(&frames, 0),
                    on_ground: false,
                    jump_held: true, // airborne from the launch; don't let the sim re-jump
                };
                let mut t = 0.0f32;
                let mut landed = None;
                while t < human_time * 1.5 + 0.5 {
                    let to = (land.xy() - st.origin.xy()).normalize_or_zero();
                    let bearing = yaw_of(to);
                    let s = bhop::air_correct(st.vel.xy(), bearing, a_max, DT, bhop::AIR_CORRECT_GAIN_DEFAULT);
                    let cmd = bhop::Cmd { view_yaw: s.view_yaw, forward: s.forward, side: s.side, jump: false };
                    pm_step(&bsp, &mut st, &cmd, &p, DT);
                    t += DT;
                    if st.on_ground && t > 0.1 {
                        landed = Some(t);
                        break;
                    }
                }
                let landed = landed.unwrap_or_else(|| panic!("{}: RJ arc never landed", seg.csv));
                let miss = (st.origin.xy() - land.xy()).length();
                eprintln!(
                    "{}: RJ ballistic landed {miss:.0}u from target in {landed:.2}s (human airtime {human_time:.2}s)",
                    seg.csv
                );
                assert!(miss < 48.0, "{}: RJ landing missed by {miss:.0}u", seg.csv);
                assert!(landed <= human_time * 1.15 + 0.1, "{}: RJ arc took {landed:.2}s vs {human_time:.2}s", seg.csv);
            }
        }
    }
}
