// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-mcp` — the rtx bot control and tuning harness bridge.
//!
//! An MCP (stdio) server that lets Claude Code drive the tuning loop directly: it manages a local
//! `mvdsv` process (playground/), connects to the rtx game module's TCP control channel
//! (`rtx_control_port`, see `crates/rtx-game/src/control.rs`), and exposes tools to enumerate a map's
//! rocket-jump links, puppet a bot through each (teleport/goto → fire the jump), read back the
//! per-attempt telemetry, and turn the `rtx_rj_*` knobs — all without hand-flying bots in a server.
//!
//! ## Wiring
//! Claude Code ──MCP stdio──▶ this bin ──TCP 127.0.0.1:port──▶ control.rs in librtx.dylib
//!                              └─ spawns/kills playground/mvdsv (+exec rjtest.cfg)
//!
//! The control protocol is line-based: we send `<id> <verb> args…` and demux replies (`{"id":…}`)
//! from unsolicited events (`{"ev":…}`) on the one inbound stream — a reply resolves its request's
//! `oneshot`; an event fans out on a `broadcast` that the goto/rocket_jump tools await.

use std::collections::{HashMap, VecDeque};
use std::io::BufRead;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, ServerCapabilities, ServerInfo};
use rmcp::transport::stdio;
use rmcp::{schemars, tool, tool_handler, tool_router, ErrorData as McpError, ServerHandler, ServiceExt};
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader as TokioBufReader};
use tokio::net::tcp::OwnedWriteHalf;
use tokio::net::TcpStream;
use tokio::sync::{broadcast, oneshot, Mutex as TokioMutex};

/// Default control port (matches the harness config). Overridable per `server_start`.
const DEFAULT_PORT: u16 = 27950;
/// Timeout for a plain request/reply (status, prep, teleport, set, …) — these reply within a frame.
const SHORT: Duration = Duration::from_secs(5);

/// The repo root, resolved at compile time from this crate's manifest dir (`<repo>/crates/rtx-mcp`),
/// so the bridge finds `playground/` regardless of the working directory it's launched from.
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
}

// --- control-channel client -------------------------------------------------------------------

/// A live connection to the game module's control channel: the write half (guarded so tool calls can
/// share it), the pending-reply map keyed by request id, and a broadcast of unsolicited events.
struct ControlConn {
    writer: TokioMutex<OwnedWriteHalf>,
    pending: Arc<StdMutex<HashMap<i64, oneshot::Sender<Value>>>>,
    events: broadcast::Sender<Value>,
    next_id: AtomicI64,
}

impl ControlConn {
    /// Connect to the local control port and start the reader task that demuxes replies vs events.
    async fn connect(port: u16) -> std::io::Result<Arc<Self>> {
        let stream = TcpStream::connect(("127.0.0.1", port)).await?;
        let _ = stream.set_nodelay(true);
        let (read, write) = stream.into_split();
        let pending: Arc<StdMutex<HashMap<i64, oneshot::Sender<Value>>>> = Arc::new(StdMutex::new(HashMap::new()));
        let (events, _) = broadcast::channel(256);
        let conn = Arc::new(ControlConn {
            writer: TokioMutex::new(write),
            pending: pending.clone(),
            events: events.clone(),
            next_id: AtomicI64::new(1),
        });
        tokio::spawn(async move {
            let mut lines = TokioBufReader::new(read).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                let Ok(v) = serde_json::from_str::<Value>(&line) else { continue };
                if let Some(id) = v.get("id").and_then(Value::as_i64) {
                    if let Some(tx) = pending.lock().unwrap().remove(&id) {
                        let _ = tx.send(v);
                    }
                } else if v.get("ev").is_some() {
                    let _ = events.send(v); // ignore "no subscribers"
                }
            }
        });
        Ok(conn)
    }

    /// Send `<id> verb` and await its id-tagged reply. Returns the reply's `data` on `ok`, else the
    /// reported error string.
    async fn request(&self, verb: &str, timeout: Duration) -> Result<Value, String> {
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);
        let (tx, rx) = oneshot::channel();
        self.pending.lock().unwrap().insert(id, tx);
        {
            let mut w = self.writer.lock().await;
            w.write_all(format!("{id} {verb}\n").as_bytes())
                .await
                .map_err(|e| e.to_string())?;
            w.flush().await.map_err(|e| e.to_string())?;
        }
        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(reply)) => {
                if reply.get("ok").and_then(Value::as_bool) == Some(true) {
                    Ok(reply.get("data").cloned().unwrap_or(Value::Null))
                } else {
                    Err(reply
                        .get("error")
                        .and_then(Value::as_str)
                        .unwrap_or("unknown error")
                        .to_string())
                }
            }
            _ => {
                self.pending.lock().unwrap().remove(&id);
                Err(format!("timeout waiting for reply to '{verb}'"))
            }
        }
    }

    /// Await the first event matching `pred` on `rx` (subscribed *before* the triggering command was
    /// sent, so nothing is missed), or time out.
    async fn await_event(
        &self,
        mut rx: broadcast::Receiver<Value>,
        pred: impl Fn(&Value) -> bool,
        timeout: Duration,
    ) -> Result<Value, String> {
        let fut = async {
            loop {
                match rx.recv().await {
                    Ok(v) => {
                        if pred(&v) {
                            return Ok(v);
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return Err("event stream closed".to_string()),
                }
            }
        };
        tokio::time::timeout(timeout, fut)
            .await
            .map_err(|_| "timeout waiting for event".to_string())?
    }
}

// --- shared server state ----------------------------------------------------------------------

struct Inner {
    repo: PathBuf,
    proc: StdMutex<Option<Child>>,
    log: Arc<StdMutex<VecDeque<String>>>,
    conn: TokioMutex<Option<Arc<ControlConn>>>,
    /// Cached `list_rj_links` result (invalidated on restart / any `map` command). Link ids aren't
    /// stable across map builds, so callers must re-list after a map change.
    links: StdMutex<Option<Vec<Value>>>,
}

#[derive(Clone)]
struct RtxMcp {
    inner: Arc<Inner>,
    // Read by the `#[tool_handler]`/`#[tool_router]` macros, which the dead-code pass can't see.
    #[allow(dead_code)]
    tool_router: ToolRouter<Self>,
}

// --- helpers (non-tool) -----------------------------------------------------------------------

impl RtxMcp {
    fn new() -> Self {
        RtxMcp {
            inner: Arc::new(Inner {
                repo: repo_root(),
                proc: StdMutex::new(None),
                log: Arc::new(StdMutex::new(VecDeque::new())),
                conn: TokioMutex::new(None),
                links: StdMutex::new(None),
            }),
            tool_router: Self::tool_router(),
        }
    }

    async fn conn(&self) -> Result<Arc<ControlConn>, String> {
        self.inner
            .conn
            .lock()
            .await
            .clone()
            .ok_or_else(|| "not connected — call server_start or server_connect first".to_string())
    }

    async fn req(&self, verb: &str, timeout: Duration) -> Result<Value, String> {
        self.conn().await?.request(verb, timeout).await
    }

    /// Resolve the target bot: the given `ent`, or the first live bot reported by `status`.
    async fn resolve_bot(&self, bot: Option<u32>) -> Result<u32, String> {
        if let Some(b) = bot {
            return Ok(b);
        }
        let st = self.req("status", SHORT).await?;
        st.get("bots")
            .and_then(Value::as_array)
            .and_then(|a| a.first())
            .and_then(|b| b.get("ent"))
            .and_then(Value::as_u64)
            .map(|x| x as u32)
            .ok_or_else(|| "no bot present (is the server up with a bot spawned?)".to_string())
    }

    /// The map's rocket-jump links (cached until the next restart / `map`).
    async fn links(&self) -> Result<Vec<Value>, String> {
        if let Some(cached) = self.inner.links.lock().unwrap().clone() {
            return Ok(cached);
        }
        let data = self.req("links", SHORT).await?;
        let arr = data
            .get("links")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        *self.inner.links.lock().unwrap() = Some(arr.clone());
        Ok(arr)
    }

    async fn op_stop(&self) -> Result<Value, String> {
        *self.inner.conn.lock().await = None; // drop the connection (its reader task ends on close)
        let killed = if let Some(mut child) = self.inner.proc.lock().unwrap().take() {
            let _ = child.kill();
            let _ = child.wait();
            true
        } else {
            false
        };
        Ok(json!({ "stopped": killed }))
    }

    async fn op_start(&self, map: String, port: u16, skill: f32) -> Result<Value, String> {
        let _ = self.op_stop().await; // idempotent: replace any running server
        write_config(&self.inner.repo, &map, port, skill)?;
        let bin = self.inner.repo.join("playground/mvdsv");
        let mut child = Command::new(&bin)
            .arg("+exec")
            .arg("rjtest.cfg")
            .current_dir(self.inner.repo.join("playground"))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("spawn {}: {e}", bin.display()))?;
        if let Some(out) = child.stdout.take() {
            spawn_drain(out, self.inner.log.clone());
        }
        if let Some(err) = child.stderr.take() {
            spawn_drain(err, self.inner.log.clone());
        }
        *self.inner.proc.lock().unwrap() = Some(child);
        self.inner.links.lock().unwrap().take();

        // Connect (the module binds the listener a few frames after the dylib loads).
        let deadline = Instant::now() + Duration::from_secs(20);
        let conn = loop {
            match ControlConn::connect(port).await {
                Ok(c) => break c,
                Err(e) => {
                    if Instant::now() >= deadline {
                        return Err(format!(
                            "could not connect to 127.0.0.1:{port}: {e}\n{}",
                            self.tail_log(40)
                        ));
                    }
                    tokio::time::sleep(Duration::from_millis(300)).await;
                }
            }
        };
        *self.inner.conn.lock().await = Some(conn);

        // Wait for the navmesh to finish building and a bot to spawn.
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            if let Ok(st) = self.req("status", SHORT).await {
                let ready = st.get("navmesh").and_then(Value::as_str) == Some("ready");
                let has_bot = st.get("bots").and_then(Value::as_array).is_some_and(|a| !a.is_empty());
                if ready && has_bot {
                    // Park the harness bot before returning control to the caller. On a race/test map
                    // an autonomous bot can otherwise reach trigger_changelevel while the caller is
                    // inspecting the first status response, leaving every subsequent trial frozen in
                    // intermission. `hold` also clears the bot's live route/bhop momentum server-side.
                    let bot = st["bots"][0]["ent"]
                        .as_u64()
                        .ok_or_else(|| "ready status had no bot ent".to_string())? as u32;
                    self.req(&format!("hold {bot}"), SHORT).await?;
                    return self.req("status", SHORT).await;
                }
            }
            if Instant::now() >= deadline {
                return Err(format!(
                    "server up but navmesh/bot not ready within 60s\n{}",
                    self.tail_log(40)
                ));
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
    }

    /// Attach to a server that is already running the rtx control channel. Unlike `server_start`,
    /// this never writes a config or owns/kills the mvdsv process; it is the safe entry point for a
    /// long-lived match server shared with a client or another development session.
    async fn op_connect(&self, port: u16) -> Result<Value, String> {
        *self.inner.conn.lock().await = None;
        self.inner.links.lock().unwrap().take();
        let conn = ControlConn::connect(port)
            .await
            .map_err(|e| format!("could not connect to 127.0.0.1:{port}: {e}"))?;
        *self.inner.conn.lock().await = Some(conn);
        self.req("status", SHORT).await
    }

    fn tail_log(&self, n: usize) -> String {
        let l = self.inner.log.lock().unwrap();
        let start = l.len().saturating_sub(n);
        l.iter().skip(start).cloned().collect::<Vec<_>>().join("\n")
    }

    /// Place the bot at a rocket-jump link's source, then fly the link, returning goto (when routed)
    /// and rj telemetry. A goto that stalls reports the source inaccessible without attempting the jump.
    async fn op_test_link(&self, link: u32, via: &str, bot: Option<u32>) -> Result<Value, String> {
        let bot = self.resolve_bot(bot).await?;
        let links = self.links().await?;
        let entry = links
            .iter()
            .find(|l| l.get("link").and_then(Value::as_u64) == Some(link as u64))
            .ok_or_else(|| format!("link {link} is not a rocket-jump link on this map"))?;
        let src = vec3_of(entry.get("src").unwrap_or(&Value::Null))?;
        let mut out = json!({ "link": link, "bot": bot });

        self.req(&format!("prep {bot}"), SHORT).await?;

        if via == "goto" {
            let conn = self.conn().await?;
            let rx = conn.events.subscribe();
            self.req(&format!("goto {bot} {} {} {}", src[0], src[1], src[2]), SHORT).await?;
            let ev = conn
                .await_event(
                    rx,
                    |v| is_ev(v, "arrived", bot) || is_ev(v, "goto_stall", bot),
                    Duration::from_secs(30),
                )
                .await?;
            let stalled = ev.get("ev").and_then(Value::as_str) == Some("goto_stall");
            out["goto"] = ev;
            if stalled {
                out["rj"] = Value::Null;
                out["source_inaccessible"] = json!(true);
                return Ok(out);
            }
        } else {
            self.req(&format!("teleport {bot} {} {} {}", src[0], src[1], src[2]), SHORT).await?;
        }

        let conn = self.conn().await?;
        let rx = conn.events.subscribe();
        self.req(&format!("rj {bot} {link}"), SHORT).await?;
        let ev = conn
            .await_event(rx, |v| is_ev(v, "rj_result", bot), Duration::from_secs(15))
            .await?;
        out["rj"] = ev;
        Ok(out)
    }

    async fn op_test_links(&self, links: Option<Vec<u32>>, via: &str) -> Result<Value, String> {
        let all = self.links().await?;
        let targets: Vec<u32> = match links {
            Some(list) if !list.is_empty() => list,
            _ => all
                .iter()
                .filter_map(|l| l.get("link").and_then(Value::as_u64).map(|x| x as u32))
                .collect(),
        };
        let mut results = Vec::new();
        let mut counts: HashMap<String, u32> = HashMap::new();
        let (mut miss_sum, mut miss_n) = (0.0_f64, 0u32);
        for link in targets {
            match self.op_test_link(link, via, None).await {
                Ok(v) => {
                    let outcome = if v.get("source_inaccessible") == Some(&json!(true)) {
                        "unreachable_src".to_string()
                    } else {
                        v["rj"]["outcome"].as_str().unwrap_or("error").to_string()
                    };
                    *counts.entry(outcome).or_insert(0) += 1;
                    if let Some(m) = v["rj"]["land"]["miss_xy"].as_f64() {
                        miss_sum += m;
                        miss_n += 1;
                    }
                    results.push(v);
                }
                Err(e) => {
                    *counts.entry("error".to_string()).or_insert(0) += 1;
                    results.push(json!({ "link": link, "error": e }));
                }
            }
        }
        let mean_miss_xy = if miss_n > 0 { miss_sum / miss_n as f64 } else { 0.0 };
        Ok(json!({
            "summary": { "total": results.len(), "counts": counts, "mean_miss_xy": mean_miss_xy, "landed_measured": miss_n },
            "results": results,
        }))
    }
}

/// Every `rtx_rj_*` knob, in wire order — shared by set_knobs / get_knobs.
const KNOBS: &[&str] = &[
    "rtx_rj_stance",
    "rtx_rj_aim_tol",
    "rtx_rj_stance_timeout",
    "rtx_rj_liftoff_timeout",
    "rtx_rj_ballistic_slack",
    "rtx_rj_delay_bias",
    "rtx_rj_pitch_bias",
];

fn is_ev(v: &Value, name: &str, bot: u32) -> bool {
    v.get("ev").and_then(Value::as_str) == Some(name) && v.get("bot").and_then(Value::as_u64) == Some(bot as u64)
}

fn vec3_of(v: &Value) -> Result<[f32; 3], String> {
    let a = v.as_array().ok_or("expected a [x,y,z] array")?;
    if a.len() != 3 {
        return Err("vec3 needs exactly 3 numbers".to_string());
    }
    Ok([
        a[0].as_f64().unwrap_or(0.0) as f32,
        a[1].as_f64().unwrap_or(0.0) as f32,
        a[2].as_f64().unwrap_or(0.0) as f32,
    ])
}

/// Reduce a control-channel goto trajectory to the invariants a directed-corridor run cares about.
/// Rows are `[t,x,y,z,vx,vy,vz]`; low-speed frames are excluded from heading metrics so the initial
/// acceleration tick cannot manufacture an arbitrary yaw.
fn corridor_metrics(ev: &Value, start: [f32; 3], end: [f32; 3]) -> Result<Value, String> {
    let traj = ev.get("traj").and_then(Value::as_array).ok_or("goto event had no trajectory")?;
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let len = dx.hypot(dy);
    if len < 1.0 {
        return Err("corridor start and end need distinct XY positions".to_string());
    }
    let (fx, fy) = (dx / len, dy / len);
    let (rx, ry) = (-fy, fx);
    let mut first_t = None::<f32>;
    let mut last_t = None::<f32>;
    let mut peak = 0.0f32;
    let mut max_cross = 0.0f32;
    let mut max_heading = 0.0f32;
    let mut max_yaw_step = 0.0f32;
    let mut reverse_frames = 0u32;
    let mut max_z = start[2];
    let mut peak_progress = 0.0f32;
    let mut progress_speeds = [0.0f32; 21];
    let mut prev_heading = None::<(f32, f32)>;
    for row in traj {
        let a = row.as_array().ok_or("trajectory row was not an array")?;
        if a.len() != 7 {
            return Err("trajectory row did not have 7 values".to_string());
        }
        let n = |i: usize| a[i].as_f64().unwrap_or(0.0) as f32;
        let (t, x, y, z, vx, vy) = (n(0), n(1), n(2), n(3), n(4), n(5));
        first_t.get_or_insert(t);
        last_t = Some(t);
        max_z = max_z.max(z);
        max_cross = max_cross.max(((x - start[0]) * rx + (y - start[1]) * ry).abs());
        let speed = vx.hypot(vy);
        let progress = ((x - start[0]) * fx + (y - start[1]) * fy) / len;
        let progress_bin = (progress.clamp(0.0, 1.0) * 20.0).floor() as usize;
        progress_speeds[progress_bin] = progress_speeds[progress_bin].max(speed);
        if speed > peak {
            peak = speed;
            peak_progress = progress;
        }
        if speed >= 100.0 {
            let heading = (vx / speed, vy / speed);
            let forward = vx * fx + vy * fy;
            max_heading = max_heading.max((heading.0 * fx + heading.1 * fy).clamp(-1.0, 1.0).acos().to_degrees());
            if forward < 0.0 {
                reverse_frames += 1;
            }
            if let Some(prev) = prev_heading {
                max_yaw_step = max_yaw_step
                    .max((prev.0 * heading.0 + prev.1 * heading.1).clamp(-1.0, 1.0).acos().to_degrees());
            }
            prev_heading = Some(heading);
        }
    }
    let elapsed = last_t.zip(first_t).map_or(0.0, |(last, first)| last - first);
    Ok(json!({
        "event": ev.get("ev").and_then(Value::as_str).unwrap_or("unknown"),
        "elapsed": elapsed,
        "samples": traj.len(),
        "peak_speed": peak,
        "peak_progress": peak_progress,
        "progress_speeds": progress_speeds,
        "max_cross_track": max_cross,
        "max_heading_error": max_heading,
        "max_yaw_step": max_yaw_step,
        "reverse_frames": reverse_frames,
        "hopped": max_z > start[2] + 10.0,
        "arrival": ev.get("origin").cloned().unwrap_or(Value::Null),
        "distance": ev.get("dist").cloned().unwrap_or(Value::Null),
    }))
}

/// Write the self-contained harness config into `playground/qw/rjtest.cfg`. Self-contained (server
/// cvars + rtx cvars + `map`) so it works whether or not mvdsv auto-execs `server.cfg` first — the
/// last `set`/`map` wins either way. One bot, alone, pacifist, the control port open, dm/no-match so
/// weapons are always hot. Movement cvars mirror the playground config so the RJ links match.
fn write_config(repo: &std::path::Path, map: &str, port: u16, skill: f32) -> Result<(), String> {
    // Every cvar that gates a navmesh *build input* (rocket-jump / bhop-speed-jump / double-jump /
    // hook link generation) is set explicitly here, before `map`. This is load-bearing: those cvars
    // are otherwise seeded by the module's GAME_INIT `cvar_default`, whose queued `set` flushes only
    // *after* the first-frame navmesh build reads them — so on a fresh boot the first map builds with
    // rocket jumps gated OFF (rjump 0). Setting them here means the first build already sees them.
    let cfg = format!(
        "// generated by rtx-mcp — the rtx bot control and tuning harness\n\
         sv_progtype 1\n\
         deathmatch 1\n\
         timelimit 0\n\
         fraglimit 0\n\
         maxclients 8\n\
         maxspectators 4\n\
         set rtx_mode dm\n\
         set rtx_match \"\"\n\
         set rtx_grapple 0\n\
         set rtx_doublejump 0\n\
         set rtx_bot_bhop 1\n\
         set rtx_bot_rocketjump 1\n\
         set rtx_bot_count 1\n\
         set rtx_bot_alone 1\n\
         set rtx_bot_pacifist 1\n\
         set rtx_bot_skill {skill}\n\
         set rtx_control_port {port}\n\
         set developer 1\n\
         map {map}\n"
    );
    let path = repo.join("playground/qw/rjtest.cfg");
    std::fs::write(&path, cfg).map_err(|e| format!("write {}: {e}", path.display()))
}

/// Drain a child stdout/stderr into the shared ring buffer (last ~500 lines), on its own thread.
fn spawn_drain<R: std::io::Read + Send + 'static>(r: R, log: Arc<StdMutex<VecDeque<String>>>) {
    std::thread::spawn(move || {
        for line in std::io::BufReader::new(r).lines().map_while(Result::ok) {
            let mut l = log.lock().unwrap();
            if l.len() >= 500 {
                l.pop_front();
            }
            l.push_back(line);
        }
    });
}

/// Wrap an operation result as an MCP tool result: success carries the JSON verbatim; an expected
/// failure (no server, timeout, bad link) is a non-protocol `isError` result, not an `Err`, so the
/// caller sees the message instead of a transport fault.
fn finish(r: Result<Value, String>) -> Result<CallToolResult, McpError> {
    match r {
        Ok(v) => Ok(CallToolResult::success(vec![ContentBlock::text(v.to_string())])),
        Err(e) => Ok(CallToolResult::error(vec![ContentBlock::text(json!({ "error": e }).to_string())])),
    }
}

// --- tool argument schemas --------------------------------------------------------------------

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct StartArgs {
    /// Map to load (default "aerowalk"). The bsp must exist under playground/qw/maps/.
    map: Option<String>,
    /// Control port to open (default 27950).
    port: Option<u16>,
    /// Bot skill 0–7 (default 7). Drives the aim-spring stiffness, itself a tuning variable.
    skill: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct ConnectArgs {
    /// Existing rtx control port (default 27950).
    port: Option<u16>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct LogArgs {
    /// How many trailing log lines to return (default 50).
    lines: Option<usize>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct PrepArgs {
    /// Bot ent id (default: the first live bot).
    bot: Option<u32>,
    /// Health to set (default 100).
    health: Option<f32>,
    /// Rockets to load (default 10).
    rockets: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct TeleportArgs {
    bot: Option<u32>,
    /// A rocket-jump link id — teleports to its source cell (overrides x/y/z when given).
    link: Option<u32>,
    x: Option<f32>,
    y: Option<f32>,
    z: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct GotoArgs {
    bot: Option<u32>,
    x: f32,
    y: f32,
    z: f32,
    /// Seconds to await arrival/stall (default 30).
    timeout: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct CorridorArgs {
    bot: Option<u32>,
    start_x: f32,
    start_y: f32,
    start_z: f32,
    end_x: f32,
    end_y: f32,
    end_z: f32,
    /// Number of fresh teleport-and-run trials (default 3, maximum 20).
    trials: Option<u32>,
    /// Largest allowed perpendicular displacement from the directed centerline (default 64u).
    max_cross_track: Option<f32>,
    /// Largest allowed velocity-heading error from the directed path (default 60 degrees).
    max_heading_error: Option<f32>,
    /// Smallest acceptable peak horizontal speed (default 500 ups).
    min_peak_speed: Option<f32>,
    /// Seconds to await each arrival/stall (default 30).
    timeout: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct RjArgs {
    bot: Option<u32>,
    /// The rocket-jump link id to fly.
    link: u32,
    /// Seconds to await the result (default 15).
    timeout: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct TestLinkArgs {
    /// The rocket-jump link id to test.
    link: u32,
    /// How to place the bot at the source: "teleport" (default) or "goto" (walk there, reporting an
    /// inaccessible source as a stall).
    via: Option<String>,
    bot: Option<u32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct TestLinksArgs {
    /// Specific link ids to test; omit to sweep every rocket-jump link on the map.
    links: Option<Vec<u32>>,
    via: Option<String>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct SetKnobsArgs {
    stance: Option<f32>,
    aim_tol: Option<f32>,
    stance_timeout: Option<f32>,
    liftoff_timeout: Option<f32>,
    ballistic_slack: Option<f32>,
    delay_bias: Option<f32>,
    pitch_bias: Option<f32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct ConsoleArgs {
    /// A raw server console command (e.g. "map bravado"). A command containing "map" invalidates the
    /// cached link list.
    command: String,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct CvarArgs {
    /// Cvar name. The game-side control protocol restricts this to letters, digits, and underscore.
    name: String,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct SetCvarArgs {
    /// Cvar name. The game-side control protocol validates it before applying the value.
    name: String,
    /// Exact string value to assign.
    value: String,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct SetCvarsArgs {
    /// Ordered cvar assignments. Each entry is validated independently by the game-side control
    /// protocol, and all entries are attempted even if one fails.
    cvars: Vec<SetCvarArgs>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct BotArgs {
    /// Bot entity id (default: the first live bot).
    bot: Option<u32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct CellArgs {
    x: f32,
    y: f32,
    z: f32,
}

// --- tools ------------------------------------------------------------------------------------

#[tool_router]
impl RtxMcp {
    #[tool(description = "Attach to an already-running mvdsv rtx control port without starting, \
        reconfiguring, or taking ownership of the server.")]
    async fn server_connect(&self, Parameters(a): Parameters<ConnectArgs>) -> Result<CallToolResult, McpError> {
        finish(self.op_connect(a.port.unwrap_or(DEFAULT_PORT)).await)
    }

    #[tool(description = "Launch mvdsv with the harness config (1 bot, control port open), wait until \
        the navmesh is built and a bot has spawned, and return the server status.")]
    async fn server_start(&self, Parameters(a): Parameters<StartArgs>) -> Result<CallToolResult, McpError> {
        let map = a.map.unwrap_or_else(|| "aerowalk".to_string());
        let port = a.port.unwrap_or(DEFAULT_PORT);
        let skill = a.skill.unwrap_or(7.0);
        finish(self.op_start(map, port, skill).await)
    }

    #[tool(description = "Stop the managed mvdsv process and close the control connection.")]
    async fn server_stop(&self) -> Result<CallToolResult, McpError> {
        finish(self.op_stop().await)
    }

    #[tool(description = "Restart the server (fresh navmesh — link ids are NOT stable across restarts).")]
    async fn server_restart(&self, Parameters(a): Parameters<StartArgs>) -> Result<CallToolResult, McpError> {
        let map = a.map.unwrap_or_else(|| "aerowalk".to_string());
        let port = a.port.unwrap_or(DEFAULT_PORT);
        let skill = a.skill.unwrap_or(7.0);
        finish(self.op_start(map, port, skill).await)
    }

    #[tool(description = "Server and strategy status: map/navmesh, match format/phase/scores/roster, \
        and each bot's team, stack, inventory, posture, perceived enemy, item plan, and route head.")]
    async fn status(&self) -> Result<CallToolResult, McpError> {
        finish(self.req("status", SHORT).await)
    }

    #[tool(description = "Read a live server cvar as both its exact string and numeric value.")]
    async fn get_cvar(&self, Parameters(a): Parameters<CvarArgs>) -> Result<CallToolResult, McpError> {
        finish(self.req(&format!("get {}", a.name), SHORT).await)
    }

    #[tool(description = "Set a live server cvar through the validated control protocol.")]
    async fn set_cvar(&self, Parameters(a): Parameters<SetCvarArgs>) -> Result<CallToolResult, McpError> {
        finish(self.req(&format!("set {} {}", a.name, a.value), SHORT).await)
    }

    #[tool(description = "Set an ordered list of live server cvars in one tool call. Every pair is \
        attempted and the result or error for each assignment is returned in input order.")]
    async fn set_cvars(&self, Parameters(a): Parameters<SetCvarsArgs>) -> Result<CallToolResult, McpError> {
        let mut results = Vec::with_capacity(a.cvars.len());
        let mut succeeded = 0usize;
        for cvar in a.cvars {
            match self.req(&format!("set {} {}", cvar.name, cvar.value), SHORT).await {
                Ok(result) => {
                    succeeded += 1;
                    results.push(json!({
                        "name": cvar.name,
                        "value": cvar.value,
                        "ok": true,
                        "result": result,
                    }));
                }
                Err(error) => results.push(json!({
                    "name": cvar.name,
                    "value": cvar.value,
                    "ok": false,
                    "error": error,
                })),
            }
        }
        finish(Ok(json!({
            "ok": succeeded == results.len(),
            "succeeded": succeeded,
            "failed": results.len() - succeeded,
            "results": results,
        })))
    }

    #[tool(description = "Lock the current structured team roster, reload the map, run the countdown, \
        and return once the match is live with its navmesh and bots ready. Fails if the requested \
        format does not have enough players.")]
    async fn match_start(&self) -> Result<CallToolResult, McpError> {
        let r = async {
            self.inner.links.lock().unwrap().take();
            self.req("match_start", SHORT).await?;
            let started = Instant::now();
            let mut last_start_attempt = started;
            let mut start_attempts = 1u32;
            let deadline = started + Duration::from_secs(90);
            let mut last = Value::Null;
            loop {
                if let Ok(st) = self.req("status", SHORT).await {
                    let phase = st.pointer("/match/phase").and_then(Value::as_str);
                    let nav_ready = st.get("navmesh").and_then(Value::as_str) == Some("ready");
                    let roster_len = st.pointer("/match/roster").and_then(Value::as_array)
                        .map_or(0, Vec::len);
                    let bots_len = st.get("bots").and_then(Value::as_array).map_or(0, Vec::len);
                    if phase == Some("live") && nav_ready && roster_len > 0 && bots_len >= roster_len {
                        return Ok(st);
                    }
                    // A request made on the exact result-pause/warmup boundary can be acknowledged by
                    // the control socket before the lifecycle is startable. Retry only while a settled
                    // server still reports warmup; an accepted start immediately enters reload/countdown.
                    if phase == Some("warmup") && last_start_attempt.elapsed() >= Duration::from_secs(1) {
                        self.req("match_start", SHORT).await?;
                        last_start_attempt = Instant::now();
                        start_attempts += 1;
                    }
                    if phase == Some("warmup") && started.elapsed() > Duration::from_secs(30) {
                        return Err(format!(
                            "match stayed in warmup after {start_attempts} start attempts (is the roster full?): {st}"
                        ));
                    }
                    last = st;
                }
                if Instant::now() >= deadline {
                    return Err(format!("match did not become ready within 90s; last status: {last}"));
                }
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
        .await;
        finish(r)
    }

    #[tool(description = "Inspect the navmesh cell nearest a world point, including incoming and \
        outgoing link kinds, costs, and hazards.")]
    async fn inspect_cell(&self, Parameters(a): Parameters<CellArgs>) -> Result<CallToolResult, McpError> {
        finish(self.req(&format!("cell {} {} {}", a.x, a.y, a.z), SHORT).await)
    }

    #[tool(description = "Dump a live bot's complete planned route as navmesh link ids, kinds, and \
        source/target world positions.")]
    async fn bot_route(&self, Parameters(a): Parameters<BotArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            self.req(&format!("route {bot}"), SHORT).await
        }
        .await;
        finish(r)
    }

    #[tool(description = "List generated curl-jump links (speed-jump links with a certified curl \
        gain), including run-up, takeoff, target, required speed, and gain.")]
    async fn list_curl_links(&self) -> Result<CallToolResult, McpError> {
        finish(self.req("curls", SHORT).await)
    }

    #[tool(description = "Tail the managed server's console output.")]
    async fn server_log(&self, Parameters(a): Parameters<LogArgs>) -> Result<CallToolResult, McpError> {
        let n = a.lines.unwrap_or(50);
        finish(Ok(json!({ "log": self.tail_log(n) })))
    }

    #[tool(description = "List every rocket-jump link the navmesh generated: id, source/target world \
        positions, and the solved fire pitch/yaw, fire delay, airtime, and self-damage.")]
    async fn list_rj_links(&self) -> Result<CallToolResult, McpError> {
        finish(self.links().await.map(|l| json!({ "count": l.len(), "links": l })))
    }

    #[tool(description = "Make a bot fit to rocket-jump: set health, give the rocket launcher with \
        rockets, select it, clear quad, and take it off cooldown.")]
    async fn prep(&self, Parameters(a): Parameters<PrepArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            let h = a.health.unwrap_or(100.0);
            let rk = a.rockets.unwrap_or(10.0);
            self.req(&format!("prep {bot} {h} {rk}"), SHORT).await
        }
        .await;
        finish(r)
    }

    #[tool(description = "Teleport a bot to a world position (or, with `link`, to that rocket-jump \
        link's source cell), zeroing momentum and resetting its navigation state.")]
    async fn teleport(&self, Parameters(a): Parameters<TeleportArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            let pos = if let Some(link) = a.link {
                let links = self.links().await?;
                let entry = links
                    .iter()
                    .find(|l| l.get("link").and_then(Value::as_u64) == Some(link as u64))
                    .ok_or_else(|| format!("link {link} is not a rocket-jump link"))?;
                vec3_of(entry.get("src").unwrap_or(&Value::Null))?
            } else {
                [a.x.unwrap_or(0.0), a.y.unwrap_or(0.0), a.z.unwrap_or(0.0)]
            };
            self.req(&format!("teleport {bot} {} {} {}", pos[0], pos[1], pos[2]), SHORT).await
        }
        .await;
        finish(r)
    }

    #[tool(description = "Order a bot to walk to a world position, awaiting an `arrived` or (if it \
        makes no progress for ~4s) `goto_stall` event — the inaccessible-source signal.")]
    async fn goto(&self, Parameters(a): Parameters<GotoArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            let timeout = Duration::from_secs_f32(a.timeout.unwrap_or(30.0));
            let conn = self.conn().await?;
            let rx = conn.events.subscribe();
            self.req(&format!("goto {bot} {} {} {}", a.x, a.y, a.z), SHORT).await?;
            conn.await_event(rx, |v| is_ev(v, "arrived", bot) || is_ev(v, "goto_stall", bot), timeout)
                .await
        }
        .await;
        finish(r)
    }

    #[tool(description = "Repeatedly run the normal bot pathfinder/bhop controller down one directed \
        corridor. Each trial teleports to the same start and reports elapsed time, peak speed, maximum \
        cross-track drift, heading error, per-frame yaw step, reverse frames, and whether it hopped. \
        The aggregate passes only when every trial arrives quickly without leaving the requested \
        movement envelope; this is a path-following test, not a benchmark-only movement mode.")]
    async fn corridor_test(&self, Parameters(a): Parameters<CorridorArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            let start = [a.start_x, a.start_y, a.start_z];
            let end = [a.end_x, a.end_y, a.end_z];
            let trials = a.trials.unwrap_or(3).clamp(1, 20);
            let cross_limit = a.max_cross_track.unwrap_or(64.0).max(0.0);
            let heading_limit = a.max_heading_error.unwrap_or(60.0).clamp(0.0, 180.0);
            let peak_floor = a.min_peak_speed.unwrap_or(500.0).max(0.0);
            let timeout = Duration::from_secs_f32(a.timeout.unwrap_or(30.0).clamp(1.0, 120.0));
            let conn = self.conn().await?;
            let mut results = Vec::with_capacity(trials as usize);
            let mut all_passed = true;
            for trial in 1..=trials {
                self.req(
                    &format!("teleport {bot} {} {} {}", start[0], start[1], start[2]),
                    SHORT,
                )
                .await?;
                let rx = conn.events.subscribe();
                self.req(&format!("goto {bot} {} {} {}", end[0], end[1], end[2]), SHORT)
                    .await?;
                let ev = conn
                    .await_event(
                        rx,
                        |v| is_ev(v, "arrived", bot) || is_ev(v, "goto_stall", bot),
                        timeout,
                    )
                    .await?;
                let mut m = corridor_metrics(&ev, start, end)?;
                let passed = m["event"] == "arrived"
                    && m["hopped"].as_bool() == Some(true)
                    && m["reverse_frames"].as_u64() == Some(0)
                    && m["max_cross_track"].as_f64().unwrap_or(f64::INFINITY) <= cross_limit as f64
                    && m["max_heading_error"].as_f64().unwrap_or(f64::INFINITY) <= heading_limit as f64
                    && m["max_yaw_step"].as_f64().unwrap_or(f64::INFINITY) <= 45.0
                    && m["peak_speed"].as_f64().unwrap_or(0.0) >= peak_floor as f64;
                m["trial"] = json!(trial);
                m["passed"] = json!(passed);
                all_passed &= passed;
                results.push(m);
                if ev.get("ev").and_then(Value::as_str) != Some("arrived") {
                    break; // a stall may have crossed a map exit; don't call later frozen trials valid
                }
            }
            Ok(json!({
                "passed": all_passed && results.len() == trials as usize,
                "limits": {
                    "max_cross_track": cross_limit,
                    "max_heading_error": heading_limit,
                    "max_yaw_step": 45.0,
                    "min_peak_speed": peak_floor,
                    "reverse_frames": 0,
                    "hopped": true,
                },
                "start": start,
                "end": end,
                "trials": results,
            }))
        }
        .await;
        finish(r)
    }

    #[tool(description = "Order a bot to fly a specific rocket-jump link and await the full rj_result \
        telemetry (stance offset, aim error, fire-timing error, landing miss, outcome).")]
    async fn rocket_jump(&self, Parameters(a): Parameters<RjArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let bot = self.resolve_bot(a.bot).await?;
            let timeout = Duration::from_secs_f32(a.timeout.unwrap_or(15.0));
            let conn = self.conn().await?;
            let rx = conn.events.subscribe();
            self.req(&format!("rj {bot} {}", a.link), SHORT).await?;
            conn.await_event(rx, |v| is_ev(v, "rj_result", bot), timeout).await
        }
        .await;
        finish(r)
    }

    #[tool(description = "End-to-end test of one rocket-jump link: prep the bot, place it at the \
        source (teleport by default, or `via: goto` to also test reachability), fire the jump, and \
        return the telemetry.")]
    async fn test_link(&self, Parameters(a): Parameters<TestLinkArgs>) -> Result<CallToolResult, McpError> {
        let via = a.via.unwrap_or_else(|| "teleport".to_string());
        finish(self.op_test_link(a.link, &via, a.bot).await)
    }

    #[tool(description = "Sweep a batch of rocket-jump links (default: all on the map), returning \
        per-link results plus a summary (outcome counts, mean landing miss).")]
    async fn test_links(&self, Parameters(a): Parameters<TestLinksArgs>) -> Result<CallToolResult, McpError> {
        let via = a.via.unwrap_or_else(|| "teleport".to_string());
        finish(self.op_test_links(a.links, &via).await)
    }

    #[tool(description = "Set any of the rtx_rj_* driver knobs (stance, aim_tol, stance_timeout, \
        liftoff_timeout, ballistic_slack, delay_bias, pitch_bias). Only the fields you pass change.")]
    async fn set_knobs(&self, Parameters(a): Parameters<SetKnobsArgs>) -> Result<CallToolResult, McpError> {
        let r = async {
            let mut set = serde_json::Map::new();
            for (name, val) in [
                ("rtx_rj_stance", a.stance),
                ("rtx_rj_aim_tol", a.aim_tol),
                ("rtx_rj_stance_timeout", a.stance_timeout),
                ("rtx_rj_liftoff_timeout", a.liftoff_timeout),
                ("rtx_rj_ballistic_slack", a.ballistic_slack),
                ("rtx_rj_delay_bias", a.delay_bias),
                ("rtx_rj_pitch_bias", a.pitch_bias),
            ] {
                if let Some(v) = val {
                    self.req(&format!("set {name} {v}"), SHORT).await?;
                    set.insert(name.to_string(), json!(v));
                }
            }
            Ok(Value::Object(set))
        }
        .await;
        finish(r)
    }

    #[tool(description = "Read back the current rtx_rj_* knob values.")]
    async fn get_knobs(&self) -> Result<CallToolResult, McpError> {
        let r = async {
            let mut out = serde_json::Map::new();
            for name in KNOBS {
                let d = self.req(&format!("get {name}"), SHORT).await?;
                out.insert((*name).to_string(), d.get("value").cloned().unwrap_or(Value::Null));
            }
            Ok(Value::Object(out))
        }
        .await;
        finish(r)
    }

    #[tool(description = "Run a raw server console command (escape hatch, e.g. \"map bravado\" or \
        \"set sv_gravity 700\"). A command containing \"map\" invalidates the cached link list.")]
    async fn console_cmd(&self, Parameters(a): Parameters<ConsoleArgs>) -> Result<CallToolResult, McpError> {
        if a.command.contains("map") {
            self.inner.links.lock().unwrap().take();
        }
        finish(self.req(&format!("cmd {}", a.command), SHORT).await)
    }
}

#[tool_handler]
impl ServerHandler for RtxMcp {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build()).with_instructions(
            "RTX QuakeWorld bot control, inspection, and movement-tuning bridge over the game's \
             TCP control channel. Attach to a running match with server_connect, or launch an \
             isolated harness with server_start; configure it with set_cvars, then match_start \
             locks the roster and waits until the match is live. To verify a movement change, run \
             corridor_test and read its drift / peak-speed / reverse-frame report. To study team \
             play, poll status for match state, each bot's goal/stack/route, and the oracle's plan \
             and evaluation counters; bot_route and inspect_cell explain a bot's path and the nav \
             links around a cell. For rocket-jump work use list_rj_links / test_links (curl links: \
             list_curl_links). Link ids are not stable across server_restart or a `map` change — \
             re-list after either.",
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corridor_metrics_measure_drift_and_reverse_motion() {
        let ev = json!({
            "ev": "arrived",
            "origin": [214.0, 2990.0, 24.0],
            "dist": 10.2,
            "traj": [
                [1.0, 224.0, 1440.0, 24.0, 0.0, 320.0, 0.0],
                [1.1, 194.0, 1500.0, 48.0, -100.0, 500.0, 100.0],
                [1.2, 214.0, 2990.0, 24.0, 0.0, -120.0, 0.0]
            ]
        });
        let m = corridor_metrics(&ev, [224.0, 1440.0, 24.0], [224.0, 2992.0, 24.0]).unwrap();
        assert_eq!(m["event"], "arrived");
        assert!((m["elapsed"].as_f64().unwrap() - 0.2).abs() < 1e-4);
        assert_eq!(m["samples"], 3);
        assert_eq!(m["max_cross_track"], 30.0);
        assert_eq!(m["reverse_frames"], 1);
        assert_eq!(m["hopped"], true);
        assert!(m["max_heading_error"].as_f64().unwrap() > 170.0);
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let service = RtxMcp::new().serve(stdio()).await?;
    service.waiting().await?;
    Ok(())
}
