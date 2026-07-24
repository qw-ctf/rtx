# Development & Tooling

Building the artifacts, the workspace layout, CI, and the harnesses used to test and tune the
bots.

Part of the [rtx manual](../README.md)

## Workspace

| crate | what it is |
|-------|------------|
| `rtx-game` | The game module itself — a native Rust reimplementation of QuakeWorld's `qwprogs.dat` for the pr2 host ABI (`GAME_API_VERSION 16`). Builds the `rtx` cdylib. All game logic lives here. |
| `rtx-nav` | The pure navigation core: collision-hull BSP reader, navmesh builder and query, movement constants. No engine or game state — deterministic math, shared by the game and the viewer. |
| `rtx-proto` | The QuakeWorld wire protocol as a pure codec — sizebuf, info strings, checksums, netchan, the out-of-band handshake, svc parser and clc builder — with an `nq/` sibling doing the same for NetQuake. No IO, no threads. |
| `rtx-client` | The [network client](netclient.md) front door: parses argv and hands off to `rtx-game`'s `netclient` module (built with the `netclient` feature). |
| `rtx-nav-view` | A minimal wgpu/winit viewer for the navmesh: renders the BSP world plus the colored nav links. Load a `.bsp` via argv or drag-and-drop, or attach to a running game with `--live [port]` to fetch its map BSP over the control channel and overlay the live bot and its route. |
| `rtx-ctlproto` | The typed control-channel schema shared by the game and its clients (`rtx-mcp`, `rtx-nav-view`): the request / reply / event enums plus the length-framed msgpack codec. Pure, no IO. |
| `rtx-auditlog` | A once-allocated per-bot ring buffer of compact `AuditFrame` sensor snapshots (speed, bhop/hook/rj phase, posture, commit, tags), replacing per-frame console spam; the MCP's `audit` tool decodes it. |
| `rtx-mcp` | An MCP (stdio) server bridging Claude Code to the game's TCP control channel, managing a local server process for live bot control and rocket-jump tuning. See [its README](../crates/rtx-mcp/README.md). |
| `rtx-waypoint-check` | An offline checker that parses KTX's hand-authored `.bot` waypoint files, rebuilds the navmesh from the map's BSP, and reports which human rocket-jump / curl-jump connections our generated mesh reproduces, routes around, or misses — surfacing blind spots in link generation. Pure `rtx-nav`; see below. |

`cargo build` builds the default members — `rtx-nav`, `rtx-proto`, `rtx-game`. The viewer, the
MCP bridge, and the network client are deliberately excluded and built explicitly with `-p`.

`playground/` is not a crate: it's a local runtime sandbox holding QuakeWorld engine binaries
and gamedirs, used by the tuning harness to spawn a real server.

## Building

The game module:

```sh
cargo build --release
cp target/release/librtx.so /path/to/server/qw/qwprogs.so    # librtx.dylib / rtx.dll likewise
```

The server needs pr2 native-module support (`GAME_API_VERSION 16`) — the same host ABI
[KTX](https://github.com/QW-Group/ktx) uses.

The rest, each on demand:

```sh
cargo build --release -p rtx-client    # the network client
cargo run -p rtx-nav-view -- <map.bsp> # the navmesh viewer
cargo run -p rtx-nav-view -- --live    # ... attached to a running game (BSP + live route)
cargo run -p rtx-mcp --quiet           # the MCP bridge (normally launched via .mcp.json)
```

The `netclient` cargo feature on `rtx-game` is default-off and purely additive: it adds the
`netclient` module plus the `rtx-proto` and HTTP-download dependencies. The default build —
and the game module — are unaffected either way.

## CI

One workflow, `.github/workflows/build.yml`:

- **test** — `cargo test --locked` across the workspace, then
  `cargo test --locked -p rtx-game --features netclient` for the client suite.
- **build** — a release matrix producing two artifacts per platform. The drop-in server module,
  `qwprogs-linux-x86_64` (`.so`), `qwprogs-macos-arm64` (`.dylib`), and
  `qwprogs-windows-x86_64` (`.dll`), each staged under the `qwprogs.*` name a server expects; and
  the standalone [network client](netclient.md), `rtx-client-<target>` (`rtx-client.exe` on
  Windows), built by its own `cargo build -p rtx-client` step so its `netclient` feature stays
  out of the game module.

## Tests

- **Wire-codec fixtures** — `crates/rtx-proto/tests/` pins the QuakeWorld and NetQuake codecs
  against recorded datagrams; `rtx-client --wiretap <dir>` records new fixtures from a live
  connection.
- **Demo replay** — `crates/rtx-game/src/demo_replay.rs` holds env-gated tests that replay real
  human QuakeWorld demos (dm3/dm4) to check the pmove simulation's fidelity and that the bhop
  bot matches or beats the human line.
- **Navmesh** — unit and integration tests in `crates/rtx-nav`.

## Navmesh coverage vs. hand-authored waypoints

Our navmesh is generated from the BSP; KTX's `.bot` waypoint files are hand-authored by humans, and
their rocket-jump and curl (air-control) jump paths are distilled from recorded human play. Where a
human wired a connection our generator didn't, we have a blind spot worth chasing.
`rtx-waypoint-check` finds them offline — no server, no network:

```sh
cargo run --release -p rtx-waypoint-check -- dm3 dm4 dm6 e1m2 bravado
cargo run --release -p rtx-waypoint-check            # every waypoints/*.bot whose BSP resolves
cargo run --release -p rtx-waypoint-check -- --radius 128 dm3   # loosen endpoint matching
```

For each map it parses the `.bot` file, rebuilds the navmesh with the viewer's stock-DM recipe (plus
teleporters wired from the entity lump), and classifies every authored rocket-jump / curl-jump path
in descending strength: **MATCHED** (a same-kind link bridges the endpoints), **JUMP** (a different
airborne link does), **ROUTE** (no matching link but a route exists), **UNREACH** / **UNSNAP** (the
endpoints don't connect, or one is off the mesh — the blind spots). Exit code is `1` when any path is
unreachable/off-mesh, `0` when all are at least route-connected.

Two things to know when reading the output:

- **Marker numbering.** KTX assigns the low marker ids to the map's *entity* markers — items, doors,
  triggers, spawns — claimed in entity-lump order *before* the file's own `CreateMarker`s. The tool
  reproduces that entity walk to resolve path references, and prints a `K ok` / `K MISMATCH`
  cross-check (`walk` vs. file-implied). A mismatch means the `.bot` file was authored against a
  different entity set (an alternate `.ent`, a map variant) and the entity-marker positions are
  unreliable for that map.
- **Plats aren't spliced offline** (their traversal needs the live mover), so a path that rides a
  lift can read as `ROUTE`/`UNREACH`; the per-map `func_plat` count flags where that applies.
  Teleporters *are* wired. Brush-entity endpoints (doors, triggers) use a submodel-bounds
  approximation and are shown with a `~`.

`waypoints/` is gitignored — drop KTX's `.bot` files there (base install lives in `playground/`).
Spot-check an `UNREACH` finding on a live server with the `rtx-mcp` `list_rj_links` /
`list_curl_links` / `teleport` tools.

## The control channel

Setting `rtx_control_port <port>` (or `--control-port` on the client) binds a localhost TCP
channel for scripted bot puppetry: teleport a bot, send it somewhere (`goto`), fly a specific
link, read back cells, links, and telemetry. The wire is length-framed msgpack of the typed
[`rtx-ctlproto`](../crates/rtx-ctlproto) schema; several clients can attach at once (replies route
to the requester, events broadcast to all), so the `rtx-nav-view` viewer can watch the same match
the MCP bridge is driving. It exists so a harness can drive precise, repeatable bot situations
against a live server. `0` (the default) binds nothing. Implementation: `crates/rtx-game/src/control.rs`.

The rocket-jump tuning loop built on top of it — an MCP server that Claude Code drives, plus the
`rtx_rj_*` knobs it turns — is documented in [`crates/rtx-mcp/README.md`](../crates/rtx-mcp/README.md).

## Profiling the bot brain

`rtx_bot_prof <seconds>` prints a periodic profile of what the bots cost, to the server console
only — never to clients or the MVD/QTV stream. It defaults to `10`, on the view that a server
that has started missing its frames should say so without being asked; `0` turns it off and
times nothing at all.

```
rtx bots: 10.0s 772 frames 6 bots | avg 0.47 p95 1.71 max 10.72 ms
rtx bots: budget 12.99ms (maxfps 77) | worst 83% (2.27ms under) | 0/772 over | per-bot avg 0.08 max 4.55 ms
rtx bots: phases avg/max ms | objective 0.03/4.49 | steer 0.01/3.26 | combat 0.01/0.47
rtx bots: worst frame 10.72ms | objective 10.32 steer 0.15 combat 0.10 | per-bot [0.12 0.02 3.14 2.55 2.63 2.26]
```

The last line is an autopsy of the single worst frame in the window, and it's the one that
answers *why*: `avg`/`p95`/`max` tell you a spike happened, but only the per-bot split tells one
dear bot (`[0.06 0.02 7.51 0.03 0.01 0.01]`) from the whole squad landing on one frame
(`[4.98 4.09 3.92 3.77 3.74 4.04]`) — a distinction that decides whether you optimise the work
or spread it. This is how the goal-selection lockstep was found; see `GOAL_SELECT_SPREAD`.

One spike is expected and harmless: a batch of bots is added within a single bot frame, so a
freshly spawned squad takes its first (dearest) pick together, once. They scatter within a cycle
or two and stay scattered.

Read it with three facts from mvdsv's `SV_RunBots` (`src/sv_phys.c`) in mind:

- **A frame is the whole squad.** The engine calls the module *once per bot frame, not once per
  bot* — `SV_ProgStartFrame(true)` runs before its client loop — so one frame sample is every
  bot's thinking, and `bots` is how many were in it.
- **The budget is `maxfps`.** `SV_RunBots` reads that cvar (`{"maxfps", "77", CVAR_SERVERINFO}`),
  substitutes 77 outside `[20, 1000]`, and won't run a bot frame until `1/maxfps` has elapsed.
  Since `SV_Frame` runs `SV_Physics()` *then* `SV_RunBots()`, time spent here is added to the
  server's own frame — an overrun makes the *server* late, not just the bots.
- **It measures deciding, not moving.** Per-bot physics (`SV_RunCmd`, trigger touches) happens
  outside our call and isn't counted.

Watch `p95` and `max` rather than `avg`: the costly work is spiky, not steady — goal selection
floods the whole navmesh but only every 1.5s, and A* re-paths every 0.4s. The `phases` line
attributes a spike (`objective` is goal selection, `steer` is A*, `combat` is traces and grenade
rollouts); they sum to less than the frame total, the remainder being sensing and command emit.

## Contributing notes

Run `cargo fmt` before committing — the tree is rustfmt-clean, with `rustfmt.toml` pinning
`max_width = 120`.

---

*See also: [cvar reference](cvars.md) · [network client](netclient.md) ·
[bot architecture](bot-architecture.md)*
