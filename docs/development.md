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
| `navview` | A minimal wgpu/winit viewer for the navmesh: renders the BSP world plus the colored nav links. Load a `.bsp` via argv or drag-and-drop. |
| `rjmcp` | An MCP (stdio) server bridging Claude Code to the game's TCP control channel, managing a local server process for rocket-jump tuning. See [its README](../crates/rjmcp/README.md). |

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
cargo run -p navview -- <map.bsp>      # the navmesh viewer
cargo run -p rjmcp --quiet             # the MCP bridge (normally launched via .mcp.json)
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

## The control channel

Setting `rtx_control_port <port>` (or `--control-port` on the client) binds a localhost TCP
channel for scripted bot puppetry: teleport a bot, send it somewhere (`goto`), fly a specific
link, read back cells, links, and telemetry — line-oriented commands in, JSON out. It exists so
a harness can drive precise, repeatable bot situations against a live server. `0` (the default)
binds nothing. Implementation: `crates/rtx-game/src/control.rs`.

The rocket-jump tuning loop built on top of it — an MCP server that Claude Code drives, plus the
`rtx_rj_*` knobs it turns — is documented in [`crates/rjmcp/README.md`](../crates/rjmcp/README.md).

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

### DM3 Ring→RA acceptance trial

The authoritative server control channel also exposes:

```text
<id> ra_trial <bot> [ring|ra_spawn|local] [max_secs]
```

`ring` is the default and starts at the corpus major-zone centre `(240,-32,56)`. `ra_spawn` is
bound to the exact stock `info_player_deathmatch` entity at `(192,-208,-176)` in `RA.tunnel`; it
refuses a custom entity set that moved/removed that spawn. The planner starts from nearest standing
cell 1281 at `(192,-224,-176)`, but physical placement preserves the production spawn XY and its
engine `+1 Z` offset exactly: `(192,-208,-175)`. `local` starts at the upper-lip regression reproduction `(360,-677,264)` and is
only an internal micro-regression. The default hard deadline is scenario-specific: 2.435059 seconds
for `local`, 12.6255 seconds for `ra_spawn`, and 9.604003 seconds for `ring`; callers may pass a longer safety deadline and grade the emitted
telemetry separately. The command is server-only: it restores
DM3's red armor, resets the bot to a stock SG spawn, chooses the cheapest reachable touch-valid RA
terminal with the live bot link pricing, and installs a completion-critical RA item goal. It does
not issue a coordinate `goto`; ordinary A*, traversal, terminal retry, item touch, and armor pickup
code execute unchanged. Because RA is deliberately seeded, this trial proves item-goal execution,
not autonomous strategic selection (`forced_item_goal:true` is explicit in the result).

The acknowledgement records the snapped start, selected terminal, and complete planned route. One
`ra_trial_result` event with the same `request_id` then records the authoritative pickup triplet
(200 armor, `ARMOR3`, hidden RA), elapsed time, current route/link, commanded wish/buttons and a
per-frame trajectory. It hard-fails `planned_drop`, `fall`, sustained BSP-blocked `wall_push`,
`no_pickup`, `item_taken_elsewhere`, `goal_lost`, `stall`, death, or `timeout`. Thus a movement
change cannot pass merely by reaching a coordinate near RA.

`elapsed` is start-centre→authoritative-pickup time. It is deliberately conservative, but it is not
the corpus benchmark's Ring-sphere-exit→RA-sphere-entry interval; the lab runner derives that
interval (and next-distinct-zone, low-speed, minimum-Z, and maximum-velocity checks) from the frame
samples. `wall_contacts` covers physical contacts with static BSP. Dynamic brush contacts require
engine collision telemetry and are outside that metric; stock DM3's accepted Ring→RA route has no
dynamic gate.

The `ra_spawn` default is independently calibrated from the local 2026-07-17 MVD snapshot. A run
starts only when the authoritative spawn event's trajectory sample is exactly the stock entity
origin `(192,-208,-176)` and ends at the same slot's authoritative RA `taken` event within 30
seconds. To make the race comparable, admission requires RA to be active at spawn and the runner to
make the first subsequent RA take, with no intervening same-slot spawn before that pickup. The
admitted same-life set is 86 runs across 77 demos: min 6.680, p10 10.0095, p50 12.6255, p90 18.0630
seconds. These are controller-safe aggregates, not human routes or input sequences.

For a one-off raw run, keep one TCP connection open long enough to receive both messages:

```sh
printf '1 ra_trial 1 local 9.604\n' | nc -q 12 127.0.0.1 27950
```

The repeatable streak grader and corpus-derived thresholds live in the separate
`bot-control-kit` lab repo (`ops/dm3_ra_acceptance.py`). Its JSONL artifacts retain the full result
event for A/B comparison and replayable diagnosis.

## Contributing notes

The source is hand-wrapped narrower than rustfmt's `max_width` — please don't run `cargo fmt`;
it would churn the whole crate. Match the surrounding style instead.

---

*See also: [cvar reference](cvars.md) · [network client](netclient.md) ·
[bot architecture](bot-architecture.md)*
