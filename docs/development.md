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

## Contributing notes

The source is hand-wrapped narrower than rustfmt's `max_width` — please don't run `cargo fmt`;
it would churn the whole crate. Match the surrounding style instead.

---

*See also: [cvar reference](cvars.md) · [network client](netclient.md)*
