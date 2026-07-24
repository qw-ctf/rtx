# rtx

A **QuakeWorld game module in native Rust** — faithful QuakeWorld gameplay with
**Unreal Tournament-style movement** on top, and **navmesh bots** that play it anywhere:
on the server, or connected to any server as real network clients, both QuakeWorld and
NetQuake servers.

It reimplements the original QuakeWorld `qwprogs.dat` logic as a `cdylib` the server loads as a
**pr2 native game module** (`GAME_API_VERSION 16`) — the same host ABI
[KTX](https://github.com/QW-Group/ktx) uses. Drop it in place of `qwprogs.dat` and the gameplay
mirrors stock QuakeWorld; everything on top is a cvar you can switch off.

## Highlights

- **UT-style movement** — double jump, wall dodge, elevator jump, and the shock combo
  (shootable grenades), layered onto QuakeWorld without breaking bunnyhopping.
  → [movement & combat](docs/movement.md)
- **The grappling hook** — the classic Wedge hook, ported from purectf. Bots swing it too.
  → [movement & combat](docs/movement.md#grappling-hook)
- **Game modes** — deathmatch, Rocket Arena, midair, CTF, and timed race routes, each
  composable with an orthogonal match format (FFA, `1on1`, `2on2`, `2on2on2`, …).
  → [modes & match composition](docs/modes.md)
- **Bots without waypoint files** — the navmesh is generated from the map's BSP at load time.
  Bots bunnyhop, speed-jump, double-jump, rocket-jump, and grapple across it.
  → [the bots](docs/bots.md)
- **Bots that play like players** — they see, hear, and feel before they fight; value items the
  way KTX duellers do; model their opponents from observation; and bank grenades around
  corners. In team modes they spread fire, reserve pickups, and run CTF roles.
  → [the bots](docs/bots.md)
- **The same bots as network clients** — `rtx-client` joins any QuakeWorld (or NetQuake) server
  over UDP with the identical brain, fetching maps it doesn't have.
  → [the bots as network clients](docs/netclient.md)
- **Everything is a cvar** — every mechanic, mode, and bot behaviour has a switch.
  → [cvar reference](docs/cvars.md)

## Quick start

Run a server with it:

```sh
cargo build --release
cp target/release/librtx.so /path/to/server/qw/qwprogs.so   # or .dylib / .dll
```

The server needs pr2 / API 16 support. Prebuilt `qwprogs` artifacts for Linux, macOS, and
Windows come out of the GitHub Actions `build` workflow.

Add bots in `server.cfg`:

```
set rtx_bot_count 4
set rtx_bot_skill 3
```

Or point the bots at somebody else's server:

```sh
cargo build --release -p rtx-client
./target/release/rtx-client --server quake.example.org --basedir ~/Games/Quake --bots 2
```

## Documentation

| page | contents |
|------|----------|
| [Movement & combat](docs/movement.md) | Double jump, wall jump, elevator jump, the shock combo, the grappling hook, choosing the arsenal. |
| [Game modes](docs/modes.md) | `dm`, `ra`, `midair`, `ctf`, `race`; match composition and the team-match lifecycle; map rotation. |
| [The bots](docs/bots.md) | Perception, navigation and the navmesh's jump/hook links, item valuation, combat, teamwork. |
| [Bot architecture](docs/bot-architecture.md) | How the brain is built: the navmesh core, the per-frame decision loop, movement drivers, the opponent model, the two embodiments. |
| [The bots as network clients](docs/netclient.md) | `rtx-client`: joining any QW/NQ server, mode detection, map downloads, squads. |
| [Cvar reference](docs/cvars.md) | Every `rtx_*` tunable with its default, on one page. |
| [Development & tooling](docs/development.md) | Building, CI artifacts, tests, the control channel and tuning harness. |

## Workspace

| crate | |
|-------|---|
| `rtx-game` | the game module (`qwprogs` replacement) — all game logic |
| `rtx-nav` | navmesh building and query over the BSP collision hull |
| `rtx-proto` | the QuakeWorld and NetQuake wire protocols as pure codecs |
| `rtx-client` | the bots as standalone network clients |
| `rtx-nav-view` | a 3D viewer for the generated navmesh (live overlay via `--live`) |
| `rtx-ctlproto` | typed msgpack schema for the game↔tools control channel |
| `rtx-auditlog` | per-bot ring buffer of compact sensor frames |
| `rtx-mcp` | an MCP bridge for driving live bot-tuning sessions |

## License

Copyright © 2026 Daniel Svensson.

Licensed under the GNU Affero General Public License, version 3 or later
([AGPL-3.0-or-later](LICENSE)).

## Thanks

Inspiration and compliance with a bit of everything thanks to: 
[Quake Bot Archive](https://github.com/Jason2Brownlee/QuakeBotArchive), [KTX](https://github.com/qw-group/ktx), [FrogBot Rocket Arena](https://web.archive.org), [ezQuake](https://github.com/qw-group/ezquake-source/), [FTE](https://github.com/fte-team/fteqw/), [QSS-M](https://github.com/timbergeron/QSS-M), [Original Quake](https://github.com/id-software/quake)