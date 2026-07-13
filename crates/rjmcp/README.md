# rjmcp — the rocket-jump tuning harness

An MCP (stdio) server that lets Claude Code drive rtx bots through scripted rocket-jump tests and
tune the driver knobs, without hand-flying bots in a live server.

```
Claude Code ──MCP stdio──▶ rjmcp ──TCP 127.0.0.1:port──▶ control.rs in librtx.dylib
                             └─ spawns/kills playground/mvdsv (+exec rjtest.cfg)
```

## Pieces

- **Game side** (`crates/rtx-game/src/control.rs`): a cvar-gated (`rtx_control_port`) localhost TCP
  server. Inbound is line text `<id> <verb> args…`; outbound is one JSON line per reply/event. It
  puppets a bot: teleport it, order it to a position (`goto`) or to fly a specific rocket-jump link
  (`rj`), and emit per-attempt telemetry (`rj_result`) and reachability (`arrived`/`goto_stall`).
- **Runtime knobs** (`rtx_rj_*` cvars, read live): `stance`, `aim_tol`, `stance_timeout`,
  `liftoff_timeout`, `ballistic_slack`, and the two solve biases `delay_bias` (added to the fire
  delay) and `pitch_bias` (added to the fire pitch). Defaults mirror the driver constants, so an
  untouched server is unchanged.
- **This bridge** (`crates/rjmcp`): manages mvdsv, connects to the control port, exposes MCP tools.

## Use

Registered in the repo-root `.mcp.json` as `rjtune`. After a Claude Code session restart (or
`/mcp`), approve it, then:

1. `server_start(map="aerowalk")` — launches mvdsv with the harness config (1 bot, control port
   open, all build-gating cvars set explicitly), waits for the navmesh + bot, returns status.
2. `list_rj_links` — every rocket-jump link: id, source/target, solved fire pitch/yaw, delay,
   airtime, self-damage.
3. `test_link(link=…)` / `test_links()` — prep the bot, place it at the source (teleport, or
   `via:"goto"` to also test reachability), fire the jump, return the telemetry.
4. `set_knobs(delay_bias=-0.08, …)` then re-test to tune. `get_knobs` reads them back.
5. `console_cmd("map bravado")` to switch maps (re-list links afterward — ids are not stable).

Every tool takes an optional `bot` (defaults to the first live bot). Reachability stalls
(no progress ~4 s) surface as `goto_stall` — the signal that a rocket-jump *source* cell can't be
stood on.

## The config quirk it exposes

On a fresh boot the first map's navmesh builds with **rocket jumps gated off** (`rjump 0`): the
gating cvars (`rtx_bot_rocketjump`, etc.) are seeded by the module's `GAME_INIT` `cvar_default`,
whose queued `set` flushes only *after* the first-frame navmesh build reads them. A later `map`
rebuilds correctly (`rjump 531` on aerowalk). The harness config sets those cvars explicitly before
`map` so the first build already has them; a real server would want the same, or a root fix in the
build-cvar flush timing.
