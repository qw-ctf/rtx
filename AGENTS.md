# Agent notes for rtx

## Live bot testing — use the `rtx-mcp` MCP

Prefer the `rtx-mcp` MCP server (registered in the repo-root `.mcp.json`) over hand-driving
mvdsv when you need to run or observe bots live:

- `server_connect` attaches to a running server; `server_start` launches an isolated harness.
- Configure with `set_cvars`, then `match_start` — it locks the roster and waits until the
  match is live (retrying only while the server stays in settled warmup).
- Verify a movement change with `corridor_test`: it reports drift, peak speed, and reverse
  frames for an A→B run (bots should hold 800+ ups on the 100m runway).
- Study play with `status` (match state, and each bot's team, stack, inventory, item goal,
  posture, enemy, route head, plus the oracle's plan and evaluation counters); `bot_route`
  expands a full route and `inspect_cell` explains the nav links around a cell.
- Rocket-jump work: `list_rj_links` / `test_links`; curl links: `list_curl_links`.

Link ids are not stable across `server_restart` or a `map` change — re-list after either. The
bridge lives in `crates/rtx-mcp/`; see its [README](crates/rtx-mcp/README.md).

## Build / test

- Default `cargo build` / `cargo test` cover only the game module, its nav core, and the wire
  codec. Build the MCP, viewer, and client explicitly: `-p rtx-mcp` / `-p navview` /
  `-p rtx-client`.
- Run `cargo fmt` before committing — the tree is rustfmt-clean, with `rustfmt.toml` pinning
  `max_width = 120`.
