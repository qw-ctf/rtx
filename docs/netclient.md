# The Bots as Network Clients

The same bots also run **outside** a server, as ordinary QuakeWorld — or NetQuake — clients.
`rtx-client` connects over UDP, completes a real handshake and signon, and plays — so the bots
can be pitted against humans on any server, or against bots hosted by someone else's `qwprogs`.

It is the *same brain*: the same `run_bots` over the same `GameState`, and it doesn't know which
of the two embodiments it's under.

Part of the [rtx manual](../README.md) · the brain itself: [the bots](bots.md)

## Quick start

```sh
cargo build --release -p rtx-client
./target/release/rtx-client --server localhost --basedir ~/Games/Quake --bots 2
```

It's optional to compile — a cargo feature (`netclient`) on `rtx-game` that the default build
never enables, so the game module is unaffected either way.

## Command line

| flag | what it does |
|------|--------------|
| `--server <host[:port]>` | server to join (default port 27500, or 26000 with `--proto nq`) |
| `--proto <qw\|nq>` | wire protocol: QuakeWorld (default) or NetQuake |
| `--game <dir>` | gamedir the maps live under (default `id1` for `--proto nq`) |
| `--basedir <dir>` | Quake directory holding `qw/` and `id1/` — the maps must be here |
| `--bots <n>` | how many bots to bring (default 1) |
| `--name <s>` | label after the `bot•` tag (default: a random name per bot) |
| `--team <s>`, `--skin <s>`, `--colors <top> <bottom>` | userinfo, as any client sends (colours 0–13) |
| `--skill <0..7>` | as `rtx_bot_skill` (default 3) |
| `--spectate` | watch without playing; the parser soak |
| `--no-auto-ready` | don't answer KTX ready/join prompts |
| `--no-download` | fail rather than fetch a missing map |
| `--config <file>` | a cvar cfg to apply on startup — the client's `server.cfg` (defaults to `<basedir>/rtx.cfg`) |
| `--soak <secs>` | exit after this long |
| `--control-port <n>` | the same TCP harness the server-side bots expose (`status`, `goto`, `cell`, `links`, …) |
| `--wiretap <dir>` | record every datagram, as a parser fixture |
| `+set <cvar> <value>` | override an rtx tunable (e.g. `+set rtx_mode ctf`), repeatable |

## It works out what game it joined

rtx servers publish the mode in serverinfo, in the same vocabulary
[KTX](https://github.com/QW-Group/ktx) uses (`mode`, `status`), so one parser reads both — and
the client selects the matching brain (deathmatch, midair, CTF, and so on) before it spawns the
world. A `+set rtx_mode`/`rtx_match` on the command line overrides the guess; a server that says
nothing recognisable is played as deathmatch. Teams, when there are any, come through userinfo,
so the bots fight the right side without needing to be told the format.

On a KTX server the client also answers the ready/join prompts by itself, so the bots actually
get into the game (`--no-auto-ready` to keep them polite).

## Maps are found or fetched

`--basedir` names the directory holding `qw/`, `id1/`, … and each is searched the way the engine
searches it: that directory's paks first (highest numbered first), then its loose files —
getting that order wrong loads a different copy than the server has, and the checksum sent at
`prespawn` is then a checksum of the wrong file, which a server answers by dropping the
connection without a word.

A map that's nowhere on disk is fetched before signon completes. Both protocols first try the
community HTTP map repository. If that fails, QuakeWorld asks the connected server for the map:
an FTE `CHUNKEDDOWNLOADS` server gets up to 75 random-access chunk requests in flight, while an
older server falls back to regular sequential QuakeWorld blocks. NetQuake remains HTTP-only.

Downloads are written to a unique partial file beside the destination, checked for a Quake v29 BSP
header, and renamed into place only after the complete file has arrived. `--no-download` disables
both HTTP and in-protocol fetching.

## Squads

`--bots N` brings a **squad**: N connections in one process, sharing one world. That isn't a
shortcut — it's what the bots already have inside `qwprogs`, where teammates share item timers
and an opponent model because they talk to each other. Each bot's body and stats are its own;
what any of them sees, all of them know.

## NetQuake

`--proto nq` speaks classic **NetQuake** instead of QuakeWorld: the NQ connection dance, default
port 26000, and `id1` as the default gamedir (`--game` to change it). The same brain plays
either protocol.

## What a client cannot know

What a client cannot know, it doesn't pretend to. An enemy's health isn't on the wire and never
will be, so a client bot estimates it from what it sees and hears — which is the position a
human is in, and the reason these can play against people without being something other than a
player.

Server-side movement features are handled the same way: an rtx server **advertises** its
movement cvars in serverinfo (like KTX's `pm_*` keys) and the client mirrors them; on any other
server it forces them off — so the navmesh never plans a double-jump gap the server won't grant.

---

*See also: [the bots](bots.md) · [bot architecture](bot-architecture.md) ·
[cvar reference](cvars.md) · [development & tooling](development.md)*
