# rtx

A native-Rust **QuakeWorld game module** with **Unreal Tournament-style movement**
layered on top of faithful QuakeWorld gameplay.

It reimplements the original QuakeWorld `qwprogs.dat` logic (the `qw-qc` QuakeC) as a
`cdylib` that the server loads as a **pr2 native game module** (`GAME_API_VERSION 16`) in
place of `qwprogs.dat` — the same host ABI [KTX](https://github.com/QW-Group/ktx) uses. The
gameplay mirrors stock QuakeWorld; the movement additions are the reason this exists.

## Unreal Tournament 4 influences

Mechanics inspired by **Unreal Tournament 4**, layered on top of QuakeWorld. Each is a cvar;
all are server-authoritative. Set any to `0` to disable.

### Movement — air game & wall dodge

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_doublejump` | `1` | A second jump in mid-air, **once per air travel**. Gated so a jump tapped just before landing can't steal it — QuakeWorld **bunny hopping is preserved**. |
| `rtx_walljump` | `1` | Kick off a wall you jump into: your velocity is mirrored across the wall and you launch out-and-up — UT's wall dodge. Repeatable; geometry-limited. |
| `rtx_elevator_jump` | `2` | Jumping off a **rising lift** folds the lift's speed into your jump (`lift_speed × cvar + base`), so you launch higher the faster it moves. It's a multiplier: `0` off, `1` = the lift's true speed, `2`+ for more air. |

### Combat — the shock combo

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_shootable_grenades` | `1` | Shooting a live grenade in flight detonates it. This is QuakeWorld's take on UT's **Shock Rifle combo** — where the slow energy ball from alt-fire is set off by the fast hitscan beam. Here the grenade is the slow projectile, and any shot that damages it triggers the blast. |

### A note on prediction

These run **server-side** (in `PlayerPreThink`, before the engine's player move). Stock
QuakeWorld client prediction doesn't know the rules, so each triggers a one-frame correction
"pop" — the jump itself is authoritative and always happens, but predicting it smoothly would
need a CSQC client mirroring the same `pmove` logic.

## Grappling hook

Not a UT mechanic but a classic QuakeWorld one — the Wedge (Steve Bond) grappling hook,
ported from **purectf** (minus its CTF team logic), with
[KTX](https://github.com/QW-Group/ktx)'s quieter sound: a one-shot throw/impact instead of
the old looping chain rattle.

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_grapple` | `1` | Every player **spawns holding** a grappling hook (no ammo). Fire to throw it; the hook sticks to walls — or players, who get dragged and lightly damaged — and reels you toward it while you hold fire (a moving anchor carries you along). Select it with **impulse 22** or by **double-tapping impulse 1** (toggles axe ↔ hook). `0` disables it. |

Throw and reel speeds are tunable via `rtx_hook_speed` (default `1.25`) and `rtx_hook_pull`
(default `1.0`) — purectf's `hookspeed`/`hookpull`, each a multiplier on its base speed.

It reels server-side too, so it shows the same one-frame prediction pop as the movement
features above. The hook's models and viewmodel must be in the gamedir:
`progs/{star,bit,v_star}.mdl`.

## Game modes

The game mode is selected with **`rtx_mode`**, read at map load. Modes are pluggable: each one
overrides only the policy it changes — **ruleset** (round/damage/respawn rules), **spawns**,
**loadout**, and the **bot brain** — behind a small `GameMode` trait (`src/mode/`). Free-for-all
is just the baseline mode, so adding a mode doesn't touch the generic gameplay or bot code.

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_mode` | `ffa` | `ffa` = free-for-all deathmatch (stock behaviour). `ra` = Rocket Arena. |
| `rtx_ra_countdown` | `3` | Rocket Arena: seconds of spawn-protected countdown before "FIGHT". |
| `rtx_ra_lightning_gun` | `0` | Rocket Arena: include the lightning gun in the arena arsenal (`0` leaves it out). |

**`ra` — Rocket Arena.** Round-based 1v1 duels following the classic arena loop (ported from the
Frogbot-Rocket-Arena QuakeC, minus its clan-arena team machinery). Two players fight in the arena
at a time; everyone else waits in the **audience** (the `info_player_deathmatch` spots — the
stands) and roams there. Each round the fighters spawn with a
**full loadout** (all weapons, full ammo, red armour) **inside the arena** (the
`info_teleport_destination` spots), are invulnerable through a short countdown, then fight. Getting
killed **drops you to the audience**; the **winner stays** and faces the next challenger pulled
from the front of the audience queue (losers go to the back), so the arena is always a fresh duel.
On a plain deathmatch map with no teleport destinations it falls back to DM spawns so the mode
still runs. Bots play it fully — see below.

## Bots

Navmesh-driven bots that need **no per-map waypoint files** — the navmesh is generated from the
map's BSP clip hull when the map loads. Bots are real client slots: the engine runs their input
through the same player-move code as humans, so gravity, stepping, and jumps come for free.

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_bots` | `0` | How many bots to keep on the server. The population is reconciled to this count (spawning/removing as needed), leaving room for humans. Bots only spawn once the map's navmesh is built. |
| `rtx_bot_skill` | `3` | Bot skill (reserved for combat tuning later). |

In free-for-all each bot pathfinds to the best reachable **item pickup**, or **follows the nearest
human** when nothing's worth fetching (through doors, off ledges, across jumps, recovering after a
missed jump). The mode can redirect this brain without touching it: in **Rocket Arena** bots
**fight** — they path to the nearest enemy and, once they have line of sight, aim (leading the
target for rockets), pick a weapon by range, strafe/retreat, and fire — and, when eliminated, roam
the audience like everyone else. The combat layer (`src/bot_combat.rs`) is generic and reused by
any mode that hands a bot an enemy; aim tightens with `rtx_bot_skill`.

## Building

```sh
cargo build --release
cp target/release/librtx.so /path/to/server/qw/qwprogs.so   # or .dylib, .dll
```

The server needs pr2 / API 16 support. Prebuilt `qwprogs.so` / `qwprogs.dylib` artifacts are
produced by the GitHub Actions `build` workflow.

## License

Copyright © 2026 Daniel Svensson.

Licensed under the GNU Affero General Public License, version 3 or later
([AGPL-3.0-or-later](LICENSE)).
