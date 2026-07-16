# Movement & Combat

The mechanics rtx layers on top of faithful QuakeWorld gameplay: three Unreal Tournament-inspired
movement moves, the shock combo, and the classic grappling hook. Each is a cvar; all are
server-authoritative. Set any to `0` to disable it.

Part of the [rtx manual](../README.md) · every tunable on one page: [cvar reference](cvars.md)

## At a glance

| cvar | default | mechanic |
|------|---------|----------|
| `rtx_doublejump` | `1` | a second jump in mid-air, once per air travel |
| `rtx_walljump` | `1` | kick off a wall you jump into — UT's wall dodge |
| `rtx_elevator_jump` | `2` | a rising lift folds its speed into your jump |
| `rtx_shootable_grenades` | `1` | shoot a live grenade to detonate it — the shock combo |
| `rtx_grapple` | `1` | every player spawns holding a grappling hook |
| `rtx_hook_speed` / `rtx_hook_pull` | `1.25` / `1.0` | hook throw / reel speed multipliers |
| `rtx_weapons` | *(full roster)* | which weapons exist on the server |

## Double jump

A second jump in mid-air, **once per air travel**. Gated so a jump tapped just before landing
can't steal it — QuakeWorld **bunny hopping is preserved**.

When the double jump is on, the bots' navmesh also links the wider gaps and higher ledges it
reaches — see [double-jump links](bots.md#double-jump-links).

## Wall jump

Kick off a wall you jump into: your velocity is mirrored across the wall and you launch
out-and-up — UT's wall dodge. Repeatable; geometry-limited.

## Elevator jump

Jumping off a **rising lift** folds the lift's speed into your jump (`lift_speed × cvar + base`),
so you launch higher the faster it moves. It's a multiplier: `0` off, `1` = the lift's true
speed, `2`+ for more air.

## The shock combo — shootable grenades

Shooting a live grenade in flight detonates it. This is QuakeWorld's take on UT's **Shock Rifle
combo** — where the slow energy ball from alt-fire is set off by the fast hitscan beam. Here the
grenade is the slow projectile, and any shot that damages it triggers the blast.

The bots play this game too, defensively and offensively — see
[the shootable-grenade game](bots.md#the-shootable-grenade-game).

## A note on prediction

These run **server-side** (in `PlayerPreThink`, before the engine's player move). Stock
QuakeWorld client prediction doesn't know the rules, so each triggers a one-frame correction
"pop" — the jump itself is authoritative and always happens, but predicting it smoothly would
need a CSQC client mirroring the same `pmove` logic.

## Grappling hook

Not a UT mechanic but a classic QuakeWorld one — the Wedge (Steve Bond) grappling hook, ported
from **purectf** (minus its CTF team logic), with [KTX](https://github.com/QW-Group/ktx)'s
quieter sound: a one-shot throw/impact instead of the old looping chain rattle.

With `rtx_grapple 1` (the default) every player **spawns holding** a grappling hook (no ammo).
Fire to throw it; the hook sticks to walls — or players, who get dragged and lightly damaged —
and reels you toward it while you hold fire (a moving anchor carries you along). Select it with
**impulse 22** or by **double-tapping impulse 1** (toggles axe ↔ hook).

Throw and reel speeds are tunable via `rtx_hook_speed` (default `1.25`) and `rtx_hook_pull`
(default `1.0`) — purectf's `hookspeed`/`hookpull`, each a multiplier on its base speed.

It reels server-side too, so it shows the same one-frame prediction pop as the movement features
above. The hook's models and viewmodel must be in the gamedir: `progs/{star,bit,v_star}.mdl`.

Bots use it to navigate — see [hook links](bots.md#hook-links), where the navmesh grows edges a
bot swings across.

## Choosing the arsenal — `rtx_weapons`

`rtx_weapons` (default `axe hook sg ssg ng sng gl rl lg`) is the set of weapons the server runs
with, as a space-separated token list. A weapon **absent from the list is removed everywhere**:
its map pickup (`weapon_*`) never spawns and it's stripped from every spawn kit, so it can't be
picked up or fired (bots included). Set e.g. `rtx_weapons "axe sg rl"` for a rockets-first
server.

Unknown tokens (e.g. `coil`) are ignored; `hook` composes with `rtx_grapple` (both must allow
it). Map pickups update on the next **map load**, spawn kits on the next **respawn**.

---

*See also: [game modes](modes.md) · [the bots](bots.md) · [cvar reference](cvars.md)*
