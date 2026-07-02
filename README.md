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
`progs/{star,bit,v_star}.mdl`. Bots use it to navigate — see [Bots](#bots), where the navmesh grows
hook links a bot swings across.

## Game modes

The game mode is selected with **`rtx_mode`**, read at map load. Modes are pluggable: each one
overrides only the policy it changes — **ruleset** (round/damage/respawn rules), **spawns**,
**loadout**, and the **bot brain** — behind a small `GameMode` trait (`src/mode/`). Free-for-all
is just the baseline mode, so adding a mode doesn't touch the generic gameplay or bot code.

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_mode` | `ffa` | `ffa` = free-for-all deathmatch (stock behaviour). `ra` = Rocket Arena. `midair` = airborne-only rocket DM. `ctf` = Capture the Flag. A **team format** (`1on1`/`duel`, `2on2`, `2on2on2`, any `NonM…`) = team deathmatch. |
| `rtx_ra_countdown` | `3` | Rocket Arena: seconds of spawn-protected countdown before "FIGHT". |
| `rtx_ra_lightning_gun` | `0` | Rocket Arena: include the lightning gun in the arena arsenal (`0` leaves it out). |
| `rtx_midair_minheight` | `40` | Midair: minimum height (units) above the floor for a victim to count as airborne. |
| `rtx_midair_kb_ground` / `rtx_midair_kb_air` | `6` / `3` | Midair: rocket knockback multipliers for grounded vs airborne victims (ground is stronger, to launch players up). |
| `rtx_match_countdown` | `3` | Team match / CTF: seconds of spawn-protected countdown after the match-start map reload before "FIGHT". |
| `rtx_capturelimit` | `8` | CTF: captures a team needs to win (`0` = no limit, ends on `timelimit`). |
| `rtx_runes` | `0` | CTF runes: `0` = on (Haste adds move speed), `1` = off, `2` = on without the speed boost. |
| `rtx_ctf_tossflag` / `rtx_ctf_tossrune` | `0` / `0` | CTF: allow tossing your carried flag (impulse 26) / held rune (impulse 24). |
| `rtx_dropitems` | `0` | Any mode: let players hand items to teammates — a **capped ammo backpack** (impulse 20; up to 20 shells / 20 nails / 10 rockets / 20 cells, deducted from you) and your **current weapon** (impulse 21; drops it as a pickup and switches you to your next-best gun — the axe, single shotgun, and grapple stay). Ported from purectf. |

**`ra` — Rocket Arena.** Round-based 1v1 duels following the classic arena loop (ported from the
Frogbot-Rocket-Arena QuakeC, minus its clan-arena team machinery). Two players fight in the arena
at a time; everyone else waits in the **audience** (the `info_player_deathmatch` spots — the
stands) and roams there. Each round the fighters spawn with a
**full loadout** (all weapons, full ammo, red armour) **inside the arena** (the
`info_teleport_destination` spots), are invulnerable through a short countdown, then fight. During
the countdown you can move to position but **can't fire yet** (a screen blink if you try); at
"FIGHT" weapons go hot. Getting killed **drops you to the audience**; the **winner stays** — kept
in place and topped back up to full — and faces the next challenger pulled from the front of the
audience queue (losers go to the back), so the arena is always a fresh duel. On a plain deathmatch
map with no teleport destinations it falls back to DM spawns so the mode still runs. Bots play it
fully — see below.

**`midair` — airborne-only rocket DM** (modeled on [KTX](https://github.com/QW-Group/ktx)'s
midair). Everyone spawns with a **rocket launcher** (+ axe), 255 rockets, red armour and 250
health, at normal gravity. A direct rocket on an **airborne** victim is an **instant kill**; on a
**grounded** victim it deals no damage but delivers a hard **knockback that launches them skyward**
— so you rocket someone up, then airshot them out of the air. Non-rocket damage is harmless, and
your own rockets never hurt you but still fling you (free rocket-jumps). Kills score by **how high
the victim was** (vertical distance from where you fired): **bronze/silver/gold/platinum** for
**+1/+2/+4/+8** at `>0/256/512/1024` units, announced as an airshot line. Bots play it — they hunt
the nearest player, launch grounded targets and airshot them.

**Team formats — `1on1`/`duel`, `2on2`, `2on2on2`, any `NonM…`.** A generic team-match layer: the
alias picks **N teams of size M** (`2on2on2` = three teams of two). It's a **continuous team
deathmatch** — teams frag to the `fraglimit`, friendly fire follows `teamplay`, and each team gets a
colour (red/blue/green/…) and its `info_player_teamN` spawns (DM spawns as fallback). The lifecycle
is warmup → **`start`** → live → results: in **warmup** everyone plays and is auto-balanced onto the
smallest team; typing **`start`** in the console **reloads the map** (fresh entities) and runs a
countdown, **locking the roster**; play then runs to the limit and returns to warmup. Players who
drop and reconnect are **reattached to their team**. Bots fill and play the teams, targeting only
the other side. The team primitives are reusable — CTF (below) is the second consumer, and a future
round-based team mode builds on the same layer.

**`ctf` — Capture the Flag** (modeled on **purectf**), built on the team layer above. Two teams
(red/blue) each own a flag at a base (`item_flag_team1`/`item_flag_team2`). Grab the **enemy** flag,
carry it to **your** base while **your** flag is home, and it's a **capture** (+1 to your team, +15
frags to the carrier). Touch your **own** flag where it lies to **return** it (+1); a dropped flag
also **auto-returns** after 40 s, and a killed carrier **drops** it where they fell. Teams win at
`rtx_capturelimit`. It uses the same match lifecycle as team DM — warmup → **`start`** (map reload +
countdown) → live → results — with friendly fire via `teamplay`, the grapple handed out for movement,
and CTF bots that **split into roles**: most attack (grab the enemy flag and run it home), while a
minority (about a third, at least one) defend — holding the base, intercepting attackers that close
on it, chasing down whoever steals the flag, and re-touching a dropped flag to return it. A carrier
always runs home regardless of role. Full purectf scoring is in:
capture +15, teammates +10, plus the frag-carrier, carrier-protect, flag-defense, and return/frag
**assist** bonuses. The four **runes** (Resistance, Strength, Haste, Regeneration — `rtx_runes`) spawn
at DM points, one per player, dropped on death; the flag and rune can be tossed (`rtx_ctf_tossflag`
/ `rtx_ctf_tossrune`, impulse 26 / 24). CTF requires the flag model **`progs/flag.mdl`** (and the
rune models `progs/end1-4.mdl`) in the gamedir.

## Bots

Navmesh-driven bots that need **no per-map waypoint files** — the navmesh is generated from the
map's BSP clip hull when the map loads. Bots are real client slots: the engine runs their input
through the same player-move code as humans, so gravity, stepping, and jumps come for free.

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_bots` | `0` | How many bots to keep on the server. The population is reconciled to this count (spawning/removing as needed), leaving room for humans. Bots only spawn once the map's navmesh is built. |
| `rtx_bot_skill` | `3` | Bot skill (0–7): tightens aim and speeds how fast a bot turns/tracks. |

In free-for-all each bot pathfinds to the best reachable **item pickup**, or **follows the nearest
human** when nothing's worth fetching (through doors, off ledges, across jumps, recovering after a
missed jump). On open, roughly-straight stretches they **bunnyhop** (`rtx_bot_bhop`) — chaining jumps
and **air-strafing** (sweeping the view while holding one strafe key) to exploit QuakeWorld's
air-acceleration and build speed far past `sv_maxspeed`, weaving the heading toward the waypoint;
they drop back to a normal gait for corners, ledges, combat, or the final approach to a goal. That
speed unlocks **speed jumps** — the navmesh links **gaps too wide for any normal or double jump**,
cleared by arriving at the takeoff with built-up bhop speed (a jump's reach = speed × airtime, and
airtime is fixed, so faster = farther). Each such link's start is the *runway* itself, so a bot that
takes it is guaranteed to run the whole accelerating approach before the leap — and it refuses to
launch if it somehow reaches the edge too slow. These are the only way across a wide gap when the
double jump is off. When `rtx_doublejump` is on, the navmesh also links the **wider gaps and higher
ledges
a double jump reaches** — the bot ground-jumps, then **air-jumps near the apex** to restack the arc
and clear a gap a single jump can't (it also spends the air jump to recover an undershot ordinary
jump). When `rtx_grapple` is on, the navmesh also grows **hook links** — edges a bot crosses
with the **grappling hook**: it throws the hook at an anchor, reels to build speed, then **releases
mid-reel so the resulting velocity flings it along a parabola** onto a ledge or across a gap a plain
jump can't reach (a straight pull-up is just the degenerate case). Because the arc is deterministic,
the links are found and verified when the map's navmesh is built by simulating the swing against the
BSP, and A* prices them as travel time — so bots take a hook only when it beats the ground route.
This measurably widens where bots can go on vertical/CTF maps. The mode can redirect the brain
without touching it: in **Rocket Arena** bots
**fight** — they path to the nearest enemy and, once they have line of sight, aim (leading the
target for rockets), pick a weapon by range, strafe/retreat, and fire — and, when eliminated, roam
the audience like everyone else. The combat layer (`src/bot_combat.rs`) is generic and reused by
any mode that hands a bot an enemy. A bot's view **lerps** toward its target angle rather than
snapping, so it turns naturally when spectated; both the turn/track speed and aim tightness scale
with `rtx_bot_skill` (a low-skill bot visibly swings onto a target more slowly).

Bots also play the **shootable-grenade** game (above). Defensively they shoot down an **incoming**
grenade — but only from outside its blast, weighed against their own health (the closer it is, the
more health it takes to justify setting it off) — and a grenade too close to safely pop makes them
**run and hop clear** instead of detonating it in their own face. Offensively they use splash weapons for
**position manipulation** — if an enemy stands near **lava, slime, a pit or a ledge**, the bot sets
off a blast so the outward **knockback shoves them into the hazard**, verifying the shove actually
carries them across the edge before committing. It's a **generic strategy** — the blast point sits on the ground **behind** the enemy (away from the
hazard) so the outward splash drives them in — with two deliveries: a **rocket** put straight onto
that ground spot (no direct hit needed; a static point is easy to hit and works from any angle with a
clear line to it), or a **grenade lob→shoot combo** when the blast must be **arced over** the enemy
to reach it — aim a ballistic arc (solved from the launcher's fixed speed/loft against gravity), lob,
switch to a hitscan gun, and detonate in flight. With no hazard the grenade combo becomes a plain
airburst. All of it is safety-checked — never self-splash, never a
teammate, never a shove the wrong way, never a lob into a wall.

They also throw **indirect bank shots** at enemies with **no line of sight**: a solver simulates a
bouncing grenade against the map's collision hull (a real `SV_RecursiveHullCheck` trace, reflecting
off surface normals with QuakeWorld's `MOVETYPE_BOUNCE` physics), searching launch angles for one
whose ricochet path reaches the hidden enemy — then lobs it and lets the **2.5 s fuse** detonate it
around the corner (a launch-jitter robustness sweep rejects knife-edge angles the throw's spread
would spoil). It's gated to stay honest and rare — a recently-seen, slow target and higher bot skill
— with flag carriers worth a blind lob past the throttle.

## Map rotation

Set **`rtx_maplist`** to a whitespace-separated list of maps and the server cycles through them **in
order** each time a level ends (on `timelimit`/`fraglimit`, or when a team match finishes):

```
set rtx_maplist "dm2 dm3 dm4 dm6 aerowalk"
```

The next map is the one after the current map in the list (wrapping around; if the current map isn't
listed, the rotation starts at the first entry). It takes precedence over a serverinfo `nextmap`.
When a list is configured the end-of-level intermission scoreboard **auto-advances** after its pause
instead of waiting for a player to press a button; with no list set, the stock behaviour is
unchanged. Leave `rtx_maplist` empty (the default) to disable rotation.

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
