# Game Modes & Match Composition

How a server picks its ruleset and how the match is organized — two orthogonal axes, each a
cvar, plus map rotation.

Part of the [rtx manual](../README.md) · every tunable on one page: [cvar reference](cvars.md)

## The two axes

The **game mode** (`rtx_mode`) is the ruleset; the **match composition** (`rtx_match`) is how
the match is organized. They're independent: deathmatch and CTF each run under any composition
(open free-for-all, or a locked `2on2`/`4on4`/…), while Rocket Arena and midair are inherently
duel rulesets.

Modes are pluggable: each one overrides only the policy it changes — **ruleset** (round/damage/
respawn rules), **spawns**, **loadout**, and the **bot brain** — behind a small `GameMode` trait
(`crates/rtx-game/src/mode/`). Deathmatch is just the baseline mode, so adding a mode doesn't
touch the generic gameplay or bot code.

## At a glance

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_mode` | `dm` | Ruleset: `dm` = deathmatch (stock behaviour). `ra` = Rocket Arena. `midair` = airborne-only rocket DM. `ctf` = Capture the Flag. `race` = timed KTX race routes. |
| `rtx_match` | *(auto)* | Composition, orthogonal to the mode. Empty = the mode's natural default (dm → open FFA, ctf → open 2-team pickup, midair → 1on1 duel). `ffa` = open free-for-all. A **team format** (`1on1`/`duel`, `2on2`, `2on2on2`, any `NonM…`) = a locked N×M match. Ignored by `ra`; CTF clamps it to 2 teams. |
| `rtx_ra_countdown` | `3` | Rocket Arena: seconds of spawn-protected countdown before "FIGHT". |
| `rtx_midair_minheight` | `40` | Midair: minimum height (units) above the floor for a victim to count as airborne. |
| `rtx_midair_kb_ground` / `rtx_midair_kb_air` | `6` / `3` | Midair: rocket knockback multipliers for grounded vs airborne victims (ground is stronger, to launch players up). |
| `rtx_match_countdown` | `3` | Team match (`rtx_match` format) / CTF: seconds of spawn-protected countdown after the match-start map reload before "FIGHT". |
| `rtx_capturelimit` | `8` | CTF: captures a team needs to win (`0` = no limit, ends on `timelimit`). |
| `rtx_race_route` | `0` | Race: which of the current map's routes to run (0-based, clamped). Read live — changing it moves everyone to the new route's start. |
| `rtx_runes` | `0` | CTF runes: `0` = on (Haste adds move speed), `1` = off, `2` = on without the speed boost. |
| `rtx_ctf_tossflag` / `rtx_ctf_tossrune` | `0` / `0` | CTF: allow tossing your carried flag (impulse 26) / held rune (impulse 24). |
| `rtx_dropitems` | `0` | Outside Race: let players hand items to teammates (see [dropping items](#dropping-items)). |
| `rtx_maplist` | *(empty)* | Whitespace-separated map rotation (see [map rotation](#map-rotation)). |

## `dm` — deathmatch

The baseline: stock QuakeWorld deathmatch, with the [movement additions](movement.md) layered
on. By default an open free-for-all; give `rtx_match` a team format for team deathmatch — teams
frag to the `fraglimit`.

## `ra` — Rocket Arena

Round-based 1v1 duels following the classic arena loop (ported from the Frogbot-Rocket-Arena
QuakeC, minus its clan-arena team machinery).

Two players fight in the arena at a time; everyone else waits in the **audience** (the
`info_player_deathmatch` spots — the stands) and roams there. Each round the fighters spawn with
a **full loadout** (all weapons, full ammo, red armour) **inside the arena** (the
`info_teleport_destination` spots), are invulnerable through a short countdown, then fight.
During the countdown you can move to position but **can't fire yet** (a screen blink if you
try); at "FIGHT" weapons go hot.

Getting killed **drops you to the audience**; the **winner stays** — kept in place and topped
back up to full — and faces the next challenger pulled from the front of the audience queue
(losers go to the back), so the arena is always a fresh duel. On a plain deathmatch map with no
teleport destinations it falls back to DM spawns so the mode still runs. Bots play it fully —
see [the bots](bots.md#modes-and-the-brain).

## `midair` — airborne-only rocket DM

Modeled on [KTX](https://github.com/QW-Group/ktx)'s midair. Everyone spawns with a **rocket
launcher** (+ axe), 255 rockets, red armour and 250 health, at normal gravity.

A direct rocket on an **airborne** victim is an **instant kill**; on a **grounded** victim it
deals no damage but delivers a hard **knockback that launches them skyward** — so you rocket
someone up, then airshot them out of the air. Non-rocket damage is harmless, and your own
rockets never hurt you but still fling you (free rocket-jumps).

Kills score by **how high the victim was** (vertical distance from where you fired):
**bronze/silver/gold/platinum** for **+1/+2/+4/+8** at `>0/256/512/1024` units, announced as an
airshot line.

By default midair runs as a **1on1 duel** (a structured match — see below); set `rtx_match ffa`
for a continuous free-for-all, or a team format like `2on2`. Bots play it — they hunt the
nearest enemy, launch grounded targets and airshot them.

## Match composition — `rtx_match`

Orthogonal to the mode, this picks how the match is organized. Empty (the default) uses the
mode's natural composition; `ffa` forces open free-for-all; a **team format** (`1on1`/`duel`,
`2on2`, `2on2on2`, any `NonM…`) picks **N teams of size M** (`2on2on2` = three teams of two) and
applies to deathmatch and CTF alike.

A team composition gives each team a colour (red/blue/green/…) and its `info_player_teamN`
spawns (DM spawns as fallback), turns on friendly fire (`teamplay`), and — for a plain **team
deathmatch** — has teams frag to the `fraglimit`.

The lifecycle is warmup → **`start`** → live → results:

- In **warmup** everyone plays and is auto-balanced onto the smallest team.
- Typing **`start`** in the console **reloads the map** (fresh entities) and runs a countdown,
  **locking the roster**.
- Play then runs to the limit and returns to warmup.

A **structured** format (`teams × size`) locks **exactly that many seats** at `start` (humans
before bots, rebalancing the sides); anyone over the count — and any **late joiner** — is
**benched** as a harmless spectator (axe only, damage refused, roaming the stands) until the
next warmup. Bots fill exactly the empty seats and freeze once the match is live. Players who
drop and reconnect are **reattached to their team**. Bots target only the other side.

## `ctf` — Capture the Flag

Modeled on **purectf**, built on the composition layer above. Two teams (red/blue) each own a
flag at a base (`item_flag_team1`/`item_flag_team2`).

Grab the **enemy** flag, carry it to **your** base while **your** flag is home, and it's a
**capture** (+1 to your team, +15 frags to the carrier). Touch your **own** flag where it lies
to **return** it (+1); a dropped flag also **auto-returns** after 40 s, and a killed carrier
**drops** it where they fell. Teams win at `rtx_capturelimit`.

It uses the same match lifecycle as team DM — warmup → **`start`** (map reload + countdown) →
live → results — with friendly fire via `teamplay`, the grapple handed out for movement, and CTF
bots that **split into roles**: most attack (grab the enemy flag and run it home), while a
minority (about a third, at least one) defend — holding the base, intercepting attackers that
close on it, chasing down whoever steals the flag, and re-touching a dropped flag to return it.
A carrier always runs home regardless of role. See [teamwork & CTF](bots.md#teamwork--ctf) for
the full role machinery.

Full purectf scoring is in: capture +15, teammates +10, plus the frag-carrier, carrier-protect,
flag-defense, and return/frag **assist** bonuses.

The four **runes** (Resistance, Strength, Haste, Regeneration — `rtx_runes`) spawn at DM points,
one per player, dropped on death; the flag and rune can be tossed (`rtx_ctf_tossflag` /
`rtx_ctf_tossrune`, impulse 26 / 24).

CTF requires the flag model **`progs/flag.mdl`** (and the rune models `progs/end1-4.mdl`) in the
gamedir.

## `race` — timed KTX race routes

Run a course from its **start pad** through its **checkpoints** to the **finish**, timed per
runner — and, first and foremost, a **sanity harness** for the bot movement system: most race
routes are unfinishable without bunnyhop-accumulated speed, so a bot finishing (or **timing out
on a named leg**) is a live regression check on the speed-jump / bhop machinery.

Routes load from two sources, exactly as [KTX](https://github.com/QW-Group/ktx) does:
`race_route_start` / `race_route_marker` **entities embedded in the map** (race11–20, race32c),
and external **`race/routes/{mapname}.route`** command files (race1–10, ztricks, ztricks2 — copy
KTX's examples into the gamedir).

Everyone spawns on the active route's start pad (**axe only**, KTX `raceWeaponNo`); the clock
starts when the runner leaves the start box, checkpoints must be touched **in order**, the
finish broadcasts the time, and `race_route_timeout` without finishing resets the run. Deaths
and manual toss commands never leave backpacks or dropped weapons on the course, and a bot
always keeps the next checkpoint/finish as its hard objective.

Because race maps are authored for **stock movement + bunnyhop only**, the mode reports
`stock_movement_only`: double jump, wall jump, elevator jump, grapple and rocket jumps are
switched **off** — both as live mechanics and as navmesh links — so bots must reach everything
by bhop and speed jumps, and a failed pathfind is honest.

At map load the mode prints a **routability report**, one `PASS`/`FAIL` line per route,
answering whether every leg is traversable with race-legal movement (needs `rtx_bot_bhop 1` for
the speed-jump links; it warns if that's off). Switch routes live with `rtx_race_route`.

Speed carried between jumps is priced by the speed-band planner (`rtx_bot_bandplan`), so chained
speed jumps route. **Known gap:** routes that gain height via a **slope launch** (hitting an
angled ramp at tremendous bhop speed, the engine redirecting horizontal velocity upward) are not
yet modelled in the navmesh — the routability report names the exact legs — so those `FAIL`
today.

## Dropping items

With `rtx_dropitems 1`, outside Race, players can hand items to teammates — a **capped ammo
backpack** (impulse 20; up to 20 shells / 20 nails / 10 rockets / 20 cells, deducted from you)
and your **current weapon** (impulse 21; drops it as a pickup and switches you to your next-best
gun — the axe, single shotgun, and grapple stay). Race never creates dropped items. Ported from
purectf.

## Map rotation

Set **`rtx_maplist`** to a whitespace-separated list of maps and the server cycles through them
**in order** each time a level ends (on `timelimit`/`fraglimit`, or when a team match finishes):

```
set rtx_maplist "dm2 dm3 dm4 dm6 aerowalk"
```

The next map is the one after the current map in the list (wrapping around; if the current map
isn't listed, the rotation starts at the first entry). It takes precedence over a serverinfo
`nextmap`. When a list is configured the end-of-level intermission scoreboard **auto-advances**
after its pause instead of waiting for a player to press a button; with no list set, the stock
behaviour is unchanged. Leave `rtx_maplist` empty (the default) to disable rotation.

---

*See also: [movement & combat](movement.md) · [the bots](bots.md) · [cvar reference](cvars.md)*
