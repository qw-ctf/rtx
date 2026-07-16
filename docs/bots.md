# The Bots

Navmesh-driven bots that need **no per-map waypoint files** — the navmesh is generated from the
map's BSP clip hull when the map loads. Bots are real client slots: the engine runs their input
through the same player-move code as humans, so gravity, stepping, and jumps come for free.

Part of the [rtx manual](../README.md) · every tunable on one page: [cvar reference](cvars.md)

## At a glance

| cvar | default | what it does |
|------|---------|--------------|
| `rtx_bot_count` | `0` | How many bots to keep on the server. The population is reconciled to this count (spawning/removing as needed), leaving room for humans. Bots only spawn once the map's navmesh is built. |
| `rtx_bot_alone` | `0` | Keep bots on the server even when **no humans** are connected (`0` = bots leave an empty server; `1` = they stay and play it out). |
| `rtx_bot_skill` | `3` | Bot skill (0–7): tightens aim, speeds how fast a bot turns/tracks, widens its view cone, and shortens its reaction time. |
| `rtx_bot_pacifist` | `0` | Make bots **not fight** outside Race — they just trail the nearest human around the map (for experimenting). Race bots always follow their checkpoint/finish route, which is already non-combat. |
| `rtx_bot_greed` | `1` | Let a fighting bot take **optional ordinary item detours** — a missing weapon or worthwhile health/armor swing. Critical local recovery and major objectives (quad/pent/ring and CTF runes) are never disabled by this cvar. |
| `rtx_bot_fov` | `120` | View cone (full angle, degrees) within which a bot can **see** a target; widened with skill. `0` = 360° sight. |
| `rtx_bot_reaction` | `0.4` | Base **reaction delay** (seconds) a target must stay in sight before the bot acts on it; shortened with skill (floored so even skill 7 isn't instant). `0` = react instantly. |

The finer-grained movement, item, and modeling switches (`rtx_bot_bhop`, `rtx_bot_rocketjump`,
`rtx_bot_stack`, `rtx_bot_model`, …) are covered in context below and listed in the
[cvar reference](cvars.md#bots--movement).

## What a bot does

In open play each bot **hunts and frags the nearest player** — everyone's an enemy, so a
bots-only server plays itself — pathing to them and, once in sight, aiming and shooting via the
shared combat layer (retreating when hurt, grabbing items it passes over). Set
`rtx_bot_pacifist 1` and, outside Race, they stop fighting and just tail the nearest human
instead. With nothing to chase and no human to follow, a bot **roams** to a random reachable
spot rather than standing on its spawn.

When a mode leaves the brain in charge, it pathfinds to the best reachable **item pickup**, or
**follows the nearest human** — through doors, off ledges, across jumps, recovering after a
missed jump.

## Perception

A bot doesn't fight a target it hasn't actually **perceived**. An enemy has to be:

- **seen** — inside the bot's view cone (`rtx_bot_fov`), with line of sight, held for a
  `rtx_bot_reaction` beat,
- **heard** — firing nearby, or
- **felt** — as incoming damage

before the bot engages it. So instead of psychically beelining an unseen enemy it patrols and
collects until real contact, and when it loses sight it **hunts the last spot it saw them** for
a few seconds before giving up rather than tracking them through walls. A nominated enemy
outside the cone (or behind cover) isn't engaged until seen, heard firing, or felt as damage.

Aim is loosest on first glimpse and while moving, tightening as the bot holds a target in view.
A bot's view **lerps** toward its target angle rather than snapping, so it turns naturally when
spectated; both the turn/track speed and aim tightness scale with `rtx_bot_skill` (a low-skill
bot visibly swings onto a target more slowly).

## Navigation

### Loop resistance

Navigation is **loop-resistant**: when a bot fails to traverse a link (a jump it keeps
undershooting, a spot it wedges on, a leg that makes no headway toward the goal) that link gets
a temporary per-bot cost penalty, so its next path **routes around** the trouble instead of the
planner handing back the identical dead route to retry until a timeout fires. A dash of per-bot
path jitter also keeps two bots from treading an identical line.

### Committed jumps

Gap/double/speed-jump traversal is an **indivisible action**: it is armed before combat
arbitration, freezes route advancement while airborne, and releases only after a physical
landing — so spotting an enemy at the lip cannot make a bot turn and fall into the gap.

### Bunnyhop & air-strafe

On open, roughly-straight stretches bots **bunnyhop** (`rtx_bot_bhop`) — chaining jumps and
**air-strafing** (sweeping the view while holding one strafe key) to exploit QuakeWorld's
air-acceleration and build speed far past `sv_maxspeed`, weaving the heading toward the
waypoint. They drop back to a normal gait for corners, ledges, combat, or the final approach to
a goal. On straight corridors too short to hop they **ground-zigzag** (circle-strafe) instead
(`rtx_bot_zigzag`, a sub-toggle of bhop).

Route planning runs over **speed bands** (`rtx_bot_bandplan`, a kinodynamic A*): speed carried
between legs is credited, so chained speed jumps and hot corridors route. `0` falls back to
plain A*.

### Speed jumps

Bhop speed unlocks **speed jumps** — the navmesh links **gaps too wide for any normal or double
jump**, cleared by arriving at the takeoff with built-up bhop speed (a jump's reach =
speed × airtime, and airtime is fixed, so faster = farther). Each such link's start is the
*runway* itself, so a bot that takes it is guaranteed to run the whole accelerating approach
before the leap — and it refuses to launch if it somehow reaches the edge too slow. These are
the only way across a wide gap when the double jump is off.

### Double-jump links

When `rtx_doublejump` is on, the navmesh also links the **wider gaps and higher ledges a double
jump reaches** — the bot ground-jumps, then **air-jumps near the apex** to restack the arc and
clear a gap a single jump can't. It also spends the air jump to recover an undershot ordinary
jump.

### Curl jumps

With `rtx_bot_curljump` (off by default), the navmesh also generates **curl jumps** — a run-up
down a corridor, then an air-turn onto an offset platform — each certified by a pmove rollout in
the navmesh build. A sub-toggle of bhop (`rtx_bot_bhop 0` disables it too).

### Hook links

When `rtx_grapple` is on, the navmesh also grows **hook links** — edges a bot crosses with the
**grappling hook**: it throws the hook at an anchor, reels to build speed, then **releases
mid-reel so the resulting velocity flings it along a parabola** onto a ledge or across a gap a
plain jump can't reach (a straight pull-up is just the degenerate case). Because the arc is
deterministic, the links are found and verified when the map's navmesh is built by simulating
the swing against the BSP, and A* prices them as travel time — so bots take a hook only when it
beats the ground route. This measurably widens where bots can go on vertical/CTF maps.

### Rocket jumps

With `rtx_bot_rocketjump` (on by default), bots **rocket-jump** to ledges that would otherwise
need a long detour — or are unreachable. A rocket jump costs health, so a bot only plans one
when it clearly beats the walk and it's fit to fly it: it has the rocket launcher, a rocket, and
the health to spare.

### Hazards

Bots price lava and slime into their routes. A bot's health weights how willing it is to
shortcut through a hazard (`rtx_bot_hazard_health`): hurt bots detour, healthy (or armored, or
biosuited) ones clip the corner; `rtx_bot_hazard_k` sets how many seconds of detour a bot
accepts per unit of survival strength a hazard would eat (higher = more timid).

## Items & resources

Bots value pickups the way ktx does: each item's **desire** is the marginal effective-HP
(health, armor), firepower (weapons, ammo), or flat dominance (powerups/runes) it would give
*this* bot now, weighted by travel and respawn time.

The planner evaluates a bounded **two-pickup sequence**, so a nearby useful item can become the
first stop on the way to armor or quad; the second stop is promoted and revalidated after the
first touch rather than followed blindly.

A perceived opponent's estimated need adds **denial value**, while a weaker bot yields an
ordinary contest the enemy reaches first. Bots skip health at full, owned weapons, capped ammo,
and anything that cannot be collected before a timed match ends, while treating **quad, pent,
ring, and CTF runes** as completion-critical objectives. While timing a hidden quad/pent/ring,
the bot may insert a nearby spawned health, armor, or weapon only when the complete detour still
reaches the powerup on time — turning respawn wait into useful preparation instead of walking
past yellow armor to idle.

When a bot reaches an item that hasn't respawned yet, it doesn't stand and twitch on the spot —
it **cruises** a short walk around the spawn, panning the view to **scan for enemies** (which
genuinely widens what it can see), and heads back to stand on the point just as the item
returns. Dropped **backpacks** (a dead player's weapon + ammo, or a teammate's toss) are sought
and collected the same way.

**Resource discipline** (`rtx_bot_stack`, on by default) values health/armor more steeply below
the bare-spawn stack, enters the Recover posture on a thin stack (not just low health), treats
red armor and megahealth as major "must-cycle" pickups, and panics for ammo when a bot's
firepower is about to collapse. Off = the leaner ktx-parity valuation, where a topped-up bot
ignores items until a true need.

**Waypoint magnetism** (`rtx_bot_magnet`, on by default) bends the immediate steering waypoint
through a desirable item lying just off the route corridor, so the bot actually steps onto it.

`rtx_bot_greed` controls only *additional ordinary combat detours* — critical recovery and major
objectives are never gated by it.

## Combat

In combat, a hysteretic **Recover / Hold / Press** posture compares effective strength and
firepower using observation-gated opponent estimates (see [opponent modeling](#opponent-modeling)).
A weak or critically hurt bot commits movement to reachable health, armor, or a needed weapon;
combat may keep aiming/firing but cannot strafe it off the final pickup. The same lock protects
a powerup plan, and a universal fake-client pickup pass covers flags and runes as well as
ordinary items.

Once a bot has line of sight it aims (leading the target for rockets), picks a weapon by range,
strafes/retreats, and fires. The combat layer (`crates/rtx-game/src/bot_combat.rs`) is generic
and reused by any mode that hands a bot an enemy.

### Splash self-care

Explosive fire projects the bot's own post-armor health loss using the live quad multiplier
(4×, or 8× in deathmatch 4) and CTF rune scaling. Quad rockets/grenades use KTX's conservative
**250-unit self-splash caution zone**: the bot switches to a direct gun when possible and the
final trigger gate withholds a dangerous shot if it cannot, while pent/god mode and Midair
self-rockets stay exempt.

### The shootable-grenade game

Bots play the [shock combo](movement.md#the-shock-combo--shootable-grenades) from both sides.

**Defensively** they shoot down an **incoming** grenade — but only from outside its blast,
weighed against their own health (the closer it is, the more health it takes to justify setting
it off) — and a grenade too close to safely pop makes them **run and hop clear** instead of
detonating it in their own face.

**Offensively** they use splash weapons for **position manipulation** — if an enemy stands near
**lava, slime, a pit or a ledge**, the bot sets off a blast so the outward **knockback shoves
them into the hazard**, verifying the shove actually carries them across the edge before
committing. It's a **generic strategy** — the blast point sits on the ground **behind** the
enemy (away from the hazard) so the outward splash drives them in — with two deliveries:

- a **rocket** put straight onto that ground spot (no direct hit needed; a static point is easy
  to hit and works from any angle with a clear line to it), or
- a **grenade lob→shoot combo** when the blast must be **arced over** the enemy to reach it —
  aim a ballistic arc (solved from the launcher's fixed speed/loft against gravity), lob, switch
  to a hitscan gun, and detonate in flight.

With no hazard the grenade combo becomes a plain airburst. All of it is safety-checked — never
self-splash, never a teammate, never a shove the wrong way, never a lob into a wall.

### Projectile evasion

Projectile survival does not depend on shootable grenades: bots predict the closest approach of
live **rockets and nails**, sample the next part of a grenade's gravity arc and fuse, respect
intervening world collisions, and take a hazard-checked lateral escape when the damage tube
intersects them. Higher skill notices the same geometry earlier. During a visible duel the
normal strafe is also biased away from the opponent's current aim line. These evasions cannot
interrupt a committed gap jump or the final approach to a critical pickup.

### Bank shots

Bots throw **indirect bank shots** at enemies with **no line of sight**: a solver simulates a
bouncing grenade against the map's collision hull (a real `SV_RecursiveHullCheck` trace,
reflecting off surface normals with QuakeWorld's `MOVETYPE_BOUNCE` physics), searching launch
angles for one whose ricochet path reaches the hidden enemy — then lobs it and lets the
**2.5 s fuse** detonate it around the corner. A launch-jitter robustness sweep rejects
knife-edge angles the throw's spread would spoil. It's gated to stay honest and rare — a
recently-seen, slow target and higher bot skill — with flag carriers worth a blind lob past the
throttle.

### Opponent modeling

With `rtx_bot_model` (on by default), bots keep a shared, observation-gated hypothesis of each
opponent's health/armor stack and arsenal — per-team blackboards; the FFA bots share one — reset
on death and updated only from events a bot could witness (pickups and gunfire in earshot,
damage it dealt). It feeds item denial, target selection, and combat risk. `0` = the old
estimate-free behavior. (Team coordination itself is unconditional in a team composition; this
switch controls only the inferred enemy model.)

## Teamwork & CTF

In team and CTF modes coordination is always active: a team **spreads its fire** across the
enemy side, gives each pickup a stable nearest-bot reservation, and sends only the nearest
responder to a teammate's recent damage call (two for a flag carrier). Bots avoid ally splash
and stagger routes around occupied teleporter exits.

CTF teams split into stable **attack, midfield, and defense** roles: attackers can detour for
major items, midfielders control powerups/runes and crossings, defenders spread around the base,
and escorts peel for the carrier.

## Modes and the brain

The mode can redirect the brain without touching it. In **Rocket Arena** bots **fight** — they
path to the nearest enemy and duel with the full combat layer — and, when eliminated, roam the
audience like everyone else. In **Race** a bot always keeps the next checkpoint/finish as its
hard objective. See [game modes](modes.md) for what each mode asks of them.

---

*See also: [movement & combat](movement.md) · [game modes](modes.md) ·
[the bots as network clients](netclient.md) · [cvar reference](cvars.md)*
