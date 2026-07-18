# Bot Architecture

**How the navmesh bots are built inside** — the navigation core, the per-frame decision loop, the
movement drivers, the opponent model, and the two hosts the same brain runs under. Where
[the bots](bots.md) is the field manual for *what* a bot does, this is the map of *how* it is
wired: the files, the types, the algorithms, and the numbers.

Part of the [rtx manual](../README.md) · what they do: [the bots](bots.md) · the client
embodiment: [network clients](netclient.md) · every tunable: [cvar reference](cvars.md)

## The shape of the system

A bot is a **real client edict**. The engine runs its input through the same `SV_RunCmd` /
`PM_PlayerMove` a human's input goes through, so gravity, friction, stepping, and jumps come for
free — the brain never simulates physics, it only chooses a usercmd (view angles, a move vector,
the jump/attack buttons, a weapon impulse) and hands it over. Everything below is how that one
choice gets made, ~77 times a second, per bot.

The same brain runs in **two hosts**. Inside the server it drives fake-client edicts; as
`rtx-client` it connects to a remote server over UDP as an ordinary network client. It is the
same `run_bots` over the same `GameState`, and *the code cannot tell which host it is under*. The
seam is the [`ClientHost`](#the-two-embodiments) trait plus one discipline — **write network truth
into exactly the fields the brain already reads** — so the brain never learns a second way to ask
a question:

```text
  the server module                      the network client
  ─────────────────                      ──────────────────
  engine fills EntVars        ──▶         mirror writes EntVars from svc_playerinfo /
                                          svc_packetentities / stats
  engine answers traceline,   ──▶         NetHost answers from the map's own BSP
    pointcontents, cvars                  (rtx-nav) and its own cvar store
  set_bot_cmd → SV_RunCmd      ──▶         cmd sink → clc_move on the wire
  server runs trigger touches  ──▶         the server does it for us (we are a real client)
```

### Where the code lives

| crate / module | role |
|----------------|------|
| `crates/rtx-game/src/bot/` | The bot brain — perception, goals, steering, combat, the movement drivers, the opponent model. 16 files. |
| `crates/rtx-game/src/netclient/` | The second host: the brain embodied as a UDP client (`netclient` cargo feature, default-off). |
| `crates/rtx-nav/` | The pure navigation core — BSP clip-hull reader, navmesh build & query, movement physics. No engine or game state; deterministic math that unit-tests without a host. |
| `crates/rtx-proto/` | QuakeWorld + NetQuake wire protocols as pure codecs. |
| `crates/rtx-client/` | The thin front-door binary that parses argv and calls `netclient::run`. |
| `crates/navview/` | A wgpu/winit 3D viewer for the generated navmesh. |
| `crates/rjmcp/` | An MCP bridge onto the control channel, for live rocket-jump tuning. |

The split between `rtx-game` and `rtx-nav` is the load-bearing one. `rtx-nav` sees only the parsed
BSP and pure physics; it knows nothing about entities, items, or clients. That purity is what lets
the navmesh build off the main thread, unit-test in isolation, and be shared verbatim by the
viewer and both bot hosts.

### The per-frame pipeline

The engine calls the module **once per bot frame for the whole squad**, not once per bot:
`run_bots` (`bot/mod.rs`) loops the in-use bot edicts and calls `run_bot` on each. One `run_bot`
is one bot's entire think for the frame, and it runs these stages in order:

```text
  sense ─▶ death/respawn ─▶ prearm_traversal ─▶ resolve_objective ─▶ weapons_hot
    │                            (lock a         (perceive + goals      (mode
    │                          committed jump)     + vigil)             lockout)
    ▼
  bot_link_pricing ─▶ plat_statuses ─▶ drown/burn override ─▶ race_line ─▶ steer
    (A* surcharges,                     (reflex goal hijack)              (route →
     incl. RJ fitness)                                                    command)
    │
    ▼
  engage ─▶ water / item reclaim ─▶ projectile & grenade overlays ─▶ emit
  (combat overlay,                    (dodge, shove, lob→shoot)      (compose usercmd,
   if enemy in sight)                                                 set_bot_cmd)
```

| stage | function | what it does |
|-------|----------|--------------|
| Sense | `sense` | Snapshots the edict into an all-`Copy` `Sense` (origin, angles, velocity, on-ground, water level, air remaining). Everything downstream reads the snapshot, not the live entity. |
| Pre-arm | `prearm_traversal` | Locks ownership of a jump/gap leg *before* the objective is chosen, so a newly-seen enemy can't hijack a leap already in flight. |
| Objective | `resolve_objective` | The intent layer: the mode nominates a target, [`perceive`](#perception--see-hear-feel) gates it to what the bot actually knows, the [item brain](#goals--hopes-and-dreams) picks a pickup, and [`vigil`](#goals--hopes-and-dreams) covers waiting on one. Returns an `Objective`. |
| Pricing | `bot_link_pricing` | Builds this bot's dynamic A* costs — closed gates, per-bot failed-link surcharges, teleport damping, and [rocket-jump fitness](#skill-fitness-and-variety). |
| Steer | `steer::steer` | Turns the objective into a movement command: repath, leg advancement, the [bunnyhop / hook / rocket-jump drivers](#movement-execution), near-field steering. Returns a `SteerOut`. |
| Combat | `combat::engage` | Overlays aim / weapon / strafe / fire onto the movement — only when an enemy is in sight and the leg isn't [traversal-locked](#movement-execution). |
| Overlays | `projectile_dodge`, `grenade_tactics`, `rocket_shove`, `grenade_combo` | Dodge incoming fire, and play the [grenade game](#combat). |
| Emit | `emit` | Composes the final usercmd, advances the aim spring, runs the fire gate against the settled view, and calls `set_bot_cmd`. The engine zeroes the cmd after running it, so it is re-sent every frame. |

Three of these stages hold the known-expensive work and are bracketed by the profiler
(`bot/prof.rs`): **Objective**, **Steer**, **Combat**. Those three names recur throughout this
document as the cost vocabulary — see [Performance engineering](#performance-engineering).

## Navigation — the world model

Everything a bot knows about *where it can go* comes from the navmesh, built once at map load in
the `rtx-nav` crate. There are no waypoint files; the graph is generated from the map's BSP.

### From BSP to cells

The build runs on one oracle: **hull 1**, Quake's standing-player collision hull, whose clip
planes were already beveled by the player box at compile time. A single **point** test against it
(`Bsp::is_solid`) answers "would the player box collide here?" — so the whole navmesh is carved
without ever inflating a bounding box. Hull 0 (the render tree) is kept separately for
`pointcontents`, the only hull that carries liquid contents; the clip hull resolves to solid-or-
empty and can't see water at all. That distinction becomes important below.

`carve_cells` sweeps every 32-unit grid column in parallel; `column_floors` walks each column
bottom-to-top and drops a `Cell` (a player-origin standing spot, tagged with its grid column) at
every solid→empty transition, its height refined by a bisection. `classify_grounded` then links
each cell to its eight neighbours, emitting a `LinkKind` per move: `Walk`, `Step` (a shallow
riser), `Drop` (a safe fall), or `JumpGap` (a gap that needs a jump). A staircase is read as
stairs, not a jump, because two shallow risers inside one grid span classify as `Step`.

### The build pipeline

The graph is assembled in a fixed order — each pass layering richer links onto the static-hull
cut beneath it:

```text
  off the main thread (pure, liquid-blind clip hull):

    NavGraph::build            cells + walk/step/drop/jump links + ledge flags
      → add_double_jumps       wider gaps an air-jump reaches
      → add_speed_jumps        bhop-carried gaps (+ curl jumps)
      → add_hooks              grappling-hook arcs
      → add_rocket_jumps       rocket-blast arcs
      → add_plats              func_plat lift boarding
      → add_teleports          trigger_teleport pairs
      → add_gates              button-gated door obstructions
      → surcharge_under_plat_links
      → build_reachability     SCC + transitive closure (O(1) "can A reach B?")
      → build_lod              the coarse cluster/portal hierarchy

  then at graph-swap on the main thread (needs the render hull's pointcontents):

    flag_hazards → flag_water → patch_lod_liquids
```

The two-part split is the **purity seam**. The worker build sees only the liquid-blind clip hull,
so every liquid-aware pass — hazard flags, water flags, folding those costs into the LOD tables —
runs afterward on the main thread, where `pointcontents` is available. The worker half is a pure
function of the BSP, which is what makes it safe to run off-thread and byte-identical to re-run.

### The link zoo

Ten `LinkKind`s carry a bot across the map. Static ones cost a fixed travel time; the solved
movement links carry their flight data in [side tables](#side-tables).

| kind | how it's found | validated by |
|------|----------------|--------------|
| `Walk` / `Step` / `Drop` | `classify_grounded` between grid neighbours | head-height corridor + floor continuity |
| `JumpGap` | `find_jumps` off a ledge edge | run-jump reach & apex, clear arc |
| `DoubleJump` | `add_double_jumps` (`rtx_doublejump`) | wider reach, taller arc, only where no direct link exists |
| `SpeedJump` | `add_speed_jumps` | a measured runway caps attainable bhop speed; the link *starts* at the runway, so a bot always runs the full approach |
| `SpeedJump` (curl) | `solve_curl_jumps_from` (`rtx_bot_curljump`) | a full `pm_step` physics rollout certifies the air-turn lands |
| `Hook` | `add_hooks` (`rtx_grapple`) | an offline swing simulation against the BSP + a perturbation-robustness sweep |
| `RocketJump` | `add_rocket_jumps` (`rtx_bot_rocketjump`) | a two-phase blast simulation + perturbation sweep; carries a health cost |
| `Plat` / `Teleport` | entity splices (`add_plats` / `add_teleports`) | derived from `func_plat` / `trigger_teleport` entities |

Two ideas keep the jump passes from exploding. First, **octant-and-elevation dedup**: candidates
are bucketed by compass octant *and* elevation band, so a ledge sprouts a handful of jumps rather
than hundreds of near-parallel ones, and — crucially — a short descending jump into a pit doesn't
shadow the level jump across the gap onto a separate ledge. Second, everything is **solved
offline against the same solidity oracle** the bot will later fly through. A rocket jump, for
instance, reduces to two numbers the runtime needs — **when** to fire (the delay) and **which
way** (the fire angles) — found by integrating the jump-then-blast arc for every (pitch, delay)
pair and keeping the ones that land with margin.

### Side tables

The graph carries five `SideTable<T>` columns — for gates, hooks, speed jumps, rocket jumps, and
plats. A `SideTable` is a sparse per-link payload column, index-parallel to `NavGraph::links`: it
maps a link to the extra data its kind needs (the door it depends on, the arc to fly, the lift to
board). Tags come in two shapes — **1:1** (one hook link owns one arc) and **n:1** (many links
board the same plat, or cross the same gate) — and both read back the same way. It is append-only
and `Vec`-backed, so iteration order is exactly link-push order: **determinism is structural,
never hashed**.

Per-*cell* data rides in plain parallel vectors instead: `water`, `breathable`, `hazard`,
`under_plat`, and `ledge` flags, read through accessors like `cell_hazard` and `is_ledge`.

That determinism is a hard requirement, not a nicety. The server-side bot and the netclient bot
must drive the *same* run from the same graph, and two consecutive identical-state repaths must
produce the same route — otherwise a bot stutters. The build gets there by solving in parallel but
**splicing serially**, so link indices are deterministic regardless of thread timing; fingerprint
tests guard it.

### Pathfinding

All routing lives in `navmesh/query.rs`, the read-only side of the graph:

- **`find_path`** — plain A*, with a straight-line-time heuristic that is admissible (never an
  overestimate), so the first path found is optimal.
- **`find_path_banded`** — a kinodynamic A* over `(cell, band)` states with four speed bands
  (`rtx_bot_bandplan`). It credits the speed a bot carries between legs, so chained speed jumps and
  hot corridors actually route; carried speed only survives a corner inside a 45° cone. This is the
  live planner.
- **`costs_from`** — a Dijkstra cost-flood ("goal flood"): one pass yields the travel time to
  *every* cell, which is what [goal selection](#goals--hopes-and-dreams) scores items against.
- **`nearest_reachable_to`** — when a goal is unreachable, an O(cells) scan for the closest cell
  that can reach it, backed by the reachability table below.

Every search shares one cost model. A link's **static** cost is its travel time in seconds. Every
**dynamic** term is added on top in `link_extra`: a closed-gate penalty, per-bot failed-link
surcharges, a dash of per-bot jitter, a water tax, and a hazard health-price. Critically, all of
those terms are **non-negative and finite** — so A* stays admissible, and no live condition can
ever make a link cost infinity. That second property is what the next piece relies on.

### Reachability

`navmesh/reach.rs` answers a *topological* question — "can a bot at A ever get to B?" — that a
routing search shouldn't have to. Because every dynamic cost is finite (a shut door is charged a
penalty, not infinity), reachability never changes at runtime and can be precomputed. Cells
collapse into strongly-connected components (Tarjan), the components form a DAG, and a forward
transitive-closure bitset over that far-smaller DAG answers `reachable(A, B)` in O(1). For real
maps the main walkable mass is one giant component and one-way links (drops, teleports, unfit
rocket jumps) carve off the rest, so the whole closure is a few tens of KB. This is what lets goal
selection reject an unreachable item instantly instead of exhausting an A* to discover the dead
end.

### The LOD hierarchy

On a big map, flooding the whole cell graph for every goal score and every long steer is the
frame's dominant cost. `navmesh/lod.rs` is the navigation analogue of mesh level-of-detail: cells
group into coarse **clusters**, and an abstract graph of **portals** between clusters lets scoring
and long-range steering reason over hundreds of nodes instead of tens of thousands of cells. Near
the bot the fine graph is still queried exactly; only the far field goes coarse.

A cluster is a connected component of cells within one spatial **block** — a 256-unit column
(`LOD_SHIFT = 3`) *banded by storey* (`LOD_STOREY = 128`). The storey band is not cosmetic: without
it the block union is Z-blind, so a one-way drop welds a platform to the pit beneath it into one
cluster spanning both heights, the single cheapest crossing kept for that cluster pair becomes a
low walk that *evicts* the climb link onto the platform, and the coarse route lands below the
platform it was aiming at — the bravado-quad-unreachable-under-LOD bug, fixed by banding.

Between each directed cluster pair the build keeps a few **representative crossings** — one per
border level and per gatedness (the cheapest, plus the cheapest gate-free one) — plus a coverage
pass that promotes any landing no rep covers. Queries then layer on top: `coarse_costs` scores
with exact in-home-cluster costs and a *bounded overestimate* beyond (never an underestimate, so it
stays safe), and `corridor` hands steering an interim target plus the cluster window the fine
`find_path_within` may stay inside.

### The near-field grid

The 32-unit navmesh deliberately lacks the resolution for the *last metre* of steering, so a
grounded bot builds a fine **8-unit clearance grid** around itself (`nearfield.rs`). It is a
**repulsion field, not a mini-navmesh** — no links, no A*. `NearField::build` floods the walkable
columns out from the bot's own footing (so a bridge over a tunnel resolves to the floor connected
to the bot, and stairs climb by construction), classifying each column walkable / wall / drop /
hazard. Then:

- **`steer_push`** returns a horizontal nudge off nearby walls and drop-edges, weighting drops
  above walls, and cancelling to zero between symmetric obstacles — which is what centres a bot in
  a doorway or on a thin beam.
- **`chord_clear`** certifies a straight glide short-cut stays on clear floor, straightening the
  navmesh's 45° zigzag into a clean line (`rtx_bot_glide`).
- **`chord_open`** trims at the flood frontier for the long bhop look-ahead, vetoing the hop the
  instant a drop crosses the line.
- **`edge_ahead`** measures the distance to the first drop or hazard edge along a heading — the
  input that makes a bot carve on the ground at a ledge lip instead of bhopping off into the void.

### Hazards

`hazard.rs` classifies where lava, slime, and lethal drops lie relative to a point, over the same
solid-plus-contents oracle pair. Its subtle trick is **shallow-film detection**: a lava film
thinner than a probe stride would be missed by marching downward, so `solid_or_film` bisects the
clip-hull boundary and runs the engine's waterlevel-1 sample at feet+1, catching ankle-deep lava
that still burns.

Two consumers share it: **offence** (`find_hazard` scans the ring around an enemy for a spot to
shove them into) and **self-preservation** (`hazard_ahead` / `ledge_ahead` ask "does stepping this
way walk me into lava or off a ledge?"). At graph-swap the flags are baked in: `flag_hazards`
stamps a per-cell hazard kind and a per-link `hazard_hp` from the *real* damage model (lava ticks
10 hp / 0.2 s, slime 4 hp / 1.0 s), a risk premium on links onto a pool edge, and a near-fatal
surcharge on jump arcs over a lava or slime pool (below).

Two design choices are worth stating. First, **risk lives in health, not seconds**: a link's cost
holds no risk premium; the damage is priced per query via a per-bot `HazardPrice { strength, k }`,
so a hurt bot detours around lava while a healthy or armored one clips the corner
(`rtx_bot_hazard_health`, `rtx_bot_hazard_k`). Second, **falls are split by what lies below**: water
is transit-only (a swim-speed tax, not a wall), and a **dry pit is not flagged as a routing
hazard** — the runtime edge guards own it, since a fall onto solid is a choice made at the lip, not a
region to route around — but a **jump whose fall-short span crosses lava *is* priced fatal**, because
an undershot leap lands in the pool rather than on the far platform, and the per-cell pricing misses
it since both footings are safe. Routing then prefers the walk-around; a sole-route lava jump stays
finite (the cost is capped) and is still attempted. Under-plat shaft cells are similarly
*transit-only*: stamped and surcharged so a bot passes through but never parks under a raised lift —
and pointedly with **no timers**, only a standing cost.

## The decision loop

Each frame, the `Sense` snapshot and a per-bot blackboard are all the state the brain has. The
blackboard is `BotState` (`bot/state.rs`), a ~60-field struct carried on the bot's client edict.
It groups into:

- **Route** — the current A* route as link indices, the leg position, the planned speed band per
  leg, the goal cell, and the failed-link / recent-teleport surcharge rings the planner diverts
  around.
- **Goal** — which item, where, the two-leg continuation, the movement-ownership commit lock, the
  team handoff/hold bookkeeping, and the avoid ring.
- **Combat memory** — the aim spring (angles, velocity, drifting error), the last true-LOS
  sighting, and the perception clocks.
- **Traversal machines** — the hook, grenade, rocket-jump, bhop, speed-jump, air-commit, and
  plat-wait state machines.
- **Reflex caches** — anti-drown and burn-escape targets, and the vigil cruise/scan state.

### Perception — see, hear, feel

`bot/perception.rs` is the gate between what the mode *nominates* as a target and what a human-like
bot actually *knows*. Without it a bot would fight a target the instant the mode picked one: 360°,
any distance, no reaction time. With it, a nominated enemy becomes actionable only once it has
been:

- **seen** — inside the view cone, with line of sight, held for a reaction beat. Only sight yields
  an *exact* position.
- **heard** — its gunfire still ringing within earshot (a cheap stand-in for a sound bus). Hearing
  yields a **direction, not a place**: `heard_hypothesis` quantizes the range to coarse buckets and
  scatters the guess laterally, so the bot investigates the right general bearing without
  wall-hacking the true point.
- **felt** — incoming damage stamps the shooter's bearing directly, so a shot in the back turns the
  bot toward the hit without seeing the attacker.

`perceive` runs once per bot per frame and returns an `Awareness` of `Unaware`, `Known` (last
seen at a remembered spot), or `Visible`. Awareness then persists five seconds
(`MEMORY`) — object permanence — so a bot that loses sight **hunts the last-seen spot** rather
than tracking a target through walls, then gives up. Skill widens the cone (+4° per level) and
shortens reaction (× (1 − skill/8), floored so even skill 7 isn't instant).

One guard matters only to the netclient: a player that leaves PVS (behind a wall, through a
teleporter) leaves a frozen ghost shadow in the mirror. `net_seen` stamps the frame a shadow was
last refreshed, and `net_shadow_stale` (0.2 s grace) makes both perception *and* combat treat a
stale shadow as gone — so a client bot stops firing the instant the real player disappears instead
of emptying its magazine into an empty teleporter. It is never true server-side.

### Objective resolution

`resolve_objective` composes it all: a control-channel puppet override wins first (for scripted
tests); otherwise the mode's `bot_intent` nominates a target, `perceive` gates it (an `Unaware`
fight downgrades to patrol; a `Known` one becomes a hunt toward the last-seen spot), the item brain
picks the best reachable pickup, and `vigil` takes over when the bot is waiting *at* an item rather
than travelling to it. The result is one `Objective` — an enemy, an item cell, a target origin —
that the rest of the frame steers and shoots toward.

## Goals — hopes and dreams

What a bot *wants* is decided in `bot/goals.rs`, ported from the shape ktx duellers use. Each item
carries a **desire** — the marginal effective-HP it would add (health, armor), the firepower gap it
would close (weapons, ammo), or a flat dominating value (powerups, runes) — weighted by how soon
the bot could reach and collect it:

```text
  score = desire × (LOOKAHEAD − t) / (t + 5)          closer and sooner-available wins
```

`t` is the travel-or-respawn time from a Dijkstra flood (fanned across the [worker
pool](#performance-engineering)), so an item mid-respawn costs the wait until it returns — a bot
will head for a quad that's about to come back. Anything not collectable within the horizon is
dropped. Planning is a bounded **two-leg** sequence: a nearby useful item can become the first
stop on the way to armor, and the second leg is revalidated after the first touch rather than
followed blindly.

The horizon itself is tiered, because not every prize is worth the same trek:

| tier | horizon | what qualifies |
|------|---------|----------------|
| Ordinary | 10 s | shards, cells, a missing shell — a short detour only |
| Major | 20 s | red armor, megahealth — worth a real trek *and* worth cycling, since holding them is half of QW map control (`rtx_bot_stack`) |
| Powerup | 30 s | quad, pent, ring — a flat dominating desire, worth crossing the map for and worth *waiting at* to deny the enemy its timing |

**Resource discipline** (`rtx_bot_stack`) sharpens valuation. The neutral stack is 100 effective
HP — exactly the 50 health + 50 armor a bot strives to hold — and desire for health/armor ramps up
to 2.5× below it. A hysteretic `CombatPosture` of `Recover` / `Hold` / `Press` governs when a bot
commits to healing: it enters Recover on a thin stack (≤ 30 hp, or ≤ 60 EHP, or a losing power
ratio), not just on low health, and only breaks contact to do it when it's actually safe — past a
550-unit disengage range or out of line of sight. A **dry-ammo panic** spikes ammo desire when a
bot's primary weapon is about to run empty.

**Denial** is how bravado shows up in the numbers: an RL or LG the enemy side *provably* lacks
gets a denial floor on its desire — but deliberately *below* the bar to break off a fight for it,
so denial never yanks a bot out of combat. A hidden powerup can even pull in a nearby spawned item
as a **bridge**, but only when the full detour still reaches the powerup on time.

**Gate valuation** is what makes a prize behind a locked door reachable. A shut door the bot can
open is priced not at the planner's full route-around penalty but at a modest 8-second **button
errand** — small enough that the prize stays inside its horizon, so the bot *chooses* the detour
and heads over to work the button. A door whose button sits on its own far side keeps the full
penalty (it can't be opened from here). **Route magnetism** (`rtx_bot_magnet`) bends the immediate
waypoint through a desirable item lying just off the corridor, so the bot steps onto it in passing
— applied only when the item is genuinely on the way. And **vigil** (`bot/vigil.rs`) covers the
wait: instead of twitching on a not-yet-respawned item, the bot cruises a short ring around it and
pans its view to scan for enemies — which, because perception reads the bot's own view angles, is
real scouting, not cosmetic.

### Worked example: ultrav's quad errand

One route ties the whole model together. On `ultrav`, the quad sits on a platform you can only
reach through a **button-gated door that opens onto a teleporter**, and the teleporter's exit
**drops you into open air** — you finish the trip by air-steering onto the quad's platform as you
fall. To a hand-authored waypoint bot this is three unrelated mechanics; to this brain it is one
priced route.

It works because each mechanic is already a link the planner understands:

1. **The quad is seen through the wall of cost, not the wall of geometry.** Goal valuation floods
   the graph and finds the quad on the far side of a shut gate. Ordinarily that gate carries the
   full route-around penalty and the quad would look impossibly far — but the gate's *button* is
   reachable without crossing any shut gate, so it prices as the 8-second button errand instead. At
   the 30-second powerup horizon, a 200-plus-desire quad easily survives that surcharge. The bot
   *wants* it.

2. **The button becomes a sub-goal.** The navmesh's gate splice recorded where to stand
   (`button_cell`) and where to aim (`aim`) when the door was built. The bot walks the detour to
   that cell and works the button, its progress watched by a give-up clock that re-routes if it
   turns out the button can't actually be reached.

3. **The opened door exposes a teleport link.** With the door no longer obstructing, the route
   through the `Teleport` link is now cheapest, and the planner takes it — the bot walks into the
   teleporter as just another leg.

4. **The exit drops it into the air, and air-steering finishes the job.** The teleport deposits the
   bot airborne over the gap to the platform. The same air-control that flies a jump arc now steers
   the fall toward the next waypoint, carrying it onto the quad's ledge.

One button push, one teleport, one airborne landing — and to the valuation it was never a special
case at all. It was seconds on a route to a 200-desire item, and the planner would happily throw
away any leg of it the moment a cheaper path to the same quad appeared.

## Opponent modeling

`bot/model.rs` gives the bots the running read a human keeps on each opponent — "he's on low
health", "they never got the RL", "that one has quad" — as data. It is a small point-estimate per
opponent (health, joint armor, arsenal bits, powerup expiries), updated **only from events a bot
could legally witness**, and pooled per side so a team shares one blackboard (`rtx_bot_model`).
Pool 0 is the FFA collective; pools 1–8 are the per-team blackboards, and one team's sightings
never leak into another's.

Every update is observation-gated within earshot (1000 units, matching perception's hearing): a
witnessed hit mirrors the exact damage `t_damage` applied, witnessed fire sets the arsenal bit for
that weapon, and an item touch updates the stack. Death is public — frags are broadcast — so a kill
resets the estimate to the mode's spawn kit. When a below-prior health estimate goes unobserved, it
holds for a five-second grace and then **drifts back up at 2 hp/s** — the "he's probably healed by
now" assumption. Armor never drifts up, a limitation the code documents and the one risk-spending
consumer guards against with a freshness gate. The design lineage is explicit in the module header:
F.E.A.R. / Killzone squad blackboards for the shared memory, van Waveren's Quake III Arena team AI
for treating items as strategic currency.

What the estimate drives:

- **Denial** — knowing the enemy lacks a key weapon turns its spawn into ground worth denying.
- **Target bias** — a weak stack scores as if nearer (× 0.4), a strong one as if farther (× 2.5),
  and a heavily-armed target in a no-weapons-stay mode is nudged preferred, since killing them
  resets their kit.
- **Combat risk** — the enemy's estimated strength × firepower is the "power" the recovery posture
  weighs the bot's own stack against.
- **The finish read** — a fresh estimate below a low threshold marks an enemy `finishable`, which
  swaps a dodgeable rocket for a hitscan shot.
- **The backpack hypothesis** — on the netclient, where a dropped pack's contents aren't on the
  wire, `believed_arsenal` unions what the pools know to guess what's in it.

## Team play

In a team composition, coordination is always on (it is the *inferred enemy* model that
`rtx_bot_model` gates, not teamwork itself). The mechanisms all live in `goals.rs`:

- **Pickup reservations** — each item gets a stable nearest-bot claim (distance, then edict id to
  break ties), and a claimed item is discounted for everyone else, so teammates spread across
  pickups instead of racing the same one. A powerup's dominating desire still wins it outright.
- **Powerup split** — a bot defers an uncontested powerup to a teammate who has clearly claimed it
  or is closer by a clear margin, while a near-tie lets both press it for redundancy — with backup
  coverage kept if a known enemy could arrive first.
- **Weapon handoff** — an idle bot will reserve a spawned RL or LG for a powerup-carrying teammate
  who lacks it, standing *on* the weapon without grabbing it until the carrier arrives (or a
  timeout lapses, because a denial beats a handoff that never arrives). An enemy inside contest
  range makes the bot take it — denial again.
- **Truthful sharing** — teammates read each other's actual arsenals directly; that honest sharing
  *is* the coordination. Teleport arrivals are staggered so a team doesn't pile onto one exit.

CTF role assignment (attack / midfield / defense, escorts peeling for a carrier) is a mode concern
— see [game modes](modes.md).

## Movement execution

Turning a route into a usercmd is the job of `bot/steer.rs` and the driver modules it calls. Steer
runs on an immutable `&NavGraph` plus `&mut BotState` plus the frame snapshot — never `&mut
GameState` — which is exactly what lets `run_bot` hold those two disjoint borrows and then resume
the combat/emit spine once steering returns.

The baseline is straightforward: repath every 0.4 s via the banded planner, advance to the next
leg within 24 units of the waypoint, glide the near-field's straight-chord short-cut, and lean off
one-sided drops. On top of that sit the specialised drivers.

### Bunnyhop and the slalom

`bot/bhop.rs` is the air-strafe controller, and it leans on two engine facts (verified against
FTEQW's `pmove.c`). QuakeWorld's air acceleration clamps the *projected* wish speed to a small cap
(~30 ups), so when the wish direction is held roughly perpendicular to velocity — one strafe key,
the view swept to keep the angle — speed grows every frame without bound. And `PM_CheckJump` runs
*before* `PM_Friction`, so a landing frame with jump held skips ground friction entirely and
strafes like an air frame — which is what makes an unbroken hop chain possible.

The controller is a `Bhop` state machine (prestrafe → hop → landing → re-takeoff). Its signature
move is the **slalom**: rather than the max-rate perpendicular weave — which turns velocity ~300°/s
and forces three jittery sign-flips per hop, "the shake" — it turns the velocity at a smooth 140°/s
(the rate demos measure real players riding), angling the wish a few degrees forward of
perpendicular so it still gains speed but carves *one wide lobe per hop*. The lobe is symmetric
about the bearing, so the S-curve stays centred on the route; when a bend or a wall forces it, an
error-gain term ramps the turn to the physical maximum. On corridors too short to hop, the bot
ground-zigzags (circle-strafe) instead, its band capped tighter in tight spaces. Bhop drops back
to a plain gait for corners, ledges, combat, stairs, and the final approach.

### Committed flight

A jump is an **indivisible action**. `prearm_traversal` arms the leg before combat arbitration even
runs, and an air-commit latch freezes route advancement while the bot is airborne, releasing only
after a real landing. So an enemy appearing at the lip, or mid-arc, cannot flip the goal and yank
the bot off the jump into the gap. A plain-jump takeoff is additionally gated on the bot actually
moving *toward* the waypoint, so it doesn't leap from a standstill.

### The specialised jumps

- **Rocket jump** (`bot/rj.rs`) runs an `RjPhase` machine — `Idle` → `Stance` → `Rise` →
  `Ballistic`. The bot walks to the launch cell with the RL selected, settles its view on the
  solved fire angles, jumps (the press held until emit confirms the aim is aligned), fires after
  the solved delay, and rides the blast arc on gentle air correction. Per-phase timeouts abandon a
  stalled attempt, and full telemetry feeds the [tuning harness](development.md).
- **Speed / curl jumps** are committed bhop run-ups: the driver holds the solved takeoff speed to
  the lip, then curls onto the landing.
- **Grappling hook** (`bot/hook.rs`) runs a `HookPhase` machine — aim the anchor, throw, reel to
  build speed, then **release mid-reel** so the resulting velocity flings the bot along a parabola
  onto the target ledge (a straight pull-up is just the degenerate case).

All three share a failure tail: two consecutive failed traversals bump the link's per-bot surcharge
and abandon the chased goal, so the planner diverts instead of re-issuing the dead leg forever.

### Plats, ledges, stairs, water

- **Plats** — a bot holds a standoff 40 units outside a raised lift's footprint (standing under it
  resets its descent timer), boards when it lowers, and gives up after 8 seconds.
- **Ledges** — on a navmesh cell flagged as an open-cored inner edge (the `ledge` flag), bhop is
  vetoed, ground speed is capped (`rtx_bot_ledgecap`, 210 u/s), and a geometric ledge brake thrusts
  backward when velocity drifts off-corridor toward the drop. A second, hazard-aware brake keys off
  the near-field's `edge_ahead`: when a drop *or lava* edge lies within the bot's stopping distance
  along its velocity, it reverses the wish and cancels the hop — killing the momentum that would
  carry a fast bot over a lip even mid-bhop — and the stuck detector likewise holds its unwedge-jump
  rather than launch a wedged bot off a lava lip.
- **Stairs** — risers drop the hop chain to a walk (a human runs *up* stairs), while the near-field
  glide tracks the treads.
- **Water** — two reflexes override navigation entirely. When submerged with under five seconds of
  air, the bot floods to nearest breathable cell and swims up, and combat hands movement back to
  navigation so it can't strand the bot underwater. A parallel burn-escape redirects onto the
  nearest safe cell when the bot is stuck in lava or slime.

## Combat

`bot/combat/mod.rs` overlays shooting and dodging onto the movement the navmesh already produced —
it never calls weapon-fire directly, it only chooses view angles, a weapon impulse, the attack
button, and evasive movement. While the bot has no line of sight it keeps navigating untouched;
once it can see the enemy, `engage` takes over the look and the trigger.

**Aim is a model, not a snap.** A critically-damped spring (stiffness `6 + 2·skill`) drives the
view toward the target, so a spectated bot turns like a mouse-controlled human rather than
teleporting its crosshair. Layered on top is a *drifting*, skill-scaled error (`spread_scale`):
loosest on first sight and while running or tracking a fast crosser, tightening over ~1.5 s of held
line of sight — so misses sweep past the target and drift back rather than buzzing around it. A
feed-forward lead cancels the spring's tracking lag on strafers, and by skill 7 the error is
essentially zero. The spring's angular speed is capped (`360 + 90·skill` deg/s; `rtx_bot_turnrate`
overrides) so a full look-reversal, like a vigil-scan flip, sweeps at human speed instead of
snapping — combat flicks sit below the cap and are untouched.

**Weapon by range**: super shotgun point-blank, LG at mid range, RL as the default around 400
units. A `finishable` enemy — fresh opponent-model estimate below 35 — swaps a dodgeable rocket for
a direct hitscan hit to close it out.

**The fire gate** runs in `emit`, *after* the spring has settled, so it judges the shot the bot is
about to take. A shot fires only when it is on target (miss distance within tolerance), the line
of fire is clear, no teammate is in the way, and the bot won't splash itself. That last check
is where the discipline lives:

- **Splash self-care** projects the bot's own post-armor damage using the live quad multiplier and
  budgets it to at most half the bot's health; quad rockets get KTX's conservative 250-unit caution
  zone. A rocket aimed into a corner — one whose muzzle→aim trace hits a wall inside its own
  160-unit blast radius — is withheld.
- **LG underwater** is barred outright (a discharge would dump every cell self-lethally), with one
  exception: a *deliberate* discharge fires only when worth it, killing a believed quad-carrier
  or two-plus enemies.

**Movement in a fight** holds the preferred range, strafes with a bias off the enemy's current aim
line, and retreats below 40 health (halved when pressing a finish) — all footing-filtered so a
strafe never carries the bot onto lava, off a ledge, or into a lift shaft. `projectile_dodge`
predicts the closest approach of live rockets and nails (and samples a grenade's arc and fuse) and
takes a hazard-checked lateral escape when the damage tube intersects the bot.

The **grenade game** (`bot/grenade.rs`) is played from both sides — the behavioural story is in
[the bots](bots.md#the-shootable-grenade-game); architecturally it is a set of solvers and gates:
a hazard-shove that puts the blast point *behind* an enemy so outward knockback drives them into
lava or off a ledge (verifying the shove actually carries them across the edge first), a lob→shoot
combo that arcs a grenade over cover and detonates it in flight, and a bank-shot solver that
searches launch angles for a bouncing-hull path around a corner, with a jitter sweep rejecting
knife-edge throws.

## Skill, fitness, and variety

There is **one global skill knob**, `rtx_bot_skill` (0–7); there is no per-bot skill field. It
scales the whole competence surface at once:

| skill scales | effect |
|--------------|--------|
| view cone | +4° per level |
| reaction time | × (1 − skill/8), floored |
| aim spring stiffness | `6 + 2·skill` |
| aim turn-rate cap | `360 + 90·skill` deg/s |
| aim spread / error | tighter with skill; ~0 at skill 7 |
| feed-forward lead, fire tolerance, dodge awareness | all sharpen with skill |

The deliberate line the code draws: **low skill reacts and shoots less precisely, but never becomes
strategically suicidal.** Posture, goal valuation, and self-preservation are skill-independent — a
weak bot misses more, it doesn't walk into lava.

**Rocket-jump fitness** is a per-bot gate, not a skill: `rocket_jump_extra` surcharges each RJ link
out of routing consideration unless the bot has the launcher, a rocket, no quad running (a
self-rocket under quad is lethal and off-model), and health above the worst-case self-blast plus a
25-hp reserve (armor lowers that bar via absorb). The driver re-checks fitness against the specific
leg's solved self-damage on arrival, so a bot that lost health en route bails rather than diving in
too hurt.

**Variety is deterministic, not a personality model.** Behavioural spread comes from per-edict-id
determinism — a per-bot A* jitter seed so two bots don't tread an identical line, strafe and dodge
phase keyed on the edict id, and a goal-select spread that de-phases a squad's floods. The only
cosmetic layer is naming: 32 rotating Quake-style names shown as `bot•Grunt`, with distinct shirt
and pants colors (`bot/population.rs`).

## The two embodiments

The brain is written once and hosted twice, joined at the `ClientHost` seam. Everything the engine
provides for a server-side bot, the netclient substitutes (`crates/rtx-game/src/netclient/`,
the `netclient` feature):

- **`mirror`** writes each entity's `EntVars` from `svc_playerinfo` / `svc_packetentities` / stats,
  where the engine would have filled them.
- **`NetHost`** answers `traceline` / `pointcontents` / cvar reads from the map's own BSP (via
  `rtx-nav`) and its own cvar store, where the engine would have answered.
- The **cmd sink** turns `set_bot_cmd` into a `clc_move` on the wire, where `SV_RunCmd` would have
  run it. Trigger touches the server performs for us, because the bot is a real client.

`Client::tick` drives it: poll every connection's wire events and apply them, rebuild the world if
the map changed, advance the clock, ensure the navmesh, mirror the entities, then call the *same*
`bot::run_bots` over the shared `GameState`, and send the moves. A squad is **N connections in one
process sharing one `GameState` and one `WorldMirror`** — which is not a shortcut but a faithful
mirror of how server-side teammates already share item timers and an opponent model. Each bot's
body and stats are its own; what any one of them sees, all of them know.

The honesty seam is the whole point. An enemy's health, armor, and ammo are **not on the wire and
never will be**, so a client bot leans on the same opponent model it already uses — the answer that
makes it play like a player rather than a cheat. Server-advertised movement cvars are mirrored from
serverinfo and forced off on any other server, so the navmesh never plans a double jump the server
won't grant. Both QuakeWorld and NetQuake are supported, and a missing map is fetched over HTTP
before signon. Usage lives in [network clients](netclient.md).

## Performance engineering

The brain shares the server's frame. `SV_RunBots` only runs a bot frame once `1/maxfps` has passed
(~13 ms at the default 77 Hz) and runs it *after* `SV_Physics`, so anything the brain spends is
added to the server's own frame work — blowing the budget makes the *server* late, not just the
bots. `bot/prof.rs` (`rtx_bot_prof`) brackets the three expensive phases — Objective, Steer,
Combat — against that slice, and reports the squad's decision cost.

The optimizations that keep it inside the budget:

- **Parallel goal floods** (`bot/par.rs`) — a single goal pick runs up to nine whole-navmesh
  Dijkstra floods, the frame's dominant cost on a big map; they are pure functions of `(graph,
  source, costs)`, so fanning across cores is bit-identical to serial (`rtx_bot_par`). The pool
  is *never* rayon's global pool — this crate ships inside a game module the engine can `dlclose`,
  and global workers would outlive the unload and later run freed code — so it owns its workers and
  joins them at shutdown.
- **Static reachability** turns "is this goal even reachable?" from an exhausted A* into an O(1)
  bitset test.
- **The LOD hierarchy** bounds far-field scoring and long steering to the abstract portal graph
  (`rtx_bot_lod`), and the **near-field grid** runs only in the last metre (`rtx_bot_nearfield`).

### Architecture-relevant knobs

These shape *how the brain computes*, as opposed to how the bots play; the full registry, with the
gameplay knobs, is the [cvar reference](cvars.md).

| cvar | default | effect |
|------|---------|--------|
| `rtx_bot_bandplan` | `1` | Kinodynamic A* over speed bands; `0` = plain A*. |
| `rtx_bot_lod` | `1` | Route over the coarse LOD hierarchy; `0` = exact whole-graph floods. |
| `rtx_bot_par` | `1` | Fan goal floods across the worker pool; `0` = serial. |
| `rtx_bot_nearfield` | `1` | Last-metre steering off the 8u clearance grid. |
| `rtx_bot_glide` | `1` | Straighten the grid zigzag on a certified clear chord (sub-toggle of nearfield). |
| `rtx_bot_ledgecap` | `210` | Careful-ledge walk-speed cap (u/s); `0` = full maxspeed. |
| `rtx_bot_turnrate` | `0` | View turn-rate ceiling (deg/s); `0` = skill-scaled default. |
| `rtx_bot_prof` | `10` | Seconds between profile reports; `0` = off. |
| `rtx_control_port` | `0` | Localhost TCP for scripted bot puppetry and tuning. |

---

*See also: [the bots](bots.md) · [the bots as network clients](netclient.md) ·
[movement & combat](movement.md) · [development & tooling](development.md) ·
[cvar reference](cvars.md)*
