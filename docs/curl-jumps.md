# Curl Jumps

A **curl jump** is a speed jump whose flight path is not a straight line from the link's source to
its target: the bot runs a corridor, leaps along that corridor, and then carves the velocity
sideways in the air to land on a platform that is offset from the takeoff heading. This document
describes how those jumps are found and certified offline in `rtx-nav`, what the navmesh stores,
how the runtime in `rtx-game` executes them, and where the design currently falls short.

It is written for someone reading this code for the first time. Every mechanism claim below cites
`path:line` so it can be checked; upstream-only citations are marked as such.

Part of the [rtx manual](../README.md) · see also [the bots](bots.md#curl-jumps) ·
[cvar reference](cvars.md)

## Why a straight-line treatment is not enough

The stock jump families price and fly a **chord**. `JumpGap`/`DoubleJump` certify the straight
source-centre → target-centre line; the straight `SpeedJump` pass
(`crates/rtx-nav/src/navmesh/jumps.rs:492`) tests the arc with `arc_clear_peak` over that same
chord and prices the leap by `v_required(horiz, dz, gravity)`
(`crates/rtx-nav/src/navmesh/physics.rs:230`). That is correct whenever the run-up, the leap and
the landing are collinear.

On the DM3 routes we drill they are not. The recurring shape is:

```
      run-up corridor                      target platform
  ────────────────────────────┐               ┌──────────
   ==>  ==>  ==>  ==>  ==> lip│               │
                              │   pit         │
                              └───────────────┘
                                   the straight chord from the lip
                                   to the platform passes through
                                   the wall on the near side
```

The bot has to leave the lip on the corridor heading — a wall or a doorframe sits on the chord —
and only then turn onto the landing. Two separate things break if you insist on the chord:

1. **Feasibility.** `arc_clear_peak` rejects the link because the straight arc is blocked
   (`jumps.rs:383`), so no link is emitted at all and the route does not exist.
2. **Execution.** Even where a chord-certified link *is* emitted, the runtime takes off offset from
   the certified line and homes back onto the target centre, which sweeps the arc into the edge
   wall beside the chord. That failure mode is exactly what the `rtx_jump_curl_hold` /
   `rtx_jump_curl_gain` band-aid on plain jump legs exists to blunt
   (`crates/rtx-game/src/cvars.rs:182`, applied at `crates/rtx-game/src/bot/steer.rs:2081`).

A curl jump replaces the chord model with a **rollout**: the whole manoeuvre — ground prestrafe to
the lip, leap, per-tick air correction, touchdown — is simulated against the BSP with the same
`pm_step` the runtime's physics mirrors, and the link is emitted only if that simulation lands.
There is no closed form; the certificate *is* the simulation.

## The offline side: finding and certifying curls

Curl generation is gated by `SpeedJumpParams::curl`, which the game derives from
`rtx_bot_curljump` (default `0`, `crates/rtx-game/src/cvars.rs:54`). With it on,
`add_speed_jumps` runs three passes in order (`jumps.rs:193`):

| pass | entry point | shape |
|------|-------------|-------|
| straight speed jumps | `solve_speed_jumps_from` (`jumps.rs:492`) | collinear run-up + leap |
| **plain curls** | `solve_curl_jumps_from` (`jumps.rs:307`) | local run-up, leap on the corridor heading, air-turn onto an offset landing |
| **chained ground-turn curls** | `solve_chained_ground_turn_from` (`jumps.rs:1712`) and `solve_chained_ground_turn_optimal_curl` (`jumps.rs:2260`) | carried entry speed + a *grounded* rotation before the leap |

Each pass has its own per-cell budget so a curl never evicts a straight jump
(`SPEED_JUMP_CURL_MAX_PER_CELL = 2`, `physics.rs:139`), and each is de-duplicated globally by
target cell (`CURL_TARGET_MAX = 2`, `physics.rs:142`; the dedup loop is `jumps.rs:222`) because a
dozen corridors certifying onto the same platform is noise the planner never needs.

### `solve_curl_jumps_from` — the plain curl pass

For each ledge cell and each of the four compass directions (`jumps.rs:332`):

1. **The leap must go into a gap** — reject if there is ground the leap way (`jumps.rs:334`).
2. **Measure the corridor behind the lip** with `measure_runway` (`jumps.rs:642`) and require at
   least `CURL_MIN_RUNWAY = 192` units (`jumps.rs:338`, `physics.rs:95`). This is a
   *corridor-quality* floor, not a speed floor — the prestrafe oracle saturates by ~90–150u; the
   comment at `physics.rs:92` records that lowering it roughly doubled the per-map curl count
   without covering any additional demo jump.
3. **Predict the takeoff speed** the ground circle-strafe delivers over the committed run-up with
   `prestrafe_delivered` (`jumps.rs:343`, `physics.rs:159`), which rolls the shared ground oracles
   at the ground-optimal wish angle `θ = acos(u*/speed)` and saturates at the friction equilibrium
   (~1.5·`sv_maxspeed`).
4. **Scan targets** within the reach the rollout tick cap allows (`jumps.rs:355`), and keep only
   those that sit **5°–78° off the corridor heading** (`CURL_ANGLE_LO`/`CURL_ANGLE_HI`,
   `physics.rs:107`). Below 5° the straight pass owns it; above 78° `air_correct` at curl speed
   cannot converge inside the airtime.
5. **Only curl what the straight pass could not own** (`jumps.rs:380`): if the arc is clear *and*
   the straight pass's air-strafe credit covers `v_required · SJ_MARGIN`, skip — a target the
   straight pass covers needs no curl.
6. **Certify by rollout** (the expensive step, reached only by survivors).

#### The psi search

The runtime leaps along the `from` → `takeoff` line, so that heading is the solver's to choose —
and certification turns out to be sharply sensitive to it. `CURL_PSI_SAMPLES = [0, -6, +6, -12,
+12]` degrees (`physics.rs:120`) are tried around the compass axis, on-axis first, and the
`from` cell is then *placed along whichever heading certified* (`jumps.rs:402`, `jumps.rs:437`) so
the runtime flies precisely the proven line. The comment at `physics.rs:116` records the
motivation: a real lip's approach is rarely exactly on a compass axis, and the DM3 `curl_mid`
geometry certifies at 6° off but not at 0°.

Orthogonally, the takeoff point itself is slid **back** along the run-up in `GRID` steps up to
`CURL_TAKEOFF_BACKOFF = 240` units (`jumps.rs:394`/`444`, `physics.rs:146`). A fast run-up
overshoots a leap taken right at the pit edge; sliding the leap point back over the near ground
(which the arc clears anyway) lengthens the flight until distance matches speed. The **latest**
leap point that certifies wins.

#### `certify_curl` — what "certified" actually means

`certify_curl` (`jumps.rs:765`) solves for the takeoff **speed**, then for the gentlest gain, and
proves the result across an envelope.

*The speed ladder* (`jumps.rs:782`). Certifying only at the run-up's equilibrium (~484 u/s, 327u of
flat reach) makes every moderate gap uncertifiable — it overshoots. So the ladder runs from the
ballistic floor `v_required(horiz, dz)` up to `v_deliver · CURL_V_LO_FRAC` (`0.94`,
`physics.rs:123`) in `CURL_V_STEP = 12` u/s increments (`physics.rs:127`, ≤24 rungs), and the
**lowest** speed whose whole envelope lands is taken. `physics.rs:124` states the rationale
plainly: a human holds a controlled speed rather than maxing out, and the runtime's takeoff regime
is built to hold exactly this solved value.

*The envelope* (`jumps.rs:800`). Six corners are proven, crossed with three tick-rate classes
(`GT_DT_CLASSES = [0.019, 0.020, 0.021]`, `jumps.rs:936`) — 18 rollouts per gain:

| corner | what it models |
|--------|----------------|
| `(takeoff, v·1.03, 0°)` / `(takeoff, v·0.97, 0°)` | the ±`CURL_V_HOLD_TOL` band the runtime holds (`physics.rs:131`) |
| `(early, v·1.03, 0°)` / `(early, v·0.97, 0°)` | leaping a lip-reach early — the runtime jumps on the frame it crosses the takeoff line, so on average it leaps `CURL_LIP_REACH = 28`u early (`jumps.rs:776`, `physics.rs:151`) |
| `(takeoff, v, +6°)` / `(early, v, −6°)` | `CURL_PSI_TOL` — the ground prestrafe exits mid-weave, so the real launch heading wanders (`physics.rs:113`) |

*The gain ladder* (`jumps.rs:809`). `CURL_GAINS = [4, 6, 8, 10, 12, 14, 16, 20]` (`physics.rs:134`),
gentlest first; the first gain that clears every corner × every tick class is baked into the link.

*One rollout* (`curl_lands`, `jumps.rs:837`) is: seed at `(v0, psi)` on the ground; tick 0 is a
bearing-forward wish with `jump: true`; every subsequent tick is `air_correct(v_xy, bearing_to_
target, a_max, dt, gain)` — the **exact** runtime air policy (`crates/rtx-nav/src/strafe.rs:170`),
with `forward`/`side` rounded to integers as the wire would (`jumps.rs:885`). It **rejects**:

- any `wall_contact` reported by `pm_step_report`, or a start-solid/all-solid hull trace before or
  after the step (`jumps.rs:890`, `jumps.rs:896`) — a curl is certified **zero-wall-contact**;
- a mid-flight bearing-sign flip while still far from the target (`jumps.rs:873`), which is a real
  overshoot the held-sign air-strafe would diverge from;
- falling more than 100u below the target's level (`jumps.rs:899`);
- a touchdown missing the target cell centre by more than `CURL_MISS_TOL = 24`u horizontally or
  `CURL_Z_TOL = 24`u vertically (`jumps.rs:902`, `physics.rs:111`).

**What certification therefore guarantees:** that a `pm_step` rollout against the map's collision
hull, at the game's air-acceleration and friction law, starting from a clean grounded state at one
of six enumerated launch corners and at one of three tick rates, reaches the ground within
`CURL_MAX_TICKS = 120` ticks of `1/77`s (`physics.rs:153`) inside a 24u box around the target
centre, without touching a wall.

**What it does not guarantee:**

- Nothing about **other entities**. The rollout is against static BSP only — no players, no doors,
  no movers, no projectiles.
- Nothing about **arriving in the certified state**. The certificate begins at the takeoff cell at
  the solved speed on the solved heading. Whether the live bot gets there is the runtime's problem
  (see the abort at `steer.rs:1376`).
- Nothing outside the **six corners**. The envelope is a lattice, not an interval hull; a live
  state between corners is not proven, merely bracketed. For the v3 family this gap is closed
  separately by a full runtime replay (`steer.rs:1288`); the plain curl family has no equivalent.
- Nothing about **tick rates outside 0.019–0.021s** or frametimes the server actually delivers
  under load.
- Nothing about **cost accuracy**. `cost = runup_len/((MAX_SPEED + v_req)/2) + airtime +
  CURL_COMMIT` (`jumps.rs:438`, `CURL_COMMIT = 0.3`, `physics.rs:137`) is a model, not a measurement.

### The entry-speed envelope (chained ground-turn family)

The plain curl funds its own takeoff speed from a local run-up, so its "envelope" is the ±3% hold
band above. The **chained** families cannot: the leap only closes with speed *carried in* from a
previous leg, and the launch heading is not the corridor heading, so the rotation has to happen on
the ground before the jump. Those links carry a `GroundTurnCurl` contract
(`crates/rtx-nav/src/navmesh/mod.rs:321`) with an explicit **entry envelope**:

```
entry_speed_lo / entry_speed_hi     horizontal speed at the link source (grounded arrival)
entry_yaw_lo   / entry_yaw_hi       velocity yaw360 at the link source
```

Where the bounds come from (v3, the optimal-sweep family, `jumps.rs:2260`):

- the **centre** is a rung of `GT_OPT_ENTRY_SPEEDS = [320, 340, 360]` (`jumps.rs:2502`), chosen —
  per its own comment — to bracket the corpus-calibrated carried DM3 entry band (p10/p50 ≈ 332/358
  u/s), distinct from the high-entry `GT_ENTRY_SPEEDS` ≈ 439/500 the v1 weave assumes
  (`jumps.rs:949`);
- the **half-width** is `GT_ENTRY_V_TOL = 0.02` and `GT_ENTRY_YAW_TOL = 12°` (`jumps.rs:951`,
  `jumps.rs:953`), optionally narrowed by `ENVELOPE_WIDTHS = [1.0, 0.5, 0.25, 0.125]`
  (`jumps.rs:2403`) when a narrower window is what certifies;
- the stamp happens at `jumps.rs:2434`.

So the widest possible v3 window at the 320 rung is `320 · (1 ∓ 0.02)` = **313.6 … 326.4 u/s**.
The executor fails closed against this envelope (`steer.rs:1253` finds a one-tick adjustment into a
parallel contract, `steer.rs:1271` aborts and replans otherwise) rather than improvising over the
lip. See the [known limitations](#known-limitations-and-open-defects) on why that number is a
calibration artifact rather than a physical bound.

## The emitted artifact

A certified curl becomes an ordinary `LinkKind::SpeedJump` link plus a `SpeedJumpTraversal`
(`crates/rtx-nav/src/navmesh/mod.rs:272`). The curl-relevant fields:

| field | meaning | plain curl (`jumps.rs:467`) | ground-turn v1/v2/v3 (`jumps.rs:1941`, `:2156`, `:2459`) |
|-------|---------|------------------------------|----------------------------------------------------------|
| `takeoff` | the certified leap point | solved leap point (slid back along the run-up) | the ledge cell origin |
| `v_req` | certified takeoff speed | ladder solution | certified entry rung |
| `airtime` | ballistic flight time | `jump_airtime(dz, g)` | same |
| `landing_speed_lo` | min certified touchdown carry the planner may credit | from the envelope | `gt.landing_speed_lo` |
| `chained` | needs carried entry speed | `false` | `true` |
| `curl_gain` | air-correct proportional gain (°/s per °) | certified gain from `CURL_GAINS` | `gt.air_gain` |
| `curl_entry_aim` | first pursuit point | `takeoff + dir(psi) · 512` — **carries the certified psi** (`jumps.rs:460`) | **`Vec3::ZERO`** |
| `curl_switch_dist` | signed gate distance along the takeoff→entry-aim axis | `−CURL_LIP_REACH` (`jumps.rs:478`) — negative means "switch on the first airborne frame" | **`0.0`** (disables the two-phase profile) |
| `curl_landing_aim` | second pursuit point | target cell origin | `gt.landing_aim` |
| `ground_turn` | full certified controller contract | `None` | `Some(GroundTurnCurl)` |
| `rollout_successor` | the ordinary cell the certified rollout continued into | `None` | `Some(..)` for v3 |

Read that table carefully: **the two-phase aim profile is populated only by the plain curl family.**
The ground-turn families emit `curl_entry_aim = ZERO` and `curl_switch_dist = 0`, which is the
sentinel that disables the profile (`mod.rs:291`); their lateral geometry lives in the
`GroundTurnCurl` instead (`runway_aim`, `runway_yaw`, `launch_yaw`, `hold_aim`, `gate_point`,
`gate_normal`, `landing_aim` — `mod.rs:327`–`mod.rs:357`).

The `−CURL_LIP_REACH` switch distance on plain curls is doing something subtle: it is *negative*,
so `travelled < switch_dist` is false the moment the bot passes the takeoff. The entry aim
therefore owns every **grounded** frame and the launch frame, and the landing aim owns the whole
flight — which is exactly what `curl_lands` rolls (bearing-forward on tick 0, bearing-to-target
thereafter). It is a launch-axis certificate, not a mid-flight hold.

## The runtime side

### Deriving the runway and the progress axis

`steer.rs` pulls the traversal for the committed leg at `steer.rs:1332` and reads three things off
it: `sj_takeoff` (`:1337`), `sj_curl_gain` / `sj_curl` (`:1339`), and `sj_gt` (`:1344`).

The **profile axis** (`steer.rs:1354`) is `normalize(curl_entry_aim.xy − takeoff.xy)`, taken only
when `ground_turn.is_none() && curl_switch_dist ≠ 0` — i.e. only for a profiled plain curl. The
**progress** scalar (`steer.rs:1358`) is the signed along-axis distance to the takeoff:

```
progress = dot(takeoff.xy − origin.xy, axis)      > 0 behind the lip, < 0 past it
```

with a fallback to the snapped `from`-cell → takeoff chord when no profile axis exists
(`steer.rs:1360`). Using a signed *line* crossing rather than a radial ball matters: a radial
trigger can be skirted into a U-turn by the ground weave.

`progress` is then fed to the hop controller as `bhop_runway` (`steer.rs:1503`), so "past the lip"
is a sign change. Ground-turn links bypass this entirely and synthesize the signal from their own
launch gate (`steer.rs:1488`, dispatching on `gt.version` to
`ground_turn_should_launch_optimal` at `jumps.rs:2585` for v3).

### The two-phase flight aim

`sj_flight_aim` (`steer.rs:1432`) is the two-phase pursuit:

```rust
if on_ground || travelled < tr.curl_switch_dist { tr.curl_entry_aim } else { tr.curl_landing_aim }
```

and it is selected into the steering bearing at `steer.rs:1466` (grounded — the certifier's stored
axis owns every grounded frame, not merely the jump pulse) and `steer.rs:1475` (at the lip and
through the flight). The comment at `steer.rs:1463` names the bug this closed: steering at the
snapped nav-source chord launched ring curls tens of degrees away from the line their flight had
proved. `steer.rs:1470` keeps the run-up aimed at the takeoff while still more than a lip-reach
behind it, so the bot never curls toward the offset landing while still over the run-up and pulls
itself off the edge.

### `bhop.rs` — firing the jump at the lip

The curl's takeoff is a distinct **regime** inside the hop controller, gated on `takeoff_speed >
0`, which `steer.rs:1538` sets to `v_req` only for curl-flagged jumps (a straight speed jump keeps
the pre-existing hop chain, because its air-strafe runway can exceed the prestrafe ceiling the
hold-to-lip regime would cap it below — `steer.rs:1534`).

Frame by frame:

```
approach   engage() -> Phase::Prestrafe even below the 512u runway gate, because a
           committed jump below v_req must ground-circle-strafe to make it   (bhop.rs:556)
             |
runway     Prestrafe: launch condition for a curl is ONLY "airborne or runway < LIP_REACH"
           (bhop.rs:388) — never the generic speed/time/runway-out triggers. Each frame is
           takeoff_cmd: above v_req·1.03 coast on the bearing and let friction bleed
           down; below it, circle-strafe to build                            (bhop.rs:513)
           The weave deadband is clamped to launch_yaw_tol = CURL_PSI_TOL when a profile
           exists (bhop.rs:521, steer.rs:1565) — the ordinary wide weave is a speed policy,
           not licence to leave the proven lateral line.
             |
lip        hop_cmd, grounded, sj_takeoff: hold takeoff_cmd while runway >= LIP_REACH
           (bhop.rs:439). Once past, a launch-yaw guard vetoes the frame if the bearing and
           the velocity differ by more than launch_yaw_tol (bhop.rs:446) — it keeps
           prestrafing instead of leaping off-profile, and counts the veto.
             |
launch     jump = !jump_prev; the launch cmd is a pure bearing-forward wish with jump: true
           and no slalom lobe (bhop.rs:463) — identical to the certifier's tick 0.
             |
flight     airborne: air_correct(v_xy, bearing, a_max, dt, curl_gain)        (bhop.rs:415)
           — pursuit guidance, not the hop slalom, whose lobe flips scatter the landing.
           `bearing` comes from sj_flight_aim, so this is the landing aim.
             |
landing    touchdown advances the leg; the ground-turn families additionally require the
           certified rollout_successor as the next edge (mod.rs:307).
```

The relationship to the certified psi is therefore: the solver picks `psi`, bakes it into
`curl_entry_aim`; the runtime derives the launch axis back out of that field, steers the ground
prestrafe along it with the weave clamped to ±`CURL_PSI_TOL`, and refuses to leap while the
velocity is outside that same tolerance. Certified and executed lateral axis are the same quantity
— **for the plain curl family**. See the limitations section for where they are not.

### Fail-safes

- **Too-slow abort** (`steer.rs:1376`): the takeoff regime leaps a curl *unconditionally* at the
  lip, so before that the runtime predicts the lip speed from the current state via
  `prestrafe_delivered_from` (`physics.rs:167`) over the remaining `progress`, and if it comes out
  below `v_req · 0.85` it penalizes the leg, drops the route and replans rather than leaping short
  into the pit.
- **Stall watchdog**: a speed-jump leg is abandoned, penalized and re-pathed after 4s
  (`steer.rs:1074`), so a run-up that never builds speed cannot wedge on the runway forever.
- **v3 full-witness replay** (`steer.rs` ground-turn entry path): before the first v3 controller
  frame may launch, the entire remaining pmove is replayed from the observed live state against the
  BSP. During an airborne setup the executor also retains the admitted controller/movement clock
  pair and revalidates it every frame; a changed schedule aborts rather than inheriting the latch.

## Operator cvars

House format, as in [cvars.md](cvars.md). These belong under *Development & tuning* — they are
bring-up knobs, not play-server settings.

| cvar | default | effect |
|------|---------|--------|
| `rtx_jump_curl_entry_x` | `0` | World **X** of the first pursuit point for a hand-planted curl. Units. |
| `rtx_jump_curl_entry_y` | `0` | World **Y** of the same point; its Z is taken from the plant's takeoff (`control.rs:1506`). |
| `rtx_jump_curl_switch_dist` | `0` | Units travelled from the takeoff along the takeoff→entry-aim axis before the aim switches to the landing point. **A nonzero value is what enables the two-phase profile at all**; `0` keeps the historical single-target curl (`mod.rs:291`). Negative = switch on the first airborne frame. |
| `rtx_jump_curl_landing_x` | `0` | World **X** of the second pursuit point. |
| `rtx_jump_curl_landing_y` | `0` | World **Y** of the same; its Z is the target cell's (`control.rs:1511`). |

Registered at `crates/rtx-game/src/cvars.rs:197`–`:201`.

**How they interact with certified values: they do not.** These five are read in exactly one place
— `plant_link_json` (`control.rs:1466`), reached from the `planlink` control-channel verb
(`control.rs:576`):

```text
<id> planlink <fx> <fy> <fz> <ox> <oy> <oz> <tx> <ty> <tz> <v_req>
```

They populate a **hand-planted** link's traversal and never override a generated one. A planted
link also flies straight by default: `curl_gain` comes from `rtx_jump_curl_gain` and is `0` unless
set (`control.rs:1492`). The comment at `control.rs:1486` records why — the old bring-up default
baked a firm gain into every plant, which homed every planted flight onto these aim cvars, and on a
server not doing curl bring-up they are `0`, so plants were pulled toward the map origin and landed
180–230u off.

Two related knobs (`crates/rtx-game/src/cvars.rs:193`–`:194`) apply to **plain jump legs**, not to
certified curls: `rtx_jump_curl_hold` (fraction of the gap flown on the takeoff heading before the
air-curl engages, `steer.rs:2084`) and `rtx_jump_curl_gain` (gain override, `steer.rs:1554` for
certified curls, `steer.rs:1885` for the plain-jump band-aid). A ground-turn curl ignores the gain
cvar entirely — its contract is the law (`steer.rs:1547`).

## The RA jump

This is the route we care about most: **DM3, RA tunnel → red-armour top**. It is the acceptance
case the whole curl treatment was built against, and the one worth arguing about.

### Geometry and the link family

The chain is diagnosed end to end by `crates/rtx-nav/tests/dm3_ra_curl_coverage_probe.rs`, which
is read-only and env-gated on `RTX_TEST_BSP`. Two landing sets are named there:

- **gap-1** — the mid floor at `(352…384, −544…−576, 56)` and `(320, −544…−576, 72)`
  (`dm3_ra_curl_coverage_probe.rs:50`);
- **upper** — the RA stair foot at `(32…96, −832…−864, 152…184)`
  (`dm3_ra_curl_coverage_probe.rs:61`).

The upper leap — the one onto the RA stair — is carried by the **chained ground-turn family**, and
the test asserts it: `upper_covered` requires at least one link landing on the stair set whose
traversal has `ground_turn.is_some()` (`dm3_ra_curl_coverage_probe.rs:199`–`:207`). The segment
`(256, −864, 32) → (384, −576, 56)` at a carried 493 u/s must plan under 1.2s
(`:210`–`:213`), and the full RA-spawn → RA banded route must come in under 9.565s when curl
generation is on (`:264`–`:266`). With `curl = false` the same test prints the route for
comparison; the assertions are curl-only.

### What the curl does differently here, and why the straight treatment fails

Three separate reasons, in the order the solver hits them:

1. **The chord is blocked.** The approach runs along the tunnel corridor; the RA stair foot sits
   off that heading. `arc_clear_peak` over the straight chord fails, so the straight `SpeedJump`
   pass emits nothing (`jumps.rs:383`, and the "only curl what the straight pass could not own"
   gate at `jumps.rs:385`).
2. **No local run-up funds the speed.** The module comment at `jumps.rs:915` is explicit: this leap
   family closes *only* with carried entry speed — `prestrafe_delivered` saturates near 430 while
   the flight needs ~470. There is no corridor behind the lip long enough to fix that, which is why
   the link is `chained: true` and the banded planner must prove the entry band before it may
   route it.
3. **The rotation cannot happen in the air.** Same comment: rotating after a lip launch provably
   cannot close the flight-time budget, because air acceleration is capped. So the turn is done
   *on the ground* in the final `turn_dist` before the jump (`mod.rs:336`), and the jump fires on a
   **yaw-and-box gate** (`ground_turn_should_launch_optimal`, `jumps.rs:2585`: grounded, inside the
   takeoff XY box, and within `GT_OPT_LAUNCH_SLACK = 8°` of `launch_yaw` on the approach side)
   rather than on crossing a takeoff line.

So the run-up/ledge approach looks like this: the bot arrives at the link source already fast and
grounded, having carried speed from the previous leg; the entry state is checked against the stored
envelope and fails closed if outside (`steer.rs:1253`/`:1271`); the grounded controller then holds
the ground-optimal wish offset off the *current velocity* — `θ = acos(u*/speed)` with `u* =
maxspeed − accel·maxspeed·dt` (`jumps.rs:2510`) — sweeping the velocity single-sidedly toward
`launch_yaw` while still accelerating; the jump fires on the first grounded tick that satisfies
the gate; the flight runs launch gain then air gain onto `landing_aim`.

The v1/v2 weave law by contrast follows a *position-scheduled* world bearing and weaves its strafe
sign to recentre onto it, and that recentring caps the exit speed a low-carried-entry runway can
build (`jumps.rs:2513`). That is the whole reason the v3 optimal-sweep family exists.

### Independent acceptance

The route is also gradeable live, independent of the navmesh: the control channel's `ra_trial`
verb, documented at [development.md § DM3 Ring→RA acceptance trial](development.md#dm3-ringra-acceptance-trial).
It seeds the RA item goal, runs ordinary A*/traversal/pickup code unchanged, and hard-fails on
`planned_drop`, `fall`, sustained BSP-blocked `wall_push`, `no_pickup`, `stall`, death or timeout —
so a movement change cannot pass merely by reaching a coordinate near RA. Its `ra_spawn` deadline
(p50 = 12.6255s) is calibrated from an admitted same-life set of 86 human runs across 77 demos;
those are aggregate thresholds only, never routes or input sequences.

## Interaction with `rtx_bot_ledgecap`

`rtx_bot_ledgecap` is upstream's careful-ledge walk-speed cap: default `210` u/s, `0` = full
maxspeed (upstream `crates/rtx-game/src/cvars.rs:86`, documented upstream
`docs/cvars.md:88`). On a navmesh cell flagged beside a fatal drop it scales the wish by
`cap / MOVE_SPEED` for the **whole run**, not just at corners (upstream `steer.rs:1125`), because
the 96u `TURN_SLOW_RADIUS` approach slowdown bites too late to bleed a full-speed straight's
momentum before the lip.

The tension with curls is direct and structural: **a ledge cap is a policy that says "be slow near
drops"; a curl is a link that says "be at exactly `v_req` at the drop."** A curl's takeoff is a
ledge over a pit by construction (`jumps.rs:334` requires no ground the leap way), and the
certificate begins at a speed the cap forbids — 210 u/s is far below every certified `v_req` in
this family, and the runtime's own too-slow abort trips at `v_req · 0.85` (`steer.rs:1393`).

Upstream already anticipates this and exempts jump run-ups (upstream `steer.rs:635`):

```rust
let is_jump = |l| matches!(graph.link_kind(l), JumpGap | DoubleJump | SpeedJump);
let jump_at_hand = cur_leg.is_some_and(&is_jump)
    || bot.route.get(bot.route_pos + 1).is_some_and(|&l| is_jump(l));
let on_ledge = graph.is_ledge(bot_cell) && !jump_at_hand;
```

That exemption covers the current leg and the next one. The structural question a reviewer should
press on is what happens **two legs out**. A plain curl's `from` cell is placed up to
`CURL_RUNUP_CAP = 512` units back from the takeoff (`physics.rs:101`), so the run-up is inside the
curl leg and exempt. A *chained* curl has no self-contained runway at all — the speed has to be
carried in over the preceding Walk/Step legs, and on those legs `jump_at_hand` is false, so a
flagged ledge cell caps the approach at 210 u/s and the bot arrives below the certified entry
envelope's floor. The fail-closed check then does its job and refuses the leg, which is correct
behaviour and a dead route.

**Current measured status.** The rebased branch now passes two consecutive 20-attempt RA-spawn
streaks (**40/40**) with near-field steering enabled and prohibited movement assists, including
the ledge cap, disabled. The failures were not fixed by a cvar workaround. Their two code causes
were approach steering acting several ordinary legs before a certified jump, and a v3 setup latch
that assumed a future frame cadence it had not proved. The current implementation scopes the
approach forces by the upcoming link contract and requires a full physical rollout under a
per-frame-revalidated controller/movement clock. The structural tension above remains useful when
reviewing upstream ledge policy, but this branch neither depends on nor certifies the ledge cap.

## Known limitations and open defects

**Lateral controller state is family-specific, not discarded.** For the **plain** curl family,
psi is threaded through the solver's `solved` tuple and emitted as the certified entry aim,
negative immediate-air switch and landing aim. A regression test pins it:
`dm3_ring_legacy_curls_preserve_the_certified_lateral_profile` builds DM3, takes two ring curls,
and asserts (a) that the preserved certified line lands at live-class speed, (b) that the
previously-observed off-profile launch line still does *not* land — kept as a red witness — and
(c) that none of the three aim fields is zero. Ground-turn curls deliberately retain the zero
two-phase sentinel because their lateral proof lives instead in `GroundTurnCurl` (`runway_yaw`,
`launch_yaw`, `hold_aim`, launch gate and `landing_aim`). Their runtime admission and setup use
that complete contract in the full-witness replay; treating the legacy fields as a second source
of truth would be wrong.

**The v3 entry envelope is a calibration artifact, not a physical bound.** There is exactly one
production entry ladder for the optimal-sweep family, `GT_OPT_ENTRY_SPEEDS = [320, 340, 360]`
(`jumps.rs:2508`), and at the 320 rung with the widest tolerance the emitted envelope is
313.6–326.4 u/s. The ladder was chosen to bracket a corpus-measured carried band (p10/p50 ≈
332/358), and the coverage probe's plausibility assertion hard-codes the resulting floor
(`assert!(gt.entry_speed_lo >= 313.0)`, `dm3_ra_curl_coverage_probe.rs:183`). The same corridor is
observably clearable at roughly 440 u/s. Nothing in the physics makes 326 a ceiling — it is
simply the highest rung this solver certified. A bot arriving hot is therefore refused the
available contract even though the corridor is physically flyable. The remedy is not to widen
the stamped envelope after the fact: a high-speed profile must itself pass the BSP/cadence
certificate and the same runtime full-witness replay. Candidate search cost is one constraint,
but the missing certificate is the correctness boundary.

*(The ~440 u/s figure is calibration evidence from recorded human play, not something derivable
from this repository. It is stated as a measured speed only.)*

**Other things the code makes obvious:**

- **Budgets are tiny and cost-ordered.** Two curls per source cell (`physics.rs:139`) and two per
  target cell (`physics.rs:142`), truncated after a sort on modelled cost (`jumps.rs:237`). A
  cheaper link from a corridor the bot can never actually enter at speed can evict the one it would
  really arrive through. The chained pass partially compensates by de-duplicating on
  `(source, exact entry envelope)` instead of source alone (`jumps.rs:280`), on the reasoning that
  cold, mid and hot arrivals are disjoint executable contracts and the planner's five speed bands
  are too coarse a key — but the plain curl pass has no such protection.
- **Build cost.** Certification is `6 corners × 3 tick classes × up to 8 gains × up to 25 speed
  rungs × 5 psi samples × takeoff-backoff steps` of full `pm_step` rollout per candidate target.
  The cheap-scout prefilters (`jumps.rs:425`, `jumps.rs:793`) exist because the pass is otherwise
  roughly 50× slower. This is why `rtx_bot_curljump` defaults to `0`.
- **The plain curl family has no runtime replay.** v3 gets a full pmove replay from the live state
  before it may launch (`steer.rs:1288`); the plain curl family gets only the predicted-lip-speed
  abort (`steer.rs:1376`). The asymmetry is not principled, just historical.
- **Low-gravity servers get no curls at all** — the whole scan is skipped when a flat leap outlasts
  the rollout tick cap (`jumps.rs:321`).
- **Cost model vs. certificate.** The link's planner cost is a closed-form estimate
  (`jumps.rs:438`) while its feasibility is a rollout. The two can disagree; only the rollout is
  authoritative.

## How to try it

Build the module as in [development.md](development.md#building), then on the server:

```
set rtx_bot_curljump 1        // curl generation (rebuilds the navmesh at map load)
set rtx_bot_bhop 1            // parent toggle — 0 disables curls regardless
set rtx_bot_bandplan 1        // the banded planner; chained curls are unroutable without it
set rtx_bot_debug 1           // per-leg jump-profile lines to the console
map dm3
set rtx_bot_count 1
```

Map and route: **dm3**, RA tunnel → red armour. Put the bot on the route by seeding the RA item
goal with the control channel's `ra_trial` verb (see
[development.md](development.md#dm3-ringra-acceptance-trial)) rather than a coordinate `goto`, so
ordinary planning and pickup code runs.

What to watch:

- **The console jump-profile line** (`steer.rs:1975`) prints `curl_gain`, `entry_aim`,
  `switch_dist` and `landing_aim` for the current speed-jump leg. `entry_aim=(0,0,0)` with
  `switch_dist=0` means the ground-turn family — the profile is in the contract, not these fields.
- **The lip.** The bot should hold a *steady* speed down the corridor rather than accelerating into
  the jump — that is `takeoff_cmd` coasting above the ±3% band (`bhop.rs:514`). If it visibly
  accelerates to the edge, `takeoff_speed` is not reaching the controller.
- **The launch heading.** The leap should go down the corridor, not at the platform. If it leaves
  the lip aimed at the landing, the profile axis is not being used.
- **The carve.** One smooth arc onto the platform, no slalom lobes. Lobes mean `curl_gain` is 0 and
  the hop slalom is running instead of `air_correct`.
- **Aborts.** With `rtx_bot_debug 1`, watch for the leg being penalized and re-pathed — that is
  either the too-slow abort (`steer.rs:1393`) or a v3 entry-envelope rejection
  (`steer.rs:1271`/`:1322`), and they mean different things.

For a single geometry without a full build, the control channel also exposes a `curl` probe verb
backed by `NavGraph::curl_probe` (`jumps.rs:688`), which reports the delivered takeoff speed,
whether the envelope certifies, and the per-gain landing points — the miss distances are the *why*
behind a rejection.

---

*See also: [the bots](bots.md#curl-jumps) · [cvar reference](cvars.md) ·
[development & tooling](development.md)*
