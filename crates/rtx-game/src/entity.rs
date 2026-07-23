// SPDX-License-Identifier: AGPL-3.0-or-later

//! The entity type and its handle.
//!
//! [`Entity`] is `#[repr(C)]` so that its [`EntVars`] prefix sits at offset 0 — the engine
//! strides the array by `size_of::<Entity>()` (reported as `sizeofent`) and reads/writes
//! that prefix directly. Everything after `v` is the *private tail*: ordinary Rust state
//! the engine never touches, so it may use any types (enums, `Option`, etc.).

use std::ffi::CString;

use glam::Vec3;

use crate::abi::{EntVars, STRING_REF_COUNT};
use crate::assets::{Model, Sound};
use crate::bot::state::BotState;
use crate::mode::ModePlayer;

/// A `Copy` index handle into the entity array. Never a borrow, so holding one across a
/// trap call is fine — this is what keeps the safe API free of aliasing hazards.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub struct EntId(pub u32);

impl EntId {
    /// The world entity is always index 0.
    #[allow(dead_code)]
    pub const WORLD: EntId = EntId(0);

    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// `EDICT_TO_PROG` — the byte offset the engine stores in entvars `.entity` fields.
    #[inline]
    pub fn to_prog(self) -> i32 {
        (self.index() * core::mem::size_of::<Entity>()) as i32
    }

    /// `PROG_TO_EDICT` — recover an index from an entvars `.entity` byte offset.
    #[inline]
    pub fn from_prog(prog: i32) -> EntId {
        EntId((prog as usize / core::mem::size_of::<Entity>()) as u32)
    }

    /// Whether this is a real (non-world) entity reference.
    #[inline]
    pub fn is_some(self) -> bool {
        self.0 != 0
    }
}

/// A scheduled `think` behaviour. Modeled as a data enum (not a fn pointer) so it is
/// `Debug`/`Eq` and the central dispatcher's `match` is exhaustiveness-checked. QuakeC's
/// think-chains map to a variant per chain entry; animation cursors live in `walkframe`.
// Variant names mirror their QuakeC `*_think` functions, hence the shared `Think` suffix.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Think {
    #[default]
    None,
    // --- subs.qc movement machinery ---
    /// `SUB_CalcMoveDone` — snap to `finaldest`, then run `think1`.
    SubCalcMoveDone,
    /// `SUB_Remove` — free this entity.
    SubRemove,
    /// `DelayThink` — fire a delayed `SUB_UseTargets`.
    DelayedUse,

    // --- player animation loops (player.qc) ---
    PlayerStand,
    PlayerRun,
    /// One-shot body-frame run from the current frame to `anim_end`, then `anim_after`
    /// (used for pain and death sequences).
    PlayerAnim,
    /// Terminal of a death sequence (`PlayerDead`: freeze, mark `DeadFlag::Dead`).
    PlayerDead,
    /// Cosmetic weapon firing animation (shotgun/rocket/axe), parameterized by the
    /// `anim_*` tail fields; fires `W_FireAxe` at `anim_fire` if set, then `player_run`.
    PlayerWeaponAnim,
    /// Looping nailgun fire (alternating left/right spikes) while attack is held.
    PlayerNail,
    /// Looping lightning fire while attack is held.
    PlayerLight,

    // --- projectiles (weapons.qc) ---
    /// Grenade timed explosion (rockets/spikes are touch-driven and need no think).
    GrenadeExplode,

    // --- items.qc ---
    /// `SUB_regen` — re-show a picked-up item after its respawn delay.
    SubRegen,
    /// `PlaceItem` — drop an item to the floor after spawn.
    PlaceItem,
    /// `item_megahealth_rot` — rot a megahealth recipient back to max health.
    MegaHealthRot,

    // --- doors.qc ---
    DoorLink,
    DoorGoDown,
    DoorHitTop,
    DoorHitBottom,

    // --- plats.qc ---
    PlatGoDown,
    PlatHitTop,
    PlatHitBottom,
    TrainNext,
    TrainWait,
    FuncTrainFind,

    // --- buttons.qc ---
    ButtonWait,
    ButtonReturn,
    ButtonDone,

    // --- triggers.qc ---
    MultiWait,
    HurtOn,
    PlayTeleport,
    ExecuteChangelevel,

    // --- rotate.rs (Hipnotic rotating brushes) ---
    /// `func_rotate_entity` deferred setup (links targets once they've spawned).
    RotateEntityFirstThink,
    /// `func_rotate_entity` continual-spin tick.
    RotateEntityThink,
    /// `func_rotate_train` per-tick move+rotate; runs `think1` when a segment ends.
    RotateTrainThink,
    /// Train `think1` continuations (find start, advance, wait, stop).
    RotateTrainFind,
    RotateTrainNext,
    RotateTrainWait,
    RotateTrainStop,
    /// `func_rotate_door` swing tick and its arrival follow-up.
    RotateDoorThink,
    RotateDoorThink2,
    /// `func_movewall` keep-alive tick (holds `ltime` current for the pusher).
    MovewallThink,

    // --- bob.rs ---
    /// `func_bob` sine-bob tick.
    FuncBobThink,

    // --- grapple.rs (grappling hook) ---
    /// Player viewmodel/body hold animation while a hook is out (`player_hook`/`player_chain`).
    GrappleAnim,
    /// Hook follows what it's anchored to (and damages a hooked player).
    GrappleTrack,
    /// Hook's deferred spawn of the chain-link entities.
    BuildChain,
    /// Lead chain link repositions all three links each frame.
    UpdateChain,
    /// Lead chain link removes the whole chain.
    RemoveChain,
    /// CTF flag idle tick: auto-return a dropped flag once its timeout elapses.
    FlagReturn,
    /// CTF rune idle tick: relocate to a fresh spawn if untouched too long.
    RuneRespawn,
}

/// A `.touch` behaviour (`GAME_EDICT_TOUCH`). The dispatcher reads `self`/`other` and
/// matches on this. Mirrors the structure of [`Think`].
// Variant names mirror their QuakeC `*_touch` functions.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Touch {
    #[default]
    None,
    // projectiles
    Spike,
    SuperSpike,
    Grenade,
    Missile,
    // items.qc
    ItemHealth,
    ItemArmor,
    ItemWeapon,
    ItemAmmo,
    ItemPowerup,
    Backpack,
    // map entities
    DoorTriggerField,
    DoorTouch,
    PlatCenter,
    ButtonTouch,
    Multi,
    Teleport,
    Hurt,
    Push,
    TriggerMonsterjump,
    Tdeath,
    Changelevel,
    /// `func_movewall` clip brush: damages the player it touches (if armed).
    Movewall,
    /// Grappling-hook head: anchor to whatever it strikes.
    Hook,
    /// CTF flag: grab (enemy), return (own, dropped), or capture (own base while carrying enemy).
    Flag,
    /// CTF rune pickup (one per player).
    Rune,
}

/// A `.use` behaviour, fired by `SUB_UseTargets` / button presses.
// Variant names mirror their QuakeC `*_use` functions, hence the shared `Use` suffix.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Use {
    #[default]
    None,
    DoorUse,
    PlatTrigger,
    PlatUse,
    TrainUse,
    ButtonUse,
    MultiUse,
    TeleportUse,
    TriggerRelay,
    CounterUse,
    LightUse,
    FuncWallUse,
    // rotate.rs
    RotateEntityUse,
    RotateTrainUse,
    RotateDoorUse,
}

/// A `.blocked` behaviour for `MoveType::Push` movers (`GAME_EDICT_BLOCKED`).
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Blocked {
    #[default]
    None,
    DoorBlocked,
    PlatBlocked,
    TrainBlocked,
    /// `func_movewall`: damages whoever blocks it and reverses a rotating-door group.
    MovewallBlocked,
}

/// A `.th_pain` behaviour, invoked by `T_Damage`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Pain {
    #[default]
    None,
    Player,
}

/// A `.th_die` behaviour, invoked by `Killed`.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Die {
    #[default]
    None,
    Player,
    GrenadeExplode,
    DoorKilled,
    ButtonKilled,
    TriggerKilled,
    ExploBoxDie,
}

/// A brush mover's phase (QuakeC `STATE_*`): at rest at the bottom/top, or in motion up/down. This
/// is a crate-owned scratch field (`MoverState::state`), not an engine-shared entvar, so it's a real
/// enum rather than an `f32` compared with `==`. `Top` is the default (QuakeC's `STATE_TOP = 0`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum MoverPhase {
    #[default]
    Top,
    Bottom,
    Up,
    Down,
}

/// How a brush targeted by a rotator follows it (hiprot's `rotate_type`), assigned by
/// `link_rotate_targets` from the target's classname.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RotateType {
    /// Plain follower: origin tracks the rotated offset from the centre.
    #[default]
    SetOrigin,
    /// `rotate_object`: also copies the rotator's angles, so the brush visibly turns.
    Rotate,
    /// `func_movewall`: a `MoveType::Push` clip brush driven by velocity, so it pushes/crushes.
    Movewall,
}

/// State-machine position for a rotator. The QC overloaded one integer `state` with three
/// unrelated meanings (continual spin / door swing / train); this names them and splits them
/// by entity. Each rotator type uses only its own subset.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RotPhase {
    #[default]
    None,
    // func_rotate_entity (continual spin)
    Active,
    Inactive,
    SpeedingUp,
    SlowingDown,
    // func_rotate_door
    Closed,
    Open,
    Opening,
    Closing,
    // func_rotate_train — only `Moving` is behaviourally read (whether to snap on arrival);
    // the find/next/wait/stop steps are tracked by `think1`.
    Moving,
}

/// One game entity: the engine-shared `v`, followed by the private Rust tail.
///
/// QuakeC `.string` fields (classname, target, ...) live here as owned strings rather
/// than in the engine-shared `v` — the engine learns about a string only through explicit
/// trap calls (e.g. `setmodel`), never by reading the struct.
#[repr(C)]
#[derive(Default)]
pub struct Entity {
    /// Engine-shared fields (offset 0). See [`EntVars`].
    pub v: EntVars,

    /// Backing cells for the native string ABI (see [`EntVars::link_string_refs`]). The
    /// engine stores each `.string` field's resolved `char*` here. They sit first in the tail
    /// so they live *inside* the entity array, which the engine's offset arithmetic requires.
    pub string_refs: [u64; STRING_REF_COUNT],

    /// `.maxspeed` — the player's ground/air move speed cap. An *extended* entvars field, declared
    /// in the `fields` table handed to the engine at `GAME_INIT` so the engine addresses it at this
    /// offset (it must therefore live inside the entity array). mvdsv clamps pmove `wishspeed` to a
    /// per-client `cl->maxspeed`, which it seeds *from this field* every frame — but only when the
    /// mod declares a `maxspeed` field. A bot's `cl->maxspeed` is otherwise left at `0` by
    /// `PF2_Add_Bot` (it never runs the normal spawn's initializer), which caps its move speed to
    /// zero: the bot can jump but not walk. Declaring this field and setting it on spawn is how the
    /// engine's sync gets a sane cap onto every client, bots included. Set in `put_client_in_server`.
    pub maxspeed: f32,

    // --- private tail: engine never addresses these ---
    /// Whether this slot is currently a live entity.
    pub in_use: bool,
    /// Network client only: the game time this entity was last updated from the wire. A player is
    /// sent every frame it's in our PVS, so a stale stamp means it left our view — walked behind a
    /// wall or teleported across the map — and its shadow is frozen at the last spot we saw it. Combat
    /// and perception read this (via [`GameState::net_shadow_stale`]) so a live line of sight to that
    /// frozen spot isn't mistaken for a live target. Always `0.0` server-side (the edict is live).
    pub net_seen: f32,

    // callbacks (QuakeC `.think`/`.touch`/`.use`/`.blocked`/`.th_pain`/`.th_die`)
    pub think: Think,
    /// Secondary think (`think1`): the function `SUB_CalcMove*` runs once the move finishes.
    pub think1: Think,
    pub touch: Touch,
    pub use_: Use,
    pub blocked: Blocked,
    pub th_pain: Pain,
    pub th_die: Die,

    // strings (owned on our side; engine sees them only via traps)
    pub classname: Option<Box<str>>,
    pub model: Option<Box<str>>,
    pub weaponmodel: Option<Box<str>>,
    pub target: Option<Box<str>>,
    pub targetname: Option<Box<str>>,
    pub killtarget: Option<Box<str>>,
    pub message: Option<Box<str>>,
    pub netname: Option<Box<str>>,
    // Mover sound slots (door/plat/button). Typed `Sound` handles so they're provably precached;
    // these are our own state, distinct from the engine-visible `EntVars.noise*` string slots.
    pub noise: Option<Sound>,
    pub noise1: Option<Sound>,
    pub noise2: Option<Sound>,
    pub noise3: Option<Sound>,
    pub noise4: Option<Sound>,
    pub deathtype: crate::obituary::DeathType,
    pub mdl: Option<Box<str>>,
    /// `trigger_changelevel`'s destination map name.
    pub map: Option<Box<str>>,
    /// `func_rotate_train` path chain: the `path` map key (first corner), then the running
    /// cursor as the train advances corner to corner.
    pub path: Option<Box<str>>,
    /// `path_rotate`'s corner event target — fired via `SUB_UseTargets` when the train arrives.
    pub event: Option<Box<str>>,
    /// `func_rotate_door`'s group name, so a whole door group reverses direction together.
    pub group: Option<Box<str>>,
    /// The item's model as a `'static` C string, kept so a respawned item can be re-shown
    /// (the engine stores the raw pointer, so it must outlive the entity — see the native
    /// string ABI notes in `abi.rs`). Item models are all string literals, so this is sound.
    pub model_cstr: Option<Model>,
    /// Owned C string backing a brush model (`*N`) passed to `setmodel`. The engine keeps the
    /// raw pointer, so this must live as long as the entity references the model.
    pub model_cs: Option<CString>,
    /// Owned C string backing the engine-visible `v.netname` StringRef (the client name the
    /// engine syncs from). Kept alive as long as the entity references it.
    pub netname_cs: Option<CString>,

    pub anim: AnimState,
    pub mover: MoverState,
    pub rot: RotState,
    pub bob: BobState,
    pub grapple: GrappleState,
    pub bot: BotState,
    pub refs: CustomRefs,
    pub combat: CombatState,
    pub item: ItemState,
    /// Per-player spawn-selection memory (KTX k_spw 4). Deliberately outside [`CombatState`]:
    /// `put_client_in_server` wipes combat *before* the spawn is selected, and this must
    /// survive respawns. The map-load edict sweep defaults the whole entity, so `last_spot`
    /// can't dangle across maps.
    pub spawn: SpawnState,
    /// Per-player game-mode state (arena role, team, CTF carry/runes). Default in FFA.
    pub mode_p: ModePlayer,
    /// CTF flag state — only meaningful on the two flag entities (`flag.team != 0`).
    pub flag: FlagState,
    /// Race-route map keys — only meaningful on `race_route_start`/`race_route_marker`
    /// entities, which live just long enough to be folded into `GameState.race` at load.
    pub race: RaceEnt,
}

#[derive(Default)]
pub struct AnimState {
    /// Walk/idle animation cursor for player movement loops; also a generic frame cursor.
    pub walkframe: i32,
    /// One-shot body animation: final frame for [`Think::PlayerAnim`].
    pub anim_end: i32,
    /// What to do after a one-shot player animation completes.
    pub anim_after: Think,
    /// Cosmetic weapon-animation parameters (see [`Think::PlayerWeaponAnim`]): body-frame
    /// base, weaponframe base, frame count, and the cursor index that fires the axe (-1 = no
    /// in-animation fire, e.g. shotgun/rocket which fire from `W_Attack` directly).
    pub anim_base: i32,
    pub anim_wf_base: i32,
    pub anim_len: i32,
    pub anim_fire: i32,
    pub anim_muzzle: i32,
}

#[derive(Default)]
#[allow(dead_code)]
pub struct MoverState {
    // movement / mover scratch (subs.qc, doors, plats, buttons, trains)
    pub finaldest: Vec3,
    pub finalangle: Vec3,
    pub dest: Vec3,
    pub dest1: Vec3,
    pub dest2: Vec3,
    pub pos1: Vec3,
    pub pos2: Vec3,
    pub mangle: Vec3,
    pub speed: f32,
    pub wait: f32,
    pub delay: f32,
    pub lip: f32,
    pub height: f32,
    pub state: MoverPhase,
    pub dmg: f32,
    pub count: f32,
    pub cnt: f32,
    pub t_length: f32,
    pub t_width: f32,
    pub pausetime: f32,
}

/// Scratch for the Hipnotic rotating-brush system (`rotate.rs`). A rotator spins its targeted
/// brushes around its own origin by recomputing their positions each tick; these fields hold
/// the rotation rate, the per-tick scratch offset, and the small state machines. The shared
/// vector/float scratch (`dest`/`dest1`/`dest2`/`finaldest`/`finalangle`, `speed`/`wait`/`dmg`/
/// `count`/`cnt`) is reused from [`MoverState`].
#[derive(Default)]
#[allow(dead_code)]
pub struct RotState {
    /// Rotation rate in degrees/sec per axis (the `rotate` map key; per-segment for trains).
    pub rotate: Vec3,
    /// A target's rotated offset from the centre, recomputed each tick.
    pub neworigin: Vec3,
    /// How this entity follows its rotator (targets only).
    pub kind: RotateType,
    /// State-machine position (rotators only).
    pub phase: RotPhase,
    /// When the current rotation/move segment ends (`ltime`-based; trains and doors).
    pub endtime: f32,
    /// `1 / segment-traveltime`, for the train's origin interpolation.
    pub duration: f32,
}

/// Scratch for `func_bob` (`bob.rs`): a brush bobbing along `movedir`. It mirrors ktx's
/// velocity-accumulation (so the bob amplitude matches map tuning — `height` is a per-tick
/// velocity *intensity*, not the amplitude), but times the cycle off `cycle_t`, reset to 0 each
/// cycle so every cycle spans the same number of ticks. With identical alternating cycles the
/// integrated displacement (`offset`) returns exactly to zero each period, so anchoring the brush
/// at `pos1 + offset·movedir` ([`MoverState::pos1`]) yields no long-term drift — ktx's bug.
#[derive(Default)]
pub struct BobState {
    /// First-half speed-up factor applied to the ramp each tick (`waitmin`, ~1.0+).
    pub waitmin: f32,
    /// Second-half slow-down factor applied to the speed each tick (`waitmin2`, 0..1).
    pub waitmin2: f32,
    /// Seconds into the current cycle; reset to 0 at each pivot for clean, equal-length cycles.
    pub cycle_t: f32,
    /// Signed per-tick velocity ramp (ktx's `t_length`), restarted at `±height` each cycle.
    pub t_length: f32,
    /// Current scalar speed along `movedir`.
    pub vel: f32,
    /// Integrated displacement from the anchor along `movedir`.
    pub offset: f32,
}

/// Per-player grappling-hook state (`grapple.rs`). The hook itself is a separate entity (this
/// player's `hook`); it stores its target in `enemy` and the chain head in `goalentity`.
#[derive(Default)]
pub struct GrappleState {
    /// The player's live hook entity, or [`EntId::WORLD`] when none is out.
    pub hook: u32,
    /// The player is being reeled in (the hook has anchored).
    pub on_hook: bool,
    /// A hook has been thrown and not yet reset.
    pub hook_out: bool,
    /// One-shot latch: the chain is still up and should be ditched once the reel gets close
    /// (QuakeC's `lefty`, reused to avoid a dedicated field).
    pub lefty: bool,
}

#[derive(Default)]
#[allow(dead_code)]
pub struct CustomRefs {
    // entity references not present in entvars (custom QC `.entity` fields), as indices.
    // (Standard entvars refs — enemy/owner/goalentity/groundentity/aiment/chain — live in
    // `v` as byte offsets; use `EntId::to_prog`/`from_prog` and the `GameState` ref helpers.)
    pub oldenemy: u32,
    pub trigger_field: u32,
    pub movetarget: u32,
}

#[derive(Default)]
#[allow(dead_code)]
pub struct CombatState {
    /// Anti-double-fire latch for projectiles (`voided`).
    pub voided: f32,
    // combat / player timers (player.qc, weapons.qc, items.qc)
    pub attack_finished: f32,
    pub pain_finished: f32,
    pub super_damage_finished: f32,
    pub invincible_finished: f32,
    pub invincible_sound: f32,
    pub invincible_time: f32,
    pub invisible_finished: f32,
    pub invisible_time: f32,
    pub invisible_sound: f32,
    pub super_time: f32,
    pub super_sound: f32,
    pub rad_time: f32,
    pub radsuit_finished: f32,
    pub air_finished: f32,
    pub bubble_count: f32,
    pub dmgtime: f32,
    pub deathtime: f32,
    pub jump_flag: f32,
    pub swim_flag: f32,
    /// Whether the one mid-air jump (`rtx_doublejump`) has been spent this air travel. Set on the
    /// air jump, cleared whenever the player is on the ground.
    pub air_jumped: bool,
    /// Last upward speed of a lift the player was riding, and when (for `rtx_elevator_jump`'s
    /// grace window — so jumping just as the lift stops still boosts).
    pub lift_vz: f32,
    pub lift_time: f32,
    pub fly_sound: f32,
    pub axhitme: f32,
    pub show_hostile: f32,
}

/// Spawn-point fairness state, per player (KTX's `k_lastspawn`/`k_1spawn`).
#[derive(Default)]
pub struct SpawnState {
    /// The spot entity this player last spawned at (`WORLD` = none yet). Consulted for the
    /// one-time re-roll that avoids back-to-back respawns on the same spot.
    pub last_spot: EntId,
    /// World time until which this player fences nearby spawn spots during live play
    /// (+2.6 on spawn, +0.78 on teleport — KTX's `k_1spawn` grace window).
    pub grace_until: f32,
}

#[derive(Default)]
#[allow(dead_code)]
pub struct ItemState {
    // item scratch (items.qc)
    pub healamount: f32,
    pub healtype: f32,
    pub aflag: f32,
    pub items2: f32,
    pub aflag2: f32,
    pub last_pickup_msg: f32,
    /// On a CTF `item_rune` pickup, which rune this is (`crate::defs::RUNE_*`); `0` on everything
    /// else. Separate from a *player's* held rune (`ModePlayer::ctf.runes`).
    pub rune_bit: u8,
}

/// A CTF flag's phase (on the flag entity).
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum FlagPhase {
    /// At its base (`home`), capturable / returnable-to.
    #[default]
    Home,
    /// Held by `carrier` (hidden, non-solid).
    Carried,
    /// Lying in the field; auto-returns at `return_at`.
    Dropped,
    /// Voluntarily tossed: like `Dropped`, but `carrier` (the tosser) can't re-grab until
    /// `return_at`, after which it becomes a normal `Dropped` flag.
    Tossed,
}

/// CTF flag state, on the flag entity's private tail (`team == 0` on any non-flag entity). See
/// [`crate::mode::ctf`].
#[derive(Default)]
pub struct FlagState {
    /// Owning team (`1`/`2`); `0` means this entity isn't a flag.
    pub team: u8,
    /// Base origin to return to.
    pub home: Vec3,
    /// The player carrying it (`WORLD` when not carried).
    pub carrier: EntId,
    /// World time a dropped flag auto-returns.
    pub return_at: f32,
    pub phase: FlagPhase,
}

/// Race-route entity data (`race_route_start`/`race_route_marker` map keys), carried only
/// long enough for `load_race_routes` to fold the markers into [`crate::race::RaceState`]
/// at the end of entity spawn — the marker entities are freed right after. See [`crate::race`].
#[derive(Default)]
pub struct RaceEnt {
    /// `race_route_name` / `race_route_description` — route identity (both mandatory in ktx).
    pub name: Option<Box<str>>,
    pub desc: Option<Box<str>>,
    /// `race_route_timeout` — seconds allowed for a run.
    pub timeout: f32,
    /// `race_route_weapon_mode` / `race_route_falsestart_mode` — ktx enum ints, validated
    /// when the route is built.
    pub weapon_mode: f32,
    pub falsestart_mode: f32,
    /// `race_route_start_yaw` / `race_route_start_pitch` — the racer's spawn view angles.
    pub start_yaw: f32,
    pub start_pitch: f32,
    /// The `size` map key — a marker's touch-box extent (kept off `v.mins`/`v.maxs`; these
    /// markers never enter collision).
    pub size: Vec3,
    /// `race_flags` — ktx teleport touch flags; parked for a future full race ruleset.
    pub flags: f32,
}

// The engine writes 8-byte native pointers into `string_refs` via unaligned-looking but
// actually-aligned `*(quintptr_t*)(array_base + offset)` stores. Guarantee those land on an
// 8-byte boundary: the array base is 8-aligned (Box of an 8-aligned type), the stride is a
// multiple of 8, and the slots themselves are 8-aligned within the entity.
const _: () = assert!(align_of::<Entity>() >= 8);
const _: () = assert!(size_of::<Entity>().is_multiple_of(8));
const _: () = assert!(core::mem::offset_of!(Entity, string_refs).is_multiple_of(8));

impl Entity {
    /// Reset a slot to a pristine spawned state, mirroring QuakeC's freshly-spawned edict
    /// (all fields zeroed). Called after the engine hands us a slot via `spawn`.
    pub fn reset(&mut self) {
        *self = Entity::default();
        self.in_use = true;
    }

    /// Borrow the classname as `&str`, if set.
    pub fn classname(&self) -> Option<&str> {
        self.classname.as_deref()
    }

    /// Whether this is a player edict (classname `"player"`) — the check spelled out at ~two dozen
    /// call sites that iterate entities looking for clients.
    pub fn is_player(&self) -> bool {
        self.classname() == Some("player")
    }

    /// Whether this entity is a live, fightable body: positive health and not partway through (or
    /// finished) a death sequence. The `health > 0 && deadflag == No` compound the spawn/mode/bot
    /// pickers all repeat.
    pub fn is_alive(&self) -> bool {
        self.v.health > 0.0 && self.v.deadflag == crate::defs::DeadFlag::No
    }

    // --- entvars entity-reference accessors (stored as byte offsets) ---
    pub fn enemy(&self) -> EntId {
        EntId::from_prog(self.v.enemy)
    }
    pub fn set_enemy(&mut self, t: EntId) {
        self.v.enemy = t.to_prog();
    }
    pub fn owner(&self) -> EntId {
        EntId::from_prog(self.v.owner)
    }
    pub fn set_owner(&mut self, t: EntId) {
        self.v.owner = t.to_prog();
    }
    pub fn goalentity(&self) -> EntId {
        EntId::from_prog(self.v.goalentity)
    }
    pub fn set_goalentity(&mut self, t: EntId) {
        self.v.goalentity = t.to_prog();
    }

    // --- engine-gated callbacks ---
    // The engine fires `GAME_EDICT_TOUCH`/`GAME_EDICT_BLOCKED` only for entities whose
    // entvars func-ref field is nonzero: its `SV_TouchLinks` skips triggers with `v.touch == 0`,
    // and `SV_Push` skips a blocked callback when `v.blocked == 0`. Our behaviour lives in the
    // private-tail enum, so these setters keep the entvars gate in sync (any nonzero marker
    // suffices — the engine never calls the value, it just dispatches back through `vmMain`).
    // (`.think` needs no such gate: think is fired on `nextthink` elapsing, not on `v.think`.)

    /// Set the `.touch` behaviour, syncing the engine-visible `v.touch` dispatch gate.
    pub fn set_touch(&mut self, t: Touch) {
        self.touch = t;
        self.v.touch = if t == Touch::None { 0 } else { 1 };
    }

    /// Set the `.blocked` behaviour, syncing the engine-visible `v.blocked` dispatch gate.
    pub fn set_blocked(&mut self, b: Blocked) {
        self.blocked = b;
        self.v.blocked = if b == Blocked::None { 0 } else { 1 };
    }
}

/// The entity array, indexable by a typed [`EntId`] handle (`entities[e]`) and transparently
/// usable as the underlying `[Entity]` slice for iteration, `len`, and the raw base pointer
/// the engine strides. Zero-cost: a newtype over the heap `Box<[Entity]>`, so the data
/// pointer the engine receives never moves.
pub struct Entities(Box<[Entity]>);

impl Entities {
    /// Allocate `count` cleared entity slots.
    pub fn new(count: usize) -> Self {
        Entities((0..count).map(|_| Entity::default()).collect())
    }

    /// Iterate the live (`in_use`) entities as `(EntId, &Entity)`, skipping slot 0 (world) and every
    /// free slot — the shape the many `for i in 1..len { let id = EntId(i); if !in_use { continue } }`
    /// scans hand-roll.
    pub fn live(&self) -> impl Iterator<Item = (EntId, &Entity)> {
        self.0
            .iter()
            .enumerate()
            .skip(1)
            .filter(|(_, e)| e.in_use)
            .map(|(i, e)| (EntId(i as u32), e))
    }
}

impl core::ops::Deref for Entities {
    type Target = [Entity];
    #[inline]
    fn deref(&self) -> &[Entity] {
        &self.0
    }
}

impl core::ops::DerefMut for Entities {
    #[inline]
    fn deref_mut(&mut self) -> &mut [Entity] {
        &mut self.0
    }
}

impl core::ops::Index<EntId> for Entities {
    type Output = Entity;
    #[inline]
    fn index(&self, e: EntId) -> &Entity {
        &self.0[e.index()]
    }
}

impl core::ops::IndexMut<EntId> for Entities {
    #[inline]
    fn index_mut(&mut self, e: EntId) -> &mut Entity {
        &mut self.0[e.index()]
    }
}

// Raw `usize` indexing for the few index-range scans (`next_door`, the spectator spawn
// cycle); `EntId` is the typed handle everywhere else.
impl core::ops::Index<usize> for Entities {
    type Output = Entity;
    #[inline]
    fn index(&self, i: usize) -> &Entity {
        &self.0[i]
    }
}

impl core::ops::IndexMut<usize> for Entities {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut Entity {
        &mut self.0[i]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defs::Solid;

    /// A slot cleared with `Entity::default()` — what the map-load `GAME_CLEAR_EDICT` handler now
    /// does — must be inert to every full-array scan gate. Our entity box is process-global, and the
    /// engine's map-load sweep does *not* memset it, so a slot left non-inert would keep the previous
    /// map's data at an index `>= num_edicts` and crash the server ("NUM_FOR_EDICT: bad pointer") the
    /// moment a scan passed it to a builtin.
    #[test]
    fn cleared_slot_is_inert_to_scans() {
        let e = Entity::default();
        // `find_by_classname`'s gate.
        assert!(!e.in_use, "a cleared slot must read as not-in-use");
        // The `bot_pickup_items` backpack sweep's gate: `touch == Backpack && v.solid == Trigger`.
        assert_eq!(e.touch, Touch::None, "a cleared slot has no touch behaviour");
        assert_eq!(e.v.solid, Solid::Not, "a cleared slot is non-solid");
        assert_ne!(e.v.solid, Solid::Trigger);
    }

    /// The map-load clear must neutralize a slot that carried a live backpack over from the previous
    /// map — the concrete stale state the crash hinged on.
    #[test]
    fn stale_backpack_slot_is_neutralized_by_clear() {
        // A slot as it looked on the previous map: a live, touchable backpack trigger.
        let mut slot = Entity {
            in_use: true,
            ..Default::default()
        };
        slot.set_touch(Touch::Backpack);
        slot.v.solid = Solid::Trigger;
        slot.v.origin = glam::Vec3::new(128.0, -64.0, 24.0);
        assert!(
            slot.touch == Touch::Backpack && slot.v.solid == Solid::Trigger,
            "sanity: matches the sweep"
        );

        // The map-load `GAME_CLEAR_EDICT` clear.
        slot = Entity::default();

        assert!(!slot.in_use);
        assert_ne!(slot.touch, Touch::Backpack);
        assert_ne!(slot.v.solid, Solid::Trigger);
    }

    /// `reset()` is for freshly *spawned* slots and marks them in-use — distinct from the `default()`
    /// clear, which must leave `in_use == false`. The fix relies on this difference.
    #[test]
    fn reset_marks_in_use_unlike_default() {
        assert!(!Entity::default().in_use);
        let mut e = Entity::default();
        e.reset();
        assert!(e.in_use, "reset() spawns the slot in-use");
    }
}
