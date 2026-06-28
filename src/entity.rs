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

/// A `Copy` index handle into the entity array. Never a borrow, so holding one across a
/// trap call is fine — this is what keeps the safe API free of aliasing hazards.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
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
    /// Terminal of a death sequence (`PlayerDead`: freeze, mark `DeadFlag::Dead.as_f32()`).
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

/// A `.blocked` behaviour for `MoveType::Push.as_f32()` movers (`GAME_EDICT_BLOCKED`).
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

/// `state` for door/plat/button movers (QuakeC `STATE_*`).
pub const STATE_TOP: f32 = 0.0;
pub const STATE_BOTTOM: f32 = 1.0;
pub const STATE_UP: f32 = 2.0;
pub const STATE_DOWN: f32 = 3.0;

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

    // --- private tail: engine never addresses these ---
    /// Whether this slot is currently a live entity.
    pub in_use: bool,

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
    pub noise: Option<crate::assets::Sound>,
    pub noise1: Option<crate::assets::Sound>,
    pub noise2: Option<crate::assets::Sound>,
    pub noise3: Option<crate::assets::Sound>,
    pub noise4: Option<crate::assets::Sound>,
    pub deathtype: Option<Box<str>>,
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
    pub model_cstr: Option<crate::assets::Model>,
    /// Owned C string backing a brush model (`*N`) passed to `setmodel`. The engine keeps the
    /// raw pointer, so this must live as long as the entity references the model.
    pub model_cs: Option<CString>,

    pub anim: AnimState,
    pub mover: MoverState,
    pub rot: RotState,
    pub bob: BobState,
    pub refs: CustomRefs,
    pub combat: CombatState,
    pub item: ItemState,
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
    pub state: f32,
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
