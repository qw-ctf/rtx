//! The entity type and its handle.
//!
//! [`Entity`] is `#[repr(C)]` so that its [`EntVars`] prefix sits at offset 0 — the engine
//! strides the array by `size_of::<Entity>()` (reported as `sizeofent`) and reads/writes
//! that prefix directly. Everything after `v` is the *private tail*: ordinary Rust state
//! the engine never touches, so it may use any types (enums, `Option`, etc.).

use crate::abi::EntVars;

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
}

/// A scheduled `think` behaviour. Modeled as a data enum (not a fn pointer) so it is
/// `Debug`/`Eq`/serializable and the central dispatcher's `match` is exhaustiveness-checked.
/// Animation chains will be added as table-driven variants (e.g. `PlayerAnim(AnimId)`)
/// rather than one variant per QC frame function.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Think {
    #[default]
    None,
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

    // --- private tail: engine never addresses these ---
    /// Whether this slot is currently a live entity.
    pub in_use: bool,
    /// Scheduled think behaviour, resolved by the central dispatcher.
    pub think: Think,
    /// Classname, for spawn dispatch and `find`.
    pub classname: Option<Box<str>>,
    pub model: Option<Box<str>>,
    pub weaponmodel: Option<Box<str>>,
    pub target: Option<Box<str>>,
    pub targetname: Option<Box<str>>,
    pub killtarget: Option<Box<str>>,
    pub message: Option<Box<str>>,
    pub netname: Option<Box<str>>,
}

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
}
