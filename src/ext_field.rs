// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed handles for the engine's *map-extension fields* — entity state the engine keeps in a
//! parallel `ext_entvars_t` (transparency, colour modulation, …) rather than in our entvars,
//! reached through the `MapExtFieldPtr`/`SetExtFieldPtr` traps.
//!
//! Same spirit as the [asset registry](crate::assets): one line both names a field and binds its
//! value type, so `ext_fields.set::<Alpha>(host, ent, 0.5)` won't compile with the wrong type.
//! The difference is strictness — a missing asset is unrepresentable, but a missing *field* is
//! fine: older engines simply lack it, and the set becomes a silent no-op. References resolve
//! once per server (lazily) and are cached, so adding a field is a single [`ext_fields!`] line
//! with no per-field plumbing in `GameState`.

use std::collections::HashMap;
use std::ffi::CStr;

use crate::entity::EntId;
use crate::host::HostApi;

/// A map-extension field: its engine wire name and the value type it stores. Implemented by the
/// zero-sized marker types from [`ext_fields!`]; used only as a type parameter to [`ExtFields::set`].
pub trait ExtField {
    /// The field's value as the engine stores it (memcpy'd raw — `f32` for `alpha`, `[f32; 3]`
    /// for a colour, …).
    type Value: Copy;
    /// The name passed to `MapExtFieldPtr`.
    const NAME: &'static CStr;
}

/// Declare map-extension field marker types, one line each: `Type: ValueType = c"wire_name";`.
macro_rules! ext_fields {
    ($($(#[$m:meta])* $ty:ident: $val:ty = $name:expr;)*) => {
        $(
            $(#[$m])*
            pub struct $ty;
            impl ExtField for $ty {
                type Value = $val;
                const NAME: &'static CStr = $name;
            }
        )*
    };
}

ext_fields! {
    /// Entity transparency, `0.0` (invisible) … `1.0` (opaque).
    Alpha: f32 = c"alpha";
    /// Per-channel RGB colour modulation, each component a multiplier around `1.0`.
    ColorMod: [f32; 3] = c"colormod";
}

/// Per-server cache of resolved extension-field references. Owned by `GameState` (like
/// [`DynAssets`](crate::assets::DynAssets)): the trap support and each field reference are
/// looked up once, on first use, and reused thereafter.
#[derive(Default)]
pub struct ExtFields {
    /// Whether the extension-field traps are available at all (`None` until first probed). When
    /// `false`, every field is treated as absent.
    supported: Option<bool>,
    /// Resolved references by wire name; `0` means the server doesn't provide that field.
    refs: HashMap<&'static CStr, u32>,
}

impl ExtFields {
    /// Set extension field `F` on `ent`. Silently does nothing if the server lacks the
    /// extension traps or that particular field — callers treat extension fields as best-effort.
    pub fn set<F: ExtField>(&mut self, host: &HostApi, ent: EntId, value: F::Value) {
        let supported = *self.supported.get_or_insert_with(|| host.register_ext_fields());
        if !supported {
            return;
        }
        let field_ref = *self
            .refs
            .entry(F::NAME)
            .or_insert_with(|| host.map_ext_field_ptr(F::NAME));
        if field_ref != 0 {
            host.set_ext_field(ent, field_ref, &value);
        }
    }
}
