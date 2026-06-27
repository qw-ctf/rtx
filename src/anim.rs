// SPDX-License-Identifier: AGPL-3.0-or-later

//! A small animation framework shared by every animated model.
//!
//! It holds only the machinery: the [`Anim`] type (a named run of model frames whose length is
//! intrinsic) and the [`frames`] macro that declares a model's `$frame` numbering and the
//! animations built on it in one place. Each model's actual frame/animation table lives with
//! that model's logic (e.g. `player.rs` for `player.mdl`); only the reusable pieces live here.

/// A named animation: a consecutive run of model frames, built from its first and last frame so
/// its length is derived (never threaded through an animation API). The runtime layer over raw
/// frame numbering — analogous to QuakeC's `*_*` frame-chains, where the chain, not the `$frame`
/// line, decides which frames play and so how long it runs.
#[derive(Clone, Copy, Default)]
pub(crate) struct Anim {
    pub(crate) first: i32,
    pub(crate) len: i32,
}

/// An animation spanning `first..=last` inclusive; length follows from the endpoints.
pub(crate) const fn seq(first: i32, last: i32) -> Anim {
    Anim {
        first,
        len: last - first + 1,
    }
}

impl Anim {
    /// Model frame for the 0-based `cursor` within this animation.
    pub(crate) const fn frame(self, cursor: i32) -> f32 {
        (self.first + cursor) as f32
    }
    /// The animation's final frame.
    pub(crate) const fn last(self) -> i32 {
        self.first + self.len - 1
    }
}

/// Declare a model's frames and the animations built on them — QuakeC's two constructs in one
/// place. Invoke it once per animated model, alongside that model's logic.
///
/// * `number { … }` is the flat `$frame` numbering: one global counter assigns each name the
///   next model frame (`#[repr(i32)]` enum discriminants supply the count — no hand-tallied
///   offsets). Like `$frame`, it lists every frame and knows nothing of animations.
/// * each `anim NAME = FIRST ..= LAST;` is the analogue of a `*_*` frame-chain: it names a
///   playable run and derives its length from the endpoints (so an unplayed tail of a `$frame`
///   group — e.g. the axe's `$axatt5`/`$axatt6` — is simply numbered but named by no `anim`).
///
/// Generates `const FRAME: i32` per frame and `const NAME: Anim` per animation in the calling
/// module. (One invocation per module: the internal `FrameIndex` counter enum is a module item.)
macro_rules! frames {
    (
        number { $($frame:ident)* }
        $( anim $anim:ident = $first:ident ..= $last:ident; )+
    ) => {
        #[repr(i32)]
        #[allow(non_camel_case_types, dead_code)]
        enum FrameIndex { $($frame),* }
        $( #[allow(dead_code)] const $frame: i32 = FrameIndex::$frame as i32; )*
        $(
            #[allow(dead_code)]
            const $anim: $crate::anim::Anim = $crate::anim::seq($first, $last);
        )+
    };
}
pub(crate) use frames;
