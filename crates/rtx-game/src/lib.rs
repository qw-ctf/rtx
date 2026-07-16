// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx` — a native Rust reimplementation of QuakeWorld `qwprogs.dat`, loadable by mvdsv
//! (the `pr2` native game-module API, `GAME_API_VERSION 16`).
//!
//! ## Single global, isolated unsafe
//! All state lives in one [`GameState`], owned by the sole global below
//! (`OnceLock<Game>`). The engine is single-threaded, so a `Game(UnsafeCell<GameState>)`
//! gives us a `&mut GameState` at the top of `vmMain` with the only `unsafe` deref in the
//! crate's control flow. The other unsafe lives in `host.rs` (the variadic syscalls). The
//! game logic above is ordinary safe Rust over index handles.

#![allow(non_snake_case)] // dllEntry / vmMain are the engine's required export names.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

mod abi;
mod anim;
mod arsenal;
mod assets;
mod bob;
mod bot;
mod buttons;
mod client;
mod combat;
mod control;
mod cvars;
mod defs;
#[cfg(test)]
mod demo_replay;
mod dispatch;
mod doors;
mod entity;
mod ext_field;
mod game;
mod game_command;
mod grapple;
mod host;
mod items;
mod math;
mod misc;
mod mode;
mod nav_build;
#[cfg(feature = "netclient")]
pub mod netclient;
mod obituary;
mod plats;
mod player;
mod pmove_sim;
mod race;
mod raceline;
mod rotate;
mod server;
mod spawn;
mod spectate;
mod subs;
mod text;
mod triggers;
mod weapons;
mod world;

// The BSP parser, navmesh, and hazard classifier live in the shared `rtx-nav` crate; re-export
// them under their old module paths so game code keeps referring to `crate::bsp` / `crate::navmesh`
// / `crate::hazard` unchanged.
pub(crate) use rtx_nav::{bsp, hazard, navmesh};

use game::GameState;
use game_command::GameCommand;
use host::SyscallFn;

/// Wrapper giving interior mutability + a `Sync` impl for the `static`.
struct Game(UnsafeCell<GameState>);

// SAFETY: the host engine drives this module from a single thread; `vmMain`/`dllEntry`
// are never called concurrently, so the raw pointers inside never cross threads.
unsafe impl Sync for Game {}
unsafe impl Send for Game {}

/// The one and only global.
static GAME: OnceLock<Game> = OnceLock::new();

/// Set by [`game::GameState::spawn`] for the duration of the `spawn` trap. That trap makes the
/// engine's `ED_Alloc` run `ED_ClearEdict`, which re-enters this module with `GAME_CLEAR_EDICT`
/// *synchronously* — while `spawn`'s `&mut GameState` is live. `spawn` re-establishes the edict's
/// string refs itself, so that callback is redundant; `vmMain` skips it while this is set, before
/// taking a borrow, so the re-entrant call can't alias `spawn`'s borrow. Engine-initiated
/// `GAME_CLEAR_EDICT` (map load, client edicts) leaves this clear and runs normally. Single-threaded.
pub(crate) static SUPPRESS_CLEAR_EDICT: AtomicBool = AtomicBool::new(false);

/// Debug-only re-entrancy guard: set while a `dispatch` borrow is live. Every host trap that
/// re-enters `vmMain` is either deferred out of the borrow ([`bot::drain_roster`]) or suppressed
/// ([`SUPPRESS_CLEAR_EDICT`]), so this must never already be set when `vmMain` takes a borrow —
/// if it is, a *new* re-entrant trap slipped in and would create an aliasing `&mut` (UB). The
/// assert makes that a loud, deterministic failure in testing instead of silent corruption.
#[cfg(debug_assertions)]
static IN_DISPATCH: AtomicBool = AtomicBool::new(false);

/// `dllEntry` — first export the engine calls, handing us the syscall dispatcher.
#[no_mangle]
pub extern "C" fn dllEntry(syscall: SyscallFn) {
    let _ = GAME.set(Game(UnsafeCell::new(GameState::new(syscall))));
}

/// `vmMain` — the sole control-flow entry from the engine. The engine passes up to 12
/// `int` args; we read only the ones current commands need.
#[no_mangle]
pub extern "C" fn vmMain(cmd: i32, arg0: i32, arg1: i32, arg2: i32) -> isize {
    // Filter unknown command ids (e.g. GAME_EDICT_CSQCSEND = 200) at the boundary.
    let Some(cmd) = GameCommand::from_i32(cmd) else {
        return 0;
    };
    let cell = GAME.get().expect("vmMain called before dllEntry");
    // A `GAME_CLEAR_EDICT` re-entered from our own `spawn()` is handled by `spawn()` itself; skip it
    // here *before* taking a borrow so it can't alias the `&mut GameState` the outer `spawn()` holds.
    // See `SUPPRESS_CLEAR_EDICT`.
    if matches!(cmd, GameCommand::ClearEdict) && SUPPRESS_CLEAR_EDICT.load(Ordering::Relaxed) {
        return 0;
    }
    // Guard (debug builds): no dispatch borrow may already be live when we take one — see
    // `IN_DISPATCH`.
    #[cfg(debug_assertions)]
    assert!(
        !IN_DISPATCH.swap(true, Ordering::Relaxed),
        "vmMain re-entered mid-dispatch: a host trap re-entered while a &mut GameState was live — \
         defer it (bot::drain_roster) or suppress it (SUPPRESS_CLEAR_EDICT)"
    );
    // SAFETY: single-threaded engine; the borrow is confined to this block and dropped before the
    // roster drain. With the deferral and the `ClearEdict` suppression above, no host trap re-enters
    // `vmMain` while this borrow is live, so it is never aliased.
    let ret = {
        let game = unsafe { &mut *cell.0.get() };
        game.dispatch(cmd, arg0, arg1, arg2)
    };
    #[cfg(debug_assertions)]
    IN_DISPATCH.store(false, Ordering::Relaxed);
    // Apply any bot add/remove queued during the frame *now*, with no `&mut GameState` borrow live.
    // `add_bot`/`remove_bot` run the module's client callbacks synchronously and re-entrantly; doing
    // this here (rather than inline in `manage_population`) keeps the re-entered `vmMain`'s borrow
    // the sole one, instead of aliasing a live outer borrow. See [`crate::bot::drain_roster`].
    // SAFETY: single-threaded; the block above dropped its borrow, so no reference into the
    // GameState is live across the re-entrant trap this fires.
    unsafe { crate::bot::drain_roster(cell.0.get()) };
    ret
}
