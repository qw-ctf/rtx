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
use std::sync::OnceLock;

mod abi;
mod anim;
mod assets;
mod bob;
mod bot;
mod bot_bhop;
mod bot_combat;
mod bot_goals;
mod bot_grenade;
mod bsp;
mod buttons;
mod client;
mod combat;
mod defs;
mod dispatch;
mod doors;
mod entity;
mod ext_field;
mod game;
mod game_command;
mod grapple;
mod host;
mod items;
mod misc;
mod mode;
mod navmesh;
mod plats;
mod player;
mod rotate;
mod server;
mod spectate;
mod subs;
mod triggers;
mod weapons;
mod world;

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
    // SAFETY: single-threaded engine; no other live borrow of the GameState exists.
    let game = unsafe { &mut *cell.0.get() };
    game.dispatch(cmd, arg0, arg1, arg2)
}
