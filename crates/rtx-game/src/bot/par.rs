// SPDX-License-Identifier: AGPL-3.0-or-later

//! The bot brain's persistent worker pool, used to fan a single goal pick's independent Dijkstra
//! floods (`best_item_plan`) out across cores. A goal pick runs up to nine whole-navmesh floods, and
//! on a big map that is the frame's dominant cost; the floods are pure functions of
//! `(&NavGraph, source, &LinkCosts)`, so running them concurrently is bit-identical to serial.
//!
//! **Never rayon's global pool.** This crate ships inside a native game module (`rtx.dll`) the engine
//! can unload, and the global pool's worker threads would outlive the DLL and later run freed code —
//! the same rule the navmesh build follows with its transient pool. But unlike that build (whose pool
//! lives and dies inside one background call, long before any unload), this pool is held across frames
//! and torn down at `GAME_SHUTDOWN`, right before the engine `dlclose`s us. `ThreadPool::drop` only
//! *signals* its workers to terminate (rayon-core's `Registry::terminate`); it does not wait for them.
//! So we spawn the workers through a custom `spawn_handler`, keep their `JoinHandle`s, and on shutdown
//! drop the pool (to signal) and then **join** every handle — that join is what guarantees no rayon or
//! std code is still executing when our code is unmapped.

use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use rayon::prelude::*;

use crate::host::HostApi;
use crate::navmesh::{CellId, LinkCosts, NavGraph};

/// A persistent rayon pool owned by `GameState`, built lazily and torn down (workers joined) at
/// shutdown. `None` pool ⇒ every helper runs serially, so a build failure or `rtx_bot_par 0` simply
/// falls back to the original single-threaded behavior.
#[derive(Default)]
pub(crate) struct BotPool {
    pool: Option<rayon::ThreadPool>,
    /// The worker `JoinHandle`s, captured at build via `spawn_handler` so [`shutdown`](Self::shutdown)
    /// can join them after dropping the pool. Empty whenever `pool` is `None`.
    handles: Vec<JoinHandle<()>>,
    /// A build that failed once — don't retry it every frame.
    failed: bool,
}

impl BotPool {
    /// Reconcile the pool to whether it's actually wanted — `rtx_bot_par` on **and** `rtx_bot_lod`
    /// off: the pool exists only to fan out the exact-mode goal floods, and under LOD those run coarse
    /// and serial, so holding idle workers (and their shutdown-join cost) buys nothing. Builds on first
    /// enable, tears down otherwise; rebuilds if lod is toggled back off. Called once per frame from
    /// `run_bots`; cheap when already in the wanted state.
    pub(crate) fn ensure(&mut self, host: &HostApi) {
        let want = host.cvar_bool(c"rtx_bot_par") && !host.cvar_bool(c"rtx_bot_lod");
        if want && self.pool.is_none() && !self.failed {
            // Leave one core for the engine/main thread, and cap at the fan-out width — a goal pick
            // never has more than nine independent floods, so more workers would just idle.
            let threads = std::thread::available_parallelism()
                .map(|n| n.get().saturating_sub(1))
                .unwrap_or(1)
                .clamp(1, 8);
            match build_pool(threads) {
                Some((pool, handles)) => {
                    self.pool = Some(pool);
                    self.handles = handles;
                }
                None => self.failed = true, // couldn't build — stay serial, don't thrash retrying
            }
        } else if !want && self.pool.is_some() {
            self.shutdown();
        }
    }

    /// Drop the pool (signalling its workers to terminate) and then join every worker, so no pool
    /// thread is still running when the DLL is unloaded. Idempotent. Called from `GAME_SHUTDOWN`.
    pub(crate) fn shutdown(&mut self) {
        // Order matters: dropping the pool is what *signals* the parked workers to exit; joining
        // before that would block forever. After the drop, each handle's join waits for real exit.
        self.pool = None;
        for h in self.handles.drain(..) {
            let _ = h.join();
        }
    }

    /// Run two independent flood closures, on the pool when built (else serially). Preserves the
    /// return order, so callers stay deterministic.
    #[inline]
    pub(crate) fn join<A, B, RA, RB>(&self, a: A, b: B) -> (RA, RB)
    where
        A: FnOnce() -> RA + Send,
        B: FnOnce() -> RB + Send,
        RA: Send,
        RB: Send,
    {
        match &self.pool {
            Some(p) => p.join(a, b),
            None => (a(), b()),
        }
    }

    /// Flood from each `source` under the shared `costs`, returning one cost vector per source **in
    /// input order**. Runs on the pool when built (else serially). The floods are pure and
    /// order-preserving, so the result is identical whether or not the pool is present.
    pub(crate) fn flood_batch(&self, graph: &NavGraph, sources: &[CellId], costs: &LinkCosts) -> Vec<Vec<f32>> {
        match &self.pool {
            // `install` runs the parallel iterator on *this* pool, never the global one.
            Some(p) => p.install(|| sources.par_iter().map(|&s| graph.costs_from(s, costs)).collect()),
            None => sources.iter().map(|&s| graph.costs_from(s, costs)).collect(),
        }
    }
}

/// Build a rayon pool of `threads` workers spawned through std threads whose `JoinHandle`s we keep,
/// so shutdown can join them (see the module docs on why `ThreadPool::drop` alone is not enough).
fn build_pool(threads: usize) -> Option<(rayon::ThreadPool, Vec<JoinHandle<()>>)> {
    let collector: Arc<Mutex<Vec<JoinHandle<()>>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&collector);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .thread_name(|i| format!("rtx-bot-{i}"))
        .spawn_handler(move |thread| {
            let mut builder = std::thread::Builder::new();
            if let Some(name) = thread.name() {
                builder = builder.name(name.to_string());
            }
            if let Some(size) = thread.stack_size() {
                builder = builder.stack_size(size);
            }
            let handle = builder.spawn(move || thread.run())?;
            sink.lock().unwrap().push(handle);
            Ok(())
        })
        .build()
        .ok()?;
    // All workers have been spawned by the time `build` returns; lift their handles out of the
    // collector (the spawn handler is done firing, so the shared Vec is complete).
    let handles = std::mem::take(&mut *collector.lock().unwrap());
    Some((pool, handles))
}
