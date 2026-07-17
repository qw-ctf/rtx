// SPDX-License-Identifier: AGPL-3.0-or-later

//! Wall-clock profile of the bot brain: what one bot frame's *thinking* costs the whole squad, and
//! how that compares with the frame the engine allots it. Opt-in via `rtx_bot_prof <seconds>`.
//!
//! ## What a "frame" is here
//!
//! mvdsv calls the module **once per bot frame, not once per bot** — `SV_RunBots` (`sv_phys.c`) does
//! `PR_GLOBAL(frametime) = sv_frametime; SV_ProgStartFrame(true);` *before* its client loop, and the
//! loop then only pushes each bot's already-emitted usercmd through `SV_RunCmd`. So one
//! [`run_bots`](super::run_bots) is one bot frame for the entire squad, and bracketing it is already
//! the all-bots total — nothing needs summing to get there.
//!
//! That also fixes the scope: the engine's per-bot *physics* (`SV_PreRunCmd`/`SV_RunCmd`/
//! `SV_PostRunCmd`, trigger touches) runs outside our call, so these numbers are the bots'
//! **decision** time, not their total cost to the server.
//!
//! ## What the budget is
//!
//! `SV_RunBots` reads the `maxfps` cvar (`cvar_t sv_maxfps = {"maxfps", "77", CVAR_SERVERINFO}`),
//! falls back to 77 outside `[20, 1000]`, and only runs a bot frame once `1/maxfps` has passed — so
//! `1/maxfps` (≈12.99ms at the default 77) is the slice the brain has to fit in. `SV_Frame` runs
//! `SV_Physics()` *then* `SV_RunBots()`, so anything we spend here is added to the server's own frame
//! work: blowing the budget doesn't just make bots late, it makes the server late.

use std::time::Instant;

use crate::game::cstring;
use crate::host::HostApi;

/// The phases we bracket inside [`run_bot`](super::run_bot) — the three that hold the known-expensive
/// work. Everything else (`sense`, `prearm_traversal`, link pricing, `emit`) is unbracketed
/// remainder, so the phases deliberately sum to less than the frame total.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Phase {
    /// `resolve_objective` — the whole-navmesh Dijkstra floods behind goal selection.
    Objective,
    /// `steer::steer` — `find_path` / banded A*.
    Steer,
    /// `engage` plus the projectile/grenade overlays — LOS traces and bounce rollouts.
    Combat,
}

/// A stopwatch that only reads the clock when profiling is on, so the default path pays nothing.
#[derive(Clone, Copy)]
pub(crate) struct Timer(Option<Instant>);

impl Timer {
    /// Start the clock, or don't.
    pub(crate) fn start(on: bool) -> Self {
        Self(on.then(Instant::now))
    }

    /// Milliseconds since [`start`](Self::start), or 0 when profiling is off.
    pub(crate) fn stop(self) -> f32 {
        self.0.map_or(0.0, |t| t.elapsed().as_secs_f32() * 1000.0)
    }
}

/// A phase's running total across the window: enough to report an average and the worst single bot.
#[derive(Clone, Copy, Default)]
struct PhaseStat {
    sum: f32,
    max: f32,
    n: u32,
}

impl PhaseStat {
    fn add(&mut self, ms: f32) {
        self.sum += ms;
        self.max = self.max.max(ms);
        self.n += 1;
    }

    fn avg(&self) -> f32 {
        if self.n == 0 {
            0.0
        } else {
            self.sum / self.n as f32
        }
    }
}

/// The profiler's accumulator, owned by `GameState` so both embodiments (server module and
/// netclient) get it. Samples land here every bot frame while `rtx_bot_prof` is non-zero; each report
/// empties the window and starts the next.
#[derive(Default)]
pub(crate) struct BotProf {
    /// Whether this frame is being profiled. Set once per frame by [`run_bots`](super::run_bots) from
    /// the cvar, so the per-bot brackets inside `run_bot` don't each re-read it — and can't disagree
    /// with the frame bracket if the cvar is toggled mid-frame.
    on: bool,
    /// Per-bot-frame squad totals (ms) — the headline series.
    frames: Vec<f32>,
    /// Per-bot samples (ms) within the window.
    bots: Vec<f32>,
    objective: PhaseStat,
    steer: PhaseStat,
    combat: PhaseStat,
    /// Frames whose total exceeded the budget.
    over: u32,
    /// Most bots seen in any one frame this window.
    peak_bots: usize,
    /// This frame's per-bot times and phase totals, rebuilt every frame. Only interesting for the one
    /// frame that turns out to be the worst — but which frame that is isn't known until it's over.
    cur_bots: Vec<f32>,
    cur_phase: [f32; 3],
    /// An autopsy of the window's worst frame: what it cost, how it split across the bots that ran in
    /// it, and across the phases. `avg`/`p95`/`max` say a spike happened; only this says *what* it was
    /// — one bot doing something dear, or every bot's cadence landing on the same frame.
    worst: f32,
    worst_bots: Vec<f32>,
    worst_phase: [f32; 3],
    /// Window anchor. `None` until the first sample, so the window measures collected time rather
    /// than counting an idle map load against us.
    started: Option<Instant>,
    /// The budget the window was collected under. A `maxfps` change mid-window would make the
    /// percentages meaningless, so it resets instead of blending two regimes.
    budget: f32,
}

impl BotProf {
    /// Arm or disarm this frame's brackets.
    pub(crate) fn set_profiling(&mut self, on: bool) {
        self.on = on;
    }

    /// Whether to start a stopwatch at all.
    pub(crate) fn profiling(&self) -> bool {
        self.on
    }

    /// Drop everything and stand ready for a fresh window. Called on a report, on a `maxfps` change,
    /// and at `GAME_INIT` so a map load never lands inside a window. Leaves `on` alone: that's this
    /// frame's arming, re-established from the cvar every frame.
    pub(crate) fn reset(&mut self) {
        // `clear` keeps the capacity: after the first window there is no steady-state allocation.
        self.frames.clear();
        self.bots.clear();
        self.objective = PhaseStat::default();
        self.steer = PhaseStat::default();
        self.combat = PhaseStat::default();
        self.over = 0;
        self.peak_bots = 0;
        self.cur_bots.clear();
        self.cur_phase = [0.0; 3];
        self.worst = 0.0;
        self.worst_bots.clear();
        self.worst_phase = [0.0; 3];
        self.started = None;
        self.budget = 0.0;
    }

    /// Open a frame: whatever the last one accumulated is banked or discarded by now.
    pub(crate) fn begin_frame(&mut self) {
        self.cur_bots.clear();
        self.cur_phase = [0.0; 3];
    }

    /// Record one phase of one bot. Called the instant a phase is measured rather than batched at the
    /// end of `run_bot`, because that function has six early returns — two of them *after*
    /// `resolve_objective` has already paid for its floods, which a write-back would silently drop.
    pub(crate) fn add_phase(&mut self, phase: Phase, ms: f32) {
        if !self.on {
            return;
        }
        match phase {
            Phase::Objective => self.objective.add(ms),
            Phase::Steer => self.steer.add(ms),
            Phase::Combat => self.combat.add(ms),
        }
        self.cur_phase[phase as usize] += ms;
    }

    /// Record one bot's total think time.
    pub(crate) fn add_bot(&mut self, ms: f32) {
        self.bots.push(ms);
        self.cur_bots.push(ms);
    }

    /// Record the squad total for one bot frame, and open the window if this is its first sample.
    pub(crate) fn add_frame(&mut self, ms: f32, bots: usize, budget: f32) {
        if self.started.is_none() {
            self.started = Some(Instant::now());
            self.budget = budget;
        }
        self.frames.push(ms);
        self.peak_bots = self.peak_bots.max(bots);
        if ms > budget {
            self.over += 1;
        }
        if ms > self.worst {
            self.worst = ms;
            self.worst_bots.clear();
            self.worst_bots.extend_from_slice(&self.cur_bots);
            self.worst_phase = self.cur_phase;
        }
    }

    /// Whether the budget moved out from under the window (a live `maxfps` change).
    pub(crate) fn budget_changed(&self, budget: f32) -> bool {
        self.started.is_some() && (self.budget - budget).abs() > 1e-4
    }

    /// Report and start a new window once `interval` seconds of collection have passed.
    pub(crate) fn maybe_report(&mut self, host: &HostApi, interval: f32) {
        let Some(started) = self.started else { return };
        let elapsed = started.elapsed().as_secs_f32();
        if elapsed < interval || self.frames.is_empty() {
            return;
        }
        for line in self.render(elapsed) {
            host.conprint(&cstring(&line));
        }
        self.reset();
    }

    /// The report itself, as lines. Split out from the printing so it can be tested without a host.
    fn render(&mut self, elapsed: f32) -> Vec<String> {
        // Nearest-rank percentiles need the window ordered; we own it and are about to clear it.
        self.frames.sort_unstable_by(f32::total_cmp);
        self.bots.sort_unstable_by(f32::total_cmp);

        let n = self.frames.len();
        let avg = self.frames.iter().sum::<f32>() / n as f32;
        let p95 = percentile(&self.frames, 0.95);
        let max = *self.frames.last().unwrap_or(&0.0);

        let budget = self.budget;
        let pct = if budget > 0.0 { max / budget * 100.0 } else { 0.0 };
        // The head-room the request asked for, signed: how far the *worst* frame sat from the line.
        let margin = (max - budget).abs();
        let verdict = if max > budget {
            format!("{margin:.2}ms OVER")
        } else {
            format!("{margin:.2}ms under")
        };

        let bot_avg = if self.bots.is_empty() {
            0.0
        } else {
            self.bots.iter().sum::<f32>() / self.bots.len() as f32
        };
        let bot_max = *self.bots.last().unwrap_or(&0.0);
        // Report the rate the budget came from, not a second cvar read: this is the one in force.
        let fps = if budget > 0.0 { 1000.0 / budget } else { 0.0 };

        vec![
            format!(
                "rtx bots: {elapsed:.1}s {n} frames {} bots | avg {avg:.2} p95 {p95:.2} max {max:.2} ms\n",
                self.peak_bots
            ),
            format!(
                "rtx bots: budget {budget:.2}ms (maxfps {fps:.0}) | worst {pct:.0}% ({verdict}) | {}/{n} over | \
                 per-bot avg {bot_avg:.2} max {bot_max:.2} ms\n",
                self.over
            ),
            format!(
                "rtx bots: phases avg/max ms | objective {:.2}/{:.2} | steer {:.2}/{:.2} | combat {:.2}/{:.2}\n",
                self.objective.avg(),
                self.objective.max,
                self.steer.avg(),
                self.steer.max,
                self.combat.avg(),
                self.combat.max,
            ),
            // The autopsy: read across it to tell one dear bot from every bot at once.
            format!(
                "rtx bots: worst frame {:.2}ms | objective {:.2} steer {:.2} combat {:.2} | per-bot [{}]\n",
                self.worst,
                self.worst_phase[Phase::Objective as usize],
                self.worst_phase[Phase::Steer as usize],
                self.worst_phase[Phase::Combat as usize],
                self.worst_bots.iter().map(|b| format!("{b:.2}")).collect::<Vec<_>>().join(" "),
            ),
        ]
    }
}

/// Nearest-rank percentile over an already-sorted slice. Exact, and the window is bounded (~770
/// samples at the default 77Hz over 10s), so there's nothing to approximate away.
fn percentile(sorted: &[f32], q: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let rank = (q * (sorted.len() - 1) as f32).round() as usize;
    sorted[rank.min(sorted.len() - 1)]
}

/// The slice the engine allots the bot brain, in milliseconds.
///
/// Mirrors mvdsv's `SV_RunBots` exactly — it reads `maxfps` and substitutes 77 outside `[20, 1000]`
/// — so the head-room we report is the engine's own arithmetic rather than our guess at it.
///
/// Deliberately not `globals.frametime`, even though the server stores precisely this value there
/// right before calling us: under the netclient that field is the *measured* frame delta, which grows
/// exactly when we overrun and would quietly excuse the overrun. `maxfps` is honest on both — on the
/// server it's the very cvar `SV_RunBots` reads, and on the netclient it's a `SERVER_RULE_CVARS` key
/// resolved from serverinfo (mvdsv publishes it: `CVAR_SERVERINFO`), which is what the client paces
/// itself to anyway.
pub(crate) fn budget_ms(host: &HostApi) -> f32 {
    let fps = host.cvar(c"maxfps");
    let fps = if (20.0..=1000.0).contains(&fps) { fps } else { 77.0 };
    1000.0 / fps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentile_is_nearest_rank() {
        let v: Vec<f32> = (1..=100).map(|i| i as f32).collect();
        assert_eq!(percentile(&v, 0.95), 95.0);
        assert_eq!(percentile(&v, 0.0), 1.0);
        assert_eq!(percentile(&v, 1.0), 100.0);
        // Single sample and empty are the edges a live window actually hits.
        assert_eq!(percentile(&[7.0], 0.95), 7.0);
        assert_eq!(percentile(&[], 0.95), 0.0);
    }

    #[test]
    fn over_budget_counts_strictly_above_the_line() {
        let mut p = BotProf::default();
        p.add_frame(5.0, 2, 10.0);
        p.add_frame(10.0, 2, 10.0); // exactly at budget is not over
        p.add_frame(10.5, 3, 10.0);
        assert_eq!(p.over, 1);
        assert_eq!(p.peak_bots, 3);
    }

    #[test]
    fn a_budget_change_invalidates_the_window() {
        let mut p = BotProf::default();
        assert!(!p.budget_changed(12.99), "no window yet, nothing to invalidate");
        p.add_frame(1.0, 1, 12.99);
        assert!(!p.budget_changed(12.99));
        assert!(p.budget_changed(25.0), "maxfps moved under us");
    }

    #[test]
    fn phases_are_ignored_unless_the_frame_is_armed() {
        let mut p = BotProf::default();
        p.add_phase(Phase::Objective, 9.8);
        assert_eq!(p.objective.n, 0, "an unarmed frame must not bank a sample");
        p.set_profiling(true);
        p.add_phase(Phase::Objective, 9.8);
        assert_eq!(p.objective.n, 1);
    }

    #[test]
    fn report_shows_headroom_against_the_budget() {
        let mut p = BotProf::default();
        p.set_profiling(true);
        p.add_frame(2.0, 6, 12.99);
        p.add_frame(11.2, 6, 12.99);
        p.add_bot(0.36);
        p.add_phase(Phase::Objective, 9.8);
        let out = p.render(10.0).join("");
        assert!(out.contains("2 frames 6 bots"), "{out}");
        assert!(out.contains("max 11.20 ms"), "{out}");
        // 12.99 - 11.2 = 1.79 of head-room left at the worst frame.
        assert!(out.contains("1.79ms under"), "{out}");
        assert!(out.contains("maxfps 77"), "{out}");
        assert!(out.contains("objective 9.80/9.80"), "{out}");
    }

    /// The autopsy exists to tell the two spike shapes apart, so pin that it can.
    #[test]
    fn worst_frame_autopsy_separates_one_dear_bot_from_a_whole_herd() {
        let mut p = BotProf::default();
        p.set_profiling(true);

        // A herd frame: every bot elevated at once.
        p.begin_frame();
        for _ in 0..4 {
            p.add_bot(5.0);
            p.add_phase(Phase::Objective, 4.5);
        }
        p.add_frame(20.0, 4, 12.99);

        // A dearer frame, but one bot's doing: the autopsy must now describe *this* one.
        p.begin_frame();
        p.add_bot(24.0);
        p.add_phase(Phase::Steer, 23.0);
        p.add_bot(0.1);
        p.add_frame(24.1, 2, 12.99);

        let out = p.render(10.0).join("");
        assert!(out.contains("worst frame 24.10ms"), "{out}");
        assert!(out.contains("per-bot [24.00 0.10]"), "{out}");
        assert!(out.contains("steer 23.00"), "{out}");
        // The herd frame's numbers must not bleed into the autopsy of a later, worse frame.
        assert!(!out.contains("per-bot [5.00"), "{out}");
    }

    #[test]
    fn report_names_an_overrun() {
        let mut p = BotProf::default();
        p.add_frame(18.32, 6, 12.99);
        let out = p.render(10.0).join("");
        assert!(out.contains("5.33ms OVER"), "{out}");
        assert!(out.contains("1/1 over"), "{out}");
    }

    #[test]
    fn reset_keeps_capacity_so_a_window_does_not_reallocate() {
        let mut p = BotProf::default();
        for _ in 0..770 {
            p.add_frame(1.0, 4, 12.99);
        }
        let cap = p.frames.capacity();
        p.reset();
        assert_eq!(p.frames.capacity(), cap);
        assert!(p.started.is_none());
    }
}
