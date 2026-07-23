// SPDX-License-Identifier: AGPL-3.0-or-later

//! Slow, observation-gated team reasoning on an owned worker thread.
//!
//! The engine thread publishes owned [`OracleSnapshot`] values and polls owned [`OraclePlan`]s. The
//! worker receives no host handle, entity reference, or mutable game state, and is joined before the
//! module unloads. Each team has an isolated evidence sheet; CTF is deliberately shadow-only.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

use crate::bot::state::CombatPosture;
use crate::defs::{Bits, Items, Weapon};
use crate::entity::EntId;
use crate::game::{GameState, MAX_EDICTS};
use crate::navmesh::{CellId, LinkCosts, NavGraph};

pub(crate) type OracleEpoch = u64;

const SNAPSHOT_INTERVAL: f32 = 0.25;
const PLAN_INTERVAL: f32 = 1.0;
const INTERCEPT_CONFIDENCE: f32 = 0.65;
const INTERCEPT_MARGIN: f32 = 0.3;
const INTERCEPT_DESTINATIONS: usize = 3;
const INTERCEPT_FAMILY_LIMIT: usize = 2;
const INTERCEPT_MIN_PATH_MASS: f32 = 0.20;
const INTERCEPT_ALT_PENALTY: f32 = 4.0;
const INTERCEPT_ALT_MAX_RATIO: f32 = 1.75;
/// Do not repeat a locally rejected, otherwise identical call every planning tick. A continuously
/// active call is refreshed in place; this applies only after the bot discarded or completed it.
const REISSUE_COOLDOWN: f32 = 4.0;
const MAX_INBOX: usize = 4;
const EVIDENCE_POOLS: usize = 9;
/// Retain a little over one full 10-minute 2on2 match at the observed Bravado proposal rate. The
/// records are diagnostics-only owned values, so this bounded history remains isolated from bot
/// decisions while avoiding a silently truncated A/B result at the end of a match.
const MAX_TRIALS: usize = 4096;
const HOLDOUT_EPISODE: f32 = 15.0;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub(crate) enum AmmoChannel {
    #[default]
    Shells,
    Nails,
    Rockets,
    Cells,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StrategicItemKind {
    Health,
    Mega,
    GreenArmor,
    YellowArmor,
    RedArmor,
    Weapon { bit: u32, ammo: AmmoChannel },
    Ammo(AmmoChannel),
    Quad,
    OtherPowerup,
}

impl StrategicItemKind {
    fn is_major(self) -> bool {
        matches!(self, Self::Mega | Self::RedArmor | Self::Quad | Self::OtherPowerup)
    }

    fn is_strong_weapon(self) -> bool {
        matches!(self, Self::Weapon { bit, .. } if bit == Items::ROCKET_LAUNCHER.bits() || bit == Items::LIGHTNING.bits())
    }
}

#[derive(Clone, Debug)]
pub(crate) struct OracleItem {
    pub ent: u32,
    pub cell: CellId,
    pub kind: StrategicItemKind,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct AmmoSnapshot {
    pub shells: f32,
    pub nails: f32,
    pub rockets: f32,
    pub cells: f32,
}

impl AmmoSnapshot {
    fn channel(self, channel: AmmoChannel) -> f32 {
        match channel {
            AmmoChannel::Shells => self.shells,
            AmmoChannel::Nails => self.nails,
            AmmoChannel::Rockets => self.rockets,
            AmmoChannel::Cells => self.cells,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct MemberSnapshot {
    pub ent: u32,
    pub cell: CellId,
    pub alive: bool,
    pub health: f32,
    pub armor: f32,
    pub items: u32,
    pub ammo: AmmoSnapshot,
    pub recovering: bool,
}

impl MemberSnapshot {
    fn owns(&self, bit: u32) -> bool {
        self.items & bit != 0
    }

    fn armed(&self) -> bool {
        (self.owns(Items::ROCKET_LAUNCHER.bits()) && self.ammo.rockets >= 1.0)
            || (self.owns(Items::LIGHTNING.bits()) && self.ammo.cells >= 1.0)
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct EnemyCue {
    pub cell: CellId,
    pub at: f32,
    pub confidence: f32,
}

#[derive(Clone, Debug)]
pub(crate) struct EnemySnapshot {
    pub ent: u32,
    pub health: Option<f32>,
    pub armor: Option<f32>,
    pub items: Option<u32>,
    /// Newest observation incorporated into this enemy belief.
    pub evidence_at: f32,
    pub cue: Option<EnemyCue>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OracleMode {
    TeamDeathmatch,
    CtfShadow,
}

#[derive(Clone, Debug)]
pub(crate) struct TeamSnapshot {
    pub team: u8,
    pub mode: OracleMode,
    pub members: Vec<MemberSnapshot>,
    pub enemies: Vec<EnemySnapshot>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum EvidenceEventKind {
    ItemTaken {
        item: u32,
        kind: StrategicItemKind,
        picker: u32,
        respawn: Option<f32>,
    },
    WeaponFired {
        player: u32,
        weapon: Weapon,
    },
    Damage {
        attacker: u32,
        target: u32,
        amount: f32,
    },
    PlayerChanged {
        player: u32,
    },
    Death {
        player: u32,
    },
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct EvidenceEvent {
    pub pools: u16,
    pub at: f32,
    pub kind: EvidenceEventKind,
}

#[derive(Clone)]
struct OracleSnapshot {
    epoch: OracleEpoch,
    at: f32,
    graph: Arc<NavGraph>,
    items: Arc<[OracleItem]>,
    teams: Vec<TeamSnapshot>,
    events: Vec<EvidenceEvent>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NuggetKind {
    Rearm,
    Regroup,
    PrepareItem,
    CoverArea,
    Intercept,
}

pub(crate) const NUGGET_KINDS: [NuggetKind; 5] = [
    NuggetKind::Rearm,
    NuggetKind::Regroup,
    NuggetKind::PrepareItem,
    NuggetKind::CoverArea,
    NuggetKind::Intercept,
];

#[derive(Clone, Copy, Debug)]
pub(crate) struct OracleNugget {
    pub epoch: OracleEpoch,
    pub generation: u64,
    pub team: u8,
    pub recipient: u32,
    pub kind: NuggetKind,
    pub target_cell: CellId,
    pub subject: u32,
    pub confidence: f32,
    /// World time the worker made the decision.
    pub decision_at: f32,
    /// Newest observation about `subject` incorporated into the decision. Any later evidence makes
    /// this advice stale before it can influence a bot.
    pub evidence_at: f32,
    pub expires_at: f32,
}

#[derive(Clone, Debug)]
pub(crate) struct TeamPlan {
    pub team: u8,
    pub mode: OracleMode,
    pub control: ControlState,
    pub nuggets: Vec<OracleNugget>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum ControlState {
    Reset,
    Prepare,
    #[default]
    Hold,
}

#[derive(Clone, Debug)]
pub(crate) struct OraclePlan {
    pub epoch: OracleEpoch,
    pub generation: u64,
    pub at: f32,
    pub teams: Vec<TeamPlan>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TrialOutcome {
    Pending,
    Success,
    Invalidated,
    Missed,
}

#[derive(Clone, Copy, Debug)]
struct OracleTrial {
    nugget: OracleNugget,
    episode: u64,
    withheld: bool,
    issued_at: f32,
    applied_at: Option<f32>,
    outcome: TrialOutcome,
    outcome_at: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct EvalSummary {
    pub treated: u32,
    pub treated_success: u32,
    pub controls: u32,
    pub control_success: u32,
    pub applied: u32,
    pub invalidated: u32,
    pub pending: u32,
}

#[derive(Clone, Copy, Debug, Default)]
struct EpisodeEvalState {
    success: bool,
    applied: bool,
    invalidated: bool,
    pending: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct CommunicationSummary {
    pub proposed: u32,
    pub communicated: u32,
    pub refreshed: u32,
    pub suppressed: u32,
    pub superseded: u32,
    pub arm_clears: u32,
}

#[derive(Clone, Copy, Debug)]
struct AdviceMemo {
    nugget: OracleNugget,
    last_seen_at: f32,
    rejected_until: f32,
    resume_on_confirmation: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InboxUpdate {
    Communicated,
    Refreshed,
    Suppressed,
    Superseded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExperimentArm {
    episode: u64,
    withheld: bool,
}

/// Per-bot addressed advice. Fixed-size and allocation-free in the frame loop.
#[derive(Default)]
pub(crate) struct OracleInbox {
    entries: [Option<OracleNugget>; MAX_INBOX],
    active: Option<OracleNugget>,
    last: [Option<AdviceMemo>; NUGGET_KINDS.len()],
}

impl OracleInbox {
    fn push(&mut self, nugget: OracleNugget) -> InboxUpdate {
        let memo_index = nugget_kind_index(nugget.kind);
        if let Some(slot) = self
            .entries
            .iter_mut()
            .find(|slot| slot.is_some_and(|old| old.kind == nugget.kind))
        {
            let old = slot.unwrap();
            *slot = Some(nugget);
            self.last[memo_index] = Some(advice_memo(nugget));
            if same_advice(old, nugget) {
                if self.active.is_some_and(|active| active.kind == nugget.kind) {
                    // The worker revalidated the same instruction. Keep one persistent acknowledgement
                    // instead of making the next frame cancel and re-apply a new generation.
                    self.active = Some(nugget);
                }
                return InboxUpdate::Refreshed;
            }
            // Keep an acknowledged old instruction until this frame either applies the replacement
            // or the next freshness pass returns it as cancelled and releases its old item goal.
            return InboxUpdate::Superseded;
        }
        let prior = self.last[memo_index];
        let same_prior = prior.is_some_and(|memo| same_advice(memo.nugget, nugget));
        if prior.is_some_and(|memo| same_prior && nugget.decision_at < memo.rejected_until) {
            return InboxUpdate::Suppressed;
        }
        let resumed = prior.is_some_and(|memo| {
            same_prior && memo.resume_on_confirmation && nugget.evidence_at >= memo.nugget.evidence_at
        });
        if prior.is_some_and(|memo| same_prior && !resumed && nugget.decision_at - memo.last_seen_at < REISSUE_COOLDOWN)
        {
            return InboxUpdate::Suppressed;
        }
        if let Some(slot) = self.entries.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(nugget);
            self.last[memo_index] = Some(advice_memo(nugget));
            return if resumed {
                InboxUpdate::Refreshed
            } else {
                InboxUpdate::Communicated
            };
        }
        let oldest = self
            .entries
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.unwrap().expires_at.total_cmp(&b.unwrap().expires_at))
            .map(|(index, _)| index)
            .unwrap_or(0);
        self.entries[oldest] = Some(nugget);
        self.last[memo_index] = Some(advice_memo(nugget));
        if resumed {
            InboxUpdate::Refreshed
        } else {
            InboxUpdate::Communicated
        }
    }

    pub(crate) fn retain_live(
        &mut self,
        epoch: OracleEpoch,
        now: f32,
        evidence_revision: &[f32; MAX_EDICTS],
    ) -> Option<OracleNugget> {
        for index in 0..self.entries.len() {
            let Some(nugget) = self.entries[index] else {
                continue;
            };
            let subject = freshness_subject(nugget);
            let evidence_stale = subject != 0
                && evidence_revision
                    .get(subject as usize)
                    .is_some_and(|&latest| latest > nugget.evidence_at);
            if nugget.epoch != epoch || nugget.expires_at <= now || evidence_stale {
                self.entries[index] = None;
                if evidence_stale {
                    let memo_index = nugget_kind_index(nugget.kind);
                    if let Some(memo) = &mut self.last[memo_index] {
                        if same_advice(memo.nugget, nugget) {
                            memo.resume_on_confirmation = true;
                        }
                    }
                }
            }
        }
        let cancelled = self.active.filter(|active| {
            !self
                .entries
                .iter()
                .flatten()
                .any(|entry| entry.generation == active.generation && entry.kind == active.kind)
        });
        if cancelled.is_some() {
            self.active = None;
        }
        cancelled
    }

    pub(crate) fn best(&self, now: f32) -> Option<OracleNugget> {
        self.entries
            .iter()
            .flatten()
            .filter(|n| n.expires_at > now)
            .max_by(|a, b| {
                nugget_priority(a.kind)
                    .cmp(&nugget_priority(b.kind))
                    .then_with(|| a.confidence.total_cmp(&b.confidence))
            })
            .copied()
    }

    #[cfg(test)]
    pub(crate) fn entries(&self) -> impl Iterator<Item = OracleNugget> + '_ {
        self.entries.iter().flatten().copied()
    }

    pub(crate) fn mark_applied(&mut self, nugget: OracleNugget) {
        self.active = Some(nugget);
    }

    pub(crate) fn discard(&mut self, nugget: OracleNugget, now: f32) {
        for entry in &mut self.entries {
            if entry.is_some_and(|old| old.generation == nugget.generation && old.kind == nugget.kind) {
                *entry = None;
            }
        }
        if self
            .active
            .is_some_and(|old| old.generation == nugget.generation && old.kind == nugget.kind)
        {
            self.active = None;
        }
        let memo_index = nugget_kind_index(nugget.kind);
        if let Some(memo) = &mut self.last[memo_index] {
            if same_advice(memo.nugget, nugget) {
                memo.last_seen_at = now;
                memo.rejected_until = now + REISSUE_COOLDOWN;
                memo.resume_on_confirmation = false;
            }
        }
    }

    pub(crate) fn clear(&mut self) -> Option<OracleNugget> {
        self.entries = [None; MAX_INBOX];
        self.active.take()
    }

    pub(crate) fn reset(&mut self) -> Option<OracleNugget> {
        self.last = [None; NUGGET_KINDS.len()];
        self.clear()
    }
}

fn advice_memo(nugget: OracleNugget) -> AdviceMemo {
    AdviceMemo {
        nugget,
        last_seen_at: nugget.decision_at,
        rejected_until: 0.0,
        resume_on_confirmation: false,
    }
}

fn nugget_kind_index(kind: NuggetKind) -> usize {
    NUGGET_KINDS
        .iter()
        .position(|&candidate| candidate == kind)
        .unwrap_or(0)
}

fn same_advice(a: OracleNugget, b: OracleNugget) -> bool {
    a.epoch == b.epoch
        && a.team == b.team
        && a.recipient == b.recipient
        && a.kind == b.kind
        && a.target_cell == b.target_cell
        && a.subject == b.subject
}

fn nugget_priority(kind: NuggetKind) -> u8 {
    match kind {
        NuggetKind::Rearm => 5,
        NuggetKind::Regroup => 4,
        NuggetKind::PrepareItem => 3,
        NuggetKind::Intercept => 2,
        NuggetKind::CoverArea => 1,
    }
}

/// Entity whose newer evidence can contradict a nugget. A regroup uses the teammate id only as its
/// outcome subject; ordinary shots and inventory changes do not invalidate a short rendezvous, and
/// the worker refreshes teammate position every second.
fn freshness_subject(nugget: OracleNugget) -> u32 {
    if nugget.kind == NuggetKind::Regroup {
        0
    } else {
        nugget.subject
    }
}

#[derive(Default)]
struct MailboxState {
    stop: bool,
    input: Option<OracleSnapshot>,
    output: Option<OraclePlan>,
}

#[derive(Default)]
struct Mailbox {
    state: Mutex<MailboxState>,
    wake: Condvar,
}

struct Worker {
    mailbox: Arc<Mailbox>,
    handle: JoinHandle<()>,
}

pub(crate) struct OracleRuntime {
    worker: Option<Worker>,
    epoch: OracleEpoch,
    next_publish: f32,
    next_debug: f32,
    pending_events: Vec<EvidenceEvent>,
    /// Main-thread truth about when each *honest evidence record* last changed. This is not game
    /// truth: only [`Self::note`] and published perception/model snapshots advance it.
    evidence_revision: Box<[[f32; MAX_EDICTS]; EVIDENCE_POOLS]>,
    last_plan: Option<OraclePlan>,
    evaluation: bool,
    trials: VecDeque<OracleTrial>,
    communication: CommunicationSummary,
    arms: [Option<ExperimentArm>; EVIDENCE_POOLS],
}

impl Default for OracleRuntime {
    fn default() -> Self {
        Self {
            worker: None,
            epoch: 0,
            next_publish: 0.0,
            next_debug: 0.0,
            pending_events: Vec::new(),
            evidence_revision: Box::new([[0.0; MAX_EDICTS]; EVIDENCE_POOLS]),
            last_plan: None,
            evaluation: false,
            trials: VecDeque::new(),
            communication: CommunicationSummary::default(),
            arms: [None; EVIDENCE_POOLS],
        }
    }
}

impl OracleRuntime {
    pub(crate) fn ensure(&mut self, wanted: bool) {
        if wanted && self.worker.is_none() {
            let mailbox = Arc::new(Mailbox::default());
            let worker_mailbox = Arc::clone(&mailbox);
            let handle = match std::thread::Builder::new()
                .name("rtx-oracle".into())
                .spawn(move || worker_loop(worker_mailbox))
            {
                Ok(handle) => handle,
                Err(_) => return,
            };
            self.worker = Some(Worker { mailbox, handle });
        } else if !wanted && self.worker.is_some() {
            self.shutdown();
        }
    }

    pub(crate) fn bump_epoch(&mut self) {
        self.epoch = self.epoch.wrapping_add(1).max(1);
        self.next_publish = 0.0;
        self.next_debug = 0.0;
        self.pending_events.clear();
        for pool in self.evidence_revision.iter_mut() {
            pool.fill(0.0);
        }
        self.last_plan = None;
        // An epoch is a different map/mode/roster or a fresh live match. Keep its experiment sample
        // self-contained instead of mixing warmup or the previous map into treated/control rates.
        self.trials.clear();
        self.communication = CommunicationSummary::default();
        self.arms = [None; EVIDENCE_POOLS];
        if let Some(worker) = &self.worker {
            let mut state = lock(&worker.mailbox.state);
            state.input = None;
            state.output = None;
        }
    }

    fn publish(&mut self, snapshot: OracleSnapshot) {
        let Some(worker) = &self.worker else { return };
        let mut state = lock(&worker.mailbox.state);
        state.input = Some(snapshot);
        worker.mailbox.wake.notify_one();
    }

    fn poll_plan(&mut self) -> Option<OraclePlan> {
        let worker = self.worker.as_ref()?;
        let plan = lock(&worker.mailbox.state).output.take()?;
        if plan.epoch != self.epoch {
            return None;
        }
        self.last_plan = Some(plan.clone());
        Some(plan)
    }

    pub(crate) fn note(&mut self, event: EvidenceEvent) {
        if self.worker.is_some() {
            let (affected, count) = match event.kind {
                EvidenceEventKind::ItemTaken { item, picker, .. } => ([item, picker], 2),
                EvidenceEventKind::WeaponFired { player, .. }
                | EvidenceEventKind::PlayerChanged { player }
                | EvidenceEventKind::Death { player } => ([player, 0], 1),
                EvidenceEventKind::Damage { attacker, target, .. } => ([attacker, target], 2),
            };
            for trial in &mut self.trials {
                if trial.outcome == TrialOutcome::Pending
                    && event.pools & (1 << trial.nugget.team) != 0
                    && event.at > trial.nugget.evidence_at
                    && affected[..count].contains(&freshness_subject(trial.nugget))
                {
                    trial.outcome = TrialOutcome::Invalidated;
                    trial.outcome_at = event.at;
                }
            }
            for team in 0..EVIDENCE_POOLS {
                if event.pools & (1 << team) == 0 {
                    continue;
                }
                let revisions = &mut self.evidence_revision[team];
                match event.kind {
                    EvidenceEventKind::ItemTaken { item, picker, .. } => {
                        set_revision(revisions, item, event.at);
                        set_revision(revisions, picker, event.at);
                    }
                    EvidenceEventKind::WeaponFired { player, .. }
                    | EvidenceEventKind::PlayerChanged { player }
                    | EvidenceEventKind::Death { player } => {
                        set_revision(revisions, player, event.at);
                    }
                    EvidenceEventKind::Damage { attacker, target, .. } => {
                        set_revision(revisions, attacker, event.at);
                        set_revision(revisions, target, event.at);
                    }
                }
            }
            self.pending_events.push(event);
        }
    }

    pub(crate) fn running(&self) -> bool {
        self.worker.is_some()
    }

    pub(crate) fn epoch(&self) -> OracleEpoch {
        self.epoch
    }

    pub(crate) fn last_plan(&self) -> Option<&OraclePlan> {
        self.last_plan.as_ref()
    }

    pub(crate) fn last_output(&self) -> f32 {
        self.last_plan.as_ref().map_or(0.0, |plan| plan.at)
    }

    pub(crate) fn set_evaluation(&mut self, enabled: bool) {
        if self.evaluation && !enabled {
            self.trials.clear();
        }
        self.evaluation = enabled;
    }

    fn record_trial(&mut self, trial: OracleTrial) {
        if !self.evaluation {
            return;
        }
        if let Some(existing) = self.trials.iter_mut().find(|existing| {
            existing.outcome == TrialOutcome::Pending
                && existing.episode == trial.episode
                && existing.withheld == trial.withheld
                && existing.nugget.team == trial.nugget.team
                && existing.nugget.recipient == trial.nugget.recipient
                && existing.nugget.kind == trial.nugget.kind
                && existing.nugget.subject == trial.nugget.subject
                && existing.nugget.target_cell == trial.nugget.target_cell
        }) {
            existing.nugget = trial.nugget;
            return;
        }
        if self.trials.len() == MAX_TRIALS {
            self.trials.pop_front();
        }
        self.trials.push_back(trial);
    }

    pub(crate) fn mark_applied(&mut self, nugget: OracleNugget, at: f32) {
        if let Some(trial) = self.trials.iter_mut().rev().find(|trial| {
            trial.outcome == TrialOutcome::Pending
                && !trial.withheld
                && trial.nugget.generation == nugget.generation
                && trial.nugget.recipient == nugget.recipient
                && trial.nugget.kind == nugget.kind
        }) {
            trial.applied_at.get_or_insert(at);
        }
    }

    pub(crate) fn invalidate_trial(&mut self, nugget: OracleNugget, at: f32) {
        for trial in &mut self.trials {
            if trial.outcome == TrialOutcome::Pending
                && trial.nugget.team == nugget.team
                && trial.nugget.recipient == nugget.recipient
                && trial.nugget.kind == nugget.kind
                && trial.nugget.subject == nugget.subject
            {
                trial.outcome = TrialOutcome::Invalidated;
                trial.outcome_at = at;
            }
        }
    }

    fn succeed_where(&mut self, at: f32, mut matches: impl FnMut(&OracleTrial) -> bool) {
        for trial in &mut self.trials {
            if trial.outcome == TrialOutcome::Pending && at >= trial.issued_at && matches(trial) {
                trial.outcome = TrialOutcome::Success;
                trial.outcome_at = at;
            }
        }
    }

    pub(crate) fn note_item_outcome(&mut self, item: EntId, picker: EntId, picker_team: u8, at: f32) {
        self.succeed_where(at, |trial| {
            trial.nugget.subject == item.0
                && match trial.nugget.kind {
                    NuggetKind::Rearm | NuggetKind::PrepareItem => trial.nugget.recipient == picker.0,
                    NuggetKind::CoverArea => trial.nugget.team == picker_team,
                    _ => false,
                }
        });
    }

    pub(crate) fn note_damage_outcome(
        &mut self,
        attacker: EntId,
        target: EntId,
        attacker_cell: Option<CellId>,
        graph: Option<&NavGraph>,
        at: f32,
    ) {
        self.succeed_where(at, |trial| {
            trial.nugget.kind == NuggetKind::Intercept
                && trial.nugget.recipient == attacker.0
                && trial.nugget.subject == target.0
                && attacker_cell.zip(graph).is_some_and(|(cell, graph)| {
                    graph.cluster_of(cell).is_some()
                        && graph.cluster_of(cell) == graph.cluster_of(trial.nugget.target_cell)
                })
        });
    }

    fn note_regroup_outcome(&mut self, recipient: EntId, teammate: EntId, at: f32) {
        self.succeed_where(at, |trial| {
            trial.nugget.kind == NuggetKind::Regroup
                && trial.nugget.recipient == recipient.0
                && trial.nugget.subject == teammate.0
        });
    }

    fn expire_trials(&mut self, now: f32) {
        for trial in &mut self.trials {
            if trial.outcome == TrialOutcome::Pending && trial.nugget.expires_at <= now {
                trial.outcome = TrialOutcome::Missed;
                trial.outcome_at = now;
            }
        }
    }

    fn close_pending_trials(&mut self, now: f32) {
        for trial in &mut self.trials {
            if trial.outcome == TrialOutcome::Pending {
                trial.outcome = TrialOutcome::Missed;
                trial.outcome_at = now;
            }
        }
    }

    pub(crate) fn eval_summary(&self) -> EvalSummary {
        self.eval_summary_matching(|_| true)
    }

    pub(crate) fn eval_summary_for(&self, kind: NuggetKind) -> EvalSummary {
        self.eval_summary_matching(|trial| trial.nugget.kind == kind)
    }

    pub(crate) fn eval_episode_summary(&self) -> EvalSummary {
        self.eval_episode_summary_matching(|_| true)
    }

    pub(crate) fn eval_episode_summary_for(&self, kind: NuggetKind) -> EvalSummary {
        self.eval_episode_summary_matching(|trial| trial.nugget.kind == kind)
    }

    pub(crate) fn communication_summary(&self) -> CommunicationSummary {
        self.communication
    }

    fn note_inbox_update(&mut self, update: InboxUpdate) {
        match update {
            InboxUpdate::Communicated => {
                self.communication.communicated = self.communication.communicated.saturating_add(1);
            }
            InboxUpdate::Refreshed => {
                self.communication.refreshed = self.communication.refreshed.saturating_add(1);
            }
            InboxUpdate::Suppressed => {
                self.communication.suppressed = self.communication.suppressed.saturating_add(1);
            }
            InboxUpdate::Superseded => {
                self.communication.communicated = self.communication.communicated.saturating_add(1);
                self.communication.superseded = self.communication.superseded.saturating_add(1);
            }
        }
    }

    fn note_proposed(&mut self) {
        self.communication.proposed = self.communication.proposed.saturating_add(1);
    }

    fn close_arm_trials(&mut self, team: u8, episode: u64, now: f32) {
        for trial in &mut self.trials {
            if trial.outcome == TrialOutcome::Pending && trial.nugget.team == team && trial.episode == episode {
                trial.outcome = TrialOutcome::Missed;
                trial.outcome_at = now;
            }
        }
    }

    /// Advance experiment arms independently of worker output. This prevents a treated instruction
    /// from surviving into a shadow-control episode merely because the next 1 Hz plan has not arrived.
    fn advance_arms(&mut self, now: f32, holdout: f32) -> Vec<u8> {
        let mut clears = Vec::new();
        for team in 1..EVIDENCE_POOLS {
            let Some(old) = self.arms[team] else { continue };
            let (episode, withheld) = plan_holdout(self.epoch, team as u8, now, holdout);
            let new = ExperimentArm { episode, withheld };
            if new == old {
                continue;
            }
            self.close_arm_trials(team as u8, old.episode, now);
            if new.withheld != old.withheld {
                clears.push(team as u8);
                self.communication.arm_clears = self.communication.arm_clears.saturating_add(1);
            }
            self.arms[team] = Some(new);
        }
        clears
    }

    fn arm(&mut self, team: u8, now: f32, holdout: f32) -> ExperimentArm {
        let index = team as usize;
        let (episode, withheld) = plan_holdout(self.epoch, team, now, holdout);
        let arm = ExperimentArm { episode, withheld };
        if let Some(slot) = self.arms.get_mut(index) {
            *slot = Some(arm);
        }
        arm
    }

    fn eval_summary_matching(&self, mut matches: impl FnMut(&OracleTrial) -> bool) -> EvalSummary {
        let mut summary = EvalSummary::default();
        for trial in self.trials.iter().filter(|trial| matches(trial)) {
            if trial.withheld {
                summary.controls += 1;
                summary.control_success += (trial.outcome == TrialOutcome::Success) as u32;
            } else {
                summary.treated += 1;
                summary.treated_success += (trial.outcome == TrialOutcome::Success) as u32;
                summary.applied += trial.applied_at.is_some() as u32;
            }
            summary.invalidated += (trial.outcome == TrialOutcome::Invalidated) as u32;
            summary.pending += (trial.outcome == TrialOutcome::Pending) as u32;
        }
        summary
    }

    /// Collapse correlated replans into one result per team, experiment arm, and optional kind.
    /// Success takes precedence over a still-pending or invalidated sibling trial; otherwise each
    /// episode receives exactly one terminal classification.
    fn eval_episode_summary_matching(&self, mut matches: impl FnMut(&OracleTrial) -> bool) -> EvalSummary {
        let mut episodes = HashMap::<(u8, u64, bool), EpisodeEvalState>::new();
        for trial in self.trials.iter().filter(|trial| matches(trial)) {
            let state = episodes
                .entry((trial.nugget.team, trial.episode, trial.withheld))
                .or_default();
            state.success |= trial.outcome == TrialOutcome::Success;
            state.applied |= trial.applied_at.is_some();
            state.invalidated |= trial.outcome == TrialOutcome::Invalidated;
            state.pending |= trial.outcome == TrialOutcome::Pending;
        }

        let mut summary = EvalSummary::default();
        for ((_, _, withheld), state) in episodes {
            if withheld {
                summary.controls += 1;
                summary.control_success += state.success as u32;
            } else {
                summary.treated += 1;
                summary.treated_success += state.success as u32;
                summary.applied += state.applied as u32;
            }
            if !state.success {
                if state.pending {
                    summary.pending += 1;
                } else if state.invalidated {
                    summary.invalidated += 1;
                }
            }
        }
        summary
    }

    pub(crate) fn shutdown(&mut self) {
        let Some(worker) = self.worker.take() else { return };
        {
            let mut state = lock(&worker.mailbox.state);
            state.stop = true;
            state.input = None;
            worker.mailbox.wake.notify_one();
        }
        let _ = worker.handle.join();
        self.pending_events.clear();
        self.last_plan = None;
    }
}

/// Drain completed plans before bots choose this frame. CTF plans stay visible in diagnostics but
/// are never delivered to an inbox.
pub(crate) fn frame_begin(game: &mut GameState) {
    let wanted = game.host().cvar_bool(c"rtx_bot_oracle");
    let evaluation = game.host().cvar_bool(c"rtx_bot_oracle_eval");
    let holdout = if evaluation {
        game.host().cvar(c"rtx_bot_oracle_holdout").clamp(0.0, 1.0)
    } else {
        0.0
    };
    game.oracle.ensure(wanted);
    game.oracle.set_evaluation(wanted && evaluation);
    let epoch = game.oracle.epoch();
    let now = game.time();
    let evaluation_live =
        evaluation && matches!(game.team_match.phase, crate::mode::MatchPhase::Live);
    if evaluation && !evaluation_live {
        // Freeze the live intention-to-treat sample at the match boundary. Warmup pickups and
        // damage must not turn unresolved match advice into successes or add new trials.
        game.oracle.close_pending_trials(now);
    }
    if !wanted {
        clear_inboxes(game);
        return;
    }
    let arm_clears = game.oracle.advance_arms(now, holdout);
    for team in arm_clears {
        clear_team_inboxes(game, team);
    }
    for player in crate::mode::players(game) {
        if game.entities[player].bot.is_bot {
            let team = game.entities[player].mode_p.team as usize;
            let revisions = game
                .oracle
                .evidence_revision
                .get(team)
                .unwrap_or(&game.oracle.evidence_revision[0]);
            let cancelled = game.entities[player].bot.oracle.retain_live(epoch, now, revisions);
            if let Some(cancelled) = cancelled {
                if now < cancelled.expires_at {
                    game.oracle.invalidate_trial(cancelled, now);
                }
                game.entities[player].bot.goal.next_pick = now;
                if matches!(cancelled.kind, NuggetKind::Rearm | NuggetKind::PrepareItem)
                    && game.entities[player].bot.goal.item == cancelled.subject
                    && game.entities[player].bot.goal.commit == crate::bot::state::GoalCommit::None
                {
                    let goal = &mut game.entities[player].bot.goal;
                    goal.item = 0;
                    goal.next_item = 0;
                    goal.next_pick = now;
                }
            }
        }
    }
    let Some(plan) = game.oracle.poll_plan() else { return };
    for team in plan.teams {
        if team.mode == OracleMode::CtfShadow {
            continue;
        }
        let arm = game.oracle.arm(team.team, now, holdout);
        for nugget in team.nuggets {
            game.oracle.note_proposed();
            let revisions = game
                .oracle
                .evidence_revision
                .get(nugget.team as usize)
                .unwrap_or(&game.oracle.evidence_revision[0]);
            let subject = freshness_subject(nugget);
            if subject != 0
                && revisions
                    .get(subject as usize)
                    .is_some_and(|&at| at > nugget.evidence_at)
            {
                continue;
            }
            if evaluation_live && trial_eligible(game, nugget) {
                game.oracle.record_trial(OracleTrial {
                    nugget,
                    episode: arm.episode,
                    withheld: arm.withheld,
                    issued_at: now,
                    applied_at: None,
                    outcome: TrialOutcome::Pending,
                    outcome_at: 0.0,
                });
            }
            if arm.withheld {
                continue;
            }
            let recipient = EntId(nugget.recipient);
            let Some(ent) = game.entities.get_mut(recipient.0 as usize) else {
                continue;
            };
            if ent.in_use && ent.bot.is_bot && ent.mode_p.team == nugget.team {
                let update = ent.bot.oracle.push(nugget);
                game.oracle.note_inbox_update(update);
            }
        }
    }
}

fn plan_holdout(epoch: OracleEpoch, team: u8, at: f32, fraction: f32) -> (u64, bool) {
    let episode = (at.max(0.0) / HOLDOUT_EPISODE).floor() as u64;
    if fraction <= 0.0 {
        return (episode, false);
    }
    let mut hash = epoch ^ episode.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ u64::from(team);
    hash ^= hash >> 30;
    hash = hash.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    hash ^= hash >> 27;
    hash = hash.wrapping_mul(0x94d0_49bb_1331_11eb);
    hash ^= hash >> 31;
    let unit = (hash >> 11) as f64 / (1u64 << 53) as f64;
    (episode, unit < f64::from(fraction))
}

fn trial_eligible(game: &GameState, nugget: OracleNugget) -> bool {
    let recipient = EntId(nugget.recipient);
    if recipient.0 as usize >= game.entities.len() || !game.entities[recipient].is_alive() {
        return false;
    }
    match nugget.kind {
        NuggetKind::Rearm => {
            !((game.entities[recipient].v.items.has(Items::ROCKET_LAUNCHER)
                && game.entities[recipient].v.ammo_rockets >= 1.0)
                || (game.entities[recipient].v.items.has(Items::LIGHTNING)
                    && game.entities[recipient].v.ammo_cells >= 1.0))
        }
        NuggetKind::Regroup => {
            let teammate = EntId(nugget.subject);
            (teammate.0 as usize) < game.entities.len()
                && game.entities[teammate].is_alive()
                && (game.entities[recipient].v.origin - game.entities[teammate].v.origin).length() > 192.0
        }
        NuggetKind::PrepareItem | NuggetKind::CoverArea => {
            nugget.subject != 0 && (nugget.subject as usize) < game.entities.len()
        }
        NuggetKind::Intercept => {
            let enemy = EntId(nugget.subject);
            (enemy.0 as usize) < game.entities.len() && game.entities[enemy].is_alive()
        }
    }
}

/// Gather a strict team snapshot after every bot has updated perception/goals, no faster than 4 Hz.
pub(crate) fn frame_end(game: &mut GameState) {
    evaluate_outcomes(game);
    debug_report(game);
    if !game.oracle.running() || game.time() < game.oracle.next_publish {
        return;
    }
    let now = game.time();
    game.oracle.next_publish = now + SNAPSHOT_INTERVAL;
    let Some(graph) = game.nav.graph.clone() else { return };
    let items: Arc<[OracleItem]> = game
        .nav
        .goals
        .iter()
        .filter_map(|&(ent, cell)| oracle_item(game, EntId(ent), cell))
        .collect::<Vec<_>>()
        .into();
    let teams = team_snapshots(game, &graph, now);
    for team in &teams {
        let revisions = &mut game.oracle.evidence_revision[team.team as usize];
        for enemy in &team.enemies {
            set_revision(revisions, enemy.ent, enemy.evidence_at);
        }
    }
    let events = std::mem::take(&mut game.oracle.pending_events);
    let snapshot = OracleSnapshot {
        epoch: game.oracle.epoch(),
        at: now,
        graph,
        items,
        teams,
        events,
    };
    game.oracle.publish(snapshot);
}

fn debug_report(game: &mut GameState) {
    if !game.host().cvar_bool(c"rtx_bot_oracle_debug") || game.time() < game.oracle.next_debug {
        return;
    }
    let now = game.time();
    game.oracle.next_debug = now + 2.0;
    let (generation, teams, nuggets) = game.oracle.last_plan.as_ref().map_or((0, 0, 0), |plan| {
        (
            plan.generation,
            plan.teams.len(),
            plan.teams.iter().map(|team| team.nuggets.len()).sum(),
        )
    });
    let eval = game.oracle.eval_summary();
    let comms = game.oracle.communication_summary();
    game.host().conprint(&crate::game::cstring(&format!(
        "rtx oracle: epoch={} gen={generation} teams={teams} nuggets={nuggets} calls={}/{} refresh={} suppress={} eval={}/{} control={}/{} applied={} stale={} pending={}\n",
        game.oracle.epoch(),
        comms.communicated,
        comms.proposed,
        comms.refreshed,
        comms.suppressed,
        eval.treated_success,
        eval.treated,
        eval.control_success,
        eval.controls,
        eval.applied,
        eval.invalidated,
        eval.pending,
    )));
}

fn evaluate_outcomes(game: &mut GameState) {
    if !game.oracle.evaluation {
        return;
    }
    let now = game.time();
    let regroup: Vec<OracleNugget> = game
        .oracle
        .trials
        .iter()
        .filter(|trial| trial.outcome == TrialOutcome::Pending && trial.nugget.kind == NuggetKind::Regroup)
        .map(|trial| trial.nugget)
        .collect();
    for nugget in regroup {
        let recipient = EntId(nugget.recipient);
        let teammate = EntId(nugget.subject);
        if recipient.0 as usize >= game.entities.len()
            || teammate.0 as usize >= game.entities.len()
            || !game.entities[recipient].is_alive()
            || !game.entities[teammate].is_alive()
        {
            continue;
        }
        if (game.entities[recipient].v.origin - game.entities[teammate].v.origin).length() <= 192.0 {
            game.oracle.note_regroup_outcome(recipient, teammate, now);
        }
    }
    game.oracle.expire_trials(now);
}

fn clear_inboxes(game: &mut GameState) {
    let now = game.time();
    for player in crate::mode::players(game) {
        if game.entities[player].bot.is_bot {
            let active = game.entities[player].bot.oracle.reset();
            if let Some(active) = active {
                let goal = &mut game.entities[player].bot.goal;
                if matches!(active.kind, NuggetKind::Rearm | NuggetKind::PrepareItem)
                    && goal.item == active.subject
                    && goal.commit == crate::bot::state::GoalCommit::None
                {
                    goal.item = 0;
                    goal.next_item = 0;
                }
                goal.next_pick = now;
            }
        }
    }
}

fn clear_team_inboxes(game: &mut GameState, team: u8) {
    let now = game.time();
    for player in crate::mode::players(game) {
        if !game.entities[player].bot.is_bot || game.entities[player].mode_p.team != team {
            continue;
        }
        let active = game.entities[player].bot.oracle.clear();
        if let Some(active) = active {
            let goal = &mut game.entities[player].bot.goal;
            if matches!(active.kind, NuggetKind::Rearm | NuggetKind::PrepareItem)
                && goal.item == active.subject
                && goal.commit == crate::bot::state::GoalCommit::None
            {
                goal.item = 0;
                goal.next_item = 0;
            }
            goal.next_pick = now;
        }
    }
}

fn team_snapshots(game: &GameState, graph: &NavGraph, now: f32) -> Vec<TeamSnapshot> {
    let players = crate::mode::players(game);
    let mut teams = Vec::new();
    for team in 1..=8u8 {
        let bots: Vec<EntId> = players
            .iter()
            .copied()
            .filter(|&e| game.entities[e].bot.is_bot && game.entities[e].mode_p.team == team)
            .collect();
        if bots.len() < 2 {
            continue;
        }
        let mode = match game.mode.name() {
            "dm" => OracleMode::TeamDeathmatch,
            "ctf" => OracleMode::CtfShadow,
            _ => continue,
        };
        let members = bots.iter().filter_map(|&e| member_snapshot(game, graph, e)).collect();
        let observer = bots[0];
        let enemies = players
            .iter()
            .copied()
            .filter(|&e| game.entities[e].mode_p.team != team)
            .filter_map(|enemy| enemy_snapshot(game, graph, &bots, observer, enemy, now))
            .collect();
        teams.push(TeamSnapshot {
            team,
            mode,
            members,
            enemies,
        });
    }
    teams
}

fn member_snapshot(game: &GameState, graph: &NavGraph, e: EntId) -> Option<MemberSnapshot> {
    let ent = &game.entities[e];
    let cell = graph.nearest(ent.v.origin)?;
    Some(MemberSnapshot {
        ent: e.0,
        cell,
        alive: ent.is_alive(),
        health: ent.v.health,
        armor: ent.v.armorvalue,
        items: Items::from_f32(ent.v.items).bits(),
        ammo: AmmoSnapshot {
            shells: ent.v.ammo_shells,
            nails: ent.v.ammo_nails,
            rockets: ent.v.ammo_rockets,
            cells: ent.v.ammo_cells,
        },
        recovering: ent.bot.posture == CombatPosture::Recover,
    })
}

fn enemy_snapshot(
    game: &GameState,
    graph: &NavGraph,
    bots: &[EntId],
    observer: EntId,
    enemy: EntId,
    now: f32,
) -> Option<EnemySnapshot> {
    if !game.entities[enemy].is_player() {
        return None;
    }
    let estimate = game.opponent_est(observer, enemy, now);
    let cue = bots
        .iter()
        .filter_map(|&bot| {
            let b = &game.entities[bot].bot;
            if b.percept.known_enemy != enemy.0 || b.percept.known_until <= now {
                return None;
            }
            let exact = now - b.seen.time <= SNAPSHOT_INTERVAL * 1.5;
            let point = if exact { b.seen.at } else { b.percept.last_seen };
            Some(EnemyCue {
                cell: graph.nearest(point)?,
                at: b.percept.known_until - crate::bot::perception::MEMORY,
                confidence: if exact { 0.95 } else { 0.72 },
            })
        })
        .max_by(|a, b| a.at.total_cmp(&b.at));
    Some(EnemySnapshot {
        ent: enemy.0,
        health: estimate.map(|e| e.health),
        armor: estimate.map(|e| e.armor_value),
        items: estimate.map(|e| Items::from_f32(e.items).bits()),
        evidence_at: estimate
            .map(|e| e.last_update)
            .unwrap_or(0.0)
            .max(cue.map(|c| c.at).unwrap_or(0.0)),
        cue,
    })
}

fn oracle_item(game: &GameState, e: EntId, cell: CellId) -> Option<OracleItem> {
    let kind = classify_item(game, e)?;
    Some(OracleItem { ent: e.0, cell, kind })
}

fn classify_item(game: &GameState, e: EntId) -> Option<StrategicItemKind> {
    let ent = &game.entities[e];
    let class = ent.classname()?;
    Some(match class {
        "item_health" if ent.item.healtype == 2.0 => StrategicItemKind::Mega,
        "item_health" => StrategicItemKind::Health,
        "item_armor1" => StrategicItemKind::GreenArmor,
        "item_armor2" => StrategicItemKind::YellowArmor,
        "item_armorInv" => StrategicItemKind::RedArmor,
        "weapon_rocketlauncher" => StrategicItemKind::Weapon {
            bit: Items::ROCKET_LAUNCHER.bits(),
            ammo: AmmoChannel::Rockets,
        },
        "weapon_lightning" => StrategicItemKind::Weapon {
            bit: Items::LIGHTNING.bits(),
            ammo: AmmoChannel::Cells,
        },
        "weapon_supershotgun" => StrategicItemKind::Weapon {
            bit: Items::SUPER_SHOTGUN.bits(),
            ammo: AmmoChannel::Shells,
        },
        "weapon_nailgun" | "weapon_supernailgun" => StrategicItemKind::Weapon {
            bit: if class == "weapon_nailgun" {
                Items::NAILGUN.bits()
            } else {
                Items::SUPER_NAILGUN.bits()
            },
            ammo: AmmoChannel::Nails,
        },
        "weapon_grenadelauncher" => StrategicItemKind::Weapon {
            bit: Items::GRENADE_LAUNCHER.bits(),
            ammo: AmmoChannel::Rockets,
        },
        "item_shells" => StrategicItemKind::Ammo(AmmoChannel::Shells),
        "item_spikes" => StrategicItemKind::Ammo(AmmoChannel::Nails),
        "item_rockets" => StrategicItemKind::Ammo(AmmoChannel::Rockets),
        "item_cells" => StrategicItemKind::Ammo(AmmoChannel::Cells),
        "item_artifact_super_damage" => StrategicItemKind::Quad,
        c if c.starts_with("item_artifact_") => StrategicItemKind::OtherPowerup,
        _ => return None,
    })
}

/// Record the disappearance of a strategic map item using only teams that could hear it (plus the
/// picker's own team). The item and player revisions are advanced together, so a route prediction
/// based on either old availability or an old enemy loadout cannot survive this event.
pub(crate) fn note_item_taken(game: &mut GameState, item: EntId, picker: EntId, at: f32) {
    let Some(kind) = classify_item(game, item) else { return };
    let picker_team = game.entities[picker].mode_p.team;
    game.oracle.note_item_outcome(item, picker, picker_team, at);
    let mut pools = game.evidence_pools(game.entities[item].v.origin);
    if let Some(pool) = game.observer_pool(picker) {
        pools |= 1 << pool;
    }
    let respawn = if kind == StrategicItemKind::Mega {
        None
    } else {
        game.entities[item]
            .classname()
            .and_then(|classname| game.respawn_delay_of(classname))
    };
    game.oracle.note(EvidenceEvent {
        pools,
        at,
        kind: EvidenceEventKind::ItemTaken {
            item: item.0,
            kind,
            picker: picker.0,
            respawn,
        },
    });
}

/// Any witnessed pickup changes what the team knows about this player, including a weapons-stay
/// pickup where the map entity never disappears. Its concrete effects remain in the opponent model;
/// this event supplies the freshness barrier for already-issued decisions.
pub(crate) fn note_player_pickup(game: &mut GameState, player: EntId, at: f32) {
    let mut pools = game.evidence_pools(game.entities[player].v.origin);
    if let Some(pool) = game.observer_pool(player) {
        pools |= 1 << pool;
    }
    game.oracle.note(EvidenceEvent {
        pools,
        at,
        kind: EvidenceEventKind::PlayerChanged { player: player.0 },
    });
}

pub(crate) fn note_weapon_fire(game: &mut GameState, player: EntId, weapon: Weapon, pools: u16, at: f32) {
    game.oracle.note(EvidenceEvent {
        pools,
        at,
        kind: EvidenceEventKind::WeaponFired {
            player: player.0,
            weapon,
        },
    });
}

pub(crate) fn note_damage(game: &mut GameState, attacker: EntId, target: EntId, amount: f32) {
    let at = game.time();
    let graph = game.nav.graph.clone();
    let attacker_cell = graph
        .as_ref()
        .and_then(|graph| graph.nearest(game.entities[attacker].v.origin));
    game.oracle
        .note_damage_outcome(attacker, target, attacker_cell, graph.as_deref(), at);
    let mut pools = 0;
    if let Some(pool) = game.observer_pool(attacker) {
        pools |= 1 << pool;
    }
    if let Some(pool) = game.observer_pool(target) {
        pools |= 1 << pool;
    }
    game.oracle.note(EvidenceEvent {
        pools,
        at,
        kind: EvidenceEventKind::Damage {
            attacker: attacker.0,
            target: target.0,
            amount,
        },
    });
}

pub(crate) fn note_death(game: &mut GameState, player: EntId, at: f32) {
    game.oracle.note(EvidenceEvent {
        pools: (1 << EVIDENCE_POOLS) - 1,
        at,
        kind: EvidenceEventKind::Death { player: player.0 },
    });
}

fn worker_loop(mailbox: Arc<Mailbox>) {
    // The exchange type is deliberately backend-neutral: a future learned sequence model consumes
    // the same honest snapshots and emits the same timestamped plans.
    let mut backend: Box<dyn OracleBackend> = Box::new(DeterministicBackend::default());
    loop {
        let snapshot = {
            let mut state = lock(&mailbox.state);
            while !state.stop && state.input.is_none() {
                state = mailbox
                    .wake
                    .wait(state)
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
            }
            if state.stop {
                return;
            }
            state.input.take()
        };
        let Some(snapshot) = snapshot else { continue };
        if let Some(plan) = backend.update(snapshot) {
            lock(&mailbox.state).output = Some(plan);
        }
    }
}

trait OracleBackend: Send {
    fn update(&mut self, snapshot: OracleSnapshot) -> Option<OraclePlan>;
}

#[derive(Default)]
struct TeamMemory {
    item_spawn_at: HashMap<u32, f32>,
    item_evidence_at: HashMap<u32, f32>,
    ammo_spent: HashMap<(u32, AmmoChannel), u16>,
}

#[derive(Default)]
struct DeterministicBackend {
    epoch: OracleEpoch,
    generation: u64,
    last_plan_at: f32,
    teams: HashMap<u8, TeamMemory>,
}

impl DeterministicBackend {
    fn update_deterministic(&mut self, snapshot: OracleSnapshot) -> Option<OraclePlan> {
        if self.epoch != snapshot.epoch {
            self.epoch = snapshot.epoch;
            self.generation = 0;
            self.last_plan_at = f32::NEG_INFINITY;
            self.teams.clear();
        }
        for event in snapshot.events.iter().copied() {
            self.observe(event);
        }
        if snapshot.at - self.last_plan_at < PLAN_INTERVAL {
            return None;
        }
        self.last_plan_at = snapshot.at;
        self.generation = self.generation.wrapping_add(1).max(1);
        let teams = snapshot
            .teams
            .iter()
            .map(|team| self.plan_team(&snapshot, team))
            .collect();
        Some(OraclePlan {
            epoch: snapshot.epoch,
            generation: self.generation,
            at: snapshot.at,
            teams,
        })
    }

    fn observe(&mut self, event: EvidenceEvent) {
        for team in 1..=8u8 {
            if event.pools & (1 << team) == 0 {
                continue;
            }
            let memory = self.teams.entry(team).or_default();
            match event.kind {
                EvidenceEventKind::ItemTaken {
                    item,
                    kind,
                    picker,
                    respawn,
                } => {
                    if let Some(delay) = respawn {
                        memory.item_spawn_at.insert(item, event.at + delay);
                    }
                    memory.item_evidence_at.insert(item, event.at);
                    if let StrategicItemKind::Weapon { ammo, .. } | StrategicItemKind::Ammo(ammo) = kind {
                        memory.ammo_spent.remove(&(picker, ammo));
                    }
                }
                EvidenceEventKind::WeaponFired { player, weapon } => {
                    if let Some(ammo) = weapon_ammo_channel(weapon) {
                        *memory.ammo_spent.entry((player, ammo)).or_default() += 1;
                    }
                }
                EvidenceEventKind::Damage {
                    attacker,
                    target,
                    amount,
                } => {
                    let _ = (attacker, target, amount);
                }
                EvidenceEventKind::PlayerChanged { .. } => {}
                EvidenceEventKind::Death { player } => {
                    memory.ammo_spent.retain(|(p, _), _| *p != player);
                }
            }
        }
    }

    fn plan_team(&self, snapshot: &OracleSnapshot, team: &TeamSnapshot) -> TeamPlan {
        let memory = self.teams.get(&team.team);
        let alive: Vec<&MemberSnapshot> = team.members.iter().filter(|m| m.alive).collect();
        let weak = alive.iter().filter(|m| !m.armed() || m.recovering).count();
        let control = if alive.len() >= 2 && weak >= 2 {
            ControlState::Reset
        } else if major_due(&snapshot.items, memory, snapshot.at).is_some() {
            ControlState::Prepare
        } else {
            ControlState::Hold
        };
        let mut nuggets = Vec::new();
        if control == ControlState::Reset {
            assign_rearm(snapshot, team, memory, self.generation, &mut nuggets);
        } else if let Some(item) = major_due(&snapshot.items, memory, snapshot.at) {
            assign_major(snapshot, team, item, memory, self.generation, &mut nuggets);
        }
        if control != ControlState::Reset {
            if let Some(intercept) = best_intercept(snapshot, team, memory, self.generation, &nuggets) {
                nuggets.push(intercept);
            }
        }
        TeamPlan {
            team: team.team,
            mode: team.mode,
            control,
            nuggets,
        }
    }
}

impl OracleBackend for DeterministicBackend {
    fn update(&mut self, snapshot: OracleSnapshot) -> Option<OraclePlan> {
        self.update_deterministic(snapshot)
    }
}

fn assign_rearm(
    snapshot: &OracleSnapshot,
    team: &TeamSnapshot,
    memory: Option<&TeamMemory>,
    generation: u64,
    out: &mut Vec<OracleNugget>,
) {
    let mut used = Vec::new();
    let mut members: Vec<&MemberSnapshot> = team.members.iter().filter(|m| m.alive && !m.armed()).collect();
    members.sort_by_key(|m| m.ent);
    for member in members {
        let pick = snapshot
            .items
            .iter()
            .filter(|item| item.kind.is_strong_weapon() && !used.contains(&item.ent))
            .filter(|item| item_available(item.ent, memory, snapshot.at))
            .filter_map(|item| travel_cost(&snapshot.graph, member.cell, item.cell).map(|cost| (item, cost)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        let Some((item, _)) = pick else { continue };
        used.push(item.ent);
        out.push(nugget(
            snapshot,
            generation,
            team.team,
            member.ent,
            NuggetKind::Rearm,
            item.cell,
            item.ent,
            0.95,
            4.0,
            memory
                .and_then(|memory| memory.item_evidence_at.get(&item.ent))
                .copied()
                .unwrap_or(0.0),
        ));
    }
    if out.len() >= 2 {
        let rendezvous = out[0].target_cell;
        let recipient = out[1].recipient;
        out.push(nugget(
            snapshot,
            generation,
            team.team,
            recipient,
            NuggetKind::Regroup,
            rendezvous,
            out[0].recipient,
            0.8,
            4.0,
            snapshot.at,
        ));
    }
}

fn assign_major(
    snapshot: &OracleSnapshot,
    team: &TeamSnapshot,
    item: &OracleItem,
    memory: Option<&TeamMemory>,
    generation: u64,
    out: &mut Vec<OracleNugget>,
) {
    let mut candidates: Vec<(&MemberSnapshot, f32)> = team
        .members
        .iter()
        .filter(|m| m.alive)
        .filter_map(|member| {
            let travel = travel_cost(&snapshot.graph, member.cell, item.cell)?;
            let need = member_item_need(member, item.kind);
            Some((member, travel - need * 0.01))
        })
        .collect();
    candidates.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.ent.cmp(&b.0.ent)));
    let Some((owner, _)) = candidates.first().copied() else {
        return;
    };
    out.push(nugget(
        snapshot,
        generation,
        team.team,
        owner.ent,
        NuggetKind::PrepareItem,
        item.cell,
        item.ent,
        0.9,
        3.0,
        memory
            .and_then(|memory| memory.item_evidence_at.get(&item.ent))
            .copied()
            .unwrap_or(0.0),
    ));
    if let Some((cover, _)) = candidates.iter().copied().find(|(member, _)| member.ent != owner.ent) {
        let cover_cell = cover_cell(&snapshot.graph, item.cell).unwrap_or(item.cell);
        out.push(nugget(
            snapshot,
            generation,
            team.team,
            cover.ent,
            NuggetKind::CoverArea,
            cover_cell,
            item.ent,
            0.75,
            3.0,
            memory
                .and_then(|memory| memory.item_evidence_at.get(&item.ent))
                .copied()
                .unwrap_or(0.0),
        ));
    }
}

#[derive(Clone, Debug)]
struct DestinationHypothesis {
    target: CellId,
    family: u8,
    weight: f32,
    probability: f32,
}

#[derive(Clone, Debug)]
struct RouteHypothesis {
    links: Vec<u32>,
    probability: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct InterceptAggregate {
    target: CellId,
    mass: f32,
    weighted_margin: f32,
}

fn best_intercept(
    snapshot: &OracleSnapshot,
    team: &TeamSnapshot,
    memory: Option<&TeamMemory>,
    generation: u64,
    reserved: &[OracleNugget],
) -> Option<OracleNugget> {
    let mut best: Option<(f32, OracleNugget)> = None;
    for enemy in &team.enemies {
        let Some(cue) = enemy.cue else { continue };
        let age = (snapshot.at - cue.at).max(0.0);
        let cue_confidence = cue.confidence * (-age / 6.0).exp();
        if cue_confidence < INTERCEPT_CONFIDENCE {
            continue;
        }
        let destinations = destination_hypotheses(snapshot, enemy, cue, memory);
        let mut crossings: HashMap<(u32, u32, u32), InterceptAggregate> = HashMap::new();
        for destination in destinations {
            for route in route_hypotheses(&snapshot.graph, cue.cell, destination.target) {
                let path_mass = destination.probability * route.probability;
                let mut enemy_eta = 0.0;
                for link in route.links {
                    enemy_eta += snapshot.graph.link_cost(link);
                    let from = snapshot.graph.link_source(link);
                    let cell = snapshot.graph.link_target(link);
                    let (Some(from_cluster), Some(to_cluster)) =
                        (snapshot.graph.cluster_of(from), snapshot.graph.cluster_of(cell))
                    else {
                        continue;
                    };
                    if from_cluster == to_cluster {
                        continue;
                    }
                    for member in team
                        .members
                        .iter()
                        .filter(|member| member.alive && !reserved.iter().any(|nugget| nugget.recipient == member.ent))
                    {
                        let Some(our_eta) = travel_cost(&snapshot.graph, member.cell, cell) else {
                            continue;
                        };
                        if our_eta + INTERCEPT_MARGIN > enemy_eta {
                            continue;
                        }
                        let entry = crossings.entry((member.ent, from_cluster, to_cluster)).or_default();
                        if entry.mass == 0.0 {
                            entry.target = cell;
                        }
                        entry.mass += path_mass;
                        entry.weighted_margin += path_mass * (enemy_eta - our_eta).min(3.0);
                    }
                }
            }
        }
        for ((recipient, _, _), crossing) in crossings {
            if crossing.mass < INTERCEPT_MIN_PATH_MASS {
                continue;
            }
            let confidence = cue_confidence * crossing.mass.min(1.0);
            let margin = crossing.weighted_margin / crossing.mass.max(f32::EPSILON);
            let score = confidence + margin * 0.08;
            let candidate = nugget(
                snapshot,
                generation,
                team.team,
                recipient,
                NuggetKind::Intercept,
                crossing.target,
                enemy.ent,
                confidence,
                2.5,
                enemy.evidence_at,
            );
            if best.as_ref().is_none_or(|(old, _)| score > *old) {
                best = Some((score, candidate));
            }
        }
    }
    best.map(|(_, nugget)| nugget)
}

fn destination_hypotheses(
    snapshot: &OracleSnapshot,
    enemy: &EnemySnapshot,
    cue: EnemyCue,
    memory: Option<&TeamMemory>,
) -> Vec<DestinationHypothesis> {
    let mut candidates: Vec<DestinationHypothesis> = snapshot
        .items
        .iter()
        .filter(|item| item_available(item.ent, memory, snapshot.at))
        .filter_map(|item| {
            let need = enemy_item_need(enemy, item.kind, memory);
            if need <= 0.0 {
                return None;
            }
            // Ranking all map items with A* would make an otherwise slow 1 Hz thought expensive.
            // Euclidean time only ranks the shortlist; actual routes and ETAs are solved below.
            let direct_eta = (snapshot.graph.cell_origin(item.cell) - snapshot.graph.cell_origin(cue.cell)).length()
                / crate::navmesh::MAX_SPEED;
            Some(DestinationHypothesis {
                target: item.cell,
                family: strategic_family(item.kind),
                weight: need / (1.0 + direct_eta * 0.45),
                probability: 0.0,
            })
        })
        .collect();
    candidates.sort_by(|a, b| b.weight.total_cmp(&a.weight).then_with(|| a.target.cmp(&b.target)));
    let mut family_counts = [0usize; 5];
    candidates.retain(|candidate| {
        let count = &mut family_counts[candidate.family as usize];
        *count += 1;
        *count <= INTERCEPT_FAMILY_LIMIT
    });
    candidates.truncate(INTERCEPT_DESTINATIONS);
    normalize_destination_probabilities(&mut candidates);
    candidates
}

fn normalize_destination_probabilities(destinations: &mut [DestinationHypothesis]) {
    let total: f32 = destinations.iter().map(|destination| destination.weight.max(0.0)).sum();
    if total <= f32::EPSILON {
        return;
    }
    for destination in destinations {
        destination.probability = destination.weight.max(0.0) / total;
    }
}

fn strategic_family(kind: StrategicItemKind) -> u8 {
    match kind {
        StrategicItemKind::Health | StrategicItemKind::Mega => 0,
        StrategicItemKind::GreenArmor | StrategicItemKind::YellowArmor | StrategicItemKind::RedArmor => 1,
        StrategicItemKind::Weapon { .. } => 2,
        StrategicItemKind::Ammo(_) => 3,
        StrategicItemKind::Quad | StrategicItemKind::OtherPowerup => 4,
    }
}

fn route_hypotheses(graph: &NavGraph, start: CellId, target: CellId) -> Vec<RouteHypothesis> {
    let Some(primary) = graph.find_path(start, target, &LinkCosts::default()) else {
        return Vec::new();
    };
    if primary.is_empty() {
        return Vec::new();
    }
    let primary_cost = route_cost(graph, &primary);
    let transitions = cluster_transitions(graph, &primary);
    let penalties: Vec<(u32, f32)> = graph
        .links
        .iter()
        .enumerate()
        .filter_map(|(index, link)| {
            let transition = (graph.cluster_of(link.from)?, graph.cluster_of(link.to)?);
            (transition.0 != transition.1 && transitions.contains(&transition))
                .then_some((index as u32, INTERCEPT_ALT_PENALTY))
        })
        .collect();
    let alternative = (!penalties.is_empty())
        .then(|| {
            graph.find_path(
                start,
                target,
                &LinkCosts {
                    penalties: &penalties,
                    ..Default::default()
                },
            )
        })
        .flatten()
        .filter(|route| {
            !route.is_empty()
                && cluster_transitions(graph, route) != transitions
                && route_cost(graph, route) <= primary_cost * INTERCEPT_ALT_MAX_RATIO
        });
    let Some(alternative) = alternative else {
        return vec![RouteHypothesis {
            links: primary,
            probability: 1.0,
        }];
    };
    let (primary_probability, alternative_probability) =
        alternative_route_probabilities(primary_cost, route_cost(graph, &alternative));
    vec![
        RouteHypothesis {
            links: primary,
            probability: primary_probability,
        },
        RouteHypothesis {
            links: alternative,
            probability: alternative_probability,
        },
    ]
}

fn cluster_transitions(graph: &NavGraph, route: &[u32]) -> Vec<(u32, u32)> {
    route
        .iter()
        .filter_map(|&link| {
            let from = graph.cluster_of(graph.link_source(link))?;
            let to = graph.cluster_of(graph.link_target(link))?;
            (from != to).then_some((from, to))
        })
        .collect()
}

fn alternative_route_probabilities(primary_cost: f32, alternative_cost: f32) -> (f32, f32) {
    let alternative_weight = (-(alternative_cost - primary_cost).max(0.0) / 2.0).exp() * 0.45;
    let total = 1.0 + alternative_weight;
    (1.0 / total, alternative_weight / total)
}

fn route_cost(graph: &NavGraph, route: &[u32]) -> f32 {
    route.iter().map(|&link| graph.link_cost(link)).sum()
}

fn major_due<'a>(items: &'a [OracleItem], memory: Option<&TeamMemory>, now: f32) -> Option<&'a OracleItem> {
    items
        .iter()
        .filter(|item| item.kind.is_major())
        // No first-cycle guess: without an observed pickup there is no honest timer, and treating
        // every map item as "due now" recreates premature Quad/RA camping.
        .filter(|item| {
            memory
                .and_then(|m| m.item_spawn_at.get(&item.ent))
                .is_some_and(|&spawn| (0.0..=8.0).contains(&(spawn - now)))
        })
        .min_by_key(|item| match item.kind {
            StrategicItemKind::Quad => 0,
            StrategicItemKind::RedArmor => 1,
            StrategicItemKind::Mega => 2,
            _ => 3,
        })
}

fn item_available(item: u32, memory: Option<&TeamMemory>, now: f32) -> bool {
    memory.and_then(|m| m.item_spawn_at.get(&item)).copied().unwrap_or(now) - now <= 8.0
}

fn member_item_need(member: &MemberSnapshot, kind: StrategicItemKind) -> f32 {
    match kind {
        StrategicItemKind::Health => (100.0 - member.health).max(0.0),
        StrategicItemKind::Mega => (250.0 - member.health).max(0.0),
        StrategicItemKind::GreenArmor => (100.0 - member.armor).max(0.0) * 0.3,
        StrategicItemKind::YellowArmor => (150.0 - member.armor).max(0.0) * 0.6,
        StrategicItemKind::RedArmor => (200.0 - member.armor).max(0.0) * 0.8,
        StrategicItemKind::Weapon { bit, ammo } => {
            if !member.owns(bit) {
                140.0
            } else {
                (20.0 - member.ammo.channel(ammo)).max(0.0)
            }
        }
        StrategicItemKind::Ammo(ammo) => (20.0 - member.ammo.channel(ammo)).max(0.0),
        StrategicItemKind::Quad | StrategicItemKind::OtherPowerup => 200.0,
    }
}

fn enemy_item_need(enemy: &EnemySnapshot, kind: StrategicItemKind, memory: Option<&TeamMemory>) -> f32 {
    let health = enemy.health.unwrap_or(100.0);
    let armor = enemy.armor.unwrap_or(0.0);
    let items = enemy.items.unwrap_or(0);
    match kind {
        StrategicItemKind::Health => (100.0 - health).max(0.0),
        StrategicItemKind::Mega => (250.0 - health).max(0.0) + 20.0,
        StrategicItemKind::GreenArmor => (100.0 - armor).max(0.0) * 0.3,
        StrategicItemKind::YellowArmor => (150.0 - armor).max(0.0) * 0.6,
        StrategicItemKind::RedArmor => (200.0 - armor).max(0.0) * 0.8 + 20.0,
        StrategicItemKind::Weapon { bit, ammo } => {
            if items & bit == 0 {
                160.0
            } else if memory
                .and_then(|m| m.ammo_spent.get(&(enemy.ent, ammo)))
                .copied()
                .unwrap_or(0)
                >= 5
            {
                80.0
            } else {
                5.0
            }
        }
        StrategicItemKind::Ammo(ammo) => {
            if memory
                .and_then(|m| m.ammo_spent.get(&(enemy.ent, ammo)))
                .copied()
                .unwrap_or(0)
                >= 5
            {
                70.0
            } else {
                2.0
            }
        }
        StrategicItemKind::Quad | StrategicItemKind::OtherPowerup => 220.0,
    }
}

fn cover_cell(graph: &NavGraph, item: CellId) -> Option<CellId> {
    let cluster = graph.cluster_of(item)?;
    graph
        .links
        .iter()
        .filter(|link| graph.cluster_of(link.from) != Some(cluster) && graph.cluster_of(link.to) == Some(cluster))
        .map(|link| link.from)
        .min_by(|&a, &b| {
            let da = (graph.cell_origin(a) - graph.cell_origin(item)).length_squared();
            let db = (graph.cell_origin(b) - graph.cell_origin(item)).length_squared();
            da.total_cmp(&db)
        })
}

fn travel_cost(graph: &NavGraph, from: CellId, to: CellId) -> Option<f32> {
    graph
        .find_path(from, to, &LinkCosts::default())
        .map(|route| route.into_iter().map(|link| graph.link_cost(link)).sum())
}

fn nugget(
    snapshot: &OracleSnapshot,
    generation: u64,
    team: u8,
    recipient: u32,
    kind: NuggetKind,
    target_cell: CellId,
    subject: u32,
    confidence: f32,
    ttl: f32,
    evidence_at: f32,
) -> OracleNugget {
    OracleNugget {
        epoch: snapshot.epoch,
        generation,
        team,
        recipient,
        kind,
        target_cell,
        subject,
        confidence,
        decision_at: snapshot.at,
        evidence_at,
        expires_at: snapshot.at + ttl,
    }
}

fn weapon_ammo_channel(weapon: Weapon) -> Option<AmmoChannel> {
    match weapon {
        w if w == Weapon::Shotgun || w == Weapon::SuperShotgun => Some(AmmoChannel::Shells),
        w if w == Weapon::Nailgun || w == Weapon::SuperNailgun => Some(AmmoChannel::Nails),
        w if w == Weapon::GrenadeLauncher || w == Weapon::RocketLauncher => Some(AmmoChannel::Rockets),
        w if w == Weapon::Lightning => Some(AmmoChannel::Cells),
        _ => None,
    }
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn set_revision(revisions: &mut [f32; MAX_EDICTS], subject: u32, at: f32) {
    if let Some(revision) = revisions.get_mut(subject as usize) {
        *revision = revision.max(at);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inbox_replaces_kind_then_evicts_earliest_expiry() {
        let mut inbox = OracleInbox::default();
        for (index, kind) in [
            NuggetKind::Rearm,
            NuggetKind::Regroup,
            NuggetKind::PrepareItem,
            NuggetKind::CoverArea,
        ]
        .into_iter()
        .enumerate()
        {
            inbox.push(OracleNugget {
                epoch: 1,
                generation: 1,
                team: 1,
                recipient: 1,
                kind,
                target_cell: index as u32,
                subject: 0,
                confidence: 1.0,
                decision_at: 0.0,
                evidence_at: 0.0,
                expires_at: 1.0 + index as f32,
            });
        }
        let base = inbox.entries().next().unwrap();
        inbox.push(OracleNugget {
            kind: NuggetKind::Regroup,
            target_cell: 99,
            expires_at: 9.0,
            ..base
        });
        assert!(
            inbox
                .entries()
                .any(|n| n.kind == NuggetKind::Regroup && n.target_cell == 99)
        );
        let base = inbox.entries().next().unwrap();
        inbox.push(OracleNugget {
            kind: NuggetKind::Intercept,
            target_cell: 100,
            expires_at: 10.0,
            ..base
        });
        assert!(!inbox.entries().any(|n| n.kind == NuggetKind::Rearm));
    }

    #[test]
    fn newer_subject_evidence_cancels_a_hint() {
        let mut inbox = OracleInbox::default();
        inbox.push(OracleNugget {
            epoch: 4,
            generation: 2,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Intercept,
            target_cell: 9,
            subject: 3,
            confidence: 0.8,
            decision_at: 20.0,
            evidence_at: 18.0,
            expires_at: 24.0,
        });
        let mut revisions = [0.0; MAX_EDICTS];
        revisions[3] = 19.0;
        let _ = inbox.retain_live(4, 20.1, &revisions);
        assert!(inbox.best(20.1).is_none());
    }

    #[test]
    fn equal_evidence_time_keeps_hint_but_later_time_cancels_it() {
        let nugget = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 2,
            recipient: 2,
            kind: NuggetKind::Intercept,
            target_cell: 7,
            subject: 4,
            confidence: 0.8,
            decision_at: 12.0,
            evidence_at: 10.0,
            expires_at: 15.0,
        };
        let mut inbox = OracleInbox::default();
        inbox.push(nugget);
        let mut revisions = [0.0; MAX_EDICTS];
        revisions[4] = 10.0;
        let _ = inbox.retain_live(1, 12.1, &revisions);
        assert!(inbox.best(12.1).is_some());
        revisions[4] = 10.01;
        let _ = inbox.retain_live(1, 12.2, &revisions);
        assert!(inbox.best(12.2).is_none());
    }

    #[test]
    fn regroup_outcome_subject_is_not_a_freshness_dependency() {
        let nugget = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Regroup,
            target_cell: 7,
            subject: 3,
            confidence: 0.8,
            decision_at: 12.0,
            evidence_at: 10.0,
            expires_at: 15.0,
        };
        let mut inbox = OracleInbox::default();
        inbox.push(nugget);
        inbox.mark_applied(nugget);
        let mut revisions = [0.0; MAX_EDICTS];
        revisions[3] = 14.0;
        assert!(inbox.retain_live(1, 12.1, &revisions).is_none());
        assert!(inbox.best(12.1).is_some());
    }

    #[test]
    fn major_preparation_requires_an_observed_timer() {
        let items = [OracleItem {
            ent: 40,
            cell: 3,
            kind: StrategicItemKind::Quad,
        }];
        assert!(major_due(&items, None, 2.0).is_none());
        let mut memory = TeamMemory::default();
        memory.item_spawn_at.insert(40, 10.0);
        assert!(major_due(&items, Some(&memory), 2.0).is_some());
        assert!(major_due(&items, Some(&memory), 11.0).is_none());
    }

    #[test]
    fn evaluator_separates_treated_and_holdout_success() {
        let base = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Rearm,
            target_cell: 7,
            subject: 40,
            confidence: 0.8,
            decision_at: 10.0,
            evidence_at: 9.0,
            expires_at: 14.0,
        };
        let mut oracle = OracleRuntime::default();
        oracle.set_evaluation(true);
        for withheld in [false, true] {
            oracle.record_trial(OracleTrial {
                nugget: OracleNugget {
                    recipient: if withheld { 2 } else { 1 },
                    ..base
                },
                episode: 0,
                withheld,
                issued_at: 10.0,
                applied_at: None,
                outcome: TrialOutcome::Pending,
                outcome_at: 0.0,
            });
        }
        oracle.mark_applied(base, 10.1);
        oracle.note_item_outcome(EntId(40), EntId(1), 1, 11.0);
        let summary = oracle.eval_summary();
        assert_eq!((summary.treated, summary.applied, summary.treated_success), (1, 1, 1));
        assert_eq!((summary.controls, summary.control_success), (1, 0));
        oracle.bump_epoch();
        assert_eq!(oracle.eval_summary().treated, 0);
        assert_eq!(oracle.eval_summary().controls, 0);
    }

    #[test]
    fn evaluator_collapses_correlated_trials_into_strategic_episodes() {
        let base = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Intercept,
            target_cell: 7,
            subject: 3,
            confidence: 0.8,
            decision_at: 10.0,
            evidence_at: 9.0,
            expires_at: 14.0,
        };
        let mut oracle = OracleRuntime::default();
        oracle.set_evaluation(true);
        for (subject, outcome) in [(3, TrialOutcome::Invalidated), (4, TrialOutcome::Success)] {
            oracle.record_trial(OracleTrial {
                nugget: OracleNugget { subject, ..base },
                episode: 2,
                withheld: false,
                issued_at: 10.0,
                applied_at: Some(10.1),
                outcome,
                outcome_at: 11.0,
            });
        }
        oracle.record_trial(OracleTrial {
            nugget: OracleNugget {
                generation: 2,
                team: 2,
                recipient: 3,
                ..base
            },
            episode: 2,
            withheld: true,
            issued_at: 10.0,
            applied_at: None,
            outcome: TrialOutcome::Missed,
            outcome_at: 14.0,
        });

        let trials = oracle.eval_summary_for(NuggetKind::Intercept);
        assert_eq!((trials.treated, trials.treated_success), (2, 1));
        let episodes = oracle.eval_episode_summary_for(NuggetKind::Intercept);
        assert_eq!((episodes.treated, episodes.treated_success), (1, 1));
        assert_eq!((episodes.controls, episodes.control_success), (1, 0));
        assert_eq!((episodes.applied, episodes.invalidated, episodes.pending), (1, 0, 0));
    }

    #[test]
    fn evaluator_freezes_pending_trials_at_the_match_boundary() {
        let nugget = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Rearm,
            target_cell: 7,
            subject: 40,
            confidence: 0.8,
            decision_at: 10.0,
            evidence_at: 9.0,
            expires_at: 14.0,
        };
        let mut oracle = OracleRuntime::default();
        oracle.set_evaluation(true);
        oracle.record_trial(OracleTrial {
            nugget,
            episode: 0,
            withheld: false,
            issued_at: 10.0,
            applied_at: Some(10.1),
            outcome: TrialOutcome::Pending,
            outcome_at: 0.0,
        });

        oracle.close_pending_trials(11.0);
        oracle.note_item_outcome(EntId(40), EntId(1), 1, 11.5);

        let summary = oracle.eval_summary();
        assert_eq!((summary.treated, summary.treated_success, summary.pending), (1, 0, 0));
    }

    #[test]
    fn holdout_choice_is_stable_for_a_whole_episode() {
        let a = plan_holdout(7, 2, 30.1, 0.5);
        let b = plan_holdout(7, 2, 44.9, 0.5);
        assert_eq!(a, b);
    }

    #[test]
    fn low_confidence_enemy_never_produces_intercept() {
        let confidence = 0.5 * (-0.0f32 / 6.0).exp();
        assert!(confidence < INTERCEPT_CONFIDENCE);
    }

    #[test]
    fn identical_plan_refreshes_one_acknowledged_instruction() {
        let mut inbox = OracleInbox::default();
        let first = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Rearm,
            target_cell: 7,
            subject: 40,
            confidence: 0.9,
            decision_at: 1.0,
            evidence_at: 0.5,
            expires_at: 5.0,
        };
        assert_eq!(inbox.push(first), InboxUpdate::Communicated);
        inbox.mark_applied(first);
        let refresh = OracleNugget {
            generation: 2,
            decision_at: 2.0,
            expires_at: 6.0,
            ..first
        };
        assert_eq!(inbox.push(refresh), InboxUpdate::Refreshed);
        assert_eq!(inbox.active.unwrap().generation, 2);
        assert!(inbox.retain_live(1, 2.1, &[0.0; MAX_EDICTS]).is_none());
    }

    #[test]
    fn rejected_identical_call_observes_cooldown_but_changed_action_bypasses_it() {
        let mut inbox = OracleInbox::default();
        let first = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Intercept,
            target_cell: 7,
            subject: 3,
            confidence: 0.8,
            decision_at: 1.0,
            evidence_at: 0.5,
            expires_at: 3.0,
        };
        assert_eq!(inbox.push(first), InboxUpdate::Communicated);
        inbox.discard(first, 1.1);
        let repeated = OracleNugget {
            generation: 2,
            decision_at: 2.0,
            expires_at: 4.0,
            ..first
        };
        assert_eq!(inbox.push(repeated), InboxUpdate::Suppressed);
        let observed_again = OracleNugget {
            evidence_at: 1.5,
            ..repeated
        };
        assert_eq!(inbox.push(observed_again), InboxUpdate::Suppressed);
        let changed_crossing = OracleNugget {
            target_cell: 8,
            ..observed_again
        };
        assert_eq!(inbox.push(changed_crossing), InboxUpdate::Communicated);
    }

    #[test]
    fn stale_evidence_can_resume_the_same_revalidated_instruction_silently() {
        let mut inbox = OracleInbox::default();
        let first = OracleNugget {
            epoch: 1,
            generation: 1,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Intercept,
            target_cell: 7,
            subject: 3,
            confidence: 0.8,
            decision_at: 1.0,
            evidence_at: 0.5,
            expires_at: 3.0,
        };
        assert_eq!(inbox.push(first), InboxUpdate::Communicated);
        let mut revisions = [0.0; MAX_EDICTS];
        revisions[3] = 1.5;
        let _ = inbox.retain_live(1, 1.6, &revisions);
        let confirmed = OracleNugget {
            generation: 2,
            decision_at: 2.0,
            evidence_at: 1.5,
            expires_at: 4.5,
            ..first
        };
        assert_eq!(inbox.push(confirmed), InboxUpdate::Refreshed);
        assert!(inbox.best(2.0).is_some());
    }

    #[test]
    fn experiment_arm_change_requests_an_inbox_clear() {
        let mut oracle = OracleRuntime::default();
        oracle.epoch = 9;
        let initial = oracle.arm(1, 10.0, 1.0);
        assert!(initial.withheld);
        let clears = oracle.advance_arms(10.1, 0.0);
        assert_eq!(clears, vec![1]);
        assert_eq!(oracle.communication_summary().arm_clears, 1);
    }

    #[test]
    fn destination_probabilities_preserve_relative_support() {
        let mut hypotheses = vec![
            DestinationHypothesis {
                target: 1,
                family: 0,
                weight: 6.0,
                probability: 0.0,
            },
            DestinationHypothesis {
                target: 2,
                family: 1,
                weight: 3.0,
                probability: 0.0,
            },
            DestinationHypothesis {
                target: 3,
                family: 2,
                weight: 1.0,
                probability: 0.0,
            },
        ];
        normalize_destination_probabilities(&mut hypotheses);
        assert!((hypotheses.iter().map(|h| h.probability).sum::<f32>() - 1.0).abs() < 1e-6);
        assert!((hypotheses[0].probability - 0.6).abs() < 1e-6);
        assert!(hypotheses[0].probability > hypotheses[1].probability);
    }

    #[test]
    fn alternative_route_probability_decays_with_detour_cost() {
        let near = alternative_route_probabilities(4.0, 4.2);
        let far = alternative_route_probabilities(4.0, 7.0);
        assert!((near.0 + near.1 - 1.0).abs() < 1e-6);
        assert!((far.0 + far.1 - 1.0).abs() < 1e-6);
        assert!(near.1 > far.1);
        assert!(near.0 > near.1);
    }
}
