// SPDX-License-Identifier: AGPL-3.0-or-later

//! Slow, observation-gated team reasoning on an owned worker thread.
//!
//! The engine thread publishes owned [`OracleSnapshot`] values and polls owned [`OraclePlan`]s. The
//! worker receives no host handle, entity reference, or mutable game state, and is joined before the
//! module unloads. Each team has an isolated evidence sheet; CTF is deliberately shadow-only.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

use crate::bot::power;
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
    Weapon {
        bit: u32,
        ammo: AmmoChannel,
    },
    Ammo(AmmoChannel),
    Quad,
    /// Pentagram of protection. Split from the ring because pricing them alike is a real error:
    /// taking a pentagram is the biggest single event on the board (+1.55 forward team frags), while
    /// a ring of shadows barely moves a fighter's power (×1.10) and prices at nothing.
    Pent,
    /// Ring of shadows, and any other artifact worth planning around this little.
    Ring,
}

impl StrategicItemKind {
    fn is_major(self) -> bool {
        // The ring is deliberately absent: a major is something worth planning a cycle around, and
        // the measurement says this one is not.
        matches!(self, Self::Mega | Self::RedArmor | Self::Quad | Self::Pent)
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
    /// The jacket's absorb fraction (0.3 / 0.6 / 0.8), needed to turn `armor` into effective health.
    /// Armor points alone say nothing about survivability without the health to spend them.
    pub armor_type: f32,
    pub items: u32,
    pub ammo: AmmoSnapshot,
    pub recovering: bool,
    /// This member's measured power — expected kills over the next minute as a multiple of a fresh
    /// spawn. Own-team data is truthful; see [`EnemySnapshot::power`] for the other side.
    pub power: f32,
    /// The material state `power` was computed from, kept so the worker can ask what a *candidate
    /// item* would add without reaching back into the entity it came from.
    pub power_input: power::PowerInput,
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
    /// Newest observation incorporated into this enemy belief.
    pub evidence_at: f32,
    pub cue: Option<EnemyCue>,
    /// Measured power of the *believed* loadout, or `None` where nothing is believed. Strictly a
    /// function of the observation-gated estimate — the honest-evidence rule holds here as
    /// everywhere else in the snapshot, so a team can be wrong about how strong the other side is
    /// in exactly the ways its bots have earned.
    pub power: Option<f32>,
    /// The believed material state `power` was computed from — `None` on the same terms. Lets the
    /// worker price what an item would be worth *to him* in the same currency it prices its own, from
    /// observation-gated fields only.
    pub believed: Option<power::PowerInput>,
    /// Whether the enemy is believed to be holding a quad right now.
    pub quad: bool,
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
    /// Our total measured power minus the believed enemy total — the one team-level number that
    /// predicts anything. One unit of gap converts to about one team frag over the next minute, and
    /// the *level* (both teams stacked, or both naked) predicts nothing at all, which is why this is
    /// stored as a difference rather than as two totals.
    ///
    /// Dead players on either side count as a fresh spawn, not as zero. An advantage held as bodies
    /// evaporates on respawn — measured at −1.0 frags per body at a fixed equipment gap — while an
    /// advantage held as equipment keeps paying. A team that has just aced the other side is not
    /// four power units ahead; it is ahead by whatever the survivors are carrying.
    pub power_gap: f32,
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
    /// The measured equipment gap this plan was made under, carried through for observability —
    /// reading it beside the control state is what makes the plan legible over the control channel.
    pub power_gap: f32,
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
        evidence_revision: &[f32],
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
    evidence_revision: [Vec<f32>; EVIDENCE_POOLS],
    last_plan: Option<OraclePlan>,
    evaluation: bool,
    trials: VecDeque<OracleTrial>,
    communication: CommunicationSummary,
    arms: [Option<ExperimentArm>; EVIDENCE_POOLS],
}

impl Default for OracleRuntime {
    fn default() -> Self {
        Self::with_entity_capacity(MAX_EDICTS)
    }
}

impl OracleRuntime {
    pub(crate) fn with_entity_capacity(entity_capacity: usize) -> Self {
        Self {
            worker: None,
            epoch: 0,
            next_publish: 0.0,
            next_debug: 0.0,
            pending_events: Vec::new(),
            evidence_revision: std::array::from_fn(|_| vec![0.0; entity_capacity]),
            last_plan: None,
            evaluation: false,
            trials: VecDeque::new(),
            communication: CommunicationSummary::default(),
            arms: [None; EVIDENCE_POOLS],
        }
    }

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
    let evaluation_live = evaluation && matches!(game.team_match.phase, crate::mode::MatchPhase::Live);
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
        // An untreated team gets no plan at all, so a split-team match compares "team play" against
        // "no team play" cleanly rather than leaving the control side half-advised.
        if !game.power_team_allows(bots[0]) {
            continue;
        }
        let mode = match game.mode.name() {
            "dm" => OracleMode::TeamDeathmatch,
            "ctf" => OracleMode::CtfShadow,
            _ => continue,
        };
        let members: Vec<MemberSnapshot> = bots
            .iter()
            .filter_map(|&e| member_snapshot(game, graph, e, now))
            .collect();
        let observer = bots[0];
        let enemies: Vec<EnemySnapshot> = players
            .iter()
            .copied()
            .filter(|&e| game.entities[e].mode_p.team != team)
            .filter_map(|enemy| enemy_snapshot(game, graph, &bots, observer, enemy, now))
            .collect();
        // Both sides count a corpse as the fresh spawn it is about to be, and an unbelieved enemy as
        // one too — which is exactly what the model's own baseline says about a player nobody has
        // seen since he died.
        let ours = power::team_power(members.iter().map(|m| m.alive.then_some(m.power)));
        let theirs = power::team_power(enemies.iter().map(|e| e.power));
        teams.push(TeamSnapshot {
            team,
            mode,
            members,
            enemies,
            power_gap: ours - theirs,
        });
    }
    teams
}

fn member_snapshot(game: &GameState, graph: &NavGraph, e: EntId, now: f32) -> Option<MemberSnapshot> {
    let ent = &game.entities[e];
    let cell = graph.nearest(ent.v.origin)?;
    let input = power::PowerInput {
        health: ent.v.health,
        armor_value: ent.v.armorvalue,
        armor_type: ent.v.armortype,
        items: ent.v.items,
        rockets: ent.v.ammo_rockets,
        cells: ent.v.ammo_cells,
        quad_age: power::powerup_age(ent.combat.super_damage_finished, now),
        pent_age: power::powerup_age(ent.combat.invincible_finished, now),
        ring: ent.v.items.has(Items::INVISIBILITY),
    };
    Some(MemberSnapshot {
        ent: e.0,
        cell,
        alive: ent.is_alive(),
        health: ent.v.health,
        armor: ent.v.armorvalue,
        armor_type: ent.v.armortype,
        items: Items::from_f32(ent.v.items).bits(),
        ammo: AmmoSnapshot {
            shells: ent.v.ammo_shells,
            nails: ent.v.ammo_nails,
            rockets: ent.v.ammo_rockets,
            cells: ent.v.ammo_cells,
        },
        recovering: ent.bot.posture == CombatPosture::Recover,
        power: power::power(&input),
        power_input: input,
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
    let believed = estimate.map(|e| crate::bot::model::believed_power_input(&e, now));
    Some(EnemySnapshot {
        ent: enemy.0,
        evidence_at: estimate
            .map(|e| e.last_update)
            .unwrap_or(0.0)
            .max(cue.map(|c| c.at).unwrap_or(0.0)),
        cue,
        // One constructor for the belief and its score, so the two can never drift apart.
        power: believed.map(|b| power::power(&b)),
        believed,
        quad: estimate.is_some_and(|e| e.quad_until > now),
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
        "item_artifact_invulnerability" => StrategicItemKind::Pent,
        c if c.starts_with("item_artifact_") => StrategicItemKind::Ring,
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
    /// Teams currently in [`ControlState::Reset`], so [`reset_by_gap`] can hold the state across the
    /// band where entering and leaving disagree. Without it a team hovering at the threshold would
    /// re-issue and withdraw rearm orders every second.
    resetting: std::collections::HashSet<u8>,
}

/// Whether the team should drop what it is doing and rebuild, from the measured power gap.
///
/// The threshold is readable rather than tuned, which is the point of putting the team on a measured
/// scale: one unit of power gap converts to about one team frag over the next minute, so
/// `ENTER` means "we are losing this at better than a frag a minute on equipment alone". Fighting
/// from there feeds the other side; the rebuild is what turns it around.
///
/// The old rule counted heads — two or more members unarmed or recovering — which fired on a team
/// that had just lost two players *and was winning*, and stayed quiet for a team whose four living
/// members were all a little behind.
fn reset_by_gap(power_gap: f32, was_resetting: bool) -> bool {
    const ENTER: f32 = -1.2;
    const EXIT: f32 = -0.9;
    if was_resetting {
        power_gap < EXIT
    } else {
        power_gap < ENTER
    }
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
        let teams: Vec<TeamPlan> = snapshot
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

    fn plan_team(&mut self, snapshot: &OracleSnapshot, team: &TeamSnapshot) -> TeamPlan {
        let was_resetting = self.resetting.contains(&team.team);
        let memory = self.teams.get(&team.team);
        let alive: Vec<&MemberSnapshot> = team.members.iter().filter(|m| m.alive).collect();
        let weak = alive.iter().filter(|m| !m.armed() || m.recovering).count();
        let control = if alive.len() >= 2 && weak >= 1 && reset_by_gap(team.power_gap, was_resetting) {
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
            // Hunting the quad carrier comes before the ordinary intercept: it is the single most
            // valuable kill available and it is on a timer, so the two nearest armed members get
            // sent while it is still worth having killed him.
            assign_quad_hunt(snapshot, team, self.generation, &mut nuggets);
            if let Some(intercept) = best_intercept(snapshot, team, memory, self.generation, &nuggets) {
                nuggets.push(intercept);
            }
            // Only from a position of material parity: holding rooms while outgunned is how a team
            // feeds. Ahead or even, the armor cycle is where the next minute is decided.
            if team.power_gap >= 0.0 {
                assign_area_control(snapshot, team, memory, self.generation, &mut nuggets);
            }
        }
        if control == ControlState::Reset {
            self.resetting.insert(team.team);
        } else {
            self.resetting.remove(&team.team);
        }
        TeamPlan {
            team: team.team,
            mode: team.mode,
            control,
            power_gap: team.power_gap,
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
            Some((member, travel - need * NEED_TRAVEL_WEIGHT))
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
    // The escort is going to a contested room to fight for it, so it answers the same fitness bar as
    // area control — unlike the owner above, who is going to *take* an item and may well need it.
    if let Some((cover, _)) = candidates
        .iter()
        .copied()
        .find(|(member, _)| member.ent != owner.ent && fit_to_contest(member))
    {
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

/// How long an area-control order stands. Longer than the item orders around it because holding a
/// room *is* the task — a three-second cover order expires before the bot has finished walking there.
const COVER_TTL: f32 = 8.0;

/// How far ahead of an item's return the team starts taking its room. Control is future stack, and
/// the value is in being there when it lands, not in arriving to find someone else already gone.
///
/// This has to be read against the item's *own* clock, which is why it is a function and not the
/// single constant it started as. Both armors return every twenty seconds, so a twenty-five second
/// lead was satisfied from the instant the armor was taken until the instant it came back — for the
/// whole cycle, every cycle. "Head over shortly before it lands" quietly became "live here", and a
/// member was parked on the red armor all game with whatever health he happened to have. The lead
/// must stay well inside the respawn interval or there is never a moment when the order is off.
fn control_lead(kind: StrategicItemKind) -> f32 {
    match kind {
        // A sixty-second cycle leaves room to set up, and the quad room is worth arriving early for.
        StrategicItemKind::Quad => 15.0,
        // Twenty-second armor clock: long enough to cross a room, and no longer.
        _ => 8.0,
    }
}

/// The effective health below which a member is not sent *into* a fight. A fresh spawn is exactly
/// 100, so this asks only that a room-holder be no worse off than someone who just respawned —
/// which is a low bar, and still excludes the case that prompted it.
pub(crate) const CONTROL_MIN_EH: f32 = 100.0;

/// Whether a member is fit to be sent somewhere a fight is the expected outcome: holding a contested
/// room, or running down a quad carrier. Fetch orders deliberately do not ask — being hurt is a
/// reason to go and take something, not a reason to stay away from it.
///
/// The bar is [effective health](power::effective_health) rather than power. Power answers "how many
/// kills is this fighter worth over the next minute" and was fitted on humans who, at 19 health under
/// a full red jacket, would break off and heal; it scores that fighter at around one and a half fresh
/// spawns. Reading a sixty-second expectation as "fit to stand here and take a rocket" asks the
/// measurement for advice it never gave.
fn fit_to_contest(member: &MemberSnapshot) -> bool {
    member.alive
        && member.armed()
        && power::effective_health(member.health, member.armor, member.armor_type) >= CONTROL_MIN_EH
}

/// Send a member to hold the room around an armor or quad spawn that is about to come back.
///
/// This is the *control channel*, the part of the game the bot had no representation of at all.
/// Holding the red-armor area over the trailing thirty seconds swings the next minute by ±1.75 team
/// frags **beyond** whatever stacks are currently standing in it; the quad area is worth ±1.6, an
/// instantly-available yellow ±1.0. Weapon rooms measured at zero once stack is held fixed, which is
/// why [`power::control`] lists no weapon: on this map, controlling the rocket launcher's room buys
/// nothing you don't already have by holding the launcher.
///
/// Only witnessed timers are used — the same honest-evidence rule the rest of the planner keeps, and
/// the reason a team never camps a spawn it has no business knowing about.
fn assign_area_control(
    snapshot: &OracleSnapshot,
    team: &TeamSnapshot,
    memory: Option<&TeamMemory>,
    generation: u64,
    out: &mut Vec<OracleNugget>,
) {
    let Some(memory) = memory else { return };
    // The most valuable room whose item is genuinely on its way back.
    let due = snapshot
        .items
        .iter()
        .filter_map(|item| {
            let weight = control_weight(item.kind)?;
            let spawn_at = *memory.item_spawn_at.get(&item.ent)?;
            let wait = spawn_at - snapshot.at;
            (wait > 0.0 && wait <= control_lead(item.kind)).then_some((item, weight))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1).then_with(|| b.0.ent.cmp(&a.0.ent)));
    let Some((item, _)) = due else { return };
    // Whoever is free, fit, and nearest. A member already carrying an order keeps it: this is the
    // least urgent thing the planner asks for, and it must never displace a rearm or a pickup.
    //
    // The fitness filter is what stops the room-holder being whoever just took the armor and is
    // therefore both standing on it (travel zero, so always nearest) and freshly hurt from the fight
    // they took it after. That bot should be leaving to heal, not holding the most contested spot on
    // the map at a quarter of its effective health.
    let holder = team
        .members
        .iter()
        .filter(|m| fit_to_contest(m) && !out.iter().any(|n| n.recipient == m.ent))
        .filter_map(|m| Some((m, travel_cost(&snapshot.graph, m.cell, item.cell)?)))
        .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.ent.cmp(&b.0.ent)));
    let Some((holder, _)) = holder else { return };
    let cover_cell = cover_cell(&snapshot.graph, item.cell).unwrap_or(item.cell);
    out.push(nugget(
        snapshot,
        generation,
        team.team,
        holder.ent,
        NuggetKind::CoverArea,
        cover_cell,
        item.ent,
        0.7,
        COVER_TTL,
        memory.item_evidence_at.get(&item.ent).copied().unwrap_or(0.0),
    ));
}

/// The measured value of holding this item's room, or `None` for the rooms that measured at nothing.
fn control_weight(kind: StrategicItemKind) -> Option<f32> {
    match kind {
        StrategicItemKind::RedArmor => Some(power::control::RED_ARMOR),
        StrategicItemKind::Quad => Some(power::control::QUAD),
        StrategicItemKind::YellowArmor => Some(power::control::YELLOW_ARMOR),
        _ => None,
    }
}

/// How long after a witnessed quad pickup the hunt is still worth ordering. Quad is front-loaded:
/// what the carrier is going to do with it, he does early, so a kill in the first third is worth
/// most of the +0.92 the event prices and one in the last third is worth arriving for a fight
/// against a fighter who is merely slightly stronger than usual.
const QUAD_HUNT_WINDOW: f32 = 20.0;

/// Send the two nearest armed members after a believed quad carrier.
///
/// The estimate carried `quad_until` from the start and nothing consumed it, so a bot fought a
/// quadded enemy exactly as it fought a naked one. Killing a quad carrier is worth +0.92 forward
/// team frags — +1.61 if the pack is then secured, which the pack economy now handles — making it
/// the most valuable kill on the board, and the one with the shortest shelf life.
fn assign_quad_hunt(snapshot: &OracleSnapshot, team: &TeamSnapshot, generation: u64, out: &mut Vec<OracleNugget>) {
    let carrier = team
        .enemies
        .iter()
        .filter(|e| e.quad)
        .filter_map(|e| Some((e, e.cue?)))
        .filter(|(_, cue)| snapshot.at - cue.at <= QUAD_HUNT_WINDOW)
        .max_by(|a, b| a.1.at.total_cmp(&b.1.at));
    let Some((enemy, cue)) = carrier else { return };
    // Fit hunters only. Sending a fighter who dies to one quadded rocket at the fighter holding the
    // quad does not deny it — it feeds it, and hands over the pack that comes with the kill.
    let mut hunters: Vec<(&MemberSnapshot, f32)> = team
        .members
        .iter()
        .filter(|m| fit_to_contest(m) && !out.iter().any(|n| n.recipient == m.ent))
        .filter_map(|m| Some((m, travel_cost(&snapshot.graph, m.cell, cue.cell)?)))
        .collect();
    hunters.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.ent.cmp(&b.0.ent)));
    // Two, not the whole team: a quad carrier is worth converging on, but a team that abandons the
    // map to chase one has handed over everything else on it.
    for (hunter, _) in hunters.into_iter().take(2) {
        out.push(nugget(
            snapshot,
            generation,
            team.team,
            hunter.ent,
            NuggetKind::Intercept,
            cue.cell,
            enemy.ent,
            cue.confidence,
            3.0,
            enemy.evidence_at,
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
        StrategicItemKind::Quad | StrategicItemKind::Pent | StrategicItemKind::Ring => 4,
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

/// Weight on a powerup carrier's claim to a major, mirroring goal selection's
/// [`POWERUP_CARRIER_OWNER_BIAS`](super::goals::POWERUP_CARRIER_OWNER_BIAS). Both layers can hand out
/// the red armour, so both have to agree on who should get it — otherwise the planner sends the
/// carrier and the goal layer lets somebody else take it on the way, which is worse than either
/// policy alone.
const CARRIER_ITEM_NEED_BIAS: f32 = 3.0;

/// Seconds of extra travel a unit of need is worth when picking who fetches a major.
///
/// Recalibrated with the needs themselves: the old hand tables topped out around 220 at `0.01`, so a
/// missing weapon (140) pulled a member about 1.4 s further than the nearest. Δpower prices that same
/// missing launcher near 334, and this keeps the pull at the same ~1.3 s rather than silently
/// tripling it. Changing the currency without re-anchoring the exchange rate would have been a
/// behaviour change disguised as a refactor.
const NEED_TRAVEL_WEIGHT: f32 = 0.004;

/// What `kind` would add to `input`'s measured power — the oracle's half of the goal layer's
/// `item_power_gain`, over item *kinds* rather than live entities because the worker only ever sees
/// a snapshot.
///
/// Kind-level standard quantities are enough: the two health pickups that differ in any interesting
/// way are already separate kinds, and the spread inside a kind (a 20-rocket box versus a 5) is far
/// smaller than the error the hand-written tables this replaces were carrying.
fn kind_power_delta(input: &power::PowerInput, kind: StrategicItemKind) -> f32 {
    let before = power::power(input);
    let mut after = *input;
    match kind {
        StrategicItemKind::Health => {
            if input.health >= 100.0 {
                return 0.0;
            }
            after.health = (input.health + 25.0).min(100.0);
        }
        StrategicItemKind::Mega => {
            if input.health >= 250.0 {
                return 0.0;
            }
            after.health = (input.health + 100.0).min(250.0);
        }
        // Armour replaces rather than stacks, so a jacket is worth nothing to someone already in a
        // better one — the "don't send the red-armour holder for the green" rule as arithmetic
        // instead of a gate constant.
        StrategicItemKind::GreenArmor => replace_armor(&mut after, 100.0, 0.3),
        StrategicItemKind::YellowArmor => replace_armor(&mut after, 150.0, 0.6),
        StrategicItemKind::RedArmor => replace_armor(&mut after, 200.0, 0.8),
        StrategicItemKind::Weapon { bit, ammo } => {
            after.items = Items::from_f32(after.items)
                .union(Items::from_bits_truncate(bit))
                .as_f32();
            add_channel(&mut after, ammo, pickup_ammo(ammo));
        }
        StrategicItemKind::Ammo(ammo) => add_channel(&mut after, ammo, pickup_ammo(ammo)),
        StrategicItemKind::Quad => after.quad_age = Some(0.0),
        StrategicItemKind::Pent => after.pent_age = Some(0.0),
        StrategicItemKind::Ring => after.ring = true,
    }
    (power::power(&after) - before).max(0.0)
}

fn replace_armor(input: &mut power::PowerInput, value: f32, at: f32) {
    if input.armor_value >= value {
        return;
    }
    input.armor_value = value;
    input.armor_type = at;
}

/// What a standard box of `channel` carries. Shells and nails move no power factor at all (the fit
/// has no term for them), which is why they need the scarcity floor below rather than a delta.
fn pickup_ammo(channel: AmmoChannel) -> f32 {
    match channel {
        AmmoChannel::Rockets => 5.0,
        AmmoChannel::Cells => 6.0,
        AmmoChannel::Shells | AmmoChannel::Nails => 20.0,
    }
}

fn add_channel(input: &mut power::PowerInput, channel: AmmoChannel, amount: f32) {
    match channel {
        AmmoChannel::Rockets => input.rockets = (input.rockets + amount).min(100.0),
        AmmoChannel::Cells => input.cells = (input.cells + amount).min(100.0),
        // Neither is an input to the power score; see [`pickup_ammo`].
        AmmoChannel::Shells | AmmoChannel::Nails => {}
    }
}

/// The forward team-frag price of *taking* `kind`, mirroring the goal layer's `event_price` over the
/// kinds the oracle sees. This is the part of an item's worth that isn't marginal power: a red armour
/// is worth taking at a full stack because of who then doesn't have it.
fn kind_event_price(kind: StrategicItemKind) -> f32 {
    match kind {
        StrategicItemKind::Quad => power::price::QUAD,
        StrategicItemKind::Pent => power::price::PENT,
        StrategicItemKind::RedArmor => power::price::RED_ARMOR,
        StrategicItemKind::YellowArmor => power::price::YELLOW_ARMOR,
        StrategicItemKind::Mega => power::price::MEGA,
        // The ring is the one pickup the measurement says to walk past if something else wants doing.
        _ => 0.0,
    }
}

/// Floor under the ammo channels the power score cannot see.
///
/// Shells and nails carry no factor in the fit, so on Δpower alone they price at exactly zero and the
/// oracle would never send anyone for them again — a bot dry on both would stand there holding an
/// axe. This keeps them **ranked last and still visible**: a dry member rates about 20, against ~334
/// for a missing rocket launcher, so ammo is fetched when nothing that matters is wanted and not
/// before. The bot's own goal layer prices these properly and is untouched; this is the team
/// planner's fetch ranking only.
fn ammo_scarcity_floor(held: f32) -> f32 {
    (20.0 - held).max(0.0)
}

/// Whether a channel is invisible to the power score, and so needs the floor above.
fn unscored_channel(channel: AmmoChannel) -> bool {
    matches!(channel, AmmoChannel::Shells | AmmoChannel::Nails)
}

/// What `kind` is worth to a teammate, in the currency everything else prices in: `K·Δpower` plus the
/// event-price floor, weighted for a powerup carrier.
///
/// Replaces a hand-written table of marginal values. The table had to assert things the power curves
/// already know — that armour tops out, that a second jacket is worth less than the first, that a
/// weapon you already own is worth only its ammo — and it asserted them with different numbers than
/// the goal layer used, so the planner and the bot it was advising disagreed about what they were
/// doing.
fn member_item_need(member: &MemberSnapshot, kind: StrategicItemKind) -> f32 {
    let carrier = member.owns(Items::QUAD.bits()) || member.owns(Items::INVULNERABILITY.bits());
    let bias = if carrier && kind.is_major() {
        CARRIER_ITEM_NEED_BIAS
    } else {
        1.0
    };
    // A *floor*, not an addend — the same composition `desire_with_floors` uses. Summing them would
    // double-count: the price of taking a red armour and the power it adds are two readings of one
    // event, and adding them put a jacket above a rocket launcher for a naked spawn while the goal
    // layer, on identical inputs, said the opposite. Two layers disagreeing about the same pickup is
    // the failure this whole workstream exists to remove.
    let mut need = (super::goals::POWER_DESIRE_K * kind_power_delta(&member.power_input, kind))
        .max(super::goals::POWER_DESIRE_L * kind_event_price(kind));
    if let StrategicItemKind::Ammo(ch) | StrategicItemKind::Weapon { ammo: ch, .. } = kind {
        if unscored_channel(ch) {
            need = need.max(ammo_scarcity_floor(member.ammo.channel(ch)));
        }
    }
    bias * need
}

/// The same question about an enemy, answered from belief alone.
///
/// An unbelieved enemy is priced as the fresh spawn the model's own baseline says he is, so an
/// unscouted opponent reads as wanting what a spawn wants rather than as wanting nothing.
fn enemy_item_need(enemy: &EnemySnapshot, kind: StrategicItemKind, memory: Option<&TeamMemory>) -> f32 {
    let believed = enemy.believed.unwrap_or_else(power::spawn_input);
    let mut need = (super::goals::POWER_DESIRE_K * kind_power_delta(&believed, kind))
        .max(super::goals::POWER_DESIRE_L * kind_event_price(kind));
    // Witnessed gunfire is the one ammo signal not carried by the loadout: somebody who has been
    // firing wants the box even though the power score cannot see the channel.
    if let StrategicItemKind::Ammo(ch) | StrategicItemKind::Weapon { ammo: ch, .. } = kind {
        if unscored_channel(ch) {
            let spent = memory
                .and_then(|m| m.ammo_spent.get(&(enemy.ent, ch)))
                .copied()
                .unwrap_or(0);
            need = need.max(if spent >= 5 { 70.0 } else { 2.0 });
        }
    }
    // Scale by who is going for it. This feeds the intercept's destination hypotheses, so a strong
    // enemy heading for the quad now outweighs a freshly-spawned one heading for the same quad, and
    // the team spends its interception on the trip that would cost it most. The enemy's power here
    // is a *belief* assembled from witnessed pickups, gunfire and damage — never exact, and never
    // meant to be; it is the difference between "somebody is going for quad" and "their best player
    // is going for quad", which is the whole of the decision.
    need * power::threat_scale(enemy.power.unwrap_or(power::DEAD_POWER))
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

fn set_revision(revisions: &mut [f32], subject: u32, at: f32) {
    if let Some(revision) = revisions.get_mut(subject as usize) {
        *revision = revision.max(at);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "netclient")]
    #[test]
    fn client_shadow_item_evidence_invalidates_advice_above_wire_range() {
        let host = Box::leak(Box::new(crate::netclient::host::NetHost::new("/nonexistent".into())));
        let mut game = GameState::new_client(host);
        let mut inbox = OracleInbox::default();
        inbox.push(OracleNugget {
            epoch: 4,
            generation: 2,
            team: 1,
            recipient: 1,
            kind: NuggetKind::Rearm,
            target_cell: 9,
            subject: 32767,
            confidence: 0.8,
            decision_at: 20.0,
            evidence_at: 18.0,
            expires_at: 24.0,
        });
        inbox.mark_applied(inbox.best(20.0).unwrap());
        set_revision(&mut game.oracle.evidence_revision[1], 32767, 19.0);
        let cancelled = inbox.retain_live(4, 20.1, &game.oracle.evidence_revision[1]);
        assert!(cancelled.is_some());
        assert!(inbox.best(20.1).is_none());
        game.oracle.bump_epoch();
        assert_eq!(game.oracle.evidence_revision[1].get(32767), Some(&0.0));
    }

    /// The rebuild call now reads the equipment gap rather than counting heads, and holds its state
    /// across the band so a team hovering at the threshold doesn't issue and withdraw rearm orders
    /// every second.
    #[test]
    fn reset_follows_the_equipment_gap_with_hysteresis() {
        // Comfortably behind on equipment: rebuild. The threshold is readable because the scale is —
        // a gap of 1.2 is worth about 1.2 team frags over the next minute.
        assert!(reset_by_gap(-2.0, false));
        assert!(!reset_by_gap(-1.0, false), "not yet worth abandoning the map for");
        assert!(!reset_by_gap(0.0, false));
        assert!(!reset_by_gap(3.0, false));

        // Once rebuilding, hold on until genuinely back in the fight.
        assert!(reset_by_gap(-1.0, true), "the band keeps it from flapping");
        assert!(!reset_by_gap(-0.5, true));

        // Being *ahead* never triggers it, however few of us are left standing — which is the whole
        // reason to count equipment instead of heads. Two survivors holding both launchers are not
        // a team in trouble.
        assert!(!reset_by_gap(2.5, false));
        assert!(!reset_by_gap(2.5, true));
    }

    /// Only the rooms the measurement says are worth holding, ordered as it ordered them.
    #[test]
    fn control_weights_cover_the_armor_cycle_and_nothing_else() {
        let ra = control_weight(StrategicItemKind::RedArmor).unwrap();
        let quad = control_weight(StrategicItemKind::Quad).unwrap();
        let ya = control_weight(StrategicItemKind::YellowArmor).unwrap();
        assert!(ra > quad && quad > ya);
        // A weapon room measured at zero once stack is held fixed: holding it buys nothing the
        // weapon in your hands hasn't already bought.
        assert!(control_weight(StrategicItemKind::Weapon {
            bit: Items::ROCKET_LAUNCHER.bits(),
            ammo: AmmoChannel::Rockets,
        })
        .is_none());
        assert!(control_weight(StrategicItemKind::Health).is_none());
    }

    /// The lead has to leave a gap in the item's own cycle, or the order never lifts.
    #[test]
    fn control_lead_stays_inside_the_respawn_cycle() {
        // Both armors return on a twenty-second clock. A lead at or above that is satisfied from the
        // moment the item is taken until it lands, which is not "arrive as it spawns" — it is
        // "stand here forever", and it is what parked a bot on the red armor for a whole match.
        const ARMOR_RESPAWN: f32 = 20.0;
        assert!(control_lead(StrategicItemKind::RedArmor) < ARMOR_RESPAWN);
        assert!(control_lead(StrategicItemKind::YellowArmor) < ARMOR_RESPAWN);

        // Quad's sixty-second cycle affords a longer approach, and still leaves most of it free.
        const QUAD_RESPAWN: f32 = 60.0;
        assert!(control_lead(StrategicItemKind::Quad) < QUAD_RESPAWN / 2.0);
        assert!(control_lead(StrategicItemKind::Quad) > control_lead(StrategicItemKind::RedArmor));
    }

    /// A teammate snapshot whose `power_input` agrees with its plain fields — the invariant
    /// `member_snapshot` maintains, and the thing every need question is asked of.
    fn member(health: f32, armor: f32, armor_type: f32, items: Items, ammo: AmmoSnapshot) -> MemberSnapshot {
        MemberSnapshot {
            ent: 1,
            cell: 0,
            alive: true,
            health,
            armor,
            armor_type,
            items: items.bits(),
            ammo,
            recovering: false,
            power: 0.0,
            power_input: power::PowerInput {
                health,
                armor_value: armor,
                armor_type,
                items: items.as_f32(),
                rockets: ammo.rockets,
                cells: ammo.cells,
                quad_age: None,
                pent_age: None,
                ring: false,
            },
        }
    }

    /// The fitness screen, in the terms that produced it.
    #[test]
    fn only_fighters_who_can_win_the_room_are_sent_to_hold_it() {
        let armed = |health: f32, armor: f32, armor_type: f32| {
            member(
                health,
                armor,
                armor_type,
                Items::ROCKET_LAUNCHER,
                AmmoSnapshot {
                    rockets: 10.0,
                    ..Default::default()
                },
            )
        };

        // The case from the live run: a full red jacket over almost no health. The armor counter
        // reads 200 and the fighter dies to one direct rocket, because only 76 of those points can
        // ever be spent at 19 health. Effective health 95 — under a fresh spawn, so: not sent.
        assert!(!fit_to_contest(&armed(19.0, 200.0, 0.8)));
        // The same jacket with enough health behind it to use it is exactly who should hold a room.
        assert!(fit_to_contest(&armed(80.0, 200.0, 0.8)));
        // A fresh spawn is the bar itself, and clears it.
        assert!(fit_to_contest(&armed(100.0, 0.0, 0.0)));
        // Hurt and unarmored is the same refusal for the same reason.
        assert!(!fit_to_contest(&armed(40.0, 0.0, 0.0)));

        // Being unarmed disqualifies at any health: an escort with nothing to shoot back with is
        // donating a pack, not holding a room.
        let mut naked = armed(100.0, 200.0, 0.8);
        naked.items = 0;
        assert!(!fit_to_contest(&naked));

        // And the dead hold nothing.
        let mut corpse = armed(100.0, 200.0, 0.8);
        corpse.alive = false;
        assert!(!fit_to_contest(&corpse));
    }

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
        assert!(inbox
            .entries()
            .any(|n| n.kind == NuggetKind::Regroup && n.target_cell == 99));
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

    // --- need pricing (K.dpower + event price) ---------------------------------------------------

    fn spawn_member() -> MemberSnapshot {
        member(100.0, 0.0, 0.0, Items::SHOTGUN, AmmoSnapshot::default())
    }

    fn rl() -> StrategicItemKind {
        StrategicItemKind::Weapon {
            bit: Items::ROCKET_LAUNCHER.bits(),
            ammo: AmmoChannel::Rockets,
        }
    }

    /// A spawn wants a launcher far more than it wants ammo or a green jacket, and the old table's
    /// flat "missing weapon = 140" is gone.
    ///
    /// It does *not* assert launcher-over-red-armour. Asked directly, the power curves say a naked
    /// spawn gains about as much from a full red jacket (100 → 300 effective health) as from a
    /// launcher with the five rockets the pickup actually carries — and five lands in the one ammo
    /// bin the fit is non-monotonic in. That is the measurement's answer rather than a bug, and the
    /// goal layer gives the same one from the same curves, which is what matters here.
    #[test]
    fn a_missing_launcher_is_priced_by_what_it_adds() {
        let m = spawn_member();
        let launcher = member_item_need(&m, rl());
        assert!(launcher > member_item_need(&m, StrategicItemKind::GreenArmor));
        assert!(launcher > member_item_need(&m, StrategicItemKind::Ammo(AmmoChannel::Rockets)));
        assert!(launcher > member_item_need(&m, StrategicItemKind::Health));
        // The anchor the travel weight is calibrated against.
        assert!((200.0..400.0).contains(&launcher), "launcher need was {launcher}");
        // A member who already owns it wants only the ammo, which is worth far less.
        let owns = member(
            100.0,
            0.0,
            0.0,
            Items::SHOTGUN.union(Items::ROCKET_LAUNCHER),
            AmmoSnapshot {
                rockets: 20.0,
                ..Default::default()
            },
        );
        assert!(member_item_need(&owns, rl()) < launcher / 2.0);
    }

    /// A stacked fighter gains almost nothing from another red armour, and should still take it.
    #[test]
    fn a_stacked_member_still_wants_red_armour_for_its_price() {
        let stacked = member(
            100.0,
            200.0,
            0.8,
            Items::ROCKET_LAUNCHER,
            AmmoSnapshot {
                rockets: 50.0,
                ..Default::default()
            },
        );
        // Marginal power is ~0 — he is already wearing it.
        assert!(kind_power_delta(&stacked.power_input, StrategicItemKind::RedArmor) < 0.05);
        // The event price carries it anyway: denying the armour is most of what taking it buys.
        let need = member_item_need(&stacked, StrategicItemKind::RedArmor);
        assert!(need > 100.0, "red armour still worth fetching, got {need}");
        // Green armour, by contrast, is worth nothing to him: it would replace nothing.
        assert_eq!(member_item_need(&stacked, StrategicItemKind::GreenArmor), 0.0);
    }

    /// Shells and nails must stay *lower priority*, never invisible.
    ///
    /// The power fit has no term for either channel, so on marginal power alone they price at exactly
    /// zero and the planner would stop fetching them for good — a bot dry on both would be left
    /// holding an axe. The floor keeps them last in the ranking and still on it.
    #[test]
    fn a_dry_shotgun_still_rates_a_shell_fetch() {
        let dry = member(100.0, 0.0, 0.0, Items::SHOTGUN, AmmoSnapshot::default());
        let shells = member_item_need(&dry, StrategicItemKind::Ammo(AmmoChannel::Shells));
        assert!(shells > 0.0, "a dry shotgun must still rate shells");

        // ...but behind everything the power score can actually see.
        assert!(shells < member_item_need(&dry, rl()));
        assert!(shells < member_item_need(&dry, StrategicItemKind::RedArmor));
        assert!(shells < member_item_need(&dry, StrategicItemKind::Quad));

        // A member already carrying shells wants them less; a full one, not at all.
        let stocked = member(
            100.0,
            0.0,
            0.0,
            Items::SHOTGUN,
            AmmoSnapshot {
                shells: 20.0,
                ..Default::default()
            },
        );
        assert_eq!(
            member_item_need(&stocked, StrategicItemKind::Ammo(AmmoChannel::Shells)),
            0.0
        );
        assert!(
            shells
                > member_item_need(
                    &member(
                        100.0,
                        0.0,
                        0.0,
                        Items::SHOTGUN,
                        AmmoSnapshot {
                            shells: 10.0,
                            ..Default::default()
                        },
                    ),
                    StrategicItemKind::Ammo(AmmoChannel::Shells)
                )
        );
    }

    /// The ring prices out of team planning; the pentagram is the most valuable thing on the board.
    #[test]
    fn the_ring_prices_out_and_the_pentagram_does_not() {
        let m = spawn_member();
        let ring = member_item_need(&m, StrategicItemKind::Ring);
        let pent = member_item_need(&m, StrategicItemKind::Pent);
        let quad = member_item_need(&m, StrategicItemKind::Quad);
        assert!(pent > quad, "pentagram {pent} is priced above quad {quad}");
        assert!(ring < quad / 4.0, "ring {ring} must not read like a powerup");
        // And it is not a major, so it never reaches the planner's cycle logic at all.
        assert!(!StrategicItemKind::Ring.is_major());
        assert!(StrategicItemKind::Pent.is_major());
    }

    /// Enemy needs come from belief, and an unseen enemy is a spawn rather than a nobody.
    #[test]
    fn enemy_needs_are_belief_not_truth() {
        let unseen = EnemySnapshot {
            ent: 2,
            evidence_at: 0.0,
            cue: None,
            power: None,
            believed: None,
            quad: false,
        };
        // Nothing believed: priced as the fresh spawn the model's own baseline says he is, so he
        // reads as wanting a launcher badly — which is exactly what a spawn wants.
        let unseen_rl = enemy_item_need(&unseen, rl(), None);
        assert!(unseen_rl > 0.0);

        // Believed already holding one: the launcher is worth much less to him.
        let armed_belief = power::PowerInput {
            items: Items::SHOTGUN.union(Items::ROCKET_LAUNCHER).as_f32(),
            rockets: 20.0,
            ..power::spawn_input()
        };
        let armed = EnemySnapshot {
            believed: Some(armed_belief),
            power: Some(power::power(&armed_belief)),
            ..unseen.clone()
        };
        assert!(
            enemy_item_need(&armed, rl(), None) < unseen_rl,
            "a believed launcher should collapse the launcher need"
        );
    }

    /// Both layers that can hand out a major must agree on who gets it.
    #[test]
    fn a_powerup_carrier_outranks_a_bare_teammate_for_a_major() {
        let bare = member(100.0, 0.0, 0.0, Items::ROCKET_LAUNCHER, AmmoSnapshot::default());
        let mut carrier = bare.clone();
        carrier.items = Items::ROCKET_LAUNCHER.union(Items::QUAD).bits();
        carrier.power_input.items = Items::ROCKET_LAUNCHER.union(Items::QUAD).as_f32();
        carrier.power_input.quad_age = Some(0.0);
        assert!(
            member_item_need(&carrier, StrategicItemKind::RedArmor)
                > member_item_need(&bare, StrategicItemKind::RedArmor),
            "feed the carrier"
        );
        // The explicit bias is majors-only. A carrier still rates ordinary pickups somewhat higher,
        // but that is the power score being multiplicative — everything is worth more under a quad —
        // not the feed-the-carrier rule reaching where it was not meant to.
        let ammo = StrategicItemKind::Ammo(AmmoChannel::Rockets);
        let (c_ammo, b_ammo) = (member_item_need(&carrier, ammo), member_item_need(&bare, ammo));
        assert!(c_ammo > b_ammo);
        assert!(
            c_ammo < b_ammo * CARRIER_ITEM_NEED_BIAS,
            "ordinary pickups must not get the major bias: {c_ammo} vs {b_ammo}"
        );
    }
}
