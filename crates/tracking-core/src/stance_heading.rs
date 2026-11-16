//! Stance heading filter: per-sensor world-yaw error from absolute-headset displacement while a
//! foot is hypothesised stationary, plus weak deadbanded anatomical priors.
//!
//! Error state `δ ∈ R⁶` is the remaining yaw error of each core segment (`Role::CORE` order):
//! the true rotation is `Ry(δ_i)·R_i` where `R_i` already includes the filter's accumulated
//! correction `c_i`. After every accepted update `c += δ̂` and `δ̂ = 0`; the caller applies
//! `c_i` as a left world-yaw `Ry(c_i)` on top of calibration and recovery.
//!
//! Observation (per planted foot, between keyframes `a → b`):
//!
//! ```text
//! Δp̂_s = Δp_head + Δhead_lever + Σ_i Δlever[s][i]
//! z = −(Δp̂_s)_xz ,  H_i = (ŷ × Δlever[s][i])_xz ,  z = H·δ + n
//! ```
//!
//! Only *changes* of the levers carry information, so a still body, a head-only turn or
//! headset noise with unchanged chain geometry cannot move the estimate. Keyframe levers are
//! re-rotated about +Y by `c_i(now) − c_i(keyframe)` so both ends are compared under one set
//! of corrections.
//!
//! By default only the leg columns of `H` are estimated (`estimate_torso_yaw = false`): the
//! chest and hip levers are not rigid (a flexed spine, soft tissue), and in the first live
//! session torso model error was absorbed as chest yaw. Their lever changes still enter the
//! noise budget (with `torso_motion_noise_ratio`), increments dominated by torso motion are not
//! used, and accumulated corrections are bounded per segment.
//!
//! This is a model-based estimator validated only on synthetic data generated with the same
//! kinematic model. A planted-foot hypothesis is not proof of a stationary ankle, and the
//! priors cannot distinguish a held anatomical twist from drift.

#![allow(clippy::needless_range_loop)] // Small fixed-size matrix arithmetic reads best indexed.

use crate::chain::{BodyChain, CALF, CHEST, FOOT_CHAIN, HIP, THIGH, lateral_heading, unit_quat};
use crate::contact::vertical_rotation;
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

const N: usize = 6;
type Matrix6 = [[f64; N]; N];

/// Longest interval over which process noise is integrated in one step.
const MAX_PREDICT_SECS: f64 = 10.0;
/// Head or ankle increments beyond this are tracking discontinuities, not motion.
const MAX_PLAUSIBLE_INCREMENT_M: f32 = 1.5;
/// Knee-flexion history capacity (decimated samples).
const FLEX_CAPACITY: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct KneePriorConfig {
    pub min_flexion_degrees: f32,
    pub max_flexion_degrees: f32,
    /// Required flexion excursion (max − min) within `change_window_secs`.
    pub min_flexion_change_degrees: f32,
    pub change_window_secs: f32,
    pub deadband_degrees: f32,
    pub sigma_degrees: f32,
    pub rate_hz: f32,
}

impl Default for KneePriorConfig {
    fn default() -> Self {
        Self {
            min_flexion_degrees: 25.0,
            max_flexion_degrees: 120.0,
            min_flexion_change_degrees: 8.0,
            change_window_secs: 2.0,
            deadband_degrees: 5.0,
            sigma_degrees: 15.0,
            rate_hz: 2.0,
        }
    }
}

/// Prior between segments that must all be near upright (pelvis vs thighs, chest vs pelvis).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct UprightPriorConfig {
    pub max_tilt_degrees: f32,
    pub deadband_degrees: f32,
    pub sigma_degrees: f32,
    pub rate_hz: f32,
}

impl Default for UprightPriorConfig {
    fn default() -> Self {
        Self {
            max_tilt_degrees: 30.0,
            deadband_degrees: 10.0,
            sigma_degrees: 20.0,
            rate_hz: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StanceHeadingConfig {
    /// Stance displacement observation (the absolute, common-mode-observing measurement).
    pub enable_displacement: bool,
    pub enable_knee_prior: bool,
    pub enable_pelvis_prior: bool,
    pub enable_chest_prior: bool,
    /// Initial 1σ after calibration, recovery or `reset`.
    pub initial_sigma_degrees: f32,
    /// Time-driven process noise: variance grows by `process_sigma_degrees²` per second.
    pub process_sigma_degrees: f32,
    /// Rotation-driven process noise `κ` (√s): variance grows by `(κ·ω)²` per second.
    pub process_rate_coefficient: f32,
    pub max_sigma_degrees: f32,
    /// Excitation `max_i |Δlever_i|_xz` that triggers a measurement (after `min_keyframe_secs`).
    pub measure_excitation_m: f32,
    /// Minimum excitation for a measurement when the keyframe times out.
    pub min_excitation_m: f32,
    pub min_keyframe_secs: f32,
    pub max_keyframe_secs: f32,
    /// Faster headset motion between consecutive frames is a tracking-space discontinuity.
    pub max_head_speed_mps: f32,
    /// A longer frame gap drops keyframes and requests release.
    pub max_frame_gap_secs: f32,
    pub hmd_sigma_m: f32,
    /// Eye/neck model error per radian of head rotation between keyframes.
    pub neck_sigma_m_per_rad: f32,
    pub creep_sigma_mps: f32,
    /// Unmodelled ankle motion per radian of calf rotation about vertical between keyframes
    /// (a foot pivoting on its ball moves the ankle on an arc; refinement beyond the spec).
    pub pivot_sigma_m_per_rad: f32,
    /// Unmodelled motion proportional to how much the leg levers moved (ρ).
    pub motion_noise_ratio: f32,
    /// Unmodelled motion proportional to how much the chest and hip levers moved (ρ_torso).
    /// A flexed spine is not a rigid rod and waist trackers ride on soft tissue, so a torso
    /// lever change is far less certain than a leg lever change; measurements taken while
    /// the torso moves are discounted for every segment, legs included.
    pub torso_motion_noise_ratio: f32,
    /// Estimate chest and hip yaw from stance displacement. Off by default ("legs only"):
    /// the torso levers are not rigid, and in the first live session torso model error was
    /// absorbed as chest yaw. Off, those sensitivity columns are zero, torso lever changes
    /// do not count as excitation, and torso yaw is held only by the deadbanded priors.
    pub estimate_torso_yaw: bool,
    /// An increment whose largest chest/hip lever change (xz) exceeds this multiple of its
    /// excitation (largest estimated lever change) is not used: a bow moves the torso far
    /// more than the legs, and its model error is correlated across increments, so even a
    /// large per-increment noise would add up. Non-positive disables the gate.
    pub max_torso_excitation_ratio: f32,
    /// Hard bound on the accumulated correction of each leg segment (degrees); a safety net
    /// reported in [`StanceHeadingOutput::correction_limited`] when it binds.
    pub max_leg_correction_degrees: f32,
    /// Hard bound on the accumulated correction of the chest and hip (degrees).
    pub max_torso_correction_degrees: f32,
    /// Relative bone-length uncertainty (ε_len), applied along each lever change.
    pub length_uncertainty: f32,
    /// χ² gate for the 2-dof displacement innovation (99 %).
    pub gate_chi2: f32,
    /// χ² gate for 1-dof prior pseudo-measurements (99 %).
    pub prior_gate_chi2: f32,
    /// Per-update clamp of the largest state step.
    pub max_step_degrees: f32,
    /// Consecutive gated-out displacement measurements that force-release a contact.
    pub outlier_release_count: u32,
    /// Smoothing of the filter's own angular-speed estimate (from successive rotations).
    pub angular_speed_tau_secs: f32,
    /// Variance (as 1σ, degrees, added in quadrature) given to every segment of a foot's chain
    /// for each gated-out displacement measurement, and for each accepted one while the
    /// running NIS average is above `nis_average_threshold`. Lets a confident but wrong state
    /// reopen instead of rejecting good data forever. Zero disables inflation.
    pub inflation_step_degrees: f32,
    /// Inflation never raises a segment's 1σ above this (process noise still may).
    pub inflation_max_sigma_degrees: f32,
    /// Weight of each formed displacement measurement in the per-foot running average of
    /// NIS per degree of freedom (expected about 1 for a consistent filter).
    pub nis_average_weight: f32,
    /// Running NIS-per-dof average above which accepted measurements also inflate.
    pub nis_average_threshold: f32,
    /// Contact speed allowance `sin(min(gain·σ_chain, max))·|v_head,xz|`, see
    /// [`contact_speed_allowance`].
    pub allowance_sigma_gain: f32,
    pub allowance_max_degrees: f32,
    /// Contact starvation: a foot that has not been planted for this long while the
    /// headset moves horizontally faster than `starvation_head_speed_mps` (walking: every
    /// step should plant) is evidence that its leg's yaw is wrong enough to make the computed
    /// stance ankle slide past the contact gates. Its thigh and calf then receive
    /// `starvation_inflation_degrees` (in quadrature, up to `inflation_max_sigma_degrees`)
    /// per period, which widens that foot's contact allowance. A yaw error cannot fake
    /// height or vertical speed, so the remaining contact gates still reject real swing.
    /// Non-positive disables.
    pub starvation_secs: f32,
    pub starvation_head_speed_mps: f32,
    pub starvation_inflation_degrees: f32,
    pub knee_prior: KneePriorConfig,
    pub pelvis_prior: UprightPriorConfig,
    pub chest_prior: UprightPriorConfig,
}

impl Default for StanceHeadingConfig {
    fn default() -> Self {
        Self {
            enable_displacement: true,
            enable_knee_prior: true,
            enable_pelvis_prior: true,
            enable_chest_prior: true,
            initial_sigma_degrees: 2.0,
            process_sigma_degrees: 0.10,
            process_rate_coefficient: 0.015,
            max_sigma_degrees: 45.0,
            measure_excitation_m: 0.03,
            min_excitation_m: 0.01,
            min_keyframe_secs: 0.05,
            max_keyframe_secs: 0.75,
            max_head_speed_mps: 4.0,
            max_frame_gap_secs: 0.10,
            hmd_sigma_m: 0.003,
            neck_sigma_m_per_rad: 0.04,
            creep_sigma_mps: 0.005,
            pivot_sigma_m_per_rad: 0.12,
            motion_noise_ratio: 0.10,
            torso_motion_noise_ratio: 0.5,
            estimate_torso_yaw: false,
            max_torso_excitation_ratio: 1.0,
            max_leg_correction_degrees: 30.0,
            max_torso_correction_degrees: 12.0,
            length_uncertainty: 0.08,
            gate_chi2: 9.21,
            prior_gate_chi2: 6.63,
            max_step_degrees: 1.0,
            outlier_release_count: 3,
            angular_speed_tau_secs: 0.05,
            inflation_step_degrees: 1.0,
            inflation_max_sigma_degrees: 15.0,
            nis_average_weight: 0.2,
            nis_average_threshold: 2.0,
            allowance_sigma_gain: 2.0,
            allowance_max_degrees: 30.0,
            starvation_secs: 1.0,
            starvation_head_speed_mps: 0.3,
            starvation_inflation_degrees: 3.0,
            knee_prior: KneePriorConfig::default(),
            pelvis_prior: UprightPriorConfig::default(),
            chest_prior: UprightPriorConfig {
                deadband_degrees: 8.0,
                ..UprightPriorConfig::default()
            },
        }
    }
}

/// One frame of filter input. `chain` and `rotations` must already include the filter's
/// current corrections (as returned by [`StanceHeadingFilter::corrections`] before this call),
/// applied as a left world-yaw `Ry(c_i)`.
///
/// Step the filter only with a live, fresh absolute headset pose. A stale headset that keeps
/// repeating its last finite position cannot be told apart from a still user here; when the
/// anchor goes stale or the session is in desktop preview, stop stepping (and `reset()` after
/// recalibration or recovery).
#[derive(Clone, Copy, Debug)]
pub struct StanceHeadingInput<'a> {
    pub at_secs: f64,
    pub head_position: Vec3,
    pub head_rotation: Quat,
    pub chain: &'a BodyChain,
    pub rotations: [Quat; 6],
    /// False for stale, firmware-held or otherwise invalid segments.
    pub usable: [bool; 6],
    /// Contact hypotheses (`ContactPhase::Planted`) per foot.
    pub planted: [bool; 2],
    /// Smoothed angular speed per segment, rad/s; zeros if unknown.
    pub angular_speed: [f32; 6],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StanceHeadingStatus {
    /// No planted foot and no prior applied.
    #[default]
    Idle,
    /// A planted foot is being tracked; no measurement this frame.
    Observing,
    /// At least one measurement or prior was accepted this frame.
    Updated,
    /// Measurements were formed but all were gated out.
    Rejected,
    /// Headset jump or time anomaly: keyframes dropped, contacts asked to release.
    Discontinuity,
    /// Nonfinite or degenerate time/headset input: nothing changed.
    InvalidInput,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FootHeadingDiagnostics {
    pub keyframe_active: bool,
    pub accepted: u32,
    pub rejected: u32,
    pub rejection_streak: u32,
    /// `max_i |Δlever_i|_xz` of the last evaluated increment, meters.
    pub last_excitation_m: f32,
    /// Norm of the last innovation, meters.
    pub last_innovation_m: f32,
    /// Last normalized innovation squared `νᵀS⁻¹ν`.
    pub last_nis: f32,
    /// Running average of NIS per degree of freedom over formed measurements (accepted and
    /// rejected, each clipped at 10).
    #[serde(default)]
    pub nis_average: f32,
    /// Covariance inflations triggered by this foot's measurements.
    #[serde(default)]
    pub inflations: u32,
    /// Increments not used because torso lever motion dominated them
    /// (`max_torso_excitation_ratio`).
    #[serde(default)]
    pub torso_skipped: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PriorDiagnostic {
    /// The prior's gates (pose, excitation, usable inputs) were satisfied this frame.
    pub active: bool,
    /// Last evaluated residual (before the deadband), degrees.
    pub last_residual_degrees: f32,
    pub updates: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PriorDiagnostics {
    /// Knee hinge prior per leg (`[left, right]`).
    pub knee: [PriorDiagnostic; 2],
    pub pelvis: PriorDiagnostic,
    pub chest: PriorDiagnostic,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StanceHeadingOutput {
    /// Total accumulated correction per segment, radians (apply as left `Ry(c_i)`).
    pub corrections: [f32; 6],
    /// 1σ of the remaining yaw error per segment, radians.
    pub sigma: [f32; 6],
    pub feet: [FootHeadingDiagnostics; 2],
    pub priors: PriorDiagnostics,
    /// The contact tracker should release these feet (outlier streak or discontinuity).
    pub release_requests: [bool; 2],
    pub status: StanceHeadingStatus,
    /// The segment's accumulated correction is held at its configured bound
    /// (`max_leg_correction_degrees` / `max_torso_correction_degrees`).
    #[serde(default)]
    pub correction_limited: [bool; 6],
}

#[derive(Clone, Copy, Debug)]
struct Keyframe {
    at_secs: f64,
    head_position: Vec3,
    head_lever: Vec3,
    head_rotation: Quat,
    calf_rotation: Quat,
    levers: [Vec3; 6],
    corrections: [f64; 6],
}

#[derive(Clone, Copy, Debug)]
struct FlexHistory {
    times: [f64; FLEX_CAPACITY],
    values: [f32; FLEX_CAPACITY],
    next: usize,
    len: usize,
}

impl FlexHistory {
    const EMPTY: Self = Self {
        times: [0.0; FLEX_CAPACITY],
        values: [0.0; FLEX_CAPACITY],
        next: 0,
        len: 0,
    };

    fn last_time(&self) -> Option<f64> {
        (self.len > 0).then(|| self.times[(self.next + FLEX_CAPACITY - 1) % FLEX_CAPACITY])
    }

    fn push(&mut self, at_secs: f64, value: f32, min_spacing: f64) {
        if let Some(last) = self.last_time() {
            if at_secs < last {
                *self = Self::EMPTY;
            } else if at_secs - last < min_spacing {
                return;
            }
        }
        self.times[self.next] = at_secs;
        self.values[self.next] = value;
        self.next = (self.next + 1) % FLEX_CAPACITY;
        self.len = (self.len + 1).min(FLEX_CAPACITY);
    }

    /// Excursion (max − min) of samples within `window` before `now`, including `current`.
    fn excursion(&self, now: f64, window: f64, current: f32) -> f32 {
        let (mut low, mut high) = (current, current);
        for k in 0..self.len {
            let index = (self.next + FLEX_CAPACITY - 1 - k) % FLEX_CAPACITY;
            if now - self.times[index] > window {
                break;
            }
            low = low.min(self.values[index]);
            high = high.max(self.values[index]);
        }
        high - low
    }
}

#[derive(Clone, Copy)]
struct Increment<'a> {
    key: &'a Keyframe,
    head_rotation: Quat,
    calf_rotation: Quat,
    calf: usize,
    delta: &'a [Vec3; N],
    tau: f32,
}

#[derive(Clone, Copy)]
struct PriorSpec {
    slot: usize,
    deadband_degrees: f32,
    sigma_degrees: f32,
    rate_hz: f32,
}

impl PriorSpec {
    fn upright(slot: usize, config: &UprightPriorConfig) -> Self {
        Self {
            slot,
            deadband_degrees: config.deadband_degrees,
            sigma_degrees: config.sigma_degrees,
            rate_hz: config.rate_hz,
        }
    }
}

enum UpdateOutcome {
    Accepted { nis: f64 },
    Rejected { nis: f64 },
    Degenerate,
}

/// Small error-state Kalman filter over six segment yaw errors. See the module docs.
#[derive(Clone, Debug)]
pub struct StanceHeadingFilter {
    config: StanceHeadingConfig,
    corrections: [f64; N],
    covariance: Matrix6,
    keyframes: [Option<Keyframe>; 2],
    last_at_secs: Option<f64>,
    last_head_position: Option<Vec3>,
    previous_raw: [Option<Quat>; N],
    angular_speed: [f32; N],
    flex: [FlexHistory; 2],
    /// Last application time of knee L, knee R, pelvis, chest priors.
    prior_last_at: [Option<f64>; 4],
    /// Smoothed horizontal headset speed (m/s) from successive positions.
    head_speed: f32,
    /// Start of the current unplanted, translating period per foot.
    starved_since: [Option<f64>; 2],
    diagnostics: StanceHeadingOutput,
}

impl StanceHeadingFilter {
    pub fn new(config: StanceHeadingConfig) -> Self {
        let mut filter = Self {
            config,
            corrections: [0.0; N],
            covariance: [[0.0; N]; N],
            keyframes: [None; 2],
            last_at_secs: None,
            last_head_position: None,
            previous_raw: [None; N],
            angular_speed: [0.0; N],
            flex: [FlexHistory::EMPTY; 2],
            prior_last_at: [None; 4],
            head_speed: 0.0,
            starved_since: [None; 2],
            diagnostics: StanceHeadingOutput::default(),
        };
        filter.reset();
        filter
    }

    pub fn config(&self) -> &StanceHeadingConfig {
        &self.config
    }

    /// Zero corrections, initial covariance, no keyframes or history (after calibration or
    /// recovery).
    pub fn reset(&mut self) {
        self.corrections = [0.0; N];
        self.covariance = self.initial_covariance();
        self.keyframes = [None; 2];
        self.last_at_secs = None;
        self.last_head_position = None;
        self.previous_raw = [None; N];
        self.angular_speed = [0.0; N];
        self.flex = [FlexHistory::EMPTY; 2];
        self.prior_last_at = [None; 4];
        self.head_speed = 0.0;
        self.starved_since = [None; 2];
        self.diagnostics = StanceHeadingOutput::default();
        self.publish_state();
    }

    /// Forget motion history (keyframes, previous samples, flexion history) while keeping
    /// the accumulated corrections and their covariance. Use after a tracking-space rebase
    /// or any interruption that makes the previous frame incomparable with the next one.
    pub fn invalidate_history(&mut self) {
        self.keyframes = [None; 2];
        self.last_at_secs = None;
        self.last_head_position = None;
        self.previous_raw = [None; N];
        self.flex = [FlexHistory::EMPTY; 2];
        self.head_speed = 0.0;
        self.starved_since = [None; 2];
        self.publish_state();
    }

    /// Replace the accumulated corrections (radians), for example to restore them when an
    /// operation that reset the filter is undone. Nonfinite values become zero. Covariance
    /// returns to its initial value and motion history is dropped.
    pub fn set_corrections(&mut self, corrections: [f32; 6]) {
        self.reset();
        self.corrections = corrections.map(|c| if c.is_finite() { f64::from(c) } else { 0.0 });
        self.apply_bounds();
        self.publish_state();
    }

    /// Whether stance displacement observes segment `i`'s yaw (legs always; chest and hip
    /// only with `estimate_torso_yaw`).
    fn displacement_observes(&self, i: usize) -> bool {
        self.config.estimate_torso_yaw || !(i == CHEST || i == HIP)
    }

    /// Bound on the accumulated correction of segment `i`, radians.
    fn correction_bound(&self, i: usize) -> f64 {
        let degrees = if i == CHEST || i == HIP {
            self.config.max_torso_correction_degrees
        } else {
            self.config.max_leg_correction_degrees
        };
        let bound = f64::from(degrees).to_radians();
        if bound.is_finite() && bound > 0.0 {
            bound.min(PI)
        } else {
            PI
        }
    }

    /// Clamp each accumulated correction to its bound.
    fn apply_bounds(&mut self) {
        for i in 0..N {
            let bound = self.correction_bound(i);
            self.corrections[i] = self.corrections[i].clamp(-bound, bound);
        }
    }

    /// Adds uncertainty per segment (1σ in degrees, combined in quadrature, the result capped
    /// at `max_sigma_degrees`), for example after a strap was re-seated or a sensor came back
    /// from a fault, so the filter may move that segment faster. Nonfinite or non-positive
    /// entries are ignored. Corrections are unchanged.
    pub fn add_uncertainty(&mut self, sigma_degrees: [f32; 6]) {
        let cap = f64::from(self.config.max_sigma_degrees)
            .to_radians()
            .powi(2);
        for (i, sigma) in sigma_degrees.into_iter().enumerate() {
            if sigma.is_finite() && sigma > 0.0 {
                let variance = f64::from(sigma).to_radians().powi(2);
                let current = self.covariance[i][i];
                self.covariance[i][i] = (current + variance).min(cap.max(current));
            }
        }
        self.sanitize_covariance();
        self.publish_state();
    }

    /// Contact speed allowance per foot for the current uncertainty and headset velocity
    /// (m/s), using this filter's configuration. See [`contact_speed_allowance`].
    pub fn contact_speed_allowance(&self, head_velocity: Vec3) -> [f32; 2] {
        let mut sigma = self.sigma();
        if !self.displacement_observes(HIP) {
            // The hip's yaw is not estimated from displacement: its uncertainty is not
            // something stance measurements could reduce, so it does not widen the gates.
            sigma[HIP] = 0.0;
        }
        contact_speed_allowance(
            sigma,
            head_velocity,
            self.config.allowance_sigma_gain,
            self.config.allowance_max_degrees,
        )
    }

    /// Accumulated corrections per segment, radians.
    pub fn corrections(&self) -> [f32; 6] {
        self.corrections.map(|c| c as f32)
    }

    /// 1σ of the remaining yaw error per segment, radians.
    pub fn sigma(&self) -> [f32; 6] {
        std::array::from_fn(|i| self.covariance[i][i].max(0.0).sqrt() as f32)
    }

    /// Full error covariance (rad²), for diagnostics and tests.
    pub fn covariance(&self) -> [[f64; 6]; 6] {
        self.covariance
    }

    /// Last output (without advancing).
    pub fn last_output(&self) -> StanceHeadingOutput {
        self.diagnostics
    }

    pub fn step(&mut self, input: &StanceHeadingInput) -> StanceHeadingOutput {
        let mut out = self.diagnostics;
        out.release_requests = [false; 2];
        out.status = StanceHeadingStatus::Idle;
        for prior in out
            .priors
            .knee
            .iter_mut()
            .chain([&mut out.priors.pelvis, &mut out.priors.chest])
        {
            prior.active = false;
        }

        let at = input.at_secs;
        if !at.is_finite() {
            self.drop_keyframes();
            out.status = StanceHeadingStatus::InvalidInput;
            return self.finish(out);
        }
        let dt = self.last_at_secs.map(|previous| at - previous);
        self.last_at_secs = Some(at);
        if let Some(dt) = dt.filter(|dt| *dt > 0.0) {
            self.predict(dt.min(MAX_PREDICT_SECS), &input.angular_speed);
        }
        let time_ok =
            dt.is_none_or(|dt| dt > 0.0 && dt <= f64::from(self.config.max_frame_gap_secs));
        if !time_ok {
            self.drop_keyframes();
            self.previous_raw = [None; N];
            self.last_head_position = input
                .head_position
                .is_finite()
                .then_some(input.head_position);
            out.release_requests = [true; 2];
            out.status = StanceHeadingStatus::Discontinuity;
            return self.finish(out);
        }

        let (head_rotation, head_rotation_ok) = unit_quat(input.head_rotation);
        let head_ok = input.head_position.is_finite()
            && head_rotation_ok
            && input.chain.head_finite
            && input.chain.head_lever.is_finite();
        if !head_ok {
            self.drop_keyframes();
            self.last_head_position = None;
            out.release_requests = [true; 2];
            out.status = StanceHeadingStatus::InvalidInput;
            return self.finish(out);
        }
        if let (Some(previous), Some(dt)) = (self.last_head_position, dt) {
            let jump = (input.head_position - previous).length();
            let speed = jump / dt as f32;
            let continuous = speed.is_finite()
                && speed <= self.config.max_head_speed_mps
                && jump <= MAX_PLAUSIBLE_INCREMENT_M;
            if !continuous {
                self.drop_keyframes();
                self.last_head_position = Some(input.head_position);
                out.release_requests = [true; 2];
                out.status = StanceHeadingStatus::Discontinuity;
                return self.finish(out);
            }
        }
        if let (Some(previous), Some(dt)) = (self.last_head_position, dt) {
            let step = input.head_position - previous;
            let speed = step.x.hypot(step.z) / dt as f32;
            let alpha = 1.0 - (-(dt as f32) / 0.2).exp();
            if speed.is_finite() {
                self.head_speed += (speed - self.head_speed) * alpha;
            }
        }
        self.last_head_position = Some(input.head_position);
        self.track_starvation(at, &input.planted, &mut out);

        let c_input = self.corrections;
        let mut rotations = [Quat::IDENTITY; N];
        let mut segment_ok = [false; N];
        for i in 0..N {
            let (rotation, finite) = unit_quat(input.rotations[i]);
            rotations[i] = rotation;
            segment_ok[i] = finite && input.usable[i] && input.chain.segment_finite[i];
        }
        self.track_angular_speed(dt, &rotations, &segment_ok, &c_input);

        let mut accepted = false;
        let mut rejected = false;
        let mut observing = false;
        for foot in 0..2 {
            let chain_ok = FOOT_CHAIN[foot].iter().all(|&i| segment_ok[i])
                && input.chain.foot_finite(foot)
                && input.chain.ankles[foot].is_finite()
                && input.chain.levers[foot]
                    .iter()
                    .all(|lever| lever.is_finite());
            if !self.config.enable_displacement || !input.planted[foot] || !chain_ok {
                self.keyframes[foot] = None;
                continue;
            }
            observing = true;
            let current = Keyframe {
                at_secs: at,
                head_position: input.head_position,
                head_lever: input.chain.head_lever,
                head_rotation,
                calf_rotation: rotations[CALF[foot]],
                levers: input.chain.levers[foot],
                corrections: c_input,
            };
            let Some(key) = self.keyframes[foot] else {
                // A new contact: store the first keyframe; streaks never span contacts.
                self.keyframes[foot] = Some(current);
                out.feet[foot].rejection_streak = 0;
                continue;
            };
            let tau = (at - key.at_secs) as f32;
            let (delta, _) =
                lever_increments(&key, &input.chain.levers[foot], &FOOT_CHAIN[foot], &c_input);
            // Only lever changes of segments the measurement estimates count as excitation.
            let excitation = FOOT_CHAIN[foot]
                .iter()
                .filter(|&&i| self.displacement_observes(i))
                .map(|&i| delta[i].x.hypot(delta[i].z))
                .fold(0.0_f32, f32::max);
            let measure = if excitation >= self.config.measure_excitation_m
                && tau >= self.config.min_keyframe_secs
            {
                true
            } else if tau >= self.config.max_keyframe_secs {
                excitation >= self.config.min_excitation_m
            } else {
                continue;
            };
            out.feet[foot].last_excitation_m = finite_f32(excitation);
            // An increment dominated by torso motion (a bow) carries torso model error
            // that is correlated from one increment to the next; treating it as white
            // noise would let many small increments add up to a false leg correction.
            let torso_motion = [CHEST, HIP]
                .iter()
                .map(|&i| delta[i].x.hypot(delta[i].z))
                .fold(0.0_f32, f32::max);
            let ratio = self.config.max_torso_excitation_ratio;
            let measure = if measure && ratio > 0.0 && torso_motion > ratio * excitation {
                out.feet[foot].torso_skipped = out.feet[foot].torso_skipped.saturating_add(1);
                false
            } else {
                measure
            };
            let mut release = false;
            if measure {
                let increment = Increment {
                    key: &key,
                    head_rotation,
                    calf_rotation: rotations[CALF[foot]],
                    calf: CALF[foot],
                    delta: &delta,
                    tau,
                };
                match self.displacement_update(input, &increment, &c_input) {
                    Some((UpdateOutcome::Accepted { nis }, innovation)) => {
                        accepted = true;
                        let diagnostics = &mut out.feet[foot];
                        diagnostics.accepted = diagnostics.accepted.saturating_add(1);
                        diagnostics.rejection_streak = 0;
                        diagnostics.last_nis = finite_f32(nis as f32);
                        diagnostics.last_innovation_m = finite_f32(innovation);
                        let average = self.track_nis(&mut out.feet[foot], nis);
                        if average > f64::from(self.config.nis_average_threshold) {
                            self.inflate_chain(foot, &mut out.feet[foot]);
                        }
                    }
                    Some((UpdateOutcome::Rejected { nis }, innovation)) => {
                        rejected = true;
                        let diagnostics = &mut out.feet[foot];
                        diagnostics.rejected = diagnostics.rejected.saturating_add(1);
                        diagnostics.rejection_streak += 1;
                        diagnostics.last_nis = finite_f32(nis as f32);
                        diagnostics.last_innovation_m = finite_f32(innovation);
                        if diagnostics.rejection_streak >= self.config.outlier_release_count.max(1)
                        {
                            diagnostics.rejection_streak = 0;
                            release = true;
                        }
                        self.track_nis(&mut out.feet[foot], nis);
                        self.inflate_chain(foot, &mut out.feet[foot]);
                    }
                    Some((UpdateOutcome::Degenerate, _)) => {}
                    None => release = true,
                }
            }
            if release {
                out.release_requests[foot] = true;
                self.keyframes[foot] = None;
            } else {
                self.keyframes[foot] = Some(current);
            }
        }

        if self.apply_priors(at, &rotations, &segment_ok, &c_input, &mut out) {
            accepted = true;
        }

        out.status = if accepted {
            StanceHeadingStatus::Updated
        } else if rejected {
            StanceHeadingStatus::Rejected
        } else if observing {
            StanceHeadingStatus::Observing
        } else {
            StanceHeadingStatus::Idle
        };
        self.finish(out)
    }

    /// Contact starvation: see `StanceHeadingConfig::starvation_secs`.
    fn track_starvation(&mut self, at: f64, planted: &[bool; 2], out: &mut StanceHeadingOutput) {
        let period = f64::from(self.config.starvation_secs);
        let translating = self.head_speed > self.config.starvation_head_speed_mps;
        for foot in 0..2 {
            if planted[foot] || !translating || period.is_nan() || period <= 0.0 {
                self.starved_since[foot] = None;
                continue;
            }
            let since = *self.starved_since[foot].get_or_insert(at);
            if at - since >= period {
                self.starved_since[foot] = Some(at);
                let step = self.config.starvation_inflation_degrees;
                self.inflate_segments(&[THIGH[foot], CALF[foot]], step, &mut out.feet[foot]);
            }
        }
    }

    /// Updates the running NIS-per-dof average of a foot and returns it.
    fn track_nis(&self, diagnostics: &mut FootHeadingDiagnostics, nis: f64) -> f64 {
        let weight = f64::from(self.config.nis_average_weight).clamp(0.0, 1.0);
        let sample = if nis.is_finite() {
            (0.5 * nis).clamp(0.0, 10.0)
        } else {
            10.0
        };
        let old = f64::from(diagnostics.nis_average);
        let old = if old.is_finite() { old } else { 0.0 };
        let average = old + (sample - old) * weight;
        diagnostics.nis_average = average as f32;
        average
    }

    /// Bounded covariance inflation of the segments that move this foot's ankle and that
    /// the displacement measurement estimates.
    fn inflate_chain(&mut self, foot: usize, diagnostics: &mut FootHeadingDiagnostics) {
        let step = self.config.inflation_step_degrees;
        let mut segments = [0; 4];
        let mut count = 0;
        for &i in &FOOT_CHAIN[foot] {
            if self.displacement_observes(i) {
                segments[count] = i;
                count += 1;
            }
        }
        self.inflate_segments(&segments[..count], step, diagnostics);
    }

    fn inflate_segments(
        &mut self,
        segments: &[usize],
        step_degrees: f32,
        diagnostics: &mut FootHeadingDiagnostics,
    ) {
        let step = f64::from(step_degrees).to_radians();
        let cap = f64::from(self.config.inflation_max_sigma_degrees).to_radians();
        if !(step.is_finite() && cap.is_finite()) || step <= 0.0 || cap <= 0.0 {
            return;
        }
        let (variance, cap) = (step * step, cap * cap);
        let mut changed = false;
        for &i in segments {
            let current = self.covariance[i][i];
            if current < cap {
                self.covariance[i][i] = (current + variance).min(cap);
                changed = true;
            }
        }
        if changed {
            diagnostics.inflations = diagnostics.inflations.saturating_add(1);
        }
    }

    fn initial_covariance(&self) -> Matrix6 {
        let sigma = f64::from(self.config.initial_sigma_degrees).to_radians();
        let variance = if sigma.is_finite() {
            sigma * sigma
        } else {
            0.0
        };
        let mut p = [[0.0; N]; N];
        for i in 0..N {
            p[i][i] = variance;
        }
        p
    }

    fn drop_keyframes(&mut self) {
        self.keyframes = [None; 2];
    }

    fn finish(&mut self, out: StanceHeadingOutput) -> StanceHeadingOutput {
        self.diagnostics = out;
        self.publish_state();
        self.diagnostics
    }

    fn publish_state(&mut self) {
        self.diagnostics.corrections = self.corrections();
        self.diagnostics.sigma = self.sigma();
        self.diagnostics.correction_limited =
            std::array::from_fn(|i| self.corrections[i].abs() >= self.correction_bound(i) - 1.0e-9);
        for foot in 0..2 {
            self.diagnostics.feet[foot].keyframe_active = self.keyframes[foot].is_some();
        }
    }

    fn track_angular_speed(
        &mut self,
        dt: Option<f64>,
        rotations: &[Quat; N],
        segment_ok: &[bool; N],
        c_input: &[f64; N],
    ) {
        let tau = f64::from(self.config.angular_speed_tau_secs);
        for i in 0..N {
            if !segment_ok[i] {
                self.previous_raw[i] = None;
                continue;
            }
            // Remove the filter's own correction so correction steps do not count as motion.
            let raw = (Quat::from_rotation_y(-c_input[i] as f32) * rotations[i]).normalize();
            if let (Some(previous), Some(dt)) = (self.previous_raw[i], dt)
                && dt > 0.0
            {
                let rate = f64::from(rotation_angle(raw * previous.conjugate())) / dt;
                let alpha = if tau > 0.0 {
                    1.0 - (-dt / tau).exp()
                } else {
                    1.0
                };
                let old = f64::from(self.angular_speed[i]);
                let smoothed = old + (rate - old) * alpha;
                self.angular_speed[i] = if smoothed.is_finite() {
                    smoothed.clamp(0.0, 100.0) as f32
                } else {
                    0.0
                };
            }
            self.previous_raw[i] = Some(raw);
        }
    }

    fn predict(&mut self, dt: f64, supplied_speed: &[f32; N]) {
        let base = f64::from(self.config.process_sigma_degrees).to_radians();
        let kappa = f64::from(self.config.process_rate_coefficient);
        let cap = f64::from(self.config.max_sigma_degrees)
            .to_radians()
            .powi(2);
        for i in 0..N {
            let supplied = if supplied_speed[i].is_finite() {
                f64::from(supplied_speed[i]).clamp(0.0, 100.0)
            } else {
                0.0
            };
            let omega = supplied.max(f64::from(self.angular_speed[i]));
            let q = base * base + (kappa * omega).powi(2);
            if q.is_finite() && q > 0.0 {
                self.covariance[i][i] += q * dt;
            }
        }
        for i in 0..N {
            let variance = self.covariance[i][i];
            if variance > cap && cap > 0.0 {
                // Congruence scaling of row and column keeps P positive semi-definite.
                let factor = (cap / variance).sqrt();
                for j in 0..N {
                    self.covariance[i][j] *= factor;
                    self.covariance[j][i] *= factor;
                }
            }
        }
        self.sanitize_covariance();
    }

    fn sanitize_covariance(&mut self) {
        let finite = self
            .covariance
            .iter()
            .flatten()
            .all(|value| value.is_finite());
        let diagonal_ok = (0..N).all(|i| self.covariance[i][i] >= 0.0);
        if !finite || !diagonal_ok {
            self.covariance = self.initial_covariance();
        }
    }

    /// Returns `None` when the increment is implausible (tracking discontinuity).
    fn displacement_update(
        &mut self,
        input: &StanceHeadingInput,
        increment: &Increment,
        c_input: &[f64; N],
    ) -> Option<(UpdateOutcome, f32)> {
        let Increment {
            key,
            head_rotation,
            calf_rotation,
            calf,
            delta,
            tau,
        } = *increment;
        let mut displacement =
            (input.head_position - key.head_position) + (input.chain.head_lever - key.head_lever);
        for lever in delta {
            displacement += *lever;
        }
        if !displacement.is_finite() || displacement.length() > MAX_PLAUSIBLE_INCREMENT_M {
            return None;
        }
        let z = [-f64::from(displacement.x), -f64::from(displacement.z)];
        let mut h = [[0.0; N]; 2];
        let mut lever_energy = 0.0;
        let mut torso_energy = 0.0;
        let mut anisotropic = [[0.0; 2]; 2];
        for i in 0..N {
            let (x, z) = (f64::from(delta[i].x), f64::from(delta[i].z));
            if self.displacement_observes(i) {
                // (ŷ × Δlever)_xz = (Δlever.z, −Δlever.x)
                h[0][i] = z;
                h[1][i] = -x;
            }
            if i == CHEST || i == HIP {
                torso_energy += x * x + z * z;
            } else {
                lever_energy += x * x + z * z;
            }
            anisotropic[0][0] += x * x;
            anisotropic[0][1] += x * z;
            anisotropic[1][1] += z * z;
        }
        // Innovation relative to the current state: updates already applied in this step
        // changed the corrections after the chain was computed.
        let mut nu = z;
        for k in 0..2 {
            for i in 0..N {
                nu[k] -= h[k][i] * (self.corrections[i] - c_input[i]);
            }
        }
        let config = &self.config;
        let head_turn = f64::from(rotation_angle(
            key.head_rotation.conjugate() * head_rotation,
        ));
        let hmd = f64::from(config.hmd_sigma_m);
        let neck = f64::from(config.neck_sigma_m_per_rad) * head_turn;
        let creep = f64::from(config.creep_sigma_mps) * f64::from(tau);
        let rho = f64::from(config.motion_noise_ratio);
        let rho_torso = f64::from(config.torso_motion_noise_ratio);
        // Calf rotation about vertical between the keyframes, compared under one set of
        // corrections.
        let key_calf = Quat::from_rotation_y((c_input[calf] - key.corrections[calf]) as f32)
            * key.calf_rotation;
        let calf_turn = f64::from(vertical_rotation(calf_rotation, key_calf));
        let pivot = f64::from(config.pivot_sigma_m_per_rad) * calf_turn;
        let isotropic = 2.0 * hmd * hmd
            + neck * neck
            + creep * creep
            + pivot * pivot
            + rho * rho * lever_energy
            + rho_torso * rho_torso * torso_energy;
        let epsilon = f64::from(config.length_uncertainty).powi(2);
        let r = [
            [
                isotropic + epsilon * anisotropic[0][0],
                epsilon * anisotropic[0][1],
            ],
            [
                epsilon * anisotropic[0][1],
                isotropic + epsilon * anisotropic[1][1],
            ],
        ];
        let innovation = nu[0].hypot(nu[1]) as f32;
        let gate = f64::from(config.gate_chi2);
        Some((self.update(nu, &h, r, gate), innovation))
    }

    fn apply_priors(
        &mut self,
        at: f64,
        rotations: &[Quat; N],
        segment_ok: &[bool; N],
        c_input: &[f64; N],
        out: &mut StanceHeadingOutput,
    ) -> bool {
        let headings: [Option<f32>; N] = std::array::from_fn(|i| {
            if segment_ok[i] {
                lateral_heading(rotations[i])
            } else {
                None
            }
        });
        let applied = |i: usize, filter: &Self| filter.corrections[i] - c_input[i];
        let mut accepted = false;

        let knee = self.config.knee_prior;
        for foot in 0..2 {
            let (thigh, calf) = (THIGH[foot], CALF[foot]);
            if !self.config.enable_knee_prior || !segment_ok[thigh] || !segment_ok[calf] {
                continue;
            }
            let flexion = (rotations[thigh] * Vec3::Y)
                .dot(rotations[calf] * Vec3::Y)
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees();
            let window = f64::from(knee.change_window_secs.max(0.0));
            let excursion = self.flex[foot].excursion(at, window, flexion);
            self.flex[foot].push(at, flexion, window / 48.0);
            let (Some(psi_thigh), Some(psi_calf)) = (headings[thigh], headings[calf]) else {
                continue;
            };
            let gated = (knee.min_flexion_degrees..=knee.max_flexion_degrees).contains(&flexion)
                && excursion >= knee.min_flexion_change_degrees;
            if !gated {
                continue;
            }
            let residual =
                f64::from(wrap(psi_calf - psi_thigh)) + applied(calf, self) - applied(thigh, self);
            let mut h = [0.0; N];
            h[calf] = 1.0;
            h[thigh] = -1.0;
            let spec = PriorSpec {
                slot: foot,
                deadband_degrees: knee.deadband_degrees,
                sigma_degrees: knee.sigma_degrees,
                rate_hz: knee.rate_hz,
            };
            if self.prior_update(at, residual, h, spec, &mut out.priors.knee[foot]) {
                accepted = true;
            }
        }

        let upright =
            |i: usize, max_tilt: f32| (rotations[i] * Vec3::Y).y >= max_tilt.to_radians().cos();
        let pelvis = self.config.pelvis_prior;
        if self.config.enable_pelvis_prior
            && [HIP, THIGH[0], THIGH[1]]
                .iter()
                .all(|&i| segment_ok[i] && upright(i, pelvis.max_tilt_degrees))
            && let (Some(psi_hip), Some(psi_left), Some(psi_right)) =
                (headings[HIP], headings[THIGH[0]], headings[THIGH[1]])
        {
            let mean = psi_left + 0.5 * wrap(psi_right - psi_left);
            let residual = f64::from(wrap(psi_hip - mean)) + applied(HIP, self)
                - 0.5 * (applied(THIGH[0], self) + applied(THIGH[1], self));
            let mut h = [0.0; N];
            h[HIP] = 1.0;
            h[THIGH[0]] = -0.5;
            h[THIGH[1]] = -0.5;
            let spec = PriorSpec::upright(2, &pelvis);
            if self.prior_update(at, residual, h, spec, &mut out.priors.pelvis) {
                accepted = true;
            }
        }

        let chest = self.config.chest_prior;
        if self.config.enable_chest_prior
            && [CHEST, HIP]
                .iter()
                .all(|&i| segment_ok[i] && upright(i, chest.max_tilt_degrees))
            && let (Some(psi_chest), Some(psi_hip)) = (headings[CHEST], headings[HIP])
        {
            let residual =
                f64::from(wrap(psi_chest - psi_hip)) + applied(CHEST, self) - applied(HIP, self);
            let mut h = [0.0; N];
            h[CHEST] = 1.0;
            h[HIP] = -1.0;
            let spec = PriorSpec::upright(3, &chest);
            if self.prior_update(at, residual, h, spec, &mut out.priors.chest) {
                accepted = true;
            }
        }
        accepted
    }

    /// Deadbanded pseudo-measurement `−r_eff = H·δ + n`. `residual` is radians and already
    /// reflects corrections applied earlier in this step.
    fn prior_update(
        &mut self,
        at: f64,
        residual: f64,
        h_row: [f64; N],
        spec: PriorSpec,
        diagnostics: &mut PriorDiagnostic,
    ) -> bool {
        let PriorSpec {
            slot,
            deadband_degrees,
            sigma_degrees,
            rate_hz,
        } = spec;
        diagnostics.active = true;
        diagnostics.last_residual_degrees = finite_f32(residual.to_degrees() as f32);
        let deadband = f64::from(deadband_degrees.max(0.0)).to_radians();
        let excess = residual.abs() - deadband;
        if excess.is_nan() || excess <= 0.0 {
            return false;
        }
        let period = if rate_hz > 0.0 {
            1.0 / f64::from(rate_hz)
        } else {
            f64::INFINITY
        };
        let due = self.prior_last_at[slot].is_none_or(|last| at < last || at - last >= period);
        if !due {
            return false;
        }
        self.prior_last_at[slot] = Some(at);
        let sigma = f64::from(sigma_degrees).to_radians();
        let z = -residual.signum() * excess;
        let h = [h_row, [0.0; N]];
        let r = [[sigma * sigma, 0.0], [0.0, 1.0]];
        let gate = f64::from(self.config.prior_gate_chi2);
        match self.update([z, 0.0], &h, r, gate) {
            UpdateOutcome::Accepted { .. } => {
                diagnostics.updates = diagnostics.updates.saturating_add(1);
                true
            }
            UpdateOutcome::Rejected { .. } | UpdateOutcome::Degenerate => false,
        }
    }

    /// Gated, step-clamped Joseph-form update with innovation `nu` (already relative to the
    /// current state).
    fn update(
        &mut self,
        nu: [f64; 2],
        h: &[[f64; N]; 2],
        r: [[f64; 2]; 2],
        gate: f64,
    ) -> UpdateOutcome {
        let p = &self.covariance;
        let mut pht = [[0.0; 2]; N];
        for i in 0..N {
            for k in 0..2 {
                pht[i][k] = (0..N).map(|j| p[i][j] * h[k][j]).sum();
            }
        }
        let mut s = [[0.0; 2]; 2];
        for a in 0..2 {
            for b in 0..2 {
                s[a][b] = (0..N).map(|i| h[a][i] * pht[i][b]).sum::<f64>() + r[a][b];
            }
        }
        let s01 = 0.5 * (s[0][1] + s[1][0]);
        let det = s[0][0] * s[1][1] - s01 * s01;
        if !det.is_finite() || det <= 1.0e-24 || s[0][0] <= 0.0 || s[1][1] <= 0.0 {
            return UpdateOutcome::Degenerate;
        }
        let inverse = [[s[1][1] / det, -s01 / det], [-s01 / det, s[0][0] / det]];
        let nis = nu[0] * nu[0] * inverse[0][0]
            + 2.0 * nu[0] * nu[1] * inverse[0][1]
            + nu[1] * nu[1] * inverse[1][1];
        if !nis.is_finite() || !nu[0].is_finite() || !nu[1].is_finite() {
            return UpdateOutcome::Degenerate;
        }
        if gate.is_nan() || nis > gate {
            return UpdateOutcome::Rejected { nis };
        }
        let mut gain = [[0.0; 2]; N];
        for i in 0..N {
            for b in 0..2 {
                gain[i][b] = pht[i][0] * inverse[0][b] + pht[i][1] * inverse[1][b];
            }
        }
        let mut step: [f64; N] = std::array::from_fn(|i| gain[i][0] * nu[0] + gain[i][1] * nu[1]);
        let largest = step.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let limit = f64::from(self.config.max_step_degrees).to_radians();
        if largest > limit && limit > 0.0 {
            // Scale the whole step (and the gain, so the Joseph covariance stays consistent).
            let scale = limit / largest;
            for i in 0..N {
                step[i] *= scale;
                gain[i][0] *= scale;
                gain[i][1] *= scale;
            }
        }
        if step.iter().any(|v| !v.is_finite()) {
            return UpdateOutcome::Degenerate;
        }
        // Joseph form: P⁺ = (I − KH) P (I − KH)ᵀ + K R Kᵀ
        let mut a = [[0.0; N]; N];
        for i in 0..N {
            for j in 0..N {
                let identity = if i == j { 1.0 } else { 0.0 };
                a[i][j] = identity - gain[i][0] * h[0][j] - gain[i][1] * h[1][j];
            }
        }
        let mut ap = [[0.0; N]; N];
        for i in 0..N {
            for j in 0..N {
                ap[i][j] = (0..N).map(|k| a[i][k] * p[k][j]).sum();
            }
        }
        let mut next = [[0.0; N]; N];
        for i in 0..N {
            for j in 0..N {
                let propagated: f64 = (0..N).map(|k| ap[i][k] * a[j][k]).sum();
                let mut noise = 0.0;
                for x in 0..2 {
                    for y in 0..2 {
                        noise += gain[i][x] * r[x][y] * gain[j][y];
                    }
                }
                next[i][j] = propagated + noise;
            }
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let mean = 0.5 * (next[i][j] + next[j][i]);
                next[i][j] = mean;
                next[j][i] = mean;
            }
        }
        if next.iter().flatten().any(|v| !v.is_finite()) {
            return UpdateOutcome::Degenerate;
        }
        self.covariance = next;
        self.sanitize_covariance();
        for i in 0..N {
            self.corrections[i] = wrap64(self.corrections[i] + step[i]);
        }
        self.apply_bounds();
        UpdateOutcome::Accepted { nis }
    }
}

/// Speed (m/s) to add to each foot's contact speed thresholds while the filter is uncertain:
/// `sin(min(sigma_gain·σ_chain, max_angle))·|v_head,xz|`, where `σ_chain` is the largest 1σ of
/// the foot's hip, thigh and calf (`Role::CORE` indices) and `v_head,xz` the horizontal headset
/// velocity. A yaw error `δ` makes a stationary ankle appear to move at about `sin δ` times
/// the speed at which the chain's levers swing, which while walking is about the body speed.
/// The chest is excluded: its lever is nearly vertical while upright and its yaw is weakly
/// observed, so it would widen the gates without reason. Nonfinite input gives zero.
pub fn contact_speed_allowance(
    sigma: [f32; 6],
    head_velocity: Vec3,
    sigma_gain: f32,
    max_angle_degrees: f32,
) -> [f32; 2] {
    let speed = head_velocity.x.hypot(head_velocity.z);
    let max_angle = max_angle_degrees.clamp(0.0, 90.0).to_radians();
    if !speed.is_finite() || !sigma_gain.is_finite() || !max_angle.is_finite() {
        return [0.0; 2];
    }
    [0, 1].map(|foot| {
        let chain_sigma = [HIP, THIGH[foot], CALF[foot]]
            .iter()
            .map(|&i| sigma[i])
            .fold(0.0_f32, |m, v| if v.is_finite() { m.max(v) } else { m });
        let angle = (sigma_gain.max(0.0) * chain_sigma).min(max_angle);
        let allowance = angle.sin() * speed;
        if allowance.is_finite() {
            allowance.max(0.0)
        } else {
            0.0
        }
    })
}

/// `Δlever_i = lever_i(now) − Ry(c_i(now) − c_i(key))·lever_i(key)` for the foot's chain, so both
/// ends are compared under the current corrections, and the excitation `max_i |Δlever_i|_xz`.
fn lever_increments(
    key: &Keyframe,
    levers: &[Vec3; N],
    segments: &[usize; 4],
    corrections: &[f64; N],
) -> ([Vec3; N], f32) {
    let mut delta = [Vec3::ZERO; N];
    let mut excitation = 0.0_f32;
    for &i in segments {
        let turn = (corrections[i] - key.corrections[i]) as f32;
        delta[i] = levers[i] - Quat::from_rotation_y(turn) * key.levers[i];
        excitation = excitation.max(delta[i].x.hypot(delta[i].z));
    }
    (delta, excitation)
}

/// Rotation angle of a unit quaternion in `[0, π]`.
pub(crate) fn rotation_angle(q: Quat) -> f32 {
    let vector = Vec3::new(q.x, q.y, q.z).length();
    let angle = 2.0 * vector.atan2(q.w.abs());
    if angle.is_finite() { angle } else { 0.0 }
}

fn wrap(angle: f32) -> f32 {
    wrap64(f64::from(angle)) as f32
}

fn wrap64(angle: f64) -> f64 {
    if !angle.is_finite() {
        return 0.0;
    }
    let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI {
        wrapped + 2.0 * PI
    } else {
        wrapped
    }
}

fn finite_f32(value: f32) -> f32 {
    if value.is_finite() { value } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chain::{ChainDimensions, ChainPose, solve_chain, yaw_sensitivity};

    /// Rotations of a moving leg chain (legs and torso change between a and b).
    fn pose(phase: f32) -> [Quat; N] {
        [
            Quat::from_rotation_x(-0.3 * phase),
            Quat::from_rotation_x(-0.2 * phase) * Quat::from_rotation_y(0.05 * phase),
            Quat::from_rotation_x(0.6 * phase),
            Quat::from_rotation_x(0.1 * phase),
            Quat::from_rotation_x(-0.4 * phase),
            Quat::from_rotation_x(-0.1 * phase),
        ]
    }

    fn corrected(rotations: [Quat; N], corrections: [f64; N]) -> [Quat; N] {
        std::array::from_fn(|i| Quat::from_rotation_y(corrections[i] as f32) * rotations[i])
    }

    /// The true body has zero yaw error, so with corrections `c` applied the remaining error is
    /// `δ = −c(now)`. With the keyframe re-rotated by `c(now) − c(key)`, the increment must obey
    /// `z ≈ H·(−c(now))`. Comparing raw keyframe levers instead would add a spurious
    /// `(c(now) − c(key))·ŷ × lever(key)` term proportional to the whole lever.
    #[test]
    fn keyframe_levers_are_compared_under_current_corrections() {
        let dims = ChainDimensions::default();
        let c_key = [0.0, 0.03, 0.02, 0.0, -0.01, 0.0];
        let c_now = [0.01, 0.05, 0.06, 0.0, -0.05, 0.0];
        let truth = [pose(0.8), pose(1.6)];
        // Head positions chosen so the true left ankle stays at the same point.
        let true_chain = |rotations: [Quat; N]| {
            solve_chain(
                &dims,
                &ChainPose {
                    head_position: Vec3::ZERO,
                    head_rotation: Quat::IDENTITY,
                    rotations,
                },
            )
        };
        let heads = truth.map(|rotations| -true_chain(rotations).ankles[0]);
        let measured = |k: usize, c: [f64; N]| {
            solve_chain(
                &dims,
                &ChainPose {
                    head_position: heads[k],
                    head_rotation: Quat::IDENTITY,
                    rotations: corrected(truth[k], c),
                },
            )
        };
        let (a, b) = (measured(0, c_key), measured(1, c_now));
        let key = Keyframe {
            at_secs: 0.0,
            head_position: heads[0],
            head_lever: a.head_lever,
            head_rotation: Quat::IDENTITY,
            calf_rotation: Quat::IDENTITY,
            levers: a.levers[0],
            corrections: c_key,
        };
        let (delta, excitation) = lever_increments(&key, &b.levers[0], &FOOT_CHAIN[0], &c_now);
        assert!(excitation > 0.1);
        let mut displacement = (heads[1] - heads[0]) + (b.head_lever - a.head_lever);
        let mut predicted = Vec3::ZERO;
        for i in 0..N {
            displacement += delta[i];
            predicted += yaw_sensitivity(delta[i]) * (-c_now[i] as f32);
        }
        let z = -Vec3::new(displacement.x, 0.0, displacement.z);
        // Second-order terms of ~3° corrections on ~0.2 m increments: well below 1 mm.
        assert!(
            (z - predicted).length() < 1.0e-3,
            "z {z:?} vs H·δ {predicted:?}"
        );

        // Without re-rotation the increment would be off by roughly Δc·|lever| (≈ 1 cm here).
        let mut naive = (heads[1] - heads[0]) + (b.head_lever - a.head_lever);
        for &i in &FOOT_CHAIN[0] {
            naive += b.levers[0][i] - a.levers[0][i];
        }
        let naive_z = -Vec3::new(naive.x, 0.0, naive.z);
        assert!((naive_z - predicted).length() > 5.0e-3);
    }

    #[test]
    fn wrap_stays_in_half_open_interval() {
        for value in [-10.0, -PI, -3.0, 0.0, 3.0, PI, 7.0, 1.0e9] {
            let wrapped = wrap64(value);
            assert!(
                wrapped > -PI - 1e-12 && wrapped <= PI + 1e-12,
                "{value} -> {wrapped}"
            );
            assert!(((wrapped - value) / (2.0 * PI)).fract().abs() < 1e-6 || value.abs() > 1e6);
        }
        assert_eq!(wrap64(f64::NAN), 0.0);
    }
}
