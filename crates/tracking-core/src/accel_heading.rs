//! Research ("shadow") estimator: the heading (yaw) error of one torso IMU,
//! observed from the agreement between its world-frame linear acceleration and
//! the acceleration of an absolutely tracked reference point (the headset, or
//! better the neck pivot inferred from it). Diagnostics only: nothing in this
//! module modifies tracking state, and a reported candidate is a hypothesis
//! for the caller to weigh. It has been exercised on synthetic data only; see
//! `docs/research/acceleration-heading.md`.
//!
//! # Model
//!
//! ```text
//! world_accel(t) = S(t) · C · accel_body(t)
//! ```
//!
//! `S` is the caller's heading-corrected sensor-to-room rotation, `accel_body`
//! the firmware's gravity-removed linear acceleration in raw IMU axes, and `C`
//! an unknown fixed rotation between the raw accelerometer axes and the axes
//! the quaternion refers to. SlimeVR firmware 0.7.2 rotates the quaternion but
//! not the acceleration by the board mounting offset (0.7.4 rotates both, so
//! `C` is the identity there). `C` is searched among the 24 axis-aligned
//! proper rotations ([`frame_candidate`]).
//!
//! The vertical component `e_y · S · C · a = (Sᵀe_y) · (C·a)` does not depend on
//! the yaw of `S`, so it identifies `C` and the time skew without circularity.
//! Only then is the horizontal component compared with the reference, and the
//! yaw that best aligns the two horizontal acceleration histories (2-D
//! Procrustes) is the heading correction candidate.
//!
//! # Explicit assumptions
//!
//! 1. **Rigid translation.** Within 0.5–6 Hz the reference point and the
//!    sensor share their translational acceleration (the torso translates with
//!    the head while stepping, walking, bobbing or dancing). Head rotation about
//!    the neck violates this for the eye point; pass the neck pivot from the
//!    chain model instead, and the angular-speed gates bound what remains.
//!    Torso flexion makes the head move more than the chest (a gain mismatch).
//! 2. **Tilt is right, only yaw is wrong.** The pitch/roll of `S` is correct
//!    (6-axis fusion observes it from gravity); the yaw error is constant over
//!    the analysis window.
//! 3. **Fixed axis-aligned `C`.** `C` is one of the 24 candidates and constant
//!    for the session. A sign-inverted acceleration convention (an improper
//!    transform) is not representable and shows up as poor agreement.
//! 4. **Gravity is removed.** Accelerometer bias and gravity-removal error are
//!    slow in the room frame and are removed by the high-pass filter; this
//!    fails when the torso rotates quickly enough to move them into the band.
//! 5. **Constant time skew** inside the lag search range, the same for the
//!    vertical and horizontal components. Receipt timestamps are used; jitter
//!    must be small compared with the signal period.
//! 6. **One room frame.** The reference and `S` use the same gravity-aligned
//!    room frame; a recentre or tracking-space rotation requires [`reset`].
//!
//! [`reset`]: AccelHeadingEstimator::reset

use crate::CoreError;
use glam::{DMat3, DQuat, DVec3, Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub mod synthetic;

/// Number of axis-aligned proper rotations considered for `C`.
pub const FRAME_CANDIDATE_COUNT: usize = 24;

/// A backwards timestamp jump larger than this restarts both streams;
/// smaller ones are out-of-order samples and are rejected.
const CLOCK_RESTART_SECS: f64 = 1.0;
const MAX_POSITION_M: f64 = 1.0e4;
const MAX_VELOCITY_MPS: f64 = 1.0e3;
const MAX_RAW_ACCEL_MPS2: f64 = 1.0e4;
const MAX_ANGULAR_SPEED_RAD_PER_SEC: f64 = 1.0e3;
const TINY: f64 = 1.0e-12;

type Mat3 = [[f64; 3]; 3];

/// The 24 signed permutation matrices with determinant +1 (row-major), in a
/// fixed order with the identity first.
fn candidate_matrices() -> [Mat3; FRAME_CANDIDATE_COUNT] {
    const PERMUTATIONS: [[usize; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut out = [[[0.0; 3]; 3]; FRAME_CANDIDATE_COUNT];
    let mut count = 0;
    for permutation in PERMUTATIONS {
        for signs in 0..8_u8 {
            let mut m = [[0.0; 3]; 3];
            for (row, values) in m.iter_mut().enumerate() {
                values[permutation[row]] = if signs & (1 << row) == 0 { 1.0 } else { -1.0 };
            }
            if determinant(&m) > 0.0 && count < FRAME_CANDIDATE_COUNT {
                out[count] = m;
                count += 1;
            }
        }
    }
    out
}

fn determinant(m: &Mat3) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

fn matrix_quat(m: &Mat3) -> DQuat {
    let column = |c: usize| DVec3::new(m[0][c], m[1][c], m[2][c]);
    DQuat::from_mat3(&DMat3::from_cols(column(0), column(1), column(2))).normalize()
}

fn flatten(m: &Mat3) -> [f64; 9] {
    let mut out = [0.0; 9];
    for (index, value) in out.iter_mut().enumerate() {
        *value = m[index / 3][index % 3];
    }
    out
}

/// Candidate `C` number `index` (0 is the identity): the rotation from raw
/// accelerometer axes to the axes the firmware quaternion refers to.
pub fn frame_candidate(index: usize) -> Option<Quat> {
    candidate_matrices()
        .get(index)
        .map(|m| matrix_quat(m).as_quat())
}

/// The candidate within 2 degrees of `rotation`, if any.
pub fn frame_candidate_index(rotation: Quat) -> Option<usize> {
    let rotation = rotation.as_dquat();
    let length = rotation.length();
    if !rotation.is_finite() || !(0.5..=1.5).contains(&length) {
        return None;
    }
    let rotation = rotation / length;
    candidate_matrices()
        .iter()
        .position(|m| quat_angle(matrix_quat(m), rotation) < 2.0_f64.to_radians())
}

/// Candidates that leave `axis` (raw IMU axes) unchanged: the 4 rotations
/// about it when it is a coordinate axis. Boards whose firmware mounting
/// option rotates about the chip Z axis can restrict
/// `allowed_frame_candidates` to `frame_candidates_fixing_axis(Vec3::Z)`.
pub fn frame_candidates_fixing_axis(axis: Vec3) -> Vec<usize> {
    let Some(axis) = axis.as_dvec3().try_normalize() else {
        return Vec::new();
    };
    candidate_matrices()
        .iter()
        .enumerate()
        .filter(|(_, m)| (matrix_quat(m) * axis - axis).length() < 1.0e-6)
        .map(|(index, _)| index)
        .collect()
}

/// Tuning of [`AccelHeadingEstimator`]. Defaults are engineering starting
/// points for a torso IMU against a headset/neck reference, not tuned values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AccelHeadingConfig {
    /// Sliding window over which every statistic is accumulated.
    pub window_secs: f64,
    /// History before the window used only to settle the causal filters, and
    /// discarded again after every stream gap.
    pub settle_secs: f64,
    /// Uniform analysis grid. The coarse lag step is one grid interval.
    pub grid_rate_hz: f64,
    /// Causal 2nd-order Butterworth high-pass (bias and gravity-removal error).
    pub highpass_hz: f64,
    /// Causal 2nd-order Butterworth low-pass.
    pub lowpass_hz: f64,
    /// Lag = IMU receipt time minus reference time of the same physical
    /// instant (positive: IMU samples are stamped later). Searched range.
    pub min_lag_secs: f64,
    pub max_lag_secs: f64,
    /// A new coarse lag replaces the latched one only if its vertical
    /// agreement is higher by more than this (hysteresis).
    pub lag_switch_margin: f64,
    /// Samples further apart than this are not interpolated across.
    pub max_gap_secs: f64,
    /// The newest sample of each stream must be at most this old at `report`.
    pub max_input_age_secs: f64,
    /// Reference angular-speed gate (head rotation about the neck).
    pub max_reference_angular_speed_rad_per_sec: f64,
    /// IMU angular-speed gate.
    pub max_imu_angular_speed_rad_per_sec: f64,
    /// Raw IMU and low-passed reference accelerations above this are implausible.
    pub max_acceleration_mps2: f64,
    /// Reference speeds above this are a tracking discontinuity (teleport).
    pub max_reference_speed_mps: f64,
    /// Every gated or uncovered grid sample also invalidates this much of its
    /// neighbourhood, because the band-pass spreads the disturbance.
    pub gate_guard_secs: f64,
    /// Differentiate reference velocity when supplied, else position.
    pub prefer_reference_velocity: bool,
    /// Firmware 0.7.2 pairs each rotation with the previous sample's
    /// acceleration: rotate acceleration `k` with rotation `k - 1`.
    pub accel_lags_rotation_by_one_sample: bool,
    /// Valid grid samples times `2·(lowpass − highpass)/grid_rate`: the number
    /// of samples at the band's Nyquist rate. A heuristic, not a test statistic.
    pub min_effective_samples: f64,
    /// Minimum band-passed vertical reference RMS (frame, lag and rigidity check).
    pub min_vertical_rms_mps2: f64,
    /// Minimum band-passed horizontal reference RMS (heading).
    pub min_horizontal_rms_mps2: f64,
    /// Minimum concordance `2Σxy/(Σx²+Σy²)` of vertical accelerations.
    pub min_vertical_agreement: f64,
    /// Required concordance margin over every distinguishable alternative frame.
    pub min_frame_margin: f64,
    /// Restrict the search to these candidate indices (empty: all 24).
    pub allowed_frame_candidates: Vec<usize>,
    /// True when the reference's horizontal acceleration is independent of this
    /// sensor's heading (headset for a torso sensor). Set false for a
    /// forward-kinematics reference built from this sensor's own orientation:
    /// then only the frame and lag are identified.
    pub horizontal_reference: bool,
    /// Minimum rotation-maximised normalised horizontal correlation.
    pub min_horizontal_correlation: f64,
    /// Candidates with a larger reported sigma are withheld.
    pub max_yaw_sigma_radians: f64,
    /// Asserted (not estimated) floor for unmodelled systematic error, added in
    /// quadrature to the data-derived sigma.
    pub yaw_sigma_floor_radians: f64,
    /// Accepted range of reference/IMU horizontal regression gain.
    pub min_horizontal_gain: f64,
    pub max_horizontal_gain: f64,
    /// Larger fitted yaw corrections are reported as `ExcessiveCorrection`.
    pub max_candidate_yaw_radians: f64,
    /// Ring-buffer capacities (samples). Memory is bounded by these.
    pub reference_capacity: usize,
    pub imu_capacity: usize,
}

impl Default for AccelHeadingConfig {
    fn default() -> Self {
        Self {
            window_secs: 6.0,
            settle_secs: 1.0,
            grid_rate_hz: 100.0,
            highpass_hz: 0.5,
            lowpass_hz: 6.0,
            min_lag_secs: -0.02,
            max_lag_secs: 0.08,
            lag_switch_margin: 0.002,
            max_gap_secs: 0.1,
            max_input_age_secs: 0.5,
            max_reference_angular_speed_rad_per_sec: 60.0_f64.to_radians(),
            max_imu_angular_speed_rad_per_sec: 90.0_f64.to_radians(),
            max_acceleration_mps2: 30.0,
            max_reference_speed_mps: 6.0,
            gate_guard_secs: 0.2,
            prefer_reference_velocity: true,
            accel_lags_rotation_by_one_sample: true,
            min_effective_samples: 30.0,
            min_vertical_rms_mps2: 0.3,
            min_horizontal_rms_mps2: 0.3,
            min_vertical_agreement: 0.6,
            min_frame_margin: 0.05,
            allowed_frame_candidates: Vec::new(),
            horizontal_reference: true,
            min_horizontal_correlation: 0.6,
            max_yaw_sigma_radians: 6.0_f64.to_radians(),
            yaw_sigma_floor_radians: 1.0_f64.to_radians(),
            min_horizontal_gain: 0.5,
            max_horizontal_gain: 2.0,
            max_candidate_yaw_radians: 60.0_f64.to_radians(),
            reference_capacity: 4096,
            imu_capacity: 2048,
        }
    }
}

impl AccelHeadingConfig {
    fn validate(&self) -> Result<(), CoreError> {
        let finite = [
            self.window_secs,
            self.settle_secs,
            self.grid_rate_hz,
            self.highpass_hz,
            self.lowpass_hz,
            self.min_lag_secs,
            self.max_lag_secs,
            self.lag_switch_margin,
            self.max_gap_secs,
            self.max_input_age_secs,
            self.max_reference_angular_speed_rad_per_sec,
            self.max_imu_angular_speed_rad_per_sec,
            self.max_acceleration_mps2,
            self.max_reference_speed_mps,
            self.gate_guard_secs,
            self.min_effective_samples,
            self.min_vertical_rms_mps2,
            self.min_horizontal_rms_mps2,
            self.min_vertical_agreement,
            self.min_frame_margin,
            self.min_horizontal_correlation,
            self.max_yaw_sigma_radians,
            self.yaw_sigma_floor_radians,
            self.min_horizontal_gain,
            self.max_horizontal_gain,
            self.max_candidate_yaw_radians,
        ];
        let ok = finite.iter().all(|v| v.is_finite())
            && (1.0..=30.0).contains(&self.window_secs)
            && (0.0..=5.0).contains(&self.settle_secs)
            && (20.0..=400.0).contains(&self.grid_rate_hz)
            && self.highpass_hz > 0.0
            && self.lowpass_hz > self.highpass_hz
            && self.lowpass_hz < 0.4 * self.grid_rate_hz
            && self.min_lag_secs <= self.max_lag_secs
            && self.min_lag_secs >= -0.5
            && self.max_lag_secs <= 0.5
            && self.lag_switch_margin >= 0.0
            && self.max_gap_secs > 0.0
            && self.max_gap_secs <= 1.0
            && self.max_input_age_secs > 0.0
            && self.max_reference_angular_speed_rad_per_sec > 0.0
            && self.max_imu_angular_speed_rad_per_sec > 0.0
            && self.max_acceleration_mps2 > 0.0
            && self.max_reference_speed_mps > 0.0
            && (0.0..=2.0).contains(&self.gate_guard_secs)
            && self.min_effective_samples >= 1.0
            && self.min_vertical_rms_mps2 >= 0.0
            && self.min_horizontal_rms_mps2 >= 0.0
            && self.min_vertical_agreement > 0.0
            && self.min_vertical_agreement <= 1.0
            && self.min_frame_margin > 0.0
            && self.min_frame_margin < 1.0
            && (0.0..=1.0).contains(&self.min_horizontal_correlation)
            && self.max_yaw_sigma_radians > 0.0
            && self.yaw_sigma_floor_radians >= 0.0
            && self.min_horizontal_gain > 0.0
            && self.max_horizontal_gain > self.min_horizontal_gain
            && self.max_candidate_yaw_radians > 0.0
            && self.max_candidate_yaw_radians <= PI
            && (16..=65_536).contains(&self.reference_capacity)
            && (16..=65_536).contains(&self.imu_capacity)
            && self.allowed_frame_candidates.len() <= FRAME_CANDIDATE_COUNT
            && self
                .allowed_frame_candidates
                .iter()
                .enumerate()
                .all(|(i, c)| {
                    *c < FRAME_CANDIDATE_COUNT && !self.allowed_frame_candidates[..i].contains(c)
                });
        if ok {
            Ok(())
        } else {
            Err(CoreError(
                "invalid acceleration-heading configuration".into(),
            ))
        }
    }

    /// Seconds of history kept behind the newest sample of each stream.
    fn history_secs(&self) -> f64 {
        self.window_secs
            + self.settle_secs
            + (self.max_lag_secs - self.min_lag_secs)
            + 2.0 * self.max_gap_secs
            + self.gate_guard_secs
            + 4.0 / self.grid_rate_hz
            + self.max_input_age_secs
    }
}

/// Result of pushing one sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SampleOutcome {
    Accepted,
    /// Nonfinite, out of range, or not newer than the previous sample.
    Rejected,
    /// The timestamp jumped back by more than 1 s: both streams and the lag
    /// were cleared (the frame latch and hint are kept), then it was accepted.
    ClockRestarted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccelHeadingStatus {
    /// No data, stale data, a nonfinite evaluation time, or too few valid
    /// paired samples in the window.
    InsufficientData,
    /// The band-passed reference acceleration is too small (vertical: frame,
    /// lag and rigidity cannot be checked; horizontal: no heading information).
    LowExcitation,
    /// No candidate frame explains the vertical acceleration clearly.
    FrameUnidentified,
    /// Several candidates are structurally indistinguishable from the vertical
    /// component alone (the sensor did not tilt enough). See
    /// [`FrameDiagnostics::ambiguous_candidates`] and
    /// [`AccelHeadingEstimator::set_frame_hint`].
    FrameAmbiguous,
    /// The caller's frame hint is contradicted by this sensor's vertical data.
    FrameHintRejected,
    /// Frame and lag identified; heading is not estimated because
    /// `horizontal_reference` is false.
    FrameIdentifiedOnly,
    /// The frame is known but the accelerations do not agree well enough (or
    /// the amplitude ratio is implausible) to support a heading.
    PoorCorrelation,
    /// The fitted yaw exceeds `max_candidate_yaw_radians` (for example a wrong
    /// frame inside an ambiguity group, which acts like a 90/180-degree yaw).
    ExcessiveCorrection,
    /// A heading correction candidate with an uncertainty estimate.
    Candidate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrameSource {
    /// Identified uniquely from this sensor's vertical acceleration.
    Identified,
    /// Supplied by the caller and confirmed by this sensor's vertical acceleration.
    Hint,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HintStatus {
    Absent,
    /// Not yet confirmed or contradicted by enough vertical evidence.
    Pending,
    Verified,
    Rejected,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CandidateScore {
    pub candidate: usize,
    /// Concordance `2Σxy/(Σx²+Σy²)` of band-passed vertical accelerations at
    /// this candidate's best lag; 1 means identical, 0 unrelated.
    pub vertical_agreement: f64,
    pub lag_secs: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrameDiagnostics {
    /// The frame used for the heading fit (latched or verified hint).
    pub candidate: Option<usize>,
    pub source: Option<FrameSource>,
    /// Highest vertical agreement in this window (may differ from `candidate`
    /// inside an ambiguity group).
    pub best_candidate: Option<usize>,
    /// Vertical concordance of `candidate` (else of `best_candidate`).
    pub vertical_agreement: Option<f64>,
    /// Pearson correlation of the same signals (scale-insensitive diagnostic).
    pub vertical_correlation: Option<f64>,
    /// Agreement minus the best agreement of any structurally distinguishable
    /// alternative.
    pub margin: Option<f64>,
    /// Candidates that even noise-free data in this window could not separate
    /// from the anchor frame by `min_frame_margin`.
    pub ambiguous_candidates: Vec<usize>,
    pub hint: Option<usize>,
    pub hint_status: HintStatus,
    pub vertical_reference_rms_mps2: Option<f64>,
    /// RMS angle of the sensor's up direction (in sensor axes) about its mean:
    /// the tilt diversity that separates candidates.
    pub tilt_spread_degrees: Option<f64>,
    /// All allowed candidates, best first.
    pub scores: Vec<CandidateScore>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HorizontalFit {
    /// Yaw to ADD as a left world-yaw rotation: `Ry(yaw) · S` re-aligns the sensor.
    pub yaw_radians: f64,
    /// `sqrt((1−ρ²)/(2·N_eff·ρ²) + floor²)`: the coherence-phase Cramér–Rao
    /// approximation plus the asserted floor. Not validated on hardware.
    pub sigma_radians: f64,
    /// Rotation-maximised normalised correlation ρ in [0, 1].
    pub correlation: f64,
    /// Reference/IMU regression gain after rotation (1 for rigid translation).
    pub gain: f64,
    pub reference_rms_mps2: f64,
    pub imu_rms_mps2: f64,
    /// Smaller/larger eigenvalue of the IMU horizontal scatter (0: 1-D motion).
    pub anisotropy: f64,
}

/// Grid samples after the filter-settle interval that each gate rejected,
/// before guard dilation (one sample can count under several gates). The IMU
/// grid is longer than the reference grid by the lag search range.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GateCounts {
    pub window_samples: usize,
    pub reference_uncovered: usize,
    pub reference_rotation: usize,
    pub reference_implausible: usize,
    pub imu_uncovered: usize,
    pub imu_rotation: usize,
    pub imu_implausible: usize,
}

/// Serializable diagnostics. Every number is finite.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AccelHeadingReport {
    pub evaluated_at_secs: Option<f64>,
    pub status: AccelHeadingStatus,
    pub reason: String,
    pub frame: FrameDiagnostics,
    /// Applied lag (coarse latched step plus parabolic refinement).
    pub lag_secs: Option<f64>,
    /// Present only for `Candidate`: the yaw to add (left world yaw, radians).
    pub yaw_correction_radians: Option<f64>,
    pub yaw_sigma_radians: Option<f64>,
    pub horizontal_correlation: Option<f64>,
    /// Present whenever the horizontal fit was computed, even if withheld.
    pub horizontal: Option<HorizontalFit>,
    pub valid_samples: usize,
    pub effective_samples: f64,
    /// Valid paired grid samples over grid samples in the window.
    pub window_coverage: f64,
    pub gates: GateCounts,
    pub rejected_inputs: u64,
    pub clock_restarts: u64,
    /// Always false: this module never applies anything.
    pub live_correction_applied: bool,
}

#[derive(Clone, Copy, Debug)]
struct ReferenceSample {
    t: f64,
    position: Option<DVec3>,
    velocity: Option<DVec3>,
    angular_speed: f64,
}

#[derive(Clone, Copy, Debug)]
struct ImuSample {
    t: f64,
    rotation: DQuat,
    accel: DVec3,
    angular_speed: Option<f64>,
    plausible: bool,
}

/// Fixed-capacity ring buffer; the oldest element is overwritten when full.
#[derive(Clone, Debug)]
struct Ring<T> {
    buf: Vec<T>,
    capacity: usize,
    head: usize,
    len: usize,
}

impl<T: Copy> Ring<T> {
    fn new(capacity: usize) -> Self {
        Self {
            buf: Vec::with_capacity(capacity),
            capacity: capacity.max(1),
            head: 0,
            len: 0,
        }
    }

    fn push(&mut self, value: T) {
        if self.len < self.capacity {
            let index = (self.head + self.len) % self.capacity;
            if index == self.buf.len() {
                self.buf.push(value);
            } else if let Some(slot) = self.buf.get_mut(index) {
                *slot = value;
            }
            self.len += 1;
        } else {
            if let Some(slot) = self.buf.get_mut(self.head) {
                *slot = value;
            }
            self.head = (self.head + 1) % self.capacity;
        }
    }

    fn pop_front(&mut self) {
        if self.len > 0 {
            self.head = (self.head + 1) % self.capacity;
            self.len -= 1;
        }
    }

    fn clear(&mut self) {
        self.buf.clear();
        self.head = 0;
        self.len = 0;
    }

    fn get(&self, index: usize) -> Option<&T> {
        if index < self.len {
            self.buf.get((self.head + index) % self.capacity)
        } else {
            None
        }
    }

    fn front(&self) -> Option<&T> {
        self.get(0)
    }

    fn last(&self) -> Option<&T> {
        self.len.checked_sub(1).and_then(|i| self.get(i))
    }

    /// First logical index whose element does not satisfy `pred`, assuming
    /// `pred` is true for a prefix (binary search).
    fn partition_point(&self, pred: impl Fn(&T) -> bool) -> usize {
        let (mut lo, mut hi) = (0, self.len);
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            match self.get(mid) {
                Some(value) if pred(value) => lo = mid + 1,
                _ => hi = mid,
            }
        }
        lo
    }
}

#[derive(Clone, Copy, Debug)]
struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
}

impl Biquad {
    /// 2nd-order Butterworth by the bilinear transform with pre-warping.
    fn butterworth(cutoff_hz: f64, sample_hz: f64, highpass: bool) -> Self {
        let k = (PI * cutoff_hz / sample_hz).tan();
        let k2 = k * k;
        let norm = 1.0 / (1.0 + std::f64::consts::SQRT_2 * k + k2);
        let a1 = 2.0 * (k2 - 1.0) * norm;
        let a2 = (1.0 - std::f64::consts::SQRT_2 * k + k2) * norm;
        if highpass {
            Self {
                b0: norm,
                b1: -2.0 * norm,
                b2: norm,
                a1,
                a2,
            }
        } else {
            let b0 = k2 * norm;
            Self {
                b0,
                b1: 2.0 * b0,
                b2: b0,
                a1,
                a2,
            }
        }
    }

    fn dc_gain(&self) -> f64 {
        (self.b0 + self.b1 + self.b2) / (1.0 + self.a1 + self.a2)
    }

    /// Transposed direct-form-II state for a constant input `x`, so a segment
    /// starts without a step transient (accelerometer bias, gravity residue).
    fn steady_state(&self, x: f64) -> [f64; 2] {
        let y = self.dc_gain() * x;
        [y - self.b0 * x, self.b2 * x - self.a2 * y]
    }

    fn step(&self, state: &mut [f64; 2], x: f64) -> f64 {
        let y = self.b0 * x + state[0];
        state[0] = self.b1 * x - self.a1 * y + state[1];
        state[1] = self.b2 * x - self.a2 * y;
        y
    }
}

/// Grid layout shared by the reference and IMU channels of one report.
#[derive(Clone, Copy, Debug)]
struct Grid {
    t0: f64,
    dt: f64,
    count: usize,
    settle: usize,
    guard: usize,
}

impl Grid {
    fn time(&self, index: usize) -> f64 {
        self.t0 + index as f64 * self.dt
    }
}

/// Band-passed channels and their validity on one grid.
struct Channels<const N: usize> {
    values: Vec<[f64; N]>,
    valid: Vec<bool>,
    /// `[uncovered, rotation, implausible]` grid samples after the settle
    /// interval, before guard dilation.
    gates: [usize; 3],
}

fn tally(settle: usize, uncovered: &[bool], rotation: &[bool], implausible: &[bool]) -> [usize; 3] {
    let count = |flags: &[bool]| flags.iter().skip(settle).filter(|f| **f).count();
    [count(uncovered), count(rotation), count(implausible)]
}

/// Per-sensor shadow estimator. Owns bounded copies of both streams and only
/// two pieces of slowly changing state: the latched frame and the latched
/// coarse lag. It has no reference to any tracking state.
pub struct AccelHeadingEstimator {
    config: AccelHeadingConfig,
    candidates: [Mat3; FRAME_CANDIDATE_COUNT],
    allowed: Vec<usize>,
    lowpass: Biquad,
    highpass: Biquad,
    reference: Ring<ReferenceSample>,
    imu: Ring<ImuSample>,
    hint: Option<usize>,
    hint_status: HintStatus,
    latched_frame: Option<(usize, FrameSource)>,
    latched_lag_index: Option<usize>,
    rejected_inputs: u64,
    clock_restarts: u64,
}

struct Anchored {
    score: f64,
    margin: f64,
    ambiguous: Vec<usize>,
    supported: bool,
    contradicted: bool,
}

struct VerticalScores {
    /// `per_lag[lag_index][candidate]`, only meaningful for allowed candidates.
    per_lag: Vec<[f64; FRAME_CANDIDATE_COUNT]>,
    best_lag: [usize; FRAME_CANDIDATE_COUNT],
    best: [f64; FRAME_CANDIDATE_COUNT],
    gram: Vec<[[f64; 9]; 9]>,
    cross: Vec<[f64; 9]>,
    reference_energy: f64,
}

impl AccelHeadingEstimator {
    pub fn new(config: AccelHeadingConfig) -> Result<Self, CoreError> {
        config.validate()?;
        let allowed = if config.allowed_frame_candidates.is_empty() {
            (0..FRAME_CANDIDATE_COUNT).collect()
        } else {
            config.allowed_frame_candidates.clone()
        };
        Ok(Self {
            candidates: candidate_matrices(),
            allowed,
            lowpass: Biquad::butterworth(config.lowpass_hz, config.grid_rate_hz, false),
            highpass: Biquad::butterworth(config.highpass_hz, config.grid_rate_hz, true),
            reference: Ring::new(config.reference_capacity),
            imu: Ring::new(config.imu_capacity),
            hint: None,
            hint_status: HintStatus::Absent,
            latched_frame: None,
            latched_lag_index: None,
            rejected_inputs: 0,
            clock_restarts: 0,
            config,
        })
    }

    pub fn config(&self) -> &AccelHeadingConfig {
        &self.config
    }

    /// Clears both streams, the latched frame and lag, the hint and counters.
    pub fn reset(&mut self) {
        self.reference.clear();
        self.imu.clear();
        self.hint = None;
        self.hint_status = HintStatus::Absent;
        self.latched_frame = None;
        self.latched_lag_index = None;
        self.rejected_inputs = 0;
        self.clock_restarts = 0;
    }

    /// Suggest `C` (for example identified on another tracker of the same
    /// hardware, such as a thigh that swings through large tilts). The hint is
    /// used only after it passes this sensor's vertical check, and it cannot
    /// be distinguished from candidates in its own ambiguity group. Changing or
    /// clearing the hint drops any latched frame (re-setting the same hint is a
    /// no-op). Index must be below 24.
    pub fn set_frame_hint(&mut self, candidate: Option<usize>) -> Result<(), CoreError> {
        if candidate.is_some_and(|c| c >= FRAME_CANDIDATE_COUNT) {
            return Err(CoreError("frame hint index out of range".into()));
        }
        if candidate.is_some() && candidate == self.hint {
            return Ok(());
        }
        self.latched_frame = None;
        self.hint = candidate;
        self.hint_status = if candidate.is_some() {
            HintStatus::Pending
        } else {
            HintStatus::Absent
        };
        Ok(())
    }

    pub fn frame_hint(&self) -> Option<usize> {
        self.hint
    }

    /// The latched frame candidate and how it was established.
    pub fn latched_frame(&self) -> Option<(usize, FrameSource)> {
        self.latched_frame
    }

    /// Buffered `[reference, imu]` samples; never above [`Self::buffer_capacity`].
    pub fn buffered_samples(&self) -> [usize; 2] {
        [self.reference.len, self.imu.len]
    }

    pub fn buffer_capacity(&self) -> [usize; 2] {
        [self.reference.capacity, self.imu.capacity]
    }

    /// One reference sample: the absolutely tracked point's room-frame
    /// position and/or velocity (at least one), and the angular speed of the
    /// body carrying it (headset for a torso sensor), in rad/s.
    pub fn push_reference(
        &mut self,
        at_secs: f64,
        position: Option<Vec3>,
        velocity: Option<Vec3>,
        angular_speed_rad_per_sec: f32,
    ) -> SampleOutcome {
        let position = position.map(|p| p.as_dvec3());
        let velocity = velocity.map(|v| v.as_dvec3());
        let angular_speed = f64::from(angular_speed_rad_per_sec);
        let ok = at_secs.is_finite()
            && at_secs >= 0.0
            && (position.is_some() || velocity.is_some())
            && position.is_none_or(|p| p.is_finite() && p.length() <= MAX_POSITION_M)
            && velocity.is_none_or(|v| v.is_finite() && v.length() <= MAX_VELOCITY_MPS)
            && angular_speed.is_finite()
            && (0.0..=MAX_ANGULAR_SPEED_RAD_PER_SEC).contains(&angular_speed);
        if !ok {
            self.rejected_inputs += 1;
            return SampleOutcome::Rejected;
        }
        let Some(outcome) = self.admit(at_secs, self.reference.last().map(|s| s.t)) else {
            return SampleOutcome::Rejected;
        };
        self.reference.push(ReferenceSample {
            t: at_secs,
            position,
            velocity,
            angular_speed,
        });
        let horizon = at_secs - self.config.history_secs();
        while self.reference.front().is_some_and(|s| s.t < horizon) {
            self.reference.pop_front();
        }
        outcome
    }

    /// One IMU sample: the caller's heading-corrected sensor-to-room rotation,
    /// the gravity-removed acceleration in raw IMU axes (m/s²) and, if known,
    /// the angular speed (rad/s; otherwise differenced from the rotations).
    pub fn push_imu(
        &mut self,
        at_secs: f64,
        sensor_to_room: Quat,
        accel_body: Vec3,
        angular_speed_rad_per_sec: Option<f32>,
    ) -> SampleOutcome {
        let rotation = sensor_to_room.as_dquat();
        let length_squared = rotation.length_squared();
        let accel = accel_body.as_dvec3();
        let angular_speed = angular_speed_rad_per_sec.map(f64::from);
        let ok = at_secs.is_finite()
            && at_secs >= 0.0
            && rotation.is_finite()
            && (0.5..=1.5).contains(&length_squared)
            && accel.is_finite()
            && accel.length() <= MAX_RAW_ACCEL_MPS2
            && angular_speed.is_none_or(|w| {
                w.is_finite() && (0.0..=MAX_ANGULAR_SPEED_RAD_PER_SEC).contains(&w)
            });
        if !ok {
            self.rejected_inputs += 1;
            return SampleOutcome::Rejected;
        }
        let Some(outcome) = self.admit(at_secs, self.imu.last().map(|s| s.t)) else {
            return SampleOutcome::Rejected;
        };
        self.imu.push(ImuSample {
            t: at_secs,
            rotation: rotation / length_squared.sqrt(),
            accel,
            angular_speed,
            plausible: accel.length() <= self.config.max_acceleration_mps2,
        });
        let horizon = at_secs - self.config.history_secs();
        while self.imu.front().is_some_and(|s| s.t < horizon) {
            self.imu.pop_front();
        }
        outcome
    }

    fn admit(&mut self, at_secs: f64, last: Option<f64>) -> Option<SampleOutcome> {
        match last {
            Some(last) if at_secs <= last => {
                if last - at_secs > CLOCK_RESTART_SECS {
                    self.reference.clear();
                    self.imu.clear();
                    self.latched_lag_index = None;
                    self.clock_restarts += 1;
                    Some(SampleOutcome::ClockRestarted)
                } else {
                    self.rejected_inputs += 1;
                    None
                }
            }
            _ => Some(SampleOutcome::Accepted),
        }
    }

    /// Analyses the window ending at `now`. Updates only the latched frame and
    /// lag (both with hysteresis, so repeated calls with the same data return
    /// the same report). Never applies anything to tracking.
    pub fn report(&mut self, now: f64) -> AccelHeadingReport {
        let mut report = AccelHeadingReport {
            evaluated_at_secs: now.is_finite().then_some(now),
            status: AccelHeadingStatus::InsufficientData,
            reason: String::new(),
            frame: FrameDiagnostics {
                candidate: self.latched_frame.map(|(c, _)| c),
                source: self.latched_frame.map(|(_, s)| s),
                best_candidate: None,
                vertical_agreement: None,
                vertical_correlation: None,
                margin: None,
                ambiguous_candidates: Vec::new(),
                hint: self.hint,
                hint_status: self.hint_status,
                vertical_reference_rms_mps2: None,
                tilt_spread_degrees: None,
                scores: Vec::new(),
            },
            lag_secs: None,
            yaw_correction_radians: None,
            yaw_sigma_radians: None,
            horizontal_correlation: None,
            horizontal: None,
            valid_samples: 0,
            effective_samples: 0.0,
            window_coverage: 0.0,
            gates: GateCounts::default(),
            rejected_inputs: self.rejected_inputs,
            clock_restarts: self.clock_restarts,
            live_correction_applied: false,
        };
        let (status, reason) = self.analyze(now, &mut report);
        report.status = status;
        report.reason = reason;
        report.frame.candidate = self.latched_frame.map(|(c, _)| c);
        report.frame.source = self.latched_frame.map(|(_, s)| s);
        report.frame.hint = self.hint;
        report.frame.hint_status = self.hint_status;
        if status != AccelHeadingStatus::Candidate {
            report.yaw_correction_radians = None;
            report.yaw_sigma_radians = None;
        }
        report
    }

    fn analyze(
        &mut self,
        now: f64,
        report: &mut AccelHeadingReport,
    ) -> (AccelHeadingStatus, String) {
        use AccelHeadingStatus as S;
        let c = self.config.clone();
        if !now.is_finite() || now < 0.0 {
            return (S::InsufficientData, "invalid evaluation time".into());
        }
        let (Some(last_ref), Some(last_imu)) = (self.reference.last(), self.imu.last()) else {
            return (
                S::InsufficientData,
                "waiting for reference and IMU samples".into(),
            );
        };
        let (last_ref_t, last_imu_t) = (last_ref.t, last_imu.t);
        if now - last_ref_t > c.max_input_age_secs || now - last_imu_t > c.max_input_age_secs {
            return (
                S::InsufficientData,
                "reference or IMU stream is stale".into(),
            );
        }
        let dt = 1.0 / c.grid_rate_hz;
        let lag_min = (c.min_lag_secs / dt).round() as i64;
        let lag_max = (c.max_lag_secs / dt).round() as i64;
        let lag_count = (lag_max - lag_min + 1).max(1) as usize;
        let t_end = now
            .min(last_ref_t - dt)
            .min(last_imu_t - lag_max as f64 * dt);
        let t_end = (t_end / dt).floor() * dt;
        let window_count = (c.window_secs / dt).round() as usize;
        let settle = (c.settle_secs / dt).round() as usize;
        let count = window_count + settle + 1;
        let guard = (c.gate_guard_secs / dt).round() as usize;
        let reference_grid = Grid {
            t0: t_end - (count - 1) as f64 * dt,
            dt,
            count,
            settle,
            guard,
        };
        let imu_grid = Grid {
            t0: reference_grid.t0 + lag_min as f64 * dt,
            count: count + lag_count - 1,
            ..reference_grid
        };

        let reference = self.reference_channels(&reference_grid);
        let vertical = self.imu_channels::<9>(&imu_grid, |rotation, accel| {
            let up = rotation.conjugate() * DVec3::Y;
            let mut m = [0.0; 9];
            for (index, value) in m.iter_mut().enumerate() {
                *value = up[index / 3] * accel[index % 3];
            }
            m
        });
        let mut imu_invalid_prefix = Vec::with_capacity(imu_grid.count + 1);
        imu_invalid_prefix.push(0_usize);
        for valid in &vertical.valid {
            let last = imu_invalid_prefix.last().copied().unwrap_or(0);
            imu_invalid_prefix.push(last + usize::from(!*valid));
        }
        let pairs: Vec<usize> = (0..count)
            .filter(|&k| {
                reference.valid.get(k).copied().unwrap_or(false)
                    && imu_invalid_prefix.get(k + lag_count).copied().unwrap_or(1)
                        == imu_invalid_prefix.get(k).copied().unwrap_or(0)
            })
            .collect();
        let n_valid = pairs.len();
        let bandwidth_factor = (2.0 * (c.lowpass_hz - c.highpass_hz) / c.grid_rate_hz).min(1.0);
        let effective = n_valid as f64 * bandwidth_factor;
        report.gates = GateCounts {
            window_samples: window_count,
            reference_uncovered: reference.gates[0],
            reference_rotation: reference.gates[1],
            reference_implausible: reference.gates[2],
            imu_uncovered: vertical.gates[0],
            imu_rotation: vertical.gates[1],
            imu_implausible: vertical.gates[2],
        };
        report.valid_samples = n_valid;
        report.effective_samples = effective;
        report.window_coverage = if window_count > 0 {
            (n_valid as f64 / window_count as f64).min(1.0)
        } else {
            0.0
        };
        report.frame.tilt_spread_degrees =
            self.tilt_spread_degrees(reference_grid.time(settle), t_end);
        if effective < c.min_effective_samples {
            return (
                S::InsufficientData,
                format!(
                    "{effective:.0} effective samples after gating; {:.0} required (of {window_count} window samples: reference uncovered {}, rotating {}, implausible {}; IMU uncovered {}, rotating {}, implausible {})",
                    c.min_effective_samples,
                    reference.gates[0],
                    reference.gates[1],
                    reference.gates[2],
                    vertical.gates[0],
                    vertical.gates[1],
                    vertical.gates[2],
                ),
            );
        }

        let scores = self.vertical_scores(&reference.values, &vertical.values, &pairs, lag_count);
        let vertical_rms = (scores.reference_energy / n_valid as f64).sqrt();
        report.frame.vertical_reference_rms_mps2 = Some(finite_or(vertical_rms, 0.0));
        let lag_secs = |index: usize| (lag_min + index as i64) as f64 * dt;
        let mut ranked: Vec<CandidateScore> = self
            .allowed
            .iter()
            .map(|&candidate| CandidateScore {
                candidate,
                vertical_agreement: finite_or(scores.best[candidate], 0.0),
                lag_secs: lag_secs(scores.best_lag[candidate]),
            })
            .collect();
        ranked.sort_by(|a, b| b.vertical_agreement.total_cmp(&a.vertical_agreement));
        let Some(best) = ranked.first().map(|s| s.candidate) else {
            return (S::InsufficientData, "no frame candidates allowed".into());
        };
        report.frame.scores = ranked;
        report.frame.best_candidate = Some(best);
        if vertical_rms < c.min_vertical_rms_mps2 {
            return (
                S::LowExcitation,
                format!(
                    "vertical reference acceleration RMS {vertical_rms:.2} m/s² is below {:.2}; frame, lag and rigidity cannot be checked",
                    c.min_vertical_rms_mps2
                ),
            );
        }

        // Frame: latched (contradiction only), else hint (needs support), else
        // unique identification from the data.
        let frame = if let Some((latched, source)) = self.latched_frame {
            let anchored = self.anchored(latched, &scores);
            self.describe_anchor(report, latched, &anchored, &scores);
            if anchored.contradicted {
                self.latched_frame = None;
                if source == FrameSource::Hint {
                    self.hint_status = HintStatus::Rejected;
                    return (
                        S::FrameHintRejected,
                        format!(
                            "frame {latched} (hint) is now contradicted by the vertical acceleration"
                        ),
                    );
                }
                return (
                    S::FrameUnidentified,
                    format!(
                        "latched frame {latched} is contradicted by the vertical acceleration; re-identifying"
                    ),
                );
            }
            latched
        } else if let Some(hint) = self.hint {
            let anchored = self.anchored(hint, &scores);
            self.describe_anchor(report, hint, &anchored, &scores);
            if anchored.supported {
                self.latched_frame = Some((hint, FrameSource::Hint));
                self.hint_status = HintStatus::Verified;
                hint
            } else if anchored.contradicted {
                self.hint_status = HintStatus::Rejected;
                return (
                    S::FrameHintRejected,
                    format!(
                        "hint {hint} agrees {:.3}; a distinguishable frame agrees better by {:.3}",
                        anchored.score, -anchored.margin
                    ),
                );
            } else {
                self.hint_status = HintStatus::Pending;
                return (
                    S::FrameUnidentified,
                    format!(
                        "hint {hint} not yet verified (agreement {:.3}, margin {:.3})",
                        anchored.score, anchored.margin
                    ),
                );
            }
        } else {
            let anchored = self.anchored(best, &scores);
            self.describe_anchor(report, best, &anchored, &scores);
            if anchored.score < c.min_vertical_agreement {
                return (
                    S::FrameUnidentified,
                    format!(
                        "best vertical agreement {:.3} is below {:.3}",
                        anchored.score, c.min_vertical_agreement
                    ),
                );
            }
            if anchored.margin < c.min_frame_margin {
                return (
                    S::FrameUnidentified,
                    format!(
                        "margin {:.3} over a distinguishable frame is below {:.3}",
                        anchored.margin, c.min_frame_margin
                    ),
                );
            }
            if anchored.ambiguous.len() > 1 {
                return (
                    S::FrameAmbiguous,
                    format!(
                        "candidates {:?} are indistinguishable from vertical acceleration (tilt diversity too small); supply a verified frame hint",
                        anchored.ambiguous
                    ),
                );
            }
            self.latched_frame = Some((best, FrameSource::Identified));
            best
        };

        // Lag with hysteresis, then parabolic refinement.
        let best_lag = scores.best_lag[frame];
        let score_at = |lag: usize, candidate: usize| {
            scores
                .per_lag
                .get(lag)
                .map_or(f64::NEG_INFINITY, |row| row[candidate])
        };
        let lag_index = match self.latched_lag_index {
            Some(latched)
                if latched < lag_count
                    && score_at(best_lag, frame)
                        <= score_at(latched, frame) + c.lag_switch_margin =>
            {
                latched
            }
            _ => best_lag,
        };
        self.latched_lag_index = Some(lag_index);
        let mut fraction = 0.0;
        if lag_index > 0 && lag_index + 1 < lag_count {
            let (a, b, d) = (
                score_at(lag_index - 1, frame),
                score_at(lag_index, frame),
                score_at(lag_index + 1, frame),
            );
            let curvature = a - 2.0 * b + d;
            if curvature < -TINY {
                fraction = finite_or(0.5 * (a - d) / curvature, 0.0).clamp(-0.5, 0.5);
            }
        }
        report.lag_secs = Some(lag_secs(lag_index) + fraction * dt);
        let agreement = score_at(lag_index, frame);
        report.frame.vertical_agreement = Some(finite_or(agreement, 0.0));
        report.frame.vertical_correlation = Some(self.pearson(frame, lag_index, &scores));
        if agreement < c.min_vertical_agreement {
            return (
                S::PoorCorrelation,
                format!(
                    "vertical agreement {agreement:.3} under frame {frame} is below {:.3}: rigid translation fails in this window",
                    c.min_vertical_agreement
                ),
            );
        }
        if !c.horizontal_reference {
            return (
                S::FrameIdentifiedOnly,
                "frame and lag identified; horizontal reference is not independent of this sensor's heading".into(),
            );
        }

        // Horizontal 2-D Procrustes under the chosen frame and lag.
        let frame_rotation = matrix_quat(&self.candidates[frame]);
        let horizontal = self.imu_channels::<2>(&imu_grid, |rotation, accel| {
            let world = rotation * (frame_rotation * accel);
            [world.x, world.z]
        });
        let offset = lag_index as f64 + fraction;
        let (mut a, mut b, mut er, mut em) = (0.0, 0.0, 0.0, 0.0);
        let (mut sxx, mut sxz, mut szz) = (0.0, 0.0, 0.0);
        for &k in &pairs {
            let Some(r) = reference.values.get(k) else {
                continue;
            };
            let (rx, rz) = (r[0], r[2]);
            let position = k as f64 + offset;
            let j0 = position.floor().max(0.0) as usize;
            let w = position - j0 as f64;
            let (Some(m0), Some(m1)) = (
                horizontal.values.get(j0),
                horizontal.values.get(j0 + 1).or(horizontal.values.get(j0)),
            ) else {
                continue;
            };
            let mx = m0[0] * (1.0 - w) + m1[0] * w;
            let mz = m0[1] * (1.0 - w) + m1[1] * w;
            a += rx * mx + rz * mz;
            b += rx * mz - rz * mx;
            er += rx * rx + rz * rz;
            em += mx * mx + mz * mz;
            sxx += mx * mx;
            sxz += mx * mz;
            szz += mz * mz;
        }
        let reference_rms = (er / n_valid as f64).sqrt();
        let imu_rms = (em / n_valid as f64).sqrt();
        if !reference_rms.is_finite() || reference_rms < c.min_horizontal_rms_mps2 {
            return (
                S::LowExcitation,
                format!(
                    "horizontal reference acceleration RMS {:.2} m/s² is below {:.2}",
                    finite_or(reference_rms, 0.0),
                    c.min_horizontal_rms_mps2
                ),
            );
        }
        if em <= TINY || er <= TINY {
            return (
                S::PoorCorrelation,
                "IMU horizontal acceleration vanishes under the chosen frame".into(),
            );
        }
        let yaw = b.atan2(a);
        let magnitude = a.hypot(b);
        let correlation = finite_or(magnitude / (er * em).sqrt(), 0.0).clamp(0.0, 1.0);
        let gain = finite_or(magnitude / em, 0.0);
        let half_trace = 0.5 * (sxx + szz);
        let spread = (0.25 * (sxx - szz) * (sxx - szz) + sxz * sxz)
            .max(0.0)
            .sqrt();
        let anisotropy = if half_trace + spread > TINY {
            ((half_trace - spread) / (half_trace + spread)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let sigma = if correlation > 1.0e-6 {
            ((1.0 - correlation * correlation) / (2.0 * effective * correlation * correlation)
                + c.yaw_sigma_floor_radians * c.yaw_sigma_floor_radians)
                .sqrt()
                .min(PI)
        } else {
            PI
        };
        let sigma = finite_or(sigma, PI);
        let fit = HorizontalFit {
            yaw_radians: finite_or(yaw, 0.0),
            sigma_radians: sigma,
            correlation,
            gain,
            reference_rms_mps2: finite_or(reference_rms, 0.0),
            imu_rms_mps2: finite_or(imu_rms, 0.0),
            anisotropy,
        };
        report.horizontal_correlation = Some(correlation);
        report.horizontal = Some(fit.clone());
        if correlation < c.min_horizontal_correlation {
            return (
                S::PoorCorrelation,
                format!(
                    "horizontal correlation {correlation:.3} is below {:.3}",
                    c.min_horizontal_correlation
                ),
            );
        }
        if !(c.min_horizontal_gain..=c.max_horizontal_gain).contains(&gain) {
            return (
                S::PoorCorrelation,
                format!(
                    "horizontal amplitude ratio {gain:.2} is implausible for rigid translation"
                ),
            );
        }
        if sigma > c.max_yaw_sigma_radians {
            return (
                S::PoorCorrelation,
                format!(
                    "yaw sigma {:.1}° exceeds {:.1}°",
                    sigma.to_degrees(),
                    c.max_yaw_sigma_radians.to_degrees()
                ),
            );
        }
        if fit.yaw_radians.abs() > c.max_candidate_yaw_radians {
            return (
                S::ExcessiveCorrection,
                format!(
                    "fitted yaw {:.1}° exceeds {:.1}°; a wrong frame inside an ambiguity group acts like a 90/180° yaw",
                    fit.yaw_radians.to_degrees(),
                    c.max_candidate_yaw_radians.to_degrees()
                ),
            );
        }
        report.yaw_correction_radians = Some(fit.yaw_radians);
        report.yaw_sigma_radians = Some(sigma);
        (
            S::Candidate,
            "shadow candidate only; nothing has been applied".into(),
        )
    }

    fn describe_anchor(
        &self,
        report: &mut AccelHeadingReport,
        anchor: usize,
        anchored: &Anchored,
        scores: &VerticalScores,
    ) {
        report.frame.vertical_agreement = Some(finite_or(anchored.score, 0.0));
        report.frame.vertical_correlation =
            Some(self.pearson(anchor, scores.best_lag[anchor], scores));
        report.frame.margin = Some(finite_or(anchored.margin, 0.0));
        report.frame.ambiguous_candidates = anchored.ambiguous.clone();
    }

    fn pearson(&self, candidate: usize, lag: usize, scores: &VerticalScores) -> f64 {
        let s = flatten(&self.candidates[candidate]);
        let (Some(cross), Some(gram)) = (scores.cross.get(lag), scores.gram.get(lag)) else {
            return 0.0;
        };
        let denominator = (scores.reference_energy * quadratic(gram, &s)).sqrt();
        if denominator > TINY {
            finite_or(dot9(&s, cross) / denominator, 0.0).clamp(-1.0, 1.0)
        } else {
            0.0
        }
    }

    /// Tests `anchor` against every allowed alternative that this window's
    /// excitation could distinguish from it.
    fn anchored(&self, anchor: usize, scores: &VerticalScores) -> Anchored {
        let c = &self.config;
        let score = scores.best[anchor];
        let lag = scores.best_lag[anchor];
        let s_anchor = flatten(&self.candidates[anchor]);
        // Noise lowers every candidate's agreement alike, shrinking the margin a
        // structural difference can produce. The best agreement in the window
        // measures that shrinkage whether or not the anchor is the true frame.
        let shrink = self
            .allowed
            .iter()
            .map(|&candidate| scores.best[candidate])
            .fold(0.0, f64::max)
            .clamp(0.0, 1.0);
        let ambiguous: Vec<usize> = match scores.gram.get(lag) {
            Some(gram) => {
                let p_anchor = quadratic(gram, &s_anchor);
                self.allowed
                    .iter()
                    .copied()
                    .filter(|&other| {
                        let s_other = flatten(&self.candidates[other]);
                        let mut difference = [0.0; 9];
                        for (d, (x, y)) in difference.iter_mut().zip(s_anchor.iter().zip(&s_other))
                        {
                            *d = x - y;
                        }
                        let denominator = p_anchor + quadratic(gram, &s_other);
                        // The margin noise-free data would give between the two.
                        let separation = if denominator > TINY {
                            quadratic(gram, &difference) / denominator
                        } else {
                            0.0
                        };
                        separation * shrink < c.min_frame_margin
                    })
                    .collect()
            }
            None => vec![anchor],
        };
        let runner = self
            .allowed
            .iter()
            .filter(|candidate| !ambiguous.contains(candidate))
            .map(|&candidate| scores.best[candidate])
            .fold(f64::NEG_INFINITY, f64::max);
        let margin = if runner.is_finite() {
            score - runner
        } else {
            score + 1.0
        };
        Anchored {
            score,
            margin,
            supported: score >= c.min_vertical_agreement && margin >= c.min_frame_margin,
            // A distinguishable alternative is clearly better, or passes the
            // agreement threshold that the anchor fails.
            contradicted: -margin >= c.min_frame_margin
                || (score < c.min_vertical_agreement && runner >= c.min_vertical_agreement),
            ambiguous,
        }
    }

    #[allow(clippy::needless_range_loop)]
    fn vertical_scores(
        &self,
        reference: &[[f64; 3]],
        imu: &[[f64; 9]],
        pairs: &[usize],
        lag_count: usize,
    ) -> VerticalScores {
        let mut cross = vec![[0.0; 9]; lag_count];
        let mut gram = vec![[[0.0; 9]; 9]; lag_count];
        let mut reference_energy = 0.0;
        for &k in pairs {
            let Some(y) = reference.get(k).map(|r| r[1]) else {
                continue;
            };
            reference_energy += y * y;
            for lag in 0..lag_count {
                let Some(m) = imu.get(k + lag) else {
                    continue;
                };
                for p in 0..9 {
                    cross[lag][p] += y * m[p];
                    for q in p..9 {
                        gram[lag][p][q] += m[p] * m[q];
                    }
                }
            }
        }
        for g in &mut gram {
            for p in 0..9 {
                for q in 0..p {
                    g[p][q] = g[q][p];
                }
            }
        }
        let mut per_lag = vec![[0.0; FRAME_CANDIDATE_COUNT]; lag_count];
        let mut best_lag = [0; FRAME_CANDIDATE_COUNT];
        let mut best = [f64::NEG_INFINITY; FRAME_CANDIDATE_COUNT];
        for &candidate in &self.allowed {
            let s = flatten(&self.candidates[candidate]);
            for lag in 0..lag_count {
                let denominator = reference_energy + quadratic(&gram[lag], &s);
                let score = if denominator > TINY {
                    finite_or(2.0 * dot9(&s, &cross[lag]) / denominator, 0.0)
                } else {
                    0.0
                };
                per_lag[lag][candidate] = score;
                if score > best[candidate] {
                    best[candidate] = score;
                    best_lag[candidate] = lag;
                }
            }
        }
        for value in &mut best {
            if !value.is_finite() {
                *value = 0.0;
            }
        }
        VerticalScores {
            per_lag,
            best_lag,
            best,
            gram,
            cross,
            reference_energy,
        }
    }

    fn tilt_spread_degrees(&self, from: f64, to: f64) -> Option<f64> {
        let lo = self.imu.partition_point(|s| s.t < from);
        let hi = self.imu.partition_point(|s| s.t <= to);
        let ups: Vec<DVec3> = (lo..hi)
            .filter_map(|i| self.imu.get(i))
            .map(|s| s.rotation.conjugate() * DVec3::Y)
            .collect();
        if ups.is_empty() {
            return None;
        }
        let mean = ups.iter().copied().sum::<DVec3>();
        let mean = mean.try_normalize()?;
        let squared: f64 = ups
            .iter()
            .map(|u| {
                let angle = u.dot(mean).clamp(-1.0, 1.0).acos();
                angle * angle
            })
            .sum();
        Some(finite_or(
            (squared / ups.len() as f64).sqrt().to_degrees(),
            0.0,
        ))
    }

    fn reference_velocity(&self, t: f64) -> Option<DVec3> {
        let (a, b, w) = bracket(&self.reference, t, self.config.max_gap_secs, |s| s.t)?;
        Some(a.velocity? * (1.0 - w) + b.velocity? * w)
    }

    fn reference_position(&self, t: f64) -> Option<DVec3> {
        let (a, b, w) = bracket(&self.reference, t, self.config.max_gap_secs, |s| s.t)?;
        Some(a.position? * (1.0 - w) + b.position? * w)
    }

    /// Reference acceleration at `t` by a centred difference (zero phase).
    fn reference_accel(&self, t: f64, h: f64) -> Option<DVec3> {
        let max_speed = self.config.max_reference_speed_mps;
        let from_velocity = || {
            let (v0, v1) = (
                self.reference_velocity(t - 0.5 * h)?,
                self.reference_velocity(t + 0.5 * h)?,
            );
            (v0.length() <= max_speed && v1.length() <= max_speed).then(|| (v1 - v0) / h)
        };
        let from_position = || {
            let (p0, p1, p2) = (
                self.reference_position(t - h)?,
                self.reference_position(t)?,
                self.reference_position(t + h)?,
            );
            ((p1 - p0).length() <= max_speed * h && (p2 - p1).length() <= max_speed * h)
                .then(|| (p2 - 2.0 * p1 + p0) / (h * h))
        };
        let accel = if self.config.prefer_reference_velocity {
            from_velocity().or_else(from_position)
        } else {
            from_position().or_else(from_velocity)
        };
        accel.filter(|a| a.is_finite())
    }

    fn reference_channels(&self, grid: &Grid) -> Channels<3> {
        let c = &self.config;
        let mut raw = Vec::with_capacity(grid.count);
        let mut gate = Vec::with_capacity(grid.count);
        for k in 0..grid.count {
            let t = grid.time(k);
            raw.push(self.reference_accel(t, grid.dt).map(|a| a.to_array()));
            let speed = bracket(&self.reference, t, c.max_gap_secs, |s| s.t)
                .map(|(a, b, _)| a.angular_speed.max(b.angular_speed));
            gate.push(speed.is_some_and(|w| w <= c.max_reference_angular_speed_rad_per_sec));
        }
        let (values, mut valid, lowpassed) =
            filter_segments(&raw, &self.lowpass, &self.highpass, grid.settle);
        let uncovered: Vec<bool> = raw.iter().map(Option::is_none).collect();
        let rotation: Vec<bool> = gate.iter().zip(&uncovered).map(|(g, u)| !g && !u).collect();
        let implausible: Vec<bool> = lowpassed
            .iter()
            .map(|l| *l > c.max_acceleration_mps2)
            .collect();
        for ((valid, gate), implausible) in valid.iter_mut().zip(&gate).zip(&implausible) {
            *valid &= *gate && !*implausible;
        }
        Channels {
            values,
            valid: dilate_valid(&valid, grid.guard),
            gates: tally(grid.settle, &uncovered, &rotation, &implausible),
        }
    }

    /// IMU channels that are linear in the raw acceleration (so filtering
    /// commutes with combining them), on the given grid.
    fn imu_channels<const N: usize>(
        &self,
        grid: &Grid,
        channels: impl Fn(DQuat, DVec3) -> [f64; N],
    ) -> Channels<N> {
        let c = &self.config;
        let first = grid.time(0) - c.max_gap_secs;
        let last = grid.time(grid.count.saturating_sub(1)) + c.max_gap_secs;
        let lo = self.imu.partition_point(|s| s.t < first);
        let hi = self.imu.partition_point(|s| s.t <= last);
        let mut points: Vec<(f64, [f64; N], Option<f64>)> =
            Vec::with_capacity(hi.saturating_sub(lo));
        let mut implausible = Vec::new();
        for i in lo..hi {
            let Some(sample) = self.imu.get(i) else {
                continue;
            };
            let previous = i
                .checked_sub(1)
                .and_then(|p| self.imu.get(p))
                .filter(|p| sample.t - p.t <= c.max_gap_secs && sample.t > p.t);
            if !sample.plausible {
                implausible.push(sample.t);
                continue;
            }
            let rotation = if c.accel_lags_rotation_by_one_sample {
                previous.map_or(sample.rotation, |p| p.rotation)
            } else {
                sample.rotation
            };
            let speed = sample.angular_speed.or_else(|| {
                previous.map(|p| quat_angle(p.rotation, sample.rotation) / (sample.t - p.t))
            });
            points.push((sample.t, channels(rotation, sample.accel), speed));
        }
        let mut raw = Vec::with_capacity(grid.count);
        let mut gate = Vec::with_capacity(grid.count);
        let mut cursor = 0;
        for j in 0..grid.count {
            let t = grid.time(j);
            while points.get(cursor + 1).is_some_and(|p| p.0 <= t) {
                cursor += 1;
            }
            let value = match (points.get(cursor), points.get(cursor + 1)) {
                (Some(a), _) if a.0 == t => Some((a.1, a.2)),
                (Some(a), Some(b)) if a.0 < t && t < b.0 && b.0 - a.0 <= c.max_gap_secs => {
                    let w = (t - a.0) / (b.0 - a.0);
                    let mut value = [0.0; N];
                    for (v, (x, y)) in value.iter_mut().zip(a.1.iter().zip(&b.1)) {
                        *v = x * (1.0 - w) + y * w;
                    }
                    let speed = match (a.2, b.2) {
                        (Some(x), Some(y)) => Some(x.max(y)),
                        _ => None,
                    };
                    Some((value, speed))
                }
                _ => None,
            };
            gate.push(
                value
                    .and_then(|(_, speed)| speed)
                    .is_some_and(|w| w <= c.max_imu_angular_speed_rad_per_sec),
            );
            raw.push(value.map(|(v, _)| v));
        }
        let uncovered: Vec<bool> = raw.iter().map(Option::is_none).collect();
        let rotation: Vec<bool> = gate.iter().zip(&uncovered).map(|(g, u)| !g && !u).collect();
        let mut flagged = vec![false; grid.count];
        for t in implausible {
            let index = ((t - grid.t0) / grid.dt).round();
            if index >= 0.0 && (index as usize) < grid.count {
                flagged[index as usize] = true;
            }
        }
        let (values, mut valid, _) =
            filter_segments(&raw, &self.lowpass, &self.highpass, grid.settle);
        for ((valid, gate), flagged) in valid.iter_mut().zip(&gate).zip(&flagged) {
            *valid &= *gate && !*flagged;
        }
        Channels {
            values,
            valid: dilate_valid(&valid, grid.guard),
            gates: tally(grid.settle, &uncovered, &rotation, &flagged),
        }
    }
}

/// Bracketing samples `a ≤ t ≤ b` no further apart than `max_gap`, with the
/// interpolation weight of `b`.
fn bracket<T: Copy>(
    ring: &Ring<T>,
    t: f64,
    max_gap: f64,
    time: impl Fn(&T) -> f64,
) -> Option<(T, T, f64)> {
    let after = ring.partition_point(|s| time(s) <= t);
    let a = *ring.get(after.checked_sub(1)?)?;
    let ta = time(&a);
    if ta == t {
        return Some((a, a, 0.0));
    }
    let b = *ring.get(after)?;
    let tb = time(&b);
    (tb > ta && tb - ta <= max_gap).then(|| (a, b, (t - ta) / (tb - ta)))
}

/// Low-pass and high-pass biquad states per channel.
type BandPassState<const N: usize> = ([[f64; 2]; N], [[f64; 2]; N]);

/// Causal band-pass per channel. Each run of `Some` samples is a segment: its
/// filters start in steady state and its first `settle` outputs are invalid.
/// Also returns the low-pass stage norm for plausibility checks.
fn filter_segments<const N: usize>(
    input: &[Option<[f64; N]>],
    lowpass: &Biquad,
    highpass: &Biquad,
    settle: usize,
) -> (Vec<[f64; N]>, Vec<bool>, Vec<f64>) {
    let mut values = Vec::with_capacity(input.len());
    let mut valid = Vec::with_capacity(input.len());
    let mut lowpass_norm = Vec::with_capacity(input.len());
    let mut state: Option<BandPassState<N>> = None;
    let mut remaining = 0;
    for sample in input {
        let Some(x) = sample else {
            state = None;
            values.push([0.0; N]);
            valid.push(false);
            lowpass_norm.push(0.0);
            continue;
        };
        if state.is_none() {
            let mut low = [[0.0; 2]; N];
            let mut high = [[0.0; 2]; N];
            for ((l, h), value) in low.iter_mut().zip(high.iter_mut()).zip(x) {
                *l = lowpass.steady_state(*value);
                *h = highpass.steady_state(lowpass.dc_gain() * value);
            }
            state = Some((low, high));
            remaining = settle;
        }
        let mut out = [0.0; N];
        let mut norm = 0.0;
        if let Some((low, high)) = state.as_mut() {
            for (((o, value), l), h) in out
                .iter_mut()
                .zip(x)
                .zip(low.iter_mut())
                .zip(high.iter_mut())
            {
                let y = lowpass.step(l, *value);
                norm += y * y;
                *o = highpass.step(h, y);
            }
        }
        let finite = out.iter().all(|v| v.is_finite());
        if !finite {
            out = [0.0; N];
            state = None;
        }
        values.push(out);
        valid.push(finite && remaining == 0);
        lowpass_norm.push(if norm.is_finite() {
            norm.sqrt()
        } else {
            f64::MAX
        });
        remaining = remaining.saturating_sub(1);
    }
    (values, valid, lowpass_norm)
}

/// A sample stays valid only if every sample within `guard` of it is valid.
fn dilate_valid(valid: &[bool], guard: usize) -> Vec<bool> {
    let mut prefix = Vec::with_capacity(valid.len() + 1);
    prefix.push(0_usize);
    for v in valid {
        let last = prefix.last().copied().unwrap_or(0);
        prefix.push(last + usize::from(!*v));
    }
    (0..valid.len())
        .map(|i| {
            let lo = i.saturating_sub(guard);
            let hi = (i + guard + 1).min(valid.len());
            prefix.get(hi).copied().unwrap_or(0) == prefix.get(lo).copied().unwrap_or(0)
        })
        .collect()
}

fn dot9(a: &[f64; 9], b: &[f64; 9]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn quadratic(gram: &[[f64; 9]; 9], s: &[f64; 9]) -> f64 {
    gram.iter().zip(s).map(|(row, si)| si * dot9(row, s)).sum()
}

fn quat_angle(a: DQuat, b: DQuat) -> f64 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos()
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}
