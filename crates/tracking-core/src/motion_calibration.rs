//! Motion calibration (EXPERIMENTAL, synthetic validation only).
//!
//! After a neutral capture the user performs any mostly sagittal movement —
//! squats, a bow, knee lifts (marching in place) or the held skiing pose —
//! continuously and without holding still. Per core sensor this module
//! accumulates the trajectory of the bone's long-axis tip relative to neutral
//! and solves the left world-yaw heading alignment `psi_i` and the right
//! mounting `M_i` of [`crate::forward_calibration::ForwardAlignment`]:
//!
//! ```text
//! R_i(t) = Ry(psi_i) · q_i(t) · M_i,   M_i = (Ry(psi_i) · q0_i)^-1 · Ry(h)
//! D_i(t) = q_i(t) · q0_i^-1            (rotation from neutral, sensor world)
//! v      = D_i(t) · Y,  p = (v.x, v.z), tilt = acos(v.y)
//! S      = Σ w p pᵀ (about the origin),  m = Σ w p,  w = sample dt
//! e      = principal eigenvector of S,   f_i = s_i · sign(m·e) · e
//! psi_i  = wrap(h − atan2(−f.x, −f.z))
//! ```
//!
//! `s_i = +1` for chest, hip and calves (their tops tip forward in the
//! instructed movements) and `−1` for thighs (their tops tip backward); this is
//! [`crate::forward_calibration::ski_pitch_sign`] negated. The tip line is
//! unaffected by twist about the bone and by the yaw part of `D`, unlike the
//! rotation-axis formulation of the held two-pose solve.
//!
//! Everything here is conditional on the movement model: a sideways lean is
//! indistinguishable from another sensor heading, and nothing has been
//! validated on a worn body. Gates are engineering tolerances, not accuracy
//! bounds. See `docs/research/motion-calibration.md`.

#![allow(clippy::needless_range_loop)] // Small fixed per-segment arrays.

use crate::chain::{CALF, CHEST, THIGH};
use crate::forward_calibration::{ForwardAlignment, ski_pitch_sign};
use crate::{CoreError, Role};
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Number of core segments, in [`Role::CORE`] order.
pub const SEGMENTS: usize = 6;
/// Fixed directions (5° apart over 180°) at which `Σ w |p·u|` is accumulated
/// so that the one-sidedness denominator `Σ w |p·e|` is available for any `e`.
const PROJECTION_BINS: usize = 36;
const HISTORY_INTERVAL_SECS: f64 = 0.1;
const HISTORY_CAPACITY: usize = 32;
/// Minimum paired frames before a knee check is reported.
const MIN_KNEE_FRAMES: usize = 20;
/// Minimum paired frames with a bent knee before the axis check is reported.
const MIN_KNEE_AXIS_FRAMES: usize = 10;
const KNEE_AXIS_MIN_BEND_DEGREES: f64 = 15.0;
const MIN_SWAP_FRAMES: usize = 50;
const MAX_POSITION_M: f32 = 10_000.0;

/// `+1` when the top of the bone tips FORWARD in every instructed movement
/// (chest, hip, calves) and `−1` when it tips BACKWARD (thighs).
pub fn tip_sign(role: Role) -> Option<f32> {
    ski_pitch_sign(role).map(|sign| -sign)
}

/// Which absolute forward reference the solve uses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionReference {
    /// Headset yaw at neutral (assumes head and torso faced the same way).
    HeadsetYaw,
    /// Inverse-variance fusion of headset yaw and neck displacement when the
    /// displacement estimate is available and agrees; otherwise headset yaw.
    #[default]
    Fused,
    /// Neck-pivot displacement heading only; the solve fails without it.
    NeckDisplacement,
}

/// Tunable gates. Defaults are proposed engineering tolerances, not
/// demonstrated accuracy bounds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MotionCalibrationConfig {
    /// Samples enter the line fit only between these tilts from neutral.
    pub min_evidence_tilt_degrees: f32,
    pub max_evidence_tilt_degrees: f32,
    /// Excitation: peak tilt and time spent beyond `tilted_threshold_degrees`.
    pub min_peak_tilt_degrees: f32,
    pub tilted_threshold_degrees: f32,
    pub min_tilted_secs: f64,
    /// `λ_min / λ_max` of the origin-anchored scatter.
    pub max_planarity: f32,
    /// `|m·e| / Σ|p·e|`; below it the sign is undetermined.
    pub min_one_sidedness: f32,
    /// Two interleaved accumulators over alternating blocks.
    pub repeatability_block_secs: f64,
    pub min_block_evidence_secs: f64,
    pub max_repeatability_degrees: f32,
    /// RMS out-of-sagittal-plane tilt of the long axis with the solved heading.
    pub max_residual_degrees: f32,
    pub stability_window_secs: f64,
    pub max_stability_degrees: f32,
    /// A stream silent for longer restarts its repeatability blocks; the
    /// sample after the gap carries no weight.
    pub max_gap_secs: f64,
    /// Upper bound of a single sample's evidence weight (its dt).
    pub max_sample_weight_secs: f64,
    /// Samples older than the newest accepted time minus this are stale.
    pub stale_after_secs: f64,
    /// A core stream silent for longer than this blocks readiness.
    pub ready_max_silence_secs: f64,
    /// Exponential forgetting time constant, in seconds of EVIDENCE (not wall
    /// time): standing still never erodes the estimate, but a bad stretch is
    /// outweighed by later good evidence.
    pub evidence_memory_secs: f64,
    /// Body-turn detection: first-epoch evidence frozen after this many
    /// seconds, compared with a short-memory recent accumulator.
    pub turn_epoch_secs: f64,
    pub turn_recent_memory_secs: f64,
    pub turn_threshold_degrees: f32,
    pub turn_consistency_degrees: f32,
    /// Knee checks with the solved thigh and calf headings.
    pub knee_max_hyperextension_degrees: f32,
    pub knee_max_axis_degrees: f32,
    /// Resolve two-sided (sign-undetermined) thigh/calf lines from the knee's
    /// one-way hinge when one sign combination is unambiguous.
    pub knee_sign_rule: bool,
    pub knee_rule_max_hyperextension_rms_degrees: f32,
    pub knee_rule_min_rejected_rms_degrees: f32,
    pub knee_min_flexion_degrees: f32,
    /// Calf peak tilt exceeding the thigh's by this raises a swap suspicion.
    pub swap_tilt_margin_degrees: f32,
    /// Paired frames kept for cross-segment checks (bounded ring).
    pub snapshot_interval_secs: f64,
    pub snapshot_max_skew_secs: f64,
    pub max_snapshots: usize,
    /// Headset-assisted absolute forward.
    pub headset_min_chest_tilt_degrees: f32,
    pub headset_min_displacement_m: f32,
    /// Assumed systematic neck-pivot error (model, sway), meters.
    pub headset_noise_m: f32,
    /// Assumed head-versus-torso yaw sigma at neutral.
    pub head_torso_sigma_degrees: f32,
    pub headset_max_disagreement_degrees: f32,
    pub headset_min_evidence_secs: f64,
    pub headset_chest_max_skew_secs: f64,
    pub reference: MotionReference,
}

impl Default for MotionCalibrationConfig {
    fn default() -> Self {
        Self {
            min_evidence_tilt_degrees: 8.0,
            max_evidence_tilt_degrees: 75.0,
            min_peak_tilt_degrees: 15.0,
            tilted_threshold_degrees: 10.0,
            min_tilted_secs: 0.4,
            max_planarity: 0.15,
            min_one_sidedness: 0.85,
            repeatability_block_secs: 0.5,
            min_block_evidence_secs: 0.15,
            max_repeatability_degrees: 8.0,
            max_residual_degrees: 12.0,
            stability_window_secs: 1.0,
            max_stability_degrees: 3.0,
            max_gap_secs: 0.25,
            max_sample_weight_secs: 0.05,
            stale_after_secs: 0.25,
            ready_max_silence_secs: 0.5,
            evidence_memory_secs: 10.0,
            turn_epoch_secs: 1.5,
            turn_recent_memory_secs: 1.5,
            turn_threshold_degrees: 25.0,
            turn_consistency_degrees: 15.0,
            knee_max_hyperextension_degrees: 10.0,
            // 25° rejected a real first attempt (27° between a thigh and its calf
            // during knee lifts) that a retry passed at 15°. Real shanks do not swing
            // in exactly the thigh's plane; a body turn has its own detector.
            knee_max_axis_degrees: 40.0,
            knee_sign_rule: true,
            knee_rule_max_hyperextension_rms_degrees: 6.0,
            knee_rule_min_rejected_rms_degrees: 15.0,
            knee_min_flexion_degrees: 20.0,
            swap_tilt_margin_degrees: 10.0,
            snapshot_interval_secs: 0.04,
            snapshot_max_skew_secs: 0.06,
            max_snapshots: 1500,
            headset_min_chest_tilt_degrees: 15.0,
            headset_min_displacement_m: 0.02,
            headset_noise_m: 0.02,
            head_torso_sigma_degrees: 8.0,
            headset_max_disagreement_degrees: 25.0,
            headset_min_evidence_secs: 0.3,
            headset_chest_max_skew_secs: 0.1,
            reference: MotionReference::Fused,
        }
    }
}

impl MotionCalibrationConfig {
    pub fn validate(&self) -> Result<(), CoreError> {
        fn check(ok: bool, name: &str) -> Result<(), CoreError> {
            if ok {
                Ok(())
            } else {
                Err(CoreError(format!(
                    "Invalid motion calibration setting {name}"
                )))
            }
        }
        let angle = |v: f32| v.is_finite() && v > 0.0 && v < 180.0;
        let secs = |v: f64| v.is_finite() && v > 0.0 && v <= 3600.0;
        let ratio = |v: f32| v.is_finite() && v > 0.0 && v <= 1.0;
        let meters = |v: f32| v.is_finite() && v > 0.0 && v < 10.0;
        check(
            angle(self.min_evidence_tilt_degrees)
                && angle(self.max_evidence_tilt_degrees)
                && self.min_evidence_tilt_degrees < self.max_evidence_tilt_degrees
                && self.max_evidence_tilt_degrees < 90.0,
            "evidence tilt window",
        )?;
        check(angle(self.min_peak_tilt_degrees), "min_peak_tilt_degrees")?;
        check(
            angle(self.tilted_threshold_degrees),
            "tilted_threshold_degrees",
        )?;
        check(secs(self.min_tilted_secs), "min_tilted_secs")?;
        check(ratio(self.max_planarity), "max_planarity")?;
        check(ratio(self.min_one_sidedness), "min_one_sidedness")?;
        check(
            secs(self.repeatability_block_secs),
            "repeatability_block_secs",
        )?;
        check(
            secs(self.min_block_evidence_secs),
            "min_block_evidence_secs",
        )?;
        check(
            angle(self.max_repeatability_degrees),
            "max_repeatability_degrees",
        )?;
        check(angle(self.max_residual_degrees), "max_residual_degrees")?;
        check(secs(self.stability_window_secs), "stability_window_secs")?;
        check(angle(self.max_stability_degrees), "max_stability_degrees")?;
        check(secs(self.max_gap_secs), "max_gap_secs")?;
        check(secs(self.max_sample_weight_secs), "max_sample_weight_secs")?;
        check(secs(self.stale_after_secs), "stale_after_secs")?;
        check(secs(self.ready_max_silence_secs), "ready_max_silence_secs")?;
        check(secs(self.evidence_memory_secs), "evidence_memory_secs")?;
        check(secs(self.turn_epoch_secs), "turn_epoch_secs")?;
        check(
            secs(self.turn_recent_memory_secs),
            "turn_recent_memory_secs",
        )?;
        check(angle(self.turn_threshold_degrees), "turn_threshold_degrees")?;
        check(
            angle(self.turn_consistency_degrees),
            "turn_consistency_degrees",
        )?;
        check(
            angle(self.knee_max_hyperextension_degrees),
            "knee_max_hyperextension_degrees",
        )?;
        check(angle(self.knee_max_axis_degrees), "knee_max_axis_degrees")?;
        check(
            angle(self.knee_rule_max_hyperextension_rms_degrees)
                && angle(self.knee_rule_min_rejected_rms_degrees)
                && self.knee_rule_max_hyperextension_rms_degrees
                    < self.knee_rule_min_rejected_rms_degrees,
            "knee rule thresholds",
        )?;
        check(
            angle(self.knee_min_flexion_degrees),
            "knee_min_flexion_degrees",
        )?;
        check(
            angle(self.swap_tilt_margin_degrees),
            "swap_tilt_margin_degrees",
        )?;
        check(secs(self.snapshot_interval_secs), "snapshot_interval_secs")?;
        check(secs(self.snapshot_max_skew_secs), "snapshot_max_skew_secs")?;
        check(
            (16..=200_000).contains(&self.max_snapshots),
            "max_snapshots",
        )?;
        check(
            angle(self.headset_min_chest_tilt_degrees),
            "headset_min_chest_tilt_degrees",
        )?;
        check(
            meters(self.headset_min_displacement_m),
            "headset_min_displacement_m",
        )?;
        check(meters(self.headset_noise_m), "headset_noise_m")?;
        check(
            angle(self.head_torso_sigma_degrees),
            "head_torso_sigma_degrees",
        )?;
        check(
            angle(self.headset_max_disagreement_degrees),
            "headset_max_disagreement_degrees",
        )?;
        check(
            secs(self.headset_min_evidence_secs),
            "headset_min_evidence_secs",
        )?;
        check(
            secs(self.headset_chest_max_skew_secs),
            "headset_chest_max_skew_secs",
        )
    }
}

/// Result of feeding one sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SampleOutcome {
    /// Accepted. `evidence` when it entered the line fit with nonzero weight.
    Accepted {
        evidence: bool,
    },
    /// Accepted as the first sample after a gap: zero weight, the
    /// repeatability blocks restart, accumulated evidence is kept.
    ResumedAfterGap,
    Rejected {
        reason: RejectReason,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RejectReason {
    NotActive,
    InvalidRole,
    NonFinite,
    BeforeNeutral,
    OutOfOrder,
    Stale,
    NoNeckReference,
    InvalidDimensions,
}

/// Which gate blocks a segment (or the whole calibration).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionGate {
    NotStarted,
    NoData,
    Stale,
    Excitation,
    Planarity,
    OneSidedness,
    Repeatability,
    ModelResidual,
    Stability,
    KneeHyperextension,
    KneeAxis,
    KneeUnchecked,
    BodyTurn,
    Reference,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionBlocker {
    pub gate: MotionGate,
    pub message: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionWarningKind {
    BodyTurn,
    KneeHyperextension,
    KneeAxisMismatch,
    SignFromKneeRule,
    SuspectedThighCalfSwap,
    SuspectedLeftRightSwap,
    HeadsetDisagreement,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionWarning {
    pub kind: MotionWarningKind,
    pub roles: Vec<Role>,
    pub message: String,
}

/// Where a segment's forward/backward sign came from.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SignEvidence {
    /// No usable line yet.
    Unknown,
    /// The movement was one-sided along the line; `sign` multiplies the
    /// principal direction to give body forward.
    OneSided { sign: f32 },
    /// Two-sided movement; the knee's one-way hinge selected the sign.
    KneeRule { sign: f32 },
    /// Two-sided movement and no unambiguous knee resolution.
    Undetermined,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionSegmentStatus {
    pub role: Role,
    /// 0..1: up to 0.5 for excitation, the rest for quality gates.
    pub progress: f32,
    pub ready: bool,
    pub blockers: Vec<MotionBlocker>,
    pub max_tilt_degrees: f32,
    pub tilted_secs: f32,
    /// Decayed evidence weight in the line fit (seconds).
    pub evidence_secs: f32,
    pub planarity: Option<f32>,
    pub one_sidedness: Option<f32>,
    pub sign: SignEvidence,
    pub repeatability_degrees: Option<f32>,
    pub residual_degrees: Option<f32>,
    pub stability_degrees: Option<f32>,
    /// RMS twist about the long axis relative to neutral (diagnostic only).
    pub twist_rms_degrees: Option<f32>,
    /// Heading of body-forward in this sensor's world frame.
    pub sensor_forward_heading_radians: Option<f32>,
    /// Heading alignment with the currently selected reference.
    pub heading_alignment_radians: Option<f32>,
    pub accepted_samples: u64,
    pub rejected_samples: u64,
    pub gaps: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KneeStatus {
    pub thigh: Role,
    pub calf: Role,
    pub frames: usize,
    /// 95th percentile of backward knee bend with the solved headings.
    pub hyperextension_p95_degrees: Option<f32>,
    pub flexion_p95_degrees: Option<f32>,
    /// Angle between thigh and calf flexion axes (bend direction, mod 180°).
    pub axis_disagreement_degrees: Option<f32>,
    pub sign_resolved_by_knee: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeadsetForwardStatus {
    pub available: bool,
    /// Why the displacement heading is unavailable.
    pub reason: Option<String>,
    pub evidence_secs: f32,
    pub max_displacement_m: f32,
    pub planarity: Option<f32>,
    pub one_sidedness: Option<f32>,
    /// Body-forward heading from the neck pivot's horizontal displacement.
    pub displacement_heading_radians: Option<f32>,
    pub displacement_sigma_radians: Option<f32>,
    /// `wrap(displacement − headset yaw reference)`.
    pub difference_from_yaw_radians: Option<f32>,
    pub fused_heading_radians: Option<f32>,
    pub fused_sigma_radians: Option<f32>,
    /// The two disagree by more than the configured limit; fusion withheld.
    pub disagreement_flagged: bool,
    pub accepted_samples: u64,
    pub rejected_samples: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionCalibrationStatus {
    pub active: bool,
    pub ready: bool,
    pub segments: Vec<MotionSegmentStatus>,
    /// Calibration-wide blockers (turn, reference, not started).
    pub blockers: Vec<MotionBlocker>,
    pub warnings: Vec<MotionWarning>,
    pub knees: Vec<KneeStatus>,
    pub yaw_reference_heading_radians: Option<f32>,
    /// Reference that `solve()` would use, after fallback.
    pub selected_reference: MotionReference,
    pub selected_reference_heading_radians: Option<f32>,
    pub headset: Option<HeadsetForwardStatus>,
    /// Latched common-mode rotation of all tip lines (positive = left turn).
    pub body_turn_degrees: Option<f32>,
    pub retained_snapshots: usize,
}

#[derive(Clone, Debug)]
pub struct MotionCalibrationSolution {
    /// One alignment per role in [`Role::CORE`] order.
    pub alignments: [ForwardAlignment; SEGMENTS],
    pub reference: MotionReference,
    pub reference_heading_radians: f32,
    pub yaw_reference_heading_radians: f32,
    pub diagnostics: MotionCalibrationStatus,
}

impl MotionCalibrationSolution {
    pub fn alignment(&self, role: Role) -> Option<&ForwardAlignment> {
        Role::CORE
            .iter()
            .position(|r| *r == role)
            .map(|i| &self.alignments[i])
    }
}

// ---------------------------------------------------------------------------------------------
// Accumulators.

#[derive(Clone, Copy, Debug)]
struct LineAccumulator {
    sxx: f64,
    sxz: f64,
    szz: f64,
    mx: f64,
    mz: f64,
    abs_projection: [f64; PROJECTION_BINS],
    weight: f64,
}

impl Default for LineAccumulator {
    fn default() -> Self {
        Self {
            sxx: 0.0,
            sxz: 0.0,
            szz: 0.0,
            mx: 0.0,
            mz: 0.0,
            abs_projection: [0.0; PROJECTION_BINS],
            weight: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LineFit {
    /// Direction angle of the principal axis `e = (cos, sin)` in (x, z).
    angle: f64,
    lambda_max: f64,
    lambda_min: f64,
    /// `m·e`.
    signed: f64,
    one_sidedness: f64,
    weight: f64,
}

impl LineFit {
    fn planarity(&self) -> f64 {
        self.lambda_min / self.lambda_max
    }
}

impl LineAccumulator {
    fn scale(&mut self, factor: f64) {
        self.sxx *= factor;
        self.sxz *= factor;
        self.szz *= factor;
        self.mx *= factor;
        self.mz *= factor;
        self.weight *= factor;
        for bin in &mut self.abs_projection {
            *bin *= factor;
        }
    }

    fn add(&mut self, x: f64, z: f64, w: f64) {
        self.sxx += w * x * x;
        self.sxz += w * x * z;
        self.szz += w * z * z;
        self.mx += w * x;
        self.mz += w * z;
        self.weight += w;
        let (step_sin, step_cos) = (PI / PROJECTION_BINS as f64).sin_cos();
        let (mut c, mut s) = (1.0_f64, 0.0_f64);
        for bin in &mut self.abs_projection {
            *bin += w * (x * c + z * s).abs();
            (c, s) = (c * step_cos - s * step_sin, s * step_cos + c * step_sin);
        }
    }

    fn forget_and_add(&mut self, x: f64, z: f64, w: f64, memory_secs: f64) {
        self.scale((-w / memory_secs).exp());
        self.add(x, z, w);
    }

    fn fit(&self) -> Option<LineFit> {
        if self.weight.is_nan() || self.weight <= 1.0e-9 {
            return None;
        }
        let (a, b, c) = (self.sxx, self.sxz, self.szz);
        let mean = 0.5 * (a + c);
        let half_difference = 0.5 * (a - c);
        let radius = half_difference.hypot(b);
        let lambda_max = mean + radius;
        let lambda_min = (mean - radius).max(0.0);
        if !lambda_max.is_finite() || lambda_max <= 1.0e-12 {
            return None;
        }
        let angle = 0.5 * (2.0 * b).atan2(a - c);
        let (sin, cos) = angle.sin_cos();
        let signed = self.mx * cos + self.mz * sin;
        let absolute = self.absolute_projection(angle);
        let one_sidedness = if absolute > 1.0e-12 {
            (signed.abs() / absolute).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let fit = LineFit {
            angle,
            lambda_max,
            lambda_min,
            signed,
            one_sidedness,
            weight: self.weight,
        };
        [angle, lambda_min, signed, one_sidedness]
            .iter()
            .all(|v| v.is_finite())
            .then_some(fit)
    }

    /// `Σ w |p·u(angle)|`, linearly interpolated between the 5° bins.
    fn absolute_projection(&self, angle: f64) -> f64 {
        let step = PI / PROJECTION_BINS as f64;
        let position = angle.rem_euclid(PI) / step;
        let index = (position.floor() as usize).min(PROJECTION_BINS - 1);
        let fraction = (position - index as f64).clamp(0.0, 1.0);
        let next = (index + 1) % PROJECTION_BINS;
        self.abs_projection[index] * (1.0 - fraction) + self.abs_projection[next] * fraction
    }
}

/// Signed difference of two line angles, wrapped to [−90°, 90°).
fn line_difference(a: f64, b: f64) -> f64 {
    (a - b + FRAC_PI_2).rem_euclid(PI) - FRAC_PI_2
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

/// Heading `atan2(−f.x, −f.z)` of `f = sign · (cos angle, 0, sin angle)`.
fn forward_heading(angle: f64, sign: f64) -> f64 {
    (-sign * angle.cos()).atan2(-sign * angle.sin())
}

fn unit(q: Quat) -> Option<Quat> {
    let length_squared = q.length_squared();
    (q.is_finite() && length_squared.is_finite() && length_squared > 1.0e-8)
        .then(|| q.normalize())
        .filter(|q| q.is_finite())
}

fn finite32(value: f64) -> Option<f32> {
    let value = value as f32;
    value.is_finite().then_some(value)
}

fn degrees32(radians: f64) -> Option<f32> {
    finite32(radians.to_degrees())
}

#[derive(Clone, Debug)]
struct SegmentState {
    neutral_inverse: Quat,
    last_at: Option<f64>,
    latest: Option<(f64, Quat)>,
    main: LineAccumulator,
    blocks: [LineAccumulator; 2],
    block_parity: usize,
    block_started_at: f64,
    first_epoch: LineAccumulator,
    first_epoch_frozen: bool,
    recent: LineAccumulator,
    twist_squared: f64,
    tilt_squared: f64,
    moment_weight: f64,
    max_tilt: f64,
    tilted_secs: f64,
    history: VecDeque<(f64, f64)>,
    last_history_at: Option<f64>,
    accepted: u64,
    rejected: u64,
    gaps: u32,
}

impl SegmentState {
    fn new(neutral: Quat) -> Self {
        Self {
            neutral_inverse: neutral.conjugate(),
            last_at: None,
            latest: None,
            main: LineAccumulator::default(),
            blocks: [LineAccumulator::default(); 2],
            block_parity: 0,
            block_started_at: 0.0,
            first_epoch: LineAccumulator::default(),
            first_epoch_frozen: false,
            recent: LineAccumulator::default(),
            twist_squared: 0.0,
            tilt_squared: 0.0,
            moment_weight: 0.0,
            max_tilt: 0.0,
            tilted_secs: 0.0,
            history: VecDeque::with_capacity(HISTORY_CAPACITY),
            last_history_at: None,
            accepted: 0,
            rejected: 0,
            gaps: 0,
        }
    }

    fn accumulate(
        &mut self,
        config: &MotionCalibrationConfig,
        at: f64,
        delta: Quat,
        tip: Vec3,
        tilt: f64,
        w: f64,
    ) -> bool {
        if at - self.block_started_at >= config.repeatability_block_secs {
            self.block_started_at = at;
            self.block_parity ^= 1;
        }
        if tilt > f64::from(config.tilted_threshold_degrees).to_radians() {
            self.tilted_secs += w;
        }
        let evidence = w > 0.0
            && tilt >= f64::from(config.min_evidence_tilt_degrees).to_radians()
            && tilt <= f64::from(config.max_evidence_tilt_degrees).to_radians();
        if !evidence {
            return false;
        }
        let (x, z) = (f64::from(tip.x), f64::from(tip.z));
        let memory = config.evidence_memory_secs;
        let factor = (-w / memory).exp();
        self.main.scale(factor);
        self.main.add(x, z, w);
        let delta = if delta.w < 0.0 { -delta } else { delta };
        let twist = 2.0 * f64::from(delta.y).atan2(f64::from(delta.w));
        self.twist_squared = self.twist_squared * factor + w * twist * twist;
        self.tilt_squared = self.tilt_squared * factor + w * tilt * tilt;
        self.moment_weight = self.moment_weight * factor + w;
        self.blocks[self.block_parity].forget_and_add(x, z, w, memory);
        if !self.first_epoch_frozen {
            self.first_epoch.add(x, z, w);
            self.first_epoch_frozen = self.first_epoch.weight >= config.turn_epoch_secs;
        }
        self.recent
            .forget_and_add(x, z, w, config.turn_recent_memory_secs);
        true
    }
}

#[derive(Clone, Copy, Debug)]
struct Snapshot {
    deltas: [Quat; SEGMENTS],
    valid: u8,
}

#[derive(Clone, Debug)]
struct HeadsetState {
    neutral_neck: Vec3,
    last_at: Option<f64>,
    accumulator: LineAccumulator,
    evidence_secs: f64,
    max_displacement: f64,
    accepted: u64,
    rejected: u64,
}

#[derive(Clone, Debug)]
struct Session {
    began_at: f64,
    neutral: [Quat; SEGMENTS],
    yaw_reference: f64,
    segments: Vec<SegmentState>,
    newest_at: Option<f64>,
    snapshots: VecDeque<Snapshot>,
    last_snapshot_at: Option<f64>,
    headset: Option<HeadsetState>,
    turn_degrees: Option<f64>,
}

/// Incremental motion calibrator. See the module documentation.
#[derive(Clone, Debug)]
pub struct MotionCalibrator {
    config: MotionCalibrationConfig,
    session: Option<Session>,
}

impl MotionCalibrator {
    pub fn new(config: MotionCalibrationConfig) -> Result<Self, CoreError> {
        config.validate()?;
        Ok(Self {
            config,
            session: None,
        })
    }

    pub fn config(&self) -> &MotionCalibrationConfig {
        &self.config
    }

    pub fn is_active(&self) -> bool {
        self.session.is_some()
    }

    /// Neck pivot `position + R·(0, −head_to_neck, +eye_forward)`, the same
    /// convention as [`crate::chain::solve_chain`]. `None` for invalid input.
    pub fn neck_pivot(
        position: Vec3,
        orientation: Quat,
        head_to_neck_m: f32,
        eye_forward_m: f32,
    ) -> Option<Vec3> {
        let orientation = unit(orientation)?;
        let dimensions_ok = head_to_neck_m.is_finite()
            && (0.0..=0.5).contains(&head_to_neck_m)
            && eye_forward_m.is_finite()
            && (-0.3..=0.3).contains(&eye_forward_m);
        let position_ok = position.is_finite() && position.abs().max_element() <= MAX_POSITION_M;
        if !dimensions_ok || !position_ok {
            return None;
        }
        let neck = position + orientation * Vec3::new(0.0, -head_to_neck_m, eye_forward_m);
        neck.is_finite().then_some(neck)
    }

    /// Starts a session. `neutral` holds the neutral-pose sensor quaternions
    /// in [`Role::CORE`] order, captured at `neutral_at_secs`; earlier samples
    /// are rejected. `reference_heading` is the headset yaw at neutral (the
    /// body-forward assumption of the existing neutral capture).
    /// `neck_at_neutral` (see [`Self::neck_pivot`]) enables the headset
    /// displacement forward estimate. Restarts any previous session; on error
    /// the calibrator is left inactive. A sensor restart (new sensor world)
    /// invalidates `neutral`: call `begin` again after a new neutral capture.
    pub fn begin(
        &mut self,
        neutral_at_secs: f64,
        neutral: [Quat; SEGMENTS],
        reference_heading: f32,
        neck_at_neutral: Option<Vec3>,
    ) -> Result<(), CoreError> {
        self.session = None;
        if !neutral_at_secs.is_finite() {
            return Err(CoreError("Invalid neutral capture time".into()));
        }
        if !reference_heading.is_finite() {
            return Err(CoreError("Invalid forward reference heading".into()));
        }
        let mut checked = [Quat::IDENTITY; SEGMENTS];
        for (i, q) in neutral.iter().enumerate() {
            checked[i] = unit(*q).ok_or_else(|| {
                CoreError(format!(
                    "Invalid neutral orientation for {:?}",
                    Role::CORE[i]
                ))
            })?;
        }
        let headset = match neck_at_neutral {
            Some(neck) if neck.is_finite() && neck.abs().max_element() <= MAX_POSITION_M => {
                Some(HeadsetState {
                    neutral_neck: neck,
                    last_at: None,
                    accumulator: LineAccumulator::default(),
                    evidence_secs: 0.0,
                    max_displacement: 0.0,
                    accepted: 0,
                    rejected: 0,
                })
            }
            Some(_) => return Err(CoreError("Invalid neutral neck position".into())),
            None => None,
        };
        let capacity = self.config.max_snapshots.min(4096);
        self.session = Some(Session {
            began_at: neutral_at_secs,
            neutral: checked,
            yaw_reference: wrap(f64::from(reference_heading)),
            segments: checked.iter().map(|q| SegmentState::new(*q)).collect(),
            newest_at: None,
            snapshots: VecDeque::with_capacity(capacity),
            last_snapshot_at: None,
            headset,
            turn_degrees: None,
        });
        Ok(())
    }

    /// Discards the session and all evidence.
    pub fn reset(&mut self) {
        self.session = None;
    }

    /// Feeds one measured sensor quaternion (sensor world, XYZW semantics of
    /// `glam::Quat`). `role_index` follows [`Role::CORE`]. Timestamps must come
    /// from the runtime's single monotonic arrival clock.
    pub fn observe_sensor(
        &mut self,
        role_index: usize,
        at_secs: f64,
        orientation: Quat,
    ) -> SampleOutcome {
        let Self { config, session } = self;
        let Some(session) = session.as_mut() else {
            return rejected(RejectReason::NotActive);
        };
        if role_index >= SEGMENTS {
            return rejected(RejectReason::InvalidRole);
        }
        let outcome = session.observe_sensor(config, role_index, at_secs, orientation);
        let segment = &mut session.segments[role_index];
        if matches!(outcome, SampleOutcome::Rejected { .. }) {
            segment.rejected = segment.rejected.saturating_add(1);
        } else {
            segment.accepted = segment.accepted.saturating_add(1);
        }
        outcome
    }

    /// Feeds one headset pose (standing space). The neck pivot is computed
    /// with the caller's dimensions, which should match those used for
    /// `neck_at_neutral`.
    pub fn observe_headset(
        &mut self,
        at_secs: f64,
        position: Vec3,
        orientation: Quat,
        head_to_neck_m: f32,
        eye_forward_m: f32,
    ) -> SampleOutcome {
        let Self { config, session } = self;
        let Some(session) = session.as_mut() else {
            return rejected(RejectReason::NotActive);
        };
        let outcome = session.observe_headset(
            config,
            at_secs,
            position,
            orientation,
            head_to_neck_m,
            eye_forward_m,
        );
        if let Some(headset) = session.headset.as_mut() {
            if matches!(outcome, SampleOutcome::Rejected { .. }) {
                headset.rejected = headset.rejected.saturating_add(1);
            } else {
                headset.accepted = headset.accepted.saturating_add(1);
            }
        }
        outcome
    }

    pub fn status(&self) -> MotionCalibrationStatus {
        match &self.session {
            None => inactive_status(self.config.reference),
            Some(session) => evaluate(&self.config, session, self.config.reference).status,
        }
    }

    /// Solves with the configured reference. Atomic: all six alignments or an
    /// error naming every blocking segment.
    pub fn solve(&self) -> Result<MotionCalibrationSolution, CoreError> {
        self.solve_with(self.config.reference)
    }

    pub fn solve_with(
        &self,
        reference: MotionReference,
    ) -> Result<MotionCalibrationSolution, CoreError> {
        let Some(session) = &self.session else {
            return Err(CoreError(
                "Capture neutral and begin motion calibration first".into(),
            ));
        };
        let evaluation = evaluate(&self.config, session, reference);
        if !evaluation.status.ready {
            return Err(CoreError(format!(
                "Motion calibration is not ready: {}",
                blocker_summary(&evaluation.status)
            )));
        }
        let heading = evaluation
            .reference_heading
            .ok_or_else(|| CoreError("No forward reference heading".into()))?;
        let reference_rotation = Quat::from_rotation_y(heading as f32);
        let mut alignments = [ForwardAlignment {
            heading_alignment_radians: 0.0,
            mounting_xyzw: [0.0, 0.0, 0.0, 1.0],
            excitation_degrees: 0.0,
            axis_vertical_degrees: 0.0,
            model_residual_degrees: 0.0,
        }; SEGMENTS];
        for i in 0..SEGMENTS {
            let role = Role::CORE[i];
            let sensor_heading = evaluation.sensor_headings[i]
                .ok_or_else(|| CoreError(format!("{role:?} has no forward direction")))?;
            let psi = wrap(heading - sensor_heading) as f32;
            let alignment = Quat::from_rotation_y(psi);
            let neutral = session.neutral[i];
            let mounting = ((alignment * neutral).conjugate() * reference_rotation).normalize();
            let check = (alignment * neutral * mounting).normalize();
            if !mounting.is_finite()
                || !check.is_finite()
                || 2.0 * check.dot(reference_rotation).abs().clamp(0.0, 1.0).acos() > 0.001
            {
                return Err(CoreError(format!(
                    "{role:?} motion calibration is numerically unstable"
                )));
            }
            let segment = &session.segments[i];
            let tilt_rms = (segment.tilt_squared / segment.moment_weight.max(1.0e-12)).sqrt();
            let twist_rms = (segment.twist_squared / segment.moment_weight.max(1.0e-12)).sqrt();
            let status = &evaluation.status.segments[i];
            alignments[i] = ForwardAlignment {
                heading_alignment_radians: psi,
                mounting_xyzw: mounting.to_array(),
                excitation_degrees: status.max_tilt_degrees,
                // Equivalent rotation-axis elevation: RMS twist versus RMS tilt.
                axis_vertical_degrees: degrees32(twist_rms.atan2(tilt_rms)).unwrap_or(0.0),
                model_residual_degrees: status.residual_degrees.unwrap_or(0.0),
            };
        }
        Ok(MotionCalibrationSolution {
            alignments,
            reference: evaluation.reference_used,
            reference_heading_radians: heading as f32,
            yaw_reference_heading_radians: session.yaw_reference as f32,
            diagnostics: evaluation.status,
        })
    }
}

fn rejected(reason: RejectReason) -> SampleOutcome {
    SampleOutcome::Rejected { reason }
}

impl Session {
    fn check_time(&self, config: &MotionCalibrationConfig, at: f64) -> Result<(), RejectReason> {
        if !at.is_finite() {
            return Err(RejectReason::NonFinite);
        }
        if at < self.began_at {
            return Err(RejectReason::BeforeNeutral);
        }
        if self
            .newest_at
            .is_some_and(|newest| at < newest - config.stale_after_secs)
        {
            return Err(RejectReason::Stale);
        }
        Ok(())
    }

    fn observe_sensor(
        &mut self,
        config: &MotionCalibrationConfig,
        index: usize,
        at: f64,
        orientation: Quat,
    ) -> SampleOutcome {
        if let Err(reason) = self.check_time(config, at) {
            return rejected(reason);
        }
        let Some(q) = unit(orientation) else {
            return rejected(RejectReason::NonFinite);
        };
        let segment = &mut self.segments[index];
        if segment.last_at.is_some_and(|last| at <= last) {
            return rejected(RejectReason::OutOfOrder);
        }
        let delta = (q * segment.neutral_inverse).normalize();
        let tip = delta * Vec3::Y;
        if !delta.is_finite() || !tip.is_finite() {
            return rejected(RejectReason::NonFinite);
        }
        let previous = segment.last_at.replace(at);
        self.newest_at = Some(self.newest_at.map_or(at, |newest| newest.max(at)));
        segment.latest = Some((at, delta));
        let tilt = f64::from(tip.y).clamp(-1.0, 1.0).acos();
        segment.max_tilt = segment.max_tilt.max(tilt);
        let outcome = match previous {
            None => {
                segment.block_started_at = at;
                SampleOutcome::Accepted { evidence: false }
            }
            Some(last) if at - last > config.max_gap_secs => {
                segment.gaps = segment.gaps.saturating_add(1);
                segment.block_started_at = at;
                segment.block_parity ^= 1;
                SampleOutcome::ResumedAfterGap
            }
            Some(last) => {
                let w = (at - last).min(config.max_sample_weight_secs);
                let evidence = segment.accumulate(config, at, delta, tip, tilt, w);
                SampleOutcome::Accepted { evidence }
            }
        };
        if segment
            .last_history_at
            .is_none_or(|t| at - t >= HISTORY_INTERVAL_SECS)
        {
            segment.last_history_at = Some(at);
            if let Some(fit) = segment.main.fit() {
                segment.history.push_back((at, fit.angle));
                while segment.history.len() > HISTORY_CAPACITY {
                    segment.history.pop_front();
                }
            }
            self.check_turn(config);
        }
        self.maybe_snapshot(config, at);
        outcome
    }

    fn maybe_snapshot(&mut self, config: &MotionCalibrationConfig, at: f64) {
        if self
            .last_snapshot_at
            .is_some_and(|t| at - t < config.snapshot_interval_secs)
        {
            return;
        }
        let mut deltas = [Quat::IDENTITY; SEGMENTS];
        let mut valid = 0_u8;
        for (i, segment) in self.segments.iter().enumerate() {
            if let Some((t, delta)) = segment.latest
                && (at - t).abs() <= config.snapshot_max_skew_secs
            {
                deltas[i] = delta;
                valid |= 1 << i;
            }
        }
        let leg = |l: usize| {
            let mask = (1_u8 << THIGH[l]) | (1_u8 << CALF[l]);
            valid & mask == mask
        };
        if !(leg(0) || leg(1)) {
            return;
        }
        self.last_snapshot_at = Some(at);
        self.snapshots.push_back(Snapshot { deltas, valid });
        while self.snapshots.len() > config.max_snapshots {
            self.snapshots.pop_front();
        }
    }

    /// Latches a body turn when the tip lines of several segments rotate by a
    /// common angle between the first evidence epoch and the recent evidence:
    /// a whole-body yaw rotates every segment's line by the same angle in its
    /// own sensor world, which independent sensor errors do not.
    /// Once latched, the reported angle follows the largest consistent rotation seen.
    fn check_turn(&mut self, config: &MotionCalibrationConfig) {
        let mut rotations = [0.0_f64; SEGMENTS];
        let mut count = 0;
        for segment in &self.segments {
            if !segment.first_epoch_frozen
                || segment.recent.weight < 0.5 * config.turn_recent_memory_secs
            {
                continue;
            }
            let (Some(first), Some(recent)) = (segment.first_epoch.fit(), segment.recent.fit())
            else {
                continue;
            };
            let limit = 2.0 * f64::from(config.max_planarity);
            if first.planarity() > limit || recent.planarity() > limit {
                continue;
            }
            // A left body turn by β moves the (x, z) line angle by −β.
            rotations[count] = -line_difference(recent.angle, first.angle);
            count += 1;
        }
        if count < 3 {
            return;
        }
        let values = &mut rotations[..count];
        values.sort_by(f64::total_cmp);
        let median = values[count / 2];
        let tolerance = f64::from(config.turn_consistency_degrees).to_radians();
        let consistent = values
            .iter()
            .filter(|v| (**v - median).abs() <= tolerance)
            .count();
        if consistent < 3 || 4 * consistent < 3 * count {
            return;
        }
        let degrees = median.to_degrees();
        match self.turn_degrees {
            Some(latched) if degrees.abs() > latched.abs() => self.turn_degrees = Some(degrees),
            None if median.abs() >= f64::from(config.turn_threshold_degrees).to_radians() => {
                self.turn_degrees = Some(degrees);
            }
            _ => {}
        }
    }

    fn observe_headset(
        &mut self,
        config: &MotionCalibrationConfig,
        at: f64,
        position: Vec3,
        orientation: Quat,
        head_to_neck_m: f32,
        eye_forward_m: f32,
    ) -> SampleOutcome {
        if self.headset.is_none() {
            return rejected(RejectReason::NoNeckReference);
        }
        if let Err(reason) = self.check_time(config, at) {
            return rejected(reason);
        }
        if !position.is_finite() || !orientation.is_finite() {
            return rejected(RejectReason::NonFinite);
        }
        let Some(neck) =
            MotionCalibrator::neck_pivot(position, orientation, head_to_neck_m, eye_forward_m)
        else {
            return rejected(RejectReason::InvalidDimensions);
        };
        let chest_tilt = self.segments[CHEST].latest.and_then(|(t, delta)| {
            ((at - t).abs() <= config.headset_chest_max_skew_secs)
                .then(|| f64::from((delta * Vec3::Y).y).clamp(-1.0, 1.0).acos())
        });
        let Some(headset) = self.headset.as_mut() else {
            return rejected(RejectReason::NoNeckReference);
        };
        if headset.last_at.is_some_and(|last| at <= last) {
            return rejected(RejectReason::OutOfOrder);
        }
        let previous = headset.last_at.replace(at);
        self.newest_at = Some(self.newest_at.map_or(at, |newest| newest.max(at)));
        let displacement = neck - headset.neutral_neck;
        let (dx, dz) = (f64::from(displacement.x), f64::from(displacement.z));
        let horizontal = dx.hypot(dz);
        if horizontal.is_finite() {
            headset.max_displacement = headset.max_displacement.max(horizontal);
        }
        let Some(last) = previous else {
            return SampleOutcome::Accepted { evidence: false };
        };
        if at - last > config.max_gap_secs {
            return SampleOutcome::ResumedAfterGap;
        }
        let w = (at - last).min(config.max_sample_weight_secs);
        let evidence = w > 0.0
            && horizontal.is_finite()
            && horizontal >= f64::from(config.headset_min_displacement_m)
            && chest_tilt.is_some_and(|tilt| {
                tilt >= f64::from(config.headset_min_chest_tilt_degrees).to_radians()
            });
        if evidence {
            headset
                .accumulator
                .forget_and_add(dx, dz, w, config.evidence_memory_secs);
            headset.evidence_secs += w;
        }
        SampleOutcome::Accepted { evidence }
    }
}

// ---------------------------------------------------------------------------------------------
// Evaluation.

struct SegmentEval {
    fit: Option<LineFit>,
    excitation_fraction: f64,
    excitation_ok: bool,
    planarity_ok: bool,
    one_sided_sign: Option<f64>,
    repeatability: Option<f64>,
    repeatability_ok: bool,
    residual: Option<f64>,
    residual_ok: bool,
    stability: Option<f64>,
    stability_ok: bool,
    silence: Option<f64>,
}

struct KneeMetrics {
    frames: usize,
    hyperextension_p95: f64,
    hyperextension_rms: f64,
    flexion_p95: f64,
    axis: Option<f64>,
}

struct Evaluation {
    status: MotionCalibrationStatus,
    sensor_headings: [Option<f64>; SEGMENTS],
    reference_heading: Option<f64>,
    reference_used: MotionReference,
}

fn inactive_status(reference: MotionReference) -> MotionCalibrationStatus {
    MotionCalibrationStatus {
        active: false,
        ready: false,
        segments: Role::CORE
            .iter()
            .map(|role| MotionSegmentStatus {
                role: *role,
                progress: 0.0,
                ready: false,
                blockers: vec![MotionBlocker {
                    gate: MotionGate::NotStarted,
                    message: format!("{role:?}: capture neutral first"),
                }],
                max_tilt_degrees: 0.0,
                tilted_secs: 0.0,
                evidence_secs: 0.0,
                planarity: None,
                one_sidedness: None,
                sign: SignEvidence::Unknown,
                repeatability_degrees: None,
                residual_degrees: None,
                stability_degrees: None,
                twist_rms_degrees: None,
                sensor_forward_heading_radians: None,
                heading_alignment_radians: None,
                accepted_samples: 0,
                rejected_samples: 0,
                gaps: 0,
            })
            .collect(),
        blockers: vec![MotionBlocker {
            gate: MotionGate::NotStarted,
            message: "Capture a neutral pose, then begin motion calibration".into(),
        }],
        warnings: Vec::new(),
        knees: Vec::new(),
        yaw_reference_heading_radians: None,
        selected_reference: reference,
        selected_reference_heading_radians: None,
        headset: None,
        body_turn_degrees: None,
        retained_snapshots: 0,
    }
}

fn evaluate_segment(
    config: &MotionCalibrationConfig,
    session: &Session,
    segment: &SegmentState,
) -> SegmentEval {
    let fit = segment.main.fit();
    let peak = f64::from(config.min_peak_tilt_degrees).to_radians();
    let excitation_fraction = 0.5
        * ((segment.max_tilt / peak).min(1.0)
            + (segment.tilted_secs / config.min_tilted_secs).min(1.0));
    let excitation_ok = segment.max_tilt >= peak && segment.tilted_secs >= config.min_tilted_secs;
    let planarity_ok = fit.is_some_and(|f| f.planarity() <= f64::from(config.max_planarity));
    let one_sided_sign = fit
        .filter(|f| f.one_sidedness >= f64::from(config.min_one_sidedness))
        .map(|f| f.signed.signum());
    let block_fits = segment.blocks.map(|block| {
        (block.weight >= config.min_block_evidence_secs)
            .then(|| block.fit())
            .flatten()
    });
    let repeatability = match block_fits {
        [Some(a), Some(b)] => Some(line_difference(a.angle, b.angle).abs()),
        _ => None,
    };
    let repeatability_ok = repeatability
        .is_some_and(|r| r <= f64::from(config.max_repeatability_degrees).to_radians());
    let residual = fit.map(|f| {
        (f.lambda_min / f.weight.max(1.0e-12))
            .clamp(0.0, 1.0)
            .sqrt()
            .asin()
    });
    let residual_ok =
        residual.is_some_and(|r| r <= f64::from(config.max_residual_degrees).to_radians());
    let stability = fit.and_then(|f| {
        let newest = segment.history.back()?.0;
        let start = newest - config.stability_window_secs;
        let mut window = segment
            .history
            .iter()
            .filter(|(t, _)| *t >= start)
            .peekable();
        let oldest = window.peek()?.0;
        if newest - oldest < 0.9 * config.stability_window_secs {
            return None;
        }
        Some(
            window
                .map(|(_, angle)| line_difference(f.angle, *angle).abs())
                .fold(0.0, f64::max),
        )
    });
    let stability_ok =
        stability.is_some_and(|s| s <= f64::from(config.max_stability_degrees).to_radians());
    let silence = match (session.newest_at, segment.last_at) {
        (Some(newest), Some(last)) => Some((newest - last).max(0.0)),
        _ => None,
    };
    SegmentEval {
        fit,
        excitation_fraction,
        excitation_ok,
        planarity_ok,
        one_sided_sign,
        repeatability,
        repeatability_ok,
        residual,
        residual_ok,
        stability,
        stability_ok,
        silence,
    }
}

/// Knee metrics for candidate body-forward headings of the thigh and calf in
/// their own sensor worlds. With `H = heading(f)` the calf tip in the
/// calibrated thigh frame is `k = Ry(−H_t)·D_t⁻¹·Ry(H_t − H_c)·D_c·Y`,
/// independent of the reference heading. Flexion `atan2(−k.z, k.y)` is
/// positive when the calf tips forward relative to the thigh.
fn knee_metrics(
    config: &MotionCalibrationConfig,
    snapshots: &VecDeque<Snapshot>,
    leg: usize,
    thigh_heading: f64,
    calf_heading: f64,
) -> Option<KneeMetrics> {
    let (ti, ci) = (THIGH[leg], CALF[leg]);
    let mask = (1_u8 << ti) | (1_u8 << ci);
    let back = Quat::from_rotation_y(-thigh_heading as f32);
    let relative = Quat::from_rotation_y((thigh_heading - calf_heading) as f32);
    let min_tilt = f64::from(config.min_evidence_tilt_degrees).to_radians();
    let min_bend = KNEE_AXIS_MIN_BEND_DEGREES.to_radians();
    let mut hyperextension = Vec::with_capacity(snapshots.len());
    let mut flexion = Vec::with_capacity(snapshots.len());
    let (mut cos2, mut sin2, mut axis_frames) = (0.0_f64, 0.0_f64, 0_usize);
    let mut squared = 0.0_f64;
    for snapshot in snapshots {
        if snapshot.valid & mask != mask {
            continue;
        }
        let (thigh, calf) = (snapshot.deltas[ti], snapshot.deltas[ci]);
        let calf_tip = calf * Vec3::Y;
        let tilt_thigh = f64::from((thigh * Vec3::Y).y).clamp(-1.0, 1.0).acos();
        let tilt_calf = f64::from(calf_tip.y).clamp(-1.0, 1.0).acos();
        if tilt_thigh < min_tilt && tilt_calf < min_tilt {
            continue;
        }
        let k = back * (thigh.conjugate() * (relative * calf_tip));
        if !k.is_finite() {
            continue;
        }
        let (kx, ky, kz) = (f64::from(k.x), f64::from(k.y), f64::from(k.z));
        let flex = (-kz).atan2(ky);
        let hyper = (-flex).max(0.0);
        squared += hyper * hyper;
        hyperextension.push(hyper);
        flexion.push(flex);
        let bend = ky.clamp(-1.0, 1.0).acos();
        if bend > min_bend {
            let direction = kx.atan2(-kz);
            let weight = bend.sin();
            cos2 += weight * (2.0 * direction).cos();
            sin2 += weight * (2.0 * direction).sin();
            axis_frames += 1;
        }
    }
    let frames = hyperextension.len();
    if frames < MIN_KNEE_FRAMES {
        return None;
    }
    let axis = (axis_frames >= MIN_KNEE_AXIS_FRAMES && cos2.hypot(sin2) > 1.0e-9)
        .then(|| (0.5 * sin2.atan2(cos2)).abs());
    Some(KneeMetrics {
        frames,
        hyperextension_p95: percentile(&mut hyperextension, 0.95),
        hyperextension_rms: (squared / frames as f64).sqrt(),
        flexion_p95: percentile(&mut flexion, 0.95),
        axis,
    })
}

fn percentile(values: &mut [f64], fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    let index = ((values.len() - 1) as f64 * fraction).round() as usize;
    values[index.min(values.len() - 1)]
}

fn pearson(a: &[f64], b: &[f64]) -> Option<f64> {
    let n = a.len().min(b.len());
    if n < 2 {
        return None;
    }
    let mean_a = a[..n].iter().sum::<f64>() / n as f64;
    let mean_b = b[..n].iter().sum::<f64>() / n as f64;
    let (mut ab, mut aa, mut bb) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let (x, y) = (a[i] - mean_a, b[i] - mean_b);
        ab += x * y;
        aa += x * x;
        bb += y * y;
    }
    let denominator = (aa * bb).sqrt();
    (denominator > 1.0e-12)
        .then(|| ab / denominator)
        .filter(|r| r.is_finite())
}

/// Left/right swap heuristic: during asymmetric movement (marching), each
/// thigh's tilt should move with its own calf, not with the opposite calf.
fn left_right_swap_suspected(snapshots: &VecDeque<Snapshot>) -> bool {
    let legs = [THIGH[0], THIGH[1], CALF[0], CALF[1]];
    let mask = legs.iter().fold(0_u8, |m, i| m | (1 << i));
    let mut series: [Vec<f64>; 4] = Default::default();
    for snapshot in snapshots.iter().filter(|s| s.valid & mask == mask) {
        for (k, index) in legs.iter().enumerate() {
            series[k].push(
                f64::from((snapshot.deltas[*index] * Vec3::Y).y)
                    .clamp(-1.0, 1.0)
                    .acos(),
            );
        }
    }
    if series[0].len() < MIN_SWAP_FRAMES {
        return false;
    }
    let r = |a: usize, b: usize| pearson(&series[a], &series[b]);
    let (Some(lt_lc), Some(rt_rc), Some(lt_rc), Some(rt_lc), Some(lt_rt)) =
        (r(0, 2), r(1, 3), r(0, 3), r(1, 2), r(0, 1))
    else {
        return false;
    };
    let same = lt_lc + rt_rc;
    let cross = lt_rc + rt_lc;
    lt_rt < 0.3 && cross > 0.8 && cross > same + 0.6
}

fn hint(role: Role) -> &'static str {
    match role {
        Role::Chest => "bow forward from the hips",
        Role::Hip => "tip the pelvis forward: push the hips back as in a squat or bow",
        Role::LeftThigh | Role::RightThigh => "lift each knee forward or squat deeper",
        _ => "bend the knees forward over the toes (squat or skiing pose)",
    }
}

fn evaluate(
    config: &MotionCalibrationConfig,
    session: &Session,
    choice: MotionReference,
) -> Evaluation {
    let evals: Vec<SegmentEval> = session
        .segments
        .iter()
        .map(|segment| evaluate_segment(config, session, segment))
        .collect();
    let mut warnings = Vec::new();
    let mut extra_blockers: [Vec<MotionBlocker>; SEGMENTS] = Default::default();

    // Body-forward sign per segment, then the knee rule for two-sided legs.
    let mut signs: [Option<f64>; SEGMENTS] = [None; SEGMENTS];
    let mut sign_evidence = [SignEvidence::Unknown; SEGMENTS];
    for i in 0..SEGMENTS {
        let role_sign = f64::from(tip_sign(Role::CORE[i]).unwrap_or(1.0));
        if evals[i].fit.is_some() {
            match evals[i].one_sided_sign {
                Some(direction) => {
                    let sign = role_sign * direction;
                    signs[i] = Some(sign);
                    sign_evidence[i] = SignEvidence::OneSided { sign: sign as f32 };
                }
                None => sign_evidence[i] = SignEvidence::Undetermined,
            }
        }
    }
    let mut knees = Vec::with_capacity(2);
    for leg in 0..2 {
        let (ti, ci) = (THIGH[leg], CALF[leg]);
        let (thigh_role, calf_role) = (Role::CORE[ti], Role::CORE[ci]);
        let mut knee = KneeStatus {
            thigh: thigh_role,
            calf: calf_role,
            frames: 0,
            hyperextension_p95_degrees: None,
            flexion_p95_degrees: None,
            axis_disagreement_degrees: None,
            sign_resolved_by_knee: false,
        };
        let usable = |i: usize| evals[i].excitation_ok && evals[i].planarity_ok;
        let (Some(thigh_fit), Some(calf_fit)) = (evals[ti].fit, evals[ci].fit) else {
            knees.push(knee);
            continue;
        };
        if !(usable(ti) && usable(ci)) {
            knees.push(knee);
            continue;
        }
        let candidates = |i: usize| -> Vec<f64> {
            match signs[i] {
                Some(sign) => vec![sign],
                None if config.knee_sign_rule => vec![1.0, -1.0],
                None => Vec::new(),
            }
        };
        let mut scored = Vec::new();
        for thigh_sign in candidates(ti) {
            for calf_sign in candidates(ci) {
                let metrics = knee_metrics(
                    config,
                    &session.snapshots,
                    leg,
                    forward_heading(thigh_fit.angle, thigh_sign),
                    forward_heading(calf_fit.angle, calf_sign),
                );
                if let Some(metrics) = metrics {
                    scored.push((thigh_sign, calf_sign, metrics));
                }
            }
        }
        let chosen = if scored.len() == 1 && signs[ti].is_some() && signs[ci].is_some() {
            scored.pop()
        } else if scored.len() > 1 {
            scored.sort_by(|a, b| a.2.hyperextension_rms.total_cmp(&b.2.hyperextension_rms));
            let best = &scored[0].2;
            let second_rms = scored[1].2.hyperextension_rms;
            let resolved = best.hyperextension_rms
                <= f64::from(config.knee_rule_max_hyperextension_rms_degrees).to_radians()
                && second_rms >= f64::from(config.knee_rule_min_rejected_rms_degrees).to_radians()
                && best.flexion_p95 >= f64::from(config.knee_min_flexion_degrees).to_radians();
            if resolved {
                let (thigh_sign, calf_sign, metrics) = scored.swap_remove(0);
                for (i, sign) in [(ti, thigh_sign), (ci, calf_sign)] {
                    if signs[i].is_none() {
                        signs[i] = Some(sign);
                        sign_evidence[i] = SignEvidence::KneeRule { sign: sign as f32 };
                    }
                }
                knee.sign_resolved_by_knee = true;
                warnings.push(MotionWarning {
                    kind: MotionWarningKind::SignFromKneeRule,
                    roles: vec![thigh_role, calf_role],
                    message: format!(
                        "{thigh_role:?}/{calf_role:?}: the movement went both forward and backward; \
                         the forward sign was chosen from the knee's one-way bend \
                         (best hyperextension {:.1}° RMS versus {:.1}° for the next option)",
                        metrics.hyperextension_rms.to_degrees(),
                        second_rms.to_degrees()
                    ),
                });
                Some((thigh_sign, calf_sign, metrics))
            } else {
                None
            }
        } else {
            None
        };
        let Some((_, _, metrics)) = chosen else {
            if signs[ti].is_some() && signs[ci].is_some() {
                // Both signs known but too few paired frames to check the knee.
                for i in [ti, ci] {
                    extra_blockers[i].push(MotionBlocker {
                        gate: MotionGate::KneeUnchecked,
                        message: format!(
                            "{:?}: knee consistency not yet checked (needs time-aligned thigh and \
                             calf samples while the leg moves)",
                            Role::CORE[i]
                        ),
                    });
                }
            }
            knees.push(knee);
            continue;
        };
        knee.frames = metrics.frames;
        knee.hyperextension_p95_degrees = degrees32(metrics.hyperextension_p95);
        knee.flexion_p95_degrees = degrees32(metrics.flexion_p95);
        knee.axis_disagreement_degrees = metrics.axis.and_then(degrees32);
        let leg_name = if leg == 0 { "Left" } else { "Right" };
        if metrics.hyperextension_p95
            > f64::from(config.knee_max_hyperextension_degrees).to_radians()
        {
            let message = format!(
                "{leg_name} knee would bend backwards by {:.0}° with these headings. The movement or \
                 the sensor assignment does not match the instruction (a thigh or calf moved \
                 opposite to the instructed direction, or sensors are on the wrong segment); check \
                 assignments and repeat squats or knee lifts",
                metrics.hyperextension_p95.to_degrees()
            );
            for i in [ti, ci] {
                extra_blockers[i].push(MotionBlocker {
                    gate: MotionGate::KneeHyperextension,
                    message: message.clone(),
                });
            }
            warnings.push(MotionWarning {
                kind: MotionWarningKind::KneeHyperextension,
                roles: vec![thigh_role, calf_role],
                message,
            });
        }
        if let Some(axis) = metrics.axis
            && axis > f64::from(config.knee_max_axis_degrees).to_radians()
        {
            let message = format!(
                "{leg_name} thigh and calf bend in directions {:.0}° apart (max {:.0}°): the \
                 movement direction changed during the procedure (turning) or one of them moved \
                 sideways part of the time; keep facing the same direction and repeat",
                axis.to_degrees(),
                config.knee_max_axis_degrees
            );
            for i in [ti, ci] {
                extra_blockers[i].push(MotionBlocker {
                    gate: MotionGate::KneeAxis,
                    message: message.clone(),
                });
            }
            warnings.push(MotionWarning {
                kind: MotionWarningKind::KneeAxisMismatch,
                roles: vec![thigh_role, calf_role],
                message,
            });
        }
        knees.push(knee);
    }

    // Swap heuristics (warnings only).
    for leg in 0..2 {
        let (ti, ci) = (THIGH[leg], CALF[leg]);
        let one_sided = evals[ti].one_sided_sign.is_some() && evals[ci].one_sided_sign.is_some();
        let excited = evals[ti].excitation_ok && evals[ci].excitation_ok;
        let thigh_tilt = session.segments[ti].max_tilt;
        let calf_tilt = session.segments[ci].max_tilt;
        if one_sided
            && excited
            && calf_tilt > thigh_tilt + f64::from(config.swap_tilt_margin_degrees).to_radians()
        {
            warnings.push(MotionWarning {
                kind: MotionWarningKind::SuspectedThighCalfSwap,
                roles: vec![Role::CORE[ti], Role::CORE[ci]],
                message: format!(
                    "{:?} tilted {:.0}° but {:?} only {:.0}°: in squats, knee lifts and the skiing \
                     pose the thigh normally tilts more. The thigh and calf sensors may be swapped \
                     (a swap is otherwise indistinguishable from a 180° heading of both)",
                    Role::CORE[ci],
                    calf_tilt.to_degrees(),
                    Role::CORE[ti],
                    thigh_tilt.to_degrees()
                ),
            });
        }
    }
    if left_right_swap_suspected(&session.snapshots) {
        warnings.push(MotionWarning {
            kind: MotionWarningKind::SuspectedLeftRightSwap,
            roles: vec![
                Role::LeftThigh,
                Role::RightThigh,
                Role::LeftCalf,
                Role::RightCalf,
            ],
            message: "Each thigh moves together with the opposite calf: thigh or calf sensors may \
                      be swapped between the left and right legs"
                .into(),
        });
    }

    // Global blockers.
    let mut blockers = Vec::new();
    if let Some(turn) = session.turn_degrees {
        let message = format!(
            "You turned about {:.0}° during the movement; keep facing the same direction. Return \
             to neutral and start again",
            turn.abs()
        );
        blockers.push(MotionBlocker {
            gate: MotionGate::BodyTurn,
            message: message.clone(),
        });
        warnings.push(MotionWarning {
            kind: MotionWarningKind::BodyTurn,
            roles: Role::CORE.to_vec(),
            message,
        });
    }

    // Headset-assisted forward and reference selection.
    let headset = session
        .headset
        .as_ref()
        .map(|headset| evaluate_headset(config, headset, session.yaw_reference));
    if let Some(status) = &headset
        && status.disagreement_flagged
    {
        warnings.push(MotionWarning {
            kind: MotionWarningKind::HeadsetDisagreement,
            roles: vec![Role::Chest],
            message: format!(
                "Neck displacement points {:.0}° away from the headset yaw at neutral (limit \
                 {:.0}°): the head and torso may not have faced the same way at neutral, or the \
                 torso moved sideways. Fusion falls back to the headset yaw; recapture neutral \
                 with head and torso facing straight ahead",
                status
                    .difference_from_yaw_radians
                    .map_or(0.0, |d| d.to_degrees().abs()),
                config.headset_max_disagreement_degrees
            ),
        });
    }
    let (reference_used, reference_heading) = match choice {
        MotionReference::HeadsetYaw => (MotionReference::HeadsetYaw, Some(session.yaw_reference)),
        MotionReference::Fused => match headset.as_ref().and_then(|h| h.fused_heading_radians) {
            Some(fused) => (MotionReference::Fused, Some(f64::from(fused))),
            None => (MotionReference::HeadsetYaw, Some(session.yaw_reference)),
        },
        MotionReference::NeckDisplacement => {
            match headset
                .as_ref()
                .and_then(|h| h.displacement_heading_radians.filter(|_| h.available))
            {
                Some(heading) => (MotionReference::NeckDisplacement, Some(f64::from(heading))),
                None => {
                    blockers.push(MotionBlocker {
                        gate: MotionGate::Reference,
                        message: headset
                            .as_ref()
                            .and_then(|h| h.reason.clone())
                            .unwrap_or_else(|| "No headset neck reference was supplied".into()),
                    });
                    (MotionReference::NeckDisplacement, None)
                }
            }
        }
    };

    // Per-segment status.
    let mut segments = Vec::with_capacity(SEGMENTS);
    let mut sensor_headings = [None; SEGMENTS];
    for i in 0..SEGMENTS {
        let role = Role::CORE[i];
        let segment = &session.segments[i];
        let eval = &evals[i];
        let mut list = Vec::new();
        let fresh = eval
            .silence
            .is_some_and(|silence| silence <= config.ready_max_silence_secs);
        if segment.last_at.is_none() {
            list.push(MotionBlocker {
                gate: MotionGate::NoData,
                message: format!("{role:?}: no sensor samples since neutral"),
            });
        } else if !fresh {
            list.push(MotionBlocker {
                gate: MotionGate::Stale,
                message: format!(
                    "{role:?}: no fresh sensor data for {:.1} s",
                    eval.silence.unwrap_or(0.0)
                ),
            });
        }
        if !eval.excitation_ok {
            list.push(MotionBlocker {
                gate: MotionGate::Excitation,
                message: format!(
                    "{role:?}: tilted {:.0}° (needs {:.0}°) and {:.1} s beyond {:.0}° (needs {:.1} s); {}",
                    segment.max_tilt.to_degrees(),
                    config.min_peak_tilt_degrees,
                    segment.tilted_secs,
                    config.tilted_threshold_degrees,
                    config.min_tilted_secs,
                    hint(role)
                ),
            });
        }
        let planarity = eval.fit.map(|f| f.planarity());
        let mut passed = 0;
        if eval.planarity_ok {
            passed += 1;
        } else {
            list.push(MotionBlocker {
                gate: MotionGate::Planarity,
                message: match planarity {
                    Some(p) => format!(
                        "{role:?}: not a clean forward/backward bend (planarity {p:.2}, max {:.2}); \
                         avoid sideways or circular movement",
                        config.max_planarity
                    ),
                    None => format!("{role:?}: no tilt evidence yet; {}", hint(role)),
                },
            });
        }
        if signs[i].is_some() {
            passed += 1;
        } else if eval.fit.is_some() {
            list.push(MotionBlocker {
                gate: MotionGate::OneSidedness,
                message: format!(
                    "{role:?}: forward/backward sign undetermined (one-sidedness {:.2} < {:.2}): \
                     the movement went both ways; use squats, a bow or knee lifts instead of \
                     walking or swinging",
                    eval.fit.map_or(0.0, |f| f.one_sidedness),
                    config.min_one_sidedness
                ),
            });
        }
        if eval.repeatability_ok {
            passed += 1;
        } else if eval.fit.is_some() {
            list.push(MotionBlocker {
                gate: MotionGate::Repeatability,
                message: match eval.repeatability {
                    Some(r) => format!(
                        "{role:?}: repetitions disagree by {:.1}° (max {:.0}°); repeat the same \
                         movement steadily",
                        r.to_degrees(),
                        config.max_repeatability_degrees
                    ),
                    None => format!("{role:?}: needs more repetitions to check consistency"),
                },
            });
        }
        if eval.residual_ok {
            passed += 1;
        } else if let Some(residual) = eval.residual {
            list.push(MotionBlocker {
                gate: MotionGate::ModelResidual,
                message: format!(
                    "{role:?}: {:.1}° RMS sideways tilt (max {:.0}°); keep the movement \
                     forward/backward",
                    residual.to_degrees(),
                    config.max_residual_degrees
                ),
            });
        }
        if eval.stability_ok {
            passed += 1;
        } else if eval.fit.is_some() {
            list.push(MotionBlocker {
                gate: MotionGate::Stability,
                message: match eval.stability {
                    Some(s) => format!(
                        "{role:?}: estimate still changing ({:.1}° in the last second, max {:.0}°); \
                         keep going",
                        s.to_degrees(),
                        config.max_stability_degrees
                    ),
                    None => format!("{role:?}: estimate needs another second of movement"),
                },
            });
        }
        list.append(&mut extra_blockers[i]);
        let sensor_heading = match (eval.fit, signs[i]) {
            (Some(fit), Some(sign)) => {
                Some(forward_heading(fit.angle, sign)).filter(|h| h.is_finite())
            }
            _ => None,
        };
        sensor_headings[i] = sensor_heading;
        let ready = list.is_empty() && sensor_heading.is_some();
        let progress = if ready {
            1.0
        } else if !eval.excitation_ok {
            0.5 * eval.excitation_fraction
        } else {
            (0.5 + 0.1 * f64::from(passed)).min(0.95)
        };
        let moment = segment.moment_weight.max(1.0e-12);
        segments.push(MotionSegmentStatus {
            role,
            progress: finite32(progress).unwrap_or(0.0),
            ready,
            blockers: list,
            max_tilt_degrees: degrees32(segment.max_tilt).unwrap_or(0.0),
            tilted_secs: finite32(segment.tilted_secs).unwrap_or(0.0),
            evidence_secs: finite32(segment.main.weight).unwrap_or(0.0),
            planarity: planarity.and_then(finite32),
            one_sidedness: eval.fit.and_then(|f| finite32(f.one_sidedness)),
            sign: sign_evidence[i],
            repeatability_degrees: eval.repeatability.and_then(degrees32),
            residual_degrees: eval.residual.and_then(degrees32),
            stability_degrees: eval.stability.and_then(degrees32),
            twist_rms_degrees: (segment.moment_weight > 0.0)
                .then(|| degrees32((segment.twist_squared / moment).sqrt()))
                .flatten(),
            sensor_forward_heading_radians: sensor_heading.and_then(finite32),
            heading_alignment_radians: match (reference_heading, sensor_heading) {
                (Some(h), Some(s)) => finite32(wrap(h - s)),
                _ => None,
            },
            accepted_samples: segment.accepted,
            rejected_samples: segment.rejected,
            gaps: segment.gaps,
        });
    }
    let ready = blockers.is_empty() && segments.iter().all(|s| s.ready);
    Evaluation {
        status: MotionCalibrationStatus {
            active: true,
            ready,
            segments,
            blockers,
            warnings,
            knees,
            yaw_reference_heading_radians: finite32(session.yaw_reference),
            selected_reference: reference_used,
            selected_reference_heading_radians: reference_heading.and_then(finite32),
            headset,
            body_turn_degrees: session.turn_degrees.and_then(finite32),
            retained_snapshots: session.snapshots.len(),
        },
        sensor_headings,
        reference_heading,
        reference_used,
    }
}

fn evaluate_headset(
    config: &MotionCalibrationConfig,
    headset: &HeadsetState,
    yaw_reference: f64,
) -> HeadsetForwardStatus {
    let fit = headset.accumulator.fit();
    let mut status = HeadsetForwardStatus {
        available: false,
        reason: None,
        evidence_secs: finite32(headset.evidence_secs).unwrap_or(0.0),
        max_displacement_m: finite32(headset.max_displacement).unwrap_or(0.0),
        planarity: fit.and_then(|f| finite32(f.planarity())),
        one_sidedness: fit.and_then(|f| finite32(f.one_sidedness)),
        displacement_heading_radians: None,
        displacement_sigma_radians: None,
        difference_from_yaw_radians: None,
        fused_heading_radians: None,
        fused_sigma_radians: None,
        disagreement_flagged: false,
        accepted_samples: headset.accepted,
        rejected_samples: headset.rejected,
    };
    let reason = match fit {
        _ if headset.evidence_secs < config.headset_min_evidence_secs => Some(format!(
            "Headset forward needs {:.1} s of neck movement while the chest is tilted beyond \
             {:.0}° (have {:.1} s)",
            config.headset_min_evidence_secs,
            config.headset_min_chest_tilt_degrees,
            headset.evidence_secs
        )),
        None => Some("No neck displacement evidence".into()),
        Some(f) if f.planarity() > f64::from(config.max_planarity) => Some(format!(
            "Neck displacement is not along one line (planarity {:.2})",
            f.planarity()
        )),
        Some(f) if f.one_sidedness < f64::from(config.min_one_sidedness) => Some(format!(
            "Neck moved both forward and backward (one-sidedness {:.2})",
            f.one_sidedness
        )),
        Some(_) => None,
    };
    if let Some(reason) = reason {
        status.reason = Some(reason);
        return status;
    }
    let Some(fit) = fit else {
        return status;
    };
    // The neck pivot moves FORWARD while the chest is tilted forward.
    let heading = forward_heading(fit.angle, fit.signed.signum());
    let weight = fit.weight.max(1.0e-12);
    let along = (fit.lambda_max / weight).sqrt();
    let lateral_squared = fit.lambda_min / weight;
    let noise = f64::from(config.headset_noise_m);
    let sigma = (noise * noise + lateral_squared).sqrt().atan2(along);
    let difference = wrap(heading - yaw_reference);
    if !(heading.is_finite() && sigma.is_finite() && sigma > 0.0 && difference.is_finite()) {
        status.reason = Some("Neck displacement estimate is numerically unstable".into());
        return status;
    }
    status.available = true;
    status.displacement_heading_radians = finite32(heading);
    status.displacement_sigma_radians = finite32(sigma);
    status.difference_from_yaw_radians = finite32(difference);
    if difference.abs() > f64::from(config.headset_max_disagreement_degrees).to_radians() {
        status.disagreement_flagged = true;
        return status;
    }
    let yaw_sigma = f64::from(config.head_torso_sigma_degrees).to_radians();
    let (w_yaw, w_displacement) = (1.0 / (yaw_sigma * yaw_sigma), 1.0 / (sigma * sigma));
    let x = w_yaw * yaw_reference.cos() + w_displacement * heading.cos();
    let y = w_yaw * yaw_reference.sin() + w_displacement * heading.sin();
    status.fused_heading_radians = finite32(y.atan2(x));
    status.fused_sigma_radians = finite32((w_yaw + w_displacement).sqrt().recip());
    status
}

fn blocker_summary(status: &MotionCalibrationStatus) -> String {
    let mut parts: Vec<String> = status.blockers.iter().map(|b| b.message.clone()).collect();
    for segment in &status.segments {
        if let Some(first) = segment.blockers.first() {
            parts.push(first.message.clone());
        } else if !segment.ready {
            parts.push(format!("{:?}: not ready", segment.role));
        }
    }
    if parts.is_empty() {
        "unknown".into()
    } else {
        parts.join("; ")
    }
}
