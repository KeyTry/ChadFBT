//! Sliding-window relative-heading experiment. This module cannot mutate tracking
//! state. A well-conditioned hinge fit is conditional on the hinge/mounting model;
//! it cannot distinguish a consistent real twist from orientation drift.
//!
//! Inputs are calibrated segment orientations in +Y-up standing space. Receipt
//! timestamps only gate pairing and freshness; they do not establish sensor-clock
//! synchronization. Independent witnesses have a stronger, explicit contract.

use crate::{CoreError, Role, SensorProvenance};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct HeadingShadowConfig {
    pub window_secs: f64,
    pub max_samples: usize,
    pub min_samples: usize,
    pub min_span_secs: f64,
    pub max_gap_secs: f64,
    pub max_pair_skew_secs: f64,
    pub stale_after_secs: f64,
    pub min_input_confidence: f64,
    pub parent_hinge_axis: [f64; 3],
    pub child_hinge_axis: [f64; 3],
    pub parent_long_axis: [f64; 3],
    pub child_long_axis: [f64; 3],
    /// Huber transition expressed as the angle between unit axis vectors.
    pub huber_angle_radians: f64,
    pub max_median_residual_radians: f64,
    pub max_downweighted_fraction: f64,
    /// Threshold on normalized frozen-weight cost curvature, not probability.
    pub min_relative_curvature: f64,
    /// A separate motion heuristic. It is NOT an observability theorem.
    pub min_flex_excursion_radians: f64,
    pub max_candidate_radians: f64,
    pub min_witness_samples: usize,
    pub min_witness_span_secs: f64,
    pub max_witness_error_bound_radians: f64,
}

impl Default for HeadingShadowConfig {
    fn default() -> Self {
        Self {
            window_secs: 2.0,
            max_samples: 256,
            min_samples: 30,
            min_span_secs: 0.5,
            max_gap_secs: 0.15,
            max_pair_skew_secs: 0.03,
            stale_after_secs: 0.25,
            min_input_confidence: 0.6,
            parent_hinge_axis: [1.0, 0.0, 0.0],
            child_hinge_axis: [1.0, 0.0, 0.0],
            parent_long_axis: [0.0, -1.0, 0.0],
            child_long_axis: [0.0, -1.0, 0.0],
            huber_angle_radians: 4.0_f64.to_radians(),
            max_median_residual_radians: 3.0_f64.to_radians(),
            max_downweighted_fraction: 0.30,
            min_relative_curvature: 0.10,
            min_flex_excursion_radians: 8.0_f64.to_radians(),
            max_candidate_radians: 30.0_f64.to_radians(),
            min_witness_samples: 10,
            min_witness_span_secs: 0.25,
            max_witness_error_bound_radians: 5.0_f64.to_radians(),
        }
    }
}

impl HeadingShadowConfig {
    fn validate(&self) -> Result<(), CoreError> {
        let positive = [
            self.window_secs,
            self.min_span_secs,
            self.max_gap_secs,
            self.max_pair_skew_secs,
            self.stale_after_secs,
            self.huber_angle_radians,
            self.max_median_residual_radians,
            self.min_relative_curvature,
            self.max_candidate_radians,
            self.min_witness_span_secs,
            self.max_witness_error_bound_radians,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || self.window_secs > 10.0
            || self.min_span_secs > self.window_secs
            || self.stale_after_secs > self.window_secs
            || self.min_witness_span_secs > self.window_secs
            || !(3..=2048).contains(&self.max_samples)
            || !(3..=self.max_samples).contains(&self.min_samples)
            || !(2..=self.max_samples).contains(&self.min_witness_samples)
            || !self.min_input_confidence.is_finite()
            || !(0.0..=1.0).contains(&self.min_input_confidence)
            || self.min_input_confidence == 0.0
            || !self.max_downweighted_fraction.is_finite()
            || !(0.0..0.5).contains(&self.max_downweighted_fraction)
            || !self.min_flex_excursion_radians.is_finite()
            || !(0.0..PI).contains(&self.min_flex_excursion_radians)
            || self.huber_angle_radians >= PI / 2.0
            || self.max_candidate_radians >= PI / 2.0
            || self.max_witness_error_bound_radians >= PI / 2.0
            || self.min_relative_curvature > 1.0
        {
            return Err(CoreError("invalid shadow-heading configuration".into()));
        }
        for axis in [
            self.parent_hinge_axis,
            self.child_hinge_axis,
            self.parent_long_axis,
            self.child_long_axis,
        ] {
            unit_vector(axis)?;
        }
        for (hinge, long) in [
            (self.parent_hinge_axis, self.parent_long_axis),
            (self.child_hinge_axis, self.child_long_axis),
        ] {
            if unit_vector(hinge)?.dot(unit_vector(long)?).abs() > 0.95 {
                return Err(CoreError(
                    "hinge and longitudinal axes are too nearly parallel".into(),
                ));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct HeadingSegmentSample {
    pub role: Role,
    /// Sensor-to-bone mounting has already been applied; XYZW, not raw sensor axes.
    pub orientation_xyzw: [f32; 4],
    pub measured_at_secs: f64,
    /// Engineering input quality, not a probability or inverse variance.
    pub confidence: f64,
    pub valid: bool,
    pub provenance: SensorProvenance,
}

#[derive(Clone, Debug)]
pub struct HeadingPairSample {
    /// Change when mounting, assignments or heading-reference calibration changes.
    pub calibration_epoch: u64,
    /// Change on transport discontinuity, recenter, reboot or stream restart.
    pub continuity_epoch: u64,
    pub parent: HeadingSegmentSample,
    pub child: HeadingSegmentSample,
    pub independent_witness: Option<IndependentHeadingWitness>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndependentWitnessSource {
    SharedJointCenterAcceleration,
    /// Must directly measure relative heading; a torso-only heading is insufficient.
    ExternalRelativeHeading,
}

/// A caller-supplied bounded relative-yaw observation. Independence means a
/// different physical constraint, not statistically independent sensor noise.
/// It must NOT be derived from the hinge candidate or the same hinge assumption.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct IndependentHeadingWitness {
    /// World-up correction to the child relative to the parent, in radians.
    pub relative_correction_radians: f64,
    /// Deterministic bound including timing, mounts, offsets and systematic error.
    /// It is an asserted bound, not uncertainty inferred from repeated samples.
    pub error_bound_radians: f64,
    pub observed_at_secs: f64,
    pub source: IndependentWitnessSource,
    pub caller_asserts_independent_physical_constraint: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeadingShadowStatus {
    Collecting,
    RejectedInput,
    ReusedMeasurement,
    Stale,
    HorizontallyDegenerate,
    InconsistentModel,
    InsufficientExcitation,
    ExcessiveCorrection,
    ModelConditionalCandidate,
    IndependentEvidenceInsufficient,
    IndependentEvidenceConflict,
    IndependentEvidenceVeto,
    IndependentlySupportedCandidate,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HeadingObservability {
    /// Exact local curvature of the FROZEN robust-weight, normalized quadratic
    /// surrogate at the reported fit. Not the Hessian of the full Huber loss.
    /// Objective: sum(w*||u_parent-Ry(delta)u_child||^2)/(2*sum(w)).
    /// Unit: dimensionless axis-vector squared per radian squared.
    pub frozen_weight_relative_curvature: f64,
    /// Scalar Gauss-Newton J'WJ/sum(w); different from observed cost curvature.
    pub relative_gauss_newton_curvature: f64,
    /// Two-yaw parameterization delta = child_yaw - parent_yaw.
    pub frozen_weight_hessian_eigenvalues: [f64; 2],
    pub normalized_cost_gradient: f64,
    pub frozen_weight_sum: f64,
    /// Always at least one. Two when relative heading is also degenerate.
    pub nullspace_dimension: usize,
    /// The full two-yaw Hessian is singular; None means no finite condition number.
    pub full_condition_number: Option<f64>,
    /// One scalar observable subspace has condition number 1 if nondegenerate.
    /// This does not describe information strength: inspect curvature separately.
    pub relative_subspace_condition_number: Option<f64>,
    pub common_mode_yaw_observable: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HeadingFitDiagnostics {
    pub measured_pairs: usize,
    pub span_secs: f64,
    pub mean_input_confidence: f64,
    pub median_residual_radians: f64,
    pub p95_residual_radians: f64,
    pub downweighted_fraction: f64,
    /// 95th minus 5th percentile flex angle; a heuristic, not extra evidence.
    pub flex_excursion_radians: f64,
    pub robust_iterations: usize,
    pub robust_converged: bool,
    pub observability: HeadingObservability,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WitnessStatus {
    Absent,
    Insufficient,
    ConsistentBounds,
    CandidateOutsideBounds,
    ConflictingBounds,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessDiagnostics {
    pub status: WitnessStatus,
    pub unique_samples: usize,
    pub span_secs: f64,
    pub latest_age_secs: Option<f64>,
    /// Intersection of asserted circular bounds, unwrapped near the first witness.
    pub intersection_center_radians: Option<f64>,
    pub intersection_half_width_radians: Option<f64>,
    pub sources: Vec<IndependentWitnessSource>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HeadingShadowReport {
    pub evaluated_at_secs: Option<f64>,
    pub parent_role: Role,
    pub child_role: Role,
    pub status: HeadingShadowStatus,
    pub reason: String,
    pub measured_pairs: usize,
    pub window_resets: usize,
    pub rejected_inputs: usize,
    pub last_reset_reason: Option<String>,
    /// May be present even when vetoed. This is a model fit, not an applied value.
    pub candidate_relative_yaw_radians: Option<f64>,
    /// Equal/opposite hypothetical increments. Absent whenever a gate blocks.
    /// No API in this module applies these to production tracking.
    pub hypothetical_parent_child_yaw_radians: Option<[f64; 2]>,
    pub fit: Option<HeadingFitDiagnostics>,
    pub witness: WitnessDiagnostics,
    pub live_correction_applied: bool,
}

#[derive(Clone, Debug)]
struct Entry {
    parent_axis: DVec3,
    child_axis: DVec3,
    flex: f64,
    confidence: f64,
    at_secs: f64,
    witness: Option<IndependentHeadingWitness>,
}

/// One calibrated segment pair, with bounded storage. A new instance should be
/// used for each joint. This object has no reference to TrackingCore yaw state.
pub struct RelativeHeadingEstimator {
    parent_role: Role,
    child_role: Role,
    config: HeadingShadowConfig,
    entries: VecDeque<Entry>,
    epoch: Option<(u64, u64)>,
    last_parent_at: Option<f64>,
    last_child_at: Option<f64>,
    last_witness_at: Option<f64>,
    last_observed_at: Option<f64>,
    window_resets: usize,
    rejected_inputs: usize,
    last_reset_reason: Option<String>,
    last_input_failure: Option<(HeadingShadowStatus, String)>,
}

impl RelativeHeadingEstimator {
    pub fn new(
        parent_role: Role,
        child_role: Role,
        config: HeadingShadowConfig,
    ) -> Result<Self, CoreError> {
        config.validate()?;
        if parent_role == child_role {
            return Err(CoreError(
                "shadow heading requires two different roles".into(),
            ));
        }
        Ok(Self {
            parent_role,
            child_role,
            config,
            entries: VecDeque::new(),
            epoch: None,
            last_parent_at: None,
            last_child_at: None,
            last_witness_at: None,
            last_observed_at: None,
            window_resets: 0,
            rejected_inputs: 0,
            last_reset_reason: None,
            last_input_failure: None,
        })
    }

    pub fn config(&self) -> &HeadingShadowConfig {
        &self.config
    }

    /// Clears observations but preserves role/configuration. Caller should also
    /// advance the epoch when resetting a transport or calibration reference.
    pub fn clear(&mut self) {
        self.reset_window("explicit reset");
        self.epoch = None;
        self.last_parent_at = None;
        self.last_child_at = None;
        self.last_witness_at = None;
        self.last_observed_at = None;
        self.last_input_failure = None;
    }

    pub fn observe(&mut self, now: f64, sample: HeadingPairSample) -> HeadingShadowReport {
        if self.last_observed_at.is_some_and(|previous| now < previous) {
            return self.reject(
                now,
                "evaluation clock moved backward; explicitly clear before a new time domain".into(),
                false,
            );
        }
        if now.is_finite() && now >= 0.0 {
            self.last_observed_at = Some(now);
        }
        let epoch = (sample.calibration_epoch, sample.continuity_epoch);
        if self.epoch != Some(epoch) {
            if self.epoch.is_some() {
                self.reset_window("calibration or continuity epoch changed");
            }
            self.epoch = Some(epoch);
            self.last_parent_at = None;
            self.last_child_at = None;
            self.last_witness_at = None;
        }
        let checked = self.check_pair(now, &sample);
        let (parent_q, child_q) = match checked {
            Ok(pair) => pair,
            Err(reason) => return self.reject(now, reason, false),
        };
        let parent_at = sample.parent.measured_at_secs;
        let child_at = sample.child.measured_at_secs;
        if self.last_parent_at.is_some_and(|t| parent_at < t)
            || self.last_child_at.is_some_and(|t| child_at < t)
        {
            return self.reject(now, "out-of-order measurement".into(), false);
        }
        if self.last_parent_at == Some(parent_at) || self.last_child_at == Some(child_at) {
            return self.reject(
                now,
                "pair reuses an already counted measurement".into(),
                true,
            );
        }
        if self
            .last_parent_at
            .is_some_and(|t| parent_at - t > self.config.max_gap_secs)
            || self
                .last_child_at
                .is_some_and(|t| child_at - t > self.config.max_gap_secs)
        {
            self.reset_window("measurement gap exceeds continuity limit");
        }
        let witness = match sample.independent_witness {
            Some(w) => {
                if let Err(reason) = self.check_witness(now, parent_at, child_at, w) {
                    return self.reject(now, reason, false);
                }
                if self.last_witness_at.is_some_and(|t| w.observed_at_secs < t) {
                    return self.reject(now, "out-of-order independent witness".into(), false);
                }
                if self.last_witness_at == Some(w.observed_at_secs) {
                    None
                } else {
                    self.last_witness_at = Some(w.observed_at_secs);
                    Some(w)
                }
            }
            None => None,
        };
        let parent_axis = parent_q * unit_vector(self.config.parent_hinge_axis).unwrap();
        let child_axis = child_q * unit_vector(self.config.child_hinge_axis).unwrap();
        let parent_long = parent_q * unit_vector(self.config.parent_long_axis).unwrap();
        let child_long = child_q * unit_vector(self.config.child_long_axis).unwrap();
        self.entries.push_back(Entry {
            parent_axis,
            child_axis,
            flex: parent_long.dot(child_long).clamp(-1.0, 1.0).acos(),
            confidence: sample.parent.confidence.min(sample.child.confidence),
            at_secs: parent_at.min(child_at),
            witness,
        });
        while self.entries.len() > self.config.max_samples
            || self
                .entries
                .front()
                .is_some_and(|e| now - e.at_secs > self.config.window_secs)
        {
            self.entries.pop_front();
        }
        self.last_parent_at = Some(parent_at);
        self.last_child_at = Some(child_at);
        self.last_input_failure = None;
        self.report(now)
    }

    pub fn report(&self, now: f64) -> HeadingShadowReport {
        let mut report = HeadingShadowReport {
            evaluated_at_secs: now.is_finite().then_some(now),
            parent_role: self.parent_role,
            child_role: self.child_role,
            status: HeadingShadowStatus::Collecting,
            reason: "collecting unique measured pairs across a continuous window".into(),
            measured_pairs: self.entries.len(),
            window_resets: self.window_resets,
            rejected_inputs: self.rejected_inputs,
            last_reset_reason: self.last_reset_reason.clone(),
            candidate_relative_yaw_radians: None,
            hypothetical_parent_child_yaw_radians: None,
            fit: None,
            witness: empty_witness(),
            live_correction_applied: false,
        };
        if let Some((status, reason)) = &self.last_input_failure {
            report.status = *status;
            report.reason.clone_from(reason);
            return report;
        }
        if !now.is_finite() {
            report.status = HeadingShadowStatus::RejectedInput;
            report.reason = "non-finite evaluation time".into();
            return report;
        }
        if self.last_observed_at.is_some_and(|observed| now < observed) {
            report.status = HeadingShadowStatus::RejectedInput;
            report.reason = "evaluation precedes accumulated observations".into();
            return report;
        }
        let Some(latest) = self.entries.back() else {
            return report;
        };
        if now < latest.at_secs || now - latest.at_secs > self.config.stale_after_secs {
            report.status = HeadingShadowStatus::Stale;
            report.reason = "latest unique measured pair is stale or in the future".into();
            return report;
        }
        let entries: Vec<_> = self
            .entries
            .iter()
            .filter(|e| now - e.at_secs <= self.config.window_secs)
            .collect();
        report.measured_pairs = entries.len();
        if entries.is_empty() {
            report.status = HeadingShadowStatus::Stale;
            report.reason = "no measured pairs remain in the current window".into();
            return report;
        }
        let span = entries.last().unwrap().at_secs - entries.first().unwrap().at_secs;
        if entries.len() < self.config.min_samples || span < self.config.min_span_secs {
            return report;
        }
        let fit = fit_axes(&entries, &self.config);
        let delta = fit.delta;
        let diagnostics = fit.diagnostics;
        report.candidate_relative_yaw_radians =
            (diagnostics.observability.nullspace_dimension == 1).then_some(delta);
        report.witness = assess_witnesses(&entries, delta, now, &self.config);
        let (status, reason) = if diagnostics.observability.frozen_weight_relative_curvature
            < self.config.min_relative_curvature
        {
            (
                HeadingShadowStatus::HorizontallyDegenerate,
                "relative heading has insufficient horizontal cost curvature",
            )
        } else if !diagnostics.robust_converged
            || diagnostics.median_residual_radians > self.config.max_median_residual_radians
            || diagnostics.downweighted_fraction > self.config.max_downweighted_fraction
        {
            (
                HeadingShadowStatus::InconsistentModel,
                "axis model is inconsistent or robust optimization has not converged",
            )
        } else if diagnostics.flex_excursion_radians < self.config.min_flex_excursion_radians {
            (
                HeadingShadowStatus::InsufficientExcitation,
                "flex excursion heuristic is insufficient; curvature alone is not validation",
            )
        } else if delta.abs() > self.config.max_candidate_radians {
            (
                HeadingShadowStatus::ExcessiveCorrection,
                "model candidate exceeds the shadow experiment's bounded correction range",
            )
        } else {
            match report.witness.status {
                WitnessStatus::Absent => (
                    HeadingShadowStatus::ModelConditionalCandidate,
                    "hinge-model candidate only; consistent real twist remains indistinguishable",
                ),
                WitnessStatus::Insufficient => (
                    HeadingShadowStatus::IndependentEvidenceInsufficient,
                    "independent evidence is present but lacks fresh coverage or a sufficiently tight bound",
                ),
                WitnessStatus::ConflictingBounds => (
                    HeadingShadowStatus::IndependentEvidenceConflict,
                    "independent witness bounds conflict; reject the evidence model or constant-offset assumption",
                ),
                WitnessStatus::CandidateOutsideBounds => (
                    HeadingShadowStatus::IndependentEvidenceVeto,
                    "independent physical evidence vetoes the hinge-model candidate",
                ),
                WitnessStatus::ConsistentBounds => (
                    HeadingShadowStatus::IndependentlySupportedCandidate,
                    "candidate agrees with caller-asserted independent bounds; still shadow-only",
                ),
            }
        };
        report.status = status;
        report.reason = reason.into();
        if matches!(
            status,
            HeadingShadowStatus::ModelConditionalCandidate
                | HeadingShadowStatus::IndependentlySupportedCandidate
        ) {
            report.hypothetical_parent_child_yaw_radians = Some([-delta / 2.0, delta / 2.0]);
        }
        report.fit = Some(diagnostics);
        report
    }

    fn check_pair(&self, now: f64, sample: &HeadingPairSample) -> Result<(DQuat, DQuat), String> {
        if !now.is_finite() || now < 0.0 {
            return Err("invalid evaluation timestamp".into());
        }
        if sample.parent.role != self.parent_role || sample.child.role != self.child_role {
            return Err("segment roles do not match this estimator".into());
        }
        for segment in [&sample.parent, &sample.child] {
            if !segment.valid || !matches!(segment.provenance, SensorProvenance::Measured) {
                return Err(
                    "shadow fit requires valid measured orientations; held evidence is excluded"
                        .into(),
                );
            }
            if !segment.measured_at_secs.is_finite()
                || segment.measured_at_secs < 0.0
                || segment.measured_at_secs > now
                || now - segment.measured_at_secs > self.config.stale_after_secs
            {
                return Err("stale, future or invalid measured timestamp".into());
            }
            if !segment.confidence.is_finite()
                || !(self.config.min_input_confidence..=1.0).contains(&segment.confidence)
            {
                return Err("input confidence is below threshold or invalid".into());
            }
        }
        if (sample.parent.measured_at_secs - sample.child.measured_at_secs).abs()
            > self.config.max_pair_skew_secs
        {
            return Err("measurement pair exceeds receipt-time skew limit".into());
        }
        Ok((
            unit_quat(sample.parent.orientation_xyzw.map(f64::from)).map_err(|e| e.0)?,
            unit_quat(sample.child.orientation_xyzw.map(f64::from)).map_err(|e| e.0)?,
        ))
    }

    fn check_witness(
        &self,
        now: f64,
        parent_at: f64,
        child_at: f64,
        witness: IndependentHeadingWitness,
    ) -> Result<(), String> {
        if !witness.caller_asserts_independent_physical_constraint
            || !witness.relative_correction_radians.is_finite()
            || !witness.error_bound_radians.is_finite()
            || !(0.0..PI / 2.0).contains(&witness.error_bound_radians)
            || !witness.observed_at_secs.is_finite()
            || witness.observed_at_secs < 0.0
            || witness.observed_at_secs > now
            || now - witness.observed_at_secs > self.config.stale_after_secs
            || (witness.observed_at_secs - parent_at).abs() > self.config.max_pair_skew_secs
            || (witness.observed_at_secs - child_at).abs() > self.config.max_pair_skew_secs
        {
            return Err("independent witness lacks valid bounds, alignment, freshness or independence assertion".into());
        }
        Ok(())
    }

    fn reset_window(&mut self, reason: &str) {
        self.entries.clear();
        self.window_resets += 1;
        self.last_reset_reason = Some(reason.into());
    }

    fn reject(&mut self, now: f64, reason: String, reused: bool) -> HeadingShadowReport {
        self.rejected_inputs += 1;
        if !reused {
            self.reset_window(&reason);
        }
        let status = if reused {
            HeadingShadowStatus::ReusedMeasurement
        } else {
            HeadingShadowStatus::RejectedInput
        };
        self.last_input_failure = Some((status, reason));
        self.report(now)
    }
}

struct Fit {
    delta: f64,
    diagnostics: HeadingFitDiagnostics,
}

fn fit_axes(entries: &[&Entry], config: &HeadingShadowConfig) -> Fit {
    let threshold = 2.0 * (config.huber_angle_radians / 2.0).sin();
    // A fixed circular seed grid avoids trusting one potentially corrupted pair.
    // This is a numerical strategy, not a guarantee of globally convex Huber loss.
    let cost = |angle: f64| -> f64 {
        let rotation = DQuat::from_rotation_y(angle);
        entries
            .iter()
            .map(|e| {
                let residual = (e.parent_axis - rotation * e.child_axis).length();
                e.confidence
                    * if residual <= threshold {
                        0.5 * residual * residual
                    } else {
                        threshold * (residual - 0.5 * threshold)
                    }
            })
            .sum()
    };
    let mut delta = (0..24)
        .map(|i| -PI + i as f64 * TAU / 24.0)
        .min_by(|a, b| cost(*a).total_cmp(&cost(*b)))
        .unwrap();
    let mut converged = false;
    let mut iterations = 0;
    for iteration in 0..32 {
        iterations = iteration + 1;
        let (a, b, _, _) = coefficients(entries, delta, threshold);
        let next = b.atan2(a);
        if wrap(next - delta).abs() < 1e-9 {
            delta = next;
            converged = true;
            break;
        }
        delta = next;
    }
    let (a, b, weight_sum, gn_sum) = coefficients(entries, delta, threshold);
    let curvature = (a * delta.cos() + b * delta.sin()) / weight_sum;
    let gradient = (a * delta.sin() - b * delta.cos()) / weight_sum;
    let rotation = DQuat::from_rotation_y(delta);
    let mut residuals: Vec<f64> = entries
        .iter()
        .map(|e| {
            let chord = (e.parent_axis - rotation * e.child_axis).length();
            2.0 * (chord / 2.0).clamp(0.0, 1.0).asin()
        })
        .collect();
    let downweighted = residuals
        .iter()
        .filter(|r| **r > config.huber_angle_radians + 1e-9)
        .count();
    residuals.sort_by(f64::total_cmp);
    let mut flex: Vec<_> = entries.iter().map(|e| e.flex).collect();
    flex.sort_by(f64::total_cmp);
    let nondegenerate = curvature > 1e-10;
    Fit {
        delta,
        diagnostics: HeadingFitDiagnostics {
            measured_pairs: entries.len(),
            span_secs: entries.last().unwrap().at_secs - entries.first().unwrap().at_secs,
            mean_input_confidence: entries.iter().map(|e| e.confidence).sum::<f64>()
                / entries.len() as f64,
            median_residual_radians: percentile(&residuals, 0.5),
            p95_residual_radians: percentile(&residuals, 0.95),
            downweighted_fraction: downweighted as f64 / entries.len() as f64,
            flex_excursion_radians: percentile(&flex, 0.95) - percentile(&flex, 0.05),
            robust_iterations: iterations,
            robust_converged: converged,
            observability: HeadingObservability {
                frozen_weight_relative_curvature: curvature,
                relative_gauss_newton_curvature: gn_sum / weight_sum,
                frozen_weight_hessian_eigenvalues: [0.0, 2.0 * curvature],
                normalized_cost_gradient: gradient,
                frozen_weight_sum: weight_sum,
                nullspace_dimension: if nondegenerate { 1 } else { 2 },
                full_condition_number: None,
                relative_subspace_condition_number: nondegenerate.then_some(1.0),
                common_mode_yaw_observable: false,
            },
        },
    }
}

fn coefficients(entries: &[&Entry], delta: f64, threshold: f64) -> (f64, f64, f64, f64) {
    let rotation = DQuat::from_rotation_y(delta);
    let (mut a, mut b, mut weight_sum, mut gn_sum) = (0.0, 0.0, 0.0, 0.0);
    for e in entries {
        let residual = (e.parent_axis - rotation * e.child_axis).length();
        let weight = e.confidence * (threshold / residual.max(threshold)).min(1.0);
        a += weight * (e.parent_axis.x * e.child_axis.x + e.parent_axis.z * e.child_axis.z);
        b += weight * (e.parent_axis.x * e.child_axis.z - e.parent_axis.z * e.child_axis.x);
        weight_sum += weight;
        gn_sum += weight * (e.child_axis.x * e.child_axis.x + e.child_axis.z * e.child_axis.z);
    }
    (a, b, weight_sum, gn_sum)
}

fn assess_witnesses(
    entries: &[&Entry],
    delta: f64,
    now: f64,
    config: &HeadingShadowConfig,
) -> WitnessDiagnostics {
    let witnesses: Vec<_> = entries.iter().filter_map(|e| e.witness).collect();
    if witnesses.is_empty() {
        return empty_witness();
    }
    let mut result = empty_witness();
    result.status = WitnessStatus::Insufficient;
    result.unique_samples = witnesses.len();
    result.span_secs =
        witnesses.last().unwrap().observed_at_secs - witnesses.first().unwrap().observed_at_secs;
    result.latest_age_secs = Some(now - witnesses.last().unwrap().observed_at_secs);
    // Unwrap near a witness, not near the possibly contradicted candidate. Each
    // half-width is < pi/2, so any common intersection must lie inside the first
    // witness's arc. This handles consistent observations straddling +/-pi.
    let reference = wrap(witnesses[0].relative_correction_radians);
    let (mut low, mut high) = (f64::NEG_INFINITY, f64::INFINITY);
    let mut tight = true;
    for w in &witnesses {
        if !result.sources.contains(&w.source) {
            result.sources.push(w.source);
        }
        let relative = wrap(w.relative_correction_radians - reference);
        low = low.max(relative - w.error_bound_radians);
        high = high.min(relative + w.error_bound_radians);
        tight &= w.error_bound_radians <= config.max_witness_error_bound_radians;
    }
    if low > high + 1e-10 {
        result.status = WitnessStatus::ConflictingBounds;
    } else {
        result.intersection_center_radians = Some(wrap(reference + (low + high) / 2.0));
        result.intersection_half_width_radians = Some(((high - low) / 2.0).max(0.0));
        let candidate_offset = wrap(delta - reference);
        if candidate_offset < low - 1e-10 || candidate_offset > high + 1e-10 {
            // Even one bounded physical observation can contradict a hypothesis.
            result.status = WitnessStatus::CandidateOutsideBounds;
        } else if tight
            && witnesses.len() >= config.min_witness_samples
            && result.span_secs >= config.min_witness_span_secs
            && result.latest_age_secs.unwrap() <= config.stale_after_secs
        {
            result.status = WitnessStatus::ConsistentBounds;
        }
    }
    result
}

fn empty_witness() -> WitnessDiagnostics {
    WitnessDiagnostics {
        status: WitnessStatus::Absent,
        unique_samples: 0,
        span_secs: 0.0,
        latest_age_secs: None,
        intersection_center_radians: None,
        intersection_half_width_radians: None,
        sources: Vec::new(),
    }
}

/// Required raw-data contract for the OPTIONAL shared-joint acceleration factor.
/// Current Slime fused-quaternion packets do not establish this contract. All
/// values must refer to the same actual sensor instant in an aligned time domain.
#[derive(Clone, Debug)]
pub struct JointCenterKinematics {
    pub orientation_xyzw: [f64; 4],
    /// Accelerometer specific force in sensor axes, INCLUDING the gravity term.
    pub specific_force_sensor_mps2: [f64; 3],
    pub angular_velocity_sensor_radps: [f64; 3],
    pub angular_acceleration_sensor_radps2: [f64; 3],
    /// Sensor origin TO the joint center, expressed in sensor axes (meters).
    pub sensor_to_joint_sensor_m: [f64; 3],
    pub sampled_at_secs: f64,
    pub provenance: SensorProvenance,
}

/// a_joint = R * (specific_force + alpha x r + omega x (omega x r)) + gravity.
/// This does not estimate lever arms, differentiate gyros or invent an error bound.
pub fn reconstruct_joint_center_acceleration(
    input: &JointCenterKinematics,
) -> Result<[f64; 3], CoreError> {
    if !matches!(input.provenance, SensorProvenance::Measured)
        || !input.sampled_at_secs.is_finite()
        || input.sampled_at_secs < 0.0
    {
        return Err(CoreError(
            "joint factor requires actual timed raw measurements".into(),
        ));
    }
    let q = unit_quat(input.orientation_xyzw)?;
    let force = finite_vector(input.specific_force_sensor_mps2)?;
    let omega = finite_vector(input.angular_velocity_sensor_radps)?;
    let alpha = finite_vector(input.angular_acceleration_sensor_radps2)?;
    let offset = finite_vector(input.sensor_to_joint_sensor_m)?;
    let acceleration = q * (force + alpha.cross(offset) + omega.cross(omega.cross(offset)))
        + DVec3::new(0.0, -9.80665, 0.0);
    if !acceleration.is_finite() {
        return Err(CoreError("joint acceleration overflow".into()));
    }
    Ok(acceleration.to_array())
}

/// Forms a bounded yaw witness from the common joint's horizontal acceleration.
/// Error bounds apply to each RECONSTRUCTED world vector and must already include
/// raw noise, orientation/tilt, lever-arm, differentiation and timing/model errors.
/// Gravity/vertical acceleration alone cannot yield a yaw witness.
pub fn joint_acceleration_witness(
    parent: &JointCenterKinematics,
    child: &JointCenterKinematics,
    parent_vector_error_bound_mps2: f64,
    child_vector_error_bound_mps2: f64,
    max_sample_skew_secs: f64,
) -> Result<IndependentHeadingWitness, CoreError> {
    if !max_sample_skew_secs.is_finite()
        || max_sample_skew_secs <= 0.0
        || (parent.sampled_at_secs - child.sampled_at_secs).abs() > max_sample_skew_secs
    {
        return Err(CoreError(
            "joint acceleration samples are not time aligned".into(),
        ));
    }
    let p = DVec3::from_array(reconstruct_joint_center_acceleration(parent)?);
    let c = DVec3::from_array(reconstruct_joint_center_acceleration(child)?);
    let ph = p.x.hypot(p.z);
    let ch = c.x.hypot(c.z);
    let bound = |error: f64, norm: f64| -> Result<f64, CoreError> {
        if !error.is_finite() || error < 0.0 || error >= norm || norm <= 1e-9 {
            return Err(CoreError(
                "horizontal joint acceleration does not exceed its asserted error bound".into(),
            ));
        }
        Ok((error / norm).asin())
    };
    let error_bound =
        bound(parent_vector_error_bound_mps2, ph)? + bound(child_vector_error_bound_mps2, ch)?;
    if error_bound >= PI / 2.0 {
        return Err(CoreError(
            "joint acceleration heading bound is too wide".into(),
        ));
    }
    // A yaw rotation preserves full magnitude and vertical component. If those
    // disagree beyond the supplied bounds, no yaw can satisfy this physical model.
    let combined_error = parent_vector_error_bound_mps2 + child_vector_error_bound_mps2;
    if (p.y - c.y).abs() > combined_error + 1e-9 || (ph - ch).abs() > combined_error + 1e-9 {
        return Err(CoreError(
            "joint acceleration vectors violate the shared-point model bounds".into(),
        ));
    }
    Ok(IndependentHeadingWitness {
        relative_correction_radians: (p.x * c.z - p.z * c.x).atan2(p.x * c.x + p.z * c.z),
        error_bound_radians: error_bound,
        observed_at_secs: parent.sampled_at_secs.min(child.sampled_at_secs),
        source: IndependentWitnessSource::SharedJointCenterAcceleration,
        caller_asserts_independent_physical_constraint: true,
    })
}

fn finite_vector(value: [f64; 3]) -> Result<DVec3, CoreError> {
    let v = DVec3::from_array(value);
    if !v.is_finite() || v.abs().max_element() > 1e6 {
        Err(CoreError("non-finite or excessive vector input".into()))
    } else {
        Ok(v)
    }
}

fn unit_vector(value: [f64; 3]) -> Result<DVec3, CoreError> {
    let v = finite_vector(value)?;
    if v.length_squared() < 1e-12 {
        Err(CoreError("zero axis vector".into()))
    } else {
        Ok(v.normalize())
    }
}

fn unit_quat(value: [f64; 4]) -> Result<DQuat, CoreError> {
    let q = DQuat::from_array(value);
    if !q.is_finite() || !(0.5..=1.5).contains(&q.length_squared()) {
        Err(CoreError("invalid quaternion".into()))
    } else {
        Ok(q.normalize())
    }
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    let x = (sorted.len() - 1) as f64 * p;
    let low = x.floor() as usize;
    let high = x.ceil() as usize;
    sorted[low] + (sorted[high] - sorted[low]) * (x - low as f64)
}
