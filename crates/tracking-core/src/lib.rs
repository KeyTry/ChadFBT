//! Deterministic, head-anchored tracking. Inputs are meters in right-handed
//! standing space (+Y up, -Z forward), with XYZW quaternions.
//!
//! Positions reconstructed from IMU orientations remain inferred. Confidence is
//! an engineering quality score, not a calibrated probability or accuracy claim.

use glam::{Quat, Vec3};
pub mod accel_heading;
pub mod chain;
pub mod contact;
pub mod contact_heading;
pub mod forward_calibration;
pub mod heading_shadow;
pub mod leg_ik;
pub mod manual_mounting;
pub mod motion_calibration;
pub mod prediction;
pub mod preview_motion;
pub mod stance_heading;
pub mod synthetic;
pub mod tracking_space;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Chest,
    Hip,
    LeftThigh,
    RightThigh,
    LeftCalf,
    RightCalf,
    LeftFoot,
    RightFoot,
    LeftUpperArm,
    RightUpperArm,
}

impl Role {
    pub const CORE: [Self; 6] = [
        Self::Chest,
        Self::Hip,
        Self::LeftThigh,
        Self::RightThigh,
        Self::LeftCalf,
        Self::RightCalf,
    ];
    pub const ALL: [Self; 10] = [
        Self::Chest,
        Self::Hip,
        Self::LeftThigh,
        Self::RightThigh,
        Self::LeftCalf,
        Self::RightCalf,
        Self::LeftFoot,
        Self::RightFoot,
        Self::LeftUpperArm,
        Self::RightUpperArm,
    ];
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RoleAssignment {
    pub sensor_id: String,
    pub role: Role,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorSample {
    pub id: String,
    pub orientation_xyzw: [f32; 4],
    pub received_at_secs: f64,
    /// Gravity-removed linear acceleration in m/s². Omit if its convention is unknown.
    pub acceleration_mps2: Option<[f32; 3]>,
    /// Actual quaternion packets remain Measured. Held evidence must retain the
    /// original received_at_secs and must never fabricate a new measurement.
    #[serde(default)]
    pub provenance: SensorProvenance,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SensorProvenance {
    #[default]
    Measured,
    /// A compatibility assumption supplied by the validated firmware transport.
    /// Heartbeat/sequence continuity is not proof of current sensor orientation.
    FirmwareHeld {
        firmware: String,
        observed_at_secs: f64,
        last_transport_at_secs: f64,
        continuity_since_secs: f64,
        sensor_status_healthy: bool,
        sequence_contiguous: bool,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AnchorPose {
    pub position: [f32; 3],
    pub orientation_xyzw: [f32; 4],
    pub received_at_secs: f64,
    pub valid: bool,
    /// Standing-space linear velocity (m/s) reported with the pose, if known.
    #[serde(default)]
    pub linear_velocity: Option<[f32; 3]>,
    /// Standing-space axis-angle angular velocity (rad/s), if known. Stored
    /// only; orientations are never extrapolated from it.
    #[serde(default)]
    pub angular_velocity: Option<[f32; 3]>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Anchors {
    pub hmd: Option<AnchorPose>,
    pub left_hand: Option<AnchorPose>,
    pub right_hand: Option<AnchorPose>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Anthropometrics {
    pub standing_eye_height_m: f32,
    pub head_to_neck_m: f32,
    pub neck_to_chest_m: f32,
    pub chest_to_hip_m: f32,
    pub hip_width_m: f32,
    pub shoulder_width_m: f32,
    pub thigh_length_m: f32,
    pub calf_length_m: f32,
    pub foot_length_m: f32,
    pub ankle_to_sole_m: f32,
    pub upper_arm_length_m: f32,
    pub forearm_length_m: f32,
    /// Horizontal distance from the HMD eye point forward of the neck pivot,
    /// measured along the head's -Z axis. Profiles without it use the default.
    pub eye_forward_m: f32,
}

impl Default for Anthropometrics {
    fn default() -> Self {
        // The default vertical chain (head_to_neck through ankle_to_sole)
        // sums to the default standing eye height, so neutral soles touch y=0.
        Self {
            standing_eye_height_m: 1.65,
            head_to_neck_m: 0.15,
            neck_to_chest_m: 0.20,
            chest_to_hip_m: 0.38,
            hip_width_m: 0.30,
            shoulder_width_m: 0.39,
            thigh_length_m: 0.42,
            calf_length_m: 0.42,
            foot_length_m: 0.25,
            ankle_to_sole_m: 0.08,
            upper_arm_length_m: 0.30,
            forearm_length_m: 0.26,
            eye_forward_m: 0.08,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct TrackingConfig {
    pub anthropometrics: Anthropometrics,
    pub stale_after_secs: f64,
    pub stable_window_secs: f64,
    pub stable_angle_degrees: f32,
    pub stable_acceleration_mps2: f32,
    pub neutral_tilt_degrees: f32,
    pub standing_height_tolerance_m: f32,
    pub recovery_max_yaw_degrees: f32,
    pub recovery_timeout_secs: f64,
    pub enable_joint_yaw_correction: bool,
    /// Experimental inferred desktop displacement; never used with a live HMD.
    pub enable_preview_motion: bool,
    pub joint_yaw_deadband_degrees: f32,
    pub joint_yaw_max_residual_degrees: f32,
    pub yaw_correction_rate_degrees_per_sec: f32,
    #[serde(default)]
    pub estimator: EstimatorConfig,
}

/// Prediction, velocity and tracking-space continuity options. Every feature
/// defaults to off in the core so a default configuration keeps the original
/// latest-sample behaviour; the application chooses its own defaults.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EstimatorConfig {
    /// Extrapolate fresh measured sensor orientations to the solve time.
    pub enable_prediction: bool,
    /// Extra extrapolation beyond the solve time, 0..=0.03 s.
    pub prediction_lead_secs: f32,
    pub prediction: prediction::AngularRateConfig,
    /// Advance HMD and hand anchor positions by their reported linear velocity
    /// over the anchor age (at most 30 ms). Orientation is never extrapolated.
    pub enable_hmd_extrapolation: bool,
    /// Treat an HMD pose discontinuity (recenter or tracking jump) as a
    /// rotation of the standing space and carry calibration over.
    pub enable_auto_rebase: bool,
    /// Stance heading filter (`stance_heading`): a per-core-segment world-yaw
    /// correction estimated from planted-foot displacement against the live
    /// headset plus weak anatomical priors. Synthetic validation only.
    pub enable_stance_heading: bool,
    /// Contact-consistent leg IK (`leg_ik`): root correction, foot lock and a
    /// flat un-instrumented foot. Changes inferred positions (and the
    /// orientation of feet without a foot IMU) only. Synthetic validation only.
    pub enable_leg_ik: bool,
    /// Stationary-foot hypothesis thresholds used by both layers above.
    #[serde(default)]
    pub contact: contact::ContactConfig,
    #[serde(default)]
    pub stance: stance_heading::StanceHeadingConfig,
    #[serde(default)]
    pub leg_ik: leg_ik::LegIkConfig,
}

impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            enable_prediction: false,
            prediction_lead_secs: 0.0,
            prediction: prediction::AngularRateConfig::default(),
            enable_hmd_extrapolation: false,
            enable_auto_rebase: false,
            enable_stance_heading: false,
            enable_leg_ik: false,
            contact: contact::ContactConfig::default(),
            stance: stance_heading::StanceHeadingConfig::default(),
            leg_ik: leg_ik::LegIkConfig::default(),
        }
    }
}

impl EstimatorConfig {
    pub const MAX_PREDICTION_LEAD_SECS: f32 = 0.03;

    pub fn validate(&self) -> Result<(), CoreError> {
        if !self.prediction_lead_secs.is_finite()
            || !(0.0..=Self::MAX_PREDICTION_LEAD_SECS).contains(&self.prediction_lead_secs)
        {
            return Err(CoreError(format!(
                "Invalid prediction lead: expected 0–{} ms",
                Self::MAX_PREDICTION_LEAD_SECS * 1000.0
            )));
        }
        let p = &self.prediction;
        let positive = [p.min_dt_secs, p.max_dt_secs, p.max_rate_rad_per_sec];
        let non_negative = [
            p.smoothing_tau_secs,
            p.silence_secs,
            p.max_extrapolation_secs,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || non_negative.iter().any(|v| !v.is_finite() || *v < 0.0)
            || p.min_dt_secs > p.max_dt_secs
            || p.max_extrapolation_secs > 0.1
        {
            return Err(CoreError("Invalid angular-rate configuration".into()));
        }
        validate_contact_config(&self.contact)?;
        validate_stance_config(&self.stance)?;
        // The solver sanitises on its own; a configuration that sanitising
        // would change (nonfinite, negative or out of range) is rejected here.
        if self.leg_ik.sanitized() != self.leg_ik {
            return Err(CoreError("Invalid leg IK configuration".into()));
        }
        Ok(())
    }
}

fn validate_contact_config(c: &contact::ContactConfig) -> Result<(), CoreError> {
    let all = [
        c.velocity_tau_secs,
        c.yaw_rate_tau_secs,
        c.enter_speed_mps,
        c.enter_height_m,
        c.enter_yaw_rate_dps,
        c.settle_secs,
        c.exit_speed_mps,
        c.exit_height_m,
        c.exit_yaw_rate_dps,
        c.exit_vertical_speed_mps,
        c.max_planted_yaw_degrees,
        c.max_gap_secs,
        c.max_speed_allowance_mps,
        c.torso_allowance_gain,
        c.double_support_relative_speed_mps,
    ];
    let time_constants = [c.velocity_tau_secs, c.yaw_rate_tau_secs, c.settle_secs];
    let entry = [c.enter_speed_mps, c.enter_height_m, c.enter_yaw_rate_dps];
    if all.iter().any(|v| !v.is_finite())
        || time_constants.iter().any(|v| !(0.0..=5.0).contains(v))
        || entry.iter().any(|v| *v <= 0.0)
        || !(0.0..=2.0).contains(&c.max_speed_allowance_mps)
        || !(0.0..=5.0).contains(&c.torso_allowance_gain)
        || c.double_support_relative_speed_mps <= 0.0
        || c.exit_speed_mps < c.enter_speed_mps
        || c.exit_height_m < c.enter_height_m
        || c.exit_yaw_rate_dps < c.enter_yaw_rate_dps
        || !(c.max_gap_secs > 0.0 && c.max_gap_secs <= 1.0)
    {
        return Err(CoreError("Invalid contact configuration".into()));
    }
    Ok(())
}

fn validate_stance_config(s: &stance_heading::StanceHeadingConfig) -> Result<(), CoreError> {
    let (k, p, c) = (&s.knee_prior, &s.pelvis_prior, &s.chest_prior);
    let non_negative = [
        s.process_sigma_degrees,
        s.process_rate_coefficient,
        s.measure_excitation_m,
        s.min_excitation_m,
        s.min_keyframe_secs,
        s.neck_sigma_m_per_rad,
        s.creep_sigma_mps,
        s.pivot_sigma_m_per_rad,
        s.motion_noise_ratio,
        s.torso_motion_noise_ratio,
        s.length_uncertainty,
        s.angular_speed_tau_secs,
        s.inflation_step_degrees,
        s.inflation_max_sigma_degrees,
        s.nis_average_threshold,
        s.allowance_sigma_gain,
        s.starvation_head_speed_mps,
        s.starvation_inflation_degrees,
        k.min_flexion_degrees,
        k.min_flexion_change_degrees,
        k.change_window_secs,
        k.deadband_degrees,
        k.rate_hz,
        p.deadband_degrees,
        p.rate_hz,
        c.deadband_degrees,
        c.rate_hz,
    ];
    // A zero headset sigma would make the displacement noise singular.
    let positive = [
        s.initial_sigma_degrees,
        s.max_sigma_degrees,
        s.max_keyframe_secs,
        s.max_head_speed_mps,
        s.max_frame_gap_secs,
        s.hmd_sigma_m,
        s.gate_chi2,
        s.prior_gate_chi2,
        s.max_step_degrees,
        k.sigma_degrees,
        p.sigma_degrees,
        c.sigma_degrees,
    ];
    if non_negative.iter().any(|v| !v.is_finite() || *v < 0.0)
        || positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
        || [k.max_flexion_degrees, p.max_tilt_degrees, c.max_tilt_degrees]
            .iter()
            .any(|v| !v.is_finite())
        // Non-positive starvation time disables starvation inflation.
        || !s.starvation_secs.is_finite()
        // Non-positive disables the torso-dominated increment gate.
        || !s.max_torso_excitation_ratio.is_finite()
        || !(0.0..=1.0).contains(&s.nis_average_weight)
        || !(0.0..=90.0).contains(&s.allowance_max_degrees)
        || !(s.max_leg_correction_degrees > 0.0 && s.max_leg_correction_degrees <= 180.0)
        || !(s.max_torso_correction_degrees > 0.0 && s.max_torso_correction_degrees <= 180.0)
        || s.inflation_max_sigma_degrees > s.max_sigma_degrees
        || s.initial_sigma_degrees > s.max_sigma_degrees
        || s.max_sigma_degrees > 180.0
        || s.min_keyframe_secs > s.max_keyframe_secs
        || s.max_frame_gap_secs > 1.0
        || k.min_flexion_degrees > k.max_flexion_degrees
        || k.max_flexion_degrees > 180.0
        || !(0.0..=90.0).contains(&p.max_tilt_degrees)
        || !(0.0..=90.0).contains(&c.max_tilt_degrees)
        || s.outlier_release_count == 0
    {
        return Err(CoreError("Invalid stance heading configuration".into()));
    }
    Ok(())
}

/// Per-frame estimator diagnostics. Later estimator phases add fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EstimatorStatus {
    /// At least one segment orientation was extrapolated from a fresh
    /// measured angular rate in this frame.
    pub prediction_active: bool,
    /// Time by which the HMD position was advanced along its velocity.
    pub hmd_extrapolated_secs: f32,
    /// Explicit and automatic standing-space rebases since the core started.
    pub standing_space_rebases: u32,
    /// Angle of the most recent rebase, degrees about +Y.
    pub last_rebase_degrees: f32,
    /// HMD discontinuities detected by the automatic detector, including
    /// ones that could not be rebased (no usable horizontal facing).
    pub tracking_discontinuities: u32,
    /// Evidence behind the most recent automatic rebase; `None` before one
    /// and after an explicit `rebase_standing_space` call.
    pub last_rebase_evidence: Option<tracking_space::RebaseEvidence>,
    /// Usable headset samples resumed after a gap longer than
    /// `tracking_space::MAX_EVENT_GAP_SECS` with a live calibration: the
    /// calibration was kept and the stance filter's uncertainty widened.
    pub headset_resumptions: u32,
    /// Length of the most recent such gap, seconds.
    pub last_headset_gap_secs: f32,
    /// The stance heading filter was stepped in this frame (enabled, live
    /// direction-calibrated HMD session, at least one usable foot chain).
    pub stance_active: bool,
    /// Stance filter state. `corrections` and `sigma` are radians per
    /// `Role::CORE` segment. The corrections applied to this frame's published
    /// rotations are the ones from before this frame's step: an update made in
    /// a frame takes effect from the next frame. `status` is `idle` and
    /// `release_requests` are false when the filter was not stepped.
    pub stance: stance_heading::StanceHeadingOutput,
    /// Stationary-foot hypotheses (`[left, right]`) after this frame's
    /// releases; all `swing` when both new layers are off or not live.
    pub contacts: [contact::FootContact; 2],
    /// Per-foot widening (m/s) of the contact speed gates used this frame,
    /// from the stance filter's yaw uncertainty and the headset speed.
    pub contact_speed_allowance_mps: [f32; 2],
    /// Torso-motion widening (m/s) of the contact speed and vertical-speed
    /// gates granted this frame (zero without the double-support cue).
    pub contact_torso_allowance_mps: f32,
    /// Both feet were in contact and not moving relative to each other.
    pub double_support: bool,
    /// Leg IK produced the published leg/torso positions in this frame.
    pub leg_ik_active: bool,
    /// Leg IK diagnostics; `status` is `disabled` when it did not run.
    pub leg_ik: leg_ik::LegIkDiagnostics,
    /// Support-foot floor correction applied before contacts and leg IK;
    /// `active` is false when leg IK did not run.
    pub support_floor: leg_ik::SupportFloorDiagnostics,
}

impl Default for EstimatorStatus {
    fn default() -> Self {
        Self {
            prediction_active: false,
            hmd_extrapolated_secs: 0.0,
            standing_space_rebases: 0,
            last_rebase_degrees: 0.0,
            tracking_discontinuities: 0,
            last_rebase_evidence: None,
            headset_resumptions: 0,
            last_headset_gap_secs: 0.0,
            stance_active: false,
            stance: stance_heading::StanceHeadingOutput::default(),
            contacts: [contact::FootContact::default(); 2],
            contact_speed_allowance_mps: [0.0; 2],
            contact_torso_allowance_mps: 0.0,
            double_support: false,
            leg_ik_active: false,
            leg_ik: idle_leg_ik_diagnostics(),
            support_floor: leg_ik::SupportFloorDiagnostics::default(),
        }
    }
}

fn idle_leg_ik_diagnostics() -> leg_ik::LegIkDiagnostics {
    leg_ik::LegIkDiagnostics {
        foot_locked: [false; 2],
        lock_error_m: [0.0; 2],
        release_requests: [false; 2],
        root_correction: [0.0; 3],
        flat_weight: [0.0; 2],
        lock_break_m: 0.0,
        root_max_m: 0.0,
        status: leg_ik::LegIkStatus::Disabled,
    }
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            anthropometrics: Anthropometrics::default(),
            stale_after_secs: 0.25,
            stable_window_secs: 0.65,
            stable_angle_degrees: 2.0,
            stable_acceleration_mps2: 1.5,
            neutral_tilt_degrees: 18.0,
            standing_height_tolerance_m: 0.20,
            recovery_max_yaw_degrees: 20.0,
            recovery_timeout_secs: 8.0,
            enable_joint_yaw_correction: false,
            enable_preview_motion: false,
            joint_yaw_deadband_degrees: 6.0,
            joint_yaw_max_residual_degrees: 25.0,
            yaw_correction_rate_degrees_per_sec: 0.20,
            estimator: EstimatorConfig::default(),
        }
    }
}

pub type Config = TrackingConfig;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MountingCalibration {
    pub sensor_id: String,
    pub role: Role,
    pub sensor_to_bone_xyzw: [f32; 4],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Profile {
    pub schema_version: u32,
    pub assignments: Vec<RoleAssignment>,
    pub anthropometrics: Anthropometrics,
    pub mounting: Vec<MountingCalibration>,
    /// Roles whose stored mounting was factored by functional calibration.
    /// This is persisted data, not permission to reuse session yaw after restart.
    #[serde(default)]
    pub direction_calibrated_roles: Vec<Role>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetRole {
    Chest,
    Waist,
    LeftKnee,
    RightKnee,
    LeftFoot,
    RightFoot,
    LeftElbow,
    RightElbow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Evidence {
    AbsoluteAnchor,
    MeasuredOrientation,
    FirmwareHeldOrientation,
    Inferred,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TargetPose {
    pub role: TargetRole,
    pub position: [f32; 3],
    pub orientation_xyzw: [f32; 4],
    pub position_source: Evidence,
    pub orientation_source: Evidence,
    pub confidence: f32,
    pub valid: bool,
    pub reason: Option<String>,
    /// Standing-space linear velocity (m/s): a smoothed finite difference of
    /// the published position. Zero for invalid targets and non-live frames.
    #[serde(default)]
    pub linear_velocity: [f32; 3],
    /// Standing-space axis-angle angular velocity (rad/s) of the owning
    /// measured segment. Zero for invalid targets and stale or held input.
    #[serde(default)]
    pub angular_velocity: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointRole {
    Head,
    Neck,
    Chest,
    Hip,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    LeftToe,
    RightToe,
    LeftShoulder,
    RightShoulder,
    LeftElbow,
    RightElbow,
    LeftHand,
    RightHand,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointPose {
    pub joint: JointRole,
    pub position: [f32; 3],
    pub confidence: f32,
    pub valid: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Bone {
    pub start: JointRole,
    pub end: JointRole,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SegmentPose {
    pub role: Role,
    pub orientation_xyzw: [f32; 4],
    pub yaw_correction_degrees: f32,
    pub confidence: f32,
    pub valid: bool,
    pub measurement_received_at_secs: Option<f64>,
    pub measurement_age_secs: Option<f64>,
    #[serde(default)]
    pub uses_firmware_hold: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum RecoveryState {
    Idle,
    Pending {
        requested_at_secs: f64,
        blocked_reason: Option<String>,
    },
    Applied {
        at_secs: f64,
    },
    Rejected {
        at_secs: f64,
        reason: String,
    },
    Undone,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CorrectionDiagnostic {
    pub thigh_role: Role,
    pub calf_role: Role,
    pub status: String,
    pub residual_degrees: f32,
    pub applied_relative_degrees: f32,
    pub confidence: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CalibrationStatus {
    pub calibrated: bool,
    pub preview_only: bool,
    pub requires_session_calibration: bool,
    pub reason: Option<String>,
    /// The stable pose was accepted partly under the named firmware continuity
    /// assumption, rather than a dense window of newly measured orientations.
    #[serde(default)]
    pub used_firmware_hold: bool,
    #[serde(default)]
    pub direction_calibrated: bool,
    #[serde(default)]
    pub reference_forward: Option<[f32; 3]>,
    /// Diagnostic only, from the last successful live neutral calibration:
    /// averaged HMD height over the stable window minus
    /// `Anthropometrics::standing_chain_height_m()`. Positive means the
    /// configured body is shorter than the headset height implies (feet float
    /// above the floor). `None` in preview or before a live neutral capture.
    #[serde(default)]
    pub floor_mismatch_m: Option<f32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrackingFrame {
    pub at_secs: f64,
    pub targets: Vec<TargetPose>,
    pub joints: Vec<JointPose>,
    pub bones: Vec<Bone>,
    pub segments: Vec<SegmentPose>,
    pub calibration: CalibrationStatus,
    pub recovery: RecoveryState,
    pub corrections: Vec<CorrectionDiagnostic>,
    pub live_anchor: bool,
    pub preview_only: bool,
    #[serde(default)]
    pub preview_motion: preview_motion::PreviewMotionStatus,
    #[serde(default)]
    pub estimator: EstimatorStatus,
    /// Progress of an active movement calibration session (experimental),
    /// refreshed at most every 0.1 s; `None` when no session is active.
    #[serde(default)]
    pub motion_calibration: Option<motion_calibration::MotionCalibrationStatus>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CoreError(pub String);
impl fmt::Display for CoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}
impl std::error::Error for CoreError {}

// Implementation follows below; public structures are intentionally independent
// of glam so the native UI, capture format and bridge share one stable contract.

#[derive(Clone)]
struct SampleState {
    latest: SensorSample,
    history: VecDeque<SensorSample>,
    hold: Option<FirmwareHoldState>,
    hold_revoked: bool,
    sensor_unhealthy: bool,
    last_evidence_at_secs: f64,
    hold_epoch_floor_secs: f64,
    /// Ingest order of the last accepted evidence, for cache eviction.
    last_seen: u64,
    /// Fed only by accepted measured quaternions, never by held tokens.
    rate: prediction::AngularRateEstimator,
}

#[derive(Clone)]
struct FirmwareHoldState {
    observed_at_secs: f64,
    last_transport_at_secs: f64,
    continuity_since_secs: f64,
}

#[derive(Clone)]
struct NeutralReference {
    captured_at_secs: f64,
    heading_radians: f32,
    orientations: BTreeMap<Role, (String, Quat)>,
    /// Live captures only: the neck pivot of the HMD pose averaged over the
    /// stable window, for the movement calibration's displacement forward.
    /// Dropped when the standing space is rebased or jumps.
    neck_at_neutral: Option<Vec3>,
}

/// An active movement calibration (see [`motion_calibration`]). It consumes
/// only measured core samples and, when `live`, usable HMD samples.
struct MotionSession {
    calibrator: motion_calibration::MotionCalibrator,
    live: bool,
    /// Last status published in a frame, with its solve time.
    published: Option<(f64, motion_calibration::MotionCalibrationStatus)>,
}

/// Frames carry a movement-calibration status at most this often.
const MOTION_STATUS_INTERVAL_SECS: f64 = 0.1;
/// Extra stance-filter heading uncertainty (1σ, degrees) after a movement
/// calibration: knees-out or toe-out squats bias legs by 5–16° in synthetic
/// tests, which only the headset-referenced stance filter can remove.
const MOTION_LEG_SIGMA_DEGREES: f32 = 10.0;
const MOTION_TORSO_SIGMA_DEGREES: f32 = 6.0;

const FIRMWARE_HOLD_LEASE_SECS: f64 = 1.5;
const FIRMWARE_HOLD_CONFIDENCE_CAP: f32 = 0.45;
const MAX_FIRMWARE_NAME_LEN: usize = 64;
const MAX_CACHED_SENSORS: usize = 256;
/// Evidence stamped further ahead of the newest time the core has seen is
/// rejected without changing state, so one bad clock cannot poison an id.
const MAX_FUTURE_EVIDENCE_SECS: f64 = 2.0;
const KNEE_CORRECTION_BUDGET_DEGREES: f32 = 20.0;
/// Anchor positions are advanced by at most this much along their velocity.
const MAX_ANCHOR_EXTRAPOLATION_SECS: f64 = 0.03;
/// Reported anchor speeds at or above this are not trusted for extrapolation.
const MAX_ANCHOR_EXTRAPOLATION_SPEED: f32 = 10.0;
/// Extra stance-filter heading uncertainty (1σ, degrees) when usable headset
/// samples resume after a long gap (SteamVR restarted, headset idle): the
/// calibration is kept on the assumption that the standing space of the same
/// headset in the same room is the same physical frame; if it was not, the
/// filter can re-converge quickly.
const RESUME_LEG_SIGMA_DEGREES: f32 = 15.0;
const RESUME_TORSO_SIGMA_DEGREES: f32 = 8.0;
const TARGET_SLOTS: usize = 8;

#[derive(Clone, Copy)]
struct Segment {
    rotation: Quat,
    confidence: f32,
    valid: bool,
    held: bool,
    /// Standing-space angular velocity (rad/s); zero unless freshly measured.
    angular_velocity: Vec3,
    /// The orientation was advanced along a nonzero measured angular rate.
    extrapolated: bool,
}

impl Default for Segment {
    fn default() -> Self {
        Self {
            rotation: Quat::IDENTITY,
            confidence: 0.0,
            valid: false,
            held: false,
            angular_velocity: Vec3::ZERO,
            extrapolated: false,
        }
    }
}

fn target_slot(role: TargetRole) -> usize {
    match role {
        TargetRole::Chest => 0,
        TargetRole::Waist => 1,
        TargetRole::LeftKnee => 2,
        TargetRole::RightKnee => 3,
        TargetRole::LeftFoot => 4,
        TargetRole::RightFoot => 5,
        TargetRole::LeftElbow => 6,
        TargetRole::RightElbow => 7,
    }
}

/// The last usable HMD sample seen by the discontinuity detector.
#[derive(Clone, Copy)]
struct HmdSample {
    at_secs: f64,
    position: Vec3,
    rotation: Quat,
}

pub struct TrackingCore {
    config: TrackingConfig,
    assignments: BTreeMap<Role, String>,
    samples: BTreeMap<String, SampleState>,
    mounting: BTreeMap<Role, Quat>,
    /// Initial sensor-world heading alignment, never included in dynamic drift.
    heading_alignment: BTreeMap<Role, f32>,
    yaw: BTreeMap<Role, f32>,
    functional_roles: BTreeSet<Role>,
    direction_session_valid: bool,
    reference_heading: Option<f32>,
    neutral_reference: Option<NeutralReference>,
    manual_mounts: BTreeMap<Role, manual_mounting::ManualMountState>,
    calibrated: bool,
    preview_calibration: bool,
    restored_profile: bool,
    head_history: VecDeque<AnchorPose>,
    recovery: RecoveryState,
    /// Per-role yaw deltas applied by the last recovery. Undo subtracts them,
    /// so later corrections survive.
    recovery_undo: Option<BTreeMap<Role, f32>>,
    /// A recovery changed headings since the neutral capture, so the stored
    /// neutral heading no longer matches the skiing-pose reference.
    forward_requires_new_neutral: bool,
    last_solve_secs: Option<f64>,
    newest_evidence_secs: Option<f64>,
    ingest_sequence: u64,
    knee_history: BTreeMap<Role, VecDeque<(f64, f32)>>,
    /// Signed relative knee correction accumulated per pair (keyed by thigh),
    /// excluding recovery offsets. The correction budget applies to this.
    knee_correction: BTreeMap<Role, f32>,
    calibration_reason: Option<String>,
    calibration_used_firmware_hold: bool,
    floor_mismatch_m: Option<f32>,
    preview_motion: preview_motion::PreviewMotionEstimator,
    /// One per published target slot, fed with final published positions.
    target_velocity: [prediction::LinearVelocityEstimator; TARGET_SLOTS],
    previous_hmd: Option<HmdSample>,
    standing_space_rebases: u32,
    last_rebase_degrees: f32,
    tracking_discontinuities: u32,
    last_rebase_evidence: Option<tracking_space::RebaseEvidence>,
    /// A possible tracking-space event reported by the application, consumed
    /// by the discontinuity detector.
    tracking_space_hint: Option<tracking_space::TrackingSpaceHint>,
    headset_resumptions: u32,
    last_headset_gap_secs: f32,
    /// Stationary-foot hypotheses shared by the stance filter and leg IK.
    contact: contact::ContactTracker,
    /// Owns the per-`Role::CORE` stance yaw correction, applied as an extra
    /// left world yaw: alignment + dynamic `yaw` + stance correction.
    stance: stance_heading::StanceHeadingFilter,
    /// Vertical support-foot floor correction, part of the leg IK layer but
    /// applied before contacts so they and the anchors see the corrected ankles.
    support_floor: leg_ik::SupportFloor,
    leg_ik: leg_ik::LegIkSolver,
    /// Smoothed headset velocity from its positions, for the contact speed
    /// allowance when the anchor reports no velocity.
    head_velocity: prediction::LinearVelocityEstimator,
    /// Experimental movement calibration in progress, if any.
    motion: Option<MotionSession>,
}

/// One usable raw HMD sample, for the headset velocity.
#[derive(Clone, Copy)]
struct HeadSample {
    at_secs: f64,
    position: Vec3,
    /// Finite and below the extrapolation speed limit.
    reported_velocity: Option<Vec3>,
}

/// Headset velocity smoothing for the contact speed allowance.
const HEAD_VELOCITY_TAU_SECS: f64 = 0.1;
/// Extra 1σ heading uncertainty (degrees) for a core segment whose sensor lost
/// its firmware-hold continuity or reported a sensor error.
const STANCE_FAULT_SIGMA_DEGREES: f32 = 10.0;

fn head_velocity_estimator() -> prediction::LinearVelocityEstimator {
    prediction::LinearVelocityEstimator::new(prediction::LinearVelocityConfig {
        smoothing_tau_secs: HEAD_VELOCITY_TAU_SECS,
        ..Default::default()
    })
}

/// What the stance layers contributed to one solve.
struct StanceLayers {
    stance_active: bool,
    ik: Option<leg_ik::LegIkOutput>,
    support_floor: leg_ik::SupportFloorDiagnostics,
}

/// What a neutral-gate evaluation is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GatePurpose {
    /// Neutral capture: the configured eye height is enforced.
    Calibration,
    /// Heading recovery: also checks each core segment against the headset yaw.
    Recovery,
    /// Desktop-to-headset alignment: stability and tilt only; the configured
    /// eye height is not enforced (changing it would void the calibration).
    Alignment,
}

struct NeutralGate {
    used_firmware_hold: bool,
    /// Roles whose stability, tilt and (in recovery) heading passed the gate.
    validated: BTreeSet<Role>,
}

impl TrackingCore {
    pub fn new(config: TrackingConfig) -> Result<Self, CoreError> {
        config.validate()?;
        Ok(Self {
            contact: contact::ContactTracker::new(config.estimator.contact),
            stance: stance_heading::StanceHeadingFilter::new(config.estimator.stance),
            support_floor: leg_ik::SupportFloor::new(config.estimator.leg_ik),
            leg_ik: leg_ik::LegIkSolver::new(config.estimator.leg_ik),
            head_velocity: head_velocity_estimator(),
            config,
            assignments: BTreeMap::new(),
            samples: BTreeMap::new(),
            mounting: BTreeMap::new(),
            heading_alignment: BTreeMap::new(),
            yaw: BTreeMap::new(),
            functional_roles: BTreeSet::new(),
            direction_session_valid: false,
            reference_heading: None,
            neutral_reference: None,
            manual_mounts: BTreeMap::new(),
            calibrated: false,
            preview_calibration: false,
            restored_profile: false,
            head_history: VecDeque::new(),
            recovery: RecoveryState::Idle,
            recovery_undo: None,
            forward_requires_new_neutral: false,
            last_solve_secs: None,
            newest_evidence_secs: None,
            ingest_sequence: 0,
            knee_history: BTreeMap::new(),
            knee_correction: BTreeMap::new(),
            calibration_reason: Some("Neutral session calibration is required".into()),
            calibration_used_firmware_hold: false,
            floor_mismatch_m: None,
            preview_motion: preview_motion::PreviewMotionEstimator::default(),
            target_velocity: Default::default(),
            previous_hmd: None,
            standing_space_rebases: 0,
            last_rebase_degrees: 0.0,
            tracking_discontinuities: 0,
            last_rebase_evidence: None,
            tracking_space_hint: None,
            headset_resumptions: 0,
            last_headset_gap_secs: 0.0,
            motion: None,
        })
    }

    fn reset_target_velocities(&mut self) {
        for estimator in &mut self.target_velocity {
            estimator.reset();
        }
    }

    /// The previous frame is no longer comparable with the next one (rebase,
    /// tracking discontinuity, recovery undo, not live): forget contacts, leg
    /// IK anchors/offsets and the stance filter's motion history. The stance
    /// corrections and their covariance stay valid.
    pub(crate) fn interrupt_stance_layers(&mut self) {
        self.contact.reset();
        self.support_floor.reset();
        self.leg_ik.reset();
        self.head_velocity.reset();
        self.stance.invalidate_history();
    }

    /// Calibration, assignment or mounting changed what the corrections were
    /// estimated against: additionally zero the stance corrections and return
    /// the filter covariance to its initial value.
    pub(crate) fn reset_stance_layers(&mut self) {
        self.contact.reset();
        self.support_floor.reset();
        self.leg_ik.reset();
        self.head_velocity.reset();
        self.stance.reset();
    }

    /// The sensor `sensor_id` lost continuity (firmware hold revoked) or
    /// reported an error: its core segment's heading becomes less certain, so
    /// the filter may move it faster once it is measured again. Applied once
    /// per outage; corrections are unchanged.
    fn widen_stance_heading(&mut self, sensor_id: &str) {
        if !self.config.estimator.enable_stance_heading {
            return;
        }
        let mut sigma = [0.0; 6];
        for (i, role) in Role::CORE.into_iter().enumerate() {
            if self
                .assignments
                .get(&role)
                .is_some_and(|id| id == sensor_id)
            {
                sigma[i] = STANCE_FAULT_SIGMA_DEGREES;
            }
        }
        if sigma.iter().any(|s| *s > 0.0) {
            self.stance.add_uncertainty(sigma);
        }
    }

    /// Stance correction applied to `role` (radians): zero for roles outside
    /// `Role::CORE` and while the filter is disabled.
    fn stance_correction(&self, role: Role) -> f32 {
        if !self.config.estimator.enable_stance_heading {
            return 0.0;
        }
        Role::CORE
            .iter()
            .position(|core| *core == role)
            .map_or(0.0, |i| self.stance.corrections()[i])
    }

    /// The stance corrections that `segment_at` applies right now, per
    /// `Role::CORE` index.
    fn applied_stance_corrections(&self) -> [f32; 6] {
        Role::CORE.map(|role| self.stance_correction(role))
    }

    pub fn config(&self) -> &TrackingConfig {
        &self.config
    }

    pub fn direction_calibrated(&self) -> bool {
        self.calibrated
            && self.direction_session_valid
            && Role::CORE
                .iter()
                .all(|role| self.functional_roles.contains(role))
    }

    pub fn calibration_forward(&self) -> Option<[f32; 3]> {
        self.reference_heading
            .filter(|_| self.calibrated)
            .map(|yaw| (Quat::from_rotation_y(yaw) * Vec3::NEG_Z).to_array())
    }

    pub fn update_config(&mut self, config: TrackingConfig) -> Result<(), CoreError> {
        self.set_config(config)
    }

    pub fn set_config(&mut self, config: TrackingConfig) -> Result<(), CoreError> {
        config.validate()?;
        if self.config.enable_preview_motion != config.enable_preview_motion
            || self.config.anthropometrics != config.anthropometrics
        {
            self.preview_motion.reset();
        }
        if self.config.anthropometrics != config.anthropometrics {
            self.calibrated = false;
            self.neutral_reference = None;
            self.motion = None;
            self.manual_mounts.clear();
            self.floor_mismatch_m = None;
            self.calibration_reason = Some("Body dimensions changed; calibrate again".into());
        }
        if self.config.estimator.prediction != config.estimator.prediction {
            // Rates restart from the next measured samples under the new settings.
            for state in self.samples.values_mut() {
                state.rate = prediction::AngularRateEstimator::new(config.estimator.prediction);
            }
        }
        if self.config.estimator.enable_auto_rebase != config.estimator.enable_auto_rebase {
            self.previous_hmd = None;
        }
        let (old, new) = (&self.config.estimator, &config.estimator);
        if old.contact != new.contact {
            self.contact = contact::ContactTracker::new(new.contact);
        }
        if old.stance != new.stance {
            // New tuning: keep the corrections, restart covariance and history.
            let kept = self.stance.corrections();
            self.stance = stance_heading::StanceHeadingFilter::new(new.stance);
            self.stance.set_corrections(kept);
        }
        if old.leg_ik != new.leg_ik {
            self.leg_ik.set_config(new.leg_ik);
            self.support_floor.set_config(new.leg_ik);
        }
        // Switching a layer off ablates it completely: no stale correction or
        // anchor is applied later, and switching it on again starts from zero.
        if !new.enable_stance_heading {
            self.stance.reset();
        }
        if !new.enable_leg_ik {
            self.leg_ik.reset();
            self.support_floor.reset();
        }
        if !new.enable_stance_heading && !new.enable_leg_ik {
            self.contact.reset();
        }
        if self.config.anthropometrics != config.anthropometrics {
            self.reset_stance_layers();
        }
        self.config = config;
        Ok(())
    }

    pub fn set_assignments(&mut self, assignments: Vec<RoleAssignment>) -> Result<(), CoreError> {
        let next = validated_assignments(&assignments)?;
        let core_changed = Role::CORE
            .iter()
            .any(|r| self.assignments.get(r) != next.get(r));
        self.mounting
            .retain(|role, _| self.assignments.get(role) == next.get(role));
        self.yaw
            .retain(|role, _| self.assignments.get(role) == next.get(role));
        self.heading_alignment
            .retain(|role, _| self.assignments.get(role) == next.get(role));
        self.functional_roles
            .retain(|role| self.assignments.get(role) == next.get(role));
        if core_changed {
            self.calibrated = false;
            self.direction_session_valid = false;
            self.floor_mismatch_m = None;
            self.calibration_reason =
                Some("Core sensor assignments changed; calibrate again".into());
        }
        if self.assignments != next {
            self.preview_motion.reset();
            self.neutral_reference = None;
            self.motion = None;
            self.manual_mounts.clear();
            self.recovery = RecoveryState::Idle;
            self.recovery_undo = None;
            self.knee_history.clear();
            self.knee_correction.clear();
            if core_changed {
                self.reset_stance_layers();
            } else {
                self.interrupt_stance_layers();
            }
        }
        self.assignments = next;
        Ok(())
    }

    /// Ignores packet ordering only by rejecting older packets. Identity never
    /// depends on discovery order or packet order.
    pub fn ingest(&mut self, mut sample: SensorSample) -> Result<(), CoreError> {
        validate_id(&sample.id)?;
        let q = checked_quat(sample.orientation_xyzw)?;
        if !sample.received_at_secs.is_finite() || sample.received_at_secs < 0.0 {
            return Err(CoreError("Invalid sensor timestamp".into()));
        }
        if sample.acceleration_mps2.is_some_and(|a| !finite3(a)) {
            return Err(CoreError("Nonfinite acceleration".into()));
        }
        self.reject_future_evidence(&sample)?;
        sample.orientation_xyzw = q.to_array();
        if matches!(sample.provenance, SensorProvenance::FirmwareHeld { .. }) {
            return self.ingest_firmware_hold(sample);
        }
        let sequence = self.ingest_sequence + 1;
        let received_at_secs = sample.received_at_secs;
        // Only measured quaternions of core-assigned sensors reach a movement
        // calibration; firmware-held tokens returned above never do.
        let motion_index = self.motion.as_ref().and_then(|_| {
            Role::CORE
                .iter()
                .position(|role| self.assignments.get(role) == Some(&sample.id))
        });
        if let Some(state) = self.samples.get_mut(&sample.id) {
            if sample.received_at_secs < state.last_evidence_at_secs {
                state.hold = None;
                state.hold_revoked = true;
                return Err(CoreError("Out-of-order sensor sample".into()));
            }
            if state.hold_revoked
                || sample.received_at_secs - state.last_evidence_at_secs >= FIRMWARE_HOLD_LEASE_SECS
                || state.hold.as_ref().is_some_and(|hold| {
                    sample.received_at_secs - hold.last_transport_at_secs
                        >= FIRMWARE_HOLD_LEASE_SECS
                })
            {
                state.hold_epoch_floor_secs = sample.received_at_secs;
            }
            state.hold_revoked = false;
            state.sensor_unhealthy = false;
            state.hold = None;
            state.last_evidence_at_secs = sample.received_at_secs;
            state.last_seen = sequence;
            if sample.received_at_secs > state.latest.received_at_secs {
                state.history.push_back(sample.clone());
                state.rate.observe(sample.received_at_secs, q);
            } else {
                if let Some(last) = state.history.back_mut() {
                    *last = sample.clone();
                }
                // A same-time replacement restarts the rate so that any
                // extrapolation starts from the stored latest orientation.
                state.rate.reset();
                state.rate.observe(sample.received_at_secs, q);
            }
            state.latest = sample;
            let keep_after =
                state.latest.received_at_secs - self.config.stable_window_secs.max(2.0) - 0.1;
            while state
                .history
                .front()
                .is_some_and(|s| s.received_at_secs < keep_after)
                || state.history.len() > 1024
            {
                state.history.pop_front();
            }
        } else {
            if self.samples.len() >= MAX_CACHED_SENSORS {
                self.evict_least_recent_unassigned()?;
            }
            let mut rate = prediction::AngularRateEstimator::new(self.config.estimator.prediction);
            rate.observe(sample.received_at_secs, q);
            self.samples.insert(
                sample.id.clone(),
                SampleState {
                    latest: sample.clone(),
                    history: VecDeque::from([sample.clone()]),
                    hold: None,
                    hold_revoked: false,
                    sensor_unhealthy: false,
                    last_evidence_at_secs: sample.received_at_secs,
                    hold_epoch_floor_secs: sample.received_at_secs,
                    last_seen: sequence,
                    rate,
                },
            );
        }
        self.ingest_sequence = sequence;
        self.note_evidence(received_at_secs);
        if let (Some(index), Some(session)) = (motion_index, self.motion.as_mut()) {
            session
                .calibrator
                .observe_sensor(index, received_at_secs, q);
        }
        Ok(())
    }

    /// Drops all cached evidence for one sensor identity, for example after
    /// the transport saw it disconnect or reboot. An assigned role whose
    /// sensor is forgotten becomes invalid until new measurements arrive.
    pub fn forget_sensor(&mut self, sensor_id: &str) {
        self.samples.remove(sensor_id);
    }

    fn evict_least_recent_unassigned(&mut self) -> Result<(), CoreError> {
        let assigned: BTreeSet<&String> = self.assignments.values().collect();
        let victim = self
            .samples
            .iter()
            .filter(|(id, _)| !assigned.contains(id))
            .min_by_key(|(_, state)| state.last_seen)
            .map(|(id, _)| id.clone())
            .ok_or_else(|| CoreError("Sensor cache limit reached".into()))?;
        self.samples.remove(&victim);
        Ok(())
    }

    fn note_evidence(&mut self, at_secs: f64) {
        if at_secs.is_finite() {
            self.newest_evidence_secs = Some(
                self.newest_evidence_secs
                    .map_or(at_secs, |newest| newest.max(at_secs)),
            );
        }
    }

    /// Rejects, without mutating any state, evidence stamped more than
    /// `MAX_FUTURE_EVIDENCE_SECS` ahead of the newest time this core has seen.
    fn reject_future_evidence(&self, sample: &SensorSample) -> Result<(), CoreError> {
        let Some(horizon) = self
            .last_solve_secs
            .into_iter()
            .chain(self.newest_evidence_secs)
            .reduce(f64::max)
        else {
            return Ok(());
        };
        let limit = horizon + MAX_FUTURE_EVIDENCE_SECS;
        let mut times = vec![sample.received_at_secs];
        if let SensorProvenance::FirmwareHeld {
            observed_at_secs,
            last_transport_at_secs,
            continuity_since_secs,
            ..
        } = &sample.provenance
        {
            times.extend([
                *observed_at_secs,
                *last_transport_at_secs,
                *continuity_since_secs,
            ]);
        }
        if times.iter().any(|time| time.is_finite() && *time > limit) {
            return Err(CoreError(format!(
                "Sensor timestamp is more than {MAX_FUTURE_EVIDENCE_SECS} s ahead of the tracking clock"
            )));
        }
        Ok(())
    }

    /// Immediately removes a transport-derived holding assumption. Once revoked,
    /// only a newly received quaternion packet can re-enable holding. A firmware
    /// session change must additionally clear session calibration in the runtime.
    pub fn revoke_firmware_hold(&mut self, sensor_id: &str) {
        if let Some(state) = self.samples.get_mut(sensor_id) {
            let newly_revoked = !state.hold_revoked;
            state.hold = None;
            state.hold_revoked = true;
            // Transport revocations follow gaps or discontinuities: the next
            // measured quaternions must not be differenced across them.
            state.rate.reset();
            if newly_revoked {
                self.widen_stance_heading(sensor_id);
            }
        }
    }

    /// A reported sensor error invalidates even a recent measured quaternion.
    /// Transport must withhold new measurements while status remains non-OK.
    pub fn mark_sensor_unhealthy(&mut self, sensor_id: &str) {
        if let Some(state) = self.samples.get_mut(sensor_id) {
            let newly_unhealthy = !state.sensor_unhealthy;
            state.hold = None;
            state.hold_revoked = true;
            state.sensor_unhealthy = true;
            state.rate.reset();
            if newly_unhealthy {
                self.widen_stance_heading(sensor_id);
            }
        }
    }

    fn ingest_firmware_hold(&mut self, sample: SensorSample) -> Result<(), CoreError> {
        let SensorProvenance::FirmwareHeld {
            firmware,
            observed_at_secs,
            last_transport_at_secs,
            continuity_since_secs,
            sensor_status_healthy,
            sequence_contiguous,
        } = sample.provenance
        else {
            unreachable!()
        };
        let sequence = self.ingest_sequence + 1;
        let state = self.samples.get_mut(&sample.id).ok_or_else(|| {
            CoreError("Held evidence requires a previously measured quaternion".into())
        })?;

        // Explicit discontinuity/health observations are valid events to record
        // and replay, but never authorize a held orientation.
        if !sensor_status_healthy || !sequence_contiguous {
            let new_fault =
                !state.hold_revoked || (!sensor_status_healthy && !state.sensor_unhealthy);
            state.hold = None;
            state.hold_revoked = true;
            state.sensor_unhealthy |= !sensor_status_healthy;
            state.rate.reset();
            if observed_at_secs.is_finite() && observed_at_secs >= state.last_evidence_at_secs {
                state.last_evidence_at_secs = observed_at_secs;
                state.last_seen = sequence;
                self.ingest_sequence = sequence;
                self.note_evidence(observed_at_secs);
            }
            if new_fault {
                self.widen_stance_heading(&sample.id);
            }
            return Ok(());
        }
        let same_rotation =
            quaternion_components_match(sample.orientation_xyzw, state.latest.orientation_xyzw);
        let time_valid = [
            observed_at_secs,
            last_transport_at_secs,
            continuity_since_secs,
        ]
        .iter()
        .all(|v| v.is_finite() && *v >= 0.0)
            && observed_at_secs >= state.last_evidence_at_secs
            && observed_at_secs >= last_transport_at_secs
            && last_transport_at_secs >= sample.received_at_secs
            && continuity_since_secs <= sample.received_at_secs
            && continuity_since_secs >= state.hold_epoch_floor_secs
            && observed_at_secs - last_transport_at_secs < FIRMWARE_HOLD_LEASE_SECS
            && observed_at_secs - state.last_evidence_at_secs < FIRMWARE_HOLD_LEASE_SECS
            && state.hold.as_ref().is_none_or(|previous| {
                last_transport_at_secs - previous.last_transport_at_secs < FIRMWARE_HOLD_LEASE_SECS
            });
        // The transport sees the handshake, status and sequence continuity and
        // vouches for unchanged-rotation suppression; the core only requires
        // that it names a firmware build. Stock, DIY "main" and "git-<rev>"
        // builds suppress identically.
        let firmware_named = !firmware.trim().is_empty() && firmware.len() <= MAX_FIRMWARE_NAME_LEN;
        let eligible = firmware_named
            && !state.hold_revoked
            && !state.sensor_unhealthy
            && time_valid
            && same_rotation
            && sample.received_at_secs == state.latest.received_at_secs
            && sample.acceleration_mps2.is_none();
        if !eligible {
            state.hold = None;
            state.hold_revoked = true;
            return Err(CoreError("Held orientation rejected: needs a named firmware build, continuous healthy transport, matching measured quaternion and a fresh evidence lease; receive a new quaternion before retrying".into()));
        }
        state.last_evidence_at_secs = observed_at_secs;
        state.last_seen = sequence;
        state.hold = Some(FirmwareHoldState {
            observed_at_secs,
            last_transport_at_secs,
            continuity_since_secs,
        });
        // In particular, do not update state.latest, its received timestamp,
        // its acceleration, or measured history from this compatibility token.
        self.ingest_sequence = sequence;
        self.note_evidence(observed_at_secs);
        Ok(())
    }

    pub fn clear_calibration(&mut self) {
        self.preview_motion.reset();
        self.reset_target_velocities();
        self.reset_stance_layers();
        self.mounting.clear();
        self.heading_alignment.clear();
        self.yaw.clear();
        self.functional_roles.clear();
        self.direction_session_valid = false;
        self.reference_heading = None;
        self.neutral_reference = None;
        self.motion = None;
        self.manual_mounts.clear();
        self.calibrated = false;
        self.preview_calibration = false;
        self.restored_profile = false;
        self.knee_history.clear();
        self.knee_correction.clear();
        self.recovery_undo = None;
        self.forward_requires_new_neutral = false;
        self.recovery = RecoveryState::Idle;
        self.calibration_reason = Some("Neutral session calibration is required".into());
        self.calibration_used_firmware_hold = false;
        self.floor_mismatch_m = None;
    }

    /// Calling this asserts that the user is in the instructed neutral pose.
    /// Unknown mounting cannot independently verify first-time anatomical tilt.
    /// `preview` explicitly permits a synthetic standing anchor, but never valid
    /// live outputs; repeat calibration with a real HMD before publishing poses.
    pub fn calibrate_neutral(
        &mut self,
        now: f64,
        anchors: &Anchors,
        preview: bool,
    ) -> Result<(), CoreError> {
        // Desktop preview is an explicit synthetic reference, even if a real
        // headset happens to be connected or resting elsewhere in the room.
        let preview_anchors = Anchors::default();
        let anchors = if preview { &preview_anchors } else { anchors };
        self.observe_head(anchors, now);
        let mut used_firmware_hold =
            match self.neutral_gate(now, anchors, preview, GatePurpose::Calibration) {
                Ok(gate) => gate.used_firmware_hold,
                Err(error) => {
                    self.calibration_reason = Some(error.0.clone());
                    return Err(error);
                }
            };
        let heading = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs)
            .map(|a| yaw_of(checked_quat(a.orientation_xyzw).unwrap()))
            .unwrap_or(0.0);
        // Diagnostic only: the gate already proved a stable live HMD window.
        let head_window: Vec<&AnchorPose> = if preview {
            Vec::new()
        } else {
            self.head_history
                .iter()
                .filter(|h| {
                    h.received_at_secs >= now - self.config.stable_window_secs
                        && h.received_at_secs <= now
                })
                .collect()
        };
        let floor_mismatch_m = (!head_window.is_empty()).then(|| {
            head_window.iter().map(|h| h.position[1]).sum::<f32>() / head_window.len() as f32
                - self.config.anthropometrics.standing_chain_height_m()
        });
        // The movement calibration compares later neck positions with this.
        let neck_at_neutral = (!head_window.is_empty())
            .then(|| {
                let position = head_window
                    .iter()
                    .map(|h| Vec3::from_array(h.position))
                    .sum::<Vec3>()
                    / head_window.len() as f32;
                let rotation = average_quaternions(
                    head_window
                        .iter()
                        .map(|h| Quat::from_array(h.orientation_xyzw)),
                );
                let body = &self.config.anthropometrics;
                motion_calibration::MotionCalibrator::neck_pivot(
                    position,
                    rotation,
                    body.head_to_neck_m,
                    body.eye_forward_m,
                )
            })
            .flatten();
        let reference = Quat::from_rotation_y(heading);
        let preserve_functional_mounts = self.direction_session_valid
            && Role::CORE.iter().all(|role| {
                self.functional_roles.contains(role) && self.mounting.contains_key(role)
            });
        let mut mounting = BTreeMap::new();
        let mut heading_alignment = BTreeMap::new();
        let mut neutral_orientations = BTreeMap::new();
        for (role, id) in &self.assignments {
            let Some(state) = self.samples.get(id) else {
                continue;
            };
            if !Role::CORE.contains(role) {
                // Optional unsolved roles must not block the six-sensor setup.
                // Capture them for the manual editor only with valid stability.
                match self.stable_sensor(*role, state, now) {
                    Ok(used_hold) => used_firmware_hold |= used_hold,
                    Err(_) => continue,
                }
            }
            let start = now - self.config.stable_window_secs;
            // Only actual packet observations participate in averaging. If a
            // held interval contains no new packets, use the original quaternion
            // directly; repeated compatibility tokens must not create samples.
            let average = if state.history.iter().any(|s| s.received_at_secs >= start) {
                average_quaternions(
                    state
                        .history
                        .iter()
                        .filter(|s| s.received_at_secs >= start)
                        .map(|s| Quat::from_array(s.orientation_xyzw)),
                )
            } else {
                Quat::from_array(state.latest.orientation_xyzw)
            };
            if preserve_functional_mounts && self.functional_roles.contains(role) {
                // Physical mounting is now identified: only world yaw is reset.
                // Absorbing heading into the right mount would rotate bend axes.
                let physical_mount = self.mounting[role];
                let raw_heading = yaw_of((average * physical_mount).normalize());
                mounting.insert(*role, physical_mount);
                heading_alignment.insert(*role, wrap_angle(heading - raw_heading));
            } else {
                // Provisional upright visualization; sagittal direction is still
                // unobservable until the instructed second pose is captured.
                mounting.insert(*role, (average.conjugate() * reference).normalize());
                heading_alignment.insert(*role, 0.0);
            }
            neutral_orientations.insert(*role, (id.clone(), average));
        }
        self.mounting = mounting;
        self.preview_motion.reset();
        self.reset_stance_layers();
        self.manual_mounts.clear();
        self.heading_alignment = heading_alignment;
        if !preserve_functional_mounts {
            self.functional_roles.clear();
        }
        self.direction_session_valid = preserve_functional_mounts;
        self.reference_heading = Some(heading);
        self.neutral_reference = Some(NeutralReference {
            captured_at_secs: now,
            heading_radians: heading,
            orientations: neutral_orientations,
            neck_at_neutral,
        });
        // A new neutral capture invalidates any movement evidence.
        self.motion = None;
        self.yaw = self.assignments.keys().map(|r| (*r, 0.0)).collect();
        self.calibrated = true;
        self.preview_calibration = preview;
        self.restored_profile = false;
        self.calibration_used_firmware_hold = used_firmware_hold;
        self.floor_mismatch_m = floor_mismatch_m;
        self.recovery = RecoveryState::Idle;
        self.recovery_undo = None;
        self.forward_requires_new_neutral = false;
        self.knee_history.clear();
        self.knee_correction.clear();
        self.calibration_reason = if preview {
            Some("Preview calibration cannot produce live tracker poses".into())
        } else if used_firmware_hold {
            Some("Calibration used a firmware-held orientation assumption; continuous fresh sensor measurements were not observed".into())
        } else if !self.direction_calibrated() {
            Some("Neutral pose captured; complete the skiing-pose forward calibration before live output".into())
        } else {
            None
        };
        Ok(())
    }

    /// Captures the explicitly instructed signed sagittal skiing pose. All six
    /// core roles must pass; no mounting, alignment or readiness changes on error.
    /// Current HMD pitch/height is irrelevant: the neutral heading is retained.
    pub fn calibrate_forward(&mut self, now: f64) -> Result<(), CoreError> {
        let result = self.forward_candidates(now);
        let (alignments, used_hold) = match result {
            Ok(candidate) => candidate,
            Err(error) => {
                self.calibration_reason = Some(error.0.clone());
                return Err(error);
            }
        };
        self.commit_forward_alignments(alignments, used_hold);
        Ok(())
    }

    /// Applies six solved forward alignments: mounting and initial heading
    /// alignment per core role, dynamic yaw zeroed, functional roles set, and
    /// every estimator layer that was estimated against the old alignment
    /// reset. Shared by the skiing-pose and movement calibrations.
    fn commit_forward_alignments(
        &mut self,
        alignments: BTreeMap<Role, forward_calibration::ForwardAlignment>,
        used_hold: bool,
    ) {
        for (role, alignment) in alignments {
            self.mounting
                .insert(role, Quat::from_array(alignment.mounting_xyzw));
            self.heading_alignment
                .insert(role, alignment.heading_alignment_radians);
            self.yaw.insert(role, 0.0);
        }
        self.functional_roles = Role::CORE.into_iter().collect();
        self.preview_motion.reset();
        self.reset_stance_layers();
        self.manual_mounts.clear();
        self.direction_session_valid = true;
        self.calibration_used_firmware_hold |= used_hold;
        self.recovery = RecoveryState::Idle;
        self.recovery_undo = None;
        self.knee_history.clear();
        self.knee_correction.clear();
        self.calibration_reason = if self.preview_calibration {
            Some("Desktop forward direction calibrated; live output remains disabled".into())
        } else if self.calibration_used_firmware_hold {
            Some("Forward direction calibrated using the explicit firmware-held stability assumption".into())
        } else {
            None
        };
    }

    /// The neutral reference a forward calibration may start from: the same
    /// preconditions as [`Self::calibrate_forward`] (a neutral capture for
    /// the current assignments, at most 300 s old, not followed by a heading
    /// recovery), without touching any state.
    fn movement_neutral(&self, now: f64) -> Result<&NeutralReference, CoreError> {
        if !self.calibrated {
            return Err(CoreError(
                "Capture a neutral pose before movement calibration".into(),
            ));
        }
        if self.forward_requires_new_neutral {
            return Err(CoreError(
                "Recovery changed headings since the neutral capture; capture neutral again before movement calibration".into(),
            ));
        }
        let neutral = self.neutral_reference.as_ref().ok_or_else(|| {
            CoreError("Capture a fresh neutral pose before movement calibration".into())
        })?;
        if !now.is_finite()
            || now < neutral.captured_at_secs
            || now - neutral.captured_at_secs > 300.0
        {
            return Err(CoreError(
                "Neutral reference expired or time changed; capture neutral again before movement calibration".into(),
            ));
        }
        for role in Role::CORE {
            let (id, _) = neutral
                .orientations
                .get(&role)
                .ok_or_else(|| CoreError(format!("Neutral reference is missing {role:?}")))?;
            if self.assignments.get(&role) != Some(id) {
                return Err(CoreError(
                    "Assignments changed; capture neutral again".into(),
                ));
            }
        }
        Ok(neutral)
    }

    /// Whether [`Self::begin_motion_calibration`] would accept `now`, without
    /// changing any state (for offering "start at the movement step").
    pub fn motion_calibration_available(&self, now: f64) -> Result<(), CoreError> {
        self.movement_neutral(now).map(|_| ())
    }

    /// Starts an EXPERIMENTAL movement calibration from the stored neutral
    /// capture (see [`motion_calibration`]). Afterwards every accepted measured
    /// sample of a core sensor, and in live mode every new usable HMD sample,
    /// is accumulated until [`Self::finish_motion_calibration`] or a cancel.
    /// Replaces any session in progress. Calling this asserts nothing about
    /// the user's pose; the gates judge only the recorded movement.
    pub fn begin_motion_calibration(&mut self, now: f64) -> Result<(), CoreError> {
        let neutral = self.movement_neutral(now)?;
        let orientations = Role::CORE.map(|role| neutral.orientations[&role].1);
        // Desktop preview has no headset reference and never compares necks.
        let neck = neutral
            .neck_at_neutral
            .filter(|_| !self.preview_calibration);
        let mut calibrator = motion_calibration::MotionCalibrator::new(
            motion_calibration::MotionCalibrationConfig::default(),
        )?;
        calibrator.begin(
            neutral.captured_at_secs,
            orientations,
            neutral.heading_radians,
            neck,
        )?;
        self.motion = Some(MotionSession {
            calibrator,
            live: neck.is_some(),
            published: None,
        });
        Ok(())
    }

    pub fn motion_calibration_active(&self) -> bool {
        self.motion.is_some()
    }

    /// Current status of the active movement calibration, freshly evaluated.
    pub fn motion_calibration_status(&self) -> Option<motion_calibration::MotionCalibrationStatus> {
        self.motion
            .as_ref()
            .map(|session| session.calibrator.status())
    }

    /// Discards the movement session and its evidence; calibration is unchanged.
    pub fn cancel_motion_calibration(&mut self) {
        self.motion = None;
    }

    /// Solves the active movement calibration atomically and commits it
    /// exactly like [`Self::calibrate_forward`], using the module's selected
    /// reference (headset-fused when available and consistent, otherwise the
    /// neutral headset yaw). The published reference heading follows that
    /// reference. With the stance heading filter enabled its uncertainty is
    /// widened (legs 10°, torso 6°) so ordinary movement with a headset can
    /// refine the result. On error nothing changes and the session continues.
    pub fn finish_motion_calibration(&mut self, now: f64) -> Result<(), CoreError> {
        let Some(session) = &self.motion else {
            return Err(CoreError("No movement calibration is in progress".into()));
        };
        if !self.calibrated || self.forward_requires_new_neutral {
            return Err(CoreError(
                "The neutral capture is no longer valid; capture neutral again".into(),
            ));
        }
        let neutral = self.neutral_reference.as_ref().ok_or_else(|| {
            CoreError("Capture a fresh neutral pose before movement calibration".into())
        })?;
        if !now.is_finite() || now < neutral.captured_at_secs {
            return Err(CoreError(
                "Invalid time for finishing movement calibration".into(),
            ));
        }
        for role in Role::CORE {
            if neutral.orientations.get(&role).map(|(id, _)| id) != self.assignments.get(&role) {
                return Err(CoreError(
                    "Assignments changed; capture neutral again".into(),
                ));
            }
        }
        let solution = session.calibrator.solve()?;
        if !solution.reference_heading_radians.is_finite() {
            return Err(CoreError(
                "Movement calibration reference is not finite".into(),
            ));
        }
        let alignments = Role::CORE
            .into_iter()
            .zip(solution.alignments)
            .collect::<BTreeMap<_, _>>();
        self.commit_forward_alignments(alignments, false);
        self.reference_heading = Some(wrap_angle(solution.reference_heading_radians));
        if self.config.estimator.enable_stance_heading {
            let (torso, leg) = (MOTION_TORSO_SIGMA_DEGREES, MOTION_LEG_SIGMA_DEGREES);
            self.stance
                .add_uncertainty([torso, torso, leg, leg, leg, leg]);
        }
        self.motion = None;
        Ok(())
    }

    /// Frame status of the active session, re-evaluated at most every
    /// `MOTION_STATUS_INTERVAL_SECS` (the evaluation scans the knee frames).
    fn published_motion_status(
        &mut self,
        now: f64,
    ) -> Option<motion_calibration::MotionCalibrationStatus> {
        let session = self.motion.as_mut()?;
        let stale = session
            .published
            .as_ref()
            .is_none_or(|(at, _)| now < *at || now - *at >= MOTION_STATUS_INTERVAL_SECS);
        if stale {
            session.published = Some((now, session.calibrator.status()));
        }
        session.published.as_ref().map(|(_, status)| status.clone())
    }

    fn forward_candidates(
        &self,
        now: f64,
    ) -> Result<(BTreeMap<Role, forward_calibration::ForwardAlignment>, bool), CoreError> {
        if !self.calibrated {
            return Err(CoreError(
                "Capture a neutral pose before forward calibration".into(),
            ));
        }
        if self.forward_requires_new_neutral {
            return Err(CoreError(
                "Recovery changed headings since the neutral capture; capture neutral again before the skiing pose".into(),
            ));
        }
        let neutral = self.neutral_reference.as_ref().ok_or_else(|| {
            CoreError("Capture a fresh neutral pose before forward calibration".into())
        })?;
        if !now.is_finite()
            || now < neutral.captured_at_secs
            || now - neutral.captured_at_secs > 300.0
        {
            return Err(CoreError("Neutral reference expired or time changed; capture neutral again before the skiing pose".into()));
        }
        let mut result = BTreeMap::new();
        let mut used_hold = false;
        for role in Role::CORE {
            let (id, q0) = neutral
                .orientations
                .get(&role)
                .ok_or_else(|| CoreError(format!("Neutral reference is missing {role:?}")))?;
            if self.assignments.get(&role) != Some(id) {
                return Err(CoreError(
                    "Assignments changed; capture neutral again".into(),
                ));
            }
            let state = self
                .samples
                .get(id)
                .ok_or_else(|| CoreError(format!("No sample for {role:?}")))?;
            if state.latest.received_at_secs <= neutral.captured_at_secs {
                return Err(CoreError(format!(
                    "Waiting for a new measured {role:?} orientation after neutral"
                )));
            }
            // A held skiing pose permits modest sway; neutral and recovery keep
            // the configured, stricter stability tolerance.
            used_hold |= self.stable_sensor_with_angle(
                role,
                state,
                now,
                self.config.stable_angle_degrees.max(4.0),
            )?;
            let start = now - self.config.stable_window_secs;
            let q1 = if state.history.iter().any(|s| s.received_at_secs >= start) {
                average_quaternions(
                    state
                        .history
                        .iter()
                        .filter(|s| s.received_at_secs >= start)
                        .map(|s| Quat::from_array(s.orientation_xyzw)),
                )
            } else {
                Quat::from_array(state.latest.orientation_xyzw)
            };
            let alignment = forward_calibration::solve_forward_alignment(
                role,
                q0.to_array(),
                q1.to_array(),
                neutral.heading_radians,
            )?;
            for observed in state.history.iter().filter(|s| {
                s.received_at_secs >= start && s.received_at_secs > neutral.captured_at_secs
            }) {
                let individual = forward_calibration::solve_forward_alignment(
                    role,
                    q0.to_array(),
                    observed.orientation_xyzw,
                    neutral.heading_radians,
                )?;
                if wrap_angle(
                    individual.heading_alignment_radians - alignment.heading_alignment_radians,
                )
                .abs()
                    > 8.0_f32.to_radians()
                {
                    return Err(CoreError(format!(
                        "{role:?} forward-axis estimates vary by more than 8°; \
                        use a deeper, steady sagittal skiing bend without twisting"
                    )));
                }
            }
            result.insert(role, alignment);
        }
        Ok((result, used_hold))
    }

    pub fn export_profile(&self) -> Profile {
        Profile {
            schema_version: 2,
            assignments: self
                .assignments
                .iter()
                .map(|(role, id)| RoleAssignment {
                    sensor_id: id.clone(),
                    role: *role,
                })
                .collect(),
            anthropometrics: self.config.anthropometrics.clone(),
            mounting: self
                .mounting
                .iter()
                .filter_map(|(role, q)| {
                    self.assignments.get(role).map(|id| MountingCalibration {
                        sensor_id: id.clone(),
                        role: *role,
                        sensor_to_bone_xyzw: q.to_array(),
                    })
                })
                .collect(),
            direction_calibrated_roles: self.functional_roles.iter().copied().collect(),
        }
    }

    /// Persisted sensor yaw origins may be different after a tracker reboot.
    /// Restoring a profile always requires a fresh neutral session calibration.
    pub fn restore_profile(&mut self, profile: Profile) -> Result<(), CoreError> {
        if ![1, 2].contains(&profile.schema_version) {
            return Err(CoreError("Unsupported profile schema".into()));
        }
        profile.anthropometrics.validate()?;
        let assignments = validated_assignments(&profile.assignments)?;
        let mut mounting = BTreeMap::new();
        for entry in &profile.mounting {
            if assignments.get(&entry.role) != Some(&entry.sensor_id)
                || mounting.contains_key(&entry.role)
            {
                return Err(CoreError(
                    "Profile mounting does not match unique sensor assignments".into(),
                ));
            }
            mounting.insert(entry.role, checked_quat(entry.sensor_to_bone_xyzw)?);
        }
        let mut functional_roles = BTreeSet::new();
        if profile.schema_version == 2 {
            for role in &profile.direction_calibrated_roles {
                if !mounting.contains_key(role) || !functional_roles.insert(*role) {
                    return Err(CoreError(
                        "Profile functional mounting roles are invalid".into(),
                    ));
                }
            }
        }
        self.assignments = assignments;
        self.preview_motion.reset();
        self.reset_stance_layers();
        self.mounting = mounting;
        self.functional_roles = functional_roles;
        self.direction_session_valid = false;
        self.heading_alignment.clear();
        self.reference_heading = None;
        self.neutral_reference = None;
        self.motion = None;
        self.manual_mounts.clear();
        self.config.anthropometrics = profile.anthropometrics;
        self.yaw.clear();
        // Also drops every per-sensor angular-rate estimator.
        self.samples.clear();
        self.head_history.clear();
        self.previous_hmd = None;
        self.reset_target_velocities();
        self.calibrated = false;
        self.preview_calibration = false;
        self.restored_profile = true;
        self.calibration_used_firmware_hold = false;
        self.floor_mismatch_m = None;
        self.recovery = RecoveryState::Idle;
        self.recovery_undo = None;
        self.forward_requires_new_neutral = false;
        self.knee_history.clear();
        self.knee_correction.clear();
        self.last_solve_secs = None;
        self.newest_evidence_secs = None;
        self.calibration_reason =
            Some("Profile restored; perform a fresh session calibration".into());
        Ok(())
    }

    /// Explicit user assertion: stand neutral and face forward with the torso.
    /// A tap recognizer must be opt-in to this instruction before calling this.
    /// Gates reject stale/moving/incompatible poses, but cannot distinguish a
    /// small head-only turn from common-mode body yaw drift. Do not call this
    /// automatically merely because a solver residual is large.
    pub fn request_recovery(&mut self, now: f64) {
        if !now.is_finite() || now < 0.0 {
            return;
        }
        self.recovery = RecoveryState::Pending {
            requested_at_secs: now,
            blocked_reason: Some("Waiting for tap impulses to settle".into()),
        };
    }

    pub fn cancel_recovery(&mut self) {
        self.recovery = RecoveryState::Idle;
    }

    /// Subtracts exactly the per-role yaw deltas the last recovery applied.
    /// Corrections accrued after the recovery, including stance-filter
    /// corrections, are kept; with none accrued the published headings return
    /// exactly to their values before the recovery.
    pub fn undo_recovery(&mut self) -> bool {
        if let Some(deltas) = self.recovery_undo.take() {
            for (role, delta) in deltas {
                if let Some(yaw) = self.yaw.get_mut(&role) {
                    *yaw = wrap_angle(*yaw - delta);
                }
            }
            self.recovery = RecoveryState::Undone;
            self.knee_history.clear();
            // The yaw under the stance filter's keyframes and anchors changed.
            self.interrupt_stance_layers();
            true
        } else {
            false
        }
    }

    /// The standing tracking space was rotated about +Y by `yaw_delta_radians`
    /// (for example a recenter). Rotates every world-heading reference by the
    /// same angle so identical sensor data with an identically rotated HMD
    /// yields identically rotated output. Mounting, dynamic yaw, stance
    /// corrections, calibration state and recovery undo remain valid; contact
    /// hypotheses, leg-IK anchors and the stance filter's motion history are
    /// dropped. Nonfinite input is ignored.
    pub fn rebase_standing_space(&mut self, yaw_delta_radians: f32) {
        if !yaw_delta_radians.is_finite() {
            return;
        }
        let delta = wrap_angle(yaw_delta_radians);
        self.rotate_world_heading(delta);
        self.last_rebase_evidence = None;
        self.standing_space_rebases = self.standing_space_rebases.saturating_add(1);
        self.last_rebase_degrees = delta.to_degrees();
    }

    /// Left-multiplies every world-heading reference by a yaw of `delta`
    /// radians (reference heading, per-role heading alignment, neutral
    /// heading, manual-mount headings). Mounting, dynamic yaw and stance
    /// corrections are untouched; histories that would compare across the
    /// rotation are dropped.
    fn rotate_world_heading(&mut self, delta: f32) {
        if let Some(heading) = &mut self.reference_heading {
            *heading = wrap_angle(*heading + delta);
        }
        let roles: BTreeSet<Role> = self
            .heading_alignment
            .keys()
            .chain(self.mounting.keys())
            .copied()
            .collect();
        for role in roles {
            let alignment = self.heading_alignment.entry(role).or_insert(0.0);
            *alignment = wrap_angle(*alignment + delta);
        }
        if let Some(neutral) = &mut self.neutral_reference {
            neutral.heading_radians = wrap_angle(neutral.heading_radians + delta);
            // The rotation's center is unknown: neck positions are no longer
            // comparable, so a later movement calibration uses yaw only.
            neutral.neck_at_neutral = None;
        }
        // Movement evidence mixes headset positions from both spaces.
        self.motion = None;
        for state in self.manual_mounts.values_mut() {
            state.rebase_heading(delta);
        }
        self.head_history.clear();
        self.knee_history.clear();
        self.preview_motion.reset();
        // The corrections are left world yaws on top of the rotated alignment
        // and stay valid; the motion history would see the space jump.
        self.interrupt_stance_layers();
        // Output positions jump with the space; the discontinuity detector must
        // not see this same event again as a jump of the next HMD sample.
        self.reset_target_velocities();
        self.previous_hmd = None;
    }

    /// The application observed something that may have moved SteamVR's
    /// standing space: a new anchor-helper session or a material change of
    /// the raw-to-standing transform (`transform_yaw_radians` is that
    /// change's yaw when it is a pure yaw). This is not evidence that the
    /// space moved: the next usable headset samples decide (see
    /// [`tracking_space`]); the transform's yaw is used only when the
    /// observed headset jump agrees with it. Never clears calibration.
    /// Nonfinite times are ignored; a nonfinite yaw counts as `None`.
    pub fn note_tracking_space_event(&mut self, at_secs: f64, transform_yaw_radians: Option<f32>) {
        if !at_secs.is_finite() {
            return;
        }
        self.tracking_space_hint = Some(tracking_space::TrackingSpaceHint {
            at_secs,
            transform_yaw_radians: transform_yaw_radians.filter(|yaw| yaw.is_finite()),
        });
    }

    /// Compares a newly observed usable HMD sample with the previous one using
    /// [`tracking_space::decide`]. Consecutive samples (< 0.1 s) are always
    /// compared; after an application-reported tracking-space event, samples
    /// up to 2 s apart are compared with time-scaled thresholds. A jump with a
    /// live calibration and usable facing rebases the standing space (by the
    /// transform's yaw when the jump agrees with it); without facing, velocity
    /// and angular histories that would see the teleport are reset. A gap
    /// longer than 2 s with a live calibration keeps the calibration and
    /// widens the stance filter's heading uncertainty (legs 15°, torso 8°).
    /// Calibration is never cleared here.
    fn detect_tracking_discontinuity(&mut self, anchors: &Anchors, now: f64) {
        let Some(head) = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs)
        else {
            return;
        };
        let current = HmdSample {
            at_secs: head.received_at_secs,
            position: Vec3::from_array(head.position),
            rotation: checked_quat(head.orientation_xyzw).unwrap(),
        };
        let Some(previous) = self.previous_hmd else {
            self.previous_hmd = Some(current);
            return;
        };
        if current.at_secs <= previous.at_secs {
            return;
        }
        self.previous_hmd = Some(current);
        let hint = self.tracking_space_hint;
        if hint.is_some_and(|hint| !tracking_space::hint_applies(&hint, previous.at_secs)) {
            self.tracking_space_hint = None;
        }
        let observation = |sample: HmdSample| tracking_space::HeadsetObservation {
            at_secs: sample.at_secs,
            position: sample.position.to_array(),
            orientation_xyzw: sample.rotation.to_array(),
        };
        let decision = tracking_space::decide(
            &observation(previous),
            &observation(current),
            self.tracking_space_hint.as_ref(),
        );
        let live = self.calibrated && !self.preview_calibration;
        match decision {
            tracking_space::ContinuityDecision::NotCompared
            | tracking_space::ContinuityDecision::Continuous => {}
            tracking_space::ContinuityDecision::Resumed { gap_secs } => {
                self.tracking_space_hint = None;
                if live {
                    self.headset_resumed(gap_secs);
                }
            }
            _ if !self.config.estimator.enable_auto_rebase || !live => {}
            tracking_space::ContinuityDecision::Rebase {
                yaw_radians,
                evidence,
            } => {
                self.tracking_discontinuities = self.tracking_discontinuities.saturating_add(1);
                self.rebase_standing_space(yaw_radians);
                self.last_rebase_evidence = Some(evidence);
                self.tracking_space_hint = None;
                // Rebasing forgets the previous sample; the new one is the baseline.
                self.previous_hmd = Some(current);
            }
            tracking_space::ContinuityDecision::JumpWithoutHeading => {
                self.tracking_discontinuities = self.tracking_discontinuities.saturating_add(1);
                self.tracking_space_hint = None;
                self.head_history.clear();
                self.knee_history.clear();
                self.preview_motion.reset();
                self.reset_target_velocities();
                self.interrupt_stance_layers();
                // Headset positions jumped: neck displacement is meaningless.
                if let Some(neutral) = &mut self.neutral_reference {
                    neutral.neck_at_neutral = None;
                }
                self.motion = None;
            }
        }
    }

    /// Usable headset samples resumed after a long gap with a live
    /// calibration. The calibration is kept: the standing space of the same
    /// headset in the same room is assumed to be the same physical frame. If
    /// that assumption is wrong, wider stance-filter uncertainty lets ordinary
    /// movement re-converge quickly. Headset positions from before the gap are
    /// not compared with later ones.
    fn headset_resumed(&mut self, gap_secs: f64) {
        self.headset_resumptions = self.headset_resumptions.saturating_add(1);
        self.last_headset_gap_secs = gap_secs as f32;
        self.head_history.clear();
        self.knee_history.clear();
        self.reset_target_velocities();
        self.interrupt_stance_layers();
        if let Some(neutral) = &mut self.neutral_reference {
            neutral.neck_at_neutral = None;
        }
        self.motion = None;
        if self.config.estimator.enable_stance_heading {
            let (torso, leg) = (RESUME_TORSO_SIGMA_DEGREES, RESUME_LEG_SIGMA_DEGREES);
            self.stance
                .add_uncertainty([torso, torso, leg, leg, leg, leg]);
        }
    }

    /// Mean standing-space headset height over the stable window, when the
    /// headset history passes the neutral gate's stability checks (dense,
    /// gap-free samples spanning the window, within 3.5 cm and 3°). Used for
    /// the automatic eye-height update before a live neutral capture.
    pub fn stable_headset_height(&self, now: f64) -> Option<f32> {
        let window = self.stable_head_window(now).ok()?;
        let height = window.iter().map(|h| h.position[1]).sum::<f32>() / window.len() as f32;
        height.is_finite().then_some(height)
    }

    /// The headset history over the stable window, if it is dense, gap free
    /// and still (the neutral gate's headset stability check).
    fn stable_head_window(&self, now: f64) -> Result<Vec<&AnchorPose>, CoreError> {
        let head_window: Vec<_> = self
            .head_history
            .iter()
            .filter(|h| h.received_at_secs >= now - self.config.stable_window_secs)
            .collect();
        if head_window.len() < 3
            || head_window.last().unwrap().received_at_secs
                - head_window.first().unwrap().received_at_secs
                < self.config.stable_window_secs * 0.85
        {
            return Err(CoreError("Waiting for stable HMD history".into()));
        }
        if head_window.windows(2).any(|pair| {
            pair[1].received_at_secs - pair[0].received_at_secs
                > self.config.stale_after_secs.min(0.15)
        }) {
            return Err(CoreError(
                "HMD sample gaps interrupted stable history".into(),
            ));
        }
        let first = head_window[0];
        if head_window.iter().any(|a| {
            (Vec3::from_array(a.position) - Vec3::from_array(first.position)).length() > 0.035
                || angle(
                    Quat::from_array(a.orientation_xyzw),
                    Quat::from_array(first.orientation_xyzw),
                ) > 3.0_f32.to_radians()
        }) {
            return Err(CoreError("Headset is moving; hold the neutral pose".into()));
        }
        Ok(head_window)
    }

    /// Carries a desktop (preview) calibration over to live use with a
    /// headset, without repeating the procedure. A desktop calibration already
    /// knows every sensor's mounting and its heading relative to the body;
    /// the only unknown is the body's heading in standing space. Waits (returns
    /// an error, changing nothing) until the user is upright and still: the
    /// neutral gate's stability checks (stable core sensors, segment tilt
    /// within the neutral limit, a stable upright headset over the stable
    /// window; the configured eye height is not enforced) and every core
    /// segment facing within `recovery_max_yaw_degrees` of the torso heading.
    /// Then rotates the reference heading and every heading alignment by
    /// `headset yaw - torso heading` (torso heading: circular mean of chest
    /// and hip yaw), the same left-factor rotation as
    /// [`Self::rebase_standing_space`] with mounting untouched, clears the
    /// preview flag, resets the estimator layers and, with the stance filter
    /// enabled, widens its uncertainty (legs 10°, torso 6°). Like a live
    /// neutral capture this assumes the head faces the same way as the torso.
    pub fn align_preview_to_headset(
        &mut self,
        now: f64,
        anchors: &Anchors,
    ) -> Result<(), CoreError> {
        if !self.calibrated || !self.preview_calibration {
            return Err(CoreError(
                "Only a desktop (preview) calibration can be aligned to the headset".into(),
            ));
        }
        if !self.direction_calibrated() {
            return Err(CoreError(
                "Finish the desktop calibration (movement or skiing pose) before aligning".into(),
            ));
        }
        if self.motion.is_some() {
            return Err(CoreError("A movement calibration is in progress".into()));
        }
        let gate = self.neutral_gate(now, anchors, false, GatePurpose::Alignment)?;
        let head = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs)
            .ok_or_else(|| CoreError("A fresh, valid live HMD is required".into()))?;
        let head_rotation = checked_quat(head.orientation_xyzw)?;
        let forward = head_rotation * Vec3::NEG_Z;
        if Vec3::new(forward.x, 0.0, forward.z).length() <= tracking_space::MIN_HORIZONTAL_FACING {
            return Err(CoreError("Look ahead with the headset upright".into()));
        }
        let head_yaw = yaw_of(head_rotation);
        let chest = yaw_of(self.segment(Role::Chest, now).rotation);
        let hip = yaw_of(self.segment(Role::Hip, now).rotation);
        let torso = (chest.sin() + hip.sin()).atan2(chest.cos() + hip.cos());
        if !torso.is_finite() || !head_yaw.is_finite() {
            return Err(CoreError("Torso heading is not defined".into()));
        }
        let limit = self.config.recovery_max_yaw_degrees.to_radians();
        for role in Role::CORE {
            let yaw = yaw_of(self.segment(role, now).rotation);
            if wrap_angle(yaw - torso).abs() > limit {
                return Err(CoreError(format!(
                    "{role:?} faces a different way than the torso; stand straight with your legs and torso facing forward"
                )));
            }
        }
        let floor_mismatch_m = self
            .stable_headset_height(now)
            .map(|height| height - self.config.anthropometrics.standing_chain_height_m());
        let delta = wrap_angle(head_yaw - torso);
        self.rotate_world_heading(delta);
        self.preview_calibration = false;
        self.reset_stance_layers();
        if self.config.estimator.enable_stance_heading {
            let (torso_sigma, leg) = (MOTION_TORSO_SIGMA_DEGREES, MOTION_LEG_SIGMA_DEGREES);
            self.stance
                .add_uncertainty([torso_sigma, torso_sigma, leg, leg, leg, leg]);
        }
        self.recovery = RecoveryState::Idle;
        self.recovery_undo = None;
        self.floor_mismatch_m = floor_mismatch_m;
        self.calibration_used_firmware_hold |= gate.used_firmware_hold;
        self.calibration_reason = self
            .calibration_used_firmware_hold
            .then(|| "Calibration used a firmware-held orientation assumption; continuous fresh sensor measurements were not observed".into());
        // The detector's baseline is this sample, in the same standing space.
        self.previous_hmd = Some(HmdSample {
            at_secs: head.received_at_secs,
            position: Vec3::from_array(head.position),
            rotation: head_rotation,
        });
        self.tracking_space_hint = None;
        Ok(())
    }

    /// Live neutral capture that first adopts `config` (for example a
    /// standing eye height measured from the headset, with body dimensions
    /// rescaled). The new configuration is applied only after the neutral gate
    /// passes under it; a rejected capture leaves the previous configuration
    /// and calibration untouched, so a dimension change can never leave the
    /// calibration cleared by itself.
    pub fn calibrate_neutral_with_config(
        &mut self,
        now: f64,
        anchors: &Anchors,
        preview: bool,
        config: TrackingConfig,
    ) -> Result<(), CoreError> {
        config.validate()?;
        let preview_anchors = Anchors::default();
        let gate_anchors = if preview { &preview_anchors } else { anchors };
        self.observe_head(gate_anchors, now);
        let previous = std::mem::replace(&mut self.config, config);
        let gate = self.neutral_gate(now, gate_anchors, preview, GatePurpose::Calibration);
        let config = std::mem::replace(&mut self.config, previous);
        if let Err(error) = gate {
            self.calibration_reason = Some(error.0.clone());
            return Err(error);
        }
        self.set_config(config)?;
        self.calibrate_neutral(now, anchors, preview)
    }

    /// Anchors used for the body solve: positions advanced by their reported
    /// linear velocity over their age when enabled. Returns the HMD advance.
    fn extrapolated_anchors(&self, anchors: &Anchors, now: f64) -> (Anchors, f32) {
        let mut out = anchors.clone();
        if !self.config.estimator.enable_hmd_extrapolation {
            return (out, 0.0);
        }
        let advance = |anchor: &mut Option<AnchorPose>| -> f32 {
            let Some(anchor) = anchor.as_mut() else {
                return 0.0;
            };
            let Some(velocity) = anchor.linear_velocity.filter(|v| finite3(*v)) else {
                return 0.0;
            };
            let velocity = Vec3::from_array(velocity);
            if !anchor.valid
                || !finite3(anchor.position)
                || !anchor.received_at_secs.is_finite()
                || !velocity.length().is_finite()
                || velocity.length() >= MAX_ANCHOR_EXTRAPOLATION_SPEED
            {
                return 0.0;
            }
            let age = (now - anchor.received_at_secs).clamp(0.0, MAX_ANCHOR_EXTRAPOLATION_SECS);
            if !age.is_finite() || age <= 0.0 {
                return 0.0;
            }
            let position = Vec3::from_array(anchor.position) + velocity * age as f32;
            if !position.is_finite() {
                return 0.0;
            }
            anchor.position = position.to_array();
            age as f32
        };
        let hmd = advance(&mut out.hmd);
        advance(&mut out.left_hand);
        advance(&mut out.right_hand);
        (out, hmd)
    }

    pub fn solve(&mut self, now: f64, anchors: &Anchors) -> TrackingFrame {
        let valid_time = now.is_finite() && now >= 0.0;
        let now = if valid_time { now } else { 0.0 };
        let empty_anchors = Anchors::default();
        let anchors = if valid_time { anchors } else { &empty_anchors };
        // Before the sample enters head history: a rebase clears that history.
        self.detect_tracking_discontinuity(anchors, now);
        self.observe_head(anchors, now);
        // Retain real HMD history for a future explicit live calibration, but
        // never let incidental head/hand anchors move an active desktop preview.
        let anchors = if self.preview_calibration {
            &empty_anchors
        } else {
            anchors
        };
        // Recovery gates use the raw anchors and raw measured orientations.
        self.try_recovery(now, anchors);
        let dt = self
            .last_solve_secs
            .map(|last| (now - last).clamp(0.0, 0.05) as f32)
            .unwrap_or(0.0);
        self.last_solve_secs = Some(now);
        let (solve_anchors, hmd_extrapolated_secs) = self.extrapolated_anchors(anchors, now);
        // The stance corrections applied to this frame are those from before
        // this frame's filter step: an update takes effect from the next frame,
        // so the chain the filter sees is exactly the published one.
        let stance_applied = self.applied_stance_corrections();
        let mut segments = self.compute_segments(now);
        let prediction_active = segments.values().any(|s| s.valid && s.extrapolated);
        let mut corrections = Vec::new();
        for (thigh, calf) in [
            (Role::LeftThigh, Role::LeftCalf),
            (Role::RightThigh, Role::RightCalf),
        ] {
            corrections.push(self.correct_knee(now, dt, thigh, calf, &mut segments));
        }
        let (head_live, head_position, head_rotation) = self.head_pose(&solve_anchors, now);
        let body_chain = self.body_chain(head_position, head_rotation, &segments);
        let live = head_live && self.direction_calibrated() && !self.preview_calibration;
        // The raw (not extrapolated) headset sample for its velocity.
        let head_sample = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs)
            .map(|head| HeadSample {
                at_secs: head.received_at_secs,
                position: Vec3::from_array(head.position),
                reported_velocity: head
                    .linear_velocity
                    .filter(|v| finite3(*v))
                    .map(Vec3::from_array)
                    .filter(|v| v.length() < MAX_ANCHOR_EXTRAPOLATION_SPEED),
            });
        let layers = self.run_stance_layers(
            now,
            live,
            head_position,
            head_rotation,
            head_sample,
            &segments,
            &body_chain,
        );
        let mut frame = self.build_frame(
            now,
            &solve_anchors,
            &segments,
            corrections,
            &body_chain,
            layers.ik.as_ref(),
            &stance_applied,
        );
        self.preview_motion.apply(
            &mut frame,
            self.config.enable_preview_motion,
            &self.config.anthropometrics,
        );
        self.finish_target_velocities(&mut frame, now);
        let mut stance = self.stance.last_output();
        if !layers.stance_active {
            stance.status = stance_heading::StanceHeadingStatus::Idle;
            stance.release_requests = [false; 2];
        }
        frame.estimator = EstimatorStatus {
            prediction_active,
            hmd_extrapolated_secs: if head_live {
                hmd_extrapolated_secs
            } else {
                0.0
            },
            standing_space_rebases: self.standing_space_rebases,
            last_rebase_degrees: self.last_rebase_degrees,
            tracking_discontinuities: self.tracking_discontinuities,
            last_rebase_evidence: self.last_rebase_evidence,
            headset_resumptions: self.headset_resumptions,
            last_headset_gap_secs: self.last_headset_gap_secs,
            stance_active: layers.stance_active,
            stance,
            contacts: self.contact.contacts(),
            contact_speed_allowance_mps: self.contact.speed_allowance(),
            contact_torso_allowance_mps: self.contact.granted_torso_allowance(),
            double_support: self.contact.double_support(),
            leg_ik_active: layers.ik.is_some(),
            leg_ik: layers
                .ik
                .map_or_else(idle_leg_ik_diagnostics, |ik| ik.diagnostics()),
            support_floor: layers.support_floor,
        };
        frame.motion_calibration = self.published_motion_status(now);
        frame
    }

    /// The usable HMD eye pose for the body solve, or the synthetic upright
    /// standing pose when there is none (never published as live).
    fn head_pose(&self, anchors: &Anchors, now: f64) -> (bool, Vec3, Quat) {
        let body = &self.config.anthropometrics;
        let live_head = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs);
        (
            live_head.is_some(),
            live_head
                .map(|h| Vec3::from_array(h.position))
                .unwrap_or(Vec3::new(0.0, body.standing_eye_height_m, 0.0)),
            live_head
                .map(|h| checked_quat(h.orientation_xyzw).unwrap())
                .unwrap_or(Quat::IDENTITY),
        )
    }

    /// Forward kinematics of the shared chain. This single function places the
    /// published skeleton and is what the stance filter linearises.
    fn body_chain(
        &self,
        head_position: Vec3,
        head_rotation: Quat,
        segments: &BTreeMap<Role, Segment>,
    ) -> chain::BodyChain {
        let body = &self.config.anthropometrics;
        chain::solve_chain(
            &chain::ChainDimensions::from_anthropometrics(body, body.eye_forward_m),
            &chain::ChainPose {
                head_position,
                head_rotation,
                rotations: Role::CORE.map(|role| segments[&role].rotation),
            },
        )
    }

    /// Contact hypotheses, stance heading filter and leg IK for one frame.
    ///
    /// They run only for a live session: a usable HMD, completed direction
    /// calibration, no desktop preview, and at least one foot whose whole chain
    /// (chest, hip, thigh, calf) is valid. Otherwise contacts and leg IK are
    /// reset and the filter keeps its corrections but forgets its motion
    /// history, so nothing is compared across the gap. Contacts and leg IK
    /// accept firmware-held segments; the filter only ever uses freshly
    /// measured ones.
    #[allow(clippy::too_many_arguments)]
    fn run_stance_layers(
        &mut self,
        now: f64,
        live: bool,
        head_position: Vec3,
        head_rotation: Quat,
        head_sample: Option<HeadSample>,
        segments: &BTreeMap<Role, Segment>,
        body_chain: &chain::BodyChain,
    ) -> StanceLayers {
        let idle = StanceLayers {
            stance_active: false,
            ik: None,
            support_floor: leg_ik::SupportFloorDiagnostics::default(),
        };
        let estimator = &self.config.estimator;
        let (stance_on, ik_on) = (estimator.enable_stance_heading, estimator.enable_leg_ik);
        if !stance_on && !ik_on {
            self.contact.reset();
            self.support_floor.reset();
            self.leg_ik.reset();
            return idle;
        }
        let rotations = Role::CORE.map(|role| segments[&role].rotation);
        let usable = Role::CORE.map(|role| {
            let segment = segments[&role];
            segment.valid && !segment.held
        });
        // Contacts and leg IK also accept firmware-held orientations: the
        // firmware stops sending while a sensor does not rotate, so a held leg
        // is a leg that is standing still, which is exactly when a planted foot
        // must stay locked and flat. The heading filter does not: `usable`
        // above excludes held segments, so it never treats an assumed-unchanged
        // orientation as new motion evidence.
        let foot_present = [0, 1].map(|foot| {
            body_chain.foot_finite(foot)
                && chain::FOOT_CHAIN[foot]
                    .iter()
                    .all(|&i| segments[&Role::CORE[i]].valid)
        });
        let Some(head_sample) = head_sample.filter(|_| {
            live && body_chain.head_finite && foot_present.iter().any(|present| *present)
        }) else {
            self.interrupt_stance_layers();
            return idle;
        };
        // While the filter is unsure about yaw, a truly planted ankle appears to
        // slide at about sin(error) times the body's horizontal speed: widen the
        // contact speed gates by that much (zero with the filter off). Smoothed
        // horizontal standing-space headset velocity: the anchor's reported
        // velocity when trustworthy, else a finite difference of its position.
        self.head_velocity
            .update(head_sample.at_secs, head_sample.position);
        let mut head_velocity = head_sample
            .reported_velocity
            .unwrap_or_else(|| self.head_velocity.velocity(now));
        head_velocity.y = 0.0;
        let allowance = if stance_on {
            self.stance.contact_speed_allowance(head_velocity)
        } else {
            [0.0; 2]
        };
        self.contact.set_speed_allowance(allowance);
        let body = &self.config.anthropometrics;
        let (neck_to_chest, chest_to_hip) = (body.neck_to_chest_m, body.chest_to_hip_m);
        // Support-foot floor constraint (part of leg IK): a vertical translation
        // of the pelvis and legs, applied before contacts so that both the
        // stationary-foot hypotheses and the foot-lock anchors see the corrected
        // ankles. The stance filter keeps linearising the uncorrected chain; it
        // only uses horizontal displacement, which this does not change.
        let support_floor = if ik_on {
            self.support_floor.update(&leg_ik::SupportFloorInput {
                at_secs: now,
                hip: body_chain.hip,
                ankles: body_chain.ankles,
                thigh_rotations: chain::THIGH.map(|i| rotations[i]),
                calf_rotations: chain::CALF.map(|i| rotations[i]),
                chest_rotation: rotations[chain::CHEST],
                spine_rotation: body_chain.spine_rotation,
                usable: foot_present,
                neck_to_chest_m: neck_to_chest,
                chest_to_hip_m: chest_to_hip,
                ankle_to_sole_m: body.ankle_to_sole_m,
            })
        } else {
            self.support_floor.reset();
            leg_ik::SupportFloorDiagnostics::default()
        };
        let lift = Vec3::Y * support_floor.vertical_m;
        let lifted = |p: Vec3| if lift == Vec3::ZERO { p } else { p + lift };
        let floor_chest = if lift == Vec3::ZERO {
            body_chain.chest
        } else {
            body_chain.chest + lift * leg_ik::chest_fraction(neck_to_chest, chest_to_hip)
        };
        let floor_hip = lifted(body_chain.hip);
        let floor_hip_joints = body_chain.hip_joints.map(lifted);
        let floor_knees = body_chain.knees.map(lifted);
        let floor_ankles = body_chain.ankles.map(lifted);
        // A torso that swings moves the head-down ankles through its model
        // error: widen the contact gates by that much (granted only in double
        // support). Swing rate: rate of change of each torso axis direction.
        let swing = |role: Role| {
            let segment = segments[&role];
            segment
                .angular_velocity
                .cross(segment.rotation * Vec3::Y)
                .length()
        };
        self.contact.set_torso_motion(
            swing(Role::Chest).max(swing(Role::Hip)),
            neck_to_chest + chest_to_hip,
        );
        self.contact.update(
            now,
            floor_ankles,
            chain::CALF.map(|i| rotations[i]),
            foot_present,
        );
        let mut stance_active = false;
        if stance_on {
            let output = self.stance.step(&stance_heading::StanceHeadingInput {
                at_secs: now,
                head_position,
                head_rotation,
                chain: body_chain,
                rotations,
                usable,
                planted: self.contact.contacts().map(|c| c.planted()),
                angular_speed: Role::CORE.map(|role| segments[&role].angular_velocity.length()),
            });
            for (foot, release) in output.release_requests.into_iter().enumerate() {
                if release {
                    self.contact.force_release(foot, now);
                }
            }
            stance_active = true;
        }
        let ik = if ik_on {
            let body = &self.config.anthropometrics;
            let measured_foot_rotations = [Role::LeftFoot, Role::RightFoot].map(|role| {
                self.direction_certified_optional(role)
                    .then(|| segments[&role].rotation)
            });
            let output = self.leg_ik.solve(&leg_ik::LegIkInput {
                at_secs: now,
                neck: body_chain.neck,
                chest: floor_chest,
                hip: floor_hip,
                hip_joints: floor_hip_joints,
                knees: floor_knees,
                ankles: floor_ankles,
                chest_rotation: rotations[chain::CHEST],
                spine_rotation: body_chain.spine_rotation,
                hip_rotation: rotations[chain::HIP],
                thigh_rotations: chain::THIGH.map(|i| rotations[i]),
                calf_rotations: chain::CALF.map(|i| rotations[i]),
                measured_foot_rotations,
                planted: self.contact.contacts().map(|c| c.planted()),
                neck_to_chest_m: body.neck_to_chest_m,
                chest_to_hip_m: body.chest_to_hip_m,
                hip_width_m: body.hip_width_m,
                thigh_m: body.thigh_length_m,
                calf_m: body.calf_length_m,
                ankle_to_sole_m: body.ankle_to_sole_m,
                enabled: true,
            });
            for (foot, release) in output.release_requests.into_iter().enumerate() {
                if release {
                    self.contact.force_release(foot, now);
                }
            }
            // Held, unavailable or passthrough output is never published.
            (output.status == leg_ik::LegIkStatus::Active).then_some(output)
        } else {
            self.leg_ik.reset();
            None
        };
        StanceLayers {
            stance_active,
            support_floor: if ik.is_some() {
                support_floor
            } else {
                leg_ik::SupportFloorDiagnostics::default()
            },
            ik,
        }
    }

    /// An optional (foot or upper-arm) role with a measured, direction-certified IMU.
    fn direction_certified_optional(&self, role: Role) -> bool {
        self.assignments.contains_key(&role)
            && self.direction_session_valid
            && self.functional_roles.contains(&role)
    }

    /// Linear velocity from the final published positions; zero (and a reset
    /// estimator) for invalid targets, non-live frames and missing slots.
    fn finish_target_velocities(&mut self, frame: &mut TrackingFrame, now: f64) {
        let live = !frame.preview_only;
        let mut published = [false; TARGET_SLOTS];
        for target in &mut frame.targets {
            let slot = target_slot(target.role);
            published[slot] = true;
            let position = Vec3::from_array(target.position);
            if !live || !target.valid || !position.is_finite() {
                self.target_velocity[slot].reset();
                target.linear_velocity = [0.0; 3];
                target.angular_velocity = [0.0; 3];
                continue;
            }
            let velocity = self.target_velocity[slot].update(now, position);
            target.linear_velocity = if velocity.is_finite() {
                velocity.to_array()
            } else {
                self.target_velocity[slot].reset();
                [0.0; 3]
            };
            if !finite3(target.angular_velocity) {
                target.angular_velocity = [0.0; 3];
            }
        }
        for (slot, estimator) in self.target_velocity.iter_mut().enumerate() {
            if !published[slot] {
                estimator.reset();
            }
        }
    }

    fn observe_head(&mut self, anchors: &Anchors, now: f64) {
        if let Some(head) = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs)
            && self
                .head_history
                .back()
                .is_none_or(|previous| head.received_at_secs > previous.received_at_secs)
        {
            let mut head = head.clone();
            let rotation = checked_quat(head.orientation_xyzw).unwrap();
            head.orientation_xyzw = rotation.to_array();
            // Live movement calibration: each new usable headset sample once.
            if let Some(session) = self.motion.as_mut().filter(|session| session.live) {
                let body = &self.config.anthropometrics;
                session.calibrator.observe_headset(
                    head.received_at_secs,
                    Vec3::from_array(head.position),
                    rotation,
                    body.head_to_neck_m,
                    body.eye_forward_m,
                );
            }
            self.head_history.push_back(head);
        }
        while self
            .head_history
            .front()
            .is_some_and(|a| a.received_at_secs < now - self.config.stable_window_secs - 0.1)
            || self.head_history.len() > 1024
        {
            self.head_history.pop_front();
        }
    }

    fn neutral_gate(
        &self,
        now: f64,
        anchors: &Anchors,
        preview: bool,
        purpose: GatePurpose,
    ) -> Result<NeutralGate, CoreError> {
        let recovery = purpose == GatePurpose::Recovery;
        if !now.is_finite() || now < 0.0 {
            return Err(CoreError("Invalid calibration time".into()));
        }
        for role in Role::CORE {
            if !self.assignments.contains_key(&role) {
                return Err(CoreError(format!("Assign the {role:?} sensor first")));
            }
        }
        let head = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs);
        if !preview && head.is_none() {
            return Err(CoreError("A fresh, valid live HMD is required".into()));
        }
        if let Some(head) = head {
            let actual_height = head.position[1];
            let configured_height = self.config.anthropometrics.standing_eye_height_m;
            let height_difference = actual_height - configured_height;
            let height_tolerance = self.config.standing_height_tolerance_m;
            if purpose != GatePurpose::Alignment && height_difference.abs() > height_tolerance {
                let minimum_height = configured_height - height_tolerance;
                let maximum_height = configured_height + height_tolerance;
                return Err(CoreError(format!(
                    "Headset height {actual_height:.2} m is outside {minimum_height:.2}–{maximum_height:.2} m \
                     (configured eye height {configured_height:.2} m, difference {height_difference:+.2} m, \
                     limit ±{height_tolerance:.2} m). Stand upright with the headset worn; \
                     check the eye-height setting and Quest/SteamVR floor level."
                )));
            }
            let rotation = checked_quat(head.orientation_xyzw)?;
            if (rotation * Vec3::Y).dot(Vec3::Y) < 25.0_f32.to_radians().cos() {
                return Err(CoreError("Look forward with the headset upright".into()));
            }
            if !preview {
                self.stable_head_window(now)?;
            }
        }
        let mut gate = NeutralGate {
            used_firmware_hold: false,
            validated: BTreeSet::new(),
        };
        for (role, id) in &self.assignments {
            let core_role = Role::CORE.contains(role);
            if !core_role && !self.functional_roles.contains(role) {
                continue;
            }
            match self.neutral_role_check(*role, id, now, head, recovery) {
                Ok(used_hold) => {
                    gate.used_firmware_hold |= used_hold;
                    gate.validated.insert(*role);
                }
                // Recovery requires only the core roles. A stale, moving or
                // otherwise unchecked optional role keeps its yaw unchanged.
                Err(_) if recovery && !core_role => {}
                Err(error) => return Err(error),
            }
        }
        Ok(gate)
    }

    fn neutral_role_check(
        &self,
        role: Role,
        id: &str,
        now: f64,
        head: Option<&AnchorPose>,
        recovery: bool,
    ) -> Result<bool, CoreError> {
        let state = self
            .samples
            .get(id)
            .ok_or_else(|| CoreError(format!("No sample for {role:?}")))?;
        let used_firmware_hold = self.stable_sensor(role, state, now)?;
        if self.calibrated
            || (self.direction_session_valid && self.functional_roles.contains(&role))
        {
            let segment = self.segment(role, now);
            if recovery && !segment.valid {
                return Err(CoreError(format!("{role:?} is not valid for recovery")));
            }
            let q = segment.rotation;
            if (q * Vec3::Y).dot(Vec3::Y) < self.config.neutral_tilt_degrees.to_radians().cos() {
                return Err(CoreError(format!(
                    "{role:?} tilt is incompatible with neutral standing"
                )));
            }
            if recovery {
                let head_yaw = head
                    .map(|a| yaw_of(checked_quat(a.orientation_xyzw).unwrap()))
                    .unwrap_or(0.0);
                if wrap_angle(head_yaw - yaw_of(q)).abs()
                    > self.config.recovery_max_yaw_degrees.to_radians()
                {
                    return Err(CoreError(format!(
                        "{role:?} heading is ambiguous; face forward and use explicit calibration"
                    )));
                }
            }
        }
        Ok(used_firmware_hold)
    }

    fn stable_sensor(&self, role: Role, state: &SampleState, now: f64) -> Result<bool, CoreError> {
        self.stable_sensor_with_angle(role, state, now, self.config.stable_angle_degrees)
    }

    fn stable_sensor_with_angle(
        &self,
        role: Role,
        state: &SampleState,
        now: f64,
        stable_angle_degrees: f32,
    ) -> Result<bool, CoreError> {
        if state.sensor_unhealthy {
            return Err(CoreError(format!("{role:?} reports a sensor health error")));
        }
        let measured = self.stable_measured_sensor(role, state, now, stable_angle_degrees);
        if measured.is_ok() {
            return Ok(false);
        }
        if let Some(hold) = held_evidence(state, now) {
            let start = now - self.config.stable_window_secs;
            if hold.continuity_since_secs > start {
                return Err(CoreError(format!(
                    "Waiting for a continuous firmware-held {role:?} interval"
                )));
            }
            // Include the actual observation immediately preceding the window;
            // it represents the assumed orientation at the window boundary.
            let reference = state
                .history
                .iter()
                .rev()
                .find(|s| s.received_at_secs <= start)
                .ok_or_else(|| {
                    CoreError(format!(
                        "Waiting for measured {role:?} history before the held interval"
                    ))
                })?;
            let reference_q = Quat::from_array(reference.orientation_xyzw);
            let moving = state
                .history
                .iter()
                .filter(|s| s.received_at_secs >= start)
                .any(|s| {
                    angle(reference_q, Quat::from_array(s.orientation_xyzw))
                        > stable_angle_degrees.to_radians()
                        || s.acceleration_mps2.is_some_and(|a| {
                            Vec3::from_array(a).length() > self.config.stable_acceleration_mps2
                        })
                });
            if moving {
                return Err(CoreError(format!(
                    "{role:?} measured motion interrupts firmware-held stability"
                )));
            }
            return Ok(true);
        }
        measured.map(|_| false)
    }

    fn stable_measured_sensor(
        &self,
        role: Role,
        state: &SampleState,
        now: f64,
        stable_angle_degrees: f32,
    ) -> Result<(), CoreError> {
        if !fresh(
            state.latest.received_at_secs,
            now,
            self.config.stale_after_secs,
        ) {
            return Err(CoreError(format!("{role:?} data is stale")));
        }
        let window: Vec<_> = state
            .history
            .iter()
            .filter(|s| s.received_at_secs >= now - self.config.stable_window_secs)
            .collect();
        if window.len() < 3
            || window.last().unwrap().received_at_secs - window.first().unwrap().received_at_secs
                < self.config.stable_window_secs * 0.85
        {
            return Err(CoreError(format!("Waiting for stable {role:?} history")));
        }
        if window.windows(2).any(|pair| {
            pair[1].received_at_secs - pair[0].received_at_secs
                > self.config.stale_after_secs.min(0.15)
        }) {
            return Err(CoreError(format!(
                "{role:?} sample gaps interrupted stable history"
            )));
        }
        let reference = Quat::from_array(window[0].orientation_xyzw);
        if window.iter().any(|s| {
            angle(reference, Quat::from_array(s.orientation_xyzw))
                > stable_angle_degrees.to_radians()
                || s.acceleration_mps2.is_some_and(|a| {
                    Vec3::from_array(a).length() > self.config.stable_acceleration_mps2
                })
        }) {
            return Err(CoreError(format!(
                "{role:?} is moving; wait for the tap and body motion to settle"
            )));
        }
        Ok(())
    }

    fn try_recovery(&mut self, now: f64, anchors: &Anchors) {
        let RecoveryState::Pending {
            requested_at_secs, ..
        } = self.recovery
        else {
            return;
        };
        if now - requested_at_secs < 0.40 {
            return;
        }
        if !self.direction_calibrated() || self.preview_calibration {
            self.recovery = RecoveryState::Rejected {
                at_secs: now,
                reason:
                    "Live neutral and forward-direction calibration are required before recovery"
                        .into(),
            };
            return;
        }
        let gate = match self.neutral_gate(now, anchors, false, GatePurpose::Recovery) {
            Ok(gate) => gate,
            Err(error) => {
                self.recovery = if now - requested_at_secs >= self.config.recovery_timeout_secs {
                    RecoveryState::Rejected {
                        at_secs: now,
                        reason: error.0,
                    }
                } else {
                    RecoveryState::Pending {
                        requested_at_secs,
                        blocked_reason: Some(error.0),
                    }
                };
                return;
            }
        };
        let heading = yaw_of(checked_quat(anchors.hmd.as_ref().unwrap().orientation_xyzw).unwrap());
        // Recovery and every calibration gate use the fully corrected rotation
        // (heading alignment + dynamic yaw + stance correction), i.e. exactly
        // the published heading. The delta is stored in `yaw` only and the
        // stance correction is kept as it is (not folded into `yaw`), so
        // nothing is counted twice: right after recovery the published heading
        // equals the HMD heading, and `undo_recovery` subtracting the same
        // deltas restores the published headings exactly. The filter's
        // covariance returns to its initial value (spec: after calibration or
        // recovery) and its keyframes, contacts and IK anchors are dropped
        // because the yaw underneath them just changed.
        // Only roles whose evidence the gate validated receive a new heading.
        let deltas: BTreeMap<Role, f32> = gate
            .validated
            .iter()
            .map(|role| {
                (
                    *role,
                    wrap_angle(heading - yaw_of(self.segment(*role, now).rotation)),
                )
            })
            .collect();
        for (role, delta) in &deltas {
            let yaw = self.yaw.entry(*role).or_insert(0.0);
            *yaw = wrap_angle(*yaw + delta);
        }
        self.recovery_undo = Some(deltas);
        // The neutral capture stays usable for manual paired edits, but its
        // heading no longer matches the yaw state for a skiing-pose solve.
        self.forward_requires_new_neutral = true;
        self.knee_history.clear();
        let kept = self.stance.corrections();
        self.stance.set_corrections(kept);
        self.contact.reset();
        self.leg_ik.reset();
        self.recovery = RecoveryState::Applied { at_secs: now };
    }

    /// The segment from the latest raw measured sample. Calibration, stability
    /// gates and recovery use only this, never an extrapolated orientation.
    /// Like every output path it includes the stance correction, so gates see
    /// the published heading.
    fn segment(&self, role: Role, now: f64) -> Segment {
        self.segment_at(role, now, false)
    }

    /// With `predict`, prediction enabled and a fresh measured (not held)
    /// sample, the orientation is extrapolated along the measured angular rate.
    fn segment_at(&self, role: Role, now: f64, predict: bool) -> Segment {
        let Some(id) = self.assignments.get(&role) else {
            return Segment::default();
        };
        let Some(state) = self.samples.get(id) else {
            return Segment::default();
        };
        let mount = self.mounting.get(&role).copied().unwrap_or(Quat::IDENTITY);
        let measured_fresh = fresh(
            state.latest.received_at_secs,
            now,
            self.config.stale_after_secs,
        );
        let latest = Quat::from_array(state.latest.orientation_xyzw);
        // Sensor-world angular rate; zero when silent or without two samples.
        let world_rate = if measured_fresh && !state.sensor_unhealthy {
            state.rate.angular_velocity(now)
        } else {
            Vec3::ZERO
        };
        let estimator = &self.config.estimator;
        let predicted = (predict && estimator.enable_prediction && world_rate != Vec3::ZERO)
            .then(|| {
                state
                    .rate
                    .extrapolate(now, f64::from(estimator.prediction_lead_secs))
            })
            .flatten()
            .filter(|q| q.is_finite());
        // Total left world yaw: initial alignment + dynamic (recovery, legacy
        // knee) yaw + stance-filter correction. Rotations about +Y commute.
        let world_yaw = Quat::from_rotation_y(
            self.heading_alignment.get(&role).copied().unwrap_or(0.0)
                + self.yaw.get(&role).copied().unwrap_or(0.0)
                + self.stance_correction(role),
        );
        let rotation = (world_yaw * predicted.unwrap_or(latest) * mount).normalize();
        // The rate is a left (reference-frame) increment: only the left world
        // yaw rotates it; the right-hand mounting factor does not.
        let angular_velocity = world_yaw * world_rate;
        let angular_velocity = if angular_velocity.is_finite() {
            angular_velocity
        } else {
            Vec3::ZERO
        };
        let held = !measured_fresh && held_evidence(state, now).is_some();
        let valid = (measured_fresh || held)
            && !state.sensor_unhealthy
            && self.mounting.contains_key(&role)
            && (Role::CORE.contains(&role)
                || (self.direction_session_valid && self.functional_roles.contains(&role)));
        let age = (now - state.latest.received_at_secs).max(0.0) as f32;
        let confidence = if valid {
            if held {
                let transport_age =
                    (now - state.hold.as_ref().unwrap().last_transport_at_secs) as f32;
                FIRMWARE_HOLD_CONFIDENCE_CAP
                    * (1.0 - 0.5 * transport_age / FIRMWARE_HOLD_LEASE_SECS as f32).clamp(0.0, 1.0)
            } else {
                (1.0 - 0.5 * age / self.config.stale_after_secs as f32).clamp(0.0, 1.0)
            }
        } else {
            0.0
        };
        Segment {
            rotation,
            confidence,
            valid,
            held,
            angular_velocity: if valid && measured_fresh {
                angular_velocity
            } else {
                Vec3::ZERO
            },
            extrapolated: predicted.is_some() && measured_fresh,
        }
    }

    /// Output segments, extrapolated when prediction is enabled.
    fn compute_segments(&self, now: f64) -> BTreeMap<Role, Segment> {
        Role::ALL
            .into_iter()
            .map(|r| (r, self.segment_at(r, now, true)))
            .collect()
    }

    fn correct_knee(
        &mut self,
        now: f64,
        dt: f32,
        thigh: Role,
        calf: Role,
        segments: &mut BTreeMap<Role, Segment>,
    ) -> CorrectionDiagnostic {
        let upper = segments[&thigh];
        let lower = segments[&calf];
        let mut diagnostic = CorrectionDiagnostic {
            thigh_role: thigh,
            calf_role: calf,
            status: "disabled".into(),
            residual_degrees: 0.0,
            applied_relative_degrees: 0.0,
            confidence: 0.0,
        };
        if !self.config.enable_joint_yaw_correction {
            return diagnostic;
        }
        if self.config.estimator.enable_stance_heading {
            // The stance filter has its own (uncertainty-weighted) knee prior;
            // two independent writers of the same yaw would fight.
            diagnostic.status = "superseded_by_stance_filter".into();
            self.knee_history.remove(&thigh);
            return diagnostic;
        }
        if !self.direction_calibrated() || self.preview_calibration {
            diagnostic.status = "live_calibration_required".into();
            return diagnostic;
        }
        if !upper.valid || !lower.valid {
            diagnostic.status = "stale_or_missing_inputs".into();
            return diagnostic;
        }
        if upper.held || lower.held {
            diagnostic.status = "firmware_held_input_is_not_new_motion_evidence".into();
            self.knee_history.remove(&thigh);
            return diagnostic;
        }
        if [thigh, calf].iter().any(|role| {
            self.assignments
                .get(role)
                .and_then(|id| self.samples.get(id))
                .and_then(|state| state.latest.acceleration_mps2)
                .is_some_and(|a| Vec3::from_array(a).length() > 6.0)
        }) {
            diagnostic.status = "high_acceleration_constraint_unreliable".into();
            return diagnostic;
        }
        let flex = (upper.rotation * Vec3::Y)
            .dot(lower.rotation * Vec3::Y)
            .clamp(-1.0, 1.0)
            .acos();
        let history = self.knee_history.entry(thigh).or_default();
        if history.back().is_none_or(|(time, _)| now > *time) {
            history.push_back((now, flex));
        }
        while history.front().is_some_and(|(time, _)| now - time > 2.0) || history.len() > 512 {
            history.pop_front();
        }
        let excursion = history
            .iter()
            .map(|(_, f)| *f)
            .fold(f32::NEG_INFINITY, f32::max)
            - history
                .iter()
                .map(|(_, f)| *f)
                .fold(f32::INFINITY, f32::min);
        if flex < 20.0_f32.to_radians() || flex > 120.0_f32.to_radians() {
            diagnostic.status = "insufficient_or_extreme_knee_flexion".into();
            return diagnostic;
        }
        if excursion < 8.0_f32.to_radians() || history.len() < 8 {
            diagnostic.status = "insufficient_recent_flexion_excitation".into();
            return diagnostic;
        }
        let upper_axis = upper.rotation * Vec3::X;
        let lower_axis = lower.rotation * Vec3::X;
        let upper_horizontal = upper_axis.x * upper_axis.x + upper_axis.z * upper_axis.z;
        let lower_horizontal = lower_axis.x * lower_axis.x + lower_axis.z * lower_axis.z;
        if upper_horizontal < 0.5 || lower_horizontal < 0.5 {
            diagnostic.status = "poor_joint_axis_geometry".into();
            return diagnostic;
        }
        let residual =
            wrap_angle((-lower_axis.z).atan2(lower_axis.x) - (-upper_axis.z).atan2(upper_axis.x));
        diagnostic.residual_degrees = residual.to_degrees();
        if residual.abs() > self.config.joint_yaw_max_residual_degrees.to_radians() {
            diagnostic.status = "large_residual_mounting_or_pose_ambiguous".into();
            return diagnostic;
        }
        let outside_deadband = residual.abs() - self.config.joint_yaw_deadband_degrees.to_radians();
        if outside_deadband <= 0.0 {
            diagnostic.status = "within_anatomical_deadband".into();
            return diagnostic;
        }
        // This experimental prior is invariant to a common global yaw. It does
        // not select which sensor drifted, and cannot correct common-mode yaw.
        let confidence =
            upper.confidence.min(lower.confidence) * upper_horizontal.min(lower_horizontal);
        let step = residual.signum()
            * outside_deadband.min(
                self.config.yaw_correction_rate_degrees_per_sec.to_radians() * dt * confidence,
            );
        // The budget bounds only this experimental correction's own relative
        // contribution (half per segment), never recovery offsets in `yaw`.
        let accumulated = self.knee_correction.get(&thigh).copied().unwrap_or(0.0) + step;
        if (accumulated * 0.5).abs() > KNEE_CORRECTION_BUDGET_DEGREES.to_radians() {
            diagnostic.status = "correction_budget_exhausted".into();
            return diagnostic;
        }
        let upper_yaw = self.yaw.get(&thigh).copied().unwrap_or(0.0) + step * 0.5;
        let lower_yaw = self.yaw.get(&calf).copied().unwrap_or(0.0) - step * 0.5;
        self.knee_correction.insert(thigh, accumulated);
        self.yaw.insert(thigh, upper_yaw);
        self.yaw.insert(calf, lower_yaw);
        segments.insert(
            thigh,
            Segment {
                rotation: (Quat::from_rotation_y(step * 0.5) * upper.rotation).normalize(),
                ..upper
            },
        );
        segments.insert(
            calf,
            Segment {
                rotation: (Quat::from_rotation_y(-step * 0.5) * lower.rotation).normalize(),
                ..lower
            },
        );
        diagnostic.status = if step.abs() > 0.0 {
            "experimental_relative_correction".into()
        } else {
            "waiting_for_next_frame".into()
        };
        diagnostic.applied_relative_degrees = step.to_degrees();
        diagnostic.confidence = confidence;
        diagnostic
    }

    /// Publishes `body_chain` (the shared forward kinematics from `solve`),
    /// with leg/torso positions from `ik` when leg IK ran. Validity and
    /// confidence depend only on the segments and the live anchor, never on IK.
    #[allow(clippy::too_many_arguments)]
    fn build_frame(
        &self,
        now: f64,
        anchors: &Anchors,
        segments: &BTreeMap<Role, Segment>,
        corrections: Vec<CorrectionDiagnostic>,
        body_chain: &chain::BodyChain,
        ik: Option<&leg_ik::LegIkOutput>,
        stance_applied: &[f32; 6],
    ) -> TrackingFrame {
        let body = &self.config.anthropometrics;
        let live_head = usable_anchor(anchors.hmd.as_ref(), now, self.config.stale_after_secs);
        let (_, head_position, _) = self.head_pose(anchors, now);
        let live = live_head.is_some() && self.direction_calibrated() && !self.preview_calibration;
        let chest = segments[&Role::Chest];
        let hip = segments[&Role::Hip];
        // The HMD reports the eye point; the neck pivot is below and BEHIND it
        // (+Z is backward in the head frame), so head pitch/yaw alone does not
        // translate the inferred body. Leg IK never moves the neck.
        let neck_position = body_chain.neck;
        let (chest_position, hip_position) =
            ik.map_or((body_chain.chest, body_chain.hip), |ik| (ik.chest, ik.hip));
        let chest_valid = live && chest.valid;
        let hip_valid = chest_valid && hip.valid;
        let chest_confidence = if chest_valid {
            chest.confidence * 0.85
        } else {
            0.0
        };
        let hip_confidence = if hip_valid {
            chest.confidence.min(hip.confidence) * 0.80
        } else {
            0.0
        };
        let mut joints = vec![
            joint(
                JointRole::Head,
                head_position,
                if live_head.is_some() { 1.0 } else { 0.0 },
                live_head.is_some(),
            ),
            joint(
                JointRole::Neck,
                neck_position,
                if live { 0.9 } else { 0.0 },
                live,
            ),
            joint(
                JointRole::Chest,
                chest_position,
                chest_confidence,
                chest_valid,
            ),
            joint(JointRole::Hip, hip_position, hip_confidence, hip_valid),
        ];
        let mut bones = vec![
            bone(JointRole::Head, JointRole::Neck),
            bone(JointRole::Neck, JointRole::Chest),
            bone(JointRole::Chest, JointRole::Hip),
        ];
        let mut targets = vec![
            target(
                TargetRole::Chest,
                chest_position,
                &chest,
                chest.rotation,
                orientation_evidence(chest),
                chest_confidence,
                chest_valid,
            ),
            target(
                TargetRole::Waist,
                hip_position,
                &hip,
                hip.rotation,
                orientation_evidence(hip),
                hip_confidence,
                hip_valid,
            ),
        ];
        for left in [true, false] {
            let (
                thigh_role,
                calf_role,
                foot_role,
                hip_joint,
                knee_joint,
                ankle_joint,
                toe_joint,
                knee_target,
                foot_target,
                side,
            ) = if left {
                (
                    Role::LeftThigh,
                    Role::LeftCalf,
                    Role::LeftFoot,
                    JointRole::LeftHip,
                    JointRole::LeftKnee,
                    JointRole::LeftAnkle,
                    JointRole::LeftToe,
                    TargetRole::LeftKnee,
                    TargetRole::LeftFoot,
                    0,
                )
            } else {
                (
                    Role::RightThigh,
                    Role::RightCalf,
                    Role::RightFoot,
                    JointRole::RightHip,
                    JointRole::RightKnee,
                    JointRole::RightAnkle,
                    JointRole::RightToe,
                    TargetRole::RightKnee,
                    TargetRole::RightFoot,
                    1,
                )
            };
            let thigh = segments[&thigh_role];
            let calf = segments[&calf_role];
            let foot = segments[&foot_role];
            let (thigh_root, knee_position, ankle_position) = match ik {
                Some(ik) => (ik.hip_joints[side], ik.knees[side], ik.ankles[side]),
                None => (
                    body_chain.hip_joints[side],
                    body_chain.knees[side],
                    body_chain.ankles[side],
                ),
            };
            let knee_valid = hip_valid && thigh.valid;
            let ankle_valid = knee_valid && calf.valid;
            let knee_confidence = if knee_valid {
                hip_confidence.min(thigh.confidence) * 0.95
            } else {
                0.0
            };
            let ankle_confidence = if ankle_valid {
                knee_confidence.min(calf.confidence) * 0.95
            } else {
                0.0
            };
            // Missing foot IMUs leave ankle flexion unmeasured. A neutral ankle
            // relative to the calf is a visible prior, including when lying;
            // no world-horizontal rotation or floor lock is imposed.
            // With leg IK an un-instrumented planted foot blends toward flat,
            // heading only; its published angular velocity is the calf rate
            // scaled by the remaining rigid-with-calf weight. A measured,
            // direction-certified foot IMU always wins.
            let has_foot = self.direction_certified_optional(foot_role);
            let (foot_owner, foot_rotation) = match ik {
                _ if has_foot => (foot, foot.rotation),
                Some(ik) => (
                    Segment {
                        angular_velocity: calf.angular_velocity
                            * (1.0 - ik.flat_weight[side]).clamp(0.0, 1.0),
                        ..calf
                    },
                    ik.foot_rotations[side],
                ),
                None => (calf, calf.rotation),
            };
            let foot_valid = ankle_valid && (!has_foot || foot.valid);
            let foot_confidence = if foot_valid {
                if has_foot {
                    ankle_confidence.min(foot.confidence)
                } else {
                    ankle_confidence * 0.60
                }
            } else {
                0.0
            };
            let foot_position = ankle_position
                + foot_rotation
                    * Vec3::new(0.0, -body.ankle_to_sole_m * 0.5, -body.foot_length_m * 0.25);
            let toe_position = ankle_position
                + foot_rotation * Vec3::new(0.0, -body.ankle_to_sole_m, -body.foot_length_m * 0.75);
            joints.extend([
                joint(hip_joint, thigh_root, hip_confidence, hip_valid),
                joint(knee_joint, knee_position, knee_confidence, knee_valid),
                joint(ankle_joint, ankle_position, ankle_confidence, ankle_valid),
                joint(toe_joint, toe_position, foot_confidence, foot_valid),
            ]);
            bones.extend([
                bone(JointRole::Hip, hip_joint),
                bone(hip_joint, knee_joint),
                bone(knee_joint, ankle_joint),
                bone(ankle_joint, toe_joint),
            ]);
            targets.push(target(
                knee_target,
                knee_position,
                &thigh,
                thigh.rotation,
                orientation_evidence(thigh),
                knee_confidence,
                knee_valid,
            ));
            targets.push(target(
                foot_target,
                foot_position,
                &foot_owner,
                foot_rotation,
                if has_foot {
                    orientation_evidence(foot)
                } else {
                    Evidence::Inferred
                },
                foot_confidence,
                foot_valid,
            ));
        }
        for left in [true, false] {
            let (arm_role, shoulder_joint, elbow_joint, hand_joint, elbow_target, hand, sign) =
                if left {
                    (
                        Role::LeftUpperArm,
                        JointRole::LeftShoulder,
                        JointRole::LeftElbow,
                        JointRole::LeftHand,
                        TargetRole::LeftElbow,
                        anchors.left_hand.as_ref(),
                        -1.0,
                    )
                } else {
                    (
                        Role::RightUpperArm,
                        JointRole::RightShoulder,
                        JointRole::RightElbow,
                        JointRole::RightHand,
                        TargetRole::RightElbow,
                        anchors.right_hand.as_ref(),
                        1.0,
                    )
                };
            let shoulder = neck_position
                + chest.rotation * Vec3::new(sign * body.shoulder_width_m * 0.5, -0.035, 0.0);
            let arm = segments[&arm_role];
            let measured_arm = self.assignments.contains_key(&arm_role)
                && self.direction_session_valid
                && self.functional_roles.contains(&arm_role);
            let hand_anchor = usable_anchor(hand, now, self.config.stale_after_secs);
            let hand_position = hand_anchor.map(|h| Vec3::from_array(h.position));
            let (elbow, elbow_confidence, elbow_valid) = if measured_arm {
                let valid = chest_valid && arm.valid;
                (
                    shoulder - arm.rotation * Vec3::Y * body.upper_arm_length_m,
                    if valid {
                        chest_confidence.min(arm.confidence) * 0.9
                    } else {
                        0.0
                    },
                    valid,
                )
            } else if let Some(wrist) = hand_position {
                let direction = wrist - shoulder;
                let distance = direction.length();
                let reachable = distance >= (body.upper_arm_length_m - body.forearm_length_m).abs()
                    && distance <= body.upper_arm_length_m + body.forearm_length_m;
                (
                    infer_elbow(
                        shoulder,
                        wrist,
                        chest.rotation,
                        sign,
                        body.upper_arm_length_m,
                        body.forearm_length_m,
                    ),
                    if chest_valid {
                        chest_confidence * if reachable { 0.45 } else { 0.20 }
                    } else {
                        0.0
                    },
                    chest_valid,
                )
            } else {
                (
                    shoulder - chest.rotation * Vec3::Y * body.upper_arm_length_m,
                    0.0,
                    false,
                )
            };
            let wrist =
                hand_position.unwrap_or(elbow - chest.rotation * Vec3::Y * body.forearm_length_m);
            joints.extend([
                joint(shoulder_joint, shoulder, chest_confidence, chest_valid),
                joint(elbow_joint, elbow, elbow_confidence, elbow_valid),
                joint(
                    hand_joint,
                    wrist,
                    if hand_anchor.is_some() { 1.0 } else { 0.0 },
                    hand_anchor.is_some(),
                ),
            ]);
            bones.extend([
                bone(JointRole::Neck, shoulder_joint),
                bone(shoulder_joint, elbow_joint),
                bone(elbow_joint, hand_joint),
            ]);
            // Publish elbow targets only when an upper-arm IMU is assigned.
            // Controller-only elbow swivel remains a preview inference.
            if measured_arm {
                targets.push(target(
                    elbow_target,
                    elbow,
                    &arm,
                    arm.rotation,
                    orientation_evidence(arm),
                    elbow_confidence,
                    elbow_valid,
                ));
            }
        }
        TrackingFrame {
            at_secs: now,
            targets,
            joints,
            bones,
            segments: self
                .assignments
                .keys()
                .map(|role| {
                    let segment = segments[role];
                    let measurement_received_at_secs = self
                        .assignments
                        .get(role)
                        .and_then(|id| self.samples.get(id))
                        .map(|state| state.latest.received_at_secs);
                    // Dynamic yaw plus the stance correction applied to this
                    // frame: every left world yaw beyond the initial alignment.
                    let stance = Role::CORE
                        .iter()
                        .position(|core| core == role)
                        .map_or(0.0, |i| stance_applied[i]);
                    SegmentPose {
                        role: *role,
                        orientation_xyzw: segment.rotation.to_array(),
                        yaw_correction_degrees: (self.yaw.get(role).copied().unwrap_or(0.0)
                            + stance)
                            .to_degrees(),
                        confidence: if self.calibrated {
                            segment.confidence
                        } else {
                            0.0
                        },
                        valid: self.calibrated && segment.valid,
                        measurement_received_at_secs,
                        measurement_age_secs: measurement_received_at_secs
                            .map(|time| (now - time).max(0.0)),
                        uses_firmware_hold: segment.held,
                    }
                })
                .collect(),
            calibration: CalibrationStatus {
                calibrated: self.calibrated,
                preview_only: self.preview_calibration,
                requires_session_calibration: !self.direction_calibrated() || self.restored_profile,
                reason: self.calibration_reason.clone(),
                used_firmware_hold: self.calibration_used_firmware_hold,
                direction_calibrated: self.direction_calibrated(),
                reference_forward: self.calibration_forward(),
                floor_mismatch_m: self.floor_mismatch_m,
            },
            recovery: self.recovery.clone(),
            corrections,
            live_anchor: live_head.is_some(),
            preview_only: !live,
            preview_motion: preview_motion::PreviewMotionStatus::default(),
            estimator: EstimatorStatus::default(),
            motion_calibration: None,
        }
    }
}

impl Anthropometrics {
    /// Eye-to-sole height of the modeled upright vertical chain. A consistent
    /// profile has this equal to `standing_eye_height_m`.
    pub fn standing_chain_height_m(&self) -> f32 {
        self.head_to_neck_m
            + self.neck_to_chest_m
            + self.chest_to_hip_m
            + self.thigh_length_m
            + self.calf_length_m
            + self.ankle_to_sole_m
    }

    /// Makes the profile consistent by adjusting `chest_to_hip_m` (the least
    /// constrained length of the modelled chain) so that
    /// `standing_chain_height_m()` equals `standing_eye_height_m`.
    ///
    /// Returns the change of `chest_to_hip_m` in meters (zero when already within
    /// 0.1 mm). When the required value is outside the valid chest-to-hip range,
    /// nothing changes and the error names the mismatch.
    pub fn normalize_chain_height(&mut self) -> Result<f32, CoreError> {
        let excess = self.standing_chain_height_m() - self.standing_eye_height_m;
        if !excess.is_finite() {
            return Err(CoreError("Body dimensions are not finite".into()));
        }
        if excess.abs() < 1.0e-4 {
            return Ok(0.0);
        }
        let mut next = self.clone();
        next.chest_to_hip_m -= excess;
        if next.validate().is_err() {
            return Err(CoreError(format!(
                "The modelled standing height differs from the eye height by {:.1} cm and \
                 chest-to-hip cannot absorb it ({:.3} m would be outside 0.10–0.8 m)",
                excess * 100.0,
                next.chest_to_hip_m
            )));
        }
        *self = next;
        Ok(-excess)
    }

    pub fn validate(&self) -> Result<(), CoreError> {
        for (name, value, minimum, maximum) in [
            ("standing eye height", self.standing_eye_height_m, 0.8, 2.5),
            ("head to neck", self.head_to_neck_m, 0.03, 0.35),
            ("neck to chest", self.neck_to_chest_m, 0.05, 0.45),
            ("chest to hip", self.chest_to_hip_m, 0.10, 0.8),
            ("hip width", self.hip_width_m, 0.10, 0.65),
            ("shoulder width", self.shoulder_width_m, 0.15, 0.75),
            ("thigh length", self.thigh_length_m, 0.15, 0.85),
            ("calf length", self.calf_length_m, 0.15, 0.85),
            ("foot length", self.foot_length_m, 0.08, 0.45),
            ("ankle to sole", self.ankle_to_sole_m, 0.02, 0.20),
            ("upper arm length", self.upper_arm_length_m, 0.10, 0.60),
            ("forearm length", self.forearm_length_m, 0.10, 0.60),
            ("eye forward offset", self.eye_forward_m, 0.0, 0.20),
        ] {
            if !value.is_finite() || value < minimum || value > maximum {
                return Err(CoreError(format!(
                    "Invalid {name}: expected {minimum}–{maximum} m"
                )));
            }
        }
        Ok(())
    }
}

impl TrackingConfig {
    pub fn validate(&self) -> Result<(), CoreError> {
        self.anthropometrics.validate()?;
        if !self.stale_after_secs.is_finite()
            || !(0.03..=2.0).contains(&self.stale_after_secs)
            || !self.stable_window_secs.is_finite()
            || !(0.25..=5.0).contains(&self.stable_window_secs)
            || !self.recovery_timeout_secs.is_finite()
            || !(1.0..=30.0).contains(&self.recovery_timeout_secs)
        {
            return Err(CoreError("Invalid timing configuration".into()));
        }
        for (name, value, low, high) in [
            ("stable angle", self.stable_angle_degrees, 0.2, 5.0),
            (
                "stable acceleration",
                self.stable_acceleration_mps2,
                0.1,
                4.0,
            ),
            ("neutral tilt", self.neutral_tilt_degrees, 5.0, 30.0),
            (
                "height tolerance",
                self.standing_height_tolerance_m,
                0.03,
                0.30,
            ),
            ("recovery yaw", self.recovery_max_yaw_degrees, 2.0, 30.0),
            ("joint deadband", self.joint_yaw_deadband_degrees, 2.0, 15.0),
            (
                "joint maximum residual",
                self.joint_yaw_max_residual_degrees,
                5.0,
                40.0,
            ),
            (
                "correction rate",
                self.yaw_correction_rate_degrees_per_sec,
                0.01,
                1.0,
            ),
        ] {
            if !value.is_finite() || value < low || value > high {
                return Err(CoreError(format!("Invalid {name}")));
            }
        }
        if self.joint_yaw_max_residual_degrees <= self.joint_yaw_deadband_degrees {
            return Err(CoreError(
                "Joint residual limit must exceed deadband".into(),
            ));
        }
        self.estimator.validate()
    }
}

fn validated_assignments(
    assignments: &[RoleAssignment],
) -> Result<BTreeMap<Role, String>, CoreError> {
    let mut result = BTreeMap::new();
    let mut ids = BTreeSet::new();
    for assignment in assignments {
        validate_id(&assignment.sensor_id)?;
        if !ids.insert(assignment.sensor_id.clone())
            || result
                .insert(assignment.role, assignment.sensor_id.clone())
                .is_some()
        {
            return Err(CoreError("Sensor IDs and roles must each be unique".into()));
        }
    }
    Ok(result)
}

fn validate_id(id: &str) -> Result<(), CoreError> {
    if id.trim().is_empty() || id.len() > 256 {
        Err(CoreError("Invalid sensor identity".into()))
    } else {
        Ok(())
    }
}
fn finite3(v: [f32; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn held_evidence(state: &SampleState, now: f64) -> Option<&FirmwareHoldState> {
    state.hold.as_ref().filter(|hold| {
        !state.hold_revoked
            && !state.sensor_unhealthy
            && now.is_finite()
            && now >= hold.observed_at_secs
            && now >= hold.last_transport_at_secs
            && now - hold.last_transport_at_secs < FIRMWARE_HOLD_LEASE_SECS
    })
}

fn quaternion_components_match(left: [f32; 4], right: [f32; 4]) -> bool {
    left.iter()
        .zip(right)
        .all(|(a, b)| (*a - b).abs() <= 1.0e-6)
        || left
            .iter()
            .zip(right)
            .all(|(a, b)| (*a + b).abs() <= 1.0e-6)
}

fn orientation_evidence(segment: Segment) -> Evidence {
    if segment.held {
        Evidence::FirmwareHeldOrientation
    } else {
        Evidence::MeasuredOrientation
    }
}
fn checked_quat(values: [f32; 4]) -> Result<Quat, CoreError> {
    let q = Quat::from_array(values);
    if !q.is_finite() || !q.length_squared().is_finite() || q.length_squared() < 1.0e-8 {
        Err(CoreError("Invalid quaternion".into()))
    } else {
        Ok(q.normalize())
    }
}
fn fresh(time: f64, now: f64, limit: f64) -> bool {
    time.is_finite() && now.is_finite() && time >= 0.0 && now >= time && now - time <= limit
}
fn usable_anchor(anchor: Option<&AnchorPose>, now: f64, limit: f64) -> Option<&AnchorPose> {
    anchor.filter(|a| {
        a.valid && fresh(a.received_at_secs, now, limit)
        // Even finite floating point extremes can overflow IK vector arithmetic.
        && finite3(a.position) && a.position.iter().all(|v| v.abs() <= 10_000.0)
        && checked_quat(a.orientation_xyzw).is_ok()
    })
}
fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos()
}
fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}
fn yaw_of(q: Quat) -> f32 {
    let forward = q * Vec3::NEG_Z;
    (-forward.x).atan2(-forward.z)
}
fn average_quaternions(quats: impl Iterator<Item = Quat>) -> Quat {
    let mut sum = [0.0_f64; 4];
    let mut reference = None;
    for q in quats {
        let first = *reference.get_or_insert(q);
        let sign = if first.dot(q) < 0.0 { -1.0 } else { 1.0 };
        for (total, value) in sum.iter_mut().zip(q.to_array()) {
            *total += value as f64 * sign;
        }
    }
    checked_quat(sum.map(|v| v as f32)).unwrap_or(Quat::IDENTITY)
}

fn joint(role: JointRole, position: Vec3, confidence: f32, valid: bool) -> JointPose {
    JointPose {
        joint: role,
        position: position.to_array(),
        confidence,
        valid,
    }
}
fn bone(start: JointRole, end: JointRole) -> Bone {
    Bone { start, end }
}
fn target(
    role: TargetRole,
    position: Vec3,
    owner: &Segment,
    rotation: Quat,
    orientation_source: Evidence,
    confidence: f32,
    valid: bool,
) -> TargetPose {
    let angular_velocity = if valid && owner.angular_velocity.is_finite() {
        owner.angular_velocity.to_array()
    } else {
        [0.0; 3]
    };
    TargetPose {
        linear_velocity: [0.0; 3],
        angular_velocity,
        role,
        position: position.to_array(),
        orientation_xyzw: rotation.to_array(),
        position_source: Evidence::Inferred,
        orientation_source,
        confidence,
        valid,
        reason: if valid {
            None
        } else {
            Some(
                "Requires neutral and forward calibration, fresh sensors, and a live HMD anchor"
                    .into(),
            )
        },
    }
}
fn infer_elbow(
    shoulder: Vec3,
    wrist: Vec3,
    chest: Quat,
    side: f32,
    upper: f32,
    lower: f32,
) -> Vec3 {
    let delta = wrist - shoulder;
    let distance = delta.length();
    let direction = if distance > 1.0e-5 {
        delta / distance
    } else {
        chest * Vec3::NEG_Y
    };
    let d = distance.clamp((upper - lower).abs() + 0.001, upper + lower - 0.001);
    let along = (upper * upper - lower * lower + d * d) / (2.0 * d);
    let height = (upper * upper - along * along).max(0.0).sqrt();
    let preferred = chest * Vec3::new(side * 0.6, -0.8, 0.25);
    let mut bend = preferred - direction * preferred.dot(direction);
    if bend.length_squared() < 1.0e-6 {
        let fallback = if direction.dot(Vec3::Z).abs() < 0.9 {
            Vec3::Z
        } else {
            Vec3::X
        };
        bend = fallback - direction * fallback.dot(direction);
    }
    shoulder + direction * along + bend.normalize() * height
}
