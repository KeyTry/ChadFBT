//! UI/runtime boundary. UI never owns network sockets or mutates solver state.
use serde::{Deserialize, Serialize};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    time::Instant,
};

pub const ROLES: &[(&str, &str)] = &[
    ("chest", "Chest"),
    ("hip", "Hip"),
    ("left_thigh", "Left thigh"),
    ("right_thigh", "Right thigh"),
    ("left_calf", "Left calf"),
    ("right_calf", "Right calf"),
    ("left_foot", "Left foot"),
    ("right_foot", "Right foot"),
    ("left_upper_arm", "Left upper arm"),
    ("right_upper_arm", "Right upper arm"),
];

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TrackerView {
    pub id: String,
    pub label: String,
    pub firmware: String,
    pub role: Option<String>,
    pub orientation: [f32; 4],
    pub acceleration: Option<[f32; 3]>,
    pub battery: Option<f32>,
    pub hz: f32,
    pub age_seconds: f32,
    pub packets: u64,
    /// Current eligibility includes clearly labeled experimental firmware-held evidence.
    #[serde(default)]
    pub eligible_for_calibration: bool,
    #[serde(default)]
    pub estimated_held: bool,
    /// A receiver pause/backlog invalidated all cached orientations until a new quaternion.
    #[serde(default)]
    pub transport_invalidated: bool,
    pub last_tap_seconds: Option<f32>,
    /// Time since any accepted datagram from this device; not orientation freshness.
    #[serde(default)]
    pub device_packet_age_seconds: Option<f32>,
    #[serde(default)]
    pub device_packets: u64,
    #[serde(default)]
    pub device_sequence_missing: u64,
    #[serde(default)]
    pub acceleration_age_seconds: Option<f32>,
    #[serde(default)]
    pub acceleration_packets: u64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JointView {
    pub name: String,
    pub position: [f32; 3],
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TargetView {
    pub name: String,
    pub position: [f32; 3],
    pub orientation: [f32; 4],
    pub confidence: f32,
    pub valid: bool,
    pub source: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ManualMountView {
    pub role: String,
    pub forward_degrees: f32,
    pub physical_degrees_xyz: [f32; 3],
    pub confirmed: bool,
    pub can_undo: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationRequestKind {
    #[default]
    NeutralPose,
    ForwardPose,
    StandingHeight,
    /// Neutral capture (unless reused) followed by the movement phase.
    MotionPose,
}

/// Which direction calibration the Setup page offers first; persisted.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationMethod {
    /// Neutral pose, then a bow and knee lifts (or squats) without holding.
    #[default]
    Movement,
    /// Fallback: neutral pose, then the held skiing pose.
    SkiPose,
}

/// User-facing estimator switches, persisted in the app settings. These are
/// the application's defaults; the tracking core's own defaults are all off.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EstimatorSettings {
    /// Extrapolate fresh sensor orientations along their measured angular rate.
    pub prediction: bool,
    /// Extra prediction beyond the solve time, 0–30 ms.
    pub prediction_lead_ms: f32,
    /// Advance headset/controller positions by their reported velocity.
    pub hmd_extrapolation: bool,
    /// Carry calibration over a SteamVR recenter or headset tracking jump.
    pub auto_rebase: bool,
    /// Stance heading filter: estimates per-sensor heading drift from the
    /// displacement of the headset versus the leg chain while a foot is
    /// stationary. Experimental; a diagnostic of model consistency only.
    pub stance_heading: bool,
    /// Contact-aware leg IK: locks planted feet horizontally, corrects the hips
    /// toward them and flattens planted feet under a near-upright shin. Seated
    /// and lying legs keep their measured knee angle. Experimental; contact is
    /// inferred.
    pub leg_ik: bool,
    /// Ask the driver to forward angular velocity to SteamVR (frame unverified).
    pub angular_velocity_output: bool,
}

impl Default for EstimatorSettings {
    fn default() -> Self {
        Self {
            prediction: true,
            prediction_lead_ms: 0.0,
            hmd_extrapolation: true,
            auto_rebase: true,
            stance_heading: true,
            leg_ik: true,
            angular_velocity_output: false,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Snapshot {
    pub trackers: Vec<TrackerView>,
    pub joints: Vec<JointView>,
    pub bones: Vec<[usize; 2]>,
    pub targets: Vec<TargetView>,
    pub messages: Vec<String>,
    pub status: String,
    pub calibrated: bool,
    #[serde(default)]
    pub calibration_request: crate::calibration_request::CalibrationRequestView,
    #[serde(default)]
    pub calibration_request_kind: CalibrationRequestKind,
    /// Current valid standing-space headset height; never a fabricated reference.
    #[serde(default)]
    pub hmd_standing_height_m: Option<f32>,
    /// Fresh horizontal headset facing in standing space, not camera direction.
    #[serde(default)]
    pub hmd_forward: Option<[f32; 3]>,
    /// Reference facing captured with the neutral pose; never follows head turns.
    #[serde(default)]
    pub calibration_forward: Option<[f32; 3]>,
    #[serde(default)]
    pub direction_calibrated: bool,
    #[serde(default)]
    pub manual_mounts: Vec<ManualMountView>,
    #[serde(default)]
    pub manual_mount_message: String,
    #[serde(default)]
    pub desktop_motion_enabled: bool,
    #[serde(default)]
    pub desktop_motion_status: String,
    #[serde(default)]
    pub desktop_root_translation: [f32; 3],
    #[serde(default)]
    pub calibration_height_tolerance_m: f32,
    pub calibration_preview: bool,
    pub hmd_connected: bool,
    pub hands_connected: [bool; 2],
    pub steamvr_connected: bool,
    /// Persisted output mode (Auto, SteamVR, VRChat OSC, Desktop only).
    #[serde(default)]
    pub output_mode: crate::output_mode::OutputMode,
    /// Where valid live targets are being sent right now.
    #[serde(default)]
    pub output_sink: crate::output_mode::OutputSink,
    /// What the output mode is currently doing, e.g. "Auto: live in SteamVR".
    #[serde(default)]
    pub mode_status: String,
    /// Why a desktop calibration is not yet aligned to the headset (empty
    /// when no alignment is pending).
    #[serde(default)]
    pub alignment_status: String,
    /// New work treats the session as desktop: Desktop only, or Auto without
    /// a worn headset. Derived from the mode; never a separate switch.
    #[serde(default)]
    pub desktop_session: bool,
    /// Live neutral captures update the eye height from the headset.
    #[serde(default)]
    pub auto_eye_height: bool,
    pub experimental_correction: bool,
    pub recovery: String,
    pub correction: String,
    pub recording: Option<String>,
    pub uptime_seconds: f64,
    pub solve_micros: f64,
    /// Solves (and SteamVR publishes) completed during the last second.
    #[serde(default)]
    pub solve_rate_hz: f32,
    /// Datagram receipt to solve latency of measured samples over the last second.
    #[serde(default)]
    pub packet_to_solve_p50_ms: Option<f32>,
    #[serde(default)]
    pub packet_to_solve_p95_ms: Option<f32>,
    /// Datagrams dropped because the worker fell behind (visible as sequence gaps).
    #[serde(default)]
    pub receive_dropped: u64,
    pub packets_received: u64,
    pub malformed_packets: u64,
    /// Datagram type counts from validated sources (before sequence rejection;
    /// undocumented types share one bucket), and decoded payload counts after acceptance.
    #[serde(default)]
    pub datagram_types: std::collections::BTreeMap<u32, u64>,
    #[serde(default)]
    pub payload_types: std::collections::BTreeMap<String, u64>,
    #[serde(default)]
    pub sequence_missing: u64,
    #[serde(default)]
    pub sequence_duplicates: u64,
    #[serde(default)]
    pub sequence_out_of_order: u64,
    /// Receiver pauses, socket errors, or polling-budget saturations.
    #[serde(default)]
    pub transport_resets: u64,
    pub profile_path: String,
    pub height_m: f32,
    pub gesture_enabled: bool,
    /// Current estimator switches.
    #[serde(default)]
    pub estimator_settings: EstimatorSettings,
    /// Estimator diagnostics of the latest solved frame.
    #[serde(default)]
    pub estimator: tracking_core::EstimatorStatus,
    /// Headset height at the last live neutral capture minus the body model's
    /// standing chain height. Positive: modelled feet float; negative: they sink.
    #[serde(default)]
    pub floor_mismatch_m: Option<f32>,
    /// Preferred direction calibration method (persisted).
    #[serde(default)]
    pub calibration_method: CalibrationMethod,
    /// Latest movement-calibration status of the current (or last finished)
    /// movement request, sampled at 10 Hz; `None` outside a movement request.
    #[serde(default)]
    pub motion_calibration: Option<tracking_core::motion_calibration::MotionCalibrationStatus>,
    /// A valid neutral capture exists, so movement can start without a new one.
    #[serde(default)]
    pub motion_move_available: bool,
    /// Seconds spent in the current movement phase (zero outside it).
    #[serde(default)]
    pub motion_elapsed_secs: f64,
    /// Spoken/tone feedback setting (persisted).
    #[serde(default)]
    pub audio_mode: crate::audio_cues::AudioMode,
    /// State of Windows speech synthesis, for example unavailable voices.
    #[serde(default)]
    pub voice_status: String,
    /// Last recognised or ignored tap sequence and what it did.
    #[serde(default)]
    pub gesture_status: String,
}

#[derive(Debug, Clone)]
pub enum Command {
    Assign {
        id: String,
        role: Option<String>,
    },
    SetHeight(f32),
    Calibrate,
    CalibrateForward,
    /// Movement calibration: countdown, neutral capture, then movement.
    CalibrateMotion,
    /// Movement calibration reusing the current valid neutral capture.
    CalibrateMotionMove,
    SetCalibrationMethod(CalibrationMethod),
    SetManualMount {
        role: String,
        forward_degrees: f32,
        physical_degrees_xyz: [f32; 3],
    },
    ConfirmManualMount {
        role: String,
    },
    UndoManualMount {
        role: String,
    },
    ResetManualMount {
        role: String,
    },
    CancelCalibration,
    UseCurrentHeadsetHeight,
    TestCalibrationSound,
    RequestRecovery,
    UndoRecovery,
    ClearCalibration,
    SetDesktopMotion(bool),
    SetCorrection(bool),
    /// Hands-free tap gestures on the chest/hip trackers.
    SetGesture(bool),
    SetAudioMode(crate::audio_cues::AudioMode),
    SetEstimator(EstimatorSettings),
    SetOutputMode(crate::output_mode::OutputMode),
    /// Live neutral captures adopt the measured headset height as eye height.
    SetAutoEyeHeight(bool),
    StartRecording,
    StopRecording,
    SaveProfile,
    ExportDiagnostics,
    Shutdown,
}

/// Liveness of the tracking worker, observable without its cooperation.
pub struct WorkerHealth {
    started: Instant,
    beat_millis: AtomicU64,
    finished: AtomicBool,
    failure: Mutex<Option<String>>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum WorkerStatus {
    Running,
    /// No loop iteration for this many seconds.
    Stalled(f64),
    /// The worker ended; `Some` carries the panic message of an unexpected stop.
    Stopped(Option<String>),
}

/// The worker loop never blocks this long; beyond it the snapshot is frozen.
pub const WORKER_STALL_SECS: f64 = 2.0;

impl Default for WorkerHealth {
    fn default() -> Self {
        Self {
            started: Instant::now(),
            beat_millis: AtomicU64::new(0),
            finished: AtomicBool::new(false),
            failure: Mutex::new(None),
        }
    }
}

impl WorkerHealth {
    pub fn beat(&self) {
        let millis = self.started.elapsed().as_millis() as u64;
        self.beat_millis.store(millis, Ordering::Release);
    }
    pub fn finish(&self, failure: Option<String>) {
        *self.failure.lock().unwrap_or_else(|e| e.into_inner()) = failure;
        self.finished.store(true, Ordering::Release);
    }
    pub fn status(&self) -> WorkerStatus {
        if self.finished.load(Ordering::Acquire) {
            return WorkerStatus::Stopped(
                self.failure
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone(),
            );
        }
        let beat = self.beat_millis.load(Ordering::Acquire) as f64 / 1000.;
        let age = self.started.elapsed().as_secs_f64() - beat;
        if age > WORKER_STALL_SECS {
            WorkerStatus::Stalled(age)
        } else {
            WorkerStatus::Running
        }
    }
}

#[derive(Clone)]
pub struct AppHandle {
    pub state: Arc<Mutex<Snapshot>>,
    pub commands: mpsc::Sender<Command>,
    pub health: Arc<WorkerHealth>,
}

impl AppHandle {
    pub fn snapshot(&self) -> Snapshot {
        self.state.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    pub fn send(&self, command: Command) {
        let _ = self.commands.send(command);
    }

    pub fn worker_status(&self) -> WorkerStatus {
        self.health.status()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn worker_health_reports_stop_reason_and_running() {
        let health = WorkerHealth::default();
        health.beat();
        assert_eq!(health.status(), WorkerStatus::Running);
        health.finish(Some("boom".into()));
        assert_eq!(health.status(), WorkerStatus::Stopped(Some("boom".into())));
    }
}
