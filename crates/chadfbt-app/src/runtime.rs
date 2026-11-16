//! Single-owner tracking worker. UI commands are messages; sockets and solver stay here.
use crate::{
    audio_cues::{AudioCues, AudioMode, Cue, Prompt},
    calibration_request::{
        CalibrationRequestController, CalibrationRequestPhase, CalibrationRequestView,
    },
    model::*,
    output_mode::{self, HeadsetPresence, OutputMode, OutputSink, PublishState},
    outputs,
    tap_gesture::{
        GestureAction, TapConfig, TapEvent, TapGesture, TapRecognizer, TapVerdict, gesture_action,
    },
    transport::{TimerResolution, TrackerHub, raise_current_thread_priority},
    voice_prompts,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, VecDeque},
    fs::{self, File},
    io::{self, BufRead, BufReader, BufWriter, Read, Write},
    net::UdpSocket,
    path::{Path, PathBuf},
    process::{Child, Command as ProcessCommand, Stdio},
    sync::{
        Arc, Mutex,
        mpsc::{self, SyncSender, TryRecvError, TrySendError},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tracking_core::{
    AnchorPose, Anchors, Profile, Role, RoleAssignment, SensorSample, TargetRole, TrackingConfig,
    TrackingCore, TrackingFrame,
};

#[derive(Serialize, Deserialize)]
struct Settings {
    profile: Profile,
    #[serde(default)]
    correction: bool,
    /// Hands-free chest/hip tap gestures (name kept for saved profiles).
    #[serde(default)]
    gesture: bool,
    #[serde(default)]
    audio_mode: AudioMode,
    /// Tap recogniser thresholds (editable in the profile; validated on load).
    #[serde(default)]
    tap_gestures: TapConfig,
    #[serde(default = "desktop_motion_default")]
    desktop_motion: bool,
    #[serde(default)]
    estimator: EstimatorSettings,
    #[serde(default)]
    calibration_method: CalibrationMethod,
    /// One output mode, default Auto (replaces the per-launch output selector
    /// and the desktop-preview checkbox).
    #[serde(default)]
    output_mode: OutputMode,
    /// Live neutral captures update the eye height from the headset.
    #[serde(default = "auto_eye_height_default")]
    auto_eye_height: bool,
}

fn desktop_motion_default() -> bool {
    true
}

fn auto_eye_height_default() -> bool {
    true
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub(crate) enum Record {
    Header {
        version: u32,
        config: TrackingConfig,
        profile: Profile,
        note: String,
    },
    /// Every attempted ingest. A rejected ingest still changes core state, so its
    /// outcome is recorded; older recordings (no field) contain accepted samples only.
    Sample {
        sample: SensorSample,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        rejected: Option<String>,
    },
    Anchors {
        at: f64,
        anchors: Anchors,
    },
    Calibrate {
        at: f64,
        preview: bool,
    },
    ForwardCalibrate {
        at: f64,
    },
    /// A movement calibration session started from the current neutral.
    MotionBegin {
        at: f64,
    },
    /// The movement calibration was solved and committed.
    MotionFinish {
        at: f64,
    },
    /// The movement calibration session was discarded by the runtime.
    MotionCancel {
        at: f64,
    },
    ManualMount {
        at: f64,
        role: Role,
        values: tracking_core::manual_mounting::ManualMountValues,
    },
    ConfirmManualMount {
        at: f64,
        role: Role,
    },
    UndoManualMount {
        at: f64,
        role: Role,
    },
    ResetManualMount {
        at: f64,
        role: Role,
    },
    Recovery {
        at: f64,
    },
    RevokeHold {
        at: f64,
        id: String,
        #[serde(default)]
        unhealthy: bool,
    },
    UndoRecovery {
        at: f64,
    },
    Clear {
        at: f64,
    },
    /// The SteamVR standing space was rotated about +Y; calibration carried
    /// over. Written by older builds, which rebased from the transform alone;
    /// still replayed.
    Rebase {
        at: f64,
        yaw_radians: f32,
    },
    /// A possible SteamVR tracking-space event: a new anchor-helper session or
    /// a material raw-to-standing transform change (`transform_yaw_radians`:
    /// that change's yaw when it is a pure yaw). Replayed through
    /// `TrackingCore::note_tracking_space_event`; the core decides from the
    /// recorded headset poses whether the standing space moved.
    TrackingSpaceEvent {
        at: f64,
        #[serde(default)]
        transform_yaw_radians: Option<f32>,
    },
    /// A desktop calibration was carried over to the headset
    /// (`TrackingCore::align_preview_to_headset`).
    AlignToHeadset {
        at: f64,
    },
    Configuration {
        at: f64,
        config: TrackingConfig,
        profile: Profile,
    },
    Frame {
        frame: TrackingFrame,
    },
    Transport {
        at: f64,
        trackers: Vec<TrackerView>,
    },
}

fn data_dir() -> PathBuf {
    std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("local-data"))
        .join("ChadFBT")
}

fn role_name(role: Role) -> String {
    serde_json::to_value(role)
        .unwrap()
        .as_str()
        .unwrap()
        .to_owned()
}
fn role_value(name: &str) -> Option<Role> {
    serde_json::from_value(serde_json::Value::String(name.to_owned())).ok()
}
// Standing origin changes invalidate the calibration frame. Compare against a
// stable reference, rather than consecutive frames, so slow changes accumulate.
fn standing_space_changed(reference: &[f32; 12], current: &[f32; 12]) -> bool {
    reference
        .iter()
        .zip(current)
        .any(|(a, b)| (a - b).abs() > 0.01)
}

fn target_index(role: TargetRole) -> usize {
    match role {
        TargetRole::Waist => 0,
        TargetRole::Chest => 1,
        TargetRole::LeftFoot => 2,
        TargetRole::RightFoot => 3,
        TargetRole::LeftKnee => 4,
        TargetRole::RightKnee => 5,
        TargetRole::LeftElbow => 6,
        TargetRole::RightElbow => 7,
    }
}

/// Yaw (about +Y) of `standing_new_from_standing_old =
/// inverse(raw_from_standing_new) * raw_from_standing_old`, when that rotation
/// is yaw-only within 1 degree.
/// The result is the `TrackingCore::rebase_standing_space` angle: a fixed raw
/// point has new standing coordinates equal to the old ones rotated by it.
/// Translation is ignored because body positions are anchored to the HMD.
fn standing_rebase_yaw(old: &[f32; 12], new: &[f32; 12]) -> Option<f32> {
    if !steamvr_bridge::valid_transform(old) || !steamvr_bridge::valid_transform(new) {
        return None;
    }
    let rotation = |m: &[f32; 12]| {
        glam::DMat3::from_cols(
            glam::DVec3::new(m[0].into(), m[4].into(), m[8].into()),
            glam::DVec3::new(m[1].into(), m[5].into(), m[9].into()),
            glam::DVec3::new(m[2].into(), m[6].into(), m[10].into()),
        )
    };
    // Raw-from-standing rotations are orthonormal: the inverse is the transpose.
    let relative = rotation(new).transpose() * rotation(old);
    let up = relative * glam::DVec3::Y;
    if !up.is_finite() || up.normalize_or_zero().dot(glam::DVec3::Y) < 1.0_f64.to_radians().cos() {
        return None;
    }
    // Forward (-Z) mapped by R_y(yaw) is (-sin yaw, 0, -cos yaw).
    let forward = relative * glam::DVec3::NEG_Z;
    let yaw = (-forward.x).atan2(-forward.z) as f32;
    yaw.is_finite().then_some(yaw)
}

/// Estimator switches applied to a clone of the tracking configuration.
fn config_with_estimator(
    config: &TrackingConfig,
    settings: &EstimatorSettings,
) -> Result<TrackingConfig, String> {
    let lead_secs = settings.prediction_lead_ms / 1000.;
    if !lead_secs.is_finite()
        || !(0.0..=tracking_core::EstimatorConfig::MAX_PREDICTION_LEAD_SECS).contains(&lead_secs)
    {
        return Err("Prediction lead must be between 0 and 30 ms.".into());
    }
    let mut next = config.clone();
    let estimator = &mut next.estimator;
    estimator.enable_prediction = settings.prediction;
    estimator.prediction_lead_secs = lead_secs;
    estimator.enable_hmd_extrapolation = settings.hmd_extrapolation;
    estimator.enable_auto_rebase = settings.auto_rebase;
    estimator.enable_stance_heading = settings.stance_heading;
    estimator.enable_leg_ik = settings.leg_ik;
    next.validate().map_err(|error| error.to_string())?;
    Ok(next)
}

// Convert the target order and quaternion convention consumed by the native receiver.
fn steamvr_poses(
    targets: &[tracking_core::TargetPose],
    enabled: bool,
) -> [steamvr_bridge::Pose; 8] {
    let mut poses = [steamvr_bridge::Pose::invalid(); 8];
    for t in targets {
        // Standing-space velocities; the bridge sanitises every record on send.
        poses[target_index(t.role)] = steamvr_bridge::Pose {
            valid: enabled && t.valid,
            position: t.position,
            rotation: [
                t.orientation_xyzw[3],
                t.orientation_xyzw[0],
                t.orientation_xyzw[1],
                t.orientation_xyzw[2],
            ],
            linear_velocity: t.linear_velocity,
            angular_velocity: t.angular_velocity,
        };
    }
    poses
}
#[derive(Debug, PartialEq, Eq)]
enum AnchorTransition {
    Duplicate,
    Updated,
    /// Same helper session; the standing transform changed materially.
    SpaceChanged,
    /// A different helper session, with or without a transform change.
    SessionChanged {
        space_changed: bool,
    },
}
fn anchor_transition(
    last: Option<(u64, u64)>,
    reference: Option<&[f32; 12]>,
    frame: &steamvr_bridge::Anchors,
) -> AnchorTransition {
    if last == Some((frame.session, frame.sequence)) {
        return AnchorTransition::Duplicate;
    }
    let space_changed = reference
        .is_some_and(|previous| standing_space_changed(previous, &frame.raw_from_standing));
    if last.is_some_and(|(session, _)| session != frame.session) {
        return AnchorTransition::SessionChanged { space_changed };
    }
    if space_changed {
        return AnchorTransition::SpaceChanged;
    }
    AnchorTransition::Updated
}

/// What an anchor frame tells the core about SteamVR's tracking space.
/// `None`: nothing happened. `Some(transform_yaw)`: a possible tracking-space
/// event (new helper session or material transform change), with the yaw of
/// the transform change when it is a pure yaw. The event is never evidence by
/// itself: the core compares headset poses and uses the transform's yaw only
/// when the observed jump agrees with it. Calibration is never cleared here.
fn tracking_space_event(
    transition: &AnchorTransition,
    reference: Option<&[f32; 12]>,
    current: &[f32; 12],
) -> Option<Option<f32>> {
    let transform_yaw = || reference.and_then(|old| standing_rebase_yaw(old, current));
    match transition {
        AnchorTransition::Duplicate | AnchorTransition::Updated => None,
        AnchorTransition::SpaceChanged
        | AnchorTransition::SessionChanged {
            space_changed: true,
        } => Some(transform_yaw()),
        AnchorTransition::SessionChanged {
            space_changed: false,
        } => Some(None),
    }
}

/// Automatic eye height: smaller differences are left alone.
const AUTO_HEIGHT_MIN_CHANGE_M: f32 = 0.015;
/// Larger automatic changes are announced ("Height updated.").
const AUTO_HEIGHT_ANNOUNCE_M: f32 = 0.05;

/// The eye height a live neutral capture should adopt: the averaged stable
/// standing headset height when it differs from the configured eye height by
/// more than 1.5 cm and is within the valid range, i.e. 0.8–2.5 m and inside
/// the live gate's own height tolerance (so the update refines the eye height
/// but never widens what the gate accepts).
fn auto_eye_height(configured: f32, measured: Option<f32>, tolerance: f32) -> Option<f32> {
    let measured = measured.filter(|height| height.is_finite())?;
    let difference = (measured - configured).abs();
    (difference > AUTO_HEIGHT_MIN_CHANGE_M
        && difference <= tolerance
        && (0.8..=2.5).contains(&measured))
    .then_some(measured)
}

fn usable_height_anchor(anchors: &Anchors, now: f64, stale_after: f64) -> Option<&AnchorPose> {
    let head = anchors.hmd.as_ref()?;
    let quaternion_norm_squared: f64 = head
        .orientation_xyzw
        .iter()
        .map(|value| f64::from(*value).powi(2))
        .sum();
    (head.valid
        && now.is_finite()
        && now >= 0.
        && stale_after.is_finite()
        && stale_after > 0.
        && head.received_at_secs.is_finite()
        && head.received_at_secs >= 0.
        && head.received_at_secs <= now
        && now - head.received_at_secs <= stale_after
        && head.position.iter().all(|value| value.is_finite())
        && (0.5..=1.5).contains(&quaternion_norm_squared))
    .then_some(head)
}

fn current_standing_headset_height(
    anchors: &Anchors,
    now: f64,
    stale_after: f64,
) -> Result<f32, String> {
    let head = usable_height_anchor(anchors, now, stale_after)
        .ok_or_else(|| "A fresh, valid live headset pose is required to measure eye height. Wear the headset and stand upright.".to_owned())?;
    let height = head.position[1];
    if !(0.8..=2.5).contains(&height) {
        return Err(format!(
            "Current headset standing height is {height:.3} m; expected 0.8–2.5 m. Stand upright and check SteamVR's floor calibration."
        ));
    }
    Ok(height)
}

/// Head facing projected onto the floor. A vertical gaze has no reliable yaw.
fn current_headset_forward(anchors: &Anchors, now: f64, stale_after: f64) -> Option<[f32; 3]> {
    let head = usable_height_anchor(anchors, now, stale_after)?;
    let direction = glam::Quat::from_array(head.orientation_xyzw).normalize() * -glam::Vec3::Z;
    let horizontal = glam::Vec3::new(direction.x, 0., direction.z);
    (horizontal.length_squared() >= 0.01).then(|| horizontal.normalize().to_array())
}

fn config_with_standing_height(
    config: &TrackingConfig,
    height: f32,
) -> Result<TrackingConfig, String> {
    if !height.is_finite() || !(0.8..=2.5).contains(&height) {
        return Err("Standing eye height must be between 0.8 and 2.5 m.".into());
    }
    let mut next = config.clone();
    let a = &mut next.anthropometrics;
    let ratio = height / a.standing_eye_height_m;
    a.standing_eye_height_m = height;
    a.head_to_neck_m *= ratio;
    a.neck_to_chest_m *= ratio;
    a.chest_to_hip_m *= ratio;
    a.hip_width_m *= ratio;
    a.shoulder_width_m *= ratio;
    a.thigh_length_m *= ratio;
    a.calf_length_m *= ratio;
    a.foot_length_m *= ratio;
    a.ankle_to_sole_m *= ratio;
    a.upper_arm_length_m *= ratio;
    a.forearm_length_m *= ratio;
    a.eye_forward_m *= ratio;
    // Scaling keeps an inconsistent profile inconsistent: make the modelled
    // standing chain equal the new eye height. If chest-to-hip cannot absorb the
    // difference the scaled dimensions stay (already reported at profile load).
    let _ = a.normalize_chain_height();
    next.validate().map_err(|error| error.to_string())?;
    Ok(next)
}

/// Profiles saved before the default dimensions changed can model a standing
/// chain (head-to-neck through ankle-to-sole) that differs from the eye height,
/// which leaves standing feet above or below the floor. Adjusts chest-to-hip so
/// they agree; returns a message when it changed something or could not.
pub(crate) fn normalize_profile_dimensions(
    anthropometrics: &mut tracking_core::Anthropometrics,
) -> Option<String> {
    let before = anthropometrics.chest_to_hip_m;
    match anthropometrics.normalize_chain_height() {
        Ok(delta) if delta != 0.0 => Some(format!(
            "Body dimensions adjusted: chest-to-hip {before:.3} m -> {:.3} m so the modelled standing height equals the eye height ({:.3} m).",
            anthropometrics.chest_to_hip_m, anthropometrics.standing_eye_height_m
        )),
        Ok(_) => None,
        Err(error) => Some(format!("Body dimensions left unchanged: {error}.")),
    }
}

fn active_calibration_phase(phase: CalibrationRequestPhase) -> bool {
    matches!(
        phase,
        CalibrationRequestPhase::Countdown
            | CalibrationRequestPhase::WaitingStable
            | CalibrationRequestPhase::Moving
    )
}

/// Readiness of the movement calibration is sampled this often (the status
/// evaluation scans the retained knee frames).
const MOTION_STATUS_INTERVAL_SECS: f64 = 0.1;
/// The first spoken movement hint waits until the instructed bow and knee
/// lifts could have been done; later hints repeat at most this often.
const MOTION_FIRST_HINT_SECS: f64 = 8.0;
const MOTION_HINT_INTERVAL_SECS: f64 = 5.0;

fn core_role_label(role: Role) -> &'static str {
    match role {
        Role::Chest => "Chest",
        Role::Hip => "Hip",
        Role::LeftThigh => "Left thigh",
        Role::RightThigh => "Right thigh",
        Role::LeftCalf => "Left calf",
        Role::RightCalf => "Right calf",
        Role::LeftFoot => "Left foot",
        Role::RightFoot => "Right foot",
        Role::LeftUpperArm => "Left upper arm",
        Role::RightUpperArm => "Right upper arm",
    }
}

/// Each blocking segment's own first blocker, in plain role names, plus any
/// calibration-wide blocker (for example a body turn).
fn motion_blocker_summary(
    status: &tracking_core::motion_calibration::MotionCalibrationStatus,
) -> String {
    let mut parts: Vec<String> = status.blockers.iter().map(|b| b.message.clone()).collect();
    for segment in status.segments.iter().filter(|s| !s.ready) {
        let label = core_role_label(segment.role);
        let text = segment.blockers.first().map_or("not ready yet", |b| {
            b.message
                .strip_prefix(&format!("{:?}: ", segment.role))
                .unwrap_or(&b.message)
        });
        parts.push(format!("{label}: {text}"));
    }
    parts.join("; ")
}

/// Never solve more often than 250 Hz, even when datagrams arrive faster.
const MIN_SOLVE_INTERVAL_SECS: f64 = 0.004;
/// Solve and publish at least at 100 Hz, with or without new data.
pub(crate) const MAX_SOLVE_INTERVAL_SECS: f64 = 0.010;
/// Helper supervision, calibration ticks, deferred disk jobs: the former 100 Hz tick.
const HOUSEKEEPING_INTERVAL_SECS: f64 = 0.010;
const OSC_INTERVAL_SECS: f64 = 1. / 60.;
const MAX_COMMANDS_PER_WAKE: usize = 32;
const PROFILE_SAVE_INTERVAL_SECS: f64 = 0.5;
const HELD_RECORD_INTERVAL_SECS: f64 = 1.0;
const RECORDING_LIMIT_BYTES: u64 = 2 * 1024 * 1024 * 1024;
const WRITER_QUEUE: usize = 8192;
const BRIDGE_MESSAGE_INTERVAL_SECS: f64 = 30.;
/// SteamVR exiting and returning is routine (Steam Link quits SteamVR after
/// 30 s of an idle headset): the helper is respawned this often, quietly.
const HELPER_RETRY_SECS: f64 = 5.;
/// A running helper that has produced no anchor frame for this long (for
/// example stuck in a SteamVR that is shutting down or starting up) is
/// stopped and respawned.
const HELPER_STALL_SECS: f64 = 20.;
/// Helper failures are reported only after this many in a row, and then at
/// most every `HELPER_MESSAGE_INTERVAL_SECS`; "SteamVR is not running" is
/// never reported (the status line shows it).
const HELPER_FAILURES_BEFORE_MESSAGE: u32 = 6;
const HELPER_MESSAGE_INTERVAL_SECS: f64 = 600.;
/// SteamVR counts as gone after this long without anchor frames.
const STEAMVR_LOST_SECS: f64 = 5.;
/// Alignment of a desktop calibration to the headset is attempted this often.
const ALIGN_TRY_INTERVAL_SECS: f64 = 0.1;

/// Alignment attempts run right after a solve (they need the frame's
/// calibration state) at most every `ALIGN_TRY_INTERVAL_SECS`.
fn housekeeping_due_alignment(next: &mut f64, now: f64) -> bool {
    if now >= *next || now + ALIGN_TRY_INTERVAL_SECS < *next {
        *next = now + ALIGN_TRY_INTERVAL_SECS;
        true
    } else {
        false
    }
}

/// Pacing decision (pure): solve when new data arrived and the 250 Hz floor
/// interval has passed, or when the 100 Hz ceiling interval passed regardless.
pub(crate) fn solve_due(now: f64, last_solve: Option<f64>, new_data: bool) -> bool {
    match last_solve {
        None => true,
        Some(last) if now < last => true,
        Some(last) => {
            let elapsed = now - last;
            (new_data && elapsed >= MIN_SOLVE_INTERVAL_SECS) || elapsed >= MAX_SOLVE_INTERVAL_SECS
        }
    }
}

/// Longest the worker may block waiting for tracker data before its next duty.
fn wait_budget(now: f64, last_solve: Option<f64>, new_data: bool, next_housekeeping: f64) -> f64 {
    let solve_at = match last_solve {
        None => now,
        Some(last) if new_data => last + MIN_SOLVE_INTERVAL_SECS,
        Some(last) => last + MAX_SOLVE_INTERVAL_SECS,
    };
    (solve_at.min(next_housekeeping) - now).clamp(0.0, MAX_SOLVE_INTERVAL_SECS)
}

/// Advance a periodic deadline by exactly one period so the average rate is the
/// nominal rate; after a long pause restart from now instead of bursting.
fn advance_deadline(deadline: f64, now: f64, period: f64) -> f64 {
    let next = deadline + period;
    if next <= now { now + period } else { next }
}

/// Rolling one-second solve rate and datagram-to-solve latency.
#[derive(Default)]
struct SolveStats {
    solves: VecDeque<f64>,
    latencies_ms: VecDeque<(f64, f32)>,
}

impl SolveStats {
    const MAX_LATENCIES: usize = 8192;
    fn record(&mut self, now: f64, received_at: &mut Vec<f64>) {
        self.solves.push_back(now);
        for at in received_at.drain(..) {
            let latency = now - at;
            if latency.is_finite() && latency >= 0.0 {
                self.latencies_ms.push_back((now, (latency * 1000.) as f32));
            }
        }
        while self.solves.front().is_some_and(|t| *t <= now - 1.0) {
            self.solves.pop_front();
        }
        while self
            .latencies_ms
            .front()
            .is_some_and(|(t, _)| *t <= now - 1.0)
            || self.latencies_ms.len() > Self::MAX_LATENCIES
        {
            self.latencies_ms.pop_front();
        }
    }
    fn rate_hz(&self, now: f64) -> f32 {
        self.solves.iter().filter(|t| **t > now - 1.0).count() as f32
    }
    /// Nearest-rank p50/p95 in milliseconds.
    fn latency_percentiles(&self) -> (Option<f32>, Option<f32>) {
        let mut values: Vec<f32> = self.latencies_ms.iter().map(|(_, v)| *v).collect();
        if values.is_empty() {
            return (None, None);
        }
        values.sort_by(f32::total_cmp);
        let rank =
            |p: f64| values[((p * values.len() as f64).ceil() as usize).clamp(1, values.len()) - 1];
        (Some(rank(0.50)), Some(rank(0.95)))
    }
}

/// Moves an unreadable or rejected profile aside so the next save cannot
/// silently overwrite it. Returns the backup location.
fn preserve_unusable_profile(path: &Path) -> io::Result<PathBuf> {
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let name = path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "profile.json".into());
    let target = path.with_file_name(format!("{name}.corrupt-{stamp}"));
    fs::rename(path, &target)?;
    Ok(target)
}

/// Runs on a short-lived thread: filesystem lookup plus process creation.
fn spawn_anchor_helper() -> io::Result<Child> {
    let exe = std::env::current_exe().unwrap_or_default();
    let candidates = [
        exe.with_file_name("chadfbt-anchors.exe"),
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../driver/build/chadfbt-anchors.exe"),
    ];
    let Some(path) = candidates.iter().find(|p| p.exists()) else {
        return Err(io::Error::from(io::ErrorKind::NotFound));
    };
    let mut command = ProcessCommand::new(path);
    command
        .current_dir(path.parent().unwrap())
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    command.spawn()
}

fn write_profile_atomically(path: &Path, bytes: &[u8]) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let temp = path.with_extension("json.tmp");
    let mut file = File::create(&temp)?;
    file.write_all(bytes)?;
    file.sync_all()?;
    drop(file);
    fs::rename(temp, path)
}

/// Whether a restarted tracker identity carries a body role (its calibration is then void).
fn reconnect_affects_assignment(identities: &[String], assignments: &[RoleAssignment]) -> bool {
    identities.iter().any(|identity| {
        let prefix = format!("{identity}/");
        assignments.iter().any(|a| a.sensor_id.starts_with(&prefix))
    })
}

/// How an anchor-helper exit is reported.
#[derive(Debug, PartialEq)]
enum HelperExit {
    /// SteamVR is not running, starting up or shutting down: retried quietly.
    Routine,
    Failed(String),
}

/// Classifies a helper exit from its status and stderr. The helper exits
/// with an error when SteamVR is not running or refuses a new application
/// (starting up, or shutting down after an idle headset); those are routine.
fn helper_exit(success: bool, status: &str, stderr: &str) -> HelperExit {
    const ROUTINE: [&str; 3] = [
        "SteamVR is not running",
        "OpenVR initialization failed",
        "OpenVR system unavailable",
    ];
    if success || stderr.is_empty() || ROUTINE.iter().any(|text| stderr.contains(text)) {
        return HelperExit::Routine;
    }
    HelperExit::Failed(format!("SteamVR anchor helper exited ({status}): {stderr}"))
}

enum WriteJob {
    Profile { path: PathBuf, bytes: Vec<u8> },
    Diagnostics { path: PathBuf, bytes: Vec<u8> },
    StartRecording { path: PathBuf },
    Record(Box<Record>),
    FlushRecording,
    StopRecording,
}

enum WriterNotice {
    Message(String),
    RecordingStopped { path: String, message: String },
}

struct ActiveRecording {
    writer: BufWriter<File>,
    path: String,
    bytes: u64,
}

/// File I/O off the real-time worker. The worker only enqueues; it never waits
/// on the disk except at shutdown.
struct DiskWriter {
    jobs: Option<SyncSender<WriteJob>>,
    notices: mpsc::Receiver<WriterNotice>,
    thread: Option<thread::JoinHandle<()>>,
}

impl DiskWriter {
    fn spawn(limit: u64, capacity: usize) -> Self {
        let (jobs, job_rx) = mpsc::sync_channel(capacity);
        let (notice_tx, notices) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("chadfbt-disk-writer".into())
            .spawn(move || writer_loop(job_rx, notice_tx, limit))
            .ok();
        Self {
            jobs: Some(jobs),
            notices,
            thread,
        }
    }
    fn try_submit(&self, job: WriteJob) -> Result<(), WriteJob> {
        match &self.jobs {
            Some(jobs) => jobs.try_send(job).map_err(|error| match error {
                TrySendError::Full(job) | TrySendError::Disconnected(job) => job,
            }),
            None => Err(job),
        }
    }
    /// Shutdown only: deliver remaining jobs and wait for the disk.
    fn finish(&mut self, remaining: impl IntoIterator<Item = WriteJob>) {
        if let Some(jobs) = self.jobs.take() {
            for job in remaining {
                let _ = jobs.send(job);
            }
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn writer_loop(jobs: mpsc::Receiver<WriteJob>, notices: mpsc::Sender<WriterNotice>, limit: u64) {
    let mut recording: Option<ActiveRecording> = None;
    let stop = |recording: &mut Option<ActiveRecording>,
                notices: &mpsc::Sender<WriterNotice>,
                message: String| {
        if let Some(mut active) = recording.take() {
            let _ = active.writer.flush();
            let _ = notices.send(WriterNotice::RecordingStopped {
                path: active.path,
                message,
            });
        }
    };
    for job in jobs {
        match job {
            WriteJob::Profile { path, bytes } => {
                if let Err(error) = write_profile_atomically(&path, &bytes) {
                    let _ = notices.send(WriterNotice::Message(format!(
                        "Profile save failed: {error}"
                    )));
                }
            }
            WriteJob::Diagnostics { path, bytes } => {
                let result = path
                    .parent()
                    .map_or(Ok(()), fs::create_dir_all)
                    .and_then(|_| fs::write(&path, &bytes));
                let _ = notices.send(WriterNotice::Message(match result {
                    Ok(()) => format!("Saved diagnostic snapshot: {}", path.display()),
                    Err(error) => format!("Diagnostics export failed: {error}"),
                }));
            }
            WriteJob::StartRecording { path } => {
                if let Some(mut active) = recording.take() {
                    let _ = active.writer.flush();
                }
                let display = path.display().to_string();
                match path
                    .parent()
                    .map_or(Ok(()), fs::create_dir_all)
                    .and_then(|_| File::create(&path))
                {
                    Ok(file) => {
                        recording = Some(ActiveRecording {
                            writer: BufWriter::new(file),
                            path: display,
                            bytes: 0,
                        })
                    }
                    Err(error) => {
                        let _ = notices.send(WriterNotice::RecordingStopped {
                            path: display,
                            message: format!("Cannot record: {error}"),
                        });
                    }
                }
            }
            WriteJob::Record(record) => {
                let Some(active) = &mut recording else {
                    continue;
                };
                let mut line = match serde_json::to_vec(&*record) {
                    Ok(line) => line,
                    Err(error) => {
                        stop(
                            &mut recording,
                            &notices,
                            format!("Recording stopped: {error}"),
                        );
                        continue;
                    }
                };
                line.push(b'\n');
                if active.bytes + line.len() as u64 > limit {
                    stop(
                        &mut recording,
                        &notices,
                        format!(
                            "Recording stopped at the {:.1} GB size limit.",
                            limit as f64 / (1024. * 1024. * 1024.)
                        ),
                    );
                    continue;
                }
                match active.writer.write_all(&line) {
                    Ok(()) => active.bytes += line.len() as u64,
                    Err(error) => stop(
                        &mut recording,
                        &notices,
                        format!("Recording stopped: {error}"),
                    ),
                }
            }
            WriteJob::FlushRecording => {
                if let Some(active) = &mut recording
                    && let Err(error) = active.writer.flush()
                {
                    stop(
                        &mut recording,
                        &notices,
                        format!("Recording flush failed: {error}"),
                    );
                }
            }
            WriteJob::StopRecording => {
                if let Some(mut active) = recording.take()
                    && let Err(error) = active.writer.flush()
                {
                    let _ = notices.send(WriterNotice::Message(format!(
                        "Recording flush failed: {error}"
                    )));
                }
            }
        }
    }
    if let Some(mut active) = recording.take() {
        let _ = active.writer.flush();
    }
}

fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|text| (*text).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "unknown panic".into())
}

pub fn start() -> (AppHandle, thread::JoinHandle<()>) {
    let (tx, rx) = mpsc::channel();
    let state = Arc::new(Mutex::new(Snapshot {
        status: "Starting native tracking worker…".into(),
        height_m: 1.65,
        ..Default::default()
    }));
    let health = Arc::new(WorkerHealth::default());
    let handle = AppHandle {
        state: state.clone(),
        commands: tx,
        health: health.clone(),
    };
    let worker = thread::Builder::new()
        .name("tracking-worker".into())
        .spawn(move || {
            let worker_health = health.clone();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                Worker::new(state, rx, worker_health).run()
            }));
            health.finish(result.err().map(|payload| panic_text(payload.as_ref())));
        })
        .expect("tracking worker");
    (handle, worker)
}

struct Worker {
    state: Arc<Mutex<Snapshot>>,
    commands: mpsc::Receiver<Command>,
    core: TrackingCore,
    hub: Option<TrackerHub>,
    bridge: Option<steamvr_bridge::Bridge>,
    osc: Option<UdpSocket>,
    origin: Instant,
    anchors: Anchors,
    raw_from_standing: [f32; 12],
    last_anchor_frame: Option<(u64, u64)>,
    /// Worker-clock receipt time of the last accepted anchor frame.
    last_anchor_received_at: Option<f64>,
    standing_reference: Option<[f32; 12]>,
    /// Anchors exactly as received from SteamVR. `anchors` (fed to the core
    /// and recorded) is empty in Desktop-only mode.
    steamvr_anchors: Anchors,
    estimator: EstimatorSettings,
    /// Persisted output mode.
    output_mode: OutputMode,
    /// Worn-headset availability with hysteresis (Auto's live/desktop switch).
    presence: HeadsetPresence,
    /// Sink used by the last publish.
    sink: OutputSink,
    /// Shutting down: nothing valid is published any more.
    suspended: bool,
    /// The current calibration request is a desktop calibration (latched
    /// when it started).
    request_preview: bool,
    /// Live neutral captures adopt the measured headset height.
    auto_eye_height: bool,
    /// The last capture changed the eye height by more than 5 cm; the next
    /// calibration prompt says so.
    height_announce: bool,
    /// A desktop calibration is waiting to be aligned to the worn headset:
    /// time of the next attempt.
    alignment_next_try: Option<f64>,
    alignment_status: String,
    /// Core counters already reported to the user.
    seen_rebases: u32,
    seen_resumptions: u32,
    /// Hands-free chest/hip tap gestures enabled.
    gesture: bool,
    taps: TapRecognizer,
    /// Last time each sensor delivered acceleration (firmware tap packets are
    /// used only for trackers without an acceleration stream).
    accel_seen_at: BTreeMap<String, f64>,
    /// Last recognised or ignored tap sequence, for the UI.
    gesture_status: String,
    /// A heading recovery was requested at this time; its outcome is spoken.
    recovery_watch: Option<f64>,
    /// Next time a movement-calibration hint may be spoken.
    motion_hint_at: Option<f64>,
    messages: Vec<String>,
    writer: DiskWriter,
    /// Jobs that must not be lost; retried each wake when the writer queue is full.
    deferred_jobs: VecDeque<WriteJob>,
    recording: Option<String>,
    held_recorded_at: BTreeMap<String, f64>,
    profile_path: PathBuf,
    /// The existing profile could not be read (not parsed); never overwrite it.
    profile_locked: bool,
    save_pending: bool,
    last_save_at: f64,
    helper: Option<Child>,
    helper_spawn: Option<mpsc::Receiver<io::Result<Child>>>,
    helper_next_try: f64,
    helper_next_message: f64,
    helper_started_at: f64,
    helper_failures: u32,
    /// SteamVR anchor frames are arriving (for one-line connect/lost messages).
    steamvr_live: bool,
    bridge_next_message: f64,
    health: Arc<WorkerHealth>,
    stats: SolveStats,
    /// Receipt times of measured samples ingested since the previous solve.
    pending_latency: Vec<f64>,
    session: u64,
    sequence: u64,
    last_ui: f64,
    next_osc: f64,
    last_flush: f64,
    calibration_request: CalibrationRequestController,
    calibration_request_kind: CalibrationRequestKind,
    calibration_audio_phase: CalibrationRequestPhase,
    calibration_countdown_second: Option<u64>,
    calibration_audio: AudioCues,
    manual_mount_message: String,
    calibration_method: CalibrationMethod,
    /// The current movement request reuses the existing neutral capture.
    motion_move_only: bool,
    /// Request ID whose core movement session was started.
    motion_session_request: Option<u64>,
    /// Latest status of the current or last movement request (10 Hz).
    motion_status: Option<tracking_core::motion_calibration::MotionCalibrationStatus>,
    motion_status_at: Option<f64>,
    /// Describes the forward reference of the last committed movement result.
    motion_reference_note: String,
}

impl Worker {
    fn new(
        state: Arc<Mutex<Snapshot>>,
        commands: mpsc::Receiver<Command>,
        health: Arc<WorkerHealth>,
    ) -> Self {
        let origin = Instant::now();
        let profile_path = data_dir().join("profile.json");
        let mut messages = Vec::new();
        let mut estimator = EstimatorSettings::default();
        let mut core = TrackingCore::new(
            config_with_estimator(
                &TrackingConfig {
                    enable_preview_motion: desktop_motion_default(),
                    ..TrackingConfig::default()
                },
                &estimator,
            )
            .expect("valid estimator defaults"),
        )
        .expect("valid defaults");
        let mut gesture = false;
        let mut audio_mode = AudioMode::default();
        let mut taps = TapRecognizer::default();
        let mut calibration_method = CalibrationMethod::default();
        let mut output_mode = OutputMode::default();
        let mut auto_eye_height = auto_eye_height_default();
        let mut profile_locked = false;
        if profile_path.exists() {
            let unusable = match fs::read(&profile_path) {
                Err(e) => {
                    // Possibly transient (locked by another program): keep the file
                    // and do not overwrite it during this session.
                    profile_locked = true;
                    messages.push(format!(
                        "Profile could not be read: {e}. Changes in this session will not be saved over it."
                    ));
                    None
                }
                Ok(bytes) => match serde_json::from_slice::<Settings>(&bytes) {
                    Err(e) => Some(format!("Profile could not be loaded: {e}")),
                    Ok(settings) => {
                        let mut config = core.config().clone();
                        config.enable_joint_yaw_correction = settings.correction;
                        config.enable_preview_motion = settings.desktop_motion;
                        gesture = settings.gesture;
                        audio_mode = settings.audio_mode;
                        match TapRecognizer::new(settings.tap_gestures) {
                            Ok(recognizer) => taps = recognizer,
                            Err(error) => messages.push(format!(
                                "Saved tap gesture thresholds ignored ({error}); using defaults."
                            )),
                        }
                        calibration_method = settings.calibration_method;
                        output_mode = settings.output_mode;
                        auto_eye_height = settings.auto_eye_height;
                        match config_with_estimator(&config, &settings.estimator) {
                            Ok(next) => {
                                estimator = settings.estimator;
                                config = next;
                            }
                            Err(error) => messages.push(format!(
                                "Saved estimator settings ignored ({error}); using defaults."
                            )),
                        }
                        let mut profile = settings.profile;
                        if let Some(note) =
                            normalize_profile_dimensions(&mut profile.anthropometrics)
                        {
                            messages.push(note);
                        }
                        core.set_config(config)
                            .and_then(|_| core.restore_profile(profile))
                            .err()
                            .map(|e| format!("Profile rejected: {e}"))
                    }
                },
            };
            if let Some(reason) = unusable {
                messages.push(reason);
                match preserve_unusable_profile(&profile_path) {
                    Ok(backup) => messages.push(format!(
                        "The unusable profile was kept as {}; settings start from defaults.",
                        backup.display()
                    )),
                    Err(e) => {
                        profile_locked = true;
                        messages.push(format!(
                            "Could not move the unusable profile aside ({e}); it will not be overwritten this session."
                        ));
                    }
                }
            }
        }
        let hub = match TrackerHub::bind(origin) {
            Ok(h) => Some(h),
            Err(e) => {
                messages.push(format!(
                    "Tracker UDP 6969 unavailable: {e}. Close other tracker receivers."
                ));
                None
            }
        };
        let bridge = match steamvr_bridge::Bridge::bind() {
            Ok(b) => Some(b),
            Err(e) => {
                messages.push(format!("SteamVR bridge unavailable: {e}"));
                None
            }
        };
        let osc = UdpSocket::bind(("127.0.0.1", 0)).ok();
        let session = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        Self {
            state,
            commands,
            core,
            hub,
            bridge,
            osc,
            origin,
            anchors: Anchors::default(),
            raw_from_standing: [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.],
            last_anchor_frame: None,
            last_anchor_received_at: None,
            standing_reference: None,
            steamvr_anchors: Anchors::default(),
            estimator,
            output_mode,
            presence: HeadsetPresence::default(),
            sink: OutputSink::Off,
            suspended: false,
            request_preview: false,
            auto_eye_height,
            height_announce: false,
            alignment_next_try: None,
            alignment_status: String::new(),
            seen_rebases: 0,
            seen_resumptions: 0,
            gesture,
            taps,
            accel_seen_at: BTreeMap::new(),
            gesture_status: String::new(),
            recovery_watch: None,
            motion_hint_at: None,
            messages,
            writer: DiskWriter::spawn(RECORDING_LIMIT_BYTES, WRITER_QUEUE),
            deferred_jobs: VecDeque::new(),
            recording: None,
            held_recorded_at: BTreeMap::new(),
            profile_path,
            profile_locked,
            save_pending: false,
            last_save_at: f64::NEG_INFINITY,
            helper: None,
            helper_spawn: None,
            helper_next_try: 0.,
            helper_next_message: 0.,
            helper_started_at: 0.,
            helper_failures: 0,
            steamvr_live: false,
            bridge_next_message: 0.,
            health,
            stats: SolveStats::default(),
            pending_latency: Vec::new(),
            session,
            sequence: 0,
            last_ui: -1.,
            next_osc: 0.,
            last_flush: 0.,
            calibration_request: CalibrationRequestController::default(),
            calibration_request_kind: CalibrationRequestKind::NeutralPose,
            calibration_audio_phase: CalibrationRequestPhase::Idle,
            calibration_countdown_second: None,
            calibration_audio: {
                let audio = AudioCues::new();
                audio.set_mode(audio_mode);
                audio
            },
            manual_mount_message: String::new(),
            calibration_method,
            motion_move_only: false,
            motion_session_request: None,
            motion_status: None,
            motion_status_at: None,
            motion_reference_note: String::new(),
        }
    }

    fn message(&mut self, text: impl Into<String>) {
        self.messages.push(text.into());
        if self.messages.len() > 16 {
            self.messages.remove(0);
        }
    }
    fn write_record(&mut self, event: &Record) {
        if self.recording.is_some() {
            self.send_record(event.clone());
        }
    }
    /// Recording records are the only droppable disk jobs: if the writer falls
    /// behind, the recording ends (truncated, consistent) instead of blocking.
    fn send_record(&mut self, event: Record) {
        if self.recording.is_none() {
            return;
        }
        if self
            .writer
            .try_submit(WriteJob::Record(Box::new(event)))
            .is_err()
        {
            self.recording = None;
            self.deferred_jobs.push_back(WriteJob::StopRecording);
            self.message("Recording stopped: the disk writer fell behind.");
        }
    }
    fn submit_job(&mut self, job: WriteJob) {
        self.deferred_jobs.push_back(job);
        self.flush_deferred_jobs();
    }
    fn flush_deferred_jobs(&mut self) {
        while let Some(job) = self.deferred_jobs.pop_front() {
            if let Err(job) = self.writer.try_submit(job) {
                self.deferred_jobs.push_front(job);
                break;
            }
        }
    }

    /// Request a profile save; saves are coalesced to one per 500 ms.
    fn save(&mut self) {
        self.save_pending = true;
    }
    fn settings_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec_pretty(&Settings {
            profile: self.core.export_profile(),
            correction: self.core.config().enable_joint_yaw_correction,
            gesture: self.gesture,
            audio_mode: self.calibration_audio.mode(),
            tap_gestures: *self.taps.config(),
            desktop_motion: self.core.config().enable_preview_motion,
            estimator: self.estimator,
            calibration_method: self.calibration_method,
            output_mode: self.output_mode,
            auto_eye_height: self.auto_eye_height,
        })
    }
    fn flush_save(&mut self, now: f64, force: bool) {
        if !self.save_pending
            || (!force
                && now - self.last_save_at < PROFILE_SAVE_INTERVAL_SECS
                && now >= self.last_save_at)
        {
            return;
        }
        self.save_pending = false;
        self.last_save_at = now;
        if self.profile_locked {
            return;
        }
        match self.settings_bytes() {
            Ok(bytes) => self.submit_job(WriteJob::Profile {
                path: self.profile_path.clone(),
                bytes,
            }),
            Err(e) => self.message(format!("Profile save failed: {e}")),
        }
    }
    /// Deferred disk work and writer results; never waits on the disk.
    fn writer_tick(&mut self, now: f64) {
        self.flush_save(now, false);
        self.flush_deferred_jobs();
        while let Ok(notice) = self.writer.notices.try_recv() {
            match notice {
                WriterNotice::Message(text) => self.message(text),
                WriterNotice::RecordingStopped { path, message } => {
                    if self.recording.as_deref() == Some(path.as_str()) {
                        self.recording = None;
                    }
                    self.message(message);
                }
            }
        }
    }

    fn helper_tick(&mut self, now: f64) {
        // Enables isolated offline smoke tests without initializing the OpenVR runtime.
        if std::env::var_os("CHADFBT_NO_HELPER").is_some() {
            return;
        }
        if let Some(child) = &mut self.helper {
            // A helper that stopped producing frames (for example wedged in a
            // SteamVR that is shutting down or starting up) is replaced.
            let last_frame = self
                .last_anchor_received_at
                .unwrap_or(f64::NEG_INFINITY)
                .max(self.helper_started_at);
            let stalled = self.bridge.is_some() && now - last_frame > HELPER_STALL_SECS;
            let outcome = match child.try_wait() {
                Ok(None) if !stalled => {
                    if self
                        .last_anchor_received_at
                        .is_some_and(|at| at >= self.helper_started_at)
                    {
                        self.helper_failures = 0;
                    }
                    return;
                }
                Ok(None) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    HelperExit::Failed(format!(
                        "SteamVR anchor helper sent nothing for {HELPER_STALL_SECS:.0} s and was restarted"
                    ))
                }
                Ok(Some(status)) => {
                    let mut detail = String::new();
                    if let Some(stderr) = child.stderr.take() {
                        let _ = stderr.take(4096).read_to_string(&mut detail);
                    }
                    helper_exit(status.success(), &status.to_string(), detail.trim())
                }
                Err(error) => {
                    HelperExit::Failed(format!("Cannot inspect SteamVR anchor helper: {error}"))
                }
            };
            self.helper = None;
            self.helper_next_try = self.helper_next_try.min(now + 1.0);
            match outcome {
                // Routine: SteamVR not running, starting up, or shutting down.
                HelperExit::Routine => self.helper_failures = 0,
                HelperExit::Failed(text) => {
                    self.helper_failures = self.helper_failures.saturating_add(1);
                    if self.helper_failures >= HELPER_FAILURES_BEFORE_MESSAGE
                        && now >= self.helper_next_message
                    {
                        self.message(text);
                        self.helper_next_message = now + HELPER_MESSAGE_INTERVAL_SECS;
                    }
                }
            }
        }
        // Process creation and the executable lookup run off the real-time worker.
        if let Some(pending) = &self.helper_spawn {
            match pending.try_recv() {
                Ok(Ok(child)) => {
                    self.helper_spawn = None;
                    self.helper = Some(child);
                    self.helper_started_at = now;
                }
                Ok(Err(e)) => {
                    self.helper_spawn = None;
                    if e.kind() == io::ErrorKind::NotFound {
                        // Only after the first attempt, as before.
                        if self.helper_next_try < 6. {
                            self.message("SteamVR anchor helper is missing. Build or use the complete portable bundle.");
                        }
                    } else if now >= self.helper_next_message {
                        self.message(format!("Cannot start SteamVR helper: {e}"));
                        self.helper_next_message = now + HELPER_MESSAGE_INTERVAL_SECS;
                    }
                }
                Err(TryRecvError::Empty) => return,
                Err(TryRecvError::Disconnected) => self.helper_spawn = None,
            }
            return;
        }
        if now < self.helper_next_try {
            return;
        }
        self.helper_next_try = now + HELPER_RETRY_SECS;
        let (tx, rx) = mpsc::channel();
        let spawned = thread::Builder::new()
            .name("chadfbt-helper-spawn".into())
            .spawn(move || {
                let _ = tx.send(spawn_anchor_helper());
            });
        match spawned {
            Ok(_) => self.helper_spawn = Some(rx),
            Err(e) => self.message(format!("Cannot start SteamVR helper: {e}")),
        }
    }

    /// One message when SteamVR anchor frames start or stop arriving; both
    /// are routine (Steam Link shuts SteamVR down after an idle headset).
    fn steamvr_connection_tick(&mut self, now: f64) {
        let live = self
            .last_anchor_received_at
            .is_some_and(|at| now >= at && now - at < STEAMVR_LOST_SECS);
        if live == self.steamvr_live {
            return;
        }
        self.steamvr_live = live;
        if live {
            self.message("SteamVR connected.");
        } else {
            self.message(
                "SteamVR stopped sending headset data. Tracking pauses and resumes by itself when it returns; calibration is kept.",
            );
        }
    }

    fn invalidate_calibration_request(&mut self, now: f64, reason: &str) {
        self.manual_mount_message.clear();
        self.calibration_audio.cancel();
        self.calibration_request.invalidate(now, reason);
        self.end_motion_session(now);
        if self.calibration_request.view(now).phase == CalibrationRequestPhase::Idle {
            self.motion_status = None;
        }
    }

    /// Discards the core's movement session (recorded for replay), if any.
    fn end_motion_session(&mut self, now: f64) {
        if self.core.motion_calibration_active() {
            self.core.cancel_motion_calibration();
            self.write_record(&Record::MotionCancel { at: now });
        }
        self.motion_session_request = None;
    }

    fn begin_calibration_request(&mut self, now: f64, kind: CalibrationRequestKind) {
        self.manual_mount_message.clear();
        self.calibration_audio.cancel();
        self.calibration_audio_phase = CalibrationRequestPhase::Idle;
        self.calibration_countdown_second = None;
        self.calibration_request_kind = kind;
        // Auto: a desktop calibration exactly when no worn headset is
        // available right now. Publishing pauses during every active phase.
        self.request_preview = self.desktop_session();
        self.height_announce = false;
        self.end_motion_session(now);
        self.motion_status = None;
        self.motion_status_at = None;
        match kind {
            CalibrationRequestKind::ForwardPose => self.calibration_request.start_forward(now),
            CalibrationRequestKind::MotionPose if self.motion_move_only => {
                self.calibration_request.start_motion_move(now)
            }
            CalibrationRequestKind::MotionPose => self.calibration_request.start_motion(now),
            _ => self.calibration_request.start(now),
        }
        self.calibration_audio_tick(now);
    }

    fn begin_motion_request(&mut self, now: f64, move_only: bool) {
        if move_only && let Err(error) = self.core.motion_calibration_available(now) {
            self.message(format!(
                "Cannot start at the movement step: {error}. Use \"Calibrate by movement\" to capture neutral first."
            ));
            return;
        }
        self.motion_move_only = move_only;
        self.begin_calibration_request(now, CalibrationRequestKind::MotionPose);
    }

    /// Movement phase supervision: starts the core session when the request
    /// enters `Moving`, samples readiness at 10 Hz, finishes once the request
    /// controller authorizes it, and discards a session the request left.
    /// Returns the clock of its last request-controller call (a finish
    /// attempt completes after `now`).
    fn motion_tick(&mut self, now: f64) -> f64 {
        let view = self.calibration_request.view(now);
        if self.calibration_request_kind != CalibrationRequestKind::MotionPose
            || view.phase != CalibrationRequestPhase::Moving
        {
            self.end_motion_session(now);
            return now;
        }
        if self.motion_session_request != Some(view.request_id) {
            match self.core.begin_motion_calibration(now) {
                Ok(()) => {
                    self.write_record(&Record::MotionBegin { at: now });
                    self.motion_session_request = Some(view.request_id);
                    self.motion_status_at = None;
                }
                Err(error) => {
                    self.calibration_request.abort_motion(
                        now,
                        format!("Movement calibration could not start: {error}"),
                    );
                    return now;
                }
            }
        } else if !self.core.motion_calibration_active() {
            self.motion_session_request = None;
            self.calibration_request.abort_motion(
                now,
                "The movement session was interrupted (calibration or tracking space changed). Start again.",
            );
            return now;
        }
        if self
            .motion_status_at
            .is_some_and(|at| now >= at && now - at < MOTION_STATUS_INTERVAL_SECS)
        {
            return now;
        }
        self.motion_status_at = Some(now);
        let Some(status) = self.core.motion_calibration_status() else {
            return now;
        };
        let ready = status.ready;
        let summary = (!ready).then(|| motion_blocker_summary(&status));
        let reference_note = match (status.selected_reference, &status.headset) {
            (tracking_core::motion_calibration::MotionReference::Fused, _) => {
                "forward from neck movement fused with the headset direction"
            }
            (_, Some(headset)) if headset.disagreement_flagged => {
                "forward from the headset direction (neck movement disagreed)"
            }
            (_, Some(_)) => "forward from the headset direction",
            (_, None) => "forward as faced during the neutral capture",
        };
        self.motion_status = Some(status);
        if !self.calibration_request.observe_motion(now, ready, summary) {
            return now;
        }
        let result = self
            .core
            .finish_motion_calibration(now)
            .map_err(|error| error.to_string());
        let succeeded = result.is_ok();
        if succeeded {
            self.write_record(&Record::MotionFinish { at: now });
            self.motion_session_request = None;
            self.motion_reference_note = reference_note.into();
        }
        let completed_at = self.clock();
        self.calibration_request
            .motion_finish_result(completed_at, result);
        if !succeeded {
            return completed_at;
        }
        if self.calibration_view(completed_at).phase == CalibrationRequestPhase::Captured {
            let message = self.calibration_view(completed_at).message;
            self.message(message);
            self.save();
        } else {
            // Committed after the fixed deadline: never keep a late result.
            self.core.clear_calibration();
            self.write_record(&Record::Clear { at: completed_at });
        }
        completed_at
    }

    fn calibration_view(&mut self, now: f64) -> CalibrationRequestView {
        let mut view = self.calibration_request.view(now);
        if self.calibration_request_kind == CalibrationRequestKind::StandingHeight {
            view.message = match view.phase {
                CalibrationRequestPhase::Countdown => "Headset height capture starts in 3 seconds. Wear the headset, stand upright and look forward.".into(),
                CalibrationRequestPhase::WaitingStable => view.message.replace("Waiting for a stable neutral pose:", "Waiting for a fresh standing headset height:")
                    .replace("Hold the neutral pose while calibration checks the headset and trackers.", "Stand upright while the current headset height is checked."),
                CalibrationRequestPhase::Captured => format!("Standing eye height captured: {:.3} m. Next start neutral calibration while standing upright.", self.core.config().anthropometrics.standing_eye_height_m),
                CalibrationRequestPhase::TimedOut | CalibrationRequestPhase::Cancelled => view.message.replace("Calibration", "Headset height capture"),
                CalibrationRequestPhase::Idle | CalibrationRequestPhase::Moving => view.message,
            };
        } else if self.calibration_request_kind == CalibrationRequestKind::ForwardPose {
            view.message = match view.phase {
                CalibrationRequestPhase::Countdown => "Take a comfortable skiing pose: feet flat, knees forward, hips back, and a forward lean. Your pelvis can tilt less than your chest. After the countdown you have up to 10 seconds to settle.".into(),
                CalibrationRequestPhase::WaitingStable => view.message.replace("Waiting for a stable neutral pose:", "Waiting for a clear, stable forward bend:")
                    .replace("Hold the neutral pose while calibration checks the headset and trackers.", "Hold the skiing pose while each tracker's movement axis is checked.")
                    .replace("is moving; wait for the tap and body motion to settle", "is still settling; hold this bend a moment longer"),
                CalibrationRequestPhase::Captured => if self.request_preview {
                    "Forward direction captured for all six body trackers. Stand upright again and test the bend directions. Desktop calibration: it is aligned to the headset automatically when one is worn (except in Desktop only mode).".into()
                } else {
                    "Forward direction captured for all six body trackers. Stand upright again; output follows the selected mode.".into()
                },
                CalibrationRequestPhase::TimedOut | CalibrationRequestPhase::Cancelled => view.message.replace("Calibration", "Forward-bend capture"),
                CalibrationRequestPhase::Idle | CalibrationRequestPhase::Moving => view.message,
            };
        } else if self.calibration_request_kind == CalibrationRequestKind::MotionPose {
            view.message = match view.phase {
                CalibrationRequestPhase::Countdown if self.motion_move_only => "Get ready to move, facing the same direction as at neutral. After the tone: bow forward once as if greeting someone, then lift each knee twice (or do two slow squats).".into(),
                CalibrationRequestPhase::Countdown => "Neutral capture starts in 3 seconds. Stand upright, feet under your hips, face forward and hold still. Start moving only after the next tone.".into(),
                CalibrationRequestPhase::WaitingStable => view.message,
                CalibrationRequestPhase::Moving => format!(
                    "Move now ({:.0} s left): bow forward once, then lift each knee twice (or do two slow squats). Keep facing the same direction, knees pointing forward, no twisting.",
                    view.seconds_remaining.ceil()
                ),
                CalibrationRequestPhase::Captured => {
                    let next = if self.request_preview {
                        "Stand upright and test the bend directions. Desktop calibration: nothing is sent; it is aligned to the headset automatically when one is worn (except in Desktop only mode)."
                    } else {
                        "Stand normally and test your movement; output follows the selected mode. While you move, the stance heading filter (if enabled) refines the leg directions."
                    };
                    format!(
                        "Movement calibration accepted for all six body trackers ({}). {next}",
                        self.motion_reference_note
                    )
                }
                CalibrationRequestPhase::TimedOut
                    if view.message.starts_with("Movement calibration did not finish") =>
                {
                    format!(
                        "{} Your neutral capture is kept: try the movement again, or use the held skiing pose instead.",
                        view.message
                    )
                }
                CalibrationRequestPhase::TimedOut => {
                    view.message.replace("Calibration timed out", "Neutral capture timed out")
                }
                CalibrationRequestPhase::Cancelled => view.message.replace("Calibration", "Movement calibration"),
                CalibrationRequestPhase::Idle => view.message,
            };
        } else if view.phase == CalibrationRequestPhase::Captured {
            view.message = if !self.core.direction_calibrated() {
                "Neutral pose captured and preview centered. Next capture the forward bend to identify each tracker's movement axes.".into()
            } else if self.request_preview {
                "Desktop neutral pose captured. Preview centered; learned mounting directions retained.".into()
            } else {
                "Neutral pose captured with headset heading; learned mounting directions retained."
                    .into()
            };
        }
        view
    }

    fn calibration_audio_tick(&mut self, now: f64) {
        let view = self.calibration_view(now);
        let entered = self.calibration_audio_phase != view.phase;
        let kind = self.calibration_request_kind;
        let motion = kind == CalibrationRequestKind::MotionPose;
        match view.phase {
            CalibrationRequestPhase::Countdown => {
                if entered {
                    let text = match kind {
                        CalibrationRequestKind::MotionPose if self.motion_move_only => {
                            voice_prompts::CALIBRATION_MOVE_ONLY_START
                        }
                        CalibrationRequestKind::MotionPose => voice_prompts::CALIBRATION_START,
                        CalibrationRequestKind::NeutralPose => voice_prompts::NEUTRAL_START,
                        CalibrationRequestKind::ForwardPose => voice_prompts::SKI_POSE_START,
                        CalibrationRequestKind::StandingHeight => voice_prompts::HEIGHT_START,
                    };
                    self.calibration_audio
                        .play(Prompt::say(None, text).with_fallback(Cue::Countdown));
                }
                let second = view.seconds_remaining.ceil() as u64;
                if second > 0 && self.calibration_countdown_second != Some(second) {
                    // Background: skipped while the instruction is spoken.
                    self.calibration_audio.tone(Cue::Countdown);
                    self.calibration_countdown_second = Some(second);
                }
            }
            CalibrationRequestPhase::WaitingStable if entered => {
                self.calibration_audio.tone(Cue::Ready)
            }
            CalibrationRequestPhase::Moving if entered => {
                self.motion_hint_at = Some(now + MOTION_FIRST_HINT_SECS);
                let text = self.with_height_note(voice_prompts::CALIBRATION_MOVE);
                self.calibration_audio
                    .play(Prompt::say(Some(Cue::Move), text));
            }
            CalibrationRequestPhase::Moving => {
                // At most every few seconds, name what is still missing.
                if self.motion_hint_at.is_some_and(|at| now >= at) {
                    self.motion_hint_at = Some(now + MOTION_HINT_INTERVAL_SECS);
                    if let Some(hint) = self
                        .motion_status
                        .as_ref()
                        .and_then(voice_prompts::motion_guidance)
                    {
                        self.calibration_audio.play(Prompt::say(None, hint));
                    }
                }
            }
            CalibrationRequestPhase::Captured if entered => {
                let text = if motion {
                    voice_prompts::CALIBRATION_COMPLETE
                } else {
                    voice_prompts::CAPTURED
                };
                let text = self.with_height_note(text);
                self.calibration_audio
                    .play(Prompt::say(Some(Cue::Success), text));
            }
            CalibrationRequestPhase::TimedOut if entered => {
                let text = if motion
                    && view
                        .message
                        .starts_with("Movement calibration did not finish")
                {
                    voice_prompts::motion_timeout(self.motion_status.as_ref(), self.gesture)
                } else if motion
                    && (view.message.starts_with("Movement calibration")
                        || view.message.starts_with("The movement session"))
                {
                    voice_prompts::motion_aborted(self.gesture)
                } else {
                    voice_prompts::capture_timeout(&view.message, self.gesture)
                };
                self.calibration_audio
                    .play(Prompt::say(Some(Cue::Failure), text));
                self.message(view.message.clone());
            }
            CalibrationRequestPhase::Cancelled if entered => {
                self.calibration_audio.cancel();
                self.calibration_audio.play(Prompt::say(
                    Some(Cue::Cancelled),
                    voice_prompts::CALIBRATION_CANCELLED,
                ));
            }
            _ => {}
        }
        if view.phase != CalibrationRequestPhase::Moving {
            self.motion_hint_at = None;
        }
        self.calibration_audio_phase = view.phase;
    }

    /// Prefixes "Height updated." once after a large automatic eye-height change.
    fn with_height_note(&mut self, text: &str) -> String {
        if std::mem::take(&mut self.height_announce) {
            format!("{} {text}", voice_prompts::HEIGHT_UPDATED)
        } else {
            text.to_owned()
        }
    }

    /// Speaks the outcome of the heading recovery requested at `recovery_watch`.
    fn recovery_feedback(&mut self, state: &tracking_core::RecoveryState) {
        use tracking_core::RecoveryState;
        let Some(requested) = self.recovery_watch else {
            return;
        };
        match state {
            RecoveryState::Applied { at_secs } if *at_secs >= requested => {
                self.recovery_watch = None;
                self.calibration_audio.play(Prompt::say(
                    Some(Cue::Success),
                    voice_prompts::RECOVERY_APPLIED,
                ));
            }
            RecoveryState::Rejected { at_secs, reason } if *at_secs >= requested => {
                self.recovery_watch = None;
                self.calibration_audio.play(Prompt::say(
                    Some(Cue::Failure),
                    voice_prompts::recovery_declined(reason),
                ));
            }
            RecoveryState::Pending { .. } => {}
            // Replaced (calibration cleared it, undo): nothing left to report.
            _ => self.recovery_watch = None,
        }
    }

    fn request_recovery(&mut self, now: f64, spoken: bool) {
        self.core.request_recovery(now);
        self.write_record(&Record::Recovery { at: now });
        self.recovery_watch = Some(now);
        if spoken {
            self.calibration_audio.play(Prompt::say(
                Some(Cue::Ready),
                voice_prompts::RECOVERY_REQUESTED,
            ));
        }
    }

    fn handle_tap_verdict(&mut self, verdict: TapVerdict, now: f64) {
        let event: TapEvent = match verdict {
            TapVerdict::Gesture(event) => event,
            TapVerdict::Ignored {
                tracker,
                taps,
                reason,
            } => {
                if taps >= 2 {
                    self.gesture_status = format!(
                        "Ignored {taps} taps on {}: {reason}",
                        self.tracker_role_label(&tracker)
                    );
                }
                return;
            }
        };
        let active = active_calibration_phase(self.calibration_view(now).phase);
        let action = gesture_action(event.gesture, active);
        let name = match event.gesture {
            TapGesture::Double => "Double tap",
            TapGesture::Triple => "Triple tap",
        };
        let what = match action {
            GestureAction::StartMovementCalibration => "calibrate by movement",
            GestureAction::CancelCalibration => "cancel calibration",
            GestureAction::RequestHeadingRecovery => "heading recovery",
            GestureAction::Ignore => "ignored during calibration",
        };
        let evidence = if event.peaks_mps2.is_empty() {
            "firmware tap report".to_owned()
        } else {
            let peaks: Vec<String> = event.peaks_mps2.iter().map(|p| format!("{p:.0}")).collect();
            format!("peaks {} m/s²", peaks.join("/"))
        };
        self.gesture_status = format!(
            "{name} on {} ({evidence}): {what}",
            self.tracker_role_label(&event.tracker)
        );
        self.message(self.gesture_status.clone());
        match action {
            GestureAction::StartMovementCalibration => self.begin_motion_request(now, false),
            GestureAction::CancelCalibration => {
                self.calibration_audio.cancel();
                self.calibration_request.cancel(
                    now,
                    "Cancelled by triple tap. No delayed capture will occur.",
                );
                self.end_motion_session(now);
                self.calibration_audio_tick(now);
            }
            GestureAction::RequestHeadingRecovery => self.request_recovery(now, true),
            GestureAction::Ignore => {}
        }
    }

    fn tracker_role_label(&self, id: &str) -> &'static str {
        self.core
            .export_profile()
            .assignments
            .iter()
            .find(|a| a.sensor_id == id)
            .map_or("an unassigned tracker", |a| core_role_label(a.role))
    }

    fn calibration_tick(&mut self, now: f64) {
        let attempt = self.calibration_request.poll(now);
        self.calibration_audio_tick(now);
        if !attempt {
            // A finish attempt completes later than `now`: continue from there.
            let later = self.motion_tick(now);
            self.calibration_audio_tick(later);
            return;
        }
        let previous_config = self.core.config().clone();
        let mut adopted_height = None;
        let result = match self.calibration_request_kind {
            CalibrationRequestKind::NeutralPose | CalibrationRequestKind::MotionPose => {
                let preview = self.request_preview;
                let configured = self.core.config().anthropometrics.standing_eye_height_m;
                let measured = (self.auto_eye_height && !preview)
                    .then(|| {
                        auto_eye_height(
                            configured,
                            self.core.stable_headset_height(now),
                            self.core.config().standing_height_tolerance_m,
                        )
                    })
                    .flatten();
                // The new height is adopted only if this neutral capture
                // passes under it (atomic in the core): a rejected capture
                // leaves height and calibration untouched.
                let adopted = measured.and_then(|height| {
                    config_with_standing_height(self.core.config(), height)
                        .ok()
                        .map(|config| (height, config))
                });
                match adopted {
                    Some((height, config)) => {
                        adopted_height = Some((configured, height));
                        self.core
                            .calibrate_neutral_with_config(now, &self.anchors, preview, config)
                    }
                    None => self.core.calibrate_neutral(now, &self.anchors, preview),
                }
                .map_err(|error| error.to_string())
            }
            CalibrationRequestKind::ForwardPose => self
                .core
                .calibrate_forward(now)
                .map_err(|error| error.to_string()),
            CalibrationRequestKind::StandingHeight => current_standing_headset_height(
                &self.anchors,
                now,
                self.core.config().stale_after_secs,
            )
            .and_then(|height| config_with_standing_height(self.core.config(), height))
            .and_then(|config| {
                self.core
                    .set_config(config)
                    .map_err(|error| error.to_string())
            }),
        };
        let succeeded = result.is_ok();
        let completed_at = self.origin.elapsed().as_secs_f64();
        let result = if succeeded && completed_at - now > self.core.config().stale_after_secs {
            Err("Capture paused too long; waiting for current measurements.".into())
        } else {
            result
        };
        self.calibration_request
            .attempt_result(completed_at, result);
        let phase = self.calibration_view(completed_at).phase;
        let captured = phase == CalibrationRequestPhase::Captured
            || (self.calibration_request_kind == CalibrationRequestKind::MotionPose
                && phase == CalibrationRequestPhase::Moving);
        let adopted_height = adopted_height.filter(|_| succeeded);
        if succeeded && captured {
            match self.calibration_request_kind {
                CalibrationRequestKind::NeutralPose | CalibrationRequestKind::MotionPose => {
                    if let Some((before, after)) = adopted_height {
                        // Replay: the configuration first, then the capture.
                        self.record_configuration(now);
                        self.message(format!(
                            "Eye height updated from the headset: {before:.3} m -> {after:.3} m (body dimensions rescaled)."
                        ));
                        self.height_announce = (after - before).abs() > AUTO_HEIGHT_ANNOUNCE_M;
                    }
                    self.write_record(&Record::Calibrate {
                        at: now,
                        preview: self.request_preview,
                    })
                }
                CalibrationRequestKind::ForwardPose => {
                    self.write_record(&Record::ForwardCalibrate { at: now })
                }
                CalibrationRequestKind::StandingHeight => self.record_configuration(completed_at),
            }
            if phase == CalibrationRequestPhase::Captured {
                let message = self.calibration_view(completed_at).message;
                self.message(message);
            }
            self.save();
            // Movement starts right after the neutral capture.
            let later = self.motion_tick(completed_at);
            self.calibration_audio_tick(later);
            return;
        } else if succeeded {
            // A synchronous operation may cross the fixed deadline. Never keep
            // or publish its late capture, and do not persist it as a success.
            // A height it adopted is discarded with it (nothing was recorded).
            if self.calibration_request_kind == CalibrationRequestKind::StandingHeight
                || adopted_height.is_some()
            {
                let _ = self.core.set_config(previous_config);
            }
            self.core.clear_calibration();
            self.write_record(&Record::Clear { at: completed_at });
        }
        self.calibration_audio_tick(completed_at);
    }

    fn command(&mut self, command: Command, now: f64) -> bool {
        match command {
            Command::Shutdown => {
                self.calibration_audio.cancel();
                return false;
            }
            Command::Assign { id, role } => {
                self.invalidate_calibration_request(
                    now,
                    "Body assignments changed. Start a new countdown when ready.",
                );
                let mut assignments = self.core.export_profile().assignments;
                assignments.retain(|a| a.sensor_id != id);
                if let Some(role) = role.and_then(|s| role_value(&s)) {
                    assignments.retain(|a| a.role != role);
                    assignments.push(RoleAssignment {
                        sensor_id: id,
                        role,
                    });
                }
                if let Err(e) = self.core.set_assignments(assignments) {
                    self.message(e.to_string());
                }
                self.save();
                self.record_configuration(now);
            }
            Command::SetHeight(value) => {
                self.invalidate_calibration_request(
                    now,
                    "Standing eye height changed. Start a new countdown when ready.",
                );
                match config_with_standing_height(self.core.config(), value).and_then(|config| {
                    self.core
                        .set_config(config)
                        .map_err(|error| error.to_string())
                }) {
                    Ok(()) => {
                        self.save();
                        self.record_configuration(now);
                    }
                    Err(error) => self.message(error),
                }
            }
            Command::Calibrate => {
                self.begin_calibration_request(now, CalibrationRequestKind::NeutralPose);
            }
            Command::CalibrateForward => {
                self.begin_calibration_request(now, CalibrationRequestKind::ForwardPose);
            }
            Command::CalibrateMotion => self.begin_motion_request(now, false),
            Command::CalibrateMotionMove => self.begin_motion_request(now, true),
            Command::SetCalibrationMethod(method) => {
                if self.calibration_method != method {
                    self.calibration_method = method;
                    self.save();
                }
            }
            Command::SetManualMount {
                role,
                forward_degrees,
                physical_degrees_xyz,
            } => {
                self.begin_manual_edit(now);
                if let Some(role) = role_value(&role) {
                    let values = tracking_core::manual_mounting::ManualMountValues {
                        forward_degrees,
                        physical_degrees_xyz,
                    };
                    let result = self.core.set_manual_mount(role, values);
                    self.finish_manual_edit(
                        result,
                        Record::ManualMount {
                            at: now,
                            role,
                            values,
                        },
                    );
                }
            }
            Command::ConfirmManualMount { role } => {
                self.begin_manual_edit(now);
                if let Some(role) = role_value(&role) {
                    let result = self.core.confirm_manual_mount(role);
                    self.finish_manual_edit(result, Record::ConfirmManualMount { at: now, role });
                }
            }
            Command::UndoManualMount { role } => {
                self.begin_manual_edit(now);
                if let Some(role) = role_value(&role) {
                    let result = self.core.undo_manual_mount(role);
                    self.finish_manual_edit(result, Record::UndoManualMount { at: now, role });
                }
            }
            Command::ResetManualMount { role } => {
                self.begin_manual_edit(now);
                if let Some(role) = role_value(&role) {
                    let result = self.core.reset_manual_mount(role);
                    self.finish_manual_edit(result, Record::ResetManualMount { at: now, role });
                }
            }
            Command::UseCurrentHeadsetHeight => {
                self.begin_calibration_request(now, CalibrationRequestKind::StandingHeight)
            }
            Command::CancelCalibration => {
                self.calibration_audio.cancel();
                self.calibration_request
                    .cancel(now, "Cancelled by you. No delayed capture will occur.");
                self.end_motion_session(now);
            }
            Command::TestCalibrationSound => self.calibration_audio.play(
                Prompt::say(Some(Cue::Countdown), voice_prompts::TEST_SOUND)
                    .with_fallback(Cue::Ready),
            ),
            Command::RequestRecovery => self.request_recovery(now, false),
            Command::UndoRecovery => {
                if self.core.undo_recovery() {
                    self.write_record(&Record::UndoRecovery { at: now });
                    self.message("Restored heading corrections from before the last recovery.");
                }
            }
            Command::ClearCalibration => {
                self.invalidate_calibration_request(now, "Calibration was cleared.");
                self.core.clear_calibration();
                self.write_record(&Record::Clear { at: now });
                self.save();
            }
            Command::SetOutputMode(mode) => {
                if mode != self.output_mode {
                    if active_calibration_phase(self.calibration_view(now).phase) {
                        self.invalidate_calibration_request(
                            now,
                            "Output mode changed during calibration. Start a new countdown when ready.",
                        );
                    }
                    // Never clears calibration: a live one shows as desktop
                    // preview without the headset, a desktop one is aligned
                    // when a worn headset is used.
                    self.output_mode = mode;
                    self.alignment_next_try = None;
                    self.set_core_anchors(now);
                    self.save();
                }
            }
            Command::SetAutoEyeHeight(value) => {
                if self.auto_eye_height != value {
                    self.auto_eye_height = value;
                    self.save();
                }
            }
            Command::SetCorrection(value) => {
                self.invalidate_calibration_request(
                    now,
                    "Tracking configuration changed. Start a new countdown when ready.",
                );
                let mut config = self.core.config().clone();
                config.enable_joint_yaw_correction = value;
                if let Err(e) = self.core.set_config(config) {
                    self.message(e.to_string());
                }
                self.save();
                self.record_configuration(now);
            }
            Command::SetDesktopMotion(value) => {
                self.invalidate_calibration_request(
                    now,
                    "Desktop movement mode changed; pending capture cancelled.",
                );
                let mut config = self.core.config().clone();
                config.enable_preview_motion = value;
                if let Err(error) = self.core.set_config(config) {
                    self.message(error.to_string());
                }
                self.save();
                self.record_configuration(now);
            }
            Command::SetGesture(value) => {
                self.gesture = value;
                self.taps.reset();
                self.gesture_status.clear();
                self.save();
            }
            Command::SetAudioMode(mode) => {
                if self.calibration_audio.mode() != mode {
                    self.calibration_audio.set_mode(mode);
                    self.save();
                }
            }
            Command::SetEstimator(settings) => {
                match config_with_estimator(self.core.config(), &settings) {
                    Ok(config) => {
                        self.invalidate_calibration_request(
                            now,
                            "Tracking configuration changed. Start a new countdown when ready.",
                        );
                        match self.core.set_config(config) {
                            Ok(()) => {
                                self.estimator = settings;
                                self.save();
                                self.record_configuration(now);
                            }
                            Err(error) => self.message(error.to_string()),
                        }
                    }
                    Err(error) => self.message(format!("Estimator settings rejected: {error}")),
                }
            }
            Command::StartRecording => {
                if self.recording.is_none() {
                    let path = data_dir().join("recordings").join(format!(
                        "session-{}.jsonl",
                        SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_millis()
                    ));
                    // Ordering: the start job must precede every record, so it is
                    // only accepted when the writer queue has room right now.
                    if !self.deferred_jobs.is_empty()
                        || self
                            .writer
                            .try_submit(WriteJob::StartRecording { path: path.clone() })
                            .is_err()
                    {
                        self.message(
                            "Cannot record right now: the disk writer is busy. Try again.",
                        );
                    } else {
                        self.recording = Some(path.display().to_string());
                        self.held_recorded_at.clear();
                        self.write_record(&Record::Header{version:1,config:self.core.config().clone(),profile:self.core.export_profile(),note:"Start recording before calibration for reproducible live-pose replay. Profiles intentionally require fresh session alignment. Accepted firmware-held tokens are recorded at most once per second per sensor.".into()});
                        self.message("Recording inputs and output. Capture a neutral calibration now for reproducible replay.");
                    }
                }
            }
            Command::StopRecording => {
                if self.recording.take().is_some() {
                    self.submit_job(WriteJob::StopRecording);
                }
            }
            Command::SaveProfile => {
                self.save();
                self.flush_save(now, true);
                self.message(format!("Profile location: {}", self.profile_path.display()));
            }
            Command::ExportDiagnostics => {
                let path = data_dir().join("diagnostics.json");
                let snapshot = self.state.lock().unwrap_or_else(|e| e.into_inner()).clone();
                match serde_json::to_vec_pretty(&snapshot) {
                    Ok(bytes) => self.submit_job(WriteJob::Diagnostics { path, bytes }),
                    Err(e) => self.message(format!("Diagnostics export failed: {e}")),
                }
            }
        }
        true
    }

    fn record_configuration(&mut self, at: f64) {
        self.write_record(&Record::Configuration {
            at,
            config: self.core.config().clone(),
            profile: self.core.export_profile(),
        });
    }

    fn begin_manual_edit(&mut self, now: f64) {
        self.invalidate_calibration_request(
            now,
            "Manual mounting edit started; any pending pose capture was cancelled.",
        );
    }

    fn finish_manual_edit(&mut self, result: Result<(), tracking_core::CoreError>, record: Record) {
        match result {
            Ok(()) => {
                self.write_record(&record);
                self.save();
                self.manual_mount_message = "Manual mounting updated. Check actual movement; live output (if the mode sends any) uses the edited mounting immediately.".into();
                self.message(self.manual_mount_message.clone());
            }
            Err(error) => {
                self.manual_mount_message = format!("Manual mounting: {error}");
                self.message(self.manual_mount_message.clone());
            }
        }
    }

    /// Drains anchors and tracker datagrams; returns whether new measured
    /// evidence (a measured quaternion or a new anchor frame) arrived.
    fn receive(&mut self, now: f64) -> bool {
        let anchor_frame_before = self.last_anchor_frame;
        let anchor_result = self.bridge.as_mut().map(|bridge| bridge.recv_anchors());
        let anchor_frame = match anchor_result {
            Some(Ok(frame)) => frame,
            Some(Err(error)) => {
                self.bridge_error(now, "anchor receive", error);
                None
            }
            None => None,
        };
        let transition = anchor_frame.as_ref().map(|frame| {
            anchor_transition(
                self.last_anchor_frame,
                self.standing_reference.as_ref(),
                frame,
            )
        });
        if let Some(bridge) = &mut self.bridge
            && let Some(frame) = anchor_frame
            && let Some(transition) = transition
            && transition != AnchorTransition::Duplicate
        {
            let received_at =
                (now - bridge.anchors_age().unwrap_or_default().as_secs_f64()).max(0.0);
            // A new helper session (SteamVR restarted) or a transform change
            // is routine. It is reported to the core as a possible event; the
            // headset's standing-space continuity decides (never a clear).
            if let Some(transform_yaw) = tracking_space_event(
                &transition,
                self.standing_reference.as_ref(),
                &frame.raw_from_standing,
            ) {
                self.core.note_tracking_space_event(now, transform_yaw);
                self.write_record(&Record::TrackingSpaceEvent {
                    at: now,
                    transform_yaw_radians: transform_yaw,
                });
                if self.calibration_request_kind == CalibrationRequestKind::StandingHeight
                    && active_calibration_phase(self.calibration_request.view(now).phase)
                {
                    self.invalidate_calibration_request(
                        now,
                        "The headset session or standing floor changed. Start a new height countdown.",
                    );
                }
                self.standing_reference = Some(frame.raw_from_standing);
            }
            if self.standing_reference.is_none() {
                self.standing_reference = Some(frame.raw_from_standing);
            }
            self.last_anchor_frame = Some((frame.session, frame.sequence));
            self.last_anchor_received_at = Some(received_at);
            self.raw_from_standing = frame.raw_from_standing;
            // Cached bridge snapshots retain their original receipt time. Re-stamping
            // them each tick would fake fresh HMD history and extend the timeout.
            // Velocities are standing-space and already validated by the bridge.
            let convert = |p: steamvr_bridge::Pose| {
                Some(AnchorPose {
                    position: p.position,
                    orientation_xyzw: [p.rotation[1], p.rotation[2], p.rotation[3], p.rotation[0]],
                    received_at_secs: received_at,
                    valid: p.valid,
                    linear_velocity: Some(p.linear_velocity),
                    angular_velocity: Some(p.angular_velocity),
                })
            };
            self.steamvr_anchors = Anchors {
                hmd: convert(frame.poses[0]),
                left_hand: convert(frame.poses[1]),
                right_hand: convert(frame.poses[2]),
            };
            self.set_core_anchors(now);
        }
        let usable_height = usable_height_anchor(
            &self.steamvr_anchors,
            now,
            self.core.config().stale_after_secs,
        )
        .map(|head| head.position[1]);
        self.presence.observe(
            now,
            usable_height,
            self.core.config().anthropometrics.standing_eye_height_m,
        );
        let mut new_data = self.last_anchor_frame != anchor_frame_before;
        let (measurements, reconnected, taps, notices, revocations) =
            if let Some(hub) = &mut self.hub {
                let samples = hub.poll(now);
                let reconnected = std::mem::take(&mut hub.reconnected);
                let taps = std::mem::take(&mut hub.taps);
                let notices = [hub.capture_error.take(), hub.notice.take()];
                let revocations = std::mem::take(&mut hub.revocations);
                (samples, reconnected, taps, notices, revocations)
            } else {
                (
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                    [None, None],
                    Default::default(),
                )
            };
        for notice in notices.into_iter().flatten() {
            self.message(notice);
        }
        let assignments = self.core.export_profile().assignments;
        // Only a restarted IMU that carries a body role voids the calibration; a
        // spare tracker rebooting, or a same-boot Wi-Fi re-handshake, does not.
        if reconnect_affects_assignment(&reconnected, &assignments) {
            self.invalidate_calibration_request(
                now,
                "An assigned tracker restarted. Start a new countdown after reconnection.",
            );
            let was_calibrated = self.core.calibration_forward().is_some();
            self.core.clear_calibration();
            self.message("An assigned tracker restarted. Recalibrate before live output.");
            self.write_record(&Record::Clear { at: now });
            if was_calibrated {
                // The user in the headset hears why tracking stopped.
                self.calibration_audio.play(Prompt::say(
                    Some(Cue::Failure),
                    voice_prompts::tracker_restarted(self.gesture),
                ));
            }
        }
        for measurement in measurements {
            // Every assigned tracker feeds the tap recogniser (whole-body
            // events are recognised across trackers); only chest/hip taps count.
            if self.gesture
                && matches!(
                    measurement.provenance,
                    tracking_core::SensorProvenance::Measured
                )
                && let Some(acc) = measurement.acceleration
                && let Some(role) = assignments
                    .iter()
                    .find(|a| a.sensor_id == measurement.id)
                    .map(|a| a.role)
            {
                let torso = matches!(role, Role::Chest | Role::Hip);
                self.taps.sample(
                    &measurement.id,
                    torso,
                    measurement.at,
                    acc,
                    measurement.orientation,
                );
                let known = self.accel_seen_at.len();
                match self.accel_seen_at.get_mut(&measurement.id) {
                    Some(at) => *at = measurement.at,
                    None if known < 64 => {
                        self.accel_seen_at
                            .insert(measurement.id.clone(), measurement.at);
                    }
                    None => {}
                }
            }
            // Firmware 0.7.2 sends gravity-removed linear acceleration in the sensor
            // body frame, whose axes are not aligned with the quaternion frame. The
            // core consumes only its magnitude, which is frame-independent.
            let held = matches!(
                measurement.provenance,
                tracking_core::SensorProvenance::FirmwareHeld { .. }
            );
            let sample = SensorSample {
                id: measurement.id,
                orientation_xyzw: measurement.orientation,
                received_at_secs: measurement.at,
                acceleration_mps2: measurement
                    .acceleration
                    .filter(|a| a.iter().all(|v| v.is_finite())),
                provenance: measurement.provenance,
            };
            // A rejected ingest still mutates core state; record every attempt.
            let rejected = self
                .core
                .ingest(sample.clone())
                .err()
                .map(|e| e.to_string());
            if !held && rejected.is_none() {
                new_data = true;
                if self.pending_latency.len() < 4096 {
                    self.pending_latency.push(sample.received_at_secs);
                }
            }
            if self.recording.is_some() {
                // Accepted held tokens repeat every wake; keep at most one per second.
                let skip =
                    held && rejected.is_none()
                        && self.held_recorded_at.get(&sample.id).is_some_and(|last| {
                            now - last < HELD_RECORD_INTERVAL_SECS && now >= *last
                        });
                if !skip {
                    if held && rejected.is_none() {
                        self.held_recorded_at.insert(sample.id.clone(), now);
                    }
                    self.send_record(Record::Sample { sample, rejected });
                }
            }
        }
        for (id, unhealthy) in revocations {
            if unhealthy {
                self.core.mark_sensor_unhealthy(&id);
            } else {
                self.core.revoke_firmware_hold(&id);
            }
            self.write_record(&Record::RevokeHold {
                at: now,
                id,
                unhealthy,
            });
        }
        if self.gesture {
            // Firmware double-tap packets stand in for the recogniser only on
            // torso trackers that do not stream acceleration.
            for id in taps {
                let torso = assignments
                    .iter()
                    .any(|a| a.sensor_id == id && matches!(a.role, Role::Chest | Role::Hip));
                let streams_acceleration =
                    self.accel_seen_at.get(&id).is_some_and(|at| now - at < 1.0);
                if torso && !streams_acceleration {
                    self.handle_tap_verdict(
                        TapVerdict::Gesture(TapEvent {
                            gesture: TapGesture::Double,
                            tracker: id,
                            first_tap_at: now,
                            last_tap_at: now,
                            peaks_mps2: Vec::new(),
                        }),
                        now,
                    );
                }
            }
            for verdict in self.taps.poll(now) {
                self.handle_tap_verdict(verdict, now);
            }
        }
        new_data
    }

    /// Anchors fed to the core and recorded: the SteamVR anchors, or none in
    /// Desktop-only mode (the headset is ignored entirely).
    fn set_core_anchors(&mut self, now: f64) {
        self.anchors = if self.output_mode.ignores_headset() {
            Anchors::default()
        } else {
            self.steamvr_anchors.clone()
        };
        self.write_record(&Record::Anchors {
            at: now,
            anchors: self.anchors.clone(),
        });
    }

    /// Whether new work (a calibration started now) treats the session as desktop.
    fn desktop_session(&self) -> bool {
        output_mode::desktop_session(self.output_mode, self.presence.present())
    }

    fn publish_state(&mut self, frame: &TrackingFrame, now: f64) -> PublishState {
        PublishState {
            calibrating: active_calibration_phase(self.calibration_view(now).phase),
            calibrated: frame.calibration.calibrated,
            direction_calibrated: self.core.direction_calibrated(),
            preview_calibration: frame.calibration.preview_only,
            frame_live: !frame.preview_only && frame.live_anchor,
            suspended: self.suspended,
        }
    }

    /// Carries a desktop calibration over to a worn headset (every mode but
    /// Desktop only) once the user stands straight and still. Announces once
    /// per headset appearance, then "Aligned." on success.
    fn alignment_tick(&mut self, now: f64, state: &PublishState) {
        if !output_mode::wants_alignment(self.output_mode, self.presence.present(), state) {
            self.alignment_next_try = None;
            self.alignment_status.clear();
            return;
        }
        let Some(next_try) = self.alignment_next_try else {
            self.alignment_next_try = Some(now);
            self.alignment_status = "Stand straight and still, looking ahead.".into();
            self.calibration_audio.play(Prompt::say(
                Some(Cue::Ready),
                voice_prompts::HEADSET_FOUND_ALIGN,
            ));
            self.message(
                "Headset found: stand straight and still, looking ahead, to align the desktop calibration with it.",
            );
            return;
        };
        if now < next_try {
            return;
        }
        self.alignment_next_try = Some(now + ALIGN_TRY_INTERVAL_SECS);
        match self.core.align_preview_to_headset(now, &self.anchors) {
            Ok(()) => {
                self.write_record(&Record::AlignToHeadset { at: now });
                self.alignment_next_try = None;
                self.alignment_status.clear();
                self.calibration_audio
                    .play(Prompt::say(Some(Cue::Success), voice_prompts::ALIGNED));
                self.message(
                    "Desktop calibration aligned with the headset; live tracking can start.",
                );
            }
            Err(error) => self.alignment_status = error.to_string(),
        }
    }

    /// Reports core events the user should know about: a rebase of the
    /// standing space, and headset tracking resuming after a long gap (once,
    /// by voice and message).
    fn tracking_space_feedback(&mut self, frame: &TrackingFrame) {
        let status = &frame.estimator;
        if status.standing_space_rebases > self.seen_rebases {
            self.seen_rebases = status.standing_space_rebases;
            let evidence = match status.last_rebase_evidence {
                Some(tracking_core::tracking_space::RebaseEvidence::Transform) => {
                    "headset jump matching the SteamVR transform change"
                }
                Some(tracking_core::tracking_space::RebaseEvidence::ObservedJump) => "headset jump",
                None => "explicit",
            };
            self.message(format!(
                "SteamVR tracking space changed ({evidence}); calibration rotated by {:.0} degrees.",
                status.last_rebase_degrees
            ));
        }
        if status.headset_resumptions > self.seen_resumptions {
            self.seen_resumptions = status.headset_resumptions;
            self.message(format!(
                "Headset tracking resumed after {:.0} s; calibration kept. If the body faces the wrong way, reset heading (double tap or Recover heading).",
                status.last_headset_gap_secs
            ));
            if !active_calibration_phase(self.calibration_request.view(frame.at_secs).phase) {
                self.calibration_audio.play(Prompt::say(
                    Some(Cue::Ready),
                    voice_prompts::tracking_resumed(self.gesture),
                ));
            }
        }
    }

    /// Bridge failures are surfaced, rate limited, instead of silently ignored.
    fn bridge_error(&mut self, now: f64, context: &str, error: io::Error) {
        if now >= self.bridge_next_message {
            self.bridge_next_message = now + BRIDGE_MESSAGE_INTERVAL_SECS;
            self.message(format!("SteamVR bridge {context} failed: {error}"));
        }
    }

    fn publish(&mut self, frame: &TrackingFrame, now: f64) {
        let state = self.publish_state(frame, now);
        self.sink = output_mode::output_sink(self.output_mode, &state);
        self.sequence += 1;
        let poses = steamvr_poses(&frame.targets, self.sink == OutputSink::SteamVr);
        // The bridge sends every pose invalid if this exceeds its 250 ms limit.
        let pose_age_secs = (self.clock() - frame.at_secs).max(0.0) as f32;
        let mut send_error = None;
        if let Some(bridge) = &self.bridge
            && let Err(error) = bridge.send_targets(&steamvr_bridge::Trackers {
                session: self.session,
                sequence: self.sequence,
                raw_from_standing: self.raw_from_standing,
                pose_age_secs,
                apply_angular_velocity: self.estimator.angular_velocity_output,
                poses,
            })
        {
            send_error = Some(error);
        }
        if let Some(error) = send_error {
            self.bridge_error(now, "target send", error);
        }
        if self.sink == OutputSink::Osc && now >= self.next_osc {
            self.next_osc = advance_deadline(self.next_osc, now, OSC_INTERVAL_SECS);
            if let Some(socket) = &self.osc {
                for t in frame.targets.iter().filter(|t| t.valid) {
                    if let Some((p, r)) = outputs::unity_pose(t.position, t.orientation_xyzw) {
                        let id = target_index(t.role) + 1;
                        for (suffix, value) in [("position", p), ("rotation", r)] {
                            if let Some(packet) = outputs::osc_vec3(
                                &format!("/tracking/trackers/{id}/{suffix}"),
                                value,
                            ) {
                                let _ = socket.send_to(&packet, "127.0.0.1:9000");
                            }
                        }
                    }
                }
            }
        }
    }

    fn snapshot(&mut self, frame: &TrackingFrame, now: f64, solve_micros: f64) {
        let mut trackers = self.hub.as_ref().map(|h| h.views(now)).unwrap_or_default();
        let profile = self.core.export_profile();
        for t in &mut trackers {
            t.role = profile
                .assignments
                .iter()
                .find(|a| a.sensor_id == t.id)
                .map(|a| role_name(a.role));
        }
        let indices: BTreeMap<_, _> = frame
            .joints
            .iter()
            .enumerate()
            .map(|(i, j)| (j.joint, i))
            .collect();
        let fresh = |anchor: &Option<AnchorPose>| {
            anchor
                .as_ref()
                .is_some_and(|a| a.valid && now - a.received_at_secs < 0.25)
        };
        let recovery = match &frame.recovery {
            tracking_core::RecoveryState::Idle => {
                "Stand neutral and face forward with your torso before requesting recovery.".into()
            }
            tracking_core::RecoveryState::Pending { blocked_reason, .. } => format!(
                "Waiting: {}",
                blocked_reason.as_deref().unwrap_or("hold neutral still")
            ),
            tracking_core::RecoveryState::Applied { .. } => {
                "Heading recovery applied; mounting preserved.".into()
            }
            tracking_core::RecoveryState::Rejected { reason, .. } => {
                format!("Recovery declined: {reason}")
            }
            tracking_core::RecoveryState::Undone => "Recovery undone.".into(),
        };
        let correction = if frame.corrections.is_empty() {
            "Joint correction is off.".into()
        } else {
            frame
                .corrections
                .iter()
                .map(|c| {
                    format!(
                        "{:?}: {} ({:.1}°)",
                        c.thigh_role, c.status, c.residual_degrees
                    )
                })
                .collect::<Vec<_>>()
                .join(" · ")
        };
        let calibration_request = self.calibration_view(now);
        let request_phase = calibration_request.phase;
        let (latency_p50, latency_p95) = self.stats.latency_percentiles();
        let status = if active_calibration_phase(calibration_request.phase) {
            calibration_request.message.clone()
        } else if self.hub.is_none() {
            "Tracker receiver unavailable".into()
        } else if trackers.is_empty() {
            "Waiting for Wi-Fi tracker packets on UDP 6969".into()
        } else if frame.calibration.calibrated {
            if !self.core.direction_calibrated() {
                match self.calibration_method {
                    CalibrationMethod::Movement => {
                        "Neutral pose captured; movement calibration is still needed".into()
                    }
                    CalibrationMethod::SkiPose => {
                        "Neutral pose captured; forward-bend calibration is still needed".into()
                    }
                }
            } else if frame.calibration.preview_only {
                "Desktop calibration: preview only, nothing is sent".into()
            } else if self.output_mode.ignores_headset() {
                "Live calibration shown as desktop preview (Desktop only ignores the headset)"
                    .into()
            } else if frame.live_anchor {
                "Live calibrated tracking".into()
            } else {
                "Live calibration kept; waiting for a fresh SteamVR headset pose".into()
            }
        } else {
            frame
                .calibration
                .reason
                .clone()
                .unwrap_or_else(|| "Assign trackers, then capture neutral pose".into())
        };
        let publish_state = self.publish_state(frame, now);
        let mode_status = output_mode::mode_status(
            self.output_mode,
            self.presence.present(),
            &publish_state,
            self.sink,
        );
        let snapshot = Snapshot {
            trackers,
            joints: frame
                .joints
                .iter()
                .map(|j| JointView {
                    name: format!("{:?}", j.joint),
                    position: j.position,
                })
                .collect(),
            bones: frame
                .bones
                .iter()
                .filter_map(|b| Some([*indices.get(&b.start)?, *indices.get(&b.end)?]))
                .collect(),
            targets: frame
                .targets
                .iter()
                .map(|t| TargetView {
                    name: format!("{:?}", t.role),
                    position: t.position,
                    orientation: t.orientation_xyzw,
                    confidence: t.confidence,
                    valid: t.valid,
                    source: format!(
                        "Position {:?}; orientation {:?}",
                        t.position_source, t.orientation_source
                    ),
                })
                .collect(),
            messages: self.messages.clone(),
            status,
            calibrated: frame.calibration.calibrated,
            calibration_request,
            calibration_request_kind: self.calibration_request_kind,
            hmd_standing_height_m: usable_height_anchor(
                &self.steamvr_anchors,
                now,
                self.core.config().stale_after_secs,
            )
            .map(|head| head.position[1]),
            hmd_forward: current_headset_forward(
                &self.anchors,
                now,
                self.core.config().stale_after_secs,
            ),
            calibration_forward: self.core.calibration_forward(),
            direction_calibrated: self.core.direction_calibrated(),
            manual_mounts: self
                .core
                .manual_mount_adjustments()
                .into_iter()
                .map(|mount| ManualMountView {
                    role: role_name(mount.role),
                    forward_degrees: mount.values.forward_degrees,
                    physical_degrees_xyz: mount.values.physical_degrees_xyz,
                    confirmed: mount.confirmed,
                    can_undo: mount.can_undo,
                })
                .collect(),
            manual_mount_message: self.manual_mount_message.clone(),
            desktop_motion_enabled: self.core.config().enable_preview_motion,
            desktop_motion_status: frame.preview_motion.reason.clone(),
            desktop_root_translation: frame.preview_motion.root_translation_m,
            calibration_height_tolerance_m: self.core.config().standing_height_tolerance_m,
            calibration_preview: frame.calibration.preview_only,
            hmd_connected: fresh(&self.steamvr_anchors.hmd),
            hands_connected: [
                fresh(&self.steamvr_anchors.left_hand),
                fresh(&self.steamvr_anchors.right_hand),
            ],
            steamvr_connected: self.bridge.as_ref().is_some_and(|b| {
                b.anchors_age()
                    .is_some_and(|a| a < Duration::from_millis(250))
            }),
            output_mode: self.output_mode,
            output_sink: self.sink,
            mode_status,
            alignment_status: self.alignment_status.clone(),
            desktop_session: self.desktop_session(),
            auto_eye_height: self.auto_eye_height,
            experimental_correction: self.core.config().enable_joint_yaw_correction,
            recovery,
            correction,
            recording: self.recording.clone(),
            uptime_seconds: now,
            solve_micros,
            solve_rate_hz: self.stats.rate_hz(now),
            packet_to_solve_p50_ms: latency_p50,
            packet_to_solve_p95_ms: latency_p95,
            receive_dropped: self.hub.as_ref().map_or(0, |h| h.receive_dropped),
            packets_received: self.hub.as_ref().map_or(0, |h| h.packets),
            malformed_packets: self.hub.as_ref().map_or(0, |h| h.malformed),
            datagram_types: self
                .hub
                .as_ref()
                .map_or_else(Default::default, |h| h.datagram_types.clone()),
            payload_types: self
                .hub
                .as_ref()
                .map_or_else(Default::default, |h| h.payload_types.clone()),
            sequence_missing: self.hub.as_ref().map_or(0, |h| h.sequence_missing),
            sequence_duplicates: self.hub.as_ref().map_or(0, |h| h.sequence_duplicates),
            sequence_out_of_order: self.hub.as_ref().map_or(0, |h| h.sequence_out_of_order),
            transport_resets: self.hub.as_ref().map_or(0, |h| h.transport_resets),
            profile_path: self.profile_path.display().to_string(),
            height_m: self.core.config().anthropometrics.standing_eye_height_m,
            gesture_enabled: self.gesture,
            estimator_settings: self.estimator,
            estimator: frame.estimator.clone(),
            floor_mismatch_m: frame.calibration.floor_mismatch_m,
            calibration_method: self.calibration_method,
            motion_calibration: (self.calibration_request_kind
                == CalibrationRequestKind::MotionPose
                && request_phase != CalibrationRequestPhase::Idle)
                .then(|| self.motion_status.clone())
                .flatten(),
            motion_move_available: !active_calibration_phase(request_phase)
                && self.core.motion_calibration_available(now).is_ok(),
            motion_elapsed_secs: self.calibration_request.moving_secs(now),
            audio_mode: self.calibration_audio.mode(),
            voice_status: self.calibration_audio.voice_status(),
            gesture_status: self.gesture_status.clone(),
        };
        *self.state.lock().unwrap_or_else(|e| e.into_inner()) = snapshot;
    }

    fn clock(&self) -> f64 {
        self.origin.elapsed().as_secs_f64()
    }

    /// Event-driven loop: wake on tracker datagrams (stamped on receipt by the
    /// transport thread) or on the next duty deadline; drain on every wake;
    /// solve and publish at 100-250 Hz according to `solve_due`.
    fn run(mut self) {
        let _timer = TimerResolution::request();
        raise_current_thread_priority();
        let mut last_solve: Option<f64> = None;
        let mut new_data = false;
        let mut next_housekeeping = 0.0;
        let mut next_alignment = 0.0;
        loop {
            self.health.beat();
            let now = self.clock();
            let wait = wait_budget(now, last_solve, new_data, next_housekeeping);
            if wait > 0.0 {
                let timeout = Duration::from_secs_f64(wait);
                match &mut self.hub {
                    Some(hub) => {
                        hub.wait(timeout);
                    }
                    None => thread::sleep(timeout),
                }
            }
            let now = self.clock();
            let housekeeping = now >= next_housekeeping;
            if housekeeping {
                next_housekeeping =
                    advance_deadline(next_housekeeping, now, HOUSEKEEPING_INTERVAL_SECS);
                self.helper_tick(now);
                self.steamvr_connection_tick(now);
                self.writer_tick(now);
            }
            new_data |= self.receive(now);
            let mut running = true;
            for _ in 0..MAX_COMMANDS_PER_WAKE {
                match self.commands.try_recv() {
                    Ok(command) => {
                        if !self.command(command, now) {
                            running = false;
                            break;
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        running = false;
                        break;
                    }
                }
            }
            if !running {
                break;
            }
            if housekeeping {
                let calibration_now = self.clock();
                self.calibration_tick(calibration_now);
            }
            // attempt_result records its actual completion time. Every later
            // controller view must use a newer clock, never the old tick time.
            let now = self.clock();
            if !solve_due(now, last_solve, new_data) {
                continue;
            }
            let started = Instant::now();
            let frame = self.core.solve(now, &self.anchors);
            let micros = started.elapsed().as_secs_f64() * 1e6;
            self.recovery_feedback(&frame.recovery);
            self.tracking_space_feedback(&frame);
            self.publish(&frame, now);
            if housekeeping_due_alignment(&mut next_alignment, now) {
                let state = self.publish_state(&frame, now);
                self.alignment_tick(now, &state);
            }
            self.stats.record(now, &mut self.pending_latency);
            last_solve = Some(now);
            new_data = false;
            if now - self.last_ui >= 1. / 30. || now < self.last_ui {
                self.last_ui = now;
                self.snapshot(&frame, now, micros);
            }
            if self.recording.is_some() {
                self.send_record(Record::Frame { frame });
            }
            if now - self.last_flush >= 1. || now < self.last_flush {
                self.last_flush = now;
                if self.recording.is_some() {
                    let trackers = self.hub.as_ref().map(|h| h.views(now)).unwrap_or_default();
                    self.send_record(Record::Transport { at: now, trackers });
                    if self.recording.is_some() {
                        self.submit_job(WriteJob::FlushRecording);
                    }
                }
            }
        }
        self.suspended = true;
        let frame = self.core.solve(self.clock(), &self.anchors);
        self.publish(&frame, self.clock());
        let now = self.clock();
        self.flush_save(now, true);
        if self.recording.take().is_some() {
            self.deferred_jobs.push_back(WriteJob::StopRecording);
        }
        let remaining: Vec<_> = self.deferred_jobs.drain(..).collect();
        self.writer.finish(remaining);
        if let Some(pending) = self.helper_spawn.take()
            && let Ok(Ok(child)) = pending.recv()
        {
            self.helper = Some(child);
        }
        if let Some(mut helper) = self.helper.take() {
            let _ = helper.kill();
            let _ = helper.wait();
        }
    }
}

pub fn replay(path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let summary = replay_reader(BufReader::new(File::open(path)?))?;
    let ReplaySummary {
        frames,
        valid,
        max_difference,
        outcome_mismatches,
    } = summary;
    println!(
        "Replayed {frames} frames; {valid} frames with valid targets; max output position difference {max_difference:.6} m; {outcome_mismatches} sample ingest outcomes differed from the recording. This measures deterministic replay, not physical accuracy. No network output was sent."
    );
    Ok(())
}

#[derive(Debug, Default, PartialEq)]
struct ReplaySummary {
    frames: u64,
    valid: u64,
    max_difference: f32,
    outcome_mismatches: u64,
}

fn replay_reader(reader: impl BufRead) -> Result<ReplaySummary, Box<dyn std::error::Error>> {
    let mut core = TrackingCore::new(TrackingConfig::default())?;
    let mut anchors = Anchors::default();
    let mut frames = 0u64;
    let mut valid = 0u64;
    let mut max_difference = 0f32;
    let mut outcome_mismatches = 0u64;
    for (index, line) in reader.lines().enumerate() {
        let line = line?;
        let record: Record =
            serde_json::from_str(&line).map_err(|e| format!("line {}: {e}", index + 1))?;
        match record {
            Record::Header {
                version,
                config,
                profile,
                ..
            } => {
                if version != 1 {
                    return Err("unsupported recording version".into());
                }
                core = TrackingCore::new(config)?;
                core.restore_profile(profile)?;
            }
            Record::Configuration {
                config, profile, ..
            } => {
                core.set_config(config)?;
                core.set_assignments(profile.assignments)?;
            }
            Record::Sample { sample, rejected } => {
                // A rejected ingest changes core state too: replay it, never abort.
                // Recordings without the field logged accepted samples only.
                let replayed_rejected = core.ingest(sample).is_err();
                if replayed_rejected != rejected.is_some() {
                    outcome_mismatches += 1;
                }
            }
            Record::Anchors { anchors: a, .. } => anchors = a,
            Record::Transport { .. } => {}
            Record::Frame { frame } => {
                let result = core.solve(frame.at_secs, &anchors);
                frames += 1;
                if result.targets.iter().any(|t| t.valid) {
                    valid += 1;
                }
                for (a, b) in result.targets.iter().zip(&frame.targets) {
                    let d = glam::Vec3::from(a.position).distance(glam::Vec3::from(b.position));
                    max_difference = max_difference.max(d);
                }
            }
            // Lifecycle events, in file order, shared with `--dump`. A recorded
            // event that fails now means the replay diverged: stop.
            event => {
                if let Some((_, _, result)) =
                    crate::replay::apply_event(&mut core, &anchors, &event)
                {
                    result?;
                }
            }
        }
    }
    Ok(ReplaySummary {
        frames,
        valid,
        max_difference,
        outcome_mismatches,
    })
}

#[cfg(test)]
mod worker_loop_tests {
    use super::*;

    #[test]
    fn pacing_solves_on_new_data_at_most_250_hz_and_at_least_100_hz() {
        assert!(solve_due(0.0, None, false));
        // New data before 4 ms waits; after 4 ms solves.
        assert!(!solve_due(1.003, Some(1.0), true));
        assert!(solve_due(1.004, Some(1.0), true));
        // Without data, solve only once 10 ms passed.
        assert!(!solve_due(1.009, Some(1.0), false));
        assert!(solve_due(1.010, Some(1.0), false));
        assert!(solve_due(0.5, Some(1.0), false));
        // Waits never exceed the 10 ms floor rate and end at the right deadline.
        assert!((wait_budget(1.001, Some(1.0), true, 9.0) - 0.003).abs() < 1e-9);
        assert!((wait_budget(1.001, Some(1.0), false, 9.0) - 0.009).abs() < 1e-9);
        assert!((wait_budget(1.001, Some(1.0), false, 1.005) - 0.004).abs() < 1e-9);
        assert_eq!(wait_budget(2.0, Some(1.0), false, 9.0), 0.0);
        assert_eq!(wait_budget(0.0, None, false, 9.0), 0.0);
        // Simulated datagram stream at 1 kHz: solve rate is capped at 250 Hz.
        let (mut last, mut solves) = (None, 0);
        for step in 0..1000 {
            let now = step as f64 * 0.001;
            if solve_due(now, last, true) {
                last = Some(now);
                solves += 1;
            }
        }
        assert!((190..=251).contains(&solves), "{solves}");
        // Idle: exactly the 100 Hz floor.
        let (mut last, mut solves, mut now) = (Some(0.0), 0, 0.0);
        while now < 1.0 - 1e-9 {
            now += wait_budget(now, last, false, f64::INFINITY).max(1e-6);
            if solve_due(now, last, false) {
                last = Some(now);
                solves += 1;
            }
        }
        assert!((99..=101).contains(&solves), "{solves}");
    }

    #[test]
    fn osc_deadline_holds_60_hz_on_a_100_hz_solve_cadence() {
        let (mut deadline, mut sends) = (0.0, 0);
        for step in 0..1000 {
            let now = step as f64 * 0.010;
            if now >= deadline {
                deadline = advance_deadline(deadline, now, OSC_INTERVAL_SECS);
                sends += 1;
            }
        }
        // 10 s of solves at 100 Hz.
        assert!((598..=601).contains(&sends), "{sends}");
        // After a long pause the deadline restarts instead of bursting.
        assert_eq!(advance_deadline(1.0, 5.0, 0.5), 5.5);
    }

    #[test]
    fn solve_stats_report_rate_and_latency_percentiles() {
        let mut stats = SolveStats::default();
        for step in 0..200 {
            let now = step as f64 * 0.005;
            let mut received = vec![now - 0.002, now - 0.010];
            stats.record(now, &mut received);
            assert!(received.is_empty());
        }
        let now = 199. * 0.005;
        assert!((stats.rate_hz(now) - 200.).abs() <= 1.);
        let (p50, p95) = stats.latency_percentiles();
        assert!((p50.unwrap() - 2.0).abs() < 0.01);
        assert!((p95.unwrap() - 10.0).abs() < 0.01);
        assert_eq!(SolveStats::default().latency_percentiles(), (None, None));
    }

    #[test]
    fn only_reconnects_of_assigned_trackers_void_calibration() {
        let assignments = vec![RoleAssignment {
            sensor_id: "aa:bb/0".into(),
            role: Role::Hip,
        }];
        assert!(!reconnect_affects_assignment(&[], &assignments));
        for (identity, affected) in [("cc:dd", false), ("aa:b", false), ("aa:bb", true)] {
            let identities = [identity.to_owned()];
            assert_eq!(
                reconnect_affects_assignment(&identities, &assignments),
                affected
            );
        }
    }

    #[test]
    fn unusable_profile_is_moved_aside_not_overwritten() {
        let dir = std::env::temp_dir().join(format!(
            "chadfbt-profile-test-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("profile.json");
        fs::write(&path, b"{ not json").unwrap();
        let backup = preserve_unusable_profile(&path).unwrap();
        assert!(!path.exists());
        assert_eq!(fs::read(&backup).unwrap(), b"{ not json");
        assert!(
            backup
                .file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("profile.json.corrupt-")
        );
        write_profile_atomically(&path, b"{}").unwrap();
        assert_eq!(fs::read(&backup).unwrap(), b"{ not json");
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn disk_writer_caps_recording_size_with_a_visible_stop() {
        let dir = std::env::temp_dir().join(format!(
            "chadfbt-writer-test-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let path = dir.join("rec.jsonl");
        let mut writer = DiskWriter::spawn(200, 64);
        writer
            .try_submit(WriteJob::StartRecording { path: path.clone() })
            .ok()
            .unwrap();
        for at in 0..20 {
            writer
                .try_submit(WriteJob::Record(Box::new(Record::Clear { at: at as f64 })))
                .ok()
                .unwrap();
        }
        writer.finish([]);
        let written = fs::read_to_string(&path).unwrap();
        assert!(written.len() <= 200 && !written.is_empty());
        assert!(
            written
                .lines()
                .all(|line| serde_json::from_str::<Record>(line).is_ok())
        );
        let notices: Vec<_> = writer.notices.try_iter().collect();
        assert!(notices.iter().any(|notice| matches!(
            notice,
            WriterNotice::RecordingStopped { message, .. } if message.contains("size limit")
        )));
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn replay_continues_past_rejected_samples_and_compares_outcomes() {
        let sample = |at: f64| SensorSample {
            id: "tracker/0".into(),
            orientation_xyzw: [0., 0., 0., 1.],
            received_at_secs: at,
            acceleration_mps2: None,
            provenance: tracking_core::SensorProvenance::Measured,
        };
        let line = |record: Record| serde_json::to_string(&record).unwrap() + "\n";
        let mut text = String::new();
        text += &line(Record::Sample {
            sample: sample(2.0),
            rejected: None,
        });
        // Out-of-order sample: rejected live, recorded with its outcome.
        text += &line(Record::Sample {
            sample: sample(1.0),
            rejected: Some("Out-of-order sensor sample".into()),
        });
        // Old-format line (accepted only) that replay now rejects: counted, not fatal.
        text += "{\"event\":\"sample\",\"sample\":{\"id\":\"tracker/0\",\"orientation_xyzw\":[0,0,0,1],\"received_at_secs\":0.5,\"acceleration_mps2\":null}}\n";
        text += &line(Record::Clear { at: 2.1 });
        let summary = replay_reader(text.as_bytes()).unwrap();
        assert_eq!(summary.outcome_mismatches, 1);
    }

    #[test]
    fn replay_reproduces_a_recorded_movement_calibration_with_cancel_and_finish() {
        use glam::{Quat, Vec3};
        use tracking_core::chain::ChainDimensions;
        use tracking_core::synthetic::{BodyGenerator, BodyGeometry, Motion, StandState};
        let generator = BodyGenerator::new(
            ChainDimensions::default(),
            BodyGeometry::default(),
            StandState {
                center: Vec3::ZERO,
                heading: 0.6,
            },
            &[Motion::stand(1.5), Motion::squats(4), Motion::stand(1.5)],
        );
        let psi = [-1.4_f32, 2.2, 1.57, -2.7, 0.9, -0.4];
        let mount = |i: usize| {
            Quat::from_rotation_x(0.3 + 0.23 * i as f32)
                * Quat::from_rotation_z(-0.7 + 0.31 * i as f32)
        };
        let id = |i: usize| format!("replay/{i}");
        let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
        core.set_assignments(
            Role::CORE
                .iter()
                .enumerate()
                .map(|(i, role)| RoleAssignment {
                    sensor_id: id(i),
                    role: *role,
                })
                .collect(),
        )
        .unwrap();
        let line = |record: &Record| serde_json::to_string(record).unwrap() + "\n";
        let profile = core.export_profile();
        let mut text = line(&Record::Header {
            version: 1,
            config: core.config().clone(),
            profile: profile.clone(),
            note: String::new(),
        });
        core.restore_profile(profile).unwrap();
        let (neutral_at, restart_at) = (1.2, 2.0);
        let mut finished = false;
        let ticks = (generator.duration() * 100.0) as u64;
        for tick in 1..=ticks {
            let at = tick as f64 / 100.0;
            let truth = generator.frame(at);
            for i in 0..6 {
                let q =
                    (Quat::from_rotation_y(-psi[i]) * truth.rotations[i] * mount(i).conjugate())
                        .normalize();
                let sample = SensorSample {
                    id: id(i),
                    orientation_xyzw: q.to_array(),
                    received_at_secs: at,
                    acceleration_mps2: None,
                    provenance: tracking_core::SensorProvenance::Measured,
                };
                let rejected = core.ingest(sample.clone()).err().map(|e| e.to_string());
                text += &line(&Record::Sample { sample, rejected });
            }
            if tick == 120 {
                core.calibrate_neutral(neutral_at, &Anchors::default(), true)
                    .unwrap();
                text += &line(&Record::Calibrate {
                    at: neutral_at,
                    preview: true,
                });
                core.begin_motion_calibration(at).unwrap();
                text += &line(&Record::MotionBegin { at });
            }
            if tick == 200 {
                core.cancel_motion_calibration();
                text += &line(&Record::MotionCancel { at: restart_at });
                core.begin_motion_calibration(restart_at).unwrap();
                text += &line(&Record::MotionBegin { at: restart_at });
            }
            if tick > 200
                && !finished
                && core
                    .motion_calibration_status()
                    .is_some_and(|status| status.ready)
            {
                core.finish_motion_calibration(at).unwrap();
                text += &line(&Record::MotionFinish { at });
                finished = true;
            }
            let frame = core.solve(at, &Anchors::default());
            text += &line(&Record::Frame { frame });
        }
        assert!(finished && core.direction_calibrated());
        let summary = replay_reader(text.as_bytes()).unwrap();
        assert_eq!(summary.outcome_mismatches, 0);
        assert_eq!(summary.max_difference, 0.0);
        assert_eq!(summary.frames, ticks);
        // Desktop preview never yields valid targets, before or after calibration.
        assert_eq!(summary.valid, 0);
    }

    /// Auto mode end to end on synthetic data, recorded and replayed: a
    /// desktop calibration, the headset appearing (alignment), a recenter
    /// reported with a matching transform, SteamVR leaving for 3 s and
    /// returning, and an auto-height neutral capture. Replay must reproduce
    /// every frame exactly.
    #[test]
    fn auto_mode_session_with_steamvr_events_replays_deterministically() {
        use crate::output_mode::{
            HeadsetPresence, OutputMode, OutputSink, PublishState, desktop_session, output_sink,
            wants_alignment,
        };
        use glam::{Quat, Vec3};
        use tracking_core::forward_calibration::ski_pitch_sign;
        let index = |role: Role| Role::ALL.iter().position(|r| *r == role).unwrap();
        let mount = |role: Role| {
            let i = index(role) as f32;
            Quat::from_rotation_x(0.3 + 0.23 * i) * Quat::from_rotation_z(-0.7 + 0.31 * i)
        };
        let bias = |role: Role| [-1.4_f32, 2.2, 1.57, -2.7, 0.9, -0.4][index(role)];
        let pitch = |role: Role| {
            ski_pitch_sign(role).unwrap_or(0.0)
                * [28.0_f32, 22.0, 38.0, 34.0, 18.0, 20.0][index(role)].to_radians()
        };
        let id = |role: Role| format!("auto/{role:?}");
        let mut core = TrackingCore::new(TrackingConfig {
            estimator: tracking_core::EstimatorConfig {
                enable_auto_rebase: true,
                enable_stance_heading: true,
                ..Default::default()
            },
            ..Default::default()
        })
        .unwrap();
        core.set_assignments(
            Role::CORE
                .iter()
                .map(|role| RoleAssignment {
                    sensor_id: id(*role),
                    role: *role,
                })
                .collect(),
        )
        .unwrap();
        let line = |record: &Record| serde_json::to_string(record).unwrap() + "\n";
        let mut text = line(&Record::Header {
            version: 1,
            config: core.config().clone(),
            profile: core.export_profile(),
            note: String::new(),
        });
        let eye = core.config().anthropometrics.standing_eye_height_m;
        let space = Quat::from_rotation_y(70.0_f32.to_radians());
        let mut presence = HeadsetPresence::default();
        let mode = OutputMode::Auto;
        let mut sinks = Vec::new();
        let mut last = None;
        for tick in 1..=1100u32 {
            let at = f64::from(tick) / 100.0;
            // Body heading: 0.5 at the desk; -1.0 once the headset is worn.
            let (heading, bend) = match tick {
                0..=100 => (0.5, false),
                101..=200 => (0.5, true),
                _ => (-1.0, false),
            };
            for role in Role::CORE {
                let body = Quat::from_rotation_y(heading)
                    * Quat::from_rotation_x(if bend { pitch(role) } else { 0.0 });
                let sample = SensorSample {
                    id: id(role),
                    orientation_xyzw: (Quat::from_rotation_y(bias(role)) * body * mount(role))
                        .normalize()
                        .to_array(),
                    received_at_secs: at,
                    acceleration_mps2: None,
                    provenance: tracking_core::SensorProvenance::Measured,
                };
                let rejected = core.ingest(sample.clone()).err().map(|e| e.to_string());
                text += &line(&Record::Sample { sample, rejected });
            }
            // Headset: absent at the desk, worn from tick 201, recentred by
            // 70° at tick 501 (SteamVR reports the transform change), gone
            // during 701-1000 (SteamVR quit), back from tick 1001.
            let worn = (201..=700).contains(&tick) || tick > 1000;
            let rotated = tick > 500;
            if tick == 501 {
                core.note_tracking_space_event(at, Some(70.0_f32.to_radians()));
                text += &line(&Record::TrackingSpaceEvent {
                    at,
                    transform_yaw_radians: Some(70.0_f32.to_radians()),
                });
            }
            if tick == 1001 {
                // New helper session: an event without a transform yaw.
                core.note_tracking_space_event(at, None);
                text += &line(&Record::TrackingSpaceEvent {
                    at,
                    transform_yaw_radians: None,
                });
            }
            let anchors = if worn {
                let r = if rotated { space } else { Quat::IDENTITY };
                Anchors {
                    hmd: Some(AnchorPose {
                        position: (r * Vec3::new(0.05, eye + 0.03, 0.0)).to_array(),
                        orientation_xyzw: (r * Quat::from_rotation_y(heading)).to_array(),
                        received_at_secs: at,
                        valid: true,
                        linear_velocity: None,
                        angular_velocity: None,
                    }),
                    ..Default::default()
                }
            } else {
                Anchors::default()
            };
            if tick == 1 || worn || tick == 701 {
                text += &line(&Record::Anchors {
                    at,
                    anchors: anchors.clone(),
                });
            }
            presence.observe(at, anchors.hmd.as_ref().map(|head| head.position[1]), eye);
            // Calibrations chosen the way the worker chooses them.
            if tick == 100 {
                let preview = desktop_session(mode, presence.present());
                assert!(preview, "no headset: a desktop calibration");
                core.calibrate_neutral(at, &anchors, preview).unwrap();
                text += &line(&Record::Calibrate { at, preview });
            }
            if tick == 200 {
                core.calibrate_forward(at).unwrap();
                text += &line(&Record::ForwardCalibrate { at });
            }
            let frame = core.solve(at, &anchors);
            let mut state = PublishState {
                calibrating: false,
                calibrated: frame.calibration.calibrated,
                direction_calibrated: core.direction_calibrated(),
                preview_calibration: frame.calibration.preview_only,
                frame_live: !frame.preview_only && frame.live_anchor,
                suspended: false,
            };
            text += &line(&Record::Frame {
                frame: frame.clone(),
            });
            if tick % 10 == 0
                && wants_alignment(mode, presence.present(), &state)
                && core.align_preview_to_headset(at, &anchors).is_ok()
            {
                text += &line(&Record::AlignToHeadset { at });
                state.preview_calibration = false;
            }
            sinks.push((tick, output_sink(mode, &state), frame.preview_only));
            last = Some(frame);
        }
        let last = last.unwrap();
        let sink_at = |tick: u32| sinks[tick as usize - 1].1;
        // Desk: desktop calibration, nothing sent.
        assert_eq!(sink_at(150), OutputSink::Off);
        assert_eq!(sink_at(250), OutputSink::Off, "aligning");
        // Aligned automatically, then live in SteamVR.
        assert_eq!(sink_at(400), OutputSink::SteamVr);
        // The recenter was carried over by the transform's exact yaw.
        assert_eq!(sink_at(600), OutputSink::SteamVr);
        // SteamVR gone: nothing sent, calibration kept, desktop preview.
        assert_eq!(sink_at(900), OutputSink::Off);
        assert!(sinks[899].2);
        // Back: live again without any user action.
        assert_eq!(sink_at(1100), OutputSink::SteamVr);
        assert!(core.direction_calibrated() && !last.calibration.preview_only);
        assert_eq!(last.estimator.standing_space_rebases, 1);
        assert_eq!(
            last.estimator.last_rebase_evidence,
            Some(tracking_core::tracking_space::RebaseEvidence::Transform)
        );
        assert_eq!(last.estimator.headset_resumptions, 1);
        // Auto-height capture in the same recording: a configuration record,
        // then the live neutral capture, exactly as the worker writes them.
        let measured = core.stable_headset_height(11.0).unwrap();
        let height = auto_eye_height(eye, Some(measured), 0.20).unwrap();
        let config = config_with_standing_height(core.config(), height).unwrap();
        let anchors_now = Anchors {
            hmd: Some(AnchorPose {
                position: (space * Vec3::new(0.05, eye + 0.03, 0.0)).to_array(),
                orientation_xyzw: (space * Quat::from_rotation_y(-1.0)).to_array(),
                received_at_secs: 11.0,
                valid: true,
                linear_velocity: None,
                angular_velocity: None,
            }),
            ..Default::default()
        };
        core.calibrate_neutral_with_config(11.0, &anchors_now, false, config)
            .unwrap();
        text += &line(&Record::Configuration {
            at: 11.0,
            config: core.config().clone(),
            profile: core.export_profile(),
        });
        text += &line(&Record::Calibrate {
            at: 11.0,
            preview: false,
        });
        assert!(
            core.direction_calibrated(),
            "mounting kept by the neutral capture"
        );
        let frame = core.solve(11.005, &anchors_now);
        assert!(!frame.preview_only);
        text += &line(&Record::Frame { frame });
        let summary = replay_reader(text.as_bytes()).unwrap();
        assert_eq!(summary.outcome_mismatches, 0);
        assert_eq!(summary.frames, 1101);
        assert_eq!(summary.max_difference, 0.0);
        assert!(summary.valid > 500, "{}", summary.valid);
        // The offline dump applies the same events without failures.
        let dumped = crate::replay::dump_reader(
            text.as_bytes(),
            &mut Vec::new(),
            &crate::replay::DumpOptions {
                every: 100,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(dumped.failed_events, 0);
    }

    #[test]
    fn motion_blocker_summary_names_each_blocking_segment_in_plain_words() {
        let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
        let status = {
            core.set_assignments(
                Role::CORE
                    .iter()
                    .map(|role| RoleAssignment {
                        sensor_id: format!("{role:?}"),
                        role: *role,
                    })
                    .collect(),
            )
            .unwrap();
            assert!(core.motion_calibration_status().is_none());
            tracking_core::motion_calibration::MotionCalibrator::new(Default::default())
                .unwrap()
                .status()
        };
        let summary = motion_blocker_summary(&status);
        for label in [
            "Chest:",
            "Hip:",
            "Left thigh:",
            "Right thigh:",
            "Left calf:",
            "Right calf:",
        ] {
            assert!(summary.contains(label), "{summary}");
        }
        assert!(!summary.contains("LeftThigh:"), "{summary}");
    }
}

#[cfg(test)]
mod tracking_space_tests {
    use super::*;
    #[test]
    fn headset_facing_uses_current_orientation_and_rejects_vertical_or_stale_gaze() {
        let mut anchors = Anchors {
            hmd: Some(AnchorPose {
                position: [0., 1.6, 0.],
                orientation_xyzw: glam::Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
                    .to_array(),
                received_at_secs: 10.,
                valid: true,
                linear_velocity: None,
                angular_velocity: None,
            }),
            ..Default::default()
        };
        let forward =
            glam::Vec3::from_array(current_headset_forward(&anchors, 10.1, 0.25).unwrap());
        assert!(forward.distance(-glam::Vec3::X) < 1e-5);
        assert!(current_headset_forward(&anchors, 10.3, 0.25).is_none());
        anchors.hmd.as_mut().unwrap().orientation_xyzw =
            glam::Quat::from_rotation_x(std::f32::consts::FRAC_PI_2).to_array();
        assert!(current_headset_forward(&anchors, 10.1, 0.25).is_none());
        anchors.hmd.as_mut().unwrap().orientation_xyzw = [f32::NAN, 0., 0., 1.];
        assert!(current_headset_forward(&anchors, 10.1, 0.25).is_none());
    }
    #[test]
    fn headset_height_capture_requires_fresh_valid_realistic_anchor() {
        let mut anchors = Anchors {
            hmd: Some(AnchorPose {
                position: [0., 1.57, 0.],
                orientation_xyzw: [0., 0., 0., 1.],
                received_at_secs: 10.,
                valid: true,
                linear_velocity: None,
                angular_velocity: None,
            }),
            ..Default::default()
        };
        assert_eq!(
            current_standing_headset_height(&anchors, 10.1, 0.25).unwrap(),
            1.57
        );
        assert!(current_standing_headset_height(&anchors, 10.3, 0.25).is_err());
        assert!(current_standing_headset_height(&anchors, 9.9, 0.25).is_err());
        anchors.hmd.as_mut().unwrap().valid = false;
        assert!(current_standing_headset_height(&anchors, 10.1, 0.25).is_err());
        anchors.hmd.as_mut().unwrap().valid = true;
        for height in [0.7, 2.6, f32::NAN] {
            anchors.hmd.as_mut().unwrap().position[1] = height;
            assert!(current_standing_headset_height(&anchors, 10.1, 0.25).is_err());
        }
        anchors.hmd.as_mut().unwrap().position[1] = 1.57;
        anchors.hmd.as_mut().unwrap().orientation_xyzw = [0., 0., 0., 0.];
        assert!(current_standing_headset_height(&anchors, 10.1, 0.25).is_err());
    }
    #[test]
    fn explicit_height_configuration_preserves_preferences_and_changes_only_the_clone() {
        let config = TrackingConfig {
            enable_joint_yaw_correction: true,
            ..Default::default()
        };
        let before = config.anthropometrics.standing_eye_height_m;
        let updated = config_with_standing_height(&config, 1.57).unwrap();
        assert_eq!(config.anthropometrics.standing_eye_height_m, before);
        assert_eq!(updated.anthropometrics.standing_eye_height_m, 1.57);
        let ratio = 1.57 / before;
        assert!(
            (updated.anthropometrics.eye_forward_m - config.anthropometrics.eye_forward_m * ratio)
                .abs()
                < 1e-6
        );
        assert!(updated.enable_joint_yaw_correction);
        assert!(config_with_standing_height(&config, f32::NAN).is_err());
        assert!(config_with_standing_height(&config, 0.79).is_err());
    }
    #[test]
    fn profile_dimensions_are_made_consistent_on_load_and_on_eye_height() {
        // A profile saved before the defaults changed: chain 2 cm taller than the eyes.
        let mut body = tracking_core::Anthropometrics::default();
        body.chest_to_hip_m += 0.02;
        let mut loaded = body.clone();
        let note = normalize_profile_dimensions(&mut loaded).unwrap();
        assert!(note.contains("adjusted"), "{note}");
        assert!((loaded.standing_chain_height_m() - loaded.standing_eye_height_m).abs() < 1e-5);
        assert_eq!(normalize_profile_dimensions(&mut loaded), None);
        let config = TrackingConfig {
            anthropometrics: body,
            ..Default::default()
        };
        let updated = config_with_standing_height(&config, 1.57).unwrap();
        let a = &updated.anthropometrics;
        assert!((a.standing_chain_height_m() - 1.57).abs() < 1e-5);
        // Impossible to absorb: kept, with a message.
        let mut impossible = tracking_core::Anthropometrics::default();
        impossible.standing_eye_height_m += 0.45;
        let kept = impossible.clone();
        let note = normalize_profile_dimensions(&mut impossible).unwrap();
        assert!(note.contains("unchanged"), "{note}");
        assert_eq!(impossible, kept);
    }
    #[test]
    fn app_targets_serialize_to_native_receiver_fixture() {
        use tracking_core::{Evidence, TargetPose};
        let target =
            |role, position, orientation_xyzw, linear_velocity, angular_velocity| TargetPose {
                role,
                position,
                orientation_xyzw,
                position_source: Evidence::Inferred,
                orientation_source: Evidence::MeasuredOrientation,
                confidence: 1.0,
                valid: true,
                reason: None,
                linear_velocity,
                angular_velocity,
            };
        let q = std::f32::consts::FRAC_1_SQRT_2;
        let targets = [
            target(
                TargetRole::Waist,
                [1., 2., 3.],
                [q, 0., 0., q],
                [0.5, -1., 2.],
                [0., 0., 3.],
            ),
            target(
                TargetRole::LeftFoot,
                [-0.25, 1.2, -0.5],
                [0., 0., 0., 1.],
                [0., 0., -0.25],
                [1., 0., 0.],
            ),
        ];
        // Target velocities pass through to the app's slot order and WXYZ
        // mapping, checked against the v2 receiver fixture.
        let poses = steamvr_poses(&targets, true);
        let frame = steamvr_bridge::Trackers {
            session: 0x1122334455667788,
            sequence: 123,
            raw_from_standing: [0., 0., 1., 2., 0., 1., 0., 3., -1., 0., 0., 4.],
            pose_age_secs: 0.015625,
            apply_angular_velocity: true,
            poses,
        };
        let fixture = include_str!("../../../driver/tests/fixtures/target-v2.hex").trim();
        let bytes: Vec<u8> = fixture
            .as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect();
        assert_eq!(frame.encode(), bytes);
        assert!(
            steamvr_poses(&targets, false)
                .iter()
                .all(|pose| !pose.valid)
        );
        for (index, role) in [
            TargetRole::Waist,
            TargetRole::Chest,
            TargetRole::LeftFoot,
            TargetRole::RightFoot,
            TargetRole::LeftKnee,
            TargetRole::RightKnee,
            TargetRole::LeftElbow,
            TargetRole::RightElbow,
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(target_index(role), index);
        }
    }
    #[test]
    fn anchor_transitions_reject_repeated_frames_and_invalidate_on_recenter_or_session() {
        let a = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.];
        let mut frame = steamvr_bridge::Anchors {
            session: 1,
            sequence: 1,
            raw_from_standing: a,
            poses: [steamvr_bridge::Pose::default(); 3],
        };
        assert_eq!(
            anchor_transition(None, None, &frame),
            AnchorTransition::Updated
        );
        assert_eq!(
            anchor_transition(Some((1, 1)), Some(&a), &frame),
            AnchorTransition::Duplicate
        );
        frame.sequence = 2;
        frame.raw_from_standing[3] = 0.005;
        assert_eq!(
            anchor_transition(Some((1, 1)), Some(&a), &frame),
            AnchorTransition::Updated
        );
        frame.sequence = 3;
        frame.raw_from_standing[3] = 0.02;
        assert_eq!(
            anchor_transition(Some((1, 2)), Some(&a), &frame),
            AnchorTransition::SpaceChanged
        );
        frame.session = 2;
        assert_eq!(
            anchor_transition(Some((1, 3)), Some(&a), &frame),
            AnchorTransition::SessionChanged {
                space_changed: true
            }
        );
        frame.raw_from_standing = a;
        assert_eq!(
            anchor_transition(Some((1, 3)), Some(&a), &frame),
            AnchorTransition::SessionChanged {
                space_changed: false
            }
        );
    }

    /// Row-major raw-from-standing with rotation `r` and translation `t`.
    fn transform(r: glam::Quat, t: glam::Vec3) -> [f32; 12] {
        let m = glam::Mat3::from_quat(r);
        [
            m.x_axis.x, m.y_axis.x, m.z_axis.x, t.x, //
            m.x_axis.y, m.y_axis.y, m.z_axis.y, t.y, //
            m.x_axis.z, m.y_axis.z, m.z_axis.z, t.z,
        ]
    }
    fn standing_from_raw(m: &[f32; 12], raw: glam::Vec3) -> glam::Vec3 {
        let r = glam::Mat3::from_cols(
            glam::Vec3::new(m[0], m[4], m[8]),
            glam::Vec3::new(m[1], m[5], m[9]),
            glam::Vec3::new(m[2], m[6], m[10]),
        );
        r.transpose() * (raw - glam::Vec3::new(m[3], m[7], m[11]))
    }

    #[test]
    fn standing_space_yaw_matches_the_core_rebase_sign_convention() {
        let old = transform(
            glam::Quat::from_rotation_y(0.7) * glam::Quat::from_rotation_x(0.02),
            glam::Vec3::new(1.5, -0.2, 3.0),
        );
        let old_rotation = glam::Quat::from_rotation_y(0.7) * glam::Quat::from_rotation_x(0.02);
        for degrees in [30.0_f32, -30.0, 170.0, 0.0] {
            let turn = glam::Quat::from_rotation_y(degrees.to_radians());
            // The standing space was rotated about +Y by `degrees`:
            // p_new = turn * p_old, so raw_from_standing_new = old * turn^-1.
            let new = transform(
                old_rotation * turn.inverse(),
                glam::Vec3::new(1.1, 0.0, 2.4),
            );
            let yaw = standing_rebase_yaw(&old, &new).unwrap();
            assert!(
                (yaw.to_degrees() - degrees).abs() < 1e-3,
                "{degrees}: {}",
                yaw.to_degrees()
            );
            // Contract shared with TrackingCore::rebase_standing_space (whose
            // test checks outputs after rebase(yaw) equal the old ones rotated
            // by R_y(yaw) about the origin): directions of a fixed raw point
            // pair map as p_new - q_new = R_y(yaw) (p_old - q_old).
            let (p, q) = (
                glam::Vec3::new(0.3, 1.6, -0.4),
                glam::Vec3::new(-1.0, 0.2, 0.9),
            );
            let old_delta = standing_from_raw(&old, p) - standing_from_raw(&old, q);
            let new_delta = standing_from_raw(&new, p) - standing_from_raw(&new, q);
            assert!(
                (new_delta - glam::Quat::from_rotation_y(yaw) * old_delta).length() < 1e-4,
                "{degrees}"
            );
        }
        // A tilt beyond 1° is not a recenter; malformed transforms are rejected.
        let tilted = transform(
            old_rotation * glam::Quat::from_rotation_x(1.5_f32.to_radians()),
            glam::Vec3::ZERO,
        );
        assert_eq!(standing_rebase_yaw(&old, &tilted), None);
        let slight = transform(
            old_rotation * glam::Quat::from_rotation_x(0.5_f32.to_radians()),
            glam::Vec3::ZERO,
        );
        assert!(standing_rebase_yaw(&old, &slight).unwrap().abs() < 1e-2);
        let mut broken = old;
        broken[0] = f32::NAN;
        assert_eq!(standing_rebase_yaw(&old, &broken), None);
        assert_eq!(standing_rebase_yaw(&broken, &old), None);
    }

    #[test]
    fn steamvr_side_changes_are_reported_as_events_never_as_clears() {
        let a = transform(glam::Quat::IDENTITY, glam::Vec3::ZERO);
        let turned = transform(
            glam::Quat::from_rotation_y(-0.5),
            glam::Vec3::new(0.3, 0.0, 0.0),
        );
        let tilted = transform(glam::Quat::from_rotation_x(0.2), glam::Vec3::ZERO);
        // A yaw-only transform change: an event carrying the transform's yaw
        // as corroboration only.
        let Some(Some(yaw)) =
            tracking_space_event(&AnchorTransition::SpaceChanged, Some(&a), &turned)
        else {
            panic!("a yaw-only change is an event with a yaw");
        };
        assert!((yaw - 0.5).abs() < 1e-5);
        // Tilted or without a reference: still only an event, no yaw.
        assert_eq!(
            tracking_space_event(&AnchorTransition::SpaceChanged, Some(&a), &tilted),
            Some(None)
        );
        assert_eq!(
            tracking_space_event(&AnchorTransition::SpaceChanged, None, &turned),
            Some(None)
        );
        // A new helper session alone (SteamVR restarted): an event without a
        // transform yaw; the core keeps the calibration if the headset pose is
        // continuous, and after a long gap keeps it anyway.
        let restart = |space_changed| AnchorTransition::SessionChanged { space_changed };
        assert_eq!(
            tracking_space_event(&restart(false), Some(&a), &a),
            Some(None)
        );
        let Some(Some(yaw)) = tracking_space_event(&restart(true), Some(&a), &turned) else {
            panic!("a session change with a yaw-only transform change");
        };
        assert!((yaw - 0.5).abs() < 1e-5);
        for transition in [AnchorTransition::Updated, AnchorTransition::Duplicate] {
            assert_eq!(tracking_space_event(&transition, Some(&a), &turned), None);
        }
    }

    #[test]
    fn eye_height_is_updated_from_the_headset_only_within_the_valid_range() {
        let tolerance = 0.20;
        // Within 1.5 cm: unchanged.
        assert_eq!(auto_eye_height(1.65, Some(1.66), tolerance), None);
        assert_eq!(auto_eye_height(1.65, Some(1.635), tolerance), None);
        // A real difference within the live gate's tolerance: adopted.
        assert_eq!(auto_eye_height(1.65, Some(1.67), tolerance), Some(1.67));
        assert_eq!(auto_eye_height(1.65, Some(1.52), tolerance), Some(1.52));
        // Outside the tolerance (leaning on the desk, wrong floor): never.
        assert_eq!(auto_eye_height(1.65, Some(1.40), tolerance), None);
        assert_eq!(auto_eye_height(1.65, Some(1.90), tolerance), None);
        // Outside 0.8-2.5 m, missing or nonfinite: never.
        assert_eq!(auto_eye_height(0.9, Some(0.78), tolerance), None);
        assert_eq!(auto_eye_height(1.65, None, tolerance), None);
        assert_eq!(auto_eye_height(1.65, Some(f32::NAN), tolerance), None);
    }

    #[test]
    fn routine_steamvr_exits_are_not_reported_as_helper_failures() {
        for stderr in [
            "SteamVR is not running; start SteamVR before the anchor helper.",
            "OpenVR initialization failed: 108",
            "OpenVR system unavailable: 101",
            "",
        ] {
            assert_eq!(
                helper_exit(false, "exit code: 1", stderr),
                HelperExit::Routine,
                "{stderr}"
            );
        }
        assert_eq!(helper_exit(true, "exit code: 0", ""), HelperExit::Routine);
        assert!(matches!(
            helper_exit(false, "exit code: 1", "openvr_api.dll not found next to helper"),
            HelperExit::Failed(text) if text.contains("openvr_api.dll")
        ));
    }

    #[test]
    fn estimator_settings_apply_validate_and_persist_with_defaults() {
        let settings = EstimatorSettings::default();
        assert!(settings.prediction && settings.hmd_extrapolation && settings.auto_rebase);
        assert!(settings.stance_heading && settings.leg_ik);
        assert!(!settings.angular_velocity_output);
        let config = config_with_estimator(&TrackingConfig::default(), &settings).unwrap();
        assert!(config.estimator.enable_prediction);
        assert!(config.estimator.enable_hmd_extrapolation);
        assert!(config.estimator.enable_auto_rebase);
        assert_eq!(config.estimator.prediction_lead_secs, 0.0);
        let led = EstimatorSettings {
            prediction_lead_ms: 12.0,
            ..settings
        };
        let config = config_with_estimator(&TrackingConfig::default(), &led).unwrap();
        assert!((config.estimator.prediction_lead_secs - 0.012).abs() < 1e-7);
        for lead in [-1.0, 31.0, f32::NAN] {
            let bad = EstimatorSettings {
                prediction_lead_ms: lead,
                ..settings
            };
            assert!(config_with_estimator(&TrackingConfig::default(), &bad).is_err());
        }
        // Settings files from before this option load with the app defaults.
        let old = serde_json::json!({
            "profile": TrackingCore::new(TrackingConfig::default()).unwrap().export_profile(),
            "correction": true,
        });
        let loaded: Settings = serde_json::from_value(old).unwrap();
        assert_eq!(loaded.estimator, EstimatorSettings::default());
        assert_eq!(loaded.calibration_method, CalibrationMethod::Movement);
        let ski = serde_json::json!({
            "profile": TrackingCore::new(TrackingConfig::default()).unwrap().export_profile(),
            "calibration_method": "ski_pose",
        });
        let loaded: Settings = serde_json::from_value(ski).unwrap();
        assert_eq!(loaded.calibration_method, CalibrationMethod::SkiPose);
        let partial: EstimatorSettings =
            serde_json::from_str(r#"{"prediction":false,"angular_velocity_output":true}"#).unwrap();
        assert!(!partial.prediction && partial.angular_velocity_output && partial.auto_rebase);
    }

    #[test]
    fn replay_applies_recorded_rebases() {
        let line = |record: Record| serde_json::to_string(&record).unwrap() + "\n";
        let text = line(Record::Rebase {
            at: 1.0,
            yaw_radians: 0.4,
        });
        assert!(text.contains("\"event\":\"rebase\""));
        replay_reader(text.as_bytes()).unwrap();
    }
    #[test]
    fn standing_space_detects_material_changes() {
        let a = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.];
        let mut b = a;
        b[3] = 0.005;
        assert!(!standing_space_changed(&a, &b));
        b[3] = 0.02;
        assert!(standing_space_changed(&a, &b));
        b = a;
        b[2] = 0.02;
        assert!(standing_space_changed(&a, &b));
    }
}
