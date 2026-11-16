//! Offline, streaming shadow-heading analysis. Never constructs a runtime,
//! network output, independent witness, or production correction.
use crate::runtime::Record;
use glam::DQuat;
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{self, BufRead, BufReader, Read, Write},
    path::Path,
};
use tracking_core::{
    RecoveryState, Role, SegmentPose, SensorProvenance, TrackingFrame,
    heading_shadow::{
        HeadingPairSample, HeadingSegmentSample, HeadingShadowConfig, HeadingShadowReport,
        HeadingShadowStatus, RelativeHeadingEstimator,
    },
};

const MAX_LINE_BYTES: usize = 2 * 1024 * 1024;
const MAX_RECORDS: u64 = 10_000_000;
const MAX_REPORT_BYTES: usize = 256 * 1024;

#[derive(Serialize)]
pub struct AnalysisReport {
    format_version: u32,
    assessment: &'static str,
    model_only: bool,
    accuracy_proof: bool,
    network_output_sent: bool,
    independent_witnesses_supplied: u64,
    note: &'static str,
    configuration: HeadingShadowConfig,
    records_read: u64,
    frame_records: u64,
    preview_calibrated_frames: u64,
    live_hmd_calibrated_frames: u64,
    unanchored_calibrated_frames: u64,
    calibrated_with_firmware_hold_frames: u64,
    eligible_pairs: u64,
    preview_eligible_pairs: u64,
    live_hmd_eligible_pairs: u64,
    unanchored_eligible_pairs: u64,
    reset_reasons: BTreeMap<String, u64>,
    left_leg: LegReport,
    right_leg: LegReport,
}

#[derive(Serialize)]
struct LegReport {
    eligible_pairs: u64,
    preview_eligible_pairs: u64,
    live_hmd_eligible_pairs: u64,
    unanchored_eligible_pairs: u64,
    model_conditional_candidate_reports: u64,
    max_abs_proposed_relative_angle_degrees: Option<f64>,
    excluded_pair_reasons: BTreeMap<String, u64>,
    estimator_status_counts: BTreeMap<String, u64>,
    source_window_resets: u64,
    last_exclusion_reason: Option<String>,
    last_report: HeadingShadowReport,
}

struct Leg {
    parent: Role,
    child: Role,
    estimator: RelativeHeadingEstimator,
    report: LegReport,
    calibration_epoch: u64,
    continuity_epoch: u64,
    last_parent_at: Option<f64>,
    last_child_at: Option<f64>,
    invalid_reason: Option<&'static str>,
}

impl Leg {
    fn new(parent: Role, child: Role, config: HeadingShadowConfig) -> io::Result<Self> {
        let estimator =
            RelativeHeadingEstimator::new(parent, child, config).map_err(io::Error::other)?;
        let report = LegReport {
            eligible_pairs: 0,
            preview_eligible_pairs: 0,
            live_hmd_eligible_pairs: 0,
            unanchored_eligible_pairs: 0,
            model_conditional_candidate_reports: 0,
            max_abs_proposed_relative_angle_degrees: None,
            excluded_pair_reasons: BTreeMap::new(),
            estimator_status_counts: BTreeMap::new(),
            source_window_resets: 0,
            last_exclusion_reason: None,
            last_report: estimator.report(0.),
        };
        Ok(Self {
            parent,
            child,
            estimator,
            report,
            calibration_epoch: 1,
            continuity_epoch: 1,
            last_parent_at: None,
            last_child_at: None,
            invalid_reason: None,
        })
    }

    fn reset(&mut self, calibration: bool, clear_watermarks: bool, at: f64) {
        if calibration {
            self.calibration_epoch += 1;
        } else {
            self.continuity_epoch += 1;
        }
        self.estimator.clear();
        self.report.last_report = self.estimator.report(at);
        self.report.source_window_resets += 1;
        self.invalid_reason = None;
        if clear_watermarks {
            self.last_parent_at = None;
            self.last_child_at = None;
        }
    }

    fn exclude(&mut self, reason: &'static str, now: f64, invalidates_window: bool) {
        *self
            .report
            .excluded_pair_reasons
            .entry(reason.to_owned())
            .or_default() += 1;
        self.report.last_exclusion_reason = Some(reason.to_owned());
        if invalidates_window && self.invalid_reason != Some(reason) {
            self.reset(false, false, now);
            self.invalid_reason = Some(reason);
        }
        self.report.last_report = self.estimator.report(now);
    }

    fn frame(&mut self, frame: &TrackingFrame) {
        let parent: Vec<_> = frame
            .segments
            .iter()
            .filter(|segment| segment.role == self.parent)
            .collect();
        let child: Vec<_> = frame
            .segments
            .iter()
            .filter(|segment| segment.role == self.child)
            .collect();
        if parent.len() != 1 || child.len() != 1 {
            self.exclude("missing_or_duplicate_segments", frame.at_secs, true);
            return;
        }
        let (parent, child) = (parent[0], child[0]);
        if parent.uses_firmware_hold || child.uses_firmware_hold {
            self.exclude("firmware_held_orientation", frame.at_secs, true);
            return;
        }
        if !parent.valid || !child.valid {
            self.exclude("invalid_segment", frame.at_secs, true);
            return;
        }
        let Some(parent_at) = parent.measurement_received_at_secs else {
            self.exclude(
                "missing_original_measurement_timestamp",
                frame.at_secs,
                true,
            );
            return;
        };
        let Some(child_at) = child.measurement_received_at_secs else {
            self.exclude(
                "missing_original_measurement_timestamp",
                frame.at_secs,
                true,
            );
            return;
        };
        let (Some(parent_age), Some(child_age)) =
            (parent.measurement_age_secs, child.measurement_age_secs)
        else {
            self.exclude(
                "missing_original_measurement_timestamp",
                frame.at_secs,
                true,
            );
            return;
        };
        let config = self.estimator.config();
        if [parent_at, child_at].iter().any(|time| {
            !time.is_finite()
                || *time < 0.
                || *time > frame.at_secs
                || frame.at_secs - *time > config.stale_after_secs
        }) {
            self.exclude(
                "stale_future_or_invalid_measurement_timestamp",
                frame.at_secs,
                true,
            );
            return;
        }
        if [(parent_at, parent_age), (child_at, child_age)]
            .iter()
            .any(|(time, age)| {
                !age.is_finite() || *age < 0. || (*age - (frame.at_secs - *time)).abs() > 1e-5
            })
        {
            self.exclude("inconsistent_original_measurement_age", frame.at_secs, true);
            return;
        }
        if [parent.confidence, child.confidence]
            .iter()
            .any(|confidence| {
                !confidence.is_finite()
                    || f64::from(*confidence) < config.min_input_confidence
                    || *confidence > 1.
            })
        {
            self.exclude(
                "insufficient_or_invalid_input_confidence",
                frame.at_secs,
                true,
            );
            return;
        }
        if (parent_at - child_at).abs() > config.max_pair_skew_secs {
            self.exclude("receipt_timestamp_pair_skew", frame.at_secs, true);
            return;
        }
        if self.last_parent_at.is_some_and(|last| parent_at < last)
            || self.last_child_at.is_some_and(|last| child_at < last)
        {
            self.exclude("out_of_order_original_measurement", frame.at_secs, true);
            return;
        }
        if self.last_parent_at == Some(parent_at) || self.last_child_at == Some(child_at) {
            self.exclude("reused_original_measurement", frame.at_secs, false);
            return;
        }
        let (Some(parent_q), Some(child_q)) = (
            without_production_yaw(parent),
            without_production_yaw(child),
        ) else {
            self.exclude("invalid_quaternion_or_yaw_correction", frame.at_secs, true);
            return;
        };
        self.last_parent_at = Some(parent_at);
        self.last_child_at = Some(child_at);
        self.invalid_reason = None;
        self.report.last_exclusion_reason = None;
        let sample =
            |role, orientation_xyzw, measured_at_secs, confidence: f32| HeadingSegmentSample {
                role,
                orientation_xyzw,
                measured_at_secs,
                confidence: f64::from(confidence),
                valid: true,
                provenance: SensorProvenance::Measured,
            };
        let result = self.estimator.observe(
            frame.at_secs,
            HeadingPairSample {
                calibration_epoch: self.calibration_epoch,
                continuity_epoch: self.continuity_epoch,
                parent: sample(self.parent, parent_q, parent_at, parent.confidence),
                child: sample(self.child, child_q, child_at, child.confidence),
                independent_witness: None,
            },
        );
        self.report.eligible_pairs += 1;
        if frame.preview_only || frame.calibration.preview_only {
            self.report.preview_eligible_pairs += 1;
        } else if frame.live_anchor {
            self.report.live_hmd_eligible_pairs += 1;
        } else {
            self.report.unanchored_eligible_pairs += 1;
        }
        let status = serde_json::to_value(result.status)
            .ok()
            .and_then(|value| value.as_str().map(str::to_owned))
            .unwrap_or_else(|| "unknown".to_owned());
        *self
            .report
            .estimator_status_counts
            .entry(status)
            .or_default() += 1;
        if result.status == HeadingShadowStatus::ModelConditionalCandidate {
            self.report.model_conditional_candidate_reports += 1;
            if let Some(angle) = result.candidate_relative_yaw_radians {
                let angle = angle.to_degrees().abs();
                self.report.max_abs_proposed_relative_angle_degrees = Some(
                    self.report
                        .max_abs_proposed_relative_angle_degrees
                        .map_or(angle, |old| old.max(angle)),
                );
            }
        }
        self.report.last_report = result;
    }
}

fn without_production_yaw(segment: &SegmentPose) -> Option<[f32; 4]> {
    if !segment.yaw_correction_degrees.is_finite()
        || !segment
            .orientation_xyzw
            .iter()
            .all(|value| value.is_finite())
    {
        return None;
    }
    let q = DQuat::from_array(segment.orientation_xyzw.map(f64::from));
    if !(0.5..=1.5).contains(&q.length_squared()) {
        return None;
    }
    // Production applies Ry(correction) on the LEFT, before the mounted sensor.
    let uncorrected =
        DQuat::from_rotation_y(-f64::from(segment.yaw_correction_degrees).to_radians())
            * q.normalize();
    Some(uncorrected.normalize().to_array().map(|value| value as f32))
}

pub fn analyze_recording(input: &Path, output: &Path) -> io::Result<AnalysisReport> {
    let input = File::open(input)?;
    if !input.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "heading analysis requires a regular recording file",
        ));
    }
    let report = analyze(BufReader::new(input))?;
    write_report(output, &report)?;
    Ok(report)
}

fn write_report(output: &Path, report: &AnalysisReport) -> io::Result<()> {
    let mut bytes = serde_json::to_vec_pretty(report).map_err(io::Error::other)?;
    bytes.push(b'\n');
    if bytes.len() > MAX_REPORT_BYTES {
        return Err(io::Error::other(
            "heading report exceeds bounded output size",
        ));
    }
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&bytes)?;
    file.flush()
}

pub fn summary(report: &AnalysisReport) -> String {
    format!(
        "Read {} records / {} frames; {} eligible measured leg pairs. {} Model-only report; no independent witness, accuracy proof or VR output.",
        report.records_read,
        report.frame_records,
        report.eligible_pairs,
        match report.assessment {
            "no_eligible_pairs" => "No eligible evidence was available.",
            "model_conditional_candidates_only" =>
                "Hinge-model candidates were found; real twist remains indistinguishable.",
            _ => "No model-conditional candidate passed the fit gates.",
        }
    )
}

fn analyze(mut reader: impl BufRead) -> io::Result<AnalysisReport> {
    let configuration = HeadingShadowConfig::default();
    let mut left = Leg::new(Role::LeftThigh, Role::LeftCalf, configuration.clone())?;
    let mut right = Leg::new(Role::RightThigh, Role::RightCalf, configuration.clone())?;
    let mut records_read = 0;
    let mut frame_records = 0;
    let mut preview_calibrated_frames = 0;
    let mut live_hmd_calibrated_frames = 0;
    let mut unanchored_calibrated_frames = 0;
    let mut calibrated_with_firmware_hold_frames = 0;
    let mut reset_reasons = BTreeMap::new();
    let mut last_frame_at = None;
    let mut last_frame_valid = None;
    let mut last_recovery_applied = None;
    let mut last_calibration_mode = None;
    let mut buffer = Vec::new();
    loop {
        buffer.clear();
        let read = reader
            .by_ref()
            .take((MAX_LINE_BYTES + 1) as u64)
            .read_until(b'\n', &mut buffer)?;
        if read == 0 {
            break;
        }
        if buffer.len() > MAX_LINE_BYTES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "recording line exceeds 2 MiB limit",
            ));
        }
        if buffer.iter().all(u8::is_ascii_whitespace) {
            continue;
        }
        records_read += 1;
        if records_read > MAX_RECORDS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "recording exceeds record-count limit",
            ));
        }
        let record: Record = serde_json::from_slice(&buffer).map_err(|error| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("record {records_read}: {error}"),
            )
        })?;
        let reset = match record {
            Record::Header { version, .. } => {
                if version != 1 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "unsupported recording version",
                    ));
                }
                last_frame_at = None;
                last_frame_valid = None;
                last_recovery_applied = None;
                last_calibration_mode = None;
                Some(("header", true, true, 0.))
            }
            Record::Calibrate { at, .. } => Some(("calibrate", true, false, at)),
            Record::ForwardCalibrate { at } => Some(("forward_calibrate", true, false, at)),
            Record::MotionFinish { at } => Some(("motion_calibrate", true, false, at)),
            // A movement session alone changes no published calibration.
            Record::MotionBegin { .. } | Record::MotionCancel { .. } => None,
            Record::ManualMount { at, .. }
            | Record::ConfirmManualMount { at, .. }
            | Record::UndoManualMount { at, .. }
            | Record::ResetManualMount { at, .. } => Some(("manual_mount", true, false, at)),
            Record::Clear { at } => Some(("clear", true, false, at)),
            Record::Rebase { at, .. } => Some(("standing_space_rebase", true, false, at)),
            // May be followed by an automatic rebase decided from the headset poses.
            Record::TrackingSpaceEvent { at, .. } => {
                Some(("tracking_space_event", true, false, at))
            }
            Record::AlignToHeadset { at } => Some(("align_to_headset", true, false, at)),
            Record::Configuration { at, .. } => Some(("configuration", true, false, at)),
            Record::Recovery { at } => Some(("recovery_requested", true, false, at)),
            Record::UndoRecovery { at } => Some(("undo_recovery", true, false, at)),
            Record::RevokeHold { at, .. } => Some(("revoke_hold", false, false, at)),
            Record::Frame { frame } => {
                frame_records += 1;
                if frame.calibration.used_firmware_hold {
                    calibrated_with_firmware_hold_frames += 1;
                }
                if !frame.at_secs.is_finite()
                    || frame.at_secs < 0.
                    || last_frame_at.is_some_and(|last| frame.at_secs <= last)
                {
                    left.exclude("invalid_or_nonmonotonic_frame_time", frame.at_secs, true);
                    right.exclude("invalid_or_nonmonotonic_frame_time", frame.at_secs, true);
                    last_frame_valid = Some(false);
                    Some((
                        "invalid_frame_epoch",
                        false,
                        false,
                        last_frame_at.unwrap_or(0.),
                    ))
                } else {
                    last_frame_at = Some(frame.at_secs);
                    let invalid = if !frame.calibration.calibrated
                        || frame.calibration.requires_session_calibration
                    {
                        Some("uncalibrated_frame")
                    } else if !frame.calibration.direction_calibrated {
                        Some("direction_not_calibrated")
                    } else {
                        None
                    };
                    if let Some(reason) = invalid {
                        left.exclude(reason, frame.at_secs, true);
                        right.exclude(reason, frame.at_secs, true);
                        let transition = last_frame_valid != Some(false);
                        last_frame_valid = Some(false);
                        transition.then_some((
                            "invalid_calibration_epoch",
                            true,
                            false,
                            frame.at_secs,
                        ))
                    } else {
                        let mode = if frame.preview_only || frame.calibration.preview_only {
                            preview_calibrated_frames += 1;
                            "preview"
                        } else if frame.live_anchor {
                            live_hmd_calibrated_frames += 1;
                            "live_hmd"
                        } else {
                            unanchored_calibrated_frames += 1;
                            "unanchored"
                        };
                        if last_calibration_mode.is_some_and(|previous| previous != mode) {
                            left.reset(true, false, frame.at_secs);
                            right.reset(true, false, frame.at_secs);
                            *reset_reasons
                                .entry("calibration_mode_changed".to_owned())
                                .or_default() += 1;
                        }
                        last_calibration_mode = Some(mode);
                        if last_frame_valid == Some(false) {
                            left.reset(true, false, frame.at_secs);
                            right.reset(true, false, frame.at_secs);
                            *reset_reasons
                                .entry("valid_calibration_epoch_started".to_owned())
                                .or_default() += 1;
                        }
                        last_frame_valid = Some(true);
                        if let RecoveryState::Applied { at_secs } = frame.recovery
                            && last_recovery_applied != Some(at_secs)
                        {
                            last_recovery_applied = Some(at_secs);
                            left.reset(true, false, frame.at_secs);
                            right.reset(true, false, frame.at_secs);
                            *reset_reasons
                                .entry("recovery_applied".to_owned())
                                .or_default() += 1;
                        }
                        left.frame(&frame);
                        right.frame(&frame);
                        None
                    }
                }
            }
            Record::Sample { .. } | Record::Anchors { .. } | Record::Transport { .. } => None,
        };
        if let Some((reason, calibration, clear_watermarks, at)) = reset {
            left.reset(calibration, clear_watermarks, at);
            right.reset(calibration, clear_watermarks, at);
            *reset_reasons.entry(reason.to_owned()).or_default() += 1;
        }
    }
    let eligible_pairs = left.report.eligible_pairs + right.report.eligible_pairs;
    let preview_eligible_pairs =
        left.report.preview_eligible_pairs + right.report.preview_eligible_pairs;
    let live_hmd_eligible_pairs =
        left.report.live_hmd_eligible_pairs + right.report.live_hmd_eligible_pairs;
    let unanchored_eligible_pairs =
        left.report.unanchored_eligible_pairs + right.report.unanchored_eligible_pairs;
    let candidates = left.report.model_conditional_candidate_reports
        + right.report.model_conditional_candidate_reports;
    Ok(AnalysisReport {
        format_version: 1,
        assessment: if eligible_pairs == 0 {
            "no_eligible_pairs"
        } else if candidates > 0 {
            "model_conditional_candidates_only"
        } else {
            "eligible_pairs_without_candidate"
        },
        model_only: true,
        accuracy_proof: false,
        network_output_sent: false,
        independent_witnesses_supplied: 0,
        note: "Receipt timestamps do not prove sensor synchronization. Hinge fits cannot distinguish real consistent twist from drift or observe common yaw. Preview-calibrated pairs are counted separately and do not demonstrate live-HMD validation. Overlapping candidate windows are not independent evidence. Production yaw corrections were removed before fitting. Calibration that used firmware-held evidence retains that assumption. No corrections are applied.",
        configuration,
        records_read,
        frame_records,
        preview_calibrated_frames,
        live_hmd_calibrated_frames,
        unanchored_calibrated_frames,
        calibrated_with_firmware_hold_frames,
        eligible_pairs,
        preview_eligible_pairs,
        live_hmd_eligible_pairs,
        unanchored_eligible_pairs,
        reset_reasons,
        left_leg: left.report,
        right_leg: right.report,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use tracking_core::CalibrationStatus;

    fn frame(at: f64, production_yaw: f32) -> TrackingFrame {
        let parent = DQuat::from_rotation_x(0.12 * (at * 2.1).sin());
        let child = DQuat::from_rotation_y(15_f64.to_radians())
            * parent
            * DQuat::from_rotation_x(0.75 + 0.35 * (at * 5.).sin());
        let segment = |role, q: DQuat, correction: f32| SegmentPose {
            role,
            orientation_xyzw: (DQuat::from_rotation_y(f64::from(correction).to_radians()) * q)
                .to_array()
                .map(|v| v as f32),
            yaw_correction_degrees: correction,
            confidence: 0.95,
            valid: true,
            measurement_received_at_secs: Some(at),
            measurement_age_secs: Some(0.),
            uses_firmware_hold: false,
        };
        TrackingFrame {
            at_secs: at,
            preview_motion: Default::default(),
            estimator: Default::default(),
            motion_calibration: None,
            targets: vec![],
            joints: vec![],
            bones: vec![],
            segments: vec![
                segment(Role::LeftThigh, parent, production_yaw),
                segment(Role::LeftCalf, child, -production_yaw),
                segment(Role::RightThigh, parent, production_yaw),
                segment(Role::RightCalf, child, -production_yaw),
            ],
            calibration: CalibrationStatus {
                calibrated: true,
                direction_calibrated: true,
                reference_forward: Some([0., 0., -1.]),
                preview_only: false,
                requires_session_calibration: false,
                reason: None,
                used_firmware_hold: false,
                floor_mismatch_m: None,
            },
            recovery: RecoveryState::Idle,
            corrections: vec![],
            live_anchor: true,
            preview_only: false,
        }
    }
    fn push(bytes: &mut Vec<u8>, record: Record) {
        serde_json::to_writer(&mut *bytes, &record).unwrap();
        bytes.push(b'\n');
    }
    fn recording(production_yaw: f32) -> Vec<u8> {
        let mut bytes = vec![];
        for step in 0..=120 {
            push(
                &mut bytes,
                Record::Frame {
                    frame: frame(f64::from(step) / 100., production_yaw),
                },
            );
        }
        bytes
    }

    #[test]
    fn synthetic_drift_recording_yields_only_a_model_conditional_candidate() {
        let report = analyze(Cursor::new(recording(0.))).unwrap();
        assert_eq!(report.eligible_pairs, 242);
        assert_eq!(report.assessment, "model_conditional_candidates_only");
        assert_eq!(
            report.left_leg.last_report.status,
            HeadingShadowStatus::ModelConditionalCandidate
        );
        assert!(
            (report
                .left_leg
                .last_report
                .candidate_relative_yaw_radians
                .unwrap()
                .to_degrees()
                + 15.)
                .abs()
                < 0.01
        );
        assert_eq!(report.independent_witnesses_supplied, 0);
        assert!(!report.network_output_sent);
        assert!(!report.accuracy_proof);
    }
    #[test]
    fn production_yaw_is_removed_instead_of_fitting_its_own_correction() {
        let raw = analyze(Cursor::new(recording(0.))).unwrap();
        let corrected = analyze(Cursor::new(recording(7.5))).unwrap();
        let a = raw
            .left_leg
            .last_report
            .candidate_relative_yaw_radians
            .unwrap();
        let b = corrected
            .left_leg
            .last_report
            .candidate_relative_yaw_radians
            .unwrap();
        assert!((a - b).abs() < 1e-6);
        assert!(b.to_degrees().abs() > 14.9);
    }
    #[test]
    fn cached_timestamps_and_held_pairs_do_not_become_new_evidence() {
        let mut bytes = recording(0.);
        for step in 121..=125 {
            let mut f = frame(f64::from(step) / 100., 0.);
            for segment in &mut f.segments {
                segment.measurement_received_at_secs = Some(1.2);
                segment.measurement_age_secs = Some(f.at_secs - 1.2);
            }
            push(&mut bytes, Record::Frame { frame: f });
        }
        let cached = analyze(Cursor::new(&bytes)).unwrap();
        assert_eq!(cached.eligible_pairs, 242);
        assert_eq!(
            cached.left_leg.excluded_pair_reasons["reused_original_measurement"],
            5
        );
        let mut held = frame(1.26, 0.);
        for segment in &mut held.segments {
            segment.uses_firmware_hold = true;
        }
        push(&mut bytes, Record::Frame { frame: held });
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.eligible_pairs, 242);
        assert_eq!(report.left_leg.last_report.measured_pairs, 0);
        assert_eq!(
            report.left_leg.excluded_pair_reasons["firmware_held_orientation"],
            1
        );
    }
    #[test]
    fn reset_events_and_invalid_epochs_clear_both_leg_windows() {
        for record in [
            Record::Clear { at: 1.21 },
            Record::Calibrate {
                at: 1.21,
                preview: false,
            },
            Record::Recovery { at: 1.21 },
            Record::UndoRecovery { at: 1.21 },
            Record::RevokeHold {
                at: 1.21,
                id: "local-test".into(),
                unhealthy: false,
            },
        ] {
            let mut bytes = recording(0.);
            push(&mut bytes, record);
            push(
                &mut bytes,
                Record::Frame {
                    frame: frame(1.22, 0.),
                },
            );
            let report = analyze(Cursor::new(bytes)).unwrap();
            assert_eq!(report.left_leg.last_report.measured_pairs, 1);
            assert_eq!(report.right_leg.last_report.measured_pairs, 1);
        }
        let mut bytes = recording(0.);
        let mut invalid = frame(1.21, 0.);
        invalid.calibration.calibrated = false;
        push(&mut bytes, Record::Frame { frame: invalid });
        push(
            &mut bytes,
            Record::Frame {
                frame: frame(1.22, 0.),
            },
        );
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.left_leg.last_report.measured_pairs, 1);
    }
    #[test]
    fn ending_on_a_reset_clears_the_current_candidate_but_retains_history_counts() {
        let profile = || tracking_core::Profile {
            schema_version: 1,
            assignments: vec![],
            anthropometrics: tracking_core::TrackingConfig::default().anthropometrics,
            mounting: vec![],
            direction_calibrated_roles: vec![],
        };
        for record in [
            Record::Clear { at: 1.25 },
            Record::RevokeHold {
                at: 1.25,
                id: "test".into(),
                unhealthy: true,
            },
            Record::Configuration {
                at: 1.25,
                config: tracking_core::TrackingConfig::default(),
                profile: profile(),
            },
        ] {
            let mut bytes = recording(0.);
            push(&mut bytes, record);
            let report = analyze(Cursor::new(bytes)).unwrap();
            assert!(report.left_leg.model_conditional_candidate_reports > 0);
            assert_eq!(report.left_leg.last_report.measured_pairs, 0);
            assert_eq!(
                report.left_leg.last_report.candidate_relative_yaw_radians,
                None
            );
            assert_eq!(
                report
                    .left_leg
                    .last_report
                    .hypothetical_parent_child_yaw_radians,
                None
            );
            assert_eq!(report.left_leg.last_report.evaluated_at_secs, Some(1.25));
        }
    }
    #[test]
    fn corrupt_scaled_quaternions_are_rejected_before_yaw_undo() {
        let mut bytes = vec![];
        let mut f = frame(0., 0.);
        for segment in &mut f.segments {
            segment.orientation_xyzw = segment.orientation_xyzw.map(|value| value * 3.);
        }
        push(&mut bytes, Record::Frame { frame: f });
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.eligible_pairs, 0);
        assert_eq!(
            report.left_leg.excluded_pair_reasons["invalid_quaternion_or_yaw_correction"],
            1
        );
    }
    #[test]
    fn invalid_frame_epoch_does_not_allow_cached_measurements_to_be_counted_again() {
        let mut bytes = recording(0.);
        push(
            &mut bytes,
            Record::Frame {
                frame: frame(1.2, 0.),
            },
        );
        let mut cached = frame(1.21, 0.);
        for segment in &mut cached.segments {
            segment.measurement_received_at_secs = Some(1.2);
            segment.measurement_age_secs = Some(0.01);
        }
        push(&mut bytes, Record::Frame { frame: cached });
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.eligible_pairs, 242);
        assert_eq!(
            report.left_leg.excluded_pair_reasons["invalid_or_nonmonotonic_frame_time"],
            1
        );
        assert_eq!(
            report.left_leg.excluded_pair_reasons["reused_original_measurement"],
            1
        );
        assert_eq!(report.left_leg.last_report.measured_pairs, 0);
    }
    #[test]
    fn preview_model_analysis_is_counted_separately_from_live_hmd_evidence() {
        let mut bytes = vec![];
        for step in 0..=120 {
            let mut f = frame(f64::from(step) / 100., 0.);
            f.preview_only = true;
            f.calibration.preview_only = true;
            f.live_anchor = false;
            push(&mut bytes, Record::Frame { frame: f });
        }
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.eligible_pairs, 242);
        assert_eq!(report.preview_eligible_pairs, 242);
        assert_eq!(report.live_hmd_eligible_pairs, 0);
        assert_eq!(report.preview_calibrated_frames, 121);
        assert_eq!(
            report.left_leg.last_report.status,
            HeadingShadowStatus::ModelConditionalCandidate
        );
        assert!(!report.accuracy_proof);
    }
    #[test]
    fn old_metadata_missing_and_offbody_frames_have_zero_eligible_pairs() {
        let mut bytes = vec![];
        let mut old = serde_json::to_value(Record::Frame {
            frame: frame(0., 0.),
        })
        .unwrap();
        for segment in old["frame"]["segments"].as_array_mut().unwrap() {
            let object = segment.as_object_mut().unwrap();
            object.remove("uses_firmware_hold");
            object.remove("measurement_received_at_secs");
            object.remove("measurement_age_secs");
        }
        serde_json::to_writer(&mut bytes, &old).unwrap();
        bytes.push(b'\n');
        let report = analyze(Cursor::new(bytes)).unwrap();
        assert_eq!(report.eligible_pairs, 0);
        assert_eq!(report.assessment, "no_eligible_pairs");
        assert_eq!(
            report.left_leg.excluded_pair_reasons["missing_original_measurement_timestamp"],
            1
        );
        let empty = analyze(Cursor::new(Vec::<u8>::new())).unwrap();
        assert_eq!(empty.eligible_pairs, 0);
        let mut offbody = frame(0., 0.);
        offbody.calibration.calibrated = false;
        let mut bytes = vec![];
        push(&mut bytes, Record::Frame { frame: offbody });
        assert_eq!(analyze(Cursor::new(bytes)).unwrap().eligible_pairs, 0);
    }
    #[test]
    fn neutral_only_and_legacy_frames_do_not_supply_mounting_evidence() {
        for legacy in [false, true] {
            let mut record = serde_json::to_value(Record::Frame {
                frame: frame(1., 0.),
            })
            .unwrap();
            let calibration = record["frame"]["calibration"].as_object_mut().unwrap();
            if legacy {
                calibration.remove("direction_calibrated");
                calibration.remove("reference_forward");
            } else {
                calibration.insert("direction_calibrated".into(), false.into());
            }
            let mut bytes = serde_json::to_vec(&record).unwrap();
            bytes.push(b'\n');
            let report = analyze(Cursor::new(bytes)).unwrap();
            assert_eq!(report.eligible_pairs, 0);
            assert_eq!(
                report.left_leg.excluded_pair_reasons["direction_not_calibrated"],
                1
            );
        }
    }
    #[test]
    fn report_file_is_never_overwritten_and_large_lines_are_rejected() {
        let report = analyze(Cursor::new(Vec::<u8>::new())).unwrap();
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "chadfbt-heading-no-overwrite-{}-{unique}.json",
            std::process::id()
        ));
        std::fs::write(&path, b"keep existing").unwrap();
        assert_eq!(
            write_report(&path, &report).unwrap_err().kind(),
            io::ErrorKind::AlreadyExists
        );
        assert_eq!(std::fs::read(&path).unwrap(), b"keep existing");
        std::fs::remove_file(path).unwrap();
        assert!(analyze(Cursor::new(vec![b'x'; MAX_LINE_BYTES + 1])).is_err());
    }
}
