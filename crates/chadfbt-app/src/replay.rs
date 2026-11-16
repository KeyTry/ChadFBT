//! Offline re-solve of a recording with the current estimator code:
//! `chadfbt --replay FILE --dump OUT.jsonl [--every N] [--estimator JSON]`.
//!
//! Input records (sensor samples, headset anchors, calibration and lifecycle events) are
//! applied in file order with the same calls the live worker made. A recording without solved
//! frames gets synthetic solve times from the live pacing rule (`runtime::solve_due`: after new
//! data once 4 ms have passed, otherwise every 10 ms), so the result approximates live timing
//! but does not reproduce it bit for bit. When the recording does contain frames, their times
//! are the solve times instead. The output is model output for regression checks on real
//! motion; it is not a measurement of tracking accuracy and nothing is sent to VR software.

use crate::runtime::{Record, solve_due};
use serde::Serialize;
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
    time::Instant,
};
use tracking_core::{
    Anchors, CoreError, EstimatorStatus, JointPose, RecoveryState, SegmentPose, TargetPose,
    TrackingConfig, TrackingCore,
};

/// Options of `--dump`.
#[derive(Clone, Debug, Default)]
pub(crate) struct DumpOptions {
    /// Write every N-th solved frame (1 = all).
    pub every: u32,
    /// JSON object merged into the recorded `config.estimator` (header and configuration
    /// records) before it is applied.
    pub estimator_overrides: Option<serde_json::Value>,
    /// Keep the recorded body dimensions instead of applying the profile-load
    /// normalisation the current app applies.
    pub raw_dimensions: bool,
}

#[derive(Debug, Default, PartialEq)]
pub(crate) struct DumpSummary {
    pub solves: u64,
    pub written: u64,
    pub events: u64,
    pub failed_events: u64,
    pub outcome_mismatches: u64,
    pub solve_micros_median: f32,
    pub solve_micros_p99: f32,
    pub solve_micros_max: f32,
}

/// Applies one recorded lifecycle event exactly as the live worker did. Returns `None` for
/// records that are not events (samples, anchors, frames, configuration, header, transport).
pub(crate) fn apply_event(
    core: &mut TrackingCore,
    anchors: &Anchors,
    record: &Record,
) -> Option<(&'static str, f64, Result<(), CoreError>)> {
    Some(match record {
        Record::Calibrate { at, preview } => (
            "calibrate",
            *at,
            core.calibrate_neutral(*at, anchors, *preview),
        ),
        Record::ForwardCalibrate { at } => ("forward_calibrate", *at, core.calibrate_forward(*at)),
        Record::MotionBegin { at } => ("motion_begin", *at, core.begin_motion_calibration(*at)),
        Record::MotionFinish { at } => ("motion_finish", *at, core.finish_motion_calibration(*at)),
        Record::MotionCancel { at } => {
            core.cancel_motion_calibration();
            ("motion_cancel", *at, Ok(()))
        }
        Record::ManualMount { at, role, values } => {
            ("manual_mount", *at, core.set_manual_mount(*role, *values))
        }
        Record::ConfirmManualMount { at, role } => (
            "confirm_manual_mount",
            *at,
            core.confirm_manual_mount(*role),
        ),
        Record::UndoManualMount { at, role } => {
            ("undo_manual_mount", *at, core.undo_manual_mount(*role))
        }
        Record::ResetManualMount { at, role } => {
            ("reset_manual_mount", *at, core.reset_manual_mount(*role))
        }
        Record::Recovery { at } => {
            core.request_recovery(*at);
            ("recovery", *at, Ok(()))
        }
        Record::RevokeHold { at, id, unhealthy } => {
            if *unhealthy {
                core.mark_sensor_unhealthy(id);
            } else {
                core.revoke_firmware_hold(id);
            }
            ("revoke_hold", *at, Ok(()))
        }
        Record::UndoRecovery { at } => {
            core.undo_recovery();
            ("undo_recovery", *at, Ok(()))
        }
        Record::Clear { at } => {
            core.clear_calibration();
            ("clear", *at, Ok(()))
        }
        Record::Rebase { at, yaw_radians } => {
            core.rebase_standing_space(*yaw_radians);
            ("rebase", *at, Ok(()))
        }
        Record::TrackingSpaceEvent {
            at,
            transform_yaw_radians,
        } => {
            core.note_tracking_space_event(*at, *transform_yaw_radians);
            ("tracking_space_event", *at, Ok(()))
        }
        Record::AlignToHeadset { at } => (
            "align_to_headset",
            *at,
            core.align_preview_to_headset(*at, anchors),
        ),
        Record::Header { .. }
        | Record::Sample { .. }
        | Record::Anchors { .. }
        | Record::Configuration { .. }
        | Record::Frame { .. }
        | Record::Transport { .. } => return None,
    })
}

/// Recursively merges `patch` into `base`: objects merge key by key, anything else replaces.
fn merge_json(base: &mut serde_json::Value, patch: &serde_json::Value) {
    match (base, patch) {
        (serde_json::Value::Object(base), serde_json::Value::Object(patch)) => {
            for (key, value) in patch {
                merge_json(
                    base.entry(key.clone()).or_insert(serde_json::Value::Null),
                    value,
                );
            }
        }
        (base, patch) => *base = patch.clone(),
    }
}

/// The recorded configuration with the dump's estimator overrides applied.
fn effective_config(
    mut config: TrackingConfig,
    options: &DumpOptions,
) -> Result<TrackingConfig, Box<dyn Error>> {
    if let Some(patch) = &options.estimator_overrides {
        let mut value = serde_json::to_value(&config.estimator)?;
        merge_json(&mut value, patch);
        config.estimator = serde_json::from_value(value)
            .map_err(|error| format!("--estimator overrides: {error}"))?;
    }
    config.validate()?;
    Ok(config)
}

#[derive(Serialize)]
struct DumpFrame<'a> {
    kind: &'static str,
    at: f64,
    solve_micros: f32,
    /// Live frame (headset, direction calibration, not preview).
    live: bool,
    calibrated: bool,
    direction_calibrated: bool,
    recovery: &'a RecoveryState,
    joints: &'a [JointPose],
    segments: &'a [SegmentPose],
    targets: &'a [TargetPose],
    estimator: &'a EstimatorStatus,
}

struct Replayer<'w, W: Write> {
    core: TrackingCore,
    anchors: Anchors,
    options: DumpOptions,
    writer: &'w mut W,
    clock: f64,
    last_solve: Option<f64>,
    new_data: bool,
    /// The recording has solved frames: their times drive the solves.
    frame_driven: bool,
    summary: DumpSummary,
    micros: Vec<f32>,
}

impl<W: Write> Replayer<'_, W> {
    fn solve(&mut self, at: f64) -> Result<(), Box<dyn Error>> {
        let started = Instant::now();
        let frame = self.core.solve(at, &self.anchors);
        let micros = (started.elapsed().as_secs_f64() * 1e6) as f32;
        self.micros.push(micros);
        self.last_solve = Some(at);
        self.new_data = false;
        self.summary.solves += 1;
        if self
            .summary
            .solves
            .is_multiple_of(u64::from(self.options.every.max(1)))
        {
            let line = DumpFrame {
                kind: "frame",
                at,
                solve_micros: micros,
                live: !frame.preview_only,
                calibrated: frame.calibration.calibrated,
                direction_calibrated: frame.calibration.direction_calibrated,
                recovery: &frame.recovery,
                joints: &frame.joints,
                segments: &frame.segments,
                targets: &frame.targets,
                estimator: &frame.estimator,
            };
            serde_json::to_writer(&mut *self.writer, &line)?;
            self.writer.write_all(b"\n")?;
            self.summary.written += 1;
        }
        Ok(())
    }

    /// Solves the live worker would have made before an input arriving at `at`.
    fn catch_up(&mut self, at: f64) -> Result<(), Box<dyn Error>> {
        if self.frame_driven || !at.is_finite() {
            return Ok(());
        }
        while let Some(last) = self.last_solve {
            let due = last + crate::runtime::MAX_SOLVE_INTERVAL_SECS;
            if due >= at {
                break;
            }
            self.solve(due)?;
        }
        self.clock = self.clock.max(at);
        Ok(())
    }

    fn solve_if_due(&mut self) -> Result<(), Box<dyn Error>> {
        if !self.frame_driven && solve_due(self.clock, self.last_solve, self.new_data) {
            let at = self
                .last_solve
                .map_or(self.clock, |last| self.clock.max(last));
            self.solve(at)?;
        }
        Ok(())
    }

    fn write_json(&mut self, value: &serde_json::Value) -> Result<(), Box<dyn Error>> {
        serde_json::to_writer(&mut *self.writer, value)?;
        self.writer.write_all(b"\n")?;
        Ok(())
    }

    fn configure(
        &mut self,
        config: TrackingConfig,
        profile: tracking_core::Profile,
        header: bool,
    ) -> Result<(), Box<dyn Error>> {
        let mut config = effective_config(config, &self.options)?;
        let mut profile = profile;
        // The app normalises body dimensions on profile load and whenever the eye
        // height is set; do the same unless asked to keep the recorded ones.
        // The live worker records the core's configuration and profile, whose
        // dimensions are identical; keep them identical after normalising so a
        // configuration record never looks like a dimension change.
        let dimensions = if self.options.raw_dimensions {
            None
        } else {
            let note = crate::runtime::normalize_profile_dimensions(&mut profile.anthropometrics);
            config.anthropometrics = profile.anthropometrics.clone();
            note
        };
        if header {
            self.core = TrackingCore::new(config)?;
            self.core.restore_profile(profile)?;
        } else {
            self.core.set_config(config)?;
            self.core.set_assignments(profile.assignments)?;
        }
        let value = serde_json::json!({
            "kind": if header { "header" } else { "configuration" },
            "at": self.clock,
            "config": self.core.config(),
            "dimensions": dimensions,
            "note": "Re-solved offline with the current code; synthetic solve times unless the recording had frames. Model output, not accuracy.",
        });
        self.write_json(&value)
    }

    fn record(&mut self, record: Record) -> Result<(), Box<dyn Error>> {
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
                self.configure(config, profile, true)?;
            }
            Record::Configuration {
                at,
                config,
                profile,
            } => {
                self.catch_up(at)?;
                self.configure(config, profile, false)?;
                self.solve_if_due()?;
            }
            Record::Sample { sample, rejected } => {
                self.catch_up(sample.received_at_secs)?;
                let replayed_rejected = self.core.ingest(sample).is_err();
                if replayed_rejected != rejected.is_some() {
                    self.summary.outcome_mismatches += 1;
                }
                self.new_data = true;
                self.solve_if_due()?;
            }
            Record::Anchors { at, anchors } => {
                self.catch_up(at)?;
                self.anchors = anchors;
                self.new_data = true;
                self.solve_if_due()?;
            }
            Record::Frame { frame } => {
                self.frame_driven = true;
                self.clock = self.clock.max(frame.at_secs);
                self.solve(frame.at_secs)?;
            }
            Record::Transport { .. } => {}
            event => {
                if let Some(at) = event_time(&event) {
                    self.catch_up(at)?;
                }
                if let Some((name, at, result)) = apply_event(&mut self.core, &self.anchors, &event)
                {
                    self.summary.events += 1;
                    if result.is_err() {
                        self.summary.failed_events += 1;
                    }
                    let value = serde_json::json!({
                        "kind": "event",
                        "event": name,
                        "at": at,
                        "ok": result.is_ok(),
                        "error": result.err().map(|error| error.to_string()),
                    });
                    self.write_json(&value)?;
                }
                self.solve_if_due()?;
            }
        }
        Ok(())
    }
}

fn event_time(record: &Record) -> Option<f64> {
    match record {
        Record::Calibrate { at, .. }
        | Record::ForwardCalibrate { at }
        | Record::MotionBegin { at }
        | Record::MotionFinish { at }
        | Record::MotionCancel { at }
        | Record::ManualMount { at, .. }
        | Record::ConfirmManualMount { at, .. }
        | Record::UndoManualMount { at, .. }
        | Record::ResetManualMount { at, .. }
        | Record::Recovery { at }
        | Record::RevokeHold { at, .. }
        | Record::UndoRecovery { at }
        | Record::Clear { at }
        | Record::Rebase { at, .. }
        | Record::TrackingSpaceEvent { at, .. }
        | Record::AlignToHeadset { at }
        | Record::Configuration { at, .. }
        | Record::Anchors { at, .. }
        | Record::Transport { at, .. } => Some(*at),
        Record::Sample { sample, .. } => Some(sample.received_at_secs),
        Record::Frame { frame } => Some(frame.at_secs),
        Record::Header { .. } => None,
    }
}

fn percentile(sorted: &[f32], p: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let index = ((sorted.len() - 1) as f32 * p).round() as usize;
    sorted[index.min(sorted.len() - 1)]
}

pub(crate) fn dump_reader(
    reader: impl BufRead,
    writer: &mut impl Write,
    options: &DumpOptions,
) -> Result<DumpSummary, Box<dyn Error>> {
    let mut replayer = Replayer {
        core: TrackingCore::new(TrackingConfig::default())?,
        anchors: Anchors::default(),
        options: options.clone(),
        writer,
        clock: 0.0,
        last_solve: None,
        new_data: false,
        frame_driven: false,
        summary: DumpSummary::default(),
        micros: Vec::new(),
    };
    for (index, line) in reader.lines().enumerate() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let record: Record =
            serde_json::from_str(&line).map_err(|e| format!("line {}: {e}", index + 1))?;
        replayer
            .record(record)
            .map_err(|e| format!("line {}: {e}", index + 1))?;
    }
    let mut micros = std::mem::take(&mut replayer.micros);
    micros.sort_by(f32::total_cmp);
    let mut summary = std::mem::take(&mut replayer.summary);
    summary.solve_micros_median = percentile(&micros, 0.5);
    summary.solve_micros_p99 = percentile(&micros, 0.99);
    summary.solve_micros_max = micros.last().copied().unwrap_or(0.0);
    let value = serde_json::json!({
        "kind": "summary",
        "solves": summary.solves,
        "written": summary.written,
        "events": summary.events,
        "failed_events": summary.failed_events,
        "outcome_mismatches": summary.outcome_mismatches,
        "solve_micros_median": summary.solve_micros_median,
        "solve_micros_p99": summary.solve_micros_p99,
        "solve_micros_max": summary.solve_micros_max,
    });
    replayer.write_json(&value)?;
    replayer.writer.flush()?;
    Ok(summary)
}

pub(crate) fn dump(
    input: &Path,
    output: &Path,
    options: &DumpOptions,
) -> Result<DumpSummary, Box<dyn Error>> {
    let reader = BufReader::new(File::open(input)?);
    let mut writer = BufWriter::new(File::create(output)?);
    dump_reader(reader, &mut writer, options)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracking_core::chain::ChainDimensions;
    use tracking_core::synthetic::{BodyGenerator, BodyGeometry, Motion, StandState};
    use tracking_core::{AnchorPose, Role, RoleAssignment, SensorProvenance, SensorSample};

    /// A synthetic input-only recording: header, 100 Hz samples and 90 Hz headset
    /// anchors, a live neutral calibration, a profile whose chain is 2 cm taller than
    /// the eye height, and a configuration record. No solved frames.
    fn input_only_recording() -> String {
        let generator = BodyGenerator::new(
            ChainDimensions::default(),
            BodyGeometry::default(),
            StandState::default(),
            &[Motion::stand(1.5), Motion::squats(1)],
        );
        let id = |i: usize| format!("dump/{i}");
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
        let mut profile = core.export_profile();
        profile.anthropometrics.chest_to_hip_m += 0.02;
        let line = |record: &Record| serde_json::to_string(record).unwrap() + "\n";
        let mut config = core.config().clone();
        config.estimator.enable_leg_ik = true;
        let mut text = line(&Record::Header {
            version: 1,
            config: config.clone(),
            profile: profile.clone(),
            note: String::new(),
        });
        let ticks = (generator.duration() * 100.0) as u64;
        for tick in 1..=ticks {
            let at = tick as f64 / 100.0;
            let truth = generator.frame(at);
            for (i, rotation) in truth.rotations.iter().enumerate() {
                let sample = SensorSample {
                    id: id(i),
                    orientation_xyzw: rotation.to_array(),
                    received_at_secs: at,
                    acceleration_mps2: None,
                    provenance: SensorProvenance::Measured,
                };
                text += &line(&Record::Sample {
                    sample,
                    rejected: None,
                });
            }
            if tick % 10 != 0 {
                let anchors = Anchors {
                    hmd: Some(AnchorPose {
                        position: truth.head_position.to_array(),
                        orientation_xyzw: truth.head_rotation.to_array(),
                        received_at_secs: at,
                        valid: true,
                        linear_velocity: None,
                        angular_velocity: None,
                    }),
                    ..Default::default()
                };
                text += &line(&Record::Anchors { at, anchors });
            }
            if tick == 100 {
                text += &line(&Record::Calibrate { at, preview: false });
            }
            if tick == 120 {
                text += &line(&Record::Configuration {
                    at,
                    config: config.clone(),
                    profile: profile.clone(),
                });
            }
        }
        text
    }

    fn lines(output: &[u8]) -> Vec<serde_json::Value> {
        String::from_utf8_lossy(output)
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect()
    }

    fn anthropometrics(record: &serde_json::Value) -> tracking_core::Anthropometrics {
        serde_json::from_value(record["config"]["anthropometrics"].clone()).unwrap()
    }

    #[test]
    fn input_only_recordings_are_resolved_with_live_pacing_and_events_in_order() {
        let text = input_only_recording();
        let mut output = Vec::new();
        let options = DumpOptions {
            every: 4,
            estimator_overrides: Some(serde_json::json!({"leg_ik": {"root_gain": 0.4}})),
            raw_dimensions: false,
        };
        let summary = dump_reader(text.as_bytes(), &mut output, &options).unwrap();
        let records = lines(&output);
        // 4.5 s of input with 100 Hz samples: solves follow the 100-250 Hz live pacing.
        assert!(
            (440..=1130).contains(&summary.solves),
            "{} solves",
            summary.solves
        );
        assert_eq!(summary.written, summary.solves / 4);
        assert_eq!((summary.events, summary.failed_events), (1, 0));
        assert_eq!(summary.outcome_mismatches, 0);
        let header = &records[0];
        assert_eq!(header["kind"], "header");
        let root_gain = header["config"]["estimator"]["leg_ik"]["root_gain"]
            .as_f64()
            .unwrap();
        assert!((root_gain - 0.4).abs() < 1.0e-6, "{root_gain}");
        // The profile was made consistent the way the app does on load.
        let body = anthropometrics(header);
        assert!((body.standing_chain_height_m() - body.standing_eye_height_m).abs() < 1.0e-5);
        assert!(header["dimensions"].as_str().unwrap().contains("adjusted"));
        // The configuration record is normalised the same way, so it does not
        // count as a body-dimension change (which would void the calibration).
        let configuration = records
            .iter()
            .find(|record| record["kind"] == "configuration")
            .unwrap();
        assert_eq!(anthropometrics(configuration), body);
        // Events, configuration and frames appear in time order.
        let mut last = f64::NEG_INFINITY;
        let mut calibrated = false;
        let mut frames_after = 0;
        for record in &records[1..records.len() - 1] {
            let at = record["at"].as_f64().unwrap();
            assert!(at >= last - 1.0e-9, "{record}");
            last = at;
            match record["kind"].as_str().unwrap() {
                "event" => {
                    assert_eq!(record["event"], "calibrate");
                    assert_eq!(record["ok"], true);
                    calibrated = true;
                }
                "frame" if calibrated => {
                    assert_eq!(record["calibrated"], true);
                    frames_after += 1;
                }
                _ => {}
            }
        }
        assert!(frames_after > 50);
        assert_eq!(records.last().unwrap()["kind"], "summary");

        // Deterministic apart from the measured solve times.
        let mut again = Vec::new();
        dump_reader(text.as_bytes(), &mut again, &options).unwrap();
        let strip = |mut value: serde_json::Value| {
            if let Some(object) = value.as_object_mut() {
                object.retain(|key, _| !key.starts_with("solve_micros"));
            }
            value
        };
        let (a, b) = (lines(&output), lines(&again));
        assert_eq!(a.len(), b.len());
        for (x, y) in a.into_iter().zip(b) {
            assert_eq!(strip(x), strip(y));
        }
    }

    #[test]
    fn raw_dimensions_keep_the_profile_and_invalid_overrides_are_rejected() {
        let text = input_only_recording();
        let mut output = Vec::new();
        let options = DumpOptions {
            every: 1000,
            estimator_overrides: None,
            raw_dimensions: true,
        };
        dump_reader(text.as_bytes(), &mut output, &options).unwrap();
        let body = anthropometrics(&lines(&output)[0]);
        let excess = body.standing_chain_height_m() - body.standing_eye_height_m;
        assert!((excess - 0.02).abs() < 1.0e-5, "{excess}");
        for overrides in [
            serde_json::json!({"leg_ik": {"root_gain": -1.0}}),
            serde_json::json!({"stance": {"gate_chi2": "x"}}),
        ] {
            let invalid = DumpOptions {
                every: 1,
                estimator_overrides: Some(overrides),
                raw_dimensions: false,
            };
            assert!(dump_reader(text.as_bytes(), &mut Vec::new(), &invalid).is_err());
        }
    }
}
