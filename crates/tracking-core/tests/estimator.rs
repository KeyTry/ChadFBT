//! Synthetic regressions for orientation prediction, output velocities, HMD
//! extrapolation and automatic standing-space rebasing. These check the
//! implementation against its own model; they say nothing about accuracy on a
//! real body or against other tracking software.

use glam::{Quat, Vec3};
use tracking_core::forward_calibration::ski_pitch_sign;
use tracking_core::*;

fn index(role: Role) -> usize {
    Role::ALL.iter().position(|r| *r == role).unwrap()
}
fn id(role: Role) -> String {
    format!("estimator-{role:?}")
}
fn mount(role: Role) -> Quat {
    let i = index(role) as f32;
    Quat::from_rotation_x(0.3 + 0.23 * i)
        * Quat::from_rotation_z(-0.7 + 0.31 * i)
        * Quat::from_rotation_y(0.4 + 0.41 * i)
}
fn bias(role: Role) -> f32 {
    [-1.4, 2.2, 1.5707964, -2.7, 0.9, -0.4, 0.3, 0.7, -1.1, 2.8][index(role)]
}
fn pitch(role: Role) -> f32 {
    ski_pitch_sign(role).unwrap_or(0.0)
        * [28.0_f32, 22.0, 38.0, 34.0, 18.0, 20.0, 0.0, 0.0, 0.0, 0.0][index(role)].to_radians()
}
/// Raw sensor quaternion for a standing-space body segment rotation.
fn raw(role: Role, body: Quat) -> Quat {
    Quat::from_rotation_y(bias(role)) * body * mount(role)
}
const HEADING: f32 = 0.4;

fn hmd(time: f64, position: Vec3, rotation: Quat) -> AnchorPose {
    AnchorPose {
        position: position.to_array(),
        orientation_xyzw: rotation.to_array(),
        received_at_secs: time,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    }
}
fn upright(time: f64, height: f32) -> Anchors {
    Anchors {
        hmd: Some(hmd(
            time,
            Vec3::new(0.0, height, 0.0),
            Quat::from_rotation_y(HEADING),
        )),
        ..Default::default()
    }
}
fn ingest(core: &mut TrackingCore, role: Role, time: f64, body: Quat) {
    core.ingest(SensorSample {
        id: id(role),
        orientation_xyzw: raw(role, body).to_array(),
        received_at_secs: time,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    })
    .unwrap();
}
/// `wobble` adds a small 3 Hz sway, within the stability gates, so the rate
/// estimators see nonzero motion during calibration.
fn feed_calibration(
    core: &mut TrackingCore,
    ticks: std::ops::RangeInclusive<usize>,
    ski: bool,
    wobble: bool,
) {
    for tick in ticks {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            let sway = if wobble {
                0.3_f32.to_radians() * (std::f32::consts::TAU * 3.0 * time as f32).sin()
            } else {
                0.0
            };
            let bend = if ski { pitch(role) } else { 0.0 };
            let body = Quat::from_rotation_y(HEADING) * Quat::from_rotation_x(bend + sway);
            ingest(core, role, time, body);
        }
        core.solve(time, &upright(time, if ski { 1.1 } else { 1.65 }));
    }
}
fn aligned_with(config: TrackingConfig, wobble: bool) -> TrackingCore {
    let mut core = TrackingCore::new(config).unwrap();
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
    feed_calibration(&mut core, 0..=100, false, wobble);
    core.calibrate_neutral(1.0, &upright(1.0, 1.65), false)
        .unwrap();
    feed_calibration(&mut core, 101..=200, true, wobble);
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}
fn aligned() -> TrackingCore {
    aligned_with(TrackingConfig::default(), false)
}
fn all_estimators() -> EstimatorConfig {
    EstimatorConfig {
        enable_prediction: true,
        enable_hmd_extrapolation: true,
        enable_auto_rebase: true,
        ..Default::default()
    }
}
fn enable(core: &mut TrackingCore, estimator: EstimatorConfig) {
    let mut config = core.config().clone();
    config.estimator = estimator;
    core.set_config(config).unwrap();
    assert!(core.direction_calibrated());
}
fn with_prediction(lead_secs: f32) -> TrackingConfig {
    TrackingConfig {
        estimator: EstimatorConfig {
            enable_prediction: true,
            prediction_lead_secs: lead_secs,
            ..Default::default()
        },
        ..Default::default()
    }
}
fn segment(frame: &TrackingFrame, role: Role) -> &SegmentPose {
    frame.segments.iter().find(|s| s.role == role).unwrap()
}
fn target(frame: &TrackingFrame, role: TargetRole) -> &TargetPose {
    frame.targets.iter().find(|t| t.role == role).unwrap()
}
fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos()
}
fn all_finite(frame: &TrackingFrame) -> bool {
    let finite = |v: &[f32]| v.iter().all(|x| x.is_finite());
    frame.targets.iter().all(|t| {
        finite(&t.position)
            && finite(&t.orientation_xyzw)
            && finite(&t.linear_velocity)
            && finite(&t.angular_velocity)
            && t.confidence.is_finite()
    }) && frame.joints.iter().all(|j| finite(&j.position))
        && frame.segments.iter().all(|s| finite(&s.orientation_xyzw))
        && frame.estimator.hmd_extrapolated_secs.is_finite()
        && frame.estimator.last_rebase_degrees.is_finite()
}

// Chest rotates at a constant standing-space rate about a tilted axis; every
// other sensor is still. Samples every 10 ms from `START`.
const START: f64 = 2.01;
const SAMPLES: usize = 30;
fn chest_rate() -> Vec3 {
    Vec3::new(0.3, 1.0, 0.2).normalize() * 3.0
}
fn chest_truth(time: f64) -> Quat {
    Quat::from_scaled_axis(chest_rate() * (time - START) as f32) * Quat::from_rotation_y(HEADING)
}
fn sample_time(k: usize) -> f64 {
    START + k as f64 * 0.01
}
fn feed_rotating_chest(core: &mut TrackingCore, k: usize) {
    let time = sample_time(k);
    for role in Role::CORE {
        let body = if role == Role::Chest {
            chest_truth(time)
        } else {
            Quat::from_rotation_y(HEADING)
        };
        ingest(core, role, time, body);
    }
}
/// Feeds the rotating chest, solving 8 ms after every sample; returns the
/// time of the last sample.
fn run_rotation(core: &mut TrackingCore) -> f64 {
    for k in 0..SAMPLES {
        feed_rotating_chest(core, k);
        let solve_at = sample_time(k) + 0.008;
        core.solve(solve_at, &upright(solve_at, 1.65));
    }
    sample_time(SAMPLES - 1)
}

#[test]
fn old_configurations_load_with_every_estimator_feature_off() {
    let mut value = serde_json::to_value(TrackingConfig::default()).unwrap();
    value.as_object_mut().unwrap().remove("estimator");
    let config: TrackingConfig = serde_json::from_value(value).unwrap();
    assert_eq!(config.estimator, EstimatorConfig::default());
    assert!(!config.estimator.enable_prediction);
    let mut invalid = TrackingConfig::default();
    for lead in [-0.001, 0.031, f32::NAN] {
        invalid.estimator.prediction_lead_secs = lead;
        assert!(invalid.validate().is_err(), "{lead}");
        assert!(TrackingCore::new(invalid.clone()).is_err());
    }
    invalid.estimator.prediction_lead_secs = 0.03;
    assert!(invalid.validate().is_ok());
    invalid.estimator.prediction.silence_secs = f64::NAN;
    assert!(invalid.validate().is_err());
    // Old recorded targets and anchors (no velocity fields) still parse.
    let target: TargetPose = serde_json::from_str(
        r#"{"role":"chest","position":[0,1,0],"orientation_xyzw":[0,0,0,1],
            "position_source":"inferred","orientation_source":"measured_orientation",
            "confidence":1,"valid":true,"reason":null}"#,
    )
    .unwrap();
    assert_eq!(target.linear_velocity, [0.0; 3]);
    let anchor: AnchorPose = serde_json::from_str(
        r#"{"position":[0,1.6,0],"orientation_xyzw":[0,0,0,1],"received_at_secs":1,"valid":true}"#,
    )
    .unwrap();
    assert!(anchor.linear_velocity.is_none());
}

#[test]
fn prediction_off_outputs_the_latest_sample_bit_for_bit() {
    // Prediction disabled but its extrapolation parameters changed: outputs
    // must be exactly those of the default core. (The stance-heading and leg
    // IK flags are no longer reserved; tests/integration_estimator.rs covers
    // them.)
    let mut changed = TrackingConfig::default();
    changed.estimator.prediction_lead_secs = 0.03;
    changed.estimator.prediction.max_extrapolation_secs = 0.08;
    let mut reference = aligned();
    let mut other = aligned_with(changed, false);
    let mut last = None;
    for k in 0..SAMPLES {
        feed_rotating_chest(&mut reference, k);
        feed_rotating_chest(&mut other, k);
        let mut pair = Vec::new();
        for at in [sample_time(k), sample_time(k) + 0.008] {
            let a = reference.solve(at, &upright(sample_time(k), 1.65));
            let b = other.solve(at, &upright(sample_time(k), 1.65));
            assert_eq!(
                serde_json::to_string(&a).unwrap(),
                serde_json::to_string(&b).unwrap()
            );
            assert!(!a.estimator.prediction_active);
            pair.push(a);
        }
        last = Some(pair);
    }
    // Without prediction the published chest is the last measured sample: the
    // same at the sample time and 8 ms later, for orientation and position.
    let last = last.unwrap();
    let (at_sample, later) = (&last[0], &last[1]);
    assert_eq!(
        segment(at_sample, Role::Chest).orientation_xyzw,
        segment(later, Role::Chest).orientation_xyzw
    );
    assert!(later.targets.iter().all(|t| t.valid));
    for (a, b) in at_sample.targets.iter().zip(&later.targets) {
        assert_eq!(a.position, b.position);
        assert_eq!(a.orientation_xyzw, b.orientation_xyzw);
    }
    let last = later.clone();
    // The measured rate is still published as target angular velocity.
    let omega = Vec3::from_array(target(&last, TargetRole::Chest).angular_velocity);
    assert!((omega - chest_rate()).length() < 0.03 * chest_rate().length());
}

#[test]
fn prediction_tracks_a_constant_rate_rotation_to_the_solve_time() {
    let mut predicted = aligned_with(with_prediction(0.0), false);
    let mut latest = aligned();
    let t_last = run_rotation(&mut predicted);
    run_rotation(&mut latest);
    let now = t_last + 0.008;
    let a = predicted.solve(now, &upright(now, 1.65));
    let b = latest.solve(now, &upright(now, 1.65));
    let truth = chest_truth(now);
    let predicted_error = angle(
        Quat::from_array(segment(&a, Role::Chest).orientation_xyzw),
        truth,
    )
    .to_degrees();
    let latest_error = angle(
        Quat::from_array(segment(&b, Role::Chest).orientation_xyzw),
        truth,
    )
    .to_degrees();
    let expected_lag = (chest_rate().length() * 0.008).to_degrees();
    assert!(predicted_error < 0.3, "predicted error {predicted_error}°");
    assert!(
        (latest_error - expected_lag).abs() < 0.05,
        "unpredicted error {latest_error}°, expected {expected_lag}°"
    );
    assert!(a.estimator.prediction_active);
    // The chest target is extrapolated with the segment; the still hip is not.
    assert!(
        angle(
            Quat::from_array(target(&a, TargetRole::Chest).orientation_xyzw),
            truth
        )
        .to_degrees()
            < 0.3
    );
    // (Only the decaying rate left from the calibration bend remains.)
    assert!(
        angle(
            Quat::from_array(segment(&a, Role::Hip).orientation_xyzw),
            Quat::from_array(segment(&b, Role::Hip).orientation_xyzw)
        ) < 1e-4
    );
    // Sensor-world rate rotated into standing space by the heading alignment
    // (each sensor here has a different world-yaw bias).
    let omega = Vec3::from_array(target(&a, TargetRole::Chest).angular_velocity);
    assert!(
        (omega - chest_rate()).length() < 0.02 * chest_rate().length(),
        "{omega:?}"
    );
    assert!(Vec3::from_array(target(&a, TargetRole::Waist).angular_velocity).length() < 1e-3);
    assert!(all_finite(&a));
    // A lead extrapolates beyond the solve time.
    let mut led = aligned_with(with_prediction(0.02), false);
    run_rotation(&mut led);
    let c = led.solve(now, &upright(now, 1.65));
    let lead_error = angle(
        Quat::from_array(segment(&c, Role::Chest).orientation_xyzw),
        chest_truth(now + 0.02),
    )
    .to_degrees();
    assert!(lead_error < 0.3, "lead error {lead_error}°");
}

#[test]
fn eighty_milliseconds_of_silence_stops_extrapolation() {
    let mut predicted = aligned_with(with_prediction(0.03), false);
    let mut latest = aligned();
    let t_last = run_rotation(&mut predicted);
    run_rotation(&mut latest);
    let now = t_last + 0.08;
    let a = predicted.solve(now, &upright(now, 1.65));
    let b = latest.solve(now, &upright(now, 1.65));
    let chest = Quat::from_array(segment(&a, Role::Chest).orientation_xyzw);
    assert!(
        angle(
            chest,
            Quat::from_array(segment(&b, Role::Chest).orientation_xyzw)
        ) < 1e-5
    );
    assert!(angle(chest, chest_truth(t_last)) < 0.1_f32.to_radians());
    assert!(segment(&a, Role::Chest).valid);
    assert!(!a.estimator.prediction_active);
    assert_eq!(target(&a, TargetRole::Chest).angular_velocity, [0.0; 3]);
}

#[test]
fn firmware_held_orientations_are_never_extrapolated() {
    // A long silence cutoff would otherwise keep extrapolating; the short
    // staleness limit makes the measurement stale while the hold is valid.
    let mut config = with_prediction(0.03);
    config.stale_after_secs = 0.05;
    config.estimator.prediction.silence_secs = 1.0;
    let mut predicted = aligned_with(config.clone(), false);
    config.estimator.enable_prediction = false;
    let mut latest = aligned_with(config, false);
    let t_last = run_rotation(&mut predicted);
    run_rotation(&mut latest);
    // Still fresh: the long silence cutoff extrapolates.
    let fresh_at = t_last + 0.04;
    let fresh = predicted.solve(fresh_at, &upright(fresh_at, 1.65));
    assert!(fresh.estimator.prediction_active);
    assert!(
        angle(
            Quat::from_array(segment(&fresh, Role::Chest).orientation_xyzw),
            chest_truth(t_last)
        ) > 0.05
    );
    let now = t_last + 0.1;
    for core in [&mut predicted, &mut latest] {
        for role in Role::CORE {
            let body = if role == Role::Chest {
                chest_truth(t_last)
            } else {
                Quat::from_rotation_y(HEADING)
            };
            core.ingest(SensorSample {
                id: id(role),
                orientation_xyzw: raw(role, body).to_array(),
                received_at_secs: t_last,
                acceleration_mps2: None,
                provenance: SensorProvenance::FirmwareHeld {
                    firmware: "0.7.2".into(),
                    observed_at_secs: now,
                    last_transport_at_secs: now,
                    continuity_since_secs: t_last,
                    sensor_status_healthy: true,
                    sequence_contiguous: true,
                },
            })
            .unwrap();
        }
    }
    let a = predicted.solve(now, &upright(now, 1.65));
    let b = latest.solve(now, &upright(now, 1.65));
    let chest = segment(&a, Role::Chest);
    assert!(chest.valid && chest.uses_firmware_hold);
    assert_eq!(
        chest.orientation_xyzw,
        segment(&b, Role::Chest).orientation_xyzw
    );
    assert!(!a.estimator.prediction_active);
    let chest_target = target(&a, TargetRole::Chest);
    assert!(chest_target.valid);
    assert_eq!(
        chest_target.orientation_source,
        Evidence::FirmwareHeldOrientation
    );
    assert_eq!(chest_target.angular_velocity, [0.0; 3]);
}

#[test]
fn calibration_and_recovery_ignore_prediction() {
    let mut on = aligned_with(with_prediction(0.03), true);
    let mut off = aligned_with(TrackingConfig::default(), true);
    assert_eq!(
        serde_json::to_string(&on.export_profile()).unwrap(),
        serde_json::to_string(&off.export_profile()).unwrap()
    );
    assert_eq!(on.calibration_forward(), off.calibration_forward());
    // Recovery: common-mode drift, swaying, then an explicit recovery.
    let mut frames = Vec::new();
    for core in [&mut on, &mut off] {
        core.request_recovery(2.3);
        let mut last = None;
        for tick in 201..=300 {
            let time = tick as f64 / 100.0;
            for role in Role::CORE {
                let sway = 0.3_f32.to_radians() * (std::f32::consts::TAU * 3.0 * time as f32).sin();
                let drifted = Quat::from_rotation_y(HEADING + 0.15) * Quat::from_rotation_x(sway);
                ingest(core, role, time, drifted);
            }
            last = Some(core.solve(time, &upright(time, 1.65)));
        }
        frames.push(last.unwrap());
    }
    assert!(frames[0].estimator.prediction_active);
    assert!(!frames[1].estimator.prediction_active);
    for frame in &frames {
        assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    }
    for (a, b) in frames[0].segments.iter().zip(&frames[1].segments) {
        assert_eq!(a.yaw_correction_degrees, b.yaw_correction_degrees);
    }
}

#[test]
fn target_linear_velocity_follows_a_translating_headset_and_is_zero_when_invalid() {
    let mut core = aligned();
    let velocity = Vec3::new(1.2, 0.0, -0.4);
    let head_at = |time: f64| Vec3::new(0.0, 1.65, 0.0) + velocity * (time - 2.0) as f32;
    let mut frame = None;
    for tick in 201..=240 {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            ingest(&mut core, role, time, Quat::from_rotation_y(HEADING));
        }
        let anchors = Anchors {
            hmd: Some(hmd(time, head_at(time), Quat::from_rotation_y(HEADING))),
            ..Default::default()
        };
        frame = Some(core.solve(time, &anchors));
    }
    let frame = frame.unwrap();
    assert!(frame.targets.iter().all(|t| t.valid));
    for t in &frame.targets {
        let v = Vec3::from_array(t.linear_velocity);
        assert!(
            (v - velocity).length() < 0.05 * velocity.length(),
            "{:?} {v:?}",
            t.role
        );
    }
    // The left calf stops reporting: the left foot becomes invalid and its
    // velocities are zero; the rest keep following the headset.
    let mut frame = None;
    for tick in 241..=280 {
        let time = tick as f64 / 100.0;
        for role in Role::CORE.into_iter().filter(|r| *r != Role::LeftCalf) {
            ingest(&mut core, role, time, Quat::from_rotation_y(HEADING));
        }
        let anchors = Anchors {
            hmd: Some(hmd(time, head_at(time), Quat::from_rotation_y(HEADING))),
            ..Default::default()
        };
        frame = Some(core.solve(time, &anchors));
    }
    let frame = frame.unwrap();
    let foot = target(&frame, TargetRole::LeftFoot);
    assert!(!foot.valid);
    assert_eq!(foot.linear_velocity, [0.0; 3]);
    assert_eq!(foot.angular_velocity, [0.0; 3]);
    let knee = target(&frame, TargetRole::LeftKnee);
    assert!(knee.valid);
    assert!(
        (Vec3::from_array(knee.linear_velocity) - velocity).length() < 0.05 * velocity.length()
    );
    // No headset: nothing is live, every velocity is zero.
    let dark = core.solve(2.81, &Anchors::default());
    assert!(
        dark.targets
            .iter()
            .all(|t| !t.valid && t.linear_velocity == [0.0; 3] && t.angular_velocity == [0.0; 3])
    );
}

#[test]
fn hmd_extrapolation_moves_the_body_by_velocity_times_age_only_when_trusted() {
    let mut core = aligned();
    enable(
        &mut core,
        EstimatorConfig {
            enable_hmd_extrapolation: true,
            ..Default::default()
        },
    );
    for tick in 201..=220 {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            ingest(&mut core, role, time, Quat::from_rotation_y(HEADING));
        }
        core.solve(time, &upright(time, 1.65));
    }
    let now = 2.2;
    let velocity = Vec3::new(0.5, -0.1, -0.25);
    let solve_with = |core: &mut TrackingCore, received_at: f64, v: Option<[f32; 3]>| {
        let mut head = hmd(
            received_at,
            Vec3::new(0.05, 1.63, 0.02),
            Quat::from_rotation_y(HEADING),
        );
        head.linear_velocity = v;
        core.solve(
            now,
            &Anchors {
                hmd: Some(head),
                ..Default::default()
            },
        )
    };
    for (received_at, expected_age) in [(2.18, 0.02_f32), (2.1, 0.03), (now, 0.0)] {
        let base = solve_with(&mut core, received_at, None);
        let moved = solve_with(&mut core, received_at, Some(velocity.to_array()));
        assert_eq!(base.estimator.hmd_extrapolated_secs, 0.0);
        assert!((moved.estimator.hmd_extrapolated_secs - expected_age).abs() < 1e-6);
        assert!(moved.targets.iter().all(|t| t.valid));
        for (a, b) in base.targets.iter().zip(&moved.targets) {
            let shift = Vec3::from_array(b.position) - Vec3::from_array(a.position);
            assert!(
                (shift - velocity * expected_age).length() < 1e-5,
                "{:?}",
                a.role
            );
            // Orientation is never extrapolated from the anchor.
            assert_eq!(a.orientation_xyzw, b.orientation_xyzw);
        }
    }
    for untrusted in [
        [f32::NAN, 0.0, 0.0],
        [f32::INFINITY, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [0.0, -12.0, 3.0],
    ] {
        let base = solve_with(&mut core, 2.18, None);
        let ignored = solve_with(&mut core, 2.18, Some(untrusted));
        assert_eq!(ignored.estimator.hmd_extrapolated_secs, 0.0);
        for (a, b) in base.targets.iter().zip(&ignored.targets) {
            assert_eq!(a.position, b.position);
        }
        assert!(all_finite(&ignored));
    }
}

struct Jump {
    rotation: Quat,
    shift: Vec3,
}
impl Jump {
    fn seventy_degrees() -> Self {
        Self {
            rotation: Quat::from_rotation_y(70.0_f32.to_radians()),
            shift: Vec3::new(0.5, 0.0, 0.0),
        }
    }
    fn anchors(&self, time: f64, position: Vec3, rotation: Quat) -> Anchors {
        Anchors {
            hmd: Some(hmd(
                time,
                self.rotation * position + self.shift,
                self.rotation * rotation,
            )),
            ..Default::default()
        }
    }
}
fn ingest_still(core: &mut TrackingCore, time: f64) {
    for role in Role::CORE {
        ingest(core, role, time, Quat::from_rotation_y(HEADING));
    }
}
fn assert_rigidly_moved(before: &TrackingFrame, after: &TrackingFrame, jump: &Jump) {
    assert_eq!(before.targets.len(), after.targets.len());
    for (a, b) in before.targets.iter().zip(&after.targets) {
        assert_eq!(a.role, b.role);
        assert!(a.valid && b.valid, "{:?}", a.role);
        let expected = jump.rotation * Vec3::from_array(a.position) + jump.shift;
        let error = (Vec3::from_array(b.position) - expected).length();
        assert!(error < 1e-3, "{:?} position error {error}", a.role);
        let rotated = jump.rotation * Quat::from_array(a.orientation_xyzw);
        let error = angle(Quat::from_array(b.orientation_xyzw), rotated);
        assert!(error < 1e-3, "{:?} orientation error {error}", a.role);
    }
}

#[test]
fn headset_tracking_jump_rebases_standing_space_and_keeps_calibration() {
    let mut core = aligned();
    enable(&mut core, all_estimators());
    let head_position = Vec3::new(0.1, 1.62, -0.05);
    let head_rotation = Quat::from_rotation_y(HEADING) * Quat::from_rotation_x(-0.2);
    let still = |time| Anchors {
        hmd: Some(hmd(time, head_position, head_rotation)),
        ..Default::default()
    };
    let mut before = None;
    for tick in 201..=230 {
        let time = tick as f64 / 100.0;
        ingest_still(&mut core, time);
        before = Some(core.solve(time, &still(time)));
    }
    let before = before.unwrap();
    assert_eq!(before.estimator.standing_space_rebases, 0);
    let forward_before = Vec3::from_array(core.calibration_forward().unwrap());
    let jump = Jump::seventy_degrees();
    // Identical IMU data, one frame later, with a rigidly jumped headset.
    ingest_still(&mut core, 2.31);
    let after = core.solve(2.31, &jump.anchors(2.31, head_position, head_rotation));
    assert_rigidly_moved(&before, &after, &jump);
    assert!(after.calibration.calibrated && after.calibration.direction_calibrated);
    assert!(core.direction_calibrated());
    assert_eq!(after.estimator.standing_space_rebases, 1);
    assert_eq!(after.estimator.tracking_discontinuities, 1);
    assert!((after.estimator.last_rebase_degrees - 70.0).abs() < 1e-3);
    assert!(
        (Vec3::from_array(core.calibration_forward().unwrap()) - jump.rotation * forward_before)
            .length()
            < 1e-4
    );
    // The jump is not reported as body velocity, and later frames neither
    // rebase again nor drift.
    assert!(after.targets.iter().all(|t| t.linear_velocity == [0.0; 3]));
    let mut later = None;
    for tick in 232..=260 {
        let time = tick as f64 / 100.0;
        ingest_still(&mut core, time);
        later = Some(core.solve(time, &jump.anchors(time, head_position, head_rotation)));
    }
    let later = later.unwrap();
    assert_eq!(later.estimator.standing_space_rebases, 1);
    assert_rigidly_moved(&before, &later, &jump);
    assert!(
        later
            .targets
            .iter()
            .all(|t| Vec3::from_array(t.linear_velocity).length() < 1e-3)
    );
}

#[test]
fn explicit_rebase_for_the_same_jump_is_not_detected_again() {
    let mut core = aligned();
    enable(&mut core, all_estimators());
    let head_position = Vec3::new(0.0, 1.65, 0.0);
    let head_rotation = Quat::from_rotation_y(HEADING);
    let mut before = None;
    for tick in 201..=220 {
        let time = tick as f64 / 100.0;
        ingest_still(&mut core, time);
        before = Some(core.solve(
            time,
            &Anchors {
                hmd: Some(hmd(time, head_position, head_rotation)),
                ..Default::default()
            },
        ));
    }
    let jump = Jump::seventy_degrees();
    // The application saw the transform change and rebased first.
    core.rebase_standing_space(70.0_f32.to_radians());
    ingest_still(&mut core, 2.21);
    let after = core.solve(2.21, &jump.anchors(2.21, head_position, head_rotation));
    assert_eq!(after.estimator.standing_space_rebases, 1);
    assert_eq!(after.estimator.tracking_discontinuities, 0);
    assert_rigidly_moved(&before.unwrap(), &after, &jump);
}

#[test]
fn fast_human_head_motion_is_not_a_tracking_jump() {
    let mut core = aligned();
    enable(&mut core, all_estimators());
    let yaw_rate = 300.0_f32.to_radians();
    let speed = Vec3::new(2.0, 0.0, 0.0);
    for tick in 201..=260 {
        let time = tick as f64 / 100.0;
        let elapsed = (time - 2.01) as f32;
        ingest_still(&mut core, time);
        // Reverse direction every 0.2 s (a continuous triangle wave), like a
        // quick look around while stepping.
        let cycle = elapsed % 0.4;
        let s = if cycle < 0.2 { cycle } else { 0.4 - cycle };
        let rotation = Quat::from_rotation_y(HEADING + yaw_rate * s);
        let position = Vec3::new(0.0, 1.65, 0.0) + speed * s;
        let frame = core.solve(
            time,
            &Anchors {
                hmd: Some(hmd(time, position, rotation)),
                ..Default::default()
            },
        );
        assert_eq!(frame.estimator.standing_space_rebases, 0, "at {time}");
        assert_eq!(frame.estimator.tracking_discontinuities, 0, "at {time}");
        assert!(all_finite(&frame));
    }
}

#[test]
fn jump_while_looking_straight_down_does_not_rebase_or_emit_nonfinite_values() {
    let mut core = aligned();
    enable(&mut core, all_estimators());
    let position = Vec3::new(0.0, 1.6, -0.1);
    let look = |down: f32| Quat::from_rotation_y(HEADING) * Quat::from_rotation_x(-down);
    // Tilt down to vertical over 0.5 s (180°/s), then hold.
    for tick in 201..=260 {
        let time = tick as f64 / 100.0;
        let down = (((time - 2.0) / 0.5).min(1.0) as f32) * std::f32::consts::FRAC_PI_2;
        ingest_still(&mut core, time);
        let frame = core.solve(
            time,
            &Anchors {
                hmd: Some(hmd(time, position, look(down))),
                ..Default::default()
            },
        );
        assert_eq!(frame.estimator.tracking_discontinuities, 0);
    }
    let jump = Jump::seventy_degrees();
    for tick in 261..=280 {
        let time = tick as f64 / 100.0;
        ingest_still(&mut core, time);
        let frame = core.solve(
            time,
            &jump.anchors(time, position, look(std::f32::consts::FRAC_PI_2)),
        );
        assert_eq!(frame.estimator.standing_space_rebases, 0);
        assert_eq!(frame.estimator.tracking_discontinuities, 1);
        assert!(frame.calibration.calibrated);
        assert!(all_finite(&frame));
        assert!(frame.targets.iter().all(|t| t.valid));
    }
}

#[test]
fn preview_calibration_never_rebases() {
    let mut core = TrackingCore::new(TrackingConfig {
        estimator: all_estimators(),
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
    feed_calibration(&mut core, 0..=100, false, false);
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    let jump = Jump::seventy_degrees();
    for tick in 101..=120 {
        let time = tick as f64 / 100.0;
        ingest_still(&mut core, time);
        let anchors = if tick < 110 {
            upright(time, 1.65)
        } else {
            jump.anchors(
                time,
                Vec3::new(0.0, 1.65, 0.0),
                Quat::from_rotation_y(HEADING),
            )
        };
        let frame = core.solve(time, &anchors);
        assert_eq!(frame.estimator.standing_space_rebases, 0);
        assert!(
            frame
                .targets
                .iter()
                .all(|t| !t.valid && t.linear_velocity == [0.0; 3])
        );
    }
}
