use glam::{Quat, Vec3};
use tracking_core::preview_motion::{
    PreviewMotionEstimator, PreviewMotionStatus, PreviewSupportFoot,
};
use tracking_core::*;

fn neutral_ankle_y(body: &Anthropometrics) -> f32 {
    body.standing_eye_height_m
        - body.head_to_neck_m
        - body.neck_to_chest_m
        - body.chest_to_hip_m
        - body.thigh_length_m
        - body.calf_length_m
}

fn frame(time: f64, left: Vec3, right: Vec3) -> TrackingFrame {
    TrackingFrame {
        at_secs: time,
        targets: vec![TargetPose {
            role: TargetRole::Waist,
            position: [0.0, 0.9, 0.0],
            orientation_xyzw: Quat::IDENTITY.to_array(),
            position_source: Evidence::Inferred,
            orientation_source: Evidence::MeasuredOrientation,
            confidence: 0.0,
            valid: false,
            reason: Some("Desktop preview".into()),
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
        }],
        joints: vec![
            JointPose {
                joint: JointRole::Head,
                position: [0.0, 1.65, 0.0],
                confidence: 0.0,
                valid: false,
            },
            JointPose {
                joint: JointRole::Hip,
                position: [0.0, 0.9, 0.0],
                confidence: 0.0,
                valid: false,
            },
            JointPose {
                joint: JointRole::LeftAnkle,
                position: left.to_array(),
                confidence: 0.0,
                valid: false,
            },
            JointPose {
                joint: JointRole::RightAnkle,
                position: right.to_array(),
                confidence: 0.0,
                valid: false,
            },
        ],
        bones: vec![Bone {
            start: JointRole::Hip,
            end: JointRole::LeftAnkle,
        }],
        segments: Role::CORE
            .into_iter()
            .map(|role| SegmentPose {
                role,
                orientation_xyzw: Quat::IDENTITY.to_array(),
                yaw_correction_degrees: 0.0,
                confidence: 0.8,
                valid: true,
                measurement_received_at_secs: Some(time),
                measurement_age_secs: Some(0.0),
                uses_firmware_hold: false,
            })
            .collect(),
        calibration: CalibrationStatus {
            calibrated: true,
            preview_only: true,
            requires_session_calibration: false,
            reason: None,
            used_firmware_hold: false,
            direction_calibrated: true,
            reference_forward: Some([0.0, 0.0, -1.0]),
            floor_mismatch_m: None,
        },
        recovery: RecoveryState::Idle,
        corrections: vec![],
        live_anchor: false,
        preview_only: true,
        preview_motion: PreviewMotionStatus::default(),
        estimator: Default::default(),
        motion_calibration: None,
    }
}

fn ankle_pair(body: &Anthropometrics) -> [Vec3; 2] {
    let y = neutral_ankle_y(body);
    [Vec3::new(-0.15, y, 0.0), Vec3::new(0.15, y, 0.0)]
}

fn apply(
    estimator: &mut PreviewMotionEstimator,
    body: &Anthropometrics,
    time: f64,
    ankles: [Vec3; 2],
) -> TrackingFrame {
    let mut next = frame(time, ankles[0], ankles[1]);
    estimator.apply(&mut next, true, body);
    next
}

fn translation(frame: &TrackingFrame) -> Vec3 {
    Vec3::from_array(frame.preview_motion.root_translation_m)
}

#[test]
fn planted_leg_infers_horizontal_displacement_without_changing_pose_validity_or_shape() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let mut ankles = ankle_pair(&body);
    apply(&mut estimator, &body, 0.0, ankles);
    for step in 1..=30 {
        ankles[0].z = step as f32 * 0.005;
        ankles[1].z = -(step as f32) * 0.008;
        ankles[1].y = neutral_ankle_y(&body) + 0.15;
        let output = apply(&mut estimator, &body, f64::from(step) * 0.02, ankles);
        assert!(
            (output.joints[2].position[2]).abs() < 1.0e-5,
            "assumed stance foot should remain planted"
        );
        assert!(output.targets.iter().all(|pose| !pose.valid));
        assert!(output.joints.iter().all(|pose| !pose.valid));
        assert_eq!(
            output.targets[0].orientation_xyzw,
            Quat::IDENTITY.to_array()
        );
        let offset = translation(&output);
        assert!(offset.x.abs() < 1.0e-6);
        assert!((offset.z + ankles[0].z).abs() < 1.0e-5);
        assert!(
            (Vec3::from_array(output.joints[0].position) - Vec3::new(0.0, 1.65, 0.0) - offset)
                .length()
                < 1.0e-6
        );
        assert!(
            (Vec3::from_array(output.targets[0].position) - Vec3::new(0.0, 0.9, 0.0) - offset)
                .length()
                < 1.0e-6
        );
        let input_length = Vec3::new(0.0, 0.9, 0.0).distance(ankles[0]);
        let output_length = Vec3::from_array(output.joints[1].position)
            .distance(Vec3::from_array(output.joints[2].position));
        assert!((input_length - output_length).abs() < 1.0e-6);
    }
}

#[test]
fn squat_height_uses_neutral_anatomy_even_when_first_ready_frame_is_bent() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let mut ankles = ankle_pair(&body);
    ankles.iter_mut().for_each(|foot| foot.y += 0.30);
    let initial = apply(&mut estimator, &body, 0.0, ankles);
    assert_eq!(translation(&initial), Vec3::ZERO);
    let mut bent = initial;
    for step in 1..=30 {
        bent = apply(&mut estimator, &body, f64::from(step) * 0.02, ankles);
    }
    assert!((translation(&bent).y + 0.30).abs() < 1.0e-5);
    assert!((bent.joints[0].position[1] - 1.35).abs() < 1.0e-5);
    for step in 31..=60 {
        let amount = (60 - step) as f32 / 29.0;
        ankles = ankle_pair(&body);
        ankles.iter_mut().for_each(|foot| foot.y += 0.30 * amount);
        bent = apply(&mut estimator, &body, f64::from(step) * 0.02, ankles);
    }
    assert!(translation(&bent).y.abs() < 1.0e-5);
}

#[test]
fn support_switch_has_dwell_and_does_not_jump_when_swing_foot_lands() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let mut ankles = ankle_pair(&body);
    apply(&mut estimator, &body, 0.0, ankles);
    // Lift the right foot first, preserving the left support hypothesis.
    for step in 1..=10 {
        ankles[1].y += 0.015;
        apply(&mut estimator, &body, f64::from(step) * 0.02, ankles);
    }
    // Land the right while lifting left. Horizontal support stays put until
    // the sustained height difference has passed the switch dwell.
    let mut previous = Vec3::ZERO;
    let mut output = frame(0.0, ankles[0], ankles[1]);
    for step in 11..=30 {
        ankles[0].y = neutral_ankle_y(&body) + ((step - 10) as f32 * 0.015).min(0.15);
        ankles[1].y = neutral_ankle_y(&body) + (0.15 - (step - 10) as f32 * 0.015).max(0.0);
        ankles[1].z = -0.20;
        output = apply(&mut estimator, &body, f64::from(step) * 0.02, ankles);
        let now = translation(&output);
        assert!(
            (now.x - previous.x).abs() < 1.0e-6 && (now.z - previous.z).abs() < 1.0e-6,
            "support handoff itself should not translate root"
        );
        previous = now;
    }
    assert_eq!(
        output.preview_motion.support,
        Some(PreviewSupportFoot::Right)
    );
    ankles[1].z += 0.01;
    let stepped = apply(&mut estimator, &body, 0.62, ankles);
    assert!((translation(&stepped).z + 0.01).abs() < 1.0e-5);
}

#[test]
fn stale_held_invalid_and_gapped_inputs_release_support_and_do_not_integrate() {
    let body = Anthropometrics::default();
    for rejection in 0..4 {
        let mut estimator = PreviewMotionEstimator::default();
        let mut ankles = ankle_pair(&body);
        apply(&mut estimator, &body, 1.0, ankles);
        ankles[0].z = 0.02;
        let moving = apply(&mut estimator, &body, 1.02, ankles);
        let retained = translation(&moving);
        let time = if rejection == 3 { 3.0 } else { 1.04 };
        let mut interrupted = frame(time, ankles[0] + Vec3::Z, ankles[1]);
        match rejection {
            0 => interrupted.segments[0].measurement_received_at_secs = Some(0.0),
            1 => interrupted.segments[0].uses_firmware_hold = true,
            2 => interrupted.segments[0].valid = false,
            _ => {}
        }
        estimator.apply(&mut interrupted, true, &body);
        assert!(interrupted.preview_motion.frozen);
        assert_eq!(interrupted.preview_motion.support, None);
        assert_eq!(translation(&interrupted), retained);
        let resumed = apply(&mut estimator, &body, time + 0.02, ankles);
        assert!(!resumed.preview_motion.frozen);
        assert_eq!(
            translation(&resumed),
            retained,
            "resume must rebase rather than catch up a lost stance"
        );
    }
}

#[test]
fn repeated_reversed_or_nonfinite_time_cannot_move_root() {
    let body = Anthropometrics::default();
    for time in [1.0, 0.5, f64::NAN, f64::INFINITY, -1.0] {
        let mut estimator = PreviewMotionEstimator::default();
        let ankles = ankle_pair(&body);
        apply(&mut estimator, &body, 1.0, ankles);
        let mut changed = frame(
            time,
            ankles[0] + Vec3::new(0.2, 0.2, 0.2),
            ankles[1] + Vec3::Y * 0.2,
        );
        estimator.apply(&mut changed, true, &body);
        assert!(changed.preview_motion.frozen);
        assert_eq!(translation(&changed), Vec3::ZERO);
    }
}

#[test]
fn rates_are_bounded_and_a_common_pose_jump_cannot_relocate_the_floor() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let mut ankles = ankle_pair(&body);
    apply(&mut estimator, &body, 0.0, ankles);
    ankles[0].z += 0.12;
    ankles[1].y += 0.05;
    let bounded = apply(&mut estimator, &body, 0.02, ankles);
    assert!(translation(&bounded).z.abs() <= 1.6 * 0.02 + 1.0e-6);
    let retained = translation(&bounded);
    ankles
        .iter_mut()
        .for_each(|foot| *foot += Vec3::new(0.5, 0.5, 0.5));
    let jumped = apply(&mut estimator, &body, 0.04, ankles);
    assert!(jumped.preview_motion.frozen);
    assert_eq!(translation(&jumped), retained);
    let mut bent = ankle_pair(&body);
    bent.iter_mut().for_each(|foot| foot.y += 0.3);
    let next = apply(&mut estimator, &body, 0.06, bent);
    assert!((translation(&next).y - retained.y).abs() <= 1.2 * 0.02 + 1.0e-6);
    // A 200ms frame advances at most the configured 50ms integration step.
    let delayed = apply(&mut estimator, &body, 0.26, bent);
    assert!((translation(&delayed).y - translation(&next).y).abs() <= 1.2 * 0.05 + 1.0e-6);
}

#[test]
fn uniform_body_turn_does_not_invent_a_translation_pivot() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let ankles = ankle_pair(&body);
    apply(&mut estimator, &body, 0.0, ankles);
    for step_degrees in [2.0_f32, 0.01_f32] {
        estimator.reset();
        apply(&mut estimator, &body, 0.0, ankles);
        for step in 1..=450 {
            let rotation = Quat::from_rotation_y(step as f32 * step_degrees.to_radians());
            let mut turned = frame(
                f64::from(step) * 0.02,
                rotation * ankles[0],
                rotation * ankles[1],
            );
            for segment in &mut turned.segments {
                segment.orientation_xyzw = rotation.to_array();
            }
            estimator.apply(&mut turned, true, &body);
            assert!(
                translation(&turned).length() < 1.0e-5,
                "slow uniform turns must not invent a translation pivot"
            );
            assert!(turned.preview_motion.reason.contains("uniform turn"));
        }
    }
}

#[test]
fn ordinary_leg_flex_with_tiny_common_yaw_still_infers_stance_displacement() {
    let body = Anthropometrics::default();
    let mut estimator = PreviewMotionEstimator::default();
    let mut ankles = ankle_pair(&body);
    apply(&mut estimator, &body, 0.0, ankles);
    let mut output = frame(0.0, ankles[0], ankles[1]);
    for step in 1..=30 {
        ankles[0].z = step as f32 * 0.005;
        ankles[1].y = neutral_ankle_y(&body) + 0.15;
        let yaw = Quat::from_rotation_y((step as f32 * 0.01).to_radians());
        let flex = (step as f32 * 0.3).to_radians();
        output = frame(f64::from(step) * 0.02, ankles[0], ankles[1]);
        for segment in &mut output.segments {
            segment.orientation_xyzw = (yaw
                * match segment.role {
                    Role::LeftThigh | Role::RightThigh => Quat::from_rotation_x(flex),
                    Role::LeftCalf | Role::RightCalf => Quat::from_rotation_x(-flex),
                    _ => Quat::IDENTITY,
                })
            .to_array();
        }
        estimator.apply(&mut output, true, &body);
        assert!(!output.preview_motion.reason.contains("uniform turn"));
    }
    assert!((translation(&output).z + 0.15).abs() < 1.0e-5);
}

#[test]
fn reset_toggle_readiness_and_live_anchor_are_strict_mode_boundaries() {
    let body = Anthropometrics::default();
    for boundary in 0..5 {
        let mut estimator = PreviewMotionEstimator::default();
        let mut ankles = ankle_pair(&body);
        apply(&mut estimator, &body, 0.0, ankles);
        ankles[0].z += 0.02;
        let moved = apply(&mut estimator, &body, 0.02, ankles);
        assert!(translation(&moved).length() > 0.01);
        let mut next = frame(0.04, ankles[0], ankles[1]);
        if boundary == 0 {
            estimator.reset();
        }
        if boundary == 2 {
            next.calibration.direction_calibrated = false;
        }
        if boundary == 3 {
            next.live_anchor = true;
            next.preview_only = false;
        }
        if boundary == 4 {
            next.calibration.preview_only = false;
        }
        let unchanged = next
            .joints
            .iter()
            .map(|joint| joint.position)
            .collect::<Vec<_>>();
        estimator.apply(&mut next, boundary != 1, &body);
        assert_eq!(translation(&next), Vec3::ZERO);
        assert_eq!(
            unchanged,
            next.joints
                .iter()
                .map(|joint| joint.position)
                .collect::<Vec<_>>()
        );
    }
}

fn feed(core: &mut TrackingCore, time: f64, angle: f32) {
    for role in Role::CORE {
        let rotation = match role {
            Role::LeftThigh | Role::RightThigh => Quat::from_rotation_x(angle),
            Role::LeftCalf | Role::RightCalf => Quat::from_rotation_x(-angle),
            _ => Quat::IDENTITY,
        };
        core.ingest(SensorSample {
            id: format!("preview-{role:?}"),
            orientation_xyzw: rotation.to_array(),
            received_at_secs: time,
            acceleration_mps2: Some([0.0; 3]),
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
}

fn desktop_core() -> TrackingCore {
    let mut core = TrackingCore::new(TrackingConfig {
        enable_preview_motion: true,
        ..Default::default()
    })
    .unwrap();
    core.set_assignments(
        Role::CORE
            .iter()
            .map(|role| RoleAssignment {
                role: *role,
                sensor_id: format!("preview-{role:?}"),
            })
            .collect(),
    )
    .unwrap();
    for step in 0..=100 {
        feed(&mut core, f64::from(step) / 100.0, 0.0);
    }
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    core
}

#[test]
fn core_preview_squat_moves_head_but_keeps_all_live_targets_invalid_and_resets_on_recalibration() {
    let mut core = desktop_core();
    let first = core.solve(1.0, &Anchors::default());
    assert!(first.preview_motion.active);
    for step in 1..=60 {
        let time = 1.0 + f64::from(step) * 0.02;
        feed(&mut core, time, (step as f32 / 60.0) * 0.8);
        let output = core.solve(time, &Anchors::default());
        assert!(output.targets.iter().all(|target| !target.valid));
        assert!(output.preview_only && !output.live_anchor);
        assert!(
            output
                .joints
                .iter()
                .all(|joint| joint.position.iter().all(|value| value.is_finite()))
        );
    }
    let crouched = core.solve(2.21, &Anchors::default());
    assert!(
        translation(&crouched).y < -0.20,
        "{:?}",
        translation(&crouched)
    );
    let head = crouched
        .joints
        .iter()
        .find(|joint| joint.joint == JointRole::Head)
        .unwrap();
    assert!(head.position[1] < 1.45);
    for step in 221..=400 {
        feed(&mut core, f64::from(step) / 100.0, 0.0);
    }
    core.calibrate_neutral(4.0, &Anchors::default(), true)
        .unwrap();
    let reset = core.solve(4.0, &Anchors::default());
    assert_eq!(translation(&reset), Vec3::ZERO);
    assert_eq!(reset.joints[0].position, [0.0, 1.65, 0.0]);
}

#[test]
fn core_config_and_manual_mounting_changes_clear_motion_origin() {
    let mut core = desktop_core();
    core.solve(1.0, &Anchors::default());
    for step in 1..=30 {
        let time = 1.0 + f64::from(step) * 0.02;
        feed(&mut core, time, step as f32 * 0.02);
        core.solve(time, &Anchors::default());
    }
    assert!(translation(&core.solve(1.61, &Anchors::default())).y < -0.1);
    let mut config = core.config().clone();
    config.enable_preview_motion = false;
    core.set_config(config).unwrap();
    let disabled = core.solve(1.62, &Anchors::default());
    assert!(!disabled.preview_motion.enabled);
    assert_eq!(translation(&disabled), Vec3::ZERO);
    let mut config = core.config().clone();
    config.enable_preview_motion = true;
    core.set_config(config).unwrap();
    core.solve(1.63, &Anchors::default());
    core.set_manual_mount(
        Role::Hip,
        tracking_core::manual_mounting::ManualMountValues {
            physical_degrees_xyz: [5.0, 0.0, 0.0],
            ..Default::default()
        },
    )
    .unwrap();
    let adjusted = core.solve(1.64, &Anchors::default());
    assert!(!adjusted.preview_motion.active);
    assert_eq!(translation(&adjusted), Vec3::ZERO);
}
