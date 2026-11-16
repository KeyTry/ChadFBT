use glam::{Quat, Vec3};
use tracking_core::forward_calibration::{ski_pitch_sign, solve_forward_alignment};
use tracking_core::*;

fn index(role: Role) -> usize {
    Role::ALL.iter().position(|r| *r == role).unwrap()
}
fn id(role: Role) -> String {
    format!("functional-{role:?}")
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
fn raw(role: Role, body: Quat, bias_change: f32) -> Quat {
    Quat::from_rotation_y(bias(role) + bias_change) * body * mount(role)
}
fn anchors(time: f64, height: f32, yaw: f32) -> Anchors {
    Anchors {
        hmd: Some(AnchorPose {
            position: [0.0, height, 0.0],
            orientation_xyzw: Quat::from_rotation_y(yaw).to_array(),
            received_at_secs: time,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        }),
        ..Default::default()
    }
}
fn feed_pose(
    core: &mut TrackingCore,
    ticks: std::ops::RangeInclusive<usize>,
    body_heading: f32,
    ski: bool,
    weak_hip: bool,
    bias_change: f32,
    head_height: f32,
) {
    for tick in ticks {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            let bend = if ski && !(weak_hip && role == Role::Hip) {
                pitch(role)
            } else {
                0.0
            };
            let body = Quat::from_rotation_y(body_heading) * Quat::from_rotation_x(bend);
            core.ingest(SensorSample {
                id: id(role),
                orientation_xyzw: raw(role, body, bias_change).to_array(),
                received_at_secs: time,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(time, &anchors(time, head_height, body_heading));
    }
}
fn neutral(preview: bool) -> TrackingCore {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
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
    feed_pose(&mut core, 0..=100, 0.4, false, false, 0.0, 1.65);
    core.calibrate_neutral(1.0, &anchors(1.0, 1.65, 0.4), preview)
        .unwrap();
    core
}
fn aligned(preview: bool) -> TrackingCore {
    let mut core = neutral(preview);
    feed_pose(&mut core, 101..=200, 0.4, true, false, 0.0, 1.1);
    core.calibrate_forward(2.0).unwrap();
    core
}
fn near(actual: Quat, expected: Quat) {
    assert!(
        actual.dot(expected).abs() > 0.99999,
        "actual {actual:?}, expected {expected:?}"
    );
}

#[test]
fn ninety_degree_sensor_yaw_no_longer_turns_a_forward_bend_sideways() {
    let q0 = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let q1 = q0 * Quat::from_rotation_x(-0.5);
    let old = q1 * q0.conjugate();
    assert!((old * Vec3::Y).distance(Quat::from_rotation_x(-0.5) * Vec3::Y) > 0.6);
    let result = solve_forward_alignment(Role::Chest, q0.to_array(), q1.to_array(), 0.0).unwrap();
    let corrected = Quat::from_rotation_y(result.heading_alignment_radians)
        * q1
        * Quat::from_array(result.mounting_xyzw);
    near(corrected, Quat::from_rotation_x(-0.5));
}

#[test]
fn independent_yaw_and_arbitrary_mounts_resolve_for_all_six_signed_roles() {
    for role in Role::CORE {
        for heading in [0.0_f32, 0.8, -2.4] {
            let q0 = raw(role, Quat::from_rotation_y(0.4), 0.0);
            let q1 = raw(
                role,
                Quat::from_rotation_y(0.4) * Quat::from_rotation_x(pitch(role)),
                0.0,
            );
            let result =
                solve_forward_alignment(role, q0.to_array(), (-q1).to_array(), heading).unwrap();
            let a = Quat::from_rotation_y(result.heading_alignment_radians);
            let m = Quat::from_array(result.mounting_xyzw);
            near(a * q0 * m, Quat::from_rotation_y(heading));
            near(
                a * q1 * m,
                Quat::from_rotation_y(heading) * Quat::from_rotation_x(pitch(role)),
            );
        }
    }
}

#[test]
fn weak_twisting_and_strongly_tilted_rotation_axes_are_rejected() {
    for (q1, expected) in [
        (Quat::from_rotation_x(-0.05), "changed only"),
        (Quat::from_rotation_y(0.4), "turning/twisting"),
        (
            Quat::from_axis_angle(Vec3::new(-1.0, 0.8, 0.0).normalize(), 0.6),
            "turning/twisting",
        ),
        (Quat::from_rotation_x(-1.6), "moderate"),
    ] {
        let error =
            solve_forward_alignment(Role::Chest, Quat::IDENTITY.to_array(), q1.to_array(), 0.0)
                .unwrap_err();
        assert!(error.0.contains(expected), "{error}");
    }
    assert!(
        solve_forward_alignment(
            Role::LeftFoot,
            Quat::IDENTITY.to_array(),
            Quat::from_rotation_x(0.4).to_array(),
            0.0
        )
        .is_err()
    );
}

#[test]
fn moderate_axis_tilt_and_residual_are_accepted_but_weak_horizontal_bend_is_not() {
    // A representative capture near the observed hip axis tilt and residual.
    let axis = Vec3::new(
        -19.6_f32.to_radians().cos(),
        19.6_f32.to_radians().sin(),
        0.0,
    );
    let q = Quat::from_axis_angle(axis, 23.2_f32.to_radians());
    let result =
        solve_forward_alignment(Role::Hip, Quat::IDENTITY.to_array(), q.to_array(), 0.0).unwrap();
    assert!((result.axis_vertical_degrees - 19.6).abs() < 0.01);
    assert!(result.model_residual_degrees > 7.5 && result.model_residual_degrees < 8.2);

    // Total rotation exceeds 10°, but its horizontal component does not.
    let axis = Vec3::new(
        -24.0_f32.to_radians().cos(),
        24.0_f32.to_radians().sin(),
        0.0,
    );
    let weak = Quat::from_axis_angle(axis, 10.5_f32.to_radians());
    let error = solve_forward_alignment(Role::Hip, Quat::IDENTITY.to_array(), weak.to_array(), 0.0)
        .unwrap_err();
    assert!(error.0.contains("more forward/backward bend"), "{error}");

    // Axis tilt alone is allowed, but excessive model residual still rejects.
    let residual = Quat::from_axis_angle(axis, 30.0_f32.to_radians());
    let error = solve_forward_alignment(
        Role::Hip,
        Quat::IDENTITY.to_array(),
        residual.to_array(),
        0.0,
    )
    .unwrap_err();
    assert!(error.0.contains("residual"), "{error}");
}

#[test]
fn modest_combined_yaw_and_pitch_has_bounded_model_conditional_heading_error() {
    // This bound describes this synthetic pose model only, not real-world accuracy:
    // 6° of actual yaw followed by a 28° pitch, with no side bend or sensor noise.
    let true_raw_heading = 0.4_f32;
    let neutral = Quat::from_rotation_y(true_raw_heading);
    let ski = neutral
        * Quat::from_rotation_y(6.0_f32.to_radians())
        * Quat::from_rotation_x(-28.0_f32.to_radians());
    let result =
        solve_forward_alignment(Role::Chest, neutral.to_array(), ski.to_array(), 0.0).unwrap();
    let error_degrees = (result.heading_alignment_radians + true_raw_heading)
        .abs()
        .to_degrees();
    assert!(
        error_degrees > 2.8 && error_degrees < 3.2,
        "heading error {error_degrees}°"
    );
}

#[test]
fn forward_capture_allows_small_sway_but_rejects_larger_turning_motion() {
    fn feed_sway(core: &mut TrackingCore, turn: bool) {
        for tick in 101..=200 {
            let time = tick as f64 / 100.0;
            let progress = ((tick as f32 - 135.0) / 65.0).clamp(0.0, 1.0);
            for role in Role::CORE {
                let sway = if role == Role::Chest { progress } else { 0.0 };
                let extra = if turn {
                    Quat::from_rotation_y(sway * 5.5_f32.to_radians())
                } else {
                    Quat::from_rotation_x(sway * 3.0_f32.to_radians())
                };
                let body = Quat::from_rotation_y(0.4) * extra * Quat::from_rotation_x(pitch(role));
                core.ingest(SensorSample {
                    id: id(role),
                    orientation_xyzw: raw(role, body, 0.0).to_array(),
                    received_at_secs: time,
                    acceleration_mps2: None,
                    provenance: SensorProvenance::Measured,
                })
                .unwrap();
            }
            core.solve(time, &Anchors::default());
        }
    }
    let mut small_sway = neutral(true);
    feed_sway(&mut small_sway, false);
    small_sway.calibrate_forward(2.0).unwrap();
    assert!(small_sway.direction_calibrated());
    let neutral_error = small_sway
        .calibrate_neutral(2.0, &Anchors::default(), true)
        .unwrap_err();
    assert!(
        neutral_error.0.contains("Chest is moving"),
        "{neutral_error}"
    );

    let mut turning = neutral(true);
    feed_sway(&mut turning, true);
    let error = turning.calibrate_forward(2.0).unwrap_err();
    assert!(error.0.contains("Chest is moving"), "{error}");
    assert!(!turning.direction_calibrated());
}

#[test]
fn lateral_motion_remains_an_explicit_pose_assertion_counterexample() {
    // With an unknown IMU yaw, a pure side bend is indistinguishable from a
    // sagittal bend in another heading frame. Gates must not claim to detect it.
    let result = solve_forward_alignment(
        Role::Chest,
        Quat::IDENTITY.to_array(),
        Quat::from_rotation_z(-0.4).to_array(),
        0.0,
    )
    .unwrap();
    assert!((result.heading_alignment_radians.abs().to_degrees() - 90.0).abs() < 0.01);
}

#[test]
fn functional_stage_is_transactional_and_neutral_only_cannot_publish_or_recover() {
    let mut core = neutral(false);
    assert!(!core.direction_calibrated());
    let provisional = core.solve(1.0, &anchors(1.0, 1.65, 0.4));
    assert!(provisional.targets.iter().all(|target| !target.valid));
    assert!(provisional.calibration.requires_session_calibration);
    core.request_recovery(1.0);
    feed_pose(&mut core, 101..=150, 0.4, false, false, 0.0, 1.65);
    assert!(matches!(
        core.solve(1.5, &anchors(1.5, 1.65, 0.4)).recovery,
        RecoveryState::Rejected { .. }
    ));
    feed_pose(&mut core, 151..=250, 0.4, true, true, 0.0, 1.1);
    let before = serde_json::to_string(&core.export_profile()).unwrap();
    assert!(core.calibrate_forward(2.5).unwrap_err().0.contains("Hip"));
    assert_eq!(
        serde_json::to_string(&core.export_profile()).unwrap(),
        before
    );
    assert!(!core.direction_calibrated());
    feed_pose(&mut core, 251..=350, 0.4, true, false, 0.0, 1.1);
    core.calibrate_forward(3.5).unwrap();
    assert!(core.direction_calibrated());
}

#[test]
fn solved_axes_generalize_beyond_ski_and_do_not_enter_dynamic_yaw_diagnostics() {
    let mut core = aligned(true);
    for role in Role::CORE {
        let local = Quat::from_rotation_y(0.3)
            * Quat::from_rotation_z(-0.2)
            * Quat::from_rotation_x(pitch(role) * 0.6);
        let body = Quat::from_rotation_y(0.4) * local;
        core.ingest(SensorSample {
            id: id(role),
            orientation_xyzw: raw(role, body, 0.0).to_array(),
            received_at_secs: 2.01,
            acceleration_mps2: None,
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
    let frame = core.solve(2.01, &Anchors::default());
    assert!(frame.calibration.direction_calibrated);
    for segment in &frame.segments {
        if Role::CORE.contains(&segment.role) {
            let expected = Quat::from_rotation_y(0.3)
                * Quat::from_rotation_z(-0.2)
                * Quat::from_rotation_x(pitch(segment.role) * 0.6);
            near(Quat::from_array(segment.orientation_xyzw), expected);
            assert_eq!(segment.yaw_correction_degrees, 0.0);
        }
    }
    assert!(frame.targets.iter().all(|target| !target.valid));
}

#[test]
fn neutral_repeat_keeps_physical_mount_and_only_realigns_world_heading() {
    let mut core = aligned(true);
    let physical = serde_json::to_string(&core.export_profile().mounting).unwrap();
    feed_pose(&mut core, 201..=300, 1.2, false, false, 0.0, 1.65);
    core.calibrate_neutral(3.0, &Anchors::default(), true)
        .unwrap();
    assert!(core.direction_calibrated());
    assert_eq!(
        serde_json::to_string(&core.export_profile().mounting).unwrap(),
        physical
    );
    feed_pose(&mut core, 301..=400, 1.2, true, false, 0.0, 1.1);
    let frame = core.solve(4.0, &Anchors::default());
    for segment in &frame.segments {
        if Role::CORE.contains(&segment.role) {
            near(
                Quat::from_array(segment.orientation_xyzw),
                Quat::from_rotation_x(pitch(segment.role)),
            );
        }
    }
}

#[test]
fn profile_restore_legacy_and_reboot_require_new_two_pose_alignment() {
    let mut core = aligned(true);
    let profile = core.export_profile();
    assert_eq!(profile.schema_version, 2);
    assert_eq!(profile.direction_calibrated_roles.len(), 6);
    for schema in [1, 2] {
        let mut restored = TrackingCore::new(TrackingConfig::default()).unwrap();
        let mut saved = profile.clone();
        saved.schema_version = schema;
        restored.restore_profile(saved).unwrap();
        assert!(!restored.direction_calibrated());
        assert!(restored.calibrate_forward(1.0).is_err());
        feed_pose(&mut restored, 0..=100, 0.4, false, false, 1.1, 1.65);
        restored
            .calibrate_neutral(1.0, &Anchors::default(), true)
            .unwrap();
        assert!(!restored.direction_calibrated());
        assert!(
            restored
                .export_profile()
                .direction_calibrated_roles
                .is_empty()
        );
        feed_pose(&mut restored, 101..=200, 0.4, true, false, 1.1, 1.1);
        restored.calibrate_forward(2.0).unwrap();
        assert!(restored.direction_calibrated());
    }
    core.clear_calibration();
    assert!(!core.direction_calibrated());
    assert!(core.calibration_forward().is_none());
    assert!(core.calibrate_forward(2.1).is_err());
}

#[test]
fn stale_expired_and_failed_recalibration_retain_previous_valid_factorization() {
    let mut core = aligned(false);
    let previous = serde_json::to_string(&core.export_profile()).unwrap();
    assert!(core.calibrate_forward(3.0).is_err());
    assert!(
        core.calibrate_forward(302.0)
            .unwrap_err()
            .0
            .contains("expired")
    );
    assert!(core.calibrate_forward(f64::NAN).is_err());
    assert!(core.direction_calibrated());
    assert_eq!(
        serde_json::to_string(&core.export_profile()).unwrap(),
        previous
    );
}

#[test]
fn near_threshold_samples_with_inconsistent_forward_axes_are_not_hidden_by_averaging() {
    let mut core = neutral(true);
    for tick in 101..=200 {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            let bend = if role == Role::Chest {
                -10.5_f32.to_radians()
            } else {
                pitch(role)
            };
            let deviation = if role == Role::Chest && tick == 200 {
                9.5_f32.to_radians()
            } else {
                0.0
            };
            let body = Quat::from_rotation_y(0.4)
                * Quat::from_rotation_y(deviation)
                * Quat::from_rotation_x(bend)
                * Quat::from_rotation_y(-deviation);
            core.ingest(SensorSample {
                id: id(role),
                orientation_xyzw: raw(role, body, 0.0).to_array(),
                received_at_secs: time,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(time, &Anchors::default());
    }
    let error = core.calibrate_forward(2.0).unwrap_err();
    assert!(error.0.contains("vary by more than 8°"), "{error}");
    assert!(!core.direction_calibrated());
}

fn segment_rotation(frame: &TrackingFrame, role: Role) -> Quat {
    Quat::from_array(
        frame
            .segments
            .iter()
            .find(|s| s.role == role)
            .unwrap()
            .orientation_xyzw,
    )
}

#[test]
fn recovery_after_real_forward_calibration_removes_common_drift_and_undoes_exactly() {
    let mut core = aligned(false);
    assert!(core.direction_calibrated());
    let mounts = serde_json::to_string(&core.export_profile()).unwrap();
    // Every sensor heading origin drifts by the same 12° (common mode).
    let drift = 12.0_f32.to_radians();
    feed_pose(&mut core, 201..=300, 0.4, false, false, drift, 1.65);
    let drifted = core.solve(3.0, &anchors(3.0, 1.65, 0.4));
    for role in Role::CORE {
        near(
            segment_rotation(&drifted, role),
            Quat::from_rotation_y(0.4 + drift),
        );
    }
    core.request_recovery(3.0);
    feed_pose(&mut core, 301..=350, 0.4, false, false, drift, 1.65);
    let recovered = core.solve(3.5, &anchors(3.5, 1.65, 0.4));
    assert!(matches!(recovered.recovery, RecoveryState::Applied { .. }));
    assert!(recovered.targets.iter().all(|t| t.valid));
    for role in Role::CORE {
        near(
            segment_rotation(&recovered, role),
            Quat::from_rotation_y(0.4),
        );
    }
    // Recovery changes yaw only: physical mounts and alignment are untouched,
    // so a bend still maps onto the bone's own sagittal axis.
    assert_eq!(
        serde_json::to_string(&core.export_profile()).unwrap(),
        mounts
    );
    feed_pose(&mut core, 351..=352, 0.4, true, false, drift, 1.1);
    let bent = core.solve(3.52, &anchors(3.52, 1.1, 0.4));
    for role in Role::CORE {
        near(
            segment_rotation(&bent, role),
            Quat::from_rotation_y(0.4) * Quat::from_rotation_x(pitch(role)),
        );
    }
    // The neutral capture remains usable for manual edits, but a new skiing
    // solve needs a new neutral after recovery.
    assert!(core.calibrate_forward(3.52).is_err());
    assert!(core.undo_recovery());
    let undone = core.solve(3.52, &anchors(3.52, 1.1, 0.4));
    for role in Role::CORE {
        near(
            segment_rotation(&undone, role),
            Quat::from_rotation_y(0.4 + drift) * Quat::from_rotation_x(pitch(role)),
        );
        let yaw = undone
            .segments
            .iter()
            .find(|s| s.role == role)
            .unwrap()
            .yaw_correction_degrees;
        assert!(yaw.abs() < 1e-3, "{role:?} {yaw}");
    }
}

#[test]
fn rebasing_standing_space_rotates_every_output_about_the_origin() {
    let mut core = aligned(false);
    // An arbitrary pose with non-identity mounts and nonzero heading alignment.
    for role in Role::CORE {
        let local = Quat::from_rotation_y(0.25)
            * Quat::from_rotation_z(-0.15)
            * Quat::from_rotation_x(pitch(role) * 0.7);
        core.ingest(SensorSample {
            id: id(role),
            orientation_xyzw: raw(role, Quat::from_rotation_y(0.4) * local, 0.0).to_array(),
            received_at_secs: 2.01,
            acceleration_mps2: None,
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
    let head = Quat::from_rotation_y(0.5) * Quat::from_rotation_x(-0.3);
    let pose = |rotation: Quat| {
        let pose = |position: Vec3, orientation: Quat| AnchorPose {
            position: (rotation * position).to_array(),
            orientation_xyzw: (rotation * orientation).to_array(),
            received_at_secs: 2.01,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        };
        Anchors {
            hmd: Some(pose(Vec3::new(0.7, 1.55, -1.2), head)),
            left_hand: Some(pose(Vec3::new(0.4, 1.1, -1.5), Quat::IDENTITY)),
            right_hand: Some(pose(Vec3::new(1.0, 1.0, -1.4), Quat::IDENTITY)),
        }
    };
    let before = core.solve(2.01, &pose(Quat::IDENTITY));
    assert!(before.targets.iter().all(|t| t.valid));
    let forward_before = Vec3::from_array(core.calibration_forward().unwrap());
    let mounts = serde_json::to_string(&core.export_profile()).unwrap();
    let turn = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    core.rebase_standing_space(std::f32::consts::FRAC_PI_2);
    let after = core.solve(2.01, &pose(turn));
    assert!(core.direction_calibrated());
    assert_eq!(
        serde_json::to_string(&core.export_profile()).unwrap(),
        mounts
    );
    assert!(
        (Vec3::from_array(core.calibration_forward().unwrap()) - turn * forward_before).length()
            < 1e-5
    );
    for (expected, actual) in before.targets.iter().zip(&after.targets) {
        assert_eq!(expected.role, actual.role);
        assert!(actual.valid);
        let rotated = turn * Vec3::from_array(expected.position);
        assert!(
            (Vec3::from_array(actual.position) - rotated).length() < 1e-5,
            "{:?}",
            expected.role
        );
        near(
            Quat::from_array(actual.orientation_xyzw),
            turn * Quat::from_array(expected.orientation_xyzw),
        );
    }
    for (expected, actual) in before.joints.iter().zip(&after.joints) {
        let rotated = turn * Vec3::from_array(expected.position);
        assert!(
            (Vec3::from_array(actual.position) - rotated).length() < 1e-5,
            "{:?}",
            expected.joint
        );
    }
    for (expected, actual) in before.segments.iter().zip(&after.segments) {
        assert_eq!(
            expected.yaw_correction_degrees,
            actual.yaw_correction_degrees
        );
    }
    // Nonfinite deltas are ignored.
    core.rebase_standing_space(f32::NAN);
    let unchanged = core.solve(2.01, &pose(turn));
    for (a, b) in after.joints.iter().zip(&unchanged.joints) {
        assert_eq!(a.position, b.position);
    }
}

#[test]
fn uncertified_optional_roles_use_fallback_but_certified_stale_roles_invalidate() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(
        Role::ALL
            .iter()
            .map(|role| RoleAssignment {
                sensor_id: id(*role),
                role: *role,
            })
            .collect(),
    )
    .unwrap();
    for tick in 0..=100 {
        let time = tick as f64 / 100.0;
        for role in Role::ALL {
            core.ingest(SensorSample {
                id: id(role),
                orientation_xyzw: Quat::IDENTITY.to_array(),
                received_at_secs: time,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(time, &anchors(time, 1.65, 0.0));
    }
    core.calibrate_neutral(1.0, &anchors(1.0, 1.65, 0.0), false)
        .unwrap();
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    let frame = core.solve(1.0, &anchors(1.0, 1.65, 0.0));
    let foot = frame
        .targets
        .iter()
        .find(|t| t.role == TargetRole::LeftFoot)
        .unwrap();
    assert!(foot.valid);
    assert_eq!(foot.orientation_source, Evidence::Inferred);
    assert!(
        !frame
            .targets
            .iter()
            .any(|t| t.role == TargetRole::LeftElbow)
    );
    core.confirm_manual_mount(Role::LeftFoot).unwrap();
    core.confirm_manual_mount(Role::LeftUpperArm).unwrap();
    for role in Role::CORE {
        core.ingest(SensorSample {
            id: id(role),
            orientation_xyzw: Quat::IDENTITY.to_array(),
            received_at_secs: 1.3,
            acceleration_mps2: None,
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
    let stale = core.solve(1.3, &anchors(1.3, 1.65, 0.0));
    assert!(
        !stale
            .targets
            .iter()
            .find(|t| t.role == TargetRole::LeftFoot)
            .unwrap()
            .valid
    );
    assert!(
        !stale
            .targets
            .iter()
            .find(|t| t.role == TargetRole::LeftElbow)
            .unwrap()
            .valid
    );
}
