use glam::{Quat, Vec3};
use tracking_core::*;

fn sensor_id(role: Role) -> String {
    format!("sensor-{role:?}")
}
fn assignments(roles: &[Role]) -> Vec<RoleAssignment> {
    roles
        .iter()
        .map(|role| RoleAssignment {
            sensor_id: sensor_id(*role),
            role: *role,
        })
        .collect()
}
fn anchor(time: f64, rotation: Quat) -> Anchors {
    Anchors {
        hmd: Some(AnchorPose {
            position: [0.0, 1.65, 0.0],
            orientation_xyzw: rotation.to_array(),
            received_at_secs: time,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        }),
        left_hand: None,
        right_hand: None,
    }
}
fn sample(role: Role, time: f64, rotation: Quat) -> SensorSample {
    SensorSample {
        id: sensor_id(role),
        orientation_xyzw: rotation.to_array(),
        received_at_secs: time,
        acceleration_mps2: Some([0.0; 3]),
        provenance: SensorProvenance::Measured,
    }
}
fn feed(core: &mut TrackingCore, time: f64, rotations: impl Fn(Role) -> Quat) {
    for role in Role::CORE {
        core.ingest(sample(role, time, rotations(role))).unwrap();
    }
}
fn calibrated(config: TrackingConfig) -> TrackingCore {
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    for step in 0..=100 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    core.calibrate_neutral(1.0, &anchor(1.0, Quat::IDENTITY), false)
        .unwrap();
    // This fixture's sensor/world axes are independently known to be identity.
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    core
}
fn target(frame: &TrackingFrame, role: TargetRole) -> &TargetPose {
    frame
        .targets
        .iter()
        .find(|target| target.role == role)
        .unwrap()
}
fn position(frame: &TrackingFrame, role: JointRole) -> Vec3 {
    Vec3::from_array(
        frame
            .joints
            .iter()
            .find(|joint| joint.joint == role)
            .unwrap()
            .position,
    )
}
fn near_quat(actual: [f32; 4], expected: Quat) {
    assert!(
        Quat::from_array(actual).dot(expected).abs() > 0.99999,
        "actual {actual:?}, expected {:?}",
        expected.to_array()
    );
}

#[test]
fn sensor_reordering_preserves_identity_and_calibration() {
    let mut core = calibrated(TrackingConfig::default());
    let before = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    let mut reordered = assignments(&Role::CORE);
    reordered.reverse();
    core.set_assignments(reordered).unwrap();
    for role in Role::CORE.into_iter().rev() {
        core.ingest(sample(role, 1.01, Quat::IDENTITY)).unwrap();
    }
    let after = core.solve(1.01, &anchor(1.01, Quat::IDENTITY));
    assert!(after.calibration.calibrated);
    assert_eq!(
        target(&before, TargetRole::LeftFoot).position,
        target(&after, TargetRole::LeftFoot).position
    );
}

#[test]
fn ordinary_head_only_turn_does_not_rotate_body() {
    // A real head rotates about the neck, so the HMD eye point moves. The
    // inferred body must stay put and must not follow the head's heading.
    let mut core = calibrated(TrackingConfig::default());
    let body = core.config().anthropometrics.clone();
    let eye_from_neck = Vec3::new(0.0, body.head_to_neck_m, -body.eye_forward_m);
    let neutral_eye = Vec3::new(0.0, 1.65, 0.0);
    let neck_pivot = neutral_eye - eye_from_neck;
    let before = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert!((position(&before, JointRole::Neck) - neck_pivot).length() < 1e-5);
    let head_poses = [
        Quat::from_rotation_y(1.3),
        Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2),
        Quat::from_rotation_x(-60.0_f32.to_radians()),
        Quat::from_rotation_x(25.0_f32.to_radians()),
        Quat::from_rotation_y(0.8) * Quat::from_rotation_x(-1.0),
    ];
    for (index, head) in head_poses.into_iter().enumerate() {
        let time = 1.01 + index as f64 * 0.01;
        feed(&mut core, time, |_| Quat::IDENTITY);
        let mut anchors = anchor(time, head);
        let eye = neck_pivot + head * eye_from_neck;
        anchors.hmd.as_mut().unwrap().position = eye.to_array();
        let after = core.solve(time, &anchors);
        assert!((eye - neutral_eye).length() > 0.03, "eye point must move");
        assert!(after.targets.iter().all(|t| t.valid));
        for joint in before.joints.iter().filter(|j| j.joint != JointRole::Head) {
            let moved = (position(&after, joint.joint) - Vec3::from_array(joint.position)).length();
            assert!(moved < 1e-4, "{:?} moved {moved} m", joint.joint);
        }
        for expected in &before.targets {
            let actual = target(&after, expected.role);
            let moved =
                (Vec3::from_array(actual.position) - Vec3::from_array(expected.position)).length();
            assert!(moved < 1e-4, "{:?} moved {moved} m", expected.role);
            near_quat(actual.orientation_xyzw, Quat::IDENTITY);
        }
    }
}

#[test]
fn real_whole_body_turn_is_preserved() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        ..Default::default()
    };
    let mut core = calibrated(config);
    let turn = Quat::from_rotation_y(1.2);
    for step in 101..=250 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| turn);
        core.solve(time, &anchor(time, turn));
    }
    let frame = core.solve(2.5, &anchor(2.5, turn));
    for target in &frame.targets {
        near_quat(target.orientation_xyzw, turn);
    }
    assert!(frame.targets.iter().all(|t| t.valid));
}

#[test]
fn explicit_role_binding_moves_the_intended_leg() {
    let mut core = calibrated(TrackingConfig::default());
    let before = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    feed(&mut core, 1.01, |role| {
        if role == Role::LeftThigh {
            Quat::from_rotation_x(-0.7)
        } else {
            Quat::IDENTITY
        }
    });
    let after = core.solve(1.01, &anchor(1.01, Quat::IDENTITY));
    assert!(
        (position(&before, JointRole::LeftKnee) - position(&after, JointRole::LeftKnee)).length()
            > 0.2
    );
    assert!(
        (position(&before, JointRole::RightKnee) - position(&after, JointRole::RightKnee)).length()
            < 1e-5
    );
}

#[test]
fn bone_lengths_stay_finite_and_constant_under_large_rotations() {
    let mut core = calibrated(TrackingConfig::default());
    for step in 1..=180 {
        let time = 1.0 + step as f64 / 100.0;
        let rotation =
            Quat::from_rotation_x(step as f32 * 0.07) * Quat::from_rotation_z(step as f32 * 0.03);
        feed(&mut core, time, |_| rotation);
        let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
        assert!(
            frame
                .joints
                .iter()
                .all(|j| j.position.iter().all(|p| p.is_finite()))
        );
        assert!(
            ((position(&frame, JointRole::LeftHip) - position(&frame, JointRole::LeftKnee))
                .length()
                - 0.42)
                .abs()
                < 1e-5
        );
        assert!(
            ((position(&frame, JointRole::LeftKnee) - position(&frame, JointRole::LeftAnkle))
                .length()
                - 0.42)
                .abs()
                < 1e-5
        );
    }
}

#[test]
fn missing_foot_orientation_is_explicitly_inferred_and_preserves_lying_rotation() {
    let mut core = calibrated(TrackingConfig::default());
    let lying = Quat::from_rotation_z(1.3);
    feed(&mut core, 1.01, |_| lying);
    let frame = core.solve(1.01, &anchor(1.01, Quat::IDENTITY));
    let foot = target(&frame, TargetRole::LeftFoot);
    assert_eq!(foot.orientation_source, Evidence::Inferred);
    assert_eq!(foot.position_source, Evidence::Inferred);
    assert!(foot.confidence < target(&frame, TargetRole::LeftKnee).confidence);
    near_quat(foot.orientation_xyzw, lying);
}

#[test]
fn optional_foot_and_upper_arm_roles_produce_measured_orientation_targets() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::ALL)).unwrap();
    for step in 0..=100 {
        let time = step as f64 / 100.0;
        for role in Role::ALL {
            core.ingest(sample(role, time, Quat::IDENTITY)).unwrap();
        }
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    core.calibrate_neutral(1.0, &anchor(1.0, Quat::IDENTITY), false)
        .unwrap();
    for role in Role::ALL {
        core.confirm_manual_mount(role).unwrap();
    }
    let frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert_eq!(frame.targets.len(), 8);
    assert_eq!(
        target(&frame, TargetRole::LeftFoot).orientation_source,
        Evidence::MeasuredOrientation
    );
    assert!(target(&frame, TargetRole::LeftElbow).valid);
}

#[test]
fn stale_required_calf_invalidates_only_its_dependent_targets() {
    let mut core = calibrated(TrackingConfig::default());
    for role in Role::CORE {
        if role != Role::LeftCalf {
            core.ingest(sample(role, 1.30, Quat::IDENTITY)).unwrap();
        }
    }
    let frame = core.solve(1.30, &anchor(1.30, Quat::IDENTITY));
    assert!(!target(&frame, TargetRole::LeftFoot).valid);
    assert_eq!(target(&frame, TargetRole::LeftFoot).confidence, 0.0);
    assert!(target(&frame, TargetRole::LeftKnee).valid);
    assert!(target(&frame, TargetRole::RightFoot).valid);
}

#[test]
fn stale_or_invalid_hmd_invalidates_every_body_target() {
    let mut core = calibrated(TrackingConfig::default());
    feed(&mut core, 1.5, |_| Quat::IDENTITY);
    let frame = core.solve(1.5, &anchor(1.0, Quat::IDENTITY));
    assert!(frame.targets.iter().all(|t| !t.valid));
    assert!(!frame.live_anchor);
    assert!(frame.preview_only);
    let mut bad = anchor(1.5, Quat::IDENTITY);
    bad.hmd.as_mut().unwrap().position[0] = f32::NAN;
    let frame = core.solve(1.5, &bad);
    assert!(
        frame
            .targets
            .iter()
            .all(|t| !t.valid && t.position.iter().all(|v| v.is_finite()))
    );
}

#[test]
fn invalid_inputs_and_duplicate_bindings_are_rejected() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    assert!(
        core.ingest(sample(Role::Chest, f64::NAN, Quat::IDENTITY))
            .is_err()
    );
    let mut bad = sample(Role::Chest, 0.0, Quat::IDENTITY);
    bad.orientation_xyzw = [0.0; 4];
    assert!(core.ingest(bad).is_err());
    let mut duplicate = assignments(&Role::CORE);
    duplicate[1].sensor_id = duplicate[0].sensor_id.clone();
    assert!(core.set_assignments(duplicate).is_err());
    let mut config = TrackingConfig::default();
    config.anthropometrics.thigh_length_m = f32::INFINITY;
    assert!(TrackingCore::new(config).is_err());
}

#[test]
fn sensor_clock_regression_and_future_samples_are_not_live() {
    let mut core = calibrated(TrackingConfig::default());
    assert!(
        core.ingest(sample(Role::Chest, 0.9, Quat::IDENTITY))
            .is_err()
    );
    // More than 2 s ahead of the newest time seen is rejected outright.
    assert!(
        core.ingest(sample(Role::Chest, 5.0, Quat::IDENTITY))
            .is_err()
    );
    // A modestly future-stamped sample is accepted but is not fresh "now".
    core.ingest(sample(Role::Chest, 2.9, Quat::IDENTITY))
        .unwrap();
    let frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert!(frame.targets.iter().all(|t| !t.valid));
}

#[test]
fn far_future_evidence_is_rejected_without_poisoning_the_sensor() {
    let mut core = calibrated(TrackingConfig::default());
    assert!(
        core.ingest(sample(Role::Chest, 3.5, Quat::IDENTITY))
            .is_err()
    );
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 3.5, 1.0))
            .is_err()
    );
    let mut far_transport = held_sample(Role::Chest, 1.0, 1.2, 1.0);
    if let SensorProvenance::FirmwareHeld {
        last_transport_at_secs,
        ..
    } = &mut far_transport.provenance
    {
        *last_transport_at_secs = 1.0e9;
    }
    assert!(core.ingest(far_transport).is_err());
    // Rejection mutated nothing: holding was not revoked and the next real
    // packet is not "out of order".
    core.ingest(held_sample(Role::Chest, 1.0, 1.2, 1.0))
        .unwrap();
    feed(&mut core, 1.21, |_| Quat::IDENTITY);
    let frame = core.solve(1.21, &anchor(1.21, Quat::IDENTITY));
    assert!(frame.targets.iter().all(|t| t.valid));
    // The horizon follows the solve clock as well as accepted evidence.
    core.solve(2.5, &anchor(2.5, Quat::IDENTITY));
    core.ingest(sample(Role::Chest, 4.4, Quat::IDENTITY))
        .unwrap();
    assert!(core.ingest(sample(Role::Hip, 6.5, Quat::IDENTITY)).is_err());
}

#[test]
fn solve_with_time_going_backwards_is_finite_and_not_live() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        enable_preview_motion: true,
        ..Default::default()
    };
    let mut core = calibrated(config);
    feed(&mut core, 1.5, |_| Quat::from_rotation_x(0.6));
    assert!(
        core.solve(1.5, &anchor(1.5, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| t.valid)
    );
    for now in [1.2, 0.0, 1.49] {
        let frame = core.solve(now, &anchor(now, Quat::IDENTITY));
        assert_finite_frame(&frame);
        // Measurements stamped after `now` are not fresh evidence for `now`.
        assert!(frame.targets.iter().all(|t| !t.valid));
    }
    feed(&mut core, 1.51, |_| Quat::IDENTITY);
    let frame = core.solve(1.51, &anchor(1.51, Quat::IDENTITY));
    assert_finite_frame(&frame);
    assert!(frame.targets.iter().all(|t| t.valid));
}

#[test]
fn sensor_cache_evicts_least_recently_seen_unassigned_ids() {
    let mut core = calibrated(TrackingConfig::default());
    // Six assigned ids are cached; fill the remaining 250 slots.
    for index in 0..250 {
        let mut extra = sample(Role::Chest, 1.0 + index as f64 * 1.0e-4, Quat::IDENTITY);
        extra.id = format!("extra-{index}");
        core.ingest(extra).unwrap();
    }
    let mut newcomer = sample(Role::Chest, 1.05, Quat::IDENTITY);
    newcomer.id = "newcomer".into();
    core.ingest(newcomer).unwrap();
    // extra-2 is still cached, so an older packet is out of order...
    let mut old = sample(Role::Chest, 0.5, Quat::IDENTITY);
    old.id = "extra-2".into();
    assert!(core.ingest(old.clone()).is_err());
    // ...but the least recently seen unassigned id (extra-0) was evicted.
    old.id = "extra-0".into();
    core.ingest(old).unwrap();
    // Assigned sensors, although seen earlier, are never evicted.
    feed(&mut core, 1.1, |_| Quat::IDENTITY);
    assert!(
        core.solve(1.1, &anchor(1.1, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| t.valid)
    );
    // Forgetting an assigned sensor invalidates it and drops its ordering.
    core.forget_sensor(&sensor_id(Role::Chest));
    assert!(
        core.solve(1.11, &anchor(1.11, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| !t.valid)
    );
    core.ingest(sample(Role::Chest, 0.9, Quat::IDENTITY))
        .unwrap();
    feed(&mut core, 1.12, |_| Quat::IDENTITY);
    assert!(
        core.solve(1.12, &anchor(1.12, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| t.valid)
    );
}

fn assert_finite_frame(frame: &TrackingFrame) {
    assert!(frame.at_secs.is_finite());
    for target in &frame.targets {
        assert!(
            target
                .position
                .iter()
                .chain(&target.orientation_xyzw)
                .chain([&target.confidence])
                .all(|v| v.is_finite()),
            "{target:?}"
        );
    }
    for joint in &frame.joints {
        assert!(
            joint
                .position
                .iter()
                .chain([&joint.confidence])
                .all(|v| v.is_finite()),
            "{joint:?}"
        );
    }
    for segment in &frame.segments {
        assert!(
            segment
                .orientation_xyzw
                .iter()
                .chain([&segment.yaw_correction_degrees, &segment.confidence])
                .all(|v| v.is_finite()),
            "{segment:?}"
        );
        assert!(
            segment
                .measurement_received_at_secs
                .into_iter()
                .chain(segment.measurement_age_secs)
                .all(f64::is_finite)
        );
    }
    for correction in &frame.corrections {
        assert!(
            [
                correction.residual_degrees,
                correction.applied_relative_degrees,
                correction.confidence
            ]
            .iter()
            .all(|v| v.is_finite())
        );
    }
    assert!(
        frame
            .calibration
            .reference_forward
            .into_iter()
            .flatten()
            .chain(frame.calibration.floor_mismatch_m)
            .chain(frame.preview_motion.root_translation_m)
            .all(f32::is_finite)
    );
}

#[test]
fn nonfinite_values_in_every_api_field_never_panic_or_emit_nonfinite_output() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        enable_preview_motion: true,
        ..Default::default()
    };
    let mut core = calibrated(config.clone());
    let mut time = 1.0;
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let bad64 = f64::from(bad);
        time += 0.01;
        feed(&mut core, time, |_| Quat::from_rotation_x(0.4));
        for component in 0..4 {
            let mut s = sample(Role::Chest, time, Quat::IDENTITY);
            s.orientation_xyzw[component] = bad;
            assert!(core.ingest(s).is_err());
        }
        for component in 0..3 {
            let mut s = sample(Role::Chest, time, Quat::IDENTITY);
            s.acceleration_mps2.as_mut().unwrap()[component] = bad;
            assert!(core.ingest(s).is_err());
        }
        assert!(
            core.ingest(sample(Role::Chest, bad64, Quat::IDENTITY))
                .is_err()
        );
        for field in 0..3 {
            let mut held = held_sample(Role::Hip, time, time, time);
            held.orientation_xyzw = Quat::from_rotation_x(0.4).to_array();
            if let SensorProvenance::FirmwareHeld {
                observed_at_secs,
                last_transport_at_secs,
                continuity_since_secs,
                ..
            } = &mut held.provenance
            {
                match field {
                    0 => *observed_at_secs = bad64,
                    1 => *last_transport_at_secs = bad64,
                    _ => *continuity_since_secs = bad64,
                }
            }
            let _ = core.ingest(held);
        }
        for anchor_index in 0..3 {
            for field in 0..8 {
                let mut anchors = anchor(time, Quat::IDENTITY);
                let hand = AnchorPose {
                    position: [0.3, 1.0, -0.2],
                    orientation_xyzw: Quat::IDENTITY.to_array(),
                    received_at_secs: time,
                    valid: true,
                    linear_velocity: None,
                    angular_velocity: None,
                };
                anchors.left_hand = Some(hand.clone());
                anchors.right_hand = Some(hand);
                let pose = match anchor_index {
                    0 => anchors.hmd.as_mut(),
                    1 => anchors.left_hand.as_mut(),
                    _ => anchors.right_hand.as_mut(),
                }
                .unwrap();
                match field {
                    0..=2 => pose.position[field] = bad,
                    3..=6 => pose.orientation_xyzw[field - 3] = bad,
                    _ => pose.received_at_secs = bad64,
                }
                assert_finite_frame(&core.solve(time, &anchors));
                // Sensors are moving here, so no calibration may be accepted;
                // a corrupt HMD must be rejected regardless.
                assert!(core.calibrate_neutral(time, &anchors, false).is_err());
            }
        }
        assert_finite_frame(&core.solve(bad64, &anchor(time, Quat::IDENTITY)));
        assert!(
            core.calibrate_neutral(bad64, &anchor(time, Quat::IDENTITY), false)
                .is_err()
        );
        assert!(
            core.calibrate_neutral(bad64, &Anchors::default(), true)
                .is_err()
        );
        assert!(core.calibrate_forward(bad64).is_err());
        core.request_recovery(bad64);
        let before = core.solve(time, &anchor(time, Quat::IDENTITY));
        core.rebase_standing_space(bad);
        let after = core.solve(time, &anchor(time, Quat::IDENTITY));
        for (a, b) in before.joints.iter().zip(&after.joints) {
            assert_eq!(a.position, b.position, "nonfinite rebase must be a no-op");
        }
        for values in [
            tracking_core::manual_mounting::ManualMountValues {
                forward_degrees: bad,
                ..Default::default()
            },
            tracking_core::manual_mounting::ManualMountValues {
                physical_degrees_xyz: [0.0, bad, 0.0],
                ..Default::default()
            },
        ] {
            assert!(core.set_manual_mount(Role::Hip, values).is_err());
        }
        let mut broken = config.clone();
        broken.anthropometrics.eye_forward_m = bad;
        assert!(core.set_config(broken).is_err());
        let mut broken = config.clone();
        broken.stale_after_secs = bad64;
        assert!(core.set_config(broken).is_err());
        let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
        assert_finite_frame(&frame);
        assert!(frame.targets.iter().all(|t| t.valid));
    }
}

#[test]
fn default_dimensions_put_neutral_soles_on_the_floor_and_report_mismatch() {
    let body = Anthropometrics::default();
    assert!((body.standing_chain_height_m() - body.standing_eye_height_m).abs() < 1e-6);
    let mut core = calibrated(TrackingConfig::default());
    let frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    let mismatch = frame.calibration.floor_mismatch_m.unwrap();
    assert!(mismatch.abs() < 1e-4, "{mismatch}");
    for toe in [JointRole::LeftToe, JointRole::RightToe] {
        assert!(position(&frame, toe).y.abs() < 1e-5);
    }
    // A taller-than-configured headset is reported but not rejected.
    let mut tall = TrackingCore::new(TrackingConfig::default()).unwrap();
    tall.set_assignments(assignments(&Role::CORE)).unwrap();
    let raised = |time: f64| {
        let mut anchors = anchor(time, Quat::IDENTITY);
        anchors.hmd.as_mut().unwrap().position[1] = 1.70;
        anchors
    };
    for step in 0..=100 {
        let time = step as f64 / 100.0;
        feed(&mut tall, time, |_| Quat::IDENTITY);
        tall.solve(time, &raised(time));
    }
    tall.calibrate_neutral(1.0, &raised(1.0), false).unwrap();
    let mismatch = tall
        .solve(1.0, &raised(1.0))
        .calibration
        .floor_mismatch_m
        .unwrap();
    assert!((mismatch - 0.05).abs() < 1e-4, "{mismatch}");
    tall.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    assert_eq!(
        tall.solve(1.0, &Anchors::default())
            .calibration
            .floor_mismatch_m,
        None
    );
}

#[test]
fn legacy_dimensions_load_with_default_eye_offset_and_offset_is_validated() {
    let mut value = serde_json::to_value(Anthropometrics::default()).unwrap();
    value.as_object_mut().unwrap().remove("eye_forward_m");
    let legacy: Anthropometrics = serde_json::from_value(value).unwrap();
    assert_eq!(legacy.eye_forward_m, 0.08);
    for (offset, ok) in [(0.0, true), (0.20, true), (-0.01, false), (0.21, false)] {
        let body = Anthropometrics {
            eye_forward_m: offset,
            ..Default::default()
        };
        assert_eq!(body.validate().is_ok(), ok, "{offset}");
    }
}

fn drifted_recovery(core: &mut TrackingCore, start_tick: u32, drift: Quat) -> u32 {
    for step in start_tick..start_tick + 100 {
        let time = step as f64 / 100.0;
        feed(core, time, |_| drift);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    let request = start_tick + 99;
    core.request_recovery(request as f64 / 100.0);
    for step in request + 1..=request + 50 {
        let time = step as f64 / 100.0;
        feed(core, time, |_| drift);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    request + 50
}

fn yaw_degrees(frame: &TrackingFrame, role: Role) -> f32 {
    frame
        .segments
        .iter()
        .find(|s| s.role == role)
        .unwrap()
        .yaw_correction_degrees
}

#[test]
fn manual_undo_and_reset_after_recovery_keep_the_recovered_yaw() {
    let mut core = calibrated(TrackingConfig::default());
    let edit = tracking_core::manual_mounting::ManualMountValues {
        forward_degrees: 30.0,
        ..Default::default()
    };
    core.set_manual_mount(Role::LeftThigh, edit).unwrap();
    core.confirm_manual_mount(Role::LeftThigh).unwrap();
    let drift = Quat::from_rotation_y(10.0_f32.to_radians());
    let end = drifted_recovery(&mut core, 101, drift);
    let time = end as f64 / 100.0;
    let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
    assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    assert!((yaw_degrees(&frame, Role::LeftThigh) + 10.0).abs() < 1e-3);
    // The neutral reference survives recovery: an unedited role can be edited.
    core.set_manual_mount(Role::RightThigh, edit).unwrap();
    core.undo_manual_mount(Role::RightThigh).unwrap();
    // Forward calibration requires a new neutral after a recovery.
    assert!(
        core.calibrate_forward(time)
            .unwrap_err()
            .0
            .contains("neutral again")
    );
    core.undo_manual_mount(Role::LeftThigh).unwrap();
    let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
    assert!(frame.calibration.direction_calibrated);
    assert!((yaw_degrees(&frame, Role::LeftThigh) + 10.0).abs() < 1e-3);
    near_quat(
        target(&frame, TargetRole::LeftKnee).orientation_xyzw,
        Quat::IDENTITY,
    );
    core.reset_manual_mount(Role::LeftThigh).unwrap();
    let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
    assert!((yaw_degrees(&frame, Role::LeftThigh) + 10.0).abs() < 1e-3);
    near_quat(
        target(&frame, TargetRole::LeftKnee).orientation_xyzw,
        Quat::IDENTITY,
    );
}

#[test]
fn knee_correction_after_recovery_uses_its_own_budget_and_survives_recovery_undo() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        yaw_correction_rate_degrees_per_sec: 1.0,
        ..Default::default()
    };
    let mut core = calibrated(config);
    let drift = 18.0_f32.to_radians();
    let end = drifted_recovery(&mut core, 101, Quat::from_rotation_y(drift));
    let time = end as f64 / 100.0;
    let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
    assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    for role in Role::CORE {
        assert!((yaw_degrees(&frame, role) + 18.0).abs() < 1e-3);
    }
    let calf_drift = 15.0_f32.to_radians();
    let mut frame = frame;
    let mut corrected = false;
    for step in end + 1..=end + 3000 {
        let time = step as f64 / 100.0;
        let flex = (45.0 + 18.0 * (time as f32 * 3.0).sin()).to_radians();
        feed(&mut core, time, |role| {
            if role == Role::LeftCalf {
                Quat::from_rotation_y(drift + calf_drift) * Quat::from_rotation_x(flex)
            } else {
                Quat::from_rotation_y(drift)
            }
        });
        frame = core.solve(time, &anchor(time, Quat::IDENTITY));
        let status = &frame.corrections[0].status;
        assert_ne!(status, "correction_budget_exhausted");
        corrected |= status == "experimental_relative_correction";
    }
    assert!(corrected);
    let relative = yaw_degrees(&frame, Role::LeftThigh) - yaw_degrees(&frame, Role::LeftCalf);
    assert!(relative > 5.0, "relative knee correction {relative}°");
    // Undo removes only the recovery deltas, keeping later knee corrections.
    assert!(core.undo_recovery());
    let time = (end + 3000) as f64 / 100.0;
    let undone = core.solve(time, &anchor(time, Quat::IDENTITY));
    let thigh = yaw_degrees(&undone, Role::LeftThigh);
    let calf = yaw_degrees(&undone, Role::LeftCalf);
    assert!((thigh - calf - relative).abs() < 1e-2, "{thigh} {calf}");
    assert!((thigh + calf).abs() < 1e-2, "{thigh} {calf}");
    assert!(yaw_degrees(&undone, Role::Hip).abs() < 1e-3);
}

#[test]
fn recovery_updates_only_fresh_certified_roles_and_ignores_stale_optional_ones() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::ALL)).unwrap();
    let feed_all = |core: &mut TrackingCore, time: f64, rotation: Quat, stale: bool| {
        for role in Role::ALL {
            if !(stale && role == Role::RightFoot) {
                core.ingest(sample(role, time, rotation)).unwrap();
            }
        }
        core.solve(time, &anchor(time, Quat::IDENTITY));
    };
    for step in 0..=100 {
        feed_all(&mut core, step as f64 / 100.0, Quat::IDENTITY, false);
    }
    core.calibrate_neutral(1.0, &anchor(1.0, Quat::IDENTITY), false)
        .unwrap();
    for role in Role::CORE
        .into_iter()
        .chain([Role::LeftFoot, Role::RightFoot])
    {
        core.confirm_manual_mount(role).unwrap();
    }
    let drift = Quat::from_rotation_y(10.0_f32.to_radians());
    // The certified right foot goes silent; it must not block recovery.
    for step in 101..=200 {
        feed_all(&mut core, step as f64 / 100.0, drift, true);
    }
    core.request_recovery(2.0);
    for step in 201..=250 {
        feed_all(&mut core, step as f64 / 100.0, drift, true);
    }
    let frame = core.solve(2.5, &anchor(2.5, Quat::IDENTITY));
    assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    for role in Role::CORE.into_iter().chain([Role::LeftFoot]) {
        assert!((yaw_degrees(&frame, role) + 10.0).abs() < 1e-3, "{role:?}");
    }
    // Stale certified and uncertified optional roles keep their yaw.
    for role in [Role::RightFoot, Role::LeftUpperArm, Role::RightUpperArm] {
        assert_eq!(yaw_degrees(&frame, role), 0.0, "{role:?}");
    }
    assert!(!target(&frame, TargetRole::RightFoot).valid);
    near_quat(
        target(&frame, TargetRole::LeftFoot).orientation_xyzw,
        Quat::IDENTITY,
    );
    // Confirming a never-certified optional role activates no unchecked yaw.
    core.confirm_manual_mount(Role::LeftUpperArm).unwrap();
    assert_eq!(
        yaw_degrees(
            &core.solve(2.5, &anchor(2.5, Quat::IDENTITY)),
            Role::LeftUpperArm
        ),
        0.0
    );
    assert!(core.undo_recovery());
    let undone = core.solve(2.5, &anchor(2.5, Quat::IDENTITY));
    for role in Role::ALL {
        assert!(yaw_degrees(&undone, role).abs() < 1e-3, "{role:?}");
    }
}

#[test]
fn calibration_rejects_motion_stale_hmd_and_seated_pose() {
    let mut core = calibrated(TrackingConfig::default());
    let mounts = serde_json::to_string(&core.export_profile()).unwrap();
    let mut moving = sample(Role::Chest, 1.01, Quat::IDENTITY);
    moving.acceleration_mps2 = Some([0.0, 8.0, 0.0]);
    core.ingest(moving).unwrap();
    assert!(
        core.calibrate_neutral(1.01, &anchor(1.01, Quat::IDENTITY), false)
            .is_err()
    );
    assert_eq!(
        mounts,
        serde_json::to_string(&core.export_profile()).unwrap()
    );
    assert!(
        core.calibrate_neutral(2.0, &anchor(1.0, Quat::IDENTITY), false)
            .is_err()
    );
    let mut seated = anchor(1.01, Quat::IDENTITY);
    seated.hmd.as_mut().unwrap().position[1] = 1.0;
    assert!(
        core.calibrate_neutral(1.01, &seated, false)
            .unwrap_err()
            .0
            .contains("height")
    );
}

#[test]
fn calibration_height_failure_exposes_measured_configured_difference_and_limit() {
    let config = TrackingConfig {
        anthropometrics: Anthropometrics {
            standing_eye_height_m: 1.70,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    let mut low_headset = anchor(1.0, Quat::IDENTITY);
    low_headset.hmd.as_mut().unwrap().position[1] = 1.20;
    let error = core
        .calibrate_neutral(1.0, &low_headset, false)
        .unwrap_err();
    for expected in [
        "Headset height 1.20 m",
        "1.50–1.90 m",
        "configured eye height 1.70 m",
        "difference -0.50 m",
        "limit ±0.20 m",
        "Stand upright",
        "floor level",
    ] {
        assert!(error.0.contains(expected), "missing {expected:?}: {error}");
    }
    let frame = core.solve(1.0, &low_headset);
    assert!(!frame.calibration.calibrated);
    assert_eq!(frame.calibration.reason.as_deref(), Some(error.0.as_str()));
    assert!(core.export_profile().mounting.is_empty());
}

#[test]
fn calibration_height_tolerance_still_accepts_boundary_and_rejects_outside() {
    // Binary-exact values test equality at the existing inclusive boundary.
    // Near-boundary default values separately retain the unchanged 20 cm gate.
    for (eye_height, tolerance, delta, should_accept) in [
        (1.625, 0.25, -0.25, true),
        (1.625, 0.25, 0.25, true),
        (1.625, 0.25, -0.2501, false),
        (1.625, 0.25, 0.2501, false),
        (1.65, 0.20, -0.1999, true),
        (1.65, 0.20, 0.1999, true),
        (1.65, 0.20, -0.2001, false),
        (1.65, 0.20, 0.2001, false),
    ] {
        let config = TrackingConfig {
            anthropometrics: Anthropometrics {
                standing_eye_height_m: eye_height,
                ..Default::default()
            },
            standing_height_tolerance_m: tolerance,
            ..Default::default()
        };
        let mut core = TrackingCore::new(config).unwrap();
        core.set_assignments(assignments(&Role::CORE)).unwrap();
        for step in 0..=100 {
            let time = step as f64 / 100.0;
            feed(&mut core, time, |_| Quat::IDENTITY);
            let mut standing = anchor(time, Quat::IDENTITY);
            standing.hmd.as_mut().unwrap().position[1] = eye_height + delta;
            core.solve(time, &standing);
        }
        let mut standing = anchor(1.0, Quat::IDENTITY);
        standing.hmd.as_mut().unwrap().position[1] = eye_height + delta;
        let result = core.calibrate_neutral(1.0, &standing, false);
        assert_eq!(
            result.is_ok(),
            should_accept,
            "height delta {delta}: {result:?}"
        );
        if let Err(error) = result {
            assert!(error.0.contains("Headset height"));
            assert!(core.export_profile().mounting.is_empty());
        }
    }
}

#[test]
fn a_sparse_history_with_long_gaps_is_not_stable_evidence() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    for time in [0.0, 0.30, 0.60] {
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    assert!(
        core.calibrate_neutral(0.60, &anchor(0.60, Quat::IDENTITY), false)
            .unwrap_err()
            .0
            .contains("gaps")
    );
}

#[test]
fn explicit_preview_calibration_never_becomes_live_automatically() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    for step in 0..=100 {
        feed(&mut core, step as f64 / 100.0, |_| Quat::IDENTITY);
    }
    assert!(
        core.calibrate_neutral(1.0, &Anchors::default(), false)
            .is_err()
    );
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    let frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert!(frame.preview_only && frame.calibration.preview_only);
    assert!(frame.targets.iter().all(|t| !t.valid));
}

#[test]
fn desktop_preview_calibrates_identically_with_no_hmd_or_incidental_low_tilted_hmd() {
    let mut without_hmd = TrackingCore::new(TrackingConfig::default()).unwrap();
    let mut with_incidental_hmd = TrackingCore::new(TrackingConfig::default()).unwrap();
    for core in [&mut without_hmd, &mut with_incidental_hmd] {
        core.set_assignments(assignments(&Role::CORE)).unwrap();
        for step in 0..=100 {
            feed(core, step as f64 / 100.0, |_| Quat::from_rotation_z(0.7));
        }
    }
    let mut incidental = anchor(1.0, Quat::from_rotation_y(1.2) * Quat::from_rotation_x(1.0));
    incidental.hmd.as_mut().unwrap().position = [2.0, 0.4, -3.0];
    incidental.left_hand = Some(AnchorPose {
        position: [1.0, 0.8, -2.0],
        orientation_xyzw: Quat::IDENTITY.to_array(),
        received_at_secs: 1.0,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    });
    without_hmd
        .calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    with_incidental_hmd
        .calibrate_neutral(1.0, &incidental, true)
        .unwrap();
    assert_eq!(
        serde_json::to_string(&without_hmd.export_profile()).unwrap(),
        serde_json::to_string(&with_incidental_hmd.export_profile()).unwrap()
    );
    let baseline = without_hmd.solve(1.0, &Anchors::default());
    let preview = with_incidental_hmd.solve(1.0, &incidental);
    assert!(preview.preview_only && preview.calibration.preview_only);
    assert!(!preview.live_anchor);
    assert!(preview.targets.iter().all(|target| !target.valid));
    for joint in &baseline.joints {
        assert_eq!(
            position(&preview, joint.joint),
            Vec3::from_array(joint.position)
        );
    }
    assert_eq!(
        position(&preview, JointRole::Head),
        Vec3::new(0.0, 1.65, 0.0)
    );
    near_quat(
        target(&preview, TargetRole::Waist).orientation_xyzw,
        Quat::IDENTITY,
    );
}

#[test]
fn desktop_preview_ignores_late_anchors_until_explicit_live_alignment() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    for step in 0..=100 {
        feed(&mut core, step as f64 / 100.0, |_| Quat::IDENTITY);
    }
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    let baseline = core.solve(1.0, &Anchors::default());
    let mut live = anchor(1.01, Quat::IDENTITY);
    live.hmd.as_mut().unwrap().position = [2.0, 1.65, -3.0];
    live.left_hand = Some(AnchorPose {
        position: [1.8, 1.2, -3.2],
        orientation_xyzw: Quat::IDENTITY.to_array(),
        received_at_secs: 1.01,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    });
    feed(&mut core, 1.01, |_| Quat::IDENTITY);
    let preview = core.solve(1.01, &live);
    assert!(preview.preview_only && preview.calibration.preview_only);
    assert!(!preview.live_anchor);
    assert!(preview.targets.iter().all(|target| !target.valid));
    for joint in &baseline.joints {
        assert_eq!(
            position(&preview, joint.joint),
            Vec3::from_array(joint.position)
        );
    }
    // Runtime's explicit desktop-to-live switch clears calibration. Real anchors
    // govern position only after fresh live neutral alignment succeeds.
    core.clear_calibration();
    for step in 102..=201 {
        let time = step as f64 / 100.0;
        live.hmd.as_mut().unwrap().received_at_secs = time;
        live.left_hand.as_mut().unwrap().received_at_secs = time;
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &live);
    }
    core.calibrate_neutral(2.01, &live, false).unwrap();
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    let aligned = core.solve(2.01, &live);
    assert!(!aligned.preview_only && !aligned.calibration.preview_only);
    assert!(aligned.live_anchor);
    assert!(aligned.targets.iter().all(|target| target.valid));
    assert_eq!(
        position(&aligned, JointRole::Head),
        Vec3::new(2.0, 1.65, -3.0)
    );
    assert_eq!(
        position(&aligned, JointRole::LeftHand),
        Vec3::new(1.8, 1.2, -3.2)
    );
}

#[test]
fn profile_round_trip_preserves_mounts_but_requires_session_calibration() {
    let core = calibrated(TrackingConfig::default());
    let serialized = serde_json::to_string(&core.export_profile()).unwrap();
    let profile: Profile = serde_json::from_str(&serialized).unwrap();
    let mut restored = TrackingCore::new(TrackingConfig::default()).unwrap();
    restored.restore_profile(profile).unwrap();
    assert_eq!(
        serialized,
        serde_json::to_string(&restored.export_profile()).unwrap()
    );
    feed(&mut restored, 1.0, |_| Quat::IDENTITY);
    let frame = restored.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert!(frame.calibration.requires_session_calibration);
    assert!(frame.targets.iter().all(|t| !t.valid));
}

#[test]
fn config_toggles_preserve_calibration_but_dimensions_do_not() {
    let mut core = calibrated(TrackingConfig::default());
    let mut config = core.config().clone();
    config.enable_joint_yaw_correction = true;
    core.update_config(config.clone()).unwrap();
    assert!(
        core.solve(1.0, &anchor(1.0, Quat::IDENTITY))
            .calibration
            .calibrated
    );
    config.anthropometrics.thigh_length_m = 0.46;
    core.update_config(config).unwrap();
    assert!(
        !core
            .solve(1.0, &anchor(1.0, Quat::IDENTITY))
            .calibration
            .calibrated
    );
}

#[test]
fn recovery_is_pending_during_motion_and_preserves_mounting() {
    let mut core = calibrated(TrackingConfig::default());
    let profile_before = serde_json::to_string(&core.export_profile()).unwrap();
    core.request_recovery(1.0);
    for step in 101..=180 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |role| {
            if role == Role::LeftThigh {
                Quat::from_rotation_x(step as f32 * 0.02)
            } else {
                Quat::IDENTITY
            }
        });
        let frame = core.solve(time, &anchor(time, Quat::IDENTITY));
        assert!(!matches!(frame.recovery, RecoveryState::Applied { .. }));
    }
    assert_eq!(
        profile_before,
        serde_json::to_string(&core.export_profile()).unwrap()
    );
}

#[test]
fn bounded_asserted_neutral_recovery_can_be_undone_without_touching_mounts() {
    let mut core = calibrated(TrackingConfig::default());
    let before = serde_json::to_string(&core.export_profile()).unwrap();
    let drift = Quat::from_rotation_y(10.0_f32.to_radians());
    for step in 101..=200 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| drift);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    core.request_recovery(2.0);
    for step in 201..=250 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| drift);
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    let frame = core.solve(2.5, &anchor(2.5, Quat::IDENTITY));
    assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    near_quat(
        target(&frame, TargetRole::Waist).orientation_xyzw,
        Quat::IDENTITY,
    );
    assert_eq!(
        before,
        serde_json::to_string(&core.export_profile()).unwrap()
    );
    assert!(core.undo_recovery());
    near_quat(
        target(
            &core.solve(2.5, &anchor(2.5, Quat::IDENTITY)),
            TargetRole::Waist,
        )
        .orientation_xyzw,
        drift,
    );
}

#[test]
fn asserted_recovery_cannot_distinguish_small_head_turn_from_common_drift() {
    // This limitation is deliberate and documented: the recovery action is the
    // user's assertion of neutral stance AND forward-facing head/torso. It is
    // not an automatic pose classifier; a mistaken assertion can rotate the body.
    let mut core = calibrated(TrackingConfig::default());
    let head_turn = Quat::from_rotation_y(15.0_f32.to_radians());
    for step in 101..=200 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &anchor(time, head_turn));
    }
    core.request_recovery(2.0);
    for step in 201..=250 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &anchor(time, head_turn));
    }
    let frame = core.solve(2.5, &anchor(2.5, head_turn));
    assert!(matches!(frame.recovery, RecoveryState::Applied { .. }));
    near_quat(
        target(&frame, TargetRole::Waist).orientation_xyzw,
        head_turn,
    );
}

#[test]
fn recovery_refuses_large_heading_ambiguity_and_missing_anchor() {
    let mut core = calibrated(TrackingConfig::default());
    core.request_recovery(1.0);
    for step in 101..=950 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| Quat::IDENTITY);
        core.solve(time, &anchor(time, Quat::from_rotation_y(0.8)));
    }
    let frame = core.solve(9.5, &anchor(9.5, Quat::from_rotation_y(0.8)));
    assert!(matches!(frame.recovery, RecoveryState::Rejected { .. }));
    near_quat(
        target(&frame, TargetRole::Waist).orientation_xyzw,
        Quat::IDENTITY,
    );
    core.request_recovery(9.5);
    feed(&mut core, 10.0, |_| Quat::IDENTITY);
    let frame = core.solve(10.0, &Anchors::default());
    assert!(matches!(frame.recovery, RecoveryState::Pending { .. }));
    assert!(frame.targets.iter().all(|t| !t.valid));
}

#[test]
fn anchor_quaternion_scale_is_normalized_before_calibration_yaw() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    let expected = Quat::from_rotation_y(0.7);
    let mut last_anchor = Anchors::default();
    for step in 0..=100 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |_| Quat::IDENTITY);
        last_anchor = anchor(time, expected);
        last_anchor.hmd.as_mut().unwrap().orientation_xyzw = expected.to_array().map(|v| v * 2.0);
        core.solve(time, &last_anchor);
    }
    core.calibrate_neutral(1.0, &last_anchor, false).unwrap();
    near_quat(
        target(&core.solve(1.0, &last_anchor), TargetRole::Waist).orientation_xyzw,
        expected,
    );
}

#[test]
fn joint_correction_requires_excitation_and_preserves_common_heading() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        yaw_correction_rate_degrees_per_sec: 1.0,
        ..Default::default()
    };
    let mut core = calibrated(config);
    let common_heading = 0.7;
    let injected_drift = 15.0_f32.to_radians();
    let mut frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    for step in 101..=3100 {
        let time = step as f64 / 100.0;
        let flex = (45.0 + 18.0 * (time as f32 * 3.0).sin()).to_radians();
        feed(&mut core, time, |role| {
            if role == Role::LeftCalf {
                Quat::from_rotation_y(common_heading + injected_drift) * Quat::from_rotation_x(flex)
            } else {
                Quat::from_rotation_y(common_heading)
            }
        });
        frame = core.solve(time, &anchor(time, Quat::from_rotation_y(-1.0)));
    }
    let upper = frame
        .segments
        .iter()
        .find(|s| s.role == Role::LeftThigh)
        .unwrap();
    let lower = frame
        .segments
        .iter()
        .find(|s| s.role == Role::LeftCalf)
        .unwrap();
    assert!(upper.yaw_correction_degrees > 2.0);
    assert!(lower.yaw_correction_degrees < -2.0);
    assert!((upper.yaw_correction_degrees + lower.yaw_correction_degrees).abs() < 1e-4);
    near_quat(
        target(&frame, TargetRole::Waist).orientation_xyzw,
        Quat::from_rotation_y(common_heading),
    );
    // This is a synthetic invariance check, not an accuracy comparison with a
    // human knee. It resolves relative disagreement, not which IMU drifted.
    let residual = frame
        .corrections
        .iter()
        .find(|c| c.thigh_role == Role::LeftThigh)
        .unwrap()
        .residual_degrees
        .abs();
    assert!(residual < 8.0, "residual {residual}");
}

#[test]
fn static_unexcited_joint_is_not_silently_corrected() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        ..Default::default()
    };
    let mut core = calibrated(config);
    for step in 101..=500 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, |role| {
            if role == Role::LeftCalf {
                Quat::from_rotation_y(0.2) * Quat::from_rotation_x(0.8)
            } else {
                Quat::IDENTITY
            }
        });
        core.solve(time, &anchor(time, Quat::IDENTITY));
    }
    let frame = core.solve(5.0, &anchor(5.0, Quat::IDENTITY));
    assert!(
        frame
            .segments
            .iter()
            .all(|s| s.yaw_correction_degrees == 0.0)
    );
    assert_eq!(
        frame.corrections[0].status,
        "insufficient_recent_flexion_excitation"
    );
}

#[test]
fn finite_extreme_anchor_coordinates_cannot_overflow_ik_outputs() {
    let mut core = calibrated(TrackingConfig::default());
    let mut anchors = anchor(1.0, Quat::IDENTITY);
    anchors.left_hand = Some(AnchorPose {
        position: [f32::MAX, 0.0, -f32::MAX],
        orientation_xyzw: Quat::IDENTITY.to_array(),
        received_at_secs: 1.0,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    });
    let frame = core.solve(1.0, &anchors);
    assert!(
        frame
            .joints
            .iter()
            .all(|j| j.position.iter().all(|v| v.is_finite()))
    );
    assert!(
        !frame
            .joints
            .iter()
            .find(|j| j.joint == JointRole::LeftHand)
            .unwrap()
            .valid
    );
}

#[test]
fn high_acceleration_blocks_experimental_joint_update() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        ..Default::default()
    };
    let mut core = calibrated(config);
    feed(&mut core, 1.01, |_| Quat::IDENTITY);
    let mut accelerated = sample(
        Role::LeftCalf,
        1.01,
        Quat::from_rotation_y(0.2) * Quat::from_rotation_x(0.8),
    );
    accelerated.acceleration_mps2 = Some([7.0, 0.0, 0.0]);
    core.ingest(accelerated).unwrap();
    let frame = core.solve(1.01, &anchor(1.01, Quat::IDENTITY));
    assert_eq!(
        frame.corrections[0].status,
        "high_acceleration_constraint_unreliable"
    );
    assert!(
        frame
            .segments
            .iter()
            .all(|s| s.yaw_correction_degrees == 0.0)
    );
}

fn held_sample(role: Role, measured_at: f64, observed_at: f64, epoch: f64) -> SensorSample {
    SensorSample {
        id: sensor_id(role),
        orientation_xyzw: Quat::IDENTITY.to_array(),
        received_at_secs: measured_at,
        acceleration_mps2: None,
        provenance: SensorProvenance::FirmwareHeld {
            firmware: "0.7.2".into(),
            observed_at_secs: observed_at,
            last_transport_at_secs: observed_at,
            continuity_since_secs: epoch,
            sensor_status_healthy: true,
            sequence_contiguous: true,
        },
    }
}

#[test]
fn named_firmware_hold_supports_idle_calibration_without_new_measurements() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    let mount = Quat::from_rotation_x(0.7) * Quat::from_rotation_z(0.3);
    feed(&mut core, 0.0, |_| mount);
    core.solve(0.0, &anchor(0.0, Quat::IDENTITY));
    for step in 1..=100 {
        let now = step as f64 / 100.0;
        for role in Role::CORE {
            let mut held = held_sample(role, 0.0, now, 0.0);
            held.orientation_xyzw = mount.to_array();
            core.ingest(held).unwrap();
        }
        core.solve(now, &anchor(now, Quat::IDENTITY));
    }
    core.calibrate_neutral(1.0, &anchor(1.0, Quat::IDENTITY), false)
        .unwrap();
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    let frame = core.solve(1.0, &anchor(1.0, Quat::IDENTITY));
    assert!(frame.calibration.used_firmware_hold);
    assert!(frame.targets.iter().all(|target| target.valid));
    assert_eq!(
        target(&frame, TargetRole::Chest).orientation_source,
        Evidence::FirmwareHeldOrientation
    );
    near_quat(
        target(&frame, TargetRole::Chest).orientation_xyzw,
        Quat::IDENTITY,
    );
    for segment in &frame.segments {
        assert!(segment.uses_firmware_hold);
        assert_eq!(segment.measurement_received_at_secs, Some(0.0));
        assert_eq!(segment.measurement_age_secs, Some(1.0));
        assert!(segment.confidence > 0.0 && segment.confidence <= 0.45);
    }
    // One hundred held tokens did not manufacture a dense measurement history.
    for role in Role::CORE {
        core.revoke_firmware_hold(&sensor_id(role));
    }
    assert!(
        core.calibrate_neutral(1.0, &anchor(1.0, Quat::IDENTITY), false)
            .is_err()
    );
    assert!(
        core.solve(1.0, &anchor(1.0, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| !t.valid)
    );
}

#[test]
fn hold_gap_revokes_until_new_rotation_and_keeps_original_age() {
    let mut core = calibrated(TrackingConfig::default());
    for role in Role::CORE {
        core.ingest(held_sample(role, 1.0, 1.4, 1.0)).unwrap();
    }
    assert!(
        core.solve(1.4, &anchor(1.4, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| t.valid)
    );
    let mut gap = held_sample(Role::LeftCalf, 1.0, 1.41, 1.0);
    if let SensorProvenance::FirmwareHeld {
        sequence_contiguous,
        ..
    } = &mut gap.provenance
    {
        *sequence_contiguous = false;
    }
    core.ingest(gap).unwrap();
    let frame = core.solve(1.41, &anchor(1.41, Quat::IDENTITY));
    assert!(!target(&frame, TargetRole::LeftFoot).valid);
    assert!(target(&frame, TargetRole::RightFoot).valid);
    assert!(
        core.ingest(held_sample(Role::LeftCalf, 1.0, 1.42, 1.0))
            .is_err()
    );
    core.ingest(sample(Role::LeftCalf, 1.43, Quat::IDENTITY))
        .unwrap();
    assert!(
        core.ingest(held_sample(Role::LeftCalf, 1.43, 1.44, 1.0))
            .is_err(),
        "old pre-gap continuity epoch must not be reused"
    );
    core.ingest(sample(Role::LeftCalf, 1.45, Quat::IDENTITY))
        .unwrap();
    core.ingest(held_sample(Role::LeftCalf, 1.45, 1.46, 1.45))
        .unwrap();
    assert!(
        target(
            &core.solve(1.46, &anchor(1.46, Quat::IDENTITY)),
            TargetRole::LeftFoot
        )
        .valid
    );
}

#[test]
fn unknown_firmware_or_changed_held_quaternion_cannot_extend_freshness() {
    fn with_firmware(mut held: SensorSample, name: &str) -> SensorSample {
        if let SensorProvenance::FirmwareHeld { firmware, .. } = &mut held.provenance {
            *firmware = name.into();
        }
        held
    }
    let mut core = calibrated(TrackingConfig::default());
    let unknown = with_firmware(held_sample(Role::Chest, 1.0, 1.4, 1.0), "");
    assert!(core.ingest(unknown).is_err());
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 1.41, 1.0))
            .is_err()
    );
    assert!(
        !target(
            &core.solve(1.41, &anchor(1.41, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
    core.ingest(sample(Role::Chest, 1.5, Quat::IDENTITY))
        .unwrap();
    let mut changed = held_sample(Role::Chest, 1.5, 1.6, 1.5);
    changed.orientation_xyzw = Quat::from_rotation_y(0.01).to_array();
    assert!(
        core.ingest(changed).is_err(),
        "a held token must not carry a new rotation"
    );
    // DIY/development builds suppress unchanged rotations identically; the
    // transport vouches for them, so any named build up to 64 bytes is held.
    for (index, name) in ["main", "git-1a2b3c4", "0.7.3-custom"].iter().enumerate() {
        let measured = 1.6 + index as f64 * 0.5;
        let observed = measured + 0.3;
        feed(&mut core, measured, |_| Quat::IDENTITY);
        core.ingest(with_firmware(
            held_sample(Role::Chest, measured, observed, measured),
            name,
        ))
        .unwrap();
        // Past the measured stale limit, only the hold keeps the chest valid.
        let frame = core.solve(observed, &anchor(observed, Quat::IDENTITY));
        let chest = target(&frame, TargetRole::Chest);
        assert!(chest.valid, "{name}");
        assert_eq!(chest.orientation_source, Evidence::FirmwareHeldOrientation);
    }
    for name in ["   ", &"x".repeat(65)] {
        feed(&mut core, 3.5, |_| Quat::IDENTITY);
        assert!(
            core.ingest(with_firmware(
                held_sample(Role::Chest, 3.5, 3.55, 3.5),
                name
            ))
            .is_err()
        );
    }
}

#[test]
fn held_evidence_requires_existing_measurement_and_rejects_reordering() {
    let mut empty = TrackingCore::new(TrackingConfig::default()).unwrap();
    assert!(
        empty
            .ingest(held_sample(Role::Chest, 0.0, 0.1, 0.0))
            .is_err()
    );
    let mut core = calibrated(TrackingConfig::default());
    core.ingest(held_sample(Role::Chest, 1.0, 1.4, 1.0))
        .unwrap();
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 1.3, 1.0))
            .is_err()
    );
    assert!(
        !target(
            &core.solve(1.5, &anchor(1.5, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
    // Session/profile restore removes old measurements; old-session evidence
    // cannot silently resurrect the calibration or a held quaternion.
    let profile = core.export_profile();
    core.restore_profile(profile).unwrap();
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 1.6, 1.0))
            .is_err()
    );
    assert!(
        core.solve(1.6, &anchor(1.6, Quat::IDENTITY))
            .calibration
            .requires_session_calibration
    );
}

#[test]
fn held_transport_lease_expires_and_cannot_bridge_a_silent_interval() {
    let mut core = calibrated(TrackingConfig::default());
    core.ingest(held_sample(Role::Chest, 1.0, 1.01, 1.0))
        .unwrap();
    assert!(
        !target(
            &core.solve(2.52, &anchor(2.52, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 2.53, 1.0))
            .is_err()
    );
    core.ingest(sample(Role::Chest, 2.54, Quat::IDENTITY))
        .unwrap();
    core.ingest(held_sample(Role::Chest, 2.54, 2.55, 2.54))
        .unwrap();
    assert!(
        target(
            &core.solve(2.55, &anchor(2.55, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
}

#[test]
fn sensor_error_invalidates_even_recent_measured_data_immediately() {
    let mut core = calibrated(TrackingConfig::default());
    let mut error = held_sample(Role::Chest, 1.0, 1.01, 1.0);
    if let SensorProvenance::FirmwareHeld {
        sensor_status_healthy,
        ..
    } = &mut error.provenance
    {
        *sensor_status_healthy = false;
    }
    core.ingest(error).unwrap();
    assert!(
        core.solve(1.01, &anchor(1.01, Quat::IDENTITY))
            .targets
            .iter()
            .all(|t| !t.valid)
    );
    assert!(
        core.ingest(held_sample(Role::Chest, 1.0, 1.02, 1.0))
            .is_err()
    );
    core.ingest(sample(Role::Chest, 1.03, Quat::IDENTITY))
        .unwrap();
    assert!(
        target(
            &core.solve(1.03, &anchor(1.03, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
    core.mark_sensor_unhealthy(&sensor_id(Role::Chest));
    assert!(
        !target(
            &core.solve(1.04, &anchor(1.04, Quat::IDENTITY)),
            TargetRole::Chest
        )
        .valid
    );
}

#[test]
fn held_evidence_cannot_hide_recent_measured_motion_for_calibration() {
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments(&Role::CORE)).unwrap();
    feed(&mut core, 0.0, |_| Quat::IDENTITY);
    for step in 1..=80 {
        let now = step as f64 / 100.0;
        if step == 30 {
            feed(&mut core, now, |_| Quat::from_rotation_x(0.2));
        }
        for role in Role::CORE {
            let mut held = held_sample(role, if step >= 30 { 0.30 } else { 0.0 }, now, 0.0);
            if step >= 30 {
                held.orientation_xyzw = Quat::from_rotation_x(0.2).to_array();
            }
            core.ingest(held).unwrap();
        }
        core.solve(now, &anchor(now, Quat::IDENTITY));
    }
    assert!(
        core.calibrate_neutral(0.8, &anchor(0.8, Quat::IDENTITY), false)
            .unwrap_err()
            .0
            .contains("motion")
    );
}

#[test]
fn held_orientation_is_never_knee_excitation_evidence() {
    let config = TrackingConfig {
        enable_joint_yaw_correction: true,
        ..Default::default()
    };
    let mut core = calibrated(config);
    let rotation = Quat::from_rotation_y(0.2) * Quat::from_rotation_x(0.8);
    core.ingest(sample(Role::LeftCalf, 1.01, rotation)).unwrap();
    for role in Role::CORE {
        if role != Role::LeftCalf {
            core.ingest(sample(role, 1.4, Quat::IDENTITY)).unwrap();
        }
    }
    let mut held = held_sample(Role::LeftCalf, 1.01, 1.4, 1.01);
    held.orientation_xyzw = rotation.to_array();
    core.ingest(held).unwrap();
    let frame = core.solve(1.4, &anchor(1.4, Quat::IDENTITY));
    assert_eq!(
        frame.corrections[0].status,
        "firmware_held_input_is_not_new_motion_evidence"
    );
    assert!(
        frame
            .segments
            .iter()
            .all(|s| s.yaw_correction_degrees == 0.0)
    );
}

#[test]
fn old_recordings_default_to_measured_and_held_records_keep_original_time() {
    let old = r#"{"id":"old","orientation_xyzw":[0,0,0,1],"received_at_secs":1.0,"acceleration_mps2":null}"#;
    let decoded: SensorSample = serde_json::from_str(old).unwrap();
    assert_eq!(decoded.provenance, SensorProvenance::Measured);
    let held = held_sample(Role::Chest, 1.0, 3.0, 1.0);
    let restored: SensorSample =
        serde_json::from_str(&serde_json::to_string(&held).unwrap()).unwrap();
    assert_eq!(restored.received_at_secs, 1.0);
    assert_eq!(restored.provenance, held.provenance);
}
