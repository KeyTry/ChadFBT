use glam::{DQuat, DVec3, Quat};
use tracking_core::heading_shadow::*;
use tracking_core::{Role, SensorProvenance, TrackingConfig};

fn estimator() -> RelativeHeadingEstimator {
    RelativeHeadingEstimator::new(
        Role::LeftThigh,
        Role::LeftCalf,
        HeadingShadowConfig::default(),
    )
    .unwrap()
}

fn observation(t: f64, relative_yaw: f64, global_yaw: f64) -> HeadingPairSample {
    let parent = Quat::from_rotation_y(global_yaw as f32)
        * Quat::from_rotation_x((0.12 * (t * 2.1).sin()) as f32);
    let flex = 0.75 + 0.35 * (t * 5.0).sin();
    let child =
        Quat::from_rotation_y(relative_yaw as f32) * parent * Quat::from_rotation_x(flex as f32);
    HeadingPairSample {
        calibration_epoch: 1,
        continuity_epoch: 1,
        parent: segment(Role::LeftThigh, parent, t),
        child: segment(Role::LeftCalf, child, t),
        independent_witness: None,
    }
}

fn segment(role: Role, q: Quat, t: f64) -> HeadingSegmentSample {
    HeadingSegmentSample {
        role,
        orientation_xyzw: q.to_array(),
        measured_at_secs: t,
        confidence: 0.95,
        valid: true,
        provenance: SensorProvenance::Measured,
    }
}

fn witness(t: f64, delta: f64, bound: f64) -> IndependentHeadingWitness {
    IndependentHeadingWitness {
        relative_correction_radians: delta,
        error_bound_radians: bound,
        observed_at_secs: t,
        source: IndependentWitnessSource::ExternalRelativeHeading,
        caller_asserts_independent_physical_constraint: true,
    }
}

fn run(mut sample: impl FnMut(f64) -> HeadingPairSample) -> HeadingShadowReport {
    let mut estimator = estimator();
    for i in 0..=200 {
        let t = i as f64 / 100.0;
        estimator.observe(t, sample(t));
    }
    estimator.report(2.0)
}

fn degrees(report: &HeadingShadowReport) -> f64 {
    report.candidate_relative_yaw_radians.unwrap().to_degrees()
}

#[test]
fn ideal_hinge_recovers_relative_drift_without_choosing_common_heading() {
    let report = run(|t| observation(t, 15.0_f64.to_radians(), 0.0));
    assert_eq!(
        report.status,
        HeadingShadowStatus::ModelConditionalCandidate
    );
    assert!((degrees(&report) + 15.0).abs() < 0.001);
    let split = report.hypothetical_parent_child_yaw_radians.unwrap();
    assert_eq!(split[0] + split[1], 0.0);
    assert!(!report.live_correction_applied);
    let observable = &report.fit.unwrap().observability;
    assert!(!observable.common_mode_yaw_observable);
    assert_eq!(observable.nullspace_dimension, 1);
    assert_eq!(observable.full_condition_number, None);
    assert_eq!(observable.frozen_weight_hessian_eigenvalues[0], 0.0);
    assert!((observable.frozen_weight_relative_curvature - 1.0).abs() < 1e-10);
    assert!(observable.normalized_cost_gradient.abs() < 1e-9);
}

#[test]
fn real_turns_and_common_mode_drift_are_exactly_invisible_to_relative_fit() {
    let turning = run(|t| observation(t, 0.0, t * 2.4));
    let common_drift = run(|t| observation(t, 0.0, 25.0_f64.to_radians()));
    let relative_while_turning = run(|t| observation(t, 15.0_f64.to_radians(), t * 2.4));
    assert!(degrees(&turning).abs() < 0.001);
    assert!(degrees(&common_drift).abs() < 0.001);
    assert!((degrees(&relative_while_turning) + 15.0).abs() < 0.001);
}

#[test]
fn systematic_legitimate_twist_is_indistinguishable_without_extra_information() {
    // The exact same observations have two possible truths. The estimator is not
    // given that truth and must emit identical reports, including its limitation.
    let drift_truth = run(|t| observation(t, 15.0_f64.to_radians(), 0.0));
    let twist_truth = run(|t| observation(t, 15.0_f64.to_radians(), 0.0));
    assert_eq!(degrees(&drift_truth), degrees(&twist_truth));
    assert_eq!(
        twist_truth.status,
        HeadingShadowStatus::ModelConditionalCandidate
    );
    assert!(twist_truth.reason.contains("real twist"));
    let fit = twist_truth.fit.unwrap();
    assert!(fit.median_residual_radians < 1e-6);
    // Under true twist, applying the hypothetical fit creates 15 degrees of
    // relative orientation error even though the model residual approaches zero.
    assert!((degrees(&drift_truth).abs() - 15.0).abs() < 0.001);
}

#[test]
fn independent_physical_observation_can_veto_a_consistent_twist_candidate() {
    let report = run(|t| {
        let mut sample = observation(t, 15.0_f64.to_radians(), 0.0);
        sample.independent_witness = Some(witness(t, 0.0, 1.0_f64.to_radians()));
        sample
    });
    assert_eq!(report.status, HeadingShadowStatus::IndependentEvidenceVeto);
    assert_eq!(report.witness.status, WitnessStatus::CandidateOutsideBounds);
    assert!((degrees(&report) + 15.0).abs() < 0.001);
    assert!(report.hypothetical_parent_child_yaw_radians.is_none());
}

#[test]
fn independent_bounds_can_support_but_do_not_apply_a_relative_candidate() {
    let report = run(|t| {
        let mut sample = observation(t, 15.0_f64.to_radians(), t);
        sample.independent_witness = Some(witness(t, -15.0_f64.to_radians(), 1.0_f64.to_radians()));
        sample
    });
    assert_eq!(
        report.status,
        HeadingShadowStatus::IndependentlySupportedCandidate
    );
    assert_eq!(report.witness.unique_samples, 201);
    assert!(!report.live_correction_applied);
    assert!(report.witness.intersection_half_width_radians.unwrap() > 0.017);
}

#[test]
fn conflicting_independent_bounds_are_not_averaged_away() {
    let report = run(|t| {
        let mut sample = observation(t, 0.0, 0.0);
        let delta = if t < 1.0 { 0.0 } else { 10.0_f64.to_radians() };
        sample.independent_witness = Some(witness(t, delta, 1.0_f64.to_radians()));
        sample
    });
    assert_eq!(
        report.status,
        HeadingShadowStatus::IndependentEvidenceConflict
    );
    assert!(report.hypothetical_parent_child_yaw_radians.is_none());
}

#[test]
fn consistent_witnesses_across_pi_veto_instead_of_claiming_physical_conflict() {
    let report = run(|t| {
        let mut sample = observation(t, 0.0, 0.0);
        let angle: f64 = if t < 1.0 { 179.0 } else { -179.0 };
        sample.independent_witness = Some(witness(t, angle.to_radians(), 3.0_f64.to_radians()));
        sample
    });
    assert_eq!(report.status, HeadingShadowStatus::IndependentEvidenceVeto);
    assert_eq!(report.witness.status, WitnessStatus::CandidateOutsideBounds);
    let center = report
        .witness
        .intersection_center_radians
        .unwrap()
        .to_degrees()
        .abs();
    assert!((center - 180.0).abs() < 1e-9);
    assert!(
        (report
            .witness
            .intersection_half_width_radians
            .unwrap()
            .to_degrees()
            - 2.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn robust_weighting_limits_isolated_axis_outlier_bias() {
    let report = run(|t| {
        let step = (t * 100.0).round() as usize;
        let offset: f64 = if step.is_multiple_of(10) { 60.0 } else { 15.0 };
        observation(t, offset.to_radians(), t * 0.3)
    });
    assert_eq!(
        report.status,
        HeadingShadowStatus::ModelConditionalCandidate
    );
    assert!((degrees(&report) + 15.0).abs() < 0.6);
    let fit = report.fit.unwrap();
    assert!(fit.downweighted_fraction > 0.09 && fit.downweighted_fraction < 0.12);
    assert!(fit.p95_residual_radians.to_degrees() > 40.0);
}

#[test]
fn incompatible_axis_tilts_are_not_solved_by_inventing_yaw() {
    let report = run(|t| {
        let mut sample = observation(t, 10.0_f64.to_radians(), 0.0);
        let q = Quat::from_rotation_z(25.0_f32.to_radians())
            * Quat::from_array(sample.child.orientation_xyzw);
        sample.child.orientation_xyzw = q.to_array();
        sample
    });
    assert_eq!(report.status, HeadingShadowStatus::InconsistentModel);
    assert!(report.hypothetical_parent_child_yaw_radians.is_none());
}

#[test]
fn vertical_hinge_axis_has_zero_heading_information_despite_flexion() {
    let report = run(|t| {
        let mut sample = observation(t, 0.0, 0.0);
        let global = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        sample.parent.orientation_xyzw =
            (global * Quat::from_array(sample.parent.orientation_xyzw)).to_array();
        sample.child.orientation_xyzw =
            (global * Quat::from_array(sample.child.orientation_xyzw)).to_array();
        sample
    });
    assert_eq!(report.status, HeadingShadowStatus::HorizontallyDegenerate);
    let observability = &report.fit.unwrap().observability;
    assert!(observability.frozen_weight_relative_curvature < 1e-12);
    assert_eq!(observability.nullspace_dimension, 2);
}

#[test]
fn numerical_observability_and_motion_heuristic_are_distinct() {
    let report = run(|t| {
        let mut sample = observation(0.3, 15.0_f64.to_radians(), 0.0);
        sample.parent.measured_at_secs = t;
        sample.child.measured_at_secs = t;
        sample
    });
    assert_eq!(report.status, HeadingShadowStatus::InsufficientExcitation);
    let fit = report.fit.unwrap();
    assert!(fit.observability.frozen_weight_relative_curvature > 0.99);
    assert!(fit.flex_excursion_radians < 1e-10);
}

#[test]
fn confidence_and_held_provenance_cannot_create_fit_history() {
    let mut estimator = estimator();
    for i in 0..100 {
        let t = i as f64 / 100.0;
        estimator.observe(t, observation(t, 0.0, 0.0));
    }
    let mut sample = observation(1.0, 0.0, 0.0);
    sample.child.provenance = SensorProvenance::FirmwareHeld {
        firmware: "0.7.2".into(),
        observed_at_secs: 1.0,
        last_transport_at_secs: 1.0,
        continuity_since_secs: 0.0,
        sensor_status_healthy: true,
        sequence_contiguous: true,
    };
    let held = estimator.observe(1.0, sample);
    assert_eq!(held.status, HeadingShadowStatus::RejectedInput);
    assert_eq!(held.measured_pairs, 0);
    let mut sample = observation(1.01, 0.0, 0.0);
    sample.child.confidence = 0.45;
    assert_eq!(
        estimator.observe(1.01, sample).status,
        HeadingShadowStatus::RejectedInput
    );
}

#[test]
fn duplicate_and_reused_packets_do_not_inflate_sample_count() {
    let mut estimator = estimator();
    let first = observation(0.0, 0.0, 0.0);
    estimator.observe(0.0, first.clone());
    for _ in 0..100 {
        let report = estimator.observe(0.01, first.clone());
        assert_eq!(report.status, HeadingShadowStatus::ReusedMeasurement);
        assert_eq!(report.measured_pairs, 1);
    }
    let mut sample = observation(0.02, 0.0, 0.0);
    sample.child.measured_at_secs = 0.0;
    assert_eq!(estimator.observe(0.02, sample).measured_pairs, 1);
    assert_eq!(
        estimator
            .observe(0.03, observation(0.03, 0.0, 0.0))
            .measured_pairs,
        2
    );
}

#[test]
fn stale_gap_epoch_and_reorder_block_reuse_of_an_old_window() {
    let mut estimator = estimator();
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        estimator.observe(t, observation(t, 0.0, 0.0));
    }
    assert_eq!(estimator.report(1.3).status, HeadingShadowStatus::Stale);
    let gap = estimator.observe(1.31, observation(1.31, 0.0, 0.0));
    assert_eq!(gap.measured_pairs, 1);
    assert_eq!(gap.status, HeadingShadowStatus::Collecting);
    let reorder = estimator.observe(1.32, observation(1.30, 0.0, 0.0));
    assert_eq!(reorder.status, HeadingShadowStatus::RejectedInput);
    assert_eq!(reorder.measured_pairs, 0);
    let mut epoch = observation(1.33, 0.0, 0.0);
    epoch.calibration_epoch = 2;
    assert_eq!(estimator.observe(1.33, epoch).measured_pairs, 1);
    let mut epoch = observation(1.34, 0.0, 0.0);
    epoch.calibration_epoch = 2;
    epoch.continuity_epoch = 2;
    assert_eq!(estimator.observe(1.34, epoch).measured_pairs, 1);
}

#[test]
fn skew_wrong_roles_invalid_numbers_and_unasserted_witness_fail_closed() {
    let mut estimator = estimator();
    let mut skew = observation(0.1, 0.0, 0.0);
    skew.child.measured_at_secs = 0.0;
    assert_eq!(
        estimator.observe(0.1, skew).status,
        HeadingShadowStatus::RejectedInput
    );
    let mut roles = observation(0.2, 0.0, 0.0);
    roles.child.role = Role::RightCalf;
    assert_eq!(
        estimator.observe(0.2, roles).status,
        HeadingShadowStatus::RejectedInput
    );
    let mut invalid = observation(0.3, 0.0, 0.0);
    invalid.child.orientation_xyzw[0] = f32::NAN;
    assert_eq!(
        estimator.observe(0.3, invalid).status,
        HeadingShadowStatus::RejectedInput
    );
    let mut unasserted = observation(0.4, 0.0, 0.0);
    let mut w = witness(0.4, 0.0, 0.01);
    w.caller_asserts_independent_physical_constraint = false;
    unasserted.independent_witness = Some(w);
    assert_eq!(
        estimator.observe(0.4, unasserted).status,
        HeadingShadowStatus::RejectedInput
    );
}

#[test]
fn configuration_validation_excludes_zero_weight_and_unbounded_buffers() {
    let zero_weight = HeadingShadowConfig {
        min_input_confidence: 0.0,
        ..Default::default()
    };
    assert!(RelativeHeadingEstimator::new(Role::LeftThigh, Role::LeftCalf, zero_weight).is_err());
    let too_big = HeadingShadowConfig {
        max_samples: usize::MAX,
        ..Default::default()
    };
    assert!(RelativeHeadingEstimator::new(Role::LeftThigh, Role::LeftCalf, too_big).is_err());
    let nan_axis = HeadingShadowConfig {
        parent_hinge_axis: [f64::NAN, 0.0, 0.0],
        ..Default::default()
    };
    assert!(RelativeHeadingEstimator::new(Role::LeftThigh, Role::LeftCalf, nan_axis).is_err());
    let short_window = HeadingShadowConfig {
        window_secs: 0.1,
        min_span_secs: 0.05,
        min_witness_span_secs: 0.05,
        stale_after_secs: 0.25,
        ..Default::default()
    };
    assert!(RelativeHeadingEstimator::new(Role::LeftThigh, Role::LeftCalf, short_window).is_err());
    let bad_geometry = HeadingShadowConfig {
        parent_long_axis: [1.0, 0.01, 0.0],
        ..Default::default()
    };
    assert!(RelativeHeadingEstimator::new(Role::LeftThigh, Role::LeftCalf, bad_geometry).is_err());
    assert!(!TrackingConfig::default().enable_joint_yaw_correction);
}

#[test]
fn reporting_cannot_move_back_before_any_accumulated_observation() {
    let mut estimator = estimator();
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let mut sample = observation(t, 0.1, 0.0);
        sample.child.measured_at_secs += 0.01;
        estimator.observe(t + 0.02, sample);
    }
    assert_eq!(
        estimator.report(1.02).status,
        HeadingShadowStatus::ModelConditionalCandidate
    );
    assert_eq!(
        estimator.report(1.005).status,
        HeadingShadowStatus::RejectedInput
    );
    assert!(estimator.report(1.005).fit.is_none());
}

#[test]
fn scaled_garbage_quaternions_are_rejected_but_small_norm_error_is_normalized() {
    let mut estimator = estimator();
    let mut bad = observation(0.0, 0.0, 0.0);
    bad.child.orientation_xyzw = [0.0, 0.0, 0.0, 2.0];
    assert_eq!(
        estimator.observe(0.0, bad).status,
        HeadingShadowStatus::RejectedInput
    );
    let mut acceptable = observation(0.1, 0.0, 0.0);
    acceptable.child.orientation_xyzw = acceptable.child.orientation_xyzw.map(|v| v * 1.01);
    assert_eq!(
        estimator.observe(0.1, acceptable).status,
        HeadingShadowStatus::Collecting
    );
}

#[test]
fn observed_curvature_matches_finite_difference_and_differs_from_gauss_newton() {
    let parent = Quat::from_rotation_z(0.4);
    let base_child = Quat::from_rotation_y(0.2) * Quat::from_rotation_z(0.7);
    let report = run(|t| HeadingPairSample {
        calibration_epoch: 1,
        continuity_epoch: 1,
        parent: segment(Role::LeftThigh, parent, t),
        child: segment(
            Role::LeftCalf,
            base_child * Quat::from_rotation_x((0.6 + 0.3 * (5.0 * t).sin()) as f32),
            t,
        ),
        independent_witness: None,
    });
    let delta = report.candidate_relative_yaw_radians.unwrap();
    let obs = &report.fit.unwrap().observability;
    let p = DQuat::from_array(parent.to_array().map(f64::from)).normalize() * DVec3::X;
    let c = DQuat::from_array(base_child.to_array().map(f64::from)).normalize() * DVec3::X;
    let cost = |yp: f64, yc: f64| {
        0.5 * (DQuat::from_rotation_y(yp) * p - DQuat::from_rotation_y(yc) * c).length_squared()
    };
    let h = 0.0001;
    let center = cost(0.0, delta);
    let finite_curvature = (cost(0.0, delta + h) + cost(0.0, delta - h) - 2.0 * center) / (h * h);
    assert!((finite_curvature - obs.frozen_weight_relative_curvature).abs() < 1e-7);
    assert!((cost(h, delta + h) - center).abs() < 1e-14);
    assert!(
        (obs.relative_gauss_newton_curvature - obs.frozen_weight_relative_curvature).abs() > 0.1
    );
}

fn raw_input(
    q: DQuat,
    acceleration: DVec3,
    r: DVec3,
    omega: DVec3,
    alpha: DVec3,
) -> JointCenterKinematics {
    let specific_force = q.conjugate() * (acceleration - DVec3::new(0.0, -9.80665, 0.0))
        - alpha.cross(r)
        - omega.cross(omega.cross(r));
    JointCenterKinematics {
        orientation_xyzw: q.to_array(),
        specific_force_sensor_mps2: specific_force.to_array(),
        angular_velocity_sensor_radps: omega.to_array(),
        angular_acceleration_sensor_radps2: alpha.to_array(),
        sensor_to_joint_sensor_m: r.to_array(),
        sampled_at_secs: 1.0,
        provenance: SensorProvenance::Measured,
    }
}

#[test]
fn joint_center_reconstruction_has_correct_gravity_and_lever_arm_signs() {
    let a = DVec3::new(2.0, -0.3, 1.5);
    let raw = raw_input(
        DQuat::from_rotation_z(0.6) * DQuat::from_rotation_y(-0.4),
        a,
        DVec3::new(0.1, -0.2, 0.05),
        DVec3::new(1.2, -0.3, 0.8),
        DVec3::new(-0.4, 0.9, 0.1),
    );
    let reconstructed = DVec3::from_array(reconstruct_joint_center_acceleration(&raw).unwrap());
    assert!(reconstructed.distance(a) < 1e-12);
}

#[test]
fn stationary_joint_with_orbiting_sensor_has_zero_reconstructed_acceleration() {
    // Independent physical case: joint at origin, sensor at +r X, omega=2 Y.
    // Its centripetal acceleration is -4r X. alpha=3 Z adds +3r Y at the sensor.
    // The sensor-to-joint offset points -X; transport must cancel both terms.
    let radius = 0.3;
    let raw = JointCenterKinematics {
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        specific_force_sensor_mps2: [-4.0 * radius, 9.80665 + 3.0 * radius, 0.0],
        angular_velocity_sensor_radps: [0.0, 2.0, 0.0],
        angular_acceleration_sensor_radps2: [0.0, 0.0, 3.0],
        sensor_to_joint_sensor_m: [-radius, 0.0, 0.0],
        sampled_at_secs: 0.0,
        provenance: SensorProvenance::Measured,
    };
    let reconstructed = DVec3::from_array(reconstruct_joint_center_acceleration(&raw).unwrap());
    assert!(reconstructed.length() < 1e-12);
}

#[test]
fn raw_joint_factor_preserves_legitimate_twist_and_detects_relative_sensor_yaw() {
    let a = DVec3::new(2.0, 0.3, -1.0);
    let parent_q = DQuat::from_rotation_y(0.4);
    let child_q =
        DQuat::from_rotation_y(15.0_f64.to_radians()) * parent_q * DQuat::from_rotation_x(0.7);
    let parent = raw_input(parent_q, a, DVec3::new(0.0, -0.2, 0.0), DVec3::X, DVec3::Y);
    let mut child = raw_input(child_q, a, DVec3::new(0.0, 0.15, 0.0), DVec3::Z, -DVec3::X);
    let true_twist = joint_acceleration_witness(&parent, &child, 0.02, 0.02, 0.01).unwrap();
    assert!(true_twist.relative_correction_radians.abs() < 1e-12);
    let drift = DQuat::from_rotation_y(12.0_f64.to_radians());
    child.orientation_xyzw = (drift * child_q).to_array();
    let drifting = joint_acceleration_witness(&parent, &child, 0.02, 0.02, 0.01).unwrap();
    assert!((drifting.relative_correction_radians.to_degrees() + 12.0).abs() < 1e-10);
    assert!(drifting.error_bound_radians.to_degrees() < 1.1);
}

#[test]
fn gravity_only_or_bad_joint_factor_contract_cannot_authorize_heading() {
    let stationary = raw_input(
        DQuat::IDENTITY,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
    );
    assert!(joint_acceleration_witness(&stationary, &stationary, 0.01, 0.01, 0.01).is_err());
    let moving = raw_input(
        DQuat::IDENTITY,
        DVec3::X,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
    );
    let mut skewed = moving.clone();
    skewed.sampled_at_secs += 0.1;
    assert!(joint_acceleration_witness(&moving, &skewed, 0.01, 0.01, 0.01).is_err());
    let incompatible = raw_input(
        DQuat::IDENTITY,
        DVec3::X * 2.0,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
    );
    assert!(joint_acceleration_witness(&moving, &incompatible, 0.01, 0.01, 0.01).is_err());
}

#[test]
fn bounded_storage_and_finite_serializable_diagnostics() {
    let mut estimator = estimator();
    for i in 0..1000 {
        let t = i as f64 / 100.0;
        estimator.observe(t, observation(t, 0.2, t));
    }
    let report = estimator.report(9.99);
    assert!(report.measured_pairs <= 201);
    let json = serde_json::to_string(&report).unwrap();
    assert!(json.contains("model_conditional_candidate"));
    assert!(!json.contains("NaN"));
    assert!(!json.contains("Infinity"));
}
