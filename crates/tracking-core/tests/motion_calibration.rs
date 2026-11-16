//! Synthetic regressions for motion calibration. These check the implementation against its
//! own movement model (see `motion_support`); they are not evidence of worn-body accuracy.

mod motion_support;

use glam::{Quat, Vec3};
use motion_support::*;
use tracking_core::Role;
use tracking_core::forward_calibration::solve_forward_alignment;
use tracking_core::motion_calibration::*;
use tracking_core::synthetic::{Motion, XorShift64};

fn options() -> RunOptions {
    RunOptions::default()
}

fn forward_errors(result: &RunResult) -> [f32; 6] {
    result
        .forward_errors_degrees()
        .map(|e| e.unwrap_or(f32::INFINITY))
}

fn max_abs(values: &[f32]) -> f32 {
    values.iter().fold(0.0_f32, |m, v| m.max(v.abs()))
}

fn has_warning(status: &MotionCalibrationStatus, kind: MotionWarningKind) -> bool {
    status.warnings.iter().any(|w| w.kind == kind)
}

fn gates(segment: &MotionSegmentStatus) -> Vec<MotionGate> {
    segment.blockers.iter().map(|b| b.gate).collect()
}

fn assert_ready_within(result: &RunResult, limit_degrees: f32, label: &str) {
    assert!(
        result.status.ready,
        "{label}: not ready: {:?}",
        result.solution.as_ref().err()
    );
    let errors = result.psi_errors_degrees().expect("solution");
    assert!(
        max_abs(&errors) <= limit_degrees,
        "{label}: heading errors {errors:?} exceed {limit_degrees}°"
    );
}

#[test]
fn squats_recover_every_heading_within_two_degrees() {
    let result = run(&squats(), &Rig::standard(1), &options());
    assert_ready_within(&result, 2.0, "squats");
    for segment in &result.status.segments {
        assert_eq!(segment.progress, 1.0);
        assert!(matches!(segment.sign, SignEvidence::OneSided { .. }));
    }
    // The solution satisfies the neutral identity R(neutral) = Ry(h) for every role.
    let solution = result.solution.as_ref().unwrap();
    let reference = Quat::from_rotation_y(solution.reference_heading_radians);
    for (i, alignment) in solution.alignments.iter().enumerate() {
        let calibrated = Quat::from_rotation_y(alignment.heading_alignment_radians)
            * result.neutral[i]
            * Quat::from_array(alignment.mounting_xyzw);
        assert!(calibrated.angle_between(reference) < 1.0e-3);
    }
}

#[test]
fn bow_then_march_in_place_recovers_every_heading() {
    let result = run(&bow_then_march(), &Rig::standard(2), &options());
    assert_ready_within(&result, 2.0, "bow + march");
    assert!(!has_warning(
        &result.status,
        MotionWarningKind::SuspectedLeftRightSwap
    ));
    assert!(!has_warning(
        &result.status,
        MotionWarningKind::SuspectedThighCalfSwap
    ));
}

#[test]
fn held_skiing_pose_matches_the_two_pose_solve() {
    let source = held_ski();
    let rig = Rig::standard(3);
    let result = run(&source, &rig, &options());
    assert_ready_within(&result, 2.0, "held skiing pose");
    let solution = result.solution.as_ref().unwrap();
    let ski_truth = source.truth(source.duration());
    for (i, role) in Role::CORE.iter().enumerate() {
        let two_pose = solve_forward_alignment(
            *role,
            result.neutral[i].to_array(),
            rig.clean(&ski_truth, i).to_array(),
            result.reference_heading,
        )
        .unwrap();
        let difference = wrap_degrees(
            (solution.alignments[i].heading_alignment_radians - two_pose.heading_alignment_radians)
                .to_degrees(),
        );
        assert!(
            difference.abs() <= 1.0,
            "{role:?}: motion and two-pose solves differ by {difference}°"
        );
        // End to end: the calibrated held pose equals the true bone rotation.
        let calibrated = Quat::from_rotation_y(solution.alignments[i].heading_alignment_radians)
            * rig.clean(&ski_truth, i)
            * Quat::from_array(solution.alignments[i].mounting_xyzw);
        let error = calibrated.angle_between(ski_truth.bones[i]).to_degrees();
        assert!(error <= 1.0, "{role:?}: calibrated pose error {error}°");
    }
}

#[test]
fn twist_about_the_bone_does_not_bias_the_tip_line() {
    // Bones exactly vertical at neutral: twist cannot move the long-axis tip.
    let twisted = Modified {
        inner: bow_then_march(),
        mods: Mods {
            twist_degrees: 15.0,
            twist_period: 1.7,
            ..Mods::default()
        },
    };
    let result = run(&twisted, &Rig::standard(4), &options());
    assert_ready_within(&result, 1.0, "bow + march with 15° twist");
    for segment in &result.status.segments {
        assert!(segment.twist_rms_degrees.unwrap() > 3.0);
    }

    // A held pose with twist: the rotation-axis (two-pose) solve is biased by about half the
    // twist (6°) or rejects the pose (15°); the tip line is unaffected.
    for (twist, two_pose_expected) in [(6.0_f32, Some(3.0_f32)), (15.0, None)] {
        let source = Parametric::new(
            HEADING,
            vec![stand(1.0), hold_twisted(SKI_TILTS, twist, 4.0)],
        );
        let rig = Rig::standard(5);
        let result = run(&source, &rig, &options());
        assert_ready_within(&result, 0.5, "held skiing pose with twist");
        let truth = source.truth(source.duration());
        for (i, role) in Role::CORE.iter().enumerate() {
            let two_pose = solve_forward_alignment(
                *role,
                result.neutral[i].to_array(),
                rig.clean(&truth, i).to_array(),
                result.reference_heading,
            );
            match (two_pose, two_pose_expected) {
                (Ok(a), Some(expected)) => {
                    let error =
                        wrap_degrees((a.heading_alignment_radians - rig.psi[i]).to_degrees());
                    assert!((error.abs() - expected).abs() < 0.5, "{role:?}: {error}°");
                }
                (Err(_), None) => {}
                (other, _) => panic!("{role:?}: unexpected two-pose result {other:?}"),
            }
        }
    }

    // Measured coupling: synthetic squats have legs leaning ~2° inward at neutral, so twist
    // about the true axis moves the assumed (vertical) tip on a small cone.
    let squats = Modified {
        inner: squats(),
        mods: Mods {
            twist_degrees: 15.0,
            twist_period: 1.7,
            ..Mods::default()
        },
    };
    let result = run(&squats, &Rig::standard(4), &options());
    assert_ready_within(
        &result,
        4.0,
        "squats with 15° twist and leaning neutral legs",
    );
}

#[test]
fn a_returning_body_yaw_wobble_has_small_measured_error() {
    for period in [2.3, 9.0] {
        let wobble = Modified {
            inner: squats(),
            mods: Mods {
                wobble_degrees: 10.0,
                wobble_period: period,
                ..Mods::default()
            },
        };
        let result = run(&wobble, &Rig::standard(6), &options());
        // Measured: 1.1° (2.3 s period) and 1.9° (9 s period) with seed 7 in the example.
        assert_ready_within(&result, 2.5, "10° yaw wobble");
        assert!(result.status.body_turn_degrees.is_none());
    }
}

#[test]
fn barely_tilting_pelvis_blocks_only_the_hip_on_excitation() {
    let source = Parametric::new(
        HEADING,
        vec![stand(1.0), bows(2, 60.0, 4.0), march(4), stand(1.0)],
    );
    let result = run(&source, &Rig::standard(7), &options());
    assert!(!result.status.ready);
    for segment in &result.status.segments {
        if segment.role == Role::Hip {
            assert_eq!(gates(segment).first(), Some(&MotionGate::Excitation));
            assert!(segment.blockers[0].message.contains("pelvis"));
            assert!(segment.progress < 0.5);
        } else {
            assert!(segment.ready, "{:?}: {:?}", segment.role, segment.blockers);
        }
    }
    let error = result.solution.as_ref().unwrap_err().0.clone();
    assert!(error.contains("Hip"), "{error}");
    assert!(!error.contains("Chest:"), "{error}");
}

#[test]
fn a_sideways_torso_bend_is_indistinguishable_from_another_heading() {
    let lean = Modified {
        inner: squats(),
        mods: Mods {
            torso_bend_yaw_degrees: -90.0,
            ..Mods::default()
        },
    };
    // Documented limitation: without a shared reference every gate passes and the torso
    // headings are off by the bend direction.
    let result = run(&lean, &Rig::standard(8), &options());
    assert!(result.status.ready);
    let errors = forward_errors(&result);
    for i in 0..2 {
        assert!((errors[i].abs() - 90.0).abs() <= 3.0, "{errors:?}");
    }
    assert!(max_abs(&errors[2..]) <= 2.0, "{errors:?}");
    assert!(result.status.warnings.is_empty());

    // The headset's neck displacement is absolute: it exposes the sideways torso motion.
    let with_headset = RunOptions {
        headset: true,
        reference_from_head: true,
        ..options()
    };
    let result = run(&lean, &Rig::standard(8), &with_headset);
    assert!(has_warning(
        &result.status,
        MotionWarningKind::HeadsetDisagreement
    ));
    assert_eq!(
        result.status.selected_reference,
        MotionReference::HeadsetYaw
    );
}

#[test]
fn knee_checks_cannot_see_a_consistent_sideways_leg_component() {
    // A thigh moving 40° off the sagittal plane while its calf moves sagittally is absorbed
    // into the thigh heading: each calibrated segment becomes sagittal, so the knee looks
    // like a perfect hinge. Documented limitation, measured here.
    let thigh_side = Modified {
        inner: squats(),
        mods: Mods {
            knees_out_degrees: 40.0,
            knees_out_thigh_only: true,
            ..Mods::default()
        },
    };
    let result = run(&thigh_side, &Rig::standard(9), &options());
    let errors = forward_errors(&result);
    assert!((errors[2].abs() - 40.0).abs() <= 2.0, "{errors:?}");
    assert!(result.status.ready);
    for knee in &result.status.knees {
        assert!(knee.axis_disagreement_degrees.unwrap() < 5.0);
    }
}

#[test]
fn a_mid_procedure_body_turn_is_flagged() {
    let turned = Synthetic::new(
        vec![
            Motion::stand(1.0),
            Motion::squats(2),
            Motion::turn(40.0),
            Motion::stand(0.5),
            Motion::squats(3),
            Motion::stand(1.0),
        ],
        HEADING,
    );
    let result = run(&turned, &Rig::standard(10), &options());
    let turn = result.status.body_turn_degrees.expect("turn detected");
    assert!((turn - 40.0).abs() <= 8.0, "turn {turn}°");
    assert!(!result.status.ready);
    assert!(
        result
            .status
            .blockers
            .iter()
            .any(|b| b.gate == MotionGate::BodyTurn)
    );
    assert!(has_warning(&result.status, MotionWarningKind::BodyTurn));
    let error = result.solution.as_ref().unwrap_err().0.clone();
    assert!(error.contains("turned"), "{error}");

    // The same squats without the turn are not flagged.
    let result = run(&squats(), &Rig::standard(10), &options());
    assert!(result.status.body_turn_degrees.is_none());
}

#[test]
fn knees_out_squats_bias_the_leg_headings_by_the_knee_angle() {
    // Knees tracking 15° outward rotate each leg's flexion plane; the tip line measures that
    // plane, so the leg headings carry the same bias and no gate can tell.
    let knees = Modified {
        inner: squats(),
        mods: Mods {
            knees_out_degrees: 15.0,
            ..Mods::default()
        },
    };
    let result = run(&knees, &Rig::standard(11), &options());
    assert!(result.status.ready);
    let errors = forward_errors(&result);
    assert!(max_abs(&errors[..2]) <= 1.0, "{errors:?}");
    for (i, error) in errors.iter().enumerate().skip(2) {
        assert!(
            (13.0..=17.0).contains(&error.abs()),
            "{:?}: bias {error}°",
            Role::CORE[i]
        );
    }
    // Synthetic default 5° toe-out (knee pole follows the foot): measured 4.5° thigh and
    // 7.2° calf bias with the example's seed.
    let toe_out = Synthetic::with_toe_out(
        vec![Motion::stand(1.0), Motion::squats(3), Motion::stand(1.0)],
        HEADING,
        5.0,
    );
    let result = run(&toe_out, &Rig::standard(11), &options());
    let errors = forward_errors(&result);
    assert!((3.0..=9.0).contains(&max_abs(&errors[2..])), "{errors:?}");
}

#[test]
fn swapped_thigh_and_calf_raise_a_suspicion() {
    let mut rig = Rig::standard(12);
    rig.source_bone = [0, 1, 4, 3, 2, 5];
    let result = run(&squats(), &rig, &options());
    let warning = result
        .status
        .warnings
        .iter()
        .find(|w| w.kind == MotionWarningKind::SuspectedThighCalfSwap)
        .expect("swap suspicion");
    assert_eq!(warning.roles, vec![Role::LeftThigh, Role::LeftCalf]);
    // A swap is exactly a 180° heading flip of both segments: knee checks pass.
    let errors = forward_errors(&result);
    assert!((errors[2].abs() - 180.0).abs() <= 2.0, "{errors:?}");
    assert!((errors[4].abs() - 180.0).abs() <= 2.0, "{errors:?}");
    // Correct assignment: no suspicion.
    let result = run(&squats(), &Rig::standard(12), &options());
    assert!(!has_warning(
        &result.status,
        MotionWarningKind::SuspectedThighCalfSwap
    ));
}

#[test]
fn left_right_swapped_thighs_raise_a_suspicion_during_marching() {
    let mut rig = Rig::standard(13);
    rig.source_bone = [0, 1, 3, 2, 4, 5];
    let result = run(&bow_then_march(), &rig, &options());
    assert!(has_warning(
        &result.status,
        MotionWarningKind::SuspectedLeftRightSwap
    ));
}

#[test]
fn walking_reports_undetermined_signs_unless_the_knee_rule_resolves_them() {
    let no_rule = RunOptions {
        config: MotionCalibrationConfig {
            knee_sign_rule: false,
            ..MotionCalibrationConfig::default()
        },
        ..options()
    };
    let result = run(&walking(), &Rig::standard(14), &no_rule);
    assert!(!result.status.ready);
    let mut undetermined = 0;
    for segment in &result.status.segments[2..] {
        if segment.sign == SignEvidence::Undetermined {
            undetermined += 1;
            assert!(gates(segment).contains(&MotionGate::OneSidedness));
            assert!(segment.sensor_forward_heading_radians.is_none());
        }
    }
    assert!(undetermined >= 2, "walking should be two-sided");
    for segment in &result.status.segments[..2] {
        assert_eq!(gates(segment).first(), Some(&MotionGate::Excitation));
    }

    let result = run(&walking(), &Rig::standard(14), &options());
    let errors = result.forward_errors_degrees();
    let mut resolved = 0;
    for i in 2..6 {
        let segment = &result.status.segments[i];
        if matches!(segment.sign, SignEvidence::KneeRule { .. }) {
            resolved += 1;
        }
        let error = errors[i].expect("leg sign determined");
        assert!(error.abs() <= 2.0, "{:?}: {error}°", segment.role);
    }
    assert!(resolved >= 2);
    assert!(has_warning(
        &result.status,
        MotionWarningKind::SignFromKneeRule
    ));
    assert!(result.status.knees.iter().all(|k| k.sign_resolved_by_knee));
}

fn squat_then_rock_back(t: f64) -> [f32; 6] {
    // 3 s cycle: a squat-like bend, then a short rock back where the left shin tips
    // backward with a vertical thigh (hyperextension relative to the instruction).
    let u = t % 3.0;
    if u < 2.5 {
        let phi = 0.5 * (1.0 - (std::f64::consts::TAU * u / 2.5).cos()) as f32;
        [
            30.0 * phi,
            20.0 * phi,
            -45.0 * phi,
            -45.0 * phi,
            50.0 * phi,
            50.0 * phi,
        ]
    } else {
        let phi = (std::f64::consts::PI * (u - 2.5) / 0.5).sin().powi(2) as f32;
        [0.0, 0.0, 0.0, 0.0, -14.0 * phi, 0.0]
    }
}

#[test]
fn backward_knee_bending_is_blocked() {
    let source = Parametric::new(
        HEADING,
        vec![
            stand(1.0),
            Phase {
                movement: Movement::Custom {
                    tilts: squat_then_rock_back,
                },
                secs: 12.0,
            },
            stand(1.0),
        ],
    );
    let result = run(&source, &Rig::standard(15), &options());
    let left_calf = &result.status.segments[4];
    assert!(
        matches!(left_calf.sign, SignEvidence::OneSided { .. }),
        "one-sidedness {:?}",
        left_calf.one_sidedness
    );
    assert!(gates(left_calf).contains(&MotionGate::KneeHyperextension));
    assert!(has_warning(
        &result.status,
        MotionWarningKind::KneeHyperextension
    ));
    assert!(result.status.segments[5].ready);
    assert!(result.solution.is_err());
}

#[test]
fn headset_displacement_gives_absolute_forward_and_fuses_or_flags() {
    let headset = RunOptions {
        headset: true,
        reference_from_head: true,
        ..options()
    };
    for (head_turn, flagged) in [(0.0_f32, false), (20.0, false), (40.0, true)] {
        let bow = Modified {
            inner: headset_bow(),
            mods: Mods {
                head_yaw_degrees: head_turn,
                ..Mods::default()
            },
        };
        let result = run(&bow, &Rig::standard(16), &headset);
        let status = result.status.headset.as_ref().expect("headset status");
        let truth = result.body_heading;
        let error = |v: f32| wrap_degrees((v - truth).to_degrees());
        assert!(
            (0.22..=0.30).contains(&status.max_displacement_m),
            "{}",
            status.max_displacement_m
        );
        assert!(status.available, "{:?}", status.reason);
        let displacement = error(status.displacement_heading_radians.unwrap());
        assert!(
            displacement.abs() <= 4.0,
            "head {head_turn}: {displacement}°"
        );
        let yaw = error(result.reference_heading);
        assert!((yaw - head_turn).abs() < 0.1);
        assert_eq!(status.disagreement_flagged, flagged);
        let solution = result.solution.as_ref().expect("ready");
        if flagged {
            assert!(status.fused_heading_radians.is_none());
            assert_eq!(solution.reference, MotionReference::HeadsetYaw);
            assert!(has_warning(
                &result.status,
                MotionWarningKind::HeadsetDisagreement
            ));
            // The caller can still choose the displacement reference explicitly.
            let explicit = result
                .calibrator
                .solve_with(MotionReference::NeckDisplacement)
                .unwrap();
            assert!(error(explicit.reference_heading_radians).abs() <= 4.0);
        } else {
            let fused = status.fused_heading_radians.unwrap();
            assert_eq!(solution.reference, MotionReference::Fused);
            assert_eq!(solution.reference_heading_radians, fused);
            // Inverse-variance circular mean of the two inputs.
            let sigma_yaw = 8.0_f32.to_radians();
            let sigma_d = status.displacement_sigma_radians.unwrap();
            let (wy, wd) = (sigma_yaw.powi(-2), sigma_d.powi(-2));
            let d = status.displacement_heading_radians.unwrap();
            let expected = (wy * result.reference_heading.sin() + wd * d.sin())
                .atan2(wy * result.reference_heading.cos() + wd * d.cos());
            assert!((wrap_degrees((fused - expected).to_degrees())).abs() < 0.01);
            assert!(status.fused_sigma_radians.unwrap() < sigma_d.min(sigma_yaw));
            if head_turn > 0.0 {
                let fused_error = error(fused);
                assert!(fused_error.abs() < head_turn && fused_error.abs() > displacement.abs());
            }
        }
    }
    // Without a neck reference the displacement reference is unavailable.
    let result = run(&headset_bow(), &Rig::standard(16), &options());
    assert!(result.status.headset.is_none());
    assert!(
        result
            .calibrator
            .solve_with(MotionReference::NeckDisplacement)
            .is_err()
    );
    assert_eq!(
        result.solution.as_ref().unwrap().reference,
        MotionReference::HeadsetYaw
    );
}

fn begun(config: MotionCalibrationConfig) -> MotionCalibrator {
    let mut calibrator = MotionCalibrator::new(config).unwrap();
    calibrator
        .begin(
            1.0,
            [Quat::IDENTITY; 6],
            0.0,
            Some(Vec3::new(0.0, 1.5, 0.0)),
        )
        .unwrap();
    calibrator
}

fn rejected(reason: RejectReason) -> SampleOutcome {
    SampleOutcome::Rejected { reason }
}

#[test]
fn invalid_late_and_gapped_samples_are_handled() {
    let mut calibrator = MotionCalibrator::new(MotionCalibrationConfig::default()).unwrap();
    assert_eq!(
        calibrator.observe_sensor(0, 1.0, Quat::IDENTITY),
        rejected(RejectReason::NotActive)
    );
    assert!(!calibrator.status().active);
    assert!(calibrator.solve().is_err());
    assert!(calibrator.begin(0.0, [Quat::NAN; 6], 0.0, None).is_err());
    assert!(
        calibrator
            .begin(0.0, [Quat::IDENTITY; 6], f32::NAN, None)
            .is_err()
    );
    assert!(!calibrator.is_active());

    let mut calibrator = begun(MotionCalibrationConfig::default());
    let tilted = Quat::from_rotation_x(-0.5);
    assert_eq!(
        calibrator.observe_sensor(6, 1.1, tilted),
        rejected(RejectReason::InvalidRole)
    );
    assert_eq!(
        calibrator.observe_sensor(0, 0.9, tilted),
        rejected(RejectReason::BeforeNeutral)
    );
    assert_eq!(
        calibrator.observe_sensor(0, f64::NAN, tilted),
        rejected(RejectReason::NonFinite)
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.1, Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
        rejected(RejectReason::NonFinite)
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.1, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)),
        rejected(RejectReason::NonFinite)
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.1, tilted),
        SampleOutcome::Accepted { evidence: false }
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.11, tilted),
        SampleOutcome::Accepted { evidence: true }
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.11, tilted),
        rejected(RejectReason::OutOfOrder)
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.105, tilted),
        rejected(RejectReason::OutOfOrder)
    );
    // Another stream moves the clock forward; a late sample is stale.
    assert_eq!(
        calibrator.observe_sensor(1, 2.0, Quat::IDENTITY),
        SampleOutcome::Accepted { evidence: false }
    );
    assert_eq!(
        calibrator.observe_sensor(0, 1.5, tilted),
        rejected(RejectReason::Stale)
    );
    let before = calibrator.status().segments[0].evidence_secs;
    assert!(before > 0.0);
    // A 0.4 s gap: the resumed sample has no weight, evidence is kept, blocks restart.
    assert_eq!(
        calibrator.observe_sensor(0, 2.0, tilted),
        SampleOutcome::ResumedAfterGap
    );
    let status = calibrator.status();
    assert_eq!(status.segments[0].gaps, 1);
    assert_eq!(status.segments[0].evidence_secs, before);
    assert_eq!(status.segments[0].rejected_samples, 7);
    assert_eq!(
        calibrator.observe_sensor(0, 2.01, tilted),
        SampleOutcome::Accepted { evidence: true }
    );

    // Headset input validation.
    let head = Vec3::new(0.0, 1.65, 0.0);
    assert_eq!(
        calibrator.observe_headset(2.0, Vec3::NAN, Quat::IDENTITY, 0.15, 0.08),
        rejected(RejectReason::NonFinite)
    );
    assert_eq!(
        calibrator.observe_headset(2.0, head, Quat::IDENTITY, f32::NAN, 0.08),
        rejected(RejectReason::InvalidDimensions)
    );
    assert_eq!(
        calibrator.observe_headset(2.0, head, Quat::IDENTITY, 0.15, 0.08),
        SampleOutcome::Accepted { evidence: false }
    );
    assert_eq!(
        calibrator.observe_headset(2.0, head, Quat::IDENTITY, 0.15, 0.08),
        rejected(RejectReason::OutOfOrder)
    );
    let mut no_neck = MotionCalibrator::new(MotionCalibrationConfig::default()).unwrap();
    no_neck.begin(0.0, [Quat::IDENTITY; 6], 0.0, None).unwrap();
    assert_eq!(
        no_neck.observe_headset(1.0, head, Quat::IDENTITY, 0.15, 0.08),
        rejected(RejectReason::NoNeckReference)
    );

    calibrator.reset();
    assert!(!calibrator.is_active());
    assert_eq!(calibrator.status().segments[0].evidence_secs, 0.0);
}

fn assert_status_finite(status: &MotionCalibrationStatus) {
    let finite = |v: Option<f32>| v.is_none_or(f32::is_finite);
    for segment in &status.segments {
        assert!((0.0..=1.0).contains(&segment.progress));
        assert!(segment.max_tilt_degrees.is_finite());
        assert!(segment.tilted_secs.is_finite() && segment.evidence_secs.is_finite());
        for value in [
            segment.planarity,
            segment.one_sidedness,
            segment.repeatability_degrees,
            segment.residual_degrees,
            segment.stability_degrees,
            segment.twist_rms_degrees,
            segment.sensor_forward_heading_radians,
            segment.heading_alignment_radians,
        ] {
            assert!(finite(value), "{segment:?}");
        }
    }
    if let Some(headset) = &status.headset {
        for value in [
            headset.displacement_heading_radians,
            headset.displacement_sigma_radians,
            headset.fused_heading_radians,
            headset.fused_sigma_radians,
            headset.difference_from_yaw_radians,
            Some(headset.max_displacement_m),
            Some(headset.evidence_secs),
        ] {
            assert!(finite(value), "{headset:?}");
        }
    }
    serde_json::to_string(status).unwrap();
}

#[test]
fn garbage_input_never_panics_or_produces_nonfinite_output() {
    let mut rng = XorShift64::new(99);
    let mut calibrator = begun(MotionCalibrationConfig::default());
    let specials = [
        f32::NAN,
        f32::INFINITY,
        -f32::INFINITY,
        f32::MAX,
        1.0e-30,
        0.0,
    ];
    let mut at = 1.0;
    for k in 0..20_000 {
        let pick = |rng: &mut XorShift64| {
            if rng.uniform() < 0.05 {
                specials[(rng.uniform() * specials.len() as f64) as usize % specials.len()]
            } else {
                (rng.gaussian() * 2.0) as f32
            }
        };
        let q = Quat::from_xyzw(
            pick(&mut rng),
            pick(&mut rng),
            pick(&mut rng),
            pick(&mut rng),
        );
        at += rng.uniform() * 0.03 - 0.005;
        let time = if rng.uniform() < 0.01 { f64::NAN } else { at };
        let role = (rng.uniform() * 7.0) as usize;
        calibrator.observe_sensor(role, time, q);
        let position = Vec3::new(pick(&mut rng), pick(&mut rng), pick(&mut rng));
        calibrator.observe_headset(time, position, q, pick(&mut rng).abs() * 0.1, 0.08);
        if k % 2_000 == 0 {
            assert_status_finite(&calibrator.status());
        }
    }
    assert_status_finite(&calibrator.status());
    if let Ok(solution) = calibrator.solve() {
        for alignment in solution.alignments {
            assert!(alignment.heading_alignment_radians.is_finite());
            assert!(alignment.mounting_xyzw.iter().all(|v| v.is_finite()));
        }
    }
}

#[test]
fn a_long_session_keeps_memory_and_evidence_bounded() {
    let config = MotionCalibrationConfig {
        max_snapshots: 400,
        ..MotionCalibrationConfig::default()
    };
    let long = Synthetic::new(vec![Motion::stand(1.0), Motion::squats(200)], HEADING);
    let options = RunOptions {
        rate_hz: 50.0,
        config: config.clone(),
        ..options()
    };
    let result = run(&long, &Rig::standard(17), &options);
    assert!(long.duration() > 600.0);
    assert_eq!(result.status.retained_snapshots, 400);
    for segment in &result.status.segments {
        assert!(segment.evidence_secs <= config.evidence_memory_secs as f32 + 0.06);
    }
    assert_ready_within(&result, 2.0, "ten minutes of squats");
}

#[test]
fn forgetting_limits_the_influence_of_an_early_bad_stretch() {
    // The first squat's torso bends 45° off the sagittal plane, then eight good squats.
    let bad_start = || Modified {
        inner: Synthetic::new(
            vec![Motion::stand(1.0), Motion::squats(9), Motion::stand(1.0)],
            HEADING,
        ),
        mods: Mods {
            torso_bend_yaw_degrees: 45.0,
            torso_bend_until: Some(4.0),
            ..Mods::default()
        },
    };
    let torso_error = |memory: f64| {
        let options = RunOptions {
            config: MotionCalibrationConfig {
                evidence_memory_secs: memory,
                ..MotionCalibrationConfig::default()
            },
            ..options()
        };
        let result = run(&bad_start(), &Rig::standard(23), &options);
        assert!(result.status.ready);
        max_abs(&forward_errors(&result)[..2])
    };
    let forgetting = torso_error(10.0);
    let remembering = torso_error(3600.0);
    // Measured: 1.4° versus 3.6°.
    assert!(forgetting <= 2.0, "{forgetting}");
    assert!(
        remembering > 2.0 * forgetting,
        "{remembering} vs {forgetting}"
    );
}

#[test]
fn asynchronous_noisy_streams_with_dropped_packets_still_converge() {
    let options = RunOptions {
        rate_hz: 90.0,
        stagger_secs: 0.004,
        drop_every: 7,
        drop_role: 4,
        ..options()
    };
    let mut rig = Rig::standard(21);
    rig.noise_degrees = 1.0;
    let result = run(&squats(), &rig, &options);
    assert_ready_within(&result, 2.5, "asynchronous 90 Hz streams, 1° noise, drops");
    assert!(result.status.knees.iter().all(|k| k.frames > 100));
}

#[test]
fn a_silent_sensor_blocks_readiness() {
    let source = squats();
    let rig = Rig::standard(22);
    let mut result = run(&source, &rig, &options());
    assert!(result.status.ready);
    let end = source.duration();
    let truth = source.truth(end);
    for k in 1..=100 {
        let at = end + f64::from(k) * 0.01;
        for i in 1..6 {
            result
                .calibrator
                .observe_sensor(i, at, rig.clean(&truth, i));
        }
    }
    let status = result.calibrator.status();
    assert!(!status.ready);
    assert_eq!(
        status.segments[0].blockers.first().map(|b| b.gate),
        Some(MotionGate::Stale)
    );
    assert!(result.calibrator.solve().is_err());
}

#[test]
fn identical_inputs_give_identical_results() {
    let a = run(&bow_then_march(), &Rig::standard(18), &options());
    let b = run(&bow_then_march(), &Rig::standard(18), &options());
    assert_eq!(
        serde_json::to_string(&a.status).unwrap(),
        serde_json::to_string(&b.status).unwrap()
    );
    let (a, b) = (a.solution.unwrap(), b.solution.unwrap());
    for (x, y) in a.alignments.iter().zip(&b.alignments) {
        assert_eq!(
            x.heading_alignment_radians.to_bits(),
            y.heading_alignment_radians.to_bits()
        );
        assert_eq!(
            x.mounting_xyzw.map(f32::to_bits),
            y.mounting_xyzw.map(f32::to_bits)
        );
    }
}

#[test]
fn progress_grows_and_gates_are_reported_before_completion() {
    let source = squats();
    let rig = Rig::standard(19);
    let early = run_until(&source, &rig, &options(), 1.3);
    assert!(!early.status.ready);
    for segment in &early.status.segments {
        assert!(segment.progress < 0.5, "{segment:?}");
        assert!(!segment.blockers.is_empty());
    }
    let late = run(&source, &rig, &options());
    for (before, after) in early.status.segments.iter().zip(&late.status.segments) {
        assert!(after.progress > before.progress);
    }
}

#[test]
fn configuration_validates_and_round_trips() {
    let config = MotionCalibrationConfig::default();
    config.validate().unwrap();
    let json = serde_json::to_string(&config).unwrap();
    let back: MotionCalibrationConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(back, config);
    let partial: MotionCalibrationConfig =
        serde_json::from_str(r#"{"reference":"headset_yaw","max_planarity":0.2}"#).unwrap();
    assert_eq!(partial.reference, MotionReference::HeadsetYaw);
    assert_eq!(partial.max_planarity, 0.2);
    for broken in [
        MotionCalibrationConfig {
            max_planarity: f32::NAN,
            ..config.clone()
        },
        MotionCalibrationConfig {
            min_evidence_tilt_degrees: 80.0,
            ..config.clone()
        },
        MotionCalibrationConfig {
            max_snapshots: 0,
            ..config.clone()
        },
        MotionCalibrationConfig {
            evidence_memory_secs: -1.0,
            ..config.clone()
        },
    ] {
        assert!(MotionCalibrator::new(broken).is_err());
    }
    let status = run(&held_ski(), &Rig::standard(20), &options()).status;
    let json = serde_json::to_string(&status).unwrap();
    let back: MotionCalibrationStatus = serde_json::from_str(&json).unwrap();
    assert_eq!(back, status);
}
