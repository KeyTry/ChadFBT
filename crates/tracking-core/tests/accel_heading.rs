//! Synthetic regressions for the acceleration-heading shadow estimator. They
//! check the algebra, gates and failure reporting against a toy rigid body;
//! they are not evidence of accuracy on worn hardware.

use glam::{Quat, Vec3};
use std::f32::consts::{FRAC_PI_2, PI};
use tracking_core::accel_heading::synthetic::*;
use tracking_core::accel_heading::*;

struct Run {
    run: SyntheticRun,
    reports: Vec<AccelHeadingReport>,
    estimator: AccelHeadingEstimator,
}

impl Run {
    fn last(&self) -> &AccelHeadingReport {
        self.reports.last().expect("at least one report")
    }

    fn candidates(&self) -> impl Iterator<Item = &AccelHeadingReport> {
        self.reports
            .iter()
            .filter(|r| r.status == AccelHeadingStatus::Candidate)
    }
}

fn simulate(
    scenario: &SyntheticScenario,
    config: AccelHeadingConfig,
    hint: Option<usize>,
    report_every_secs: f64,
) -> Run {
    let run = generate(scenario);
    let mut estimator = AccelHeadingEstimator::new(config).unwrap();
    estimator.set_frame_hint(hint).unwrap();
    let mut reports = Vec::new();
    let mut next = 1.0;
    for event in &run.events {
        assert_ne!(event.push_into(&mut estimator), SampleOutcome::Rejected);
        if event.at_secs() >= next {
            let report = estimator.report(event.at_secs());
            assert_finite(&report);
            assert!(!report.live_correction_applied);
            reports.push(report);
            next += report_every_secs;
        }
    }
    Run {
        run,
        reports,
        estimator,
    }
}

fn degrees_error(report: &AccelHeadingReport, expected_radians: f64) -> f64 {
    let yaw = report.yaw_correction_radians.expect("candidate yaw");
    let error = (yaw - expected_radians).to_degrees();
    (error + 180.0).rem_euclid(360.0) - 180.0
}

fn assert_finite(report: &AccelHeadingReport) {
    let mut values = vec![
        report.effective_samples,
        report.window_coverage,
        report.evaluated_at_secs.unwrap_or(0.0),
    ];
    values.extend(
        [
            report.lag_secs,
            report.yaw_correction_radians,
            report.yaw_sigma_radians,
            report.horizontal_correlation,
            report.frame.vertical_agreement,
            report.frame.vertical_correlation,
            report.frame.margin,
            report.frame.vertical_reference_rms_mps2,
            report.frame.tilt_spread_degrees,
        ]
        .into_iter()
        .flatten(),
    );
    for score in &report.frame.scores {
        values.extend([score.vertical_agreement, score.lag_secs]);
    }
    if let Some(h) = &report.horizontal {
        values.extend([
            h.yaw_radians,
            h.sigma_radians,
            h.correlation,
            h.gain,
            h.reference_rms_mps2,
            h.imu_rms_mps2,
            h.anisotropy,
        ]);
    }
    assert!(
        values.iter().all(|v| v.is_finite()),
        "nonfinite value in {report:?}"
    );
    assert!(serde_json::to_string(report).is_ok());
}

fn tilted_dance() -> SyntheticScenario {
    SyntheticScenario::default()
}

/// Upright walking, board axes aligned with the torso: no tilt diversity.
fn upright_walk() -> SyntheticScenario {
    SyntheticScenario {
        motion: SyntheticMotion::Walk,
        torso_tilt_degrees: 2.0,
        mount: Quat::IDENTITY,
        ..SyntheticScenario::default()
    }
}

fn leg_config() -> AccelHeadingConfig {
    AccelHeadingConfig {
        horizontal_reference: false,
        max_reference_angular_speed_rad_per_sec: 8.0,
        max_imu_angular_speed_rad_per_sec: 8.0,
        ..AccelHeadingConfig::default()
    }
}

/// Candidates whose matrix row for world-up (in sensor axes) matches the
/// true frame when the sensor's up axis is +Y: exactly those differ by a
/// rotation about the sensor's vertical axis.
fn same_up_row(frame: usize) -> Vec<usize> {
    let truth = frame_candidate(frame).unwrap();
    let up = truth.inverse() * Vec3::Y;
    (0..FRAME_CANDIDATE_COUNT)
        .filter(|&c| (frame_candidate(c).unwrap().inverse() * Vec3::Y - up).length() < 1e-4)
        .collect()
}

#[test]
fn candidate_set_is_the_24_proper_axis_aligned_rotations() {
    assert_eq!(frame_candidate(0), Some(Quat::IDENTITY));
    assert!(frame_candidate(FRAME_CANDIDATE_COUNT).is_none());
    for i in 0..FRAME_CANDIDATE_COUNT {
        let q = frame_candidate(i).unwrap();
        assert!((q.length() - 1.0).abs() < 1e-5);
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            let v = q * axis;
            let ones = v
                .to_array()
                .iter()
                .filter(|c| (c.abs() - 1.0).abs() < 1e-5)
                .count();
            assert_eq!(ones, 1, "candidate {i} is not axis aligned");
        }
        assert_eq!(frame_candidate_index(q), Some(i));
        for j in 0..i {
            assert!(q.angle_between(frame_candidate(j).unwrap()) > 1.0);
        }
    }
    assert_eq!(frame_candidate_index(Quat::from_rotation_y(0.3)), None);
    assert_eq!(
        frame_candidate_index(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
        None
    );
}

#[test]
fn sign_convention_is_a_left_world_yaw_that_realigns_the_sensor() {
    for yaw_error in [25.0, -25.0] {
        let scenario = SyntheticScenario {
            yaw_error_degrees: yaw_error,
            ..tilted_dance()
        };
        let run = simulate(&scenario, AccelHeadingConfig::default(), None, 1.0);
        let report = run.last();
        assert_eq!(report.status, AccelHeadingStatus::Candidate, "{report:?}");
        let yaw = report.yaw_correction_radians.unwrap() as f32;
        // The returned value has the opposite sign of the injected error...
        assert!(yaw.to_degrees() * (yaw_error as f32) < 0.0);
        // ...and applying it on the left re-aligns every supplied orientation.
        for event in &run.run.events {
            if let SyntheticEvent::Imu {
                sensor_to_room,
                true_sensor_to_room,
                ..
            } = event
            {
                let corrected = Quat::from_rotation_y(yaw) * *sensor_to_room;
                assert!(corrected.angle_between(*true_sensor_to_room).to_degrees() < 3.0);
            }
        }
    }
}

#[test]
fn identifies_the_true_frame_among_24_given_tilt_diversity() {
    for frame in 0..FRAME_CANDIDATE_COUNT {
        let scenario = SyntheticScenario {
            true_frame: frame,
            duration_secs: 12.0,
            seed: 100 + frame as u64,
            ..tilted_dance()
        };
        let run = simulate(&scenario, AccelHeadingConfig::default(), None, 1.0);
        for report in &run.reports {
            assert!(
                report.frame.candidate.is_none_or(|c| c == frame),
                "frame {frame}: wrong latch {report:?}"
            );
        }
        let report = run.last();
        assert_eq!(report.frame.candidate, Some(frame), "{report:?}");
        assert_eq!(report.frame.source, Some(FrameSource::Identified));
        assert_eq!(report.frame.ambiguous_candidates, vec![frame]);
        assert!(report.frame.margin.unwrap() >= 0.05);
        assert_eq!(report.status, AccelHeadingStatus::Candidate, "{report:?}");
    }
}

#[test]
fn upright_torso_reports_the_vertical_ambiguity_group() {
    let scenario = upright_walk();
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
    assert!(run.reports.iter().all(|r| r.frame.candidate.is_none()));
    assert_eq!(run.candidates().count(), 0);
    let report = run.last();
    assert_eq!(
        report.status,
        AccelHeadingStatus::FrameAmbiguous,
        "{report:?}"
    );
    assert!(report.yaw_correction_radians.is_none());
    let group = same_up_row(scenario.true_frame);
    assert_eq!(group.len(), 4);
    assert_eq!(report.frame.ambiguous_candidates, group);
    // The group really is about the vertical: small tilt spread.
    assert!(report.frame.tilt_spread_degrees.unwrap() < 3.0);
}

#[test]
fn a_correct_hint_resolves_ambiguity_and_wrong_hints_are_not_accepted() {
    let scenario = upright_walk();
    let truth = frame_candidate(scenario.true_frame).unwrap();

    let correct = simulate(
        &scenario,
        AccelHeadingConfig::default(),
        Some(scenario.true_frame),
        0.5,
    );
    let report = correct.last();
    assert_eq!(report.status, AccelHeadingStatus::Candidate, "{report:?}");
    assert_eq!(report.frame.source, Some(FrameSource::Hint));
    assert_eq!(report.frame.hint_status, HintStatus::Verified);
    for candidate in correct.candidates() {
        assert!(degrees_error(candidate, correct.run.expected_correction_radians).abs() < 3.0);
    }
    // Re-setting the same hint keeps the verified latch; clearing it drops it.
    let mut estimator = correct.estimator;
    let latched = Some((scenario.true_frame, FrameSource::Hint));
    assert_eq!(estimator.latched_frame(), latched);
    estimator.set_frame_hint(Some(scenario.true_frame)).unwrap();
    assert_eq!(estimator.latched_frame(), latched);
    estimator.set_frame_hint(None).unwrap();
    assert_eq!(estimator.latched_frame(), None);

    // A hint whose vertical acceleration differs is rejected outright.
    let outside = frame_candidate_index(Quat::from_rotation_x(FRAC_PI_2) * truth).unwrap();
    assert!(!same_up_row(scenario.true_frame).contains(&outside));
    let rejected = simulate(&scenario, AccelHeadingConfig::default(), Some(outside), 0.5);
    assert_eq!(rejected.candidates().count(), 0);
    assert_eq!(
        rejected.last().status,
        AccelHeadingStatus::FrameHintRejected
    );
    assert_eq!(rejected.last().frame.hint_status, HintStatus::Rejected);

    // A wrong hint inside the ambiguity group cannot be rejected from the
    // vertical component: it acts like a 180-degree yaw and is only caught by
    // the plausible-yaw bound. Documented limitation.
    let inside = frame_candidate_index(Quat::from_rotation_y(PI) * truth).unwrap();
    let flipped = simulate(&scenario, AccelHeadingConfig::default(), Some(inside), 0.5);
    assert_eq!(flipped.candidates().count(), 0);
    assert_eq!(
        flipped.last().status,
        AccelHeadingStatus::ExcessiveCorrection
    );
}

#[test]
fn a_frame_identified_on_a_swinging_thigh_resolves_the_upright_chest() {
    let thigh = SyntheticScenario {
        sensor: SyntheticSensor::Thigh,
        motion: SyntheticMotion::Walk,
        torso_tilt_degrees: 0.0,
        ..upright_walk()
    };
    let leg = simulate(&thigh, leg_config(), None, 0.5);
    let report = leg.last();
    // The kinematic reference was built from the drifting orientation, so its
    // horizontal part carries no independent heading: frame only.
    assert_eq!(
        report.status,
        AccelHeadingStatus::FrameIdentifiedOnly,
        "{report:?}"
    );
    assert!(report.yaw_correction_radians.is_none() && report.horizontal.is_none());
    let identified = report.frame.candidate.unwrap();
    assert_eq!(identified, thigh.true_frame);

    let chest = simulate(
        &upright_walk(),
        AccelHeadingConfig::default(),
        Some(identified),
        0.5,
    );
    assert_eq!(chest.last().status, AccelHeadingStatus::Candidate);
    assert!(degrees_error(chest.last(), chest.run.expected_correction_radians).abs() < 3.0);
}

#[test]
fn recovers_plus_and_minus_25_degrees_within_3_degrees_under_lag_and_noise() {
    for (yaw_error, seed) in [(25.0, 1), (-25.0, 2), (25.0, 3), (-25.0, 4)] {
        let scenario = SyntheticScenario {
            yaw_error_degrees: yaw_error,
            imu_lag_secs: 0.02,
            accel_noise_mps2: 0.3,
            seed,
            ..tilted_dance()
        };
        let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
        assert_eq!(
            run.last().status,
            AccelHeadingStatus::Candidate,
            "{:?}",
            run.last()
        );
        assert!(run.candidates().count() >= 25);
        for candidate in run.candidates() {
            let error = degrees_error(candidate, run.run.expected_correction_radians);
            assert!(
                error.abs() < 3.0,
                "yaw {yaw_error} seed {seed}: error {error}"
            );
            let sigma = candidate.yaw_sigma_radians.unwrap().to_degrees();
            assert!(sigma > 0.5 && sigma < 6.0);
        }
    }
}

#[test]
fn lag_estimate_follows_the_true_skew() {
    for lag in [0.0, 0.02, 0.05] {
        let scenario = SyntheticScenario {
            imu_lag_secs: lag,
            ..tilted_dance()
        };
        let run = simulate(&scenario, AccelHeadingConfig::default(), None, 1.0);
        let estimated = run.last().lag_secs.unwrap();
        assert!(
            (estimated - run.run.nominal_lag_secs).abs() < 0.008,
            "estimated {estimated}, nominal {}",
            run.run.nominal_lag_secs
        );
    }
}

#[test]
fn standing_still_reports_low_excitation_and_no_heading() {
    let scenario = SyntheticScenario {
        motion: SyntheticMotion::Still,
        torso_tilt_degrees: 1.0,
        head_rotation_degrees: 3.0,
        ..tilted_dance()
    };
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
    assert_eq!(run.candidates().count(), 0);
    assert!(run.reports.iter().all(|r| r.frame.candidate.is_none()));
    assert_eq!(run.last().status, AccelHeadingStatus::LowExcitation);
}

#[test]
fn torso_moving_independently_of_the_head_never_yields_a_candidate() {
    let scenario = SyntheticScenario {
        motion: SyntheticMotion::IndependentTorso,
        ..tilted_dance()
    };
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
    assert_eq!(run.candidates().count(), 0);
    assert!(run.reports.iter().all(|r| r.frame.candidate.is_none()));
    assert_eq!(run.last().status, AccelHeadingStatus::FrameUnidentified);
}

#[test]
fn independent_horizontal_motion_reports_poor_correlation_with_large_sigma() {
    let scenario = SyntheticScenario {
        motion: SyntheticMotion::IndependentHorizontal,
        ..tilted_dance()
    };
    let run = simulate(
        &scenario,
        AccelHeadingConfig::default(),
        Some(scenario.true_frame),
        0.5,
    );
    assert_eq!(run.candidates().count(), 0);
    let report = run.last();
    // The shared vertical motion verifies the frame; the horizontal does not agree.
    assert_eq!(report.frame.hint_status, HintStatus::Verified);
    assert_eq!(
        report.status,
        AccelHeadingStatus::PoorCorrelation,
        "{report:?}"
    );
    let fit = report.horizontal.as_ref().unwrap();
    assert!(fit.correlation < 0.3);
    assert!(fit.sigma_radians.to_degrees() > 20.0);
}

#[test]
fn constant_accelerometer_bias_does_not_change_the_estimate() {
    let clean = simulate(&tilted_dance(), AccelHeadingConfig::default(), None, 1.0);
    let biased_scenario = SyntheticScenario {
        accel_bias_mps2: [0.8, -0.5, 0.6],
        ..tilted_dance()
    };
    let biased = simulate(&biased_scenario, AccelHeadingConfig::default(), None, 1.0);
    let (a, b) = (clean.last(), biased.last());
    assert_eq!(a.status, AccelHeadingStatus::Candidate);
    assert_eq!(b.status, AccelHeadingStatus::Candidate);
    assert_eq!(a.frame.candidate, b.frame.candidate);
    let difference =
        (a.yaw_correction_radians.unwrap() - b.yaw_correction_radians.unwrap()).to_degrees();
    assert!(
        difference.abs() < 0.3,
        "bias moved the yaw by {difference} degrees"
    );
}

#[test]
fn head_rotation_with_a_neck_reference_stays_within_3_degrees() {
    let scenario = SyntheticScenario {
        head_rotation_degrees: 15.0,
        head_rotation_hz: 0.5,
        reference: SyntheticReference::NeckModel {
            offset_error_m: [0.0, 0.02, 0.03],
        },
        ..tilted_dance()
    };
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
    assert_eq!(run.last().status, AccelHeadingStatus::Candidate);
    for candidate in run.candidates() {
        assert!(degrees_error(candidate, run.run.expected_correction_radians).abs() < 3.0);
    }
}

#[test]
fn fast_head_rotation_is_gated_rather_than_fitted() {
    let scenario = SyntheticScenario {
        head_rotation_degrees: 30.0,
        head_rotation_hz: 0.8,
        reference: SyntheticReference::HeadsetEye,
        ..tilted_dance()
    };
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 0.5);
    assert_eq!(run.candidates().count(), 0);
    let report = run.last();
    assert_eq!(report.status, AccelHeadingStatus::InsufficientData);
    assert!(report.gates.reference_rotation > report.gates.window_samples / 2);
}

#[test]
fn position_only_reference_is_differentiated_twice() {
    let scenario = SyntheticScenario {
        provide_reference_velocity: false,
        ..tilted_dance()
    };
    let run = simulate(&scenario, AccelHeadingConfig::default(), None, 1.0);
    assert_eq!(run.last().status, AccelHeadingStatus::Candidate);
    assert!(degrees_error(run.last(), run.run.expected_correction_radians).abs() < 3.0);
}

#[test]
fn malformed_input_is_rejected_without_panics_or_nonfinite_output() {
    let mut estimator = AccelHeadingEstimator::new(AccelHeadingConfig::default()).unwrap();
    let nan = f32::NAN;
    let inf = f32::INFINITY;
    let reject = SampleOutcome::Rejected;
    assert_eq!(
        estimator.push_reference(f64::NAN, Some(Vec3::ZERO), None, 0.0),
        reject
    );
    assert_eq!(
        estimator.push_reference(-1.0, Some(Vec3::ZERO), None, 0.0),
        reject
    );
    assert_eq!(estimator.push_reference(0.0, None, None, 0.0), reject);
    assert_eq!(
        estimator.push_reference(0.0, Some(Vec3::splat(nan)), None, 0.0),
        reject
    );
    assert_eq!(
        estimator.push_reference(0.0, Some(Vec3::ZERO), Some(Vec3::splat(inf)), 0.0),
        reject
    );
    assert_eq!(
        estimator.push_reference(0.0, Some(Vec3::ZERO), None, nan),
        reject
    );
    assert_eq!(
        estimator.push_reference(0.0, Some(Vec3::ZERO), None, -1.0),
        reject
    );
    assert_eq!(
        estimator.push_imu(f64::INFINITY, Quat::IDENTITY, Vec3::ZERO, None),
        reject
    );
    assert_eq!(
        estimator.push_imu(0.0, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0), Vec3::ZERO, None),
        reject
    );
    assert_eq!(
        estimator.push_imu(0.0, Quat::from_xyzw(nan, 0.0, 0.0, 1.0), Vec3::ZERO, None),
        reject
    );
    assert_eq!(
        estimator.push_imu(0.0, Quat::IDENTITY, Vec3::new(0.0, nan, 0.0), None),
        reject
    );
    assert_eq!(
        estimator.push_imu(0.0, Quat::IDENTITY, Vec3::splat(1.0e9), None),
        reject
    );
    assert_eq!(
        estimator.push_imu(0.0, Quat::IDENTITY, Vec3::ZERO, Some(nan)),
        reject
    );
    assert_eq!(estimator.buffered_samples(), [0, 0]);
    assert!(
        estimator
            .set_frame_hint(Some(FRAME_CANDIDATE_COUNT))
            .is_err()
    );

    for now in [f64::NAN, f64::INFINITY, -1.0, 0.0, 1.0e300] {
        let report = estimator.report(now);
        assert_eq!(report.status, AccelHeadingStatus::InsufficientData);
        assert_finite(&report);
    }
    assert_eq!(estimator.report(0.0).rejected_inputs, 13);

    // Unnormalised but valid quaternions and implausible (but finite)
    // accelerations are accepted and handled downstream.
    assert_eq!(
        estimator.push_imu(
            0.0,
            Quat::from_xyzw(0.0, 0.0, 0.0, 1.1),
            Vec3::splat(40.0),
            None
        ),
        SampleOutcome::Accepted
    );

    // A run whose samples are interleaved with garbage still reports finitely.
    let run = generate(&tilted_dance());
    let mut estimator = AccelHeadingEstimator::new(AccelHeadingConfig::default()).unwrap();
    for (i, event) in run.events.iter().enumerate() {
        event.push_into(&mut estimator);
        if i % 7 == 0 {
            estimator.push_imu(
                event.at_secs(),
                Quat::from_xyzw(nan, nan, nan, nan),
                Vec3::ZERO,
                None,
            );
            estimator.push_reference(event.at_secs(), Some(Vec3::splat(inf)), None, 0.0);
        }
        if i % 400 == 0 {
            assert_finite(&estimator.report(event.at_secs()));
        }
    }
    let report = estimator.report(run.events.last().unwrap().at_secs());
    assert_finite(&report);
    assert_eq!(report.status, AccelHeadingStatus::Candidate);
}

#[test]
fn time_going_backwards_and_gaps_are_handled() {
    let run = generate(&tilted_dance());
    let mut estimator = AccelHeadingEstimator::new(AccelHeadingConfig::default()).unwrap();
    let mut saw_candidate_before_gap = false;
    let mut after_gap = Vec::new();
    for event in &run.events {
        let t = event.at_secs();
        // A 1.5 s IMU outage between 10 and 11.5 s.
        if matches!(event, SyntheticEvent::Imu { .. }) && (10.0..11.5).contains(&t) {
            continue;
        }
        event.push_into(&mut estimator);
        if (9.5..10.0).contains(&t) {
            saw_candidate_before_gap |= estimator.report(t).status == AccelHeadingStatus::Candidate;
        }
        if t >= 10.6 && (t * 10.0).fract() < 0.05 {
            after_gap.push((t, estimator.report(t)));
        }
    }
    assert!(saw_candidate_before_gap);
    // During the outage the IMU stream is stale: no candidate.
    for (t, report) in after_gap.iter().filter(|(t, _)| *t < 11.5) {
        assert_eq!(
            report.status,
            AccelHeadingStatus::InsufficientData,
            "at {t}"
        );
    }
    // Afterwards the outage plus a settle interval is excluded from the
    // window (older valid data inside the window still counts), and every
    // candidate stays correct.
    for (t, report) in after_gap.iter().filter(|(t, _)| (11.6..16.0).contains(t)) {
        assert!(
            report.valid_samples + 150 <= report.gates.window_samples,
            "at {t}: {} valid",
            report.valid_samples
        );
    }
    for (_, report) in after_gap
        .iter()
        .filter(|(_, r)| r.status == AccelHeadingStatus::Candidate)
    {
        assert!(degrees_error(report, run.expected_correction_radians).abs() < 3.0);
    }
    let last = &after_gap.last().unwrap().1;
    assert_eq!(last.status, AccelHeadingStatus::Candidate);

    // Evaluating long after the last sample is stale.
    let end = run.events.last().unwrap().at_secs();
    assert_eq!(
        estimator.report(end + 2.0).status,
        AccelHeadingStatus::InsufficientData
    );

    // Slightly older samples are rejected; a large backwards jump restarts.
    let rejected = estimator.report(end).rejected_inputs;
    assert_eq!(
        estimator.push_reference(end - 0.5, Some(Vec3::ZERO), None, 0.0),
        SampleOutcome::Rejected
    );
    assert_eq!(estimator.report(end).rejected_inputs, rejected + 1);
    let frame = estimator.latched_frame();
    assert!(frame.is_some());
    assert_eq!(
        estimator.push_imu(1.0, Quat::IDENTITY, Vec3::ZERO, None),
        SampleOutcome::ClockRestarted
    );
    assert_eq!(estimator.buffered_samples(), [0, 1]);
    // The hardware frame survives a clock restart; the data does not.
    assert_eq!(estimator.latched_frame(), frame);
    let report = estimator.report(1.0);
    assert_eq!(report.status, AccelHeadingStatus::InsufficientData);
    assert_eq!(report.clock_restarts, 1);
    estimator.reset();
    assert_eq!(estimator.latched_frame(), None);
    assert_eq!(estimator.buffered_samples(), [0, 0]);
}

#[test]
fn memory_is_bounded_by_the_ring_capacities() {
    let config = AccelHeadingConfig {
        reference_capacity: 256,
        imu_capacity: 128,
        ..AccelHeadingConfig::default()
    };
    let mut estimator = AccelHeadingEstimator::new(config).unwrap();
    assert_eq!(estimator.buffer_capacity(), [256, 128]);
    for i in 0..200_000 {
        let t = i as f64 * 0.001;
        let p = Vec3::new((t as f32).sin(), 1.6, 0.0);
        estimator.push_reference(t, Some(p), None, 0.1);
        estimator.push_imu(
            t,
            Quat::from_rotation_y(t as f32),
            Vec3::new(0.0, (5.0 * t as f32).sin(), 0.0),
            Some(0.2),
        );
        let [reference, imu] = estimator.buffered_samples();
        assert!(reference <= 256 && imu <= 128);
    }
    let report = estimator.report(200.0);
    assert_finite(&report);
    // 128 IMU samples at 1 kHz cover far less than a window.
    assert_eq!(report.status, AccelHeadingStatus::InsufficientData);

    // With the default capacities, pushing long runs keeps only the history span.
    let mut estimator = AccelHeadingEstimator::new(AccelHeadingConfig::default()).unwrap();
    for i in 0..60_000 {
        let t = i as f64 * 0.005;
        estimator.push_reference(t, Some(Vec3::Y), None, 0.0);
        if i % 2 == 0 {
            estimator.push_imu(t, Quat::IDENTITY, Vec3::ZERO, None);
        }
    }
    let [reference, imu] = estimator.buffered_samples();
    assert!(reference < 2_000 && imu < 1_000, "{reference} {imu}");
}

#[test]
fn repeated_reports_are_identical() {
    let run = generate(&tilted_dance());
    let mut estimator = AccelHeadingEstimator::new(AccelHeadingConfig::default()).unwrap();
    for event in &run.events {
        event.push_into(&mut estimator);
    }
    let now = run.events.last().unwrap().at_secs();
    let first = serde_json::to_string(&estimator.report(now)).unwrap();
    let second = serde_json::to_string(&estimator.report(now)).unwrap();
    assert_eq!(first, second);
}

#[test]
fn config_round_trips_and_invalid_values_are_rejected() {
    let config = AccelHeadingConfig::default();
    let json = serde_json::to_string(&config).unwrap();
    let back: AccelHeadingConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(back, config);
    let partial: AccelHeadingConfig = serde_json::from_str(r#"{"window_secs": 4.0}"#).unwrap();
    assert_eq!(partial.window_secs, 4.0);
    assert!(AccelHeadingEstimator::new(partial).is_ok());
    for invalid in [
        AccelHeadingConfig {
            window_secs: f64::NAN,
            ..config.clone()
        },
        AccelHeadingConfig {
            lowpass_hz: 0.4,
            ..config.clone()
        },
        AccelHeadingConfig {
            min_lag_secs: 0.1,
            max_lag_secs: 0.0,
            ..config.clone()
        },
        AccelHeadingConfig {
            allowed_frame_candidates: vec![3, 3],
            ..config.clone()
        },
        AccelHeadingConfig {
            allowed_frame_candidates: vec![24],
            ..config.clone()
        },
        AccelHeadingConfig {
            imu_capacity: 1_000_000,
            ..config.clone()
        },
    ] {
        assert!(AccelHeadingEstimator::new(invalid).is_err());
    }
}

#[test]
fn restricting_to_rotations_about_the_chip_z_axis_identifies_an_upright_chest() {
    let about_z = frame_candidates_fixing_axis(Vec3::Z);
    assert_eq!(about_z.len(), 4);
    assert!(about_z.contains(&0));
    assert!(frame_candidates_fixing_axis(Vec3::ZERO).is_empty());
    // Board flat on the chest: chip Z is horizontal while standing, so the
    // vertical ambiguity group (rotations about sensor Y) meets this subset
    // only in the true frame.
    let true_frame = frame_candidate_index(Quat::from_rotation_z(-FRAC_PI_2)).unwrap();
    assert!(about_z.contains(&true_frame));
    let scenario = SyntheticScenario {
        true_frame,
        ..upright_walk()
    };
    let config = AccelHeadingConfig {
        allowed_frame_candidates: about_z,
        ..AccelHeadingConfig::default()
    };
    let run = simulate(&scenario, config, None, 0.5);
    let report = run.last();
    assert_eq!(report.status, AccelHeadingStatus::Candidate, "{report:?}");
    assert_eq!(report.frame.candidate, Some(true_frame));
    assert_eq!(report.frame.ambiguous_candidates, vec![true_frame]);
    assert!(degrees_error(report, run.run.expected_correction_radians).abs() < 3.0);
}

#[test]
fn restricting_candidates_to_a_known_frame_only_verifies_it() {
    // Firmware >= 0.7.4 rotates the acceleration itself: C is the identity.
    let scenario = SyntheticScenario {
        true_frame: 0,
        ..upright_walk()
    };
    let config = AccelHeadingConfig {
        allowed_frame_candidates: vec![0],
        ..AccelHeadingConfig::default()
    };
    let run = simulate(&scenario, config.clone(), None, 0.5);
    assert_eq!(run.last().status, AccelHeadingStatus::Candidate);
    assert!(degrees_error(run.last(), run.run.expected_correction_radians).abs() < 3.0);

    // The same restriction on data from a frame with a different vertical
    // axis finds no agreement.
    let other = (1..FRAME_CANDIDATE_COUNT)
        .find(|c| !same_up_row(0).contains(c))
        .unwrap();
    let wrong = SyntheticScenario {
        true_frame: other,
        ..upright_walk()
    };
    let run = simulate(&wrong, config, None, 0.5);
    assert_eq!(run.candidates().count(), 0);
    assert_eq!(run.last().status, AccelHeadingStatus::FrameUnidentified);
}
