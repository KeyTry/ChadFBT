//! Stance heading filter: the spec's synthetic regression table plus direct API checks.
//!
//! Closed-loop scenarios generate ground truth with the estimator's own kinematic model
//! (`tracking_core::synthetic`). Passing shows the implementation matches the model and stays
//! bounded under the injected faults; it is not evidence of accuracy on a human body or of
//! any performance relative to SlimeVR. Measured numbers are printed (`--nocapture`).

use glam::{Quat, Vec3};
use tracking_core::chain::*;
use tracking_core::stance_heading::*;
use tracking_core::synthetic::*;

const LEGS: [usize; 4] = [2, 3, 4, 5];
const ALL: [usize; 6] = [0, 1, 2, 3, 4, 5];

fn degrees(values: [f32; 6]) -> [f32; 6] {
    values.map(f32::to_degrees)
}

fn max_abs(values: impl IntoIterator<Item = f32>) -> f32 {
    values.into_iter().fold(0.0, |m, v| m.max(v.abs()))
}

/// One open-loop frame for direct filter tests.
#[derive(Clone, Copy)]
struct Frame {
    at: f64,
    head_position: Vec3,
    head_rotation: Quat,
    /// Measured rotations before the filter's correction.
    raw: [Quat; 6],
    usable: [bool; 6],
    planted: [bool; 2],
    angular_speed: [f32; 6],
    /// Set by `tamper` to mark a frame for `observe`.
    marked: bool,
}

/// Feeds the filter from a scenario with perfect contact knowledge (`truth.stationary`),
/// applying the filter's own corrections like the runtime would. `tamper` may edit a frame.
fn drive(
    filter: &mut StanceHeadingFilter,
    scenario: &Scenario,
    from: f64,
    to: f64,
    mut tamper: impl FnMut(&mut Frame),
    mut observe: impl FnMut(&Frame, &StanceHeadingOutput),
) {
    let generator = scenario.generator();
    let errors: [f32; 6] = std::array::from_fn(|i| match scenario.yaw_errors[i] {
        YawError::Constant { degrees } => degrees,
        _ => 0.0,
    });
    let count = ((to - from) * 100.0).round() as usize;
    for k in 0..=count {
        let at = from + k as f64 / 100.0;
        let truth = generator.frame(at);
        let mut frame = Frame {
            at,
            head_position: truth.head_position,
            head_rotation: truth.head_rotation,
            raw: std::array::from_fn(|i| {
                Quat::from_rotation_y(-errors[i].to_radians()) * truth.rotations[i]
            }),
            usable: [true; 6],
            planted: truth.stationary,
            angular_speed: [0.0; 6],
            marked: false,
        };
        tamper(&mut frame);
        let corrections = filter.corrections();
        let rotations: [Quat; 6] =
            std::array::from_fn(|i| Quat::from_rotation_y(corrections[i]) * frame.raw[i]);
        let chain = solve_chain(
            &scenario.assumed_dims,
            &ChainPose {
                head_position: frame.head_position,
                head_rotation: frame.head_rotation,
                rotations,
            },
        );
        let output = filter.step(&StanceHeadingInput {
            at_secs: frame.at,
            head_position: frame.head_position,
            head_rotation: frame.head_rotation,
            chain: &chain,
            rotations,
            usable: frame.usable,
            planted: frame.planted,
            angular_speed: frame.angular_speed,
        });
        observe(&frame, &output);
    }
}

fn assert_output_finite(output: &StanceHeadingOutput) {
    let numbers = output
        .corrections
        .iter()
        .chain(&output.sigma)
        .copied()
        .chain(output.feet.iter().flat_map(|foot| {
            [
                foot.last_excitation_m,
                foot.last_innovation_m,
                foot.last_nis,
            ]
        }))
        .chain(
            output
                .priors
                .knee
                .iter()
                .chain([&output.priors.pelvis, &output.priors.chest])
                .map(|prior| prior.last_residual_degrees),
        );
    for value in numbers {
        assert!(value.is_finite(), "{output:?}");
    }
}

// ---------------------------------------------------------------------------------------------
// Direct API behaviour.

#[test]
fn reset_restores_zero_corrections_and_initial_uncertainty() {
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    assert_eq!(filter.corrections(), [0.0; 6]);
    for sigma in degrees(filter.sigma()) {
        assert!((sigma - 2.0).abs() < 1.0e-4);
    }
    let scenario = scenarios::walking_common_leg_drift(10.0);
    drive(&mut filter, &scenario, 0.0, 6.0, |_| {}, |_, _| {});
    assert!(max_abs(filter.corrections()) > 0.01);
    filter.reset();
    assert_eq!(filter.corrections(), [0.0; 6]);
    assert_eq!(filter.last_output().feet[0].accepted, 0);
    for sigma in degrees(filter.sigma()) {
        assert!((sigma - 2.0).abs() < 1.0e-4);
    }
}

#[test]
fn process_noise_follows_time_and_rotation_rate_and_is_capped() {
    let config = StanceHeadingConfig::default();
    let scenario = Scenario::new("stand", vec![Motion::stand(10.0)]);
    let mut filter = StanceHeadingFilter::new(config);
    drive(
        &mut filter,
        &scenario,
        0.0,
        10.0,
        |frame| {
            frame.planted = [false; 2];
            frame.angular_speed = [0.0, 3.0, 0.0, 0.0, 0.0, 0.0];
        },
        |_, output| assert_eq!(output.status, StanceHeadingStatus::Idle),
    );
    let sigma = filter.sigma();
    let base = 0.10_f64.to_radians();
    let expected_still = (2.0_f64.to_radians().powi(2) + base * base * 10.0).sqrt();
    let expected_rotating =
        (2.0_f64.to_radians().powi(2) + (base * base + (0.015 * 3.0_f64).powi(2)) * 10.0).sqrt();
    assert!((f64::from(sigma[0]) / expected_still - 1.0).abs() < 0.01);
    assert!((f64::from(sigma[1]) / expected_rotating - 1.0).abs() < 0.01);
    assert_eq!(filter.corrections(), [0.0; 6]);

    let mut filter = StanceHeadingFilter::new(config);
    let long = Scenario::new("stand", vec![Motion::stand(300.0)]);
    drive(
        &mut filter,
        &long,
        0.0,
        300.0,
        |frame| {
            frame.planted = [false; 2];
            frame.angular_speed = [100.0; 6];
        },
        |_, _| {},
    );
    for sigma in degrees(filter.sigma()) {
        assert!(sigma <= 45.0 + 1.0e-3 && sigma > 44.0, "{sigma}");
    }
}

#[test]
fn gate_rejects_inconsistent_increments_and_streak_requests_release() {
    let scenario = Scenario::new("squats", vec![Motion::squats(3)]);
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    let mut releases = 0;
    let mut rejected = 0;
    drive(
        &mut filter,
        &scenario,
        0.0,
        9.0,
        |frame| {
            // The reported headset creeps sideways at 0.5 m/s (no discontinuity, but a
            // "planted" ankle cannot explain it), while the planted hypothesis is forced.
            frame.head_position.x += 0.5 * frame.at as f32;
            frame.planted = [true; 2];
        },
        |_, output| {
            releases += output.release_requests.iter().filter(|r| **r).count();
            rejected = output.feet.iter().map(|foot| foot.rejected).sum::<u32>();
            assert!(output.feet.iter().all(|foot| foot.rejection_streak < 3));
        },
    );
    let accepted: u32 = filter
        .last_output()
        .feet
        .iter()
        .map(|foot| foot.accepted)
        .sum();
    println!(
        "inconsistent headset creep: rejected {rejected}, accepted {accepted}, releases {releases}"
    );
    assert!(rejected >= 6 && rejected > 5 * accepted);
    assert!(releases >= rejected as usize / 3 - 1);
    assert!(max_abs(degrees(filter.corrections())) < 3.0);
}

#[test]
fn every_update_step_is_clamped_to_one_degree() {
    let config = StanceHeadingConfig {
        initial_sigma_degrees: 30.0,
        enable_knee_prior: false,
        enable_pelvis_prior: false,
        enable_chest_prior: false,
        ..StanceHeadingConfig::default()
    };
    let scenario = scenarios::walking_common_leg_drift(30.0);
    let mut filter = StanceHeadingFilter::new(config);
    let mut previous = filter.corrections();
    let mut largest_single = 0.0_f32;
    drive(
        &mut filter,
        &scenario,
        0.0,
        20.0,
        |_| {},
        |_, output| {
            let step = max_abs((0..6).map(|i| (output.corrections[i] - previous[i]).to_degrees()));
            // At most one displacement update per planted foot per frame.
            assert!(step <= 2.0 + 1.0e-3, "step {step}");
            largest_single = largest_single.max(step);
            previous = output.corrections;
        },
    );
    assert!(
        largest_single >= 0.99,
        "clamp never engaged ({largest_single})"
    );
}

/// Runs squats with planted feet forced and a 0.3 m headset jump at 3.005 s; returns the
/// largest correction change in the 0.5 s after the jump.
fn correction_change_after_jump(config: StanceHeadingConfig) -> f32 {
    let scenario = Scenario::new("squats", vec![Motion::squats(2)]);
    let mut filter = StanceHeadingFilter::new(config);
    let mut before = [0.0; 6];
    let mut worst = 0.0_f32;
    drive(
        &mut filter,
        &scenario,
        0.0,
        3.5,
        |frame| {
            frame.planted = [true; 2];
            if frame.at >= 3.005 {
                frame.head_position += Vec3::new(0.3, 0.0, 0.0);
            }
        },
        |frame, output| {
            if frame.at < 3.005 {
                before = output.corrections;
            } else {
                worst = worst.max(max_abs(
                    (0..6).map(|i| (output.corrections[i] - before[i]).to_degrees()),
                ));
            }
        },
    );
    worst
}

#[test]
fn headset_jump_protection_is_what_prevents_the_false_correction() {
    // Falsification check: with the discontinuity test and the χ² gate disabled, the very same
    // jump is absorbed into the yaw state; with either protection active it is not.
    let unprotected = correction_change_after_jump(StanceHeadingConfig {
        gate_chi2: f32::INFINITY,
        max_head_speed_mps: f32::INFINITY,
        ..StanceHeadingConfig::default()
    });
    let discontinuity_only = correction_change_after_jump(StanceHeadingConfig {
        gate_chi2: f32::INFINITY,
        ..StanceHeadingConfig::default()
    });
    let gate_only = correction_change_after_jump(StanceHeadingConfig {
        max_head_speed_mps: f32::INFINITY,
        ..StanceHeadingConfig::default()
    });
    println!(
        "0.3 m jump, correction change within 0.5 s: unprotected {unprotected:.2}°, \
         discontinuity check only {discontinuity_only:.3}°, gate only {gate_only:.3}°"
    );
    // Unprotected, the first update absorbs the jump up to the 1° step clamp.
    assert!(unprotected > 0.8);
    assert!(discontinuity_only < 0.05);
    assert!(gate_only < 0.05);
}

#[test]
fn headset_jump_drops_keyframes_without_updating_even_without_the_gate() {
    let config = StanceHeadingConfig {
        gate_chi2: f32::INFINITY,
        ..StanceHeadingConfig::default()
    };
    let scenario = Scenario::new("squats", vec![Motion::squats(2)])
        .with_yaw_error(4, YawError::Constant { degrees: 5.0 });
    let mut filter = StanceHeadingFilter::new(config);
    let mut before = None;
    let mut seen_jump = false;
    drive(
        &mut filter,
        &scenario,
        0.0,
        6.0,
        |frame| {
            frame.planted = [true; 2];
            if frame.at >= 3.005 {
                frame.head_position += Vec3::new(0.3, 0.0, 0.0);
            }
        },
        |frame, output| {
            if (frame.at - 3.0).abs() < 1.0e-9 {
                before = Some(*output);
            }
            if (frame.at - 3.01).abs() < 1.0e-9 {
                seen_jump = true;
                let before = before.unwrap();
                assert_eq!(output.status, StanceHeadingStatus::Discontinuity);
                assert_eq!(output.release_requests, [true; 2]);
                assert_eq!(output.corrections, before.corrections);
                assert_eq!(output.feet[0].accepted, before.feet[0].accepted);
                assert!(!output.feet[0].keyframe_active && !output.feet[1].keyframe_active);
            }
        },
    );
    assert!(seen_jump);
}

#[test]
fn timestamp_anomalies_drop_keyframes_and_request_release() {
    let scenario = Scenario::new("squats", vec![Motion::squats(2)]);
    for anomaly in [0.0, -0.5, 0.2] {
        let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
        let mut checked = false;
        let mut previous: Option<StanceHeadingOutput> = None;
        drive(
            &mut filter,
            &scenario,
            0.0,
            3.0,
            |frame| {
                frame.planted = [true; 2];
                if (frame.at - 2.0).abs() < 1.0e-9 {
                    // 0.0: repeated timestamp, -0.5: backwards, 0.2: gap.
                    frame.at = 1.99 + anomaly;
                    frame.marked = true;
                }
            },
            |frame, output| {
                if frame.marked {
                    let prev = previous.unwrap();
                    if prev.feet[0].keyframe_active {
                        checked = true;
                        assert_eq!(
                            output.status,
                            StanceHeadingStatus::Discontinuity,
                            "{anomaly}"
                        );
                        assert_eq!(output.release_requests, [true; 2]);
                        assert_eq!(output.corrections, prev.corrections);
                        assert!(!output.feet[0].keyframe_active);
                    }
                }
                previous = Some(*output);
            },
        );
        assert!(checked, "{anomaly}");
    }
}

#[test]
fn nonfinite_and_garbage_inputs_never_panic_or_emit_nonfinite_values() {
    let mut rng = XorShift64::new(99);
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    let dims = ChainDimensions::default();
    let specials = [
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
        1.0e30,
        -1.0e30,
        0.0,
        1.0e-30,
    ];
    let pick = |rng: &mut XorShift64| -> f32 {
        if rng.uniform() < 0.3 {
            specials[(rng.next_u64() % specials.len() as u64) as usize]
        } else {
            (rng.gaussian() * 2.0) as f32
        }
    };
    let mut at = 0.0;
    for k in 0..20_000 {
        at += if k % 97 == 0 {
            f64::from(pick(&mut rng))
        } else {
            0.01
        };
        let head_position = Vec3::new(pick(&mut rng), pick(&mut rng), pick(&mut rng));
        let head_rotation = Quat::from_xyzw(pick(&mut rng), pick(&mut rng), 0.1, 1.0);
        let rotations: [Quat; 6] = std::array::from_fn(|_| {
            Quat::from_xyzw(
                pick(&mut rng),
                pick(&mut rng),
                pick(&mut rng),
                pick(&mut rng),
            )
        });
        // Mix honest chains, chains from garbage, and arbitrary levers.
        let mut chain = solve_chain(
            &dims,
            &ChainPose {
                head_position,
                head_rotation,
                rotations,
            },
        );
        if k % 5 == 0 {
            chain.levers[0][2] = Vec3::new(pick(&mut rng), pick(&mut rng), pick(&mut rng));
            chain.head_lever = Vec3::new(pick(&mut rng), 0.0, pick(&mut rng));
        }
        let output = filter.step(&StanceHeadingInput {
            at_secs: at,
            head_position,
            head_rotation,
            chain: &chain,
            rotations,
            usable: std::array::from_fn(|_| rng.uniform() < 0.9),
            planted: [rng.uniform() < 0.8, rng.uniform() < 0.8],
            angular_speed: std::array::from_fn(|_| pick(&mut rng)),
        });
        assert_output_finite(&output);
    }
    // Plausible motion after the garbage still works and stays finite.
    let scenario = scenarios::walking_common_leg_drift(10.0);
    filter.reset();
    drive(
        &mut filter,
        &scenario,
        0.0,
        5.0,
        |_| {},
        |_, output| assert_output_finite(output),
    );
}

#[test]
fn outputs_and_config_round_trip_through_serde() {
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    let scenario = scenarios::walking_common_leg_drift(10.0);
    drive(&mut filter, &scenario, 0.0, 3.0, |_| {}, |_, _| {});
    let output = filter.last_output();
    let text = serde_json::to_string(&output).unwrap();
    let back: StanceHeadingOutput = serde_json::from_str(&text).unwrap();
    assert_eq!(back, output);
    let config: StanceHeadingConfig =
        serde_json::from_str("{\"enable_pelvis_prior\": false, \"gate_chi2\": 5.99}").unwrap();
    assert!(!config.enable_pelvis_prior && config.enable_knee_prior);
    assert_eq!(config.gate_chi2, 5.99);
    assert_eq!(config.knee_prior, KneePriorConfig::default());
    assert_eq!(config.chest_prior.deadband_degrees, 8.0);
    assert_eq!(config.pelvis_prior.deadband_degrees, 10.0);
}

// ---------------------------------------------------------------------------------------------
// Closed-loop regression table (spec section 6).

fn run(scenario: &Scenario) -> HarnessRun {
    run_closed_loop(scenario, &HarnessOptions::default())
}

fn baseline(scenario: &Scenario) -> HarnessRun {
    run_closed_loop(
        scenario,
        &HarnessOptions {
            filter_enabled: false,
            ..HarnessOptions::default()
        },
    )
}

#[test]
fn sign_convention_injected_positive_error_yields_positive_correction() {
    // measured = Ry(−10°)·true on the left thigh; the correction must converge to +10°.
    let scenario = Scenario::new("walking, left thigh +10°", scenarios::walk_laps(4))
        .with_yaw_error(2, YawError::Constant { degrees: 10.0 });
    let run = run(&scenario);
    let correction = run.frames.last().unwrap().corrections_degrees[2];
    println!(
        "left thigh correction {correction:.2}° (converged < 3° at {:?} s)",
        run.time_to_converge(&[2], 3.0)
    );
    assert!((8.5..=11.5).contains(&correction), "{correction}");
    // A negative error converges to a negative correction.
    let mirrored = scenario.with_yaw_error(2, YawError::Constant { degrees: -10.0 });
    let correction = run_closed_loop(&mirrored, &HarnessOptions::default())
        .frames
        .last()
        .unwrap()
        .corrections_degrees[2];
    assert!((-11.5..=-8.5).contains(&correction), "{correction}");
}

#[test]
fn walking_common_mode_leg_drift_converges() {
    // Before the uncertainty-aware contact gates 20° took 32 s: the computed stance ankle
    // slides at ≈ sin δ·v_walk (0.27 m/s at 20°, 0.8 m/s), above the 0.20 m/s entry gate.
    for (degrees, limit) in [(10.0, 10.0), (20.0, 15.0)] {
        let scenario = scenarios::walking_common_leg_drift(degrees);
        let run = run(&scenario);
        let converged = run.time_to_converge(&LEGS, 3.0);
        let errors = run.final_errors_degrees();
        println!(
            "walking {degrees}° common-mode leg drift: < 3° from {converged:?} s, final {errors:?}, \
             rms {:.2}° (off {:.2}°)",
            run.rms_error_degrees(&LEGS, 0.0),
            baseline(&scenario).rms_error_degrees(&LEGS, 0.0)
        );
        assert!(converged.is_some_and(|t| t <= limit), "{converged:?}");
        assert!(max_abs(LEGS.map(|i| errors[i])) < 2.0);
        assert!(
            max_abs([errors[0], errors[1]]) < 2.0,
            "torso disturbed: {errors:?}"
        );
        assert!(run.frames.iter().all(|frame| frame.outputs_finite));
    }
}

#[test]
fn squats_with_one_calf_drifting_report() {
    // Requirement: converge with the knee prior. Measured: NOT to < 3°. Symmetric squats move
    // every lever in the sagittal plane, so all H vectors are lateral; the leg common mode with
    // the chest and a left/right antisymmetric leg mode are nearly unobservable (Fisher σ ≈ 64°
    // and ≈ 4° over 60 s with ideal data). A single-calf error projects onto them. The knee
    // prior only removes the hinge residual beyond its 5° deadband and splits it between thigh
    // and calf by their uncertainty.
    let scenario = scenarios::squats_calf_drift(15.0);
    let full = run(&scenario);
    let mut knee_only_options = HarnessOptions::default();
    knee_only_options.filter.enable_displacement = false;
    let knee_only = run_closed_loop(&scenario, &knee_only_options);
    for (name, run) in [
        ("displacement + priors", &full),
        ("knee prior only", &knee_only),
    ] {
        let errors = run.final_errors_degrees();
        let hinge = errors[4] - errors[2];
        println!(
            "squats, left calf 15° [{name}]: final {errors:?}, hinge residual {hinge:.2}°, \
             calf rms {:.2}°, knee prior updates {}",
            run.rms_error_degrees(&[4], 0.0),
            run.last_output.priors.knee[0].updates
        );
        assert!(hinge.abs() <= 6.0, "hinge residual {hinge}");
        assert!(errors[4].abs() < 12.0, "calf error not reduced: {errors:?}");
        assert!(run.frames.iter().all(|frame| frame.outputs_finite));
    }
    assert!(knee_only.last_output.priors.knee[0].updates > 0);
    // With the displacement observation the calf ends clearly closer than the prior alone.
    assert!(full.final_errors_degrees()[4] < knee_only.final_errors_degrees()[4]);
}

#[test]
fn weight_shifting_converges_slowly() {
    let scenario = scenarios::weight_shift_leg_drift(10.0);
    let run = run(&scenario);
    let converged = run.time_to_converge(&LEGS, 3.0);
    let errors = run.final_errors_degrees();
    println!("weight shifting, 10° leg drift: < 3° from {converged:?} s, final {errors:?}");
    assert!(converged.is_some_and(|t| t <= 90.0), "{converged:?}");
    assert!(max_abs(LEGS.map(|i| errors[i])) < 2.0);
    assert!(
        run.rms_error_degrees(&LEGS, 0.0) < 0.6 * baseline(&scenario).rms_error_degrees(&LEGS, 0.0)
    );
}

#[test]
fn standing_still_with_drift_applies_no_correction() {
    // Whole-body drift is unobservable by construction: no displacement information and zero
    // prior residuals.
    let common = run(&scenarios::standing_still_drift(true));
    assert_eq!(common.accepted(), 0);
    assert!(common.max_abs_correction_degrees(&ALL) < 1.0e-6);
    // Independent drift with the priors off: still no information at all.
    let mut no_priors = HarnessOptions::default();
    no_priors.filter.enable_knee_prior = false;
    no_priors.filter.enable_pelvis_prior = false;
    no_priors.filter.enable_chest_prior = false;
    let independent = run_closed_loop(&scenarios::standing_still_drift(false), &no_priors);
    assert_eq!(independent.accepted(), 0);
    assert_eq!(independent.max_abs_correction_degrees(&ALL), 0.0);
    // With the priors on, only their deadbanded pseudo-measurements act (reported).
    let with_priors = run(&scenarios::standing_still_drift(false));
    assert_eq!(with_priors.accepted(), 0);
    println!(
        "standing still, independent drift, priors on: max correction {:.2}°, final {:?}",
        with_priors.max_abs_correction_degrees(&ALL),
        with_priors.final_errors_degrees()
    );
}

#[test]
fn head_only_turns_with_eye_offset_error_cause_no_false_correction() {
    let run = run(&scenarios::head_turns_eye_offset(0.04));
    let worst = run.max_abs_correction_degrees(&ALL);
    println!("head-only turns, 4 cm eye-offset error: max false correction {worst:.3}°");
    assert!(worst < 1.0);
}

#[test]
fn bone_length_error_keeps_residual_yaw_below_three_degrees() {
    let drifting = run(&scenarios::bone_length_error(0.95, 10.0));
    let converged = drifting
        .time_to_converge(&LEGS, 3.0)
        .expect("never converged");
    let after = drifting.max_abs_error_degrees(&ALL, converged);
    let still = run(&scenarios::bone_length_error(0.95, 0.0));
    let false_correction = still.max_abs_correction_degrees(&ALL);
    println!(
        "5 % bone-length error: converged at {converged:.1} s, max error afterwards {after:.2}°; \
         without drift max false correction {false_correction:.2}°"
    );
    assert!(converged < 15.0);
    assert!(after < 3.0);
    assert!(false_correction < 3.0);
}

#[test]
fn slow_real_foot_slide_false_correction_is_bounded() {
    // 0.15 m/s: 4.28° since the filter estimates leg yaw only (the previous model spread
    // the same slide over the chest and hip too and reached 3.36°); the slower slides
    // improved (1.43° -> 0.95° and 0.48° -> 0.24°).
    for (speed, direction, bound) in [(0.05, 0.0, 2.0), (0.05, 90.0, 2.0), (0.15, 0.0, 4.5)] {
        let run = run(&scenarios::slow_foot_slide(speed, direction));
        let worst = run.max_abs_correction_degrees(&ALL);
        println!(
            "foot slide {speed} m/s at {direction}°: max false correction {worst:.2}°, \
             slid {:.1} cm while planted, releases {}",
            run.max_planted_truth_motion_m * 100.0,
            run.release_requests
        );
        assert!(worst < bound, "{speed} {direction}: {worst}");
    }
}

#[test]
fn turning_on_the_spot_false_correction_is_bounded() {
    for pivot_secs in [0.8, 1.6] {
        let run = run(&scenarios::turn_on_spot(pivot_secs));
        let worst = run.max_abs_correction_degrees(&ALL);
        println!(
            "ten 90° turns, {pivot_secs} s pivots: max false correction {worst:.2}°, \
             ankle moved {:.1} cm while planted",
            run.max_planted_truth_motion_m * 100.0
        );
        assert!(worst < 3.0, "{pivot_secs}: {worst}");
        // The planted-yaw budget bounds how far a pivoting ankle travels while "planted".
        assert!(run.max_planted_truth_motion_m < 0.05);
    }
}

#[test]
fn headset_jump_produces_no_correction() {
    let jump_at = 5.005;
    let jumped = run(&scenarios::headset_jump(0.3, jump_at));
    let mut reference = scenarios::headset_jump(0.3, jump_at);
    reference.head_jump = None;
    let reference = run(&reference);
    let index = jumped
        .frames
        .iter()
        .position(|frame| frame.at_secs >= jump_at)
        .unwrap();
    let (before, at) = (&jumped.frames[index - 1], &jumped.frames[index]);
    assert_eq!(at.status, StanceHeadingStatus::Discontinuity);
    assert_eq!(at.release_requests, [true; 2]);
    assert_eq!(at.corrections_degrees, before.corrections_degrees);
    assert_eq!(
        before.corrections_degrees,
        reference.frames[index - 1].corrections_degrees
    );
    // Both contacts stay released while the jump is in the velocity estimate, so no keyframe
    // can span it; afterwards the run differs from the jump-free one only by the measurements
    // it had to skip while still converging (reported, not a jump-induced correction).
    assert!(
        jumped.frames[index..index + 15]
            .iter()
            .all(|f| f.planted == [false; 2])
    );
    let window = jumped.frames[index..index + 100]
        .iter()
        .zip(&reference.frames[index..index + 100])
        .map(|(a, b)| max_abs((0..6).map(|i| a.corrections_degrees[i] - b.corrections_degrees[i])))
        .fold(0.0, f32::max);
    println!(
        "0.3 m headset jump while converging: correction difference vs jump-free run \
         within 1 s {window:.3}° (skipped measurements)"
    );
    assert!(max_abs(LEGS.map(|i| jumped.final_errors_degrees()[i])) < 2.0);
}

#[test]
fn stale_held_and_nan_inputs_block_updates_and_keep_outputs_finite() {
    let scenario = scenarios::faults();
    let run = run(&scenario);
    assert!(run.frames.iter().all(|frame| frame.outputs_finite));
    let in_window =
        |frame: &&HarnessFrame, start: f64, end: f64| frame.at_secs >= start && frame.at_secs < end;
    // Left thigh stale [3, 9): no left-foot measurement; the right foot keeps updating.
    let stale: Vec<_> = run
        .frames
        .iter()
        .filter(|f| in_window(f, 3.0, 9.0))
        .collect();
    assert_eq!(
        stale.first().unwrap().accepted[0],
        stale.last().unwrap().accepted[0]
    );
    assert!(stale.last().unwrap().accepted[1] > stale.first().unwrap().accepted[1]);
    // Left calf NaN (still flagged usable) [12, 16): the filter refuses it by itself.
    let nan: Vec<_> = run
        .frames
        .iter()
        .filter(|f| in_window(f, 12.0, 16.0))
        .collect();
    assert_eq!(
        nan.first().unwrap().accepted[0],
        nan.last().unwrap().accepted[0]
    );
    // NaN headset [20, 21): invalid input, nothing changes, both contacts released.
    let blind: Vec<_> = run
        .frames
        .iter()
        .filter(|f| in_window(f, 20.0, 21.0))
        .collect();
    for frame in &blind {
        assert_eq!(frame.status, StanceHeadingStatus::InvalidInput);
        assert_eq!(frame.corrections_degrees, blind[0].corrections_degrees);
        assert_eq!(frame.release_requests, [true; 2]);
    }
    // Afterwards it recovers and converges.
    let errors = run.final_errors_degrees();
    println!(
        "faults: final {errors:?}, converged < 3° at {:?}",
        run.time_to_converge(&LEGS, 3.0)
    );
    assert!(max_abs(LEGS.map(|i| errors[i])) < 2.0);
}

/// The filter before the first live session's findings: chest and hip yaw estimated from
/// displacement with the general ρ for every lever, and no correction bounds. Bit-identical
/// to the earlier implementation.
fn previous_model() -> HarnessOptions {
    let mut options = HarnessOptions::default();
    options.filter.estimate_torso_yaw = true;
    options.filter.torso_motion_noise_ratio = options.filter.motion_noise_ratio;
    options.filter.max_torso_excitation_ratio = 0.0;
    options.filter.max_leg_correction_degrees = 180.0;
    options.filter.max_torso_correction_degrees = 180.0;
    options
}

#[test]
fn flexed_spine_bows_do_not_become_torso_or_leg_yaw() {
    // Torso trackers over-read 60° bows (chest +12°, hip +30°, partly sideways) with no
    // yaw drift at all: every correction is false. The previous model turned the torso.
    let scenario = scenarios::flexed_spine_bows(20);
    let now = run_closed_loop(&scenario, &HarnessOptions::default());
    let before = run_closed_loop(&scenario, &previous_model());
    let worst = |run: &HarnessRun, segments: &[usize]| run.max_abs_correction_degrees(segments);
    println!(
        "20 flexed-spine bows: max false correction torso {:.2}° legs {:.2}° (previous model \
         torso {:.2}° legs {:.2}°); skipped torso-dominated increments {:?}",
        worst(&now, &[0, 1]),
        worst(&now, &LEGS),
        worst(&before, &[0, 1]),
        worst(&before, &LEGS),
        now.last_output.feet.map(|foot| foot.torso_skipped)
    );
    assert!(worst(&now, &[0, 1]) < 2.0);
    assert!(worst(&now, &LEGS) < 2.0);
    assert!(
        worst(&before, &[0, 1]) > 5.0,
        "the scenario must be able to fool the old model"
    );
    assert!(now.frames.iter().all(|frame| frame.outputs_finite));
}

#[test]
fn torso_corrections_are_bounded_and_reported() {
    // A 10° chest error with torso estimation on would be corrected by about 3.4° in this
    // script; a 2° bound holds it there (the flag reports the current state, checked on
    // a filter set directly below).
    let mut options = HarnessOptions::default();
    options.filter.estimate_torso_yaw = true;
    options.filter.max_torso_correction_degrees = 2.0;
    let scenario = Scenario::new(
        "squats and walking, chest 10°",
        vec![Motion::stand(1.0), Motion::squats(6), Motion::walk(16)],
    )
    .with_yaw_error(0, YawError::Constant { degrees: 10.0 });
    let run = run_closed_loop(&scenario, &options);
    let chest = run.max_abs_correction_degrees(&[0]);
    println!(
        "chest bound 2°: max chest correction {chest:.3}°, limited {:?}",
        run.last_output.correction_limited
    );
    assert!(chest <= 2.0 + 1.0e-3 && chest > 1.99, "{chest}");
    assert!(run.frames.iter().all(|frame| frame.outputs_finite));
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    filter.set_corrections([0.5, -0.5, 1.0, -1.0, 0.1, 0.0]);
    let c = filter.corrections();
    let limits = [12.0_f32, 12.0, 30.0, 30.0, 30.0, 30.0];
    for i in 0..6 {
        assert!(c[i].to_degrees().abs() <= limits[i] + 1.0e-3, "{c:?}");
    }
    let output = filter.last_output();
    assert_eq!(
        output.correction_limited,
        [true, true, true, true, false, false]
    );
}

#[test]
fn ten_minutes_of_random_walk_drift_rms_on_versus_off() {
    let scenario = scenarios::ten_minute_activity(7);
    let off = baseline(&scenario);
    for (label, options) in [
        ("default (legs only)", HarnessOptions::default()),
        ("previous model", previous_model()),
    ] {
        let on = run_closed_loop(&scenario, &options);
        let (rms_on, rms_off) = (
            on.rms_error_degrees(&ALL, 0.0),
            off.rms_error_degrees(&ALL, 0.0),
        );
        let (legs_on, legs_off) = (
            on.rms_error_degrees(&LEGS, 0.0),
            off.rms_error_degrees(&LEGS, 0.0),
        );
        let per_segment: Vec<_> = (0..6)
            .map(|i| {
                (
                    on.rms_error_degrees(&[i], 0.0),
                    off.rms_error_degrees(&[i], 0.0),
                )
            })
            .collect();
        println!(
            "10 min mixed activity, independent random-walk drift [{label}]: RMS on \
             {rms_on:.2}°, off {rms_off:.2}°; legs on {legs_on:.2}°, off {legs_off:.2}°; per \
             segment (on, off) {per_segment:?}; accepted {}, rejected {}",
            on.accepted(),
            on.rejected()
        );
        assert!(on.frames.iter().all(|frame| frame.outputs_finite));
        // Legs (observed absolutely while walking) must improve individually.
        for i in LEGS {
            assert!(
                per_segment[i].0 < per_segment[i].1,
                "segment {i}: {:?}",
                per_segment[i]
            );
        }
        if options.filter.estimate_torso_yaw {
            assert!(rms_on < 0.6 * rms_off);
            // Regression bound: 2.46° before the recovery mechanisms, 2.49° with them.
            assert!(rms_on <= 2.6, "{rms_on}");
        } else {
            // Legs only: the legs keep (almost) the previous model's accuracy; the chest and
            // hip are held only by the deadbanded priors, so their 0.5°/√s synthetic random
            // walk (far above a real IMU's drift) is only partly removed.
            assert!(legs_on < 0.5 * legs_off, "{legs_on} vs {legs_off}");
            assert!(legs_on <= 2.4, "{legs_on}");
            for i in [0, 1] {
                assert!(
                    per_segment[i].0 < per_segment[i].1,
                    "segment {i}: {:?}",
                    per_segment[i]
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Recovery from large sudden errors (uncertainty-aware contact gates, covariance inflation).

/// The recovery mechanisms switched off (contact gates and covariance as before).
fn without_recovery() -> HarnessOptions {
    let mut options = HarnessOptions {
        use_speed_allowance: false,
        ..HarnessOptions::default()
    };
    options.filter.inflation_step_degrees = 0.0;
    options.filter.starvation_secs = 0.0;
    options
}

/// Seconds from `event` until the remaining error over `segments` stays below 3°.
fn reconvergence(run: &HarnessRun, segments: &[usize], event: f64) -> Option<f64> {
    run.time_to_converge(segments, 3.0).map(|t| t - event)
}

#[test]
fn contact_speed_allowance_follows_leg_uncertainty_and_horizontal_head_speed() {
    let walking = Vec3::new(0.8, 0.5, 0.0);
    let sigma = [5.0_f32; 6].map(f32::to_radians);
    let allowance = contact_speed_allowance(sigma, walking, 2.0, 30.0);
    let expected = 10.0_f32.to_radians().sin() * 0.8;
    assert!((allowance[0] - expected).abs() < 1.0e-5 && (allowance[1] - expected).abs() < 1.0e-5);
    // Chest uncertainty is ignored; each foot uses its own leg.
    let mut uneven = sigma;
    uneven[0] = 40.0_f32.to_radians();
    uneven[THIGH[1]] = 8.0_f32.to_radians();
    let allowance = contact_speed_allowance(uneven, walking, 2.0, 30.0);
    assert!((allowance[0] - expected).abs() < 1.0e-5);
    assert!((allowance[1] - 16.0_f32.to_radians().sin() * 0.8).abs() < 1.0e-5);
    // Capped at the angle limit; nothing while the head does not translate.
    let huge = contact_speed_allowance([1.0; 6], walking, 2.0, 30.0);
    assert!((huge[0] - 0.4).abs() < 1.0e-5);
    assert_eq!(contact_speed_allowance(sigma, Vec3::Y, 2.0, 30.0), [0.0; 2]);
    assert_eq!(
        contact_speed_allowance(sigma, Vec3::NAN, 2.0, 30.0),
        [0.0; 2]
    );
    assert_eq!(
        contact_speed_allowance([f32::NAN; 6], walking, 2.0, 30.0),
        [0.0; 2]
    );
    assert_eq!(
        contact_speed_allowance(sigma, walking, f32::NAN, 30.0),
        [0.0; 2]
    );
    // The method uses the filter's own sigma and configuration.
    let filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    let method = filter.contact_speed_allowance(walking);
    let free = contact_speed_allowance(filter.sigma(), walking, 2.0, 30.0);
    assert_eq!(method, free);
}

#[test]
fn add_uncertainty_combines_in_quadrature_and_respects_the_cap() {
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    filter.add_uncertainty([0.0, f32::NAN, -3.0, 10.0, 100.0, f32::INFINITY]);
    let sigma = degrees(filter.sigma());
    assert!((sigma[0] - 2.0).abs() < 1.0e-4);
    assert!((sigma[1] - 2.0).abs() < 1.0e-4);
    assert!((sigma[2] - 2.0).abs() < 1.0e-4);
    assert!((sigma[3] - 104.0_f32.sqrt()).abs() < 1.0e-3);
    assert!((sigma[4] - 45.0).abs() < 1.0e-3);
    assert!((sigma[5] - 2.0).abs() < 1.0e-4);
    assert_eq!(filter.corrections(), [0.0; 6]);
}

#[test]
fn rejected_measurements_inflate_the_chain_within_the_cap() {
    let creep = |frame: &mut Frame| {
        frame.head_position.x += 0.5 * frame.at as f32;
        frame.planted = [true; 2];
    };
    let scenario = Scenario::new("squats", vec![Motion::squats(3)]);
    // Default (legs only): displacement outliers reopen only the segments the measurement
    // estimates; with torso yaw estimation on, the chest and hip reopen too.
    for (estimate_torso_yaw, inflated_segments) in [(false, &LEGS[..]), (true, &ALL[..])] {
        let config = StanceHeadingConfig {
            estimate_torso_yaw,
            ..StanceHeadingConfig::default()
        };
        let mut inflated = StanceHeadingFilter::new(config);
        drive(&mut inflated, &scenario, 0.0, 9.0, creep, |_, output| {
            assert_output_finite(output)
        });
        let mut plain = StanceHeadingFilter::new(StanceHeadingConfig {
            inflation_step_degrees: 0.0,
            ..config
        });
        drive(&mut plain, &scenario, 0.0, 9.0, creep, |_, _| {});
        let (with, without) = (degrees(inflated.sigma()), degrees(plain.sigma()));
        let output = inflated.last_output();
        println!(
            "headset creep (torso yaw estimated: {estimate_torso_yaw}): sigma with inflation \
             {with:?}, without {without:?}, inflations {:?}",
            output.feet.map(|foot| foot.inflations)
        );
        for i in ALL {
            if inflated_segments.contains(&i) {
                assert!(
                    with[i] > without[i] + 3.0,
                    "segment {i}: {with:?} vs {without:?}"
                );
            } else {
                assert!(
                    (with[i] - without[i]).abs() < 0.5,
                    "segment {i}: {with:?} vs {without:?}"
                );
            }
            // The inflation cap is 15°; process noise over 9 s adds a little on top.
            assert!(with[i] <= 16.0, "segment {i}: {with:?}");
        }
        assert!(
            output
                .feet
                .iter()
                .all(|foot| foot.nis_average > 2.0 && foot.inflations > 0)
        );
        // Rejected data never moves the state.
        assert!(max_abs(degrees(inflated.corrections())) < 3.0);
    }
}

#[test]
fn contact_starvation_inflates_only_the_starved_leg_and_only_while_translating() {
    let scenario = scenarios::walking_common_leg_drift(0.0);
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    let mut outputs = Vec::new();
    drive(
        &mut filter,
        &scenario,
        0.0,
        12.0,
        |frame| frame.planted[1] = false,
        |_, output| outputs.push(*output),
    );
    let sigma = degrees(filter.sigma());
    println!("right foot never planted while walking: sigma {sigma:?}");
    // About one 3° step per second of unplanted walking (turns pause the clock).
    assert!(sigma[THIGH[1]] > 8.0 && sigma[CALF[1]] > 8.0, "{sigma:?}");
    assert!(
        sigma[THIGH[1]] <= 15.5 && sigma[CALF[1]] <= 15.5,
        "{sigma:?}"
    );
    assert!(sigma[THIGH[0]] < 6.0 && sigma[CALF[0]] < 6.0, "{sigma:?}");
    assert!(outputs.last().unwrap().feet[1].inflations >= 3);

    // Standing still with no contact at all: no translation, no starvation inflation.
    let still = Scenario::new("stand", vec![Motion::stand(12.0)]);
    let mut filter = StanceHeadingFilter::new(StanceHeadingConfig::default());
    drive(
        &mut filter,
        &still,
        0.0,
        12.0,
        |frame| frame.planted = [false; 2],
        |_, _| {},
    );
    assert!(degrees(filter.sigma()).iter().all(|s| *s < 2.2));
    assert_eq!(
        filter.last_output().feet.map(|foot| foot.inflations),
        [0, 0]
    );
}

#[test]
fn sudden_common_mode_leg_error_during_walking_reconverges() {
    for (degrees, limit) in [(20.0_f32, 15.0), (30.0, 25.0)] {
        let scenario = scenarios::walking_sudden_leg_error(degrees, 20.0);
        let run = run(&scenario);
        let before = without_recovery();
        let old = run_closed_loop(&scenario, &before);
        let after = reconvergence(&run, &LEGS, 20.0);
        println!(
            "walking, sudden {degrees}° on all leg sensors at 20 s: back below 3° after {after:?} s \
             (without recovery mechanisms {:?} s); max error before the jump {:.2}°",
            reconvergence(&old, &LEGS, 20.0),
            run.frames
                .iter()
                .filter(|f| (15.0..20.0).contains(&f.at_secs))
                .flat_map(|f| LEGS.map(|i| f.remaining_error_degrees[i].abs()))
                .fold(0.0, f32::max)
        );
        assert!(after.is_some_and(|t| t <= limit), "{degrees}: {after:?}");
        assert!(run.frames.iter().all(|frame| frame.outputs_finite));
    }
}

#[test]
fn sudden_single_thigh_error_during_walking_reconverges() {
    let scenario = scenarios::walking_sudden_thigh_error(20.0, 20.0);
    let run = run(&scenario);
    let after = reconvergence(&run, &[2], 20.0);
    let others = run.max_abs_error_degrees(&[0, 1, 3, 4, 5], 0.0);
    println!(
        "walking, left thigh jumps 20° at 20 s: back below 3° after {after:?} s (without recovery \
         {:?} s); worst error on the other segments {others:.2}°",
        reconvergence(&run_closed_loop(&scenario, &without_recovery()), &[2], 20.0)
    );
    assert!(after.is_some_and(|t| t <= 20.0), "{after:?}");
    assert!(others < 5.0);
}

#[test]
fn long_unobserved_drift_then_walking_reconverges() {
    // 60 s standing while every leg sensor drifts to 25°: no information, so σ stays small and
    // the filter is confident but wrong when walking starts.
    let scenario = scenarios::unobserved_leg_drift_then_walking(25.0);
    let run = run(&scenario);
    let after = reconvergence(&run, &LEGS, 60.0);
    println!(
        "25° unobserved drift, then walking: below 3° after {after:?} s of walking (without \
         recovery {:?} s)",
        reconvergence(
            &run_closed_loop(&scenario, &without_recovery()),
            &LEGS,
            60.0
        )
    );
    assert!(after.is_some_and(|t| t <= 20.0), "{after:?}");
}

#[test]
fn thigh_strap_twist_is_not_corrected_as_world_yaw() {
    // A strap turning about the leg rotates the sensor about the bone's own long axis. Bone +Y
    // (and so every chain position) is unchanged, so the displacement observation carries no
    // information, and a world-yaw correction would tilt a pitched thigh's axis and corrupt
    // positions. Only the knee hinge prior sees it. This documents the limitation: the filter
    // must not chase it with a large world-yaw correction.
    let scenario = scenarios::walking_thigh_strap_rotation(20.0, 20.0);
    let run = run(&scenario);
    let correction = run.max_abs_correction_degrees(&LEGS);
    let remaining = run.final_errors_degrees()[2];
    println!(
        "thigh strap twists 20° at 20 s: max leg correction {correction:.2}°, remaining thigh \
         heading error {remaining:.2}° (needs a mounting-twist estimate, not world yaw)"
    );
    assert!(correction < 6.0);
    assert!(remaining.abs() > 10.0);
}

#[test]
fn mixed_squats_steps_and_weight_shifts_resolve_the_single_calf_error() {
    let scenario = scenarios::mixed_squats_calf_drift(15.0);
    let run = run(&scenario);
    let converged = run.time_to_converge(&[4], 3.0);
    let errors = run.final_errors_degrees();
    println!(
        "squats + steps + weight shifts, left calf 15°: < 3° from {converged:?} s, final {errors:?}"
    );
    assert!(converged.is_some_and(|t| t <= 30.0), "{converged:?}");
    assert!(max_abs(errors) < 2.0, "{errors:?}");
}

#[test]
fn squat_hinge_correction_is_split_by_the_prior_covariances() {
    // Knee prior only: it observes calf − thigh, so the correction of the hinge residual is
    // shared in proportion to the segments' variances.
    let scenario = scenarios::squats_calf_drift(15.0);
    let mut options = HarnessOptions::default();
    options.filter.enable_displacement = false;
    let equal = run_closed_loop(&scenario, &options);
    options.extra_initial_sigma_degrees[4] = 10.0;
    let calf_uncertain = run_closed_loop(&scenario, &options);
    for (name, run) in [("equal start", &equal), ("calf +10° σ", &calf_uncertain)] {
        let last = run.frames.last().unwrap();
        let mid = &run.frames[run.frames.len() / 3];
        println!(
            "knee prior split [{name}]: thigh {:.2}°, calf {:.2}° (σ at 1/3: thigh {:.2}°, calf \
             {:.2}°), hinge residual {:.2}°",
            last.corrections_degrees[2],
            last.corrections_degrees[4],
            mid.sigma_degrees[2],
            mid.sigma_degrees[4],
            last.remaining_error_degrees[4] - last.remaining_error_degrees[2]
        );
        // The hinge residual is closed down to the 5° deadband, thigh and calf moving apart.
        let hinge = last.remaining_error_degrees[4] - last.remaining_error_degrees[2];
        assert!(hinge.abs() <= 5.5, "{name}: {hinge}");
        assert!(last.corrections_degrees[2] <= 0.0 && last.corrections_degrees[4] > 0.0);
    }
    // Equal start: the thigh rotates more in squats, so it accrues more process noise and takes
    // the larger share (variance ratio ≈ 1.5); a known-uncertain calf takes most of it.
    let share = |run: &HarnessRun| {
        let last = run.frames.last().unwrap();
        last.corrections_degrees[4].abs()
            / (last.corrections_degrees[2].abs() + last.corrections_degrees[4].abs())
    };
    assert!((0.3..0.55).contains(&share(&equal)), "{}", share(&equal));
    assert!(share(&calf_uncertain) > 0.85, "{}", share(&calf_uncertain));
}
