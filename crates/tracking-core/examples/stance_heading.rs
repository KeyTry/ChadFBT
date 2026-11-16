//! Deterministic synthetic scenarios for the stance heading filter.
//!
//! cargo run --release -p tracking-core --example stance_heading -- --json path.json
//!
//! Ground truth is generated with the estimator's own kinematic model (see
//! `tracking_core::synthetic`). These numbers show that the implementation behaves as designed
//! under injected faults; they are not hardware accuracy, latency or SlimeVR comparisons.

use serde::Serialize;
use std::error::Error;
use std::time::Instant;
use tracking_core::synthetic::{HarnessOptions, HarnessRun, Scenario, run_closed_loop, scenarios};

const LEGS: [usize; 4] = [2, 3, 4, 5];
const ALL: [usize; 6] = [0, 1, 2, 3, 4, 5];

#[derive(Serialize)]
struct Row {
    scenario: String,
    variant: &'static str,
    duration_secs: f64,
    /// Segments the error metrics use (Role::CORE indices).
    segments: Vec<usize>,
    /// Event time (sudden error) the convergence time is measured from.
    event_secs: f64,
    /// Seconds after `event_secs` from which max |remaining error| over `segments` stays
    /// below 3°.
    time_to_3deg_secs: Option<f64>,
    final_abs_error_degrees_on: f32,
    final_abs_error_degrees_off: f32,
    rms_error_degrees_on: f32,
    rms_error_degrees_off: f32,
    /// Largest absolute correction on any core segment (false-correction measure when the
    /// scenario has no drift).
    max_abs_correction_degrees: f32,
    final_errors_degrees_on: [f32; 6],
    rms_per_segment_degrees_on: [f32; 6],
    rms_per_segment_degrees_off: [f32; 6],
    accepted: u32,
    rejected: u32,
    release_requests: u32,
    /// Largest true ankle motion while the contact tracker said `Planted`.
    max_planted_truth_motion_m: f32,
    max_ik_shortfall_m: f32,
    all_outputs_finite: bool,
    wall_ms_filter_on: f64,
}

fn final_abs(run: &HarnessRun, segments: &[usize]) -> f32 {
    let errors = run.final_errors_degrees();
    segments
        .iter()
        .map(|&i| errors[i].abs())
        .fold(0.0, f32::max)
}

fn evaluate(
    scenario: Scenario,
    segments: &[usize],
    variant: &'static str,
    options: HarnessOptions,
) -> Row {
    evaluate_at(scenario, segments, variant, options, 0.0)
}

fn evaluate_at(
    scenario: Scenario,
    segments: &[usize],
    variant: &'static str,
    options: HarnessOptions,
    event_secs: f64,
) -> Row {
    let started = Instant::now();
    let on = run_closed_loop(&scenario, &options);
    let wall_ms = started.elapsed().as_secs_f64() * 1e3;
    let off = run_closed_loop(
        &scenario,
        &HarnessOptions {
            filter_enabled: false,
            ..options
        },
    );
    Row {
        scenario: scenario.name.clone(),
        variant,
        duration_secs: scenario.duration(),
        segments: segments.to_vec(),
        event_secs,
        time_to_3deg_secs: on.time_to_converge(segments, 3.0).map(|t| t - event_secs),
        final_abs_error_degrees_on: final_abs(&on, segments),
        final_abs_error_degrees_off: final_abs(&off, segments),
        rms_error_degrees_on: on.rms_error_degrees(segments, 0.0),
        rms_error_degrees_off: off.rms_error_degrees(segments, 0.0),
        max_abs_correction_degrees: on.max_abs_correction_degrees(&ALL),
        final_errors_degrees_on: on.final_errors_degrees(),
        rms_per_segment_degrees_on: std::array::from_fn(|i| on.rms_error_degrees(&[i], 0.0)),
        rms_per_segment_degrees_off: std::array::from_fn(|i| off.rms_error_degrees(&[i], 0.0)),
        accepted: on.accepted(),
        rejected: on.rejected(),
        release_requests: on.release_requests,
        max_planted_truth_motion_m: on.max_planted_truth_motion_m,
        max_ik_shortfall_m: on.max_ik_shortfall_m,
        all_outputs_finite: on.frames.iter().all(|frame| frame.outputs_finite),
        wall_ms_filter_on: wall_ms,
    }
}

/// The large-error recovery mechanisms switched off (no contact speed allowance, no
/// covariance inflation, no starvation inflation): the previous behaviour.
fn before_recovery() -> HarnessOptions {
    let mut options = HarnessOptions {
        use_speed_allowance: false,
        ..HarnessOptions::default()
    };
    options.filter.inflation_step_degrees = 0.0;
    options.filter.starvation_secs = 0.0;
    options
}

/// The filter before the first live session's findings (2026-11-15): chest and hip yaw
/// estimated from displacement, the general ρ for every lever, no torso-dominated increment
/// gate and no correction bounds. Reproduces the earlier default numbers exactly.
fn previous_model() -> HarnessOptions {
    let mut options = HarnessOptions::default();
    options.filter.estimate_torso_yaw = true;
    options.filter.torso_motion_noise_ratio = options.filter.motion_noise_ratio;
    options.filter.max_torso_excitation_ratio = 0.0;
    options.filter.max_leg_correction_degrees = 180.0;
    options.filter.max_torso_correction_degrees = 180.0;
    options
}

/// The contact/noise refinements that go beyond the written spec, switched off.
fn spec_only() -> HarnessOptions {
    let mut options = HarnessOptions::default();
    options.contact.max_planted_yaw_degrees = 0.0;
    options.contact.exit_vertical_speed_mps = 0.0;
    options.filter.pivot_sigma_m_per_rad = 0.0;
    options
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !args.is_empty() && (args.len() != 2 || args[0] != "--json") {
        return Err("usage: stance_heading [--json output.json]".into());
    }
    let default = HarnessOptions::default();
    let mut enter_030 = default;
    enter_030.contact.enter_speed_mps = 0.30;
    let mut knee_only = default;
    knee_only.filter.enable_displacement = false;
    let before = before_recovery();
    let mut knee_only_calf_uncertain = knee_only;
    knee_only_calf_uncertain.extra_initial_sigma_degrees[4] = 10.0;
    let mut no_priors = default;
    no_priors.filter.enable_knee_prior = false;
    no_priors.filter.enable_pelvis_prior = false;
    no_priors.filter.enable_chest_prior = false;
    let previous = previous_model();
    let mut no_torso_gate = default;
    no_torso_gate.filter.max_torso_excitation_ratio = 0.0;

    let rows = vec![
        evaluate(
            scenarios::squats_calf_drift(15.0),
            &[4],
            "no torso gate",
            no_torso_gate,
        ),
        evaluate(
            scenarios::squats_calf_drift(15.0),
            &[4],
            "previous model",
            previous,
        ),
        evaluate(
            scenarios::ten_minute_activity(7),
            &ALL,
            "no torso gate",
            no_torso_gate,
        ),
        evaluate(
            scenarios::flexed_spine_bows(20),
            &ALL,
            "no torso gate",
            no_torso_gate,
        ),
        evaluate(
            scenarios::walking_common_leg_drift(20.0),
            &LEGS,
            "previous model",
            previous,
        ),
        evaluate(scenarios::flexed_spine_bows(20), &ALL, "default", default),
        evaluate(
            scenarios::flexed_spine_bows(20),
            &ALL,
            "previous model",
            previous,
        ),
        evaluate(
            scenarios::slow_foot_slide(0.15, 0.0),
            &ALL,
            "previous model",
            previous,
        ),
        evaluate(
            scenarios::ten_minute_activity(7),
            &ALL,
            "previous model",
            previous,
        ),
        evaluate(
            scenarios::walking_common_leg_drift(20.0),
            &LEGS,
            "default",
            default,
        ),
        evaluate(
            scenarios::walking_common_leg_drift(20.0),
            &LEGS,
            "before recovery",
            before,
        ),
        evaluate_at(
            scenarios::walking_sudden_leg_error(20.0, 20.0),
            &LEGS,
            "default",
            default,
            20.0,
        ),
        evaluate_at(
            scenarios::walking_sudden_leg_error(20.0, 20.0),
            &LEGS,
            "before recovery",
            before,
            20.0,
        ),
        evaluate_at(
            scenarios::walking_sudden_leg_error(30.0, 20.0),
            &LEGS,
            "default",
            default,
            20.0,
        ),
        evaluate_at(
            scenarios::walking_sudden_leg_error(30.0, 20.0),
            &LEGS,
            "before recovery",
            before,
            20.0,
        ),
        evaluate_at(
            scenarios::walking_sudden_thigh_error(20.0, 20.0),
            &[2],
            "default",
            default,
            20.0,
        ),
        evaluate_at(
            scenarios::walking_sudden_thigh_error(20.0, 20.0),
            &[2],
            "before recovery",
            before,
            20.0,
        ),
        evaluate_at(
            scenarios::unobserved_leg_drift_then_walking(25.0),
            &LEGS,
            "default",
            default,
            60.0,
        ),
        evaluate_at(
            scenarios::unobserved_leg_drift_then_walking(25.0),
            &LEGS,
            "before recovery",
            before,
            60.0,
        ),
        evaluate_at(
            scenarios::walking_thigh_strap_rotation(20.0, 20.0),
            &[2],
            "default",
            default,
            20.0,
        ),
        evaluate(
            scenarios::mixed_squats_calf_drift(15.0),
            &[4],
            "default",
            default,
        ),
        evaluate(
            scenarios::squats_calf_drift(15.0),
            &[4],
            "knee only, calf +10° σ",
            knee_only_calf_uncertain,
        ),
        evaluate(
            scenarios::walking_common_leg_drift(20.0),
            &LEGS,
            "enter 0.30 m/s",
            enter_030,
        ),
        evaluate(
            scenarios::walking_common_leg_drift(10.0),
            &LEGS,
            "default",
            default,
        ),
        evaluate(
            Scenario::new("walking, left thigh +10°", scenarios::walk_laps(4)).with_yaw_error(
                2,
                tracking_core::synthetic::YawError::Constant { degrees: 10.0 },
            ),
            &[2],
            "default",
            default,
        ),
        evaluate(scenarios::squats_calf_drift(15.0), &[4], "default", default),
        evaluate(
            scenarios::squats_calf_drift(15.0),
            &[4],
            "knee prior only",
            knee_only,
        ),
        evaluate(
            scenarios::weight_shift_leg_drift(10.0),
            &LEGS,
            "default",
            default,
        ),
        evaluate(
            scenarios::standing_still_drift(true),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::standing_still_drift(false),
            &ALL,
            "priors off",
            no_priors,
        ),
        evaluate(
            scenarios::standing_still_drift(false),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::head_turns_eye_offset(0.04),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::bone_length_error(0.95, 10.0),
            &LEGS,
            "default",
            default,
        ),
        evaluate(
            scenarios::bone_length_error(0.95, 0.0),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::slow_foot_slide(0.05, 0.0),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::slow_foot_slide(0.05, 90.0),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::slow_foot_slide(0.15, 0.0),
            &ALL,
            "default",
            default,
        ),
        evaluate(
            scenarios::slow_foot_slide(0.05, 0.0),
            &ALL,
            "spec only",
            spec_only(),
        ),
        evaluate(
            scenarios::slow_foot_slide(0.05, 0.0),
            &ALL,
            "before recovery",
            before,
        ),
        evaluate(scenarios::turn_on_spot(0.8), &ALL, "default", default),
        evaluate(
            scenarios::turn_on_spot(0.8),
            &ALL,
            "before recovery",
            before,
        ),
        evaluate(scenarios::turn_on_spot(1.6), &ALL, "default", default),
        evaluate(scenarios::turn_on_spot(0.8), &ALL, "spec only", spec_only()),
        evaluate(
            scenarios::headset_jump(0.3, 5.005),
            &LEGS,
            "default",
            default,
        ),
        evaluate(scenarios::faults(), &LEGS, "default", default),
        evaluate(scenarios::ten_minute_activity(7), &ALL, "default", default),
        evaluate(
            scenarios::ten_minute_activity(7),
            &ALL,
            "before recovery",
            before,
        ),
        evaluate(scenarios::ten_minute_activity(11), &ALL, "default", default),
        evaluate(scenarios::ten_minute_activity(23), &ALL, "default", default),
        evaluate(
            scenarios::ten_minute_activity(7),
            &ALL,
            "spec only",
            spec_only(),
        ),
    ];

    println!(
        "{:<58} {:>7} {:>6} {:>6} {:>6} {:>6} {:>6} {:>5} {:>5} {:>4} {:>5}",
        "scenario [variant] (segments)",
        "t<3°",
        "fin on",
        "fin off",
        "rms on",
        "rms off",
        "max|c|",
        "acc",
        "rej",
        "rel",
        "slip"
    );
    for row in &rows {
        let label = format!("{} [{}] {:?}", row.scenario, row.variant, row.segments);
        println!(
            "{:<58} {:>7} {:>6.2} {:>6.2} {:>6.2} {:>6.2} {:>6.2} {:>5} {:>5} {:>4} {:>4.1}c",
            label,
            row.time_to_3deg_secs
                .map_or("never".to_owned(), |t| format!("{t:.1}s")),
            row.final_abs_error_degrees_on,
            row.final_abs_error_degrees_off,
            row.rms_error_degrees_on,
            row.rms_error_degrees_off,
            row.max_abs_correction_degrees,
            row.accepted,
            row.rejected,
            row.release_requests,
            row.max_planted_truth_motion_m * 100.0,
        );
    }
    println!(
        "\nfinal remaining error per segment with the filter on (chest hip lt rt lc rc), degrees:"
    );
    for row in &rows {
        println!(
            "  {:<56} {}",
            format!("{} [{}]", row.scenario, row.variant),
            row.final_errors_degrees_on
                .iter()
                .map(|v| format!("{v:6.2}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
    let finite = rows.iter().all(|row| row.all_outputs_finite);
    let shortfall = rows
        .iter()
        .map(|row| row.max_ik_shortfall_m)
        .fold(0.0, f32::max);
    println!("\nall outputs finite: {finite}; max generator IK shortfall: {shortfall:.1e} m");
    if let Some(path) = args.get(1) {
        if let Some(parent) = std::path::Path::new(path).parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(&rows)?)?;
    }
    Ok(())
}
