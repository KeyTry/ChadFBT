//! Deterministic synthetic solver benchmark. This is neither a sensor accuracy
//! test nor a measurement of transport, motion-to-photon or SteamVR latency.
//!
//! cargo run --release -p tracking-core --example benchmark --
//!   --json local-data/benchmarks/tracking-core.json

use glam::{Quat, Vec3};
use serde::Serialize;
use std::error::Error;
use std::hint::black_box;
use std::path::PathBuf;
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use tracking_core::{
    AnchorPose, Anchors, Role, RoleAssignment, SensorSample, TrackingConfig, TrackingCore,
    TrackingFrame,
};

const HZ: f64 = 100.0;
const CALIBRATION_FRAMES: usize = 100;
const ERROR_FRAMES: usize = 12_000;
const ERROR_FINAL_WINDOW_FRAMES: usize = 1_000;

#[derive(Debug)]
struct Options {
    frames: usize,
    warmup: usize,
    trials: usize,
    json: Option<PathBuf>,
}

#[derive(Clone, Copy)]
struct Case {
    sensor_count: usize,
    correction: bool,
    /// Prediction, HMD extrapolation, auto rebase, stance heading filter and
    /// leg IK all enabled (the application's defaults).
    estimator: bool,
}

#[derive(Serialize)]
struct Distribution {
    count: usize,
    mean_us: f64,
    p50_us: f64,
    p95_us: f64,
    p99_us: f64,
    max_us: f64,
}

#[derive(Serialize)]
struct TimingCase {
    sensor_count: usize,
    experimental_correction_enabled: bool,
    estimator_layers_enabled: bool,
    combined_solve_time: Distribution,
    trials: Vec<Distribution>,
    valid_frames: usize,
    expected_valid_frames: usize,
    checksum: f64,
}

#[derive(Serialize)]
struct ErrorMetrics {
    sample_count: usize,
    relative_knee_heading_rmse_deg: f64,
    thigh_heading_rmse_deg: f64,
    calf_heading_rmse_deg: f64,
    final_window_relative_knee_heading_mae_deg: f64,
    final_window_thigh_heading_mae_deg: f64,
    final_window_calf_heading_mae_deg: f64,
    final_window_pair_common_heading_mae_deg: f64,
    correction_active_frames: usize,
    all_targets_valid: bool,
}

#[derive(Serialize)]
struct ErrorCase {
    name: &'static str,
    construction: &'static str,
    baseline: ErrorMetrics,
    experimental: ErrorMetrics,
}

#[derive(Serialize)]
struct BenchmarkReport {
    schema_version: u32,
    unix_timestamp_secs: u64,
    crate_version: &'static str,
    operating_system: &'static str,
    architecture: &'static str,
    debug_assertions: bool,
    input_frequency_hz: f64,
    timing_frames_per_trial: usize,
    warmup_frames_per_trial: usize,
    trial_count: usize,
    calibration_frames_per_trial: usize,
    simulated_error_duration_secs: f64,
    final_error_window_secs: f64,
    parameters: TrackingConfig,
    methodology: Vec<&'static str>,
    timer_pair_overhead: Distribution,
    timings: Vec<TimingCase>,
    synthetic_error_cases: Vec<ErrorCase>,
}

#[derive(Clone, Copy)]
enum ErrorScenario {
    RelativeDrift,
    AnatomicalTwist,
    CommonDrift,
    RealTurn,
}

fn main() -> Result<(), Box<dyn Error>> {
    let options = parse_options()?;
    if cfg!(debug_assertions) {
        eprintln!("WARNING: debug build; use --release for meaningful solver timing.");
    }
    let case = |sensor_count, correction, estimator| Case {
        sensor_count,
        correction,
        estimator,
    };
    let cases = [
        case(6, false, false),
        case(6, true, false),
        case(10, false, false),
        case(10, true, false),
        case(6, false, true),
        case(10, false, true),
    ];
    let mut all_samples: Vec<Vec<f64>> = cases
        .iter()
        .map(|_| Vec::with_capacity(options.frames * options.trials))
        .collect();
    let mut trial_distributions: Vec<Vec<Distribution>> =
        cases.iter().map(|_| Vec::new()).collect();
    let mut valid_frames = [0_usize; 6];
    let mut checksums = [0.0_f64; 6];

    let timer_pair_overhead = measure_timer_overhead(10_000);
    for trial in 0..options.trials {
        // Rotate case order to avoid always giving one configuration the coldest
        // cache or the same position in the host's scheduling/thermal cycle.
        for offset in 0..cases.len() {
            let index = (trial + offset) % cases.len();
            let (samples, valid, checksum) = run_timing(cases[index], &options)?;
            valid_frames[index] += valid;
            checksums[index] += checksum;
            trial_distributions[index].push(distribution(&samples));
            all_samples[index].extend(samples);
        }
    }
    let timings = cases
        .into_iter()
        .enumerate()
        .map(|(index, case)| TimingCase {
            sensor_count: case.sensor_count,
            experimental_correction_enabled: case.correction,
            estimator_layers_enabled: case.estimator,
            combined_solve_time: distribution(&all_samples[index]),
            trials: std::mem::take(&mut trial_distributions[index]),
            valid_frames: valid_frames[index],
            expected_valid_frames: options.frames * options.trials,
            checksum: checksums[index],
        })
        .collect();

    let mut synthetic_error_cases = Vec::new();
    for (scenario, name, construction) in [
        (
            ErrorScenario::RelativeDrift,
            "injected_left_calf_yaw_drift",
            "Ideal hinge motion; add world-up yaw to the left calf measurement only, ramping 0 to 15 degrees over 60 seconds and holding for 60 seconds. Ground truth contains no knee twist.",
        ),
        (
            ErrorScenario::AnatomicalTwist,
            "real_left_calf_twist_counterexample",
            "Exactly the same observations as the calf-drift case, but the 0-to-15-degree rotation is true calf motion, with no sensor error. This exposes ambiguity and false correction by the hinge prior.",
        ),
        (
            ErrorScenario::CommonDrift,
            "common_mode_yaw_drift",
            "Add the same 0-to-15-degree heading drift to every body IMU while keeping the head anchor accurate. A relative knee constraint cannot remove this common heading error.",
        ),
        (
            ErrorScenario::RealTurn,
            "real_whole_body_turn",
            "All body segments and the head execute the same smooth changing heading with no injected sensor error. The correction must preserve that real turn.",
        ),
    ] {
        synthetic_error_cases.push(ErrorCase {
            name,
            construction,
            baseline: run_error(scenario, false)?,
            experimental: run_error(scenario, true)?,
        });
    }
    let report = BenchmarkReport {
        schema_version: 2,
        unix_timestamp_secs: SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs(),
        crate_version: env!("CARGO_PKG_VERSION"),
        operating_system: std::env::consts::OS,
        architecture: std::env::consts::ARCH,
        debug_assertions: cfg!(debug_assertions),
        input_frequency_hz: HZ,
        timing_frames_per_trial: options.frames,
        warmup_frames_per_trial: options.warmup,
        trial_count: options.trials,
        calibration_frames_per_trial: CALIBRATION_FRAMES,
        simulated_error_duration_secs: ERROR_FRAMES as f64 / HZ,
        final_error_window_secs: ERROR_FINAL_WINDOW_FRAMES as f64 / HZ,
        parameters: TrackingConfig::default(),
        methodology: vec![
            "Deterministic analytic trajectories; no network, physical sensors, random input or external model weights.",
            "Every trial creates and calibrates a fresh core, then runs an untimed moving warmup before timed frames.",
            "Input timestamps advance exactly 10 ms; execution is unpaced as fast as the host allows. This is not a real-time 100 Hz scheduling test.",
            "Instant measures elapsed wall time of solve and returned-frame construction, including allocator/scheduler effects. It excludes ingest, synthetic input construction, caller output drop, serialization, UI, transport and driver work.",
            "black_box consumes solve inputs/outputs; benchmark output validates all expected body targets and finite values outside the timed section.",
            "Percentiles use linear interpolation at p*(N-1). Timing samples combine rotated-order independent trials; per-trial summaries are also retained.",
            "Timer-pair overhead is measured separately and never subtracted. Host contention and timer precision can affect the tails.",
            "Error tests use default solver thresholds/rate. The experimental toggle is the only estimator difference between each paired run.",
            "Timing cases marked estimator=on enable prediction, HMD extrapolation, auto rebase, the stance heading filter and leg IK (the application defaults); their cost depends on how often feet are hypothesised planted in this synthetic motion.",
            "Heading errors compare projected calibrated local X axes with analytic ground truth; relative residual reduction alone is not proof of anatomical accuracy.",
            "The anatomical-twist counterexample intentionally reinterprets identical observations as real motion to expose the prior's ambiguity.",
            "No physical accuracy, long-session drift, motion-to-photon latency, SteamVR end-to-end latency, or superiority over SlimeVR is claimed.",
        ],
        timer_pair_overhead,
        timings,
        synthetic_error_cases,
    };
    validate_report(&report)?;
    print_report(&report);
    if let Some(path) = options.json {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&path, serde_json::to_string_pretty(&report)?)?;
        println!("JSON report: {}", path.display());
    }
    Ok(())
}

fn parse_options() -> Result<Options, Box<dyn Error>> {
    let mut options = Options {
        frames: 10_000,
        warmup: 1_000,
        trials: 3,
        json: None,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--frames" => {
                options.frames = args.next().ok_or("--frames needs an integer")?.parse()?
            }
            "--warmup" => {
                options.warmup = args.next().ok_or("--warmup needs an integer")?.parse()?
            }
            "--trials" => {
                options.trials = args.next().ok_or("--trials needs an integer")?.parse()?
            }
            "--json" => options.json = Some(args.next().ok_or("--json needs a path")?.into()),
            "--help" | "-h" => {
                println!(
                    "Synthetic tracking-core benchmark\n  --frames N  timed frames per case/trial (default 10000)\n  --warmup N  untimed moving frames (default 1000)\n  --trials N  rotated-order trials (default 3)\n  --json PATH write complete JSON report\nUse --release. No physical/end-to-end claims."
                );
                std::process::exit(0);
            }
            _ => return Err(format!("Unknown argument: {arg}").into()),
        }
    }
    if !(100..=1_000_000).contains(&options.frames)
        || !(100..=100_000).contains(&options.warmup)
        || !(1..=20).contains(&options.trials)
    {
        return Err("Bounds: frames 100..1000000; warmup 100..100000; trials 1..20".into());
    }
    Ok(options)
}

fn configured(
    sensor_count: usize,
    correction: bool,
    estimator: bool,
) -> Result<TrackingCore, Box<dyn Error>> {
    let mut config = TrackingConfig {
        enable_joint_yaw_correction: correction,
        ..Default::default()
    };
    if estimator {
        let e = &mut config.estimator;
        e.enable_prediction = true;
        e.enable_hmd_extrapolation = true;
        e.enable_auto_rebase = true;
        e.enable_stance_heading = true;
        e.enable_leg_ik = true;
    }
    let mut core = TrackingCore::new(config)?;
    core.set_assignments(
        roles(sensor_count)
            .iter()
            .map(|role| RoleAssignment {
                sensor_id: sensor_id(*role),
                role: *role,
            })
            .collect(),
    )?;
    for frame in 0..=CALIBRATION_FRAMES {
        let now = frame as f64 / HZ;
        for role in roles(sensor_count) {
            core.ingest(sample(*role, now, Quat::IDENTITY))?;
        }
        black_box(core.solve(now, &anchors(now, 0.0, 0.0)));
    }
    let now = CALIBRATION_FRAMES as f64 / HZ;
    core.calibrate_neutral(now, &anchors(now, 0.0, 0.0), false)?;
    // These synthetic fixtures explicitly know each sensor's physical axes.
    for role in Role::ALL {
        if core
            .export_profile()
            .assignments
            .iter()
            .any(|assignment| assignment.role == role)
        {
            core.confirm_manual_mount(role)?;
        }
    }
    Ok(core)
}

fn roles(sensor_count: usize) -> &'static [Role] {
    if sensor_count == 10 {
        &Role::ALL
    } else {
        &Role::CORE
    }
}
fn sensor_id(role: Role) -> String {
    format!("synthetic-{role:?}")
}
fn sample(role: Role, now: f64, q: Quat) -> SensorSample {
    SensorSample {
        id: sensor_id(role),
        orientation_xyzw: q.to_array(),
        received_at_secs: now,
        // This orientation-only scenario intentionally omits accelerometer data;
        // zero acceleration during rotating/translating motion would be misleading.
        acceleration_mps2: None,
        provenance: Default::default(),
    }
}

fn anchors(now: f64, motion_time: f64, heading: f32) -> Anchors {
    let time = motion_time as f32;
    let head_position = Vec3::new(
        0.3 * (time * 0.3).sin(),
        1.65 + 0.02 * (time * 1.4).sin(),
        0.2 * (time * 0.23).sin(),
    );
    let rotation = Quat::from_rotation_y(heading);
    let head = AnchorPose {
        position: head_position.to_array(),
        orientation_xyzw: rotation.to_array(),
        received_at_secs: now,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    };
    let hand = |side: f32| AnchorPose {
        position: (head_position
            + rotation * Vec3::new(side * 0.35, -0.40 + 0.04 * (time * 1.8).sin(), -0.25))
        .to_array(),
        orientation_xyzw: rotation.to_array(),
        received_at_secs: now,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    };
    Anchors {
        hmd: Some(head),
        left_hand: Some(hand(-1.0)),
        right_hand: Some(hand(1.0)),
    }
}

fn truth(role: Role, motion_time: f64) -> Quat {
    let time = motion_time as f32;
    let heading = body_heading(motion_time);
    let yaw = Quat::from_rotation_y(heading);
    let side = match role {
        Role::LeftThigh | Role::LeftCalf | Role::LeftFoot | Role::LeftUpperArm => 0.0,
        _ => std::f32::consts::PI,
    };
    let swing = 18.0_f32.to_radians() * (time * 2.2 + side).sin();
    let flex = (45.0 + 18.0 * (time * 2.2 + side + 0.6).sin()).to_radians();
    let pitch = match role {
        Role::Chest => 5.0_f32.to_radians() * (time * 0.7).sin(),
        Role::Hip => 2.0_f32.to_radians() * (time * 0.7).sin(),
        Role::LeftThigh | Role::RightThigh => swing,
        Role::LeftCalf | Role::RightCalf => swing + flex,
        Role::LeftFoot | Role::RightFoot => 10.0_f32.to_radians() * (time * 2.2 + side).sin(),
        Role::LeftUpperArm | Role::RightUpperArm => -0.35 + 0.25 * (time * 1.6 + side).sin(),
    };
    yaw * Quat::from_rotation_x(pitch)
}

fn body_heading(time: f64) -> f32 {
    0.8 * (time as f32 * 0.17).sin()
}
fn injected_heading(time: f64) -> f32 {
    (time / 60.0).clamp(0.0, 1.0) as f32 * 15.0_f32.to_radians()
}

fn run_timing(case: Case, options: &Options) -> Result<(Vec<f64>, usize, f64), Box<dyn Error>> {
    let mut core = configured(case.sensor_count, case.correction, case.estimator)?;
    let mut samples = Vec::with_capacity(options.frames);
    let mut valid_frames = 0;
    let mut checksum = 0.0;
    for frame_number in 1..=options.warmup + options.frames {
        let motion_time = frame_number as f64 / HZ;
        let now = CALIBRATION_FRAMES as f64 / HZ + motion_time;
        for role in roles(case.sensor_count) {
            let drift = if *role == Role::LeftCalf {
                injected_heading(motion_time)
            } else {
                0.0
            };
            core.ingest(sample(
                *role,
                now,
                Quat::from_rotation_y(drift) * truth(*role, motion_time),
            ))?;
        }
        let input_anchors = anchors(
            now,
            motion_time,
            body_heading(motion_time) + 0.2 * (motion_time as f32 * 0.9).sin(),
        );
        if frame_number <= options.warmup {
            black_box(core.solve(now, &input_anchors));
            continue;
        }
        let start = Instant::now();
        let frame = core.solve(black_box(now), black_box(&input_anchors));
        black_box(&frame);
        let elapsed_us = start.elapsed().as_secs_f64() * 1_000_000.0;
        samples.push(elapsed_us);
        let expected_targets = if case.sensor_count == 10 { 8 } else { 6 };
        if frame.targets.len() == expected_targets
            && frame.targets.iter().all(|target| {
                target.valid
                    && target.position.iter().all(|v| v.is_finite())
                    && target.orientation_xyzw.iter().all(|v| v.is_finite())
            })
        {
            valid_frames += 1;
        }
        checksum += frame
            .targets
            .iter()
            .map(|target| target.position[1] as f64)
            .sum::<f64>();
    }
    Ok((samples, valid_frames, black_box(checksum)))
}

fn run_error(scenario: ErrorScenario, correction: bool) -> Result<ErrorMetrics, Box<dyn Error>> {
    let mut core = configured(6, correction, false)?;
    let mut relative_squared = 0.0;
    let mut thigh_squared = 0.0;
    let mut calf_squared = 0.0;
    let mut final_relative = 0.0;
    let mut final_thigh = 0.0;
    let mut final_calf = 0.0;
    let mut final_common = 0.0;
    let mut active = 0;
    let mut all_valid = true;
    for frame_number in 1..=ERROR_FRAMES {
        let time = frame_number as f64 / HZ;
        let now = CALIBRATION_FRAMES as f64 / HZ + time;
        let injected = injected_heading(time);
        for role in Role::CORE {
            let yaw = match scenario {
                ErrorScenario::RelativeDrift | ErrorScenario::AnatomicalTwist
                    if role == Role::LeftCalf =>
                {
                    injected
                }
                ErrorScenario::CommonDrift => injected,
                _ => 0.0,
            };
            core.ingest(sample(
                role,
                now,
                Quat::from_rotation_y(yaw) * truth(role, time),
            ))?;
        }
        let frame = core.solve(now, &anchors(now, time, body_heading(time)));
        all_valid &= frame.targets.iter().all(|target| target.valid);
        active += usize::from(
            frame
                .corrections
                .iter()
                .any(|d| d.applied_relative_degrees.abs() > 0.0),
        );
        let thigh_estimated = segment_heading(&frame, Role::LeftThigh);
        let calf_estimated = segment_heading(&frame, Role::LeftCalf);
        let thigh_true = projected_heading(truth(Role::LeftThigh, time));
        let calf_true = projected_heading(truth(Role::LeftCalf, time))
            + if matches!(scenario, ErrorScenario::AnatomicalTwist) {
                injected
            } else {
                0.0
            };
        let thigh_error = wrap(thigh_estimated - thigh_true).to_degrees() as f64;
        let calf_error = wrap(calf_estimated - calf_true).to_degrees() as f64;
        let relative_error =
            wrap((calf_estimated - thigh_estimated) - (calf_true - thigh_true)).to_degrees() as f64;
        relative_squared += relative_error * relative_error;
        thigh_squared += thigh_error * thigh_error;
        calf_squared += calf_error * calf_error;
        if frame_number > ERROR_FRAMES - ERROR_FINAL_WINDOW_FRAMES {
            final_relative += relative_error.abs();
            final_thigh += thigh_error.abs();
            final_calf += calf_error.abs();
            final_common += ((thigh_error + calf_error) * 0.5).abs();
        }
    }
    Ok(ErrorMetrics {
        sample_count: ERROR_FRAMES,
        relative_knee_heading_rmse_deg: (relative_squared / ERROR_FRAMES as f64).sqrt(),
        thigh_heading_rmse_deg: (thigh_squared / ERROR_FRAMES as f64).sqrt(),
        calf_heading_rmse_deg: (calf_squared / ERROR_FRAMES as f64).sqrt(),
        final_window_relative_knee_heading_mae_deg: final_relative
            / ERROR_FINAL_WINDOW_FRAMES as f64,
        final_window_thigh_heading_mae_deg: final_thigh / ERROR_FINAL_WINDOW_FRAMES as f64,
        final_window_calf_heading_mae_deg: final_calf / ERROR_FINAL_WINDOW_FRAMES as f64,
        final_window_pair_common_heading_mae_deg: final_common / ERROR_FINAL_WINDOW_FRAMES as f64,
        correction_active_frames: active,
        all_targets_valid: all_valid,
    })
}

fn segment_heading(frame: &TrackingFrame, role: Role) -> f32 {
    projected_heading(Quat::from_array(
        frame
            .segments
            .iter()
            .find(|segment| segment.role == role)
            .unwrap()
            .orientation_xyzw,
    ))
}
fn projected_heading(rotation: Quat) -> f32 {
    let axis = rotation * Vec3::X;
    (-axis.z).atan2(axis.x)
}
fn wrap(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn measure_timer_overhead(count: usize) -> Distribution {
    let mut times = Vec::with_capacity(count);
    for value in 0..count {
        let start = Instant::now();
        black_box(value);
        times.push(start.elapsed().as_secs_f64() * 1_000_000.0);
    }
    distribution(&times)
}
fn distribution(samples: &[f64]) -> Distribution {
    let mut sorted = samples.to_vec();
    sorted.sort_by(f64::total_cmp);
    let percentile = |p: f64| {
        let at = p * (sorted.len() - 1) as f64;
        let lower = at.floor() as usize;
        let upper = at.ceil() as usize;
        sorted[lower] + (sorted[upper] - sorted[lower]) * at.fract()
    };
    Distribution {
        count: samples.len(),
        mean_us: samples.iter().sum::<f64>() / samples.len() as f64,
        p50_us: percentile(0.50),
        p95_us: percentile(0.95),
        p99_us: percentile(0.99),
        max_us: *sorted.last().unwrap(),
    }
}

fn validate_report(report: &BenchmarkReport) -> Result<(), Box<dyn Error>> {
    if report
        .timings
        .iter()
        .any(|case| case.valid_frames != case.expected_valid_frames || !case.checksum.is_finite())
    {
        return Err("Benchmark generated invalid/nonfinite expected body targets".into());
    }
    if report
        .synthetic_error_cases
        .iter()
        .any(|case| !case.baseline.all_targets_valid || !case.experimental.all_targets_valid)
    {
        return Err("Synthetic error scenario lost valid input/output unexpectedly".into());
    }
    let drift = &report.synthetic_error_cases[0];
    if drift
        .experimental
        .final_window_relative_knee_heading_mae_deg
        >= drift.baseline.final_window_relative_knee_heading_mae_deg
    {
        return Err("Synthetic excited relative-drift scenario did not reduce relative error; inspect benchmark/core compatibility".into());
    }
    let turn = &report.synthetic_error_cases[3];
    if turn.experimental.final_window_thigh_heading_mae_deg > 0.01
        || turn.experimental.final_window_calf_heading_mae_deg > 0.01
    {
        return Err("Synthetic genuine global turn was changed by the estimator".into());
    }
    Ok(())
}

fn print_report(report: &BenchmarkReport) {
    println!("ChadFBT deterministic synthetic tracking-core benchmark");
    println!(
        "{} {}, release={}, 100 Hz input timestamps, {} warmup + {} timed frames/trial, {} trials",
        report.operating_system,
        report.architecture,
        !report.debug_assertions,
        report.warmup_frames_per_trial,
        report.timing_frames_per_trial,
        report.trial_count
    );
    println!(
        "Unpaced solve-only elapsed wall time; excludes input ingest, output drop, UI, network and driver."
    );
    println!(
        "Timer pair p50 {:.3} us / p99 {:.3} us (not subtracted).",
        report.timer_pair_overhead.p50_us, report.timer_pair_overhead.p99_us
    );
    println!(
        "sensors correction estimator    p50_us    p95_us    p99_us     max_us     valid/total"
    );
    let on_off = |on: bool| if on { "on" } else { "off" };
    for case in &report.timings {
        let d = &case.combined_solve_time;
        println!(
            "{:>7} {:>10} {:>9} {:>9.3} {:>9.3} {:>9.3} {:>10.3} {:>8}/{}",
            case.sensor_count,
            on_off(case.experimental_correction_enabled),
            on_off(case.estimator_layers_enabled),
            d.p50_us,
            d.p95_us,
            d.p99_us,
            d.max_us,
            case.valid_frames,
            case.expected_valid_frames
        );
    }
    println!("\nPaired 120-second synthetic scenarios; final 10-second heading MAE (degrees):");
    for case in &report.synthetic_error_cases {
        println!("{}", case.name);
        for (name, metrics) in [
            ("baseline", &case.baseline),
            ("experimental", &case.experimental),
        ] {
            println!(
                "  {:>12}: relative={:.4}, thigh={:.4}, calf={:.4}, pair_common={:.4}, relative_RMSE={:.4}",
                name,
                metrics.final_window_relative_knee_heading_mae_deg,
                metrics.final_window_thigh_heading_mae_deg,
                metrics.final_window_calf_heading_mae_deg,
                metrics.final_window_pair_common_heading_mae_deg,
                metrics.relative_knee_heading_rmse_deg
            );
        }
    }
    println!(
        "\nRelative improvement assumes ideal hinge motion. Identical data interpreted as real calf twist exposes false correction."
    );
    println!(
        "No physical accuracy, end-to-end latency, or superiority over SlimeVR is established."
    );
}
