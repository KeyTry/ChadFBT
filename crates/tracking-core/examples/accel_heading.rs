//! Deterministic synthetic scenarios for the acceleration-heading shadow
//! estimator. Toy rigid-body motion, not hardware accuracy or latency claims.
//! cargo run --release -p tracking-core --example accel_heading -- --json path.json

use glam::{Quat, Vec3};
use serde::Serialize;
use std::error::Error;
use tracking_core::accel_heading::synthetic::*;
use tracking_core::accel_heading::*;

struct Case {
    name: &'static str,
    scenario: SyntheticScenario,
    config: AccelHeadingConfig,
    hint: Hint,
}

#[derive(Clone, Copy)]
enum Hint {
    None,
    True,
    /// The true frame rotated 180 degrees about the sensor's vertical axis:
    /// inside the vertical ambiguity group of an upright sensor.
    InsideGroup,
    /// A frame whose vertical acceleration differs.
    OutsideGroup,
}

#[derive(Serialize)]
struct Row {
    scenario: &'static str,
    true_frame: usize,
    hint: Option<usize>,
    nominal_lag_secs: f64,
    expected_correction_degrees: f64,
    first_candidate_at_secs: Option<f64>,
    candidate_reports: usize,
    reports: usize,
    /// Worst |error| over every Candidate report of the run.
    worst_candidate_error_degrees: Option<f64>,
    /// Error of the last report's fit, even when withheld.
    fit_error_degrees: Option<f64>,
    report: AccelHeadingReport,
}

fn case(name: &'static str, scenario: SyntheticScenario, hint: Hint) -> Case {
    Case {
        name,
        scenario,
        config: AccelHeadingConfig::default(),
        hint,
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !args.is_empty() && (args.len() != 2 || args[0] != "--json") {
        return Err("usage: accel_heading [--json output.json]".into());
    }
    let base = SyntheticScenario::default();
    let upright = SyntheticScenario {
        motion: SyntheticMotion::Walk,
        torso_tilt_degrees: 2.0,
        mount: Quat::IDENTITY,
        ..base.clone()
    };
    let open_gates = AccelHeadingConfig {
        max_reference_angular_speed_rad_per_sec: 10.0,
        max_imu_angular_speed_rad_per_sec: 10.0,
        ..AccelHeadingConfig::default()
    };
    let fast_head = SyntheticScenario {
        head_rotation_degrees: 30.0,
        head_rotation_hz: 0.8,
        reference: SyntheticReference::HeadsetEye,
        ..base.clone()
    };
    let chip_z_frame = frame_candidate_index(Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2))
        .ok_or("no chip-Z candidate")?;
    let mut cases = vec![
        case("dance, tilt 25, yaw +25", base.clone(), Hint::None),
        case(
            "dance, tilt 25, yaw -25",
            SyntheticScenario {
                yaw_error_degrees: -25.0,
                seed: 2,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, yaw +25, seed 3",
            SyntheticScenario {
                seed: 3,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, noise 1.0 m/s2",
            SyntheticScenario {
                accel_noise_mps2: 1.0,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, bias (0.8,-0.5,0.6) m/s2",
            SyntheticScenario {
                accel_bias_mps2: [0.8, -0.5, 0.6],
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, lag 40 ms",
            SyntheticScenario {
                imu_lag_secs: 0.04,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, jitter 15 ms",
            SyntheticScenario {
                network_jitter_secs: 0.015,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, position-only reference",
            SyntheticScenario {
                provide_reference_velocity: false,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "dance, fw 0.7.4 (C = identity)",
            SyntheticScenario {
                true_frame: 0,
                ..base.clone()
            },
            Hint::None,
        ),
        case("walk upright, no hint", upright.clone(), Hint::None),
        case("walk upright, true hint", upright.clone(), Hint::True),
        case(
            "walk upright, wrong hint outside",
            upright.clone(),
            Hint::OutsideGroup,
        ),
        case(
            "walk upright, wrong hint in group",
            upright.clone(),
            Hint::InsideGroup,
        ),
        case(
            "walk, mount lean 8, no hint",
            SyntheticScenario {
                mount: Quat::from_rotation_x(-8.0_f32.to_radians()),
                ..upright.clone()
            },
            Hint::None,
        ),
        Case {
            name: "walk upright, C among chip-Z rotations",
            scenario: SyntheticScenario {
                true_frame: chip_z_frame,
                ..upright.clone()
            },
            config: AccelHeadingConfig {
                allowed_frame_candidates: frame_candidates_fixing_axis(Vec3::Z),
                ..AccelHeadingConfig::default()
            },
            hint: Hint::None,
        },
        case(
            "standing still",
            SyntheticScenario {
                motion: SyntheticMotion::Still,
                torso_tilt_degrees: 1.0,
                head_rotation_degrees: 3.0,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "torso independent of head",
            SyntheticScenario {
                motion: SyntheticMotion::IndependentTorso,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "horizontal independent, true hint",
            SyntheticScenario {
                motion: SyntheticMotion::IndependentHorizontal,
                ..base.clone()
            },
            Hint::True,
        ),
        case(
            "head 15 deg 0.5 Hz, neck ref",
            SyntheticScenario {
                head_rotation_degrees: 15.0,
                ..base.clone()
            },
            Hint::None,
        ),
        case(
            "head 15 deg 0.5 Hz, eye ref",
            SyntheticScenario {
                head_rotation_degrees: 15.0,
                reference: SyntheticReference::HeadsetEye,
                ..base.clone()
            },
            Hint::None,
        ),
        case("head 30 deg 0.8 Hz, eye ref", fast_head.clone(), Hint::None),
        Case {
            name: "head 30 deg 0.8 Hz, eye, gates open",
            scenario: fast_head.clone(),
            config: open_gates.clone(),
            hint: Hint::None,
        },
        Case {
            name: "head 30 deg 0.8 Hz, neck, gates open",
            scenario: SyntheticScenario {
                reference: SyntheticReference::NeckModel {
                    offset_error_m: [0.0, 0.02, 0.03],
                },
                ..fast_head.clone()
            },
            config: open_gates,
            hint: Hint::None,
        },
        Case {
            name: "thigh swing, kinematic ref",
            scenario: SyntheticScenario {
                sensor: SyntheticSensor::Thigh,
                motion: SyntheticMotion::Walk,
                torso_tilt_degrees: 0.0,
                ..base.clone()
            },
            config: AccelHeadingConfig {
                horizontal_reference: false,
                max_reference_angular_speed_rad_per_sec: 8.0,
                max_imu_angular_speed_rad_per_sec: 8.0,
                ..AccelHeadingConfig::default()
            },
            hint: Hint::None,
        },
    ];

    let mut rows = Vec::new();
    for case in cases.drain(..) {
        let run = generate(&case.scenario);
        let mut estimator = AccelHeadingEstimator::new(case.config.clone())?;
        let hint = hint_index(case.hint, run.true_frame);
        estimator.set_frame_hint(hint)?;
        let mut next_report = 1.0;
        let mut first_candidate = None;
        let mut candidate_reports = 0;
        let mut reports = 0;
        let mut worst: Option<f64> = None;
        let mut last = None;
        for event in &run.events {
            event.push_into(&mut estimator);
            if event.at_secs() >= next_report {
                let report = estimator.report(event.at_secs());
                reports += 1;
                if let Some(yaw) = report.yaw_correction_radians {
                    candidate_reports += 1;
                    first_candidate.get_or_insert(event.at_secs());
                    let error = wrap_degrees((yaw - run.expected_correction_radians).to_degrees());
                    worst = Some(worst.map_or(error.abs(), |w| w.max(error.abs())));
                }
                next_report += 0.5;
                last = Some(report);
            }
        }
        let Some(report) = last else {
            continue;
        };
        let fit_error_degrees = report
            .horizontal
            .as_ref()
            .map(|h| wrap_degrees((h.yaw_radians - run.expected_correction_radians).to_degrees()));
        rows.push(Row {
            scenario: case.name,
            true_frame: run.true_frame,
            hint,
            nominal_lag_secs: run.nominal_lag_secs,
            expected_correction_degrees: run.expected_correction_radians.to_degrees(),
            first_candidate_at_secs: first_candidate,
            candidate_reports,
            reports,
            worst_candidate_error_degrees: worst,
            fit_error_degrees,
            report,
        });
    }

    println!(
        "Synthetic toy body: IMU 100 Hz (+0-4 ms jitter, accel one sample old), reference 200 Hz, 0.3 m/s2 noise,"
    );
    println!(
        "20 s runs, report every 0.5 s, 6 s window; columns describe the LAST report. Not a hardware result."
    );
    println!(
        "{:<38} {:>19} {:>6} {:>4} {:>6} {:>6} {:>10} {:>7} {:>7} {:>6} {:>6} {:>4} {:>5} {:>6}",
        "scenario",
        "status",
        "frame",
        "best",
        "v.agr",
        "margin",
        "lag/nom ms",
        "fit yaw",
        "fit err",
        "sigma",
        "rho_h",
        "Neff",
        "1st",
        "worst"
    );
    for row in &rows {
        let r = &row.report;
        println!(
            "{:<38} {:>19} {:>6} {:>4} {:>6} {:>6} {:>10} {:>7} {:>7} {:>6} {:>6} {:>4.0} {:>5} {:>6}",
            row.scenario,
            format!("{:?}", r.status),
            format!(
                "{}/{}",
                r.frame.candidate.map_or("-".into(), |c| c.to_string()),
                row.true_frame
            ),
            r.frame.best_candidate.map_or("-".into(), |c| c.to_string()),
            fmt(r.frame.vertical_agreement, 3),
            fmt(r.frame.margin, 3),
            format!(
                "{}/{:.0}",
                fmt(r.lag_secs.map(|l| l * 1000.0), 1),
                row.nominal_lag_secs * 1000.0
            ),
            fmt(r.horizontal.as_ref().map(|h| h.yaw_radians.to_degrees()), 1),
            fmt(row.fit_error_degrees, 2),
            fmt(
                r.horizontal.as_ref().map(|h| h.sigma_radians.to_degrees()),
                2
            ),
            fmt(r.horizontal_correlation, 3),
            r.effective_samples,
            fmt(row.first_candidate_at_secs, 1),
            fmt(row.worst_candidate_error_degrees, 2),
        );
    }
    println!();
    for row in &rows {
        let r = &row.report;
        if r.frame.ambiguous_candidates.len() > 1 {
            println!(
                "{}: ambiguous candidates {:?}",
                row.scenario, r.frame.ambiguous_candidates
            );
        }
        if r.status == AccelHeadingStatus::InsufficientData {
            println!("{}: {}", row.scenario, r.reason);
        }
    }
    if let Some(path) = args.get(1) {
        let path = std::path::Path::new(path);
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(&rows)?)?;
        println!("Saved {}", path.display());
    }
    Ok(())
}

fn hint_index(hint: Hint, true_frame: usize) -> Option<usize> {
    let truth = frame_candidate(true_frame)?;
    match hint {
        Hint::None => None,
        Hint::True => Some(true_frame),
        Hint::InsideGroup => {
            frame_candidate_index(Quat::from_rotation_y(std::f32::consts::PI) * truth)
        }
        Hint::OutsideGroup => {
            frame_candidate_index(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2) * truth)
        }
    }
}

fn wrap_degrees(angle: f64) -> f64 {
    (angle + 180.0).rem_euclid(360.0) - 180.0
}

fn fmt(value: Option<f64>, digits: usize) -> String {
    value
        .map(|v| format!("{v:.digits$}"))
        .unwrap_or_else(|| "-".into())
}
