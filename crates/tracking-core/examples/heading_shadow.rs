//! Deterministic mathematical scenarios, not hardware accuracy or latency claims.
//! cargo run --release -p tracking-core --example heading_shadow -- --json path.json

use glam::Quat;
use serde::Serialize;
use std::error::Error;
use tracking_core::heading_shadow::*;
use tracking_core::{Role, SensorProvenance};

#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    observed_relative_yaw_degrees: f64,
    true_parent_error_degrees: f64,
    true_child_error_degrees: f64,
    global_turn: bool,
    vertical_axis: bool,
    outliers: bool,
    witness_degrees: Option<f64>,
}

#[derive(Serialize)]
struct ResultRow {
    scenario: &'static str,
    baseline_relative_error_degrees: f64,
    /// Applies even to a vetoed candidate to reveal what blindly applying it does.
    candidate_if_forced_relative_error_degrees: Option<f64>,
    candidate_if_forced_segment_rms_error_degrees: Option<f64>,
    candidate_degrees: Option<f64>,
    median_axis_residual_degrees: Option<f64>,
    report: HeadingShadowReport,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !args.is_empty() && (args.len() != 2 || args[0] != "--json") {
        return Err("usage: heading_shadow [--json output.json]".into());
    }
    let baseline = Scenario {
        name: "relative drift 15 deg",
        observed_relative_yaw_degrees: 15.0,
        true_parent_error_degrees: 0.0,
        true_child_error_degrees: 15.0,
        global_turn: false,
        vertical_axis: false,
        outliers: false,
        witness_degrees: None,
    };
    let cases = [
        baseline,
        Scenario {
            name: "same observations, real twist",
            true_child_error_degrees: 0.0,
            ..baseline
        },
        Scenario {
            name: "drift with independent witness",
            witness_degrees: Some(-15.0),
            ..baseline
        },
        Scenario {
            name: "real twist independently vetoed",
            true_child_error_degrees: 0.0,
            witness_degrees: Some(0.0),
            ..baseline
        },
        Scenario {
            name: "common yaw drift 20 deg",
            observed_relative_yaw_degrees: 0.0,
            true_parent_error_degrees: 20.0,
            true_child_error_degrees: 20.0,
            ..baseline
        },
        Scenario {
            name: "true global turn",
            observed_relative_yaw_degrees: 0.0,
            true_child_error_degrees: 0.0,
            global_turn: true,
            ..baseline
        },
        Scenario {
            name: "vertical hinge axis",
            observed_relative_yaw_degrees: 0.0,
            true_child_error_degrees: 0.0,
            vertical_axis: true,
            ..baseline
        },
        Scenario {
            name: "drift with 10% 45 deg outliers",
            outliers: true,
            ..baseline
        },
    ];
    let mut rows = Vec::new();
    for case in cases {
        let mut estimator = RelativeHeadingEstimator::new(
            Role::LeftThigh,
            Role::LeftCalf,
            HeadingShadowConfig::default(),
        )?;
        for i in 0..=200 {
            let t = i as f64 * 0.01;
            estimator.observe(t, sample(case, i, t));
        }
        let report = estimator.report(2.0);
        let candidate = report.candidate_relative_yaw_radians.map(f64::to_degrees);
        let baseline_relative = case.true_child_error_degrees - case.true_parent_error_degrees;
        let after = candidate.map(|delta| (baseline_relative + delta).abs());
        let rms = candidate.map(|delta| {
            let p = case.true_parent_error_degrees - delta / 2.0;
            let c = case.true_child_error_degrees + delta / 2.0;
            ((p * p + c * c) / 2.0).sqrt()
        });
        rows.push(ResultRow {
            scenario: case.name,
            baseline_relative_error_degrees: baseline_relative.abs(),
            candidate_if_forced_relative_error_degrees: after,
            candidate_if_forced_segment_rms_error_degrees: rms,
            candidate_degrees: candidate,
            median_axis_residual_degrees: report
                .fit
                .as_ref()
                .map(|f| f.median_residual_radians.to_degrees()),
            report,
        });
    }
    println!(
        "Synthetic 100 Hz timestamps, 2 second windows; no live corrections, hardware accuracy or timing claims."
    );
    println!(
        "{:<34} {:>10} {:>10} {:>11} {:>10} {:>10} status",
        "scenario", "candidate", "residual", "eigenvalue", "base err", "forced err"
    );
    for row in &rows {
        let eigen = row
            .report
            .fit
            .as_ref()
            .map(|f| f.observability.frozen_weight_hessian_eigenvalues[1]);
        println!(
            "{:<34} {:>10} {:>10} {:>11} {:>10.4} {:>10} {:?}",
            row.scenario,
            formatted(row.candidate_degrees),
            formatted(row.median_axis_residual_degrees),
            formatted(eigen),
            row.baseline_relative_error_degrees,
            formatted(row.candidate_if_forced_relative_error_degrees),
            row.report.status
        );
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

fn formatted(value: Option<f64>) -> String {
    value
        .map(|v| format!("{v:.4}"))
        .unwrap_or_else(|| "n/a".into())
}

fn sample(case: Scenario, i: usize, t: f64) -> HeadingPairSample {
    let common_error = case.true_parent_error_degrees.to_radians();
    let global = if case.global_turn { 2.4 * t } else { 0.0 } + common_error;
    let q = Quat::from_rotation_y(global as f32)
        * Quat::from_rotation_x((0.12 * (t * 2.1).sin()) as f32);
    let observed_offset = case.observed_relative_yaw_degrees
        + if case.outliers && i % 10 == 5 {
            45.0
        } else {
            0.0
        };
    let c = Quat::from_rotation_y(observed_offset.to_radians() as f32)
        * q
        * Quat::from_rotation_x((0.75 + 0.35 * (t * 5.0).sin()) as f32);
    let axis_rotation = if case.vertical_axis {
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)
    } else {
        Quat::IDENTITY
    };
    let segment = |role, rotation: Quat| HeadingSegmentSample {
        role,
        orientation_xyzw: (axis_rotation * rotation).to_array(),
        measured_at_secs: t,
        confidence: 0.95,
        valid: true,
        provenance: SensorProvenance::Measured,
    };
    HeadingPairSample {
        calibration_epoch: 1,
        continuity_epoch: 1,
        parent: segment(Role::LeftThigh, q),
        child: segment(Role::LeftCalf, c),
        independent_witness: case
            .witness_degrees
            .map(|degrees| IndependentHeadingWitness {
                relative_correction_radians: degrees.to_radians(),
                error_bound_radians: 1.0_f64.to_radians(),
                observed_at_secs: t,
                source: IndependentWitnessSource::ExternalRelativeHeading,
                caller_asserts_independent_physical_constraint: true,
            }),
    }
}
