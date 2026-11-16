//! Synthetic motion-calibration scenarios. Deterministic model-conditional results, NOT
//! worn-body accuracy or a comparison with SlimeVR.
//! cargo run --release -p tracking-core --example motion_calibration

#[path = "../tests/motion_support/mod.rs"]
mod motion_support;

use motion_support::*;
use tracking_core::Role;
use tracking_core::forward_calibration::solve_forward_alignment;
use tracking_core::motion_calibration::{MotionCalibrationConfig, MotionGate, SignEvidence};
use tracking_core::synthetic::Motion;

fn main() {
    let rig = Rig::standard(7);
    let base = RunOptions::default();
    let mut rows: Vec<(String, RunResult)> = Vec::new();
    let mut add = |name: &str, source: &dyn Source, rig: &Rig, options: &RunOptions| {
        rows.push((name.to_owned(), run(source, rig, options)));
    };

    add("squats x3", &squats(), &rig, &base);
    add("bow x2 + march x4", &bow_then_march(), &rig, &base);
    add("held skiing pose", &held_ski(), &rig, &base);
    let twisted_hold = Parametric::new(
        HEADING,
        vec![stand(1.0), hold_twisted(SKI_TILTS, 15.0, 4.0)],
    );
    add("held skiing pose, 15 deg twist", &twisted_hold, &rig, &base);
    let twisted_march = Modified {
        inner: bow_then_march(),
        mods: Mods {
            twist_degrees: 15.0,
            twist_period: 1.7,
            ..Mods::default()
        },
    };
    add(
        "bow + march, 15 deg bone twist",
        &twisted_march,
        &rig,
        &base,
    );
    let twisted = Modified {
        inner: squats(),
        mods: Mods {
            twist_degrees: 15.0,
            twist_period: 1.7,
            ..Mods::default()
        },
    };
    add("squats, 15 deg bone twist", &twisted, &rig, &base);
    for (name, period) in [
        ("squats, 10 deg yaw wobble 2.3 s", 2.3),
        ("squats, 10 deg yaw wobble 9 s", 9.0),
    ] {
        let wobble = Modified {
            inner: squats(),
            mods: Mods {
                wobble_degrees: 10.0,
                wobble_period: period,
                ..Mods::default()
            },
        };
        add(name, &wobble, &rig, &base);
    }
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
    add("squats, 40 deg turn mid-way", &turned, &rig, &base);
    for (name, thigh_only) in [
        ("knees-out squat 15 deg (leg plane)", false),
        ("knees-out squat 15 deg (thigh only)", true),
    ] {
        let knees = Modified {
            inner: squats(),
            mods: Mods {
                knees_out_degrees: 15.0,
                knees_out_thigh_only: thigh_only,
                ..Mods::default()
            },
        };
        add(name, &knees, &rig, &base);
    }
    let toe_out = Synthetic::with_toe_out(
        vec![Motion::stand(1.0), Motion::squats(3), Motion::stand(1.0)],
        HEADING,
        5.0,
    );
    add(
        "squats, 5 deg toe-out (synthetic default)",
        &toe_out,
        &rig,
        &base,
    );
    let nine = || {
        Synthetic::new(
            vec![Motion::stand(1.0), Motion::squats(9), Motion::stand(1.0)],
            HEADING,
        )
    };
    for (name, memory) in [
        ("squats x9, first 3.5 s torso 45 deg off", 10.0),
        ("  same without forgetting", 3600.0),
    ] {
        let bad = Modified {
            inner: nine(),
            mods: Mods {
                torso_bend_yaw_degrees: 45.0,
                torso_bend_until: Some(4.0),
                ..Mods::default()
            },
        };
        let options = RunOptions {
            config: MotionCalibrationConfig {
                evidence_memory_secs: memory,
                ..MotionCalibrationConfig::default()
            },
            ..RunOptions::default()
        };
        add(name, &bad, &rig, &options);
    }
    let mut noisy = rig.clone();
    noisy.noise_degrees = 1.0;
    let async_options = RunOptions {
        rate_hz: 90.0,
        stagger_secs: 0.004,
        drop_every: 7,
        drop_role: 4,
        ..RunOptions::default()
    };
    add(
        "squats, async 90 Hz, 1 deg noise, drops",
        &squats(),
        &noisy,
        &async_options,
    );
    let lean = Modified {
        inner: squats(),
        mods: Mods {
            torso_bend_yaw_degrees: -90.0,
            ..Mods::default()
        },
    };
    add("squats, torso bends sideways", &lean, &rig, &base);
    let headset_options = RunOptions {
        headset: true,
        reference_from_head: true,
        ..RunOptions::default()
    };
    add(
        "squats, torso sideways + headset",
        &lean,
        &rig,
        &headset_options,
    );
    let thigh_side = Modified {
        inner: squats(),
        mods: Mods {
            knees_out_degrees: 40.0,
            knees_out_thigh_only: true,
            ..Mods::default()
        },
    };
    add("squats, thighs 40 deg sideways", &thigh_side, &rig, &base);
    let mut swapped = rig.clone();
    swapped.source_bone = [0, 1, 4, 3, 2, 5];
    add(
        "squats, left thigh/calf swapped",
        &squats(),
        &swapped,
        &base,
    );
    let mut left_right = rig.clone();
    left_right.source_bone = [0, 1, 3, 2, 4, 5];
    add(
        "bow + march, thighs L/R swapped",
        &bow_then_march(),
        &left_right,
        &base,
    );
    add("walking, knee rule on", &walking(), &rig, &base);
    let no_knee = RunOptions {
        config: MotionCalibrationConfig {
            knee_sign_rule: false,
            ..MotionCalibrationConfig::default()
        },
        ..RunOptions::default()
    };
    add("walking, knee rule off", &walking(), &rig, &no_knee);
    let weak_pelvis = Parametric::new(
        HEADING,
        vec![stand(1.0), bows(2, 60.0, 4.0), march(4), stand(1.0)],
    );
    add("bow (pelvis 4 deg) + march", &weak_pelvis, &rig, &base);
    for head in [0.0_f32, 20.0, 40.0] {
        let bow = Modified {
            inner: headset_bow(),
            mods: Mods {
                head_yaw_degrees: head,
                ..Mods::default()
            },
        };
        let name = format!("headset bow, head turned {head} deg");
        add(&name, &bow, &rig, &headset_options);
    }

    println!(
        "{:<42} {:>5} {:>6} {:>8}  {:<44} gates / warnings",
        "scenario", "ready", "at (s)", "max|err|", "forward error C H LT RT LC RC (deg)"
    );
    for (name, result) in &rows {
        let errors = result.forward_errors_degrees();
        let max = errors.iter().flatten().fold(None, |m: Option<f32>, v| {
            Some(m.map_or(v.abs(), |m| m.max(v.abs())))
        });
        let cells: Vec<String> = errors
            .iter()
            .zip(&result.status.segments)
            .map(|(e, s)| match (e, s.sign) {
                (Some(e), SignEvidence::KneeRule { .. }) => format!("{e:+.1}k"),
                (Some(e), _) => format!("{e:+.1}"),
                (None, _) => "-".into(),
            })
            .collect();
        let mut gates: Vec<String> = Vec::new();
        for segment in &result.status.segments {
            for blocker in &segment.blockers {
                if blocker.gate != MotionGate::Stability || segment.blockers.len() == 1 {
                    gates.push(format!("{}:{:?}", short(segment.role), blocker.gate));
                }
            }
        }
        for blocker in &result.status.blockers {
            gates.push(format!("{:?}", blocker.gate));
        }
        for warning in &result.status.warnings {
            gates.push(format!("warn:{:?}", warning.kind));
        }
        println!(
            "{:<42} {:>5} {:>6} {:>8}  {:<44} {}",
            name,
            result.status.ready,
            result
                .ready_at
                .map_or("-".into(), |t| format!("{:.1}", t - base.neutral_at)),
            max.map_or("-".into(), |m| format!("{m:.2}")),
            cells.join(" "),
            gates.join(", ")
        );
        if let Some(headset) = &result.status.headset {
            let truth = result.body_heading;
            let deg = |v: Option<f32>| {
                v.map_or("-".into(), |v| {
                    format!("{:+.1}", wrap_degrees((v - truth).to_degrees()))
                })
            };
            println!(
                "{:<42}              headset: displacement err {} (sigma {}), yaw err {:+.1}, fused err {}, flagged {}, max disp {:.3} m",
                "",
                deg(headset.displacement_heading_radians),
                headset
                    .displacement_sigma_radians
                    .map_or("-".into(), |s| format!("{:.1}", s.to_degrees())),
                wrap_degrees((result.reference_heading - truth).to_degrees()),
                deg(headset.fused_heading_radians),
                headset.disagreement_flagged,
                headset.max_displacement_m,
            );
        }
    }
    println!(
        "\nErrors compare each segment's solved body-forward direction with the generator truth \
         (k = sign from the knee rule); 'at' is the first 0.25 s check reporting ready, measured \
         from neutral. Synthetic, model-conditional results only; nothing here is validated on a \
         worn body."
    );
    println!(
        "\nRotation-axis (two-pose) solve on the held skiing pose, heading error per segment:"
    );
    for twist in [0.0_f32, 6.0, 15.0] {
        let source = Parametric::new(
            HEADING,
            vec![stand(1.0), hold_twisted(SKI_TILTS, twist, 4.0)],
        );
        let result = run(&source, &rig, &base);
        let truth = source.truth(source.duration());
        let errors: Vec<String> = Role::CORE
            .iter()
            .enumerate()
            .map(|(i, role)| {
                solve_forward_alignment(
                    *role,
                    result.neutral[i].to_array(),
                    rig.clean(&truth, i).to_array(),
                    result.reference_heading,
                )
                .map_or("rejected".into(), |a| {
                    format!(
                        "{:+.1}",
                        wrap_degrees((a.heading_alignment_radians - rig.psi[i]).to_degrees())
                    )
                })
            })
            .collect();
        println!("  twist {twist:>4} deg: {}", errors.join(" "));
    }
}

fn short(role: Role) -> &'static str {
    match role {
        Role::Chest => "C",
        Role::Hip => "H",
        Role::LeftThigh => "LT",
        Role::RightThigh => "RT",
        Role::LeftCalf => "LC",
        Role::RightCalf => "RC",
        _ => "?",
    }
}
