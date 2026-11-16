//! Synthetic chair geometry; no observations of a person are used.
//! Stationary is not the same evidence as contact with the floor.
use glam::{Quat, Vec3};
use tracking_core::Anthropometrics;
use tracking_core::leg_ik::{LegIkConfig, LegIkInput, LegIkOutput, LegIkSolver};

const DT: f64 = 0.01;

/// Seated on a chair: thighs horizontal toward -Z, shanks vertical, ankles
/// `clearance` above their floor height. A positive `torso_tilt` (radians)
/// reclines the torso backward (toward +Z); `FRAC_PI_2` is lying supine.
fn chair(at_secs: f64, clearance: f32, torso_tilt: f32) -> LegIkInput {
    let d = Anthropometrics::default();
    let thigh = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let torso = Quat::from_rotation_x(torso_tilt);
    let hip = Vec3::new(0.0, d.ankle_to_sole_m + d.calf_length_m + clearance, 0.0);
    let hip_joints = [-1.0_f32, 1.0].map(|side| hip + Vec3::X * side * d.hip_width_m * 0.5);
    let knees = hip_joints.map(|joint| joint - thigh * Vec3::Y * d.thigh_length_m);
    let ankles = knees.map(|knee| knee - Vec3::Y * d.calf_length_m);
    let chest = hip + torso * Vec3::Y * d.chest_to_hip_m;
    LegIkInput {
        at_secs,
        neck: chest + torso * Vec3::Y * d.neck_to_chest_m,
        chest,
        hip,
        hip_joints,
        knees,
        ankles,
        chest_rotation: torso,
        spine_rotation: torso,
        hip_rotation: Quat::IDENTITY,
        thigh_rotations: [thigh; 2],
        calf_rotations: [Quat::IDENTITY; 2],
        measured_foot_rotations: [None; 2],
        planted: [true; 2],
        neck_to_chest_m: d.neck_to_chest_m,
        chest_to_hip_m: d.chest_to_hip_m,
        hip_width_m: d.hip_width_m,
        thigh_m: d.thigh_length_m,
        calf_m: d.calf_length_m,
        ankle_to_sole_m: d.ankle_to_sole_m,
        enabled: true,
    }
}

/// Rigidly translate the whole FK skeleton (a head-down chain error).
fn shifted(mut pose: LegIkInput, delta: Vec3) -> LegIkInput {
    pose.neck += delta;
    pose.chest += delta;
    pose.hip += delta;
    for s in 0..2 {
        pose.hip_joints[s] += delta;
        pose.knees[s] += delta;
        pose.ankles[s] += delta;
    }
    pose
}

/// Both legs straight and horizontal toward -Z from a pelvis at `hip_height`:
/// lying supine (`torso_tilt = FRAC_PI_2`) or long sitting (`torso_tilt = 0`).
fn outstretched(at_secs: f64, hip_height: f32, torso_tilt: f32) -> LegIkInput {
    let seated = chair(at_secs, 0.0, torso_tilt);
    let mut pose = shifted(seated, Vec3::Y * (hip_height - seated.hip.y));
    let flat = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    pose.thigh_rotations = [flat; 2];
    pose.calf_rotations = [flat; 2];
    pose.knees = pose
        .hip_joints
        .map(|joint| joint - flat * Vec3::Y * pose.thigh_m);
    pose.ankles = pose.knees.map(|knee| knee - flat * Vec3::Y * pose.calf_m);
    pose
}

fn assert_bone_lengths(out: &LegIkOutput, pose: &LegIkInput, context: &str) {
    for s in 0..2 {
        let thigh = (out.knees[s] - out.hip_joints[s]).length();
        let calf = (out.ankles[s] - out.knees[s]).length();
        assert!(
            (thigh - pose.thigh_m).abs() < 1e-4,
            "{context}: thigh {thigh}"
        );
        assert!((calf - pose.calf_m).abs() < 1e-4, "{context}: calf {calf}");
    }
}

/// Interior knee angle (π for a straight leg).
fn knee_interior(hip: Vec3, knee: Vec3, ankle: Vec3) -> f32 {
    (hip - knee).angle_between(ankle - knee)
}

/// Hip-ankle distance of a leg whose interior knee angle is `interior`.
fn reach_at(thigh: f32, calf: f32, interior: f32) -> f32 {
    (thigh * thigh + calf * calf - 2.0 * thigh * calf * interior.cos()).sqrt()
}

/// Distance of the knee from the hip-ankle line when a straight leg is flexed
/// by `slack` (interior angle π − slack): twice the triangle's area over its
/// base, `thigh·calf·sin(slack) / reach`. For equal bones this is
/// `thigh·sin(slack/2)`, 7.3 mm for 0.42 m and the default 2°.
fn guard_knee_offset(thigh: f32, calf: f32, slack: f32) -> f32 {
    thigh * calf * slack.sin() / reach_at(thigh, calf, std::f32::consts::PI - slack)
}

#[test]
fn stationary_chair_ankles_keep_their_inferred_elevation_and_locks() {
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let input = chair(0.0, 0.02, 0.0);
    solver.solve(&input);
    for frame in 1..=200 {
        let pose = chair(frame as f64 * 0.01, 0.02, 0.0);
        let out = solver.solve(&pose);
        assert!(out.foot_locked[0] && out.foot_locked[1]);
        for s in 0..2 {
            assert!(
                (out.ankles[s].y - pose.ankles[s].y).abs() < 0.001,
                "{out:?}"
            );
            assert!((out.knees[s].y - out.hip_joints[s].y).abs() < 0.001);
        }
        // Nothing moved, so nothing may pull the seated pelvis either.
        assert!((out.hip - pose.hip).length() < 1e-5, "{out:?}");
    }
}

#[test]
fn seated_foot_lock_follows_slow_height_changes_while_keeping_horizontal_anchor() {
    let config = LegIkConfig {
        root_correction_enabled: false,
        ..LegIkConfig::default()
    };
    let mut solver = LegIkSolver::new(config);
    let first = chair(0.0, 0.02, 0.0);
    solver.solve(&first);
    // The FK skeleton rises 2 cm in 2 s (1 cm/s) with a constant 5 mm sideways
    // offset. Sideways is inside the 1 cm deadzone: the anchor does not move.
    // Its height is a first-order low-pass (τ = anchor_vertical_tau_secs) of the
    // FK height, so it trails a ramp of `rise` per frame by at most
    // `rise·(1 − a)/a` with `a = 1 − exp(−dt/τ)`: 2.95 mm here (≈ v·τ = 3 mm).
    let frames = 200;
    let rise = 0.02 / frames as f32;
    let a = 1.0 - (-(DT as f32) / config.anchor_vertical_tau_secs).exp();
    let steady_lag = rise * (1.0 - a) / a;
    assert!((steady_lag - 0.00295).abs() < 2e-5, "{steady_lag}");
    for frame in 1..=frames {
        let shift = Vec3::new(0.005, rise * frame as f32, 0.0);
        let pose = shifted(chair(frame as f64 * DT, 0.02, 0.0), shift);
        let out = solver.solve(&pose);
        assert!(out.ankles[0].is_finite() && out.knees[0].is_finite());
        for s in 0..2 {
            assert!(out.foot_locked[s]);
            assert!((out.ankles[s].x - first.ankles[s].x).abs() < 1e-5);
            assert!((out.ankles[s].z - first.ankles[s].z).abs() < 1e-5);
            let lag = pose.ankles[s].y - out.ankles[s].y;
            assert!(
                (-5e-5..=steady_lag + 5e-5).contains(&lag),
                "frame {frame}: the anchor height trails FK by {lag}"
            );
            if frame == frames {
                // After 6.7 time constants the lag is the steady one, not zero:
                // the height follows, it is not copied.
                assert!((lag - steady_lag).abs() < 1e-4, "{lag} vs {steady_lag}");
            }
            assert!(out.ankles[s].y > pose.ankle_to_sole_m + 0.019);
        }
        assert_bone_lengths(&out, &pose, "rising chair");
    }
}

#[test]
fn seated_horizontal_anchor_deadzone_widens_for_a_forward_lean_but_not_for_a_recline() {
    let config = LegIkConfig {
        root_correction_enabled: false,
        ..LegIkConfig::default()
    };
    let d = Anthropometrics::default();
    let lean = 40.0_f32.to_radians();
    // (torso tilt, sideways FK shift, whether the torso widens the limits).
    // The deadzone is anchor_deadzone_m + torso_slack_gain·bow_horizontal_m:
    // 1 cm upright; 1 cm + 0.3·0.58 m·sin 40° = 12.2 cm bowed 40° forward; and
    // still 1 cm reclined by 40° or lying supine, because a supported torso is
    // not a bowed one (a positive `torso_tilt` leans back).
    for (torso_tilt, shift, widened) in [
        (0.0_f32, 0.009_f32, false),
        (0.0, 0.03, false),
        (lean, 0.03, false),
        (std::f32::consts::FRAC_PI_2, 0.03, false),
        (-lean, 0.03, true),
    ] {
        let slack = if widened {
            config.torso_slack_gain
                * (d.neck_to_chest_m + d.chest_to_hip_m)
                * torso_tilt.abs().sin()
        } else {
            0.0
        };
        let deadzone = config.anchor_deadzone_m + slack;
        let mut solver = LegIkSolver::new(config);
        // Feet 15 cm up: clear of the floor attraction, which a forward bow
        // hands back to a seated leg (the squat rule).
        let first = chair(0.0, 0.15, torso_tilt);
        solver.solve(&first);
        for frame in 1..=200 {
            let pose = shifted(chair(frame as f64 * DT, 0.15, torso_tilt), Vec3::X * shift);
            let out = solver.solve(&pose);
            assert!(
                (out.lock_break_m - (config.lock_break_m + slack)).abs() < 1e-6,
                "tilt {torso_tilt}: lock radius {}",
                out.lock_break_m
            );
            assert!((out.root_max_m - (config.root_max_m + slack)).abs() < 1e-6);
            // The error beyond the deadzone decays with anchor_creep_tau_secs;
            // the part inside it never does. A sideways shift lengthens this
            // leg by under 1 mm, so the flexion guard does not interfere.
            let excess = (shift - deadzone).max(0.0);
            let keep = (-(frame as f32) * DT as f32 / config.anchor_creep_tau_secs).exp();
            let crept = excess * (1.0 - keep);
            for s in 0..2 {
                assert!(out.foot_locked[s], "tilt {torso_tilt} shift {shift}");
                let moved = out.ankles[s] - first.ankles[s];
                assert!(
                    (moved.x - crept).abs() < 2e-5,
                    "tilt {torso_tilt} shift {shift} frame {frame}: ankle moved {} m, model {crept}",
                    moved.x
                );
                assert!(moved.y.abs() < 2e-5 && moved.z.abs() < 2e-5, "{moved:?}");
            }
            assert_bone_lengths(&out, &pose, "sideways chair");
        }
        if widened {
            assert!(
                shift < deadzone,
                "the bowed case must sit inside its deadzone"
            );
        } else if shift > deadzone {
            // 2 s of a 4 s time constant: 2 cm·(1 − e^-0.5) = 7.9 mm.
            let crept = (shift - deadzone) * (1.0 - (-2.0_f32 / 4.0).exp());
            assert!((crept - 0.00787).abs() < 1e-5);
        }
    }
}

/// A supine wearer slowly abducts the locked, straight left leg by 15° over
/// 3 s and holds for 2 s (the ankle travels 0.84 m·2·sin 7.5° = 21.9 cm at
/// about 7 cm/s, too slow for a speed gate to see a step). The contact layer
/// is emulated: planted while slow, 80 ms unplanted after a release request.
/// Returns the published left ankle's distance from FK at the end and at its
/// worst, and whether the lock ever broke.
fn slow_abduction(torso_tilt: f32) -> (f32, f32, bool) {
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let flat = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let pose_at = |frame: usize| {
        let progress = (frame as f32 / 300.0).clamp(0.0, 1.0);
        let progress = progress * progress * (3.0 - 2.0 * progress);
        let mut pose = outstretched(frame as f64 * DT, 0.8, torso_tilt);
        // About the vertical through the hip joint: the left leg, which points
        // to -Z, swings out toward -X.
        let leg = Quat::from_rotation_y(15.0_f32.to_radians() * progress) * flat;
        pose.thigh_rotations[0] = leg;
        pose.calf_rotations[0] = leg;
        pose.knees[0] = pose.hip_joints[0] - leg * Vec3::Y * pose.thigh_m;
        pose.ankles[0] = pose.knees[0] - leg * Vec3::Y * pose.calf_m;
        pose
    };
    let travel = (pose_at(300).ankles[0] - pose_at(0).ankles[0]).length();
    assert!((travel - 0.219).abs() < 0.001, "{travel}");
    let mut unplanted_until = [0_usize; 2];
    let (mut last, mut worst, mut broke) = (0.0_f32, 0.0_f32, false);
    for frame in 0..=500 {
        let mut pose = pose_at(frame);
        pose.planted = [0, 1].map(|s| frame >= unplanted_until[s]);
        let out = solver.solve(&pose);
        assert_bone_lengths(&out, &pose, "abduction");
        for s in 0..2 {
            if out.release_requests[s] {
                broke = true;
                unplanted_until[s] = frame + 8;
            }
        }
        last = (out.ankles[0] - pose.ankles[0]).length();
        worst = worst.max(last);
        // The other leg did not move and the pelvis is not dragged far by it.
        assert!((out.ankles[1] - pose.ankles[1]).length() < 0.08);
    }
    (last, worst, broke)
}

#[test]
fn slowly_abducting_a_locked_leg_while_supine_does_not_hold_the_foot_to_a_stale_anchor() {
    // Supine: no torso slack, so the lock radius is the plain 7 cm and the creep
    // deadzone 1 cm. The moving foot may trail FK only by what those allow.
    let (last, worst, broke) = slow_abduction(std::f32::consts::FRAC_PI_2);
    eprintln!(
        "supine abduction: ankle ends {last:.4} m from FK, worst {worst:.4} m, broke {broke}"
    );
    assert!(broke, "a 22 cm slide must break a 7 cm lock");
    // The pelvis takes root_gain/2 = 30 % of one foot's error, so the lock
    // breaks when the anchor is 7 cm/0.7 = 10 cm from FK: the most the
    // published foot can trail. Where it ends depends on where the last
    // re-plant fell; here 4 cm, creeping on toward the 1 cm deadzone.
    assert!(worst < 0.07 / 0.7 + 0.005, "ankle trailed FK by {worst} m");
    assert!(last < 0.05, "ankle ends {last} m from FK");
    // What a bow's slack would do to the same legs (the limits a supine torso
    // got while every tilt counted): a torso tilted 90° forward widens the lock
    // radius to 7 + 0.3·58 = 24.4 cm and the deadzone to 18.4 cm, the lock
    // never breaks and the published foot stays near where it was planted.
    let (held, _, broke) = slow_abduction(-std::f32::consts::FRAC_PI_2);
    eprintln!("same legs under a 90° forward torso: ankle ends {held:.4} m from FK, broke {broke}");
    assert!(!broke);
    assert!(held > 0.10 && held > 2.0 * last, "{held}");
}

#[test]
fn existing_standing_floor_bias_fades_after_a_folded_leg_transition() {
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let mut pose = chair(0.0, 0.02, 0.0);
    // Isolate the anchor's floor policy; positions remain stationary throughout.
    pose.thigh_rotations = [Quat::IDENTITY; 2];
    for frame in 0..=200 {
        pose.at_secs = frame as f64 * 0.01;
        solver.solve(&pose);
    }
    pose.thigh_rotations = [Quat::from_rotation_x(std::f32::consts::FRAC_PI_2); 2];
    let mut out = solver.solve(&pose);
    for frame in 201..=400 {
        pose.at_secs = frame as f64 * 0.01;
        out = solver.solve(&pose);
    }
    for s in 0..2 {
        assert!(out.foot_locked[s]);
        assert!(
            (out.ankles[s].y - pose.ankles[s].y).abs() < 0.001,
            "{out:?}"
        );
    }
}

#[test]
fn tiny_root_motion_does_not_create_large_knee_bend_for_straight_reclined_legs() {
    let mut pose = outstretched(0.0, 0.8, std::f32::consts::FRAC_PI_2);
    let config = LegIkConfig {
        root_correction_enabled: false,
        ..LegIkConfig::default()
    };
    let mut solver = LegIkSolver::new(config);
    solver.solve(&pose);
    pose = shifted(pose, Vec3::Y * 0.001);
    pose.at_secs = 0.01;
    let out = solver.solve(&pose);
    for s in 0..2 {
        let error = (out.knees[s] - pose.knees[s]).length();
        assert!(
            error < 0.002,
            "1 mm root motion led to {error:.6} m knee change; FK {:?}, output {:?}",
            pose.knees[s],
            out.knees[s]
        );
    }
}

#[test]
fn seated_bent_leg_follows_a_held_ankle_only_as_far_as_the_flexion_guard_allows() {
    // Thigh 80° from vertical, shank vertical, torso upright: interior knee
    // angle 100°, no standing-floor prior, so the lock may change that angle by
    // at most knee_flex_slack_rad. For 0.42 m bones the hip-ankle reach is
    // 0.84·sin(interior/2): 643.5 mm at 100°, and with the default 1° slack
    // 638.7 mm at 99° and 648.2 mm at 101°. The hip moves along Z while the
    // ankle is held:
    //   +5 mm  -> reach +3.3 mm: inside, the ankle stays on its anchor;
    //   +20 mm -> reach +13.0 mm: 4.7 mm from bending, the ankle stops 8.3 mm short;
    //   -20 mm -> reach -12.7 mm: 4.7 mm from bending, the ankle passes by 8.0 mm.
    let bent = |at_secs: f64| {
        let mut pose = chair(at_secs, 0.15, 0.0);
        pose.thigh_rotations = [Quat::from_rotation_x(80_f32.to_radians()); 2];
        pose.knees = pose
            .hip_joints
            .map(|hip| hip - pose.thigh_rotations[0] * Vec3::Y * pose.thigh_m);
        pose.ankles = pose.knees.map(|knee| knee - Vec3::Y * pose.calf_m);
        pose
    };
    let default_slack = LegIkConfig::default().knee_flex_slack_rad;
    // The cases below need 3.3 mm to fit and 12.7 mm not to: 0.75°..2.5°.
    assert!((0.75_f32.to_radians()..2.5_f32.to_radians()).contains(&default_slack));
    for (shift, clamped) in [(0.005_f32, false), (0.02, true), (-0.02, true)] {
        let run = |slack: f32| {
            let mut solver = LegIkSolver::new(LegIkConfig {
                root_correction_enabled: false,
                knee_flex_slack_rad: slack,
                ..LegIkConfig::default()
            });
            let first = bent(0.0);
            let anchors = solver.solve(&first).ankles;
            let pose = shifted(bent(DT), Vec3::Z * shift);
            (anchors, pose, solver.solve(&pose))
        };
        let (anchors, pose, out) = run(default_slack);
        assert_bone_lengths(&out, &pose, "bent chair");
        for s in 0..2 {
            assert!(out.foot_locked[s]);
            let fk_interior = knee_interior(pose.hip_joints[s], pose.knees[s], pose.ankles[s]);
            assert!((fk_interior - 100_f32.to_radians()).abs() < 1e-4);
            let wanted = (anchors[s] - out.hip_joints[s]).length();
            let allowed = wanted.clamp(
                reach_at(pose.thigh_m, pose.calf_m, fk_interior - default_slack),
                reach_at(pose.thigh_m, pose.calf_m, fk_interior + default_slack),
            );
            let reach = (out.ankles[s] - out.hip_joints[s]).length();
            // 5e-5: the anchor creeps 25 µm on this frame (1 cm beyond the
            // deadzone, dt/τ = 0.0025) and the soft extension shifts a reach
            // this far from straight by about 2 µm.
            assert!(
                (reach - allowed).abs() < 5e-5,
                "shift {shift}: reach {reach}, guard allows {allowed} of {wanted}"
            );
            let bend =
                (knee_interior(out.hip_joints[s], out.knees[s], out.ankles[s]) - fk_interior).abs();
            assert!(
                bend < default_slack + 1e-3,
                "shift {shift}: the lock changed the knee angle by {bend} rad"
            );
            // The ankle is on the hip-anchor line: it slides, it does not swing.
            let line = (anchors[s] - out.hip_joints[s]).normalize();
            let off_line = (out.ankles[s] - out.hip_joints[s]).cross(line).length();
            assert!(off_line < 5e-5, "shift {shift}: {off_line} m off the line");
            let miss = (out.ankles[s] - anchors[s]).length();
            if clamped {
                assert!((miss - (wanted - allowed).abs()).abs() < 5e-5);
                assert!(miss > 0.003, "shift {shift}: slid only {miss} m");
                assert!(bend > default_slack - 1e-3, "guard not saturated: {bend}");
                // It still follows the hold for the rest of the 2 cm.
                assert!((out.ankles[s] - pose.ankles[s]).length() > 0.02 - miss - 1e-4);
            } else {
                assert!(miss < 5e-5, "shift {shift}: missed the anchor by {miss} m");
            }
        }
        // Without the guard the same hold is met exactly by bending the knee
        // against its measured angle (13 mm of reach at 0.27 m/rad ≈ 2.7°).
        let (anchors, pose, out) = run(std::f32::consts::PI);
        assert_bone_lengths(&out, &pose, "bent chair, guard off");
        for s in 0..2 {
            assert!((out.ankles[s] - anchors[s]).length() < 5e-5);
            let bend = (knee_interior(out.hip_joints[s], out.knees[s], out.ankles[s])
                - knee_interior(pose.hip_joints[s], pose.knees[s], pose.ankles[s]))
            .abs();
            if clamped {
                assert!(bend > 2.5_f32.to_radians(), "shift {shift}: {bend}");
                assert!(bend > default_slack + 0.01, "shift {shift}: {bend}");
            }
        }
    }
}

#[test]
fn locked_ankle_is_a_smooth_function_of_thigh_tilt_across_the_upright_taper() {
    // Shank vertical, torso upright, thigh tilt swept from 55° to 85° in 0.1°
    // steps. The pelvis is 1 cm closer to the locked ankle along the leg line
    // than when the foot was planted. A leg with the standing prior (thigh up
    // to 60°) bends to keep the ankle on the anchor; one without it (from 80°)
    // may flex by knee_flex_slack_rad only and lets the ankle pass the anchor by
    // the rest, `gap`. In between the reach is blended linearly by
    // g = smoothstep(60°, 80°, tilt), so the miss is g·gap. The steepest slope
    // of g is 1.5/20° = 0.0075 per 0.1° step.
    let defaults = LegIkConfig::default();
    let config = LegIkConfig {
        root_correction_enabled: false,
        // Keep the anchor height where it was planted, so that the residual is
        // the same 1 cm at every tilt (the height would otherwise follow FK by
        // 3 % per frame, itself weighted by the prior under test).
        anchor_vertical_tau_secs: 1.0e6,
        ..defaults
    };
    let slack = defaults.knee_flex_slack_rad;
    let residual = 0.01_f32;
    let miss_and_gap = |tenths: u32| {
        let tilt = (tenths as f32 * 0.1).to_radians();
        let pose_at = |at_secs: f64| {
            let mut pose = chair(at_secs, 0.45, 0.0);
            pose.thigh_rotations = [Quat::from_rotation_x(tilt); 2];
            pose.knees = pose
                .hip_joints
                .map(|hip| hip - pose.thigh_rotations[0] * Vec3::Y * pose.thigh_m);
            pose.ankles = pose.knees.map(|knee| knee - Vec3::Y * pose.calf_m);
            pose
        };
        let mut solver = LegIkSolver::new(config);
        let first = pose_at(0.0);
        let anchors = solver.solve(&first).ankles;
        let along = (first.ankles[0] - first.hip_joints[0]).normalize();
        let pose = shifted(pose_at(DT), along * residual);
        let out = solver.solve(&pose);
        assert_bone_lengths(&out, &pose, "taper sweep");
        let mut result = (0.0_f32, 0.0_f32);
        for s in 0..2 {
            assert!(out.foot_locked[s], "tilt {tenths}/10°");
            let fk_interior = knee_interior(pose.hip_joints[s], pose.knees[s], pose.ankles[s]);
            let free = (anchors[s] - out.hip_joints[s]).length();
            let guarded = free.max(reach_at(pose.thigh_m, pose.calf_m, fk_interior - slack));
            let reach = (out.ankles[s] - out.hip_joints[s]).length();
            // The ankle is on the hip-anchor line; `passed` is how far beyond
            // the anchor (negative: short of it).
            let line = (anchors[s] - out.hip_joints[s]).normalize();
            let off_line = (out.ankles[s] - out.hip_joints[s]).cross(line).length();
            assert!(
                off_line < 3e-5,
                "tilt {tenths}/10°: {off_line} m off the line"
            );
            result = (reach - free, guarded - free);
        }
        result
    };
    let (full, zero) = (
        defaults.support_thigh_full_tilt_rad.to_degrees(),
        defaults.support_thigh_zero_tilt_rad.to_degrees(),
    );
    let weight_lost = |degrees: f32| {
        let t = ((degrees - full) / (zero - full)).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    };
    let sweep: Vec<(f32, f32, f32)> = (550..=850)
        .map(|tenths| {
            let (miss, gap) = miss_and_gap(tenths);
            (tenths as f32 * 0.1, miss, gap)
        })
        .collect();
    let largest_gap = sweep.iter().map(|s| s.2).fold(0.0_f32, f32::max);
    // 1 cm asked, 1° allowed (0.42·sin(tilt/2) m/rad: 3.4 to 5.0 mm).
    assert!((0.004..0.008).contains(&largest_gap), "{largest_gap}");
    // 1.05: the gap itself shrinks by a few µm per step; 2e-5: f32 positions.
    let step_bound = largest_gap * 1.5 * (0.1 / (zero - full)) * 1.05 + 2e-5;
    let mut largest_step = 0.0_f32;
    for (k, &(degrees, miss, gap)) in sweep.iter().enumerate() {
        // 6e-5: the soft extension shortens the free reach of a leg this bent
        // by up to 30 µm more than asked (d − d³/(d² + s²) ≈ s²/d differs by
        // that much between 9.5 and 10.5 cm from full extension).
        assert!(
            (miss - weight_lost(degrees) * gap).abs() < 6e-5,
            "thigh {degrees}°: ankle {miss} m past the anchor, blend gives {}",
            weight_lost(degrees) * gap
        );
        if degrees <= full {
            assert!(miss.abs() < 6e-5, "thigh {degrees}°: {miss}");
        }
        if degrees >= zero {
            assert!(
                (miss - gap).abs() < 6e-5,
                "thigh {degrees}°: {miss} of {gap}"
            );
        }
        if k > 0 {
            let step = miss - sweep[k - 1].1;
            largest_step = largest_step.max(step.abs());
            assert!(
                step > -2e-5,
                "thigh {degrees}°: the ankle moved back by {step}"
            );
            assert!(
                step.abs() < step_bound,
                "thigh {degrees}°: a 0.1° step moved the ankle {step} m (bound {step_bound})"
            );
        }
    }
    eprintln!(
        "taper sweep: gap up to {largest_gap:.5} m, largest 0.1° ankle step {largest_step:.6} m (bound {step_bound:.6})"
    );
    // No switch: a guard applied by widening the slack angle with the weight
    // would keep the ankle on the anchor until about 79° (where 1° plus 1 % of
    // 179° stops covering the 1 cm) and then let go within a degree. The blend
    // is already most of the way at 75° and moves little from 78° to 80°.
    let at = |degrees: f32| sweep[((degrees - 55.0) * 10.0).round() as usize];
    assert!(at(75.0).1 > 0.8 * at(75.0).2, "{:?}", at(75.0));
    assert!(at(70.0).1 > 0.45 * at(70.0).2 && at(70.0).1 < 0.55 * at(70.0).2);
    assert!(at(80.0).1 - at(78.0).1 < 0.05 * largest_gap);
    assert!(largest_step < 0.01 * largest_gap);
}

#[test]
fn an_absurd_reach_margin_cannot_straighten_a_bent_locked_leg() {
    // The margin keeps the reach away from the fully folded limit. Uncapped, a
    // margin above thigh + calf would push the minimum reach to full extension
    // and straighten every locked leg; inside two_bone it is limited to a
    // quarter of the reach range.
    for margin in [0.5_f32, 5.0, f32::MAX] {
        let mut solver = LegIkSolver::new(LegIkConfig {
            root_correction_enabled: false,
            reach_margin_m: margin,
            ..LegIkConfig::default()
        });
        let first = chair(0.0, 0.15, 0.0);
        let anchors = solver.solve(&first).ankles;
        // 5 mm sideways (inside the deadzone) so that the lock has to solve.
        let pose = shifted(chair(DT, 0.15, 0.0), Vec3::X * 0.005);
        let out = solver.solve(&pose);
        assert_bone_lengths(&out, &pose, "absurd margin");
        for s in 0..2 {
            assert!(out.foot_locked[s], "margin {margin}");
            let miss = (out.ankles[s] - anchors[s]).length();
            assert!(
                miss < 5e-5,
                "margin {margin}: ankle {miss} m off its anchor"
            );
            let interior = knee_interior(out.hip_joints[s], out.knees[s], out.ankles[s]);
            assert!(
                (interior - std::f32::consts::FRAC_PI_2).abs() < 1e-3,
                "margin {margin}: knee angle {interior}"
            );
        }
    }
}

#[test]
fn collapsed_targets_and_extreme_margins_stay_finite() {
    for margin in [0.0, f32::MAX] {
        let config = LegIkConfig {
            root_correction_enabled: false,
            lock_break_m: 10.0,
            reach_margin_m: margin,
            ..LegIkConfig::default()
        };
        let mut solver = LegIkSolver::new(config);
        let mut pose = chair(0.0, 0.15, 0.0);
        solver.solve(&pose);
        let delta = pose.ankles[0] - pose.hip_joints[0];
        pose = shifted(pose, delta);
        pose.at_secs = 0.01;
        let out = solver.solve(&pose);
        for s in 0..2 {
            assert!(
                out.knees[s].is_finite() && out.ankles[s].is_finite(),
                "margin {margin}"
            );
        }
        assert_bone_lengths(&out, &pose, "collapsed target");
    }
}

#[test]
fn small_transverse_residuals_do_not_create_a_discontinuous_straight_leg_bend() {
    for calf in [0.42_f32, 0.36] {
        for step in 0..=10 {
            let mut pose = chair(0.0, 0.2, 0.0);
            let horizontal = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
            pose.hip.y = 0.8;
            pose.hip_joints =
                [-1.0_f32, 1.0].map(|side| pose.hip + Vec3::X * side * pose.hip_width_m * 0.5);
            pose.thigh_rotations = [horizontal; 2];
            pose.calf_rotations = [horizontal; 2];
            pose.calf_m = calf;
            pose.knees = pose
                .hip_joints
                .map(|hip| hip - horizontal * Vec3::Y * pose.thigh_m);
            pose.ankles = pose.knees.map(|knee| knee - horizontal * Vec3::Y * calf);
            let mut solver = LegIkSolver::new(LegIkConfig {
                root_correction_enabled: false,
                ..LegIkConfig::default()
            });
            solver.solve(&pose);
            pose = shifted(pose, Vec3::Y * (step as f32 * 0.0001));
            pose.at_secs = 0.01;
            let out = solver.solve(&pose);
            for s in 0..2 {
                assert!(
                    (out.knees[s] - pose.knees[s]).length() < 0.003,
                    "calf {calf} step {step}: {:?}",
                    out.knees[s] - pose.knees[s]
                );
            }
            assert_bone_lengths(&out, &pose, "transverse residual");
        }
    }
}

/// Largest knee rise above its hip joint over 30 frames after the whole FK
/// skeleton steps `push` meters toward feet that were locked with both legs
/// straight and level, with the given knee flexion slack.
fn flat_leg_push(hip_height: f32, torso_tilt: f32, root: bool, slack: f32, push: f32) -> f32 {
    let mut solver = LegIkSolver::new(LegIkConfig {
        root_correction_enabled: root,
        knee_flex_slack_rad: slack,
        ..LegIkConfig::default()
    });
    let first = outstretched(0.0, hip_height, torso_tilt);
    solver.solve(&first);
    let mut worst = 0.0_f32;
    for frame in 1..=30 {
        // The feet lie toward -Z, so -Z moves the pelvis toward them.
        let pose = shifted(
            outstretched(frame as f64 * DT, hip_height, torso_tilt),
            Vec3::NEG_Z * push,
        );
        let out = solver.solve(&pose);
        let context = format!("height {hip_height} root {root} slack {slack} push {push}");
        assert_bone_lengths(&out, &pose, &context);
        for s in 0..2 {
            assert!(out.foot_locked[s], "{context}");
            let rise = out.knees[s].y - out.hip_joints[s].y;
            // 3e-4: f32 rounding of `thigh² − along²` near full extension.
            assert!(rise > -3e-4, "{context}: knee sank {rise}");
            worst = worst.max(rise);
            // Whatever the knee does, the ankle only slides along the leg.
            assert!(
                (out.ankles[s].y - first.ankles[s].y).abs() < 1e-4
                    && (out.ankles[s].x - first.ankles[s].x).abs() < 1e-4,
                "{context}: ankle left the leg line: {:?}",
                out.ankles[s] - first.ankles[s]
            );
        }
        assert!((out.hip.y - pose.hip.y).abs() < 1e-5, "{context}");
    }
    worst
}

/// Reach a straight leg gives up when asked to shorten by `d`: the solver's
/// soft full extension, a cubic deadband `d³/(d² + s²)` with s = 5 mm.
fn soft_extension(d: f32) -> f32 {
    d * d * d / (d * d + 0.005 * 0.005)
}

#[test]
fn pelvis_moving_toward_locked_flat_feet_raises_the_knees_by_at_most_the_flexion_guard() {
    let d = Anthropometrics::default();
    let slack = LegIkConfig::default().knee_flex_slack_rad;
    // A straight leg may flex by `slack`, which puts the knee
    // thigh·calf·sin(slack)/reach off the leg line: thigh·sin(slack/2) for
    // equal bones, 3.7 mm for 0.42 m and the default 1°.
    let bound = guard_knee_offset(d.thigh_length_m, d.calf_length_m, slack);
    assert!((bound - d.thigh_length_m * (0.5 * slack).sin()).abs() < 1e-6);
    assert!(bound < 0.005, "{bound}");
    for (label, hip_height, torso_tilt) in [
        ("supine", 0.8, std::f32::consts::FRAC_PI_2),
        ("long sitting", d.ankle_to_sole_m, 0.0),
    ] {
        for root in [false, true] {
            for push in [0.001_f32, 0.005, 0.020] {
                let guarded = flat_leg_push(hip_height, torso_tilt, root, slack, push);
                // 3e-4: for a straight leg the solver recovers the interior
                // angle as acos(−1 + ε), which turns a few f32 ulps of ε into
                // up to ~1e-3 rad of extra flexion.
                assert!(
                    guarded < bound + 3e-4,
                    "{label}, root {root}: a {push} m push raised the knees {guarded} m (guard {bound})"
                );
                let unguarded =
                    flat_leg_push(hip_height, torso_tilt, root, std::f32::consts::PI, push);
                eprintln!(
                    "{label}, root {root}, push {push}: knees rise {guarded:.5} m, {unguarded:.5} m without the guard"
                );
                // Without the guard the knee offset of a leg asked to shorten
                // by d is sqrt(thigh·soft(d)): 4.0 mm for 1 mm, 3.2 cm for 5 mm
                // and 8.9 cm for 2 cm (root correction takes about 10 % of the
                // push on the first frame).
                let free = (d.thigh_length_m * soft_extension(0.85 * push)).sqrt();
                assert!(
                    unguarded > free,
                    "{label}, root {root}: push {push} without the guard rose only {unguarded}"
                );
                if push >= 0.005 {
                    assert!(unguarded > 0.02, "{label}, root {root}: {unguarded}");
                    // And the guard is what stops it: it is saturated.
                    assert!(guarded > bound - 3e-4, "{label}: {guarded}");
                }
            }
        }
    }
}

#[test]
fn sub_millimetre_along_leg_noise_on_a_locked_flat_leg_barely_moves_the_knee() {
    let d = Anthropometrics::default();
    let noise = 0.00025_f32;
    // A straight leg asked to shorten by δ shortens by e = δ³/(δ² + s²)
    // (s = 5 mm), and a leg shortened by e has its knee sqrt(L·e) off the line
    // with L = 2·thigh·calf/(thigh + calf). The knee offset is therefore
    // sqrt(L·δ³/(δ² + s²)) ≈ δ^1.5·sqrt(L)/s: 0.51 mm for ±0.25 mm of noise on
    // 0.42 m bones, where the bare square root would give sqrt(0.42 m·0.25 mm)
    // = 10 mm. Along the leg the knee rides with the pelvis, at most 2δ from
    // one frame to the next; 1e-4 covers f32 rounding at full extension.
    let leg = 2.0 * d.thigh_length_m * d.calf_length_m / (d.thigh_length_m + d.calf_length_m);
    let offset = (leg * soft_extension(noise)).sqrt();
    assert!((0.00045..0.00056).contains(&offset), "{offset}");
    let bound = offset.hypot(2.0 * noise) + 1e-4;
    for root in [false, true] {
        let mut solver = LegIkSolver::new(LegIkConfig {
            root_correction_enabled: root,
            ..LegIkConfig::default()
        });
        let first = outstretched(0.0, 0.8, std::f32::consts::FRAC_PI_2);
        let mut previous = solver.solve(&first).knees;
        let (mut largest, mut largest_rise) = (0.0_f32, 0.0_f32);
        for frame in 1..=200 {
            let sign = if frame % 2 == 0 { 1.0 } else { -1.0 };
            let pose = shifted(
                outstretched(frame as f64 * DT, 0.8, std::f32::consts::FRAC_PI_2),
                Vec3::Z * (sign * noise),
            );
            let out = solver.solve(&pose);
            assert_bone_lengths(&out, &pose, "flat jitter");
            for s in 0..2 {
                assert!(out.foot_locked[s]);
                let step = (out.knees[s] - previous[s]).length();
                largest = largest.max(step);
                assert!(
                    step < bound,
                    "root {root} frame {frame}: 0.25 mm of noise moved the knee {step} m"
                );
                // The part that shows: the knee leaving the leg line.
                let rise = (out.knees[s].y - out.hip_joints[s].y).abs();
                largest_rise = largest_rise.max(rise);
                assert!(rise < offset + 1e-4, "root {root} frame {frame}: {rise}");
            }
            previous = out.knees;
        }
        eprintln!(
            "flat leg, root {root}: largest knee step {largest:.6} m (bound {bound:.6}), largest rise {largest_rise:.6} m (model {offset:.6})"
        );
    }
}

/// Crawling on hands and knees: torso level and facing down, thighs vertical
/// (hip joints 0.42 m above the knees), shanks flat on the floor behind the
/// knees with the ankles at their floor height.
fn crawling(at_secs: f64) -> LegIkInput {
    let mut pose = chair(at_secs, 0.0, -std::f32::consts::FRAC_PI_2);
    let shank = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    pose.thigh_rotations = [Quat::IDENTITY; 2];
    pose.calf_rotations = [shank; 2];
    pose.knees = pose.hip_joints.map(|joint| joint - Vec3::Y * pose.thigh_m);
    pose.ankles = pose.knees.map(|knee| knee - shank * Vec3::Y * pose.calf_m);
    pose
}

#[test]
fn the_flexion_guard_never_slides_a_crawling_ankle_through_the_floor() {
    let first = crawling(0.0);
    let defaults = LegIkConfig::default();
    let floor = first.ankle_to_sole_m - defaults.floor_margin_m;
    for s in 0..2 {
        assert!((first.ankles[s].y - first.ankle_to_sole_m).abs() < 1e-6);
        assert!((first.ankles[s].z - first.hip_joints[s].z - first.calf_m).abs() < 1e-6);
    }
    // The hip-ankle line points 45° down and back; a horizontal shank has no
    // standing prior, so the knee angle is guarded. The pelvis then moves `back`
    // toward the point above the locked ankles: the anchor is closer than the
    // measured leg is long, the guard lets the reach shrink by 1° of knee angle
    // only (0.42 m·cos 45°·1° = 5 mm) and carries the ankle past the anchor
    // along a line that keeps going down. For 6 cm back the line is
    // (−0.42, +0.36)/0.553 and a 0.589 m reach ends 0.42·0.589/0.553 = 44.7 cm
    // below the hip joint: 2.7 cm below the anchor, 1.7 cm below the 7 cm that
    // the floor clamp allows.
    let run = |floor_clamp_enabled: bool, back: f32| {
        let mut solver = LegIkSolver::new(LegIkConfig {
            root_correction_enabled: false,
            floor_clamp_enabled,
            ..defaults
        });
        solver.solve(&crawling(0.0));
        let mut lowest = f32::INFINITY;
        for frame in 1..=30 {
            let pose = shifted(crawling(frame as f64 * DT), Vec3::Z * back);
            let out = solver.solve(&pose);
            assert_bone_lengths(&out, &pose, "crawling");
            for s in 0..2 {
                assert!(out.foot_locked[s], "back {back}");
                assert!(out.knees[s].is_finite() && out.ankles[s].is_finite());
                lowest = lowest.min(out.ankles[s].y);
            }
        }
        lowest
    };
    for back in [0.01_f32, 0.02, 0.04, 0.06] {
        let clamped = run(true, back);
        assert!(
            clamped > floor - 1e-5,
            "{back} m back: ankle at {clamped} m, floor limit {floor}"
        );
        let unclamped = run(false, back);
        eprintln!(
            "crawling, pelvis {back} m back: lowest ankle {clamped:.4} m, {unclamped:.4} m without the floor clamp"
        );
        assert!(unclamped <= clamped + 1e-6);
        if back >= 0.06 {
            assert!(
                unclamped < floor - 0.015,
                "{back} m back without the clamp: ankle only down to {unclamped} m"
            );
            // The clamp is what holds it: the ankle rests on the limit.
            assert!(clamped < floor + 1e-4, "{clamped}");
        }
    }
}
