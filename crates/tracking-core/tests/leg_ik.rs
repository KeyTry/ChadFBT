//! Synthetic regressions for contact-consistent leg IK. They check that the
//! implementation follows the model in docs/research/stance-anchored-estimator.md
//! section 4; they are not accuracy results on a worn body.

use glam::{Quat, Vec3};
use tracking_core::leg_ik::{LegIkConfig, LegIkInput, LegIkOutput, LegIkSolver, LegIkStatus};

/// Robust rotation angle between two quaternions (no acos domain issues).
trait AngleTo {
    fn angle_to(self, other: Quat) -> f32;
}

impl AngleTo for Quat {
    fn angle_to(self, other: Quat) -> f32 {
        let d = (self * other.inverse()).normalize();
        2.0 * glam::Vec3::new(d.x, d.y, d.z).length().atan2(d.w.abs())
    }
}
const NECK_TO_CHEST: f32 = 0.25;
const CHEST_TO_HIP: f32 = 0.30;
const HIP_WIDTH: f32 = 0.26;
const THIGH: f32 = 0.45;
const CALF: f32 = 0.43;
const ANKLE_TO_SOLE: f32 = 0.08;
const DT: f64 = 0.01;
/// Largest knee offset from the hip-ankle line that still counts as a straight
/// leg. Not zero: the offset is `sqrt(thigh² − along²)`, so f32 rounding of a
/// reach near full extension can show up as tens of micrometres.
const STRAIGHT_KNEE_TOLERANCE_M: f32 = 1.0e-4;

#[derive(Clone, Copy)]
struct Pose {
    hip: Vec3,
    hip_rotation: Quat,
    thighs: [Quat; 2],
    calves: [Quat; 2],
}

impl Pose {
    /// Slightly bent standing pose; thigh `Rx(+a)` puts the knee forward (-Z),
    /// calf `Rx(-a)` puts the ankle back under it.
    fn standing(bend: f32) -> Self {
        Self {
            // Feet on the floor: planted anchors near the floor rest on it.
            hip: Vec3::new(0.1, ANKLE_TO_SOLE + (THIGH + CALF) * bend.cos(), -0.3),
            hip_rotation: Quat::IDENTITY,
            thighs: [Quat::from_rotation_x(bend); 2],
            calves: [Quat::from_rotation_x(-bend); 2],
        }
    }

    fn input(&self, at_secs: f64, planted: [bool; 2]) -> LegIkInput {
        let chest = self.hip + self.hip_rotation * Vec3::Y * CHEST_TO_HIP;
        let neck = chest + self.hip_rotation * Vec3::Y * NECK_TO_CHEST;
        let side = [-1.0_f32, 1.0];
        let hip_joints =
            [0, 1].map(|s| self.hip + self.hip_rotation * Vec3::X * (side[s] * HIP_WIDTH * 0.5));
        let knees = [0, 1].map(|s| hip_joints[s] - self.thighs[s] * Vec3::Y * THIGH);
        let ankles = [0, 1].map(|s| knees[s] - self.calves[s] * Vec3::Y * CALF);
        LegIkInput {
            at_secs,
            neck,
            chest,
            hip: self.hip,
            hip_joints,
            knees,
            ankles,
            chest_rotation: self.hip_rotation,
            spine_rotation: self.hip_rotation,
            hip_rotation: self.hip_rotation,
            thigh_rotations: self.thighs,
            calf_rotations: self.calves,
            measured_foot_rotations: [None; 2],
            planted,
            neck_to_chest_m: NECK_TO_CHEST,
            chest_to_hip_m: CHEST_TO_HIP,
            hip_width_m: HIP_WIDTH,
            thigh_m: THIGH,
            calf_m: CALF,
            ankle_to_sole_m: ANKLE_TO_SOLE,
            enabled: true,
        }
    }
}

/// Rigidly translate the whole FK skeleton (simulated upstream model error).
fn shifted(mut input: LegIkInput, delta: Vec3) -> LegIkInput {
    input.neck += delta;
    input.chest += delta;
    input.hip += delta;
    for s in 0..2 {
        input.hip_joints[s] += delta;
        input.knees[s] += delta;
        input.ankles[s] += delta;
    }
    input
}

fn assert_bone_lengths(out: &LegIkOutput, s: usize) {
    let thigh = (out.knees[s] - out.hip_joints[s]).length();
    let calf = (out.ankles[s] - out.knees[s]).length();
    assert!((thigh - THIGH).abs() < 2.0e-5, "thigh {thigh}");
    assert!((calf - CALF).abs() < 2.0e-5, "calf {calf}");
}

fn assert_finite(out: &LegIkOutput) {
    for p in [out.chest, out.hip]
        .into_iter()
        .chain(out.hip_joints)
        .chain(out.knees)
        .chain(out.ankles)
    {
        assert!(p.is_finite());
    }
    for q in out.foot_rotations {
        assert!(q.is_finite() && (q.length() - 1.0).abs() < 1.0e-4);
    }
    assert!(out.root_correction.iter().all(|v| v.is_finite()));
    assert!(out.lock_error_m.iter().all(|v| v.is_finite()));
}

fn perpendicular(v: Vec3, along: Vec3) -> Vec3 {
    let unit = along.normalize();
    v - unit * v.dot(unit)
}

fn bits(out: &LegIkOutput) -> Vec<u32> {
    [out.chest, out.hip]
        .into_iter()
        .chain(out.hip_joints)
        .chain(out.knees)
        .chain(out.ankles)
        .flat_map(|p| p.to_array().map(f32::to_bits))
        .collect()
}

fn input_bits(input: &LegIkInput) -> Vec<u32> {
    [input.chest, input.hip]
        .into_iter()
        .chain(input.hip_joints)
        .chain(input.knees)
        .chain(input.ankles)
        .flat_map(|p| p.to_array().map(f32::to_bits))
        .collect()
}

#[test]
fn planted_feet_stay_put_while_fk_wobbles_two_centimeters() {
    let pose = Pose::standing(0.15);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let first = solver.solve(&pose.input(0.0, [true; 2]));
    let anchors = first.ankles;
    let mut max_slip = 0.0_f32;
    let mut max_root = 0.0_f32;
    for k in 1..=300 {
        let t = k as f64 * DT;
        let wobble = Vec3::new(
            0.02 * (std::f32::consts::TAU * 2.0 * t as f32).sin(),
            0.0,
            0.0,
        );
        let input = shifted(pose.input(t, [true; 2]), wobble);
        let out = solver.solve(&input);
        assert_eq!(out.status, LegIkStatus::Active);
        for s in 0..2 {
            assert!(out.foot_locked[s]);
            assert!(!out.release_requests[s]);
            max_slip = max_slip.max((out.ankles[s] - anchors[s]).length());
            assert_bone_lengths(&out, s);
        }
        let root = Vec3::from_array(out.root_correction);
        max_root = max_root.max(root.length());
        assert!((out.hip - (input.hip + root)).length() < 1.0e-6);
        // Neck does not move; the chest takes the proportional share.
        let chest_share = NECK_TO_CHEST / (NECK_TO_CHEST + CHEST_TO_HIP);
        assert!((out.chest - (input.chest + root * chest_share)).length() < 1.0e-6);
    }
    assert!(
        max_slip < 1.0e-3,
        "published planted ankle moved {max_slip} m"
    );
    assert!(
        max_root > 0.005,
        "pelvis took part of the residual: {max_root}"
    );
}

#[test]
fn pelvis_takes_root_gain_fraction_of_a_step_error() {
    let pose = Pose::standing(0.15);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    solver.solve(&pose.input(0.0, [true; 2]));
    let step = Vec3::new(0.02, 0.0, -0.01);
    let mut out = None;
    let mut input = pose.input(0.0, [true; 2]);
    for k in 1..=60 {
        input = shifted(pose.input(k as f64 * DT, [true; 2]), step);
        out = Some(solver.solve(&input));
    }
    let out = out.unwrap();
    // Locked ankle equals the (slowly creeping) anchor.
    let anchor_error = (out.ankles[0] - input.ankles[0] + out.ankles[1] - input.ankles[1]) * 0.5;
    let root = Vec3::from_array(out.root_correction);
    let expected = anchor_error * 0.6;
    assert!(
        (root - expected).length() < 3.0e-4,
        "root {root:?} expected {expected:?}"
    );
    assert!((out.hip - input.hip - root).length() < 1.0e-6);
    // 5 cm clamp on the target: ramp the error to 11 cm (slowly enough that the
    // remaining error after root correction never reaches the 7 cm break).
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    solver.solve(&pose.input(0.0, [true; 2]));
    let mut last = None;
    for k in 1..=80 {
        let ramp = (k as f32 / 50.0).min(1.0) * 0.11;
        let input = shifted(
            pose.input(k as f64 * DT, [true; 2]),
            Vec3::new(ramp, 0.0, 0.0),
        );
        let out = solver.solve(&input);
        assert!(out.foot_locked[0] && !out.release_requests[0]);
        last = Some(out);
    }
    let root = Vec3::from_array(last.unwrap().root_correction);
    assert!(
        (root.length() - 0.05).abs() < 1.0e-4,
        "clamped root {root:?}"
    );
}

#[test]
fn sliding_foot_breaks_lock_requests_release_and_returns_to_fk_smoothly() {
    let pose = Pose::standing(0.2);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let mut planted = [true, false];
    let mut previous = solver.solve(&pose.input(0.0, planted)).ankles[0];
    let mut released_at = None;
    let mut max_step = 0.0_f32;
    let mut last = None;
    for k in 1..=250 {
        let t = k as f64 * DT;
        // Upstream error: the left FK leg swings sideways, moving the FK ankle
        // at ~0.5 m/s between t = 0.1 s and t = 0.6 s, while the foot is planted.
        let swing = ((t.clamp(0.1, 0.6) - 0.1) as f32) * 0.5 / 0.87;
        let mut swung = pose;
        swung.thighs[0] = Quat::from_rotation_z(swing) * pose.thighs[0];
        swung.calves[0] = Quat::from_rotation_z(swing) * pose.calves[0];
        let input = swung.input(t, planted);
        let out = solver.solve(&input);
        assert_finite(&out);
        assert_bone_lengths(&out, 0);
        let step = (out.ankles[0] - previous).length();
        max_step = max_step.max(step);
        previous = out.ankles[0];
        if out.release_requests[0] && released_at.is_none() {
            released_at = Some(t);
            assert!(out.lock_error_m[0] > 0.07);
            assert!(!out.foot_locked[0]);
            // No jump on the release frame itself (the soft-extension reach is
            // a difference of two f32 terms, good to a few tens of micrometres).
            assert!(step < 1.0e-4, "release frame step {step}");
            // The contact layer honours the request.
            planted[0] = false;
        }
        last = Some((input, out));
    }
    assert!(released_at.is_some(), "lock never broke");
    eprintln!("lock released at {released_at:?} s; largest ankle step {max_step} m/frame");
    // Bound: 0.5 m/s FK slide + 5 cm root decay (tau 60 ms) + 7 cm offset decay
    // (tau 100 ms) can add up to ~19 mm in one 10 ms frame.
    assert!(max_step < 0.020, "largest per-frame ankle step {max_step}");
    let (input, out) = last.unwrap();
    assert_eq!(out.ankles, input.ankles);
    assert_eq!(out.root_correction, [0.0; 3]);
}

#[test]
fn knee_recomputation_keeps_bone_lengths_for_varied_poses() {
    let mut seed = 0x1234_5678_u32;
    let mut rand = move || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (seed >> 8) as f32 / (1 << 24) as f32 * 2.0 - 1.0
    };
    for _ in 0..40 {
        let flex = 0.2 + 0.8 * rand().abs();
        let base = Pose {
            hip: Vec3::new(rand(), 0.95 + 0.05 * rand(), rand()),
            hip_rotation: Quat::from_rotation_y(rand() * 3.0),
            thighs: [0, 1].map(|_| Quat::from_rotation_y(rand()) * Quat::from_rotation_x(flex)),
            calves: [0, 1].map(|_| Quat::from_rotation_x(-flex * 0.8)),
        };
        let mut solver = LegIkSolver::new(LegIkConfig {
            floor_clamp_enabled: false,
            ..Default::default()
        });
        solver.solve(&base.input(0.0, [true; 2]));
        for k in 1..=20 {
            let mut pose = base;
            let wobble = 0.05 * k as f32 / 20.0;
            pose.thighs[0] = Quat::from_rotation_z(wobble * rand()) * pose.thighs[0];
            pose.calves[1] = Quat::from_rotation_x(wobble * rand()) * pose.calves[1];
            let input = pose.input(k as f64 * DT, [true; 2]);
            let out = solver.solve(&input);
            assert_finite(&out);
            for s in 0..2 {
                assert_bone_lengths(&out, s);
                // Hip joints only move with the pelvis.
                let root = Vec3::from_array(out.root_correction);
                assert!((out.hip_joints[s] - input.hip_joints[s] - root).length() < 1.0e-6);
            }
        }
    }
}

#[test]
fn knee_stays_on_measured_bend_side() {
    for bend in [0.5_f32, 0.15] {
        for i in 0..8 {
            let angle = i as f32 * std::f32::consts::TAU / 8.0;
            let pose = Pose::standing(bend);
            let mut solver = LegIkSolver::new(LegIkConfig::default());
            let start = pose.input(0.0, [true; 2]);
            solver.solve(&start);
            let delta = Vec3::new(angle.cos(), 0.3 * angle.sin(), angle.sin()) * 0.05;
            let mut input = start;
            let mut out = None;
            for k in 1..=10 {
                input = shifted(pose.input(k as f64 * DT, [true; 2]), delta);
                out = Some(solver.solve(&input));
            }
            let out = out.unwrap();
            for s in 0..2 {
                assert!(out.foot_locked[s]);
                let fk_bend = perpendicular(
                    input.knees[s] - input.hip_joints[s],
                    input.ankles[s] - input.hip_joints[s],
                );
                let ik_bend = perpendicular(
                    out.knees[s] - out.hip_joints[s],
                    out.ankles[s] - out.hip_joints[s],
                );
                assert!(
                    ik_bend.normalize().dot(fk_bend.normalize()) > 0.9,
                    "bend {bend} dir {i}: {ik_bend:?} vs {fk_bend:?}"
                );
            }
        }
    }
}

#[test]
fn straight_fk_leg_bends_forward_never_backward() {
    let pose = Pose::standing(0.0);
    // (rigid FK shift, whether the locked legs must bend). A shift that moves
    // the hip joint away from or across the anchor only stretches a leg that is
    // already straight; one with a downward part shortens the hip-anchor
    // distance (3 cm down against at most 0.04²/(2·0.88) = 0.9 mm of sideways
    // lengthening) and has to bend the knee, whatever its horizontal direction.
    for (delta, bends) in [
        (Vec3::new(0.0, 0.0, -0.04), false),
        (Vec3::new(0.0, 0.0, 0.04), false),
        (Vec3::new(0.04, 0.0, 0.0), false),
        (Vec3::new(-0.04, 0.0, 0.0), false),
        (Vec3::new(0.0, 0.03, 0.0), false),
        (Vec3::new(0.0, -0.03, 0.0), true),
        (Vec3::new(0.0, -0.03, 0.04), true),
        (Vec3::new(0.0, -0.03, -0.04), true),
        (Vec3::new(0.04, -0.03, 0.0), true),
        (Vec3::new(-0.04, -0.03, 0.0), true),
    ] {
        let mut solver = LegIkSolver::new(LegIkConfig::default());
        solver.solve(&pose.input(0.0, [true; 2]));
        for k in 1..=10 {
            let out = solver.solve(&shifted(pose.input(k as f64 * DT, [true; 2]), delta));
            for s in 0..2 {
                assert!(out.foot_locked[s], "{delta:?} frame {k}");
                assert_bone_lengths(&out, s);
                let bend = perpendicular(
                    out.knees[s] - out.hip_joints[s],
                    out.ankles[s] - out.hip_joints[s],
                );
                if bends {
                    // After the first frame the pelvis has taken at most 15 %
                    // of its 60 % share and the anchor has sunk 3 % of the way:
                    // over 2 cm of shortening is left, a knee offset of
                    // sqrt(0.44·0.02) ≈ 9 cm; after 10 frames about 1 cm is
                    // left (≈ 6 cm). 3 cm is a loose lower bound for both.
                    assert!(
                        bend.length() > 0.03,
                        "{delta:?} frame {k}: expected a bent knee, got {bend:?}"
                    );
                    assert!(
                        bend.normalize().z < -0.9,
                        "knee bent backward for {delta:?}: {bend:?}"
                    );
                } else {
                    // At full extension there is no bend direction to test,
                    // so the leg must really be straight.
                    assert!(
                        bend.length() < STRAIGHT_KNEE_TOLERANCE_M,
                        "{delta:?} frame {k}: a stretched leg bent by {bend:?}"
                    );
                }
            }
        }
    }
}

/// Reach a straight leg gives up when asked to shorten by `d`: the solver's
/// soft full extension, a cubic deadband `d³/(d² + s²)` with s = 5 mm.
fn soft_extension(d: f32) -> f32 {
    d * d * d / (d * d + 0.005 * 0.005)
}

#[test]
fn sub_millimetre_vertical_noise_on_a_locked_straight_standing_leg_barely_moves_the_knee() {
    // Standing upright the flexion guard is off (the floor prior owns the
    // leg), so only the soft full extension keeps the knee still. A straight
    // leg asked to shorten by δ shortens by e = δ³/(δ² + s²) (s = 5 mm), and a
    // leg shortened by e has its knee sqrt(L·e) off the line with
    // L = 2·thigh·calf/(thigh + calf) = 0.44 m. The knee offset is therefore
    // sqrt(L·δ³/(δ² + s²)) ≈ δ^1.5·sqrt(L)/s: 0.52 mm for ±0.25 mm of noise,
    // where the bare square root would give sqrt(0.44 m·0.25 mm) = 10 mm.
    // Along the leg the knee rides with the pelvis, at most 2δ from one frame
    // to the next.
    let noise = 0.00025_f32;
    let offset = (2.0 * THIGH * CALF / (THIGH + CALF) * soft_extension(noise)).sqrt();
    assert!((0.00045..0.00058).contains(&offset), "{offset}");
    let bound = offset.hypot(2.0 * noise) + STRAIGHT_KNEE_TOLERANCE_M;
    let pose = Pose::standing(0.0);
    for root in [false, true] {
        let mut solver = LegIkSolver::new(LegIkConfig {
            root_correction_enabled: root,
            ..Default::default()
        });
        let mut previous = solver.solve(&pose.input(0.0, [true; 2])).knees;
        let (mut largest, mut largest_offset) = (0.0_f32, 0.0_f32);
        for k in 1..=200 {
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            let input = shifted(
                pose.input(k as f64 * DT, [true; 2]),
                Vec3::Y * (sign * noise),
            );
            let out = solver.solve(&input);
            for s in 0..2 {
                assert!(out.foot_locked[s]);
                assert_bone_lengths(&out, s);
                let step = (out.knees[s] - previous[s]).length();
                largest = largest.max(step);
                assert!(
                    step < bound,
                    "root {root} frame {k}: 0.25 mm of noise moved the knee {step} m"
                );
                // The part that shows: the knee leaving the leg line.
                let bend = perpendicular(
                    out.knees[s] - out.hip_joints[s],
                    out.ankles[s] - out.hip_joints[s],
                )
                .length();
                largest_offset = largest_offset.max(bend);
                assert!(
                    bend < offset + STRAIGHT_KNEE_TOLERANCE_M,
                    "root {root} frame {k}: knee {bend} m off the leg line"
                );
            }
            previous = out.knees;
        }
        eprintln!(
            "standing leg, root {root}: largest knee step {largest:.6} m (bound {bound:.6}), largest offset {largest_offset:.6} m (model {offset:.6})"
        );
    }
}

#[test]
fn near_full_extension_a_locked_ankle_slides_by_at_most_half_the_soft_extension_length() {
    // The price of the deadband: a straight locked leg whose hip comes d closer
    // to the anchor shortens by d³/(d² + s²) only, so the ankle passes its
    // anchor by d·s²/(d² + s²). That is nearly all of a sub-millimetre d, at
    // most s/2 = 2.5 mm (at d = s = 5 mm), and fades for a real bend.
    let pose = Pose::standing(0.0);
    let config = LegIkConfig {
        root_correction_enabled: false,
        // Keep the anchor where it was planted: below the floor height its
        // height would follow FK by 3 % per frame.
        anchor_vertical_tau_secs: 1.0e6,
        ..Default::default()
    };
    let mut worst = 0.0_f32;
    for millimetres in [0.25_f32, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0] {
        let d = millimetres * 0.001;
        let mut solver = LegIkSolver::new(config);
        let anchors = solver.solve(&pose.input(0.0, [true; 2])).ankles;
        let out = solver.solve(&shifted(pose.input(DT, [true; 2]), Vec3::NEG_Y * d));
        let expected = d - soft_extension(d);
        for s in 0..2 {
            assert!(out.foot_locked[s]);
            assert_bone_lengths(&out, s);
            let slide = anchors[s] - out.ankles[s];
            // Straight down the leg line, never sideways.
            assert!(
                slide.x.abs() < 1.0e-5 && slide.z.abs() < 1.0e-5,
                "{slide:?}"
            );
            assert!(
                (slide.y - expected).abs() < 5.0e-5,
                "{millimetres} mm closer: ankle {} m below its anchor, model {expected}",
                slide.y
            );
            assert!(slide.y < 0.0025 + 5.0e-5, "{millimetres} mm: {}", slide.y);
            worst = worst.max(slide.y);
        }
    }
    assert!(worst > 0.0024, "the sweep includes the worst case: {worst}");
}

#[test]
fn flat_foot_weight_reaches_one_upright_and_stays_zero_kneeling() {
    let mut pose = Pose::standing(0.1);
    let heading = 0.7_f32;
    pose.calves = [Quat::from_rotation_y(heading) * Quat::from_rotation_x(-0.2); 2];
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let mut weights = Vec::new();
    let mut out = None;
    for k in 0..=12 {
        let o = solver.solve(&pose.input(k as f64 * DT, [true; 2]));
        weights.push(o.flat_weight[0]);
        out = Some(o);
    }
    // Ramp: 0 on the plant frame, ~0.5 after 40 ms, 1 from 80 ms.
    assert_eq!(weights[0], 0.0);
    assert!((weights[4] - 0.5).abs() < 0.01, "{weights:?}");
    assert!(weights[8] > 0.999, "{weights:?}");
    assert_eq!(weights[9], 1.0);
    let out = out.unwrap();
    assert_eq!(out.flat_weight, [1.0, 1.0]);
    let flat = Quat::from_rotation_y(heading);
    assert!(out.foot_rotations[0].angle_to(flat) < 1.0e-3);

    // Kneeling: calf horizontal (90 degrees from vertical).
    let mut kneel = Pose::standing(0.0);
    kneel.hip.y = 0.6;
    kneel.thighs = [Quat::IDENTITY; 2];
    kneel.calves = [Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2); 2];
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    for k in 0..=30 {
        let input = kneel.input(k as f64 * DT, [true; 2]);
        let out = solver.solve(&input);
        assert_eq!(out.flat_weight, [0.0, 0.0]);
        for s in 0..2 {
            assert!(out.foot_rotations[s].angle_to(input.calf_rotations[s]) < 1.0e-5);
        }
    }

    // Halfway through the tilt band (50 degrees) the smoothstep gives 0.5.
    let mut half = Pose::standing(0.0);
    half.calves = [Quat::from_rotation_x(-50.0_f32.to_radians()); 2];
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let mut out = None;
    for k in 0..=12 {
        out = Some(solver.solve(&half.input(k as f64 * DT, [true; 2])));
    }
    assert!((out.unwrap().flat_weight[0] - 0.5).abs() < 1.0e-3);
}

#[test]
fn measured_foot_rotation_always_wins() {
    let pose = Pose::standing(0.1);
    let measured = Quat::from_rotation_y(-0.4) * Quat::from_rotation_x(0.3);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    for k in 0..=20 {
        let mut input = pose.input(k as f64 * DT, [true; 2]);
        input.measured_foot_rotations = [
            Some(measured),
            Some(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
        ];
        let out = solver.solve(&input);
        assert!(out.foot_rotations[0].angle_to(measured) < 1.0e-5);
        assert_eq!(out.flat_weight[0], 0.0);
        // An invalid measured rotation is ignored, not trusted.
        assert!(out.foot_rotations[1].is_finite());
    }
}

#[test]
fn the_solver_never_bends_a_knee_for_the_floor() {
    // The floor is handled upstream by `SupportFloor`, which raises the whole
    // lower body (tests/support_floor.rs). The ankle-only clamp that used to bend
    // the knees instead of moving the body is gone: below-floor FK input with
    // nothing planted passes through unchanged.
    let mut pose = Pose::standing(0.3);
    pose.hip.y = 0.88; // FK ankles end up below ankle_to_sole - 1 cm.
    let input = pose.input(0.0, [false; 2]);
    assert!(input.ankles[0].y < ANKLE_TO_SOLE - 0.01);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let out = solver.solve(&input);
    assert_eq!(bits(&out), input_bits(&input));
    for s in 0..2 {
        assert_bone_lengths(&out, s);
    }
}

#[test]
fn a_flexed_torso_widens_the_lock_break_and_root_limits() {
    // 2 cm of FK wobble always locks; a 12 cm shift breaks the lock with an
    // upright torso (7 cm radius) but not with a 70 degree bow, whose torso
    // slack 0.3·(0.25·sin 70° + 0.30·sin 70°) ≈ 15.5 cm widens it.
    let run = |torso_pitch: f32| {
        let pose = Pose::standing(0.2);
        let mut solver = LegIkSolver::new(LegIkConfig::default());
        let bow = Quat::from_rotation_x(-torso_pitch);
        let mut out = None;
        for k in 0..=40 {
            let mut input = pose.input(k as f64 * DT, [true; 2]);
            input.chest_rotation = bow;
            input.spine_rotation = bow;
            let shift = if k == 0 {
                Vec3::ZERO
            } else {
                Vec3::new(0.0, 0.0, 0.12)
            };
            out = Some(solver.solve(&shifted(input, shift)));
        }
        out.unwrap()
    };
    let upright = run(0.0);
    assert!((upright.lock_break_m - 0.07).abs() < 1.0e-6);
    assert!((upright.root_max_m - 0.05).abs() < 1.0e-6);
    assert!(!upright.foot_locked[0] && upright.release_requests[0]);
    let bowed = run(70.0_f32.to_radians());
    let slack = 0.3 * (NECK_TO_CHEST + CHEST_TO_HIP) * 70.0_f32.to_radians().sin();
    assert!((bowed.lock_break_m - (0.07 + slack)).abs() < 1.0e-5);
    assert!((bowed.root_max_m - (0.05 + slack)).abs() < 1.0e-5);
    assert!(bowed.foot_locked[0] && bowed.foot_locked[1]);
    assert!(!bowed.release_requests[0]);
    // The root took more than the upright 5 cm limit.
    assert!(Vec3::from_array(bowed.root_correction).length() > 0.06);
    for s in 0..2 {
        assert_bone_lengths(&bowed, s);
    }
    // Zero gain restores the fixed limits.
    let pose = Pose::standing(0.2);
    let mut solver = LegIkSolver::new(LegIkConfig {
        torso_slack_gain: 0.0,
        ..Default::default()
    });
    let mut input = pose.input(0.0, [true; 2]);
    input.chest_rotation = Quat::from_rotation_x(-1.2);
    input.spine_rotation = Quat::from_rotation_x(-1.2);
    let out = solver.solve(&input);
    assert_eq!(out.lock_break_m, 0.07);
    assert_eq!(out.root_max_m, 0.05);
}

#[test]
fn disabled_and_settled_unplanted_solver_pass_fk_through_bit_for_bit() {
    let pose = Pose::standing(0.25);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    for k in 0..30 {
        let mut input = shifted(
            pose.input(k as f64 * DT, [true; 2]),
            Vec3::new(0.0, 0.0, -0.0),
        );
        input.enabled = false;
        input.ankles[0].x = -0.0;
        let out = solver.solve(&input);
        assert_eq!(out.status, LegIkStatus::Disabled);
        assert_eq!(bits(&out), input_bits(&input));
        for s in 0..2 {
            assert!(out.foot_rotations[s].angle_to(input.calf_rotations[s]) < 1.0e-5);
        }
    }

    // Never planted: passthrough from the first frame.
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let input = pose.input(0.0, [false; 2]);
    assert_eq!(bits(&solver.solve(&input)), input_bits(&input));

    // Plant, accumulate correction and offsets, release: returns to exact FK.
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let mut t = 0.0;
    for k in 0..50 {
        t = k as f64 * DT;
        let wobble = Vec3::new(0.03 * (k as f32 * 0.2).sin(), 0.0, 0.01);
        solver.solve(&shifted(pose.input(t, [true; 2]), wobble));
    }
    let mut exact_from = None;
    for k in 1..=200 {
        let input = pose.input(t + k as f64 * DT, [false; 2]);
        let out = solver.solve(&input);
        if bits(&out) == input_bits(&input) {
            exact_from.get_or_insert(k);
        } else {
            assert!(
                exact_from.is_none(),
                "passthrough must persist once reached"
            );
        }
    }
    let exact_from = exact_from.expect("never returned to exact FK passthrough");
    assert!(exact_from < 150, "took {exact_from} frames");
}

#[test]
fn invalid_input_holds_output_and_does_not_poison_state() {
    let pose = Pose::standing(0.2);
    let mut clean = LegIkSolver::new(LegIkConfig::default());
    let mut dirty = LegIkSolver::new(LegIkConfig::default());
    let fresh = LegIkSolver::new(LegIkConfig::default()).solve(&{
        let mut i = pose.input(0.0, [true; 2]);
        i.hip.x = f32::NAN;
        i
    });
    assert_eq!(fresh.status, LegIkStatus::Unavailable);
    assert_finite(&fresh);

    for k in 0..80 {
        let t = k as f64 * DT;
        let wobble = Vec3::new(0.02 * (k as f32 * 0.3).sin(), 0.0, 0.0);
        let input = shifted(pose.input(t, [k > 5; 2]), wobble);
        let expected = clean.solve(&input);
        if k % 7 == 3 {
            let mut corrupted = [input; 6];
            corrupted[0].ankles[1].y = f32::NAN;
            corrupted[1].thigh_rotations[0] = Quat::from_xyzw(0.0, f32::INFINITY, 0.0, 1.0);
            corrupted[2].thigh_m = 0.0;
            corrupted[3].at_secs = f64::NAN;
            corrupted[4].calf_rotations[1] = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
            corrupted[5].knees[0] = Vec3::splat(f32::NEG_INFINITY);
            for bad in corrupted {
                let held = dirty.solve(&bad);
                assert_eq!(held.status, LegIkStatus::HeldLastOutput);
                assert_finite(&held);
            }
        }
        let out = dirty.solve(&input);
        assert_eq!(bits(&out), bits(&expected), "frame {k}");
        assert_eq!(out.root_correction, expected.root_correction);
    }
}

#[test]
fn time_going_backwards_or_stalling_is_finite_and_does_not_advance_filters() {
    let pose = Pose::standing(0.2);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    solver.solve(&pose.input(1.0, [true; 2]));
    let shift = Vec3::new(0.03, 0.0, 0.0);
    let a = solver.solve(&shifted(pose.input(1.0, [true; 2]), shift));
    let b = solver.solve(&shifted(pose.input(0.5, [true; 2]), shift));
    // dt <= 0: the root correction filter has not moved yet.
    assert_eq!(a.root_correction, [0.0; 3]);
    assert_eq!(b.root_correction, [0.0; 3]);
    assert_finite(&b);
    let c = solver.solve(&shifted(pose.input(0.51, [true; 2]), shift));
    assert!(Vec3::from_array(c.root_correction).length() > 0.0);
}

#[test]
fn switches_disable_individual_components() {
    let pose = Pose::standing(0.2);
    let delta = Vec3::new(0.03, 0.0, 0.0);
    let run = |config: LegIkConfig| {
        let mut solver = LegIkSolver::new(config);
        solver.solve(&pose.input(0.0, [true; 2]));
        let mut out = None;
        for k in 1..=40 {
            out = Some(solver.solve(&shifted(pose.input(k as f64 * DT, [true; 2]), delta)));
        }
        out.unwrap()
    };
    let no_root = run(LegIkConfig {
        root_correction_enabled: false,
        ..Default::default()
    });
    assert_eq!(no_root.root_correction, [0.0; 3]);
    assert!(no_root.foot_locked[0]);
    let no_lock = run(LegIkConfig {
        foot_lock_enabled: false,
        ..Default::default()
    });
    assert!(!no_lock.foot_locked[0]);
    let root = Vec3::from_array(no_lock.root_correction);
    assert!(root.x < -0.015, "root correction still active: {root:?}");
    assert_bone_lengths(&no_lock, 0);
    let no_flat = run(LegIkConfig {
        flat_foot_enabled: false,
        ..Default::default()
    });
    assert_eq!(no_flat.flat_weight, [0.0; 2]);
}

#[test]
fn diagnostics_serialize() {
    let pose = Pose::standing(0.2);
    let mut solver = LegIkSolver::new(LegIkConfig::default());
    let out = solver.solve(&pose.input(0.0, [true, false]));
    let json = serde_json::to_string(&out.diagnostics()).unwrap();
    assert!(json.contains("\"status\":\"active\""));
    let config: LegIkConfig = serde_json::from_str("{\"root_gain\":0.4}").unwrap();
    assert_eq!(config.root_gain, 0.4);
    assert!(config.foot_lock_enabled);
}
