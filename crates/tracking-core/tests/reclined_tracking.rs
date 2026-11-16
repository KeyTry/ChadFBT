//! Synthetic integration regressions for horizontal legs after public calibration.
//! The body uses the estimator's rigid-chain model; these tests establish numerical
//! invariants and stale-input safety, not accuracy on a person or a bed.

use glam::{Quat, Vec3};
use tracking_core::chain::{BodyChain, ChainDimensions, ChainPose, solve_chain};
use tracking_core::forward_calibration::ski_pitch_sign;
use tracking_core::leg_ik::LegIkConfig;
use tracking_core::*;

fn sensor(role: Role, at: f64, bone: Quat) -> SensorSample {
    SensorSample {
        id: format!("reclined-{role:?}"),
        orientation_xyzw: bone.to_array(),
        received_at_secs: at,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    }
}

fn anchors(at: f64, position: Vec3, rotation: Quat) -> Anchors {
    Anchors {
        hmd: Some(AnchorPose {
            position: position.to_array(),
            orientation_xyzw: rotation.to_array(),
            received_at_secs: at,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        }),
        ..Default::default()
    }
}

fn calibrated(yaw: f32) -> TrackingCore {
    calibrated_with(yaw, |_| {})
}

/// A calibrated core whose leg IK configuration was adjusted by `tune`
/// (ablations that show what a default protects against).
fn calibrated_with(yaw: f32, tune: impl FnOnce(&mut LegIkConfig)) -> TrackingCore {
    let mut config = TrackingConfig::default();
    config.estimator.enable_leg_ik = true;
    tune(&mut config.estimator.leg_ik);
    // Isolate position constraints from the independent heading estimator.
    config.estimator.enable_stance_heading = false;
    let eye = config.anthropometrics.standing_eye_height_m;
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(
        Role::CORE
            .map(|role| RoleAssignment {
                sensor_id: format!("reclined-{role:?}"),
                role,
            })
            .to_vec(),
    )
    .unwrap();
    let heading = Quat::from_rotation_y(yaw);
    for tick in 0..=200 {
        let at = tick as f64 * 0.01;
        let skiing = tick > 100;
        for role in Role::CORE {
            let bend = if skiing {
                ski_pitch_sign(role).unwrap() * 30.0_f32.to_radians()
            } else {
                0.0
            };
            core.ingest(sensor(role, at, heading * Quat::from_rotation_x(bend)))
                .unwrap();
        }
        let head = anchors(
            at,
            Vec3::new(0.0, if skiing { eye - 0.45 } else { eye }, 0.0),
            heading,
        );
        core.solve(at, &head);
        if tick == 100 {
            core.calibrate_neutral(at, &head, false).unwrap();
        }
    }
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}

fn joint(frame: &TrackingFrame, role: JointRole) -> Vec3 {
    Vec3::from_array(
        frame
            .joints
            .iter()
            .find(|j| j.joint == role)
            .unwrap()
            .position,
    )
}

fn reclining_frame(core: &mut TrackingCore, yaw: f32, tick: usize) -> TrackingFrame {
    reclining_frame_with_residual(core, yaw, tick, 0.00005)
}

/// Supine on a 0.65 m support. After settling, the entire FK chain steps
/// `away_from_feet` meters along the legs (negative: toward the feet), so a
/// locked endpoint then differs from rigid FK by a tiny, nonzero residual.
fn reclining_frame_with_residual(
    core: &mut TrackingCore,
    yaw: f32,
    tick: usize,
    away_from_feet: f32,
) -> TrackingFrame {
    let residual = if tick >= 100 {
        Quat::from_rotation_y(yaw) * Vec3::Z * away_from_feet
    } else {
        Vec3::ZERO
    };
    pose_frame(
        core,
        yaw,
        tick,
        std::f32::consts::FRAC_PI_2,
        Vec3::new(0.0, 0.65, 0.0),
        residual,
    )
}

fn pose_frame(
    core: &mut TrackingCore,
    yaw: f32,
    tick: usize,
    tilt: f32,
    hip: Vec3,
    residual: Vec3,
) -> TrackingFrame {
    let at = 2.2 + tick as f64 * 0.01;
    let heading = Quat::from_rotation_y(yaw);
    let rotation = heading * Quat::from_rotation_x(tilt);
    let body = &core.config().anthropometrics;
    let dimensions = ChainDimensions::from_anthropometrics(body, body.eye_forward_m);
    let zero = solve_chain(
        &dimensions,
        &ChainPose {
            head_position: Vec3::ZERO,
            head_rotation: rotation,
            rotations: [rotation; 6],
        },
    );
    for role in Role::CORE {
        core.ingest(sensor(role, at, rotation)).unwrap();
    }
    core.solve(at, &anchors(at, hip - zero.hip + residual, rotation))
}

#[test]
fn reclined_locked_legs_remain_level_under_tiny_headset_residual_at_any_yaw() {
    // 50 µm in either direction along the legs. Away from the feet a straight
    // leg stays straight; toward them it is asked to shorten by at most d = 50 µm,
    // of which the soft full extension passes on d³/(d² + s²) (s = 5 mm), 5 nm:
    // a knee offset of sqrt(0.42 m·5 nm) = 0.05 mm. 0.3 mm on top covers f32
    // rounding at full extension.
    for (yaw, residual) in [
        (0.0, 0.00005),
        (0.8, 0.00005),
        (-1.3, 0.00005),
        (0.0, -0.00005),
        (0.8, -0.00005),
        (-1.3, -0.00005),
    ] {
        let mut core = calibrated(yaw);
        let mut exercised_lock = false;
        let mut worst_knee_height = 0.0_f32;
        for tick in 0..180 {
            let frame = reclining_frame_with_residual(&mut core, yaw, tick, residual);
            assert!(frame.live_anchor && !frame.preview_only);
            assert!(frame.targets.iter().all(|target| target.valid));
            if tick < 100 {
                continue;
            }
            exercised_lock |= frame.estimator.leg_ik.foot_locked == [true; 2];
            assert!(frame.estimator.support_floor.vertical_m.abs() < 1.0e-5);
            for (hip, knee, ankle) in [
                (
                    JointRole::LeftHip,
                    JointRole::LeftKnee,
                    JointRole::LeftAnkle,
                ),
                (
                    JointRole::RightHip,
                    JointRole::RightKnee,
                    JointRole::RightAnkle,
                ),
            ] {
                let (hip, knee, ankle) = (
                    joint(&frame, hip),
                    joint(&frame, knee),
                    joint(&frame, ankle),
                );
                worst_knee_height = worst_knee_height.max((knee.y - hip.y).abs());
                assert!(
                    (knee.y - hip.y).abs() < 0.00035,
                    "yaw {yaw}, tick {tick}: residual {residual} raised knee {:.6} m relative to hip",
                    knee.y - hip.y
                );
                let body = &core.config().anthropometrics;
                assert!(((knee - hip).length() - body.thigh_length_m).abs() < 2.0e-5);
                assert!(((ankle - knee).length() - body.calf_length_m).abs() < 2.0e-5);
            }
        }
        assert!(
            exercised_lock,
            "test must exercise both foot locks at yaw {yaw}"
        );
        eprintln!(
            "reclined yaw {yaw}, residual {residual}: worst knee/hip height difference {worst_knee_height:.9} m"
        );
    }
}

#[test]
fn standing_anchors_do_not_drag_a_reclined_platform_pose_toward_the_floor() {
    for yaw in [0.0, 0.8, -1.3] {
        let mut core = calibrated(yaw);
        let body = &core.config().anthropometrics;
        let standing_hip = body.ankle_to_sole_m + body.thigh_length_m + body.calf_length_m;
        let mut started_planted = false;
        let mut released_during_transition = false;
        let mut replanted_above_floor = false;
        for tick in 0..500 {
            let progress = ((tick as f32 - 100.0) / 200.0).clamp(0.0, 1.0);
            let progress = progress * progress * (3.0 - 2.0 * progress);
            // Purely invented elevated support plane. This does not assert a bed
            // can be detected, only that existing standing anchors must release.
            let hip = Vec3::new(0.0, standing_hip + (0.65 - standing_hip) * progress, 0.0);
            let frame = pose_frame(
                &mut core,
                yaw,
                tick,
                progress * std::f32::consts::FRAC_PI_2,
                hip,
                Vec3::ZERO,
            );
            let locked = frame.estimator.leg_ik.foot_locked;
            if tick == 99 {
                started_planted = locked == [true; 2];
            }
            if (101..300).contains(&tick) && locked == [false; 2] {
                released_during_transition = true;
            }
            if tick >= 400 {
                replanted_above_floor |= locked == [true; 2];
                for role in [JointRole::Hip, JointRole::LeftAnkle, JointRole::RightAnkle] {
                    assert!(
                        (joint(&frame, role).y - 0.65).abs() < 0.003,
                        "yaw {yaw}, tick {tick}: standing anchor displaced {role:?} from elevated support"
                    );
                }
                for (root, knee) in [
                    (JointRole::LeftHip, JointRole::LeftKnee),
                    (JointRole::RightHip, JointRole::RightKnee),
                ] {
                    assert!((joint(&frame, root).y - joint(&frame, knee).y).abs() < 0.005);
                }
            }
        }
        assert!(started_planted && released_during_transition && replanted_above_floor);
    }
}

#[test]
fn chair_pose_keeps_model_height_without_promoting_stationary_feet_to_floor_support() {
    for yaw in [0.0, 0.8, -1.3] {
        let mut core = calibrated(yaw);
        let heading = Quat::from_rotation_y(yaw);
        let torso = heading * Quat::from_rotation_x(55.0_f32.to_radians());
        let thigh = heading * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        let rotations = [torso, torso, thigh, thigh, heading, heading];
        let body = &core.config().anthropometrics;
        let hip_height = body.calf_length_m + body.ankle_to_sole_m + 0.04;
        let dimensions = ChainDimensions::from_anthropometrics(body, body.eye_forward_m);
        let zero = solve_chain(
            &dimensions,
            &ChainPose {
                head_position: Vec3::ZERO,
                head_rotation: heading,
                rotations,
            },
        );
        let head_position = Vec3::Y * hip_height - zero.hip;
        let mut locked_stationary_feet = false;
        for tick in 0..200 {
            let at = 2.2 + tick as f64 * 0.01;
            for (role, rotation) in Role::CORE.into_iter().zip(rotations) {
                core.ingest(sensor(role, at, rotation)).unwrap();
            }
            let frame = core.solve(at, &anchors(at, head_position, heading));
            if tick < 100 {
                continue;
            }
            locked_stationary_feet |= frame.estimator.leg_ik.foot_locked == [true; 2];
            assert!(frame.estimator.support_floor.vertical_m.abs() < 1.0e-5);
            assert!(
                (joint(&frame, JointRole::Hip).y - hip_height).abs() < 0.001,
                "yaw {yaw}: hypothetical floor support lowered the seated pelvis"
            );
            for (hip, knee) in [
                (JointRole::LeftHip, JointRole::LeftKnee),
                (JointRole::RightHip, JointRole::RightKnee),
            ] {
                assert!((joint(&frame, knee).y - joint(&frame, hip).y).abs() < 0.001);
            }
            for target in frame.targets.iter().filter(|target| {
                matches!(
                    target.role,
                    TargetRole::Waist | TargetRole::LeftFoot | TargetRole::RightFoot
                )
            }) {
                assert!(target.valid);
            }
        }
        assert!(
            locked_stationary_feet,
            "reduced floor prior must retain stationary foot locks"
        );
    }
}

#[test]
fn reclined_contact_never_keeps_targets_valid_after_headset_or_limb_stales() {
    for missing_headset in [true, false] {
        let mut core = calibrated(0.0);
        let mut frame = reclining_frame(&mut core, 0.0, 0);
        for tick in 1..180 {
            frame = reclining_frame(&mut core, 0.0, tick);
        }
        assert!(
            frame
                .estimator
                .leg_ik
                .foot_locked
                .iter()
                .all(|locked| *locked)
        );
        let at = frame.at_secs + core.config().stale_after_secs + 0.01;
        let head_rotation = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        for role in Role::CORE {
            if missing_headset || role != Role::LeftCalf {
                core.ingest(sensor(role, at, head_rotation)).unwrap();
            }
        }
        let head = if missing_headset {
            Anchors::default()
        } else {
            anchors(at, joint(&frame, JointRole::Head), head_rotation)
        };
        let stale = core.solve(at, &head);
        for target in &stale.targets {
            assert!(
                target
                    .position
                    .iter()
                    .chain(&target.orientation_xyzw)
                    .all(|v| v.is_finite())
            );
            if missing_headset || target.role == TargetRole::LeftFoot {
                assert!(
                    !target.valid,
                    "stale required evidence authorized {:?}",
                    target.role
                );
            }
        }
        if missing_headset {
            assert!(!stale.estimator.leg_ik_active);
        }
    }
}

// ---------------------------------------------------------------------------
// Poses with exactly known truth: the headset is placed by the estimator's own
// rigid chain, so head-down FK reproduces the pose and any difference in the
// published joints is something a prior or the leg IK did.

const LEGS: [(JointRole, JointRole, JointRole); 2] = [
    (
        JointRole::LeftHip,
        JointRole::LeftKnee,
        JointRole::LeftAnkle,
    ),
    (
        JointRole::RightHip,
        JointRole::RightKnee,
        JointRole::RightAnkle,
    ),
];

/// The estimator's rigid chain for these `Role::CORE` rotations, translated so
/// that the point `pick` selects lies at `at`. Returns the headset position too.
fn placed_chain(
    core: &TrackingCore,
    head_rotation: Quat,
    rotations: [Quat; 6],
    pick: fn(&BodyChain) -> Vec3,
    at: Vec3,
) -> (Vec3, BodyChain) {
    let body = &core.config().anthropometrics;
    let dimensions = ChainDimensions::from_anthropometrics(body, body.eye_forward_m);
    let solve = |head_position: Vec3| {
        solve_chain(
            &dimensions,
            &ChainPose {
                head_position,
                head_rotation,
                rotations,
            },
        )
    };
    let head = at - pick(&solve(Vec3::ZERO));
    (head, solve(head))
}

/// Feeds one frame of measured segment rotations and a headset pose.
fn feed(
    core: &mut TrackingCore,
    tick: usize,
    head_position: Vec3,
    head_rotation: Quat,
    rotations: [Quat; 6],
) -> TrackingFrame {
    let at = 2.2 + tick as f64 * 0.01;
    for (role, rotation) in Role::CORE.into_iter().zip(rotations) {
        core.ingest(sensor(role, at, rotation)).unwrap();
    }
    let frame = core.solve(at, &anchors(at, head_position, head_rotation));
    assert!(frame.live_anchor && !frame.preview_only);
    frame
}

fn assert_leg_bone_lengths(core: &TrackingCore, frame: &TrackingFrame, context: &str) {
    let body = &core.config().anthropometrics;
    for (hip, knee, ankle) in LEGS {
        let (hip, knee, ankle) = (joint(frame, hip), joint(frame, knee), joint(frame, ankle));
        let (thigh, calf) = ((knee - hip).length(), (ankle - knee).length());
        assert!(
            (thigh - body.thigh_length_m).abs() < 5.0e-5
                && (calf - body.calf_length_m).abs() < 5.0e-5,
            "{context}: thigh {thigh}, calf {calf}"
        );
    }
}

/// Distance of the knee from the hip-ankle line.
fn knee_offset(hip: Vec3, knee: Vec3, ankle: Vec3) -> f32 {
    let line = (ankle - hip).normalize();
    (knee - hip).cross(line).length()
}

/// Knee offset of a straight leg flexed by `slack`: `thigh·calf·sin(slack)/reach`
/// (twice the area of the triangle over its base), 3.7 mm for 0.42 m bones and 1°.
fn guard_knee_offset(body: &Anthropometrics, slack: f32) -> f32 {
    let (t, c) = (body.thigh_length_m, body.calf_length_m);
    t * c * slack.sin() / (t * t + c * c + 2.0 * t * c * slack.cos()).sqrt()
}

/// Tolerance on [`guard_knee_offset`]. Near a straight leg the solver recovers
/// the interior angle with `acos(−1 + ε)`, which turns a few f32 ulps of ε into
/// up to ~1e-3 rad, i.e. 0.2 mm of knee offset.
const GUARD_TOLERANCE_M: f32 = 3.0e-4;

/// Both legs straight and level, locked while still; then the headset drifts
/// `drift` meters along the legs at 5 cm/s (positive: toward the feet) and
/// holds. Returns the largest knee offset from the hip-ankle line.
fn flat_leg_drift(yaw: f32, torso_tilt: f32, hip_height: f32, slack: f32, drift: f32) -> f32 {
    let mut core = calibrated_with(yaw, |ik| ik.knee_flex_slack_rad = slack);
    let heading = Quat::from_rotation_y(yaw);
    let torso = heading * Quat::from_rotation_x(torso_tilt);
    let flat = heading * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let rotations = [torso, torso, flat, flat, flat, flat];
    let (head, truth) = placed_chain(
        &core,
        torso,
        rotations,
        |chain| chain.hip,
        Vec3::Y * hip_height,
    );
    // The feet lie toward heading·(−Z) of the pelvis.
    let toward_feet = heading * Vec3::NEG_Z;
    assert!(
        (truth.ankles[0] - truth.hip_joints[0])
            .normalize()
            .dot(toward_feet)
            > 0.9999
    );
    let mut worst = 0.0_f32;
    for tick in 0..260_usize {
        let moved = (tick.saturating_sub(100) as f32 * 0.0005).min(drift.abs()) * drift.signum();
        let frame = feed(
            &mut core,
            tick,
            head + toward_feet * moved,
            torso,
            rotations,
        );
        if tick < 100 {
            continue;
        }
        let context =
            format!("yaw {yaw} tilt {torso_tilt} slack {slack} drift {drift} tick {tick}");
        assert!(frame.targets.iter().all(|target| target.valid), "{context}");
        assert_eq!(frame.estimator.leg_ik.foot_locked, [true; 2], "{context}");
        assert!(
            frame.estimator.support_floor.vertical_m.abs() < 1.0e-5,
            "{context}"
        );
        // Nothing lowers or raises the pelvis of a body that lies or sits flat.
        assert!(
            (joint(&frame, JointRole::Hip).y - hip_height).abs() < 1.0e-3,
            "{context}: pelvis at {}",
            joint(&frame, JointRole::Hip).y
        );
        assert_leg_bone_lengths(&core, &frame, &context);
        for (hip, knee, ankle) in LEGS {
            let (hip, knee, ankle) = (
                joint(&frame, hip),
                joint(&frame, knee),
                joint(&frame, ankle),
            );
            // The knee can only leave the leg line upward (the thighs face up)...
            assert!((knee_offset(hip, knee, ankle) - (knee.y - hip.y).abs()).abs() < 1.0e-4);
            assert!(knee.y - hip.y > -GUARD_TOLERANCE_M, "{context}");
            // ...and the ankle only slides along it.
            assert!((ankle.y - hip.y).abs() < 1.0e-4, "{context}");
            worst = worst.max(knee.y - hip.y);
        }
    }
    worst
}

#[test]
fn headset_drift_along_locked_flat_legs_raises_the_knees_by_at_most_the_flexion_guard() {
    let body = Anthropometrics::default();
    let slack = LegIkConfig::default().knee_flex_slack_rad;
    let bound = guard_knee_offset(&body, slack);
    assert!((bound - body.thigh_length_m * (0.5 * slack).sin()).abs() < 1.0e-6);
    assert!(bound < 0.005, "{bound}");
    // Reach lost by a straight leg asked to shorten by `d`, and the knee offset
    // that goes with it (soft full extension d³/(d² + s²), s = 5 mm; equal bones).
    let soft = |d: f32| d * d * d / (d * d + 0.005_f32 * 0.005);
    let unguarded_offset = |d: f32| (body.thigh_length_m * soft(d)).sqrt();
    for (label, torso_tilt, hip_height) in [
        ("supine on a platform", std::f32::consts::FRAC_PI_2, 0.65),
        ("long sitting on the floor", 0.0, body.ankle_to_sole_m),
    ] {
        for yaw in [0.0, 0.8] {
            for size in [0.001_f32, 0.005, 0.020] {
                // Away from the feet a straight leg just stays straight.
                let away = flat_leg_drift(yaw, torso_tilt, hip_height, slack, -size);
                assert!(
                    away < GUARD_TOLERANCE_M,
                    "{label}, yaw {yaw}: {size} m away from the feet raised the knees {away} m"
                );
                let toward = flat_leg_drift(yaw, torso_tilt, hip_height, slack, size);
                assert!(
                    toward < bound + GUARD_TOLERANCE_M,
                    "{label}, yaw {yaw}: {size} m toward the feet raised the knees {toward} m"
                );
                let unguarded =
                    flat_leg_drift(yaw, torso_tilt, hip_height, std::f32::consts::PI, size);
                eprintln!(
                    "{label}, yaw {yaw}, {size} m toward the feet: knees rise {toward:.5} m, {unguarded:.5} m without the guard"
                );
                // The pelvis settles on root_gain = 60 % of the drift, so the
                // legs are left 40 % short: 0.4, 2 and 8 mm. Without the guard
                // that is a knee offset of 1.0 mm, 1.1 cm and 4.9 cm.
                let expected = unguarded_offset(0.4 * size);
                assert!(
                    unguarded > 0.95 * expected,
                    "{label}, yaw {yaw}, {size}: {unguarded} without the guard, expected {expected}"
                );
                if size >= 0.02 {
                    assert!(unguarded > 0.04, "{label}: {unguarded}");
                }
                if expected > bound {
                    assert!(toward > bound - GUARD_TOLERANCE_M, "{label}: {toward}");
                    assert!(
                        unguarded > toward + 0.005,
                        "{label}: {unguarded} vs {toward}"
                    );
                }
            }
        }
    }
}

#[test]
fn supine_on_a_raised_platform_with_knees_drawn_up_keeps_the_pelvis_at_model_height() {
    for yaw in [0.0, 0.8, -1.3] {
        let mut core = calibrated(yaw);
        let body = core.config().anthropometrics.clone();
        let base = core.config().estimator.leg_ik.support_floor_base_m;
        let heading = Quat::from_rotation_y(yaw);
        // Torso flat on its back; knees up and toward the feet (thigh 50° from
        // vertical, its hip end pointing down: an axis tilt of 130°); shanks
        // 40° from vertical with the feet on a platform 0.5 m above the floor.
        let torso = heading * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        let thigh = heading * Quat::from_rotation_x(130.0_f32.to_radians());
        let calf = heading * Quat::from_rotation_x(40.0_f32.to_radians());
        let rotations = [torso, torso, thigh, thigh, calf, calf];
        let platform = 0.5;
        let feet = Vec3::Y * (platform + body.ankle_to_sole_m);
        let left_ankle = feet + heading * Vec3::X * (-0.5 * body.hip_width_m);
        let (head, truth) = placed_chain(&core, torso, rotations, |c| c.ankles[0], left_ankle);
        let knee_rise = body.thigh_length_m * 50.0_f32.to_radians().cos();
        assert!((truth.knees[0].y - truth.hip_joints[0].y - knee_rise).abs() < 1.0e-5);
        assert!((truth.ankles[1].y - feet.y).abs() < 1.0e-5);
        let mut locked = false;
        for tick in 0..250 {
            let frame = feed(&mut core, tick, head, torso, rotations);
            if tick < 100 {
                continue;
            }
            let context = format!("yaw {yaw} tick {tick}");
            assert!(frame.targets.iter().all(|target| target.valid));
            locked |= frame.estimator.leg_ik.foot_locked == [true; 2];
            let floor = frame.estimator.support_floor;
            assert!(
                (floor.gap_m - platform).abs() < 1.0e-3,
                "{context}: {floor:?}"
            );
            // Three independent reasons not to pull this body 0.5 m down, each
            // pinned on its own: a supine torso earns no bow budget (counting a
            // recline it would be 2 cm + 0.58 m = 60 cm, which covers the gap),
            // a thigh tilted 130° has no upright weight, and a 50 cm gap is not
            // explained by a 2 cm budget.
            assert!(floor.budget_m < base + 1.0e-3, "{context}: {floor:?}");
            assert!(floor.support_weight < 1.0e-3, "{context}: {floor:?}");
            assert!(floor.vertical_m.abs() < 1.0e-5, "{context}: {floor:?}");
            let hip = joint(&frame, JointRole::Hip);
            assert!(
                (hip - truth.hip).length() < 1.0e-3,
                "{context}: pelvis {hip:?}, model {:?}",
                truth.hip
            );
            for (s, (hip, knee, ankle)) in LEGS.into_iter().enumerate() {
                for (role, expected) in [
                    (hip, truth.hip_joints[s]),
                    (knee, truth.knees[s]),
                    (ankle, truth.ankles[s]),
                ] {
                    assert!(
                        (joint(&frame, role) - expected).length() < 1.0e-3,
                        "{context}: {role:?} at {:?}, model {expected:?}",
                        joint(&frame, role)
                    );
                }
            }
        }
        assert!(locked, "yaw {yaw}: the still feet must be locked");
    }
}

/// Seated with the thighs level and the shanks vertical, torso reclined by
/// `recline`, ankles `clearance` above their floor height. Returns the largest
/// drop of the published pelvis and of a published ankle below the model's,
/// and the largest pull-down budget, after settling.
fn reclined_chair(
    yaw: f32,
    recline: f32,
    clearance: f32,
    tune: impl FnOnce(&mut LegIkConfig),
) -> (f32, f32, f32) {
    let mut core = calibrated_with(yaw, tune);
    let body = core.config().anthropometrics.clone();
    let heading = Quat::from_rotation_y(yaw);
    let torso = heading * Quat::from_rotation_x(recline);
    let thigh = heading * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let rotations = [torso, torso, thigh, thigh, heading, heading];
    let hip_height = body.calf_length_m + body.ankle_to_sole_m + clearance;
    let (head, truth) = placed_chain(
        &core,
        heading,
        rotations,
        |chain| chain.hip,
        Vec3::Y * hip_height,
    );
    assert!((truth.ankles[0].y - body.ankle_to_sole_m - clearance).abs() < 1.0e-5);
    let (mut hip_drop, mut ankle_drop, mut budget) = (0.0_f32, 0.0_f32, 0.0_f32);
    let mut locked = false;
    for tick in 0..300 {
        let frame = feed(&mut core, tick, head, heading, rotations);
        if tick < 100 {
            continue;
        }
        assert!(frame.targets.iter().all(|target| target.valid));
        assert_leg_bone_lengths(&core, &frame, "reclined chair");
        locked |= frame.estimator.leg_ik.foot_locked == [true; 2];
        budget = budget.max(frame.estimator.support_floor.budget_m);
        hip_drop = hip_drop.max(truth.hip.y - joint(&frame, JointRole::Hip).y);
        for (s, (_, _, ankle)) in LEGS.into_iter().enumerate() {
            ankle_drop = ankle_drop.max(truth.ankles[s].y - joint(&frame, ankle).y);
        }
    }
    assert!(locked, "the still feet must be locked");
    (hip_drop, ankle_drop, budget)
}

#[test]
fn chair_reclined_thirty_degrees_with_dangling_feet_keeps_pelvis_and_feet_at_model_height() {
    let ablate_thigh_gate = |ik: &mut LegIkConfig| {
        ik.support_thigh_full_tilt_rad = std::f32::consts::PI;
        ik.support_thigh_zero_tilt_rad = std::f32::consts::PI;
    };
    let recline = 30.0_f32.to_radians();
    for yaw in [0.0, 0.8, -1.3] {
        for clearance in [0.10_f32, 0.03] {
            let (hip_drop, ankle_drop, budget) = reclined_chair(yaw, recline, clearance, |_| {});
            // A backward lean earns nothing beyond the 2 cm base (counting it,
            // 30° would give 2 cm + 0.58 m·(1 − cos 30°) = 9.8 cm).
            assert!(budget < 0.0201, "yaw {yaw}: budget {budget}");
            assert!(
                hip_drop < 1.0e-3 && ankle_drop < 1.0e-3,
                "yaw {yaw}, feet {clearance} m up: pelvis dropped {hip_drop} m, ankles {ankle_drop} m"
            );
            // Sitting upright, or leaning 15° forward (short of the 20° where
            // the squat rule starts), with the feet lifted: no pull either.
            for forward in [0.0_f32, 15.0] {
                let (hip_drop, ankle_drop, _) =
                    reclined_chair(yaw, -forward.to_radians(), clearance, |_| {});
                assert!(
                    hip_drop < 1.0e-3 && ankle_drop < 1.0e-3,
                    "yaw {yaw}, {forward}° forward, feet {clearance} m up: pelvis dropped {hip_drop} m, ankles {ankle_drop} m"
                );
            }
        }
        // What the thigh gate is for: with it ablated the vertical shanks count
        // as standing legs, and feet 3 cm up are "explained" by the 2 cm base
        // budget. The pelvis is pulled 1.9 cm down (2 cm·(1 − smoothstep(2 cm,
        // 9 cm, 3 cm)) = 1.89 cm) and the planted anchors sink toward the floor.
        let (hip_drop, ankle_drop, _) = reclined_chair(yaw, recline, 0.03, ablate_thigh_gate);
        eprintln!(
            "reclined chair, yaw {yaw}, thigh gate ablated: pelvis drops {hip_drop:.4} m, ankles {ankle_drop:.4} m"
        );
        assert!(hip_drop > 0.015, "yaw {yaw}: {hip_drop}");
        assert!(ankle_drop > 0.02, "yaw {yaw}: {ankle_drop}");
    }
}

/// Lying on the left side on a platform, legs straight and stacked; after
/// settling the headset drifts `drift` meters toward the feet at 5 cm/s.
/// Returns the largest knee offset from its hip-ankle line.
fn side_lying_drift(yaw: f32, slack: f32, drift: f32) -> f32 {
    let mut core = calibrated_with(yaw, |ik| ik.knee_flex_slack_rad = slack);
    let body = core.config().anthropometrics.clone();
    let heading = Quat::from_rotation_y(yaw);
    // Every segment rolled 90°: the body's right side faces up, its long axis
    // lies level with the feet toward heading·(+X).
    let side = heading * Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let rotations = [side; 6];
    let platform = 0.5;
    let lower_ankle = Vec3::Y * (platform + body.ankle_to_sole_m);
    let (head, truth) = placed_chain(&core, side, rotations, |c| c.ankles[0], lower_ankle);
    assert!((truth.ankles[1].y - truth.ankles[0].y - body.hip_width_m).abs() < 1.0e-5);
    let toward_feet = (truth.ankles[0] - truth.hip_joints[0]).normalize();
    assert!(toward_feet.y.abs() < 1.0e-5);
    let mut worst = 0.0_f32;
    let mut locked = false;
    for tick in 0..260_usize {
        let moved = (tick.saturating_sub(100) as f32 * 0.0005).min(drift);
        let frame = feed(&mut core, tick, head + toward_feet * moved, side, rotations);
        if tick < 100 {
            continue;
        }
        let context = format!("yaw {yaw} slack {slack} drift {drift} tick {tick}");
        assert!(frame.targets.iter().all(|target| target.valid), "{context}");
        // The lower foot is still and locked; the upper one is 30 cm above it,
        // which the contact layer does not take for a stance foot.
        locked |= frame.estimator.leg_ik.foot_locked[0];
        let floor = frame.estimator.support_floor;
        // A sideways torso keeps its full bow budget (60 cm, more than the
        // 50 cm gap): here only the level legs keep the pelvis up.
        assert!(floor.budget_m > platform, "{context}: {floor:?}");
        assert!(floor.support_weight < 1.0e-3, "{context}: {floor:?}");
        assert!(floor.vertical_m.abs() < 1.0e-5, "{context}: {floor:?}");
        assert_leg_bone_lengths(&core, &frame, &context);
        assert!(
            (joint(&frame, JointRole::Hip).y - truth.hip.y).abs() < 1.0e-3,
            "{context}"
        );
        for (s, (hip, knee, ankle)) in LEGS.into_iter().enumerate() {
            let (hip, knee, ankle) = (
                joint(&frame, hip),
                joint(&frame, knee),
                joint(&frame, ankle),
            );
            // Each leg stays at its own height: stacked, not dropped.
            assert!((hip.y - truth.hip_joints[s].y).abs() < 1.0e-3, "{context}");
            assert!((ankle.y - truth.ankles[s].y).abs() < 1.0e-3, "{context}");
            worst = worst.max(knee_offset(hip, knee, ankle));
        }
    }
    assert!(locked, "yaw {yaw}: the lower foot must be locked");
    worst
}

#[test]
fn side_lying_on_a_platform_keeps_the_stacked_legs_and_the_pelvis_in_place() {
    let body = Anthropometrics::default();
    let slack = LegIkConfig::default().knee_flex_slack_rad;
    let bound = guard_knee_offset(&body, slack);
    for yaw in [0.0, 0.8, -1.3] {
        let still = side_lying_drift(yaw, slack, 0.0);
        assert!(still < GUARD_TOLERANCE_M, "yaw {yaw}: {still}");
        let pushed = side_lying_drift(yaw, slack, 0.02);
        assert!(
            pushed < bound + GUARD_TOLERANCE_M,
            "yaw {yaw}: 2 cm toward the feet bent the lower knee {pushed} m"
        );
        // Without the flexion guard the locked lower leg, left 8 mm short after
        // root correction, bends by sqrt(0.42 m·5.75 mm) = 4.9 cm.
        let unguarded = side_lying_drift(yaw, std::f32::consts::PI, 0.02);
        eprintln!(
            "side lying, yaw {yaw}: knee offset {pushed:.5} m, {unguarded:.5} m without the guard"
        );
        assert!(unguarded > 0.04, "yaw {yaw}: {unguarded}");
    }
}

struct SitCycle {
    /// Largest drop of the published pelvis below the head-down FK pelvis of
    /// the same frame, over the whole cycle and while seated and settled.
    worst_drop: f32,
    seated_drop: f32,
    /// Largest distance of a published ankle from the true (still) ankle.
    worst_ankle_error: f32,
    /// At the end, standing again: pelvis drop below FK, largest ankle height
    /// error against the floor, and whether both feet are locked.
    final_drop: f32,
    final_ankle_height_error: f32,
    final_locked: bool,
}

/// Stand 1.5 s, sit down over 1.5 s (thighs to level, feet still on the floor,
/// torso leaning up to 25° forward on the way), sit 2 s, stand up over 1.5 s,
/// stand 1.5 s. `chain_error` raises the reported headset: the head-down chain
/// is that much too short, so its feet float by it.
fn stand_sit_stand(yaw: f32, chain_error: f32, tune: impl FnOnce(&mut LegIkConfig)) -> SitCycle {
    let mut core = calibrated_with(yaw, tune);
    let body = core.config().anthropometrics.clone();
    let heading = Quat::from_rotation_y(yaw);
    let left_ankle = Vec3::Y * body.ankle_to_sole_m + heading * Vec3::X * (-0.5 * body.hip_width_m);
    let smooth = |x: f32| {
        let x = x.clamp(0.0, 1.0);
        x * x * (3.0 - 2.0 * x)
    };
    let mut report = SitCycle {
        worst_drop: f32::NEG_INFINITY,
        seated_drop: f32::NEG_INFINITY,
        worst_ankle_error: 0.0,
        final_drop: 0.0,
        final_ankle_height_error: 0.0,
        final_locked: false,
    };
    for tick in 0..800_usize {
        let seated = if tick < 500 {
            smooth((tick as f32 - 150.0) / 150.0)
        } else {
            1.0 - smooth((tick as f32 - 500.0) / 150.0)
        };
        let lean = 25.0_f32.to_radians() * (std::f32::consts::PI * seated).sin();
        let torso = heading * Quat::from_rotation_x(-lean);
        let thigh = heading * Quat::from_rotation_x(seated * std::f32::consts::FRAC_PI_2);
        let rotations = [torso, torso, thigh, thigh, heading, heading];
        let (head, truth) = placed_chain(&core, heading, rotations, |c| c.ankles[0], left_ankle);
        if tick == 400 {
            // Seated: pelvis at knee height, behind the feet.
            assert!((truth.hip.y - body.ankle_to_sole_m - body.calf_length_m).abs() < 1.0e-4);
        }
        let frame = feed(
            &mut core,
            tick,
            head + Vec3::Y * chain_error,
            heading,
            rotations,
        );
        if tick < 60 {
            continue;
        }
        assert!(
            frame.targets.iter().all(|target| target.valid),
            "tick {tick}"
        );
        assert_leg_bone_lengths(&core, &frame, "stand-sit-stand");
        let fk_hip_height = truth.hip.y + chain_error;
        let drop = fk_hip_height - joint(&frame, JointRole::Hip).y;
        report.worst_drop = report.worst_drop.max(drop);
        if (420..500).contains(&tick) {
            report.seated_drop = report.seated_drop.max(drop.abs());
        }
        for (s, (_, _, ankle)) in LEGS.into_iter().enumerate() {
            let ankle = joint(&frame, ankle);
            report.worst_ankle_error = report
                .worst_ankle_error
                .max((ankle - truth.ankles[s]).length());
            if tick >= 760 {
                report.final_ankle_height_error = report
                    .final_ankle_height_error
                    .max((ankle.y - body.ankle_to_sole_m).abs());
            }
        }
        if tick >= 760 {
            report.final_drop = drop;
            report.final_locked = frame.estimator.leg_ik.foot_locked == [true; 2];
        }
    }
    report
}

#[test]
fn stand_sit_stand_never_drops_the_pelvis_and_recovers_the_standing_floor() {
    let ablate_thigh_gate = |ik: &mut LegIkConfig| {
        ik.support_thigh_full_tilt_rad = std::f32::consts::PI;
        ik.support_thigh_zero_tilt_rad = std::f32::consts::PI;
    };
    for yaw in [0.0, 0.8] {
        // An exact chain: head-down FK is the truth, so nothing may move.
        let exact = stand_sit_stand(yaw, 0.0, |_| {});
        eprintln!(
            "stand-sit-stand, yaw {yaw}, exact chain: worst pelvis drop {:.5} m, worst ankle error {:.5} m",
            exact.worst_drop, exact.worst_ankle_error
        );
        assert!(exact.worst_drop < 1.0e-3, "yaw {yaw}: {}", exact.worst_drop);
        assert!(
            exact.seated_drop < 1.0e-3,
            "yaw {yaw}: {}",
            exact.seated_drop
        );
        assert!(
            exact.worst_ankle_error < 2.0e-3,
            "yaw {yaw}: {}",
            exact.worst_ankle_error
        );
        assert!(exact.final_locked && exact.final_ankle_height_error < 1.0e-3);

        // A chain 1.5 cm too short: standing, the 2 cm base budget puts the
        // feet on the floor by lowering the pelvis 1.5 cm. That is the most the
        // pelvis may ever be below head-down FK (the pull fades with the thigh
        // weight while the planted anchors, with root_gain 0.6, take over less
        // than what it gives up). Seated, nothing supports "the feet are on
        // the floor", so the pelvis is back at head-down height.
        let error = 0.015;
        let short = stand_sit_stand(yaw, error, |_| {});
        eprintln!(
            "stand-sit-stand, yaw {yaw}, chain 1.5 cm short: worst pelvis drop {:.5} m, seated {:.5} m, final {:.5} m",
            short.worst_drop, short.seated_drop, short.final_drop
        );
        assert!(
            short.worst_drop < error + 1.0e-3,
            "yaw {yaw}: pelvis dropped {} m below head-down FK",
            short.worst_drop
        );
        assert!(
            short.seated_drop < 2.0e-3,
            "yaw {yaw}: seated pelvis {} m from head-down FK",
            short.seated_drop
        );
        // Standing again: feet on the floor, pelvis lowered by the chain error.
        assert!(short.final_locked, "yaw {yaw}");
        assert!(
            short.final_ankle_height_error < 2.0e-3,
            "yaw {yaw}: {}",
            short.final_ankle_height_error
        );
        assert!(
            (short.final_drop - error).abs() < 2.0e-3,
            "yaw {yaw}: {}",
            short.final_drop
        );

        // With the thigh gate ablated the vertical shanks keep the standing
        // prior while seated and the pelvis stays pulled down by the error.
        let calf_only = stand_sit_stand(yaw, error, ablate_thigh_gate);
        assert!(
            calf_only.seated_drop > error - 2.0e-3,
            "yaw {yaw}: {}",
            calf_only.seated_drop
        );
    }
}

/// Stand 1.5 s, sink into a parallel squat over 1.5 s (thighs level, shanks 30°
/// forward, torso `lean` forward, feet still on the floor) and hold it 2 s,
/// with a head-down chain `chain_error` too short. Returns, over the last
/// second of the hold, the largest height of a published ankle above its floor
/// height, the largest horizontal distance of one from the true ankle, and the
/// smallest support-foot weight.
fn parallel_squat(
    yaw: f32,
    lean: f32,
    chain_error: f32,
    tune: impl FnOnce(&mut LegIkConfig),
) -> (f32, f32, f32) {
    let mut core = calibrated_with(yaw, tune);
    let body = core.config().anthropometrics.clone();
    let heading = Quat::from_rotation_y(yaw);
    let left_ankle = Vec3::Y * body.ankle_to_sole_m + heading * Vec3::X * (-0.5 * body.hip_width_m);
    let (mut float, mut slide, mut weight) = (0.0_f32, 0.0_f32, 1.0_f32);
    for tick in 0..500_usize {
        let depth = ((tick as f32 - 150.0) / 150.0).clamp(0.0, 1.0);
        let depth = depth * depth * (3.0 - 2.0 * depth);
        let torso = heading * Quat::from_rotation_x(-lean * depth);
        let thigh = heading * Quat::from_rotation_x(depth * std::f32::consts::FRAC_PI_2);
        let calf = heading * Quat::from_rotation_x(-30.0_f32.to_radians() * depth);
        let rotations = [torso, torso, thigh, thigh, calf, calf];
        let (head, truth) = placed_chain(&core, heading, rotations, |c| c.ankles[0], left_ankle);
        let frame = feed(
            &mut core,
            tick,
            head + Vec3::Y * chain_error,
            heading,
            rotations,
        );
        if tick < 60 {
            continue;
        }
        assert!(
            frame.targets.iter().all(|target| target.valid),
            "tick {tick}"
        );
        assert_leg_bone_lengths(&core, &frame, "parallel squat");
        if tick < 400 {
            continue;
        }
        weight = weight.min(frame.estimator.support_floor.support_weight);
        for (s, (_, _, ankle)) in LEGS.into_iter().enumerate() {
            let ankle = joint(&frame, ankle);
            float = float.max(ankle.y - body.ankle_to_sole_m);
            let offset = ankle - truth.ankles[s];
            slide = slide.max(Vec3::new(offset.x, 0.0, offset.z).length());
        }
    }
    (float, slide, weight)
}

#[test]
fn a_parallel_squat_with_a_forward_bowed_torso_keeps_its_feet_on_the_floor() {
    let ablate_squat_bow = |ik: &mut LegIkConfig| {
        ik.squat_bow_start_tilt_rad = 2.0;
        ik.squat_bow_full_tilt_rad = 2.0;
    };
    // The chain is 3 cm short, so head-down FK leaves the feet 3 cm in the air.
    // In the squat the thighs are level (no thigh weight) and the shanks 30°
    // from vertical (full shank weight).
    let error = 0.03;
    for yaw in [0.0, 0.8] {
        // Torso 40° forward: beyond squat_bow_full_tilt_rad (35°), so the legs
        // keep the standing prior; the bow's budget of 2 cm + 0.58 m·(1 − cos
        // 40°) = 15.6 cm covers the 3 cm and the pelvis is lowered onto the feet.
        let lean = 40.0_f32.to_radians();
        let (float, slide, weight) = parallel_squat(yaw, lean, error, |_| {});
        eprintln!(
            "parallel squat, yaw {yaw}: ankles {float:.4} m above the floor, {slide:.4} m aside, weight {weight}"
        );
        assert!(weight > 0.999, "yaw {yaw}: {weight}");
        assert!(float.abs() < 0.003, "yaw {yaw}: ankles float {float} m");
        assert!(slide < 0.005, "yaw {yaw}: ankles slid {slide} m");
        // With the rule ablated the level thighs remove the prior and the feet
        // float by the whole chain error, as on a chair.
        let (float, _, weight) = parallel_squat(yaw, lean, error, ablate_squat_bow);
        eprintln!("parallel squat, yaw {yaw}, squat bow ablated: ankles {float:.4} m up");
        assert!(weight < 0.001, "yaw {yaw}: {weight}");
        assert!(float > error - 0.003, "yaw {yaw}: {float}");
        // The same legs under a torso leaning 15° (below the 20° start): a
        // seat as far as the rule can tell, and the feet stay where FK has them.
        let (float, _, weight) = parallel_squat(yaw, 15.0_f32.to_radians(), error, |_| {});
        assert!(weight < 0.001, "yaw {yaw}: {weight}");
        assert!(float > error - 0.003, "yaw {yaw}: {float}");
    }
}

/// Crawling on hands and knees with exactly known truth (torso level and face
/// down, thighs vertical, shanks flat on the floor behind the knees); after
/// settling the headset drifts `drift` meters backward, toward the point above
/// the locked ankles, at 5 cm/s. Returns the lowest published ankle height.
fn crawling_drift(yaw: f32, drift: f32, tune: impl FnOnce(&mut LegIkConfig)) -> f32 {
    let mut core = calibrated_with(yaw, tune);
    let body = core.config().anthropometrics.clone();
    let heading = Quat::from_rotation_y(yaw);
    let torso = heading * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let shank = heading * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let rotations = [torso, torso, heading, heading, shank, shank];
    let left_ankle = Vec3::Y * body.ankle_to_sole_m + heading * Vec3::X * (-0.5 * body.hip_width_m);
    let (head, truth) = placed_chain(&core, torso, rotations, |c| c.ankles[0], left_ankle);
    assert!((truth.hip_joints[0].y - truth.knees[0].y - body.thigh_length_m).abs() < 1.0e-5);
    assert!((truth.ankles[1].y - body.ankle_to_sole_m).abs() < 1.0e-5);
    let backward = heading * Vec3::Z;
    assert!((truth.ankles[0] - truth.knees[0]).normalize().dot(backward) > 0.9999);
    let mut lowest = f32::INFINITY;
    let mut locked = false;
    for tick in 0..500_usize {
        let moved = (tick.saturating_sub(100) as f32 * 0.0005).min(drift);
        let frame = feed(&mut core, tick, head + backward * moved, torso, rotations);
        if tick < 100 {
            continue;
        }
        assert!(
            frame.targets.iter().all(|target| target.valid),
            "tick {tick}"
        );
        assert_leg_bone_lengths(&core, &frame, "crawling");
        locked |= frame.estimator.leg_ik.foot_locked == [true; 2];
        for (_, _, ankle) in LEGS {
            lowest = lowest.min(joint(&frame, ankle).y);
        }
    }
    assert!(locked, "yaw {yaw}: the still feet must be locked");
    lowest
}

#[test]
fn crawling_with_a_drifting_headset_keeps_the_guarded_ankles_above_the_floor() {
    let body = Anthropometrics::default();
    let defaults = LegIkConfig::default();
    let floor = body.ankle_to_sole_m - defaults.floor_margin_m;
    for yaw in [0.0, 0.8] {
        for drift in [0.05_f32, 0.15] {
            let lowest = crawling_drift(yaw, drift, |_| {});
            let unclamped = crawling_drift(yaw, drift, |ik| ik.floor_clamp_enabled = false);
            eprintln!(
                "crawling, yaw {yaw}, headset {drift} m back: lowest ankle {lowest:.4} m, {unclamped:.4} m without the floor clamp"
            );
            assert!(
                lowest > floor - 1.0e-4,
                "yaw {yaw}, {drift} m: ankle at {lowest} m, floor limit {floor}"
            );
            if drift >= 0.15 {
                // The pelvis takes 60 % of the drift; the other 6 cm shorten the
                // hip-anchor distance by 4 cm, of which the knee may take 5 mm.
                assert!(
                    unclamped < floor - 0.01,
                    "yaw {yaw}: without the clamp the ankle only reached {unclamped} m"
                );
            }
        }
    }
}
