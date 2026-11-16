//! End-to-end synthetic regressions for the stance-anchored estimator layers
//! (contact hypotheses, stance heading filter, contact-consistent leg IK) as
//! integrated in `TrackingCore`, driven only through the public API:
//! assignments, live neutral + skiing-pose forward calibration, sensor samples
//! and HMD anchors.
//!
//! Ground truth comes from `synthetic::BodyGenerator`, which builds bodies with
//! the estimator's own kinematic model. Passing shows that the integration
//! matches that model and keeps its invariants under the injected faults. It
//! is not evidence of accuracy on a worn body, of latency, or of any
//! comparison with other tracking software.

use glam::{Quat, Vec3};
use tracking_core::chain::{ChainDimensions, ChainPose, lateral_heading, solve_chain};
use tracking_core::contact::ContactPhase;
use tracking_core::forward_calibration::ski_pitch_sign;
use tracking_core::leg_ik::LegIkStatus;
use tracking_core::stance_heading::StanceHeadingStatus;
use tracking_core::synthetic::{Motion, Scenario, TruthFrame, XorShift64, scenarios};
use tracking_core::*;

const HZ: f64 = 100.0;
/// Scenario time zero on the core clock. Forward calibration completes at
/// 2.0 s; the gap keeps the calibration headset pose (crouched skiing pose)
/// out of the HMD discontinuity detector.
const START: f64 = 2.2;
const LEGS: [usize; 4] = [2, 3, 4, 5];
const CALIBRATION_HEADING: f32 = 0.4;

// ---------------------------------------------------------------------------
// Sensors, calibration and input streams.

fn index(role: Role) -> usize {
    Role::ALL.iter().position(|r| *r == role).unwrap()
}
fn id(role: Role) -> String {
    format!("integration-{role:?}")
}
/// Arbitrary physical sensor mounting (right factor), different per role.
fn mount(role: Role) -> Quat {
    let i = index(role) as f32;
    Quat::from_rotation_x(0.3 + 0.23 * i)
        * Quat::from_rotation_z(-0.7 + 0.31 * i)
        * Quat::from_rotation_y(0.4 + 0.41 * i)
}
/// Arbitrary sensor-world heading origin (left factor), different per role.
fn bias(role: Role) -> f32 {
    [-1.4, 2.2, 1.5707964, -2.7, 0.9, -0.4, 0.3, 0.7, -1.1, 2.8][index(role)]
}
fn ski_pitch(role: Role) -> f32 {
    ski_pitch_sign(role).unwrap_or(0.0)
        * [28.0_f32, 22.0, 38.0, 34.0, 18.0, 20.0, 0.0, 0.0, 0.0, 0.0][index(role)].to_radians()
}
/// Raw sensor quaternion for a standing-space bone rotation.
fn raw(role: Role, body: Quat) -> Quat {
    Quat::from_rotation_y(bias(role)) * body * mount(role)
}
fn sample(role: Role, time: f64, body: Quat) -> SensorSample {
    SensorSample {
        id: id(role),
        orientation_xyzw: raw(role, body).to_array(),
        received_at_secs: time,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    }
}
fn hmd(time: f64, position: Vec3, rotation: Quat) -> AnchorPose {
    AnchorPose {
        position: position.to_array(),
        orientation_xyzw: rotation.to_array(),
        received_at_secs: time,
        valid: true,
        linear_velocity: None,
        angular_velocity: None,
    }
}
fn anchors(time: f64, position: Vec3, rotation: Quat) -> Anchors {
    Anchors {
        hmd: Some(hmd(time, position, rotation)),
        ..Default::default()
    }
}

/// Assignments, live neutral calibration and the skiing-pose forward
/// calibration (arbitrary mounts and sensor heading origins).
fn calibrated(config: TrackingConfig) -> TrackingCore {
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(
        Role::CORE
            .iter()
            .map(|role| RoleAssignment {
                sensor_id: id(*role),
                role: *role,
            })
            .collect(),
    )
    .unwrap();
    let heading = Quat::from_rotation_y(CALIBRATION_HEADING);
    for tick in 0..=200 {
        let time = tick as f64 / 100.0;
        let ski = tick > 100;
        for role in Role::CORE {
            let bend = if ski { ski_pitch(role) } else { 0.0 };
            let body = heading * Quat::from_rotation_x(bend);
            core.ingest(sample(role, time, body)).unwrap();
        }
        let height = if ski { 1.1 } else { 1.65 };
        core.solve(time, &anchors(time, Vec3::new(0.0, height, 0.0), heading));
        if tick == 100 {
            core.calibrate_neutral(
                time,
                &anchors(time, Vec3::new(0.0, 1.65, 0.0), heading),
                false,
            )
            .unwrap();
        }
    }
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}

fn config(stance: bool, leg_ik: bool) -> TrackingConfig {
    let mut config = TrackingConfig::default();
    config.estimator.enable_stance_heading = stance;
    config.estimator.enable_leg_ik = leg_ik;
    config
}

/// One synthetic instant: truth, the injected measurement and the headset.
#[derive(Clone, Copy)]
struct Input {
    /// Core clock.
    at: f64,
    /// Scenario clock.
    t: f64,
    truth: TruthFrame,
    /// Measured bone rotations `Ry(−e)·true`, `Role::CORE` order.
    measured: [Quat; 6],
    head_position: Vec3,
    head_rotation: Quat,
}

/// Errors are degrees per `Role::CORE` segment; 1 mm white headset noise.
fn inputs(script: Vec<Motion>, errors: impl Fn(f64) -> [f32; 6]) -> Vec<Input> {
    inputs_with_noise(script, errors, 0.001)
}

fn inputs_with_noise(
    script: Vec<Motion>,
    errors: impl Fn(f64) -> [f32; 6],
    noise_m: f32,
) -> Vec<Input> {
    let generator = Scenario::new("integration", script).generator();
    let mut rng = XorShift64::new(7);
    let count = (generator.duration() * HZ).floor() as usize + 1;
    (0..count)
        .map(|k| {
            let t = k as f64 / HZ;
            let truth = generator.frame(t);
            let e = errors(t);
            let noise = Vec3::new(
                rng.gaussian() as f32,
                rng.gaussian() as f32,
                rng.gaussian() as f32,
            ) * noise_m;
            Input {
                at: START + t,
                t,
                truth,
                measured: std::array::from_fn(|i| {
                    Quat::from_rotation_y(-e[i].to_radians()) * truth.rotations[i]
                }),
                head_position: truth.head_position + noise,
                head_rotation: truth.head_rotation,
            }
        })
        .collect()
}

fn leg_error(degrees: f32) -> impl Fn(f64) -> [f32; 6] {
    move |_| [0.0, 0.0, degrees, degrees, degrees, degrees]
}

fn feed_sensors(core: &mut TrackingCore, input: &Input, skip: Option<Role>) {
    for (i, role) in Role::CORE.into_iter().enumerate() {
        if Some(role) != skip {
            core.ingest(sample(role, input.at, input.measured[i]))
                .unwrap();
        }
    }
}

fn step(core: &mut TrackingCore, input: &Input) -> TrackingFrame {
    feed_sensors(core, input, None);
    core.solve(
        input.at,
        &anchors(input.at, input.head_position, input.head_rotation),
    )
}

// ---------------------------------------------------------------------------
// Frame accessors and metrics.

fn segment(frame: &TrackingFrame, role: Role) -> &SegmentPose {
    frame.segments.iter().find(|s| s.role == role).unwrap()
}
fn published(frame: &TrackingFrame, role: Role) -> Quat {
    Quat::from_array(segment(frame, role).orientation_xyzw)
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
fn target(frame: &TrackingFrame, role: TargetRole) -> &TargetPose {
    frame.targets.iter().find(|t| t.role == role).unwrap()
}
fn wrap_degrees(value: f32) -> f32 {
    (value + 180.0).rem_euclid(360.0) - 180.0
}
fn wrap(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}
fn yaw_of(q: Quat) -> f32 {
    let forward = q * Vec3::NEG_Z;
    (-forward.x).atan2(-forward.z)
}
fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos()
}
/// Published minus true lateral heading of each leg segment, degrees. The
/// truth is first rotated by `space_yaw` (a rebased standing space).
fn leg_heading_errors(frame: &TrackingFrame, truth: &TruthFrame, space_yaw: f32) -> [f32; 4] {
    LEGS.map(|i| {
        let estimated = lateral_heading(published(frame, Role::CORE[i])).unwrap();
        let truth = lateral_heading(Quat::from_rotation_y(space_yaw) * truth.rotations[i]).unwrap();
        wrap_degrees((estimated - truth).to_degrees())
    })
}
fn max_abs(values: impl IntoIterator<Item = f32>) -> f32 {
    values.into_iter().fold(0.0, |m, v| m.max(v.abs()))
}
fn corrections_degrees(frame: &TrackingFrame) -> [f32; 6] {
    frame.estimator.stance.corrections.map(f32::to_degrees)
}
/// First scenario time after which `worst` stays below `limit` to the end.
fn settled_at(series: &[(f64, f32)], limit: f32) -> Option<f64> {
    let mut since = None;
    for (t, worst) in series {
        if *worst < limit {
            since.get_or_insert(*t);
        } else {
            since = None;
        }
    }
    since
}

fn assert_finite(frame: &TrackingFrame) {
    let finite = |v: &[f32]| v.iter().all(|x| x.is_finite());
    for t in &frame.targets {
        assert!(
            finite(&t.position)
                && finite(&t.orientation_xyzw)
                && finite(&t.linear_velocity)
                && finite(&t.angular_velocity)
                && t.confidence.is_finite(),
            "{t:?}"
        );
    }
    for j in &frame.joints {
        assert!(finite(&j.position) && j.confidence.is_finite(), "{j:?}");
    }
    for s in &frame.segments {
        assert!(
            finite(&s.orientation_xyzw)
                && s.yaw_correction_degrees.is_finite()
                && s.confidence.is_finite()
                && s.measurement_age_secs.is_none_or(f64::is_finite),
            "{s:?}"
        );
    }
    for c in &frame.corrections {
        assert!(c.residual_degrees.is_finite() && c.applied_relative_degrees.is_finite());
    }
    let e = &frame.estimator;
    let stance = &e.stance;
    let mut numbers = vec![e.hmd_extrapolated_secs, e.last_rebase_degrees];
    numbers.extend(stance.corrections);
    numbers.extend(stance.sigma);
    for foot in &stance.feet {
        numbers.extend([
            foot.last_excitation_m,
            foot.last_innovation_m,
            foot.last_nis,
            foot.nis_average,
        ]);
    }
    for prior in stance
        .priors
        .knee
        .iter()
        .chain([&stance.priors.pelvis, &stance.priors.chest])
    {
        numbers.push(prior.last_residual_degrees);
    }
    for contact in &e.contacts {
        assert!(contact.since_secs.is_finite());
        numbers.extend([
            contact.speed_mps,
            contact.height_above_lower_m,
            contact.yaw_rate_dps,
            contact.planted_yaw_degrees,
        ]);
    }
    numbers.extend(e.contact_speed_allowance_mps);
    numbers.extend(e.leg_ik.lock_error_m);
    numbers.extend(e.leg_ik.root_correction);
    numbers.extend(e.leg_ik.flat_weight);
    assert!(finite(&numbers), "{e:?}");
    assert!(
        finite(&frame.preview_motion.root_translation_m),
        "{:?}",
        frame.preview_motion
    );
}

// ---------------------------------------------------------------------------
// (a) Flags off: the published skeleton is exactly the shared chain.

#[test]
fn flags_off_publish_exactly_the_shared_chain_and_the_previous_inline_kinematics() {
    let mut core = calibrated(TrackingConfig::default());
    let body = Anthropometrics::default();
    let dims = ChainDimensions::from_anthropometrics(&body, body.eye_forward_m);
    for input in inputs(scenarios::walk_laps(1), leg_error(10.0)).iter() {
        let frame = step(&mut core, input);
        assert!(frame.targets.iter().all(|t| t.valid));
        let e = &frame.estimator;
        assert!(!e.stance_active && !e.leg_ik_active);
        assert_eq!(e.stance.corrections, [0.0; 6]);
        assert_eq!(e.stance.status, StanceHeadingStatus::Idle);
        assert_eq!(e.leg_ik.status, LegIkStatus::Disabled);
        assert!(e.contacts.iter().all(|c| c.phase == ContactPhase::Swing));
        // Forward calibration recovered every arbitrary mount and heading
        // origin: published segments are the injected measurements.
        for (i, role) in Role::CORE.into_iter().enumerate() {
            let error = angle(published(&frame, role), input.measured[i]).to_degrees();
            assert!(error < 0.1, "{role:?} calibration error {error}°");
            assert_eq!(segment(&frame, role).yaw_correction_degrees, 0.0);
        }
        // Single source of truth: bit-for-bit the shared chain.
        let rotations = Role::CORE.map(|role| published(&frame, role));
        // The core normalises the anchor quaternion it receives.
        let head_rotation = Quat::from_array(input.head_rotation.to_array()).normalize();
        let chain = solve_chain(
            &dims,
            &ChainPose {
                head_position: input.head_position,
                head_rotation,
                rotations,
            },
        );
        for (role, expected) in [
            (JointRole::Neck, chain.neck),
            (JointRole::Chest, chain.chest),
            (JointRole::Hip, chain.hip),
            (JointRole::LeftHip, chain.hip_joints[0]),
            (JointRole::RightHip, chain.hip_joints[1]),
            (JointRole::LeftKnee, chain.knees[0]),
            (JointRole::RightKnee, chain.knees[1]),
            (JointRole::LeftAnkle, chain.ankles[0]),
            (JointRole::RightAnkle, chain.ankles[1]),
        ] {
            assert_eq!(joint(&frame, role), expected, "{role:?}");
        }
        assert_eq!(
            Vec3::from_array(target(&frame, TargetRole::Chest).position),
            chain.chest
        );
        assert_eq!(
            Vec3::from_array(target(&frame, TargetRole::Waist).position),
            chain.hip
        );
        assert_eq!(
            Vec3::from_array(target(&frame, TargetRole::LeftKnee).position),
            chain.knees[0]
        );
        // The previous inline forward kinematics, to within float noise.
        let neck = input.head_position
            + head_rotation * Vec3::new(0.0, -body.head_to_neck_m, body.eye_forward_m);
        let chest = neck - rotations[0] * Vec3::Y * body.neck_to_chest_m;
        let spine = rotations[0].slerp(rotations[1], 0.5).normalize();
        let hip = chest - spine * Vec3::Y * body.chest_to_hip_m;
        for (foot, sign) in [(0, -1.0), (1, 1.0)] {
            let root = hip + rotations[1] * Vec3::X * (sign * body.hip_width_m * 0.5);
            let knee = root - rotations[2 + foot] * Vec3::Y * body.thigh_length_m;
            let ankle = knee - rotations[4 + foot] * Vec3::Y * body.calf_length_m;
            assert!((chain.ankles[foot] - ankle).length() < 1.0e-5);
            assert!((chain.knees[foot] - knee).length() < 1.0e-5);
            let foot_target = ankle
                + rotations[4 + foot]
                    * Vec3::new(0.0, -body.ankle_to_sole_m * 0.5, -body.foot_length_m * 0.25);
            let published = target(&frame, [TargetRole::LeftFoot, TargetRole::RightFoot][foot]);
            assert!((Vec3::from_array(published.position) - foot_target).length() < 1.0e-5);
        }
        assert!((chain.neck - neck).length() < 1.0e-5);
        assert!((chain.hip - hip).length() < 1.0e-5);
    }
}

// ---------------------------------------------------------------------------
// (b) Common-mode leg drift during walking.

fn walking_errors(stance: bool) -> Vec<(f64, [f32; 4], TrackingFrame)> {
    let mut core = calibrated(config(stance, false));
    inputs(scenarios::walk_laps(4), leg_error(10.0))
        .iter()
        .map(|input| {
            let frame = step(&mut core, input);
            (
                input.t,
                leg_heading_errors(&frame, &input.truth, 0.0),
                frame,
            )
        })
        .collect()
}

#[test]
fn stance_filter_removes_common_mode_leg_drift_during_walking() {
    let on = walking_errors(true);
    let off = walking_errors(false);
    // Off: every published leg heading carries the injected −10° throughout.
    for (t, errors, _) in &off {
        for error in errors {
            assert!((error + 10.0).abs() < 0.05, "off at {t}: {errors:?}");
        }
    }
    let series: Vec<(f64, f32)> = on.iter().map(|(t, e, _)| (*t, max_abs(*e))).collect();
    let converged = settled_at(&series, 3.0);
    let (_, final_errors, last) = on.last().unwrap();
    println!(
        "core walking, 10° common-mode leg drift: < 3° from {converged:?} s, final {final_errors:?}, \
         corrections {:?}°",
        corrections_degrees(last)
    );
    assert!(converged.is_some_and(|t| t <= 20.0), "{converged:?}");
    assert!(max_abs(*final_errors) < 2.0);
    assert!(
        on.iter()
            .filter(|(_, _, f)| f.estimator.stance_active)
            .count()
            > on.len() * 9 / 10
    );
    let accepted: u32 = last.estimator.stance.feet.iter().map(|f| f.accepted).sum();
    assert!(accepted > 20, "{accepted}");
    // The contact speed gates widen by the filter's yaw uncertainty times the
    // headset speed; never with the filter off.
    let widest = |run: &[(f64, [f32; 4], TrackingFrame)]| {
        run.iter()
            .flat_map(|(_, _, f)| f.estimator.contact_speed_allowance_mps)
            .fold(0.0_f32, f32::max)
    };
    assert!(widest(&on) > 0.02, "{}", widest(&on));
    assert_eq!(widest(&off), 0.0);
    // The torso carries no error and stays essentially uncorrected.
    let c = corrections_degrees(last);
    assert!(max_abs([c[0], c[1]]) < 2.5, "{c:?}");
    // The reported yaw correction is exactly the stance correction applied to
    // the published rotation (no recovery, no legacy knee correction here).
    for window in on.windows(2) {
        let (previous, current) = (&window[0].2, &window[1].2);
        for (i, role) in Role::CORE.into_iter().enumerate() {
            let applied = previous.estimator.stance.corrections[i].to_degrees();
            let reported = segment(current, role).yaw_correction_degrees;
            assert!((reported - applied).abs() < 1.0e-4, "{role:?}");
        }
    }
}

// ---------------------------------------------------------------------------
// (c) Head-only turns with a wrong eye offset.

#[test]
fn head_only_turns_with_eye_offset_error_produce_no_correction() {
    let mut config = config(true, true);
    // Truth uses 8 cm; the core assumes 12 cm.
    config.anthropometrics.eye_forward_m = 0.12;
    let mut core = calibrated(config);
    let mut worst = 0.0_f32;
    let mut observed = 0;
    for input in inputs(vec![Motion::stand(1.0), Motion::head_turns(40.0)], |_| {
        [0.0; 6]
    })
    .iter()
    {
        let frame = step(&mut core, input);
        assert_finite(&frame);
        worst = worst.max(max_abs(corrections_degrees(&frame)));
        if frame.estimator.stance_active && frame.estimator.contacts.iter().all(|c| c.planted()) {
            observed += 1;
        }
    }
    println!("core head-only turns, 4 cm eye-offset error: max correction {worst:.3}°");
    assert!(
        observed > 2000,
        "feet were not observed planted: {observed}"
    );
    assert!(worst < 1.0, "{worst}");
}

// ---------------------------------------------------------------------------
// (d) Leg IK: foot lock under an upstream error, exact bones, validity.

const WOBBLE_START: f64 = 2.0;
const WOBBLE_M: f32 = 0.015;
const STALE_CALF: (f64, f64) = (8.0, 8.6);
const HMD_DROPOUT: (f64, f64) = (10.0, 10.3);

/// The headset position carries a horizontal ±1.5 cm model error (as from a
/// wrong eye/neck offset) from 2 s on; the left calf goes silent and the HMD
/// drops out later.
fn sway_step(core: &mut TrackingCore, input: &Input) -> TrackingFrame {
    let stale = (STALE_CALF.0..STALE_CALF.1).contains(&input.t);
    feed_sensors(core, input, stale.then_some(Role::LeftCalf));
    let wobble = if input.t >= WOBBLE_START {
        WOBBLE_M * (std::f64::consts::PI * (input.t - WOBBLE_START)).sin() as f32
    } else {
        0.0
    };
    let head = input.head_position + Vec3::X * wobble;
    if (HMD_DROPOUT.0..HMD_DROPOUT.1).contains(&input.t) {
        core.solve(input.at, &Anchors::default())
    } else {
        core.solve(input.at, &anchors(input.at, head, input.head_rotation))
    }
}

#[test]
fn leg_ik_locks_planted_feet_keeps_bone_lengths_and_never_validates_anything() {
    let mut ik = calibrated(config(false, true));
    let mut fk = calibrated(config(false, false));
    let body = Anthropometrics::default();
    // A small pelvis sway keeps the nearly straight synthetic legs inside
    // their reach, so the ankle lock is never limited by the reach clamp.
    let script = vec![Motion::Stand {
        secs: 14.0,
        sway_m: 0.01,
    }];
    let mut locked_at_start: Option<[Vec3; 2]> = None;
    let mut ik_deviation = 0.0_f32;
    let (mut fk_low, mut fk_high) = (f32::INFINITY, f32::NEG_INFINITY);
    let mut locked_frames = 0;
    for input in inputs(script, |_| [0.0; 6]).iter() {
        let a = sway_step(&mut ik, input);
        let b = sway_step(&mut fk, input);
        assert_finite(&a);
        // Validity and confidence never depend on IK.
        assert_eq!(a.targets.len(), b.targets.len());
        for (x, y) in a.targets.iter().zip(&b.targets) {
            assert_eq!(
                (x.role, x.valid, x.confidence),
                (y.role, y.valid, y.confidence)
            );
            // Measured segment orientations are never altered (feet without an
            // IMU are inferred and may differ).
            if !matches!(x.role, TargetRole::LeftFoot | TargetRole::RightFoot) {
                assert_eq!(x.orientation_xyzw, y.orientation_xyzw, "{:?}", x.role);
            }
        }
        for (x, y) in a.joints.iter().zip(&b.joints) {
            assert_eq!(
                (x.joint, x.valid, x.confidence),
                (y.joint, y.valid, y.confidence)
            );
        }
        for (x, y) in a.segments.iter().zip(&b.segments) {
            assert_eq!(x.orientation_xyzw, y.orientation_xyzw);
        }
        // Bones keep their declared lengths whenever IK publishes positions.
        if a.estimator.leg_ik_active {
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
                let thigh = joint(&a, hip).distance(joint(&a, knee));
                let calf = joint(&a, knee).distance(joint(&a, ankle));
                assert!((thigh - body.thigh_length_m).abs() < 1.0e-5, "{thigh}");
                assert!((calf - body.calf_length_m).abs() < 1.0e-5, "{calf}");
            }
        }
        if (STALE_CALF.0..STALE_CALF.1).contains(&input.t) && !segment(&a, Role::LeftCalf).valid {
            assert!(!target(&a, TargetRole::LeftFoot).valid);
            assert!(!a.estimator.leg_ik.foot_locked[0]);
            assert_eq!(a.estimator.contacts[0].phase, ContactPhase::Swing);
        }
        if (HMD_DROPOUT.0..HMD_DROPOUT.1).contains(&input.t) {
            assert!(a.targets.iter().all(|t| !t.valid));
            assert!(!a.estimator.leg_ik_active);
        }
        // Lock window: both feet planted before the wobble starts.
        if input.t >= WOBBLE_START - 0.01 && input.t < 7.5 {
            let ankles = [
                joint(&a, JointRole::LeftAnkle),
                joint(&a, JointRole::RightAnkle),
            ];
            let start = *locked_at_start.get_or_insert(ankles);
            for foot in 0..2 {
                ik_deviation = ik_deviation.max(ankles[foot].distance(start[foot]));
            }
            let fk_x = joint(&b, JointRole::LeftAnkle).x;
            fk_low = fk_low.min(fk_x);
            fk_high = fk_high.max(fk_x);
            if a.estimator.leg_ik.foot_locked == [true; 2] {
                locked_frames += 1;
            }
        }
    }
    println!(
        "leg IK sway: FK ankle wobble {:.1} mm p-p, published ankle deviation {:.3} mm, locked {locked_frames}",
        (fk_high - fk_low) * 1000.0,
        ik_deviation * 1000.0
    );
    assert!(fk_high - fk_low > 0.025, "FK wobble {}", fk_high - fk_low);
    assert!(ik_deviation < 0.002, "{ik_deviation}");
    assert!(locked_frames > 540, "{locked_frames}");
}

// ---------------------------------------------------------------------------
// (e) Stale and firmware-held calf.

#[test]
fn stale_calf_blocks_contact_and_held_calf_is_never_filter_evidence() {
    let mut core = calibrated(config(true, true));
    let mut reference = calibrated(config(false, false));
    let stale = 8.0..9.5;
    let held = 12.0..13.0;
    let mut last_measured: Option<(f64, Quat)> = None;
    let mut stale_frames = 0;
    let mut held_frames = 0;
    let mut accepted_left: Option<u32> = None;
    let mut right_accepted = (u32::MAX, 0);
    for input in inputs(scenarios::walk_laps(3), leg_error(10.0)).iter() {
        let silent = stale.contains(&input.t) || held.contains(&input.t);
        for c in [&mut core, &mut reference] {
            feed_sensors(c, input, silent.then_some(Role::LeftCalf));
        }
        if !silent {
            last_measured = Some((input.at, input.measured[4]));
        } else if held.contains(&input.t) {
            let (measured_at, body) = last_measured.unwrap();
            for c in [&mut core, &mut reference] {
                c.ingest(SensorSample {
                    provenance: SensorProvenance::FirmwareHeld {
                        firmware: "0.7.2".into(),
                        observed_at_secs: input.at,
                        last_transport_at_secs: input.at,
                        continuity_since_secs: measured_at,
                        sensor_status_healthy: true,
                        sequence_contiguous: true,
                    },
                    ..sample(Role::LeftCalf, measured_at, body)
                })
                .unwrap();
            }
        }
        let head = anchors(input.at, input.head_position, input.head_rotation);
        let frame = core.solve(input.at, &head);
        let expected = reference.solve(input.at, &head);
        assert_finite(&frame);
        for (x, y) in frame.targets.iter().zip(&expected.targets) {
            assert_eq!(
                (x.role, x.valid, x.confidence),
                (y.role, y.valid, y.confidence)
            );
            assert_eq!(x.orientation_source, y.orientation_source);
        }
        let calf = segment(&frame, Role::LeftCalf);
        let feet = &frame.estimator.stance.feet;
        let unusable = !calf.valid || calf.uses_firmware_hold;
        if unusable {
            if !calf.valid {
                // A stale calf has no usable orientation at all: no contact, no lock.
                stale_frames += 1;
                assert!(!target(&frame, TargetRole::LeftFoot).valid);
                assert!(target(&frame, TargetRole::LeftKnee).valid);
                assert_eq!(frame.estimator.contacts[0].phase, ContactPhase::Swing);
                assert!(!frame.estimator.leg_ik.foot_locked[0]);
            } else {
                // A firmware-held calf is a calf assumed to be standing still, so
                // contact and the foot lock may continue. The heading filter must
                // still refuse it as motion evidence (checked below).
                held_frames += 1;
                assert!(target(&frame, TargetRole::LeftFoot).valid);
            }
            assert!(!feet[0].keyframe_active);
            let before = *accepted_left.get_or_insert(feet[0].accepted);
            assert_eq!(
                feet[0].accepted, before,
                "left foot updated from a stale calf"
            );
            right_accepted.0 = right_accepted.0.min(feet[1].accepted);
            right_accepted.1 = right_accepted.1.max(feet[1].accepted);
        } else {
            accepted_left = None;
        }
    }
    assert!(stale_frames > 100, "{stale_frames}");
    assert!(held_frames > 60, "{held_frames}");
    // The other foot keeps observing meanwhile.
    assert!(right_accepted.1 > right_accepted.0, "{right_accepted:?}");
}

// ---------------------------------------------------------------------------
// (f) Recovery and undo with nonzero stance corrections.

#[test]
fn recovery_and_undo_compose_with_stance_corrections_exactly() {
    let mut core = calibrated(config(true, true));
    let mut script = scenarios::walk_laps(2);
    script.push(Motion::stand(6.0));
    let walking = 16.0;
    let step_at = walking + 0.5;
    let request_at = walking + 2.0;
    let undo_at = walking + 4.0;
    // 10° common-mode leg drift while walking; then, standing still, an
    // unobservable +6° whole-body drift that recovery has to remove.
    let errors = move |t: f64| {
        let extra = if t >= step_at { 6.0 } else { 0.0 };
        [
            extra,
            extra,
            10.0 + extra,
            10.0 + extra,
            10.0 + extra,
            10.0 + extra,
        ]
    };
    let mut frames: Vec<(Input, TrackingFrame)> = Vec::new();
    let mut requested = false;
    let mut undone = false;
    for input in inputs(script, errors) {
        if !requested && input.t >= request_at {
            core.request_recovery(input.at);
            requested = true;
        }
        if !undone && input.t >= undo_at {
            assert!(core.undo_recovery());
            undone = true;
        }
        let frame = step(&mut core, &input);
        frames.push((input, frame));
    }
    let applied = frames
        .iter()
        .position(|(_, f)| matches!(f.recovery, RecoveryState::Applied { .. }))
        .expect("recovery applied");
    let undo = frames
        .iter()
        .position(|(_, f)| f.recovery == RecoveryState::Undone)
        .expect("recovery undone");
    let (before, after, undone) = (&frames[applied - 1].1, &frames[applied].1, &frames[undo].1);
    let c = before.estimator.stance.corrections;
    assert!(max_abs(corrections_degrees(before)) > 2.0, "{c:?}");
    // Standing still: the filter does not move from before recovery to after undo,
    // so the comparison below is exact.
    for (_, frame) in &frames[applied - 2..=undo] {
        assert_eq!(frame.estimator.stance.corrections, c);
    }
    // Recovery keeps the stance corrections and re-initialises the covariance.
    for sigma in after.estimator.stance.sigma {
        assert!((sigma.to_degrees() - 2.0).abs() < 1.0e-3, "{sigma}");
    }
    // Right after recovery every published heading equals the HMD heading.
    let head_heading = yaw_of(frames[applied].0.head_rotation);
    for role in Role::CORE {
        let heading = yaw_of(published(after, role));
        assert!(
            wrap(heading - head_heading).abs() < 1.0e-3,
            "{role:?}: {} vs {}",
            heading.to_degrees(),
            head_heading.to_degrees()
        );
        // Before recovery: the unobservable +6° whole-body drift remains (legs
        // also carry what is left of their stance-corrected 10° error).
        let truth = frames[applied - 1].0.truth.rotations[index(role)];
        let drifted = wrap(yaw_of(published(before, role)) - yaw_of(truth)).to_degrees();
        assert!(
            (drifted + 6.0).abs() < 1.5,
            "{role:?} drift before recovery {drifted}"
        );
        // The reported yaw correction moved by exactly the recovery delta.
        let moved = segment(after, role).yaw_correction_degrees
            - segment(before, role).yaw_correction_degrees;
        let delta = wrap(heading - yaw_of(published(before, role))).to_degrees();
        assert!(
            (moved - delta).abs() < 1.0e-2,
            "{role:?} {moved} vs {delta}"
        );
    }
    // Undo restores the published headings (and full orientations) exactly.
    for role in Role::CORE {
        let error = angle(published(undone, role), published(before, role));
        assert!(error < 1.0e-4, "{role:?} {error}");
        assert!(
            (segment(undone, role).yaw_correction_degrees
                - segment(before, role).yaw_correction_degrees)
                .abs()
                < 1.0e-3
        );
    }
}

// ---------------------------------------------------------------------------
// (g) Rebase mid-walk.

const REBASE_AT: f64 = 20.0;

/// Walking with 10° leg drift and automatic rebasing enabled. With `rebase`
/// `(yaw, shift, explicit)`, from `REBASE_AT` on the standing space is rotated
/// by `yaw` and shifted by `shift`; `explicit` tells the core first, otherwise
/// the automatic discontinuity detector has to find it.
fn rebased_walk(rebase: Option<(f32, Vec3, bool)>) -> Vec<(Input, TrackingFrame)> {
    let mut config = config(true, true);
    let mut core = calibrated(config.clone());
    config.estimator.enable_auto_rebase = true;
    core.set_config(config).unwrap();
    let mut rebased = false;
    inputs(scenarios::walk_laps(4), leg_error(10.0))
        .into_iter()
        .map(|mut input| {
            if let Some((yaw, shift, explicit)) = rebase.filter(|_| input.t >= REBASE_AT) {
                if explicit && !rebased {
                    core.rebase_standing_space(yaw);
                }
                rebased = true;
                let space = Quat::from_rotation_y(yaw);
                input.head_position = space * input.head_position + shift;
                input.head_rotation = space * input.head_rotation;
            }
            let frame = step(&mut core, &input);
            (input, frame)
        })
        .collect()
}

#[test]
fn rebase_mid_walk_keeps_corrections_without_a_false_correction() {
    // No rebase at all: normal walking never looks like a tracking jump.
    let reference = rebased_walk(None);
    let last = &reference.last().unwrap().1.estimator;
    assert_eq!(
        (last.standing_space_rebases, last.tracking_discontinuities),
        (0, 0)
    );
    for (yaw, shift, explicit) in [
        (50.0_f32.to_radians(), Vec3::ZERO, true),
        (70.0_f32.to_radians(), Vec3::new(0.5, 0.0, 0.0), false),
    ] {
        let run = rebased_walk(Some((yaw, shift, explicit)));
        let first = run.iter().position(|(i, _)| i.t >= REBASE_AT).unwrap();
        let (previous, at) = (&run[first - 1].1, &run[first].1);
        assert_eq!(previous.estimator.standing_space_rebases, 0);
        assert_eq!(at.estimator.standing_space_rebases, 1);
        assert_eq!(at.estimator.tracking_discontinuities, u32::from(!explicit));
        let after = &run.last().unwrap().1.estimator;
        assert_eq!(after.standing_space_rebases, 1);
        assert!((at.estimator.last_rebase_degrees - yaw.to_degrees()).abs() < 1.0e-3);
        // The frame of the rebase applies exactly the corrections from before it.
        for (i, role) in Role::CORE.into_iter().enumerate() {
            let applied = previous.estimator.stance.corrections[i].to_degrees();
            assert!((segment(at, role).yaw_correction_degrees - applied).abs() < 1.0e-4);
        }
        assert_ne!(
            at.estimator.stance.status,
            StanceHeadingStatus::Discontinuity
        );
        assert!(max_abs(corrections_degrees(previous)) > 5.0);
        // Afterwards the estimate stays converged in the rotated space and
        // follows the run without any rebase.
        let mut worst_error = 0.0_f32;
        let mut worst_divergence = 0.0_f32;
        for ((input, frame), (_, expected)) in run[first..].iter().zip(&reference[first..]) {
            assert_finite(frame);
            worst_error = worst_error.max(max_abs(leg_heading_errors(frame, &input.truth, yaw)));
            let difference = std::array::from_fn::<f32, 6, _>(|i| {
                corrections_degrees(frame)[i] - corrections_degrees(expected)[i]
            });
            worst_divergence = worst_divergence.max(max_abs(difference));
        }
        println!(
            "rebase {:.0}° ({}): worst leg error after {worst_error:.2}°, \
             max divergence from the unrebased run {worst_divergence:.3}°",
            yaw.to_degrees(),
            if explicit { "explicit" } else { "detected" }
        );
        assert!(worst_error < 3.0, "{worst_error}");
        assert!(worst_divergence < 1.0, "{worst_divergence}");
    }
}

// ---------------------------------------------------------------------------
// (h) Desktop preview.

fn preview_core(new_layers: bool) -> TrackingCore {
    let mut config = TrackingConfig {
        enable_preview_motion: true,
        ..TrackingConfig::default()
    };
    config.estimator.enable_prediction = true;
    config.estimator.enable_stance_heading = new_layers;
    config.estimator.enable_leg_ik = new_layers;
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(
        Role::CORE
            .iter()
            .map(|role| RoleAssignment {
                sensor_id: format!("preview-{role:?}"),
                role: *role,
            })
            .collect(),
    )
    .unwrap();
    for tick in 0..=100 {
        let time = tick as f64 / 100.0;
        for role in Role::CORE {
            core.ingest(SensorSample {
                id: format!("preview-{role:?}"),
                orientation_xyzw: Quat::IDENTITY.to_array(),
                received_at_secs: time,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(time, &Anchors::default());
    }
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    // Desktop fixture: identity sensor axes are known.
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }
    core
}

#[test]
fn preview_keeps_the_new_layers_inactive_and_preview_motion_unchanged() {
    let mut with_layers = preview_core(true);
    let mut without = preview_core(false);
    let mut moved = 0.0_f32;
    for input in inputs(scenarios::walk_laps(1), leg_error(10.0)).iter() {
        let at = input.at - START + 1.01;
        for core in [&mut with_layers, &mut without] {
            for (i, role) in Role::CORE.into_iter().enumerate() {
                core.ingest(SensorSample {
                    id: format!("preview-{role:?}"),
                    orientation_xyzw: input.measured[i].to_array(),
                    received_at_secs: at,
                    acceleration_mps2: None,
                    provenance: SensorProvenance::Measured,
                })
                .unwrap();
            }
        }
        // An incidental live headset must not activate anything in preview.
        let head = anchors(at, input.head_position, input.head_rotation);
        let a = with_layers.solve(at, &head);
        let b = without.solve(at, &head);
        assert_eq!(
            serde_json::to_string(&a).unwrap(),
            serde_json::to_string(&b).unwrap()
        );
        assert!(a.preview_only && a.calibration.preview_only);
        assert!(a.targets.iter().all(|t| !t.valid));
        assert!(!a.estimator.stance_active && !a.estimator.leg_ik_active);
        assert!(
            a.estimator
                .contacts
                .iter()
                .all(|c| c.phase == ContactPhase::Swing)
        );
        assert_eq!(a.estimator.stance.corrections, [0.0; 6]);
        assert!(a.preview_motion.enabled);
        moved = moved.max(Vec3::from_array(a.preview_motion.root_translation_m).length());
    }
    assert!(moved > 0.1, "preview motion did not move: {moved}");
}

// ---------------------------------------------------------------------------
// (i) Nonfinite fuzz through the public API with every flag on.

fn everything_on() -> TrackingConfig {
    let mut config = TrackingConfig {
        enable_joint_yaw_correction: true,
        enable_preview_motion: true,
        ..TrackingConfig::default()
    };
    let e = &mut config.estimator;
    e.enable_prediction = true;
    e.prediction_lead_secs = 0.02;
    e.enable_hmd_extrapolation = true;
    e.enable_auto_rebase = true;
    e.enable_stance_heading = true;
    e.enable_leg_ik = true;
    config
}

#[test]
fn nonfinite_fuzz_never_panics_or_emits_nonfinite_values_with_every_flag_on() {
    let mut core = calibrated(everything_on());
    let mut rng = XorShift64::new(2024);
    let mut pick = |n: u64| rng.next_u64() % n;
    let walk = inputs(scenarios::walk_laps(2), leg_error(10.0));
    let mut live_frames = 0;
    for input in &walk {
        for (i, role) in Role::CORE.into_iter().enumerate() {
            let mut s = sample(role, input.at, input.measured[i]);
            match pick(60) {
                0 => s.orientation_xyzw[1] = f32::NAN,
                1 => s.orientation_xyzw = [0.0; 4],
                2 => s.received_at_secs = f64::NAN,
                3 => s.received_at_secs = f64::INFINITY,
                4 => s.acceleration_mps2 = Some([f32::INFINITY, 0.0, 0.0]),
                5 => s.orientation_xyzw = [3.0e38, 3.0e38, 0.0, 0.0],
                6 => s.received_at_secs = -1.0,
                7 => s.acceleration_mps2 = Some([30.0, -5.0, 2.0]),
                8 => continue,
                _ => {}
            }
            let _ = core.ingest(s);
        }
        let mut head = hmd(input.at, input.head_position, input.head_rotation);
        match pick(40) {
            0 => head.position[0] = f32::NAN,
            1 => head.position = [1.0e4, 1.0e4, -1.0e4],
            2 => head.orientation_xyzw = [f32::INFINITY, 0.0, 0.0, 1.0],
            3 => head.received_at_secs = f64::NAN,
            4 => head.linear_velocity = Some([f32::NAN, 0.0, 0.0]),
            5 => head.linear_velocity = Some([9.9, -9.0, 0.0]),
            6 => head.valid = false,
            7 => head.position = [f32::MAX, 0.0, f32::MIN],
            8 => head.angular_velocity = Some([f32::INFINITY; 3]),
            9 => head.orientation_xyzw = [0.0, 0.0, 0.0, 1.0e-5],
            10 => head.position[1] += 0.6,
            _ => {}
        }
        let mut now = input.at;
        match pick(80) {
            0 => now = f64::NAN,
            1 => now = f64::INFINITY,
            2 => now = -1.0,
            3 => now = input.at - 0.5,
            4 => now = input.at + 0.3,
            _ => {}
        }
        match pick(150) {
            0 => core.rebase_standing_space(f32::NAN),
            1 => core.rebase_standing_space(1.0e9),
            2 => core.rebase_standing_space(0.4),
            3 => core.request_recovery(input.at),
            4 => {
                core.undo_recovery();
            }
            5 => core.revoke_firmware_hold(&id(Role::LeftCalf)),
            6 => core.mark_sensor_unhealthy(&id(Role::Hip)),
            7 => core.request_recovery(f64::NAN),
            _ => {}
        }
        let frame = core.solve(
            now,
            &Anchors {
                hmd: Some(head),
                ..Default::default()
            },
        );
        assert_finite(&frame);
        live_frames += usize::from(frame.estimator.stance_active);
    }
    // Invalid estimator settings are rejected and leave the core usable.
    let mut bad = everything_on();
    bad.estimator.contact.enter_speed_mps = f32::NAN;
    assert!(core.set_config(bad).is_err());
    let mut bad = everything_on();
    bad.estimator.stance.hmd_sigma_m = f32::INFINITY;
    assert!(core.set_config(bad).is_err());
    let mut bad = everything_on();
    bad.estimator.leg_ik.root_gain = f32::NAN;
    assert!(core.set_config(bad).is_err());
    let last = walk.last().unwrap();
    assert_finite(&step(
        &mut core,
        &Input {
            at: last.at + 0.01,
            ..*last
        },
    ));
    assert!(live_frames > walk.len() / 2, "{live_frames}");
}

// ---------------------------------------------------------------------------
// (j) Determinism.

#[test]
fn identical_runs_produce_identical_frames() {
    let run = || {
        let mut core = calibrated(everything_on());
        inputs(scenarios::walk_laps(1), leg_error(10.0))
            .iter()
            .map(|input| serde_json::to_string(&step(&mut core, input)).unwrap())
            .collect::<Vec<_>>()
    };
    let (a, b) = (run(), run());
    assert_eq!(a.len(), b.len());
    for (k, (x, y)) in a.iter().zip(&b).enumerate() {
        assert_eq!(x, y, "frame {k}");
    }
}

// ---------------------------------------------------------------------------
// Configuration, legacy correction and lifecycle.

#[test]
fn estimator_configuration_loads_old_files_and_rejects_invalid_layers() {
    let mut value = serde_json::to_value(TrackingConfig::default()).unwrap();
    let estimator = value["estimator"].as_object_mut().unwrap();
    for key in ["contact", "stance", "leg_ik"] {
        assert!(estimator.remove(key).is_some(), "{key}");
    }
    estimator.insert(
        "stance".into(),
        serde_json::json!({ "enable_pelvis_prior": false }),
    );
    let config: TrackingConfig = serde_json::from_value(value).unwrap();
    assert_eq!(config.estimator.contact, Default::default());
    assert_eq!(config.estimator.leg_ik, Default::default());
    assert!(!config.estimator.stance.enable_pelvis_prior);
    assert!(config.estimator.stance.enable_knee_prior);
    assert!(!config.estimator.enable_stance_heading && !config.estimator.enable_leg_ik);
    // Old recorded frames without the new diagnostics still parse.
    let status: EstimatorStatus =
        serde_json::from_str(r#"{"prediction_active":true,"standing_space_rebases":2}"#).unwrap();
    assert!(status.prediction_active && !status.stance_active);
    assert_eq!(status.leg_ik.status, LegIkStatus::Disabled);
    let round_trip: EstimatorStatus =
        serde_json::from_str(&serde_json::to_string(&status).unwrap()).unwrap();
    assert_eq!(round_trip, status);

    let invalid: [fn(&mut EstimatorConfig); 8] = [
        |e| e.contact.exit_speed_mps = 0.1,
        |e| e.contact.max_gap_secs = 0.0,
        |e| e.stance.hmd_sigma_m = 0.0,
        |e| e.stance.initial_sigma_degrees = 50.0,
        |e| e.stance.outlier_release_count = 0,
        |e| e.stance.knee_prior.min_flexion_degrees = 130.0,
        |e| e.leg_ik.root_gain = 1.5,
        |e| e.leg_ik.lock_break_m = -0.01,
    ];
    for (k, change) in invalid.iter().enumerate() {
        let mut config = TrackingConfig::default();
        change(&mut config.estimator);
        assert!(config.validate().is_err(), "case {k}");
        assert!(TrackingCore::new(config).is_err(), "case {k}");
    }
}

/// Walks until the stance filter holds a sizeable correction.
fn walked(core: &mut TrackingCore) -> (Vec<Input>, TrackingFrame) {
    let walk = inputs(scenarios::walk_laps(2), leg_error(10.0));
    let mut last = None;
    for input in &walk {
        let frame = step(core, input);
        for correction in &frame.corrections {
            assert_eq!(correction.status, "superseded_by_stance_filter");
            assert_eq!(correction.applied_relative_degrees, 0.0);
        }
        last = Some(frame);
    }
    let last = last.unwrap();
    assert!(max_abs(corrections_degrees(&last)) > 5.0);
    (walk, last)
}

#[test]
fn legacy_knee_correction_is_superseded_and_lifecycle_events_reset_the_filter() {
    let mut config = config(true, true);
    config.enable_joint_yaw_correction = true;
    let mut core = calibrated(config.clone());
    let (walk, last) = walked(&mut core);
    let next = |k: usize| Input {
        at: walk.last().unwrap().at + 0.01 * k as f64,
        ..*walk.last().unwrap()
    };
    let kept = last.estimator.stance.corrections;

    // New tuning keeps the corrections and restarts the covariance.
    let mut tuned = config.clone();
    tuned.estimator.stance.max_step_degrees = 0.5;
    tuned.estimator.contact.settle_secs = 0.1;
    tuned.estimator.leg_ik.root_gain = 0.5;
    core.set_config(tuned.clone()).unwrap();
    let frame = step(&mut core, &next(1));
    for (i, role) in Role::CORE.into_iter().enumerate() {
        assert!((segment(&frame, role).yaw_correction_degrees - kept[i].to_degrees()).abs() < 1e-4);
    }
    assert!(
        frame
            .estimator
            .stance
            .sigma
            .iter()
            .all(|s| (s.to_degrees() - 2.0).abs() < 0.1)
    );

    // A sensor fault widens only that segment's heading uncertainty, once per
    // outage, and keeps every correction.
    let before = step(&mut core, &next(2));
    core.mark_sensor_unhealthy(&id(Role::LeftThigh));
    core.mark_sensor_unhealthy(&id(Role::LeftThigh));
    let after = step(&mut core, &next(3));
    let (s0, s1) = (
        before.estimator.stance.sigma.map(f32::to_degrees),
        after.estimator.stance.sigma.map(f32::to_degrees),
    );
    let widened = (s0[2].powi(2) + 100.0).sqrt();
    assert!((s1[2] - widened).abs() < 0.2, "{s0:?} -> {s1:?}");
    for i in [0, 1, 3, 4, 5] {
        assert!((s1[i] - s0[i]).abs() < 0.2, "{s0:?} -> {s1:?}");
    }
    assert!(max_abs(corrections_degrees(&after)) > 5.0);

    // Switching the filter off removes its corrections from the output at once.
    let mut off = tuned.clone();
    off.estimator.enable_stance_heading = false;
    core.set_config(off).unwrap();
    let frame = step(&mut core, &next(4));
    assert!(
        frame
            .segments
            .iter()
            .all(|s| s.yaw_correction_degrees == 0.0)
    );
    assert_eq!(frame.estimator.stance.corrections, [0.0; 6]);
    // The legacy correction runs again while the filter is off.
    assert!(
        frame
            .corrections
            .iter()
            .all(|c| c.status != "superseded_by_stance_filter")
    );

    // A manual mounting edit of a core role starts the filter from zero.
    let mut core = calibrated(config.clone());
    walked(&mut core);
    core.set_manual_mount(
        Role::LeftCalf,
        tracking_core::manual_mounting::ManualMountValues {
            forward_degrees: 5.0,
            ..Default::default()
        },
    )
    .unwrap();
    let frame = step(&mut core, &next(3));
    assert_eq!(frame.estimator.stance.corrections, [0.0; 6]);

    // Clearing calibration and changing core assignments do the same.
    let mut core = calibrated(config.clone());
    walked(&mut core);
    core.clear_calibration();
    let frame = step(&mut core, &next(3));
    assert_eq!(frame.estimator.stance.corrections, [0.0; 6]);
    assert!(!frame.estimator.stance_active);

    let mut core = calibrated(config);
    walked(&mut core);
    let mut assignments = core.export_profile().assignments;
    assignments[0].sensor_id = "replacement".into();
    core.set_assignments(assignments).unwrap();
    let frame = step(&mut core, &next(3));
    assert_eq!(frame.estimator.stance.corrections, [0.0; 6]);
}
