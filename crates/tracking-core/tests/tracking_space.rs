//! Synthetic regressions for headset-pose continuity across SteamVR
//! tracking-space events, long headset gaps, the desktop-to-headset alignment
//! and the height-adopting neutral capture. They check the implementation
//! against its own model; none of this has been exercised against a real
//! SteamVR restart, input-mode switch or recenter.

use glam::{Quat, Vec3};
use tracking_core::forward_calibration::ski_pitch_sign;
use tracking_core::tracking_space::{
    self, ContinuityDecision, HeadsetObservation, RebaseEvidence, TrackingSpaceHint,
};
use tracking_core::*;

fn index(role: Role) -> usize {
    Role::ALL.iter().position(|r| *r == role).unwrap()
}
fn id(role: Role) -> String {
    format!("space-{role:?}")
}
fn mount(role: Role) -> Quat {
    let i = index(role) as f32;
    Quat::from_rotation_x(0.3 + 0.23 * i)
        * Quat::from_rotation_z(-0.7 + 0.31 * i)
        * Quat::from_rotation_y(0.4 + 0.41 * i)
}
fn bias(role: Role) -> f32 {
    [-1.4, 2.2, 1.5707964, -2.7, 0.9, -0.4, 0.3, 0.7, -1.1, 2.8][index(role)]
}
fn pitch(role: Role) -> f32 {
    ski_pitch_sign(role).unwrap_or(0.0)
        * [28.0_f32, 22.0, 38.0, 34.0, 18.0, 20.0, 0.0, 0.0, 0.0, 0.0][index(role)].to_radians()
}
/// Raw sensor quaternion for a standing-space body segment rotation. `drift`
/// is an extra sensor-world yaw (a different IMU heading origin).
fn raw(role: Role, body: Quat, drift: f32) -> Quat {
    Quat::from_rotation_y(bias(role) + drift) * body * mount(role)
}
const EYE: f32 = 1.65;

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
fn head(time: f64, heading: f32) -> Anchors {
    Anchors {
        hmd: Some(hmd(
            time,
            Vec3::new(0.0, EYE, 0.0),
            Quat::from_rotation_y(heading),
        )),
        ..Default::default()
    }
}
fn core_with(config: TrackingConfig) -> TrackingCore {
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
    core
}
/// Body segment rotation of a pose: everything faces `heading`; `bend`
/// selects the skiing pose; `leg_turn` turns the four leg segments.
fn body(role: Role, heading: f32, bend: bool, leg_turn: f32) -> Quat {
    let turn = if matches!(role, Role::Chest | Role::Hip) {
        0.0
    } else {
        leg_turn
    };
    Quat::from_rotation_y(heading + turn)
        * Quat::from_rotation_x(if bend { pitch(role) } else { 0.0 })
}
fn ingest_pose(core: &mut TrackingCore, time: f64, heading: f32, bend: bool, drift: f32) {
    for role in Role::CORE {
        core.ingest(SensorSample {
            id: id(role),
            orientation_xyzw: raw(role, body(role, heading, bend, 0.0), drift).to_array(),
            received_at_secs: time,
            acceleration_mps2: None,
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
}
/// Feeds `ticks` at 100 Hz: sensors facing `heading` (`bend` for the skiing
/// pose) and, when `head_heading` is set, an upright headset facing it.
fn feed(
    core: &mut TrackingCore,
    ticks: std::ops::RangeInclusive<usize>,
    heading: f32,
    bend: bool,
    drift: f32,
    head_heading: Option<f32>,
) -> Option<TrackingFrame> {
    let mut last = None;
    for tick in ticks {
        let time = tick as f64 / 100.0;
        ingest_pose(core, time, heading, bend, drift);
        let anchors = head_heading.map_or_else(Anchors::default, |h| head(time, h));
        last = Some(core.solve(time, &anchors));
    }
    last
}

/// Live calibration (neutral + skiing pose) with the body facing `heading`.
fn live_calibrated(config: TrackingConfig, heading: f32) -> TrackingCore {
    let mut core = core_with(config);
    feed(&mut core, 0..=100, heading, false, 0.0, Some(heading));
    core.calibrate_neutral(1.0, &head(1.0, heading), false)
        .unwrap();
    feed(&mut core, 101..=200, heading, true, 0.0, Some(heading));
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}

fn json(config: &TrackingConfig) -> serde_json::Value {
    serde_json::to_value(config).unwrap()
}

fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos()
}

fn observation(at_secs: f64, position: Vec3, yaw: f32) -> HeadsetObservation {
    HeadsetObservation {
        at_secs,
        position: position.to_array(),
        orientation_xyzw: (Quat::from_rotation_y(yaw) * Quat::from_rotation_x(-0.15)).to_array(),
    }
}

// ---------------------------------------------------------------------------
// Pure continuity decision.

#[test]
fn continuous_headset_with_a_transform_change_is_not_a_rebase() {
    let p = Vec3::new(0.2, 1.6, -0.3);
    let previous = observation(10.0, p, 0.4);
    // Ordinary head motion between consecutive frames and across a 1.5 s gap.
    for (dt, step, turn) in [(0.011, 0.01, 0.02), (1.5, 0.4, 0.5)] {
        let current = observation(10.0 + dt, p + Vec3::X * step, 0.4 + turn);
        let hint = TrackingSpaceHint {
            at_secs: 10.0,
            transform_yaw_radians: Some(70.0_f32.to_radians()),
        };
        assert_eq!(
            tracking_space::decide(&previous, &current, Some(&hint)),
            ContinuityDecision::Continuous,
            "{dt}"
        );
    }
}

#[test]
fn jump_agreeing_with_the_transform_rebases_by_its_exact_yaw() {
    let p = Vec3::new(0.3, 1.6, -0.4);
    let previous = observation(10.0, p, 0.4);
    let transform = 70.0_f32.to_radians();
    // The space rotated by 70°; the head itself also turned 4° in that frame.
    let rotated = Quat::from_rotation_y(transform) * p + Vec3::new(1.2, 0.0, 0.5);
    let current = observation(10.011, rotated, 0.4 + transform + 4.0_f32.to_radians());
    let hint = TrackingSpaceHint {
        at_secs: 10.005,
        transform_yaw_radians: Some(transform),
    };
    assert_eq!(
        tracking_space::decide(&previous, &current, Some(&hint)),
        ContinuityDecision::Rebase {
            yaw_radians: transform,
            evidence: RebaseEvidence::Transform,
        }
    );
    // The same jump with a transform that disagrees by more than 10° uses
    // the observed yaw instead.
    let wrong = TrackingSpaceHint {
        at_secs: 10.005,
        transform_yaw_radians: Some(transform + 15.0_f32.to_radians()),
    };
    let ContinuityDecision::Rebase {
        yaw_radians,
        evidence: RebaseEvidence::ObservedJump,
    } = tracking_space::decide(&previous, &current, Some(&wrong))
    else {
        panic!("observed jump");
    };
    assert!((yaw_radians - (transform + 4.0_f32.to_radians())).abs() < 1e-4);
}

#[test]
fn jump_without_transform_change_rebases_by_the_observed_yaw() {
    let p = Vec3::new(0.0, 1.6, 0.0);
    let previous = observation(10.0, p, 0.4);
    let current = observation(10.011, p + Vec3::new(0.8, 0.0, 0.0), 0.4 - 1.2);
    for hint in [
        None,
        Some(TrackingSpaceHint {
            at_secs: 10.0,
            transform_yaw_radians: None,
        }),
    ] {
        let ContinuityDecision::Rebase {
            yaw_radians,
            evidence: RebaseEvidence::ObservedJump,
        } = tracking_space::decide(&previous, &current, hint.as_ref())
        else {
            panic!("observed jump");
        };
        assert!((yaw_radians + 1.2).abs() < 1e-4);
    }
    // Looking straight down: a jump without a usable yaw.
    let down = |at: f64, position: Vec3, yaw: f32| HeadsetObservation {
        at_secs: at,
        position: position.to_array(),
        orientation_xyzw: (Quat::from_rotation_y(yaw)
            * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2))
        .to_array(),
    };
    assert_eq!(
        tracking_space::decide(
            &down(10.0, p, 0.4),
            &down(10.011, p + Vec3::X * 0.9, 1.0),
            None
        ),
        ContinuityDecision::JumpWithoutHeading
    );
}

#[test]
fn long_gaps_resume_and_short_gaps_need_a_hint() {
    let p = Vec3::new(0.0, 1.6, 0.0);
    let previous = observation(10.0, p, 0.4);
    // Minutes later, anywhere: resumed, never compared.
    let later = observation(190.0, p + Vec3::X * 3.0, 2.5);
    let hint = TrackingSpaceHint {
        at_secs: 12.0,
        transform_yaw_radians: Some(2.1),
    };
    for hint in [None, Some(&hint)] {
        assert_eq!(
            tracking_space::decide(&previous, &later, hint),
            ContinuityDecision::Resumed { gap_secs: 180.0 }
        );
    }
    // A 1 s gap without a hint is not compared, even with a large change.
    let jumped = observation(11.0, p + Vec3::X * 5.0, 0.4);
    assert_eq!(
        tracking_space::decide(&previous, &jumped, None),
        ContinuityDecision::NotCompared
    );
    // With a hint, 5 m in 1 s exceeds plausible motion (2.0 m).
    let hint = TrackingSpaceHint {
        at_secs: 10.2,
        transform_yaw_radians: None,
    };
    assert!(matches!(
        tracking_space::decide(&previous, &jumped, Some(&hint)),
        ContinuityDecision::Rebase {
            evidence: RebaseEvidence::ObservedJump,
            ..
        }
    ));
    // A hint older than its 0.5 s window no longer applies.
    let stale = TrackingSpaceHint {
        at_secs: 9.0,
        transform_yaw_radians: None,
    };
    assert_eq!(
        tracking_space::decide(&previous, &jumped, Some(&stale)),
        ContinuityDecision::NotCompared
    );
}

#[test]
fn slow_recenter_across_a_gap_is_recognised_from_the_matching_transform() {
    // A 90° recenter across a 1.5 s gap stays within plausible motion (166°),
    // so it is not a jump; the observed 88° change is clearly the transform's
    // 90° rather than no change.
    let p = Vec3::new(0.1, 1.6, 0.0);
    let previous = observation(10.0, p, 0.4);
    let transform = 90.0_f32.to_radians();
    let current = observation(11.5, p, 0.4 + 88.0_f32.to_radians());
    let hint = TrackingSpaceHint {
        at_secs: 10.3,
        transform_yaw_radians: Some(transform),
    };
    assert_eq!(
        tracking_space::decide(&previous, &current, Some(&hint)),
        ContinuityDecision::Rebase {
            yaw_radians: transform,
            evidence: RebaseEvidence::Transform,
        }
    );
    // Without transform evidence that slow change is plausible head motion.
    let session_only = TrackingSpaceHint {
        at_secs: 10.3,
        transform_yaw_radians: None,
    };
    assert_eq!(
        tracking_space::decide(&previous, &current, Some(&session_only)),
        ContinuityDecision::Continuous
    );
}

#[test]
fn thresholds_scale_with_time_and_reject_nonfinite_input() {
    let (p0, a0) = tracking_space::plausible_motion(0.0);
    let (p1, a1) = tracking_space::plausible_motion(0.1);
    let (p2, a2) = tracking_space::plausible_motion(2.0);
    assert!((p0 - 0.25).abs() < 1e-6 && (a0 - 25.0_f32.to_radians()).abs() < 1e-6);
    assert!((p1 - 0.65).abs() < 1e-5 && (a1 - (25.0_f32.to_radians() + 1.0)).abs() < 1e-5);
    assert!((p2 - 3.5).abs() < 1e-4 && a2 > a1);
    assert!((tracking_space::agreement_tolerance(0.0) - 10.0_f32.to_radians()).abs() < 1e-6);
    assert!((tracking_space::agreement_tolerance(2.0) - 50.0_f32.to_radians()).abs() < 1e-5);
    let p = Vec3::new(0.0, 1.6, 0.0);
    let previous = observation(10.0, p, 0.4);
    let mut broken = observation(10.01, p, 0.4);
    broken.orientation_xyzw = [f32::NAN, 0.0, 0.0, 1.0];
    assert_eq!(
        tracking_space::decide(&previous, &broken, None),
        ContinuityDecision::NotCompared
    );
    broken = observation(f64::NAN, p, 0.4);
    assert_eq!(
        tracking_space::decide(&previous, &broken, None),
        ContinuityDecision::NotCompared
    );
    let nan_hint = TrackingSpaceHint {
        at_secs: 10.0,
        transform_yaw_radians: Some(f32::NAN),
    };
    let jumped = observation(10.011, p + Vec3::X, 1.4);
    assert!(matches!(
        tracking_space::decide(&previous, &jumped, Some(&nan_hint)),
        ContinuityDecision::Rebase {
            evidence: RebaseEvidence::ObservedJump,
            ..
        }
    ));
}

// ---------------------------------------------------------------------------
// The detector inside the core.

const HEADING: f32 = 0.4;

fn assert_targets_match(a: &TrackingFrame, b: &TrackingFrame, rotation: Quat, shift: Vec3) {
    assert_eq!(a.targets.len(), b.targets.len());
    for (a, b) in a.targets.iter().zip(&b.targets) {
        assert_eq!(a.role, b.role);
        assert!(a.valid && b.valid, "{:?}", a.role);
        let expected = rotation * Vec3::from_array(a.position) + shift;
        let error = (Vec3::from_array(b.position) - expected).length();
        assert!(error < 2e-3, "{:?} position error {error}", a.role);
        let error = angle(
            Quat::from_array(b.orientation_xyzw),
            rotation * Quat::from_array(a.orientation_xyzw),
        );
        assert!(error < 2e-3, "{:?} orientation error {error}", a.role);
    }
}

fn steady(
    core: &mut TrackingCore,
    ticks: std::ops::RangeInclusive<usize>,
    space: Quat,
    shift: Vec3,
) -> TrackingFrame {
    let mut last = None;
    for tick in ticks {
        let time = tick as f64 / 100.0;
        ingest_pose(core, time, HEADING, false, 0.0);
        let anchors = Anchors {
            hmd: Some(hmd(
                time,
                space * Vec3::new(0.1, EYE, -0.05) + shift,
                space * Quat::from_rotation_y(HEADING) * Quat::from_rotation_x(-0.1),
            )),
            ..Default::default()
        };
        last = Some(core.solve(time, &anchors));
    }
    last.unwrap()
}

#[test]
fn recenter_across_a_reported_gap_rebases_by_the_transform_yaw() {
    let mut core = live_calibrated(TrackingConfig::default(), HEADING);
    let mut config = core.config().clone();
    config.estimator.enable_auto_rebase = true;
    core.set_config(config).unwrap();
    let before = steady(&mut core, 201..=230, Quat::IDENTITY, Vec3::ZERO);
    // SteamVR reports a 70° pure-yaw transform change; the headset is absent
    // for 1 s; it returns in the rotated space.
    let space = Quat::from_rotation_y(70.0_f32.to_radians());
    let shift = Vec3::new(0.5, 0.0, 0.0);
    core.note_tracking_space_event(2.32, Some(70.0_f32.to_radians()));
    for tick in 231..=330 {
        ingest_pose(&mut core, tick as f64 / 100.0, HEADING, false, 0.0);
        let frame = core.solve(tick as f64 / 100.0, &Anchors::default());
        assert!(frame.targets.iter().all(|t| !t.valid), "headset absent");
    }
    let after = steady(&mut core, 331..=340, space, shift);
    assert!(after.calibration.calibrated && after.calibration.direction_calibrated);
    assert_eq!(after.estimator.standing_space_rebases, 1);
    assert_eq!(
        after.estimator.last_rebase_evidence,
        Some(RebaseEvidence::Transform)
    );
    assert!((after.estimator.last_rebase_degrees - 70.0).abs() < 1e-3);
    assert_targets_match(&before, &after, space, shift);
}

#[test]
fn transform_change_without_headset_jump_keeps_the_calibration_unchanged() {
    for gap_ticks in [0usize, 100] {
        let mut core = live_calibrated(TrackingConfig::default(), HEADING);
        let mut config = core.config().clone();
        config.estimator.enable_auto_rebase = true;
        core.set_config(config).unwrap();
        let before = steady(&mut core, 201..=230, Quat::IDENTITY, Vec3::ZERO);
        // Raw space re-originated (driver restart, input-mode switch): the
        // transform changes by 70°, the standing pose does not move.
        core.note_tracking_space_event(2.305, Some(70.0_f32.to_radians()));
        let mut tick = 230;
        for _ in 0..gap_ticks {
            tick += 1;
            ingest_pose(&mut core, tick as f64 / 100.0, HEADING, false, 0.0);
            core.solve(tick as f64 / 100.0, &Anchors::default());
        }
        let after = steady(&mut core, tick + 1..=tick + 20, Quat::IDENTITY, Vec3::ZERO);
        assert_eq!(after.estimator.standing_space_rebases, 0, "{gap_ticks}");
        assert_eq!(after.estimator.tracking_discontinuities, 0);
        assert_targets_match(&before, &after, Quat::IDENTITY, Vec3::ZERO);
    }
}

#[test]
fn long_headset_gap_keeps_calibration_and_widens_uncertainty() {
    let config = TrackingConfig {
        estimator: EstimatorConfig {
            enable_auto_rebase: true,
            enable_stance_heading: true,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut core = live_calibrated(config, HEADING);
    let before = steady(&mut core, 201..=230, Quat::IDENTITY, Vec3::ZERO);
    let sigma_before = before.estimator.stance.sigma;
    // SteamVR shut down (idle headset) and restarted with a new helper session
    // 30 s later; nothing connects the two poses.
    core.note_tracking_space_event(5.0, None);
    for tick in 231..=3300 {
        if tick % 10 == 0 {
            ingest_pose(&mut core, tick as f64 / 100.0, HEADING, false, 0.0);
            let frame = core.solve(tick as f64 / 100.0, &Anchors::default());
            assert!(frame.targets.iter().all(|t| !t.valid));
            assert!(frame.calibration.calibrated);
        }
    }
    let after = steady(&mut core, 3301..=3310, Quat::IDENTITY, Vec3::ZERO);
    assert!(after.calibration.calibrated && after.calibration.direction_calibrated);
    assert!(
        after.targets.iter().all(|t| t.valid),
        "valid again, no action"
    );
    assert_eq!(after.estimator.headset_resumptions, 1);
    assert!((after.estimator.last_headset_gap_secs - 30.71).abs() < 0.02);
    assert_eq!(after.estimator.standing_space_rebases, 0);
    let sigma_after = after.estimator.stance.sigma;
    for i in 0..6 {
        let widened = if i < 2 { 8.0_f32 } else { 15.0 };
        let expected = (sigma_before[i].powi(2) + widened.to_radians().powi(2)).sqrt();
        assert!(sigma_after[i] > sigma_before[i] + 1e-4, "{i}");
        assert!(sigma_after[i] <= expected + 1e-3, "{i}");
    }
    // Stance filter off: still kept, only the counter.
    let mut plain = live_calibrated(TrackingConfig::default(), HEADING);
    steady(&mut plain, 201..=230, Quat::IDENTITY, Vec3::ZERO);
    for tick in (240..900).step_by(10) {
        ingest_pose(&mut plain, tick as f64 / 100.0, HEADING, false, 0.0);
        plain.solve(tick as f64 / 100.0, &Anchors::default());
    }
    let after = steady(&mut plain, 900..=910, Quat::IDENTITY, Vec3::ZERO);
    assert_eq!(after.estimator.headset_resumptions, 1);
    assert!(after.targets.iter().all(|t| t.valid));
}

#[test]
fn session_change_alone_keeps_calibration_and_preview_ignores_hints() {
    let mut core = live_calibrated(TrackingConfig::default(), HEADING);
    let mut config = core.config().clone();
    config.estimator.enable_auto_rebase = true;
    core.set_config(config).unwrap();
    let before = steady(&mut core, 201..=230, Quat::IDENTITY, Vec3::ZERO);
    core.note_tracking_space_event(2.305, None);
    core.note_tracking_space_event(f64::NAN, Some(1.0));
    let after = steady(&mut core, 231..=260, Quat::IDENTITY, Vec3::ZERO);
    assert_eq!(after.estimator.standing_space_rebases, 0);
    assert_targets_match(&before, &after, Quat::IDENTITY, Vec3::ZERO);

    // A desktop calibration never follows the headset space.
    let mut preview = core_with(TrackingConfig {
        estimator: EstimatorConfig {
            enable_auto_rebase: true,
            ..Default::default()
        },
        ..Default::default()
    });
    feed(&mut preview, 0..=100, HEADING, false, 0.0, None);
    preview
        .calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    steady(&mut preview, 101..=130, Quat::IDENTITY, Vec3::ZERO);
    preview.note_tracking_space_event(1.305, Some(1.2));
    let frame = steady(&mut preview, 131..=140, Quat::from_rotation_y(1.2), Vec3::X);
    assert_eq!(frame.estimator.standing_space_rebases, 0);
    assert!(frame.calibration.preview_only);
}

// ---------------------------------------------------------------------------
// Desktop calibration carried over to the headset.

/// Desktop calibration (neutral + skiing pose) facing preview heading
/// `preview_heading`, with IMU heading origins offset by `drift`.
fn preview_calibrated(config: TrackingConfig, preview_heading: f32, drift: f32) -> TrackingCore {
    let mut core = core_with(config);
    feed(&mut core, 0..=100, preview_heading, false, drift, None);
    core.calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    feed(&mut core, 101..=200, preview_heading, true, drift, None);
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}

/// Feeds an upright, still body facing `heading` with the headset facing the
/// same way, from `start` for `ticks`; returns the last time.
fn stand_with_headset(
    core: &mut TrackingCore,
    start: usize,
    ticks: usize,
    heading: f32,
    drift: f32,
) -> f64 {
    let mut time = 0.0;
    for tick in start..start + ticks {
        time = tick as f64 / 100.0;
        ingest_pose(core, time, heading, false, drift);
        core.solve(time, &head(time, heading));
    }
    time
}

#[test]
fn aligned_desktop_calibration_equals_a_live_calibration_in_the_same_pose() {
    let live_heading = -2.3_f32;
    // The desktop calibration faced some other physical direction (1.1 rad)
    // in a preview frame; the IMU heading origins are unknown to it.
    let mut aligned = preview_calibrated(TrackingConfig::default(), 1.1, 0.0);
    let mounting_before = aligned.export_profile().mounting;
    let at = stand_with_headset(&mut aligned, 201, 100, live_heading, 0.0);
    assert!(
        aligned
            .solve(at, &head(at, live_heading))
            .calibration
            .preview_only
    );
    aligned
        .align_preview_to_headset(at, &head(at, live_heading))
        .unwrap();
    // A live calibration made in exactly that pose.
    let mut live = core_with(TrackingConfig::default());
    feed(
        &mut live,
        0..=100,
        live_heading,
        false,
        0.0,
        Some(live_heading),
    );
    live.calibrate_neutral(1.0, &head(1.0, live_heading), false)
        .unwrap();
    feed(
        &mut live,
        101..=200,
        live_heading,
        true,
        0.0,
        Some(live_heading),
    );
    live.calibrate_forward(2.0).unwrap();
    // Mounting is untouched by the alignment.
    for (a, b) in aligned
        .export_profile()
        .mounting
        .iter()
        .zip(&mounting_before)
    {
        assert_eq!(a.sensor_to_bone_xyzw, b.sensor_to_bone_xyzw);
    }
    // Identical input from here on: a knee lift, a turn and a lean.
    for tick in 400..=460 {
        let time = tick as f64 / 100.0;
        let phase = (tick - 400) as f32 / 60.0;
        for core in [&mut aligned, &mut live] {
            for role in Role::CORE {
                let lift = if role == Role::LeftThigh {
                    -1.2 * phase
                } else {
                    0.0
                };
                let body = Quat::from_rotation_y(live_heading + 0.6 * phase)
                    * Quat::from_rotation_x(lift + 0.1 * phase);
                core.ingest(SensorSample {
                    id: id(role),
                    orientation_xyzw: raw(role, body, 0.0).to_array(),
                    received_at_secs: time,
                    acceleration_mps2: None,
                    provenance: SensorProvenance::Measured,
                })
                .unwrap();
            }
        }
        let anchors = Anchors {
            hmd: Some(hmd(
                time,
                Vec3::new(0.1 * phase, EYE - 0.05 * phase, 0.0),
                Quat::from_rotation_y(live_heading + 0.6 * phase),
            )),
            ..Default::default()
        };
        let a = aligned.solve(time, &anchors);
        let b = live.solve(time, &anchors);
        assert!(!a.preview_only && !a.calibration.preview_only);
        // Tolerance: 1 mm and 0.1°.
        for (x, y) in a.targets.iter().zip(&b.targets) {
            assert!(x.valid && y.valid, "{:?}", x.role);
            let d = (Vec3::from_array(x.position) - Vec3::from_array(y.position)).length();
            assert!(d < 1e-3, "{:?} {d}", x.role);
            let e = angle(
                Quat::from_array(x.orientation_xyzw),
                Quat::from_array(y.orientation_xyzw),
            );
            assert!(e < 0.1_f32.to_radians(), "{:?} {e}", x.role);
        }
    }
    // Recovery/undo state is clean and recovery is now allowed (live).
    assert!(!aligned.undo_recovery());
    assert!(
        aligned
            .align_preview_to_headset(4.6, &head(4.6, live_heading))
            .is_err()
    );
}

#[test]
fn alignment_waits_for_an_upright_still_pose_and_changes_nothing_until_then() {
    let heading = 0.7;
    let base = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    let unchanged = |core: &TrackingCore| {
        core.calibration_forward() == base.calibration_forward()
            && core.export_profile().mounting.len() == base.export_profile().mounting.len()
    };
    // No headset at all.
    let mut core = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    let at = {
        let mut t = 0.0;
        for tick in 201..=300 {
            t = tick as f64 / 100.0;
            ingest_pose(&mut core, t, heading, false, 0.3);
            core.solve(t, &Anchors::default());
        }
        t
    };
    assert!(
        core.align_preview_to_headset(at, &Anchors::default())
            .is_err()
    );
    assert!(unchanged(&core));
    // Headset present but the body sways beyond the stability limit.
    let mut core = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    let mut at = 0.0;
    for tick in 201..=300 {
        at = tick as f64 / 100.0;
        let sway = 6.0_f32.to_radians() * (at as f32 * 12.0).sin();
        ingest_pose(&mut core, at, heading + sway, false, 0.3);
        core.solve(at, &head(at, heading));
    }
    let error = core
        .align_preview_to_headset(at, &head(at, heading))
        .unwrap_err();
    assert!(error.0.contains("moving"), "{error}");
    let frame = core.solve(at, &head(at, heading));
    assert!(frame.calibration.preview_only && frame.preview_only);
    // Headset looking down.
    let mut core = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    let mut at = 0.0;
    let down = |t: f64| Anchors {
        hmd: Some(hmd(
            t,
            Vec3::new(0.0, EYE, 0.0),
            Quat::from_rotation_y(heading) * Quat::from_rotation_x(-0.8),
        )),
        ..Default::default()
    };
    for tick in 201..=300 {
        at = tick as f64 / 100.0;
        ingest_pose(&mut core, at, heading, false, 0.3);
        core.solve(at, &down(at));
    }
    assert!(core.align_preview_to_headset(at, &down(at)).is_err());
    // Segments tilted: still in the skiing pose.
    let mut core = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    for tick in 201..=300 {
        at = tick as f64 / 100.0;
        ingest_pose(&mut core, at, heading, true, 0.3);
        core.solve(at, &head(at, heading));
    }
    let error = core
        .align_preview_to_headset(at, &head(at, heading))
        .unwrap_err();
    assert!(error.0.contains("tilt"), "{error}");
    // Legs turned 40° away from the torso.
    let mut core = preview_calibrated(TrackingConfig::default(), -0.4, 0.3);
    for tick in 201..=300 {
        at = tick as f64 / 100.0;
        for role in Role::CORE {
            core.ingest(SensorSample {
                id: id(role),
                orientation_xyzw: raw(role, body(role, heading, false, 0.7), 0.3).to_array(),
                received_at_secs: at,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(at, &head(at, heading));
    }
    let error = core
        .align_preview_to_headset(at, &head(at, heading))
        .unwrap_err();
    assert!(error.0.contains("different way"), "{error}");
    assert!(unchanged(&core));
    // Then standing straight works, regardless of the configured eye height
    // (not enforced here: changing it would void the calibration).
    let tall = |t: f64| Anchors {
        hmd: Some(hmd(
            t,
            Vec3::new(0.0, EYE + 0.35, 0.0),
            Quat::from_rotation_y(heading),
        )),
        ..Default::default()
    };
    for tick in 301..=400 {
        at = tick as f64 / 100.0;
        ingest_pose(&mut core, at, heading, false, 0.3);
        core.solve(at, &tall(at));
    }
    core.align_preview_to_headset(at, &tall(at)).unwrap();
    let frame = core.solve(at + 0.01, &tall(at + 0.01));
    assert!(!frame.preview_only && frame.targets.iter().all(|t| t.valid));
    let forward = Vec3::from_array(core.calibration_forward().unwrap());
    assert!(forward.is_finite());
}

#[test]
fn alignment_widens_the_stance_filter_and_requires_a_desktop_calibration() {
    let config = TrackingConfig {
        estimator: EstimatorConfig {
            enable_stance_heading: true,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut core = preview_calibrated(config.clone(), 0.2, 0.0);
    let at = stand_with_headset(&mut core, 201, 100, 1.4, 0.0);
    let sigma_before = core.solve(at, &head(at, 1.4)).estimator.stance.sigma;
    core.align_preview_to_headset(at, &head(at, 1.4)).unwrap();
    let frame = core.solve(at + 0.01, &head(at + 0.01, 1.4));
    for i in 0..6 {
        assert!(frame.estimator.stance.sigma[i] > sigma_before[i], "{i}");
    }
    assert_eq!(frame.estimator.standing_space_rebases, 0);
    assert!(matches!(frame.recovery, RecoveryState::Idle));
    // A live calibration, a neutral-only desktop capture and no calibration
    // are all refused.
    let mut live = live_calibrated(config.clone(), 0.2);
    let at = stand_with_headset(&mut live, 201, 100, 0.2, 0.0);
    assert!(live.align_preview_to_headset(at, &head(at, 0.2)).is_err());
    let mut neutral_only = core_with(config);
    feed(&mut neutral_only, 0..=100, 0.2, false, 0.0, None);
    neutral_only
        .calibrate_neutral(1.0, &Anchors::default(), true)
        .unwrap();
    let at = stand_with_headset(&mut neutral_only, 101, 100, 0.2, 0.0);
    assert!(
        neutral_only
            .align_preview_to_headset(at, &head(at, 0.2))
            .is_err()
    );
}

// ---------------------------------------------------------------------------
// Headset height for the automatic eye-height update.

#[test]
fn stable_headset_height_and_height_adopting_neutral_capture() {
    let mut core = core_with(TrackingConfig::default());
    let measured = 1.71;
    let at_head = |t: f64| Anchors {
        hmd: Some(hmd(
            t,
            Vec3::new(0.0, measured, 0.0),
            Quat::from_rotation_y(0.3),
        )),
        ..Default::default()
    };
    assert_eq!(core.stable_headset_height(0.5), None);
    for tick in 0..=100 {
        let t = tick as f64 / 100.0;
        ingest_pose(&mut core, t, 0.3, false, 0.0);
        core.solve(t, &at_head(t));
    }
    let height = core.stable_headset_height(1.0).unwrap();
    assert!((height - measured).abs() < 1e-5);
    // Rejected under the new configuration (sensors moving): nothing changes.
    let original = core.config().clone();
    let mut taller = original.clone();
    taller.anthropometrics.standing_eye_height_m = measured;
    taller.anthropometrics.normalize_chain_height().unwrap();
    let mut moving = core_with(TrackingConfig::default());
    for tick in 0..=100 {
        let t = tick as f64 / 100.0;
        ingest_pose(
            &mut moving,
            t,
            0.3 + 0.3 * (t as f32 * 9.0).sin(),
            false,
            0.0,
        );
        moving.solve(t, &at_head(t));
    }
    assert!(
        moving
            .calibrate_neutral_with_config(1.0, &at_head(1.0), false, taller.clone())
            .is_err()
    );
    assert_eq!(json(moving.config()), json(&original));
    // A previously valid calibration survives a rejected attempt.
    let mut calibrated = live_calibrated(TrackingConfig::default(), 0.3);
    for tick in 201..=300 {
        let t = tick as f64 / 100.0;
        ingest_pose(
            &mut calibrated,
            t,
            0.3 + 0.3 * (t as f32 * 9.0).sin(),
            false,
            0.0,
        );
        calibrated.solve(t, &at_head(t));
    }
    assert!(
        calibrated
            .calibrate_neutral_with_config(3.0, &at_head(3.0), false, taller.clone())
            .is_err()
    );
    assert!(calibrated.direction_calibrated());
    assert_eq!(json(calibrated.config()), json(&original));
    // Accepted: the configuration is adopted and the capture is valid.
    core.calibrate_neutral_with_config(1.0, &at_head(1.0), false, taller.clone())
        .unwrap();
    assert_eq!(json(core.config()), json(&taller));
    let frame = core.solve(1.0, &at_head(1.0));
    assert!(frame.calibration.calibrated);
    assert!(frame.calibration.floor_mismatch_m.unwrap().abs() < 1e-4);
}
