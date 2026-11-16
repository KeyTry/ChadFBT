//! Movement calibration through the public `TrackingCore` API (EXPERIMENTAL).
//!
//! Synthetic only: truth comes from the shared generators in `motion_support`, which share
//! the estimator's own assumptions (rigid segments, upright neutral, sensor worlds that differ
//! by yaw only). Passing runs show that the integration commits the module's solution
//! correctly and keeps its invariants; they are not accuracy claims for a worn body.

mod motion_support;

use glam::{Quat, Vec3};
use motion_support::*;
use tracking_core::motion_calibration::{MotionReference, SignEvidence};
use tracking_core::synthetic::XorShift64;
use tracking_core::*;

const RATE_HZ: f64 = 100.0;
const NEUTRAL_AT: f64 = 1.2;

fn id(index: usize) -> String {
    format!("motion-{:?}", Role::CORE[index])
}

fn core_with(config: TrackingConfig) -> TrackingCore {
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(
        (0..6)
            .map(|i| RoleAssignment {
                sensor_id: id(i),
                role: Role::CORE[i],
            })
            .collect(),
    )
    .unwrap();
    core
}

fn hmd(truth: &Truth, at: f64, noise: Vec3) -> Anchors {
    Anchors {
        hmd: Some(AnchorPose {
            position: (truth.head_position + noise).to_array(),
            orientation_xyzw: truth.head_rotation.to_array(),
            received_at_secs: at,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        }),
        ..Default::default()
    }
}

fn angle_degrees(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos().to_degrees()
}

/// Feeds a source at 100 Hz into a core: six measured samples then one solve per tick.
struct Feeder<'a> {
    source: &'a dyn Source,
    rig: Rig,
    rng: XorShift64,
    live: bool,
    tick: u64,
}

impl<'a> Feeder<'a> {
    fn new(source: &'a dyn Source, live: bool, seed: u64) -> Self {
        Self {
            source,
            rig: Rig::standard(seed),
            rng: XorShift64::new(seed),
            live,
            tick: 0,
        }
    }

    fn now(&self) -> f64 {
        self.tick as f64 / RATE_HZ
    }

    fn anchors(&mut self, truth: &Truth, at: f64) -> Anchors {
        if !self.live {
            return Anchors::default();
        }
        let noise = Vec3::new(
            self.rng.gaussian() as f32,
            self.rng.gaussian() as f32,
            self.rng.gaussian() as f32,
        ) * 0.002;
        hmd(truth, at, noise)
    }

    /// Advances to `until` (inclusive). `each` sees every solved frame with its truth.
    fn run(
        &mut self,
        core: &mut TrackingCore,
        until: f64,
        mut each: impl FnMut(&mut TrackingCore, f64, &Truth, &TrackingFrame),
    ) {
        while self.now() + 1.0e-9 < until {
            self.tick += 1;
            let at = self.now();
            let truth = self.source.truth(at);
            for i in 0..6 {
                let noise = Vec3::new(
                    self.rng.gaussian() as f32,
                    self.rng.gaussian() as f32,
                    self.rng.gaussian() as f32,
                ) * 0.2_f32.to_radians();
                let q = (self.rig.clean(&truth, i) * Quat::from_scaled_axis(noise)).normalize();
                core.ingest(SensorSample {
                    id: id(i),
                    orientation_xyzw: q.to_array(),
                    received_at_secs: at,
                    acceleration_mps2: None,
                    provenance: SensorProvenance::Measured,
                })
                .unwrap();
            }
            let anchors = self.anchors(&truth, at);
            let frame = core.solve(at, &anchors);
            each(core, at, &truth, &frame);
        }
    }

    fn skip(&mut self, core: &mut TrackingCore, until: f64) {
        self.run(core, until, |_, _, _, _| {});
    }

    fn neutral(&mut self, core: &mut TrackingCore) {
        self.skip(core, NEUTRAL_AT);
        let truth = self.source.truth(NEUTRAL_AT);
        let anchors = self.anchors(&truth, NEUTRAL_AT);
        core.calibrate_neutral(NEUTRAL_AT, &anchors, !self.live)
            .unwrap();
    }
}

/// Bow ×2 and knee lifts ×4 between still stands, then (after `calibration_end`) more of
/// the same movement for checking the committed result.
fn bow_and_march(pelvis_degrees: f32) -> (Parametric, f64) {
    let source = Parametric::new(
        HEADING,
        vec![
            stand(1.5),
            bows(2, 60.0, pelvis_degrees),
            march(4),
            stand(1.0),
            bows(1, 45.0, 20.0),
            march(2),
            stand(0.5),
        ],
    );
    (source, 1.5 + 8.0 + 6.4 + 1.0)
}

/// Max angle between published core segments and `Ry(offset)·truth` after finishing.
fn check_after_finish(
    core: &mut TrackingCore,
    feeder: &mut Feeder,
    offset: f32,
) -> (f32, usize, usize) {
    let end = feeder.source.duration();
    let mut max_error = 0.0_f32;
    let mut frames = 0;
    let mut valid_targets = 0;
    let rotation = Quat::from_rotation_y(offset);
    feeder.run(core, end, |_, _, truth, frame| {
        frames += 1;
        valid_targets += frame.targets.iter().filter(|t| t.valid).count();
        for (i, role) in Role::CORE.iter().enumerate() {
            let segment = frame.segments.iter().find(|s| s.role == *role).unwrap();
            assert!(segment.valid);
            let published = Quat::from_array(segment.orientation_xyzw);
            max_error = max_error.max(angle_degrees(published, rotation * truth.bones[i]));
        }
    });
    (max_error, frames, valid_targets)
}

fn forward_yaw(core: &TrackingCore) -> f32 {
    let forward = Vec3::from_array(core.calibration_forward().unwrap());
    (-forward.x).atan2(-forward.z)
}

#[test]
fn preview_bow_and_knee_lifts_calibrate_all_six_segments_through_the_core() {
    let mut config = TrackingConfig::default();
    config.estimator.enable_stance_heading = true;
    let mut core = core_with(config);
    let (source, calibration_end) = bow_and_march(30.0);
    let mut feeder = Feeder::new(&source, false, 11);
    feeder.neutral(&mut core);
    assert!(!core.direction_calibrated());
    assert!(core.motion_calibration_available(NEUTRAL_AT).is_ok());
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    let mut ready_at = None;
    let mut published = 0;
    feeder.run(&mut core, calibration_end, |core, at, _, frame| {
        let status = frame.motion_calibration.as_ref().expect("status in frame");
        published += 1;
        if ready_at.is_none() && status.ready {
            ready_at = Some(at);
            // The frame copy may lag by up to 0.1 s; the direct query is fresh.
            assert!(core.motion_calibration_status().unwrap().active);
        }
    });
    assert!(published > 1000);
    let status = core.motion_calibration_status().unwrap();
    assert!(status.ready, "{:?}", status.blockers);
    assert!(status.headset.is_none());
    assert_eq!(status.selected_reference, MotionReference::HeadsetYaw);
    assert!(status.segments.iter().all(|s| s.accepted_samples > 1000));
    let finish = feeder.now();
    core.finish_motion_calibration(finish).unwrap();
    assert!(core.direction_calibrated());
    assert!(!core.motion_calibration_active());
    assert!(core.motion_calibration_status().is_none());
    // Preview neutral heading is 0; the true body faces HEADING.
    assert!(forward_yaw(&core).abs() < 1.0e-5);
    let frame = core.solve(finish, &Anchors::default());
    assert!(frame.motion_calibration.is_none());
    assert!(frame.calibration.direction_calibrated);
    assert!(frame.calibration.preview_only);
    // Stance uncertainty widened from its initial 2°: legs √(2²+10²), torso √(2²+6²).
    let sigma = frame.estimator.stance.sigma.map(f32::to_degrees);
    for (i, expected) in [6.32, 6.32, 10.2, 10.2, 10.2, 10.2].iter().enumerate() {
        assert!((sigma[i] - expected).abs() < 0.1, "sigma {sigma:?}");
    }
    let (error, frames, valid_targets) = check_after_finish(&mut core, &mut feeder, -HEADING);
    println!(
        "preview: ready {:.2} s after neutral, max segment error {error:.2}° over {frames} frames",
        ready_at.unwrap() - NEUTRAL_AT
    );
    assert!(error < 2.0, "max segment error {error}°");
    assert!(frames > 700);
    assert_eq!(valid_targets, 0, "preview must never publish live targets");
}

#[test]
fn live_headset_forward_follows_the_displacement_fused_reference() {
    let inner = Parametric::new(
        HEADING,
        vec![
            stand(1.5),
            Phase {
                movement: Movement::Bow {
                    chest_degrees: 42.0,
                    pelvis_degrees: 22.0,
                    hips_back_m: 0.08,
                    period: 4.0,
                },
                secs: 12.0,
            },
            march(4),
            stand(1.0),
            bows(1, 42.0, 22.0),
            stand(0.5),
        ],
    );
    let source = Modified {
        inner,
        mods: Mods {
            head_yaw_degrees: 20.0,
            ..Mods::default()
        },
    };
    let calibration_end = 1.5 + 12.0 + 6.4 + 1.0;
    let mut core = core_with(TrackingConfig::default());
    let mut feeder = Feeder::new(&source, true, 5);
    feeder.neutral(&mut core);
    // The neutral heading is the headset yaw: 20° off the torso.
    let yaw_error = (forward_yaw(&core) - HEADING).to_degrees();
    assert!((yaw_error - 20.0).abs() < 0.5, "{yaw_error}");
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    feeder.skip(&mut core, calibration_end);
    let status = core.motion_calibration_status().unwrap();
    assert!(status.ready, "{:?}", status.blockers);
    let headset = status
        .headset
        .clone()
        .expect("live session has headset forward");
    assert!(
        headset.available && !headset.disagreement_flagged,
        "{headset:?}"
    );
    assert_eq!(status.selected_reference, MotionReference::Fused);
    let displacement_error =
        wrap_degrees((headset.displacement_heading_radians.unwrap() - HEADING).to_degrees());
    core.finish_motion_calibration(feeder.now()).unwrap();
    assert!(core.direction_calibrated());
    let fused_error = wrap_degrees((forward_yaw(&core) - HEADING).to_degrees());
    let fused_reported =
        wrap_degrees((status.selected_reference_heading_radians.unwrap() - HEADING).to_degrees());
    assert!((fused_error - fused_reported).abs() < 0.05);
    let (error, _, valid_targets) =
        check_after_finish(&mut core, &mut feeder, fused_error.to_radians());
    println!(
        "live, head 20° off torso: yaw reference {yaw_error:.1}°, displacement {displacement_error:.2}°, \
         committed (fused) {fused_error:.2}° off; segments follow it within {error:.2}°"
    );
    assert!(displacement_error.abs() < 2.0);
    assert!(fused_error.abs() < 10.0 && fused_error > 0.0);
    assert!(error < 2.0);
    assert!(valid_targets > 0, "live calibration publishes targets");
}

#[test]
fn held_tokens_and_non_core_sensors_never_enter_the_session() {
    let source = Parametric::new(HEADING, vec![stand(3.0), bows(1, 60.0, 30.0)]);
    let mut core = core_with(TrackingConfig::default());
    let foot = "motion-left-foot".to_string();
    let mut assignments: Vec<_> = (0..6)
        .map(|i| RoleAssignment {
            sensor_id: id(i),
            role: Role::CORE[i],
        })
        .collect();
    assignments.push(RoleAssignment {
        sensor_id: foot.clone(),
        role: Role::LeftFoot,
    });
    core.set_assignments(assignments).unwrap();
    let mut feeder = Feeder::new(&source, false, 3);
    feeder.neutral(&mut core);
    // Nothing is forwarded without a session.
    assert!(core.motion_calibration_status().is_none());
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    let counts = |core: &TrackingCore| -> Vec<(u64, u64)> {
        core.motion_calibration_status()
            .unwrap()
            .segments
            .iter()
            .map(|s| (s.accepted_samples, s.rejected_samples))
            .collect()
    };
    assert!(counts(&core).iter().all(|c| *c == (0, 0)));
    // An optional (foot) sensor is not a core role: never forwarded.
    for k in 1..=5 {
        core.ingest(SensorSample {
            id: foot.clone(),
            orientation_xyzw: Quat::from_rotation_x(0.1 * k as f32).to_array(),
            received_at_secs: NEUTRAL_AT + 0.01 * f64::from(k),
            acceleration_mps2: None,
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
    assert!(counts(&core).iter().all(|c| *c == (0, 0)));
    // One measured chest sample is forwarded exactly once.
    let measured_at = NEUTRAL_AT + 0.01;
    let chest = feeder.rig.clean(&source.truth(measured_at), 0);
    core.ingest(SensorSample {
        id: id(0),
        orientation_xyzw: chest.to_array(),
        received_at_secs: measured_at,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    })
    .unwrap();
    let after_measured = counts(&core);
    assert_eq!(after_measured[0], (1, 0));
    // A firmware-held token that the core accepts never reaches the calibrator.
    let stored = core.solve(measured_at, &Anchors::default());
    assert!(stored.segments.iter().any(|s| s.role == Role::Chest));
    core.ingest(SensorSample {
        id: id(0),
        orientation_xyzw: chest.normalize().to_array(),
        received_at_secs: measured_at,
        acceleration_mps2: None,
        provenance: SensorProvenance::FirmwareHeld {
            firmware: "0.7.2".into(),
            observed_at_secs: measured_at + 0.09,
            last_transport_at_secs: measured_at + 0.04,
            continuity_since_secs: measured_at,
            sensor_status_healthy: true,
            sequence_contiguous: true,
        },
    })
    .expect("a well-formed held token is accepted by the core");
    assert_eq!(counts(&core), after_measured);
    let frame = core.solve(measured_at + 0.3, &Anchors::default());
    let chest_segment = frame
        .segments
        .iter()
        .find(|s| s.role == Role::Chest)
        .unwrap();
    assert!(
        chest_segment.uses_firmware_hold,
        "the token was really held"
    );
    assert_eq!(counts(&core), after_measured);
}

#[test]
fn samples_stamped_before_the_neutral_capture_are_rejected_by_the_session() {
    // Neutral at 1.2 s; the hip stream lags so its newest sample is 1.15 s. After begin, a hip
    // sample stamped 1.17 s is newer than its own stream (the core accepts it) but predates
    // neutral: the session must reject it.
    let mut core = core_with(TrackingConfig::default());
    let (source, _) = bow_and_march(30.0);
    let rig = Rig::standard(9);
    let mut tick = 0_u64;
    loop {
        tick += 1;
        let at = tick as f64 / RATE_HZ;
        if at > NEUTRAL_AT + 1.0e-9 {
            break;
        }
        let truth = source.truth(at);
        for i in 0..6 {
            if i == 1 && at > 1.15 + 1.0e-9 {
                continue;
            }
            core.ingest(SensorSample {
                id: id(i),
                orientation_xyzw: rig.clean(&truth, i).to_array(),
                received_at_secs: at,
                acceleration_mps2: None,
                provenance: SensorProvenance::Measured,
            })
            .unwrap();
        }
        core.solve(at, &Anchors::default());
    }
    core.calibrate_neutral(NEUTRAL_AT, &Anchors::default(), true)
        .unwrap();
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    core.ingest(SensorSample {
        id: id(1),
        orientation_xyzw: rig.clean(&source.truth(1.17), 1).to_array(),
        received_at_secs: 1.17,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    })
    .unwrap();
    let hip = &core.motion_calibration_status().unwrap().segments[1];
    assert_eq!(hip.accepted_samples, 0);
    assert_eq!(hip.rejected_samples, 1);
}

#[test]
fn failed_finish_leaves_the_previous_calibration_and_the_session_untouched() {
    let mut core = core_with(TrackingConfig::default());
    let (source, calibration_end) = bow_and_march(30.0);
    let mut feeder = Feeder::new(&source, false, 21);
    feeder.neutral(&mut core);
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    feeder.skip(&mut core, calibration_end);
    core.finish_motion_calibration(feeder.now()).unwrap();
    // A second session that sees no movement (the final stand only).
    let start = feeder.now();
    core.begin_motion_calibration(start).unwrap();
    feeder.skip(&mut core, start + 0.4);
    let at = feeder.now();
    let before = core.solve(at, &Anchors::default());
    let profile_before = serde_json::to_string(&core.export_profile()).unwrap();
    let forward_before = core.calibration_forward();
    let error = core.finish_motion_calibration(at).unwrap_err();
    assert!(error.0.contains("not ready"), "{error}");
    assert!(core.direction_calibrated());
    assert!(
        core.motion_calibration_active(),
        "a failed finish keeps the session"
    );
    assert_eq!(core.calibration_forward(), forward_before);
    assert_eq!(
        serde_json::to_string(&core.export_profile()).unwrap(),
        profile_before
    );
    let after = core.solve(at, &Anchors::default());
    for (a, b) in after.segments.iter().zip(&before.segments) {
        assert_eq!(a.orientation_xyzw, b.orientation_xyzw);
    }
    assert_eq!(after.calibration.reason, before.calibration.reason);
    // Finishing without a session is an error and changes nothing either.
    core.cancel_motion_calibration();
    assert!(core.finish_motion_calibration(at).is_err());
    assert!(core.direction_calibrated());
}

#[test]
fn a_barely_tilted_pelvis_blocks_finishing_and_the_error_names_hip() {
    let mut core = core_with(TrackingConfig::default());
    let (source, calibration_end) = bow_and_march(4.0);
    let mut feeder = Feeder::new(&source, false, 2);
    feeder.neutral(&mut core);
    core.begin_motion_calibration(NEUTRAL_AT).unwrap();
    feeder.skip(&mut core, calibration_end);
    let status = core.motion_calibration_status().unwrap();
    assert!(!status.ready);
    for segment in &status.segments {
        assert_eq!(segment.ready, segment.role != Role::Hip, "{segment:?}");
    }
    let error = core.finish_motion_calibration(feeder.now()).unwrap_err();
    assert!(error.0.contains("Hip"), "{error}");
    assert!(!core.direction_calibrated());
}

#[test]
fn sessions_end_on_clear_rebase_new_neutral_and_profile_restore() {
    let (source, _) = bow_and_march(30.0);
    let started = || {
        let mut core = core_with(TrackingConfig::default());
        let mut feeder = Feeder::new(&source, false, 8);
        feeder.neutral(&mut core);
        core.begin_motion_calibration(NEUTRAL_AT).unwrap();
        feeder.skip(&mut core, 3.0);
        assert!(core.motion_calibration_active());
        (core, feeder)
    };
    let (mut core, _) = started();
    core.clear_calibration();
    assert!(!core.motion_calibration_active());
    let (mut core, _) = started();
    core.rebase_standing_space(0.3);
    assert!(!core.motion_calibration_active());
    // The neutral survives a rebase, so movement can be started again (yaw reference).
    assert!(core.motion_calibration_available(3.0).is_ok());
    let (mut core, mut feeder) = started();
    feeder.skip(&mut core, 6.0);
    // Mid-bow: a new neutral is not stable, so it fails and the session continues.
    assert!(
        core.calibrate_neutral(6.0, &Anchors::default(), true)
            .is_err()
    );
    assert!(core.motion_calibration_active());
    let (mut core, mut feeder) = started();
    feeder.skip(&mut core, calibration_still_time());
    core.calibrate_neutral(feeder.now(), &Anchors::default(), true)
        .unwrap();
    assert!(!core.motion_calibration_active());
    let (mut core, _) = started();
    let profile = core.export_profile();
    core.restore_profile(profile).unwrap();
    assert!(!core.motion_calibration_active());
    // Expired neutral or no neutral: begin is refused.
    let (mut core, _) = started();
    core.cancel_motion_calibration();
    assert!(core.begin_motion_calibration(NEUTRAL_AT + 301.0).is_err());
    let mut fresh = core_with(TrackingConfig::default());
    assert!(fresh.begin_motion_calibration(1.0).is_err());
}

/// End of the final still stand in `bow_and_march` before the extra movement.
fn calibration_still_time() -> f64 {
    1.5 + 8.0 + 6.4 + 1.0
}

#[test]
fn the_held_skiing_pose_path_still_calibrates_unchanged() {
    let source = Parametric::new(HEADING, vec![stand(1.5), hold(SKI_TILTS, 3.0)]);
    let mut core = core_with(TrackingConfig::default());
    let mut feeder = Feeder::new(&source, false, 6);
    feeder.neutral(&mut core);
    feeder.skip(&mut core, source.duration());
    assert!(!core.motion_calibration_active());
    core.calibrate_forward(feeder.now()).unwrap();
    assert!(core.direction_calibrated());
    let frame = core.solve(feeder.now(), &Anchors::default());
    let truth = source.truth(feeder.now());
    let offset = Quat::from_rotation_y(-HEADING);
    for (i, role) in Role::CORE.iter().enumerate() {
        let segment = frame.segments.iter().find(|s| s.role == *role).unwrap();
        let error = angle_degrees(
            Quat::from_array(segment.orientation_xyzw),
            offset * truth.bones[i],
        );
        assert!(error < 1.5, "{role:?} {error}°");
    }
}

#[test]
fn identical_inputs_replay_bit_identically_including_session_events() {
    let run = || {
        let mut core = core_with(TrackingConfig::default());
        let (source, calibration_end) = bow_and_march(30.0);
        let mut feeder = Feeder::new(&source, false, 31);
        let mut frames = Vec::new();
        feeder.neutral(&mut core);
        core.begin_motion_calibration(NEUTRAL_AT).unwrap();
        feeder.run(&mut core, 5.0, |_, _, _, frame| {
            frames.push(serde_json::to_string(frame).unwrap())
        });
        core.cancel_motion_calibration();
        core.begin_motion_calibration(feeder.now()).unwrap();
        feeder.run(&mut core, calibration_end + 4.0, |_, _, _, frame| {
            frames.push(serde_json::to_string(frame).unwrap())
        });
        let status = core.motion_calibration_status().unwrap();
        core.finish_motion_calibration(feeder.now()).unwrap();
        feeder.run(&mut core, calibration_end + 6.0, |_, _, _, frame| {
            frames.push(serde_json::to_string(frame).unwrap())
        });
        (frames, serde_json::to_string(&status).unwrap())
    };
    let (a, status_a) = run();
    let (b, status_b) = run();
    assert_eq!(a.len(), b.len());
    assert!(a == b, "frames differ between identical runs");
    assert_eq!(status_a, status_b);
    // Frames deserialize with the new field; old recordings without it default to None.
    let frame: TrackingFrame = serde_json::from_str(&a[10]).unwrap();
    assert!(frame.motion_calibration.is_some());
    let mut value: serde_json::Value = serde_json::from_str(&a[10]).unwrap();
    value.as_object_mut().unwrap().remove("motion_calibration");
    let old: TrackingFrame = serde_json::from_value(value).unwrap();
    assert!(old.motion_calibration.is_none());
    let status: tracking_core::motion_calibration::MotionCalibrationStatus =
        serde_json::from_str(&status_a).unwrap();
    assert!(
        status
            .segments
            .iter()
            .all(|s| matches!(s.sign, SignEvidence::OneSided { .. }))
    );
}
