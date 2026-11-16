//! Contact hypotheses: hysteresis, gates and time anomalies.

use glam::{Quat, Vec3};
use tracking_core::contact::*;

const DT: f64 = 0.01;
const LEFT: Vec3 = Vec3::new(-0.12, 0.08, 0.0);
const RIGHT: Vec3 = Vec3::new(0.12, 0.08, 0.0);

struct Rig {
    tracker: ContactTracker,
    at: f64,
    ankles: [Vec3; 2],
    calves: [Quat; 2],
}

impl Rig {
    fn new() -> Self {
        Self::with(ContactConfig::default())
    }

    fn with(config: ContactConfig) -> Self {
        Self {
            tracker: ContactTracker::new(config),
            at: 0.0,
            ankles: [LEFT, RIGHT],
            calves: [Quat::IDENTITY; 2],
        }
    }

    /// Advances one frame moving the left foot by `velocity` and turning its calf at `yaw_dps`.
    fn step(&mut self, velocity: Vec3, yaw_dps: f32) -> [FootContact; 2] {
        self.at += DT;
        self.ankles[0] += velocity * DT as f32;
        self.calves[0] = Quat::from_rotation_y(yaw_dps.to_radians() * DT as f32) * self.calves[0];
        self.tracker
            .update(self.at, self.ankles, self.calves, [true, true])
    }

    fn run(&mut self, secs: f64, velocity: Vec3, yaw_dps: f32) -> [FootContact; 2] {
        let mut last = self.tracker.contacts();
        for _ in 0..(secs / DT).round() as usize {
            last = self.step(velocity, yaw_dps);
        }
        last
    }

    /// Frames until the left foot is planted (None within `limit` seconds).
    fn time_to_plant(&mut self, velocity: Vec3, limit: f64) -> Option<f64> {
        let start = self.at;
        while self.at - start < limit {
            if self.step(velocity, 0.0)[0].planted() {
                return Some(self.at - start);
            }
        }
        None
    }
}

#[test]
fn still_feet_plant_only_after_the_settling_dwell() {
    let mut rig = Rig::new();
    let mut settled_at = None;
    let mut planted_at = None;
    for _ in 0..30 {
        let contacts = rig.step(Vec3::ZERO, 0.0);
        if contacts[0].phase == ContactPhase::Settling && settled_at.is_none() {
            settled_at = Some(rig.at);
        }
        if contacts[0].planted() && planted_at.is_none() {
            planted_at = Some(rig.at);
            assert!(contacts[1].planted());
            assert_eq!(contacts[0].since_secs, rig.at);
        }
    }
    let (settled, planted) = (settled_at.unwrap(), planted_at.unwrap());
    let dwell = planted - settled;
    assert!(
        (0.08 - 1.0e-9..0.08 + DT + 1.0e-9).contains(&dwell),
        "dwell {dwell}"
    );
}

#[test]
fn speed_hysteresis_between_enter_and_exit_thresholds() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    // 0.30 m/s is above the entry (0.20) but below the exit (0.45) threshold.
    let contacts = rig.run(0.5, Vec3::new(0.30, 0.0, 0.0), 0.0);
    assert!(contacts[0].planted(), "{contacts:?}");
    assert!(contacts[1].planted());
    // Faster than the exit threshold releases.
    let contacts = rig.run(0.3, Vec3::new(0.60, 0.0, 0.0), 0.0);
    assert_eq!(contacts[0].phase, ContactPhase::Swing);
    // At 0.30 m/s the foot cannot re-enter.
    let contacts = rig.run(1.0, Vec3::new(0.30, 0.0, 0.0), 0.0);
    assert_eq!(contacts[0].phase, ContactPhase::Swing);
    // Slowing below 0.20 m/s plants again after the dwell.
    assert!(rig.time_to_plant(Vec3::new(0.10, 0.0, 0.0), 1.0).is_some());
}

#[test]
fn height_above_lower_ankle_gates_entry_and_exit() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    assert!(rig.tracker.contacts()[0].planted());
    // Lift the left ankle slowly to 8 cm (between 6 and 10 cm): stays planted.
    let contacts = rig.run(1.0, Vec3::new(0.0, 0.08, 0.0), 0.0);
    assert!((contacts[0].height_above_lower_m - 0.08).abs() < 0.005);
    assert!(contacts[0].planted());
    assert_eq!(contacts[1].height_above_lower_m, 0.0);
    // Beyond 10 cm releases.
    let contacts = rig.run(0.5, Vec3::new(0.0, 0.08, 0.0), 0.0);
    assert_eq!(contacts[0].phase, ContactPhase::Swing);
    // Held still at 12 cm: cannot plant (entry requires < 6 cm).
    assert!(rig.time_to_plant(Vec3::ZERO, 1.0).is_none());
}

#[test]
fn calf_yaw_rate_gates_entry_and_exit() {
    let config = ContactConfig {
        // Isolate the rate gates from the accumulated-yaw budget.
        max_planted_yaw_degrees: 0.0,
        ..ContactConfig::default()
    };
    let mut rig = Rig::with(config);
    rig.run(0.3, Vec3::ZERO, 0.0);
    let contacts = rig.run(0.5, Vec3::ZERO, 60.0);
    assert!(contacts[0].planted(), "60°/s is inside the hysteresis band");
    assert!((contacts[0].yaw_rate_dps - 60.0).abs() < 1.0);
    let contacts = rig.run(0.2, Vec3::ZERO, 120.0);
    assert_eq!(contacts[0].phase, ContactPhase::Swing);
    let contacts = rig.run(1.0, Vec3::ZERO, 60.0);
    assert_eq!(
        contacts[0].phase,
        ContactPhase::Swing,
        "entry needs < 40°/s"
    );
    assert!(contacts[1].planted(), "the other foot is unaffected");
}

#[test]
fn planted_calf_yaw_budget_releases_a_slow_pivot() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    // 30°/s is below every rate gate, but after > 15° the foot cannot be a stationary point.
    let mut released_after = None;
    for k in 0..100 {
        let contacts = rig.step(Vec3::ZERO, 30.0);
        if !contacts[0].planted() {
            released_after = Some(k as f32 * 0.3);
            break;
        }
        assert!(contacts[0].planted_yaw_degrees.abs() <= 15.0 + 0.5);
    }
    let turned = released_after.expect("pivot never released");
    assert!((14.0..=16.0).contains(&turned), "released after {turned}°");
    // Disabled budget: the same pivot stays planted.
    let mut rig = Rig::with(ContactConfig {
        max_planted_yaw_degrees: 0.0,
        ..ContactConfig::default()
    });
    rig.run(0.3, Vec3::ZERO, 0.0);
    assert!(rig.run(2.0, Vec3::ZERO, 30.0)[0].planted());
}

#[test]
fn vertical_lift_releases_before_the_speed_gate() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    // 0.25 m/s straight up: below the 0.45 m/s speed and 10 cm height gates for a while.
    let mut released = None;
    for k in 0..20 {
        if !rig.step(Vec3::new(0.0, 0.25, 0.0), 0.0)[0].planted() {
            released = Some(k);
            break;
        }
    }
    assert!(released.is_some_and(|frames| frames <= 6), "{released:?}");
    // Without the vertical gate it stays planted while the ankle rises 5 cm.
    let mut rig = Rig::with(ContactConfig {
        exit_vertical_speed_mps: 0.0,
        ..ContactConfig::default()
    });
    rig.run(0.3, Vec3::ZERO, 0.0);
    assert!(rig.run(0.2, Vec3::new(0.0, 0.25, 0.0), 0.0)[0].planted());
}

#[test]
fn repeated_or_backward_time_releases_both_feet_and_restarts_dwell() {
    for offset in [0.0, -0.05, -10.0] {
        let mut rig = Rig::new();
        rig.run(0.3, Vec3::ZERO, 0.0);
        assert!(rig.tracker.contacts().iter().all(FootContact::planted));
        let anomalous = rig.at + offset;
        let contacts = rig
            .tracker
            .update(anomalous, rig.ankles, rig.calves, [true, true]);
        assert!(
            contacts.iter().all(|c| c.phase == ContactPhase::Swing),
            "{offset}"
        );
        rig.at = anomalous;
        // No dwell is carried over: planting again takes the full velocity warm-up + 80 ms.
        let replant = rig.time_to_plant(Vec3::ZERO, 1.0).unwrap();
        assert!(replant >= 0.08, "{offset}: replanted after {replant}");
    }
}

#[test]
fn frame_gap_releases_and_never_manufactures_dwell() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    rig.at += 0.15; // > max_gap_secs
    let contacts = rig.step(Vec3::ZERO, 0.0);
    assert!(contacts.iter().all(|c| c.phase == ContactPhase::Swing));
    let replant = rig.time_to_plant(Vec3::ZERO, 1.0).unwrap();
    assert!(replant >= 0.08);

    // Sparse but valid frames (0.09 s apart): planting needs real elapsed settle time.
    let mut tracker = ContactTracker::new(ContactConfig::default());
    let calves = [Quat::IDENTITY; 2];
    let mut first_settling = None;
    for k in 0..10 {
        let at = 0.09 * k as f64;
        let contacts = tracker.update(at, [LEFT, RIGHT], calves, [true, true]);
        if contacts[0].phase == ContactPhase::Settling {
            first_settling.get_or_insert(at);
        }
        if contacts[0].planted() {
            assert!(at - first_settling.unwrap() >= 0.08);
            return;
        }
    }
    panic!("never planted with 90 ms frames");
}

#[test]
fn nonfinite_time_and_inputs_release_without_panicking() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    let contacts = rig
        .tracker
        .update(f64::NAN, rig.ankles, rig.calves, [true, true]);
    assert!(contacts.iter().all(|c| c.phase == ContactPhase::Swing));
    rig.run(0.3, Vec3::ZERO, 0.0);
    assert!(rig.tracker.contacts()[0].planted());

    rig.at += DT;
    let contacts = rig.tracker.update(
        rig.at,
        [Vec3::NAN, RIGHT],
        [Quat::IDENTITY, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)],
        [true, true],
    );
    assert!(contacts.iter().all(|c| c.phase == ContactPhase::Swing));
    for c in contacts {
        assert!(c.speed_mps.is_finite() && c.height_above_lower_m.is_finite());
        assert!(c.yaw_rate_dps.is_finite() && c.planted_yaw_degrees.is_finite());
    }
    // Huge finite jumps are just fast motion.
    rig.at += DT;
    let contacts = rig.tracker.update(
        rig.at,
        [Vec3::splat(1.0e30), RIGHT],
        [Quat::from_xyzw(1.0e30, 0.0, 0.0, 1.0), Quat::IDENTITY],
        [true, true],
    );
    for c in contacts {
        assert!(c.speed_mps.is_finite() && c.height_above_lower_m.is_finite());
    }
}

#[test]
fn unusable_input_and_force_release_require_a_new_dwell() {
    let mut rig = Rig::new();
    rig.run(0.3, Vec3::ZERO, 0.0);
    rig.at += DT;
    let contacts = rig
        .tracker
        .update(rig.at, rig.ankles, rig.calves, [false, true]);
    assert_eq!(contacts[0].phase, ContactPhase::Swing);
    assert!(contacts[1].planted());
    assert!(rig.time_to_plant(Vec3::ZERO, 1.0).unwrap() >= 0.08);

    rig.tracker.force_release(0, rig.at);
    assert_eq!(rig.tracker.contacts()[0].phase, ContactPhase::Swing);
    assert!(rig.tracker.contacts()[1].planted());
    let replant = rig.time_to_plant(Vec3::ZERO, 1.0).unwrap();
    assert!(replant >= 0.08, "replanted after {replant}");
    // Out-of-range feet and nonfinite times are ignored safely.
    rig.tracker.force_release(7, rig.at);
    rig.tracker.force_release(1, f64::NAN);
    assert_eq!(rig.tracker.contacts()[1].phase, ContactPhase::Swing);

    rig.tracker.reset();
    assert!(
        rig.tracker
            .contacts()
            .iter()
            .all(|c| c.phase == ContactPhase::Swing)
    );
}

#[test]
fn speed_allowance_widens_enter_and_exit_speed_gates_only() {
    // 0.30 m/s cannot enter with the default 0.20 m/s gate.
    let mut rig = Rig::new();
    assert!(rig.time_to_plant(Vec3::new(0.30, 0.0, 0.0), 1.0).is_none());
    // With a 0.15 m/s allowance it enters (0.35 m/s) ...
    rig.tracker.set_speed_allowance([0.15, 0.0]);
    assert_eq!(rig.tracker.speed_allowance(), [0.15, 0.0]);
    assert!(rig.time_to_plant(Vec3::new(0.30, 0.0, 0.0), 1.0).is_some());
    // ... and the exit moves from 0.45 to 0.60 m/s.
    assert!(rig.run(0.5, Vec3::new(0.55, 0.0, 0.0), 0.0)[0].planted());
    assert_eq!(
        rig.run(0.3, Vec3::new(0.70, 0.0, 0.0), 0.0)[0].phase,
        ContactPhase::Swing
    );
    // Height and yaw-rate gates are not widened.
    rig.tracker.set_speed_allowance([0.4, 0.4]);
    rig.run(0.5, Vec3::ZERO, 0.0);
    assert!(rig.tracker.contacts()[0].planted());
    assert_eq!(
        rig.run(0.2, Vec3::ZERO, 120.0)[0].phase,
        ContactPhase::Swing
    );
}

#[test]
fn speed_allowance_is_sanitized_capped_and_reset() {
    let mut tracker = ContactTracker::new(ContactConfig::default());
    assert_eq!(tracker.speed_allowance(), [0.0; 2]);
    tracker.set_speed_allowance([f32::NAN, -1.0]);
    assert_eq!(tracker.speed_allowance(), [0.0; 2]);
    tracker.set_speed_allowance([10.0, f32::INFINITY]);
    assert_eq!(tracker.speed_allowance(), [0.40, 0.0]);
    tracker.reset();
    assert_eq!(tracker.speed_allowance(), [0.0; 2]);
    let capped = ContactTracker::new(ContactConfig {
        max_speed_allowance_mps: f32::NAN,
        ..ContactConfig::default()
    });
    let mut capped = capped;
    capped.set_speed_allowance([0.2, 0.2]);
    assert_eq!(capped.speed_allowance(), [0.0; 2]);
}

#[test]
fn config_round_trips_through_serde_with_defaults() {
    let config: ContactConfig = serde_json::from_str("{\"enter_speed_mps\": 0.25}").unwrap();
    assert_eq!(config.enter_speed_mps, 0.25);
    assert_eq!(config.max_speed_allowance_mps, 0.40);
    assert_eq!(
        config.exit_speed_mps,
        ContactConfig::default().exit_speed_mps
    );
    let text = serde_json::to_string(&ContactConfig::default()).unwrap();
    let back: ContactConfig = serde_json::from_str(&text).unwrap();
    assert_eq!(back, ContactConfig::default());
}
