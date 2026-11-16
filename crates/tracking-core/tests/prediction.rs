//! Synthetic regressions for angular-rate estimation, bounded extrapolation and
//! output velocity filters (stance-anchored estimator, section 5). These check
//! numerical behaviour on scripted streams, not end-to-end latency or accuracy.

use glam::{DQuat, DVec3, Quat, Vec3};
use tracking_core::prediction::{
    AngularRateConfig, AngularRateEstimator, LinearVelocityEstimator, OneEuroConfig, OneEuroVec3,
    RateObservation,
};

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
const OMEGA: [f64; 3] = [0.8, -2.5, 1.2];

fn truth(t: f64) -> Quat {
    let start = DQuat::from_rotation_y(0.4) * DQuat::from_rotation_x(-0.3);
    (DQuat::from_scaled_axis(DVec3::from_array(OMEGA) * t) * start).as_quat()
}

struct Lcg(u32);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 8) as f64 / (1 << 24) as f64
    }
}

/// Irregular 6-14 ms sampling with random quaternion sign flips.
fn feed_constant_rate(estimator: &mut AngularRateEstimator, until: f64) -> f64 {
    let mut rng = Lcg(7);
    let mut t = 0.0;
    while t < until {
        let q = truth(t);
        let q = if rng.next() < 0.5 { -q } else { q };
        estimator.observe(t, q);
        t += 0.006 + 0.008 * rng.next();
    }
    estimator.last_sample_at().unwrap()
}

#[test]
fn constant_angular_velocity_is_recovered_within_two_percent() {
    let mut estimator = AngularRateEstimator::default();
    let last = feed_constant_rate(&mut estimator, 0.4);
    let omega = estimator.angular_velocity(last);
    let expected = Vec3::new(OMEGA[0] as f32, OMEGA[1] as f32, OMEGA[2] as f32);
    let relative = (omega - expected).length() / expected.length();
    assert!(relative < 0.02, "omega {omega:?} relative error {relative}");
    assert!((estimator.speed() - expected.length()).abs() / expected.length() < 0.02);
}

#[test]
fn extrapolation_of_constant_rate_lands_within_a_fifth_of_a_degree_at_30ms() {
    let mut estimator = AngularRateEstimator::default();
    let last = feed_constant_rate(&mut estimator, 0.4);
    let predicted = estimator.extrapolate(last, 0.030).unwrap();
    let error = predicted.angle_to(truth(last + 0.030)).to_degrees();
    assert!(error < 0.2, "extrapolation error {error} deg");
    // Without extrapolation the error would be ~5 degrees at this rate.
    assert!(
        estimator
            .extrapolate(last, 0.0)
            .unwrap()
            .angle_to(truth(last + 0.030))
            .to_degrees()
            > 4.0
    );

    // The horizon (age + lead) is bounded to 35 ms.
    let bounded = estimator.extrapolate(last + 0.020, 0.030).unwrap();
    let error = bounded.angle_to(truth(last + 0.035)).to_degrees();
    assert!(error < 0.2, "bounded horizon error {error} deg");
}

#[test]
fn silence_means_still_and_stops_extrapolation() {
    let mut estimator = AngularRateEstimator::default();
    let last = feed_constant_rate(&mut estimator, 0.3);
    let held = truth(last);
    assert!(estimator.angular_velocity(last + 0.059).length() > 1.0);
    assert_eq!(estimator.angular_velocity(last + 0.061), Vec3::ZERO);
    let q = estimator.extrapolate(last + 0.061, 0.030).unwrap();
    assert!(q.angle_to(held) < 5.0e-4);
    assert!(estimator.extrapolate(f64::NAN, 0.03).unwrap().is_finite());
    assert!(
        estimator
            .extrapolate(last, f64::INFINITY)
            .unwrap()
            .is_finite()
    );
}

#[test]
fn quaternion_sign_flips_do_not_create_spurious_rates() {
    let mut estimator = AngularRateEstimator::default();
    let q = Quat::from_rotation_z(0.9) * Quat::from_rotation_x(0.2);
    for k in 0..40 {
        let sample = if k % 2 == 0 { q } else { -q };
        estimator.observe(k as f64 * 0.01, sample);
    }
    assert!(estimator.angular_velocity(0.39).length() < 1.0e-4);
    assert!(estimator.extrapolate(0.39, 0.03).unwrap().angle_to(q) < 1.0e-4);
}

#[test]
fn invalid_samples_and_time_are_ignored() {
    let mut estimator = AngularRateEstimator::default();
    assert_eq!(estimator.extrapolate(0.0, 0.03), None);
    assert_eq!(estimator.angular_velocity(0.0), Vec3::ZERO);
    assert_eq!(
        estimator.observe(0.0, truth(0.0)),
        RateObservation::Restarted
    );
    // A single sample has no rate.
    assert_eq!(estimator.angular_velocity(0.0), Vec3::ZERO);
    assert_eq!(
        estimator.observe(0.01, truth(0.01)),
        RateObservation::RateUpdated
    );
    let before = estimator.angular_velocity(0.01);
    for (t, q) in [
        (0.01, truth(0.5)),
        (0.005, truth(0.5)),
        (f64::NAN, truth(0.5)),
        (0.02, Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
        (0.02, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)),
        (0.02, Quat::from_xyzw(f32::INFINITY, 0.0, 0.0, 1.0)),
    ] {
        assert_eq!(estimator.observe(t, q), RateObservation::Ignored);
    }
    assert_eq!(estimator.angular_velocity(0.01), before);
    // Closer than 2 ms: kept as orientation, not differenced.
    assert_eq!(
        estimator.observe(0.011, truth(0.011)),
        RateObservation::OrientationOnly
    );
    assert_eq!(
        estimator.observe(0.02, truth(0.02)),
        RateObservation::RateUpdated
    );
    // A gap longer than 50 ms restarts the rate: no extrapolation until a new pair.
    assert_eq!(
        estimator.observe(0.2, truth(0.2)),
        RateObservation::Restarted
    );
    assert_eq!(estimator.angular_velocity(0.2), Vec3::ZERO);
    assert!(
        estimator
            .extrapolate(0.2, 0.03)
            .unwrap()
            .angle_to(truth(0.2))
            < 5.0e-4
    );
    // A clock restart (large backwards jump) re-bases the stream.
    assert_eq!(
        estimator.observe(-5.0, truth(0.0)),
        RateObservation::Restarted
    );
    // A glitch rate (> 40 rad/s) restarts instead of being smoothed in.
    assert_eq!(
        estimator.observe(-4.99, Quat::from_rotation_x(1.0) * truth(0.0)),
        RateObservation::Restarted
    );
    estimator.reset();
    assert_eq!(estimator.extrapolate(0.0, 0.0), None);
}

#[test]
fn configurable_extrapolation_bound_is_respected() {
    let mut estimator = AngularRateEstimator::new(AngularRateConfig {
        max_extrapolation_secs: 0.010,
        ..Default::default()
    });
    let last = feed_constant_rate(&mut estimator, 0.3);
    let q = estimator.extrapolate(last, 0.5).unwrap();
    assert!(q.angle_to(truth(last + 0.010)).to_degrees() < 0.1);
}

#[test]
fn linear_velocity_converges_and_rejects_teleports_and_gaps() {
    let mut estimator = LinearVelocityEstimator::default();
    let velocity = Vec3::new(1.2, -0.3, 0.8);
    let origin = Vec3::new(0.5, 1.0, -2.0);
    let mut t = 0.0;
    let mut v = Vec3::ZERO;
    for _ in 0..30 {
        v = estimator.update(t, origin + velocity * t as f32);
        t += 1.0 / 90.0;
    }
    assert!((v - velocity).length() / velocity.length() < 0.01, "{v:?}");

    // Non-advancing time and nonfinite samples change nothing.
    let last_t = t - 1.0 / 90.0;
    let held = estimator.update(last_t, origin + Vec3::splat(9.0));
    assert_eq!(held, v);
    assert_eq!(estimator.update(t, Vec3::new(f32::NAN, 0.0, 0.0)), v);
    assert_eq!(estimator.update(f64::INFINITY, origin), v);

    // A 0.5 m jump in one frame (45 m/s) is a discontinuity: velocity resets.
    let jumped = origin + velocity * t as f32 + Vec3::new(0.5, 0.0, 0.0);
    assert_eq!(estimator.update(t, jumped), Vec3::ZERO);
    let mut after = Vec3::ZERO;
    for k in 1..=30 {
        let tk = t + k as f64 / 90.0;
        after = estimator.update(tk, jumped + velocity * (k as f32 / 90.0));
        assert!(after.length() <= velocity.length() * 1.001);
    }
    assert!((after - velocity).length() / velocity.length() < 0.01);

    // A gap > 0.1 s restarts at zero, and a stale stream reports zero.
    let late = t + 30.0 / 90.0 + 0.2;
    assert_eq!(estimator.update(late, Vec3::ZERO), Vec3::ZERO);
    assert_eq!(estimator.velocity(late + 0.2), Vec3::ZERO);
}

#[test]
fn one_euro_smooths_jitter_when_still_and_follows_motion() {
    let mut filter = OneEuroVec3::default();
    assert_eq!(filter.value(), None);
    assert_eq!(filter.filter(0.0, Vec3::new(f32::NAN, 0.0, 0.0)), None);
    let center = Vec3::new(0.2, 1.1, -0.4);
    let mut rng = Lcg(3);
    let mut raw_spread = 0.0_f32;
    let mut out_spread = 0.0_f32;
    let mut t = 0.0;
    for k in 0..400 {
        let noise = Vec3::new(rng.next() as f32 - 0.5, rng.next() as f32 - 0.5, 0.0) * 0.004;
        let out = filter.filter(t, center + noise).unwrap();
        assert!(out.is_finite());
        if k > 100 {
            raw_spread = raw_spread.max(noise.length());
            out_spread = out_spread.max((out - center).length());
        }
        t += 0.011;
    }
    assert!(
        out_spread < raw_spread * 0.5,
        "{out_spread} vs {raw_spread}"
    );

    // During 1 m/s motion the lag stays small (speed raises the cutoff).
    let start = filter.value().unwrap();
    let mut out = start;
    let mut target = start;
    for k in 1..=60 {
        target = start + Vec3::X * (k as f32 * 0.011);
        out = filter.filter(t + k as f64 * 0.011, target).unwrap();
    }
    assert!(
        (out - target).length() < 0.04,
        "lag {}",
        (out - target).length()
    );

    // Teleport resets to the new sample exactly; time going back is ignored.
    let t2 = t + 61.0 * 0.011;
    let far = target + Vec3::new(2.0, 0.0, 0.0);
    assert_eq!(filter.filter(t2, far), Some(far));
    assert_eq!(filter.filter(t2 - 0.005, center), Some(far));
}

#[test]
fn one_euro_without_motion_reproduces_constant_input() {
    let mut filter = OneEuroVec3::new(OneEuroConfig {
        min_cutoff_hz: f32::NAN,
        ..Default::default()
    });
    let p = Vec3::new(1.0, 2.0, 3.0);
    for k in 0..50 {
        assert_eq!(filter.filter(k as f64 * 0.01, p), Some(p));
    }
}
