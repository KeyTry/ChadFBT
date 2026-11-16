//! Angular-rate estimation, bounded orientation extrapolation and output
//! velocities (stance-anchored estimator, section 5).
//!
//! Everything here is per stream and allocation-free. Invalid samples (nonfinite
//! values, unnormalizable quaternions, non-increasing time) never reach the state,
//! and no query returns a nonfinite value.
//!
//! The firmware suppresses packets while a rotation is unchanged, so silence is
//! read as "still": after `silence_secs` without a sample the angular velocity is
//! zero and extrapolation returns the last orientation unchanged.

use glam::{DQuat, DVec3, Quat, Vec3};
use serde::{Deserialize, Serialize};

/// A backwards timestamp jump larger than this is treated as a restarted clock
/// (the stream is reset); smaller ones are out-of-order samples and ignored.
const CLOCK_RESTART_SECS: f64 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AngularRateConfig {
    /// Consecutive samples closer than this are not differenced (the older
    /// reference is kept until enough time has passed).
    pub min_dt_secs: f64,
    /// Samples further apart than this are not differenced; the rate restarts.
    pub max_dt_secs: f64,
    /// EMA time constant of the angular velocity.
    pub smoothing_tau_secs: f64,
    /// No sample for longer than this means "still": zero rate, no extrapolation.
    pub silence_secs: f64,
    /// Upper bound of `now - t_last + lead` used for extrapolation.
    pub max_extrapolation_secs: f64,
    /// Not in the spec: a raw rate above this is a glitch or discontinuity (the
    /// BMI-class gyros used saturate near 35 rad/s); the rate restarts.
    pub max_rate_rad_per_sec: f64,
}

impl Default for AngularRateConfig {
    fn default() -> Self {
        Self {
            min_dt_secs: 0.002,
            max_dt_secs: 0.050,
            smoothing_tau_secs: 0.025,
            silence_secs: 0.060,
            max_extrapolation_secs: 0.035,
            max_rate_rad_per_sec: 40.0,
        }
    }
}

impl AngularRateConfig {
    fn sanitized(self) -> Self {
        let d = Self::default();
        let nonneg = |v: f64, def: f64| if v.is_finite() && v >= 0.0 { v } else { def };
        let positive = |v: f64, def: f64| if v.is_finite() && v > 0.0 { v } else { def };
        let mut c = Self {
            min_dt_secs: positive(self.min_dt_secs, d.min_dt_secs),
            max_dt_secs: positive(self.max_dt_secs, d.max_dt_secs),
            smoothing_tau_secs: nonneg(self.smoothing_tau_secs, d.smoothing_tau_secs),
            silence_secs: nonneg(self.silence_secs, d.silence_secs),
            max_extrapolation_secs: nonneg(self.max_extrapolation_secs, d.max_extrapolation_secs),
            max_rate_rad_per_sec: positive(self.max_rate_rad_per_sec, d.max_rate_rad_per_sec),
        };
        if c.max_dt_secs < c.min_dt_secs {
            c.min_dt_secs = d.min_dt_secs;
            c.max_dt_secs = d.max_dt_secs;
        }
        c
    }
}

/// Outcome of [`AngularRateEstimator::observe`], for diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RateObservation {
    /// The sample updated the angular velocity.
    RateUpdated,
    /// Stored as the latest orientation; too close to the rate reference.
    OrientationOnly,
    /// Stored; the rate restarted (first sample, gap, glitch or clock restart).
    Restarted,
    /// Rejected: nonfinite, unnormalizable or non-increasing time.
    Ignored,
}

/// Per-sensor angular rate: `Ï‰ = 2Â·log(q_kÂ·q_refâ»Â¹)/Î”t` (shortest arc), EMA-smoothed.
///
/// `q` maps sensor axes to a reference frame; `Ï‰` is expressed in that reference
/// frame (left increment), so extrapolation is `exp(Ï‰Â·Î”)Â·q_last`.
#[derive(Clone, Debug)]
pub struct AngularRateEstimator {
    config: AngularRateConfig,
    last: Option<(f64, DQuat)>,
    reference: Option<(f64, DQuat)>,
    omega: DVec3,
    rate_valid: bool,
}

impl Default for AngularRateEstimator {
    fn default() -> Self {
        Self::new(AngularRateConfig::default())
    }
}

impl AngularRateEstimator {
    pub fn new(config: AngularRateConfig) -> Self {
        Self {
            config: config.sanitized(),
            last: None,
            reference: None,
            omega: DVec3::ZERO,
            rate_valid: false,
        }
    }

    pub fn config(&self) -> &AngularRateConfig {
        &self.config
    }

    pub fn reset(&mut self) {
        self.last = None;
        self.reference = None;
        self.omega = DVec3::ZERO;
        self.rate_valid = false;
    }

    pub fn observe(&mut self, at_secs: f64, q: Quat) -> RateObservation {
        let q = q.as_dquat();
        let length = q.length();
        if !at_secs.is_finite() || !q.is_finite() || !(1.0e-3..=1.0e3).contains(&length) {
            return RateObservation::Ignored;
        }
        let q = q / length;
        if let Some((last_at, _)) = self.last
            && at_secs <= last_at
        {
            if last_at - at_secs <= CLOCK_RESTART_SECS {
                return RateObservation::Ignored;
            }
            self.reset();
        }
        self.last = Some((at_secs, q));
        let Some((ref_at, ref_q)) = self.reference else {
            return self.restart(at_secs, q);
        };
        let dt = at_secs - ref_at;
        if dt < self.config.min_dt_secs {
            return RateObservation::OrientationOnly;
        }
        if dt > self.config.max_dt_secs {
            return self.restart(at_secs, q);
        }
        let raw = rotation_vector(q * ref_q.inverse()) / dt;
        if !raw.is_finite() || raw.length() > self.config.max_rate_rad_per_sec {
            return self.restart(at_secs, q);
        }
        if self.rate_valid {
            let alpha = if self.config.smoothing_tau_secs > 0.0 {
                1.0 - (-dt / self.config.smoothing_tau_secs).exp()
            } else {
                1.0
            };
            self.omega += (raw - self.omega) * alpha;
        } else {
            self.omega = raw;
            self.rate_valid = true;
        }
        self.reference = Some((at_secs, q));
        RateObservation::RateUpdated
    }

    fn restart(&mut self, at_secs: f64, q: DQuat) -> RateObservation {
        self.reference = Some((at_secs, q));
        self.omega = DVec3::ZERO;
        self.rate_valid = false;
        RateObservation::Restarted
    }

    fn live(&self, now: f64) -> Option<(f64, DQuat)> {
        let (at, q) = self.last?;
        let age = now - at;
        (self.rate_valid && now.is_finite() && age <= self.config.silence_secs).then_some((at, q))
    }

    /// Angular velocity (rad/s, reference frame). Zero without two usable samples
    /// or when the last sample is older than the silence cutoff.
    pub fn angular_velocity(&self, now: f64) -> Vec3 {
        match self.live(now) {
            Some(_) => self.omega.as_vec3(),
            None => Vec3::ZERO,
        }
    }

    /// The last orientation advanced by `Ï‰Â·min(now - t_last + lead, max)`.
    /// `None` before the first valid sample; the last orientation unchanged when
    /// silent or without a rate. Negative ages and leads count as zero.
    pub fn extrapolate(&self, now: f64, lead_secs: f64) -> Option<Quat> {
        let (at, q) = self.last?;
        let Some(_) = self.live(now) else {
            return Some(q.as_quat());
        };
        let lead = if lead_secs.is_finite() {
            lead_secs.max(0.0)
        } else {
            0.0
        };
        let horizon = ((now - at).max(0.0) + lead).min(self.config.max_extrapolation_secs);
        let advanced = (DQuat::from_scaled_axis(self.omega * horizon) * q).normalize();
        Some(if advanced.is_finite() {
            advanced.as_quat()
        } else {
            q.as_quat()
        })
    }

    /// Magnitude of the current smoothed rate (rad/s), ignoring silence.
    pub fn speed(&self) -> f32 {
        if self.rate_valid {
            self.omega.length() as f32
        } else {
            0.0
        }
    }

    /// Timestamp of the latest accepted sample.
    pub fn last_sample_at(&self) -> Option<f64> {
        self.last.map(|(at, _)| at)
    }
}

/// `2Â·log(q)` on the shortest arc: axis times angle.
fn rotation_vector(q: DQuat) -> DVec3 {
    let q = if q.w < 0.0 { -q } else { q };
    let v = DVec3::new(q.x, q.y, q.z);
    let s = v.length();
    if s < 1.0e-15 {
        return DVec3::ZERO;
    }
    v / s * (2.0 * s.atan2(q.w))
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StreamGuardConfig {
    /// A larger time gap restarts the stream.
    pub max_gap_secs: f64,
    /// Raw speed above this is a discontinuity (teleport, recenter), not motion.
    pub max_speed_m_per_sec: f32,
    /// Samples closer than this are ignored (the older sample stays the baseline).
    pub min_dt_secs: f64,
}

impl Default for StreamGuardConfig {
    fn default() -> Self {
        Self {
            max_gap_secs: 0.1,
            max_speed_m_per_sec: 6.0,
            min_dt_secs: 1.0e-4,
        }
    }
}

impl StreamGuardConfig {
    fn sanitized(self) -> Self {
        let d = Self::default();
        Self {
            max_gap_secs: if self.max_gap_secs.is_finite() && self.max_gap_secs > 0.0 {
                self.max_gap_secs
            } else {
                d.max_gap_secs
            },
            max_speed_m_per_sec: if self.max_speed_m_per_sec.is_finite()
                && self.max_speed_m_per_sec > 0.0
            {
                self.max_speed_m_per_sec
            } else {
                d.max_speed_m_per_sec
            },
            min_dt_secs: if self.min_dt_secs.is_finite() && self.min_dt_secs > 0.0 {
                self.min_dt_secs
            } else {
                d.min_dt_secs
            },
        }
    }
}

/// How a position sample relates to the previous one.
#[derive(Clone, Copy, Debug)]
enum Step {
    Ignore,
    Restart,
    Advance { dt: f64, raw_velocity: Vec3 },
}

fn classify(guard: &StreamGuardConfig, last: Option<(f64, Vec3)>, at: f64, p: Vec3) -> Step {
    if !at.is_finite() || !p.is_finite() {
        return Step::Ignore;
    }
    let Some((last_at, last_p)) = last else {
        return Step::Restart;
    };
    let dt = at - last_at;
    if dt < -CLOCK_RESTART_SECS {
        return Step::Restart;
    }
    if dt < guard.min_dt_secs {
        return Step::Ignore;
    }
    if dt > guard.max_gap_secs {
        return Step::Restart;
    }
    let raw_velocity = (p - last_p) / dt as f32;
    if !raw_velocity.is_finite() || raw_velocity.length() > guard.max_speed_m_per_sec {
        return Step::Restart;
    }
    Step::Advance { dt, raw_velocity }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LinearVelocityConfig {
    /// EMA time constant of the finite-difference velocity.
    pub smoothing_tau_secs: f64,
    pub guard: StreamGuardConfig,
}

impl Default for LinearVelocityConfig {
    fn default() -> Self {
        Self {
            smoothing_tau_secs: 0.030,
            guard: StreamGuardConfig::default(),
        }
    }
}

/// EMA of finite-difference velocity of a position stream (m/s).
///
/// A gap, a raw speed above the teleport limit, or a restarted clock re-bases the
/// stream at zero velocity. Out-of-order or same-time samples are ignored.
#[derive(Clone, Debug)]
pub struct LinearVelocityEstimator {
    config: LinearVelocityConfig,
    last: Option<(f64, Vec3)>,
    velocity: Vec3,
}

impl Default for LinearVelocityEstimator {
    fn default() -> Self {
        Self::new(LinearVelocityConfig::default())
    }
}

impl LinearVelocityEstimator {
    pub fn new(config: LinearVelocityConfig) -> Self {
        let smoothing = if config.smoothing_tau_secs.is_finite() && config.smoothing_tau_secs >= 0.0
        {
            config.smoothing_tau_secs
        } else {
            LinearVelocityConfig::default().smoothing_tau_secs
        };
        Self {
            config: LinearVelocityConfig {
                smoothing_tau_secs: smoothing,
                guard: config.guard.sanitized(),
            },
            last: None,
            velocity: Vec3::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.last = None;
        self.velocity = Vec3::ZERO;
    }

    /// Feed a sample; returns the current velocity estimate (always finite).
    pub fn update(&mut self, at_secs: f64, position: Vec3) -> Vec3 {
        match classify(&self.config.guard, self.last, at_secs, position) {
            Step::Ignore => {}
            Step::Restart => {
                self.last = Some((at_secs, position));
                self.velocity = Vec3::ZERO;
            }
            Step::Advance { dt, raw_velocity } => {
                let alpha = if self.config.smoothing_tau_secs > 0.0 {
                    1.0 - (-dt / self.config.smoothing_tau_secs).exp()
                } else {
                    1.0
                } as f32;
                self.velocity += (raw_velocity - self.velocity) * alpha;
                self.last = Some((at_secs, position));
            }
        }
        self.velocity
    }

    /// Velocity, or zero when the stream has been silent longer than the gap limit.
    pub fn velocity(&self, now: f64) -> Vec3 {
        match self.last {
            Some((at, _)) if now.is_finite() && now - at <= self.config.guard.max_gap_secs => {
                self.velocity
            }
            _ => Vec3::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OneEuroConfig {
    /// Cutoff (Hz) at zero speed: lower removes more jitter when still.
    pub min_cutoff_hz: f32,
    /// Cutoff increase (Hz) per m/s of filtered speed: higher reduces lag in motion.
    pub beta: f32,
    /// Cutoff (Hz) of the derivative filter.
    pub d_cutoff_hz: f32,
    pub guard: StreamGuardConfig,
}

impl Default for OneEuroConfig {
    /// Untuned starting points for meter-scale tracker positions.
    fn default() -> Self {
        Self {
            min_cutoff_hz: 1.5,
            beta: 4.0,
            d_cutoff_hz: 1.0,
            guard: StreamGuardConfig::default(),
        }
    }
}

/// 1â‚¬ filter on a 3D position, with a shared, speed-adaptive cutoff.
///
/// Casiez, Roussel and Vogel, "1â‚¬ Filter: A Simple Speed-based Low-pass Filter for
/// Noisy Input in Interactive Systems", Proc. CHI 2012, pp. 2527-2530,
/// doi:10.1145/2207676.2208639. The speed is the norm of the filtered derivative,
/// so the cutoff is the same on every axis. Gaps, teleports and restarted clocks
/// re-initialize the filter at the new sample.
#[derive(Clone, Debug)]
pub struct OneEuroVec3 {
    config: OneEuroConfig,
    last: Option<(f64, Vec3)>,
    filtered: Vec3,
    derivative: Vec3,
}

impl Default for OneEuroVec3 {
    fn default() -> Self {
        Self::new(OneEuroConfig::default())
    }
}

impl OneEuroVec3 {
    pub fn new(config: OneEuroConfig) -> Self {
        let d = OneEuroConfig::default();
        let positive = |v: f32, def: f32| if v.is_finite() && v > 0.0 { v } else { def };
        Self {
            config: OneEuroConfig {
                min_cutoff_hz: positive(config.min_cutoff_hz, d.min_cutoff_hz),
                beta: if config.beta.is_finite() && config.beta >= 0.0 {
                    config.beta
                } else {
                    d.beta
                },
                d_cutoff_hz: positive(config.d_cutoff_hz, d.d_cutoff_hz),
                guard: config.guard.sanitized(),
            },
            last: None,
            filtered: Vec3::ZERO,
            derivative: Vec3::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.last = None;
        self.filtered = Vec3::ZERO;
        self.derivative = Vec3::ZERO;
    }

    /// Filter one sample. `None` until a valid sample has been seen; ignored
    /// samples return the previous output.
    pub fn filter(&mut self, at_secs: f64, x: Vec3) -> Option<Vec3> {
        match classify(&self.config.guard, self.last, at_secs, x) {
            Step::Ignore => {}
            Step::Restart => {
                self.last = Some((at_secs, x));
                self.filtered = x;
                self.derivative = Vec3::ZERO;
            }
            Step::Advance { dt, .. } => {
                let dt = dt as f32;
                let dx = (x - self.filtered) / dt;
                self.derivative += (dx - self.derivative) * smoothing(dt, self.config.d_cutoff_hz);
                let cutoff =
                    self.config.min_cutoff_hz + self.config.beta * self.derivative.length();
                self.filtered += (x - self.filtered) * smoothing(dt, cutoff);
                self.last = Some((at_secs, x));
            }
        }
        self.last.map(|_| self.filtered)
    }

    pub fn value(&self) -> Option<Vec3> {
        self.last.map(|_| self.filtered)
    }
}

/// 1â‚¬ smoothing factor `Î± = 1 / (1 + Ï„/Te)`, `Ï„ = 1 / (2Ï€Â·fc)`.
fn smoothing(dt: f32, cutoff_hz: f32) -> f32 {
    let tau = 1.0 / (std::f32::consts::TAU * cutoff_hz);
    1.0 / (1.0 + tau / dt)
}
