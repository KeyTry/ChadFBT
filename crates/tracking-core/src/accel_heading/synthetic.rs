//! Deterministic synthetic streams for the acceleration-heading regressions
//! and example. A toy body of rigid segments driven by sums of sinusoids: it
//! checks the estimator's algebra, gates and failure reporting. It is NOT a
//! validated human-motion, soft-tissue, Wi-Fi or sensor model, and results on
//! it say nothing about accuracy on worn hardware.
//!
//! Frames follow the crate: meters, +Y up, -Z forward, right-handed.

use super::{AccelHeadingEstimator, SampleOutcome, frame_candidate};
use glam::{DQuat, DVec3, Quat, Vec3};
use std::f64::consts::TAU;

/// Translational motion of the body root.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyntheticMotion {
    /// Stepping/walking in place: step-frequency bob with harmonics, half-
    /// frequency lateral sway, slow heading changes.
    Walk,
    /// Sums of incommensurate sinusoids on all axes.
    Dance,
    /// Millimetre sway only.
    Still,
    /// The sensor's segment translates independently of the head on all axes.
    IndependentTorso,
    /// Shared vertical motion, independent horizontal motion.
    IndependentHorizontal,
}

/// What the caller passes as the reference position for a chest sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SyntheticReference {
    /// The headset eye point itself (head rotation contaminates it).
    HeadsetEye,
    /// The neck pivot from the chain model `p_head + R_head·(0, −h, +f)` with a
    /// model error that rotates with the head (meters, head axes).
    NeckModel { offset_error_m: [f64; 3] },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyntheticSensor {
    /// Chest IMU against the headset/neck reference.
    Chest,
    /// Thigh IMU against its forward-kinematics position computed with the
    /// drifting orientation (vertical is yaw-independent; horizontal is not).
    Thigh,
}

#[derive(Clone, Debug)]
pub struct SyntheticScenario {
    pub duration_secs: f64,
    pub motion: SyntheticMotion,
    pub sensor: SyntheticSensor,
    pub reference: SyntheticReference,
    /// Amplitude of slow torso pitch/roll excursions (tilt diversity).
    pub torso_tilt_degrees: f64,
    /// Amplitude of head yaw about the neck (pitch is half of it).
    pub head_rotation_degrees: f64,
    pub head_rotation_hz: f64,
    /// Sensor (quaternion) axes relative to the segment.
    pub mount: Quat,
    /// True `C` index: world = S · C · accel_body.
    pub true_frame: usize,
    /// The supplied orientation is `Ry(error) · S_true`; the expected
    /// correction is `−error`.
    pub yaw_error_degrees: f64,
    /// Arrival delay of IMU packets relative to the reference clock.
    pub imu_lag_secs: f64,
    /// Firmware 0.7.2: packet `k` carries the acceleration of sample `k − 1`.
    pub accel_one_sample_old: bool,
    pub accel_noise_mps2: f64,
    /// Constant bias in raw accelerometer axes.
    pub accel_bias_mps2: [f64; 3],
    pub imu_rate_hz: f64,
    pub reference_rate_hz: f64,
    /// Uniform extra arrival delay in [0, jitter] per IMU packet.
    pub network_jitter_secs: f64,
    pub reference_position_noise_m: f64,
    pub reference_velocity_noise_mps: f64,
    pub provide_reference_velocity: bool,
    pub seed: u64,
}

impl Default for SyntheticScenario {
    fn default() -> Self {
        Self {
            duration_secs: 20.0,
            motion: SyntheticMotion::Dance,
            sensor: SyntheticSensor::Chest,
            reference: SyntheticReference::NeckModel {
                offset_error_m: [0.0, 0.01, 0.02],
            },
            torso_tilt_degrees: 25.0,
            head_rotation_degrees: 10.0,
            head_rotation_hz: 0.5,
            mount: (DQuat::from_rotation_y(20.0_f64.to_radians())
                * DQuat::from_rotation_x(-12.0_f64.to_radians())
                * DQuat::from_rotation_z(7.0_f64.to_radians()))
            .as_quat(),
            true_frame: 13,
            yaw_error_degrees: 25.0,
            imu_lag_secs: 0.02,
            accel_one_sample_old: true,
            accel_noise_mps2: 0.3,
            accel_bias_mps2: [0.0; 3],
            imu_rate_hz: 100.0,
            reference_rate_hz: 200.0,
            network_jitter_secs: 0.004,
            reference_position_noise_m: 0.0003,
            reference_velocity_noise_mps: 0.005,
            provide_reference_velocity: true,
            seed: 1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum SyntheticEvent {
    Reference {
        at_secs: f64,
        position: Vec3,
        velocity: Option<Vec3>,
        angular_speed_rad_per_sec: f32,
    },
    Imu {
        at_secs: f64,
        sensor_to_room: Quat,
        accel_body: Vec3,
        angular_speed_rad_per_sec: f32,
        /// Ground truth, not passed to the estimator.
        true_sensor_to_room: Quat,
    },
}

impl SyntheticEvent {
    pub fn at_secs(&self) -> f64 {
        match self {
            Self::Reference { at_secs, .. } | Self::Imu { at_secs, .. } => *at_secs,
        }
    }

    pub fn push_into(&self, estimator: &mut AccelHeadingEstimator) -> SampleOutcome {
        match *self {
            Self::Reference {
                at_secs,
                position,
                velocity,
                angular_speed_rad_per_sec,
            } => estimator.push_reference(
                at_secs,
                Some(position),
                velocity,
                angular_speed_rad_per_sec,
            ),
            Self::Imu {
                at_secs,
                sensor_to_room,
                accel_body,
                angular_speed_rad_per_sec,
                ..
            } => estimator.push_imu(
                at_secs,
                sensor_to_room,
                accel_body,
                Some(angular_speed_rad_per_sec),
            ),
        }
    }
}

#[derive(Clone, Debug)]
pub struct SyntheticRun {
    /// Sorted by arrival time.
    pub events: Vec<SyntheticEvent>,
    /// The yaw to add to the supplied orientation (`−yaw_error`).
    pub expected_correction_radians: f64,
    pub true_frame: usize,
    /// Nominal acceleration lag: arrival delay + mean jitter + one IMU
    /// interval when the acceleration is one sample old.
    pub nominal_lag_secs: f64,
}

/// xorshift64* seeded through SplitMix64; deterministic across platforms.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        Self((z ^ (z >> 31)) | 1)
    }

    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform().max(1.0e-300);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }

    fn gaussian3(&mut self, sigma: f64) -> DVec3 {
        DVec3::new(self.gaussian(), self.gaussian(), self.gaussian()) * sigma
    }
}

type Terms = &'static [(f64, f64, f64)];

fn sines(t: f64, terms: Terms) -> f64 {
    terms
        .iter()
        .map(|(amplitude, hz, phase)| amplitude * (TAU * hz * t + phase).sin())
        .sum()
}

struct Translation {
    lateral: Terms,
    vertical: Terms,
    forward: Terms,
}

const WALK: Translation = Translation {
    lateral: &[(0.025, 0.9, 0.3), (0.004, 2.7, 1.0)],
    vertical: &[(0.020, 1.8, 0.0), (0.007, 3.6, 0.8), (0.003, 5.4, 2.1)],
    forward: &[(0.012, 1.8, 1.2), (0.005, 3.6, 0.4)],
};
const DANCE: Translation = Translation {
    lateral: &[(0.05, 0.8, 0.1), (0.03, 1.3, 1.7), (0.012, 2.1, 0.5)],
    vertical: &[(0.03, 1.1, 0.9), (0.015, 2.0, 2.2), (0.006, 2.9, 0.3)],
    forward: &[(0.05, 0.7, 2.5), (0.025, 1.7, 0.2), (0.01, 2.5, 1.1)],
};
const OTHER: Translation = Translation {
    lateral: &[(0.05, 0.95, 0.7), (0.03, 1.55, 0.2), (0.012, 2.35, 1.9)],
    vertical: &[(0.03, 1.25, 0.1), (0.015, 2.25, 1.4), (0.006, 3.1, 2.6)],
    forward: &[(0.05, 0.6, 1.3), (0.025, 1.45, 2.8), (0.01, 2.7, 0.6)],
};
const STILL: Translation = Translation {
    lateral: &[(0.004, 0.25, 0.0)],
    vertical: &[(0.001, 0.3, 0.5)],
    forward: &[(0.003, 0.18, 1.0)],
};

struct Body<'a> {
    s: &'a SyntheticScenario,
    tilt: f64,
    head: f64,
}

impl Body<'_> {
    fn heading(&self, t: f64) -> f64 {
        match self.s.motion {
            SyntheticMotion::Walk => 0.5 * (TAU * 0.05 * t).sin(),
            SyntheticMotion::Still => 0.02 * (0.1 * t).sin(),
            _ => 0.6 * (TAU * 0.07 * t + 0.4).sin(),
        }
    }

    fn translation(&self, t: f64, terms: &Translation, vertical: Option<&Translation>) -> DVec3 {
        let local = DVec3::new(sines(t, terms.lateral), 0.0, sines(t, terms.forward));
        let up = sines(t, vertical.unwrap_or(terms).vertical);
        DVec3::new(0.0, 1.0 + up, 0.0) + DQuat::from_rotation_y(self.heading(t)) * local
    }

    /// Root (hip) of the head side of the body.
    fn root(&self, t: f64) -> DVec3 {
        match self.s.motion {
            SyntheticMotion::Walk => self.translation(t, &WALK, None),
            SyntheticMotion::Still => self.translation(t, &STILL, None),
            _ => self.translation(t, &DANCE, None),
        }
    }

    /// Root carrying the sensor's segment.
    fn sensor_root(&self, t: f64) -> DVec3 {
        match self.s.motion {
            SyntheticMotion::IndependentTorso => self.translation(t, &OTHER, None),
            SyntheticMotion::IndependentHorizontal => self.translation(t, &OTHER, Some(&DANCE)),
            _ => self.root(t),
        }
    }

    fn torso(&self, t: f64) -> DQuat {
        let pitch = self.tilt * (0.6 * (TAU * 0.23 * t).sin() + 0.4 * (TAU * 0.11 * t + 1.0).sin());
        let roll = self.tilt * 0.7 * (TAU * 0.17 * t + 2.0).sin();
        DQuat::from_rotation_y(self.heading(t))
            * DQuat::from_rotation_x(pitch)
            * DQuat::from_rotation_z(roll)
    }

    fn head(&self, t: f64) -> DQuat {
        let f = self.s.head_rotation_hz;
        DQuat::from_rotation_y(self.heading(t) + self.head * (TAU * f * t).sin())
            * DQuat::from_rotation_x(0.5 * self.head * (TAU * 1.37 * f * t + 0.5).sin())
    }

    fn neck(&self, t: f64) -> DVec3 {
        self.root(t) + self.torso(t) * DVec3::new(0.0, 0.55, 0.0)
    }

    fn eye(&self, t: f64) -> DVec3 {
        self.neck(t) + self.head(t) * DVec3::new(0.0, 0.10, -0.09)
    }

    fn hip_joint(&self, t: f64) -> DVec3 {
        self.sensor_root(t) + DQuat::from_rotation_y(self.heading(t)) * DVec3::new(0.09, -0.05, 0.0)
    }

    fn thigh(&self, t: f64) -> DQuat {
        let swing = 0.55 * (TAU * 0.9 * t).sin() + 0.2 * (TAU * 0.45 * t + 1.0).sin();
        let abduction = 0.12 * (TAU * 0.37 * t + 0.3).sin();
        DQuat::from_rotation_y(self.heading(t))
            * DQuat::from_rotation_x(swing)
            * DQuat::from_rotation_z(abduction)
    }

    fn segment(&self, t: f64) -> DQuat {
        match self.s.sensor {
            SyntheticSensor::Chest => self.torso(t),
            SyntheticSensor::Thigh => self.thigh(t),
        }
    }

    fn sensor_point(&self, t: f64) -> DVec3 {
        match self.s.sensor {
            SyntheticSensor::Chest => {
                self.sensor_root(t) + self.torso(t) * DVec3::new(0.0, 0.35, -0.12)
            }
            SyntheticSensor::Thigh => {
                self.hip_joint(t) + self.thigh(t) * DVec3::new(0.0, -0.20, -0.07)
            }
        }
    }

    fn reference_point(&self, t: f64, yaw_error: f64) -> DVec3 {
        match (self.s.sensor, self.s.reference) {
            (SyntheticSensor::Thigh, _) => {
                self.hip_joint(t)
                    + DQuat::from_rotation_y(yaw_error)
                        * self.thigh(t)
                        * DVec3::new(0.0, -0.20, -0.07)
            }
            (SyntheticSensor::Chest, SyntheticReference::HeadsetEye) => self.eye(t),
            (SyntheticSensor::Chest, SyntheticReference::NeckModel { offset_error_m }) => {
                // Chain model neck = p_head + R_head·(0, −0.10, +0.09), with an
                // error that rotates with the head.
                self.eye(t)
                    + self.head(t)
                        * (DVec3::new(0.0, -0.10, 0.09) + DVec3::from_array(offset_error_m))
            }
        }
    }

    fn reference_rotation(&self, t: f64) -> DQuat {
        match self.s.sensor {
            SyntheticSensor::Chest => self.head(t),
            SyntheticSensor::Thigh => self.thigh(t),
        }
    }
}

fn derivative(f: impl Fn(f64) -> DVec3, t: f64) -> DVec3 {
    let h = 1.0e-4;
    (f(t + h) - f(t - h)) / (2.0 * h)
}

fn second_derivative(f: impl Fn(f64) -> DVec3, t: f64) -> DVec3 {
    let h = 1.0e-3;
    (f(t + h) - 2.0 * f(t) + f(t - h)) / (h * h)
}

fn angular_speed(f: impl Fn(f64) -> DQuat, t: f64) -> f64 {
    let h = 1.0e-4;
    let delta = f(t + h) * f(t - h).conjugate();
    let angle = 2.0 * delta.w.abs().clamp(0.0, 1.0).acos();
    angle / (2.0 * h)
}

/// Generates both streams in arrival order.
pub fn generate(s: &SyntheticScenario) -> SyntheticRun {
    let body = Body {
        s,
        tilt: s.torso_tilt_degrees.to_radians(),
        head: s.head_rotation_degrees.to_radians(),
    };
    let yaw_error = s.yaw_error_degrees.to_radians();
    let error_rotation = DQuat::from_rotation_y(yaw_error);
    let frame = frame_candidate(s.true_frame)
        .unwrap_or(Quat::IDENTITY)
        .as_dquat();
    let mount = s.mount.as_dquat().normalize();
    let bias = DVec3::from_array(s.accel_bias_mps2);
    let mut rng = Rng::new(s.seed);
    let mut events = Vec::new();

    let reference_count = (s.duration_secs * s.reference_rate_hz).floor() as usize;
    for j in 0..=reference_count {
        let t = j as f64 / s.reference_rate_hz;
        let point = |time: f64| body.reference_point(time, yaw_error);
        let position = point(t) + rng.gaussian3(s.reference_position_noise_m);
        let velocity = s
            .provide_reference_velocity
            .then(|| derivative(point, t) + rng.gaussian3(s.reference_velocity_noise_mps));
        events.push(SyntheticEvent::Reference {
            at_secs: t,
            position: position.as_vec3(),
            velocity: velocity.map(|v| v.as_vec3()),
            angular_speed_rad_per_sec: angular_speed(|time| body.reference_rotation(time), t)
                as f32,
        });
    }

    let imu_count = (s.duration_secs * s.imu_rate_hz).floor() as usize;
    let mut previous_body_accel: Option<DVec3> = None;
    let mut last_arrival = f64::NEG_INFINITY;
    for k in 0..=imu_count {
        let tau = k as f64 / s.imu_rate_hz + (rng.uniform() - 0.5) * 0.001;
        let true_rotation = body.segment(tau) * mount;
        let world_accel = second_derivative(|time| body.sensor_point(time), tau);
        let body_accel = frame.conjugate() * (true_rotation.conjugate() * world_accel);
        let carried = if s.accel_one_sample_old {
            previous_body_accel.unwrap_or(body_accel)
        } else {
            body_accel
        };
        previous_body_accel = Some(body_accel);
        let measured = carried + bias + rng.gaussian3(s.accel_noise_mps2);
        let arrival = (tau.max(0.0) + s.imu_lag_secs + rng.uniform() * s.network_jitter_secs)
            .max(last_arrival + 1.0e-4);
        last_arrival = arrival;
        events.push(SyntheticEvent::Imu {
            at_secs: arrival,
            sensor_to_room: (error_rotation * true_rotation).normalize().as_quat(),
            accel_body: measured.as_vec3(),
            angular_speed_rad_per_sec: angular_speed(|time| body.segment(time), tau) as f32,
            true_sensor_to_room: true_rotation.normalize().as_quat(),
        });
    }
    events.sort_by(|a, b| a.at_secs().total_cmp(&b.at_secs()));
    SyntheticRun {
        events,
        expected_correction_radians: -yaw_error,
        true_frame: s.true_frame,
        nominal_lag_secs: s.imu_lag_secs
            + 0.5 * s.network_jitter_secs
            + if s.accel_one_sample_old {
                1.0 / s.imu_rate_hz
            } else {
                0.0
            },
    }
}
