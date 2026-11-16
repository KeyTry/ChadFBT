//! Shared synthetic support for the motion-calibration tests and example.
//!
//! **Test support only.** Truth comes either from `tracking_core::synthetic::BodyGenerator`
//! (squats, bows, walking, turns; IK-consistent legs) or from the compact parametric
//! generator below (bows, knee lifts, held skiing pose). Modifiers then add whole-body yaw
//! (wobble or a turn), knees-out flexion planes, sideways torso bends, bone twist and a
//! head turned relative to the torso. A `Rig` turns true bone rotations into sensor
//! quaternions `q_i = Ry(−psi_i)·B·M_i⁻¹` with arbitrary heading offsets and mountings.
//! Everything is generated with the estimator's own assumptions (rigid segments, upright
//! neutral), so passing runs show agreement with the model, not with a worn body.

#![allow(dead_code)]

use glam::{Quat, Vec3};
use tracking_core::CoreError;
use tracking_core::chain::ChainDimensions;
use tracking_core::motion_calibration::{
    MotionCalibrationConfig, MotionCalibrationSolution, MotionCalibrationStatus, MotionCalibrator,
    SampleOutcome,
};
use tracking_core::synthetic::{BodyGenerator, BodyGeometry, Motion, StandState, XorShift64};

pub const HEAD_TO_NECK: f32 = 0.15;
pub const EYE_FORWARD: f32 = 0.08;
const CHEST_TO_HIP: f32 = 0.38;
const NECK_TO_CHEST: f32 = 0.20;
const THIGH: f32 = 0.42;
const CALF: f32 = 0.42;
const HALF_HIP: f32 = 0.15;

#[derive(Clone, Copy, Debug)]
pub struct Truth {
    /// True bone rotations in `Role::CORE` order.
    pub bones: [Quat; 6],
    pub head_position: Vec3,
    pub head_rotation: Quat,
}

pub trait Source {
    fn truth(&self, at: f64) -> Truth;
    fn duration(&self) -> f64;
    /// True body heading at neutral.
    fn body_heading(&self) -> f32;
}

pub fn wrap_degrees(value: f32) -> f32 {
    let wrapped = (value + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped.is_finite() {
        wrapped
    } else {
        f32::NAN
    }
}

pub fn yaw_of(q: Quat) -> f32 {
    let forward = q * Vec3::NEG_Z;
    (-forward.x).atan2(-forward.z)
}

fn head_lever(rotation: Quat) -> Vec3 {
    -(rotation * Vec3::Y) * HEAD_TO_NECK + (rotation * Vec3::Z) * EYE_FORWARD
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

// ---------------------------------------------------------------------------------------------
// synthetic.rs adapter.

pub struct Synthetic {
    pub generator: BodyGenerator,
    pub heading: f32,
}

impl Synthetic {
    /// Feet point straight ahead (`toe_out = 0`) unless `toe_out_degrees` is given: the
    /// generator's knee pole follows the foot, so toe-out rotates the legs' flexion plane.
    pub fn new(script: Vec<Motion>, heading: f32) -> Self {
        Self::with_toe_out(script, heading, 0.0)
    }

    pub fn with_toe_out(script: Vec<Motion>, heading: f32, toe_out_degrees: f32) -> Self {
        let geometry = BodyGeometry {
            toe_out_degrees,
            ..BodyGeometry::default()
        };
        let generator = BodyGenerator::new(
            ChainDimensions::default(),
            geometry,
            StandState {
                center: Vec3::ZERO,
                heading,
            },
            &script,
        );
        Self { generator, heading }
    }
}

impl Source for Synthetic {
    fn truth(&self, at: f64) -> Truth {
        let frame = self.generator.frame(at);
        Truth {
            bones: frame.rotations,
            head_position: frame.head_position,
            head_rotation: frame.head_rotation,
        }
    }
    fn duration(&self) -> f64 {
        self.generator.duration()
    }
    fn body_heading(&self) -> f32 {
        self.heading
    }
}

// ---------------------------------------------------------------------------------------------
// Compact parametric generator. Tilts are "top forward" angles in the body's sagittal plane:
// bone = Ry(h)·Rx(−tilt). Thighs in knee lifts have negative tilt (top backward).

#[derive(Clone, Copy, Debug)]
pub enum Movement {
    Stand,
    /// Repeated bows: chest and pelvis tip forward, hips move back (thighs tip back).
    Bow {
        chest_degrees: f32,
        pelvis_degrees: f32,
        hips_back_m: f32,
        period: f64,
    },
    /// Alternating knee lifts: thigh flexion and knee flexion (calf tips forward).
    March {
        thigh_degrees: f32,
        knee_degrees: f32,
        period: f64,
    },
    /// Ramp into a fixed pose (top-forward tilts per segment), then hold it. `twist_degrees`
    /// ramps in with the pose about every bone's long axis.
    Hold {
        ramp_secs: f64,
        tilts_degrees: [f32; 6],
        twist_degrees: f32,
    },
    /// Arbitrary tilts as a function of phase-local time.
    Custom {
        tilts: fn(f64) -> [f32; 6],
    },
}

#[derive(Clone, Copy, Debug)]
pub struct Phase {
    pub movement: Movement,
    pub secs: f64,
}

pub struct Parametric {
    pub heading: f32,
    pub phases: Vec<Phase>,
}

/// Skiing-pose tilts used by the existing two-pose regression (top forward positive).
pub const SKI_TILTS: [f32; 6] = [28.0, 22.0, -38.0, -34.0, 18.0, 20.0];

impl Parametric {
    pub fn new(heading: f32, phases: Vec<Phase>) -> Self {
        Self { heading, phases }
    }

    fn tilts(&self, at: f64) -> ([f32; 6], f32, Option<usize>) {
        let mut start = 0.0;
        for phase in &self.phases {
            if at < start + phase.secs {
                return movement_tilts(phase.movement, at - start, phase.secs);
            }
            start += phase.secs;
        }
        match self.phases.last() {
            Some(Phase {
                movement: movement @ (Movement::Hold { .. } | Movement::Custom { .. }),
                secs,
            }) => movement_tilts(*movement, *secs, *secs),
            _ => ([0.0; 6], 0.0, None),
        }
    }
}

fn movement_tilts(movement: Movement, t: f64, secs: f64) -> ([f32; 6], f32, Option<usize>) {
    let bump = |period: f64| 0.5 * (1.0 - (std::f64::consts::TAU * t / period).cos());
    match movement {
        Movement::Stand => ([0.0; 6], 0.0, None),
        Movement::Bow {
            chest_degrees,
            pelvis_degrees,
            hips_back_m,
            period,
        } => {
            let phi = bump(period) as f32;
            let thigh = -(hips_back_m * phi / THIGH)
                .clamp(-1.0, 1.0)
                .asin()
                .to_degrees();
            (
                [
                    chest_degrees * phi,
                    pelvis_degrees * phi,
                    thigh,
                    thigh,
                    0.0,
                    0.0,
                ],
                0.0,
                None,
            )
        }
        Movement::March {
            thigh_degrees,
            knee_degrees,
            period,
        } => {
            let u = (t / period).fract();
            let lift = |offset: f64| {
                let v = u - offset;
                if (0.0..0.5).contains(&v) {
                    (std::f64::consts::TAU * v).sin().powi(2) as f32
                } else {
                    0.0
                }
            };
            let (left, right) = (lift(0.0), lift(0.5));
            let thigh = |phi: f32| -thigh_degrees * phi;
            let calf = |phi: f32| (knee_degrees - thigh_degrees) * phi;
            let stance = if left > right { 1 } else { 0 };
            (
                [0.0, 0.0, thigh(left), thigh(right), calf(left), calf(right)],
                0.0,
                Some(stance),
            )
        }
        Movement::Hold {
            ramp_secs,
            tilts_degrees,
            twist_degrees,
        } => {
            let s = smoothstep(t / ramp_secs.max(1.0e-6)) as f32;
            let _ = secs;
            (tilts_degrees.map(|v| v * s), twist_degrees * s, None)
        }
        Movement::Custom { tilts } => (tilts(t), 0.0, None),
    }
}

/// Builds rotations and a consistent head pose (feet planted at hip width under the hips).
fn assemble(
    heading: f32,
    tilts_degrees: [f32; 6],
    twist_degrees: f32,
    stance: Option<usize>,
    head_pitch: f32,
) -> Truth {
    let yaw = Quat::from_rotation_y(heading);
    let twist = Quat::from_rotation_y(twist_degrees.to_radians());
    let bones = tilts_degrees.map(|t| yaw * Quat::from_rotation_x(-t.to_radians()) * twist);
    let pelvis = pelvis_position(heading, &bones, stance);
    let head_rotation = yaw * Quat::from_rotation_x(-head_pitch);
    let neck = neck_position(pelvis, bones[0], bones[1]);
    Truth {
        bones,
        head_position: neck - head_lever(head_rotation),
        head_rotation,
    }
}

fn pelvis_position(heading: f32, bones: &[Quat; 6], stance: Option<usize>) -> Vec3 {
    let yaw = Quat::from_rotation_y(heading);
    let hip_for = |leg: usize| {
        let side = if leg == 0 { -1.0 } else { 1.0 };
        let ankle = yaw * Vec3::new(side * HALF_HIP, 0.08, 0.0);
        let knee = ankle + bones[4 + leg] * Vec3::Y * CALF;
        let joint = knee + bones[2 + leg] * Vec3::Y * THIGH;
        joint - bones[1] * Vec3::X * (side * HALF_HIP)
    };
    match stance {
        Some(leg) => hip_for(leg.min(1)),
        None => 0.5 * (hip_for(0) + hip_for(1)),
    }
}

fn neck_position(pelvis: Vec3, chest: Quat, hip: Quat) -> Vec3 {
    let spine = chest.slerp(hip, 0.5).normalize();
    pelvis + spine * Vec3::Y * CHEST_TO_HIP + chest * Vec3::Y * NECK_TO_CHEST
}

impl Source for Parametric {
    fn truth(&self, at: f64) -> Truth {
        let (tilts, twist, stance) = self.tilts(at);
        assemble(
            self.heading,
            tilts,
            twist,
            stance,
            0.6 * tilts[0].to_radians(),
        )
    }
    fn duration(&self) -> f64 {
        self.phases.iter().map(|p| p.secs).sum()
    }
    fn body_heading(&self) -> f32 {
        self.heading
    }
}

pub fn stand(secs: f64) -> Phase {
    Phase {
        movement: Movement::Stand,
        secs,
    }
}

pub fn bows(count: u32, chest_degrees: f32, pelvis_degrees: f32) -> Phase {
    Phase {
        movement: Movement::Bow {
            chest_degrees,
            pelvis_degrees,
            hips_back_m: 0.08,
            period: 4.0,
        },
        secs: 4.0 * f64::from(count),
    }
}

pub fn march(count: u32) -> Phase {
    Phase {
        movement: Movement::March {
            thigh_degrees: 60.0,
            knee_degrees: 80.0,
            period: 1.6,
        },
        secs: 1.6 * f64::from(count),
    }
}

pub fn hold(tilts_degrees: [f32; 6], secs: f64) -> Phase {
    hold_twisted(tilts_degrees, 0.0, secs)
}

pub fn hold_twisted(tilts_degrees: [f32; 6], twist_degrees: f32, secs: f64) -> Phase {
    Phase {
        movement: Movement::Hold {
            ramp_secs: 1.0,
            tilts_degrees,
            twist_degrees,
        },
        secs,
    }
}

// ---------------------------------------------------------------------------------------------
// Modifiers.

#[derive(Clone, Copy, Debug, Default)]
pub struct Mods {
    /// Whole-body yaw wobble amplitude and period.
    pub wobble_degrees: f32,
    pub wobble_period: f64,
    /// Whole-body turn completed smoothly over `turn_secs` starting at `turn_at`.
    pub turn_degrees: f32,
    pub turn_at: f64,
    pub turn_secs: f64,
    /// Flexion planes rotated outward (left leg +, right leg −) by conjugation.
    pub knees_out_degrees: f32,
    /// Apply knees-out to thighs only (calves stay sagittal).
    pub knees_out_thigh_only: bool,
    /// Rotates the torso's bend direction about vertical (90 = pure sideways bend).
    pub torso_bend_yaw_degrees: f32,
    /// Apply the torso bend rotation only before this time (a bad stretch).
    pub torso_bend_until: Option<f64>,
    /// Twist about each bone's long axis (amplitude), varying per segment.
    pub twist_degrees: f32,
    pub twist_period: f64,
    /// Head yaw relative to the torso, at all times (including neutral).
    pub head_yaw_degrees: f32,
}

pub struct Modified<S> {
    pub inner: S,
    pub mods: Mods,
}

impl<S: Source> Source for Modified<S> {
    fn truth(&self, at: f64) -> Truth {
        let truth = self.inner.truth(at);
        let m = self.mods;
        let neck = truth.head_position + head_lever(truth.head_rotation);
        let spine = truth.bones[0].slerp(truth.bones[1], 0.5).normalize();
        let pelvis =
            neck - truth.bones[0] * Vec3::Y * NECK_TO_CHEST - spine * Vec3::Y * CHEST_TO_HIP;
        let mut bones = truth.bones;
        let conjugate = |q: Quat, degrees: f32| {
            let r = Quat::from_rotation_y(degrees.to_radians());
            (r * q * r.conjugate()).normalize()
        };
        if m.knees_out_degrees != 0.0 {
            for (leg, side) in [(0_usize, 1.0_f32), (1, -1.0)] {
                bones[2 + leg] = conjugate(bones[2 + leg], side * m.knees_out_degrees);
                if !m.knees_out_thigh_only {
                    bones[4 + leg] = conjugate(bones[4 + leg], side * m.knees_out_degrees);
                }
            }
        }
        if m.torso_bend_yaw_degrees != 0.0 && m.torso_bend_until.is_none_or(|until| at < until) {
            for bone in bones.iter_mut().take(2) {
                *bone = conjugate(*bone, m.torso_bend_yaw_degrees);
            }
        }
        if m.twist_degrees != 0.0 {
            for (i, bone) in bones.iter_mut().enumerate() {
                let period = m.twist_period.max(0.1) * (1.0 + 0.13 * i as f64);
                let angle = m.twist_degrees.to_radians()
                    * (std::f64::consts::TAU * at / period).sin() as f32;
                *bone = (*bone * Quat::from_rotation_y(angle)).normalize();
            }
        }
        let neck = neck_position(pelvis, bones[0], bones[1]);
        let mut head_rotation =
            Quat::from_rotation_y(m.head_yaw_degrees.to_radians()) * truth.head_rotation;
        let mut yaw = 0.0_f32;
        if m.wobble_degrees != 0.0 && m.wobble_period > 0.0 {
            yaw += m.wobble_degrees.to_radians()
                * (std::f64::consts::TAU * at / m.wobble_period).sin() as f32;
        }
        if m.turn_degrees != 0.0 {
            yaw += m.turn_degrees.to_radians()
                * smoothstep((at - m.turn_at) / m.turn_secs.max(1.0e-6)) as f32;
        }
        let turn = Quat::from_rotation_y(yaw);
        let bones = bones.map(|b| (turn * b).normalize());
        head_rotation = (turn * head_rotation).normalize();
        let neck = turn * neck;
        Truth {
            bones,
            head_position: neck - head_lever(head_rotation),
            head_rotation,
        }
    }
    fn duration(&self) -> f64 {
        self.inner.duration()
    }
    fn body_heading(&self) -> f32 {
        self.inner.body_heading()
    }
}

// ---------------------------------------------------------------------------------------------
// Sensor rig and runner.

#[derive(Clone, Debug)]
pub struct Rig {
    /// True sensor-world heading offsets: `R = Ry(psi)·q·M`.
    pub psi: [f32; 6],
    /// True mounting rotations.
    pub mounts: [Quat; 6],
    /// Per-axis 1σ rotation noise of each sensor sample.
    pub noise_degrees: f32,
    /// Per-axis 1σ headset position noise.
    pub head_noise_m: f32,
    pub seed: u64,
    /// Role stream `i` carries the sensor strapped to bone `source_bone[i]`.
    pub source_bone: [usize; 6],
}

impl Rig {
    pub fn standard(seed: u64) -> Self {
        let mounts = std::array::from_fn(|i| {
            let i = i as f32;
            Quat::from_rotation_x(0.3 + 0.23 * i)
                * Quat::from_rotation_z(-0.7 + 0.31 * i)
                * Quat::from_rotation_y(0.4 + 0.41 * i)
        });
        Self {
            psi: [-1.4, 2.2, 1.570_796_4, -2.7, 0.9, -0.4],
            mounts,
            noise_degrees: 0.3,
            head_noise_m: 0.005,
            seed,
            source_bone: [0, 1, 2, 3, 4, 5],
        }
    }

    pub fn clean(&self, truth: &Truth, i: usize) -> Quat {
        let bone = truth.bones[self.source_bone[i]];
        (Quat::from_rotation_y(-self.psi[i]) * bone * self.mounts[i].conjugate()).normalize()
    }

    fn noisy(&self, truth: &Truth, i: usize, rng: &mut XorShift64) -> Quat {
        let sigma = self.noise_degrees.to_radians();
        let noise = Vec3::new(
            rng.gaussian() as f32,
            rng.gaussian() as f32,
            rng.gaussian() as f32,
        ) * sigma;
        (self.clean(truth, i) * Quat::from_scaled_axis(noise)).normalize()
    }
}

#[derive(Clone, Debug)]
pub struct RunOptions {
    pub rate_hz: f64,
    pub neutral_at: f64,
    pub headset: bool,
    /// Reference heading = headset yaw at neutral (as the app does); otherwise the true
    /// body heading (isolates per-segment errors).
    pub reference_from_head: bool,
    pub config: MotionCalibrationConfig,
    /// Sensor `i` is sampled `i · stagger_secs` after the tick (asynchronous streams).
    pub stagger_secs: f64,
    /// Every `drop_every`-th tick of sensor `drop_role` is lost (0 = never).
    pub drop_every: u64,
    pub drop_role: usize,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            rate_hz: 100.0,
            neutral_at: 0.5,
            headset: false,
            reference_from_head: false,
            config: MotionCalibrationConfig::default(),
            stagger_secs: 0.0,
            drop_every: 0,
            drop_role: 0,
        }
    }
}

pub struct RunResult {
    pub calibrator: MotionCalibrator,
    pub status: MotionCalibrationStatus,
    pub solution: Result<MotionCalibrationSolution, CoreError>,
    pub reference_heading: f32,
    pub body_heading: f32,
    pub psi_true: [f32; 6],
    pub neutral: [Quat; 6],
    pub outcomes: Vec<SampleOutcome>,
    /// First time (checked every 0.25 s) at which the status reported `ready`.
    pub ready_at: Option<f64>,
}

impl RunResult {
    /// Error of each segment's solved body-forward direction in its own sensor world,
    /// degrees (`None` where the sign is undetermined). Independent of the reference.
    pub fn forward_errors_degrees(&self) -> [Option<f32>; 6] {
        std::array::from_fn(|i| {
            self.status.segments[i]
                .sensor_forward_heading_radians
                .map(|h| wrap_degrees((h - (self.body_heading - self.psi_true[i])).to_degrees()))
        })
    }

    /// `wrap(psi_est − psi_true)` from the solution (includes any reference error).
    pub fn psi_errors_degrees(&self) -> Option<[f32; 6]> {
        let solution = self.solution.as_ref().ok()?;
        Some(std::array::from_fn(|i| {
            wrap_degrees(
                (solution.alignments[i].heading_alignment_radians - self.psi_true[i]).to_degrees(),
            )
        }))
    }

    pub fn max_abs_psi_error_degrees(&self) -> Option<f32> {
        self.psi_errors_degrees()
            .map(|e| e.iter().fold(0.0_f32, |m, v| m.max(v.abs())))
    }
}

pub fn run(source: &dyn Source, rig: &Rig, options: &RunOptions) -> RunResult {
    run_until(source, rig, options, source.duration())
}

pub fn run_until(source: &dyn Source, rig: &Rig, options: &RunOptions, end: f64) -> RunResult {
    let mut calibrator = MotionCalibrator::new(options.config.clone()).expect("valid config");
    let t0 = options.neutral_at;
    let neutral_truth = source.truth(t0);
    let neutral: [Quat; 6] = std::array::from_fn(|i| rig.clean(&neutral_truth, i));
    let reference_heading = if options.reference_from_head {
        yaw_of(neutral_truth.head_rotation)
    } else {
        source.body_heading()
    };
    let neck = options.headset.then(|| {
        MotionCalibrator::neck_pivot(
            neutral_truth.head_position,
            neutral_truth.head_rotation,
            HEAD_TO_NECK,
            EYE_FORWARD,
        )
        .expect("finite neck")
    });
    calibrator
        .begin(t0, neutral, reference_heading, neck)
        .expect("begin");
    let mut rng = XorShift64::new(rig.seed);
    let mut outcomes = Vec::new();
    let mut ready_at = None;
    let mut next_check = t0;
    let dt = 1.0 / options.rate_hz;
    let mut k = 1_u64;
    loop {
        let at = t0 + k as f64 * dt;
        if at > end + 1.0e-9 {
            break;
        }
        let truth = source.truth(at);
        for i in 0..6 {
            let sampled_at = at + i as f64 * options.stagger_secs;
            let truth = if options.stagger_secs > 0.0 {
                source.truth(sampled_at)
            } else {
                truth
            };
            let q = rig.noisy(&truth, i, &mut rng);
            if options.drop_every > 0
                && i == options.drop_role
                && k.is_multiple_of(options.drop_every)
            {
                continue;
            }
            outcomes.push(calibrator.observe_sensor(i, sampled_at, q));
        }
        if options.headset {
            let noise = Vec3::new(
                rng.gaussian() as f32,
                rng.gaussian() as f32,
                rng.gaussian() as f32,
            ) * rig.head_noise_m;
            outcomes.push(calibrator.observe_headset(
                at,
                truth.head_position + noise,
                truth.head_rotation,
                HEAD_TO_NECK,
                EYE_FORWARD,
            ));
        }
        if ready_at.is_none() && at >= next_check {
            next_check += 0.25;
            if calibrator.status().ready {
                ready_at = Some(at);
            }
        }
        k += 1;
    }
    let status = calibrator.status();
    let solution = calibrator.solve();
    RunResult {
        calibrator,
        status,
        solution,
        reference_heading,
        body_heading: source.body_heading(),
        psi_true: rig.psi,
        neutral,
        outcomes,
        ready_at,
    }
}

// ---------------------------------------------------------------------------------------------
// Named scenarios shared by tests and the example.

pub const HEADING: f32 = 0.6;

pub fn squats() -> Synthetic {
    Synthetic::new(
        vec![Motion::stand(1.0), Motion::squats(3), Motion::stand(1.0)],
        HEADING,
    )
}

pub fn bow_then_march() -> Parametric {
    Parametric::new(
        HEADING,
        vec![stand(1.0), bows(2, 60.0, 30.0), march(4), stand(1.0)],
    )
}

pub fn held_ski() -> Parametric {
    Parametric::new(HEADING, vec![stand(1.0), hold(SKI_TILTS, 4.0)])
}

pub fn walking() -> Synthetic {
    Synthetic::new(
        vec![Motion::stand(1.0), Motion::walk(12), Motion::stand(1.0)],
        HEADING,
    )
}

/// Bows whose neck pivot moves about 25 cm forward, then knee lifts for the legs.
pub fn headset_bow() -> Parametric {
    Parametric::new(
        HEADING,
        vec![
            stand(1.0),
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
        ],
    )
}
