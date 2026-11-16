//! Deterministic articulated-body generator and closed-loop harness.
//!
//! **Test and benchmark support only.** Nothing here is used by live tracking. The generator
//! builds ground truth *with the estimator's own kinematic model* (same chain topology, same
//! mid-spine slerp, a perfect knee hinge), so passing scenarios shows that the implementation
//! matches the model and degrades gracefully under the injected faults. It does not show that
//! the model matches a human body.
//!
//! Ground truth: scripted pelvis/chest/head motion plus world ankle targets (planted feet are
//! fixed points, swing feet follow trajectories), two-bone IK from each hip joint to its ankle
//! (knee in the plane of the leg's forward pole), bone axes per the chain convention (+Y distal
//! → proximal, +X right / hinge axis, −Z forward), and a headset eye pose derived *upwards*
//! from the body with a configurable true eye offset. The chain identity therefore holds
//! exactly for the true dimensions.
//!
//! Injection hooks: per-sensor yaw error functions of time (measured = `Ry(−e(t))·true`),
//! seeded headset position noise, a headset (tracking-space) jump, bone-length and eye-offset
//! errors between true and assumed dimensions, and stale/NaN faults.

#![allow(clippy::needless_range_loop)] // Small fixed-size per-segment arrays.

use crate::chain::{
    BodyChain, CALF, ChainDimensions, ChainPose, FOOT_CHAIN, LATERAL_SIGN, lateral_heading,
    solve_chain,
};
use crate::contact::{ContactConfig, ContactTracker};
use crate::stance_heading::{
    StanceHeadingConfig, StanceHeadingFilter, StanceHeadingInput, StanceHeadingOutput,
    StanceHeadingStatus,
};
use glam::{Mat3, Quat, Vec3};
use std::f32::consts::{PI, TAU};

// ---------------------------------------------------------------------------------------------
// Deterministic random numbers.

/// Tiny xorshift64* generator; deterministic across platforms. Not cryptographic.
#[derive(Clone, Debug)]
pub struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    pub fn new(seed: u64) -> Self {
        // SplitMix64 scrambling so that small seeds give unrelated streams; never zero.
        let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        Self {
            state: if z == 0 { 0x2545_F491_4F6C_DD1D } else { z },
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in the open interval (0, 1).
    pub fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal sample (Box–Muller).
    pub fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

// ---------------------------------------------------------------------------------------------
// Body geometry and motion script.

/// Geometry of the synthetic body beyond the chain dimensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyGeometry {
    /// Ankle centre height above the floor when the foot is flat.
    pub ankle_height_m: f32,
    /// Lateral offset of each ankle from the body midline in the neutral stance.
    pub foot_half_width_m: f32,
    /// Ball of the foot (pivot point) ahead of the ankle.
    pub ball_forward_m: f32,
    pub toe_out_degrees: f32,
    /// Neutral standing hip-joint-to-ankle distance as a fraction of thigh + calf.
    pub stand_reach: f32,
    /// The pelvis is lowered whenever a leg would need more than this fraction.
    pub max_reach: f32,
    /// Walking pelvis height (hip joint above ankle) as a fraction of thigh + calf.
    pub walk_reach: f32,
}

impl Default for BodyGeometry {
    fn default() -> Self {
        Self {
            ankle_height_m: 0.08,
            foot_half_width_m: 0.12,
            ball_forward_m: 0.14,
            toe_out_degrees: 5.0,
            stand_reach: 0.996,
            max_reach: 0.998,
            walk_reach: 0.95,
        }
    }
}

/// Standing state between script segments: floor point under the pelvis and body heading
/// (`Ry(heading)`; positive turns left).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StandState {
    pub center: Vec3,
    pub heading: f32,
}

impl Default for StandState {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            heading: 0.0,
        }
    }
}

/// One scripted motion. Every motion starts and ends in the neutral stance of its
/// [`StandState`], so motions can be sequenced freely.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    /// Quiet standing with optional sway of the pelvis (amplitude in meters).
    Stand { secs: f64, sway_m: f32 },
    /// Lateral weight shifting with both feet planted.
    WeightShift {
        secs: f64,
        amplitude_m: f32,
        period_secs: f64,
    },
    /// Squats with both feet planted; the torso leans forward with depth.
    Squats {
        count: u32,
        depth_m: f32,
        period_secs: f64,
    },
    /// Straight walking along the heading: a half step to start, full strides, a closing half
    /// step. Stance feet are fixed points except for a heel rise (rotation about the ball of
    /// the foot) at the end of stance.
    Walk {
        steps: u32,
        step_length_m: f32,
        cadence_hz: f64,
        clearance_m: f32,
        heel_rise_degrees: f32,
    },
    /// Turn on the spot: both feet pivot about the balls of the feet for `pivot_fraction` of
    /// the turn, then each foot is lifted and re-planted.
    Turn {
        degrees: f32,
        pivot_fraction: f32,
        pivot_secs: f64,
        replant_secs: f64,
    },
    /// Head-only turns (yaw and pitch) with a small body sway.
    HeadTurns {
        secs: f64,
        yaw_degrees: f32,
        pitch_degrees: f32,
        period_secs: f64,
        body_sway_m: f32,
    },
    /// While squatting, one foot really slides along the floor below the contact speed
    /// thresholds, then is re-planted at its neutral position.
    FootSlide {
        foot: usize,
        speed_mps: f32,
        distance_m: f32,
        /// 0 = outward (lateral, away from the midline), 90 = forward.
        direction_degrees: f32,
        squat_depth_m: f32,
        squat_period_secs: f64,
        settle_secs: f64,
        replant_secs: f64,
    },
    /// Forward torso bows with both feet planted.
    TorsoBows {
        count: u32,
        chest_degrees: f32,
        period_secs: f64,
    },
    /// Two-footed vertical jumps with an upright torso: a crouch, a flight phase in which
    /// both feet leave the floor by `height_m`, and a landing crouch.
    Jumps {
        count: u32,
        height_m: f32,
        period_secs: f64,
    },
    /// Standing on one leg while the other knee is lifted: the lifted ankle rises by
    /// `height_m` and moves forward; the pelvis shifts over the support foot.
    KneeLift {
        foot: usize,
        height_m: f32,
        secs: f64,
    },
}

impl Motion {
    pub fn stand(secs: f64) -> Self {
        Self::Stand { secs, sway_m: 0.0 }
    }

    pub fn weight_shift(secs: f64) -> Self {
        Self::WeightShift {
            secs,
            amplitude_m: 0.07,
            period_secs: 4.0,
        }
    }

    pub fn squats(count: u32) -> Self {
        Self::Squats {
            count,
            depth_m: 0.30,
            period_secs: 3.0,
        }
    }

    /// Default gait: 0.5 m steps at 1.6 steps/s (0.8 m/s), 7 cm clearance, 20° heel rise.
    pub fn walk(steps: u32) -> Self {
        Self::Walk {
            steps,
            step_length_m: 0.5,
            cadence_hz: 1.6,
            clearance_m: 0.07,
            heel_rise_degrees: 20.0,
        }
    }

    /// Default pivot turn: half of the angle pivoted in 0.8 s (peak calf yaw rate below the
    /// 90°/s contact release threshold for 90° turns), then two 0.5 s re-plant steps.
    pub fn turn(degrees: f32) -> Self {
        Self::Turn {
            degrees,
            pivot_fraction: 0.5,
            pivot_secs: 0.8,
            replant_secs: 0.5,
        }
    }

    pub fn head_turns(secs: f64) -> Self {
        Self::HeadTurns {
            secs,
            yaw_degrees: 60.0,
            pitch_degrees: 20.0,
            period_secs: 4.0,
            body_sway_m: 0.02,
        }
    }

    pub fn foot_slide(foot: usize) -> Self {
        Self::FootSlide {
            foot,
            speed_mps: 0.05,
            distance_m: 0.15,
            direction_degrees: 0.0,
            squat_depth_m: 0.20,
            squat_period_secs: 3.0,
            settle_secs: 3.0,
            replant_secs: 0.6,
        }
    }

    pub fn torso_bows(count: u32) -> Self {
        Self::TorsoBows {
            count,
            chest_degrees: 60.0,
            period_secs: 4.0,
        }
    }

    /// 20 cm jumps, one every 1.5 s.
    pub fn jumps(count: u32) -> Self {
        Self::Jumps {
            count,
            height_m: 0.20,
            period_secs: 1.5,
        }
    }

    /// The ankle rises by 30 cm over a 3 s lift.
    pub fn knee_lift(foot: usize) -> Self {
        Self::KneeLift {
            foot,
            height_m: 0.30,
            secs: 3.0,
        }
    }

    pub fn duration(&self) -> f64 {
        let value = match *self {
            Self::Stand { secs, .. }
            | Self::WeightShift { secs, .. }
            | Self::HeadTurns { secs, .. }
            | Self::KneeLift { secs, .. } => secs,
            Self::Squats {
                count, period_secs, ..
            }
            | Self::TorsoBows {
                count, period_secs, ..
            }
            | Self::Jumps {
                count, period_secs, ..
            } => f64::from(count) * period_secs,
            Self::Walk {
                steps, cadence_hz, ..
            } => f64::from(steps.max(2)) / cadence_hz,
            Self::Turn {
                pivot_secs,
                replant_secs,
                ..
            } => pivot_secs + 2.0 * replant_secs,
            Self::FootSlide {
                speed_mps,
                distance_m,
                settle_secs,
                replant_secs,
                ..
            } => 2.0 * settle_secs + slide_secs(distance_m, speed_mps) + replant_secs,
        };
        if value.is_finite() {
            value.max(0.0)
        } else {
            0.0
        }
    }

    fn end_state(&self, start: StandState) -> StandState {
        match *self {
            Self::Walk {
                steps,
                step_length_m,
                ..
            } => StandState {
                center: start.center
                    + forward(start.heading) * ((steps.max(2) - 1) as f32 * step_length_m),
                heading: start.heading,
            },
            Self::Turn { degrees, .. } => StandState {
                center: start.center,
                heading: start.heading + degrees.to_radians(),
            },
            _ => start,
        }
    }
}

fn slide_secs(distance: f32, speed: f32) -> f64 {
    // Smoothstep displacement profile: peak speed = 1.5 · distance / duration.
    if speed > 0.0 {
        f64::from(1.5 * distance / speed)
    } else {
        0.0
    }
}

/// Ground truth at one instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TruthFrame {
    pub at_secs: f64,
    /// True headset eye point and rotation.
    pub head_position: Vec3,
    pub head_rotation: Quat,
    /// True bone rotations in `Role::CORE` order.
    pub rotations: [Quat; 6],
    pub ankles: [Vec3; 2],
    /// The ankle is truly a stationary point (foot flat and not sliding or pivoting).
    pub stationary: [bool; 2],
    /// Distance by which IK could not reach the scripted ankle target (0 when reachable).
    pub ik_shortfall_m: f32,
}

#[derive(Clone, Copy, Debug)]
struct PoseSpec {
    pelvis: Vec3,
    pelvis_rotation: Quat,
    chest_rotation: Quat,
    head_rotation: Quat,
    ankles: [Vec3; 2],
    foot_headings: [f32; 2],
    stationary: [bool; 2],
}

#[derive(Clone, Copy, Debug)]
struct ScriptSegment {
    start_secs: f64,
    motion: Motion,
    state: StandState,
}

/// Deterministic articulated-body generator.
#[derive(Clone, Debug)]
pub struct BodyGenerator {
    dims: ChainDimensions,
    geometry: BodyGeometry,
    segments: Vec<ScriptSegment>,
    end: StandState,
    duration: f64,
}

impl BodyGenerator {
    /// `dims` are the TRUE dimensions (including the true eye offset).
    pub fn new(
        dims: ChainDimensions,
        geometry: BodyGeometry,
        start: StandState,
        script: &[Motion],
    ) -> Self {
        let mut segments = Vec::with_capacity(script.len());
        let mut state = start;
        let mut at = 0.0;
        for motion in script {
            segments.push(ScriptSegment {
                start_secs: at,
                motion: *motion,
                state,
            });
            at += motion.duration();
            state = motion.end_state(state);
        }
        Self {
            dims,
            geometry,
            segments,
            end: state,
            duration: at,
        }
    }

    pub fn duration(&self) -> f64 {
        self.duration
    }

    pub fn frame(&self, at_secs: f64) -> TruthFrame {
        let index = self
            .segments
            .partition_point(|segment| segment.start_secs <= at_secs);
        let spec = match index.checked_sub(1).map(|i| self.segments[i]) {
            Some(segment) if at_secs < segment.start_secs + segment.motion.duration() => {
                self.motion_pose(&segment.motion, segment.state, at_secs - segment.start_secs)
            }
            Some(_) | None if at_secs >= self.duration => self.stand_pose(self.end, Vec3::ZERO),
            _ => {
                let state = self.segments.first().map_or(self.end, |s| s.state);
                self.stand_pose(state, Vec3::ZERO)
            }
        };
        self.assemble(at_secs, spec)
    }

    fn leg_length(&self) -> f32 {
        self.dims.thigh + self.dims.calf
    }

    fn stand_height(&self) -> f32 {
        let reach = self.geometry.stand_reach * self.leg_length();
        let lateral = 0.5 * self.dims.hip_width - self.geometry.foot_half_width_m;
        self.geometry.ankle_height_m + (reach * reach - lateral * lateral).max(0.0).sqrt()
    }

    fn nominal_feet(&self, state: StandState) -> [Vec3; 2] {
        [0, 1].map(|foot| {
            state.center * Vec3::new(1.0, 0.0, 1.0)
                + right(state.heading) * (LATERAL_SIGN[foot] * self.geometry.foot_half_width_m)
                + Vec3::Y * self.geometry.ankle_height_m
        })
    }

    fn foot_heading(&self, heading: f32, foot: usize) -> f32 {
        heading - LATERAL_SIGN[foot] * self.geometry.toe_out_degrees.to_radians()
    }

    fn ball(&self, ankle: Vec3, foot_heading: f32) -> Vec3 {
        let mut ball = ankle + forward(foot_heading) * self.geometry.ball_forward_m;
        ball.y = ankle.y - self.geometry.ankle_height_m;
        ball
    }

    /// Ankle position after the heel has risen by `angle` about the ball of the foot.
    fn heel_rise(&self, rest: Vec3, foot_heading: f32, angle: f32) -> Vec3 {
        let ball = self.ball(rest, foot_heading);
        ball + Quat::from_axis_angle(right(foot_heading), -angle) * (rest - ball)
    }

    fn stand_pose(&self, state: StandState, offset: Vec3) -> PoseSpec {
        let heading = state.heading;
        PoseSpec {
            pelvis: state.center * Vec3::new(1.0, 0.0, 1.0)
                + offset
                + Vec3::Y * self.stand_height(),
            pelvis_rotation: yaw(heading),
            chest_rotation: yaw(heading),
            head_rotation: yaw(heading),
            ankles: self.nominal_feet(state),
            foot_headings: [self.foot_heading(heading, 0), self.foot_heading(heading, 1)],
            stationary: [true; 2],
        }
    }

    fn motion_pose(&self, motion: &Motion, state: StandState, t: f64) -> PoseSpec {
        let h = state.heading;
        let tf = t as f32;
        match *motion {
            Motion::Stand { secs, sway_m } => {
                let envelope = envelope(t, secs, 1.0);
                let offset = (right(h) * (sway_m * (TAU * 0.23 * tf).sin())
                    + forward(h) * (0.6 * sway_m * (TAU * 0.17 * tf + 1.0).sin()))
                    * envelope;
                self.stand_pose(state, offset)
            }
            Motion::WeightShift {
                secs,
                amplitude_m,
                period_secs,
            } => {
                let lateral =
                    amplitude_m * (TAU * (t / period_secs) as f32).sin() * envelope(t, secs, 1.0);
                let mut pose = self.stand_pose(state, right(h) * lateral);
                pose.pelvis_rotation = body_rotation(h, 0.0, -0.5 * lateral);
                pose.chest_rotation = body_rotation(h, 0.0, 0.3 * lateral);
                pose
            }
            Motion::Squats {
                depth_m,
                period_secs,
                ..
            } => self.squat_pose(state, t, depth_m, period_secs, 1.0),
            Motion::Walk {
                steps,
                step_length_m,
                cadence_hz,
                clearance_m,
                heel_rise_degrees,
            } => self.walk_pose(
                state,
                t,
                steps,
                step_length_m,
                cadence_hz,
                clearance_m,
                heel_rise_degrees,
            ),
            Motion::Turn {
                degrees,
                pivot_fraction,
                pivot_secs,
                replant_secs,
            } => self.turn_pose(state, t, degrees, pivot_fraction, pivot_secs, replant_secs),
            Motion::HeadTurns {
                secs,
                yaw_degrees,
                pitch_degrees,
                period_secs,
                body_sway_m,
            } => {
                let envelope = envelope(t, secs, 1.0);
                let offset = (right(h) * (body_sway_m * (TAU * tf / 5.3).sin())
                    + forward(h) * (0.5 * body_sway_m * (TAU * tf / 7.1 + 0.5).sin()))
                    * envelope;
                let mut pose = self.stand_pose(state, offset);
                let phase = (t / period_secs) as f32;
                let head_yaw = yaw_degrees.to_radians() * (TAU * phase).sin() * envelope;
                let head_pitch = pitch_degrees.to_radians() * (TAU * phase / 1.37).sin() * envelope;
                pose.head_rotation = yaw(h + head_yaw) * Quat::from_rotation_x(head_pitch);
                pose
            }
            Motion::FootSlide {
                foot,
                speed_mps,
                distance_m,
                direction_degrees,
                squat_depth_m,
                squat_period_secs,
                settle_secs,
                replant_secs,
            } => {
                let foot = foot.min(1);
                let slide = slide_secs(distance_m, speed_mps);
                let active = 2.0 * settle_secs + slide;
                let squat_envelope = envelope(t, active, 0.5);
                let mut pose =
                    self.squat_pose(state, t, squat_depth_m, squat_period_secs, squat_envelope);
                let angle = direction_degrees.to_radians();
                let direction =
                    right(h) * (LATERAL_SIGN[foot] * angle.cos()) + forward(h) * angle.sin();
                let nominal = pose.ankles[foot];
                let moved = nominal + direction * distance_m;
                if t < settle_secs {
                    // planted at the nominal position
                } else if t < settle_secs + slide {
                    let x = smoothstep(((t - settle_secs) / slide) as f32);
                    pose.ankles[foot] = nominal + direction * (distance_m * x);
                    pose.stationary[foot] = false;
                } else if t < active {
                    pose.ankles[foot] = moved;
                } else {
                    let sigma = (((t - active) / replant_secs) as f32).clamp(0.0, 1.0);
                    pose.ankles[foot] = moved
                        + (nominal - moved) * smootherstep(sigma)
                        + Vec3::Y * (0.05 * (PI * sigma).sin());
                    pose.stationary[foot] = false;
                }
                pose
            }
            Motion::TorsoBows {
                chest_degrees,
                period_secs,
                ..
            } => {
                let phi = 0.5 * (1.0 - (TAU * (t / period_secs) as f32).cos());
                let depth = chest_degrees.to_radians();
                let scale = chest_degrees / 60.0;
                let offset = -forward(h) * (0.10 * scale * phi) - Vec3::Y * (0.03 * phi);
                let mut pose = self.stand_pose(state, offset);
                pose.chest_rotation = body_rotation(h, depth * phi, 0.0);
                pose.pelvis_rotation = body_rotation(h, 0.5 * depth * phi, 0.0);
                pose.head_rotation = body_rotation(h, 0.6 * depth * phi, 0.0);
                pose
            }
            Motion::Jumps {
                height_m,
                period_secs,
                ..
            } => {
                const FLIGHT_START: f32 = 0.35;
                const FLIGHT_END: f32 = 0.65;
                let x = ((t / period_secs).fract() as f32).clamp(0.0, 1.0);
                let (crouch, lift) = if x < FLIGHT_START {
                    (0.12 * (PI * x / FLIGHT_START).sin(), 0.0)
                } else if x < FLIGHT_END {
                    let s = (x - FLIGHT_START) / (FLIGHT_END - FLIGHT_START);
                    (0.0, height_m * (PI * s).sin())
                } else {
                    let s = (x - FLIGHT_END) / (1.0 - FLIGHT_END);
                    (0.08 * (PI * s).sin(), 0.0)
                };
                let mut pose = self.stand_pose(state, Vec3::Y * (lift - crouch));
                for foot in 0..2 {
                    pose.ankles[foot].y += lift;
                    pose.stationary[foot] = lift <= 0.0;
                }
                // Upright torso: a few degrees of lean in the crouch only.
                let lean = (40.0 * crouch).to_radians();
                pose.pelvis_rotation = body_rotation(h, lean, 0.0);
                pose.chest_rotation = body_rotation(h, 0.5 * lean, 0.0);
                pose
            }
            Motion::KneeLift {
                foot,
                height_m,
                secs,
            } => {
                let foot = foot.min(1);
                let phi = 0.5 * (1.0 - (TAU * (t / secs) as f32).cos());
                let support = 1 - foot;
                let shift = right(h) * (LATERAL_SIGN[support] * 0.05 * phi);
                let mut pose = self.stand_pose(state, shift);
                pose.ankles[foot] += Vec3::Y * (height_m * phi) + forward(h) * (0.12 * phi);
                pose.stationary[foot] = phi <= 0.0;
                pose
            }
        }
    }

    fn squat_pose(
        &self,
        state: StandState,
        t: f64,
        depth_m: f32,
        period_secs: f64,
        amplitude: f32,
    ) -> PoseSpec {
        let h = state.heading;
        let phi = 0.5 * (1.0 - (TAU * (t / period_secs) as f32).cos()) * amplitude;
        let scale = depth_m / 0.30;
        let offset = -forward(h) * (0.12 * scale * phi) - Vec3::Y * (depth_m * phi);
        let mut pose = self.stand_pose(state, offset);
        pose.pelvis_rotation = body_rotation(h, (25.0 * scale * phi).to_radians(), 0.0);
        pose.chest_rotation = body_rotation(h, (35.0 * scale * phi).to_radians(), 0.0);
        pose.head_rotation = body_rotation(h, (10.0 * phi).to_radians(), 0.0);
        pose
    }

    #[allow(clippy::too_many_arguments)]
    fn walk_pose(
        &self,
        state: StandState,
        t: f64,
        steps: u32,
        step_length: f32,
        cadence_hz: f64,
        clearance: f32,
        heel_rise_degrees: f32,
    ) -> PoseSpec {
        const HEEL_END: f32 = 0.12;
        const SWING_END: f32 = 0.88;
        let h = state.heading;
        let n = steps.max(2) as usize;
        let period = 1.0 / cadence_hz;
        let step_index = ((t / period).floor().max(0.0) as usize).min(n - 1);
        let x = ((t / period) as f32 - step_index as f32).clamp(0.0, 1.0);
        let target = |k: usize| {
            if k + 1 < n {
                (k + 1) as f32 * step_length
            } else {
                (n - 1) as f32 * step_length
            }
        };
        let swing_foot = |k: usize| if k.is_multiple_of(2) { 1 } else { 0 };
        // Forward position of `foot` after steps 0..=last (none when `last` is None).
        let position_after = |foot: usize, last: Option<usize>| -> f32 {
            last.and_then(|last| (0..=last).rev().find(|&k| swing_foot(k) == foot))
                .map_or(0.0, target)
        };
        let swing = swing_foot(step_index);
        let stance = 1 - swing;
        let before = step_index.checked_sub(1);
        let u_before = [position_after(0, before), position_after(1, before)];
        let u_after = [
            position_after(0, Some(step_index)),
            position_after(1, Some(step_index)),
        ];
        let mid_before = 0.5 * (u_before[0] + u_before[1]);
        let mid_after = 0.5 * (u_after[0] + u_after[1]);
        let first = step_index == 0;
        let last = step_index == n - 1;
        let progress = if first {
            x * x
        } else if last {
            1.0 - (1.0 - x) * (1.0 - x)
        } else {
            x
        };
        let u_pelvis = mid_before + (mid_after - mid_before) * progress;
        let blend = if first {
            smoothstep(x)
        } else if last {
            1.0 - smoothstep(x)
        } else {
            1.0
        };
        let walk_height =
            self.geometry.ankle_height_m + self.geometry.walk_reach * self.leg_length();
        let bob = 0.012 * -(TAU * x).cos();
        let height = self.stand_height() + (walk_height + bob - self.stand_height()) * blend;
        let arc = (PI * x).sin();
        let sway = LATERAL_SIGN[stance] * 0.02 * arc;
        let floor_center = state.center * Vec3::new(1.0, 0.0, 1.0);
        let pelvis = floor_center + forward(h) * u_pelvis + right(h) * sway + Vec3::Y * height;
        let pelvis_yaw = (4.0_f32).to_radians() * arc * LATERAL_SIGN[swing] * blend;
        let pelvis_roll = (3.0_f32).to_radians() * arc * LATERAL_SIGN[stance] * blend;
        let pelvis_rotation =
            body_rotation(h + pelvis_yaw, (4.0 * blend).to_radians(), pelvis_roll);
        let chest_rotation = body_rotation(h - 0.5 * pelvis_yaw, (2.0 * blend).to_radians(), 0.0);
        let look = (3.0_f32).to_radians() * (TAU * 0.3 * t as f32).sin() * blend;
        let head_rotation = body_rotation(h + look, (8.0 * blend).to_radians(), 0.0);

        let foot_position = |foot: usize, u: f32| {
            floor_center
                + forward(h) * u
                + right(h) * (LATERAL_SIGN[foot] * self.geometry.foot_half_width_m)
                + Vec3::Y * self.geometry.ankle_height_m
        };
        let foot_headings = [self.foot_heading(h, 0), self.foot_heading(h, 1)];
        let mut ankles = [Vec3::ZERO; 2];
        let mut stationary = [true; 2];
        let heel = heel_rise_degrees.to_radians();
        for foot in 0..2 {
            let rest = foot_position(foot, u_before[foot]);
            if foot != swing {
                ankles[foot] = rest;
                continue;
            }
            let land = foot_position(foot, u_after[foot]);
            if x < HEEL_END {
                let rise = x / HEEL_END;
                ankles[foot] = self.heel_rise(rest, foot_headings[foot], heel * rise * rise);
                stationary[foot] = x <= 0.0;
            } else if x < SWING_END {
                let sigma = (x - HEEL_END) / (SWING_END - HEEL_END);
                let start = self.heel_rise(rest, foot_headings[foot], heel);
                ankles[foot] = start
                    + (land - start) * smootherstep(sigma)
                    + Vec3::Y * (clearance * (PI * sigma).sin());
                stationary[foot] = false;
            } else {
                ankles[foot] = land;
            }
        }
        PoseSpec {
            pelvis,
            pelvis_rotation,
            chest_rotation,
            head_rotation,
            ankles,
            foot_headings,
            stationary,
        }
    }

    fn turn_pose(
        &self,
        state: StandState,
        t: f64,
        degrees: f32,
        pivot_fraction: f32,
        pivot_secs: f64,
        replant_secs: f64,
    ) -> PoseSpec {
        let h = state.heading;
        let total = degrees.to_radians();
        let pivot_angle = total * pivot_fraction.clamp(0.0, 1.0);
        let start_feet = self.nominal_feet(state);
        let start_headings = [self.foot_heading(h, 0), self.foot_heading(h, 1)];
        let balls = [0, 1].map(|foot| self.ball(start_feet[foot], start_headings[foot]));
        let pivoted =
            |foot: usize, angle: f32| balls[foot] + yaw(angle) * (start_feet[foot] - balls[foot]);
        let mut pose = self.stand_pose(state, Vec3::ZERO);
        if t < pivot_secs {
            let x = (t / pivot_secs) as f32;
            let angle = pivot_angle * smoothstep(x);
            for foot in 0..2 {
                pose.ankles[foot] = pivoted(foot, angle);
                pose.foot_headings[foot] = start_headings[foot] + angle;
                pose.stationary[foot] = x <= 0.0;
            }
            pose.pelvis_rotation = yaw(h + angle);
            pose.chest_rotation = yaw(h + angle);
            pose.head_rotation = yaw(h + pivot_angle * smoothstep((x / 0.7).min(1.0)));
            return pose;
        }
        let end = StandState {
            center: state.center,
            heading: h + total,
        };
        let end_feet = self.nominal_feet(end);
        let first = if total >= 0.0 { 0 } else { 1 };
        let tt = t - pivot_secs;
        let progress = [
            ((tt / replant_secs) as f32).clamp(0.0, 1.0),
            (((tt - replant_secs) / replant_secs) as f32).clamp(0.0, 1.0),
        ];
        let remaining = total - pivot_angle;
        for (order, foot) in [first, 1 - first].into_iter().enumerate() {
            let sigma = progress[order];
            let from = pivoted(foot, pivot_angle);
            let to = end_feet[foot];
            let from_heading = start_headings[foot] + pivot_angle;
            let to_heading = self.foot_heading(end.heading, foot);
            let s = smootherstep(sigma);
            pose.ankles[foot] = from + (to - from) * s + Vec3::Y * (0.05 * (PI * sigma).sin());
            pose.foot_headings[foot] = from_heading + (to_heading - from_heading) * s;
            pose.stationary[foot] = sigma <= 0.0 || sigma >= 1.0;
        }
        let body = h
            + pivot_angle
            + remaining * (0.5 * smoothstep(progress[0]) + 0.5 * smoothstep(progress[1]));
        pose.pelvis_rotation = yaw(body);
        pose.chest_rotation = yaw(body);
        pose.head_rotation = yaw(h + pivot_angle + remaining * smoothstep(progress[0]));
        pose
    }

    /// Bottom-up assembly: pelvis → spine → neck → eye point, and two-bone IK for the legs.
    fn assemble(&self, at_secs: f64, spec: PoseSpec) -> TruthFrame {
        let d = self.dims;
        let max_reach = self.geometry.max_reach * self.leg_length();
        let mut hip = spec.pelvis;
        for _ in 0..3 {
            let mut drop = 0.0_f32;
            for foot in 0..2 {
                let joint =
                    hip + spec.pelvis_rotation * Vec3::X * (LATERAL_SIGN[foot] * d.hip_width * 0.5);
                let v = spec.ankles[foot] - joint;
                if v.length() > max_reach {
                    let horizontal = v.x * v.x + v.z * v.z;
                    let allowed = (max_reach * max_reach - horizontal).max(0.0).sqrt();
                    drop = drop.max(-v.y - allowed);
                }
            }
            if drop <= 1.0e-6 {
                break;
            }
            hip.y -= drop;
        }
        let spine = spec
            .chest_rotation
            .slerp(spec.pelvis_rotation, 0.5)
            .normalize();
        let chest = hip + spine * Vec3::Y * d.chest_to_hip;
        let neck = chest + spec.chest_rotation * Vec3::Y * d.neck_to_chest;
        let head_lever = -(spec.head_rotation * Vec3::Y) * d.head_to_neck
            + (spec.head_rotation * Vec3::Z) * d.eye_forward;
        let head_position = neck - head_lever;

        let mut rotations = [Quat::IDENTITY; 6];
        rotations[0] = spec.chest_rotation;
        rotations[1] = spec.pelvis_rotation;
        let mut ankles = [Vec3::ZERO; 2];
        let mut shortfall = 0.0_f32;
        for foot in 0..2 {
            let joint =
                hip + spec.pelvis_rotation * Vec3::X * (LATERAL_SIGN[foot] * d.hip_width * 0.5);
            let leg = two_bone(
                joint,
                spec.ankles[foot],
                d.thigh,
                d.calf,
                forward(spec.foot_headings[foot]),
            );
            rotations[2 + foot] = leg.thigh;
            rotations[CALF[foot]] = leg.calf;
            ankles[foot] = leg.ankle;
            shortfall = shortfall.max(leg.shortfall);
        }
        TruthFrame {
            at_secs,
            head_position,
            head_rotation: spec.head_rotation,
            rotations,
            ankles,
            stationary: spec.stationary,
            ik_shortfall_m: shortfall,
        }
    }
}

struct LegSolution {
    thigh: Quat,
    calf: Quat,
    ankle: Vec3,
    shortfall: f32,
}

/// Two-bone IK from the hip joint to the ankle target with the knee bending towards `pole`.
fn two_bone(hip_joint: Vec3, target: Vec3, thigh: f32, calf: f32, pole: Vec3) -> LegSolution {
    let v = target - hip_joint;
    let distance = v.length();
    let direction = if distance > 1.0e-6 {
        v / distance
    } else {
        -Vec3::Y
    };
    let reach = distance.clamp((thigh - calf).abs() + 1.0e-4, thigh + calf - 1.0e-4);
    let cos_hip =
        ((thigh * thigh + reach * reach - calf * calf) / (2.0 * thigh * reach)).clamp(-1.0, 1.0);
    let sin_hip = (1.0 - cos_hip * cos_hip).max(0.0).sqrt();
    let bend = (pole - direction * pole.dot(direction)).normalize_or(Vec3::NEG_Z);
    let knee = hip_joint + direction * (thigh * cos_hip) + bend * (thigh * sin_hip);
    let ankle = hip_joint + direction * reach;
    let hinge = direction.cross(bend).normalize_or(Vec3::X);
    let thigh_axis = (hip_joint - knee) / thigh;
    let calf_axis = (knee - ankle) / calf;
    let basis = |axis: Vec3| {
        let y = (axis - hinge * hinge.dot(axis)).normalize_or(Vec3::Y);
        Quat::from_mat3(&Mat3::from_cols(hinge, y, hinge.cross(y))).normalize()
    };
    LegSolution {
        thigh: basis(thigh_axis),
        calf: basis(calf_axis),
        ankle,
        shortfall: (distance - reach).abs(),
    }
}

// ---------------------------------------------------------------------------------------------
// Injection models.

/// Per-sensor yaw error `e(t)` in degrees: measured = `Ry(−e)·true`, so the ideal filter
/// correction converges to `+e`.
#[derive(Clone, Debug)]
pub enum YawError {
    None,
    Constant {
        degrees: f32,
    },
    Step {
        at_secs: f64,
        degrees: f32,
    },
    Ramp {
        start_secs: f64,
        degrees_per_sec: f32,
        max_degrees: f32,
    },
    /// Independent Gaussian random walk, deterministic per seed.
    RandomWalk {
        sigma_degrees_per_sqrt_sec: f32,
        seed: u64,
    },
    Function(fn(f64) -> f32),
}

#[derive(Clone, Debug)]
struct YawTrack {
    model: YawError,
    knots: Vec<f32>,
}

const RANDOM_WALK_KNOT_SECS: f64 = 0.1;

impl YawTrack {
    fn new(model: &YawError, duration: f64) -> Self {
        let knots = match *model {
            YawError::RandomWalk {
                sigma_degrees_per_sqrt_sec,
                seed,
            } => {
                let mut rng = XorShift64::new(seed);
                let knots = (duration / RANDOM_WALK_KNOT_SECS).ceil();
                let count = if knots.is_finite() {
                    knots.clamp(0.0, 1.0e7) as usize + 2
                } else {
                    2
                };
                let step = f64::from(sigma_degrees_per_sqrt_sec) * RANDOM_WALK_KNOT_SECS.sqrt();
                let mut value = 0.0;
                (0..count)
                    .map(|k| {
                        if k > 0 {
                            value += step * rng.gaussian();
                        }
                        value as f32
                    })
                    .collect()
            }
            _ => Vec::new(),
        };
        Self {
            model: model.clone(),
            knots,
        }
    }

    fn degrees(&self, at: f64) -> f32 {
        match self.model {
            YawError::None => 0.0,
            YawError::Constant { degrees } => degrees,
            YawError::Step { at_secs, degrees } => {
                if at >= at_secs {
                    degrees
                } else {
                    0.0
                }
            }
            YawError::Ramp {
                start_secs,
                degrees_per_sec,
                max_degrees,
            } => {
                let value = ((at - start_secs).max(0.0) as f32) * degrees_per_sec;
                value.clamp(-max_degrees.abs(), max_degrees.abs())
            }
            YawError::RandomWalk { .. } => {
                let position = (at / RANDOM_WALK_KNOT_SECS).max(0.0);
                let index = position.floor() as usize;
                let fraction = (position - index as f64) as f32;
                match (self.knots.get(index), self.knots.get(index + 1)) {
                    (Some(a), Some(b)) => a + (b - a) * fraction,
                    (Some(a), None) => *a,
                    _ => self.knots.last().copied().unwrap_or(0.0),
                }
            }
            YawError::Function(function) => function(at),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    /// The segment is reported stale/held (`usable = false`) but keeps its last pose.
    Unusable,
    /// The segment rotation is NaN while still flagged usable (a caller bug).
    NanRotation,
    /// The headset position is NaN (segment ignored).
    NanHead,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fault {
    pub segment: usize,
    pub start_secs: f64,
    pub end_secs: f64,
    pub kind: FaultKind,
}

/// Torso model error of a flexed spine: in a forward bend the chest and waist trackers (on
/// soft tissue, over a curved spine) read more tilt than the straight hip-to-neck line the
/// chain models. The extra forward pitch grows linearly with the true chest tilt and equals
/// `chest_degrees_at_60` / `hip_degrees_at_60` at a 60° chest bow. The over-read rotation axis
/// is the bone's lateral axis turned by `axis_offset_degrees` about the bone's long axis, so a
/// nonzero offset gives the error a sideways component, as an asymmetric bend or a tracker
/// riding off-centre does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlexedSpine {
    pub chest_degrees_at_60: f32,
    pub hip_degrees_at_60: f32,
    pub axis_offset_degrees: f32,
}

impl FlexedSpine {
    /// Chest +12°, hip +30° at a 60° bow, 15° axis offset.
    pub const LIVE_LIKE: Self = Self {
        chest_degrees_at_60: 12.0,
        hip_degrees_at_60: 30.0,
        axis_offset_degrees: 15.0,
    };

    /// The torso rotations the trackers report for true bone rotations `truth`
    /// (`Role::CORE` order); the leg rotations are unchanged.
    pub fn measured(&self, truth: &[Quat; 6]) -> [Quat; 6] {
        let chest_tilt = (truth[0] * Vec3::Y).y.clamp(-1.0, 1.0).acos();
        let scale = chest_tilt / 60.0_f32.to_radians();
        let offset = self.axis_offset_degrees.to_radians();
        // Bone-local lateral axis turned about the bone's +Y; rotating about it by a
        // negative angle tips the bone's top towards −Z (forward).
        let axis = Vec3::new(offset.cos(), 0.0, -offset.sin());
        let mut measured = *truth;
        for (i, degrees) in [(0, self.chest_degrees_at_60), (1, self.hip_degrees_at_60)] {
            let extra = (degrees * scale).to_radians();
            measured[i] = (truth[i] * Quat::from_axis_angle(axis, -extra)).normalize();
        }
        measured
    }
}

/// A tracking-space discontinuity: from `at_secs` on, the reported headset position is offset.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadJump {
    pub at_secs: f64,
    pub offset: Vec3,
}

/// A complete synthetic experiment.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: String,
    pub script: Vec<Motion>,
    pub start: StandState,
    pub rate_hz: f64,
    pub true_dims: ChainDimensions,
    /// Dimensions assumed by the estimator (bone-length and eye-offset errors live here).
    pub assumed_dims: ChainDimensions,
    pub geometry: BodyGeometry,
    pub yaw_errors: [YawError; 6],
    /// Strap rotation of each sensor about its bone's own long axis (+Y), degrees:
    /// measured = `Ry(−e)·true·Ry_bone(−s)`. For an upright bone this equals a world-yaw
    /// error `e = s`; for a pitched bone a world-yaw correction can only approximate it.
    pub strap_errors: [YawError; 6],
    /// 1σ per axis of white headset position noise.
    pub head_noise_m: f32,
    pub head_jump: Option<HeadJump>,
    pub faults: Vec<Fault>,
    /// Torso trackers over-read a forward bend (applied to the truth before yaw errors).
    pub flexed_spine: Option<FlexedSpine>,
    pub seed: u64,
}

impl Scenario {
    pub fn new(name: &str, script: Vec<Motion>) -> Self {
        Self {
            name: name.to_owned(),
            script,
            start: StandState::default(),
            rate_hz: 100.0,
            true_dims: ChainDimensions::default(),
            assumed_dims: ChainDimensions::default(),
            geometry: BodyGeometry::default(),
            yaw_errors: std::array::from_fn(|_| YawError::None),
            strap_errors: std::array::from_fn(|_| YawError::None),
            head_noise_m: 0.001,
            head_jump: None,
            faults: Vec::new(),
            flexed_spine: None,
            seed: 1,
        }
    }

    pub fn duration(&self) -> f64 {
        self.script.iter().map(Motion::duration).sum()
    }

    pub fn with_yaw_error(mut self, segment: usize, error: YawError) -> Self {
        if let Some(slot) = self.yaw_errors.get_mut(segment) {
            *slot = error;
        }
        self
    }

    pub fn with_strap_error(mut self, segment: usize, error: YawError) -> Self {
        if let Some(slot) = self.strap_errors.get_mut(segment) {
            *slot = error;
        }
        self
    }

    /// Same constant error on all four leg segments (common mode).
    pub fn with_leg_error_degrees(mut self, degrees: f32) -> Self {
        for segment in 2..6 {
            self.yaw_errors[segment] = YawError::Constant { degrees };
        }
        self
    }

    /// Scales the estimator's assumed bone lengths (not the eye offset) by `factor`.
    pub fn with_assumed_bone_scale(mut self, factor: f32) -> Self {
        let d = &mut self.assumed_dims;
        d.head_to_neck *= factor;
        d.neck_to_chest *= factor;
        d.chest_to_hip *= factor;
        d.hip_width *= factor;
        d.thigh *= factor;
        d.calf *= factor;
        self
    }

    /// Estimator eye-forward offset minus the true one.
    pub fn with_eye_offset_error(mut self, meters: f32) -> Self {
        self.assumed_dims.eye_forward = self.true_dims.eye_forward + meters;
        self
    }

    pub fn generator(&self) -> BodyGenerator {
        BodyGenerator::new(self.true_dims, self.geometry, self.start, &self.script)
    }
}

// ---------------------------------------------------------------------------------------------
// Closed-loop harness.

#[derive(Clone, Copy, Debug)]
pub struct HarnessOptions {
    pub filter: StanceHeadingConfig,
    pub contact: ContactConfig,
    /// When false the filter is not run and no correction is applied (baseline).
    pub filter_enabled: bool,
    /// Pass finite-difference angular speeds of the measured rotations to the filter.
    pub supply_angular_speed: bool,
    /// Widen the contact speed gates by the filter's uncertainty
    /// (`StanceHeadingFilter::contact_speed_allowance` with the smoothed headset velocity).
    pub use_speed_allowance: bool,
    /// Extra initial 1σ per segment (degrees), applied with
    /// `StanceHeadingFilter::add_uncertainty` before the first frame.
    pub extra_initial_sigma_degrees: [f32; 6],
}

impl Default for HarnessOptions {
    fn default() -> Self {
        Self {
            filter: StanceHeadingConfig::default(),
            contact: ContactConfig::default(),
            filter_enabled: true,
            supply_angular_speed: true,
            use_speed_allowance: true,
            extra_initial_sigma_degrees: [0.0; 6],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HarnessFrame {
    pub at_secs: f64,
    /// True yaw error remaining after the filter's correction, degrees: `wrap(e_i − c_i)`, or
    /// for a segment with a strap error the lateral-axis heading of the truth minus that of
    /// the corrected measurement.
    pub remaining_error_degrees: [f32; 6],
    pub corrections_degrees: [f32; 6],
    pub sigma_degrees: [f32; 6],
    pub planted: [bool; 2],
    pub truth_stationary: [bool; 2],
    pub status: StanceHeadingStatus,
    pub release_requests: [bool; 2],
    /// Cumulative accepted displacement measurements per foot.
    pub accepted: [u32; 2],
    /// Contact speed allowance applied this frame (m/s).
    pub speed_allowance: [f32; 2],
    /// Every number the filter emitted this frame was finite.
    pub outputs_finite: bool,
}

#[derive(Clone, Debug)]
pub struct HarnessRun {
    pub frames: Vec<HarnessFrame>,
    pub last_output: StanceHeadingOutput,
    pub release_requests: u32,
    pub max_ik_shortfall_m: f32,
    /// Largest true ankle displacement while the contact tracker reported `Planted`
    /// (real foot motion accepted as stationary: heel rise, pivots, slides).
    pub max_planted_truth_motion_m: f32,
}

impl HarnessRun {
    fn values<'a>(
        &'a self,
        segments: &'a [usize],
        from_secs: f64,
    ) -> impl Iterator<Item = f32> + 'a {
        self.frames
            .iter()
            .filter(move |frame| frame.at_secs >= from_secs)
            .flat_map(move |frame| segments.iter().map(|&i| frame.remaining_error_degrees[i]))
    }

    /// First time after which `max |remaining error|` over `segments` stays below
    /// `threshold_degrees` until the end of the run.
    pub fn time_to_converge(&self, segments: &[usize], threshold_degrees: f32) -> Option<f64> {
        let mut converged_at = None;
        for frame in &self.frames {
            let worst = segments
                .iter()
                .map(|&i| frame.remaining_error_degrees[i].abs())
                .fold(0.0, f32::max);
            if worst < threshold_degrees {
                converged_at.get_or_insert(frame.at_secs);
            } else {
                converged_at = None;
            }
        }
        converged_at
    }

    pub fn rms_error_degrees(&self, segments: &[usize], from_secs: f64) -> f32 {
        let (sum, count) = self
            .values(segments, from_secs)
            .fold((0.0_f64, 0_usize), |(s, n), v| {
                (s + f64::from(v * v), n + 1)
            });
        if count == 0 {
            0.0
        } else {
            (sum / count as f64).sqrt() as f32
        }
    }

    pub fn max_abs_error_degrees(&self, segments: &[usize], from_secs: f64) -> f32 {
        self.values(segments, from_secs)
            .fold(0.0, |m, v| m.max(v.abs()))
    }

    pub fn final_errors_degrees(&self) -> [f32; 6] {
        self.frames
            .last()
            .map_or([0.0; 6], |frame| frame.remaining_error_degrees)
    }

    pub fn max_abs_correction_degrees(&self, segments: &[usize]) -> f32 {
        self.frames
            .iter()
            .flat_map(|frame| segments.iter().map(|&i| frame.corrections_degrees[i].abs()))
            .fold(0.0, f32::max)
    }

    pub fn accepted(&self) -> u32 {
        self.last_output.feet.iter().map(|foot| foot.accepted).sum()
    }

    pub fn rejected(&self) -> u32 {
        self.last_output.feet.iter().map(|foot| foot.rejected).sum()
    }
}

fn wrap_degrees(value: f32) -> f32 {
    let wrapped = (value + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped.is_finite() { wrapped } else { 0.0 }
}

/// Runs truth → injected measurements → (apply filter corrections) → `solve_chain` →
/// `ContactTracker` → `StanceHeadingFilter`, and records the true remaining yaw error per
/// segment after correction.
pub fn run_closed_loop(scenario: &Scenario, options: &HarnessOptions) -> HarnessRun {
    let generator = scenario.generator();
    let duration = generator.duration();
    let tracks: Vec<YawTrack> = scenario
        .yaw_errors
        .iter()
        .map(|model| YawTrack::new(model, duration))
        .collect();
    let straps: Vec<YawTrack> = scenario
        .strap_errors
        .iter()
        .map(|model| YawTrack::new(model, duration))
        .collect();
    let mut head_velocity = Vec3::ZERO;
    let mut previous_head: Option<Vec3> = None;
    let mut rng = XorShift64::new(scenario.seed);
    let mut contact = ContactTracker::new(options.contact);
    let mut filter = StanceHeadingFilter::new(options.filter);
    filter.add_uncertainty(options.extra_initial_sigma_degrees);
    let rate = if scenario.rate_hz > 0.0 {
        scenario.rate_hz
    } else {
        100.0
    };
    let count = (duration * rate).floor() as usize + 1;
    let mut frames = Vec::with_capacity(count);
    let mut previous_raw: [Option<Quat>; 6] = [None; 6];
    let mut angular_speed = [0.0_f32; 6];
    let mut previous_at: Option<f64> = None;
    let mut plant_anchor: [Option<Vec3>; 2] = [None; 2];
    let mut max_planted_motion = 0.0_f32;
    let mut max_shortfall = 0.0_f32;
    let mut release_count = 0;
    let mut last_output = filter.last_output();

    for k in 0..count {
        let at = k as f64 / rate;
        let truth = generator.frame(at);
        max_shortfall = max_shortfall.max(truth.ik_shortfall_m);
        let errors: [f32; 6] = std::array::from_fn(|i| tracks[i].degrees(at));
        let strap: [f32; 6] = std::array::from_fn(|i| straps[i].degrees(at));
        let worn = scenario
            .flexed_spine
            .map_or(truth.rotations, |spine| spine.measured(&truth.rotations));
        let mut raw: [Quat; 6] = std::array::from_fn(|i| {
            Quat::from_rotation_y(-errors[i].to_radians())
                * worn[i]
                * Quat::from_rotation_y(-strap[i].to_radians())
        });
        let noise = Vec3::new(
            rng.gaussian() as f32,
            rng.gaussian() as f32,
            rng.gaussian() as f32,
        ) * scenario.head_noise_m;
        let mut head_position = truth.head_position + noise;
        if let Some(jump) = scenario.head_jump.filter(|jump| at >= jump.at_secs) {
            head_position += jump.offset;
        }
        let mut usable = [true; 6];
        for fault in &scenario.faults {
            if at < fault.start_secs || at >= fault.end_secs {
                continue;
            }
            match fault.kind {
                FaultKind::Unusable => {
                    if let Some(flag) = usable.get_mut(fault.segment) {
                        *flag = false;
                    }
                }
                FaultKind::NanRotation => {
                    if let Some(rotation) = raw.get_mut(fault.segment) {
                        *rotation = Quat::NAN;
                    }
                }
                FaultKind::NanHead => head_position = Vec3::NAN,
            }
        }

        let dt = previous_at.map(|previous| at - previous);
        previous_at = Some(at);
        // Smoothed horizontal headset velocity (τ = 0.1 s); jumps faster than 4 m/s are
        // discontinuities, not motion, and are skipped.
        if let (Some(previous), Some(dt)) = (previous_head, dt)
            && head_position.is_finite()
            && dt > 0.0
        {
            let mut velocity = (head_position - previous) / dt as f32;
            velocity.y = 0.0;
            if velocity.is_finite() && velocity.length() <= 4.0 {
                let alpha = 1.0 - (-(dt as f32) / 0.1).exp();
                head_velocity += (velocity - head_velocity) * alpha;
            }
        }
        previous_head = head_position.is_finite().then_some(head_position);
        for i in 0..6 {
            if !raw[i].is_finite() {
                previous_raw[i] = None;
                continue;
            }
            if let (Some(previous), Some(dt)) = (previous_raw[i], dt) {
                let delta = raw[i] * previous.conjugate();
                let angle = 2.0
                    * Vec3::new(delta.x, delta.y, delta.z)
                        .length()
                        .atan2(delta.w.abs());
                let alpha = 1.0 - (-(dt as f32) / 0.025).exp();
                angular_speed[i] += (angle / dt as f32 - angular_speed[i]) * alpha;
            }
            previous_raw[i] = Some(raw[i]);
        }

        let corrections = if options.filter_enabled {
            filter.corrections()
        } else {
            [0.0; 6]
        };
        let rotations: [Quat; 6] =
            std::array::from_fn(|i| Quat::from_rotation_y(corrections[i]) * raw[i]);
        let chain: BodyChain = solve_chain(
            &scenario.assumed_dims,
            &ChainPose {
                head_position,
                head_rotation: truth.head_rotation,
                rotations,
            },
        );
        let foot_usable = [0, 1]
            .map(|foot| FOOT_CHAIN[foot].iter().all(|&i| usable[i]) && chain.foot_finite(foot));
        let allowance = if options.filter_enabled && options.use_speed_allowance {
            filter.contact_speed_allowance(head_velocity)
        } else {
            [0.0; 2]
        };
        contact.set_speed_allowance(allowance);
        let contacts = contact.update(
            at,
            chain.ankles,
            [rotations[CALF[0]], rotations[CALF[1]]],
            foot_usable,
        );
        let planted = contacts.map(|c| c.planted());
        for foot in 0..2 {
            if planted[foot] {
                let anchor = *plant_anchor[foot].get_or_insert(truth.ankles[foot]);
                max_planted_motion = max_planted_motion.max((truth.ankles[foot] - anchor).length());
            } else {
                plant_anchor[foot] = None;
            }
        }

        let mut release_requests = [false; 2];
        if options.filter_enabled {
            let output = filter.step(&StanceHeadingInput {
                at_secs: at,
                head_position,
                head_rotation: truth.head_rotation,
                chain: &chain,
                rotations,
                usable,
                planted,
                angular_speed: if options.supply_angular_speed {
                    angular_speed
                } else {
                    [0.0; 6]
                },
            });
            for foot in 0..2 {
                if output.release_requests[foot] {
                    contact.force_release(foot, at);
                    release_count += 1;
                }
            }
            release_requests = output.release_requests;
            last_output = output;
        }
        let corrections_after = if options.filter_enabled {
            last_output.corrections
        } else {
            [0.0; 6]
        };
        let outputs_finite = last_output
            .corrections
            .iter()
            .chain(last_output.sigma.iter())
            .all(|v| v.is_finite());
        let remaining_error_degrees = std::array::from_fn(|i| {
            let yaw_only = wrap_degrees(errors[i] - corrections_after[i].to_degrees());
            if strap[i] == 0.0 {
                return yaw_only;
            }
            let corrected = Quat::from_rotation_y(corrections_after[i]) * raw[i];
            match (
                lateral_heading(truth.rotations[i]),
                lateral_heading(corrected),
            ) {
                (Some(truth_heading), Some(heading)) => {
                    wrap_degrees((truth_heading - heading).to_degrees())
                }
                _ => yaw_only,
            }
        });
        frames.push(HarnessFrame {
            at_secs: at,
            remaining_error_degrees,
            corrections_degrees: corrections_after.map(f32::to_degrees),
            sigma_degrees: last_output.sigma.map(f32::to_degrees),
            planted,
            truth_stationary: truth.stationary,
            status: last_output.status,
            release_requests,
            accepted: last_output.feet.map(|foot| foot.accepted),
            speed_allowance: allowance,
            outputs_finite,
        });
    }
    HarnessRun {
        frames,
        last_output,
        release_requests: release_count,
        max_ik_shortfall_m: max_shortfall,
        max_planted_truth_motion_m: max_planted_motion,
    }
}

// ---------------------------------------------------------------------------------------------
// Small math helpers.

/// `Ry(heading)·(−Z)`.
pub fn forward(heading: f32) -> Vec3 {
    Vec3::new(-heading.sin(), 0.0, -heading.cos())
}

/// `Ry(heading)·X`.
pub fn right(heading: f32) -> Vec3 {
    Vec3::new(heading.cos(), 0.0, -heading.sin())
}

fn yaw(heading: f32) -> Quat {
    Quat::from_rotation_y(heading)
}

/// Heading, then forward pitch (top towards −Z), then roll.
fn body_rotation(heading: f32, pitch_forward: f32, roll: f32) -> Quat {
    yaw(heading) * Quat::from_rotation_x(-pitch_forward) * Quat::from_rotation_z(roll)
}

fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn smootherstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * x * (x * (6.0 * x - 15.0) + 10.0)
}

fn envelope(t: f64, duration: f64, ramp: f64) -> f32 {
    if ramp <= 0.0 {
        return 1.0;
    }
    smoothstep((t / ramp) as f32) * smoothstep(((duration - t) / ramp) as f32)
}

/// Scenario definitions shared by the regression tests and the `stance_heading` example.
pub mod scenarios {
    use super::{Fault, FaultKind, FlexedSpine, HeadJump, Motion, Scenario, YawError};
    use glam::Vec3;

    /// Back-and-forth room-scale walking: `legs` × (walk 8 steps, turn 180°).
    pub fn walk_laps(laps: usize) -> Vec<Motion> {
        let mut script = vec![Motion::stand(1.0)];
        for _ in 0..laps {
            script.push(Motion::walk(8));
            script.push(Motion::Turn {
                degrees: 180.0,
                pivot_fraction: 0.5,
                pivot_secs: 1.0,
                replant_secs: 0.5,
            });
        }
        script.push(Motion::stand(1.0));
        script
    }

    /// Walking with the same constant yaw error on all four leg sensors.
    pub fn walking_common_leg_drift(degrees: f32) -> Scenario {
        Scenario::new("walking, common-mode leg drift", walk_laps(8))
            .with_leg_error_degrees(degrees)
    }

    /// Walking (after convergence) when all four leg sensors suddenly gain the same world-yaw
    /// error at `at_secs`.
    pub fn walking_sudden_leg_error(degrees: f32, at_secs: f64) -> Scenario {
        let mut scenario = Scenario::new(
            &format!("walking, sudden {degrees}° leg error at {at_secs} s"),
            walk_laps(8),
        );
        for segment in 2..6 {
            scenario.yaw_errors[segment] = YawError::Step { at_secs, degrees };
        }
        scenario
    }

    /// Walking when the left thigh sensor suddenly gains a world-yaw error at `at_secs`.
    pub fn walking_sudden_thigh_error(degrees: f32, at_secs: f64) -> Scenario {
        Scenario::new(
            &format!("walking, left thigh jumps {degrees}° at {at_secs} s"),
            walk_laps(8),
        )
        .with_yaw_error(2, YawError::Step { at_secs, degrees })
    }

    /// Walking when the left thigh strap rotates about the leg at `at_secs` (a rotation about
    /// the bone's long axis, not a world-yaw error).
    pub fn walking_thigh_strap_rotation(degrees: f32, at_secs: f64) -> Scenario {
        Scenario::new(
            &format!("walking, left thigh strap turns {degrees}° at {at_secs} s"),
            walk_laps(8),
        )
        .with_strap_error(2, YawError::Step { at_secs, degrees })
    }

    /// 60 s of standing while every leg sensor drifts to `degrees` (no information, so the
    /// filter stays confident), then room-scale walking.
    pub fn unobserved_leg_drift_then_walking(degrees: f32) -> Scenario {
        let mut script = vec![Motion::stand(60.0)];
        script.extend(walk_laps(6));
        let mut scenario = Scenario::new(
            &format!("stand 60 s while legs drift {degrees}°, then walk"),
            script,
        );
        for segment in 2..6 {
            scenario.yaw_errors[segment] = YawError::Ramp {
                start_secs: 0.0,
                degrees_per_sec: degrees / 60.0,
                max_degrees: degrees.abs(),
            };
        }
        scenario
    }

    /// Squats, a few steps, weight shifts and turns, with one calf drifting: the steps and
    /// weight shifts observe the leg sum mode that symmetric squats cannot.
    pub fn mixed_squats_calf_drift(degrees: f32) -> Scenario {
        let mut script = vec![Motion::stand(1.0)];
        for _ in 0..3 {
            script.extend([
                Motion::squats(3),
                Motion::walk(4),
                Motion::weight_shift(8.0),
                Motion::turn(180.0),
                Motion::walk(4),
                Motion::turn(180.0),
            ]);
        }
        Scenario::new("squats + steps + weight shifts, left calf drift", script)
            .with_yaw_error(4, YawError::Constant { degrees })
    }

    /// Squats with one calf drifting.
    pub fn squats_calf_drift(degrees: f32) -> Scenario {
        Scenario::new(
            "squats, left calf drift",
            vec![Motion::stand(1.0), Motion::squats(20), Motion::stand(1.0)],
        )
        .with_yaw_error(4, YawError::Constant { degrees })
    }

    /// Lateral weight shifting with common-mode leg drift.
    pub fn weight_shift_leg_drift(degrees: f32) -> Scenario {
        Scenario::new(
            "weight shifting, common-mode leg drift",
            vec![Motion::stand(1.0), Motion::weight_shift(120.0)],
        )
        .with_leg_error_degrees(degrees)
    }

    /// Perfectly still body with drifting sensors and 3 mm headset noise. `common` gives every
    /// sensor the same ramp (whole-body heading drift); otherwise each sensor drifts
    /// differently.
    pub fn standing_still_drift(common: bool) -> Scenario {
        let name = if common {
            "standing still, common-mode drift"
        } else {
            "standing still, independent drift"
        };
        let mut scenario = Scenario::new(name, vec![Motion::stand(60.0)]);
        let rates = if common {
            [0.25_f32; 6]
        } else {
            [0.20, -0.15, 0.30, 0.25, -0.20, 0.35]
        };
        for (segment, rate) in rates.into_iter().enumerate() {
            scenario.yaw_errors[segment] = YawError::Ramp {
                start_secs: 0.0,
                degrees_per_sec: rate,
                max_degrees: 30.0,
            };
        }
        scenario.head_noise_m = 0.003;
        scenario
    }

    /// Head-only turns (±60° yaw, ±20° pitch) with a small body sway and a wrong eye offset.
    pub fn head_turns_eye_offset(error_m: f32) -> Scenario {
        Scenario::new(
            "head-only turns, eye-offset error",
            vec![Motion::stand(1.0), Motion::head_turns(90.0)],
        )
        .with_eye_offset_error(error_m)
    }

    /// Walking and squats with the estimator's bone lengths scaled by `scale`.
    pub fn bone_length_error(scale: f32, leg_degrees: f32) -> Scenario {
        let mut script = walk_laps(4);
        script.push(Motion::squats(8));
        script.extend(walk_laps(4));
        Scenario::new("bone-length error", script)
            .with_assumed_bone_scale(scale)
            .with_leg_error_degrees(leg_degrees)
    }

    /// Squatting while one foot really slides slowly (below the contact thresholds).
    pub fn slow_foot_slide(speed_mps: f32, direction_degrees: f32) -> Scenario {
        let slide = Motion::FootSlide {
            foot: 0,
            speed_mps,
            distance_m: 0.20,
            direction_degrees,
            squat_depth_m: 0.20,
            squat_period_secs: 3.0,
            settle_secs: 3.0,
            replant_secs: 0.6,
        };
        Scenario::new(
            &format!("foot slide {speed_mps} m/s at {direction_degrees}°"),
            vec![Motion::stand(1.0), slide, slide, slide],
        )
    }

    /// Forward bows to 60° with planted feet while the torso trackers over-read the bend
    /// ([`FlexedSpine::LIVE_LIKE`]); no yaw drift, so every correction is false.
    pub fn flexed_spine_bows(count: u32) -> Scenario {
        let mut scenario = Scenario::new(
            &format!("{count} flexed-spine bows"),
            vec![
                Motion::stand(2.0),
                Motion::torso_bows(count),
                Motion::stand(2.0),
            ],
        );
        scenario.flexed_spine = Some(FlexedSpine::LIVE_LIKE);
        scenario
    }

    /// Repeated 90° turns on the spot with pivoting feet.
    pub fn turn_on_spot(pivot_secs: f64) -> Scenario {
        let turn = |degrees| Motion::Turn {
            degrees,
            pivot_fraction: 0.5,
            pivot_secs,
            replant_secs: 0.5,
        };
        let mut script = vec![Motion::stand(1.0)];
        for degrees in [
            90.0, 90.0, -90.0, -90.0, 90.0, -90.0, 90.0, 90.0, 90.0, -90.0,
        ] {
            script.push(turn(degrees));
            script.push(Motion::stand(0.5));
        }
        Scenario::new(
            &format!("10 turns on the spot, {pivot_secs} s pivots"),
            script,
        )
    }

    /// Walking with 10° common-mode leg drift and a tracking-space jump of the headset.
    pub fn headset_jump(offset_m: f32, at_secs: f64) -> Scenario {
        let mut scenario =
            Scenario::new("headset jump during walking", walk_laps(2)).with_leg_error_degrees(10.0);
        scenario.head_jump = Some(HeadJump {
            at_secs,
            offset: Vec3::new(offset_m, 0.0, 0.0),
        });
        scenario
    }

    /// Walking with 10° common-mode leg drift and stale, NaN-rotation and NaN-headset windows.
    pub fn faults() -> Scenario {
        let mut scenario =
            Scenario::new("stale / held / NaN inputs", walk_laps(3)).with_leg_error_degrees(10.0);
        scenario.faults = vec![
            Fault {
                segment: 2,
                start_secs: 3.0,
                end_secs: 9.0,
                kind: FaultKind::Unusable,
            },
            Fault {
                segment: 4,
                start_secs: 12.0,
                end_secs: 16.0,
                kind: FaultKind::NanRotation,
            },
            Fault {
                segment: 0,
                start_secs: 20.0,
                end_secs: 21.0,
                kind: FaultKind::NanHead,
            },
        ];
        scenario
    }

    /// Ten minutes of mixed activity with independent random-walk drift on every sensor.
    pub fn ten_minute_activity(seed: u64) -> Scenario {
        let block = [
            Motion::stand(10.0),
            Motion::walk(8),
            Motion::turn(180.0),
            Motion::walk(8),
            Motion::turn(180.0),
            Motion::squats(4),
            Motion::weight_shift(16.0),
            Motion::head_turns(12.0),
            Motion::torso_bows(3),
            Motion::turn(90.0),
            Motion::turn(-90.0),
            Motion::stand(5.0),
        ];
        let mut script = Vec::new();
        let mut duration = 0.0;
        while duration < 600.0 {
            for motion in block {
                duration += motion.duration();
                script.push(motion);
            }
        }
        let mut scenario = Scenario::new(
            &format!("10 min activity, random walk, seed {seed}"),
            script,
        );
        scenario.seed = seed;
        for segment in 0..6 {
            scenario.yaw_errors[segment] = YawError::RandomWalk {
                sigma_degrees_per_sqrt_sec: 0.5,
                seed: seed.wrapping_mul(31).wrapping_add(segment as u64 + 1),
            };
        }
        scenario
    }
}
