//! Shared head-anchored kinematic chain with analytic yaw sensitivities.
//!
//! One function, [`solve_chain`], maps the absolute headset pose and six calibrated core
//! segment rotations to inferred joint positions. The stance heading filter linearises
//! exactly this function, so the filter model and a published skeleton built from it
//! cannot diverge.
//!
//! Frames follow `AGENTS.md`: right-handed standing space, meters, +Y up, -Z forward.
//! A bone rotation maps bone-local axes to standing space: bone +Y points from the distal
//! to the proximal joint, +X is the body's right (the flexion hinge axis), -Z is anatomical
//! forward. Segment indices follow [`crate::Role::CORE`]:
//! `0 Chest, 1 Hip, 2 LeftThigh, 3 RightThigh, 4 LeftCalf, 5 RightCalf`.
//! Foot index `0 = left` (lateral sign -1), `1 = right` (lateral sign +1).
//!
//! All positions produced here are inferred from orientations plus an assumed body model;
//! they are not measurements.

use crate::Anthropometrics;
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Segment index of the chest sensor in `Role::CORE` order.
pub const CHEST: usize = 0;
/// Segment index of the hip (pelvis) sensor in `Role::CORE` order.
pub const HIP: usize = 1;
/// Thigh segment index per foot (`[left, right]`).
pub const THIGH: [usize; 2] = [2, 3];
/// Calf segment index per foot (`[left, right]`).
pub const CALF: [usize; 2] = [4, 5];
/// Lateral sign `s` per foot: left `-1`, right `+1`.
pub const LATERAL_SIGN: [f32; 2] = [-1.0, 1.0];
/// Segments whose world yaw moves each foot's ankle (`[chest, hip, thigh_s, calf_s]`).
pub const FOOT_CHAIN: [[usize; 4]; 2] = [[CHEST, HIP, 2, 4], [CHEST, HIP, 3, 5]];

/// Assumed body dimensions used by the chain, in meters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainDimensions {
    /// Vertical distance from the tracked eye point down to the neck pivot (head frame).
    pub head_to_neck: f32,
    /// Distance the tracked eye point lies in front of the neck pivot (head frame).
    pub eye_forward: f32,
    pub neck_to_chest: f32,
    pub chest_to_hip: f32,
    /// Distance between the two hip joint centres.
    pub hip_width: f32,
    pub thigh: f32,
    pub calf: f32,
}

impl ChainDimensions {
    /// Default eye-to-neck forward offset when nothing better is known.
    pub const DEFAULT_EYE_FORWARD_M: f32 = 0.08;

    pub fn from_anthropometrics(body: &Anthropometrics, eye_forward_m: f32) -> Self {
        Self {
            head_to_neck: body.head_to_neck_m,
            eye_forward: eye_forward_m,
            neck_to_chest: body.neck_to_chest_m,
            chest_to_hip: body.chest_to_hip_m,
            hip_width: body.hip_width_m,
            thigh: body.thigh_length_m,
            calf: body.calf_length_m,
        }
    }

    /// All dimensions finite, non-negative and below 3 m.
    pub fn is_valid(&self) -> bool {
        [
            self.head_to_neck,
            self.eye_forward,
            self.neck_to_chest,
            self.chest_to_hip,
            self.hip_width,
            self.thigh,
            self.calf,
        ]
        .iter()
        .all(|value| (0.0..3.0).contains(value))
    }
}

impl Default for ChainDimensions {
    fn default() -> Self {
        Self::from_anthropometrics(&Anthropometrics::default(), Self::DEFAULT_EYE_FORWARD_M)
    }
}

/// Inputs of the chain: absolute headset eye pose plus calibrated (and corrected) segment
/// rotations in `Role::CORE` order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChainPose {
    pub head_position: Vec3,
    pub head_rotation: Quat,
    pub rotations: [Quat; 6],
}

/// Result of [`solve_chain`].
///
/// `levers[s][i]` is the sum of the chain's bone vectors between the headset eye point and
/// ankle `s` that rotate rigidly with a world-yaw change of segment `i` (zero for the other
/// leg's segments). The exact identity `ankles[s] = head_position + head_lever + Σ_i
/// levers[s][i]` holds, and `∂ankles[s]/∂δ_i ≈ yaw_sensitivity(levers[s][i])` for a left
/// world-yaw perturbation `Ry(δ_i)` of segment `i` (the chest/hip split of the spine is the
/// first-order sensitivity of the mid-spine slerp).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyChain {
    pub neck: Vec3,
    pub chest: Vec3,
    pub hip: Vec3,
    pub spine_rotation: Quat,
    pub hip_joints: [Vec3; 2],
    pub knees: [Vec3; 2],
    pub ankles: [Vec3; 2],
    pub levers: [[Vec3; 6]; 2],
    /// `neck - head_position`; depends on the absolute headset only.
    pub head_lever: Vec3,
    /// False when the head pose or the dimensions were nonfinite/degenerate. The affected
    /// values were replaced by zero/identity so that the output stays finite; such a chain
    /// carries no information and must not be used as valid tracking.
    pub head_finite: bool,
    /// False for each segment whose rotation was nonfinite or zero-length and was replaced by
    /// identity; every position downstream of that segment is then meaningless.
    pub segment_finite: [bool; 6],
}

impl BodyChain {
    /// True when every input that foot `foot`'s ankle depends on was finite.
    pub fn foot_finite(&self, foot: usize) -> bool {
        foot < 2 && self.head_finite && FOOT_CHAIN[foot].iter().all(|&i| self.segment_finite[i])
    }
}

/// `ŷ × lever`: the first-order change of a lever per radian of left world yaw.
pub fn yaw_sensitivity(lever: Vec3) -> Vec3 {
    Vec3::new(lever.z, 0.0, -lever.x)
}

/// Horizontal heading of a segment's lateral axis `a = q·X`: `atan2(-a.z, a.x)`.
///
/// Equals `θ` for `q = Ry(θ)`. Defined only when `a.x² + a.z² ≥ 0.5` (the lateral axis is
/// within 45° of horizontal); `None` otherwise or for nonfinite input.
pub fn lateral_heading(q: Quat) -> Option<f32> {
    let (q, ok) = unit_quat(q);
    if !ok {
        return None;
    }
    let a = q * Vec3::X;
    let horizontal = a.x * a.x + a.z * a.z;
    if horizontal >= 0.5 {
        Some((-a.z).atan2(a.x))
    } else {
        None
    }
}

/// Normalises `q` when it is finite and non-degenerate. Returns identity and `false`
/// otherwise. A rotation that is already unit length is returned unchanged.
pub(crate) fn unit_quat(q: Quat) -> (Quat, bool) {
    if !q.is_finite() {
        return (Quat::IDENTITY, false);
    }
    let length_squared = q.length_squared();
    if !length_squared.is_finite() || length_squared <= 1.0e-12 {
        return (Quat::IDENTITY, false);
    }
    if (length_squared - 1.0).abs() <= 1.0e-6 {
        (q, true)
    } else {
        (q / length_squared.sqrt(), true)
    }
}

/// Solves the shared chain:
///
/// ```text
/// neck       = p_head + R_head·(0, −head_to_neck, +eye_forward)
/// chest      = neck  − (R_chest·Y)·neck_to_chest
/// R_spine    = slerp(R_chest, R_hip, 0.5)
/// hip        = chest − (R_spine·Y)·chest_to_hip
/// hipJoint_s = hip   + (R_hip·X)·(s·hip_width/2)
/// knee_s     = hipJoint_s − (R_thigh_s·Y)·thigh
/// ankle_s    = knee_s     − (R_calf_s·Y)·calf
/// ```
///
/// Never panics and never returns nonfinite values: nonfinite inputs are replaced and
/// flagged in [`BodyChain::head_finite`] / [`BodyChain::segment_finite`].
pub fn solve_chain(dims: &ChainDimensions, pose: &ChainPose) -> BodyChain {
    let dims_ok = dims.is_valid();
    let d = if dims_ok {
        *dims
    } else {
        ChainDimensions {
            head_to_neck: 0.0,
            eye_forward: 0.0,
            neck_to_chest: 0.0,
            chest_to_hip: 0.0,
            hip_width: 0.0,
            thigh: 0.0,
            calf: 0.0,
        }
    };
    let position_ok = pose.head_position.is_finite();
    let head_position = if position_ok {
        pose.head_position
    } else {
        Vec3::ZERO
    };
    let (head_rotation, head_rotation_ok) = unit_quat(pose.head_rotation);
    let mut r = [Quat::IDENTITY; 6];
    let mut segment_finite = [false; 6];
    for (i, rotation) in pose.rotations.iter().enumerate() {
        (r[i], segment_finite[i]) = unit_quat(*rotation);
    }

    // Same operation order as the legacy inline forward kinematics, so that eye_forward = 0
    // reproduces it to the last bit.
    let head_lever =
        -(head_rotation * Vec3::Y) * d.head_to_neck + (head_rotation * Vec3::Z) * d.eye_forward;
    let neck = head_position + head_lever;
    let chest_axis = r[CHEST] * Vec3::Y;
    let chest = neck - chest_axis * d.neck_to_chest;
    let spine_rotation = r[CHEST].slerp(r[HIP], 0.5).normalize();
    let spine_axis = spine_rotation * Vec3::Y;
    let hip = chest - spine_axis * d.chest_to_hip;
    let half_spine = -spine_axis * (0.5 * d.chest_to_hip);
    let chest_lever = -chest_axis * d.neck_to_chest + half_spine;

    let mut hip_joints = [Vec3::ZERO; 2];
    let mut knees = [Vec3::ZERO; 2];
    let mut ankles = [Vec3::ZERO; 2];
    let mut levers = [[Vec3::ZERO; 6]; 2];
    for foot in 0..2 {
        let lateral = r[HIP] * Vec3::X * (LATERAL_SIGN[foot] * d.hip_width * 0.5);
        hip_joints[foot] = hip + lateral;
        let thigh_axis = r[THIGH[foot]] * Vec3::Y;
        let calf_axis = r[CALF[foot]] * Vec3::Y;
        knees[foot] = hip_joints[foot] - thigh_axis * d.thigh;
        ankles[foot] = knees[foot] - calf_axis * d.calf;
        levers[foot][CHEST] = chest_lever;
        levers[foot][HIP] = half_spine + lateral;
        levers[foot][THIGH[foot]] = -thigh_axis * d.thigh;
        levers[foot][CALF[foot]] = -calf_axis * d.calf;
    }

    BodyChain {
        neck,
        chest,
        hip,
        spine_rotation,
        hip_joints,
        knees,
        ankles,
        levers,
        head_lever,
        head_finite: dims_ok && position_ok && head_rotation_ok,
        segment_finite,
    }
}
