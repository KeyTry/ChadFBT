//! Contact-consistent leg IK (stance-anchored estimator, section 4).
//!
//! Top-down forward kinematics (FK) from the absolute headset pushes every upstream
//! model error (neck offset, spine shape, bone lengths, residual yaw) into the feet.
//! When the contact layer hypothesises that an ankle is a stationary point, this
//! module distributes part of that residual into the pelvis (root correction),
//! pins the ankle to its anchor (foot lock) and gives an un-instrumented foot a
//! flat orientation. [`SupportFloor`] runs before it (and before the contact
//! layer): it translates the pelvis and legs vertically so the support foot rests
//! on the floor, within a budget that grows with torso flexion.
//!
//! Inputs and outputs use right-handed standing space: meters, +Y up, -Z forward.
//! A segment rotation maps bone-local axes to standing space; bone +Y points from
//! the distal to the proximal joint, +X is the body's right, -Z anatomical forward.
//! Index 0 is the left leg, 1 the right leg.
//!
//! Guarantees:
//! - Segment rotations are never modified. Only joint positions and the
//!   orientation of a foot without a usable foot IMU change.
//! - Hip joint -> knee -> ankle keep the declared thigh and calf lengths whenever
//!   the knee is recomputed; otherwise the leg is translated rigidly.
//! - Disabled, or with nothing planted and every offset decayed, positions equal
//!   the input positions bit for bit.
//! - No input, including NaN, infinities, zero lengths or time going backwards,
//!   produces a nonfinite output. Invalid input holds the previous output and is
//!   reported through [`LegIkStatus`].
//!
//! The output positions are still inferred evidence, not measurements. Synthetic
//! tests show that the implementation follows this model; they do not show that
//! the model matches a worn body.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Offsets and corrections below this magnitude snap to exactly zero, so that a
/// released solver returns to bit-exact FK passthrough in finite time.
const SNAP_M: f32 = 1.0e-6;
/// Largest time step applied to the filters. Longer gaps simply complete decays.
const MAX_DT_SECS: f32 = 1.0;
/// Support floor: largest raise of the body for feet below the floor (meters). A
/// forward fold beyond 90° (torso trackers reading past horizontal) was seen live to
/// put the head-down ankles 0.67 m below the floor; this only rejects absurd input.
const MAX_RAISE_M: f32 = 2.0;
/// Support floor: inputs outside these magnitudes are treated as unusable.
const MAX_SOLE_M: f32 = 1.0;
const MAX_POSITION_M: f32 = 1.0e4;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LegIkConfig {
    /// Move the pelvis (and part of the chest) toward the planted anchors.
    pub root_correction_enabled: bool,
    /// Pin a planted ankle to its anchor and recompute the knee.
    pub foot_lock_enabled: bool,
    /// Blend an un-instrumented planted foot toward flat, heading-only orientation.
    pub flat_foot_enabled: bool,
    /// [`SupportFloor`]: when the lowest usable ankle is below
    /// `ankle_to_sole_m - floor_margin_m`, raise the pelvis and both legs rigidly
    /// (knees are not bent) until it is not. Wrong if the SteamVR floor is wrong.
    pub floor_clamp_enabled: bool,
    /// [`SupportFloor`]: when the lowest usable ankle is above the floor, lower
    /// the pelvis and both legs until it rests at `ankle_to_sole_m`, by at most
    /// the torso compliance budget and only while that shank is near vertical.
    pub support_floor_enabled: bool,
    /// Pull-down budget with an upright torso (meters).
    pub support_floor_base_m: f32,
    /// Budget gain `c` on the torso flexion slack
    /// `neck_to_chest·(1 − cos chest tilt) + chest_to_hip·(1 − cos spine tilt)`.
    pub support_floor_tilt_gain: f32,
    /// Low-pass time constant of the pull-down (raising below the floor is immediate).
    pub support_floor_tau_secs: f32,
    /// The pull-down never lowers the FK pelvis centre below this height (meters).
    pub support_floor_min_hip_height_m: f32,
    /// Gain `k` of the torso's horizontal uncertainty
    /// `k·(neck_to_chest·sin chest tilt + chest_to_hip·sin spine tilt)`, added to
    /// `root_max_m`, `lock_break_m` and `anchor_deadzone_m`. Zero restores the
    /// fixed limits. The default 0.3 makes the lock radius 7 cm + 0.3·0.58 m·sin θ
    /// (≈ 24 cm in a 75° bow), which covers the 2–15 cm horizontal ankle errors the
    /// first live session showed in bows (head-down chain, before any correction).
    pub torso_slack_gain: f32,
    /// Anchor creep ignores this much horizontal anchor-to-FK error.
    pub anchor_deadzone_m: f32,
    /// Time constant of anchor creep toward FK for the horizontal error beyond the
    /// deadzone.
    pub anchor_creep_tau_secs: f32,
    /// Time constant with which a planted anchor's height approaches its target,
    /// without a deadzone. Zero snaps it. The target is the floor ankle height
    /// (`ankle_to_sole_m`) while the floor-corrected FK ankle is up to
    /// `planted_floor_full_m` above it, the FK height beyond `planted_floor_zero_m`
    /// (blended between) and at or below the floor: planted feet near the floor
    /// rest on it.
    pub anchor_vertical_tau_secs: f32,
    pub planted_floor_full_m: f32,
    pub planted_floor_zero_m: f32,
    /// Fraction of the mean anchor error moved into the pelvis.
    pub root_gain: f32,
    /// Magnitude clamp of the root-correction target (plus the torso slack).
    pub root_max_m: f32,
    /// Low-pass (and decay) time constant of the root correction.
    pub root_tau_secs: f32,
    /// Remaining error after root correction above which the lock breaks (plus
    /// the torso slack).
    pub lock_break_m: f32,
    /// The ankle is kept at least this far inside full extension / full flexion.
    pub reach_margin_m: f32,
    /// Decay time constant of the positional offset left by a released lock.
    pub release_tau_secs: f32,
    /// Not in the spec: bounds the release offset so a tracking-space jump does
    /// not drag a foot across the room while it decays.
    pub release_offset_max_m: f32,
    /// Plant weight ramps 0 -> 1 (and back) over this time.
    pub plant_ramp_secs: f32,
    /// Calf tilt from vertical below which the flat-foot tilt weight is 1.
    pub flat_full_tilt_rad: f32,
    /// Calf tilt from vertical above which the flat-foot tilt weight is 0.
    pub flat_zero_tilt_rad: f32,
    /// The captured flat heading follows the calf heading with this time constant.
    pub heading_follow_tau_secs: f32,
    /// Floor clamp tolerance below `ankle_to_sole_m`.
    pub floor_margin_m: f32,
    /// Not in the spec: below this measured knee offset from the hip-ankle line the
    /// bend direction is progressively biased toward the thigh's forward axis, so a
    /// straight FK leg cannot be bent backwards by a small anchor offset.
    pub straight_knee_m: f32,
}

impl Default for LegIkConfig {
    fn default() -> Self {
        Self {
            root_correction_enabled: true,
            foot_lock_enabled: true,
            flat_foot_enabled: true,
            floor_clamp_enabled: true,
            support_floor_enabled: true,
            support_floor_base_m: 0.02,
            support_floor_tilt_gain: 1.0,
            support_floor_tau_secs: 0.06,
            support_floor_min_hip_height_m: 0.15,
            torso_slack_gain: 0.3,
            anchor_deadzone_m: 0.01,
            anchor_creep_tau_secs: 4.0,
            anchor_vertical_tau_secs: 0.3,
            planted_floor_full_m: 0.03,
            planted_floor_zero_m: 0.08,
            root_gain: 0.6,
            root_max_m: 0.05,
            root_tau_secs: 0.06,
            lock_break_m: 0.07,
            reach_margin_m: 0.001,
            release_tau_secs: 0.1,
            release_offset_max_m: 0.25,
            plant_ramp_secs: 0.08,
            flat_full_tilt_rad: 40.0_f32.to_radians(),
            flat_zero_tilt_rad: 60.0_f32.to_radians(),
            heading_follow_tau_secs: 1.0,
            floor_margin_m: 0.01,
            straight_knee_m: 0.02,
        }
    }
}

impl LegIkConfig {
    /// Replace nonfinite or out-of-range numbers with their defaults.
    pub fn sanitized(self) -> Self {
        let d = Self::default();
        let nonneg = |v: f32, def: f32| if v.is_finite() && v >= 0.0 { v } else { def };
        let mut c = Self {
            support_floor_base_m: nonneg(self.support_floor_base_m, d.support_floor_base_m),
            support_floor_tilt_gain: nonneg(
                self.support_floor_tilt_gain,
                d.support_floor_tilt_gain,
            ),
            support_floor_tau_secs: nonneg(self.support_floor_tau_secs, d.support_floor_tau_secs),
            support_floor_min_hip_height_m: if self.support_floor_min_hip_height_m.is_finite() {
                self.support_floor_min_hip_height_m
            } else {
                d.support_floor_min_hip_height_m
            },
            torso_slack_gain: nonneg(self.torso_slack_gain, d.torso_slack_gain),
            anchor_deadzone_m: nonneg(self.anchor_deadzone_m, d.anchor_deadzone_m),
            anchor_creep_tau_secs: nonneg(self.anchor_creep_tau_secs, d.anchor_creep_tau_secs),
            anchor_vertical_tau_secs: nonneg(
                self.anchor_vertical_tau_secs,
                d.anchor_vertical_tau_secs,
            ),
            planted_floor_full_m: nonneg(self.planted_floor_full_m, d.planted_floor_full_m),
            planted_floor_zero_m: nonneg(self.planted_floor_zero_m, d.planted_floor_zero_m),
            root_gain: if self.root_gain.is_finite() {
                self.root_gain.clamp(0.0, 1.0)
            } else {
                d.root_gain
            },
            root_max_m: nonneg(self.root_max_m, d.root_max_m),
            root_tau_secs: nonneg(self.root_tau_secs, d.root_tau_secs),
            lock_break_m: nonneg(self.lock_break_m, d.lock_break_m),
            reach_margin_m: nonneg(self.reach_margin_m, d.reach_margin_m),
            release_tau_secs: nonneg(self.release_tau_secs, d.release_tau_secs),
            release_offset_max_m: nonneg(self.release_offset_max_m, d.release_offset_max_m),
            plant_ramp_secs: nonneg(self.plant_ramp_secs, d.plant_ramp_secs),
            flat_full_tilt_rad: nonneg(self.flat_full_tilt_rad, d.flat_full_tilt_rad),
            flat_zero_tilt_rad: nonneg(self.flat_zero_tilt_rad, d.flat_zero_tilt_rad),
            heading_follow_tau_secs: nonneg(
                self.heading_follow_tau_secs,
                d.heading_follow_tau_secs,
            ),
            floor_margin_m: if self.floor_margin_m.is_finite() {
                self.floor_margin_m
            } else {
                d.floor_margin_m
            },
            straight_knee_m: nonneg(self.straight_knee_m, d.straight_knee_m),
            ..self
        };
        if c.flat_zero_tilt_rad < c.flat_full_tilt_rad {
            c.flat_full_tilt_rad = d.flat_full_tilt_rad;
            c.flat_zero_tilt_rad = d.flat_zero_tilt_rad;
        }
        if c.planted_floor_zero_m < c.planted_floor_full_m {
            c.planted_floor_full_m = d.planted_floor_full_m;
            c.planted_floor_zero_m = d.planted_floor_zero_m;
        }
        c
    }
}

/// One frame of forward kinematics plus contact hypotheses.
#[derive(Clone, Copy, Debug)]
pub struct LegIkInput {
    pub at_secs: f64,
    /// Not moved by IK; validated only.
    pub neck: Vec3,
    pub chest: Vec3,
    pub hip: Vec3,
    pub hip_joints: [Vec3; 2],
    pub knees: [Vec3; 2],
    pub ankles: [Vec3; 2],
    /// Chest and mid-spine (`BodyChain::spine_rotation`) rotations: their tilt
    /// sets the torso slack added to the root-correction and lock-break limits.
    pub chest_rotation: Quat,
    pub spine_rotation: Quat,
    pub hip_rotation: Quat,
    pub thigh_rotations: [Quat; 2],
    pub calf_rotations: [Quat; 2],
    /// A measured, direction-certified foot IMU orientation. Always wins over the
    /// inferred flat foot. An invalid quaternion is treated as `None`.
    pub measured_foot_rotations: [Option<Quat>; 2],
    /// Stationary-point hypotheses from the contact layer.
    pub planted: [bool; 2],
    pub neck_to_chest_m: f32,
    pub chest_to_hip_m: f32,
    /// Unused by the current model.
    pub hip_width_m: f32,
    pub thigh_m: f32,
    pub calf_m: f32,
    pub ankle_to_sole_m: f32,
    pub enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LegIkStatus {
    /// `enabled` was false: positions are the FK input, foot = measured or calf.
    Disabled,
    /// The solver ran on this frame's input.
    Active,
    /// The input was invalid; the previous output is repeated, state untouched.
    HeldLastOutput,
    /// The solver produced a nonfinite intermediate; state was reset and the FK
    /// input passed through unchanged.
    FkPassthrough,
    /// The input was invalid and there is no previous output. All positions are
    /// zero and must not be published as tracking.
    Unavailable,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegIkOutput {
    pub chest: Vec3,
    pub hip: Vec3,
    pub hip_joints: [Vec3; 2],
    pub knees: [Vec3; 2],
    pub ankles: [Vec3; 2],
    pub foot_rotations: [Quat; 2],
    /// The ankle was placed on its anchor this frame.
    pub foot_locked: [bool; 2],
    /// Anchor minus root-corrected FK ankle (0 without an anchor).
    pub lock_error_m: [f32; 2],
    /// The lock broke; the contact layer should force-release this foot. Repeated
    /// every frame until `planted` goes false.
    pub release_requests: [bool; 2],
    /// Current pelvis translation (meters, standing space).
    pub root_correction: [f32; 3],
    /// Applied flat-foot blend weight (0 = rigid with calf, 1 = flat).
    pub flat_weight: [f32; 2],
    /// Lock-break radius and root-correction limit used this frame (meters):
    /// the configured values plus the torso slack.
    pub lock_break_m: f32,
    pub root_max_m: f32,
    pub status: LegIkStatus,
}

/// Serializable diagnostic part of [`LegIkOutput`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LegIkDiagnostics {
    pub foot_locked: [bool; 2],
    pub lock_error_m: [f32; 2],
    pub release_requests: [bool; 2],
    pub root_correction: [f32; 3],
    pub flat_weight: [f32; 2],
    #[serde(default)]
    pub lock_break_m: f32,
    #[serde(default)]
    pub root_max_m: f32,
    pub status: LegIkStatus,
}

impl LegIkOutput {
    pub fn diagnostics(&self) -> LegIkDiagnostics {
        LegIkDiagnostics {
            foot_locked: self.foot_locked,
            lock_error_m: self.lock_error_m,
            release_requests: self.release_requests,
            root_correction: self.root_correction,
            flat_weight: self.flat_weight,
            lock_break_m: self.lock_break_m,
            root_max_m: self.root_max_m,
            status: self.status,
        }
    }
}

/// Tilt of a bone's +Y axis from vertical (radians); 0 for a degenerate rotation.
fn axis_tilt(rotation: Quat) -> f32 {
    match valid_rotation(rotation) {
        Some(q) => (q * Vec3::Y).y.clamp(-1.0, 1.0).acos(),
        None => 0.0,
    }
}

/// How far a flexed torso can be wrong, from the tilts of the chest and of the
/// mid-spine (both measured on soft tissue and modelled as rigid rods).
///
/// `vertical_m = neck_to_chest·(1 − cos θ_chest) + chest_to_hip·(1 − cos θ_spine)` is
/// how much the hip-to-neck height shrinks with flexion, i.e. the height the
/// torso model can lose in a bend; `horizontal_m = neck_to_chest·sin θ_chest +
/// chest_to_hip·sin θ_spine` is the torso's horizontal reach. Both are zero for
/// an upright torso.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TorsoFlexion {
    pub vertical_m: f32,
    pub horizontal_m: f32,
}

impl TorsoFlexion {
    pub fn new(
        chest_rotation: Quat,
        spine_rotation: Quat,
        neck_to_chest_m: f32,
        chest_to_hip_m: f32,
    ) -> Self {
        let (chest, spine) = (axis_tilt(chest_rotation), axis_tilt(spine_rotation));
        let n = if neck_to_chest_m.is_finite() {
            neck_to_chest_m.max(0.0)
        } else {
            0.0
        };
        let h = if chest_to_hip_m.is_finite() {
            chest_to_hip_m.max(0.0)
        } else {
            0.0
        };
        let flexion = Self {
            vertical_m: n * (1.0 - chest.cos()) + h * (1.0 - spine.cos()),
            horizontal_m: n * chest.sin() + h * spine.sin(),
        };
        if flexion.vertical_m.is_finite() && flexion.horizontal_m.is_finite() {
            flexion
        } else {
            Self::default()
        }
    }
}

/// One frame of input for [`SupportFloor`]: forward-kinematics positions before
/// any leg IK, plus the rotations that decide how compliant the torso is.
#[derive(Clone, Copy, Debug)]
pub struct SupportFloorInput {
    pub at_secs: f64,
    /// FK pelvis centre (only its height is used: the pull-down never lowers it
    /// below `support_floor_min_hip_height_m`).
    pub hip: Vec3,
    pub ankles: [Vec3; 2],
    pub calf_rotations: [Quat; 2],
    pub chest_rotation: Quat,
    pub spine_rotation: Quat,
    /// The foot's whole chain (chest, hip, thigh, calf) is valid this frame.
    pub usable: [bool; 2],
    pub neck_to_chest_m: f32,
    pub chest_to_hip_m: f32,
    pub ankle_to_sole_m: f32,
}

/// Serializable per-frame report of [`SupportFloor`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SupportFloorDiagnostics {
    /// The correction ran this frame (leg IK live and at least one switch on).
    pub active: bool,
    /// Vertical translation of the pelvis and both legs (meters, +up). The chest
    /// moves by the spine fraction of it; the neck and head never move.
    pub vertical_m: f32,
    /// Largest pull-down allowed this frame (meters).
    pub budget_m: f32,
    /// Lowest usable FK ankle (`0` left, `1` right), if any.
    pub support_foot: Option<u8>,
    /// Shank-tilt weight of the support foot: 1 within `flat_full_tilt_rad` of
    /// vertical, 0 beyond `flat_zero_tilt_rad`.
    pub support_weight: f32,
    /// Lowest usable FK ankle height minus `ankle_to_sole_m`, before correction.
    pub gap_m: f32,
    /// The budget, the shank weight or the pelvis height guard left the support
    /// ankle above the floor by more than 5 mm.
    pub limited: bool,
    /// The lowest ankle was below the floor and the body was raised.
    pub raised: bool,
}

/// Support-foot floor constraint with torso compliance.
///
/// The legs are rigid bones with well-coupled trackers; a flexed spine is not a
/// rigid rod and waist-worn trackers ride on soft tissue, so in a bend the
/// head-down chain is typically too short and leaves planted feet in the air.
/// This treats the lower torso as the compliant, unmeasured part: the pelvis
/// and both legs are translated vertically (no rotation changes, bone lengths
/// kept) so that the lowest usable ankle rests on the floor, but a pull-down
/// may not exceed `base + gain·TorsoFlexion::vertical_m` and only applies while
/// that shank is near vertical (kneeling, sitting or lying on the floor are left
/// alone). An ankle below the floor raises the body by the full amount at once.
///
/// It runs before contacts and leg IK so anchors are captured on the corrected
/// skeleton. Positions stay inferred evidence: this is a model assumption that
/// the support foot is on the floor, not a measurement.
#[derive(Clone, Debug)]
pub struct SupportFloor {
    config: LegIkConfig,
    last_at: Option<f64>,
    vertical: f32,
}

impl SupportFloor {
    pub fn new(config: LegIkConfig) -> Self {
        Self {
            config: config.sanitized(),
            last_at: None,
            vertical: 0.0,
        }
    }

    pub fn set_config(&mut self, config: LegIkConfig) {
        self.config = config.sanitized();
    }

    pub fn reset(&mut self) {
        self.last_at = None;
        self.vertical = 0.0;
    }

    /// The vertical correction for this frame. Invalid input decays nothing and
    /// returns an inactive report with zero correction for this frame.
    pub fn update(&mut self, input: &SupportFloorInput) -> SupportFloorDiagnostics {
        let cfg = self.config;
        let mut out = SupportFloorDiagnostics::default();
        if !(cfg.support_floor_enabled || cfg.floor_clamp_enabled) || !input.at_secs.is_finite() {
            self.reset();
            return out;
        }
        let dt = match self.last_at {
            Some(last) if input.at_secs > last => ((input.at_secs - last) as f32).min(MAX_DT_SECS),
            _ => 0.0,
        };
        self.last_at = Some(input.at_secs);
        let floor = input.ankle_to_sole_m;
        let support = (0..2)
            .filter(|&s| {
                input.usable[s]
                    && input.ankles[s].is_finite()
                    && valid_rotation(input.calf_rotations[s]).is_some()
            })
            .min_by(|&a, &b| input.ankles[a].y.total_cmp(&input.ankles[b].y));
        let follow = 1.0 - keep(dt, cfg.support_floor_tau_secs);
        let plausible = (0.0..=MAX_SOLE_M).contains(&floor)
            && input.hip.y.is_finite()
            && support.is_some_and(|s| input.ankles[s].y.abs() <= MAX_POSITION_M);
        let Some(s) = support.filter(|_| plausible) else {
            // Nothing to stand on: let the correction fade.
            self.vertical -= self.vertical * follow;
            if self.vertical.abs() < SNAP_M || !self.vertical.is_finite() {
                self.vertical = 0.0;
            }
            out.vertical_m = self.vertical;
            out.active = true;
            return out;
        };
        let gap = input.ankles[s].y - floor;
        let tilt = axis_tilt(input.calf_rotations[s]);
        let weight = 1.0 - smoothstep(cfg.flat_full_tilt_rad, cfg.flat_zero_tilt_rad, tilt);
        let flexion = TorsoFlexion::new(
            input.chest_rotation,
            input.spine_rotation,
            input.neck_to_chest_m,
            input.chest_to_hip_m,
        );
        let budget = cfg.support_floor_base_m + cfg.support_floor_tilt_gain * flexion.vertical_m;
        let hip_room = (input.hip.y - cfg.support_floor_min_hip_height_m).max(0.0);
        let pull = if cfg.support_floor_enabled {
            gap.max(0.0).min(budget).min(hip_room) * weight
        } else {
            0.0
        };
        self.vertical += (-pull - self.vertical) * follow;
        if !self.vertical.is_finite() {
            self.vertical = 0.0;
        }
        // Never below the floor: raise at once (also catches a pull-down that
        // lags a quickly descending support foot). MAX_RAISE_M only bounds absurd
        // input; a real fold can need well over half a meter.
        let lowest = (-gap - cfg.floor_margin_m).min(MAX_RAISE_M);
        if cfg.floor_clamp_enabled && self.vertical < lowest {
            self.vertical = lowest;
            out.raised = lowest > 0.0;
        }
        if self.vertical.abs() < SNAP_M {
            self.vertical = 0.0;
        }
        out.active = true;
        out.vertical_m = self.vertical;
        out.budget_m = budget;
        out.support_foot = Some(s as u8);
        out.support_weight = weight;
        out.gap_m = gap;
        out.limited = cfg.support_floor_enabled && gap - pull > 0.005;
        out
    }
}

/// The share of a pelvis translation that the chest takes: linear along the
/// spine from the fixed neck (0) to the pelvis (1).
pub fn chest_fraction(neck_to_chest_m: f32, chest_to_hip_m: f32) -> f32 {
    let total = neck_to_chest_m + chest_to_hip_m;
    if total.is_finite() && total > 1.0e-6 {
        (neck_to_chest_m / total).clamp(0.0, 1.0)
    } else {
        0.5
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct FootState {
    anchor: Option<Vec3>,
    /// The lock broke while the contact layer still reports `planted`.
    broken: bool,
    release_offset: Vec3,
    plant_weight: f32,
    flat_heading: Option<f32>,
    published_ankle: Option<Vec3>,
}

#[derive(Clone, Debug)]
pub struct LegIkSolver {
    config: LegIkConfig,
    last_at: Option<f64>,
    correction: Vec3,
    feet: [FootState; 2],
    last_output: Option<LegIkOutput>,
}

impl LegIkSolver {
    pub fn new(config: LegIkConfig) -> Self {
        Self {
            config: config.sanitized(),
            last_at: None,
            correction: Vec3::ZERO,
            feet: [FootState::default(); 2],
            last_output: None,
        }
    }

    pub fn config(&self) -> &LegIkConfig {
        &self.config
    }

    /// Takes effect on the next solve; dynamic state is kept so that switches
    /// fade instead of jumping where the filters allow it.
    pub fn set_config(&mut self, config: LegIkConfig) {
        self.config = config.sanitized();
    }

    /// Drop anchors, corrections, offsets and history.
    pub fn reset(&mut self) {
        self.last_at = None;
        self.correction = Vec3::ZERO;
        self.feet = [FootState::default(); 2];
        self.last_output = None;
    }

    pub fn solve(&mut self, input: &LegIkInput) -> LegIkOutput {
        if !input_valid(input) {
            return match self.last_output {
                Some(mut held) => {
                    held.status = LegIkStatus::HeldLastOutput;
                    held.release_requests = [false; 2];
                    held
                }
                None => unavailable(),
            };
        }
        if !input.enabled {
            self.reset();
            let out = passthrough(input);
            self.last_output = Some(out);
            return out;
        }
        let out = self.solve_enabled(input);
        self.last_output = Some(out);
        out
    }

    fn advance_time(&mut self, at: f64) -> f32 {
        let dt = match self.last_at {
            Some(last) if at > last => ((at - last) as f32).min(MAX_DT_SECS),
            _ => 0.0,
        };
        // Time going backwards re-bases the clock without advancing any filter.
        self.last_at = Some(at);
        dt
    }

    fn solve_enabled(&mut self, input: &LegIkInput) -> LegIkOutput {
        let cfg = self.config;
        let dt = self.advance_time(input.at_secs);
        let use_anchors = cfg.root_correction_enabled || cfg.foot_lock_enabled;
        let release_keep = keep(dt, cfg.release_tau_secs);
        let creep = 1.0 - keep(dt, cfg.anchor_creep_tau_secs);
        let vertical_creep = 1.0 - keep(dt, cfg.anchor_vertical_tau_secs);
        // A flexed torso is the least certain part of the chain: its horizontal
        // uncertainty widens the root-correction and lock-break limits and the
        // anchor-creep deadzone, so a bow neither breaks nor drags a planted foot.
        let slack = cfg.torso_slack_gain
            * TorsoFlexion::new(
                input.chest_rotation,
                input.spine_rotation,
                input.neck_to_chest_m,
                input.chest_to_hip_m,
            )
            .horizontal_m;
        let deadzone = cfg.anchor_deadzone_m + slack;

        // Step 1: release-offset decay, anchor capture / creep, unplant release.
        let prev_correction = self.correction;
        let mut unplanted = [false; 2];
        for (s, foot) in self.feet.iter_mut().enumerate() {
            foot.release_offset *= release_keep;
            if foot.release_offset.length() < SNAP_M {
                foot.release_offset = Vec3::ZERO;
            }
            let planted = input.planted[s] && use_anchors;
            if !planted {
                if foot.anchor.take().is_some() {
                    unplanted[s] = true;
                }
                foot.broken = false;
            } else if foot.broken {
                // Waiting for the contact layer to honour the release request.
            } else if let Some(anchor) = foot.anchor.as_mut() {
                let error = input.ankles[s] - *anchor;
                // Horizontal: slow creep beyond the deadzone only.
                let horizontal = Vec3::new(error.x, 0.0, error.z);
                let distance = horizontal.length();
                if distance > deadzone {
                    *anchor += horizontal * ((distance - deadzone) / distance * creep);
                }
                // Vertical: a planted foot near the floor rests on it; one clearly
                // above it (a step, a box) follows the floor-corrected FK height. No
                // deadzone, so an anchor captured with a leftover swing offset does
                // not keep a planted foot hovering or sunk, and a torso roll error
                // that lifts only the other leg's FK ankle does not lift that foot.
                // At or just below the floor (within the support floor's margin)
                // the FK height is kept, so this never fights `SupportFloor`.
                let above = input.ankles[s].y - input.ankle_to_sole_m;
                let free = smoothstep(cfg.planted_floor_full_m, cfg.planted_floor_zero_m, above);
                let target = if above <= 0.0 {
                    input.ankles[s].y
                } else {
                    input.ankle_to_sole_m + above * free
                };
                anchor.y += (target - anchor.y) * vertical_creep;
            } else {
                // Capture where the foot is currently shown. With no correction
                // and no offset this is exactly the FK ankle (spec); otherwise it
                // avoids a pop when a foot plants while the pelvis is corrected.
                foot.anchor = Some(input.ankles[s] + prev_correction + foot.release_offset);
                foot.release_offset = Vec3::ZERO;
            }
        }

        // Step 2: root correction.
        let mut sum = Vec3::ZERO;
        let mut count = 0.0_f32;
        for (s, foot) in self.feet.iter().enumerate() {
            if let Some(anchor) = foot.anchor {
                sum += anchor - input.ankles[s];
                count += 1.0;
            }
        }
        let root_max = cfg.root_max_m + slack;
        let lock_break = cfg.lock_break_m + slack;
        let target = if cfg.root_correction_enabled && count > 0.0 {
            (sum / count * cfg.root_gain).clamp_length_max(root_max)
        } else {
            Vec3::ZERO
        };
        self.correction += (target - self.correction) * (1.0 - keep(dt, cfg.root_tau_secs));
        if target == Vec3::ZERO && self.correction.length() < SNAP_M {
            self.correction = Vec3::ZERO;
        }
        if !self.correction.is_finite() {
            self.correction = Vec3::ZERO;
        }
        let c = self.correction;
        let chest_fraction = chest_fraction(input.neck_to_chest_m, input.chest_to_hip_m);

        let mut out = LegIkOutput {
            chest: offset(input.chest, c * chest_fraction),
            hip: offset(input.hip, c),
            hip_joints: [Vec3::ZERO; 2],
            knees: [Vec3::ZERO; 2],
            ankles: [Vec3::ZERO; 2],
            foot_rotations: [Quat::IDENTITY; 2],
            foot_locked: [false; 2],
            lock_error_m: [0.0; 2],
            release_requests: [false; 2],
            root_correction: c.to_array(),
            flat_weight: [0.0; 2],
            lock_break_m: lock_break,
            root_max_m: root_max,
            status: LegIkStatus::Active,
        };

        for s in 0..2 {
            let foot = &mut self.feet[s];
            let rigid = offset(input.ankles[s], c);
            let release_from_published = |foot: &FootState| {
                foot.published_ankle
                    .map(|p| (p - rigid).clamp_length_max(cfg.release_offset_max_m))
                    .filter(|o| o.is_finite())
                    .unwrap_or(Vec3::ZERO)
            };
            if unplanted[s] && cfg.foot_lock_enabled {
                foot.release_offset = release_from_published(foot);
            }

            // Step 3: foot lock.
            let mut target = rigid;
            if let Some(anchor) = foot.anchor {
                let remaining = anchor - rigid;
                let error = remaining.length();
                out.lock_error_m[s] = error;
                if error > lock_break {
                    foot.anchor = None;
                    foot.broken = true;
                    if cfg.foot_lock_enabled {
                        foot.release_offset = release_from_published(foot);
                    }
                } else if cfg.foot_lock_enabled {
                    target = anchor;
                    out.foot_locked[s] = true;
                }
            }
            if foot.broken && input.planted[s] {
                out.release_requests[s] = true;
            }
            if !out.foot_locked[s] && foot.release_offset != Vec3::ZERO {
                target = rigid + foot.release_offset;
            }
            // The floor is handled before this solver by `SupportFloor`, which
            // moves the whole lower body instead of bending a knee.

            let hip_joint = offset(input.hip_joints[s], c);
            let (knee, ankle) = if target == rigid {
                (offset(input.knees[s], c), rigid)
            } else {
                two_bone(
                    hip_joint,
                    target,
                    input.hip_joints[s],
                    input.knees[s],
                    input.ankles[s],
                    input.thigh_rotations[s].normalize(),
                    input.thigh_m,
                    input.calf_m,
                    cfg.reach_margin_m,
                    cfg.straight_knee_m,
                )
            };
            out.hip_joints[s] = hip_joint;
            out.knees[s] = knee;
            out.ankles[s] = ankle;
            foot.published_ankle = Some(ankle);

            // Step 4: foot orientation.
            let calf = input.calf_rotations[s].normalize();
            let flat_planted = input.planted[s] && !foot.broken;
            foot.plant_weight = if cfg.plant_ramp_secs <= 0.0 {
                if flat_planted { 1.0 } else { 0.0 }
            } else {
                let step = dt / cfg.plant_ramp_secs;
                if flat_planted {
                    (foot.plant_weight + step).min(1.0)
                } else {
                    (foot.plant_weight - step).max(0.0)
                }
            };
            let current_heading = lateral_heading(calf);
            if let Some(heading) = foot.flat_heading.as_mut() {
                if let Some(current) = current_heading {
                    let follow = 1.0 - keep(dt, cfg.heading_follow_tau_secs);
                    *heading = wrap_angle(*heading + wrap_angle(current - *heading) * follow);
                }
            } else if flat_planted {
                // Captured at plant time; the lateral axis is undefined only for a
                // strongly rolled calf, where the twist yaw is the best available.
                foot.flat_heading = Some(current_heading.unwrap_or_else(|| twist_yaw(calf)));
            }
            if !flat_planted && foot.plant_weight <= 0.0 {
                foot.flat_heading = None;
            }
            let measured = input.measured_foot_rotations[s].and_then(valid_rotation);
            out.foot_rotations[s] = if let Some(measured) = measured {
                measured
            } else {
                let tilt = (calf * Vec3::Y).y.clamp(-1.0, 1.0).acos();
                let tilt_weight =
                    1.0 - smoothstep(cfg.flat_full_tilt_rad, cfg.flat_zero_tilt_rad, tilt);
                let weight = if cfg.flat_foot_enabled {
                    foot.plant_weight * tilt_weight
                } else {
                    0.0
                };
                match foot.flat_heading {
                    Some(heading) if weight > 0.0 => {
                        out.flat_weight[s] = weight;
                        let flat = Quat::from_rotation_y(heading);
                        let flat = if flat.dot(calf) < 0.0 { -flat } else { flat };
                        calf.slerp(flat, weight).normalize()
                    }
                    _ => calf,
                }
            };
        }
        if !output_finite(&out) {
            // Defensive: unreachable for validated input. Never publish it.
            self.reset();
            let mut fk = passthrough(input);
            fk.status = LegIkStatus::FkPassthrough;
            return fk;
        }
        out
    }
}

/// Two-bone IK from `hip` toward `target`, preserving the measured bend direction.
///
/// The FK triangle's knee offset from its hip-ankle line is carried to the new
/// hip-target line by the minimal rotation between the two lines. When the FK leg
/// is (nearly) straight that offset is (nearly) zero, and the thigh's forward
/// axis takes over continuously.
#[allow(clippy::too_many_arguments)]
fn two_bone(
    hip: Vec3,
    target: Vec3,
    fk_hip: Vec3,
    fk_knee: Vec3,
    fk_ankle: Vec3,
    thigh_rotation: Quat,
    thigh: f32,
    calf: f32,
    margin: f32,
    straight_knee_m: f32,
) -> (Vec3, Vec3) {
    let fk_line = fk_ankle - fk_hip;
    let fk_length = fk_line.length();
    let to_target = target - hip;
    let distance = to_target.length();
    let direction = if distance > 1.0e-6 {
        to_target / distance
    } else if fk_length > 1.0e-6 {
        fk_line / fk_length
    } else {
        thigh_rotation * Vec3::NEG_Y
    };
    let min_reach = (thigh - calf).abs() + margin;
    let max_reach = (thigh + calf - margin).max(min_reach);
    let reach = distance.clamp(min_reach, max_reach);
    let ankle = hip + direction * reach;
    let along = (thigh * thigh - calf * calf + reach * reach) / (2.0 * reach);
    let height = (thigh * thigh - along * along).max(0.0).sqrt();

    let fk_direction = if fk_length > 1.0e-6 {
        fk_line / fk_length
    } else {
        direction
    };
    let fk_offset = fk_knee - fk_hip;
    let fk_bend = fk_offset - fk_direction * fk_offset.dot(fk_direction);
    let carried = Quat::from_rotation_arc(fk_direction, direction) * fk_bend;
    let carried = perpendicular(carried, direction);
    let forward = perpendicular(thigh_rotation * Vec3::NEG_Z, direction).normalize_or_zero();
    let mut bend = carried + forward * (straight_knee_m - fk_bend.length()).max(0.0);
    if bend.length_squared() < 1.0e-12 {
        bend = [
            thigh_rotation * Vec3::NEG_Z,
            thigh_rotation * Vec3::Y,
            Vec3::NEG_Z,
            Vec3::X,
        ]
        .into_iter()
        .map(|candidate| perpendicular(candidate, direction))
        .find(|candidate| candidate.length_squared() > 1.0e-6)
        .unwrap_or(Vec3::NEG_Z);
    }
    let knee = hip + direction * along + bend.normalize_or_zero() * height;
    (knee, ankle)
}

fn perpendicular(v: Vec3, unit: Vec3) -> Vec3 {
    v - unit * v.dot(unit)
}

fn offset(v: Vec3, delta: Vec3) -> Vec3 {
    // Skipping a zero offset keeps passthrough bit-exact (e.g. -0.0 + 0.0).
    if delta == Vec3::ZERO { v } else { v + delta }
}

/// exp(-dt/tau), the fraction of a first-order state that survives `dt`.
fn keep(dt: f32, tau: f32) -> f32 {
    if dt <= 0.0 {
        1.0
    } else if tau <= 0.0 {
        0.0
    } else {
        (-dt / tau).exp()
    }
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    if edge1 <= edge0 {
        return if x < edge0 { 0.0 } else { 1.0 };
    }
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

/// Heading of the lateral axis `a = R·X`: `atan2(-a.z, a.x)`, defined only when
/// the axis is at least 45 degrees from vertical (`a.x² + a.z² >= 0.5`).
fn lateral_heading(rotation: Quat) -> Option<f32> {
    let a = rotation * Vec3::X;
    (a.x * a.x + a.z * a.z >= 0.5).then(|| (-a.z).atan2(a.x))
}

/// Yaw of the twist about +Y; fallback when the lateral axis is near vertical.
fn twist_yaw(rotation: Quat) -> f32 {
    if rotation.y.abs() < 1.0e-9 && rotation.w.abs() < 1.0e-9 {
        0.0
    } else {
        2.0 * rotation.y.atan2(rotation.w)
    }
}

fn valid_rotation(q: Quat) -> Option<Quat> {
    let length = q.length();
    (q.is_finite() && (0.5..=2.0).contains(&length)).then(|| q / length)
}

fn input_valid(input: &LegIkInput) -> bool {
    let positions = [input.neck, input.chest, input.hip]
        .into_iter()
        .chain(input.hip_joints)
        .chain(input.knees)
        .chain(input.ankles);
    let mut rotations = [input.hip_rotation]
        .into_iter()
        .chain(input.thigh_rotations)
        .chain(input.calf_rotations);
    let positive = |v: f32| v.is_finite() && v > 1.0e-3;
    let nonneg = |v: f32| v.is_finite() && v >= 0.0;
    input.at_secs.is_finite()
        && positions.into_iter().all(|p| p.is_finite())
        && rotations.all(|q| valid_rotation(q).is_some())
        && positive(input.thigh_m)
        && positive(input.calf_m)
        && nonneg(input.neck_to_chest_m)
        && nonneg(input.chest_to_hip_m)
        && input.ankle_to_sole_m.is_finite()
}

fn passthrough(input: &LegIkInput) -> LegIkOutput {
    let foot = |s: usize| {
        input.measured_foot_rotations[s]
            .and_then(valid_rotation)
            .unwrap_or_else(|| input.calf_rotations[s].normalize())
    };
    LegIkOutput {
        chest: input.chest,
        hip: input.hip,
        hip_joints: input.hip_joints,
        knees: input.knees,
        ankles: input.ankles,
        foot_rotations: [foot(0), foot(1)],
        foot_locked: [false; 2],
        lock_error_m: [0.0; 2],
        release_requests: [false; 2],
        root_correction: [0.0; 3],
        flat_weight: [0.0; 2],
        lock_break_m: 0.0,
        root_max_m: 0.0,
        status: LegIkStatus::Disabled,
    }
}

fn unavailable() -> LegIkOutput {
    LegIkOutput {
        chest: Vec3::ZERO,
        hip: Vec3::ZERO,
        hip_joints: [Vec3::ZERO; 2],
        knees: [Vec3::ZERO; 2],
        ankles: [Vec3::ZERO; 2],
        foot_rotations: [Quat::IDENTITY; 2],
        foot_locked: [false; 2],
        lock_error_m: [0.0; 2],
        release_requests: [false; 2],
        root_correction: [0.0; 3],
        flat_weight: [0.0; 2],
        lock_break_m: 0.0,
        root_max_m: 0.0,
        status: LegIkStatus::Unavailable,
    }
}

fn output_finite(out: &LegIkOutput) -> bool {
    [out.chest, out.hip]
        .into_iter()
        .chain(out.hip_joints)
        .chain(out.knees)
        .chain(out.ankles)
        .all(|p| p.is_finite())
        && out.foot_rotations.iter().all(|q| q.is_finite())
        && out.lock_error_m.iter().all(|v| v.is_finite())
        && out.flat_weight.iter().all(|v| v.is_finite())
        && out.root_correction.iter().all(|v| v.is_finite())
}
