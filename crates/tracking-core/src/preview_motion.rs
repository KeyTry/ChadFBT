//! Experimental desktop displacement from leg geometry and contact hypotheses.
//!
//! This is a kinematic preview prior, not an absolute-position measurement. The
//! lowest inferred ankle supplies a floor-support hypothesis; a likely planted
//! ankle supplies horizontal displacement while the other leg swings. Contact is
//! never measured. Sliding, hopping, moving platforms, a uniform body turn, and
//! translation with unchanged segment angles are not observable from these inputs.
//! No acceleration is integrated and this module never makes a target valid.

use crate::{Anthropometrics, JointRole, Role, TrackingFrame};
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

const MAX_STEP_SECS: f64 = 0.05;
const MAX_GAP_SECS: f64 = 0.25;
const MAX_ORIENTATION_AGE_SECS: f64 = 0.25;
const MAX_HORIZONTAL_SPEED_MPS: f32 = 1.6;
const MAX_VERTICAL_SPEED_MPS: f32 = 1.2;
const CONTACT_RELEASE_HEIGHT_M: f32 = 0.065;
const CONTACT_SWITCH_SECS: f64 = 0.08;
const MAX_KINEMATIC_SPEED_MPS: f32 = 4.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PreviewSupportFoot {
    Left,
    Right,
}

impl PreviewSupportFoot {
    fn index(self) -> usize {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PreviewMotionStatus {
    pub enabled: bool,
    /// The experimental desktop estimator has direction-calibrated inputs.
    /// This never means that a target has a live position measurement.
    pub active: bool,
    pub frozen: bool,
    pub root_translation_m: [f32; 3],
    /// An inferred support hypothesis, never an observed floor contact.
    pub support: Option<PreviewSupportFoot>,
    pub reason: String,
}

impl Default for PreviewMotionStatus {
    fn default() -> Self {
        Self {
            enabled: false,
            active: false,
            frozen: false,
            root_translation_m: [0.0; 3],
            support: None,
            reason: "Experimental IMU-only desktop motion is disabled".into(),
        }
    }
}

#[derive(Clone, Copy)]
struct Observation {
    ankles: [Vec3; 2],
    rotations: [Quat; 6],
}

#[derive(Default)]
pub struct PreviewMotionEstimator {
    translation: Vec3,
    last_at_secs: Option<f64>,
    previous: Option<Observation>,
    support: Option<PreviewSupportFoot>,
    support_anchor: Option<Vec3>,
    pending_support: Option<(PreviewSupportFoot, f64)>,
}

impl PreviewMotionEstimator {
    /// Calibration and mounting changes establish a new local preview origin.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Translate every joint and target equally, preserving segment rotations,
    /// lengths, evidence, confidence and validity. Live-anchor frames are untouched.
    pub fn apply(&mut self, frame: &mut TrackingFrame, enabled: bool, body: &Anthropometrics) {
        let unavailable = if !enabled {
            Some("Experimental IMU-only desktop motion is disabled")
        } else if frame.live_anchor || !frame.preview_only || !frame.calibration.preview_only {
            Some("Live anchor owns position; desktop motion is inactive")
        } else if !frame.calibration.direction_calibrated {
            Some("Experimental desktop motion awaits neutral and forward calibration")
        } else {
            None
        };
        if let Some(reason) = unavailable {
            self.reset();
            frame.preview_motion = PreviewMotionStatus {
                enabled,
                reason: reason.into(),
                ..PreviewMotionStatus::default()
            };
            return;
        }

        if !frame.at_secs.is_finite() || frame.at_secs < 0.0 {
            self.release_contact();
            self.publish(
                frame,
                true,
                "Experimental desktop motion held: invalid time",
            );
            return;
        }
        let dt = if let Some(last) = self.last_at_secs {
            if frame.at_secs <= last {
                // Retain the high-water timestamp. Repeated or reordered solve
                // times cannot manufacture elapsed motion or contact dwell time.
                self.release_contact();
                self.publish(
                    frame,
                    true,
                    "Experimental desktop motion held: time did not advance",
                );
                return;
            }
            let elapsed = frame.at_secs - last;
            self.last_at_secs = Some(frame.at_secs);
            if elapsed > MAX_GAP_SECS {
                self.release_contact();
                self.publish(
                    frame,
                    true,
                    "Experimental desktop motion held: interrupted input window",
                );
                return;
            }
            elapsed.min(MAX_STEP_SECS) as f32
        } else {
            self.last_at_secs = Some(frame.at_secs);
            0.0
        };
        let Some(observation) = observation(frame) else {
            self.release_contact();
            self.publish(
                frame,
                true,
                "Experimental desktop motion held: fresh measured core orientations required",
            );
            return;
        };
        let Some(neutral_ankle_y) = neutral_ankle_height(body) else {
            self.release_contact();
            self.publish(
                frame,
                true,
                "Experimental desktop motion held: invalid body dimensions",
            );
            return;
        };

        if let Some(previous) = self.previous {
            let discontinuity_limit = MAX_KINEMATIC_SPEED_MPS * dt + 0.03;
            let displacements = [
                (observation.ankles[0] - previous.ankles[0]).length(),
                (observation.ankles[1] - previous.ankles[1]).length(),
            ];
            if displacements
                .iter()
                .all(|distance| *distance > discontinuity_limit)
                || displacements.iter().any(|distance| *distance > 0.50)
            {
                // A large pose discontinuity is not evidence that the floor or
                // root moved. Rebase contact at the next usable frame.
                self.release_contact();
                self.publish(
                    frame,
                    true,
                    "Experimental desktop motion held: discontinuous leg geometry",
                );
                return;
            }
        }

        let lowest = if observation.ankles[0].y <= observation.ankles[1].y {
            PreviewSupportFoot::Left
        } else {
            PreviewSupportFoot::Right
        };
        let mut horizontal_held = false;
        let uniform_turn = self
            .previous
            .is_some_and(|old| uniform_yaw_turn(old, observation));
        if self.support.is_none() {
            self.support = Some(lowest);
            self.support_anchor = Some(observation.ankles[lowest.index()] + self.translation);
        } else if uniform_turn {
            // Uniform yaw leaves absolute translation unobserved. Rebase the
            // conditional foot anchor rather than inventing a turn pivot.
            self.support_anchor = self
                .support
                .map(|support| observation.ankles[support.index()] + self.translation);
            self.pending_support = None;
            horizontal_held = true;
        } else if let Some(support) = self.support {
            let height_above_lowest =
                observation.ankles[support.index()].y - observation.ankles[lowest.index()].y;
            if support != lowest && height_above_lowest > CONTACT_RELEASE_HEIGHT_M {
                horizontal_held = true;
                match self.pending_support {
                    Some((candidate, since))
                        if candidate == lowest && frame.at_secs - since >= CONTACT_SWITCH_SECS =>
                    {
                        // Anchor the new stance at its current translated place;
                        // changing support never teleports the root.
                        self.support = Some(lowest);
                        self.support_anchor =
                            Some(observation.ankles[lowest.index()] + self.translation);
                        self.pending_support = None;
                    }
                    Some((candidate, _)) if candidate == lowest => {}
                    _ => self.pending_support = Some((lowest, frame.at_secs)),
                }
            } else {
                self.pending_support = None;
            }
        }

        if !horizontal_held
            && let (Some(support), Some(anchor)) = (self.support, self.support_anchor)
        {
            let residual =
                horizontal(anchor - observation.ankles[support.index()] - self.translation);
            self.translation += residual.clamp_length_max(MAX_HORIZONTAL_SPEED_MPS * dt);
        }
        // The neutral reference comes from calibrated anatomy, not the first
        // direction-ready frame, which can still be the bent-knee forward pose.
        let desired_y = neutral_ankle_y - observation.ankles[lowest.index()].y;
        self.translation.y += (desired_y - self.translation.y)
            .clamp(-MAX_VERTICAL_SPEED_MPS * dt, MAX_VERTICAL_SPEED_MPS * dt);
        self.previous = Some(observation);
        let reason = if uniform_turn {
            "Experimental IMU-only motion: uniform turn leaves translation unobserved"
        } else if horizontal_held {
            "Experimental IMU-only motion: changing inferred foot support; horizontal position held"
        } else {
            "Experimental IMU-only motion: inferred floor and planted-foot contact; absolute position unobserved"
        };
        self.publish(frame, false, reason);
    }

    fn release_contact(&mut self) {
        self.previous = None;
        self.support = None;
        self.support_anchor = None;
        self.pending_support = None;
    }

    fn publish(&self, frame: &mut TrackingFrame, frozen: bool, reason: &str) {
        for joint in &mut frame.joints {
            joint.position = (Vec3::from_array(joint.position) + self.translation).to_array();
        }
        for target in &mut frame.targets {
            target.position = (Vec3::from_array(target.position) + self.translation).to_array();
        }
        frame.preview_motion = PreviewMotionStatus {
            enabled: true,
            active: true,
            frozen,
            root_translation_m: self.translation.to_array(),
            support: self.support,
            reason: reason.into(),
        };
    }
}

fn neutral_ankle_height(body: &Anthropometrics) -> Option<f32> {
    if body.validate().is_err() {
        return None;
    }
    Some(
        body.standing_eye_height_m
            - body.head_to_neck_m
            - body.neck_to_chest_m
            - body.chest_to_hip_m
            - body.thigh_length_m
            - body.calf_length_m,
    )
}

fn observation(frame: &TrackingFrame) -> Option<Observation> {
    let mut rotations = [Quat::IDENTITY; 6];
    for (index, role) in Role::CORE.iter().enumerate() {
        let segment = frame
            .segments
            .iter()
            .find(|segment| segment.role == *role)?;
        let time = segment.measurement_received_at_secs?;
        let age = frame.at_secs - time;
        let rotation = Quat::from_array(segment.orientation_xyzw);
        if !segment.valid
            || segment.uses_firmware_hold
            || !time.is_finite()
            || time < 0.0
            || !age.is_finite()
            || !(0.0..=MAX_ORIENTATION_AGE_SECS).contains(&age)
            || !rotation.is_finite()
            || rotation.length_squared() < 1.0e-8
        {
            return None;
        }
        rotations[index] = rotation.normalize();
    }
    let mut ankles = [Vec3::ZERO; 2];
    for (index, role) in [JointRole::LeftAnkle, JointRole::RightAnkle]
        .iter()
        .enumerate()
    {
        let position = frame
            .joints
            .iter()
            .find(|joint| joint.joint == *role)?
            .position;
        let point = Vec3::from_array(position);
        if !point.is_finite() || point.abs().max_element() > 10_000.0 {
            return None;
        }
        ankles[index] = point;
    }
    Some(Observation { ankles, rotations })
}

fn horizontal(vector: Vec3) -> Vec3 {
    Vec3::new(vector.x, 0.0, vector.z)
}

fn uniform_yaw_turn(previous: Observation, current: Observation) -> bool {
    let hip_index = 1; // Role::CORE is chest, hip, left/right thigh, left/right calf.
    let delta = current.rotations[hip_index] * previous.rotations[hip_index].inverse();
    let forward = delta * Vec3::NEG_Z;
    let yaw = (-forward.x).atan2(-forward.z);
    if yaw.abs() <= f32::EPSILON {
        return false;
    }
    let yaw_rotation = Quat::from_rotation_y(yaw);
    // A fixed tolerance mistakes ordinary slow gait plus tiny hip-yaw noise
    // for a uniform turn, suppressing legitimate stance displacement.
    let tolerance = (yaw.abs() * 0.20).max(1.0e-5);
    current
        .rotations
        .iter()
        .zip(previous.rotations)
        .all(|(now, old)| {
            let residual = *now * (yaw_rotation * old).inverse();
            // atan2 retains small-angle precision when dot/acos rounds to zero.
            // q and -q describe the same residual rotation.
            let vector = Vec3::new(residual.x, residual.y, residual.z).length();
            2.0 * vector.atan2(residual.w.abs()) <= tolerance
        })
}
