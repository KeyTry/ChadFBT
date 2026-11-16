//! Functional mounting from a neutral pose followed by an explicitly asserted
//! sagittal skiing pose. This resolves a yaw/mounting ambiguity that a single
//! neutral orientation cannot resolve. A sideways bend can imitate another yaw
//! reference: the instructed direction and pitch sign remain user assertions.

use crate::{CoreError, Role, checked_quat};
use glam::Quat;

pub const MIN_EXCITATION_DEGREES: f32 = 10.0;
pub const MAX_EXCITATION_DEGREES: f32 = 80.0;
pub const MAX_AXIS_VERTICAL_DEGREES: f32 = 25.0;
pub const MAX_MODEL_RESIDUAL_DEGREES: f32 = 10.0;

#[derive(Clone, Copy, Debug)]
pub struct ForwardAlignment {
    /// World-up yaw applied on the LEFT of the sensor quaternion.
    pub heading_alignment_radians: f32,
    /// Fixed mounting rotation applied on the RIGHT of the sensor quaternion.
    pub mounting_xyzw: [f32; 4],
    pub excitation_degrees: f32,
    pub axis_vertical_degrees: f32,
    pub model_residual_degrees: f32,
}

/// Positive X tips a bone's +Y axis backward (+Z). In the instructed ski pose,
/// thighs tip backward while torso/pelvis/shins tip forward (-Z).
pub fn ski_pitch_sign(role: Role) -> Option<f32> {
    match role {
        Role::LeftThigh | Role::RightThigh => Some(1.0),
        Role::Chest | Role::Hip | Role::LeftCalf | Role::RightCalf => Some(-1.0),
        _ => None,
    }
}

pub fn solve_forward_alignment(
    role: Role,
    neutral_xyzw: [f32; 4],
    ski_xyzw: [f32; 4],
    reference_yaw_radians: f32,
) -> Result<ForwardAlignment, CoreError> {
    let sign = ski_pitch_sign(role).ok_or_else(|| {
        CoreError(format!(
            "{role:?} has no supported skiing-pose mounting solve"
        ))
    })?;
    if !reference_yaw_radians.is_finite() {
        return Err(CoreError("Invalid forward reference heading".into()));
    }
    let neutral = checked_quat(neutral_xyzw)?;
    let ski = checked_quat(ski_xyzw)?;
    let mut delta = (ski * neutral.conjugate()).normalize();
    if delta.w < 0.0 {
        delta = -delta;
    }
    let (axis, angle) = delta.to_axis_angle();
    let excitation_degrees = angle.to_degrees();
    if excitation_degrees < MIN_EXCITATION_DEGREES {
        return Err(CoreError(format!(
            "{role:?} changed only {excitation_degrees:.1}°; the skiing pose needs at least \
             {MIN_EXCITATION_DEGREES:.0}° of this segment's instructed forward/backward tilt"
        )));
    }
    if excitation_degrees > MAX_EXCITATION_DEGREES {
        return Err(CoreError(format!(
            "{role:?} moved {excitation_degrees:.1}° from neutral; use a moderate skiing pose \
             below {MAX_EXCITATION_DEGREES:.0}° without turning or twisting"
        )));
    }
    let signed_axis = axis * sign;
    let axis_vertical_degrees = signed_axis.y.abs().clamp(0.0, 1.0).asin().to_degrees();
    if !signed_axis.is_finite() || axis_vertical_degrees > MAX_AXIS_VERTICAL_DEGREES {
        return Err(CoreError(format!(
            "{role:?} motion includes too much turning/twisting (axis tilt {axis_vertical_degrees:.1}°); \
             keep facing the same direction and use the instructed skiing bend"
        )));
    }
    // A large rotation can be almost entirely yaw. The horizontal component
    // must itself have enough excitation for a meaningful bend axis estimate.
    let horizontal_excitation_degrees = (2.0
        * ((angle * 0.5).sin() * signed_axis.x.hypot(signed_axis.z))
            .clamp(0.0, 1.0)
            .asin())
    .to_degrees();
    if horizontal_excitation_degrees < MIN_EXCITATION_DEGREES {
        return Err(CoreError(format!(
            "{role:?} needs more forward/backward bend (horizontal motion \
             {horizontal_excitation_degrees:.1}°); bend a little deeper while facing forward"
        )));
    }
    // For Ry(h) X = [cos(h), 0, -sin(h)], h = atan2(-axis.z, axis.x).
    let raw_body_heading = (-signed_axis.z).atan2(signed_axis.x);
    let heading_alignment_radians = crate::wrap_angle(reference_yaw_radians - raw_body_heading);
    let alignment = Quat::from_rotation_y(heading_alignment_radians);
    let reference = Quat::from_rotation_y(reference_yaw_radians);
    let mounting = ((alignment * neutral).conjugate() * reference).normalize();
    // Express the aligned relative motion in the captured reference's local axes.
    let aligned_delta =
        reference.conjugate() * alignment * delta * alignment.conjugate() * reference;
    let expected = Quat::from_rotation_x(sign * angle);
    let model_residual_degrees = crate::angle(aligned_delta.normalize(), expected).to_degrees();
    if model_residual_degrees > MAX_MODEL_RESIDUAL_DEGREES {
        return Err(CoreError(format!(
            "{role:?} does not fit a sagittal skiing bend (residual {model_residual_degrees:.1}°); \
             hold the skiing bend while facing forward, with less twisting"
        )));
    }
    let neutral_check = (alignment * neutral * mounting).normalize();
    if !neutral_check.is_finite() || crate::angle(neutral_check, reference) > 0.001 {
        return Err(CoreError(format!(
            "{role:?} forward calibration is numerically unstable"
        )));
    }
    // No forward/position measurement is synthesized here; this is a two-pose
    // factorization conditional on the signed sagittal-motion instruction.
    Ok(ForwardAlignment {
        heading_alignment_radians,
        mounting_xyzw: mounting.to_array(),
        excitation_degrees,
        axis_vertical_degrees,
        model_residual_degrees,
    })
}
