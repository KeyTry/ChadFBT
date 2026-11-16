//! Headset-pose continuity across SteamVR tracking-space events (pure).
//!
//! The calibration is expressed in SteamVR's standing space. What matters to it
//! is only whether the headset's STANDING-space pose stayed physically
//! continuous: a change of the raw-to-standing transform alone also happens
//! when SteamVR re-originates its raw space (driver restart, input-mode switch,
//! relocalisation) while the standing space did not move. The transform change
//! is therefore used only as corroboration of an observed headset jump.
//!
//! Decision for a new usable headset sample compared with the previous one
//! (`dt` = time between them):
//!
//! - `dt > MAX_EVENT_GAP_SECS` (2 s): [`ContinuityDecision::Resumed`]. Nothing
//!   observable connects the two poses; the caller keeps the calibration (same
//!   headset, same room: assumed the same standing frame) and widens uncertainty.
//! - `dt < BURST_WINDOW_SECS` (0.1 s), or up to 2 s when the application
//!   reported a possible tracking-space event ([`TrackingSpaceHint`]) whose
//!   window covers the previous sample: compared against plausible human motion
//!   for `dt` ([`plausible_motion`]).
//!   - Jumped: rebase by the transform's exact yaw when the observed yaw change
//!     agrees with it within [`agreement_tolerance`]; otherwise by the observed
//!     yaw change; without usable horizontal facing, no rebase.
//!   - Not jumped: nothing, unless the observed yaw change is clearly the
//!     transform's (within tolerance of it, outside tolerance of zero, and
//!     closer to it than to zero), in which case rebase by the transform's yaw.
//! - Otherwise (a gap without a hint): not compared.
//!
//! These are engineering thresholds checked by synthetic regressions only.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Longer gaps between usable headset samples are never compared.
pub const MAX_EVENT_GAP_SECS: f64 = 2.0;
/// Consecutive samples closer than this are always compared (burst motion limits).
pub const BURST_WINDOW_SECS: f64 = 0.1;
/// A hint applies to comparisons whose previous sample is at most this much
/// newer than the hint (the poses may switch one or two frames after the
/// transform does).
pub const HINT_WINDOW_SECS: f64 = 0.5;
const BURST_POSITION_M: f32 = 0.25;
const BURST_SPEED_M_PER_SEC: f32 = 4.0;
const BURST_ANGLE_DEGREES: f32 = 25.0;
const BURST_RATE_RAD_PER_SEC: f32 = 10.0;
/// Beyond the first 0.1 s, sustained head translation and rotation rates.
const SUSTAINED_SPEED_M_PER_SEC: f32 = 1.5;
const SUSTAINED_TURN_DEGREES_PER_SEC: f32 = 60.0;
/// Yaw agreement between the observed jump and the transform change.
const AGREEMENT_DEGREES: f32 = 10.0;
const AGREEMENT_DEGREES_PER_SEC: f32 = 20.0;
/// Head forward vectors with a shorter horizontal projection have no usable yaw.
pub const MIN_HORIZONTAL_FACING: f32 = 0.3;

/// One usable headset pose in standing space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeadsetObservation {
    pub at_secs: f64,
    pub position: [f32; 3],
    pub orientation_xyzw: [f32; 4],
}

/// The application saw something that may have moved the standing space: a
/// new helper session or a material change of the raw-to-standing transform.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackingSpaceHint {
    /// Application clock of the event (same clock as anchor receipt times).
    pub at_secs: f64,
    /// Yaw of `standing_new_from_standing_old` derived from the transforms,
    /// when that change is a pure yaw; `None` when the transform did not change
    /// or changed in some other way.
    pub transform_yaw_radians: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RebaseEvidence {
    /// The observed headset jump agreed with the transform change; its exact yaw was used.
    Transform,
    /// The headset jumped without (agreeing) transform evidence; the observed yaw was used.
    ObservedJump,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "decision")]
pub enum ContinuityDecision {
    /// A gap without an event hint, or unusable input: nothing is concluded.
    NotCompared,
    /// The change is plausible human motion: nothing to do.
    Continuous,
    /// The standing space rotated about +Y by `yaw_radians` (the
    /// `TrackingCore::rebase_standing_space` angle).
    Rebase {
        yaw_radians: f32,
        evidence: RebaseEvidence,
    },
    /// The headset jumped but its gaze is near vertical: no usable yaw.
    JumpWithoutHeading,
    /// Usable headset samples resumed after more than `MAX_EVENT_GAP_SECS`.
    Resumed { gap_secs: f64 },
}

/// Largest plausible human head translation (m) and rotation (rad) over
/// `elapsed_secs`: 0.25 m + 4 m/s and 25° + 10 rad/s for the first 0.1 s (the
/// consecutive-sample limits), then 1.5 m/s and 60°/s sustained. At 0.1 s:
/// 0.65 m / 82°; at 1 s: 2.0 m / 136°; at 2 s: 3.5 m / 196°.
pub fn plausible_motion(elapsed_secs: f64) -> (f32, f32) {
    let elapsed = elapsed_secs.max(0.0);
    let burst = elapsed.min(BURST_WINDOW_SECS) as f32;
    let sustained = (elapsed - BURST_WINDOW_SECS).max(0.0) as f32;
    (
        BURST_POSITION_M + BURST_SPEED_M_PER_SEC * burst + SUSTAINED_SPEED_M_PER_SEC * sustained,
        BURST_ANGLE_DEGREES.to_radians()
            + BURST_RATE_RAD_PER_SEC * burst
            + SUSTAINED_TURN_DEGREES_PER_SEC.to_radians() * sustained,
    )
}

/// Largest yaw difference (rad) between an observed headset jump and a
/// transform change that still counts as agreement: 10° + 20°/s × elapsed
/// (10° for consecutive samples, 50° across a 2 s gap).
pub fn agreement_tolerance(elapsed_secs: f64) -> f32 {
    (AGREEMENT_DEGREES + AGREEMENT_DEGREES_PER_SEC * elapsed_secs.max(0.0) as f32).to_radians()
}

/// Whether `hint` covers a comparison whose previous sample is at `previous_at_secs`.
pub fn hint_applies(hint: &TrackingSpaceHint, previous_at_secs: f64) -> bool {
    hint.at_secs.is_finite() && previous_at_secs <= hint.at_secs + HINT_WINDOW_SECS
}

fn quat(values: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_array(values);
    (q.is_finite() && q.length_squared() > 1.0e-8).then(|| q.normalize())
}

fn wrap(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

/// Yaw about +Y of the head's horizontal facing, if it has one.
fn facing_yaw(q: Quat) -> Option<f32> {
    let forward = q * Vec3::NEG_Z;
    (Vec3::new(forward.x, 0.0, forward.z).length() > MIN_HORIZONTAL_FACING)
        .then(|| (-forward.x).atan2(-forward.z))
}

/// Classifies the change from `previous` to `current` (see the module docs).
pub fn decide(
    previous: &HeadsetObservation,
    current: &HeadsetObservation,
    hint: Option<&TrackingSpaceHint>,
) -> ContinuityDecision {
    let dt = current.at_secs - previous.at_secs;
    if !dt.is_finite() || dt <= 0.0 {
        return ContinuityDecision::NotCompared;
    }
    if dt > MAX_EVENT_GAP_SECS {
        return ContinuityDecision::Resumed { gap_secs: dt };
    }
    let hint = hint.filter(|hint| hint_applies(hint, previous.at_secs));
    if dt >= BURST_WINDOW_SECS && hint.is_none() {
        return ContinuityDecision::NotCompared;
    }
    let (Some(q0), Some(q1)) = (
        quat(previous.orientation_xyzw),
        quat(current.orientation_xyzw),
    ) else {
        return ContinuityDecision::NotCompared;
    };
    let (p0, p1) = (
        Vec3::from_array(previous.position),
        Vec3::from_array(current.position),
    );
    if !p0.is_finite() || !p1.is_finite() {
        return ContinuityDecision::NotCompared;
    }
    let moved = (p1 - p0).length();
    let turned = 2.0 * q0.dot(q1).abs().clamp(0.0, 1.0).acos();
    let (position_allowance, angle_allowance) = plausible_motion(dt);
    let jumped = moved > position_allowance || turned > angle_allowance;
    let observed = facing_yaw(q0)
        .zip(facing_yaw(q1))
        .map(|(before, after)| wrap(after - before));
    let transform = hint
        .and_then(|hint| hint.transform_yaw_radians)
        .filter(|yaw| yaw.is_finite())
        // Keep the transform's exact value unless it needs wrapping.
        .map(|yaw| {
            if yaw.abs() <= std::f32::consts::PI {
                yaw
            } else {
                wrap(yaw)
            }
        });
    let tolerance = agreement_tolerance(dt);
    let agrees = |observed: f32, transform: f32| wrap(observed - transform).abs() <= tolerance;
    if jumped {
        return match (observed, transform) {
            (Some(observed), Some(transform)) if agrees(observed, transform) => {
                ContinuityDecision::Rebase {
                    yaw_radians: transform,
                    evidence: RebaseEvidence::Transform,
                }
            }
            (Some(observed), _) => ContinuityDecision::Rebase {
                yaw_radians: observed,
                evidence: RebaseEvidence::ObservedJump,
            },
            (None, _) => ContinuityDecision::JumpWithoutHeading,
        };
    }
    match (observed, transform) {
        (Some(observed), Some(transform))
            if agrees(observed, transform)
                && observed.abs() > tolerance
                && wrap(observed - transform).abs() < observed.abs() =>
        {
            ContinuityDecision::Rebase {
                yaw_radians: transform,
                evidence: RebaseEvidence::Transform,
            }
        }
        _ => ContinuityDecision::Continuous,
    }
}
