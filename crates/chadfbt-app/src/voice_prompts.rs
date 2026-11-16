//! Spoken guidance wording (pure). One table maps movement-calibration
//! blockers to short instructions so the UI can reuse the same words. Every
//! phrase is meant to be spoken in under about three seconds.
use tracking_core::{
    Role,
    motion_calibration::{MotionCalibrationStatus, MotionGate, MotionSegmentStatus},
};

pub const CALIBRATION_START: &str = "Calibration. Stand straight and still.";
pub const CALIBRATION_MOVE_ONLY_START: &str = "Get ready to move, facing the same way.";
pub const CALIBRATION_MOVE: &str = "Now bow forward, then lift each knee twice.";
pub const CALIBRATION_COMPLETE: &str = "Calibration complete.";
pub const CALIBRATION_CANCELLED: &str = "Calibration cancelled.";
pub const NEUTRAL_START: &str = "Stand straight and still.";
pub const SKI_POSE_START: &str = "Take the skiing pose and hold it.";
pub const HEIGHT_START: &str = "Stand up straight and look ahead.";
pub const CAPTURED: &str = "Captured.";
pub const RECOVERY_REQUESTED: &str = "Resetting heading. Hold still, facing forward.";
pub const RECOVERY_APPLIED: &str = "Heading reset.";
pub const RECOVERY_NEEDS_CALIBRATION: &str =
    "Heading reset needs a live calibration first. Triple tap to calibrate.";
pub const RECOVERY_RETRY: &str = "Stand still and face forward, then double tap again.";
pub const TEST_SOUND: &str = "Voice test. This is how calibration prompts will sound.";
/// A desktop calibration is about to be carried over to the headset.
pub const HEADSET_FOUND_ALIGN: &str = "Headset found. Stand straight and look ahead.";
pub const ALIGNED: &str = "Aligned.";
/// Prefixed to the next calibration prompt after a large automatic eye-height change.
pub const HEIGHT_UPDATED: &str = "Height updated.";

/// Spoken once when headset tracking resumes after a long gap (SteamVR
/// restarted, headset idle) with the calibration kept.
pub fn tracking_resumed(gestures: bool) -> &'static str {
    if gestures {
        "Tracking resumed. If the body faces the wrong way, double tap to reset heading."
    } else {
        "Tracking resumed. If the body faces the wrong way, reset heading in the app."
    }
}

/// Spoken when an assigned tracker rebooted and the calibration was cleared.
pub fn tracker_restarted(gestures: bool) -> &'static str {
    if gestures {
        "A tracker restarted. Triple tap to calibrate."
    } else {
        "A tracker restarted. Calibrate again in the app."
    }
}

/// Body part as far as wording is concerned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Part {
    Torso,
    Thigh,
    Calf,
}

/// One row: what to say while moving (`{side}` becomes left/right), and what
/// is missing, for the timeout sentence "<segment> needs <need>".
pub struct BlockerPhrase {
    pub gate: MotionGate,
    pub torso: &'static str,
    pub thigh: &'static str,
    pub calf: &'static str,
    pub need: &'static str,
}

/// Blocker gate → wording. Global gates (body turn, headset reference) use the
/// `torso` column for every segment.
pub const BLOCKER_PHRASES: &[BlockerPhrase] = &[
    BlockerPhrase {
        gate: MotionGate::BodyTurn,
        torso: "Keep facing the same way.",
        thigh: "Keep facing the same way.",
        calf: "Keep facing the same way.",
        need: "you to keep facing the same way",
    },
    BlockerPhrase {
        gate: MotionGate::Reference,
        torso: "Keep the headset on and face forward.",
        thigh: "Keep the headset on and face forward.",
        calf: "Keep the headset on and face forward.",
        need: "a usable headset direction",
    },
    BlockerPhrase {
        gate: MotionGate::NoData,
        torso: "The {part} tracker is not sending data.",
        thigh: "The {side} thigh tracker is not sending data.",
        calf: "The {side} calf tracker is not sending data.",
        need: "a working tracker",
    },
    BlockerPhrase {
        gate: MotionGate::Stale,
        torso: "The {part} tracker stopped sending data.",
        thigh: "The {side} thigh tracker stopped sending data.",
        calf: "The {side} calf tracker stopped sending data.",
        need: "a working tracker",
    },
    BlockerPhrase {
        gate: MotionGate::Excitation,
        torso: "Bow a little deeper.",
        thigh: "Lift your {side} knee higher.",
        calf: "Bend your {side} knee more.",
        need: "more movement",
    },
    BlockerPhrase {
        gate: MotionGate::Planarity,
        torso: "Bow straight forward, not sideways.",
        thigh: "Lift your {side} knee straight forward.",
        calf: "Bend your {side} knee straight forward.",
        need: "a straighter forward movement",
    },
    BlockerPhrase {
        gate: MotionGate::OneSidedness,
        torso: "Bow forward only, don't lean back.",
        thigh: "Lift your {side} knee forward, don't swing it back.",
        calf: "Lift your {side} knee, don't swing the leg.",
        need: "movement in one direction only",
    },
    BlockerPhrase {
        gate: MotionGate::ModelResidual,
        torso: "Keep the bow straight, not sideways.",
        thigh: "Keep your {side} leg moving straight forward.",
        calf: "Keep your {side} leg moving straight forward.",
        need: "less sideways movement",
    },
    BlockerPhrase {
        gate: MotionGate::Repeatability,
        torso: "Repeat the same bow steadily.",
        thigh: "Repeat the same knee lift steadily.",
        calf: "Repeat the same knee lift steadily.",
        need: "steadier repetitions",
    },
    BlockerPhrase {
        gate: MotionGate::KneeHyperextension,
        torso: "Keep going.",
        thigh: "Don't push your {side} knee backward.",
        calf: "Don't push your {side} knee backward.",
        need: "less backward knee bend",
    },
    BlockerPhrase {
        gate: MotionGate::KneeAxis,
        torso: "Keep going.",
        thigh: "Point your {side} knee forward.",
        calf: "Point your {side} knee forward.",
        need: "the knee pointing forward",
    },
    BlockerPhrase {
        gate: MotionGate::KneeUnchecked,
        torso: "Keep going.",
        thigh: "Lift your {side} knee.",
        calf: "Lift your {side} knee.",
        need: "a knee lift",
    },
    BlockerPhrase {
        gate: MotionGate::Stability,
        torso: "Keep going.",
        thigh: "Keep going.",
        calf: "Keep going.",
        need: "a little more movement",
    },
];

fn row(gate: MotionGate) -> Option<&'static BlockerPhrase> {
    BLOCKER_PHRASES.iter().find(|row| row.gate == gate)
}

fn part_and_side(role: Role) -> (Part, &'static str, &'static str) {
    match role {
        Role::Chest => (Part::Torso, "", "chest"),
        Role::Hip => (Part::Torso, "", "hip"),
        Role::LeftThigh => (Part::Thigh, "left", "left thigh"),
        Role::RightThigh => (Part::Thigh, "right", "right thigh"),
        Role::LeftCalf => (Part::Calf, "left", "left calf"),
        Role::RightCalf => (Part::Calf, "right", "right calf"),
        Role::LeftFoot => (Part::Calf, "left", "left foot"),
        Role::RightFoot => (Part::Calf, "right", "right foot"),
        Role::LeftUpperArm => (Part::Torso, "left", "left upper arm"),
        Role::RightUpperArm => (Part::Torso, "right", "right upper arm"),
    }
}

/// The instruction for one segment's blocker, e.g. "Lift your left knee higher."
pub fn blocker_instruction(gate: MotionGate, role: Role) -> Option<String> {
    let row = row(gate)?;
    let (part, side, label) = part_and_side(role);
    let template = match part {
        Part::Torso => row.torso,
        Part::Thigh => row.thigh,
        Part::Calf => row.calf,
    };
    Some(template.replace("{side}", side).replace("{part}", label))
}

/// Gates in the order they are worth mentioning: whole-calibration problems,
/// then dead trackers, then what the user should do more of.
const PRIORITY: [MotionGate; 13] = [
    MotionGate::BodyTurn,
    MotionGate::Reference,
    MotionGate::NoData,
    MotionGate::Stale,
    MotionGate::Excitation,
    MotionGate::KneeUnchecked,
    MotionGate::Planarity,
    MotionGate::OneSidedness,
    MotionGate::ModelResidual,
    MotionGate::KneeAxis,
    MotionGate::KneeHyperextension,
    MotionGate::Repeatability,
    MotionGate::Stability,
];

/// The single most useful (gate, role) to mention: calibration-wide blockers
/// first, then each blocking segment's first blocker by gate priority and body
/// order (torso before thighs before calves, matching the instructed order).
fn main_blocker(status: &MotionCalibrationStatus) -> Option<(MotionGate, Option<Role>)> {
    let global = status
        .blockers
        .iter()
        .filter(|b| row(b.gate).is_some())
        .min_by_key(|b| PRIORITY.iter().position(|g| *g == b.gate))
        .map(|b| (b.gate, None));
    if global.is_some() {
        return global;
    }
    let segments: Vec<&MotionSegmentStatus> = status.segments.iter().filter(|s| !s.ready).collect();
    PRIORITY.iter().find_map(|gate| {
        segments
            .iter()
            .find(|s| s.blockers.first().is_some_and(|b| b.gate == *gate))
            .map(|s| (*gate, Some(s.role)))
    })
}

/// What to say while the movement phase is still incomplete, or `None` when
/// nothing useful can be said (ready, or no recognised blocker).
pub fn motion_guidance(status: &MotionCalibrationStatus) -> Option<String> {
    if status.ready {
        return None;
    }
    match main_blocker(status)? {
        (gate, Some(role)) => blocker_instruction(gate, role),
        (gate, None) => blocker_instruction(gate, Role::Chest),
    }
}

/// "hip needs more forward bend", "left thigh needs a higher knee lift", ...
fn missing_clause(status: &MotionCalibrationStatus) -> Option<String> {
    let (gate, role) = main_blocker(status)?;
    let row = row(gate)?;
    let Some(role) = role else {
        return Some(format!("it needs {}", row.need));
    };
    let (part, _, label) = part_and_side(role);
    let need = match (gate, part) {
        (MotionGate::Excitation, Part::Torso) => "more forward bend",
        (MotionGate::Excitation, Part::Thigh) => "a higher knee lift",
        (MotionGate::Excitation, Part::Calf) => "more knee bend",
        _ => row.need,
    };
    Some(format!("{label} needs {need}"))
}

fn retry_hint(gestures: bool) -> &'static str {
    if gestures {
        " Triple tap to try again."
    } else {
        ""
    }
}

/// Spoken when the movement phase times out.
pub fn motion_timeout(status: Option<&MotionCalibrationStatus>, gestures: bool) -> String {
    match status.and_then(missing_clause) {
        Some(clause) => format!(
            "Calibration did not finish: {clause}.{}",
            retry_hint(gestures)
        ),
        None => format!("Calibration did not finish.{}", retry_hint(gestures)),
    }
}

/// Core neutral-gate / recovery rejection text → short spoken phrase. The
/// first matching key wins (more specific keys first).
const NEUTRAL_REASONS: &[(&str, &str)] = &[
    (
        "Live neutral and forward-direction calibration are required",
        RECOVERY_NEEDS_CALIBRATION,
    ),
    ("Assign the", "Assign all six trackers first."),
    ("fresh, valid live HMD", "The headset is not tracked."),
    (
        "Headset height",
        "Headset height is off. Stand up straight.",
    ),
    (
        "Look forward with the headset upright",
        "Look straight ahead.",
    ),
    ("Headset is moving", "Hold your head still."),
    ("HMD", "The headset is not tracked steadily."),
    ("sensor health error", "A tracker reports an error."),
    ("tilt is incompatible", "Stand up straight."),
    ("heading is ambiguous", "Face forward with your whole body."),
    ("not valid for recovery", "A tracker is not ready."),
    ("is moving", "Hold still."),
    ("stale", "A tracker stopped sending data."),
    ("gaps", "A tracker signal is dropping out."),
    ("firmware-held", "Hold still."),
    ("history", "Hold still."),
    ("settle", "Hold still."),
    ("paused", "Hold still."),
];

pub fn neutral_reason(reason: &str) -> &'static str {
    NEUTRAL_REASONS
        .iter()
        .find(|(key, _)| reason.contains(key))
        .map_or("Stand straight and hold still.", |(_, phrase)| phrase)
}

/// Spoken when a neutral (or other single) capture times out.
pub fn capture_timeout(message: &str, gestures: bool) -> String {
    let reason = neutral_reason(message);
    if reason == RECOVERY_NEEDS_CALIBRATION {
        return reason.into();
    }
    format!(
        "Calibration did not start. {reason}{}",
        retry_hint(gestures)
    )
}

/// Spoken for a movement request that stopped for a reason other than its
/// timeout (session could not start or was interrupted).
pub fn motion_aborted(gestures: bool) -> String {
    format!(
        "Calibration stopped because tracking changed.{}",
        retry_hint(gestures)
    )
}

/// Spoken when the core declines a heading recovery (immediately or after its
/// timeout).
pub fn recovery_declined(reason: &str) -> String {
    let phrase = neutral_reason(reason);
    if phrase == RECOVERY_NEEDS_CALIBRATION {
        return phrase.into();
    }
    format!("Heading not reset. {phrase} {RECOVERY_RETRY}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracking_core::motion_calibration::{MotionBlocker, MotionCalibrator};

    fn base() -> MotionCalibrationStatus {
        MotionCalibrator::new(Default::default()).unwrap().status()
    }

    fn segment(role: Role, blockers: &[MotionGate]) -> MotionSegmentStatus {
        let mut segment = base().segments[0].clone();
        segment.role = role;
        segment.ready = blockers.is_empty();
        segment.blockers = blockers
            .iter()
            .map(|gate| MotionBlocker {
                gate: *gate,
                message: format!("{role:?}: {gate:?}"),
            })
            .collect();
        segment
    }

    fn status(
        segments: Vec<MotionSegmentStatus>,
        global: &[MotionGate],
    ) -> MotionCalibrationStatus {
        let mut status = base();
        status.ready = segments.iter().all(|s| s.ready) && global.is_empty();
        status.segments = segments;
        status.blockers = global
            .iter()
            .map(|gate| MotionBlocker {
                gate: *gate,
                message: format!("{gate:?}"),
            })
            .collect();
        status
    }

    #[test]
    fn every_gate_except_not_started_has_a_phrase_for_every_role() {
        for gate in PRIORITY {
            for role in Role::CORE {
                let text = blocker_instruction(gate, role).unwrap();
                assert!(!text.contains('{'), "{text}");
                assert!(text.ends_with('.'), "{text}");
                // About three seconds of speech at most.
                assert!(text.split_whitespace().count() <= 10, "{text}");
            }
        }
        assert!(blocker_instruction(MotionGate::NotStarted, Role::Hip).is_none());
    }

    #[test]
    fn guidance_names_the_most_useful_missing_movement() {
        let s = status(
            vec![
                segment(Role::Chest, &[]),
                segment(Role::Hip, &[MotionGate::Excitation]),
                segment(Role::LeftThigh, &[MotionGate::Excitation]),
                segment(Role::RightThigh, &[]),
                segment(Role::LeftCalf, &[MotionGate::Stability]),
                segment(Role::RightCalf, &[]),
            ],
            &[],
        );
        assert_eq!(motion_guidance(&s).as_deref(), Some("Bow a little deeper."));
        assert_eq!(
            motion_timeout(Some(&s), true),
            "Calibration did not finish: hip needs more forward bend. Triple tap to try again."
        );

        let s = status(
            vec![
                segment(Role::Chest, &[]),
                segment(Role::Hip, &[]),
                segment(Role::LeftThigh, &[MotionGate::Excitation]),
                segment(Role::RightCalf, &[MotionGate::Stability]),
            ],
            &[],
        );
        assert_eq!(
            motion_guidance(&s).as_deref(),
            Some("Lift your left knee higher.")
        );
        assert_eq!(
            motion_timeout(Some(&s), false),
            "Calibration did not finish: left thigh needs a higher knee lift."
        );

        // A body turn outranks every segment blocker.
        let s = status(
            vec![segment(Role::Hip, &[MotionGate::Excitation])],
            &[MotionGate::BodyTurn],
        );
        assert_eq!(
            motion_guidance(&s).as_deref(),
            Some("Keep facing the same way.")
        );
        // A stale tracker outranks movement advice.
        let s = status(
            vec![
                segment(Role::Chest, &[MotionGate::Excitation]),
                segment(Role::RightCalf, &[MotionGate::Stale]),
            ],
            &[],
        );
        assert_eq!(
            motion_guidance(&s).as_deref(),
            Some("The right calf tracker stopped sending data.")
        );
        // Ready: nothing to say; unknown: generic timeout.
        let s = status(vec![segment(Role::Chest, &[])], &[]);
        assert_eq!(motion_guidance(&s), None);
        assert_eq!(
            motion_timeout(None, true),
            "Calibration did not finish. Triple tap to try again."
        );
    }

    #[test]
    fn neutral_and_recovery_reasons_become_short_phrases() {
        assert_eq!(
            neutral_reason(
                "Headset height 0.75 m is outside 1.42–1.82 m (configured eye height 1.62 m)"
            ),
            "Headset height is off. Stand up straight."
        );
        assert_eq!(
            neutral_reason("Headset is moving; hold the neutral pose"),
            "Hold your head still."
        );
        assert_eq!(
            neutral_reason("Hip is moving; wait for the tap and body motion to settle"),
            "Hold still."
        );
        assert_eq!(
            neutral_reason("HMD sample gaps interrupted stable history"),
            "The headset is not tracked steadily."
        );
        assert_eq!(
            neutral_reason(
                "LeftThigh heading is ambiguous; face forward and use explicit calibration"
            ),
            "Face forward with your whole body."
        );
        assert_eq!(
            capture_timeout(
                "Calibration timed out. Last blocked reason: Look forward with the headset upright",
                true
            ),
            "Calibration did not start. Look straight ahead. Triple tap to try again."
        );
        assert_eq!(
            recovery_declined(
                "Live neutral and forward-direction calibration are required before recovery"
            ),
            RECOVERY_NEEDS_CALIBRATION
        );
        assert_eq!(
            recovery_declined("Chest is moving; wait for the tap and body motion to settle"),
            "Heading not reset. Hold still. Stand still and face forward, then double tap again."
        );
        assert_eq!(
            neutral_reason("something new"),
            "Stand straight and hold still."
        );
    }
}
