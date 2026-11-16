//! Output mode machine (pure). One persisted mode decides whether the app uses
//! the headset, where valid live targets go, and what the status line says.
//!
//! - Auto: live and publishing to SteamVR while a worn headset is available
//!   and the calibration is valid for live use; desktop preview otherwise.
//!   Moves between the two by itself, both ways.
//! - SteamVR / VRChat OSC: live only, publishing to that sink.
//! - Desktop only: never publishes and ignores the headset.
//!
//! Safety rules, whatever the mode: nothing is published during a calibration
//! countdown or capture, without a valid live (non-preview, direction
//! calibrated) calibration, or from a frame that is not live; SteamVR and OSC
//! are mutually exclusive because there is exactly one sink.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputMode {
    #[default]
    Auto,
    SteamVr,
    Osc,
    DesktopOnly,
}

impl OutputMode {
    pub const ALL: [OutputMode; 4] = [Self::Auto, Self::SteamVr, Self::Osc, Self::DesktopOnly];

    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto",
            Self::SteamVr => "SteamVR",
            Self::Osc => "VRChat OSC",
            Self::DesktopOnly => "Desktop only",
        }
    }

    /// The headset is ignored entirely (core anchors are empty).
    pub fn ignores_headset(self) -> bool {
        self == Self::DesktopOnly
    }
}

/// Where valid live targets are sent this frame.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputSink {
    #[default]
    Off,
    SteamVr,
    Osc,
}

/// Without a usable headset sample for this long, the headset is gone.
pub const HEADSET_LOST_SECS: f64 = 3.0;
/// A headset counts as worn (rather than lying on the desk) once a usable
/// sample is at most this far below the configured eye height.
pub const WORN_BELOW_EYE_M: f32 = 0.45;

/// Headset availability with hysteresis. It becomes present on a usable
/// sample at a standing-like height (so a headset on the desk does not take
/// Auto out of desktop preview), stays present while usable samples keep
/// arriving at any height (sitting, kneeling), and is lost after
/// `HEADSET_LOST_SECS` without one.
#[derive(Debug, Clone, Default)]
pub struct HeadsetPresence {
    present: bool,
    last_usable_at: Option<f64>,
}

impl HeadsetPresence {
    /// `usable_height`: the standing height of a fresh, valid headset pose.
    pub fn observe(&mut self, now: f64, usable_height: Option<f32>, eye_height: f32) {
        if let Some(height) = usable_height.filter(|h| h.is_finite()) {
            if self.present || height >= eye_height - WORN_BELOW_EYE_M {
                self.present = true;
                self.last_usable_at = Some(now);
            }
            return;
        }
        if self
            .last_usable_at
            .is_none_or(|at| now < at || now - at > HEADSET_LOST_SECS)
        {
            self.present = false;
        }
    }

    pub fn present(&self) -> bool {
        self.present
    }
}

/// Whether new work treats the session as desktop (no headset): Desktop
/// only, or Auto without a worn headset. A calibration started now is a
/// desktop calibration exactly when this is true.
pub fn desktop_session(mode: OutputMode, headset_present: bool) -> bool {
    match mode {
        OutputMode::DesktopOnly => true,
        OutputMode::Auto => !headset_present,
        OutputMode::SteamVr | OutputMode::Osc => false,
    }
}

/// What the publisher knows about the current frame and calibration.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PublishState {
    /// A calibration countdown, capture or movement phase is active.
    pub calibrating: bool,
    /// Calibration captured (neutral at least).
    pub calibrated: bool,
    pub direction_calibrated: bool,
    /// The calibration is a desktop (preview) calibration.
    pub preview_calibration: bool,
    /// The frame is live: fresh headset anchor and live calibration.
    pub frame_live: bool,
    /// Shutting down: everything invalid.
    pub suspended: bool,
}

impl PublishState {
    /// Every safety condition for sending targets anywhere.
    pub fn live_ready(&self) -> bool {
        !self.suspended
            && !self.calibrating
            && self.calibrated
            && self.direction_calibrated
            && !self.preview_calibration
            && self.frame_live
    }
}

pub fn output_sink(mode: OutputMode, state: &PublishState) -> OutputSink {
    if !state.live_ready() {
        return OutputSink::Off;
    }
    match mode {
        OutputMode::Auto | OutputMode::SteamVr => OutputSink::SteamVr,
        OutputMode::Osc => OutputSink::Osc,
        OutputMode::DesktopOnly => OutputSink::Off,
    }
}

/// Whether a desktop calibration should now be carried over to the headset
/// (`TrackingCore::align_preview_to_headset`): any mode that uses the headset,
/// a worn headset, a complete desktop calibration, and no calibration running.
pub fn wants_alignment(mode: OutputMode, headset_present: bool, state: &PublishState) -> bool {
    !mode.ignores_headset()
        && headset_present
        && !state.calibrating
        && state.calibrated
        && state.direction_calibrated
        && state.preview_calibration
}

/// One-line status of what the mode is currently doing.
pub fn mode_status(
    mode: OutputMode,
    headset_present: bool,
    state: &PublishState,
    sink: OutputSink,
) -> String {
    let name = mode.label();
    if mode == OutputMode::DesktopOnly {
        return "Desktop only: desktop preview; the headset is ignored and nothing is sent".into();
    }
    if state.calibrating {
        return format!("{name}: calibrating (output paused)");
    }
    let target = if mode == OutputMode::Osc {
        "VRChat OSC"
    } else {
        "SteamVR"
    };
    match sink {
        OutputSink::SteamVr | OutputSink::Osc => return format!("{name}: live in {target}"),
        OutputSink::Off => {}
    }
    if !headset_present {
        return if mode == OutputMode::Auto {
            "Auto: desktop preview (no headset)".into()
        } else {
            format!("{name}: waiting for a headset")
        };
    }
    if wants_alignment(mode, headset_present, state) {
        return format!("{name}: waiting to align with headset");
    }
    if !state.calibrated || !state.direction_calibrated {
        return format!("{name}: headset found; calibrate to start tracking");
    }
    format!("{name}: waiting for a fresh headset pose")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn live() -> PublishState {
        PublishState {
            calibrating: false,
            calibrated: true,
            direction_calibrated: true,
            preview_calibration: false,
            frame_live: true,
            suspended: false,
        }
    }

    #[test]
    fn auto_follows_the_headset_both_ways_without_a_prompt() {
        let mut presence = HeadsetPresence::default();
        let eye = 1.65;
        // Start: no SteamVR. Desktop preview.
        presence.observe(0.0, None, eye);
        assert!(desktop_session(OutputMode::Auto, presence.present()));
        // A headset lying on the desk does not count as worn.
        presence.observe(1.0, Some(0.8), eye);
        assert!(!presence.present());
        // Put on: live.
        presence.observe(2.0, Some(1.62), eye);
        assert!(presence.present());
        assert!(!desktop_session(OutputMode::Auto, presence.present()));
        // Sitting down keeps it present; a short dropout too.
        presence.observe(3.0, Some(1.1), eye);
        presence.observe(5.5, None, eye);
        assert!(presence.present());
        // Gone (SteamVR quit after an idle headset): desktop preview.
        presence.observe(6.1, None, eye);
        assert!(!presence.present());
        assert!(desktop_session(OutputMode::Auto, presence.present()));
        // Back: live again, with no user action.
        presence.observe(60.0, Some(1.64), eye);
        assert!(presence.present());
        assert_eq!(output_sink(OutputMode::Auto, &live()), OutputSink::SteamVr);
        // A clock step backwards never keeps a stale presence.
        presence.observe(10.0, None, eye);
        assert!(!presence.present());
    }

    #[test]
    fn calibration_kind_follows_the_mode_at_start() {
        for (mode, present, desktop) in [
            (OutputMode::Auto, true, false),
            (OutputMode::Auto, false, true),
            (OutputMode::SteamVr, false, false),
            (OutputMode::Osc, true, false),
            (OutputMode::DesktopOnly, true, true),
        ] {
            assert_eq!(desktop_session(mode, present), desktop, "{mode:?}");
        }
    }

    #[test]
    fn output_safety_rules_hold_in_every_mode() {
        for mode in OutputMode::ALL {
            let blocked = [
                PublishState {
                    calibrating: true,
                    ..live()
                },
                PublishState {
                    calibrated: false,
                    ..live()
                },
                PublishState {
                    direction_calibrated: false,
                    ..live()
                },
                PublishState {
                    preview_calibration: true,
                    ..live()
                },
                PublishState {
                    frame_live: false,
                    ..live()
                },
                PublishState {
                    suspended: true,
                    ..live()
                },
            ];
            for state in blocked {
                assert_eq!(
                    output_sink(mode, &state),
                    OutputSink::Off,
                    "{mode:?} {state:?}"
                );
            }
        }
        assert_eq!(output_sink(OutputMode::Auto, &live()), OutputSink::SteamVr);
        assert_eq!(
            output_sink(OutputMode::SteamVr, &live()),
            OutputSink::SteamVr
        );
        // OSC and SteamVR are exclusive: one sink per mode.
        assert_eq!(output_sink(OutputMode::Osc, &live()), OutputSink::Osc);
        assert_eq!(
            output_sink(OutputMode::DesktopOnly, &live()),
            OutputSink::Off
        );
    }

    #[test]
    fn desktop_calibrations_are_aligned_only_when_a_worn_headset_is_used() {
        let preview = PublishState {
            preview_calibration: true,
            frame_live: false,
            ..live()
        };
        assert!(wants_alignment(OutputMode::Auto, true, &preview));
        assert!(wants_alignment(OutputMode::SteamVr, true, &preview));
        assert!(wants_alignment(OutputMode::Osc, true, &preview));
        assert!(!wants_alignment(OutputMode::DesktopOnly, true, &preview));
        assert!(!wants_alignment(OutputMode::Auto, false, &preview));
        assert!(!wants_alignment(
            OutputMode::Auto,
            true,
            &PublishState {
                calibrating: true,
                ..preview
            }
        ));
        assert!(!wants_alignment(
            OutputMode::Auto,
            true,
            &PublishState {
                direction_calibrated: false,
                ..preview
            }
        ));
        // Already live: nothing to align.
        assert!(!wants_alignment(OutputMode::Auto, true, &live()));
    }

    #[test]
    fn status_says_what_auto_is_doing() {
        let preview = PublishState {
            preview_calibration: true,
            frame_live: false,
            ..live()
        };
        let status = |mode, present, state: PublishState| {
            mode_status(mode, present, &state, output_sink(mode, &state))
        };
        assert_eq!(
            status(OutputMode::Auto, true, live()),
            "Auto: live in SteamVR"
        );
        assert_eq!(
            status(
                OutputMode::Auto,
                false,
                PublishState {
                    frame_live: false,
                    ..live()
                }
            ),
            "Auto: desktop preview (no headset)"
        );
        assert_eq!(
            status(OutputMode::Auto, true, preview),
            "Auto: waiting to align with headset"
        );
        assert_eq!(
            status(OutputMode::Osc, true, live()),
            "VRChat OSC: live in VRChat OSC"
        );
        assert!(status(OutputMode::SteamVr, false, preview).contains("waiting for a headset"));
        assert!(status(OutputMode::DesktopOnly, true, live()).starts_with("Desktop only"));
        assert!(
            status(
                OutputMode::Auto,
                true,
                PublishState {
                    calibrating: true,
                    ..live()
                }
            )
            .contains("calibrating")
        );
    }

    #[test]
    fn modes_persist_with_auto_as_the_default() {
        assert_eq!(OutputMode::default(), OutputMode::Auto);
        for mode in OutputMode::ALL {
            let text = serde_json::to_string(&mode).unwrap();
            assert_eq!(serde_json::from_str::<OutputMode>(&text).unwrap(), mode);
        }
        assert_eq!(
            serde_json::to_string(&OutputMode::SteamVr).unwrap(),
            "\"steam_vr\""
        );
    }
}
