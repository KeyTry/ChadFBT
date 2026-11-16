//! Pure, single-owner timing for an explicitly requested calibration.
//!
//! The first attempt is authorized only after three seconds. Failed requirements
//! may be retried at most 10 Hz during a bounded settling window: three seconds
//! for neutral/height capture, ten seconds for the skiing pose. This object does
//! not calibrate, play audio, or weaken any tracking-core requirement.
//! Call `attempt_result` synchronously after each true `poll`, before processing
//! another command; asynchronous results require a separate request-ID contract.
//!
//! Movement calibration adds a `Moving` phase: after the neutral capture (or,
//! when a valid neutral exists, directly after the countdown) the user moves
//! for up to 30 s. A finish attempt is authorized only once the runtime has
//! reported "ready" continuously for 0.5 s and at least 3 s of the movement
//! phase have passed, so one hasty squat cannot end it.

use serde::{Deserialize, Serialize};

const COUNTDOWN_SECS: f64 = 3.0;
const CAPTURE_WINDOW_SECS: f64 = 3.0;
const FORWARD_CAPTURE_WINDOW_SECS: f64 = 10.0;
const RETRY_INTERVAL_SECS: f64 = 0.1;
/// Upper bound of the movement phase.
pub const MOVE_WINDOW_SECS: f64 = 30.0;
/// Minimum movement time before a finish can be attempted.
pub const MOVE_MIN_SECS: f64 = 3.0;
/// The calibration must report ready continuously for this long.
pub const MOVE_READY_HOLD_SECS: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationRequestPhase {
    #[default]
    Idle,
    Countdown,
    WaitingStable,
    /// Movement calibration: the user moves while evidence accumulates.
    Moving,
    Captured,
    TimedOut,
    Cancelled,
}

impl CalibrationRequestPhase {
    fn active(self) -> bool {
        matches!(self, Self::Countdown | Self::WaitingStable | Self::Moving)
    }
}

/// What the request does once its countdown ends.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Flow {
    /// One synchronous capture (neutral, skiing pose, headset height).
    #[default]
    Capture,
    /// Neutral capture, then the movement phase.
    Motion,
    /// Movement phase directly after the countdown (neutral already valid).
    MotionMoveOnly,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CalibrationRequestView {
    pub request_id: u64,
    pub phase: CalibrationRequestPhase,
    /// Countdown: time until first attempt. WaitingStable: time until deadline.
    /// Terminal/idle states always expose zero. Never NaN or infinity.
    pub seconds_remaining: f64,
    pub message: String,
}

#[derive(Debug, Default)]
pub struct CalibrationRequestController {
    request_id: u64,
    phase: CalibrationRequestPhase,
    message: String,
    ready_at: Option<f64>,
    deadline_at: Option<f64>,
    next_attempt_at: Option<f64>,
    attempt_pending: bool,
    last_blocking_reason: Option<String>,
    last_observed_at: Option<f64>,
    flow: Flow,
    move_started_at: Option<f64>,
    ready_since: Option<f64>,
}

impl CalibrationRequestController {
    /// Starts a new user request, replacing any earlier active request. Request
    /// IDs change even when an invalid clock immediately cancels the new request.
    pub fn start(&mut self, now: f64) {
        self.flow = Flow::Capture;
        self.start_with_capture_window(now, CAPTURE_WINDOW_SECS);
    }

    /// Gives the user time to settle into the skiing pose after the same
    /// three-second countdown. Failed attempts never extend the fixed deadline.
    pub fn start_forward(&mut self, now: f64) {
        self.flow = Flow::Capture;
        self.start_with_capture_window(now, FORWARD_CAPTURE_WINDOW_SECS);
    }

    /// Movement calibration: the neutral capture exactly as [`Self::start`],
    /// whose success enters `Moving` instead of `Captured`.
    pub fn start_motion(&mut self, now: f64) {
        self.flow = Flow::Motion;
        self.start_with_capture_window(now, CAPTURE_WINDOW_SECS);
    }

    /// Movement calibration reusing a valid neutral capture: `Moving` begins
    /// when the three-second countdown ends.
    pub fn start_motion_move(&mut self, now: f64) {
        self.flow = Flow::MotionMoveOnly;
        self.start_with_capture_window(now, MOVE_WINDOW_SECS);
    }

    /// Seconds since the movement phase began (zero outside it).
    pub fn moving_secs(&self, now: f64) -> f64 {
        match (self.phase, self.move_started_at) {
            (CalibrationRequestPhase::Moving, Some(start)) if now.is_finite() => {
                (now - start).max(0.0)
            }
            _ => 0.0,
        }
    }

    /// Feeds the current movement-calibration readiness (sampled by the
    /// runtime) and returns permission for one synchronous finish attempt.
    /// `blocking` is the latest human-readable list of blockers, kept for the
    /// timeout message.
    pub fn observe_motion(&mut self, now: f64, ready: bool, blocking: Option<String>) -> bool {
        if !self.advance(now)
            || self.phase != CalibrationRequestPhase::Moving
            || self.attempt_pending
        {
            return false;
        }
        if !ready {
            self.ready_since = None;
            if let Some(blocking) = blocking.filter(|b| !b.trim().is_empty()) {
                self.last_blocking_reason = Some(blocking);
            }
            return false;
        }
        let since = *self.ready_since.get_or_insert(now);
        let moved = self.move_started_at.map_or(0.0, |start| now - start);
        if now - since >= MOVE_READY_HOLD_SECS && moved >= MOVE_MIN_SECS {
            self.attempt_pending = true;
            return true;
        }
        false
    }

    /// Completes the finish attempt authorized by `observe_motion`. A failure
    /// keeps the movement phase (same deadline) and restarts the ready hold.
    pub fn motion_finish_result(&mut self, now: f64, result: Result<(), String>) {
        if !self.advance(now)
            || self.phase != CalibrationRequestPhase::Moving
            || !self.attempt_pending
        {
            return;
        }
        self.attempt_pending = false;
        match result {
            Ok(()) => self.finish(
                CalibrationRequestPhase::Captured,
                "Movement calibration captured.",
            ),
            Err(reason) => {
                self.ready_since = None;
                if !reason.trim().is_empty() {
                    self.last_blocking_reason = Some(reason);
                }
            }
        }
    }

    /// Ends an active movement request with a failure that is not a timeout
    /// (the session could not start or was interrupted). Terminal results are
    /// kept, as with `cancel`.
    pub fn abort_motion(&mut self, now: f64, reason: impl Into<String>) {
        if !self.accept_clock(now) || !self.phase.active() {
            return;
        }
        self.finish(CalibrationRequestPhase::TimedOut, reason);
    }

    fn enter_moving(&mut self, start: f64) {
        let deadline = start + MOVE_WINDOW_SECS;
        if !deadline.is_finite() || deadline <= start {
            self.finish(
                CalibrationRequestPhase::Cancelled,
                "Calibration cancelled: clock cannot represent the movement window safely.",
            );
            return;
        }
        self.phase = CalibrationRequestPhase::Moving;
        self.message = "Move now: bow forward once, then lift each knee twice (or do two slow squats). Keep facing the same direction.".into();
        self.move_started_at = Some(start);
        self.ready_since = None;
        self.deadline_at = Some(deadline);
        self.next_attempt_at = None;
        self.attempt_pending = false;
        self.last_blocking_reason = None;
    }

    fn start_with_capture_window(&mut self, now: f64, capture_window_secs: f64) {
        let Some(next_id) = self.request_id.checked_add(1) else {
            self.finish(
                CalibrationRequestPhase::Cancelled,
                "Calibration cancelled: request counter exhausted.",
            );
            return;
        };
        self.request_id = next_id;
        self.phase = CalibrationRequestPhase::Countdown;
        self.message = "Calibration starts in 3 seconds. Stand upright, face forward with your torso, and hold still.".into();
        self.ready_at = None;
        self.deadline_at = None;
        self.next_attempt_at = None;
        self.attempt_pending = false;
        self.last_blocking_reason = None;
        self.move_started_at = None;
        self.ready_since = None;
        if !self.accept_clock(now) {
            return;
        }
        let ready_at = now + COUNTDOWN_SECS;
        let deadline_at = ready_at + capture_window_secs;
        if !ready_at.is_finite()
            || !deadline_at.is_finite()
            || ready_at <= now
            || deadline_at <= ready_at
            || ready_at + RETRY_INTERVAL_SECS <= ready_at
        {
            self.finish(
                CalibrationRequestPhase::Cancelled,
                "Calibration cancelled: clock cannot represent the countdown safely.",
            );
            return;
        }
        self.ready_at = Some(ready_at);
        self.deadline_at = Some(deadline_at);
        self.next_attempt_at = Some(ready_at);
    }

    /// Returns permission for one synchronous attempt. No attempt is authorized
    /// before start+3 seconds or at/after the request's fixed deadline, including
    /// after pauses.
    pub fn poll(&mut self, now: f64) -> bool {
        if !self.advance(now)
            || self.phase != CalibrationRequestPhase::WaitingStable
            || self.attempt_pending
            || self.next_attempt_at.is_none_or(|next| now < next)
        {
            return false;
        }
        self.attempt_pending = true;
        self.next_attempt_at = Some(now + RETRY_INTERVAL_SECS);
        true
    }

    /// Completes the immediately preceding authorized attempt. Unsolicited,
    /// cancelled, expired and already-completed results cannot capture a request.
    pub fn attempt_result(&mut self, now: f64, result: Result<(), String>) {
        if !self.advance(now)
            || self.phase != CalibrationRequestPhase::WaitingStable
            || !self.attempt_pending
        {
            return;
        }
        self.attempt_pending = false;
        match result {
            Ok(()) if self.flow == Flow::Motion => self.enter_moving(now),
            Ok(()) => self.finish(
                CalibrationRequestPhase::Captured,
                "Neutral calibration captured.",
            ),
            Err(reason) => {
                let reason = if reason.trim().is_empty() {
                    "Calibration requirements are not satisfied yet.".into()
                } else {
                    reason
                };
                self.message = format!("Waiting for a stable neutral pose: {reason}");
                self.last_blocking_reason = Some(reason);
            }
        }
    }

    /// Cancels only active work. Terminal outcomes remain available until another
    /// deliberate start, so a background session event cannot erase the result.
    pub fn cancel(&mut self, now: f64, reason: impl Into<String>) {
        if !self.accept_clock(now) || !self.phase.active() {
            return;
        }
        let reason = reason.into();
        self.finish(
            CalibrationRequestPhase::Cancelled,
            if reason.trim().is_empty() {
                "Calibration cancelled.".into()
            } else {
                format!("Calibration cancelled: {reason}")
            },
        );
    }

    /// Use on changed assignments, dimensions, preview mode, tracking space,
    /// tracker/anchor sessions, profile load, or other invalidated prerequisites.
    /// Active work is cancelled; a result from an obsolete setup is cleared.
    /// The request ID stays monotonic so audio/UI can discard old notifications.
    pub fn invalidate(&mut self, now: f64, reason: impl Into<String>) {
        if !self.accept_clock(now) {
            return;
        }
        if self.phase.active() {
            self.cancel(now, reason);
        } else {
            self.finish(CalibrationRequestPhase::Idle, "");
            self.last_blocking_reason = None;
        }
    }

    /// Advances timing/timeout diagnostics but never authorizes an attempt.
    pub fn view(&mut self, now: f64) -> CalibrationRequestView {
        self.advance(now);
        let until = match self.phase {
            CalibrationRequestPhase::Countdown => self.ready_at,
            CalibrationRequestPhase::WaitingStable | CalibrationRequestPhase::Moving => {
                self.deadline_at
            }
            _ => None,
        };
        CalibrationRequestView {
            request_id: self.request_id,
            phase: self.phase,
            seconds_remaining: until
                .filter(|_| now.is_finite())
                .map_or(0.0, |until| (until - now).max(0.0)),
            message: self.message.clone(),
        }
    }

    fn advance(&mut self, now: f64) -> bool {
        if !self.accept_clock(now) {
            return false;
        }
        if !self.phase.active() {
            return true;
        }
        if self.phase == CalibrationRequestPhase::Countdown
            && self.flow == Flow::MotionMoveOnly
            && let Some(ready) = self.ready_at.filter(|ready| now >= *ready)
        {
            self.enter_moving(ready);
            if !self.phase.active() {
                return false;
            }
        }
        if self.deadline_at.is_some_and(|deadline| now >= deadline) {
            let message = match (self.phase, self.last_blocking_reason.as_deref()) {
                (CalibrationRequestPhase::Moving, Some(reason)) => format!(
                    "Movement calibration did not finish within {MOVE_WINDOW_SECS:.0} s. Still blocking: {reason}"
                ),
                (CalibrationRequestPhase::Moving, None) => format!(
                    "Movement calibration did not finish within {MOVE_WINDOW_SECS:.0} s."
                ),
                (_, Some(reason)) => {
                    format!("Calibration timed out. Last blocked reason: {reason}")
                }
                (_, None) => "Calibration timed out before a stable neutral pose was captured. Start a new countdown to retry.".into(),
            };
            self.finish(CalibrationRequestPhase::TimedOut, message);
            return false;
        }
        if self.phase == CalibrationRequestPhase::Countdown
            && self.ready_at.is_some_and(|ready| now >= ready)
        {
            self.phase = CalibrationRequestPhase::WaitingStable;
            self.message =
                "Hold the neutral pose while calibration checks the headset and trackers.".into();
        }
        true
    }

    fn accept_clock(&mut self, now: f64) -> bool {
        let invalid = !now.is_finite() || now < 0.0;
        let backward = self.last_observed_at.is_some_and(|previous| now < previous);
        if invalid || backward {
            if self.phase.active() {
                self.finish(
                    CalibrationRequestPhase::Cancelled,
                    if invalid {
                        "Calibration cancelled: invalid clock reading."
                    } else {
                        "Calibration cancelled: clock moved backward. Start a new request with a stable clock."
                    },
                );
            }
            return false;
        }
        self.last_observed_at = Some(now);
        true
    }

    fn finish(&mut self, phase: CalibrationRequestPhase, message: impl Into<String>) {
        self.phase = phase;
        self.message = message.into();
        self.ready_at = None;
        self.deadline_at = None;
        self.next_attempt_at = None;
        self.attempt_pending = false;
        self.ready_since = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_flow_moves_after_neutral_and_needs_three_seconds_and_a_half_second_ready_hold() {
        let mut request = CalibrationRequestController::default();
        request.start_motion(0.0);
        assert!(!request.poll(2.9));
        assert!(request.poll(3.0));
        request.attempt_result(3.0, Err("Chest is moving".into()));
        assert!(request.poll(3.2));
        request.attempt_result(3.2, Ok(()));
        let moving = request.view(3.2);
        assert_eq!(moving.phase, CalibrationRequestPhase::Moving);
        assert!((moving.seconds_remaining - MOVE_WINDOW_SECS).abs() < 1e-9);
        // Neutral attempts are never authorized while moving.
        assert!(!request.poll(3.5));
        // Ready from the start: still needs 3 s of movement.
        for k in 3..=25 {
            let at = 3.2 + 0.1 * f64::from(k);
            assert!(!request.observe_motion(at, true, None), "at {at}");
        }
        assert!(request.observe_motion(6.2, true, None));
        // Only one attempt until its result arrives.
        assert!(!request.observe_motion(6.3, true, None));
        request.motion_finish_result(6.3, Err("Hip: not ready".into()));
        assert_eq!(request.view(6.3).phase, CalibrationRequestPhase::Moving);
        // A failure restarts the hold; a not-ready sample interrupts it.
        assert!(!request.observe_motion(6.4, true, None));
        assert!(!request.observe_motion(6.8, false, Some("Hip: tilt further".into())));
        assert!(!request.observe_motion(6.9, true, None));
        assert!(!request.observe_motion(7.3, true, None));
        assert!(request.observe_motion(7.4, true, None));
        request.motion_finish_result(7.4, Ok(()));
        assert_eq!(request.view(7.4).phase, CalibrationRequestPhase::Captured);
        assert!(!request.observe_motion(8.0, true, None));
    }

    #[test]
    fn motion_timeout_lists_the_last_blockers_and_cancel_works_in_every_stage() {
        let mut request = CalibrationRequestController::default();
        request.start_motion(0.0);
        assert!(request.poll(3.0));
        request.attempt_result(3.0, Ok(()));
        assert!(!request.observe_motion(10.0, false, Some("Left calf: tilt further".into())));
        assert!(!request.observe_motion(32.9, true, None));
        assert!(!request.observe_motion(33.0, true, None));
        let view = request.view(33.0);
        assert_eq!(view.phase, CalibrationRequestPhase::TimedOut);
        assert!(
            view.message.contains("Left calf: tilt further"),
            "{}",
            view.message
        );
        assert_eq!(request.view(100.0), view);

        for stage in 0..3 {
            let mut request = CalibrationRequestController::default();
            request.start_motion(0.0);
            let at = match stage {
                0 => 1.0,
                1 => {
                    assert!(request.poll(3.0));
                    request.attempt_result(3.0, Err("moving".into()));
                    3.05
                }
                _ => {
                    assert!(request.poll(3.0));
                    request.attempt_result(3.0, Ok(()));
                    assert!(!request.observe_motion(6.5, true, None));
                    7.0
                }
            };
            request.cancel(at, "user cancelled");
            assert_eq!(request.view(at).phase, CalibrationRequestPhase::Cancelled);
            request.motion_finish_result(at, Ok(()));
            assert_eq!(request.view(at).phase, CalibrationRequestPhase::Cancelled);
        }
    }

    #[test]
    fn move_only_flow_skips_neutral_after_the_countdown() {
        let mut request = CalibrationRequestController::default();
        request.start_motion_move(10.0);
        assert_eq!(request.view(12.9).phase, CalibrationRequestPhase::Countdown);
        assert!(!request.poll(13.0));
        let view = request.view(13.0);
        assert_eq!(view.phase, CalibrationRequestPhase::Moving);
        assert_eq!(view.seconds_remaining, MOVE_WINDOW_SECS);
        assert!((request.moving_secs(14.5) - 1.5).abs() < 1e-9);
        request.abort_motion(14.0, "Movement session interrupted");
        let aborted = request.view(14.0);
        assert_eq!(aborted.phase, CalibrationRequestPhase::TimedOut);
        assert!(aborted.message.contains("interrupted"));
        // A long stall past the whole window times out without a finish attempt.
        let mut request = CalibrationRequestController::default();
        request.start_motion_move(0.0);
        assert!(!request.observe_motion(40.0, true, None));
        assert_eq!(request.view(40.0).phase, CalibrationRequestPhase::TimedOut);
    }

    #[test]
    fn no_early_attempt_and_exact_three_second_transition() {
        let mut request = CalibrationRequestController::default();
        assert_eq!(request.view(10.0), CalibrationRequestView::default());
        request.start(10.0);
        assert_eq!(request.view(10.0).seconds_remaining, 3.0);
        for at in [10.0, 11.0, 12.0, 12.999999] {
            assert!(!request.poll(at));
            assert_eq!(request.view(at).phase, CalibrationRequestPhase::Countdown);
        }
        assert!(request.poll(13.0));
        assert_eq!(
            request.view(13.0).phase,
            CalibrationRequestPhase::WaitingStable
        );
        assert_eq!(request.view(13.0).seconds_remaining, 3.0);
        assert!(!request.poll(13.0));
    }

    #[test]
    fn retries_are_at_most_ten_hz_and_only_after_completed_attempts() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert!(request.poll(3.0));
        assert!(!request.poll(3.05));
        request.attempt_result(3.05, Err("moving".into()));
        assert!(!request.poll(3.09999));
        assert!(request.poll(3.1));
        request.attempt_result(3.1, Err("moving".into()));
        assert!(!request.poll(3.19999));
        assert!(request.poll(3.2));
    }

    #[test]
    fn exact_deadline_and_long_worker_pause_cannot_authorize_an_attempt() {
        for at in [6.0, 6.000001, 60.0] {
            let mut request = CalibrationRequestController::default();
            request.start(0.0);
            assert!(!request.poll(at));
            let view = request.view(at);
            assert_eq!(view.phase, CalibrationRequestPhase::TimedOut);
            assert_eq!(view.seconds_remaining, 0.0);
        }
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert!(request.poll(5.999));
        request.attempt_result(5.999, Err("not stable".into()));
        assert!(!request.poll(6.0));
    }

    #[test]
    fn forward_capture_preserves_countdown_and_allows_ten_seconds_to_settle() {
        let mut request = CalibrationRequestController::default();
        request.start_forward(10.0);
        assert_eq!(request.view(10.0).seconds_remaining, 3.0);
        assert!(!request.poll(12.999999));
        assert!(request.poll(13.0));
        assert_eq!(request.view(13.0).seconds_remaining, 10.0);
        request.attempt_result(13.0, Err("Chest is moving".into()));
        assert!(!request.poll(13.09999));
        assert!(request.poll(13.1));
        request.attempt_result(13.1, Err("Chest is moving".into()));
        // The old six-second deadline must not cut off a forward request.
        assert!(request.poll(16.0));
        request.attempt_result(16.0, Err("Chest is moving".into()));
        assert!(request.poll(22.9));
        request.attempt_result(22.9, Ok(()));
        assert_eq!(request.view(22.9).phase, CalibrationRequestPhase::Captured);
        assert!(!request.poll(24.0));
    }

    #[test]
    fn forward_capture_deadline_stays_fixed_and_requires_a_deliberate_restart() {
        for at in [13.0, 13.000001, 60.0] {
            let mut request = CalibrationRequestController::default();
            request.start_forward(0.0);
            assert!(request.poll(3.0));
            request.attempt_result(3.0, Err("Chest is moving".into()));
            assert!(request.poll(12.9));
            request.attempt_result(12.9, Err("Hip is moving".into()));
            assert!(!request.poll(at));
            let timed_out = request.view(at);
            assert_eq!(timed_out.phase, CalibrationRequestPhase::TimedOut);
            assert_eq!(timed_out.seconds_remaining, 0.0);
            assert!(timed_out.message.contains("Hip is moving"));
            assert_eq!(request.view(100.0), timed_out);
            assert!(!request.poll(101.0));
        }
        let mut request = CalibrationRequestController::default();
        request.start_forward(0.0);
        assert!(request.poll(12.9));
        request.attempt_result(13.0, Ok(()));
        assert_eq!(request.view(13.0).phase, CalibrationRequestPhase::TimedOut);
    }

    #[test]
    fn restarting_with_neutral_restores_its_shorter_window_and_clears_forward_work() {
        let mut request = CalibrationRequestController::default();
        request.start_forward(0.0);
        assert!(request.poll(3.0));
        let forward_id = request.view(3.0).request_id;
        request.start(4.0);
        request.attempt_result(4.0, Ok(()));
        assert_eq!(request.view(4.0).phase, CalibrationRequestPhase::Countdown);
        assert!(request.view(4.0).request_id > forward_id);
        assert!(request.poll(7.0));
        assert_eq!(request.view(7.0).seconds_remaining, 3.0);
        request.attempt_result(7.0, Err("Still moving".into()));
        assert!(!request.poll(10.0));
        assert_eq!(request.view(10.0).phase, CalibrationRequestPhase::TimedOut);
    }

    #[test]
    fn success_is_terminal_and_cannot_be_overwritten_by_later_failure() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert!(request.poll(3.0));
        request.attempt_result(3.0, Ok(()));
        let captured = request.view(3.0);
        assert_eq!(captured.phase, CalibrationRequestPhase::Captured);
        assert_eq!(captured.seconds_remaining, 0.0);
        assert!(!request.poll(4.0));
        request.attempt_result(4.0, Err("late failure".into()));
        request.cancel(5.0, "already completed");
        assert_eq!(request.view(7.0), captured);
    }

    #[test]
    fn timeout_preserves_last_blocking_reason_without_extending_deadline() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert!(request.poll(3.0));
        request.attempt_result(3.0, Err("headset moving".into()));
        assert!(request.poll(5.8));
        request.attempt_result(5.8, Err("left calf data is stale".into()));
        assert!(!request.poll(6.0));
        let timed_out = request.view(6.0);
        assert_eq!(timed_out.phase, CalibrationRequestPhase::TimedOut);
        assert!(timed_out.message.contains("left calf data is stale"));
        assert!(!timed_out.message.contains("headset moving"));
        assert_eq!(request.view(100.0), timed_out);
    }

    #[test]
    fn cancel_and_invalidate_clear_pending_attempts_and_do_not_retry() {
        for waiting in [false, true] {
            let mut request = CalibrationRequestController::default();
            request.start(0.0);
            let at = if waiting { 3.0 } else { 1.0 };
            if waiting {
                assert!(request.poll(at));
            }
            request.cancel(at, "user cancelled");
            request.attempt_result(at, Ok(()));
            assert_eq!(request.view(at).phase, CalibrationRequestPhase::Cancelled);
            assert!(!request.poll(5.0));
        }
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        request.invalidate(1.0, "tracker session changed");
        assert!(!request.poll(3.0));
        assert!(
            request
                .view(3.0)
                .message
                .contains("tracker session changed")
        );
    }

    #[test]
    fn invalidating_obsolete_terminal_results_clears_them_without_reusing_ids() {
        for terminal in [
            CalibrationRequestPhase::Captured,
            CalibrationRequestPhase::TimedOut,
            CalibrationRequestPhase::Cancelled,
        ] {
            let mut request = CalibrationRequestController::default();
            request.start(0.0);
            match terminal {
                CalibrationRequestPhase::Captured => {
                    assert!(request.poll(3.0));
                    request.attempt_result(3.0, Ok(()));
                }
                CalibrationRequestPhase::TimedOut => {
                    assert!(!request.poll(6.0));
                }
                CalibrationRequestPhase::Cancelled => request.cancel(1.0, "user cancelled"),
                _ => unreachable!(),
            }
            let previous_id = request.view(6.0).request_id;
            request.invalidate(7.0, "configuration changed");
            let view = request.view(7.0);
            assert_eq!(view.phase, CalibrationRequestPhase::Idle);
            assert_eq!(view.request_id, previous_id);
            assert!(view.message.is_empty());
            assert!(!request.poll(8.0));
            request.start(9.0);
            assert!(request.view(9.0).request_id > previous_id);
        }
    }

    #[test]
    fn restart_has_new_id_full_countdown_and_no_old_pending_result() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        let first_id = request.view(0.0).request_id;
        assert!(request.poll(3.0));
        request.start(3.1);
        request.attempt_result(3.1, Ok(()));
        let restarted = request.view(3.1);
        assert!(restarted.request_id > first_id);
        assert_eq!(restarted.phase, CalibrationRequestPhase::Countdown);
        assert!((restarted.seconds_remaining - 3.0).abs() < 1e-12);
        assert!(!request.poll(6.0));
        assert!(request.poll(6.1));
        request.attempt_result(6.1, Ok(()));
        assert_eq!(request.view(6.1).phase, CalibrationRequestPhase::Captured);
    }

    #[test]
    fn view_can_advance_phase_or_timeout_but_cannot_authorize_success() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert_eq!(
            request.view(3.0).phase,
            CalibrationRequestPhase::WaitingStable
        );
        request.attempt_result(3.0, Ok(()));
        assert_eq!(
            request.view(3.0).phase,
            CalibrationRequestPhase::WaitingStable
        );
        assert_eq!(request.view(6.0).phase, CalibrationRequestPhase::TimedOut);
        assert!(!request.poll(6.0));
    }

    #[test]
    fn expired_pending_result_cannot_report_capture() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        assert!(request.poll(5.9));
        request.attempt_result(6.0, Ok(()));
        assert_eq!(request.view(6.0).phase, CalibrationRequestPhase::TimedOut);
    }

    #[test]
    fn invalid_and_backward_clock_values_cancel_active_work() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            let mut request = CalibrationRequestController::default();
            request.start(bad);
            assert_eq!(request.view(0.0).phase, CalibrationRequestPhase::Cancelled);
            assert!(!request.poll(3.0));
            request.start(4.0);
            assert!(!request.poll(bad));
            assert_eq!(request.view(4.0).phase, CalibrationRequestPhase::Cancelled);
        }
        let mut request = CalibrationRequestController::default();
        request.start(10.0);
        assert!(!request.poll(9.0));
        assert_eq!(request.view(10.0).phase, CalibrationRequestPhase::Cancelled);
        request.start(11.0);
        assert!(request.poll(14.0));
        request.attempt_result(13.9, Ok(()));
        assert_eq!(request.view(14.0).phase, CalibrationRequestPhase::Cancelled);
    }

    #[test]
    fn invalid_view_time_cancels_without_nonfinite_serialized_values() {
        let mut request = CalibrationRequestController::default();
        request.start(0.0);
        let view = request.view(f64::NAN);
        assert_eq!(view.phase, CalibrationRequestPhase::Cancelled);
        assert_eq!(view.seconds_remaining, 0.0);
        let json = serde_json::to_string(&view).unwrap();
        assert!(json.contains("cancelled"));
        assert!(!json.contains("null"));
        assert!(!request.poll(3.0));
    }

    #[test]
    fn huge_finite_clock_cannot_collapse_countdown_or_deadline() {
        for now in [f64::MAX, (1u64 << 53) as f64] {
            let mut request = CalibrationRequestController::default();
            request.start(now);
            assert_eq!(request.view(now).phase, CalibrationRequestPhase::Cancelled);
            assert!(!request.poll(now));
        }
    }

    #[test]
    fn exhausted_request_counter_cannot_reuse_an_active_request_id() {
        let mut request = CalibrationRequestController {
            request_id: u64::MAX,
            ..Default::default()
        };
        request.start(0.0);
        assert_eq!(request.view(0.0).phase, CalibrationRequestPhase::Cancelled);
        assert!(!request.poll(3.0));
    }
}
