//! Hands-free multi-tap gestures on one torso tracker (chest or hip).
//!
//! Pure state machine: no clock, sockets or audio. The runtime feeds every
//! measured sample of every assigned tracker (gravity-removed body-frame
//! acceleration as sent by SlimeVR firmware 0.7.2, plus the orientation) and
//! polls with its own clock. A gesture is classified only after the sequence
//! has ended (no further tap within `max_spacing_secs`), so a triple tap is
//! never first reported as a double.
//!
//! Thresholds come from a worn recording (`live-20261115-tap-windows`, local
//! data only): two real chest taps peaked at 11.0 and 18.1 m/s² above the slow
//! baseline, with a 13–14 m/s² sample-to-sample jump at onset, 30–70 ms above
//! 5 m/s², 1–7° of tracker rotation, and were 0.24 s apart. Quiet standing
//! stays below 0.7 m/s² on every tracker. Events that the old detector
//! accepted while a tracker was handled rotated it 33–58° within 50–140 ms;
//! walking produced 10–35 m/s² on the calves while the chest saw 4–8 m/s².
//! These values describe one user and one firmware/IMU combination: they are
//! configurable and still require worn validation.
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, VecDeque};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TapGesture {
    Double,
    Triple,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TapEvent {
    pub gesture: TapGesture,
    /// Sensor ID of the tapped torso tracker.
    pub tracker: String,
    pub first_tap_at: f64,
    pub last_tap_at: f64,
    pub peaks_mps2: Vec<f32>,
}

/// What a recognised gesture does (pure policy, see `gesture_action`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GestureAction {
    /// Start "Calibrate by movement" exactly as the Setup button does.
    StartMovementCalibration,
    /// Cancel the running calibration.
    CancelCalibration,
    /// Request the gated heading recovery.
    RequestHeadingRecovery,
    /// A double tap during a calibration is ignored (it could be movement).
    Ignore,
}

pub fn gesture_action(gesture: TapGesture, calibration_active: bool) -> GestureAction {
    match (gesture, calibration_active) {
        (TapGesture::Triple, false) => GestureAction::StartMovementCalibration,
        (TapGesture::Triple, true) => GestureAction::CancelCalibration,
        (TapGesture::Double, false) => GestureAction::RequestHeadingRecovery,
        (TapGesture::Double, true) => GestureAction::Ignore,
    }
}

/// Outcome of one finished tap sequence.
#[derive(Clone, Debug, PartialEq)]
pub enum TapVerdict {
    Gesture(TapEvent),
    /// A finished sequence that is not a gesture (single tap, 4+ taps,
    /// whole-body event, handled tracker, ...). Diagnostic only.
    Ignored {
        tracker: String,
        taps: usize,
        reason: String,
    },
}

/// Recogniser thresholds. Accelerations are high-passed magnitudes in m/s²
/// (the slowly varying firmware gravity-removal bias is subtracted).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TapConfig {
    /// Minimum high-passed peak of one tap (real taps: 11–18 m/s²).
    pub impulse_mps2: f32,
    /// Minimum sample-to-sample change of the acceleration vector inside the
    /// impulse (real taps: 13–14 m/s²; walking ramps: 2–3 m/s² per sample at
    /// about 100 Hz). Rejects slow pushes and sways.
    pub min_jerk_mps2: f32,
    /// Below this the tracker counts as quiet (re-arm and ring-down level).
    pub quiet_mps2: f32,
    /// An impulse ends when the signal drops below max(quiet, ratio × peak).
    pub release_ratio: f32,
    /// Quiet time required before the next tap may start.
    pub rearm_secs: f64,
    /// Longest time above the release level for one tap (real: 30–110 ms).
    pub max_impulse_secs: f64,
    /// Exceedances this soon after a tap are its ring-down, not a new event.
    pub ring_grace_secs: f64,
    /// Tap onsets closer than this spoil the sequence.
    pub min_spacing_secs: f64,
    /// A sequence ends after this long without another tap (real gap: 0.24 s).
    pub max_spacing_secs: f64,
    /// A tracker that rotates more than this during one impulse is being
    /// handled or knocked, not tapped while strapped on (real taps: 1–7°).
    pub max_rotation_degrees: f32,
    /// Whole-body check: another tracker's peak within this window around a
    /// tap...
    pub body_window_secs: f64,
    /// ...that reaches max(floor, ratio × tap peak) rejects the sequence.
    pub body_ratio: f32,
    pub body_floor_mps2: f32,
    /// No new sequence starts this soon after a recognised gesture.
    pub cooldown_secs: f64,
    /// Time constant of the slow acceleration baseline (high-pass).
    pub baseline_tau_secs: f64,
}

impl Default for TapConfig {
    fn default() -> Self {
        Self {
            impulse_mps2: 8.0,
            min_jerk_mps2: 8.0,
            quiet_mps2: 5.0,
            release_ratio: 0.3,
            rearm_secs: 0.05,
            max_impulse_secs: 0.16,
            ring_grace_secs: 0.25,
            min_spacing_secs: 0.12,
            max_spacing_secs: 0.7,
            max_rotation_degrees: 25.0,
            body_window_secs: 0.1,
            body_ratio: 0.5,
            body_floor_mps2: 6.0,
            cooldown_secs: 1.5,
            baseline_tau_secs: 0.4,
        }
    }
}

impl TapConfig {
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            ("impulse_mps2", f64::from(self.impulse_mps2)),
            ("min_jerk_mps2", f64::from(self.min_jerk_mps2)),
            ("quiet_mps2", f64::from(self.quiet_mps2)),
            ("rearm_secs", self.rearm_secs),
            ("max_impulse_secs", self.max_impulse_secs),
            ("max_spacing_secs", self.max_spacing_secs),
            ("max_rotation_degrees", f64::from(self.max_rotation_degrees)),
            ("body_window_secs", self.body_window_secs),
            ("body_floor_mps2", f64::from(self.body_floor_mps2)),
            ("baseline_tau_secs", self.baseline_tau_secs),
        ];
        for (name, value) in positive {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("{name} must be positive"));
            }
        }
        for (name, value) in [
            ("release_ratio", f64::from(self.release_ratio)),
            ("body_ratio", f64::from(self.body_ratio)),
        ] {
            if !(0.0..=1.0).contains(&value) {
                return Err(format!("{name} must be within 0..1"));
            }
        }
        for (name, value) in [
            ("ring_grace_secs", self.ring_grace_secs),
            ("min_spacing_secs", self.min_spacing_secs),
            ("cooldown_secs", self.cooldown_secs),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!("{name} must be non-negative"));
            }
        }
        if self.quiet_mps2 >= self.impulse_mps2 {
            return Err("quiet_mps2 must be below impulse_mps2".into());
        }
        if self.min_spacing_secs >= self.max_spacing_secs {
            return Err("min_spacing_secs must be below max_spacing_secs".into());
        }
        Ok(())
    }
}

/// Bounds memory regardless of input rate or tracker count.
const MAX_TRACKERS: usize = 32;
const HISTORY_SECS: f64 = 3.0;
const MAX_HISTORY: usize = 512;
const MAX_TAPS: usize = 8;
/// A silence longer than this inside an impulse ends it at the last sample.
const IMPULSE_GAP_SECS: f64 = 0.1;
/// Sample-to-sample jerk is only meaningful between neighbouring samples.
const JERK_GAP_SECS: f64 = 0.05;

#[derive(Clone, Copy, Debug)]
struct Impulse {
    onset: f64,
    last_at: f64,
    peak: f32,
    max_jerk: f32,
    reference: Quat,
    max_rotation: f32,
}

#[derive(Clone, Copy, Debug)]
enum Phase {
    Settled,
    Impulse(Impulse),
    Ringing { ended_at: f64 },
    Moving,
}

#[derive(Debug)]
struct TrackerState {
    last_at: Option<f64>,
    last_accel: Option<Vec3>,
    last_orientation: Option<Quat>,
    baseline: Vec3,
    phase: Phase,
    /// Start of the current quiet stretch (None while above the quiet level).
    quiet_since: Option<f64>,
    /// High-passed magnitudes for the whole-body check.
    history: VecDeque<(f64, f32)>,
}

impl Default for TrackerState {
    fn default() -> Self {
        Self {
            last_at: None,
            last_accel: None,
            last_orientation: None,
            baseline: Vec3::ZERO,
            phase: Phase::Settled,
            // A tracker never seen before counts as quiet: SlimeVR firmware
            // suppresses unchanged rotations, so a still hip may be silent.
            quiet_since: Some(f64::NEG_INFINITY),
            history: VecDeque::new(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Tap {
    onset: f64,
    end: f64,
    peak: f32,
}

#[derive(Debug)]
struct Sequence {
    taps: Vec<Tap>,
    last_activity: f64,
    spoiled: Option<String>,
}

#[derive(Debug, Default)]
pub struct TapRecognizer {
    config: TapConfig,
    trackers: BTreeMap<String, TrackerState>,
    sequences: BTreeMap<String, Sequence>,
    cooldown_until: f64,
}

impl TapRecognizer {
    pub fn new(config: TapConfig) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            ..Self::default()
        })
    }

    pub fn config(&self) -> &TapConfig {
        &self.config
    }

    /// Forgets all trackers, sequences and the cooldown.
    pub fn reset(&mut self) {
        self.trackers.clear();
        self.sequences.clear();
        self.cooldown_until = 0.0;
    }

    /// Feeds one measured sample. `torso` marks the chest/hip trackers whose
    /// taps count; every other assigned tracker is still fed so whole-body
    /// events can be recognised. Nonfinite or out-of-order samples are ignored.
    pub fn sample(
        &mut self,
        id: &str,
        torso: bool,
        at: f64,
        acceleration: [f32; 3],
        orientation_xyzw: [f32; 4],
    ) {
        let accel = Vec3::from_array(acceleration);
        let orientation = Quat::from_array(orientation_xyzw);
        if !at.is_finite()
            || !accel.is_finite()
            || !orientation.is_finite()
            || orientation.length_squared() < 1e-6
        {
            return;
        }
        let orientation = orientation.normalize();
        if !self.trackers.contains_key(id) && self.trackers.len() >= MAX_TRACKERS {
            return;
        }
        let config = self.config;
        let state = self.trackers.entry(id.to_owned()).or_default();
        if state.last_at.is_some_and(|last| at < last) {
            return;
        }
        let gap = state.last_at.map_or(f64::INFINITY, |last| at - last);
        // A silent tracker is a still tracker (firmware rotation suppression).
        if gap > IMPULSE_GAP_SECS {
            if let Phase::Impulse(impulse) = state.phase {
                let end = impulse.last_at;
                state.phase = Phase::Ringing { ended_at: end };
                state.quiet_since = Some(end);
                finish_impulse(
                    &config,
                    &mut self.sequences,
                    id,
                    torso,
                    impulse,
                    end,
                    self.cooldown_until,
                );
            } else if state.quiet_since.is_none() {
                state.quiet_since = state.last_at;
            }
            if matches!(state.phase, Phase::Moving) {
                state.phase = Phase::Settled;
            }
        }
        let hp = (accel - state.baseline).length();
        let jerk = match state.last_accel {
            Some(previous) if gap <= JERK_GAP_SECS => (accel - previous).length(),
            _ => hp,
        };
        state.history.push_back((at, hp));
        while state.history.len() > MAX_HISTORY
            || state
                .history
                .front()
                .is_some_and(|(t, _)| at - t > HISTORY_SECS)
        {
            state.history.pop_front();
        }
        if torso {
            step_torso(
                &config,
                state,
                &mut self.sequences,
                id,
                at,
                hp,
                jerk,
                orientation,
                self.cooldown_until,
            );
        }
        if !matches!(state.phase, Phase::Impulse(_)) {
            let dt = if gap.is_finite() { gap } else { 0.0 };
            let alpha = (dt / config.baseline_tau_secs).clamp(0.0, 1.0) as f32;
            state.baseline += (accel - state.baseline) * alpha;
        }
        state.last_at = Some(at);
        state.last_accel = Some(accel);
        state.last_orientation = Some(orientation);
    }

    /// Classifies every sequence that has ended by `now`.
    pub fn poll(&mut self, now: f64) -> Vec<TapVerdict> {
        if !now.is_finite() {
            return Vec::new();
        }
        let ended: Vec<String> = self
            .sequences
            .iter()
            .filter(|(id, sequence)| {
                now - sequence.last_activity > self.config.max_spacing_secs
                    && !self
                        .trackers
                        .get(*id)
                        .is_some_and(|t| matches!(t.phase, Phase::Impulse(_)))
            })
            .map(|(id, _)| id.clone())
            .collect();
        let mut verdicts = Vec::new();
        for id in ended {
            let Some(sequence) = self.sequences.remove(&id) else {
                continue;
            };
            let verdict = self.classify(&id, sequence);
            if matches!(verdict, TapVerdict::Gesture(_)) {
                self.cooldown_until = now + self.config.cooldown_secs;
                // Echoes of this gesture on the other torso tracker are void.
                self.sequences.clear();
            }
            verdicts.push(verdict);
        }
        verdicts
    }

    fn classify(&self, id: &str, sequence: Sequence) -> TapVerdict {
        let taps = sequence.taps.len();
        let ignored = |reason: String| TapVerdict::Ignored {
            tracker: id.to_owned(),
            taps,
            reason,
        };
        if let Some(reason) = sequence.spoiled {
            return ignored(reason);
        }
        let gesture = match taps {
            2 => TapGesture::Double,
            3 => TapGesture::Triple,
            1 => return ignored("single tap".into()),
            n => return ignored(format!("{n} taps")),
        };
        for tap in &sequence.taps {
            let limit = self
                .config
                .body_floor_mps2
                .max(self.config.body_ratio * tap.peak);
            let from = tap.onset - self.config.body_window_secs;
            let to = tap.end + self.config.body_window_secs;
            for (other, state) in self.trackers.iter().filter(|(other, _)| *other != id) {
                if let Some(peak) = state
                    .history
                    .iter()
                    .filter(|(t, _)| (from..=to).contains(t))
                    .map(|(_, hp)| *hp)
                    .reduce(f32::max)
                    .filter(|peak| *peak >= limit)
                {
                    return ignored(format!(
                        "whole-body event: {other} reached {peak:.1} m/s² during a {:.1} m/s² tap",
                        tap.peak
                    ));
                }
            }
        }
        TapVerdict::Gesture(TapEvent {
            gesture,
            tracker: id.to_owned(),
            first_tap_at: sequence.taps[0].onset,
            last_tap_at: sequence.taps[taps - 1].onset,
            peaks_mps2: sequence.taps.iter().map(|t| t.peak).collect(),
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn step_torso(
    config: &TapConfig,
    state: &mut TrackerState,
    sequences: &mut BTreeMap<String, Sequence>,
    id: &str,
    at: f64,
    hp: f32,
    jerk: f32,
    orientation: Quat,
    cooldown_until: f64,
) {
    match state.phase {
        Phase::Impulse(mut impulse) => {
            impulse.last_at = at;
            impulse.peak = impulse.peak.max(hp);
            impulse.max_jerk = impulse.max_jerk.max(jerk);
            impulse.max_rotation = impulse
                .max_rotation
                .max(impulse.reference.angle_between(orientation).to_degrees());
            if hp < config.quiet_mps2.max(config.release_ratio * impulse.peak) {
                state.phase = Phase::Ringing { ended_at: at };
                state.quiet_since = (hp < config.quiet_mps2).then_some(at);
                finish_impulse(config, sequences, id, true, impulse, at, cooldown_until);
            } else if at - impulse.onset > config.max_impulse_secs {
                state.phase = Phase::Moving;
                state.quiet_since = None;
                spoil(sequences, id, at, "sustained movement, not a tap");
            } else {
                state.phase = Phase::Impulse(impulse);
            }
        }
        phase => {
            if hp >= config.impulse_mps2 {
                let armed = state
                    .quiet_since
                    .is_some_and(|quiet| at - quiet >= config.rearm_secs);
                if armed {
                    let reference = state.last_orientation.unwrap_or(orientation);
                    state.phase = Phase::Impulse(Impulse {
                        onset: at,
                        last_at: at,
                        peak: hp,
                        max_jerk: jerk,
                        reference,
                        max_rotation: reference.angle_between(orientation).to_degrees(),
                    });
                } else if matches!(phase, Phase::Ringing { ended_at } if at - ended_at <= config.ring_grace_secs)
                {
                    // Ring-down of the previous tap.
                } else {
                    state.phase = Phase::Moving;
                    spoil(sequences, id, at, "moving between taps");
                }
                state.quiet_since = None;
            } else if hp >= config.quiet_mps2 {
                state.quiet_since = None;
                if matches!(phase, Phase::Moving)
                    && let Some(sequence) = sequences.get_mut(id)
                {
                    sequence.last_activity = at;
                }
            } else {
                let since = *state.quiet_since.get_or_insert(at);
                if at - since >= config.rearm_secs {
                    state.phase = Phase::Settled;
                }
            }
        }
    }
}

fn finish_impulse(
    config: &TapConfig,
    sequences: &mut BTreeMap<String, Sequence>,
    id: &str,
    torso: bool,
    impulse: Impulse,
    end: f64,
    cooldown_until: f64,
) {
    if !torso || impulse.onset < cooldown_until {
        return;
    }
    let problem = if impulse.max_rotation > config.max_rotation_degrees {
        Some(format!(
            "tracker rotated {:.0}° during the impulse (handled or knocked)",
            impulse.max_rotation
        ))
    } else if impulse.max_jerk < config.min_jerk_mps2 {
        Some("impulse too soft to be a tap".to_owned())
    } else {
        None
    };
    if let Some(problem) = problem {
        spoil(sequences, id, impulse.onset, &problem);
        return;
    }
    let sequence = sequences.entry(id.to_owned()).or_insert_with(|| Sequence {
        taps: Vec::new(),
        last_activity: impulse.onset,
        spoiled: None,
    });
    if let Some(previous) = sequence.taps.last()
        && impulse.onset - previous.onset < config.min_spacing_secs
    {
        sequence
            .spoiled
            .get_or_insert_with(|| "taps too close together".into());
    }
    if sequence.taps.len() < MAX_TAPS {
        sequence.taps.push(Tap {
            onset: impulse.onset,
            end,
            peak: impulse.peak,
        });
    } else {
        sequence
            .spoiled
            .get_or_insert_with(|| "too many taps".into());
    }
    sequence.last_activity = impulse.onset;
}

fn spoil(sequences: &mut BTreeMap<String, Sequence>, id: &str, at: f64, reason: &str) {
    let sequence = sequences.entry(id.to_owned()).or_insert_with(|| Sequence {
        taps: Vec::new(),
        last_activity: at,
        spoiled: None,
    });
    sequence.spoiled.get_or_insert_with(|| reason.to_owned());
    sequence.last_activity = sequence.last_activity.max(at);
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHEST: &str = "chest";
    const HIP: &str = "hip";
    const LEGS: [&str; 4] = ["left_thigh", "right_thigh", "left_calf", "right_calf"];
    const DT: f64 = 0.01;

    /// A real-data-shaped tap (446.97 s in the worn recording): onset jump to
    /// ~11 m/s², sign reversal, ring-down below 5 m/s² within ~40 ms.
    fn tap_shape(scale: f32) -> Vec<[f32; 3]> {
        [
            [1.2, -7.2, -8.3],
            [0.6, -5.9, -5.6],
            [-2.8, 2.9, 8.0],
            [-2.3, 0.2, 0.0],
            [-0.2, 3.3, -1.7],
            [2.0, 3.7, 1.2],
            [0.8, 1.0, -0.3],
        ]
        .iter()
        .map(|v| [v[0] * scale, v[1] * scale, v[2] * scale])
        .collect()
    }

    struct Sim {
        recognizer: TapRecognizer,
        t: f64,
        verdicts: Vec<TapVerdict>,
        /// Leg trackers get samples (quiet unless overridden) every step.
        legs: bool,
        /// Bunch this many samples per arrival time (Wi-Fi bunching).
        bunch: usize,
        step: usize,
        noise: f32,
    }

    impl Sim {
        fn new() -> Self {
            Self {
                recognizer: TapRecognizer::new(TapConfig::default()).unwrap(),
                t: 10.0,
                verdicts: Vec::new(),
                legs: true,
                bunch: 1,
                step: 0,
                noise: 0.3,
            }
        }
        fn arrival(&self) -> f64 {
            if self.bunch <= 1 {
                self.t
            } else {
                // Samples are produced every 10 ms but arrive in bursts.
                let burst = (self.step / self.bunch) as f64 * self.bunch as f64 * DT;
                10.0 + burst + (self.step % self.bunch) as f64 * 0.0002
            }
        }
        fn wobble(&self, k: usize) -> [f32; 3] {
            let p = (self.step * 7 + k * 13) as f32;
            [
                self.noise * (p * 0.37).sin(),
                self.noise * (p * 0.53).cos(),
                self.noise * (p * 0.71).sin(),
            ]
        }
        /// One 10 ms step: chest sample plus quiet (or given) other trackers.
        fn step_with(&mut self, chest: [f32; 3], chest_rot_deg: f32, others: [f32; 3]) {
            let at = self.arrival();
            let q = Quat::from_rotation_x(chest_rot_deg.to_radians()).to_array();
            let w = self.wobble(0);
            let a = [chest[0] + w[0], chest[1] + w[1], chest[2] + w[2]];
            self.recognizer.sample(CHEST, true, at, a, q);
            let hip = self.wobble(1);
            self.recognizer
                .sample(HIP, true, at, hip, Quat::IDENTITY.to_array());
            if self.legs {
                for (k, leg) in LEGS.iter().enumerate() {
                    let w = self.wobble(k + 2);
                    let a = [others[0] + w[0], others[1] + w[1], others[2] + w[2]];
                    self.recognizer
                        .sample(leg, false, at, a, Quat::IDENTITY.to_array());
                }
            }
            self.t += DT;
            self.step += 1;
            let now = self.arrival().max(self.t);
            self.verdicts.extend(self.recognizer.poll(now));
        }
        fn quiet(&mut self, secs: f64) {
            for _ in 0..(secs / DT).round() as usize {
                self.step_with([0.0; 3], 0.0, [0.0; 3]);
            }
        }
        fn tap(&mut self, scale: f32) {
            for a in tap_shape(scale) {
                self.step_with(a, 1.5, [0.0; 3]);
            }
        }
        /// Taps separated by `gap` seconds onset to onset.
        fn taps(&mut self, n: usize, gap: f64, scale: f32) {
            for i in 0..n {
                self.tap(scale);
                if i + 1 < n {
                    self.quiet(gap - tap_shape(1.0).len() as f64 * DT);
                }
            }
        }
        fn gestures(&self) -> Vec<TapGesture> {
            self.verdicts
                .iter()
                .filter_map(|v| match v {
                    TapVerdict::Gesture(e) => Some(e.gesture),
                    _ => None,
                })
                .collect()
        }
        fn ignored(&self) -> Vec<String> {
            self.verdicts
                .iter()
                .filter_map(|v| match v {
                    TapVerdict::Ignored { reason, .. } => Some(reason.clone()),
                    _ => None,
                })
                .collect()
        }
    }

    #[test]
    fn gesture_map_starts_cancels_and_recovers() {
        use GestureAction::*;
        assert_eq!(
            gesture_action(TapGesture::Triple, false),
            StartMovementCalibration
        );
        assert_eq!(gesture_action(TapGesture::Triple, true), CancelCalibration);
        assert_eq!(
            gesture_action(TapGesture::Double, false),
            RequestHeadingRecovery
        );
        assert_eq!(gesture_action(TapGesture::Double, true), Ignore);
    }

    #[test]
    fn default_config_is_valid_and_bad_configs_are_rejected() {
        assert!(TapConfig::default().validate().is_ok());
        let bad = TapConfig {
            quiet_mps2: 9.0,
            ..TapConfig::default()
        };
        assert!(TapRecognizer::new(bad).is_err());
        let bad = TapConfig {
            max_spacing_secs: f64::NAN,
            ..TapConfig::default()
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn double_tap_is_recognised_once_after_the_sequence_ends() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.taps(2, 0.24, 1.0);
        // Not yet: a third tap could still follow.
        sim.quiet(0.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
        sim.quiet(1.0);
        assert_eq!(sim.gestures(), vec![TapGesture::Double]);
    }

    #[test]
    fn triple_tap_is_never_reported_as_a_double_first() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.taps(3, 0.3, 1.0);
        sim.quiet(1.5);
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Triple],
            "{:?}",
            sim.verdicts
        );
        assert_eq!(sim.verdicts.len(), 1);
    }

    #[test]
    fn irregular_triple_with_a_strong_tap_is_recognised() {
        // Real spacing 0.24 s, then a slower one; the last tap much stronger.
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.tap(1.0);
        sim.quiet(0.24 - 0.07);
        sim.tap(1.6);
        sim.quiet(0.48 - 0.07);
        sim.tap(3.0);
        sim.quiet(1.5);
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Triple],
            "{:?}",
            sim.verdicts
        );
    }

    #[test]
    fn single_tap_and_four_taps_produce_no_gesture() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.tap(1.0);
        sim.quiet(1.5);
        sim.taps(4, 0.25, 1.0);
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
        let ignored = sim.ignored();
        assert!(ignored.iter().any(|r| r == "single tap"), "{ignored:?}");
        assert!(ignored.iter().any(|r| r == "4 taps"), "{ignored:?}");
    }

    #[test]
    fn slow_taps_are_separate_singles() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.taps(3, 1.0, 1.0);
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
        assert_eq!(sim.ignored().len(), 3);
    }

    #[test]
    fn walking_with_heel_strikes_on_the_legs_is_not_a_gesture() {
        // Real walking: calves 10-35 m/s² at heel strike while the chest sees a
        // sharp but smaller echo.
        let mut sim = Sim::new();
        sim.quiet(1.0);
        for _ in 0..6 {
            for (k, chest) in [9.0f32, 4.0, 2.0, 1.0].iter().enumerate() {
                let legs = [0.0, [22.0f32, 9.0, 4.0, 1.0][k], 0.0];
                sim.step_with([*chest, 0.0, -chest], 3.0, legs);
            }
            sim.step_with([0.0; 3], 0.0, [0.0; 3]);
            for _ in 0..25 {
                sim.step_with([2.0, 1.0, 0.0], 0.0, [3.0, 0.0, 0.0]);
            }
        }
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
    }

    #[test]
    fn a_jump_landing_is_rejected_as_a_whole_body_event() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        // Two landings 0.3 s apart (a small hop twice): every tracker spikes.
        for _ in 0..2 {
            for scale in [14.0f32, 6.0, 2.0, 0.5] {
                sim.step_with([scale, -scale, 0.0], 2.0, [0.0, 2.0 * scale, 0.0]);
            }
            sim.quiet(0.26);
        }
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
        assert!(
            sim.ignored().iter().any(|r| r.starts_with("whole-body")),
            "{:?}",
            sim.verdicts
        );
    }

    #[test]
    fn handled_tracker_rotating_fast_is_not_a_tap() {
        // 869/876 s in the recording: 33-37° within the impulse, turning at
        // 500-1800°/s; here 1200°/s.
        let mut sim = Sim::new();
        sim.quiet(1.0);
        for _ in 0..3 {
            for (i, a) in tap_shape(2.5).into_iter().enumerate() {
                sim.step_with(a, 12.0 * (i + 1) as f32, [0.0; 3]);
            }
            sim.quiet(0.2);
        }
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
        assert!(sim.ignored().iter().any(|r| r.contains("rotated")));
    }

    #[test]
    fn bunched_arrival_times_still_recognise_a_triple() {
        let mut sim = Sim::new();
        sim.bunch = 4;
        sim.quiet(1.0);
        sim.taps(3, 0.3, 1.0);
        sim.quiet(1.5);
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Triple],
            "{:?}",
            sim.verdicts
        );
    }

    #[test]
    fn a_noisy_leg_tracker_does_not_block_taps_and_noise_alone_is_not_a_tap() {
        let mut sim = Sim::new();
        sim.noise = 1.5;
        sim.quiet(2.0);
        assert!(sim.verdicts.is_empty(), "{:?}", sim.verdicts);
        sim.taps(2, 0.25, 1.0);
        sim.quiet(1.5);
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Double],
            "{:?}",
            sim.verdicts
        );
    }

    #[test]
    fn soft_push_without_a_sharp_onset_is_not_a_tap() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        for _ in 0..2 {
            for m in [2.0f32, 4.0, 6.0, 8.0, 9.0, 8.0, 6.0, 4.0, 2.0, 0.5] {
                sim.step_with([m, 0.0, 0.0], 0.5, [0.0; 3]);
            }
            sim.quiet(0.2);
        }
        sim.quiet(1.5);
        assert!(sim.gestures().is_empty(), "{:?}", sim.verdicts);
    }

    #[test]
    fn cooldown_suppresses_an_immediate_second_gesture_and_bias_is_high_passed() {
        let mut sim = Sim::new();
        sim.quiet(1.0);
        sim.taps(2, 0.25, 1.0);
        sim.quiet(0.75);
        assert_eq!(sim.gestures(), vec![TapGesture::Double]);
        sim.taps(2, 0.25, 1.0);
        sim.quiet(1.5);
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Double],
            "{:?}",
            sim.verdicts
        );
        // A constant 7.5 m/s² firmware gravity-removal bias (seen after a
        // tracker was handled) is absorbed by the baseline.
        for _ in 0..200 {
            sim.step_with([7.5, 0.0, 0.0], 0.0, [0.0; 3]);
        }
        for _ in 0..2 {
            for a in tap_shape(1.0) {
                sim.step_with([a[0] + 7.5, a[1], a[2]], 1.5, [0.0; 3]);
            }
            for _ in 0..18 {
                sim.step_with([7.5, 0.0, 0.0], 0.0, [0.0; 3]);
            }
        }
        for _ in 0..150 {
            sim.step_with([7.5, 0.0, 0.0], 0.0, [0.0; 3]);
        }
        assert_eq!(
            sim.gestures(),
            vec![TapGesture::Double, TapGesture::Double],
            "{:?}",
            sim.verdicts
        );
    }

    #[test]
    fn silent_hip_tracker_can_be_tapped_after_a_gap() {
        // SlimeVR suppresses unchanged rotations: a still hip sends nothing.
        let mut r = TapRecognizer::new(TapConfig::default()).unwrap();
        let q = Quat::IDENTITY.to_array();
        r.sample(HIP, true, 1.0, [0.1, 0.0, 0.0], q);
        let mut t = 5.0;
        for _ in 0..2 {
            for a in tap_shape(1.0) {
                r.sample(HIP, true, t, a, q);
                t += DT;
            }
            t += 0.2;
        }
        assert!(r.poll(t).is_empty());
        let verdicts = r.poll(t + 1.0);
        assert!(
            matches!(&verdicts[..], [TapVerdict::Gesture(TapEvent { gesture: TapGesture::Double, tracker, .. })] if tracker == HIP),
            "{verdicts:?}"
        );
    }

    #[test]
    fn nonfinite_and_out_of_order_samples_are_ignored() {
        let mut r = TapRecognizer::new(TapConfig::default()).unwrap();
        let q = Quat::IDENTITY.to_array();
        r.sample(CHEST, true, 1.0, [0.0; 3], q);
        r.sample(CHEST, true, 1.01, [f32::NAN, 0.0, 0.0], q);
        r.sample(CHEST, true, f64::INFINITY, [0.0; 3], q);
        r.sample(CHEST, true, 0.5, [50.0, 0.0, 0.0], q);
        r.sample(CHEST, true, 1.02, [0.0; 3], [0.0; 4]);
        assert!(r.poll(5.0).is_empty());
        assert!(r.poll(f64::NAN).is_empty());
    }
}
