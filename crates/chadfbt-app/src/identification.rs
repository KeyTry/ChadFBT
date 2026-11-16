//! Guided, user-confirmed identification from observed quaternion changes.
//! Scores describe the path visible to snapshot polling, not an integrated IMU
//! path or motion confidence. No assignment or body-role inference occurs here.
use std::collections::BTreeMap;

pub const WINDOW_SECONDS: f64 = 3.0;
const CONNECTED_AGE_SECONDS: f32 = 1.5;
const MAX_OBSERVATION_GAP_SECONDS: f64 = 0.75;
const MAX_NEW_SAMPLE_AGE_SECONDS: f32 = 0.5;
const MIN_STEP_DEGREES: f64 = 0.75;
const MIN_ACTIVITY_DEGREES: f64 = 60.0;
const OTHER_MOVEMENT_DEGREES: f64 = 25.0;
const MIN_OBSERVATIONS: u32 = 3;

#[derive(Clone, Copy)]
pub struct Observation<'a> {
    pub id: &'a str,
    pub quaternion_xyzw: [f32; 4],
    /// Actual received rotation counter; repeated firmware-held tokens do not advance it.
    pub rotation_packets: u64,
    /// Age of the original received quaternion, never the age of its held token.
    pub rotation_age_seconds: f32,
    pub device_packet_age_seconds: Option<f32>,
}

impl Observation<'_> {
    fn connected(&self) -> bool {
        self.device_packet_age_seconds
            .is_some_and(|age| age.is_finite() && (0.0..CONNECTED_AGE_SECONDS).contains(&age))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    Candidate { tracker_id: String },
    NoMovement,
    Ambiguous,
    CandidateDisconnected,
    Interrupted,
}

#[derive(Debug, Clone)]
pub struct Activity {
    pub tracker_id: String,
    pub observed_degrees: f64,
    pub observed_degrees_per_second: f64,
    pub new_observations: u32,
    pub connected: bool,
}

struct Device {
    activity_anchor: Option<[f64; 4]>,
    last_counter: u64,
    observed_degrees: f64,
    observations: u32,
    connected: bool,
    lost_connection: bool,
}

pub struct Session {
    started_at: f64,
    last_update_at: f64,
    elapsed: f64,
    devices: BTreeMap<String, Device>,
    interrupted: bool,
    outcome: Option<Outcome>,
}

impl Session {
    pub fn start(now: f64, observations: &[Observation<'_>]) -> Self {
        let devices = observations
            .iter()
            .map(|observation| {
                (
                    observation.id.to_owned(),
                    Device {
                        // The first new sample inside this capture establishes
                        // a baseline. Never count a jump from pre-capture data.
                        activity_anchor: None,
                        last_counter: observation.rotation_packets,
                        observed_degrees: 0.0,
                        observations: 0,
                        connected: observation.connected(),
                        lost_connection: false,
                    },
                )
            })
            .collect();
        Self {
            started_at: now,
            last_update_at: now,
            elapsed: 0.0,
            devices,
            interrupted: !now.is_finite(),
            outcome: None,
        }
    }

    pub fn observe(&mut self, now: f64, observations: &[Observation<'_>]) {
        if self.outcome.is_some() {
            return;
        }
        if !now.is_finite() || !self.started_at.is_finite() {
            self.outcome = Some(Outcome::Interrupted);
            return;
        }
        if !now.is_finite()
            || now < self.last_update_at
            || now - self.last_update_at > MAX_OBSERVATION_GAP_SECONDS
        {
            self.interrupted = true;
        }
        self.last_update_at = now;
        self.elapsed = (now - self.started_at).clamp(0.0, WINDOW_SECONDS);
        // Changing the cohort makes comparison unfair; ask the user to retry.
        if observations.len() != self.devices.len()
            || observations
                .iter()
                .any(|o| !self.devices.contains_key(o.id))
        {
            self.interrupted = true;
        }
        for (id, device) in &mut self.devices {
            let Some(observation) = observations.iter().find(|o| o.id == id) else {
                device.connected = false;
                device.lost_connection = true;
                continue;
            };
            device.connected = observation.connected();
            device.lost_connection |= !device.connected;
            if observation.rotation_packets < device.last_counter {
                self.interrupted = true;
            }
            if observation.rotation_packets == device.last_counter {
                continue;
            }
            device.last_counter = observation.rotation_packets;
            let Some(quaternion) = normalized(observation.quaternion_xyzw) else {
                self.interrupted = true;
                continue;
            };
            let age = observation.rotation_age_seconds;
            let received_at = now - f64::from(age);
            let in_window = age.is_finite()
                && (0.0..=MAX_NEW_SAMPLE_AGE_SECONDS).contains(&age)
                && received_at >= self.started_at
                && received_at <= self.started_at + WINDOW_SECONDS;
            if in_window && device.connected {
                if let Some(previous) = device.activity_anchor {
                    let dot = previous
                        .iter()
                        .zip(quaternion)
                        .map(|(a, b)| a * b)
                        .sum::<f64>()
                        .abs()
                        .clamp(0.0, 1.0);
                    let degrees = (2.0 * dot.acos()).to_degrees();
                    if degrees >= MIN_STEP_DEGREES {
                        device.observed_degrees += degrees;
                        // Retain the anchor below the deadband so slow motion
                        // accumulates instead of disappearing at high UI rates.
                        device.activity_anchor = Some(quaternion);
                    }
                    device.observations = device.observations.saturating_add(1);
                } else {
                    device.activity_anchor = Some(quaternion);
                }
            } else {
                // A stale/unobserved jump is not movement evidence. Use the next
                // valid real sample as a new baseline instead of bridging it.
                device.activity_anchor = None;
            }
        }
        if self.elapsed >= WINDOW_SECONDS {
            self.finish();
        }
    }

    fn finish(&mut self) {
        let ranked = self.activities();
        let result = if self.interrupted {
            Outcome::Interrupted
        } else if let Some(leader) = ranked.first() {
            if leader.observed_degrees < MIN_ACTIVITY_DEGREES
                || leader.new_observations < MIN_OBSERVATIONS
            {
                Outcome::NoMovement
            } else {
                let device = &self.devices[&leader.tracker_id];
                if !device.connected || device.lost_connection {
                    Outcome::CandidateDisconnected
                } else if ranked.get(1).is_some_and(|second| {
                    second.observed_degrees >= OTHER_MOVEMENT_DEGREES
                        || leader.observed_degrees < second.observed_degrees * 3.0 + 15.0
                }) {
                    Outcome::Ambiguous
                } else {
                    Outcome::Candidate {
                        tracker_id: leader.tracker_id.clone(),
                    }
                }
            }
        } else {
            Outcome::NoMovement
        };
        self.outcome = Some(result);
    }

    pub fn activities(&self) -> Vec<Activity> {
        let denominator = self.elapsed.max(0.05);
        let mut ranked: Vec<_> = self
            .devices
            .iter()
            .map(|(id, device)| Activity {
                tracker_id: id.clone(),
                observed_degrees: device.observed_degrees,
                observed_degrees_per_second: device.observed_degrees / denominator,
                new_observations: device.observations,
                connected: device.connected,
            })
            .collect();
        ranked.sort_by(|a, b| {
            b.observed_degrees
                .total_cmp(&a.observed_degrees)
                .then_with(|| a.tracker_id.cmp(&b.tracker_id))
        });
        ranked
    }

    pub fn progress(&self) -> f32 {
        (self.elapsed / WINDOW_SECONDS).clamp(0.0, 1.0) as f32
    }
    pub fn outcome(&self) -> Option<&Outcome> {
        self.outcome.as_ref()
    }
    pub fn candidate_id(&self) -> Option<&str> {
        match &self.outcome {
            Some(Outcome::Candidate { tracker_id }) => Some(tracker_id),
            _ => None,
        }
    }
    pub fn candidate_still_connected(&self, observations: &[Observation<'_>]) -> bool {
        let Some(id) = self.candidate_id() else {
            return false;
        };
        observations.iter().any(|o| {
            o.id == id && o.connected() && o.rotation_packets >= self.devices[id].last_counter
        })
    }
}

fn normalized(quaternion: [f32; 4]) -> Option<[f64; 4]> {
    if !quaternion.iter().all(|value| value.is_finite()) {
        return None;
    }
    let q = quaternion.map(f64::from);
    let norm = q.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm < 1e-8 {
        return None;
    }
    Some(q.map(|value| value / norm))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn q(angle: f64) -> [f32; 4] {
        [0., (angle / 2.).sin() as f32, 0., (angle / 2.).cos() as f32]
    }
    fn sample(id: &str, packets: u64, angle: f64) -> Observation<'_> {
        Observation {
            id,
            quaternion_xyzw: q(angle),
            rotation_packets: packets,
            rotation_age_seconds: 0.,
            device_packet_age_seconds: Some(0.),
        }
    }
    fn trial(moving_ids: &[&str]) -> Session {
        let baseline = [
            sample("a", 500, 0.),
            sample("b", 900, 0.),
            sample("c", 100, 0.),
        ];
        let mut session = Session::start(0., &baseline);
        for step in 1..=90 {
            let time = f64::from(step) / 30.;
            let observations: Vec<_> = baseline
                .iter()
                .map(|base| {
                    let moving = moving_ids.contains(&base.id);
                    sample(
                        base.id,
                        base.rotation_packets + if moving { step as u64 * 5 } else { 0 },
                        if moving {
                            (time * std::f64::consts::TAU / 3.).sin()
                        } else {
                            0.
                        },
                    )
                })
                .collect();
            session.observe(time, &observations);
        }
        session
    }
    #[test]
    fn clear_mover_identifies_stable_id_without_assigning_a_role() {
        let session = trial(&["b"]);
        assert_eq!(
            session.outcome(),
            Some(&Outcome::Candidate {
                tracker_id: "b".to_owned()
            })
        );
        assert!(session.activities()[0].observed_degrees > 180.);
        assert!(
            (session.activities()[0].observed_degrees_per_second * 3.
                - session.activities()[0].observed_degrees)
                .abs()
                < 1e-6
        );
    }
    #[test]
    fn no_movement_or_small_noise_never_selects_a_tracker() {
        assert_eq!(trial(&[]).outcome(), Some(&Outcome::NoMovement));
        let mut session = Session::start(0., &[sample("a", 5, 0.)]);
        for step in 1..=90 {
            session.observe(
                f64::from(step) / 30.,
                &[sample(
                    "a",
                    5 + step as u64,
                    if step % 2 == 0 { 0.001 } else { -0.001 },
                )],
            );
        }
        assert_eq!(session.outcome(), Some(&Outcome::NoMovement));
    }
    #[test]
    fn two_movers_and_all_movers_are_ambiguous() {
        assert_eq!(trial(&["a", "b"]).outcome(), Some(&Outcome::Ambiguous));
        assert_eq!(trial(&["a", "b", "c"]).outcome(), Some(&Outcome::Ambiguous));
    }
    #[test]
    fn quaternion_sign_flips_and_held_repetitions_are_not_motion() {
        let mut session = Session::start(0., &[sample("a", 1, 0.), sample("b", 10, 0.)]);
        for step in 1..=90 {
            let mut a = sample("a", 1 + step as u64, 0.);
            if step % 2 == 0 {
                a.quaternion_xyzw = a.quaternion_xyzw.map(|v| -v);
            }
            let mut held = sample("b", 10, f64::from(step));
            held.rotation_age_seconds = 30.;
            session.observe(f64::from(step) / 30., &[a, held]);
        }
        assert_eq!(session.outcome(), Some(&Outcome::NoMovement));
    }
    #[test]
    fn moving_candidate_dropout_rejects_assignment() {
        let mut session = Session::start(0., &[sample("a", 1, 0.)]);
        for step in 1..=90 {
            let mut observation = sample("a", 1 + step as u64, f64::from(step) * 0.05);
            if step >= 80 {
                observation.device_packet_age_seconds = Some(2.);
            }
            session.observe(f64::from(step) / 30., &[observation]);
        }
        assert_eq!(session.outcome(), Some(&Outcome::CandidateDisconnected));
    }
    #[test]
    fn confirmed_candidate_must_still_be_connected() {
        let session = trial(&["b"]);
        let mut candidate = sample("b", 2000, 0.);
        assert!(session.candidate_still_connected(&[candidate]));
        candidate.device_packet_age_seconds = Some(2.);
        assert!(!session.candidate_still_connected(&[candidate]));
    }
    #[test]
    fn reset_and_missed_observation_window_require_retry() {
        let mut session = Session::start(0., &[sample("a", 100, 0.)]);
        session.observe(0.1, &[sample("a", 1, 0.5)]);
        session.observe(3., &[sample("a", 99, 1.)]);
        assert_eq!(session.outcome(), Some(&Outcome::Interrupted));
        let mut session = Session::start(0., &[sample("a", 1, 0.)]);
        session.observe(3., &[sample("a", 500, 2.)]);
        assert_eq!(session.outcome(), Some(&Outcome::Interrupted));
    }
    #[test]
    fn stale_samples_that_arrive_in_later_snapshots_are_not_new_motion() {
        let mut session = Session::start(0., &[sample("a", 1, 0.)]);
        for step in 1..=90 {
            let mut observation = sample("a", 1 + step as u64, f64::from(step));
            observation.rotation_age_seconds = 5.;
            session.observe(f64::from(step) / 30., &[observation]);
        }
        assert_eq!(session.outcome(), Some(&Outcome::NoMovement));
    }

    #[test]
    fn slow_small_steps_accumulate_without_turning_stationary_noise_into_motion() {
        let mut session = Session::start(0., &[sample("a", 100, 0.), sample("b", 500, 0.)]);
        for step in 1..=180 {
            let time = f64::from(step) / 60.;
            let slow = sample(
                "a",
                100 + step as u64,
                f64::from(step) * 0.5_f64.to_radians(),
            );
            let noise = sample(
                "b",
                500 + step as u64,
                if step % 2 == 0 { 0.001 } else { -0.001 },
            );
            session.observe(time, &[slow, noise]);
        }
        assert_eq!(session.candidate_id(), Some("a"));
        let activity = session.activities();
        assert!(activity[0].observed_degrees > 88. && activity[0].observed_degrees < 91.);
        assert_eq!(activity[1].observed_degrees, 0.);
    }

    #[test]
    fn old_baseline_and_first_fresh_jump_do_not_count_as_captured_movement() {
        let mut old = sample("a", 100, 0.);
        old.rotation_age_seconds = 30.;
        let mut session = Session::start(0., &[old]);
        for step in 1..=90 {
            session.observe(f64::from(step) / 30., &[sample("a", 100 + step as u64, 2.)]);
        }
        assert_eq!(session.outcome(), Some(&Outcome::NoMovement));
        assert_eq!(session.activities()[0].observed_degrees, 0.);
    }

    #[test]
    fn packet_counter_magnitude_is_not_an_activity_weight() {
        let mut first = Session::start(0., &[sample("a", 100, 0.)]);
        let mut second = Session::start(0., &[sample("a", 10_000, 0.)]);
        for step in 1..=90 {
            let time = f64::from(step) / 30.;
            let angle = (time * std::f64::consts::TAU / 3.).sin();
            first.observe(time, &[sample("a", 100 + step as u64, angle)]);
            second.observe(time, &[sample("a", 10_000 + step as u64 * 20, angle)]);
        }
        assert_eq!(
            first.activities()[0].observed_degrees,
            second.activities()[0].observed_degrees
        );
        assert_eq!(
            first.activities()[0].observed_degrees_per_second,
            second.activities()[0].observed_degrees_per_second
        );
    }
}
