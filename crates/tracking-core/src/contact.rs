//! Per-foot stationary-point hypotheses.
//!
//! A "contact" here means only: *this ankle is hypothesised to be a stationary point in the
//! room*. It is not proof of floor support and is inferred from the IMU chain, so it inherits
//! every chain error. Hysteresis: `Swing → Settling → Planted`, see [`ContactConfig`].
//!
//! Time not advancing, time going backwards or a gap longer than `max_gap_secs` releases both
//! feet and restarts velocity estimation; dwell time is never manufactured across such an
//! anomaly.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ContactConfig {
    /// Exponential smoothing time constant of the finite-difference ankle velocity.
    pub velocity_tau_secs: f32,
    /// Smoothing time constant of the calf world-vertical yaw rate.
    pub yaw_rate_tau_secs: f32,
    pub enter_speed_mps: f32,
    pub enter_height_m: f32,
    pub enter_yaw_rate_dps: f32,
    /// Entry conditions must hold continuously for this long before `Planted`.
    pub settle_secs: f32,
    pub exit_speed_mps: f32,
    pub exit_height_m: f32,
    pub exit_yaw_rate_dps: f32,
    /// Leave `Planted` when the smoothed vertical ankle speed exceeds this. A world-yaw error
    /// cannot create vertical ankle motion, so this gate stays tight even while the filter
    /// has not converged (refinement beyond the spec: catches lift-off and heel rise early).
    /// Non-positive or nonfinite disables it.
    pub exit_vertical_speed_mps: f32,
    /// Leave `Planted` once the calf has turned this far about vertical since `Settling`
    /// began (a foot pivoting on its ball moves the ankle; refinement beyond the spec).
    /// Non-positive or nonfinite disables the budget.
    pub max_planted_yaw_degrees: f32,
    /// A longer gap between updates releases both feet.
    pub max_gap_secs: f32,
    /// Upper bound on the per-foot speed allowance set with
    /// [`ContactTracker::set_speed_allowance`], and separately on the torso allowance.
    pub max_speed_allowance_mps: f32,
    /// Torso-motion allowance gain `k₂`: while the torso swings at `ω` (rad/s, rate of its
    /// long axis) a planted ankle of the head-down chain can appear to move by about
    /// `k₂·ω·torso_length` purely from torso model error (a flexed spine is not a rigid
    /// rod; waist trackers ride on soft tissue). That much is added to the speed and
    /// vertical-speed gates, see [`ContactTracker::set_torso_motion`]. Zero disables.
    pub torso_allowance_gain: f32,
    /// Double-support cue: grant the torso allowance only while both feet are in contact
    /// (settling or planted) and the vector between the two ankles changes slower than
    /// `double_support_relative_speed_mps`. That vector depends only on the pelvis width
    /// and the leg rotations, not on the torso or the headset, so a constant one means the
    /// feet are not moving relative to each other. False grants it regardless.
    pub require_double_support: bool,
    pub double_support_relative_speed_mps: f32,
}

impl Default for ContactConfig {
    fn default() -> Self {
        Self {
            velocity_tau_secs: 0.040,
            yaw_rate_tau_secs: 0.040,
            enter_speed_mps: 0.20,
            enter_height_m: 0.06,
            enter_yaw_rate_dps: 40.0,
            settle_secs: 0.080,
            exit_speed_mps: 0.45,
            exit_height_m: 0.10,
            exit_yaw_rate_dps: 90.0,
            exit_vertical_speed_mps: 0.15,
            max_planted_yaw_degrees: 15.0,
            max_gap_secs: 0.10,
            max_speed_allowance_mps: 0.40,
            torso_allowance_gain: 0.5,
            require_double_support: true,
            double_support_relative_speed_mps: 0.30,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactPhase {
    #[default]
    Swing,
    Settling,
    Planted,
}

/// Per-foot contact diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootContact {
    pub phase: ContactPhase,
    /// Time at which the current phase was entered.
    pub since_secs: f64,
    pub speed_mps: f32,
    pub height_above_lower_m: f32,
    /// Smoothed world-vertical calf rotation rate, degrees per second.
    pub yaw_rate_dps: f32,
    /// Calf rotation about vertical accumulated since `Settling` began, degrees.
    pub planted_yaw_degrees: f32,
}

impl FootContact {
    pub fn planted(&self) -> bool {
        self.phase == ContactPhase::Planted
    }
}

impl Default for FootContact {
    fn default() -> Self {
        Self {
            phase: ContactPhase::Swing,
            since_secs: 0.0,
            speed_mps: 0.0,
            height_above_lower_m: 0.0,
            yaw_rate_dps: 0.0,
            planted_yaw_degrees: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct FootState {
    phase: ContactPhase,
    since_secs: f64,
    previous: Option<(Vec3, Quat)>,
    velocity: Option<Vec3>,
    yaw_rate: Option<f32>,
    height: f32,
    /// Accumulated vertical calf rotation (radians) since `Settling` began.
    turned: f32,
}

impl FootState {
    fn release(&mut self, at_secs: f64) {
        if self.phase != ContactPhase::Swing {
            self.phase = ContactPhase::Swing;
            self.since_secs = at_secs;
        }
    }

    fn forget_history(&mut self) {
        self.previous = None;
        self.velocity = None;
        self.yaw_rate = None;
    }

    fn report(&self) -> FootContact {
        FootContact {
            phase: self.phase,
            since_secs: self.since_secs,
            speed_mps: finite_or_zero(self.velocity.map_or(0.0, Vec3::length)),
            height_above_lower_m: finite_or_zero(self.height),
            yaw_rate_dps: finite_or_zero(self.yaw_rate.map_or(0.0, f32::to_degrees)),
            planted_yaw_degrees: finite_or_zero(self.turned.to_degrees()),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ContactTracker {
    config: ContactConfig,
    last_at_secs: Option<f64>,
    feet: [FootState; 2],
    speed_allowance: [f32; 2],
    /// Requested torso allowance (m/s), see [`Self::set_torso_motion`].
    torso_allowance: f32,
    /// Torso allowance granted in the last update (0 without the double-support cue).
    torso_granted: f32,
    double_support: bool,
}

impl ContactTracker {
    pub fn new(config: ContactConfig) -> Self {
        Self {
            config,
            last_at_secs: None,
            feet: [FootState::default(); 2],
            speed_allowance: [0.0; 2],
            torso_allowance: 0.0,
            torso_granted: 0.0,
            double_support: false,
        }
    }

    pub fn config(&self) -> &ContactConfig {
        &self.config
    }

    /// Forget all history and release both feet.
    pub fn reset(&mut self) {
        self.last_at_secs = None;
        self.feet = [FootState::default(); 2];
        self.speed_allowance = [0.0; 2];
        self.torso_allowance = 0.0;
        self.torso_granted = 0.0;
        self.double_support = false;
    }

    /// Torso motion for the next update: `swing_rate_rad_per_sec` is how fast the torso's
    /// long axis turns (not its spin about that axis), `torso_length_m` the neck-to-pelvis
    /// length. Requests `torso_allowance_gain · rate · length` (capped at
    /// `max_speed_allowance_mps`) on the speed and vertical-speed gates of both feet; it
    /// is granted only while the double-support cue holds (see
    /// [`ContactConfig::require_double_support`]). The height, yaw-rate and pivot gates
    /// are unaffected. Nonfinite or negative inputs request nothing.
    pub fn set_torso_motion(&mut self, swing_rate_rad_per_sec: f32, torso_length_m: f32) {
        let cap = if self.config.max_speed_allowance_mps.is_finite() {
            self.config.max_speed_allowance_mps.max(0.0)
        } else {
            0.0
        };
        let gain = self.config.torso_allowance_gain;
        let value = gain * swing_rate_rad_per_sec * torso_length_m;
        self.torso_allowance = if value.is_finite() && gain > 0.0 {
            value.clamp(0.0, cap)
        } else {
            0.0
        };
    }

    /// Torso allowance (m/s) granted in the last update.
    pub fn granted_torso_allowance(&self) -> f32 {
        self.torso_granted
    }

    /// The double-support cue held in the last update.
    pub fn double_support(&self) -> bool {
        self.double_support
    }

    /// Per-foot allowance (m/s) added to both the enter and the exit ankle speed
    /// thresholds until it is set again; zero by default.
    ///
    /// A world-yaw error `δ` makes a truly stationary ankle appear to move at roughly
    /// `sin δ` times the speed at which its chain levers change. While the heading filter is
    /// uncertain, the caller widens the gates by that amount each frame (see
    /// [`crate::stance_heading::contact_speed_allowance`]) so a large error can still be
    /// observed. Nonfinite or negative values count as zero; values are capped at
    /// `max_speed_allowance_mps`. The height, yaw-rate, pivot and vertical-speed gates are
    /// unaffected, because a yaw error cannot fake them.
    pub fn set_speed_allowance(&mut self, allowance_mps: [f32; 2]) {
        let cap = if self.config.max_speed_allowance_mps.is_finite() {
            self.config.max_speed_allowance_mps.max(0.0)
        } else {
            0.0
        };
        self.speed_allowance = allowance_mps.map(|value| {
            if value.is_finite() {
                value.clamp(0.0, cap)
            } else {
                0.0
            }
        });
    }

    /// The allowance currently applied per foot (m/s).
    pub fn speed_allowance(&self) -> [f32; 2] {
        self.speed_allowance
    }

    /// External release (filter outlier streak, foot-lock break, headset discontinuity).
    /// The foot must re-satisfy the full settling dwell before it can be planted again.
    pub fn force_release(&mut self, foot: usize, at_secs: f64) {
        if let Some(state) = self.feet.get_mut(foot) {
            let at = if at_secs.is_finite() {
                at_secs
            } else {
                state.since_secs
            };
            state.release(at);
        }
    }

    /// Current diagnostics without advancing time.
    pub fn contacts(&self) -> [FootContact; 2] {
        [self.feet[0].report(), self.feet[1].report()]
    }

    /// Advances the hypotheses with the chain's ankle positions and calf rotations.
    /// `usable[s]` must be false whenever any input of foot `s`'s chain is stale, held or
    /// invalid.
    pub fn update(
        &mut self,
        at_secs: f64,
        ankles: [Vec3; 2],
        calf_rotations: [Quat; 2],
        usable: [bool; 2],
    ) -> [FootContact; 2] {
        if !at_secs.is_finite() {
            let at = self.last_at_secs.unwrap_or(0.0);
            for foot in &mut self.feet {
                foot.release(at);
                foot.forget_history();
            }
            return self.contacts();
        }
        let dt = match self.last_at_secs {
            Some(previous) => {
                let dt = at_secs - previous;
                if dt <= 0.0 || dt > f64::from(self.config.max_gap_secs) {
                    // Time anomaly or gap: no velocity can be trusted, no dwell may accrue.
                    for foot in &mut self.feet {
                        foot.release(at_secs);
                        foot.forget_history();
                    }
                    if dt > 0.0 {
                        // A forward gap: restart history from this frame.
                        self.last_at_secs = Some(at_secs);
                        self.seed_history(ankles, calf_rotations, usable);
                    } else {
                        // Not advancing or backwards: restart the timeline from here.
                        self.last_at_secs = Some(at_secs);
                    }
                    return self.contacts();
                }
                Some(dt as f32)
            }
            None => None,
        };
        self.last_at_secs = Some(at_secs);

        let valid = [0, 1].map(|foot| {
            usable[foot] && ankles[foot].is_finite() && finite_rotation(calf_rotations[foot])
        });
        let lower = (0..2)
            .filter(|&foot| valid[foot])
            .map(|foot| ankles[foot].y)
            .fold(f32::INFINITY, f32::min);
        // Double-support cue from the previous update's phases and smoothed velocities:
        // both feet in contact and not moving relative to each other.
        let relative_speed = match (self.feet[0].velocity, self.feet[1].velocity) {
            (Some(left), Some(right)) => (right - left).length(),
            _ => f32::INFINITY,
        };
        self.double_support = valid[0]
            && valid[1]
            && self
                .feet
                .iter()
                .all(|foot| foot.phase != ContactPhase::Swing)
            && relative_speed < self.config.double_support_relative_speed_mps;
        self.torso_granted = if self.double_support || !self.config.require_double_support {
            self.torso_allowance
        } else {
            0.0
        };
        let speed_cap = if self.config.max_speed_allowance_mps.is_finite() {
            self.config.max_speed_allowance_mps.max(0.0)
        } else {
            0.0
        };

        for foot in 0..2 {
            let state = &mut self.feet[foot];
            if !valid[foot] {
                state.release(at_secs);
                state.forget_history();
                state.height = 0.0;
                continue;
            }
            let ankle = ankles[foot];
            let calf = calf_rotations[foot].normalize();
            state.height = ankle.y - lower;
            let (Some(dt), Some((previous_ankle, previous_calf))) = (dt, state.previous) else {
                state.previous = Some((ankle, calf));
                state.release(at_secs);
                continue;
            };
            state.previous = Some((ankle, calf));

            let raw_velocity = (ankle - previous_ankle) / dt;
            let velocity_alpha = smoothing_alpha(dt, self.config.velocity_tau_secs);
            let velocity = match state.velocity {
                Some(old) => old + (raw_velocity - old) * velocity_alpha,
                None => raw_velocity,
            };
            let turn = vertical_rotation(calf, previous_calf);
            let raw_yaw_rate = turn / dt;
            let yaw_alpha = smoothing_alpha(dt, self.config.yaw_rate_tau_secs);
            let yaw_rate = match state.yaw_rate {
                Some(old) => old + (raw_yaw_rate - old) * yaw_alpha,
                None => raw_yaw_rate,
            };
            if !velocity.is_finite() || !yaw_rate.is_finite() {
                state.release(at_secs);
                state.forget_history();
                continue;
            }
            state.velocity = Some(velocity);
            state.yaw_rate = Some(yaw_rate);

            let speed = velocity.length();
            let yaw_dps = yaw_rate.to_degrees().abs();
            let config = &self.config;
            let allowance = (self.speed_allowance[foot] + self.torso_granted).min(speed_cap);
            if state.phase == ContactPhase::Swing {
                state.turned = 0.0;
            } else {
                state.turned += turn;
            }
            let budget = config.max_planted_yaw_degrees;
            let pivoted = budget > 0.0 && state.turned.to_degrees().abs() > budget;
            let lifting = config.exit_vertical_speed_mps > 0.0
                && velocity.y.abs() > config.exit_vertical_speed_mps + self.torso_granted;
            let enter = speed < config.enter_speed_mps + allowance
                && state.height < config.enter_height_m
                && yaw_dps < config.enter_yaw_rate_dps;
            let exit = speed > config.exit_speed_mps + allowance
                || state.height > config.exit_height_m
                || yaw_dps > config.exit_yaw_rate_dps
                || pivoted
                || lifting;
            match state.phase {
                ContactPhase::Swing => {
                    if enter {
                        state.phase = ContactPhase::Settling;
                        state.since_secs = at_secs;
                        state.turned = 0.0;
                    }
                }
                ContactPhase::Settling => {
                    if !enter || pivoted || lifting {
                        state.phase = ContactPhase::Swing;
                        state.since_secs = at_secs;
                    } else if at_secs - state.since_secs >= f64::from(config.settle_secs) {
                        state.phase = ContactPhase::Planted;
                        state.since_secs = at_secs;
                    }
                }
                ContactPhase::Planted => {
                    if exit {
                        state.phase = ContactPhase::Swing;
                        state.since_secs = at_secs;
                    }
                }
            }
        }
        self.contacts()
    }

    fn seed_history(&mut self, ankles: [Vec3; 2], calf_rotations: [Quat; 2], usable: [bool; 2]) {
        for foot in 0..2 {
            if usable[foot] && ankles[foot].is_finite() && finite_rotation(calf_rotations[foot]) {
                self.feet[foot].previous = Some((ankles[foot], calf_rotations[foot].normalize()));
            }
        }
    }
}

fn finite_rotation(q: Quat) -> bool {
    q.is_finite() && q.length_squared() > 1.0e-12
}

fn finite_or_zero(value: f32) -> f32 {
    if value.is_finite() { value } else { 0.0 }
}

fn smoothing_alpha(dt: f32, tau: f32) -> f32 {
    if tau > 0.0 && tau.is_finite() {
        1.0 - (-dt / tau).exp()
    } else {
        1.0
    }
}

/// World-vertical component of the rotation vector `2·log(current·previous⁻¹)` (radians).
pub(crate) fn vertical_rotation(current: Quat, previous: Quat) -> f32 {
    let mut delta = current * previous.conjugate();
    if delta.w < 0.0 {
        delta = -delta;
    }
    let vector = Vec3::new(delta.x, delta.y, delta.z);
    let sine = vector.length();
    if sine < 1.0e-9 {
        return 2.0 * delta.y;
    }
    let angle = 2.0 * sine.atan2(delta.w);
    vector.y / sine * angle
}
