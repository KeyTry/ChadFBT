//! Conditional absolute-heading hypothesis from two explicitly asserted landmarks.
//!
//! This experiment never changes solver state. The caller asserts that the tracked
//! left/right points correspond to repeatable left/right body landmarks. Neither
//! stillness nor plausible separation proves contact. The positional error budget
//! must include controller-origin offsets, placement error, tracking error and
//! timing uncertainty; it is a bound supplied by the experimenter, not measured
//! accuracy. Head orientation is deliberately absent from this observation model.

use crate::AnchorPose;
use glam::DVec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ContactHeadingConfig {
    pub min_baseline_m: f64,
    pub max_baseline_m: f64,
    pub max_vertical_difference_m: f64,
    pub max_point_wander_m: f64,
    pub max_heading_spread_rad: f64,
    /// Worst-case error for EACH tracked landmark, including systematic placement.
    pub point_error_bound_m: f64,
    pub max_heading_error_bound_rad: f64,
    pub min_span_secs: f64,
    pub max_span_secs: f64,
    pub max_pair_skew_secs: f64,
    pub max_gap_secs: f64,
    pub max_latest_age_secs: f64,
    pub assertion_lifetime_secs: f64,
}

impl Default for ContactHeadingConfig {
    fn default() -> Self {
        Self {
            min_baseline_m: 0.20,
            max_baseline_m: 0.65,
            max_vertical_difference_m: 0.08,
            max_point_wander_m: 0.015,
            max_heading_spread_rad: 3.0_f64.to_radians(),
            point_error_bound_m: 0.02,
            max_heading_error_bound_rad: 8.0_f64.to_radians(),
            min_span_secs: 0.5,
            max_span_secs: 1.5,
            max_pair_skew_secs: 0.03,
            max_gap_secs: 0.12,
            max_latest_age_secs: 0.15,
            assertion_lifetime_secs: 3.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ContactHeadingSample {
    /// Must change whenever standing space is recentered or anchor source restarts.
    pub anchor_session: u64,
    pub left: AnchorPose,
    pub right: AnchorPose,
}

/// A deliberate request that begins a new, bounded contact experiment.
#[derive(Clone, Copy, Debug)]
pub struct ContactAssertion {
    pub requested_at_secs: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactHeadingRejection {
    InvalidConfiguration,
    MissingAssertion,
    ExpiredAssertion,
    InsufficientWindow,
    InvalidAnchor,
    DifferentAnchorSessions,
    InvalidTime,
    ReusedOrReorderedSample,
    UnsynchronizedPair,
    InterruptedWindow,
    StaleAnchor,
    ImplausibleLandmarkGeometry,
    MovingLandmarks,
    HeadingUncertaintyTooLarge,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContactHeadingHypothesis {
    /// Rotation about standing-space +Y. Zero corresponds to body forward -Z.
    pub heading_rad: f64,
    /// Geometric worst-case bound CONDITIONAL on valid contact/error assumptions.
    /// This is not a probability or independently established tracking accuracy.
    pub conditional_error_bound_rad: f64,
    pub horizontal_baseline_m: f64,
    pub observed_heading_spread_rad: f64,
    pub observed_point_wander_m: f64,
    pub sample_count: usize,
    pub span_secs: f64,
    pub latest_observation_at_secs: f64,
    pub anchor_session: u64,
}

/// Estimate a heading hypothesis from a fresh, stable, same-session contact window.
/// Repeated cached anchors cannot manufacture a stable interval. No inferred body
/// positions or head directions enter this calculation, avoiding circular evidence.
pub fn estimate_contact_heading(
    samples: &[ContactHeadingSample],
    assertion: Option<ContactAssertion>,
    now: f64,
    config: &ContactHeadingConfig,
) -> Result<ContactHeadingHypothesis, ContactHeadingRejection> {
    use ContactHeadingRejection as Reject;
    if !valid_config(config) {
        return Err(Reject::InvalidConfiguration);
    }
    let assertion = assertion.ok_or(Reject::MissingAssertion)?;
    if !now.is_finite()
        || !assertion.requested_at_secs.is_finite()
        || assertion.requested_at_secs < 0.0
        || now < assertion.requested_at_secs
    {
        return Err(Reject::InvalidTime);
    }
    if now - assertion.requested_at_secs > config.assertion_lifetime_secs {
        return Err(Reject::ExpiredAssertion);
    }
    if !(6..=512).contains(&samples.len()) {
        return Err(Reject::InsufficientWindow);
    }
    let session = samples[0].anchor_session;
    let mut left_sum = DVec3::ZERO;
    let mut right_sum = DVec3::ZERO;
    let mut previous: Option<(f64, f64)> = None;
    let mut shortest_baseline = f64::INFINITY;
    for sample in samples {
        if sample.anchor_session != session {
            return Err(Reject::DifferentAnchorSessions);
        }
        let left = point(&sample.left)?;
        let right = point(&sample.right)?;
        let lt = sample.left.received_at_secs;
        let rt = sample.right.received_at_secs;
        if !lt.is_finite()
            || !rt.is_finite()
            || lt < assertion.requested_at_secs
            || rt < assertion.requested_at_secs
            || lt > now
            || rt > now
        {
            return Err(Reject::InvalidTime);
        }
        if (lt - rt).abs() > config.max_pair_skew_secs {
            return Err(Reject::UnsynchronizedPair);
        }
        if let Some((pl, pr)) = previous {
            if lt <= pl || rt <= pr {
                return Err(Reject::ReusedOrReorderedSample);
            }
            if lt - pl > config.max_gap_secs || rt - pr > config.max_gap_secs {
                return Err(Reject::InterruptedWindow);
            }
        }
        let baseline = right - left;
        let horizontal = horizontal(baseline).length();
        if horizontal < config.min_baseline_m
            || horizontal > config.max_baseline_m
            || baseline.y.abs() > config.max_vertical_difference_m
        {
            return Err(Reject::ImplausibleLandmarkGeometry);
        }
        left_sum += left;
        right_sum += right;
        shortest_baseline = shortest_baseline.min(horizontal);
        previous = Some((lt, rt));
    }
    let first = &samples[0];
    let last = samples.last().expect("sample count checked");
    let span = (last.left.received_at_secs - first.left.received_at_secs)
        .min(last.right.received_at_secs - first.right.received_at_secs);
    let total_span = last.left.received_at_secs.max(last.right.received_at_secs)
        - first
            .left
            .received_at_secs
            .min(first.right.received_at_secs);
    if span < config.min_span_secs || total_span > config.max_span_secs {
        return Err(Reject::InsufficientWindow);
    }
    let latest = last.left.received_at_secs.min(last.right.received_at_secs);
    if now - latest > config.max_latest_age_secs {
        return Err(Reject::StaleAnchor);
    }
    let mean_left = left_sum / samples.len() as f64;
    let mean_right = right_sum / samples.len() as f64;
    let baseline = horizontal(mean_right - mean_left);
    let baseline_length = baseline.length();
    if baseline_length < config.min_baseline_m {
        return Err(Reject::ImplausibleLandmarkGeometry);
    }
    let heading = (-baseline.z).atan2(baseline.x);
    let mut wander: f64 = 0.0;
    let mut spread: f64 = 0.0;
    for sample in samples {
        let left = point(&sample.left)?;
        let right = point(&sample.right)?;
        wander = wander
            .max(left.distance(mean_left))
            .max(right.distance(mean_right));
        let b = right - left;
        spread = spread.max(wrap_angle((-b.z).atan2(b.x) - heading).abs());
    }
    if wander > config.max_point_wander_m || spread > config.max_heading_spread_rad {
        return Err(Reject::MovingLandmarks);
    }
    // Difference of two bounded point errors lies in a disk of radius 2*epsilon.
    // Its tangent from the origin yields asin(2*epsilon / baseline). Systematic
    // placement error does NOT shrink with sample count. Adding observed heading
    // spread covers direction change; the shortest observed baseline bounds the
    // worst conditioned frame, rather than relying on its better averaged length.
    let ratio = 2.0 * config.point_error_bound_m / shortest_baseline;
    if ratio >= 1.0 {
        return Err(Reject::HeadingUncertaintyTooLarge);
    }
    let bound = ratio.asin() + spread;
    if bound > config.max_heading_error_bound_rad {
        return Err(Reject::HeadingUncertaintyTooLarge);
    }
    Ok(ContactHeadingHypothesis {
        heading_rad: heading,
        conditional_error_bound_rad: bound,
        horizontal_baseline_m: baseline_length,
        observed_heading_spread_rad: spread,
        observed_point_wander_m: wander,
        sample_count: samples.len(),
        span_secs: span,
        latest_observation_at_secs: latest,
        anchor_session: session,
    })
}

fn point(anchor: &AnchorPose) -> Result<DVec3, ContactHeadingRejection> {
    let point = DVec3::from_array(anchor.position.map(f64::from));
    if !anchor.valid || !point.is_finite() {
        return Err(ContactHeadingRejection::InvalidAnchor);
    }
    Ok(point)
}

fn horizontal(p: DVec3) -> DVec3 {
    DVec3::new(p.x, 0.0, p.z)
}

fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn valid_config(c: &ContactHeadingConfig) -> bool {
    [
        c.min_baseline_m,
        c.max_baseline_m,
        c.max_vertical_difference_m,
        c.max_point_wander_m,
        c.max_heading_spread_rad,
        c.point_error_bound_m,
        c.max_heading_error_bound_rad,
        c.min_span_secs,
        c.max_span_secs,
        c.max_pair_skew_secs,
        c.max_gap_secs,
        c.max_latest_age_secs,
        c.assertion_lifetime_secs,
    ]
    .iter()
    .all(|v| v.is_finite() && *v > 0.0)
        && c.min_baseline_m < c.max_baseline_m
        && c.min_span_secs <= c.max_span_secs
        && c.max_span_secs <= c.assertion_lifetime_secs
        && c.max_heading_spread_rad < std::f64::consts::FRAC_PI_2
        && c.max_heading_error_bound_rad < std::f64::consts::FRAC_PI_2
}
