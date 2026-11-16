use glam::{Quat, Vec3};
use tracking_core::{
    AnchorPose,
    contact_heading::{
        ContactAssertion, ContactHeadingConfig, ContactHeadingRejection as Rejection,
        ContactHeadingSample, estimate_contact_heading,
    },
};

fn samples(yaw: f32, width: f32) -> Vec<ContactHeadingSample> {
    let turn = Quat::from_rotation_y(yaw);
    let center = Vec3::new(1.2, 0.95, -2.1);
    (0..12)
        .map(|i| {
            let anchor = |x| AnchorPose {
                position: (center + turn * Vec3::new(x, 0.0, 0.0)).to_array(),
                // Orientation is unrelated to this positional cue.
                orientation_xyzw: Quat::from_rotation_y(-1.2).to_array(),
                received_at_secs: i as f64 * 0.1,
                valid: true,
                linear_velocity: None,
                angular_velocity: None,
            };
            ContactHeadingSample {
                anchor_session: 8,
                left: anchor(-width * 0.5),
                right: anchor(width * 0.5),
            }
        })
        .collect()
}

fn assertion() -> Option<ContactAssertion> {
    Some(ContactAssertion {
        requested_at_secs: 0.0,
    })
}

fn angle_delta(a: f64, b: f64) -> f64 {
    (a - b + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[test]
fn positional_heading_recovers_turns_without_using_head_or_controller_orientation() {
    for yaw in [-3.1_f32, -1.4, 0.0, 0.8, 3.1] {
        let window = samples(yaw, 0.4);
        let estimate =
            estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap();
        assert!(angle_delta(estimate.heading_rad, yaw as f64).abs() < 1e-5);
        assert!((estimate.conditional_error_bound_rad.to_degrees() - 5.73917).abs() < 1e-3);
    }
}

#[test]
fn no_assertion_and_expired_assertion_cannot_create_a_body_measurement() {
    let window = samples(0.0, 0.4);
    assert_eq!(
        estimate_contact_heading(&window, None, 1.1, &Default::default()).unwrap_err(),
        Rejection::MissingAssertion
    );
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 4.0, &Default::default()).unwrap_err(),
        Rejection::ExpiredAssertion
    );
}

#[test]
fn cached_anchors_cannot_manufacture_stability() {
    let mut window = samples(0.0, 0.4);
    window[7].left.received_at_secs = window[6].left.received_at_secs;
    window[7].right.received_at_secs = window[6].right.received_at_secs;
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::ReusedOrReorderedSample
    );
}

#[test]
fn session_changes_gaps_skew_and_stale_input_reject_the_window() {
    let mut window = samples(0.0, 0.4);
    window[6].anchor_session += 1;
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::DifferentAnchorSessions
    );
    let mut window = samples(0.0, 0.4);
    window.remove(6);
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::InterruptedWindow
    );
    let mut window = samples(0.0, 0.4);
    window[6].right.received_at_secs += 0.04;
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::UnsynchronizedPair
    );
    assert_eq!(
        estimate_contact_heading(&samples(0.0, 0.4), assertion(), 1.3, &Default::default())
            .unwrap_err(),
        Rejection::StaleAnchor
    );
}

#[test]
fn short_baseline_and_wide_placement_error_budget_are_not_overconfident() {
    assert_eq!(
        estimate_contact_heading(&samples(0.0, 0.25), assertion(), 1.1, &Default::default())
            .unwrap_err(),
        Rejection::HeadingUncertaintyTooLarge
    );
    let config = ContactHeadingConfig {
        point_error_bound_m: 0.04,
        ..Default::default()
    };
    assert_eq!(
        estimate_contact_heading(&samples(0.0, 0.4), assertion(), 1.1, &config).unwrap_err(),
        Rejection::HeadingUncertaintyTooLarge
    );
}

#[test]
fn stable_systematic_placement_bias_does_not_average_away() {
    let mut window = samples(0.0, 0.4);
    for sample in &mut window {
        sample.left.position[2] += 0.02;
        sample.right.position[2] -= 0.02;
    }
    let result = estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap();
    assert!(result.heading_rad.to_degrees() > 5.7);
    assert!(result.heading_rad.abs() <= result.conditional_error_bound_rad + 1e-6);
    assert!(result.conditional_error_bound_rad.to_degrees() > 5.7);
}

#[test]
fn bounded_point_errors_stay_inside_the_conditional_heading_bound() {
    let config = ContactHeadingConfig {
        max_heading_error_bound_rad: 30_f64.to_radians(),
        ..Default::default()
    };
    for degrees in (0..360).step_by(7) {
        let angle = (degrees as f32).to_radians();
        let error = Vec3::new(angle.cos(), 0.0, angle.sin()) * 0.02;
        let mut window = samples(0.0, 0.4);
        for sample in &mut window {
            sample.left.position = (Vec3::from_array(sample.left.position) - error).to_array();
            sample.right.position = (Vec3::from_array(sample.right.position) + error).to_array();
        }
        let result = estimate_contact_heading(&window, assertion(), 1.1, &config).unwrap();
        assert!(result.heading_rad.abs() <= result.conditional_error_bound_rad + 1e-6);
    }
}

#[test]
fn movement_invalid_points_and_impossible_geometry_are_rejected() {
    let mut window = samples(0.0, 0.4);
    window[6].left.position[0] += 0.04;
    window[6].right.position[0] += 0.04;
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::MovingLandmarks
    );
    window[6].left.position[0] = f32::NAN;
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap_err(),
        Rejection::InvalidAnchor
    );
    assert_eq!(
        estimate_contact_heading(&samples(0.0, 0.1), assertion(), 1.1, &Default::default())
            .unwrap_err(),
        Rejection::ImplausibleLandmarkGeometry
    );
}

#[test]
fn reversed_labels_or_false_contacts_remain_an_explicit_unresolved_assumption() {
    let mut window = samples(0.0, 0.4);
    for sample in &mut window {
        std::mem::swap(&mut sample.left, &mut sample.right);
    }
    // Plausible, stable but mislabeled points pass the numerical gates and are
    // exactly backward. This counterexample forbids treating acceptance as proof
    // of contact or as an unconditional anatomical heading measurement.
    let result = estimate_contact_heading(&window, assertion(), 1.1, &Default::default()).unwrap();
    assert!((result.heading_rad.abs() - std::f64::consts::PI).abs() < 1e-6);
}

#[test]
fn nonfinite_time_and_configuration_are_rejected_without_nan_results() {
    let window = samples(0.0, 0.4);
    assert_eq!(
        estimate_contact_heading(&window, assertion(), f64::NAN, &Default::default()).unwrap_err(),
        Rejection::InvalidTime
    );
    let config = ContactHeadingConfig {
        point_error_bound_m: f64::NAN,
        ..Default::default()
    };
    assert_eq!(
        estimate_contact_heading(&window, assertion(), 1.1, &config).unwrap_err(),
        Rejection::InvalidConfiguration
    );
}
