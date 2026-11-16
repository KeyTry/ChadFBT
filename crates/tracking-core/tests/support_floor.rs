//! Synthetic regressions for the support-foot floor constraint with torso compliance,
//! the torso-scaled lock/root limits, the torso contact allowance and the legs-only stance
//! heading filter (findings from the first live session, 2026-11-15).
//!
//! The flexed-spine scenario reproduces the mechanism seen live, not its numbers: the
//! torso trackers over-read a forward bend (chest +12°, hip +30° at a 60° bow), so the
//! head-down chain is too short and planted feet float. Ground truth comes from the
//! estimator's own kinematic model (`synthetic`). Passing shows the implementation behaves
//! as designed under that injected error; it is not evidence of accuracy on a worn body.

use glam::{Quat, Vec3};
use tracking_core::forward_calibration::ski_pitch_sign;
use tracking_core::leg_ik::{
    LegIkConfig, SupportFloor, SupportFloorDiagnostics, SupportFloorInput, TorsoFlexion,
};
use tracking_core::synthetic::{FlexedSpine, Motion, Scenario, TruthFrame, XorShift64};
use tracking_core::*;

const HZ: f64 = 100.0;
const START: f64 = 2.2;
const CALIBRATION_HEADING: f32 = 0.4;

// ---------------------------------------------------------------------------
// Direct SupportFloor checks.

const NECK_TO_CHEST: f32 = 0.20;
const CHEST_TO_HIP: f32 = 0.38;
const SOLE: f32 = 0.08;

fn pitch(degrees: f32) -> Quat {
    // Positive degrees tip the bone's top forward (−Z).
    Quat::from_rotation_x(-degrees.to_radians())
}

fn floor_input(
    at: f64,
    ankle_y: [f32; 2],
    torso_degrees: f32,
    shank_degrees: f32,
) -> SupportFloorInput {
    SupportFloorInput {
        at_secs: at,
        hip: Vec3::new(0.0, 0.9, 0.0),
        ankles: [
            Vec3::new(-0.1, ankle_y[0], 0.0),
            Vec3::new(0.1, ankle_y[1], 0.0),
        ],
        thigh_rotations: [Quat::IDENTITY; 2],
        calf_rotations: [pitch(-shank_degrees); 2],
        chest_rotation: pitch(torso_degrees),
        spine_rotation: pitch(torso_degrees),
        usable: [true; 2],
        neck_to_chest_m: NECK_TO_CHEST,
        chest_to_hip_m: CHEST_TO_HIP,
        ankle_to_sole_m: SOLE,
    }
}

/// Runs `secs` of a constant input and returns the last report.
fn settle(
    floor: &mut SupportFloor,
    mut input: SupportFloorInput,
    from: f64,
    secs: f64,
) -> SupportFloorDiagnostics {
    let mut out = SupportFloorDiagnostics::default();
    for k in 0..=(secs * HZ) as usize {
        input.at_secs = from + k as f64 / HZ;
        out = floor.update(&input);
        assert!(out.vertical_m.is_finite() && out.budget_m.is_finite() && out.gap_m.is_finite());
    }
    out
}

#[test]
fn torso_flexion_terms_follow_the_chain_geometry() {
    let upright = TorsoFlexion::new(Quat::IDENTITY, Quat::IDENTITY, 0.2, 0.38);
    assert_eq!(upright, TorsoFlexion::default());
    let bow = TorsoFlexion::new(pitch(72.0), pitch(66.0), 0.2, 0.38);
    let expected_v =
        0.2 * (1.0 - 72f32.to_radians().cos()) + 0.38 * (1.0 - 66f32.to_radians().cos());
    let expected_h = 0.2 * 72f32.to_radians().sin() + 0.38 * 66f32.to_radians().sin();
    assert!((bow.vertical_m - expected_v).abs() < 1.0e-5);
    assert!((bow.horizontal_m - expected_h).abs() < 1.0e-5);
    // A forward (or sideways) bow keeps both terms in full as "bow" terms.
    assert!((bow.bow_vertical_m - expected_v).abs() < 1.0e-5);
    assert!((bow.bow_horizontal_m - expected_h).abs() < 1.0e-5);
    let side = TorsoFlexion::new(
        Quat::from_rotation_z(72f32.to_radians()),
        Quat::from_rotation_z(-66f32.to_radians()),
        0.2,
        0.38,
    );
    assert!((side.bow_vertical_m - expected_v).abs() < 1.0e-5);
    assert!((side.bow_horizontal_m - expected_h).abs() < 1.0e-5);
    // The same tilts backward: the geometry is unchanged, the bow terms vanish.
    let recline = TorsoFlexion::new(pitch(-72.0), pitch(-66.0), 0.2, 0.38);
    assert!((recline.vertical_m - expected_v).abs() < 1.0e-5);
    assert!((recline.horizontal_m - expected_h).abs() < 1.0e-5);
    assert!(recline.bow_vertical_m.abs() < 1.0e-6 && recline.bow_horizontal_m.abs() < 1.0e-6);
    // Per segment: chest reclined, spine bowed keeps the spine terms only.
    let mixed = TorsoFlexion::new(pitch(-72.0), pitch(66.0), 0.2, 0.38);
    assert!((mixed.bow_vertical_m - 0.38 * (1.0 - 66f32.to_radians().cos())).abs() < 1.0e-5);
    assert!((mixed.bow_horizontal_m - 0.38 * 66f32.to_radians().sin()).abs() < 1.0e-5);
    // Halfway between backward and sideways the fade is linear in the upward
    // component of the forward axis: 1 − 1/sqrt(2) of each term is left.
    let diagonal = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 1.0).normalize(), 50f32.to_radians());
    let partial = TorsoFlexion::new(diagonal, diagonal, 0.2, 0.38);
    let left = 1.0 - std::f32::consts::FRAC_1_SQRT_2;
    assert!((partial.bow_horizontal_m - 0.58 * 50f32.to_radians().sin() * left).abs() < 1.0e-5);
    assert!(
        (partial.bow_vertical_m - 0.58 * (1.0 - 50f32.to_radians().cos()) * left).abs() < 1.0e-5
    );
    let broken = TorsoFlexion::new(
        Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
        Quat::IDENTITY,
        0.2,
        0.38,
    );
    assert!(broken.vertical_m.is_finite() && broken.horizontal_m.is_finite());
    assert!(broken.bow_vertical_m.is_finite() && broken.bow_horizontal_m.is_finite());
}

#[test]
fn upright_torso_gets_only_the_base_budget_so_a_lifted_body_still_lifts() {
    let mut floor = SupportFloor::new(LegIkConfig::default());
    // Both feet 20 cm up (a jump) with an upright torso.
    let out = settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.20; 2], 0.0, 0.0),
        0.0,
        1.0,
    );
    assert!((out.budget_m - 0.02).abs() < 1.0e-6, "{out:?}");
    // A 20 cm gap is far beyond what a 2 cm budget can explain: no pull at all.
    assert_eq!(out.vertical_m, 0.0, "{out:?}");
    assert!(out.limited && !out.raised);
    // A gap the budget does explain is still closed in full.
    let mut near = SupportFloor::new(LegIkConfig::default());
    let out = settle(
        &mut near,
        floor_input(0.0, [SOLE + 0.015; 2], 0.0, 0.0),
        0.0,
        1.0,
    );
    assert!((out.vertical_m + 0.015).abs() < 1.0e-4, "{out:?}");
    // A knee lift: the support foot is on the floor, the lifted one is not pulled down.
    let out = settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.30, SOLE + 0.003], 5.0, 0.0),
        1.1,
        1.0,
    );
    assert_eq!(out.support_foot, Some(1));
    assert!(out.vertical_m <= 0.0 && out.vertical_m > -0.0035, "{out:?}");
}

#[test]
fn flexed_torso_pulls_the_support_foot_to_the_floor_with_a_low_pass() {
    let mut floor = SupportFloor::new(LegIkConfig::default());
    let input = floor_input(0.0, [SOLE + 0.15, SOLE + 0.17], 70.0, 5.0);
    let first = floor.update(&input);
    assert_eq!(first.vertical_m, 0.0, "the first frame has no time step");
    let second = floor.update(&SupportFloorInput {
        at_secs: 0.01,
        ..input
    });
    // τ = 60 ms: about 15 % of the 15 cm target after 10 ms.
    let expected = -0.15 * (1.0 - (-0.01_f32 / 0.06).exp());
    assert!((second.vertical_m - expected).abs() < 1.0e-4, "{second:?}");
    let out = settle(&mut floor, input, 0.02, 1.0);
    assert_eq!(out.support_foot, Some(0));
    assert!((out.vertical_m + 0.15).abs() < 1.0e-4, "{out:?}");
    assert!(out.budget_m > 0.3 && !out.limited, "{out:?}");
    assert_eq!(out.support_weight, 1.0);
}

#[test]
fn kneeling_sitting_and_lying_on_the_floor_are_left_alone() {
    let mut floor = SupportFloor::new(LegIkConfig::default());
    // Kneeling: shank horizontal behind the knee, ankle 6 cm above its floor height.
    let out = settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.06; 2], 10.0, 90.0),
        0.0,
        1.0,
    );
    assert_eq!(out.support_weight, 0.0);
    assert_eq!(out.vertical_m, 0.0, "{out:?}");
    // Long sitting / lying: torso horizontal (huge budget), legs horizontal.
    let out = settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.04; 2], 90.0, 85.0),
        1.1,
        1.0,
    );
    assert_eq!(out.vertical_m, 0.0, "{out:?}");
    // Lying with the knees up (shanks vertical): the pelvis is already on the floor, so
    // the hip-height guard leaves nothing to lower.
    let mut lying = floor_input(0.0, [SOLE + 0.10; 2], 90.0, 0.0);
    lying.hip.y = 0.12;
    let out = settle(&mut floor, lying, 2.2, 1.0);
    assert_eq!(out.vertical_m, 0.0, "{out:?}");
    assert!(out.limited);
    // Halfway (shank 50° from vertical): half weight.
    let out = settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.10; 2], 70.0, 50.0),
        3.3,
        1.0,
    );
    assert!((out.support_weight - 0.5).abs() < 1.0e-3, "{out:?}");
    assert!((out.vertical_m + 0.05).abs() < 1.0e-3, "{out:?}");
}

/// Torso lean (degrees forward) and support-foot gap for tests of the thigh
/// taper alone: the lean stays below `squat_bow_start_tilt_rad` (20°), where a
/// forward-bowed torso would hand a folded thigh the standing prior back, and
/// the gap stays inside the budget of that lean, 2 cm + 0.58 m·(1 − cos 15°) =
/// 4.0 cm.
const THIGH_TAPER_LEAN: f32 = 15.0;
const THIGH_TAPER_GAP: f32 = 0.03;

#[test]
fn floor_pull_fades_smoothly_for_folded_thighs_and_keeps_the_below_floor_guard() {
    for (degrees, expected_weight) in [
        (0.0, 1.0),
        (60.0, 1.0),
        (70.0, 0.5),
        (80.0, 0.0),
        (90.0, 0.0),
    ] {
        let mut floor = SupportFloor::new(LegIkConfig::default());
        let mut input = floor_input(0.0, [SOLE + THIGH_TAPER_GAP; 2], THIGH_TAPER_LEAN, 0.0);
        input.thigh_rotations = [pitch(degrees); 2];
        let out = settle(&mut floor, input, 0.0, 1.0);
        assert!(out.budget_m > THIGH_TAPER_GAP, "{out:?}");
        assert!(
            (out.support_weight - expected_weight).abs() < 0.001,
            "{degrees}: {out:?}"
        );
        assert!(
            (out.vertical_m + THIGH_TAPER_GAP * expected_weight).abs() < 0.0003,
            "{degrees}: {out:?}"
        );
    }
    let mut floor = SupportFloor::new(LegIkConfig::default());
    let mut input = floor_input(0.0, [SOLE - 0.08; 2], 0.0, 0.0);
    input.thigh_rotations = [pitch(90.0); 2];
    let out = floor.update(&input);
    assert!(out.raised && out.vertical_m > 0.06, "{out:?}");
    input.thigh_rotations = [Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0); 2];
    input.ankles[0].y = SOLE + 0.10;
    input.ankles[1].y = SOLE + 0.10;
    let mut floor = SupportFloor::new(LegIkConfig::default());
    let out = settle(&mut floor, input, 0.0, 1.0);
    assert_eq!(out.support_weight, 0.0);
    assert_eq!(out.vertical_m, 0.0);
}

#[test]
fn folded_support_releases_previous_pull_and_an_upright_leg_gets_it_back() {
    let mut floor = SupportFloor::new(LegIkConfig::default());
    let mut input = floor_input(0.0, [SOLE + 0.035; 2], THIGH_TAPER_LEAN, 0.0);
    let upright = settle(&mut floor, input, 0.0, 1.0);
    assert!((upright.vertical_m + 0.035).abs() < 1.0e-4, "{upright:?}");
    // Folding the thighs releases the pull although the gap is unchanged.
    input.thigh_rotations = [pitch(90.0); 2];
    let folded = settle(&mut floor, input, 1.01, 1.0);
    assert_eq!(folded.vertical_m, 0.0, "{folded:?}");
    assert!((folded.gap_m - 0.035).abs() < 1.0e-6 && folded.limited);
    // A smaller gap is not pulled closed either while the thighs are folded...
    input.ankles = input.ankles.map(|p| Vec3::new(p.x, SOLE + 0.02, p.z));
    let folded_near = settle(&mut floor, input, 2.02, 1.0);
    assert_eq!(folded_near.vertical_m, 0.0, "{folded_near:?}");
    // ...and the same gap is closed again once they are upright: the gate
    // follows the pose, it does not latch.
    input.thigh_rotations = [Quat::IDENTITY; 2];
    let restored = settle(&mut floor, input, 3.03, 1.0);
    assert!((restored.vertical_m + 0.02).abs() < 1.0e-4, "{restored:?}");
    assert_eq!(restored.support_weight, 1.0);
    // With the feet on the floor the stored pull decays instead of sticking.
    input.ankles = input.ankles.map(|p| Vec3::new(p.x, SOLE, p.z));
    let grounded = settle(&mut floor, input, 4.04, 1.0);
    assert!(grounded.vertical_m.abs() < 1.0e-5, "{grounded:?}");
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The documented pull-down for a support foot `gap` above the floor: the gap,
/// capped by the budget, scaled by the upright-leg weight and faded out between
/// a gap of one budget and of two budgets plus 5 cm.
fn documented_pull(gap: f32, budget: f32, weight: f32) -> f32 {
    gap.max(0.0).min(budget) * weight * (1.0 - smoothstep(budget, 2.0 * budget + 0.05, gap))
}

#[test]
fn a_backward_recline_earns_no_pull_down_budget_while_a_forward_or_sideways_tilt_does() {
    let base = LegIkConfig::default().support_floor_base_m;
    let torso = NECK_TO_CHEST + CHEST_TO_HIP;
    for degrees in [30.0_f32, 55.0, 90.0] {
        let slack = torso * (1.0 - degrees.to_radians().cos());
        let forward = pitch(degrees);
        let backward = pitch(-degrees);
        // Both sideways directions, and a forward lean seen from another heading.
        let sideways = [
            Quat::from_rotation_z(degrees.to_radians()),
            Quat::from_rotation_z(-degrees.to_radians()),
            Quat::from_rotation_y(1.1) * forward,
        ];
        let flexion = |q: Quat| TorsoFlexion::new(q, q, NECK_TO_CHEST, CHEST_TO_HIP);
        // The direction only matters for the pull-down budget: the height the
        // torso spans and its horizontal reach are the same either way.
        assert!((flexion(forward).vertical_m - slack).abs() < 1.0e-5);
        assert!((flexion(backward).vertical_m - slack).abs() < 1.0e-5);
        assert!((flexion(backward).horizontal_m - flexion(forward).horizontal_m).abs() < 1.0e-5);
        assert!((flexion(forward).bow_vertical_m - slack).abs() < 1.0e-5);
        assert!(flexion(backward).bow_vertical_m.abs() < 1.0e-6);
        // A recline seen from another heading is still a recline.
        let turned = flexion(Quat::from_rotation_y(-2.3) * backward);
        assert!(turned.bow_vertical_m.abs() < 1.0e-5, "{turned:?}");
        for q in sideways {
            assert!((flexion(q).bow_vertical_m - slack).abs() < 1.0e-5, "{q:?}");
        }
        // Halfway between backward and sideways: the forward axis points up by
        // sin(tilt)/sqrt(2), so 1 − 1/sqrt(2) of the term is left.
        let diagonal =
            Quat::from_axis_angle(Vec3::new(1.0, 0.0, 1.0).normalize(), degrees.to_radians());
        let partial = slack * (1.0 - std::f32::consts::FRAC_1_SQRT_2);
        assert!((flexion(diagonal).bow_vertical_m - partial).abs() < 1.0e-5);
        // Chest reclined, lower spine bowed: only the bowed segment counts.
        let mixed = TorsoFlexion::new(backward, forward, NECK_TO_CHEST, CHEST_TO_HIP);
        let spine_only = CHEST_TO_HIP * (1.0 - degrees.to_radians().cos());
        assert!((mixed.bow_vertical_m - spine_only).abs() < 1.0e-5);

        // Through SupportFloor, with upright legs and a gap of 95 % of what the
        // forward budget covers (9.3, 25.4 and 57 cm; the pelvis is put high
        // enough for its height guard to stay out of the way).
        let gap = 0.95 * (base + slack);
        let run = |rotation: Quat| {
            let mut input = floor_input(0.0, [SOLE + gap; 2], 0.0, 0.0);
            input.hip.y = 1.2;
            input.chest_rotation = rotation;
            input.spine_rotation = rotation;
            settle(
                &mut SupportFloor::new(LegIkConfig::default()),
                input,
                0.0,
                1.0,
            )
        };
        let bowed = run(forward);
        assert!(
            (bowed.budget_m - (base + slack)).abs() < 1.0e-5,
            "{bowed:?}"
        );
        assert!(
            (bowed.vertical_m + gap).abs() < 1.0e-4,
            "{degrees}: {bowed:?}"
        );
        for q in sideways {
            let leaned = run(q);
            assert!(
                (leaned.vertical_m + gap).abs() < 1.0e-4,
                "{degrees}: {leaned:?}"
            );
        }
        let reclined = run(backward);
        assert_eq!(reclined.support_weight, 1.0);
        assert!((reclined.budget_m - base).abs() < 1.0e-6, "{reclined:?}");
        // Every one of these gaps is beyond 2·2 cm + 5 cm: no pull at all,
        // where the same tilt forward lowers the body by the whole gap.
        assert!(gap > 2.0 * base + 0.05);
        assert_eq!(reclined.vertical_m, 0.0, "{degrees}: {reclined:?}");
        assert!(reclined.limited);
        // A gap the upright base budget explains is still closed when reclined.
        let mut near = floor_input(0.0, [SOLE + 0.015; 2], 0.0, 0.0);
        near.chest_rotation = backward;
        near.spine_rotation = backward;
        let near = settle(
            &mut SupportFloor::new(LegIkConfig::default()),
            near,
            0.0,
            1.0,
        );
        assert!((near.vertical_m + 0.015).abs() < 1.0e-4, "{near:?}");
    }
}

#[test]
fn the_pull_fades_out_for_a_gap_the_budget_cannot_explain() {
    // A 40° bow: budget 2 cm + 0.58 m·(1 − cos 40°) = 15.6 cm. The pull closes a
    // gap up to the budget, then fades to nothing at 2·15.6 + 5 = 36.1 cm.
    let budget = 0.02 + (NECK_TO_CHEST + CHEST_TO_HIP) * (1.0 - 40.0_f32.to_radians().cos());
    let end = 2.0 * budget + 0.05;
    let pull_for = |gap: f32| {
        let out = settle(
            &mut SupportFloor::new(LegIkConfig::default()),
            floor_input(0.0, [SOLE + gap; 2], 40.0, 0.0),
            0.0,
            1.0,
        );
        assert!((out.budget_m - budget).abs() < 1.0e-5, "{out:?}");
        assert!((out.gap_m - gap).abs() < 1.0e-5, "{out:?}");
        -out.vertical_m
    };
    let steps = 40;
    // Pull at the previous gap beyond the budget.
    let mut previous = f32::INFINITY;
    let mut largest = 0.0_f32;
    for k in 1..=steps {
        let gap = (end + 0.04) * k as f32 / steps as f32;
        let pull = pull_for(gap);
        assert!(
            (pull - documented_pull(gap, budget, 1.0)).abs() < 1.0e-4,
            "gap {gap}: pull {pull}"
        );
        largest = largest.max(pull);
        if gap <= budget {
            assert!((pull - gap).abs() < 1.0e-4, "gap {gap}: pull {pull}");
        } else {
            // Never more than the budget, never growing with the gap again.
            assert!(pull <= budget + 1.0e-5, "gap {gap}: pull {pull}");
            assert!(
                pull <= previous + 1.0e-5,
                "gap {gap}: {pull} after {previous}"
            );
            previous = pull;
        }
        if gap >= end {
            assert_eq!(pull, 0.0, "gap {gap}");
        }
    }
    assert!(
        largest > budget - 0.01,
        "the sweep reaches the budget: {largest}"
    );
    // The pull is continuous at the budget and half of it is left midway.
    assert!((pull_for(budget) - budget).abs() < 1.0e-4);
    assert!((pull_for(budget + 0.002) - budget).abs() < 1.0e-3);
    assert!((pull_for(0.5 * (budget + end)) - 0.5 * budget).abs() < 1.0e-4);
    assert_eq!(pull_for(end), 0.0);
}

#[test]
fn thigh_sweep_with_a_slight_forward_lean_documents_the_residual_pull_between_60_and_80_degrees() {
    // A 15° forward lean (too little for the squat rule) gives a 4.0 cm budget;
    // the support ankle reads 3 cm above the floor with a vertical shank. The
    // pull is 3 cm times the thigh weight 1 − smoothstep(60°, 80°, tilt): a
    // perched or half-reclined seat (thighs 60–75° from vertical) is still
    // pulled toward the floor.
    let gap = THIGH_TAPER_GAP;
    let run = |thigh: Quat| {
        let mut input = floor_input(0.0, [SOLE + gap; 2], THIGH_TAPER_LEAN, 0.0);
        input.thigh_rotations = [thigh; 2];
        settle(
            &mut SupportFloor::new(LegIkConfig::default()),
            input,
            0.0,
            1.0,
        )
    };
    let mut previous = f32::INFINITY;
    for (degrees, expected_pull) in [
        (55.0_f32, 0.03_f32),
        (60.0, 0.03),
        (65.0, 0.025_312_5),
        (70.0, 0.015),
        (75.0, 0.004_687_5),
        (80.0, 0.0),
        (85.0, 0.0),
        (90.0, 0.0),
        (130.0, 0.0),
        (175.0, 0.0),
    ] {
        let weight = 1.0 - smoothstep(60.0, 80.0, degrees);
        assert!((gap * weight - expected_pull).abs() < 1.0e-6);
        // The gate looks at the tilt only: knee forward, backward or sideways.
        for thigh in [
            pitch(degrees),
            pitch(-degrees),
            Quat::from_rotation_z(degrees.to_radians()),
        ] {
            let out = run(thigh);
            assert!(out.budget_m > gap, "{out:?}");
            assert!(
                (out.support_weight - weight).abs() < 1.0e-3,
                "{degrees}: {out:?}"
            );
            assert!(
                (out.vertical_m + expected_pull).abs() < 2.0e-4,
                "thigh {degrees}°: pulled {} m, documented {expected_pull}",
                -out.vertical_m
            );
        }
        let pull = -run(pitch(degrees)).vertical_m;
        assert!(
            pull <= previous + 1.0e-6,
            "{degrees}: {pull} after {previous}"
        );
        if degrees >= 80.0 {
            assert_eq!(pull, 0.0, "{degrees}");
        }
        previous = pull;
    }
}

#[test]
fn thigh_thresholds_at_or_above_pi_restore_the_calf_only_gate() {
    let pi = std::f32::consts::PI;
    for threshold in [pi, 4.0] {
        let ablated = LegIkConfig {
            support_thigh_full_tilt_rad: threshold,
            support_thigh_zero_tilt_rad: threshold,
            ..LegIkConfig::default()
        };
        // Sanitising keeps the ablation.
        assert_eq!(ablated.sanitized(), ablated);
        for thigh_degrees in [0.0_f32, 70.0, 90.0, 130.0, 170.0] {
            for (shank_degrees, calf_weight) in [(0.0_f32, 1.0_f32), (50.0, 0.5), (90.0, 0.0)] {
                let mut input = floor_input(
                    0.0,
                    [SOLE + THIGH_TAPER_GAP; 2],
                    THIGH_TAPER_LEAN,
                    shank_degrees,
                );
                input.thigh_rotations = [pitch(thigh_degrees); 2];
                let calf_only = settle(&mut SupportFloor::new(ablated), input, 0.0, 1.0);
                assert!(
                    (calf_only.support_weight - calf_weight).abs() < 1.0e-3,
                    "threshold {threshold}, thigh {thigh_degrees}: {calf_only:?}"
                );
                assert!(
                    (calf_only.vertical_m + THIGH_TAPER_GAP * calf_weight).abs() < 3.0e-4,
                    "threshold {threshold}, thigh {thigh_degrees}: {calf_only:?}"
                );
                // The default gate is what removes the pull for a folded thigh.
                let gated = settle(
                    &mut SupportFloor::new(LegIkConfig::default()),
                    input,
                    0.0,
                    1.0,
                );
                if thigh_degrees >= 80.0 {
                    assert_eq!(gated.vertical_m, 0.0, "thigh {thigh_degrees}: {gated:?}");
                } else if thigh_degrees <= 60.0 {
                    assert!((gated.vertical_m - calf_only.vertical_m).abs() < 1.0e-5);
                }
            }
        }
    }
}

#[test]
fn sanitized_repairs_bad_thigh_thresholds_and_knee_flex_slack() {
    let d = LegIkConfig::default();
    assert_eq!(d.sanitized(), d);
    assert!((d.support_thigh_full_tilt_rad - 60.0_f32.to_radians()).abs() < 1.0e-6);
    assert!((d.support_thigh_zero_tilt_rad - 80.0_f32.to_radians()).abs() < 1.0e-6);
    assert!(d.knee_flex_slack_rad > 0.0 && d.knee_flex_slack_rad < 5.0_f32.to_radians());
    let thresholds = |full: f32, zero: f32| {
        let c = LegIkConfig {
            support_thigh_full_tilt_rad: full,
            support_thigh_zero_tilt_rad: zero,
            ..d
        }
        .sanitized();
        // Nothing else changes, and the result is a fixed point.
        assert_eq!(
            LegIkConfig {
                support_thigh_full_tilt_rad: d.support_thigh_full_tilt_rad,
                support_thigh_zero_tilt_rad: d.support_thigh_zero_tilt_rad,
                ..c
            },
            d
        );
        assert_eq!(c.sanitized(), c);
        (c.support_thigh_full_tilt_rad, c.support_thigh_zero_tilt_rad)
    };
    let defaults = (d.support_thigh_full_tilt_rad, d.support_thigh_zero_tilt_rad);
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.2] {
        assert_eq!(
            thresholds(bad, d.support_thigh_zero_tilt_rad),
            defaults,
            "{bad}"
        );
        assert_eq!(
            thresholds(d.support_thigh_full_tilt_rad, bad),
            defaults,
            "{bad}"
        );
        assert_eq!(thresholds(bad, bad), defaults, "{bad}");
        // A valid partner that the repaired value would invert: both reset.
        assert_eq!(thresholds(bad, 0.5), defaults, "{bad}");
        assert_eq!(thresholds(1.5, bad), defaults, "{bad}");
    }
    // Inverted (zero weight before full weight): both reset.
    assert_eq!(thresholds(1.4, 0.5), defaults);
    // Valid values are kept, including a hard step and the ablation.
    assert_eq!(thresholds(0.5, 0.9), (0.5, 0.9));
    assert_eq!(thresholds(1.0, 1.0), (1.0, 1.0));
    assert_eq!(thresholds(4.0, 4.0), (4.0, 4.0));

    let slack = |value: f32| {
        let c = LegIkConfig {
            knee_flex_slack_rad: value,
            ..d
        }
        .sanitized();
        assert_eq!(
            LegIkConfig {
                knee_flex_slack_rad: d.knee_flex_slack_rad,
                ..c
            },
            d
        );
        c.knee_flex_slack_rad
    };
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.01] {
        assert_eq!(slack(bad), d.knee_flex_slack_rad, "{bad}");
    }
    for good in [0.0, 0.1, std::f32::consts::PI] {
        assert_eq!(slack(good), good);
    }
    // A repaired configuration behaves like the default one.
    let repaired = LegIkConfig {
        support_thigh_full_tilt_rad: f32::NAN,
        support_thigh_zero_tilt_rad: -1.0,
        knee_flex_slack_rad: f32::NAN,
        ..d
    };
    let mut input = floor_input(0.0, [SOLE + THIGH_TAPER_GAP; 2], THIGH_TAPER_LEAN, 0.0);
    input.thigh_rotations = [pitch(70.0); 2];
    let out = settle(&mut SupportFloor::new(repaired), input, 0.0, 1.0);
    assert!((out.support_weight - 0.5).abs() < 1.0e-3, "{out:?}");
    assert!(
        (out.vertical_m + 0.5 * THIGH_TAPER_GAP).abs() < 3.0e-4,
        "{out:?}"
    );
}

/// Folded thighs (90°) over upright shanks, 1.5 cm above the floor (inside the
/// 2 cm base budget, so the pull is the gap times the upright-leg weight).
fn folded_thigh_weight(config: LegIkConfig, chest: Quat, spine: Quat) -> SupportFloorDiagnostics {
    let mut input = floor_input(0.0, [SOLE + 0.015; 2], 0.0, 0.0);
    input.thigh_rotations = [pitch(90.0); 2];
    input.chest_rotation = chest;
    input.spine_rotation = spine;
    let out = settle(&mut SupportFloor::new(config), input, 0.0, 1.0);
    assert!(
        (out.vertical_m + 0.015 * out.support_weight).abs() < 1.0e-4,
        "{out:?}"
    );
    out
}

#[test]
fn a_forward_bowed_spine_gives_folded_thighs_the_standing_prior_back() {
    let d = LegIkConfig::default();
    assert!((d.squat_bow_start_tilt_rad - 20.0_f32.to_radians()).abs() < 1.0e-6);
    assert!((d.squat_bow_full_tilt_rad - 35.0_f32.to_radians()).abs() < 1.0e-6);
    // Forward lean of the mid-spine in 1.25° steps: no weight up to 20°, all of
    // it from 35°, smoothstep between. Its steepest slope is 1.5/15° = 0.1 per
    // degree, so no 1.25° step may change the weight by more than 0.125.
    let mut previous = 0.0_f32;
    for step in 0..=72 {
        let degrees = step as f32 * 1.25;
        let out = folded_thigh_weight(d, pitch(degrees), pitch(degrees));
        let expected = smoothstep(20.0, 35.0, degrees);
        assert!(
            (out.support_weight - expected).abs() < 2.0e-3,
            "lean {degrees}°: {out:?}, expected weight {expected}"
        );
        if degrees <= 20.0 {
            assert_eq!(out.vertical_m, 0.0, "lean {degrees}°: {out:?}");
        }
        if degrees >= 35.0 {
            assert!((out.vertical_m + 0.015).abs() < 1.0e-4, "lean {degrees}°");
        }
        assert!(out.support_weight >= previous - 1.0e-4, "lean {degrees}°");
        assert!(out.support_weight - previous < 0.13, "lean {degrees}°");
        previous = out.support_weight;
    }
    for degrees in [10.0_f32, 25.0, 40.0, 60.0, 90.0] {
        let radians = degrees.to_radians();
        // Reclined or leaning sideways by any amount: a seat, not a squat.
        for lean in [
            pitch(-degrees),
            Quat::from_rotation_z(radians),
            Quat::from_rotation_z(-radians),
            Quat::from_rotation_y(0.9) * pitch(-degrees),
        ] {
            let out = folded_thigh_weight(d, lean, lean);
            assert_eq!(out.support_weight, 0.0, "{degrees}°: {out:?}");
            assert_eq!(out.vertical_m, 0.0, "{degrees}°: {out:?}");
        }
        // The rule reads the mid-spine: a bowed chest over an upright spine
        // does not count, a bowed spine under an upright chest does.
        let chest_only = folded_thigh_weight(d, pitch(degrees), Quat::IDENTITY);
        assert_eq!(chest_only.support_weight, 0.0, "{degrees}°");
        let spine_only = folded_thigh_weight(d, Quat::IDENTITY, pitch(degrees));
        assert!((spine_only.support_weight - smoothstep(20.0, 35.0, degrees)).abs() < 2.0e-3);
        // Heading does not matter.
        let turned = Quat::from_rotation_y(-2.1) * pitch(degrees);
        let turned = folded_thigh_weight(d, turned, turned);
        assert!((turned.support_weight - smoothstep(20.0, 35.0, degrees)).abs() < 2.0e-3);
    }
    // Folded past horizontal (head below the pelvis, or trackers over-reading a
    // deep fold): the forward axis points less far down again, but this is
    // still a full forward lean, not a shallow one.
    for degrees in [90.0_f32, 100.0, 120.0, 145.0, 160.0, 175.0] {
        let out = folded_thigh_weight(d, pitch(degrees), pitch(degrees));
        assert_eq!(
            out.support_weight, 1.0,
            "folded {degrees}° forward: {out:?}"
        );
        let turned = Quat::from_rotation_y(2.4) * pitch(degrees);
        let out = folded_thigh_weight(d, turned, turned);
        assert_eq!(
            out.support_weight, 1.0,
            "folded {degrees}° forward: {out:?}"
        );
        // Bent backward or sideways past horizontal earns nothing.
        for lean in [
            pitch(-degrees),
            Quat::from_rotation_y(2.4) * pitch(-degrees),
            Quat::from_rotation_z(degrees.to_radians()),
            Quat::from_rotation_z(-degrees.to_radians()),
        ] {
            let out = folded_thigh_weight(d, lean, lean);
            assert_eq!(out.support_weight, 0.0, "{degrees}°: {out:?}");
        }
    }
    // The shank gate still applies on top: kneeling under a bowed torso.
    let mut kneeling = floor_input(0.0, [SOLE + 0.015; 2], 60.0, 90.0);
    kneeling.thigh_rotations = [pitch(90.0); 2];
    let out = settle(&mut SupportFloor::new(d), kneeling, 0.0, 1.0);
    assert_eq!(out.support_weight, 0.0, "{out:?}");
}

#[test]
fn squat_bow_thresholds_above_a_right_angle_restore_the_thigh_only_gate() {
    for threshold in [1.6_f32, 2.0, 4.0] {
        assert!(threshold > std::f32::consts::FRAC_PI_2);
        let ablated = LegIkConfig {
            squat_bow_start_tilt_rad: threshold,
            squat_bow_full_tilt_rad: threshold,
            ..LegIkConfig::default()
        };
        assert_eq!(ablated.sanitized(), ablated);
        for degrees in [0.0_f32, 30.0, 45.0, 70.0, 90.0] {
            let lean = pitch(degrees);
            // Folded thighs: nothing, however far the torso bows...
            let folded = folded_thigh_weight(ablated, lean, lean);
            assert_eq!(
                folded.support_weight, 0.0,
                "{threshold}, {degrees}°: {folded:?}"
            );
            assert_eq!(folded.vertical_m, 0.0, "{threshold}, {degrees}°");
            // ...where the default gives the prior back from 35°.
            let default = folded_thigh_weight(LegIkConfig::default(), lean, lean);
            if degrees >= 35.0 {
                assert_eq!(default.support_weight, 1.0, "{degrees}°: {default:?}");
            }
            // The thigh taper itself is untouched by the ablation.
            for (thigh, weight) in [(0.0_f32, 1.0_f32), (70.0, 0.5), (80.0, 0.0)] {
                let mut input = floor_input(0.0, [SOLE + 0.015; 2], degrees, 0.0);
                input.thigh_rotations = [pitch(thigh); 2];
                let out = settle(&mut SupportFloor::new(ablated), input, 0.0, 1.0);
                assert!(
                    (out.support_weight - weight).abs() < 1.0e-3,
                    "{threshold}, lean {degrees}°, thigh {thigh}°: {out:?}"
                );
            }
        }
    }
}

#[test]
fn sanitized_repairs_bad_squat_bow_thresholds() {
    let d = LegIkConfig::default();
    let thresholds = |start: f32, full: f32| {
        let c = LegIkConfig {
            squat_bow_start_tilt_rad: start,
            squat_bow_full_tilt_rad: full,
            ..d
        }
        .sanitized();
        // Nothing else changes, and the result is a fixed point.
        assert_eq!(
            LegIkConfig {
                squat_bow_start_tilt_rad: d.squat_bow_start_tilt_rad,
                squat_bow_full_tilt_rad: d.squat_bow_full_tilt_rad,
                ..c
            },
            d
        );
        assert_eq!(c.sanitized(), c);
        (c.squat_bow_start_tilt_rad, c.squat_bow_full_tilt_rad)
    };
    let defaults = (d.squat_bow_start_tilt_rad, d.squat_bow_full_tilt_rad);
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.2] {
        assert_eq!(
            thresholds(bad, d.squat_bow_full_tilt_rad),
            defaults,
            "{bad}"
        );
        assert_eq!(
            thresholds(d.squat_bow_start_tilt_rad, bad),
            defaults,
            "{bad}"
        );
        assert_eq!(thresholds(bad, bad), defaults, "{bad}");
        // A valid partner that the repaired value would invert: both reset.
        assert_eq!(thresholds(bad, 0.1), defaults, "{bad}");
        assert_eq!(thresholds(1.2, bad), defaults, "{bad}");
    }
    // Inverted (full weight before any): both reset.
    assert_eq!(thresholds(0.9, 0.3), defaults);
    // Valid values are kept, including a hard step and the ablation.
    assert_eq!(thresholds(0.2, 0.5), (0.2, 0.5));
    assert_eq!(thresholds(0.4, 0.4), (0.4, 0.4));
    assert_eq!(thresholds(2.0, 2.0), (2.0, 2.0));
    // A repaired configuration behaves like the default one: half weight at
    // 27.5°, the middle of 20–35°.
    let repaired = LegIkConfig {
        squat_bow_start_tilt_rad: f32::NAN,
        squat_bow_full_tilt_rad: -3.0,
        ..d
    };
    let out = folded_thigh_weight(repaired, pitch(27.5), pitch(27.5));
    assert!((out.support_weight - 0.5).abs() < 2.0e-3, "{out:?}");
}

#[test]
fn feet_below_the_floor_raise_the_body_at_once_and_switches_ablate() {
    let config = LegIkConfig::default();
    let mut floor = SupportFloor::new(config);
    let out = floor.update(&floor_input(0.0, [SOLE - 0.04, SOLE + 0.10], 0.0, 0.0));
    assert!(out.raised);
    assert!(
        (out.vertical_m - (0.04 - config.floor_margin_m)).abs() < 1.0e-6,
        "{out:?}"
    );

    let mut no_raise = SupportFloor::new(LegIkConfig {
        floor_clamp_enabled: false,
        ..config
    });
    let out = settle(
        &mut no_raise,
        floor_input(0.0, [SOLE - 0.04; 2], 0.0, 0.0),
        0.0,
        0.5,
    );
    assert_eq!(out.vertical_m, 0.0);
    let mut no_pull = SupportFloor::new(LegIkConfig {
        support_floor_enabled: false,
        ..config
    });
    let out = settle(
        &mut no_pull,
        floor_input(0.0, [SOLE + 0.15; 2], 70.0, 0.0),
        0.0,
        0.5,
    );
    assert_eq!(out.vertical_m, 0.0);
    let mut off = SupportFloor::new(LegIkConfig {
        support_floor_enabled: false,
        floor_clamp_enabled: false,
        ..config
    });
    let out = off.update(&floor_input(0.0, [SOLE - 0.04; 2], 70.0, 0.0));
    assert!(!out.active && out.vertical_m == 0.0);
}

#[test]
fn chain_height_normalisation_adjusts_chest_to_hip_only() {
    // A profile with the older default proportions: eye height 1.65 m, chain 1.67 m.
    let mut body = Anthropometrics {
        standing_eye_height_m: 1.65,
        head_to_neck_m: 0.15,
        neck_to_chest_m: 0.20,
        chest_to_hip_m: 0.40,
        hip_width_m: 0.30,
        shoulder_width_m: 0.39,
        thigh_length_m: 0.42,
        calf_length_m: 0.42,
        foot_length_m: 0.25,
        ankle_to_sole_m: 0.08,
        upper_arm_length_m: 0.30,
        forearm_length_m: 0.26,
        eye_forward_m: 0.08,
    };
    let before = body.clone();
    let delta = body.normalize_chain_height().unwrap();
    assert!((delta + 0.02).abs() < 1.0e-3, "{delta}");
    assert!((body.standing_chain_height_m() - body.standing_eye_height_m).abs() < 1.0e-5);
    assert_eq!(
        Anthropometrics {
            chest_to_hip_m: before.chest_to_hip_m,
            ..body.clone()
        },
        before
    );
    // Already consistent: unchanged, bit for bit.
    let again = body.clone();
    assert_eq!(body.normalize_chain_height().unwrap(), 0.0);
    assert_eq!(body, again);
    // Chest-to-hip (0.38 m) cannot absorb 45 cm (0.83 m > 0.8 m): left unchanged
    // and reported.
    let mut short = Anthropometrics::default();
    short.standing_eye_height_m += 0.45;
    let kept = short.clone();
    assert!(short.normalize_chain_height().is_err());
    assert_eq!(short, kept);
}

#[test]
fn nonfinite_or_unusable_input_never_produces_a_nonfinite_correction() {
    let mut floor = SupportFloor::new(LegIkConfig::default());
    settle(
        &mut floor,
        floor_input(0.0, [SOLE + 0.15; 2], 70.0, 0.0),
        0.0,
        1.0,
    );
    let mut rng = XorShift64::new(3);
    for k in 0..400 {
        let mut input = floor_input(1.1 + k as f64 * 0.01, [SOLE + 0.15; 2], 70.0, 0.0);
        let bad = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 1.0e30][k % 4];
        match (rng.next_u64() % 7) as u8 {
            0 => input.ankles[0].y = bad,
            1 => input.hip.y = bad,
            2 => input.calf_rotations[1] = Quat::from_xyzw(bad, 0.0, 0.0, 1.0),
            3 => input.chest_rotation = Quat::from_xyzw(0.0, bad, 0.0, 0.0),
            4 => input.ankle_to_sole_m = bad,
            5 => input.usable = [false; 2],
            _ => input.at_secs = f64::NAN,
        }
        let out = floor.update(&input);
        for v in [out.vertical_m, out.budget_m, out.gap_m, out.support_weight] {
            assert!(v.is_finite(), "{input:?} -> {out:?}");
        }
        // Wildly large inputs may not command a huge correction either.
        assert!(out.vertical_m.abs() <= 2.0, "{out:?}");
    }
}

// ---------------------------------------------------------------------------
// Pipeline (TrackingCore) scenarios. Helpers follow tests/integration_estimator.rs.

fn index(role: Role) -> usize {
    Role::ALL.iter().position(|r| *r == role).unwrap()
}
fn id(role: Role) -> String {
    format!("floor-{role:?}")
}
fn mount(role: Role) -> Quat {
    let i = index(role) as f32;
    Quat::from_rotation_x(0.3 + 0.23 * i)
        * Quat::from_rotation_z(-0.7 + 0.31 * i)
        * Quat::from_rotation_y(0.4 + 0.41 * i)
}
fn bias(role: Role) -> f32 {
    [-1.4, 2.2, 1.5707964, -2.7, 0.9, -0.4, 0.3, 0.7, -1.1, 2.8][index(role)]
}
fn ski_pitch(role: Role) -> f32 {
    ski_pitch_sign(role).unwrap_or(0.0)
        * [28.0_f32, 22.0, 38.0, 34.0, 18.0, 20.0, 0.0, 0.0, 0.0, 0.0][index(role)].to_radians()
}
fn sample(role: Role, time: f64, body: Quat) -> SensorSample {
    SensorSample {
        id: id(role),
        orientation_xyzw: (Quat::from_rotation_y(bias(role)) * body * mount(role)).to_array(),
        received_at_secs: time,
        acceleration_mps2: None,
        provenance: SensorProvenance::Measured,
    }
}
fn anchors(time: f64, position: Vec3, rotation: Quat) -> Anchors {
    Anchors {
        hmd: Some(AnchorPose {
            position: position.to_array(),
            orientation_xyzw: rotation.to_array(),
            received_at_secs: time,
            valid: true,
            linear_velocity: None,
            angular_velocity: None,
        }),
        ..Default::default()
    }
}

fn calibrated(config: TrackingConfig) -> TrackingCore {
    let mut core = TrackingCore::new(config).unwrap();
    core.set_assignments(
        Role::CORE
            .iter()
            .map(|role| RoleAssignment {
                sensor_id: id(*role),
                role: *role,
            })
            .collect(),
    )
    .unwrap();
    let heading = Quat::from_rotation_y(CALIBRATION_HEADING);
    let eye = core.config().anthropometrics.standing_eye_height_m;
    for tick in 0..=200 {
        let time = tick as f64 / 100.0;
        let ski = tick > 100;
        for role in Role::CORE {
            let bend = if ski { ski_pitch(role) } else { 0.0 };
            core.ingest(sample(role, time, heading * Quat::from_rotation_x(bend)))
                .unwrap();
        }
        let height = if ski { eye - 0.55 } else { eye };
        core.solve(time, &anchors(time, Vec3::new(0.0, height, 0.0), heading));
        if tick == 100 {
            core.calibrate_neutral(
                time,
                &anchors(time, Vec3::new(0.0, eye, 0.0), heading),
                false,
            )
            .unwrap();
        }
    }
    core.calibrate_forward(2.0).unwrap();
    assert!(core.direction_calibrated());
    core
}

/// Stance filter and leg IK on, every new default.
fn current() -> TrackingConfig {
    let mut config = TrackingConfig::default();
    config.estimator.enable_stance_heading = true;
    config.estimator.enable_leg_ik = true;
    config
}

/// The estimator before this change: no support floor, fixed lock/root limits, no torso
/// contact allowance, torso yaw estimated from displacement with the general ρ, no bounds.
fn previous() -> TrackingConfig {
    let mut config = current();
    let e = &mut config.estimator;
    e.leg_ik.support_floor_enabled = false;
    e.leg_ik.torso_slack_gain = 0.0;
    e.contact.torso_allowance_gain = 0.0;
    e.stance.estimate_torso_yaw = true;
    e.stance.torso_motion_noise_ratio = e.stance.motion_noise_ratio;
    e.stance.max_torso_excitation_ratio = 0.0;
    e.stance.max_leg_correction_degrees = 180.0;
    e.stance.max_torso_correction_degrees = 180.0;
    config
}

struct Step {
    t: f64,
    truth: TruthFrame,
    frame: TrackingFrame,
}

/// Drives a calibrated core through `scenario` (truth + optional flexed-spine over-read,
/// 1 mm headset noise) and returns every frame.
fn run(config: TrackingConfig, scenario: &Scenario) -> Vec<Step> {
    let mut core = calibrated(config);
    let generator = scenario.generator();
    let mut rng = XorShift64::new(11);
    let count = (generator.duration() * HZ).floor() as usize + 1;
    (0..count)
        .map(|k| {
            let t = k as f64 / HZ;
            let at = START + t;
            let truth = generator.frame(t);
            let worn = scenario
                .flexed_spine
                .map_or(truth.rotations, |spine| spine.measured(&truth.rotations));
            for (i, role) in Role::CORE.into_iter().enumerate() {
                core.ingest(sample(role, at, worn[i])).unwrap();
            }
            let noise = Vec3::new(
                rng.gaussian() as f32,
                rng.gaussian() as f32,
                rng.gaussian() as f32,
            ) * 0.001;
            let frame = core.solve(
                at,
                &anchors(at, truth.head_position + noise, truth.head_rotation),
            );
            Step { t, truth, frame }
        })
        .collect()
}

fn joint(frame: &TrackingFrame, role: JointRole) -> Vec3 {
    Vec3::from_array(
        frame
            .joints
            .iter()
            .find(|j| j.joint == role)
            .unwrap()
            .position,
    )
}
fn ankles(frame: &TrackingFrame) -> [Vec3; 2] {
    [
        joint(frame, JointRole::LeftAnkle),
        joint(frame, JointRole::RightAnkle),
    ]
}
fn assert_finite(frame: &TrackingFrame) {
    let finite = |v: &[f32]| v.iter().all(|x| x.is_finite());
    for t in &frame.targets {
        assert!(finite(&t.position) && finite(&t.orientation_xyzw) && finite(&t.linear_velocity));
    }
    for j in &frame.joints {
        assert!(finite(&j.position));
    }
    let e = &frame.estimator;
    let s = &e.support_floor;
    assert!(finite(&[
        s.vertical_m,
        s.budget_m,
        s.gap_m,
        s.support_weight
    ]));
    assert!(finite(&[
        e.contact_torso_allowance_mps,
        e.leg_ik.lock_break_m,
        e.leg_ik.root_max_m
    ]));
    assert!(finite(&e.stance.corrections));
}

/// Worst vertical distance of a published ankle from its floor height and worst
/// horizontal distance from the true ankle, over frames where the truth has both feet
/// planted (after the first 0.5 s).
fn planted_errors(steps: &[Step], sole: f32) -> (f32, f32) {
    let (mut vertical, mut horizontal) = (0.0_f32, 0.0_f32);
    for step in steps
        .iter()
        .filter(|s| s.t > 0.5 && s.truth.stationary == [true; 2])
    {
        assert!(step.frame.estimator.leg_ik_active, "t {}", step.t);
        for (foot, ankle) in ankles(&step.frame).into_iter().enumerate() {
            vertical = vertical.max((ankle.y - sole).abs());
            let truth = step.truth.ankles[foot];
            horizontal =
                horizontal.max(Vec3::new(ankle.x - truth.x, 0.0, ankle.z - truth.z).length());
        }
    }
    (vertical, horizontal)
}

fn chest_tilt_degrees(rotation: Quat) -> f32 {
    (rotation * Vec3::Y).y.clamp(-1.0, 1.0).acos().to_degrees()
}

#[test]
fn flexed_spine_bows_keep_planted_feet_on_the_floor() {
    let scenario = Scenario::new(
        "flexed-spine bows",
        vec![
            Motion::stand(2.0),
            Motion::torso_bows(3),
            Motion::stand(1.0),
        ],
    );
    let scenario = Scenario {
        flexed_spine: Some(FlexedSpine::LIVE_LIKE),
        ..scenario
    };
    let sole = Anthropometrics::default().ankle_to_sole_m;
    let with = run(current(), &scenario);
    let without = run(previous(), &scenario);
    let fk = run(
        {
            let mut config = current();
            config.estimator.enable_leg_ik = false;
            config
        },
        &scenario,
    );
    for step in &with {
        assert_finite(&step.frame);
    }
    if std::env::var_os("CHADFBT_TRACE").is_some() {
        for s in with.iter().step_by(5) {
            let e = &s.frame.estimator;
            let a = ankles(&s.frame);
            println!(
                "{:.2} ank {:+.3} {:+.3} sf {:+.3} gap {:+.3} lock {:?} root {:?} ph {:?}",
                s.t,
                a[0].y - sole,
                a[1].y - sole,
                e.support_floor.vertical_m,
                e.support_floor.gap_m,
                e.leg_ik.foot_locked,
                e.leg_ik.root_correction,
                e.contacts.map(|c| c.phase)
            );
        }
    }
    let (v_with, h_with) = planted_errors(&with, sole);
    let (v_without, h_without) = planted_errors(&without, sole);
    let fk_float = fk
        .iter()
        .map(|s| {
            ankles(&s.frame)
                .map(|a| a.y - sole)
                .into_iter()
                .fold(f32::INFINITY, f32::min)
        })
        .fold(0.0_f32, f32::max);
    let peak = with.iter().map(|s| s.frame.estimator.support_floor).fold(
        SupportFloorDiagnostics::default(),
        |m, d| {
            if d.vertical_m < m.vertical_m { d } else { m }
        },
    );
    let locked = with
        .iter()
        .filter(|s| s.t > 0.5)
        .filter(|s| s.frame.estimator.leg_ik.foot_locked == [true; 2])
        .count() as f32
        / with.iter().filter(|s| s.t > 0.5).count() as f32;
    let max_tilt = with
        .iter()
        .map(|s| {
            let seg = s
                .frame
                .segments
                .iter()
                .find(|g| g.role == Role::Chest)
                .unwrap();
            chest_tilt_degrees(Quat::from_array(seg.orientation_xyzw))
        })
        .fold(0.0_f32, f32::max);
    println!(
        "flexed-spine bows (published chest tilt up to {max_tilt:.0}°): head-down FK floats {:.1} cm; \
         with the fix ankles within {:.1} cm vertically / {:.1} cm horizontally (both feet locked {:.0} % of frames, \
         deepest pull-down {:.1} cm of a {:.1} cm budget); previous estimator {:.1} cm / {:.1} cm",
        fk_float * 100.0,
        v_with * 100.0,
        h_with * 100.0,
        locked * 100.0,
        -peak.vertical_m * 100.0,
        peak.budget_m * 100.0,
        v_without * 100.0,
        h_without * 100.0,
    );
    assert!(max_tilt > 70.0, "the over-read reaches the published chest");
    assert!(fk_float > 0.10, "{fk_float}");
    assert!(v_with < 0.02, "{v_with}");
    assert!(h_with < 0.03, "{h_with}");
    assert!(v_without > 0.10, "{v_without}");
    assert!(locked > 0.95, "{locked}");
}

#[test]
fn twenty_flexed_spine_bows_do_not_turn_the_torso_heading() {
    let scenario = tracking_core::synthetic::scenarios::flexed_spine_bows(20);
    let report = |config: TrackingConfig| {
        let steps = run(config, &scenario);
        let mut worst = [0.0_f32; 6];
        for step in &steps {
            for (i, c) in step.frame.estimator.stance.corrections.iter().enumerate() {
                worst[i] = worst[i].max(c.to_degrees().abs());
            }
        }
        worst
    };
    let now = report(current());
    let before = report(previous());
    println!(
        "20 flexed-spine bows, max |stance correction| (chest hip lt rt lc rc) degrees: \
         current {now:.2?}, previous estimator {before:.2?}"
    );
    assert!(now[0] < 2.0 && now[1] < 2.0, "{now:?}");
    assert!(now[2..].iter().all(|c| *c < 3.0), "{now:?}");
}

#[test]
fn an_upright_jump_still_leaves_the_floor() {
    let scenario = Scenario::new(
        "jumps",
        vec![Motion::stand(2.0), Motion::jumps(3), Motion::stand(1.0)],
    );
    let sole = Anthropometrics::default().ankle_to_sole_m;
    let steps = run(current(), &scenario);
    let truth_peak = steps
        .iter()
        .map(|s| s.truth.ankles[0].y.min(s.truth.ankles[1].y) - sole)
        .fold(0.0_f32, f32::max);
    let published_peak = steps
        .iter()
        .map(|s| {
            assert_finite(&s.frame);
            let [l, r] = ankles(&s.frame);
            l.y.min(r.y) - sole
        })
        .fold(0.0_f32, f32::max);
    println!(
        "upright jumps: true feet up {:.1} cm, published lowest ankle up {:.1} cm",
        truth_peak * 100.0,
        published_peak * 100.0
    );
    assert!(truth_peak > 0.19);
    assert!(
        published_peak > truth_peak - 0.03,
        "{published_peak} vs {truth_peak}"
    );
}

#[test]
fn a_knee_lift_lifts_and_the_support_foot_stays_down() {
    let scenario = Scenario::new(
        "knee lift",
        vec![Motion::stand(2.0), Motion::knee_lift(0), Motion::stand(1.0)],
    );
    let sole = Anthropometrics::default().ankle_to_sole_m;
    let steps = run(current(), &scenario);
    let mut lifted = 0.0_f32;
    let mut support = 0.0_f32;
    for step in steps.iter().filter(|s| s.t > 0.5) {
        assert_finite(&step.frame);
        let [l, r] = ankles(&step.frame);
        lifted = lifted.max(l.y - sole);
        support = support.max((r.y - sole).abs());
    }
    println!(
        "knee lift: lifted ankle up {:.1} cm (truth 30), support ankle within {:.1} cm of the floor",
        lifted * 100.0,
        support * 100.0
    );
    assert!(lifted > 0.27, "{lifted}");
    assert!(support < 0.02, "{support}");
}

#[test]
fn a_chain_longer_than_the_body_raises_the_pelvis_instead_of_bending_the_knees() {
    // The assumed legs are 3 cm longer each than the true ones: head-down FK puts the
    // standing feet 6 cm below the floor.
    let mut config = current();
    let a = &mut config.estimator;
    a.enable_stance_heading = false;
    let body = &mut config.anthropometrics;
    body.thigh_length_m += 0.03;
    body.calf_length_m += 0.03;
    body.standing_eye_height_m += 0.06;
    let scenario = Scenario::new("standing", vec![Motion::stand(3.0)]);
    let steps = run(config.clone(), &scenario);
    let sole = config.anthropometrics.ankle_to_sole_m;
    let last = &steps.last().unwrap().frame;
    let support = last.estimator.support_floor;
    assert!(support.raised && support.gap_m < -0.05, "{support:?}");
    for (hip, knee, ankle, thigh, calf) in [
        (
            JointRole::LeftHip,
            JointRole::LeftKnee,
            JointRole::LeftAnkle,
            Role::LeftThigh,
            Role::LeftCalf,
        ),
        (
            JointRole::RightHip,
            JointRole::RightKnee,
            JointRole::RightAnkle,
            Role::RightThigh,
            Role::RightCalf,
        ),
    ] {
        let (hip, knee, ankle) = (joint(last, hip), joint(last, knee), joint(last, ankle));
        assert!(
            (ankle.y - (sole - config.estimator.leg_ik.floor_margin_m)).abs() < 0.005,
            "{ankle:?}"
        );
        // The published leg keeps the measured thigh and calf directions: translated, not bent.
        let measured = |role: Role| {
            let q = Quat::from_array(
                last.segments
                    .iter()
                    .find(|s| s.role == role)
                    .unwrap()
                    .orientation_xyzw,
            );
            q * Vec3::Y
        };
        let thigh_axis = (hip - knee).normalize();
        let calf_axis = (knee - ankle).normalize();
        assert!(thigh_axis.angle_between(measured(thigh)) < 0.5_f32.to_radians());
        assert!(calf_axis.angle_between(measured(calf)) < 0.5_f32.to_radians());
    }
}

#[test]
fn flexed_spine_replays_are_deterministic() {
    let scenario = tracking_core::synthetic::scenarios::flexed_spine_bows(2);
    let a = run(current(), &scenario);
    let b = run(current(), &scenario);
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(
            serde_json::to_string(&x.frame).unwrap(),
            serde_json::to_string(&y.frame).unwrap()
        );
    }
}
