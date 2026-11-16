//! Shared chain: exact lever identity, analytic yaw sensitivity, legacy reproduction.

use glam::{Quat, Vec3};
use tracking_core::chain::*;
use tracking_core::synthetic::XorShift64;
use tracking_core::*;

fn random_unit(rng: &mut XorShift64) -> Quat {
    loop {
        let q = Quat::from_xyzw(
            rng.gaussian() as f32,
            rng.gaussian() as f32,
            rng.gaussian() as f32,
            rng.gaussian() as f32,
        );
        if q.length_squared() > 1.0e-3 {
            return q.normalize();
        }
    }
}

/// A rotation within `max_angle` of `base` (random axis).
fn near(rng: &mut XorShift64, base: Quat, max_angle: f32) -> Quat {
    let axis = Vec3::new(
        rng.gaussian() as f32,
        rng.gaussian() as f32,
        rng.gaussian() as f32,
    )
    .normalize_or(Vec3::Y);
    (Quat::from_axis_angle(axis, max_angle * rng.uniform() as f32) * base).normalize()
}

fn random_pose(rng: &mut XorShift64) -> ChainPose {
    ChainPose {
        head_position: Vec3::new(
            4.0 * rng.uniform() as f32 - 2.0,
            1.0 + rng.uniform() as f32,
            4.0 * rng.uniform() as f32 - 2.0,
        ),
        head_rotation: random_unit(rng),
        rotations: std::array::from_fn(|_| random_unit(rng)),
    }
}

#[test]
fn chain_identity_holds_for_random_poses() {
    let mut rng = XorShift64::new(11);
    let dims = ChainDimensions::default();
    for _ in 0..2000 {
        let pose = random_pose(&mut rng);
        let chain = solve_chain(&dims, &pose);
        assert!(chain.head_finite && chain.segment_finite.iter().all(|ok| *ok));
        for foot in 0..2 {
            let sum = pose.head_position
                + chain.head_lever
                + chain.levers[foot].iter().copied().sum::<Vec3>();
            let error = (sum - chain.ankles[foot]).length();
            assert!(error <= 1.0e-5, "foot {foot}: identity error {error}");
            // The other leg's segments never move this ankle.
            let other = 1 - foot;
            assert_eq!(chain.levers[foot][THIGH[other]], Vec3::ZERO);
            assert_eq!(chain.levers[foot][CALF[other]], Vec3::ZERO);
        }
        assert!(((chain.neck - pose.head_position) - chain.head_lever).length() <= 1.0e-6);
    }
}

#[test]
fn yaw_sensitivity_is_y_cross_lever_and_matches_small_rotation() {
    let v = Vec3::new(0.3, -0.7, 0.2);
    assert_eq!(yaw_sensitivity(v), Vec3::Y.cross(v));
    assert_eq!(yaw_sensitivity(v), Vec3::new(0.2, 0.0, -0.3));
    // Ry(δ)·v − v ≈ δ·(ŷ × v)
    let delta = 1.0e-3_f32;
    let finite = (Quat::from_rotation_y(delta) * v - v) / delta;
    assert!((finite - yaw_sensitivity(v)).length() < 1.0e-3);
}

/// Central finite difference of ankle `foot` w.r.t. a left world-yaw of segment `segment`.
fn finite_difference(
    dims: &ChainDimensions,
    pose: &ChainPose,
    foot: usize,
    segment: usize,
) -> Vec3 {
    let epsilon = 2.0e-3_f32;
    let mut plus = *pose;
    let mut minus = *pose;
    plus.rotations[segment] = Quat::from_rotation_y(epsilon) * pose.rotations[segment];
    minus.rotations[segment] = Quat::from_rotation_y(-epsilon) * pose.rotations[segment];
    (solve_chain(dims, &plus).ankles[foot] - solve_chain(dims, &minus).ankles[foot])
        / (2.0 * epsilon)
}

#[test]
fn finite_difference_yaw_perturbation_matches_analytic_sensitivity() {
    let mut rng = XorShift64::new(5);
    let dims = ChainDimensions::default();
    let mut worst_leg = 0.0_f32;
    let mut worst_spine_ratio = 0.0_f32;
    for _ in 0..500 {
        // Upright-ish bodies: every segment within 70° of standing, chest and hip within
        // 30° of each other (the slerp split is the first-order sensitivity around that).
        let hip = near(&mut rng, Quat::IDENTITY, 50.0_f32.to_radians());
        let chest = near(&mut rng, hip, 30.0_f32.to_radians());
        let mut rotations = [Quat::IDENTITY; 6];
        rotations[CHEST] = chest;
        rotations[HIP] = hip;
        for rotation in &mut rotations[2..] {
            *rotation = near(&mut rng, Quat::IDENTITY, 70.0_f32.to_radians());
        }
        let pose = ChainPose {
            head_position: Vec3::new(0.3, 1.6, -0.2),
            head_rotation: near(&mut rng, Quat::IDENTITY, 60.0_f32.to_radians()),
            rotations,
        };
        let chain = solve_chain(&dims, &pose);
        let relative = chest.angle_between(hip).max(1.0e-3);
        for foot in 0..2 {
            for segment in FOOT_CHAIN[foot] {
                let analytic = yaw_sensitivity(chain.levers[foot][segment]);
                let numeric = finite_difference(&dims, &pose, foot, segment);
                let error = (analytic - numeric).length();
                if segment == CHEST || segment == HIP {
                    // Error per radian of chest–hip relative rotation per meter of spine.
                    let ratio = (error - 2.0e-3).max(0.0) / (relative * dims.chest_to_hip);
                    worst_spine_ratio = worst_spine_ratio.max(ratio);
                } else {
                    worst_leg = worst_leg.max(error);
                }
            }
            // Segments outside the foot's chain have no effect at all.
            let other = 1 - foot;
            assert!(finite_difference(&dims, &pose, foot, THIGH[other]).length() < 1.0e-4);
        }
    }
    // Legs: exact rigid rotation, only finite-difference/f32 error remains.
    assert!(
        worst_leg < 2.0e-3,
        "leg sensitivity error {worst_leg} m/rad"
    );
    // Spine: the ½/½ split of the mid-spine slerp is exact only when chest and hip agree; its
    // error grows linearly with their relative rotation φ (≈ 0.12·φ·chest_to_hip measured, so
    // ≈ 2.5 cm/rad at φ = 30°). The filter treats this as model error.
    assert!(
        worst_spine_ratio < 0.15,
        "spine sensitivity error {worst_spine_ratio} per rad of φ per m"
    );
    println!(
        "worst sensitivity error: legs {worst_leg:.2e} m/rad, spine {worst_spine_ratio:.3}·φ·chest_to_hip"
    );
}

#[test]
fn spine_split_is_exact_when_chest_and_hip_agree() {
    let dims = ChainDimensions::default();
    let bow = Quat::from_rotation_x(-0.6);
    let pose = ChainPose {
        head_position: Vec3::new(0.0, 1.5, 0.0),
        head_rotation: Quat::IDENTITY,
        rotations: [
            bow,
            bow,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
        ],
    };
    let chain = solve_chain(&dims, &pose);
    let both = {
        let epsilon = 1.0e-3_f32;
        let mut plus = pose;
        let mut minus = pose;
        for segment in [CHEST, HIP] {
            plus.rotations[segment] = Quat::from_rotation_y(epsilon) * bow;
            minus.rotations[segment] = Quat::from_rotation_y(-epsilon) * bow;
        }
        (solve_chain(&dims, &plus).ankles[0] - solve_chain(&dims, &minus).ankles[0])
            / (2.0 * epsilon)
    };
    let analytic = yaw_sensitivity(chain.levers[0][CHEST] + chain.levers[0][HIP]);
    assert!((both - analytic).length() < 1.0e-3);
}

#[test]
fn eye_forward_offset_places_neck_behind_and_below_the_eyes() {
    let dims = ChainDimensions {
        head_to_neck: 0.15,
        eye_forward: 0.08,
        ..ChainDimensions::default()
    };
    let head = Vec3::new(0.0, 1.65, 0.0);
    let pose = ChainPose {
        head_position: head,
        head_rotation: Quat::IDENTITY,
        rotations: [Quat::IDENTITY; 6],
    };
    let chain = solve_chain(&dims, &pose);
    // +Z is backward in the head frame (−Z forward).
    assert!((chain.neck - (head + Vec3::new(0.0, -0.15, 0.08))).length() < 1.0e-6);
    // Turning the head 90° left swings the neck model to the right of the eye point.
    let turned = ChainPose {
        head_rotation: Quat::from_rotation_y(90.0_f32.to_radians()),
        ..pose
    };
    let chain = solve_chain(&dims, &turned);
    assert!((chain.neck - (head + Vec3::new(0.08, -0.15, 0.0))).length() < 1.0e-5);
}

#[test]
fn from_anthropometrics_copies_dimensions() {
    let body = Anthropometrics::default();
    let dims = ChainDimensions::from_anthropometrics(&body, 0.07);
    assert_eq!(dims.head_to_neck, body.head_to_neck_m);
    assert_eq!(dims.eye_forward, 0.07);
    assert_eq!(dims.neck_to_chest, body.neck_to_chest_m);
    assert_eq!(dims.chest_to_hip, body.chest_to_hip_m);
    assert_eq!(dims.hip_width, body.hip_width_m);
    assert_eq!(dims.thigh, body.thigh_length_m);
    assert_eq!(dims.calf, body.calf_length_m);
}

#[test]
fn lateral_heading_matches_yaw_and_rejects_vertical_axis() {
    for degrees in [-170.0_f32, -90.0, -10.0, 0.0, 35.0, 120.0] {
        let yaw = degrees.to_radians();
        let heading = lateral_heading(Quat::from_rotation_y(yaw)).unwrap();
        assert!((heading - yaw).abs() < 1.0e-5, "{degrees}");
        // Pitch about the lateral axis leaves the heading unchanged.
        let pitched = Quat::from_rotation_y(yaw) * Quat::from_rotation_x(1.2);
        assert!((lateral_heading(pitched).unwrap() - yaw).abs() < 1.0e-5);
    }
    // Lateral axis 60° from horizontal (a.x² + a.z² = 0.25 < 0.5): undefined.
    assert_eq!(
        lateral_heading(Quat::from_rotation_z(60.0_f32.to_radians())),
        None
    );
    assert!(lateral_heading(Quat::from_rotation_z(40.0_f32.to_radians())).is_some());
    assert_eq!(lateral_heading(Quat::NAN), None);
    assert_eq!(lateral_heading(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)), None);
}

#[test]
fn nonfinite_inputs_are_flagged_and_outputs_stay_finite() {
    let dims = ChainDimensions::default();
    let mut pose = ChainPose {
        head_position: Vec3::new(0.0, 1.6, 0.0),
        head_rotation: Quat::IDENTITY,
        rotations: [Quat::IDENTITY; 6],
    };
    pose.rotations[CALF[0]] = Quat::NAN;
    pose.rotations[THIGH[1]] = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
    let chain = solve_chain(&dims, &pose);
    assert!(!chain.segment_finite[CALF[0]] && !chain.segment_finite[THIGH[1]]);
    assert!(!chain.foot_finite(0) && !chain.foot_finite(1));
    assert!(chain.head_finite);

    let garbage = ChainPose {
        head_position: Vec3::new(f32::NAN, f32::INFINITY, 1.0e30),
        head_rotation: Quat::from_xyzw(f32::INFINITY, 0.0, 0.0, 1.0),
        rotations: [Quat::from_xyzw(1.0e30, 1.0e30, 0.0, 0.0); 6],
    };
    for d in [
        dims,
        ChainDimensions {
            thigh: f32::NAN,
            ..dims
        },
    ] {
        let chain = solve_chain(&d, &garbage);
        assert!(!chain.head_finite);
        let values = [chain.neck, chain.chest, chain.hip, chain.head_lever]
            .into_iter()
            .chain(chain.hip_joints)
            .chain(chain.knees)
            .chain(chain.ankles)
            .chain(chain.levers.iter().flatten().copied());
        for value in values {
            assert!(value.is_finite());
        }
        assert!(chain.spine_rotation.is_finite());
    }
}

// --- Legacy reproduction: with eye_forward = 0 the chain equals TrackingCore::build_frame.

fn assignments() -> Vec<RoleAssignment> {
    Role::CORE
        .iter()
        .map(|role| RoleAssignment {
            sensor_id: format!("sensor-{role:?}"),
            role: *role,
        })
        .collect()
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
        left_hand: None,
        right_hand: None,
    }
}

fn feed(core: &mut TrackingCore, time: f64, rotations: &[Quat; 6]) {
    for (role, rotation) in Role::CORE.iter().zip(rotations) {
        core.ingest(SensorSample {
            id: format!("sensor-{role:?}"),
            orientation_xyzw: rotation.to_array(),
            received_at_secs: time,
            acceleration_mps2: Some([0.0; 3]),
            provenance: SensorProvenance::Measured,
        })
        .unwrap();
    }
}

/// The chain must reproduce the core's own skeleton. The snapshot this was written against
/// had no eye offset in `build_frame`; later cores add one. The offset the core uses is read
/// back from its Neck joint (head frame +Z), and every other joint is then compared exactly.
#[test]
fn chain_reproduces_core_build_frame() {
    let head_position = Vec3::new(0.0, 1.65, 0.0);
    let mut core = TrackingCore::new(TrackingConfig::default()).unwrap();
    core.set_assignments(assignments()).unwrap();
    for step in 0..=100 {
        let time = step as f64 / 100.0;
        feed(&mut core, time, &[Quat::IDENTITY; 6]);
        core.solve(time, &anchors(time, head_position, Quat::IDENTITY));
    }
    core.calibrate_neutral(1.0, &anchors(1.0, head_position, Quat::IDENTITY), false)
        .unwrap();
    for role in Role::CORE {
        core.confirm_manual_mount(role).unwrap();
    }

    let mut rng = XorShift64::new(3);
    let body = TrackingConfig::default().anthropometrics;
    let mut core_eye_forward = None;
    for k in 0..20 {
        let time = 1.01 + 0.01 * k as f64;
        let rotations: [Quat; 6] =
            std::array::from_fn(|_| near(&mut rng, Quat::IDENTITY, 25.0_f32.to_radians()));
        let head_rotation = near(&mut rng, Quat::IDENTITY, 40.0_f32.to_radians());
        let head = head_position + Vec3::new(0.01 * k as f32, 0.0, -0.02 * k as f32);
        feed(&mut core, time, &rotations);
        let frame = core.solve(time, &anchors(time, head, head_rotation));
        let neck = frame
            .joints
            .iter()
            .find(|joint| joint.joint == JointRole::Neck)
            .map(|joint| Vec3::from_array(joint.position))
            .unwrap();
        let eye_forward = (head_rotation.conjugate() * (neck - head)).z;
        let eye_forward = *core_eye_forward.get_or_insert(eye_forward);
        assert!((0.0..0.2).contains(&eye_forward), "{eye_forward}");
        let dims = ChainDimensions::from_anthropometrics(&body, eye_forward);
        // Use the segment rotations the core actually used for its skeleton.
        let used: [Quat; 6] = std::array::from_fn(|i| {
            let segment = frame
                .segments
                .iter()
                .find(|segment| segment.role == Role::CORE[i])
                .unwrap();
            Quat::from_array(segment.orientation_xyzw)
        });
        let chain = solve_chain(
            &dims,
            &ChainPose {
                head_position: head,
                head_rotation,
                rotations: used,
            },
        );
        let joint = |role: JointRole| {
            Vec3::from_array(
                frame
                    .joints
                    .iter()
                    .find(|joint| joint.joint == role)
                    .unwrap()
                    .position,
            )
        };
        let pairs = [
            (JointRole::Neck, chain.neck),
            (JointRole::Chest, chain.chest),
            (JointRole::Hip, chain.hip),
            (JointRole::LeftHip, chain.hip_joints[0]),
            (JointRole::RightHip, chain.hip_joints[1]),
            (JointRole::LeftKnee, chain.knees[0]),
            (JointRole::RightKnee, chain.knees[1]),
            (JointRole::LeftAnkle, chain.ankles[0]),
            (JointRole::RightAnkle, chain.ankles[1]),
        ];
        for (role, position) in pairs {
            let error = (joint(role) - position).length();
            assert!(error < 1.0e-5, "{role:?} differs by {error} m");
        }
    }
}
