# Tracking research and implementation boundary

Research checked 2026-11-14. Target hardware: six Wi-Fi LSM6DSV IMUs on chest, hip, thighs and calves; tracked Quest headset/controllers through Steam Link/SteamVR; optional feet and upper arms later. This document distinguishes published results, inspected upstream behavior, and hypotheses for ChadFBT. No hardware accuracy or superiority over SlimeVR has been established.

## What the hardware can observe

The LSM6DSV contains a three-axis accelerometer and gyroscope; its embedded SFLP can supply a game rotation quaternion, gravity vector and gyro-bias estimate. Those features do not create an external heading reference. Hardware identity does not establish which firmware/fusion path a particular tracker is running. [ST product documentation](https://www.st.com/en/mems-and-sensors/lsm6dsv.html), [ST AN5922, section 6.5](https://www.st.com/resource/en/application_note/DM00945663.pdf).

Gravity constrains inclination, while absolute yaw remains unobservable from a stationary accelerometer and gyroscope without another reference. The VQF authors specifically show that the bias component along the instantaneous vertical direction is not observable from accelerometer measurements during motion. Rest assumptions can estimate gyro bias; changing inclination can expose different bias components. Neither fact guarantees drift-free heading indefinitely. [VQF paper, appendix C](https://arxiv.org/pdf/2203.17024).

Implications for this layout, derived from these measurement properties:

- Headset position anchors global translation. It cannot directly measure pelvis position or every segment's heading, and head direction is not body direction.
- Thigh and calf orientations strongly constrain leg geometry once mounting and lengths are known. Feet without IMUs have unmeasured ankle articulation; floor contact or a plausible foot rotation is an inference.
- Head and controller poses do not uniquely determine elbow swivel. Upper-arm IMUs later constrain that ambiguity.
- A rotated strap can look like changed sensor heading or changed anatomy. Treat mounting changes separately from drift corrections and surface suspected slip.
- A motion prior or joint constraint can select a plausible explanation, but cannot make an unobserved degree of freedom into a measurement.
- One first-time static pose cannot independently verify arbitrary mounting transforms and anatomical pose. The user must assert the neutral pose, or perform a sufficiently informative functional calibration.

## What SlimeVR already does

The latest stable release found is **v21.1.0**, published 2026-09-15. The release page also has **v22.0.0-rc.4**, dated November 12, with a server rewrite, improved IK and constraints. Earlier RCs introduce step mounting, revised spine tracking and hybrid smoothing/prediction. Claims that SlimeVR lacks these capabilities would be outdated. A fair comparison should identify and test both stable and the current RC. [Release history](https://github.com/SlimeVR/SlimeVR-Server/releases).

Source inspection below is pinned to `83941fd38e91cc91ca6b360deab5c2ae986dd1b6` on the main branch. It describes that source snapshot, not a completed audit of the separate v22 rewrite.

Stay Aligned is already a pose-aware yaw correction system. Its inspected implementation distinguishes moving, resting and recently resting trackers. Resting orientations act as references; moving trackers receive relaxed-pose centering and neighbor consistency terms. It chooses a small positive/negative yaw update by comparing weighted errors. Therefore, “add relaxed pose correction” is not an original differentiator. [AdjustTrackerYaw.kt](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/processor/stayaligned/adjust/AdjustTrackerYaw.kt).

The inspected rest detector uses angular range and dwell time; its comment explicitly notes acceleration as a possible future input. Defaults are a 2-degree range, 1-second entry and 3-second recently-resting period. LSM6DSV is assigned a maximum yaw correction rate of 0.15 degrees/second. These are implementation settings, not measured error guarantees. [RestDetector.kt](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/processor/stayaligned/trackers/RestDetector.kt), [StayAlignedDefaults.kt](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/processor/stayaligned/StayAlignedDefaults.kt).

The inspected reset code keeps several rotation offsets: full reset aligns sensor orientation to a reference and attachment pose; yaw reset changes heading; mounting reset infers a mounting adjustment from the tilted pose. The setup documentation explains the I-pose and ski-pose assumptions. Performing those resets in an incompatible pose can therefore corrupt calibration. Step mounting in the newer RC is relevant competing work. [TrackerResetsHandler.kt](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/trackers/TrackerResetsHandler.kt), [official setup guide](https://docs.slimevr.dev/quick-setup.html).

SlimeVR already detects multiple taps using acceleration changes and rejects taps when too many other trackers move. A tap detector is a command recognizer, not a body-pose measurement. [TapDetection.kt](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/processor/skeleton/TapDetection.kt).

## Relevant research, and transfer limits

| Work | Published contribution | Relevance and limitation here |
| --- | --- | --- |
| [DIP](https://dip.is.tue.mpg.de/about.html) | Learns full body pose from six IMUs using synthetic motion data and real DIP-IMU data. | Useful evidence for learned priors; its input placement/training distribution must be matched. A pretrained model does not directly accept arbitrary six-sensor placement. |
| [TransPose](https://xinyu-yi.github.io/TransPose/) | Progressive leaf-to-full joint estimation; combines supporting-foot and learned root-velocity estimates. Authors report over 90 fps. | Confidence-weighted hypotheses are useful. Its inertial translation estimator is less necessary when a live HMD supplies absolute position. Reported fps is not end-to-end SteamVR latency. |
| [PIP](https://xinyu-yi.github.io/PIP/) | Neural kinematics followed by physics optimization, estimating motion, forces and torques. Authors report 60 fps and 16 ms latency. | Physics can reduce implausible results, but model/contact assumptions and data requirements add complexity. Research timing is not a benchmark of this app or hardware. |
| [TIP](https://arxiv.org/html/2203.15720) | Transformer with prediction history, stationary body points and plausible terrain reconstruction. | Useful contact and temporal ideas. Authors explicitly cannot distinguish a stationary foot in the air from a foot resting on terrain using IMUs alone. |
| [HMD-Poser](https://arxiv.org/html/2403.03561) | HMD/controllers plus optional lower-leg and pelvis IMUs, temporal/spatial features and head-anchored FK. | More directly aligned to this product. Demonstrated sensor configurations and trained weights still differ from chest+hip+thighs+calves; retargeting/training and held-out testing are required. |
| [GlobalPose](https://xinyu-yi.github.io/GlobalPose/) | SIGGRAPH 2025 work on multiple contacts, physical optimization and global motion in 3D space. | A stronger modern comparison than stopping at 2022 methods; not evidence that a small native implementation automatically inherits those results. |
| [VQF](https://vqf.readthedocs.io/en/stable/) | Orientation fusion with gyro-bias estimation, 6D/9D modes and magnetic disturbance rejection; native C++ implementation, MIT license. | Strong raw-IMU baseline. Requires actual gyro/accel samples and timing. Fused quaternion packets alone cannot be rerun through VQF as if they were raw readings. |

The common DIP/TransPose/PIP/TIP placement is head, forearms, pelvis and lower legs, as summarized in HMD-Poser's related work. Chest and thigh IMUs provide valuable direct measurements, but change the observation model. IMU orientation conventions, acceleration including/excluding gravity, sensor offsets, sample rates, latencies and dropout behavior must match any training/evaluation pipeline. [HMD-Poser, sections 2.2–3.2](https://arxiv.org/html/2403.03561).

Relative heading from kinematic constraints is established prior art. Self-calibrating joint methods estimate axes and heading using motion windows; sparse double-hinge observability depends on motion and joint geometry. Human knees are not perfect mechanical hinges. Do not equate a small hinge residual with proof of anatomical correctness. [Self-calibrating 2-DoF joint paper](https://www.mdpi.com/1424-8220/22/24/9850), [double-hinge observability paper](https://arxiv.org/abs/2002.00902).

A recent October 2026 preprint investigates joint-center acceleration constraints and online sensor location estimation. Its experiments emphasize excitation and show that filtering can outperform simple fixed-threshold innovation rejection. This is a research lead, not an established production result or an implemented feature here. [Calibration-free relative orientation preprint](https://arxiv.org/html/2610.31156v1).

## Incremental solver plan

The following is an engineering proposal, not a published accuracy claim.

1. **Observable baseline:** normalize and validate sensor samples, map stable sensor IDs to explicit roles, retain timestamps, and solve fixed-length FK from the live HMD. Keep input conversion at the protocol boundary. Maintain fixed sensor-to-bone mounts and separate world-up yaw adjustments. Never substitute head yaw for body yaw every frame.
2. **Calibration with evidence:** retain a stable sample window; require fresh core sensors, valid HMD and plausible configured standing height. Average quaternion samples with hemisphere handling. Once mounting exists, reject large non-neutral segment tilts and incompatible headings. A first calibration still relies on the user's neutral-pose assertion. Persist assignments, mounts and anthropometrics atomically, with version checks.
3. **Conservative experimental relative correction:** use knee-axis consistency only during informative flexion, with an explicit neutral deadband for legitimate twist, good axis conditioning, fresh inputs, recent flexion excitation, bounded residuals and slow rate limits. Apply equal/opposite relative corrections so a global turn is not pulled toward a preferred heading. Default it off until live paired evaluation supports it. It cannot remove common-mode yaw drift.
4. **Contact-aware refinement later:** estimate contact probability from foot height/velocity and motion evidence; preserve competing contact hypotheses, use hysteresis, and release anchors before constraining a swing. Calf-only stationary orientation is insufficient evidence of a planted foot. Do not initially hard-pin feet or flatten foot rotation in every crouch/sit/lying pose.
5. **Raw-IMU extension:** when firmware supplies synchronized gyro/accelerometer/timestamps, compare VQF, temperature/rest bias models and joint-center acceleration factors against the quaternion baseline. Network receive time is not sensor sample time. Optimize clock offsets separately from body geometry.
6. **Learned residuals later:** train small residual/contact/confidence models for the exact sensor layout and dropout patterns. Preserve the deterministic measurements and confidence diagnostics; use a model to propose uncertain degrees of freedom, not silently overwrite trusted measurements.

Useful potential differentiators are transparent uncertainty, reversible calibration, stable sensor assignment, refusal to “correct” ambiguous poses, synchronized replay, measured latency, and per-user calibration quality. Whether these lead to better tracking is a hypothesis to test. The component algorithms themselves are prior art; no novelty claim is made.

## Implemented experiment status

*Updated 2026-11-15.* The tracking core implements the deterministic FK/calibration baseline, the legacy optional relative knee yaw constraint, and the stance-anchored estimator layers: a shared kinematic chain with a neck-pivot head model, stationary-foot contact hypotheses, a displacement-based per-sensor heading filter with deadbanded anatomical priors, contact-consistent leg IK, and time-based orientation prediction with velocity output. Design, equations and limits are in [stance-anchored-estimator.md](stance-anchored-estimator.md); the source-level comparison with SlimeVR is in [slimevr-internals.md](slimevr-internals.md); the wider literature is in [literature-review.md](literature-review.md). It still does not implement a research network, raw-IMU fusion or a general factor-graph optimizer. The estimator is an incremental combination of published ideas, verified by synthetic regressions only; it is not a claim of a new estimation principle or of demonstrated improvement over SlimeVR.

The [reproducible benchmark](../../crates/tracking-core/examples/benchmark.rs) measures solve-only CPU cost with deterministic six/ten-sensor streams at 100 Hz synthetic timestamps, separate warmup, and rotated trial order. Its paired error experiment includes an ideal hinge with injected calf drift and an identical observation stream interpreted as true calf twist. Reporting both reveals the prior's tradeoff: reduced relative disagreement can introduce errors into real anatomical motion. Common-mode drift and true global turns are evaluated separately. These synthetic experiments are implementation evidence only; actual hardware and comparative validation remain outstanding. Usage and exact measurement boundaries are in the [core README](../../crates/tracking-core/README.md).

## Safe double-tap recovery

A double tap should queue a recovery request. Recognition should require two separated local impulses, return below threshold between them, monotonic timing, a cooldown, and rejection of whole-body impacts. It must not call a full reset directly.

After the impulse settles, require fresh core sensors, continuous stable history, a valid live HMD, previously valid mounting, and a supported neutral standing pose with compatible body/head headings. Decline sitting, lying, motion, stale inputs, large unexplained disagreement or a suspected slipped strap. Explain the blocked reason and let the user retry with explicit forward-facing calibration. A compatible small yaw correction can update the separate yaw state, leaving mounts intact. Save the previous correction for undo.

The reset command explicitly asserts that the user is standing neutral and facing forward **with the torso**. Double-tap reset must be off by default and opt-in to that instruction. The sensors cannot distinguish a small stationary head-only turn from common-mode body yaw drift: an asserted reset in that mistaken pose can rotate the body heading by the bounded reset amount. The ordinary solver and continuous correction never use headset heading to rotate the body. An arbitrary-pose universal reset is not observable with this hardware and is not promised.

## Fair validation before claiming “better”

- Compare the same recordings, calibrated dimensions, firmware, placement and rendering time against SlimeVR stable and current RC. Record exact versions/configurations. Compare SlimeVR with Stay Aligned properly configured, and include ablations with correction disabled.
- Gather optical/reference tracking or synchronized multi-view reference for joint/target error. HMD-only self-consistency cannot establish true leg accuracy. Simulated drift injection verifies code behavior but is not hardware validation.
- Include walking, side/back steps, slow real turns, head-only turns, asymmetrical poses, seated leg crossing, kneeling, crouching, lying on either side, stationary feet held in the air, spins, kicks, tracker slip and Wi-Fi loss. Include different people, strap placements, sessions and thermal warm-up states.
- Report joint/target position and orientation error distributions, drift over long sessions, foot sliding during verified contact, false constraints during swing, invalid-output duration, reset/recovery frequency and failure rate, latency/CPU distributions, and calibration repeatability. Smoother output alone can conceal lag or motion suppression.
- Measure each pipeline stage and actual motion-to-output timing. Use synchronized clocks and consistent prediction horizons; frame rate alone is not latency.
- Separate training/tuning sessions from held-out evaluation. Publish failures and motion-class results, not only an aggregate average. Prefer paired comparisons and uncertainty intervals over an unqualified percentage improvement.

## Licensing and scope

Reading algorithms does not authorize copying implementation or distributing model assets under unrelated terms. TransPose and PIP repositories identify GPL-3.0 licenses; learned model/body assets and datasets have separate terms. VQF documents MIT licensing. The core in this repository is an independently written deterministic prototype; it does not ship research network weights, SMPL assets or copied SlimeVR implementation. [TransPose repository](https://github.com/Xinyu-Yi/TransPose), [PIP repository](https://github.com/Xinyu-Yi/PIP), [VQF documentation](https://vqf.readthedocs.io/en/stable/).
