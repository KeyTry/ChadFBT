# Literature review: drift, contact, calibration and latency

Reviewed 2026-11-15 for this exact sensor set: firmware-fused quaternions (no magnetometer, no raw
gyro, no sensor timestamps) on chest, pelvis, thighs and calves, plus an absolute headset and
controllers. It extends [tracking-research.md](tracking-research.md) and feeds the
[stance-anchored estimator design](stance-anchored-estimator.md).

**Verification legend.** **[V]** record and claim checked against the primary page.
**[A]** only the abstract or catalogue record was checked. **[M]** the equation is written from
knowledge of the paper and was not re-checked against the full text; treat it as a standard
engineering form, not a quotation. Numbers quoted from abstracts are the authors' reported results
on their data and do not transfer to this hardware.

## A. Heading without a magnetometer

| Work | Model and what it observes | Use here |
| --- | --- | --- |
| Laidig, Schauer & Seel, ICORR 2017 [A] ([record](https://cris.fau.de/publications/253670726)) | Hinge joint: the yaw `δ` that aligns the horizontal projections of the hinge axis seen from both segments, `Rz(δ)·R₂j₂ ≈ R₁j₁` [M]. Relative heading only; needs a horizontal axis component. Reported RMS 25.8° → 2.6° under magnetic disturbance. | Knee prior in the stance filter. Blind to common-mode yaw. Fails for tibial rotation, vertical axes (lying, kneeling). |
| Laidig et al., EMBC 2019; Laidig, Weygers & Seel, Sensors 2022 [A] ([paper](https://www.mdpi.com/1424-8220/22/24/9850)) | 2-DoF joints, joint axes and heading estimated together over a window. | Hip is 3-DoF, so no direct transfer. |
| Lehmann, Laidig, Deimel & Seel, IFAC 2020 [V] ([arXiv 2002.00639](https://arxiv.org/abs/2002.00639)) | Range-of-motion violations counted over an 8 s window, updated each second; assumes drift < 0.5 °/s. Tested on a rigid test object. | Template for the chest–pelvis and pelvis–thigh priors, used as soft deadbanded terms. |
| Eckhoff, Kok, Lucia & Seel, 2020 [V] ([arXiv 2002.00902](https://arxiv.org/abs/2002.00902)) | Gyro-only double hinge; observability needs the middle segment's angular velocity neither parallel nor perpendicular to the constraint axis. | Excitation requirements are geometric, not "enough motion". |
| Weygers et al., IEEE Sensors J. 2020 [A]; Li, Osman, Seth & Kok, 2026 [V] ([arXiv 2610.31156](https://arxiv.org/html/2610.31156v1)); MinJointTracker, 2025 [V] ([arXiv 2509.12398](https://arxiv.org/html/2509.12398v1)) | Joint-centre acceleration equality across a joint. Needs synchronised raw specific force, angular rate and angular acceleration. | Not usable on the stock stream. Worth revisiting with the headset as the single heading root once raw data exists. |
| Kok, Hol & Schön 2014; Taetz, Bleser & Miezal, FUSION 2016 [V] ([arXiv 1606.03754](https://arxiv.org/abs/1606.03754)); Miezal et al., Sensors 2016 [A] | Sliding-window constrained least squares, online mounting. Mounting rotation error propagates linearly into segment orientation; length error matters little for orientation. | Estimate mounting before trusting positional residuals for yaw. |
| VQF, Laidig & Seel, Information Fusion 2023 [A] ([code](https://github.com/dlaidig/vqf)) | Rest and motion gyro-bias estimation; vertical-axis bias is unobservable from the accelerometer during motion. | Explains why yaw drift survives good firmware fusion. Needs raw samples to rerun. |

## B. Contact-aided estimation and the closest prior art to our observation

The stance filter's observation is: *while a foot is hypothesised stationary, the chain built from
IMU orientations must reproduce the absolute head displacement.*

- **Legged-robot state estimation.** Bloesch et al., RSS 2012 [A] and the contact-aided invariant
  EKF of Hartley et al., IJRR 2020 [A] ([arXiv 1904.09251](https://arxiv.org/abs/1904.09251)) keep
  contact points in the filter state. Hartley et al. state that absolute position and **yaw are
  unobservable from IMU plus contact alone**. An absolute position reference is what makes yaw
  observable, and only while that reference translates.
- **Absolute position plus a kinematic chain gives segment heading.**
  Xsens patent US10222450B2 (Bellusci, Hol & Slycke, 2015) [V, partial]: joint constraints, then
  rigid alignment to a measured position; one positional unit keeps the chain drift-free.
  Zihajehzadeh & Park, IEEE TSMC 2017 [A] (patent WO2017120669A1 [V]): UWB tags on pelvis and feet
  give thigh/shank yaw from the horizontal hip-to-ankle vector, assuming thigh and shank share one
  yaw. Lee et al., Sensors 2015 [A]: foot zero-velocity positions plus a waist IMU correct heading
  through a leg model. Lorenz et al., FUSION 2022 [A]: one heading source keeps the whole chain
  drift-free.
- **Lengths from leg odometry against an external reference.** Yang, Choset & Manchester, RA-L
  2022 [A] calibrate leg length online. SlimeVR AutoBone [V, docs] fits bone lengths offline from a
  recording by minimising foot sliding at fixed heels and headset height; it does not estimate yaw
  or mounting.
- **Contact in sparse-IMU capture.** TransPose (SIGGRAPH 2021) [A] fuses supporting-foot
  translation by contact probability. GlobalPose (SIGGRAPH 2025) [V]
  ([arXiv 2505.05010](https://arxiv.org/html/2505.05010v1)) uses stationary probability > 0.7,
  ≤ 0.05 m ground distance or previous contact, 5-frame persistence, and aligns per-IMU heading by
  step trajectories; its stated failures are sliding or light contacts, tilted surfaces and small
  steps. TIP (SIGGRAPH Asia 2022) concedes IMUs alone cannot separate a still airborne foot from a
  grounded one.
- **Slip rejection.** Camurri et al., RA-L 2017 [A]: slip appears as disagreement with consensus.
  Here the consensus is consistency with the headset, enforced by the innovation gate.
- **Rollover.** Hansen, Childress & Knox, Clin. Biomech. 2004 [A]: the contact point migrates
  from heel to forefoot during stance. A fixed ankle point is therefore an approximation; a
  shank-fixed rocker arc is the documented refinement and is not implemented yet.

**Novelty assessment.** Each ingredient is published. No publication was found that combines a
headset endpoint, a hypothesised (unmeasured) stationary foot and an increment-based per-segment
yaw regression on fused quaternions, but industry systems are undocumented. We treat the stance
filter as an **incremental combination of prior art** and make no novelty claim. A patent search
would be required before anyone does.

## C. Learned sparse-IMU methods

PIP, TIP, PNP (SIGGRAPH 2024, fictitious forces in a non-inertial root frame), DynaIP, HMD-Poser,
EgoPoser, AvatarPoser (IK refinement after the network), Ultra Inertial Poser (inter-sensor
distances) and MobilePoser were reviewed [A]. Transferable without training: contact hysteresis
rules, least-squares root velocity that keeps stationary joints fixed, and post-hoc IK to the
measured anchors. Training our own model is feasible on a consumer GPU from synthesised virtual
sensors, but the AMASS licence forbids commercial training, so a shippable model needs licensed
or self-captured data.

## D. Mounting and proportions

Seel, Raisch & Schauer, Sensors 2014 [A]: hinge-axis identification from angular rates,
`min Σ(‖ω₁×j₁‖ − ‖ω₂×j₂‖)²` [M], usable with rates differenced from quaternions. McGrath &
Stirling, Sensors 2020 [A]: knee hinge plus segment-length priors. Pacher et al., Sensors 2020
(review) [A]: no single best calibration; functional methods depend on how the user moves.

## E. IK, foot lock, filtering and timing

- Kovar, Schreiner & Gleicher, SCA 2002 [A]: footskate cleanup by filtering plant targets and
  letting the leg absorb the residual. Basis of `leg_ik`.
- Bollo, GDC 2018 [A]: inertialization. Decay a correction offset instead of cross-fading; used
  for lock release.
- Casiez, Roussel & Vogel, CHI 2012 [A]: the 1€ filter, cut-off rising with speed. Outputs only.
- LaValle et al., ICRA 2014 [A]: constant-rate quaternion prediction `q·exp(½ωΔ)`.
- Moon, Skelly & Towsley, INFOCOM 1999 [A]: clock-skew estimation from the lower envelope of
  one-way delays. The stock firmware sends no sample timestamps, so only arrival-time mapping is
  possible; a 20 ms offset while walking is about 2.6 cm of displacement error.

## F. Anatomical priors

Knee axial rotation grows with flexion (cadaver and in-vivo figures were seen only as snippets and
are unverified), which argues for a flexion-dependent knee deadband. The head anticipates turns by
roughly 0.2–1 s (Grasso et al., 1998 [A]), so instantaneous headset yaw is never used as body
heading. No citable distribution was found for feet-versus-pelvis heading in relaxed standing; the
pelvis–thigh prior is therefore weak, deadbanded and logged.

## What this changes in the plan

1. Idea adopted: displacement increments with the headset as the absolute endpoint (section B).
2. Known gaps to close next, in order: arrival-time clock mapping and a jitter buffer; a rocker
   contact model; a flexion-dependent knee deadband; a walking-direction heading cue from headset
   velocity during steady gait; bone-length states with strong priors in the same filter.
3. Open risks: contact without foot sensors is the weakest link; residual mounting tilt appears as
   yaw; thigh soft-tissue artefacts peak at heel strike; the chest–pelvis link is not rigid;
   real-user accuracy is unknown until worn comparisons are recorded.
