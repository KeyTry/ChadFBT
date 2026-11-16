# ChadFBT: evidence-driven tracking development

This is an experimental implementation, **not a demonstrated improvement over SlimeVR**. The
target is better long-session fidelity, fewer disruptive resets, steadier feet and lower perceived
latency. A good-looking skeleton, a passing unit test or a synthetic benchmark does not establish
any of those claims. The release gate for calling this "better" is comparative worn evidence.

Revised 2026-11-15. This revision adds a source-level review of SlimeVR
([slimevr-internals.md](research/slimevr-internals.md)), a literature review
([literature-review.md](research/literature-review.md)), and a new estimator design that is now
implemented and synthetically tested ([stance-anchored-estimator.md](research/stance-anchored-estimator.md)).

## What the hardware can and cannot tell us

The LSM6DSV is a six-axis accelerometer/gyroscope, not a heading or position sensor
([ST datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf)). Firmware 0.7.2 runs VQF in
6-D mode and sends a fused quaternion and gravity-removed body-frame acceleration at up to 100 Hz,
with no raw samples and no sensor timestamps, and it stops sending while rotation is unchanged.
Gravity fixes inclination; yaw drifts, and the vertical-axis gyro bias is not observable from the
accelerometer during motion. The headset and controllers, in contrast, are absolute and drift-free
in a gravity-aligned room frame with a known floor.

Everything that follows is a consequence of one question: **which absolute evidence exists, and
when is it informative?**

| Evidence | Observes | Informative when | Status |
| --- | --- | --- | --- |
| Gravity (in firmware) | Inclination of every sensor | Always | Used via the quaternions |
| Headset position against the IMU chain during stance | Per-sensor yaw error **including common mode** | The body moves over a stationary foot | Implemented: stance heading filter |
| Knee hinge, pelvis–thigh and chest–pelvis consistency | Relative yaw only | Flexion; upright posture | Implemented as weak deadbanded priors inside the filter |
| Headset acceleration against torso IMU acceleration | Chest and pelvis yaw | Stepping, walking, bobbing | Research module (shadow); see [acceleration-heading.md](research/acceleration-heading.md) |
| Headset yaw | Nothing about body yaw by itself | Only under an explicit user assertion | Used only by explicit recovery |
| Floor height and planted feet | Root position, bone lengths | Contact | Root correction implemented; length estimation planned |

## Where SlimeVR stands, and the gap we aim at

From the inspected source snapshots: SlimeVR builds a pure top-down forward-kinematic skeleton
from the headset; its yaw-drift tools are relaxed-pose priors applied at a fixed rate (Stay
Aligned, off by default) and an older reset-rate extrapolation that is disabled; headset position
is never used as heading evidence; contact handling is cosmetic, position-only, does not preserve
segment lengths and is disabled when the hips are low; feet rotate rigidly with the shin; the
driver sends no pose age and no velocity by default. Details and file references are in
[slimevr-internals.md](research/slimevr-internals.md).

None of that proves SlimeVR tracks worse. It identifies evidence SlimeVR does not use, which is
where a different estimator has room to be better.

## Working hypothesis

Improvement comes from choosing *when and what to correct* from evidence that is actually
informative, with explicit uncertainty, and never from increasing the strength of a prior.
Sensor mounting, slowly changing heading error, body dimensions, tracking-space alignment and
contact hypotheses are separate states with different timescales and different evidence.

## What is implemented now

All of this is verified by synthetic regressions only. Each layer has its own switch in the app
and its own diagnostics in every recorded frame, so it can be ablated on a recording.

1. **One kinematic chain** with a head model that separates the eye point from the neck pivot
   (looking down no longer swings the body), shared by the solver and the estimator.
2. **Stance heading filter.** A six-state error filter over per-sensor yaw. Its measurement is the
   mismatch between the headset's displacement and the chain's displacement between keyframes
   while a foot is hypothesised stationary. Only changes of the chain carry information, so head
   turns, static poses and headset noise cannot move the estimate. A direction-dependent noise
   budget keeps bone-length error from being read as yaw. Weak deadbanded priors tie the upright
   torso to the absolutely observed legs.
3. **Contact hypotheses** from ankle speed, relative height and shin yaw rate, with hysteresis and
   pivot release. A contact is "this point is stationary", not "this foot bears weight".
4. **Contact-consistent leg IK.** Root correction toward planted feet, exact-length foot lock by
   two-bone IK, inertialized release, flat planted feet gated by shin tilt, optional floor clamp.
5. **Time handling.** Packets are stamped on arrival by a dedicated receive thread; the solver runs
   on arrival (100–250 Hz); each orientation is extrapolated to solve time along its measured
   angular rate, with silence treated as stillness; tracker velocities and pose age go to SteamVR
   so it can extrapolate to display time; the headset helper runs at 200 Hz with velocities.
6. **Recenter-proof calibration.** A SteamVR tracking-space change or headset tracking jump
   rotates the stored heading references instead of discarding calibration.
7. **Robustness fixes** from three audits: stationary trackers no longer drop out after a lost
   packet, a receiver pause or a non-stock firmware string; a reconnecting spare tracker no longer
   wipes calibration; the bridge no longer loses headset anchors when the driver is not listening.

## Hypotheses to test on worn recordings

Each is falsifiable with the listed measurement. Until measured, each is only a hypothesis.

| # | Hypothesis | Measurement |
| --- | --- | --- |
| H1 | The stance filter keeps leg heading error bounded over a 30-minute session without resets | Return-to-marked-pose heading error every 5 minutes, filter on versus off on the same recording |
| H2 | It does not invent rotation: head-only turns, sitting still and deliberate held twists produce < 2° of correction | Filter correction traces during those scripted segments |
| H3 | Foot lock removes visible sliding without pops | Planted-foot target displacement during verified stance; largest per-frame target jump at release |
| H4 | Root correction steadies the hips when the head moves | Hip target displacement during seated or standing head-only motion, on versus off |
| H5 | Prediction and velocity output reduce perceived lag without overshoot | Tracker-versus-headset phase lag in a head-and-hip bob; overshoot on abrupt stops |
| H6 | Recenter carry-over preserves alignment | Heading error before and after a deliberate recenter |
| H7 | Torso acceleration matching identifies the acceleration frame and chest yaw | Shadow candidate versus a deliberate recovery in the same session |
| H8 | None of this costs meaningful CPU | Solve p50/p95/p99 with every layer on |

The filter's own innovation statistics are diagnostics, not accuracy: a small residual shows the
model fits its own assumptions, not that the pose is anatomically right.

## Next experiments, in order

1. **Torso acceleration heading**, from shadow to applied, once worn recordings show the frame
   identification and the yaw candidates behaving (H7).
2. **Bone-length states** in the same filter. The displacement residual *along* a moving bone and
   the vertical closure to the floor observe lengths; today that component is only discounted.
   Strong priors, slow adaptation, saved to the profile. This is an online counterpart to
   AutoBone's offline recording.
3. **Rocker contact model.** The contact point migrates from heel to forefoot during stance; a
   shank-fixed arc should reduce the systematic residual in walking.
4. **Arrival-time clock mapping and a small jitter buffer** per tracker, from the lower envelope of
   packet delays, to tighten time alignment without sensor timestamps.
5. **Flexion-dependent knee deadband**, since tibial rotation grows with flexion.
6. **Mounting-twist states and strap-slip detection.** Synthetic testing showed that a strap
   rotating around the limb is a twist about the bone's own axis: it leaves every chain position
   unchanged, so displacement cannot see it, and a world-yaw correction is the wrong remedy on a
   pitched bone. Add a per-segment twist state on the mounting (right) side, observed by the knee
   hinge and the upright priors, so sensor drift (left, world yaw, seen by displacement) and
   mounting change (right, bone twist) stay separate as the engineering rules require. Surface a
   suspected slip to the user rather than silently absorbing it.
7. **Continuous mounting refinement** from motion, separated from heading by its different
   signature across motion planes.
8. **Learned residuals**, only for this exact layout and only as proposals for unmeasured degrees
   of freedom. AMASS-based training cannot ship commercially; self-captured data would be needed.
9. **Raw-IMU path** if firmware can supply synchronized samples: VQF comparison and joint-centre
   acceleration constraints with the headset as the single heading root.

## Recovery as a measured event

A double tap means "please try to recover", not "I am upright and facing forward". Gesture
detection, pose eligibility and correction execution stay separate. Recovery remains an explicit,
gated, undoable yaw assertion. With the stance filter running it should be needed far less often;
that is hypothesis H1, not a promise.

## Contact and additional sensors

A stationary calf does not prove a planted foot, and IMUs alone cannot distinguish a still
airborne foot from a grounded one. Contact here is a hypothesis that must stay consistent with
the absolute headset, is released on pivots and outliers, and never makes a target valid. Foot
IMUs would measure ankle articulation and improve contact evidence; upper-arm IMUs remove elbow
ambiguity. Sensor roles and output targets remain separate concepts.

## Headset-free development

Desktop preview keeps its separate, clearly labelled kinematic displacement estimate. The stance
filter and leg IK require an absolute headset and are inactive in preview. Preview is never sent
to VR software as valid tracking.

## Evaluation protocol

1. Record firmware version, placement, strap tightness, body dimensions, Wi-Fi conditions, headset
   runtime, solver version and every estimator switch. Capture inputs once; replay is deterministic.
2. Run neutral standing, head-only turns, full-body turns, squats, single-leg stance, seated
   movement, crossed legs, floor poses, fast stepping, pivots, slow shuffles and returning to a
   marked neutral pose. Include controller occlusion, sensor loss and reconnect, a deliberate
   recenter and a deliberately rotated thigh strap.
3. Test at least 20–30 minutes for drift, cold versus warmed sensors. Repeat with stable SlimeVR
   and its current release candidate, with Stay Aligned properly configured, using comparable
   calibration effort.
4. Report solve time percentiles, packet-to-solve latency, output age, visible latency, foot
   sliding during verified contact, return-to-reference heading error, false corrections, resets
   per hour, recovery time and setup time. Separate accuracy from smoothness.
5. A camera or optical reference is needed for credible joint-position error. Without one, report
   repeatability and constraint residuals, not anatomical accuracy. Never use the same constraint
   as both the optimizer's objective and the only accuracy metric.
6. Ablate every layer independently using its switch. Inject synthetic yaw drift, packet loss,
   jitter and sensor swaps into recordings to check behaviour; synthetic success does not replace
   worn tests.

## Integration and product choices

- Native Rust application with GPU-rendered egui/eframe; no web server or browser UI.
- A worker owns estimation, a receive thread owns the tracker socket, a writer thread owns disk.
  The window can hide to a tray icon.
- Original firmware-protocol parser and estimator; wire compatibility is not algorithm cloning.
- An original C++ OpenVR driver publishes generic trackers from its own receive thread; a client
  helper reads headset and controller poses in standing space. Raw/standing transforms are explicit.
- SteamVR is the primary path. VRChat OSC is a separate, mutually exclusive output mode.
- Local profiles and captures; no cloud dependency, telemetry or uploaded motion data.
