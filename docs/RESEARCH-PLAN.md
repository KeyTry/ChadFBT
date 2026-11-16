# ChadFBT: evidence-driven tracking development

This is an experimental implementation, **not a demonstrated improvement over SlimeVR**. The
target is better long-session fidelity, fewer disruptive resets, steadier feet and lower perceived
latency. A good-looking skeleton, a passing unit test or a synthetic benchmark does not establish
any of those claims. The release gate for calling this "better" is comparative worn evidence.

Revised 2026-11-16. The 2026-11-15 revision added a source-level review of SlimeVR
([slimevr-internals.md](research/slimevr-internals.md)), a literature review
([literature-review.md](research/literature-review.md)), and a new estimator design that is now
implemented and synthetically tested ([stance-anchored-estimator.md](research/stance-anchored-estimator.md)).
This revision brings "What is implemented now" up to date with the application and adds the
seated and lying hypothesis ([seated-ik-investigation.md](research/seated-ik-investigation.md)).

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
| Floor height and planted feet | Root position, bone lengths | Contact, and for the floor only an upright leg | Root correction and standing-floor prior implemented; length estimation planned |

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

All of this is checked by synthetic regressions, and parts of it by replaying recorded sessions;
none of it is a measured accuracy result. Every recorded frame carries each layer's diagnostics.

What can be switched: the app has one switch per layer (stance heading filter, contact-aware
legs, motion prediction, headset extrapolation, recenter-proof calibration, angular velocity
output) plus the prediction lead and the legacy knee correction. The parts inside a layer are
configuration fields that can be overridden when replaying a recording
(`--replay FILE --dump OUT --estimator JSON`): the stance filter's displacement measurement and
each prior, torso-yaw estimation, and for the legs root correction, foot lock, flat foot,
support-floor pull-down, below-floor raise, the thigh taper (both thresholds at π or more
ablates it), the squat-bow term (both thresholds above π/2) and the measured-flexion guard (a
large `knee_flex_slack_rad`). No switch: the backward-lean rules of the pull-down budget and the
lock slack, the fade-out for unexplained gaps and the soft full extension. The shin taper of the
standing-floor prior shares the flat-foot thresholds.

Estimator:

1. **One kinematic chain** with a head model that separates the eye point from the neck pivot
   (looking down no longer swings the body), shared by the solver and the estimator.
2. **Stance heading filter.** An error filter over per-sensor yaw with six states, one per core
   segment. Its measurement is the mismatch between the headset's displacement and the chain's
   displacement between keyframes while a foot is hypothesised stationary. Only changes of the
   chain carry information, so head turns, static poses and headset noise cannot move the
   estimate. A direction-dependent noise budget keeps bone-length error from being read as yaw.
   By default the displacement measurement estimates the **four leg states only**
   (`estimate_torso_yaw = false`, since the first live session): chest and hip yaw are held by
   weak deadbanded priors that tie the upright torso to the legs, torso lever changes enter the
   noise budget, and accumulated corrections are bounded.
3. **Contact hypotheses** from ankle speed, relative height and shin yaw rate, with hysteresis and
   pivot release, and allowances for heading uncertainty and for torso motion in double support.
   A contact is "this point is stationary", not "this foot bears weight".
4. **Contact-consistent leg IK.** Root correction toward planted feet, horizontal foot lock by
   two-bone IK with exact bone lengths, inertialized release, flat planted feet gated by shin
   tilt. The **support-foot floor constraint** (`SupportFloor`) replaced the earlier ankle-only
   floor clamp: it translates the pelvis and legs so the support foot rests on the floor, within
   a budget that grows with forward or sideways torso flexion, fades out for gaps the budget
   cannot explain, and applies to a leg whose thigh and shin are near upright or, for a folded
   thigh, whose torso is bowed forward as in a squat; feet below the floor raise the body. For
   legs without that standing-floor prior (seated or lying under an upright or reclined torso)
   a foot lock keeps the knee angle the thigh and calf sensors measure to within 1°, and a
   backward-leaning torso no longer widens the lock limits.
5. **Time handling.** Packets are stamped on arrival by a dedicated receive thread; the solver runs
   on arrival (100–250 Hz); each orientation is extrapolated to solve time along its measured
   angular rate, with silence treated as stillness, plus an optional lead of up to 30 ms; headset
   and controller positions are advanced by their reported velocity over their age (at most
   30 ms); tracker velocities and pose age go to SteamVR so it can extrapolate to display time.
6. **Calibration that survives SteamVR.** A headset tracking jump (a recenter) rotates the stored
   heading references instead of discarding calibration; a SteamVR restart, a transform change
   without a jump and headset gaps keep it.
7. **Robustness fixes** from three audits: stationary trackers no longer drop out after a lost
   packet, a receiver pause or a non-stock firmware string; a reconnecting spare tracker no longer
   wipes calibration; the bridge no longer loses headset anchors when the driver is not listening.
8. **Research modules that never change output:** the relative-heading shadow estimator, the
   torso acceleration-heading estimator and the two-landmark contact-heading experiment.

Application:

9. **Calibration by movement** as the primary method (neutral capture, then a bow and knee lifts
   or squats, with per-segment progress and blockers); the held skiing pose remains as a
   fallback, and a manual mounting editor as a last resort.
10. **Desktop-to-headset alignment.** A calibration made without a headset is rotated onto the
    headset's heading once the user stands upright and still, instead of being repeated.
11. **Output modes** with **Auto** as the default: live in SteamVR while a worn headset and a
    live-valid calibration exist, desktop preview otherwise, switching both ways by itself.
    SteamVR, VRChat OSC and Desktop only are the fixed modes.
12. **Hands-free tap gestures** on the chest or hip tracker (off by default): double tap requests
    heading recovery, triple tap starts or cancels calibration by movement.
13. **Spoken prompts and tones** for calibration, gestures and tracking events, on a separate
    audio thread through Windows speech synthesis, with tones as the fallback.
14. **Tracker identification by movement** with explicit confirmation, and automatic eye-height
    update at a live neutral capture.
15. **Single instance with a tray icon:** a second launch restores the running window.
16. **Recording, replay and offline analysis:** input-complete session recordings, a
    deterministic replay check, a re-solve with the current code and optional estimator overrides
    (`--dump`), and the offline relative-heading report (`--analyze-heading`).

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
| H9 | With the 2026-11-16 changes, seated and lying legs are shown as measured: the pelvis is not pulled toward the floor while seated, and locked straight legs do not lift at the knee when the head moves | Sit on a seat of known height with feet flat, then recline, then lie flat on the floor and on a bed, with one standing calibration; compare published pelvis height with the seat height, knee height against hip height while lying, and contact-aware legs on versus off in the same pose and on a replay |
| H9b | The forward bow of the torso separates a squat from sitting well enough: deep squats keep their feet on the floor, and ordinary seated postures do not reach the bow threshold | Deep squats, and seated forward leans with feet on the floor, off the floor and on a rung, in the same recording; standing-floor weight and pull-down traces, with the squat-bow term on versus ablated in replay |
| H10 | The remaining seated pelvis error comes from the torso model (no pelvis segment, proportions absorbed into the chest-to-hip length), not from the leg layers | In the same recording, the raw chain's pelvis height against the seat height with every leg layer off; repeat with a deliberately slouched and an upright seated posture |

H9, H9b and H10 are untested on a worn body. The mechanisms behind H9 and H9b were reproduced
on synthetic exact-truth probes and one replayed seated segment only; see the [investigation](research/seated-ik-investigation.md).

The filter's own innovation statistics are diagnostics, not accuracy: a small residual shows the
model fits its own assumptions, not that the pose is anatomically right.

## Next experiments, in order

0. **Worn seated and lying validation** (H9, H10), before any further change to the leg layers:
   it decides whether a pelvis segment, a seat-support hypothesis or a proportion fit is the
   next step, or none of them.
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
