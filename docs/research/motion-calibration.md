# Motion calibration (experimental)

**Status: synthetic only.** Nothing described here has been validated on a worn body, with real IMUs, or against SlimeVR. The numbers below come from deterministic generators that share the estimator's own assumptions (rigid segments, upright neutral, sensor worlds that differ only by yaw). They show that the implementation matches its model and degrades in the ways listed; they are not accuracy claims. Gate values are proposed engineering tolerances.

Code: `crates/tracking-core/src/motion_calibration.rs`. Tests: `tests/motion_calibration.rs` (support in `tests/motion_support/mod.rs`). Results table: `cargo run --release -p tracking-core --example motion_calibration`.

## What it replaces

The two-pose method ([functional-mounting.md](functional-mounting.md)) takes one held "skiing pose" sample and uses the rotation axis of `q1·q0⁻¹` to find each sensor's heading. The pose must be held still within tight gates and a single sample has no redundancy. Motion calibration keeps the same neutral capture and the same output (`forward_calibration::ForwardAlignment`: left world-yaw `heading_alignment_radians`, right `mounting_xyzw`), but replaces the held sample with evidence accumulated over any mostly sagittal movement performed continuously: squats, a bow, knee lifts (marching in place), or the old skiing pose.

## Model

For core sensor `i` the calibrated bone rotation is `R_i(t) = Ry(ψ_i)·q_i(t)·M_i`. Neutral (user upright, facing reference heading `h`) gives `q0_i`, and once `ψ_i` is known `M_i = (Ry(ψ_i)·q0_i)⁻¹·Ry(h)`, exactly the formula of `solve_forward_alignment`. Hence `R_i(t) = Ry(ψ_i)·D_i(t)·Ry(−ψ_i)·Ry(h)` with `D_i(t) = q_i(t)·q0_i⁻¹`, and the bone's long axis in the room is `Ry(ψ_i)·D_i(t)·Y`.

Per sample, in the sensor's own world frame:

```text
v = D_i(t)·Y          tilt τ = acos(v.y)          p = (v.x, v.z)
```

A sagittal flexion moves `p` along one line through the origin whose direction is the body's fore-aft axis in that sensor world. Twist about the bone (and therefore the yaw part of `D` for an upright bone) does not move `v`. This is the main difference from the rotation-axis formulation: in the held skiing pose with 6° of twist the two-pose solve is biased by 3.0° on every segment and with 15° it rejects every segment (residual gate), while the tip line is unchanged (0.06° error, see table).

Accumulators (samples with 8° ≤ τ ≤ 75°, weight `w` = sample dt capped at 50 ms, so evidence is measured in seconds and is independent of packet rate):

```text
S = Σ w p pᵀ  (about the ORIGIN, not mean-centred)     m = Σ w p
e = principal eigenvector of S,  planarity = λ_min/λ_max
one-sidedness = |m·e| / Σ w|p·e|
f_i = s_i · sign(m·e) · e            s_i = +1 chest, hip, calves;  −1 thighs
ψ_i = wrap(h − atan2(−f.x, −f.z))
```

`s_i` is `−ski_pitch_sign`: the tops of chest, pelvis and calves tip forward in every instructed movement, the top of the thigh tips backward. `Σ w|p·e|` is not a quadratic form, so `Σ w|p·u_k|` is accumulated for 36 fixed directions `u_k` (5° apart over 180°) and interpolated at `e` (error well below 1 % of the ratio).

Model residual: the RMS out-of-sagittal-plane tilt of the long axis with the solved heading, `asin(√(λ_min/W))`, `W = Σw`. This deliberately excludes twist about the bone and yaw (see deviations). RMS twist about the long axis is reported as a diagnostic, and `ForwardAlignment::axis_vertical_degrees` is filled with the equivalent axis elevation `atan2(RMS twist, RMS tilt)`; it is not gated.

## Evidence memory

Every accumulator forgets exponentially in **evidence time**: before adding a sample of weight `w`, everything is multiplied by `exp(−w/τ)`, `τ = 10 s` (`evidence_memory_secs`). Reasons:

- Standing still (no tilt, no evidence) never erodes the estimate, so waiting at the end of the procedure is harmless.
- A bad early stretch is outweighed by later good evidence instead of being frozen in (a hard stop after N seconds would keep the first stretch forever). Measured: nine squats whose first 3.5 s had the torso bending 45° off the sagittal plane end with 1.4° torso error with forgetting and 3.6° without.
- Effective evidence is bounded to about τ seconds, so magnitudes and float precision stay bounded in arbitrarily long sessions; memory is fixed-size anyway.

The cost is that a persistent change of movement direction (a body turn) is eventually accepted, which is why turns are detected separately and latched.

## Gates (per segment) and progress

| Gate | Default | Message intent |
|---|---|---|
| Excitation | peak tilt ≥ 15° **and** ≥ 0.4 s beyond 10° | which segment must move more and how (per-role hint) |
| Planarity | λ_min/λ_max ≤ 0.15 | avoid sideways or circular movement |
| One-sidedness (sign) | ≥ 0.85, else sign undetermined (unless the knee rule resolves it) | use squats/bow/knee lifts, not walking or swinging |
| Repeatability | two accumulators fed by alternating 0.5 s blocks (≥ 0.15 s evidence each) agree within 8° | repeat the same movement steadily |
| Model residual | ≤ 12° RMS sideways tilt | keep the movement forward/backward |
| Stability | estimate moved ≤ 3° over the last 1 s (history sampled every 0.1 s) | keep going |
| Fresh data | stream silent ≤ 0.5 s | sensor stopped |
| Knee checks (legs) | see below | assignment or movement mismatch |

Progress is `0.5 × excitation fraction` until excitation passes, then `0.5 + 0.1 × (passed quality gates out of 5)` capped at 0.95, and 1.0 when the segment is ready. `ready` requires all six core segments ready and no calibration-wide blocker (body turn, unavailable explicit reference). `solve()` is atomic: six alignments or one error naming every blocking segment.

Repeatability is temporal consistency between interleaved blocks, not a repetition count: in noise-free synthetic data one slow squat descent satisfies every gate (squats become ready 2.2 s after neutral). If worn data shows single sweeps are unreliable, add a minimum evidence time rather than tightening these gates.

## Cross-segment checks: what they can and cannot see

**Knee checks.** A paired-frame ring (25 Hz, 1500 frames, frames only when thigh and calf samples are within 60 ms) stores `D` for all six sensors. With body-forward headings `H = heading(f)` in each sensor world, the calf tip in the calibrated thigh frame is `k = Ry(−H_t)·D_t⁻¹·Ry(H_t − H_c)·D_c·Y` (independent of `h`). Flexion `atan2(−k.z, k.y)` is positive for a normal bend; its 95th percentile backward value must stay ≤ 10°; the bend direction `atan2(k.x, −k.z)` averaged on doubled angles over frames bent > 15° must stay within 40° of the flexion direction (25° until the first worn run on 2026-11-15, where a real attempt was rejected at 27° and the retry passed at 15°: a real shank does not swing in exactly the thigh's plane during knee lifts).

Correction to the original sketch: these checks **cannot** detect an error that the per-segment solve has already absorbed. Each segment's heading is chosen to make its own motion sagittal, so a consistent sideways component (thigh moving 40° off-plane while the calf moves sagittally: thigh error 40°, knee looks like a perfect hinge) and a thigh/calf swap (with role signs, a swap is exactly a 180° heading flip of both segments, which leaves knee flexion unchanged) both pass. What the knee checks do catch is time-varying inconsistency (all four leg segments are flagged after a mid-procedure turn) and minority backward bends that the one-sidedness threshold tolerates (a test shows a short "rock back" with the shin tipping 14° backward blocked as hyperextension).

**Knee rule for the sign.** When a leg's thigh or calf line is two-sided (walking), all candidate sign combinations are scored by the RMS backward knee bend. Both flipped turns every flexion into hyperextension; one flipped produces backward bends during straight-leg phases. A combination is accepted only if its RMS hyperextension ≤ 6°, the next best ≥ 15°, and its flexion 95th percentile ≥ 20°; the result is labelled `SignEvidence::KneeRule` and a warning is emitted. In synthetic walking it resolved every two-sided leg segment in 25/25 noise seeds with ≤ 1.4° error (1° sensor noise). It cannot resolve chest or hip; walking leaves them unexcited anyway. It is untested on real knees with soft-tissue artefact; disable with `knee_sign_rule = false` if worn data contradicts it.

**Swap heuristics (warnings only).** Thigh/calf swap: both one-sided and the calf's peak tilt exceeds the thigh's by > 10° (in squats, knee lifts and the skiing pose the thigh normally tilts more). Left/right swap: during asymmetric movement (left and right thigh tilt correlation < 0.3), each thigh's tilt correlates with the opposite calf rather than its own (cross sum > 0.8 and > same-leg sum + 0.6). Neither is proof; both are named as suspicions.

**Body turn.** A whole-body yaw rotates every segment's line by the same angle in its own sensor world; independent sensor errors do not. Each segment keeps a first-epoch line (frozen after 1.5 s of evidence) and a recent line (1.5 s memory). If at least three segments are comparable, the median rotation is ≥ 25°, and at least 3 and ≥ 75 % of them agree within 15° of the median, a turn is latched until `reset()`/`begin()` with the message "You turned about N°…; keep facing the same direction". Measured: a 40° turn between squats is reported as 39.8–39.9°. A returning 10° wobble does not trigger it.

## Headset-assisted absolute forward

Today `h` is the headset yaw at neutral, which assumes head and torso faced the same way. If headset samples are supplied, the neck pivot `p_head + R_head·(0, −head_to_neck, +eye_forward)` (same convention as `chain::solve_chain`) is compared with its neutral position. While the chest is tilted ≥ 15° from neutral and the horizontal displacement is ≥ 2 cm, the displacement `d` (room frame) is accumulated with the same origin-anchored, one-sided line fit; the neck is assumed to move **forward** when the chest tips forward (true for bows and squats; a movement where the hips move back further than the neck moves forward would invert it). Uncertainty: `σ = atan2(√(σ0² + λ_min/W), √(λ_max/W))` with assumed `σ0 = 2 cm`. Fusion with the yaw reference (assumed 8°) is a circular inverse-variance mean; if the two differ by more than 25° fusion is withheld, the yaw reference is kept and `HeadsetDisagreement` is raised. The caller chooses `MotionReference::{HeadsetYaw, Fused (default, falls back to yaw), NeckDisplacement}`.

Measured (bow with 25–26 cm peak neck displacement, 5 mm headset noise): displacement heading error 0.0°, σ 5.7°. Head turned 20° at neutral: yaw reference 20° off, fused 6.8° off, not flagged. Head turned 40°: flagged, yaw kept, displacement still 0.0° off and available via `solve_with(NeckDisplacement)`. A sideways torso bend during squats moves the neck sideways: displacement differs from yaw by 111° and is flagged; this is the only check here that exposes a sideways torso lean.

## Robustness rules

- Samples are ignored (with a reason) when the session is inactive, the role index is invalid, time or quaternion is nonfinite/degenerate, the sample predates the neutral capture, it is out of order or duplicate for its stream, or it is older than the newest accepted time minus 0.25 s (stale). All streams must share one monotonic arrival clock.
- A sample arriving after a > 0.25 s gap carries zero weight, restarts the repeatability block, and keeps the accumulated evidence.
- Memory is fixed: six segments with fixed accumulators, a 32-entry history each, and the paired-frame ring (default 1500 frames, validated 16…200 000). A 10-minute session keeps 400/400 frames with that cap and evidence ≤ τ.
- No panics and no nonfinite outputs: 20 000 garbage samples (NaN, ±∞, `f32::MAX`, zero quaternions, random times and roles) leave every status field finite. Results are bit-identical for identical inputs.

## API

`MotionCalibrationConfig` (serde, `Default`, `validate`), `MotionCalibrator::new(config) -> Result`, `begin(neutral_at_secs, neutral: [Quat; 6] in Role::CORE order, reference_heading, neck_at_neutral: Option<Vec3>)`, `MotionCalibrator::neck_pivot(...)` to compute `neck_at_neutral` consistently, `observe_sensor(role_index, at_secs, q) -> SampleOutcome`, `observe_headset(at_secs, position, orientation, head_to_neck_m, eye_forward_m) -> SampleOutcome`, `status() -> MotionCalibrationStatus` (serde: per-segment progress, blockers with gate kind and message, tilt, planarity, one-sidedness, sign source, repeatability, residual, stability, twist, sensor-frame forward heading, heading alignment so far; knees; warnings; headset forward; selected reference; latched turn), `solve()` / `solve_with(reference) -> Result<MotionCalibrationSolution, CoreError>`, `reset()`.

`begin` takes the neutral capture time in addition to the sketch's arguments so that samples from before neutral are rejected. Integration into `TrackingCore` (feeding samples, applying the solution like `calibrate_forward`, storing the chosen reference as the new `reference_heading`) was added afterwards: see `begin_motion_calibration`/`finish_motion_calibration` in the [tracking-core README](../../crates/tracking-core/README.md) and `tests/motion_calibration_core.rs`.

## What it cannot detect

- A sideways lean of a segment, or of a group of segments, is indistinguishable from another sensor heading. Without a headset the torso headings simply come out 90° wrong for a sideways torso bend and every gate passes. The headset displacement check exposes torso leans only.
- A leg whose flexion plane is not sagittal biases that leg. Knees tracking 15° outward: 15–16° leg error, undetected. Synthetic default 5° toe-out (knee pole follows the foot): 4.5° thigh, 7.2° calf.
- Thigh/calf swap within a leg (only the tilt-range heuristic hints at it) and left/right swaps during symmetric movement (squats).
- Twist couples into the tip when the bone was not vertical at neutral: synthetic squat legs lean about 2° inward at neutral, and ±15° twist then gives up to 3.4° error (0.0° for bones that were vertical at neutral).
- A gradual body yaw during the movement biases all headings by roughly its evidence-weighted mean (10° wobble: ≤ 1.9° measured); a turn larger than 25° is caught.
- Sensor yaw drift, gravity-estimation error during fast motion, strap slip, soft-tissue motion and real knee kinematics are not modelled.
- A sensor restart changes its world frame and invalidates neutral; the runtime must call `begin` again after a new neutral capture.

## Measured synthetic results

`cargo run --release -p tracking-core --example motion_calibration` (rig seed 7, 100 Hz, 0.3° per-axis sensor noise, reference = true body heading unless stated). "Error" is each segment's solved body-forward direction versus truth, in degrees, ordered C H LT RT LC RC; `k` marks a sign chosen by the knee rule. "Ready at" is seconds after neutral.

| Scenario | Ready (at) | Max abs error | Notes |
|---|---|---|---|
| Squats ×3 | yes (2.2 s) | 0.89 | legs lean 2° at neutral |
| Bow ×2 + march ×4 | yes (11.2 s) | 0.06 | |
| Held skiing pose | yes (2.0 s) | 0.06 | matches two-pose solve within 1° |
| Held skiing pose, 15° twist | yes (2.0 s) | 0.06 | two-pose solve rejects all six |
| Bow + march, ±15° twist | yes (11.2 s) | 0.05 | |
| Squats, ±15° twist | yes (2.8 s) | 3.44 | coupling via 2° neutral leg lean |
| Squats, 10° yaw wobble (2.3 s / 9 s) | yes | 1.08 / 1.85 | |
| Squats, 40° turn mid-way | no | 32.2 | turn latched at 39.9°, knee-axis flags |
| Knees-out 15° (leg plane / thigh only) | yes | 15.9 / 15.2 | undetected bias |
| Squats, 5° toe-out | yes | 7.22 | undetected bias |
| Squats ×9, first 3.5 s torso 45° off | yes | 1.44 (3.57 without forgetting) | |
| Squats, async 90 Hz, 1° noise, 1/7 drops | yes (2.3 s) | 0.89 | |
| Squats, torso bends sideways | yes | 90.0 | undetectable without headset |
| Same with headset | yes | 90.0 | `HeadsetDisagreement` (111°) |
| Squats, thighs 40° sideways | yes | 40.2 | absorbed; knee looks normal |
| Squats, left thigh/calf swapped | yes | 179.8 | `SuspectedThighCalfSwap` |
| Bow + march, thighs L/R swapped | yes | 0.06 | `SuspectedLeftRightSwap` |
| Walking, knee rule on | no (torso unexcited) | 1.02 | LT, LC, RC signs via knee rule |
| Walking, knee rule off | no | — | LT, LC, RC sign undetermined |
| Bow (pelvis 4°) + march | no | — | hip: Excitation only |
| Headset bow, head turned 0° / 20° / 40° | yes | 0.14 | displacement 0.0°; fused 0.0 / 6.8°; 40° flagged |

## Validation still required

Record live sessions separately from these tests: worn squats, bows, knee lifts and skiing poses with an independent reference (optical markers or a careful video/protractor protocol), deliberate knees-out and toe-out variants, sideways leans with and without the headset, deliberate assignment swaps, and walking for the knee rule. Compare against the two-pose method on the same captures. Only then can gate values or defaults be described as anything more than proposals.
