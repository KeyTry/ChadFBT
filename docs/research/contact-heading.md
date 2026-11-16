# Conditional heading from two body landmarks

Research experiment added November 14, 2026. This module produces a hypothesis for offline evaluation; it does not change live calibration, recovery, or tracker output.

## Why investigate this

Head direction is not torso direction. A head-facing neutral reset is convenient, but cannot independently distinguish a small head-only turn from common heading drift. The headset/controller system also supplies absolute point positions. If a user deliberately places two tracked points at corresponding left/right body landmarks, their separation supplies another heading cue without using head yaw.

This is an application of geometric position aiding and contact constraints, not a claim that landmark calibration or contact fusion is new. Recent [IMU-HOI research](https://arxiv.org/html/2606.28604v1) explicitly models hand/object contact to couple inertial and kinematic hypotheses. Its learned body/object system, sensor placement and evaluation are different from this simple two-point experiment. A [Fraunhofer head/body classification study](https://www.iis.fraunhofer.de/content/dam/iis/de/doc/lv/dataanalytics/Poster/Poster_HeadToBodyNoPoseVR.pdf) instead uses motion patterns and walking direction as heading evidence; its behavioral assumptions should not silently be imposed on seated, sideways-moving or dancing VR users.

The considered gesture is deliberate contact near left/right hip landmarks. Controller tracking origins are not skin contact points. Controller geometry, grip, hand pose and placement repeatability must be calibrated or included in the error budget. Bare-hand tracking may expose a different reference point. The current SteamVR helper does not identify anatomical contact points.

## Geometry and a conditional bound

Let `pL` and `pR` be the tracked points in right-handed standing space (+Y up, -Z forward), and `b = horizontal(pR - pL)`. Under the asserted correspondence, `b` points toward the person's right. Heading is:

`heading = atan2(-b.z, b.x)`

No IMU-estimated hip position or HMD orientation appears in this equation, so the hypothesis does not validate itself against a skeleton derived from the same yaw offsets.

If each point has a worst-case positional error of at most `epsilon`, the difference error is at most `2*epsilon`. For measured horizontal separation `L > 2*epsilon`, the tangent to that error disk gives:

`heading_error <= asin(2*epsilon / L)`

The implementation uses the shortest measured baseline in the accepted window and adds the observed angular spread. This conservatively accounts for the worst-conditioned observation and small allowed changes during the window. The bound is conditional on correct landmarks, correct left/right labels and a valid point-error budget. It is not a statistical confidence interval or demonstrated anatomical accuracy.

For perfectly still observations and a 2 cm error budget **per point**:

| Horizontal separation | Conditional angular bound | Default 8-degree gate |
|---|---:|---|
| 25 cm | 9.21 degrees | Reject |
| 30 cm | 7.66 degrees | Pass |
| 40 cm | 5.74 degrees | Pass |
| 50 cm | 4.59 degrees | Pass |

At 40 cm separation, a 4 cm per-point budget increases the bound to 11.54 degrees and is rejected. Averaging many samples does not shrink unknown systematic grip or placement error. The experiment deliberately preserves this uncertainty floor.

## Implemented acceptance conditions

`tracking_core::contact_heading::estimate_contact_heading` consumes a bounded sequence of paired `AnchorPose` values and a new explicit contact assertion. It requires:

- Finite, valid absolute points, matching anchor session and monotonic new sample times for each hand.
- A sufficiently long but bounded interval, limited pair skew and gaps, and fresh final observations.
- Plausible point separation and vertical difference, small position wander and heading spread.
- A conditional angular bound below the configured limit.

Cached anchors cannot accumulate a stable window. A recenter or anchor-source restart must change the supplied session identifier. The timestamps are received timestamps; equal receipt times do not establish hardware synchronization. Any unmodeled timing error belongs in the positional error budget.

Neither plausible geometry nor stillness verifies body contact. A regression deliberately swaps left/right points: the numerical gates accept them and the result is 180 degrees wrong. Stable points held in front of the body can likewise pass while measuring an arbitrary hand arrangement. This demonstrates why the module returns a **contact-conditioned hypothesis**, not an automatically observed body heading.

## Evaluation before live use

1. Record actual controller/hand reference points and repeated landmark placements, including grip changes and head-only turns.
2. Determine a defensible error budget from repeatability and an external heading reference. Repeated precision alone does not establish absence of systematic bias.
3. Test seated and standing placement, missing/occluded hands, swapped identities, runtime recenter and source changes.
4. Compare this recovery action's time, false acceptance and heading error against the existing head-facing neutral recovery.
5. Only consider live application when the contact assumption is explicit, the error budget is justified, and the change can be previewed and undone.

This provides a possible independent validation cue for a future multi-tracker estimator. It does not continuously solve common yaw while the hands are unconstrained, and it has not been tested on the user's body.
