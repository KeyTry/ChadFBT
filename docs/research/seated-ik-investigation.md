# Seated tracking: pelvis height, support evidence and leg IK

Reviewed 2026-11-16, in two passes on the same day. This is a source review and engineering
investigation, not live validation. No hardware identifier, saved profile or personal body
measurement was used here, and no figure from a recording appears below.
Scope: HMD position/orientation plus calibrated chest, pelvis, thigh and calf orientations. The
user reports low hips and raised legs in both the app view and avatar, including lying in bed.
This broadens the investigation beyond an avatar-only or floor-only explanation.

## Status at a glance

| Statement | Evidence class |
| --- | --- |
| A margin at full extension bends every locked straight leg | Demonstrated on synthetic poses; changed |
| A locked straight leg compressed by millimetres raises its knee by centimetres, with or without that margin | Demonstrated on synthetic supine and floor-sitting poses, end to end; changed (flexion guard, soft extension) |
| The floor pull-down lowers a seated pelvis | Demonstrated on a synthetic chair pose and seen in a replay of one live session with a short seated segment (no ground truth); changed (thigh taper) |
| A backward-leaning torso earned pull-down budget; the budget was spent on gaps it could not explain | Code reasoning; changed (bow-only budget, fade-out) |
| With the thigh taper alone, deep squats lose the floor prior and their feet float | Demonstrated on synthetic squats with a spine-model error; changed (squat-bow term) |
| A supine wearer's slowly moved leg stays held to a stale anchor | Demonstrated on a synthetic supine pose; changed (bow-only lock slack) |
| These changes fix what the user sees when sitting or lying | **Hypothesis. Not tested on a worn body.** |
| The modelled pelvis is at the right height on a seat or bed | Not established; known reasons it may not be are listed below |

## First pass, 2026-11-16: full extension and the thigh taper

A synthetic straight horizontal leg reproduced a two-bone IK defect: maximum reach was forced
to `thigh + calf - 1 mm`, so a roughly 1 mm anchor/root disagreement that activated IK
introduced roughly 2 cm of knee bend. The square-root geometry near full extension amplifies the
small enforced shortening. The first pass removed the margin at full extension and kept a
protected minimum reach. This removed the forced bend. It was described at the time as the
correction of the straight-leg defect; the second pass showed it was only part of it.

The first pass also weighted the standing-floor prior by thigh tilt as well as calf tilt.
The calf weight is full through 40° and zero at 60°; the thigh weight is full through
60° and zero at 80°, with smoothstep tapers multiplied. This affects downward `SupportFloor`
correction and near-floor planted-anchor attraction. Contact eligibility, horizontal foot locks,
below-floor raising and measured segment orientations are unchanged. When the prior is zero,
planted anchor height follows FK with τ = 0.3 s, so `foot_locked = true` need not mean a fixed
vertical coordinate. This intentionally avoids an unsupported floor-height guarantee and can
permit vertical drift. A neutral synthetic chair case's pull-down is removed.

The thigh thresholds are an independent code-review heuristic for conservatively discounting
near-horizontal legs, not paper-derived values or a measured contact classifier. A chair and a
parallel squat have the same leg orientations, so with this taper alone both lost the prior (the
verification follow-up below restores it for a forward-bowed torso); genuinely grounded seated
feet may float if body geometry is wrong.
Near-horizontal thighs with vertical calves remain compatible with ordinary chair sitting, raised
unsupported feet and other poses. A bed may support the pelvis above floor level.

## Second pass, 2026-11-16: measured flexion, bow-only budget, unexplained gaps

A review of the first pass found its documentation overstated and three mechanisms untouched.
The source of truth for everything below is `crates/tracking-core/src/leg_ik.rs`.

1. **Knee rise from compression is not a margin artefact.** A straight leg whose hip–ankle
   distance is shortened by `d` moves its knee out of line by about `√(thigh·d)`; a few
   millimetres become a few centimetres. Removing the 1 mm margin removed a constant
   compression, not the sensitivity. When lying or long-sitting, any headset motion toward the
   locked feet compresses both legs and lifts the knees. A synthetic
   end-to-end probe on the intermediate code measured knee rise of several centimetres for
   supine and floor-sitting poses under millimetre headset slides.
   *Change:* a measured-flexion guard. The guarded reach keeps the knee's interior angle within
   `knee_flex_slack_rad` (default 1°, after the follow-up below) of the angle the thigh and calf
   sensors measure; the rest of the request slides the ankle along the hip–anchor line. The
   applied reach is blended between the free and the guarded one by `1 −` the standing-floor
   weight, so an upright standing leg behaves as before, on the reasoning that there the floor
   is the stronger evidence. In addition the reach near full extension is softened and pinned
   to the measured reach, so a target at the measured distance reproduces the measured leg.
2. **A backward lean is not a bow.** The pull-down budget was the height a rigid torso loses
   with flexion in any direction. A torso reclined against a backrest or lying supine is
   supported, not bowed. *Change:* the budget uses `TorsoFlexion::bow_vertical_m`, which counts
   forward and sideways tilt in full and fades to nothing for a straight backward lean, judged
   from each torso sensor's own forward axis against gravity (independent of heading drift).
3. **A gap the budget cannot explain is evidence against floor contact.** Previously a foot far
   above the floor still received the whole budget. *Change:* the pull is scaled by an
   `explained` factor that is 1 up to the budget and fades to 0 at `2·budget + 5 cm`, so a jump,
   a step or box, a stool or a bed gets no pull instead of a partial one.
4. **The thigh taper became configuration** (`support_thigh_full_tilt_rad`,
   `support_thigh_zero_tilt_rad`, defaults 60° and 80°). Setting both to π or more ablates it,
   which makes with/without replays possible.

### Verification follow-up, 2026-11-16

An adversarial verification run on synthetic end-to-end scenarios (exact synthetic truth through
`TrackingCore`, app-like jitter) found defects in the second pass itself. Changes:

1. **Knee slack 2° → 1°, blended by weight.** Widening the slack angle with the standing-floor
   weight acted like a switch near the end of the thigh taper, because a few degrees already
   free centimetres of reach. The reach is now blended linearly. 1° caps the knee rise of a
   straight 0.45 m thigh at about 4 mm.
2. **Cubic deadband at full extension.** `d³/(d² + s²)`, `s` = 5 mm, replaces the 3 mm
   square-root softening: with the earlier form standing-still knee jitter roughly doubled
   against the committed code, whose 1 mm margin had been an accidental deadband.
3. **Squat-bow term.** `upright_leg_weight = max(thigh taper, bowed) × shank taper`; `bowed`
   rises from 0 to 1 as the forward lean of the mid-spine goes from 20° to 35°
   (`squat_bow_start_tilt_rad`, `squat_bow_full_tilt_rad`; both above π/2 ablates). With the
   thigh taper alone the verifier measured feet floating by several centimetres, up to over ten,
   in deep squats when the spine model was in error. A parallel or deep squat with a
   forward-bowed torso keeps the prior; an upright or reclined seated wearer still loses it.
   Known cost: a wearer leaning well forward on a seat with the feet off the floor gets the
   pull-down back, and a perched pose (thigh about 60–70°) with a forward lean and the feet on a
   rung is still pulled down (that one is not a regression).
4. **Lock slack counts bows only.** The lock-break radius, root-correction limit and
   anchor-creep deadzone use `TorsoFlexion::bow_horizontal_m`. Before this a 15° leg abduction
   while supine left the ankle well over 10 cm from the model, held to a stale anchor.
5. `reach_margin_m` is capped at a quarter of the reach range inside the solve.
6. With the floor clamp enabled, the guarded ankle is bounded at the floor margin: a re-run of
   the probes found a crawling leg's ankle carried centimetres below the floor along a downward
   hip-to-anchor line.

Synthetic exact-truth probes through `TrackingCore` with app-like jitter (not worn validation)
covered a chair with feet on and off the floor, a recline with a footrest, supine flat, supine
with knees up on a raised platform, side-lying, floor sitting and cross-legged sitting. After the
follow-up changes these ended within about a centimetre of the synthetic truth at hip, knee and
ankle, where the committed code dropped the pelvis by tens of centimetres for supine knees-up and
raised the knees by centimetres for flat legs; headset slides toward locked flat feet no longer
raise the knees by centimetres. The probes use the estimator's own body model, so they show the
code no longer adds these errors to its model, not that the model matches a body.
Residuals the verifier still found: transitions through the tapers; perched with a forward lean;
side-lying and prone torsos still earn bow budget and slack, so only the leg tapers protect
them. Its slow-leg-move residual while lying was measured before change 4.

**Replay.** One recorded live session contains a short seated segment; no reclined or lying data
exists. Replayed with the thigh taper ablated and enabled, the earlier pull-down lowered the
pelvis substantially during sit-down, stand-up and forward-leaning crouches, and the taper
removed that while seated. With the new settings ablated the same replay differed from the
defaults by millimetres in upright-leg spans (median zero) and by centimetres only in seated or
folded-leg spans, which bounds the standing regression risk in that one session. A replay has no
ground truth: it shows what the code computes, not where the body was.

**What remains unverified or unaddressed.**

- Worn validation of sitting on a seat of known height and of lying down is **pending**. Until
  then every statement about seated or lying appearance is a hypothesis.
- The torso is two rigid rods with no pelvis segment. Posterior pelvic tilt or a slouch when
  sitting can leave the modelled pelvis a few centimetres low in the raw chain. Nothing models a
  seat or bed height, and no absolute pelvis position is measured.
- `normalize_chain_height` absorbs any leg/torso proportion error into `chest_to_hip`. Standing
  hides that (only the total matters); sitting does not, because the legs no longer carry the
  pelvis height.
- Pull-down transients can still occur while sitting down or standing up, while the leg passes
  through the tapers or the torso leans forward.
- A forward lean on a seat is indistinguishable from a squat: feet off the floor or on a rung
  are pulled down. Side-lying and prone torsos are protected by the leg tapers only.
- The guard keeps the measured knee angle, which is only as good as the thigh and calf mounting
  calibration. With the default slack a straight guarded leg can still move its knee by a few
  millimetres.
- The 1° slack, 5 mm deadband, taper and squat-bow angles and fade range are engineering choices
  with no worn tuning.

Compare the existing **Contact-aware legs** switch on/off in the same pose with the same standing
calibration; do not recalibrate seated to conceal the mismatch.

## Finding and evidence boundary

An IMU gives segment orientation, not the location of the segment or proof of support. A headset
provides one absolute endpoint. Inferring the pelvis from that endpoint requires a torso and neck
model; inferring a planted foot additionally requires a contact hypothesis. A chair can support the
pelvis while both feet are raised, and a vertical calf can belong to a seated, unsupported leg.

The most directly relevant primary research is [GlobalPose, SIGGRAPH 2025, section
3.3](https://arxiv.org/html/2505.05010v1): it identifies missing pelvis contact during sitting as a
specific limitation of ground-only physics models. Its method includes hands, feet and pelvis as
candidate contacts. This supports separating seated support from floor support; it does not prove
that a chair contact can be reliably recognized from ChadFBT's stream.

[TIP, SIGGRAPH Asia 2022, appendix F](https://arxiv.org/html/2203.15720) explicitly describes the
ambiguity between a stationary foot in air and a foot resting on terrain. Its temporal learned
prior and stationary-body-point model select plausible explanations. They do not turn stationary
orientation into measured floor contact.

## Baseline before the 2026-11-16 changes

The observations below describe the code as it was when the investigation started, before
either pass above; bracketed notes say what has changed. Inspected
`crates/tracking-core/src/chain.rs` and `leg_ik.rs`, and
`crates/tracking-core/tests/support_floor.rs` at investigation time. These are code observations,
not measurements of a wearer:

- The raw chain locates the neck from HMD position and an eye-to-neck lever, then subtracts a
  chest-oriented rod and a chest/pelvis midpoint-oriented rod to locate the pelvis. Thus spine
  shape, neck articulation, mounting tilt and segment lengths can shift the inferred pelvis even
  when headset position is correct. Midpoint slerp is a model choice, not an observed spine curve.
- `SupportFloor` selects the lowest usable ankle. If it is above the floor and its calf is near
  vertical, it can lower the pelvis and both legs within a torso-flexion-derived budget. It has no
  chair-height measurement, seat-contact state or thigh-based seated-support evidence.
  [Changed: thigh taper, bow-only budget and fade-out for unexplained gaps. Still no chair-height
  measurement or seat-contact state.]
- Support-floor translation preserves knee-minus-hip and ankle-minus-hip vectors. It can lower a
  whole seated leg, but cannot alone rotate a horizontal thigh into a raised thigh.
- `LegIk` can then move the pelvis toward planted anchors and recompute knees with analytic
  two-bone IK. Its pole comes from the measured FK bend plane, with a near-straight-leg fallback.
  Segment rotations remain unchanged while joint positions can change. A consumer performing
  further IK may interpret that combination differently from the local preview.
  [Changed: for legs without the standing-floor prior the solve keeps the measured knee angle
  within 1°.]
- The existing test named `kneeling_sitting_and_lying_on_the_floor_are_left_alone` covers long
  sitting with horizontal calves, and a low-pelvis lying case. It does not establish ordinary chair
  sitting with vertical calves is excluded. The broad sitting claim in the comment exceeds that
  particular test's coverage. [Chair sitting with vertical calves is now covered separately by
  `tests/seated_ik.rs` and `tests/reclined_tracking.rs`.]

## Separate the likely mechanisms before tuning

These are diagnostic hypotheses, not established causes of the reported seated pose:

1. **Pelvis too low already in raw FK:** inspect the HMD-to-pelvis vertical and horizontal lever,
   chest and pelvis mounting, and fixed-rod torso approximation. A seat-height clamp would conceal
   this error and introduce an unobserved environmental assumption.
2. **Raw FK reasonable, support-floor pelvis lower:** inspect its support ankle, calf weight,
   gap and vertical correction. Test chair sitting with raised feet, crossed legs, one grounded
   foot and both vertical calves. These cases are not equivalent to long sitting on the floor.
3. **Knee above hip already in FK:** inspect calibrated thigh `R * -Y` and compare its vertical
   component with the expected synthetic pose. Check assignment, mount, frame/sign and tilt;
   changing root height cannot repair a wrong relative direction.
4. **Raised knee appears after lock:** compare target reach and pole direction with FK. A false
   anchor can force a mathematically valid triangle that contradicts the sensed leg. (This is
   the mechanism the second pass reproduced synthetically and bounded with the flexion guard.
   Whether it is what the user saw is not established.)
5. **Only the avatar looks wrong:** compare exported waist/knee/foot targets and tracker rotations
   with preview coordinates before changing the core. Avatar proportions and a second IK solver
   create a separate inference layer.

For a two-bone leg, target distance must lie between the difference and sum of the bone lengths.
The endpoint does not uniquely choose the knee's swivel around the hip-to-ankle line. Preserve
the sensed bend plane, stabilize degeneracy near straightness, and retain a measured-orientation
residual diagnostic. Exact endpoint satisfaction is not proof of correct anatomy. (The flexion
guard now bounds that residual for non-standing legs; a per-leg residual diagnostic is still not
reported.) Analytical limb
redundancy and separate position/orientation constraints are treated in [Tolani, Goswami and
Badler, 2000](https://www.cis.upenn.edu/~badler/gmod/0528a.pdf).

Foot-lock cleanup assumes footplants are supplied as constraints; it does not solve whether a
plant occurred. [Kovar, Schreiner and Gleicher, 2002](https://graphics.cs.wisc.edu/Papers/2002/KSG02/)
is useful precedent for restrained postprocessing. Its allowance for small length changes should
not be silently imported into this fixed-length tracking model.

## Scoped recommendation

First expose or compare three pose stages: head-anchored FK, support-floor translation, and
contact/leg-IK output. Report inferred pelvis position, each correction, contact confidence and
per-leg orientation-versus-position discrepancy. Keep diagnostics out of normal product flow
unless they help a calibration or recovery decision. *Status:* the replay dump reports the
support-floor correction, root correction and lock state per frame; a dedicated three-stage
comparison and the per-leg discrepancy are not implemented.

The thigh prior conservatively reduces floor attraction for near-horizontal
legs. Further support inference could use motion history and competing hypotheses, but must not
be presented as proof of a chair or its height. Distinguish the lower body's floor-penetration
guard, floor-attraction prior and stationary-point hypotheses: horizontal locks remain independent
of the new prior, while vertical anchors can follow inferred height. A vertical calf alone does
not establish floor support. Preserve validity and
finite/stale rules through any later refinement.

Avoid adding a global seated height, a generic knee-forward pose, or head-yaw-driven body
alignment. Those overwrite valid crossed-leg, reclined, sideways or asymmetric poses. If seat
support is later added, keep its explicit anchor/evidence independent of foot contact. A user
assertion, an observed surface, pressure data or a real positional pelvis tracker can strengthen
that evidence; they have different capabilities and failure modes.

Use synthetic chair-sitting, unsupported raised feet, crossed legs, transitions, head-only motion,
single-foot support and no-contact cases to test correction gates, bone lengths, bend continuity,
frame invariance and stale dependencies. Use neutral synthetic dimensions only. These tests prove
model behavior; independent reference tracking is needed to assess worn accuracy.

*Status of these recommended tests when this document was last revised.* Written: chair sitting
with vertical calves and slightly raised ankles, a standing-to-folded and a standing-to-reclined
transition, straight horizontal legs under small root and headset residuals at several headings,
bone lengths and stale inputs for those poses, and the thigh taper's weights. Not written:
crossed legs, single-foot support while seated, seated head-only motion as its own scenario, and
no-contact seated cases. The second-pass mechanisms (flexion guard, backward-lean budget, the
fade-out for unexplained gaps beyond the upright jump case, the squat-bow term, bow-only lock
slack, ablation of the thigh taper) were exercised by the probes and replay described above; check `crates/tracking-core/tests` for
which of them have dedicated regressions before relying on this list.

## Current research leads and transfer limits

This is a targeted scan through 2026-11-16, not a claim of exhaustive literature coverage or a
ranking on this hardware. Results belong to each paper's input configuration and evaluation.

| Primary work | Relevant idea | Limit here |
| --- | --- | --- |
| [HMD-Poser, CVPR 2024](https://arxiv.org/html/2403.03561) | HMD/controllers plus optional pelvis/lower-leg IMUs; pose and body-shape estimation followed by head-anchored FK. | Shape and learned pose affect accumulated positional error. Its trained configurations and real free-dancing evaluation do not establish seated accuracy for chest/thigh sensors or quaternion-only packets. |
| [GlobalPose, SIGGRAPH 2025](https://arxiv.org/html/2505.05010v1) | Multiple 3D contacts, including pelvis support, coupled with physics and learned local motion. | Authors report limitations for light/sliding contacts, nonhorizontal proxy surfaces, limited height-change training data and ambiguous multiple-contact forces. A full physics optimizer is a larger project than a seated gate. |
| [Ground Reaction Inertial Poser (GRIP), CVPR 2026](https://arxiv.org/html/2603.16233v1) | Four wrist/foot wearable devices combine IMUs with actual insole pressure, with kinematics and physics-controller stages (full-text sections 3.1–3.4 checked). | Pressure adds information about support that orientation alone lacks. It does not validate an IMU-only contact detector or justify claiming pressure-like certainty. |
| [RoSHI, April 2026 preprint / authors' IROS 2026 project](https://roshi-mocap.github.io/) | Nine IMUs plus Aria SLAM; learned body generation guided by observable joint angles, relative bone orientations and temporal consistency. | Useful separation of absolute trajectory and relative orientation constraints. Different hardware, sensor count, calibration, trained prior and data pipeline; the inspected paper does not establish a production real-time latency budget for ChadFBT. |
| [Application-Oriented In-Cabin Human Pose Estimation With Six Wearable IMUs and Sensor-Aware Training, IEEE Sensors Journal, July 2026 early access](https://ieeexplore.ieee.org/abstract/document/11611566) | Seated and confined motions receive domain-specific training, sensor-noise modeling and augmentation. | Publisher abstract checked; full text was not available in this review. Its reported real-field error remains materially larger than its synthetic error. The lesson is domain coverage and sensor realism, not automatic generalization. |

No library API changes are proposed by this report; Context7 was not needed for these research
and geometric-model questions. Any later integration of an SDK, inference runtime or platform
API needs its own current documentation and licensing review. Do not copy model assets or code
without checking their separate licenses and training-data terms.
