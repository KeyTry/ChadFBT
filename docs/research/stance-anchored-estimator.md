# Stance-anchored estimator: design specification

Status: **design + implementation in progress, synthetic validation plus the replay of one
recorded live session (see the dated section at the end).** Nothing in this document is a
demonstrated accuracy result on worn hardware. Every layer has a switch in the app and reports
diagnostics; the components inside a layer are configuration switches or gains that can be
overridden when a recorded session is replayed (the few exceptions are noted in section 4).

Prior art and positioning are in the [literature review](literature-review.md) and the
[SlimeVR source review](slimevr-internals.md). In short: using an absolute position reference
through a kinematic chain to make heading observable is published (legged-robot odometry, UWB- and
GNSS-aided inertial capture, an Xsens patent). The combination here, a headset endpoint with an
*inferred* stationary foot and an increment-based per-segment regression on fused quaternions, was
not found in the literature or in SlimeVR's source, and is treated as an incremental combination,
not a novelty claim.

This document specifies five cooperating layers that sit between calibrated segment
orientations and published tracker targets:

1. `chain` – one shared kinematic chain (head anchor to ankles) with analytic yaw sensitivities.
2. `contact` – per-foot stationary-point hypotheses.
3. `stance_heading` – a small error-state Kalman filter that observes per-sensor yaw error
   (including the common-mode component) from displacement mismatch between the absolute
   headset and the IMU chain while a foot is hypothesised stationary, plus weak anatomical priors.
4. `leg_ik` – contact-consistent root correction, foot lock, flat-foot orientation.
5. `prediction` – per-sensor angular-rate estimation, bounded extrapolation, output velocities.

Frames follow `AGENTS.md`: right-handed standing space, meters, +Y up, −Z forward, XYZW.
A calibrated bone rotation `R_i` maps bone-local axes to standing space: bone +Y points from the
distal joint to the proximal joint (knee→hip, ankle→knee, hip→chest), +X is the body's right
(the flexion hinge axis), −Z is anatomical forward. Segment index order is `Role::CORE`:
`0 Chest, 1 Hip, 2 LeftThigh, 3 RightThigh, 4 LeftCalf, 5 RightCalf`. Foot index `0 = left`
(lateral sign `s = −1`), `1 = right` (`s = +1`).

## 1. Shared chain (`chain.rs`)

With headset eye point `p_head`, headset rotation `R_head`, and dimensions `L`:

```
neck      = p_head + R_head · (0, −L.head_to_neck, +L.eye_forward)
chest     = neck  − (R_chest · Y) · L.neck_to_chest
R_spine   = slerp(R_chest, R_hip, 0.5)
hip       = chest − (R_spine · Y) · L.chest_to_hip
hipJoint_s= hip   + (R_hip · X) · (s · L.hip_width / 2)
knee_s    = hipJoint_s − (R_thigh_s · Y) · L.thigh
ankle_s   = knee_s     − (R_calf_s  · Y) · L.calf
```

`L.eye_forward` (default 0.08 m) is new: the tracked eye point is in front of the neck pivot, so
without it a head pitch/yaw swings the whole inferred body. `+Z` is backward in the head frame.

**Lever vectors.** For foot `s` and segment `i`, `lever[s][i]` is the sum of the chain's bone
vectors (between `p_head` and `ankle_s`) that rotate rigidly with a world-yaw change of segment
`i`:

```
lever[s][Chest]   = −(R_chest·Y)·L.neck_to_chest − ½·(R_spine·Y)·L.chest_to_hip
lever[s][Hip]     = −½·(R_spine·Y)·L.chest_to_hip + (R_hip·X)·(s·L.hip_width/2)
lever[s][Thigh_s] = −(R_thigh_s·Y)·L.thigh
lever[s][Calf_s]  = −(R_calf_s·Y)·L.calf
lever[s][other leg's segments] = 0
head_lever        = neck − p_head            (depends on the absolute headset only)
ankle_s = p_head + head_lever + Σ_i lever[s][i]            (exact identity; unit-tested)
```

The spine split is the first-order sensitivity of the slerp to each end's yaw. If segment `i`'s
true rotation is `Ry(δ_i)·R_i` for a small remaining yaw error `δ_i`, then
`∂ankle_s/∂δ_i = ŷ × lever[s][i]`, where `ŷ × (x, y, z) = (z, 0, −x)`.

`build_frame` must use this same function so the filter's model and the published skeleton
cannot diverge.

## 2. Contact hypotheses (`contact.rs`)

A "contact" here means only: *this ankle is hypothesised to be a stationary point in the room*.
It does not have to be floor support. Per foot, from the chain's ankle positions:

- velocity: exponential moving average (τ ≈ 40 ms) of finite-difference ankle velocity;
- height above the lower ankle: `ankle_s.y − min(ankle_L.y, ankle_R.y)`;
- calf yaw rate: the world-vertical component of `2·log(R_calf(t)·R_calf(t−dt)⁻¹)/dt`, smoothed.

State machine `Swing → Settling → Planted` with hysteresis. Defaults: enter when speed < 0.20 m/s,
height above lower ankle < 0.06 m, |calf yaw rate| < 40 °/s, inputs usable; `Settling → Planted`
after 80 ms continuously satisfied; leave `Planted` when speed > 0.45 m/s, height > 0.10 m,
|calf yaw rate| > 90 °/s, inputs unusable, a time gap > 0.10 s, or an external `force_release`
(filter outlier streak, foot-lock break, headset discontinuity). Time not advancing or going
backwards releases both feet and never manufactures dwell.

The calf yaw-rate gate matters: turning on the spot pivots the planted foot, which moves the
ankle around the ball of the foot and would otherwise be read as a displacement mismatch.

## 3. Stance heading filter (`stance_heading.rs`)

### Observation

While foot `s` is planted its true ankle displacement is zero. Between two keyframes `a → b`
the computed displacement is

```
Δp̂_s = Δp_head + Δhead_lever + Σ_i Δlever[s][i]
```

and, to first order in the remaining yaw errors,

```
0 = Δp̂_s + Σ_i (ŷ × Δlever[s][i]) · δ_i  + noise
⇒  z = −(Δp̂_s)_xz ,   H_i = (ŷ × Δlever[s][i])_xz ,   z = H·δ + n
```

Only *changes* of the levers carry information. A static pose, a head-only turn, or headset
noise with unchanged leg/torso geometry has `H = 0` and cannot move the estimate, whatever the
residual. This is the deliberate difference from anchoring a foot position as a filter state:
slowly varying model bias (eye/neck offset, spine curvature, soft tissue) would then be treated
as white noise and integrated into a false yaw.

Because the head position is absolute, the common-mode yaw of the whole leg chain is observable:
when every leg segment carries the same error δ, `Δp̂ = (Ry(δ) − I)·Δlever_total`. Relative
joint constraints (hinge, range of motion, relaxed-pose priors) are blind to exactly this.

Keyframes must be compared under one set of corrections. Each keyframe stores `p_head`,
`head_lever`, the six levers, the headset rotation and the filter corrections `c_i` at that time;
when forming an increment the stored `lever[s][i]` is first rotated about +Y by
`c_i(now) − c_i(keyframe)`.

### Keyframe policy (per planted foot)

- On `Planted` entry: store a keyframe.
- Let `e = max_i |Δlever[s][i]|_xz` (excitation) and `τ = t − t_key`.
- If `e ≥ 0.03 m` and `τ ≥ 0.05 s`: form a measurement, then re-key.
- Else if `τ ≥ 0.75 s`: form a measurement only if `e ≥ 0.01 m`; re-key either way.
- If the headset moved faster than 4 m/s between consecutive frames, or its timestamp jumped:
  drop both keyframes and ask the contact tracker to release (tracking-space discontinuity).

### Noise budget

```
R = [ 2σ_hmd² + (σ_neck·Δθ_head)² + (σ_creep·τ)² + ρ²·Σ_i |Δlever[s][i]|²_xz ]·I₂
    + ε_len²·Σ_i (Δlever[s][i])_xz (Δlever[s][i])_xzᵀ
```

`σ_hmd = 3 mm` (headset jitter), `σ_neck = 0.04 m/rad` (eye/neck model error per radian of head
rotation between the keyframes), `σ_creep = 5 mm/s`, `ρ = 0.10` (unmodelled motion proportional to
how much the chain moved), `ε_len = 0.08` (relative bone-length uncertainty). The last term is
anisotropic on purpose: a wrong bone length produces a residual *along* the bone's displacement,
a yaw error produces one *perpendicular* to it. The filter therefore trusts the perpendicular
component and largely ignores the parallel one.

### Filter

- Error state `δ ∈ R⁶` (remaining yaw error per core sensor) and covariance `P` (f64).
  The filter owns an accumulated correction `c ∈ R⁶`; after each accepted update
  `c += δ̂`, `δ̂ = 0` (the core applies `c` as a left world-yaw on top of calibration and recovery).
- Initial `P = diag((2°)²)` after calibration or recovery.
- Process noise per second: `q_i = (0.10°)² + (κ·ω_i)²`, `κ = 0.015 √s`, with `ω_i` the smoothed
  magnitude of segment `i`'s angular velocity (rad/s). Drift of a 6-axis fusion grows mostly
  while the sensor rotates (gyro scale and g-sensitivity), so uncertainty should too.
  `P_ii` is capped at `(45°)²`.
- Update: `S = HPHᵀ + R`, gate on `zᵀS⁻¹z ≤ 9.21` (χ², 2 dof, 99 %). Joseph-form update.
  Per-update clamp: if any `|δ̂_i| > 1.0°`, scale the whole state step so the largest is 1.0°.
  Three consecutive gated-out measurements on a foot force-release that contact.
- Skip a foot's measurement when any segment in its chain is unusable (stale, firmware-held,
  invalid) at either keyframe.

### Relative priors (pseudo-measurements, each individually switchable)

Heading of a segment for these priors is the horizontal direction of its lateral axis
`a = R·X`: `ψ = atan2(−a.z, a.x)`; defined only when `a.x² + a.z² ≥ 0.5`.

| Prior | Residual | Gate | Deadband | σ | Rate |
| --- | --- | --- | --- | --- | --- |
| Knee hinge (per leg) | `ψ_calf − ψ_thigh` | flexion 25–120°, ≥ 8° flexion change in the last 2 s | 5° | 15° | 2 Hz |
| Pelvis vs thighs | `ψ_hip − mean(ψ_Lthigh, ψ_Rthigh)` | hip and both thighs within 30° of upright | 10° | 20° | 1 Hz |
| Chest vs pelvis | `ψ_chest − ψ_hip` | both within 30° of upright | 8° | 20° | 1 Hz |

For residual `r` beyond the deadband (`r_eff = sign(r)·(|r| − deadband)`), the pseudo-measurement
is `−r_eff = (δ_a − δ_b) + n`. These are priors, not measurements: a held anatomical twist looks
identical to drift. They exist so that torso heading follows the absolutely observed legs when
the torso is upright (its levers are vertical and carry no displacement information), and they
are bounded by their deadbands. All three default on in the app and can be disabled.

### What this cannot do

- No motion, no information: a user who is perfectly still gets no correction (and needs none
  for positions; orientation drift accrues until they move).
- A foot that really slides slowly below the contact thresholds, with the rest of the chain
  moving, biases the estimate until the gate or the anisotropic budget rejects it.
- Mounting-axis error is partly indistinguishable from yaw error for motion in one plane. The
  filter then picks the yaw that makes the chain reproduce the measured head displacement, which
  reduces foot sliding but is not an anatomical measurement.
- Requires a live absolute headset pose; it is inactive in desktop preview.

### Observability, stated plainly

- **Common mode during gait.** In stance the thigh and shank levers move nearly parallel, so one
  step constrains the weighted sum of their yaw errors, not each separately.
- **Relative mode during squats.** In a symmetric squat the hip stays over the ankle, the thigh
  and shank levers change by equal and opposite amounts, and the displacement residual observes
  only *thigh minus shank* yaw, the same quantity as the knee hinge prior. A squat alone cannot
  tell which of the two sensors drifted; the filter splits the correction by prior covariance.
  Mixed motion (steps plus bends) resolves both modes.
- **Upright torso.** Chest and pelvis levers are vertical when standing, so their yaw is not
  observed by displacement at all and is held only by the deadbanded priors. Torso bends make it
  observable. [Acceleration heading](acceleration-heading.md) is the planned direct observation.
- **Spine split.** The ½/½ chest–pelvis split of the spine lever is exact only when the two
  rotations agree; the error grows by roughly `0.115·φ·L.chest_to_hip` per radian of yaw error,
  with `φ` the chest–pelvis angle (about 2.4 cm/rad at 30°). It is absorbed by the noise budget.

### Refinements found necessary in synthetic testing

These were added during implementation because the first version failed specific scenarios:

- **Pivot release.** A foot is released once its calf has turned more than 15° about vertical
  since settling, or its ankle's vertical speed exceeds 0.15 m/s. Without this, pivots slower than
  the yaw-rate gate stayed "planted", and because pivot error has the same sign for left and right
  turns it accumulated (15° of false correction after ten 90° turns; about 2° with the rule).
- **Pivot noise term.** The budget adds `0.12 m/rad ×` the calf's turn between keyframes.
- **Self-estimated process noise.** The filter uses the larger of the caller's angular speed and
  its own estimate, so a caller passing zeros cannot make it overconfident.
- **Prior gating.** Priors are χ²-gated at 6.63 (1 degree of freedom).
- **Entry gate versus large errors.** The contact entry gate uses the computed ankle speed, which
  a yaw error inflates by roughly `error × chain speed`. Uncertainty-aware gate allowances and
  innovation-driven covariance inflation address recovery from large sudden errors such as a
  strap rotating around the thigh.

## 4. Contact-consistent leg IK (`leg_ik.rs`)

Top-down forward kinematics puts every upstream error (neck model, spine, bone lengths) into
the feet. With stationary-point hypotheses available the residual can be distributed instead.

1. **Anchor.** When a foot becomes planted, its anchor is the FK ankle position. While planted,
   its horizontal coordinates creep toward FK with τ = 4 s only for the part of the error beyond
   a 1 cm deadzone, so slow systematic error never accumulates into a pop. Height approaches a
   target with τ = 0.3 s: the floor ankle height for an upright leg whose FK ankle is within 3 cm
   of it, the FK height beyond 8 cm, blended between and weighted by the standing-floor prior
   below. With that prior at zero the anchor height follows FK, so a lock need not fix the
   foot's vertical coordinate.
2. **Root correction.** `c_target = mean over planted feet of (anchor_s − fk_ankle_s)`, scaled
   by `root_gain = 0.6`, clamped to 5 cm, low-passed with τ = 60 ms, and decaying to zero with
   the same τ when nothing is planted. The pelvis moves by `c`; the chest moves by
   `c · L.neck_to_chest / (L.neck_to_chest + L.chest_to_hip)`; the neck and head do not move.
3. **Foot lock.** After root correction, a planted ankle is steered to its anchor when the
   remaining error is ≤ 7 cm (otherwise the lock breaks and the contact is force-released). The
   knee is recomputed by two-bone IK from the corrected hip joint toward the anchor, using the FK
   knee as the pole so measured bend direction is preserved. The reach is capped at
   `L.thigh + L.calf` with no margin (the configured reach margin protects minimum reach only),
   softened near full extension, and limited by the measured-flexion guard described
   below, so the ankle sits on the anchor unless the bone lengths or that guard stop it along
   the hip–anchor line. On release the positional offset decays with τ = 100 ms.
4. **Foot orientation without a foot IMU.** Blend from "rigid with the calf" to "flat, heading
   only" by the planted weight (ramps over 80 ms) times a calf-tilt weight (1 below 40° from
   vertical, 0 above 60°, smoothstep between) so kneeling does not fold the foot through the
   shin. Flat heading is the calf's lateral-axis heading captured at plant time, following the
   current value with τ = 1 s. A measured, direction-certified foot IMU always wins.
5. **Floor clamp (optional, default on).** Ankles are kept at or above
   `L.ankle_to_sole − 1 cm` in standing space. A wrong SteamVR floor makes this wrong; hence the
   switch. *Superseded 2026-11-15:* the ankle-only clamp bent the knees instead of moving the
   body; it is replaced by the support-foot floor constraint described in
   [Findings from the first live session](#findings-from-the-first-live-session-2026-11-15).

Rotations of measured segments are never altered by IK. Positions remain `Evidence::Inferred`.

**Seated and lying changes, 2026-11-16.** Status: synthetic regressions, one synthetic
end-to-end probe and one replay with a short seated segment; **not validated on a worn body.**
Details and evidence: [investigation](seated-ik-investigation.md). Current behaviour
(`leg_ik.rs` is the source of truth):

- **Standing-floor prior** `w = max(thigh taper, bowed) × calf taper`. The tapers are
  `1 − smoothstep` of the bone's tilt from vertical: calf full through 40° and zero at 60° (the
  flat-foot thresholds), thigh full through 60° and zero at 80° (`support_thigh_full_tilt_rad`,
  `support_thigh_zero_tilt_rad`; both ≥ π ablates the thigh taper). `bowed` is a smoothstep of
  the mid-spine's forward lean from 20° to 35° (`squat_bow_start_tilt_rad`,
  `squat_bow_full_tilt_rad`; both above π/2 ablates it). `w` scales the `SupportFloor`
  pull-down, the planted anchor's floor attraction and the flexion guard. It is a heuristic,
  not a contact classifier. A chair and a parallel squat have the same leg orientations; a
  squat balances the torso forward over the feet, so a folded leg under a forward-bowed torso
  keeps the prior and an upright or reclined seated wearer loses it. Cost: leaning well forward
  on a seat with the feet off the floor brings the pull-down back, and a perched pose with a
  forward lean and the feet on a rung is still pulled down. Contact
  eligibility, horizontal foot locks, below-floor raising and measured orientations do not
  depend on it.
- **Pull-down budget counts bows only.** `budget = 2 cm + 1.0·bow_vertical`, where
  `bow_vertical` is `neck_to_chest·(1 − cos θ_chest) + chest_to_hip·(1 − cos θ_spine)` with each
  term weighted 1 for a forward or sideways tilt and fading linearly to 0 for a straight
  backward lean. A recliner or lying supine earns nothing beyond the base.
- **Unexplained gaps fade the pull.** `pull = min(gap, budget, pelvis room) · w · explained`,
  `explained = 1 − smoothstep(budget, 2·budget + 5 cm, gap)`: a foot further above the floor
  than the budget can explain (a jump, a box or step, a stool, a bed) is not pulled, instead of
  receiving the whole budget.
- **No margin at full extension.** The previous maximum reach of `thigh + calf − 1 mm` forced
  roughly 2 cm of knee bend on every locked straight leg, because a leg shortened by `d` moves
  its knee by about `√(thigh·d)`.
- **Soft full extension.** The shortening `d` from full extension is replaced by the cubic
  deadband `d³/(d² + s²)`, `s` = 5 mm, pinned to the measured reach: a target at the FK distance
  reproduces the FK leg, sub-millimetre noise barely moves a straight knee, and a real bend
  converges to the plain solution.
- **Measured-flexion guard** (`knee_flex_slack_rad`, 1°). The guarded reach keeps the knee's
  interior angle within 1° of the one the thigh and calf sensors measure; the rest slides the
  ankle along the hip–anchor line. The applied reach is `free + (guarded − free)·(1 − w)`, so an
  upright standing leg is unguarded as before. Reason: with the square-root relation above,
  4 mm of compression raises a straight knee by about 4 cm, so any headset motion toward locked
  feet lifted the legs of a lying wearer. 1° caps the rise of a straight 0.45 m thigh at about
  4 mm.
- **Lock slack counts bows only.** The torso term added to the root-correction limit, the
  lock-break radius and the anchor-creep deadzone is `0.3·bow_horizontal`: the horizontal reach
  `neck_to_chest·sin θ_chest + chest_to_hip·sin θ_spine` with the same backward-lean fade as the
  budget. A supine or reclined torso earns none, so a slowly moved leg is not held to a stale
  anchor.
- The reach margin is capped at a quarter of the reach range inside the solve.

Not addressed: no seat or bed height is modelled and no pelvis position is measured; the torso
is two rigid rods without a pelvis segment, so posterior pelvic tilt or a slouch can leave the
seated pelvis a few centimetres low in the raw chain; `normalize_chain_height` puts any
leg/torso proportion error into `chest_to_hip`, which standing hides and sitting does not;
pull-down transients can occur while sitting down, as the leg passes through the tapers;
side-lying and prone torsos still earn bow budget and slack, so only the leg tapers protect
them. Every threshold here is an engineering choice, not a
value from a paper or from worn tuning. Switches: the thigh taper and the squat-bow term by
their thresholds, the guard by a large slack; the calf taper (shared with the flat foot), the
backward-lean rules, the `explained` fade and the soft extension have none.

## 5. Prediction and velocities (`prediction.rs`)

- Per sensor, angular velocity from consecutive measured quaternions
  `ω = 2·log(q_k·q_{k−1}⁻¹)/Δt` (shortest arc, valid for 2 ms ≤ Δt ≤ 50 ms), EMA τ = 25 ms.
- At solve time, each orientation is extrapolated by `Δ = min(now − t_last + lead, 35 ms)` only
  if a sample arrived within the last 60 ms; otherwise ω is treated as zero. The firmware
  suppresses packets when rotation is unchanged, so silence means "still", not "keep turning".
- Target linear velocity is an EMA (τ = 30 ms) finite difference of target position; angular
  velocity is the owning segment's ω rotated into standing space. Both are sent to the driver so
  SteamVR can extrapolate tracker poses to the client's predicted display time instead of showing
  them one pipeline delay late.

## 6. Required synthetic regressions

A deterministic articulated-body generator (`synthetic.rs`) produces ground-truth segment
rotations and headset poses for scripted motions with planted feet solved by two-bone IK, then
injects per-sensor yaw error, headset noise, bone-length error and eye-offset error.

| Scenario | Requirement |
| --- | --- |
| Chain identity | `ankle = p_head + head_lever + Σ lever` to 1e-5 for random poses |
| Sensitivity | Finite-difference yaw perturbation matches `ŷ × lever` |
| Walking, 20° common-mode leg drift | Converges; report time to < 3° |
| Squats, one calf drifting 15° | Converges with knee prior; report |
| Weight shifting while standing | Converges slowly; report |
| Standing still with drift | No correction applied (unobservable) |
| Head-only turns, 4 cm eye-offset error | False correction < 1° |
| 5 % bone-length error | Residual yaw error stays < 3° |
| Slow real foot slide | False correction bounded; report |
| Turn on the spot with pivoting feet | False correction bounded; report |
| Headset jump of 0.3 m | No correction from the jump |
| Stale / held / NaN inputs | No update, finite outputs |
| 10 min, independent random-walk drift during activity | Report RMS yaw error with filter on vs off |

Passing these shows the implementation matches the model. It does not show the model matches a
human body.

## Findings from the first live session (2026-11-15)

Status: **the changes below are checked on a replay of one recorded session and on synthetic
regressions. They have not been tried live.** A replay has no ground truth; it shows what the
then-current code computed from real inputs, not where the user's body was. **This section is a
dated record of the 2026-11-15 code and is written in that day's present tense.** Its calf-only
support rule, its budget for any torso tilt, its full-budget pull on large gaps (the 2 cm pull on
upright jumps) and its numerical tables predate the 2026-11-16 changes described at the end of
section 4, which is the current behaviour. The replay figures have not been regenerated with the
current code.

### What was observed

One live session (six IMUs, headset; calibration by movement finished at t = 259.9 s, live
until 493.8 s) was replayed from its recorded inputs
(`chadfbt --replay FILE --dump OUT.jsonl`, synthetic solve times from the live pacing rule).
The user reported that bending down and squatting pulled the calves and thighs up and that the
avatar's feet sometimes left the ground while theirs stayed planted.

- **The head-down chain is too short in bends.** Standing, the modelled ankles sat 1.5 cm below
  the floor (the stored profile's chain was 2 cm taller than its eye height: an older
  default chest-to-hip length). In bows the modelled hip rose 8–12 cm above its
  standing height and the ankles rose 15–22 cm while the feet were planted; over all live frames
  the chain was 9–16 cm too short (median, growing with how far the head dropped). No choice of
  segment lengths within the same structure removes it. The horizontal error (ankles move back,
  not forward) points at the torso: in deep flexion the torso trackers over-read the tilt of the
  hip-joint-to-neck line (hip tracker 78–94° where about 57–62° is needed). A flexed spine is not
  a rigid rod and waist-worn trackers ride on soft tissue.
- **The foot lock let go when it was needed.** The top-down ankle appeared to move at up to about
  0.5 m/s while the torso rotated, so contacts were released ("swing" on both feet) during bows
  and the 7 cm lock radius broke.
- **Torso model error became chest yaw.** The stance filter turned the chest by −23° within four
  minutes (real drift of these IMUs is of the order of 0.2°/min).

### What was changed

All parts are reported in `EstimatorStatus`. Switches: the pull-down (`support_floor_enabled`) and
the raise (`floor_clamp_enabled`) separately, the torso slack by `torso_slack_gain = 0` (since
2026-11-16 the slack counts forward and sideways bows only), the
legs-only filter by `estimate_torso_yaw`; all through configuration (replay `--estimator`), not
from the app's UI, which switches whole layers.

1. **Support-foot floor constraint with torso compliance** (`leg_ik::SupportFloor`, run before
   the contact layer so contacts and foot-lock anchors see the corrected ankles; the stance filter
   keeps the uncorrected chain because it only uses horizontal displacement). The lowest usable
   ankle of the head-down chain is the support foot. Above the floor, pelvis and both legs move
   down rigidly (chest by the spine fraction, neck and head fixed) until it rests at
   `ankle_to_sole`, by at most a compliance budget
   `0.02 m + 1.0·[neck_to_chest·(1 − cos θ_chest) + chest_to_hip·(1 − cos θ_spine)]`, the height
   a rigid torso model loses with flexion. As of 2026-11-15 it applied only while the support
   shank was within 40° of vertical (smoothstep to zero at 60°); the thigh taper, bow-only budget
   and fade-out were added on 2026-11-16 (section 4). It never lowers the pelvis below 0.15 m,
   and is low-passed with τ = 60 ms. Below the floor the body is raised at once by the full amount (up
   to 2 m; a forward fold past horizontal put the replayed ankles 0.67 m below the floor). An
   upright torso gets about 2 cm, so jumps and lifted feet still lift.
2. **Horizontal consistency in bends.** The torso's horizontal uncertainty
   `k·(neck_to_chest·sin θ_chest + chest_to_hip·sin θ_spine)` with `k = 0.3` is added to the
   root-correction limit (5 cm), the lock-break radius (7 cm) and the horizontal anchor-creep
   deadzone (1 cm). `k = 0.3` makes the lock radius about 24 cm in a 75° bow, which covers the
   2–15 cm horizontal ankle errors seen in the bows. A planted anchor's height is drawn to the
   floor while its ankle is within 3 cm of it (follows the model beyond 8 cm, τ = 0.3 s; since
   2026-11-16 only in proportion to the standing-floor prior), so neither a
   swing offset captured at plant time nor a torso roll error that lifts one leg leaves a planted
   foot hovering. The contact tracker grants a torso allowance
   `0.5 · torso swing rate · torso length` (capped at 0.4 m/s) on its speed and vertical-speed
   gates, but only under a **double-support cue**: both feet in contact and the vector between
   the ankles (which depends on pelvis width and leg rotations only) changing slower than
   0.3 m/s (on the replay, 0.15 m/s left 16–26 % of the frames of three bow windows unlocked;
   0.3 m/s kept 92–100 % locked).
3. **Stance heading filter: legs only.** Chest and hip yaw are no longer estimated from
   displacement (`estimate_torso_yaw = false`); they are held by the deadbanded priors only.
   Chest/hip lever changes enter the noise budget with ρ_torso = 0.5 (legs keep 0.10), and an
   increment whose torso lever change exceeds its leg excitation is not used: the torso error is
   correlated from one increment to the next, so many small discounted increments would still add
   up (without this gate, ρ_torso alone kept 20 synthetic flexed-spine bows to 0.6° of false leg
   correction; with it, 0°). Corrections are bounded at 30° (legs) and 12° (chest, hip), reported
   in `correction_limited`.
4. **Body dimensions.** On profile load and whenever the eye height is set, `chest_to_hip` is
   adjusted so the modelled standing chain equals the eye height (2 cm shorter for the session's
   profile); if that would leave its valid range, nothing changes and the app says so.
5. **Replay tooling.** `chadfbt --replay FILE --dump OUT.jsonl [--every N] [--estimator JSON]`
   re-solves an input-only recording with the current code, applying recorded events in file order
   (calibrate, movement calibration, recovery, rebase, configuration, clear) with the same calls
   as the live worker; the determinism check for recordings with frames is unchanged.

### Replay of the session, before and after

Before = the code at the time of the session; after = all of the above with defaults. Live
frames after calibration; heights are the lowest published ankle above its floor height
(`ankle_to_sole`).

| Measure | Before | After |
| --- | --- | --- |
| Bent (head down > 30 cm, both shanks < 50°): median / 95th percentile | −1.0 / 14.6 cm | −1.0 / 0.6 cm |
| Standing (head down < 5 cm, shanks < 20°): median / 95th percentile | −1.0 / 0.0 cm | −0.4 / 0.0 cm |
| All frames with shanks < 50°: float > 3 cm | 7.0 % | 0.4 % |
| Bow 299–300.5 s: hip up / back; ankles up / horizontal; locked | +12.4 / 25.3; 15.1 / 13.6 cm; 56 % | +0.3 / 18.0; 0.0 / 1.2 cm; 100 % |
| Bow 422–423 s: hip up / back; ankles up / horizontal; locked | +9.2 / 26.1; 13.8 / 8.5 cm; 43 % | +0.7 / 21.9; 0.0 / 0.6 cm; 100 % |
| Bows 429–430 s and 433–434 s: locked | 52 %, 49 % | 100 %, 100 % |
| Corrections at t = 480 s (chest, hip, thighs L/R, calves L/R) | −23.2, −0.6, −4.0, −7.3, −3.5, −4.5° | −2.7, +6.6, +7.3, +2.6, +10.3, +4.1° |
| Solve time median / 99th percentile (this replay, one machine) | 9 / 40 µs | 7 / 16 µs |

The hip still moves back in a bow (a real hip does). Knee lifts while standing upright
(302.7–304.4, 362.8–372.3 and 415.6–420.8 s) still lift the foot (46.7, 73.6 and 63.4 cm
above the floor) and keep the support ankle within −0.9 to +1.3 cm of the floor.

Synthetic regressions (`tests/support_floor.rs`, generated with the estimator's own model; the
torso trackers over-read 60° bows by +12° chest and +30° hip, partly sideways): published
ankles stay within 0.7 cm of the floor and 0.1 cm horizontally with both feet locked throughout,
against 13.6 cm of head-down float and 15.8 cm / 11.2 cm with the previous estimator; twenty such
bows leave every stance correction at 0.00° (previous filter: chest 10.5°, hip 8.4°); upright
20 cm jumps still publish 18.1 cm of flight; a 30 cm knee lift publishes 30.0 cm with the support
ankle within 0.1 cm of the floor; a chain 6 cm longer than the body raises the pelvis instead of
bending the knees; the covered horizontal-shank long-sitting/kneeling cases and low-pelvis lying
case receive no pull-down. This does not cover ordinary chair sitting with vertical calves or a
pelvis supported above the floor by a bed.

Stance-filter regressions (`examples/stance_heading.rs`; the previous model is kept as a variant
and reproduces its earlier numbers exactly): unchanged in substance for walking drift, sudden
errors, weight shifts, head turns, bone-length error and headset jumps. Changed: slow foot slide
at 0.15 m/s 3.36° → 4.28° of false correction (the slower slides improved, 1.43° → 0.95° and
0.48° → 0.24°); squats with one drifting calf 7.64° → 9.84° final error (relative mode, weakly
observable either way); 10-minute random-walk activity: legs about the same (seed 7 RMS 2.04,
2.03, 2.38, 1.65° → 2.08, 2.14, 2.76, 1.81°), torso worse by design (chest 2.59 → 4.62°, hip
3.74 → 5.60°) because a 0.5°/√s synthetic torso random walk, far above real drift, is now held
only by the priors; turns on the spot improved (2.06° → 0.73°). With the recovery mechanisms
switched off (an ablation), a sudden 20° leg error no longer converges (the previous model's torso
columns helped it); with them on it converges in 9.8 s as before.

### What remains unverified

- **None of this has been worn.** The replay shows that the published feet now stay on the floor
  through the recorded bows; it cannot show that the published hip and knees are where the user's
  were, nor that the user will judge it better.
- **The leg yaw is not validated.** Leg corrections at t = 480 s changed sign between the two
  versions (both of the order of ±10°). There is no ground truth for leg heading in this
  recording; how much of either is real calibration error and how much is absorbed model error is
  unknown.
- **The compliance budget is a model assumption** (the support foot is on a flat floor at the
  SteamVR floor height; the missing height is in the torso). It does not cover model error with
  an upright torso: sitting on a chair still left the replayed feet 3–11 cm above the floor
  (0.4 % of frames). A wrong SteamVR floor makes the constraint wrong. Near-horizontal thighs
  with vertical calves do not prove a planted foot; chair sitting with raised feet remains
  ambiguous. Bed support is unmeasured. The 2026-11-16 changes discount floor attraction and
  keep the measured knee angle in these configurations without identifying a seat or bed, and
  live seated/lying validation is still pending.
- **The constants are fitted to one session of one user** (k = 0.3, gain 1.0, ρ_torso 0.5,
  double-support 0.3 m/s, torso allowance gain 0.5). Their transfer to other bodies, straps and
  motions is untested.
- **Torso yaw is now held only by deadbanded priors**, i.e. within about 8–10° of the legs and
  only while upright. Large real torso drift would not be corrected by displacement.
- The replay approximates live timing (synthetic solve times); 1,669 recorded firmware-held
  ingest outcomes differ from the live ones (the recording keeps at most one accepted held token
  per second per sensor).
