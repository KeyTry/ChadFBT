# Acceleration heading: torso yaw from headset acceleration (shadow estimator)

Status: **research / diagnostics only, synthetic validation only.** Nothing here is a demonstrated
result on worn hardware. The estimator never modifies tracking state; it reports a yaw candidate,
its uncertainty and the reasons it withholds one, and the caller decides what to do with it.

Code: [`crates/tracking-core/src/accel_heading.rs`](../../crates/tracking-core/src/accel_heading.rs)
(generator in `accel_heading/synthetic.rs`), regressions in
[`tests/accel_heading.rs`](../../crates/tracking-core/tests/accel_heading.rs), scenarios in
[`examples/accel_heading.rs`](../../crates/tracking-core/examples/accel_heading.rs). This is
lead 1 of [slimevr-internals.md](slimevr-internals.md).

## Why

The [stance-anchored filter](stance-anchored-estimator.md) observes a segment's yaw only through
the horizontal displacement its lever arm produces. An upright torso has nearly vertical levers,
so chest and pelvis yaw are almost unobservable there and fall back to anatomical priors. But the
chest and pelvis *translate* nearly rigidly with the head while stepping, walking, bobbing or
dancing, and the headset's acceleration is known in the absolute room frame. Rotating the chest
IMU's acceleration into the room with its drifting orientation and asking which yaw best aligns
it with the headset's acceleration observes the chest yaw directly, independently of levers.

## Model

Frames follow `AGENTS.md`: room frame meters, right-handed, +Y up, −Z forward, XYZW quaternions.

```
world_imu(t) = S(t) · C · a_body(t)           a_body: gravity-removed, raw IMU axes
S(t)         = Ry(ε) · S_true(t)                ε: constant yaw error over the window
world_ref(t) ≈ world_imu_true(t − L)            L: lag, IMU stamped later than the reference
```

`C` is a fixed unknown rotation from raw accelerometer axes to the quaternion's axes. SlimeVR
firmware 0.7.2 rotates the quaternion but not the acceleration by the board mounting offset
(0.7.4 rotates both, so `C = I`). The search set is the 24 axis-aligned proper rotations
(`frame_candidate(i)`, index 0 = identity).

### Signal path

1. **Grid.** Both streams are linearly interpolated onto a uniform 100 Hz grid (never across a
   gap > 0.1 s). The reference grid spans `window + settle` (6 s + 1 s); the IMU grid is longer
   by the lag search range (−20…+80 ms = 11 steps).
2. **Reference acceleration** by centred differences (zero phase): `(v(t+h/2) − v(t−h/2))/h`
   from velocity when supplied, else `(p(t+h) − 2p(t) + p(t−h))/h²`. Speeds above 6 m/s are a
   tracking discontinuity and break the segment.
3. **IMU channels.** Acceleration `k` is rotated with rotation `k − 1` (firmware 0.7.2 pairs a
   rotation with the previous sample's acceleration; configurable).
4. **Band-pass** 0.5–6 Hz: causal 2nd-order Butterworth low-pass then high-pass, identical on
   both streams (equal phase response), started in steady state at each segment start so a
   constant bias produces no step, and the first second of each segment is discarded.
5. **Gates** on the grid: reference angular speed > 60 °/s, IMU angular speed > 90 °/s (supplied,
   or differenced from the quaternions), raw IMU |a| > 30 m/s², low-passed reference |a| >
   30 m/s², missing coverage. Every rejected sample also invalidates ±0.2 s around it because
   the filter spreads the disturbance. Pairs are kept only where the reference sample and the IMU
   samples at *every* searched lag are valid, so all lags are scored on the same set.
6. **Effective samples** `N_eff = valid × 2·(6 − 0.5)/100` — samples at the band's Nyquist rate.
   This is a heuristic count for the noise, not a test statistic. At least 30 are required.

### Frame identification (vertical, yaw-free)

`e_y · S · C · a = (Sᵀe_y) · (C·a)`: the vertical component does not depend on the yaw of `S`.
It is also linear in `M = u·aᵀ` with `u = Sᵀe_y` (world up in sensor axes):
`y_C = ⟨C, M⟩` (Frobenius). Filtering and interpolation are linear and `C` is constant, so the
nine channels of `M` are filtered once and every candidate's band-passed vertical signal is a
fixed combination of them. Per lag `l`, with `r_l = Σ y_ref·m` and `G_l = Σ m·mᵀ` (9 × 9):

```
agreement(C, l) = 2·vec(C)·r_l / (Σ y_ref² + vec(C)ᵀ G_l vec(C))     (Lin's concordance)
```

Concordance, not Pearson correlation: for nearly 1-D vertical motion every candidate whose
vertical axis has a positive projection on the true one correlates ≈ 1 regardless of gain.
Concordance is 1 only for identical signals.

**Ambiguity.** Two candidates differing by a rotation about the axis that is world-vertical in
sensor coordinates produce the same vertical signal if the sensor never tilts. Rather than
thresholding a tilt angle, the estimator measures it in the window's own data: the margin that
*noise-free* data would give between anchor `A` and alternative `B` is
`sep = (s_A − s_B)ᵀ G (s_A − s_B) / (s_Aᵀ G s_A + s_Bᵀ G s_B)`; noise shrinks every agreement
alike, so `sep` is scaled by the best agreement in the window. Candidates with scaled `sep` below
the required margin (0.05) form `A`'s ambiguity set. Then:

- **identified** if the best agreement ≥ 0.6, it beats every candidate *outside* its ambiguity
  set by ≥ 0.05, and the set has one member — latched;
- **FrameAmbiguous** if the margin holds but the set has several members (reported);
- **FrameUnidentified** otherwise.

A latched frame persists across windows (it is hardware) and is dropped only when contradicted:
a distinguishable alternative beats it by ≥ 0.05, or passes 0.6 while it does not.

**Hints.** `set_frame_hint(i)` supplies `C` identified elsewhere, e.g. on a thigh that swings
through large tilts against its forward-kinematics position. A hint is latched only when it
passes the same test anchored on itself (agreement ≥ 0.6 and ≥ 0.05 over every alternative
outside its own ambiguity set), and is rejected when contradicted. **A wrong hint inside the
ambiguity group cannot be rejected from vertical data**: for an upright sensor it acts like a
world yaw of 90°, 180° or 270°. The only guard is the plausible-yaw bound (60°), which a true
error of 40° plus a −90° wrong frame (−50°) would pass.

`allowed_frame_candidates` restricts the search. With firmware ≥ 0.7.4 use `[0]` (then the
estimator only verifies `C = I`). If the board's mounting option is known to rotate about the
chip Z axis, `frame_candidates_fixing_axis(Z)` (4 candidates) makes an upright chest with a
board lying flat on it (chip Z horizontal) identifiable without a hint, because that subset meets
the vertical ambiguity group only in the true frame.

### Lag

The candidate in use is scored at 11 coarse lags (10 ms steps); a new coarse lag replaces the
latched one only if it improves agreement by more than 0.002 (hysteresis). A parabola through
the neighbouring scores refines it by up to ±half a step, and the IMU horizontal signal is
interpolated at that fractional offset. Refinement matters: for elliptical horizontal motion a
residual time offset is a phase shift, and for circular motion a phase shift *is* a rotation, so
an uncorrected 5 ms at 2 Hz could bias yaw by ~3.6°.

### Heading (horizontal 2-D Procrustes)

With reference `r = (x, z)` and IMU `m = (x, z)` band-passed and aligned:

```
A = Σ (r_x m_x + r_z m_z)        B = Σ (r_x m_z − r_z m_x)        yaw = atan2(B, A)
ρ = √(A² + B²) / √(Σ|r|² Σ|m|²)  gain = √(A² + B²) / Σ|m|²
σ = √( (1 − ρ²) / (2·N_eff·ρ²) + σ_floor² )      (σ_floor = 1° asserted, σ ≤ π)
```

`B = Σ cross_y(m, r)` as in the original sketch. With glam's `Ry(θ)`
(`x' = x cos θ + z sin θ`, `z' = −x sin θ + z cos θ`), `yaw` maximises `Σ r · Ry(yaw) m`, so it is
the correction to **add** as a left world yaw: `Ry(yaw) · S ≈ S_true`, `yaw ≈ −ε`. A regression
checks that `Ry(yaw)·S` re-aligns every supplied orientation within 3°.

`σ` is the Cramér–Rao approximation for the phase of a coherence estimate: it tends to π as the
correlation vanishes and is ~`σ_noise / (√N_eff · rms)` when it is high. It does not grow for 1-D
motion, because 1-D motion still observes the yaw (direction plus sign); the scatter anisotropy
is reported instead. The 1° floor stands for unmodelled systematic error and is asserted, not
estimated; in clean synthetic runs it dominates `σ`.

**A Candidate requires, in the same window:** vertical RMS ≥ 0.3 m/s² and the frame in use
agreeing ≥ 0.6 vertically (a per-window rigidity check, also for latched frames); horizontal
reference RMS ≥ 0.3 m/s²; `ρ ≥ 0.6`; `0.5 ≤ gain ≤ 2`; `σ ≤ 6°`; `|yaw| ≤ 60°`; both streams'
newest samples ≤ 0.5 s old.

Statuses: `InsufficientData`, `LowExcitation`, `FrameUnidentified`, `FrameAmbiguous`,
`FrameHintRejected`, `FrameIdentifiedOnly` (leg sensors, below), `PoorCorrelation`,
`ExcessiveCorrection`, `Candidate`. The report also carries every candidate's agreement and lag,
the ambiguity set, tilt spread (RMS angle of the sensor's up axis about its mean), per-gate
rejection counts, `N_eff`, window coverage and the fit even when withheld.

### Leg sensors

A thigh or calf does not translate with the head, but its forward-kinematics position gives a
vertical reference that is yaw-independent (rotating a bone about +Y does not change its vertical
extent). Its horizontal part is computed from the sensor's own drifting orientation, so matching
it would be circular. With `horizontal_reference = false` the estimator stops at
`FrameIdentifiedOnly`; the identified `C` can then be handed to the torso sensors of the same
hardware as a hint. Leg gates need raising (thighs swing at well over 90 °/s).

## Explicit assumptions

1. **Rigid translation** of the reference point and the sensor within 0.5–6 Hz. Head rotation
   about the neck violates it for the eye point; pass the chain model's neck pivot
   `p_head + R_head·(0, −head_to_neck, +eye_forward)`. Torso flexion makes the head move more than
   the chest (a gain mismatch, tolerated by the gain gate).
2. **Tilt of `S` is correct**; only yaw is wrong, and constant over the 6 s window.
3. **`C` is one of the 24 candidates** and fixed. A negated acceleration convention is improper
   and not representable; it would show as no candidate agreeing.
4. **Gravity is removed**; bias and gravity-removal error are slow in the room frame. Fast torso
   rotation moves them into the band.
5. **Constant lag** within −20…+80 ms, the same for vertical and horizontal components;
   receipt-time jitter small compared with the motion period.
6. **One room frame** for reference and `S`; a recentre requires `reset()`.

## What it cannot do

- **Standing still** (or any motion below ~0.3 m/s² band-passed RMS): no information;
  `LowExcitation`. Drift accrued meanwhile is seen only once the user moves.
- **Head moving independently of the torso** — looking around, nodding, talking with the head:
  fast rotation is gated out entirely (InsufficientData at 30° / 0.8 Hz); slow rotation is
  tolerated only to the extent the eye-to-neck model is right (see results). Translation of the
  head relative to the chest (neck flexion, leaning the head) is not detected by the gates and
  acts as correlated contamination.
- **Sitting, lying, kneeling with only upper-body motion**: little shared vertical motion, and
  the torso–head rigidity premise fails; expect `LowExcitation`/`PoorCorrelation`.
- **Turning on the spot**: rotation without translation carries no information.
- **Upright torso with unknown `C`**: `FrameAmbiguous` until a verified hint, a chip-axis
  restriction or a period of torso tilt; a wrong in-group hint cannot be rejected vertically.
- **Leg sensors**: frame and lag only.
- **Pelvis sensors**: same model, but the pelvis rotates and sways relative to the head during
  gait more than the chest does. Untested.
- **Time-varying yaw error inside the window** is averaged, not tracked.

## Synthetic results

Toy rigid body (hip root, torso with lever arms to chest sensor 0.35 m and neck 0.55 m, head on a
neck pivot with a 0.10/0.09 m eye offset), motion = sums of sinusoids, default chest mount
`Ry(20°)·Rx(−12°)·Rz(7°)`, `C` = candidate 13, yaw error +25°, IMU 100 Hz with ±0.5 ms sampling
jitter, 20 ms arrival lag plus uniform 0–4 ms jitter, acceleration one sample old, 0.3 m/s²
Gaussian noise per axis, headset 200 Hz with 0.3 mm position / 5 mm/s velocity noise, neck-pivot
reference with a (0, 1, 2) cm model error in head axes (2–3 cm where noted), torso tilt amplitude
25°, head yaw 10° at 0.5 Hz. 20 s runs,
report every 0.5 s. "nom" lag = 20 ms + mean jitter + one IMU interval. "fit err" is the last
report's fitted yaw minus truth (even when withheld); "worst" is the largest |error| over every
`Candidate` report of the run; "1st" is the time of the first candidate.

```
cargo run --release -p tracking-core --example accel_heading -- --json local-data/benchmarks/accel-heading-synthetic.json
```

| Scenario | Status (last) | Frame / true | Agreement | Margin | Lag / nom ms | Fit err ° | σ ° | ρ_h | 1st s | Worst ° |
|---|---|---|---|---|---|---|---|---|---|---|
| dance, tilt 25°, yaw +25° | Candidate | 13/13 | 0.998 | 0.096 | 31.7/32 | −0.54 | 1.03 | 0.999 | 4.5 | 0.54 |
| dance, yaw −25° | Candidate | 13/13 | 0.998 | 0.100 | 32.0/32 | −0.32 | 1.03 | 0.999 | 4.5 | 0.56 |
| dance, yaw +25°, seed 3 | Candidate | 13/13 | 0.998 | 0.101 | 32.5/32 | −0.22 | 1.03 | 0.999 | 4.5 | 0.53 |
| dance, noise 1.0 m/s² | Candidate | 13/13 | 0.986 | 0.089 | 31.1/32 | −1.73 | 1.22 | 0.990 | 4.5 | 1.73 |
| dance, bias (0.8, −0.5, 0.6) m/s² | Candidate | 13/13 | 0.998 | 0.095 | 31.7/32 | −0.53 | 1.03 | 0.999 | 4.5 | 0.55 |
| dance, lag 40 ms | Candidate | 13/13 | 0.998 | 0.096 | 51.7/52 | −0.52 | 1.03 | 0.999 | 4.5 | 0.59 |
| dance, jitter 0–15 ms | Candidate | 13/13 | 0.998 | 0.094 | 37.6/38 | −0.51 | 1.03 | 0.999 | 4.5 | 0.62 |
| dance, position-only reference | Candidate | 13/13 | 0.994 | 0.095 | 31.8/32 | +0.42 | 1.16 | 0.993 | 4.5 | 1.12 |
| dance, firmware 0.7.4 (`C = I`) | Candidate | 0/0 | 0.998 | 0.099 | 32.1/32 | +0.18 | 1.03 | 0.999 | 4.5 | 0.24 |
| walk upright, no hint | FrameAmbiguous {5, 7, 13, 15} | –/13 | 0.998 | 0.453 | – | – | – | – | – | – |
| walk upright, true hint | Candidate | 13/13 | 0.998 | 0.453 | 31.8/32 | −0.25 | 1.08 | 0.997 | 4.5 | 0.65 |
| walk upright, wrong hint outside group | FrameHintRejected | –/13 | −0.178 | −1.177 | – | – | – | – | – | – |
| walk upright, wrong hint in group (180°) | ExcessiveCorrection | 15/13 | 0.998 | 0.453 | 31.8/32 | +179.67 | 1.14 | 0.994 | – | – |
| walk, mount leaning 8°, no hint | FrameAmbiguous {5, 7, 13, 15} | –/13 | 0.998 | 0.214 | – | – | – | – | – | – |
| walk upright, `C` among chip-Z rotations | Candidate | 9/9 | 0.999 | 0.986 | 31.6/32 | −0.16 | 1.08 | 0.997 | 4.5 | 0.87 |
| standing still | LowExcitation | –/13 | – | – | – | – | – | – | – | – |
| torso translating independently of head | FrameUnidentified | –/13 | 0.203 | 0.112 | – | – | – | – | – | – |
| horizontal independent, true hint | PoorCorrelation | 13/13 | 0.998 | 0.179 | 31.7/32 | +92.75 | 80.83 | 0.063 | – | – |
| head 15° @ 0.5 Hz, neck reference | Candidate | 13/13 | 0.998 | 0.097 | 31.6/32 | −0.56 | 1.04 | 0.999 | 4.5 | 0.65 |
| head 15° @ 0.5 Hz, eye reference | Candidate | 13/13 | 0.997 | 0.102 | 31.4/32 | −0.70 | 1.09 | 0.997 | 4.5 | 0.74 |
| head 30° @ 0.8 Hz, eye reference | InsufficientData (557/600 gated) | – | – | – | – | – | – | – | – | – |
| same, eye reference, rotation gates off | Candidate | 13/13 | 0.949 | 0.081 | 27.5/32 | −0.23 | 2.27 | 0.928 | 5.0 | 3.88 |
| same, neck reference (2–3 cm error), gates off | Candidate | 13/13 | 0.991 | 0.102 | 33.0/32 | −0.69 | 1.15 | 0.994 | 4.5 | 1.03 |
| thigh swing, kinematic reference | FrameIdentifiedOnly | 13/13 | 0.999 | 0.073 | 31.8/32 | – | – | – | – | – |

Reading it:

- In this toy, the frame is recovered among 24 when the torso tilts, the lag to within ~1 ms of
  its nominal value (4.5 ms off with an uncorrected eye reference and open gates), and ±25° yaw
  errors to within ~0.6° (1.7° at 1 m/s² noise). The ~0.5° systematic offset comes from the
  toy's own non-rigidity (torso tilt lever arms, neck model error), not from noise. The
  regressions require < 3° for every candidate report.
- A constant accelerometer bias changes the yaw by ≈ 0.01° here (test bound 0.3°).
- Head rotation contaminates the eye point: with the gates opened, the eye reference's worst
  candidate is 3.9° off while its σ grows to 2.3°; the neck reference stays ≈ 1°. Gating on
  angular *speed* does not remove the tangential acceleration at rotation reversals (ω ≈ 0, α
  maximal), which is why the neck pivot, not the gate, is the primary defence.
- Without tilt diversity the 4-member ambiguity group is reported, a correct hint resolves it,
  a wrong hint outside the group is rejected, and a wrong hint inside it surfaces only as an
  implausible ~180° yaw. An 8° board lean is not enough tilt to separate the group.
- Independent motion never produced a candidate: fully independent translation leaves no frame;
  shared vertical with independent horizontal motion gives ρ = 0.06 and σ ≈ 81°.

These numbers show the implementation matches its own model on a toy body. They do not show the
model matches a human body, the SlimeVR firmware's actual acceleration output, SteamVR's
velocity estimates, or Wi-Fi timing.

## Regressions (`tests/accel_heading.rs`)

Candidate set (24 distinct proper axis-aligned rotations, identity first, index round trip);
sign convention (`Ry(yaw)·S` re-aligns every orientation within 3°); `C` identified for all 24
true frames with tilt diversity and no wrong latch at any report; upright ambiguity group equals
the 4 candidates sharing the true up-row; correct hint → candidate, wrong hint outside the group
→ rejected, inside the group → `ExcessiveCorrection`, re-setting a hint keeps its latch; thigh
frame handed to an upright chest; ±25° within 3° at every candidate report for 4 seeds with
20 ms lag and 0.3 m/s² noise; lag within 8 ms of nominal for 0, 20 and 50 ms; standing still →
`LowExcitation`; independent torso → never a candidate; independent horizontal →
`PoorCorrelation`, ρ < 0.3, σ > 20°; bias invariance; neck reference under head rotation; fast
head rotation gated; position-only reference; chip-Z restriction identifies an upright chest;
a single allowed candidate only verifies; malformed input (NaN/inf in every field, zero
quaternion, huge values, garbage interleaved with a valid run) never panics or emits a nonfinite
number; small backwards time rejected, > 1 s backwards restarts the streams (keeping the frame
latch), a 1.5 s IMU outage yields `InsufficientData` and then correct candidates; ring buffers
never exceed capacity over 200 000 samples; repeated reports are identical; config serde round
trip and validation.

## Live validation still needed

Record (in `local-data/`) headset pose and velocity plus raw IMU packets while walking, stepping
in place, bobbing and dancing, with an externally measured chest heading (e.g. a tracked
controller strapped to the chest) as truth. Check, in order: that firmware 0.7.2 acceleration is
gravity-removed with the expected sign and that one `C` is identified consistently across
sessions and trackers of the same board; the lag distribution; the vertical agreement actually
reached between headset/neck and chest; and only then yaw error against the external truth,
reported separately for chest and pelvis.
