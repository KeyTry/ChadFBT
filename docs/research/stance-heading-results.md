# Stance heading filter: synthetic results

Recorded 2026-11-15 from `cargo run --release -p tracking-core --example stance_heading`.
Design: [stance-anchored-estimator.md](stance-anchored-estimator.md).

**Read this first.** Every number below comes from a deterministic articulated-body generator
that uses the estimator's own kinematic model, with injected yaw error, headset noise, bone-length
error and eye-offset error. Passing these scenarios shows the implementation matches the model and
behaves sensibly when the model's assumptions are violated in the specific ways that were
scripted. It does not show that the model matches a human body, and none of this is a measurement
on worn hardware or a comparison with SlimeVR.

"Time to 3°" is the time until the true yaw error of the affected segments stays below 3°.
"False correction" is the largest correction the filter applied in a scenario where the correct
answer is zero.

## Convergence

| Scenario | Result |
| --- | --- |
| Walking, 10° common-mode error on all leg sensors | below 3° in 4.4 s |
| Walking, 20° common-mode error from the start | below 3° in 10.1 s (32.4 s before uncertainty-aware contact gates) |
| Walking, sudden 20° on all leg sensors after convergence | re-converges in 9.8 s (30.9 s before) |
| Walking, sudden 30° on all leg sensors | re-converges in 17.2 s (never, before) |
| Walking, a single thigh jumps 20° | re-converges in 8.6 s |
| Legs drift 25° while unobserved, then walking | below 3° in 13.1 s (40.8 s before) |
| Weight shifting while standing, 10° on legs | below 3° in about 50 s; final 1.1° |
| Sign check: thigh measured at `Ry(−10°)·true` | correction converges to +9.1° |
| 10 min of independent random-walk drift during mixed activity, three seeds | RMS 2.49 / 2.83 / 2.65° with the filter, 5.5 / 10.1 / 10.2° without |

## Where it must not act

| Scenario | False correction |
| --- | --- |
| Standing still with drift present | 0 displacement measurements, 0.000° from displacement (the upright priors may act, at most 2.8°) |
| Head-only turns with a 4 cm eye-offset model error | 0.10° |
| 0.3 m headset tracking jump | 0.000° |
| Real foot slide at 5 cm/s | 1.4° |
| Real foot slide at 15 cm/s | 3.4° |
| Ten 90° turns on the spot with pivoting feet | 2.1° (15.4° without pivot release and the pivot noise term) |
| 5 % bone-length error, with drift | largest error after convergence 2.8° |

## What it cannot do, measured

- **Squats alone cannot say which leg sensor drifted.** With one calf 15° off, symmetric squats
  observe only thigh minus calf. The filter closes the relative error and splits the correction by
  prior variance: thigh −5.9°, calf +3.9° when both started equally uncertain; calf +9.0°,
  thigh −1.0° when the calf was marked as more uncertain. A mixed sequence of squats, steps and
  weight shifts resolves it to below 3° in 22 s.
- **A strap rotating around the limb is invisible to displacement.** That is a rotation about the
  bone's own long axis. It leaves every chain position unchanged, so the displacement residual is
  exactly zero and a 17° heading error of the bone's forward axis remains. A world-yaw correction
  would be the wrong remedy on a pitched thigh because it would move the knee. This needs a
  separate mounting-twist state observed through the knee hinge, or recalibration; see the
  research plan.
- **Torso heading is weakly observed** when upright; the chest keeps a residual of roughly 1.4°
  after the walking scenarios, and pelvis RMS is the worst of the six over ten minutes.
- **Large errors are slow.** About 40° takes about 25 s of walking.
- **Noise floor.** Walking with no drift at all shows about 1–1.5° of wander from 1 mm of headset
  noise.

## Changes that moved these numbers

| Change | Effect |
| --- | --- |
| Pivot release (calf turned > 15° since settling, or ankle vertical speed > 0.15 m/s) and a pivot noise term | Pivot-turn false correction 15.4° → about 2° |
| Uncertainty-aware contact speed allowance | A foot can be recognised as planted while the filter's own yaw uncertainty inflates its computed speed |
| Innovation-driven covariance inflation (rejections and a running normalised-innovation average) | A confident-but-wrong state recovers instead of rejecting good data |
| Contact-starvation inflation (a foot that has not planted for 1 s while the head translates) | Recovery from errors large enough to prevent planting at all |

These refinements slightly worsened three figures: single-thigh recovery 6.4 s → 8.6 s, the
5 cm/s slide 1.0° → 1.4°, and the ten-minute RMS by at most 0.03°.
