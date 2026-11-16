# Relative-heading shadow experiment

This is an independently implemented research path that **cannot change live tracking**. The existing production knee correction remains separate and off by default. A sliding window can reduce the influence of isolated bad observations, expose poor geometry and reject a candidate contradicted by another physical constraint. It cannot turn an anatomical prior into a measured truth.

The implementation is [heading_shadow.rs](../../crates/tracking-core/src/heading_shadow.rs); deterministic regressions are in [the tests](../../crates/tracking-core/tests/heading_shadow.rs). There are no additional dependencies. The numerical work uses double precision; public calibrated orientation inputs match the core's XYZW float convention in right-handed standing space, +Y up and -Z forward.

## Source basis and scope

Relative heading from joint constraints and moving windows is established prior art. Laidig, Weygers and Seel jointly estimate axes and heading offsets with rotation- and orientation-based constraints. Their paper also describes a moving-window implementation and an ambiguity between axis orientation and heading when the motion is insufficient. This prototype fixes mounting/axes rather than implementing their self-calibration algorithm. [Self-Calibrating Magnetometer-Free Inertial Motion Tracking of 2-DoF Joints, Sensors 2022](https://www.mdpi.com/1424-8220/22/24/9850).

Eckhoff and colleagues analyze a different, sparse double-hinge system with nonparallel axes and gyro constraints. Their observability conditions are specific to that system and motion. They do not establish observability of a human knee from our fused-quaternion packets. [Sparse Magnetometer-free Inertial Motion Tracking — A Condition for Observability in Double Hinge Joint Systems](https://arxiv.org/html/2002.00902v1).

Li and colleagues' October 2026 preprint uses equality of reconstructed joint-center accelerations, estimates sensor locations online, and investigates excitation and impact handling. It motivates a distinct physical constraint that does not require a hinge. Our helper only implements rigid-body acceleration transport with supplied offsets; it does not implement their EKF, online location estimation, filtering pipeline or claimed experimental performance. [Calibration-free IMU-based Relative Orientation Estimation under Dynamic Human Motion, equations 4–6](https://arxiv.org/html/2610.31156v1).

The implementation is an experiment, with no claim of a novel estimation algorithm or demonstrated superiority to SlimeVR. The [broader evaluation plan](tracking-research.md) still requires worn recordings, reliable reference tracking and fair comparisons.

## The fitted quantity

For calibrated parent/child segment orientations `R_p`, `R_c`, let `h_p`, `h_c` be fixed local hinge axes. The defaults are local +X. Their world vectors are:

```text
u_p = R_p h_p
u_c = R_c h_c
r_i(delta) = u_p,i - Ry(delta) u_c,i
```

`delta` is the world-up yaw correction to the child **relative to** the parent. A child measurement with +15° yaw error has a −15° fit. A human knee need not satisfy the equality: true tibial rotation, mounting error, soft-tissue motion and sensor heading error can all affect this residual.

The window is two seconds by default, bounded to 256 unique measured pairs. The estimator minimizes a Huber loss on the Euclidean unit-vector residual. The transition is the chord length corresponding to four degrees. Input confidence supplies a linear engineering weight, not inverse noise variance. Twenty-four fixed circular seeds initialize a maximum of 32 iteratively reweighted least-squares steps. Failure to converge blocks the hypothetical proposal. This numerical strategy is not a proof of global convexity.

Each frozen-weight step has an analytic solution. With `W = sum(w_i)`:

```text
A = sum w_i (u_p,x u_c,x + u_p,z u_c,z)
B = sum w_i (u_p,x u_c,z - u_p,z u_c,x)
L(delta) = sum w_i ||r_i(delta)||² / (2 W)
         = constant - (A cos(delta) + B sin(delta)) / W
delta_fit = atan2(B, A)
```

The offset is assumed approximately constant over the window. A rapidly changing offset or genuine non-hinge motion may violate that model. Robust weighting can downweight real motion as well as corrupt packets; its presence does not prove that rejected samples were sensor errors.

## Mathematical conditioning versus engineering gates

At the reported fit, with the final robust weights frozen, the exact scalar **observed cost curvature** is:

```text
kappa = (A cos(delta_fit) + B sin(delta_fit)) / W
gradient = (A sin(delta_fit) - B cos(delta_fit)) / W
```

Its units are squared dimensionless axis-vector units per radian squared. The code reports the gradient to reveal numerical convergence. This is the curvature of the normalized quadratic surrogate with fixed robust weights, **not** the full Huber-loss Hessian, Fisher information, a covariance estimate or a calibrated accuracy score. The frozen weights and their sum are exposed. At an exact minimizer, `kappa = hypot(A, B)/W`.

For independent parent/child yaw parameters, `delta = theta_c - theta_p`. A common rotation leaves the residual norm unchanged. Thus the exact surrogate Hessian is:

```text
H = kappa [ 1 -1 ]
          [-1  1 ]
eigenvalues = [0, 2 kappa]
common-yaw null direction = [1, 1]
```

The full condition number is never finite. The nonzero scalar subspace has condition number one when nondegenerate; that does not mean it contains much information. Inspect `kappa` for sensitivity. Both directions become null when horizontal information vanishes. Vertically aligned hinge axes demonstrate that case. The hypothetical split `[-delta/2, +delta/2]` preserves the pair's common yaw, but does not determine which sensor drifted.

The scalar Gauss–Newton quantity `sum(w_i ||u_c,horizontal||²)/W` is reported separately. It differs from the observed curvature when model residual remains. A regression compares the observed curvature with finite differences of the actual objective and verifies that a simultaneous yaw shift leaves the objective unchanged. Using a full vector-residual Gauss–Newton matrix without handling this gauge can misleadingly suggest common-yaw information away from zero residual.

These statements concern the specified, fixed-axis model. They do not prove anatomical observability. Input confidence, minimum sample count/span, minimum flex excursion, residual thresholds and the maximum candidate magnitude are separate engineering gates. The flex heuristic is the 95th-minus-5th percentile of the angle between the uncorrected measured longitudinal axes. It is not an independently measured knee angle or an observability theorem. The configuration rejects longitudinal axes nearly parallel to their hinge axes. A stationary, horizontal-axis window can have strong conditional curvature while failing the motion heuristic.

Default gates require 30 unique pairs spanning at least 0.5 seconds, input confidence at least 0.6, no packet gap above 0.15 seconds, receipt skew at most 0.03 seconds and latest real measurement age at most 0.25 seconds. At most 30% of pairs may be Huber-downweighted; the median fitted axis residual must be at most three degrees, flex excursion at least eight degrees and candidate magnitude at most thirty degrees. These thresholds are provisional and have not been tuned on worn validation data.

## Provenance and causal history

`HeadingPairSample` carries explicit roles, calibrated quaternions, original measured timestamps, validity, engineering confidence, calibration epoch and continuity epoch. The quaternion boundary accepts squared norms in `[0.5, 1.5]` and normalizes small numerical error. Invalid numbers, invalid geometry or wrong roles fail closed.

Only `SensorProvenance::Measured` can populate this window. Firmware-held orientations are useful under the separate live compatibility assumption but do not constitute new measured motion here. Cached/reused pair timestamps never increment the sample count. Invalid, stale, misaligned or out-of-order inputs clear the window; a duplicate is rejected without destroying the accepted history. New calibration/continuity epochs and excessive gaps restart coverage. A report cannot evaluate before observations already accumulated by the object. Reset explicitly before changing the monotonic clock origin.

For a replay adapter, undo the existing production yaw adjustment before passing a segment orientation: `Ry(-yaw_correction) * segment_orientation`. Reject preview, uncalibrated, missing-timestamp and invalid frames. Advance epochs on assignment/mounting/recovery/reference changes and transport/recenter/session discontinuities. A trace that lacks enough original timestamps must remain insufficient; render cadence must not become synthetic measurement cadence. Receipt-time alignment does not establish synchronization of the underlying IMU sampling clocks.

## Independent evidence can veto the prior

`IndependentHeadingWitness` carries a directly observed relative-yaw correction, a deterministic angular error bound, observation time, source and an explicit caller assertion that it comes from a different physical constraint. It must not be derived from the candidate or the same hinge equality. Statistical noise can still be correlated because both factors may use the same sensors.

The window intersects the asserted circular bounds. It never averages a contradictory physical witness into a compromise that hides disagreement. Conflicting bounds indicate an inconsistent evidence model, bad bounds or failure of the constant-offset assumption. A candidate outside valid bounds is vetoed. Support requires ten unique fresh witness samples spanning at least 0.25 seconds and individual bounds at most five degrees by default. Repeating a witness does not create additional evidence. Consistent intervals crossing ±π are handled around a witness reference, not unwrapped around the contradicted candidate.

The bounds are supplied assumptions, not estimated accuracy. Repeated identical intervals do not shrink systematic error. A future independent experiment must justify timing, reference frames, sensor identities and its total error budget. The separate [two-landmark contact experiment](contact-heading.md) can conditionally observe torso heading; that alone does not directly measure relative knee yaw and must not be injected as a knee witness without another justified relation.

The optional shared-joint acceleration helper uses the sensor-origin-to-joint vector `r`, all expressed in sensor axes:

```text
a_joint_world = R_world_sensor *
    (specific_force + alpha × r + omega × (omega × r)) + gravity_world
gravity_world = [0, -9.80665, 0]
```

The source preprint defines its lever arm in the opposite direction, so its transport expression has the opposite sign. Specific force includes the gravity term; this is deliberately different from the core's optional gravity-removed acceleration field. A regression verifies gravity, angular-acceleration and centripetal signs with nonzero offsets.

For the same physical joint, independently reconstructed horizontal accelerations should agree after the correct relative yaw. Given each reconstructed horizontal vector norm `n` and a total vector-error bound `e < n`, its direction lies within `asin(e/n)` of the measured direction. Adding the parent/child direction bounds gives a conservative relative-heading interval. Vertical acceleration or gravity alone supplies no yaw direction. Incompatible horizontal magnitudes or vertical components are rejected within the supplied vector bounds.

This requires synchronized actual specific force, angular velocity, angular acceleration, calibrated offsets and justified total error bounds that include tilt, differentiation, timing and nonrigid effects. The helper does not identify offsets, differentiate gyros or estimate those bounds. **Current firmware telemetry has not established this raw-data contract, so no live witness is created.** The factor is useful because, under its assumptions, true non-hinge rotation still leaves the physical joint acceleration common to both segments. It can contradict the hinge prior without assuming the knee is a perfect hinge.

## Reproducible synthetic evidence

Run:

```powershell
cargo test -p tracking-core --test heading_shadow
cargo run --release -p tracking-core --example heading_shadow -- --json local-data/benchmarks/heading-shadow-synthetic.json
```

The example uses 201 synthetic samples from zero to two seconds at 100 Hz timestamps, executed without pacing. It measures no CPU latency and uses no physical recording. The independent-witness rows deliberately supply synthetic bounds from known truth; they are not witnesses extracted from existing trackers. The table below records the release example run on November 14, 2026 (America/New_York). Angles are degrees; the listed eigenvalue is the nonzero eigenvalue `2*kappa` of the normalized frozen-weight surrogate. The other eigenvalue is always zero.

| Scenario | Model candidate | Median axis residual | `2*kappa` | True relative error before | True relative error if candidate were forced | Status |
|---|---:|---:|---:|---:|---:|---|
| 15° relative calf drift, ideal hinge | −15.0000 | <0.0001 | 2.0000 | 15 | <0.0001 | Model-dependent candidate |
| Identical observations caused by true twist | −15.0000 | <0.0001 | 2.0000 | 0 | 15.0000 | Model-dependent candidate |
| Relative drift plus independent −15° ±1° witness | −15.0000 | <0.0001 | 2.0000 | 15 | <0.0001 | Supported by asserted independent bounds |
| True twist plus independent 0° ±1° witness | −15.0000 | <0.0001 | 2.0000 | 0 | 15.0000 | **Vetoed; no hypothetical proposal** |
| 20° common yaw error on both segments | 0.0000 | <0.0001 | 2.0000 | 0 | 0 | Common error remains unobservable |
| Genuine global turn | 0.0000 | <0.0001 | 2.0000 | 0 | 0 | Turn preserved |
| Vertical hinge axes with flexion | Not determined | <0.0001 | <0.0001 | 0 | Not determined | Degenerate; blocked |
| 15° relative drift plus 45° outliers in about 10% of the window | −15.4089 | 0.4089 | 1.9942 | 15 | 0.4089 | Robust model-dependent candidate |

The common-drift row retains **20° absolute error on each segment** despite zero relative error. In the ideal 15° relative-drift row, the symmetric split leaves 7.5° error on each segment: common heading remains unknown. In the true-twist row, a zero prior residual would create 15° relative pose error, or 7.5° on each segment after the symmetric split. Those values are mathematical counterexamples, not claims about the distribution of anatomical knee motion. No correction is applied by the example, including in the supported rows.

The outlier row ends on an uncorrupted sample so the reported pose error uses the clean 15° offset; outlier corruption inside the window is a separate observation error. A temporal held-out test using the same hinge assumption would establish predictive consistency only. Worn reference data, sensor-offset calibration, timestamp validation and independent ground truth remain necessary before judging tracking quality.
