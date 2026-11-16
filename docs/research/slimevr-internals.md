# SlimeVR internals: what we are comparing against

Source inspection on 2026-11-15. Server pinned at SlimeVR-Server `83941fd38e91` (the legacy Kotlin
v21.x line), firmware at SlimeVR-Tracker-ESP v0.7.2 (`c12c475`), driver `main` at `dcc0f56`; the
v22 rewrite (tag `v22.0.0-rc.4`) was read on GitHub. **[V]** means read in the code, **[I]** means
inferred from it. This describes those snapshots, not a completed audit of every release, and it
says nothing about measured accuracy. Paths are relative to
`server/core/src/main/java/dev/slimevr/`.

## Skeleton and IK

- **Pure forward kinematics from the headset.** [V] `HumanSkeleton.kt`. The neck sits
  0.10 m behind the headset (`headShift`) and 0.10 m down; with no neck tracker it pitches fully
  with the head. No IMU acceleration or position enters the pose.
- For chest + hip + thighs + calves: waist = `slerp(chest, hip, 0.30)`; thighs and calves are used
  directly; **foot = full calf rotation** (no ankle model); the knee tracker output is
  `slerp(thigh, thigh pitch with shin yaw/roll, 0.85)`. [V]
- Joint limits are hard clamps, and the constraint-driven correction path is disabled
  (`updateWithConstraints(false)`). [V]
- **The CCD IK solver has no caller in the legacy server.** [V] v22 routes leg tweaks through a
  CCD solver; its benefit for IMU-only users is unknown.

## LegTweaks (floor clip, skating correction, foot plant)

- Heuristic and position-only. [V] The floor is captured at full reset. Feet are forced unlocked
  whenever the hip is below 35 % of its reset height, so corrections are off when sitting,
  kneeling or crouching.
- Lock/unlock uses horizontal drift (0.5 m), speed (2.4 m/s × sensitivity), foot height
  (floor + 0.065 m) and calf linear-acceleration magnitude. [V]
- Locked feet are pinned in x/z; knees are shifted by 0.8 × the foot correction with
  `KNEE_CORRECTION_WEIGHT = 0`, so segment lengths are not preserved and the hip is never moved
  horizontally. [V]
- Inferred defects [I]: the engage threshold is looser than the stay-locked threshold (inverted
  hysteresis), sensitivity scalars are computed before the state they depend on, and several
  per-frame quantities are compared against per-second thresholds.
- Contact is cosmetic: it is never fed back into orientation or drift.

## Yaw drift

- **Stay Aligned is off by default** and needs a relaxed-pose setup step. [V] It nudges each
  tracker's yaw by a fixed ±0.15 °/s (LSM6DSV) using a sign-only search over a weighted error:
  10 × locked-rest error, 2 × body-centre error, 1 × neighbour error. Rest is "raw rotation within
  2° for 1 s". Poses come from a fixed list (standing, sitting, lying, kneeling).
- Its only absolute reference is headset **yaw**, at weight 0.5 inside a circular mean with the
  body trackers. It never uses headset position or velocity, acceleration, or contact. It cannot
  observe a whole-body common-mode error, pulls deliberate asymmetric poses toward its template,
  and cannot follow drift faster than its rate. The code notes its rest lock "does not work well
  when the player is standing or moving around". [V]
- **The older drift compensation is hard-disabled** (`compensateDrift = false`) and absent from
  v22. [V] It extrapolated yaw from a time-weighted average of the last six reset deltas.

## Resets, mounting, taps

- Full reset (I-pose), yaw reset, and a ski-pose mounting reset that sets one yaw about the bone
  axis from the tilted "up" vector. [V] Users report it is hard to perform (issue #1869).
- v22 "step mounting" (work in progress) double-integrates tracker acceleration during a step and
  yaw-aligns the result to the headset's displacement. [V] This is the only place headset
  displacement is used as heading evidence, and it is a one-shot calibration, not continuous.
- Tap detection is server-side on linear-acceleration magnitude. [V]

## Filtering and latency

- Default "prediction" at amount 0.2 slerps toward `latest × product of the last six packet deltas`
  with a ~77 ms time constant. It counts packets rather than time; the net lead is about zero. [V/I]
- v22-rc.3 "dynamic hybrid filtering" upsamples to 500 Hz at roughly 10 ms cost. [V]
- **Driver:** `poseTimeOffset` is never set; angular velocity stays zero; linear velocity is sent
  only if the server provides it, and that defaults to off. There is no staleness timeout: a
  tracker stays valid until a status message changes it. [V] Reported bridge latency is 6–12 ms
  (server issue #511) and "at least one frame behind" (driver issue #34).

## Firmware 0.7.2 on LSM6DSV

- Software VQF in 6-D mode; gyro 240 Hz, accelerometer 120 Hz, output ≤ 100 Hz. [V]
- No raw samples and no device timestamps on the wire, only a packet number. [V]
- The acceleration packet is gravity-removed, in the sensor body frame, and **not** rotated by the
  board mounting offset (corrected in 0.7.4). [V]
- A rotation is not sent when every quaternion component changed by less than 1e-4. [V]
- Saved gyro bias applies only after a reboot; temperature compensation is effectively a no-op. [V]

## AutoBone

Offline: 30 s recording, greedy coordinate descent on foot-slide error plus proportion limiters.
It fits lengths only, not yaw or mounting. [V]

## Structural gaps we target

| SlimeVR (inspected snapshots) | ChadFBT |
| --- | --- |
| Yaw drift handled by relaxed-pose priors at a fixed rate, off by default | Per-sensor yaw error estimated with covariance from displacement evidence; priors only where evidence is absent |
| Headset position never used for heading | Headset displacement against the IMU chain during stance observes common-mode yaw |
| Contact is cosmetic and disabled when not standing tall | Contact is a stationary-point hypothesis that feeds the heading filter and the root correction in any posture |
| Foot lock stretches segments; hip never moved horizontally | Root correction plus two-bone IK keeps segment lengths exact |
| No ankle model; feet rotate with the shin | Flat-foot blend while planted, gated by shin tilt |
| No `poseTimeOffset`, velocities off by default, no staleness timeout | Velocities and pose age sent; stale targets invalidated |
| Packet-count "prediction" | Time-based angular-rate extrapolation to solve time |

Every row is a design difference. Whether any of them produces better tracking is an open
question until matched worn recordings are compared.

## Leads taken from this review

1. **Torso acceleration matching.** The chest and pelvis translate nearly rigidly with the head.
   Horizontal headset acceleration (absolute frame) against the chest IMU's world-frame linear
   acceleration observes chest yaw, exactly where stance displacement is weakest (an upright torso
   has vertical levers). It needs the firmware acceleration frame established first; the vertical
   component is yaw-independent and can verify the frame convention before the horizontal
   components are trusted.
2. Continuous floor and bone-length estimation from the headset trajectory.
3. Continuous mounting refinement from motion instead of a single pose snapshot.
