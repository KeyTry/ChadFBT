# ChadFBT

Independent, native Windows full-body tracking for SteamVR and VRChat, built around inexpensive
Wi-Fi IMU trackers that have **only an accelerometer and a gyroscope**. It works with trackers
running [SlimeVR firmware](https://github.com/SlimeVR/SlimeVR-Tracker-ESP) and replaces the
server-side software: it receives the trackers, estimates the body, and publishes virtual
trackers to SteamVR.

> **Status: research build.** It runs, it has been worn in VR, and its estimator is covered by a
> large synthetic test suite. It has **not** been measured against SlimeVR or any reference
> system, and nothing here is a demonstrated accuracy improvement. See
> [Status and evidence](#status-and-evidence).

## The problem it is built around

A six-axis IMU measures acceleration and rotation rate. Gravity tells it which way is down, so
tilt is reliable. Nothing tells it which way is north: heading comes only from integrating the
gyroscope, so it drifts, and every tracker starts with its own arbitrary heading. Without a
magnetometer (which indoor magnetic disturbance makes unreliable anyway) there is no absolute
heading reference inside the tracker.

A VR headset is the opposite. Its position and orientation are absolute and drift-free in the
room, and the floor height is known.

ChadFBT's premise is that the headset should be used as **evidence**, not only as the point the
skeleton hangs from. If a foot is standing still and the head moves, the chain of body segments
between them has to reproduce that movement. When it does not, the mismatch says how far each
tracker's heading has drifted.

## What it does

- Receives Wi-Fi trackers over the SlimeVR firmware protocol (UDP 6969). No SlimeVR Server needed;
  the firmware on the trackers stays as it is.
- Estimates a body from six trackers (chest, hip, both thighs, both calves) plus the headset.
  Foot and upper-arm trackers are optional.
- Publishes hip, chest, knee and foot trackers to SteamVR through its own OpenVR driver, or to
  VRChat over OSC.
- Runs as one native window (Rust, egui) with a tray icon. No browser, web server, cloud service
  or telemetry. Profiles and recordings stay on your machine.

## How it works

```
 trackers ──► orientation per segment ──┐
 (Wi-Fi)      calibrated by movement    │
                                        ▼
 headset ───► kinematic chain: head → neck → chest → hips → knees → ankles
 (absolute)                             │
                                        ├─► contact: which foot is standing still?
                                        ├─► heading filter: head displacement vs. leg chain
                                        ├─► leg IK: standing-floor prior, horizontal foot locks
                                        ▼
                             tracker poses + velocities ──► SteamVR / VRChat
```

1. **Calibration by movement.** Stand straight, then bow once and lift each knee a couple of
   times (or do two squats). Each tracker's mounting and heading are fitted from the whole
   movement, with per-tracker progress and spoken prompts, so there is no pose to hold. With a
   headset, "forward" comes from where the head actually travelled during the bow. A triple tap
   on the chest or hip starts it hands-free.
2. **One kinematic chain** from the headset down to the ankles, with a neck pivot behind the
   eyes so looking around does not swing the body.
3. **Contact hypotheses.** A foot that is not moving is treated as a stationary point in the
   room. This is inferred from the skeleton, never measured.
4. **Stance heading filter.** While a foot is stationary, the difference between how the headset
   moved and how the leg chain says it moved is used to estimate each leg tracker's heading
   error, with an uncertainty. Because the headset is absolute, this sees drift of the whole
   body, which joint-to-joint constraints cannot. Only changes in the chain carry information,
   so standing still or turning the head cannot move the estimate.
5. **Contact-consistent legs.** A foot hypothesised to be planted is locked horizontally, the
   pelvis is shifted a bounded distance toward consistency with it, and the knee is re-solved
   with fixed bone lengths. A standing-floor prior applies only to a leg whose thigh and shin
   are both near upright: it may lower the pelvis and legs so the support foot rests on the
   floor, within a budget that grows when the torso bows forward or sideways and that fades out
   when the gap is too large for that budget to explain (a jump, a step, a stool, a bed). A
   squat keeps the prior through its forward-bowed torso. Legs that are folded under an upright
   or reclined torso (seated, lying) lose it: their height follows the body model and a foot
   lock may not change the knee angle the thigh and shin sensors measure by more than about a
   degree. Feet below the floor always raise the body. Pelvis position remains
   inferred; chair and bed support are not measured or modelled.
6. **Timing.** Packets are time-stamped on arrival, the solver runs when data arrives, each
   tracker is extrapolated to the moment of solving, and poses are sent to SteamVR with
   velocities so it can predict to display time.
7. **Staying connected.** Output switches itself between desktop preview and SteamVR as the
   headset comes and goes. A SteamVR restart or a recenter does not discard calibration.

The design, equations and known limits of each part are in
[docs/research/stance-anchored-estimator.md](docs/research/stance-anchored-estimator.md).

## How it differs from SlimeVR

SlimeVR is the mature, cross-platform project that made this hardware usable, and ChadFBT uses
its firmware. The differences below are **design differences**, read from specific SlimeVR source
snapshots ([details and file references](docs/research/slimevr-internals.md)). Whether any of
them produces better tracking is an open question.

| | SlimeVR Server (snapshots inspected) | ChadFBT |
|---|---|---|
| Heading drift | Nudged toward a relaxed-pose template at a fixed rate (Stay Aligned, off by default), plus manual resets | Estimated per tracker, with uncertainty, from headset displacement during stance |
| Whole-body drift | Not observable without a reset | Observable whenever the body moves over a planted foot |
| Headset position | Root of the skeleton only | Root of the skeleton and heading evidence |
| Planted feet | Position-only correction that can stretch the leg; off when the hips are low | Horizontal lock, bounded pelvis correction, fixed bone lengths; floor support assumed only for upright legs; seated/lying support is not modelled |
| Feet without foot trackers | Rotate with the shin | Flat while planted with the shin near upright |
| Calibration | Held I-pose and ski pose (a step-based method is in progress upstream) | Stand, then any forward-bending movement; ski pose kept as a fallback |
| Latency handling | No pose age sent; velocity off by default | Arrival timestamps, extrapolation to solve time, velocity and pose age sent |
| Scope | Many tracker counts, boards, platforms and languages | Windows, SteamVR, one six-tracker layout |

None of the ideas is new on its own. Using an absolute position reference through a kinematic
chain to observe heading is established in legged-robot odometry and position-aided inertial
capture; the combination used here is described, with its prior art, in the
[literature review](docs/research/literature-review.md).

## What it cannot do

- It needs movement. A perfectly still body gives the heading filter nothing to work with.
- Contact is a guess. A foot sliding slowly can be mistaken for a planted one.
- Torso heading is only weakly observed when you stand upright.
- A strap that rotates around the limb changes mounting, not heading, and needs recalibration.
- Sitting, kneeling and lying are outside what has been validated. Low hips and raised legs
  were reported in both the app view and the avatar, including lying in bed. Mechanisms that
  produce such errors were reproduced on synthetic poses and changed (a forced knee bend on
  locked straight legs, floor pull-down of seated legs, foot locks overriding the measured knee
  angle), but none of the changes has been checked on a worn body, and nothing models a seat or
  a bed. A chair and a parallel squat look the same to the leg sensors; they are told apart only
  by a forward-bowed torso, so leaning well forward on a seat with the feet off the floor is
  treated as a squat and pulled down, and genuinely grounded seated feet can float when the
  body model is wrong.
  [Seated and lying investigation](docs/research/seated-ik-investigation.md).
- Feet have no trackers by default, so ankle movement is inferred.

## Getting started

Requirements: Windows 10 or 11, SteamVR, a headset, and six Wi-Fi trackers running SlimeVR
firmware on the same network. Development and testing so far used LSM6DSV trackers on firmware
0.7.2.

Build (Rust stable, Visual Studio 2022 C++ tools, Windows SDK):

```powershell
.\tools\build.ps1
.\tools\package.ps1
```

This produces a portable bundle in `dist\ChadFBT`. Register the SteamVR driver once with
`driver\registration.ps1 -Action install`, and set `steamvr.activateMultipleDrivers=true` in
SteamVR's settings if another tracking driver is installed.

Then:

1. Close any other tracker server using UDP 6969, start `chadfbt.exe` and power on the trackers.
2. Assign each tracker to a body location (the app can identify a tracker when you rotate it).
3. Start SteamVR and put the headset on. Output mode **Auto** goes live by itself.
4. Calibrate by movement (button, or triple tap your chest), then assign the virtual trackers
   in SteamVR and run VRChat's own full-body calibration.

The [user guide](docs/USER-GUIDE.md) covers every control, the estimator switches, gestures,
recording and replay, and troubleshooting.

## Repository layout

| Path | Contents |
|---|---|
| `crates/tracking-core` | Estimator: calibration, kinematic chain, contact, heading filter, leg IK, prediction, research modules, synthetic tests |
| `crates/tracker-protocol` | Firmware wire-protocol parser, written from the protocol's behaviour |
| `crates/steamvr-bridge` | Versioned loopback protocol between the app and the driver |
| `crates/chadfbt-app` | Network worker, calibration flows, voice prompts, recording and replay, OSC, native UI |
| `driver` | OpenVR driver and headset helper (C++), with the pinned OpenVR SDK |
| `docs` | User guide, research plan, validation record, design and literature notes |
| `tools` | Build, packaging and development scripts |

## Documentation

- [User guide](docs/USER-GUIDE.md)
- [Research plan](docs/RESEARCH-PLAN.md): hypotheses, what is implemented, next experiments
- [Validation record](docs/VALIDATION.md): what was tested, how, and what was not
- [Estimator design](docs/research/stance-anchored-estimator.md) and
  [synthetic results](docs/research/stance-heading-results.md)
- [Movement calibration](docs/research/motion-calibration.md) and
  [calibration methods survey](docs/research/calibration-methods.md)
- [SlimeVR source review](docs/research/slimevr-internals.md) and
  [literature review](docs/research/literature-review.md)
- [Estimator core reference](crates/tracking-core/README.md)
- [Seated and lying IK investigation](docs/research/seated-ik-investigation.md): synthetic
  mechanisms, what was changed, known limits and the pending worn validation

## Status and evidence

- **Tests:** several hundred synthetic regressions across the workspace cover numerical
  invariants, frame conversions, stale and malformed data, calibration gates and the estimator
  layers. Synthetic tests show the code matches its model, not that the model matches a body.
- **Worn sessions:** a small number, by one person, recorded in the
  [validation record](docs/VALIDATION.md). They found real defects (legs lifting during bends, a
  false chest heading correction, a phantom tracker), which were fixed and checked by replaying
  the recorded session. The later seated and lying changes rest on synthetic reproduction and
  one replayed seated segment; they have not been worn.
- **Not done:** any comparison against SlimeVR, any optical or camera reference, more than one
  body, long sessions, other tracker hardware or firmware versions.

The project's rule is that unit tests are never described as proof of accuracy, and that
"better" requires matched worn comparisons. Contributions of recordings and comparisons are the
most useful thing anyone can add.

## Development

The engineering rules are in [AGENTS.md](AGENTS.md): frames and ownership, evidence classes,
validation expectations. Much of the code and documentation was written with AI coding agents
under those rules, directed and tested by a human on real hardware.

```powershell
cargo test --workspace
cargo run --release -p tracking-core --example stance_heading
cargo run -p chadfbt -- --replay <recording.jsonl> --dump <out.jsonl> [--every N] [--estimator JSON|@FILE]
```

Recordings and anything else captured from a real session stay in the gitignored `local-data`
folder.

## License

Dual-licensed under either of

- [MIT License](LICENSE-MIT)
- [Apache License, Version 2.0](LICENSE-APACHE)

at your option, the same terms SlimeVR uses. Unless you state otherwise, any contribution you
submit is dual-licensed as above without additional terms.

The pinned OpenVR SDK under `driver/third_party/openvr` is Valve's, under its own BSD 3-Clause
license. Dependency licenses are collected into the portable bundle by `tools/package.ps1`.

ChadFBT is an independent project. It is not affiliated with or endorsed by SlimeVR, Valve or
VRChat. It contains no SlimeVR Server code; it speaks the tracker firmware's network protocol so
existing trackers work unchanged. SlimeVR, SteamVR and VRChat are names of their respective
owners' products.
