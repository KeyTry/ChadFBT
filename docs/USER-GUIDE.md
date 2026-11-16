# ChadFBT user guide

How to run, calibrate and troubleshoot ChadFBT, and what each control does. For what the project is
and how it differs from other trackers' software, see the [project README](../README.md).

Everything here describes a research build. The estimator layers are verified by synthetic tests
and a small number of worn sessions recorded in [VALIDATION.md](VALIDATION.md); none of it is a
demonstrated accuracy improvement over any other software.

## Run

Open `dist/ChadFBT/chadfbt.exe` after building the portable bundle. Keep the helper, OpenVR client DLL and driver folder with the application. No Rust, Python or compiler is needed on the machine running the completed bundle.

**Hide to tray** leaves tracking running. Opening the executable again restores the existing window instead of starting a second tracker receiver. Use the tray menu to quit the process completely.

1. Close other tracker servers using UDP 6969. Turn on the trackers on the same reachable local network.
2. Start SteamVR through your normal headset streaming software, before or after ChadFBT. ChadFBT connects by itself as soon as SteamVR is running and reconnects quietly whenever SteamVR exits and returns (Steam Link, for example, shuts SteamVR down after 30 s of an idle headset); it never starts SteamVR itself. The **output mode** (Tracking output, saved in the profile) is **Auto** by default: with a worn headset the app runs live and sends to SteamVR as soon as the calibration is valid for live use; without one it shows the desktop preview. It switches both ways by itself. **SteamVR** and **VRChat OSC** are live only (they wait for a headset); **Desktop only** ignores the headset and sends nothing.
3. Assign the six physical sensor roles: chest, hip, left/right thigh and left/right calf. Use **Identify a tracker by movement**: choose a body location, rotate only its tracker during a three-second capture, then explicitly confirm the highlighted match. [Identification guide](TRACKER-IDENTIFICATION.md). Optional foot and upper-arm sensors have separate roles. These sensor roles are distinct from SteamVR output target roles.
4. Set standing **eye height**, measured from the floor to the eyes, rather than total body height. Setup displays the current valid headset height in SteamVR's standing space and its difference from the configured value. **Use current headset height** is an explicit action for when you are standing upright with the headset worn; an incorrect Quest/SteamVR floor level must be corrected before using that reading. The current UI scales default body proportions; a future guided body measurement stage is planned. Per-segment dimensions are stored in the profile. With **Update eye height from the headset at each live neutral capture** (on by default) a live neutral capture first adopts the headset height averaged over its stable window when it differs from the saved eye height by more than 1.5 cm and stays within the live height tolerance (±0.20 m by default); body dimensions are rescaled as with the explicit button, and a change over 5 cm is announced ("Height updated."). The new height is applied only if that capture passes, so a rejected capture leaves height and calibration unchanged.
5. **Calibrate by movement (experimental; primary).** Click **Calibrate by movement (3 s)**, then use the **three-second countdown** to stand upright with feet under the hips, arms relaxed, and head facing forward with the torso. Keep still: the neutral capture allows up to three additional seconds for stable valid inputs (live mode also requires the fresh, valid headset pose, exactly as before). At the next tone, move: **bow forward once as if greeting someone, then lift each knee twice (or do two slow squats)**. Keep facing the same direction, knees pointing forward, no twisting; you do not need to hold any pose. Setup shows a progress bar and the current blocker in plain words for each of the six segments (chest, hip, both thighs, both calves), warnings such as a turn during the procedure or suspected swapped trackers, and in live mode the headset forward diagnostics. The calibration finishes by itself once every segment has stayed ready for half a second (and at least three seconds of movement have passed), with a success tone. If it is not ready within 30 seconds, a failure tone and a message list each blocking segment's reason; the neutral capture is kept, so **Try movement again** repeats only the movement. A valid neutral from the last five minutes can also be reused directly with **Start at the movement step**. Cancel works at every stage. **Spoken guidance** (setting next to **Test sound**: *Voice + tones*, *Tones only* or *Silent*) tells you the goal of each step for when you cannot see the screen: "Calibration. Stand straight and still." → "Now bow forward, then lift each knee twice." → while moving, at most every five seconds, what is still missing ("Bow a little deeper.", "Lift your left knee higher.", "Keep facing the same way.") → "Calibration complete.", or on timeout what blocked it ("Calibration did not finish: hip needs more forward bend."). The tones remain as punctuation. Speech uses Windows speech synthesis (an installed Microsoft voice) on the Windows default audio device, on its own thread; if no voice is available the tones play instead and Setup says so. Use **Test sound** (a tone plus a spoken sentence) to check routing and volume before putting on the headset; whether it reaches the headset depends on how your streaming software routes the Windows default device. First calibration depends on the user's neutral-pose assertion: unknown mounting cannot independently prove anatomical uprightness. [Method, gates and limits](research/motion-calibration.md).
   What it cannot detect: a sideways lean of a segment looks exactly like a different tracker direction (with a headset only a sideways torso lean is exposed, by the neck's movement); knees or toes pointing outward during squats bias the leg directions by several degrees (about 5–16° in synthetic tests). With a live headset the stance heading filter, if enabled, starts with widened uncertainty after this calibration so ordinary movement can refine those leg headings; desktop preview has no such correction. With a headset, forward comes from the neck's movement during the bow fused with the headset direction at neutral, so the head does not have to face exactly the same way as the torso. Everything here is validated on synthetic data only, not on a worn body.
6. **Fallback: held skiing pose.** **Use held skiing pose instead** switches Setup (persistently) to the original two-step flow: **Calibrate neutral pose (3 s)**, then **Calibrate forward bend (3 s)** while facing the same direction: feet flat and parallel, knees forward, hips back, with torso and pelvis leaning forward into a comfortable skiing pose. Your pelvis can tilt less than your chest; matching angles are not required. After the three-second countdown, there are up to ten seconds to settle, and small sway is allowed. This second observation identifies each core tracker's bend axis; an upright reset or HMD heading alone cannot separate arbitrary mounting from independent IMU yaw. Follow the per-segment blocker if there is insufficient tilt. Sideways bends can look like different mounting angles. [Math and limitations](research/functional-mounting.md). **Calibrate by movement instead** switches back.
7. Return upright and inspect bends and turns in the 3D view. The floor arrow shows the preview or captured calibration direction; orbiting the camera does not change tracking. **Behind** is useful for comparing your left/right limbs without a face-to-face view. Once a live calibration is complete, Auto (or SteamVR) mode sends to SteamVR without a further step; assign the virtual tracker body roles in SteamVR, then perform VRChat's own avatar FBT calibration.

**Desktop and VR.** A calibration started while no worn headset is available (Auto without a headset, or **Desktop only**) is a desktop calibration: choose a physical direction and keep facing it through the neutral capture and the movement; the app maps it to the preview arrow. In desktop preview nothing can detect a sideways lean, and nothing corrects a knees-out bias afterwards. A desktop calibration carries over to VR without repeating the procedure: when a worn headset appears (any mode except Desktop only) you hear "Headset found. Stand straight and look ahead."; once you stand upright and still for about a second, legs and torso facing the same way, the app rotates the whole calibration so the torso faces where the headset looks ("Aligned.") and live output can start. Like a live neutral capture this assumes the head faces the same way as the torso; the stance heading filter starts with widened uncertainty afterwards. A calibration made live stays valid when the headset disappears (the body is shown as desktop preview) and is live again when it returns. A headset lying on the desk does not count as worn until its height is within 45 cm of the eye height, so it does not take Auto out of desktop preview. **Estimate body movement (experimental)** uses leg kinematics and possible supporting-foot contact to estimate root displacement in desktop preview, including lowering the body during a squat. It is enabled by default in the app and can be disabled for the original fixed-reference comparison. These contact assumptions can be wrong; movement without informative leg motion and jumps are not fully observable. Calibration resets estimated displacement and recenters the camera. Desktop preview poses never authorize live VR output. Merely showing or hiding the 3D view does not change tracking mode.

**Advanced · manual mounting fallback** lets you select any assigned tracker, adjust bend direction while preserving neutral, or use the local X/Y/Z gizmo for physical mounting trim. The physical trim may change neutral tilt. Apply and test actual movement, then explicitly confirm each tracker; Undo and Reset are local to the selected tracker. This is a secondary user-asserted alignment path, not an automatic pose measurement. Movement and skiing-pose calibration currently solve the six core roles; optional foot/upper-arm mounts need manual confirmation before they can supply measured segment orientations.

If distributing the bundle to another installation, register its driver with `driver/registration.ps1 -Action install`. SteamVR must have `steamvr.activateMultipleDrivers=true` to accept these devices alongside another headset driver. The development installation was registered and this setting was enabled with a backup; see [validation notes](VALIDATION.md). Avoid simultaneously emitting duplicate body targets from multiple tracking applications.

### Calibration or grey SteamVR trackers

The driver can be registered while no virtual trackers are visible: it adds a tracker to SteamVR only when the app first sends a valid pose for it, so trackers the app cannot drive (such as elbows without upper-arm sensors) never appear. **Desktop only** or **VRChat OSC** mode, a running calibration countdown or capture, incomplete neutral/direction calibration, a desktop calibration not yet aligned to the headset, or a missing valid headset anchor means no valid body poses are published to SteamVR. The Tracking output line says what the mode is doing ("Auto: live in SteamVR", "Auto: desktop preview (no headset)", "Auto: waiting to align with headset"). Finish a calibration (by movement, or neutral plus skiing pose; desktop calibrations are aligned automatically); then assign the virtual tracker roles in SteamVR (or with `--assign-core-roles`, below).

SteamVR-side events never clear the calibration. A new helper session (SteamVR restarted), a change of SteamVR's raw-to-standing transform (driver restart, switching between controllers and hand tracking, relocalisation, recenter), headset gaps and hands or controllers appearing or disappearing are all routine. What decides is whether the headset's standing-space pose stayed physically continuous: within plausible head motion for the elapsed time nothing changes; a jump (a recenter) rotates the calibration by the jump's yaw, using the transform's exact yaw when it agrees with the jump. After a gap longer than 2 s (SteamVR restarted, headset idle) the calibration is kept on the assumption that the same headset in the same room reports the same standing space; the stance heading filter starts with wider uncertainty and you hear once "Tracking resumed. If the body faces the wrong way, double tap to reset heading." Only an assigned tracker that really rebooted clears the calibration ("A tracker restarted. Triple tap to calibrate."). None of this has been exercised yet against a real SteamVR restart, input-mode switch or recenter; see [validation notes](VALIDATION.md). A tracker that has appeared is reported out of range at its last valid position when its pose is lost, and disconnected when the app stops sending for more than 1 s; how SteamVR and VRChat display those states has not been checked live. Missing or grey trackers alone do not establish that the driver failed to load.

Calibration results are displayed in Setup. A live-calibration height rejection reports the actual headset height, configured eye height and permitted range. With a 1.60 m setting and the default 0.20 m tolerance, for example, the permitted headset height is 1.40–1.80 m. Leaning toward the desktop to press the button can move the headset outside that range; the countdown gives time to return upright. If the live reading remains wrong when standing, check the eye-height setting and Quest/SteamVR floor alignment rather than widening the gate. Desktop preview does not require a headset or pass through this live-headset gate.

For read-only integration diagnostics while SteamVR is already running, `chadfbt-anchors.exe --inspect` prints standing-space HMD height and presence/validity of the eight ChadFBT output slots. It sends no tracker poses and does not start SteamVR. Slot names describe the driver's serial identities, not verified SteamVR body-role assignments.

`chadfbt-anchors.exe --assign-core-roles` explicitly assigns the six core virtual trackers to Waist, Chest, Left/Right Foot and Left/Right Knee through SteamVR's settings API, then reads the values back. SteamVR must already be running. Other devices and elbow roles are preserved. This is a setup command; normal app startup and `--inspect` do not change roles. A partial failure reports which writes succeeded and exits with an error.

## Estimator layers (experimental)

The **Estimator · experimental** section below the 3D view switches each layer independently. All
of them are verified by synthetic regressions only; none is a demonstrated accuracy result on a
worn body, and every switch exists so a recording can be replayed with and without it. Design,
equations and limits: [stance-anchored estimator](research/stance-anchored-estimator.md);
synthetic results: [stance-heading-results](research/stance-heading-results.md).

- **Stance heading filter.** While a foot is standing still, the IMU leg chain has to reproduce
  how the headset actually moved. The mismatch estimates each sensor's heading drift, including
  the whole-body drift that joint constraints cannot see. It needs a live headset and movement:
  a perfectly still user gets no correction. It cannot see a strap that rotated around the limb.
- **Contact-aware legs.** A planted foot is held in place, the hips are shifted toward
  consistency with it, the knee is re-solved so bone lengths stay exact, and planted feet are
  kept flat. Foot contact is inferred, never measured.
- **Motion prediction.** Each sensor is extrapolated along its measured rotation rate to the
  moment of solving. A silent tracker means "still", never "keep turning".
- **Headset extrapolation** advances the headset position by its reported velocity over its age.
- **Recenter-proof calibration.** A headset tracking jump (including a recenter reported by a
  SteamVR transform change) rotates the stored heading references instead of discarding
  calibration; a transform change without a headset jump changes nothing.
- **Send angular velocity to SteamVR** is off by default: the frame SteamVR expects for that field
  is unverified. Linear velocity and pose age are always sent.

The Health page shows solve rate, packet-to-solve latency, per-sensor heading corrections with
their uncertainty, contact states and foot locks. Those numbers describe the model's internal
consistency, not tracking accuracy.

## Recovery and experimental correction

Recovery means **return to neutral, keep the head and torso facing together, then request heading recovery**. It waits for the required stable/fresh inputs and adjusts bounded yaw offsets while preserving mounting and body dimensions. Undo restores the previous yaw state.

**Hands-free tap gestures** (off by default; *Hands-free tap gestures (chest / hip)*). Tap the chest or hip tracker firmly with your fingertips while otherwise standing still:

- **Double tap**: heading recovery as above. Every outcome is spoken: "Resetting heading. Hold still, facing forward." on recognition, then "Heading reset." or why it was declined (for example "Heading not reset. Headset height is off. Stand up straight. Stand still and face forward, then double tap again."). Recovery needs a live (not desktop-preview) calibration; otherwise you hear "Heading reset needs a live calibration first."
- **Triple tap**: starts **Calibrate by movement** exactly like the Setup button (desktop preview or live, whichever is active). A triple tap during a running calibration cancels it ("Calibration cancelled."). A double tap during a calibration is ignored.

A gesture is classified only after the taps stop (0.7 s without another tap), so a triple tap is never first taken as a double; single taps and four or more taps do nothing. Taps must be short, sharp impulses on one torso tracker with a quiet moment between them; impulses that coincide with comparable impulses on other trackers (steps, jumps, sitting down), that rotate the tracker strongly (handling it in the hand), or soft pushes are ignored. *Last gesture* in the recovery section shows what was recognised or why a sequence was ignored. A tap is a request, never proof of the pose: the recovery still waits for stillness, upright headset at standing height and torso facing within 20° of the headset (8 s limit), and small head-only turns cannot be distinguished from common heading drift.

What was checked with real data (one user, local recording, firmware 0.7.2, about 100 Hz): the two worn chest taps in it peaked at 11 and 18 m/s² above the slow baseline, lasted 30–70 ms and were 0.24 s apart, with 1–7° of tracker rotation; the old detector did register its three recovery requests, but each was then declined or left waiting silently (headset at 0.75 m and tilted while kneeling; no headset at all for the other two, whose "taps" were the tracker being turned 33–58° in the hand). The recogniser's thresholds come from that recording; its behaviour on doubles, triples, slow taps, walking, jumps, handling, bunched packets and noise is covered by synthetic tests only. A deliberate worn triple tap and audio reaching the headset still need live validation.

The optional legacy knee correction is also off by default and is skipped whenever the stance heading filter is on, which has its own knee prior. It uses recent flexion and compatible joint-axis geometry to reduce bounded *relative* thigh/calf heading disagreement. It cannot determine which IMU drifted, cannot remove common heading drift, and can incorrectly reduce a real anatomical twist. The synthetic benchmark explicitly demonstrates this counterexample. The research plan proposes an estimator that tests the observability of each correction and uses independent evidence to reject harmful updates.

The core also contains a separate sliding-window **shadow estimator** and a [two-landmark heading experiment](research/contact-heading.md). These produce research hypotheses and diagnostics without changing live output. The shadow estimator reports the common-yaw nullspace and can reject a joint-model candidate when independently supplied physical evidence disagrees. Its raw-acceleration validation helper requires measurements the current stock firmware stream does not supply.

SlimeVR firmware suppresses unchanged rotations. A tracker that completed its handshake, reports a healthy sensor and keeps its packet sequence continuous can therefore show **Firmware-held orientation**: a low-confidence estimate based on the last measured rotation and continuity/liveness assumptions. This applies to any firmware version string, including DIY builds, and to extension sensors. The actual sample age remains visible, confidence is capped at 45%, and the heading filter never treats a held orientation as motion evidence (a planted foot stays locked through it). A lost packet revokes the hold only when it could plausibly have been the last rotation of a movement; sensor errors, reconnects and transport silence always revoke it. Gently rotate a tracker once if it is awaiting its first new measurement. This is compatibility with the firmware's behavior, not proof of a new measurement or a remedy for silent sensor failure.

## Outputs and coordinate conventions

- Core: right-handed meters, +Y up, -Z forward, XYZW quaternions.
- Firmware adapter: an explicit world-basis conversion, separate from mounting calibration.
- OpenVR bridge: versioned loopback protocol (v2 carries linear and angular velocity and pose age), WXYZ quaternion wire order and explicit raw/standing transform. The driver submits each pose from its own receive thread as it arrives. Invalid/stale targets are marked invalid after 100 ms; a tracker reports "connected, out of range" during transient loss and disconnects only after a second of silence from the app. **The app, helper and driver must come from the same build.**
- Output mode: Auto (default) and SteamVR send to the OpenVR driver, VRChat OSC sends OSC, Desktop only sends nothing. Exactly one sink is active at a time.
- VRChat OSC: optional mutually exclusive output to `127.0.0.1:9000`, reflected into Unity's left-handed space with Z-X-Y Euler application order. OSC has no implemented per-tracker invalidation message; stopping sends does not guarantee that VRChat immediately clears its last pose. Use SteamVR for the primary PC validation path.

The SteamVR helper reads the runtime's left/right controller-role poses. Steam Link can expose hand tracking, but hand-only availability through those legacy pose roles requires testing. This implementation does not fabricate missing hand anchors or claim a complete skeletal-hand API integration.

## Recordings and diagnostics

Profiles, diagnostic exports and recordings live in `%LOCALAPPDATA%/ChadFBT`. The output mode (default Auto) and the automatic eye-height switch are saved in the profile; nothing is sent until a calibration of this session is valid for live use. Profiles preserve assignments, dimensions and mounting data but require fresh session alignment because IMU yaw origins may change after reboot.

Use **Record session** before neutral calibration to capture replayable inputs, anchors, configuration changes and solver outputs. Diagnostic snapshots include identities, firmware, packet health and current estimates; they stay local. Recordings started after calibration require a new captured calibration for valid deterministic replay. Movement calibration is recorded as `motion_begin`, `motion_finish` and `motion_cancel` events and replayed through the same core calls; recorded frames carry the session's per-segment status (refreshed at most every 0.1 s). A possible SteamVR tracking-space event is recorded as `tracking_space_event` (with the transform's yaw when it was a pure yaw) and a desktop calibration carried over to the headset as `align_to_headset`; replay feeds them to the core, which reaches the same decisions from the recorded headset poses. An automatic eye-height update is recorded as a `configuration` record before its `calibrate` record. In Desktop only mode the recorded anchors are empty, exactly what the core saw. Older recordings with `rebase` events still replay.

Developer commands:

```powershell
cargo test --workspace
cargo run -p chadfbt -- --headless --seconds 30
cargo run -p chadfbt -- --replay C:\path\to\session.jsonl
cargo run -p chadfbt -- --analyze-heading C:\path\to\session.jsonl --report C:\path\to\new-heading-report.json
cargo run --release -p tracking-core --example benchmark
```

`--analyze-heading` streams an existing JSONL recording into the offline relative-heading shadow estimator before starting any single-instance service, tracker receiver or VR output. The report must be a **new file**; an existing file is never overwritten. The reader bounds line size and record count, and the JSON report is limited to 256 KiB. Malformed or incompatible input returns an error without creating a report. Missing original measurement timestamps in older recordings are excluded rather than treated as fresh data.

The report separates preview-calibrated, live-HMD and unanchored eligible pairs, counts held/reused/invalid inputs and epoch resets, and includes the last report and largest proposed model-relative angle for each leg. Production yaw corrections are removed before fitting. No independent witness is invented and no correction is applied. A model candidate cannot distinguish consistent real twist from drift or establish tracking accuracy. An empty or uncalibrated recording explicitly reports **zero eligible pairs**, rather than a successful tracking experiment.

`CHADFBT_NO_HELPER=1` prevents starting the anchor helper for isolated tests. A process running under a restricted execution account may have different network access from the normal desktop application. No Codex-managed firewall restriction is changed by this project.

## Build and package

Requires Rust stable, Visual Studio 2022 C++ tools and Windows SDK. The supplied direct MSVC build does not require CMake; a CMake project is also available under `driver/`.

```powershell
.\tools\build.ps1
.\tools\package.ps1
```

The release build statically links the Rust application's C runtime. The portable bundle contains the native application, anchor helper, official OpenVR client DLL and tracker driver. It is unsigned development software; code signing, installer/update design and broad Windows compatibility testing are pending.

## Source organization

| Component | Responsibility |
|---|---|
| `crates/tracker-protocol` | Original bounded firmware-wire parser and response encoders |
| `crates/tracking-core` | Calibration, shared kinematic chain, contact hypotheses, stance heading filter, leg IK, prediction, recovery, research shadow estimators, tests |
| `crates/steamvr-bridge` | Versioned local IPC and validation |
| `crates/chadfbt-app` | Network worker, recording/replay, profiles, OSC and native UI |
| `driver` | Original OpenVR driver/helper, build, registration, pinned upstream SDK |
| `docs/research` | Source-backed comparison and integration research |

Read [the research plan](RESEARCH-PLAN.md), [the core's exact behavior](../crates/tracking-core/README.md), [validation evidence](VALIDATION.md), the [SlimeVR source review](research/slimevr-internals.md) and the [literature review](research/literature-review.md). Measured foot articulation, body-proportion fitting, sensor clock mapping, mounting-twist estimation, torso acceleration heading and learned priors remain active research work. Claims of superiority require matched worn comparisons, not feature counts or synthetic tests.
