# Validation record

Evidence collected on the user's Windows machine on November 14, 2026. This document separates software checks, hardware transport observations, and unverified tracking claims.

## Build and environment

- Rust 1.100.0 and Visual Studio 2022 C++ tools were already installed.
- Windows SDK 10.0.26100.7705 was installed using WinGet after native compilation found missing SDK headers/libraries.
- The original C++ OpenVR driver and anchor helper compile. Native wire-format/rigid-transform tests and DLL factory smoke checks pass.
- The initial packaged release, including the AccessKit accessibility integration, built successfully. `cargo fmt` verification, workspace Clippy across all targets with warnings denied, and the full workspace test suite passed: **63 tests** (15 app, 4 bridge, 10 protocol, and 34 tracking core). Subsequent development is recorded below.
- That initial portable app executable was **14,781,440 bytes** and used a statically linked runtime. The resolved dependency-license inventory contained **159 entries**.

## Actual tracker transport

Initial isolated Python probes received no UDP packets. Inspection found a Codex sandbox-account inbound firewall block; it was not disabled or modified.

The native application subsequently received all six physical trackers. A UI-exported diagnostic snapshot at 44.9827 seconds uptime recorded firmware **0.7.2** on all six, **1,475 received packets**, and **zero malformed packets**. All supplied finite orientation, acceleration and battery data. The trackers were off-body, unassigned, and output was disabled. No live headset anchor existed in that sample.

The final local raw capture, `local-data/release-wire.jsonl`, contained **10,685 datagrams** with **six complete handshakes** and **zero decode errors**. Each handshake reported firmware **0.7.2**, protocol **21**, and IMU enum **13**, identified as LSM6DSV by the inspected firmware enumeration. These are device-reported metadata, not independent physical inspection of each sensor.

A later diagnostic export from the final packaged app at **52.6173 seconds** recorded **six trackers**, **1,511 received packets**, and **zero malformed packets, sequence gaps, duplicates, or out-of-order packets**. The UI showed six connected of six discovered, distinguished firmware-held estimates and their raw rotation age, and showed no IP-based duplicate tracker entries. These observations validate the exercised transport and reconnect presentation, not body-position accuracy or continuous sensor health.

The live stream showed sparse orientation updates while the devices rested. A subsequent raw local capture confirmed frequent temperature-only bundles and paired rotation/acceleration updates. Official firmware v0.7.2 uses a compile-time setting to suppress quaternion updates below its change threshold. See [stream observability](research/tracker-stream-observability.md) for the assumptions and limits of any compatibility hold.

The capture also exposed a receiver restart issue: orphan packets could be admitted before handshake and identified by IP. The receiver was changed to accept identities only after a complete handshake, avoid keeping unknown sessions alive, and follow MAC identity across endpoint changes. The later packaged-app run above showed six trackers without the observed IP-based duplicates; a DHCP address-change exercise was not performed.

## Actual SteamVR registration

SteamVR's local `vrserver.txt` records the following in the user's Steam Link session:

- **22:33:46:** loaded the ChadFBT server driver from the project driver DLL.
- **22:33:48:** finished adding all eight stable tracker serials: hip, chest, left/right feet, left/right knees, and left/right elbows.
- **22:35:14:** runtime exited gracefully.

This verifies runtime driver loading and device registration. Body output was not calibrated. Tracker roles, raw/standing-space pose alignment, loss/recenter behavior and VRChat avatar motion still require a worn headset session.

Existing SlimeVR and BodyTracking driver registrations were preserved. `steamvr.activateMultipleDrivers` was enabled with the prior `steamvr.vrsettings` saved under gitignored `local-data/steamvr-backup/`.

## Native UI and capture checks

Computer Use directly exercised the native window. It exposed and led to fixes for an inherited horizontal layout, a rendering toggle incorrectly coupled to offline calibration, and an initial window size that placed footer controls outside the usable desktop area.

The live sensor axes and perspective floor grid render. Dragging changes the orbit camera. The diagnostic export button saved a readable snapshot. The record and stop buttons created a local recording; replay processed **11,949 frames** with zero output position difference. Those frames were uncalibrated and had zero valid body targets, so this demonstrates capture/replay plumbing only.

In the final packaged app, Hide to tray removed the main window while process **38360** continued running. A secondary launch exited successfully, restored the window for the same process, and the received-packet counter continued to **2,260**. Restoring by directly clicking the tray icon was not tested.

## Synthetic estimator evidence

The following synthetic benchmark is a **pre-firmware-held-path snapshot** and has not been rerun against the final held-orientation implementation. The optimized core benchmark produced valid finite outputs for 120,000 timed synthetic frames. Median solve/frame construction time was 1.5–2.3 microseconds; p95 was 2.4–4.4 microseconds across six/ten sensors and correction off/on. This excludes input handling, transport, UI, driver work and display latency; it is not an end-to-end latency measurement.

A 15-degree relative drift scenario fell to a 6-degree residual under the experimental hinge correction, while the identical observations interpreted as real non-hinge movement acquired 9 degrees of error. Common-mode heading drift was unchanged. These results justify keeping the experiment optional and illustrate why residual reduction alone cannot prove more accurate tracking. Full method and results: [synthetic benchmark](research/synthetic-benchmark-results.md).

## Remaining acceptance work

- Verify live HMD/controller anchors, body role assignment and mounted calibration. SteamVR was closed at the time of the final packaged-app check; the requested worn-user session was still pending.
- Check actual motion direction, proportions, mounting stability, latency and foot estimation while worn.
- Verify SteamVR roles and standing-space output, then VRChat avatar calibration and motion.
- Test physical double taps, accidental triggers and recovery under real drift.
- Test restoring the window by directly clicking the tray icon; secondary-launch restoration was observed.
- Measure long-session performance and compare against properly calibrated SlimeVR stable/current RC.

No superiority over SlimeVR, optical tracking, or published learned estimators has been demonstrated.

## Follow-up development while worn testing is deferred

The user deferred the worn test and requested continued development. The following changes can be evaluated independently of that physical session:

- Guided three-second movement identification in the native Setup and Trackers pages. It proposes a stable device identity for a user-selected role and requires confirmation; ambiguous or interrupted captures leave assignments unchanged. Twelve detector/UI-boundary regressions cover slow motion, quaternion sign equivalence, stale starting poses, held repetitions, ambiguity, dropout, and transport quarantine.
- A separate robust sliding-window heading estimator, with explicit common-yaw nullspace and conditional model-fit diagnostics. Twenty-four regressions cover ambiguity, outliers, timing, geometry, bounded independent evidence, and shared-joint acceleration physics. Its reproducible synthetic example is documented in [heading-shadow.md](research/heading-shadow.md). It does not modify live output.
- A two-landmark contact-heading hypothesis with ten regressions and an explicit placement-error bound. It does not prove contact and is not enabled as live recovery. See [contact-heading.md](research/contact-heading.md).
- Cross-language bridge verification uses the app's actual target mapping and Rust serializer, a shared checked-in wire fixture, and the actual native receiver and pose conversion. The offline harness passed. This adds evidence for coordinate mathematics and packet handling, not a new live SteamVR session.
- Receive-budget exhaustion and receiver pauses now quarantine cached data while queues drain. Sequence watermarks survive invalidation and advance for validated same-session packets discarded during draining. Tracker data becomes eligible only after a new measured rotation. Recent cached data from a prior tracker session is invalidated on reconnect.
- PE import inspection found Windows system/API-set dependencies only in the app, native driver, helper and bundled OpenVR client DLL. SteamVR remains an installed runtime dependency; static import inspection is not testing on a clean Windows machine.

Final integrated Rust verification passed **127 tests**: 42 application, 7 bridge, 10 protocol, and 68 tracking-core tests. Workspace formatting and all-target Clippy with warnings denied passed. The native build also passed the wire-format, receiver-fixture and driver-factory checks; the vendored OpenVR header emits unused-parameter compiler warnings.

The new offline analyzer processed the actual initial recording: **12,176 records and 11,949 frames**, yielding **zero eligible measured leg pairs**. It wrote a bounded report marked `no_eligible_pairs`, `accuracy_proof=false`, and `network_output_sent=false`. This confirms the uncalibrated capture is not being promoted into tracking evidence. Ten application regressions cover analysis, correction removal, input limits, no-overwrite behavior, source distinctions and invalidation.

The initial driver-loading log and hardware captures above predate these follow-up changes. Worn pose accuracy, physical movement identification and VRChat motion still require the user's later session.

The updated portable app was launched and inspected through Computer Use. It reconnected to **six of six trackers**. A three-second capture while the trackers rested returned **No clear movement observed**; all six roles remained unassigned and output remained off. A UI export at **204.6414 seconds** showed **5,996 received packets**, zero malformed packets, gaps, duplicates or out-of-order packets, and one transport-reset event. The app had recovered to six connected trackers; the transient reset and its precise cause were not captured, so this is not a complete live fault-injection test.

The packaged executable's offline analysis command also ran successfully while the native receiver continued in its existing process. It produced the same **11,949-frame, zero-eligible-pair** result without starting a second receiver. A subsequent layout-only adjustment moved Retry/Cancel above the identification activity list; workspace formatting and all-target Clippy were rechecked before the release rebuild. Computer Use verified the rebuilt package again: the resting capture returned **No clear movement observed**, both **Try again** and **Choose another location** were visible above the activity list, and choosing another location returned to the ready setup form. Six trackers remained connected, assignments unchanged, and output off.

## Calibration investigation — November 15, 2026

The user reported grey SteamVR trackers and a neutral-calibration button that appeared to do nothing. Computer Use inspected the running app and found all six roles assigned, a live HMD and two hands during one observation, output off, and repeated Health messages rejecting headset height. The saved height turned out to be the user's total body height rather than floor-to-eye height. The old one-shot command mislabeled a rejection as “Calibration waiting” and did not show its result beside the Setup button. The actual HMD height at the failed attempt was not recorded, so total-height entry alone does not establish the full magnitude or cause of the discrepancy.

Current SteamVR logs confirmed that the driver loaded and registered eight virtual devices in the user's sessions. With output off and no completed calibration, the driver's disconnected/invalid poses explain the grey state; this is distinct from driver registration failure. Those sessions later disconnected through Steam Link. No live calibrated tracker pose was established during this investigation.

The follow-up introduces a three-second countdown with up to three additional seconds for stable inputs, durable results in Setup, explicit delayed headset-height capture, and desktop body preview independent of incidental headset input. Successful neutral calibration recenters the preview camera. Desktop mode keeps a fixed synthetic reference until explicit live alignment; it does not yet implement walking displacement. Neither a calibration result nor a desktop skeleton is a measurement of anatomical tracking accuracy.

Integrated verification passed **147 Rust tests** (58 application, 7 bridge, 10 protocol, 72 core), workspace formatting, and all-target Clippy with warnings denied. The new coverage includes countdown/deadline/cancellation behavior, clock discontinuities, current valid headset-height capture, unchanged numeric height-gate boundaries, and desktop isolation from incidental HMD/controller poses. The native helper's new read-only `--inspect` mode compiled with MSVC and returned the expected nonzero exits for invalid arguments and absent SteamVR without launching the runtime; live inspect and graceful SteamVR-quit handling remain unverified. Audio uses a bounded separate thread and the Windows default audio device; actual audibility/routing requires the user's check.

Computer Use exposed a further layout defect: conditional firmware-held status rows changed the heights of tracker cards, moving controls farther down the Setup page while the user clicked. Assignment cards now reserve a fixed-height clipped status line. Formatting and Clippy were rechecked after this layout-only change, and the final release was rebuilt and launched. All six trackers reconnected with the user's six assignments preserved. The native app displayed **Neutral pose: 3…** after a calibration request, then a durable missing-HMD timeout in live mode. No calibration was accepted and output remained off. Switching to **Desktop body preview (no headset)** cleared the obsolete result and left its calibration button ready. Stored height, assignments, mounting data and preferences were unchanged. Cancellation is covered by the controller tests; a separate successful GUI cancellation and actual standing desktop/live capture were not completed in this check.

## Direction calibration and desktop displacement — November 15, 2026

In the subsequent worn desktop test, neutral capture succeeded and the user confirmed that the reset and audible tone worked. The user also reported incorrect movement directions. This is consistent with a mathematical defect in neutral-only mounting: independent sensor yaw can rotate the inferred bend axes even when the neutral skeleton looks aligned. A bounded local capture review found finite preview outputs, two desktop calibration events, and no valid live targets; those observations do not validate anatomical movement direction.

The new primary procedure captures neutral and then a forward skiing pose. It solves sensor-world heading separately from physical mounting, checks each endpoint window for sufficient and consistent rotation, and commits all six core mounts together. Synthetic counterexamples explicitly preserve the fact that an incorrect sideways pose can mimic a different heading. The viewport includes a forward arrow and facing-relative camera controls. A separate advanced editor supplies per-tracker paired heading adjustment, physical XYZ mounting trim, undo/reset, and explicit user confirmation. Fresh calibration after application restart currently requires repeating the direction procedure or manual adjustments.

Desktop preview now optionally estimates displacement from inferred floor and planted-foot contact. It is enabled by default in the app, remains experimental, and is independent of the live HMD position path. Calibration resets its origin. Stale/held orientations, time discontinuities, or abrupt leg geometry freeze displacement and release contact assumptions. Tests exercise crouching, stance/swing and support changes, preserved bone geometry, invalid live targets, reset boundaries, and uniform turning versus small common yaw during leg flexion. Sliding, jumping, strap movement and incorrect contact assumptions can invalidate these estimates; this is not measured room-scale position.

Combined verification passed **177 Rust tests**: 62 application, 7 bridge, 10 protocol, and 98 tracking-core tests. Workspace formatting and all-target Clippy with warnings denied passed. Independent source review found no remaining release blocker in the reviewed direction/manual/desktop-motion integration. The new tests establish numerical and state-machine behavior only. A worn two-pose capture, movement-direction check and desktop walking test of this new build remain pending; live SteamVR and VRChat accuracy are also unverified.

The static release build completed successfully and the refreshed portable executable was launched. Its SHA-256 is `2C9AFDB5445444D8D5FF11A455350403CF8A6C2C69CC01EB3E8BEABF370B6F61`; an independent comparison confirmed that the packaged executable matches the release build and its accompanying setup documents match source. Exactly one ChadFBT receiver process was running. Computer Use observed six connected trackers, the new forward-bend instructions and pose sketch, the experimental movement checkbox, the floor arrow and camera-relative compass. The Behind camera button changed the viewpoint as expected. Desktop preview and experimental motion were enabled, output remained off, and neutral capture was left ready. The six saved role assignments and the existing configured eye-height value were preserved; that value is still a total body height and needs a proper floor-to-eye measurement for accurate scale.

Known presentation issue: before the first fresh desktop calibration, the motion status can say that the live anchor owns position even when desktop mode is selected. This reflects the uninitialized calibration mode, not an observed HMD anchor. No calibration was triggered during this launch check, and manual adjustment behavior and worn displacement were not exercised through the UI.

## Worn feedback and a more forgiving forward capture — November 15, 2026

The user reported successful worn desktop forward calibration, with confusing guidance about how much the pelvis should tilt. The saved schema-two profile confirmed direction calibration for all six core roles at that point. The subsequent live test reached a valid SteamVR HMD and hand stream, captured the standing headset height, and completed live neutral calibration. SteamVR restarted during the session; the app refused capture while its HMD anchor was unavailable and subsequently reconnected. The precise cause of the interruption was not established. The user reported that their Quest streaming connection can be unstable around idle transitions.

Live forward capture exposed excessive strictness: observed failures included Hip rotation-axis elevation of 19.6 degrees, Hip model residual of 7.8 degrees, and a moving Chest at the end of the short settling window. These are gate diagnostics, not anatomical measurements or proof that the user performed an incorrect pose. No successful live forward capture or valid SteamVR output was established in that run. The user explicitly preferred retaining the skiing procedure while making it more tolerant.

The follow-up changes forward capture only: axis elevation 15 to 25 degrees, residual 6 to 10 degrees, and orientation variation 2 to 4 degrees under defaults. It additionally requires at least ten degrees of actual inclination, so yaw-only motion cannot count as an informative bend. The 0.65-second sample window, health, freshness, acceleration, and eight-degree heading-consistency requirements remain. A three-second countdown now precedes up to ten seconds of settling for the skiing pose; neutral and height timings remain unchanged. Instructions explain that pelvis and chest angles need not match, and a model-fit rejection no longer automatically instructs the user to recapture neutral. The misleading uncalibrated desktop-motion status is suppressed in favor of the required calibration step.

Integrated verification passed **183 Rust tests** (65 application, 7 bridge, 10 protocol, 101 core), workspace formatting, and all-target Clippy with warnings denied. New synthetic cases admit an observation near the reported hip rejection, preserve weak-inclination/pure-yaw/excessive-motion rejection, allow a small forward-pose sway while retaining stricter neutral checks, and quantify nonzero heading error in one imperfect-pose model. Controller tests verify the longer fixed deadline without automatic retries after timeout. These changes still require a worn acceptance test; the tests do not establish superior accuracy.

The original native anchor helper also gained an explicit `--assign-core-roles` command, with an allowlist of the six core ChadFBT serials and read-back verification through OpenVR settings. Native fake-settings tests passed for exact roles, preservation of unrelated/elbow entries, idempotence, and write/read/mismatch failures in every slot. Conflicting CLI modes return an error. A live invocation found SteamVR had closed and correctly performed no assignment. While SteamVR remained closed, the six documented role keys were saved to its configuration with a backup and atomic replacement; read-back verified all six and preserved every pre-existing role. Runtime loading of those saved roles still requires the next SteamVR session.


## Estimator rebuild, audits and protocol v2 — November 15, 2026

This entry records software evidence only. **No worn session, no live SteamVR session and no SlimeVR comparison was run for anything below.** Research sources and their verification status are in [literature-review.md](research/literature-review.md), [slimevr-internals.md](research/slimevr-internals.md) and [calibration-methods.md](research/calibration-methods.md).

### Audits and fixes

Three independent read-only audits (estimation core, application layer, SteamVR driver and bridge) traced code paths and reproduced several findings with scratch programs. Confirmed and fixed:

- The headset eye point was used as the head pivot. In a reproduction, looking down 60° moved the whole inferred body 8.8 cm back and 5.3 cm down. The body model now has a neck pivot behind the eyes (`eye_forward_m`, default 0.08 m).
- The bridge sent targets from the socket it reads headset anchors on. On Windows a send to an unbound port makes the next receive fail with error 10054, so anchors were never accepted while the driver was not listening. Reproduced with a socket test; targets now use a separate socket and 10054 is skipped. The same error on the tracker socket no longer marks every sensor unhealthy.
- Stationary trackers could stay invalid indefinitely after a receiver pause, a full datagram budget, or a single lost packet, because the firmware does not resend an unchanged rotation. Held-orientation eligibility also required the exact firmware string "0.7.2". These rules were relaxed as described in the README.
- A reconnecting tracker wiped calibration even when unassigned. Calibration is now cleared only for an assigned tracker that rebooted; the pinned firmware source shows the packet counter survives a Wi-Fi re-handshake, which is used to tell a reboot from a reconnect.
- The driver published poses only inside `RunFrame`, with zero velocity and no pose age, and the headset helper's 10 ms sleep measured about 15.6 ms on this machine. The driver now has its own receive thread, the wire protocol (v2) carries velocities and pose age, and the helper uses a high-resolution timer at about 200 Hz.
- The app's fixed 10 ms loop stamped every sample in a batch with the tick time. A receive thread now stamps each datagram on arrival and the solver runs on arrival at 100–250 Hz.
- Undo paths that restored stale yaw snapshots, recovery writing yaw into uncertified optional roles, a knee-correction budget that counted recovery yaw, default body dimensions 2 cm taller than the default eye height, an unbounded sensor cache, and a future-timestamp lockout.

Not everything the audits raised was changed: per-frame allocation notes in the core were left, and a worker pause longer than 1.5 s still expires held leases.

### New estimator layers

Shared kinematic chain, contact hypotheses, stance heading filter, contact-consistent leg IK, time-based prediction with velocity output, headset extrapolation and recenter carry-over were implemented and integrated; the design is in [stance-anchored-estimator.md](research/stance-anchored-estimator.md). Synthetic results, including the scenarios the filter cannot solve, are in [stance-heading-results.md](research/stance-heading-results.md). End-to-end synthetic tests through the public solver API measured: walking with a 10° common-mode leg error falls below 3° from 4.35 s and ends under 0.3° (it stays at 10° with the filter off); head-only turns cause at most 0.06° of correction; with leg IK the published planted ankle stays within 1.5 mm while the forward-kinematic ankle wobbles 33 mm peak to peak. These numbers come from a generator that shares the estimator's kinematic model.

One integration decision was changed after review: a firmware-held leg (a leg assumed still because its tracker went quiet) keeps its contact and foot lock, while the heading filter still refuses held orientations as motion evidence.

Two research modules are present but do not affect tracking: the acceleration-heading shadow estimator ([acceleration-heading.md](research/acceleration-heading.md)) is not yet wired into the solver.

### Verification

- `cargo test --workspace`: **351 tests pass** (90 application, 14 bridge, 10 protocol, 237 tracking core). `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --all --check` are clean.
- The native build ran the wire-layout, shared pose-validity table, receiver v1/v2 fixture and tracker-role suites; all passed. The v2 fixtures were generated independently of the Rust encoder.
- Release benchmark, six sensors: solve p50/p95/p99 2.1/2.3/3.2 µs with the new layers off and 3.3/3.6/5.9 µs with all of them on. This is solve-only CPU time, not latency.
- The release application, helper and driver were rebuilt together and packaged. Packaged executable: 16,003,072 bytes, SHA-256 `65FB93607C40D3B21E2F29FD40551D4D6427BDDD464B00C4E945BE8C3A312B0E`. SteamVR's registered driver path is the project's `driver\chadfbt`, which holds the rebuilt DLL.
- The packaged app was launched with no trackers powered and SteamVR closed. Window captures showed the Setup, Health and expanded Estimator sections rendering: six estimator switches, the prediction-lead slider, the legacy knee correction reporting `superseded_by_stance_filter`, an idle solve rate of 94 Hz and 7–8 µs solve time, and zeroed contact and foot-lock rows. A headless four-second run printed a snapshot containing the new estimator fields.

### Not verified

- Any behaviour on a worn body: heading correction, contact detection, foot lock, flat feet, prediction, floor clamp.
- SteamVR's use of the submitted velocities and pose age, the input profile and render model, the driver receive thread under a real runtime, and the helper's actual rate.
- Recenter carry-over against a real recenter.
- The relaxed firmware-hold heuristics (25 ms gap rule, 2 Hz idle assumption) on real Wi-Fi.
- Packet-to-solve latency: no tracker was connected, so no samples were recorded.

### Movement calibration integrated as the primary method — November 15, 2026

Software and synthetic evidence only. **No worn calibration has been run with this method, with or without a headset**, and nothing here is compared with SlimeVR or with the skiing pose on real captures.

- `TrackingCore` now owns a movement-calibration session (`begin_motion_calibration`, `motion_calibration_status`, `cancel_motion_calibration`, `finish_motion_calibration`, `motion_calibration_available`; status also in `TrackingFrame.motion_calibration`). A finish commits through the same code path as `calibrate_forward`, which was factored out without changing its behaviour. The app's Setup page offers **Calibrate by movement** first in both desktop preview and live mode, with **Use held skiing pose instead** as a persisted fallback (`calibration_method`).
- New end-to-end tests through the public core API (`tests/motion_calibration_core.rs`, 9 tests), generator shared with the module tests (rigid segments, upright neutral, sensor worlds differing by yaw, 0.2° sensor noise, arbitrary per-sensor heading offsets and mountings):
  - Desktop preview, two bows and four knee lifts: ready 10.3 s after neutral; afterwards every published core segment matched truth within **1.06°** over 770 frames of further movement; `direction_calibrated()` true; no valid target in any preview frame; stance uncertainty widened to 10.2° (legs) and 6.3° (chest/hip).
  - Live, headset yaw at neutral 20° off the torso: neck-displacement forward 0.03° off, committed (fused) forward **6.59° off** (the yaw reference alone would be 20.0° off); segments followed that reference within 0.92°; live targets valid afterwards.
  - Firmware-held tokens, optional-role sensors and a core sample stamped before the neutral capture never reach the session; a failed finish leaves calibration, profile, published orientations and the session untouched; a 4° pelvis tilt blocks finishing with an error naming Hip; sessions end on clear, rebase, new neutral and profile restore; the skiing-pose `calibrate_forward` path still calibrates (segments within 1.5°); identical inputs produce bit-identical frames including the status.
- App: request-controller tests for the 3 s minimum, the 0.5 s continuous-ready hold, the 30 s timeout listing each blocking segment, move-only start and cancel at every stage; a recording containing `motion_begin`, `motion_cancel`, `motion_begin`, `motion_finish` replays with zero output difference and zero ingest-outcome mismatches.
- `cargo test --workspace`: **389 tests pass** (95 application, 14 bridge, 10 protocol, 270 tracking core). `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --all --check` are clean.
- The debug app was launched with no trackers and the SteamVR helper disabled; a window capture in desktop-preview mode showed the **Calibrate by movement** card (instructions, disabled primary button with 0/6 trackers ready, limits note and hover text) and the **Use held skiing pose instead** fallback. The movement phase, per-segment rows and tones were not exercised on screen because no trackers were connected.

Not verified: any worn movement calibration; whether real squats, bows and knee lifts pass the gates in the documented times; the size of knees-out and toe-out bias on real legs and whether the stance filter removes it in practice; the headset displacement reference with a real headset and neck; the audio cues on real hardware.

### First worn desktop run of movement calibration — November 15, 2026

One user, six trackers on firmware 0.7.2, desktop preview (no headset), packaged build with SHA-256 `63A9AF7C…A64821`. No recording was made, so this is a single diagnostic snapshot plus the user's report, not a replayable session.

- **Transport, measured by the app:** 134,527 packets received with zero rejected, missing, duplicate or out-of-order datagrams and zero transport resets; solve rate 216 Hz; packet-to-solve delay 2.1 ms median and 4.4 ms at the 95th percentile; per-tracker rotation rates 84–100 Hz while moving. These are the app's own counters. They exclude firmware and Wi-Fi delay.
- **First attempt timed out.** The blocker was the thigh/calf bend-direction agreement gate: the left thigh and calf differed by 27° against a 25° limit.
- **Second attempt was accepted** for all six trackers. Reported by the module for that run: maximum tilt 44–104° per segment, planarity 0.0006–0.0023 (limit 0.15), repeatability 0.06–1.5° (limit 8°), model residual 0.7–1.7° (limit 12°), left knee direction disagreement 15.3°. The right calf's one-sidedness was 0.867 against a 0.85 threshold, which is closer to failing than the synthetic scenarios suggested.
- **User report:** "It works pretty well, it's easier than the skiing pose."

Changes made because of this run: the thigh/calf agreement limit was raised from 25° to 40° (a body turn has its own detector), and the repeated "SteamVR anchor helper exited" message is no longer shown in desktop preview.

What this does not establish: that the resulting bend directions are anatomically correct (no reference measurement), how often the calf's sign comes close to undetermined, or anything about the headset-assisted variant.

### Automatic SteamVR connection and calibration that survives SteamVR — November 15, 2026

Software and synthetic evidence only. **None of this has been exercised against a real SteamVR restart, an idle-headset shutdown, a controllers/hands input-mode switch or a recenter yet**, nor with a worn headset aligning a desktop calibration.

Prompted by three reports: switching between controllers and hands with Steam Link broke the calibration; SteamVR exiting (the last session's log shows Steam Link quitting SteamVR after "Connection inactive for 30.17 > 30 seconds") and returning broke it; and going from desktop preview to SteamVR needed an extra manual step. The cause of the first two: the app cleared the calibration when the helper session changed after more than 2 s or when the raw-to-standing transform changed in a non-yaw way, and rebased by the transform's yaw otherwise, although that transform also changes when SteamVR re-originates its raw space while the standing space (in which the calibration lives) has not moved.

- **Nothing SteamVR-side clears the calibration any more.** The app reports a new helper session or a material transform change to the core as a hint (`TrackingCore::note_tracking_space_event`, recorded as `tracking_space_event`). The core decides from the headset's standing-space continuity (`tracking_space::decide`): plausible motion for the elapsed time (0.25 m + 4 m/s and 25° + 10 rad/s over the first 0.1 s, then 1.5 m/s and 60°/s; gaps up to 2 s are compared only after a hint) means nothing happened; a jump rebases by the transform's exact yaw when the observed yaw change agrees within 10° + 20°/s·dt, otherwise by the observed yaw; a slow change that is clearly the transform's yaw is recognised too. After more than 2 s without a usable headset sample the calibration is kept, the stance filter's uncertainty widened (legs 15°, chest/hip 8°) and the user told once by voice. An assigned tracker that rebooted still clears the calibration, now announced by voice.
- **Helper supervision.** Respawned every 5 s; an alive helper that has produced no anchor frame for 20 s is stopped and respawned; "SteamVR is not running" and OpenVR initialization refusals are routine and never reported; other failures are reported only after six in a row and at most every 10 minutes. One message when SteamVR anchor frames start or stop.
- **One output mode, persisted, default Auto** (Auto / SteamVR / VRChat OSC / Desktop only), replacing the per-launch output selector and the desktop-preview checkbox. Auto runs live and sends to SteamVR while a worn headset is available and the calibration is valid for live use, and shows the desktop preview otherwise. A calibration started without a worn headset is a desktop calibration. Desktop only feeds the core empty anchors (also in the recording).
- **Desktop calibration carried over to VR** (`TrackingCore::align_preview_to_headset`, recorded as `align_to_headset`), attempted automatically every 0.1 s while a worn headset is present, with "Headset found. Stand straight and look ahead." once and "Aligned." on success.
- **Automatic eye height** at live neutral captures (setting, default on): adopted when the stable headset height differs by more than 1.5 cm and is within the live gate's ±0.20 m tolerance, atomically with the capture (`calibrate_neutral_with_config`); a change over 5 cm is announced.
- Tests: 14 new core tests (`tests/tracking_space.rs`: the pure decision cases, a 70° recenter across a reported 1 s gap rebased by the transform yaw with targets rigidly rotated within 2 mm/0.11°, a transform change without a headset jump leaving every target unchanged, a 30 s gap keeping the calibration and widening the filter, a session change alone keeping it, preview calibrations ignoring hints, alignment equal to a live calibration in the same pose within 1 mm and 0.1°, alignment gates while swaying, looking down, in the skiing pose and with legs turned 40°, mounting untouched, and the height-adopting capture leaving configuration and calibration untouched on rejection); 6 mode-machine tests; 4 app tests (event classification, auto-height range, helper exit classification, and an Auto session — desktop calibration, automatic alignment, a 70° recenter, SteamVR gone for 3 s and back, an auto-height neutral capture — recorded and replayed with zero output difference). `cargo test --workspace`: **453 tests pass**; clippy with `-D warnings` and `cargo fmt --all --check` are clean.
- A headless run without SteamVR showed "Auto: desktop preview (no headset)" and no helper failure messages.

Not verified: any of the above with real SteamVR, Steam Link, a worn headset or trackers; whether SteamVR's standing space really is the same physical frame after a SteamVR restart (the long-gap assumption); whether a controllers/hands switch changes the transform, the helper session or the headset pose; the plausible-motion and agreement thresholds against real head motion and real recenters; the worn-headset height threshold (45 cm below eye height) for Auto; the new spoken prompts in a headset.

### First live SteamVR session and the fixes it prompted — November 15, 2026

One user, six trackers on firmware 0.7.2, Quest over Steam Link, VRChat. Packaged build SHA-256 `A7799F10…D3F084`. A 22-minute recording was made; the headset was live for 4 min 15 s (t = 238.9–493.8 s). An input-only excerpt is kept locally under gitignored `local-data/recordings/` and can be re-solved with `chadfbt --replay <file> --dump <out>`.

**What worked, as observed in the logs and reported by the user.** SteamVR loaded the rebuilt driver from the project path and added the virtual trackers; the helper connected on its second attempt once the wireless headset was up; movement calibration completed live 5.5 s after the neutral capture; the user reported "for the most part, it seems to work" and that calibration is easy.

**Defects the session exposed.**

- *Legs pulled up when bending or squatting* (user report, confirmed in the recording). With the feet planted, the modelled hips rose 8–12 cm and moved back 18–42 cm in bows, and the modelled ankles rose 15–22 cm. Over all live frames the top-down chain was 9–16 cm too short once the head had dropped more than 30 cm, and no choice of segment lengths within the same structure removed it. The horizontal sign of the error (ankles moving back, not forward) rules out the thigh angles and points at the torso: in deep flexion the torso trackers read 65–94° where the hip-to-neck line needs about 60°. The hip tracker read more tilt than the chest tracker throughout.
- *False chest heading correction.* The stance heading filter turned the chest by −22° within four minutes, far beyond this IMU's drift: torso model error was being absorbed as chest yaw. Contacts were also released during bows, so the foot lock was off when it was needed.
- *An extra tracker between the legs in VRChat* (user report). The two unused elbow slots were registered with SteamVR and reported as connected without a pose, at the tracking-space origin.
- *"Double tap does nothing"* (user report). The recording shows the tap was detected; the recovery it requested was then left waiting or declined (headset far below standing height and tilted about 100° in the one live case; no live headset in the other two) with no feedback audible in the headset.
- *SteamVR exit on idle.* The SteamVR log shows Steam Link quitting SteamVR after "Connection inactive for 30.17 > 30 seconds". This is routine behaviour, not a crash, and the app treated it as a reason to discard calibration.

**Changes, each verified as stated.**

- Support-foot floor constraint with a torso compliance budget, tilt-scaled lock and root limits, and a torso-motion allowance for contact detection. Re-solving the recorded session: ankle height while bent, 95th percentile 14.6 cm → 0.6 cm; frames floating more than 3 cm 7.0 % → 0.4 %; hip rise in the two deepest bows +12.4/+9.2 cm → +0.3/+0.7 cm; frames with a locked foot during bows 43–56 % → 100 %; knee lifts still rise 47–74 cm with the support ankle within 1.3 cm of the floor. Synthetic "flexed spine" regressions cover the same mechanism. The constants are fitted to this one session.
- The heading filter estimates leg headings only from displacement and discounts measurements taken while the torso moves. Chest correction at t = 480 s on the same recording: −23.2° → −2.7°. The leg corrections remain around ±10° with different signs before and after the change, and the recording has no reference for leg heading, so **the leg corrections are unverified**.
- The driver registers a tracker only after its first valid pose and holds the last valid pose during transient loss. Verified by native tests only.
- Multi-tap recogniser (double = heading recovery, triple = full movement calibration), every outcome spoken through Windows speech synthesis. The recording contains no deliberate tap made while standing still; thresholds come from the few real impulses plus synthetic tests.
- Calibration is no longer cleared by SteamVR-side events, and an Auto output mode connects without a manual step (previous subsection).

**Not verified:** any of these changes on a worn body; whether the extra tracker is gone in VRChat; whether spoken prompts reach the headset; a deliberate double or triple tap; behaviour when seated (an upright seated torso still floats the feet by 3–11 cm in the recording); a real SteamVR restart or controllers/hands switch.
