# Wire protocol v2

Both endpoints bind IPv4 loopback only. Core-to-driver goes to 127.0.0.1:39570; anchors helper-to-core goes to 127.0.0.1:39571. This is a **local prototype boundary**, not authentication: any program running locally can inject or deny tracking. Do not expose these sockets on a network. Port conflicts fail closed. The app sends targets from a separate ephemeral loopback socket, not from its anchor socket: on Windows an unanswered send queues an ICMP port-unreachable that fails the sending socket's next `recv_from` with WSAECONNRESET (10054). The app also treats 10054 on the anchor socket as a non-fatal, skipped read.

## Layout

All fields are little endian, with no padding. Quaternions are WXYZ. Poses, velocities and transforms are in OpenVR standing coordinates (meters, +Y up, -Z forward).

Header (76 bytes):

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | ASCII magic: `CFBP` (targets) or `CFBA` (anchors) |
| 4 | 4 | u32 version = 2 |
| 8 | 8 | u64 sender session, nonzero |
| 16 | 8 | u64 sequence, strictly increasing within a session |
| 24 | 4 | u32 flags |
| 28 | 48 | 12 x f32 row-major 3x4 raw-from-standing rigid transform |

Pose record (56 bytes, offsets relative to the record):

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | u32 valid (0 or 1) |
| 4 | 12 | position xyz, f32 meters |
| 16 | 16 | orientation wxyz, f32 |
| 32 | 12 | linear velocity xyz, f32 m/s |
| 44 | 12 | angular velocity xyz, f32 rad/s, axis-angle (direction = axis, magnitude = rate) |

Target frame (`CFBP`, exactly **528 bytes**): header; f32 `pose_age_secs` at offset 76; eight records at offset 80 + 56*i: hip, chest, left foot, right foot, left knee, right knee, left elbow, right elbow. Flags bit 0 = apply angular velocity; every other bit must be zero.

Anchor frame (`CFBA`, exactly **244 bytes**): header with flags = 0; three records at offset 76 + 56*i: HMD, left controller/hand, right controller/hand. The helper fills velocities from the same `TrackedDevicePose_t` (`vVelocity`, `vAngularVelocity`) as the pose. Velocities that fail the limits below are sent as zero with the pose still valid.

## Validation

A receiver rejects the **whole frame** for: wrong size for its version, wrong magic or version, session 0, nonzero undefined flag bits, a non-rigid transform, `pose_age_secs` that is nonfinite, negative or above 0.25, or any record (valid or not) with a flag other than 0/1, a nonfinite value, a position component beyond 1000 m, |linear velocity| > 50 m/s, |angular velocity| > 100 rad/s, or (for valid records) a quaternion whose squared norm is outside (0.99, 1.01). Invalid records still carry finite in-range values.

The Rust sender sanitises per pose, so one bad slot cannot cost the whole frame: a pose that is invalid, out of range, nonfinite, or has a non-normalisable quaternion is sent as valid=0 with zero position/velocity and identity rotation, and a valid quaternion is normalised. A non-rigid transform or out-of-range pose age sends an identity transform with every pose invalid. The case table `driver/tests/fixtures/pose-validity.txt` is run by both the native `valid_pose` test and the Rust `Pose::wire_valid` test.

## Driver behaviour

The driver converts targets into raw world space with identity world-from-driver: positions by the full raw-from-standing transform, orientations by its rotation, and velocities by its **rotation only**. It always sets `vecVelocity`. It sets `vecAngularVelocity` only when flag bit 0 is set: the frame of that field is unverified in SteamVR (Valve's driver documentation describes it as driver-world space, while Monado's SteamVR driver host treats it as device-local), so it stays opt-in until checked live. `poseTimeOffset = -(pose_age_secs + time since receipt in the driver)`.

A driver-owned receive thread waits on the socket (WSAPoll, 5 ms timeout), drains it, and calls `TrackedDevicePoseUpdated` immediately for each newly accepted frame.

**Lazy registration.** `Init` creates the eight tracker objects but adds none to SteamVR. The first time a slot's pose is valid in an accepted frame, the receive thread sets that slot's bit in an atomic pending mask; the next `RunFrame` (vrserver's main loop) calls `TrackedDeviceAdded` for it, once per driver `Init` (a rejected slot is logged and not retried). A slot that never carries a valid pose, such as an elbow without an upper-arm IMU, never appears in SteamVR or applications. Devices are never removed during a session.

**After registration**, once SteamVR has activated the device:

| Slot state | Submitted | Position / rotation |
|---|---|---|
| valid record in a live frame | once per newly accepted frame, never re-stamped; `Running_OK` | live, with velocities and `poseTimeOffset` |
| invalid record, expired 100 ms lease, or drain, while frames are still accepted within 1 s | once per transition; `deviceIsConnected=true`, `poseIsValid=false`, `Running_OutOfRange` | last valid, zero velocity, `poseTimeOffset=0` |
| no frame accepted for more than 1 s (app closed) | once per transition; `deviceIsConnected=false`, `poseIsValid=false` | last valid, zero velocity |

A device activated after its slot's first valid pose (activation is queued by SteamVR), or reactivated with a new index, receives its current state once. A slot the app stops driving stays registered and out of range at its last valid pose. `RunFrame` adds pending devices and drains driver events; it never submits poses.

## Freshness and replay

A session ID changes whenever a sender starts, and its sequence increases for every frame; a different session may take over only after the prior session is stale. The receiver measures freshness with its own monotonic clock. Timestamps from other processes are not compared. `pose_age_secs` is the sender's own estimate of sample age and is used only for `poseTimeOffset`.

Accepted target frames expire 100 ms after receipt in the driver. The app sends at 100 Hz, and SteamVR itself stops extrapolating 100 ms after a submission. Accepted anchor frames expire 100 ms after receipt in the app; the helper sends at about 200 Hz using a high-resolution waitable timer, falling back to `timeBeginPeriod(1)` with `Sleep`. If either receiver is not polled for its lease, or exhausts its 64-datagram polling budget, it invalidates its prior frame and drains the socket to empty before accepting new packets. Sequence watermarks survive invalidation; valid packets discarded while draining can advance the current session's watermark without extending the pose lease. Thus a previously accepted or observed duplicate from the same session cannot revive a pose after the drain. The eight most recently retired session IDs are also rejected on attempted rollback. This is a bounded replay guard: a delayed higher-sequence datagram, or a session older than the retained history, cannot be distinguished reliably without sender timestamps or authentication.

The helper inverts OpenVR's raw-to-standing transform to produce raw-from-standing. This mapping requires real SteamVR end-to-end verification, particularly after room setup or recentering. Core must withhold valid targets without a recent valid HMD, successful calibration, and valid required sensor observations.

Helper controller role enumeration uses legacy OpenVR roles; whether hand-only devices expose these roles depends on Steam Link and requires live testing. It does not read, create, alter, or replace hand skeletons.

## Legacy v1

The driver still accepts v1 target frames so an older app build keeps working; they are treated as zero velocity, zero pose age and zero flags. The app decoder also accepts v1 anchor frames from an older helper, with zero velocity. The app and the current helper send only v2.

v1 header (72 bytes): magic, u32 version 1, u64 session, u64 sequence, then the 12-float transform at offset 24 (no flags field). v1 record (32 bytes): u32 valid, position xyz, orientation wxyz. A v1 target frame is exactly 328 bytes (eight records at offset 72); a v1 anchor frame is exactly 168 bytes (three records). v1 has no pose age and no velocities; the other v1 validation rules are the same as v2.

## Verification

In a source checkout, offline cross-language verification is available with `powershell -ExecutionPolicy Bypass -File driver/test-bridge.ps1`. Its test sources and build tools are not included in the portable runtime bundle. The checked-in fixtures `target-v2.hex` and `anchor-v2.hex` were generated independently of the Rust encoder. The Rust serializer and the app's target mapping must reproduce `target-v2.hex`, which the native receiver used by the driver decodes and transforms. `target-v1.hex` pins the legacy layout and is still accepted by the native receiver. The native tests also cover the publish-once and out-of-range/disconnected transitions, lazy registration (unused slots never request a device; a slot requests once, on its first valid pose, from v2 or v1 frames), the held last valid pose, late activation and reactivation, and re-validation after a disconnect; the `RunFrame` call into SteamVR itself is not exercised offline. A Rust test sends targets to a closed port and still receives anchors. Tests use receiver-local synthetic monotonic times. They do not establish actual UDP delivery latency, SteamVR's runtime tracking-space mapping or velocity frames, prediction quality, tracker roles, input-profile loading, how SteamVR or VRChat display a tracker added mid-session or held out of range, or VRChat behavior.
