# WiFi tracker wire protocol research

Verified 2026-11-14 against official upstream source, independently implemented in `tracker-protocol`. ChadFBT uses the trackers' existing firmware; it does not run or embed the SlimeVR server or solver.

## Sources and provenance

The inspected firmware revision is [`9fae27ac71e0f8236a72f890da213f7dbf3f6908`](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/tree/9fae27ac71e0f8236a72f890da213f7dbf3f6908). The inspected server revision is [`83941fd38e91cc91ca6b360deab5c2ae986dd1b6`](https://github.com/SlimeVR/SlimeVR-Server/tree/83941fd38e91cc91ca6b360deab5c2ae986dd1b6). Source was consulted for interoperability, with original Rust and Python implementations written here.

After the native application connected to the real trackers, their firmware string identified **0.7.2**. Its release tag resolves to [`c12c475fe067e74005e7dd83999bc3a19b478b15`](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/tree/c12c475fe067e74005e7dd83999bc3a19b478b15). The [version-specific sensor enum](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/c12c475fe067e74005e7dd83999bc3a19b478b15/src/consts.h) maps LSM6DSV to **13**. Firmware text alone does not prove the hardware sensor type; retain the handshake and sensor-info IMU fields for confirmation. Both fields are already decoded by the Rust protocol crate.

- [Firmware connection and session handling](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/9fae27ac71e0f8236a72f890da213f7dbf3f6908/src/network/connection.cpp)
- [Firmware packet structures](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/9fae27ac71e0f8236a72f890da213f7dbf3f6908/src/network/packets.h)
- [Firmware feature flags](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/9fae27ac71e0f8236a72f890da213f7dbf3f6908/src/network/featureflags.h)
- [Official server packet parser](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPProtocolParser.kt)
- [Official server packet bodies](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPPacket.kt)
- [Official server axes conversion and keepalive](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/TrackersUDPServer.kt)
- [Official tap bit definition](https://github.com/SlimeVR/SlimeVR-Server/blob/83941fd38e91cc91ca6b360deab5c2ae986dd1b6/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/SensorTap.kt)

## Framing and session

UDP port 6969. Tracker-to-host datagrams begin with big-endian `u32 packet_type`, `u64 sequence`, then the payload. Bundles share one outer sequence. A handshake resets the sequence; ping echoes carry the server's zero sequence and must not be counted as tracker reordering/loss.

The host can broadcast type 0 on active private IPv4 interface subnet broadcast addresses every two seconds. Firmware searches for a host and broadcasts a type 3 handshake while disconnected. Host replies to the source endpoint. Never share the port with another tracker server.

Two host responses intentionally omit the normal header:

- Handshake: 64 bytes, starting `03` followed by ASCII `Hey OVR =D 5`, then zero padding.
- Sensor info ACK: big-endian `u32=15`, then sensor ID and status, exactly six bytes.

Host heartbeat is normal-header type 1 with sequence 0 every 500 ms; firmware responds type 0. Firmware disconnects after roughly three seconds without host traffic. Type 22 feature flags use the normal header; bit 0 in the host payload enables type-100 bundling. Do not send configuration, firmware, reboot, or flash commands as part of connection.

## Packet bodies

All multi-byte values are big-endian. `f32` means IEEE binary32. Strings and identifiers must be length-checked against datagram bounds.

| Type | Body |
| --- | --- |
| 3 | Board u32, legacy IMU u32, MCU u32, three unused u32, protocol u32, firmware byte length u8 + firmware bytes, MAC six bytes, optional tracker/vendor/product metadata. Empty legacy handshake is possible. |
| 4 | Acceleration xyz f32, optional sensor ID u8 (defaults 0 in old senders). |
| 10 | Ping ID u32; firmware echoes the entire host packet. |
| 12 | Battery voltage f32 + level f32, or legacy level-only f32. |
| 13 | Sensor ID u8 + tap bits u8. Bit 0x40 denotes a double tap in the official server. |
| 14 | Sensor ID u8 + error code u8. |
| 15 | Sensor ID u8, status u8, then version-dependent IMU u8, config u16, rest-calibrated u8, default body position u8, data type u8. Current firmware appends two debug f32 rates. |
| 17 | Sensor ID u8, rotation type u8 (1 normal, 2 correction), quaternion **xyzw** f32, accuracy u8. |
| 19 | Sensor ID u8 (255 means device-wide), signed RSSI i8. |
| 20 | Sensor ID u8, Celsius f32. |
| 21 | User action u8: 2 full reset, 3 yaw reset, 4 mounting reset, 5 pause. Treat as an input request with explicit product policy. |
| 22 | Feature bit bytes. |
| 23 | Sensor ID u8, quaternion xyzw i16 divided by 32768, accel xyz i16 divided by 128. Supported by current server; inspected ESP firmware emits 17/4 instead. |
| 100 | Repeated u16 length + inner u32 packet type + body. Inner length includes packet type; no inner sequence. |
| 101 | Repeated u8 length + inner u8 packet type + body. Current server supports it; inspected ESP firmware uses 100. |
| 105 | Inspection subtype u8, sensor ID u8, encoding u8, then gyro xyz + accuracy u8, accel xyz + accuracy u8, magnetometer xyz + accuracy u8. Each xyz is three 32-bit words. Subtype 1 is raw IMU; encoding 1 is sign-extended integer samples, encoding 2 float. |

Legacy orientation types 1 and 16 contain quaternion xyzw f32 for sensors 0 and 1. ChadFBT also preserves unknown types for diagnostics. Length overflows, nonfinite values, zero quaternions, malformed bundles, and excessive bundle counts are rejected without unsafe code or recursion.

## Coordinate and raw-data limits

Wire orientation is Z-up. The official server converts to its Y-up basis by **left-multiplying** `qX(-π/2) * wire_quaternion`. This is an axes offset, separate from body mounting calibration. Do not apply it twice. Current parser preserves wire order and values; the runtime/solver owns conversion and normalization.

Official acceleration handling uses the vector directly for protocol >=22. For older firmware, it applies a -90° Z rotation (`SENSOR_OFFSET_CORRECTION`) to compensate historical sensor mounting behavior. The specific installed firmware is unverified; acceleration should not drive unconstrained position integration. No claim of absolute IMU position is justified.

Ordinary tracker traffic supplies fused orientation and acceleration, not a continuous raw gyro feed. Inspection type105 exists only when firmware is built with `ENABLE_INSPECTION`, and its units depend on sensor implementation. It is parsed if present, but ChadFBT does not enable it or assume it exists. No firmware changes are made. Tap support must likewise be observed on the installed device; the existence of packet13 does not prove LSM6DSV firmware emits it.

## Live LAN check and reproducibility

`tools/probe-trackers.py` exclusively binds UDP6969, bounds duration/samples, uses configured private local interfaces, acknowledges sessions, and stores bounded raw samples locally. It never scans address ranges, changes firmware/configuration, stops a server, or alters firewall rules. Optional `--peer` values are explicit known tracker IPs from the existing neighbor table.

Two initial 30-second Python attempts (broadcast and six targeted Espressif neighbor entries) received **zero datagrams** and captured no tracker samples. Two candidate trackers answered ICMP on the active subnet. UDP6969 had no listener before either attempt. An active Windows inbound Block rule named `codex_sandbox_offline_block_inbound` covers all programs/profiles for a restricted local user, alongside an enabled UDP6969 Allow rule. The sandbox rule's security filter is user-scoped, not a machine-wide block for every Windows user. A block takes precedence for matching traffic; no sandbox rule was modified or bypassed. The initial zero-packet result does not establish its cause.

A subsequent native ChadFBT launch **successfully discovered all six devices** without firewall or firmware changes. An exported application snapshot at 44.9826555 seconds uptime recorded **1,475 received packets and zero malformed packets**. Every tracker reported firmware **0.7.2** and supplied finite orientation, acceleration, and battery data. Trackers were off-body, unassigned, and uncalibrated; no HMD/controller anchor was connected, and output was off. This confirms direct device transport and decode, not body-tracking accuracy.

Resting orientation packets were sparse: the captured sample ages ranged from approximately 0.04 to 31.25 seconds, and instantaneous rotation packet rates were zero at that snapshot. Orientation arrival age is not interchangeable with device liveness or the IMU sampling rate. Firmware may suppress unchanged rotations. Use independently received transport heartbeat age and version-specific sending behavior to interpret held rotations. No controlled moving-body rate, raw-IMU stream, tap gesture, calibrated motion, recovery, or VR output was demonstrated in this capture.

ChadFBT therefore labels a separate **Firmware-held orientation** compatibility estimate for known firmware 0.7.2. It retains the original quaternion measurement timestamp and raw age; it does not manufacture fresh samples from heartbeats. Eligibility requires an actual received rotation, healthy reported sensor status, recent device traffic, and no intervening sequence gap/reordering/error. Loss of continuity revokes the estimate until new rotation evidence arrives; transport older than 1.5 seconds expires it. Core confidence is capped at 0.45, and experimental drift correction skips held input. This is a firmware-continuity assumption, not proof of current orientation or IMU health. UI connection, measurement age, calibration eligibility and held provenance remain separate.

Local evidence: `local-data/tracker-probe/summary.json`, `local-data/tracker-probe-targeted/summary.json`, and `%LOCALAPPDATA%/ChadFBT/diagnostics.json` (native export). These contain local environment details and hardware identifiers; do not commit them. Protocol unit tests passed 10/10, including every truncation of rotation payload, bundle length errors, nonfinite/zero quaternion rejection, raw IMU bounds, sequence behavior, and exact response bytes. Tests validate software fixtures; the separate native capture supplies the limited real-device interoperability evidence above.
