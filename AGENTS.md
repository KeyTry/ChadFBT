# ChadFBT engineering rules

Build an independent, native Windows FBT application. Keep firmware wire compatibility separate from original estimation and UI code. Preserve third-party licenses and pin downloaded SDK sources.

Use subagents for implementation and verification. Research platform/library APIs with Context7 and official sources before relying on them. Distinguish demonstrated behavior from experimental hypotheses in documentation and UI.

## Frames and ownership

- The tracking core consumes right-handed standing-space measurements: meters, +Y up, -Z forward, quaternions XYZW.
- SteamVR bridge owns all runtime raw/standing transforms and its WXYZ wire order.
- Firmware decoding owns packet validation; runtime owns identity, arrival times, frame conversion, and transport.
- Treat sensor orientations, inferred joint positions, contact hypotheses, and absolute anchors as different evidence.
- Keep calibration mounting rotations separate from slowly changing heading corrections.
- Never make a tracker valid after its required sensor or anchor goes stale. Never emit nonfinite data.
- A gesture requests recovery; it does not prove the user is in a calibration pose.
- Preview/simulation is visibly labeled and never sent to VR software as valid live tracking.

## Validation

Use meaningful synthetic regressions for numerical invariants, reference frame conversions, stale data, malformed wire packets, and recovery gates. Record live validation separately. Do not describe unit tests as proof of accuracy or performance against SlimeVR. Local captures and hardware identifiers stay in gitignored local-data.

Test fixtures use neutral, synthetic values. Never copy numbers from a real recording, saved profile or person into tests, code or docs: reproduce the mechanism a recording exposed, not its data. Body measurements count as personal data.

