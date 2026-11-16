# Identify a tracker by deliberate movement

The native **Setup** and **Trackers** pages include **Identify a tracker by movement**. This avoids matching a physical tracker to a MAC address by hand. It runs only when requested and never assigns a body role automatically.

1. Choose the body location you intend to assign, such as **Left calf**.
2. Keep the other trackers still. Press **Start 3-second identification**, then gently rotate only the intended tracker roughly a quarter-turn and back during the capture.
3. Watch the observed angular activity bars. Slow, deliberate motion works better than a fast shake because the UI receives snapshots and can miss motion between frames.
4. When one tracker clearly stands out, the app highlights its card and offers **Assign this tracker to [body location]**. Confirm only when you deliberately moved exactly that tracker.
5. Choose another location and repeat. Normal body-location dropdowns remain available.

No movement, several moving trackers, an unclear lead, device-list changes, packet-counter resets, an interrupted observation window, or a disconnected candidate produce a retry message and no assignment. The candidate must still be connected when confirmed. Cancelling or retrying does not change assignments.

Orientations quarantined after a receive backlog are excluded from identification even if queued packets have apparently recent arrival ages. Their cards say **Awaiting a genuinely new rotation after transport backlog**. Identification becomes usable only after the transport clears that state using a post-drain rotation.

Confirmation uses the same assignment policy as the ordinary dropdown: a tracker moves out of its previous role, and the new role replaces any tracker already occupying it. The candidate view names an existing assignment that would be replaced. Changing roles can invalidate neutral calibration, so complete identification before calibrating.

## What the activity bars mean

The comparison uses actual received rotation counters and normalized quaternion changes observed during a bounded three-second window. Quaternion `q` and `-q` represent the same orientation and contribute no false motion. A new, recent sample inside the capture establishes the initial baseline; the detector never credits a jump from an old pre-capture quaternion. Repeated firmware-held tokens do not advance rotation counters and add no activity. An actual packet-counter advance can contribute its original recent sample even if the UI's current provenance label has since changed to held.

The detector retains a small angular deadband anchor until displacement crosses it, allowing slow continuous rotation to accumulate while rejecting small stationary noise. The displayed **degrees per second observed** divide visible angular displacement by elapsed observation time. Packet-count magnitude is not a weight. These values are neither a full integrated motion path, an IMU sampling rate, nor an accuracy/confidence percentage.

The initial comparison requires at least 60 degrees of observed angular activity and several real observations from the leader, rejects a significant second mover, and requires a clear separation from the runner-up. These are conservative usability heuristics, not a learned body-location classifier. Physical body location comes from the role the user selected; off-body motion cannot establish anatomical placement.

The implementation is independent of tracker firmware changes and uses the existing native app model. It does not add another network listener or restart a running tracker session.

## Validation

Eleven deterministic detector tests cover a clear leader with stable IDs, no movement/noise, multiple/all movers, quaternion sign equivalence, firmware-held repetitions, candidate dropout and confirmation liveness, packet-counter reset, missed observation windows, old sample rejection, slow sub-deadband steps accumulating to 90 degrees, stale starting poses followed by an unobserved jump, and invariance to packet-counter magnitude. These tests establish the synthetic decision rules. Physical identification usability and worn-body assignment still need live validation.

A separate app-boundary regression verifies that quarantined queued rotations cannot identify or confirm a tracker, while subsequent clean motion can.
