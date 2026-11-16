//! Loopback wire protocol (v2) for ChadFBT's SteamVR helper and driver.
//! See driver/PROTOCOL.md. Never assume an anchor frame is fresh without checking receipt time.
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::time::{Duration, Instant};

pub const DRIVER_PORT: u16 = 39570;
pub const ANCHOR_PORT: u16 = 39571;
pub const TRACKER_COUNT: usize = 8;
pub const ANCHOR_COUNT: usize = 3;
/// Version written by this crate. The driver also accepts v1 target frames.
pub const PROTOCOL_VERSION: u32 = 2;
pub const HEADER_BYTES: usize = 76;
pub const POSE_BYTES: usize = 56;
/// v2 target frame: header, f32 pose age, eight pose records.
pub const TRACKER_BYTES: usize = HEADER_BYTES + 4 + TRACKER_COUNT * POSE_BYTES;
/// v2 anchor frame: header, three pose records.
pub const ANCHOR_BYTES: usize = HEADER_BYTES + ANCHOR_COUNT * POSE_BYTES;
pub const TRACKER_BYTES_V1: usize = 328;
pub const ANCHOR_BYTES_V1: usize = 168;
/// Header flag bit 0 (targets only): the driver forwards angular velocity to SteamVR.
pub const FLAG_APPLY_ANGULAR_VELOCITY: u32 = 1;
pub const MAX_POSITION_M: f32 = 1000.0;
pub const MAX_LINEAR_SPEED_MPS: f32 = 50.0;
pub const MAX_ANGULAR_SPEED_RADPS: f32 = 100.0;
pub const MAX_POSE_AGE_SECS: f32 = 0.25;
const IDENTITY_TRANSFORM: [f32; 12] = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.];

/// A standing-space pose record. Rotation is WXYZ. Velocities are standing-space
/// linear (m/s) and axis-angle angular (rad/s) velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
    pub valid: bool,
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}
impl Pose {
    /// Invalid record with zero position/velocity and identity rotation.
    pub const fn invalid() -> Self {
        Self {
            valid: false,
            position: [0.; 3],
            rotation: [1., 0., 0., 0.],
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
        }
    }
    /// Exactly the native `valid_pose` rule (driver/src/wire.h). A record that
    /// fails it makes the receiver reject the entire frame.
    pub fn wire_valid(&self) -> bool {
        if self
            .position
            .iter()
            .any(|x| !x.is_finite() || x.abs() > MAX_POSITION_M)
        {
            return false;
        }
        if self.rotation.iter().any(|x| !x.is_finite()) {
            return false;
        }
        if !bounded_vector(&self.linear_velocity, MAX_LINEAR_SPEED_MPS)
            || !bounded_vector(&self.angular_velocity, MAX_ANGULAR_SPEED_RADPS)
        {
            return false;
        }
        let mut n = 0f32;
        for x in self.rotation {
            n += x * x;
        }
        !self.valid || (n > 0.99 && n < 1.01)
    }
    /// Per-pose send-side sanitising: a pose that is invalid, out of range,
    /// nonfinite, or has a non-normalisable quaternion becomes [`Pose::invalid`];
    /// a valid quaternion is normalised. The result always passes `wire_valid`,
    /// so one bad slot never makes the driver drop the whole frame.
    pub fn sanitized(&self) -> Self {
        let in_range = self
            .position
            .iter()
            .all(|x| x.is_finite() && x.abs() <= MAX_POSITION_M)
            && bounded_vector(&self.linear_velocity, MAX_LINEAR_SPEED_MPS)
            && bounded_vector(&self.angular_velocity, MAX_ANGULAR_SPEED_RADPS);
        if !self.valid || !in_range {
            return Self::invalid();
        }
        let n: f64 = self.rotation.iter().map(|x| f64::from(*x).powi(2)).sum();
        if !n.is_finite() || n < 1e-12 {
            return Self::invalid();
        }
        let scale = n.sqrt().recip();
        let out = Self {
            rotation: self.rotation.map(|x| (f64::from(x) * scale) as f32),
            ..*self
        };
        if out.wire_valid() {
            out
        } else {
            Self::invalid()
        }
    }
}
fn bounded_vector(v: &[f32; 3], max: f32) -> bool {
    v.iter().all(|x| x.is_finite()) && v[0] * v[0] + v[1] * v[1] + v[2] * v[2] <= max * max
}

/// App-to-driver snapshot, always sent as v2.
#[derive(Clone, Debug)]
pub struct Trackers {
    pub session: u64,
    pub sequence: u64,
    pub raw_from_standing: [f32; 12],
    /// How old the pose sample already was when sent; must be finite and in
    /// `0..=MAX_POSE_AGE_SECS`, otherwise every pose is sent invalid.
    pub pose_age_secs: f32,
    /// Sets header flag bit 0. Opt-in: SteamVR's angular-velocity frame is unverified.
    pub apply_angular_velocity: bool,
    pub poses: [Pose; TRACKER_COUNT],
}
/// Helper-to-app snapshot (HMD, left hand, right hand).
#[derive(Clone, Debug)]
pub struct Anchors {
    pub session: u64,
    pub sequence: u64,
    pub raw_from_standing: [f32; 12],
    pub poses: [Pose; ANCHOR_COUNT],
}

fn put32(out: &mut Vec<u8>, value: u32) {
    out.extend(value.to_le_bytes());
}
fn put64(out: &mut Vec<u8>, value: u64) {
    out.extend(value.to_le_bytes());
}
fn putf(out: &mut Vec<u8>, value: f32) {
    put32(out, value.to_bits());
}
fn number(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}
fn number64(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}
fn fnumber(bytes: &[u8], at: usize) -> f32 {
    f32::from_bits(number(bytes, at))
}
fn put_header(out: &mut Vec<u8>, magic: &[u8; 4], session: u64, sequence: u64, flags: u32) {
    out.extend(magic);
    put32(out, PROTOCOL_VERSION);
    put64(out, session);
    put64(out, sequence);
    put32(out, flags);
}
fn put_pose(out: &mut Vec<u8>, pose: &Pose) {
    put32(out, pose.valid as u32);
    for x in pose
        .position
        .iter()
        .chain(&pose.rotation)
        .chain(&pose.linear_velocity)
        .chain(&pose.angular_velocity)
    {
        putf(out, *x);
    }
}
fn read_vector<const N: usize>(bytes: &[u8], at: usize) -> [f32; N] {
    std::array::from_fn(|i| fnumber(bytes, at + i * 4))
}

impl Trackers {
    /// Serializes a complete atomic v2 snapshot, sanitising each pose so the
    /// driver never rejects the frame because of one bad slot. A non-rigid
    /// transform or out-of-range pose age sends an identity transform with
    /// every pose invalid. Validate calibration and freshness before marking any pose valid.
    pub fn encode(&self) -> Vec<u8> {
        let transform_ok = valid_transform(&self.raw_from_standing);
        let age_ok = self.pose_age_secs.is_finite()
            && (0.0..=MAX_POSE_AGE_SECS).contains(&self.pose_age_secs);
        let usable = transform_ok && age_ok;
        let flags = if self.apply_angular_velocity {
            FLAG_APPLY_ANGULAR_VELOCITY
        } else {
            0
        };
        let mut out = Vec::with_capacity(TRACKER_BYTES);
        put_header(&mut out, b"CFBP", self.session, self.sequence, flags);
        let transform = if usable {
            self.raw_from_standing
        } else {
            IDENTITY_TRANSFORM
        };
        for x in transform {
            putf(&mut out, x);
        }
        putf(&mut out, if usable { self.pose_age_secs } else { 0.0 });
        for pose in &self.poses {
            let pose = if usable {
                pose.sanitized()
            } else {
                Pose::invalid()
            };
            put_pose(&mut out, &pose);
        }
        debug_assert_eq!(out.len(), TRACKER_BYTES);
        out
    }
    pub fn send(&self, socket: &UdpSocket) -> io::Result<usize> {
        socket.send_to(
            &self.encode(),
            SocketAddrV4::new(Ipv4Addr::LOCALHOST, DRIVER_PORT),
        )
    }
}
impl Anchors {
    /// Accepts v2 (244 bytes) and legacy v1 (168 bytes, zero velocity) frames.
    /// Any malformed field rejects the whole frame.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 8 || &bytes[..4] != b"CFBA" {
            return None;
        }
        let version = number(bytes, 4);
        let (transform_at, poses_at, pose_bytes) = match (version, bytes.len()) {
            (2, ANCHOR_BYTES) => {
                if number(bytes, 24) != 0 {
                    return None; // anchor frames define no flags
                }
                (28, HEADER_BYTES, POSE_BYTES)
            }
            (1, ANCHOR_BYTES_V1) => (24, 72, 32),
            _ => return None,
        };
        let session = number64(bytes, 8);
        let sequence = number64(bytes, 16);
        let transform: [f32; 12] = read_vector(bytes, transform_at);
        if session == 0 || !valid_transform(&transform) {
            return None;
        }
        let mut poses = [Pose::invalid(); ANCHOR_COUNT];
        for (i, p) in poses.iter_mut().enumerate() {
            let at = poses_at + i * pose_bytes;
            let flags = number(bytes, at);
            if flags > 1 {
                return None;
            }
            p.valid = flags == 1;
            p.position = read_vector(bytes, at + 4);
            p.rotation = read_vector(bytes, at + 16);
            if version == 2 {
                p.linear_velocity = read_vector(bytes, at + 32);
                p.angular_velocity = read_vector(bytes, at + 44);
            }
            if !p.wire_valid() {
                return None;
            }
        }
        Some(Self {
            session,
            sequence,
            raw_from_standing: transform,
            poses,
        })
    }
    /// v2 encoding as written by the native helper (used for tests and tools).
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(ANCHOR_BYTES);
        put_header(&mut out, b"CFBA", self.session, self.sequence, 0);
        for x in self.raw_from_standing {
            putf(&mut out, x);
        }
        for pose in &self.poses {
            put_pose(&mut out, pose);
        }
        out
    }
    pub fn bind() -> io::Result<UdpSocket> {
        UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, ANCHOR_PORT))
    }
}

/// Verifies that a row-major 3x4 transform is finite, orthonormal, and right-handed.
pub fn valid_transform(m: &[f32; 12]) -> bool {
    if m.iter().any(|v| !v.is_finite() || v.abs() > 1000.0) {
        return false;
    }
    for i in 0..3 {
        for j in 0..3 {
            let dot: f32 = (0..3).map(|k| m[4 * i + k] * m[4 * j + k]).sum();
            if (dot - if i == j { 1.0 } else { 0.0 }).abs() > 0.01 {
                return false;
            }
        }
    }
    let d = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8])
        + m[2] * (m[4] * m[9] - m[5] * m[8]);
    (d - 1.).abs() < 0.01
}

/// The helper sends at ~200 Hz, so 100 ms is about 20 send periods.
const LEASE: Duration = Duration::from_millis(100);
const DRAIN_LIMIT: usize = 64;
const WSAEMSGSIZE: i32 = 10040;
const WSAECONNRESET: i32 = 10054;

/// Pure receive state, driven by receiver-local monotonic instants. In particular,
/// no timestamps from the other process are trusted or compared.
struct AnchorState {
    latest: Option<(Anchors, Instant)>,
    // Replay watermark survives pose invalidation and socket draining.
    watermark: Option<(u64, u64, Instant)>,
    retired: [u64; 8],
    retired_count: usize,
    retired_next: usize,
    last_poll: Instant,
    flushing: bool,
}
impl AnchorState {
    fn new(now: Instant) -> Self {
        Self {
            latest: None,
            watermark: None,
            retired: [0; 8],
            retired_count: 0,
            retired_next: 0,
            last_poll: now,
            flushing: false,
        }
    }
    fn begin_poll(&mut self, now: Instant) {
        if now
            .checked_duration_since(self.last_poll)
            .is_none_or(|age| age >= LEASE)
        {
            self.overflow_or_error();
        }
        self.last_poll = now;
    }
    fn drained(&mut self) {
        self.flushing = false;
    }
    fn overflow_or_error(&mut self) {
        self.latest = None;
        self.flushing = true;
    }
    fn accept(&mut self, frame: Anchors, now: Instant) -> bool {
        if self.flushing {
            return false;
        }
        if let Some((session, sequence, time)) = self.watermark {
            let Some(age) = now.checked_duration_since(time) else {
                self.overflow_or_error();
                return false;
            };
            if frame.session == session && frame.sequence <= sequence {
                return false;
            }
            if frame.session != session {
                if age < LEASE || self.retired[..self.retired_count].contains(&frame.session) {
                    return false;
                }
                self.retired[self.retired_next] = session;
                self.retired_next = (self.retired_next + 1) % self.retired.len();
                self.retired_count = (self.retired_count + 1).min(self.retired.len());
            }
        }
        self.watermark = Some((frame.session, frame.sequence, now));
        self.latest = Some((frame, now));
        true
    }
    // A validated discarded packet can advance the current session's replay
    // watermark without extending the accepted frame's liveness lease.
    fn observe_discarded(&mut self, frame: &Anchors) {
        if !self.flushing {
            return;
        }
        if let Some((session, sequence, time)) = self.watermark {
            if frame.session == session && frame.sequence > sequence {
                self.watermark = Some((session, frame.sequence, time));
            }
        }
    }
    fn current(&self, now: Instant) -> Option<Anchors> {
        if self.flushing {
            return None;
        }
        self.latest.as_ref().and_then(|(frame, time)| {
            now.checked_duration_since(*time)
                .filter(|age| *age < LEASE)
                .map(|_| frame.clone())
        })
    }
    fn age(&self, now: Instant) -> Option<Duration> {
        if self.flushing {
            return None;
        }
        self.latest
            .as_ref()
            .and_then(|(_, time)| now.checked_duration_since(*time))
    }
}

/// Nonblocking loopback transport. A restarted sender takes over after the old session is stale.
/// Targets are sent from a separate ephemeral socket: on Windows, sending to a
/// port nobody listens on queues an ICMP port-unreachable that would otherwise
/// fail the next anchor `recv_from` with WSAECONNRESET.
pub struct Bridge {
    socket: UdpSocket,
    sender: UdpSocket,
    driver: SocketAddr,
    state: AnchorState,
}
impl Bridge {
    pub fn bind() -> io::Result<Self> {
        Self::bind_at(
            SocketAddrV4::new(Ipv4Addr::LOCALHOST, ANCHOR_PORT),
            SocketAddrV4::new(Ipv4Addr::LOCALHOST, DRIVER_PORT),
        )
    }
    fn bind_at(anchors: SocketAddrV4, driver: SocketAddrV4) -> io::Result<Self> {
        let socket = UdpSocket::bind(anchors)?;
        socket.set_nonblocking(true)?;
        let sender = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0))?;
        sender.set_nonblocking(true)?;
        Ok(Self {
            socket,
            sender,
            driver: driver.into(),
            state: AnchorState::new(Instant::now()),
        })
    }
    /// Drain incoming snapshots; only fresh, monotonic frames from localhost are returned.
    pub fn recv_anchors(&mut self) -> io::Result<Option<Anchors>> {
        self.state.begin_poll(Instant::now());
        let mut bytes = [0u8; 512];
        let mut exhausted = true;
        for _ in 0..DRAIN_LIMIT {
            match self.socket.recv_from(&mut bytes) {
                Ok((n, source)) => {
                    if !source.ip().is_loopback() {
                        continue;
                    }
                    if let Some(frame) = Anchors::decode(&bytes[..n]) {
                        if self.state.flushing {
                            self.state.observe_discarded(&frame);
                        } else {
                            self.state.accept(frame, Instant::now());
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    self.state.drained();
                    exhausted = false;
                    break;
                }
                Err(e) if e.raw_os_error() == Some(WSAEMSGSIZE) => continue,
                // A stale ICMP port-unreachable report, not a fault of this socket.
                Err(e)
                    if e.raw_os_error() == Some(WSAECONNRESET)
                        || e.kind() == io::ErrorKind::ConnectionReset =>
                {
                    continue
                }
                Err(e) => {
                    self.state.overflow_or_error();
                    return Err(e);
                }
            }
        }
        // A full budget may leave an arbitrarily old queue. Invalidate, then
        // discard until empty before accepting fresh data on a later poll.
        if exhausted {
            self.state.overflow_or_error();
        }
        Ok(self.state.current(Instant::now()))
    }
    pub fn send_targets(&self, frame: &Trackers) -> io::Result<usize> {
        self.sender.send_to(&frame.encode(), self.driver)
    }
    pub fn anchors_age(&self) -> Option<Duration> {
        self.state.age(Instant::now())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;

    fn hex(text: &str) -> Vec<u8> {
        text.trim()
            .as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect()
    }
    fn identity() -> [f32; 12] {
        [1., 0., 0., 2., 0., 1., 0., 3., 0., 0., 1., 4.]
    }
    fn anchors() -> Anchors {
        Anchors {
            session: 11,
            sequence: 12,
            raw_from_standing: identity(),
            poses: [Pose {
                valid: true,
                position: [1., 2., 3.],
                rotation: [1., 0., 0., 0.],
                linear_velocity: [0.1, 0.2, 0.3],
                angular_velocity: [0., 1., 0.],
            }; 3],
        }
    }
    fn anchor_bytes() -> Vec<u8> {
        anchors().encode()
    }
    fn fixture_poses() -> [Pose; TRACKER_COUNT] {
        let mut poses = [Pose::invalid(); TRACKER_COUNT];
        let q = std::f32::consts::FRAC_1_SQRT_2;
        poses[0] = Pose {
            valid: true,
            position: [1., 2., 3.],
            rotation: [q, q, 0., 0.],
            linear_velocity: [0.5, -1., 2.],
            angular_velocity: [0., 0., 3.],
        };
        poses[2] = Pose {
            valid: true,
            position: [-0.25, 1.2, -0.5],
            rotation: [1., 0., 0., 0.],
            linear_velocity: [0., 0., -0.25],
            angular_velocity: [1., 0., 0.],
        };
        poses
    }
    fn fixture_frame() -> Trackers {
        Trackers {
            session: 0x1122334455667788,
            sequence: 123,
            raw_from_standing: [0., 0., 1., 2., 0., 1., 0., 3., -1., 0., 0., 4.],
            pose_age_secs: 0.015625,
            apply_angular_velocity: true,
            poses: fixture_poses(),
        }
    }
    // Legacy v1 layout, kept only to pin the v1 fixture the driver still accepts.
    fn encode_v1(frame: &Trackers) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend(b"CFBP");
        put32(&mut out, 1);
        put64(&mut out, frame.session);
        put64(&mut out, frame.sequence);
        for x in frame.raw_from_standing {
            putf(&mut out, x);
        }
        for pose in frame.poses {
            put32(&mut out, pose.valid as u32);
            for x in pose.position.iter().chain(&pose.rotation) {
                putf(&mut out, *x);
            }
        }
        out
    }

    #[test]
    fn fixed_layout() {
        assert_eq!(TRACKER_BYTES, 528);
        assert_eq!(ANCHOR_BYTES, 244);
        let bytes = fixture_frame().encode();
        assert_eq!(bytes.len(), TRACKER_BYTES);
        assert_eq!(&bytes[..4], b"CFBP");
        assert_eq!(number(&bytes, 4), 2);
        assert_eq!(number64(&bytes, 8), 0x1122334455667788);
        assert_eq!(number64(&bytes, 16), 123);
        assert_eq!(number(&bytes, 24), FLAG_APPLY_ANGULAR_VELOCITY);
        assert_eq!(fnumber(&bytes, 28 + 3 * 4), 2.0);
        assert_eq!(fnumber(&bytes, 76), 0.015625);
        // pose 0: valid, position, rotation, linear, angular velocity
        assert_eq!(number(&bytes, 80), 1);
        assert_eq!(fnumber(&bytes, 80 + 32), 0.5);
        assert_eq!(fnumber(&bytes, 80 + 44 + 8), 3.0);
        assert_eq!(anchor_bytes().len(), ANCHOR_BYTES);
    }
    #[test]
    fn rust_targets_match_cross_language_receiver_fixture() {
        let fixture = hex(include_str!("../../../driver/tests/fixtures/target-v2.hex"));
        assert_eq!(fixture_frame().encode(), fixture);
    }
    #[test]
    fn legacy_v1_fixture_layout_is_pinned() {
        let mut frame = fixture_frame();
        frame.poses = fixture_poses().map(|p| Pose {
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
            ..p
        });
        let fixture = hex(include_str!("../../../driver/tests/fixtures/target-v1.hex"));
        assert_eq!(fixture.len(), TRACKER_BYTES_V1);
        assert_eq!(encode_v1(&frame), fixture);
    }
    #[test]
    fn anchor_fixture_matches_helper_layout() {
        let fixture = hex(include_str!("../../../driver/tests/fixtures/anchor-v2.hex"));
        let a = Anchors::decode(&fixture).unwrap();
        assert_eq!((a.session, a.sequence), (11, 12));
        assert!(a.poses[0].valid && !a.poses[1].valid && a.poses[2].valid);
        assert_eq!(a.poses[0].position, [0.1, 1.6, -0.2]);
        assert_eq!(a.poses[0].linear_velocity, [0.5, 0., 0.]);
        assert_eq!(a.poses[0].angular_velocity, [0., 1., 0.]);
        assert_eq!(a.poses[2].rotation, [0.5; 4]);
        assert_eq!(a.poses[2].linear_velocity, [0., -0.25, 0.]);
        assert_eq!(a.poses[2].angular_velocity, [0., 0., 2.]);
        assert_eq!(a.encode(), fixture);
    }

    /// Runs the case table shared with driver/tests/wire_tests.cpp (native
    /// `valid_pose`), so neither side's rules can change without the other.
    #[test]
    fn wire_validity_mirrors_native_valid_pose() {
        let table = include_str!("../../../driver/tests/fixtures/pose-validity.txt");
        let mut checked = 0;
        for line in table.lines().map(str::trim) {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let v: Vec<f32> = line
                .split_whitespace()
                .map(|x| x.parse().unwrap())
                .collect();
            assert_eq!(v.len(), 16, "{line}");
            if v[0] > 1.0 {
                continue; // a non-boolean flag is not representable here; decode rejects it
            }
            let pose = Pose {
                valid: v[0] == 1.0,
                position: [v[1], v[2], v[3]],
                rotation: [v[4], v[5], v[6], v[7]],
                linear_velocity: [v[8], v[9], v[10]],
                angular_velocity: [v[11], v[12], v[13]],
            };
            assert_eq!(pose.wire_valid(), v[14] == 1.0, "{line}");
            let clean = pose.sanitized();
            assert!(clean.wire_valid(), "sanitized {line}");
            assert_eq!(clean.valid, v[15] == 1.0, "{line}");
            if !clean.valid {
                assert_eq!(clean, Pose::invalid(), "{line}");
            } else {
                let n: f32 = clean.rotation.iter().map(|x| x * x).sum();
                assert!((n - 1.0).abs() < 1e-6, "{line}");
            }
            checked += 1;
        }
        assert!(checked >= 15);
    }
    #[test]
    fn one_bad_slot_is_invalidated_not_the_frame() {
        let mut frame = fixture_frame();
        frame.poses[1] = Pose {
            valid: true,
            position: [f32::NAN, 0., 0.],
            ..Pose::invalid()
        };
        frame.poses[3] = Pose {
            valid: true,
            linear_velocity: [0., 60., 0.],
            ..Pose::invalid()
        };
        frame.poses[4] = Pose {
            valid: true,
            rotation: [0.; 4],
            ..Pose::invalid()
        };
        frame.poses[5] = Pose {
            valid: true,
            rotation: [0., 0., 3., 4.],
            ..Pose::invalid()
        };
        let bytes = frame.encode();
        let record = |i: usize| 80 + i * POSE_BYTES;
        for i in [1, 3, 4] {
            assert_eq!(number(&bytes, record(i)), 0, "slot {i}");
            assert_eq!(fnumber(&bytes, record(i) + 4), 0.0);
            assert_eq!(fnumber(&bytes, record(i) + 16), 1.0);
        }
        // Valid but unnormalised quaternions are normalised, not dropped.
        assert_eq!(number(&bytes, record(5)), 1);
        assert_eq!(fnumber(&bytes, record(5) + 24), 0.6);
        assert_eq!(fnumber(&bytes, record(5) + 28), 0.8);
        // The untouched valid slot keeps its data.
        assert_eq!(number(&bytes, record(0)), 1);
        assert_eq!(fnumber(&bytes, record(0) + 4), 1.0);
    }
    #[test]
    fn unusable_transform_or_age_sends_all_invalid() {
        for mutate in [
            |f: &mut Trackers| f.raw_from_standing[0] = 2.,
            |f: &mut Trackers| f.pose_age_secs = 0.3,
            |f: &mut Trackers| f.pose_age_secs = -0.001,
            |f: &mut Trackers| f.pose_age_secs = f32::NAN,
        ] {
            let mut frame = fixture_frame();
            mutate(&mut frame);
            let bytes = frame.encode();
            assert_eq!(bytes.len(), TRACKER_BYTES);
            assert!(valid_transform(&read_vector(&bytes, 28)));
            assert_eq!(fnumber(&bytes, 76), 0.0);
            assert!((0..TRACKER_COUNT).all(|i| number(&bytes, 80 + i * POSE_BYTES) == 0));
        }
        let mut frame = fixture_frame();
        frame.pose_age_secs = MAX_POSE_AGE_SECS;
        assert_eq!(number(&frame.encode(), 80), 1);
        frame.apply_angular_velocity = false;
        assert_eq!(number(&frame.encode(), 24), 0);
    }
    #[test]
    fn cached_sequence_cannot_refresh_age_and_session_takeover_waits_for_expiry() {
        let t = Instant::now();
        let ms = |n| t + Duration::from_millis(n);
        let first = Anchors::decode(&anchor_bytes()).unwrap();
        let mut state = AnchorState::new(t);
        assert!(state.accept(first.clone(), t));
        assert!(!state.accept(first.clone(), ms(80)));
        assert_eq!(state.age(ms(99)), Some(Duration::from_millis(99)));
        assert!(state.current(ms(99)).is_some());
        assert!(state.current(ms(100)).is_none());
        let mut next = first.clone();
        next.session += 1;
        next.sequence = 1;
        assert!(!state.accept(next.clone(), ms(80)));
        state.begin_poll(ms(40));
        state.begin_poll(ms(80));
        state.begin_poll(ms(100));
        assert!(state.accept(next.clone(), ms(100)));
        assert!(state.current(ms(100)).is_some());
        assert!(!state.accept(next.clone(), ms(104)));
        next.sequence += 1;
        assert!(state.accept(next.clone(), ms(108)));
        state.begin_poll(ms(140));
        state.begin_poll(ms(180));
        state.begin_poll(ms(210));
        let mut retired = first;
        retired.sequence = 999;
        assert!(!state.accept(retired, ms(210)));
        next.session += 1;
        next.sequence = 1;
        assert!(state.accept(next, ms(210)));
    }
    #[test]
    fn pause_or_budget_exhaustion_discards_backlog_until_empty() {
        let t = Instant::now();
        let ms = |n| t + Duration::from_millis(n);
        let mut state = AnchorState::new(t);
        let mut frame = Anchors::decode(&anchor_bytes()).unwrap();
        let old = frame.clone();
        assert!(state.accept(frame.clone(), t));
        state.begin_poll(ms(100));
        assert!(state.flushing);
        assert!(state.current(ms(100)).is_none());
        frame.sequence += 1;
        assert!(!state.accept(frame.clone(), ms(101)));
        state.observe_discarded(&frame);
        state.drained();
        assert!(state.current(ms(101)).is_none());
        assert!(!state.accept(old, ms(101))); // replay watermark survives invalidation
        assert!(!state.accept(frame.clone(), ms(101))); // seen while draining
        frame.sequence += 1;
        assert!(state.accept(frame.clone(), ms(102)));
        state.overflow_or_error();
        assert!(state.current(ms(102)).is_none());
        frame.sequence += 1;
        assert!(!state.accept(frame.clone(), ms(103)));
        state.drained();
        assert!(state.accept(frame, ms(104)));
        assert!(state.current(ms(103)).is_none());
    }
    #[test]
    fn rigid_transforms_reject_reflection_scale_and_nan() {
        assert!(valid_transform(&identity()));
        let mut m = identity();
        m[0] = -1.;
        assert!(!valid_transform(&m));
        m = identity();
        m[5] = 2.;
        assert!(!valid_transform(&m));
        m = identity();
        m[0] = f32::NAN;
        assert!(!valid_transform(&m));
    }
    #[test]
    fn anchor_decoder_checks_version_flags_finiteness_velocity_and_quaternion() {
        let mut b = anchor_bytes();
        let a = Anchors::decode(&b).unwrap();
        assert_eq!(a.session, 11);
        assert_eq!(a.sequence, 12);
        assert_eq!(a.poses[0].position, [1., 2., 3.]);
        assert_eq!(a.poses[0].linear_velocity, [0.1, 0.2, 0.3]);
        b[4] = 3;
        assert!(Anchors::decode(&b).is_none());
        b = anchor_bytes();
        b[24] = 1; // anchors define no flags
        assert!(Anchors::decode(&b).is_none());
        let rotation_w = HEADER_BYTES + 16;
        b = anchor_bytes();
        b[rotation_w..rotation_w + 4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(Anchors::decode(&b).is_none());
        b = anchor_bytes();
        b[rotation_w..rotation_w + 4].copy_from_slice(&2f32.to_le_bytes());
        assert!(Anchors::decode(&b).is_none());
        let velocity = HEADER_BYTES + POSE_BYTES + 32;
        b = anchor_bytes();
        b[velocity..velocity + 4].copy_from_slice(&51f32.to_le_bytes());
        assert!(Anchors::decode(&b).is_none());
        let angular = HEADER_BYTES + 2 * POSE_BYTES + 44;
        b = anchor_bytes();
        b[angular..angular + 4].copy_from_slice(&f32::INFINITY.to_le_bytes());
        assert!(Anchors::decode(&b).is_none());
        assert!(Anchors::decode(&anchor_bytes()[..ANCHOR_BYTES - 1]).is_none());
    }
    #[test]
    fn legacy_v1_anchor_frames_decode_with_zero_velocity() {
        let mut b = Vec::new();
        b.extend(b"CFBA");
        put32(&mut b, 1);
        put64(&mut b, 11);
        put64(&mut b, 12);
        for x in identity() {
            putf(&mut b, x)
        }
        for _ in 0..3 {
            put32(&mut b, 1);
            for x in [1., 2., 3., 1., 0., 0., 0.] {
                putf(&mut b, x)
            }
        }
        assert_eq!(b.len(), ANCHOR_BYTES_V1);
        let a = Anchors::decode(&b).unwrap();
        assert!(a
            .poses
            .iter()
            .all(|p| p.valid && p.linear_velocity == [0.; 3] && p.angular_velocity == [0.; 3]));
        b[4] = 2; // v2 version with a v1 length
        assert!(Anchors::decode(&b).is_none());
    }

    fn unbound_port() -> SocketAddrV4 {
        let probe = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        match probe.local_addr().unwrap() {
            SocketAddr::V4(addr) => addr,
            SocketAddr::V6(_) => unreachable!(),
        }
    }
    fn receive_within(bridge: &mut Bridge, attempts: usize) -> Option<Anchors> {
        for _ in 0..attempts {
            // Every poll must succeed; an Err here was the WSAECONNRESET bug.
            if let Some(frame) = bridge.recv_anchors().expect("anchor receive failed") {
                return Some(frame);
            }
            sleep(Duration::from_millis(2));
        }
        None
    }

    /// Regression: with no driver listening, target sends must not stop anchor
    /// reception (Windows reports ICMP port-unreachable as WSAECONNRESET on the
    /// next recv_from of the socket that sent).
    #[test]
    fn anchors_still_arrive_while_driver_port_is_closed() {
        let dead = unbound_port();
        let mut bridge = Bridge::bind_at(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0), dead).unwrap();
        let anchor_addr = bridge.socket.local_addr().unwrap();
        let helper = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        for sequence in 1..=3 {
            let mut frame = fixture_frame();
            frame.sequence = sequence;
            bridge.send_targets(&frame).unwrap();
        }
        // Target sends use their own socket: no error is queued on the anchor socket.
        sleep(Duration::from_millis(20));
        let raw = bridge.socket.recv_from(&mut [0u8; 16]).unwrap_err();
        assert_eq!(raw.kind(), io::ErrorKind::WouldBlock, "{raw:?}");
        // Also provoke the ICMP report on the anchor socket itself, as the old
        // shared-socket design did, to exercise the non-fatal WSAECONNRESET path.
        bridge.socket.send_to(&[0u8; 4], dead).unwrap();
        sleep(Duration::from_millis(20));
        helper.send_to(&anchor_bytes(), anchor_addr).unwrap();
        let frame = receive_within(&mut bridge, 20).expect("anchor frame was not accepted");
        assert_eq!((frame.session, frame.sequence), (11, 12));
        // And again after another round of unanswered target sends.
        bridge.send_targets(&fixture_frame()).unwrap();
        bridge.socket.send_to(&[0u8; 4], dead).unwrap();
        sleep(Duration::from_millis(20));
        let mut next = anchors();
        next.sequence = 13;
        helper.send_to(&next.encode(), anchor_addr).unwrap();
        let mut seen = None;
        for _ in 0..20 {
            match bridge.recv_anchors().expect("anchor receive failed") {
                Some(frame) if frame.sequence == 13 => {
                    seen = Some(frame);
                    break;
                }
                _ => sleep(Duration::from_millis(2)),
            }
        }
        assert!(seen.is_some(), "second anchor frame was not accepted");
    }

    /// Documents the platform hazard the fix targets: on Windows the socket
    /// that sent to a closed loopback port gets WSAECONNRESET from recv_from.
    #[cfg(windows)]
    #[test]
    fn windows_reports_unreachable_port_as_connreset_on_sender() {
        let dead = unbound_port();
        let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        socket.send_to(&[0u8; 4], dead).unwrap();
        socket
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let error = socket.recv_from(&mut [0u8; 16]).unwrap_err();
        assert_eq!(error.raw_os_error(), Some(WSAECONNRESET));
    }
}
