//! Network transport and device identity; deliberately independent of the IK solver.
//!
//! A dedicated receive thread blocks on the tracker socket and stamps every
//! datagram from the shared monotonic clock immediately after `recv_from`
//! returns. The tracking worker drains those stamped datagrams whenever it wakes,
//! so its own scheduling or solve time never changes a datagram's arrival time.
use crate::model::TrackerView;
use glam::Quat;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, File, OpenOptions},
    io::{self, BufWriter, Write},
    net::{SocketAddr, UdpSocket},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, RecvTimeoutError, SyncSender, TryRecvError, TrySendError},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
use tracker_protocol::{Packet, SequenceObservation, SequenceTracker};

/// Requested kernel receive buffer; absorbs bursts while the receive thread is descheduled.
pub const RECEIVE_BUFFER_BYTES: usize = 4 * 1024 * 1024;
/// Stamped datagrams queued for the worker. Overflow drops are visible as sequence gaps.
const EVENT_CAPACITY: usize = 8192;
/// Receive-thread wake period without traffic; also bounds shutdown latency.
const RECEIVER_POLL: Duration = Duration::from_millis(50);
/// A receive thread that was not scheduled for this long may hold a stale kernel queue.
const RECEIVER_STALL_SECS: f64 = 0.25;
/// Upper bound of datagrams handled per worker wake. Remaining ones keep their
/// true arrival stamps and are handled on the immediately following wake.
const MAX_EVENTS_PER_POLL: usize = 4096;
/// Device liveness required for a firmware-held orientation (matches the core lease).
const HOLD_TRANSPORT_LEASE_SECS: f64 = 1.5;
/// Firmware bundles for 10 ms (PACKET_BUNDLING_BUFFER_SIZE_MICROS). A loss within
/// about two bundle periods after the last rotation may have been the final
/// rotation of a movement; a later isolated loss is most likely an idle heartbeat.
pub const HOLD_GAP_AFTER_ROTATION_SECS: f64 = 0.025;
/// A still tracker answers our 2 Hz heartbeat; losing more datagrams than this rate
/// explains implies the tracker was streaming (moving) during the gap.
pub const IDLE_DATAGRAMS_PER_SEC: f64 = 2.0;
/// Datagram types outside the documented tracker set are counted in one bucket.
pub const OTHER_DATAGRAM_TYPE: u32 = u32::MAX;
const MAX_SESSION_HISTORY: usize = 256;

#[derive(Clone, Debug)]
pub struct Measurement {
    pub id: String,
    pub orientation: [f32; 4],
    pub acceleration: Option<[f32; 3]>,
    pub at: f64,
    pub provenance: tracking_core::SensorProvenance,
}

/// One datagram as received by the receive thread.
#[derive(Clone, Debug)]
pub struct Received {
    pub bytes: Vec<u8>,
    pub source: SocketAddr,
    /// Monotonic seconds since the worker origin, taken right after `recv_from`.
    pub at: f64,
    /// Read from a queue that built up while the receive thread was not running
    /// (or after a socket error); `at` is then only an upper bound.
    pub backlog: bool,
}

#[derive(Clone, Debug)]
pub enum ReceiverEvent {
    Datagram(Received),
    /// Following datagrams are backlog until `CaughtUp`.
    Disrupted {
        error: Option<String>,
    },
    /// The kernel queue was observed empty at `at`.
    CaughtUp {
        at: f64,
    },
}

/// Evidence about a finished session of one tracker identity, kept so a later
/// handshake can be classified as a same-boot reconnection or a reboot.
#[derive(Clone, Debug, PartialEq)]
struct PriorSession {
    last_sequence: u64,
    firmware: String,
}

#[derive(Default)]
struct Device {
    name: String,
    firmware: String,
    battery: Option<f32>,
    sequence: SequenceTracker,
    last_sequence: Option<u64>,
    last_seen: f64,
    /// This IMU boot has delivered rotations in this or a verified-continuous earlier session.
    lineage: bool,
    /// Previous session awaiting comparison with this session's first data sequence.
    pending_prior: Option<PriorSession>,
    accepted_packets: u64,
    sequence_missing: u64,
    mac_known: bool,
    sensor_status: BTreeMap<u8, u8>,
    drain_gap: bool,
    drain_datagrams: u64,
}

#[derive(Default)]
struct Sensor {
    view: TrackerView,
    last_rotation: f64,
    rate_start: Option<f64>,
    rate_count: u64,
    last_accel: Option<f64>,
    device_source: Option<SocketAddr>,
    sensor_id: u8,
    hold_valid: bool,
    continuity_since: Option<f64>,
    measurement_valid: bool,
    /// A backlog drain is in progress; validity is decided when it completes.
    drain_pending: bool,
    drain_rotation: Option<[f32; 4]>,
}

impl Sensor {
    fn mark_measured(&mut self, now: f64) {
        self.measurement_valid = true;
        self.observe_rotation_rate(now);
    }
    fn observe_rotation_rate(&mut self, now: f64) {
        if self.rate_start.is_some() && (now - self.last_rotation > 1.0 || now < self.last_rotation)
        {
            self.rate_start = None;
            self.rate_count = 0;
            self.view.hz = 0.0;
        }
        let start = *self.rate_start.get_or_insert(now);
        self.rate_count += 1;
        let elapsed = now - start;
        if elapsed >= 1.0 {
            self.view.hz = (self.rate_count.saturating_sub(1) as f64 / elapsed) as f32;
            self.rate_start = Some(now);
            self.rate_count = 1;
        }
        self.last_rotation = now;
    }
    fn revoke_hold(&mut self, id: &str, revocations: &mut BTreeMap<String, bool>) {
        self.hold_valid = false;
        self.continuity_since = None;
        revocations.entry(id.to_owned()).or_insert(false);
    }
}

fn revoke_source(
    sensors: &mut BTreeMap<String, Sensor>,
    revocations: &mut BTreeMap<String, bool>,
    source: SocketAddr,
) {
    for (id, sensor) in sensors
        .iter_mut()
        .filter(|(_, sensor)| sensor.device_source == Some(source))
    {
        sensor.revoke_hold(id, revocations);
    }
}
fn revoke_sensor(
    sensors: &mut BTreeMap<String, Sensor>,
    revocations: &mut BTreeMap<String, bool>,
    source: SocketAddr,
    sensor_id: u8,
) {
    for (id, sensor) in sensors
        .iter_mut()
        .filter(|(_, sensor)| sensor.device_source == Some(source) && sensor.sensor_id == sensor_id)
    {
        sensor.measurement_valid = false;
        sensor.hold_valid = false;
        sensor.continuity_since = None;
        revocations.insert(id.clone(), true);
    }
}
// A handshake starts a new tracker session even when MAC is unchanged.
// Previously received quaternions must not survive into the new session.
fn revoke_session(
    sensors: &BTreeMap<String, Sensor>,
    revocations: &mut BTreeMap<String, bool>,
    source: SocketAddr,
    identity: &str,
) {
    for (id, sensor) in sensors {
        if sensor.device_source == Some(source) || id.starts_with(&(identity.to_owned() + "/")) {
            revocations.insert(id.clone(), true);
        }
    }
}
/// Stock firmware (OPTIMIZE_UPDATES in debug.h, all 0.x releases and DIY builds
/// from the same source) suppresses unchanged rotations, so a healthy, still
/// sensor legitimately stops sending quaternions. Any firmware string from a
/// handshaken device qualifies; the sensor's own reported status must be OK.
fn held_eligible(sensor: &Sensor, device: &Device, now: f64) -> bool {
    sensor.measurement_valid
        && !sensor.drain_pending
        && sensor.hold_valid
        && sensor.continuity_since.is_some()
        && device.mac_known
        && !device.name.is_empty()
        && !device.firmware.trim().is_empty()
        && device.sensor_status.get(&sensor.sensor_id) == Some(&1)
        && now >= device.last_seen
        && now - device.last_seen <= HOLD_TRANSPORT_LEASE_SECS
        && now >= sensor.last_rotation
}

/// Whether a sequence gap may have hidden a rotation that changes the held
/// orientation. Heuristic (experimental): a loss soon after the last rotation
/// may be the movement's final sample; more losses than an idle tracker sends
/// imply it was streaming. Otherwise the lost datagram was idle traffic.
pub fn gap_may_hide_rotation(
    previous_datagram_at: f64,
    last_rotation_at: f64,
    missing: u64,
    arrival_span: f64,
) -> bool {
    if missing == 0 {
        return false;
    }
    if !(previous_datagram_at.is_finite()
        && last_rotation_at.is_finite()
        && arrival_span.is_finite())
    {
        return true;
    }
    previous_datagram_at - last_rotation_at <= HOLD_GAP_AFTER_ROTATION_SECS
        || missing as f64 > 1.0 + arrival_span.max(0.0) * IDLE_DATAGRAMS_PER_SEC
}

/// Firmware 0.7.2 keeps `m_PacketNumber` in the global `networkConnection` for the
/// whole boot: neither a server timeout, a Wi-Fi reconnect (`Connection::reset`)
/// nor a new handshake (sent with sequence override 0) resets it. A first data
/// sequence beyond the previous session's last one therefore proves the same
/// boot (the IMU kept running and kept its yaw origin); a restart from a low
/// number means the tracker rebooted.
fn continues_same_boot(prior: &PriorSession, firmware: &str, first_sequence: u64) -> bool {
    prior.firmware == firmware && first_sequence > prior.last_sequence
}

fn remember_session(
    history: &mut BTreeMap<String, PriorSession>,
    reconnected: &mut Vec<String>,
    device: &mut Device,
) {
    if device.name.is_empty() {
        return;
    }
    let record = if let Some(prior) = device.pending_prior.take() {
        // This session never sent data; the earlier evidence still applies.
        Some(prior)
    } else if device.lineage {
        match device.last_sequence {
            Some(last_sequence) => Some(PriorSession {
                last_sequence,
                firmware: device.firmware.clone(),
            }),
            None => {
                reconnected.push(device.name.clone());
                None
            }
        }
    } else {
        None
    };
    device.lineage = false;
    if let Some(record) = record {
        if history.len() >= MAX_SESSION_HISTORY && !history.contains_key(&device.name) {
            // Cannot keep the evidence: conservatively treat as a reboot.
            reconnected.push(device.name.clone());
            return;
        }
        history.insert(device.name.clone(), record);
    }
}

fn verify_continuation(device: &mut Device, sequence: u64, reconnected: &mut Vec<String>) {
    if let Some(prior) = device.pending_prior.take() {
        if continues_same_boot(&prior, &device.firmware, sequence) {
            device.lineage = true;
        } else {
            reconnected.push(device.name.clone());
        }
    }
}

// A handshake identifies a tracker; packet source addresses alone are not identities.
fn handshake_identity(info: &tracker_protocol::Handshake, source: SocketAddr) -> Option<String> {
    if info.board_type.is_none() || info.firmware.trim().is_empty() {
        return None;
    }
    Some(
        info.mac
            .map(|m| {
                m.iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<Vec<_>>()
                    .join(":")
            })
            .unwrap_or_else(|| source.ip().to_string()),
    )
}

struct Session<'a> {
    devices: &'a mut BTreeMap<SocketAddr, Device>,
    sensors: &'a mut BTreeMap<String, Sensor>,
    history: &'a mut BTreeMap<String, PriorSession>,
    reconnected: &'a mut Vec<String>,
}

/// Returns false when the endpoint table is full.
fn establish_session(
    session: Session<'_>,
    source: SocketAddr,
    identity: String,
    firmware: String,
    mac_known: bool,
) -> bool {
    let Session {
        devices,
        sensors,
        history,
        reconnected,
    } = session;
    let displaced: Vec<_> = devices
        .iter()
        .filter_map(|(address, device)| {
            (*address != source && device.name == identity).then_some(*address)
        })
        .collect();
    if devices.len() >= 64 && !devices.contains_key(&source) && displaced.is_empty() {
        return false;
    }
    for address in displaced {
        if let Some(mut device) = devices.remove(&address) {
            remember_session(history, reconnected, &mut device);
        }
    }
    if let Some(previous) = devices.get_mut(&source) {
        remember_session(history, reconnected, previous);
        if !previous.name.is_empty() && previous.name != identity {
            for sensor in sensors.values_mut() {
                if sensor.device_source == Some(source) {
                    sensor.device_source = None;
                    sensor.measurement_valid = false;
                    sensor.hold_valid = false;
                    sensor.continuity_since = None;
                }
            }
        }
    }
    for sensor in sensors.values_mut() {
        if sensor.view.id.starts_with(&(identity.clone() + "/")) {
            sensor.device_source = Some(source);
            sensor.measurement_valid = false;
            sensor.hold_valid = false;
            sensor.continuity_since = None;
            sensor.drain_pending = false;
            sensor.drain_rotation = None;
        }
    }
    let device = devices.entry(source).or_default();
    device.lineage = false;
    device.pending_prior = history.remove(&identity);
    device.name = identity;
    device.firmware = firmware;
    device.mac_known = mac_known;
    device.sensor_status.clear();
    device.last_sequence = None;
    device.drain_gap = false;
    device.drain_datagrams = 0;
    // Handshake sequence zero is unrelated to the running firmware counter.
    device.sequence.reset();
    true
}

fn datagram_type_bucket(packet_type: u32) -> u32 {
    if packet_type <= 32 || matches!(packet_type, 100 | 101 | 200) {
        packet_type
    } else {
        OTHER_DATAGRAM_TYPE
    }
}

// Opt-in, local-only raw capture for diagnosing firmware/protocol behavior.
// Source addresses and packet bytes are sensitive; never send or display them.
struct WireCapture {
    file: BufWriter<File>,
    packets: u64,
    bytes: u64,
}
impl WireCapture {
    const MAX_PACKETS: u64 = 20_000;
    const MAX_BYTES: u64 = 20 * 1024 * 1024;
    fn open(path: &std::ffi::OsStr) -> io::Result<Self> {
        let path = std::path::Path::new(path);
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
        let file = OpenOptions::new().write(true).create_new(true).open(path)?;
        Ok(Self {
            file: BufWriter::new(file),
            packets: 0,
            bytes: 0,
        })
    }
    fn capture(&mut self, now: f64, source: SocketAddr, bytes: &[u8]) -> io::Result<()> {
        if self.packets >= Self::MAX_PACKETS || self.bytes >= Self::MAX_BYTES {
            return Ok(());
        }
        use std::fmt::Write as _;
        let mut hex = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            let _ = write!(&mut hex, "{byte:02x}");
        }
        let mut record = serde_json::to_vec(
            &serde_json::json!({"at":now,"source":source.to_string(),"bytes_hex":hex}),
        )
        .map_err(io::Error::other)?;
        record.push(b'\n');
        if self.bytes + record.len() as u64 > Self::MAX_BYTES {
            self.bytes = Self::MAX_BYTES;
            return Ok(());
        }
        self.file.write_all(&record)?;
        self.packets += 1;
        self.bytes += record.len() as u64;
        Ok(())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()
    }
}

fn is_timeout(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}
/// WSAECONNRESET (10054) on UDP reports an ICMP port-unreachable caused by an
/// earlier send (e.g. a heartbeat to a rebooting tracker). It is not a fault of
/// this socket or of any other tracker and must never invalidate the transport.
fn is_connection_reset(error: &io::Error) -> bool {
    error.kind() == io::ErrorKind::ConnectionReset || error.raw_os_error() == Some(10054)
}

struct ReceiveLoop {
    socket: UdpSocket,
    origin: Instant,
    events: SyncSender<ReceiverEvent>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
}

impl ReceiveLoop {
    /// Returns false when the worker side has gone away.
    fn begin_catch_up(&self, error: Option<String>) -> bool {
        if self
            .events
            .send(ReceiverEvent::Disrupted { error })
            .is_err()
        {
            return false;
        }
        // Empty the queue without blocking; every datagram until WouldBlock is backlog.
        let _ = self.socket.set_nonblocking(true);
        true
    }
    fn finish_catch_up(&self, at: f64) -> bool {
        let _ = self.socket.set_nonblocking(false);
        let _ = self.socket.set_read_timeout(Some(RECEIVER_POLL));
        self.events.send(ReceiverEvent::CaughtUp { at }).is_ok()
    }
    fn run(self) {
        raise_current_thread_priority();
        let _ = self.socket.set_nonblocking(false);
        let _ = self.socket.set_read_timeout(Some(RECEIVER_POLL));
        let mut buffer = vec![0u8; tracker_protocol::MAX_DATAGRAM_SIZE];
        let mut last_return = Instant::now();
        let mut catching_up = false;
        while !self.stop.load(Ordering::Acquire) {
            let result = self.socket.recv_from(&mut buffer);
            let returned = Instant::now();
            let at = returned.duration_since(self.origin).as_secs_f64();
            let unscheduled =
                returned.duration_since(last_return).as_secs_f64() - RECEIVER_POLL.as_secs_f64();
            last_return = returned;
            if unscheduled > RECEIVER_STALL_SECS && !catching_up {
                if !self.begin_catch_up(None) {
                    return;
                }
                catching_up = true;
            }
            match result {
                Ok((size, source)) => {
                    let received = Received {
                        bytes: buffer[..size].to_vec(),
                        source,
                        at,
                        backlog: catching_up,
                    };
                    match self.events.try_send(ReceiverEvent::Datagram(received)) {
                        Ok(()) => {}
                        // Never block stamping on a slow worker; the loss shows up
                        // as a per-device sequence gap.
                        Err(TrySendError::Full(_)) => {
                            self.dropped.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(TrySendError::Disconnected(_)) => return,
                    }
                }
                Err(error) if is_timeout(&error) => {
                    if catching_up {
                        if !self.finish_catch_up(at) {
                            return;
                        }
                        catching_up = false;
                    }
                }
                Err(error) if is_connection_reset(&error) => {}
                Err(error) => {
                    if !catching_up {
                        if !self
                            .begin_catch_up(Some(format!("Tracker socket receive failed: {error}")))
                        {
                            return;
                        }
                        catching_up = true;
                    }
                    thread::sleep(Duration::from_millis(20));
                }
            }
        }
    }
}

#[cfg(windows)]
fn set_receive_buffer(socket: &UdpSocket, bytes: usize) -> io::Result<()> {
    use std::os::windows::io::AsRawSocket;
    use windows_sys::Win32::Networking::WinSock::{SO_RCVBUF, SOL_SOCKET, setsockopt};
    let value = i32::try_from(bytes).unwrap_or(i32::MAX);
    // SAFETY: valid socket handle owned by `socket`; optval points to a live i32
    // of the advertised length for the duration of the call.
    let result = unsafe {
        setsockopt(
            socket.as_raw_socket() as usize,
            SOL_SOCKET,
            SO_RCVBUF,
            (&value as *const i32).cast(),
            std::mem::size_of::<i32>() as i32,
        )
    };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}
#[cfg(not(windows))]
fn set_receive_buffer(_socket: &UdpSocket, _bytes: usize) -> io::Result<()> {
    Ok(())
}

/// Raise the calling thread one priority step (THREAD_PRIORITY_ABOVE_NORMAL).
pub fn raise_current_thread_priority() {
    #[cfg(windows)]
    {
        use windows_sys::Win32::System::Threading::{
            GetCurrentThread, SetThreadPriority, THREAD_PRIORITY_ABOVE_NORMAL,
        };
        // SAFETY: GetCurrentThread returns a pseudo-handle valid for this thread.
        unsafe {
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
        }
    }
}

/// Requests 1 ms timer resolution for this process while alive, so 4-10 ms
/// waits are not rounded up to the 15.6 ms default tick.
pub struct TimerResolution {
    #[cfg(windows)]
    active: bool,
}
impl TimerResolution {
    #[cfg(windows)]
    pub fn request() -> Self {
        // SAFETY: plain winmm call; balanced by timeEndPeriod in Drop.
        let active = unsafe { windows_sys::Win32::Media::timeBeginPeriod(1) } == 0;
        Self { active }
    }
    #[cfg(not(windows))]
    pub fn request() -> Self {
        Self {}
    }
}
#[cfg(windows)]
impl Drop for TimerResolution {
    fn drop(&mut self) {
        if self.active {
            // SAFETY: balances the successful timeBeginPeriod(1) above.
            unsafe {
                windows_sys::Win32::Media::timeEndPeriod(1);
            }
        }
    }
}

pub struct TrackerHub {
    socket: UdpSocket,
    events: mpsc::Receiver<ReceiverEvent>,
    pending: Option<ReceiverEvent>,
    receiver_stop: Arc<AtomicBool>,
    receiver_thread: Option<JoinHandle<()>>,
    dropped: Arc<AtomicU64>,
    receiver_lost: bool,
    devices: BTreeMap<SocketAddr, Device>,
    sensors: BTreeMap<String, Sensor>,
    history: BTreeMap<String, PriorSession>,
    last_heartbeat: f64,
    last_capture_flush: f64,
    last_notice: Option<f64>,
    flushing: bool,
    pub packets: u64,
    pub malformed: u64,
    /// Datagram types from validated sources only; unknown types share one bucket.
    pub datagram_types: BTreeMap<u32, u64>,
    pub payload_types: BTreeMap<String, u64>,
    pub sequence_missing: u64,
    pub sequence_duplicates: u64,
    pub sequence_out_of_order: u64,
    pub transport_resets: u64,
    pub receive_dropped: u64,
    /// Identities whose session restarted without proof of the same IMU boot.
    pub reconnected: Vec<String>,
    pub taps: Vec<String>,
    pub revocations: BTreeMap<String, bool>,
    capture: Option<WireCapture>,
    pub capture_error: Option<String>,
    /// User-visible transport problems (rate limited).
    pub notice: Option<String>,
}

fn packet_label(packet: &Packet) -> &'static str {
    match packet {
        Packet::Heartbeat => "heartbeat",
        Packet::Handshake(_) => "handshake",
        Packet::Rotation { data_type: 1, .. } => "rotation_normal",
        Packet::Rotation { .. } => "rotation_other",
        Packet::RotationAcceleration { .. } => "rotation_acceleration",
        Packet::Acceleration { .. } => "acceleration",
        Packet::SensorInfo(_) => "sensor_info",
        Packet::Battery { .. } => "battery",
        Packet::Signal { .. } => "signal",
        Packet::Temperature { .. } => "temperature",
        Packet::Tap { .. } => "tap",
        Packet::UserAction { .. } => "user_action",
        Packet::SensorError { .. } => "sensor_error",
        Packet::MagnetometerAccuracy { .. } => "magnetometer_accuracy",
        Packet::FeatureFlags(_) => "feature_flags",
        Packet::PingPong(_) => "ping",
        Packet::Serial(_) => "serial",
        Packet::AckConfig { .. } => "config_ack",
        Packet::Flex { .. } => "flex",
        Packet::Position { .. } => "position",
        Packet::ProtocolChange { .. } => "protocol_change",
        Packet::RawImu { .. } => "raw_imu",
        Packet::Unknown { .. } => "unknown",
    }
}

/// Slime-compatible firmware world is Z-up; the core uses Y-up.
/// This fixed world-basis rotation is distinct from sensor-to-body mounting.
pub fn firmware_to_tracking(q: [f32; 4]) -> Option<[f32; 4]> {
    if !q.iter().all(|x| x.is_finite()) {
        return None;
    }
    let q = Quat::from_array(q);
    if !(0.5..=1.5).contains(&q.length_squared()) {
        return None;
    }
    Some((Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2) * q.normalize()).to_array())
}

fn sensor_id_for(device: &Device, source: SocketAddr, sensor_id: u8) -> String {
    format!(
        "{}/{}",
        if device.name.is_empty() {
            source.ip().to_string()
        } else {
            device.name.clone()
        },
        sensor_id
    )
}

impl TrackerHub {
    /// `origin` is the worker clock origin; datagram stamps use the same clock.
    pub fn bind(origin: Instant) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", tracker_protocol::TRACKER_PORT))?;
        let buffer_notice = set_receive_buffer(&socket, RECEIVE_BUFFER_BYTES)
            .err()
            .map(|error| format!("Tracker receive buffer could not be enlarged: {error}"));
        let (capture, capture_error) = match std::env::var_os("CHADFBT_CAPTURE_WIRE") {
            Some(path) => match WireCapture::open(&path) {
                Ok(capture) => (Some(capture), None),
                Err(error) => (None, Some(format!("Cannot start wire capture: {error}"))),
            },
            None => (None, None),
        };
        let receive_socket = socket.try_clone()?;
        let (sender, events) = mpsc::sync_channel(EVENT_CAPACITY);
        let mut hub = Self::with_parts(socket, events, capture, capture_error);
        hub.notice = buffer_notice;
        let receive_loop = ReceiveLoop {
            socket: receive_socket,
            origin,
            events: sender,
            dropped: hub.dropped.clone(),
            stop: hub.receiver_stop.clone(),
        };
        hub.receiver_thread = Some(
            thread::Builder::new()
                .name("tracker-receive".into())
                .spawn(move || receive_loop.run())?,
        );
        Ok(hub)
    }
    fn with_parts(
        socket: UdpSocket,
        events: mpsc::Receiver<ReceiverEvent>,
        capture: Option<WireCapture>,
        capture_error: Option<String>,
    ) -> Self {
        Self {
            socket,
            events,
            pending: None,
            receiver_stop: Arc::new(AtomicBool::new(false)),
            receiver_thread: None,
            dropped: Arc::new(AtomicU64::new(0)),
            receiver_lost: false,
            devices: BTreeMap::new(),
            sensors: BTreeMap::new(),
            history: BTreeMap::new(),
            last_heartbeat: -1.,
            last_capture_flush: 0.,
            last_notice: None,
            flushing: false,
            packets: 0,
            malformed: 0,
            datagram_types: BTreeMap::new(),
            payload_types: BTreeMap::new(),
            sequence_missing: 0,
            sequence_duplicates: 0,
            sequence_out_of_order: 0,
            transport_resets: 0,
            receive_dropped: 0,
            reconnected: Vec::new(),
            taps: Vec::new(),
            revocations: BTreeMap::new(),
            capture,
            capture_error,
            notice: None,
        }
    }

    /// Blocks until a datagram/event is available or `timeout` passes.
    /// Returns true when an event is ready for `poll`.
    pub fn wait(&mut self, timeout: Duration) -> bool {
        if self.pending.is_some() {
            return true;
        }
        if self.receiver_lost {
            thread::sleep(timeout);
            return false;
        }
        match self.events.recv_timeout(timeout) {
            Ok(event) => {
                self.pending = Some(event);
                true
            }
            Err(RecvTimeoutError::Timeout) => false,
            Err(RecvTimeoutError::Disconnected) => {
                self.lose_receiver();
                false
            }
        }
    }

    fn lose_receiver(&mut self) {
        if !self.receiver_lost {
            self.receiver_lost = true;
            self.notice =
                Some("Tracker receive thread stopped; restart ChadFBT to receive trackers.".into());
        }
    }

    fn set_notice(&mut self, at: f64, text: String) {
        if self
            .last_notice
            .is_none_or(|last| at - last >= 30.0 || at < last)
        {
            self.last_notice = Some(at);
            self.notice = Some(text);
        }
    }

    /// Backlog may carry stale stamps. Suspend measured and held output for every
    /// sensor until the queue is drained; do not escalate yet (our own scheduling
    /// is not evidence about any tracker).
    fn begin_drain(&mut self) {
        if !self.flushing {
            self.transport_resets = self.transport_resets.saturating_add(1);
        }
        self.flushing = true;
        for sensor in self.sensors.values_mut() {
            sensor.drain_pending = true;
            sensor.drain_rotation = None;
        }
        for device in self.devices.values_mut() {
            device.drain_gap = false;
            device.drain_datagrams = 0;
        }
    }

    /// The queue is empty. Sensors whose device stayed sequence-contiguous and
    /// healthy keep their state; the newest drained rotation (stamped with the
    /// drain time) restores a measurement. A real gap or error escalates.
    fn finish_drain(&mut self, at: f64, measurements: &mut Vec<Measurement>) {
        if !self.flushing {
            return;
        }
        self.flushing = false;
        for (id, sensor) in &mut self.sensors {
            if !sensor.drain_pending {
                continue;
            }
            sensor.drain_pending = false;
            let rotation = sensor.drain_rotation.take();
            let Some(device) = sensor
                .device_source
                .and_then(|source| self.devices.get_mut(&source))
            else {
                sensor.measurement_valid = false;
                continue;
            };
            let healthy = device
                .sensor_status
                .get(&sensor.sensor_id)
                .is_none_or(|status| *status == 1);
            if device.drain_gap || !healthy {
                sensor.measurement_valid = false;
                sensor.hold_valid = false;
                sensor.continuity_since = None;
                self.revocations.insert(id.clone(), true);
                continue;
            }
            if device.drain_datagrams > 0 && at > device.last_seen {
                device.last_seen = at;
            }
            if let Some(q) = rotation {
                if !sensor.hold_valid {
                    sensor.continuity_since = Some(at);
                }
                sensor.hold_valid = true;
                self.revocations.remove(id);
                sensor.view.orientation = q;
                sensor.view.packets += 1;
                sensor.mark_measured(at);
                device.lineage = true;
                measurements.push(Measurement {
                    id: id.clone(),
                    orientation: q,
                    acceleration: None,
                    at,
                    provenance: tracking_core::SensorProvenance::Measured,
                });
            }
        }
        for device in self.devices.values_mut() {
            device.drain_gap = false;
            device.drain_datagrams = 0;
        }
    }

    // During drain, use validated known-session packets only to advance the
    // replay watermark, track contiguity and keep the newest rotation per sensor.
    // Liveness, taps and measurements are not refreshed from backlog stamps.
    fn observe_drained(&mut self, source: SocketAddr, bytes: &[u8]) {
        let Ok(message) = tracker_protocol::parse_datagram(bytes) else {
            if let Some(device) = self.devices.get_mut(&source) {
                self.malformed += 1;
                device.drain_gap = true;
            }
            return;
        };
        if message
            .packets
            .iter()
            .any(|packet| matches!(packet, Packet::Handshake(_)))
        {
            // Do not keep an unacknowledged session alive; it must re-handshake.
            if let Some(mut device) = self.devices.remove(&source) {
                remember_session(&mut self.history, &mut self.reconnected, &mut device);
                for (id, sensor) in &mut self.sensors {
                    if sensor.device_source == Some(source) {
                        sensor.measurement_valid = false;
                        sensor.hold_valid = false;
                        sensor.continuity_since = None;
                        self.revocations.insert(id.clone(), true);
                    }
                }
            }
            return;
        }
        if message.packet_type == 10 {
            return;
        } // server-sequence ping echo
        let Some(device) = self.devices.get_mut(&source) else {
            return;
        };
        *self
            .datagram_types
            .entry(datagram_type_bucket(message.packet_type))
            .or_default() += 1;
        device.drain_datagrams += 1;
        match device.sequence.observe(message.sequence, false) {
            SequenceObservation::Next { missing } => {
                device.last_sequence = Some(message.sequence);
                self.sequence_missing = self.sequence_missing.saturating_add(missing);
                device.sequence_missing = device.sequence_missing.saturating_add(missing);
                device.drain_gap |= missing != 0;
            }
            SequenceObservation::First => {
                device.last_sequence = Some(message.sequence);
                verify_continuation(device, message.sequence, &mut self.reconnected);
            }
            SequenceObservation::Duplicate => {
                self.sequence_duplicates += 1;
                device.drain_gap = true;
                return;
            }
            SequenceObservation::OutOfOrder => {
                self.sequence_out_of_order += 1;
                device.drain_gap = true;
                return;
            }
            SequenceObservation::HandshakeReset => {}
        }
        for packet in message.packets {
            match packet {
                Packet::Unknown { .. } => device.drain_gap = true,
                Packet::SensorInfo(info) => {
                    device.sensor_status.insert(info.sensor_id, info.status);
                    if info.status != 1 {
                        revoke_sensor(
                            &mut self.sensors,
                            &mut self.revocations,
                            source,
                            info.sensor_id,
                        );
                    }
                }
                Packet::SensorError { sensor_id, .. } => {
                    device.sensor_status.insert(sensor_id, 2);
                    revoke_sensor(&mut self.sensors, &mut self.revocations, source, sensor_id);
                }
                Packet::Battery { level, .. } => device.battery = Some(level.clamp(0., 1.)),
                Packet::Rotation {
                    sensor_id,
                    data_type: 1,
                    quaternion_xyzw,
                    ..
                }
                | Packet::RotationAcceleration {
                    sensor_id,
                    quaternion_xyzw,
                    ..
                } => {
                    if device
                        .sensor_status
                        .get(&sensor_id)
                        .is_some_and(|status| *status != 1)
                    {
                        continue;
                    }
                    let id = sensor_id_for(device, source, sensor_id);
                    match (
                        firmware_to_tracking(quaternion_xyzw),
                        self.sensors.get_mut(&id),
                    ) {
                        (Some(q), Some(sensor)) => sensor.drain_rotation = Some(q),
                        (None, _) => {
                            self.malformed += 1;
                            device.drain_gap = true;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }

    fn handle_datagram(&mut self, received: Received, measurements: &mut Vec<Measurement>) {
        let Received {
            bytes,
            source,
            at,
            backlog,
        } = received;
        if let Some(capture) = &mut self.capture
            && let Err(error) = capture.capture(at, source, &bytes)
        {
            self.capture_error = Some(format!("Wire capture stopped: {error}"));
            self.capture = None;
        }
        if self.flushing || backlog {
            self.observe_drained(source, &bytes);
            return;
        }
        self.packets += 1;
        let message = match tracker_protocol::parse_datagram(&bytes) {
            Ok(v) => v,
            Err(_) => {
                self.malformed += 1;
                if self.devices.contains_key(&source) {
                    revoke_source(&mut self.sensors, &mut self.revocations, source);
                }
                return;
            }
        };
        // A tracker already bound to an earlier server can send rotations before
        // discovery. Do not create an IP-named sensor or keep that old session
        // alive: with no replies it will time out and send a real handshake.
        let handshake_info = message.packets.iter().find_map(|packet| match packet {
            Packet::Handshake(info) => {
                handshake_identity(info, source).map(|identity| (info, identity))
            }
            _ => None,
        });
        let is_handshake = handshake_info.is_some();
        if !is_handshake && !self.devices.contains_key(&source) {
            return;
        }
        if let Some((info, identity)) = handshake_info {
            revoke_session(&self.sensors, &mut self.revocations, source, &identity);
            if !establish_session(
                Session {
                    devices: &mut self.devices,
                    sensors: &mut self.sensors,
                    history: &mut self.history,
                    reconnected: &mut self.reconnected,
                },
                source,
                identity,
                info.firmware.clone(),
                info.mac.is_some(),
            ) {
                return;
            }
        }
        // Count only validated sources, so arbitrary LAN traffic cannot grow this map.
        *self
            .datagram_types
            .entry(datagram_type_bucket(message.packet_type))
            .or_default() += 1;
        let device = self
            .devices
            .get_mut(&source)
            .expect("known endpoint or validated handshake");
        if !is_handshake && message.packet_type != 10 {
            let previous_at = device.last_seen;
            match device.sequence.observe(message.sequence, false) {
                SequenceObservation::Duplicate => {
                    self.sequence_duplicates += 1;
                    revoke_source(&mut self.sensors, &mut self.revocations, source);
                    return;
                }
                SequenceObservation::OutOfOrder => {
                    self.sequence_out_of_order += 1;
                    revoke_source(&mut self.sensors, &mut self.revocations, source);
                    return;
                }
                SequenceObservation::First => {
                    device.last_sequence = Some(message.sequence);
                    verify_continuation(device, message.sequence, &mut self.reconnected);
                }
                SequenceObservation::Next { missing } => {
                    device.last_sequence = Some(message.sequence);
                    self.sequence_missing = self.sequence_missing.saturating_add(missing);
                    device.sequence_missing = device.sequence_missing.saturating_add(missing);
                    if missing != 0 {
                        for (id, sensor) in &mut self.sensors {
                            if sensor.device_source == Some(source)
                                && sensor.hold_valid
                                && gap_may_hide_rotation(
                                    previous_at,
                                    sensor.last_rotation,
                                    missing,
                                    at - previous_at,
                                )
                            {
                                sensor.revoke_hold(id, &mut self.revocations);
                            }
                        }
                    }
                }
                SequenceObservation::HandshakeReset => {}
            }
        }
        device.last_seen = at;
        device.accepted_packets += 1;
        for reply in tracker_protocol::session_replies(&message) {
            let _ = self.socket.send_to(&reply, source);
        }
        for packet in message.packets {
            *self
                .payload_types
                .entry(packet_label(&packet).to_owned())
                .or_default() += 1;
            match packet {
                Packet::Handshake(_) => {} // handled before endpoint registration
                Packet::Unknown { .. } => {
                    // An unrecognized payload could be a rotation or error; it
                    // cannot extend the compatibility assumption.
                    revoke_source(&mut self.sensors, &mut self.revocations, source);
                }
                Packet::SensorInfo(info) => {
                    // Each sensor reports its own status (extension sensors included);
                    // only a non-OK status affects that sensor.
                    device.sensor_status.insert(info.sensor_id, info.status);
                    if info.status != 1 {
                        revoke_sensor(
                            &mut self.sensors,
                            &mut self.revocations,
                            source,
                            info.sensor_id,
                        );
                    }
                }
                Packet::SensorError { sensor_id, .. } => {
                    device.sensor_status.insert(sensor_id, 2);
                    revoke_sensor(&mut self.sensors, &mut self.revocations, source, sensor_id);
                }
                Packet::Battery { level, .. } => device.battery = Some(level.clamp(0., 1.)),
                Packet::Rotation {
                    sensor_id,
                    data_type: 1,
                    quaternion_xyzw,
                    ..
                }
                | Packet::RotationAcceleration {
                    sensor_id,
                    quaternion_xyzw,
                    ..
                } => {
                    // An explicit error/offline sensor is not revived by data until
                    // its own status returns healthy and a new quaternion arrives.
                    if device
                        .sensor_status
                        .get(&sensor_id)
                        .is_some_and(|status| *status != 1)
                    {
                        continue;
                    }
                    let Some(q) = firmware_to_tracking(quaternion_xyzw) else {
                        self.malformed += 1;
                        continue;
                    };
                    let id = sensor_id_for(device, source, sensor_id);
                    if self.sensors.len() >= 128 && !self.sensors.contains_key(&id) {
                        continue;
                    }
                    let sensor = self.sensors.entry(id.clone()).or_default();
                    sensor.device_source = Some(source);
                    sensor.sensor_id = sensor_id;
                    if !sensor.hold_valid {
                        sensor.continuity_since = Some(at);
                    }
                    sensor.hold_valid = device
                        .sensor_status
                        .get(&sensor_id)
                        .is_none_or(|status| *status == 1);
                    if sensor.hold_valid {
                        self.revocations.remove(&id);
                    }
                    sensor.view.id = id.clone();
                    sensor.view.label = format!(
                        "{} · {}",
                        device
                            .name
                            .chars()
                            .rev()
                            .take(8)
                            .collect::<String>()
                            .chars()
                            .rev()
                            .collect::<String>(),
                        sensor_id
                    );
                    sensor.view.firmware = device.firmware.clone();
                    sensor.view.orientation = q;
                    sensor.view.battery = device.battery;
                    sensor.view.packets += 1;
                    sensor.mark_measured(at);
                    device.lineage = true;
                    if let Packet::RotationAcceleration { acceleration, .. } = packet {
                        sensor.view.acceleration = Some(acceleration);
                        sensor.last_accel = Some(at);
                        sensor.view.acceleration_packets += 1;
                    }
                    measurements.push(Measurement {
                        id,
                        orientation: q,
                        acceleration: if sensor.last_accel.is_some_and(|t| at - t < 0.25) {
                            sensor.view.acceleration
                        } else {
                            None
                        },
                        at,
                        provenance: tracking_core::SensorProvenance::Measured,
                    });
                }
                Packet::Acceleration {
                    sensor_id,
                    acceleration,
                } => {
                    let id = sensor_id_for(device, source, sensor_id);
                    if let Some(sensor) = self.sensors.get_mut(&id) {
                        sensor.view.acceleration = Some(acceleration);
                        sensor.last_accel = Some(at);
                        sensor.view.acceleration_packets += 1;
                    }
                }
                Packet::Tap { sensor_id, bits } if bits & 0x40 != 0 => {
                    let id = format!("{}/{}", device.name, sensor_id);
                    if let Some(sensor) = self.sensors.get_mut(&id) {
                        sensor.view.last_tap_seconds = Some(at as f32);
                    }
                    self.taps.push(id);
                }
                _ => {}
            }
        }
    }

    /// Handle every stamped datagram received so far (bounded per call), then
    /// emit held-orientation evidence at `now` and send due heartbeats.
    pub fn poll(&mut self, now: f64) -> Vec<Measurement> {
        let mut measurements = Vec::new();
        for _ in 0..MAX_EVENTS_PER_POLL {
            let event = match self.pending.take() {
                Some(event) => event,
                None => match self.events.try_recv() {
                    Ok(event) => event,
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.lose_receiver();
                        break;
                    }
                },
            };
            match event {
                ReceiverEvent::Datagram(received) => {
                    self.handle_datagram(received, &mut measurements)
                }
                ReceiverEvent::Disrupted { error } => {
                    if let Some(error) = error {
                        self.set_notice(now, error);
                    }
                    self.begin_drain();
                }
                ReceiverEvent::CaughtUp { at } => self.finish_drain(at, &mut measurements),
            }
        }
        self.receive_dropped = self.dropped.load(Ordering::Relaxed);
        let measured_ids: BTreeSet<_> = measurements.iter().map(|m| m.id.clone()).collect();
        for (id, sensor) in &mut self.sensors {
            let Some(device) = sensor
                .device_source
                .and_then(|source| self.devices.get(&source))
            else {
                continue;
            };
            // A drain decides liveness when it completes.
            if sensor.hold_valid
                && !sensor.drain_pending
                && now - device.last_seen > HOLD_TRANSPORT_LEASE_SECS
            {
                sensor.revoke_hold(id, &mut self.revocations);
            }
            if measured_ids.contains(id) || !held_eligible(sensor, device, now) {
                continue;
            }
            measurements.push(Measurement {
                id: id.clone(),
                orientation: sensor.view.orientation,
                acceleration: None,
                at: sensor.last_rotation,
                provenance: tracking_core::SensorProvenance::FirmwareHeld {
                    firmware: device.firmware.clone(),
                    observed_at_secs: now,
                    last_transport_at_secs: device.last_seen,
                    continuity_since_secs: sensor.continuity_since.unwrap_or(sensor.last_rotation),
                    sensor_status_healthy: true,
                    // No sequence gap that could have hidden a rotation since continuity began.
                    sequence_contiguous: true,
                },
            });
        }
        if now - self.last_capture_flush >= 1.0 || now < self.last_capture_flush {
            self.last_capture_flush = now;
            if let Some(capture) = &mut self.capture
                && let Err(error) = capture.flush()
            {
                self.capture_error = Some(format!("Wire capture flush failed: {error}"));
                self.capture = None;
            }
        }
        if now - self.last_heartbeat >= 0.5 {
            self.last_heartbeat = now;
            for (addr, device) in &self.devices {
                if now - device.last_seen < 10. {
                    let _ = self.socket.send_to(&tracker_protocol::heartbeat(), addr);
                }
            }
        }
        // Trackers broadcast their own discovery handshakes. Broadcasting from the
        // server would keep an unrecognized old tracker session alive indefinitely,
        // because stock firmware refreshes its timeout for every inbound datagram.
        measurements
    }

    pub fn views(&self, now: f64) -> Vec<TrackerView> {
        self.sensors
            .values()
            .map(|s| {
                let mut v = s.view.clone();
                v.age_seconds = (now - s.last_rotation).max(0.) as f32;
                v.transport_invalidated = !s.measurement_valid || s.drain_pending;
                v.acceleration_age_seconds = s.last_accel.map(|t| (now - t).max(0.) as f32);
                if let Some(device) = s.device_source.and_then(|source| self.devices.get(&source)) {
                    v.device_packet_age_seconds = Some((now - device.last_seen).max(0.) as f32);
                    v.device_packets = device.accepted_packets;
                    v.device_sequence_missing = device.sequence_missing;
                    v.estimated_held = v.age_seconds > 0.25 && held_eligible(s, device, now);
                    v.eligible_for_calibration = s.measurement_valid
                        && !s.drain_pending
                        && ((v.age_seconds <= 0.25
                            && device
                                .sensor_status
                                .get(&s.sensor_id)
                                .is_none_or(|status| *status == 1))
                            || v.estimated_held);
                }
                if let Some(t) = v.last_tap_seconds {
                    v.last_tap_seconds = Some((now as f32 - t).max(0.));
                }
                if v.age_seconds > 1. {
                    v.hz = 0.;
                }
                v
            })
            .collect()
    }
}

impl Drop for TrackerHub {
    fn drop(&mut self) {
        self.receiver_stop.store(true, Ordering::Release);
        if let Some(handle) = self.receiver_thread.take() {
            // Keep the queue moving so a blocked control send can finish.
            let deadline = Instant::now() + Duration::from_secs(2);
            while !handle.is_finished() && Instant::now() < deadline {
                while self.events.try_recv().is_ok() {}
                thread::sleep(Duration::from_millis(5));
            }
            if handle.is_finished() {
                let _ = handle.join();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const SOURCE: &str = "127.0.0.1:1234";
    const MAC: [u8; 6] = [1, 2, 3, 4, 5, 6];
    const ID: &str = "01:02:03:04:05:06/0";

    fn test_hub() -> (TrackerHub, SyncSender<ReceiverEvent>) {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        let (sender, events) = mpsc::sync_channel(EVENT_CAPACITY);
        (TrackerHub::with_parts(socket, events, None, None), sender)
    }
    fn addr() -> SocketAddr {
        SOURCE.parse().unwrap()
    }
    fn datagram(kind: u32, sequence: u64, payload: &[u8]) -> Vec<u8> {
        let mut data = kind.to_be_bytes().to_vec();
        data.extend(sequence.to_be_bytes());
        data.extend(payload);
        data
    }
    fn handshake(firmware: &str) -> Vec<u8> {
        let mut payload = Vec::new();
        for value in [1u32, 1, 1, 0, 0, 0, 21] {
            payload.extend(value.to_be_bytes());
        }
        payload.push(firmware.len() as u8);
        payload.extend(firmware.as_bytes());
        payload.extend(MAC);
        datagram(3, 0, &payload)
    }
    fn rotation(sequence: u64, sensor_id: u8, angle: f32) -> Vec<u8> {
        let (s, c) = (angle / 2.).sin_cos();
        let mut payload = vec![sensor_id, 1];
        for v in [0., 0., s, c] {
            payload.extend(f32::to_be_bytes(v));
        }
        payload.push(3);
        datagram(17, sequence, &payload)
    }
    fn sensor_info(sequence: u64, sensor_id: u8, status: u8) -> Vec<u8> {
        datagram(15, sequence, &[sensor_id, status])
    }
    fn heartbeat(sequence: u64) -> Vec<u8> {
        datagram(0, sequence, &[])
    }
    fn send(tx: &SyncSender<ReceiverEvent>, bytes: Vec<u8>, at: f64) {
        tx.send(ReceiverEvent::Datagram(Received {
            bytes,
            source: addr(),
            at,
            backlog: false,
        }))
        .unwrap();
    }
    fn send_backlog(tx: &SyncSender<ReceiverEvent>, bytes: Vec<u8>, at: f64) {
        tx.send(ReceiverEvent::Datagram(Received {
            bytes,
            source: addr(),
            at,
            backlog: true,
        }))
        .unwrap();
    }
    fn held(measurements: &[Measurement]) -> bool {
        measurements.iter().any(|m| {
            matches!(
                m.provenance,
                tracking_core::SensorProvenance::FirmwareHeld { .. }
            )
        })
    }
    /// Handshake, healthy status and one measured rotation at t=1.0 (sequence 2).
    fn connected(firmware: &str, sensor_id: u8) -> (TrackerHub, SyncSender<ReceiverEvent>) {
        let (mut hub, tx) = test_hub();
        send(&tx, handshake(firmware), 0.9);
        send(&tx, sensor_info(1, sensor_id, 1), 0.95);
        send(&tx, rotation(2, sensor_id, 0.1), 1.0);
        let measured = hub.poll(1.0);
        assert_eq!(measured.len(), 1);
        (hub, tx)
    }

    #[test]
    fn datagrams_keep_receive_stamps_not_poll_time() {
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, rotation(3, 0, 0.2), 1.010);
        send(&tx, rotation(4, 0, 0.3), 1.014);
        let measured = hub.poll(1.020);
        let stamps: Vec<_> = measured.iter().map(|m| m.at).collect();
        assert_eq!(stamps, vec![1.010, 1.014]);
    }

    #[test]
    fn held_orientation_accepts_dev_builds_and_extension_sensors() {
        for (firmware, sensor_id) in [("0.7.2", 0), ("main", 0), ("git-3fa2b1", 0), ("0.6.3", 1)] {
            let (mut hub, tx) = connected(firmware, sensor_id);
            send(&tx, heartbeat(3), 1.4);
            let out = hub.poll(1.5);
            assert!(held(&out), "{firmware}/{sensor_id} should hold");
            // A non-OK status of the sensor itself still stops holding immediately.
            send(&tx, sensor_info(4, sensor_id, 2), 1.6);
            let out = hub.poll(1.6);
            assert!(!held(&out));
            let id = format!("01:02:03:04:05:06/{sensor_id}");
            assert!(hub.revocations[&id]);
        }
        // A second healthy sensor on the device no longer revokes the first.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, sensor_info(3, 1, 1), 1.2);
        assert!(held(&hub.poll(1.5)));
    }

    #[test]
    fn isolated_idle_loss_keeps_hold_but_loss_after_rotation_revokes() {
        // Still tracker: last rotation at 1.0, heartbeat reply at 1.5, one lost, next at 2.5.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, heartbeat(3), 1.5);
        send(&tx, heartbeat(5), 2.5);
        let out = hub.poll(2.5);
        assert!(held(&out));
        assert!(!hub.revocations.contains_key(ID));
        assert_eq!(hub.sequence_missing, 1);

        // Loss right after the final rotation of a movement revokes.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, heartbeat(4), 1.3);
        assert!(!held(&hub.poll(1.3)));
        assert_eq!(hub.revocations.get(ID), Some(&false));

        // Several losses in a short span mean the tracker was streaming.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, heartbeat(3), 1.5);
        send(&tx, heartbeat(7), 1.6);
        assert!(!held(&hub.poll(1.6)));
    }

    #[test]
    fn gap_heuristic_boundaries() {
        assert!(!gap_may_hide_rotation(5.0, 1.0, 0, 0.0));
        assert!(gap_may_hide_rotation(1.02, 1.0, 1, 0.5));
        assert!(!gap_may_hide_rotation(1.5, 1.0, 1, 1.0));
        assert!(!gap_may_hide_rotation(1.5, 1.0, 2, 1.0));
        assert!(gap_may_hide_rotation(1.5, 1.0, 3, 0.4));
        assert!(gap_may_hide_rotation(f64::NAN, 1.0, 1, 1.0));
    }

    #[test]
    fn connection_reset_is_not_a_transport_fault() {
        assert!(is_connection_reset(&io::Error::from_raw_os_error(10054)));
        assert!(is_connection_reset(&io::Error::from(
            io::ErrorKind::ConnectionReset
        )));
        assert!(!is_connection_reset(&io::Error::from(
            io::ErrorKind::PermissionDenied
        )));
    }

    #[test]
    fn receive_thread_survives_port_unreachable_and_stamps_datagrams() {
        let origin = Instant::now();
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        let local = socket.local_addr().unwrap();
        // Provoke ICMP port unreachable: send to a closed loopback port.
        let closed = UdpSocket::bind("127.0.0.1:0").unwrap();
        let closed_addr = closed.local_addr().unwrap();
        drop(closed);
        let _ = socket.send_to(b"x", closed_addr);
        let (sender, events) = mpsc::sync_channel(16);
        let stop = Arc::new(AtomicBool::new(false));
        let receive_loop = ReceiveLoop {
            socket: socket.try_clone().unwrap(),
            origin,
            events: sender,
            dropped: Arc::new(AtomicU64::new(0)),
            stop: stop.clone(),
        };
        let handle = thread::spawn(move || receive_loop.run());
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.send_to(&heartbeat(1), local).unwrap();
        let before = origin.elapsed().as_secs_f64();
        let event = events.recv_timeout(Duration::from_secs(5)).unwrap();
        let ReceiverEvent::Datagram(received) = event else {
            panic!("expected a datagram, not a disruption: {event:?}");
        };
        assert!(!received.backlog);
        assert!(received.at <= origin.elapsed().as_secs_f64());
        assert!(received.at + 1.0 >= before);
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    }

    #[test]
    fn stall_drain_restores_contiguous_sensors_with_newest_rotation() {
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted { error: None }).unwrap();
        send_backlog(&tx, rotation(3, 0, 0.4), 1.8);
        send_backlog(&tx, rotation(4, 0, 0.5), 1.8);
        send_backlog(&tx, heartbeat(5), 1.8);
        let out = hub.poll(1.8);
        assert!(out.is_empty(), "backlog must not produce measurements");
        assert!(hub.views(1.8)[0].transport_invalidated);
        assert!(
            hub.revocations.is_empty(),
            "a stall alone must not escalate"
        );
        tx.send(ReceiverEvent::CaughtUp { at: 1.85 }).unwrap();
        let out = hub.poll(1.85);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].at, 1.85);
        assert_eq!(out[0].provenance, tracking_core::SensorProvenance::Measured);
        let expected = firmware_to_tracking([0., 0., 0.25f32.sin(), 0.25f32.cos()]).unwrap();
        assert_eq!(out[0].orientation, expected);
        assert!(!hub.views(1.85)[0].transport_invalidated);
        assert_eq!(hub.transport_resets, 1);
        // Sequence watermark advanced through the drain.
        send(&tx, heartbeat(5), 1.9);
        hub.poll(1.9);
        assert_eq!(hub.sequence_duplicates, 1);
    }

    #[test]
    fn stall_without_new_rotation_keeps_still_sensor_holding() {
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted { error: None }).unwrap();
        send_backlog(&tx, heartbeat(3), 1.4);
        tx.send(ReceiverEvent::CaughtUp { at: 1.45 }).unwrap();
        let out = hub.poll(1.5);
        assert!(held(&out));
        assert!(hub.revocations.is_empty());
    }

    #[test]
    fn gap_or_error_during_drain_escalates_until_new_quaternion() {
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted { error: None }).unwrap();
        send_backlog(&tx, heartbeat(3), 1.4);
        send_backlog(&tx, rotation(6, 0, 0.3), 1.4);
        tx.send(ReceiverEvent::CaughtUp { at: 1.45 }).unwrap();
        let out = hub.poll(1.5);
        assert!(out.is_empty());
        assert!(hub.revocations[ID]);
        assert!(hub.views(1.5)[0].transport_invalidated);
        send(&tx, rotation(7, 0, 0.3), 1.6);
        let out = hub.poll(1.6);
        assert_eq!(out.len(), 1);
        assert!(!hub.views(1.6)[0].transport_invalidated);

        // A reported sensor error inside the backlog invalidates immediately.
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted { error: None }).unwrap();
        send_backlog(&tx, datagram(14, 3, &[0, 1]), 1.4);
        hub.poll(1.4);
        assert!(hub.revocations[ID]);
        tx.send(ReceiverEvent::CaughtUp { at: 1.45 }).unwrap();
        assert!(hub.poll(1.5).is_empty());
        assert!(hub.views(1.5)[0].transport_invalidated);
    }

    #[test]
    fn receive_error_reports_notice_and_drains() {
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted {
            error: Some("Tracker socket receive failed: test".into()),
        })
        .unwrap();
        hub.poll(1.1);
        assert!(hub.notice.take().is_some());
        assert!(hub.flushing);
        assert!(hub.revocations.is_empty());
    }

    #[test]
    fn datagram_types_count_only_validated_sources_and_bucket_unknown() {
        let (mut hub, tx) = test_hub();
        // Unknown endpoint: ignored entirely.
        for kind in 1000..1100u32 {
            send(&tx, datagram(kind, 1, &[]), 0.1);
        }
        hub.poll(0.1);
        assert!(hub.datagram_types.is_empty());
        let (mut hub, tx) = connected("0.7.2", 0);
        for (sequence, kind) in (3..).zip(5000..5100u32) {
            send(&tx, datagram(kind, sequence, &[]), 1.1);
        }
        hub.poll(1.1);
        assert_eq!(hub.datagram_types[&OTHER_DATAGRAM_TYPE], 100);
        assert!(hub.datagram_types.len() <= 5);
    }

    #[test]
    fn same_boot_rehandshake_is_not_a_reconnect_but_reboot_is() {
        // Same boot: sequence continues after re-handshake.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, handshake("0.7.2"), 5.0);
        send(&tx, sensor_info(9, 0, 1), 5.1);
        hub.poll(5.1);
        assert!(hub.reconnected.is_empty());
        // Even if the next session is idle, a later reboot is still detected.
        send(&tx, handshake("0.7.2"), 9.0);
        send(&tx, sensor_info(0, 0, 1), 9.1);
        hub.poll(9.1);
        assert_eq!(hub.reconnected, vec!["01:02:03:04:05:06".to_owned()]);

        // Reboot: counter restarts.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, handshake("0.7.2"), 5.0);
        hub.poll(5.0);
        assert!(hub.reconnected.is_empty(), "undecided until data arrives");
        send(&tx, sensor_info(0, 0, 1), 5.1);
        hub.poll(5.1);
        assert_eq!(hub.reconnected.len(), 1);

        // Reflash: different firmware string is never the same boot.
        let (mut hub, tx) = connected("0.7.2", 0);
        send(&tx, handshake("0.7.3"), 5.0);
        send(&tx, sensor_info(50, 0, 1), 5.1);
        hub.poll(5.1);
        assert_eq!(hub.reconnected.len(), 1);
    }

    #[test]
    fn handshake_uses_stable_mac_and_new_session_resets_sequence() {
        let source: SocketAddr = "10.0.0.4:6969".parse().unwrap();
        let next: SocketAddr = "10.0.0.5:6969".parse().unwrap();
        let info = tracker_protocol::Handshake {
            board_type: Some(1),
            imu_type: Some(1),
            mcu_type: Some(1),
            protocol_version: Some(21),
            firmware: "0.7.2".into(),
            mac: Some(MAC),
            extension: vec![],
        };
        let identity = handshake_identity(&info, source).unwrap();
        assert_eq!(identity, "01:02:03:04:05:06");
        let mut devices = BTreeMap::new();
        let mut sensors = BTreeMap::new();
        let mut history = BTreeMap::new();
        let mut reconnected = Vec::new();
        macro_rules! session {
            () => {
                Session {
                    devices: &mut devices,
                    sensors: &mut sensors,
                    history: &mut history,
                    reconnected: &mut reconnected,
                }
            };
        }
        assert!(establish_session(
            session!(),
            source,
            identity.clone(),
            info.firmware.clone(),
            true
        ));
        let device = devices.get_mut(&source).unwrap();
        device.lineage = true;
        device.last_sequence = Some(45_000);
        assert!(matches!(
            device.sequence.observe(45_000, false),
            SequenceObservation::First
        ));
        let mut sensor = Sensor::default();
        sensor.view.id = format!("{identity}/0");
        sensor.device_source = Some(source);
        sensor.hold_valid = true;
        sensor.measurement_valid = true;
        sensor.last_rotation = 1.0;
        sensor.continuity_since = Some(1.0);
        sensors.insert(sensor.view.id.clone(), sensor);
        let mut revoked = BTreeMap::new();
        revoke_session(&sensors, &mut revoked, next, &identity);
        assert!(revoked[&format!("{identity}/0")]);
        assert!(establish_session(
            session!(),
            next,
            identity.clone(),
            info.firmware.clone(),
            true
        ));
        assert!(!devices.contains_key(&source));
        assert_eq!(
            devices[&next].pending_prior,
            Some(PriorSession {
                last_sequence: 45_000,
                firmware: "0.7.2".into()
            })
        );
        assert_eq!(sensors[&format!("{identity}/0")].device_source, Some(next));
        assert!(!sensors[&format!("{identity}/0")].hold_valid);
        assert!(!sensors[&format!("{identity}/0")].measurement_valid);
        assert!(matches!(
            devices
                .get_mut(&next)
                .unwrap()
                .sequence
                .observe(45_100, false),
            SequenceObservation::First
        ));
        let mut invalid = info.clone();
        invalid.board_type = None;
        assert!(handshake_identity(&invalid, source).is_none());
    }

    #[test]
    fn drain_ignores_unacknowledged_handshake_and_retires_endpoint() {
        let (mut hub, tx) = connected("0.7.2", 0);
        tx.send(ReceiverEvent::Disrupted { error: None }).unwrap();
        send_backlog(&tx, handshake("0.7.2"), 2.0);
        hub.poll(2.0);
        assert!(!hub.devices.contains_key(&addr()));
        assert!(hub.revocations[ID]);
        assert!(hub.history.contains_key("01:02:03:04:05:06"));
    }

    #[test]
    fn held_evidence_requires_identity_status_continuity_and_transport() {
        let source: SocketAddr = "10.0.0.4:6969".parse().unwrap();
        let mut device = Device {
            name: "tracker".into(),
            firmware: "0.7.2".into(),
            mac_known: true,
            last_seen: 1.0,
            ..Default::default()
        };
        let mut sensor = Sensor {
            device_source: Some(source),
            sensor_id: 0,
            hold_valid: true,
            measurement_valid: true,
            continuity_since: Some(0.5),
            last_rotation: 0.5,
            ..Default::default()
        };
        assert!(!held_eligible(&sensor, &device, 1.1));
        device.sensor_status.insert(0, 1);
        assert!(held_eligible(&sensor, &device, 1.1));
        device.sensor_status.insert(1, 1);
        assert!(held_eligible(&sensor, &device, 1.1));
        assert!(!held_eligible(&sensor, &device, 2.6));
        device.firmware = " ".into();
        assert!(!held_eligible(&sensor, &device, 1.1));
        device.firmware = "main".into();
        assert!(held_eligible(&sensor, &device, 1.1));
        device.mac_known = false;
        assert!(!held_eligible(&sensor, &device, 1.1));
        device.mac_known = true;
        sensor.drain_pending = true;
        assert!(!held_eligible(&sensor, &device, 1.1));
        sensor.drain_pending = false;
        let mut sensors = BTreeMap::new();
        sensor.view.id = "tracker/0".into();
        sensors.insert("tracker/0".into(), sensor);
        let mut revoked = BTreeMap::new();
        revoke_source(&mut sensors, &mut revoked, source);
        assert!(!held_eligible(&sensors["tracker/0"], &device, 1.1));
        assert!(!revoked["tracker/0"]);
        sensors.get_mut("tracker/0").unwrap().hold_valid = true;
        revoke_sensor(&mut sensors, &mut revoked, source, 0);
        assert!(revoked["tracker/0"]);
    }

    #[test]
    fn wire_capture_serializes_and_enforces_packet_cap() {
        let path = std::env::temp_dir().join(format!(
            "chadfbt-wire-test-{}-{}.jsonl",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut capture = WireCapture::open(path.as_os_str()).unwrap();
        capture
            .capture(1.25, "127.0.0.1:1234".parse().unwrap(), &[0, 0xfe, 0xff])
            .unwrap();
        capture.flush().unwrap();
        let contents = std::fs::read_to_string(&path).unwrap();
        let record: serde_json::Value = serde_json::from_str(contents.trim()).unwrap();
        assert_eq!(record["bytes_hex"], "00feff");
        assert_eq!(record["at"], 1.25);
        capture.packets = WireCapture::MAX_PACKETS;
        capture
            .capture(2.0, "127.0.0.1:1234".parse().unwrap(), &[1])
            .unwrap();
        capture.flush().unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), contents);
        drop(capture);
        std::fs::remove_file(path).unwrap();
    }
    #[test]
    fn receive_rate_counts_bursts_and_resets_after_a_gap() {
        let mut sensor = Sensor::default();
        for tick in 0..100 {
            let now = tick as f64 * 0.01;
            sensor.observe_rotation_rate(now);
            sensor.observe_rotation_rate(now);
        }
        sensor.observe_rotation_rate(1.0);
        assert!((sensor.view.hz - 200.0).abs() < 0.01);
        sensor.observe_rotation_rate(3.0);
        assert_eq!(sensor.view.hz, 0.0);
    }
    #[test]
    fn firmware_up_maps_to_tracking_up() {
        let q = Quat::from_array(firmware_to_tracking([0., 0., 0., 1.]).unwrap());
        assert!((q * glam::Vec3::Z - glam::Vec3::Y).length() < 1e-5);
    }
    #[test]
    fn bad_quaternions_rejected() {
        assert!(firmware_to_tracking([0.; 4]).is_none());
        assert!(firmware_to_tracking([f32::INFINITY, 0., 0., 1.]).is_none());
    }
}
