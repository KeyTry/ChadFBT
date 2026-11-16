//! Original implementation of the public SlimeVR tracker wire protocol.
//! This crate performs no networking or coordinate/mounting transformations.
//! Quaternions are always wire-order `[x, y, z, w]`; acceleration retains firmware units.
#![forbid(unsafe_code)]

use std::{error::Error, fmt};

pub const TRACKER_PORT: u16 = 6969;
pub const MAX_DATAGRAM_SIZE: usize = 65_507;
pub const MAX_BUNDLE_PACKETS: usize = 512;
pub const SERVER_HEARTBEAT_INTERVAL_MS: u64 = 500;

#[derive(Debug, Clone, PartialEq)]
pub struct Datagram {
    pub packet_type: u32,
    pub sequence: u64,
    pub packets: Vec<Packet>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Handshake {
    pub board_type: Option<u32>,
    pub imu_type: Option<u32>,
    pub mcu_type: Option<u32>,
    pub protocol_version: Option<u32>,
    pub firmware: String,
    pub mac: Option<[u8; 6]>,
    /// Forward-compatible vendor/product metadata after MAC, uninterpreted.
    pub extension: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SensorInfo {
    pub sensor_id: u8,
    /// 0 offline, 1 OK, 2 error. Unknown statuses are retained.
    pub status: u8,
    pub imu_type: Option<u8>,
    pub config: Option<u16>,
    pub rest_calibrated: Option<bool>,
    pub position: Option<u8>,
    pub data_type: Option<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InspectionValues {
    /// Firmware sign-extends 16-bit raw samples to these 32-bit wire words.
    Integer([i32; 3]),
    Float([f32; 3]),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Packet {
    Heartbeat,
    Handshake(Handshake),
    Rotation {
        sensor_id: u8,
        data_type: u8,
        quaternion_xyzw: [f32; 4],
        calibration: u8,
    },
    Acceleration {
        sensor_id: u8,
        acceleration: [f32; 3],
    },
    RotationAcceleration {
        sensor_id: u8,
        quaternion_xyzw: [f32; 4],
        acceleration: [f32; 3],
    },
    SensorInfo(SensorInfo),
    Battery {
        voltage: Option<f32>,
        level: f32,
    },
    Signal {
        sensor_id: u8,
        rssi_dbm: i8,
    },
    Temperature {
        sensor_id: u8,
        celsius: f32,
    },
    Tap {
        sensor_id: u8,
        bits: u8,
    },
    UserAction {
        action: u8,
    },
    SensorError {
        sensor_id: u8,
        error: u8,
    },
    MagnetometerAccuracy {
        sensor_id: u8,
        accuracy: f32,
    },
    FeatureFlags(Vec<u8>),
    PingPong(u32),
    Serial(Vec<u8>),
    AckConfig {
        sensor_id: u8,
        config_type: u16,
    },
    Flex {
        sensor_id: u8,
        value: f32,
    },
    Position {
        sensor_id: u8,
        position: [f32; 3],
    },
    ProtocolChange {
        protocol: u8,
        version: u8,
    },
    RawImu {
        sensor_id: u8,
        gyro: InspectionValues,
        accel: InspectionValues,
        magnetometer: InspectionValues,
        accuracies: [u8; 3],
    },
    Unknown {
        packet_type: u32,
        payload: Vec<u8>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    DatagramTooLarge(usize),
    Truncated {
        offset: usize,
        needed: usize,
        remaining: usize,
    },
    Invalid(&'static str),
    TooManyPackets,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DatagramTooLarge(n) => write!(f, "UDP datagram too large: {n}"),
            Self::Truncated {
                offset,
                needed,
                remaining,
            } => write!(f, "truncated at {offset}: need {needed}, have {remaining}"),
            Self::Invalid(reason) => write!(f, "invalid packet: {reason}"),
            Self::TooManyPackets => write!(f, "bundle packet limit exceeded"),
        }
    }
}
impl Error for ParseError {}

struct Reader<'a> {
    data: &'a [u8],
    offset: usize,
    base: usize,
}
impl<'a> Reader<'a> {
    fn new(data: &'a [u8], base: usize) -> Self {
        Self {
            data,
            offset: 0,
            base,
        }
    }
    fn remaining(&self) -> usize {
        self.data.len() - self.offset
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], ParseError> {
        if n > self.remaining() {
            return Err(ParseError::Truncated {
                offset: self.base + self.offset,
                needed: n,
                remaining: self.remaining(),
            });
        }
        let bytes = &self.data[self.offset..self.offset + n];
        self.offset += n;
        Ok(bytes)
    }
    fn u8(&mut self) -> Result<u8, ParseError> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, ParseError> {
        Ok(u16::from_be_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn i16(&mut self) -> Result<i16, ParseError> {
        Ok(i16::from_be_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, ParseError> {
        Ok(u32::from_be_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, ParseError> {
        Ok(u64::from_be_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn float(&mut self) -> Result<f32, ParseError> {
        let value = f32::from_bits(self.u32()?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(ParseError::Invalid("nonfinite float"))
        }
    }
    fn vector(&mut self) -> Result<[f32; 3], ParseError> {
        Ok([self.float()?, self.float()?, self.float()?])
    }
    fn quaternion(&mut self) -> Result<[f32; 4], ParseError> {
        validate_quaternion([self.float()?, self.float()?, self.float()?, self.float()?])
    }
}

fn validate_quaternion(q: [f32; 4]) -> Result<[f32; 4], ParseError> {
    let norm = q.iter().map(|&x| f64::from(x).powi(2)).sum::<f64>();
    if norm < 1e-12 {
        Err(ParseError::Invalid("zero quaternion"))
    } else {
        Ok(q)
    }
}

/// Decode a complete tracker-to-server datagram. Unknown packets are preserved.
/// Invalid bundle lengths are rejected rather than silently clamped. Nested bundles
/// are rejected; packet counts and input size are bounded before allocation.
pub fn parse_datagram(data: &[u8]) -> Result<Datagram, ParseError> {
    if data.len() > MAX_DATAGRAM_SIZE {
        return Err(ParseError::DatagramTooLarge(data.len()));
    }
    let mut reader = Reader::new(data, 0);
    let packet_type = reader.u32()?;
    let sequence = reader.u64()?;
    let mut packets = Vec::new();
    if packet_type == 100 || packet_type == 101 {
        let compact = packet_type == 101;
        let mut entries = 0;
        while reader.remaining() != 0 {
            entries += 1;
            if entries > MAX_BUNDLE_PACKETS {
                return Err(ParseError::TooManyPackets);
            }
            let length = if compact {
                usize::from(reader.u8()?)
            } else {
                usize::from(reader.u16()?)
            };
            if length == 0 {
                continue;
            }
            let base = reader.offset;
            let inner = reader.take(length)?;
            let mut inner_reader = Reader::new(inner, base);
            let kind = if compact {
                u32::from(inner_reader.u8()?)
            } else {
                inner_reader.u32()?
            };
            if kind == 100 || kind == 101 {
                return Err(ParseError::Invalid("nested bundle"));
            }
            let payload_base = base + inner_reader.offset;
            let payload = inner_reader.take(inner_reader.remaining())?;
            packets.push(parse_payload(kind, payload, payload_base)?);
        }
    } else {
        packets.push(parse_payload(
            packet_type,
            reader.take(reader.remaining())?,
            12,
        )?);
    }
    Ok(Datagram {
        packet_type,
        sequence,
        packets,
    })
}

fn parse_payload(kind: u32, data: &[u8], base: usize) -> Result<Packet, ParseError> {
    let mut r = Reader::new(data, base);
    Ok(match kind {
        0 => Packet::Heartbeat,
        1 if data.is_empty() => Packet::Heartbeat,
        1 | 16 => Packet::Rotation {
            sensor_id: if kind == 1 { 0 } else { 1 },
            data_type: 1,
            quaternion_xyzw: r.quaternion()?,
            calibration: 0,
        },
        3 => {
            if data.is_empty() {
                Packet::Handshake(Handshake {
                    board_type: None,
                    imu_type: None,
                    mcu_type: None,
                    protocol_version: None,
                    firmware: String::new(),
                    mac: None,
                    extension: Vec::new(),
                })
            } else {
                let board_type = Some(r.u32()?);
                let imu_type = Some(r.u32()?);
                let mcu_type = Some(r.u32()?);
                r.take(12)?;
                let protocol_version = Some(r.u32()?);
                let len = usize::from(r.u8()?);
                let firmware_bytes = r.take(len)?;
                let firmware = String::from_utf8_lossy(
                    firmware_bytes.split(|&x| x == 0).next().unwrap_or_default(),
                )
                .into_owned();
                let mac = if r.remaining() == 0 {
                    None
                } else {
                    let bytes: [u8; 6] = r.take(6)?.try_into().unwrap();
                    if bytes == [0; 6] { None } else { Some(bytes) }
                };
                let extension = r.take(r.remaining())?.to_vec();
                Packet::Handshake(Handshake {
                    board_type,
                    imu_type,
                    mcu_type,
                    protocol_version,
                    firmware,
                    mac,
                    extension,
                })
            }
        }
        4 => {
            let acceleration = r.vector()?;
            Packet::Acceleration {
                sensor_id: if r.remaining() > 0 { r.u8()? } else { 0 },
                acceleration,
            }
        }
        10 => Packet::PingPong(r.u32()?),
        11 => {
            let len = r.u32()? as usize;
            Packet::Serial(r.take(len)?.to_vec())
        }
        12 => {
            let first = r.float()?;
            if r.remaining() == 0 {
                Packet::Battery {
                    voltage: None,
                    level: first,
                }
            } else {
                Packet::Battery {
                    voltage: Some(first),
                    level: r.float()?,
                }
            }
        }
        13 => Packet::Tap {
            sensor_id: r.u8()?,
            bits: r.u8()?,
        },
        14 => Packet::SensorError {
            sensor_id: r.u8()?,
            error: r.u8()?,
        },
        15 => {
            let sensor_id = r.u8()?;
            let status = r.u8()?;
            let imu_type = if r.remaining() > 0 {
                Some(r.u8()?)
            } else {
                None
            };
            let config = if r.remaining() > 0 {
                Some(r.u16()?)
            } else {
                None
            };
            let rest_calibrated = if r.remaining() > 0 {
                Some(r.u8()? != 0)
            } else {
                None
            };
            let position = if r.remaining() > 0 {
                Some(r.u8()?)
            } else {
                None
            };
            let data_type = if r.remaining() > 0 {
                Some(r.u8()?)
            } else {
                None
            };
            Packet::SensorInfo(SensorInfo {
                sensor_id,
                status,
                imu_type,
                config,
                rest_calibrated,
                position,
                data_type,
            })
        }
        17 => Packet::Rotation {
            sensor_id: r.u8()?,
            data_type: r.u8()?,
            quaternion_xyzw: r.quaternion()?,
            calibration: r.u8()?,
        },
        18 => Packet::MagnetometerAccuracy {
            sensor_id: r.u8()?,
            accuracy: r.float()?,
        },
        19 => Packet::Signal {
            sensor_id: r.u8()?,
            rssi_dbm: r.u8()? as i8,
        },
        20 => Packet::Temperature {
            sensor_id: r.u8()?,
            celsius: r.float()?,
        },
        21 => Packet::UserAction { action: r.u8()? },
        22 => Packet::FeatureFlags(r.take(r.remaining())?.to_vec()),
        23 => {
            let sensor_id = r.u8()?;
            let quaternion_xyzw = validate_quaternion([
                f32::from(r.i16()?) / 32768.,
                f32::from(r.i16()?) / 32768.,
                f32::from(r.i16()?) / 32768.,
                f32::from(r.i16()?) / 32768.,
            ])?;
            let acceleration = [
                f32::from(r.i16()?) / 128.,
                f32::from(r.i16()?) / 128.,
                f32::from(r.i16()?) / 128.,
            ];
            Packet::RotationAcceleration {
                sensor_id,
                quaternion_xyzw,
                acceleration,
            }
        }
        24 => Packet::AckConfig {
            sensor_id: r.u8()?,
            config_type: r.u16()?,
        },
        26 => Packet::Flex {
            sensor_id: r.u8()?,
            value: r.float()?,
        },
        27 => Packet::Position {
            sensor_id: r.u8()?,
            position: r.vector()?,
        },
        105 if data.first() == Some(&1) => {
            r.u8()?;
            let sensor_id = r.u8()?;
            let encoding = r.u8()?;
            let mut values = || -> Result<(InspectionValues, u8), ParseError> {
                let value = match encoding {
                    1 => InspectionValues::Integer([
                        r.u32()? as i32,
                        r.u32()? as i32,
                        r.u32()? as i32,
                    ]),
                    2 => InspectionValues::Float(r.vector()?),
                    _ => return Err(ParseError::Invalid("raw IMU encoding")),
                };
                Ok((value, r.u8()?))
            };
            let (gyro, a0) = values()?;
            let (accel, a1) = values()?;
            let (magnetometer, a2) = values()?;
            Packet::RawImu {
                sensor_id,
                gyro,
                accel,
                magnetometer,
                accuracies: [a0, a1, a2],
            }
        }
        200 => Packet::ProtocolChange {
            protocol: r.u8()?,
            version: r.u8()?,
        },
        _ => Packet::Unknown {
            packet_type: kind,
            payload: data.to_vec(),
        },
    })
}

/// Special legacy handshake response: it does NOT use the ordinary header.
pub fn handshake_response() -> [u8; 64] {
    let mut response = [0; 64];
    response[0] = 3;
    response[1..13].copy_from_slice(b"Hey OVR =D 5");
    response
}

fn server_packet(kind: u32, payload: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(12 + payload.len());
    bytes.extend_from_slice(&kind.to_be_bytes());
    bytes.extend_from_slice(&0u64.to_be_bytes());
    bytes.extend_from_slice(payload);
    bytes
}
pub fn heartbeat() -> Vec<u8> {
    server_packet(1, &[])
}
pub fn discovery() -> Vec<u8> {
    server_packet(0, &[])
}
/// Special sensor ACK: u32 type + id + status, without sequence number.
pub fn sensor_info_ack(sensor_id: u8, status: u8) -> [u8; 6] {
    [0, 0, 0, 15, sensor_id, status]
}
pub fn feature_flags() -> Vec<u8> {
    server_packet(22, &[1])
}
pub fn ping(ping_id: u32) -> Vec<u8> {
    server_packet(10, &ping_id.to_be_bytes())
}

/// Required session replies, in packet order. Caller sends only to the verified
/// source endpoint, and independently schedules a heartbeat every 500 ms.
pub fn session_replies(datagram: &Datagram) -> Vec<Vec<u8>> {
    datagram
        .packets
        .iter()
        .filter_map(|p| match p {
            Packet::Handshake(_) => Some(handshake_response().to_vec()),
            Packet::SensorInfo(info) => Some(sensor_info_ack(info.sensor_id, info.status).to_vec()),
            Packet::FeatureFlags(_) => Some(feature_flags()),
            _ => None,
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SequenceObservation {
    First,
    Next { missing: u64 },
    Duplicate,
    OutOfOrder,
    HandshakeReset,
}
#[derive(Debug, Default, Clone)]
pub struct SequenceTracker {
    last: Option<u64>,
}
impl SequenceTracker {
    pub fn reset(&mut self) {
        self.last = None;
    }
    /// Handshakes reset a tracker session. Exclude type-10 ping echoes: they use
    /// the server's zero sequence, not the tracker's monotonic sequence counter.
    pub fn observe(&mut self, sequence: u64, handshake: bool) -> SequenceObservation {
        if handshake {
            self.last = Some(sequence);
            return SequenceObservation::HandshakeReset;
        }
        match self.last {
            None => {
                self.last = Some(sequence);
                SequenceObservation::First
            }
            Some(last) if sequence == last => SequenceObservation::Duplicate,
            Some(last) if sequence < last => SequenceObservation::OutOfOrder,
            Some(last) => {
                self.last = Some(sequence);
                SequenceObservation::Next {
                    missing: sequence - last - 1,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn packet(kind: u32, payload: &[u8]) -> Vec<u8> {
        server_packet(kind, payload)
    }
    fn rotation() -> Vec<u8> {
        let mut p = vec![2, 1];
        for v in [0.1f32, -0.2, 0.3, 0.9] {
            p.extend(v.to_be_bytes());
        }
        p.push(3);
        packet(17, &p)
    }
    #[test]
    fn rotation_wire_order_and_all_truncations() {
        let data = rotation();
        for end in 0..data.len() {
            assert!(parse_datagram(&data[..end]).is_err(), "length {end}");
        }
        assert_eq!(
            parse_datagram(&data).unwrap().packets,
            vec![Packet::Rotation {
                sensor_id: 2,
                data_type: 1,
                quaternion_xyzw: [0.1, -0.2, 0.3, 0.9],
                calibration: 3
            }]
        );
    }
    #[test]
    fn reply_bytes_match_wire_exceptions() {
        let h = handshake_response();
        assert_eq!(&h[..13], b"\x03Hey OVR =D 5");
        assert!(h[13..].iter().all(|x| *x == 0));
        assert_eq!(heartbeat(), vec![0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(sensor_info_ack(3, 1), [0, 0, 0, 15, 3, 1]);
        assert_eq!(feature_flags()[12], 1);
    }
    #[test]
    fn firmware_handshake_with_vendor_extension() {
        let mut p = Vec::new();
        for v in [22u32, 18, 1, 0, 0, 0, 22] {
            p.extend(v.to_be_bytes());
        }
        p.push(5);
        p.extend(b"0.7.1");
        p.extend([1, 2, 3, 4, 5, 6]);
        p.extend([0, 3, b'D', b'I', b'Y']);
        let data = parse_datagram(&packet(3, &p)).unwrap();
        let Packet::Handshake(h) = &data.packets[0] else {
            panic!()
        };
        assert_eq!(h.protocol_version, Some(22));
        assert_eq!(h.firmware, "0.7.1");
        assert_eq!(h.mac, Some([1, 2, 3, 4, 5, 6]));
        assert_eq!(h.extension, [0, 3, b'D', b'I', b'Y']);
        assert_eq!(session_replies(&data), vec![handshake_response().to_vec()]);
    }
    #[test]
    fn bundles_have_no_inner_sequence() {
        let rot = rotation();
        let mut p = vec![];
        p.extend(23u16.to_be_bytes());
        p.extend(17u32.to_be_bytes());
        p.extend(&rot[12..]);
        p.extend(6u16.to_be_bytes());
        p.extend([0, 0, 0, 19, 255, 201]);
        let d = parse_datagram(&packet(100, &p)).unwrap();
        assert_eq!(d.packets.len(), 2);
        assert_eq!(
            d.packets[1],
            Packet::Signal {
                sensor_id: 255,
                rssi_dbm: -55
            }
        );
        let mut compact = vec![20, 17];
        compact.extend(&rot[12..]);
        compact.extend([3, 19, 255, 201]);
        assert_eq!(
            parse_datagram(&packet(101, &compact)).unwrap().packets,
            d.packets
        );
    }
    #[test]
    fn invalid_lengths_and_nested_bundles() {
        for payload in [
            &[0u8][..],
            &[0, 3, 0, 0, 0][..],
            &[0, 20, 0, 0, 0, 17][..],
            &[0, 4, 0, 0, 0, 100][..],
        ] {
            assert!(parse_datagram(&packet(100, payload)).is_err());
        }
        assert!(matches!(
            parse_datagram(&packet(101, &vec![0; 513])),
            Err(ParseError::TooManyPackets)
        ));
        assert!(matches!(
            parse_datagram(&vec![0; 65_508]),
            Err(ParseError::DatagramTooLarge(_))
        ));
        assert!(parse_datagram(&packet(11, &u32::MAX.to_be_bytes())).is_err());
    }
    #[test]
    fn reject_nonfinite_and_zero_rotations() {
        for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut p = rotation();
            p[14..18].copy_from_slice(&v.to_be_bytes());
            assert!(parse_datagram(&p).is_err());
        }
        let mut p = vec![0, 1];
        p.extend([0; 16]);
        p.push(0);
        assert!(parse_datagram(&packet(17, &p)).is_err());
    }
    #[test]
    fn legacy_accel_optional_sensor_and_compact_scaling() {
        let mut p = vec![];
        for x in [1f32, 2., 3.] {
            p.extend(x.to_be_bytes());
        }
        assert_eq!(
            parse_datagram(&packet(4, &p)).unwrap().packets[0],
            Packet::Acceleration {
                sensor_id: 0,
                acceleration: [1., 2., 3.]
            }
        );
        let mut p = vec![4];
        for x in [0i16, 0, 0, 32767, 128, -256, 384] {
            p.extend(x.to_be_bytes());
        }
        assert_eq!(
            parse_datagram(&packet(23, &p)).unwrap().packets[0],
            Packet::RotationAcceleration {
                sensor_id: 4,
                quaternion_xyzw: [0., 0., 0., 32767. / 32768.],
                acceleration: [1., -2., 3.]
            }
        );
    }
    #[test]
    fn sensor_optional_fields_and_truncated_config() {
        let d = parse_datagram(&packet(15, &[0, 1, 18, 0, 5, 1, 0, 0])).unwrap();
        assert_eq!(session_replies(&d), vec![vec![0, 0, 0, 15, 0, 1]]);
        assert!(parse_datagram(&packet(15, &[0, 1, 18, 0])).is_err());
        assert!(parse_datagram(&packet(15, &[0, 1])).is_ok());
    }
    #[test]
    fn raw_imu_is_typed_and_unknowns_retained() {
        let mut p = vec![1, 0, 1];
        for accuracy in [1, 2, 3] {
            for v in [-1i32, 2, 3] {
                p.extend(v.to_be_bytes());
            }
            p.push(accuracy);
        }
        let data = parse_datagram(&packet(105, &p)).unwrap();
        assert!(matches!(
            &data.packets[0],
            Packet::RawImu {
                gyro: InspectionValues::Integer([-1, 2, 3]),
                accuracies: [1, 2, 3],
                ..
            }
        ));
        for end in 0..p.len() {
            if end == 0 {
                continue;
            }
            assert!(parse_datagram(&packet(105, &p[..end])).is_err());
        }
        assert_eq!(
            parse_datagram(&packet(987, &[1, 2, 3])).unwrap().packets[0],
            Packet::Unknown {
                packet_type: 987,
                payload: vec![1, 2, 3]
            }
        );
    }
    #[test]
    fn sequence_gaps_dont_move_backwards() {
        let mut seq = SequenceTracker::default();
        assert_eq!(seq.observe(20, false), SequenceObservation::First);
        assert_eq!(
            seq.observe(23, false),
            SequenceObservation::Next { missing: 2 }
        );
        assert_eq!(seq.observe(23, false), SequenceObservation::Duplicate);
        assert_eq!(seq.observe(22, false), SequenceObservation::OutOfOrder);
        assert_eq!(
            seq.observe(24, false),
            SequenceObservation::Next { missing: 0 }
        );
        assert_eq!(seq.observe(0, true), SequenceObservation::HandshakeReset);
        assert_eq!(
            seq.observe(1, false),
            SequenceObservation::Next { missing: 0 }
        );
    }
}
