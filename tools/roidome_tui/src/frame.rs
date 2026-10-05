// Living Map frame decoder. Big-endian fields; last byte = XOR of all preceding bytes.

pub const FT_EVENT_BEACON: u8 = 0x01;
pub const FT_GO_TRIGGER: u8 = 0x02;
pub const FT_ACK: u8 = 0x03;
pub const FT_STATUS_UPDATE: u8 = 0x04; // reserved, layout TBD

#[derive(Clone, Debug, PartialEq)]
pub enum Primitive {
    Forward(u8), // cm
    Turn(i8),    // signed degrees
    Unknown(u8, u8),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Frame {
    EventBeacon {
        msg_id: u16,
        seq_num: u8,
        ttl: u8,
        timestamp: u32,
        event_type: u8,
        event_value: u8,
        primitives: Vec<Primitive>,
    },
    GoTrigger { msg_id: u16, mission_code: u8 },
    Ack,
    StatusUpdate { msg_id: u16, payload_len: usize },
    Unknown { frame_type: u8 },
}

#[derive(Clone, Debug, PartialEq)]
pub enum DecodeError {
    TooShort,
    BadChecksum,
    BadLength { expected: usize, got: usize },
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            DecodeError::TooShort => write!(f, "too short"),
            DecodeError::BadChecksum => write!(f, "bad checksum"),
            DecodeError::BadLength { expected, got } => {
                write!(f, "bad length (expected {expected}, got {got})")
            }
        }
    }
}

impl Frame {
    pub fn msg_id(&self) -> Option<u16> {
        match self {
            Frame::EventBeacon { msg_id, .. }
            | Frame::GoTrigger { msg_id, .. }
            | Frame::StatusUpdate { msg_id, .. } => Some(*msg_id),
            _ => None,
        }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Frame::EventBeacon { .. } => "EVENT_BEACON",
            Frame::GoTrigger { .. } => "GO_TRIGGER",
            Frame::Ack => "ACK",
            Frame::StatusUpdate { .. } => "STATUS_UPDATE",
            Frame::Unknown { .. } => "UNKNOWN",
        }
    }
}

/// event_type names from docs/protocol/frame-format.md.
pub fn event_name(event_type: u8) -> &'static str {
    match event_type {
        0x00 => "connectivity",
        0x01 => "gas",
        0x02 => "fire",
        0x03 => "victim, alive",
        0x04 => "victim, dead",
        0x05 => "robot down",
        _ => "unknown",
    }
}

pub fn xor_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, b| acc ^ b)
}

fn expect_len(bytes: &[u8], expected: usize) -> Result<(), DecodeError> {
    if bytes.len() == expected {
        Ok(())
    } else {
        Err(DecodeError::BadLength { expected, got: bytes.len() })
    }
}

pub fn decode(bytes: &[u8]) -> Result<Frame, DecodeError> {
    if bytes.len() < 2 {
        return Err(DecodeError::TooShort);
    }
    let (body, sum) = bytes.split_at(bytes.len() - 1);
    if xor_checksum(body) != sum[0] {
        return Err(DecodeError::BadChecksum);
    }
    let msg_id = || u16::from_be_bytes([bytes[1], bytes[2]]);

    match bytes[0] {
        FT_EVENT_BEACON => {
            if bytes.len() < 13 {
                return Err(DecodeError::TooShort);
            }
            let n = bytes[11] as usize;
            expect_len(bytes, 12 + 2 * n + 1)?;
            let primitives = bytes[12..12 + 2 * n]
                .chunks_exact(2)
                .map(|p| match p[0] {
                    0 => Primitive::Forward(p[1]),
                    1 => Primitive::Turn(p[1] as i8),
                    t => Primitive::Unknown(t, p[1]),
                })
                .collect();
            Ok(Frame::EventBeacon {
                msg_id: msg_id(),
                seq_num: bytes[3],
                ttl: bytes[4],
                timestamp: u32::from_be_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]),
                event_type: bytes[9],
                event_value: bytes[10],
                primitives,
            })
        }
        FT_GO_TRIGGER => {
            expect_len(bytes, 5)?;
            Ok(Frame::GoTrigger { msg_id: msg_id(), mission_code: bytes[3] })
        }
        FT_ACK => {
            expect_len(bytes, 3)?;
            Ok(Frame::Ack)
        }
        FT_STATUS_UPDATE => {
            if bytes.len() < 4 {
                return Err(DecodeError::TooShort);
            }
            Ok(Frame::StatusUpdate { msg_id: msg_id(), payload_len: bytes.len() - 4 })
        }
        t => Ok(Frame::Unknown { frame_type: t }),
    }
}

/// A bridge line: `RX <millis> <mac> <hex bytes>`.
#[derive(Debug, PartialEq)]
pub struct RxLine {
    pub millis: u64,
    pub mac: String,
    pub bytes: Vec<u8>,
}

pub fn parse_rx_line(line: &str) -> Option<RxLine> {
    let mut parts = line.trim().splitn(4, ' ');
    if parts.next()? != "RX" {
        return None;
    }
    let millis = parts.next()?.parse().ok()?;
    let mac = parts.next()?.to_lowercase();
    let bytes = decode_hex(parts.next().unwrap_or(""))?;
    Some(RxLine { millis, mac, bytes })
}

pub fn decode_hex(s: &str) -> Option<Vec<u8>> {
    let digits: Vec<u8> = s.bytes().filter(|c| !c.is_ascii_whitespace()).collect();
    if digits.len() % 2 != 0 {
        return None;
    }
    digits
        .chunks_exact(2)
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).ok()?, 16).ok())
        .collect()
}

pub fn encode_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with_checksum(mut b: Vec<u8>) -> Vec<u8> {
        b.push(xor_checksum(&b));
        b
    }

    #[test]
    fn decodes_event_beacon() {
        let raw = with_checksum(vec![
            0x01, 0x12, 0x34, 7, 3, 0, 0, 0x01, 0x2c, 0x02, 200, 2, 0, 100, 1, 0xa6,
        ]);
        let f = decode(&raw).unwrap();
        assert_eq!(
            f,
            Frame::EventBeacon {
                msg_id: 0x1234,
                seq_num: 7,
                ttl: 3,
                timestamp: 300,
                event_type: 2,
                event_value: 200,
                primitives: vec![Primitive::Forward(100), Primitive::Turn(-90)],
            }
        );
    }

    #[test]
    fn rejects_bad_checksum_and_length() {
        let mut raw = with_checksum(vec![0x01, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode(&raw).is_ok());
        raw[4] ^= 0xff;
        assert_eq!(decode(&raw), Err(DecodeError::BadChecksum));
        let short_prims = with_checksum(vec![0x01, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 3, 0, 5]);
        assert!(matches!(decode(&short_prims), Err(DecodeError::BadLength { .. })));
    }

    #[test]
    fn decodes_other_types() {
        let go = with_checksum(vec![0x02, 0xab, 0xcd, 9]);
        assert_eq!(decode(&go).unwrap(), Frame::GoTrigger { msg_id: 0xabcd, mission_code: 9 });
        let st = with_checksum(vec![0x04, 0, 5, 1, 2, 3]);
        assert_eq!(decode(&st).unwrap(), Frame::StatusUpdate { msg_id: 5, payload_len: 3 });
    }

    #[test]
    fn parses_bridge_line() {
        let l = parse_rx_line("RX 12345 AA:bb:cc:dd:ee:ff 0201020300\r\n").unwrap();
        assert_eq!(l.millis, 12345);
        assert_eq!(l.mac, "aa:bb:cc:dd:ee:ff");
        assert_eq!(l.bytes, vec![2, 1, 2, 3, 0]);
        assert!(parse_rx_line("READY bridge").is_none());
        assert!(parse_rx_line("RX 1 aa:bb 0g").is_none());
    }
}
