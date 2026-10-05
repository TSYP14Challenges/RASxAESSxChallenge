use crate::frame::{self, DecodeError, Frame, RxLine};

use std::collections::{BTreeMap, HashSet, VecDeque};
use std::time::Instant;

const LOG_CAPACITY: usize = 200;

pub struct App {
    pub running: bool,
    pub connected: bool,
    pub port_name: String,
    pub bridge_status: String, // last non-RX line from the bridge (READY / OK TX / ERR ...)
    pub message_count: u32,
    pub bad_count: u32,
    pub devices: BTreeMap<String, DeviceStatus>, // sorted -> stable row order
    pub log: VecDeque<LogEntry>,
    pub tx_port: Option<Box<dyn serialport::SerialPort>>, // write side of the bridge port
    pub bridge: Option<BridgeStatus>,
    pending_tx: VecDeque<Vec<u8>>, // injected frames awaiting the bridge's OK/ERR
}

/// The bridge never hears its own broadcasts, so its row is built from its
/// READY line and the frames injected through it.
pub struct BridgeStatus {
    pub mac: String,
    pub ready_at: Instant,
    pub last_tx: Option<Instant>,
    pub tx_count: u32,
    pub errors: u32,
    pub distinct_ids: HashSet<u16>,
    pub last: Option<Frame>,
}

pub struct DeviceStatus {
    pub last_seen: Instant,
    pub bridge_millis: u64,
    pub frames: u32,
    pub bad: u32,
    pub distinct_ids: HashSet<u16>,
    pub last: Option<Frame>,
}

pub struct LogEntry {
    pub tx: bool, // sent by the bridge rather than received
    pub bridge_millis: u64,
    pub mac: String,
    pub decoded: Result<Frame, DecodeError>,
    pub hex: String,
}

impl App {
    pub fn new(port_name: String) -> Self {
        App {
            running: true,
            connected: false,
            port_name,
            bridge_status: String::from("—"),
            message_count: 0,
            bad_count: 0,
            devices: BTreeMap::new(),
            log: VecDeque::with_capacity(LOG_CAPACITY),
            tx_port: None,
            bridge: None,
            pending_tx: VecDeque::new(),
        }
    }

    pub fn handle_line(&mut self, line: &str) {
        let line = line.trim();
        if line.is_empty() {
            return;
        }
        if let Some(rx) = frame::parse_rx_line(line) {
            self.record_rx(rx);
            return;
        }
        // Skip the ESP8266 boot ROM's 74880-baud noise; control bytes would corrupt the terminal.
        if !line.chars().all(|c| c.is_ascii_graphic() || c == ' ') {
            return;
        }
        self.bridge_status = line.to_string();

        if let Some(rest) = line.strip_prefix("READY bridge ") {
            let mac = rest
                .split(' ')
                .find_map(|kv| kv.strip_prefix("mac="))
                .unwrap_or("?")
                .to_lowercase();
            let b = self.bridge.get_or_insert_with(|| BridgeStatus {
                mac: mac.clone(),
                ready_at: Instant::now(),
                last_tx: None,
                tx_count: 0,
                errors: 0,
                distinct_ids: HashSet::new(),
                last: None,
            });
            b.mac = mac;
            b.ready_at = Instant::now();
        } else if let Some(rest) = line.strip_prefix("OK TX ") {
            let millis = rest.split(' ').nth(1).and_then(|m| m.parse().ok()).unwrap_or(0);
            if let Some(bytes) = self.pending_tx.pop_front() {
                self.record_tx(bytes, millis);
            }
        } else if line.starts_with("ERR") {
            self.pending_tx.pop_front();
            if let Some(b) = self.bridge.as_mut() {
                b.errors += 1;
            }
        }
    }

    /// Called when an injected `TX <hex>` line has been written to the bridge.
    pub fn queue_tx(&mut self, line: &str) {
        let hex = line.strip_prefix("TX ").unwrap_or("");
        self.pending_tx.push_back(frame::decode_hex(hex).unwrap_or_default());
    }

    fn record_tx(&mut self, bytes: Vec<u8>, bridge_millis: u64) {
        let decoded = frame::decode(&bytes);
        let mac = match self.bridge.as_mut() {
            Some(b) => {
                b.last_tx = Some(Instant::now());
                b.tx_count += 1;
                if let Ok(f) = &decoded {
                    if let Some(id) = f.msg_id() {
                        b.distinct_ids.insert(id);
                    }
                    b.last = Some(f.clone());
                }
                b.mac.clone()
            }
            None => String::from("bridge"),
        };
        self.push_log(LogEntry { tx: true, bridge_millis, mac, decoded, hex: frame::encode_hex(&bytes) });
    }

    fn push_log(&mut self, entry: LogEntry) {
        if self.log.len() == LOG_CAPACITY {
            self.log.pop_back();
        }
        self.log.push_front(entry);
    }

    fn record_rx(&mut self, rx: RxLine) {
        let decoded = frame::decode(&rx.bytes);
        self.message_count += 1;

        let dev = self.devices.entry(rx.mac.clone()).or_insert_with(|| DeviceStatus {
            last_seen: Instant::now(),
            bridge_millis: 0,
            frames: 0,
            bad: 0,
            distinct_ids: HashSet::new(),
            last: None,
        });
        dev.last_seen = Instant::now();
        dev.bridge_millis = rx.millis;
        dev.frames += 1;
        match &decoded {
            Ok(f) => {
                if let Some(id) = f.msg_id() {
                    dev.distinct_ids.insert(id);
                }
                dev.last = Some(f.clone());
            }
            Err(_) => {
                dev.bad += 1;
                self.bad_count += 1;
            }
        }

        self.push_log(LogEntry {
            tx: false,
            bridge_millis: rx.millis,
            mac: rx.mac,
            decoded,
            hex: frame::encode_hex(&rx.bytes),
        });
    }

    pub fn clear(&mut self) {
        self.devices.clear();
        self.log.clear();
        if let Some(b) = self.bridge.as_mut() {
            b.last_tx = None;
            b.tx_count = 0;
            b.errors = 0;
            b.distinct_ids.clear();
            b.last = None;
        }
        self.message_count = 0;
        self.bad_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_bridge_from_ready_and_injections() {
        let mut app = App::new("test".into());
        app.handle_line("READY bridge mac=24:D7:EB:EE:F5:BD channel=1");
        app.queue_tx("TX 0230010734");
        app.handle_line("OK TX 5 1234");
        app.queue_tx("TX 0g");
        app.handle_line("ERR malformed TX line");

        let b = app.bridge.as_ref().unwrap();
        assert_eq!(b.mac, "24:d7:eb:ee:f5:bd");
        assert_eq!((b.tx_count, b.errors), (1, 1));
        assert_eq!(b.last.as_ref().and_then(|f| f.msg_id()), Some(0x3001));
        assert!(app.devices.is_empty()); // its own TX never counts as a heard node
        assert!(app.log.front().unwrap().tx);
        assert_eq!(app.log.front().unwrap().bridge_millis, 1234);
    }
}
