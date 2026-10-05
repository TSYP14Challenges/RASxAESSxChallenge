use crate::app::App;
use serialport::SerialPort;
use std::io::{ErrorKind, Read, Write};
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Local UDP port the TUI listens on for `TX <hex>` lines (see `lmframe.py --via-tui`).
/// Opening the bridge's serial port a second time resets the board (CH340 on
/// macOS toggles DTR/RTS), so injection goes through the TUI's open handle.
pub const INJECT_ADDR: &str = "127.0.0.1:47474";

fn open(port: &str, baud: u32) -> serialport::Result<Box<dyn SerialPort>> {
    serialport::new(port, baud).timeout(Duration::from_millis(200)).open()
}

/// Reads bridge lines forever, reconnecting if the port drops.
pub fn start_serial(app: Arc<Mutex<App>>, port: String, baud: u32) {
    thread::spawn(move || loop {
        let mut p = match open(&port, baud) {
            Ok(p) => p,
            Err(e) => {
                let mut app = app.lock().unwrap();
                app.connected = false;
                app.bridge_status = format!("open {port} failed: {e}");
                drop(app);
                thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        {
            let mut app = app.lock().unwrap();
            app.connected = true;
            app.tx_port = p.try_clone().ok();
            // Ask for the READY line in case the bridge booted before we attached.
            if let Some(tx) = app.tx_port.as_mut() {
                let _ = tx.write_all(b"ID\n");
            }
        }

        let mut pending: Vec<u8> = Vec::new();
        let mut buf = [0u8; 512];
        loop {
            match p.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    pending.extend_from_slice(&buf[..n]);
                    while let Some(pos) = pending.iter().position(|&b| b == b'\n') {
                        let line: Vec<u8> = pending.drain(..=pos).collect();
                        app.lock().unwrap().handle_line(&String::from_utf8_lossy(&line));
                    }
                }
                Err(e) if e.kind() == ErrorKind::TimedOut => {}
                Err(e) => {
                    let mut app = app.lock().unwrap();
                    app.connected = false;
                    app.tx_port = None;
                    app.bridge_status = format!("serial error: {e}");
                    break;
                }
            }
        }
        thread::sleep(Duration::from_secs(1));
    });
}

/// Forwards `TX <hex>` datagrams received on INJECT_ADDR to the bridge.
pub fn start_injector(app: Arc<Mutex<App>>) {
    let sock = match UdpSocket::bind(INJECT_ADDR) {
        Ok(s) => s,
        Err(e) => {
            app.lock().unwrap().bridge_status = format!("injector bind {INJECT_ADDR} failed: {e}");
            return;
        }
    };
    thread::spawn(move || {
        let mut buf = [0u8; 1024];
        while let Ok((n, _)) = sock.recv_from(&mut buf) {
            let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
            let mut app = app.lock().unwrap();
            if !line.starts_with("TX ") {
                app.bridge_status = format!("injector ignored non-TX line: {line}");
                continue;
            }
            let result = match app.tx_port.as_mut() {
                Some(port) => port.write_all(format!("{line}\n").as_bytes()),
                None => Err(std::io::Error::new(ErrorKind::NotConnected, "bridge not connected")),
            };
            match result {
                Ok(()) => {
                    app.queue_tx(&line);
                    app.bridge_status = format!("injected {line}");
                }
                Err(e) => app.bridge_status = format!("inject failed: {e}"),
            }
        }
    });
}

/// Feeds a saved bridge log (e.g. `pio device monitor | tee run.log`) through
/// the same line handler, for offline review or testing without hardware.
pub fn start_replay(app: Arc<Mutex<App>>, path: String) {
    thread::spawn(move || {
        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) => {
                app.lock().unwrap().bridge_status = format!("replay {path} failed: {e}");
                return;
            }
        };
        app.lock().unwrap().connected = true;
        for line in text.lines() {
            app.lock().unwrap().handle_line(line);
            thread::sleep(Duration::from_millis(50));
        }
    });
}
