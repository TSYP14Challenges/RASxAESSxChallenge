# ESP8266 ESP-NOW mesh testbed

This is an early validation rig for the Living Map beacon mesh, built on the
3 ESP8266 boards available now. **It is not the production firmware.** That
firmware targets ESP32 and is a separate task, so don't try to port this code.

```
firmware/                 PlatformIO project, two envs
  include/frame.h         frame constants + checksum/validity helpers
  src/beacon/main.cpp     beacon: EEPROM-stored frame, 3 s rebroadcast, dedup(32)+TTL relay
  src/bridge/main.cpp     bridge: RX -> serial lines, serial "TX <hex>" -> broadcast
```

Tools live at the repo root (reused by the ESP32 phase):
`../../tools/lmframe.py` (frame builder / injector / provisioner) and
`../../tools/roidome_tui/` (forked monitor TUI). Commands below are run from
the repo root. Frame spec: `../../docs/protocol/frame-format.md`.

## Setup

```sh
cd experiments/esp8266-mesh-test/firmware
pio run -e bridge -t upload --upload-port /dev/cu.usbserial-BRIDGE
pio run -e beacon -t upload --upload-port /dev/cu.usbserial-A
pio run -e beacon -t upload --upload-port /dev/cu.usbserial-B
```

On a Wemos D1 Mini, set `board = d1_mini` in `platformio.ini`. All nodes use
Wi-Fi channel 1 (`WIFI_CHANNEL` in both mains).

**Provision each beacon.** The beacon must have a unique `msg_id`. On first
boot it prints `PROVISION waiting for EVENT_BEACON frame` every 5 s. Close
any serial monitor attached to it, then run:

```sh
pip install pyserial
tools/lmframe.py event --msg-id 0x0A01 --seq 1 --ttl 3 --event 0 -p F100 -p T90 --provision /dev/cu.usbserial-A
tools/lmframe.py event --msg-id 0x0B01 --seq 2 --ttl 3 --event 1 --value 180 --provision /dev/cu.usbserial-B
#   <  PROVISIONED msg_id=0x0a01 len=17
#   <  READY beacon
```

To erase a stored frame and go back to provisioning, run
`pio run -e beacon -t erase --upload-port ...` (from `firmware/`) and then re-upload. This phase
doesn't support rewriting a frame at runtime.

**Monitor the mesh:**

```sh
cd tools/roidome_tui && cargo run --release -- /dev/cu.usbserial-BRIDGE
```

Keys: `q` quits, `c` clears. While the TUI runs, inject frames with
`tools/lmframe.py … --via-tui`. The TUI forwards them to the bridge over a
local UDP port (127.0.0.1:47474). Opening the bridge's serial port a second
time would reset the board (the CH340 driver on macOS toggles DTR/RTS on
open), so the TUI holds the port exclusively and `--send` refuses while it
runs.

Only the bridge needs the laptop. Beacons just need power, so a phone
charger or power bank is fine.

To keep a raw log for later, use `pio device monitor -p BRIDGE | tee run.log`.
To view that log in the TUI, run `cargo run -- --replay run.log`.

## Serial output reference

| Node | Line | Meaning |
|---|---|---|
| bridge | `READY bridge mac=… channel=1` | booted |
| bridge | `RX <millis> <mac> <hex>` | one per received packet, unfiltered |
| bridge | `OK TX <n> <millis>` / `ERR malformed TX line` | injection result |
| bridge | `ID` (input) → `READY …` | re-announce; the TUI sends this on connect |
| beacon | `LOADED stored frame …` / `PROVISIONED …` / `READY beacon` | boot path |
| beacon | `EVT from <mac> msg_id=… seq=… ttl=in->out event=… value=… prims=… RELAY\|DROP(ttl)` | new EVENT_BEACON |
| beacon | `FRM type=0x02\|0x04 …` | new non-event frame, relayed |
| beacon | `STAT own_tx relayed dup ttl_drop bad ignored send_fail rx_overflow heap` | every 30 s — use these for reliability numbers |

## Relay rules as implemented

1. A frame is dropped silently (counted as `bad`) if its checksum fails, its
   length doesn't match its type, or its type isn't 0x01–0x04.
2. ACK (0x03) is ignored and never relayed.
3. A frame whose `msg_id` is already in the 32-entry ring is dropped. A
   beacon's own `msg_id` goes into that ring at boot.
4. EVENT_BEACON: the beacon decrements `ttl` and **recomputes the checksum**.
   It relays the frame only if the new `ttl` is above 0. A frame that arrives
   with `ttl=0` is treated as expired.
5. GO_TRIGGER (0x02) and STATUS_UPDATE (0x04) have no TTL field in the
   current spec. They are relayed unmodified, once per node, and the dedup
   ring is the only thing that bounds them. **This spec gap needs a decision
   before the ESP32 build.**

## Validation checklist procedure

Wait until the TUI shows both beacons live and each has sent its own frame.
Then run these with `--via-tui` (commands abbreviated):

| Check | Do | Expect |
|---|---|---|
| Bridge ready | reset bridge | `READY bridge …` (TUI footer) |
| Injected frame reaches both beacons | `lmframe.py event --msg-id 0x2001 --ttl 3 --via-tui` | both beacons print `EVT … msg_id=0x2001 … ttl=3->2 RELAY` |
| Relay observed | same | the TUI shows `0x2001` with `ttl=2`, arriving from the beacon MACs |
| No re-relay of seen IDs | same, then watch | exactly one `0x2001` relay per beacon, and each beacon's `STAT dup` goes up (it hears the other's relay and drops it) |
| TTL stops relaying | `--msg-id 0x2002 --ttl 2` | beacons relay it with `ttl=1`, then the other beacon drops that copy (dedup). The TTL-only path is next |
| TTL = 1 is not relayed | `--msg-id 0x2003 --ttl 1` | beacons print `ttl=1->0 DROP(ttl)`, and the bridge sees **no** `0x2003` from any beacon |
| TTL = 0 on arrival | `--msg-id 0x2004 --ttl 0` | `ttl=0->0 DROP(ttl)` |
| Bad checksum ignored | `--msg-id 0x2005 --bad-checksum` | nothing printed, `STAT bad` +1 |
| Other types relay by validity | `go --msg-id 0x3001 --mission 7`, `status --msg-id 0x4001 --payload 0102` | `FRM type=… RELAY` on both beacons, and relays show in the TUI |
| EEPROM persistence | power-cycle a beacon | `LOADED stored frame msg_id=…`, and the TUI keeps getting its frames |
| TUI live | watch | every 3 s, each beacon row's LAST SEEN refreshes. IDS rises past 1 once a beacon relays other nodes' IDs |

With only 2 beacons in range of each other, dedup hides most multi-hop TTL
behaviour. That's why the TTL checks use injected frames with a preset low
TTL. Use the 30 s `STAT` lines (`send_fail`, `rx_overflow`, and relayed/dup
ratios) to characterise ESP8266 ESP-NOW reliability.

## Bench results — 2026-09-30

Setup: 2 beacons and 1 bridge, all within about 1 m of each other. Beacon A
and the bridge were on the laptop's USB, and beacon B was on a phone charger.

Every checklist item passed:
- The bridge booted and printed `READY`.
- Injected frames reached the beacons, and each beacon relayed a new frame
  once with `ttl` going from 3 to 2.
- IDs a beacon had already seen were not relayed again (`relayed` stayed
  flat while `dup` climbed).
- `ttl=1` became `1->0 DROP(ttl)` and the bridge saw no copy of it.
  `ttl=0` was dropped too.
- A frame with a bad checksum was dropped silently (`bad` +1).
- GO_TRIGGER and STATUS_UPDATE were relayed by both beacons.
- A malformed TX line got `ERR malformed TX line`.
- The stored frame reloaded after a reset.
- The TUI updated live.

`send_fail=0` and `rx_overflow=0` throughout. In the 40 s runs, the bridge
missed at most one relay per run.

**Finding: don't print to Serial right after `esp_now_send()`.** At first,
none of beacon A's relays reached the bridge, even though `esp_now_send`
returned 0 and the send callback reported success. A's own periodic frames
did arrive, and those weren't followed by a `Serial.printf`. A debug build
that printed after every send lost *all* of A's frames. Adding a random
delay before relaying (up to 500 ms) didn't help. The fix was to flush the
UART before sending and keep it idle for 5 ms afterwards (`broadcast()` in
the beacon, and the TX path in the bridge). After that, 100% of A's relays
arrived. The root cause wasn't isolated; it may be CPU/UART-interrupt
contention or USB-side power noise. The ESP32 port should re-check this.

## Known limitations (by design for this phase)

- **No RSSI.** The ESP8266 receive callback doesn't expose it.
- **The dedup ring holds 32 IDs.** A beacon relays its neighbour's own frame
  once; every 3 s rebroadcast after that is a `dup`. After 32 other IDs pass
  through (e.g. many injected tests), the neighbour's ID is evicted and gets
  relayed once more. This is expected, not a loop.
- **Bridge-injected frames don't show in the TUI directly.** The bridge
  doesn't hear itself, so they appear only when a beacon relays them.
