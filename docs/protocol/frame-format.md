# Living Map radio frame format

This is the single source of truth for the frame format. These must all
match it, and change in the same PR (see `CONTRIBUTING.md`):

- `firmware/shared/living_map_protocol/src/lm_frame.h` (production firmware)
- `tools/roidome_tui/src/frame.rs` (monitor decoder)
- `tools/lmframe.py` (frame builder)

> **Version 2 (2026-10-04): the code still implements version 1.** Version 2
> adds `origin_id`, a TTL on every frame type, `target_id` in GO_TRIGGER,
> the STATUS_UPDATE layout and dedup expiry. It was written for the Phase 1
> report, and the three files above haven't been updated yet. The ESP8266
> testbed (`experiments/esp8266-mesh-test`) stays on version 1 for good.

The ESP8266 testbed keeps a frozen copy of version 1 in
`experiments/esp8266-mesh-test/firmware/include/frame.h`.

All multi-byte fields are big-endian. `checksum` is the XOR of every
preceding byte in the frame. The ESP-NOW payload limit is 250 bytes.

## Identifiers

Every node that composes frames has a unique 1-byte ID:

| ID | Node |
|---|---|
| `0x00` | Command Post |
| `0x01`–`0x7F` | Writers |
| `0x80`–`0xFE` | Executors |
| `0xFF` | all robots (GO_TRIGGER `target_id` only) |

A frame is identified by **(frame_type, origin_id, msg_id)**. `msg_id` is a
counter that is unique per origin. A beacon has no ID of its own: it is named
by the `(origin_id, msg_id)` of the EVENT_BEACON frame it stores.

## Common header

Every frame except ACK starts with the same 5 bytes:

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0 | frame_type | 1 B | |
| 1 | origin_id | 1 B | node that composed the frame |
| 2–3 | msg_id | 2 B | unique per origin |
| 4 | ttl | 1 B | hop counter, decremented per relay, dropped at 0 |

A relay changes `ttl`, so it must recompute `checksum`.

## EVENT_BEACON — `frame_type = 0x01`

Composed by a Writer and stored in the beacon it drops. A robot whose
battery is critical also broadcasts one about itself (`event_type` `0x05`). Length is
`14 + 2·N` bytes, so N ≤ 118.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–4 | header | 5 B | `origin_id` = the Writer (or the robot that is down) |
| 5 | seq_num | 1 B | drop order in this Writer's run (1 = first beacon after the PEB) |
| 6–9 | timestamp | 4 B | seconds since this Writer's GO_TRIGGER |
| 10 | event_type | 1 B | see below |
| 11 | event_value | 1 B | severity 0–255; `0` for victims and connectivity beacons |
| 12 | primitive_count | 1 B | N = number of movement primitives that follow |
| 13.. | primitives | 2 B × N | path **from the previous beacon** of the same Writer (from PEB-Inside for `seq_num` 1). Byte 0 = type (`0` = FORWARD, magnitude in cm; `1` = TURN, magnitude = signed degrees as int8). Byte 1 = magnitude |
| last | checksum | 1 B | |

A FORWARD longer than 255 cm or a TURN beyond ±127° is split into several
primitives.

| event_type | Meaning |
|---|---|
| `0x00` | none: connectivity beacon, dropped before the Writer goes out of range of the mesh |
| `0x01` | gas |
| `0x02` | fire |
| `0x03` | victim, alive |
| `0x04` | victim, dead |
| `0x05` | robot down: the robot sending it has a critical battery and has stopped there; it relays mesh traffic until its battery is empty |

How each sensor reading maps to `event_value` is set by the event-detection
hardware.

## GO_TRIGGER — `frame_type = 0x02` (8 B)

Composed by the Command Post (`origin_id` = `0x00`) and broadcast by
PEB-Outside.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–4 | header | 5 B | |
| 5 | target_id | 1 B | robot that must act, `0xFF` = all |
| 6 | mission_code | 1 B | `0x00` = explore (Writers); `0x01`–`0x05` = solve the events of that `event_type` (`0x05`: take over from the robot that is down); `0x80` = repair: replace the beacons that reported low battery or went silent (Writers) |
| 7 | checksum | 1 B | |

The mission brief needs no frame of its own. Once the robot is inside,
PEB-Inside replays the stored EVENT_BEACON frames whose `event_type` matches
`mission_code`, in drop order. For a repair, it replays the frames of the
beacons to replace, and the Writer loads an exact copy of each frame (same
identifier) into the new beacon it drops next to the old one.

## ACK — `frame_type = 0x03` (3 B)

The ESP-NOW segment doesn't use ACK. Beacons ignore it.

## STATUS_UPDATE — `frame_type = 0x04` (8 B)

Sent by a beacon or a robot about itself. A beacon's header carries the
`(origin_id, msg_id)` of its stored EVENT_BEACON, which is how the beacon is
named. A robot uses its own ID and `msg_id` counter.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–4 | header | 5 B | |
| 5 | status_code | 1 B | `0x01` = beacon low battery; `0x02` = robot heartbeat, every 10 s |
| 6 | value | 1 B | battery level, 0–100 % |
| 7 | checksum | 1 B | |

## Relay rules (beacons)

Beacons relay a frame based on whether it's valid, not on what the payload
means:

1. Drop the frame if its checksum or length is wrong.
2. Drop the frame if its `(frame_type, origin_id, msg_id)` is in the dedup
   cache.
3. Otherwise, add it to the cache, decrement `ttl`, and relay only if
   `ttl > 0`.

A cache entry **expires after 60 s**. Each frame is therefore re-flooded
across the mesh about once a minute, so a node that arrives later (the
Executor, or a PEB that was replaced) still gets multi-hop copies, while the
3 s rebroadcasts in between stay with direct neighbours. Receivers that store
data (PEB-Inside, Command Post) keep one copy per `(frame_type, origin_id,
msg_id)`, however many times it arrives.

## Decided in version 2

- **TTL for GO_TRIGGER and STATUS_UPDATE:** every frame type now has a TTL.
- **Dedup that never forgets:** replaced by the 60 s expiry above.
