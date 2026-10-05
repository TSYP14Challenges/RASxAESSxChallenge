# Production firmware (ESP32)

Each device gets its own PlatformIO project in its own folder:

| Folder | Device | Owner |
|---|---|---|
| [`writer/`](writer/) | Writer robot | _TBD_ |
| [`executor/`](executor/) | Executor robot | _TBD_ |
| [`beacon/`](beacon/) | Droppable beacon | _TBD_ |
| [`peb/`](peb/) | PEB gateway | _TBD_ |
| [`shared/living_map_protocol/`](shared/living_map_protocol/) | Frame format helpers, used by all of the above | everyone (see below) |

The ESP8266 testbed in `experiments/` is **not** production code. Don't
build on it or copy from it.

## Project layout (per device)

```
firmware/<device>/
├── platformio.ini
├── README.md        what it does, wiring/pinout, how to flash and test
├── include/         headers for this device only
├── src/             main.cpp + this device's modules
├── lib/             private libraries for this device only (optional)
└── test/            PlatformIO unit tests (optional)
```

Start `platformio.ini` from this template:

```ini
[env:esp32]
platform = espressif32
board = esp32dev          ; change to your actual board
framework = arduino
monitor_speed = 115200
lib_extra_dirs = ../shared   ; gives you #include <lm_frame.h>
```

## Shared code rules

- **Never copy `lm_frame.h` into a device folder.** Include it from
  `shared/`, so every device speaks the same format.
- Code goes in `shared/` only if **two or more devices** use it. Everything
  else stays in the device folder.
- A change to `shared/living_map_protocol` changes the radio protocol for
  every device. Update [`docs/protocol/frame-format.md`](../docs/protocol/frame-format.md)
  in the same PR and ask the other owners to review it.
