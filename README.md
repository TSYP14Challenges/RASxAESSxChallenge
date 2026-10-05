# TSYP14 — The Living Map (IEEE RAS × AESS Tunisia)

Two robots, a Writer and an Executor, explore a GPS-denied, disconnected
zone. They drop small radio beacons to record what they find. A gateway (the
PEB) relays that data out to a Command Post.

**New to the repo? Read [`CONTRIBUTING.md`](CONTRIBUTING.md) first.** It says
where your code goes and how to get it into `main`.

## Repository layout

```
├── firmware/                  production firmware (ESP32), one PlatformIO project per device
│   ├── writer/                Writer robot
│   ├── executor/              Executor robot
│   ├── beacon/                droppable beacon
│   ├── peb/                   PEB gateway
│   └── shared/                code shared by 2+ devices (living_map_protocol: frame format)
├── tools/
│   ├── roidome_tui/           live ESP-NOW mesh monitor (Rust), fork of RoidOME
│   └── lmframe.py             build / inject / provision frames
├── experiments/
│   └── esp8266-mesh-test/     ESP8266 mesh testbed — validated relay/dedup/TTL logic (not production)
├── computer_vision/           camera → event detection (owner: Bacha)
├── factory_environment_simulation/   Gazebo Harmonic demo: Writer/Executor beacon scenario in a factory (run: bash run.sh demo)
├── mechanical-design/
│   └── 3D/
│       ├── design-files/      editable CAD sources
│       └── print-files/       print-ready exports (.stl, .3mf, .gcode)
├── docs/
│   ├── protocol/frame-format.md   byte-exact radio frame spec (single source of truth)
│   └── hardware/boards.md         board registry: MAC → role
└── .github/                   CI (builds all firmware, tests tools) + PR template
```

## Status

| Part | State |
|---|---|
| Radio frame format | Specified. Open question: TTL for GO_TRIGGER and STATUS_UPDATE |
| ESP-NOW beacon mesh | Validated on 3 × ESP8266, see the [bench results](experiments/esp8266-mesh-test/README.md#bench-results--2026-09-30) |
| Computer vision | Not started |
| Writer / Executor / Beacon (ESP32) / PEB | Not started |
