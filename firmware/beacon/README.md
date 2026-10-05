# Beacon firmware

**Owner:** _TBD — put your name here when you start_
**Status:** not started

The droppable radio beacon: stores one EVENT_BEACON frame and relays mesh traffic (dedup + TTL). The ESP8266 testbed in experiments/esp8266-mesh-test is the validated reference for its relay logic.

## What goes in this folder

A PlatformIO project for this device only. See [`../README.md`](../README.md)
for the layout, the `platformio.ini` template and the shared-code rules.

## To fill in when you start

- Board and pinout / wiring
- How to build, flash and test
- Which frame types it sends and receives (see [`docs/protocol/frame-format.md`](../../docs/protocol/frame-format.md))
