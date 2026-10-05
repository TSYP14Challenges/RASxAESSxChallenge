# Fork notes

Forked from `tools/roidome_tui` in https://github.com/3ab3abb/RoidOME
(commit 63e59f23dfbdd9730faa2a91657a50cf06f3d084, MIT — see LICENSE).

Changes for the Living Map ESP-NOW testbed:

- `src/mqtt.rs` → `src/serial.rs`: input is the bridge node's serial port
  (`RX <millis> <mac> <hex>` lines) instead of MQTT topics.
- `src/frame.rs` (new): decoder for the Living Map frame format.
- `src/app.rs`: state is per-sender-MAC (last seen, last msg_id/seq/ttl/event,
  distinct msg_id count) plus a recent-frame log.
- `src/ui.rs`: same palette, header/body/footer layout and block style; the
  sensor gauges / motion / camera panels are replaced by a nodes table, a
  last-frame detail panel and a frame log.
- Injection: the TUI listens on UDP 127.0.0.1:47474 and forwards `TX <hex>`
  lines to the bridge (`tools/lmframe.py --via-tui`). A second opener of the
  port would reset the board.
- The bridge gets a pinned NODES row (MAC from its `READY` line, requested
  with `ID` on connect; activity = frames injected through it), since it
  never hears its own broadcasts. The frame log marks entries TX/RX.
- `--replay <file>` plays back a saved bridge log.
- Dropped deps no longer needed: rumqttc, tokio, serde, ratatui-image, image.
