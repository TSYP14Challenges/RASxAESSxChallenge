# Board registry

Identify boards by **MAC address**. Serial port names (`/dev/cu.usbserial-…`,
`COM5`) change with the USB socket. Put a physical label on every board and
add a row here when you flash or re-purpose one.

| Label | MAC | Chip / USB-serial | Role | Firmware | Notes |
|---|---|---|---|---|---|
| A | `f8:b3:b7:81:7f:43` | ESP8266EX / CH340, 4 MB | Beacon (testbed) | `experiments/esp8266-mesh-test` beacon | msg_id `0x0A01` |
| B | `24:d7:eb:ee:f9:d9` | ESP8266EX / CH340, 4 MB | Beacon (testbed) | `experiments/esp8266-mesh-test` beacon | msg_id `0x0B01` |
| BR | `24:d7:eb:ee:f5:bd` | ESP8266EX / CH340, 4 MB | Bridge (testbed) | `experiments/esp8266-mesh-test` bridge | |

To read a board's MAC without flashing it:
`pio pkg exec -p tool-esptoolpy -- esptool.py --port <port> read_mac`
