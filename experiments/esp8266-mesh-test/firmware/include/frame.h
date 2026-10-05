// Living Map radio frame format — shared constants and helpers.
// All multi-byte fields big-endian; last byte = XOR of every preceding byte.
#pragma once

#include <stdint.h>

namespace lm {

// ESP-NOW payload limit.
const uint8_t MAX_FRAME = 250;

const uint8_t FT_EVENT_BEACON  = 0x01;
const uint8_t FT_GO_TRIGGER    = 0x02;
const uint8_t FT_ACK           = 0x03;  // not used on the ESP-NOW segment this phase
const uint8_t FT_STATUS_UPDATE = 0x04;  // RESERVED — internal layout not specified yet

// EVENT_BEACON offsets
const uint8_t OFF_MSG_ID      = 1;  // 2 B
const uint8_t OFF_SEQ_NUM     = 3;
const uint8_t OFF_TTL         = 4;
const uint8_t OFF_TIMESTAMP   = 5;  // 4 B
const uint8_t OFF_EVENT_TYPE  = 9;
const uint8_t OFF_EVENT_VALUE = 10;
const uint8_t OFF_PRIM_COUNT  = 11;
const uint8_t EB_HEADER_LEN   = 12;  // bytes before the primitives

const uint8_t GO_TRIGGER_LEN = 5;
const uint8_t ACK_LEN        = 3;

// PLACEHOLDER event types — final list TBD.
const uint8_t EV_CONNECTIVITY = 0x00;
const uint8_t EV_PLACEHOLDER_1 = 0x01;
const uint8_t EV_PLACEHOLDER_2 = 0x02;
const uint8_t EV_PLACEHOLDER_3 = 0x03;
const uint8_t EV_PLACEHOLDER_4 = 0x04;

// Movement primitive types
const uint8_t PRIM_FORWARD = 0;  // magnitude = distance, cm
const uint8_t PRIM_TURN    = 1;  // magnitude = signed degrees (int8)

inline uint8_t xorChecksum(const uint8_t *b, uint16_t n) {
  uint8_t x = 0;
  for (uint16_t i = 0; i < n; i++) x ^= b[i];
  return x;
}

inline bool checksumOk(const uint8_t *b, uint16_t len) {
  return len >= 2 && xorChecksum(b, len - 1) == b[len - 1];
}

inline uint16_t msgId(const uint8_t *b) {
  return (uint16_t(b[OFF_MSG_ID]) << 8) | b[OFF_MSG_ID + 1];
}

inline uint16_t eventBeaconLen(uint8_t primCount) {
  return EB_HEADER_LEN + 2 * uint16_t(primCount) + 1;
}

// Checksum plus a length check for the types whose length is known.
inline bool isValidFrame(const uint8_t *b, uint16_t len) {
  if (len < 3 || len > MAX_FRAME || !checksumOk(b, len)) return false;
  switch (b[0]) {
    case FT_EVENT_BEACON:  return len > EB_HEADER_LEN && len == eventBeaconLen(b[OFF_PRIM_COUNT]);
    case FT_GO_TRIGGER:    return len == GO_TRIGGER_LEN;
    case FT_ACK:           return len == ACK_LEN;
    case FT_STATUS_UPDATE: return len >= 4;  // type + msg_id + ... + checksum; layout TBD
    default:               return false;
  }
}

}  // namespace lm
