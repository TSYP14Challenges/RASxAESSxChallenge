// Living Map — ESP8266 beacon node (mesh testbed).
//
// Boot: load stored EVENT_BEACON frame from EEPROM, or wait for one on Serial
// (raw bytes) and store it. Then: rebroadcast own frame every ~3 s, and relay
// every new, valid frame heard over ESP-NOW with dedup (msg_id) + TTL rules.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>
extern "C" {
#include <user_interface.h>
}

#include "frame.h"

using namespace lm;

// ── Config ───────────────────────────────────────────────────────────────────
static const uint8_t  WIFI_CHANNEL      = 1;     // all nodes must share it
static const uint32_t REBROADCAST_MS    = 3000;
static const uint32_t REBROADCAST_JITTER_MS = 250;  // de-sync the two beacons
static const uint32_t STATS_MS          = 30000;
static const uint8_t  DEDUP_SIZE        = 32;
static const uint32_t PROVISION_GAP_MS  = 500;   // byte gap that aborts a partial frame
static const uint32_t TX_QUIET_MS       = 5;     // no Serial output this long after a send

// EEPROM layout: [0..1] magic, [2] frame length, [3..] raw frame bytes
static const uint16_t EEPROM_SIZE = 512;
static const uint8_t  MAGIC0 = 'L', MAGIC1 = 'M';
static const uint16_t EE_LEN_ADDR   = 2;
static const uint16_t EE_FRAME_ADDR = 3;

static uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ── State ────────────────────────────────────────────────────────────────────
static uint8_t ownFrame[MAX_FRAME];
static uint8_t ownLen = 0;
static uint32_t nextOwnTx = 0;
static uint32_t nextStats = 0;

static uint16_t seenIds[DEDUP_SIZE];
static uint8_t seenCount = 0, seenHead = 0;

struct Stats {
  uint32_t ownTx, relayed, dupDrop, ttlDrop, badDrop, ignored, sendFail, rxOverflow;
} stats = {};

// ESP-NOW callbacks run in the SDK task; hand frames to loop() via a small ring.
struct RxSlot {
  uint8_t mac[6];
  uint8_t len;
  uint8_t data[MAX_FRAME];
};
static const uint8_t RXQ = 8;
static RxSlot rxq[RXQ];
static volatile uint8_t rxHead = 0, rxTail = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────
static void printMac(const uint8_t *mac) {
  char s[18];
  snprintf(s, sizeof(s), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(s);
}

static bool seen(uint16_t id) {
  for (uint8_t i = 0; i < seenCount; i++)
    if (seenIds[i] == id) return true;
  return false;
}

static void remember(uint16_t id) {
  seenIds[seenHead] = id;
  seenHead = (seenHead + 1) % DEDUP_SIZE;
  if (seenCount < DEDUP_SIZE) seenCount++;
}

// Serial output starting right after esp_now_send() made the frame never reach
// other nodes on a USB-connected board (bench-tested 2026-09-30), so keep the
// UART idle around each transmission.
static void broadcast(const uint8_t *b, uint8_t len) {
  Serial.flush();
  if (esp_now_send(BROADCAST, const_cast<uint8_t *>(b), len) != 0) stats.sendFail++;
  delay(TX_QUIET_MS);
}

static void onRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  uint8_t next = (rxHead + 1) % RXQ;
  if (next == rxTail || len == 0 || len > MAX_FRAME) {
    stats.rxOverflow++;
    return;
  }
  memcpy(rxq[rxHead].mac, mac, 6);
  memcpy(rxq[rxHead].data, data, len);
  rxq[rxHead].len = len;
  rxHead = next;
}

static void onSent(uint8_t *, uint8_t status) {
  if (status != 0) stats.sendFail++;
}

// ── EEPROM ───────────────────────────────────────────────────────────────────
static bool loadStoredFrame() {
  if (EEPROM.read(0) != MAGIC0 || EEPROM.read(1) != MAGIC1) return false;
  uint8_t len = EEPROM.read(EE_LEN_ADDR);
  if (len == 0 || len > MAX_FRAME) return false;
  for (uint8_t i = 0; i < len; i++) ownFrame[i] = EEPROM.read(EE_FRAME_ADDR + i);
  if (ownFrame[0] != FT_EVENT_BEACON || !isValidFrame(ownFrame, len)) return false;
  ownLen = len;
  return true;
}

static bool storeFrame(const uint8_t *b, uint8_t len) {
  EEPROM.write(0, MAGIC0);
  EEPROM.write(1, MAGIC1);
  EEPROM.write(EE_LEN_ADDR, len);
  for (uint8_t i = 0; i < len; i++) EEPROM.write(EE_FRAME_ADDR + i, b[i]);
  return EEPROM.commit();
}

// ── Provisioning (wired, raw EVENT_BEACON bytes over Serial) ─────────────────
static void provision() {
  uint8_t buf[MAX_FRAME];
  uint16_t n = 0;
  uint32_t lastByte = 0, lastPrompt = 0;

  for (;;) {
    uint32_t now = millis();
    if (n > 0 && now - lastByte > PROVISION_GAP_MS) {
      Serial.println("ERR provision: partial frame timed out, discarded");
      n = 0;
    }
    if (n == 0 && now - lastPrompt > 5000) {
      Serial.println("PROVISION waiting for EVENT_BEACON frame (raw bytes)");
      lastPrompt = now;
    }

    while (Serial.available()) {
      uint8_t c = Serial.read();
      lastByte = millis();
      if (n == 0 && c != FT_EVENT_BEACON) continue;  // resync on frame_type
      buf[n++] = c;
      if (n < EB_HEADER_LEN) continue;

      uint16_t want = eventBeaconLen(buf[OFF_PRIM_COUNT]);
      if (want > MAX_FRAME) {
        Serial.println("ERR provision: primitive_count too large");
        n = 0;
      } else if (n == want) {
        if (!checksumOk(buf, n)) {
          Serial.println("ERR provision: bad checksum");
          n = 0;
        } else if (!storeFrame(buf, n)) {
          Serial.println("ERR provision: EEPROM commit failed");
          n = 0;
        } else {
          memcpy(ownFrame, buf, n);
          ownLen = n;
          Serial.printf("PROVISIONED msg_id=0x%04x len=%u\n", msgId(ownFrame), ownLen);
          return;
        }
      }
    }
    delay(1);  // feed the watchdog / SDK
  }
}

// ── Mesh handling ────────────────────────────────────────────────────────────
static void handleFrame(RxSlot &s) {
  uint8_t *b = s.data;
  uint8_t len = s.len;

  if (!isValidFrame(b, len)) { stats.badDrop++; return; }
  if (b[0] == FT_ACK) { stats.ignored++; return; }

  uint16_t id = msgId(b);
  if (seen(id)) { stats.dupDrop++; return; }
  remember(id);

  bool relay = true;
  uint8_t ttlIn = 0, ttlOut = 0;
  if (b[0] == FT_EVENT_BEACON) {
    ttlIn = b[OFF_TTL];
    ttlOut = ttlIn > 0 ? ttlIn - 1 : 0;  // TTL 0 on arrival = already expired
    b[OFF_TTL] = ttlOut;
    b[len - 1] = xorChecksum(b, len - 1);  // TTL changed -> recompute
    relay = ttlOut > 0;
  }
  // GO_TRIGGER / STATUS_UPDATE have no TTL field in the current spec: they are
  // relayed once per node, bounded by the dedup cache alone.

  if (relay) {
    broadcast(b, len);
    stats.relayed++;
  } else {
    stats.ttlDrop++;
  }

  if (b[0] == FT_EVENT_BEACON) {
    Serial.print("EVT from ");
    printMac(s.mac);
    Serial.printf(" msg_id=0x%04x seq=%u ttl=%u->%u event=0x%02x value=%u prims=%u %s\n",
                  id, b[OFF_SEQ_NUM], ttlIn, ttlOut, b[OFF_EVENT_TYPE],
                  b[OFF_EVENT_VALUE], b[OFF_PRIM_COUNT], relay ? "RELAY" : "DROP(ttl)");
  } else {
    Serial.printf("FRM type=0x%02x msg_id=0x%04x len=%u RELAY\n", b[0], id, len);
  }
}

static void printStats() {
  Serial.printf("STAT own_tx=%u relayed=%u dup=%u ttl_drop=%u bad=%u ignored=%u "
                "send_fail=%u rx_overflow=%u heap=%u\n",
                stats.ownTx, stats.relayed, stats.dupDrop, stats.ttlDrop, stats.badDrop,
                stats.ignored, stats.sendFail, stats.rxOverflow, ESP.getFreeHeap());
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(WIFI_CHANNEL);

  Serial.print("BOOT beacon mac=");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != 0) {
    Serial.println("FATAL esp_now_init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(BROADCAST, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
  esp_now_register_send_cb(onSent);

  EEPROM.begin(EEPROM_SIZE);
  if (loadStoredFrame()) {
    Serial.printf("LOADED stored frame msg_id=0x%04x len=%u\n", msgId(ownFrame), ownLen);
  } else {
    provision();
  }

  // Our own msg_id must never be relayed back to us as "new".
  remember(msgId(ownFrame));
  randomSeed(ESP.getChipId() ^ micros());
  nextOwnTx = millis() + random(REBROADCAST_JITTER_MS);
  nextStats = millis() + STATS_MS;

  // Register RX last so nothing is queued while provisioning.
  esp_now_register_recv_cb(onRecv);
  Serial.println("READY beacon");
}

void loop() {
  while (rxTail != rxHead) {
    handleFrame(rxq[rxTail]);
    rxTail = (rxTail + 1) % RXQ;
  }

  uint32_t now = millis();
  if ((int32_t)(now - nextOwnTx) >= 0) {
    broadcast(ownFrame, ownLen);
    stats.ownTx++;
    nextOwnTx = now + REBROADCAST_MS + random(REBROADCAST_JITTER_MS);
  }
  if ((int32_t)(now - nextStats) >= 0) {
    printStats();
    nextStats = now + STATS_MS;
  }
}
