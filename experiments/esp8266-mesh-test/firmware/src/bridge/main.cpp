// Living Map — ESP8266 bridge node (mesh testbed).
//
// Passive observer + injector. No dedup, no TTL, no validation.
//   RX: every ESP-NOW packet -> "RX <millis> <sender_mac> <hex bytes>"
//   TX: Serial line "TX <hex bytes>" -> broadcast exactly as given,
//       replies "OK TX <len> <millis>" or "ERR ..."
//   ID: Serial line "ID" -> re-prints the READY line (for a monitor that
//       attached after boot)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C" {
#include <user_interface.h>
}

static const uint8_t  WIFI_CHANNEL = 1;  // must match the beacons
static const uint8_t  MAX_FRAME    = 250;
static const uint16_t LINE_MAX     = 2 * MAX_FRAME + 64;  // hex + "TX " + optional spaces

static uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW callbacks run in the SDK task; print from loop() instead.
struct RxSlot {
  uint32_t ms;
  uint8_t mac[6];
  uint8_t len;
  uint8_t data[MAX_FRAME];
};
static const uint8_t RXQ = 16;
static RxSlot rxq[RXQ];
static volatile uint8_t rxHead = 0, rxTail = 0;
static volatile uint32_t rxOverflow = 0;

static char line[LINE_MAX + 1];
static uint16_t lineLen = 0;
static bool lineTooLong = false;

static void onRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  uint8_t next = (rxHead + 1) % RXQ;
  if (next == rxTail || len > MAX_FRAME) {
    rxOverflow++;
    return;
  }
  rxq[rxHead].ms = millis();
  memcpy(rxq[rxHead].mac, mac, 6);
  memcpy(rxq[rxHead].data, data, len);
  rxq[rxHead].len = len;
  rxHead = next;
}

static void printRx(const RxSlot &s) {
  static const char HEX_DIGITS[] = "0123456789abcdef";
  char mac[18];
  snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x",
           s.mac[0], s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5]);
  char hex[2 * MAX_FRAME + 1];
  for (uint8_t i = 0; i < s.len; i++) {
    hex[2 * i] = HEX_DIGITS[s.data[i] >> 4];
    hex[2 * i + 1] = HEX_DIGITS[s.data[i] & 0x0f];
  }
  hex[2 * s.len] = '\0';
  Serial.printf("RX %lu %s %s\n", (unsigned long)s.ms, mac, hex);
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void printReady() {
  Serial.print("READY bridge mac=");
  Serial.print(WiFi.macAddress());
  Serial.printf(" channel=%u\n", WIFI_CHANNEL);
}

static void handleLine() {
  if (lineLen == 0 && !lineTooLong) return;  // blank line
  if (!lineTooLong && strcmp(line, "ID") == 0) {
    printReady();
    return;
  }
  if (lineTooLong || strncmp(line, "TX ", 3) != 0) {
    Serial.println("ERR malformed TX line");
    return;
  }

  uint8_t buf[MAX_FRAME];
  uint16_t n = 0;
  int hi = -1;
  for (uint16_t i = 3; i < lineLen; i++) {
    char c = line[i];
    if (c == ' ') continue;  // allow "TX 01 12 34 ..."
    int v = hexVal(c);
    if (v < 0 || (hi < 0 && n >= MAX_FRAME)) {
      Serial.println("ERR malformed TX line");
      return;
    }
    if (hi < 0) {
      hi = v;
    } else {
      buf[n++] = (hi << 4) | v;
      hi = -1;
    }
  }
  if (hi >= 0 || n == 0) {
    Serial.println("ERR malformed TX line");
    return;
  }

  Serial.flush();  // keep the UART idle around the transmission (see beacon)
  uint32_t sentAt = millis();
  int rc = esp_now_send(BROADCAST, buf, n);
  delay(5);
  if (rc == 0) {
    Serial.printf("OK TX %u %lu\n", n, (unsigned long)sentAt);
  } else {
    Serial.println("ERR esp_now_send failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(WIFI_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("FATAL esp_now_init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(BROADCAST, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
  esp_now_register_recv_cb(onRecv);

  printReady();
}

void loop() {
  while (rxTail != rxHead) {
    printRx(rxq[rxTail]);
    rxTail = (rxTail + 1) % RXQ;
  }
  if (rxOverflow) {
    Serial.printf("WARN rx queue overflow, dropped=%lu\n", (unsigned long)rxOverflow);
    rxOverflow = 0;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      handleLine();
      lineLen = 0;
      lineTooLong = false;
    } else if (lineLen < LINE_MAX) {
      line[lineLen++] = c;
    } else {
      lineTooLong = true;
    }
  }
}
