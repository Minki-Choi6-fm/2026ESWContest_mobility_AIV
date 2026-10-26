#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CH      1
#define TX_PERIOD_MS 100
#define PKT_MAGIC    0xA1B2C3D4

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint32_t ms;
} Pkt;

volatile uint32_t rxCount = 0;
volatile uint32_t lost    = 0;
volatile uint32_t lastSeq = 0;
volatile bool     haveLast = false;
volatile int8_t   rxRssi  = 0;
uint8_t           rxMac[6] = {0};

uint32_t txSeq = 0;
uint32_t lastTx = 0, lastReport = 0, prevRxCount = 0;

static void handlePkt(const uint8_t *mac, const uint8_t *data, int len, int8_t rssi) {
  if (len != (int)sizeof(Pkt)) return;
  Pkt p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != PKT_MAGIC) return;
  if (haveLast && p.seq > lastSeq + 1) {
    lost += p.seq - lastSeq - 1;
  }
  lastSeq = p.seq;
  haveLast = true;
  rxCount++;
  rxRssi = rssi;
  memcpy(rxMac, mac, 6);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePkt(info->src_addr, data, len, info->rx_ctrl->rssi);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handlePkt(mac, data, len, 0);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(80);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CH;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add broadcast peer FAIL");
  }
  Serial.printf("\n=== ESP-NOW ping test ===\nMY MAC: %s  CH=%d\n",
                WiFi.macAddress().c_str(), WIFI_CH);
}

void loop() {
  uint32_t now = millis();
  if (now - lastTx >= TX_PERIOD_MS) {
    Pkt p = {PKT_MAGIC, txSeq++, now};
    esp_now_send(BCAST, (uint8_t *)&p, sizeof(p));
    lastTx = now;
  }
  if (now - lastReport >= 1000) {
    uint32_t rc = rxCount;
    uint32_t perSec = rc - prevRxCount;
    prevRxCount = rc;
    uint32_t ls = lost;
    float lossPct = (rc + ls) ? (100.0f * ls / (rc + ls)) : 0.0f;
    if (rc == 0) {
      Serial.printf("TX=%lu | RX none\n", (unsigned long)txSeq);
    } else {
      Serial.printf("TX=%lu | RX/s=%lu total=%lu lost=%lu loss=%.1f%% RSSI=%ddBm peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
                    (unsigned long)txSeq, (unsigned long)perSec,
                    (unsigned long)rc, (unsigned long)ls, lossPct, (int)rxRssi,
                    rxMac[0], rxMac[1], rxMac[2], rxMac[3], rxMac[4], rxMac[5]);
    }
    lastReport = now;
  }
}
