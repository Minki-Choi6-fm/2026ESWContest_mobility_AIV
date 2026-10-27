/*
 * AIV V2V ESP-NOW 브릿지
 *
 *   Pi --USB Serial--> ESP32 --ESP-NOW 브로드캐스트--> 다른 차 ESP32 --USB Serial--> Pi
 *
 * - 26바이트 BSM 프레임을 그대로 통과만 시킴 (SYNC/END/CRC만 확인, 내용 해석 X)
 * - Serial은 데이터 전용: 디버그 출력 금지
 * - 모든 차량 ESP32가 같은 WIFI_CH 를 써야 서로 들림
 * - 보드 LED(GPIO2): 송신 또는 수신할 때마다 토글 (보드에 따라 LED 없을 수 있음)
 */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CH     1
#define SERIAL_BAUD 115200
#define FRAME_LEN   26
#define SYNC_B      0xAA
#define END_B       0x55
#define LED_PIN     2

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  uint8_t b[FRAME_LEN];
} Frame;

static QueueHandle_t rxQueue;
static uint8_t sbuf[FRAME_LEN];
static size_t sidx = 0;

static uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int j = 0; j < 8; j++) {
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
  }
  return c;
}

// SYNC(1) + BODY(22) + CRC(2, little-endian) + END(1)
static bool frameValid(const uint8_t *f) {
  if (f[0] != SYNC_B || f[FRAME_LEN - 1] != END_B) return false;
  uint16_t got = (uint16_t)f[23] | ((uint16_t)f[24] << 8);
  return crc16_ccitt(f + 1, 22) == got;
}

static void toggleLed() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ESP-NOW 수신 콜백 (Wi-Fi 태스크에서 호출) → 큐에 넣고 loop()에서 Serial로 내보냄
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != FRAME_LEN || !frameValid(data)) return;
  Frame f;
  memcpy(f.b, data, FRAME_LEN);
  xQueueSend(rxQueue, &f, 0);
}

// Pi → ESP32 시리얼 바이트를 하나씩 받아 프레임 단위로 재조립
static void feedSerialByte(uint8_t c) {
  if (sidx == 0 && c != SYNC_B) return;
  sbuf[sidx++] = c;
  if (sidx < FRAME_LEN) return;

  if (frameValid(sbuf)) {
    esp_now_send(BCAST, sbuf, FRAME_LEN);
    toggleLed();
    sidx = 0;
    return;
  }

  // 재동기화: 버퍼 안의 다음 SYNC부터 다시 시작
  size_t next = 1;
  while (next < FRAME_LEN && sbuf[next] != SYNC_B) next++;
  sidx = FRAME_LEN - next;
  memmove(sbuf, sbuf + next, sidx);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);

  rxQueue = xQueueCreate(32, sizeof(Frame));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(80);   // 20dBm

  if (esp_now_init() != ESP_OK) {
    while (true) {                 // 초기화 실패: LED 빠르게 깜빡임
      toggleLed();
      delay(100);
    }
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CH;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
}

void loop() {
  Frame f;
  while (xQueueReceive(rxQueue, &f, 0) == pdTRUE) {
    Serial.write(f.b, FRAME_LEN);
    toggleLed();
  }
  while (Serial.available()) {
    feedSerialByte((uint8_t)Serial.read());
  }
  delay(1);
}
