/* [S12] 코어 3.x 호환: 수신/송신 콜백 시그니처를 ESP_ARDUINO_VERSION_MAJOR 로 분기 (컴파일만 확인, 업로드 안 함).
 * 원인: 코어 3.x 에서 esp_now_register_send_cb 의 콜백 인자가 (const uint8_t* mac) -> (const wifi_tx_info_t*) 로 바뀜.
 */
/*
 * ESP32 <-> RPi(UART/USB-Serial) <-> ESP-NOW 브릿지 펌웨어
 *
 * 역할: RPi가 USB-Serial로 보내주는 26바이트 BSM 프레임(bsm_protocol.py와
 * 동일한 포맷, SYNC=0xAA ~ END=0x55)을 그대로 ESP-NOW 브로드캐스트로 쏘고,
 * 반대로 ESP-NOW로 받은 26바이트 프레임은 그대로 Serial로 RPi에 올려줍니다.
 *
 * 이 펌웨어는 프레임 "내용"을 전혀 해석하지 않습니다(속도/위치 같은 필드는
 * RPi 쪽 Python(bsm_protocol.py)에서만 다룹니다). 여기서는 그냥 26바이트
 * 뭉치를 시리얼<->ESP-NOW 사이로 그대로 통과시키는 역할만 합니다.
 *
 * arduino-cli 사용 예:
 *   arduino-cli compile --fqbn esp32:esp32:esp32 esp32_espnow_bridge
 *   arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 esp32_espnow_bridge
 *
 * 주의: Arduino-ESP32 코어 버전에 따라 esp_now_register_recv_cb()의 콜백
 * 시그니처가 다릅니다. 이 코드는 코어 2.x 기준(const uint8_t*, const uint8_t*,
 * int)입니다. 코어 3.x(esp_now_recv_info_t*)를 쓰신다면 onDataRecv 시그니처만
 * 바꿔주시면 됩니다. (컴파일 에러 나면 저한테 에러 메시지 보여주세요.)
 */

#include <WiFi.h>
#include <esp_now.h>

static const uint8_t SYNC_BYTE = 0xAA;
static const uint8_t END_BYTE = 0x55;
static const size_t FRAME_SIZE = 26;

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// RPi -> ESP32로 들어오는 시리얼 바이트를 모아서 프레임 경계를 다시 찾는 버퍼
static uint8_t rxBuf[128];
static size_t rxLen = 0;

// ---- ESP-NOW로 프레임을 받았을 때: 그대로 Serial로 RPi에 전달 ----
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {   // 코어 3.x
#else
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {           // 코어 2.x
#endif
  if (len == (int)FRAME_SIZE) {
    Serial.write(data, FRAME_SIZE);
  }
  // 길이가 안 맞는 건 우리 BSM 프레임이 아니니 무시
}

// ---- ESP-NOW 송신 결과 콜백 (디버깅용, 없어도 동작엔 지장 없음) ----
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {        // 코어 3.x
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {           // 코어 2.x
#endif
  // 필요하면 여기서 Serial.println으로 성공/실패 로그를 남길 수 있지만,
  // Serial을 프레임 통로로도 쓰고 있어서 평소엔 조용히 둡니다.
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // AP에 붙지 않고 ESP-NOW 전용으로 사용

  if (esp_now_init() != ESP_OK) {
    // 초기화 실패 시 LED나 시리얼로 표시하고 재시작
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = 0;   // 0 = 현재 WiFi 채널 그대로 사용
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(BROADCAST_MAC)) {
    esp_now_add_peer(&peerInfo);
  }
}

// ---- Serial(RPi->ESP32)에서 26바이트 프레임을 찾아내는 함수 ----
// 찾으면 true를 반환하고 outFrame에 채워줌. 버퍼 앞부분(찾은 프레임 + 그 이전
// 쓰레기 바이트)은 소비해서 지움.
bool extractFrame(uint8_t *outFrame) {
  while (true) {
    // SYNC 바이트 위치 탐색
    size_t syncIdx = 0;
    bool found = false;
    for (; syncIdx < rxLen; syncIdx++) {
      if (rxBuf[syncIdx] == SYNC_BYTE) {
        found = true;
        break;
      }
    }

    if (!found) {
      rxLen = 0;  // SYNC조차 없으면 버퍼 전체가 쓰레기
      return false;
    }

    // SYNC 앞의 쓰레기 바이트 버림
    if (syncIdx > 0) {
      memmove(rxBuf, rxBuf + syncIdx, rxLen - syncIdx);
      rxLen -= syncIdx;
    }

    if (rxLen < FRAME_SIZE) {
      return false;  // 아직 26바이트 안 모임
    }

    if (rxBuf[FRAME_SIZE - 1] == END_BYTE) {
      memcpy(outFrame, rxBuf, FRAME_SIZE);
      memmove(rxBuf, rxBuf + FRAME_SIZE, rxLen - FRAME_SIZE);
      rxLen -= FRAME_SIZE;
      return true;
    }

    // END 위치가 안 맞으면 우연히 낀 0xAA였다는 뜻 -> 한 바이트 버리고 재탐색
    memmove(rxBuf, rxBuf + 1, rxLen - 1);
    rxLen -= 1;
  }
}

void loop() {
  // Serial(RPi)에서 들어오는 바이트를 버퍼에 누적
  while (Serial.available() > 0 && rxLen < sizeof(rxBuf)) {
    rxBuf[rxLen++] = (uint8_t)Serial.read();
  }

  uint8_t frame[FRAME_SIZE];
  if (extractFrame(frame)) {
    esp_now_send(BROADCAST_MAC, frame, FRAME_SIZE);
  }
}
