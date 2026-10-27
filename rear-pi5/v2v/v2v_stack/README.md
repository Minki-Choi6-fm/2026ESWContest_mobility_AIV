# V2V BSM 통신 테스트 코드

`V2V_통신_프로토콜_설계.md` 문서의 BSM 프레임을 실제로 구현한 코드입니다.
RPi 내장 Wi-Fi(UDP 브로드캐스트)와 ESP32(ESP-NOW) 두 가지 전송 방식을 모두
지원하며, `--transport` 옵션 하나로 전환됩니다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| `bsm_protocol.py` | BSM 프레임 인코딩/디코딩, CRC-16 체크 (전송 방식과 무관, 안 바뀜) |
| `transport.py` | 전송 계층. `UDPBroadcastTransport`(RPi Wi-Fi)와 `EspNowTransport`(ESP32 USB-Serial) 둘 다 구현됨 |
| `freshness.py` | 차량별 데이터 신선도(300ms 기준 STALE 판단) 추적 |
| `v2v_node.py` | 실행 스크립트 — 이걸 각 라즈베리파이에서 돌리면 됨 |
| `esp32_espnow_bridge/esp32_espnow_bridge.ino` | ESP32에 올리는 펌웨어. Serial↔ESP-NOW 사이로 26바이트 프레임을 그대로 통과시켜주는 브릿지 (내용은 해석 안 함) |

## 실행 방법

세 라즈베리파이가 **같은 Wi-Fi 네트워크(같은 서브넷)**에 연결된 상태에서, 각자
자기 차량 ID로 실행합니다.

```bash
# 후방차 라즈베리파이에서
python3 v2v_node.py --id 1

# 전방차 라즈베리파이에서
python3 v2v_node.py --id 2

# 측면차 라즈베리파이에서
python3 v2v_node.py --id 3
```

정상 동작하면 이런 식으로 다른 차량이 보낸 BSM을 받아서 찍습니다:

```
[OK ]  전방차 seq=   31 speed= 48.0km/h heading= 240.2 K=0.75 danger=False
  --- 상태: 전방차=LIVE (기준 300ms) ---
```

- `[OK ]` : 정상 수신, CRC/시퀀스 검증 통과
- `[DUP]` : 중복/역순 프레임(폐기됨)
- `[DROP]` : CRC 실패 등으로 파싱 자체가 실패한 프레임(폐기됨)
- 1초마다 `--- 상태: ... ---` 줄로 각 차량이 LIVE인지 STALE인지 요약해서 보여줌

## GPS/센서 아직 다 안 붙었을 때

`v2v_node.py` 안의 `DummySensorSource`가 위경도를 원을 그리며 서서히 움직이는
가짜 값으로 채워서, GPS/레이더/OBD가 아직 다 연결 안 돼있어도 **통신 자체는
지금 바로 테스트 가능**합니다. 실센서 연결되면 `DummySensorSource.read()` 안의
TODO 표시된 부분만 실제 GPS(gpsd)/OBD/IMU 읽는 코드로 바꿔치기하면 됩니다.

## 네트워크가 다를 때 (다른 서브넷이거나 브로드캐스트가 막힌 공유기)

기본 브로드캐스트 주소(`255.255.255.255`)가 특정 공유기/핫스팟 환경에서 막히면,
`--broadcast-addr` 옵션으로 서브넷 브로드캐스트 주소를 직접 지정하세요. 예를
들어 공유기 IP 대역이 `192.168.1.x`라면:

```bash
python3 v2v_node.py --id 1 --broadcast-addr 192.168.1.255
```

## 동작 확인 완료 사항 (2026-09-23)

같은 머신에서 두 프로세스를 띄워 로컬로 인코딩→송신→수신→CRC검증→시퀀스체크→
신신선도판단까지 파이프라인 전체가 정상 동작하는 것을 확인했습니다. 실제
라즈베리파이 3대 환경에서의 무선 테스트는 아직 안 해봤으니, 처음 돌릴 때
방화벽(ufw 등)이 UDP 9000번 포트를 막고 있지 않은지 정도만 확인하면 됩니다.

## ESP32(ESP-NOW)로 실행하기

1. 먼저 pyserial 설치: `pip install pyserial --break-system-packages`
2. ESP32에 펌웨어 업로드 (arduino-cli 기준):
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32 esp32_espnow_bridge
   arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 esp32_espnow_bridge
   ```
3. ESP32를 RPi에 USB로 꽂은 상태에서 `--transport esp-now` 옵션으로 실행:
   ```bash
   python3 v2v_node.py --id 1 --transport esp-now --serial-port /dev/ttyUSB0
   ```
4. 3대 라즈베리파이 전부 이렇게 각자의 `--id`로 실행하면, ESP32끼리 ESP-NOW로
   직접 브로드캐스트를 주고받습니다 (RPi Wi-Fi/공유기와 무관하게 동작).

**동작 원리**: RPi(Python) → USB-Serial → ESP32(펌웨어) → ESP-NOW 브로드캐스트
→ 다른 차량 ESP32 → USB-Serial → 다른 차량 RPi(Python). 프레임 포맷(SYNC/END,
CRC-16)은 UDP 버전과 완전히 동일해서 `bsm_protocol.py`, `freshness.py`는 손댈
필요가 없습니다 — `transport.py`가 통로만 바꿔치기해주는 구조입니다.

**아직 확인 안 된 것**: 로컬 코드 레벨 구현/문법 확인까지만 마쳤고, 실제 ESP32
2대 이상으로 ESP-NOW 무선 송수신이 되는지는 하드웨어에서 직접 테스트해봐야
합니다. 안테나(U.FL 커넥터) 제대로 물려있어야 신호가 나갑니다.
