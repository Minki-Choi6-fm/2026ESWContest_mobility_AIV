# S12 실차 테스트용 통합 — 설계 확인과 구현 메모

범례: **가짜 입력 시험** = 가짜 STM32(pty)/가짜 카메라/앞차 루프백/mock 부저. **하드웨어 확인** = 실제 보드/카메라/무선으로 본 것. S12 에서 하드웨어로 새로 확인한 것은 **없습니다**(보드 접속·카메라·실부저 금지).

## [a] Pi→STM32 명령 (main_v18c.c 수신부 = 보드의 v18d2 와 같음) — 소스를 읽은 사실
| 명령 | 펌웨어가 하는 일 | OLED | 부저 | STM32 쪽 자동 해제 |
|---|---|---|---|---|
| 0x22 EGO_SPEED (LEN 5) | `gps_speed_x100`, `gps_src`, `gps_age_ms`, `gps_rx_ms` 저장. OBD 가 1 s 넘게 없을 때 `get_ego_speed()` 가 GPS 값을 씀 | 5번째 줄 `SPD:xx.x GPS` 로 표시 | 없음 | GPS 값은 `((now-rx)+age) < GPS_STALE_MS` 이면 사용, 아니면 `SPD: --- (no src)` — 오래되면 자동으로 안 씀 |
| 0x20 DISPLAY_WARNING | **프레임 검사(STX/LEN/XOR/ETX)만 하고 버림**. `rpi_rx_ok++` 만 증가 (`rpi_rx_step`, 약 2096~2106행) | **아무것도 안 함** (OLED 에 경고 줄 자체가 없음) | **없음** (펌웨어에 부저 핀/코드 없음) | 해당 없음 |
| 0x21 LINK_STATUS | 0x20 과 동일하게 버림 | 없음 | 없음 | 해당 없음 |
→ **현재 보드로는 "내 차 OLED 알림"을 STM32 경유로 만들 수 없습니다.** 만들려면 펌웨어에서 0x20 해석 + OLED 줄 + 타임아웃이 필요(펌웨어 변경 = v18e 이후, 이번 범위 아님; 사용자가 보류). 부저는 Pi5 의 GPIO26 로 직접 울리는 `BuzzerSink`(능동형)만 유효합니다.
→ 그래서 `Stm32WarnSink` 는 프레임 형식·재전송·끄기 동작을 가짜 pty 로만 시험했고, 실제 보드 효과는 **없음(소스 기준) / 보드 확인 필요**.
- 프레임 형식 불일치 [확인 필요]: PROTOCOLS.md 는 `SENDER_ID, DANGER_LEVEL, DISTANCE cm`, 앞차 코드(`front_logic.encode_v2v_warn`)는 `level, src_vid, dist(0.1 m)`. 이번 sink 는 **앞차 코드 쪽**을 따름(SRC=0 은 "내 레이더").

### 같은 포트에 두 프로세스가 쓸 때 (ego_speed_tx.py PID 648 + pipeline_main --write-stm32)
- 조건: **프레임 하나 = write() 한 번**, 끝까지 쓰고(부분 쓰기는 버림), 두 쪽 다 같은 프레임 형식. ego_speed_tx.py 는 `serial.write(frame)` 한 번(부분 쓰기 시 예외 처리 코드 있음), `Stm32WarnSink` 도 `os.write()` 한 번. 리눅스 tty 쓰기는 write() 호출 단위로 잠금(atomic_write_lock)이 걸려 서로 안 섞인다는 것이 일반적 동작이지만, **이 PC 의 실제 /dev/ttyAMA2 에서 측정한 것은 아님 [확인 필요]**.
- 가짜 pty 시험 `writer_mix_test.py`: 두 프로세스(0x22 와 0x20) 각 6000 프레임을 최대 속도로 동시에 쓰기 → 아래 "시험 결과" 참고.
- 쓰기 열기: `O_WRONLY|O_NOCTTY|O_NONBLOCK`, termios 건드리지 않음(포트 설정은 서비스가 이미 함).

## [b] 카메라 연동 (구조)
```
카메라 프로세스 camera_node.py (taskset 코어 0-1) --JSON UDP 127.0.0.1:19100--> pipeline_main (코어 2-3)
                                                                    └ camera_link.CameraLink → vision_adapter.CameraConfirm (N=3 연속 프레임, 박스당 가장 가까운 레이더 점 1개)
                                                                    └ brain.evaluate(camera_confirmed=...)
```
- **camera_confirmed 는 보조 플래그**입니다: danger.py 의 "정지 물체 억제"와 "비현실 차간거리(implausible headway) 억제"를 풀어 주는 용도뿐(코드 주석 `camera_link.py`, `pipeline_main.py`). 카메라가 없거나 죽거나 패킷이 깨져도 예외 없이 False → 레이더 단독으로 진행.
- `camera_node.py`: 카메라/YOLO 가 죽으면 3초 뒤 다시 시도(재연결 시 자동 복구). `camera_check.py`: 실차 전 카메라 열림/프레임률/YOLO 지연 점검(**작성만, 실행 안 함**).
- 지금까지 카메라 장치는 열지 않았고 YOLO 도 돌리지 않았습니다. 가짜 검출은 `sim_scenario.py` 가 앞차 레이더 점을 `fusion.project` 로 투영해 박스를 그립니다.

## [c] ESP-NOW 브리지
- "컴파일 안 됨" 원인(사실): 코어 3.x 에서 `esp_now_register_send_cb` 콜백의 첫 인자가 `const uint8_t* mac` → `const esp_now_send_info_t*`(= `wifi_tx_info_t`) 로 바뀌었는데 9/27 `.ino` 의 `onDataSent` 는 옛 시그니처, 수신 콜백도 2.x 시그니처였음.
- 수정: `v2v/v2v_stack/esp32_espnow_bridge/esp32_espnow_bridge.ino` 의 두 콜백을 `#if ESP_ARDUINO_VERSION_MAJOR >= 3` 로 분기(원본은 `.bak_*_s12`). `arduino-cli compile --fqbn esp32:esp32:esp32` (코어 3.3.12) **컴파일 성공**(883,436 B, 67 %). **업로드는 안 함.**
- 앞차 쪽 `v2v.zip` 의 브리지(`~/front_test/v2v/esp32_espnow_bridge`)는 이미 코어 3.x 분기, CRC 검사, **고정 채널 1**, 20 dBm 이 들어 있고 같은 코어에서 컴파일됨(886,764 B). **두 차의 ESP32 가 같은 채널이어야 하므로 양쪽에 같은 파일(앞차 버전)을 올리는 것을 권장**합니다(후방 원본은 채널 0 = 기본 채널, CRC 검사 없음). 어느 쪽이든 업로드는 사용자가 합니다.
- 가짜 시리얼 시험 `espnow_bridge_sim_test.py`: 스케치 로직을 파이썬으로 옮긴 모델 + 앞차 `front_node.py --transport esp-now` → 아래 결과. **실제 ESP32/무선은 미검증**.
- [확인 필요] CP2102 보드는 포트를 여는 순간 DTR/RTS 로 ESP32 가 리셋될 수 있음 → 시작 후 2~3초 BSM 이 비는 것은 정상일 수 있음. 부팅 시 ROM 문구(쓰레기 바이트)는 재동기화로 걸러짐(모델 시험).

## [d] 내 차 알림
- OLED: `Stm32WarnSink`(위 표대로 **현재 보드에선 효과 없음**). 규칙 구현: 레벨 바뀔 때 즉시 전송, 위험(≠0)이면 0.2 s 마다 재전송, 0 이면 1 s 마다 keep-alive, `safe_state` 에서 레벨 0(OFF) 2번 전송, 허브 워치독이 메인 루프 멈춤(>1 s)을 감지하면 OFF 전송, 정상 종료/예외/SIGTERM 에서도 OFF. **SIGKILL/전원 차단은 못 막음**(STM32 쪽 타임아웃이 생겨야 함; 제안 타임아웃은 재전송 주기의 3배, 약 0.6 s [PROPOSAL]). 0xFF 는 레벨 0xFF 프레임("?") 이고 부저 후보 아님.
- 부저: `BuzzerSink` 그대로(레벨 2 만 울림). 기본 mock/off, 실제 구동은 `--buzzer real --really-drive-gpio26` 둘 다 있어야 함. **새 규칙**: 자차 속도를 모르면(OBD/GPS 없음) 부저 안 울리고 표시(OLED 명령/콘솔/CSV)만. `--buzz-without-speed` 로 해제. 기본 동작이 이것.
- 모든 새 쓰기 기능은 `--write-stm32` 가 없으면 꺼져 있음.
- BSM SEQ: `seq = int(time.time()*10) & 0xFFFF` 로 시작(약 109분마다 한 바퀴). 재시작해도 이전 SEQ 보다 앞서므로 앞차가 1 s 기다리지 않음(루프백 시험 4d). STM32 펌웨어는 건드리지 않음.

## [e] 주행 로그와 통합 시험
- `drive_log.csv`(20 Hz): `t_rel, wall, n_radar_obj, tgt_id, tgt_d_m, tgt_lat_m, tgt_vrel_ms, ttc_s, level, level_text, camera_confirmed, camera_alive, speed_kmh, speed_src, bsm_sent, stm_status, safe_mode, radar_down, radar_down_s, power_suspect, reason`. 첫 줄은 `# radar_power=…` 메모. `radar_down` = STM32 SAFE_MODE 비트 또는 RADAR_OK=0. `power_suspect` = radar_down 이면서 `--radar-power suspect`(기본; 레이더를 차량 12 V 직결한 뒤 `direct12v`).
- `integration_test.py`: 가짜 STM32+가짜 카메라+pipeline_main+앞차 `front_node`(루프백, `--buzzer-gpio` 없음) 케이스 A(속도 있음 50 km/h + 카메라 + 레이더 단절 + 카메라 단절) / B(속도 모름) / B2(속도 모름 + 부저 허용) / C(카메라 없음).
