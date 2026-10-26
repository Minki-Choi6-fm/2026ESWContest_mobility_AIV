
## 9. 실차 통합 실행 (S12): 주행 전 체크리스트 → 실행 순서 → 안 될 때 확인할 것

이 장은 한 번 실행으로 "레이더(+카메라) → 앞차 선택 → 거리/상대속도 → 급정거 위험 판단 → 앞차로 BSM 송신 + 내 차 알림"을 돌리는 방법입니다. 지금까지 **가짜 입력(가짜 STM32/카메라/앞차 루프백/mock 부저)으로만** 시험했습니다. 실제 차·실제 카메라·실제 무선·실제 부저로는 아직 아무것도 확인하지 않았습니다.

### 9-1. 주행 전 체크리스트 (사람이 하는 것)
- [ ] 레이더 전원: 차량 12 V 직결로 바꿨나? 바꿨으면 실행할 때 `RADAR_POWER=direct12v` 를 붙이고, 아니면 로그의 `power_suspect=1` 구간(Safe/0xFF)은 전원 영향일 수 있음으로 읽는다.
- [ ] 멀티미터 MIN 유지로 레이더 12 V 단자 최저 전압을 한 번 확인(10 V 아래로 떨어지면 끊김 가능).
- [ ] 보드에 v18d2 가 올라가 있나(`~/flash_log.txt` 마지막 줄 `rear_v18d2.bin 30d3b65f0608fc88`, 부팅 배너 `(re)`).
- [ ] 카메라(RealSense) USB3 연결. 연결하면 **먼저** `python3 ~/ESWContest_ws/pi5/pipeline/camera_check.py` (이번에 만들어만 두고 실행 안 함).
- [ ] ESP-NOW 를 쓰면: 양쪽 ESP32 에 **같은 브리지 파일**(앞차 버전 권장)을 사용자가 올렸나, 안테나 연결, `ls /dev/ttyUSB0`.
- [ ] 부저(능동형, Pi5 물리 37번 = BCM GPIO26, GND 34번): 기본은 `BUZZER=mock`(소리 안 남). 울리려면 `BUZZER=real` 로 직접 지정. 소리 패턴은 앞차와 같음(위험 2단계에서 0.1 s 켜짐/0.1 s 꺼짐).
- [ ] 앞차 쪽도 켜져 있나(앞차 `front_node.py`, 같은 ESP-NOW 채널).

### 9-2. 실행 순서 (터미널에 치는 것)
```
cd ~/ESWContest_ws
tools/run_car_test.sh                          # 점검 출력 → 레이더 단독/V2V(UDP)/부저 mock 으로 시작
# 변형
CAMERA=1 tools/run_car_test.sh                 # 카메라도 시작(안 열려도 레이더 단독으로 계속)
TRANSPORT=esp-now tools/run_car_test.sh        # ESP32 무선 사용
RADAR_POWER=direct12v tools/run_car_test.sh    # 레이더를 12 V 직결한 뒤
BUZZER=real tools/run_car_test.sh              # 실제 부저 (--really-drive-gpio26 가 자동으로 같이 붙음)
BIG=1 tools/run_car_test.sh                    # 조수석용 큰 글씨 0/1/2/? 화면 (터미널을 크게 열고 실행)
WRITE_STM32=1 tools/run_car_test.sh            # 0x20 경고를 STM32 로 전송(현재 펌웨어는 버림 — 효과 없음)
```
끝낼 때는 `Ctrl+C` 한 번(부저/경고가 자동으로 꺼짐). 로그는 `~/ESWContest_ws/logs/car_test/<날짜_시각>/` 에 남습니다(`drive_log.csv`, `output_sink.csv`, `bsm_tx.csv`, `rec/`).
- `[FAIL]` 이 하나라도 있으면 시작하지 않고 이유를 알려 줍니다. `[WARN]` 은 계속 진행(예: 카메라 없음 → 레이더 단독).
- 속도(OBD/GPS)가 없으면 **부저는 울리지 않고 표시만** 합니다(오경보 방지, 기본값). 속도가 있으면 기존 규칙대로입니다.

### 9-3. 안 될 때 확인할 것
| 증상 | 확인 |
|---|---|
| 시작이 거부됨 `/dev/ttyAMA2 를 다른 프로그램이 잡고 있음` | `fuser -v /dev/ttyAMA2` — drive_logger/다른 리시버를 먼저 종료(읽기는 한 프로그램만). `ego_speed_tx.py`(PID 확인)는 쓰기 전용이라 괜찮음 |
| 레벨이 계속 `?`(0xFF) | `drive_log.csv` 의 `radar_down`, `power_suspect`, `stm_status`: 레이더 전원 처짐이면 약 5.5 초 주기로 끊김. 12 V 직결·MIN 유지 측정. `stm_status` 가 비어 있으면 STM32 프레임이 안 들어옴(UART 선/전원/보드) |
| 레벨이 0/1 만 나오고 2 가 안 나옴 | `reason` 열: 억제(정지 물체/비현실 차간거리/정차)인지, 속도 `speed_src=none` 인지 확인 |
| 위험인데 부저가 안 울림 | 속도 모름이면 정상(표시만). `BUZZER=mock`(기본, 소리 안 남)인지 확인 → 울리려면 `BUZZER=real` |
| 앞차에 경고가 안 뜸 | `bsm_tx.csv` 에 행이 늘어나는지, 앞차 화면의 `후방차=LIVE` 인지(STALE 이면 전송/채널 문제). ESP-NOW 는 양쪽 채널 동일해야 함, 시작 후 2~3초는 비어 있을 수 있음 |
| 카메라가 안 열림 | `camera_check.py` 출력, USB3 포트, 다른 프로그램이 카메라를 쓰는지. 안 열려도 레이더 단독으로 계속 동작(`camera_alive=0`) |
| OLED 에 위험 단계가 안 보임 | 정상: 현재 보드 펌웨어는 0x20 을 버림(`S12_INTEGRATION_NOTES.md` [a]). OLED 알림은 펌웨어 변경이 필요 |
| 로그에 `RADAR TIMEOUT`(ST-Link 텍스트) | v18d2 는 1 초 넘게 레이더 프레임이 없을 때만 나옴 → 전원/배선 의심 |
