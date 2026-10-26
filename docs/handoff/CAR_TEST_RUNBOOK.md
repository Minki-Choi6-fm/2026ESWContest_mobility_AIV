# CAR_TEST_RUNBOOK — 후방차 첫 주행 시험 절차서

> **실차 부저 필수 조건: OBD 또는 GPS에서 유효한 자차 속도가 들어와야 합니다.**
> **속도가 없거나 오래된 값이면 위험 레벨 2도 기본적으로 소리 없이 표시만 합니다.**
> - [ ] 출발 전 `speed_src`가 OBD/GPS이고 속도가 최신인지 확인합니다.
> - [ ] `BUZZER=real` 지정 여부를 확인합니다. 기본 mock은 소리가 나지 않습니다.
> - [ ] `--buzz-without-speed`는 책상 시험용 예외이며 실차 기본 실행에는 넣지 않습니다.
> - [ ] **GPS 속도를 쓰려면**: GPS 안테나를 하늘이 보이는 곳(차량 대시보드 위·창가)에 두고, 출발 전에 `timeout 5 gpspipe -w -n 8 | grep TPV` 로 `"mode":2` 이상(fix)이 나오는지 **먼저 확인**합니다. 실내/책상에서는 fix가 안 잡혀 속도가 `none`이 됩니다(그러면 부저는 안 울리고 표시만).


> ## ⚠ 먼저 할 일: 장착 직후 사전 점검 (읽기 전용)
> ```
> python3 -I ~/ESWContest_ws/pi5/pipeline/preflight_check.py
> ```
> 서비스(aiv-ego-speed, gpsd), `/dev/ttyAMA2` 점유자, gpsd 가 읽는 GPS 장치와 fix, 디스크, 시스템 시각, 로그 폴더 쓰기 가능 여부, `state/k_profile.json`, 코어 분리, vision 패키지 경로, 전원(throttled, EXT5V)을 한 번에 점검해 **PASS / WARN / FAIL** 표로 보여 줍니다. FAIL 이 하나라도 있으면 종료 코드가 1 입니다.
> 어떤 시리얼 포트도 열지 않고 아무것도 쓰지 않습니다(gpsd 에 localhost 읽기 전용 구독만 함). 시험 중에도 다시 실행해도 안전합니다. 일반 사용자 권한으로는 **다른 사용자 소유 프로세스의 파일 핸들은 `fuser` 로 보이지 않으므로**, 확인 못 한 항목은 WARN 으로 표시됩니다.
>
> ### ⛔ **비전 패키지의 `main.py` / `radar_receiver.py` 를 차에서 그대로 실행하지 마세요.**
> **`radar_receiver.py` 의 기본 포트는 CP2102(`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`) 이고 `serial.Serial(..., exclusive=True)` 로 엽니다.**
> 기본값으로 실행하면 **CP2102(= ESP32 브리지일 가능성 [가설])를 독점**해 V2V 가 끊기고, 다른 포트를 지정해도 **`/dev/ttyAMA2` 를 읽는 파이프라인과 UART 를 두고 경쟁**해 바이트가 나뉩니다. `uart_status_receiver.py` 도 `exclusive=True` 입니다.
> 비전/카메라 프로세스는 **UART 를 열지 않고** 검출 결과(bbox, class, camera_mono)만 발행해야 합니다 (`VISION_INTEGRATION.md`). `preflight_check.py` 가 이 파일들이 실행 중이면 FAIL, 기본 포트 설정이 남아 있으면 WARN 으로 알려 줍니다.


대상: 최민기 님과 팀원 (차에 Pi5를 장착하고 주행 로그를 뽑는 사람). 작성 시점의 코드/측정 상태를 기준으로 하며,
`[가설]` 은 측정 안 된 추정, `[PROPOSAL]` 은 팀 합의 전 숫자입니다. 이 시험의 목적은 **측정하지 못한 값을 처음 측정하는 것**입니다
(실차 프레임 손실/BAD 비율, Safe Mode, STALE/PDR, 5V). 경고 정확도를 재는 시험이 아닙니다(정답 라벨 없음).

## 0. 원칙 (어기면 데이터가 깨집니다)
1. **시리얼 포트는 읽는 프로그램이 하나만.** 두 프로그램이 같은 포트를 읽으면 바이트가 나뉘어 프레임이 깨집니다.
2. STM32 리셋/플래시/ST-Link 만지기 금지, 실제 UART의 0x20/0x21 송신 금지(가짜 pty 쓰기 경로만 시험), `aiv-ego-speed` 서비스는 그대로 둡니다(쓰기 전용).
3. Pi5 CPU: YOLO 코어 0–1, V2V 코어 2–3 (규칙 3). 수신기/노드는 `taskset -c 2,3` 로 실행.

## 1. 누가 무엇을 읽나
| 자원 | 담당 프로그램 | 비고 |
|---|---|---|
| `/dev/ttyAMA2` (STM32 바이너리) | **`live_receiver.py --record` 또는 `bsm_node.py --record`** (둘 중 하나만) | `aiv-ego-speed` 서비스는 같은 포트에 **쓰기만** 함(코드 확인) → 병행 가능 |
| `/dev/ttyACM0` (ST-Link 디버그 글자) | `drive_logger_nouart.py` (stm_debug.log) | 다른 `cat`/터미널로 읽지 말 것 |
| gpsd (localhost:2947) | 여러 클라이언트 동시 가능 | drive_logger, 수신기, 서비스가 각자 구독 |
| 카메라 | `drive_logger_nouart.py` (mp4 + camera_ts.csv) | `~/realsense-venv/bin/python` 으로 실행 |
| 전원 | `pi5_power_log_v1.py` | EXT5V_V, throttled, temp |

### drive_logger.py 의 UART 읽기를 끄는 옵션 — 확인 결과
`drive_logger.py` 에는 UART 끄기 옵션이 **없습니다** (`--no-cam --no-debug --no-gps --sim-cam` 만 있음; `uart_thread` 는 항상 시작).
그래서 원본은 수정하지 않고 **래퍼 `pi5/pipeline/drive_logger_nouart.py`** 를 만들었습니다: 원본을 그대로 import 한 뒤 메모리에서 `uart_thread` 만
"포트를 열지 않는 함수"로 바꿉니다. 명령줄은 원본과 같고 `--uart` 는 무시됩니다. 사무실에서 확인: UART를 열지 않음(fuser 에 로거 없음),
`frames.csv`/`radar.csv` 는 만들지 않음, `sys.csv`/`meta.json` 정상. 콘솔의 "uart_frames" 경고도 숨깁니다.

## 2. 장착 직후 체크리스트 (시동 전/후)
| # | 확인 | 명령 | 정상 |
|---|---|---|---|
| 1 | 전원 | `vcgencmd get_throttled` / `vcgencmd pmic_read_adc \| grep EXT5V_V` | `0x0` / 5.0 V 이상 (미만이면 5V 문제 가설 §4.1 후보) |
| 2 | 디스크 | `df -h ~` | 10 GB 이상 (영상 30fps 약 30–60 MB/분, 2 GB 미만이면 로거가 시작 안 함) |
| 3 | 포트 점유 | `fuser -v /dev/ttyAMA2 /dev/ttyACM0` | AMA2: `ego_speed_tx.py`(서비스) 하나뿐, ACM0: 없음 |
| 4 | 서비스 | `systemctl is-active aiv-ego-speed gpsd` | 둘 다 active |
| 5 | STM32 살아있음 | `python3 -I pi5/pipeline/live_receiver.py --duration 10` | `sts=0x0F[RADAR IMU OBD CAN]`, loss≈0, BAD=0. **차에서 OBD 연결 시 OBD 비트가 켜져야 함** (책상에서는 0x0B) |
| 6 | 레이더 | 같은 출력의 `radar_0x12=` | 물체가 있으면 ok. 물체가 없어 `--`/TIMEOUT 이어도 `RADAR` 비트가 켜져 있으면 정상(레이더 생존은 RADAR_OK 비트) |
| 7 | GPS fix | `gpspipe -w -n 6 \| grep TPV` | `"mode":3` (실외, 수 분 걸릴 수 있음). mode 1 이면 속도 폴백 불가 |
| 8 | ESP32 브리지 | `ls -l /dev/serial/by-id/` | CP2102(`/dev/ttyUSB0`)가 보임. 이 CP2102 가 ESP32 보드의 USB-UART 일 가능성 **[가설]** — 팀원에게 확인. **U.FL 안테나 연결 확인(없으면 신호 거의 안 나감)** |
| 9 | 시각 기록 | `date -Is` | 노트에 적기 (두 차는 시계가 달라 지연 직접 비교 불가) |

## 3. 시작/종료 순서 (tmux 창 4개)
시작 (이 순서):
```
# 창1  전원
python3 ~/ESWContest_ws/pi5/logging_tools/pi5_power_log_v1.py --out ~/vehicle_test_logs/power_$(date +%Y%m%d_%H%M%S).csv
# 창2  카메라 + 디버그 + gps + sys (UART 읽기 없음)
~/realsense-venv/bin/python ~/ESWContest_ws/pi5/pipeline/drive_logger_nouart.py --note "시나리오 이름"
#      -> ~/vehicle_test_logs/drive_YYYYMMDD_HHMMSS/ 가 만들어짐
# 창3  UART 담당 (A 또는 B 중 하나)
D=$(ls -dt ~/vehicle_test_logs/drive_* | head -1)
#  A) 센서 기록만:
taskset -c 2,3 python3 -I ~/ESWContest_ws/pi5/pipeline/live_receiver.py --record "$D" --stats-dir "$D"
#  B) 통합 실행기: 기록 + BSM 송수신 + 위험판단 + OutputSink(시뮬레이션; STM32 로는 아무것도 안 씀, UART 는 읽기 전용):
taskset -c 2,3 python3 -I ~/ESWContest_ws/pi5/pipeline/pipeline_main.py --uart /dev/ttyAMA2 --gpsd-port 2947 --id 1 \
     --port 9000 --broadcast-addr <서브넷>.255 --out "$D/pipeline" --record "$D" --stats-dir "$D" \
     --k-profile ~/ESWContest_ws/state/k_profile.json
# 창4  노트/이벤트 시각 메모
echo "$(date +%s.%N) 앞차 급제동" >> "$D/notes.txt"
```
종료 (역순): 창3 Ctrl+C(마지막 통계가 저장됨) → 창2 Ctrl+C **한 번** 후 3초 대기 → 창1 Ctrl+C.
주의: `--broadcast-addr 255.255.255.255` 는 같은 핫스팟의 다른 기기에도 닿습니다. 서브넷 브로드캐스트를 쓰고 팀원 노드와 포트가 겹치지 않는지 확인하세요.
**ESP-NOW** 는 `--transport esp-now --serial-port /dev/ttyUSB0`(어댑터에만 연결됨, 이 환경에서 **미시험**; 후방 브리지는 S12 코어 3.x 수정 후 컴파일 성공, 실제 무선은 미검증 → `V2V_DEPLOY_NOTES.md`). 링크만 먼저 볼 때는 기존 `v2v_node.py --id N` (더미 센서) 로도 됩니다.
**주의:** B 모드는 위험판단이 STM32 K/DANGER 와 무관한 Pi5 시뮬레이션입니다. 아직 STM32 의 K/DANGER 는 0xFF 입니다.

## 4. 시험 중 반드시 기록할 지표
| 지표 | 어디에 저장되나 | 어떻게 보나 |
|---|---|---|
| UART 프레임 손실률(STM32 시계 기준) | `$D/rx_stats_*.csv` (`loss_pct`, `expected_0x10/0x11`) | 책상 0.000 %, 과거 주행 로그 1.35–2.34 % 였음 → 같은지 비교 |
| BAD (LEN/체크섬/재동기화) | 같은 CSV `bad_len`, `bad_chk`, `resync_events` | loss>0 이고 BAD>0 이면 비트 오류, loss>0 인데 BAD=0 이면 STM32 누락/수신 지연 **[가설 구분법]** |
| 센서 상태/Safe Mode | 수신기 종료 시 `SENSOR_STATUS values seen`, `$D/stm_debug.log` 의 `RADAR TIMEOUT` | SAFE_MODE(bit4) 시간 비율 |
| STM32 재시작 | 상태줄 `STM_RESETS`, stm_debug.log `RESET CAUSE` | 몇 번, 언제 |
| 레이더 생존 vs 타깃 | `frames.csv` → `s4_replay.py`, 상태줄 `sts` | RADAR_OK 비트 시간 비율과 0x12 유무를 분리해서 |
| OBD vs GPS 속도 | `frames.csv`+`gps.jsonl` → `speed_compare.py` | 오차와 GPS 지연 |
| 5V / 스로틀 / 온도 | `power_*.csv`, `sys.csv` | 레이더 끊김과 시각 맞춰 보기 (과거 로그에는 5V 열이 없었음) |
| V2V 송신 수 | B 모드 `$D/pipeline/bsm_tx.csv` (수신 쪽 PDR/STALE 은 상대 차량의 `bsm_node.py`/로그에서) | `loopback_v2v_test.py` 의 분석 함수와 같은 방식 |
| 위험판단 결과(시뮬레이션) | `$D/pipeline/output_sink.csv` (레벨·사유·억제 태그, 종료 시 SAFE_STATE 행) | **정확도 측정이 아님.** 주행 후 `s4_replay.py` 로도 같은 로그를 재생 가능 |
| RSSI | **ESP-NOW 브리지 펌웨어는 프레임만 통과시켜 RSSI 를 기록 못 함**(펌웨어 수정은 승인 필요). Wi-Fi/UDP 는 `iw dev wlan0 link` 의 signal 을 1초마다 CSV 로: `while :; do echo "$(date +%s.%N),$(iw dev wlan0 link \| grep signal)"; sleep 1; done >> $D/rssi_wifi.csv` | |
| 두 차 간 지연 | **측정 불가** (시계 불일치). 필요하면 별도 왕복 시험 설계 | 루프백 지연은 성능으로 쓰지 말 것 |
| 이벤트 시각 | `$D/notes.txt` | 앞차 제동, 차선 변경, V2V 차단 등 |

## 5. 주행 후 분석 명령
```
cd ~/ESWContest_ws/pi5/pipeline
python3 -I loss_analysis.py "$D"            # 빠진 0x10 의 시각 상관
python3 -I s4_replay.py "$D"                # 타깃/K/위험 재생 (정답 라벨 없음)
python3 -I speed_compare.py "$D"            # OBD vs GPS
python3 -I k_estimator.py "$D"              # 제동 구간별 K (구간 수가 적으면 그대로 적기)
```

## 6. 아직 모르는 것 / 한계
- 레이더 좌우(lat) 부호 미확인(타깃 선택은 |lat| 사용). 차에서 오른쪽 차선 물체를 세워 놓고 부호 확인 필요.
- `[PROPOSAL]` 숫자 전부(constants.py 의 S3/S4 구역, OBD 1000 ms, K 기본값 0.5, d0 2 m, 임계값들)는 합의 전.
- K 는 실주행 로그에서 **인정되는 제동 구간 0개** (플래그가 단발). 이번 시험에서 일부러 0.3 g 이상 제동을 몇 번 넣어야 K 첫 실측이 생깁니다.
- 정지된 도로변 물체가 타깃으로 잡히면 위험(2)이 자주 뜹니다(재생에서 확인). 이번 시험에서 원인 시각을 노트에 남겨 주세요.

## 7. K 프로필 만들기 (운전자의 평소 제동 습관 → K)
K 는 차의 최대 성능이 아니라 **운전자가 평소 감속할 때 쓰는 제동 강도**입니다(평소 절반만 밟으면 K 가 낮아져 경고가 더 일찍 나옴). Pi5 는 OBD 속도와 IMU 로 스스로 계산합니다(펌웨어 BRAKE_ACTIVE 는 쓰지 않음).
1. **정차 2 초 이상** (평지에서, 시동 켠 채 OBD 속도 0): IMU 정차 편향을 측정합니다. 이 편향은 장착 각도/경사의 중력 성분입니다(로그: 편향 ≈ −1000·sin(피치)).
   확인: `python3 -I pi5/pipeline/k_estimator.py "$D"` 의 "standstills used for the bias" 에 줄이 생기고 bias 가 +60…+110 mg 근처. 정차 장소가 바뀌어(경사가 다르면) 편향도 달라지므로 가능하면 평소 정차하는 평지에서.
2. **안전한 곳에서 평소처럼 감속을 몇 회** (예: 40 km/h → 15 km/h 를 신호 대기 하듯, 0.4 초 이상 0.08 g 이상, OBD 속도가 2 km/h 이상 떨어져야 이벤트로 인정). 3 회 미만이면 K 는 N/A(BSM 에 광고 안 함)이고 계산에는 기본값 0.5 가 쓰입니다.
3. `~/ESWContest_ws/state/k_profile.json` 확인: `bias_mg`, `events`(최근 10개), `n_events`, `k`(3개 이상일 때만 숫자). 재시작해도 유지됩니다. 초기화는 `--reset-k`, 시연용 고정은 `--k-fixed 0.6`.
4. 로그 3개에서는 인정 이벤트가 **2/0/0개**(처음부터 편향을 알아도 2/0/3개)뿐이라 K 는 아직 측정 근거가 없습니다 → 이번 시험에서 이 절차를 실제로 해 보는 것이 첫 실측입니다.
5. 급제동 시험은 **폐쇄된 안전한 장소에서만, 선택 사항**입니다. 급제동 값은 "평소 습관"이 아니므로 K 에 섞이지 않게 하려면 프로필을 따로 보관하세요(`--k-profile` 경로를 다르게).
한계: 도로 경사와 서스펜션의 앞쪽 쏠림(피치 변화)은 보정하지 않습니다(정차 편향만 보정). 한 축 가속도만 사용. 프로필은 운전자별입니다.

## 8. 펌웨어 플래시 기록 규칙 (S10)
- **펌웨어를 올리기 전에** `~/flash_log.txt` 에 한 줄을 적는다: `날짜 | 파일명 | sha256 앞 16자 | 올린 사람/PC`. 노트북 CubeIDE 로 올릴 때도 똑같이 적는다(올린 뒤가 아니라 **올리기 전**).
- 기록만 하는 도구: `~/ESWContest_ws/tools/flash_note.sh <파일> "<올린 사람/PC>"` (sha256 을 계산해 한 줄을 덧붙임. 플래시·빌드·보드 접속은 하지 않음).
- 왜: 10/16 이후 보드에 무엇이 올라갔는지 기록이 없어 펌웨어를 배너로도 확정하지 못했다(`FIRMWARE_ON_BOARD.md`). 기록이 있으면 로그 해석(예: v17 vs v18c)이 가능하다.
- 절차 전체: `FLASH_PROCEDURE_V18D2.md`.


## 9. 실차 통합 실행 (S12): 주행 전 체크리스트 → 실행 순서 → 안 될 때 확인할 것

이 장은 한 번 실행으로 "레이더(+카메라) → 앞차 선택 → 거리/상대속도 → 급정거 위험 판단 → 앞차로 BSM 송신 + 내 차 알림"을 돌리는 방법입니다. 지금까지 **가짜 입력(가짜 STM32/카메라/앞차 루프백/mock 부저)으로만** 시험했습니다. 실제 차·실제 카메라·실제 무선·실제 부저로는 아직 아무것도 확인하지 않았습니다.

### 9-1. 주행 전 체크리스트 (사람이 하는 것)
- [ ] 레이더 전원: 차량 12 V 직결로 바꿨나? 바꿨으면 실행할 때 `RADAR_POWER=direct12v` 를 붙이고, 아니면 로그의 `power_suspect=1` 구간(Safe/0xFF)은 전원 영향일 수 있음으로 읽는다.
- [ ] 멀티미터 MIN 유지로 레이더 12 V 단자 최저 전압을 기록. MR76 허용 전압 범위는 [확인 필요]이므로 10 V를 판정 기준으로 단정하지 않음.
- [ ] 보드에 v18d2 가 올라가 있나(`~/flash_log.txt` 마지막 줄 `rear_v18d2.bin 30d3b65f0608fc88`, 부팅 배너 `(re)`).
- [ ] 카메라(RealSense) USB3 연결. 연결하면 **먼저** `taskset -c 0,1 ~/realsense-venv/bin/python ~/ESWContest_ws/pi5/pipeline/camera_check.py --warmup-frames 5` (10/25 책상 실측: 640 → 7.6 FPS, 416 → 14.5 FPS, 320 → 15.0 FPS; 기본은 640 유지).
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

### 9-4. CODEX1 책상 시험 후 추가 확인
- [ ] CODEX1_REPORT.md의 로컬 회귀 결과와 실제 장치 미검증 항목을 구분했나?
- [ ] 파이5 직접 터미널에서 CODEX1_LOCAL_COMMANDS.md의 읽기 전용 점검을 실행했나?
- [ ] 실제 카메라 FPS/YOLO p50·p90, ESP32 양방향 무선, 앞차 경고, 실제 부저 OFF를 확인했나?
- [ ] SIGKILL은 종료 처리 실행이 불가능하므로 실제 GPIO OFF를 보장한다고 표현하지 않음. 별도 검증 필요.

## 10. 책상 단계별 시험 (레이더 앞에서 금속판/몸을 움직이기) — 2~5분
시작 명령(카메라는 사용자가 직접 켬, 부저 mock):
```
cd ~/ESWContest_ws
CAMERA=1 TRANSPORT=udp BIG=1 BUZZER=mock DURATION=240 tools/run_car_test.sh
```
(`TRANSPORT=esp-now` 는 ESP32 에 BSM 브리지 펌웨어를 올린 뒤에만 의미가 있습니다. 10/25 확인: 이 파이의 ESP32 는 `esp_now_ping_test` 펌웨어(`TX=… | RX none` 출력)였습니다.)
| 단계 | 시간 | 하는 일 | 기대하는 것 |
|---|---|---|---|
| 1 | 30초 | 레이더 앞에 아무것도 없게 | 레벨 0, 타깃 없음(또는 책상 근거리 물체) |
| 2 | 30초 | 정면 2~3 m 에서 금속판을 **천천히** 다가오게 | 레벨 0→1 가능. 속도 모름이면 부저는 안 울림(표시만) |
| 3 | 20초 | **빠르게** 다가오게 | 레벨 2 가능(속도 모름이면 표시만) |
| 4 | 20초 | 치우기 | 레벨 0으로 복귀 |
끝나면 `Ctrl+C`. 보기: `logs/car_test/<날짜>/drive_log.csv` 의 `level`, `tgt_d_m`, `radar_down`, `power_suspect`, `camera_alive`, `camera_confirmed`. 레이더 전원이 불안정한 동안 `radar_down=1` 구간(Safe/0xFF)은 전원 영향일 수 있습니다.

### 실제 부저 종료 시험 (사용자가 직접, 3초)
```
cd ~/ESWContest_ws/pi5/pipeline
python3 -I buzzer_bench.py --really-drive-gpio26 --yes --type active --pattern --seconds 3      # 정상 종료
python3 -I buzzer_bench.py --really-drive-gpio26 --yes --type active --pattern --seconds 5      # 도중에 Ctrl+C → 바로 조용해지는지 듣기
```
`kill -9` (SIGKILL) 는 프로그램이 정리할 기회가 없어 부저가 켜진 채 멈출 수 있습니다 [미시험, OFF 보장 불가].

### 10-1. 카메라 포함 3분 실측(10/25 S14) — 이 숫자를 기대값으로 쓰지 마세요(책상, 단계 일부 미수행)
- 카메라 켠 상태: 평가 20 Hz 유지(간격 최대 0.29 s), 0x10/0x11/0x13 수신률은 카메라 없을 때와 동일, 카메라 약 6.9 FPS(NCNN 640, 코어 0-1 사용률 약 80%), 온도 최대 61.5 °C, throttled 0x0, camera_alive 97%.
- 카메라가 안 켜졌거나 멈춰도 레이더 단독으로 계속 도는 것은 `camera_alive`/`camera_confirmed` 열로 확인(카메라 끊김 구간은 이번 로그에 없음 → 가짜 입력 시험으로만 확인됨).
- **STM32 가 시험 중 재시작될 수 있음(10/25 17:28:34, 4.9 s)**: 로그에서 `level=255`(95행) + `stm_status` 빈칸 + 가동시간 역행. 이때 하트비트 reset_cause=1. 일어나면 시각을 적어 두고 ST-Link 디버그 텍스트(RESET CAUSE)를 확인하세요.

## 11. 실차 당일 체크리스트 (S17) — 위에서 아래로, 한 줄씩 확인
**출발 전 (주차 상태, 안전한 장소에서)**
- [ ] 안전한 장소: 사람·차가 없는 넓은 주차장/폐쇄 구간. **운전자는 화면을 만지지 않는다.** 실행·종료·기록은 조수석 사람이 한다.
- [ ] Pi5 전원: **5 V / 5 A PD 어댑터(또는 차량용 PD 충전기)** 사용. 연결 후 `vcgencmd get_throttled` 가 `0x0` 인지 확인. `0x0` 이 아니면 전원부터 고치기(저전압이면 시험 중단).
- [ ] 레이더 전원: 차량 12 V 직결(인라인 퓨즈) + 레이더/MCP2515/STM32/Pi **GND 공통**, 선 끝 절연. 멀티미터로 레이더 단자 전압 기록. 직결했으면 실행할 때 `RADAR_POWER=direct12v`.
- [ ] 모니터: 조수석 쪽에 고정(큰 글씨 화면 `BIG=1` 이 보이게). 밝기 최대, 햇빛 가림.
- [ ] 카메라: RealSense 연결, 렌즈 청결. 카메라는 보조(없어도 레이더 단독으로 동작).
- [ ] ESP32: 뒷차·앞차 모두 **같은 브리지 파일**, 안테나 연결, 앞차 켜서 `후방차=LIVE` 확인.
- [ ] **GPS 고정 확인**(하늘이 보이는 곳에 안테나): `timeout 5 gpspipe -w -n 10 | grep TPV` 에서 `"mode":2` 이상. 고정 안 되면 속도 없음 → **부저가 안 울림(표시만)**.
- [ ] OBD 가 연결되면 속도 출처가 OBD 로 잡히는지 `drive_log.csv` 의 `speed_src` 로 확인.
- [ ] 부저: `BUZZER=real` 로 실행(능동형, Pi5 물리 37번). 소리가 나는지 시작 직후 확인.
- [ ] **책상용 옵션 금지**: `--buzz-without-speed`, `--force-level2-*`, `--test-speed-kmh` 는 실차에서 절대 넣지 않는다(실행 명령에 이 글자가 있는지 눈으로 확인).
- [ ] 로그 기록기: ST-Link 디버그 텍스트를 읽기 전용으로 기록 `python3 ~/ESWContest_ws/tools/acm_logger.py <폴더>/acm_debug.log 3600 &` (STM32 재시작 원인 텍스트가 남음).
**실행**
```
cd ~/ESWContest_ws
CAMERA=1 TRANSPORT=esp-now BIG=1 BUZZER=real RADAR_POWER=direct12v tools/run_car_test.sh
```
**끝난 뒤 (로그 회수)**
- [ ] `python3 ~/ESWContest_ws/tools/post_run_report.py ~/ESWContest_ws/logs/car_test/<폴더>` 로 요약 한 번에 확인.
- [ ] 폴더 통째로 복사(USB): `cp -r ~/ESWContest_ws/logs/car_test/<폴더> /media/user/<USB>/` (+ `~/flash_log.txt`).
**중단 기준 — 하나라도 해당하면 차를 세우고 시험을 멈춘다 [PROPOSAL, 팀 합의 필요]**
1. 위험이 없는데 큰 화면이 `2` 이거나 부저가 울리는 일이 반복(오경보 2회 이상).
2. STM32 재시작 1회라도 발생(`level=255` + `stm_status` 빈칸) — 원인(ACM 로그의 RESET CAUSE) 확인 전에는 재개하지 않는다.
3. `vcgencmd get_throttled` 가 `0x0` 이 아님(저전압/과열), Pi 온도 80 °C 이상.
4. 레이더 끊김(`level=255`)이 10초 넘게 지속되거나 반복.
5. 앞차 화면이 `STALE` 인 상태가 10초 이상.
6. 선이 뜨겁거나 탄 냄새·연기, 커넥터 헐거움.
7. 운전자가 화면·노트북을 보느라 전방 주시를 못 할 때.

## 12. 부팅 자동 시작 — [PROPOSAL] 정리만 (이 세션에서는 서비스/설정을 건드리지 않았음)
- 지금은 사람이 `tools/run_car_test.sh` 를 직접 실행한다. 차에서 매번 노트북 없이 켜려면 systemd 사용자 서비스가 후보이나, **실차 검증 전에는 권장하지 않음**(오작동이 부저/전송으로 이어질 수 있음).
- 후보 구성(설치 안 함): `~/.config/systemd/user/aiv-pipeline.service` — `ExecStart=/home/user/ESWContest_ws/tools/run_car_test.sh`(BUZZER=mock 기본, `Environment=CAMERA=auto TRANSPORT=esp-now`), `Restart=on-failure`, `RestartSec=5`, `After=gpsd.socket`. 부저를 켜는 `BUZZER=real` 은 서비스에 넣지 말고 사람이 정할 것.
- 필요한 결정: 어느 단계에서 자동 시작을 허용할지, 화면(BIG)을 자동으로 띄울 방법(자동 로그인 터미널), 로그 폴더 용량 관리.

