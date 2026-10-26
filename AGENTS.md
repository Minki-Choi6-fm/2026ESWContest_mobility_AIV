# AGENTS.md — AIV 팀 (제24회 임베디드SW경진대회) 인수인계 지침

> 이 파일은 `~/ESWContest_ws/AGENTS.md` 에 두면 Codex가 자동으로 읽습니다.
> 작성: Claude(대화 세션), 2026-10-25 15:30 기준. **이 문서는 대화 기억으로 쓴 것이라 틀릴 수 있습니다.
> 디스크의 파일(코드·docs/handoff/*.md)과 다르면 항상 파일이 맞습니다.** 틀린 곳을 발견하면 이 파일을 고쳐 주세요.

## 0. 사용자와 일하는 방식
- 사용자: 최민기(AIV 팀장, STM32 담당). **한국어, 존댓말, 쉬운 말.** 전문적 설명은 사용자가 요청할 때만.
- 짧게 답하고, 반복해서 묻지 않기. 확인 안 된 것은 "됨"이라고 쓰지 않고 `[가설]` `[PROPOSAL]` `[확인 필요]`로 표시.
- 사실은 두세 번 확인(파일·로그·명령 출력). 추측을 사실처럼 쓰지 않기. 틀렸으면 바로 정정하고 기록.
- 이 환경은 사용자가 파이5에서 직접 파일을 다룹니다. 큰 변경 전에 백업(`.bak_날짜_s번호`).

## 1. 프로젝트 한 줄
상용차(뒷차, 큰 차) ↔ 승용차(앞차) 사이 **V2V 안전거리 경고**. 뒷차가 앞차와의 거리·상대속도로 급정거 위험을 판단하고, 앞차에 V2V(BSM)로 경고를 보내며, 자기 차에도 알린다.

### 하드웨어
- **뒷차**: Raspberry Pi 5 + NUCLEO-F446RE(STM32) + MR76 레이더 + MCP2515(CAN) + RealSense D435i(YOLOv8n, 보조용) + ESP32(ESP-NOW 브리지) + 부저.
- **앞차**: Raspberry Pi 4 + STM32 + OLED + ESP32 + 부저. (앞차 코드는 `~/Downloads/v2v.zip`, 풀어 둔 곳 `~/front_test/v2v`. **원본 수정 금지**.)
- 연결: STM32 UART1(PA9/PA10) 115200 8N1 ↔ 파이5 `/dev/ttyAMA2`(GPIO4/5, 물리 7/29). ST-Link VCP `/dev/ttyACM0`에 STM32 디버그 텍스트(RADSTAT, OLEDSTAT, `RADAR TIMEOUT`/`RECOVERED`, 부팅 배너). GPS는 gpsd(`/dev/ttyAMA0`). 부저: 파이5 물리 37번(BCM26), GND 34번. 앞차 파이4도 같은 물리 37번(GND는 다른 핀). **부저는 active형**(계속 울림 확인됨).
- UART 프레임: `STX 0x02 | TYPE | LEN | PAYLOAD | XOR | ETX 0x03`. STM32→파이 0x10~0x14, 파이→STM32 0x20/0x21/0x22. **자차 속도(0x22)는 서비스 `aiv-ego-speed`(`ego_speed_tx.py`, PID 648)가 보냄.**

## 2. 먼저 읽을 것 (순서대로)
`ls ~/ESWContest_ws/docs/handoff/` 로 목록을 보고 아래부터 읽으세요. 이 폴더의 `.md`는 모두 읽을 가치가 있습니다.
1. `PROJECT_STATE.md` — **가장 중요. 항목이 번호순(63번 이상)으로 쌓인 경과 기록.** 맨 끝부터 최근 상태.
2. `S12_INTEGRATION_NOTES.md`, `CAR_TEST_RUNBOOK.md`, `CAR_TEST_RUNBOOK_S12_SECTION.md` — 실차 실행 순서.
3. `IMPLEMENTATION_STATUS.md` — 구간별 "하드웨어 확인 / 가짜 입력 / 미검증" 표.
4. `FIRMWARE_ON_BOARD.md`, `FIRMWARE_V18D2_PROPOSAL.md`, `FLASH_PROCEDURE_V18D2.md`, `FIRMWARE_BOOT_BANNER.md` — 보드에 올라간 펌웨어.
5. `SAFE_GAP_EVENTS.md`, `RADAR_RX_NOTES.md` — 레이더 Safe Mode/끊김/수신 손실 분석.
6. `FRONT_COMPAT_TEST.md`, `BSM_STATUS_BITS.md`, `PROTOCOLS.md`, `OUTPUT_POLICY.md`, `SERIAL_WRITE_OPTIONS.md`, `BUZZER_BENCH.md`, `DESK_REHEARSAL.md`.
코드: `~/ESWContest_ws/pi5/pipeline/`(pipeline_main.py, brain.py, danger.py, target_select.py, bsm_builder.py, v2v_adapter.py, vision_adapter.py, buzzer_out.py, big_console_sink.py 등), `tools/run_car_test.sh`, `firmware/stm32_rear/`.

## 3. 지켜야 할 규칙 (사용자가 정한 것)
- **STM32 펌웨어 빌드/플래시 금지, 보드 리셋 금지.** 펌웨어는 사용자가 직접 올립니다. v18e(TEST_ALL_CLASSES 0)는 보류.
- 실제 UART 쓰기 금지(가짜 pty에서만 시험). 정식 프레임을 쓰는 코드는 `--write-stm32` 같은 플래그 뒤에 기본 OFF.
- 수정 금지: `ego_speed_tx.py`, systemd 서비스, `/boot/.../config.txt`, gpsd 설정, 안전 상수(constants.py의 안전 관련 값은 사용자 합의 후에만).
- **실제 GPIO 부저 구동은 사용자가 `BUZZER=real`(+`--really-drive-gpio26`)을 줄 때만.** 실제 카메라 열기/YOLO 구동도 사용자 요청 시에만.
- GitHub push 금지(사용자가 말할 때까지). 원본 SD 이미지 백업 권장.
- 앞차 코드(`~/front_test/v2v`) 수정 금지. 호환 시험은 그 폴더를 읽고 따로 하는 시험 코드로.

## 4. 현재 상태 (2026-10-25 오후)
### 확인된 것
- 펌웨어 v18d2(`~/rear_v18d2.bin`, sha256 앞 16자 `30d3b65f0608fc88`)가 보드에 올라감: 부팅 배너 `(re)`, `RADAR_TIMEOUT_MS=1000`, RADSTAT에 `txdrop=`. `~/flash_log.txt`에 기록. 롤백 `~/rear_v18c.bin`(sha 앞 16자 `d47f01a6f976840f`).
- 레이더→STM32→파이5 프레임 수신·해석은 실제 하드웨어로 확인됨.
- 부저(active, 물리 37번) 파이5/파이4 모두 울림 확인.
- 앞차 코드와의 BSM 호환은 **루프백으로** 11/11 통과(위험 ON 4 ms, 해제 약 1.08 s). 비트 5·6(SPEED_SRC_GPS, SPEED_STALE)은 앞차가 미사용이라 충돌 없음(팀 합의는 아직).
- 가짜 입력 통합 시험: integration_test 16/16, ESP-NOW 브리지 모델 5/5 — **단, 그 뒤에 pipeline_main.py / buzzer_out.py / constants.py 를 고쳤으므로 회귀 결과는 다시 확인해야 함.**
### 알려진 문제
- **레이더 전원 불안정**: 레이더가 파이5 GPIO 5V → XL6009(승압) → 12V 로 전원을 받았고, 사용자가 멀티미터로 XL6009 입력 4.5 V(주기적으로 3 V대), 출력 12.1 V(10 V대로 처짐)를 측정. 레이더가 약 5.5초 주기로 약 1.5초 끊기고 Safe Mode(0xFF)가 반복됨. **원인은 전원 처짐일 가능성이 크지만 전원을 고친 뒤 재측정한 적은 없음 [가설].** MR76 사양표: 주기 60 ms(초당 16~17개가 정상, 3~5개는 비정상), 12 V 2.5 W, CAN 500 kbit/s. 허용 전압 범위는 찾지 못함 [확인 필요].
- **XL6009는 승압 모듈**이라 차 12~14.4 V 입력을 받는 용도로 부적합. 계획: 차량 퓨즈(인라인 1~2 A)를 거친 12 V를 레이더에 직결하거나 입력 폭이 넓은 DC-DC 12 V 모듈 사용, 레이더/MCP2515/STM32/파이 **GND 공통**. 선 끝 절연(이번 사고처럼 옆 핀 단락 주의). 사용자 쪽 하드웨어 작업.
- **OLED 위험 알림은 현재 펌웨어로 불가**: 펌웨어는 0x22(자차 속도: u16 0.01 km/h, 출처 u8, AGE_MS u16)만 해석하고 0x20/0x21은 버림. 그래서 내 차 알림 = 파이5 부저 + 큰 글씨 콘솔(`big_console_sink.py`, 시험 안 됨). OLED 알림은 v18e 이후 펌웨어 변경 필요(사용자 결정). 0x20 payload 순서가 문서마다 다름(PROTOCOLS.md: SENDER_ID, LEVEL, cm / 앞차 코드: level, src, 0.1 m) → 합의 필요.
- 현재 빌드는 `TEST_ALL_CLASSES 1`(차량 클래스 필터 꺼짐). 노이즈 필터 RCS < −5 dBsm 때문에 pub < b. `ovf` = "넘침이 있었던 초 수"(몇 프레임 손실인지는 모름). exp 대비 b 3~7 % 부족은 CAN 칩 수신 쪽 가능성(상관만 확인).
- 사고: 10/25 선 교체 중 굵은 + 선이 옆 핀을 단락. 파이5는 현재 정상 부팅(사용자 확인). GPIO 뱅크 손상 가능성은 남아 있음 → 이상 증상이 있으면 GPIO4/5(UART2), GPIO14/15, GPIO26 루프백 확인 [확인 필요].

## 5. 미완료 작업 (S12 Claude Code 세션이 도구 오류로 중단한 곳)
Claude Code의 Bash 안전 검사기가 계속 오류를 내서 아래를 못 했습니다. 먼저 끝내세요.
1. `chmod +x tools/run_car_test.sh`, `bash -n tools/run_car_test.sh` (최근에 수정됨).
2. `CAR_TEST_RUNBOOK.md`, `PROJECT_STATE.md` 를 `.bak_`로 백업한 뒤 `CAR_TEST_RUNBOOK_S12_SECTION.md`(§9)와 S12 항목 반영. 부저 승인/정격 전류 문구가 다른 문서(HARDWARE_REAR.md, BUZZER_BENCH.md 등)에 남아 있으면 정리(부저는 이미 파이5에서 울려 확인됨).
3. 새/수정 코드 시험: `big_console_sink.py`(파이프로 렌더링), `pipeline_main.py --sinks csv,big`, `writer_mix_test.py`(두 프로세스가 같은 포트에 쓸 때 프레임 섞임, 가짜 pty), `front_compat_test.py` 4d(SEQ 시계 기반 시작).
4. 전체 회귀: 각 모듈 `--selftest`, `pipeline_main_test.py`, `buzzer_out_test.py`(부저 패턴을 0.1/0.1로 바꿈 → 시험이 값에 의존하는지 확인), `integration_test.py`, `front_compat_test.py`, 서비스(`aiv-ego-speed`, `gpsd`)와 `/dev/ttyAMA2` 상태.

## 6. 다음에 할 일 (우선순위)
1. 위 §5 마무리 → 회귀 전부 녹색 확인.
2. **레이더 전원을 차량 12 V 직결로 교체**(사용자 작업) → 멀티미터 MIN 유지 측정 → 60초 RADSTAT 캡처 재측정. 기대: a=16~17/s 지속, `RADAR TIMEOUT` 없음. 결과로 `RADAR_TIMEOUT_MS`(현재 1000)와 슬라이드 14(레이더 끊김, 전원 확정 전까지 "전원 문제 가능성, 개선 후 재측정 [확인 필요]" 표현만) 결정.
3. 양쪽 ESP32에 **앞차 쪽 브리지 파일**(코어 3.x, 채널 1, CRC) 업로드(사용자), 책상에서 실제 ESP-NOW 송수신 시험. 뒷차 쪽 `.ino`는 코어 3.x 호환으로 고쳤고 컴파일만 확인됨(업로드 안 함).
4. RealSense 연결 → `camera_check.py`(카메라 열림, 프레임률, YOLO 지연). 파이5에서 YOLOv8n 속도 확인. **카메라는 보조 플래그일 뿐**(camera_confirmed N=3 프레임)이며 실패하면 레이더 단독으로 계속해야 함.
5. 5분 책상 리허설(`desk_rehearsal.py`, `DESK_REHEARSAL.md`): 전원 교체 후 사용자가 "시작"이라고 할 때.
6. 실차 시험: `tools/run_car_test.sh`(변형 `CAMERA=1`, `TRANSPORT=esp-now`, `RADAR_POWER=direct12v`, `BUZZER=real`, `BIG=1`). 로그 `logs/car_test/<날짜>/drive_log.csv` 등으로 분석. 속도(OBD/GPS) 없으면 부저는 울리지 않고 표시만(기본). 주행 중 K(운전 제동 습관) 이벤트가 3개 이상 필요.
7. 정리할 것: 비트 5/6·0x20 순서 팀 합의, 후방 SEQ 시계 시작이 앞차에서 실제로 지연을 없애는지, 앞차 GpioBuzzer에 종료 시 OFF 장치 없음(뒷차는 safe_state 있음), 앞차 구성(Pi 부저 vs STM32 PA0) 확인, 원본 SD 이미지 백업, GitHub 정리(사용자 지시 후).
8. 앞차(Pi4)는 아직 실제 장비 시험이 없음. 파이4는 YOLO가 느리므로 레이더 단독이나 NCNN imgsz 320/416 고려(앞차 쪽 설계는 팀 확인 필요).
9. **발표 자료(PPT)는 별도 흐름**: 슬라이드 13(워치독 리셋 근거 부족), 14(레이더 끊김, 보류), 18(문구) 수정 남음. 팀 정보·YouTube/QR·GitHub 링크 채우기. 대회 일정은 사용자에게 확인(미확인) [확인 필요].

## 7. 사용자가 정해야 하는 것
- OLED 알림용 펌웨어 변경(v18e 이후)을 할지, 부저+콘솔만으로 갈지.
- 레이더 전원 방식(차량 12 V 직결 / DC-DC).
- 실차 시험 일정, 앞차 구성(Pi4 몇 대, 부저 위치), 비트 5/6·0x20 순서 합의.

## 8. 보고 방식
작업이 끝나면: ① 구간별 상태 표(하드웨어 확인 / 가짜 입력 / 미검증) ② 사용자가 직접 할 일 ③ 시험하지 않은 것 목록. 사용자가 실행한 결과 로그(RADSTAT 등)가 오면 사실과 가설을 분리해 해석하고, `PROJECT_STATE.md`에 번호를 이어 붙여 기록.

## 정정 — CODEX1 (2026-10-25)
- 작업 시작 시 이 AGENTS.md는 디스크에 없었습니다. 사용자가 첨부한 지침을 바탕으로 새로 저장했습니다.
- CODEX1 요청에서는 카메라/YOLO 구동과 사전 안내 후 실제 부저 3초 시험을 허용하셨습니다. ESP32 업로드는 여전히 사용자의 “업로드” 응답이 필요합니다. STM32 리셋/빌드/플래시와 실제 UART 쓰기는 금지입니다.
- 로컬 회귀: 통합 16/16, 파이프라인 11/11, 앞차 호환 12/12, 가짜 부저 19/19, 브리지 모델 5/5, 카메라 확인 13/13 통과. 상세 CODEX1_REPORT.md 참조.
- /sys USB 정보: D435i 연결 속도 480 Mbps(USB2), CP2102 1개 ttyUSB0, ST-Link ttyACM0. 실제 /dev 장치는 이 실행 환경에 노출되지 않아 센서 읽기/실제 무선/실부저는 미실행입니다.
- 전원 문제는 고치지 않았습니다. 실제 실차 준비 완료로 판정하지 않습니다.

- 후속 실제 수신 정정: 사용자 제공 실제 10초 수신(20261025_155938): 0x10=500/500, 0x11=100/100, 0x12=506, 0x13=200, 0x14=3. BAD len/chk/resync=0/0/0, 0x10·0x11 시계 기준 손실 0%. 상태 0x0B 500개, 레이더/IMU/summary/heartbeat 타임아웃 0회. OBD 타임아웃 2회, 속도 0xFF·GPS fix 0으로 자차 속도 없음(책상 환경). 수신기가 ego_speed_tx.py를 비읽기 점유자로 확인. 시작 백로그 1314832 B 제거. 0x12는 물체 프레임 50.6/s이며 레이더 CAN 상태 a=16~17/s와 다른 지표. 10초 동안 끊김이 없었다는 사실이며 전원 개선/장시간 안정성 증거는 아님.

- 실제 카메라 후속 결과: 사용자 직접 카메라 시험: taskset -c 0,1 및 realsense-venv로 D435I(S/N 047322070031) 실제 RGB 스트림·YOLO NCNN imgsz640 실행 성공. 설정 640x480@15, 15초 시험 92프레임, 보고 FPS 8.6, YOLO p50 113 ms/p90 134 ms/max 3869 ms, total p50 113 ms, 검출 평균 0.0. 제안 기준 FPS>=15/p90<=100 ms에 미달하여 WARN. 검출 0은 차량 확인 정확도나 고장 증거가 아님. 최대 지연 원인은 로그만으로 미확정(초기 준비 가능성은 가설). CPU 온도·사용률/현재 USB 링크 속도/실제 camera_confirmed 통합은 미측정. 카메라 보조 처리 가능하나 성능 최적화 및 통합 검증 필요.

- CODEX1 후속: 사용자 지시로 USB3 추가 조사 종료. 카메라 워밍업5 제외 실제640/416/320=7.58/14.51/15.02FPS, 기본640유지. 실제2분 통합완료(2397평가20Hz/BSM시리얼1196/Safe0/BAD0/레벨0만/confirmed0). 실제부저3초 소리 사용자확인 및 GPIO26읽기0. 실제무선/위험판정정확도 미검증. PROJECT_STATE71·72, CODEX1_REPORT 최신절 참조.

## 정정 — Claude S13 (2026-10-25 16:5x)
- Codex 변경을 diff 로 검증함(PROJECT_STATE 73~76). 회귀 숫자 재현. 안전 상수·서비스·ego_speed_tx.py·`~/front_test`·펌웨어 변경 없음.
- **이 파이의 ESP32 는 BSM 브리지가 아니라 `esp_now_ping_test` 펌웨어**로 확인됨(`TX=… | RX none` 출력). 지금까지의 esp-now 실행은 무선 송출 증거 아님. 업로드는 사용자의 '업로드' 응답 후에만.
- 한 번에 한 에이전트만 파일을 고친다(지금은 Claude).

