# VISION_INTEGRATION — 최민준 비전 패키지 통합 준비 (패키지 없이 가능한 부분만 완료)

**상태 (S6-a 갱신):** 패키지를 찾아 `pi5/vision_lead/` 에 풀고 읽기 전용으로 분석했습니다 (§6). S5 때의 "경로 자리표시자/패키지 없음" 서술은 §2의 10/09 코드 사실과 §1 규칙을 제외하고 §6 이 대체합니다. 새 패키지는 `pi5/vision_lead/vehicle_algorithm/` 에만 있습니다(기존 파일 덮어쓰기 없음, 코드 수정 없음).

## 1. 설계 규칙 (지시서 기준)
| 항목 | 결정 |
|---|---|
| UART 소유자 | **`live_receiver`/`pipeline_main` 하나뿐.** 비전 쪽 `radar_receiver.RadarReceiver` 는 통합에서 사용하지 않음 |
| 카메라/YOLO | **별도 프로세스, 코어 0–1.** 파이프라인(수신+brain+V2V)은 **코어 2–3** (팀 규칙 3, `docs/handoff/RULES.md`) |
| 카메라 → 파이프라인 | 카메라 프로세스는 `detections(bbox, class, camera_mono)` 만 로컬로 발행(소켓/파일). **연관(`associate_candidates`)은 파이프라인에서 수행** |
| 카메라 없음/지연 | 카메라가 없거나 마지막 검출이 **300 ms 초과**여도 위험 판단은 **레이더만으로** 동작. `camera_confirmed` 는 보조 플래그(정지 물체 억제 해제에만 사용, `danger.assess(lifted=...)`) |
| 규칙 충돌 | 규칙 1: STM32 판단과 Pi5 역할 분리 / 규칙 2: Pi5가 죽어도 STM32 로컬 경고 유지 — 비전은 어떤 경우에도 안전 경로에 들어가지 않음 |

## 2. Pi5에 이미 있는 10/09 코드에서 확인한 사실 (읽기만)
- `radar_receiver.py`: 기본 포트가 **CP2102** (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-if00-port0`) 이고 `serial.Serial(..., exclusive=True)` 로 연다. 후방차의 STM32 링크는 `/dev/ttyAMA2` 이므로 기본값을 쓰면 **엉뚱한 포트(현재 CP2102 = ESP32 브리지일 가능성 [가설])를 독점**할 수 있음. 통합에서는 쓰지 않음. `main.py` 도 이 기본 포트를 import 함.
- `uart_status_receiver.py` 도 `exclusive=True` — 다른 프로세스가 같은 포트를 읽지 못하게 막는 용도이므로 `live_receiver` 와 동시 실행 금지.
- `camera_detector.py`: `~/yolov8n_ncnn_model` 폴더가 **있으면 NCNN(640) 자동 사용**, 없으면 `yolov8n.pt`(416). **폴더는 존재함** (`metadata.yaml`, `model.ncnn.param`, `model.ncnn.bin`).
- `fusion.py`: `RadarObject(object_id, forward_m, left_m, relative_mps, measured_rx_monotonic, stm32_ms)`, `associate_candidates(detections, radar_objects, shape, now_mono, max_age_s=0.15)`. 카메라 외부파라미터는 **가정값**(레이더 뒤 1.5 m, 위 1.2 m) — 연관은 후보 수준.
- 코어 분리 설정(`sched_setaffinity`/`taskset`)은 이 10/09 코드 어디에도 **없음** — 실행 시 `taskset -c 0,1` / `taskset -c 2,3` 로 해야 함.
- 새 패키지의 변경점(10/09 이후: lead 선택, hold, pitch, NCNN)과 "RCS unsigned 읽기" 문제는 **패키지를 못 봐서 확인하지 못함** (아래 §5 에서 우리 디코더는 RCS 를 signed int8 × 0.5 로 읽음을 명시).

## 3. 만든 것
- `pi5/pipeline/vision_adapter.py` (+selftest): 우리 `Obj(id, long_m, lat_m, vrel_ms, rcs, dyn)` ↔ `fusion.RadarObject`. fusion.py 는 수정하지 않고 import 만 함. `RadarObject` 에 새 **선택** 필드가 생겨도 동작하고, 새 **필수** 필드가 생기면 명확한 오류로 알려 줌.
  selftest: 변환 후 기존 `associate_candidates` 에 넣어 투영(1점)과 후보 연관(1건)까지 동작 확인.
- `target_select.py` 기본 규칙을 지시서의 `fusion.select_lead_vehicle` 숫자에 맞춤 (constants.py): **앞쪽 1–100 m, |lat| ≤ 1.75 m, |vrel| ≤ 60 m/s**, 히스테리시스 없음. 이전 S4 규칙(진입 1.5 m/이탈 2.0 m, |vrel| ≤ 100)은 `TGT_USE_HYSTERESIS=True` 옵션으로 남김.
  **주의:** 이 숫자는 지시서에서 받은 값이고, 실제 `select_lead_vehicle` 코드는 아직 못 봤음 → 새 패키지가 오면 코드와 대조해야 두 정의가 어긋나지 않음. 그 밖의 규칙(3/5 사이클 확인, 0.3 s hold, id 유지)은 우리 쪽 추가분.
  재생 영향(S5 비교, 변형 D→E): 위험 시간은 0.3→0.2 %, 0.6→1.0 %, 0.5→0.5 % 로 거의 같고 안전/주의 비율이 소폭 바뀜(자세한 표는 `s4_replay.py --compare`).

## 4. 레이더 값 해석 (우리 쪽 근거, 비전 쪽과 맞출 것)
- `vrel` < 0 = 접근 (정지 물체의 vrel ≈ −자기속도, 로그 검증). `lat` 좌우 부호는 **미검증**이라 |lat| 사용.
- `dyn`: 1 = 정지(지면 고정 89 %), 0 = 이동(99 % 앞에서 움직임), 2 = 마주 오는 차로 보이나 72 % 뿐, **5 는 표준 정의("정지 계열")와 달리 90 % 가 이동 물체처럼 행동**, 3/4 는 표본 부족. 정지 억제에는 dyn=1 만 사용.
- 객체 `vrel` 에 정확히 −128.0/+127.75 무효값 패턴이 있어 제외(|vrel| ≤ 60 이면 자동 제외됨).
- RCS: STM32 는 `int8(rcs_dBsm × 2)` 로 보냄. 우리 디코더/brain 은 **signed** 로 읽음(`aiv_uart_rx.decode`: `sb(rcs)/2`). 비전 쪽 디코더가 unsigned 로 읽으면 음수 RCS(−5 dBsm 이상만 통과)가 큰 양수로 보임 → 새 패키지 확인 항목.

## 5. (S6 에서 수행 완료, 결과는 §6) 받으면 할 일 체크리스트
1. 압축 해제: `~/ESWContest_ws/pi5/vision_lead/` (새 폴더, `unzip -n`). 홈의 `main.py`, `fusion.py`, `camera_detector*.py`, `radar_receiver.py`, `tracking_diagnostics.py` 와 `diff -u` → 10/09 이후 변경(lead 선택, hold, pitch, NCNN) 요약.
2. 읽기만 하고 옵션/의존성 파악(코드 수정 금지). `select_lead_vehicle` 의 숫자를 `constants.py` 와 대조.
3. 재생 검증: Pi5에 mp4 가 있으면(`logs/vehicle_test_logs/drive_*/camera_*.mp4`, 60초 단위) 10초 구간에서 "레이더 단독 선택 vs 최민준 lead 선택이 다른 구간과 이유", `camera_confirmed` 비율. mp4 가 없으면 건너뜀.
4. 발견 항목 목록 작성(RCS unsigned 읽기 등).

## 6. S6-a: 패키지 분석 결과
**패키지:** `~/Downloads/39b3e19b-TalkFile_vehicle_algorithm_lead_hold.zip` (5.99 MB, 항목 17개; 같은 이름 후보는 이것 하나뿐). 압축 안 경로 이탈(`..`, 절대경로)·심볼릭 링크 없음 → `unzip -n` 으로 `pi5/vision_lead/` 에 풂. 안의 `vehicle_algorithm_pi5_additions.zip` 은 **더 오래된(리드 이전) `main.py`/`radar_receiver.py` + 동일 `calibration.py`** 라서 무시. 패키지 파일 날짜 10/16 16:40–17:31.

**Pi5 홈(`~/vehicle_algorithm`, 10/09)과의 diff**
| 파일 | 결과 |
|---|---|
| `camera_detector.py`, `radar_receiver.py`, `calibration.py`, `record_camera.py`, `uart_status_receiver.py` | **동일** → **NCNN 은 10/09 부터 이미 있음**(패키지에서 새로 바뀐 것 아님). `~/yolov8n_ncnn_model` 폴더 **존재**(`model.ncnn.param/bin`, `metadata.yaml`) |
| `fusion.py` (5,105 → 8,206 B) | 투영에 **pitch**: `project(..., pitch_deg)`, `associate_candidates(..., pitch_deg)` (`v = CY + FY·tan(atan2(1.2, fwd) + pitch)`); 매칭 안 된 레이더 점은 그리지 않음; **새 `select_lead_vehicle()`**, `draw_lead_vehicle()`. `RadarObject`·`RadarPacketDecoder` 는 그대로(필드 6개 동일) |
| `main.py` (6,391 → 8,915 B) | 옵션 `--lead-lane-half-width 1.75`, `--lead-hold-frames 5`(표시 전용), `--pitch-deg 4.0`(**시험값, 보정 아님**); IoU 트래커, `lead_decisions_*.csv` 기록; `--record` 필수(진단 프로그램). 여전히 `RadarReceiver(DEFAULT_PORT=CP2102, exclusive=True)` 사용 |
| `tracking_diagnostics.py` (**새 파일**, 홈에 없음) | `IoUTracker`(탐지 박스 ID), `lead_decisions()`(YOLO 차량마다 이유: NO_RADAR_MATCH/TOO_NEAR/TOO_FAR/OUTSIDE_CORRIDOR/SPEED_OUTLIER/SELECTED/ELIGIBLE_NOT_NEAREST), **`LeadBoxHold`(lead hold)** = 마지막 lead 박스를 **N 처리 프레임(기본 5 ≈ 15 fps 에서 0.33 s)** 동안 **화면에만** 유지. "stale, 안전 로직에 쓰면 안 됨" 이라고 명시 |
→ 10/09 이후 달라진 것은 **lead 선택(`select_lead_vehicle`), hold(표시 전용), pitch(시험값 +4°), 진단 트래커/CSV** 뿐.

**`select_lead_vehicle` 실제 숫자 vs 우리 기본값 (constants.py)**
| 항목 | 패키지 코드 | `target_select.py` 기본 | 일치 |
|---|---|---|---|
| 전방 최소 | `min_forward_m=1.0` | `TGT_MIN_LONG_M=1.0` | 같음 |
| 전방 최대 | `max_forward_m=100.0` | `TGT_MAX_LONG_M=100.0` | 같음 |
| 횡방향 | `lane_half_width_m=1.75`, `abs(left_m)` | `TGT_LANE_LAT_M=1.75`, `abs(lat)` | 같음 |
| 상대속도 | `max_abs_relative_mps=60.0` | `TGT_VREL_VALID_MAX_MPS=60.0` | 같음 |
**숫자는 모두 같아서 constants.py 변경 없음.** 동작 차이(숫자가 아닌 정의): 패키지는 **카메라(YOLO)와 1:1 매칭된 후보 중** 가장 가까운 것을 매 프레임 독립적으로 고름(시간적 확인 없음, 카메라 없으면 lead 없음); 우리는 **레이더 단독**으로 3-of-5 확인 + 0.3 s hold(값 유지) + id 고정으로 고르고 카메라는 `camera_confirmed` 보조 플래그. 패키지의 hold 는 표시용 프레임 수.

**어댑터 재시험:** `vision_adapter.py` 를 패키지의 실제 `fusion.RadarObject` 로 `VISION_FUSION_DIR=pi5/vision_lead/vehicle_algorithm` 지정해 재시험 → 통과 (변환, `associate_candidates(pitch_deg=4)`, `select_lead_vehicle` 가 >100 m / |lat|>1.75 / |vrel|>60 객체를 거부).

**재생 비교 (`lead_replay.py`, mp4 가 Pi5 에 있음)** drive_20261016_152211, 영상 10초 구간 2곳, 2프레임마다(150프레임), NCNN 640 conf 0.25, 패키지와 같은 클래스/pitch 4°, 프레임 시각은 `camera_ts.csv`, 레이더는 프레임별 수신 시각 복원본:
| | 구간 A (세그먼트 0, 5–15 s) | 구간 B (세그먼트 3, 10–20 s) |
|---|---|---|
| 비전 lead 있는 프레임 / 우리 레이더 단독 타깃 있는 프레임 / 둘 다 | 126 / 138 / 114 | **0** / 77 / 0 |
| 같은 객체 | 61 % | (둘 다 없음 49 %) |
| 우리만 있음: YOLO 매칭 없는 레이더 단독 물체 | 14 % | 21 % |
| 서로 다름: 우리가 더 가까운데 YOLO 매칭 없음 | 10 % | – |
| 비전만 있음(우리 선택기는 아직 3-of-5 미확인/놓침) | 8 % | – |
| 우리만 있음: 레이더 hold(0.3 s) | 2 % | 21 % |
| 우리만 있음: YOLO 가 차량을 못 찾음 | – | 9 % |
| **camera_confirmed**(우리 타깃의 레이더 id 가 그 프레임에서 YOLO 매칭됨) | **73 %** (101/138) | **0 %** (0/77) |
구간 B 의 프레임 1장(t=10.7 s)을 직접 그려 확인: **우리 차선 정면은 비어 있고**, YOLO 박스는 왼쪽 옆 차선의 트럭/승용차에 있으며, 우리 쪽 레이더 단독 타깃은 **91 m 앞 도로 끝/가드레일 부근의 지면 고정 물체**였음 → 이 구간에서 비전 lead 가 "없음"인 것이 맞고, 레이더 단독 타깃은 가짜 후보(그리고 지면 고정이라 위험은 주의까지만 제한됨). 구간 A 의 14+10 % 도 같은 종류(가까운 가드레일/구조물)일 수 있으나 **영상으로 일일이 확인하지는 않았음 [가설]**. 옆 차선 차량(|lat| 2–3.5 m)은 이번 S6 의 옆 차선 감시 대상.
**두 구간 표본일 뿐이라 비율을 일반화하지 말 것** (73 % ↔ 0 %).

**발견한 문제/주의**
1. `fusion.RadarPacketDecoder` 는 `rcs`, `dyn` 을 **unsigned `B`** 로 읽고 버림(`'<IBhhhBBB'`). 지금은 쓰지 않아 무해하지만 RCS 를 쓰는 순간 틀림(펌웨어는 `int8(rcs_dBsm×2)`, 음수 가능). 우리 디코더는 signed.
2. `main.py`/`radar_receiver.py` 의 기본 포트가 **CP2102** + `exclusive=True` (변경 없음) → 통합/차량에서 기본값으로 실행 금지.
3. 패키지의 lead 는 **카메라가 죽으면 lead 없음**. 안전 판단은 레이더 단독이어야 하므로(§1) 우리 선택기가 맞는 방향이지만, 레이더 단독은 가드레일/먼 구조물 같은 **가짜 후보**를 만듦(구간 B).
4. `--pitch-deg 4.0` 은 시험값(부호/크기 미보정), 카메라-레이더 외부 파라미터도 가정값(레이더 뒤 1.5 m, 위 1.2 m) → 매칭은 후보 수준. 옆 차선 차량도 박스 근처에 투영됨(이미지에서 확인).
5. `LeadBoxHold` 는 **프레임 수** 기준(처리 fps 에 따라 시간이 달라짐)이고 표시 전용 — 우리 hold(0.3 s, 값 유지)와 의미가 다르므로 섞지 말 것.
6. 패키지 `README_LEAD_KR.md` 가 말하듯 "진짜 차선 검출/추적이 없으므로 내 차선 차량이라고 확정할 수 없음". 안전 제어/긴급제동에 사용 금지.
