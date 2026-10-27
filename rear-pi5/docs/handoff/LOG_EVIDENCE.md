# LOG_EVIDENCE — 보고서용 로그 증거 (주행 로그 3개의 stm_debug.log, 읽기 전용 집계)

생성: `python3 -I pi5/pipeline/log_evidence.py` (로그 파일은 수정하지 않음). **측정값**과 **가설**을 구분했고, 파일에서 알 수 없는 것은 "no data/unknown" 으로 적었습니다.
대상은 `~/ESWContest_ws/logs` 아래 `stm_debug.log` 가 있는 주행 3개(10/16 14:15, 15:12, 15:22)뿐입니다(다른 폴더에는 `stm_debug.log` 없음).

**요약(측정)**
- OLEDSTAT: 3개 로그 모두 **0줄** → pushes/fails/retries/retry_ok/err 요약 불가. 이 로그들은 `FW ...` 배너와 OLEDSTAT 가 없는 **v18c 이전(v16/v17 계열) 펌웨어**로 기록됨(v16/v17 소스에는 둘 다 없고 v18c 에는 둘 다 있음; Pi5 의 v18 계열 .bin 파일 시각 10/16 17:43–17:53 이 로그 시각보다 뒤(복사 시각일 수 있음)). 정확한 버전(v16 vs v17)은 **미확인**.
- Safe Mode: 141541 에서 **754회 진입(65개 에피소드, 가장 긴 것 148–249 s, 212회)**, 151202 와 152211 은 0회. `RADAR TIMEOUT` 줄 수(754)와 `safe=` 증가 합(754)이 일치.
- IWDG(RESET_CAUSE=1): **3개 로그 모두 없음.** 151202 에서 448 s 에 리셋 1회(원인 `other/pin`), 그 부팅의 I2C 스캔(453 s)에서 "No I2C device found on I2C2".
- 새로 확인된 것: ① MCP2515 수신 버퍼 넘침이 레이더 객체가 많은 초(≥500/s)에 거의 항상 동반(222/223, 93/95, 51/51), 레이더가 알린 객체 대비 STM32 수신 부족 3.5–5.3 %. ② 펌웨어 `TEST_ALL_CLASSES 1` 로 차량 클래스 필터가 꺼져 있어 UART 로 나가는 0x12 가 차량 클래스 수의 2.8–4.7 배.

## 상세 출력

# Log evidence from 3 drive log(s) (read-only)

## drive_20261016_141541  (start 2026-10-16 14:15:41, duration 1594 s, 62108 debug lines, note: 후방차 직선 주행 테스트)
- firmware indicators: `FW ...` banner lines: 0; OLEDSTAT lines: 0; RADSTAT lines: 1602. Source check: v16/v17 print neither FW banner nor OLEDSTAT, v18c prints both → this log was recorded with firmware **older than v18c (v16/v17 class; exact version unknown)**.
- OLEDSTAT (pushes/fails/retries/retry_ok/err): **no lines in this log → nothing to summarise** (counter did not exist in this firmware)
- Safe Mode (RADSTAT `safe=` cumulative per second; `RADAR TIMEOUT -> SAFE MODE` lines: 754, `RADAR RECOVERED` lines: 754): **754 entries**, in 65 episode(s) (increments within 3 s merged)
  | episode | start s | end s | duration s | entries |
  |---|---|---|---|---|
  | 1 | 140 | 140 | 0 | 1 |
  | 2 | 145 | 145 | 0 | 2 |
  | 3 | 148 | 249 | 100 | 212 |
  | 4 | 253 | 253 | 0 | 2 |
  | 5 | 257 | 257 | 0 | 2 |
  | 6 | 261 | 261 | 0 | 2 |
  | 7 | 265 | 266 | 1 | 2 |
  | 8 | 270 | 274 | 4 | 4 |
  | 9 | 277 | 277 | 0 | 2 |
  | 10 | 281 | 282 | 1 | 2 |
  | 11 | 286 | 286 | 0 | 2 |
  | 12 | 290 | 291 | 1 | 2 |
  | ... 53 more episodes | | | | |
- CAN receive side: radar announced 465050 objects (RADSTAT `exp`), STM32 received 440583 (`b`) -> shortfall 24467 (5.3 %); MCP2515 RX-overflow seconds (`ovf` increase) 1105/1601 (69 %), and in the 223 seconds with >= 500 objects/s: 222 had an overflow
- UART publishing: `pub` 291018 vs vehicle-class objects `veh` 61602 -> published = 4.7 x vehicle-class count (firmware has `#define TEST_ALL_CLASSES 1` in v16, v17 and v18c: the class filter is OFF)
- CAN error counters: tec max 0, rec max 0, number of changes between consecutive seconds: 0; seconds with tec/rec != 0 (t, tec, rec): none; RX buffer overflow seconds `ovf`: 547 → 1652
- heartbeat RESET_CAUSE over time: 0s: power-on; IWDG (cause 1) seen: **no**; `RESET CAUSE` text lines: none
- coincidence of IWDG resets with OLED errors: **no reset and no OLED message in this log → nothing to match**

## drive_20261016_151202  (start 2026-10-16 15:12:02, duration 605 s, 26432 debug lines, note: 후방차 직선 주행 테스트)
- firmware indicators: `FW ...` banner lines: 0; OLEDSTAT lines: 0; RADSTAT lines: 597. Source check: v16/v17 print neither FW banner nor OLEDSTAT, v18c prints both → this log was recorded with firmware **older than v18c (v16/v17 class; exact version unknown)**.
- OLEDSTAT (pushes/fails/retries/retry_ok/err): **no lines in this log → nothing to summarise** (counter did not exist in this firmware)
- OLED/I2C text: 450s `--- I2C2 Scan Start ---`; 453s `No I2C device found on I2C2. Check wiring (SCL=PB10, SDA=PC12).`; 453s `--- I2C2 Scan End ---`
- Safe Mode (RADSTAT `safe=` cumulative per second; `RADAR TIMEOUT -> SAFE MODE` lines: 0, `RADAR RECOVERED` lines: 0): **0 entries**, in 0 episode(s) (increments within 3 s merged)
- CAN receive side: radar announced 222473 objects (RADSTAT `exp`), STM32 received 211482 (`b`) -> shortfall 10991 (4.9 %); MCP2515 RX-overflow seconds (`ovf` increase) 244/595 (41 %), and in the 95 seconds with >= 500 objects/s: 93 had an overflow
- UART publishing: `pub` 135356 vs vehicle-class objects `veh` 48579 -> published = 2.8 x vehicle-class count (firmware has `#define TEST_ALL_CLASSES 1` in v16, v17 and v18c: the class filter is OFF)
- CAN error counters: tec max 24, rec max 0, number of changes between consecutive seconds: 2; seconds with tec/rec != 0 (t, tec, rec): [(350, 24, 0)]; RX buffer overflow seconds `ovf`: 12 → 28
- heartbeat RESET_CAUSE over time: 0s: other/pin, 456s: other/pin; IWDG (cause 1) seen: **no**; `RESET CAUSE` text lines: 448s `RESET CAUSE: other/pin`
- coincidence with OLED messages: the only OLED/I2C message(s) at 453s sit inside the boot sequence of the reset at 448s (the boot-time I2C scan found no OLED); there is no OLEDSTAT, so OLED errors during normal running cannot be matched to resets [no data].

## drive_20261016_152211  (start 2026-10-16 15:22:11, duration 424 s, 18267 debug lines, note: 후방차 직선 주행 테스트)
- firmware indicators: `FW ...` banner lines: 0; OLEDSTAT lines: 0; RADSTAT lines: 426. Source check: v16/v17 print neither FW banner nor OLEDSTAT, v18c prints both → this log was recorded with firmware **older than v18c (v16/v17 class; exact version unknown)**.
- OLEDSTAT (pushes/fails/retries/retry_ok/err): **no lines in this log → nothing to summarise** (counter did not exist in this firmware)
- Safe Mode (RADSTAT `safe=` cumulative per second; `RADAR TIMEOUT -> SAFE MODE` lines: 0, `RADAR RECOVERED` lines: 0): **0 entries**, in 0 episode(s) (increments within 3 s merged)
- CAN receive side: radar announced 146071 objects (RADSTAT `exp`), STM32 received 140921 (`b`) -> shortfall 5150 (3.5 %); MCP2515 RX-overflow seconds (`ovf` increase) 155/425 (36 %), and in the 51 seconds with >= 500 objects/s: 51 had an overflow
- UART publishing: `pub` 97852 vs vehicle-class objects `veh` 27032 -> published = 3.6 x vehicle-class count (firmware has `#define TEST_ALL_CLASSES 1` in v16, v17 and v18c: the class filter is OFF)
- CAN error counters: tec max 0, rec max 0, number of changes between consecutive seconds: 0; seconds with tec/rec != 0 (t, tec, rec): none; RX buffer overflow seconds `ovf`: 29 → 185
- heartbeat RESET_CAUSE over time: 0s: other/pin; IWDG (cause 1) seen: **no**; `RESET CAUSE` text lines: none
- coincidence of IWDG resets with OLED errors: **no reset and no OLED message in this log → nothing to match**
