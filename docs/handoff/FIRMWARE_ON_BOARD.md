# FIRMWARE_ON_BOARD — 지금 NUCLEO 에 올라가 있는 펌웨어는 어느 버전인가 (S8-a, 읽기 전용 추적)

대상: STM32 담당(최민기). **보드에는 접속하지 않았습니다**(`st-info/st-flash/openocd` 실행 안 함, `/dev/ttyACM0`·`ttyAMA2` 새로 읽지 않음, 리셋 안 함). 이미 있는 파일/기록만 읽었습니다.
결론부터(캡처 전): **미확인.** → **캡처 후 갱신(맨 아래 §5): v18b 이후 계열(v18b / v18c / v18c-TEST / test2 중 하나)로 좁혀졌고, v15–v18 은 배제됨.** 기록으로는 "마지막으로 기록된 플래시 = rear_v17.bin" 이지만, 10/16 17:43–18:26 에 만들어진 v18 계열 `.bin` 5개가 홈에 있고 그것을 플래시한 기록은 남아 있지 않습니다.

## 1. 증거 (측정/관찰)
| 증거 | 내용 | 한계 |
|---|---|---|
| `~/.bash_history` (781줄, 10/24 16:20 저장) | `st-flash [--connect-under-reset] --reset write` 를 **rear_v15 → rear_v16_orig → rear_v16_fast → rear_v17.bin 순서**로 실행한 기록(10/12). **v18/v18b/v18c/test 를 쓴 명령은 없음.** 시각 정보 없음(HISTTIMEFORMAT 미사용) | 셸을 정상 종료하지 않으면 히스토리가 저장되지 않음 → 10/16 의 다른 터미널/IDE/에이전트 쉘 명령은 빠졌을 수 있음 |
| `~/.codex`, `~/.copilot`, `~/.vscode-server`, `~/.config/Code`, `~/.local/state` | `st-flash`, `rear_v1x`, `openocd`, `NOD_F446RE` 검색 결과 **흔적 없음** | 에이전트 로그가 남아 있는 범위만 |
| `journalctl` | **현재 부팅(10/24 16:20 이후)만 저장**(`--list-boots` 1개). 10/16 기록 없음. 현재 부팅에서 NUCLEO 가 `/media/user/NOD_F446RE` 로 마운트된 기록(16:20:41)만 있음 | 지속 저널 아님 |
| 마운트된 NUCLEO 볼륨 (`/media/user/NOD_F446RE`, 이름이 `NODE_` 가 아니라 `NOD_`) | 파일은 `DETAILS.TXT`, `MBED.HTM` 두 개뿐. **복사해 넣은 .bin 흔적 없음**(드래그 앤 드롭 플래시는 파일을 남기지 않음) | |
| 홈의 `.bin` | `~/rear_v18.bin` 10/16 17:43, `rear_v18b.bin` 17:51, `rear_v18c.bin` 17:53, `rear_v18c_test.bin` 18:21, `rear_v18c_test2.bin` 18:26 (홈에 직접 복사됨; Downloads 의 `rear_v15–v17` 은 10/12 별도) | 파일 시각 = 만든/복사한 시각이지 플래시 시각이 아님 |
| 파일 접근 시각 | `/` 가 **noatime** 으로 마운트되어 atime 이 모든 파일에서 mtime 과 같음 → **증거 아님** | |
| 10/16 주행 로그(14:15–15:29, `stm_debug.log`) | `FW` 배너·OLEDSTAT 없음 → **그 시각 보드는 v17 이하 계열**(v16/v17 소스에는 둘 다 없고 v18 이후 .bin 에는 있음). 이 로그들은 v18 계열 `.bin` 이 만들어지기 전(17:43)임 | 그 이후의 플래시는 이 로그로 알 수 없음 |
| UART 로 나가는 데이터 | `proto_send_obd`, `proto_task`, `build_sensor_status`, `obd_poll_task` 가 **v17 과 v18c 에서 동일** → 10/24 책상 측정(0x10 STATUS=0x0B, 0x14 약 3초 간격 등)으로는 v17/v18c 를 구분 **불가** | 구분하려면 디버그 텍스트(ttyACM0)가 필요 |
| `~/range_rear.log` (10/16 18:40) | ESP32 핑 시험(RSSI 출력)이며 STM32 와 무관 | |

## 2. 각 .bin 이 담고 있는 것 (바이너리 안 문자열 확인 — 빌드 없이 읽기만)
| 바이너리 | 크기 | sha256 (앞 16자) | 부팅 배너 | OLEDSTAT | 비고 |
|---|---|---|---|---|---|
| rear_v15 | 45,400 | `ba097c4ba1369d91` | 없음 | 없음 | RADSTAT 있음, `rd=` 없음 |
| rear_v16_orig | 45,732 | `337b430845ead007` | 없음 | 없음 | `rd=` 있음 |
| rear_v16_fast | 45,840 | `ea812aa944509be6` | 없음 | 없음 | |
| rear_v17 | 46,116 | `635e83eb4dd6c2db` | 없음 | 없음 | **기록된 마지막 플래시** |
| rear_v18 | 48,164 | `c315fb4c2ec386d3` | `FW v18 (OLED I2C guard)` | 없음 | |
| rear_v18b | 48,784 | `dcdfafb25f7fdba8` | `FW v18b (OLED I2C guard + diag)` | 있음 | |
| rear_v18c | 48,748 | `d47f01a6f976840f` | `FW v18c (OLED I2C guard + diag)` | 있음 | Downloads/`stm32_projects/_bins` 의 v18c 와 동일 해시 |
| rear_v18c_test | 48,132 | `87b47c0b169648c9` | `FW v18c-TEST (OLED column pattern)` | 있음 | **OLED 시험 패턴 빌드(평소 사용 금지)** |
| rear_v18c_test2 | 48,144 | `541699c963e44b8d` | `FW v18c-TEST (OLED column pattern)` | 있음 | 파일 시각상 **가장 마지막** 빌드 |
소스는 `main_v16.c`, `main_v17.c`(`firmware/stm32_rear/old/`), `main_v18c.c` 만 있어 **v18, v18b, test 빌드의 소스 대조는 못 함**(바이너리 문자열로만 구분). 팀 노트북 CubeIDE 프로젝트(`stm32_projects_latest.zip`, 10/24) 의 `main.c` 는 v18c(`FW v18c` 배너, OLEDSTAT 있음)이지만 **프로젝트 상태일 뿐 보드 상태가 아님.**

## 3. 결론 표
| 후보 | 근거 | 신뢰도 |
|---|---|---|
| **rear_v17** | 히스토리에 기록된 마지막 `st-flash write`; 10/16 14:15–15:29 주행 로그도 v17 이하와 일치 | 낮음~중간 — 10/16 17:43 이후의 플래시를 배제할 수 없음 |
| rear_v18 / v18b / v18c | 17:43–17:53 에 홈에 복사됨. "마지막으로 Pi5 에서 업로드했다"는 기억과 부합 가능. 플래시 기록 없음 | 낮음 |
| **rear_v18c_test / test2 (OLED 시험 패턴)** | 파일이 가장 최신(18:21, 18:26)이고 홈에 있음. 플래시했다면 보드는 **시험 패턴 빌드**(평소 동작용 아님) | 낮음 — 하지만 **가장 위험한 경우**이므로 먼저 배제할 것 |
| **미확인** | 위 기록만으로 확정 불가 | **현재의 정직한 결론** |

## 4. 확인 방법 (사용자가 직접; 이 문서는 안내만)
1. NUCLEO 의 **검은 RESET 버튼**을 누르면서 `/dev/ttyACM0`(ST-Link 가상 COM, 115200)의 **첫 줄들**을 읽습니다. 부팅 때 `RESET CAUSE: …` 다음 줄에 **`FW v18c ...` 같은 배너**가 나오면 버전이 확정됩니다. 배너가 **아예 없으면 v17 이하(v15–v17)**, `FW v18c-TEST (OLED column pattern)` 이면 **시험 패턴 빌드**입니다.
2. 저(Claude)가 대신 읽어 드릴 수 있습니다: **"지금 리셋 누른다"** 라고 답해 주시면 `/dev/ttyACM0` 를 **읽기 전용으로 15초만** 캡처합니다(포트에 쓰기·`stty` 변경·플래시/리셋 명령 없음; 리셋은 사용자가 버튼으로).
3. 확정 후 이 표를 갱신하고, 이후 업로드할 때는 **업로드 명령과 파일 sha256 을 `~/flash_log.txt` 에 남기기**를 권장합니다 (예: `echo "$(date -Is) st-flash write rear_v18c.bin $(sha256sum rear_v18c.bin | cut -c1-16)" >> ~/flash_log.txt`).

## 5. 사용자가 RESET 을 누른 15초 읽기 전용 캡처 결과 (2026-10-25)
`/dev/ttyACM0` 를 `O_RDONLY` 로 15초 읽음(포트에 쓰기·`stty`·플래시/리셋 명령 없음; 시작 전 점유 프로세스 없음 확인). 원본: `logs/ttyACM0_capture_20261025_reset.txt` (238줄).
| 관찰(측정) | 내용 |
|---|---|
| **OLEDSTAT 줄이 나옴** | `OLEDSTAT ok=1 pushes=… fails=0 retries=0 retry_ok=0 err=0 hal_err=0x0 t=…` — 형식이 v18c 소스의 `sprintf` 와 같음. 리셋 후 `pushes` 1 → 11 → 21 (5초에 +10 = 정상), `fails=0`. `OLEDSTAT` 문자열이 들어 있는 바이너리는 **v18b, v18c, v18c_test, v18c_test2** 뿐 (v15–v17, **v18 에는 없음**) → **v15–v18 배제** |
| 리셋 전 줄 | `OLEDSTAT … pushes=6391 … t=3200000` → 리셋 직전까지 약 53분(3,200 s) 가동, OLED 푸시 6,391회 실패 0. `RPIRX bytes=159250 ok=15925 bad=0` → **STM32 가 53분간 받은 Pi5 의 0x22 프레임 15,925개 중 불량 0** (Pi→STM32 방향 링크 건강; 약 5 Hz × 3,200 s 와 일치) |
| 리셋 후 부팅 | `BOOT2`, I2C 스캔 `Found I2C device at 0x3C`(**OLED 인식**), `MCP2515 OK`, `OBDLINK … OK`, `OBD no response x3 -> slow polling (3s)`(책상: OBD 없음) |
| **빠진 줄** | 소스상 `RESET CAUSE`, `FW v18…` 배너, `BOOT1` 이 `BOOT2` 보다 앞에 나와야 하는데 캡처에는 **`…SAFE MODE t=3202220BOOT2` 로 바로 이어짐**(이전 줄이 CRLF 없이 끊기고 BOOT2). 즉 **배너가 포함된 부팅 첫 줄들이 캡처에서 유실**. 원인은 모름 [가설: 리셋 직후 ST-Link VCP 가 앞 바이트를 놓침 / 사용자의 버튼과 캡처 시작의 시차]. 따라서 배너로는 **확정하지 못함** |
| v18b/v18c/TEST 구분 | 디버그 텍스트로는 구분 불가: 세 빌드는 배너와 **OLED 화면 문구**만 다름(`strings` 비교: v18b `AIV REAR MCP:%s %lu`; v18c `AIV REAR  MCP:%s` + `RADAR: %s`; TEST 는 열 패턴 + `FW v18c-TEST (OLED column pattern)`) |

**갱신된 결론 표**
| 후보 | 근거 | 신뢰도 |
|---|---|---|
| rear_v15–v17, rear_v18 | OLEDSTAT 가 캡처에 나옴 → 이 바이너리들은 OLEDSTAT 를 출력하지 않으므로 **배제** | 높음 (측정) |
| **rear_v18c** | 홈의 `rear_v18c.bin` 이 정상 동작용 최신 빌드. OLEDSTAT 형식 일치, OLED 정상(pushes 증가·fails 0), **사용자 육안 확인(OLED 평소 두 줄), 배너 미확인: OLED 에 `AIV REAR MCP:…` + `RADAR: …` 두 줄(§6)** | **중간~높음** (배너 미확인; v18b 와는 첫 줄 끝 숫자 유무로만 구분) |
| rear_v18b | 같은 OLEDSTAT 출력. 정상 빌드이지만 v18c 가 더 나중 | 낮음~중간 |
| rear_v18c_test / test2 (OLED 시험 패턴) | OLED 가 평소 두 줄로 보인다는 사용자 확인(§6)으로 **배제** | 높음 (사용자 관찰) |

**확정 방법(남은 것)**: ① **OLED 화면을 눈으로 확인** — 평소 문구(`AIV REAR  MCP:…` 와 `RADAR: …` 두 줄)면 v18c, `AIV REAR MCP:… <숫자>` 이면 v18b, 세로줄/열 패턴이면 TEST 빌드. ② 또는 **캡처를 먼저 시작한 뒤** 알려 드리는 순간에 RESET(앞 캡처는 버튼과 캡처 시작이 거의 동시여서 앞부분이 빠졌을 수 있음; 부팅 후 2초 뒤 `BOOT2` 가 나오므로 리셋 후 최소 6초 읽기).

## 6. 사용자 확인: OLED 화면 (2026-10-25)
사용자가 OLED 에 **평소 두 줄 — `AIV REAR MCP:…` 와 `RADAR: …` — 이 보이는 것이 현재 상태**라고 확인(사용자 보고, 사진/캡처 없음).
- **TEST 빌드(열 패턴)는 배제**: 시험 패턴 빌드였다면 이 두 줄이 아니라 열 패턴이 나옴.
- **v18c 가 가장 유력하지만 v18b 와 완전히 갈리지는 않음**: 바이너리 문자열 비교에서 `RADAR: %s` 는 v18b 와 v18c **양쪽에** 있고, 차이는 첫 줄뿐 — v18b 는 `AIV REAR MCP:%s %lu`(끝에 **숫자**가 붙음), v18c 는 `AIV REAR  MCP:%s`(**숫자 없음**, MCP 앞 공백 2칸). 사용자 보고만으로는 첫 줄 끝의 숫자 유무를 알 수 없음.
- 최종 결론: **rear_v18c (신뢰도 중간~높음; 첫 줄 끝에 숫자가 없다면 높음)**. v18b 가능성은 낮음. 확정 배너 `FW v18c …` 는 여전히 미확인(원하면 캡처를 먼저 시작한 뒤 "지금" 신호에 맞춰 RESET 을 눌러 재시도).

## 7. 표기 정정 (S9)
보드 펌웨어가 v18c 라는 것은 **"사용자 육안 확인, 배너 미확인"** 입니다(OLED 평소 두 줄 `AIV REAR MCP:…`, `RADAR:…`). 부팅 배너가 캡처에 없어 시리얼로는 확정하지 못했습니다(원인 조사: `FIRMWARE_BOOT_BANNER.md`). 현재 보드의 sha256 은 모름 → v18d2 적용 전 플래시 기록 필수(`FIRMWARE_V18D2_PROPOSAL.md` §5).

## 8. v18d2 플래시 결과 (2026-10-25, 사용자 보고 + 이 PC 확인)
| 항목 | 내용 | 출처 |
|---|---|---|
| 파일 | `~/rear_v18d2.bin` 49,012 B, sha256 앞 16자 `30d3b65f0608fc88` (`~/Downloads/rear_v18d2.bin` 과 바이트 동일) | 이 PC에서 확인 |
| 바이너리 문자열 | `FW v18d2 (OLED I2C guard + diag + radar timeout 1000 + txdrop)` 2회, `txdrop=` 1회, `(re)` 2회, `FW v18c` 0회 | 이 PC에서 확인(읽기만) |
| flash_log | `~/flash_log.txt`: `2026-10-25 13:11 \| rear_v18d2.bin \| 30d3b65f0608fc88 \| 최민기/Pi5` — **기록 있음, 해시 일치** | 이 PC에서 확인 |
| 빌드 | CubeIDE ARM 빌드 성공(바이너리가 존재) | 사용자 |
| 부팅 배너 | **`(re)` 줄 확인** → 보드가 v18d2 임이 시리얼로 확정 | 사용자 보고 |
| RADSTAT | `txdrop=` 표시됨 | 사용자 보고 |
| 빈 장면 | `safe=` 증가 없음 40초 이상 | 사용자 보고 |
| 레이더 전원/CAN 뽑기 | 약 1초 뒤 SAFE, 복구 시 `RADAR RECOVERED` | 사용자 보고 |
| 물체 시험 | `txdrop=0`, `ovf` 가 거의 매초 증가 (`RADAR_RX_NOTES.md`: ovf = 넘침이 있었던 초 수) | 사용자 보고 |
- 같은 보고에 "레이더가 약 5.5초 주기로 약 1.5초 끊김, 처진 동안 Safe Mode 반복 가능"이 있다. "빈 장면 40초 이상 safe 증가 없음"과는 **측정 시각/전원 상태가 달랐던 것으로 보이며 이 문서에서는 조정하지 못함** [가설].
- 사용자 멀티미터 측정(레이더 전원): XL6009 입력(Pi5 GPIO 5V에서 공급) 평소 약 4.5 V, 주기적으로 3 V대까지; 출력 평소 12.1 V, 10 V대까지 처짐. 레이더 사양 정상 주기 60 ms(초당 약 16~17개).

