# OUTPUT_POLICY — 위험 단계(0/1/2/0xFF)와 출력(화면/부저/OLED)의 의미

대상: 부저·OLED·전방차 표시 연결을 맡는 사람. 지금은 **부저/OLED 를 붙이지 않았고**(`pipeline_main.py` 의 해당 싱크는 생성 자체를 거부), 정책만 상수와 문서로 고정했습니다. 모든 단계 기준은 `[PROPOSAL]` 입니다.

| 단계 | 의미 | 화면(표시) | 부저 | 비고 |
|---|---|---|---|---|
| 0 | 안전 | `-` | 금지 | 타깃 없음(레이더 정상)도 0 |
| 1 | 주의 | `!` | **금지 — 표시 전용** | 정지 물체 억제/정차 억제/마주오는·추월 차 억제로 내려온 경우 포함 |
| 2 | 위험 | `!!` | **후보** (`BUZZER_MIN_LEVEL = 2`) | 실제로 울릴지는 부저 싱크의 책임(아래 요건) |
| 0xFF | 모름 | `?` | **절대 금지** | 레이더 이상/0x10 무효/SAFE_MODE. 0(안전)로 보이면 안 됨 |

- 상수: `constants.BUZZER_MIN_LEVEL = 2`. `pipeline_main.output_flags(level)` 가 `(표시문자, 부저후보)` 를 계산하며 **0xFF 는 임계값을 1로 낮춰도 부저 후보가 되지 않음**(시험으로 확인).
- 싱크 인터페이스(`OutputSink.update(now, level, reason, info)`): `info["display"]`(`-`/`!`/`!!`/`?`), `info["buzzer"]`(부저 후보 여부), `info["side_count"]`, `info["side_near"]`(옆 차선 가장 가까운 후보 d·|lat|·vrel), `info["cutin"]` 가 함께 옵니다.
- CSV(`output_sink.csv`) 열: `t,event,level,display,buzzer_candidate,side_count,side_d,side_lat_abs,side_vrel,cutin,text,reason`. 종료/오류/SIGTERM/메인 루프 정지(워치독 1 s) 때는 `SAFE_STATE` 행(출력 꺼짐)으로 끝남.
- **옆 차선(1.75 m < |lat| ≤ 3.5 m)** 은 경고 대상이 아님: 추적만 하고 개수/최근접만 기록. 끼어들어 정상 확인 조건(3-of-5)을 채우면 앞차로 승격되며 `cutin` 표시. 좌/우 구분은 하지 않음(레이더 lat 부호 미검증).

## 부저 싱크를 만들 때의 요건 (아직 미구현)
1. `safe_state()` 에서 즉시 꺼짐, 멱등.
2. 출력 **자동 만료**(예: 마지막 update 후 500 ms): 허브 워치독은 이 프로세스 안의 정지만 잡으므로 부저 쪽에도 자체 만료가 필요.
3. 0xFF 와 level 1 은 소리 금지(위 표). 연속 경보 억제/최소 간격(깜빡임 방지)은 부저 쪽에서 별도 정의.
4. `/dev/ttyAMA2` 에 쓰지 않음(0x20/0x21 은 `SERIAL_WRITE_OPTIONS.md` 결정 후).
5. 배선/전원은 사용자가 직접 연결(레이더 5V 레일 XL6009 와 분리 권장 — HARDWARE_REAR.md).
