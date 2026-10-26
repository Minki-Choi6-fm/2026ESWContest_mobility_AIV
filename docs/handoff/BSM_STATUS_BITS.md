# BSM STATUS 비트 충돌 확인 (S10-e, 읽기 전용 · 자동 수정 없음)

BSM 규격은 팀 합의 사항이라 **코드·문서를 바꾸지 않았습니다.** 비교 대상: 앞차(사용자 설명) / 후방 `v2v/v2v_stack/bsm_protocol.py` / 후방 BSM 빌더(`pi5/pipeline/bsm_builder.py`) / 문서(`PROTOCOLS.md`, `PROJECT_STATE.md` 18번).
**S11 갱신**: 앞차 코드(`~/front_test/v2v/bsm_protocol.py`, `Downloads/v2v.zip`)를 직접 읽었습니다. 앞차는 비트 0~4 만 정의합니다(비트 5·6·7 정의/사용 없음) → **후방 제안 비트 5·6 과 충돌 없음**. 앞차 decode 는 CRC/버전만 검사하고 STATUS 를 그대로 받으므로 비트 5·6 이 켜져도 정상 수신(루프백 시험 PASS, `FRONT_COMPAT_TEST.md`).

| 비트 | 앞차 (사용자 설명) | 후방 bsm_protocol.py | 후방 문서 | 판정 |
|---|---|---|---|---|
| 0 | GPS_VALID | `STATUS_GPS_VALID` | GPS_VALID | 일치 |
| 1 | DANGER_ACTIVE | `STATUS_DANGER_ACTIVE` | DANGER_ACTIVE | 일치 |
| 2 | SENSOR_DEGRADED | `STATUS_SENSOR_DEGRADED` | SENSOR_DEGRADED | 일치 |
| 3 | SAFE_MODE | `STATUS_SAFE_MODE` | SAFE_MODE | 일치 |
| 4 | **BRAKE_ACTIVE (급제동)** | `STATUS_BRAKE_ACTIVE` ([PROPOSAL]) | BRAKE_ACTIVE (IMU 추정) | **이름·위치 일치 → 충돌 없음.** 단 **의미(근거)가 다름 [사실]**: 앞차는 STM32 IMU 플래그 또는 속도 변화 ≥3.0 m/s²(약 0.3 g, 해제 1.5 m/s²)로 판단, 후방은 IMU 감속 추정(≤ −0.1 g, 실제 페달 신호 아님, 센서 기울기 편향 약 +90 mg 미보정 — `FIRMWARE_IMU_BIAS.md`) |
| 5 | 정의 없음(앞차 코드 확인) | `STATUS_SPEED_SRC_GPS` ([PROPOSAL]) | SPEED_SRC_GPS | 충돌 없음(앞차 미사용) |
| 6 | 정의 없음(앞차 코드 확인) | `STATUS_SPEED_STALE` ([PROPOSAL]) | SPEED_STALE | 충돌 없음(앞차 미사용) |
| 7 | 정의 없음 | 사용 안 함(reserved) | reserved | 비어 있음 |

## 다른 층과의 혼동 위험 (충돌은 아님)
STM32→Pi5 의 `SENSOR_STATUS`(0x10 프레임)와 BSM `STATUS` 는 **다른 바이트·다른 번호 체계**입니다.
| 의미 | STM32 SENSOR_STATUS 비트 | BSM STATUS 비트 |
|---|---|---|
| SAFE_MODE | 4 | 3 |
| BRAKE_ACTIVE | 5 | 4 |
`bsm_builder.py` 는 두 번호를 따로 정의해 이름으로 옮기므로(`STM_BRAKE_ACTIVE = 1<<5` → `BSM_STATUS_BRAKE_ACTIVE = 1<<4`) 현재 코드에서는 섞이지 않았습니다. 다만 상수 이름 없이 `status & 0x10` 처럼 쓰면 층마다 뜻이 달라집니다.

## 해결 후보 (제안만)
1. **앞차 코드에서 비트 5, 6, 7 의 사용 여부를 확인**(`grep -n "1 << [567]\|0x20\|0x40\|0x80"`)하고, 쓰는 중이면 후방 제안 비트를 다른 번호(비트가 모자라면 STATUS 를 2바이트로 늘리는 규격 변경) 또는 앞차 정의에 맞춤.
2. 비트 5, 6 을 **BSM 에 싣지 않고** 후방 내부 로그에만 남김(규격 변경 없음, V2V 로 속도 출처/신선도는 전달되지 않음). 수신 쪽이 아직 비트 5, 6 을 읽지 않으므로 손실은 작음.
3. 비트 4 의미를 맞추기 위해 후방 BRAKE_ACTIVE 의 근거를 문서에 명시(IMU 추정, 오탐 가능)하고, 수신 쪽은 이 비트만으로 경고를 만들지 않도록 합의.
4. 팀 합의 후 `PROTOCOLS.md` 와 `bsm_protocol.py` 주석의 `[PROPOSAL]` 표시를 정식으로 바꿈.
