# BUZZER_BENCH — 후방 부저 연결·시험 절차 (코드는 준비됨, 실제 핀 구동은 사용자 승인 후)

**연결(사용자 설명):** Pi5 물리 핀 **37 = BCM GPIO26**(신호), 물리 핀 **34 = GND**. 부저는 **능동형** 입니다(사용자 청취 보고: 앞차 `buzzer_test.py` 로 켜면 계속 "삐—" 소리, 2026-10-25 — **형식은 확정, 모델명은 미확인**, 후방 개체는 같은 부품으로 추정).  이 세션에서는 실제 핀을 구동하지 않았습니다(모든 시험은 gpiozero `MockFactory`).
(`HARDWARE_REAR.md` 에 있던 후보 GPIO17/핀 11 은 사용자 설명으로 GPIO26/핀 37 로 대체. 레이더 전원의 XL6009 5 V 레일에는 연결하지 말 것 — 5 V 강하 의심 때문.)

## 파일
| 파일 | 역할 |
|---|---|
| `pi5/pipeline/buzzer_out.py` | `BuzzerSink` — gpiozero 사용(RPi.GPIO 금지). 기본 **능동형**(켜기/끄기), `kind="passive"` 이면 PWM(`BUZZER_PASSIVE_HZ`=3900 Hz, 듀티 50 %) |
| `pi5/pipeline/buzzer_out_test.py` | MockFactory 로만 실행하는 시험 19개(전부 통과) |
| `pi5/pipeline/buzzer_bench.py` | 사람이 2초 켜 보는 수동 시험 스크립트 |
| `pipeline_main.py --buzzer {off,mock,real}` | 출력 싱크로 연결(기본 `off`) |

## 동작 규칙 (코드와 시험으로 확인)
- **위험(2)에서만** 0.15 s 켜짐 / 0.15 s 꺼짐 반복. **0, 1(주의), 0xFF(모름), None 은 절대 울리지 않음**(임계값 상수가 아니라 `level == 2` 로 고정).
- 부팅 직후 기본 **꺼짐**(`initial_value=False`/duty 0). 꺼짐을 보장하는 경로: `safe_state()`, `close()`, **atexit**, SIGTERM/SIGINT(독립 실행 시 `install_signal_handlers()`; `pipeline_main` 은 자체 시그널 처리 + `finally`), **업데이트가 `BUZZER_TTL_S`(0.5 s) 끊기면 자체 스레드가 끔**(메인 루프가 멈춰도), 패턴 스레드가 죽어도 `finally` 에서 끔, 허브 워치독(1 s).
- **실제 핀 보호:** `BuzzerSink` 는 `pin_factory`(모의) 없이 `really=True` 도 없으면 생성 자체를 거부. `pipeline_main --buzzer real` 은 `--really-drive-gpio26` 가 **함께** 없으면 아무것도 시작하지 않고 종료 코드 2.
- 시험 결과(Mock): 패턴 켜짐 0.150 s/꺼짐 0.152 s(첫 펄스만 스레드 틱 때문에 최대 10 ms 짧음), 주의 전환 30 ms 안에 꺼짐, 업데이트 끊김 → 0.5 s 후 스스로 꺼짐, 예외/SIGTERM 중 울리던 상태에서도 종료 시 꺼짐, `pipeline_main` 정상 종료·예외·SIGTERM·메인 루프 3초 정지에서도 최종 꺼짐.
- 프로세스가 **SIGKILL** 로 죽으면 파이썬 정리 코드가 돌지 않습니다 — 이때는 lgpio 가 라인을 해제해 핀이 입력(고임피던스)으로 돌아가므로 보통 꺼지지만 **[가설: 실제 부저/배선으로 확인 필요]**. 부저 쪽에 풀다운 저항을 두면 더 안전합니다.

## 수동 시험 사용법 (사용자가 직접; 이 세션에서는 실행하지 않음)
```
python3 -I ~/ESWContest_ws/pi5/pipeline/buzzer_bench.py                         # 모의 핀 건식 시험 (실제 GPIO 안 건드림)
python3 -I ~/ESWContest_ws/pi5/pipeline/buzzer_bench.py --really-drive-gpio26 --type active   # 실제 핀: 능동형, 2초 연속음
python3 -I ~/ESWContest_ws/pi5/pipeline/buzzer_bench.py --really-drive-gpio26 --type passive  # 실제 핀: 수동형, 3900 Hz PWM 2초
   --pattern  을 붙이면 위험 패턴(0.15/0.15)으로, --seconds N(최대 5)으로 길이 변경
```
실제 핀 구동은 `DRIVE GPIO26` 를 직접 입력해야 시작됩니다(`--yes` 로 생략 가능). 끝나거나 Ctrl+C/예외가 나도 핀은 꺼진 채 해제됩니다.

## 능동형/수동형 구분 (S10: 능동형으로 기록 — 사용자 청취 보고. 아래는 일반 설명, 후방에서 직접 확인은 아직 안 함)
1. 능동형 부저는 직류만 걸어도 **연속음**이 납니다(내장 발진기). 수동형 압전은 직류를 걸면 **딸깍** 소리만 납니다. 먼저 `--type active` 로 시험해 연속음이면 능동형.
2. 딸깍만 나면 수동형 → `--type passive`(3900 Hz) 로 소리가 나는지 확인하고, 필요하면 `BUZZER_PASSIVE_HZ` 를 부저 정격 주파수로 조정(constants.py [PROPOSAL]).
3. GPIO 3.3 V 로 직접 구동하면 정격 12 V 부저는 소리가 작을 수 있음(HARDWARE_REAR.md). 전류는 ≤10 mA 라 GPIO 한 핀 한도(16 mA) 안. 더 큰 소리가 필요하면 트랜지스터 구동(사용자 배선).

## 전방/후방 핀 차이 (S10)
- 후방: Pi5 물리 핀 37(BCM26) + 34(GND). 전방: Pi4 의 **같은 물리 핀 37**, GND 핀만 다름.
- 앞차 `buzzer_test.py` 의 기본 핀이 **18** 이므로 전방은 `--gpio 26` / `--buzzer-gpio 26` 지정 필수.
- 실부저 구동은 **아직 사용자 승인 전**이라 이 세션에서 실행하지 않았습니다(`--buzzer real` 은 `--really-drive-gpio26` 없이는 거부).


## CODEX1 정정 (2026-10-25)
- 사용자 보고: 부저는 active형이며 파이5/파이4 물리 37번에서 울림을 확인하셨습니다. 위의 미확인·미구동 표기는 이전 세션 기준입니다. CODEX1에서는 실제 GPIO를 시험하지 못했습니다.
- 이번 실제 부저 3초 시험은 사용자 승인 범위에 포함됩니다. 다만 현재 실행 환경에는 GPIO 장치가 노출되지 않아 미실행입니다.
- ESP32 후방 소스는 코어 3.x 호환 수정 후 컴파일 성공했습니다. 무선 검증과는 별개입니다.
- 상세 최신 결과: CODEX1_REPORT.md.
