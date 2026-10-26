# CODEX1 작업 보고 — 2026-10-25
## 최신 결과 — 카메라 비교·실제 통합·부저 (2026-10-25)

이 절은 아래의 이전 세션 기록보다 최신입니다. **실차 경고 검증 준비는 아직 미완료**입니다. 실제 2분 통합은 성공했지만 위험 단계 도달·상대 ESP32 무선 수신은 확인하지 못했습니다.

| 구간 | 최신 확인 | 남은 확인 |
|---|---|---|
| STM32 수신 | 실제 120초, 0x10 5996/5996·0x11 1203/1203, BAD0·시계기준 손실0% | CAN 물체 손실·거리 정확도 |
| 카메라 | 실제 640/416/320 FPS·추론 지연 측정, 통합 camera_alive 확인 | 차량 검출 품질·실제 camera_confirmed |
| 선택·위험 판단 | 실제 타깃 거리/상대속도·출력20Hz 확인; 위험 경로는 가짜 입력 | 실제 위험 단계2/반사체 정답 라벨 |
| ESP-NOW | 실제 CP2102 시리얼50개 및 통합1196개 전송 | 무선 송출·상대 수신·손실률·지연 |
| 앞차 경고 | 루프백12/12·통합 브리지 모델 성공 | 실제 앞차 장비 |
| 부저 | 실제3초패턴 소리 발생(사용자 청취), OFF 읽기값0 | 실제 SIGINT/SIGTERM/SIGKILL 종료별 시험 |
| 큰 화면 | 실제120초 SAFE 표시·종료SAFE_STATE | 차량 모니터 가독성 |

### 카메라 비교 (워밍업 5프레임 제외 후 15초, 코어0-1)
| imgsz | FPS | YOLO p50 ms | p90 ms | max ms | 워밍업 포함 max ms | 최대 발생 프레임 |
|---|---:|---:|---:|---:|---:|---:|
| 640 | 7.58 | 127.4 | 159.8 | 189.2 | 2181.5 | 1 |
| 416 | 14.51 | 52.5 | 80.3 | 110.2 | 1930.7 | 1 |
| 320 | 15.02 | 35.2 | 52.6 | 70.6 | 2483.9 | 1 |

이번 세 실행의 최대 지연은 모두 첫 프레임. 큰 지연이 첫 추론 초기화 구간에 집중됨을 확인했습니다. 이전 3869ms 표본의 프레임 번호 자체는 기록이 없어 미확인입니다. 기본640은 유지했습니다. 416/320은 로컬 기존 가중치에서 별도 변환한 고정 크기 NCNN 모델이며 실제 predictor 입력 일치 확인. **검출 품질은 책상이라 [확인 필요]**. USB3 추가 조사 종료.

### 실제 2분 통합 — logs/car_test/20261025_161623/
- CAMERA=1 TRANSPORT=esp-now BIG=1 BUZZER=mock DURATION=120. 기본 NCNN640, 카메라 코어0-1·파이프라인 코어2-3.
- 시작 오류(UDP sock 속성 참조)는 수정. 새 pipeline_espnow_test 전체 경로 통과, 수정 후 pipeline_main_test 11/11 재통과(logs/pipeline_main_test/20261025_161442).
- 2397평가, 출력20Hz. 레벨0=2397, 1/2/255=0. 대상 거리1.0~18.6m, 상대속도−0.5~+0.5m/s. 실제 위험 경고 검증은 아님.
- 타임아웃 summary/heartbeat/레이더/IMU=0, STM32 reset0, BAD len/chk/resync0. Safe/레이더down/power_suspect 표식0행. **전원 문제를 고친 것은 아님**. CAN 손실을0이라고 주장하지 않음.
- 시리얼 BSM1196개. 앞차 수신 증거 없음(ESP32 후보1개). 전송 성공은 OS 시리얼 쓰기 성공을 의미.
- camera_alive2219/2397행, 첫 alive t9.101s, 이후 마지막120초까지 alive. camera_confirmed0행. 초기 모델 준비 중에도 레이더 처리가 계속됨.
- 자차 속도 전부none. 속도 모름→부저 표시차단 기본 동작 확인; 이번 통합 레벨은0뿐이므로 위험2 차단 증거는 별도 모의 시험과 실제 부저 bench inhibit 확인을 함께 참조.
- output_sink 마지막 SAFE_STATE reason=duration reached. 레이더→레벨 전환 지연은 실제 위험 전환이 없어 미측정.
- 반사체 느린/빠른 접근·제거 안내 시각 notes.txt 기록. **사용자의 실제 수행 여부는 응답에서 확인되지 않았으므로 시나리오 수행 완료로 표시하지 않음**.

### 실제 부저
- 사용자 직접 --really-drive-gpio26 --yes --type active --pattern --check-levels --seconds3 실행. 사용자 “소리났어” 보고.
- 실제 level0/1/255 및 level2+buzzer_inhibit 모두OFF verified. 패턴 종료 후 GPIO26 release 이전 읽기값0 확인. 정상 경로 실제 출력OFF는 확인. 종료 후 청취상 완전 정지 여부는 별도 응답 없음.
- 실제 SIGINT/SIGTERM/SIGKILL은 시험하지 않음. 기존 mock SIGTERM/예외/스레드 사망 시험은 통과. SIGKILL 안전정리는 보장하지 않음.

### 남은 무선 시험 방법
- CP2102 메타데이터는 ESP32 MCU 모델/기존 펌웨어를 증명하지 않음. 보드 인쇄/기존 업로드 기록 확인 필요. 업로드는 하지 않았으며 사용자 “업로드” 응답 필요.
- 두 번째 ESP32를 이 파이에 연결하면 ttyUSB0/1을 재식별한 후, 뒷차는 ttyUSB0, 앞차는 아래처럼 ttyUSB1로 분리해 실행. 두 브리지는 동일한 채널/프레임 규격 필요(앞차 브리지 채널1·CRC 버전 권장).
```bash
python3 /home/user/front_test/v2v/front_node.py --no-stm32 --transport esp-now --serial-port /dev/ttyUSB1
```
- 두 번째 ESP32가 앞차 Pi4에 있으면 앞차 Pi4에서 해당 로컬 포트를 지정. 앞차에 --buzzer-gpio를 주지 않아 무음으로 시험. 후방차LIVE·위험ON·약1초 뒤OFF·시계 정합 또는 같은 호스트 기준 지연을 확인해야 함. 위험ON/OFF 주입 시험은 아직 실제로 하지 않음.

### 사용자 요청4 반영
CAR_TEST_RUNBOOK 최상단에 **실차 부저는 유효한 OBD/GPS 속도 필수**, 속도 없음/오래됨은 위험2도 무음 표시, BUZZER=real 필요, --buzz-without-speed는 책상시험 예외를 추가했습니다.

### 변경/검증
- camera_check.py: --warmup-frames(default0), --imgsz, --model, --json-out; 최초 샘플과 측정 구간 분리, 실제 입력 크기 검증, 로컬 모델만 사용.
- tools/compare_camera.py, tools/esp32_probe.py 추가. 후자는 STM32 포트를 거부하고 CP2102 후보만 명시적으로 시리얼 시험.
- tools/run_car_test.sh 카메라 전용Python을 기존 realsense-venv로 선택. 기본640 유지.
- pipeline_main.py: UDP socket이 없는 ESP-NOW 경로 처리. 새로운 pipeline_espnow_test.py 통과(가짜 장치만).
- buzzer_bench.py: --check-levels 및 종료 전 GPIO 읽기확인 추가. mock 검증 후 실제 실행.
- 기존 파일 수정 전 .bak_날짜_codex1 백업. 시스템 서비스/설정/안전상수/앞차 원본/STM32 펌웨어 변경·업로드·push 없음.

## 이전 단계 기록 (당시 상태)


## 결론
**실차 시험 불가능(현재 검증 기준).** 로컬 가짜 입력 회귀는 통과했지만 실제 센서·카메라·무선·경고 통합 검증이 남았습니다. 레이더 전원 처짐도 해결하지 않았습니다.

## 구간별 상태
| 구간 | 이번 확인 | 남은 점 |
|---|---|---|
| STM32 수신 | 이번 실제 하드웨어 10초 수신 확인 | 장시간 안정성·RADSTAT a 주기 미확인 |
| 카메라 | 실제 카메라·YOLO 실행 확인, 8.6 FPS, p50/p90 113/134 ms | 성능 WARN; USB3·통합 확인 필요 |
| 선택·위험 판단 | 가짜 입력·자체 시험 통과 | 실차 정답 비교 없음 |
| ESP-NOW 송수신 | 브리지 모델 5/5, 양쪽 소스 컴파일 성공 | 실제 무선 미시험; CP2102 1개만 보임 |
| 앞차 경고 | 로컬 루프백 12/12 | 앞차 실제 장비 미시험 |
| 부저 | mock 19/19; 과거 사용자 실음 확인 보고 | 이번 실제 GPIO 구동/종료 OFF 미시험 |
| 큰 화면 | 5개 기호 파이프 렌더 및 csv,big 통합 성공 | 실제 차량 모니터 미확인 |

## 시험 숫자 (모두 로컬 가짜 입력/루프백)
- integration_test: 16/16. 로그 logs/integration_test/20261025_155329/.
- pipeline_main_test: 11/11. 로그 logs/pipeline_main_test/20261025_155317/.
- front_compat_test: 12/12. 위험 ON 13 ms, 마지막 위험 BSM 뒤 OFF 0.98 s, 시계 SEQ 재시작 ON 0.01 s. 실제 무선 성능이 아님.
- buzzer_out_test: 19/19. 실제 관측 패턴 약 0.1/0.1초. 스레드 오류 traceback은 의도적으로 clock 실패를 주입한 시험이며 OFF 확인 통과.
- espnow_bridge_sim_test: 5/5. 실제 ESP32 무선이 아닌 Python 모델.
- camera_confirm_test: 13/13, v2v_compat_test ALL PASS.
- brain/danger/target_select/vision_adapter/k_estimator/live_receiver/loss_analysis/replay_check 자체 시험 성공.
- writer_mix_test: 0x22 6000개 수신, 0x20 5978개 수신; 버퍼 포화에 따른 의도적 전송 버림 24개, 부분 쓰기 0, 깨짐 0, 쓰레기 바이트 0. 12002개 전체 무손실이라고 주장하지 않음.
- 큰 화면: 0/1/2/?/OFF 5종 렌더 성공; pipeline_main --sinks csv,big 4초 종료코드 0.
- desk_tests a~f 완료(가짜 입력 관측 시험, exit 0): 정상 기록/재생 7739프레임 일치, OBD 실패창 GPS 9/9줄·복구 후 OBD, 물체 단절과 레이더 사망 상태 구분, 오류 주입 후 BAD 증가 0·수신 50/s 복구, 백로그 20625 B 제거, 리셋 모사 1회 감지. 로그 logs/desk_tests/20261025_155454/. 최초 샌드박스 실행은 소켓 제한과 45초 제한으로 중단; 로컬 소켓 허용 후 전체 완료.
- run_car_test.sh 실행 권한 부여, bash -n 성공. 종료코드 보존도 수정.
- ESP32 후방 883436 B, 앞차 886764 B 컴파일 성공. 업로드 없음.

합성 A 시나리오: 297행, 출력 20 Hz. 레벨 0/1/2/255=132/27/98/40행. 레이더 단절 모사 40행(약 2초), 가짜 camera_alive 276행, camera_confirmed 224행. 세부 reason 분포 logs/codex1/integration_metrics.json. 센서 실제 지연·실제 손실률의 측정값이 아닙니다.
속도 모름 B에서는 레벨 2를 표시하되 부저 전이 0회. --buzz-without-speed에서만 mock 부저 전이 50회. 카메라 없는 C도 종료코드 0, camera_confirmed 항상 0.

## 하드웨어 읽기 점검
| 항목 | 확인 결과 | 근거/한계 |
|---|---|---|
| 장치 노출 | Codex /dev에는 실제 장치 없음 | 샌드박스 한계; 실제 미연결로 해석하지 않음 |
| RealSense | Intel D435i 8086:0b3a, 480 Mbps | /sys/bus/usb/devices/1-1; USB2 속도 |
| ESP32 후보 | CP2102 10c4:ea60 1개, ttyUSB0 | /sys; ESP32 칩 종류/펌웨어는 확인 못함 |
| STM32 | ST-Link 0483:374b, ttyACM0 | /sys; 리셋/시리얼 열기 하지 않음 |
| UART2 | ttyAMA2 sysfs 존재 | 사용자 직접 출력에서도 장치 파일 확인 |
| 서비스 | aiv-ego-speed/gpsd 모두 active | 사용자가 직접 터미널 출력 제공 |
| UART 점유 | PID 653 python3 하나 | 사용자 출력. ego_speed 서비스와의 일치 확인 대기 |
| GPS fix | 미확인 | Codex에서는 소켓 차단; active가 fix를 의미하지 않음 |
| USB 오류 | 과거 UVC ROI control error -5 1건 | dmesg 읽기; 현재 스트림 장애 원인이라고 단정하지 않음 |

## 발견 및 수정
- [사실] AGENTS.md가 디스크에 없어 첫 첨부 내용을 저장하고 정정 항목을 추가함.
- [사실] S12 §9가 본 실행 문서에 빠져 있어 백업 후 반영함. 전원 최저 허용값을 10 V로 단정한 문구는 근거 미확인으로 정정.
- [사실] writer_mix_test가 O_NONBLOCK 전송 버림과 바이트 섞임을 구분하지 않아 실패함. 성공 전송 수·버림 수·부분 쓰기와 실제 수신 수를 대조하도록 수정. 전송 버림 정책 자체는 변경하지 않음.
- [사실] 부저 시험은 상수를 이미 사용했지만 출력 문구만 0.15초로 남았음. 실제 상수 값으로 표시하도록 변경. 안전 상수는 수정하지 않음.
- [사실] 실행 스크립트가 파이프라인 실패에도 마지막 echo 때문에 성공 코드로 끝날 수 있어 pipefail과 종료코드 보존을 추가함.
- [가설] 기존 레이더 주기적 끊김은 XL6009 전원 처짐 영향 가능. 전원/배선 변경이나 개선 확인을 하지 않음.

## 사용자가 직접 할 일
1. 카메라를 USB3 포트/케이블로 연결하고 5000M인지 확인.
2. 실제 10초 수신 완료. 다음은 카메라 성능 점검.
3. CODEX1_LOCAL_COMMANDS.md 순서로 GPS fix·카메라 성능 점검.
4. 실제 ESP32 보드 종류·포트와 두 번째 수신 보드 위치 확인. 업로드는 보드/포트 확인 및 “업로드” 응답 후에만.
5. 실제 레이더+카메라+무선 mock 부저 통합을 2~5분 실행. 물리 반사체 단계는 사용자와 시작 시각을 맞춰야 함.
6. 실제 부저 3초 및 종료 GPIO26 OFF 확인. SIGKILL은 종료 정리가 실행되지 않으므로 OFF 보장 불가, 미검증.
7. 실차 전 안정된 레이더 전원 및 멀티미터 MIN 기록 확보. 현재 전원은 기존 방식 그대로.

## 시험하지 않은 것
RADSTAT 60초·정상 a 주기·GPS fix·CPU 온도/사용률·실제 ESP-NOW 지연/손실률·앞차 실제 경고·실제 부저 종료 OFF·물리 반사체 단계·차량 주행·K 3회 이벤트·거리 정확도 모두 [확인 필요].

수정 전 기존 파일은 .bak_20261025_*_codex1로 백업. 실제 UART 쓰기, STM32 빌드/플래시/리셋, 서비스/설정/안전 상수/앞차 원본 수정, push는 하지 않았습니다.

## 후속 실제 수신 결과
사용자 제공 실제 10초 수신(20261025_155938): 0x10=500/500, 0x11=100/100, 0x12=506, 0x13=200, 0x14=3. BAD len/chk/resync=0/0/0, 0x10·0x11 시계 기준 손실 0%. 상태 0x0B 500개, 레이더/IMU/summary/heartbeat 타임아웃 0회. OBD 타임아웃 2회, 속도 0xFF·GPS fix 0으로 자차 속도 없음(책상 환경). 수신기가 ego_speed_tx.py를 비읽기 점유자로 확인. 시작 백로그 1314832 B 제거. 0x12는 물체 프레임 50.6/s이며 레이더 CAN 상태 a=16~17/s와 다른 지표. 10초 동안 끊김이 없었다는 사실이며 전원 개선/장시간 안정성 증거는 아님.

## 후속 실제 카메라 결과
사용자 직접 카메라 시험: taskset -c 0,1 및 realsense-venv로 D435I(S/N 047322070031) 실제 RGB 스트림·YOLO NCNN imgsz640 실행 성공. 설정 640x480@15, 15초 시험 92프레임, 보고 FPS 8.6, YOLO p50 113 ms/p90 134 ms/max 3869 ms, total p50 113 ms, 검출 평균 0.0. 제안 기준 FPS>=15/p90<=100 ms에 미달하여 WARN. 검출 0은 차량 확인 정확도나 고장 증거가 아님. 최대 지연 원인은 로그만으로 미확정(초기 준비 가능성은 가설). CPU 온도·사용률/현재 USB 링크 속도/실제 camera_confirmed 통합은 미측정. 카메라 보조 처리 가능하나 성능 최적화 및 통합 검증 필요.

## 후속 USB 연결 속도 확인
사용자 직접 lsusb -t 결과: 카메라 Video/HID 인터페이스는 Bus001 Port001에서 480M, USB3 root hub Bus002/004는 5000M이나 하위 장치 없음. 사용자는 USB3 포트에 연결했다고 확인함. 따라서 포트 삽입 위치와 별개로 현재 카메라 연결은 USB2 속도임. 원인(케이블/접촉/기타)은 미확정이며 USB 속도가 YOLO p50 113 ms 원인이라는 증거 없음. 기존 RGB 스트림은 실제 실행 성공했으므로 카메라 장애 또는 전체 시험 불가로 단정하지 않음.

## CODEX1 카메라 비교・ESP32 (2026-10-25)
사용자 지시로 USB3 추가 조사는 종료했습니다.
- camera_check.py에 --warmup-frames(기본 0), --imgsz, --model, --json-out 추가. 기존 기본 모델/크기 유지. 처음 5프레임 제외 후 15초 측정.
- 현재 고정 NCNN 모델은 입력 크기 옵션을 덮어쓰므로 로컬 기존 pt 가중치로 416/320 시험 모델을 별도 생성. 기본 모델/비전 소스 변경 없음. 각 실제 입력 크기 일치 확인.
- 실제 측정(코어0-1): 640 FPS7.58/p50 127.4/p90 159.8/max189.2ms; 416 FPS14.51/p50 52.5/p90 80.3/max110.2ms; 320 FPS15.02/p50 35.2/p90 52.6/max70.6ms.
- 워밍업 포함 최대 2181.5/1930.7/2483.9ms는 모두 각 실행 첫 프레임. 따라서 이번 큰 지연은 최초 추론 준비 구간에 집중. 이전 3869ms의 정확한 프레임은 이전 샘플 로그가 없어 단정 불가.
- 검출 품질은 책상 환경 [확인 필요]. 로그 logs/codex1/camera_compare_20261025_161024/.
- ESP32 후보 CP2102 VID10C4:EA60 ttyUSB0 1개. 실제 MCU 모델/업로드된 브리지 펌웨어는 USB 메타데이터만으로 확인 못함. 사용자 시리얼 시험: 50 BSM 전송, 수신0, 손상0. 시리얼 드라이버에 쓴 사실만 확인, 무선 송출/상대 수신 증거 아님. 업로드 없음.
- 통합 스크립트 카메라 Python을 realsense-venv로 선택(시스템 Python은 ultralytics/ncnn 없음). 기본 imgsz640 유지.
- CAR_TEST_RUNBOOK 최상단에 실차 부저는 유효한 OBD/GPS 속도 필수, mock 무음, --buzz-without-speed는 책상 시험 예외라고 명시.

## 실제 통합 첫 실행 실패 및 수정
실제 통합 20261025_161313은 FAIL0/WARN2 시작 점검 뒤 AttributeError: EspNowTransport has no attribute sock로 즉시 종료(tx0/evals0/camera0). 부저 mock OFF. 원인: pipeline_main.py의 select 목록이 UDP sock을 무조건 참조. ESP-NOW는 sock 없이 최대20ms 주기로 비차단 시리얼 수신을 처리하도록 수정. 새 pipeline_espnow_test.py는 가짜 STM32+가짜 ESP32 두개+앞차 노드를 연결해 통과(117평가/송신56/상대수신80/앞차LIVE/mockOFF). 실제 무선 결과가 아님. 기존 브리지 모델 시험은 pipeline_main 경로를 실행하지 않아 이 결함을 잡지 못했음. 실제 통합은 재시도 필요.
