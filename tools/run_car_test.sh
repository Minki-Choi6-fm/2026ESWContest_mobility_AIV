#!/bin/bash
# 실차 테스트 한 번 실행 (뒷차 Pi5). 시작 전 점검을 출력하고, 실패해도 가능한 범위로 계속 진행합니다(카메라 없음 -> 레이더 단독).
# 이 스크립트가 하지 않는 것: 펌웨어 빌드/플래시, 보드 리셋, 서비스/설정 변경, 실제 부저 구동(기본 mock), 카메라 강제 실행.
#
# 사용 예
#   tools/run_car_test.sh                       # 레이더 + V2V(UDP) + 부저 mock, 카메라는 자동 판단
#   CAMERA=1 tools/run_car_test.sh              # 카메라 노드도 같이 시작 (카메라가 안 열려도 레이더 단독으로 계속)
#   TRANSPORT=esp-now tools/run_car_test.sh     # ESP32 브리지(/dev/ttyUSB0) 사용
#   WRITE_STM32=1 tools/run_car_test.sh         # 0x20 경고 프레임을 STM32로 전송 (현재 펌웨어는 0x20 을 버림)
#   BUZZER=real tools/run_car_test.sh           # 실제 부저(BCM GPIO26, 능동형): 이렇게 줄 때만 울림 (--really-drive-gpio26 가 자동으로 같이 붙음)
#   TARGET_SEL=win tools/run_car_test.sh        # 타깃 확인을 ts 조각이 아닌 수신 시각 창(0.4s/3회)으로 (기본: 기존 3-of-5)
#   LAT2_MAX_M=1.2 PASS_D_M=2.5 PASS_LAT_M=1.0 tools/run_car_test.sh   # 위험(2) 오경보 억제 옵션(기본 꺼짐): 옆 간격 상한 / 나란히 지나가는 물체 제외
#   SNAP_EVERY_S=2 tools/run_car_test.sh        # 카메라 프레임을 2초마다 $OUT/snap/<epoch_ms>.jpg (품질 70) 로 저장 (기본 0=꺼짐)
#   BIG=1 tools/run_car_test.sh                 # 조수석용 큰 글씨 0/1/2/? 화면 (터미널을 크게 열어 두세요)
#   RADAR_POWER=direct12v tools/run_car_test.sh # 레이더를 차량 12 V 에 직결한 뒤
set -u
set -o pipefail
WS="$HOME/ESWContest_ws"; PIPE="$WS/pi5/pipeline"
TRANSPORT="${TRANSPORT:-udp}"; BUZZER="${BUZZER:-mock}"; CAMERA="${CAMERA:-auto}"; WRITE_STM32="${WRITE_STM32:-0}"
RADAR_POWER="${RADAR_POWER:-suspect}"; DURATION="${DURATION:-0}"; UART="${UART:-/dev/ttyAMA2}"; CAM_PORT="${CAM_PORT:-19100}"; export SNAP_EVERY_S="${SNAP_EVERY_S:-0}"; export TARGET_SEL="${TARGET_SEL:-}"; export LAT2_MAX_M="${LAT2_MAX_M:-}" PASS_D_M="${PASS_D_M:-}" PASS_LAT_M="${PASS_LAT_M:-}"
OUT="${OUT:-$WS/logs/car_test/$(date +%Y%m%d_%H%M%S)}"
CAMERA_PYTHON="${CAMERA_PYTHON:-$HOME/realsense-venv/bin/python}"
[ -x "$CAMERA_PYTHON" ] || CAMERA_PYTHON=python3
WARN=0; FAIL=0
ok()   { echo "  [OK]   $*"; }
warn() { echo "  [WARN] $*"; WARN=$((WARN+1)); }
bad()  { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }

echo "=== 1) 시작 전 점검 ==="
# 서비스
for s in aiv-ego-speed gpsd; do
  if [ "$(systemctl is-active $s 2>/dev/null)" = "active" ]; then ok "서비스 $s 동작 중"; else warn "서비스 $s 가 active 가 아님 (속도/GPS 가 없을 수 있음. 서비스는 이 스크립트가 건드리지 않음)"; fi
done
# STM32 UART
if [ -e "$UART" ]; then
  HOLD=$(fuser "$UART" 2>/dev/null | tr -s ' ')
  OTHER=""
  for p in $HOLD; do c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null); case "$c" in *ego_speed_tx.py*) ;; *) OTHER="$OTHER $p($c)";; esac; done
  if [ -z "$OTHER" ]; then ok "$UART 를 읽는 다른 프로그램 없음 (ego_speed_tx 서비스는 쓰기 전용이라 정상)"; else bad "$UART 를 다른 프로그램이 잡고 있음:$OTHER -> 먼저 끝내세요 (읽기는 한 프로그램만)"; fi
else bad "$UART 없음 (STM32 UART 연결/설정 확인)"; fi
# gpsd fix
if command -v gpspipe >/dev/null 2>&1; then
  M=$(timeout 3 gpspipe -w -n 8 2>/dev/null | grep -m1 '"class":"TPV"' | sed -n 's/.*"mode":\([0-9]\).*/\1/p')
  if [ "${M:-0}" -ge 2 ] 2>/dev/null; then ok "GPS fix 있음 (mode $M)"; else warn "GPS fix 없음 -> 속도 출처는 OBD 만 (OBD 도 없으면 속도 모름: 부저 안 울리고 표시만)"; fi
else warn "gpspipe 없음: GPS 상태 확인 못 함"; fi
# 카메라
HAVE_CAM=0
if [ "$CAMERA" = "0" ]; then ok "카메라 사용 안 함 (CAMERA=0) -> 레이더 단독"
else
  if "$CAMERA_PYTHON" -c "import pyrealsense2 as rs,sys; sys.exit(0 if len(rs.context().query_devices())>0 else 1)" 2>/dev/null; then HAVE_CAM=1; ok "RealSense 장치 보임"
  else warn "RealSense 장치 없음/열거 실패 -> 레이더 단독으로 계속 (camera_confirmed=False)"; fi
fi
# ESP32
if [ "$TRANSPORT" = "esp-now" ]; then
  if [ -e /dev/ttyUSB0 ]; then ok "ESP32 브리지 포트 /dev/ttyUSB0 있음 (CP2102 일 수 있음. DTR 로 ESP32 가 리셋될 수 있어 시작 후 2~3초 걸림)"; else bad "TRANSPORT=esp-now 인데 /dev/ttyUSB0 없음 -> UDP 로 대체하려면 TRANSPORT=udp"; fi
fi
# 타깃 선택기
case "$TARGET_SEL" in
  win) ok "타깃 선택기 = WindowTargetSelector (시간 창 0.4s 안에 3번 확인, TARGET_SEL=win)";;
  "")  ok "타깃 선택기 = TargetSelector (기존 3-of-5, 기본값)";;
  *)   warn "TARGET_SEL=$TARGET_SEL 는 알 수 없는 값 -> 기존 TargetSelector 를 씁니다 (시간 창 방식은 TARGET_SEL=win)";;
esac
# 위험(2단계) 오경보 억제 옵션 (모두 기본 꺼짐)
if [ -n "$LAT2_MAX_M$PASS_D_M$PASS_LAT_M" ]; then ok "위험 억제 옵션 켜짐: LAT2_MAX_M=${LAT2_MAX_M:-꺼짐} PASS_D_M=${PASS_D_M:-꺼짐} PASS_LAT_M=${PASS_LAT_M:-꺼짐} (주의 1단계는 그대로)"; else ok "위험 억제 옵션 꺼짐 (LAT2_MAX_M/PASS_D_M/PASS_LAT_M 미설정 = 기존과 동일)"; fi
# 부저
if [ "$BUZZER" = "real" ]; then ok "부저 = real (BCM GPIO26 = 물리 37번 핀, 위험 단계 2에서만 울림, 속도 모르면 안 울림)"; else ok "부저 = $BUZZER (실제 핀 안 건드림. 울리려면 BUZZER=real)"; fi
[ "$WRITE_STM32" = "1" ] && warn "WRITE_STM32=1: $UART 로 0x20 프레임을 씁니다 (프레임당 write 1번, 현재 펌웨어는 버림)"
# 전원 메모
if [ "$RADAR_POWER" = "suspect" ]; then warn "레이더 전원이 불안정으로 표시됨(RADAR_POWER=suspect): Safe/0xFF 는 전원 처짐 영향 가능 -> 로그의 power_suspect 열로 구분"; else ok "레이더 전원 = 차량 12V 직결 표시"; fi
# Pi5 전원
T=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2); [ "${T:-0x0}" = "0x0" ] && ok "Pi5 throttled=0x0" || warn "Pi5 throttled=$T (저전압/과열 이력)"
echo "점검 결과: WARN $WARN, FAIL $FAIL"
if [ "$FAIL" -gt 0 ]; then echo "FAIL 이 있어 시작하지 않습니다. 위 [FAIL] 을 해결한 뒤 다시 실행하세요."; exit 2; fi

echo "=== 2) 실행 ==="
mkdir -p "$OUT"; echo "로그 폴더: $OUT"
CAM_PID=""
cleanup() { [ -n "$CAM_PID" ] && kill "$CAM_PID" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM
CAMARGS=""
if [ "$CAMERA" != "0" ] && { [ "$HAVE_CAM" = "1" ] || [ "$CAMERA" = "1" ]; }; then
  echo "카메라 노드 시작 (코어 0-1)"; taskset -c 0,1 "$CAMERA_PYTHON" "$PIPE/camera_node.py" --source realsense --port "$CAM_PORT" --snap-dir "$OUT/snap" > "$OUT/camera_node.log" 2>&1 &
  CAM_PID=$!; CAMARGS="--camera-udp-port $CAM_PORT"
fi
SINKS="csv"; [ "${BIG:-0}" = "1" ] && SINKS="csv,big"
ARGS=(--uart "$UART" --transport "$TRANSPORT" --out "$OUT" --sinks "$SINKS" --stats-dir "$OUT" --record "$OUT/rec" --duration "$DURATION" --radar-power "$RADAR_POWER" --affinity 2,3)
[ -n "$CAMARGS" ] && ARGS+=($CAMARGS)
[ "$WRITE_STM32" = "1" ] && ARGS+=(--write-stm32)
case "$BUZZER" in real) ARGS+=(--buzzer real --really-drive-gpio26);; mock) ARGS+=(--buzzer mock);; *) ARGS+=(--buzzer off);; esac
[ -f "$WS/state/k_profile.json" ] && ARGS+=(--k-profile "$WS/state/k_profile.json")
echo "python3 -I pipeline_main.py ${ARGS[*]}"
echo "종료: Ctrl+C (부저/경고는 자동으로 꺼집니다)"
cd "$PIPE"
if [ "${BIG:-0}" = "1" ]; then python3 -I pipeline_main.py "${ARGS[@]}" 2> "$OUT/pipeline_stderr.txt"   # 큰 글씨 화면을 tee 로 가리지 않음
else python3 -I pipeline_main.py "${ARGS[@]}" 2>&1 | tee "$OUT/pipeline_stdout.txt"; fi
PIPELINE_RC=$?
echo "=== 3) 끝 ===  로그: $OUT/drive_log.csv (+ output_sink.csv, bsm_tx.csv, rec/)"

exit "$PIPELINE_RC"
