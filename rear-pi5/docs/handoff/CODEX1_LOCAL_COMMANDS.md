# CODEX1 — 파이5 직접 터미널 실행 명령

현재 Codex 실행 환경에는 실제 /dev 장치가 노출되지 않습니다. 아래 명령은 파이5 데스크톱 터미널에서 실행해 주세요. 서비스·설정·펌웨어는 변경하지 않습니다.

## 1. 읽기 전용 점검
```bash
cd ~/ESWContest_ws
ls -l /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* /dev/video*
lsusb
lsusb -t
fuser -v /dev/ttyAMA2 /dev/ttyACM0
systemctl is-active aiv-ego-speed gpsd
timeout 5 gpspipe -w -n 5
dmesg | tail -n 80
```
AMA2에는 ego_speed_tx.py 쓰기 전용 서비스만 있어야 합니다. 다른 읽기 프로그램이 있으면 아래 수신기는 실행하지 마세요. 포트 점유 확인이 안 되면 그 사실도 보내 주세요.

```bash
taskset -c 2,3 python3 -I /home/user/ESWContest_ws/pi5/pipeline/live_receiver.py --duration 10
```
0x10은 보통 50 Hz, 0x11은 10 Hz이며, 0x12는 물체 수에 따라 달라집니다. 레이더 상태 CAN 프레임 a=16~17/s와 UART 0x10/0x12 주기는 서로 다른 지표입니다. a는 RADSTAT 기록으로 별도 확인해야 합니다. ST-Link는 이번 명령에서 열지 않습니다.

## 2. 카메라
/sys에서 D435i가 480 Mbps(USB2)로 보였습니다. 파란 USB3 포트와 USB3 지원 케이블로 연결 후 lsusb -t에서 카메라가 5000M인지 확인해 주세요.
```bash
cd ~/ESWContest_ws
taskset -c 0,1 ~/realsense-venv/bin/python pi5/pipeline/camera_check.py --seconds 15
```
기존 NCNN 모델 ~/yolov8n_ncnn_model 및 ~/yolov8n.pt가 있습니다. 새 가중치를 받지 않았습니다. 현재 선택은 NCNN imgsz 640이며, 320/416 대안은 실측 후 판단해야 합니다.

## 3. 통합 실행 (모두 mock 부저, 150초)
읽기 프로그램이 끝난 뒤 실행합니다. ttyUSB0가 실제 ESP32 브리지인지, 업로드된 펌웨어가 무엇인지 먼저 확인해야 합니다. ESP32는 /sys 기준 1개 후보뿐이며 앞차 실제 수신은 별도 장치가 필요합니다.
```bash
cd ~/ESWContest_ws
CAMERA=1 TRANSPORT=esp-now BIG=1 BUZZER=mock DURATION=150 tools/run_car_test.sh
```
카메라 라이브러리가 system python에 없으면 별도 터미널에서 아래처럼 실행할 수 있습니다. 이 경우 위 명령 대신 아래 두 명령을 사용합니다.
```bash
# 터미널 1
taskset -c 0,1 ~/realsense-venv/bin/python ~/ESWContest_ws/pi5/pipeline/camera_node.py --source realsense --port 19100
# 터미널 2
cd ~/ESWContest_ws
python3 -I pi5/pipeline/pipeline_main.py --uart /dev/ttyAMA2 --transport esp-now --serial-port /dev/ttyUSB0 --camera-udp-port 19100 --sinks csv,big --buzzer mock --duration 150 --affinity 2,3 --radar-power suspect --out logs/car_test/codex1_manual
```
물리 시험 단계 안내와 동기화는 채팅에서 시작을 맞춘 뒤 진행합니다. 아직 단계별 실제 시험은 하지 않았습니다. 전원은 기존 XL6009 상태이므로 suspect 표기를 유지합니다. 금속판 이동은 반사체 접근 시험이며 실제 차량 급제동의 정답 검증이 아닙니다.

## 4. 실제 부저와 업로드
실제 부저 시험은 승인됐지만 현재 실행 환경에서 GPIO를 확인할 수 없어 실행하지 않았습니다. 부저 시험 전 안내와 종료 후 GPIO26 OFF 확인을 함께 진행해야 합니다. SIGKILL은 프로그램의 종료 정리를 실행하지 못하므로 OFF를 보장하지 않습니다.
ESP32 업로드는 보드/포트를 다시 확인하고 사용자가 “업로드”라고 답한 후에만 진행합니다. 현재 업로드 명령은 실행하지 않았습니다.
