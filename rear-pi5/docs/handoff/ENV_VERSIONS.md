# ENV_VERSIONS — 개발 환경 버전 (보고서용, 읽기 전용 수집; 설치/수정 없음)

수집: 2026-10-25, 후방 Pi5(`car-rear`)에서 `cat /etc/os-release`, `uname`, `--version`/`pip list`/`dpkg -l`/`git describe` 로 확인. **확인 못 한 것은 "미확인"**, 이 컴퓨터에 없는 것은 "해당 없음/미설치".

| 항목 | 값 | 비고 |
|---|---|---|
| 하드웨어 | Raspberry Pi 5 Model B Rev 1.1 | `/proc/device-tree/model` |
| OS | Debian GNU/Linux 13 (trixie) | `/etc/os-release` |
| 커널 | 6.18.39+rpt-rpi-2712 | `uname -r` |
| VideoCore 펌웨어 | 2026/01/21 13:38:47 (Broadcom) | `vcgencmd version` |
| Python | 3.13.5 (시스템 `/usr`), 3.13.5 (`~/realsense-venv`) | |
| gpsd | 3.25 (apt `3.25-5+deb13u2`) | GPS 모듈 드라이버 MTK-3301, 9600 bps |
| pyserial | 3.5 (시스템 apt `python3-serial 3.5-2`, venv 3.5) | |
| gpiozero | 2.0.1 (시스템 apt, venv 2.0.1) | `lgpio 0.2.2.0`, `rpi-lgpio 0.6` |
| ultralytics | **8.4.166 (`~/realsense-venv` 에만)** | 시스템 Python 에는 미설치 |
| ncnn (Python) | **1.0.20260526 (venv 에만)** | NCNN YOLO 모델 폴더 `~/yolov8n_ncnn_model` 존재 |
| torch / torchvision | 2.14.0 / 0.29.0 (venv) | |
| numpy / OpenCV | 2.2.4 / apt `python3-opencv 4.10.0` (시스템), venv: `opencv-python 5.0.0.93` | |
| pyrealsense2 | 2.58.4 (venv 에서 import 확인) | 시스템 Python 에서는 미설치 |
| librealsense | **2.58.4** (`~/librealsense` git `v2.58.4-3-ge15c5d6bb`, `rs-enumerate-devices 2.58.4.0`, `/usr/local/lib/librealsense2-gl.so.2.58.4`) | D435i 사용 |
| arduino-cli | 1.5.1 (2026-06-05) | `~/bin/arduino-cli` |
| ESP32 Arduino 코어 | **esp32:esp32 3.3.12** | 9/27 브리지 .ino 는 이 코어에서 컴파일 안 됨(`V2V_DEPLOY_NOTES.md`) |
| Arduino IDE | **미설치** (실행 파일 없음; `~/Arduino` 스케치북 폴더만 있음) | |
| STM32CubeIDE | **해당 없음 — 이 컴퓨터에 없음** (STM32 프로젝트는 팀 노트북 CubeIDE 에서 관리) | 프로젝트 `.ioc` 기록: **STM32CubeMX 6.18.1, DB 6.0.181, 펌웨어 패키지 STM32Cube FW_F4 V1.28.3, MCU STM32F446RETx, TargetToolchain=STM32CubeIDE** (CubeIDE 자체 버전은 **미확인**) |
| arm-none-eabi-gcc | 미설치 | 문법 확인은 호스트 gcc 로 대체(`FIRMWARE_TX_QUEUE.md` §5) |
| st-flash / stlink-tools | 1.8.0 (apt `stlink-tools 1.8.0-1.1`) | 플래시는 하지 않음 |
| git / tmux | git 2.47.3 / tmux 설치 여부는 버전 문자열 미출력 → **미확인** | |
| STM32 펌웨어(현재 보드) | **미확인** — 하드웨어 직접 읽기 없이는 알 수 없음. 소스 최신은 `main_v18c.c`(보드에 v18c 가 올라가 있는지는 미확인; 10/16 로그는 v18c 이전 펌웨어) | `LOG_EVIDENCE.md` |
