# PI5_TOOLS_AND_WORKFLOW — scripts and routines

All scripts below were written in chat (mostly read-only). The Pi5 may hold newer/different copies —
**check the real files** (`ls -la ~`, compare sizes/mtimes). Python for camera work: `~/realsense-venv/bin/python`.

## Recording and analysis
| Script | Purpose |
|---|---|
| `drive_logger_v1.py` (often installed as `~/drive_logger.py`) | Records camera mp4 (60 s chunks), `camera_ts.csv`, `frames.csv` (all STM32 binary frames from `/dev/ttyAMA2`), `radar.csv`, `stm_debug.log` (`/dev/ttyACM0` text with timestamps), `gps.jsonl` (gpsd), `sys.csv` (power/temp), `meta.json`. Output `~/vehicle_test_logs/drive_YYYYMMDD_HHMMSS/`. Usage: `--no-cam`, `--note "..."`. Run in tmux. Stop other readers of the ports first |
| `pi5_power_log_v1.py` | 1 s CSV of Pi5 EXT5V_V, throttled flag, temperature (`--interval`, `--out`). Warns if 5 V < 5.0 or throttled ≠ 0x0 |
| `analyze_drive_v1.py` / `v2.py` | Read-only summary of a drive folder: source rates, frame gaps (`--gap`), camera fps, GPS fix ratio/max speed, radar nearest-object stats, TIMEOUT/RECOVERED counts |
| `analyze_radstat_v1.py` / `v2.py` | Finds seconds when radar sends few frames from `RADSTAT` lines (`--good-a`, normal a≈17/s radar 60 ms period, b≈16 with objects) |
| `radstat_gps_v1.py` | RADSTAT correlated with GPS (❓ read the file) |
| `overlay_drive_v1.py`, `overlay_yolo_radar_v1.py` | Laptop tools: overlay radar points / YOLO boxes on recorded video; camera–radar mounting offsets are **assumed** (`--dx --h --yaw --pitch --flip --lag`) |
| `radar_range_cal.py` / `_v3.py` | Radar distance calibration against tape measure, appends to `~/radar_calib.csv`; resolution only 0.2 m; calibration was decided NOT to be applied |
| `offline_yolo_test.py` | Compare YOLO settings (`--imgsz`, `--conf`) on a recorded mp4 (lower bound quality) |
| `calibration.py` | Exports RealSense color intrinsics; does not estimate radar→camera extrinsics |
| `camera_detector.py` / `camera_detector_v2.py` | RealSense + YOLOv8n wrapper (640×480@15, conf 0.30, vehicle classes). ❓ Which version is live |
| `fusion.py` | UART decoder (`RadarPacketDecoder`, type 0x12) + radar→image projection + candidate association (intrinsics FX=607.16, FY=607.34, CX=321.98, CY=240.56; camera 1.5 m behind and 1.2 m above radar = assumed). **Not safety logic** |
| `radar_receiver.py` | Raw UART reader thread (`DEFAULT_PORT` is a CP2102 path — wrong for the rear car, pass `/dev/ttyAMA2`) |
| `main.py`, `record_camera.py` | Diagnostic recorders (YOLO video + CSV + raw UART). Observational only |
| `ego_speed_tx.py` | Sends 0x22 EGO_SPEED frames to STM32 (test version: fixed speed). Production behavior is the systemd service `aiv-ego-speed` ❓ |
| `v2v_node.py`, `bsm_protocol.py`, `transport.py`, `freshness.py` (+ ESP32 bridge `.ino`) | V2V BSM test stack. `python3 v2v_node.py --id 1 [--transport esp-now --serial-port /dev/ttyUSB0] [--broadcast-addr x.x.x.255]`. Uses dummy GPS unless wired to real sensors. Owner: 최민준 (pipeline) — ask before editing |
| `uart_status_receiver.py`, `aiv_uart_rx.py` | Old UART status/diagnostic readers |

## STM32 firmware workflow
- Project: STM32CubeIDE `AIV_rear_stm32` (`Core/Src/main.c`, `imu_uart.c`, `ssd1306.c`, `.ioc`). Chat-side versions `main_v15…v18c.c` + `rear_v*.bin`.
  **The CubeIDE project may still hold an older main.c than v18c** ❓ — do not assume it is in sync.
- Latest known good: **v18c** (OLED I2C guard, OLEDSTAT every 5 s: `ok/pushes/fails/retries/retry_ok/err/hal_err`; healthy = pushes +10 per 5 s, fails=0).
  `rear_v18c_test*.bin` are test-pattern builds, not for normal use.
- Debug text: `stty -F /dev/ttyACM0 115200 raw -echo; grep -a "" /dev/ttyACM0` (or `cat`). Key lines: `FW ...`, `RADSTAT`, `OBJ`, `OBD`, `IMU`, `OLEDSTAT`, `RADAR TIMEOUT -> SAFE MODE`.
- Flash (from Pi5 or laptop, **ask the user first**): `st-flash --connect-under-reset --reset write <file>.bin 0x8000000`; success shows "Flash written and verified" / "Download verified successfully".
- Build used `-O0`, nano specs, `--gc-sections` (CubeIDE Debug config).

## Working rules for you (Claude Code)
1. Read-only first (`snapshot_pi5.sh`, `ls`, `diff`, `git status`). Tell the user what changed since these docs.
2. One change at a time; keep a `.bak_*` copy; show a diff; run the smallest test.
3. Long jobs (loggers, drives) → `tmux`. Mind disk space for video (`df -h`).
4. Do not run `drive_logger` and other serial readers at the same time (exclusive ports).
5. Everything that touches safety constants, watchdog, core affinity, or flashing → ask first.
6. Report in easy Korean; mark hypotheses; never present unmeasured numbers as results.
