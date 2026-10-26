# HARDWARE_REAR — rear car wiring, ports, power

Source: `후방차_배선_정리.md` (2026-10-09, updated 10-12) + later chat. ✅ verified / 📝 documented / ❓ unknown.
Physical Pi5 header pin numbers were not fully recorded.

## STM32 NUCLEO-F446RE pin map
| Function | Peripheral | Pins | Connected to | Conf. |
|---|---|---|---|---|
| Pi5 link (binary frames) | USART1 115200 8N1 | PA9 TX / PA10 RX | Pi5 `/dev/ttyAMA2` (crossed, common GND) | ✅ code / 📝 Pi pins |
| Debug text | USART2 115200 | PA2 / PA3 | ST-Link VCP → `/dev/ttyACM0` (laptop or Pi5 via USB) | ✅ |
| IMU (UART module) | USART3 | PC10 TX / PC11 RX | module TX → PC11 | ✅ |
| OBD module (Longan Serial CAN) | USART6 9600 | PC6 TX / PC7 RX | white wire → PC6, yellow → PC7 (crossed) | ✅ pins / 📝 colors |
| Radar CAN controller MCP2515 | SPI2 | MOSI PC1, MISO PC2, SCK PB13 | MR76 radar (CAN_H/L) | ✅ MOSI/MISO / 📝 SCK |
| MCP2515 CS / INT | GPIO / EXTI8 | PA8 / PC8 | | ✅ |
| OLED SSD1306 (0x3C) | I2C2 | SCL PB10 / SDA PC12 | OLED | ✅ |
| Board LED / button | GPIO | PA5 / PC13 | | ✅ |
- OBD wiring mistake happened once (yellow/white swapped → `+++ NO REPLY`).
- Radar CAN termination (120 Ω) state: ❓ not confirmed. OBD module's 120 Ω jumper is intentionally open (tapping an existing car bus).
- Module power (3.3 V vs 5 V) for IMU/OLED/MCP2515: ❓ check physically. Whether OLED I2C at 3.3 V vs 5 V matters: raised, not concluded.
- IWDG ≈ 40 ms. Do not change.

## Raspberry Pi 5 (rear)
| Item | Value | Conf. |
|---|---|---|
| STM32 link | `/dev/ttyAMA2`, 115200 8N1 (overlay `uart2-pi5`) | ✅ |
| GPS | `/dev/ttyAMA0` 9600, via `gpsd` (`/etc/default/gpsd`, backup `gpsd.bak`). VCC pin 2 (5 V), GND pin 6, module TX → pin 10 (GPIO15), module RX → pin 8 (GPIO14). `ttyAMA3` is the FRONT RPi4 setting — do not apply here | ✅ NMEA / 📝 pins |
| Ego speed service | `aiv-ego-speed.service` runs `~/ego_speed_tx.py` at boot, sends 0x22 EGO_SPEED to STM32 (documented 5 Hz; the test copy in chat sends fixed 12.5 km/h at 10 Hz) — ❓ read the real file/service | 📝 |
| Camera | Intel RealSense D435i, color only, python venv `~/realsense-venv` | ✅ |
| Flashing STM32 from Pi5 | `st-flash --connect-under-reset --reset write <bin> 0x8000000` (stlink-tools installed) | ✅ |
| CP2102 USB-UART adapter | **not present** on the rear car (user confirmed 10-11) | ✅ |
| Hostname / user (seen in logs) | `user@car-rear`; hotspot IP like 172.20.10.x changes | 📝 |

## Power
- Car cigarette-lighter 12 V → 5 V 5 A DC-DC (Pi5) ; Pi5 5 V → **XL6009 boost → MR76 radar** (MR76 accepts 6–32 V).
- Suspected weak point (hypothesis): 5 V sag on this chain may cause radar silence. One earlier event: non-standard charger on Pi5 → radar nearly silent, fixed after full power cycle (cause unconfirmed).
- Monitor with `pi5_power_log_v1.py` (EXT5V_V, throttled, temperature); `sys.csv` in drive logger also records throttled/temp.

## Buzzer (in progress by the user)
Piezo, 3–24 V (rated 12 V), ≤10 mA, ~3.9 kHz, 36×12 mm, 145 mm wire. **Active type (S10: user listening report, 2026-10-25): when switched on with the FRONT car's `buzzer_test.py` it gave a continuous "beep—"; type determined from that, exact model NOT confirmed. The REAR unit is presumed to be the same part (not separately tested).** Direct GPIO drive possible (3.3 V quieter);
alternative 5 V + NPN with ~1 kΩ base resistor. **Connected (user, 2026-10-25): Pi5 physical pin 37 = BCM GPIO26 (signal) and physical pin 34 = GND** (the earlier candidate GPIO17/pin 11 is not used). Type active/passive still UNCONFIRMED; software ready, not yet driven: see `BUZZER_BENCH.md`. Avoid the XL6009 rail until the radar issue is understood.
Do not wire or write buzzer code unless the user asks.

## Front car (for reference, separate machine)
RPi4 + STM32. RPi4 STM32 link planned `dtoverlay=uart3` → `/dev/ttyAMA1` (doc 2026-09-19, **outdated**: it also says
OLED on I2C2 PB10/PB3; the delivered `front_v4b` uses OLED on **I2C1 PB8/PB9**). Honda CR-V 2009: OBD not working → GPS speed.

### Front vs rear buzzer wiring (S10)
| | 후방 (Pi5) | 전방 (Pi4) |
|---|---|---|
| 신호 | 물리 핀 37 = BCM GPIO26 | **같은 물리 핀 37** = BCM GPIO26 |
| GND | 물리 핀 34 | **다른 GND 핀**(핀 번호는 사용자 확인 필요) |
- 앞차 `buzzer_test.py` 의 **기본 핀은 18** 이므로, 전방에서 핀 37 에 연결했다면 반드시 `--gpio 26` (또는 앞차 본 프로그램의 `--buzzer-gpio 26`)으로 지정해야 한다. 지정을 빼면 BCM18(물리 핀 12)로 나가 소리가 안 난다.


## CODEX1 정정 (2026-10-25)
- 사용자 보고: 부저는 active형이며 파이5/파이4 물리 37번에서 울림을 확인하셨습니다. 위의 미확인·미구동 표기는 이전 세션 기준입니다. CODEX1에서는 실제 GPIO를 시험하지 못했습니다.
- 이번 실제 부저 3초 시험은 사용자 승인 범위에 포함됩니다. 다만 현재 실행 환경에는 GPIO 장치가 노출되지 않아 미실행입니다.
- ESP32 후방 소스는 코어 3.x 호환 수정 후 컴파일 성공했습니다. 무선 검증과는 별개입니다.
- 상세 최신 결과: CODEX1_REPORT.md.
