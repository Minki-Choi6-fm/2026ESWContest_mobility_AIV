# PROTOCOLS — STM32↔Pi5 UART (v0.2) and V2V BSM

Source: `AIV_STM32-RPi_UART_메시지규격_v0.2.md`, `V2V_통신_프로토콜_설계.md` (draft v0.1, 2026-09-23).
Both are **drafts "pending team review"**; the code on the Pi5 is the truth. Check `fusion.py` (decoder) and `bsm_protocol.py`.

## 1. UART frame (both directions)
`STX 0x02 | MSG_TYPE | LEN | PAYLOAD (LEN bytes) | CHECKSUM | ETX 0x03`
- CHECKSUM = XOR of MSG_TYPE, LEN and all PAYLOAD bytes (STX/ETX excluded).
- Frame length = LEN + 5. LEN is fixed per type; wrong LEN → discard and resync on next STX.
- Little-endian, signed = two's complement. 115200 8N1. Bad checksum → drop, never retry.
- Payload bytes can contain 0x02/0x03 (boundary found by LEN, no byte stuffing).

### STM32 → Pi5
| Type | Name | LEN | Rate | Notes |
|---|---|---|---|---|
| 0x10 | SENSOR_SUMMARY | 11 | 50 Hz | TS u32, K u8 (0xFF=not computed), TARGET_DIST u16 cm (0xFFFF none), REL_SPEED i16 0.01 m/s (0x7FFF none), DANGER u8 (0 safe,1 caution,2 danger,0xFF unknown), SENSOR_STATUS u8 |
| 0x11 | HEARTBEAT | 6 | 10 Hz | UPTIME u32 ms, SENSOR_STATUS, RESET_CAUSE (0 power, 1 IWDG, 2 other). Pi marks STM32 offline after 500 ms |
| 0x12 | RADAR_OBJECT | 14 | per radar object (one frame each; see correction below) | TS u32, OBJ_ID, DIST_LONG i16 cm, DIST_LAT i16 cm, VREL_LONG i16 0.01 m/s, RCS i8 (0.5 dBsm, **signed**), DYN_PROP, CYCLE_INFO (0xFF = not provided). Filter: RCS ≥ −5 dBsm; the vehicle-class filter exists in the code but is **OFF in the builds on record** (`#define TEST_ALL_CLASSES 1` in v16/v17/v18c) — see correction below |
| 0x13 | IMU_DATA | 23 | 20 Hz | acc mg, gyro 0.1°/s, roll/pitch/yaw 0.1°, FLAGS (VALID, BRAKE_ACTIVE). Vehicle axes (+X fwd, +Y left, +Z up) |
| 0x14 | OBD_DATA | 5 | ~3 Hz | SPEED km/h (0xFF none), RPM (0xFFFF none), AGE_MS |

SENSOR_STATUS bits: 0 RADAR_OK (frame within 100 ms), 1 IMU_OK, 2 OBD_OK, 3 CAN_CTRL_OK, 4 SAFE_MODE, 5 BRAKE_ACTIVE.

### Pi5 → STM32
| Type | Name | LEN | Notes |
|---|---|---|---|
| 0x20 | DISPLAY_WARNING | 4 | SENDER_ID, DANGER_LEVEL, DISTANCE cm — (front-car display) |
| 0x21 | LINK_STATUS | 1 | V2V_CONNECTED |
| 0x22 | EGO_SPEED | 5 | SPEED u16 0.01 km/h, SOURCE (0 OBD, 1 GPS, 0xFF none), AGE_MS — **implemented** (OLED shows `SPD:xx.x GPS`) |
Whether 0x10–0x14 binary frames are emitted by the current firmware: v0.2 doc §10 said "not yet" at 09-30, but
the 10-16 `drive_logger` is written to record types 0x10–0x14 from `/dev/ttyAMA2` and its drive folders contain `radar.csv`/`frames.csv` 📝 (the rows themselves were not inspected here). Re-check on the real device.
Danger/K fields in 0x10 are still 0xFF in v18c firmware.

Example (SENSOR_SUMMARY): `02 10 0B 40 E2 01 00 FF C4 09 A2 FE 00 27 F1 03`.

## 2. V2V BSM (26 bytes, 10 Hz, UDP broadcast port 9000 / ESP-NOW)
`SYNC 0xAA | VER 0x01 | VEHICLE_ID | SEQ u16 | TIMESTAMP u32 ms | LAT i32 ×1e7 | LON i32 ×1e7 | SPEED u16 0.01 km/h | HEADING u16 0.01° | K_VALUE u8 ×100 (0xFF N/A) | STATUS u8 | CRC16-CCITT u16 | END 0x55`
- Vehicle IDs: 1 rear, 2 front, 3 side (virtual car).
- STATUS: bit0 GPS_VALID, bit1 DANGER_ACTIVE, bit2 SENSOR_DEGRADED, bit3 SAFE_MODE, bit4 BRAKE_ACTIVE (IMU), bit5 SPEED_SRC_GPS, bit6 SPEED_STALE, bit7 reserved. (bit4–6 are *proposals*.)
- Receive order: SYNC/END → CRC (drop on fail) → SEQ monotonic (uint16 wrap aware) → freshness (**300 ms STALE**) → STATUS (GPS_VALID=0 → ignore position).
- Fail-safe: V2V all-STALE → rear STM32 keeps local radar warning independently.
- Open items from the doc: coordinate precision vs GPS module, port 9000 conflicts, 10 vs 20 Hz on ESP-NOW, 300 ms threshold tuning, byte order (little-endian assumed; confirm in `bsm_protocol.py`).

## 3. Bandwidth (115200 bps ≈ 11.5 kB/s)
Estimated STM32→Pi load ≈ 3.0 kB/s (~26 %) assuming radar 20 Hz × 4 objects (assumption).

## Corrections measured on the 10/16 drive logs (S7/S8, 2026-10-25) — the original draft text above was not true for the test builds
- **"Only vehicle class passes" is NOT true for builds with `TEST_ALL_CLASSES 1`** (main_v16.c, main_v17.c, main_v18c.c all have it): every object class (including point-like poles/fences/people) is sent. Measured per log: `pub` (frames handed to the UART queue) = 4.7 / 2.8 / 3.6 × the number of vehicle-class objects (`veh`) in RADSTAT. The class itself (`objClass`) is NOT transmitted (CYCLE_INFO byte is 0xFF), so the Pi5 cannot tell classes apart. With `TEST_ALL_CLASSES 0` only class 1 (vehicle) would be sent (see `FIRMWARE_V18E_PROPOSAL.md`).
- **"≤ 4 objects per radar cycle" is NOT true:** objects per radar cycle (same STM32 timestamp) averaged 6.2, p95 15-16, **max 22-24**; objects per second averaged 182-230 and reached 745-777/s (the line carries at most ≈527 0x12/s next to the other frames at 115200 baud).
- **Bandwidth (§3 estimate of ≈3.0 kB/s is too low):** measured ≈ 5.7 kB/s average (≈ 50 % of the 11.52 kB/s line), with seconds above 100 % (see `FIRMWARE_TX_QUEUE.md`).
- `DYN_PROP` meaning in the firmware (main_v18c.c comment): 0 moving, 1 stationary, 2 oncoming, 3 left-crossing, 4 right-crossing, 5 unknown, 6 stopped. Measured against own speed: dyn 1 = ground-fixed in 89 % of samples, dyn 0 = moving in 99 %, dyn 2 looks oncoming in only 72 %, dyn 5 behaves like a moving object (consistent with "unknown").
