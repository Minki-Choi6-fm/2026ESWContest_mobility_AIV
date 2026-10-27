# PIPELINE — end-to-end integration plan (rear car)  [drop into docs/handoff/]

Goal: radar/IMU/OBD/GPS/camera → STM32 → Pi5 → target + danger → V2V → warning on BOTH cars.
Status tags: ✅ exists (verified in logs/code) · 📝 exists on paper · ❌ missing · ❓ unknown, check.

## 1. Data flow
```
MR76 radar ─CAN→ MCP2515 ─SPI→ STM32 ─UART 0x12→ Pi5 ┐
IMU ─UART→ STM32 ─0x13→ Pi5 (brake est. BRAKE_ACTIVE in STM32)     │
OBD (SM3) ─UART→ STM32 ─0x14→ Pi5 ──┐                               │
GPS (gpsd) ────────────────────────┴→ get_speed() [OBD→GPS]        │
                       Pi5 → STM32 0x22 EGO_SPEED ✅ (service aiv-ego-speed)
RealSense + YOLOv8n ──→ Pi5 camera_detector ✅                      │
Pi5: fusion (radar↔camera candidates) ✅ (not safety logic)         │
Pi5: TARGET SELECTION (which radar object = lead vehicle) ❌/❓     │
        radar corridor + camera check + V2V BSM match               │
STM32: K value + DANGER_FLAG ❌ (0xFF in v18c)                      │
Pi5: build BSM (STATUS bits) → v2v_node → UDP/ESP-NOW ✅/📝 (code not on Pi5, see v2v/)
Front car: receive BSM → 0x20 DISPLAY_WARNING → STM32 OLED/buzzer ❓
Rear car: local warning (OLED ✅, buzzer in progress by user)
```

## 2. Gaps found (verify, then ask the user who owns each)
1. **Danger decision does not exist in firmware** (K=0xFF, DANGER=0xFF). Rule 1 says STM32 owns the safe-distance math, Pi must not imitate it.
2. **No message tells the STM32 which radar object is the lead vehicle.** UART v0.2 has 0x20/0x21/0x22 downlink only; lead-vehicle selection is assigned to the Pi (doc §3 0x12). Either add a downlink (e.g. 0x23 TARGET_SELECT: OBJ_ID + age) or let the STM32 pick the nearest in-corridor object. **Team decision needed.**
3. **V2V code is not on the Pi5** (v2v_node.py, bsm_protocol.py, transport.py, freshness.py). A copy from 2026-09-27 is in v2v_stack.zip; teammates may have newer ones. Ask before overwriting.
4. Real 2-car wireless V2V test never done. ESP-NOW 2×ESP32 ping test succeeded; bridge .ino exists.
5. Front car path 0x20 → OLED/buzzer: not verified.
6. No end-to-end latency/PDR numbers yet (needed for the report).

## 3. Staged plan (each stage: small, tested, reversible; ask before flashing)
- **S0 Inventory** (read-only): run tools/snapshot_pi5.sh; list who owns what; place v2v_stack under ESWContest_ws/v2v/ (no overwrite).
- **S1 Pi5 ingest**: one process owns /dev/ttyAMA2 read; decode 0x10–0x14 (reuse fusion.py RadarPacketDecoder, extend to all types); shared state object with per-source timestamp/age; timeouts 100 ms (rule 5); log CSV.
- **S2 get_speed()**: single function OBD→GPS fallback with source/age (rule 4, 12). Keep aiv-ego-speed service behavior intact.
- **S3 BSM from real data**: replace DummySensorSource in v2v_node.py with real GPS/speed/status; keep bsm_protocol/transport separation (rule 11). Loopback test → 2-car test.
- **S4 Target selection**: camera+radar candidate → lead object ID; cross-check with front BSM speed (rule 12); radar wins on mismatch. Decide message to STM32 (gap 2).
- **S5 Danger decision in STM32** (firmware change, ask first): K from IMU brake estimate, d = v²/(2·K·g)+v·t, DANGER 0/1/2, Safe Mode 0xFF. Constants per rule 8.
- **S6 Warning loop**: rear local warning; BSM DANGER_ACTIVE → front 0x20 DISPLAY_WARNING; Safe Mode/stale handling.
- **S7 Measure**: latency radar→OLED, V2V latency, PDR, front-car TTC at warning (report metrics).

## 4. Run/ops constraints
- CPU: YOLO cores 0–1, V2V cores 2–3 (rule 3) — use taskset/affinity, do not change existing settings.
- Serial: only ONE reader on /dev/ttyAMA2 and on /dev/ttyACM0. The ego-speed service only writes (ok next to a reader).
- Long runs in tmux; record with drive_logger + pi5_power_log; never present unmeasured numbers.
