# RULES — team development rules (13)

Source: the team's rules document pasted in chat. Keep as binding unless the user says otherwise.

## I. AUTOSAR 4-layer separation
1. **No layer-boundary violations.** STM32 = safety judgement; Raspberry Pi = AI inference + V2V.
   Pi code must not call or imitate the STM32 safe-distance calculation.
2. **STM32 safety loop never blocks.** 50 Hz deterministic loop must not wait for the Pi5.
   If the Pi5 dies or V2V drops, radar-based local warning keeps running. Non-blocking + timeout only.
3. **Keep Pi5 CPU core isolation.** YOLOv8 on cores 0–1, V2V on cores 2–3. Do not change core affinity.
4. **Sensor values through an abstraction.** Upper logic must not know the sensor source.
   Speed is read only through one function (`get_speed()`); never call OBD/GPS drivers from upper code.

## II. ISO 26262 fail-safe
5. **Timeout + Safe Mode fallback on every sensor interface** (radar/IMU/GPS/OBD), reference 100 ms,
   fall back to conservative fixed parameters.
6. **IWDG watchdog ≈ 40 ms.** Share before adding anything longer; re-check kick timing.
7. **Frame validation failure → drop immediately, no retry** (STM32↔Pi and V2V). Retries must not hold the safety loop.
8. **Safety constants are not changed on personal judgement**: K range 0.3–1.0, sensor timeout 100 ms, watchdog 40 ms. Code review first.
9. **Radar reception (MCP2515) is interrupt-driven.** Only 2 RX buffers → polling overflows. Use INT pin + CAN-ID filter.
   Path: MR76 → MCP2515 → STM32 (SPI) → Pi5 (UART).

## III. SAE J2735 BSM
10. **BSM "brake status" = IMU deceleration estimate** (≤ −0.1 g → BRAKE_ACTIVE). Real pedal signal unavailable.
11. **BSM ≤ 50 bytes**, direct struct serialization with CRC16 (no ASN.1 UPER), currently 26 B.
    Keep `bsm_protocol.py` (structure) separate from `transport.py` (UDP / ESP-NOW) so only the transport changes for a real OBU.

## IV. Sensor source rules (current test cars)
12. **Speed: OBD first, GPS fallback.** Rear = SM3 (OBD OK), front = Honda (OBD not working → GPS always).
    GPS speed: raise rate to ≥5 Hz, zero-out or filter at very low speed, mark source/age in BSM (reserved STATUS bits).
    Rear car can cross-check front car speed (own OBD speed + radar relative speed vs front BSM GPS speed);
    on mismatch suspect latency/matching error and **trust radar for danger decisions**.
13. **Brake pedal opening/pressure is unavailable.** OBD only uses speed (0x0D) and RPM (0x0C).
    No code or document may claim the pedal opening rate is used.
