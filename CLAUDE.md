# CLAUDE.md — AIV rear-car (Raspberry Pi 5 + STM32) handoff

You are being started on the **rear car's Raspberry Pi 5** of team AIV
(제24회 임베디드SW경진대회, 자동차/모빌리티, finalist). The user is the team leader
(최민기). Teammates are working on the same Pi5 in parallel.

## Read this first: the Pi5 may differ from these docs
These docs were written from a chat history that ended around **2026-10-16 ~ 10-24**.
Coworkers may have edited, added or deleted files on the Pi5 since then.
**Treat every file name, version and status below as a claim to verify, not a fact.**

Before changing anything:
1. Run `bash tools/snapshot_pi5.sh` (read-only) and read its output.
2. Compare with `docs/PROJECT_STATE.md` and tell the user what differs.
3. Never overwrite or delete a file you did not create. Before editing an existing
   file, copy it to `<name>.bak_YYYYMMDD_HHMM` in the same folder.
4. If the folder is not a git repo, ask the user before `git init`.
   Do not commit/push without being asked.

## Language and style
- The user speaks Korean; answer in **polite, easy Korean (존댓말)** unless asked otherwise.
  Code and comments may be English. Explain simply; go technical only if asked.
- Verify facts before stating them. Label guesses as hypotheses ("추측").
- Short outputs. Do the next concrete step instead of asking repeated questions.
- Never invent measurements. Missing data → say so and propose how to measure.

## What the system is (one paragraph)
A V2V safe-distance warning system between a **commercial truck/rear car** and a
**passenger/front car**. Rear car: Pi5 (camera + YOLO + decision/V2V) + NUCLEO-F446RE
(real-time radar/IMU/OBD/OLED/safety). Front car: RPi4 + STM32 + OLED. Cars exchange
26-byte BSM frames at 10 Hz (UDP now, ESP-NOW planned/partly tested). Both cars get warned
(bidirectional). See `docs/PROJECT_STATE.md`.

## Hard rules (do not break; full list in docs/RULES.md)
- STM32 safety loop must never block on the Pi5. Pi5 or V2V dying must not stop local radar warning.
- **Do NOT change**: IWDG ≈40 ms, sensor timeout 100 ms, K-value range 0.3–1.0,
  Pi5 CPU core split (YOLO cores 0–1, V2V cores 2–3).
- Speed only via one function (`get_speed()`: OBD first, GPS fallback). No brake-pedal signals.
- Bad checksum/CRC → drop immediately, no retry.
- Do not mix BSM structure (`bsm_protocol.py`) with transport (`transport.py`).

## Serial ports are exclusive
`/dev/ttyAMA2` (STM32 binary link) and `/dev/ttyACM0` (ST-Link debug text) can be read by
**one program at a time**. Stop other readers (`drive_logger`, `cat`, overlay tools) first.
Check with `fuser -v /dev/ttyAMA2 /dev/ttyACM0` before opening them.

## Do not do without asking
- Flashing STM32 firmware (`st-flash`), changing systemd services, editing `/boot/firmware/config.txt`,
  changing `/etc/default/gpsd`, installing system packages, rebooting.
- Editing teammates' files (V2V pipeline, targeting) — ask who owns it.
- Touching the buzzer wiring/code: the user is integrating it on their own.

## Where things are documented
| File | Content |
|---|---|
| docs/PROJECT_STATE.md | goal, architecture, status, known issues, next steps |
| docs/RULES.md | the 13 team development rules |
| docs/HARDWARE_REAR.md | pins, ports, wiring, power |
| docs/PROTOCOLS.md | STM32↔Pi5 UART v0.2, BSM V2V frame |
| docs/PI5_TOOLS_AND_WORKFLOW.md | scripts, how to record/analyze/flash |
| tools/snapshot_pi5.sh | read-only state report to run first |

Confidence tags used in these docs: ✅ verified in logs/by the user, 📝 documented but not
re-checked, ❓ unknown/needs check.
