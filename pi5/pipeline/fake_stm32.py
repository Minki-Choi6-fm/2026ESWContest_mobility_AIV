#!/usr/bin/env python3
"""Desk-test fake STM32: replays <drive>/frames.csv (0x10-0x14) into a pty at the original timing.

Creates a NEW pty (never touches /dev/ttyAMA2 or any real port). Point live_receiver.py --port at the printed path.
Scenarios (times are seconds since playback start):
  --drop-radar SEC [--drop-radar-at 10] [--drop-radar-mode targets|dead]   no 0x12 for SEC s.
        targets: radar alive but no object (RADAR_OK stays 1).  dead (default): also RADAR_OK cleared, SAFE_MODE set [mimic = hypothesis]
  --obd-fail SEC   [--obd-fail-at 10]     OBD answers stop: 0x14 only every --obd-fail-period s (default 3) with the LAST speed,
                                          RPM 0xFFFF and growing AGE_MS (as main_v18c.c is described); OBD_OK cleared
  --corrupt P      [--corrupt-at 10 --corrupt-for 10]   each byte replaced by a random byte with probability P inside the window
  --stm-reset      [--reset-at 10 --reset-gap 9.5]      silence for the gap, then timestamps/uptime restart; reset cause = 2 for 5 s
"""
import argparse, csv, os, pty, random, signal, sys, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import build_frame

TS_AT = {0x10: 0, 0x11: 0, 0x12: 0, 0x13: 0}      # u32 ms clock at payload offset 0 (0x14 has none)


def load(drive, start_off):
    rows = []
    with open(os.path.join(drive, "frames.csv"), newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            rows.append((float(row[0]), int(row[1], 16), bytes.fromhex(row[2])))
    t0 = rows[0][0] + start_off
    return [x for x in rows if x[0] >= t0], t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("drive"); ap.add_argument("--from", dest="start", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=1.0); ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--link", help="create a symlink to the pty slave here (stable path for tests)")
    ap.add_argument("--drop-radar", type=float, default=0); ap.add_argument("--drop-radar-at", type=float, default=10)
    ap.add_argument("--drop-radar-mode", choices=["targets", "dead"], default="dead",
                    help="targets: only 0x12 stops (radar alive, no object; RADAR_OK stays 1). dead: 0x12 stops AND RADAR_OK=0, SAFE_MODE=1")
    ap.add_argument("--obd-fail", type=float, default=0); ap.add_argument("--obd-fail-at", type=float, default=10)
    ap.add_argument("--obd-fail-period", type=float, default=3.0)
    ap.add_argument("--corrupt", type=float, default=0); ap.add_argument("--corrupt-at", type=float, default=10)
    ap.add_argument("--corrupt-for", type=float, default=10)
    ap.add_argument("--stm-reset", action="store_true"); ap.add_argument("--reset-at", type=float, default=10)
    ap.add_argument("--reset-gap", type=float, default=9.5)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    rows, t0 = load(a.drive, a.start)

    master, slave = pty.openpty()
    tty.setraw(slave)                                  # behave like a raw UART (no CR/NL translation)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if a.link:
        if os.path.lexists(a.link):
            os.unlink(a.link)
        os.symlink(path, a.link)
    print(f"[fake_stm32] pty slave: {path}" + (f"  link: {a.link}" if a.link else ""), flush=True)

    pending = bytearray()
    st = dict(sent={t: 0 for t in (0x10, 0x11, 0x12, 0x13, 0x14)}, dropped_radar=0, dropped_reset=0, dropped_obd=0,
              synth_obd=0, overflow=0, corrupted_bytes=0, corrupted_frames=0, corrupted_by_type={})
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))

    def flush():
        if pending:
            try:
                n = os.write(master, pending); del pending[:n]
            except BlockingIOError:
                pass

    def emit(typ, payload, rel):
        fr = bytearray(build_frame(typ, payload))
        if a.corrupt > 0 and a.corrupt_at <= rel < a.corrupt_at + a.corrupt_for:
            hit = False
            for i in range(len(fr)):
                if rnd.random() < a.corrupt:
                    fr[i] = rnd.randrange(256); st["corrupted_bytes"] += 1; hit = True
            st["corrupted_frames"] += hit
            if hit:
                st["corrupted_by_type"][f"0x{typ:02X}"] = st["corrupted_by_type"].get(f"0x{typ:02X}", 0) + 1
        if len(pending) > 2048:                       # tiny queue on purpose: when the pty is full, data is LOST like a UART overrun
            st["overflow"] += 1
            return
        pending.extend(fr); st["sent"][typ] += 1

    in_win = lambda rel, at, dur: dur > 0 and at <= rel < at + dur
    last_obd = None                                    # (speed, rel) of last good 0x14
    next_synth = a.obd_fail_at + a.obd_fail_period
    base_ts = None                                     # STM32 clock origin after a simulated reset
    announced = set()
    start = time.monotonic()
    for t, typ, pl in rows:
        if stop:
            break
        rel = (t - t0) / a.speed
        if a.duration and rel > a.duration:
            break
        while time.monotonic() - start < rel and not stop:
            flush(); time.sleep(0.002)
        pl = bytearray(pl)

        def note(k, msg):
            if k not in announced:
                announced.add(k); print(f"[fake_stm32] t={rel:6.1f}s {msg}", flush=True)

        # --- simulated STM32 reset: silence, then clock restarts
        if a.stm_reset and a.reset_at <= rel < a.reset_at + a.reset_gap:
            note("rst", "STM32 reset: silence begins"); st["dropped_reset"] += 1; continue
        if a.stm_reset and rel >= a.reset_at + a.reset_gap:
            note("rst2", "STM32 back: clock restarted, reset cause=2")
            if typ in TS_AT:
                ts = int.from_bytes(pl[0:4], "little")
                if base_ts is None:
                    base_ts = ts
                pl[0:4] = ((ts - base_ts + 800) & 0xFFFFFFFF).to_bytes(4, "little")
                if typ == 0x11 and rel < a.reset_at + a.reset_gap + 5:
                    pl[5] = 2
        # --- radar dropout
        radar_down = in_win(rel, a.drop_radar_at, a.drop_radar)
        obd_down = in_win(rel, a.obd_fail_at, a.obd_fail)
        if radar_down and typ == 0x12:
            note("rd", "radar frames suppressed"); st["dropped_radar"] += 1; continue
        if not radar_down and "rd" in announced:
            note("rd_end", "radar frames resume")
        if typ in (0x10, 0x11):
            i = 10 if typ == 0x10 else 4
            if radar_down and a.drop_radar_mode == "dead":
                pl[i] = (pl[i] & ~0x01) | 0x10         # RADAR_OK off, SAFE_MODE on
            if obd_down:
                pl[i] &= ~0x04                          # OBD_OK off
        # --- OBD failure: synthetic 0x14 with growing AGE_MS
        if typ == 0x14:
            if not obd_down:
                if pl[0] != 0xFF:
                    last_obd = (pl[0], rel)
                if "of" in announced:
                    note("of_end", "OBD answers again")
            else:
                note("of", "OBD failing: 0x14 every %.1fs with last speed + growing AGE_MS" % a.obd_fail_period)
                st["dropped_obd"] += 1; continue
        while obd_down and next_synth <= rel:
            spd, lt = last_obd if last_obd else (0xFF, a.obd_fail_at)
            age = min(0xFFFE, int((next_synth - lt) * 1000))
            emit(0x14, bytes([spd]) + (0xFFFF).to_bytes(2, "little") + age.to_bytes(2, "little"), next_synth)
            st["synth_obd"] += 1; next_synth += a.obd_fail_period
        if not obd_down and rel >= a.obd_fail_at + a.obd_fail:
            next_synth = float("inf")
        emit(typ, bytes(pl), rel)
        flush()
    print("[fake_stm32] done:", st, flush=True)
    if a.link and os.path.islink(a.link):
        os.unlink(a.link)


if __name__ == "__main__":
    main()
