#!/usr/bin/env python3
"""S11-b desk rehearsal: REAL STM32 frames (read-only) -> receiver -> brain -> local BSM builder output (NEVER sent anywhere) + mock buzzer.
* Opens the UART O_RDONLY (live_receiver.open_readonly). Never writes. Aborts when a process other than the known write-only ego_speed_tx.py holds the port.
* No UDP/ESP-NOW transport is created: BSM bytes are only built and decoded locally (status bits recorded).
* Buzzer = gpiozero MockFactory only. No camera, no YOLO.
usage: python3 -I desk_rehearsal.py --duration 300 --out DIR [--uart /dev/ttyAMA2]"""
import argparse, collections, csv, json, os, re, select, statistics as S, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2v_adapter as V
from bsm_builder import build_fields
from brain import Brain
import constants as C
from gps_position import GpsPositionClient
from live_receiver import Receiver, open_readonly
from speed_select import get_speed
from pipeline_main import output_flags

EVAL_HZ = 20
KNOWN_WRITER = "ego_speed_tx.py"


def holders(port):
    r = subprocess.run(["fuser", port], capture_output=True, text=True)
    pids = [int(x) for x in r.stdout.split() if x.isdigit()]
    out = []
    for p in pids:
        try:
            cmd = open(f"/proc/{p}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            cmd = "?"
        out.append((p, cmd))
    return out


def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttyAMA2"); ap.add_argument("--duration", type=float, default=300)
    ap.add_argument("--out", required=True); ap.add_argument("--gpsd-port", type=int, default=2947)
    ap.add_argument("--allow-any-holder", action="store_true", help="TEST ONLY (fake pty)")
    a = ap.parse_args()
    if not a.allow_any_holder:
        bad = [(p, c) for p, c in holders(a.uart) if KNOWN_WRITER not in c and p != os.getpid()]
        if bad:
            print("REFUSED: other processes hold", a.uart, bad); sys.exit(2)
    os.makedirs(a.out, exist_ok=True)
    from gpiozero.pins.mock import MockFactory
    from buzzer_out import BuzzerSink
    buzzer = BuzzerSink(kind="active", pin_factory=MockFactory())
    gps = GpsPositionClient(port=a.gpsd_port)
    rx = Receiver(gps, None, 5.0, None)
    brain = Brain(k_profile=None, reset_k=False, k_fixed=None)       # no profile file: nothing is stored
    fd, _ = open_readonly(a.uart)
    td = time.monotonic()
    while time.monotonic() - td < 2.0:                              # stale kernel backlog
        try:
            if not os.read(fd, 65536): break
        except BlockingIOError: break
    f = open(os.path.join(a.out, "rehearsal.csv"), "w", newline=""); w = csv.writer(f)
    w.writerow(["t", "level", "reason", "tgt_id", "tgt_d", "tgt_lat", "tgt_vrel", "speed_kmh", "speed_src", "status", "radar_alive", "radar_has_target",
                "n_obj_fresh", "supp", "bsm_status_raw", "bsm_danger", "eval_ms"])
    t0 = time.monotonic(); next_eval = t0 + 0.2
    pend = []                              # monotonic arrival times of 0x12 frames not yet followed by an evaluation
    lat = []; cnt = collections.Counter(); sec = collections.defaultdict(collections.Counter)
    levels = collections.Counter(); reasons = collections.Counter(); prev_safe = None; safe_entries = 0; safe_secs = 0
    last_summary_rx = None; sum_gaps = []
    try:
        while time.monotonic() - t0 < a.duration:
            now = time.monotonic()
            gps.connect(now)
            rl = [fd] + ([gps.sock] if gps.sock else [])
            r, _, _ = select.select(rl, [], [], max(0.0, min(0.02, next_eval - now)))
            now = time.monotonic()
            for s in r:
                if s is fd:
                    try: data = os.read(fd, 4096)
                    except BlockingIOError: data = b""
                    if data:
                        for fr in rx.feed(data, now, time.time()):
                            brain.on_frame(fr, now); cnt[f"rx_0x{fr.typ:02x}"] += 1; sec[int(now - t0)][f"0x{fr.typ:02x}"] += 1
                            if fr.typ == 0x12: pend.append(now)
                            if fr.typ == 0x10:
                                if last_summary_rx is not None: sum_gaps.append((now - last_summary_rx) * 1000)
                                last_summary_rx = now
                                sf = bool(fr.data["status"] & 0x10)
                                if sf and not prev_safe: safe_entries += 1
                                prev_safe = sf
                                if sf: safe_secs += 0
                elif gps.sock and s is gps.sock:
                    gps.pump(now)
            rx.poll(now)
            if now >= next_eval:
                next_eval += 1.0 / EVAL_HZ
                c0 = time.perf_counter()
                sp = get_speed(rx.state, gps, now); own = gps.position(now)
                out = brain.evaluate(rx.state, sp, None, True, now, own_fix=own, own_heading=gps.last_heading)
                done = time.monotonic()
                for ta in pend: lat.append((done - ta) * 1000)
                pend.clear()
                disp, buz = output_flags(out.level)
                buzzer.update(now, out.level, out.reason, dict(display=disp, buzzer=buz))      # mock pin
                fields = build_fields(rx.state, sp, own, gps.last_heading, now, danger_level=out.level, k_measured=out.k_measured)
                dd = V.decode(V.encode(fields, 1, 0))                                           # local build + decode only
                tg = out.target
                st = rx.state.sensor_status(now)
                w.writerow([f"{now - t0:.3f}", out.level, out.reason[:160], "" if tg is None else tg.id, "" if tg is None else f"{tg.long_m:.1f}",
                            "" if tg is None else f"{tg.lat_m:.1f}", "" if tg is None else f"{tg.vrel_ms:.2f}", "" if sp.speed_kmh is None else f"{sp.speed_kmh:.1f}",
                            getattr(sp, "source", ""), "" if st is None else st, int(rx.state.radar_alive(now)), int(rx.state.radar_has_target(now)),
                            len(rx.state.fresh_radar_objects(now)), "|".join(out.supp), dd["status_raw"], int(dd["danger_active"]), f"{(time.perf_counter() - c0) * 1000:.2f}"])
                levels[out.level] += 1; cnt["evals"] += 1; sec[int(now - t0)]["evals"] += 1
                key = re.sub(r"[-+]?\d+(\.\d+)?", "#", out.reason.split("|")[-1].strip())[:90] if out.reason else ""
                reasons[key] += 1
    finally:
        f.close(); os.close(fd)
        buzzer.safe_state("rehearsal end")
    dur = time.monotonic() - t0
    summ = dict(duration_s=round(dur, 1), counts=dict(cnt), levels={str(k): v for k, v in levels.items()}, safe_mode_entries_seen_in_0x10=safe_entries,
                updates_per_s=round(cnt["evals"] / dur, 2), latency_ms=dict(n=len(lat), p50=round(pct(lat, .5), 1), p90=round(pct(lat, .9), 1), max=round(max(lat), 1) if lat else None),
                summary_gap_ms=dict(p50=round(pct(sum_gaps, .5), 1), p99=round(pct(sum_gaps, .99), 1), max=round(max(sum_gaps), 1) if sum_gaps else None),
                top_reasons=reasons.most_common(12), status_hist={hex(k): v for k, v in rx.status_hist.items()},
                buzzer=dict(on_transitions=buzzer.transitions, final_on=bool(buzzer.on), mock=True),
                per_second_0x12=[sec[i]["0x12"] for i in range(int(dur))], per_second_0x10=[sec[i]["0x10"] for i in range(int(dur))])
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps({k: v for k, v in summ.items() if not k.startswith("per_second")}, ensure_ascii=False, indent=1))
    rx.close(time.monotonic(), time.time())


if __name__ == "__main__":
    main()
