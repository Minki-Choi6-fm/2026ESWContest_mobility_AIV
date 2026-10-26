#!/usr/bin/env python3
"""List the cut-in promotions of a replay (logs/s4_replay/<drive>.csv, column cutin) with the |lat| trajectory of the promoted radar id, to judge by eye
whether it looks like a real lane change. Offline, read-only. usage: python3 -I cutin_review.py <drive_dir> [--n 8]"""
import bisect, csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from s4_replay import load_rows
from uart_ingest import ExtendedDecoder, build_frame

drive = sys.argv[1]; n_show = int(sys.argv[sys.argv.index("--n") + 1]) if "--n" in sys.argv else 8
name = os.path.basename(drive.rstrip("/"))
rows = load_rows(drive); t0 = rows[0][0]
dec = ExtendedDecoder(); obs = {}                                  # id -> [(t_rel, long, |lat|, vrel)]
for tv, typ, pl in rows:
    if typ == 0x12:
        for fr in dec.feed(build_frame(typ, pl), tv):
            if fr.typ == 0x12:
                r = fr.radar; obs.setdefault(r.object_id, []).append((tv - t0, r.forward_m, abs(r.left_m), r.relative_mps))
ev = list(csv.DictReader(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs", "s4_replay", name + ".csv"))))
promo = [(float(ev[i]["t_s"]), ev[i]["target_id"]) for i in range(1, len(ev)) if ev[i]["cutin"] == "1" and ev[i - 1]["cutin"] != "1"]
print(f"\n## {name}: {len(promo)} cut-in promotion(s) in {len(ev) * 0.05:.0f} s of replay")
if not promo:
    sys.exit()
pick = [promo[int(i * (len(promo) - 1) / max(1, n_show - 1))] for i in range(min(n_show, len(promo)))] if len(promo) > 1 else promo
print("| t s | id | d at promotion m | side-zone time s | lateral speed m/s (|lat| 2.0 s before -> at promotion) | |lat| samples t-2.0 .. t+0.5 (every 0.5 s) | looks like a lane change? |\n|---|---|---|---|---|---|---|")
for t, tid in pick:
    o = obs.get(int(tid), []); ts = [x[0] for x in o]
    def at(tt):
        i = bisect.bisect_left(ts, tt)
        c = [o[j] for j in (i - 1, i) if 0 <= j < len(o)]
        return min(c, key=lambda x: abs(x[0] - tt)) if c and min(abs(x[0] - tt) for x in c) < 0.25 else None
    samp = [at(t + dt) for dt in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5)]
    lat = [None if s is None else s[2] for s in samp]
    d0 = at(t)
    # side-zone time: walk back from the last sample OUTSIDE the corridor (|lat| > 1.75) before t, while contiguous in the side zone (gaps < 0.35 s)
    j = bisect.bisect_left(ts, t) - 1
    while j >= 0 and o[j][2] <= 1.75:
        j -= 1
    first = last = None
    while j >= 0 and 1.75 < o[j][2] <= 3.5 and (last is None or last - o[j][0] < 0.35):
        if first is None:
            first = o[j][0]
        last = o[j][0]; j -= 1
    side_s = (first - last) if first is not None and last is not None else 0.0
    ok = [x for x in lat if x is not None]
    rate = (ok[0] - ok[min(len(ok) - 1, 4)]) / 2.0 if len(ok) >= 5 and lat[0] is not None and lat[4] is not None else None
    looks = "yes" if (rate is not None and 0.3 <= rate <= 2.5 and lat[0] is not None and lat[0] >= 2.2) else ("unclear" if rate is None else "no")
    print(f"| {t:.1f} | {tid} | {'' if d0 is None else f'{d0[1]:.1f}'} | {side_s:.1f} | {'' if rate is None else f'{rate:.2f}'} | {' '.join('-' if x is None else f'{x:.1f}' for x in lat)} | {looks} |")

# ---- statistics over ALL promotions of this drive
import statistics as _S
def classify(t, tid):
    o = obs.get(int(tid), []); ts = [x[0] for x in o]
    def at(tt):
        i = bisect.bisect_left(ts, tt); c = [o[j] for j in (i - 1, i) if 0 <= j < len(o)]
        return min(c, key=lambda x: abs(x[0] - tt)) if c and min(abs(x[0] - tt) for x in c) < 0.25 else None
    a0, a4 = at(t - 2.0), at(t)
    rate = None if (a0 is None or a4 is None) else (a0[2] - a4[2]) / 2.0
    return rate, a0
by_t = {round(float(r["t_s"]), 2): r for r in ev}
lc = gf = unk = 0
for t, tid in promo:
    rate, a0 = classify(t, tid)
    r = by_t.get(round(t, 2))
    if rate is not None and a0 is not None and 0.3 <= rate <= 2.5 and a0[2] >= 2.2:
        lc += 1
    if r is not None and r["speed_kmh"] not in ("", None) and r["t_vrel"] not in ("", None):
        if abs(float(r["t_vrel"]) + float(r["speed_kmh"]) / 3.6) <= 1.5:
            gf += 1
    else:
        unk += 1
print(f"over all {len(promo)} promotions: lane-change-like trajectory (|lat| >= 2.2 m two seconds before, closing laterally 0.3-2.5 m/s): {lc} ({100 * lc / len(promo):.0f} %); "
      f"target ground-fixed at promotion (|vrel + v_ego| <= 1.5 m/s: roadside object seen laterally sweeping in, e.g. on a curve [hypothesis]): {gf} ({100 * gf / len(promo):.0f} %)")
