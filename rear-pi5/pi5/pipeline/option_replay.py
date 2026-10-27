#!/usr/bin/env python3
"""READ-ONLY: one offline replay of a recorded drive (rec/frames.csv) through brain.Brain -> danger.step (s4_replay.replay), using the environment as it is
(TARGET_SEL, LAT2_MAX_M, PASS_D_M, PASS_LAT_M). Writes one CSV row per evaluation, with times in the same t_rel base as drive_log.csv.
usage: python3 -I option_replay.py LOGDIR OUT.csv"""
import csv, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import s4_replay as S

log, out = sys.argv[1], sys.argv[2]
d = [r for r in csv.DictReader(l for l in open(os.path.join(log, "drive_log.csv")) if not l.startswith("#"))]
w0 = float(d[0]["wall"]) - float(d[0]["t_rel"])
cap = {}
_orig = S.load_rows
S.load_rows = lambda drive: (lambda r: (cap.__setitem__("t0", r[0][0]), r)[1])(_orig(drive))
_, ev, _ = S.replay(os.path.join(log, "rec"), write_csv=False)
with open(out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_rel", "level", "pre", "tgt_id", "d", "lat", "vrel", "speed", "supp", "reason"])
    for e in ev:
        tg = e["tgt"]
        w.writerow([f"{cap['t0'] + e['t'] - w0:.2f}", e["level"], e["pre"], "" if tg is None else tg.id, "" if tg is None else tg.long_m, "" if tg is None else tg.lat_m,
                    "" if tg is None else tg.vrel_ms, "" if e["speed"] is None else e["speed"], ";".join(e["supp"]), e["reason"]])
