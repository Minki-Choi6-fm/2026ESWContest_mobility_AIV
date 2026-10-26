#!/usr/bin/env python3
"""READ-ONLY: recompute the danger level of every drive_log.csv row from the logged inputs (target d/lat/vrel, own speed, camera_confirmed, K from the reason text)
through danger.step with fresh hysteresis state, optionally with the LAT2_MAX_M / PASS_* options. Used to check that, with the options off, the recomputation
matches the RECORDED levels, and to show what the options change on the same rows. Inputs not in drive_log (radar dyn flag, V2V peer, exact unrounded numbers) are approximated.
usage: python3 -I option_recompute.py LOGDIR [LAT2_MAX_M PASS_D_M PASS_LAT_M]  -> prints recorded vs recomputed level counts"""
import csv, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import constants as C
import danger as D


class T:
    def __init__(self, long_m, lat_m, vrel_ms, dyn): self.long_m, self.lat_m, self.vrel_ms, self.dyn = long_m, lat_m, vrel_ms, dyn


def load(log):
    return [r for r in csv.DictReader(l for l in open(os.path.join(log, "drive_log.csv")) if not l.startswith("#"))]


def recompute(rows, lat2=None, pass_d=None, pass_lat=None):
    D.LAT2_MAX_M, D.PASS_D_M, D.PASS_LAT_M = lat2, pass_d, pass_lat
    st, out = D.DangerState(), []
    for r in rows:
        now = float(r["wall"])
        tgt = None
        if r["tgt_id"] != "":
            tgt = T(float(r["tgt_d_m"]), float(r["tgt_lat_m"]), float(r["tgt_vrel_ms"]), 1 if "정지 물체(dyn)" in r["reason"] else None)
        v_r = None if r["speed_kmh"] == "" else float(r["speed_kmh"]) / 3.6
        m = re.search(r"a_req=\S+?/([\d.]+)", r["reason"])
        k_r = float(m.group(1)) / C.G_MPS2 if m else 0.5
        cam = r["camera_confirmed"] == "1"
        lv, why, met = D.step(st, now, r["radar_down"] != "1", "0x10 무효" not in r["reason"], r["safe_mode"] == "1", tgt, v_r, k_r, C.K_FRONT_DEFAULT,
                              lifted=cam, lift_headway=cam, lift_why="camera_confirmed" if cam else "")
        out.append((float(r["t_rel"]), lv, tgt, why))
    return out


def episodes(levels, gap=1.0, want=2):
    eps, cur = [], None
    for t, lv in levels:
        if lv != want: continue
        if cur and t - cur[1] <= gap: cur[1] = t; cur[2] += 1
        else:
            if cur: eps.append(cur)
            cur = [t, t, 1]
    if cur: eps.append(cur)
    return eps


if __name__ == "__main__":
    rows = load(sys.argv[1])
    a = [float(x) if x else None for x in sys.argv[2:5]] + [None] * 3
    rec = [(float(r["t_rel"]), 0xFF if r["level"] == "255" else int(r["level"])) for r in rows]
    off = recompute(rows); on = recompute(rows, *a[:3])
    c = lambda xs: {k: sum(1 for x in xs if x == k) for k in (0, 1, 2, 255)}
    print("recorded  ", c([l for _, l in rec]), len(episodes(rec)), "episodes")
    print("recomputed", c([x[1] for x in off]), len(episodes([(x[0], x[1]) for x in off])), "episodes")
    print("rows differing recorded vs recomputed(off):", sum(1 for r, x in zip(rec, off) if r[1] != x[1]))
    print("with options", c([x[1] for x in on]), len(episodes([(x[0], x[1]) for x in on])), "episodes")
