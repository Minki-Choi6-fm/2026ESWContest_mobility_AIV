#!/usr/bin/env python3
"""READ-ONLY offline replay: old TargetSelector (3-of-5 per ts group) vs WindowTargetSelector (time window) on a recorded rec/frames.csv.
Groups 0x12 frames by the STM32 ts exactly like brain.py (flush when ts changes, receive time = last frame of the group) and evaluates current() on a 20 Hz grid.
usage: python3 -I replay_target_select.py LOGDIR [HH:MM:SS HH:MM:SS [ID]]   (no times = whole run)"""
import csv, json, os, sys, time, collections

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from target_select import Obj, TargetSelector
from target_select_win import WindowTargetSelector


def load_groups(logdir, t0=None, t1=None):
    groups, cur = [], None
    with open(os.path.join(logdir, "rec", "frames.csv")) as f:
        for r in csv.DictReader(f):
            if r["type"] != "0x12":
                continue
            t = float(r["time"])
            if t0 is not None and t < t0 - 2:
                continue
            if t1 is not None and t > t1:
                break
            d = json.loads(r["decoded"])
            if cur is None or d["ts"] != cur[0]:
                cur = [d["ts"], t, []]; groups.append(cur)
            cur[1] = t
            cur[2].append(Obj(d["id"], d["long_m"], d["lat_m"], d["vrel_ms"], d.get("rcs_dbsm"), d.get("dyn")))
    return groups


def run(groups, sel, t0, t1, step=0.05):
    out, gi, t = [], 0, t0
    while t <= t1:
        while gi < len(groups) and groups[gi][1] <= t:
            sel.update(groups[gi][2], groups[gi][1]); gi += 1
        r = sel.current(t)
        out.append((t, r.target.id if r.target else None, r.held))
        t += step
    return out


def stats(rows, want=None):
    n = len(rows); tg = [x for x in rows if x[1] is not None]
    eps, cur, last = [], None, None
    for t, i, h in rows:
        if i is None:
            if cur: eps.append(cur); cur = None
        elif cur is None or i != cur[0]:
            if cur: eps.append(cur)
            cur = [i, t, t]
        else:
            cur[2] = t
    if cur: eps.append(cur)
    short = sum(1 for e in eps if e[2] - e[1] + 0.05 < 0.5)
    d = dict(n=n, sel=len(tg) / n if n else 0, ids=len({x[1] for x in tg}), episodes=len(eps), short=short, held=sum(1 for x in tg if x[2]) / n if n else 0)
    if want is not None:
        d["want"] = sum(1 for x in rows if x[1] == want) / n if n else 0
    return d


def main():
    logdir = sys.argv[1]
    if len(sys.argv) >= 4:
        day = time.strftime("%Y-%m-%d", time.localtime(float(open(os.path.join(logdir, "rec", "frames.csv")).readlines()[1].split(",")[0])))
        ep = lambda h: time.mktime(time.strptime(day + " " + h, "%Y-%m-%d %H:%M:%S"))
        t0, t1 = ep(sys.argv[2]), ep(sys.argv[3]); want = int(sys.argv[4]) if len(sys.argv) > 4 else None
    else:
        want = None; t0 = t1 = None
    g = load_groups(logdir, t0, t1)
    if t0 is None:
        t0, t1 = g[0][1], g[-1][1]
    variants = [("old 3-of-5/ts", lambda: TargetSelector())]
    for spec in os.environ.get("WIN", "0.4:3").split(","):
        w, h = spec.split(":"); variants.append((f"new {w}s>={h}", lambda w=float(w), h=int(h): WindowTargetSelector(w, h)))
    for name, mk in variants:
        sel = mk()
        # warm-up 2 s before the window so histories exist
        s0 = [x for x in g if x[1] < t0]; 
        for x in s0: sel.update(x[2], x[1])
        rows = run([x for x in g if x[1] >= t0], sel, t0, t1)
        s = stats(rows, want)
        print(f"{name:14s} n={s['n']:6d} 선택률 {100*s['sel']:5.1f}% (held {100*s['held']:4.1f}%) 목표id {100*s.get('want',0):5.1f}% ids {s['ids']:4d} 구간 {s['episodes']:4d} 0.5s미만 {s['short']:4d}")


if __name__ == "__main__":
    main()
