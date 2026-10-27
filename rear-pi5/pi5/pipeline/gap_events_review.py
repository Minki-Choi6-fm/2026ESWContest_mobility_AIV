#!/usr/bin/env python3
"""S10-a: list every RADSTAT one-second window whose largest CAN gap exceeds 1000 ms (= Safe Mode entry with a 1000 ms radar timeout) with context. READ-ONLY on recorded logs.
usage: python3 -I gap_events_review.py <drive_dir>... [--md]"""
import bisect, csv, json, math, os, re, statistics as S, sys

THRESH = 1000


def rows(d):
    meta = json.load(open(os.path.join(d, "meta.json"))); t0 = meta["start_time"]; R = []
    for ln in open(os.path.join(d, "stm_debug.log"), errors="replace"):
        if " RADSTAT " in ln:
            p = ln.split(" ", 1); m = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", p[1])}
            R.append(dict(abs=float(p[0]), host=float(p[0]) - t0, **m))
    out = []
    for i, cur in enumerate(R):
        if i == 0: continue
        prev = R[i - 1]
        if cur["t"] < prev["t"] - 1000 or not (800 <= cur["t"] - prev["t"] <= 1200): continue
        cur = dict(cur); cur["inc"] = cur["safe"] - prev["safe"]; out.append(cur)
    return t0, out


def series(d):
    imu = []; obd = []; gps = []
    for r in csv.DictReader(open(os.path.join(d, "frames.csv"))):
        if r["type"] == "0x13":
            try:
                j = json.loads(r["decoded"]); a = j["acc_mg"]
                imu.append((float(r["time"]), math.sqrt(sum(x * x for x in a))))
            except Exception: pass
    for ln in open(os.path.join(d, "stm_debug.log"), errors="replace"):
        m = re.match(r"(\d+\.\d+) OBD speed=(\d+)km/h", ln)
        if m: obd.append((float(m.group(1)), int(m.group(2))))
    gp = os.path.join(d, "gps.jsonl")
    for ln in open(gp, errors="replace"):
        try: j = json.loads(ln)
        except Exception: continue
        m = j.get("msg", {})
        if m.get("class") == "TPV" and "speed" in m: gps.append((j["t"], m["speed"] * 3.6))
    return imu, obd, gps


def near(ser, t, maxd):
    if not ser: return None
    ks = [x[0] for x in ser]; i = bisect.bisect_left(ks, t)
    c = [ser[j] for j in (i - 1, i) if 0 <= j < len(ser)]
    b = min(c, key=lambda x: abs(x[0] - t))
    return b[1] if abs(b[0] - t) <= maxd else None


def imu_max(imu, t0, t1):
    v = [x[1] for x in imu if t0 <= x[0] <= t1]
    return max(v) if v else None


def klass(r):
    if r["a"] == 0 and r["b"] == 0: return "A"
    if r["b"] > 0: return "C"
    return "B"


def run(d):
    name = os.path.basename(d.rstrip("/")); t0, R = rows(d); imu, obd, gps = series(d)
    ev = [i for i, r in enumerate(R) if r["gap"] > THRESH]
    allg = [x[1] for x in imu]; p99 = sorted(allg)[int(.99 * (len(allg) - 1))] if allg else 0
    print(f"\n## {name}: {len(R)} clean windows, gap>{THRESH} ms windows: {len(ev)}; IMU |acc| (mg) over the whole log: median {S.median(allg):.0f}, p99 {p99:.0f}, max {max(allg):.0f}")
    out = []
    for n, i in enumerate(ev, 1):
        r = R[i]; w = lambda k: f"{R[k]['a']}/{R[k]['b']}/{R[k]['exp']}" if 0 <= k < len(R) else "-"
        ctx = " ".join(w(k) for k in range(i - 3, i + 4))
        o = near(obd, r["abs"], 5); g = near(gps, r["abs"], 5)
        im = imu_max(imu, r["abs"] - 1.0, r["abs"])
        out.append(dict(n=n, log=name, t=r["t"], host=r["host"], gap=r["gap"], a=r["a"], b=r["b"], exp=r["exp"], cls=klass(r), ctx=ctx, obd=o, gps=g, imu=im, ovf=r["ovf"], tec=r["tec"], rec=r["rec"], inc=r["inc"]))
        print(f"| {n} | {name[-6:]} | {r['t']} | {r['host']:.0f} | {r['gap']} | {r['a']}/{r['b']}/{r['exp']} | {ctx} | {o if o is not None else '-'} | {'-' if g is None else f'{g:.0f}'} | {'-' if im is None else f'{im:.0f}'} | {r['ovf']}/{r['tec']}/{r['rec']} | {r['inc']} | {klass(r)} |")
    return out


if __name__ == "__main__":
    ds = [a for a in sys.argv[1:] if not a.startswith("--")]; tot = []
    print("| # | log | STM32 t ms | s from start | gap ms | a/b/exp (that second) | a/b/exp for -3..+3 s | OBD km/h | GPS km/h | IMU max|acc| mg (that second) | ovf/tec/rec | safe+ | class |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for d in ds: tot += run(d)
    from collections import Counter
    c = Counter(x["cls"] for x in tot)
    print(f"\nTOTAL events {len(tot)}; A (a=0,b=0) {c['A']}, B (a>0,b=0) {c['B']}, C (b>0) {c['C']}; sum {sum(c.values())}")
    print("safe+ sum over these windows:", sum(x["inc"] for x in tot))
    hs = sorted(x["host"] for x in tot if x["log"].endswith("141541"))
    print("141541 event times (s):", [round(h) for h in hs]); print("diffs (s):", [round(b - a) for a, b in zip(hs, hs[1:])])
    json.dump(tot, open(os.path.join(os.environ.get("OUT", "/tmp"), "gap_events.json"), "w"))
