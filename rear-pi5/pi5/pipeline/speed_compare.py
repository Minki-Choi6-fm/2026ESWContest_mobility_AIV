#!/usr/bin/env python3
"""Offline: OBD (0x14 in frames.csv) vs GPS (gps.jsonl TPV) + get_speed() replay. Reads logs only."""
import bisect, csv, json, os, re, statistics as S, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import ExtendedDecoder, SensorState, build_frame
from speed_select import get_speed
from constants import SOURCE_OBD, SOURCE_GPS, SOURCE_NONE


def load_gps(d):
    out = []
    for line in open(d + "/gps.jsonl"):
        try: r = json.loads(line)
        except ValueError: continue
        m = r.get("msg", {})
        if m.get("class") == "TPV" and m.get("mode", 0) >= 2 and isinstance(m.get("speed"), (int, float)):
            out.append((r["t"], m["speed"] * 3.6))
    return out


class ReplayGps:
    def __init__(self, fixes): self.f, self.t, self.now = fixes, [x[0] for x in fixes], 0
    def latest(self):
        i = bisect.bisect_right(self.t, self.now) - 1
        return None if i < 0 else (self.f[i][1], self.f[i][0])


def interp(fixes, ts, t):
    i = bisect.bisect_left(ts, t)
    if i == 0 or i >= len(ts): return None
    (t0, v0), (t1, v1) = fixes[i-1], fixes[i]
    if t1 - t0 > 2.5: return None                      # don't interpolate across GPS dropouts
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def run(d):
    gps = load_gps(d); gt = [x[0] for x in gps]
    dec, st, rg = ExtendedDecoder(), SensorState(), ReplayGps(gps)
    obd, choice, ages = [], {SOURCE_OBD: 0, SOURCE_GPS: 0, SOURCE_NONE: 0}, []
    why = {}
    nxt = None
    with open(d + "/frames.csv", newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            t = float(row[0])
            for fr in dec.feed(build_frame(int(row[1], 16), bytes.fromhex(row[2])), t):
                st.update(fr)
                if fr.typ == 0x14 and fr.data["speed_kmh"] is not None:
                    obd.append((t, fr.data["speed_kmh"])); 
                    if fr.data["age_ms"] is not None: ages.append(fr.data["age_ms"])
            nxt = t if nxt is None else nxt
            while t >= nxt:                                   # evaluate get_speed at 10 Hz
                rg.now = nxt; res = get_speed(st, rg, nxt)
                choice[res.source] += 1
                k = "obd ok" if res.source == SOURCE_OBD else re.sub(r"\d+", "N", res.reason.split(";")[0])
                why[k] = why.get(k, 0) + 1
                nxt += 0.1
    return gps, gt, obd, ages, choice, why


def stats(obd, gps, gt, lag):
    e = []
    for t, v in obd:
        g = interp(gps, gt, t + lag)                          # GPS value at (t+lag)
        if g is not None: e.append((v, g, v - g))
    return e


for d in sys.argv[1:]:
    gps, gt, obd, ages, choice, why = run(d)
    name = os.path.basename(d.rstrip("/"))
    print(f"\n## {name}: OBD samples={len(obd)} GPS fixes(mode>=2)={len(gps)}  OBD AGE_MS: mean={S.mean(ages):.0f} max={max(ages)} >500: {sum(a>500 for a in ages)}")
    best = None
    for lagms in range(-3000, 3001, 100):
        e = stats(obd, gps, gt, lagms / 1000)
        if len(e) > 50:
            mae = S.mean(abs(x[2]) for x in e)
            if best is None or mae < best[1]: best = (lagms, mae)
    print("| subset | n | mean err (OBD-GPS) km/h | MAE | RMSE | max abs |")
    print("|---|---|---|---|---|---|")
    for label, lag in (("no shift", 0.0), (f"best lag {best[0]} ms", best[0] / 1000)):
        for sub, flt in (("all", lambda x: True), ("OBD>=5 km/h", lambda x: x[0] >= 5)):
            e = [x for x in stats(obd, gps, gt, lag) if flt(x)]
            if e:
                er = [x[2] for x in e]
                print(f"| {label}, {sub} | {len(e)} | {S.mean(er):+.2f} | {S.mean(map(abs, er)):.2f} | "
                      f"{(S.mean(x*x for x in er))**.5:.2f} | {max(map(abs, er)):.1f} |")
    print(f"best lag (GPS time shift minimising MAE): {best[0]} ms  (+ = GPS reports the same speed that much LATER than OBD, i.e. GPS lags OBD)")
    tot = sum(choice.values())
    print("get_speed() replay @10Hz:", {k: f"{v} ({100*v/tot:.1f}%)" for k, v in {"OBD": choice[0], "GPS": choice[1], "NONE": choice[255]}.items()}, "| reasons:", why)
