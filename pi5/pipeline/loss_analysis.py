#!/usr/bin/env python3
"""S4 part 7: where do the missing 0x10 frames of recorded drives come from? Offline, read-only.

A missing 0x10 = STM32 clock jump of ~k*20 ms between two consecutive 0x10 frames (see live_receiver.LossTracker).
frames.csv is in ARRIVAL order, so what lies between the two neighbouring 0x10 frames in the file is exactly what arrived in that gap.
Correlations (each against a baseline of ordinary consecutive 0x10 pairs):
  * a 0x14 (OBD) frame arrived inside the gap          * number of 0x12 radar frames inside the gap (bursts)
  * the gap happened within +-0.2 s of an OLEDSTAT line of stm_debug.log (5 s period)
  * gap crosses a read-chunk boundary of the logger (kernel/Pi overrun would drop at boundaries; bit errors drop inside a chunk)
  * Pi5 temperature bin / throttled from sys.csv
Host times are chunk-coarse (~0.1 s); anything seen here is a [hypothesis] to test with the new BAD/loss counters in the car.
"""
import bisect, collections, csv, json, os, re, statistics as S, struct, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def analyse(d):
    rows = list(csv.reader(open(os.path.join(d, "frames.csv"))))[1:]
    pairs = []                      # consecutive 0x10 pairs: dict(miss, between types, same_chunk, host_t)
    last = None                     # (index, ts, host_t)
    types_since = collections.Counter()
    for i, r in enumerate(rows):
        typ, t = int(r[1], 16), float(r[0])
        if typ == 0x10:
            ts = struct.unpack_from("<I", bytes.fromhex(r[2]))[0]
            if last is not None and ts > last[1]:
                miss = max(0, round((ts - last[1]) / 20) - 1)
                pairs.append(dict(miss=miss, n14=types_since[0x14], n12=types_since[0x12], n13=types_since[0x13], n11=types_since[0x11],
                                  same_chunk=(t == last[2]), host_t=t))
            if last is None or ts > last[1] or ts < last[1]:
                last = (i, ts, t)
            types_since = collections.Counter()
        else:
            types_since[typ] += 1
    gaps = [p for p in pairs if p["miss"] > 0]
    base = [p for p in pairs if p["miss"] == 0]
    out = dict(name=os.path.basename(d.rstrip("/")), n_pairs=len(pairs), n_gap_events=len(gaps), missing=sum(p["miss"] for p in gaps),
               dist=collections.Counter(p["miss"] for p in gaps))
    frac = lambda lst, f: 100.0 * sum(1 for p in lst if f(p)) / max(1, len(lst))
    out["has14"] = (frac(gaps, lambda p: p["n14"] > 0), frac(base, lambda p: p["n14"] > 0))
    # time-normalised: a gap pair spans (1+miss)*20 ms, so compare radar frames PER 20 ms slot, not per pair
    out["n12"] = (sum(p["n12"] for p in gaps) / max(1, sum(1 + p["miss"] for p in gaps)), sum(p["n12"] for p in base) / max(1, len(base)))
    out["has14_exp"] = sum(min(1.0, (1 + p["miss"]) * 0.06) for p in gaps) * 100.0 / max(1, len(gaps))
    out["n12_gap_ge4"] = (frac(gaps, lambda p: p["n12"] >= 4), frac(base, lambda p: p["n12"] >= 4))
    out["cross_chunk"] = (frac(gaps, lambda p: not p["same_chunk"]), frac(base, lambda p: not p["same_chunk"]))
    # OLEDSTAT proximity
    ol = []
    lp = os.path.join(d, "stm_debug.log")
    if os.path.exists(lp):
        for line in open(lp, errors="replace"):
            if "OLEDSTAT" in line:
                try:
                    ol.append(float(line.split()[0]))
                except ValueError:
                    pass
    near = lambda t: bool(ol) and min(abs(t - ol[j]) for j in {max(0, bisect.bisect_left(ol, t) - 1), min(len(ol) - 1, bisect.bisect_left(ol, t))}) <= 0.2
    out["oledstat"] = (frac(gaps, lambda p: near(p["host_t"])), frac(base, lambda p: near(p["host_t"])), len(ol))
    # temperature / throttled
    sp = os.path.join(d, "sys.csv")
    if os.path.exists(sp):
        sysr = [(float(r[0]), r[1], float(r[2])) for r in list(csv.reader(open(sp)))[1:]]
        st = [x[0] for x in sysr]
        def temp_of(t):
            i = min(max(bisect.bisect_left(st, t), 0), len(st) - 1); return sysr[i][2]
        bins = collections.OrderedDict((b, [0, 0]) for b in ("<50", "50-53", "53-56", ">=56"))
        def tb(x): return "<50" if x < 50 else "50-53" if x < 53 else "53-56" if x < 56 else ">=56"
        for p in pairs:
            b = bins[tb(temp_of(p["host_t"]))]; b[0] += 1 + p["miss"]; b[1] += p["miss"]
        out["temp"] = {k: (v[0], v[1], 100.0 * v[1] / v[0] if v[0] else float("nan")) for k, v in bins.items()}
        out["throttled"] = sorted({x[1] for x in sysr})
    return out


def show(o):
    print(f"\n## {o['name']}: {o['n_pairs']} consecutive 0x10 pairs; {o['n_gap_events']} gap events, {o['missing']} missing frames "
          f"({100 * o['missing'] / (o['n_pairs'] + o['missing']):.2f}% of expected)")
    print("   gap length (frames missing in a row) -> events: " + ", ".join(f"{k}:{v}" for k, v in sorted(o["dist"].items())))
    print("| check | gap events | ordinary pairs (baseline) | reading |\n|---|---|---|---|")
    a, b = o["has14"]; print(f"| a 0x14 OBD frame arrived inside the gap (baseline x span: {o['has14_exp']:.1f}%) | {a:.1f}% | {b:.1f}% | {'correlated' if a > 1.5 * o['has14_exp'] else 'no correlation'} |")
    a, b = o["n12"]; print(f"| radar 0x12 frames per 20 ms slot inside the gap (time-normalised) | {a:.2f} | {b:.2f} | {'radar-heavy slots coincide with losses [hypothesis]' if a > 1.3 * b else 'no correlation'} |")
    a, b = o["n12_gap_ge4"]; print(f"| >= 4 radar frames inside the gap | {a:.1f}% | {b:.1f}% | {'more bursts' if a > 2 * b + 5 else 'no correlation'} |")
    a, b = o["cross_chunk"]; print(f"| gap spans two logger read chunks | {a:.1f}% | {b:.1f}% | {'compare with 1-(1-baseline)^(1+miss) ~ 64% for a 40 ms span: ' + ('higher -> boundary-linked [hypothesis]' if a > 75 else 'consistent with a random position, not boundary-linked')} |")
    a, b, n = o["oledstat"]; print(f"| within +-0.2 s of an OLEDSTAT line ({n} lines, 5 s period) | {a:.1f}% | {b:.1f}% | {'no OLEDSTAT lines in this log (firmware before v18c): cannot test' if n == 0 else ('correlated [hypothesis]' if a > 2 * b + 5 else 'no correlation')} |")
    if "temp" in o:
        print("   Pi5 temperature bin -> frames expected / missing / loss%: " + "; ".join(f"{k}: {v[0]}/{v[1]}/{v[2]:.2f}%" for k, v in o["temp"].items() if v[0]) + f"   throttled values: {o['throttled']}")


def txdrop_check(d):
    """If stm_debug.log has RADSTAT lines with txdrop= (firmware v18d and later), compare its per-second increase with the per-second losses seen by the Pi5.
    Returns None (and prints a note) when there is no txdrop field."""
    import re
    lp = os.path.join(d, "stm_debug.log")
    rs = []
    if os.path.exists(lp):
        for line in open(lp, errors="replace"):
            if " RADSTAT " in line and "txdrop=" in line:
                m = dict(re.findall(r"(\w+)=(-?\d+)", line))
                rs.append((int(m["t"]), int(m["txdrop"]), int(m["pub"])))
    name = os.path.basename(d.rstrip("/"))
    if not rs:
        print(f"\n## {name}: txdrop check skipped (no RADSTAT txdrop= in stm_debug.log; firmware before v18d)")
        return None
    from tx_queue_analysis import load, epochs
    fr = load(d)
    eps, rs_ep, cur = epochs(fr), [], []
    for r in rs:
        if cur and r[0] < cur[-1][0] - 1000:
            rs_ep.append(cur); cur = []
        cur.append(r)
    rs_ep.append(cur)
    secs = []                                                   # (t_end, d_txdrop, missing_0x10, missing_0x12)
    for ep, rse in zip(eps, rs_ep):
        t10 = sorted(ts for ts, typ, _ in ep if typ == 0x10)
        t12 = sorted(ts for ts, typ, _ in ep if typ == 0x12)
        for (t0, tx0, _), (t1, tx1, pub) in zip(rse, rse[1:]):
            if not (900 <= t1 - t0 <= 1100) or tx1 < tx0:
                continue
            w = [x for x in t10 if t0 < x <= t1]
            miss10 = sum(max(0, round((b - a_) / 20) - 1) for a_, b in zip(w, w[1:]))
            got12 = bisect.bisect_right(t12, t1) - bisect.bisect_right(t12, t0)
            secs.append((t1, tx1 - tx0, miss10, max(0, pub - got12)))
    n = len(secs)
    tx = [x[1] for x in secs]; m10 = [x[2] for x in secs]; m12 = [x[3] for x in secs]
    def corr(a, b):
        ma, mb = sum(a) / n, sum(b) / n
        cov = sum((x - ma) * (y - mb) for x, y in zip(a, b)); va = sum((x - ma) ** 2 for x in a); vb = sum((y - mb) ** 2 for y in b)
        return cov / (va * vb) ** 0.5 if va * vb > 0 else float("nan")
    with_tx = [x for x in secs if x[1] > 0]
    print(f"\n## {name}: txdrop check over {n} one-second windows: tx_drop_count rose by {sum(tx)} in total; Pi5-side missing 0x10 = {sum(m10)}, 0x12 (pub - received) = {sum(m12)}")
    print(f"   seconds with txdrop increase: {len(with_tx)} ({100 * len(with_tx) / max(1, n):.0f} %); of these with missing 0x10 or 0x12: {sum(1 for x in with_tx if x[2] or x[3])}; "
          f"seconds with missing frames but NO txdrop increase: {sum(1 for x in secs if (x[2] or x[3]) and not x[1])}")
    print(f"   correlation per second: txdrop vs missing 0x10 {corr(tx, m10):+.2f}, txdrop vs missing 0x12 {corr(tx, m12):+.2f}  "
          f"[txdrop counts every dropped 0x1x frame; missing > txdrop means other loss (bit errors / logger), txdrop > missing means frames dropped that the Pi5 could not see]")
    worst = sorted(secs, key=lambda x: -(x[1] + x[2] + x[3]))[:5]
    print("   worst seconds (STM32 t ms, d_txdrop, missing 0x10, missing 0x12): " + "; ".join(str(w) for w in worst))
    return secs


def _selftest_txdrop():
    """Plumbing test with a SYNTHETIC txdrop column (built from the measured losses of a real log), not data: txdrop must be skipped when absent and correlated when present."""
    import re, shutil, tempfile
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs", "vehicle_test_logs", "drive_20261016_152211")
    tmp = tempfile.mkdtemp()
    shutil.copy(os.path.join(src, "frames.csv"), tmp)
    shutil.copy(os.path.join(src, "stm_debug.log"), tmp)
    assert txdrop_check(tmp) is None                                     # no txdrop field -> skipped
    lines = open(os.path.join(src, "stm_debug.log"), errors="replace").read().splitlines()
    from tx_queue_analysis import load, epochs
    fr = load(tmp)
    ts12 = sorted(ts for ts, typ, _ in fr if typ == 0x12)
    out, last_t, cum = [], None, 0
    for line in lines:
        if " RADSTAT " in line:
            m = dict(re.findall(r"(\w+)=(-?\d+)", line)); t = int(m["t"]); pub = int(m["pub"])
            if last_t is not None and 900 <= t - last_t <= 1100:
                got = bisect.bisect_right(ts12, t) - bisect.bisect_right(ts12, last_t)
                cum += max(0, pub - got)
            last_t = t
            line = line.replace(" drop=", f" txdrop={cum} drop=")
        out.append(line)
    open(os.path.join(tmp, "stm_debug.log"), "w").write("\n".join(out) + "\n")
    secs = txdrop_check(tmp)
    assert secs and sum(x[1] for x in secs) > 100
    import math
    n = len(secs); a = [x[1] for x in secs]; b = [x[3] for x in secs]
    ma, mb = sum(a) / n, sum(b) / n
    r = sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))
    assert r > 0.95, r
    print("txdrop plumbing selftest OK (synthetic column; skipped when absent)")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest_txdrop()
    else:
        for d in [x for x in sys.argv[1:] if not x.startswith("--")]:
            show(analyse(d))
            if "--txdrop" in sys.argv:
                txdrop_check(d)
