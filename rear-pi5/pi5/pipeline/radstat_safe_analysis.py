#!/usr/bin/env python3
"""S9-a: what makes the STM32 enter Safe Mode (radar timeout 100 ms)? Parses the per-second RADSTAT lines of recorded stm_debug.log files. READ-ONLY.
RADSTAT fields per second: a = 0x60A status frames, b = 0x60B object frames, exp = objects announced by the radar, gap = largest gap between two received CAN frames
(any ID), safe = cumulative Safe Mode entries. usage: python3 -I radstat_safe_analysis.py <drive_dir>..."""
import json, os, re, statistics as S, sys


def load(d):
    meta = json.load(open(os.path.join(d, "meta.json")))
    t0 = meta["start_time"]; rows = []
    for ln in open(os.path.join(d, "stm_debug.log"), errors="replace"):
        if " RADSTAT " in ln:
            p = ln.split(" ", 1); m = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", p[1])}
            rows.append(dict(host=float(p[0]) - t0, **m))
    out = []
    for prev, cur in zip(rows, rows[1:]):
        if cur["t"] < prev["t"] - 1000 or not (800 <= cur["t"] - prev["t"] <= 1200):
            continue                                              # STM32 restart / log gap: the window is not a clean second
        cur = dict(cur); cur["inc"] = cur["safe"] - prev["safe"]
        out.append(cur)
    return out


def stat(v, f):
    return "-" if not v else f"{f(v):.1f}"


def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float("nan")


def run(d):
    name = os.path.basename(d.rstrip("/")); R = load(d); n = len(R)
    up = [r for r in R if r["inc"] > 0]; no = [r for r in R if r["inc"] == 0]
    print(f"\n## {name}: {n} clean one-second RADSTAT windows; Safe Mode entries (sum of safe= increases): {sum(r['inc'] for r in R)}")
    print("1) per-second means / medians (a, b, exp, gap) by whether `safe` increased in that second")
    print("| group | seconds | a mean/med | b mean/med | exp mean/med | gap(max in the second) mean/med ms |\n|---|---|---|---|---|---|")
    for lab, g in (("safe increased", up), ("safe did not increase", no)):
        f = lambda k: f"{stat([r[k] for r in g], S.mean)} / {stat([r[k] for r in g], S.median)}"
        print(f"| {lab} | {len(g)} | {f('a')} | {f('b')} | {f('exp')} | {f('gap')} |")
    b0 = [r for r in R if r["b"] == 0]; bp = [r for r in R if r["b"] > 0]
    print(f"2) seconds with b>0: {len(bp)}, of which safe increased in {sum(1 for r in bp if r['inc'] > 0)} ({100 * sum(1 for r in bp if r['inc'] > 0) / max(1, len(bp)):.1f} %), entries {sum(r['inc'] for r in bp)}; "
          f"seconds with b=0: {len(b0)}, of which safe increased in {sum(1 for r in b0 if r['inc'] > 0)} ({100 * sum(1 for r in b0 if r['inc'] > 0) / max(1, len(b0)):.1f} %), entries {sum(r['inc'] for r in b0)}"
          + (f" ({sum(r['inc'] for r in b0) / len(b0):.2f} entries per b=0 second)" if b0 else ""))
    tot = max(1, sum(r["inc"] for r in R))
    print(f"   share of all Safe Mode entries that fall into b=0 seconds: {100 * sum(r['inc'] for r in b0) / tot:.0f} %")
    sus = [r for r in R if r["a"] >= 10 and r["gap"] > 100]
    print(f"3) seconds with a>=10 but gap>100 ms (status frames flowing normally yet a long hole = power/CAN-problem candidates): {len(sus)}")
    for r in sus[:12]:
        print(f"   t={r['host']:.0f}s  a={r['a']} b={r['b']} exp={r['exp']} gap={r['gap']} ms  safe+{r['inc']}")
    if len(sus) > 12:
        print(f"   ... {len(sus) - 12} more")
    gaps = [r["gap"] for r in R]
    print(f"5) per-second max gap [ms]: p50 {pct(gaps, .5):.0f}  p90 {pct(gaps, .9):.0f}  p99 {pct(gaps, .99):.0f}  max {max(gaps)};  seconds with gap>100: {sum(g > 100 for g in gaps)}, gap>1000: {sum(g > 1000 for g in gaps)}")
    print(f"   Safe Mode entries with the 100 ms timeout (measured): {sum(r['inc'] for r in R)};  with a 1000 ms timeout: {sum(g > 1000 for g in gaps)} (one entry per gap > 1000 ms; fact check from the per-second max gap)")
    longg = sorted(((r["gap"], r["host"], r["a"], r["b"]) for r in R if r["gap"] > 1000), reverse=True)[:6]
    if longg:
        print("   longest gaps (gap ms, at t s, a, b): " + "; ".join(f"{g} ms @ {t:.0f}s a={a} b={b}" for g, t, a, b in longg))
    sil = [r for r in R if r["a"] == 0 and r["b"] == 0]
    print(f"   seconds with NO CAN frame at all (a=0 and b=0): {len(sil)}")
    return R


if __name__ == "__main__":
    allr = {}
    for d in sys.argv[1:]:
        allr[d] = run(d)
