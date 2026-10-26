#!/usr/bin/env python3
"""READ-ONLY Pi5 power sampler (S19): every 0.2 s one CSV row with epoch ms, EXT5V_V, ALL rails of `vcgencmd pmic_read_adc` (V and A), get_throttled, measure_temp.
Run it with `nice -n 19`. Calls that take long are flagged in `call_ms` (duration of the pmic call) and `late_ms` (how late the row is versus its 0.2 s slot).
Changes nothing. usage: nice -n 19 python3 power_sampler.py OUT.csv SECONDS"""
import re, subprocess, sys, time

def vc(*a):
    t = time.perf_counter()
    try: o = subprocess.run(["vcgencmd", *a], capture_output=True, text=True, timeout=2).stdout
    except Exception: o = ""
    return o, (time.perf_counter() - t) * 1000

o, _ = vc("pmic_read_adc")
rails = []
for l in o.splitlines():
    m = re.match(r"\s*(\S+)\s+(volt|current)\(\d+\)=([\d.]+)[VA]", l)
    if m and (m.group(1), m.group(2)) not in rails: rails.append((m.group(1), m.group(2)))
cols = ["epoch_ms", "wall", "call_ms", "late_ms", "throttled", "temp_c"] + [n for n, k in rails]
out = open(sys.argv[1], "w", buffering=1); out.write(",".join(cols) + "\n")
end = time.time() + float(sys.argv[2]); slot = time.monotonic()
while time.time() < end:
    slot += 0.2
    d = slot - time.monotonic()
    if d > 0: time.sleep(d)
    late = (time.monotonic() - slot) * 1000
    o, ms = vc("pmic_read_adc"); th, _ = vc("get_throttled"); tp, _ = vc("measure_temp")
    vals = {}
    for l in o.splitlines():
        m = re.match(r"\s*(\S+)\s+(volt|current)\(\d+\)=([\d.]+)[VA]", l)
        if m: vals[(m.group(1), m.group(2))] = m.group(3)
    t = re.search(r"([\d.]+)", tp)
    now = time.time()
    out.write(",".join([str(int(now * 1000)), time.strftime("%H:%M:%S", time.localtime(now)), f"{ms:.0f}", f"{late:.0f}", th.strip().split("=")[-1], t.group(1) if t else ""] + [vals.get(r, "") for r in rails]) + "\n")
