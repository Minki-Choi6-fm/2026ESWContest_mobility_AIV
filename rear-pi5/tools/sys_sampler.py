#!/usr/bin/env python3
"""READ-ONLY Pi5 sampler: every 1 s writes wall time, temp, get_throttled, EXT5V_V (vcgencmd pmic_read_adc), CPU% per core. Changes nothing.
usage: python3 sys_sampler.py OUT.csv SECONDS"""
import re, subprocess, sys, time
out = open(sys.argv[1], "w", buffering=1); out.write("wall,temp_c,throttled,ext5v_v,cpu0,cpu1,cpu2,cpu3\n")
def vc(*a):
    try: return subprocess.run(["vcgencmd", *a], capture_output=True, text=True, timeout=2).stdout
    except Exception: return ""
def cpu():
    r = []
    for l in open("/proc/stat"):
        if l.startswith("cpu") and l[3].isdigit():
            v = list(map(int, l.split()[1:])); r.append((sum(v) - v[3] - v[4], sum(v)))
    return r
p = cpu(); end = time.time() + float(sys.argv[2]); nxt = time.time() + 1
while time.time() < end:
    time.sleep(max(0, nxt - time.time())); nxt += 1
    c = cpu(); u = [100 * (a[0] - b[0]) / max(1, a[1] - b[1]) for a, b in zip(c, p)]; p = c
    t = re.search(r"([\d.]+)", vc("measure_temp")); th = vc("get_throttled").strip().split("=")[-1]
    v = re.search(r"=([\d.]+)V", vc("pmic_read_adc", "EXT5V_V"))
    out.write(f"{time.strftime('%H:%M:%S')},{t.group(1) if t else ''},{th},{v.group(1) if v else ''}," + ",".join(f"{x:.0f}" for x in u) + "\n")
