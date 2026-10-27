#!/usr/bin/env python3
"""Gaps measured with the STM32's own ms timestamp (0x10, 0x12, 0x13) -- immune to host chunk jitter."""
import csv, struct, sys
LIM = {0x10: 100, 0x12: 100, 0x13: 100}
for d in sys.argv[1:]:
    last, res = {}, {t: [] for t in LIM}
    resets = 0
    with open(d + "/frames.csv", newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            t = int(row[1], 16)
            if t not in LIM: continue
            ts = struct.unpack_from("<I", bytes.fromhex(row[2]))[0]
            if t in last:
                dt = ts - last[t]
                if dt < 0: resets += 1
                elif dt > LIM[t]: res[t].append(dt)
            last[t] = max(ts, last.get(t, 0)) if t in last and ts >= last[t] else ts
    print(d.split("/")[-1], "STM32 ts reset/backwards:", resets,
          {f"0x{t:02X}": (len(v), max(v) if v else 0) for t, v in res.items()})
