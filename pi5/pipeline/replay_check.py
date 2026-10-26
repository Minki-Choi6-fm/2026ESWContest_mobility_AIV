#!/usr/bin/env python3
"""Offline S1 check. Reads logs only (never opens a serial port).
usage: python3 -I replay_check.py <drive_dir|uart_xxx.bin ...>   (+ --selftest)"""
import csv
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import (ExtendedDecoder, SensorState, build_frame, MSG_LEN, NAME, SRC_OF_TYPE, TIMEOUT_MS)

GAP_REPORT_MIN_MS = {"radar": 100, "summary": 100, "imu": 100, "heartbeat": 500, "obd": 1000}


def replay(chunks, label):
    """chunks: iterable of (time_s, bytes). Returns stats dict."""
    dec, st = ExtendedDecoder(), SensorState()
    gaps = {s: [] for s in st.src}
    n = {t: 0 for t in MSG_LEN}
    first = last = None
    cross_bad = False
    for t, data in chunks:
        first = t if first is None else first
        last = t
        for f in dec.feed(data, t):
            n[f.typ] += 1
            src, gap, stale = st.update(f)
            if stale:
                gaps[src].append((f.rx - gap / 1000, gap))
    return dict(label=label, n=n, bad=dec.bad, radar_bad=dec.radar_dec.invalid_frames,
                ext_counts=dec.counts, gaps=gaps, state=st, span=(last - first) if first else 0)


def chunks_from_frames_csv(path):
    with open(path, newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            yield float(row[0]), build_frame(int(row[1], 16), bytes.fromhex(row[2]))


def chunks_from_bin(path):
    side = path[:-4] + ".csv"
    raw = open(path, "rb").read()
    if os.path.exists(side):
        with open(side, newline="") as f:
            r = csv.reader(f); next(r)
            for row in r:
                yield float(row[1]), bytes.fromhex(row[3].replace(" ", ""))
    else:
        yield 0.0, raw


def report(s):
    print(f"\n## {s['label']}  span={s['span']:.1f}s")
    print("| type | name | frames | ext-framer ok | radar-wrapper | gaps>timeout | worst gap ms |")
    print("|---|---|---|---|---|---|---|")
    for t in MSG_LEN:
        src = SRC_OF_TYPE[t]
        print(f"| 0x{t:02X} | {NAME[t]} | {s['n'][t]} | {s['ext_counts'][t]} | "
              f"{'(via wrapper)' if t==0x12 else '-'} | "
              f"{len(s['gaps'][src])} | {s['state'].src[src].worst_gap_ms:.0f} |")
    print(f"checksum/format failures: ext framer={s['bad']}, wrapped RadarPacketDecoder={s['radar_bad']}")
    return s


def selftest():
    rnd = random.Random(1)
    good = build_frame(0x10, bytes.fromhex("41790200ff500575f9ff0f")) + build_frame(0x14, bytes.fromhex("1e00100a00"))
    d = ExtendedDecoder()
    fr = d.feed(good, 1.0)
    assert [f.typ for f in fr] == [0x10, 0x14], fr
    bad = bytearray(good); bad[5] ^= 0xFF                      # corrupt a payload byte -> dropped
    d = ExtendedDecoder(); fr = d.feed(bytes(bad) + good, 2.0)
    assert [f.typ for f in fr] == [0x14, 0x10, 0x14] and d.bad >= 1, (fr, d.bad)   # corrupt 0x10 dropped, rest kept
    d = ExtendedDecoder(); fr = d.feed(bytes(rnd.randrange(256) for _ in range(300)) + good, 3.0)
    assert [f.typ for f in fr][-2:] == [0x10, 0x14]
    d = ExtendedDecoder(); a = d.feed(good[:7], 4.0); b = d.feed(good[7:], 4.1)   # split across reads
    assert a == [] and [f.typ for f in b] == [0x10, 0x14]
    st = SensorState(); st.update(fr[-2]); assert st.is_fresh("summary", fr[-2].rx + 0.05) and not st.is_fresh("summary", fr[-2].rx + 0.2)
    print("selftest OK")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--selftest" in args:
        selftest(); args.remove("--selftest")
    for a in args:
        if os.path.isdir(a):
            report(replay(chunks_from_frames_csv(os.path.join(a, "frames.csv")), os.path.basename(a.rstrip("/")) + "/frames.csv"))
        else:
            report(replay(chunks_from_bin(a), os.path.basename(a)))
