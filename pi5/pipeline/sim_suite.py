#!/usr/bin/env python3
"""S17-A: fake STM32 on a NEW pty playing one scenario from scenario_lib.py in real time (never a real port). Also records the 0x20 frames the pipeline writes.
usage: python3 sim_suite.py --link PATH --index N --result FILE"""
import argparse, json, os, pty, select, struct, sys, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import build_frame
import scenario_lib


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--link", required=True); ap.add_argument("--index", type=int, required=True); ap.add_argument("--result", required=True)
    a = ap.parse_args(); sc = scenario_lib.S()[a.index]
    master, slave = pty.openpty(); tty.setraw(master); tty.setraw(slave)
    if os.path.lexists(a.link): os.unlink(a.link)
    os.symlink(os.ttyname(slave), a.link)
    t0 = time.monotonic(); nxt = dict(s=0, h=0, r=0, i=0, o=0); per = dict(s=.02, h=.1, r=.0625, i=.05, o=.33)
    rxbuf = bytearray(); warns = []
    try:
        while True:
            t = time.monotonic() - t0
            if t >= sc["dur"] + 3.0: break                       # +3 s: the pipeline starts about 1-2 s later
            ts = int(t * 1000) + 100000
            ok = sc["radar_ok"](t); obd = sc["own_kmh"] is not None
            status = (0x0F if obd else 0x0B)
            if not ok: status = (status & ~0x01) | 0x10
            out = bytearray()
            if t >= nxt["s"]: nxt["s"] += per["s"]; out += build_frame(0x10, struct.pack("<IBHhBB", ts, 0xFF, 0xFFFF, 0x7FFF, 0xFF, status))
            if t >= nxt["h"]: nxt["h"] += per["h"]; out += build_frame(0x11, struct.pack("<IBB", ts, status, 0))
            if t >= nxt["i"]: nxt["i"] += per["i"]; out += build_frame(0x13, struct.pack("<Ihhhhhhhhh", ts, 0, 0, 1000, 0, 0, 0, 0, 0, 0) + b"\x01")
            if obd and t >= nxt["o"]: nxt["o"] += per["o"]; out += build_frame(0x14, struct.pack("<BHH", int(sc["own_kmh"]), 0xFFFF, 0))
            if t >= nxt["r"]:
                nxt["r"] += per["r"]
                if ok and t <= sc["dur"]:
                    for o in sc["obj"](t):
                        out += build_frame(0x12, struct.pack("<IBhhhbBB", ts & 0xFFFFFFFF, o["id"], int(o["d"] * 100), int(o["lat"] * 100), int(o["vrel"] * 100), 20, o["dyn"], 0xFF))
            if out: os.write(master, bytes(out))
            r, _, _ = select.select([master], [], [], 0.003)
            if r:
                try: rxbuf += os.read(master, 4096)
                except OSError: pass
                while True:
                    i = rxbuf.find(b"\x02")
                    if i < 0: rxbuf.clear(); break
                    del rxbuf[:i]
                    if len(rxbuf) < 3: break
                    n = rxbuf[2]; tot = n + 5
                    if len(rxbuf) < tot: break
                    fr = bytes(rxbuf[:tot]); x = 0
                    for b in fr[1:n + 3]: x ^= b
                    if fr[-1] == 3 and x == fr[n + 3]:
                        del rxbuf[:tot]
                        if fr[1] == 0x20 and n == 4: warns.append((round(t, 2), fr[3]))
                    else: del rxbuf[0]
    finally:
        json.dump(dict(warn=warns), open(a.result, "w"))
        if os.path.lexists(a.link): os.unlink(a.link)


if __name__ == "__main__":
    main()
