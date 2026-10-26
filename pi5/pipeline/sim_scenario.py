#!/usr/bin/env python3
"""S12-e: scripted fake STM32 + fake camera for the integrated test. Creates a NEW pty (never a real port) and plays a "lead car brakes hard" scenario in real time.

* pty side = what the Pi5 sees as /dev/ttyAMA2: sends 0x10 (50 Hz), 0x11 (10 Hz), 0x12 radar objects (16 Hz per object), 0x13 IMU (20 Hz), optional 0x14 OBD speed (3 Hz).
  It also READS whatever the pipeline writes to the pty and decodes 0x20 frames (the own-car warning) -> result file.
* camera side = JSON datagrams to 127.0.0.1:<camera-port> (camera_link.py format), a box around the projected lead car, 15 Hz.
Scenario (own car constant speed, driver does not react):  t < 2.5 s lead 25 m ahead at the same speed; from 2.5 s the lead brakes at 6 m/s^2 until it stops;
the gap is clamped at 1.5 m.  Radar dropout window (status RADAR_OK=0 + SAFE_MODE, no 0x12) and camera dropout window are optional.
After the dropout a fresh lead appears 60 m ahead at the same speed (recovery check).
usage: python3 sim_scenario.py --link PATH [--duration 16] [--obd-speed 50] [--radar-drop 9,11] [--cam-drop 11,13] [--camera-port 19100] [--result FILE]
"""
import argparse, json, os, pty, select, socket, struct, sys, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import build_frame

V_OWN = 50 / 3.6
LAT = 0.2


def lead_state(t, drop_end):
    """-> (distance m, relative speed m/s) of the lead at scenario time t (before the dropout end) or the recovery lead after it."""
    if drop_end is not None and t >= drop_end:
        return 60.0, 0.0
    if t < 2.5:
        return 25.0, 0.0
    # braking: lead speed v(t) = max(0, V_OWN - 6 (t-2.5)); gap integrates (V_OWN - v)
    tb = t - 2.5; t_stop = V_OWN / 6.0
    if tb <= t_stop:
        gap = 25.0 - 0.5 * 6.0 * tb * tb; v = V_OWN - 6.0 * tb
    else:
        gap = 25.0 - 0.5 * 6.0 * t_stop * t_stop - V_OWN * (tb - t_stop); v = 0.0
    return max(1.5, gap), v - V_OWN


def radar_payload(ts, oid, d, lat, vrel, rcs=10.0, dyn=0):
    return struct.pack("<IBhhhbBB", ts & 0xFFFFFFFF, oid, int(d * 100), int(lat * 100), int(vrel * 100), int(rcs * 2), dyn, 0xFF)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", required=True); ap.add_argument("--duration", type=float, default=16.0)
    ap.add_argument("--obd-speed", type=float, default=None); ap.add_argument("--radar-drop", default=None); ap.add_argument("--cam-drop", default=None)
    ap.add_argument("--camera-port", type=int, default=0); ap.add_argument("--result", default=None); ap.add_argument("--no-camera-boxes", action="store_true")
    a = ap.parse_args()
    drop = tuple(float(x) for x in a.radar_drop.split(",")) if a.radar_drop else None
    cdrop = tuple(float(x) for x in a.cam_drop.split(",")) if a.cam_drop else None
    master, slave = pty.openpty(); tty.setraw(master); tty.setraw(slave)
    if os.path.islink(a.link) or os.path.exists(a.link):
        os.unlink(a.link)
    os.symlink(os.ttyname(slave), a.link)
    csock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if a.camera_port else None
    fusion = None
    if csock:
        import vision_adapter                                   # sets sys.path for fusion
        import fusion as fusion
    rxbuf = bytearray(); warns = []
    t0 = time.monotonic(); nxt = dict(s=0, h=0, r=0, i=0, o=0, c=0); per = dict(s=.02, h=.1, r=.0625, i=.05, o=.33, c=.0667)
    print(f"[sim] pty slave {os.ttyname(slave)} -> {a.link}; scenario running {a.duration:g} s", flush=True)
    try:
        while True:
            t = time.monotonic() - t0
            if t >= a.duration:
                break
            ts = int(t * 1000) + 100000
            radar_down = drop is not None and drop[0] <= t < drop[1]
            d, vrel = lead_state(t, drop[1] if drop else None)
            status = (0x0F if a.obd_speed is not None else 0x0B)
            if radar_down:
                status = (status & ~0x01) | 0x10
            out = bytearray()
            if t >= nxt["s"]:
                nxt["s"] += per["s"]
                out += build_frame(0x10, struct.pack("<IBHhBB", ts, 0xFF, 0xFFFF, 0x7FFF, 0xFF, status))
            if t >= nxt["h"]:
                nxt["h"] += per["h"]; out += build_frame(0x11, struct.pack("<IBB", ts, status, 0))
            if t >= nxt["i"]:
                nxt["i"] += per["i"]
                out += build_frame(0x13, struct.pack("<Ihhhhhhhhh", ts, 0, 0, 1000, 0, 0, 0, 0, 0, 0) + b"\x01")
            if a.obd_speed is not None and t >= nxt["o"]:
                nxt["o"] += per["o"]; out += build_frame(0x14, struct.pack("<BHH", int(a.obd_speed), 0xFFFF, 0))
            if t >= nxt["r"]:
                nxt["r"] += per["r"]
                if not radar_down:
                    out += build_frame(0x12, radar_payload(ts, 7, d, LAT, vrel))
                    out += build_frame(0x12, radar_payload(ts, 9, 40.0, 8.0, 0.0, dyn=1))        # roadside clutter far to the side
            if out:
                os.write(master, bytes(out))
            if csock and t >= nxt["c"]:
                nxt["c"] += per["c"]
                if not (cdrop and cdrop[0] <= t < cdrop[1]):
                    dets = []
                    if not radar_down and not a.no_camera_boxes:
                        class O: pass
                        o = O(); o.object_id, o.forward_m, o.left_m, o.relative_mps = 7, d, LAT, vrel
                        uv = fusion.project(o, 640, 480)
                        if uv:
                            w = max(30.0, min(300.0, 900.0 / d)); h = 0.8 * w; y1 = uv[1] - 0.75 * h
                            dets.append(dict(class_name="car", bbox=[uv[0] - w / 2, y1, uv[0] + w / 2, y1 + h], confidence=0.9))
                    csock.sendto(json.dumps(dict(v=1, seq=int(t * 100), w=640, h=480, dets=dets, yolo_ms=30.0, total_ms=60.0)).encode(), ("127.0.0.1", a.camera_port))
            r, _, _ = select.select([master], [], [], 0.003)
            if r:
                try:
                    rxbuf += os.read(master, 4096)
                except OSError:
                    pass
                while True:                                     # decode 0x20 frames written by the pipeline
                    i = rxbuf.find(b"\x02")
                    if i < 0:
                        rxbuf.clear(); break
                    del rxbuf[:i]
                    if len(rxbuf) < 3:
                        break
                    n = rxbuf[2]; tot = n + 5
                    if len(rxbuf) < tot:
                        break
                    fr = bytes(rxbuf[:tot]); x = 0
                    for b in fr[1:n + 3]: x ^= b
                    if fr[-1] == 3 and x == fr[n + 3]:
                        del rxbuf[:tot]
                        if fr[1] == 0x20 and n == 4:
                            warns.append((round(t, 3), fr[3], fr[4], struct.unpack("<H", fr[5:7])[0]))
                    else:
                        warns.append((round(t, 3), "BAD_FRAME", fr.hex(), 0)); del rxbuf[0]
    finally:
        res = dict(warn_frames=len(warns), levels=sorted({w[1] for w in warns if isinstance(w[1], int)}), bad_frames=sum(1 for w in warns if w[1] == "BAD_FRAME"),
                   first_level2=next((w[0] for w in warns if w[1] == 2), None), last=warns[-1] if warns else None, frames=warns)
        if a.result:
            json.dump(res, open(a.result, "w"))
        os.unlink(a.link)
        print("[sim] done", {k: v for k, v in res.items() if k != "frames"}, flush=True)


if __name__ == "__main__":
    main()
