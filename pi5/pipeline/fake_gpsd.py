#!/usr/bin/env python3
"""Desk-test fake gpsd: replays <drive>/gps.jsonl as gpsd JSON on a LOCAL port (never 2947, never touches the real gpsd).
Aligned to the first frame time of <drive>/frames.csv so it matches fake_stm32.py started at the same moment."""
import argparse, csv, json, os, select, signal, socket, time

REAL_GPSD_PORT = 2947


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("drive"); ap.add_argument("--port", type=int, default=22947)
    ap.add_argument("--from", dest="start", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=1.0); ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--no-fix", type=float, default=0, help="seconds (from --no-fix-at) during which TPV is sent with mode 1 / no speed")
    ap.add_argument("--no-fix-at", type=float, default=10)
    a = ap.parse_args()
    if a.port == REAL_GPSD_PORT:
        raise SystemExit("refusing to use the real gpsd port 2947")
    with open(os.path.join(a.drive, "frames.csv"), newline="") as f:
        r = csv.reader(f); next(r); base = float(next(r)[0]) + a.start
    msgs = []
    for line in open(os.path.join(a.drive, "gps.jsonl")):
        try:
            m = json.loads(line)
        except ValueError:
            continue
        msgs.append((m["t"], m["msg"]))
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port)); srv.listen(4); srv.setblocking(False)
    print(f"[fake_gpsd] listening 127.0.0.1:{a.port}", flush=True)
    clients, i, sent = [], 0, 0
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    start = time.monotonic()
    while i < len(msgs) and msgs[i][0] < base:
        i += 1                                          # skip history before the start offset
    while not stop and i < len(msgs):
        rel = (time.monotonic() - start) * a.speed
        if a.duration and rel > a.duration:
            break
        r, _, _ = select.select([srv] + clients, [], [], 0.02)
        for s in r:
            if s is srv:
                c, _ = srv.accept(); c.setblocking(False); clients.append(c)
                c.sendall((json.dumps({"class": "VERSION", "release": "fake", "proto_major": 3, "proto_minor": 15}) + "\n").encode())
            else:
                try:
                    if not s.recv(4096):
                        raise ConnectionError
                except BlockingIOError:
                    pass
                except OSError:
                    clients.remove(s); s.close()
        while i < len(msgs) and msgs[i][0] - base <= rel:
            m = dict(msgs[i][1])
            if a.no_fix > 0 and a.no_fix_at <= rel < a.no_fix_at + a.no_fix and m.get("class") == "TPV":
                m.update(mode=1); m.pop("speed", None)
            data = (json.dumps(m) + "\n").encode()
            for c in list(clients):
                try:
                    c.sendall(data); sent += 1
                except OSError:
                    clients.remove(c); c.close()
            i += 1
    print(f"[fake_gpsd] done, sent {sent} lines", flush=True)


if __name__ == "__main__":
    main()
