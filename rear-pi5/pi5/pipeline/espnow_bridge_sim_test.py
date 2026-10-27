#!/usr/bin/env python3
"""S12-c: protocol test of the Pi-side ESP-NOW transport with FAKE serial ports (ptys). No ESP32, no radio, no real serial port.
The ESP32 side is a Python MODEL of esp32_espnow_bridge.ino (front version: SYNC/CRC check on the serial side, re-sync, pass-through of
valid 26-byte frames to the 'air' and from the 'air' to the serial). It is a model of the sketch's logic, NOT the firmware -> the real
board/radio behaviour stays [확인 필요].  Rear = this pipeline's EspNowTransport; front = ~/front_test/v2v/front_node.py --transport esp-now
(run in its own process because both stacks use the module name 'transport').
usage: python3 -I espnow_bridge_sim_test.py"""
import os, pty, select, subprocess, sys, threading, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2v_adapter as V
from front_compat_test import rear_frame, FRONT_DIR

SYNC, END, N = 0xAA, 0x55, 26


def crc16(d, c=0xFFFF):
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def valid(f):
    return len(f) == N and f[0] == SYNC and f[-1] == END and crc16(f[1:23]) == (f[23] | f[24] << 8)


class Esp32Model:
    """Model of the .ino: serial bytes -> frames (re-sync like feedSerialByte) -> air; air -> serial."""
    def __init__(self, name):
        self.name = name; self.master, slave = pty.openpty(); tty.setraw(self.master); tty.setraw(slave)
        self.path = os.ttyname(slave); self._slave = slave
        self.sbuf = bytearray(); self.peer = None; self.tx_ok = self.tx_dropped = self.rx_ok = 0
        self.stop = False; threading.Thread(target=self._run, daemon=True).start()

    def air_in(self, frame):
        if valid(frame):
            os.write(self.master, frame); self.rx_ok += 1

    def _feed(self, c):
        if not self.sbuf and c != SYNC:
            return
        self.sbuf.append(c)
        if len(self.sbuf) < N:
            return
        if valid(bytes(self.sbuf)):
            self.tx_ok += 1
            if self.peer: self.peer.air_in(bytes(self.sbuf))
            self.sbuf.clear(); return
        self.tx_dropped += 1
        nxt = 1
        while nxt < N and self.sbuf[nxt] != SYNC: nxt += 1
        del self.sbuf[:nxt]

    def _run(self):
        while not self.stop:
            r, _, _ = select.select([self.master], [], [], 0.05)
            if r:
                try: d = os.read(self.master, 256)
                except OSError: return
                for c in d: self._feed(c)


def main():
    if not os.path.isdir(FRONT_DIR):
        print("SKIP: ~/front_test/v2v missing"); return 0
    res = []
    def check(n, ok, note=""):
        res.append(ok); print(("PASS " if ok else "FAIL ") + n + (f"  [{note}]" if note else ""), flush=True)
    A, B = Esp32Model("rear-esp"), Esp32Model("front-esp"); A.peer, B.peer = B, A
    front = subprocess.Popen([sys.executable, "-u", "front_node.py", "--no-stm32", "--transport", "esp-now", "--serial-port", B.path],
                             cwd=FRONT_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = []; threading.Thread(target=lambda: [lines.append((time.monotonic(), l.rstrip())) for l in front.stdout], daemon=True).start()
    tr = V.make_transport(0, "", "esp-now", serial_port=A.path)
    got = []
    def pump(sec, send_fn=None, period=0.1):
        end = time.monotonic() + sec; nxt = time.monotonic()
        while time.monotonic() < end:
            if send_fn and time.monotonic() >= nxt:
                send_fn(); nxt += period
            d = tr.recv_bytes(0.0)
            while d is not None:
                got.append(d); d = tr.recv_bytes(0.0)
            time.sleep(0.005)
    seq = [0]
    def send(danger):
        def f():
            fr, _ = rear_frame(seq[0], danger); tr.send_bytes(fr); seq[0] += 1
        return f
    try:
        time.sleep(1.0)
        pump(2.0, send(False))
        check("1 rear BSMs: rear Pi -> rear ESP model -> air -> front ESP model -> front_node (후방차=LIVE)", any("후방차=LIVE" in l for _, l in lines), f"rear tx_ok={A.tx_ok}")
        t0 = time.monotonic(); pump(1.5, send(True))
        on = [t for t, l in lines if "경고 ON" in l and t >= t0]
        check("2 DANGER_ACTIVE BSM through the bridge model -> front warning ON", bool(on), f"{on[0] - t0:.2f} s after the first danger BSM" if on else "")
        ids = set()
        for d in got:
            try: ids.add(V.decode(d)["vehicle_id"])
            except Exception: pass
        check("3 front BSMs return through the bridge and decode in the rear stack", ids == {2}, f"{len(got)} frames, ids {ids}")
        n0 = A.tx_ok
        fd = os.open(A.path, os.O_WRONLY | os.O_NOCTTY)
        os.write(fd, b"\x00\xaa\x01\x02ets Jun  8 2016 00:22:57\r\n"); os.close(fd)      # boot-ROM-like junk + a fake SYNC
        pump(1.0, send(True))
        check("4 junk bytes (boot text, fake SYNC) in front of valid frames do not stop forwarding", A.tx_ok > n0 + 5, f"forwarded {A.tx_ok - n0} frames after the junk")
        fr, _ = rear_frame(seq[0], True); bad = bytearray(fr); bad[10] ^= 0xFF; d0 = A.tx_dropped
        tr.send_bytes(bytes(bad)); pump(0.3)
        check("5 a corrupted frame (bad CRC) is dropped on the ESP side (not sent over the air)", A.tx_dropped > d0)
    finally:
        front.terminate()
        try: front.wait(3)
        except subprocess.TimeoutExpired: front.kill()
        A.stop = B.stop = True
    print(f"\n{sum(res)}/{len(res)} PASS  (model of the sketch only; real ESP32 / radio NOT tested)")
    return 0 if all(res) else 1


if __name__ == "__main__":
    sys.exit(main())
