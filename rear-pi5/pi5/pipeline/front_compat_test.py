#!/usr/bin/env python3
"""S11-c: compatibility loop-back test between the REAR BSM builder/receiver and the FRONT code (~/front_test/v2v, unzipped from Downloads/v2v.zip, not modified).
Local only: UDP broadcast 127.255.255.255:29000. front_node.py runs with --no-stm32 --transport udp (NO --buzzer-gpio, so no GPIO is touched).
Nothing leaves this machine, no serial port is opened. usage: python3 -I front_compat_test.py [--out DIR]"""
import argparse, math, os, re, socket, subprocess, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import v2v_adapter as V
from bsm_builder import build_fields
from speed_select import SpeedResult
from gps_position import GpsFix

FRONT_DIR = os.path.expanduser("~/front_test/v2v")
PORT = 29000
ADDR = ("127.255.255.255", PORT)
BASE_LAT, BASE_LON, M_LAT = 37.5503, 127.0731, 111_320.0


class StubState:
    def sensor_status(self, now): return 0x0F
    def is_valid(self, s, now): return True
    def radar_alive(self, now): return True


def rear_frame(seq, danger, gps_speed=True, stale=False, ok_status=True):
    """BSM built by the rear builder at the position the front's DummySource gives vehicle 1 (30 m behind vehicle 2, heading north, 36 km/h)."""
    t = time.time() % 60.0
    north = 36.0 / 3.6 * t
    fix = GpsFix(3, BASE_LAT + north / M_LAT, BASE_LON, 0.0, 36.0, time.monotonic())
    sp = SpeedResult(36.0, C.SOURCE_GPS if gps_speed else C.SOURCE_OBD, 2000 if stale else 100, "test")
    f = build_fields(StubState(), sp, fix, 0.0, time.monotonic(), danger_level=2 if danger else 0, k_measured=0.45)
    return V.encode(f, 1, seq), f


class Front:
    def __init__(self):
        self.lines = []; self.lock = threading.Lock()
        self.p = subprocess.Popen([sys.executable, "-u", "front_node.py", "--no-stm32", "--transport", "udp", "--udp-port", str(PORT),
                                   "--broadcast-addr", "127.255.255.255"], cwd=FRONT_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self._rd, daemon=True).start()

    def _rd(self):
        for ln in self.p.stdout:
            with self.lock: self.lines.append((time.monotonic(), ln.rstrip("\n")))

    def since(self, t0):
        with self.lock: return [(t, l) for t, l in self.lines if t >= t0]

    def stop(self):
        self.p.terminate()
        try: self.p.wait(5)
        except subprocess.TimeoutExpired: self.p.kill()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=None); a = ap.parse_args()
    if not os.path.isdir(FRONT_DIR):
        print("SKIP: ~/front_test/v2v does not exist (no Downloads/v2v.zip)"); return 0
    res = []
    def check(name, ok, note=""):
        res.append((name, bool(ok), note)); print(("PASS " if ok else "FAIL ") + name + (f"  [{note}]" if note else ""), flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", PORT)); sock.settimeout(0.0)
    front = Front(); time.sleep(1.5)
    rx_front = []                                                 # (t, decoded dict) of the FRONT's own BSMs, decoded by the REAR code
    tracker = V.FreshnessTracker()
    def pump():
        while True:
            try: d, _ = sock.recvfrom(256)
            except (BlockingIOError, socket.timeout): return
            try: dd = V.decode(d)
            except V.BSMDecodeError: continue
            if dd["vehicle_id"] != 1: rx_front.append((time.monotonic(), dd, tracker.update(dd)))
    seq = 0
    def send(danger, n, period=0.1, **kw):
        nonlocal seq
        t_first = None
        for _ in range(n):
            fr, f = rear_frame(seq, danger, **kw); sock.sendto(fr, ADDR); seq = (seq + 1) & 0xFFFF
            t_first = t_first or time.monotonic()
            t_end = time.monotonic() + period
            while time.monotonic() < t_end: pump(); time.sleep(0.005)
        return t_first
    try:
        # 1) the rear BSM (bits 5/6 set) passes the FRONT decode and is tracked LIVE, no drop counters
        tA = time.monotonic(); fr, f = rear_frame(0, False, gps_speed=True, stale=True)
        st = fr[22]
        send(False, 30, gps_speed=True, stale=True)
        ln = [l for t, l in front.since(tA) if l.startswith("---")]
        check("1 rear BSM with bit5 (SPEED_SRC_GPS)+bit6 (SPEED_STALE) is decoded by the front (후방차=LIVE, no drop=)", ln and "후방차=LIVE" in ln[-1] and "drop=" not in ln[-1],
              f"STATUS byte 0x{st:02x}; last front line: {ln[-1][-60:] if ln else 'none'}")
        # 2) DANGER_ACTIVE -> front warning level 2; off -> release ~1 s later
        t_send = send(True, 20)
        on = [t for t, l in front.since(t_send - 0.05) if "경고 ON" in l]
        check("2a DANGER_ACTIVE BSM -> front prints '경고 ON' (level 2)", bool(on), f"latency {1000 * (on[0] - t_send):.0f} ms after the first danger BSM" if on else "no ON line")
        lvl = [l for t, l in front.since(t_send) if l.startswith("---") and "warn=2" in l]
        check("2b front 1 s summary shows warn=2 while danger BSMs arrive", bool(lvl))
        t_last = time.monotonic(); t_off_send = send(False, 25)
        off = [t for t, l in front.since(t_last - 0.2) if "경고 OFF" in l]
        check("2c after the danger bit is cleared the front releases the warning (WARN_HOLD_S = 1 s)", bool(off) and 0.7 <= off[0] - t_last <= 1.6, f"released {off[0] - t_last:.2f} s after the last danger BSM" if off else "no OFF")
        # 4a) STALE: stop sending
        t_stop = time.monotonic(); time.sleep(2.2); pump()
        stl = [l for t, l in front.since(t_stop) if l.startswith("---") and "후방차=STALE" in l]
        check("4a no rear BSM for 2 s -> front marks 후방차=STALE (300 ms rule)", bool(stl), stl[0][-40:] if stl else "")
        # 4b) SEQ restart after > 1 s silence is accepted at once
        seq = 7000; send(False, 5); time.sleep(1.2); seq = 0; t4 = send(True, 15)
        on2 = [t for t, l in front.since(t4 - 0.05) if "경고 ON" in l]
        check("4b SEQ restarts at 0 after >1 s silence -> accepted (front SEQ_RESET_MS = 1000)", bool(on2), f"warning ON after {1000 * (on2[0] - t4):.0f} ms" if on2 else "NOT accepted")
        # 4c) SEQ restart WITHOUT silence (rear reboots within 1 s): front treats as dup until 1 s since its last accepted frame
        time.sleep(1.5); seq = 6000; send(False, 10); seq = 0; t5 = time.monotonic(); send(True, 25)
        on3 = [t for t, l in front.since(t5 - 0.05) if "경고 ON" in l]
        d3 = (on3[0] - t5) if on3 else None
        check("4c SEQ restart WITHOUT a 1 s pause: front ignores the new BSMs as duplicates for a while (documented, not fixed)", d3 is None or d3 > 0.5,
              f"warning ON only after {d3:.2f} s" if d3 is not None else "never within 2.5 s")
        # 4d) S12: clock-based SEQ start (pipeline_main: seq = int(time.time()*BSM_RATE_HZ) & 0xFFFF). A rear process that restarts 0.3 s after the old one stopped
        #     starts ahead of the old SEQ, so the front accepts it at once (no 1 s wait)
        time.sleep(1.5)
        seq = int(time.time() * C.BSM_RATE_HZ) & 0xFFFF; send(False, 10)                  # "old process": 10 Hz, seq follows the clock
        time.sleep(0.3)
        seq = int(time.time() * C.BSM_RATE_HZ) & 0xFFFF; t6 = send(True, 15)              # "restarted process": clock-based start again
        on4 = [t for t, l in front.since(t6 - 0.05) if "경고 ON" in l]
        check("4d clock-based SEQ start: a restart after 0.3 s is accepted at once by the front (was ~1 s late with SEQ 0)", bool(on4) and on4[0] - t6 < 0.4,
              f"warning ON after {on4[0] - t6:.2f} s" if on4 else "not accepted")
        # 3) FRONT BSMs (real front_node output) handled by the rear receiver
        n_ok = sum(1 for t, dd, ok in rx_front if ok)
        pump()
        fr_ids = {dd["vehicle_id"] for t, dd, ok in rx_front}
        check("3a front_node BSMs decode in the rear stack (CRC, version) and pass the rear freshness check", n_ok > 20 and fr_ids == {2}, f"{n_ok} accepted of {len(rx_front)}, ids {fr_ids}")
        if rx_front:
            dd = rx_front[-1][1]
            check("3b front BSM fields read correctly (36 km/h, heading 0, GPS valid, brake=0 without STM32)", abs(dd["speed_kmh"] - 36.0) < 0.1 and dd["gps_valid"] and not dd["brake_active"], f"status_raw 0x{dd['status_raw']:02x}")
    finally:
        front.stop(); sock.close()
    # 3c) a front BSM with bit4 BRAKE_ACTIVE (made with the FRONT's own bsm_protocol in its own interpreter) through the rear decode/freshness
    code = ("import sys;sys.path.insert(0,'.');import bsm_protocol as b;"
            "fr=[b.encode(b.BSM(vehicle_id=2,seq=s,timestamp_ms=1000+s*100,lat=37.55,lon=127.07,speed_kmh=40.0,heading_deg=0.0,k_value=0.6,"
            "status=b.STATUS_GPS_VALID|b.STATUS_BRAKE_ACTIVE)) for s in (10,11,12)];print(' '.join(x.hex() for x in fr))")
    out = subprocess.run([sys.executable, "-I", "-c", code], cwd=FRONT_DIR, capture_output=True, text=True).stdout.split()
    tr = V.FreshnessTracker(); oks = []; last = None
    for h in out:
        dd = V.decode(bytes.fromhex(h)); oks.append(tr.update(dd)); last = dd
    check("3c front BSM with bit4 BRAKE_ACTIVE: rear decode shows brake_active, 3 frames accepted, not stale", last and last["brake_active"] and all(oks) and not tr.is_stale(2),
          f"status_raw 0x{last['status_raw']:02x}" if last else "")
    # 3d) front GpioBuzzer on a mock pin only
    code2 = ("from gpiozero import Device;from gpiozero.pins.mock import MockFactory;Device.pin_factory=MockFactory();"
             "import sys;sys.path.insert(0,'.');from buzzer import GpioBuzzer;b=GpioBuzzer(26);b.set_level(2);import time;time.sleep(0.5);b.set_level(0);b.close();print('MOCK_OK',type(Device.pin_factory).__name__)")
    r = subprocess.run([sys.executable, "-I", "-c", code2], cwd=FRONT_DIR, capture_output=True, text=True)
    check("3d front GpioBuzzer(26) runs on a MockFactory pin (no real GPIO)", "MOCK_OK MockFactory" in r.stdout, (r.stdout + r.stderr).strip()[-80:])
    bad = [n for n, ok, _ in res if not ok]
    print(f"\n{len(res) - len(bad)}/{len(res)} PASS" + ("" if not bad else "  FAIL: " + "; ".join(bad)))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        open(os.path.join(a.out, "front_compat_result.txt"), "w").write("\n".join(("PASS " if ok else "FAIL ") + n + (f"  [{note}]" if note else "") for n, ok, note in res))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
