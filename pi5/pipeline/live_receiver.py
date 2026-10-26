#!/usr/bin/env python3
"""S2.5 live receiver: STM32 UART (read-only) + gpsd (read-only) -> ExtendedDecoder/SensorState/get_speed().

* Opens the UART O_RDONLY. Never writes. Only touches termios if the line is not already 115200 raw 8N1.
* Aborts if any process other than the known write-only ego_speed_tx.py holds the port (fuser).
* gpsd: localhost:2947 WATCH (subscription only, no gpsd config change).
* Once a second prints one status line; every --stats-every s appends frame_OK/frame_BAD counters to a CSV.
Modes:  live (default) | --replay <drive_dir> | --selftest
"""
import argparse, bisect, csv, json, os, select, signal, socket, subprocess, sys, termios, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logging_tools"))
from aiv_uart_rx import decode as aiv_decode            # same decoder drive_logger.py uses for the "decoded" column
from constants import TIMEOUT_MS
from speed_select import get_speed
from uart_ingest import ExtendedDecoder, SensorState, build_frame, MSG_LEN, SRC_OF_TYPE

DEFAULT_PORT = "/dev/ttyAMA2"
ALLOWED_HOLDERS = ("ego_speed_tx.py",   # write-only (verified by reading the code, S0)
                   "fake_stm32.py")     # desk-test pty owner: holds the slave open but never reads it
STATS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs", "live_receiver")
SRC_ORDER = ["summary", "heartbeat", "radar", "imu", "obd"]
# SENSOR_STATUS bits (UART v0.2 0x10 / 0x11)
STATUS_BITS = [(0, "RADAR"), (1, "IMU"), (2, "OBD"), (3, "CAN"), (4, "SAFE_MODE"), (5, "BRAKE")]


def status_text(st):
    """0x0F -> 'sts=0x0F[RADAR IMU OBD CAN]'; None (no 0x10 yet) -> 'sts=--'."""
    if st is None:
        return "sts=--"
    return f"sts=0x{st:02X}[" + " ".join(n for b, n in STATUS_BITS if st >> b & 1) + "]"


class LossTracker:
    """Frame loss from the STM32's own ms clock. For each pair of consecutive frames, missing = round(dt/period) - 1
    (so 21-24 ms jitter at a 20 ms period counts as 0 missing, a 40 ms gap as 1, 60 ms as 2).
    A backwards clock (STM32 restart) starts a new segment: the silent gap of a reset is NOT counted as loss.
    Frames lost at the very end of the observation are invisible (no later frame reveals them)."""

    def __init__(self, period_ms):
        self.period, self.last, self.cnt, self.missing = period_ms, None, 0, 0

    def add(self, ts):
        if self.last is not None and ts > self.last:
            self.missing += max(0, round((ts - self.last) / self.period) - 1)
        self.last, self.cnt = ts, self.cnt + 1

    def totals(self):
        return self.cnt + self.missing, self.cnt

    def loss_pct(self):
        e, r = self.totals()
        return None if e == 0 else 100.0 * (e - r) / e


class Recorder:
    """Writes frames.csv / radar.csv in drive_logger.py's format (same header, columns, number formats) so that
    ONE process can own the port in the car. Does not replace drive_logger's other outputs (camera, gps, sys)."""

    def __init__(self, outdir, meta):
        os.makedirs(outdir, exist_ok=True)
        fp, rp = os.path.join(outdir, "frames.csv"), os.path.join(outdir, "radar.csv")
        fresh_f, fresh_r = not os.path.exists(fp) or os.path.getsize(fp) == 0, not os.path.exists(rp) or os.path.getsize(rp) == 0
        self.ff, self.rf = open(fp, "a", newline="", encoding="utf-8"), open(rp, "a", newline="", encoding="utf-8")
        self.fw, self.rw = csv.writer(self.ff), csv.writer(self.rf)
        if fresh_f:
            self.fw.writerow(["time", "type", "payload_hex", "decoded"])
        if fresh_r:
            self.rw.writerow(["time", "id", "long_m", "lat_m", "vrel_ms", "rcs_dbsm", "dyn"])
        with open(os.path.join(outdir, "meta_live_receiver.json"), "w") as f:
            json.dump(meta, f, indent=1, default=str)
        self._flush_t = time.monotonic()
        self.written = 0

    def write(self, frames, wall):
        for fr in frames:
            pl = fr.payload
            if not pl:                                  # 0x12 payload unknown (framers disagreed) -> cannot record faithfully
                continue
            d = None
            try:
                d = aiv_decode(fr.typ, pl)
                dec = json.dumps(d, default=str, ensure_ascii=False) if d is not None else ""
            except Exception:                           # noqa: BLE001  (same tolerance as drive_logger)
                dec = ""
            self.fw.writerow([f"{wall:.4f}", f"0x{fr.typ:02X}", pl.hex(), dec])
            self.written += 1
            if fr.typ == 0x12 and isinstance(d, dict) and "long_m" in d:
                self.rw.writerow([f"{wall:.4f}", d.get("id"), d.get("long_m"), d.get("lat_m"),
                                  d.get("vrel_ms"), d.get("rcs_dbsm"), d.get("dyn")])
        if time.monotonic() - self._flush_t >= 1.0:
            self.ff.flush(); self.rf.flush(); self._flush_t = time.monotonic()

    def close(self):
        self.ff.close(); self.rf.close()


# ----------------------------------------------------------------- port access
def port_holders(port):
    """[(pid, cmdline)] of processes that have the port open."""
    r = subprocess.run(["fuser", port], capture_output=True, text=True)
    pids = [int(x) for x in r.stdout.split() if x.isdigit()]
    out = []
    for pid in pids:
        try:
            out.append((pid, open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode().strip()))
        except OSError:
            out.append((pid, "?"))
    return out


def open_readonly(port, baud=115200):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    speed = getattr(termios, f"B{baud}")
    a = termios.tcgetattr(fd)                    # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    want = list(a)
    want[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR
                 | termios.IGNCR | termios.ICRNL | termios.IXON | termios.IXOFF)
    want[1] &= ~termios.OPOST
    want[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    want[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    want[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    want[4] = want[5] = speed
    changed = want[:6] != a[:6]
    if changed:
        termios.tcsetattr(fd, termios.TCSANOW, want)
    return fd, changed


# ----------------------------------------------------------------- GPS providers (get_speed's GpsProvider)
class GpsdClient:
    """Non-blocking read-only gpsd client. latest() -> (speed_kmh, rx_monotonic) of newest TPV with mode>=2."""

    def __init__(self, host="127.0.0.1", port=2947):
        self.host, self.port, self.sock, self.buf, self.last, self.retry, self.fixes = host, port, None, b"", None, 0.0, 0

    def fileno(self):
        return self.sock.fileno() if self.sock else -1

    def connect(self, now):
        if self.sock or now - self.retry < 2.0:
            return
        self.retry = now
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.3)
            s.sendall(b'?WATCH={"enable":true,"json":true};\n')
            s.setblocking(False)
            self.sock = s
        except OSError:
            self.sock = None

    def pump(self, now):
        if not self.sock:
            return
        try:
            d = self.sock.recv(8192)
            if not d:
                raise ConnectionError
        except BlockingIOError:
            return
        except OSError:
            self.sock.close(); self.sock = None; self.buf = b""; return
        self.buf += d
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                m = json.loads(line)
            except ValueError:
                continue
            sp = m.get("speed")
            if m.get("class") == "TPV" and m.get("mode", 0) >= 2 and isinstance(sp, (int, float)) and sp >= 0:
                self.last = (sp * 3.6, now); self.fixes += 1

    def latest(self):
        return self.last

    def close(self):
        if self.sock:
            self.sock.close()


class ReplayGps:
    def __init__(self, drive):
        self.f = []
        gp = os.path.join(drive, "gps.jsonl")
        for line in (open(gp) if os.path.exists(gp) else []):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            m = r.get("msg", {})
            if m.get("class") == "TPV" and m.get("mode", 0) >= 2 and isinstance(m.get("speed"), (int, float)):
                self.f.append((r["t"], m["speed"] * 3.6))
        self.t = [x[0] for x in self.f]; self.now = 0.0; self.fixes = len(self.f)

    def latest(self):
        i = bisect.bisect_right(self.t, self.now) - 1
        return None if i < 0 else (self.f[i][1], self.f[i][0])


# ----------------------------------------------------------------- core (shared by live / replay / selftest)
class Receiver:
    def __init__(self, gps, stats_path=None, stats_every=5.0, recorder=None):
        self.recorder = recorder
        self.stm_resets, self.last_uptime, self.last_reset_cause = 0, None, None
        self.loss = {0x10: LossTracker(20), 0x11: LossTracker(100)}      # 50 Hz, 10 Hz
        self.status_hist = {}                                           # 0x10 SENSOR_STATUS value -> count
        self.dec, self.state, self.gps = ExtendedDecoder(), SensorState(), gps
        self.to_events = {s: 0 for s in SRC_ORDER}       # fresh->stale transitions seen by the 1 Hz-or-faster poll
        self._was_stale = {s: False for s in SRC_ORDER}
        self.read_errors = 0
        self.backlog_bytes = 0
        self.t0 = None
        self.stats_path, self.stats_every, self._next_stats = stats_path, stats_every, None
        self.stats_f = None
        if stats_path:
            os.makedirs(os.path.dirname(stats_path), exist_ok=True)
            new = not os.path.exists(stats_path)
            self.stats_f = open(stats_path, "a", newline="")
            self.stats_w = csv.writer(self.stats_f)
            if new:
                self.stats_w.writerow(["wall_time", "uptime_s", "bytes_rx", "ok_0x10", "ok_0x11", "ok_0x12", "ok_0x13",
                                       "ok_0x14", "bad_len", "bad_chk", "resync_events", "junk_bytes",
                                       "radar_wrapper_bad", "read_errors", "backlog_discarded", "stm_resets", "expected_0x10", "expected_0x11",
                                       "loss_pct_0x10", "loss_pct_0x11", "loss_pct"]
                                      + [("timeouts_radar_0x12" if s == "radar" else f"timeouts_{s}") for s in SRC_ORDER])

    def feed(self, data, now, wall=None):
        if self.t0 is None:
            self.t0 = now
        frames = self.dec.feed(data, now)
        for fr in frames:
            self.state.update(fr)
            if fr.typ == 0x10:
                self.loss[0x10].add(fr.data["ts"])
                self.status_hist[fr.data["status"]] = self.status_hist.get(fr.data["status"], 0) + 1
            if fr.typ == 0x11:
                self.loss[0x11].add(fr.data["uptime_ms"])                                   # uptime going backwards = STM32 restarted
                up = fr.data["uptime_ms"]
                if self.last_uptime is not None and up < self.last_uptime:
                    self.stm_resets += 1; self.last_reset_cause = fr.data["reset_cause"]
                self.last_uptime = up
        if self.recorder:
            self.recorder.write(frames, time.time() if wall is None else wall)
        return frames

    def loss_text(self):
        p = self.loss_pct_all()
        return "--" if p is None else f"{p:.2f}%"

    def loss_pct_all(self):
        e = sum(t.totals()[0] for t in self.loss.values()); r = sum(t.totals()[1] for t in self.loss.values())
        return None if e == 0 else max(0.0, 100.0 * (e - r) / e)

    def sensor_status(self):
        """Newest SENSOR_STATUS byte from 0x10 (None if no 0x10 yet)."""
        v = self.state.src["summary"].value
        return None if not v else v["status"]

    def poll(self, now):
        """Call at least every ~50 ms: counts fresh->stale transitions per source."""
        for s in SRC_ORDER:
            a = self.state.age_ms(s, now)
            stale = a is not None and a > self.state.timeout_ms[s]
            if stale and not self._was_stale[s]:
                self.to_events[s] += 1
            self._was_stale[s] = stale

    def status_line(self, now):
        c, d = self.dec.counts, self.dec
        res = get_speed(self.state, self.gps, now)
        spd = f"NONE[{res.reason}]" if res.speed_kmh is None else f"{res.speed_kmh:.0f}km/h {'OBD' if res.source == 0 else 'GPS'} age={res.age_ms}ms"
        tos = []
        for s in SRC_ORDER:
            a = self.state.age_ms(s, now)
            tos.append(f"{'radar_0x12' if s == 'radar' else s}={'--' if a is None else ('TIMEOUT' if a > self.state.timeout_ms[s] else 'ok')}")
        return (f"t={now - (self.t0 or now):6.0f}s | OK " + " ".join(f"{t:02X}:{c[t]}" for t in MSG_LEN)
                + f" | BAD len={d.bad_len} chk={d.bad_chk} resync={d.resync_events}"
                + f" | loss={self.loss_text()} | speed={spd} | " + status_text(self.sensor_status()) + " | " + " ".join(tos)
                + (f" | STM_RESETS={self.stm_resets}(cause={self.last_reset_cause})" if self.stm_resets else ""))

    def write_stats(self, now, wall):
        if not self.stats_f:
            return
        d, c = self.dec, self.dec.counts
        self.stats_w.writerow([f"{wall:.3f}", f"{now - (self.t0 or now):.1f}", d.bytes_rx] + [c[t] for t in MSG_LEN]
                              + [d.bad_len, d.bad_chk, d.resync_events, d.junk_bytes, d.radar_dec.invalid_frames,
                                 self.read_errors, self.backlog_bytes, self.stm_resets, self.loss[0x10].totals()[0], self.loss[0x11].totals()[0]]
                              + [("" if self.loss[t].loss_pct() is None else f"{self.loss[t].loss_pct():.3f}") for t in (0x10, 0x11)]
                              + [("" if self.loss_pct_all() is None else f"{self.loss_pct_all():.3f}")]
                              + [self.to_events[s] for s in SRC_ORDER])
        self.stats_f.flush()

    def maybe_stats(self, now, wall):
        if self._next_stats is None:
            self._next_stats = now + self.stats_every
        elif now >= self._next_stats:
            self.write_stats(now, wall); self._next_stats = now + self.stats_every

    def close(self, now, wall):
        self.write_stats(now, wall)
        if self.stats_f:
            self.stats_f.close()


# ----------------------------------------------------------------- modes
def run_live(a):
    holders = [h for h in port_holders(a.port) if h[0] != os.getpid()]
    others = [h for h in holders if not any(n in h[1] for n in ALLOWED_HOLDERS)]
    if others:
        print(f"ABORT: {a.port} is held by {others}. Not opening it. Stop that reader first / tell the owner.", file=sys.stderr)
        return 2
    if holders:
        print(f"note: {a.port} also held by non-reading {[h[1] for h in holders]} (allowed)")
    gps = GpsdClient(port=a.gpsd_port)
    stats_path = os.path.join(a.stats_dir, time.strftime("rx_stats_%Y%m%d_%H%M%S.csv"))
    rec = Recorder(a.record, {"tool": "live_receiver.py", "port": a.port, "gpsd_port": a.gpsd_port, "start": time.time(),
                              "note": "frames.csv/radar.csv only (drive_logger format); no camera/gps/sys"}) if a.record else None
    rx = Receiver(gps, stats_path, a.stats_every, rec)
    fd, changed = open_readonly(a.port, a.baud)
    print(f"opened {a.port} O_RDONLY {a.baud} 8N1 (termios changed: {changed}); stats -> {stats_path}")
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    # Unread UART data piles up in the kernel (seen: ~1.3 MB = ~15 min). Discard that stale backlog, report its size.
    backlog, td = 0, time.monotonic()
    while time.monotonic() - td < 2.0:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            break
        if not chunk:
            break
        backlog += len(chunk)
    print(f"discarded stale kernel backlog: {backlog} bytes (= {backlog / 1470:.0f} s at the desk rate of ~1.47 kB/s; with radar objects the rate is several kB/s)")
    rx.backlog_bytes = backlog
    start = time.monotonic(); rx.t0 = start; next_print = start + 1.0
    try:
        while not stop and (a.duration <= 0 or time.monotonic() - start < a.duration):
            now = time.monotonic()
            gps.connect(now)
            rl = [fd] + ([gps.sock] if gps.sock else [])
            ready, _, _ = select.select(rl, [], [], 0.05)
            now = time.monotonic()
            for r in ready:
                if r is fd:
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        data = b""
                    except OSError:
                        rx.read_errors += 1; data = b""
                    if data:
                        rx.feed(data, now, time.time())
                else:
                    gps.pump(now)
            rx.poll(now)
            if now >= next_print:
                print(rx.status_line(now), flush=True); next_print += 1.0
            rx.maybe_stats(now, time.time())
    finally:
        os.close(fd); gps.close()
        rx.close(time.monotonic(), time.time())
        if rec:
            rec.close()
    print("final:", rx.status_line(time.monotonic()))
    print("SENSOR_STATUS values seen (0x10):", {f"0x{k:02X}": v for k, v in sorted(rx.status_hist.items())},
          "| STM32 clock loss: " + ", ".join(f"0x{t:02X} exp={rx.loss[t].totals()[0]} got={rx.loss[t].totals()[1]}" for t in rx.loss))
    print("timeout events:", rx.to_events, "| gpsd TPV fixes:", gps.fixes, "| bytes_rx:", rx.dec.bytes_rx)
    return 0


def run_replay(a):
    d = a.replay
    gps = ReplayGps(d)
    stats_path = os.path.join(a.stats_dir, "replay_" + os.path.basename(d.rstrip("/")) + ".csv") if a.stats_dir_given else None
    rec = Recorder(a.record, {"tool": "live_receiver.py --replay", "source": d}) if a.record else None
    rx = Receiver(gps, stats_path, a.stats_every, rec)
    nxt = None; lines = 0; last = 0.0
    with open(os.path.join(d, "frames.csv"), newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            t = float(row[0]); last = t
            rx.feed(build_frame(int(row[1], 16), bytes.fromhex(row[2])), t, t)
            nxt = t + 1.0 if nxt is None else nxt
            while t >= nxt:
                gps.now = nxt; rx.poll(nxt); rx.maybe_stats(nxt, nxt)
                if a.replay_print_every and lines % a.replay_print_every == 0:
                    print(rx.status_line(nxt))
                lines += 1; nxt += 1.0
    rx.close(last, last)
    if rec:
        rec.close()
    print("final:", rx.status_line(last)); print("timeout events (1 s poll in replay):", rx.to_events)
    return 0


def run_selftest():
    import pty
    # 1) decoder error categories
    good = build_frame(0x10, bytes.fromhex("41790200ff500575f9ff0f")) + build_frame(0x14, bytes.fromhex("1e00100a00"))
    bad_chk = bytearray(good[:14]); bad_chk[5] ^= 0xFF
    bad_len = bytes((2, 0x14, 7, 1, 2, 3, 4, 5, 6, 7, 0, 3))
    rx = Receiver(None)
    rx.feed(b"\x55\x55" + bytes(bad_chk) + bad_len + good + b"\x00\x01" + good, 1.0)
    d = rx.dec
    assert d.counts[0x10] == 2 and d.counts[0x14] == 2, d.counts
    assert d.bad_chk >= 1 and d.bad_len >= 1 and d.resync_events >= 1 and d.bad == d.bad_len + d.bad_chk, vars(d).keys()
    # 2) pty: open read-only like the real port, feed frames, read them back
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fd, changed = open_readonly(path)
    os.write(master, good + good)
    time.sleep(0.1)
    rx2 = Receiver(None); rx2.feed(os.read(fd, 4096), time.monotonic())
    assert rx2.dec.counts[0x10] == 2 and rx2.dec.counts[0x14] == 2
    os.close(fd); os.close(master); os.close(slave)
    # 3) get_speed wiring: OBD 30 km/h age 10 ms
    res = get_speed(rx2.state, None, time.monotonic())
    assert res.source == 0 and res.speed_kmh == 30.0, res
    # 4) abort path uses fuser: this very process opens a file -> holder list works
    print("selftest OK (error categories, read-only pty read, get_speed wiring)")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = until Ctrl+C")
    ap.add_argument("--gpsd-port", type=int, default=2947, help="gpsd TCP port on localhost (default 2947)")
    ap.add_argument("--record", metavar="DIR", help="also write frames.csv + radar.csv (drive_logger format) into DIR")
    ap.add_argument("--stats-dir", default=None)
    ap.add_argument("--stats-every", type=float, default=5.0)
    ap.add_argument("--replay", help="drive folder (frames.csv + gps.jsonl)")
    ap.add_argument("--replay-print-every", type=int, default=0, help="print every Nth virtual second (0=only final)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    a.stats_dir_given = a.stats_dir is not None
    a.stats_dir = os.path.abspath(a.stats_dir or STATS_DIR)
    sys.exit(run_selftest() if a.selftest else run_replay(a) if a.replay else run_live(a))
