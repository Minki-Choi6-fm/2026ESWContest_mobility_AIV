"""S3 (2): read-only gpsd POSITION provider (lat, lon, heading, fix mode, age). New file; GpsdClient in live_receiver.py is untouched.

Subscribes to localhost gpsd (WATCH json; no gpsd config change). Non-blocking: call pump(now) when the socket is readable.
Also satisfies speed_select.GpsProvider via latest() -> (speed_kmh, rx_time) so one client can feed both get_speed() and the BSM.
"""
import json
import socket
from dataclasses import dataclass
from typing import Optional

from constants import GPS_POS_MAX_AGE_MS


@dataclass(frozen=True)
class GpsFix:
    mode: int                      # 0/1 = no fix, 2 = 2D, 3 = 3D
    lat: Optional[float]
    lon: Optional[float]
    heading_deg: Optional[float]   # gpsd "track" (course over ground); often missing when standing still
    speed_kmh: Optional[float]
    rx: float                      # receive time of this TPV (same clock as `now`)


class GpsPositionClient:
    def __init__(self, host="127.0.0.1", port=2947):
        self.host, self.port = host, port
        self.sock, self.buf, self.retry = None, b"", 0.0
        self.tpv: Optional[GpsFix] = None          # newest TPV of any mode (tells "no fix" apart from "no data")
        self.last_heading: Optional[float] = None  # last known course, kept for standing-still moments
        self.fixes = 0

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
            self.sock.close(); self.sock, self.buf = None, b""
            return
        self.buf += d
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                m = json.loads(line)
            except ValueError:
                continue
            if m.get("class") != "TPV":
                continue
            num = lambda k: m[k] if isinstance(m.get(k), (int, float)) else None
            mode = m.get("mode", 0)
            fix = GpsFix(mode, num("lat"), num("lon"), num("track"), None if num("speed") is None else num("speed") * 3.6, now)
            self.tpv = fix
            if mode >= 2 and fix.lat is not None and fix.lon is not None:
                self.fixes += 1
                if fix.heading_deg is not None:
                    self.last_heading = fix.heading_deg

    # --- position API
    def position(self, now) -> Optional[GpsFix]:
        """Newest TPV if it is a usable fix (mode>=2, lat/lon present) and not older than GPS_POS_MAX_AGE_MS, else None."""
        f = self.tpv
        if f is None or f.mode < 2 or f.lat is None or f.lon is None:
            return None
        return f if 0 <= (now - f.rx) * 1000 <= GPS_POS_MAX_AGE_MS else None

    def age_ms(self, now) -> Optional[float]:
        return None if self.tpv is None else (now - self.tpv.rx) * 1000

    # --- speed_select.GpsProvider
    def latest(self):
        f = self.tpv
        if f is None or f.mode < 2 or f.speed_kmh is None:
            return None
        return (f.speed_kmh, f.rx)

    def fileno(self):
        return self.sock.fileno() if self.sock else -1

    def close(self):
        if self.sock:
            self.sock.close()
