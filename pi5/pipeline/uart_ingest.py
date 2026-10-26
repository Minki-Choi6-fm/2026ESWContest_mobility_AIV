"""S1 (offline): STM32->Pi5 frame decoder for 0x10-0x14 + per-source state with timeouts.

Wraps fusion.RadarPacketDecoder (not modified): 0x12 comes from the wrapped decoder,
the other types from a small framer here. Pure functions / no port access -- the
caller feeds bytes (live reader later, or replay from logs).
Framing (UART v0.2): STX 02 | TYPE | LEN | PAYLOAD | XOR(TYPE,LEN,PAYLOAD) | ETX 03.
Bad checksum -> drop, resync at next STX, no retry (rule 7).
"""
import os
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vehicle_algorithm"))
from fusion import RadarPacketDecoder, RadarObject  # noqa: E402

MSG_LEN = {0x10: 11, 0x11: 6, 0x12: 14, 0x13: 23, 0x14: 5}
NAME = {0x10: "SENSOR_SUMMARY", 0x11: "HEARTBEAT", 0x12: "RADAR_OBJECT", 0x13: "IMU_DATA", 0x14: "OBD_DATA"}

from constants import TIMEOUT_MS  # noqa: E402  (all timeouts live in constants.py)
SRC_OF_TYPE = {0x10: "summary", 0x11: "heartbeat", 0x12: "radar", 0x13: "imu", 0x14: "obd"}


def xor_chk(b) -> int:
    c = 0
    for x in b:
        c ^= x
    return c


def build_frame(typ: int, payload: bytes) -> bytes:
    return bytes((0x02, typ, len(payload))) + payload + bytes((xor_chk(bytes((typ, len(payload))) + payload), 0x03))


def decode_payload(typ: int, p: bytes) -> dict:
    """Same field scaling as aiv_uart_rx.decode (UART v0.2)."""
    if typ == 0x10:
        ts, k, dist, rel, danger, st = struct.unpack("<IBHhBB", p)
        return dict(ts=ts, K=None if k == 0xFF else k / 100, dist_m=None if dist == 0xFFFF else dist / 100,
                    rel_ms=None if rel == 0x7FFF else rel / 100, danger=None if danger == 0xFF else danger, status=st)
    if typ == 0x11:
        up, st, rc = struct.unpack("<IBB", p)
        return dict(uptime_ms=up, status=st, reset_cause=rc)
    if typ == 0x13:
        v = struct.unpack("<Ihhhhhhhhh", p[:22])
        fl = p[22]
        return dict(ts=v[0], acc_mg=v[1:4], gyro_dps=tuple(x / 10 for x in v[4:7]),
                    rpy_deg=tuple(x / 10 for x in v[7:10]), imu_valid=bool(fl & 1), brake=bool(fl & 2))
    if typ == 0x14:
        spd, rpm, age = struct.unpack("<BHH", p)
        return dict(speed_kmh=None if spd == 0xFF else spd, rpm=None if rpm == 0xFFFF else rpm,
                    age_ms=None if age == 0xFFFF else age)
    raise ValueError(typ)


@dataclass
class Frame:
    typ: int
    payload: bytes
    rx: float          # host monotonic/epoch time of the chunk that completed it
    data: Optional[dict] = None   # decoded (0x12: RadarObject in .radar)
    radar: Optional[RadarObject] = None


class ExtendedDecoder:
    """Wraps RadarPacketDecoder (0x12) and adds 0x10/0x11/0x13/0x14. Never blocks."""

    def __init__(self):
        self.radar_dec = RadarPacketDecoder()
        self.buf = bytearray()
        self.bad = 0                       # total failures (= bad_len + bad_chk)
        self.bad_len = 0                   # unknown TYPE or LEN != fixed LEN of that TYPE
        self.bad_chk = 0                   # ETX != 0x03 or XOR checksum mismatch
        self.resync_events = 0             # times leading non-STX bytes were skipped to find the next STX
        self.junk_bytes = 0                # how many bytes those skips discarded
        self.bytes_rx = 0
        self.counts = {t: 0 for t in MSG_LEN}

    def feed(self, data: bytes, rx: Optional[float] = None) -> list:
        rx = time.monotonic() if rx is None else rx
        radar_objs = self.radar_dec.feed(data, rx)       # wrapped, unmodified
        out = []
        radar_pls = []                                # raw 0x12 payloads, same order as radar_objs
        self.buf += data
        self.bytes_rx += len(data)
        while True:
            i = self.buf.find(b"\x02")
            if i < 0:
                if self.buf:
                    self.resync_events += 1; self.junk_bytes += len(self.buf)
                self.buf.clear(); break
            if i:
                self.resync_events += 1; self.junk_bytes += i
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            t, n = self.buf[1], self.buf[2]
            if t not in MSG_LEN or MSG_LEN[t] != n:
                self.bad += 1; self.bad_len += 1; del self.buf[0]; continue
            total = n + 5
            if len(self.buf) < total:
                break
            if self.buf[total - 1] != 0x03 or xor_chk(self.buf[1:n + 3]) != self.buf[n + 3]:
                self.bad += 1; self.bad_chk += 1; del self.buf[0]; continue
            pl = bytes(self.buf[3:3 + n])
            del self.buf[:total]
            self.counts[t] += 1
            if t == 0x12:
                radar_pls.append(pl)
                continue                              # object comes from the wrapped decoder below
            out.append(Frame(t, pl, rx, decode_payload(t, pl)))
        same = len(radar_objs) == len(radar_pls)      # both framers saw the same 0x12 frames (else payload unknown)
        for i, o in enumerate(radar_objs):
            out.append(Frame(0x12, radar_pls[i] if same else b"", rx, None, o))
        return out


@dataclass
class SourceState:
    last_rx: Optional[float] = None
    last_stm_ts: Optional[int] = None
    value: object = None
    count: int = 0
    timeouts: int = 0                 # fresh->stale transitions seen at the next update
    worst_gap_ms: float = 0.0


class SensorState:
    """Per-source last value / rx time / STM32 timestamp; age and freshness vs timeout (rule 5)."""

    def __init__(self, timeout_ms: Optional[dict] = None):
        self.timeout_ms = dict(TIMEOUT_MS, **(timeout_ms or {}))
        self.src = {s: SourceState() for s in self.timeout_ms}
        self.radar_objs = {}              # object_id -> RadarObject (expires with the radar timeout)

    def update(self, f: Frame):
        """Returns (source, gap_ms_before_this_frame or None, was_stale)."""
        s = self.src[SRC_OF_TYPE[f.typ]]
        gap = None if s.last_rx is None else (f.rx - s.last_rx) * 1000
        stale = gap is not None and gap > self.timeout_ms[SRC_OF_TYPE[f.typ]]
        if stale:
            s.timeouts += 1
        if gap is not None:
            s.worst_gap_ms = max(s.worst_gap_ms, gap)
        s.last_rx, s.count = f.rx, s.count + 1
        if f.typ == 0x12:
            s.last_stm_ts = f.radar.stm32_ms
            s.value = f.radar
            self.radar_objs[f.radar.object_id] = f.radar
        else:
            s.value = f.data
            s.last_stm_ts = (f.data or {}).get("ts", s.last_stm_ts)
        return SRC_OF_TYPE[f.typ], gap, stale

    def age_ms(self, source: str, now: float) -> Optional[float]:
        s = self.src[source]
        return None if s.last_rx is None else max(0.0, (now - s.last_rx) * 1000)

    def is_fresh(self, source: str, now: float) -> bool:
        a = self.age_ms(source, now)
        return a is not None and a <= self.timeout_ms[source]

    def is_valid(self, source: str, now: float) -> bool:
        """Single gate for decision logic: False if never received (None) or older than its timeout. Same as is_fresh;
        kept as a separate name so statistics (poll/timeouts) can change without touching decision code.

        MEANING OF is_valid("radar"): "a 0x12 RADAR_OBJECT frame arrived within RADAR_TIMEOUT_MS" = a TARGET is being
        reported. It is NOT radar health: the radar can be alive with no object in view (then 0x12 simply does not come).
        Use radar_alive() for health and radar_has_target() for target presence."""
        s = self.src.get(source)
        if s is None or s.last_rx is None or s.value is None:
            return False
        return 0.0 <= (now - s.last_rx) * 1000 <= self.timeout_ms[source]

    def sensor_status(self, now: float):
        """SENSOR_STATUS byte of the newest 0x10 if 0x10 is valid (<=100 ms old), else None (STM32 link not trustworthy)."""
        if not self.is_valid("summary", now):
            return None
        return self.src["summary"].value["status"]

    def radar_alive(self, now: float) -> bool:
        """Radar health = RADAR_OK bit (bit0) of 0x10 SENSOR_STATUS, judged by the STM32 (radar frame within 100 ms).
        False if 0x10 itself is stale/never seen. Independent of whether any target exists."""
        st = self.sensor_status(now)
        return st is not None and bool(st & 0x01)

    def radar_has_target(self, now: float) -> bool:
        """A 0x12 object frame arrived within the radar timeout (target present). Alias of is_valid("radar")."""
        return self.is_valid("radar", now)

    def fresh_radar_objects(self, now: float) -> list:
        lim = self.timeout_ms["radar"] / 1000
        return [o for o in self.radar_objs.values() if 0 <= now - o.measured_rx_monotonic <= lim]
