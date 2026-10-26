"""S2: the single speed-selection function (rules 4, 12). OBD first, GPS fallback.

Upper logic calls get_speed() only; it must never touch OBD/GPS drivers itself.
Pure: takes the SensorState (S1) and a GPS provider, returns a SpeedResult. No I/O, never blocks.
Does NOT replace ego_speed_tx.py / aiv-ego-speed.service (those stay GPS-only until the team switches).
"""
from dataclasses import dataclass
from typing import Optional, Protocol

from constants import (GPS_MAX_AGE_MS, OBD_AGE_MS_MAX, OBD_FRAME_TIMEOUT_MS, SOURCE_GPS, SOURCE_NONE,
                       SOURCE_OBD, SPEED_MAX_KMH)


class GpsProvider(Protocol):
    def latest(self) -> Optional[tuple]:
        """(speed_kmh, rx_time) of the newest valid fix (mode>=2), or None. Same clock as `now`."""


@dataclass(frozen=True)
class SpeedResult:
    speed_kmh: Optional[float]   # None when no source
    source: int                  # SOURCE_OBD / SOURCE_GPS / SOURCE_NONE
    age_ms: Optional[int]        # total age of the value at `now`
    reason: str                  # why this source (for logs)


def get_speed(state, gps: Optional[GpsProvider], now: float) -> SpeedResult:
    # 1) OBD: frame must have arrived recently AND the value inside must be fresh and valid.
    obd = state.src["obd"]
    why = "obd: no frame"
    if obd.last_rx is not None and obd.value:
        rx_age = (now - obd.last_rx) * 1000
        d = obd.value
        if rx_age > OBD_FRAME_TIMEOUT_MS:
            why = f"obd: frame timeout {rx_age:.0f}ms"
        elif d["speed_kmh"] is None:
            why = "obd: speed=0xFF"
        elif d["age_ms"] is None:
            why = "obd: AGE_MS unknown (0xFFFF)"
        elif d["age_ms"] > OBD_AGE_MS_MAX:
            why = f"obd: AGE_MS {d['age_ms']}>{OBD_AGE_MS_MAX}"
        else:
            return SpeedResult(float(d["speed_kmh"]), SOURCE_OBD, int(d["age_ms"] + max(0.0, rx_age)), "obd ok")
    # 2) GPS fallback
    fix = gps.latest() if gps is not None else None
    if fix is not None:
        spd, t = fix
        age = (now - t) * 1000
        if 0 <= age <= GPS_MAX_AGE_MS and 0 <= spd <= SPEED_MAX_KMH:
            return SpeedResult(float(spd), SOURCE_GPS, int(age), f"gps ({why})")
        why += f"; gps stale/invalid age={age:.0f}ms"
    else:
        why += "; gps: none"
    return SpeedResult(None, SOURCE_NONE, None, why)
