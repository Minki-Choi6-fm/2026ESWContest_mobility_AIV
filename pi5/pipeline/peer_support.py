"""Does a fresh V2V front-car BSM back up "a stopped car is ahead" for a radar target? Pure function, no I/O.

peer_supports(target, peer, own_lat, own_lon, own_heading_deg) -> (bool, reason)
True only if ALL hold: the BSM is a decoded dict of a fresh (caller's freshness verdict) vehicle, GPS_VALID, speed < PEER_SUPPORT_V_MAX_KMH,
the own pose is known, the peer lies AHEAD of us, and its GPS position (converted to forward/lateral metres in our heading frame) agrees with the
radar target within PEER_SUPPORT_POS_TOL_M (both [PROPOSAL], constants.py). The lateral sign of the radar is unverified, so |lateral| is compared.
GPS accuracy of two cars is a few metres each, which is why the tolerance is 10 m.
Used to LIFT the "stationary object -> at most caution" suppression of danger.py; it can only add confirmation, never lower a level.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C

M_PER_DEG_LAT = 111_320.0


def relative_position(own_lat, own_lon, own_heading_deg, lat, lon):
    """(forward_m, lateral_m) of (lat, lon) as seen from the own position/heading (flat-earth approximation, fine for < 1 km)."""
    dn = (lat - own_lat) * M_PER_DEG_LAT
    de = (lon - own_lon) * M_PER_DEG_LAT * math.cos(math.radians(own_lat))
    th = math.radians(own_heading_deg)
    return dn * math.cos(th) + de * math.sin(th), -dn * math.sin(th) + de * math.cos(th)


def peer_supports(target, peer, own_lat, own_lon, own_heading_deg, require_stopped=True):
    """require_stopped=True (default): the front car must be (nearly) stopped = backs up "a stopped car is ahead" (stationary-object cap).
    require_stopped=False: only its POSITION must agree with the radar target, at any speed = backs up "there really is a vehicle at that spot"
    (implausible-headway cap, S8)."""
    if target is None:
        return False, "no target"
    if peer is None:
        return False, "no fresh V2V front car"
    if not peer.get("gps_valid"):
        return False, "front car GPS_VALID=0"
    v = peer.get("speed_kmh")
    if require_stopped and (v is None or v >= C.PEER_SUPPORT_V_MAX_KMH):
        return False, f"front car not stopped (speed {v})"
    if own_lat is None or own_lon is None or own_heading_deg is None:
        return False, "own position/heading unknown"
    f, lat = relative_position(own_lat, own_lon, own_heading_deg, peer["lat"], peer["lon"])
    if f <= 0:
        return False, f"front car is not ahead (forward {f:.1f} m)"
    err = math.hypot(f - target.long_m, abs(lat) - abs(target.lat_m))
    ok = err <= C.PEER_SUPPORT_POS_TOL_M
    return ok, (f"front car {'stopped ' if require_stopped else ''}at forward {f:.1f} m / lateral {abs(lat):.1f} m vs radar {target.long_m:.1f}/{abs(target.lat_m):.1f}: "
                f"error {err:.1f} m {'<=' if ok else '>'} {C.PEER_SUPPORT_POS_TOL_M:g} m")


class _T:
    def __init__(self, long_m, lat_m):
        self.long_m, self.lat_m = long_m, lat_m


def _selftest():
    lat0, lon0 = 37.55, 127.07
    def at(forward, lateral, heading):                               # place a point forward/lateral metres from the own pose
        th = math.radians(heading)
        dn = forward * math.cos(th) - lateral * math.sin(th)
        de = forward * math.sin(th) + lateral * math.cos(th)
        return lat0 + dn / M_PER_DEG_LAT, lon0 + de / (M_PER_DEG_LAT * math.cos(math.radians(lat0)))
    for hd in (0, 90, 215, 359):
        la, lo = at(30.0, 0.5, hd)
        peer = dict(gps_valid=True, speed_kmh=0.0, lat=la, lon=lo)
        assert peer_supports(_T(31.0, 0.0), peer, lat0, lon0, hd)[0], hd                       # 1 m apart
        assert peer_supports(_T(38.0, 1.0), peer, lat0, lon0, hd)[0], hd                       # 8 m error: inside 10 m
        assert not peer_supports(_T(45.0, 0.0), peer, lat0, lon0, hd)[0], hd                   # 15 m error
        assert peer_supports(_T(30.0, -0.5), peer, lat0, lon0, hd)[0], "lateral sign must not matter"
    la, lo = at(30.0, 0.0, 90)
    assert not peer_supports(_T(30.0, 0.0), dict(gps_valid=True, speed_kmh=40.0, lat=la, lon=lo), lat0, lon0, 90)[0], "moving front car does not back a STOPPED car"
    assert peer_supports(_T(30.0, 0.0), dict(gps_valid=True, speed_kmh=40.0, lat=la, lon=lo), lat0, lon0, 90, require_stopped=False)[0], "...but it does back 'a vehicle is there'"
    assert not peer_supports(_T(45.0, 0.0), dict(gps_valid=True, speed_kmh=40.0, lat=la, lon=lo), lat0, lon0, 90, require_stopped=False)[0], "position still has to agree"
    assert not peer_supports(_T(30.0, 0.0), dict(gps_valid=False, speed_kmh=0.0, lat=la, lon=lo), lat0, lon0, 90)[0]
    assert not peer_supports(_T(30.0, 0.0), None, lat0, lon0, 90)[0]
    assert not peer_supports(_T(30.0, 0.0), dict(gps_valid=True, speed_kmh=0.0, lat=la, lon=lo), None, lon0, 90)[0]
    lb, lob = at(-30.0, 0.0, 90)                                      # behind us
    assert not peer_supports(_T(30.0, 0.0), dict(gps_valid=True, speed_kmh=0.0, lat=lb, lon=lob), lat0, lon0, 90)[0]
    print("peer_support selftest OK")


if __name__ == "__main__":
    _selftest()
