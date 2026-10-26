"""S4 target selection: which radar object is the lead vehicle. Pure logic, no I/O.

Input per radar CYCLE (objects that share the same STM32 timestamp): a list of Obj. Output: TargetSelector.current(now) -> TargetResult.
All numbers live in constants.py (TGT_*) and are [PROPOSAL]. Rules (defaults = the numbers of the vision package's lead selection):
  * ahead distance TGT_MIN_LONG_M <= long <= TGT_MAX_LONG_M (1..100 m), |vrel| <= TGT_VREL_VALID_MAX_MPS (60 m/s; logs also contain exactly +-128 / 127.75 = invalid)
  * lane corridor by |lat| <= TGT_LANE_LAT_M (1.75 m). OPTION TGT_USE_HYSTERESIS: enter <= TGT_ENTER_LAT_M (1.5), leave > TGT_EXIT_LAT_M (2.0).
    The radar's left/right SIGN has not been verified in the car, so only the absolute value is used.
  * confirmation: seen in the corridor in >= TGT_CONFIRM_HITS of the last TGT_WINDOW_CYCLES cycles
  * pick the nearest confirmed candidate; keep the current radar id unless another is nearer by more than TGT_SWITCH_MARGIN_M
  * hold: after the target disappears it is kept for TGT_HOLD_S (its values are the last seen ones, flagged held)
Side-lane monitoring (S6): objects with lane edge < |lat| <= SIDE_LAT_MAX_M (3.5 m) are TRACKED with the same 3-of-5 confirmation and only counted
(side_count) / the nearest one reported; they are never a warning target. An object that was seen in the side zone within SIDE_MEMORY_S and then
becomes the lead target (enters the corridor and passes the normal confirmation) is flagged "cut-in" in TargetResult.reason / .cutin -- but ONLY (S7) if it
was seen CONTINUOUSLY in the side zone for >= SIDE_CUTIN_MIN_S (1.0 s; a gap > SIDE_GAP_S restarts the track) while |lat| DECREASED (least-squares slope
< SIDE_CUTIN_SLOPE_MPS) before it entered the corridor. Objects flickering around the lane edge, or driving parallel, are ordinary candidates, not cut-ins.
No left/right labelling (radar lat sign unverified).
Sign convention (verified on logs, see PROJECT_STATE): vrel < 0 = approaching (stationary objects show vrel ~ -ego speed).
"""
import math
import os
import sys
from collections import deque
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C            # attributes are read at call time so tests / replay variants can override them

CYCLE_TOLERANCE_S = 0.12      # a target last seen longer ago than this is reported as "held" (radar cycle is ~60 ms)


@dataclass(frozen=True)
class Obj:
    id: int
    long_m: float
    lat_m: float
    vrel_ms: float
    rcs: Optional[float] = None
    dyn: Optional[int] = None      # radar DYN_PROP (see constants.STATIONARY_DYN for what is trusted)


@dataclass(frozen=True)
class TargetResult:
    target: Optional[Obj]
    reason: str
    held: bool = False
    cutin: bool = False


def valid_obj(o: Obj) -> bool:
    return (all(math.isfinite(x) for x in (o.long_m, o.lat_m, o.vrel_ms)) and C.TGT_MIN_LONG_M <= o.long_m <= C.TGT_MAX_LONG_M
            and abs(o.vrel_ms) <= C.TGT_VREL_VALID_MAX_MPS)


def lane_limits():
    """(enter, exit) |lat| limits of the corridor."""
    return (C.TGT_ENTER_LAT_M, C.TGT_EXIT_LAT_M) if C.TGT_USE_HYSTERESIS else (C.TGT_LANE_LAT_M, C.TGT_LANE_LAT_M)


def nearest_any(objs) -> Optional[Obj]:
    """Reference for comparisons: nearest valid object ahead, ignoring the corridor."""
    v = [o for o in objs if valid_obj(o)]
    return min(v, key=lambda o: o.long_m) if v else None


class TargetSelector:
    def __init__(self):
        self.hist = {}            # radar id -> deque of 0/1 (in corridor this cycle)
        self.in_lane = {}         # radar id -> corridor state (hysteresis)
        self.cur: Optional[Obj] = None
        self.cur_seen = None      # time the current target was last seen
        self.switches = 0
        self.side_hist = {}       # radar id -> deque of 0/1 (in the side zone this cycle)
        self.side_track = {}      # radar id -> dict(t0, last, pts=[(t, |lat|)]) of the CURRENT continuous side-zone observation
        self.cutin_ready = {}     # radar id -> time it left the side zone into the corridor after a qualifying (>=1 s, |lat| falling) approach
        self.side_count = 0       # confirmed side-zone objects in the latest cycle
        self.side_nearest: Optional[Obj] = None
        self.cur_cutin = False
        self.last_update = None

    def update(self, objs, now: float) -> None:
        """Process ONE radar cycle (call with the list of objects of that cycle, possibly empty)."""
        self.last_update = now
        present, side = {}, {}
        for o in objs:
            if not valid_obj(o):
                continue
            was = self.in_lane.get(o.id, False)
            enter, leave = lane_limits()
            inl = abs(o.lat_m) <= (leave if was else enter)
            self.in_lane[o.id] = inl
            if inl:
                present[o.id] = o
            elif abs(o.lat_m) <= C.SIDE_LAT_MAX_M:
                side[o.id] = o
        for i in set(self.hist) | set(present):
            h = self.hist.setdefault(i, deque(maxlen=C.TGT_WINDOW_CYCLES))
            h.append(1 if i in present else 0)
        for i in [i for i, h in self.hist.items() if not any(h) and i not in present]:
            del self.hist[i]; self.in_lane.pop(i, None)
        for i in set(self.side_hist) | set(side):
            self.side_hist.setdefault(i, deque(maxlen=C.TGT_WINDOW_CYCLES)).append(1 if i in side else 0)
        for i in [i for i, h in self.side_hist.items() if not any(h) and i not in side]:
            del self.side_hist[i]
        conf_side = [o for i, o in side.items() if sum(self.side_hist[i]) >= C.TGT_CONFIRM_HITS]
        self.side_count = len(conf_side)
        self.side_nearest = min(conf_side, key=lambda o: o.long_m) if conf_side else None
        for i, o in side.items():                                   # continuous side-zone tracks (|lat| over time)
            tr = self.side_track.get(i)
            if tr is None or now - tr["last"] > C.SIDE_GAP_S:
                tr = self.side_track[i] = dict(t0=now, last=now, pts=[])
            tr["last"] = now
            tr["pts"].append((now, abs(o.lat_m)))
        for i in list(self.side_track):
            if i in side:
                continue
            tr = self.side_track[i]
            if i in present and self._qualifies(tr):                # left the side zone INTO the corridor after a real approach
                self.cutin_ready[i] = now
                del self.side_track[i]
            elif now - tr["last"] > C.SIDE_GAP_S:
                del self.side_track[i]
        for i in [i for i, t in self.cutin_ready.items() if now - t > 4 * C.SIDE_MEMORY_S]:
            del self.cutin_ready[i]
        cand = [o for i, o in present.items() if sum(self.hist[i]) >= C.TGT_CONFIRM_HITS]
        if not cand:
            return
        best = min(cand, key=lambda o: o.long_m)
        cur_now = present.get(self.cur.id) if self.cur is not None else None
        if cur_now is not None and cur_now in cand and not (best.long_m < cur_now.long_m - C.TGT_SWITCH_MARGIN_M):
            best = cur_now                                   # keep the same radar id (anti-flicker)
        if self.cur is None or best.id != self.cur.id:
            self.cur_cutin = best.id in self.cutin_ready and (now - self.cutin_ready[best.id]) <= C.SIDE_MEMORY_S
        if self.cur is not None and best.id != self.cur.id:
            self.switches += 1
        self.cur, self.cur_seen = best, now

    @staticmethod
    def _qualifies(tr) -> bool:
        """>= SIDE_CUTIN_MIN_S of continuous side-zone observation and a falling |lat| (least-squares slope below SIDE_CUTIN_SLOPE_MPS)."""
        pts = tr["pts"]
        if len(pts) < 4 or tr["last"] - tr["t0"] < C.SIDE_CUTIN_MIN_S:
            return False
        n = len(pts)
        mt = sum(p[0] for p in pts) / n; ml = sum(p[1] for p in pts) / n
        var = sum((p[0] - mt) ** 2 for p in pts)
        slope = sum((p[0] - mt) * (p[1] - ml) for p in pts) / var if var > 0 else 0.0
        return slope < C.SIDE_CUTIN_SLOPE_MPS

    def current(self, now: float) -> TargetResult:
        if self.cur is None:
            return TargetResult(None, f"no confirmed in-corridor object ({len(self.hist)} tracked, side {self.side_count})")
        age = now - self.cur_seen
        if age > C.TGT_HOLD_S:
            self.cur, self.cur_cutin = None, False
            return TargetResult(None, f"target lost (hold {C.TGT_HOLD_S}s expired)")
        o = self.cur
        base = f"id{o.id} long={o.long_m:.1f} lat={o.lat_m:.1f} vrel={o.vrel_ms:.1f} hits={sum(self.hist.get(o.id, []))}/{C.TGT_WINDOW_CYCLES}"
        if self.cur_cutin:
            base = "cut-in " + base
        if age > CYCLE_TOLERANCE_S:
            return TargetResult(o, f"HOLD {age:.2f}s {base}", True, self.cur_cutin)
        return TargetResult(o, base, False, self.cur_cutin)


def _selftest():
    s = TargetSelector(); t = 0.0
    def cyc(objs):
        nonlocal t
        t += 0.06; s.update(objs, t)
    car = lambda lat=0.5, long=20.0, i=7, v=-5.0: Obj(i, long, lat, v)
    cyc([car()]); cyc([car()]); assert s.current(t).target is None, "2 hits must not confirm"
    cyc([car()]); r = s.current(t); assert r.target and r.target.id == 7 and not r.held, r
    # hold 0.3 s
    t0 = t
    for _ in range(3): cyc([])
    r = s.current(t); assert r.target and r.held and "HOLD" in r.reason, r
    r = s.current(t0 + 0.25); assert r.target is not None
    r = s.current(t0 + 0.35); assert r.target is None, r
    # default corridor: |lat| <= 1.75, no hysteresis
    s = TargetSelector(); t = 0.0
    for _ in range(5): cyc([car(lat=1.9)])
    assert s.current(t).target is None, "|lat| 1.9 must not enter"
    for _ in range(3): cyc([car(lat=1.7)])
    assert s.current(t).target is not None, "1.7 is inside 1.75"
    # option: hysteresis enter <=1.5, stay until >2.0
    C.TGT_USE_HYSTERESIS = True
    try:
        s = TargetSelector(); t = 0.0
        for _ in range(5): cyc([car(lat=1.8)])
        assert s.current(t).target is None, "|lat| 1.8 must not enter (hysteresis option)"
        for _ in range(3): cyc([car(lat=1.4)])
        assert s.current(t).target is not None
        for _ in range(3): cyc([car(lat=1.9)])
        assert s.current(t).target is not None and not s.current(t).held, "1.9 stays inside"
        for _ in range(6): cyc([car(lat=2.1)])
        assert s.current(t + 0.4).target is None, "2.1 leaves"
    finally:
        C.TGT_USE_HYSTERESIS = False
    # |lat| is used: negative lat works the same
    s = TargetSelector(); t = 0.0
    for _ in range(3): cyc([car(lat=-1.0)])
    assert s.current(t).target is not None
    # nearest + id stickiness + switch margin
    s = TargetSelector(); t = 0.0
    for _ in range(4): cyc([car(i=1, long=30), car(i=2, long=28)])
    assert s.current(t).target.id == 2, "nearest wins when nothing is current yet"
    s = TargetSelector(); t = 0.0
    for _ in range(4): cyc([car(i=1, long=30)])
    for _ in range(4): cyc([car(i=1, long=30), car(i=2, long=29)])
    assert s.current(t).target.id == 1, "within margin: keep id 1"
    for _ in range(4): cyc([car(i=1, long=30), car(i=2, long=25)])
    assert s.current(t).target.id == 2, "nearer than margin: switch"
    # side zone: tracked and counted, never a target
    s = TargetSelector(); t = 0.0
    for _ in range(6): cyc([car(i=31, lat=2.6, long=25.0, v=-10.0)])
    assert s.current(t).target is None, "a side-lane car is never a warning target"
    assert s.side_count == 1 and s.side_nearest.id == 31 and abs(s.side_nearest.lat_m) == 2.6
    for _ in range(2): cyc([car(i=31, lat=3.9, long=25.0)])                      # outside 3.5 m: not tracked
    assert s.side_count == 0
    s = TargetSelector(); t = 0.0
    for _ in range(4): cyc([car(i=32, lat=-2.4, long=20.0)])                      # negative lat is the same
    assert s.side_count == 1 and s.current(t).target is None
    # (S7) a REAL cut-in: 1.9 s in the side zone with |lat| falling ~1 m/s (3.4 -> 1.8), then into the corridor
    s = TargetSelector(); t = 0.0
    lat = lambda k: 3.4 - 0.06 * k
    k = 0
    while lat(k) > 1.75:
        cyc([car(i=40, lat=lat(k), long=25.0)]); k += 1
    assert s.current(t).target is None and s.side_count == 1 and k >= 25
    for _ in range(2): cyc([car(i=40, lat=lat(k), long=25.0)]); k += 1
    assert s.current(t).target is None, "only 2 corridor cycles so far"
    cyc([car(i=40, lat=lat(k), long=25.0)])
    r = s.current(t); assert r.target is not None and r.target.id == 40 and r.cutin and "cut-in" in r.reason, r
    # (S7) boundary flicker 1.6 <-> 1.9 for 3 s: becomes an ORDINARY candidate, never a cut-in
    s = TargetSelector(); t = 0.0
    for k in range(50): cyc([car(i=41, lat=1.6 if k % 2 == 0 else 1.9, long=25.0)])
    r = s.current(t); assert r.target is not None and r.target.id == 41 and not r.cutin and "cut-in" not in r.reason, r
    # (S7) parallel driving in the side zone (|lat| 2.6 +- noise, 3 s) never becomes a target or a cut-in; if it then moves in abruptly it is NOT a cut-in (no trend)
    s = TargetSelector(); t = 0.0
    import random as _r
    rr = _r.Random(5)
    for k in range(50): cyc([car(i=42, lat=2.6 + rr.uniform(-0.06, 0.06), long=25.0)])
    assert s.current(t).target is None and s.side_count == 1
    for _ in range(4): cyc([car(i=42, lat=1.0, long=25.0)])
    r = s.current(t); assert r.target is not None and not r.cutin, r
    # (S7) less than 1.0 s in the side zone, even with a falling trend: not a cut-in
    s = TargetSelector(); t = 0.0
    for k in range(10): cyc([car(i=43, lat=2.6 - 0.07 * k, long=25.0)])
    for _ in range(3): cyc([car(i=43, lat=1.4, long=25.0)])
    r = s.current(t); assert r.target is not None and not r.cutin, r
    # (S7) a gap of > 0.3 s restarts the track: 0.7 s + gap + 0.7 s is not "continuous 1.0 s"
    s = TargetSelector(); t = 0.0
    for k in range(12): cyc([car(i=44, lat=3.2 - 0.07 * k, long=25.0)])
    t += 0.5
    for k in range(12): cyc([car(i=44, lat=2.4 - 0.07 * k, long=25.0)])
    for _ in range(3): cyc([car(i=44, lat=1.0, long=25.0)])
    r = s.current(t); assert r.target is not None and not r.cutin, r
    # an object that was never in the side zone: no cut-in flag
    s = TargetSelector(); t = 0.0
    for _ in range(3): cyc([car(i=33, lat=0.4, long=20.0)])
    r = s.current(t); assert r.target is not None and not r.cutin and "cut-in" not in r.reason
    # invalid objects ignored
    s = TargetSelector(); t = 0.0
    for _ in range(4): cyc([Obj(9, 20, 0, -128.0), Obj(10, 0.5, 0, -3), Obj(11, 20, 0, float("nan")), Obj(12, 130, 0, -3), Obj(13, 20, 0, -61.0)])
    assert s.current(t).target is None
    print("target_select selftest OK")


if __name__ == "__main__":
    _selftest()
