"""PROPOSAL (NOT wired into brain.py): time-window confirmation for the lead-target selection.

Problem (road_1026_0141, see analysis): the STM32 stamps every 0x12 frame with its own HAL_GetTick(), so one ~60 ms radar cycle arrives as 2-5 groups with
different ts. TargetSelector counts one "cycle" per ts group, so an id that is only in one of the fragments gets 1 hit in ~3 groups and 3-of-5 often fails.

Change: confirmation = the id was seen in the corridor (side zone: side zone) in >= WIN_HITS DISTINCT radar updates inside the last WIN_S seconds of RECEIVE time.
Everything else (corridor limits, validity, nearest + id stickiness + switch margin, TGT_HOLD_S hold, side-zone tracking, cut-in rule) is inherited unchanged
from target_select.TargetSelector. Only update() and the "hits" text of current() differ. The constants TGT_WINDOW_CYCLES / TGT_CONFIRM_HITS are NOT used here.
WIN_S / WIN_HITS / MIN_SIGHT_GAP_S are [PROPOSAL] values and live in this file only (constants.py untouched).
usage: python3 -I target_select_win.py   (selftest)
"""
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
from target_select import Obj, TargetResult, TargetSelector, valid_obj, lane_limits

WIN_S = 0.4               # [PROPOSAL] look-back window on receive time
WIN_HITS = 3              # [PROPOSAL] sightings needed inside the window
MIN_SIGHT_GAP_S = 0.02    # [PROPOSAL] two sightings of one id closer than this are the same radar cycle (counted once)
FRESH_S = 0.15            # [PROPOSAL] a confirmed id is a candidate only if its latest corridor sighting is at most this old (~2 radar cycles)


class WindowTargetSelector(TargetSelector):
    def __init__(self, win_s=None, hits=None):
        super().__init__()
        self.win_s, self.win_hits = win_s or WIN_S, hits or WIN_HITS
        self.sight = {}           # radar id -> deque of receive times (corridor)
        self.side_sight = {}      # radar id -> deque of receive times (side zone)
        self.last_seen = {}       # radar id -> (latest Obj in the corridor, receive time): candidates are NOT limited to the current ts fragment

    def _mark(self, table, ids, now):
        for i in ids:
            d = table.setdefault(i, deque())
            if not d or now - d[-1] >= MIN_SIGHT_GAP_S:
                d.append(now)
        for i in list(table):
            d = table[i]
            while d and now - d[0] > self.win_s:
                d.popleft()
            if not d:
                del table[i]

    def update(self, objs, now: float) -> None:
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
        self._mark(self.sight, present, now)
        for i, o in present.items():
            self.last_seen[i] = (o, now)
        for i in [i for i in self.last_seen if i not in self.sight]:
            del self.last_seen[i]
        self._mark(self.side_sight, side, now)
        for i in [i for i in self.in_lane if i not in self.sight and i not in present]:
            self.in_lane.pop(i, None)
        self.hist = {i: [1] * len(d) for i, d in self.sight.items()}   # base class current() only DISPLAYS sum(hist)/len(hist): sightings in the window, not timestamps
        conf_side = [o for i, o in side.items() if len(self.side_sight.get(i, ())) >= self.win_hits]
        self.side_count = len(conf_side)
        self.side_nearest = min(conf_side, key=lambda o: o.long_m) if conf_side else None
        for i, o in side.items():                                   # continuous side-zone tracks (identical to the base class)
            tr = self.side_track.get(i)
            if tr is None or now - tr["last"] > C.SIDE_GAP_S:
                tr = self.side_track[i] = dict(t0=now, last=now, pts=[])
            tr["last"] = now
            tr["pts"].append((now, abs(o.lat_m)))
        for i in list(self.side_track):
            if i in side:
                continue
            tr = self.side_track[i]
            if i in present and self._qualifies(tr):
                self.cutin_ready[i] = now
                del self.side_track[i]
            elif now - tr["last"] > C.SIDE_GAP_S:
                del self.side_track[i]
        for i in [i for i, t in self.cutin_ready.items() if now - t > 4 * C.SIDE_MEMORY_S]:
            del self.cutin_ready[i]
        cand = [o for i, (o, ts) in self.last_seen.items() if len(self.sight.get(i, ())) >= self.win_hits and now - ts <= FRESH_S]
        if not cand:
            return
        best = min(cand, key=lambda o: o.long_m)
        cur_now = next((o for o in cand if self.cur is not None and o.id == self.cur.id), None)
        if cur_now is not None and not (best.long_m < cur_now.long_m - C.TGT_SWITCH_MARGIN_M):
            best = cur_now
        if self.cur is None or best.id != self.cur.id:
            self.cur_cutin = best.id in self.cutin_ready and (now - self.cutin_ready[best.id]) <= C.SIDE_MEMORY_S
        if self.cur is not None and best.id != self.cur.id:
            self.switches += 1
        self.cur, self.cur_seen = best, now

    def current(self, now: float) -> TargetResult:
        r = super().current(now)
        if r.target is None:
            return r
        n = len(self.sight.get(r.target.id, ()))
        import re
        return TargetResult(r.target, re.sub(r"hits=[\d.]+/[\d.]+", f"hits={n}/{self.win_s:g}s", r.reason), r.held, r.cutin)


def _selftest():
    s = WindowTargetSelector(); t = 0.0
    car = lambda i=7, lat=0.5, long=20.0: Obj(i, long, lat, -5.0)
    def cyc(objs, dt=0.02):
        nonlocal t
        t += dt; s.update(objs, t)
    # fragmented cycle: id 7 only in 1 of 3 fragments, 60 ms cycle -> 3 sightings within 0.4 s must confirm
    for _ in range(3):
        cyc([car()], 0.02); cyc([car(i=1, long=30)], 0.02); cyc([car(i=2, long=40)], 0.02)
    r = s.current(t); assert r.target is not None and r.target.id in (1, 2, 7), r
    s = WindowTargetSelector(); t = 0.0
    cyc([car()], 0.06); cyc([car()], 0.06); assert s.current(t).target is None, "2 sightings must not confirm"
    cyc([car()], 0.06); assert s.current(t).target.id == 7
    s = WindowTargetSelector(); t = 0.0                       # sightings spread over more than the window never confirm
    for _ in range(3): cyc([car()], 0.25)
    assert s.current(t).target is None, "3 sightings in 0.5 s > 0.4 s window"
    s = WindowTargetSelector(); t = 0.0                       # same cycle in two fragments (<20 ms) counts once
    for _ in range(2): cyc([car()], 0.06); cyc([car()], 0.005)
    assert s.current(t).target is None
    s = WindowTargetSelector(); t = 0.0                       # hold 0.3 s then lost
    for _ in range(3): cyc([car()], 0.06)
    t0 = t; assert s.current(t0 + 0.25).target is not None and s.current(t0 + 0.35).target is None
    s = WindowTargetSelector(); t = 0.0                       # outside corridor / side zone never a target, side counted
    for _ in range(5): cyc([car(i=31, lat=2.6)], 0.06)
    assert s.current(t).target is None and s.side_count == 1
    s = WindowTargetSelector(); t = 0.0                       # invalid objects ignored
    for _ in range(4): cyc([Obj(9, 20, 0, -128.0), Obj(10, 0.5, 0, -3), Obj(12, 130, 0, -3)], 0.06)
    assert s.current(t).target is None
    print("target_select_win selftest OK")


if __name__ == "__main__":
    _selftest()
