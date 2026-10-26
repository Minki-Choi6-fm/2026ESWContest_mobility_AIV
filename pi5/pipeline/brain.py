"""S4 glue: pipeline frames/state -> target selection + K estimate + danger level. No I/O, no STM32 output (0x20/0x21 are NOT sent).

on_frame(frame, now): feed every decoded Frame (uart_ingest.Frame).  evaluate(...): call at ~20 Hz.
Used by bsm_node.py (simulation) and s4_replay.py (log replay).
"""
import os
import sys
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
from danger import DangerState, front_k, step
from k_estimator import KEstimator
from peer_support import peer_supports
from target_select import Obj, TargetSelector, lane_limits, nearest_any, valid_obj

CYCLE_FLUSH_S = 0.02


@dataclass
class BrainOut:
    level: int
    reason: str
    target: Optional[Obj]
    held: bool
    nearest: Optional[Obj]
    k_r: float
    k_src: str
    k_f: float
    k_f_reason: str
    d_req: Optional[float]
    ttc: Optional[float]
    k_measured: Optional[float]
    nearest_lane: Optional[Obj] = None      # nearest valid object inside the corridor (no confirmation), for comparisons only
    a_req: Optional[float] = None
    pre_level: Optional[int] = None         # level before suppressions
    supp: tuple = ()                        # suppression tags applied this evaluation
    lifted: bool = False                    # stationary-object suppression lifted by camera_confirmed / V2V front car
    peer_reason: str = ""
    side_count: int = 0                     # confirmed objects in the side zone (lane edge < |lat| <= SIDE_LAT_MAX_M), tracked only
    side_nearest: Optional[Obj] = None
    cutin: bool = False


class Brain:
    def __init__(self, k_profile=None, reset_k=False, k_fixed=None):
        """k_profile: path of state/k_profile.json (None = nothing is stored); reset_k: ignore/clear the stored profile; k_fixed: demo value."""
        self.sel, self.kest, self.dstate = TargetSelector(), KEstimator(profile_path=k_profile, reset=reset_k, k_fixed=k_fixed), DangerState()
        if os.environ.get("TARGET_SEL") == "win":              # opt-in: time-window target confirmation (target_select_win.py); default = TargetSelector
            from target_select_win import WindowTargetSelector
            self.sel = WindowTargetSelector()
        self._stm_t = None                       # latest STM32 clock [s]; 0x14 frames carry no timestamp and are placed here
        self._cyc, self._cyc_ts, self._cyc_rx = [], None, 0.0
        self.last_objs, self.last_objs_t = [], -1e9

    def on_frame(self, fr, now):
        if fr.typ == 0x12 and fr.radar is not None:
            r = fr.radar
            if self._cyc_ts is not None and r.stm32_ms != self._cyc_ts:
                self._flush()
            rcs = dyn = None
            if fr.payload and len(fr.payload) == 14:
                b = fr.payload[11]; rcs = (b - 256 if b > 127 else b) / 2.0
                dyn = fr.payload[12]
            self._cyc_ts, self._cyc_rx = r.stm32_ms, now
            self._cyc.append(Obj(r.object_id, r.forward_m, r.left_m, r.relative_mps, rcs, dyn))
        elif fr.typ == 0x13 and fr.data:
            self._stm_t = fr.data["ts"] / 1000.0
            self.kest.add_imu(self._stm_t, fr.data["acc_mg"][0], fr.data["rpy_deg"][1])
        elif fr.typ == 0x10 and fr.data:
            self._stm_t = max(self._stm_t or 0.0, fr.data["ts"] / 1000.0) if (self._stm_t is None or fr.data["ts"] / 1000.0 >= self._stm_t - 1.0) else fr.data["ts"] / 1000.0
        elif fr.typ == 0x14 and fr.data and self._stm_t is not None:
            self.kest.add_speed(self._stm_t, fr.data.get("speed_kmh"))

    def _flush(self):
        if self._cyc:
            self.sel.update(self._cyc, self._cyc_rx)
            self.last_objs, self.last_objs_t = self._cyc, self._cyc_rx
        self._cyc, self._cyc_ts = [], None

    def evaluate(self, state, speed_res, peer, peer_stale, now, own_fix=None, own_heading=None, camera_confirmed=False) -> BrainOut:
        """state: SensorState; speed_res: speed_select.SpeedResult; peer: decoded front BSM dict or None; peer_stale: freshness verdict.
        own_fix: gps_position.GpsFix or None, own_heading: last known course [deg] or None (for peer_supports);
        camera_confirmed: bool from the vision side (None/False when there is no camera)."""
        if self._cyc and now - self._cyc_rx >= CYCLE_FLUSH_S:
            self._flush()
        tr = self.sel.current(now)
        side_ok = self.sel.last_update is not None and now - self.sel.last_update <= 0.3
        side_count, side_near = (self.sel.side_count, self.sel.side_nearest) if side_ok else (0, None)
        st = state.sensor_status(now)
        v_r = None if speed_res.speed_kmh is None else speed_res.speed_kmh / 3.6
        k_r, k_src = self.kest.current()
        k_f, kf_why = front_k(peer, peer_stale)
        # cut-in label: a ground-fixed target is a roadside object, not a car changing lanes
        cutin, cutin_note, tr_reason = tr.cutin, "", tr.reason
        if tr.cutin and tr.target is not None and C.CUTIN_REJECT_GROUND_FIXED:
            if v_r is None:
                cutin_note = "cut-in ground-fixed test NOT applied (own speed unknown)"
            elif abs(tr.target.vrel_ms + v_r) <= C.STATIONARY_VREL_TOL_MPS:
                cutin = False
                tr_reason = tr_reason.replace("cut-in ", "", 1)
                cutin_note = f"cut-in REJECTED: target ground-fixed (|vrel {tr.target.vrel_ms:.1f} + v_ego {v_r:.1f}| = {abs(tr.target.vrel_ms + v_r):.1f} <= {C.STATIONARY_VREL_TOL_MPS:g} m/s)"
        lifted, lift_headway, peer_why = bool(camera_confirmed), bool(camera_confirmed), ""
        lift_why = "camera_confirmed" if camera_confirmed else ""
        if tr.target is not None and not camera_confirmed:
            heading = own_fix.heading_deg if (own_fix is not None and own_fix.heading_deg is not None) else own_heading
            pr = None if (peer is None or peer_stale) else peer
            args = (None if own_fix is None else own_fix.lat, None if own_fix is None else own_fix.lon, heading)
            ok, peer_why = peer_supports(tr.target, pr, *args)                                    # stopped front car at that position
            ok_any, why_any = peer_supports(tr.target, pr, *args, require_stopped=False)         # any front car at that position
            lifted, lift_headway = ok, ok_any
            lift_why = "V2V 앞차(정지)" if ok else ("V2V 앞차(위치 일치)" if ok_any else "")
            if not ok and ok_any:
                peer_why = why_any
        level, why, m = step(self.dstate, now, state.radar_alive(now), st is not None, st is not None and bool(st & 0x10),
                             tr.target, v_r, k_r, k_f, lifted=lifted, lift_headway=lift_headway, lift_why=lift_why)
        fresh = self.last_objs if now - self.last_objs_t <= 0.15 else []
        near = nearest_any(fresh)
        lane = [o for o in fresh if valid_obj(o) and abs(o.lat_m) <= lane_limits()[1]]
        near_lane = min(lane, key=lambda o: o.long_m) if lane else None
        return BrainOut(level, f"{tr_reason}{' [' + cutin_note + ']' if cutin_note else ''} | {why}", tr.target, tr.held, near, k_r, k_src, k_f, kf_why, m["d_req1"], m["ttc"], self.kest.measured(), near_lane,
                        m["a_req"], m["pre"], tuple(m["supp"]), lifted or lift_headway, peer_why, side_count, side_near, cutin)


def _selftest():
    """Brain-level checks of the S6 rules with a stub sensor state: side-lane car = no warning; cut-in promotion; stopped car vs oncoming."""
    import struct
    from types import SimpleNamespace
    import vision_adapter                                    # RadarObject from the fusion module in use
    from speed_select import SpeedResult
    from uart_ingest import Frame

    class StubState:
        def sensor_status(self, now): return 0x0F
        def radar_alive(self, now): return True

    def radar_frame(ts, oid, long_m, lat_m, vrel, rx, dyn=0):
        pl = struct.pack("<IBhhhBBB", ts, oid, int(long_m * 100), int(lat_m * 100), int(vrel * 100), 0, dyn, 0xFF)
        ro = vision_adapter.obj_to_radar_object(Obj(oid, long_m, lat_m, vrel), rx, ts)
        return Frame(0x12, pl, rx, None, ro)

    v50 = 50 / 3.6
    sp = SpeedResult(50.0, 0, 0, "test")
    def run(path, vrel, own_kmh=50.0, seconds=1.2, camera=False, speed=None):
        b, st = Brain(), StubState()
        out, t, ts = None, 0.0, 1000
        for k, (long_m, lat_m) in enumerate(path):
            ts += 60; t += 0.06
            b.on_frame(radar_frame(ts, 7, long_m, lat_m, vrel, t), t)
            out = b.evaluate(st, speed or SpeedResult(own_kmh, 0, 0, "t"), None, True, t + 0.03, camera_confirmed=camera)
        return out, b
    # (3) a fast-closing car in the SIDE zone (|lat| 2.6 m) never produces a warning
    out, b = run([(15.0 - 0.7 * i, 2.6) for i in range(20)], vrel=-12.0)
    assert out.target is None and out.level == 0 and out.side_count == 1 and abs(out.side_nearest.lat_m) == 2.6, (out.level, out.side_count, out.reason)
    # (4) a REAL cut-in (S7): 1.7 s in the side zone with |lat| falling ~1.2 m/s, then into the corridor while closing at 12 m/s:
    #     promoted as cut-in, then judged like any lead car (danger when the time to collision gets short)
    path = [(45.0 - 0.72 * i, 3.4 - 0.07 * i) for i in range(40)]
    out, b = run(path, vrel=-12.0)
    assert out.target is not None and out.cutin and "cut-in" in out.reason and out.level == 2, (out.level, out.reason)
    # the same approach but hovering around the lane edge (1.6 <-> 1.9): ordinary candidate, no cut-in label
    path = [(45.0 - 0.72 * i, 1.6 if i % 2 == 0 else 1.9) for i in range(40)]
    out, b = run(path, vrel=-12.0)
    assert out.target is not None and not out.cutin, (out.cutin, out.reason)
    # stopped car ahead (c == v_r): not "oncoming/passing"; it is ground-fixed so it is limited to caution unless lifted, and never excluded
    out, b = run([(30.0 - 0.8 * i, 0.3) for i in range(20)], vrel=-v50)
    assert out.target is not None and out.level in (1, 2) and not any("마주오는" in x for x in out.supp), (out.level, out.supp)
    # oncoming (c = v_r + 8 m/s): at most caution, with the reason
    out, b = run([(40.0 - 1.2 * i, 0.3) for i in range(20)], vrel=-(v50 + 8.0))
    assert out.target is not None and out.level == 1 and any("마주오는" in x for x in out.supp), (out.level, out.supp)
    # (S8) cut-in vs ground-fixed: the same real approach, but the target is fixed to the ground (vrel = -v_ego) -> NOT a cut-in; unknown own speed -> test not applied
    path = [(45.0 - 0.72 * i, 3.4 - 0.07 * i) for i in range(40)]
    out, b = run(path, vrel=-12.0)                                                       # moving over ground (|-12 + 13.9| = 1.9 > 1.5): stays a cut-in
    assert out.cutin
    out, b = run(path, vrel=-50 / 3.6)                                                   # ground-fixed
    assert out.target is not None and not out.cutin and "cut-in REJECTED" in out.reason and "cut-in " not in out.reason.split(" [")[0], out.reason
    out, b = run(path, vrel=-12.0, speed=SpeedResult(None, 255, None, "none"))           # own speed unknown: not applied, flag kept, reason says so
    assert out.cutin and "NOT applied (own speed unknown)" in out.reason, out.reason
    # (S8) implausible headway: 1.8 m ahead at 55 km/h closing 3 m/s -> caution without camera, danger when the camera confirms the target
    close = [(1.8, 0.3)] * 20
    out, b = run(close, vrel=-3.0, own_kmh=55.0)
    assert out.target is not None and out.level == 1 and any("implausible-headway" in x for x in out.supp), (out.level, out.supp)
    out, b = run(close, vrel=-3.0, own_kmh=55.0, camera=True)
    assert out.level == 2 and any("해제" in x for x in out.supp) and out.lifted, (out.level, out.supp)
    # (S19) WindowTargetSelector (TARGET_SEL=win): the STM32 stamps every 0x12 frame separately, so one radar cycle arrives as 3 ts groups and id 7 is in only one of them.
    #       Old 3-of-5-per-ts-group confirmation never confirms it; the 0.4 s window does. Ground-fixed (stationary) targets stay capped to caution.
    from target_select_win import WindowTargetSelector
    def run_frag(win, vrel, own_kmh=50.0, cycles=14):
        b, st = Brain(), StubState()
        b.sel = WindowTargetSelector() if win else TargetSelector()            # explicit, whatever TARGET_SEL says
        out, t, ts = None, 0.0, 1000
        for k in range(cycles):
            for j in range(3):
                ts += 20; t += 0.02
                if j == 0:
                    b.on_frame(radar_frame(ts, 7, 30.0 - 0.8 * k, 0.3, vrel, t), t)
                else:
                    b.on_frame(radar_frame(ts, 90 + j, 70.0, 8.0, 0.0, t), t)      # decoys outside corridor and side zone: they only split the cycle
            out = b.evaluate(st, SpeedResult(own_kmh, 0, 0, "t"), None, True, t + 0.01)
        return out
    out = run_frag(False, vrel=-v50 + 6.0)
    assert out.target is None, "old selector must NOT confirm an id that is in 1 of 3 ts fragments (documents the S19 problem)"
    out = run_frag(True, vrel=-v50 + 6.0)
    assert out.target is not None and out.target.id == 7 and not out.held, (out.reason,)
    out = run_frag(True, vrel=-v50)                                                      # stopped car: still ground-fixed -> caution at most, with the reason
    assert out.target is not None and out.level <= 1 and any("정지 물체" in x for x in out.supp), (out.level, out.supp, out.reason)
    print("brain selftest OK (side-lane car = no warning, cut-in promotion, stopped car vs oncoming, camera lifts the headway cap, window selector on fragmented ts)")


if __name__ == "__main__":
    _selftest()
