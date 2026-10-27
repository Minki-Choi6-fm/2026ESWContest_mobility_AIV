"""Danger decision (WARNING ONLY -- no braking intervention). Pure functions + a tiny hysteresis state, written so it can be moved to STM32 C
later (the core formulas use plain numbers only).

Own design, informed by the general idea of forward-collision-warning systems (reaction time + braking distance, time-to-collision, required
deceleration). It is not an implementation of any standard; no standard number or threshold is claimed. All numbers are [PROPOSAL] (constants.py).

  d      target distance [m]              c   = max(0, -vrel)  closing speed [m/s]  (vrel < 0 = approaching; sign verified on logs)
  v_r    own speed [m/s] or None          v_f = max(0, v_r - c)  lead-car speed estimate
  K_r    own effective braking coefficient (driver's usual braking strength, k_estimator; K*g = deceleration assumed by the formulas)
  closing = c > MIN_CLOSING_MPS (0.5 m/s); a_req and TTC exist only while closing
  a_req  = c^2 / (2*max(d - d0, 0.1))          required constant deceleration to stop the closing in the remaining gap
  TTC    = d / c
  d_req(t) = v_r*t + v_r^2/(2*K_r*g) - v_f^2/(2*K_f*g) + d0   (floored at d0)   [d_req(1.5 s) = 37.6 m for 60 km/h behind a 60 km/h car, K_r .5, K_f .8, d0 2]
  level 2 (danger):  closing and (a_req >= 0.7*K_r*g or TTC < 2.0 s)
  level 1 (caution): closing and (a_req >= 0.4*K_r*g or TTC < 3.0 s)   -- or, even when not closing, d < d_req(1.0 s) (this one only up to caution)
  level 0 (safe)     otherwise.      level 0xFF = UNKNOWN (never reported as 0)
Own speed unknown -> the d_req term is dropped (a_req / TTC need only the radar). Unknown (0xFF): radar not alive, 0x10 invalid, STM32 SAFE_MODE.
"Radar alive but no target" is 0 (safe).
Suppressions (each has a flag in constants.py, every use is written into the reason string and into info["supp"]):
  * implausible headway (S7): own speed >= 10 km/h and d / v_r < 0.3 s (e.g. 1.8 m at 55 km/h) -> near-field clutter/ghost -> at most caution
    (NOT excluded). Own speed unknown / < 10 km/h -> not applied. S8: LIFTED (no cap) when the target is backed by the camera (camera_confirmed) or by a
    fresh V2V front car whose GPS position agrees with the target (`lift_headway`); the reason says "억제 해제". A real extreme-tailgating case that
    the camera/V2V confirms therefore stays a danger.
  * own speed < 2 km/h                         -> at most caution
  * target fixed to the ground (|vrel+v_r| <= 1.5 or dyn in STATIONARY_DYN) -> at most caution, UNLESS lifted (camera_confirmed or a fresh V2V front car backs
    a stopped car ahead, see peer_support.py)
  * oncoming / passing: own speed known and closing speed c > v_r + 3 m/s (the target moves TOWARD us; a stopped or slower car cannot close faster
    than we drive) -> at most caution, NOT excluded, reason says so. Own speed unknown -> not applied. The radar dyn flag is not used for this.
Hysteresis: up after RISE_CONFIRM_N consecutive evaluations; down only when the situation is safe even with thresholds x FALL_MARGIN for FALL_HOLD_S.
"""
import os
import sys
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C

DANGER_UNKNOWN = C.DANGER_UNKNOWN


def _env_float(name):
    v = os.environ.get(name, "").strip()
    try:
        return float(v) if v else None
    except ValueError:
        return None


# OPTIONAL lateral caps for danger (2), read once from the environment. All unset (default) = nothing changes. They only lower 2 -> 1, never touch level 1 or the corridor.
LAT2_MAX_M = _env_float("LAT2_MAX_M")      # danger only if |lat| <= this (e.g. 1.2); caution still uses the corridor limit of target_select (1.75 m)
PASS_D_M = _env_float("PASS_D_M")          # with PASS_LAT_M: d < PASS_D_M and |lat| >= PASS_LAT_M = car passing alongside -> at most caution
PASS_LAT_M = _env_float("PASS_LAT_M")


def d_req(v_r, v_f, k_r, k_f, t, d0=None, g=None):
    d0 = C.D0_M if d0 is None else d0
    g = C.G_MPS2 if g is None else g
    r = v_r * t + v_r * v_r / (2.0 * k_r * g) - v_f * v_f / (2.0 * k_f * g) + d0
    return r if r > d0 else d0


def closing_speed(vrel_ms):
    return -vrel_ms if vrel_ms < 0 else 0.0


def raw_level(d, vrel_ms, v_r, k_r, k_f, scale=1.0):
    """Instantaneous level BEFORE suppressions and hysteresis. scale multiplies every threshold (hysteresis margin).
    Returns (level 0/1/2, reason, metrics dict with a_req, ttc, d_req1)."""
    c = closing_speed(vrel_ms)
    closing = c > C.MIN_CLOSING_MPS
    ka = k_r * C.G_MPS2
    a_req = c * c / (2.0 * max(d - C.D0_M, C.D_FLOOR_M)) if closing else None
    ttc = d / c if closing else None
    d_req1 = None if v_r is None else d_req(v_r, max(0.0, v_r - c), k_r, k_f, C.T_FOLLOW_CAUTION_S) * scale
    level = 0
    if closing:
        if a_req >= C.A_REQ_DANGER_FRAC * ka / scale or ttc < C.TTC_DANGER_S * scale:
            level = 2
        elif a_req >= C.A_REQ_CAUTION_FRAC * ka / scale or ttc < C.TTC_CAUTION_S * scale:
            level = 1
    if level < 1 and d_req1 is not None and d < d_req1:
        level = 1
    why = [f"d={d:.1f}", f"c={c:.1f}", "a_req=" + ("-" if a_req is None else f"{a_req:.2f}") + f"/{ka:.2f}", "TTC=" + ("-" if ttc is None else f"{ttc:.1f}s")]
    why.append("d_req1.0=" + ("-(속도 없음→d_req 제외)" if d_req1 is None else f"{d_req1:.1f}"))
    return level, " ".join(why), dict(a_req=a_req, ttc=ttc, d_req1=d_req1)


def _suppress(level, v_r, target, lifted, lift_headway=None, lift_why=""):
    """Returns (level, tags). Only danger (2) can be lowered, and only to caution (1).
    lifted: camera_confirmed or a V2V front car that is STOPPED and agrees in position (backs a stopped car); lift_headway: camera_confirmed or a V2V front
    car at any speed that agrees in position (backs "a vehicle is really there"); defaults to `lifted`."""
    tags = []
    if level < 2:
        return level, tags
    lift_headway = lifted if lift_headway is None else lift_headway
    if C.SUPPRESS_IMPLAUSIBLE_HEADWAY and v_r is not None and v_r * 3.6 >= C.IMPLAUSIBLE_MIN_KMH and target.long_m / v_r < C.IMPLAUSIBLE_HEADWAY_S:
        if lift_headway:
            tags.append(f"억제 해제:implausible-headway(d {target.long_m:.1f} m / v {v_r:.1f} m/s = {target.long_m / v_r:.2f} s)이지만 {lift_why or '카메라/V2V'}가 뒷받침")
        else:
            level = 1; tags.append(f"억제:implausible-headway(d {target.long_m:.1f} m / v {v_r:.1f} m/s = {target.long_m / v_r:.2f} s < {C.IMPLAUSIBLE_HEADWAY_S:g} s)→최대 주의")
    if level == 2 and C.SUPPRESS_STANDSTILL and v_r is not None and v_r * 3.6 < C.STANDSTILL_KMH:
        level = 1; tags.append(f"억제:정차/서행(<{C.STANDSTILL_KMH:g}km/h)→최대 주의")
    if level == 2 and C.SUPPRESS_STATIONARY_OBJECT:
        ground = v_r is not None and abs(target.vrel_ms + v_r) <= C.STATIONARY_VREL_TOL_MPS
        dynstat = getattr(target, "dyn", None) in C.STATIONARY_DYN
        if ground or dynstat:
            if lifted:
                tags.append(f"억제 해제:정지 물체지만 {lift_why or '카메라/V2V 앞차'}가 뒷받침")
            else:
                level = 1; tags.append(f"억제:정지 물체({'대지' if ground else 'dyn'})→최대 주의")
    if level == 2 and C.SUPPRESS_ONCOMING_PASSING and v_r is not None and closing_speed(target.vrel_ms) > v_r + C.ONCOMING_MARGIN_MPS:
        level = 1; tags.append(f"억제:마주오는/추월 차(c {closing_speed(target.vrel_ms):.1f} > v_r {v_r:.1f}+{C.ONCOMING_MARGIN_MPS:g} m/s)→최대 주의")
    lat = getattr(target, "lat_m", None)
    if level == 2 and lat is not None and LAT2_MAX_M is not None and abs(lat) > LAT2_MAX_M:
        level = 1; tags.append(f"억제(옵션):옆 간격 |{lat:.1f}| m > LAT2_MAX_M {LAT2_MAX_M:g}→최대 주의")
    if level == 2 and lat is not None and PASS_D_M is not None and PASS_LAT_M is not None and target.long_m < PASS_D_M and abs(lat) >= PASS_LAT_M:
        level = 1; tags.append(f"억제(옵션):나란히 지나가는 물체(d {target.long_m:.1f} < {PASS_D_M:g} m, |lat| {abs(lat):.1f} >= {PASS_LAT_M:g} m)→최대 주의")
    return level, tags


def assess(radar_alive, summary_valid, safe_mode, target, v_r, k_r, k_f, scale=1.0, lifted=False, lift_headway=None, lift_why=""):
    """target: object with .long_m .vrel_ms (.dyn optional) or None. Returns (level incl. 0xFF, reason, metrics dict).
    metrics: a_req, ttc, d_req1, pre (level before suppression), supp (list of tags)."""
    m = dict(a_req=None, ttc=None, d_req1=None, pre=None, supp=[])
    if not summary_valid:
        return DANGER_UNKNOWN, "모름: 0x10 무효(STM32 링크)", m
    if safe_mode:
        return DANGER_UNKNOWN, "모름: STM32 SAFE_MODE", m
    if not radar_alive:
        return DANGER_UNKNOWN, "모름: 레이더 이상(RADAR_OK=0)", m
    if target is None:
        m["pre"] = 0
        return 0, "타깃 없음(레이더 정상)", m
    lv, why, met = raw_level(target.long_m, target.vrel_ms, v_r, k_r, k_f, scale)
    m.update(met); m["pre"] = lv
    lv2, tags = _suppress(lv, v_r, target, lifted, lift_headway, lift_why)
    m["supp"] = tags
    return lv2, why + ("" if not tags else " [" + "; ".join(tags) + "]"), m


@dataclass
class DangerState:
    level: int = 0
    rise_n: int = 0
    rise_to: int = 0
    fall_since: Optional[float] = None


def step(state: DangerState, now, radar_alive, summary_valid, safe_mode, target, v_r, k_r, k_f, lifted=False, lift_headway=None, lift_why=""):
    """One evaluation with hysteresis. Returns (level, reason, metrics). `state` is updated in place."""
    lv, why, m = assess(radar_alive, summary_valid, safe_mode, target, v_r, k_r, k_f, lifted=lifted, lift_headway=lift_headway, lift_why=lift_why)
    if lv == DANGER_UNKNOWN:
        state.level, state.rise_n, state.fall_since = DANGER_UNKNOWN, 0, None
        return lv, why, m
    if state.level == DANGER_UNKNOWN:
        state.level = 0                                            # coming back from unknown: start from safe, rise rules apply
    if lv > state.level:
        state.rise_to = lv if state.rise_n == 0 else min(state.rise_to, lv)
        state.rise_n += 1
        state.fall_since = None
        if state.rise_n >= C.RISE_CONFIRM_N:
            state.level, state.rise_n = state.rise_to, 0
        else:
            why += f" [상승 대기 {state.rise_n}/{C.RISE_CONFIRM_N}]"
    else:
        state.rise_n = 0
        if state.level > 0:
            lm = assess(radar_alive, summary_valid, safe_mode, target, v_r, k_r, k_f, scale=C.FALL_MARGIN, lifted=lifted, lift_headway=lift_headway, lift_why=lift_why)[0]
            if lm < state.level:
                state.fall_since = now if state.fall_since is None else state.fall_since
                if now - state.fall_since >= C.FALL_HOLD_S:
                    state.level, state.fall_since = lm, None
                else:
                    why += f" [하강 대기 {now - state.fall_since:.2f}/{C.FALL_HOLD_S}s]"
            else:
                state.fall_since = None
    return state.level, why, m


def front_k(peer, stale):
    """K_f for d_req from the front car's BSM. peer: decoded BSM dict or None; stale: freshness verdict. Returns (K_f, reason)."""
    if peer is None or stale:
        return C.K_FRONT_DEFAULT, f"K_f={C.K_FRONT_DEFAULT} 기본값 ({'STALE' if peer is not None else '수신 없음'})"
    k = peer.get("k_value")
    if k is None or not (C.K_MIN <= k <= C.K_MAX):
        return C.K_FRONT_DEFAULT, f"K_f={C.K_FRONT_DEFAULT} 기본값 (상대 K 없음/범위 밖: {k})"
    return k, f"K_f={k:.2f} 상대 BSM"


class _T:
    def __init__(self, long_m, vrel_ms, dyn=None):
        self.long_m, self.vrel_ms, self.dyn = long_m, vrel_ms, dyn


def _selftest():
    v60 = 60 / 3.6
    # --- d_req values (d0 included)
    assert abs(d_req(v60, v60, 0.5, 0.8, 1.5) - 37.6) < 0.1                      # both 60 km/h, d_req(1.5 s) = 37.6 m (35.6 m without d0)
    assert abs(d_req(v60, v60, 0.5, 0.8, 1.0) - 29.3) < 0.1                      # d_req(1.0 s) = 29.3 m
    assert abs(d_req(v60, 0.0, 0.5, 0.8, 1.5) - 55.3) < 0.1                      # standing lead: old S4 number, kept for reference
    # --- boundaries behind a standing car: 60 km/h -> 0, K_r=0.5, d0=2
    def lvl(d, v=v60, vrel=-v60, kr=0.5, kf=0.8):
        return raw_level(d, vrel, v, kr, kf)[0]
    assert lvl(42.0) == 2 and lvl(43.0) == 1, (lvl(42.0), lvl(43.0))             # danger boundary ~ 42.5 m
    assert lvl(72.0) == 1 and lvl(74.0) == 0, (lvl(72.0), lvl(74.0))             # caution boundary ~ 72.9 m
    # --- equal speeds: not closing -> no danger possible, caution only inside d_req(1.0 s) = 29.3 m
    assert lvl(20.0, vrel=0.0) == 1 and lvl(35.0, vrel=0.0) == 0
    assert all(lvl(d, vrel=0.0) < 2 for d in (1.0, 3.0, 10.0, 20.0))
    # --- speed unknown: d_req term dropped, TTC still works (c=10, d=15 -> TTC 1.5 s -> danger)
    lv, why, m = raw_level(15.0, -10.0, None, 0.5, 0.8)
    assert lv == 2 and abs(m["ttc"] - 1.5) < 1e-9 and "속도 없음" in why, (lv, why)
    assert raw_level(30.0, 0.0, None, 0.5, 0.8)[0] == 0                          # nothing to evaluate without speed and closing
    # --- unknown / safe
    assert assess(False, True, False, _T(10, -5), v60, 0.5, 0.8)[0] == 0xFF
    assert assess(True, False, False, _T(10, -5), v60, 0.5, 0.8)[0] == 0xFF
    assert assess(True, True, True, _T(10, -5), v60, 0.5, 0.8)[0] == 0xFF
    lv, why, _ = assess(True, True, False, None, v60, 0.5, 0.8)
    assert lv == 0 and "타깃 없음" in why
    # --- suppressions
    stat = _T(30.0, -v60, dyn=1)                                                  # fixed to the ground, would be danger
    lv, why, m = assess(True, True, False, stat, v60, 0.5, 0.8)
    assert m["pre"] == 2 and lv == 1 and "정지 물체" in why and m["supp"], (lv, why)
    lv, why, m = assess(True, True, False, stat, v60, 0.5, 0.8, lifted=True)      # camera / V2V backs it -> not capped
    assert lv == 2 and "억제 해제" in why, (lv, why)
    mov = _T(10.0, -8.0, dyn=0)                                                   # moving car ahead closing at 8 m/s (v_r 60): not stationary
    lv, why, m = assess(True, True, False, mov, v60, 0.5, 0.8)
    assert lv == 2 and not m["supp"], (lv, why)
    lv, why, m = assess(True, True, False, _T(10.0, -8.0, dyn=0), 1.0 / 3.6, 0.5, 0.8)   # we are nearly standing (1 km/h)
    assert lv == 1 and "정차/서행" in why, (lv, why)
    C.SUPPRESS_STATIONARY_OBJECT = False
    try:
        assert assess(True, True, False, stat, v60, 0.5, 0.8)[0] == 2, "flag off -> no suppression"
    finally:
        C.SUPPRESS_STATIONARY_OBJECT = True
    # --- implausible headway (S7): d / v_r < 0.3 s at >= 10 km/h -> at most caution; below 10 km/h, own speed unknown, or a normal approach -> unchanged
    lv, why, m = assess(True, True, False, _T(1.8, -3.0, dyn=0), 55 / 3.6, 0.5, 0.8)             # 55 km/h, 1.8 m (0.12 s) -> limited
    assert m["pre"] == 2 and lv == 1 and "implausible-headway" in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(1.4, -4.0, dyn=0), 5 / 3.6, 0.5, 0.8)              # 5 km/h, 1.4 m -> rule not applied (below 10 km/h)
    assert lv == 2 and "implausible" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(1.8, -6.0, dyn=0), None, 0.5, 0.8)                 # own speed unknown -> not applied
    assert lv == 2 and "implausible" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(40.0, -v60, dyn=0), v60, 0.5, 0.8, lifted=True)    # stopped car 40 m ahead at 60 km/h (headway 2.4 s): stays danger
    assert lv == 2 and "implausible" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(5.2, -3.0, dyn=0), v60, 0.5, 0.8)                  # 5.2 m at 60 km/h = 0.31 s: just outside the rule
    assert "implausible" not in why, why
    lv, why, m = assess(True, True, False, _T(4.9, -3.0, dyn=0), v60, 0.5, 0.8)                  # 4.9 m at 60 km/h = 0.29 s: inside
    assert "implausible" in why and lv <= 1, (lv, why)
    # --- S8: the implausible-headway cap is LIFTED by camera_confirmed / V2V position support
    lv, why, m = assess(True, True, False, _T(1.8, -3.0, dyn=0), 55 / 3.6, 0.5, 0.8, lifted=True, lift_why="camera_confirmed")         # 55 km/h, 1.8 m + camera
    assert m["pre"] == 2 and lv == 2 and "억제 해제:implausible-headway" in why and "camera_confirmed" in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(1.8, -3.0, dyn=0), 55 / 3.6, 0.5, 0.8)                                                  # same gap, no camera: cap stays
    assert lv == 1 and "implausible-headway" in why and "해제" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(1.8, -3.0, dyn=0), 55 / 3.6, 0.5, 0.8, lifted=False, lift_headway=True, lift_why="V2V 앞차")  # moving V2V front car backs 'a vehicle is there'
    assert lv == 2 and "V2V 앞차" in why, (lv, why)
    # slow close approach to a STOPPED car (the demo scenario): 20 km/h, 1.5 m (0.27 s) -> both caps apply; with support the danger is kept
    stopcar = _T(1.5, -20 / 3.6, dyn=0)
    lv, why, m = assess(True, True, False, stopcar, 20 / 3.6, 0.5, 0.8)
    assert m["pre"] == 2 and lv == 1 and "implausible-headway" in why, (lv, why)
    lv, why, m = assess(True, True, False, stopcar, 20 / 3.6, 0.5, 0.8, lifted=True, lift_why="V2V 앞차(정지)")
    assert lv == 2 and why.count("해제") == 2, (lv, why)                                            # both the headway cap and the stationary-object cap are lifted
    # below 10 km/h the headway cap does not exist; the stationary-object cap still needs support
    lv, why, m = assess(True, True, False, _T(1.5, -8 / 3.6, dyn=0), 8 / 3.6, 0.5, 0.8)
    assert lv == 1 and "implausible" not in why and "정지 물체" in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(1.5, -8 / 3.6, dyn=0), 8 / 3.6, 0.5, 0.8, lifted=True)
    assert lv == 2, (lv, why)
    C.SUPPRESS_IMPLAUSIBLE_HEADWAY = False
    try:
        assert assess(True, True, False, _T(1.8, -3.0, dyn=0), 55 / 3.6, 0.5, 0.8)[0] == 2, "flag off -> no limit"
    finally:
        C.SUPPRESS_IMPLAUSIBLE_HEADWAY = True
    # --- oncoming / passing rule (S6): speeds only, dyn ignored
    stopped = _T(30.0, -v60, dyn=0)                                                # stopped car ahead: c == v_r  -> NOT oncoming/passing
    lv, why, m = assess(True, True, False, stopped, v60, 0.5, 0.8, lifted=True)
    assert lv == 2 and "마주오는" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(30.0, -(v60 + 3.5), dyn=0), v60, 0.5, 0.8)   # c = v_r + 3.5 > v_r + 3 -> moves toward us
    assert m["pre"] == 2 and lv == 1 and "마주오는" in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(30.0, -(v60 + 2.5), dyn=0), v60, 0.5, 0.8, lifted=True)   # c = v_r + 2.5: not limited
    assert lv == 2 and "마주오는" not in why, (lv, why)
    lv, why, m = assess(True, True, False, _T(15.0, -(v60 + 10.0), dyn=2), None, 0.5, 0.8)            # own speed unknown: rule not applied
    assert lv == 2 and "마주오는" not in why, (lv, why)
    for dyn in (None, 0, 2):                                                                          # the result is the same whatever dyn says
        lv, why, m = assess(True, True, False, _T(30.0, -(v60 + 10.0), dyn=dyn), v60, 0.5, 0.8)
        assert lv == 1 and "마주오는" in why, (dyn, lv, why)
    C.SUPPRESS_ONCOMING_PASSING = False
    try:
        assert assess(True, True, False, _T(30.0, -(v60 + 10.0), dyn=0), v60, 0.5, 0.8)[0] == 2, "flag off -> no limit"
    finally:
        C.SUPPRESS_ONCOMING_PASSING = True
    # --- hysteresis: needs 2 consecutive to rise
    st = DangerState(); t = 0.0
    def go(d, vrel=-5.0, v=v60):
        nonlocal t
        t += 0.05; return step(st, t, True, True, False, _T(d, vrel, dyn=0), v, 0.5, 0.8)[0]
    assert go(8) == 0, "first danger-evaluation must wait"
    assert go(8) == 2, "second consecutive -> 2"
    lv = [go(300, vrel=0.0) for _ in range(12)]
    assert lv[0] == 2 and lv[-1] == 0, lv                                          # far away and not closing: falls after 0.5 s
    # --- front K
    assert front_k(None, True)[0] == C.K_FRONT_DEFAULT
    assert front_k({"k_value": 0.6}, True)[0] == C.K_FRONT_DEFAULT
    assert front_k({"k_value": 0.6}, False)[0] == 0.6
    assert front_k({"k_value": None}, False)[0] == C.K_FRONT_DEFAULT
    print("danger selftest OK  (d_req(1.0)=%.1f d_req(1.5)=%.1f both 60 km/h; boundaries behind a standing car: danger 42.5 m, caution 72.9 m)"
          % (d_req(v60, v60, .5, .8, 1.0), d_req(v60, v60, .5, .8, 1.5)))


if __name__ == "__main__":
    _selftest()
