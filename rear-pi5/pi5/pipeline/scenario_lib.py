"""S17-A: scripted FAKE radar scenarios (own car + objects) for scenario_suite.py. Pure functions of time: no I/O, nothing real is touched.

Each scenario = dict(name, desc, dur, own_kmh (None = no OBD/GPS speed), obj(t) -> list of dict(id, d, lat, vrel, dyn), radar_ok(t) -> bool, expect (levels allowed / required)).
Conventions (same as the radar frames): d = distance ahead [m], lat = lateral position [m, 0 = straight ahead], vrel = relative speed [m/s] (negative = closing),
dyn = radar dynamic property (0 moving, 1 stationary ...). Own car drives at a constant speed, the driver never reacts.
The expectations come from the documented rules (danger.py / OUTPUT_POLICY.md); they are NOT measured truth. [PROPOSAL] constants stay untouched.
"""
V60 = 60 / 3.6


def _lead_brakes(t, own=V60, gap0=25.0, t0=2.5, decel=6.0):
    if t < t0:
        return gap0, 0.0
    tb = t - t0; ts = own / decel
    if tb <= ts:
        gap = gap0 - 0.5 * decel * tb * tb; v = own - decel * tb
    else:
        gap = gap0 - 0.5 * decel * ts * ts - own * (tb - ts); v = 0.0
    return max(1.5, gap), v - own


def S():
    sc = []
    sc.append(dict(name="1 정속 추종(안전거리)", desc="앞차 40 m, 같은 속도 60 km/h", dur=12, own_kmh=60,
                   obj=lambda t: [dict(id=3, d=40.0, lat=0.0, vrel=0.0, dyn=0)], radar_ok=lambda t: True,
                   expect=dict(allowed={0}, required=set(), danger_bsm=False)))
    sc.append(dict(name="2 앞차 천천히 접근", desc="앞차가 3 m/s 느림, 60 m → 약 18 m", dur=15, own_kmh=60,
                   obj=lambda t: [dict(id=3, d=max(15.0, 60.0 - 3.0 * t), lat=0.0, vrel=-3.0, dyn=0)], radar_ok=lambda t: True,
                   expect=dict(allowed={0, 1}, required={1}, danger_bsm=False)))
    sc.append(dict(name="3a 앞차 급정거(움직이던 앞차)", desc="같은 속도로 25 m 앞, 2.5초에 6 m/s² 제동", dur=12, own_kmh=60,
                   obj=lambda t: [dict(id=3, d=_lead_brakes(t)[0], lat=0.0, vrel=_lead_brakes(t)[1], dyn=0)], radar_ok=lambda t: True,
                   expect=dict(allowed={0, 1, 2}, required={2}, danger_bsm=True)))
    sc.append(dict(name="3b 정지한 앞차에 급접근", desc="정지 차량이 80 m 앞(레이더 dyn=정지), 자차 60 km/h", dur=8, own_kmh=60,
                   obj=lambda t: [dict(id=3, d=max(2.0, 80.0 - V60 * t), lat=0.0, vrel=-V60, dyn=1)], radar_ok=lambda t: True,
                   expect=dict(allowed={0, 1}, required={1}, danger_bsm=False, note="레이더만으로는 정지 물체 억제로 최대 1 (카메라/V2V 없을 때)")))
    def cutin(t):
        lat = max(0.0, 3.0 - 1.0 * max(0.0, t - 2.0)) if t >= 2.0 else 3.0
        vrel = 0.0 if t < 6 else -3.0
        d = 22.0 if t < 6 else max(10.0, 22.0 - 3.0 * (t - 6))
        return [dict(id=5, d=d, lat=lat, vrel=vrel, dyn=0)]
    sc.append(dict(name="4 옆차로 끼어들기", desc="옆 차선(횡 3 m)에서 2~5초 사이 차선 안으로 들어와 느려짐", dur=14, own_kmh=60,
                   obj=cutin, radar_ok=lambda t: True, expect=dict(allowed={0, 1, 2}, required={1}, danger_bsm=None)))
    sc.append(dict(name="5 정지물체 통과(길가 표지)", desc="길가 정지 물체(횡 2.5 m, dyn=정지) 자차 60 km/h로 통과", dur=6, own_kmh=60,
                   obj=lambda t: [dict(id=9, d=max(1.0, 70.0 - V60 * t), lat=2.5, vrel=-V60, dyn=1)], radar_ok=lambda t: True,
                   expect=dict(allowed={0}, required=set(), danger_bsm=False)))
    def indoor(t):
        # desk-like near-field flicker: one id jumps between 1.0-1.8 m with vrel -0.5..-0.75 for ~0.5 s every 3 s
        ph = t % 3.0
        if ph < 0.6:
            return [dict(id=0, d=1.4, lat=1.0, vrel=-0.75, dyn=0)]
        return [dict(id=3, d=1.8, lat=-0.2, vrel=0.0, dyn=0)]
    sc.append(dict(name="6a 1~1.8 m 근거리 튐(주행 60 km/h)", desc="밀폐 실내식 근거리 반사", dur=14, own_kmh=60,
                   obj=indoor, radar_ok=lambda t: True, expect=dict(allowed={0, 1}, required=set(), danger_bsm=False)))
    sc.append(dict(name="6b 근거리 튐(정차 0 km/h)", desc="같은 반사, 자차 정차", dur=14, own_kmh=0,
                   obj=indoor, radar_ok=lambda t: True, expect=dict(allowed={0, 1}, required=set(), danger_bsm=False)))
    sc.append(dict(name="6c 근거리 튐(속도 모름)", desc="같은 반사, OBD/GPS 없음 (책상과 같은 조건)", dur=14, own_kmh=None,
                   obj=indoor, radar_ok=lambda t: True, expect=dict(allowed={0, 1, 2}, required=set(), danger_bsm=None, buzzer_must_be_off=True)))
    sc.append(dict(name="7 레이더 끊김→복구", desc="4~7초 레이더 단절(STM32 SAFE_MODE), 이후 앞차 60 m", dur=14, own_kmh=60,
                   obj=lambda t: [dict(id=3, d=40.0, lat=0.0, vrel=0.0, dyn=0)] if t < 4 else [dict(id=3, d=60.0, lat=0.0, vrel=0.0, dyn=0)],
                   radar_ok=lambda t: not (4.0 <= t < 7.0),
                   expect=dict(allowed={0, 255}, required={255}, danger_bsm=False, end_level=0)))
    sc.append(dict(name="B 속도 주입용(앞차 급정거, OBD 없음)", desc="3a 와 같은 움직임이지만 STM32 가 속도를 안 보냄 → pipeline_main --test-speed-kmh 로 가짜 속도 주입", dur=12, own_kmh=None,
                   obj=lambda t: [dict(id=3, d=_lead_brakes(t)[0], lat=0.0, vrel=_lead_brakes(t)[1], dyn=0)], radar_ok=lambda t: True,
                   expect=dict(allowed={0, 1, 2}, required=set(), danger_bsm=None)))
    return sc
