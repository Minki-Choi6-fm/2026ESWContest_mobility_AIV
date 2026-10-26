#!/usr/bin/env python3
"""front_logic.py 시험 (하드웨어 없이)  —  python3 test_front_logic.py"""
import math
import sys

from bsm_protocol import BSM, STATUS_DANGER_ACTIVE, STATUS_GPS_VALID
from front_logic import (FrontLogic, encode_v2v_warn, WARN_DANGER, WARN_NONE, TYPE_V2V_WARN)

M = 111_320.0
LAT0, LON0 = 37.5503, 127.0731
results = []


def check(name, cond, note=''):
    results.append(bool(cond))
    print(('PASS  ' if cond else 'FAIL  ') + name + (f'   [{note}]' if note else ''))


def place(north_m, east_m):
    return LAT0 + north_m / M, LON0 + east_m / (M * math.cos(math.radians(LAT0)))


def own(speed=36.0, heading=0.0, gps=True):
    return dict(lat=LAT0, lon=LON0, speed_kmh=speed, heading=heading, gps_valid=gps)


def rear(north_m, east_m, danger=True, heading=0.0, speed=36.0, gps=True, vid=1):
    lat, lon = place(north_m, east_m)
    st = (STATUS_GPS_VALID if gps else 0) | (STATUS_DANGER_ACTIVE if danger else 0)
    return BSM(vehicle_id=vid, seq=0, timestamp_ms=0, lat=lat, lon=lon,
               speed_kmh=speed, heading_deg=heading, k_value=0.45, status=st)


print('== 1. Pi -> STM32 경고 프레임 (0x20)')
f = encode_v2v_warn(WARN_DANGER, 1, 15.3)
x = 0
for b in f[1:-2]:
    x ^= b
check('형식: STX 0x20 LEN=4 ... XOR ETX', f[0] == 2 and f[1] == TYPE_V2V_WARN and f[2] == 4
      and len(f) == 9 and f[-1] == 3 and f[-2] == x, f.hex(' '))
check('거리 15.3m -> 153 (0.1m 단위)', int.from_bytes(f[5:7], 'little') == 153)
check('거리 모름 -> 0xFFFF', int.from_bytes(encode_v2v_warn(0, 0, None)[5:7], 'little') == 0xFFFF)

print('\n== 2. 속도 출처 (OBD 우선, 없으면 GPS)')
L = FrontLogic()
check('STM32 없음 -> GPS', L.speed(own(speed=50), 0.0) == (50.0, 'gps'))
L.on_stm32(0x14, dict(speed_kmh=48, rpm=1500, age_ms=50), 10.0)
check('OBD 신선 -> OBD', L.speed(own(speed=50), 10.2) == (48.0, 'obd'))
check('OBD 오래됨(1초) -> GPS', L.speed(own(speed=50), 11.0) == (50.0, 'gps'))
L.on_stm32(0x14, dict(speed_kmh=None, rpm=None, age_ms=None), 12.0)
check('OBD 값 없음(0xFF) -> GPS', L.speed(own(speed=50), 12.1) == (50.0, 'gps'))
check('GPS도 무효 -> none', FrontLogic().speed(own(gps=False), 0.0) == (0.0, 'none'))

print('\n== 3. 급제동 (IMU 플래그 우선, 없으면 속도 변화로 추정)')
L = FrontLogic()
L.on_stm32(0x13, dict(imu_valid=True, brake=True), 5.0)
check('IMU 신선 + brake=1 -> 급제동', L.brake(36, 'gps', 5.1) == (True, 'imu'))
check('IMU 오래됨 -> 속도 추정으로 전환', L.brake(36, 'gps', 6.0)[1] == 'gps-diff')
L = FrontLogic()
L.on_stm32(0x13, dict(imu_valid=False, brake=True), 5.0)
check('IMU 무효(imu_valid=0)면 플래그 안 믿음', L.brake(36, 'gps', 5.1)[1] == 'gps-diff')

def drive(L, profile, dt=0.1):
    """profile: [(초, km/h)] 구간 선형. 매 dt 마다 brake 를 부르고 (t, on) 기록"""
    out, t = [], 0.0
    end = profile[-1][0]
    while t <= end + 1e-9:
        for (t1, v1), (t2, v2) in zip(profile, profile[1:]):
            if t1 <= t <= t2:
                v = v1 + (v2 - v1) * (t - t1) / (t2 - t1)
                break
        out.append((round(t, 1), L.brake(v, 'gps', t)[0]))
        t += dt
    return out

r = drive(FrontLogic(), [(0, 36), (5, 36)])
check('등속 36km/h -> 급제동 아님', not any(on for _, on in r))
r = drive(FrontLogic(), [(0, 36), (2, 36), (4, 0), (6, 0)])     # 2초 동안 36->0 = 5 m/s^2
on_t = [t for t, on in r if on]
check('36->0km/h 를 2초에 (5 m/s²) -> 급제동 감지', bool(on_t), f'첫 감지 {on_t[0] if on_t else None}s')
check('감지 지연 1초 이내', bool(on_t) and on_t[0] - 2.0 <= 1.0)
check('정지 후 해제', r[-1][1] is False)
r = drive(FrontLogic(), [(0, 36), (2, 36), (6, 0), (8, 0)])     # 4초에 36->0 = 2.5 m/s^2 (보통 감속)
check('보통 감속(2.5 m/s²)은 급제동 아님', not any(on for _, on in r))

print('\n== 4. 후방 경고: "내 뒤에서 오는 차"가 보낸 것만 받기')
def warn(o, others, now=0.0, L=None):
    L = L or FrontLogic()
    return L.warning(o, o['speed_kmh'], others, now), L

(lv, src, d), _ = warn(own(), [rear(-15, 0)])
check('뒤 15m 같은 방향 + 위험 -> 경고', lv == WARN_DANGER and src == 1 and abs(d - 15) < 0.5, f'dist={d:.1f}')
(lv, *_), L = warn(own(), [rear(0, 3.5)])
check('옆 차선(나란히) -> 무시', lv == WARN_NONE, L.last_warn_reason)
(lv, *_), L = warn(own(), [rear(+20, 0)])
check('내 앞에 있는 차 -> 무시', lv == WARN_NONE, L.last_warn_reason)
(lv, *_), L = warn(own(), [rear(-15, 0, heading=180)])
check('뒤에 있지만 반대 방향 -> 무시', lv == WARN_NONE, L.last_warn_reason)
(lv, *_), L = warn(own(), [rear(-150, 0)])
check('150m 뒤 -> 무시', lv == WARN_NONE, L.last_warn_reason)
(lv, *_), L = warn(own(), [rear(-15, 0, danger=False)])
check('위험 플래그 없음 -> 경고 없음', lv == WARN_NONE)
(lv, *_), L = warn(own(), [rear(-15, 0, gps=False)])
check('상대 GPS 무효 -> 확인 못 하니 받아들임', lv == WARN_DANGER, L.last_warn_reason)
(lv, *_), L = warn(own(speed=0), [rear(0, 3.5)])
check('내가 정지(진행방향 모름) -> 거리만 보고 받아들임', lv == WARN_DANGER, L.last_warn_reason)
(lv, *_), L = warn(own(heading=90), [rear(0, -15, heading=90)])
check('동쪽으로 주행 중, 서쪽 15m 뒤 -> 경고 (방향 계산 확인)', lv == WARN_DANGER, L.last_warn_reason)

print('\n== 5. 경고 유지(깜빡임 방지)')
L = FrontLogic()
L.warning(own(), 36, [rear(-15, 0)], 10.0)
check('경고 신호 끊긴 직후(0.5s)에도 유지', L.warning(own(), 36, [], 10.5)[0] == WARN_DANGER)
check('1초 지나면 해제', L.warning(own(), 36, [], 11.1)[0] == WARN_NONE)

print('\n== 6. 여러 차가 동시에 경고 -> 가까운 차 기준')
lv, src, d = FrontLogic().warning(own(), 36, [rear(-40, 0, vid=3), rear(-12, 0, vid=1)], 0.0)
check('가까운 후방차(12m) 선택', src == 1 and abs(d - 12) < 0.5, f'src={src} dist={d:.1f}')

print()
print(f'결과: {sum(results)}/{len(results)} 통과')
sys.exit(0 if all(results) else 1)
