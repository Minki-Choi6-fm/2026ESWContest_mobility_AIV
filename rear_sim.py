#!/usr/bin/env python3
"""
후방차 흉내 (전방차 시험용) — 후방차 V2V 가 붙기 전에 전방차 경고 기능만 시험

전방차 BSM 을 받아서 그 차의 '뒤 GAP m' 위치에 있는 것처럼 후방차 BSM(id=1)을 보내고,
위험 플래그를 일정 주기로 켰다 껐다 한다. 전방차가 경고를 켜고 끄는지 보면 된다.
전방차가 보내는 급제동(BRAKE) 비트도 화면에 표시한다.

  python3 rear_sim.py --serial-port /dev/ttyUSB0           # 다른 라즈베리파이/노트북 + ESP32
  python3 rear_sim.py --transport udp                      # 같은 Wi-Fi 에서
  python3 rear_sim.py --side                                # 옆 차선(경고가 무시돼야 정상) 시험
"""
import argparse
import math
import time

from bsm_protocol import BSM, FrameError, STATUS_DANGER_ACTIVE, STATUS_GPS_VALID, decode, encode
from transport import EspNowTransport, UDPBroadcastTransport

M = 111_320.0


def offset(lat, lon, heading_deg, back_m, right_m):
    """(lat,lon) 에서 진행방향 기준 뒤로 back_m, 오른쪽으로 right_m 이동한 위치"""
    h = math.radians(heading_deg)
    north = -back_m * math.cos(h) + right_m * math.cos(h + math.pi / 2)
    east = -back_m * math.sin(h) + right_m * math.sin(h + math.pi / 2)
    return lat + north / M, lon + east / (M * math.cos(math.radians(lat)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--transport', choices=['udp', 'esp-now'], default='esp-now')
    ap.add_argument('--serial-port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--broadcast-addr', default='255.255.255.255')
    ap.add_argument('--udp-port', type=int, default=9000)
    ap.add_argument('--gap', type=float, default=15.0, help='전방차 뒤 몇 m 에 있는 척할지')
    ap.add_argument('--side', action='store_true', help='뒤가 아니라 옆 차선(오른쪽 3.5m, 나란히)')
    ap.add_argument('--on', type=float, default=3.0, help='위험 ON 시간(초)')
    ap.add_argument('--off', type=float, default=3.0, help='위험 OFF 시간(초)')
    a = ap.parse_args()

    tr = (EspNowTransport(a.serial_port, a.baud) if a.transport == 'esp-now'
          else UDPBroadcastTransport(a.udp_port, a.broadcast_addr))
    front = None
    seq = 0
    t0 = time.monotonic()
    last_print = 0
    print(f'[후방차 흉내] gap={a.gap}m side={a.side} danger ON {a.on}s / OFF {a.off}s')
    try:
        while True:
            end = time.monotonic() + 0.1
            while time.monotonic() < end:
                data = tr.recv(0.02)
                if not data:
                    continue
                try:
                    b = decode(data)
                except FrameError:
                    continue
                if b.vehicle_id == 2:
                    front = (b, time.monotonic())
            now = time.monotonic()
            danger = ((now - t0) % (a.on + a.off)) < a.on
            if front and now - front[1] < 1.0 and front[0].gps_valid:
                fb = front[0]
                lat, lon = (offset(fb.lat, fb.lon, fb.heading_deg, 0.0, 3.5) if a.side
                            else offset(fb.lat, fb.lon, fb.heading_deg, a.gap, 0.0))
                st = STATUS_GPS_VALID
                spd, hdg = fb.speed_kmh, fb.heading_deg
            else:
                lat = lon = 0.0
                st, spd, hdg = 0, 0.0, 0.0      # 전방차 위치를 모르면 GPS 무효로 보냄
            if danger:
                st |= STATUS_DANGER_ACTIVE
            tr.send(encode(BSM(vehicle_id=1, seq=seq, timestamp_ms=int(now * 1000), lat=lat, lon=lon,
                               speed_kmh=spd, heading_deg=hdg, k_value=0.45, status=st)))
            seq = (seq + 1) & 0xFFFF
            if now - last_print >= 1.0:
                last_print = now
                f = front[0] if front else None
                print(f'보냄 danger={int(danger)} gps={"Y" if st & STATUS_GPS_VALID else "N"} | 전방차: '
                      + ('없음' if f is None else
                         f'{f.speed_kmh:.1f}km/h brake={int(f.brake)} degraded={int(bool(f.status & 4))}'))
    except KeyboardInterrupt:
        pass
    finally:
        tr.close()


if __name__ == '__main__':
    main()
