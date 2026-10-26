#!/usr/bin/env python3
"""
AIV V2V 노드 — 각 라즈베리파이에서 실행

  # ESP32(ESP-NOW) 사용
  python3 v2v_node.py --id 1 --transport esp-now --serial-port /dev/ttyUSB0
  # 같은 Wi-Fi(핫스팟)에서 UDP 사용
  python3 v2v_node.py --id 2 --transport udp
  # 실제 GPS(gpsd) 사용 — 없으면 더미 위치로 통신만 테스트
  python3 v2v_node.py --id 3 --transport esp-now --gps

차량 ID: 1=후방차, 2=전방차, 3=측면차
"""
import argparse
import json
import math
import socket
import threading
import time

from bsm_protocol import BSM, FrameError, STATUS_GPS_VALID, VEHICLE_NAMES, decode, encode
from freshness import VehicleTracker
from transport import EspNowTransport, UDPBroadcastTransport

BASE_LAT, BASE_LON = 37.5503, 127.0731
M_PER_DEG_LAT = 111_320.0


class DummySource:
    """GPS 없이 통신만 테스트: 세 차량이 같은 도로를 북쪽으로 36km/h 주행한다고 가정"""
    OFFSETS = {1: (0.0, 0.0), 2: (30.0, 0.0), 3: (10.0, 3.5)}   # (전방 m, 동쪽 m)
    SPEED_KMH = 36.0

    def __init__(self, vid: int):
        self.north0, self.east0 = self.OFFSETS.get(vid, (0.0, 0.0))

    def read(self) -> dict:
        t = time.time() % 60.0   # 벽시계 기준이라 파이끼리 시간이 맞으면 서로 위치도 맞음
        north = self.north0 + self.SPEED_KMH / 3.6 * t
        lat = BASE_LAT + north / M_PER_DEG_LAT
        lon = BASE_LON + self.east0 / (M_PER_DEG_LAT * math.cos(math.radians(BASE_LAT)))
        return dict(lat=lat, lon=lon, speed_kmh=self.SPEED_KMH, heading=0.0, gps_valid=True)


class GpsdSource:
    """gpsd(localhost:2947) JSON에서 TPV 읽기. 외부 라이브러리 불필요."""

    def __init__(self, host: str = '127.0.0.1', port: int = 2947):
        self.host, self.port = host, port
        self.fix = dict(lat=0.0, lon=0.0, speed_kmh=0.0, heading=0.0, gps_valid=False)
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as s:
                    s.sendall(b'?WATCH={"enable":true,"json":true};\n')
                    for line in s.makefile('r'):
                        try:
                            msg = json.loads(line)
                        except ValueError:
                            continue
                        if msg.get('class') != 'TPV':
                            continue
                        ok = msg.get('mode', 0) >= 2 and 'lat' in msg and 'lon' in msg
                        with self.lock:
                            if ok:
                                self.fix.update(
                                    lat=msg['lat'],
                                    lon=msg['lon'],
                                    speed_kmh=msg.get('speed', 0.0) * 3.6,
                                    heading=msg.get('track', self.fix['heading']),
                                )
                            self.fix['gps_valid'] = ok
            except OSError:
                with self.lock:
                    self.fix['gps_valid'] = False
                time.sleep(2)

    def read(self) -> dict:
        with self.lock:
            return dict(self.fix)


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(x, y)) % 360.0


def rel_angle(a: float) -> float:
    """-180 ~ +180 로 정규화"""
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser(description='AIV V2V BSM 노드')
    ap.add_argument('--id', type=int, required=True, choices=[1, 2, 3], help='1=후방 2=전방 3=측면')
    ap.add_argument('--transport', choices=['udp', 'esp-now'], default='esp-now')
    ap.add_argument('--serial-port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--broadcast-addr', default='255.255.255.255')
    ap.add_argument('--udp-port', type=int, default=9000)
    ap.add_argument('--rate', type=float, default=10.0, help='BSM 송신 주기(Hz)')
    ap.add_argument('--gps', action='store_true', help='gpsd 실제 GPS 사용 (기본: 더미 위치)')
    ap.add_argument('--quiet', action='store_true', help='프레임별 출력 끄고 1초 요약만')
    ap.add_argument('--dump-raw', help='[진단] ESP32 원시 바이트 저장 파일')
    args = ap.parse_args()

    if args.transport == 'esp-now':
        transport = EspNowTransport(args.serial_port, args.baud, dump_path=args.dump_raw)
    else:
        transport = UDPBroadcastTransport(args.udp_port, args.broadcast_addr)

    source = GpsdSource() if args.gps else DummySource(args.id)
    tracker = VehicleTracker()
    stop = threading.Event()
    own = {'fix': source.read(), 'tx': 0}
    my_name = VEHICLE_NAMES[args.id]

    print(f'[{my_name}] id={args.id} transport={transport.name} rate={args.rate}Hz '
          f'source={"gpsd" if args.gps else "dummy"}')

    def sender():
        seq = 0
        period = 1.0 / args.rate
        nxt = time.monotonic()
        while not stop.is_set():
            fix = source.read()
            own['fix'] = fix
            m = BSM(
                vehicle_id=args.id,
                seq=seq,
                timestamp_ms=int(time.monotonic() * 1000),
                lat=fix['lat'],
                lon=fix['lon'],
                speed_kmh=fix['speed_kmh'],
                heading_deg=fix['heading'],
                k_value=None,               # STM32 연동 전까지 N/A
                status=STATUS_GPS_VALID if fix['gps_valid'] else 0,
            )
            try:
                transport.send(encode(m))
                own['tx'] += 1
            except OSError as e:
                print(f'[TX ERR] {e}')
            seq = (seq + 1) & 0xFFFF
            nxt += period
            stop.wait(max(0.0, nxt - time.monotonic()))

    threading.Thread(target=sender, daemon=True).start()

    drops = {}
    last_summary = time.monotonic()
    t0 = last_summary
    first_dup = False
    try:
        while True:
            data = transport.recv(0.1)
            now = time.monotonic()
            if data:
                try:
                    m = decode(data)
                except FrameError as e:
                    drops[e.reason] = drops.get(e.reason, 0) + 1
                    if not args.quiet:
                        print(f'[DROP] {e.reason}')
                else:
                    if m.vehicle_id != args.id:   # UDP는 내가 보낸 것도 돌아옴
                        res = tracker.accept(m, now)
                        if res == 'dup' and not first_dup:
                            first_dup = True
                            print(f'[진단] 첫 DUP: 시작 후 {1000 * (now - t0):.0f}ms, seq={m.seq}')
                        if not args.quiet:
                            k = 'N/A' if m.k_value is None else f'{m.k_value:.2f}'
                            tag = 'OK ' if res == 'ok' else 'DUP'
                            print(f'[{tag}] {VEHICLE_NAMES.get(m.vehicle_id, m.vehicle_id)} '
                                  f'seq={m.seq:5d} speed={m.speed_kmh:6.1f}km/h '
                                  f'heading={m.heading_deg:6.1f} K={k} '
                                  f'gps={"Y" if m.gps_valid else "N"} danger={m.danger}')

            if now - last_summary >= 1.0:
                last_summary = now
                fix = own['fix']
                parts = []
                for s in tracker.summary(now):
                    b = s['bsm']
                    txt = (f'{VEHICLE_NAMES.get(s["vid"], s["vid"])}='
                           f'{"LIVE" if s["live"] else "STALE"}'
                           f'(age {s["age_ms"]}ms, rx {s["count"]}, lost {s["lost"]}, dup {s["dup"]})')
                    if fix['gps_valid'] and b.gps_valid:
                        d = haversine_m(fix['lat'], fix['lon'], b.lat, b.lon)
                        rb = rel_angle(bearing_deg(fix['lat'], fix['lon'], b.lat, b.lon) - fix['heading'])
                        txt += f' dist={d:.1f}m rel={rb:+.0f}deg'
                    parts.append(txt)
                if not parts:
                    parts.append('수신 없음')
                extra = f' drop={drops}' if drops else ''
                print(f'--- [{my_name}] tx={own["tx"]} | ' + ' | '.join(parts) + extra)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        transport.close()


if __name__ == '__main__':
    main()
