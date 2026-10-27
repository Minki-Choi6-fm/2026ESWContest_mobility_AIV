#!/usr/bin/env python3
"""
측면차(위장차량) 통합 노드 -- 측면차 라즈베리파이에서 실행 (STM32 없음, OLED/부저는 파이에 직접 연결)

  GPS(gpsd) --> 측면차 BSM(위치/속도/진행방향) --> ESP32 --> 다른 차
  전방차 BSM(급제동 표시) <-- ESP32  --> 가까이 있으면 OLED "전방추돌 경고" + 부저

역할 규칙 (위치로 차선을 구분하는 것이 아님):
  * 전방차(id 2)가 급제동 표시를 방송하면 경고 (전방차가 100 m 이내일 때. GPS를 모르면 그냥 받아들임)
  * 후방차(id 1)의 위험(danger) 신호에는 반응하지 않음 -- 후방차 경고는 전방차용이므로 무시

필요 파일(같은 폴더): bsm_protocol.py transport.py freshness.py v2v_node.py  (측면차에 이미 있음, 수정 불필요)
필요 패키지: luma.oled pillow gpiozero  (이미 설치됨)

실행 예
  python3 -u side_node.py --gps --serial-port /dev/ttyUSB0 --buzzer-gpio 26
  python3 -u side_node.py --transport udp --no-oled --no-buzzer      # ESP32/OLED/부저 없이 시험
"""
import argparse
import base64
import io
import math
import time

from bsm_protocol import BSM, FrameError, VEHICLE_NAMES, decode, encode
from freshness import VehicleTracker
from transport import EspNowTransport, UDPBroadcastTransport

MY_ID = 3            # 측면차
FRONT_ID = 2         # 전방차
REAR_ID = 1          # 후방차
STATUS_GPS_VALID = 1 << 0
STATUS_BRAKE_ACTIVE = 1 << 4   # 전방차 bsm_protocol.py 와 같은 값(측면차 파일에는 없어서 여기에 둠)

WARN_HOLD_S = 1.0        # 경고가 한 번 들어오면 최소 이만큼 유지(깜빡임 방지)
WARN_MAX_DIST_M = 100.0  # 이보다 먼 전방차의 급제동은 무시(GPS 오차가 커서 느슨하게)

# "전방추돌 경고" 128x64 1비트 그림(전방차 OLED 와 같은 그림)
WARN_PNG_B64 = (
    'iVBORw0KGgoAAAANSUhEUgAAAIAAAABAAQAAAAD6rULSAAABhUlEQVR42q3SMY4TQRAF0NftgXGA'
    'ZIcESJ4jEBKsducIHGG5BdmORMIhQNqjDCSkHGEcQbZjicBeGReBp7GXzdBW0uqqX7/q/27+K+qe'
    '0YqIQbaK/ryaIQ3jwwSshgDVOXw3J1vvWiuPWhZQx6jCe1g37wpiomlKS8IabVumZNjoNvLdErOo'
    'v8Q41rGvIw6ySgdv7mfdkePl92sYfhTSZtJ223aTiPpbz/hCPSHivodfdiJGzNW9vZuIIWKUbbGY'
    '9SdxaWRZFi9qG0tIR/9GLtNk8jghLmP+yNOzxCL6r2kLu7ScEOmgWHhMDMaHHD8N/1DtlbFPFA1c'
    'lJmZt6fas7LH4mA1jcx8PCHmXVGbzv5Z2uNqz8iql7k9Fe/JcsbvlpGmKRwXf0GVKuFDqhLa4ths'
    'q1iUzXVeTbduLltq+dxCUk0td92Z2KsYuOptBrM4lMX616Is1lpK5jQqScX1hvUn8BwittJwcRNb'
    'i4ju8WOn4+9vYTmdpNg+fNvotv90rnpPGH8AGGGFQJ43f3oAAAAASUVORK5CYII='
)


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def front_brake_warning(own_fix, others):
    """전방차가 급제동 중이고 가까이 있으면 (True, 거리m 또는 None). 아니면 (False, None)"""
    for b in others:
        if b.vehicle_id != FRONT_ID or not (b.status & STATUS_BRAKE_ACTIVE):
            continue
        dist = None
        if own_fix.get('gps_valid') and (b.status & STATUS_GPS_VALID):
            dist = haversine_m(own_fix['lat'], own_fix['lon'], b.lat, b.lon)
            if dist > WARN_MAX_DIST_M:
                continue
        return True, dist
    return False, None


class Oled:
    """SSD1306 128x64 I2C. 리셋 핀(BCM)을 직접 제어. 실패해도 프로그램은 계속 돈다(3초마다 재시도)."""

    def __init__(self, addr=0x3C, rst_gpio=17, port=1):
        from PIL import Image
        self._Image = Image
        self.addr, self.rst_gpio, self.port = addr, rst_gpio, port
        self.warn_img = Image.open(io.BytesIO(base64.b64decode(WARN_PNG_B64))).convert('1')
        self.dev = None
        self._rst = None
        self._retry_at = 0.0
        self.mode = None          # 'warn' | 'normal'
        self.last_normal = 0.0
        self._try_init()

    def _try_init(self):
        try:
            if self.rst_gpio:
                from gpiozero import OutputDevice
                if self._rst is None:
                    self._rst = OutputDevice(self.rst_gpio, initial_value=True)   # 프로그램이 끝날 때까지 유지
                self._rst.off(); time.sleep(0.05); self._rst.on(); time.sleep(0.05)
            from luma.core.interface.serial import i2c
            from luma.oled.device import ssd1306
            self.dev = ssd1306(i2c(port=self.port, address=self.addr), width=128, height=64)
            self.mode = None
        except Exception as e:
            self.dev = None
            self._retry_at = time.monotonic() + 3.0
            print(f'[OLED] 초기화 실패: {e} (3초 후 재시도)')

    def _show(self, img):
        if self.dev is None:
            if time.monotonic() >= self._retry_at:
                self._try_init()
            if self.dev is None:
                return False
        try:
            self.dev.display(img)
            return True
        except Exception as e:
            print(f'[OLED] 표시 실패: {e} (3초 후 재초기화)')
            self.dev = None
            self._retry_at = time.monotonic() + 3.0
            self.mode = None
            return False

    def warn(self, now):
        if self.mode != 'warn' or now - self.last_normal >= 1.0:   # 바뀔 때 + 1초마다 다시 그려서 혹시 잘려도 복구
            if self._show(self.warn_img):
                self.mode = 'warn'
            self.last_normal = now

    def normal(self, now, lines):
        if self.mode == 'warn' or now - self.last_normal >= 0.5:
            from PIL import ImageDraw
            img = self._Image.new('1', (128, 64), 0)
            d = ImageDraw.Draw(img)
            for i, t in enumerate(lines[:4]):
                d.text((2, 2 + i * 15), t, fill=1)
            if self._show(img):
                self.mode = 'normal'
            self.last_normal = now


class Buzzer:
    def __init__(self, gpio):
        from gpiozero import Buzzer as _Bz
        self._bz = _Bz(gpio, active_high=True, initial_value=False)
        self.level = 0

    def set_level(self, level):
        if level == self.level:
            return
        self.level = level
        if level == 2:
            self._bz.beep(on_time=0.1, off_time=0.1, background=True)
        else:
            self._bz.off()

    def close(self):
        try:
            self._bz.off()
        finally:
            self._bz.close()


def main():
    ap = argparse.ArgumentParser(description='AIV 측면차 노드')
    ap.add_argument('--transport', choices=['udp', 'esp-now'], default='esp-now')
    ap.add_argument('--serial-port', default='/dev/ttyUSB0', help='ESP32 포트')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--broadcast-addr', default='255.255.255.255')
    ap.add_argument('--udp-port', type=int, default=9000)
    ap.add_argument('--gps', action='store_true', help='gpsd 실제 GPS 사용 (기본: 더미 위치)')
    ap.add_argument('--rate', type=float, default=10.0, help='BSM 주기(Hz)')
    ap.add_argument('--buzzer-gpio', type=int, default=26)
    ap.add_argument('--no-buzzer', action='store_true')
    ap.add_argument('--no-oled', action='store_true')
    ap.add_argument('--oled-addr', type=lambda x: int(x, 0), default=0x3C)
    ap.add_argument('--oled-rst', type=int, default=17, help='OLED 리셋 BCM 핀(물리 11번=BCM17). 0이면 사용 안 함')
    args = ap.parse_args()

    from v2v_node import DummySource, GpsdSource
    transport = (EspNowTransport(args.serial_port, args.baud) if args.transport == 'esp-now'
                 else UDPBroadcastTransport(args.udp_port, args.broadcast_addr))
    source = GpsdSource() if args.gps else DummySource(MY_ID)
    tracker = VehicleTracker()
    oled = None if args.no_oled else Oled(args.oled_addr, args.oled_rst)
    buzzer = None if args.no_buzzer else Buzzer(args.buzzer_gpio)

    print(f'[측면차] transport={transport.name} gps={"gpsd" if args.gps else "dummy"} '
          f'oled={"off" if oled is None else hex(args.oled_addr)} buzzer={"off" if buzzer is None else args.buzzer_gpio} rate={args.rate}Hz')

    period = 1.0 / args.rate
    nxt = time.monotonic()
    seq = tx = 0
    drops = {}
    last_print = 0.0
    warn_until = 0.0
    warn_dist = None
    prev_warn = False
    ignored_rear = 0
    try:
        while True:
            now = time.monotonic()
            fix = source.read()

            # ---- 1) 내 BSM 송신 (측면차는 센서 없음: 위치/속도/방향만) ----
            status = STATUS_GPS_VALID if fix['gps_valid'] else 0
            spd = float(fix.get('speed_kmh', 0.0)) if fix['gps_valid'] else 0.0
            m = BSM(vehicle_id=MY_ID, seq=seq, timestamp_ms=int(now * 1000),
                    lat=fix['lat'], lon=fix['lon'], speed_kmh=spd,
                    heading_deg=fix['heading'], k_value=None, status=status)
            try:
                transport.send(encode(m))
                tx += 1
            except OSError as e:
                print(f'[TX ERR] {e}')
            seq = (seq + 1) & 0xFFFF

            # ---- 2) 다음 주기까지 수신 ----
            nxt += period
            while True:
                remain = nxt - time.monotonic()
                if remain <= 0:
                    break
                data = transport.recv(min(remain, 0.02))
                if not data:
                    continue
                try:
                    b = decode(data)
                except FrameError as e:
                    drops[e.reason] = drops.get(e.reason, 0) + 1
                    continue
                if b.vehicle_id != MY_ID:
                    tracker.accept(b, time.monotonic())
            if nxt < time.monotonic() - period:
                nxt = time.monotonic()

            # ---- 3) 경고 판단: 전방차 급제동만 ----
            now = time.monotonic()
            others = tracker.live_bsms(now)
            hit, dist = front_brake_warning(fix, others)
            if hit:
                warn_until, warn_dist = now + WARN_HOLD_S, dist
            warn = now < warn_until
            if any(b.vehicle_id == REAR_ID and (b.status & 2) for b in others):
                ignored_rear += 1          # 후방차 위험 신호는 측면차용이 아니므로 무시(횟수만 기록)

            if buzzer is not None:
                buzzer.set_level(2 if warn else 0)
            if oled is not None:
                if warn:
                    oled.warn(now)
                else:
                    live = {b.vehicle_id for b in others}
                    oled.normal(now, ['SIDE CAR  fx1',
                                      f'SPD {spd:4.1f} km/h GPS:{"Y" if fix["gps_valid"] else "N"}',
                                      f'FRONT:{"LIVE" if FRONT_ID in live else "--"} REAR:{"LIVE" if REAR_ID in live else "--"}',
                                      'WARN: none'])
            if warn != prev_warn:
                print('*** 경고 ON: 전방차 급제동%s ***' % ('' if warn_dist is None else f' {warn_dist:.1f}m(GPS)') if warn else '*** 경고 OFF ***')
                prev_warn = warn

            # ---- 4) 1초 요약 ----
            if now - last_print >= 1.0:
                last_print = now
                parts = []
                for s in tracker.summary(now):
                    bb = s['bsm']
                    parts.append(f'{VEHICLE_NAMES.get(s["vid"], s["vid"])}='
                                 f'{"LIVE" if s["live"] else "STALE"}'
                                 f'{" BRAKE" if bb.status & STATUS_BRAKE_ACTIVE else ""}'
                                 f'{" DANGER" if bb.status & 2 else ""}(lost {s["lost"]})')
                print(f'--- tx={tx} speed={spd:.1f}km/h gps={"Y" if fix["gps_valid"] else "N"} '
                      f'warn={int(warn)} ignored_rear_danger={ignored_rear} | ' + (' | '.join(parts) or '수신 없음')
                      + (f' drop={drops}' if drops else ''))
    except KeyboardInterrupt:
        print('\n종료합니다...')
    finally:
        if buzzer is not None:
            buzzer.close()
        transport.close()


if __name__ == '__main__':
    main()
