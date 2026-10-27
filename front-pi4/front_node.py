#!/usr/bin/env python3
"""
전방차 통합 노드 — 전방차 라즈베리파이에서 v2v_node.py 대신 실행

  GPS(gpsd) ─┐
  STM32 ─────┼─> 전방차 BSM(위치·속도·진행방향·급제동) ─> ESP32 ─> 후방차
  (OBD/IMU)  │
             └<─ 후방차 BSM(위험 경고) <─ ESP32
                   └─ '내 뒤의 차'가 보낸 경고면 STM32 로 0x20 경고 명령 -> 부저/OLED

실행 예
  python3 front_node.py --gps --stm32-port /dev/serial0 --serial-port /dev/ttyUSB0
  python3 front_node.py --no-stm32                 # STM32 없이 (GPS 속도, 속도변화로 급제동 추정)
  python3 front_node.py --no-stm32 --transport udp # ESP32 없이 같은 Wi-Fi 로 시험

필요 파일(같은 폴더): bsm_protocol.py transport.py freshness.py v2v_node.py aiv_uart_rx.py front_logic.py
"""
import argparse
import threading
import time

from aiv_uart_rx import FrameParser, decode as decode_stm32
from bsm_protocol import (BSM, FrameError, STATUS_BRAKE_ACTIVE, STATUS_GPS_VALID,
                          STATUS_SENSOR_DEGRADED, VEHICLE_NAMES, decode, encode)
from freshness import VehicleTracker
from front_logic import FrontLogic, encode_v2v_warn, WARN_DANGER
from transport import EspNowTransport, UDPBroadcastTransport
from v2v_node import DummySource, GpsdSource

MY_ID = 2   # 전방차


class Stm32Link:
    """STM32 와 UART: 규격 v0.2 프레임 수신(스레드) + 0x20 경고 명령 송신"""

    def __init__(self, port: str, baud: int, logic: FrontLogic, lock: threading.Lock):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = FrameParser()
        self.logic, self.lock = logic, lock
        self.rx_frames = 0
        self.tx_errors = 0
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name='stm32-rx').start()

    def _run(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except OSError:
                time.sleep(0.2)
                continue
            if not data:
                continue
            now = time.monotonic()
            for t, p in self.parser.feed(data):
                self.rx_frames += 1
                with self.lock:
                    self.logic.on_stm32(t, decode_stm32(t, p), now)

    def send_warn(self, level, src, dist_m):
        try:
            self.ser.write(encode_v2v_warn(level, src, dist_m))
        except OSError:
            self.tx_errors += 1

    def close(self):
        self._stop.set()
        self.ser.close()


def main():
    ap = argparse.ArgumentParser(description='AIV 전방차 통합 노드')
    ap.add_argument('--transport', choices=['udp', 'esp-now'], default='esp-now')
    ap.add_argument('--serial-port', default='/dev/ttyUSB0', help='ESP32 포트')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--broadcast-addr', default='255.255.255.255')
    ap.add_argument('--udp-port', type=int, default=9000)
    ap.add_argument('--stm32-port', default='/dev/serial0')
    ap.add_argument('--stm32-baud', type=int, default=115200)
    ap.add_argument('--no-stm32', action='store_true', help='STM32 없이 실행')
    ap.add_argument('--gps', action='store_true', help='gpsd 실제 GPS 사용 (기본: 더미 위치)')
    ap.add_argument('--rate', type=float, default=10.0, help='BSM/경고 주기(Hz)')
    ap.add_argument('--buzzer-gpio', type=int, default=None)
    args = ap.parse_args()
    buzzer = None
    if args.buzzer_gpio is not None:
        from buzzer import GpioBuzzer
        buzzer = GpioBuzzer(args.buzzer_gpio)

    transport = (EspNowTransport(args.serial_port, args.baud) if args.transport == 'esp-now'
                 else UDPBroadcastTransport(args.udp_port, args.broadcast_addr))
    source = GpsdSource() if args.gps else DummySource(MY_ID)
    tracker = VehicleTracker()
    logic = FrontLogic()
    lock = threading.Lock()
    stm32 = None if args.no_stm32 else Stm32Link(args.stm32_port, args.stm32_baud, logic, lock)

    print(f'[전방차] transport={transport.name} gps={"gpsd" if args.gps else "dummy"} '
          f'stm32={"off" if stm32 is None else args.stm32_port} rate={args.rate}Hz')

    period = 1.0 / args.rate
    nxt = time.monotonic()
    seq = tx = 0
    drops = {}
    last_print = 0.0
    prev_level = 0
    try:
        while True:
            now = time.monotonic()
            fix = source.read()
            with lock:
                spd, spd_src = logic.speed(fix, now)
                brk, brk_src = logic.brake(spd, spd_src, now)
                k = logic.k_value(now)
                stm_alive = logic.stm32_alive(now)

            # ---- 1) 내 BSM 송신 ----
            status = STATUS_GPS_VALID if fix['gps_valid'] else 0
            if brk:
                status |= STATUS_BRAKE_ACTIVE
            if stm32 is not None and not stm_alive:
                status |= STATUS_SENSOR_DEGRADED     # STM32 가 끊겼음을 후방차에 알림
            m = BSM(vehicle_id=MY_ID, seq=seq, timestamp_ms=int(now * 1000),
                    lat=fix['lat'], lon=fix['lon'], speed_kmh=spd,
                    heading_deg=fix['heading'], k_value=k, status=status)
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
            if nxt < time.monotonic() - period:      # 많이 밀렸으면 기준 시각 재설정
                nxt = time.monotonic()

            # ---- 3) 후방 경고 판단 -> STM32 ----
            now = time.monotonic()
            others = tracker.live_bsms(now)
            with lock:
                level, src, dist = logic.warning(fix, spd, others, now)
                reason = logic.last_warn_reason
            if stm32 is not None:
                stm32.send_warn(level, src, dist)    # 경고 없을 때도 level 0 을 계속 보냄(STM32 쪽 생존 확인용)
            if buzzer is not None:
                buzzer.set_level(level)
            if level != prev_level:
                txt = (f'*** 경고 ON: {VEHICLE_NAMES.get(src, src)} '
                       f'{"" if dist is None else f"{dist:.1f}m "}({reason}) ***' if level == WARN_DANGER
                       else '*** 경고 OFF ***')
                print(txt)
                prev_level = level

            # ---- 4) 1초 요약 ----
            if now - last_print >= 1.0:
                last_print = now
                parts = []
                for s in tracker.summary(now):
                    bb = s['bsm']
                    parts.append(f'{VEHICLE_NAMES.get(s["vid"], s["vid"])}='
                                 f'{"LIVE" if s["live"] else "STALE"}'
                                 f'{" DANGER" if bb.danger else ""}(lost {s["lost"]})')
                print(f'--- tx={tx} speed={spd:.1f}km/h({spd_src}) brake={int(brk)}({brk_src}) '
                      f'gps={"Y" if fix["gps_valid"] else "N"} '
                      f'stm32={"-" if stm32 is None else ("OK" if stm_alive else "LOST")} '
                      f'warn={level} | ' + (' | '.join(parts) or '수신 없음')
                      + (f' drop={drops}' if drops else ''))
    except KeyboardInterrupt:
        print('\n종료합니다...')
    finally:
        if buzzer is not None:
            buzzer.close()
        if stm32 is not None:
            stm32.send_warn(0, 0, None)
            stm32.close()
        transport.close()


if __name__ == '__main__':
    main()
