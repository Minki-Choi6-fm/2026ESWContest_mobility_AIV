#!/usr/bin/env python3
"""Pi5 -> STM32 proposed v0.2 EGO_SPEED (0x22) transmitter.

Default: continuous UART send. Use --dry-run to inspect frames without writing.
GPS data come from gpsd TCP JSON (2947).
No OBD source is implemented here; central get_speed() is the only source selector.
Protocol is PROPOSED, not confirmed adopted by STM32 firmware.
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

STX, ETX, TYPE, LENGTH = 0x02, 0x03, 0x22, 5
SOURCE_OBD, SOURCE_GPS, SOURCE_NONE = 0, 1, 0xFF


@dataclass
class SpeedMeasurement:
    speed_kmh: float
    source: int
    received_at: float  # monotonic time at reception of latest valid gpsd TPV


class GpsdReader:
    def __init__(self, host: str, port: int):
        self.host, self.port = host, port
        self.sock: Optional[socket.socket] = None
        self.last: Optional[SpeedMeasurement] = None
        self.last_retry = 0.0
        self._buffer = b''

    def connect(self):
        if self.sock is not None or time.monotonic() - self.last_retry < 1.0:
            return
        self.last_retry = time.monotonic()
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.5)
            s.settimeout(0.2)
            s.sendall(b'?WATCH={"enable":true,"json":true};\n')
            self.sock = s
        except OSError as e:
            print(f'[gpsd] 연결 대기: {e}', file=sys.stderr)

    def poll(self, deadline: float):
        self.connect()
        if self.sock is None:
            return
        while time.monotonic() < deadline:
            try:
                self.sock.settimeout(min(0.1, max(0.01, deadline - time.monotonic())))
                # Avoid socket.makefile buffering/timeouts: read raw socket bytes.
                data = self.sock.recv(4096)
                if not data:
                    raise ConnectionError('gpsd disconnected')
                self._buffer += data
                while b'\n' in self._buffer:
                    line, self._buffer = self._buffer.split(b'\n', 1)
                    try:
                        msg = json.loads(line)
                    except (ValueError, UnicodeError):
                        continue
                    if msg.get('class') != 'TPV' or msg.get('mode', 0) < 2:
                        continue
                    speed = msg.get('speed')
                    if not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed < 0:
                        continue
                    self.last = SpeedMeasurement(speed * 3.6, SOURCE_GPS, time.monotonic())
            except socket.timeout:
                return
            except (OSError, ConnectionError) as e:
                print(f'[gpsd] 연결 끊김: {e}', file=sys.stderr)
                self.close()
                return

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self._buffer = b''


def get_speed(gps: GpsdReader, max_age_ms: int) -> tuple[int, int, int]:
    """Sole source selection: GPS only in this implementation; stale => no source.

    Future OBD integration must happen here, not in transmit or packetization.
    """
    now = time.monotonic()
    if gps.last is not None:
        age = max(0, int((now - gps.last.received_at) * 1000))
        if age <= max_age_ms and 0 <= gps.last.speed_kmh <= 655.35:
            return round(gps.last.speed_kmh * 100), SOURCE_GPS, min(age, 65535)
    return 0, SOURCE_NONE, 65535  # sentinel is local test choice, requires firmware agreement


def build_frame(speed_centikmh: int, source: int, age_ms: int) -> bytes:
    if not 0 <= speed_centikmh <= 65535:
        raise ValueError('speed out of uint16 range')
    if source not in (SOURCE_OBD, SOURCE_GPS, SOURCE_NONE):
        raise ValueError('unknown speed source')
    if not 0 <= age_ms <= 65535:
        raise ValueError('age out of uint16 range')
    payload = struct.pack('<HBH', speed_centikmh, source, age_ms)
    checksum = TYPE ^ LENGTH
    for b in payload:
        checksum ^= b
    return bytes((STX, TYPE, LENGTH)) + payload + bytes((checksum, ETX))


def main():
    p = argparse.ArgumentParser(description='Proposed v0.2 0x22 EGO_SPEED transmitter; UART send by default')
    p.add_argument('--port', default='/dev/ttyAMA2', help='STM32 UART device')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--gpsd-host', default='127.0.0.1')
    p.add_argument('--gpsd-port', type=int, default=2947)
    p.add_argument('--hz', type=float, default=5)
    p.add_argument('--max-age-ms', type=int, default=1500)
    p.add_argument('--dry-run', action='store_true', help='Do not write UART; only display frames')
    p.add_argument('--send', action='store_true', help=argparse.SUPPRESS)  # backward compatibility
    p.add_argument('--count', type=int, default=0, help='number of frames; 0=continuous (default)')
    args = p.parse_args()
    if not 0 < args.hz <= 50 or not 0 < args.max_age_ms <= 65535 or args.count < 0:
        p.error('require 0<hz<=50, 0<max-age-ms<=65535 and count>=0')
    serial_port = None
    if not args.dry_run:
        try:
            import serial
            serial_port = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2)
        except (ImportError, OSError) as e:
            p.error(f'UART open failed: {e}')
    gps = GpsdReader(args.gpsd_host, args.gpsd_port)
    period = 1 / args.hz
    print(f"mode={'UART SEND' if not args.dry_run else 'DRY RUN'} port={args.port} baud={args.baud} rate={args.hz}Hz")
    try:
        idx = 0
        while args.count == 0 or idx < args.count:
            start = time.monotonic()
            gps.poll(start + period * 0.7)
            value, source, age = get_speed(gps, args.max_age_ms)
            frame = build_frame(value, source, age)
            if serial_port is not None:
                n = serial_port.write(frame)
                if n != len(frame):
                    raise RuntimeError(f'partial UART write {n}/{len(frame)}')
            print(f'{idx+1:04d} SPD={value/100:.2f}km/h source={"GPS" if source==1 else "NONE"} '
                  f'age_ms={age} frame={frame.hex(" ")}')
            idx += 1
            remain = period - (time.monotonic() - start)
            if remain > 0:
                time.sleep(remain)
    except KeyboardInterrupt:
        pass
    finally:
        gps.close()
        if serial_port is not None:
            serial_port.close()


if __name__ == '__main__':
    main()
