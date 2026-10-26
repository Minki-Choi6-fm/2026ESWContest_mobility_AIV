#!/usr/bin/env python3
"""Identify USB serial metadata; optional 5-second safe BSM send (never STM32).
Serial write success is NOT proof of radio transmission or remote reception.
"""
import argparse
import json
from pathlib import Path
import sys
import time

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'v2v/v2v_stack'))


def main():
    from serial.tools import list_ports
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--send', action='store_true')
    a = ap.parse_args()
    ports = list(list_ports.comports())
    for p in ports:
        print(p.device, p.description, p.hwid)
    candidates = [p for p in ports if p.device.startswith('/dev/ttyUSB')]
    print('USB serial candidates:', len(candidates), '(USB bridge identity does not identify ESP32 chip/firmware)')
    if not a.send:
        return 0
    match = next((p for p in candidates if p.device == a.port), None)
    if match is None or match.vid != 0x10c4 or match.pid != 0xea60:
        print('REFUSED: selected port is not the expected CP2102 USB bridge; STM32 ports are never opened'); return 2
    import serial
    from bsm_protocol import encode_bsm, decode_bsm, BSMDecodeError
    from transport import EspNowTransport
    # Check occupancy before opening. Do not stop anyone else's process.
    import subprocess
    owners = subprocess.run(['fuser', a.port], capture_output=True, text=True)
    if owners.stdout.strip():
        print('REFUSED: port is already held by PID(s)', owners.stdout.strip()); return 2
    tr = EspNowTransport(a.port)
    tx = rx = bad = 0; ids = set()
    try:
        time.sleep(3)  # USB-serial open can restart ESP32, not STM32.
        start = time.monotonic(); next_tx = start; seq = int(time.time()*10)&0xffff
        while time.monotonic()-start < 5:
            now = time.monotonic()
            if now >= next_tx:
                frame = encode_bsm(vehicle_id=1,seq=seq,lat=0,lon=0,speed_kmh=0,heading_deg=0,
                                   k_value=None,gps_valid=False,danger_active=False,sensor_degraded=True)
                n = tr.ser.write(frame)
                if n != len(frame): raise RuntimeError('short serial write')
                tx += 1; seq=(seq+1)&0xffff; next_tx += .1
            frame=tr.recv_bytes(timeout=.01)
            if frame:
                try:
                    d=decode_bsm(frame); ids.add(d['vehicle_id']); rx+=1
                except BSMDecodeError: bad+=1
        tr.ser.flush()
    finally:
        tr.close()
    result=dict(serial_tx_frames=tx,valid_rx_frames=rx,bad_rx_frames=bad,peer_ids=sorted(ids),
                radio_tx_confirmed=False,warning_tested=False)
    out = WS / 'logs/codex1/esp32_probe.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result,indent=2))
    print('Saved:', out)
    print('Serial TX only; without a second ESP32, radio delivery, latency and warning remain unverified.')
    return 0


if __name__=='__main__':sys.exit(main())
