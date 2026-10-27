#!/usr/bin/env python3
"""Read the observed STM32/serial status lines: `TX=42665 | RX none`.

This is a read-only diagnostic receiver, NOT an MR76 RADAR_OBJECT decoder.
"""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import re
import time
import serial

PATTERN = re.compile(rb'^\s*TX\s*=\s*(\d+)\s*\|\s*RX\s*(?:=|\s)\s*(none|-?\d+)\s*$', re.IGNORECASE)


def parse_line(raw: bytes):
    match = PATTERN.fullmatch(raw.strip())
    if not match:
        return None
    tx = int(match.group(1))
    rx_text = match.group(2).decode('ascii').lower()
    return tx, (None if rx_text == 'none' else int(rx_text))


def main():
    parser = argparse.ArgumentParser(description='Read TX=... | RX none UART status messages (not radar values)')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=10.0, help='0: run until Ctrl+C')
    parser.add_argument('--record-dir', default='~/vehicle_test_logs')
    parser.add_argument('--print-every', type=float, default=1.0, help='minimum seconds between console updates')
    args = parser.parse_args()
    if args.baud <= 0 or args.seconds < 0 or args.print_every <= 0:
        parser.error('baud and print-every must be positive; seconds must be >= 0')

    out_dir = Path(args.record_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    file_path = out_dir / f'uart_status_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    lines = valid = invalid = rx_none = 0
    latest = None
    start = time.monotonic()
    next_print = start
    end = start + args.seconds if args.seconds else float('inf')

    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.2, exclusive=True) as ser, \
                file_path.open('w', newline='', encoding='utf-8') as fp:
            writer = csv.writer(fp)
            writer.writerow(['utc_time', 'elapsed_s', 'tx_raw', 'rx_raw', 'valid', 'raw_text'])
            print(f'[UART] Read-only monitor: {args.port} @ {args.baud} (8N1)')
            print('[UART] Parsing status text only. Ctrl+C to stop.')
            # Bound reads per line so malformed/no-newline input cannot grow without limit.
            while time.monotonic() < end:
                raw = ser.read_until(b'\n', size=256)
                if not raw:
                    continue
                lines += 1
                parsed = parse_line(raw)
                elapsed = time.monotonic() - start
                raw_text = raw.decode('ascii', errors='replace').strip()
                if parsed is None:
                    invalid += 1
                    writer.writerow([datetime.now(timezone.utc).isoformat(), f'{elapsed:.3f}', '', '', 0, raw_text])
                else:
                    valid += 1
                    tx, rx = parsed
                    rx_none += (rx is None)
                    latest = (tx, rx)
                    writer.writerow([datetime.now(timezone.utc).isoformat(), f'{elapsed:.3f}', tx, '' if rx is None else rx, 1, raw_text])
                if time.monotonic() >= next_print:
                    if latest is None:
                        print(f'[STATUS] lines={lines} valid={valid} invalid={invalid} (no valid status yet)')
                    else:
                        tx, rx = latest
                        print(f'[STATUS] TX(raw)={tx} RX(raw)={"none" if rx is None else rx} | valid={valid} invalid={invalid}')
                    next_print = time.monotonic() + args.print_every
                if lines % 100 == 0:
                    fp.flush()
    except KeyboardInterrupt:
        print('\n[UART] Stopped by user.')
    except (serial.SerialException, OSError) as exc:
        raise SystemExit(f'[ERROR] Serial/file error: {exc}') from exc
    print(f'[DONE] lines={lines}, valid={valid}, invalid={invalid}, RX-none={rx_none}')
    print(f'[DONE] CSV: {file_path}')
    print('[NOTE] TX/RX semantics are not defined; no radar measurements were decoded.')


if __name__ == '__main__':
    main()
