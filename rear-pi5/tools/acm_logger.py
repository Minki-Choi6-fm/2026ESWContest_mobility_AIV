#!/usr/bin/env python3
"""Read-only logger for the ST-Link debug text (/dev/ttyACM0, 115200 8N1). Writes NOTHING to the port, never resets the board.
DTR/RTS are kept low; the line coding (115200) is the only thing set. Each line is saved with a host timestamp. Re-opens if the port disappears.
usage: python3 acm_logger.py OUTFILE SECONDS"""
import sys, time, serial
out = open(sys.argv[1], "a", buffering=1); end = time.time() + float(sys.argv[2]); buf = b""
while time.time() < end:
    try:
        s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.2; s.dtr = False; s.rts = False; s.open()
        out.write(f"{time.time():.3f} #LOGGER open {time.strftime('%H:%M:%S')}\n")
        while time.time() < end:
            d = s.read(4096)
            if d:
                buf += d
                while b"\n" in buf:
                    ln, buf = buf.split(b"\n", 1)
                    out.write(f"{time.time():.3f} {ln.decode('ascii','replace').rstrip()}\n")
        s.close()
    except (serial.SerialException, OSError) as e:
        out.write(f"{time.time():.3f} #LOGGER error {e}\n"); time.sleep(1)
out.write(f"{time.time():.3f} #LOGGER end\n")
