#!/usr/bin/env python3
"""drive_logger.py WITHOUT its UART reader (wrapper; the original file is imported unchanged and one function is replaced in memory).

Why: /dev/ttyAMA2 must be read by ONE program only. In the car live_receiver.py --record owns the port; this wrapper keeps everything else
of drive_logger (camera mp4 + camera_ts.csv, stm_debug.log from ttyACM0, gps.jsonl, sys.csv, meta.json, console report).
Same command line as drive_logger.py (--out, --note, --no-cam, --no-debug, --no-gps, --fps ...). --uart is accepted and ignored.
No frames.csv / radar.csv are written by this wrapper (live_receiver.py --record writes them).
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logging_tools")))
import drive_logger as dl                                                 # noqa: E402  (original, unmodified)


def _no_uart(port, baud, outdir):
    print(f"[nouart] UART {port} is NOT opened by this logger (live_receiver.py --record owns it)")
    dl.stop.wait()


_orig_reporter = dl.reporter
dl.uart_thread = _no_uart
dl.reporter = lambda outdir, expect: _orig_reporter(outdir, [e for e in expect if e != "uart_frames"])

if __name__ == "__main__":
    dl.main()
