#!/usr/bin/env python3
import argparse
import csv
import re
import subprocess
import sys
import time

V_RE = re.compile(r"^\s*(\w+)\s+volt\(\d+\)=([0-9.]+)V", re.M)


def run(*cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--warn-v", type=float, default=5.0)
    a = ap.parse_args()
    out = a.out or time.strftime("power_log_%Y%m%d_%H%M%S.csv")
    cols = None
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    print(f"기록 시작: {out}  (Ctrl+C 로 중지)")
    n = 0
    try:
        while True:
            t = time.time()
            volts = {k: float(v) for k, v in V_RE.findall(run("vcgencmd", "pmic_read_adc"))}
            thr = run("vcgencmd", "get_throttled").strip().split("=")[-1]
            tmp = run("vcgencmd", "measure_temp").strip().replace("temp=", "").replace("'C", "")
            if not volts:
                print("[경고] vcgencmd pmic_read_adc 값을 읽지 못함", file=sys.stderr)
            if cols is None and volts:
                cols = sorted(volts)
                w.writerow(["time"] + cols + ["throttled", "temp_c"])
            if cols is not None:
                w.writerow([f"{t:.3f}"] + [volts.get(c, "") for c in cols] + [thr, tmp])
                f.flush()
            n += 1
            ev = volts.get("EXT5V_V")
            bad = (ev is not None and ev < a.warn_v) or (thr not in ("", "0x0"))
            if bad or n % 10 == 0:
                print(f"{time.strftime('%H:%M:%S', time.localtime(t))} EXT5V={ev} throttled={thr} temp={tmp}" + ("  !!!" if bad else ""), flush=True)
            time.sleep(max(0.0, a.interval - (time.time() - t)))
    except KeyboardInterrupt:
        print("\n중지. 저장:", out)
    finally:
        f.close()


if __name__ == "__main__":
    main()
