#!/usr/bin/env python3
"""STM32 silence watcher (READ-ONLY). Watches two files that grow while the STM32 talks: the pipeline's rec/frames.csv (0x10..0x14 frames from /dev/ttyAMA2)
and acm_debug.log (ST-Link debug text). If EITHER stops growing for SILENCE_S (default 3 s) it is a silence event.
On a silence event, BEFORE anything else, it runs the agreed READ-ONLY OpenOCD memory dump (no reset, no halt, no resume), saves it, then records pmic rails, USB, processes, dmesg tail.
It never resets, halts, resumes or flashes anything. It prints one line per event so a Monitor can pick it up.
usage: python3 silence_watch.py RUN_DIR SECONDS [SILENCE_S]"""
import os, subprocess, sys, time

run, total = sys.argv[1], float(sys.argv[2]); sil = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
frames, acm = os.path.join(run, "rec", "frames.csv"), os.path.join(run, "acm_debug.log")
OPENOCD = ["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg", "-c", "reset_config none", "-c", "init", "-c", "mdw 0x2001E000 40",
           "-c", "mdw 0x40023874 1", "-c", "mdw 0xE000E010 3", "-c", "mdw 0x40003000 3", "-c", "mdw 0x40023830 1", "-c", "mdw 0xE000EDF0 1", "-c", "mdw 0xE000EDFC 1", "-c", "shutdown"]


def size(p):
    try: return os.path.getsize(p)
    except OSError: return -1


def sh(cmd, timeout=40):
    try: return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=isinstance(cmd, str)).stdout
    except Exception as e: return f"(error {e})"


def event(n):
    stamp = time.strftime("%H%M%S"); d = os.path.join(run, f"silence_{n}_{stamp}"); os.makedirs(d, exist_ok=True)
    t0 = time.time()
    out = subprocess.run(OPENOCD, capture_output=True, text=True, timeout=60)           # FIRST: read-only dump
    open(os.path.join(d, "openocd_read.txt"), "w").write(out.stdout + out.stderr)
    open(os.path.join(d, "pmic.txt"), "w").write(sh(["vcgencmd", "pmic_read_adc"]) + "\nthrottled " + sh(["vcgencmd", "get_throttled"]) + "temp " + sh(["vcgencmd", "measure_temp"]))
    open(os.path.join(d, "usb_procs.txt"), "w").write(sh(["lsusb"]) + "\n" + sh("lsusb -t") + "\n" + sh("pgrep -af 'pipeline_main|camera_node|acm_logger|power_sampler' | grep -v pgrep") + "\n--- dmesg tail\n" + sh("dmesg -T | tail -30"))
    mem = [l for l in out.stdout.splitlines() if l.startswith("0x")]
    return d, time.time() - t0, mem[:3]


def main():
    end = time.time() + total; last = {frames: (size(frames), time.time()), acm: (size(acm), time.time())}
    state = {frames: "ok", acm: "ok"}; n = 0; start = time.time()
    print(f"WATCH start {time.strftime('%H:%M:%S')} silence>{sil}s files: {frames}, {acm}", flush=True)
    while time.time() < end:
        time.sleep(0.5); now = time.time(); stalled = []
        for p in (frames, acm):
            s = size(p)
            if s != last[p][0]:
                last[p] = (s, now)
                if state[p] == "silent":
                    print(f"RECOVERED {os.path.basename(p)} {time.strftime('%H:%M:%S')}", flush=True); state[p] = "ok"
            elif now - last[p][1] >= sil and now - start > 20 and state[p] == "ok":
                stalled.append(p); state[p] = "silent"
        if stalled:
            n += 1; since = time.strftime("%H:%M:%S", time.localtime(max(last[p][1] for p in stalled)))
            print(f"!!! SILENCE #{n} detected {time.strftime('%H:%M:%S')} (last data {since}) in {[os.path.basename(p) for p in stalled]} -> running read-only OpenOCD dump", flush=True)
            d, dt, mem = event(n)
            print(f"!!! SILENCE #{n} dump saved {d} ({dt:.1f}s). 0x2001E000..: {mem}", flush=True)
    print(f"WATCH end {time.strftime('%H:%M:%S')} events={n}", flush=True)


if __name__ == "__main__":
    main()
