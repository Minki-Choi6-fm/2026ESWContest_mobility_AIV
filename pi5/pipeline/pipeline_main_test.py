#!/usr/bin/env python3
"""Test of pipeline_main.py with fake_stm32 + fake_gpsd + a fixed front-car node (bsm_node.py). SIMULATION ONLY, nothing real is touched.
Checks: normal run, injected crash, SIGTERM, main-loop hang (watchdog) all end with the outputs in SAFE state; the UART fd is O_RDONLY; sinks keep working
when one fails; the buzzer/OLED attach point refuses to be constructed."""
import csv, os, re, signal, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
DRIVE = os.path.join(WS, "logs", "vehicle_test_logs", "drive_20261016_152211")
OUT = os.path.join(WS, "logs", "pipeline_main_test", time.strftime("%Y%m%d_%H%M%S"))
PY = [sys.executable, "-I"]


def scenario(name, extra, kill_after=None, dur=10):
    od = os.path.join(OUT, name); os.makedirs(od, exist_ok=True)
    link = os.path.join(od, "uart.tty")
    bg = lambda args, log: subprocess.Popen(PY + args, stdout=open(os.path.join(od, log), "w"), stderr=subprocess.STDOUT)
    procs = [bg([os.path.join(HERE, "fake_stm32.py"), DRIVE, "--link", link, "--duration", str(dur + 12)], "fake_stm32.log"),
             bg([os.path.join(HERE, "fake_gpsd.py"), DRIVE, "--port", "22947", "--duration", str(dur + 12)], "fake_gpsd.log")]
    time.sleep(2.0)
    procs.append(bg([os.path.join(HERE, "bsm_node.py"), "--id", "2", "--source", "fixed", "--duration", str(dur + 4), "--out", od, "--fixed-k", "0.6"], "front.log"))
    try:
        pm = subprocess.Popen(PY + [os.path.join(HERE, "pipeline_main.py"), "--uart", link, "--gpsd-port", "22947", "--port", "19000", "--out", od,
                                    "--duration", str(dur)] + extra, stdout=open(os.path.join(od, "pipeline.log"), "w"), stderr=subprocess.STDOUT)
        fd_mode = None
        time.sleep(3.0)
        try:                                                    # access mode of the UART fd of the running process
            for fdn in os.listdir(f"/proc/{pm.pid}/fd"):
                try:
                    if os.readlink(f"/proc/{pm.pid}/fd/{fdn}").startswith("/dev/pts/"):
                        flags = int([l for l in open(f"/proc/{pm.pid}/fdinfo/{fdn}") if l.startswith("flags")][0].split()[1], 8)
                        fd_mode = flags & 3
                except OSError:
                    pass
        except OSError:
            pass
        if kill_after:
            time.sleep(max(0.0, kill_after - 3.0)); pm.send_signal(signal.SIGTERM)
        try:
            rc = pm.wait(timeout=dur + 20)
        except subprocess.TimeoutExpired:
            pm.kill(); rc = "TIMEOUT"
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
    rows = list(csv.DictReader(open(os.path.join(od, "output_sink.csv")))) if os.path.exists(os.path.join(od, "output_sink.csv")) else []
    log = open(os.path.join(od, "pipeline.log")).read()
    return od, rc, rows, log, fd_mode


def main():
    os.makedirs(OUT, exist_ok=True)
    res = []
    ok = lambda c: "PASS" if c else "FAIL"
    BZ = ["--buzzer", "mock"]                                       # gpiozero MockFactory: no real pin is touched
    od, rc, rows, log, fdm = scenario("normal", BZ + ["--force-level2-at", "2", "--force-level2-for", "4"])
    ups = [r for r in rows if r["event"] == "UPDATE"]
    tx = sum(1 for _ in open(os.path.join(od, "bsm_tx.csv"))) - 1
    res.append(("normal run (10 s)", f"exit {rc}; {len(ups)} output updates (~20/s); BSM sent {tx}; last sink event = {rows[-1]['event'] if rows else '-'} ('{rows[-1]['reason'] if rows else ''}'); UART fd access mode = {'O_RDONLY' if fdm == 0 else fdm}",
                ok(rc == 0 and len(ups) > 150 and rows and rows[-1]["event"] == "SAFE_STATE" and fdm == 0 and tx > 60)))
    m = re.search(r"\[buzzer\] final: pin (ON|OFF), on/off transitions (\d+) \((MOCK pin)", log)
    res.append(("normal run + mock buzzer, level 2 injected for 4 s", f"buzzer line: {m.group(0) if m else 'MISSING'}", ok(m and m.group(1) == "OFF" and int(m.group(2)) >= 8)))
    od, rc, rows, log, _ = scenario("crash", ["--crash-at", "4"] + BZ + ["--force-level2-at", "1", "--force-level2-for", "9"])
    res.append(("injected exception at 4 s", f"exit {rc}; last sink event = {rows[-1]['event'] if rows else '-'} ('{rows[-1]['reason'] if rows else ''}')",
                ok(rc == 1 and rows and rows[-1]["event"] == "SAFE_STATE" and "RuntimeError" in rows[-1]["reason"])))
    m = re.search(r"\[buzzer\] final: pin (ON|OFF), on/off transitions (\d+)", log)
    res.append(("crash while the (mock) buzzer is ringing", f"buzzer line: {m.group(0) if m else 'MISSING'}", ok(m and m.group(1) == "OFF" and int(m.group(2)) >= 4)))
    od, rc, rows, log, _ = scenario("sigterm", BZ + ["--force-level2-at", "1", "--force-level2-for", "20"], kill_after=6, dur=30)
    res.append(("SIGTERM at 6 s", f"exit {rc}; last sink event = {rows[-1]['event'] if rows else '-'} ('{rows[-1]['reason'] if rows else ''}')",
                ok(rc == 0 and rows and rows[-1]["event"] == "SAFE_STATE" and "SIGTERM" in rows[-1]["reason"])))
    m = re.search(r"\[buzzer\] final: pin (ON|OFF), on/off transitions (\d+)", log)
    res.append(("SIGTERM while the (mock) buzzer is ringing", f"buzzer line: {m.group(0) if m else 'MISSING'}", ok(m and m.group(1) == "OFF" and int(m.group(2)) >= 4)))
    od, rc, rows, log, _ = scenario("hang", ["--hang-at", "4", "--hang-for", "3"] + BZ + ["--force-level2-at", "1", "--force-level2-for", "5"], dur=12)
    wd = [r for r in rows if r["event"] == "SAFE_STATE" and "watchdog" in r["reason"]]
    after = [r for r in rows if r["event"] == "UPDATE" and wd and float(r["t"]) > float(wd[0]["t"])]
    res.append(("main loop blocked 3 s at 4 s", f"exit {rc}; watchdog SAFE_STATE while blocked: {len(wd)}; updates resumed afterwards: {len(after)}; last event = {rows[-1]['event'] if rows else '-'}",
                ok(rc == 0 and len(wd) == 1 and len(after) > 20 and rows[-1]["event"] == "SAFE_STATE")))
    m = re.search(r"\[buzzer\] final: pin (ON|OFF), on/off transitions (\d+)", log)
    res.append(("hang (main loop blocked 3 s) with the (mock) buzzer ringing", f"buzzer line: {m.group(0) if m else 'MISSING'}", ok(m and m.group(1) == "OFF" and int(m.group(2)) >= 4)))
    # output policy (unit): caution = display only, danger = buzzer candidate, unknown = "?" and never a buzzer
    sys.path.insert(0, HERE)
    import pipeline_main as PM
    import tempfile, constants as CC
    tp = os.path.join(tempfile.mkdtemp(), "o.csv")
    cs = PM.CsvSink(tp); hub = PM.SinkHub([cs])
    for lv in (0, 1, 2, 0xFF):
        hub.update(1.0, lv, "test", {})
    hub.safe_state("done"); cs.close()
    pr = list(csv.DictReader(open(tp)))
    flags = [(r["level"], r["display"], r["buzzer_candidate"]) for r in pr if r["event"] == "UPDATE"]
    CC.BUZZER_MIN_LEVEL = 1
    try:
        unk_ok = not PM.output_flags(0xFF)[1] and PM.output_flags(1)[1]       # even with the threshold lowered to 1, unknown never sounds
    finally:
        CC.BUZZER_MIN_LEVEL = 2
    res.append(("output policy (caution display-only, danger buzzer candidate, unknown '?' never sounds)", f"{flags}; last = {pr[-1]['event']}; unknown never sounds even at BUZZER_MIN_LEVEL=1: {unk_ok}",
                ok(flags == [("0", "-", "0"), ("1", "!", "0"), ("2", "!!", "1"), ("255", "?", "0")] and pr[-1]["event"] == "SAFE_STATE" and unk_ok)))
    r = subprocess.run(PY + [os.path.join(HERE, "pipeline_main.py"), "--sinks", "console,buzzer", "--out", os.path.join(OUT, "buzzer"), "--duration", "1"], capture_output=True, text=True)
    rr = subprocess.run(PY + [os.path.join(HERE, "pipeline_main.py"), "--buzzer", "real", "--out", os.path.join(OUT, "refuse"), "--duration", "1"], capture_output=True, text=True)
    res.append(("--buzzer real WITHOUT --really-drive-gpio26 is refused (no pin touched, nothing started)", f"exit {rr.returncode}; {rr.stderr.strip()[:90]}", ok(rr.returncode == 2 and "REFUSED" in rr.stderr)))
    res.append(("buzzer/OLED attach point", f"exit {r.returncode}; refuses: {'NotImplementedError' in r.stderr}", ok(r.returncode != 0 and "NotImplementedError" in r.stderr)))
    print("| test | result | verdict |\n|---|---|---|")
    for n, a, v in res:
        print(f"| {n} | {a} | {v} |")
    print(f"\nlogs -> {OUT}")
    sys.exit(0 if all(v == "PASS" for *_, v in res) else 1)


if __name__ == "__main__":
    main()
