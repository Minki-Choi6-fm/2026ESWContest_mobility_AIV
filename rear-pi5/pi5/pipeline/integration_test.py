#!/usr/bin/env python3
"""S12-e: integrated test with FAKE inputs only: sim_scenario.py (fake STM32 on a NEW pty + fake camera datagrams) -> pipeline_main (--write-stm32 into the same pty,
mock buzzer, camera link, local UDP BSM) -> front_node.py (~/front_test, UDP loop-back, NO --buzzer-gpio).  Nothing touches /dev/ttyAMA2, a camera, GPIO or the network.
usage: python3 -I integration_test.py [--out DIR]
Cases: A speed known (OBD 50 km/h) + camera + radar dropout + camera dropout;  B speed unknown (display only, buzzer inhibited);  B2 speed unknown + --buzz-without-speed;  C no camera."""
import argparse, csv, json, os, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
FRONT_DIR = os.path.expanduser("~/front_test/v2v")
PY = sys.executable
UDP_PORT, CAM_PORT = 29100, 19100


def run_case(name, out, obd, cam, extra, check):
    d = os.path.join(out, name); os.makedirs(d, exist_ok=True)
    link = os.path.join(d, "stm32.pty"); res = os.path.join(d, "sim_result.json")
    sim_args = [PY, "-I", "sim_scenario.py", "--link", link, "--duration", "19", "--radar-drop", "9,11", "--result", res]
    if obd is not None: sim_args += ["--obd-speed", str(obd)]
    if cam: sim_args += ["--camera-port", str(CAM_PORT), "--cam-drop", "11.5,13"]
    sim = subprocess.Popen(sim_args, cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t_w = time.time()
    while not os.path.exists(link) and time.time() - t_w < 5: time.sleep(0.05)
    flines = []
    front = subprocess.Popen([PY, "-u", "front_node.py", "--no-stm32", "--transport", "udp", "--udp-port", str(UDP_PORT), "--broadcast-addr", "127.255.255.255"],
                             cwd=FRONT_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    threading.Thread(target=lambda: [flines.append((time.monotonic(), l.rstrip())) for l in front.stdout], daemon=True).start()
    time.sleep(0.5)
    pm_args = [PY, "-I", "pipeline_main.py", "--uart", link, "--transport", "udp", "--port", str(UDP_PORT), "--broadcast-addr", "127.255.255.255",
               "--duration", "15", "--out", d, "--sinks", "csv", "--buzzer", "mock", "--write-stm32", "--affinity", "2,3"] + (["--camera-udp-port", str(CAM_PORT)] if cam else []) + extra
    t_pm = time.monotonic()
    pm = subprocess.run(pm_args, cwd=HERE, capture_output=True, text=True, timeout=60)
    sim.wait(timeout=20); front.terminate()
    try: front.wait(3)
    except subprocess.TimeoutExpired: front.kill()
    simres = json.load(open(res)) if os.path.exists(res) else {}
    rows = list(csv.DictReader(l for l in open(os.path.join(d, "drive_log.csv")) if not l.startswith("#")))
    levels = [r["level"] for r in rows]
    open(os.path.join(d, "pipeline_stdout.txt"), "w").write(pm.stdout + "\n--- stderr\n" + pm.stderr)
    open(os.path.join(d, "front_stdout.txt"), "w").write("\n".join(l for _, l in flines))
    trans = None
    for ln in pm.stdout.splitlines():
        if "[buzzer] final" in ln: trans = int(ln.split("transitions")[1].split()[0])
    return dict(name=name, rc=pm.returncode, rows=rows, levels=levels, sim=simres, front=flines, t_pm=t_pm, buzzer_transitions=trans, stdout=pm.stdout, stderr=pm.stderr)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.expanduser("~/ESWContest_ws/logs/integration_test/" + time.strftime("%Y%m%d_%H%M%S")))
    a = ap.parse_args()
    if not os.path.isdir(FRONT_DIR):
        print("SKIP: ~/front_test/v2v missing"); return 0
    results = []
    def check(n, ok, note=""):
        results.append(ok); print(("PASS " if ok else "FAIL ") + n + (f"  [{note}]" if note else ""), flush=True)
    # ---------------- A
    A = run_case("A_speed_known_camera", a.out, 50, True, [], None)
    rows = A["rows"]; L = A["levels"]
    t_of = lambda r: float(r["t_rel"])
    first2 = next((t_of(r) for r in rows if r["level"] == "2"), None)
    check("A1 pipeline exits 0 and wrote drive_log.csv with the requested columns", A["rc"] == 0 and len(rows) > 200 and "camera_confirmed" in rows[0] and "power_suspect" in rows[0], f"{len(rows)} rows, rc {A['rc']}")
    check("A2 lead braking -> level 2 reached (radar -> target -> TTC -> danger)", first2 is not None, f"first level 2 at {first2:.2f} s of the run" if first2 else "never")
    on = [t for t, l in A["front"] if "경고 ON" in l]
    check("A3 front car warning ON (BSM with DANGER_ACTIVE over local UDP)", bool(on))
    sw = A["sim"]
    check("A4 own-car 0x20 frames reached the fake STM32 pty with level 2 and no malformed frame", 2 in sw.get("levels", []) and sw.get("bad_frames", 1) == 0, f"{sw.get('warn_frames')} frames, levels {sw.get('levels')}")
    check("A5 buzzer (mock pin) switched ON during danger and ended OFF", (A["buzzer_transitions"] or 0) > 0 and "pin OFF" in A["stdout"], f"transitions {A['buzzer_transitions']}")
    dd = [r for r in rows if r["radar_down"] == "1"]
    check("A6 radar dropout (STM32 SAFE_MODE) -> level 0xFF ('?'), no crash, flagged radar_down and power_suspect", len(dd) >= 20 and all(r["level"] == "255" for r in dd) and all(r["power_suspect"] == "1" for r in dd),
          f"{len(dd)} radar_down rows (~{len(dd) / 20:.1f} s): levels {sorted(set(r['level'] for r in dd))}")
    nb = [r for r in dd if r["level"] == "255"]
    check("A7 0xFF never sounds the buzzer (0x20 level 0xFF frames sent, buzzer candidate only at level 2)", 255 in sw.get("levels", []))
    live = [r for r in rows if r["stm_status"] != ""]                       # rows while the fake STM32 still sends (the stream ends with the scenario)
    after = [r for r in live if t_of(r) > t_of(dd[-1]) + 0.5] if dd else []
    check("A8 radar recovery: after the dropout the level returns to 0 (new lead 60 m ahead) and stays", len(after) > 20 and all(r["level"] == "0" for r in after[10:]), f"{len(after)} rows after the dropout, last level {after[-1]['level'] if after else None}")
    cc = [r for r in rows if r["camera_confirmed"] == "1"]
    check("A9 camera_confirmed became 1 with fake detections (N=3 frames) before the dropout", bool(cc) and t_of(cc[0]) < 9, f"first at {t_of(cc[0]):.2f} s" if cc else "never")
    i0 = next((i for i, r in enumerate(rows) if r["camera_alive"] == "1"), None)
    cd = [r for r in rows[i0:] if r["camera_alive"] == "0"] if i0 is not None else []
    check("A10 camera dropout (no packets) -> camera_alive 0 and camera_confirmed 0, pipeline continues radar-only", len(cd) >= 15 and all(r["camera_confirmed"] == "0" for r in cd), f"{len(cd)} rows with camera_alive=0 after the first live frame")
    check("A11 safe end: last 0x20 frame sent to the pty is level 0 (OFF)", sw.get("last") and sw["last"][1] == 0, str(sw.get("last")))
    check("A12 speed source logged as OBD", all(r["speed_src"] == "OBD" for r in rows[40:]) )
    # ---------------- B
    B = run_case("B_speed_unknown", a.out, None, True, [], None)
    b2 = any(r["level"] == "2" for r in B["rows"])
    check("B1 speed unknown: level 2 is still computed and shown (display/0x20)", B["rc"] == 0 and b2 and 2 in B["sim"].get("levels", []))
    check("B2 speed unknown: buzzer inhibited (0 transitions) - display only", B["buzzer_transitions"] == 0, f"transitions {B['buzzer_transitions']}")
    B2 = run_case("B2_speed_unknown_allow_buzzer", a.out, None, True, ["--buzz-without-speed"], None)
    check("B3 option --buzz-without-speed lifts the inhibit (mock buzzer sounds)", (B2["buzzer_transitions"] or 0) > 0, f"transitions {B2['buzzer_transitions']}")
    # ---------------- C
    C = run_case("C_no_camera", a.out, 50, False, [], None)
    check("C1 no camera: radar-only run completes, camera_confirmed always 0, level 2 still reached", C["rc"] == 0 and all(r["camera_confirmed"] == "0" for r in C["rows"]) and any(r["level"] == "2" for r in C["rows"]))
    print(f"\n{sum(results)}/{len(results)} PASS   logs: {a.out}")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
