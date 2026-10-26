#!/usr/bin/env python3
"""S17-A: run every scenario of scenario_lib.py through the REAL pipeline_main (fake STM32 on a pty, --write-stm32 into that pty only, mock buzzer, local UDP BSM)
and the front car's front_node.py (~/front_test/v2v, loop-back UDP 127.255.255.255:29200, NO --buzzer-gpio). Reports expected vs actual level, BSM danger_active,
front warning, buzzer (mock) transitions. Nothing real is touched. usage: python3 -I scenario_suite.py [--only N,M] [--out DIR]"""
import argparse, collections, csv, json, os, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import scenario_lib
FRONT = os.path.expanduser("~/front_test/v2v"); PY = sys.executable; PORT = 29200


def run_one(i, sc, out):
    d = os.path.join(out, f"s{i}"); os.makedirs(d, exist_ok=True); link = os.path.join(d, "stm32.pty"); res = os.path.join(d, "sim.json")
    sim = subprocess.Popen([PY, "-I", "sim_suite.py", "--link", link, "--index", str(i), "--result", res], cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    t = time.time()
    while not os.path.exists(link) and time.time() - t < 5: time.sleep(0.05)
    fl = []
    fr = subprocess.Popen([PY, "-u", "front_node.py", "--no-stm32", "--transport", "udp", "--udp-port", str(PORT), "--broadcast-addr", "127.255.255.255"],
                          cwd=FRONT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    threading.Thread(target=lambda: [fl.append((time.monotonic(), l.rstrip())) for l in fr.stdout], daemon=True).start()
    time.sleep(0.5)
    pm = subprocess.run([PY, "-I", "pipeline_main.py", "--uart", link, "--transport", "udp", "--port", str(PORT), "--broadcast-addr", "127.255.255.255", "--duration", str(sc["dur"] + 1),
                         "--out", d, "--sinks", "csv", "--buzzer", "mock", "--write-stm32", "--affinity", "2,3"], cwd=HERE, capture_output=True, text=True, timeout=120)
    sim.wait(timeout=30); fr.terminate()
    try: fr.wait(3)
    except subprocess.TimeoutExpired: fr.kill()
    rows = list(csv.DictReader(l for l in open(os.path.join(d, "drive_log.csv")) if not l.startswith("#")))
    bsm = list(csv.DictReader(open(os.path.join(d, "bsm_tx.csv"))))
    trans = None
    for ln in pm.stdout.splitlines():
        if "[buzzer] final" in ln: trans = int(ln.split("transitions")[1].split()[0])
    open(os.path.join(d, "front.txt"), "w").write("\n".join(l for _, l in fl)); open(os.path.join(d, "pipeline_stdout.txt"), "w").write(pm.stdout + pm.stderr)
    # trim the part where the fake stream has already ended (after sc.dur + startup)
    live = [r for r in rows if r["stm_status"] != ""]
    lv = collections.Counter(r["level"] for r in live)
    on = [l for _, l in fl if "경고 ON" in l]
    return dict(rc=pm.returncode, levels=dict(lv), live=live, n_bsm_danger=sum(x["danger_active"] == "1" for x in bsm), front_on=len(on), front_w2=any("warn=2" in l for _, l in fl),
                buzzer=trans, sim=json.load(open(res)) if os.path.exists(res) else {})


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--only"); ap.add_argument("--out", default=os.path.expanduser("~/ESWContest_ws/logs/scenario_suite/" + time.strftime("%Y%m%d_%H%M%S")))
    a = ap.parse_args(); allsc = scenario_lib.S(); idx = [int(x) for x in a.only.split(",")] if a.only else range(len(allsc))
    rows = []
    for i in idx:
        sc = allsc[i]; r = run_one(i, sc, a.out); ex = sc["expect"]
        got = {int(k) for k in r["levels"]}
        ok_allowed = got <= ex["allowed"]; ok_req = ex["required"] <= got
        ok_b = True if ex.get("danger_bsm") is None else ((r["n_bsm_danger"] > 0) == ex["danger_bsm"])
        ok_buz = True
        if ex.get("buzzer_must_be_off"): ok_buz = (r["buzzer"] == 0)
        ok_end = True
        if "end_level" in ex: ok_end = r["live"][-1]["level"] == str(ex["end_level"]) if r["live"] else False
        verdict = "일치" if (ok_allowed and ok_req and ok_b and ok_buz and ok_end and r["rc"] == 0) else "어긋남"
        rows.append((sc["name"], sc["desc"], ex, r, got, verdict)); print(f"[{verdict}] {sc['name']}: levels {r['levels']}, BSM danger rows {r['n_bsm_danger']}, front ON {r['front_on']}, buzzer {r['buzzer']}, rc {r['rc']}", flush=True)
    json.dump([dict(name=n, levels=r["levels"], bsm_danger=r["n_bsm_danger"], front_on=r["front_on"], buzzer=r["buzzer"], verdict=v, note=ex.get("note", "")) for n, d, ex, r, g, v in rows],
              open(os.path.join(a.out, "summary.json"), "w"), ensure_ascii=False, indent=1)
    print("logs:", a.out); return 0 if all(v == "일치" for *_, v in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
