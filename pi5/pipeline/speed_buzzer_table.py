#!/usr/bin/env python3
"""S17-B: behaviour table "own speed known/unknown x level 1/2" for the mock buzzer and the big screen. TEST ONLY: fake speeds come from the test option
pipeline_main.py --test-speed-kmh (default OFF; never set it in a real drive). Everything is mock: no GPIO, no serial port.
Part 1 (pure logic): danger.assess for a closing target at several speeds + the output policy (buzzer candidate / inhibit) + big-screen symbol.
Part 2 (end-to-end): sim_suite scenario 'B' (lead brakes, STM32 sends no speed) through pipeline_main --test-speed-kmh X; counts mock-buzzer transitions and levels.
usage: python3 -I speed_buzzer_table.py [--e2e]"""
import io, os, subprocess, sys, csv, collections, time

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import danger as D, constants as C
from pipeline_main import output_flags
import big_console_sink as B
import scenario_lib


class T:
    def __init__(self, d, vrel, dyn=0): self.long_m, self.vrel_ms, self.dyn = d, vrel, dyn


def part1():
    print("| 자차 속도 | 상황(거리 d / 접근속도 c) | 위험 판단 | 억제 태그 | 부저(기본 정책) | 부저(--buzz-without-speed) | 큰 화면 |\n|---|---|---|---|---|---|---|")
    cases = [("레벨 1 후보", 30.0, 4.0), ("레벨 2 후보", 12.0, 8.0)]
    for spd in (None, 20, 60, 100):
        for name, d, c in cases:
            vr = None if spd is None else spd / 3.6
            lv, why, m = D.assess(True, True, False, T(d, -c), vr, 0.5, 0.8)
            disp, buz = output_flags(lv)
            inhibit = vr is None
            sound_default = buz and not inhibit
            sound_override = buz
            sym = B.symbol(lv)
            print(f"| {'모름' if spd is None else str(spd) + ' km/h'} | {name}: d={d:g} m, c={c:g} m/s | level {lv} | {', '.join(m['supp']) or '-'} | "
                  f"{'울림' if sound_default else '무음'} | {'울림' if sound_override else '무음'} | '{sym}' ({B.STYLE[sym][1].strip()}) |")


def part2():
    allsc = scenario_lib.S(); idx = len(allsc) - 1; assert allsc[idx]["name"].startswith("B")
    out = os.path.expanduser("~/ESWContest_ws/logs/scenario_suite/speed_" + time.strftime("%Y%m%d_%H%M%S")); os.makedirs(out, exist_ok=True)
    print("\n종단 시험(가짜 STM32 pty, 속도 주입): ")
    print("| 주입 속도 | 레벨 분포 | mock 부저 전환 | 비고 |\n|---|---|---|---|")
    for spd in (None, 20, 60, 100):
        d = os.path.join(out, f"v{spd}"); os.makedirs(d, exist_ok=True); link = os.path.join(d, "stm32.pty"); res = os.path.join(d, "sim.json")
        sim = subprocess.Popen([sys.executable, "-I", "sim_suite.py", "--link", link, "--index", str(idx), "--result", res], cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        t = time.time()
        while not os.path.exists(link) and time.time() - t < 5: time.sleep(0.05)
        args = [sys.executable, "-I", "pipeline_main.py", "--uart", link, "--transport", "udp", "--port", "29300", "--broadcast-addr", "127.255.255.255", "--duration", "13", "--out", d,
                "--sinks", "csv", "--buzzer", "mock", "--affinity", "2,3"] + ([] if spd is None else ["--test-speed-kmh", str(spd)])
        pm = subprocess.run(args, cwd=HERE, capture_output=True, text=True, timeout=60); sim.wait(timeout=30)
        rows = [r for r in csv.DictReader(l for l in open(os.path.join(d, "drive_log.csv")) if not l.startswith("#")) if r["stm_status"] != ""]
        lv = collections.Counter(r["level"] for r in rows)
        tr = next((int(l.split("transitions")[1].split()[0]) for l in pm.stdout.splitlines() if "[buzzer] final" in l), None)
        supp = collections.Counter(tag for r in rows for tag in ("정지 물체" if "정지 물체" in r["reason"] else "", "마주오는/추월" if "마주오는" in r["reason"] else "", "비현실 차간거리" if "implausible" in r["reason"] else "") if tag)
        print(f"| {'모름(주입 없음)' if spd is None else str(spd) + ' km/h'} | {dict(lv)} | {tr} | {dict(supp) or ''} |")
    print("logs:", out)


if __name__ == "__main__":
    part1()
    if "--e2e" in sys.argv: part2()
