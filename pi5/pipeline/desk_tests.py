#!/usr/bin/env python3
"""S2.7 desk-test harness: fake_stm32 (pty) + fake_gpsd (port 22947) + live_receiver, per scenario, then a result table.
Never touches /dev/ttyAMA2, the real gpsd (2947) or any service. Output logs -> logs/desk_tests/<timestamp>/.
usage: python3 -I desk_tests.py [a b c d e f]   (default: all)"""
import os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
DRIVE = os.path.join(WS, "logs", "vehicle_test_logs", "drive_20261016_152211")
OUT = os.path.join(WS, "logs", "desk_tests", time.strftime("%Y%m%d_%H%M%S"))
PY = [sys.executable, "-I"]
GPSD_PORT = "22947"
LINE = re.compile(r"t=\s*(\d+)s \| OK 10:(\d+) 11:(\d+) 12:(\d+) 13:(\d+) 14:(\d+) \| BAD len=(\d+) chk=(\d+) resync=(\d+) \| loss=(\S+) \| "
                  r"speed=(NONE\[[^\]]*\]|\d+km/h (OBD|GPS) age=\d+ms) \| sts=(--|0x[0-9A-F]{2}\[[^\]]*\]) \| summary=(\S+) heartbeat=(\S+) radar_0x12=(\S+) imu=(\S+) obd=(\S+)")


def parse(text):
    rows = []
    for l in text.splitlines():
        m = LINE.search(l)
        if m:
            g = m.groups()
            rows.append(dict(t=int(g[0]), ok=[int(x) for x in g[1:6]], len=int(g[6]), chk=int(g[7]), rs=int(g[8]),
                             loss=g[9], src="NONE" if g[10].startswith("NONE") else g[11], sts=g[12], summary=g[13], hb=g[14],
                             radar=g[15], imu=g[16], obd=g[17], resets=("STM_RESETS=" in l and re.search(r"STM_RESETS=(\d+)", l).group(1))))
    return rows


def run(name, stm_args, total, rx_args=(), pre_wait=2.0, rx_delay=None):
    """Start fake_stm32 + fake_gpsd, wait pre_wait, run live_receiver for `total` s. Returns (rows, text, lag, fake_log)."""
    os.makedirs(OUT, exist_ok=True)
    link = os.path.join(OUT, f"{name}.tty")
    fs_log, fg_log = open(os.path.join(OUT, f"{name}_fake_stm32.log"), "w"), open(os.path.join(OUT, f"{name}_fake_gpsd.log"), "w")
    t_fake = time.monotonic()
    fs = subprocess.Popen(PY + [os.path.join(HERE, "fake_stm32.py"), DRIVE, "--link", link, "--duration", str(total + pre_wait + 3)] + list(stm_args),
                          stdout=fs_log, stderr=subprocess.STDOUT)
    fg = subprocess.Popen(PY + [os.path.join(HERE, "fake_gpsd.py"), DRIVE, "--port", GPSD_PORT, "--duration", str(total + pre_wait + 3)],
                          stdout=fg_log, stderr=subprocess.STDOUT)
    try:
        time.sleep(pre_wait)
        t_rx = time.monotonic()
        r = subprocess.run(PY + [os.path.join(HERE, "live_receiver.py"), "--port", link, "--gpsd-port", GPSD_PORT, "--duration", str(total),
                                 "--stats-dir", OUT] + list(rx_args), capture_output=True, text=True, timeout=total + 30)
        text = r.stdout + r.stderr
    finally:
        for p in (fs, fg):
            p.terminate()
        for p in (fs, fg):
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        fs_log.close(); fg_log.close()
    open(os.path.join(OUT, f"{name}_receiver.log"), "w").write(text)
    return parse(text), text, t_rx - t_fake, open(os.path.join(OUT, f"{name}_fake_stm32.log")).read()


def first(rows, pred, after=0):
    for r in rows:
        if r["t"] >= after and pred(r):
            return r["t"]
    return None


def main():
    want = sys.argv[1:] or ["a", "b", "c", "d", "e", "f"]
    res = []
    print(f"logs -> {OUT}\ndrive: {os.path.basename(DRIVE)}  (fake GPS port {GPSD_PORT}, real gpsd/ttyAMA2 untouched)\n")

    if "a" in want:
        rec = os.path.join(OUT, "a_record")
        rows, text, lag, fl = run("a_normal", [], 25, ["--record", rec])
        body = [r for r in rows if r["t"] >= 3]
        obd = sum(r["src"] == "OBD" for r in body)
        last = rows[-1]
        # replay of what we just recorded
        rp = subprocess.run(PY + [os.path.join(HERE, "live_receiver.py"), "--replay", rec], capture_output=True, text=True).stdout
        rl = parse(rp)[-1]
        rec_rows = sum(1 for _ in open(os.path.join(rec, "frames.csv"))) - 1
        res.append(("a 정상", f"OBD 선택 {obd}/{len(body)}줄(t≥3s); BAD len/chk/resync={last['len']}/{last['chk']}/{last['rs']}; 마지막 OK={last['ok']}",
                    f"저장 frames.csv={rec_rows}줄, replay 카운트={rl['ok']} → {'일치' if sum(rl['ok']) == rec_rows and rl['ok'] == last['ok'] else '불일치(수신 종료 시점 차이 가능, 아래 참고)'}"))
        res.append(("a 정상(참고)", f"live OK 합={sum(last['ok'])}", f"record 줄수={rec_rows}, replay OK 합={sum(rl['ok'])}"))

    if "b" in want:
        rows, text, lag, fl = run("b_obd_fail", ["--obd-fail", "12", "--obd-fail-at", "8"], 30)
        s0, s1 = 8 - lag, 8 + 12 - lag
        gps_first = first(rows, lambda r: r["src"] == "GPS", max(1, int(s0)))
        obd_back = first(rows, lambda r: r["src"] == "OBD", int(s1))
        src_win = [r["src"] for r in rows if s0 + 2 <= r["t"] <= s1 - 1]
        res.append(("b OBD 실패", f"창(수신기 시각)≈{s0:.0f}~{s1:.0f}s; GPS 첫 전환 t={gps_first}s (창 시작 +{(gps_first - s0) if gps_first is not None else '-'}s); 창 안 소스={dict((k, src_win.count(k)) for k in set(src_win))}",
                    f"OBD 복귀 t={obd_back}s (창 끝 +{(obd_back - s1) if obd_back is not None else '-'}s); NONE 줄={sum(r['src'] == 'NONE' for r in rows)}"))

    for key, mode, label in (("c1", "targets", "c1 레이더 0x12만 끊김(RADAR_OK=1 유지)"), ("c2", "dead", "c2 레이더 사망(RADAR_OK=0, SAFE_MODE=1)")):
        if key not in want and "c" not in want:
            continue
        rows, text, lag, fl = run(key + "_drop_radar_" + mode, ["--drop-radar", "10", "--drop-radar-at", "8", "--drop-radar-mode", mode], 28)
        s0, s1 = 8 - lag, 18 - lag
        to = first(rows, lambda r: r["radar"] == "TIMEOUT", int(s0))
        flag = first(rows, lambda r: "RADAR" not in r["sts"] and r["sts"] != "--", int(s0))
        inwin = [r for r in rows if s0 + 2 <= r["t"] <= s1 - 1]
        alive = sum("RADAR" in r["sts"] for r in inwin); safe = sum("SAFE_MODE" in r["sts"] for r in inwin)
        back = first(rows, lambda r: r["radar"] == "ok" and "RADAR" in r["sts"], int(s1))
        res.append((label, f"창≈{s0:.0f}~{s1:.0f}s; radar_0x12=TIMEOUT 첫 감지 t={to}s; 창 안 RADAR_OK=1 줄 {alive}/{len(inwin)}, SAFE_MODE=1 줄 {safe}/{len(inwin)}; RADAR 비트 꺼짐 첫 t={flag}",
                    f"0x12 복구 t={back}s (창 끝 +{(back - s1) if back is not None else '-'}s)"))

    if "d0" in want:
        rows, text, lag, fl = run("d0_baseline", [], 30)
        csvp = max((os.path.join(OUT, f) for f in os.listdir(OUT) if f.startswith("rx_stats_")), key=os.path.getmtime)
        import csv as _csv
        lr = list(_csv.DictReader(open(csvp)))[-1]
        res.append(("d0 기준선(오류 주입 없음)", f"빠진 프레임: 0x10={int(lr['expected_0x10']) - int(lr['ok_0x10'])}, 0x11={int(lr['expected_0x11']) - int(lr['ok_0x11'])}",
                    f"loss_pct 0x10={lr['loss_pct_0x10']}%, 0x11={lr['loss_pct_0x11']}%, 합산={lr['loss_pct']}% (재생 원본 로그 자체의 자연 손실)"))

    if "d" in want:
        rows, text, lag, fl = run("d_corrupt", ["--corrupt", "0.002", "--corrupt-at", "8", "--corrupt-for", "10"], 30)
        s0, s1 = 8 - lag, 18 - lag
        pre = [r for r in rows if r["t"] <= s0 - 1][-1]
        end = next(r for r in rows if r["t"] >= s1 + 2)          # 2 s margin after the injection window ends
        post = rows[-1]
        inj = re.search(r"'corrupted_bytes': (\d+)", fl); injf = re.search(r"'corrupted_frames': (\d+)", fl)
        rate = lambda a, b: (b["ok"][0] - a["ok"][0]) / max(1, b["t"] - a["t"])
        csvp = max((os.path.join(OUT, f) for f in os.listdir(OUT) if f.startswith("rx_stats_")), key=os.path.getmtime)
        import csv as _csv
        last_row = list(_csv.DictReader(open(csvp)))[-1]
        cbt = re.search(r"'corrupted_by_type': (\{[^}]*\})", fl)
        cb = eval(cbt.group(1)) if cbt else {}
        miss10 = int(last_row["expected_0x10"]) - int(last_row["ok_0x10"]); miss11 = int(last_row["expected_0x11"]) - int(last_row["ok_0x11"])
        res.append(("d 손실률 검증(STM32 시계)", f"주입 손상 프레임: 0x10={cb.get('0x10', 0)}, 0x11={cb.get('0x11', 0)} / 측정 빠진 프레임: 0x10={miss10}, 0x11={miss11}",
                    f"loss_pct 0x10={last_row['loss_pct_0x10']}%, 0x11={last_row['loss_pct_0x11']}%, 합산={last_row['loss_pct']}%"))
        res.append(("d 바이트 오류", f"주입 {injf.group(1) if injf else '?'}프레임/{inj.group(1) if inj else '?'}바이트; BAD(len+chk): 창 전 {pre['len'] + pre['chk']} → 창 끝+2s {end['len'] + end['chk']} → 종료 {post['len'] + post['chk']}; resync {pre['rs']}→{end['rs']}→{post['rs']}",
                    f"복구: 창 후 BAD 증가 {post['len'] + post['chk'] - end['len'] - end['chk']}; 0x10 수신율 창 후 {rate(end, post):.0f}/s (정상 50). BAD는 거절된 STX 후보 수라 손상 프레임 수보다 큼"))

    if "e" in want:
        rows, text, lag, fl = run("e_backlog", [], 8, pre_wait=8.0)
        m = re.search(r"discarded stale kernel backlog: (\d+) bytes", text)
        r1 = rows[0] if rows else None
        res.append(("e 시작 백로그", f"fake가 8초간 먼저 송신, 수신기 늦게 시작 → 버린 백로그 {m.group(1) if m else '없음'} bytes",
                    f"첫 상태줄 t={r1['t'] if r1 else '-'}s: OK={r1['ok'] if r1 else '-'} (백로그를 안 버렸다면 8초 분량≈400개 이상); BAD len/chk/resync={r1['len'] if r1 else '-'}/{r1['chk'] if r1 else '-'}/{r1['rs'] if r1 else '-'}"))

    if "f" in want:
        rows, text, lag, fl = run("f_stm_reset", ["--stm-reset", "--reset-at", "8", "--reset-gap", "6"], 28)
        s0, s1 = 8 - lag, 14 - lag
        to = first(rows, lambda r: r["summary"] == "TIMEOUT", int(s0))
        rs = first(rows, lambda r: r["resets"] not in (False, "0"), int(s0))
        back = first(rows, lambda r: r["summary"] == "ok", int(s1))
        res.append(("f STM32 리셋 모사(참고)", f"침묵 창≈{s0:.0f}~{s1:.0f}s; summary TIMEOUT 첫 감지 t={to}s; STM_RESETS 표시 t={rs}s",
                    f"복귀 t={back}s; 마지막 줄 resets={rows[-1]['resets']}"))

    print("| 시나리오 | 결과 1 | 결과 2 |\n|---|---|---|")
    for n, a, b in res:
        print(f"| {n} | {a} | {b} |")


if __name__ == "__main__":
    main()
