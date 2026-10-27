#!/usr/bin/env python3
"""S17-E: READ-ONLY summary of one run_car_test.sh log folder (logs/car_test/<date>/). Opens nothing but files in that folder.
Prints: eval rate, level distribution, level-2 / 0xFF intervals, STM32 restarts, 0x10 gaps, Safe Mode / power_suspect, speed source, camera alive/confirmed (+ outages),
BSM tx / rx_ok / rx_drop (from the console summary line), ST-Link debug log (RESET CAUSE, LOOP max, RADSTAT a/safe/ovf/txdrop) if acm_debug.log exists, Pi temp/CPU if sys_sampler.csv exists.
usage: python3 post_run_report.py <log folder>"""
import collections, csv, glob, os, re, statistics as S, sys, time

tm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))


def intervals(rows, pred, step=0.05):
    out, cur = [], None
    for r in rows:
        w = float(r["wall"])
        if pred(r):
            cur = [w, w] if cur is None else [cur[0], w]
        elif cur:
            out.append(cur); cur = None
    if cur: out.append(cur)
    return [(tm(a), round(b - a + step, 2)) for a, b in out]


def main(d):
    d = d.rstrip("/")
    print(f"=== {d}")
    p = os.path.join(d, "drive_log.csv")
    if not os.path.exists(p):
        print("drive_log.csv 없음"); return 2
    head = open(p).readline().strip()
    rows = list(csv.DictReader(l for l in open(p) if not l.startswith("#")))
    if not rows:
        print("행 없음"); return 2
    print(head)
    T = [float(r["t_rel"]) for r in rows]
    gaps = sorted((b - a) * 1000 for a, b in zip(T, T[1:]))
    print(f"평가 {len(rows)}회, {T[-1]:.1f}s ({len(rows) / max(T[-1], 1e-9):.2f} Hz) {tm(float(rows[0]['wall']))}~{tm(float(rows[-1]['wall']))}; 간격 p50 {gaps[len(gaps) // 2]:.0f} p99 {gaps[int(.99 * (len(gaps) - 1))]:.0f} max {gaps[-1]:.0f} ms, 100ms 초과 {sum(g > 100 for g in gaps)}회")
    lv = collections.Counter(r["level"] for r in rows)
    print("레벨 분포:", {k: f"{v} ({100 * v / len(rows):.1f}%)" for k, v in sorted(lv.items())})
    print("레벨 2 구간(시각,초):", intervals(rows, lambda r: r["level"] == "2") or "없음")
    print("레벨 0xFF 구간:", intervals(rows, lambda r: r["level"] == "255") or "없음")
    print(f"Safe Mode 행 {sum(r['safe_mode'] == '1' for r in rows)}, radar_down {sum(r['radar_down'] == '1' for r in rows)}, power_suspect {sum(r['power_suspect'] == '1' for r in rows)} (power_suspect=전원 처짐 의심 표시이지 확정이 아님)")
    print("속도 출처:", dict(collections.Counter(r["speed_src"] for r in rows)), "(none 이면 부저 억제, 표시만)")
    al = sum(r["camera_alive"] == "1" for r in rows)
    first = next((i for i, r in enumerate(rows) if r["camera_alive"] == "1"), None)
    out = intervals(rows[first:], lambda r: r["camera_alive"] == "0") if first is not None else []
    print(f"카메라 alive {100 * al / len(rows):.1f}%, confirmed 행 {sum(r['camera_confirmed'] == '1' for r in rows)}, 첫 alive 이후 끊김 구간: {out or '없음'}")
    dl = sum(r["danger_active"] == "1" for r in csv.DictReader(open(os.path.join(d, "bsm_tx.csv")))) if os.path.exists(os.path.join(d, "bsm_tx.csv")) else None
    if os.path.exists(os.path.join(d, "bsm_tx.csv")):
        b = list(csv.DictReader(open(os.path.join(d, "bsm_tx.csv")))); bt = [float(x["t"]) for x in b]
        print(f"BSM 송신 {len(b)}행, 0.3s 초과 송신 간격 {sum((y - x) > 0.3 for x, y in zip(bt, bt[1:]))}회, danger_active=1 {dl}행, status_raw {dict(collections.Counter(x['status_raw'] for x in b))}")
    txt = ""
    for f in ("run_console.txt", "pipeline_stdout.txt"):
        if os.path.exists(os.path.join(d, f)): txt += open(os.path.join(d, f), errors="replace").read()
    m = re.findall(r"\[pipeline_main\] (\{.*?\})", txt)
    print("BSM 요약(tx / rx_ok=앞차→뒷차 수신 / rx_drop):", m[-1] if m else "없음(이 실행은 요약 줄 미저장)")
    for k in re.findall(r"\[buzzer\] final: [^\n]*|\[camera\] [^\n]*", txt)[-2:]: print(k.strip())
    # STM32 restarts
    fr = os.path.join(d, "rec", "frames.csv")
    if os.path.exists(fr):
        up, ts = [], []
        for r in csv.DictReader(open(fr)):
            if r["type"] == "0x11": up.append((float(r["time"]), int.from_bytes(bytes.fromhex(r["payload_hex"])[:4], "little")))
            if r["type"] == "0x10": ts.append(float(r["time"]))
        print("STM32 가동시간 역행(재시작):", [(tm(t2), u1, u2) for (t1, u1), (t2, u2) in zip(up, up[1:]) if u2 < u1 - 1000] or "없음", "| 0x10 끊김>0.3s:", [(tm(a), round(b - a, 2)) for a, b in zip(ts, ts[1:]) if b - a > 0.3] or "없음")
    for sf in glob.glob(os.path.join(d, "rx_stats_*.csv")):
        r = list(csv.DictReader(open(sf)))
        if r:
            x = r[-1]; print(f"rx_stats: stm_resets {x['stm_resets']}, 오류 len/chk {x['bad_len']}/{x['bad_chk']}, 손실 {x['loss_pct']}%, 타임아웃 summary/hb/radar/imu {x['timeouts_summary']}/{x['timeouts_heartbeat']}/{x['timeouts_radar_0x12']}/{x['timeouts_imu']}")
    ap = os.path.join(d, "acm_debug.log")
    if os.path.exists(ap):
        L = open(ap, errors="replace").read().splitlines()
        rc = [l for l in L if "RESET CAUSE" in l]
        print(f"ST-Link 디버그 로그 {len(L)}줄: RESET CAUSE {len(rc)}개 {[l[:60] for l in rc[:3]]}, BOOT {sum('BOOT' in l for l in L)}개")
        loop = [int(re.search(r"max=(\d+)", l).group(1)) for l in L if "LOOP max" in l]
        rs = [l for l in L if "RADSTAT" in l]
        if loop: print(f"LOOP max: 최대 {max(loop)} ms, 20ms 이상 {sum(v >= 20 for v in loop)}회")
        if rs:
            g = lambda k, l: int(re.search(k + r"=(\d+)", l).group(1))
            A = [g("a", l) for l in rs if re.search(r" a=\d+", l)]
            print(f"RADSTAT {len(rs)}줄: a(상태 프레임/s) 최소 {min(A)} 평균 {S.mean(A):.1f}, a<14 인 초 {sum(v < 14 for v in A)}, safe {g('safe', rs[0])}->{g('safe', rs[-1])}, ovf {g('ovf', rs[0])}->{g('ovf', rs[-1])}, gap 최대 {max(g('gap', l) for l in rs)} ms, txdrop 마지막 {g('txdrop', rs[-1]) if 'txdrop=' in rs[-1] else '-'}")
        ol = [l for l in L if "OLEDSTAT" in l]
        if ol: print("OLEDSTAT 마지막:", ol[-1][-90:])
    sp = os.path.join(d, "sys_sampler.csv")
    if os.path.exists(sp):
        r = list(csv.DictReader(open(sp)))
        if r: print(f"Pi 온도 {r[0]['temp_c']}->최대 {max(float(x['temp_c']) for x in r)} °C, throttled {sorted(set(x['throttled'] for x in r))}, CPU 코어 평균 {[round(S.mean(float(x['cpu%d' % i]) for x in r)) for i in range(4)]}%")
    else:
        print("(sys_sampler.csv 없음: 온도/CPU/throttled 기록 안 함 — 수동: vcgencmd measure_temp; vcgencmd get_throttled)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__); sys.exit(1)
    sys.exit(main(sys.argv[1]))
