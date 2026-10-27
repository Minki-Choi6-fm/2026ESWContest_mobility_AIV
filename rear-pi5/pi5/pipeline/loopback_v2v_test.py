#!/usr/bin/env python3
"""S3 software-loopback test of the BSM chain on ONE Pi5.

rear node (ID1)  : bsm_node.py --source pipeline  <- fake_stm32 (pty) + fake_gpsd (port 22947)
front node (ID2) : bsm_node.py --source fixed
Both run on 127.255.255.255:19000 (real UDP sockets, loopback only). Faults are injected on the rear node's send side.
THIS IS NOT A WIRELESS MEASUREMENT: no radio, same host clock. PDR/STALE logic results are software results;
interval/latency numbers must not be quoted as performance.
Outputs: logs/v2v_loopback/<ts>/<scenario>/{tx,rx,events}_id*.csv and summary.csv.
usage: python3 -I loopback_v2v_test.py [s1 s2 ... s7]
"""
import csv, json, os, statistics as S, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
DRIVE = os.path.join(WS, "logs", "vehicle_test_logs", "drive_20261016_152211")
OUT = os.path.join(WS, "logs", "v2v_loopback", time.strftime("%Y%m%d_%H%M%S"))
PY = [sys.executable, "-I"]
D = 20.0


def bg(args, log):
    return subprocess.Popen(PY + args, stdout=open(log, "w"), stderr=subprocess.STDOUT)


def run(name, rear_args=(), phases=None, front_args=()):
    od = os.path.join(OUT, name); os.makedirs(od, exist_ok=True)
    link = os.path.join(od, "uart.tty")
    total = D if not phases else sum(p[0] for p in phases) + 2
    procs = [bg([os.path.join(HERE, "fake_stm32.py"), DRIVE, "--link", link, "--duration", str(total + 8)], os.path.join(od, "fake_stm32.log")),
             bg([os.path.join(HERE, "fake_gpsd.py"), DRIVE, "--port", "22947", "--duration", str(total + 8)], os.path.join(od, "fake_gpsd.log"))]
    try:
        time.sleep(2.0)
        front = bg([os.path.join(HERE, "bsm_node.py"), "--id", "2", "--source", "fixed", "--duration", str(total + 5), "--out", od] + list(front_args),
                   os.path.join(od, "node_front.log"))
        for i, (dur, extra) in enumerate(phases or [(D, list(rear_args))]):
            rear = bg([os.path.join(HERE, "bsm_node.py"), "--id", "1", "--source", "pipeline", "--uart", link, "--duration", str(dur),
                       "--out", od if i == 0 else os.path.join(od, f"phase{i}")] + list(extra), os.path.join(od, f"node_rear{i}.log"))
            rear.wait(timeout=dur + 20)
            if phases and i + 1 < len(phases):
                time.sleep(1.0)
        front.wait(timeout=total + 30)
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
    return od


def rd(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(p / 100 * (len(v) - 1))))] if v else float("nan")


def analyse(od, label, extra=None):
    tx = rd(os.path.join(od, "tx_id1.csv")) if os.path.exists(os.path.join(od, "tx_id1.csv")) else []
    rx = rd(os.path.join(od, "rx_id2.csv")); ev = rd(os.path.join(od, "events_id2.csv"))
    ok = [r for r in rx if r["result"] == "OK" and r["sender"] == "1"]
    dup = sum(r["result"] == "DUP" for r in rx); drop = sum(r["result"] == "DROP" for r in rx)
    ticks = len(tx)
    iv = [(float(b["t"]) - float(a["t"])) * 1000 for a, b in zip(ok, ok[1:])]
    seq_lost = sum(int(r["gap"]) for r in ok if r["gap"] not in ("", None) and 0 <= int(r["gap"]) < 0x8000)   # a SEQ restart (huge gap) is not "loss"
    row = dict(scenario=label, tx_ticks=ticks, rx_ok=len(ok), pdr_pct=round(100 * len(ok) / ticks, 1) if ticks else None,
               dup_rejected=dup, crc_drop=drop, seq_lost=seq_lost,
               int_mean_ms=round(S.mean(iv), 1) if iv else None, int_p50_ms=round(pct(iv, 50), 1), int_p95_ms=round(pct(iv, 95), 1),
               int_max_ms=round(max(iv), 1) if iv else None, stale_events=sum(e["event"] == "STALE" for e in ev))
    if extra:
        row.update(extra(tx, rx, ok, ev))
    return row


def main():
    want = sys.argv[1:] or ["s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9"]
    os.makedirs(OUT, exist_ok=True)
    rows = []
    print(f"logs -> {OUT}\n(소프트웨어 루프백: 무선 측정 아님, 같은 호스트 시계)\n")

    if "s1" in want:
        od = run("s1_normal")
        def ex(tx, rx, ok, ev):
            txm = {r["seq"]: r for r in tx}
            mism = sum(1 for r in ok if r["seq"] in txm and (r["status_raw"], r["speed_kmh"], r["lat"], r["lon"]) !=
                       (txm[r["seq"]]["status_raw"], txm[r["seq"]]["speed_kmh"], txm[r["seq"]]["lat"], txm[r["seq"]]["lon"]))
            bits = {}
            for r in ok:
                st = int(r["status_raw"])
                for n, b in (("GPS_VALID", 0), ("DANGER", 1), ("DEGRADED", 2), ("SAFE", 3), ("BRAKE", 4), ("SPD_GPS", 5), ("SPD_STALE", 6)):
                    bits[n] = bits.get(n, 0) + (st >> b & 1)
            sp = sorted({float(r["speed_kmh"]) for r in tx})
            return dict(content_mismatch=mism, status_bits_set={k: f"{v}/{len(ok)}" for k, v in bits.items()}, rear_speed_range=f"{sp[0]}..{sp[-1]}" if sp else "")
        rows.append(analyse(od, "S1 정상 10Hz", ex))
    for key, loss in (("s2", 0.10), ("s3", 0.30)):
        if key in want:
            rows.append(analyse(run(f"{key}_loss{int(loss * 100)}", ["--loss", str(loss)]), f"{key.upper()} 패킷손실 {int(loss * 100)}% (주입)"))
    if "s4" in want:
        def ex4(tx, rx, ok, ev):
            sent = sum(1 for r in tx if int(r["copies"]) > 0)
            return dict(note=f"전송 시도 {sent}프레임 중 약 20% 손상 주입(1비트 반전)")
        rows.append(analyse(run("s4_crc", ["--corrupt", "0.2"]), "S4 CRC 오류 20% (주입)", ex4))
    if "s5" in want:
        def ex5(tx, rx, ok, ev):
            t_ok = [float(r["t"]) for r in ok]
            pause_t0 = [float(r["t"]) for r in tx][0] + 8.0       # node t0 ~ first tx - 0.5
            stales = [float(e["t"]) for e in ev if e["event"] == "STALE"]; lives = [float(e["t"]) for e in ev if e["event"] == "LIVE"]
            if not stales:
                return dict(note="STALE 이벤트 없음")
            last_before = max(t for t in t_ok if t < stales[0])
            nxt = min((t for t in t_ok if t > stales[0]), default=None)
            return dict(stale_detect_after_last_frame_ms=round((stales[0] - last_before) * 1000), recovered_after_next_frame_ms=None if nxt is None else round((min(l for l in lives if l > stales[0]) - nxt) * 1000) if any(l > stales[0] for l in lives) else None,
                        silence_ms=None if nxt is None else round((nxt - last_before) * 1000))
        rows.append(analyse(run("s5_pause", ["--pause-at", "8", "--pause-for", "2"]), "S5 송신 2초 중단 → STALE(300ms)", ex5))
    if "s6" in want:
        rows.append(analyse(run("s6_seq", ["--seq-jump-at", "8", "--seq-jump", "100", "--dup", "0.1"]), "S6 SEQ 점프(+100)와 중복 10%"))
    if "s7" in want:
        def ex7(tx, rx, ok, ev):
            od7 = os.path.join(OUT, "s7_restart")
            txb = rd(os.path.join(od7, "phase1", "tx_id1.csv"))
            t_b0 = float(txb[0]["t"])                                   # first frame the restarted node sent (SEQ 0)
            seqa = [int(r["seq"]) for r in tx]
            last_a = max(float(r["t"]) for r in ok if int(r["seq"]) >= 300)
            after = [r for r in rx if r["sender"] == "1" and float(r["t"]) >= t_b0 - 0.05]
            first_ok = next((r for r in after if r["result"] == "OK"), None)
            rejected_before = 0
            for r in after:
                if r["result"] == "OK":
                    break
                rejected_before += r["result"] == "DUP"
            okb = [r for r in after if r["result"] == "OK"]
            lives = [float(e["t"]) for e in ev if e["event"] == "LIVE" and float(e["t"]) > last_a]
            return dict(tx_ticks=len(tx) + len(txb), pdr_pct=round(100 * len(ok) / (len(tx) + len(txb)), 1), phase_A_seq=f"{min(seqa)}..{max(seqa)}", rejected_frames_before_recovery=rejected_before,
                        restarted_frames_accepted=f"{len(okb)}/{len(txb)}",
                        outage_last_A_to_first_B_accepted_ms=None if first_ok is None else round((float(first_ok['t']) - last_a) * 1000),
                        LIVE_event=bool(lives), note="outage includes the 1 s pause + node start-up of the harness, not protocol delay")
        rows.append(analyse(run("s7_restart", phases=[(8, ["--start-seq", "300"]), (8, ["--start-seq", "0"])]), "S7 송신 노드 재시작(SEQ 300→0)", ex7))

    if "s8" in want:
        od = run("s8_front_k", front_args=["--fixed-k", "0.6", "--pause-at", "9", "--pause-for", "3"])
        dg = rd(os.path.join(od, "danger_id1.csv"))
        okt = [float(r["t"]) for r in rd(os.path.join(od, "rx_id1.csv")) if r["result"] == "OK" and r["sender"] == "2"]   # front frames the rear accepted
        import bisect as _b
        def fresh(t):                                           # front accepted within the last 300 ms (same rule as FreshnessTracker)
            i = _b.bisect_right(okt, t) - 1
            return i >= 0 and (t - okt[i]) <= 0.3
        n_f = n_s = ok_f = ok_s = 0; wrong = []
        for r in dg:
            t = float(r["t"]); k = float(r["k_f"])
            if fresh(t):
                n_f += 1; ok_f += abs(k - 0.6) < 1e-9
                if abs(k - 0.6) >= 1e-9: wrong.append((round(t, 2), "fresh but default K_f"))
            else:
                n_s += 1; ok_s += abs(k - 0.8) < 1e-9
                if abs(k - 0.8) >= 1e-9: wrong.append((round(t, 2), "stale but K_f=0.6"))
        reasons = sorted({r["k_f_reason"] for r in dg})
        rows.append(dict(scenario="S8 앞차 K 수신 → STALE 시 기본값", tx_ticks=len(dg), rx_ok=len(okt), pdr_pct="", dup_rejected="", crc_drop="", seq_lost="",
                         int_mean_ms="", int_p50_ms="", int_p95_ms="", int_max_ms="", stale_events="",
                         note=f"앞차 BSM 신선(300ms 이내) 행 {n_f}개 중 K_f=0.60 사용 {ok_f}; STALE/없음 행 {n_s}개 중 기본값 0.80 사용 {ok_s}; 불일치 {len(wrong)}건 {wrong[:3]}; 사유 종류: {reasons}"))

    if "s9" in want:
        import math, time as _t
        sys.path.insert(0, HERE)
        import v2v_adapter as V
        import danger, peer_support as PS
        lat0, lon0, hd = 37.55, 127.07, 90.0
        def at(fwd, lat_m, heading):
            th = math.radians(heading)
            dn = fwd * math.cos(th) - lat_m * math.sin(th); de = fwd * math.sin(th) + lat_m * math.cos(th)
            return lat0 + dn / PS.M_PER_DEG_LAT, lon0 + de / (PS.M_PER_DEG_LAT * math.cos(math.radians(lat0)))
        class T:                                              # radar target stand-in (stationary over ground at 60 km/h)
            def __init__(self, long_m, lat_m): self.long_m, self.lat_m, self.vrel_ms, self.dyn = long_m, lat_m, -60 / 3.6, 1
        res9 = []
        for label, spd in (("front car STOPPED (0 km/h)", 0.0), ("front car MOVING (40 km/h)", 40.0)):
            od = os.path.join(OUT, "s9_" + ("stopped" if spd == 0 else "moving")); os.makedirs(od, exist_ok=True)
            la, lo = at(30.0, 0.5, hd)
            front = bg([os.path.join(HERE, "bsm_node.py"), "--id", "2", "--source", "fixed", "--duration", "5", "--out", od, "--fixed-lat", repr(la),
                        "--fixed-lon", repr(lo), "--fixed-speed", str(spd)], os.path.join(od, "front.log"))
            tr = V.make_transport(19000, "127.255.255.255"); trk = V.FreshnessTracker()
            t_end = _t.monotonic() + 3.0
            while _t.monotonic() < t_end:
                data = tr.recv_bytes(0.05)
                if data:
                    try:
                        d = V.decode(data)
                    except V.BSMDecodeError:
                        continue
                    if d["vehicle_id"] == 2:
                        trk.update(d)
            peer = trk.get_valid(2)
            cases = [("radar target 1 m from the BSM position", T(31.0, 0.0)), ("radar target 8 m off", T(38.0, 1.0)), ("radar target 15 m off", T(45.0, 0.0))]
            row = []
            for cl, tg in cases:
                ok, why = PS.peer_supports(tg, peer, lat0, lon0, hd)
                row.append(f"{cl}: {'LIFT' if ok else 'no'}")
            ok1, _ = PS.peer_supports(cases[0][1], peer, lat0, lon0, hd)
            lv_no = danger.assess(True, True, False, T(31.0, 0.0), 60 / 3.6, 0.5, 0.8, lifted=False)[0]
            lv_yes = danger.assess(True, True, False, T(31.0, 0.0), 60 / 3.6, 0.5, 0.8, lifted=ok1)[0]
            front.wait(timeout=20)                           # front car stops transmitting
            _t.sleep(0.5)
            stale_peer = trk.get_valid(2)
            ok_stale = PS.peer_supports(cases[0][1], stale_peer, lat0, lon0, hd)[0]
            tr.close()
            res9.append((label, row, lv_no, lv_yes, ok_stale))
        for label, row, lv_no, lv_yes, ok_stale in res9:
            rows.append(dict(scenario=f"S9 peer_supports: {label}", tx_ticks="", rx_ok="", pdr_pct="", dup_rejected="", crc_drop="", seq_lost="",
                             int_mean_ms="", int_p50_ms="", int_p95_ms="", int_max_ms="", stale_events="",
                             note=f"{'; '.join(row)}; danger level for a ground-fixed target at 31 m / 60 km/h: without support {lv_no}, with support {lv_yes}; after the front car stopped transmitting (STALE) support = {ok_stale}"))

    print("| 시나리오 | tx(생성) | rx OK | PDR% | DUP거절 | CRC/형식 DROP | SEQ손실(수신기 계산) | 수신간격 mean/p50/p95/max ms | STALE 이벤트 | 비고 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        extra = {k: v for k, v in r.items() if k not in ("scenario", "tx_ticks", "rx_ok", "pdr_pct", "dup_rejected", "crc_drop", "seq_lost", "int_mean_ms", "int_p50_ms", "int_p95_ms", "int_max_ms", "stale_events")}
        print(f"| {r['scenario']} | {r['tx_ticks']} | {r['rx_ok']} | {r['pdr_pct']} | {r['dup_rejected']} | {r['crc_drop']} | {r['seq_lost']} | "
              f"{r['int_mean_ms']}/{r['int_p50_ms']}/{r['int_p95_ms']}/{r['int_max_ms']} | {r['stale_events']} | {json.dumps(extra, ensure_ascii=False) if extra else ''} |")
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(OUT, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)


if __name__ == "__main__":
    main()
