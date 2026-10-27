#!/usr/bin/env python3
"""S3 V2V node (software loopback / later real use): builds BSM from the pipeline (or fixed values), sends at 10 Hz, receives and
checks CRC / SEQ / freshness through v2v_adapter (9/27 V2V code). Writes tx/rx/event CSVs for metrics.

--source pipeline : reads a UART (read-only; a fake_stm32 pty in tests) + gpsd (fake_gpsd in tests) -> bsm_builder -> BSM
--source fixed    : constant values (stands in for the front car)
Fault injection (send side, wraps the real UDP transport; the bytes still travel over a real local UDP socket):
  --loss P  --corrupt P  --dup P  --seq-jump-at T --seq-jump N  --pause-at T --pause-for S (node freezes: nothing sent, SEQ not advanced)
Default broadcast address is the LOOPBACK broadcast 127.255.255.255 so nothing leaves via wlan0 (255.255.255.255 would reach the Wi-Fi!).
"""
import argparse, csv, json, os, random, select, signal, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2v_adapter as V
from bsm_builder import BsmFields, build_fields
from constants import BSM_RATE_HZ
from gps_position import GpsPositionClient
from brain import Brain
from speed_select import get_speed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True); ap.add_argument("--port", type=int, default=19000)
    ap.add_argument("--broadcast-addr", default="127.255.255.255"); ap.add_argument("--duration", type=float, default=20)
    ap.add_argument("--source", choices=["pipeline", "fixed"], default="fixed")
    ap.add_argument("--uart"); ap.add_argument("--gpsd-port", type=int, default=22947)
    ap.add_argument("--out", required=True); ap.add_argument("--start-seq", type=int, default=0)
    ap.add_argument("--loss", type=float, default=0); ap.add_argument("--corrupt", type=float, default=0); ap.add_argument("--dup", type=float, default=0)
    ap.add_argument("--seq-jump-at", type=float, default=-1); ap.add_argument("--seq-jump", type=int, default=0)
    ap.add_argument("--pause-at", type=float, default=-1); ap.add_argument("--pause-for", type=float, default=0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--transport", choices=["udp", "esp-now"], default="udp"); ap.add_argument("--serial-port", default="/dev/ttyUSB0")
    ap.add_argument("--record", metavar="DIR", help="pipeline source: also write frames.csv/radar.csv (drive_logger format) into DIR, so this ONE process owns the UART")
    ap.add_argument("--stats-dir", default=None, help="pipeline source: write rx_stats_*.csv (OK/BAD/loss counters) here")
    ap.add_argument("--k-profile", default=None, help="path of the K profile json (default: none = not stored). Real runs: ~/ESWContest_ws/state/k_profile.json")
    ap.add_argument("--reset-k", action="store_true", help="ignore/clear the stored K profile"); ap.add_argument("--k-fixed", type=float, default=None, help="demo: pin K to this value")
    ap.add_argument("--fixed-lat", type=float, default=37.55); ap.add_argument("--fixed-lon", type=float, default=127.07)
    ap.add_argument("--fixed-speed", type=float, default=40.0, help="fixed source: speed km/h advertised")
    ap.add_argument("--fixed-k", type=float, default=None, help="fixed source only: K_VALUE advertised in the BSM (front car stand-in)")
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    os.makedirs(a.out, exist_ok=True)
    tag = f"id{a.id}"
    f_tx, f_rx, f_ev = (open(os.path.join(a.out, f"{n}_{tag}.csv"), "w", newline="") for n in ("tx", "rx", "events"))
    w_tx, w_rx, w_ev = csv.writer(f_tx), csv.writer(f_rx), csv.writer(f_ev)
    f_dg = open(os.path.join(a.out, f"danger_{tag}.csv"), "w", newline=""); w_dg = csv.writer(f_dg)
    w_dg.writerow(["t", "level", "k_r", "k_src", "k_f", "k_f_reason", "target", "level_before_suppression", "suppression", "reason"])
    w_tx.writerow(["t", "seq", "copies", "status_raw", "speed_kmh", "lat", "lon"])
    w_rx.writerow(["t", "result", "sender", "seq", "gap", "status_raw", "speed_kmh", "lat", "lon", "ts_ms", "drop_reason"])
    w_ev.writerow(["t", "event"])

    transport = V.make_transport(a.port, a.broadcast_addr, a.transport, a.serial_port)
    tracker = V.FreshnessTracker()
    other = 2 if a.id == 1 else 1
    gps = rx = fd = brain = None
    if a.source == "pipeline":
        brain = Brain(k_profile=a.k_profile, reset_k=a.reset_k, k_fixed=a.k_fixed)
        from live_receiver import open_readonly, Receiver, Recorder
        gps = GpsPositionClient(port=a.gpsd_port)
        rec = Recorder(a.record, {"tool": "bsm_node.py", "uart": a.uart, "start": time.time()}) if a.record else None
        sp_path = os.path.join(a.stats_dir, time.strftime("rx_stats_%Y%m%d_%H%M%S.csv")) if a.stats_dir else None
        rx = Receiver(gps, sp_path, 5.0, rec)
        fd, _ = open_readonly(a.uart)
        td = time.monotonic()
        while time.monotonic() - td < 2.0:                         # drop stale backlog like live_receiver
            try:
                if not os.read(fd, 65536):
                    break
            except BlockingIOError:
                break
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))

    fixed = BsmFields(lat=a.fixed_lat, lon=a.fixed_lon, speed_kmh=a.fixed_speed, heading_deg=90.0, k_value=None, gps_valid=True, danger_active=False,
                      sensor_degraded=False, safe_mode=False, brake_active=False, speed_src_gps=False, speed_stale=False)
    if a.fixed_k is not None:
        from dataclasses import replace
        fixed = replace(fixed, k_value=a.fixed_k)
    seq, period = a.start_seq & 0xFFFF, 1.0 / BSM_RATE_HZ
    t0 = time.monotonic(); next_tx = t0 + 0.5
    last_ok_seq, stale_now, seen_other, jumped = None, False, False, False
    cnt = dict(ticks=0, sent_copies=0, lost_by_fault=0, corrupted=0, dup=0, ok=0, dup_rejected=0, crc_drop=0, self_rx=0)
    while not stop and time.monotonic() - t0 < a.duration:
        now = time.monotonic(); rel = now - t0
        rl = [transport.sock] + ([fd] if fd is not None else []) + ([gps.sock] if gps and gps.sock else [])
        if gps:
            gps.connect(now)
        r, _, _ = select.select(rl, [], [], max(0.0, min(0.02, next_tx - now)))
        now = time.monotonic(); rel = now - t0
        for s in r:
            if fd is not None and s is fd:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    data = b""
                if data:
                    for fr in rx.feed(data, now, time.time()):
                        brain.on_frame(fr, now)
            elif gps and s is gps.sock:
                gps.pump(now)
        if rx:
            rx.poll(now)
            rx.maybe_stats(now, time.time())
        # ---- receive all pending datagrams
        while True:
            data = transport.recv_bytes(0.0)
            if data is None:
                break
            t = time.monotonic()
            try:
                d = V.decode(data)
            except V.BSMDecodeError as e:
                cnt["crc_drop"] += 1
                w_rx.writerow([f"{t:.4f}", "DROP", "", "", "", "", "", "", "", "", str(e)[:40]]); continue
            if d["vehicle_id"] == a.id:
                cnt["self_rx"] += 1; continue
            ok = tracker.update(d)
            gap = ""
            if ok:
                if last_ok_seq is not None:
                    gap = ((d["seq"] - last_ok_seq) & 0xFFFF) - 1
                last_ok_seq = d["seq"]; cnt["ok"] += 1; seen_other = True
            else:
                cnt["dup_rejected"] += 1
            w_rx.writerow([f"{t:.4f}", "OK" if ok else "DUP", d["vehicle_id"], d["seq"], gap, d["status_raw"], d["speed_kmh"], d["lat"], d["lon"], d["timestamp_ms"], ""])
        # ---- freshness transitions of the other vehicle
        if seen_other:
            st = tracker.is_stale(other)
            if st != stale_now:
                stale_now = st; w_ev.writerow([f"{now:.4f}", "STALE" if st else "LIVE"])
        # ---- transmit tick
        if now >= next_tx:
            next_tx += period
            if a.pause_at >= 0 and a.pause_at <= rel < a.pause_at + a.pause_for:
                continue                                              # frozen node
            if not jumped and a.seq_jump_at >= 0 and rel >= a.seq_jump_at:
                seq = (seq + a.seq_jump) & 0xFFFF; jumped = True
            if a.source == "pipeline":
                sp = get_speed(rx.state, gps, now)
                peer = tracker.get_valid(other)                           # None when the front car is STALE / never heard
                own = gps.position(now)
                bo = brain.evaluate(rx.state, sp, peer, peer is None, now, own_fix=own, own_heading=gps.last_heading)
                fields = build_fields(rx.state, sp, own, gps.last_heading, now, danger_level=bo.level, k_measured=bo.k_measured)
                w_dg.writerow([f"{now:.4f}", bo.level, f"{bo.k_r:.2f}", bo.k_src, f"{bo.k_f:.2f}", bo.k_f_reason,
                               "" if bo.target is None else bo.target.id, "" if bo.pre_level is None else bo.pre_level, ";".join(bo.supp), bo.reason[:160]])
            else:
                fields = fixed
            frame = V.encode(fields, a.id, seq)
            d0 = V.decode(frame)
            cnt["ticks"] += 1
            out = bytearray(frame); copies = 1
            if rnd.random() < a.loss:
                copies = 0; cnt["lost_by_fault"] += 1
            else:
                if rnd.random() < a.corrupt:
                    out[rnd.randrange(len(out))] ^= 1 << rnd.randrange(8); cnt["corrupted"] += 1
                if rnd.random() < a.dup:
                    copies = 2; cnt["dup"] += 1
                for _ in range(copies):
                    transport.send_bytes(bytes(out))
            cnt["sent_copies"] += copies
            w_tx.writerow([f"{now:.4f}", seq, copies, d0["status_raw"], d0["speed_kmh"], d0["lat"], d0["lon"]])
            seq = (seq + 1) & 0xFFFF
    transport.close()
    if brain:
        brain.kest.finish()
    if rx:
        rx.close(time.monotonic(), time.time())
        if rx.recorder:
            rx.recorder.close()
    for f in (f_tx, f_rx, f_ev, f_dg):
        f.close()
    json.dump(cnt, open(os.path.join(a.out, f"summary_{tag}.json"), "w"))
    print(f"[bsm_node id={a.id}]", cnt, flush=True)


if __name__ == "__main__":
    main()
