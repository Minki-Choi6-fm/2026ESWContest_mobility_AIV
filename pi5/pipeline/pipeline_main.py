#!/usr/bin/env python3
"""Integrated runner (SIMULATION ONLY): live UART receiver + S4/S5 brain + BSM send/receive in ONE process, outputs behind an OutputSink interface.

* Reads /dev/ttyAMA2 (or a fake pty) READ-ONLY. It NEVER writes to the UART: no 0x20/0x21/0x22 (first drive = no change in who writes, see
  docs/handoff/SERIAL_WRITE_OPTIONS.md). The warning level only goes to OutputSinks (console / CSV now; buzzer / OLED link = attach points).
* Output policy (docs/handoff/OUTPUT_POLICY.md): level 1 (caution) is DISPLAY ONLY, only level 2 (danger) is a buzzer CANDIDATE (constants.BUZZER_MIN_LEVEL),
  unknown (0xFF) never sounds and is displayed as "?". No buzzer/OLED is attached (their sinks refuse to be constructed).
* Because it owns the port it can record frames.csv/radar.csv (--record) and the rx statistics (--stats-dir): the ONE reader of the port.
* Safe output state: whenever the program ends, crashes, receives SIGINT/SIGTERM, or its main loop hangs for > WATCHDOG_S, every sink is put in SAFE
  state (output OFF). Level 0xFF (unknown) is also shown as "off + unknown", never as a warning.
Process/core layout (rule 3: YOLO cores 0-1, V2V cores 2-3): this process = receiver + brain + V2V -> cores 2-3 (--affinity 2,3 or taskset);
the camera/YOLO process (vision, later) is a separate process on cores 0-1 and only publishes detections to this one.
"""
import argparse, csv, json, os, select, signal, sys, threading, time
from abc import ABC, abstractmethod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2v_adapter as V
from bsm_builder import build_fields
from brain import Brain
import constants as C
from constants import BSM_RATE_HZ
from gps_position import GpsPositionClient
from live_receiver import Receiver, Recorder, open_readonly
from speed_select import get_speed, SpeedResult
from camera_link import CameraLink                         # S12: camera detections -> camera_confirmed (auxiliary flag only)
from constants import SOURCE_NONE

EVAL_HZ = 20
WATCHDOG_S = 1.0
LEVEL_TEXT = {0: "OFF(safe)", 1: "CAUTION(display only)", 2: "DANGER(buzzer candidate)", 0xFF: "?(unknown, never a buzzer)"}
DISPLAY_CHAR = {0: "-", 1: "!", 2: "!!", 0xFF: "?"}


def output_flags(level):
    """(display character, buzzer candidate?) for a level. Unknown never sounds, even if BUZZER_MIN_LEVEL were lowered."""
    return DISPLAY_CHAR.get(level, "?"), (level != 0xFF and level >= C.BUZZER_MIN_LEVEL)


# ------------------------------------------------------------------ outputs
class OutputSink(ABC):
    """An output device. update() is called at EVAL_HZ with the newest level; safe_state() MUST switch the output off and be idempotent."""
    @abstractmethod
    def update(self, now: float, level: int, reason: str, info: dict) -> None: ...
    @abstractmethod
    def safe_state(self, why: str) -> None: ...
    def close(self) -> None: ...


class ConsoleSink(OutputSink):
    def __init__(self):
        self.last, self.t_print = None, 0.0

    def update(self, now, level, reason, info):
        if level != self.last or now - self.t_print >= 5.0:
            side = f"  side={info.get('side_count', 0)}" + (f" nearest d={info['side_near'][0]:.1f} |lat|={info['side_near'][1]:.1f} vrel={info['side_near'][2]:+.1f}" if info.get("side_near") else "")
            print(f"[output] {info.get('display', '?')} {LEVEL_TEXT.get(level, level)}{side}{'  CUT-IN' if info.get('cutin') else ''}  {reason[:120]}", flush=True)
            self.last, self.t_print = level, now

    def safe_state(self, why):
        if self.last != "SAFE":
            print(f"[output] SAFE STATE (output off): {why}", flush=True)
        self.last = "SAFE"


class CsvSink(OutputSink):
    def __init__(self, path):
        self.f = open(path, "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(["t", "event", "level", "display", "buzzer_candidate", "side_count", "side_d", "side_lat_abs", "side_vrel", "cutin", "text", "reason"])
        self.safe = False

    def update(self, now, level, reason, info):
        self.safe = False
        sn = info.get("side_near")
        self.w.writerow([f"{now:.3f}", "UPDATE", level, info.get("display", "?"), int(bool(info.get("buzzer"))), info.get("side_count", 0),
                         "" if not sn else f"{sn[0]:.1f}", "" if not sn else f"{sn[1]:.1f}", "" if not sn else f"{sn[2]:.1f}", int(bool(info.get("cutin"))),
                         LEVEL_TEXT.get(level, level), reason[:200]]); self.f.flush()

    def safe_state(self, why):
        if not self.safe:
            self.w.writerow([f"{time.monotonic():.3f}", "SAFE_STATE", "", "-", 0, "", "", "", "", "", "OFF(safe)", why]); self.f.flush()
        self.safe = True

    def close(self):
        self.f.close()


class NotWiredSink(OutputSink):
    """Attach point for the buzzer / OLED warning link (0x20/0x21). Intentionally NOT implemented: constructing it fails, so nobody can
    believe an output is connected. A real implementation must (1) honour safe_state(), (2) auto-expire its output (e.g. 500 ms after the last
    update) because the hub watchdog only covers THIS process, (3) not write to /dev/ttyAMA2 unless SERIAL_WRITE_OPTIONS.md has been decided."""
    def __init__(self, *a, **k):
        raise NotImplementedError("buzzer/OLED sink is an attach point only (wiring and Pi5->STM32 writes are not decided)")

    def update(self, *a): ...
    def safe_state(self, why): ...


class SinkHub:
    def __init__(self, sinks):
        self.sinks, self.errors, self.beat = sinks, 0, time.monotonic()
        self.safe = False
        self.lock = threading.Lock()

    def update(self, now, level, reason, info):
        self.beat = time.monotonic(); self.safe = False
        disp, buz = output_flags(level)
        if info.get("buzzer_inhibit"):
            buz = False                                  # S12: own speed unknown -> display only, no buzzer (option --buzz-without-speed lifts this)
        info = dict(info, display=disp, buzzer=buz)
        for s in self.sinks:
            try:
                s.update(now, level, reason, info)
            except Exception as e:                      # one broken sink must not stop the others
                self.errors += 1; print(f"[sink error] {type(s).__name__}: {e}", file=sys.stderr)

    def safe_state(self, why):
        with self.lock:
            if self.safe:
                return
            self.safe = True
        for s in self.sinks:
            try:
                s.safe_state(why)
            except Exception as e:
                self.errors += 1; print(f"[sink error] {type(s).__name__}.safe_state: {e}", file=sys.stderr)

    def start_watchdog(self):
        def run():
            while not self.safe:
                time.sleep(0.1)
                if time.monotonic() - self.beat > WATCHDOG_S:
                    self.safe_state(f"watchdog: main loop silent > {WATCHDOG_S:g} s")
        threading.Thread(target=run, daemon=True).start()


# ------------------------------------------------------------------ pipeline
class DriveLog:
    """S12-e: one CSV row per evaluation (20 Hz): the columns asked for the car test. radar_down = SAFE_MODE bit or RADAR_OK=0 (STM32 judgement, timeout 1000 ms on v18d2).
    power_suspect = 1 while the radar is down AND --radar-power is 'suspect' (default until the radar runs from the car's 12 V directly): such Safe/0xFF seconds
    are probably the radar supply sag, not the algorithm [가설]."""
    COLS = ["t_rel", "wall", "n_radar_obj", "tgt_id", "tgt_d_m", "tgt_lat_m", "tgt_vrel_ms", "ttc_s", "level", "level_text", "camera_confirmed", "camera_alive",
            "speed_kmh", "speed_src", "bsm_sent", "stm_status", "safe_mode", "radar_down", "radar_down_s", "power_suspect", "reason"]

    def __init__(self, path, a):
        self.f = open(path, "w", newline=""); self.w = csv.writer(self.f); self.power = a.radar_power
        self.f.write(f"# radar_power={a.radar_power} buzz_without_speed={a.buzz_without_speed} write_stm32={a.write_stm32} camera_udp_port={a.camera_udp_port} test_speed_kmh={a.test_speed_kmh} start={time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.w.writerow(self.COLS)

    def row(self, trel, wall, state, bo, lvl, cam_ok, cam, sp, st, down, down_s, bsm_sent):
        tg = bo.target
        src = {C.SOURCE_OBD: "OBD", C.SOURCE_GPS: "GPS"}.get(sp.source, "none")
        self.w.writerow([f"{trel:.3f}", f"{wall:.3f}", len(state.fresh_radar_objects(time.monotonic())), "" if tg is None else tg.id,
                         "" if tg is None else f"{tg.long_m:.1f}", "" if tg is None else f"{tg.lat_m:.1f}", "" if tg is None else f"{tg.vrel_ms:.2f}",
                         "" if bo.ttc is None else f"{bo.ttc:.2f}", lvl, LEVEL_TEXT.get(lvl, lvl), int(cam_ok), int(bool(cam and cam.alive(time.monotonic()))),
                         "" if sp.speed_kmh is None else f"{sp.speed_kmh:.1f}", src, bsm_sent, "" if st is None else st,
                         int(st is not None and bool(st & 0x10)), int(down), f"{down_s:.2f}", int(down and self.power == "suspect"), bo.reason[:120]])
        self.f.flush()

    def close(self):
        self.f.close()


def run(a, hub, buzzer=None):
    transport = V.make_transport(a.port, a.broadcast_addr, a.transport, a.serial_port)
    tracker = V.FreshnessTracker()
    gps = GpsPositionClient(port=a.gpsd_port)
    rec = Recorder(a.record, {"tool": "pipeline_main.py", "uart": a.uart, "start": time.time()}) if a.record else None
    stats = os.path.join(a.stats_dir, time.strftime("rx_stats_%Y%m%d_%H%M%S.csv")) if a.stats_dir else None
    rx = Receiver(gps, stats, 5.0, rec)
    brain = Brain(k_profile=a.k_profile, reset_k=a.reset_k, k_fixed=a.k_fixed)
    fd, _ = open_readonly(a.uart)                              # O_RDONLY: this program cannot write to the UART
    td = time.monotonic()
    while time.monotonic() - td < 2.0:                         # stale kernel backlog
        try:
            if not os.read(fd, 65536):
                break
        except BlockingIOError:
            break
    other = 2 if a.id == 1 else 1
    seq = int(time.time() * BSM_RATE_HZ) & 0xFFFF                # S12: clock-based start, a restarted sender is never behind its previous run (wraps every ~109 min)
    cam = CameraLink(a.camera_udp_port) if a.camera_udp_port else None
    dl = DriveLog(os.path.join(a.out, "drive_log.csv"), a)
    down_since = None
    t0 = time.monotonic(); next_eval = t0 + 0.2; next_tx = t0 + 0.5; last_bo = None; last_fields_t = 0
    cnt = dict(tx=0, rx_ok=0, rx_drop=0, evals=0)
    bsm_sent_last = 0
    inj_on = False; lvl_out = None
    def inj_msg(m):                                           # S13: visible even with the big screen: stderr file + inject_times.txt + stdout
        print(f"[TEST] {m}", flush=True); print(f"[TEST] {m}", file=sys.stderr, flush=True)
        with open(os.path.join(a.out, "inject_times.txt"), "a") as f: f.write(m + "\n")
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append("SIGINT")); signal.signal(signal.SIGTERM, lambda *_: stop.append("SIGTERM"))
    f_tx = open(os.path.join(a.out, "bsm_tx.csv"), "w", newline=""); w_tx = csv.writer(f_tx); w_tx.writerow(["t", "seq", "status_raw", "speed_kmh", "danger_active", "k"])
    try:
        while not stop and (a.duration <= 0 or time.monotonic() - t0 < a.duration):
            now = time.monotonic(); rel = now - t0
            if a.crash_at >= 0 and rel >= a.crash_at:
                raise RuntimeError("injected crash (test)")
            if a.hang_at >= 0 and rel >= a.hang_at:
                a.hang_at = -1; time.sleep(a.hang_for)          # injected main-loop hang (test of the watchdog)
            gps.connect(now)
            # UDP exposes a socket; ESP-NOW uses nonblocking serial, polled below.
            # Keep the <=20 ms wait bound so serial RX is serviced without a socket.
            transport_sock = getattr(transport, "sock", None)
            rl = [fd] + ([transport_sock] if transport_sock is not None else []) + ([gps.sock] if gps.sock else [])
            r, _, _ = select.select(rl, [], [], max(0.0, min(0.02, min(next_eval, next_tx) - now)))
            now = time.monotonic()
            for s in r:
                if s is fd:
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        data = b""
                    if data:
                        for fr in rx.feed(data, now, time.time()):
                            brain.on_frame(fr, now)
                elif gps.sock and s is gps.sock:
                    gps.pump(now)
            rx.poll(now); rx.maybe_stats(now, time.time())
            if cam is not None:
                cam.pump(rx.state, None if last_bo is None or last_bo.target is None else last_bo.target.id, now)
            while True:                                         # V2V receive
                d = transport.recv_bytes(0.0)
                if d is None:
                    break
                try:
                    dd = V.decode(d)
                except V.BSMDecodeError:
                    cnt["rx_drop"] += 1; continue
                if dd["vehicle_id"] != a.id and tracker.update(dd):
                    cnt["rx_ok"] += 1
            if now >= next_eval:
                next_eval += 1.0 / EVAL_HZ
                sp = get_speed(rx.state, gps, now)
                if a.test_speed_kmh is not None:                  # S17 TEST ONLY: fake own speed (default OFF); never use in a real drive
                    sp = SpeedResult(a.test_speed_kmh, C.SOURCE_OBD, 0, "TEST injected speed")
                peer = tracker.get_valid(other)
                own = gps.position(now)
                cam_ok = cam.confirmed(now) if cam is not None else False
                last_bo = brain.evaluate(rx.state, sp, peer, peer is None, now, own_fix=own, own_heading=gps.last_heading, camera_confirmed=cam_ok)
                sn = last_bo.side_nearest
                injecting = a.force_level2_at >= 0 and a.force_level2_at <= rel < a.force_level2_at + a.force_level2_for
                lvl = 2 if injecting else last_bo.level   # TEST injection (S13: also goes into the BSM DANGER_ACTIVE bit, so the front car can be tested)
                lvl_out = lvl
                if injecting and not inj_on:
                    inj_on = True; inj_msg(f"inject START level 2 at run t={rel:.1f}s  wall {time.strftime('%H:%M:%S')}")
                if not injecting and inj_on:
                    inj_on = False; inj_msg(f"inject END   at run t={rel:.1f}s  wall {time.strftime('%H:%M:%S')}")
                speed_unknown = sp.speed_kmh is None
                tg = last_bo.target
                hub.update(now, lvl, last_bo.reason, dict(k_r=last_bo.k_r, k_src=last_bo.k_src, supp=last_bo.supp, side_count=last_bo.side_count,
                                                                    side_near=None if sn is None else (sn.long_m, abs(sn.lat_m), sn.vrel_ms), cutin=last_bo.cutin,
                                                                    target_dist=None if tg is None else tg.long_m,
                                                                    buzzer_inhibit=speed_unknown and not a.buzz_without_speed, injected=injecting))
                st_now = rx.state.sensor_status(now); down = (not rx.state.radar_alive(now)) or (st_now is not None and bool(st_now & 0x10))
                if down and down_since is None: down_since = now
                if not down: down_since = None
                dl.row(now - t0, time.time(), rx.state, last_bo, lvl, cam_ok, cam, sp, st_now, down, 0.0 if down_since is None else now - down_since, bsm_sent_last)
                cnt["evals"] += 1
                last_fields = (sp, own)
            if now >= next_tx and last_bo is not None:
                next_tx += 1.0 / BSM_RATE_HZ
                sp, own = last_fields
                fields = build_fields(rx.state, sp, own, gps.last_heading, now, danger_level=lvl_out, k_measured=last_bo.k_measured)
                frame = V.encode(fields, a.id, seq)
                transport.send_bytes(frame); bsm_sent_last = 1
                dd = V.decode(frame)
                w_tx.writerow([f"{now:.3f}", seq, dd["status_raw"], dd["speed_kmh"], int(dd["danger_active"]), dd["k_value"]]); seq = (seq + 1) & 0xFFFF; cnt["tx"] += 1
        why = f"stopped by {stop[0]}" if stop else "duration reached"
    except BaseException as e:
        hub.safe_state(f"error: {type(e).__name__}: {e}")
        raise
    finally:
        hub.safe_state(locals().get("why", "exit"))
        brain.kest.finish()
        if buzzer is not None:                                   # report the buzzer pin state AFTER the sinks were put in safe state
            print(f"[buzzer] final: pin {'ON' if buzzer.on else 'OFF'}, on/off transitions {buzzer.transitions} ({'MOCK pin' if a.buzzer == 'mock' else 'REAL GPIO' + str(C.BUZZER_BCM_PIN)})", flush=True)
        f_tx.close(); transport.close(); os.close(fd); dl.close()
        if cam is not None:
            print(f"[camera] frames {cam.frames}, bad packets {cam.bad}, errors {cam.errors}", flush=True); cam.close()
        rx.close(time.monotonic(), time.time())
        if rec:
            rec.close()
        print("[pipeline_main]", cnt, flush=True)
    return cnt


def main():
    ap = argparse.ArgumentParser(description="integrated pipeline (simulation only; never writes to the UART)")
    ap.add_argument("--uart", default="/dev/ttyAMA2"); ap.add_argument("--gpsd-port", type=int, default=2947)
    ap.add_argument("--id", type=int, default=1); ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--broadcast-addr", default="127.255.255.255", help="default = loopback; for the real network use the subnet broadcast address")
    ap.add_argument("--transport", choices=["udp", "esp-now"], default="udp"); ap.add_argument("--serial-port", default="/dev/ttyUSB0")
    ap.add_argument("--duration", type=float, default=0); ap.add_argument("--out", default=".", help="folder for bsm_tx.csv / output_sink.csv")
    ap.add_argument("--record", metavar="DIR"); ap.add_argument("--stats-dir")
    ap.add_argument("--k-profile", default=None, help="e.g. ~/ESWContest_ws/state/k_profile.json (default: not stored)")
    ap.add_argument("--reset-k", action="store_true"); ap.add_argument("--k-fixed", type=float)
    ap.add_argument("--sinks", default="console,csv", help="comma list of console,csv  (buzzer/oled are attach points, not implemented)")
    ap.add_argument("--buzzer", choices=["off", "mock", "real"], default="off",
                    help="off (default) | mock = gpiozero MockFactory (tests, no pin touched) | real = BCM GPIO26 (needs --really-drive-gpio26)")
    ap.add_argument("--buzzer-type", choices=["active", "passive"], default="active", help="active = on/off, passive = PWM at BUZZER_PASSIVE_HZ (type is unconfirmed)")
    ap.add_argument("--really-drive-gpio26", action="store_true", help="REQUIRED together with --buzzer real; only after the user has approved driving the real pin")
    ap.add_argument("--camera-udp-port", type=int, default=0, help="S12: receive detections from camera_node.py on 127.0.0.1:PORT (0 = no camera, radar only)")
    ap.add_argument("--write-stm32", action="store_true", help="S12: send the own-car level to the STM32 as 0x20 frames (DEFAULT OFF). The board firmware v18d2 discards 0x20; see stm32_warn_sink.py")
    ap.add_argument("--write-path", default=None, help="serial path for --write-stm32 (default: same as --uart)")
    ap.add_argument("--buzz-without-speed", action="store_true", help="S12: allow the buzzer even when own speed is unknown (default: speed unknown -> display only, no buzzer)")
    ap.add_argument("--test-speed-kmh", type=float, default=None, help="S17 TEST ONLY: replace the own speed by this fake value (OBD source). Default OFF; real drives must not use it")
    ap.add_argument("--radar-power", choices=["suspect", "direct12v"], default="suspect", help="S12: log marker; 'suspect' = radar supply known to sag, Safe/0xFF rows are flagged power_suspect")
    ap.add_argument("--force-level2-at", type=float, default=-1, help="TEST: inject level 2 into the outputs from N s"); ap.add_argument("--force-level2-for", type=float, default=0)
    ap.add_argument("--affinity", default="", help="e.g. 2,3 (V2V cores, rule 3). Default: unchanged; or start with taskset")
    ap.add_argument("--crash-at", type=float, default=-1, help="TEST: raise an exception after N s")
    ap.add_argument("--hang-at", type=float, default=-1, help="TEST: block the main loop at N s"); ap.add_argument("--hang-for", type=float, default=3.0)
    a = ap.parse_args()
    if a.affinity:
        os.sched_setaffinity(0, {int(x) for x in a.affinity.split(",")})
    os.makedirs(a.out, exist_ok=True)
    sinks = []
    for name in [x for x in a.sinks.split(",") if x]:
        if name == "big":                                       # S12: big 0/1/2/? display for the passenger seat (terminal / monitor)
            from big_console_sink import BigConsoleSink
            sinks.append(BigConsoleSink()); continue
        sinks.append(ConsoleSink() if name == "console" else CsvSink(os.path.join(a.out, "output_sink.csv")) if name == "csv" else NotWiredSink())
    if a.write_stm32:
        from stm32_warn_sink import Stm32WarnSink
        wp = a.write_path or a.uart
        print(f"[write] --write-stm32 ON: 0x20 frames go to {wp} (one write() per frame, no termios)", flush=True)
        sinks.append(Stm32WarnSink(wp))
    buzzer = None
    if a.buzzer != "off":
        if a.buzzer == "real" and not a.really_drive_gpio26:
            print("REFUSED: --buzzer real drives the REAL pin BCM GPIO26 (physical 37). Add --really-drive-gpio26 only after the user approved it. Nothing was started.", file=sys.stderr)
            sys.exit(2)
        from buzzer_out import BuzzerSink
        if a.buzzer == "mock":
            from gpiozero.pins.mock import MockFactory, MockPWMPin
            buzzer = BuzzerSink(kind=a.buzzer_type, pin_factory=MockFactory(pin_class=MockPWMPin) if a.buzzer_type == "passive" else MockFactory())
        else:
            buzzer = BuzzerSink(kind=a.buzzer_type, really=True)
        sinks.append(buzzer)
    hub = SinkHub(sinks)
    hub.start_watchdog()
    try:
        run(a, hub, buzzer)
        rc = 0
    except Exception as e:
        print(f"[pipeline_main] ERROR {type(e).__name__}: {e}", file=sys.stderr)
        rc = 1
    finally:
        for s in sinks:
            s.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
