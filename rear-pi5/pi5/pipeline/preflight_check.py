#!/usr/bin/env python3
"""Pre-installation check for the car test (READ-ONLY): one PASS / WARN / FAIL table. Opens NO serial port, writes NO file, sends nothing.
Uses only: systemctl, fuser, /proc, df, timedatectl, vcgencmd, os.access, reading state/k_profile.json and source files, and one read-only subscription (?WATCH) to localhost gpsd:2947 to see the fix mode.
usage: python3 -I pi5/pipeline/preflight_check.py          (exit code 1 if any FAIL)"""
import json, os, re, shutil, subprocess, sys, time

WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HOME = os.path.expanduser("~")
rows = []


def run(*cmd, timeout=4):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (r.stdout + r.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return f"<{type(e).__name__}>"


def add(name, status, detail):
    rows.append((name, status, detail))


def cmdline(pid):
    try:
        return open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode().strip()
    except OSError:
        return "?"


IGNORED = {"grep", "ugrep", "rg", "timeout", "bash", "sh", "tmux", "ps", "pgrep", "less", "tail", "cat", "watch", "sudo", "nohup", "vim", "nano", "code"}


def ignorable(cmd):
    """Wrappers / search tools whose command line merely CONTAINS a program name."""
    first = os.path.basename(cmd.split()[0]) if cmd.split() else ""
    return first in IGNORED


def holders(path):
    out = subprocess.run(["fuser", path], capture_output=True, text=True).stdout.split()
    return [(int(p), cmdline(int(p))) for p in out if p.isdigit()]


# 1-2 services
for svc in ("aiv-ego-speed", "gpsd"):
    st = run("systemctl", "is-active", svc)
    add(f"service {svc}", "PASS" if st == "active" else "FAIL", st)
# 3 /dev/ttyAMA2 owner(s)
if os.path.exists("/dev/ttyAMA2"):
    h = holders("/dev/ttyAMA2")
    bad = [x for x in h if "ego_speed_tx.py" not in x[1]]
    add("/dev/ttyAMA2 holders", "PASS" if (h and not bad) else ("FAIL" if bad else "WARN"),
        ("only the write-only ego_speed_tx.py service: " + ", ".join(f"PID {p}" for p, _ in h)) if (h and not bad) else
        ("OTHER process(es) hold the UART -> a second reader would split the bytes: " + "; ".join(f"PID {p} {c[:70]}" for p, c in bad)) if bad else "nobody holds it (is aiv-ego-speed running?)")
else:
    add("/dev/ttyAMA2 exists", "FAIL", "device missing (uart2-pi5 overlay?)")
# 4 GPS: gpsd process, the device it reads, and (read-only query/subscription on localhost:2947) the fix mode. fuser cannot see other users' processes (gpsd runs as
#   its own user), so the process list (/proc/*/cmdline is world-readable) and gpsd's own answers are used instead.
import socket
gp = [(int(x), cmdline(int(x))) for x in filter(str.isdigit, os.listdir("/proc"))]
gp = [(p_, c) for p_, c in gp if re.match(r"\S*gpsd\b", c) and "-n" in c.split()]
gpsd_dev = re.search(r"(/dev/\S+)", gp[0][1]).group(1) if gp else None
mode, dev_info = None, None
try:
    sk = socket.create_connection(("127.0.0.1", 2947), timeout=1.0)
    sk.sendall(b'?WATCH={"enable":true,"json":true};\n'); sk.settimeout(0.4)
    buf, t_end = b"", time.time() + 1.8
    while time.time() < t_end:
        try:
            buf += sk.recv(8192)
        except socket.timeout:
            pass
    sk.close()
    for line in buf.split(b"\n"):
        try:
            m_ = json.loads(line)
        except ValueError:
            continue
        if m_.get("class") == "TPV":
            mode = m_.get("mode")
        if m_.get("class") == "DEVICES" and m_.get("devices"):
            dev_info = m_["devices"][0]
except OSError:
    pass
add("gpsd reads the GPS device", "PASS" if (gpsd_dev == "/dev/ttyAMA0" and dev_info) else "FAIL",
    f"gpsd PID {gp[0][0]} reads {gpsd_dev}; gpsd reports {dev_info.get('path')} {dev_info.get('bps')} bps, driver {dev_info.get('driver')}" if (gp and dev_info) else
    ("gpsd process found but it did not answer on localhost:2947" if gp else "no gpsd process with -n found"))
add("GPS fix (gpsd TPV)", "PASS" if (mode or 0) >= 2 else "WARN", f"mode {mode} ({'3D' if mode == 3 else '2D' if mode == 2 else 'NO FIX: needs open sky, can take minutes; speed falls back to NONE if OBD is missing too'})" if mode is not None else "no TPV received within 1.8 s")
# other UART readers (any owner) -- a second reader of /dev/ttyAMA2 splits the bytes
readers = []
for x in filter(str.isdigit, os.listdir("/proc")):
    c = cmdline(int(x))
    if re.search(r"live_receiver|pipeline_main|bsm_node.*--uart|drive_logger(?!_nouart)|aiv_uart_rx|radar_receiver|uart_status_receiver|vehicle_algorithm/main\.py|ttyAMA2", c) and "preflight_check" not in c and "ego_speed_tx" not in c and not ignorable(c):
        readers.append((int(x), c[:80]))
add("other processes that may read the UART", "PASS" if not readers else "WARN", "none running" if not readers else "; ".join(f"PID {p} {c}" for p, c in readers) + " (only ONE may read /dev/ttyAMA2)")
# STM32 debug port and ESP32/CP2102
acm = holders("/dev/ttyACM0") if os.path.exists("/dev/ttyACM0") else None
add("/dev/ttyACM0 (ST-Link debug text)", "PASS" if acm == [] else ("WARN" if acm is None else "FAIL"),
    "present, nobody reads it (drive_logger_nouart will)" if acm == [] else ("not present: ST-Link/STM32 USB not connected" if acm is None else "held by " + "; ".join(f"PID {p} {c[:50]}" for p, c in acm)))
by_id = sorted(os.listdir("/dev/serial/by-id")) if os.path.isdir("/dev/serial/by-id") else []
add("USB serial devices", "PASS" if by_id else "WARN", ", ".join(b[:60] for b in by_id) or "none (ST-Link / ESP32 CP2102 not plugged)")
# 5 disk
du = shutil.disk_usage(HOME)
free = du.free / 1e9
add("disk free (home)", "PASS" if free >= 10 else "FAIL", f"{free:.1f} GB free (>= 10 GB wanted; drive_logger refuses to start below 2 GB; video ~30-60 MB/min)")
# 6 time
ts = run("timedatectl", "show", "-p", "NTPSynchronized", "--value")
add("system time", "PASS" if ts == "yes" else "WARN", f"{time.strftime('%Y-%m-%d %H:%M:%S %Z')}; NTP synchronized: {ts or '?'} (two cars have unsynchronised clocks anyway; note the time in the log notes)")
# 7 log folder writable
lf = os.path.join(HOME, "vehicle_test_logs")
if os.path.isdir(lf):
    newest = sorted((d for d in os.listdir(lf) if d.startswith("drive_")), reverse=True)[:1]
    add("log folder ~/vehicle_test_logs", "PASS" if os.access(lf, os.W_OK) else "FAIL", f"exists, writable={os.access(lf, os.W_OK)}; newest drive folder: {newest[0] if newest else 'none'}")
else:
    parent_ok = os.access(HOME, os.W_OK)
    add("log folder ~/vehicle_test_logs", "PASS" if parent_ok else "FAIL", "does not exist yet; drive_logger will create it (home writable)" if parent_ok else "home not writable")
# 8 K profile
kp = os.path.join(WS, "state", "k_profile.json")
if os.path.exists(kp):
    try:
        d = json.load(open(kp))
        add("state/k_profile.json", "PASS", f"valid JSON; bias {d.get('bias_mg')} mg, {d.get('n_events')} event(s), K {d.get('k')} (N/A if fewer than 3 events), updated {d.get('updated')}")
    except (OSError, ValueError) as e:
        add("state/k_profile.json", "FAIL", f"unreadable/corrupt: {e}")
else:
    add("state/k_profile.json", "WARN", "absent: K is N/A until 3 braking events are measured (CAR_TEST_RUNBOOK §7 'K profile'); the default 0.5 is used for calculations only")
# 9 core separation
ncpu = os.cpu_count()
procs = []
for pid in filter(str.isdigit, os.listdir("/proc")):
    c = cmdline(int(pid))
    kind = "yolo" if re.search(r"camera_detector|vehicle_algorithm/main\.py|yolo|lead_replay|frame_check", c) else "v2v" if re.search(r"pipeline_main|bsm_node|live_receiver|v2v_node", c) else None
    if kind and "preflight_check" not in c and not ignorable(c):
        al = re.search(r"Cpus_allowed_list:\s*(\S+)", open(f"/proc/{pid}/status").read()) if os.path.exists(f"/proc/{pid}/status") else None
        procs.append((kind, int(pid), al.group(1) if al else "?", c[:60]))
if not procs:
    add("core separation (YOLO 0-1, V2V 2-3)", "PASS" if ncpu == 4 else "WARN", f"{ncpu} CPUs; no YOLO/V2V process running now, so nothing to check. Start with taskset -c 0,1 (camera) and -c 2,3 (pipeline)")
else:
    bad = [p for p in procs if (p[0] == "yolo" and p[2] != "0-1") or (p[0] == "v2v" and p[2] != "2-3")]
    add("core separation (YOLO 0-1, V2V 2-3)", "FAIL" if bad else "PASS", "; ".join(f"{k} PID {pid} cpus {al} ({c})" for k, pid, al, c in procs))
# 10 vision package
vl = os.path.join(WS, "pi5", "vision_lead", "vehicle_algorithm")
need = ["fusion.py", "main.py", "radar_receiver.py", "camera_detector.py", "tracking_diagnostics.py"]
miss = [f for f in need if not os.path.exists(os.path.join(vl, f))]
ncnn = os.path.isdir(os.path.join(HOME, "yolov8n_ncnn_model"))
add("vision package (pi5/vision_lead)", "PASS" if (not miss and ncnn) else "FAIL", f"files {'all present' if not miss else 'MISSING ' + ', '.join(miss)}; ~/yolov8n_ncnn_model {'present' if ncnn else 'MISSING'}")
rr = os.path.join(vl, "radar_receiver.py")
if os.path.exists(rr):
    txt = open(rr).read()
    port = re.search(r"DEFAULT_PORT\s*=\s*'([^']+)'", txt)
    excl = "exclusive=True" in txt
    add("vision radar_receiver.py default port", "WARN" if (port and "CP2102" in port.group(1)) or excl else "PASS",
        f"DEFAULT_PORT = {port.group(1)[:62] if port else '?'}...; exclusive={excl}. DO NOT run the package's main.py / radar_receiver.py in the car: it would grab the CP2102 (ESP32?) port exclusively, and with another port it would compete with the pipeline for the UART")
running = [(p, c) for p, c in [(int(x), cmdline(int(x))) for x in filter(str.isdigit, os.listdir("/proc"))] if re.search(r"vehicle_algorithm/(main|radar_receiver|uart_status_receiver)\.py|vision_lead.*main\.py", c)]
add("vision main.py/radar_receiver running", "FAIL" if running else "PASS", ("RUNNING: " + "; ".join(f"PID {p} {c[:60]}" for p, c in running)) if running else "not running")
# power
thr = run("vcgencmd", "get_throttled")
add("Pi5 throttled", "PASS" if thr.endswith("0x0") else "FAIL", thr)
m = re.search(r"EXT5V_V volt\(\d+\)=([0-9.]+)V", run("vcgencmd", "pmic_read_adc"))
add("Pi5 EXT5V_V", ("PASS" if m and float(m.group(1)) >= 5.0 else "FAIL") if m else "WARN", f"{m.group(1)} V (>= 5.0 V wanted)" if m else "could not read")
# leftovers from tests
busy = [p for p in (19000, 22947) if re.search(rf":{p}\b", run("ss", "-uln") + run("ss", "-tln"))]
add("test ports 19000/22947 free", "PASS" if not busy else "WARN", "free" if not busy else f"in use: {busy} (leftover test processes?)")

w = max(len(r[0]) for r in rows)
print(f"{'check'.ljust(w)} | status | detail\n{'-' * w}-+--------+-------")
for n, s, d in rows:
    print(f"{n.ljust(w)} | {s.ljust(6)} | {d}")
nf = sum(1 for r in rows if r[1] == "FAIL"); nw = sum(1 for r in rows if r[1] == "WARN")
print(f"\n{len(rows) - nf - nw} PASS, {nw} WARN, {nf} FAIL   (read-only: no port opened, nothing written)")
sys.exit(1 if nf else 0)
