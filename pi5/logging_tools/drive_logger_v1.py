#!/usr/bin/env python3
"""주행 데이터 한꺼번에 녹화기 (Pi5 후방차용) v1

한 번 실행하면 아래를 전부 같은 시계(time.time)로 기록합니다. 읽기 전용이라 STM32/레이더 설정은 건드리지 않습니다.

  camera_000.mp4 ...   D435i 컬러 영상 (60초마다 파일을 나눔: 전원이 갑자기 꺼져도 마지막 1분만 잃음)
  camera_ts.csv        영상 프레임별 시각 (영상과 다른 데이터를 맞출 때 사용)
  frames.csv           /dev/ttyAMA2 로 들어오는 STM32 프레임 전부 (레이더 0x12, 요약 0x10, 하트비트 0x11,
                       IMU 0x13, OBD 0x14 ...)  time,type,ok,payload_hex,decoded
  radar.csv            위 프레임 중 레이더 물체(0x12)만 풀어 쓴 표
  stm_debug.log        /dev/ttyACM0 (STM32 디버그 글자: RADSTAT, OBJ, OBD, IMU 줄 등) 앞에 시각을 붙임
  gps.jsonl            gpsd 가 주는 GPS 원본(JSON) 한 줄씩 (속도, 위치, 고정 상태)
  sys.csv              Pi5 전원/온도 (throttled 값이 0x0 이 아니면 전원 부족 등)
  meta.json            실행 옵션, 시작 시각

사용:
  ~/realsense-venv/bin/python ~/drive_logger.py              # 끝낼 때 Ctrl+C
  ~/realsense-venv/bin/python ~/drive_logger.py --no-cam     # 카메라 없이 나머지만
  ~/realsense-venv/bin/python ~/drive_logger.py --note "혼다 앞, 직선 주행"

주의:
  - /dev/ttyAMA2, /dev/ttyACM0 는 한 번에 한 프로그램만 읽을 수 있습니다.
    radar_cam_overlay.py, cat, 다른 모니터링은 먼저 끄세요. (aiv-ego-speed 서비스는 쓰기만 해서 괜찮습니다.)
  - 저장 위치: ~/vehicle_test_logs/drive_날짜_시각/
"""
import argparse
import csv
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

HOME = os.path.expanduser("~")
sys.path.insert(0, HOME)

try:
    from aiv_uart_rx import decode as aiv_decode   # 있으면 프레임을 풀어서 같이 기록
except Exception:                                   # noqa: BLE001
    aiv_decode = None

stop = threading.Event()
stats_lock = threading.Lock()
stats = {}           # 소스별 누적 개수
last_seen = {}       # 소스별 마지막 수신 시각
status = {}          # 콘솔에 보여줄 마지막 값 (GPS 등)


def bump(name, n=1):
    with stats_lock:
        stats[name] = stats.get(name, 0) + n
        last_seen[name] = time.time()


def xor_chk(b):
    c = 0
    for x in b:
        c ^= x
    return c


class FrameParser:
    """STX 02 / TYPE / LEN / PAYLOAD / XOR(TYPE^LEN^PAYLOAD) / ETX 03"""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\x02")
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            n = self.buf[2]
            if n > 64:                       # 말이 안 되는 길이: 이 STX 는 가짜
                self.bad += 1
                del self.buf[:1]
                continue
            total = n + 5
            if len(self.buf) < total:
                break
            ok = self.buf[total - 1] == 0x03 and xor_chk(self.buf[1:n + 3]) == self.buf[n + 3]
            if ok:
                out.append((self.buf[1], bytes(self.buf[3:3 + n])))
                del self.buf[:total]
            else:
                self.bad += 1
                del self.buf[:1]
        return out


def open_csv(path, header):
    new = not os.path.exists(path)
    f = open(path, "a", newline="", encoding="utf-8")
    w = csv.writer(f)
    if new:
        w.writerow(header)
    return f, w


def flusher(files):
    while not stop.wait(1.0):
        for f in files:
            try:
                f.flush()
            except Exception:                # noqa: BLE001
                pass


# ---------------------------------------------------------------- STM32 프레임 (ttyAMA2)
def uart_thread(port, baud, outdir):
    import serial
    ff, fw = open_csv(os.path.join(outdir, "frames.csv"), ["time", "type", "payload_hex", "decoded"])
    rf, rw = open_csv(os.path.join(outdir, "radar.csv"),
                      ["time", "id", "long_m", "lat_m", "vrel_ms", "rcs_dbsm", "dyn"])
    threading.Thread(target=flusher, args=([ff, rf],), daemon=True).start()
    p = FrameParser()
    ser = None
    while not stop.is_set():
        try:
            if ser is None:
                ser = serial.Serial(port, baud, timeout=0.05)
                print(f"[연결] {port} {baud}")
            data = ser.read(512)
        except Exception as e:               # noqa: BLE001
            print(f"[경고] {port}: {e} -> 2초 뒤 다시 시도")
            ser = None
            stop.wait(2.0)
            continue
        if not data:
            continue
        t = time.time()
        bad0 = p.bad
        for typ, pl in p.feed(data):
            dec = ""
            d = None
            if aiv_decode is not None:
                try:
                    d = aiv_decode(typ, pl)
                    dec = json.dumps(d, default=str, ensure_ascii=False) if d is not None else ""
                except Exception:            # noqa: BLE001
                    d = None
            fw.writerow([f"{t:.4f}", f"0x{typ:02X}", pl.hex(), dec])
            bump(f"frame_0x{typ:02X}")
            bump("uart_frames")
            if typ == 0x12 and isinstance(d, dict) and "long_m" in d:
                rw.writerow([f"{t:.4f}", d.get("id"), d.get("long_m"), d.get("lat_m"),
                             d.get("vrel_ms"), d.get("rcs_dbsm"), d.get("dyn")])
        if p.bad > bad0:
            bump("frame_BAD", p.bad - bad0)
    ff.close()
    rf.close()


# ---------------------------------------------------------------- STM32 디버그 글자 (ttyACM0)
def debug_thread(port, baud, outdir):
    import serial
    f = open(os.path.join(outdir, "stm_debug.log"), "a", encoding="utf-8")
    threading.Thread(target=flusher, args=([f],), daemon=True).start()
    ser = None
    buf = b""
    while not stop.is_set():
        try:
            if ser is None:
                ser = serial.Serial(port, baud, timeout=0.1)
                print(f"[연결] {port} {baud}")
            data = ser.read(1024)
        except Exception as e:               # noqa: BLE001
            print(f"[경고] {port}: {e} -> 2초 뒤 다시 시도")
            ser = None
            stop.wait(2.0)
            continue
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            f.write(f"{time.time():.4f} {line.decode('utf-8', errors='replace').rstrip()}\n")
            bump("stm_debug_lines")
    f.close()


# ---------------------------------------------------------------- GPS (gpsd)
def gps_thread(host, port, outdir):
    f = open(os.path.join(outdir, "gps.jsonl"), "a", encoding="utf-8")
    threading.Thread(target=flusher, args=([f],), daemon=True).start()
    while not stop.is_set():
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(1.0)
            s.sendall(b'?WATCH={"enable":true,"json":true}\n')
            print(f"[연결] gpsd {host}:{port}")
            buf = b""
            while not stop.is_set():
                try:
                    data = s.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("gpsd 가 연결을 닫음")
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    f.write(f'{{"t":{time.time():.4f},"msg":{line.decode("utf-8", errors="replace")}}}\n')
                    try:
                        m = json.loads(line)
                    except ValueError:
                        continue
                    if m.get("class") == "TPV":
                        bump("gps_tpv")
                        status["gps"] = (m.get("mode"), m.get("speed"), m.get("lat"), m.get("lon"))
        except Exception as e:               # noqa: BLE001
            print(f"[경고] gpsd: {e} -> 2초 뒤 다시 시도")
            stop.wait(2.0)
    f.close()


# ---------------------------------------------------------------- Pi5 상태
def sys_thread(outdir):
    f, w = open_csv(os.path.join(outdir, "sys.csv"), ["time", "throttled", "temp_c"])
    threading.Thread(target=flusher, args=([f],), daemon=True).start()

    def run(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=2).stdout.strip()
        except Exception:                    # noqa: BLE001
            return ""
    while not stop.wait(2.0):
        thr = run(["vcgencmd", "get_throttled"]).replace("throttled=", "")
        tmp = run(["vcgencmd", "measure_temp"]).replace("temp=", "").replace("'C", "")
        w.writerow([f"{time.time():.3f}", thr, tmp])
        status["thr"] = thr
        status["temp"] = tmp
    f.close()


# ---------------------------------------------------------------- 카메라
class Segmenter:
    """60초마다 새 mp4 파일을 만든다. (갑자기 꺼져도 앞부분은 살아 있음)"""

    def __init__(self, outdir, fps, size, seg_sec):
        self.outdir, self.fps, self.size, self.seg_sec = outdir, fps, size, seg_sec
        self.w = None
        self.seg = -1
        self.seg_start = 0.0
        self.idx_in_seg = 0

    def write(self, frame):
        import cv2
        now = time.time()
        if self.w is None or now - self.seg_start >= self.seg_sec:
            if self.w is not None:
                self.w.release()
            self.seg += 1
            self.seg_start = now
            self.idx_in_seg = 0
            path = os.path.join(self.outdir, f"camera_{self.seg:03d}.mp4")
            self.w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, self.size)
            if not self.w.isOpened():
                raise RuntimeError(f"영상 파일을 열 수 없음: {path}")
        self.w.write(frame)
        self.idx_in_seg += 1
        return self.seg, self.idx_in_seg - 1

    def close(self):
        if self.w is not None:
            self.w.release()
            self.w = None


def camera_thread(outdir, fps, sim):
    import numpy as np
    f, w = open_csv(os.path.join(outdir, "camera_ts.csv"),
                    ["time", "global_idx", "segment", "idx_in_segment", "device_ts_ms"])
    threading.Thread(target=flusher, args=([f],), daemon=True).start()
    seg = Segmenter(outdir, fps, (640, 480), 60.0)
    gi = 0
    pipe = None
    while not stop.is_set():
        try:
            if sim:
                t0 = time.time()
                img = np.zeros((480, 640, 3), np.uint8)
                img[:, :, 1] = (gi * 3) % 255
                import cv2
                cv2.putText(img, f"SIM {gi}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                dev_ts = t0 * 1000
                stop.wait(max(0.0, 1.0 / fps - (time.time() - t0)))
            else:
                import pyrealsense2 as rs
                if pipe is None:
                    pipe = rs.pipeline()
                    cfg = rs.config()
                    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
                    pipe.start(cfg)
                    print("[연결] D435i 컬러 640x480 @%d" % fps)
                fr = pipe.wait_for_frames(5000).get_color_frame()
                if not fr:
                    continue
                img = np.asanyarray(fr.get_data())
                dev_ts = fr.get_timestamp()
            t = time.time()
            s, k = seg.write(img)
            w.writerow([f"{t:.4f}", gi, s, k, f"{dev_ts:.1f}"])
            gi += 1
            bump("camera_frames")
        except Exception as e:               # noqa: BLE001
            print(f"[경고] 카메라: {e} -> 2초 뒤 다시 시도 (D435i 를 뽑았다 꽂아야 할 수도 있음)")
            try:
                if pipe is not None:
                    pipe.stop()
            except Exception:                # noqa: BLE001
                pass
            pipe = None
            stop.wait(2.0)
    try:
        if pipe is not None:
            pipe.stop()
    except Exception:                        # noqa: BLE001
        pass
    seg.close()
    f.close()


# ---------------------------------------------------------------- 콘솔 요약
def reporter(outdir, expect):
    prev = {}
    t_prev = time.time()
    while not stop.wait(5.0):
        now = time.time()
        dt = now - t_prev
        t_prev = now
        with stats_lock:
            cur = dict(stats)
            seen = dict(last_seen)
        parts = []
        warn = []
        for k in sorted(cur):
            parts.append(f"{k}={cur[k] - prev.get(k, 0)}/{dt:.0f}s")
        for name in expect:
            if now - seen.get(name, 0) > 3.0:
                warn.append(name)
        prev = cur
        free = shutil.disk_usage(outdir).free / 1e9
        g = status.get("gps")
        gtxt = "GPS 없음" if not g else f"GPS mode={g[0]} spd={g[1]}"
        print(f"[{time.strftime('%H:%M:%S')}] {' '.join(parts)} | {gtxt} | thr={status.get('thr', '?')} "
              f"temp={status.get('temp', '?')} | 남은 디스크 {free:.1f} GB")
        if warn:
            print("   !!! 3초 넘게 데이터가 안 들어옴:", ", ".join(warn))
        if free < 1.0:
            print("   !!! 디스크가 1 GB 미만입니다. 곧 멈춥니다. 오래된 로그를 지우세요.")


def main():
    ap = argparse.ArgumentParser(description="주행 데이터 통합 녹화기 (읽기 전용)")
    ap.add_argument("--out", default=os.path.join(HOME, "vehicle_test_logs"))
    ap.add_argument("--uart", default="/dev/ttyAMA2")
    ap.add_argument("--uart-baud", type=int, default=115200)
    ap.add_argument("--debug-port", default="/dev/ttyACM0")
    ap.add_argument("--debug-baud", type=int, default=115200)
    ap.add_argument("--gpsd", default="127.0.0.1:2947")
    ap.add_argument("--fps", type=int, default=30, help="카메라 프레임 수 (D435i 컬러 640x480: 30 또는 15)")
    ap.add_argument("--no-cam", action="store_true")
    ap.add_argument("--no-debug", action="store_true", help="ttyACM0(STM32 디버그 글자)를 읽지 않음")
    ap.add_argument("--no-gps", action="store_true")
    ap.add_argument("--sim-cam", action="store_true", help="시험용 가짜 영상 (카메라 없이 동작 확인)")
    ap.add_argument("--note", default="", help="이 주행에 대한 메모 (meta.json 에 저장)")
    a = ap.parse_args()

    missing = []
    for mod in ("serial",) + (() if a.no_cam else ("cv2", "numpy")):
        try:
            __import__(mod)
        except ImportError:
            missing.append(mod)
    if not a.no_cam and not a.sim_cam:
        try:
            __import__("pyrealsense2")
        except ImportError:
            missing.append("pyrealsense2")
    if missing:
        sys.exit("없는 모듈: %s\n  -> ~/realsense-venv/bin/python 으로 실행했는지 확인하세요. "
                 "(pyrealsense2 는 그 안에만 있음)\n  -> pyserial/opencv 가 없다고 나오면: "
                 "~/realsense-venv/bin/pip install pyserial opencv-python-headless numpy" % ", ".join(missing))

    outdir = os.path.join(os.path.expanduser(a.out), "drive_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    free = shutil.disk_usage(outdir).free / 1e9
    print(f"저장 폴더: {outdir}\n남은 디스크: {free:.1f} GB (카메라 30fps 기준 1분에 약 30~60 MB)")
    if free < 2.0:
        sys.exit("디스크가 2 GB 미만이라 시작하지 않습니다. 오래된 로그를 지우고 다시 실행하세요.")

    meta = dict(start_time=time.time(), start_local=time.strftime("%Y-%m-%d %H:%M:%S"),
                args=vars(a), hostname=socket.gethostname(), python=sys.version.split()[0],
                aiv_uart_rx_decode=bool(aiv_decode))
    with open(os.path.join(outdir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)

    def on_sig(*_):
        stop.set()
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    th = [threading.Thread(target=uart_thread, args=(a.uart, a.uart_baud, outdir), daemon=True),
          threading.Thread(target=sys_thread, args=(outdir,), daemon=True)]
    expect = ["uart_frames"]
    if not a.no_debug:
        th.append(threading.Thread(target=debug_thread, args=(a.debug_port, a.debug_baud, outdir), daemon=True))
        expect.append("stm_debug_lines")
    if not a.no_gps:
        h, _, p = a.gpsd.partition(":")
        th.append(threading.Thread(target=gps_thread, args=(h, int(p or 2947), outdir), daemon=True))
        expect.append("gps_tpv")
    if not a.no_cam:
        th.append(threading.Thread(target=camera_thread, args=(outdir, a.fps, a.sim_cam), daemon=True))
        expect.append("camera_frames")
    th.append(threading.Thread(target=reporter, args=(outdir, expect), daemon=True))
    for t in th:
        t.start()
    print("녹화 시작. 끝내려면 Ctrl+C 한 번 (마무리에 몇 초 걸립니다).")
    while not stop.is_set():
        stop.wait(0.5)
    print("마무리 중...")
    time.sleep(1.5)
    with stats_lock:
        total = dict(stats)
    print("총 개수:", total)
    print("저장 완료:", outdir)


if __name__ == "__main__":
    main()
