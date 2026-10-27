#!/usr/bin/env python3
"""
레이더 물체를 D435i 컬러 영상 위에 점으로 그려주는 확인/보정 도구 (Pi5용) v2

사용법 (우리 장착 기준: 카메라가 레이더보다 150cm 뒤, 75cm 위):
    python3 radar_cam_overlay.py --dx 1.5 --h 0.75
    -> 노트북 브라우저에서  http://<파이5 IP>:8080   (IP는 hostname -I)
    -> 화면의 버튼으로 좌우/상하 각도를 조금씩 맞춤, [저장] 누르면 ~/radar_cam_calib.json 에 기록
    python3 radar_cam_overlay.py --load       # 저장해 둔 값으로 시작
    python3 radar_cam_overlay.py --sim        # 카메라/레이더 없이 화면만 시험

값의 의미 (모두 미터/도):
    dx   : 카메라가 레이더보다 "뒤쪽"으로 떨어진 거리 (+면 카메라가 더 뒤)
    dy   : 카메라가 레이더보다 "오른쪽"으로 떨어진 거리 (+면 카메라가 오른쪽)
    h    : 카메라가 레이더 높이보다 위로 떨어진 거리
    yaw  : +면 점이 화면에서 오른쪽으로 이동
    pitch: +면 점이 화면에서 아래로 이동
    flip : 레이더 lat(+)이 화면 오른쪽이면 켬 (기본: lat+ = 왼쪽)
* 이 도구는 확인/보정용입니다. 안전 판단에는 쓰지 않습니다.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("opencv가 없습니다:  sudo apt install -y python3-opencv  (설치 후 다시 실행)")

sys.path.insert(0, os.path.expanduser("~"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aiv_uart_rx import FrameParser, decode   # noqa: E402

W, H = 640, 480
FX, FY, CX, CY = 607.16, 607.34, 321.98, 240.56   # D435i 컬러 640x480 공장 보정값
MAX_AGE = 0.4
MAP_RANGE_M = 30.0
CALIB_PATH = os.path.expanduser("~/radar_cam_calib.json")

P = dict(dx=1.5, dy=0.0, h=0.75, yaw=0.0, pitch=0.0, flip=0)   # 현재 보정값 (버튼으로 바뀜)
P_lock = threading.Lock()

objs = {}
objs_lock = threading.Lock()
latest_jpg = b""
jpg_lock = threading.Lock()

STEP = dict(dx=0.05, dy=0.05, h=0.05, yaw=0.25, pitch=0.25)


def radar_thread(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=0.05)
    p = FrameParser()
    while True:
        data = ser.read(256)
        if not data:
            continue
        for t, pl in p.feed(data):
            if t == 0x12:
                d = decode(t, pl)
                d["seen"] = time.time()
                with objs_lock:
                    objs[d["id"]] = d


def sim_thread():
    t0 = time.time()
    while True:
        t = time.time() - t0
        with objs_lock:
            objs[1] = dict(id=1, long_m=10 + 3 * math.sin(t), lat_m=-2 + 4 * (math.sin(t / 3) + 1) / 2,
                           vrel_ms=-1.2, rcs_dbsm=12.0, dyn=0, seen=time.time())
            objs[2] = dict(id=2, long_m=25.0, lat_m=3.0, vrel_ms=0.0, rcs_dbsm=3.0, dyn=1, seen=time.time())
        time.sleep(0.05)


def project(long_m, lat_m, p):
    """레이더 좌표(앞 long, 좌우 lat) -> 화면 픽셀.
    카메라 기준으로 옮김: 앞쪽 거리 X = long + dx (카메라가 뒤에 있으니 더 멀다),
    오른쪽 거리 = (lat+ 가 왼쪽이면 -lat) - dy, 레이더 높이의 점은 카메라보다 h 만큼 아래."""
    X = long_m + p["dx"]
    if X < 1.0:
        return None
    right = (lat_m if p["flip"] else -lat_m) - p["dy"]
    ang_h = math.atan2(right, X) + math.radians(p["yaw"])
    ang_v = math.atan2(p["h"], X) + math.radians(p["pitch"])
    return CX + FX * math.tan(ang_h), CY + FY * math.tan(ang_v)


def color_of(o):
    return {0: (0, 200, 255), 1: (160, 160, 160), 2: (0, 0, 255)}.get(o["dyn"], (0, 255, 0))


def draw(frame, snap, p):
    cv2.putText(frame, "dx=%.2f dy=%.2f h=%.2f yaw=%.2f pitch=%.2f flip=%d  objs=%d" % (
        p["dx"], p["dy"], p["h"], p["yaw"], p["pitch"], p["flip"], len(snap)),
        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (int(CX), 0), (int(CX), H), (80, 80, 80), 1)       # 화면 중앙 세로선
    for o in snap:
        pt = project(o["long_m"], o["lat_m"], p)
        if pt is None:
            continue
        u, v = int(pt[0]), int(pt[1])
        if 0 <= u < W and 0 <= v < H:
            c = color_of(o)
            cv2.circle(frame, (u, v), 7, c, 2)
            cv2.putText(frame, "id%d %.1fm lat%.1f v%.1f" % (o["id"], o["long_m"], o["lat_m"], o["vrel_ms"]),
                        (u + 9, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)
    mw, mh = 170, 190   # 미니맵: 아래가 레이더, 위가 앞쪽, 오른쪽 = lat 값이 +
    m = np.zeros((mh, mw, 3), np.uint8)
    cv2.rectangle(m, (0, 0), (mw - 1, mh - 1), (90, 90, 90), 1)
    for r in (10, 20, 30):
        y = int(mh - 10 - r / MAP_RANGE_M * (mh - 20))
        cv2.line(m, (0, y), (mw, y), (60, 60, 60), 1)
        cv2.putText(m, "%dm" % r, (3, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
    cv2.line(m, (mw // 2, 0), (mw // 2, mh), (60, 60, 60), 1)
    cv2.putText(m, "lat+ ->", (mw - 62, mh - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
    for o in snap:
        if 0 < o["long_m"] <= MAP_RANGE_M and abs(o["lat_m"]) <= 10:
            x = int(mw / 2 + o["lat_m"] / 10 * (mw / 2 - 5))
            y = int(mh - 10 - o["long_m"] / MAP_RANGE_M * (mh - 20))
            cv2.circle(m, (x, y), 4, color_of(o), -1)
    frame[H - mh - 6:H - 6, W - mw - 6:W - 6] = m
    return frame


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>radar-cam</title><style>
body{font-family:sans-serif;background:#111;color:#eee;margin:8px}
img{max-width:100%;border:1px solid #444}
.row{margin:6px 0}.row b{display:inline-block;width:64px}
button{font-size:16px;padding:6px 12px;margin:2px;background:#333;color:#eee;border:1px solid #666;border-radius:6px}
#msg{color:#7fdc7f}
</style>
<img src="/stream">
<div class=row><b>좌우각도</b><button onclick="a('yaw',-1)">◀ 점을 왼쪽</button><button onclick="a('yaw',1)">점을 오른쪽 ▶</button></div>
<div class=row><b>상하각도</b><button onclick="a('pitch',-1)">▲ 점을 위로</button><button onclick="a('pitch',1)">점을 아래로 ▼</button></div>
<div class=row><b>높이 h</b><button onclick="a('h',-1)">−</button><button onclick="a('h',1)">+</button>
<b>앞뒤 dx</b><button onclick="a('dx',-1)">−</button><button onclick="a('dx',1)">+</button>
<b>좌우 dy</b><button onclick="a('dy',-1)">−</button><button onclick="a('dy',1)">+</button></div>
<div class=row><button onclick="f()">좌우 뒤집기(flip)</button><button onclick="s()">💾 저장</button> <span id=msg></span></div>
<script>
function g(u){fetch(u).then(r=>r.text()).then(t=>document.getElementById('msg').textContent=t)}
function a(k,d){g('/adj?k='+k+'&d='+d)} function f(){g('/flip')} function s(){g('/save')}
</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _txt(self, s):
        b = s.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/":
            b = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif u.path == "/adj":
            k = q.get("k", [""])[0]
            d = float(q.get("d", ["0"])[0])
            if k in STEP:
                with P_lock:
                    P[k] = round(P[k] + d * STEP[k], 3)
                    self._txt("%s = %s" % (k, P[k]))
            else:
                self._txt("?")
        elif u.path == "/flip":
            with P_lock:
                P["flip"] = 0 if P["flip"] else 1
                self._txt("flip = %d" % P["flip"])
        elif u.path == "/save":
            with P_lock:
                json.dump(P, open(CALIB_PATH, "w"), indent=1)
                self._txt("저장됨: %s  %s" % (CALIB_PATH, dict(P)))
        elif u.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with jpg_lock:
                        j = latest_jpg
                    if j:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j))
                        self.wfile.write(j + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


def main():
    global latest_jpg
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA2")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dx", type=float, help="카메라가 레이더보다 뒤쪽으로 (m)")
    ap.add_argument("--dy", type=float, help="카메라가 레이더보다 오른쪽으로 (m)")
    ap.add_argument("--h", type=float, help="카메라가 레이더보다 위로 (m)")
    ap.add_argument("--yaw", type=float)
    ap.add_argument("--pitch", type=float)
    ap.add_argument("--flip", action="store_true")
    ap.add_argument("--load", action="store_true", help="저장해 둔 값으로 시작")
    ap.add_argument("--window", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--web-port", type=int, default=8080)
    a = ap.parse_args()

    if a.load and os.path.exists(CALIB_PATH):
        P.update(json.load(open(CALIB_PATH)))
        print("불러옴:", P)
    for k in ("dx", "dy", "h", "yaw", "pitch"):
        if getattr(a, k) is not None:
            P[k] = getattr(a, k)
    if a.flip:
        P["flip"] = 1

    if a.sim:
        threading.Thread(target=sim_thread, daemon=True).start()
        pipe = None
    else:
        import pyrealsense2 as rs
        threading.Thread(target=radar_thread, args=(a.port, a.baud), daemon=True).start()
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        pipe.start(cfg)

    if not a.window:
        srv = ThreadingHTTPServer(("0.0.0.0", a.web_port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print("브라우저에서 열기:  http://<파이5 IP>:%d   (파이5 IP는 hostname -I)" % a.web_port)
    print("시작 값:", P)

    try:
        while True:
            if pipe:
                fr = pipe.wait_for_frames().get_color_frame()
                if not fr:
                    continue
                frame = np.asanyarray(fr.get_data()).copy()
            else:
                frame = np.full((H, W, 3), 40, np.uint8)
                time.sleep(0.03)
            now = time.time()
            with objs_lock:
                for k in [k for k, o in objs.items() if now - o["seen"] > MAX_AGE]:
                    del objs[k]
                snap = list(objs.values())
            with P_lock:
                p = dict(P)
            frame = draw(frame, snap, p)
            if a.window:
                cv2.imshow("radar overlay", frame)
                if cv2.waitKey(1) == 27:
                    break
            else:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with jpg_lock:
                        latest_jpg = buf.tobytes()
    except KeyboardInterrupt:
        pass
    finally:
        print("최종 값:", P)
        if pipe:
            pipe.stop()


if __name__ == "__main__":
    main()
