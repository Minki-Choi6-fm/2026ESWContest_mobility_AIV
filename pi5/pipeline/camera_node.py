#!/usr/bin/env python3
"""S12-b: camera/YOLO process (own process, CPU cores 0-1) -> sends detections to pipeline_main over local UDP (127.0.0.1).

!! NOT RUN in the S12 session: no camera was connected and the instruction was not to open one. Start it only when the user says "카메라 시작".
Backends:  --source realsense   the lead package's CameraDetector (RealSense RGB + YOLO NCNN, pi5/vision_lead/vehicle_algorithm/camera_detector.py)
           --source video FILE  recorded video through the same YOLO (for an offline check)   [needs ultralytics + the model]
It never writes to any serial port and never touches the radar. If the camera or YOLO dies it keeps retrying (so a replugged camera comes back);
the pipeline treats missing packets as camera_confirmed=False and continues radar-only.

usage: taskset -c 0,1 python3 camera_node.py --source realsense --port 19100
Packet: see camera_link.py.
"""
import argparse
import json
import os
import queue
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.environ.get("VISION_FUSION_DIR") or os.path.abspath(os.path.join(HERE, "..", "vision_lead", "vehicle_algorithm"))


def packet(seq, w, h, dets, metrics):
    return json.dumps(dict(v=1, seq=seq, w=w, h=h, dets=[dict(class_name=d.get("class_name"), bbox=[float(x) for x in d.get("bbox", [])],
                                                              confidence=d.get("confidence")) for d in dets],
                           yolo_ms=metrics.get("yolo_ms"), total_ms=metrics.get("total_ms"))).encode()


class Snapper:
    """Optional (SNAP_EVERY_S > 0, default 0 = off): every N s save the current frame as JPEG q70 to <dir>/<epoch_ms>.jpg.
    Encoding + writing run in a worker thread; the frame loop only does a non-blocking handoff (one slot, a busy worker => frame skipped)."""
    def __init__(self, every_s, out_dir):
        self.every, self.dir, self.next_t = every_s, out_dir, 0.0
        self.q = queue.Queue(maxsize=1)
        os.makedirs(out_dir, exist_ok=True)
        threading.Thread(target=self._work, daemon=True).start()

    def offer(self, img):
        now = time.monotonic()
        if now < self.next_t:
            return
        self.next_t = now + self.every
        try:
            self.q.put_nowait((int(time.time() * 1000), img.copy()))
        except queue.Full:
            pass

    def _work(self):
        import cv2
        while True:
            ms, img = self.q.get()
            try:
                cv2.imwrite(os.path.join(self.dir, f"{ms}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            except Exception as e:
                print(f"[camera_node] snap error {type(e).__name__}: {e}", flush=True)


def run_realsense(sock, addr, retry_s, snap=None):
    sys.path.insert(0, VISION)
    seq = 0
    while True:
        cam = None
        try:
            from camera_detector import CameraDetector            # imports pyrealsense2 + ultralytics (heavy) -> only here
            cam = CameraDetector(); cam.start()
            print("[camera_node] camera + YOLO running", flush=True)
            while True:
                img, dets, met = cam.process_once()
                if img is None:
                    continue
                h, w = img.shape[:2]
                sock.sendto(packet(seq, w, h, dets, met), addr); seq += 1
                if snap is not None:
                    snap.offer(img)
        except KeyboardInterrupt:
            break
        except Exception as e:                                         # camera unplugged, YOLO error ... -> retry, never exit
            print(f"[camera_node] error {type(e).__name__}: {e}; retry in {retry_s:g} s", flush=True)
            try:
                if cam is not None:
                    cam.stop()
            except Exception:
                pass
            time.sleep(retry_s)


def run_video(sock, addr, path):
    sys.path.insert(0, VISION)
    import cv2
    from ultralytics import YOLO
    model = YOLO(os.path.expanduser("~/yolov8n_ncnn_model"), task="detect")
    cap = cv2.VideoCapture(path); seq = 0
    while cap.isOpened():
        ok, img = cap.read()
        if not ok:
            break
        t = time.perf_counter(); r = model(img, imgsz=640, conf=0.25, verbose=False)[0]; ms = (time.perf_counter() - t) * 1000
        dets = [dict(class_name=model.names[int(b.cls)], bbox=b.xyxy[0].tolist(), confidence=float(b.conf)) for b in r.boxes]
        h, w = img.shape[:2]
        sock.sendto(packet(seq, w, h, dets, dict(yolo_ms=ms, total_ms=ms)), addr); seq += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["realsense", "video"], default="realsense")
    ap.add_argument("--video"); ap.add_argument("--port", type=int, default=19100); ap.add_argument("--retry-s", type=float, default=3.0)
    ap.add_argument("--snap-dir", help="where SNAP_EVERY_S frames go (default $OUT/snap or ./snap)")
    a = ap.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); addr = ("127.0.0.1", a.port)
    if a.source == "video":
        run_video(sock, addr, a.video)
    else:
        try:
            every = float(os.environ.get("SNAP_EVERY_S", "0") or 0)
        except ValueError:
            every = 0.0
        snap = Snapper(every, a.snap_dir or os.path.join(os.environ.get("OUT", "."), "snap")) if every > 0 else None
        run_realsense(sock, addr, a.retry_s, snap)


if __name__ == "__main__":
    main()
