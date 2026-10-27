"""S12-b: receiving side of the camera -> pipeline link (runs inside pipeline_main, cores 2-3).

The camera/YOLO process (camera_node.py, cores 0-1) sends one JSON datagram per processed frame to 127.0.0.1:<port>:
  {"v":1, "seq":N, "w":640, "h":480, "dets":[{"class_name":"car","bbox":[x1,y1,x2,y2],"confidence":0.8}, ...], "yolo_ms":F, "total_ms":F}
CameraLink turns that into camera_confirmed for brain.evaluate() using vision_adapter.CameraConfirm (N consecutive frames, nearest radar point per box).

!! camera_confirmed is only an AUXILIARY flag: it can LIFT two radar-side caps (stationary-object and implausible-headway) in danger.py, nothing else.
!! No camera, a dead camera, malformed packets or any exception here => camera_confirmed = False and the radar-only pipeline carries on unchanged.
!! This module never opens a camera and never imports the vision/YOLO stack except through vision_adapter (fusion.project only).
"""
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C


class CameraLink:
    def __init__(self, port, confirm=None, clock=time.monotonic):
        self.clock = clock
        self.sock = None
        self.port = port
        self.frames = self.bad = self.errors = 0
        self.last_rx = None
        self.last_info = {}
        self.confirm = confirm
        if port:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port)); s.setblocking(False)
            self.sock = s
        if self.confirm is None:
            try:
                import vision_adapter
                self.confirm = vision_adapter.CameraConfirm()
            except Exception as e:                               # fusion/vision code missing -> radar only
                self.confirm = None; self.errors += 1; self.import_error = repr(e)

    def pump(self, state, target_id, now):
        """Read every waiting datagram. state: SensorState (for fresh radar objects). Never raises."""
        if self.sock is None:
            return
        while True:
            try:
                d, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                self.errors += 1; return
            try:
                m = json.loads(d)
                w, h = int(m["w"]), int(m["h"])
                dets = [x for x in m.get("dets", []) if isinstance(x, dict)]
                self.frames += 1; self.last_rx = now
                self.last_info = dict(yolo_ms=m.get("yolo_ms"), total_ms=m.get("total_ms"), n_det=len(dets), seq=m.get("seq"))
                if self.confirm is not None:
                    self.confirm.update(target_id, state.fresh_radar_objects(now), dets, (h, w, 3), now)
            except Exception:
                self.bad += 1

    def alive(self, now):
        return self.last_rx is not None and now - self.last_rx <= C.CAMERA_CONFIRM_MAX_FRAME_GAP_S

    def confirmed(self, now):
        try:
            return bool(self.confirm is not None and self.confirm.value(now))
        except Exception:
            self.errors += 1
            return False

    def close(self):
        if self.sock is not None:
            self.sock.close(); self.sock = None
