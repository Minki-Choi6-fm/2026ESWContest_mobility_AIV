#!/usr/bin/env python3
"""Save annotated video frames at given drive times (the t_s axis of logs/s4_replay/<drive>.csv) for a visual check. Offline, read-only.
YOLO boxes (green, package settings) + radar points projected with the lead package's fusion.project (pitch 4 deg trial value):
red = radar objects closer than --near m, blue = others. Usage: ~/realsense-venv/bin/python -I frame_check.py <drive> T [T ...] --out DIR"""
import argparse, csv, os, sys, bisect
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vision_lead", "vehicle_algorithm")); sys.path.insert(0, HERE)
import fusion                                                   # noqa: E402
from s4_replay import load_rows                                 # noqa: E402
from uart_ingest import ExtendedDecoder, build_frame           # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("drive"); ap.add_argument("times", type=float, nargs="+")
ap.add_argument("--out", default="."); ap.add_argument("--near", type=float, default=8.0); ap.add_argument("--pitch-deg", type=float, default=4.0)
a = ap.parse_args()
import cv2
from ultralytics import YOLO
model = YOLO(os.path.expanduser("~/yolov8n_ncnn_model"), task="detect")
rows = load_rows(a.drive); t0 = rows[0][0]
dec = ExtendedDecoder(); radar = []
for tv, typ, pl in rows:
    if typ == 0x12:
        for fr in dec.feed(build_frame(typ, pl), tv):
            if fr.typ == 0x12:
                radar.append((tv, fr.radar))
rt = [x[0] for x in radar]
cam = list(csv.DictReader(open(os.path.join(a.drive, "camera_ts.csv"))))
ct = [float(r["time"]) for r in cam]
for T in a.times:
    th = t0 + T
    i = min(range(len(ct)), key=lambda k: abs(ct[k] - th)) if ct else None
    r = cam[bisect.bisect_left(ct, th) - 1 if bisect.bisect_left(ct, th) > 0 else 0]
    seg, idx = int(r["segment"]), int(r["idx_in_segment"])
    cap = cv2.VideoCapture(os.path.join(a.drive, f"camera_{seg:03d}.mp4")); cap.set(cv2.CAP_PROP_POS_FRAMES, idx); ok, img = cap.read(); cap.release()
    if not ok:
        print("cannot read frame for", T); continue
    res = model.predict(source=img, imgsz=640, conf=0.25, device="cpu", verbose=False)[0]
    for box in (res.boxes or []):
        cid = int(box.cls[0].item())
        if cid in (2, 3, 5, 7):
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist()); cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    hi = bisect.bisect_right(rt, float(r["time"]))
    latest = {}
    for tv, ro in radar[max(0, hi - 400):hi]:
        if 0 <= float(r["time"]) - tv <= 0.15:
            latest[ro.object_id] = ro
    for ro in latest.values():
        uv = fusion.project(ro, img.shape[1], img.shape[0], pitch_deg=a.pitch_deg)
        if uv:
            near = ro.forward_m <= a.near
            col = (0, 0, 255) if near else (255, 160, 0)
            cv2.circle(img, (int(uv[0]), int(uv[1])), 6 if near else 3, col, -1)
            if near or ro.forward_m < 40:
                cv2.putText(img, f"R{ro.object_id} {ro.forward_m:.1f}m lat{ro.left_m:+.1f} v{ro.relative_mps:+.1f}", (int(uv[0]) + 7, int(uv[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    cv2.putText(img, f"{os.path.basename(a.drive.rstrip('/'))} t={T:g}s  red=radar <= {a.near:g} m, blue=others, green=YOLO", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    out = os.path.join(a.out, f"frame_{os.path.basename(a.drive.rstrip('/'))[-6:]}_{T:g}.png"); cv2.imwrite(out, img); print(out)
