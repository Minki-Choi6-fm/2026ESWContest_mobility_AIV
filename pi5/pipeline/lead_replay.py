#!/usr/bin/env python3
"""S6-a: replay N seconds of a recorded drive (video + radar frames) and compare
   (A) the vision package's lead choice  : YOLO x radar one-to-one match -> fusion.select_lead_vehicle (nearest in |lat|<=1.75)
   (B) our radar-only target selection   : target_select.TargetSelector (3-of-5 confirmation, hold, id stickiness) fed with the same radar cycles
and measure camera_confirmed = "our target's radar id is among the YOLO-matched radar ids of that frame".
Offline, read-only. Uses the lead package's fusion.py / tracking_diagnostics.py (pi5/vision_lead, unmodified) and the NCNN YOLO model with the
package's own settings (conf 0.25, COCO car/motorcycle/bus/truck, imgsz 640, pitch 4 deg trial value).
Run:  ~/realsense-venv/bin/python -I lead_replay.py <drive_dir> [--start 5 --dur 10 --skip 2 --pitch-deg 4]
Not a measurement of lead-selection accuracy (no ground truth).
"""
import argparse, collections, csv, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
LEAD_DIR = os.path.abspath(os.path.join(HERE, "..", "vision_lead", "vehicle_algorithm"))
sys.path.insert(0, LEAD_DIR)                       # package fusion.py first (cached in sys.modules for uart_ingest too)
sys.path.insert(0, HERE)
import fusion                                       # noqa: E402  (package version, read-only)
from tracking_diagnostics import lead_decisions     # noqa: E402
import constants as C                               # noqa: E402
from s4_replay import load_rows                     # noqa: E402
from target_select import Obj, TargetSelector       # noqa: E402
from uart_ingest import ExtendedDecoder, build_frame  # noqa: E402

YOLO_CONF, VEHICLE_CLASSES = 0.25, {2, 3, 5, 7}    # same values as camera_detector.py in the package


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("drive"); ap.add_argument("--start", type=float, default=5.0); ap.add_argument("--dur", type=float, default=10.0)
    ap.add_argument("--skip", type=int, default=2); ap.add_argument("--pitch-deg", type=float, default=4.0); ap.add_argument("--segment", type=int, default=0)
    ap.add_argument("--save-frame", type=int, default=0, help="save the N-th processed frame with YOLO boxes + radar projections as PNG (scratch check)")
    ap.add_argument("--save-dir", default=".")
    a = ap.parse_args()
    import cv2
    from ultralytics import YOLO
    model = YOLO(os.path.expanduser("~/yolov8n_ncnn_model"), task="detect")
    # camera timestamps
    cam = [r for r in csv.DictReader(open(os.path.join(a.drive, "camera_ts.csv"))) if int(r["segment"]) == a.segment]
    t_first = float(cam[0]["time"])
    by_idx = {int(r["idx_in_segment"]): float(r["time"]) for r in cam}
    # radar objects with reconstructed per-frame receive times (same clock as camera_ts: host epoch)
    dec = ExtendedDecoder()
    radar = []                                      # (rx, RadarObject, rcs, dyn)
    for tv, typ, pl in load_rows(a.drive):
        if typ == 0x12:
            for fr in dec.feed(build_frame(typ, pl), tv):
                if fr.typ == 0x12:
                    b = pl[11]; radar.append((tv, fr.radar, (b - 256 if b > 127 else b) / 2.0, pl[12]))
    radar_t = [x[0] for x in radar]
    import bisect
    cap = cv2.VideoCapture(os.path.join(a.drive, f"camera_{a.segment:03d}.mp4"))
    fps_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    sel = TargetSelector()
    fed_upto = 0                                    # index into radar list already fed to the selector (by cycles)
    rows, reasons, ex = [], collections.Counter(), []
    n_proc = 0
    t_run = time.time()
    idx = -1
    while True:
        ok, img = cap.read()
        idx += 1
        if not ok:
            break
        t = by_idx.get(idx)
        if t is None or t < t_first + a.start:
            continue
        if t > t_first + a.start + a.dur:
            break
        if (idx % a.skip) != 0:
            continue
        n_proc += 1
        res = model.predict(source=img, imgsz=640, conf=YOLO_CONF, device="cpu", verbose=False)[0]
        dets = []
        for box in (res.boxes or []):
            cid = int(box.cls[0].item())
            if cid in VEHICLE_CLASSES:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                dets.append(dict(class_id=cid, class_name=model.names.get(cid, str(cid)), confidence=float(box.conf[0].item()), bbox=[x1, y1, x2, y2]))
        # package: latest radar object per id within the last 0.15 s
        hi = bisect.bisect_right(radar_t, t)
        latest = {}
        for rx, ro, rcs, dyn in radar[max(0, hi - 400):hi]:
            if 0 <= t - rx <= 0.15:
                latest[ro.object_id] = ro
        pts, matches = fusion.associate_candidates(dets, list(latest.values()), img.shape, t, pitch_deg=a.pitch_deg)
        lead = fusion.select_lead_vehicle(matches)
        # ours: feed all radar cycles up to t
        cyc, cyc_ts, cyc_rx = [], None, 0
        while fed_upto < hi:
            rx, ro, rcs, dyn = radar[fed_upto]
            if cyc_ts is not None and ro.stm32_ms != cyc_ts:
                sel.update(cyc, cyc_rx); cyc = []
            cyc_ts, cyc_rx = ro.stm32_ms, rx
            cyc.append(Obj(ro.object_id, ro.forward_m, ro.left_m, ro.relative_mps, rcs, dyn))
            fed_upto += 1
        if cyc:
            sel.update(cyc, cyc_rx)
        tr = sel.current(t)
        T = tr.target
        match_ids = {m[1].object_id for m in matches}
        row = dict(t=round(t - t_first, 2), n_det=len(dets), n_match=len(matches), lead=None if lead is None else (lead[1].object_id, lead[1].forward_m),
                   ours=None if T is None else (T.id, T.long_m), held=tr.held, confirmed=(T is not None and T.id in match_ids))
        rows.append(row)
        if a.save_frame and n_proc == a.save_frame:
            vis = img.copy()
            for d in dets:
                x1, y1, x2, y2 = d["bbox"]; cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"{d['class_name']} {d['confidence']:.2f}", (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            for ro in latest.values():
                uv = fusion.project(ro, img.shape[1], img.shape[0], pitch_deg=a.pitch_deg)
                if uv:
                    col = (0, 0, 255) if abs(ro.left_m) <= 1.75 else (255, 160, 0)
                    cv2.circle(vis, (int(uv[0]), int(uv[1])), 5, col, -1)
                    cv2.putText(vis, f"R{ro.object_id} {ro.forward_m:.0f}m lat{ro.left_m:+.1f} v{ro.relative_mps:+.0f}", (int(uv[0]) + 6, int(uv[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
            cv2.putText(vis, f"t={row['t']}s  red = radar |lat|<=1.75, blue = other; green = YOLO", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imwrite(os.path.join(a.save_dir, f"lead_replay_seg{a.segment}_{a.save_frame}.png"), vis)
        if T is None and lead is None:
            cat = "both none"
        elif T is not None and lead is None:
            if tr.held: cat = "only ours: held radar target (hold 0.3 s)"
            elif not dets: cat = "only ours: YOLO found no vehicle"
            elif T.id not in match_ids: cat = "only ours: our target has no YOLO match (radar-only object)"
            else: cat = "only ours: matched but filtered by the lead rule"
        elif T is None and lead is not None:
            cat = "only lead: camera-matched object, not yet confirmed 3-of-5 / lost by radar selector"
        elif T.id == lead[1].object_id:
            cat = "same object"
        else:
            cat = ("different: ours nearer, no YOLO match" if T.long_m < lead[1].forward_m else "different: lead nearer / stickiness margin")
        reasons[cat] += 1
        if cat not in ("same object", "both none") and len(ex) < 8:
            ex.append((row["t"], cat, row["lead"], row["ours"], len(dets), len(matches)))
    dt = time.time() - t_run
    print(f"\n## {os.path.basename(a.drive.rstrip('/'))} video segment {a.segment}, window {a.start:g}..{a.start + a.dur:g} s, every {a.skip}nd frame: {n_proc} frames processed in {dt:.0f} s (video has {fps_total} frames)")
    print("YOLO settings of the package: NCNN 640, conf 0.25, car/motorcycle/bus/truck; pitch", a.pitch_deg, "deg (trial value, not calibrated)")
    both = sum(1 for r in rows if r["lead"] and r["ours"]); ours_n = sum(1 for r in rows if r["ours"]); lead_n = sum(1 for r in rows if r["lead"])
    conf = sum(1 for r in rows if r["ours"] and r["confirmed"])
    print(f"frames with a vision lead: {lead_n}; with our radar-only target: {ours_n}; both: {both}")
    print(f"camera_confirmed (our target's radar id is YOLO-matched in that frame): {conf}/{ours_n} = {100 * conf / max(1, ours_n):.0f} %")
    print("| category | frames | share |\n|---|---|---|")
    for k, v in reasons.most_common():
        print(f"| {k} | {v} | {100 * v / max(1, len(rows)):.0f} % |")
    print("examples (t, category, vision lead (radar id, m), ours (id, m), n_yolo, n_match):")
    for e in ex:
        print("  ", e)


if __name__ == "__main__":
    main()
