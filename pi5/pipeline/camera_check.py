#!/usr/bin/env python3
"""Read-only camera benchmark. Defaults unchanged; optional warmup and fixed-size NCNN models.
No serial/GPIO access. Models must already exist locally; never download weights.
"""
import argparse
import json
import os
from pathlib import Path
import statistics as S
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.environ.get('VISION_FUSION_DIR') or os.path.abspath(os.path.join(HERE, '..', 'vision_lead', 'vehicle_algorithm'))


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else None


def summary(values):
    return dict(p50=pct(values, .5), p90=pct(values, .9), max=max(values) if values else None)


def measure(cam, seconds, warmup_frames, clock=time.monotonic):
    """Warmup is excluded from a full measurement window; keep each first-frame sample."""
    warm = []
    warm_start = clock()
    while len(warm) < warmup_frames:
        if clock() - warm_start > max(30, warmup_frames * 5):
            raise RuntimeError('warmup timed out: not enough valid camera frames')
        img, _, met = cam.process_once()
        if img is not None:
            warm.append(dict(yolo_ms=float(met['yolo_ms']), total_ms=float(met['total_ms'])))
    start = clock()
    samples = []
    while clock() - start < seconds:
        img, dets, met = cam.process_once()
        if img is not None:
            samples.append(dict(t=clock() - start, yolo_ms=float(met['yolo_ms']),
                                total_ms=float(met['total_ms']), detections=len(dets)))
    elapsed = clock() - start
    yolo = [x['yolo_ms'] for x in samples]
    raw = [x['yolo_ms'] for x in warm] + yolo
    return dict(warmup_frames=len(warm), warmup_samples=warm, seconds_requested=seconds,
                elapsed_s=elapsed, frames=len(samples), fps=len(samples) / elapsed if elapsed else 0,
                yolo_ms=summary(yolo), total_ms=summary([x['total_ms'] for x in samples]),
                including_warmup_yolo_ms=summary(raw),
                max_yolo_frame_index=raw.index(max(raw)) + 1 if raw else None,
                detections_per_frame=S.mean(x['detections'] for x in samples) if samples else 0,
                samples=samples)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15)
    ap.add_argument('--warmup-frames', type=int, default=0)
    ap.add_argument('--imgsz', type=int, choices=[640, 416, 320], default=None)
    ap.add_argument('--model', help='existing local weights/model directory; no downloads')
    ap.add_argument('--json-out', help='save full timing samples and summary')
    a = ap.parse_args()
    if a.seconds <= 0 or a.warmup_frames < 0:
        ap.error('seconds must be positive; warmup-frames must be nonnegative')
    try:
        import pyrealsense2 as rs
        devs = rs.context().query_devices()
        print(f'RealSense devices: {len(devs)}')
        for d in devs:
            print(f'  - {d.get_info(rs.camera_info.name)} S/N {d.get_info(rs.camera_info.serial_number)}')
        if not len(devs):
            print('RESULT: FAIL - no device accessible in this environment'); return 2
    except Exception as e:
        print(f'RESULT: FAIL - pyrealsense2: {type(e).__name__}: {e}'); return 2
    sys.path.insert(0, VISION)
    cam = None
    try:
        import camera_detector as cd
        size = a.imgsz if a.imgsz is not None else cd.YOLO_IMAGE_SIZE
        model = Path(a.model).expanduser() if a.model else Path(cd.YOLO_MODEL_PATH).expanduser()
        if not a.model and size != cd.YOLO_IMAGE_SIZE and cd.USE_NCNN:
            model = Path(HERE).parents[1] / 'logs' / 'codex1' / 'camera_models' / str(size) / 'yolov8n_ncnn_model'
        if not model.exists():
            raise RuntimeError(f'local model missing: {model}; no weights downloaded')
        if model.is_dir():
            import yaml
            metadata = yaml.safe_load((model / 'metadata.yaml').read_text())
            exported = metadata.get('imgsz')
            if exported != [size, size]:
                raise RuntimeError(f'fixed NCNN model exported at {exported}, requested {size}; use matching model')
        # Process-local overrides only: camera_detector.py and default models stay unchanged.
        cd.YOLO_MODEL_PATH = str(model)
        cd.YOLO_IMAGE_SIZE = size
        cam = cd.CameraDetector(); cam.start()
        print(f'BENCH imgsz={size}, warmup={a.warmup_frames}, measured window={a.seconds:g}s', flush=True)
        result = measure(cam, a.seconds, a.warmup_frames)
        actual = list(cam.model.predictor.imgsz) if cam.model.predictor is not None else None
        result.update(imgsz_requested=size, imgsz_actual=actual, model=str(model),
                      affinity=sorted(os.sched_getaffinity(0)), detection_quality='unverified (desk environment)')
        if actual != [size, size]:
            raise RuntimeError(f'actual input size {actual} does not match requested {[size, size]}')
    except Exception as e:
        print(f'RESULT: FAIL - {type(e).__name__}: {e}'); return 2
    finally:
        if cam is not None:
            cam.stop()
    if a.json_out:
        p = Path(a.json_out).expanduser(); p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','warmup_samples')}, ensure_ascii=False, indent=2))
    print('warmup YOLO ms:', [round(x['yolo_ms'], 1) for x in result['warmup_samples']])
    ok = result['fps'] >= 15 and result['yolo_ms']['p90'] is not None and result['yolo_ms']['p90'] <= 100
    print('RESULT: ' + ('OK' if ok else 'WARN - below proposal FPS>=15 / YOLO p90<=100ms; detection quality unverified'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
