#!/usr/bin/env python3
"""Run three isolated, two-core live camera benchmarks and print a comparison."""
import datetime
import json
from pathlib import Path
import subprocess
import sys

WS = Path(__file__).resolve().parents[1]
out = WS / 'logs/codex1' / ('camera_compare_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True)
rows = []
for size in (640, 416, 320):
    report = out / f'{size}.json'
    cmd = ['taskset', '-c', '0,1', '/home/user/realsense-venv/bin/python',
           str(WS / 'pi5/pipeline/camera_check.py'), '--seconds', '15', '--warmup-frames', '5',
           '--imgsz', str(size), '--json-out', str(report)]
    print(f'\n=== imgsz {size}: warmup 5 + measurement 15s ===', flush=True)
    # Keep complete output locally; print only summaries after each run.
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        print('FAIL: camera benchmark timed out; remaining sizes not started'); sys.exit(2)
    (out / f'{size}.log').write_text(r.stdout + r.stderr)
    if r.returncode not in (0, 1) or not report.exists():
        print(r.stdout + r.stderr); print('FAIL; logs:', out); sys.exit(2)
    data = json.loads(report.read_text()); rows.append(data)
    print(f"FPS {data['fps']:.2f}; YOLO {data['yolo_ms']}; actual imgsz {data['imgsz_actual']}", flush=True)
print('\nsize | FPS | p50 ms | p90 ms | max ms | max incl warmup | max frame index')
for d in rows:
    y = d['yolo_ms']
    print(f"{d['imgsz_requested']} | {d['fps']:.2f} | {y['p50']:.1f} | {y['p90']:.1f} | {y['max']:.1f} | {d['including_warmup_yolo_ms']['max']:.1f} | {d['max_yolo_frame_index']}")
    print('  warmup YOLO ms:', [round(x['yolo_ms'], 1) for x in d['warmup_samples']])
print('Detection quality: [확인 필요] (desk); defaults unchanged. Logs:', out)
