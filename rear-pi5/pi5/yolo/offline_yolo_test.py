#!/usr/bin/env python3
"""녹화된 mp4로 YOLO 설정을 비교하는 오프라인 테스트 (Pi5에서 실행, 차/카메라 불필요).

사용법:
    python3 offline_yolo_test.py ~/vehicle_test_logs/vehicle_XXXX.mp4
    python3 offline_yolo_test.py 영상.mp4 --imgsz 416 640 960 --conf 0.25
    python3 offline_yolo_test.py 영상.mp4 --save-video out.mp4 --save-imgsz 640

주의:
 - 영상에 이미 박스가 그려져 있고 mp4 압축이 되어 있어서, 결과는 실시간보다 불리한 "하한선"이다.
 - 녹화 영상은 같은 프레임이 반복 저장돼 있을 수 있어서, 거의 같은 프레임은 건너뛴다.
 - 이 프로그램은 파일을 읽기만 하고 원본 영상은 바꾸지 않는다.
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

VEHICLE_CLASSES = {2, 3, 5, 7}   # car, motorcycle, bus, truck
SMALL_BOX_H = 40                 # 박스 높이가 이 값(px) 미만이면 "작은(먼) 물체"로 센다
SAME_FRAME_DIFF = 0.5            # 평균 픽셀 차이가 이 값 미만이면 같은 프레임으로 본다


def load_unique_frames(path, max_frames):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"영상을 열 수 없습니다: {path}")
    frames = []
    prev = None
    total = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        total += 1
        small = cv2.resize(frame, (80, 60)).astype(np.float32)
        if prev is not None and float(np.mean(np.abs(small - prev))) < SAME_FRAME_DIFF:
            continue
        prev = small
        frames.append(frame)
        if max_frames and len(frames) >= max_frames:
            break
    cap.release()
    return frames, total


def run_one(model, frames, imgsz, conf, save_path=None):
    counts, small_counts, times = [], [], []
    writer = None
    for frame in frames:
        t0 = time.perf_counter()
        res = model.predict(source=frame, imgsz=imgsz, conf=conf, device="cpu", verbose=False)
        times.append((time.perf_counter() - t0) * 1000.0)
        n = small = 0
        out = frame.copy() if save_path else None
        for r in res:
            if r.boxes is None:
                continue
            for b in r.boxes:
                if int(b.cls[0].item()) not in VEHICLE_CLASSES:
                    continue
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                n += 1
                if (y2 - y1) < SMALL_BOX_H:
                    small += 1
                if out is not None:
                    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(out, f"{float(b.conf[0].item()):.2f}", (x1, max(y1 - 6, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        counts.append(n)
        small_counts.append(small)
        if out is not None:
            if writer is None:
                h, w = out.shape[:2]
                writer = cv2.VideoWriter(str(save_path), cv2.VideoWriter_fourcc(*"mp4v"), 10, (w, h))
            writer.write(out)
    if writer is not None:
        writer.release()
    return counts, small_counts, times


def count_gaps(counts, max_gap=5):
    """차가 보이다가(>0) 짧게(<=max_gap 프레임) 0이 되었다가 다시 보이는 '끊김' 횟수."""
    gaps = 0
    i = 0
    n = len(counts)
    while i < n:
        if counts[i] == 0 and i > 0 and counts[i - 1] > 0:
            j = i
            while j < n and counts[j] == 0:
                j += 1
            if j < n and (j - i) <= max_gap:
                gaps += 1
            i = j
        else:
            i += 1
    return gaps


def apply_hold_counts(counts, hold):
    """camera_detector.py의 끊김 보정과 같은 규칙을 개수에만 적용해 본다(차가 전부 사라졌을 때 hold 프레임까지 유지)."""
    out = []
    last = 0
    miss = 0
    for c in counts:
        if c > 0:
            last, miss = c, 0
            out.append(c)
        elif hold > 0 and last > 0 and miss < hold:
            miss += 1
            out.append(last)
        else:
            last = 0
            out.append(0)
    return out


def main():
    ap = argparse.ArgumentParser(description="녹화 영상으로 YOLO imgsz/conf 비교 (읽기 전용)")
    ap.add_argument("video")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, nargs="+", default=[416, 640, 960])
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--max-frames", type=int, default=150, help="비교에 쓸 최대 프레임 수(시간이 오래 걸려서 제한)")
    ap.add_argument("--hold", type=int, default=2, help="끊김 보정으로 유지할 프레임 수(camera_detector.py의 HOLD_FRAMES와 같게)")
    ap.add_argument("--save-video", default=None, help="한 설정의 결과를 영상으로 저장")
    ap.add_argument("--save-imgsz", type=int, default=640)
    a = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(a.model)

    video = Path(a.video).expanduser()
    frames, total = load_unique_frames(video, a.max_frames)
    print(f"[INFO] 영상 전체 {total}프레임 중 서로 다른 프레임 {len(frames)}개 사용")
    if not frames:
        raise SystemExit("사용할 프레임이 없습니다.")

    print(f"[INFO] conf={a.conf}, 작은 물체 기준 = 박스 높이 {SMALL_BOX_H}px 미만")
    print()
    print(f"{'imgsz':>6} | {'평균ms':>7} | {'평균차량수':>9} | {'차량있는프레임%':>13} | {'작은물체합':>9} | {'끊김횟수':>7} | {'보정후끊김':>8}")
    print("-" * 84)
    for sz in a.imgsz:
        save = a.save_video if (a.save_video and sz == a.save_imgsz) else None
        counts, smalls, times = run_one(model, frames, sz, a.conf, save)
        avg_ms = sum(times) / len(times)
        avg_n = sum(counts) / len(counts)
        seen_pct = 100.0 * sum(1 for c in counts if c > 0) / len(counts)
        print(f"{sz:>6} | {avg_ms:>7.1f} | {avg_n:>9.2f} | {seen_pct:>12.1f}% | {sum(smalls):>9} | {count_gaps(counts):>7} | {count_gaps(apply_hold_counts(counts, a.hold)):>8}")
    print()
    print("[읽는 법] 평균ms: 한 프레임 처리 시간(작을수록 빠름). 작은물체합: 멀리 있는 차를 얼마나 더 잡는지.")
    print("          끊김횟수: 보이다가 잠깐 사라졌다 다시 나타난 횟수(적을수록 좋음).")
    print(f"          보정후끊김: 끊김 보정(--hold {a.hold}프레임)을 적용했다고 가정한 끊김 횟수.")
    if a.save_video:
        print(f"[INFO] 결과 영상 저장: {a.save_video}")


if __name__ == "__main__":
    main()
