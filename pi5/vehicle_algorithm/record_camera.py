import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera_detector import CameraDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-dir", default="~/vehicle_test_logs")
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()

    if args.fps <= 0:
        parser.error("--fps must be positive")

    # 저장 폴더 생성
    save_dir = Path(args.record_dir).expanduser()
    save_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    video_path = save_dir / f"vehicle_{timestamp}.mp4"
    csv_path = save_dir / f"detection_{timestamp}.csv"

    camera = CameraDetector()
    writer = None
    video_start = None
    written_frames = 0

    print("[RECORD] Starting RealSense camera...")
    print("[RECORD] Press Ctrl+C to stop.")

    try:
        with open(csv_path, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)

            csv_writer.writerow([
                "time_s",
                "vehicle_id",
                "class_name",
                "confidence",
                "bbox",
                "center_px",
                "lateral_ratio",
                "timestamp"
            ])

            camera.start()

            while True:
                # YOLO 차량 인식
                image, detections, metrics = camera.process_once()

                if image is None:
                    continue

                # Bounding Box 표시
                display_image = camera.draw(
                    image, detections, metrics
                )

                height, width = display_image.shape[:2]

                # 첫 프레임에서 MP4 파일 생성
                if writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

                    writer = cv2.VideoWriter(
                        str(video_path),
                        fourcc,
                        args.fps,
                        (width, height)
                    )

                    if not writer.isOpened():
                        raise RuntimeError(
                            "Failed to open MP4 writer"
                        )

                    video_start = time.monotonic()

                elapsed = time.monotonic() - video_start

                # 실제 경과 시간에 맞춰 프레임 기록
                # YOLO가 느리면 마지막 인식 프레임을 반복
                target_count = int(elapsed * args.fps) + 1

                while written_frames < target_count:
                    writer.write(display_image)
                    written_frames += 1

                # CSV 저장
                for index, det in enumerate(detections):
                    csv_writer.writerow([
                        round(elapsed, 3),
                        index,
                        det["class_name"],
                        det["confidence"],
                        json.dumps(det["bbox"]),
                        json.dumps(det["center_px"]),
                        det["lateral_ratio"],
                        det["timestamp"]
                    ])

                csv_file.flush()

                camera.print_latency_statistics(
                    metrics, detections
                )

                print(
                    f"[RECORD] {elapsed:.1f}s | "
                    f"Vehicles: {len(detections)} | "
                    f"Frames: {written_frames}"
                )

    except KeyboardInterrupt:
        print("\n[RECORD] Stopping...")

    finally:
        if writer is not None:
            writer.release()

        camera.stop()

        print("[RECORD] Recording finished.")
        print(f"[RECORD] Video: {video_path}")
        print(f"[RECORD] CSV: {csv_path}")


if __name__ == "__main__":
    main()
