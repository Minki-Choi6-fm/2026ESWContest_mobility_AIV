#!/usr/bin/env python3

import os
import time
from collections import deque
from typing import List, Dict, Any, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


# ============================================================
# Configuration
# ============================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15

# v2: NCNN 모델(Pi5에서 PyTorch보다 약 3.8배 빠름, 오프라인 테스트 실측: 328ms -> 87ms @640)
#     NCNN 폴더가 없으면 예전처럼 yolov8n.pt(416)로 자동 대체한다.
NCNN_MODEL_DIR = os.path.expanduser("~/yolov8n_ncnn_model")
PT_MODEL_PATH = "yolov8n.pt"
USE_NCNN = os.path.isdir(NCNN_MODEL_DIR)
YOLO_MODEL_PATH = NCNN_MODEL_DIR if USE_NCNN else PT_MODEL_PATH
YOLO_IMAGE_SIZE = 640 if USE_NCNN else 416   # NCNN은 변환할 때 정한 640으로 고정
YOLO_CONFIDENCE_THRESHOLD = 0.25

# 차량이 전부 사라진 순간에도 이 프레임 수까지는 직전 결과를 유지해 끊김을 메운다.
# (한 프레임 놓침 때문에 앞차가 깜빡 사라지는 것을 줄이는 용도. 0이면 끈다)
HOLD_FRAMES = 2

# COCO vehicle classes
# 2 = car
# 3 = motorcycle
# 5 = bus
# 7 = truck
VEHICLE_CLASSES = {2, 3, 5, 7}

# Camera center corridor visualization
CORRIDOR_HALF_RATIO = 0.20

# Rolling statistics window
LATENCY_WINDOW_SIZE = 100

# Terminal print interval
TERMINAL_PRINT_INTERVAL_SEC = 1.0


# ============================================================
# Utility
# ============================================================

def summarize(values: deque) -> Dict[str, float]:
    """Return avg/min/max for a deque of float values."""
    if not values:
        return {
            "avg": 0.0,
            "min": 0.0,
            "max": 0.0,
        }

    arr = np.asarray(values, dtype=np.float64)

    return {
        "avg": float(np.mean(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


# ============================================================
# Camera Detector
# ============================================================

class CameraDetector:
    """
    RealSense RGB camera + YOLO vehicle detector.

    Depth information is NOT used.

    Detection output:
        - timestamp
        - class_id
        - class_name
        - confidence
        - bbox [x1, y1, x2, y2]
        - center_px [cx, cy]
        - lateral_pixel
        - lateral_ratio
        - inference_latency_ms

    lateral_ratio:
        -1.0 : left edge
         0.0 : image center
        +1.0 : right edge
    """

    def __init__(self):
        # ----------------------------------------------------
        # RealSense
        # ----------------------------------------------------
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            rs.format.bgr8,
            CAMERA_FPS
        )

        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------
        print("[CAMERA] Loading YOLO model...")
        self.model = YOLO(YOLO_MODEL_PATH, task="detect")
        print(f"[CAMERA] YOLO model loaded: {YOLO_MODEL_PATH} (imgsz={YOLO_IMAGE_SIZE})")

        # 끊김 보정(hold) 상태
        self._last_dets: List[Dict[str, Any]] = []
        self._miss_count = 0

        self.running = False

        # ----------------------------------------------------
        # Latency statistics
        # ----------------------------------------------------
        self.frame_wait_history = deque(maxlen=LATENCY_WINDOW_SIZE)
        self.yolo_history = deque(maxlen=LATENCY_WINDOW_SIZE)
        self.total_history = deque(maxlen=LATENCY_WINDOW_SIZE)
        self.fps_history = deque(maxlen=LATENCY_WINDOW_SIZE)

        self.last_terminal_print = 0.0

    # ========================================================
    # Start camera
    # ========================================================

    def start(self):
        if self.running:
            return

        self.pipeline.start(self.config)
        self.running = True

        print("[CAMERA] RealSense RGB stream started.")
        print(
            f"[CAMERA] Resolution: "
            f"{CAMERA_WIDTH}x{CAMERA_HEIGHT} "
            f"@ {CAMERA_FPS} FPS"
        )

    # ========================================================
    # Stop camera
    # ========================================================

    def stop(self):
        if self.running:
            self.pipeline.stop()
            self.running = False

        cv2.destroyAllWindows()
        print("[CAMERA] Camera stopped.")

    # ========================================================
    # Get RGB frame
    # ========================================================

    def get_frame(self) -> Tuple[Optional[np.ndarray], float]:
        """
        Returns:
            image
            frame_wait_latency_ms
        """

        if not self.running:
            raise RuntimeError(
                "Camera is not running. Call start() first."
            )

        t_wait_start = time.perf_counter()

        frames = self.pipeline.wait_for_frames()

        t_wait_end = time.perf_counter()

        frame_wait_latency_ms = (
            t_wait_end - t_wait_start
        ) * 1000.0

        color_frame = frames.get_color_frame()

        if not color_frame:
            return None, frame_wait_latency_ms

        image = np.asanyarray(
            color_frame.get_data()
        )

        return image, frame_wait_latency_ms

    # ========================================================
    # Detect vehicles
    # ========================================================

    def detect(
        self,
        image: np.ndarray
    ) -> Tuple[List[Dict[str, Any]], float]:

        if image is None:
            return [], 0.0

        height, width = image.shape[:2]
        image_center_x = width / 2.0

        # Sensor-fusion timestamp
        timestamp = time.monotonic()

        detections = []

        # ----------------------------------------------------
        # YOLO inference latency
        # ----------------------------------------------------
        t_yolo_start = time.perf_counter()

        results = self.model.predict(
            source=image,
            imgsz=YOLO_IMAGE_SIZE,
            conf=YOLO_CONFIDENCE_THRESHOLD,
            device="cpu",
            verbose=False
        )

        t_yolo_end = time.perf_counter()

        inference_latency_ms = (
            t_yolo_end - t_yolo_start
        ) * 1000.0

        # ----------------------------------------------------
        # Parse YOLO output
        # ----------------------------------------------------
        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                cls_id = int(
                    box.cls[0].item()
                )

                if cls_id not in VEHICLE_CLASSES:
                    continue

                confidence = float(
                    box.conf[0].item()
                )

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0].tolist()
                )

                # Clamp bbox
                x1 = max(0, min(x1, width - 1))
                x2 = max(0, min(x2, width - 1))
                y1 = max(0, min(y1, height - 1))
                y2 = max(0, min(y2, height - 1))

                if x2 <= x1 or y2 <= y1:
                    continue

                center_x = (
                    x1 + x2
                ) / 2.0

                center_y = (
                    y1 + y2
                ) / 2.0

                lateral_pixel = (
                    center_x
                    - image_center_x
                )

                lateral_ratio = (
                    lateral_pixel
                    / image_center_x
                )

                class_name = (
                    self.model.names.get(
                        cls_id,
                        str(cls_id)
                    )
                )

                detection = {
                    "timestamp": timestamp,
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2],
                    "center_px": [center_x, center_y],
                    "lateral_pixel": lateral_pixel,
                    "lateral_ratio": lateral_ratio,
                    "inference_latency_ms": inference_latency_ms,
                }

                detections.append(detection)

        return detections, inference_latency_ms

    # ========================================================
    # 끊김 보정: 전부 사라진 경우에만 직전 결과를 잠깐 유지
    # (일부 차만 사라진 경우는 보정하지 않는다. 유지된 항목은 held=True,
    #  timestamp는 원래 측정 시각 그대로라서 오래된 값임이 드러난다.)
    # ========================================================

    def _apply_hold(self, detections):
        if detections:
            self._last_dets = detections
            self._miss_count = 0
            return detections
        if HOLD_FRAMES > 0 and self._last_dets and self._miss_count < HOLD_FRAMES:
            self._miss_count += 1
            return [dict(d, held=True) for d in self._last_dets]
        self._last_dets = []
        return detections

    # ========================================================
    # One-frame processing
    # ========================================================

    def process_once(self):
        """
        Returns:
            image
            detections
            metrics
        """

        t_total_start = time.perf_counter()

        image, frame_wait_latency_ms = (
            self.get_frame()
        )

        if image is None:
            return None, [], {
                "frame_wait_ms": frame_wait_latency_ms,
                "yolo_ms": 0.0,
                "total_ms": 0.0,
                "fps": 0.0,
            }

        detections, yolo_latency_ms = (
            self.detect(image)
        )
        detections = self._apply_hold(detections)

        t_total_end = time.perf_counter()

        total_latency_ms = (
            t_total_end - t_total_start
        ) * 1000.0

        effective_fps = (
            1000.0 / total_latency_ms
            if total_latency_ms > 0.0
            else 0.0
        )

        # Rolling history
        self.frame_wait_history.append(
            frame_wait_latency_ms
        )

        self.yolo_history.append(
            yolo_latency_ms
        )

        self.total_history.append(
            total_latency_ms
        )

        self.fps_history.append(
            effective_fps
        )

        metrics = {
            "frame_wait_ms": frame_wait_latency_ms,
            "yolo_ms": yolo_latency_ms,
            "total_ms": total_latency_ms,
            "fps": effective_fps,
        }

        return image, detections, metrics

    # ========================================================
    # Draw
    # ========================================================

    def draw(
        self,
        image: np.ndarray,
        detections: List[Dict[str, Any]],
        metrics: Dict[str, float]
    ) -> np.ndarray:

        output = image.copy()

        height, width = output.shape[:2]
        image_center_x = width / 2.0

        # ----------------------------------------------------
        # Camera center line
        # ----------------------------------------------------
        cv2.line(
            output,
            (int(image_center_x), 0),
            (int(image_center_x), height),
            (255, 0, 0),
            2
        )

        # ----------------------------------------------------
        # Camera corridor visualization
        # ----------------------------------------------------
        corridor_half_width = int(
            width * CORRIDOR_HALF_RATIO
        )

        left_corridor_x = int(
            image_center_x
            - corridor_half_width
        )

        right_corridor_x = int(
            image_center_x
            + corridor_half_width
        )

        cv2.line(
            output,
            (left_corridor_x, 0),
            (left_corridor_x, height),
            (255, 255, 0),
            1
        )

        cv2.line(
            output,
            (right_corridor_x, 0),
            (right_corridor_x, height),
            (255, 255, 0),
            1
        )

        # ----------------------------------------------------
        # Draw vehicle detections
        # ----------------------------------------------------
        for det in detections:

            x1, y1, x2, y2 = det["bbox"]

            center_x, center_y = (
                det["center_px"]
            )

            confidence = (
                det["confidence"]
            )

            lateral_ratio = (
                det["lateral_ratio"]
            )

            class_name = (
                det["class_name"]
            )

            box_color = (0, 200, 255) if det.get("held") else (0, 255, 0)   # 유지된 박스는 주황색

            cv2.rectangle(
                output,
                (x1, y1),
                (x2, y2),
                box_color,
                2
            )

            cv2.circle(
                output,
                (
                    int(center_x),
                    int(center_y)
                ),
                5,
                (0, 0, 255),
                -1
            )

            label = (
                f"{class_name} "
                f"{confidence:.2f} "
                f"L:{lateral_ratio:+.2f}"
            )

            cv2.putText(
                output,
                label,
                (
                    x1,
                    max(y1 - 10, 20)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        # ----------------------------------------------------
        # Detection count
        # ----------------------------------------------------
        cv2.putText(
            output,
            f"vehicles: {len(detections)}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        # ----------------------------------------------------
        # Current latency overlay
        # ----------------------------------------------------
        cv2.putText(
            output,
            f"Frame wait: {metrics['frame_wait_ms']:.1f} ms",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1
        )

        cv2.putText(
            output,
            f"YOLO: {metrics['yolo_ms']:.1f} ms",
            (20, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1
        )

        cv2.putText(
            output,
            f"Total: {metrics['total_ms']:.1f} ms",
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1
        )

        cv2.putText(
            output,
            f"Effective FPS: {metrics['fps']:.2f}",
            (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1
        )

        return output

    # ========================================================
    # Terminal statistics
    # ========================================================

    def print_latency_statistics(
        self,
        metrics: Dict[str, float],
        detections: List[Dict[str, Any]]
    ):

        now = time.monotonic()

        if (
            now - self.last_terminal_print
            < TERMINAL_PRINT_INTERVAL_SEC
        ):
            return

        self.last_terminal_print = now

        frame_stats = summarize(
            self.frame_wait_history
        )

        yolo_stats = summarize(
            self.yolo_history
        )

        total_stats = summarize(
            self.total_history
        )

        fps_stats = summarize(
            self.fps_history
        )

        print("\n========================================")
        print("[CAMERA PERFORMANCE]")

        print(
            f"Current frame wait : "
            f"{metrics['frame_wait_ms']:.2f} ms"
        )

        print(
            f"Current YOLO       : "
            f"{metrics['yolo_ms']:.2f} ms"
        )

        print(
            f"Current total      : "
            f"{metrics['total_ms']:.2f} ms"
        )

        print(
            f"Current FPS        : "
            f"{metrics['fps']:.2f}"
        )

        print(
            f"Vehicles detected  : "
            f"{len(detections)}"
        )

        print("----------------------------------------")

        print(
            f"Frame wait avg/min/max : "
            f"{frame_stats['avg']:.2f} / "
            f"{frame_stats['min']:.2f} / "
            f"{frame_stats['max']:.2f} ms"
        )

        print(
            f"YOLO avg/min/max       : "
            f"{yolo_stats['avg']:.2f} / "
            f"{yolo_stats['min']:.2f} / "
            f"{yolo_stats['max']:.2f} ms"
        )

        print(
            f"Total avg/min/max      : "
            f"{total_stats['avg']:.2f} / "
            f"{total_stats['min']:.2f} / "
            f"{total_stats['max']:.2f} ms"
        )

        print(
            f"FPS avg/min/max        : "
            f"{fps_stats['avg']:.2f} / "
            f"{fps_stats['min']:.2f} / "
            f"{fps_stats['max']:.2f}"
        )


# ============================================================
# Detection output
# ============================================================

def print_detections(
    detections: List[Dict[str, Any]]
):

    if not detections:
        return

    for index, det in enumerate(
        detections
    ):

        print(
            f"[Vehicle {index}] "
            f"class={det['class_name']} "
            f"conf={det['confidence']:.3f} "
            f"bbox={det['bbox']} "
            f"center={det['center_px']} "
            f"lat_ratio={det['lateral_ratio']:+.3f} "
            f"ts={det['timestamp']:.6f}"
        )


# ============================================================
# Standalone test
# ============================================================

def main():

    camera = CameraDetector()

    camera.start()

    print(
        "[CAMERA] Press q or ESC to exit."
    )

    try:

        while True:

            image, detections, metrics = (
                camera.process_once()
            )

            if image is None:
                continue

            # Performance log once per second
            camera.print_latency_statistics(
                metrics,
                detections
            )

            # Vehicle information
            print_detections(
                detections
            )

            # Visualization
            display_image = camera.draw(
                image,
                detections,
                metrics
            )

            cv2.imshow(
                "RealSense Vehicle Detection",
                display_image
            )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if (
                key == ord("q")
                or key == 27
            ):
                break

    except KeyboardInterrupt:

        print(
            "\n[CAMERA] Keyboard interrupt."
        )

    finally:

        camera.stop()


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    main()
