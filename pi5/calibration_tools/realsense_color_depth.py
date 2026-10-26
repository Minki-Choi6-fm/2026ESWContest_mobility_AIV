#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 및 깊이 스트림 설정
    config.enable_stream(
        rs.stream.color,
        640,
        480,
        rs.format.bgr8,
        30
    )

    config.enable_stream(
        rs.stream.depth,
        640,
        480,
        rs.format.z16,
        30
    )

    # 카메라 스트리밍 시작
    pipeline_profile = pipeline.start(config)

    # 깊이 영상을 컬러 영상 좌표에 정렬
    align = rs.align(rs.stream.color)

    # 깊이 영상 색상 표시용
    colorizer = rs.colorizer()

    try:
        while True:
            frames = pipeline.wait_for_frames()

            # 깊이 프레임을 컬러 프레임에 맞춤
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # RealSense 프레임을 NumPy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())
            depth_color_image = np.asanyarray(
                colorizer.colorize(depth_frame).get_data()
            )

            # 컬러 영상 중심 좌표
            height, width = color_image.shape[:2]
            center_x = width // 2
            center_y = height // 2

            # 중심 위치의 거리 측정
            distance_m = depth_frame.get_distance(center_x, center_y)

            # 중심점 표시
            cv2.circle(
                color_image,
                (center_x, center_y),
                5,
                (0, 0, 255),
                -1
            )

            # 거리 표시
            cv2.putText(
                color_image,
                f"Distance: {distance_m:.2f} m",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # 컬러 영상과 깊이 영상 연결
            combined_image = np.hstack(
                (color_image, depth_color_image)
            )

            cv2.imshow(
                "RealSense Color | Depth",
                combined_image
            )

            # q 또는 ESC를 누르면 종료
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
