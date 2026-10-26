"""Camera intrinsics exporter + radar-to-image projection helper.

This does NOT estimate a radar-to-camera transform. A validated transform,
coordinate convention and radar object positions must be supplied separately.
"""
import argparse
import json
from pathlib import Path


def export_realsense_color_intrinsics(output_path, width=640, height=480, fps=15):
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)
    try:
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = stream.get_intrinsics()
        result = {
            'width': intr.width, 'height': intr.height,
            'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy,
            'distortion_model': str(intr.model), 'distortion_coefficients': list(intr.coeffs),
            'note': 'RGB camera intrinsics ONLY. Extrinsic radar-to-camera transform NOT estimated.'
        }
        target = Path(output_path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'[CALIB] RGB intrinsics saved to {target}')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        pipeline.stop()


def project_radar_xyz_to_pixel(point_r, R_cr, t_cr, fx, fy, ppx, ppy):
    """Ideal pinhole projection; only for UNDISTORTED images/points.

    Coordinate conventions must be verified. No correspondence is inferred.
    R_cr: 3x3 matrix, t_cr: translation (m), point_r: radar (x,y,z) in meters.
    """
    import numpy as np
    p = np.asarray(R_cr, dtype=float).reshape(3, 3) @ np.asarray(point_r, dtype=float).reshape(3)
    p += np.asarray(t_cr, dtype=float).reshape(3)
    if not np.all(np.isfinite(p)) or p[2] <= 0:
        raise ValueError('Point lies behind camera or has invalid coordinates')
    return float(fx * p[0] / p[2] + ppx), float(fy * p[1] / p[2] + ppy)


def main():
    parser = argparse.ArgumentParser(description='Save RealSense RGB intrinsics; does not calibrate radar extrinsics')
    parser.add_argument('--intrinsics-out', default='~/vehicle_test_logs/color_intrinsics.json')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        parser.error('width, height and fps must be positive')
    export_realsense_color_intrinsics(args.intrinsics_out, args.width, args.height, args.fps)

if __name__ == '__main__':
    main()
