"""Pi5 diagnostic recording: YOLO annotated MP4 + detection CSV + raw STM32 UART.

NOT a calibrated sensor-fusion or active-safety controller. Open the Pi5<->STM32
UART only if its receive side isn't also required by another Pi program.
"""
import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera_detector import CameraDetector
from radar_receiver import RadarReceiver, DEFAULT_PORT
from fusion import associate_candidates, draw_overlay


def main():
    parser = argparse.ArgumentParser(description='Camera recording and separate RAW UART capture')
    parser.add_argument('--record', action='store_true', help='Record annotated YOLO video and CSV')
    parser.add_argument('--record-dir', default='~/vehicle_test_logs')
    parser.add_argument('--radar-port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--no-uart', action='store_true', help='Run camera only; do not open UART')
    parser.add_argument('--seconds', type=float, default=0, help='0 = until Ctrl+C')
    parser.add_argument('--fusion', action='store_true', help='Draw initial radar projection + candidate association; observational only')
    parser.add_argument('--video-fps', type=int, default=15)
    args = parser.parse_args()
    if args.fusion and args.no_uart:
        parser.error('--fusion requires UART')
    if not args.record:
        parser.error('This diagnostic program requires --record')
    if args.baud <= 0 or args.video_fps <= 0 or args.seconds < 0:
        parser.error('baud/video-fps must be positive, seconds nonnegative')

    output = Path(args.record_dir).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    tag = datetime.now().strftime('%Y%m%d_%H%M%S')
    movie_path = output / f'vehicle_{tag}.mp4'
    det_path = output / f'detection_{tag}.csv'
    raw_path = output / f'uart_{tag}.bin'
    uart_log_path = output / f'uart_{tag}.csv'
    match_path = output / f'fusion_candidates_{tag}.csv'
    receiver = None if args.no_uart else RadarReceiver(args.radar_port, args.baud, raw_path, uart_log_path)
    camera = CameraDetector()
    writer = None
    camera_started = False
    first_mono = None
    frames_written = 0
    try:
        if receiver is not None:
            receiver.start()
            print(f'[UART] Raw capture started: {args.radar_port} @ {args.baud}')
        with open(det_path, 'w', newline='') as f, open(match_path, 'w', newline='') as mf:
            log = csv.writer(f)
            match_log = csv.writer(mf)
            match_log.writerow(['camera_elapsed_s', 'camera_monotonic_s', 'detection_index', 'class_name', 'object_id', 'radar_forward_m', 'radar_left_m', 'radar_relative_mps', 'radar_rx_monotonic_s', 'projection_u_px', 'projection_v_px', 'status'])
            log.writerow(['camera_elapsed_s', 'detection_index', 'class_name', 'confidence',
                          'bbox', 'center_px', 'lateral_ratio', 'camera_timestamp'])
            camera.start()
            camera_started = True
            print('[MAIN] Recording; press Ctrl+C to stop. No graphical window.')
            while True:
                if receiver is not None:
                    receiver.check()
                if first_mono is not None and args.seconds and time.monotonic() - first_mono >= args.seconds:
                    break
                image, detections, metrics = camera.process_once()
                if image is None:
                    continue
                annotated = camera.draw(image, detections, metrics)
                camera_mono = time.monotonic()
                if args.fusion and receiver is not None:
                    points, matches = associate_candidates(detections, receiver.snapshot_objects(), annotated.shape, camera_mono)
                    draw_overlay(annotated, points, matches)
                if writer is None:
                    h, w = annotated.shape[:2]
                    writer = cv2.VideoWriter(str(movie_path), cv2.VideoWriter_fourcc(*'mp4v'),
                                             args.video_fps, (w, h))
                    if not writer.isOpened():
                        raise RuntimeError('VideoWriter did not open. Check MP4 support and disk space.')
                    first_mono = time.monotonic()
                elapsed = time.monotonic() - first_mono
                # Preserve approximate wall-clock duration by duplicating the most recent
                # annotated frame. These are NOT fresh 15-FPS YOLO inferences.
                target = int(elapsed * args.video_fps) + 1
                while frames_written < target:
                    writer.write(annotated)
                    frames_written += 1
                for index, d in enumerate(detections):
                    log.writerow([f'{elapsed:.3f}', index, d['class_name'], d['confidence'],
                                  json.dumps(d['bbox']), json.dumps(d['center_px']),
                                  d['lateral_ratio'], d['timestamp']])
                f.flush()
                if args.fusion:
                    for di, obj, pos in matches:
                        match_log.writerow([f'{elapsed:.3f}', f'{camera_mono:.6f}', di, detections[di].get('class_name', ''), obj.object_id, f'{obj.forward_m:.3f}', f'{obj.left_m:.3f}', f'{obj.relative_mps:.3f}', f'{obj.measured_rx_monotonic:.6f}', f'{pos[0]:.2f}', f'{pos[1]:.2f}', 'CANDIDATE_NOT_CONFIRMED'])
                    mf.flush()
                print(f'[MAIN] {elapsed:.1f}s; detections={len(detections)}; '
                      f'UART_bytes={receiver.received_bytes if receiver is not None else "disabled"}')
    except KeyboardInterrupt:
        print('\n[MAIN] Stop requested')
    finally:
        if writer is not None:
            writer.release()
        if camera_started:
            camera.stop()
        if receiver is not None:
            receiver.stop()
    print(f'[MAIN] MP4: {movie_path if writer is not None else "not created"}')
    print(f'[MAIN] Detection CSV: {det_path}')
    print(f'[MAIN] UART raw: {raw_path if receiver is not None else "disabled"}')
    print(f'[MAIN] Candidate CSV: {match_path if args.fusion else "disabled"}')
    print('[MAIN] Projection and candidate association are preliminary, not calibrated or suitable for safety decisions.')

if __name__ == '__main__':
    main()
