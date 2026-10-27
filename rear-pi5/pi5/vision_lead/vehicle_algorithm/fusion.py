"""Initial camera/radar projection and *candidate* association, not safety logic."""
from dataclasses import dataclass
import math
import struct
import time
import cv2

FX, FY, CX, CY = 607.1615, 607.34296, 321.98395, 240.55905
CAMERA_BEHIND_M, CAMERA_ABOVE_M = 1.5, 1.2

@dataclass(frozen=True)
class RadarObject:
    object_id: int
    forward_m: float
    left_m: float
    relative_mps: float
    measured_rx_monotonic: float
    stm32_ms: int

class RadarPacketDecoder:
    """Parse a continuous UART stream (02 id length payload XOR 03)."""
    def __init__(self):
        self.buffer = bytearray()
        self.valid_objects = 0
        self.invalid_frames = 0

    def feed(self, data, rx_monotonic=None):
        rx_monotonic = time.monotonic() if rx_monotonic is None else rx_monotonic
        self.buffer.extend(data)
        if len(self.buffer) > 65536:
            # Resynchronize if input is not the expected binary UART protocol.
            self.buffer = self.buffer[-4096:]
        found = []
        while len(self.buffer) >= 5:
            start = self.buffer.find(0x02)
            if start == -1:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            payload_len = self.buffer[2]
            if payload_len > 64:
                self.invalid_frames += 1
                del self.buffer[0]
                continue
            size = payload_len + 5
            if len(self.buffer) < size:
                break
            frame = self.buffer[:size]
            if frame[-1] != 0x03 or frame[-2] != (0 if not frame[1:-2] else _xor(frame[1:-2])):
                self.invalid_frames += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            if frame[1] != 0x12 or payload_len != 14:
                continue
            stamp, oid, fcm, lcm, vcm, _rcs, _dyn, _cycle = struct.unpack('<IBhhhBBB', frame[3:-2])
            found.append(RadarObject(oid, fcm / 100.0, lcm / 100.0, vcm / 100.0, rx_monotonic, stamp))
            self.valid_objects += 1
        return found

def _xor(data):
    value = 0
    for n in data:
        value ^= n
    return value

def project(obj, width, height, pitch_deg=0.0):
    """Assumes parallel forward optical axes, radar reflection height = radar origin."""
    forward = obj.forward_m + CAMERA_BEHIND_M
    if not (0.5 <= obj.forward_m <= 120.0 and abs(obj.left_m) <= 25.0 and forward > 0):
        return None
    u = CX - FX * obj.left_m / forward
    vertical_angle = math.atan2(CAMERA_ABOVE_M, forward)
    v = CY + FY * math.tan(vertical_angle + math.radians(pitch_deg))
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    if not (-30 <= u < width + 30 and -30 <= v < height + 30):
        return None
    return (u, v)

def associate_candidates(detections, radar_objects, shape, now_mono, max_age_s=0.15, pitch_deg=0.0):
    """Greedy one-to-one bounding-box gate; yields *candidate* associations only."""
    height, width = shape[:2]
    points = []
    for r in radar_objects:
        if now_mono - r.measured_rx_monotonic > max_age_s or now_mono < r.measured_rx_monotonic:
            continue
        pos = project(r, width, height, pitch_deg=pitch_deg)
        if pos is not None:
            points.append((r, pos))
    candidates = []
    for di, d in enumerate(detections):
        if str(d.get('class_name', '')).lower() not in ('car', 'truck', 'bus', 'motorcycle'):
            continue
        bbox = d.get('bbox', [])
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        x1, y1, x2, y2 = map(float, bbox)
        if not (x1 < x2 and y1 < y2):
            continue
        margin = 15
        for ri, (r, (u, v)) in enumerate(points):
            if x1-margin <= u <= x2+margin and y1-margin <= v <= y2+margin:
                # Prefer projected radar point near box horizontal center and lower half.
                target_v = y1 + (y2-y1) * .75
                cost = abs(u-(x1+x2)/2) / max(x2-x1, 1) + .4 * abs(v-target_v)/max(y2-y1, 1)
                candidates.append((cost, di, ri))
    candidates.sort()
    matches = []
    used_detection, used_radar = set(), set()
    for _cost, di, ri in candidates:
        if di in used_detection or ri in used_radar:
            continue
        used_detection.add(di)
        used_radar.add(ri)
        obj, (u, v) = points[ri]
        matches.append((di, obj, (u, v)))
    return points, matches

def draw_overlay(frame, points, matches, detections=None):
    """Show only one-to-one candidate matches; annotate corresponding YOLO boxes.

    These associations are provisional, NOT confirmed identities or safety measurements.
    `points` is intentionally unused: unmatched radar detections stay invisible.
    """
    height, width = frame.shape[:2]
    for di, radar, (u, v) in matches:
        x, y = round(u), round(v)
        if 0 <= x < width and 0 <= y < height:
            cv2.circle(frame, (x, y), 8, (255, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (255, 255, 0), -1)
        label = f'R{radar.object_id} {radar.forward_m:.1f}m {radar.relative_mps:+.1f}m/s CAND'
        if detections is not None and 0 <= di < len(detections):
            bbox = detections[di].get('bbox', [])
            if len(bbox) == 4:
                x1, y1, x2, y2 = map(int, bbox)
                label_x = max(0, min(x1 + 4, max(0, width - 200)))
                # Place inside top part of box, below original class label.
                label_y = max(18, min(y1 + 18, height - 6))
                cv2.putText(frame, label, (label_x, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1, cv2.LINE_AA)
                continue
        if 0 <= x < width and 0 <= y < height:
            cv2.putText(frame, label, (max(0, x - 70), max(18, y - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1, cv2.LINE_AA)
    return frame


# Candidate lead vehicle selection. NOT a validated lane/identity estimate.
def select_lead_vehicle(matches, lane_half_width_m=1.75, min_forward_m=1.0,
                        max_forward_m=100.0, max_abs_relative_mps=60.0):
    """Return (detection_index, RadarObject, projected_uv) or None.

    Geometric ego corridor only: assumes radar is centered and road is locally straight.
    A true lane assignment requires lane geometry, ego pose and calibrated extrinsics.
    """
    eligible = []
    for di, obj, uv in matches:
        if not (math.isfinite(obj.forward_m) and math.isfinite(obj.left_m)
                and math.isfinite(obj.relative_mps)):
            continue
        if not (min_forward_m <= obj.forward_m <= max_forward_m):
            continue
        if abs(obj.left_m) > lane_half_width_m:
            continue
        if abs(obj.relative_mps) > max_abs_relative_mps:
            continue
        eligible.append((di, obj, uv))
    return min(eligible, key=lambda item: item[1].forward_m) if eligible else None


def draw_lead_vehicle(frame, lead, detections=None):
    """Highlight the chosen lead candidate and show distance/relative speed."""
    h, w = frame.shape[:2]
    if lead is not None:
        di, obj, uv = lead
        text = f'LEAD CAND: {obj.forward_m:.1f}m  REL {obj.relative_mps:+.1f}m/s  ID{obj.object_id}'
        if detections is not None and 0 <= di < len(detections):
            bbox = detections[di].get('bbox', [])
            if len(bbox) == 4:
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(frame, (max(x1, 0), max(y1, 0)),
                              (min(x2, w-1), min(y2, h-1)), (0, 165, 255), 3)
    else:
        text = 'LEAD CAND: NONE'
    # Put it in an opaque banner; top overlay in replay occupies 0..31 pixels.
    banner_h = 32
    cv2.rectangle(frame, (0, 32), (w - 1, 32 + banner_h), (0, 0, 0), -1)
    font_scale = min(0.59, max(0.36, (w - 12) / max(1, len(text) * 12)))
    cv2.putText(frame, text, (7, 53), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 165, 255) if lead else (175, 175, 175), 2, cv2.LINE_AA)
    return frame
