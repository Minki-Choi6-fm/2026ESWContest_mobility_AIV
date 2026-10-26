"""Diagnostic-only 2D IoU tracks and explainable lead-candidate decisions.

Track IDs are tentative image associations, NOT persistent identities or safety outputs.
"""
import math


def iou(a, b):
    ax1, ay1, ax2, ay2 = map(float, a)
    bx1, by1, bx2, by2 = map(float, b)
    inter = max(0, min(ax2, bx2)-max(ax1, bx1))*max(0, min(ay2, by2)-max(ay1, by1))
    area_a = max(0, ax2-ax1)*max(0, ay2-ay1)
    area_b = max(0, bx2-bx1)*max(0, by2-by1)
    denom = area_a+area_b-inter
    return inter/denom if denom > 0 else 0.0


class IoUTracker:
    """Greedy class-consistent IoU tracking; expires after max_missing_s.

    Large motions, occlusions and camera ego motion may change track IDs.
    """
    def __init__(self, min_iou=0.2, max_missing_s=0.7):
        self.min_iou = min_iou
        self.max_missing_s = max_missing_s
        self.tracks = {}
        self.next_id = 1

    def update(self, detections, stamp):
        self.tracks = {tid: t for tid, t in self.tracks.items()
                       if 0 <= stamp - t['seen'] <= self.max_missing_s}
        options = []
        for di, det in enumerate(detections):
            for tid, t in self.tracks.items():
                if det.get('class_name') != t['class_name']:
                    continue
                score = iou(det['bbox'], t['bbox'])
                if score >= self.min_iou:
                    options.append((score, di, tid))
        used_d, used_t = set(), set()
        assigned = {}
        for _, di, tid in sorted(options, reverse=True):
            if di not in used_d and tid not in used_t:
                assigned[di] = tid
                used_d.add(di)
                used_t.add(tid)
        for di, det in enumerate(detections):
            tid = assigned.get(di)
            if tid is None:
                tid = self.next_id
                self.next_id += 1
            assigned[di] = tid
            self.tracks[tid] = {'bbox': list(det['bbox']),
                                'class_name': det.get('class_name'), 'seen': stamp}
        return assigned


def lead_decisions(detections, matches, lead, track_ids, lane_half_width_m=1.75,
                   min_forward_m=1.0, max_forward_m=100.0, max_abs_relative_mps=60.0):
    """One diagnostic row for EVERY YOLO vehicle (including unmatched detections)."""
    by_detection = {di: radar for di, radar, _ in matches}
    selected = lead[0] if lead is not None else None
    rows = []
    for di, det in enumerate(detections):
        radar = by_detection.get(di)
        if radar is None:
            reason = 'NO_RADAR_MATCH'
        elif not all(math.isfinite(v) for v in (radar.forward_m, radar.left_m, radar.relative_mps)):
            reason = 'NONFINITE_RADAR'
        elif radar.forward_m < min_forward_m:
            reason = 'TOO_NEAR'
        elif radar.forward_m > max_forward_m:
            reason = 'TOO_FAR'
        elif abs(radar.left_m) > lane_half_width_m:
            reason = 'OUTSIDE_CORRIDOR'
        elif abs(radar.relative_mps) > max_abs_relative_mps:
            reason = 'SPEED_OUTLIER'
        elif di == selected:
            reason = 'SELECTED'
        else:
            reason = 'ELIGIBLE_NOT_NEAREST'
        rows.append({'detection_index': di, 'track_id': track_ids.get(di),
                     'class_name': det.get('class_name', ''), 'confidence': det.get('confidence', ''),
                     'bbox': det.get('bbox', []), 'radar_id': radar.object_id if radar else '',
                     'forward_m': radar.forward_m if radar else '',
                     'left_m': radar.left_m if radar else '',
                     'relative_mps': radar.relative_mps if radar else '',
                     'reason': reason, 'selected': int(reason == 'SELECTED')})
    return rows


DECISION_FIELDS = ['time_s', 'frame_index', 'detection_index', 'track_id', 'class_name',
                   'confidence', 'bbox', 'radar_id', 'forward_m', 'left_m',
                   'relative_mps', 'reason', 'selected']

class LeadBoxHold:
    """Visual-only hold of the last selected lead for up to N processed frames.

    Held boxes and measurements are stale and MUST NOT be used for safety logic.
    """
    def __init__(self, frames=5):
        self.frames = max(0, int(frames))
        self.track_id = None
        self.bbox = None
        self.missing = 0

    def update(self, lead, detections, track_ids):
        """Return (bbox, held, missed_frames), or (None, False, 0).

        When the previous lead is still YOLO-detected but lacks radar association,
        follow that detection's latest bbox. Otherwise freeze last-known bbox.
        A newly selected lead takes precedence immediately.
        """
        if lead is not None:
            di = lead[0]
            self.track_id = track_ids.get(di)
            self.bbox = list(detections[di]['bbox'])
            self.missing = 0
            return self.bbox, False, 0
        if self.bbox is None or self.frames == 0:
            self.reset()
            return None, False, 0
        self.missing += 1
        if self.missing > self.frames:
            self.reset()
            return None, False, 0
        for di, tid in track_ids.items():
            if tid == self.track_id:
                self.bbox = list(detections[di]['bbox'])
                break
        return self.bbox, True, self.missing

    def reset(self):
        self.track_id = None
        self.bbox = None
        self.missing = 0


def draw_held_lead(frame, bbox, missing, frames):
    """Draw a stale lead box in dim orange with explicit HOLD status."""
    import cv2
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = (int(v) for v in bbox)
    cv2.rectangle(frame, (max(0,x1), max(0,y1)), (min(w-1,x2), min(h-1,y2)),
                  (0, 110, 175), 2)
    cv2.rectangle(frame, (0, 32), (w-1, 64), (0, 0, 0), -1)
    text = f'LEAD HOLD {missing}/{frames} frames - NO FRESH RADAR'
    cv2.putText(frame, text, (7, 53), cv2.FONT_HERSHEY_SIMPLEX, .51,
                (0, 170, 255), 1, cv2.LINE_AA)
    return frame
