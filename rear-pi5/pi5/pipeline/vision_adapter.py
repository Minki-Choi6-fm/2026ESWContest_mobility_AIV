"""Adapter: this pipeline's radar objects <-> the vision code's fusion.RadarObject (fusion.py is NOT modified; it is imported from
pi5/vehicle_algorithm (10/09 version) or, with env VISION_FUSION_DIR, from the lead package in pi5/vision_lead/vehicle_algorithm).

fusion.RadarObject fields (10/09): object_id, forward_m, left_m, relative_mps, measured_rx_monotonic, stm32_ms.
Our Obj has long_m / lat_m / vrel_ms (+ rcs, dyn, which RadarObject lacks; they are carried separately). The conversion fills only the fields
the dataclass actually has, so a newer RadarObject with extra OPTIONAL fields still works; a new REQUIRED field raises a clear error.
"""
import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# fusion.py location: env VISION_FUSION_DIR (e.g. pi5/vision_lead/vehicle_algorithm = 10/16 lead package) or the 10/09 copy in pi5/vehicle_algorithm
sys.path.insert(0, os.environ.get("VISION_FUSION_DIR") or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vehicle_algorithm")))
from fusion import RadarObject                                      # noqa: E402  (existing vision code, read-only use)
from target_select import Obj                                       # noqa: E402
import constants as C                                               # noqa: E402

_MAP = {"object_id": "id", "forward_m": "long_m", "left_m": "lat_m", "relative_mps": "vrel_ms", "rcs_dbsm": "rcs", "dyn_prop": "dyn", "dyn": "dyn", "rcs": "rcs"}


def obj_to_radar_object(o: Obj, rx_monotonic: float, stm32_ms: int):
    """Our Obj -> fusion.RadarObject (positional meaning: left_m = lat_m as logged; its sign is unverified, see PROJECT_STATE)."""
    kw = {}
    for f in dataclasses.fields(RadarObject):
        if f.name == "measured_rx_monotonic":
            kw[f.name] = rx_monotonic
        elif f.name == "stm32_ms":
            kw[f.name] = stm32_ms
        elif f.name in _MAP:
            kw[f.name] = getattr(o, _MAP[f.name])
        elif f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING:
            raise TypeError(f"fusion.RadarObject has a new required field '{f.name}' that this adapter does not know")
    return RadarObject(**kw)


def radar_object_to_obj(r, rcs=None, dyn=None) -> Obj:
    return Obj(r.object_id, r.forward_m, r.left_m, r.relative_mps, rcs, dyn)


VEHICLE_CLASSES = ("car", "truck", "bus", "motorcycle")      # same set as fusion.associate_candidates
BOX_MARGIN_PX = 15                                          # same margin as fusion.associate_candidates
RADAR_MAX_AGE_S = 0.15                                      # same radar-age gate as fusion.associate_candidates


class CameraConfirm:
    """camera_confirmed for brain.evaluate(). Definition (S10): our target's radar point, projected into the image, is inside a YOLO vehicle box, one-to-one
    (several radar points in one box -> only the nearest counts), for C.CAMERA_CONFIRM_FRAMES consecutive frames. Any break -> False at once.
    Never opens a camera: it only receives already-made detections (list of dict(class_name, bbox)) and RadarObjects."""

    def __init__(self, frames=None, max_gap_s=None):
        self.n = C.CAMERA_CONFIRM_FRAMES if frames is None else frames
        self.max_gap = C.CAMERA_CONFIRM_MAX_FRAME_GAP_S if max_gap_s is None else max_gap_s
        self.streak = 0; self.tid = None; self.last_t = None

    def reset(self):
        self.streak = 0; self.tid = None

    def _project(self, r, w, h, pitch_deg):
        import inspect, fusion
        if pitch_deg is not None and "pitch_deg" in inspect.signature(fusion.project).parameters:
            return fusion.project(r, w, h, pitch_deg=pitch_deg)
        return fusion.project(r, w, h)

    def update(self, target_id, radar_objects, detections, shape, now_mono, pitch_deg=None) -> bool:
        """One camera frame. target_id = id of the radar target the pipeline follows (None = no target)."""
        if self.last_t is not None and now_mono - self.last_t > self.max_gap:
            self.reset()                                    # frames stopped for a while
        self.last_t = now_mono
        if target_id is None:
            self.reset(); return False
        h, w = shape[:2]
        pts = []
        for r in radar_objects:
            if now_mono < r.measured_rx_monotonic or now_mono - r.measured_rx_monotonic > RADAR_MAX_AGE_S:
                continue
            uv = self._project(r, w, h, pitch_deg)
            if uv is not None:
                pts.append((r, uv))
        ok = False
        for d in detections or []:
            if str(d.get("class_name", "")).lower() not in VEHICLE_CLASSES:
                continue
            bb = d.get("bbox", [])
            if not isinstance(bb, (list, tuple)) or len(bb) != 4:
                continue
            x1, y1, x2, y2 = map(float, bb)
            if not (x1 < x2 and y1 < y2):
                continue
            inside = [r for r, (u, v) in pts if x1 - BOX_MARGIN_PX <= u <= x2 + BOX_MARGIN_PX and y1 - BOX_MARGIN_PX <= v <= y2 + BOX_MARGIN_PX]
            if inside and min(inside, key=lambda r: r.forward_m).object_id == target_id:
                ok = True; break                            # the box belongs to its nearest radar point; that is our target
        if not ok:
            self.reset(); return False
        self.streak = self.streak + 1 if self.tid == target_id else 1
        self.tid = target_id
        return self.streak >= self.n

    def value(self, now_mono) -> bool:
        """Current value for brain.evaluate(camera_confirmed=...). False when no frame arrived recently (also when there is no camera at all)."""
        return self.last_t is not None and now_mono - self.last_t <= self.max_gap and self.tid is not None and self.streak >= self.n


def _selftest():
    o = Obj(17, 22.5, -1.2, -8.0, rcs=4.0, dyn=1)
    r = obj_to_radar_object(o, 12.5, 162131)
    assert (r.object_id, r.forward_m, r.left_m, r.relative_mps, r.measured_rx_monotonic, r.stm32_ms) == (17, 22.5, -1.2, -8.0, 12.5, 162131)
    back = radar_object_to_obj(r, rcs=4.0, dyn=1)
    assert back == o
    # the existing fusion code accepts the converted object (projection + candidate association with a fake detection)
    from fusion import associate_candidates
    det = [dict(class_name="car", bbox=[250, 200, 400, 300])]
    pts, matches = associate_candidates(det, [r], (480, 640, 3), 12.55)
    assert len(pts) == 1, "projection works"
    import fusion
    extra = ""
    if hasattr(fusion, "select_lead_vehicle"):
        pts2, m2 = fusion.associate_candidates(det, [r], (480, 640, 3), 12.55, pitch_deg=4.0)
        lead = fusion.select_lead_vehicle(m2)
        assert lead is not None and lead[1].object_id == 17, "lead selection on the converted object"
        far = obj_to_radar_object(Obj(18, 120.0, 0.0, -3.0), 12.5, 1)            # > 100 m: outside the lead rule
        wide = obj_to_radar_object(Obj(19, 20.0, 2.0, -3.0), 12.5, 1)            # |lat| 2.0 > 1.75
        fast = obj_to_radar_object(Obj(20, 20.0, 0.0, -61.0), 12.5, 1)           # |vrel| 61 > 60
        for o in (far, wide, fast):
            assert fusion.select_lead_vehicle([(0, o, (0, 0))]) is None
        extra = " + pitch_deg + select_lead_vehicle (1..100 m, |lat|<=1.75, |vrel|<=60) ok"
    print("vision_adapter selftest OK (projected points:", len(pts), "matches:", len(matches), ")" + extra, "| fusion from", os.path.dirname(os.path.abspath(fusion.__file__)))


if __name__ == "__main__":
    _selftest()
