#!/usr/bin/env python3
"""S10-d unit tests for vision_adapter.CameraConfirm. SYNTHETIC data only: no camera device is opened, YOLO is not run (detections are hand-made dicts)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import vision_adapter as V
from target_select import Obj

SHAPE = (480, 640, 3)
BOX = dict(class_name="car", bbox=[250, 200, 400, 300])           # contains the projection of Obj(17, 22.5 m, lat -1.2 m) (see vision_adapter selftest)


def ro(i, long_m, lat_m, t):
    return V.obj_to_radar_object(Obj(i, long_m, lat_m, -3.0), t, 1)


def frame(cc, tid, objs, dets, t):
    return cc.update(tid, [ro(i, l, la, t) for i, l, la in objs], dets, SHAPE, t)


def main():
    res = []
    def check(name, cond):
        res.append((name, bool(cond))); print(("PASS " if cond else "FAIL ") + name)
    assert C.CAMERA_CONFIRM_FRAMES == 3
    # 1. match: True only from the 3rd consecutive frame
    cc = V.CameraConfirm(); t = 10.0; seq = []
    for k in range(4):
        seq.append(frame(cc, 17, [(17, 22.5, -1.2)], [BOX], t + k * 0.033))
    check("match: False,False,True,True (N=3)", seq == [False, False, True, True])
    # 2. no match (box far away from the projected point)
    cc = V.CameraConfirm(); far = dict(class_name="car", bbox=[10, 10, 80, 60])
    check("no match: never True", not any(frame(cc, 17, [(17, 22.5, -1.2)], [far], 10 + k * .033) for k in range(6)))
    # 2b. non-vehicle class and bad bbox never confirm
    cc = V.CameraConfirm()
    bad = [dict(class_name="person", bbox=BOX["bbox"]), dict(class_name="car", bbox=[400, 200, 250, 300]), dict(class_name="car", bbox=[1, 2, 3])]
    check("person / inverted / malformed box: never True", not any(frame(cc, 17, [(17, 22.5, -1.2)], bad, 10 + k * .033) for k in range(6)))
    # 3. fewer than N frames
    cc = V.CameraConfirm(); r = [frame(cc, 17, [(17, 22.5, -1.2)], [BOX], 10 + k * .033) for k in range(2)]
    check("2 frames < N: False", r == [False, False] and cc.value(10.07) is False)
    # 4. several radar targets in one box: only the nearest one counts
    objs = [(17, 22.5, -1.2), (18, 24.0, -1.3)]                  # both project inside BOX (checked below); 17 is nearer
    cc17 = V.CameraConfirm(); cc18 = V.CameraConfirm()
    r17 = [frame(cc17, 17, objs, [BOX], 10 + k * .033) for k in range(4)]
    r18 = [frame(cc18, 18, objs, [BOX], 10 + k * .033) for k in range(4)]
    import fusion
    inb = lambda o: (lambda uv: uv is not None and 250 - 15 <= uv[0] <= 400 + 15 and 200 - 15 <= uv[1] <= 300 + 15)(fusion.project(o, 640, 480))
    assert inb(ro(17, 22.5, -1.2, 0)) and inb(ro(18, 24.0, -1.3, 0)), "test setup: both radar points must project inside the box"
    check("multi-target box: nearest (17) confirmed, farther (18) never", r17 == [False, False, True, True] and not any(r18))
    # 5. box breaks after 2 frames -> reset, needs N frames again
    cc = V.CameraConfirm(); t = 10.0
    a = [frame(cc, 17, [(17, 22.5, -1.2)], [BOX], t + k * .033) for k in range(2)]
    b = frame(cc, 17, [(17, 22.5, -1.2)], [], t + 2 * .033)
    c = [frame(cc, 17, [(17, 22.5, -1.2)], [BOX], t + (3 + k) * .033) for k in range(3)]
    check("box lost mid-way: reset; True again only after N new frames", a == [False, False] and b is False and c == [False, False, True])
    # 5b. after confirmation, a single missing box frame -> False at once
    cc = V.CameraConfirm()
    for k in range(4): frame(cc, 17, [(17, 22.5, -1.2)], [BOX], 10 + k * .033)
    x = frame(cc, 17, [(17, 22.5, -1.2)], [], 10.14)
    check("confirmed -> box missing for one frame: False immediately", x is False and cc.value(10.14) is False)
    # 6. target vanishes (id None / radar object not in list)
    cc = V.CameraConfirm()
    for k in range(4): frame(cc, 17, [(17, 22.5, -1.2)], [BOX], 10 + k * .033)
    y1 = frame(cc, None, [(17, 22.5, -1.2)], [BOX], 10.14)
    cc2 = V.CameraConfirm()
    for k in range(4): frame(cc2, 17, [(17, 22.5, -1.2)], [BOX], 10 + k * .033)
    y2 = frame(cc2, 17, [], [BOX], 10.14)
    check("target id None or radar object gone: False immediately", y1 is False and y2 is False)
    # 7. target id changes -> streak restarts at 1
    cc = V.CameraConfirm()
    for k in range(3): frame(cc, 17, [(17, 22.5, -1.2), (18, 21.0, -1.2)], [BOX], 10 + k * .033)
    z = frame(cc, 18, [(17, 22.5, -1.2), (18, 21.0, -1.2)], [BOX], 10.1)
    check("id change: not confirmed on the first frame of the new id", z is False)
    # 8. stale radar object (older than 0.15 s) is ignored
    cc = V.CameraConfirm()
    old = [V.obj_to_radar_object(Obj(17, 22.5, -1.2, -3.0), 9.0, 1)]
    check("stale radar object (1 s old) never confirms", not any(cc.update(17, old, [BOX], SHAPE, 10 + k * .033) for k in range(5)))
    # 9. frozen camera: no frame for > max gap -> value() False and streak dropped
    cc = V.CameraConfirm()
    for k in range(4): frame(cc, 17, [(17, 22.5, -1.2)], [BOX], 10 + k * .033)
    v_now = cc.value(10.2); v_late = cc.value(11.0)
    nxt = frame(cc, 17, [(17, 22.5, -1.2)], [BOX], 11.0)
    check("frozen camera: value() False after the gap; first frame afterwards not confirmed", v_now is True and v_late is False and nxt is False)
    # 10. no camera at all
    cc = V.CameraConfirm()
    check("no camera frame ever: value() False", cc.value(5.0) is False)
    # 11. wiring: brain.evaluate accepts the value (False by default = unchanged replay behaviour)
    import inspect, brain
    check("brain.evaluate(camera_confirmed=False) default", inspect.signature(brain.Brain.evaluate).parameters["camera_confirmed"].default is False)
    bad = [n for n, ok in res if not ok]
    print(f"\n{len(res) - len(bad)}/{len(res)} PASS" + ("" if not bad else "  FAIL: " + "; ".join(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
