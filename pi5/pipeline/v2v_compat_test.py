#!/usr/bin/env python3
"""S4 1-3: compatibility of the modified V2V code (freshness.py / bsm_protocol.py) with the ORIGINAL (.bak) versions.
Loads the .bak copies by path (read-only) and checks: old-style calls give byte-identical frames, new frames still pass the
ORIGINAL decoder (CRC, bits 0-3 equal), the new decoder reads bits 4-6, and FreshnessTracker keeps its old behaviour except
'accept when STALE / first'."""
import glob, importlib.util, os, sys, time

V = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "v2v", "v2v_stack"))   # folder holding bsm_protocol.py / freshness.py and their .bak_* files
sys.path.insert(0, V)
import bsm_protocol as new_bp
import freshness as new_fr


def load(path, name):
    import importlib.machinery
    loader = importlib.machinery.SourceFileLoader(name, path)       # .bak_* has no .py suffix -> explicit loader
    spec = importlib.util.spec_from_loader(name, loader)
    m = importlib.util.module_from_spec(spec); loader.exec_module(m)
    return m


old_bp = load(sorted(glob.glob(os.path.join(V, "bsm_protocol.py.bak_*")))[0], "old_bp")
old_fr = load(sorted(glob.glob(os.path.join(V, "freshness.py.bak_*")))[0], "old_fr")
ok = True


def check(name, cond):
    global ok
    ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name)


kw = dict(vehicle_id=1, seq=42, lat=37.55, lon=127.07, speed_kmh=63.5, heading_deg=182.3, k_value=0.72, gps_valid=True,
          danger_active=True, sensor_degraded=True, safe_mode=True, timestamp_ms=123456)
a, b = old_bp.encode_bsm(**kw), new_bp.encode_bsm(**kw)
check("old-style call -> byte-identical 26-byte frame", a == b and len(b) == 26)
f = new_bp.encode_bsm(**kw, brake_active=True, speed_src_gps=True, speed_stale=True)
check("new frame is 26 bytes", len(f) == 26)
d_old = old_bp.decode_bsm(f)                       # ORIGINAL decoder on a frame with bits 4-6 set
check("ORIGINAL decoder accepts new frame (CRC ok)", d_old["seq"] == 42)
check("bits 0-3 identical in ORIGINAL decoder", all(d_old[k] == (kw[k2]) for k, k2 in
      (("gps_valid", "gps_valid"), ("danger_active", "danger_active"), ("sensor_degraded", "sensor_degraded"), ("safe_mode", "safe_mode"))))
d_new = new_bp.decode_bsm(f)
check("new decoder reads bits 4-6", d_new["brake_active"] and d_new["speed_src_gps"] and d_new["speed_stale"] and d_new["status_raw"] == 0x7F)
d_new0 = new_bp.decode_bsm(a)
check("new decoder on old-style frame: bits 4-6 = 0", not (d_new0["brake_active"] or d_new0["speed_src_gps"] or d_new0["speed_stale"]))
bad = bytearray(f); bad[10] ^= 1
try:
    new_bp.decode_bsm(bytes(bad)); check("CRC error still detected", False)
except new_bp.BSMDecodeError:
    check("CRC error still detected", True)

# FreshnessTracker behaviour: identical for non-stale frames, different (recovering) after STALE
def mk(seq, vid=1):
    return {"vehicle_id": vid, "seq": seq}
for label, mod in (("old", old_fr), ("new", new_fr)):
    t = mod.FreshnessTracker(stale_timeout_ms=80)
    r = [t.update(mk(10)), t.update(mk(11)), t.update(mk(11)), t.update(mk(5)), t.update(mk(12))]       # ok, ok, dup, backwards, ok
    check(f"{label}: ok/ok/dup/backwards/ok = {r}", r == [True, True, False, False, True])
    t2 = mod.FreshnessTracker(stale_timeout_ms=80)
    t2.update(mk(65530)); r2 = t2.update(mk(3))                                                     # uint16 wrap, not stale
    check(f"{label}: wraparound 65530->3 accepted ({r2})", r2 is True)
    t3 = mod.FreshnessTracker(stale_timeout_ms=80)
    t3.update(mk(300)); time.sleep(0.12); r3 = t3.update(mk(0))                                     # sender restarted while STALE
    check(f"{label}: restart SEQ 300->0 after STALE -> {'accepted' if r3 else 'REJECTED'}", r3 is (label == "new"))
print("\nALL PASS" if ok else "\nSOME FAILED")
sys.exit(0 if ok else 1)
