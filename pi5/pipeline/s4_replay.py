#!/usr/bin/env python3
"""S4 log replay: target selection + K estimate + danger level over recorded drives. Reads logs only; sends nothing anywhere.

THIS IS A REPLAY OF RECORDED DATA, NOT A MEASUREMENT OF WARNING ACCURACY: there are no ground-truth danger labels, so no accuracy
numbers are produced. usage: python3 -I s4_replay.py <drive_dir> [...]   -> logs/s4_replay/<drive>.csv + summary on stdout

Receive times: frames.csv stamps whole read chunks. To get per-frame times the host chunk time is shifted back by
(STM32 ts of the last frame in the chunk - STM32 ts of this frame) [approximation].
"""
import bisect, collections, csv, json, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import constants as C
from brain import Brain
from live_receiver import ReplayGps
from speed_select import get_speed
from uart_ingest import ExtendedDecoder, SensorState, build_frame

WS = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(WS, "logs", "s4_replay")
EVAL_DT = 0.05


def load_rows(drive):
    """-> list of (t_virtual, typ, payload) with per-frame receive times reconstructed from STM32 timestamps."""
    chunks, cur_t, cur = [], None, []
    with open(os.path.join(drive, "frames.csv"), newline="") as f:
        r = csv.reader(f); next(r)
        for row in r:
            t = float(row[0]); typ = int(row[1], 16); pl = bytes.fromhex(row[2])
            if cur_t is None or t != cur_t:
                if cur:
                    chunks.append((cur_t, cur))
                cur_t, cur = t, []
            cur.append((typ, pl))
    if cur:
        chunks.append((cur_t, cur))
    out = []
    for t, frs in chunks:
        ts = [int.from_bytes(p[:4], "little") for typ, p in frs if typ in (0x10, 0x11, 0x12, 0x13)]
        last = max(ts) if ts else None
        prev = 0.0
        for typ, p in frs:
            tv = t
            if last is not None and typ in (0x10, 0x11, 0x12, 0x13):
                d = last - int.from_bytes(p[:4], "little")
                tv = t - d / 1000.0 if 0 <= d < 2000 else t
            tv = max(tv, prev) if out and out[-1][0] <= tv else tv
            out.append((tv, typ, p))
    out.sort(key=lambda x: x[0])
    return out


def segments(times, gap=0.5):
    segs = []
    for t in times:
        if segs and t - segs[-1][1] <= gap:
            segs[-1][1] = t; segs[-1][2] += 1
        else:
            segs.append([t, t, 1])
    return segs


VARIANTS = [   # name, constant overrides  (S4 targeting = hysteresis corridor 1.5/2.0 m, no 100 m limit, |vrel| <= 100)
    ("A new rule only", dict(TGT_USE_HYSTERESIS=True, TGT_MAX_LONG_M=1e9, TGT_VREL_VALID_MAX_MPS=100.0, SUPPRESS_STANDSTILL=False, SUPPRESS_STATIONARY_OBJECT=False)),
    ("B A + standstill cap", dict(TGT_USE_HYSTERESIS=True, TGT_MAX_LONG_M=1e9, TGT_VREL_VALID_MAX_MPS=100.0, SUPPRESS_STANDSTILL=True, SUPPRESS_STATIONARY_OBJECT=False)),
    ("C A + stationary-object cap", dict(TGT_USE_HYSTERESIS=True, TGT_MAX_LONG_M=1e9, TGT_VREL_VALID_MAX_MPS=100.0, SUPPRESS_STANDSTILL=False, SUPPRESS_STATIONARY_OBJECT=True)),
    ("D A + both caps", dict(TGT_USE_HYSTERESIS=True, TGT_MAX_LONG_M=1e9, TGT_VREL_VALID_MAX_MPS=100.0)),
    ("E S5 final (D + vision-aligned targeting; oncoming/passing + implausible-headway caps OFF)", dict(SUPPRESS_ONCOMING_PASSING=False, SUPPRESS_IMPLAUSIBLE_HEADWAY=False)),
    ("F S6 final (E + oncoming/passing cap c > v_r + 3 m/s)", dict(SUPPRESS_IMPLAUSIBLE_HEADWAY=False)),
    ("G S7 final (F + implausible-headway cap d/v < 0.3 s at >= 10 km/h)", dict()),
    ("H S8 final (= G + cap lifted by camera_confirmed / V2V; the logs have neither, so H must equal G)", dict()),
    ("H* HYPOTHETICAL upper bound: every target camera-confirmed (lifts the headway and the stationary-object caps)", dict(REPLAY_ASSUME_CAMERA_CONFIRMED=True)),
]


class override:
    def __init__(self, **kw):
        self.kw, self.old = kw, {}

    def __enter__(self):
        for k, v in self.kw.items():
            self.old[k] = getattr(C, k); setattr(C, k, v)

    def __exit__(self, *a):
        for k, v in self.old.items():
            setattr(C, k, v)


def replay(drive, write_csv=True):
    name = os.path.basename(drive.rstrip("/"))
    rows = load_rows(drive)
    dec, st, brain, gps = ExtendedDecoder(), SensorState(), Brain(), ReplayGps(drive)
    t0 = rows[0][0]
    os.makedirs(OUT, exist_ok=True)
    f = open(os.path.join(OUT, f"{name}.csv") if write_csv else os.devnull, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "level", "speed_kmh", "speed_src", "radar_alive", "target_id", "t_long", "t_lat", "t_vrel", "held", "nearest_id", "nearest_long",
                "K_r", "K_src", "d_req1.0", "a_req", "ttc", "level_before_suppression", "suppression", "side_count", "side_d", "side_lat_abs", "side_vrel", "cutin", "reason"])
    ev = []                                     # evaluation records (dict) for the summary
    nxt = rows[0][0] + 1.0
    for tv, typ, pl in rows:
        while nxt <= tv:
            gps.now = nxt
            sp = get_speed(st, gps, nxt)
            o = brain.evaluate(st, sp, None, True, nxt, camera_confirmed=C.REPLAY_ASSUME_CAMERA_CONFIRMED)
            ev.append(dict(t=nxt - t0, level=o.level, speed=sp.speed_kmh, src=sp.source, alive=st.radar_alive(nxt), tgt=o.target, held=o.held, near=o.nearest, near_lane=o.nearest_lane, pre=o.pre_level, supp=o.supp, side=o.side_count, cutin=o.cutin, reason=o.reason))
            tg = o.target
            w.writerow([f"{nxt - t0:.2f}", f"0x{o.level:02X}" if o.level == 0xFF else o.level, "" if sp.speed_kmh is None else f"{sp.speed_kmh:.0f}", sp.source,
                        int(st.radar_alive(nxt)), "" if tg is None else tg.id, "" if tg is None else f"{tg.long_m:.1f}", "" if tg is None else f"{tg.lat_m:.1f}",
                        "" if tg is None else f"{tg.vrel_ms:.1f}", int(o.held), "" if o.nearest is None else o.nearest.id, "" if o.nearest is None else f"{o.nearest.long_m:.1f}",
                        f"{o.k_r:.2f}", o.k_src, "" if o.d_req is None else f"{o.d_req:.1f}", "" if o.a_req is None else f"{o.a_req:.2f}",
                        "" if o.ttc is None else f"{o.ttc:.1f}", "" if o.pre_level is None else o.pre_level, ";".join(o.supp), o.side_count,
                        "" if o.side_nearest is None else f"{o.side_nearest.long_m:.1f}", "" if o.side_nearest is None else f"{abs(o.side_nearest.lat_m):.1f}",
                        "" if o.side_nearest is None else f"{o.side_nearest.vrel_ms:.1f}", int(o.cutin), o.reason])
            nxt += EVAL_DT
        for fr in dec.feed(build_frame(typ, pl), tv):
            st.update(fr)
            brain.on_frame(fr, tv)
    brain.kest.finish()
    f.close()
    return name, ev, brain


def summarize(name, ev, brain):
    n = len(ev)
    print(f"\n## {name}: {n} evaluations ({n * EVAL_DT:.0f} s), braking events accepted {len(brain.kest.events)} (rejected {len(brain.kest.rejected)}), bias {brain.kest.bias_mg if brain.kest.bias_mg is None else round(brain.kest.bias_mg)} mg, K source {brain.kest.current()[1]}")
    cnt = {0: 0, 1: 0, 2: 0, 0xFF: 0}
    for e in ev:
        cnt[e["level"]] += 1
    print("| level | share of time | seconds |\n|---|---|---|")
    for k, lab in ((0, "0 safe"), (1, "1 caution"), (2, "2 danger"), (0xFF, "0xFF unknown")):
        print(f"| {lab} | {100 * cnt[k] / n:.1f}% | {cnt[k] * EVAL_DT:.0f} |")
    both = [e for e in ev if e["tgt"] is not None and e["near"] is not None]
    diff = [e for e in both if e["tgt"].id != e["near"].id]
    print(f"\ntarget vs nearest object: both present {len(both)} evals; different id in {100 * len(diff) / max(1, len(both)):.1f}%; "
          f"target present {100 * sum(e['tgt'] is not None for e in ev) / n:.1f}% of time; "
          f"selector switches {brain.sel.switches}")
    bl = [e for e in ev if e["tgt"] is not None and e["near_lane"] is not None]
    dl = [e for e in bl if e["tgt"].id != e["near_lane"].id]
    print(f"target vs nearest object inside |lat|<=2.0 (no confirmation): both present {len(bl)} evals; different id in {100 * len(dl) / max(1, len(bl)):.1f}% (differences come from the 3-of-5 confirmation, hold and id stickiness)")
    d2 = [e for e in ev if e["level"] == 2 and e["tgt"] is not None and e["speed"] is not None and e["speed"] >= 10]
    st2 = [e for e in d2 if abs(e["tgt"].vrel_ms + e["speed"] / 3.6) <= 1.5]
    print(f"level-2 evals at speed >= 10 km/h: {len(d2)}; target looks STATIONARY over ground (|vrel + own speed| <= 1.5 m/s) in {100 * len(st2) / max(1, len(d2)):.1f}% of them")
    foll = [e for e in d2 if e["tgt"].vrel_ms > -0.5]
    ttcd = [e for e in d2 if e["tgt"].vrel_ms < -0.5 and e["tgt"].long_m / (-e["tgt"].vrel_ms) < 2.0]
    print(f"   of those: no closing speed (vrel > -0.5, distance rule d < d_req only) {100 * len(foll) / max(1, len(d2)):.1f}%; TTC < 2 s {100 * len(ttcd) / max(1, len(d2)):.1f}%; "
          f"median target distance {sorted(e['tgt'].long_m for e in d2)[len(d2) // 2] if d2 else '-'} m, median own speed {sorted(e['speed'] for e in d2)[len(d2) // 2] if d2 else '-'} km/h")
    for e in diff[:4]:
        print(f"   t={e['t']:.1f}s  target id{e['tgt'].id} long={e['tgt'].long_m:.1f} lat={e['tgt'].lat_m:.1f}  vs nearest id{e['near'].id} long={e['near'].long_m:.1f} lat={e['near'].lat_m:.1f}")
    # suspicious segments
    def tab(title, pred, limit=6):
        segs = segments([e["t"] for e in ev if pred(e)])
        tot = sum(s[2] for s in segs) * EVAL_DT
        print(f"\n{title}: {len(segs)} segment(s), {tot:.1f} s total")
        for s in segs[:limit]:
            e = next(x for x in ev if x["t"] == s[0])
            tg = e["tgt"]
            print(f"   t={s[0]:.1f}s dur={s[1] - s[0] + EVAL_DT:.1f}s speed={e['speed']} " + ("" if tg is None else f"target id{tg.id} long={tg.long_m:.1f} lat={tg.lat_m:.1f} vrel={tg.vrel_ms:.1f}"))
    tab("(a) level 2 while standing (speed < 2 km/h)", lambda e: e["level"] == 2 and e["speed"] is not None and e["speed"] < 2)
    tab("(c) level >= 1 at low speed (< 5 km/h; reverse cannot be told from OBD speed [hypothesis])", lambda e: e["level"] in (1, 2) and e["speed"] is not None and 0 < e["speed"] < 5)
    # implausible headway: how much time (any level) does a lead target sit closer than 0.3 s of travel?
    ih = [e for e in ev if e["tgt"] is not None and e["speed"] is not None and e["speed"] >= C.IMPLAUSIBLE_MIN_KMH and e["tgt"].long_m / (e["speed"] / 3.6) < C.IMPLAUSIBLE_HEADWAY_S]
    ih_pre2 = [e for e in ih if e["pre"] == 2]
    segs_ih = segments([e["t"] for e in ih])
    print(f"\nimplausible headway (d / v_ego < {C.IMPLAUSIBLE_HEADWAY_S:g} s at >= {C.IMPLAUSIBLE_MIN_KMH:g} km/h): {len(ih) * EVAL_DT:.1f} s in {len(segs_ih)} segment(s) with a lead target; "
          f"of which {len(ih_pre2) * EVAL_DT:.1f} s were raw level 2; target stationary over ground in {100 * sum(1 for e in ih if e['tgt'] is not None and abs(e['tgt'].vrel_ms + e['speed'] / 3.6) <= 1.5) / max(1, len(ih)):.0f} %")
    # (b) flicker: target id changes within 5 s
    ids = [(e["t"], e["tgt"].id if e["tgt"] is not None else None) for e in ev]
    ch = [ids[i][0] for i in range(1, len(ids)) if ids[i][1] != ids[i - 1][1]]
    wins = [t for i, t in enumerate(ch) if i + 2 < len(ch) and ch[i + 2] - t <= 5.0]
    print(f"\n(b) target flicker: {len(ch)} target changes (incl. appear/vanish); windows with >= 3 changes inside 5 s: {len(segments(wins, 5.0))}")
    for s in segments(wins, 5.0)[:5]:
        print(f"   around t={s[0]:.1f}s ({s[2]} change-starts)")
    # (d) radar not alive
    na = [e for e in ev if not e["alive"]]
    print(f"\n(d) radar_alive False: {len(na) * EVAL_DT:.1f} s; of these level 0xFF: {100 * sum(e['level'] == 0xFF for e in na) / max(1, len(na)):.1f}%, level 0: {100 * sum(e['level'] == 0 for e in na) / max(1, len(na)):.1f}%")
    alive_no_target = segments([e["t"] for e in ev if e["alive"] and e["tgt"] is None], 1.0)
    longest = sorted(alive_no_target, key=lambda s: s[1] - s[0], reverse=True)[:2]
    print("    longest stretches with radar_alive but no target: " + ", ".join(f"{s[0]:.0f}s..{s[1]:.0f}s" for s in longest))


def shares(ev):
    n = len(ev); c = {0: 0, 1: 0, 2: 0, 0xFF: 0}
    for e in ev:
        c[e["level"]] += 1
    return {k: 100.0 * v / n for k, v in c.items()}


def old_s4_shares(name):
    p = os.path.join(OUT, "old_S4", name + ".csv")
    if not os.path.exists(p):
        return None
    c = {"0": 0, "1": 0, "2": 0, "0xFF": 0}; n = 0
    for r in csv.DictReader(open(p)):
        c[r["level"]] += 1; n += 1
    return {0: 100.0 * c["0"] / n, 1: 100.0 * c["1"] / n, 2: 100.0 * c["2"] / n, 0xFF: 100.0 * c["0xFF"] / n}


def compare(drive):
    name = os.path.basename(drive.rstrip("/"))
    rows = []
    o4 = old_s4_shares(name)
    if o4:
        rows.append(("S4 (saved earlier: old rule + S4 targeting)", o4, None))
    for vn, kw in VARIANTS:
        with override(**kw):
            _, ev, brain = replay(drive, write_csv=False)
        sh = shares(ev)
        low = collections.Counter()
        for e in ev:
            if e["pre"] == 2 and e["level"] < 2:
                if not e["supp"]:
                    low["hysteresis delay (no suppression)"] += 1; continue
                t = e["supp"][0]
                low["standstill <2 km/h" if "정차" in t else "stationary object (ground speed)" if "대지" in t else "stationary object (dyn)" if "dyn" in t else "oncoming/passing (c > v_r+3)" if "마주" in t else "implausible-headway (d/v < 0.3 s)" if "implausible" in t else t] += 1
        rows.append((vn, sh, low))
    print(f"\n### {name}: share of time per level (replay of recorded data; NO ground truth, so this is a consistency check, not accuracy)")
    print("| variant | safe 0 | caution 1 | danger 2 | unknown 0xFF | danger seconds lowered to caution by suppression |\n|---|---|---|---|---|---|")
    for vn, sh, low in rows:
        lowtxt = "" if low is None else (", ".join(f"{k}: {v * EVAL_DT:.0f}s" for k, v in low.items()) or "-")
        print(f"| {vn} | {sh[0]:.1f}% | {sh[1]:.1f}% | {sh[2]:.1f}% | {sh[0xFF]:.1f}% | {lowtxt} |")


if __name__ == "__main__":
    import collections
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for d in args:
        t = time.time()
        summarize(*replay(d))
        print(f"   [{time.time() - t:.0f}s to compute]")
        if "--compare" in sys.argv:
            compare(d)
