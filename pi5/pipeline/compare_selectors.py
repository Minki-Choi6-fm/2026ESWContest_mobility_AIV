#!/usr/bin/env python3
"""READ-ONLY offline comparison: old TargetSelector vs WindowTargetSelector, both connected through brain.Brain -> danger.step (s4_replay.replay).
Prints the level distribution (0/1/2/0xFF) of both and every interval where the NEW selector gives level 2 and the OLD one does not (time, target, reason).
Optional FAKE_KMH=NN replaces the recorded own speed by a constant (TEST-ONLY stress, e.g. for desk logs that have no OBD speed).
usage: [FAKE_KMH=30] python3 -I compare_selectors.py LOGDIR [LOGDIR ...]   (LOGDIR contains rec/frames.csv)"""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import s4_replay as S
import brain as B
from target_select_win import WindowTargetSelector
from speed_select import SpeedResult


class BrainWin(B.Brain):
    def __init__(self, *a, **k):
        super().__init__(*a, **k); self.sel = WindowTargetSelector()


def run(drive, cls):
    S.Brain = cls
    _, ev, br = S.replay(os.path.join(drive, "rec"), write_csv=False)
    return ev


def shares(ev):
    c = {0: 0, 1: 0, 2: 0, 255: 0}
    for e in ev: c[e["level"]] += 1
    return c


def main():
    fake = os.environ.get("FAKE_KMH")
    if fake:
        S.get_speed = lambda st, gps, now: SpeedResult(float(fake), 0, 0, "fake")
    for d in sys.argv[1:]:
        t_start = None
        old = run(d, B.Brain); new = run(d, BrainWin)
        n = len(old); name = os.path.basename(d.rstrip("/")) + (f" [FAKE {fake} km/h]" if fake else "")
        so, sn = shares(old), shares(new)
        print(f"\n## {name}: {n} evaluations ({n * S.EVAL_DT:.0f} s)")
        print("| level | old s | new s | old % | new % |\n|---|---|---|---|---|")
        for k, lab in ((0, "0"), (1, "1"), (2, "2"), (255, "0xFF")):
            print(f"| {lab} | {so[k] * S.EVAL_DT:.0f} | {sn[k] * S.EVAL_DT:.0f} | {100 * so[k] / n:.2f} | {100 * sn[k] / n:.2f} |")
        # level changes per evaluation pair
        up = [(o, w) for o, w in zip(old, new) if w["level"] == 2 and o["level"] != 2]
        dn = [(o, w) for o, w in zip(old, new) if o["level"] == 2 and w["level"] != 2]
        print(f"level 2 only in NEW: {len(up) * S.EVAL_DT:.2f} s; level 2 only in OLD: {len(dn) * S.EVAL_DT:.2f} s")
        t0 = time.time()
        segs = S.segments([w["t"] for o, w in up], 0.5)
        wall0 = None
        for s in segs[:12]:
            o, w = next(x for x in up if x[1]["t"] == s[0])
            tg = w["tgt"]
            print(f"   t={s[0]:.1f}s dur={s[1] - s[0] + S.EVAL_DT:.2f}s speed={w['speed']} old_level={o['level']} new target " +
                  ("-" if tg is None else f"id{tg.id} long={tg.long_m:.1f} lat={tg.lat_m:.1f} vrel={tg.vrel_ms:.1f} dyn={tg.dyn}") + f" | {w['reason'][-110:]}")
        if len(segs) > 12: print(f"   ... {len(segs) - 12} more segment(s)")
        n1 = [(o, w) for o, w in zip(old, new) if w["level"] == 1 and o["level"] == 0]
        import collections
        tags = collections.Counter(("dyn%s" % (w["tgt"].dyn if w["tgt"] else "-")) + "/" + (";".join(w["supp"]) or "no-suppression") for o, w in n1)
        print("   level-1-only-in-NEW breakdown (target dyn / suppression tag): " + str(dict(tags.most_common(5))))
        print(f"level 1 only-in-NEW (old=0): {len(n1) * S.EVAL_DT:.1f} s in {len(S.segments([w['t'] for o, w in n1], 0.5))} segment(s); speed_src new: " +
              str({k: sum(1 for e in new if e['src'] == k) for k in {e['src'] for e in new}}))


if __name__ == "__main__":
    main()
