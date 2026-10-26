"""K estimation from the driver's USUAL braking (S5-b). Pure logic + persistence + a log CLI.

K = effective braking coefficient = deceleration the driver actually uses when slowing down, divided by g (NOT the car's maximum). Someone who
normally presses half as hard gets a lower K, so warnings come earlier. K*g is the deceleration assumed in danger.py (d_req, a_req).

Pipeline (all on the Pi5; the firmware and its BRAKE_ACTIVE flag are NOT used):
 1. IMU bias: when OBD speed has been 0 for >= K_BIAS_STOP_S, the median forward acceleration acc_x of that standstill is the bias b (mounting/slope
    component, ~ +90 mg at pitch -5 deg in the logs [hypothesis: = g*sin(5 deg) = 87 mg]). The first stop sets b, later stops update it slowly (K_BIAS_ALPHA).
    Until b is known no event is evaluated and the default K is used.
 2. Braking event: corrected deceleration a_dec = -(acc_x - b)/1000 [g], 3-sample median filtered, >= K_EVENT_A_MIN_G for >= K_EVENT_MIN_S
    (dips < K_EVENT_GAP_S allowed) AND the OBD speed falls by >= K_EVENT_SPEED_DROP_KMH over the event. Representative deceleration = mean of the
    central K_EVENT_CENTER_FRAC of the event. The OBD speed slope is stored as an independent cross-check.
 3. K = 75 % quantile [hypothesis: reduces the under-estimation caused by gentle "waiting for a red light" decelerations] of the last K_N_EVENTS
    representative decelerations, clamped to [K_MIN, K_MAX] (team range, unchanged). With fewer than K_MIN_EVENTS events measured() is None
    (BSM advertises N/A) and current() gives K_DEFAULT_REAR for calculations only.
 4. Profile: bias, last events and K are stored in state/k_profile.json (survives restarts). --reset-k clears it, --k-fixed pins a value (demo).
LIMITS: pitch/road slope and suspension dive are not corrected (only the standstill bias is); one axis; OBD speed is integer km/h at ~3 Hz; the profile is per driver.
Times are STM32 timestamps in seconds (host time is chunked and unusable for durations); OBD samples are placed on that clock by the caller.
"""
import json
import os
import sys
import time
from collections import deque
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C

DEFAULT_PROFILE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "state", "k_profile.json"))


def quantile(v, q):
    v = sorted(v)
    if not v:
        return float("nan")
    k = (len(v) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def median(v):
    return quantile(v, 0.5)


class KEstimator:
    def __init__(self, profile_path: Optional[str] = None, reset: bool = False, k_fixed: Optional[float] = None, bias_override_mg: Optional[float] = None):
        self.profile_path, self.k_fixed = profile_path, k_fixed
        self.bias_mg: Optional[float] = bias_override_mg
        self._bias_forced = bias_override_mg is not None        # analysis only: use this bias, never update it
        self.stops, self.events, self.rejected = [], [], []     # stops/events: dicts; rejected: events that failed a check
        self.t_latest = None
        # stop tracking
        self._stop_t0 = self._stop_tlast = None
        self._stop_acc, self._stop_pitch, self._stop_provisional = [], [], False
        # IMU stream
        self._raw = deque(maxlen=3)                            # (t, a_dec_g, pitch) corrected samples for the median filter
        self._ev = None                                        # running event: dict(t0, t_last_above, samples=[(t, a, pitch)], below_since)
        self._speed = deque(maxlen=2000)                       # (t, kmh)
        self._pitch_hist = deque(maxlen=60)                    # (t, pitch) last ~3 s for the pre-event pitch
        self.no_bias_samples = 0
        if profile_path and not reset:
            self._load()

    # ---------------- persistence
    def _load(self):
        try:
            with open(self.profile_path) as f:
                d = json.load(f)
            if not self._bias_forced:
                self.bias_mg = d.get("bias_mg")
            self.events = d.get("events", [])[-C.K_N_EVENTS:]
            self.stops = d.get("stops", [])[-20:]
        except (OSError, ValueError):
            pass

    def save(self):
        if not self.profile_path:
            return
        os.makedirs(os.path.dirname(self.profile_path), exist_ok=True)
        d = dict(version=1, updated=time.strftime("%Y-%m-%d %H:%M:%S"), bias_mg=self.bias_mg, k=self.measured(), n_events=len(self.events),
                 note="K = driver's usual braking strength (decel/g); see k_estimator.py", events=self.events[-C.K_N_EVENTS:], stops=self.stops[-20:],
                 k_fixed=self.k_fixed)
        tmp = self.profile_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, self.profile_path)

    # ---------------- inputs
    def add_speed(self, t, kmh):
        """OBD speed sample at STM32 time t [s]; kmh None = unknown (0xFF)."""
        self.t_latest = t if self.t_latest is None else max(self.t_latest, t)
        if kmh is None:
            self._end_stop(self._stop_tlast)
            return
        self._speed.append((t, kmh))
        if kmh == 0:
            if self._stop_t0 is None or (self._stop_tlast is not None and t - self._stop_tlast > 1.0):
                self._end_stop(self._stop_tlast)
                self._stop_t0, self._stop_acc, self._stop_pitch, self._stop_provisional = t, [], [], False
            self._stop_tlast = t
            if self.bias_mg is None and not self._bias_forced and t - self._stop_t0 >= C.K_BIAS_STOP_S:
                self._set_bias_from_stop(provisional=True)
        else:
            self._end_stop(self._stop_tlast)

    def add_imu(self, t, acc_x_mg, pitch_deg=None):
        """One IMU sample (20 Hz): t = STM32 0x13 ts [s]; acc_x_mg forward-positive (raw, uncorrected)."""
        if self.t_latest is not None and t < self.t_latest - 1.0:          # STM32 clock restarted: drop running state, keep bias/events
            self._ev, self._stop_t0, self._stop_tlast = None, None, None
            self._raw.clear(); self._speed.clear(); self._pitch_hist.clear()
        self.t_latest = t if self.t_latest is None else max(self.t_latest, t)
        if pitch_deg is not None:
            self._pitch_hist.append((t, pitch_deg))
        if self._stop_t0 is not None and t >= self._stop_t0 + C.K_BIAS_SETTLE_S:
            self._stop_acc.append(acc_x_mg)
            if pitch_deg is not None:
                self._stop_pitch.append(pitch_deg)
        if self.bias_mg is None:
            self.no_bias_samples += 1
            return
        a = -(acc_x_mg - self.bias_mg) / 1000.0
        self._raw.append((t, a, pitch_deg))
        if len(self._raw) == 3:                                             # 3-sample median removes single-sample spikes
            mid_t = self._raw[1][0]
            sm = median([x[1] for x in self._raw])
            self._step_event(mid_t, sm, self._raw[1][2])

    def finish(self):
        self._end_stop(self._stop_tlast)
        if self._ev is not None:
            self._close_event()

    # ---------------- bias from standstills
    def _set_bias_from_stop(self, provisional):
        if len(self._stop_acc) < 20:
            return
        self.bias_mg = median(self._stop_acc)
        self._stop_provisional = provisional

    def _end_stop(self, t_last):
        if self._stop_t0 is None:
            return
        dur = (t_last or self._stop_t0) - self._stop_t0
        if dur >= C.K_BIAS_STOP_S and len(self._stop_acc) >= 20 and not self._bias_forced:
            b_new = median(self._stop_acc)
            self.bias_mg = b_new if (self.bias_mg is None or self._stop_provisional) else (1 - C.K_BIAS_ALPHA) * self.bias_mg + C.K_BIAS_ALPHA * b_new
            pm = sum(self._stop_pitch) / len(self._stop_pitch) if self._stop_pitch else None
            self.stops.append(dict(t=round(self._stop_t0, 2), dur=round(dur, 1), bias_mg=round(b_new, 1), pitch_deg=None if pm is None else round(pm, 2), n=len(self._stop_acc)))
            self.save()
        self._stop_t0 = self._stop_tlast = None
        self._stop_acc, self._stop_pitch, self._stop_provisional = [], [], False

    # ---------------- braking events
    def _step_event(self, t, a, pitch):
        if self._ev is None:
            if a >= C.K_EVENT_A_MIN_G:
                self._ev = dict(t0=t, t_last=t, samples=[(t, a, pitch)], below_since=None)
            return
        ev = self._ev
        ev["samples"].append((t, a, pitch))
        if a >= C.K_EVENT_A_MIN_G:
            ev["t_last"], ev["below_since"] = t, None
        else:
            ev["below_since"] = t if ev["below_since"] is None else ev["below_since"]
            if t - ev["below_since"] > C.K_EVENT_GAP_S:
                self._close_event()

    def _speed_around(self, t0, t1):
        """OBD speed at the start/end of the event: (v_start, v_end, t_vstart, t_vend) or None."""
        s = [x for x in self._speed]
        pre = [x for x in s if t0 - 0.8 <= x[0] <= t0 + 0.1]
        post = [x for x in s if t1 - 0.1 <= x[0] <= t1 + 0.8]
        if not pre or not post:
            return None
        a = max(pre, key=lambda x: x[1]); b = min(post, key=lambda x: x[1])
        return a[1], b[1], a[0], b[0]

    def _close_event(self):
        ev, self._ev = self._ev, None
        t0, t1 = ev["t0"], ev["t_last"]
        smp = [x for x in ev["samples"] if x[0] <= t1]
        dur = t1 - t0
        info = dict(t=round(t0, 2), dur=round(dur, 2), n=len(smp))
        if dur < C.K_EVENT_MIN_S:
            self.rejected.append(dict(info, why=f"too short ({dur:.2f}s)")); return
        sp = self._speed_around(t0, t1)
        if sp is None:
            self.rejected.append(dict(info, why="no OBD speed around the event")); return
        v0, v1, tv0, tv1 = sp
        if v0 - v1 < C.K_EVENT_SPEED_DROP_KMH:
            self.rejected.append(dict(info, why=f"OBD speed did not fall ({v0:.0f}->{v1:.0f} km/h)", peak_g=round(max(x[1] for x in smp), 3))); return
        lo = int(len(smp) * (1 - C.K_EVENT_CENTER_FRAC) / 2); hi = len(smp) - lo
        core = [x[1] for x in smp[lo:hi]] or [x[1] for x in smp]
        rep = sum(core) / len(core)
        pre_p = [p for tt, p in self._pitch_hist if t0 - 1.2 <= tt < t0 - 0.2]
        mid_p = [x[2] for x in smp[lo:hi] if x[2] is not None]
        dpitch = (sum(mid_p) / len(mid_p) - sum(pre_p) / len(pre_p)) if (pre_p and mid_p) else None
        obd_dec = ((v0 - v1) / 3.6 / (tv1 - tv0) / 9.80665) if tv1 > tv0 else None
        self.events.append(dict(info, v0=v0, v1=v1, rep_g=round(rep, 3), peak_g=round(max(x[1] for x in smp), 3), obd_g=None if obd_dec is None else round(obd_dec, 3),
                                dpitch_deg=None if dpitch is None else round(dpitch, 2)))
        self.events = self.events[-200:]
        self.save()

    # ---------------- outputs
    def _k_raw(self):
        last = self.events[-C.K_N_EVENTS:]
        return None if len(last) < C.K_MIN_EVENTS else quantile([e["rep_g"] for e in last], C.K_QUANTILE)

    def measured(self, now=None) -> Optional[float]:
        """K to advertise (BSM) or None = N/A. `now` is accepted for compatibility and ignored."""
        if self.k_fixed is not None:
            return self.k_fixed
        q = self._k_raw()
        return None if q is None else min(C.K_MAX, max(C.K_MIN, q))

    def current(self, now=None):
        """(K used in the danger formulas, source 'fixed'|'measured'|'default')."""
        if self.k_fixed is not None:
            return self.k_fixed, "fixed"
        m = self.measured()
        return (m, "measured") if m is not None else (C.K_DEFAULT_REAR, "default")


def _selftest():
    import random
    rnd = random.Random(3)
    e = KEstimator()
    t = 0.0
    def run(seconds, speed_fn, acc_fn, pitch=-5.0):
        nonlocal t
        n = int(seconds * 20)
        for i in range(n):
            e.add_imu(t, acc_fn(t) + rnd.gauss(0, 8), pitch)
            if i % 6 == 0:
                e.add_speed(t, speed_fn(t))
            t += 0.05
    # 1) stand still 4 s with +90 mg bias -> bias measured
    run(4, lambda _: 0, lambda _: 90.0)
    run(0.5, lambda _: 5, lambda _: 90.0)          # moving off ends the stop
    assert e.bias_mg is not None and abs(e.bias_mg - 90) < 8, e.bias_mg
    assert e.stops and abs(e.stops[0]["bias_mg"] - 90) < 8
    # 2) five real braking events: true decel 0.25 g (raw acc_x = 90 - 250), speed 60 -> 40 km/h over 2 s
    for k in range(5):
        v0 = 60.0
        t_ev = t
        run(2.0, lambda tt, t_ev=t_ev: v0 - 10.0 * (tt - t_ev), lambda _: 90.0 - 250.0)
        run(2.0, lambda _: 40, lambda _: 90.0)
    e.finish()
    assert len(e.events) == 5, (len(e.events), e.rejected)
    assert all(abs(x["rep_g"] - 0.25) < 0.03 for x in e.events), [x["rep_g"] for x in e.events]
    k = e.measured()
    assert k is not None and abs(k - 0.3) < 1e-9 or abs(k - 0.25) < 0.03, k            # 0.25 g is below K_MIN -> clamped to 0.3
    assert e.measured() == C.K_MIN
    # 3) events fall below 3 -> N/A and default
    e2 = KEstimator()
    assert e2.measured() is None and e2.current() == (C.K_DEFAULT_REAR, "default")
    # 4) without a known bias nothing is evaluated
    e3 = KEstimator()
    for i in range(100):
        e3.add_imu(i * 0.05, -300.0, -5.0)
    e3.finish(); assert not e3.events and e3.no_bias_samples == 100
    # 5) a deceleration without speed drop (bump / slope) is rejected
    e4 = KEstimator(bias_override_mg=0.0)
    for i in range(60):
        e4.add_imu(i * 0.05, -200.0 if 10 <= i < 30 else 0.0)
        if i % 6 == 0: e4.add_speed(i * 0.05, 50)
    e4.finish(); assert not e4.events and e4.rejected and "did not fall" in e4.rejected[-1]["why"], e4.rejected
    # 6) persistence + reset + fixed
    import tempfile
    pth = os.path.join(tempfile.mkdtemp(), "k_profile.json")
    e5 = KEstimator(profile_path=pth)
    run_e = e5
    for ev in e.events: e5.events.append(ev)
    e5.bias_mg = 88.0; e5.save()
    e6 = KEstimator(profile_path=pth)
    assert len(e6.events) == 5 and e6.bias_mg == 88.0 and e6.measured() == C.K_MIN
    e7 = KEstimator(profile_path=pth, reset=True); assert e7.events == [] and e7.measured() is None
    e8 = KEstimator(k_fixed=0.62); assert e8.current() == (0.62, "fixed") and e8.measured() == 0.62
    print("k_estimator selftest OK")


def _final_bias(rows):
    """Bias of a first pass over the log (used only to emulate a stored profile)."""
    e = KEstimator()
    stm_t = None
    for r in rows:
        typ = int(r[1], 16); dec = json.loads(r[3]) if r[3] else {}
        if typ == 0x13 and dec:
            stm_t = dec["ts"] / 1000.0; e.add_imu(stm_t, dec["acc_mg"][0], dec["rpy_deg"][1])
        elif typ == 0x14 and stm_t is not None:
            e.add_speed(stm_t, dec.get("speed_kmh"))
    return e.bias_mg if e.bias_mg is not None else 0.0


def _cli(drives):
    import csv
    for d in drives:
        rows = list(csv.reader(open(os.path.join(d, "frames.csv"))))[1:]
        name = os.path.basename(d.rstrip("/"))
        flag_runs, imu_n = 0, 0
        variants = {"with bias correction": KEstimator(), "without correction (bias 0)": KEstimator(bias_override_mg=0.0)}
        pre_bias = _final_bias(rows)          # what a stored profile (bias known from an earlier drive) would give
        variants["bias known from the start (stored profile)"] = KEstimator(bias_override_mg=pre_bias)
        stm_t = None; ts0 = None; imu_pitch = []
        for r in rows:
            typ = int(r[1], 16); dec = json.loads(r[3]) if r[3] else {}
            if typ in (0x10, 0x11, 0x12, 0x13) and dec:
                tsv = dec.get("ts", dec.get("uptime_ms"))
                if tsv is not None:
                    if stm_t is not None and tsv / 1000.0 < stm_t - 1.0:
                        stm_t = None
                    stm_t = tsv / 1000.0 if stm_t is None else max(stm_t, tsv / 1000.0)
                    ts0 = stm_t if ts0 is None else ts0
            if typ == 0x13 and dec:
                imu_n += 1
                for k in variants.values():
                    k.add_imu(dec["ts"] / 1000.0, dec["acc_mg"][0], dec["rpy_deg"][1])
            elif typ == 0x14 and stm_t is not None:
                for k in variants.values():
                    k.add_speed(stm_t, dec.get("speed_kmh"))
        for k in variants.values():
            k.finish()
        kb = variants["with bias correction"]; k0 = variants["without correction (bias 0)"]
        print(f"\n## {name}: {imu_n} IMU samples")
        print(f"standstills used for the bias ({len(kb.stops)}): final bias {kb.bias_mg if kb.bias_mg is None else round(kb.bias_mg, 1)} mg; evaluated causally (events before the first standstill are not evaluated: {kb.no_bias_samples} IMU samples skipped)")
        if kb.stops:
            print("| stop t(s) | dur s | bias mg | mean pitch deg | 1000*sin(pitch) mg |\n|---|---|---|---|---|")
            import math
            for s in kb.stops[:8]:
                pm = s["pitch_deg"]
                print(f"| {s['t'] - ts0:.0f} | {s['dur']} | {s['bias_mg']:+.0f} | {'' if pm is None else f'{pm:+.1f}'} | {'' if pm is None else f'{1000 * math.sin(math.radians(pm)):+.0f}'} |")
            if len(kb.stops) > 8:
                print(f"| ... {len(kb.stops) - 8} more | | | | |")
            xs = [(s["pitch_deg"], s["bias_mg"]) for s in kb.stops if s["pitch_deg"] is not None]
            if len(xs) >= 3:
                mx = sum(x for x, _ in xs) / len(xs); my = sum(y for _, y in xs) / len(xs)
                cov = sum((x - mx) * (y - my) for x, y in xs); vx = sum((x - mx) ** 2 for x, _ in xs); vy = sum((y - my) ** 2 for _, y in xs)
                print(f"bias vs standstill pitch over {len(xs)} stops: correlation {cov / (vx * vy) ** .5:+.2f}; mean bias {my:+.0f} mg, mean pitch {mx:+.1f} deg, 1000*sin(|pitch|) = {1000 * math.sin(math.radians(abs(mx))):.0f} mg")
        kp = variants["bias known from the start (stored profile)"]
        print(f"braking events: causal bias correction {len(kb.events)} accepted / {len(kb.rejected)} rejected; WITHOUT correction {len(k0.events)} accepted / {len(k0.rejected)} rejected; "
              f"bias known from the start ({pre_bias:+.0f} mg) {len(kp.events)} accepted / {len(kp.rejected)} rejected")
        if kp.events and not kb.events:
            print("   (events with the stored-profile bias:) " + "; ".join(f"t={e['t'] - ts0:.0f}s {e['rep_g']:.3f} g {e['v0']:.0f}->{e['v1']:.0f} km/h" for e in kp.events[:8]))
        if kb.events:
            print("| t(s) | dur s | OBD km/h | rep decel g | peak g | OBD-slope decel g | dpitch deg | K after this event (running) |\n|---|---|---|---|---|---|---|---|")
            run = KEstimator()
            for ev in kb.events:
                run.events.append(ev)
                kk = run.measured()
                print(f"| {ev['t'] - ts0:.0f} | {ev['dur']} | {ev['v0']:.0f}->{ev['v1']:.0f} | {ev['rep_g']:.3f} | {ev['peak_g']:.3f} | {'' if ev['obd_g'] is None else ev['obd_g']} | {'' if ev['dpitch_deg'] is None else ev['dpitch_deg']} | {'N/A (<3 events)' if kk is None else f'{kk:.2f}'} |")
            pairs = [(e['dpitch_deg'], e['rep_g']) for e in kb.events if e['dpitch_deg'] is not None]
            if len(pairs) >= 4:
                mx = sum(x for x, _ in pairs) / len(pairs); my = sum(y for _, y in pairs) / len(pairs)
                cov = sum((x - mx) * (y - my) for x, y in pairs); vx = sum((x - mx) ** 2 for x, _ in pairs); vy = sum((y - my) ** 2 for _, y in pairs)
                print(f"dpitch vs representative decel over {len(pairs)} events: correlation {cov / (vx * vy) ** .5 if vx * vy > 0 else float('nan'):+.2f}")
        for rj in kb.rejected[:6]:
            print(f"   rejected: t={rj['t'] - ts0:.0f}s dur={rj['dur']}s: {rj['why']}")
        m = kb.measured()
        print("K (last <=10 events, q75, clamped 0.3..1.0): " + ("N/A (fewer than 3 events) -> default 0.5 used for calculations only" if m is None else f"{m:.2f}"))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] != "--selftest":
        _cli(sys.argv[1:])
    else:
        _selftest()
