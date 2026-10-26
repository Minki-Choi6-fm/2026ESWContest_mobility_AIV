#!/usr/bin/env python3
"""Tests of buzzer_out.BuzzerSink with gpiozero's MockFactory ONLY -- no real pin is ever opened (a BuzzerSink without pin_factory/really refuses).
Checks: OFF at boot, danger pattern durations from constants, levels 0/1/0xFF/None never sound, caution after danger silences at once, update timeout (TTL),
safe_state, passive PWM, refusal to drive a real pin, OFF after an exception / SIGTERM / a dying pattern thread."""
import os, signal, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import constants as C
from buzzer_out import BuzzerSink
from gpiozero.pins.mock import MockFactory, MockPWMPin

res = []


def check(name, cond, detail=""):
    res.append((name, bool(cond), detail)); print(("PASS " if cond else "FAIL ") + name + (f"  [{detail}]" if detail else ""))


class Sampler:
    """Polls the (mock) pin state every 2 ms in its own thread and records the changes: (t, state). MockPin.states timestamps are not reliable for durations."""
    def __init__(self, pin):
        self.pin, self.ch, self.stop = pin, [], False
        self.t0 = time.monotonic(); self.th = threading.Thread(target=self._run, daemon=True); self.th.start()

    def _run(self):
        last = None
        while not self.stop:
            v = self.pin.state
            if v != last:
                self.ch.append((time.monotonic() - self.t0, v)); last = v
            time.sleep(0.002)

    def finish(self):
        self.stop = True; self.th.join()
        on = [b[0] - a[0] for a, b in zip(self.ch, self.ch[1:]) if a[1]]
        off = [b[0] - a[0] for a, b in zip(self.ch, self.ch[1:]) if not a[1]]
        return on, off


def feed(sink, level, seconds, hz=20):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        sink.update(time.monotonic(), level, "t", {})
        time.sleep(1.0 / hz)


# 1) boot state OFF; refuses a real pin
try:
    BuzzerSink()
    check("refuses to drive a REAL pin without really=True", False)
except RuntimeError as e:
    check("refuses to drive a REAL pin without really=True", "really=True" in str(e))
f = MockFactory(); s = BuzzerSink(pin_factory=f); pin = f.pin(C.BUZZER_BCM_PIN)
time.sleep(0.1)
check("OFF at boot (nothing sounds before the first update)", not s.on and pin.state == 0 and all(not x.state for x in pin.states))
# 2) danger pattern (sampled from the pin itself)
sm = Sampler(pin)
feed(s, 2, 1.4)
on, off = sm.finish()
on_, off_ = on[1:], off[1:-0 or None]                                  # the very first pulse starts one thread tick (<= 10 ms) late, so it is not judged
check(f"level 2 -> pattern {C.BUZZER_ON_S:g} s on / {C.BUZZER_OFF_S:g} s off", len(on_) >= 3 and all(abs(d - C.BUZZER_ON_S) < 0.03 for d in on_) and all(abs(d - C.BUZZER_OFF_S) < 0.03 for d in off_[1:]),
      f"{len(on)} on-pulses, on {[round(d, 3) for d in on]}, off {[round(d, 3) for d in off]}")
# 3) caution right after danger silences at once
t = time.monotonic(); s.update(t, 1, "caution", {}); time.sleep(0.03)
check("danger -> caution: output OFF within 30 ms and stays off", not s.on and pin.state == 0)
feed(s, 1, 0.4)
check("caution (1) keeps it silent", not s.on and pin.state == 0)
s.close()
# 4) levels that must never sound
for lv, name in ((0, "0 safe"), (1, "1 caution"), (0xFF, "0xFF unknown"), (None, "None")):
    f = MockFactory(); s = BuzzerSink(pin_factory=f); pin = f.pin(C.BUZZER_BCM_PIN)
    feed(s, lv, 0.7)
    check(f"level {name} never sounds", s.transitions == 0 and all(not x.state for x in pin.states), f"transitions={s.transitions}")
    s.close()
# 5) update timeout (hung main loop): danger updates stop -> OFF by itself
f = MockFactory(); s = BuzzerSink(pin_factory=f); pin = f.pin(C.BUZZER_BCM_PIN)
feed(s, 2, 0.4); last = time.monotonic()
while time.monotonic() - last < C.BUZZER_TTL_S + 0.3:
    time.sleep(0.02)
check(f"no update for {C.BUZZER_TTL_S} s -> OFF by itself", not s.on and pin.state == 0)
time.sleep(0.4)
check("... and it stays off", not s.on and pin.state == 0)
# 6) safe_state is immediate and idempotent
feed(s, 2, 0.5)                                                      # ringing again (pattern may be in an on or off phase)
s.safe_state("test"); s.safe_state("again")
check("safe_state() -> OFF at once, repeatable", not s.on and pin.state == 0)
time.sleep(0.2)
check("... and no more pulses after safe_state until a new update", not s.on and pin.state == 0)
# 7) dying pattern thread -> OFF
feed(s, 2, 0.35)
def boom():
    raise RuntimeError("clock failure (test)")
s.clock = boom
time.sleep(0.2)
check("pattern thread dies -> output OFF (finally)", not s.on and pin.state == 0 and not s.thread.is_alive())
s.close()
# 8) passive buzzer: PWM at BUZZER_PASSIVE_HZ, 50 % duty when on, 0 when off
f = MockFactory(pin_class=MockPWMPin); s = BuzzerSink(kind="passive", pin_factory=f); pin = f.pin(C.BUZZER_BCM_PIN)
check("passive: PWM frequency from constants and duty 0 at boot", abs(pin.frequency - C.BUZZER_PASSIVE_HZ) < 1 and pin.state == 0, f"{pin.frequency} Hz")
saw_on = False
t0 = time.monotonic()
while time.monotonic() - t0 < 0.6:
    s.update(time.monotonic(), 2, "t", {}); saw_on |= abs(pin.state - 0.5) < 1e-6; time.sleep(0.01)
check("passive: 50 % duty while danger pattern is on", saw_on)
s.update(time.monotonic(), 0xFF, "unknown", {}); time.sleep(0.03)
check("passive: unknown (0xFF) -> duty 0", pin.state == 0)
s.close()

# 9) exception / SIGTERM in a separate process: pin state printed by an atexit hook that runs AFTER the buzzer's own
code = r'''
import atexit, sys, time, signal
sys.path.insert(0, %(here)r)
from buzzer_out import BuzzerSink
from gpiozero.pins.mock import MockFactory
f = MockFactory(); pin_holder = {}
atexit.register(lambda: print("FINAL_STATE=%%d" %% f.pin(26).state, flush=True))   # registered first -> runs last
s = BuzzerSink(pin_factory=f); pin_holder["s"] = s
mode = sys.argv[1]
if mode == "sigterm":
    s.install_signal_handlers()
t0 = time.monotonic()
while time.monotonic() - t0 < 0.5:
    s.update(time.monotonic(), 2, "danger", {}); time.sleep(0.02)
print("ON_SEEN=%%d" %% s.transitions, flush=True)
if mode == "exception":
    raise RuntimeError("crash while the buzzer is ringing")
print("READY", flush=True)
while True:
    s.update(time.monotonic(), 2, "danger", {}); time.sleep(0.02)
''' % dict(here=HERE)
p = subprocess.run([sys.executable, "-I", "-c", code, "exception"], capture_output=True, text=True, timeout=20)
check("uncaught exception while ringing -> OFF at interpreter exit", p.returncode == 1 and "FINAL_STATE=0" in p.stdout and "ON_SEEN=" in p.stdout and "ON_SEEN=0" not in p.stdout, p.stdout.replace("\n", " | "))
p = subprocess.Popen([sys.executable, "-I", "-c", code, "sigterm"], stdout=subprocess.PIPE, text=True)
while True:
    line = p.stdout.readline()
    if "READY" in line or not line:
        break
time.sleep(0.15)
p.send_signal(signal.SIGTERM)
out = p.stdout.read(); rc = p.wait(timeout=10)
check("SIGTERM while ringing -> OFF and exit 143", rc == 143 and "FINAL_STATE=0" in out, f"rc={rc} {out.strip()!r}")

bad = [r for r in res if not r[1]]
print(f"\n{len(res) - len(bad)}/{len(res)} PASS (MockFactory only; no real GPIO pin was opened)")
sys.exit(1 if bad else 0)
