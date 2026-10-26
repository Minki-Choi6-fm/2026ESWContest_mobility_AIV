"""Rear buzzer output (S8-d). gpiozero only (RPi.GPIO is NOT used), BCM GPIO26 = physical pin 37, GND = physical pin 34.

Policy (docs/handoff/OUTPUT_POLICY.md): it sounds ONLY at level 2 (danger), with a 0.15 s on / 0.15 s off pattern. Levels 0, 1 (caution = display only), 0xFF (unknown)
and None NEVER sound. Safe state = OFF: at creation, on safe_state(), on close(), at interpreter exit (atexit), on SIGTERM/SIGINT when install_signal_handlers() is used,
when the update stream stops for BUZZER_TTL_S (own thread, so a hung main loop cannot leave it on), and if the thread itself dies (finally).
Type: "active" = plain on/off (DigitalOutputDevice); "passive" = PWM at BUZZER_PASSIVE_HZ, 50 % duty (PWMOutputDevice). The real type is UNCONFIRMED:
test an unknown piezo with 5 V DC / see docs/handoff/BUZZER_BENCH.md before choosing.

REAL PIN SAFETY: a BuzzerSink drives a real pin only when really=True is given; otherwise a pin_factory (e.g. gpiozero MockFactory) is required.
Nothing in this module runs at import time.
"""
import atexit
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C


class BuzzerSink:
    """OutputSink-compatible (update / safe_state / close)."""

    def __init__(self, kind="active", pin_factory=None, really=False, pin=None, clock=time.monotonic):
        if kind not in ("active", "passive"):
            raise ValueError("kind must be 'active' or 'passive'")
        if pin_factory is None and not really:
            raise RuntimeError("refusing to drive a REAL GPIO pin without really=True (needs the user's approval); pass a gpiozero pin_factory (e.g. MockFactory) for tests")
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
        self.kind, self.clock, self.pin = kind, clock, C.BUZZER_BCM_PIN if pin is None else pin
        kw = dict(pin_factory=pin_factory) if pin_factory is not None else {}
        if kind == "active":
            self.dev = DigitalOutputDevice(self.pin, active_high=True, initial_value=False, **kw)      # initial_value=False: OFF from the very first instant
        else:
            self.dev = PWMOutputDevice(self.pin, active_high=True, initial_value=0, frequency=C.BUZZER_PASSIVE_HZ, **kw)
        self.lock = threading.Lock()
        self.level, self.t_update, self.t_danger = None, None, None
        self.on, self.closed, self.transitions = False, False, 0
        self._stop = threading.Event()
        self._off()
        atexit.register(self._off_atexit)
        self.thread = threading.Thread(target=self._run, name="buzzer", daemon=True)
        self.thread.start()

    # ---- low level (call with the lock held, or from the thread owner)
    def _set(self, on):
        if on == self.on or self.closed:
            return
        if self.kind == "active":
            self.dev.on() if on else self.dev.off()
        else:
            self.dev.value = 0.5 if on else 0
        self.on = on
        self.transitions += 1

    def _off(self):
        with self.lock:
            if self.closed:
                return
            if self.kind == "active":
                self.dev.off()
            else:
                self.dev.value = 0
            self.on = False

    def _off_atexit(self):
        try:
            self._off()
        except Exception:                                   # noqa: BLE001  (interpreter is shutting down; never raise from atexit)
            pass

    # ---- sink interface
    def update(self, now, level, reason="", info=None):
        with self.lock:
            if level != 2 or (info or {}).get("buzzer_inhibit"):   # ONLY danger sounds. 0, 1, 0xFF, None: never. S12: also never while own speed is unknown (info["buzzer_inhibit"])
                self.t_danger = None
                self.level = level
                self.t_update = self.clock()
                self._set(False)
                return
            if self.level != 2 or self.t_danger is None:
                self.t_danger = self.clock()
            self.level, self.t_update = level, self.clock()

    def safe_state(self, why=""):
        with self.lock:
            self.level, self.t_danger = None, None
            self._set(False)

    def close(self):
        self._stop.set()
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.0)
        self._off()
        with self.lock:
            self.closed = True
            try:
                self.dev.close()
            except Exception:                               # noqa: BLE001
                pass

    def install_signal_handlers(self):
        """For standalone use (bench/tests): SIGTERM/SIGINT -> buzzer off, then exit. pipeline_main handles its own signals."""
        def h(signum, frame):
            self._off_atexit()
            raise SystemExit(128 + signum)
        signal.signal(signal.SIGTERM, h)
        signal.signal(signal.SIGINT, h)

    # ---- pattern thread
    def _run(self):
        period = C.BUZZER_ON_S + C.BUZZER_OFF_S
        try:
            while not self._stop.wait(0.01):
                with self.lock:
                    now = self.clock()
                    want = False
                    if self.level == 2 and self.t_danger is not None and self.t_update is not None and now - self.t_update <= C.BUZZER_TTL_S:
                        want = ((now - self.t_danger) % period) < C.BUZZER_ON_S
                    self._set(want)
        finally:
            try:
                self._off()                                  # whatever ended the thread: OFF
            except Exception:                                # noqa: BLE001
                pass
