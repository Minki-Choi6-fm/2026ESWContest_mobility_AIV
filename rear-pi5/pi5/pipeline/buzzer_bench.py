#!/usr/bin/env python3
"""Manual bench test of the rear buzzer: ON for 2 seconds, then OFF. BCM GPIO26 = physical pin 37, GND = physical pin 34. gpiozero only.

DEFAULT RUN TOUCHES NO REAL PIN: it uses gpiozero's MockFactory and only prints what it WOULD do.
The real pin is driven ONLY with   --really-drive-gpio26   AND after you type the confirmation phrase (or pass --yes, meant for a person standing at the car).
    python3 -I buzzer_bench.py                                    # dry run (mock pin)
    python3 -I buzzer_bench.py --really-drive-gpio26 --type active   # real: active buzzer = 2 s steady tone
    python3 -I buzzer_bench.py --really-drive-gpio26 --type passive  # real: passive piezo = 2 s of PWM at BUZZER_PASSIVE_HZ (3900 Hz, 50 %)
    add  --pattern  to hear the danger pattern (0.15 s on / 0.15 s off) instead of a steady tone
Whatever happens (Ctrl+C, SIGTERM, exception, normal end) the pin is switched OFF and released at exit.
"""
import argparse, atexit, os, signal, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C

PHRASE = "DRIVE GPIO26"


def main():
    ap = argparse.ArgumentParser(description="rear buzzer bench test (2 s)")
    ap.add_argument("--type", choices=["active", "passive"], default="active")
    ap.add_argument("--really-drive-gpio26", action="store_true", help="drive the REAL pin (otherwise a mock pin is used)")
    ap.add_argument("--yes", action="store_true", help="skip typing the confirmation phrase (only with --really-drive-gpio26)")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--check-levels", action="store_true", help="with --pattern: verify 0/1/255 and speed inhibit keep output OFF")
    ap.add_argument("--pattern", action="store_true", help="use the danger pattern instead of a steady tone")
    a = ap.parse_args()
    a.seconds = min(max(a.seconds, 0.2), 5.0)                 # never longer than 5 s
    real = a.really_drive_gpio26
    if real and not a.yes:
        print(f"This will drive the REAL pin BCM GPIO{C.BUZZER_BCM_PIN} (physical pin 37; GND on pin 34) for {a.seconds:g} s as a {a.type} buzzer.")
        if input(f'Type "{PHRASE}" to continue: ').strip() != PHRASE:
            print("not confirmed -> nothing was touched"); return 1
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
    kw = {}
    if not real:
        from gpiozero.pins.mock import MockFactory, MockPWMPin
        kw["pin_factory"] = MockFactory(pin_class=MockPWMPin) if a.type == "passive" else MockFactory()
        print("DRY RUN with a MOCK pin (no real GPIO is touched). Use --really-drive-gpio26 for the real pin.")
    if a.pattern:
        from buzzer_out import BuzzerSink
        sink = BuzzerSink(kind=a.type, pin_factory=kw.get("pin_factory"), really=real)
        sink.install_signal_handlers()
        t0 = time.monotonic()
        try:
            if a.check_levels:
                for lv, info in [(0, {}), (1, {}), (255, {}), (2, {"buzzer_inhibit": True})]:
                    sink.update(time.monotonic(), lv, "bench off check", info)
                    time.sleep(0.1)
                    if sink.dev.value != 0:
                        raise RuntimeError(f"OFF check failed: level={lv}, info={info}")
                    print(f"OFF verified: level={lv}, inhibit={bool(info)}")
            t0 = time.monotonic()
            while time.monotonic() - t0 < a.seconds:
                sink.update(time.monotonic(), 2, "bench", {}); time.sleep(0.02)
        finally:
            try:
                sink.safe_state("bench end")
                value = sink.dev.value
                print(f"GPIO{C.BUZZER_BCM_PIN} readback before release: {value} (0=OFF)")
                if value != 0:
                    raise RuntimeError("GPIO output did not return OFF")
            finally:
                sink.close()
        print(f"pattern finished; pin transitions: {sink.transitions}; pin left OFF")
        return 0
    dev = DigitalOutputDevice(C.BUZZER_BCM_PIN, initial_value=False, **kw) if a.type == "active" else PWMOutputDevice(C.BUZZER_BCM_PIN, initial_value=0, frequency=C.BUZZER_PASSIVE_HZ, **kw)
    def off(*_):
        try:
            dev.off() if a.type == "active" else setattr(dev, "value", 0)
        except Exception:                                          # noqa: BLE001
            pass
    atexit.register(off)
    for sg in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sg, lambda s, f: (off(), sys.exit(128 + s)))
    try:
        print(f"ON for {a.seconds:g} s ({a.type}{', ' + str(C.BUZZER_PASSIVE_HZ) + ' Hz PWM 50 %' if a.type == 'passive' else ''}) ...")
        dev.on() if a.type == "active" else setattr(dev, "value", 0.5)
        time.sleep(a.seconds)
    finally:
        off(); dev.close()
    print("OFF, pin released.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
