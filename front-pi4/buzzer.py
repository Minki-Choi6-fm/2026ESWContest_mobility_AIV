PATTERNS = {1: (0.3, 0.7), 2: (0.1, 0.1)}

class GpioBuzzer:
    def __init__(self, gpio=18):
        from gpiozero import Buzzer
        self._bz = Buzzer(gpio, active_high=True, initial_value=False)
        self.level = 0
    def set_level(self, level):
        if level == self.level:
            return
        self.level = level
        if level in PATTERNS:
            on_t, off_t = PATTERNS[level]
            self._bz.beep(on_time=on_t, off_time=off_t, background=True)
        else:
            self._bz.off()
    def on(self):
        self.level = -1
        self._bz.on()
    def off(self):
        self.level = 0
        self._bz.off()
    def close(self):
        try:
            self._bz.off()
        finally:
            self._bz.close()
