"""S12-마무리: big-letter danger display for the passenger seat (Pi5 terminal / monitor). OutputSink-compatible (update / safe_state / close).

Shows ONE big symbol for the level: 0 -> "0" (green, "SAFE"), 1 -> "1" (yellow, "CAUTION"), 2 -> "2" (red, reversed colours, "DANGER"), 0xFF -> "?" (grey, "UNKNOWN"),
OFF/safe state -> "-". A small status line under it: target distance / relative speed, speed source, camera flag, radar-down note. ANSI colours only, no extra packages.
It redraws at most 10 times per second, never writes to a serial port, never touches GPIO, and a broken terminal can only raise inside this sink (the SinkHub isolates sink errors).
Enable with pipeline_main.py --sinks csv,big  (the normal single-line console sink is separate: --sinks console).
Not tested on the car's monitor [확인 필요]; tested only by rendering into a pipe.
"""
import shutil
import sys
import time

BIG = {  # 7 rows x 7 cols, drawn with block characters (scaled by the terminal size)
    "0": ["  ###  ", " #   # ", "#     #", "#     #", "#     #", " #   # ", "  ###  "],
    "1": ["   #   ", "  ##   ", "   #   ", "   #   ", "   #   ", "   #   ", " ##### "],
    "2": [" ##### ", "#     #", "      #", "  #### ", " #     ", "#      ", "#######"],
    "?": [" ##### ", "#     #", "      #", "   ###  ", "   #    ", "        ", "   #    "],
    "-": ["       ", "       ", "       ", " ##### ", "       ", "       ", "       "],
}
STYLE = {  # fg/bg ANSI, caption
    "0": ("\x1b[30;42m", "SAFE  (0)"), "1": ("\x1b[30;43m", "CAUTION  (1)"), "2": ("\x1b[97;41m", "DANGER  (2)"),
    "?": ("\x1b[97;100m", "UNKNOWN  (?)"), "-": ("\x1b[0m", "OFF"),
}
RESET = "\x1b[0m"


def symbol(level):
    return {0: "0", 1: "1", 2: "2", 0xFF: "?"}.get(level, "?")


def render(sym, status, cols=80, rows=24):
    """Return the full screen text (pure function, used by the tests)."""
    sty, cap = STYLE[sym]
    scale_x = max(1, min(3, (cols - 2) // 7 // 2)); scale_y = max(1, min(2, (rows - 6) // 7))
    lines = []
    for r in BIG[sym]:
        row = "".join(("██" * scale_x if ch == "#" else "  " * scale_x) for ch in r)
        lines += [row] * scale_y
    w = max(len(lines[0]), len(cap) + 4, 30)
    out = ["\x1b[H\x1b[2J"]
    out.append(sty + cap.center(w + 4) + RESET)
    for ln in lines:
        out.append(sty + ln.center(w + 4) + RESET)
    out.append(sty + " " * (w + 4) + RESET)
    out.append(status[:cols - 1])
    return "\n".join(out) + "\n"


class BigConsoleSink:
    def __init__(self, stream=None, clock=time.monotonic, min_period=0.1):
        self.out = stream or sys.stdout
        self.clock, self.min_period = clock, min_period
        self.t_last, self.last_sym, self.frames = None, None, 0

    def _size(self):
        try:
            c = shutil.get_terminal_size((80, 24)); return c.columns, c.lines
        except Exception:
            return 80, 24

    def _draw(self, sym, status):
        cols, rows = self._size()
        self.out.write(render(sym, status, cols, rows)); self.out.flush(); self.frames += 1
        self.t_last, self.last_sym = self.clock(), sym

    def update(self, now, level, reason, info):
        sym = symbol(level); t = self.clock()
        if sym == self.last_sym and self.t_last is not None and t - self.t_last < self.min_period:
            return
        info = info or {}
        d = info.get("target_dist")
        st = []
        st.append("target %.1f m" % d if d is not None else "no target")
        if info.get("buzzer_inhibit"):
            st.append("speed unknown: buzzer OFF")
        if sym == "?":
            st.append("radar/STM32 not trusted")
        if info.get("injected"):
            st.insert(0, "*** TEST INJECT level 2 ***")
        st.append(reason[:60])
        self._draw(sym, " | ".join(st))

    def safe_state(self, why):
        self._draw("-", f"OFF (safe state): {why}"[:78])

    def close(self):
        try:
            self.out.write(RESET + "\n"); self.out.flush()
        except Exception:
            pass
