"""S12-d: OutputSink that sends the own-car danger level to the STM32 as a 0x20 DISPLAY_WARNING frame (for the STM32 OLED).

!! The firmware on the board (v18c/v18d2 receive part, main_v18c.c rpi_rx_step) only INTERPRETS 0x22. A well-formed 0x20 is checked
!! (STX/LEN/XOR/ETX) and then DISCARDED: no OLED text, no buzzer, no timeout. So with the current board this sink changes nothing on the
!! OLED. It exists so the frame, the resend rhythm and the safe-off behaviour are ready and tested against a fake pty. [확인 필요 on the real board]

Frame (UART v0.2, same framing as every other frame): STX 02 | 0x20 | LEN 4 | LEVEL u8 | SRC u8 | DIST_dm u16 LE | XOR(TYPE,LEN,payload) | ETX 03.
Payload order/units follow the front-car sender (front_logic.encode_v2v_warn: level, src_vid, dist in 0.1 m, 0xFFFF = unknown). PROTOCOLS.md lists
"SENDER_ID, DANGER_LEVEL, DISTANCE cm" (different order and unit) -> the layout is NOT agreed [PROPOSAL / 확인 필요 with the STM32 owner].
SRC = 0 means "own radar" (front-car V2V warnings use the other car's vehicle id). LEVEL: 0 off/safe, 1 caution, 2 danger, 0xFF unknown ("?", never a buzzer).

Rules implemented:
* exactly ONE os.write() per frame (so a concurrent writer such as ego_speed_tx.py cannot interleave inside a frame; see docs/handoff/CAR_TEST_RUNBOOK.md),
  O_WRONLY|O_NOCTTY|O_NONBLOCK, termios is never touched; a short write or EAGAIN drops the frame (counted), never retried in a loop.
* sent at every level change, re-sent every RESEND_S while the level is not 0, keep-alive every IDLE_S while 0 (the STM32 could use that for its own timeout later).
* safe_state() (hub watchdog / crash / SIGTERM / exit) sends level 0 (OFF) twice. A SIGKILL or power loss cannot send anything: only an STM32-side timeout could
  clear the display then, and the current firmware has none.
Nothing here runs at import time. The default of pipeline_main is OFF (flag --write-stm32).
"""
import os
import struct
import threading
import time

TYPE_DISPLAY_WARNING = 0x20
RESEND_S = 0.2          # [PROPOSAL] while level != 0 (the STM32 timeout, once it exists, should be a few times this, e.g. 0.6-1.0 s)
IDLE_S = 1.0            # [PROPOSAL] keep-alive at level 0


def build_warn_frame(level, src=0, dist_m=None):
    dist_dm = 0xFFFF if dist_m is None else max(0, min(0xFFFE, int(round(dist_m * 10))))
    pl = struct.pack("<BBH", level & 0xFF, src & 0xFF, dist_dm)
    x = TYPE_DISPLAY_WARNING ^ len(pl)
    for b in pl:
        x ^= b
    return bytes((0x02, TYPE_DISPLAY_WARNING, len(pl))) + pl + bytes((x, 0x03))


class Stm32WarnSink:
    """OutputSink-compatible: update(now, level, reason, info) / safe_state(why) / close()."""

    def __init__(self, path, clock=time.monotonic):
        self.path, self.clock = path, clock
        self.fd = os.open(path, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)      # no termios call: the port is configured by whoever owns it
        self.lock = threading.Lock()
        self.level, self.t_tx = None, None
        self.sent = self.dropped = self.short = 0
        self.closed = False

    def _write(self, frame):
        with self.lock:
            if self.closed:
                return False
            try:
                n = os.write(self.fd, frame)          # ONE write() per frame
            except (BlockingIOError, InterruptedError):
                self.dropped += 1; return False
            except OSError:
                self.dropped += 1; return False
            if n != len(frame):
                self.short += 1; return False
            self.sent += 1; self.t_tx = self.clock(); return True

    def update(self, now, level, reason, info):
        lv = 0 if level is None else int(level)
        dist = info.get("target_dist") if info else None
        t = self.clock()
        due = (self.level != lv or self.t_tx is None or
               (t - self.t_tx) >= (IDLE_S if lv == 0 else RESEND_S))
        if due:
            self._write(build_warn_frame(lv, 0, dist if lv in (1, 2) else None))
            self.level = lv

    def safe_state(self, why):
        for _ in range(2):
            self._write(build_warn_frame(0, 0, None))
        self.level = 0

    def close(self):
        self.safe_state("close")
        with self.lock:
            if not self.closed:
                self.closed = True
                try:
                    os.close(self.fd)
                except OSError:
                    pass
