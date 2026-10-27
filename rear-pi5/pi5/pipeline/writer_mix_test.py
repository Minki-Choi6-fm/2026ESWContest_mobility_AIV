#!/usr/bin/env python3
"""S12-a: do frames from TWO writer processes on the same tty stay intact when each frame is ONE write() call?
Test bed = a NEW pty (never /dev/ttyAMA2). Process 1 mimics ego_speed_tx.py (0x22, 10-byte frames, 5 Hz in real life; here as fast as possible),
process 2 is the pipeline's Stm32WarnSink (0x20, 9-byte frames). A reader on the pty master checks every byte with the same framing rules as the STM32.
Backpressure drops are counted separately from byte interleaving. PASS only means: on a Linux pty, whole-frame write() calls were not interleaved in N frames. A real UART tty (amba-pl011 driver, /dev/ttyAMA2) uses the
same n_tty write path (atomic_write_lock per write() call) but this was NOT measured there -> [확인 필요]. usage: python3 -I writer_mix_test.py"""
import multiprocessing as mp, os, pty, struct, sys, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_ingest import build_frame
from stm32_warn_sink import Stm32WarnSink, build_warn_frame

N = 6000


def writer_ego(path, n):
    fd = os.open(path, os.O_WRONLY | os.O_NOCTTY)
    f = build_frame(0x22, struct.pack("<HBH", 1234, 1, 50))
    for _ in range(n):
        os.write(fd, f)


def writer_warn(path, n, result):
    s = Stm32WarnSink(path)
    for i in range(n):
        s._write(build_warn_frame(i % 3, 0, 12.3))        # the sink's own write path (one os.write per frame)
        time.sleep(0)
    s.close()
    result.send((s.sent, s.dropped, s.short))
    result.close()


def main():
    master, slave = pty.openpty(); tty.setraw(master); tty.setraw(slave); path = os.ttyname(slave)
    rx, tx = mp.Pipe(duplex=False)
    ps = [mp.Process(target=writer_ego, args=(path, N)), mp.Process(target=writer_warn, args=(path, N, tx))]
    for p in ps: p.start()
    buf = bytearray(); ok22 = ok20 = bad = 0; junk = 0
    import select
    deadline = time.time() + 30
    while time.time() < deadline:
        r, _, _ = select.select([master], [], [], 0.2)
        if r:
            buf += os.read(master, 65536)
        elif not any(p.is_alive() for p in ps):
            break
        while True:
            i = buf.find(b"\x02")
            if i < 0:
                junk += len(buf); buf.clear(); break
            if i: junk += i; del buf[:i]
            if len(buf) < 3: break
            n = buf[2]; tot = n + 5
            if len(buf) < tot: break
            x = 0
            for b in buf[1:n + 3]: x ^= b
            if buf[tot - 1] == 3 and x == buf[n + 3] and buf[1] in (0x20, 0x22) and n == {0x20: 4, 0x22: 5}[buf[1]]:
                if buf[1] == 0x22: ok22 += 1
                else: ok20 += 1
                del buf[:tot]
            else:
                bad += 1; del buf[0]
    for p in ps: p.join()
    print(f"0x22 frames ok {ok22}/{N}, 0x20 frames ok {ok20}/{N + 2} (incl. 2 OFF frames from close), bad/interleaved {bad}, junk bytes {junk}")
    sent, dropped, short = rx.recv()
    print(f"warning writer: sent={sent}, backpressure drops={dropped}, short={short}; drops are intentional for O_NONBLOCK")
    good = (all(p.exitcode == 0 for p in ps) and ok22 == N and ok20 == sent
            and sent + dropped == N + 2 and short == 0 and bad == 0 and junk == 0 and not buf)
    print("PASS" if good else "FAIL", "(pty only; real tty = [확인 필요])")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
