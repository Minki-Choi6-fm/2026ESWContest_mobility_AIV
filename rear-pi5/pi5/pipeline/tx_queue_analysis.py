#!/usr/bin/env python3
"""S5-c: test the hypothesis "STM32 TX ring (512 B) overflows -> proto_send() drops frames" on recorded drives. Offline, read-only.

Code facts (firmware/stm32_rear/main_v18c.c): proto_send() drops a frame when free ring space < frame size and only increments tx_drop_count
(NOT printed anywhere); rad_npub++ ("pub") is incremented right before radar_publish() -> proto_send(), so pub counts frames HANDED to the queue.
Two independent checks on the logs:
 (1) pub (RADSTAT, per second, stm_debug.log) vs 0x12 frames actually received in the same STM32-time window  => direct estimate of 0x12 lost
     after the queue (queue drops + anything the logger rejected).
 (2) model of the ring driven by the received frames in transmission order, copying the firmware exactly: 511 usable bytes, the UART drains
     contiguous CHUNKS at 11.52 B/ms (115200 8N1), and tx_tail only moves when a whole chunk has finished (HAL_UART_TxCpltCallback), so the chunk in
     flight still counts as "used". Occupancy ("used") is compared at each MISSING 0x10 slot vs ordinary 0x10 slots. A 16 B frame is dropped when used > 495.
Frame sizes on the wire: 0x10 16, 0x11 11, 0x12 19, 0x13 28, 0x14 10 bytes.
"""
import bisect, collections, csv, json, os, re, statistics as S, struct, sys

SIZE = {0x10: 16, 0x11: 11, 0x12: 19, 0x13: 28, 0x14: 10}
RING, RATE = 511, 11.52     # usable bytes, bytes per ms


def load(d):
    fr = []                 # (ts_ms or None, typ, host_t) in stream order
    for r in list(csv.reader(open(os.path.join(d, "frames.csv"))))[1:]:
        typ = int(r[1], 16); pl = bytes.fromhex(r[2])
        ts = struct.unpack_from("<I", pl)[0] if typ in (0x10, 0x11, 0x12, 0x13) else None
        fr.append((ts, typ, float(r[0])))
    return fr


def radstat(d):
    out = []
    for line in open(os.path.join(d, "stm_debug.log"), errors="replace"):
        if " RADSTAT " in line:
            m = dict(re.findall(r"(\w+)=(-?\d+)", line))
            out.append((int(m["t"]), int(m["pub"]), int(m["b"]), int(m["veh"])))
    return out


def epochs(fr):
    """Split the stream where the STM32 clock restarts (reset). Returns list of lists of (ts, typ, host_t)."""
    out, cur, tmax = [], [], None
    for ts, typ, h in fr:
        if ts is not None:
            if tmax is not None and ts < tmax - 1000:
                out.append(cur); cur, tmax = [], None
            tmax = ts if tmax is None else max(tmax, ts)
        cur.append((ts, typ, h))
    out.append(cur)
    return out


def check_pub(d, fr):
    rs_all = radstat(d)
    rs_ep, cur = [], []
    for r in rs_all:
        if cur and r[0] < cur[-1][0] - 1000:
            rs_ep.append(cur); cur = []
        cur.append(r)
    rs_ep.append(cur)
    rows = []
    for ep, rs in zip(epochs(fr), rs_ep):
        ts12 = sorted(ts for ts, typ, _ in ep if typ == 0x12)
        for (t0, _, _, _), (t1, pub, b, veh) in zip(rs, rs[1:]):
            if not (900 <= t1 - t0 <= 1100):
                continue
            got = bisect.bisect_right(ts12, t1) - bisect.bisect_right(ts12, t0)
            rows.append((t1, pub, got))
    return rows


class Ring:
    """Firmware TX ring: head/tail in absolute byte counts, chunk-wise release (see module docstring)."""
    def __init__(self):
        self.head = self.tail = 0
        self.busy, self.chunk, self.end = False, 0, 0.0
        self.overflow_model = 0

    def _kick(self, t):
        if self.busy or self.head == self.tail:
            return
        ph, pt = self.head % 512, self.tail % 512
        self.chunk = (ph - pt) if ph > pt else (512 - pt)
        self.busy, self.end = True, t + self.chunk / RATE

    def advance(self, t):
        while self.busy and self.end <= t:
            self.tail += self.chunk
            tend = self.end
            self.busy = False
            self._kick(tend)

    def used(self, t):
        self.advance(t)
        return self.head - self.tail

    def put(self, t, size):
        self.advance(t)
        if RING - (self.head - self.tail) < size:
            self.overflow_model += 1              # the real firmware accepted this frame, so the model drains slower than reality
            return
        self.head += size
        self._kick(t)


def ring_fill_at_slots(ep, queries):
    """ep: frames of one epoch. queries: sorted list of times. Returns {query_time: used bytes just before it}."""
    ring, out = Ring(), {}
    qi, tmax = 0, None
    for ts, typ, _ in ep:
        if ts is None:
            ts = tmax
        if ts is None:
            continue
        tmax = ts if tmax is None else max(tmax, ts)
        while qi < len(queries) and queries[qi] <= tmax:
            out[queries[qi]] = ring.used(queries[qi]); qi += 1
        ring.put(tmax, SIZE[typ])
    return out, ring.overflow_model


def analyse(d):
    name = os.path.basename(d.rstrip("/"))
    fr = load(d)
    print(f"\n## {name}")
    # per-cycle radar burst
    cyc = collections.Counter(ts for ts, typ, _ in fr if typ == 0x12)
    n = sorted(cyc.values())
    big = sum(1 for v in n if v * 19 > RING)
    print(f"radar cycles (same STM32 ts): {len(n)}; objects/cycle mean {S.mean(n):.1f}, p95 {n[int(.95 * len(n))]}, max {n[-1]} -> max {n[-1] * 19} B queued back-to-back "
          f"({'> ' if n[-1] * 19 > RING else '<= '}{RING} B ring); cycles whose 0x12 alone exceed the ring: {big}; 26 objects = 494 B is the most one empty ring can take")
    # check 1
    rows = check_pub(d, fr)
    pub = sum(r[1] for r in rows); got = sum(r[2] for r in rows)
    bad = [r for r in rows if r[1] != r[2]]
    print(f"(1) pub vs received 0x12 over {len(rows)} one-second windows: published {pub}, received {got} -> missing {pub - got} ({100 * (pub - got) / max(1, pub):.2f} %); "
          f"windows with a difference: {len(bad)} ({100 * len(bad) / max(1, len(rows)):.0f} %); received > pub in {sum(1 for r in rows if r[2] > r[1])} windows (window edge effect)")
    # check 3: offered load per second vs the line capacity (11,520 B/s at 115200 8N1)
    OTHER_BPS = 50 * 16 + 10 * 11 + 20 * 28 + 3 * 10          # 0x10 + 0x11 + 0x13 + 0x14 at their nominal rates = 1,500 B/s
    cap_pub = (11520 - OTHER_BPS) / 19.0                       # max 0x12 per second the line can carry next to the others (~527)
    over = [r for r in rows if r[1] > cap_pub]
    miss_all = sum(max(0, r[1] - r[2]) for r in rows); miss_over = sum(max(0, r[1] - r[2]) for r in over)
    print(f"(3) offered 0x12 load: mean {sum(r[1] for r in rows) / max(1, len(rows)):.0f}/s, p95 {sorted(r[1] for r in rows)[int(.95 * len(rows))] if rows else 0}/s, max {max((r[1] for r in rows), default=0)}/s; "
          f"the line carries at most ~{cap_pub:.0f} 0x12/s next to the other frames (11,520 B/s - {OTHER_BPS} B/s) / 19 B")
    print(f"    seconds ABOVE that capacity: {len(over)}/{len(rows)} ({100 * len(over) / max(1, len(rows)):.1f} %); they contain {100 * miss_over / max(1, miss_all):.0f} % of all missing 0x12 "
          f"({miss_over} of {miss_all}); in those seconds {100 * sum(r[2] for r in over) / max(1, sum(r[1] for r in over)):.0f} % of the published 0x12 arrived")
    # check 2 (per epoch)
    gap_fill, base_fill, model_over = [], [], 0
    for ep in epochs(fr):
        last, gaps, bases = None, [], []
        for ts, typ, _ in ep:
            if typ != 0x10:
                continue
            if last is not None and ts > last:
                miss = max(0, round((ts - last) / 20) - 1)
                if miss:
                    gaps += [last + 20 * k for k in range(1, miss + 1)]
                elif len(bases) < 15000:
                    bases.append(ts - 0.001)
            last = ts
        qs = sorted(set(gaps) | set(bases))
        res, mo = ring_fill_at_slots(ep, qs)
        model_over += mo
        gap_fill += [res[t] for t in gaps if t in res]; base_fill += [res[t] for t in bases if t in res]
    q = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))] if v else float("nan")
    full = RING - 16
    print(f"(2) ring 'used' bytes just before a 0x10 slot (chunk-release model): ordinary slots median {q(base_fill, .5):.0f} p95 {q(base_fill, .95):.0f} max {max(base_fill):.0f}; "
          f"MISSING slots median {q(gap_fill, .5):.0f} p95 {q(gap_fill, .95):.0f} max {max(gap_fill) if gap_fill else float('nan'):.0f} (n={len(gap_fill)}); a 16 B frame is dropped when used > {full}; "
          f"frames the model could not fit although the firmware accepted them: {model_over}")
    f_gap = sum(1 for f in gap_fill if f > full); f_base = sum(1 for f in base_fill if f > full)
    near_gap = sum(1 for f in gap_fill if f > full - 100); near_base = sum(1 for f in base_fill if f > full - 100)
    print(f"    MISSING slots with used > {full} (drop would happen): {f_gap}/{len(gap_fill)} ({100 * f_gap / max(1, len(gap_fill)):.0f} %); ordinary slots {f_base}/{len(base_fill)} ({100 * f_base / max(1, len(base_fill)):.1f} %); "
          f"within 100 B of that: missing {100 * near_gap / max(1, len(gap_fill)):.0f} % vs ordinary {100 * near_base / max(1, len(base_fill)):.1f} %")
    # exact-multiple signature
    dts = collections.Counter()
    last = None
    for ts, typ, _ in fr:
        if typ == 0x10:
            if last is not None and ts > last:
                dts[ts - last] += 1
            last = ts
    print(f"    0x10 interval signature: 20-24 ms {sum(dts[k] for k in range(20, 25))}, 40 ms {dts[40]}, 41-44 ms {sum(dts[k] for k in range(41, 45))}, 60 ms {dts[60]}, 61-64 ms {sum(dts[k] for k in range(61, 65))}, "
          f"25-39 ms {sum(dts[k] for k in range(25, 40))}, 45-59 ms {sum(dts[k] for k in range(45, 60))}")


if __name__ == "__main__":
    for d in sys.argv[1:]:
        analyse(d)
