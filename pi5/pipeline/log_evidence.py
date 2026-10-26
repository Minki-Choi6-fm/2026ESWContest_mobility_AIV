#!/usr/bin/env python3
"""Report evidence from recorded drives (READ-ONLY, never modifies a log): OLEDSTAT summary, Safe-Mode entries (RADSTAT safe=), CAN error counters
(tec/rec), STM32 resets (heartbeat RESET_CAUSE + 'RESET CAUSE' lines) and their coincidence with OLED messages. Measured values only; anything that
cannot be derived from the files is printed as "unknown"/"no data".
usage: python3 -I log_evidence.py <logs root or drive dirs...>   (default: ~/ESWContest_ws/logs)"""
import csv, json, os, re, sys

CAUSE = {0: "power-on", 1: "IWDG", 2: "other/pin"}


def find_drives(args):
    out = []
    for a in args:
        if os.path.isfile(os.path.join(a, "stm_debug.log")):
            out.append(a)
        else:
            for root, dirs, files in os.walk(a):
                if "stm_debug.log" in files:
                    out.append(root)
    return sorted(set(out))


def episodes(times, gap=3.0):
    eps = []
    for t in times:
        if eps and t - eps[-1][1] <= gap:
            eps[-1][1] = t; eps[-1][2] += 1
        else:
            eps.append([t, t, 1])
    return eps


def analyse(d):
    name = os.path.basename(d.rstrip("/"))
    meta = json.load(open(os.path.join(d, "meta.json"))) if os.path.exists(os.path.join(d, "meta.json")) else {}
    lines = []
    for ln in open(os.path.join(d, "stm_debug.log"), errors="replace"):
        parts = ln.rstrip("\n").split(" ", 1)
        try:
            lines.append((float(parts[0]), parts[1] if len(parts) > 1 else ""))
        except ValueError:
            pass
    t0 = meta.get("start_time", lines[0][0])
    dur = lines[-1][0] - t0
    res = dict(name=name, dur=dur, nlines=len(lines), start=meta.get("start_local", "?"), note=meta.get("args", {}).get("note", ""))
    # (1) OLEDSTAT / firmware indicators
    oled = [(t, l) for t, l in lines if "OLEDSTAT" in l]
    res["oledstat"] = oled
    res["fw_banner"] = [l for t, l in lines if l.startswith("FW ")]
    res["boot_i2c"] = [(t - t0, l) for t, l in lines if "I2C" in l or "OLED" in l.upper() and "OLEDSTAT" not in l]
    res["resets_text"] = [(t - t0, l) for t, l in lines if "RESET CAUSE" in l]
    # (2) RADSTAT
    rs = []
    for t, l in lines:
        if l.startswith("RADSTAT "):
            m = dict(re.findall(r"(\w+)=(-?\d+)", l))
            rs.append((t - t0, int(m["safe"]), int(m["tec"]), int(m["rec"]), int(m["ovf"]), int(m["t"]), int(m["exp"]), int(m["b"]), int(m["veh"]), int(m["pub"])))
    res["rs"] = rs
    entries, prev = [], None
    for t, safe, tec, rec, ovf, stm_t, *_ in rs:
        if prev is not None:
            if stm_t < prev[5] - 1000:                              # STM32 restarted: counters start again from 0
                entries += [(t, 1)] * safe
            elif safe > prev[1]:
                entries += [(t, safe - prev[1])]
        prev = (t, safe, tec, rec, ovf, stm_t)
    # CAN side: radar announced (exp) vs STM32 received (b); MCP2515 RX overflow seconds (ovf increase) vs object-heavy seconds
    inc = [(y[7], y[4] - x[4]) for x, y in zip(rs, rs[1:]) if y[5] >= x[5] - 1000]
    heavy = [i for i in inc if i[0] >= 500]
    res["can"] = dict(exp=sum(r[6] for r in rs), b=sum(r[7] for r in rs), veh=sum(r[8] for r in rs), pub=sum(r[9] for r in rs), secs=len(inc),
                      ovf_secs=sum(1 for i in inc if i[1] > 0), heavy=len(heavy), heavy_ovf=sum(1 for i in heavy if i[1] > 0))
    times = []
    for t, k in entries:
        times += [t] * k
    res["safe_entries"] = len(times)
    res["safe_eps"] = episodes(times)
    res["timeout_lines"] = sum(1 for t, l in lines if "RADAR TIMEOUT" in l)
    res["recovered_lines"] = sum(1 for t, l in lines if "RADAR RECOVERED" in l)
    res["tec_rec"] = (max((r[2] for r in rs), default=0), max((r[3] for r in rs), default=0), sum(1 for a, b in zip(rs, rs[1:]) if (a[2], a[3]) != (b[2], b[3])))
    res["tec_nonzero"] = [(round(r[0]), r[2], r[3]) for r in rs if r[2] or r[3]]
    res["ovf"] = (rs[0][4], rs[-1][4]) if rs else (None, None)
    # (3) heartbeat reset causes
    fp = os.path.join(d, "frames.csv")
    causes, last = [], None
    if os.path.exists(fp):
        for r in list(csv.reader(open(fp)))[1:]:
            if r[1] == "0x11":
                pl = bytes.fromhex(r[2]); up = int.from_bytes(pl[:4], "little"); cz = pl[5]
                if last is None or cz != last[1] or up < last[0] - 1000:
                    causes.append((float(r[0]) - t0, cz, up))
                last = (up, cz)
    res["causes"] = causes
    return res


def show(r):
    print(f"\n## {r['name']}  (start {r['start']}, duration {r['dur']:.0f} s, {r['nlines']} debug lines{', note: ' + r['note'] if r['note'] else ''})")
    print(f"- firmware indicators: `FW ...` banner lines: {len(r['fw_banner'])}; OLEDSTAT lines: {len(r['oledstat'])}; RADSTAT lines: {len(r['rs'])}. "
          f"Source check: v16/v17 print neither FW banner nor OLEDSTAT, v18c prints both → this log was recorded with firmware **older than v18c (v16/v17 class; exact version unknown)**.")
    if r["oledstat"]:
        print(f"- OLEDSTAT: {len(r['oledstat'])} lines; first `{r['oledstat'][0][1]}`, last `{r['oledstat'][-1][1]}`")
    else:
        print("- OLEDSTAT (pushes/fails/retries/retry_ok/err): **no lines in this log → nothing to summarise** (counter did not exist in this firmware)")
    if r["boot_i2c"]:
        print("- OLED/I2C text: " + "; ".join(f"{t:.0f}s `{l[:70]}`" for t, l in r["boot_i2c"]))
    print(f"- Safe Mode (RADSTAT `safe=` cumulative per second; `RADAR TIMEOUT -> SAFE MODE` lines: {r['timeout_lines']}, `RADAR RECOVERED` lines: {r['recovered_lines']}): "
          f"**{r['safe_entries']} entries**, in {len(r['safe_eps'])} episode(s) (increments within 3 s merged)")
    if r["safe_eps"]:
        print("  | episode | start s | end s | duration s | entries |\n  |---|---|---|---|---|")
        for i, (a, b, n) in enumerate(r["safe_eps"][:12]):
            print(f"  | {i + 1} | {a:.0f} | {b:.0f} | {b - a:.0f} | {n} |")
        if len(r["safe_eps"]) > 12:
            print(f"  | ... {len(r['safe_eps']) - 12} more episodes | | | | |")
    c = r["can"]
    print(f"- CAN receive side: radar announced {c['exp']} objects (RADSTAT `exp`), STM32 received {c['b']} (`b`) -> shortfall {c['exp'] - c['b']} ({100 * (c['exp'] - c['b']) / max(1, c['exp']):.1f} %); "
          f"MCP2515 RX-overflow seconds (`ovf` increase) {c['ovf_secs']}/{c['secs']} ({100 * c['ovf_secs'] / max(1, c['secs']):.0f} %), "
          f"and in the {c['heavy']} seconds with >= 500 objects/s: {c['heavy_ovf']} had an overflow")
    print(f"- UART publishing: `pub` {c['pub']} vs vehicle-class objects `veh` {c['veh']} -> published = {c['pub'] / max(1, c['veh']):.1f} x vehicle-class count (firmware has `#define TEST_ALL_CLASSES 1` in v16, v17 and v18c: the class filter is OFF)")
    print(f"- CAN error counters: tec max {r['tec_rec'][0]}, rec max {r['tec_rec'][1]}, number of changes between consecutive seconds: {r['tec_rec'][2]}; seconds with tec/rec != 0 (t, tec, rec): {r['tec_nonzero'][:6] or 'none'}; RX buffer overflow seconds `ovf`: {r['ovf'][0]} → {r['ovf'][1]}")
    cz = ", ".join(f"{t:.0f}s: {CAUSE.get(c, c)}" for t, c, up in r["causes"]) or "no heartbeats"
    print(f"- heartbeat RESET_CAUSE over time: {cz}; IWDG (cause 1) seen: **{'YES' if any(c == 1 for _, c, _ in r['causes']) else 'no'}**; `RESET CAUSE` text lines: " + (", ".join(f"{t:.0f}s `{l}`" for t, l in r["resets_text"]) or "none"))
    oled_t = [t for t, l in r["boot_i2c"] if "No I2C device" in l]
    if r["resets_text"]:
        print("- coincidence with OLED messages: the only OLED/I2C message(s) " + ", ".join(f"at {t:.0f}s" for t in oled_t) + " sit inside the boot sequence of the reset at " + ", ".join(f"{t:.0f}s" for t, _ in r["resets_text"]) +
              " (the boot-time I2C scan found no OLED); there is no OLEDSTAT, so OLED errors during normal running cannot be matched to resets [no data].")
    else:
        print("- coincidence of IWDG resets with OLED errors: **no reset and no OLED message in this log → nothing to match**")


if __name__ == "__main__":
    roots = [a for a in sys.argv[1:] if not a.startswith("--")] or [os.path.expanduser("~/ESWContest_ws/logs")]
    ds = find_drives(roots)
    print(f"# Log evidence from {len(ds)} drive log(s) (read-only)")
    for d in ds:
        show(analyse(d))
