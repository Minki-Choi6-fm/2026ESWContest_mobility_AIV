#!/usr/bin/env python3
"""레이더 거리 보정용 측정 프로그램 (Pi5에서 실행).

하는 일:
  1) STM32 디버그 포트(/dev/ttyACM0)에서 'OBJ id=.. dist=..m lat=..m vrel=.. rcs=..' 줄을 N초 동안 읽는다.
  2) 강한 반사(rcs 기준)이고 정면(lat 기준)이고 정지한(vrel 기준) 물체만 골라 거리의 중앙값을 낸다.
  3) 줄자로 잰 진짜 거리(--true)와 함께 ~/radar_calib.csv에 한 줄 쌓는다.
  4) 서로 다른 거리가 2개 이상 쌓이면 "측정값 = 기울기 * 진짜거리 + 오프셋"을 계산해서 보여준다.

사용법:
  python3 radar_range_cal.py --true 5.00            # 줄자로 5.00 m 잰 자리에서 10초 측정
  python3 radar_range_cal.py --true 3.00 --secs 15
  python3 radar_range_cal.py --report               # 쌓인 값으로 보정식만 다시 계산
  python3 radar_range_cal.py --file ~/obj_wall.txt --true 5.00 --no-save   # 저장된 로그로 연습

주의:
  - 한 번에 한 프로그램만 /dev/ttyACM0를 읽을 수 있다 (cat 등은 먼저 끝낼 것).
  - 레이더는 거리를 0.2 m 단위로 보고하므로 0.2 m보다 작은 오차는 측정할 수 없다.
  - 이 프로그램은 파일을 읽고 CSV에 한 줄 추가할 뿐, 레이더/STM32 설정은 바꾸지 않는다.
"""
import argparse
import csv
import os
import re
import statistics as st
import sys
import time

OBJ_RE = re.compile(r"OBJ id=(\d+) dist=(-?[\d.]+)m lat=(-?[\d.]+)m vrel=(-?[\d.]+)m/s rcs=(-?[\d.]+)")
CSV_PATH = os.path.expanduser("~/radar_calib.csv")


def parse_line(line):
    m = OBJ_RE.search(line)
    if not m:
        return None
    return dict(id=int(m[1]), dist=float(m[2]), lat=float(m[3]), vrel=float(m[4]), rcs=float(m[5]))


def read_serial(port, secs):
    import serial  # pyserial
    out = []
    ser = serial.Serial(port, 115200, timeout=0.5)
    t_end = time.time() + secs
    buf = b""
    try:
        while time.time() < t_end:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                out.append(line.decode("ascii", errors="ignore"))
    finally:
        ser.close()
    return out


def read_file(path):
    with open(os.path.expanduser(path), encoding="utf-8", errors="ignore") as f:
        return f.read().splitlines()


BASE_PATH = os.path.expanduser("~/radar_baseline.json")


def save_baseline(objs, a):
    """빈 장면: 0.2 m 구간마다 '전체 OBJ 줄 중 몇 %가 그 구간이었나'를 저장."""
    import json
    cnt = {}
    for o in objs:
        if o["rcs"] >= a.rcs_min and abs(o["lat"]) <= a.lat_max and abs(o["vrel"]) <= a.vrel_max and o["dist"] > 0:
            k = round(o["dist"] / 0.2)
            cnt[k] = cnt.get(k, 0) + 1
    frac = {str(k): v / len(objs) for k, v in cnt.items()}
    with open(BASE_PATH, "w") as f:
        json.dump(frac, f)
    print(f"[저장] 빈 장면 기록 {BASE_PATH} (OBJ {len(objs)}줄 기준)")
    print("[빈 장면에서 보인 강한 정지 물체]  (거리 / 전체 줄 중 비율)")
    for k, v in sorted(cnt.items(), key=lambda kv: -kv[1])[:8]:
        print(f"   {k * 0.2:5.1f} m  {100.0 * v / len(objs):4.1f}%")


def load_baseline():
    import json
    if not os.path.exists(BASE_PATH):
        return {}
    try:
        return {int(k): float(v) for k, v in json.load(open(BASE_PATH)).items()}
    except (ValueError, OSError):
        return {}


def fit_line(xs, ys):
    """최소제곱 직선 y = a*x + b."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return a, my - a * mx


def report():
    if not os.path.exists(CSV_PATH):
        print("아직 저장된 측정이 없습니다 (~/radar_calib.csv).")
        return
    rows = []
    with open(CSV_PATH, newline="") as f:
        for r in csv.DictReader(f):
            rows.append((float(r["true_m"]), float(r["radar_median_m"]), int(r["n"])))
    print("\n[저장된 측정]  진짜거리(줄자) -> 레이더 중앙값 | 차이(레이더-진짜) | 표본수")
    for t, m, n in sorted(rows):
        print(f"  {t:6.2f} m -> {m:6.2f} m | {m - t:+5.2f} m | n={n}")
    distinct = sorted(set(t for t, _, _ in rows))
    if len(distinct) < 2:
        print("\n보정식을 만들려면 서로 다른 거리 2곳 이상(가능하면 3곳 이상) 측정해 주세요.")
        return
    fit = fit_line([t for t, _, _ in rows], [m for _, m, _ in rows])
    a, b = fit
    print(f"\n[보정식] 레이더값 = {a:.3f} x 진짜거리 + {b:+.2f} m")
    print(f"        진짜거리 = (레이더값 - ({b:+.2f})) / {a:.3f}")
    print("        (기울기가 1에 가깝고 오프셋만 크면 '고정 오프셋', 기울기가 1에서 멀면 '거리에 따라 커지는 오차'입니다.)")
    print("        측정 점이 2~3개뿐이면 참고용입니다. 0.2 m 단위 해상도 때문에 +-0.1 m 정도는 오차로 봐야 합니다.")


def main():
    ap = argparse.ArgumentParser(description="레이더 거리 보정 측정 (읽기 전용 + CSV 한 줄 추가)")
    ap.add_argument("--true", type=float, help="줄자로 잰 진짜 거리(m): 레이더 앞면에서 반사판/벽까지")
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--file", default=None, help="시리얼 대신 저장된 로그 파일을 분석")
    ap.add_argument("--rcs-min", type=float, default=5.0, help="이 값 이상으로 강하게 반사된 물체만 사용")
    ap.add_argument("--lat-max", type=float, default=1.5, help="정면에서 좌우로 이만큼(m) 이내만 사용")
    ap.add_argument("--vrel-max", type=float, default=0.5, help="상대속도 절댓값이 이 값(m/s) 이하(정지)만 사용")
    ap.add_argument("--win", type=float, default=1.0,
                    help="줄자 거리에서 +-이 값(m) 안에 있는 물체만 사용 (뒤쪽 벽 등 다른 물체가 섞이는 것을 막음)")
    ap.add_argument("--baseline", action="store_true",
                    help="금속판을 치운 '빈 장면'을 기록해 둔다 (~/radar_baseline.json). --true 없이도 실행 가능")
    ap.add_argument("--no-baseline", action="store_true", help="빈 장면 기록이 있어도 쓰지 않는다")
    ap.add_argument("--report", action="store_true", help="측정 없이 보정식만 계산")
    ap.add_argument("--no-save", action="store_true", help="CSV에 저장하지 않음(연습용)")
    a = ap.parse_args()

    if a.report:
        report()
        return
    if a.true is None and not a.baseline:
        sys.exit("--true (줄자로 잰 거리, m)를 적어 주세요. 예: --true 5.00")
    if a.true is None:
        a.true = 0.0

    lines = read_file(a.file) if a.file else read_serial(a.port, a.secs)
    objs = [o for o in (parse_line(l) for l in lines) if o]
    print(f"[INFO] 읽은 OBJ 줄 {len(objs)}개 (전체 줄 {len(lines)}개)")
    if not objs:
        sys.exit("OBJ 줄이 없습니다. 레이더 연결/전원, STM32 로그 포트를 확인해 주세요.")

    if a.baseline:
        save_baseline(objs, a)
        return

    sel = [o for o in objs if o["rcs"] >= a.rcs_min and abs(o["lat"]) <= a.lat_max and abs(o["vrel"]) <= a.vrel_max
           and o["dist"] > 0 and abs(o["dist"] - a.true) <= a.win]
    print(f"[INFO] 조건(rcs>={a.rcs_min}, |lat|<={a.lat_max}, |vrel|<={a.vrel_max}, 줄자거리 +-{a.win} m 이내) 통과 {len(sel)}개")
    if len(sel) < 5:
        print("조건을 통과한 표본이 너무 적습니다. 반사판을 정면에 두거나 --rcs-min을 낮춰 보세요.")
        by_id = {}
        for o in objs:
            by_id.setdefault(o["id"], []).append(o)
        print("[참고] 이번에 보인 물체 요약 (id: 횟수, 거리 중앙값, lat 중앙값, rcs 중앙값)")
        for k, v in sorted(by_id.items()):
            print(f"   id {k}: n={len(v)}, dist={st.median(x['dist'] for x in v):.1f}, lat={st.median(x['lat'] for x in v):.1f}, rcs={st.median(x['rcs'] for x in v):.1f}")
        sys.exit(1)

    # 여러 물체가 섞였을 수 있어서 0.2 m 구간별로 센다.
    # 빈 장면(금속판 없음) 기준 파일이 있으면, 그 장면에도 있던 고정 물체(책상, 벽 등)를 빼고
    # "금속판을 놓았더니 새로 늘어난 구간"을 대표로 삼는다. 없으면 가장 많이 보인 구간을 쓴다.
    bins = {}
    for o in sel:
        bins.setdefault(round(o["dist"] / 0.2), []).append(o["dist"])
    base = load_baseline() if not a.no_baseline else {}
    pick = None
    if base:
        best_score = 0.0
        for k, v in bins.items():
            nb = max(base.get(k + dk, 0.0) for dk in (-1, 0, 1))
            score = len(v) / len(objs) - nb
            if score > best_score:
                best_score, pick = score, k
        if pick is None:
            print("[주의] 빈 장면에 없던 새 물체를 못 찾았습니다. 금속판이 레이더에 안 잡혔을 수 있습니다. 가장 많이 보인 구간으로 대신합니다.")
        else:
            print(f"[INFO] 빈 장면(baseline)을 빼고 '새로 생긴 물체'를 골랐습니다 (구간 {pick * 0.2:.1f} m)")
    if pick is None:
        pick = max(bins, key=lambda k: len(bins[k]))
    best = [d for dk in (-1, 0, 1) for d in bins.get(pick + dk, [])]
    med = st.median(best)
    sd = st.pstdev(best) if len(best) > 1 else 0.0
    print(f"[결과] 대표 거리 중앙값 = {med:.2f} m (표본 {len(best)}개, 표준편차 {sd:.2f} m, 범위 {min(best):.1f}~{max(best):.1f} m)")
    print(f"       줄자 거리 {a.true:.2f} m 와의 차이(레이더-진짜) = {med - a.true:+.2f} m")
    other = len(sel) - len(best)
    if other > len(best) * 0.5:
        print(f"[주의] 다른 거리에서도 강한 물체가 {other}개 더 있었습니다. 뒤쪽 벽이나 주변 금속이 섞였을 수 있으니, 배경을 비우고 다시 측정하는 것을 권합니다.")

    if not a.no_save:
        new = not os.path.exists(CSV_PATH)
        with open(CSV_PATH, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["true_m", "radar_median_m", "n", "time"])
            w.writerow([f"{a.true:.2f}", f"{med:.2f}", len(best), time.strftime("%Y-%m-%d %H:%M:%S")])
        print(f"[저장] {CSV_PATH} 에 한 줄 추가")
    report()


if __name__ == "__main__":
    main()
