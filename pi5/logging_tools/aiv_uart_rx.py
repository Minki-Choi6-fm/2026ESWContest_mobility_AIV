#!/usr/bin/env python3
"""
STM32 -> RPi UART 프레임 수신/검증 도구  (규격: AIV_STM32-RPi_UART_메시지규격_v0.2)

사용법 (RPi5에서):
    ls /dev/ttyAMA*                      # UART 장치 이름 확인
    python3 aiv_uart_rx.py /dev/ttyAMA2  # 장치 이름은 환경에 맞게
    python3 aiv_uart_rx.py --file dump.bin   # 저장된 바이트 파일로 파서만 시험

프레임: STX(02) TYPE LEN PAYLOAD XOR(TYPE^LEN^PAYLOAD) ETX(03), 리틀엔디안
검증 실패 프레임은 즉시 버리고(재요청 없음) STX 다음 바이트부터 다시 동기화한다.
"""
import argparse
import struct
import sys
import time

STX, ETX = 0x02, 0x03
# TYPE -> (이름, 고정 LEN)
MSG = {
    0x10: ("SENSOR_SUMMARY", 11),
    0x11: ("HEARTBEAT", 6),
    0x12: ("RADAR_OBJECT", 14),
    0x13: ("IMU_DATA", 23),
    0x14: ("OBD_DATA", 5),
}


class FrameParser:
    """바이트를 넣으면 검증된 (type, payload) 목록을 돌려준다. 절대 기다리지 않는다."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0  # 검증 실패로 버린 프레임 수

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([STX]))
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]           # STX 앞의 잡음 버림
            if len(self.buf) < 3:
                break                       # TYPE, LEN이 아직 안 옴
            t, ln = self.buf[1], self.buf[2]
            if t not in MSG or MSG[t][1] != ln:
                self.bad += 1
                del self.buf[0]             # 이 STX는 가짜 -> 다음 STX 탐색
                continue
            total = ln + 5
            if len(self.buf) < total:
                break                       # 프레임이 덜 왔음: 기다리지 않고 다음 입력 때 이어서
            payload = bytes(self.buf[3:3 + ln])
            chk, etx = self.buf[3 + ln], self.buf[4 + ln]
            x = t ^ ln
            for b in payload:
                x ^= b
            if etx != ETX or x != chk:
                self.bad += 1
                del self.buf[0]             # 검증 실패: 폐기, 재시도 없음
                continue
            out.append((t, payload))
            del self.buf[:total]
        return out


def sb(v):  # 부호 없는 바이트 -> 부호 있는 int8
    return v - 256 if v > 127 else v


def decode(t, p):
    if t == 0x10:
        ts, k, dist, rel, danger, st = struct.unpack("<IBHhBB", p)
        return dict(ts=ts, K=None if k == 0xFF else k / 100,
                    dist_m=None if dist == 0xFFFF else dist / 100,
                    rel_ms=None if rel == 0x7FFF else rel / 100,
                    danger=None if danger == 0xFF else danger, status=f"0x{st:02X}")
    if t == 0x11:
        up, st, rc = struct.unpack("<IBB", p)
        return dict(uptime_ms=up, status=f"0x{st:02X}", reset={0: "power", 1: "iwdg"}.get(rc, "other"))
    if t == 0x12:
        ts, oid, dl, dt, vr, rcs, dyn, cyc = struct.unpack("<IBhhhBBB", p)
        return dict(ts=ts, id=oid, long_m=dl / 100, lat_m=dt / 100, vrel_ms=vr / 100,
                    rcs_dbsm=sb(rcs) / 2, dyn=dyn, cycle=None if cyc == 0xFF else (cyc >> 4, cyc & 0xF))
    if t == 0x13:
        v = struct.unpack("<Ihhhhhhhhh", p[:22])
        fl = p[22]
        return dict(ts=v[0], acc_mg=v[1:4], gyro_dps=tuple(x / 10 for x in v[4:7]),
                    rpy_deg=tuple(x / 10 for x in v[7:10]), imu_valid=bool(fl & 1), brake=bool(fl & 2))
    if t == 0x14:
        spd, rpm, age = struct.unpack("<BHH", p)
        return dict(speed_kmh=None if spd == 0xFF else spd, rpm=None if rpm == 0xFFFF else rpm,
                    age_ms=None if age == 0xFFFF else age)
    return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="예: /dev/ttyAMA2")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="시리얼 대신 바이트 파일을 읽어 시험")
    ap.add_argument("--show", default="0x10,0x11,0x12,0x13,0x14",
                    help="화면에 출력할 TYPE 목록 (기본: 전부, 예: --show 0x12)")
    a = ap.parse_args()
    show = {int(x, 16) for x in a.show.split(",")}

    parser = FrameParser()
    counts = {t: 0 for t in MSG}
    last_report = time.time()

    def handle(frames):
        for t, p in frames:
            counts[t] += 1
            if t in show:
                print(f"{MSG[t][0]:15s} {decode(t, p)}")

    if a.file:
        handle(parser.feed(open(a.file, "rb").read()))
        print("counts:", {MSG[t][0]: c for t, c in counts.items()}, "bad:", parser.bad)
        return
    if not a.port:
        ap.error("포트 또는 --file 이 필요합니다")

    import serial  # pip install pyserial
    ser = serial.Serial(a.port, a.baud, timeout=0.05)
    print(f"{a.port} @ {a.baud} 수신 시작 (Ctrl+C 종료)")
    try:
        while True:
            data = ser.read(256)
            if data:
                handle(parser.feed(data))
            now = time.time()
            if now - last_report >= 5:
                dt = now - last_report
                rates = {MSG[t][0]: round(c / dt, 1) for t, c in counts.items()}
                print(f"--- 최근 5초 수신율(Hz): {rates}  검증실패 누적: {parser.bad}")
                counts = {t: 0 for t in MSG}
                last_report = now
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
