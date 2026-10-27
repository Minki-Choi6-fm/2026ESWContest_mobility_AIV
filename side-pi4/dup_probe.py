#!/usr/bin/env python3
"""
dup 원인 추적용 진단 스크립트

v2v_node.py(EspNowTransport)와 똑같은 방식으로 포트를 열고,
처음 N초 동안 들어오는 원시 바이트를 도착 시각과 함께 기록한 뒤 분석한다.

  python3 dup_probe.py --id 3                 # 노드와 동일 조건 (내 BSM도 10Hz 송신)
  python3 dup_probe.py --id 3 --no-send       # 수신만 (송신이 원인인지 비교용)
  python3 dup_probe.py --replay probe_xxx.bin # 저장된 파일 다시 분석

결과: probe_<시각>.bin (원시 바이트), probe_<시각>.log (도착 시각별 바이트 수)
"""
import argparse
import collections
import json
import threading
import time

from bsm_protocol import BSM, FRAME_LEN, SYNC, VEHICLE_NAMES, decode, encode, is_valid_frame


def split_frames(raw: bytes):
    """원시 스트림에서 (offset, frame) 목록과 프레임이 아닌 바이트 조각 목록을 뽑는다."""
    frames, junk = [], []
    i, junk_start = 0, 0
    while i <= len(raw) - FRAME_LEN:
        if raw[i] == SYNC and is_valid_frame(raw[i:i + FRAME_LEN]):
            if i > junk_start:
                junk.append((junk_start, raw[junk_start:i]))
            frames.append((i, raw[i:i + FRAME_LEN]))
            i += FRAME_LEN
            junk_start = i
        else:
            i += 1
    if junk_start < len(raw):
        junk.append((junk_start, raw[junk_start:]))
    return frames, junk


def analyze(raw: bytes, chunks):
    print(f'\n=== 분석: 총 {len(raw)} 바이트 ===')

    # 1) 도착 속도: 첫 수신 시점과 초기 폭주 여부
    if chunks:
        t0 = chunks[0][0]
        print(f'첫 바이트 도착: 포트 연 뒤 {t0 * 1000:.0f}ms, 첫 조각 크기 {chunks[0][1]}바이트')
        bins = collections.OrderedDict()
        for t, n in chunks:
            b = int(t * 10) / 10          # 100ms 구간
            bins[b] = bins.get(b, 0) + n
        print('100ms 구간별 수신 바이트 (115200bps 최대치는 구간당 약 1150바이트):')
        for b, n in list(bins.items())[:15]:
            flag = '  <-- 물리적으로 불가능한 속도 (어딘가 쌓여 있던 데이터)' if n > 1300 else ''
            print(f'  {b:4.1f}s  {n:6d}{flag}')

    # 2) 프레임 분석: 차량별 seq, 연속 중복
    frames, junk = split_frames(raw)
    print(f'\n유효 프레임 {len(frames)}개, 프레임 아닌 조각 {len(junk)}개 '
          f'({sum(len(j) for _, j in junk)}바이트)')

    by_vid = collections.defaultdict(list)
    for off, f in frames:
        m = decode(f)
        by_vid[m.vehicle_id].append((off, m.seq, f))

    for vid, lst in sorted(by_vid.items()):
        seqs = [s for _, s, _ in lst]
        cnt = collections.Counter(seqs)
        repeated = {s: c for s, c in cnt.items() if c > 1}
        name = VEHICLE_NAMES.get(vid, vid)
        print(f'\n[{name} id={vid}] 프레임 {len(lst)}개, seq {seqs[0]}~{seqs[-1]}, '
              f'고유 seq {len(cnt)}개')
        if repeated:
            worst = max(repeated.items(), key=lambda kv: kv[1])
            print(f'  중복 발견: {len(repeated)}개 seq가 반복됨, 최다 seq={worst[0]} x{worst[1]}회')
            same = [f for _, s, f in lst if s == worst[0]]
            identical = all(x == same[0] for x in same)
            print(f'  그 반복 프레임들이 바이트 단위로 완전히 동일한가: {identical}')
            first_off = next(o for o, s, _ in lst if s == worst[0])
            last_off = max(o for o, s, _ in lst if s == worst[0])
            print(f'  반복 구간: 스트림 {first_off}~{last_off} 바이트 위치')
        else:
            print('  중복 없음')

    # 3) 프레임이 아닌 바이트: ESP32 리셋 부트로그 여부
    if junk:
        text = b''.join(j for _, j in junk)
        ascii_part = ''.join(chr(c) if 32 <= c < 127 or c in (10, 13) else '.' for c in text[:400])
        print('\n프레임 아닌 바이트 앞부분 (ESP32 부트 로그면 rst:, boot: 같은 글자가 보임):')
        print(ascii_part)
        if b'rst:' in text or b'boot:' in text or b'ets ' in text:
            print('  -> 포트를 열 때 ESP32가 리셋된 흔적이 있음')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--id', type=int, default=3)
    ap.add_argument('--serial-port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--no-send', action='store_true', help='내 BSM 송신 안 함')
    ap.add_argument('--no-flush', action='store_true', help='열자마자 버퍼 비우기(reset_input_buffer) 생략')
    ap.add_argument('--replay', help='저장된 .bin 파일 분석만')
    args = ap.parse_args()

    if args.replay:
        raw = open(args.replay, 'rb').read()
        chunks = []
        log = args.replay[:-4] + '.log'
        try:
            chunks = [tuple(x) for x in json.load(open(log))]
        except (OSError, ValueError):
            pass
        analyze(raw, chunks)
        return

    import serial

    ser = serial.Serial()
    ser.port = args.serial_port
    ser.baudrate = args.baud
    ser.timeout = 0.01
    ser.dtr = False
    ser.rts = False
    t_open = time.monotonic()
    ser.open()
    time.sleep(0.3)
    waiting_before_flush = ser.in_waiting
    if not args.no_flush:
        ser.reset_input_buffer()
    t_start = time.monotonic()
    print(f'포트 열고 0.3초 뒤 대기 중이던 바이트: {waiting_before_flush} '
          f'({"비움" if not args.no_flush else "안 비움"})')

    stop = threading.Event()
    tx = [0]

    def sender():
        seq = 0
        while not stop.is_set():
            m = BSM(args.id, seq, int(time.monotonic() * 1000), 0.0, 0.0, 0.0, 0.0)
            ser.write(encode(m))
            tx[0] += 1
            seq = (seq + 1) & 0xFFFF
            stop.wait(0.1)

    if not args.no_send:
        threading.Thread(target=sender, daemon=True).start()

    raw = bytearray()
    chunks = []
    while time.monotonic() - t_start < args.seconds:
        n = ser.in_waiting
        data = ser.read(n or 1)
        if data:
            chunks.append((round(time.monotonic() - t_start, 4), len(data)))
            raw += data
    stop.set()
    ser.close()

    stamp = time.strftime('%H%M%S')
    open(f'probe_{stamp}.bin', 'wb').write(raw)
    json.dump(chunks, open(f'probe_{stamp}.log', 'w'))
    print(f'송신 {tx[0]}개, 저장: probe_{stamp}.bin / probe_{stamp}.log '
          f'(포트 열기~기록 시작 {1000 * (t_start - t_open):.0f}ms)')
    analyze(bytes(raw), chunks)


if __name__ == '__main__':
    main()
