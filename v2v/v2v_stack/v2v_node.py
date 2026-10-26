"""
V2V 통신 테스트용 실행 스크립트.

각 라즈베리파이(후방차/전방차/측면차)에서 이 스크립트를 각자의 --id로 실행하면:
  1) 자기 BSM을 10Hz로 UDP 브로드캐스트
  2) 다른 차량들의 BSM을 수신해서 CRC/시퀀스/신선도 검증 후 화면에 출력

사용 예 (세 대의 라즈베리파이에서 각각 실행):
  후방차:  python3 v2v_node.py --id 1
  전방차:  python3 v2v_node.py --id 2
  측면차:  python3 v2v_node.py --id 3

세 라즈베리파이가 전부 같은 Wi-Fi 네트워크(같은 서브넷)에 붙어있어야 합니다.
GPS/센서가 아직 다 안 붙어있어도, 기본값으로 더미(시뮬레이션) 위치·속도를
생성해서 통신 자체는 바로 테스트할 수 있게 만들어뒀습니다.

ESP32(ESP-NOW)로 테스트하려면 --transport esp-now --serial-port /dev/ttyUSB0
옵션을 주면 됩니다. ESP32에는 esp32_espnow_bridge/esp32_espnow_bridge.ino를
먼저 업로드해둬야 합니다.
"""

import argparse
import math
import threading
import time

from bsm_protocol import (
    encode_bsm, decode_bsm, BSMDecodeError,
    VEHICLE_REAR, VEHICLE_FRONT, VEHICLE_SIDE, VEHICLE_NAMES,
)
from transport import UDPBroadcastTransport, EspNowTransport
from freshness import FreshnessTracker, STALE_TIMEOUT_MS

BSM_RATE_HZ = 10
BSM_PERIOD_S = 1.0 / BSM_RATE_HZ

VEHICLE_ID_MAP = {1: VEHICLE_REAR, 2: VEHICLE_FRONT, 3: VEHICLE_SIDE}

# 실제 GPS/OBD/레이더가 아직 다 안 붙었을 때 쓰는 더미 시뮬레이터.
# 나중에 실센서 값으로 교체할 지점을 명확히 표시해뒀습니다.
class DummySensorSource:
    """차량이 원을 그리며 도는 것처럼 위경도가 서서히 움직이는 가짜 데이터 생성기."""

    def __init__(self, base_lat: float, base_lon: float, vehicle_id: int):
        self.base_lat = base_lat
        self.base_lon = base_lon
        self.vehicle_id = vehicle_id
        self.t0 = time.monotonic()

    def read(self) -> dict:
        # TODO: 실제 GPS(gpsd)/OBD(속도)/IMU(헤딩) 값으로 교체
        t = time.monotonic() - self.t0
        radius_deg = 0.0003
        angle = t * 0.2 + self.vehicle_id  # 차량마다 위상 다르게
        lat = self.base_lat + radius_deg * math.sin(angle)
        lon = self.base_lon + radius_deg * math.cos(angle)
        speed_kmh = 40 + 10 * math.sin(t * 0.3)
        heading_deg = (math.degrees(angle) + 90) % 360
        return {"lat": lat, "lon": lon, "speed_kmh": speed_kmh, "heading_deg": heading_deg}


def sender_loop(vehicle_id: int, transport: UDPBroadcastTransport, sensor: DummySensorSource,
                 stop_event: threading.Event):
    seq = 0
    k_value = None if vehicle_id == VEHICLE_SIDE else 0.75  # 측면차는 STM32 없음 -> K값 없음
    while not stop_event.is_set():
        s = sensor.read()
        frame = encode_bsm(
            vehicle_id=vehicle_id,
            seq=seq,
            lat=s["lat"], lon=s["lon"],
            speed_kmh=s["speed_kmh"],
            heading_deg=s["heading_deg"],
            k_value=k_value,
            gps_valid=True,
            danger_active=False,
        )
        transport.send_bytes(frame)
        seq = (seq + 1) & 0xFFFF
        time.sleep(BSM_PERIOD_S)


def receiver_loop(my_vehicle_id: int, transport: UDPBroadcastTransport,
                   tracker: FreshnessTracker, stop_event: threading.Event):
    while not stop_event.is_set():
        data = transport.recv_bytes(timeout=0.2)
        if data is None:
            continue
        try:
            decoded = decode_bsm(data)
        except BSMDecodeError as e:
            print(f"[DROP] 프레임 검증 실패: {e}")
            continue

        if decoded["vehicle_id"] == my_vehicle_id:
            continue  # 자기 자신이 브로드캐스트한 걸 다시 받은 경우 무시

        accepted = tracker.update(decoded)
        tag = "OK " if accepted else "DUP"
        print(
            f"[{tag}] {decoded['vehicle_name']:>4} seq={decoded['seq']:5d} "
            f"speed={decoded['speed_kmh']:5.1f}km/h heading={decoded['heading_deg']:6.1f} "
            f"K={decoded['k_value']} danger={decoded['danger_active']}"
        )


def status_printer_loop(tracker: FreshnessTracker, stop_event: threading.Event):
    """1초마다 각 차량이 STALE인지 요약 출력."""
    while not stop_event.is_set():
        time.sleep(1.0)
        if not tracker.known_vehicles():
            continue
        parts = []
        for vid in sorted(tracker.known_vehicles()):
            stale = tracker.is_stale(vid)
            name = VEHICLE_NAMES.get(vid, str(vid))
            parts.append(f"{name}={'STALE' if stale else 'LIVE'}")
        print("  --- 상태:", " | ".join(parts), f"(기준 {STALE_TIMEOUT_MS}ms) ---")


def main():
    parser = argparse.ArgumentParser(description="AIV V2V BSM 통신 테스트 노드")
    parser.add_argument("--id", type=int, choices=[1, 2, 3], required=True,
                         help="1=후방차 2=전방차 3=측면차")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--broadcast-addr", type=str, default="255.255.255.255",
                         help="같은 서브넷 브로드캐스트 주소. 필요시 예: 192.168.1.255")
    parser.add_argument("--base-lat", type=float, default=37.5509)
    parser.add_argument("--base-lon", type=float, default=127.0741)
    parser.add_argument("--transport", choices=["udp", "esp-now"], default="udp",
                         help="udp=RPi 내장 Wi-Fi 브로드캐스트(기본), esp-now=ESP32 USB-Serial 브릿지")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyUSB0",
                         help="--transport esp-now일 때 ESP32가 물린 시리얼 포트")
    parser.add_argument("--baudrate", type=int, default=115200,
                         help="--transport esp-now일 때 시리얼 보레이트 (esp32_espnow_bridge.ino와 맞출 것)")
    args = parser.parse_args()

    vehicle_id = VEHICLE_ID_MAP[args.id]
    print(f"=== {VEHICLE_NAMES[vehicle_id]} (id={vehicle_id}) V2V 테스트 시작 ===")
    print(f"전송방식={args.transport} 주기={BSM_RATE_HZ}Hz")

    if args.transport == "udp":
        print(f"포트={args.port} 브로드캐스트={args.broadcast_addr}")
        transport = UDPBroadcastTransport(port=args.port, broadcast_addr=args.broadcast_addr)
    else:
        print(f"시리얼포트={args.serial_port} 보레이트={args.baudrate}")
        transport = EspNowTransport(serial_port=args.serial_port, baudrate=args.baudrate)
    sensor = DummySensorSource(args.base_lat, args.base_lon, vehicle_id)
    tracker = FreshnessTracker()
    stop_event = threading.Event()

    threads = [
        threading.Thread(target=sender_loop, args=(vehicle_id, transport, sensor, stop_event), daemon=True),
        threading.Thread(target=receiver_loop, args=(vehicle_id, transport, tracker, stop_event), daemon=True),
        threading.Thread(target=status_printer_loop, args=(tracker, stop_event), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n종료합니다...")
    finally:
        stop_event.set()
        transport.close()


if __name__ == "__main__":
    main()
