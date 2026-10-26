"""
AIV V2V BSM 프레임 인코딩/디코딩 모듈

설계 문서(V2V_통신_프로토콜_설계.md) 1장의 27바이트 고정 프레임을 그대로 구현합니다.
전송 방식(UDP / ESP-NOW)과 완전히 분리되어 있어서, transport.py 쪽만 바꾸면
이 모듈은 손대지 않고 그대로 재사용할 수 있습니다.

프레임 구조:
  SYNC(1) VERSION(1) VEHICLE_ID(1) SEQ(2) TIMESTAMP(4)
  LAT(4) LON(4) SPEED(2) HEADING(2) K_VALUE(1) STATUS(1)
  CRC16(2) END(1)
  = 총 26바이트
  (설계문서 초안에는 27바이트로 적혀있었는데, 실제 필드 합산 결과 26바이트가 맞습니다.
   문서 쪽 오타이니 참고 시 26바이트로 정정해서 봐주세요.)
"""

import struct
import time

# ---- 상수 ----
SYNC_BYTE = 0xAA
END_BYTE = 0x55
PROTOCOL_VERSION = 0x01
FRAME_SIZE = 26

# 차량 ID
VEHICLE_REAR = 0x01    # 후방차
VEHICLE_FRONT = 0x02   # 전방차
VEHICLE_SIDE = 0x03    # 측면차

VEHICLE_NAMES = {
    VEHICLE_REAR: "후방차",
    VEHICLE_FRONT: "전방차",
    VEHICLE_SIDE: "측면차",
}

K_VALUE_NA = 0xFF  # STM32가 없는 차량(측면차)용 "값 없음"

# STATUS 비트플래그
STATUS_GPS_VALID = 1 << 0
STATUS_DANGER_ACTIVE = 1 << 1
STATUS_SENSOR_DEGRADED = 1 << 2
STATUS_SAFE_MODE = 1 << 3
# [PROPOSAL] bit 4-6: 팀(V2V 담당) 합의 전 제안값. PROTOCOLS.md 의 STATUS 제안과 같음.
STATUS_BRAKE_ACTIVE = 1 << 4    # IMU 감속 추정 기반 (실제 브레이크 페달 신호 아님)
STATUS_SPEED_SRC_GPS = 1 << 5   # 속도 출처가 GPS (OBD 아님)
STATUS_SPEED_STALE = 1 << 6     # 속도 값이 오래됐거나 없음

# struct 포맷: '<' = 리틀엔디안 (STM32 기본과 통일, 설계문서 3장 참고)
# B=uint8, H=uint16, I=uint32, i=int32
_STRUCT_FMT = "<BBBHIiiHHBB"
# SYNC VERSION VEHICLE_ID SEQ TIMESTAMP LAT LON SPEED HEADING K_VALUE STATUS
# (CRC16과 END는 별도로 뒤에 붙인다)


class BSMDecodeError(Exception):
    """프레임 파싱/검증 실패 시 발생"""
    pass


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly=0x1021). 설계문서에서 지정한 무결성 체크용."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_bsm(
    vehicle_id: int,
    seq: int,
    lat: float,
    lon: float,
    speed_kmh: float,
    heading_deg: float,
    k_value: float | None,
    gps_valid: bool = True,
    danger_active: bool = False,
    sensor_degraded: bool = False,
    safe_mode: bool = False,
    timestamp_ms: int | None = None,
    brake_active: bool = False,      # [PROPOSAL] bit4 (기존 호출과의 호환을 위해 맨 뒤에 추가, 기본값 False)
    speed_src_gps: bool = False,     # [PROPOSAL] bit5
    speed_stale: bool = False,       # [PROPOSAL] bit6
) -> bytes:
    """BSM 프레임을 27바이트 바이너리로 인코딩합니다.

    Args:
        vehicle_id: VEHICLE_REAR / VEHICLE_FRONT / VEHICLE_SIDE
        seq: 0~65535 시퀀스 번호 (호출하는 쪽에서 매 프레임마다 증가시켜야 함)
        lat, lon: 위경도 (도 단위, 예: 37.5326)
        speed_kmh: 속도 (km/h)
        heading_deg: 방위각 (0~359.99도)
        k_value: 유효 제동계수 (0.0~2.0 정도 범위, 측면차처럼 없으면 None)
        gps_valid, danger_active, sensor_degraded, safe_mode: STATUS 비트
        timestamp_ms: 미지정 시 자동으로 현재 monotonic 시각(ms) 사용
    """
    if timestamp_ms is None:
        timestamp_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

    lat_fixed = int(round(lat * 1e7))
    lon_fixed = int(round(lon * 1e7))
    speed_fixed = max(0, min(65535, int(round(speed_kmh * 100))))
    heading_fixed = max(0, min(35999, int(round(heading_deg * 100))))
    k_fixed = K_VALUE_NA if k_value is None else max(0, min(254, int(round(k_value * 100))))

    status = 0
    if gps_valid:
        status |= STATUS_GPS_VALID
    if danger_active:
        status |= STATUS_DANGER_ACTIVE
    if sensor_degraded:
        status |= STATUS_SENSOR_DEGRADED
    if safe_mode:
        status |= STATUS_SAFE_MODE
    if brake_active:
        status |= STATUS_BRAKE_ACTIVE
    if speed_src_gps:
        status |= STATUS_SPEED_SRC_GPS
    if speed_stale:
        status |= STATUS_SPEED_STALE

    body = struct.pack(
        _STRUCT_FMT,
        SYNC_BYTE,
        PROTOCOL_VERSION,
        vehicle_id,
        seq & 0xFFFF,
        timestamp_ms,
        lat_fixed,
        lon_fixed,
        speed_fixed,
        heading_fixed,
        k_fixed,
        status,
    )

    # CRC는 SYNC를 제외한 나머지 필드에 대해 계산 (설계문서 1-1 표 참고)
    crc = crc16_ccitt(body[1:])
    frame = body + struct.pack("<H", crc) + bytes([END_BYTE])

    assert len(frame) == FRAME_SIZE, f"프레임 길이 불일치: {len(frame)} != {FRAME_SIZE}"
    return frame


def decode_bsm(frame: bytes) -> dict:
    """27바이트 프레임을 파싱합니다. 검증 실패 시 BSMDecodeError를 던집니다."""
    if len(frame) != FRAME_SIZE:
        raise BSMDecodeError(f"프레임 길이 오류: {len(frame)} (기대값 {FRAME_SIZE})")

    if frame[0] != SYNC_BYTE:
        raise BSMDecodeError(f"SYNC 불일치: {frame[0]:#x}")
    if frame[-1] != END_BYTE:
        raise BSMDecodeError(f"END 불일치: {frame[-1]:#x}")

    body = frame[:-3]  # SYNC ~ STATUS까지
    crc_recv = struct.unpack("<H", frame[-3:-1])[0]
    crc_calc = crc16_ccitt(body[1:])
    if crc_recv != crc_calc:
        raise BSMDecodeError(f"CRC 불일치: 수신={crc_recv:#06x} 계산={crc_calc:#06x}")

    (
        _sync,
        version,
        vehicle_id,
        seq,
        timestamp_ms,
        lat_fixed,
        lon_fixed,
        speed_fixed,
        heading_fixed,
        k_fixed,
        status,
    ) = struct.unpack(_STRUCT_FMT, body)

    if version != PROTOCOL_VERSION:
        # 당장은 폐기하지 않고 경고만 (설계문서 3장 5번 항목 참고, 추후 팀 논의)
        pass

    return {
        "version": version,
        "vehicle_id": vehicle_id,
        "vehicle_name": VEHICLE_NAMES.get(vehicle_id, f"unknown({vehicle_id:#x})"),
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "lat": lat_fixed / 1e7,
        "lon": lon_fixed / 1e7,
        "speed_kmh": speed_fixed / 100.0,
        "heading_deg": heading_fixed / 100.0,
        "k_value": None if k_fixed == K_VALUE_NA else k_fixed / 100.0,
        "gps_valid": bool(status & STATUS_GPS_VALID),
        "danger_active": bool(status & STATUS_DANGER_ACTIVE),
        "sensor_degraded": bool(status & STATUS_SENSOR_DEGRADED),
        "safe_mode": bool(status & STATUS_SAFE_MODE),
        "brake_active": bool(status & STATUS_BRAKE_ACTIVE),      # [PROPOSAL]
        "speed_src_gps": bool(status & STATUS_SPEED_SRC_GPS),    # [PROPOSAL]
        "speed_stale": bool(status & STATUS_SPEED_STALE),        # [PROPOSAL]
        "status_raw": status,
        "recv_monotonic_ms": int(time.monotonic() * 1000),
    }


if __name__ == "__main__":
    # 간단한 자체 테스트: 인코딩 후 바로 디코딩해서 원래 값과 맞는지 확인
    frame = encode_bsm(
        vehicle_id=VEHICLE_REAR,
        seq=42,
        lat=37.55, lon=127.07,
        speed_kmh=63.5,
        heading_deg=182.3,
        k_value=0.72,
        gps_valid=True,
        danger_active=True,
    )
    print("frame bytes:", frame.hex(" "))
    print("frame length:", len(frame))
    decoded = decode_bsm(frame)
    for k, v in decoded.items():
        print(f"  {k}: {v}")
