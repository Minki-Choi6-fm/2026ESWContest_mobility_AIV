"""
AIV V2V BSM 프레임 (팀 프로토콜 문서 v0.1 기준, 26바이트 고정 길이)

  SYNC(1) VERSION(1) VEHICLE_ID(1) SEQ(2) TIMESTAMP(4) LAT(4) LON(4)
  SPEED(2) HEADING(2) K_VALUE(1) STATUS(1) CRC16(2) END(1)

- 바이트 순서: 리틀엔디안 (STM32 기본값과 통일. 문서 3-5번 열린 항목을 이 값으로 가정)
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), SYNC·CRC·END를 제외한 22바이트 대상
- 전송 계층(UDP / ESP-NOW)과 무관한 순수 인코딩/디코딩 모듈
"""
import struct
from dataclasses import dataclass
from typing import Optional

SYNC = 0xAA
END = 0x55
VERSION = 0x01
FRAME_LEN = 26

_BODY = struct.Struct('<BBHIiiHHBB')   # VERSION ~ STATUS, 22바이트
_CRC = struct.Struct('<H')
assert 1 + _BODY.size + _CRC.size + 1 == FRAME_LEN

VEHICLE_NAMES = {1: '후방차', 2: '전방차', 3: '측면차'}

STATUS_GPS_VALID = 1 << 0
STATUS_DANGER_ACTIVE = 1 << 1
STATUS_SENSOR_DEGRADED = 1 << 2
STATUS_SAFE_MODE = 1 << 3
STATUS_BRAKE_ACTIVE = 1 << 4   # 추가: 급제동 중 (예약 비트 사용, 기존 수신 코드는 무시하므로 호환)

K_NA = 0xFF   # STM32 없는 차량(측면차)


class FrameError(ValueError):
    """프레임 검증 실패. reason: 'len' | 'sync' | 'end' | 'crc' | 'version'"""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class BSM:
    vehicle_id: int
    seq: int
    timestamp_ms: int
    lat: float
    lon: float
    speed_kmh: float
    heading_deg: float
    k_value: Optional[float] = None   # None = N/A
    status: int = 0

    @property
    def gps_valid(self) -> bool:
        return bool(self.status & STATUS_GPS_VALID)

    @property
    def danger(self) -> bool:
        return bool(self.status & STATUS_DANGER_ACTIVE)

    @property
    def brake(self) -> bool:
        return bool(self.status & STATUS_BRAKE_ACTIVE)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def encode(m: BSM) -> bytes:
    k_raw = K_NA if m.k_value is None else _clamp(int(round(m.k_value * 100)), 0, 254)
    body = _BODY.pack(
        VERSION,
        m.vehicle_id & 0xFF,
        m.seq & 0xFFFF,
        m.timestamp_ms & 0xFFFFFFFF,
        _clamp(int(round(m.lat * 1e7)), -900_000_000, 900_000_000),
        _clamp(int(round(m.lon * 1e7)), -1_800_000_000, 1_800_000_000),
        _clamp(int(round(m.speed_kmh * 100)), 0, 0xFFFF),
        int(round((m.heading_deg % 360.0) * 100)) % 36000,
        k_raw,
        m.status & 0xFF,
    )
    return bytes([SYNC]) + body + _CRC.pack(crc16_ccitt(body)) + bytes([END])


def decode(frame: bytes) -> BSM:
    if len(frame) != FRAME_LEN:
        raise FrameError('len')
    if frame[0] != SYNC:
        raise FrameError('sync')
    if frame[-1] != END:
        raise FrameError('end')
    body = frame[1:1 + _BODY.size]
    (crc,) = _CRC.unpack_from(frame, 1 + _BODY.size)
    if crc16_ccitt(body) != crc:
        raise FrameError('crc')
    ver, vid, seq, ts, lat, lon, spd, hdg, k, st = _BODY.unpack(body)
    if ver != VERSION:
        raise FrameError('version')
    return BSM(
        vehicle_id=vid,
        seq=seq,
        timestamp_ms=ts,
        lat=lat / 1e7,
        lon=lon / 1e7,
        speed_kmh=spd / 100,
        heading_deg=hdg / 100,
        k_value=None if k == K_NA else k / 100,
        status=st,
    )


def is_valid_frame(frame: bytes) -> bool:
    try:
        decode(frame)
        return True
    except FrameError:
        return False
