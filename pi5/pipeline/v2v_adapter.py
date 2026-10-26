"""S3: thin adapter between this pipeline and the V2V code (ESWContest_ws/v2v/v2v_stack).

Everything that touches the V2V code lives here, so replacing the V2V stack with another version means editing this file only.
Exposes: encode(), decode(), make_transport(), FreshnessTracker.

Since S4 the V2V code itself carries the [PROPOSAL] STATUS bits 4-6 (brake_active, speed_src_gps, speed_stale) as official
encode_bsm()/decode_bsm() arguments/keys, so no byte patching or CRC recomputation happens here anymore.
Frame (26 B, little-endian): 0 SYNC AA | 1 VER | 2 ID | 3-4 SEQ | 5-8 TS ms | 9-12 LAT 1e-7 | 13-16 LON 1e-7 | 17-18 SPEED 0.01km/h
| 19-20 HEADING 0.01deg | 21 K x100 (FF=N/A) | 22 STATUS | 23-24 CRC16-CCITT-FALSE over bytes 1..22 | 25 END 55.
"""
import os
import struct
import sys

_V2V = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "v2v", "v2v_stack"))
if _V2V not in sys.path:
    sys.path.insert(0, _V2V)
import bsm_protocol as _bp                                       # noqa: E402
from bsm_protocol import BSMDecodeError, FRAME_SIZE              # noqa: E402,F401
from freshness import FreshnessTracker, STALE_TIMEOUT_MS         # noqa: E402,F401
from transport import UDPBroadcastTransport                      # noqa: E402,F401


def encode(f, vehicle_id, seq, timestamp_ms=None):
    return _bp.encode_bsm(
        vehicle_id=vehicle_id, seq=seq, lat=f.lat, lon=f.lon, speed_kmh=f.speed_kmh, heading_deg=f.heading_deg, k_value=f.k_value,
        gps_valid=f.gps_valid, danger_active=f.danger_active, sensor_degraded=f.sensor_degraded, safe_mode=f.safe_mode,
        timestamp_ms=timestamp_ms, brake_active=f.brake_active, speed_src_gps=f.speed_src_gps, speed_stale=f.speed_stale)


def decode(frame):
    """-> bsm_protocol.decode_bsm() dict (includes brake_active, speed_src_gps, speed_stale, status_raw). Raises BSMDecodeError."""
    return _bp.decode_bsm(frame)


def make_transport(port, broadcast_addr, kind="udp", serial_port=None, baudrate=115200):
    """kind 'udp' (tested in loopback) or 'esp-now' (EspNowTransport over an ESP32 USB-serial bridge; NOT tested here, no radio test yet)."""
    if kind == "esp-now":
        from transport import EspNowTransport
        return EspNowTransport(serial_port=serial_port, baudrate=baudrate)
    return UDPBroadcastTransport(port=port, broadcast_addr=broadcast_addr)
