"""S3 (3): pipeline state -> BSM field values. Pure (no I/O). Encoding is done by v2v_adapter.py.

Inputs : SensorState (S1), SpeedResult from get_speed() (S2), GpsFix from GpsPositionClient (S3).
Output : BsmFields. DANGER_ACTIVE = (danger level == 2) and K_VALUE = measured K, both only from the S4 simulation/replay chain
(brain.py); without them DANGER_ACTIVE = 0 and K = N/A. K is N/A (0xFF) until a K was actually MEASURED: the conservative default
K_r=0.5 is a calculation input only and is not advertised to the other car (it would look like a measurement).

STATUS bit layout of the 9/27 V2V code (bsm_protocol.py): bit0 GPS_VALID, bit1 DANGER_ACTIVE, bit2 SENSOR_DEGRADED, bit3 SAFE_MODE.
Bits 4-6 below are NOT in bsm_protocol.py; they come from PROTOCOLS.md and are [PROPOSAL] (not agreed with the V2V owner):
  bit4 BRAKE_ACTIVE, bit5 SPEED_SRC_GPS, bit6 SPEED_STALE.
Do not confuse with SENSOR_STATUS of the STM32 0x10 frame (bit4 = SAFE_MODE, bit5 = BRAKE_ACTIVE) -- different byte, different layout.
"""
from dataclasses import dataclass
from typing import Optional

from constants import SOURCE_GPS, SOURCE_NONE, SPEED_STALE_MS

# [PROPOSAL] BSM STATUS bits 4-6
BSM_STATUS_BRAKE_ACTIVE = 1 << 4
BSM_STATUS_SPEED_SRC_GPS = 1 << 5
BSM_STATUS_SPEED_STALE = 1 << 6
# STM32 0x10 SENSOR_STATUS bits used here
STM_SAFE_MODE = 1 << 4
STM_BRAKE_ACTIVE = 1 << 5

# sources whose absence/staleness means "degraded". Radar is judged by radar_alive() (RADAR_OK), NOT by 0x12 presence.
DEGRADED_IF_INVALID = ("summary", "heartbeat", "imu", "obd")


@dataclass(frozen=True)
class BsmFields:
    lat: float
    lon: float
    speed_kmh: float
    heading_deg: float
    k_value: Optional[float]
    gps_valid: bool
    danger_active: bool
    sensor_degraded: bool
    safe_mode: bool
    brake_active: bool          # [PROPOSAL] bit4
    speed_src_gps: bool         # [PROPOSAL] bit5
    speed_stale: bool           # [PROPOSAL] bit6


def build_fields(state, speed, fix, last_heading, now, danger_level=None, k_measured=None) -> BsmFields:
    """state: SensorState; speed: SpeedResult; fix: GpsFix or None (usable fix only); last_heading: last known course or None.
    danger_level: 0/1/2/0xFF from brain.py or None (not computed); k_measured: measured K or None."""
    st = state.sensor_status(now)                       # None when 0x10 is stale -> STM32 link not trustworthy
    safe_mode = st is not None and bool(st & STM_SAFE_MODE)
    brake = st is not None and bool(st & STM_BRAKE_ACTIVE)
    invalid = [s for s in DEGRADED_IF_INVALID if not state.is_valid(s, now)]
    degraded = bool(invalid) or not state.radar_alive(now) or speed.source == SOURCE_NONE
    gps_valid = fix is not None
    heading = fix.heading_deg if (fix is not None and fix.heading_deg is not None) else (last_heading or 0.0)
    stale = speed.source == SOURCE_NONE or (speed.age_ms is not None and speed.age_ms > SPEED_STALE_MS)
    return BsmFields(
        lat=fix.lat if gps_valid else 0.0, lon=fix.lon if gps_valid else 0.0,       # position ignored by receivers when GPS_VALID=0
        speed_kmh=0.0 if speed.speed_kmh is None else speed.speed_kmh, heading_deg=heading,
        k_value=k_measured, gps_valid=gps_valid, danger_active=(danger_level == 2),
        sensor_degraded=degraded, safe_mode=safe_mode, brake_active=brake,
        speed_src_gps=speed.source == SOURCE_GPS, speed_stale=stale)
