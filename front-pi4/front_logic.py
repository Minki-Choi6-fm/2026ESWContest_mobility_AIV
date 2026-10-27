"""
전방차 판단 로직 (하드웨어와 분리 — 시간 now 를 인자로 받아서 그대로 시험 가능)

전방차가 하는 일 (개발계획서 기준)
  1) 후방차가 필요한 정보를 BSM 으로 보낸다: 위치, 속도, 진행방향, 급제동 여부
  2) 후방차가 보낸 위험 경고(BSM danger)를 받으면, 정말 '내 뒤에 있는 차'가 보낸 것인지
     확인한 뒤 STM32 로 경고 명령(0x20)을 보내 부저/OLED 를 울리게 한다

입력 우선순위
  - 속도: STM32 의 OBD 차속(0x14, 신선할 때) > GPS 속도
  - 급제동: STM32 의 IMU 브레이크 플래그(0x13, 신선할 때) > 속도 변화로 직접 추정(보조)
  - 어떤 입력이 끊겨도 나머지로 계속 동작하고, 끊긴 사실은 BSM 의 SENSOR_DEGRADED 로 알린다
"""
import math
import struct
from collections import deque

# ---------------- 튜닝 값 ----------------
STM32_FRESH_S = 0.3          # STM32 프레임이 이보다 오래되면 그 값은 안 씀
OBD_FRESH_S = 0.6            # OBD 차속은 STM32 쪽 폴링이 느려서 조금 더 여유

BRAKE_ON_MPS2 = 3.0          # 속도 변화로 추정할 때: 이 감속도(약 0.3g) 이상이면 급제동
BRAKE_OFF_MPS2 = 1.5         # 이 아래로 내려가야 해제 (켜졌다 꺼졌다 반복 방지)
BRAKE_WINDOW_S = 0.5         # 감속도 계산 구간
BRAKE_MIN_HOLD_S = 0.5       # 한 번 켜지면 최소 이만큼 유지 (10Hz 기준 5프레임은 후방차에 전달되게)

WARN_HOLD_S = 1.0            # 경고가 한 번 들어오면 최소 이만큼 유지 (깜빡임 방지)
WARN_MAX_DIST_M = 100.0      # 이보다 멀리 있는 차의 경고는 무시
WARN_BEHIND_MIN_DEG = 120.0  # 상대 방위가 ±120° 밖(=뒤쪽 ±60°)이어야 '내 뒤'
WARN_HEADING_MAX_DEG = 60.0  # 진행방향 차이가 이보다 크면 다른 방향 차(마주오는 차 등)
HEADING_VALID_MPS = 2.0      # 이 속도 미만에선 GPS 진행방향을 믿지 않음

M_PER_DEG_LAT = 111_320.0

# ---------------- Pi -> STM32 경고 명령 (UART 규격 v0.2 에 추가 제안: v0.3) ----------------
# 프레임 형식은 기존 규격과 동일: STX(02) TYPE LEN PAYLOAD XOR(TYPE^LEN^PAYLOAD) ETX(03)
TYPE_V2V_WARN = 0x20
V2V_WARN_LEN = 4             # level(u8) src_vid(u8) dist_dm(u16, 0xFFFF=모름)
WARN_NONE, WARN_CAUTION, WARN_DANGER = 0, 1, 2


def encode_v2v_warn(level: int, src_vid: int, dist_m) -> bytes:
    dist_dm = 0xFFFF if dist_m is None else max(0, min(0xFFFE, int(round(dist_m * 10))))
    payload = struct.pack('<BBH', level & 0xFF, src_vid & 0xFF, dist_dm)
    x = TYPE_V2V_WARN ^ V2V_WARN_LEN
    for b in payload:
        x ^= b
    return bytes([0x02, TYPE_V2V_WARN, V2V_WARN_LEN]) + payload + bytes([x, 0x03])


# ---------------- 좌표 계산 ----------------
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(x, y)) % 360.0


def rel_angle(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class FrontLogic:
    def __init__(self):
        # STM32 에서 받은 최신 값과 받은 시각
        self.imu = None;  self.imu_t = None      # 0x13 디코딩 결과
        self.obd = None;  self.obd_t = None      # 0x14
        self.summ = None; self.summ_t = None     # 0x10
        self.hb_t = None                         # 0x11
        # 급제동 추정(보조)용 속도 기록
        self._speeds = deque()                   # (t, v_mps)
        self._brake_est = False
        self._brake_on_t = None
        # 경고 상태
        self._warn_until = 0.0
        self._warn_src = 0
        self._warn_dist = None
        self.last_warn_reason = 'none'

    # ---------- STM32 입력 ----------
    def on_stm32(self, msg_type: int, d: dict, now: float):
        if msg_type == 0x13:
            self.imu, self.imu_t = d, now
        elif msg_type == 0x14:
            self.obd, self.obd_t = d, now
        elif msg_type == 0x10:
            self.summ, self.summ_t = d, now
        elif msg_type == 0x11:
            self.hb_t = now

    def stm32_alive(self, now) -> bool:
        ts = [t for t in (self.imu_t, self.obd_t, self.summ_t, self.hb_t) if t is not None]
        return bool(ts) and now - max(ts) <= 1.0

    # ---------- 속도 ----------
    def speed(self, gps_fix: dict, now: float):
        """(속도 km/h, 출처) — OBD 우선, 없으면 GPS"""
        if self.obd and self.obd_t is not None and now - self.obd_t <= OBD_FRESH_S:
            age_ms = self.obd.get('age_ms')
            if self.obd.get('speed_kmh') is not None and (age_ms is None or age_ms <= OBD_FRESH_S * 1000):
                return float(self.obd['speed_kmh']), 'obd'
        if gps_fix.get('gps_valid'):
            return float(gps_fix.get('speed_kmh', 0.0)), 'gps'
        return 0.0, 'none'

    # ---------- 급제동 ----------
    def brake(self, speed_kmh: float, speed_src: str, now: float):
        """(급제동 여부, 출처)"""
        if (self.imu and self.imu_t is not None and now - self.imu_t <= STM32_FRESH_S
                and self.imu.get('imu_valid')):
            # STM32 가 IMU 감속도로 판단한 값을 그대로 씀 (계획서: 판단은 STM32)
            self._speeds.clear()
            return bool(self.imu.get('brake')), 'imu'
        if speed_src == 'none':
            self._speeds.clear()
            self._brake_est = False
            return False, 'none'
        return self._estimate_brake(speed_kmh / 3.6, now), f'{speed_src}-diff'

    def _estimate_brake(self, v_mps: float, now: float) -> bool:
        self._speeds.append((now, v_mps))
        while self._speeds and now - self._speeds[0][0] > BRAKE_WINDOW_S * 2:
            self._speeds.popleft()
        # 구간 시작점: BRAKE_WINDOW_S 이상 전의 값 중 가장 최근 것
        old = None
        for t, v in self._speeds:
            if now - t >= BRAKE_WINDOW_S:
                old = (t, v)
        decel = None
        if old is not None and now > old[0]:
            decel = (old[1] - v_mps) / (now - old[0])   # 양수 = 감속
        if decel is not None:
            if not self._brake_est and decel >= BRAKE_ON_MPS2:
                self._brake_est, self._brake_on_t = True, now
            elif self._brake_est and decel < BRAKE_OFF_MPS2 and now - self._brake_on_t >= BRAKE_MIN_HOLD_S:
                self._brake_est = False
        return self._brake_est

    def k_value(self, now):
        if self.summ and self.summ_t is not None and now - self.summ_t <= 1.0:
            return self.summ.get('K')
        return None

    # ---------- 후방 경고 판단 ----------
    def warning(self, own_fix: dict, own_speed_kmh: float, others: list, now: float):
        """others: 신선한(STALE 아닌) 다른 차량 BSM 목록.
        반환: (level, src_vid, dist_m 또는 None)"""
        best = None
        for b in others:
            if not b.danger:
                continue
            ok, reason, dist = self._is_behind_me(own_fix, own_speed_kmh, b)
            self.last_warn_reason = f'vid{b.vehicle_id}:{reason}'
            if ok and (best is None or (dist or 1e9) < (best[1] or 1e9)):
                best = (b.vehicle_id, dist)
        if best is not None:
            self._warn_until = now + WARN_HOLD_S
            self._warn_src, self._warn_dist = best
        if now < self._warn_until:
            return WARN_DANGER, self._warn_src, self._warn_dist
        return WARN_NONE, 0, None

    def _is_behind_me(self, own: dict, own_speed_kmh: float, b):
        """경고를 보낸 차가 '내 뒤에서 같은 방향으로 오는 차'인지.
        GPS 를 모르면 확인할 수 없으므로 경고를 받아들인다 (놓치는 것보다 낫다)."""
        if not (own.get('gps_valid') and b.gps_valid):
            return True, 'no_gps(accept)', None
        dist = haversine_m(own['lat'], own['lon'], b.lat, b.lon)
        if dist > WARN_MAX_DIST_M:
            return False, f'far({dist:.0f}m)', dist
        if own_speed_kmh / 3.6 < HEADING_VALID_MPS:
            # 서 있으면 내 진행방향을 모름 -> 거리 조건만 보고 받아들임
            return True, 'stopped(accept)', dist
        rel = rel_angle(bearing_deg(own['lat'], own['lon'], b.lat, b.lon) - own['heading'])
        if abs(rel) < WARN_BEHIND_MIN_DEG:
            return False, f'not_behind({rel:+.0f}deg)', dist
        if b.speed_kmh / 3.6 >= HEADING_VALID_MPS:
            hd = abs(rel_angle(b.heading_deg - own['heading']))
            if hd > WARN_HEADING_MAX_DEG:
                return False, f'heading({hd:.0f}deg)', dist
        return True, 'behind', dist
