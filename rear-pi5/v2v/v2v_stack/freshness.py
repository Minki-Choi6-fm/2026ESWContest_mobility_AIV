"""
차량별 데이터 신선도(freshness) 추적기.

설계문서 1-4절: 특정 VEHICLE_ID로부터 마지막 유효 프레임 이후
STALE_TIMEOUT_MS(기본 300ms)가 지나면 그 차량 데이터를 무효(STALE) 처리합니다.
"""

import time

STALE_TIMEOUT_MS = 300  # 10Hz 기준 3프레임 분량 (설계문서 1-4절과 동일)


class FreshnessTracker:
    def __init__(self, stale_timeout_ms: int = STALE_TIMEOUT_MS):
        self.stale_timeout_ms = stale_timeout_ms
        # vehicle_id -> {"last_seq": int, "last_recv_ms": int, "last_data": dict}
        self._table: dict[int, dict] = {}

    def update(self, decoded: dict) -> bool:
        """새로 디코딩된 BSM을 반영합니다.

        Returns:
            True  -> 정상 반영됨 (SEQ가 이전보다 증가함, 또는 처음/STALE 이후 첫 프레임)
            False -> 재전송/중복/역순으로 판단되어 폐기됨
        """
        vid = decoded["vehicle_id"]
        seq = decoded["seq"]
        now_ms = int(time.monotonic() * 1000)

        prev = self._table.get(vid)
        # 처음 보는 차량이거나, 이미 STALE(마지막 수락 후 STALE_TIMEOUT_MS 초과)이면 SEQ 비교 없이 받아들이고
        # last_seq 를 새로 시작한다. (송신 노드가 재시작해 SEQ 가 0 부터 다시 시작해도 복구되도록)
        # 판단은 STALE 기준만 사용한다: 송신 노드 재시작(프로세스 기동)은 항상 300ms 보다 오래 걸리므로
        # timestamp_ms 역행 검사는 추가하지 않는다.
        if prev is not None and (now_ms - prev["last_recv_ms"]) <= self.stale_timeout_ms:
            prev_seq = prev["last_seq"]
            # uint16 순환(wraparound)을 감안한 "seq가 더 최신인가" 판단
            diff = (seq - prev_seq) & 0xFFFF
            if diff == 0 or diff > 0x8000:
                # 같은 프레임 재수신 또는 과거 프레임(역순) -> 폐기
                return False

        self._table[vid] = {
            "last_seq": seq,
            "last_recv_ms": now_ms,
            "last_data": decoded,
        }
        return True

    def is_stale(self, vehicle_id: int) -> bool:
        """해당 차량이 STALE 상태인지 확인. 한 번도 못 받았어도 STALE 취급."""
        entry = self._table.get(vehicle_id)
        if entry is None:
            return True
        now_ms = int(time.monotonic() * 1000)
        return (now_ms - entry["last_recv_ms"]) > self.stale_timeout_ms

    def get_valid(self, vehicle_id: int) -> dict | None:
        """STALE이 아니면 마지막 데이터를, STALE이면 None을 반환."""
        if self.is_stale(vehicle_id):
            return None
        return self._table[vehicle_id]["last_data"]

    def known_vehicles(self) -> list[int]:
        return list(self._table.keys())
