"""
차량별 시퀀스 검사 + 신선도(STALE) 판단 — 프로토콜 문서 1-4절

- SEQ가 이전 값보다 작거나 같으면 중복/역순으로 보고 폐기 (65535→0 순환 고려)
- 마지막 유효 프레임 이후 300ms 초과 → STALE (타겟팅/위험판단에서 제외)
- 1초 넘게 끊겼다가 다시 들어오면 송신측 재부팅으로 보고 SEQ 기준을 새로 잡음
"""
import time
from typing import Dict, List, Optional

STALE_MS = 300
SEQ_RESET_MS = 1000


class _State:
    __slots__ = ('seq', 't', 'bsm', 'count', 'lost', 'dup')

    def __init__(self, bsm, now):
        self.seq = bsm.seq
        self.t = now
        self.bsm = bsm
        self.count = 1
        self.lost = 0
        self.dup = 0


class VehicleTracker:
    def __init__(self, stale_ms: int = STALE_MS, reset_ms: int = SEQ_RESET_MS):
        self.stale_s = stale_ms / 1000
        self.reset_s = reset_ms / 1000
        self._v: Dict[int, _State] = {}

    def accept(self, bsm, now: Optional[float] = None) -> str:
        """'ok' 또는 'dup' 반환"""
        now = time.monotonic() if now is None else now
        st = self._v.get(bsm.vehicle_id)
        if st is None:
            self._v[bsm.vehicle_id] = _State(bsm, now)
            return 'ok'
        if now - st.t > self.reset_s:
            st.seq, st.t, st.bsm = bsm.seq, now, bsm
            st.count += 1
            return 'ok'
        diff = (bsm.seq - st.seq) & 0xFFFF
        if diff == 0 or diff >= 0x8000:
            st.dup += 1
            return 'dup'
        st.lost += diff - 1
        st.seq, st.t, st.bsm = bsm.seq, now, bsm
        st.count += 1
        return 'ok'

    def is_live(self, vid: int, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        st = self._v.get(vid)
        return st is not None and (now - st.t) <= self.stale_s

    def live_bsms(self, now: Optional[float] = None) -> List:
        now = time.monotonic() if now is None else now
        return [st.bsm for vid, st in self._v.items() if self.is_live(vid, now)]

    def summary(self, now: Optional[float] = None) -> List[dict]:
        now = time.monotonic() if now is None else now
        out = []
        for vid in sorted(self._v):
            st = self._v[vid]
            out.append(dict(
                vid=vid,
                live=(now - st.t) <= self.stale_s,
                age_ms=int((now - st.t) * 1000),
                count=st.count,
                lost=st.lost,
                dup=st.dup,
                bsm=st.bsm,
            ))
        return out
