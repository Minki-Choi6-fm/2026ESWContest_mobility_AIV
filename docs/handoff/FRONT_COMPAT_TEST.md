# 앞차 코드 호환 통합 시험 (S11-c) — 루프백, 로컬만

- 앞차 코드: `~/Downloads/v2v.zip` 을 **`~/front_test/`** 에 풀었다(`unzip -n`, 원본 수정 없음, 24개 파일). 실행: `python3 -u front_node.py --no-stm32 --transport udp --udp-port 29000 --broadcast-addr 127.255.255.255` (**`--buzzer-gpio` 안 줌**, GPIO 접근 없음). 기본 broadcast 주소(255.255.255.255)는 LAN 으로 나가므로 쓰지 않았다.
- 시험 도구: `pi5/pipeline/front_compat_test.py` (후방 `bsm_builder` → 후방 `v2v_adapter.encode` → 로컬 UDP → 앞차 `front_node`). 결과 파일: `logs/front_compat/front_compat_result.txt`. **11/11 PASS**.
| 항목 | 결과 |
|---|---|
| 1 후방 BSM(비트 5·6 켬, STATUS 0x61)이 앞차 decode 통과 | PASS — 앞차는 CRC/버전만 검사하고 STATUS 상위 비트를 무시(코드로 확인), 요약줄 `후방차=LIVE(lost 0)`, `drop=` 없음 |
| 2a 후방 DANGER_ACTIVE → 앞차 `경고 ON`(level 2) | PASS — 첫 위험 BSM 후 4 ms |
| 2b 요약줄 `warn=2` | PASS |
| 2c 위험 비트 끄면 해제 | PASS — 마지막 위험 BSM 후 **1.08 s** (WARN_HOLD_S = 1.0 s) |
| 3a 앞차 BSM(`front_node` 실제 출력)을 후방 decode + 신선도 통과 | PASS — 196/196 수락, id 2 |
| 3b 필드(36 km/h, GPS 유효, brake=0) | PASS |
| 3c 비트 4 BRAKE_ACTIVE 가 켜진 앞차 BSM(앞차 `bsm_protocol` 로 생성, STATUS 0x11) | PASS — 후방 decode 에서 `brake_active=True`, 3개 수락, STALE 아님. 단 `front_node` 는 STM32 없이는 브레이크 비트를 켜지 못해(속도 변화 추정만) 비트 4 는 합성 프레임으로 시험 |
| 3d 앞차 `GpioBuzzer(26)` mock 핀 | PASS — `MockFactory` 위에서만 (실 GPIO 안 염) |
| 4a 2 s 무송신 → `후방차=STALE` | PASS (300 ms 규칙) |
| 4b SEQ 를 7000대에서 0 으로 재시작 + 1.2 s 침묵 | PASS — 27 ms 만에 수락 |
| 4c SEQ 재시작 **침묵 없이**(후방이 1 s 안에 재부팅) | 문서화된 한계 확인: 앞차는 마지막 수락 프레임 후 1 s 가 지나야 새 SEQ 를 받는다 → 경고 ON 이 **0.99 s** 늦음. 후방 `freshness.py`(S4 수정)는 STALE 이면 SEQ 비교 없이 수락하지만 앞차는 아님 |
## 발견/제안
1. 비트 5·6: 앞차 `bsm_protocol.py` 는 비트 0~4 만 정의 → **충돌 없음**(상세 `BSM_STATUS_BITS.md` 갱신).
2. 4c: 후방이 재부팅하며 SEQ 가 0 으로 돌아가면 앞차 경고가 최대 1 s 늦다. 후방 쪽 해결안: 부팅 시 SEQ 를 시계 기반 값으로 시작(재시작해도 이전보다 커짐) — 제안만, 적용 안 함.
3. 앞차 `front_node` 는 위험 BSM 의 "내 뒤인지" 판정에 GPS 거리/방위를 쓴다. 시험은 앞차 DummySource(후방차를 30 m 뒤, 북쪽 36 km/h)로 위치를 맞췄다. 실차에서 GPS fix 가 없으면 `no_gps(accept)` 로 위험을 그냥 수락한다(앞차 코드 주석).
