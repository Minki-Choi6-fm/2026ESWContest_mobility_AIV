# 변경 내용 (실시간 Pi5용)

- `fusion.py`: 매칭되지 않은 레이더 점은 그리지 않습니다.
- 후보 매칭된 레이더 점만 청록색 원으로 표시하고, YOLO 박스 안쪽 상단에 `R<ID> 거리m 상대속도m/s CAND`를 추가합니다.
- `main.py`: `--pitch-deg` 옵션 추가. 기본값 +4°는 영상 관찰을 위한 **시험값**이며 정식 보정값이 아닙니다.
- 실시간 실행: `~/realsense-venv/bin/python main.py --record --fusion --pitch-deg 4`
- 생성된 MP4는 `--record-dir` 아래에 저장됩니다. 기존에 녹화한 MP4는 변경되지 않습니다.
- `CAND`는 단지 투영점이 YOLO 박스에 들어온 후보이며, 물체 동일성 검증 및 안전 제어에 이용해서는 안 됩니다.
- 원본 ZIP은 변경되지 않았습니다. 적용 전 Pi5의 기존 `main.py`, `fusion.py`를 백업하세요.
