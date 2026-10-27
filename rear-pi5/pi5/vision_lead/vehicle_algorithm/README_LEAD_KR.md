선행 차량 후보 표시 (진단·시각화용)

레이더 기준 횡방향 위치 |left_m| <= 1.75m, 전방 1~100m 내에서 YOLO와 1:1 매칭된 후보 중 가장 가까운 물체를 선택합니다.
진짜 차선 검출/추적이 없으므로 실제 내 차선 차량이라고 확정할 수 없습니다.
화면에 LEAD CAND 거리/상대속도/레이더 ID 표시, 선택된 YOLO box 주황색 강조.

Pi5: ~/realsense-venv/bin/python main.py --record --fusion --pitch-deg 4
Ubuntu replay: ~/yolo_env/bin/python ~/Downloads/replay_lead_fusion.py ~/aiv_drive_logs/drive_20261016_152211 --fusion-file ~/Downloads/vehicle_lead_modified/vehicle_algorithm/fusion.py --yolo-model ~/Downloads/vehicle_lead_modified/vehicle_algorithm/yolov8n.pt --start 0 --dur 10 --skip 2 --pitch-deg 4 --out ~/aiv_drive_logs/drive_20261016_152211/lead_test.mp4

WARNING: NOT for active safety control or emergency braking. Needs sensor calibration, temporal tracking, lane geometry and confirmation.
