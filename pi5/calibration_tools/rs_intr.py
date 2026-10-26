import pyrealsense2 as rs
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
prof = pipe.start(cfg)
i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("해상도:", i.width, "x", i.height)
print("fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (i.fx, i.fy, i.ppx, i.ppy))
print("왜곡 모델:", i.model, "계수:", list(i.coeffs))
for _ in range(30):
    fr = pipe.wait_for_frames()
print("프레임 수신 OK")
pipe.stop()
