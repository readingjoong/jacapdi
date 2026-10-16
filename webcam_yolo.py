"""USB 웹캠 → YOLO GPU 추론 (ROS 없이). 사용: python webcam_yolo.py <device> <model.pt> <seconds>"""
import sys, time
import cv2, torch
from ultralytics import YOLO

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
model_path = sys.argv[2] if len(sys.argv) > 2 else "/home/sean/models/sim.pt"
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 15
m = YOLO(model_path)

for w, h in [(640, 480), (1280, 720)]:
    c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, w); c.set(cv2.CAP_PROP_FRAME_HEIGHT, h); c.set(cv2.CAP_PROP_FPS, 30)
    for _ in range(15): c.read()  # 노출 안정화
    # 1) 카메라 단독 수신 fps
    n, t = 0, time.perf_counter()
    while time.perf_counter() - t < 3:
        ok, f = c.read(); n += ok
    cam_fps = n / 3
    # 2) 카메라 + 추론 루프 (실사용과 동일)
    for _ in range(5): m.predict(f, imgsz=640, half=True, device="cuda:0", verbose=False)
    n, dets, t = 0, [], time.perf_counter()
    while time.perf_counter() - t < secs:
        ok, f = c.read()
        if not ok: continue
        r = m.predict(f, imgsz=640, half=True, device="cuda:0", verbose=False)[0]
        n += 1; dets.append(len(r.boxes))
    torch.cuda.synchronize()
    loop_fps = n / (time.perf_counter() - t)
    names = {}
    for cls in r.boxes.cls.tolist(): names[m.names[int(cls)]] = names.get(m.names[int(cls)], 0) + 1
    print("%dx%d (실제 %dx%d): 카메라 단독 %.1f fps | 카메라+YOLO %.1f fps (%d프레임) | 프레임당 검출 평균 %.1f, 0개인 프레임 %d/%d | 마지막 프레임: %s"
          % (w, h, f.shape[1], f.shape[0], cam_fps, loop_fps, n, sum(dets) / max(n, 1), dets.count(0), n, names or "없음"))
    cv2.imwrite("/tmp/yolo_test/webcam_%d.jpg" % w, r.plot())
    c.release()
