"""ROS 없이 YOLO 모델만 GPU에서 속도/검출 확인. 사용: python yolo_bench.py <model.pt> <이미지폴더>"""
import glob, sys, time
import cv2, torch
from ultralytics import YOLO

model_path = sys.argv[1] if len(sys.argv) > 1 else "/home/sean/models/sim.pt"
img_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/yolo_test"
m = YOLO(model_path)
print("model:", model_path, "| task:", m.task, "| classes:", m.names)
imgs = [cv2.imread(p) for p in sorted(glob.glob(img_dir + "/gz_*.jpg"))]
cam = cv2.imread(img_dir + "/cam_00.jpg")

for imgsz, half in [(320, True), (640, True), (640, False)]:
    for _ in range(10):
        m.predict(imgs[0], imgsz=imgsz, half=half, device="cuda:0", verbose=False)
    torch.cuda.synchronize()
    n = 100
    t = time.perf_counter()
    for i in range(n):
        r = m.predict(imgs[i % len(imgs)], imgsz=imgsz, half=half, device="cuda:0", verbose=False)
    torch.cuda.synchronize()
    dt = (time.perf_counter() - t) / n
    sp = r[0].speed
    print("imgsz=%d half=%s: %5.1f FPS (%.1f ms/img | pre %.1f / infer %.1f / post %.1f ms)"
          % (imgsz, half, 1 / dt, dt * 1000, sp["preprocess"], sp["inference"], sp["postprocess"]))
print("max GPU mem:", round(torch.cuda.max_memory_allocated() / 2**20), "MiB")

for conf in (0.25, 0.05):
    cnt = [len(m.predict(f, imgsz=640, conf=conf, device="cuda:0", verbose=False)[0].boxes) for f in imgs]
    print("gazebo %d frames conf=%.2f detections:" % (len(imgs), conf), cnt)
if cam is not None:
    rc = m.predict(cam, imgsz=640, conf=0.05, device="cuda:0", verbose=False)[0]
    print("laptop camera conf=0.05:", [(m.names[int(c)], round(float(s), 2)) for c, s in zip(rc.boxes.cls, rc.boxes.conf)])
r = m.predict(imgs[0], imgsz=640, conf=0.05, device="cuda:0", verbose=False)[0]
cv2.imwrite(img_dir + "/result_gz.jpg", r.plot())
