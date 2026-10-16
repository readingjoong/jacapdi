"""USB 웹캠 실시간 YOLO 추론 창. q/Esc 종료.
사용: python webcam_live.py [device] [model.pt] [width] [height] [imgsz]"""
import sys, threading, time
import cv2
from ultralytics import YOLO

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
model_path = sys.argv[2] if len(sys.argv) > 2 else "/home/sean/models/sim.pt"
W = int(sys.argv[3]) if len(sys.argv) > 3 else 640
H = int(sys.argv[4]) if len(sys.argv) > 4 else 480
imgsz = int(sys.argv[5]) if len(sys.argv) > 5 else 640

m = YOLO(model_path)
cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H); cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    sys.exit("camera open failed: " + dev)

# 최신 프레임만 유지하는 캡처 스레드 (추론이 카메라를 기다리지 않도록)
latest, lock, running = None, threading.Lock(), True
def grab():
    global latest
    while running:
        ok, f = cap.read()
        if ok:
            with lock: latest = f
threading.Thread(target=grab, daemon=True).start()

win = "YOLO live - %s (q/Esc)" % model_path.split("/")[-1]
cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 960 if H == 480 else 720)
fps, t_prev = 0.0, time.perf_counter()
while True:
    with lock: f = None if latest is None else latest.copy()
    if f is None:
        time.sleep(0.01); continue
    r = m.predict(f, imgsz=imgsz, half=True, device="cuda:0", verbose=False)[0]
    out = r.plot()
    now = time.perf_counter(); fps = 0.9 * fps + 0.1 / (now - t_prev) if fps else 1 / (now - t_prev); t_prev = now
    txt = "%.1f FPS | infer %.1f ms | %dx%d imgsz %d | det %d" % (fps, r.speed["inference"], f.shape[1], f.shape[0], imgsz, len(r.boxes))
    cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(out, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow(win, out)
    k = cv2.waitKey(1) & 0xFF
    if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
        break
running = False; cap.release(); cv2.destroyAllWindows()
