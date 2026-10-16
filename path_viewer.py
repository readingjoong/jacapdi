"""/detections(선) + /lane_detections(lane 폴리곤) + /lane_trajectory + /topic_control_signal 시각화. q/Esc 종료.
좌: 원본 + 선 마스크 + lane 영역 + IPM 사다리꼴 + 경로 역투영 / 우: BEV + 경로 곡선.
IPM/미터 변환 값은 lane_info_extractor_node 기본값과 동일하게 사용."""
import os, sys, threading
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

IMAGE_TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
LANE_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/lane_detections"
CLASS = "lane"
IPM_SRC = np.float32([238.0, 316.0, 402.0, 313.0, 501.0, 476.0, 155.0, 476.0]).reshape(4, 2)
BASE_W, BASE_H, L_R, R_R, BEV_SCALE = 640.0, 480.0, 0.3, 0.7, 4.0
LAT_MPP, LON_MPP = 0.005, 0.01
# lane_info_extractor 와 같은 IPM 파라미터 파일이 있으면 그 값을 쓴다 (3번째 인자)
IPM_YAML = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")
if os.path.isfile(IPM_YAML):
    import yaml
    _p = yaml.safe_load(open(IPM_YAML))["lane_info_extractor_node"]["ros__parameters"]
    IPM_SRC = np.float32(_p["ipm_src_points"]).reshape(4, 2)
    BASE_W, BASE_H = _p["ipm_src_base_width"], _p["ipm_src_base_height"]
    L_R, R_R, BEV_SCALE = _p["ipm_dst_left_ratio"], _p["ipm_dst_right_ratio"], _p["bev_height_scale"]
    LAT_MPP, LON_MPP = _p["lateral_meter_per_pixel"], _p["longitudinal_meter_per_pixel"]
COLORS = {}

class Viewer(Node):
    def __init__(self):
        super().__init__("path_viewer")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.img, self.det, self.traj, self.lane, self.cmd = None, None, None, None, None
        self.n_traj, self.n_valid = 0, 0
        self.create_subscription(Image, IMAGE_TOPIC, self.on_img, 10)
        self.create_subscription(DetectionArray, "/detections", self.on_det, q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.create_subscription(DetectionArray, LANE_TOPIC, self.on_lane, q)
        self.create_subscription(MotionCommand, "/topic_control_signal", self.on_cmd, q)
    def on_lane(self, m):
        with self.lock: self.lane = m
    def on_cmd(self, m):
        with self.lock: self.cmd = m
    def on_img(self, m):
        with self.lock: self.img = self.bridge.imgmsg_to_cv2(m, "bgr8")
    def on_det(self, m):
        with self.lock: self.det = m
    def on_traj(self, m):
        with self.lock:
            self.traj = m; self.n_traj += 1; self.n_valid += int(m.valid)

def color(name):
    if name not in COLORS:
        rng = np.random.default_rng(abs(hash(name)) % 2**32); COLORS[name] = tuple(int(c) for c in rng.integers(60, 255, 3))
    return COLORS[name]

def render(v):
    with v.lock:
        img = None if v.img is None else v.img.copy(); det, traj, lane, cmd = v.det, v.traj, v.lane, v.cmd; nt, nv = v.n_traj, v.n_valid
    if img is None:
        return None
    h, w = img.shape[:2]
    overlay, sel = img.copy(), np.zeros((h, w), np.uint8)
    counts = {}
    if det is not None:
        for d in det.detections:
            counts[d.class_name] = counts.get(d.class_name, 0) + 1
            if not d.mask.data: continue
            pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
            cv2.fillPoly(overlay, [pts], color(d.class_name))
            cv2.putText(img, d.class_name, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color(d.class_name), 1)
    if lane is not None:
        for d in lane.detections:
            if d.class_name == CLASS and d.mask.data:
                pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
                cv2.fillPoly(sel, [pts], 255)
                lo = overlay.copy(); cv2.fillPoly(lo, [pts], (0, 180, 0)); overlay = cv2.addWeighted(lo, 0.5, overlay, 0.5, 0)
                cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)
    img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)
    src = IPM_SRC.copy(); src[:, 0] *= w / BASE_W; src[:, 1] *= h / BASE_H
    bev_h = int(round(h * BEV_SCALE))
    dst = np.float32([[round(w * L_R), 0], [round(w * R_R), 0], [round(w * R_R), bev_h - 1], [round(w * L_R), bev_h - 1]])
    M = cv2.getPerspectiveTransform(src, dst); Minv = np.linalg.inv(M)
    cv2.polylines(img, [src.astype(np.int32)], True, (0, 255, 255), 1)
    bev = cv2.cvtColor(cv2.warpPerspective(sel, M, (w, bev_h), flags=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    status = "no trajectory yet"
    if traj is not None:
        status = "valid=%s pts=%d fit_err=%.3fm x=[%.2f,%.2f]m a=%.3f b=%.3f c=%.3f" % (
            traj.valid, traj.num_points, traj.fit_error_m, traj.x_min_m, traj.x_max_m, traj.a, traj.b, traj.c) if traj.valid else "valid=False"
        if traj.valid:
            xs = np.linspace(traj.x_min_m, traj.x_max_m, 60)
            ys = traj.a * xs**2 + traj.b * xs + traj.c
            bx = (w - 1) * 0.5 - ys / LAT_MPP; by = (bev_h - 1.0) - xs / LON_MPP
            bev_pts = np.stack([bx, by], 1).astype(np.float32)
            cv2.polylines(bev, [bev_pts.astype(np.int32)], False, (0, 0, 255), 6)
            img_pts = cv2.perspectiveTransform(bev_pts.reshape(-1, 1, 2), Minv).reshape(-1, 2)
            cv2.polylines(img, [img_pts.astype(np.int32)], False, (0, 0, 255), 3)
    bev_small = cv2.resize(bev, (int(w * h / bev_h), h))
    canvas = np.hstack([img, bev_small])
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 44), (0, 0, 0), -1)
    cmd_s = "none" if cmd is None else "s%d l%d r%d" % (cmd.steering, cmd.left_speed, cmd.right_speed)
    cv2.putText(canvas, "cmd=%s | %s" % (cmd_s, status), (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(canvas, "valid %d/%d | det: %s" % (nv, nt, ", ".join("%s:%d" % kv for kv in sorted(counts.items())) or "none"),
                (6, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return canvas

def main():
    rclpy.init(); v = Viewer()
    win = "path viewer (q/Esc)"; cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1400, 720)
    while True:
        for _ in range(5): rclpy.spin_once(v, timeout_sec=0.002)
        c = render(v)
        if c is not None: cv2.imshow(win, c)
        k = cv2.waitKey(15) & 0xFF
        if k in (ord("q"), 27): break
    cv2.destroyAllWindows(); v.destroy_node(); rclpy.try_shutdown()

if __name__ == "__main__":
    main()
