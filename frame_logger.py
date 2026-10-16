# -*- coding: utf-8 -*-
"""주행 시계열 기록기 (10 Hz). 주행에는 관여하지 않는다(구독만).
사용: python3 frame_logger.py <출력폴더> [hz=10] [raw=0]
  <출력폴더>/frames/NNNNNN_HHMMSS.mmm.png : 카메라 + YOLO 마스크/확신도 + IPM 영역 + 경로 곡선 + 명령/조향값
  <출력폴더>/raw/NNNNNN_....png            : (raw=1 일 때) 덧칠 없는 원본
  <출력폴더>/frames.csv                     : 같은 번호의 시계열 한 줄씩
보드 조향값(ADC/목표)은 같은 기록 폴더 위의 board.log 마지막 줄에서 읽는다."""
import csv, os, sys, time, threading
import cv2, numpy as np, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

OUT = sys.argv[1]
HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
RAW = (sys.argv[3] if len(sys.argv) > 3 else "0") == "1"
BOARD = os.path.join(os.path.dirname(OUT.rstrip("/")), "board.log")
IPM = yaml.safe_load(open(os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")))["lane_info_extractor_node"]["ros__parameters"]
COL = {"solid_line": (255, 80, 0), "dashed_line": (0, 220, 0)}
os.makedirs(OUT + "/frames", exist_ok=True)
if RAW: os.makedirs(OUT + "/raw", exist_ok=True)

def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9

def board_last():
    """board.log 끝부분에서 마지막 조향 보고 (ADC, T, S, FAULT 여부)."""
    try:
        with open(BOARD, "rb") as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 600)); lines = f.read().decode(errors="replace").splitlines()
    except OSError:
        return "", "", ""
    for l in reversed(lines):
        if "ADC=" in l:
            kv = dict(x.split("=", 1) for x in l.split()[1:] if "=" in x)
            state = "FAULT" if "FAULT" in l else ("IDLE" if "IDLE" in l else "ACTIVE")
            return kv.get("ADC", ""), kv.get("T", ""), state
    return "", "", ""

class Logger(Node):
    def __init__(self):
        super().__init__("frame_logger")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.br = CvBridge(); self.lock = threading.Lock()
        self.imgs = {}; self.det = None; self.traj = None; self.cmd = None
        self.create_subscription(Image, "/image_raw", self.on_img, 10)
        self.create_subscription(DetectionArray, "/detections", lambda m: self._set("det", m), q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: self._set("traj", m), q)
        self.create_subscription(MotionCommand, "/topic_control_signal", lambda m: self._set("cmd", m), q)
        self.f = open(OUT + "/frames.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["idx", "wall_time", "file", "img_stamp", "det_stamp", "det_age_ms", "n_solid", "n_dashed", "n_other",
                         "solid_max_score", "dashed_max_score", "other_classes", "traj_valid", "a", "b", "c", "x_min_m", "x_max_m",
                         "fit_error_m", "traj_age_ms", "steer", "left_pwm", "right_pwm", "board_adc", "board_target", "board_state"])
        self.idx = 0
        self.create_timer(1.0 / HZ, self.tick)

    def _set(self, k, m):
        with self.lock: setattr(self, k, m)

    def on_img(self, m):
        with self.lock:
            self.imgs[round(stamp(m.header), 6)] = m
            if len(self.imgs) > 15:
                for k in sorted(self.imgs)[:-15]: del self.imgs[k]

    def tick(self):
        with self.lock:
            if not self.imgs: return
            det, traj, cmd = self.det, self.traj, self.cmd
            m = self.imgs.get(round(stamp(det.header), 6)) if det is not None else None
            m = m or self.imgs[max(self.imgs)]          # 추론한 바로 그 프레임, 없으면 최신
        img = self.br.imgmsg_to_cv2(m, "bgr8"); now = time.time(); ist = stamp(m.header)
        name = "%06d_%s.%03d.png" % (self.idx, time.strftime("%H%M%S", time.localtime(now)), int(now * 1000) % 1000)
        if RAW: cv2.imwrite(OUT + "/raw/" + name, img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        vis = img.copy(); over = img.copy(); counts = {}; best = {}
        if det is not None:
            for d in det.detections:
                counts[d.class_name] = counts.get(d.class_name, 0) + 1
                best[d.class_name] = max(best.get(d.class_name, 0.0), d.score)
                if d.mask.data:
                    pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
                    cv2.fillPoly(over, [pts], COL.get(d.class_name, (0, 0, 255)))
                    cv2.putText(vis, "%s %.2f" % (d.class_name, d.score), tuple(int(v) for v in pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        vis = cv2.addWeighted(over, 0.4, vis, 0.6, 0)
        h, w = img.shape[:2]
        src = np.float32(IPM["ipm_src_points"]).reshape(4, 2); src[:, 0] *= w / IPM["ipm_src_base_width"]; src[:, 1] *= h / IPM["ipm_src_base_height"]
        cv2.polylines(vis, [src.astype(np.int32)], True, (0, 255, 255), 1)
        if traj is not None and traj.valid:
            bh = int(round(h * IPM["bev_height_scale"])); lat, lon = IPM["lateral_meter_per_pixel"], IPM["longitudinal_meter_per_pixel"]
            M = cv2.getPerspectiveTransform(src, np.float32([[w * IPM["ipm_dst_left_ratio"], 0], [w * IPM["ipm_dst_right_ratio"], 0],
                                                             [w * IPM["ipm_dst_right_ratio"], bh - 1], [w * IPM["ipm_dst_left_ratio"], bh - 1]]))
            xs = np.linspace(traj.x_min_m, traj.x_max_m, 30); ys = traj.a * xs ** 2 + traj.b * xs + traj.c
            bp = np.stack([(w - 1) * 0.5 - ys / lat, bh - 1 - xs / lon], 1).astype(np.float32)
            ip = cv2.perspectiveTransform(bp.reshape(-1, 1, 2), np.linalg.inv(M)).reshape(-1, 2)
            cv2.polylines(vis, [ip.astype(np.int32)], False, (0, 0, 255), 3)
        adc, tgt, bstate = board_last()
        line1 = "#%d %s  cmd %s  path %s" % (self.idx, time.strftime("%H:%M:%S", time.localtime(now)),
                                             ("s%d l%d r%d" % (cmd.steering, cmd.left_speed, cmd.right_speed)) if cmd else "-",
                                             ("c=%+.2f b=%+.2f" % (traj.c, traj.b)) if (traj and traj.valid) else "INVALID")
        line2 = "solid %d dashed %d  steerADC %s->%s %s" % (counts.get("solid_line", 0), counts.get("dashed_line", 0), adc, tgt, bstate)
        cv2.rectangle(vis, (0, 0), (w, 40), (0, 0, 0), -1)
        cv2.putText(vis, line1, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(vis, line2, (5, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.imwrite(OUT + "/frames/" + name, vis, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        dst = stamp(det.header) if det is not None else None
        tst = time.time()
        self.w.writerow([self.idx, "%.3f" % now, name, "%.3f" % ist, "%.3f" % dst if dst else "",
                         "%.0f" % ((now - dst) * 1000) if dst else "", counts.get("solid_line", 0), counts.get("dashed_line", 0),
                         sum(v for k, v in counts.items() if k not in COL), "%.3f" % best.get("solid_line", 0), "%.3f" % best.get("dashed_line", 0),
                         ";".join("%s:%d" % kv for kv in counts.items() if kv[0] not in COL),
                         int(bool(traj and traj.valid)), *(("%.4f" % traj.a, "%.4f" % traj.b, "%.4f" % traj.c, "%.3f" % traj.x_min_m,
                                                          "%.3f" % traj.x_max_m, "%.4f" % traj.fit_error_m) if traj else ("",) * 6),
                         "%.0f" % ((now - stamp(traj.header)) * 1000) if traj else "",
                         cmd.steering if cmd else "", cmd.left_speed if cmd else "", cmd.right_speed if cmd else "", adc, tgt, bstate])
        self.idx += 1
        if self.idx % 10 == 0: self.f.flush()

rclpy.init(); n = Logger()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    n.f.close(); print("frames:", n.idx, "->", OUT)
