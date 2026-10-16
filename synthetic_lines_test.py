"""카메라 없이 연결부 검증: 합성 dashed_line(왼쪽)+solid_line(오른쪽) DetectionArray 를 /detections 로 10Hz 발행.
중간에 선을 끊어 invalid -> 정지(l0 r0) 전환도 본다. 모터/시리얼 없음."""
import time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory, MotionCommand

W, H = 640, 480
def poly(xs_ys):
    m = []
    for x, y in xs_ys:
        p = Point2D(); p.x = float(x); p.y = float(y); m.append(p)
    return m

def lines(shift):
    dets = []
    # 오른쪽 실선: 아래 x=520 -> 위 x=390 (원근), 폭 8px
    pts = [(520 + shift - (480 - y) * 0.55, y) for y in range(479, 200, -10)]
    d = Detection(); d.class_name = "solid_line"; d.mask.width, d.mask.height = W, H
    d.mask.data = poly([(x - 4, y) for x, y in pts] + [(x + 4, y) for x, y in reversed(pts)]); dets.append(d)
    # 왼쪽 점선 조각
    for y0 in range(479, 220, -60):
        y1 = y0 - 30
        xa, xb = 120 + shift + (480 - y0) * 0.55, 120 + shift + (480 - y1) * 0.55
        d = Detection(); d.class_name = "dashed_line"; d.mask.width, d.mask.height = W, H
        d.mask.data = poly([(xa - 4, y0), (xa + 4, y0), (xb + 4, y1), (xb - 4, y1)]); dets.append(d)
    return dets

class T(Node):
    def __init__(self):
        super().__init__("synthetic_lines_test")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(DetectionArray, "/detections", q)
        self.traj, self.cmds = [], []
        self.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: self.traj.append((self.phase, m)), q)
        self.create_subscription(MotionCommand, "/topic_control_signal", lambda m: self.cmds.append((self.phase, m)), q)
        self.phase = "none"

rclpy.init(); n = T()
t0 = time.time()
for phase, shift, dur in [("center", 0, 3.0), ("shift_right_60px", 60, 3.0), ("no_lines", None, 2.0)]:
    n.phase = phase; end = time.time() + dur
    while time.time() < end:
        m = DetectionArray(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = "synthetic"
        if shift is not None: m.detections = lines(shift)
        n.pub.publish(m)
        e2 = time.time() + 0.1
        while time.time() < e2: rclpy.spin_once(n, timeout_sec=0.02)
for phase in ("center", "shift_right_60px", "no_lines"):
    tr = [m for p, m in n.traj if p == phase]; cm = [m for p, m in n.cmds if p == phase]
    v = [m for m in tr if m.valid]
    s = "%s: trajectory %d (valid %d)" % (phase, len(tr), len(v))
    if v: s += " last c=%.3fm b=%.3f x=[%.2f,%.2f]" % (v[-1].c, v[-1].b, v[-1].x_min_m, v[-1].x_max_m)
    if cm: s += " | cmds %d, last s%d l%d r%d, steering set %s" % (len(cm), cm[-1].steering, cm[-1].left_speed, cm[-1].right_speed, sorted({c.steering for c in cm}))
    print(s)
n.destroy_node(); rclpy.shutdown()
