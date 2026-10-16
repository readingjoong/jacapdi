"""파이프라인 각 토픽의 실제 수신 Hz와 지연(수신시각 - header.stamp)을 5초간 측정."""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand
rclpy.init(); n = Node("rate_probe")
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
st = {}
def cb(name, stamped=True):
    def f(m):
        s = st.setdefault(name, {"n": 0, "lat": [], "valid": 0})
        s["n"] += 1
        if stamped:
            s["lat"].append((n.get_clock().now().nanoseconds - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e6)
        if hasattr(m, "valid"): s["valid"] += int(m.valid)
        if hasattr(m, "left_speed"): s.setdefault("cmds", {}).setdefault("s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed), 0); s["cmds"]["s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed)] += 1
    return f
n.create_subscription(Image, "/image_raw", cb("/image_raw"), 10)
n.create_subscription(DetectionArray, "/detections", cb("/detections"), q)
n.create_subscription(DetectionArray, "/lane_detections", cb("/lane_detections"), q)
n.create_subscription(LaneTrajectory, "/lane_trajectory", cb("/lane_trajectory"), q)
n.create_subscription(MotionCommand, "/topic_control_signal", cb("/topic_control_signal", False), q)
T = 5.0; end = time.time() + T
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
for k in ["/image_raw", "/detections", "/lane_detections", "/lane_trajectory", "/topic_control_signal"]:
    s = st.get(k)
    if not s: print("%-22s 0 msgs" % k); continue
    lat = sorted(s["lat"]); extra = ""
    if lat: extra += " latency median %.0f ms, max %.0f ms" % (lat[len(lat)//2], lat[-1])
    if k == "/lane_trajectory": extra += " valid %d/%d" % (s["valid"], s["n"])
    if "cmds" in s: extra += " cmds %s" % s["cmds"]
    print("%-22s %5.1f Hz%s" % (k, s["n"] / T, extra))
