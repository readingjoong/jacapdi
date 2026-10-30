# -*- coding: utf-8 -*-
"""파이프라인 토픽마다 5 초 동안 받은 Hz, 지연(받은 시각 - header.stamp), 경로 valid 비율, 명령 종류를 출력.
사용: python3 rate_probe.py   (car_prep.sh 준비 확인, car.sh lap 차선 인식 확인)"""
import time
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

T = 5.0
TOPICS = [("/image_raw", Image), ("/detections", DetectionArray), ("/lane_detections", DetectionArray),
          ("/lane_trajectory", LaneTrajectory), ("/topic_control_signal", MotionCommand)]

rclpy.init(); n = Node("rate_probe")
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
st = {k: {"n": 0, "lat": [], "valid": 0, "cmds": Counter()} for k, _ in TOPICS}


def cb(name):
    s = st[name]

    def f(m):
        s["n"] += 1
        if hasattr(m, "header"):
            s["lat"].append((n.get_clock().now().nanoseconds - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e6)
        if hasattr(m, "valid"): s["valid"] += int(m.valid)
        if hasattr(m, "left_speed"): s["cmds"]["s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed)] += 1
    return f


for k, typ in TOPICS:
    n.create_subscription(typ, k, cb(k), 10 if typ is Image else q)
end = time.time() + T
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
for k, _ in TOPICS:
    s = st[k]
    if not s["n"]: print("%-22s 0 msgs" % k); continue
    lat = sorted(s["lat"]); extra = ""
    if lat: extra += " latency median %.0f ms, max %.0f ms" % (lat[len(lat) // 2], lat[-1])
    if k == "/lane_trajectory": extra += " valid %d/%d" % (s["valid"], s["n"])
    if s["cmds"]: extra += " cmds %s" % dict(s["cmds"])
    print("%-22s %5.1f Hz%s" % (k, s["n"] / T, extra))
