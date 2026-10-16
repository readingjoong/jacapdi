# -*- coding: utf-8 -*-
"""start_ab.sh 가 같이 띄우는 기록기: 경로/명령/검출을 10 Hz 로 CSV 에 남긴다 (<기록폴더>/timeline.csv).
경로 유효, 계수 a b c, 조향/속도 명령, 실선/점선 검출 수, YOLO 결과 속도를 본다. 주행에는 관여하지 않는다."""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand


def main():
    out = sys.argv[1]
    rclpy.init(); n = Node("ab_run_monitor")
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    st = {"traj": None, "cmd": None, "det": (0, 0), "n_det": 0}
    n.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: st.__setitem__("traj", m), q)
    n.create_subscription(MotionCommand, "/topic_control_signal", lambda m: st.__setitem__("cmd", m), q)

    def on_det(m):
        s = sum(d.class_name == "solid_line" for d in m.detections)
        d_ = sum(d.class_name == "dashed_line" for d in m.detections)
        st["det"] = (s, d_); st["n_det"] += 1
    n.create_subscription(DetectionArray, "/detections", on_det, q)
    f = open(out + "/timeline.csv", "w", newline=""); w = csv.writer(f)
    w.writerow(["t", "traj_valid", "a", "b", "c", "steer", "speed", "n_solid", "n_dashed", "det_hz"])
    t0 = time.time(); last = t0; last_n = 0; hz = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            if now - last >= 0.1:
                hz = 0.8 * hz + 0.2 * ((st["n_det"] - last_n) / (now - last)); last_n = st["n_det"]; last = now
                t, c = st["traj"], st["cmd"]
                w.writerow(["%.2f" % (now - t0), int(bool(t and t.valid)),
                            "%.4f" % t.a if t else "", "%.4f" % t.b if t else "", "%.4f" % t.c if t else "",
                            c.steering if c else "", c.left_speed if c else "", st["det"][0], st["det"][1], "%.1f" % hz])
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
