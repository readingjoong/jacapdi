"""라이브 /detections 를 받아 (a) 그대로 (b) solid 제거 (c) dashed 제거 세 가지를 테스트용 line_path 노드에 넣고 경로 비교."""
import subprocess, time, os, rclpy, copy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray, LaneTrajectory
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
procs = []
for tag in ("all", "nosolid", "nodashed"):
    procs.append(subprocess.Popen(["python3", os.path.expanduser("~/jacapdi-docker/line_path_node.py"), "--ros-args",
        "-r", "__node:=lp_" + tag, "-p", "mode:=solid_follow", "-p", "sub_detection_topic:=/t_%s/det" % tag,
        "-p", "pub_topic:=/t_%s/lane" % tag, "-p", "pub_trajectory_topic:=/t_%s/traj" % tag, "-p", "image_topic:=/t_none",
        "-p", "max_image_age_sec:=5.0"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
rclpy.init(); n = Node("test_dashed_only")
pubs = {t: n.create_publisher(DetectionArray, "/t_%s/det" % t, q) for t in ("all", "nosolid", "nodashed")}
res = {t: [] for t in pubs}
for t in pubs: n.create_subscription(LaneTrajectory, "/t_%s/traj" % t, lambda m, t=t: res[t].append(m), q)
dets = []; n.create_subscription(DetectionArray, "/detections", dets.append, q)
time.sleep(3)
end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.02)
    if dets:
        d = dets.pop(); dets.clear()
        pubs["all"].publish(d)
        a = copy.deepcopy(d); a.detections = [x for x in d.detections if x.class_name != "solid_line"]; pubs["nosolid"].publish(a)
        b = copy.deepcopy(d); b.detections = [x for x in d.detections if x.class_name != "dashed_line"]; pubs["nodashed"].publish(b)
for t, v in res.items():
    ok = [m for m in v if m.valid]
    if ok:
        m = ok[-1]; import numpy as np
        cs = np.array([x.c for x in ok]); bs = np.array([x.b for x in ok])
        print("%-9s valid %3d/%3d  c(0m) median %+.3f m  b median %+.3f  x=[%.2f,%.2f]" % (t, len(ok), len(v), np.median(cs), np.median(bs), m.x_min_m, m.x_max_m))
    else:
        print("%-9s valid 0/%d" % (t, len(v)))
for p in procs: p.terminate()
