"""같은 시각의 /image_raw + /detections + /lane_detections + /lane_trajectory 를 저장 (진단용)."""
import json, os, time, cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, LaneTrajectory
os.makedirs("/tmp/diag", exist_ok=True)
rclpy.init(); n = Node("grab_lane"); b = CvBridge()
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
imgs, dets, lanes, trajs = {}, {}, {}, {}
n.create_subscription(Image, "/image_raw", lambda m: imgs.__setitem__(st(m.header), b.imgmsg_to_cv2(m, "bgr8")), 30)
n.create_subscription(DetectionArray, "/detections", lambda m: dets.__setitem__(st(m.header), m), q)
n.create_subscription(DetectionArray, "/lane_detections", lambda m: lanes.__setitem__(st(m.header), m), q)
n.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: trajs.__setitem__(st(m.header), m), q)
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
k = max(set(imgs) & set(dets) & set(lanes) & set(trajs))
t = trajs[k]
out = {"traj": {f: getattr(t, f) for f in ("valid", "a", "b", "c", "x_min_m", "x_max_m", "fit_error_m", "num_points")},
       "lines": [{"cls": d.class_name, "pts": [[p.x, p.y] for p in d.mask.data]} for d in dets[k].detections],
       "lane": [[[p.x, p.y] for p in d.mask.data] for d in lanes[k].detections]}
cv2.imwrite("/tmp/diag/frame.jpg", imgs[k]); json.dump(out, open("/tmp/diag/diag.json", "w"))
print(out["traj"]); print("lines:", [d["cls"] for d in out["lines"]], "lane polys:", len(out["lane"]))
