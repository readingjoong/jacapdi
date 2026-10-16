"""/image_raw 한 장 + 같은 시각 /detections(선 마스크)를 저장. 출력: /tmp/calib/frame.jpg, lines.png, dets.json"""
import json, os, time
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray
os.makedirs("/tmp/calib", exist_ok=True)
rclpy.init(); n = Node("grab_frame"); b = CvBridge()
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
imgs, dets = {}, []
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
n.create_subscription(Image, "/image_raw", lambda m: imgs.__setitem__(st(m.header), b.imgmsg_to_cv2(m, "bgr8")), 30)
n.create_subscription(DetectionArray, "/detections", lambda m: dets.append(m), q)
end = time.time() + 4
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
pair = next(((imgs[st(d.header)], d) for d in reversed(dets) if st(d.header) in imgs), None)
img, d = pair
h, w = img.shape[:2]
out = {"w": w, "h": h, "dets": []}
vis = img.copy()
for x in d.detections:
    pts = [[p.x, p.y] for p in x.mask.data]
    out["dets"].append({"cls": x.class_name, "score": x.score, "pts": pts})
    if pts:
        c = (255, 0, 0) if x.class_name == "solid_line" else (0, 255, 0) if x.class_name == "dashed_line" else (0, 0, 255)
        cv2.polylines(vis, [np.array(pts, np.int32)], True, c, 2)
cv2.imwrite("/tmp/calib/frame.jpg", img); cv2.imwrite("/tmp/calib/lines.png", vis)
json.dump(out, open("/tmp/calib/dets.json", "w"))
print("frame", w, h, "dets:", [(x["cls"], round(x["score"], 2)) for x in out["dets"]])
