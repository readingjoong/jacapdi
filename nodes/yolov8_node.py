# -*- coding: utf-8 -*-
"""YOLO 선 검출: /image_raw → /detections (검출마다 클래스, 확신도, 마스크 테두리 점).
models/best.pt (분할 모델), GPU FP16, 입력 640. 원본: yolov8_ros (Miguel Ángel González Santamarta, GPL-3).
사용 (venv 파이썬, pipeline.sh 가 띄움): ~/.venvs/jacapdi/bin/python3 nodes/yolov8_node.py [확신도 기준=0.3]"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from interfaces_pkg.msg import Point2D, Detection, DetectionArray

MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "best.pt")
DEVICE = "cuda:0"
IMGSZ = 640
THRESHOLD = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 0.3


class Yolov8Node(Node):
    def __init__(self):
        super().__init__("yolov8_node")
        self.yolo = YOLO(MODEL)
        self.yolo.fuse()
        self.bridge = CvBridge()
        self.pub = self.create_publisher(DetectionArray, "/detections", 10)
        q = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                       durability=QoSDurabilityPolicy.VOLATILE, depth=1)   # 최신 사진 하나만
        self.create_subscription(Image, "/image_raw", self.image_cb, q)
        self.get_logger().info("model %s, threshold %.2f, imgsz %d, %s FP16" % (MODEL, THRESHOLD, IMGSZ, DEVICE))

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        res = self.yolo.predict(source=img, verbose=False, stream=False, conf=THRESHOLD, imgsz=IMGSZ,
                                quantize=16, device=DEVICE)[0].cpu()        # quantize=16 = FP16 (ultralytics 8.4)
        out = DetectionArray(); out.header = msg.header
        h, w = res.orig_img.shape[:2]
        cls = res.boxes.cls.int().tolist() if res.boxes else None
        conf = res.boxes.conf.tolist() if res.boxes else None
        xy = res.masks.xy if res.masks else None
        for i in range(len(res)):
            d = Detection()
            if cls is not None:
                d.class_id = cls[i]; d.class_name = self.yolo.names[cls[i]]; d.score = conf[i]
            if xy is not None:
                d.mask.height, d.mask.width = h, w
                d.mask.data = [Point2D(x=x, y=y) for x, y in xy[i].tolist()]
            out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Yolov8Node()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
