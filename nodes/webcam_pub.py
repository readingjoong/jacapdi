# -*- coding: utf-8 -*-
"""USB 웹캠 → /image_raw (bgr8, MJPG 640x480 30 fps). 1 초마다 실제로 보낸 장수와 읽기 실패 수를
/tmp/pipeline/cam_hz 에 쓴다 (run_monitor 가 timeline.csv 에 기록. 사진을 다시 구독하지 않으려고 파일로).
사용: python3 webcam_pub.py <장치> [토픽=/image_raw] [가로=640] [세로=480] [fps=30]   (pipeline.sh 가 띄움)"""
import os, sys, time
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEV = sys.argv[1]
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/image_raw"
W = int(sys.argv[3]) if len(sys.argv) > 3 else 640
H = int(sys.argv[4]) if len(sys.argv) > 4 else 480
FPS = int(sys.argv[5]) if len(sys.argv) > 5 else 30
HZ_FILE = "/tmp/pipeline/cam_hz"


class WebcamPub(Node):
    def __init__(self):
        super().__init__("webcam_pub")
        self.cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FPS, FPS); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 버퍼 1장 = 늘 최신 사진
        if not self.cap.isOpened():
            raise SystemExit("camera open failed: " + DEV)
        self.pub = self.create_publisher(Image, TOPIC, 10)
        self.bridge = CvBridge()
        self.n_ok = self.n_fail = 0; self.t_rate = time.time()
        os.makedirs(os.path.dirname(HZ_FILE), exist_ok=True)
        self.create_timer(1.0 / FPS, self.tick)
        self.get_logger().info("publishing %s from %s at %dx%d, %d fps (camera reports %.0f)" % (TOPIC, DEV, W, H, FPS, self.cap.get(cv2.CAP_PROP_FPS)))

    def tick(self):
        ok, f = self.cap.read()
        now = time.time()
        if now - self.t_rate >= 1.0:
            try:
                with open(HZ_FILE + ".tmp", "w") as fp: fp.write("%.1f %d %.3f" % (self.n_ok / (now - self.t_rate), self.n_fail, now))
                os.replace(HZ_FILE + ".tmp", HZ_FILE)
            except OSError:
                pass
            self.n_ok = self.n_fail = 0; self.t_rate = now
        if not ok:
            self.n_fail += 1
            return
        self.n_ok += 1
        msg = self.bridge.cv2_to_imgmsg(f, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "webcam"
        self.pub.publish(msg)


def main():
    rclpy.init(); n = WebcamPub()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.cap.release(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
