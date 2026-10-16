"""USB 웹캠 -> sensor_msgs/Image 발행 (팀 코드 수정 없이 시험용).
사용: python3 webcam_pub.py [device] [topic] [width] [height]"""
import sys
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
topic = sys.argv[2] if len(sys.argv) > 2 else "/webcam/image_raw"
W = int(sys.argv[3]) if len(sys.argv) > 3 else 640
H = int(sys.argv[4]) if len(sys.argv) > 4 else 480

class WebcamPub(Node):
    def __init__(self):
        super().__init__("webcam_pub")
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FPS, 30); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit("camera open failed: " + dev)
        self.pub = self.create_publisher(Image, topic, 10)  # yolov8_node 기본 QoS(reliable)와 호환
        self.bridge = CvBridge()
        self.create_timer(1.0 / 30, self.tick)
        self.get_logger().info("publishing %s from %s at %dx%d" % (topic, dev, W, H))

    def tick(self):
        ok, f = self.cap.read()
        if not ok:
            return
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
