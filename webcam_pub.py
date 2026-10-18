"""USB 웹캠 -> sensor_msgs/Image 발행 (팀 코드 수정 없이 시험용).
사용: python3 webcam_pub.py [device] [topic] [width] [height] [fps=30]"""
import os, sys, time
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
topic = sys.argv[2] if len(sys.argv) > 2 else "/webcam/image_raw"
W = int(sys.argv[3]) if len(sys.argv) > 3 else 640
H = int(sys.argv[4]) if len(sys.argv) > 4 else 480
FPS = int(sys.argv[5]) if len(sys.argv) > 5 else 30   # [1018] 다시 30 (20 으로도 3.5~4분 뒤 느려짐은 그대로). 20 으로 하려면 다섯째 인자 20

class WebcamPub(Node):
    def __init__(self):
        super().__init__("webcam_pub")
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FPS, FPS); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit("camera open failed: " + dev)
        self.pub = self.create_publisher(Image, topic, 10)  # yolov8_node 기본 QoS(reliable)와 호환
        self.bridge = CvBridge()
        self.create_timer(1.0 / FPS, self.tick)
        # [1018] 실제로 보낸 장수를 1초마다 파일에 적음 -> run_monitor 가 timeline.csv 의 cam_hz 로 기록 (사진을 다시 구독하지 않으려고)
        self.n_ok = self.n_fail = 0; self.t_rate = time.time()
        os.makedirs("/tmp/pipeline", exist_ok=True)
        self.get_logger().info("publishing %s from %s at %dx%d, %d fps (camera reports %.0f)" % (topic, dev, W, H, FPS, self.cap.get(cv2.CAP_PROP_FPS)))

    def tick(self):
        ok, f = self.cap.read()
        now = time.time()
        if now - self.t_rate >= 1.0:
            try:
                with open("/tmp/pipeline/cam_hz.tmp", "w") as fp: fp.write("%.1f %d %.3f" % (self.n_ok / (now - self.t_rate), self.n_fail, now))
                os.replace("/tmp/pipeline/cam_hz.tmp", "/tmp/pipeline/cam_hz")
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
