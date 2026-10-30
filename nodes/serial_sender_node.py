# -*- coding: utf-8 -*-
"""/topic_control_signal → 아두이노 시리얼 "s{조향}l{왼쪽}r{오른쪽}\\n" (115200). 끝날 때 s0l0r0.
car_prep.sh 가 띄우고, 없으면 car_go.sh 가 다시 띄운다.
board_monitor.py 가 포트를 먼저 열어 두므로 여기서 열어도 아두이노가 리셋되지 않는다 (기다릴 필요 없음)."""
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand

PORT = "/dev/ttyACM0"   # 컨테이너 안에서는 항상 ttyACM0 (car.sh container_up 이 맞춰 줌)
BAUD = 115200


def line(steer, left, right):
    return ("s%dl%dr%d\n" % (steer, left, right)).encode()


class SerialSender(Node):
    def __init__(self):
        super().__init__("serial_sender_node")
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        q = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                       durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(MotionCommand, "/topic_control_signal",
                                 lambda m: self.ser.write(line(m.steering, m.left_speed, m.right_speed)), q)


def main():
    rclpy.init()
    n = SerialSender()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():                            # 주행 중 오류는 그대로, 끄는 중(Ctrl+C 로 ROS 가 먼저 닫힘)에 난 것은 무시
            raise
    finally:
        n.ser.write(line(0, 0, 0)); n.ser.close()
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
