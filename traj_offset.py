"""경로를 옆으로 옮기는 중계 노드 (경로 알고리즘 코드는 그대로).
/lane_trajectory -> c 에 OFFSET(m, +는 왼쪽)을 더해 /lane_trajectory_offset 으로 발행. valid=false 는 그대로 전달.
사용: python3 traj_offset.py [offset_m]  (기본 0.10)"""
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces_pkg.msg import LaneTrajectory
OFF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
rclpy.init(); n = Node("traj_offset")
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE, depth=1)
pub = n.create_publisher(LaneTrajectory, "/lane_trajectory_offset", q)
def cb(m):
    if m.valid: m.c = m.c + OFF
    pub.publish(m)
n.create_subscription(LaneTrajectory, "/lane_trajectory", cb, q)
n.get_logger().info("offset %+.3f m (left +)" % OFF)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
