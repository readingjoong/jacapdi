#!/usr/bin/env bash
# 시간 제한 주행: serial_sender 를 DRIVE_SEC(+부팅 대기) 동안만 실행하고, 끝나면 s0l0r0 을 여러 번 보낸다.
source /opt/ros/humble/setup.bash; source ~/team_ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1
PORT=${PORT:-/dev/ttyACM0}; DRIVE_SEC=${DRIVE_SEC:-5}; BOOT_SEC=3
L=/tmp/pipeline; mkdir -p $L
# 주행 중 명령 기록
python3 - > $L/drive_cmds.log 2>&1 <<PY &
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import MotionCommand, LaneTrajectory
rclpy.init(); n = Node("drive_recorder"); q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
t0 = time.time(); c = {}; tv = [0, 0]
def cb(m):
    if time.time() - t0 < $BOOT_SEC: return
    k = "s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed); c[k] = c.get(k, 0) + 1
def tr(m):
    if time.time() - t0 < $BOOT_SEC: return
    tv[0] += 1; tv[1] += int(m.valid)
n.create_subscription(MotionCommand, "/topic_control_signal", cb, q); n.create_subscription(LaneTrajectory, "/lane_trajectory", tr, q)
while time.time() - t0 < $BOOT_SEC + $DRIVE_SEC: rclpy.spin_once(n, timeout_sec=0.05)
print("commands during drive window:", c); print("trajectory valid %d/%d" % (tv[1], tv[0]))
PY
REC=$!
echo "$(date +%T) serial_sender start ($PORT), drive ${DRIVE_SEC}s after ${BOOT_SEC}s boot"
timeout -s INT $((BOOT_SEC + DRIVE_SEC)) ros2 run serial_communication_pkg serial_sender_node --ros-args \
  -p port:=$PORT -p baud_rate:=115200 -p sub_topic:=/topic_control_signal -p use_sim_time:=false > $L/serial.log 2>&1
echo "$(date +%T) serial_sender stopped (exit $?)"
pkill -f "ros-args -p port:=$PORT" 2>/dev/null
sleep 0.5
python3 -c "
import serial, time
s = serial.Serial('$PORT', 115200, timeout=0.2)
for _ in range(5): s.write(b's0l0r0\n'); time.sleep(0.05)
time.sleep(0.3); print('stop sent; board says:', repr(s.read(512).decode(errors='replace')))
s.close()"
wait $REC; cat $L/drive_cmds.log; tail -3 $L/serial.log
