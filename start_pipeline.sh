#!/usr/bin/env bash
# 컨테이너 안에서 실행: 웹캠 -> YOLO(best.pt) -> 선 라벨 lane 폴리곤 -> LaneTrajectory -> Stanley(벤치 설정)
# serial_sender_node 는 여기서 실행하지 않는다(모터 명령 송신 없음).
set -e
source /opt/ros/humble/setup.bash
source ~/team_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 YOLO_CONFIG_DIR=/tmp/Ultralytics
PACK=~/jacapdi-live-car-2026-09-24
D=~/jacapdi-docker
L=/tmp/pipeline; mkdir -p $L
DEV=${WEBCAM_DEV:-$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)}
[ -n "$DEV" ] || { echo "C920 not found"; exit 1; }
echo "webcam: $DEV"
VENV_PY=~/.venvs/jacapdi/bin/python3

if ros2 node list 2>/dev/null | grep -q serial_sender; then echo "serial_sender 실행 중 - 중단"; exit 1; fi

python3 $D/webcam_pub.py $DEV /image_raw 640 480 > $L/webcam.log 2>&1 &
sleep 2
$VENV_PY -c 'from camera_perception_pkg.yolov8_node import main; main()' --ros-args \
  -p model:=$HOME/models/best.pt -p device:=cuda:0 -p imgsz:=640 -p frame_skip:=1 \
  -p half:=true -p threshold:=0.5 -p image_topic:=/image_raw -p image_reliability:=1 -p use_sim_time:=false > $L/yolo.log 2>&1 &
python3 $D/line_path_node.py --ros-args -p use_sim_time:=false \
  -p mode:=${PATH_MODE:-solid_follow} -p online_width:=true -p hold_last_sec:=${HOLD_LAST_SEC:-1.0} \
  -p min_rows:=0.06 -p max_gap:=0.40 > $L/line_path.log 2>&1 &
[ "${PATH_MODE:-solid_follow}" = solid_follow ] || ros2 run camera_perception_pkg lane_info_extractor_node --ros-args --params-file $D/ipm_c920.yaml \
  > $L/lane_info.log 2>&1 &
ros2 run decision_making_pkg stanley_controller_node --ros-args \
  --params-file ${CTRL_YAML:-$D/drive_controller.yaml} > $L/stanley.log 2>&1 &
echo "started; logs in $L"
wait
