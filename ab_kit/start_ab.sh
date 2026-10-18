#!/usr/bin/env bash
# 실차 A/B 시험 파이프라인 (도커 컨테이너 안에서 실행). 기존 start_pipeline.sh 와 같은 구성에
# 경로 알고리즘(ALGO)과 YOLO threshold(CONF)만 골라 넣는다.
#   웹캠 -> YOLO(best.pt, threshold=CONF) -> 경로 생성(ALGO) -> Stanley(drive_controller.yaml)
# serial_sender_node 는 여기서 실행하지 않는다 (기존과 같음. 모터는 따로 켠다).
#
# 사용 예
#   ALGO=B CONF=0.5 bash ~/jacapdi-docker/ab_kit/start_ab.sh
#   ALGO=A CONF=0.3 RECORD=1 bash ~/jacapdi-docker/ab_kit/start_ab.sh     # 결과 토픽 기록
#   ALGO=C RECORD=2 bash ~/jacapdi-docker/ab_kit/start_ab.sh              # + 카메라 영상 기록 (1분 약 1.6 GB)
# ALGO: A 원본 / B 점선 대체 / C B+커브 보정 / D C+안정화      CONF 기본 0.5
# 종료: Ctrl+C -> 요약 출력 + ~/jacapdi-docker/ab_runs/results.csv 에 한 줄 추가
set -e
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGO="${ALGO:-A}"; CONF="${CONF:-0.5}"; RECORD="${RECORD:-0}"
case "$ALGO" in
  A) F=A_orig.py ;; B) F=B_dashed_fallback.py ;; C) F=C_curve_offset.py ;; D) F=D_stable.py ;; E) F=E_curve_cross.py ;; F) F=F_outlier_hold.py ;; G) F=G_cross_hold.py ;; H) F=H_boundary_fit.py ;;
  *) echo "ALGO 는 A~H 중 하나"; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
source ~/team_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 YOLO_CONFIG_DIR=/tmp/Ultralytics PYTHONUNBUFFERED=1
D=~/jacapdi-docker
TAG="$(date +%m%d_%H%M%S)_${ALGO}_conf${CONF}"
L="$D/ab_runs/$TAG"; mkdir -p "$L"
VENV_PY=~/.venvs/jacapdi/bin/python3
CTRL="${CTRL_YAML:-$D/drive_controller.yaml}"
DEV=${WEBCAM_DEV:-$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)}
[ -n "$DEV" ] || { echo "C920 없음 (WEBCAM_DEV=/dev/videoN 로 지정 가능)"; exit 1; }
if ros2 node list 2>/dev/null | grep -qE "serial_sender|line_path_node|yolov8_node"; then
  echo "이미 실행 중인 노드가 있음 (serial_sender/line_path/yolov8) - 먼저 끄고 다시"; ros2 node list; exit 1
fi
{
  echo "tag=$TAG"; echo "algo=$ALGO ($F) md5=$(md5sum "$KIT/algo/$F" | cut -d' ' -f1)"; echo "conf=$CONF"
  echo "controller=$CTRL"; grep -E "speed_command|max_steering_step|stanley_gain|heading_gain" "$CTRL" | sed 's/^ */  /'
  echo "ipm=$D/ipm_c920.yaml"; head -1 "$D/ipm_c920.yaml"; echo "webcam=$DEV"; date
} > "$L/run_info.txt"
cat "$L/run_info.txt"

PIDS=()
python3 "$D/webcam_pub.py" "$DEV" /image_raw 640 480 > "$L/webcam.log" 2>&1 & PIDS+=($!)
sleep 2
"$VENV_PY" -c 'from camera_perception_pkg.yolov8_node import main; main()' --ros-args \
  -p model:=$HOME/models/best.pt -p device:=cuda:0 -p imgsz:=640 -p frame_skip:=1 \
  -p half:=true -p threshold:=$CONF -p image_topic:=/image_raw -p image_reliability:=1 -p use_sim_time:=false \
  > "$L/yolo.log" 2>&1 & PIDS+=($!)
python3 "$KIT/algo/$F" --ros-args -p use_sim_time:=false \
  -p mode:=solid_follow -p online_width:=true -p hold_last_sec:=${HOLD_LAST_SEC:-1.0} \
  -p min_rows:=0.06 -p max_gap:=0.40 > "$L/line_path.log" 2>&1 & PIDS+=($!)
# [1018] TUNE=1 (조정형 제어기 사용) 이면 팀 Stanley 결과는 안 쓰는 토픽으로 가므로 아예 안 띄움 (CPU/발열 절약)
if grep -q "team_unused" "$CTRL"; then echo "팀 Stanley 안 띄움 (TUNE=1, tunable_controller 사용)" > "$L/stanley.log"
else ros2 run decision_making_pkg stanley_controller_node --ros-args --params-file "$CTRL" > "$L/stanley.log" 2>&1 & PIDS+=($!); fi
python3 "$KIT/run_monitor.py" "$L" > "$L/monitor.log" 2>&1 & PIDS+=($!)
if [ "$RECORD" -ge 1 ]; then
  TOPICS="/detections /lane_trajectory /lane_detections /topic_control_signal"
  [ "$RECORD" -ge 2 ] && TOPICS="$TOPICS /image_raw"
  ros2 bag record -o "$L/bag" $TOPICS > "$L/bag.log" 2>&1 & PIDS+=($!)
  echo "rosbag 기록: $L/bag ($TOPICS)"
fi

finish() {
  trap - INT TERM
  echo; echo "== 종료 중 ($TAG)"
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 2
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  python3 "$KIT/summarize_run.py" "$L" | tee "$L/summary.txt"
  echo "기록 폴더: $L"
  echo "결과 표에 사람이 적을 것: 완주 여부 / 멈춘 곳 / 선 밟음 / 메모 -> ~/jacapdi-docker/ab_kit/시험기록표.csv"
  exit 0
}
trap finish INT TERM
echo "== 시작: ALGO=$ALGO CONF=$CONF (기록 $L). 모터는 따로 켠다. 끝낼 때 Ctrl+C"
wait
