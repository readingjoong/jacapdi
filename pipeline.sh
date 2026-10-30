#!/usr/bin/env bash
# 주행 파이프라인 (컨테이너 안에서 실행, car_prep.sh 가 띄움). 모터 명령은 출발 스위치(tune/go_until)가 켜져야 나간다.
#   웹캠 -> YOLO(models/best.pt) -> 경로(lane_path_node) -> 제어기(tunable_controller)  + 기록기(run_monitor)
#   시리얼 송신기(serial_sender_node)와 보드 모니터는 car_prep.sh 가 따로 띄운다.
# 사용: RUN=<기록폴더> CONF=0.3 bash ~/jacapdi-docker/pipeline.sh       종료: Ctrl+C -> 요약 + ab_runs/results.csv 에 한 줄
#       source ~/jacapdi-docker/pipeline.sh env                         ROS 환경만 켜기 (car.sh 의 docker exec, 컨테이너 bashrc)

# ROS 환경. 메시지(interfaces_pkg)는 ros/ 에서 빌드한 것 (car.sh container_up 이 처음 한 번 빌드)
source /opt/ros/humble/setup.bash
[ -f ~/jacapdi-docker/ros/install/setup.bash ] && source ~/jacapdi-docker/ros/install/setup.bash
export ROS_LOCALHOST_ONLY=1 YOLO_CONFIG_DIR=/tmp/Ultralytics PYTHONUNBUFFERED=1
[ "$1" = env ] && return 0

set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; N=$D/nodes
CONF="${CONF:-0.3}"
RUN="${RUN:-$D/ab_runs/$(date +%m%d_%H%M%S)_conf$CONF}"; mkdir -p "$RUN"
DEV=${WEBCAM_DEV:-$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)}
[ -n "$DEV" ] || { echo "C920 없음 (WEBCAM_DEV=/dev/videoN 로 지정 가능)"; exit 1; }
if pgrep -f "nodes/(yolov8_node|lane_path_node|tunable_controller)\.py" >/dev/null; then
  echo "이미 실행 중인 노드가 있음 - 먼저 끄고 다시 (car.sh stop all)"; pgrep -af "nodes/"; exit 1
fi
{ echo "tag=$(basename "$RUN")"; echo "algo=lane_path_node md5=$(md5sum "$N/lane_path_node.py" | cut -d' ' -f1)"
  echo "conf=$CONF"; echo "webcam=$DEV"; date; } > "$RUN/run_info.txt"
cp "$D/tune/tune.json" "$RUN/tune_at_start.json"
cat "$RUN/run_info.txt"

PIDS=()
python3 "$N/webcam_pub.py" "$DEV" /image_raw 640 480 > "$RUN/webcam.log" 2>&1 & PIDS+=($!)
sleep 2
~/.venvs/jacapdi/bin/python3 "$N/yolov8_node.py" "$CONF" > "$RUN/yolo.log" 2>&1 & PIDS+=($!)
python3 "$N/lane_path_node.py" > "$RUN/line_path.log" 2>&1 & PIDS+=($!)
python3 "$N/tunable_controller.py" "$RUN" > "$RUN/tunable.log" 2>&1 & PIDS+=($!)
python3 "$N/run_monitor.py" "$RUN" > "$RUN/monitor.log" 2>&1 & PIDS+=($!)

finish() {
  trap - INT TERM
  echo; echo "== 종료 중 ($RUN)"
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 2
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  python3 "$N/summarize_run.py" "$RUN" | tee "$RUN/summary.txt"
  exit 0
}
trap finish INT TERM
echo "== 시작 (기록 $RUN). 끝낼 때 Ctrl+C"
wait
