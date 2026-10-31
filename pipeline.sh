#!/usr/bin/env bash
# 주행 파이프라인 (컨테이너 안에서 실행, car_prep.sh 가 띄움). 모터 명령은 출발 스위치(tune/go_until)가 켜져야 나간다.
#   웹캠 -> YOLO(models/best.pt) -> 경로(lane_path_node) -> 제어기(tunable_controller)  + 기록기(run_monitor)
#   시리얼 송신기(serial_sender_node)와 보드 모니터는 car_prep.sh 가 따로 띄운다.
# 사용: RUN=<기록폴더> CONF=0.3 LANE=2 ALGO=base bash ~/jacapdi-docker/pipeline.sh    종료: Ctrl+C -> 요약 + ab_runs/results.csv 에 한 줄
#       LANE(기본 2)·ALGO(기본 base, 이름은 nodes/variant.py --check 로 검증) 를 tune/lane·tune/algo 에 쓰고 run_info.txt 에 기록. 미지정 = base
#       DRY=1  노드를 띄우기 직전에 멈춤 (웹캠·ROS 없이 run_info.txt 기록만 확인. tests/test_shell_static.sh)
#       source ~/jacapdi-docker/pipeline.sh env                         ROS 환경만 켜기 (car.sh 의 docker exec, 컨테이너 bashrc)

# ROS 환경. 메시지(interfaces_pkg)는 ros/ 에서 빌드한 것 (car.sh container_up 이 처음 한 번 빌드)
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f ~/jacapdi-docker/ros/install/setup.bash ] && source ~/jacapdi-docker/ros/install/setup.bash
export ROS_LOCALHOST_ONLY=1 YOLO_CONFIG_DIR=/tmp/Ultralytics PYTHONUNBUFFERED=1
[ "$1" = env ] && return 0

set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; N=$D/nodes
CONF="${CONF:-0.3}"
RUN="${RUN:-$D/ab_runs/$(date +%m%d_%H%M%S)_conf$CONF}"; mkdir -p "$RUN"
DRY="${DRY:-0}"
DEV=${WEBCAM_DEV:-$(for v in /sys/class/video4linux/video*; do [ -e "$v/index" ] && [ "$(cat $v/index)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done; true)}   # 카메라 없으면 빈 값 (set -e 에 안 걸리게 true)
[ -n "$DEV" ] || [ "$DRY" = 1 ] || { echo "C920 없음 (WEBCAM_DEV=/dev/videoN 로 지정 가능)"; exit 1; }
if [ "$DRY" != 1 ] && pgrep -f "nodes/(yolov8_node|lane_path_node|tunable_controller)\.py" >/dev/null; then
  echo "이미 실행 중인 노드가 있음 - 먼저 끄고 다시 (car.sh stop all)"; pgrep -af "nodes/"; exit 1
fi

# 출발 차선·변형. 이름 목록의 출처는 nodes/variant.py 하나. ALGO 미지정 = base (tune/algo 에 남은 이전 값으로 출발하지 않게 덮어씀)
LANE="${LANE:-2}"; ALGO="${ALGO:-base}"
[ "$LANE" = 1 ] || [ "$LANE" = 2 ] || { echo "LANE 은 1 또는 2 ($LANE)"; exit 1; }
[ "$ALGO" = base ] || python3 "$N/variant.py" --check "$ALGO" || { echo "ALGO '$ALGO' 모름 (목록: python3 $N/variant.py --list)"; exit 1; }
[ -f "$D/config/vehicle.yaml" ] || { echo "config/vehicle.yaml 없음 (제어기가 임포트 단계에서 죽음). 저장소에 함께 있어야 함"; exit 1; }
set_file() { printf '%s\n' "$2" > "$1.tmp" && mv "$1.tmp" "$1"; }   # car.sh set_file 과 같은 방식 (원자적 교체)
set_file "$D/tune/lane" "$LANE"
set_file "$D/tune/algo" "$ALGO"
md5_of() { if [ -f "$1" ]; then md5sum "$1" | cut -d' ' -f1; else echo none; fi; }

# 기록. 기존 키(tag conf webcam)는 유지, 옛 algo=(경로 노드 md5) 는 path_md5= 로 개명 (summarize_run.py 가 둘 다 읽음)
{ echo "tag=$(basename "$RUN")"
  echo "variant=$ALGO"; echo "lane=$LANE"
  echo "path_md5=$(md5_of "$N/lane_path_node.py")"
  echo "tune_md5=$(md5_of "$D/tune/tune.json")"; echo "variants_md5=$(md5_of "$D/tune/variants.json")"
  echo "conf=$CONF"; echo "webcam=${DEV:-none}"; date; } > "$RUN/run_info.txt"
cp "$D/tune/tune.json" "$RUN/tune_at_start.json"
cp "$D/tune/algo" "$RUN/algo_at_start"; cp "$D/tune/lane" "$RUN/lane_at_start"   # 벤치·주행 재현용 사본 (어떤 설정으로 돌았나)
[ -f "$D/tune/variants.json" ] && cp "$D/tune/variants.json" "$RUN/variants_at_start.json"
cat "$RUN/run_info.txt"
[ "$DRY" = 1 ] && { echo "== DRY=1: 노드 실행 전 종료 ($RUN)"; exit 0; }

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
