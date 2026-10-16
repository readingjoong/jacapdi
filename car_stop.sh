#!/usr/bin/env bash
# 비상 정지 (모터 명령 끄기). 사용: ~/jacapdi-docker/car_stop.sh        전부 끄기(요약 저장): car_stop.sh all
source "$(dirname "$0")/car_common.sh"
docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음"
send_stop
if [ "$1" = all ]; then
  docker exec "$C" bash -c 'kill -INT $(pgrep -f "bash /home/sean/jacapdi-docker/ab_kit/start_ab.sh" | head -1) 2>/dev/null; sleep 6;
    for p in $(pgrep -f "stanley_controller_node|tunable_controller.py|ab_kit/algo/|yolov8_node import|webcam_pub.py|run_monitor.py|board_monitor.py|path_viewer.py"); do kill -9 $p 2>/dev/null; done' >/dev/null 2>&1
  RUN=$(cat /tmp/car_run_dir 2>/dev/null); [ -f "$RUN/summary.txt" ] && head -8 "$RUN/summary.txt"
  say "파이프라인 종료"
fi
