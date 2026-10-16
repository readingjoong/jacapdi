#!/usr/bin/env bash
# 모터 켜고 주행 (car_prep.sh 다음). 사용: ~/jacapdi-docker/car_go.sh [초]   Ctrl+C = 즉시 정지
# 주행 중 10 Hz 기록: <기록폴더>/drive_HHMMSS/{frames/*.png, frames.csv}  (LOG=0 이면 끔, RAW=1 이면 원본 PNG 도)
source "$(dirname "$0")/car_common.sh"
SEC="${1:-$DRIVE_SEC}"
docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음. 먼저 ~/jacapdi-docker/car_prep.sh"
docker exec "$C" bash -c 'pgrep -f "ab_kit/algo/" >/dev/null' || die "파이프라인이 안 돌고 있음. 먼저 ~/jacapdi-docker/car_prep.sh"
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
OUT=""
if [ "${LOG:-1}" = 1 ] && [ -n "$RUN" ]; then
  OUT="$RUN/drive_$(date +%H%M%S)"; mkdir -p "$OUT"
  docker exec -d "$C" bash -ic "python3 ~/jacapdi-docker/frame_logger.py $OUT 10 ${RAW:-0} > $OUT/logger.log 2>&1"
  say "10 Hz 기록 시작: $OUT"
fi
stop_logger() { [ -n "$OUT" ] || return 0; docker exec "$C" bash -c 'pkill -INT -f frame_logger.py' >/dev/null 2>&1; sleep 1.5
  say "기록: $(ls "$OUT/frames" 2>/dev/null | wc -l)장, $(du -sh "$OUT" 2>/dev/null | cut -f1) -> $OUT  (frames.csv)"; }
stop_now() { trap - INT TERM; echo; warn "정지 중..."; send_stop; stop_logger
        # 정지 명령 다음에 YOLO 끄기 (절전 모드 복귀)
        yolo_on false && say "YOLO 꺼 둠 (배터리 절약)"
        [ -n "$RUN" ] && grep "FAULT:" "$RUN/board.log" 2>/dev/null | tail -2; exit 0; }
trap stop_now INT TERM
# 출발 직전에 YOLO 깨우기 (car_prep 이 꺼 둠). 못 켜면 출발 안 함
yolo_on true || { trap - INT TERM; warn "YOLO 켜기 실패. 출발하지 않음 (car_prep.sh 다시)"; stop_logger; exit 1; }
if [ "$TUNE" = 1 ]; then
  docker exec "$C" bash -c 'pgrep -f tunable_controller.py >/dev/null' || die "조정형 제어기가 안 돌고 있음. car_prep.sh 를 다시 실행"
  [ -n "$OUT" ] && cp "$D/tune/tune.json" "$OUT/tune_at_start.json"
  say "출발: ${SEC}초 (슬라이더 값: $(python3 -c "import json;d=json.load(open('$D/tune/tune.json'));print('Stanley %.2f, 방향 %.2f, PWM %s'%(d['stanley_gain'],d['heading_gain'],d['pwm_by_step']))"))  Ctrl+C = 정지"
else
  say "출발: ${SEC}초 ($(grep -E 'speed_command|max_steering' "$CTRL_YAML" | tr -s ' ' | tr '\n' ' '))  Ctrl+C = 정지"
fi
docker exec "$C" bash -c "DRIVE_SEC=$SEC ~/jacapdi-docker/timed_drive.sh" 2>&1 | grep -vE "RCLError|rcl_shutdown|Traceback|File |^\s|KeyboardInterrupt|ros2run" &
wait $!
stop_now
