#!/usr/bin/env bash
# 출발 (car_prep.sh 다음). 사용: ~/jacapdi-docker/car_go.sh [초]   Ctrl+C = 즉시 정지
# 출발 스위치 tune/go_until 에 끝 시각을 쓰면 제어기가 0.05초 안에 명령을 내보낸다 (이 창이 죽어도 그 시각에 스스로 멈춤).
# 주행 중 10 Hz 기록: <기록폴더>/drive_HHMMSS/{frames/*.png, frames.csv}  (LOG=0 이면 끔, RAW=1 이면 원본 PNG 도)
source "$(dirname "$0")/car.sh"
SEC="${1:-$DRIVE_SEC}"
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
OUT=""
stop_now() { trap - INT TERM; echo; warn "정지 중..."
  rm -f "$D/tune/go_until"; sleep 0.6; say "정지 (출발 스위치 끔)"   # 제어기가 0.5초 동안 s0l0r0. 송신기는 다음 출발을 위해 켜 둠
  if [ -n "$OUT" ]; then
    docker exec $C pkill -INT -f nodes/frame_logger.py >/dev/null 2>&1; sleep 1.5
    say "기록: $(ls "$OUT/frames" 2>/dev/null | wc -l)장, $(du -sh "$OUT" 2>/dev/null | cut -f1) -> $OUT  (frames.csv)"
  fi
  [ -n "$RUN" ] && grep "FAULT:" "$RUN/board.log" 2>/dev/null | tail -2; exit 0; }
trap stop_now INT TERM

# 확인은 docker exec 한 번 (0: 다 있음, 1: 파이프라인/제어기 없음, 2: 송신기 없음). [n]odes = bash -c 자신의 명령줄과 안 맞게
docker exec $C bash -c 'pgrep -f "[n]odes/lane_path_node.py" >/dev/null && pgrep -f "[n]odes/tunable_controller.py" >/dev/null || exit 1
  pgrep -f "[n]odes/serial_sender_node.py" >/dev/null || exit 2' 2>/dev/null; ok=$?
[ $ok = 1 ] && die "파이프라인/제어기가 안 돌고 있음. 먼저 ~/jacapdi-docker/car_prep.sh"
python3 -c "import json,time;d=json.load(open('$D/tune/status.json'));exit(0 if 'go' in d and time.time()-d['time']<1 else 1)" 2>/dev/null \
  || die "제어기가 멈춰 있음 (status.json 이 1초 넘게 안 바뀜). ~/jacapdi-docker/car_prep.sh 다시"
if [ $ok = 2 ]; then   # car.sh stop 뒤 등: 송신기 다시 켜기 (이때만 약 2초 늦음)
  warn "시리얼 송신기 없음 -> 다시 켬 (2초)"; start_sender; sleep 2
fi
python3 -c "import time;print(time.time()+$SEC)" > "$D/tune/go_until"   # 출발!
GO_T=$(date +%T.%N | cut -c1-12)

if [ "${LOG:-1}" = 1 ] && [ -n "$RUN" ]; then
  OUT="$RUN/drive_$(date +%H%M%S)"; mkdir -p "$OUT"
  docker exec -d $C bash -c "$CX; python3 $D/nodes/frame_logger.py $OUT 10 ${RAW:-0} > $OUT/logger.log 2>&1"
  cp "$D/tune/tune.json" "$OUT/tune_at_start.json"
  say "10 Hz 기록 시작: $OUT"
fi
say "출발 $GO_T: ${SEC}초 (슬라이더 값: $(python3 -c "import json;d=json.load(open('$D/tune/tune.json'));print('Stanley %.2f, 방향 %.2f, PWM %s'%(d['stanley_gain'],d['heading_gain'],d['pwm_by_step']))"))  Ctrl+C = 정지"
sleep "$SEC" &
wait $!
stop_now
