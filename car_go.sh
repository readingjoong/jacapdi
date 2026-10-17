#!/usr/bin/env bash
# 모터 켜고 주행 (car_prep.sh 다음). 사용: ~/jacapdi-docker/car_go.sh [초]   Ctrl+C = 즉시 정지
# 주행 중 10 Hz 기록: <기록폴더>/drive_HHMMSS/{frames/*.png, frames.csv}  (LOG=0 이면 끔, RAW=1 이면 원본 PNG 도)
source "$(dirname "$0")/car_common.sh"
SEC="${1:-$DRIVE_SEC}"
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
OUT=""
stop_logger() { [ -n "$OUT" ] || return 0; docker exec "$C" bash -c 'pkill -INT -f frame_logger.py' >/dev/null 2>&1; sleep 1.5
  say "기록: $(ls "$OUT/frames" 2>/dev/null | wc -l)장, $(du -sh "$OUT" 2>/dev/null | cut -f1) -> $OUT  (frames.csv)"; }
stop_now() { trap - INT TERM; echo; warn "정지 중..."
        if [ "$TUNE" = 1 ]; then   # 출발 스위치 끄면 제어기가 0.5초 동안 s0l0r0 보냄. 송신기는 다음 출발을 위해 켜 둠
          rm -f "$D/tune/go_until"; sleep 0.6; say "정지 (출발 스위치 끔)"
        else send_stop; fi
        stop_logger
        [ -n "$RUN" ] && grep "FAULT:" "$RUN/board.log" 2>/dev/null | tail -2; exit 0; }
trap stop_now INT TERM
if [ "$TUNE" = 1 ]; then
  # [1017] 바로 출발: 확인은 docker exec 한 번 (0: 다 있음, 1: 파이프라인/제어기 없음, 2: 송신기 없음)
  docker exec "$C" bash -c 'pgrep -f "ab_kit/algo/" >/dev/null && pgrep -f tunable_controller.py >/dev/null || exit 1
    pgrep -f "serial_communication_pkg/serial_sender_node" >/dev/null || exit 2' 2>/dev/null; ok=$?
  [ $ok = 1 ] && die "파이프라인/조정형 제어기가 안 돌고 있음. 먼저 ~/jacapdi-docker/car_prep.sh"
  # 지금 도는 제어기가 출발 스위치를 아는지 (status.json 에 "go" 키, 1초 안에 갱신). 모르는 예전 제어기면 정지를 못 하므로 출발 안 함
  python3 -c "import json,time;d=json.load(open('$D/tune/status.json'));exit(0 if 'go' in d and time.time()-d['time']<1 else 1)" 2>/dev/null \
    || die "제어기가 예전 코드이거나 멈춰 있음 (출발 스위치 없음). ~/jacapdi-docker/car_prep.sh 다시"
  if [ $ok = 2 ]; then   # car_stop 뒤 등: 송신기 다시 켜기 (이때만 약 2초 늦음)
    warn "시리얼 송신기 없음 -> 다시 켬 (2초)"; start_sender; sleep 2
  fi
  echo "$(python3 -c "import time;print(time.time()+$SEC)")" > "$D/tune/go_until"   # 출발! (제어기가 0.05초 안에 읽음)
  GO_T=$(date +%T.%N | cut -c1-12)
else
  docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음. 먼저 ~/jacapdi-docker/car_prep.sh"
  docker exec "$C" bash -c 'pgrep -f "ab_kit/algo/" >/dev/null' || die "파이프라인이 안 돌고 있음. 먼저 ~/jacapdi-docker/car_prep.sh"
fi
if [ "${LOG:-1}" = 1 ] && [ -n "$RUN" ]; then
  OUT="$RUN/drive_$(date +%H%M%S)"; mkdir -p "$OUT"
  docker exec -d "$C" bash -ic "python3 ~/jacapdi-docker/frame_logger.py $OUT 10 ${RAW:-0} > $OUT/logger.log 2>&1"
  say "10 Hz 기록 시작: $OUT"
fi
if [ "$TUNE" = 1 ]; then
  [ -n "$OUT" ] && cp "$D/tune/tune.json" "$OUT/tune_at_start.json"
  say "출발 $GO_T: ${SEC}초 (슬라이더 값: $(python3 -c "import json;d=json.load(open('$D/tune/tune.json'));print('Stanley %.2f, 방향 %.2f, PWM %s'%(d['stanley_gain'],d['heading_gain'],d['pwm_by_step']))"))  Ctrl+C = 정지"
else
  say "출발: ${SEC}초 ($(grep -E 'speed_command|max_steering' "$CTRL_YAML" | tr -s ' ' | tr '\n' ' '))  Ctrl+C = 정지"
fi
if [ "$TUNE" = 1 ]; then
  sleep "$SEC" &   # 제어기도 go_until 시각에 스스로 멈춤 (이 창이 죽어도 SEC 초 뒤 정지)
else
  docker exec "$C" bash -c "DRIVE_SEC=$SEC ~/jacapdi-docker/timed_drive.sh" 2>&1 | grep -vE "RCLError|rcl_shutdown|Traceback|File |^\s|KeyboardInterrupt|ros2run" &
fi
wait $!
stop_now
