#!/usr/bin/env bash
# 출발 (car_prep.sh 다음). 사용: ~/jacapdi-docker/car_go.sh [초]
# 주행 중 키: 1 / 2 = 그 차선으로 변경, q / 스페이스 / Ctrl+C = 정지
# 자동 차선 변경: LANE_AT="<초>:<차선>[,<초>:<차선>...]" 예) LANE_AT=8:1,20:2 ~/jacapdi-docker/car_go.sh 60
#   출발 뒤 8초에 1차선, 20초에 2차선으로 set_lane (키 1/2 와 같이 씀). 실차는 자기 위치를 모르니 시간 기준이다 (가제보는 sim/run/7_lane_scenario.sh 가 위치 s 기준).
#   형식이 틀린 항목(초가 숫자 아님, 차선이 1|2 아님)은 경고만 하고 건너뜀. 실행된 항목은 <기록폴더>/lane_events.txt 에 남김 (10 Hz 기록이 꺼져 있어도)
#   LANE_AT 를 쓰면 출발 전에 tune.json lane_change_sec 가 권장 범위(2.0~3.5)인지 확인한다 (10/31 가제보 벤치 기준, 실차는 3.0 부터)
# 출발 스위치 tune/go_until 에 끝 시각을 쓰면 제어기가 0.05초 안에 명령을 내보낸다 (이 창이 죽어도 그 시각에 스스로 멈춤).
# 주행 중 10 Hz 기록: <기록폴더>/drive_HHMMSS/{frames/*.png, frames.csv}  (LOG=0 이면 끔, RAW=1 이면 원본 PNG 도)
source "$(dirname "${BASH_SOURCE[0]}")/car.sh"
SEC="${1:-$DRIVE_SEC}"
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
OUT=""
SCHED=""
lane_sched() {   # LANE_AT 처리: 출발 시각 기준 각 항목의 초가 되면 set_lane. 항목은 적은 순서대로 실행. 구문 오류 항목은 경고 뒤 건너뜀
  local t0 item sec lane left ev
  t0=$(date +%s.%N)
  for item in ${LANE_AT//,/ }; do
    sec="${item%%:*}"; lane="${item##*:}"
    if [[ ! "$item" =~ ^[0-9]+(\.[0-9]+)?:[12]$ ]]; then warn "LANE_AT 항목 무시 (초:차선 형식, 차선 1|2): $item"; continue; fi
    left=$(python3 -c "import time;print(max(0.0, $sec - (time.time() - $t0)))" 2>/dev/null) || continue
    sleep "$left"
    set_lane "$lane"
    ev="${OUT:-$RUN}"   # 10 Hz 기록 폴더가 있으면 거기, 없으면 RUN 폴더
    [ -n "$ev" ] && [ -d "$ev" ] && echo "$(date +%T.%N | cut -c1-12) +${sec}s lane $lane" >> "$ev/lane_events.txt"
  done
}
stop_now() { trap - INT TERM; echo; warn "정지 중..."
  rm -f "$D/tune/go_until"; sleep 0.6; say "정지 (출발 스위치 끔)"   # 제어기가 0.5초 동안 s0l0r0. 송신기는 다음 출발을 위해 켜 둠
  [ -n "$SCHED" ] && { pkill -P "$SCHED" >/dev/null 2>&1; kill "$SCHED" >/dev/null 2>&1; }   # LANE_AT 예약 남은 것 취소
  if [ -n "$OUT" ]; then
    docker exec $C pkill -INT -f nodes/frame_logger.py >/dev/null 2>&1; sleep 1.5
    say "기록: $(ls "$OUT/frames" 2>/dev/null | wc -l)장, $(du -sh "$OUT" 2>/dev/null | cut -f1) -> $OUT  (frames.csv)"
  fi
  [ -n "$RUN" ] && grep "FAULT:" "$RUN/board.log" 2>/dev/null | tail -2; exit 0; }
[ "${BASH_SOURCE[0]}" = "$0" ] || return 0      # source 됐으면 (tests/test_shell_static.sh 가 lane_sched 검사) 여기까지
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
if [ -n "$LANE_AT" ]; then   # 차선 변경 예약: lane_change_sec 확인 (10/31 가제보: 2.0·3.0 완주, 4.0 은 코너와 겹치면 정지. 실차는 3.0 부터 권장)
  python3 -c "import json,sys;d=json.load(open('$D/tune/tune.json'));sys.exit(0 if 2.0 <= float(d.get('lane_change_sec',0)) <= 3.5 else 1)" 2>/dev/null \
    || warn "LANE_AT: tune.json lane_change_sec 가 권장 범위(2.0~3.5) 밖. 변경 지점이 코너 2 m 안이면 정지 위험"
fi
python3 -c "import time;print(time.time()+$SEC)" > "$D/tune/go_until"   # 출발!
GO_T=$(date +%T.%N | cut -c1-12)

if [ "${LOG:-1}" = 1 ] && [ -n "$RUN" ]; then
  OUT="$RUN/drive_$(date +%H%M%S)"; mkdir -p "$OUT"
  docker exec -d $C bash -c "$CX; python3 $D/nodes/frame_logger.py $OUT 10 ${RAW:-0} > $OUT/logger.log 2>&1"
  cp "$D/tune/tune.json" "$OUT/tune_at_start.json"
  say "10 Hz 기록 시작: $OUT"
fi
say "출발 $GO_T: ${SEC}초, $(cat "$D/tune/lane" 2>/dev/null || echo 2)차선, 변형 $(cat "$D/tune/algo" 2>/dev/null || echo base) (슬라이더 값: $(python3 -c "import json;d=json.load(open('$D/tune/tune.json'));print('Stanley %.2f, 방향 %.2f, PWM %d'%(d['stanley_gain'],d['heading_gain'],d['pwm']))"))"
say "키: 1 / 2 = 차선 변경, q / 스페이스 / Ctrl+C = 정지"
if [ -n "$LANE_AT" ]; then
  say "자동 차선 변경 예약 (출발 기준 초:차선): $LANE_AT"
  lane_sched & SCHED=$!
fi
wait_keys "$SEC"
stop_now
