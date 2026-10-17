#!/usr/bin/env bash
# 한 방에 한 바퀴: 준비 -> 차선 인식 확인 -> 5초 카운트다운 -> LAP_SEC 초 주행 -> 자동 정지 (10 Hz 기록 포함)
# 사용: ~/jacapdi-docker/car_lap.sh            (기본 75초 = 9/27 한 바퀴 주행 시간 기준)
#       ~/jacapdi-docker/car_lap.sh 70         (한 바퀴 시간을 초로 지정)
# 차는 바퀴 위치를 모른다(엔코더/GPS 없음). "한 바퀴"는 시간으로 정한다. Ctrl+C = 즉시 정지.
source "$(dirname "$0")/car_common.sh"
LAP_SEC="${1:-${LAP_SEC:-75}}"
"$D/car_prep.sh" || exit 1

say "차선 인식 확인 (경로 valid 80% 이상이 될 때까지, 최대 30초). 차를 2차선 출발 위치에 놓으세요"
ok=0
for i in $(seq 6); do
  line=$(docker exec "$C" bash -ic 'python3 ~/jacapdi-docker/rate_probe.py 2>/dev/null' 2>/dev/null | grep "/lane_trajectory")
  v=$(echo "$line" | grep -o "valid [0-9]*/[0-9]*" | cut -d' ' -f2)
  n=${v%/*}; m=${v#*/}
  if [ -n "$m" ] && [ "$m" -gt 0 ] && [ $((100 * n / m)) -ge 80 ]; then ok=1; say "   경로 valid $v - 차선 인식 OK"; break; fi
  warn "   경로 valid ${v:-0/0} - 차선이 잘 안 보임, 다시 확인"
done
[ "$ok" = 1 ] || die "차선이 안 보여서 출발하지 않음. 차를 2차선에 놓고: ~/jacapdi-docker/car_go.sh $LAP_SEC"

trap 'echo; warn "출발 취소"; exit 0' INT
for s in 5 4 3 2 1; do printf '\r\033[1;33m[car] %d초 뒤 출발 (Ctrl+C = 취소)\033[0m ' "$s"; sleep 1; done; echo
trap - INT
exec "$D/car_go.sh" "$LAP_SEC"
