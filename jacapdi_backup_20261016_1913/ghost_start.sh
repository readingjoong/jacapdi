#!/usr/bin/env bash
# 고스트 노드 (그림자 모드: 기록만, 조향 안 함) 켜기/끄기. car_prep.sh 다음에.
#   ~/jacapdi-docker/ghost_start.sh        켜기 → <기록폴더>/ghost.csv, ghost.log
#   ~/jacapdi-docker/ghost_start.sh stop   끄기
source "$(dirname "$0")/car_common.sh"
docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음. 먼저 ~/jacapdi-docker/car_prep.sh"
if [ "$1" = stop ]; then
  docker exec "$C" bash -c 'pkill -INT -f ghost_node.py' >/dev/null 2>&1; say "고스트 노드 끔"; exit 0
fi
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
[ -n "$RUN" ] && [ -d "$RUN" ] || die "기록 폴더 없음 (/tmp/car_run_dir). car_prep.sh 를 다시"
docker exec "$C" bash -c 'pgrep -f ghost_node.py >/dev/null' && { warn "이미 켜져 있음"; exit 0; }
docker exec -d "$C" bash -ic "python3 $D/ghost_node.py $RUN > $RUN/ghost.log 2>&1"
sleep 3
docker exec "$C" bash -c 'pgrep -f ghost_node.py >/dev/null' || { tail -5 "$RUN/ghost.log"; die "고스트 노드 시작 실패"; }
say "고스트 노드 켬 (그림자 모드) → $RUN/ghost.csv"
