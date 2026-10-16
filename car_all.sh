#!/usr/bin/env bash
# 한 방에: 준비 -> (엔터) -> 주행. 사용: ~/jacapdi-docker/car_all.sh [초]
source "$(dirname "$0")/car_common.sh"
"$(dirname "$0")/car_prep.sh" || exit 1
read -r -p $'\n차를 출발 위치에 놓고 엔터 = 출발, Ctrl+C = 취소 ' _
exec "$(dirname "$0")/car_go.sh" "${1:-$DRIVE_SEC}"
