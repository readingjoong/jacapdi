#!/usr/bin/env bash
# [1018] 느려짐 원인 찾기용 준비: car_prep.sh 를 실시간 화면 끄고(VIEW=0) 실행. 이 창은 열어 두고, 끝나면 여기서 Ctrl+C = 전부 끄기
# timeline.csv 에 cam_hz / cam_fail / cpu_temp_c / cpu_mhz / yolo_cpu 가 같이 남는다 (기존 car_prep.sh 도 같음)
# 사용: ~/jacapdi-docker/car_diag_prep.sh
VIEW=0 exec "$(dirname "$0")/car_prep.sh" "$@"
