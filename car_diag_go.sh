#!/usr/bin/env bash
# [1018] 느려짐 원인 찾기용 출발: 사진 기록 끄고(LOG=0) 기본 180초 주행. car_diag_prep.sh 가 "준비 완료" 띄우면 다른 창에서 바로 실행
# 사진도 보고 싶으면 (1초 10장): LOG=1 ~/jacapdi-docker/car_diag_go.sh
# 사용: ~/jacapdi-docker/car_diag_go.sh [초=180]     Ctrl+C = 정지
LOG="${LOG:-0}" exec "$(dirname "$0")/car_go.sh" "${1:-180}"
