#!/usr/bin/env bash
# 카메라 없이 A~D 경로 알고리즘 점검 (도커 컨테이너 안에서): bash ~/jacapdi-docker/ab_kit/check_ab.sh
source /opt/ros/humble/setup.bash
source ~/team_ws/install/setup.bash
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$KIT/check_algos.py"
