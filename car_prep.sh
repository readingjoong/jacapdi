#!/usr/bin/env bash
# 전원 켠 뒤 한 번: 장치 확인 -> 컨테이너 -> 파이프라인(웹캠/YOLO/경로/Stanley, 모터 제외) -> 보드 모니터 -> 화면 -> 준비 확인
# 사용: ~/jacapdi-docker/car_prep.sh          (ALGO=B CONF=0.5 기본, 바꾸려면 ALGO=C CONF=0.3 ~/jacapdi-docker/car_prep.sh)
source "$(dirname "$0")/car_common.sh"
say "1/6 장치 확인 (웹캠 C920, 아두이노) - 최대 30초 대기"
for i in $(seq 30); do
  CAM=$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index 2>/dev/null)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)
  ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1)
  [ -n "$CAM" ] && [ -n "$ACM" ] && break; sleep 1
done
[ -n "$CAM" ] || die "웹캠 C920 이 안 보임. USB 확인"
[ -n "$ACM" ] || die "아두이노가 안 보임. USB 확인"
docker rm -f "$C" >/dev/null 2>&1 || true          # 이전 실행의 보드 모니터/송신기도 함께 정리
pkill -f "jacapdi-docker/board_monitor.py" 2>/dev/null || true
sleep 1
U=$(port_users); [ -z "$U" ] || die "아두이노 포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
say "   웹캠 $CAM, 아두이노 $ACM (컨테이너 안에서는 /dev/ttyACM0)"

say "2/6 컨테이너 새로 만들기"
docker rm -f "$C" >/dev/null 2>&1 || true
"$D/run.sh" true </dev/null 2>&1 | grep started || die "컨테이너 시작 실패"
docker exec -u root "$C" bash -c 'for d in /dev/video*; do chgrp video $d; chmod 660 $d; done; chgrp dialout /dev/ttyACM0; chmod 660 /dev/ttyACM0'

say "3/6 파이프라인 시작 (ALGO=$ALGO CONF=$CONF, $(basename "$CTRL_YAML")) - 모터는 아직 안 켬"
T0=$(date +%s)
docker exec -d "$C" bash -ic "CTRL_YAML=$CTRL_YAML ALGO=$ALGO CONF=$CONF bash ~/jacapdi-docker/ab_kit/start_ab.sh > /tmp/start_ab.log 2>&1"
RUN=""
for i in $(seq 20); do
  RUN=$(ls -td "$D"/ab_runs/*_"${ALGO}"_conf"${CONF}" 2>/dev/null | head -1)
  [ -n "$RUN" ] && [ "$(stat -c %Y "$RUN")" -ge "$T0" ] && break; RUN=""; sleep 1
done
[ -n "$RUN" ] || die "파이프라인 시작 실패: docker exec $C cat /tmp/start_ab.log"
echo "$RUN" > /tmp/car_run_dir

if [ "$TUNE" = 1 ]; then
  docker exec -d "$C" bash -ic "python3 ~/jacapdi-docker/tunable_controller.py $RUN > $RUN/tunable.log 2>&1"
  cp "$D/tune/tune.json" "$RUN/tune_at_start.json"
  say "   조정형 제어기 사용 (슬라이더: python3 ~/jacapdi-docker/tune_gui.py)"
fi
say "4/6 보드 모니터 (펌웨어 확인, FAULT 기록)"
docker exec -d "$C" bash -c "python3 ~/jacapdi-docker/board_monitor.py $RUN/board.log"
sleep 4
FW=$(grep -o "DRIVING_TEAM[^ ]*" "$RUN/board.log" | head -1)
ADC=$(grep -o "IDLE ADC=[0-9]*" "$RUN/board.log" | tail -1)
[ "$FW" = "$FW_NAME" ] && say "   펌웨어 $FW, 조향 $ADC" || warn "   펌웨어가 $FW_NAME 이 아님 (보드: ${FW:-응답없음}). 업로드: ~/jacapdi-docker/car_flash.sh"
A=${ADC#IDLE ADC=}   # 조향 가변저항 값이 정상 범위(360~570, 펌웨어 1016 기준)인지 확인
if [ -n "$A" ] && { [ "$A" -lt 360 ] || [ "$A" -gt 570 ]; }; then
  printf '\033[1;31m[car]    !!! 조향 가변저항 값 %s = 비정상 (정상 400~530). 출발하면 펌웨어가 바로 모터를 끔 (adc_out_of_range).\n[car]    !!! 1023 = GND 선 빠짐/신호가 5V 에 닿음, 0 = 신호선 빠짐/GND 에 닿음. 배터리·USB 끄고 가변저항 배선 확인\033[0m\n' "$A"
fi

say "5/6 실시간 화면"
[ "${VIEW:-1}" = 1 ] && docker exec -d "$C" bash -ic 'python3 ~/jacapdi-docker/path_viewer.py /image_raw /lane_detections > /tmp/viewer.log 2>&1'

say "6/6 YOLO/경로 준비 대기 (보통 20~30초)"
for i in $(seq 60); do
  docker exec "$C" bash -c "grep -q 'frames=' $RUN/line_path.log 2>/dev/null && grep 'frames=' $RUN/line_path.log | tail -1 | grep -qv 'frames=0 '" && break; sleep 1
done
docker exec "$C" bash -ic 'python3 ~/jacapdi-docker/rate_probe.py 2>/dev/null | tail -3' 2>/dev/null | grep -vE "job control|terminal process group"
say "준비 완료. 기록 폴더: $RUN"
say "출발: ~/jacapdi-docker/car_go.sh   (기본 ${DRIVE_SEC}초, 예: car_go.sh 60)   비상정지: Ctrl+C 또는 ~/jacapdi-docker/car_stop.sh"

# 준비 완료 후 사람이 출발 누를 때까지 YOLO GPU 추론 꺼두기 (배터리 보존). car_go.sh 가 출발 직전에 켠다
yolo_on false && say "YOLO 꺼 둠 (배터리 절약, car_go 출발 때 켬. 그동안 실시간 화면은 멈춤)" || warn "YOLO 끄기 실패 (켜진 채로 둠, 주행에는 문제 없음)"
