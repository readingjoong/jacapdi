#!/usr/bin/env bash
# 전원 켠 뒤 한 번: 장치 확인 -> 컨테이너 -> 파이프라인(웹캠/YOLO/경로/제어기) -> 보드 모니터 + 송신기 -> 화면 -> 준비 확인
# 사용: ~/jacapdi-docker/car_prep.sh          화면 끄기: VIEW=0, 1차선에서 출발: LANE=1
# 이 창에서 Ctrl+C / 창 닫기 = 전부 끄기. car.sh all/lap 처럼 다른 스크립트가 부를 땐 PREP_HOLD=0
source "$(dirname "$0")/car.sh"
say "1/6 장치 확인 (웹캠 C920, 아두이노) - 최대 30초 대기"
for i in $(seq 30); do
  CAM=$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index 2>/dev/null)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)
  ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1)
  [ -n "$CAM" ] && [ -n "$ACM" ] && break; sleep 1
done
[ -n "$CAM" ] || die "웹캠 C920 이 안 보임. USB 확인"
[ -n "$ACM" ] || die "아두이노가 안 보임. USB 확인"
docker rm -f $C >/dev/null 2>&1                   # 이전 실행의 보드 모니터/송신기도 함께 정리
sleep 1
U=$(port_users); [ -z "$U" ] || die "아두이노 포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
say "   웹캠 $CAM, 아두이노 $ACM (컨테이너 안에서는 /dev/ttyACM0)"

say "2/6 컨테이너 새로 만들기"
container_up || die "컨테이너 시작 실패"
HOLD="${PREP_HOLD:-1}"
CID=$(docker ps -q -f name=$C)                    # 이 car_prep 이 만든 컨테이너 (다른 창에서 car_prep 을 다시 하면 그건 안 건드림)
shutdown_all() { trap - INT TERM HUP; echo
  if [ "$(docker ps -q -f name=$C)" = "$CID" ]; then warn "car_prep 종료 -> 전부 끄는 중 (car.sh stop all)"; stop all
  else warn "car_prep 종료 (컨테이너가 이미 바뀌어서 아무것도 안 끔)"; fi; exit 0; }
[ "$HOLD" = 1 ] && trap shutdown_all INT TERM HUP

say "3/6 파이프라인 시작 (CONF=$CONF, ${LANE:-2}차선) - 출발 스위치 꺼짐"
rm -f "$D/tune/go_until"
echo "${LANE:-2}" > "$D/tune/lane"               # 출발 차선 (주행 중 변경: car.sh lane 1|2 또는 tune_gui 버튼)
RUN="$D/ab_runs/$(date +%m%d_%H%M%S)_conf$CONF"
docker exec -d $C bash -c "RUN=$RUN CONF=$CONF bash $D/pipeline.sh > /tmp/pipeline.log 2>&1"
for i in $(seq 20); do [ -f "$RUN/run_info.txt" ] && break; sleep 1; done
[ -f "$RUN/run_info.txt" ] || die "파이프라인 시작 실패: docker exec $C cat /tmp/pipeline.log"
echo "$RUN" > /tmp/car_run_dir
say "   슬라이더: python3 ~/jacapdi-docker/tune_gui.py"

say "4/6 보드 모니터 (펌웨어 확인, FAULT 기록) + 시리얼 송신기"
docker exec -d $C python3 "$D/nodes/board_monitor.py" "$RUN/board.log"
sleep 4
FW=$(grep -o "DRIVING_TEAM[^ ]*" "$RUN/board.log" | head -1)
ADC=$(grep -o "IDLE ADC=[0-9]*" "$RUN/board.log" | tail -1)
[ "$FW" = "$FW_NAME" ] && say "   펌웨어 $FW, 조향 $ADC" || warn "   펌웨어가 $FW_NAME 이 아님 (보드: ${FW:-응답없음}). 업로드: ~/jacapdi-docker/car.sh flash"
start_sender; say "   시리얼 송신기 대기 중 (출발 스위치 꺼짐)"   # board_monitor 가 포트를 먼저 열어서 보드가 다시 리셋되지 않음
A=${ADC#IDLE ADC=}   # 조향 가변저항 값이 정상 범위(펌웨어 1016r: POT_RIGHT-40 ~ POT_LEFT+40 = 352~599)인지 확인
if [ -n "$A" ] && { [ "$A" -lt 352 ] || [ "$A" -gt 599 ]; }; then
  printf '\033[1;31m[car]    !!! 조향 가변저항 값 %s = 비정상 (정상 352~599). 출발하면 펌웨어가 바로 모터를 끔 (adc_out_of_range).\n[car]    !!! 1023 = GND 선 빠짐/신호가 5V 에 닿음, 0 = 신호선 빠짐/GND 에 닿음. 배터리·USB 끄고 가변저항 배선 확인\033[0m\n' "$A"
fi

say "5/6 실시간 화면"
[ "${VIEW:-1}" = 1 ] && docker exec -d $C bash -c "$CX; python3 $D/nodes/path_viewer.py > /tmp/viewer.log 2>&1"

say "6/6 YOLO/경로 준비 대기 (보통 20~30초)"
for i in $(seq 60); do
  grep 'frames=' "$RUN/line_path.log" 2>/dev/null | tail -1 | grep -q 'frames=[1-9]' && break; sleep 1
done
docker exec $C bash -c "$CX; python3 $D/nodes/rate_probe.py 2>/dev/null | tail -3"
say "준비 완료. 기록 폴더: $RUN"
say "출발: (다른 창에서) ~/jacapdi-docker/car_go.sh   (기본 ${DRIVE_SEC}초, 예: car_go.sh 60)   비상정지: car_go 창에서 Ctrl+C 또는 ~/jacapdi-docker/car.sh stop"
if [ "$HOLD" = 1 ]; then
  say "이 창은 열어 두세요. 여기서 Ctrl+C 또는 창 닫기 = YOLO/웹캠/제어기 전부 끄기"
  while [ "$(docker ps -q -f name=$C)" = "$CID" ]; do sleep 5 & wait $!; done   # 컨테이너가 없어지거나 바뀌면 그냥 끝
fi
