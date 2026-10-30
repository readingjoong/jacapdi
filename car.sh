#!/usr/bin/env bash
# 공통 설정·함수 + 보조 명령 (호스트에서 실행). car_prep.sh / car_go.sh 가 source 한다.
#   준비: ~/jacapdi-docker/car_prep.sh      출발: ~/jacapdi-docker/car_go.sh [초]
#   ~/jacapdi-docker/car.sh stop [all]   비상 정지.  all = 파이프라인까지 끄고 요약
#   ~/jacapdi-docker/car.sh all [초]     car_prep → 엔터 → car_go
#   ~/jacapdi-docker/car.sh lap [초]     car_prep → 차선 인식 확인 → 5초 카운트다운 → car_go (기본 75초)
#   ~/jacapdi-docker/car.sh flash        아두이노에 펌웨어 업로드 (필요할 때만)
#   ~/jacapdi-docker/car.sh shell [명령] 컨테이너 셸 (명령을 주면 그것만 실행)
CONF="${CONF:-0.3}"                                           # YOLO 확신도 기준 (경로 노드가 실선/점선을 0.30 으로 다시 거름)
DRIVE_SEC="${DRIVE_SEC:-100}"                                 # go 기본 주행 시간(초)
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"             # 이 폴더 (컨테이너 안에서도 같은 경로로 보임)
FW_HEX="${FW_HEX:-$D/arduino/driving_team_cal_1016r/driving_team_cal_1016r.ino.hex}"
FW_NAME="${FW_NAME:-DRIVING_TEAM_CAL_1016R}"                  # 보드 부팅 메시지에 있어야 할 이름
AVR=$HOME/Downloads/arduino-1.8.19/hardware/tools/avr
C=jacapdi-humble
IMAGE=jacapdi/humble:latest
CX="source $D/pipeline.sh env"                                # docker exec 안에서 ROS 환경 켜기
SENDER_LOG=/tmp/pipeline/serial.log
ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1); ACM=${ACM:-/dev/ttyACM0}   # 호스트의 아두이노 포트 (ttyACM0 이 아닐 수도 있음)

say()  { printf '\033[1;32m[car]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[car]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[car] %s\033[0m\n' "$*"; exit 1; }
running() { [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = true ]; }

port_users() {   # 아두이노 포트를 잡고 있는 호스트 프로세스 (Arduino IDE 시리얼 모니터 등)
  for p in /proc/[0-9]*; do ls -l "$p/fd" 2>/dev/null | grep -q "$(basename "$ACM")" && echo "$(basename "$p") $(tr '\0' ' ' < "$p/cmdline" | cut -c1-60)"; done
}

container_up() { # 컨테이너 시작 (이미 돌고 있으면 그대로). 이 폴더만 같은 경로로 붙인다
  running && return 0
  docker rm -f $C >/dev/null 2>&1
  local DEVS=() d a
  for d in /dev/video* /dev/ttyUSB*; do [ -e "$d" ] && DEVS+=(--device "$d"); done
  a=$(ls /dev/ttyACM* 2>/dev/null | head -1); [ -n "$a" ] && DEVS+=(--device "$a:/dev/ttyACM0")   # 안에서는 항상 ttyACM0
  docker run -d --name $C --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --network host --ipc host \
    -e DISPLAY="${DISPLAY:-:1}" -e XAUTHORITY=/tmp/.Xauthority -e QT_X11_NO_MITSHM=1 -e YOLO_CONFIG_DIR=/tmp/Ultralytics \
    -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${XAUTHORITY:-$HOME/.Xauthority}":/tmp/.Xauthority:ro \
    --group-add 44 --group-add 20 \
    --device-cgroup-rule='c 81:* rmw' --device-cgroup-rule='c 166:* rmw' --device-cgroup-rule='c 188:* rmw' \
    "${DEVS[@]}" -v "$D:$D" $IMAGE sleep infinity >/dev/null || return 1
  docker exec -u root $C bash -c 'for d in /dev/video* /dev/ttyACM0; do [ -e $d ] || continue; case $d in *video*) chgrp video $d;; *) chgrp dialout $d;; esac; chmod 660 $d; done'
  if [ ! -f "$D/ros/install/setup.bash" ]; then   # 메시지 패키지 (처음 한 번, 약 10초)
    say "   메시지 패키지(interfaces_pkg) 빌드"
    docker exec $C bash -c "source /opt/ros/humble/setup.bash && cd $D/ros && colcon build --base-paths interfaces_pkg > /tmp/colcon.log 2>&1" \
      || die "빌드 실패: docker exec $C cat /tmp/colcon.log"
  fi
  say "   컨테이너 시작 (장치: ${DEVS[*]:-없음})"
}

send_stop() {    # 출발 스위치 끄고, 송신기 끄고, s0l0r0 5번 (컨테이너 안에서)
  rm -f "$D/tune/go_until"
  docker exec $C bash -c 'pkill -INT -f "[n]odes/serial_sender_node.py"; sleep 1.5; pkill -9 -f "[n]odes/serial_sender_node.py"' >/dev/null 2>&1   # [n]: bash -c 자신과 안 맞게
  docker exec $C python3 -c "
import serial, time
s = serial.Serial(); s.port = '/dev/ttyACM0'; s.baudrate = 115200; s.timeout = 0.1; s.dsrdtr = False; s.open()
for _ in range(5): s.write(b's0l0r0\n'); time.sleep(0.05)
s.close()" >/dev/null 2>&1 && say "정지 명령 s0l0r0 전송" || warn "정지 명령 전송 실패 (아두이노 연결 확인). 펌웨어가 0.3초 뒤 스스로 정지함"
}

start_sender() { # 시리얼 송신기를 미리 띄워 둠. 출발 스위치(tune/go_until)가 꺼져 있으면 제어기가 아무것도 안 보내서 보드는 IDLE
  docker exec -d $C bash -c "mkdir -p /tmp/pipeline; $CX; exec python3 $D/nodes/serial_sender_node.py > $SENDER_LOG 2>&1"
}

# ------------------------------------------------------------------------------------------------
stop() {
  running || die "컨테이너 없음"
  send_stop
  if [ "$1" = all ]; then
    docker exec $C bash -c "pkill -INT -f '^bash $D/pipeline.sh'; sleep 6; pkill -9 -f '[p]ython3 $D/nodes/'" >/dev/null 2>&1   # ^, [p]: bash -c 자신과 안 맞게
    local RUN; RUN=$(cat /tmp/car_run_dir 2>/dev/null); [ -f "$RUN/summary.txt" ] && head -8 "$RUN/summary.txt"
    say "파이프라인 종료"
  fi
}

lap() {          # 차는 바퀴 위치를 모른다(엔코더/GPS 없음). "한 바퀴"는 시간으로 정한다 (75초 = 9/27 한 바퀴)
  local LAP_SEC="${1:-${LAP_SEC:-75}}" ok=0 i line v n m s
  PREP_HOLD=0 "$D/car_prep.sh" || exit 1
  say "차선 인식 확인 (경로 valid 80% 이상이 될 때까지, 최대 30초). 차를 2차선 출발 위치에 놓으세요"
  for i in $(seq 6); do
    line=$(docker exec $C bash -c "$CX; python3 $D/nodes/rate_probe.py 2>/dev/null" 2>/dev/null | grep "/lane_trajectory")
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
}

flash() {
  [ -e "$ACM" ] || die "아두이노($ACM) 없음. USB 연결 확인"
  [ -f "$FW_HEX" ] || die "펌웨어 파일 없음: $FW_HEX"
  docker exec $C bash -c 'pkill -9 -f "[b]oard_monitor.py"; pkill -INT -f "[s]erial_sender_node"' >/dev/null 2>&1
  sleep 1
  local U; U=$(port_users); [ -z "$U" ] || die "포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
  say "업로드: $FW_HEX"
  "$AVR/bin/avrdude" -C"$AVR/etc/avrdude.conf" -patmega2560 -cwiring -P$ACM -b115200 -D -Uflash:w:"$FW_HEX":i 2>&1 \
    | grep -E "bytes of flash verified|error|mismatch" || die "업로드 실패"
  sleep 2
  python3 -c "
import serial, time
s = serial.Serial('$ACM', 115200, timeout=0.3); t = time.time(); b = b''
while time.time() - t < 3: b += s.read(256)
s.close(); print(b.decode(errors='replace').strip().splitlines()[:2])"
}

shell() {
  container_up || die "컨테이너 시작 실패"
  if [ $# -gt 0 ]; then exec docker exec -i $C bash -c "$CX; $*"
  else exec docker exec -it $C bash -c "$CX; exec bash"; fi
}

[ "${BASH_SOURCE[0]}" = "$0" ] || return 0      # source 됐으면 (car_prep.sh / car_go.sh) 여기까지
case "$1" in
  stop)  stop "$2" ;;
  all)   PREP_HOLD=0 "$D/car_prep.sh" || exit 1; read -r -p $'\n차를 출발 위치에 놓고 엔터 = 출발, Ctrl+C = 취소 ' _
         exec "$D/car_go.sh" "${2:-$DRIVE_SEC}" ;;
  lap)   lap "$2" ;;
  flash) flash ;;
  shell) shift; shell "$@" ;;
  *)     sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
