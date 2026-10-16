# 공통 설정 (car_*.sh 가 source). 여기 값만 바꾸면 된다.
ALGO="${ALGO:-F}"                                            # 경로 알고리즘 A/B/C/D/E/F (E = B + 정지선 무시 + 점선 민감 + 더 멀리 봄, F = E + 경로 튀면 0.5초 무시). 예전대로: ALGO=B CONF=0.5
CONF="${CONF:-0.3}"                                          # YOLO threshold (E 는 0.3: 점선 놓치지 않게. 실선은 E 안에서 0.5로 다시 거름)
TUNE="${TUNE:-1}"                                             # 1: 슬라이더로 조정 가능한 제어기(tunable_controller.py) 사용, 0: 팀 Stanley 그대로
if [ "$TUNE" = 1 ]; then CTRL_YAML="${CTRL_YAML:-$HOME/jacapdi-docker/drive_controller_tunable.yaml}"
else CTRL_YAML="${CTRL_YAML:-$HOME/jacapdi-docker/drive_controller.yaml}"; fi   # 마지막 성공 주행 설정 (PWM 240, 조향 7)
DRIVE_SEC="${DRIVE_SEC:-120}"                                # car_go 기본 주행 시간(초)
FW_HEX="${FW_HEX:-$HOME/jacapdi-live-car-2026-09-24/arduino/driving_team_cal_1009c/driving_team_cal_1009c.ino.hex}"
FW_NAME="${FW_NAME:-DRIVING_TEAM_CAL_1009C}"                  # 보드 부팅 메시지에 있어야 할 이름
ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1); ACM=${ACM:-/dev/ttyACM0}   # 호스트의 아두이노 포트 (ttyACM0 이 아닐 수도 있음)
C=jacapdi-humble
D=$HOME/jacapdi-docker
AVR=$HOME/Downloads/arduino-1.8.19/hardware/tools/avr

say()  { printf '\033[1;32m[car]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[car]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[car] %s\033[0m\n' "$*"; exit 1; }

port_users() {   # /dev/ttyACM0 를 잡고 있는 호스트 프로세스 (Arduino IDE 시리얼 모니터 등)
  for p in /proc/[0-9]*; do ls -l "$p/fd" 2>/dev/null | grep -q "$(basename "$ACM")" && echo "$(basename "$p") $(tr '\0' ' ' < "$p/cmdline" | cut -c1-60)"; done
}

send_stop() {    # 모든 송신기 끄고 s0l0r0 5번 (컨테이너 안에서)
  docker exec "$C" bash -c 'pkill -INT -f "timeout -s INT"; pkill -INT -f "serial_communication_pkg/serial_sender_node"; sleep 1.5;
    for p in $(pgrep -f "serial_communication_pkg/serial_sender_node"); do kill -9 $p; done' >/dev/null 2>&1
  docker exec "$C" python3 -c "
import serial, time
s = serial.Serial(); s.port = '/dev/ttyACM0'; s.baudrate = 115200; s.timeout = 0.1; s.dsrdtr = False; s.open()
for _ in range(5): s.write(b's0l0r0\n'); time.sleep(0.05)
s.close()" >/dev/null 2>&1 && say "정지 명령 s0l0r0 전송" || warn "정지 명령 전송 실패 (아두이노 연결 확인). 펌웨어가 0.3초 뒤 스스로 정지함"
}
