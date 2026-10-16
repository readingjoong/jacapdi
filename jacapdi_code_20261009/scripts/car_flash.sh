#!/usr/bin/env bash
# 아두이노에 주행 펌웨어 업로드 (필요할 때만). 사용: ~/jacapdi-docker/car_flash.sh
source "$(dirname "$0")/car_common.sh"
[ -e "$ACM" ] || die "아두이노($ACM) 없음. USB 연결 확인"
[ -f "$FW_HEX" ] || die "펌웨어 파일 없음: $FW_HEX"
docker exec "$C" bash -c 'pkill -9 -f "board_monitor.py"; pkill -INT -f serial_sender_node' >/dev/null 2>&1 || true
sleep 1
U=$(port_users); [ -z "$U" ] || die "포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
say "업로드: $FW_HEX"
"$AVR/bin/avrdude" -C"$AVR/etc/avrdude.conf" -patmega2560 -cwiring -P$ACM -b115200 -D -Uflash:w:"$FW_HEX":i 2>&1 \
  | grep -E "bytes of flash verified|error|mismatch" || die "업로드 실패"
sleep 2
python3 -c "
import serial, time
s = serial.Serial('$ACM', 115200, timeout=0.3); t = time.time(); b = b''
while time.time() - t < 3: b += s.read(256)
s.close(); print(b.decode(errors='replace').strip().splitlines()[:2])"
