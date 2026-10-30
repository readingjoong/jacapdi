# -*- coding: utf-8 -*-
"""아두이노 메시지 기록기 (읽기만, 명령은 안 보냄). 줄마다 시각을 붙여 <로그파일> 에 쓴다 (부팅 배너, IDLE ADC, FAULT).
포트를 열 때 보드가 한 번 리셋된다(DTR). 이 기록기가 포트를 계속 열고 있어서 나중에 serial_sender 가 열어도 리셋되지 않는다.
사용: python3 board_monitor.py <로그파일>   (car_prep.sh 가 띄움)"""
import sys, time
import serial

out = open(sys.argv[1], "a", buffering=1)
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.5; s.open()
out.write("%s monitor start\n" % time.strftime("%H:%M:%S"))
buf = b""
try:
    while True:
        try:
            buf += s.read(256)
        except serial.SerialException as e:   # 다른 프로그램(정지 명령 등)이 같은 포트를 잠깐 열면 생김. 다시 열면 리셋되므로 계속 읽기만
            out.write("%s monitor read error (ignored): %s\n" % (time.strftime("%H:%M:%S"), e))
            time.sleep(0.2)
            continue
        *lines, buf = buf.split(b"\n")
        for line in lines:
            out.write("%s %s\n" % (time.strftime("%H:%M:%S"), line.decode(errors="replace").strip()))
except KeyboardInterrupt:
    pass
