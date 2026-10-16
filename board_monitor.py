"""아두이노 보드 메시지 읽기 전용 기록기 (명령을 보내지 않음). 사용: python3 board_monitor.py <로그파일>
FAULT / 부팅 배너(DRIVING_TEAM_CAL_V1) 를 시각과 함께 남긴다. 포트를 열 때 보드가 한 번 리셋된다(DTR)."""
import sys, time, serial
out = open(sys.argv[1], "a", buffering=1)
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.5; s.open()
out.write("%s monitor start\n" % time.strftime("%H:%M:%S"))
buf = b""
try:
    while True:
        try:
            buf += s.read(256)
        except serial.SerialException as e:
            # 다른 프로그램(정지 명령 전송 등)이 같은 포트를 잠깐 열면 생기는 오류 -> 기록만 하고 계속 읽는다
            # (포트를 다시 열면 보드가 리셋되므로 다시 열지 않음)
            out.write("%s monitor read error (ignored): %s\n" % (time.strftime("%H:%M:%S"), e))
            time.sleep(0.2)
            continue
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            out.write("%s %s\n" % (time.strftime("%H:%M:%S"), line.decode(errors="replace").strip()))
except KeyboardInterrupt:
    pass
