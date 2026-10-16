"""벤치 조향 순서 시험: 명령 목록을 20Hz 로 각각 N초씩 보내고 마지막에 s0l0r0. 보드 메시지는 board_monitor 가 기록.
사용: python3 steer_sequence_test.py "s-7l0r0:5" "s0l1r1:5" "s7l0r0:5" "s0l1r1:5" """
import sys, time, serial
steps = [(a.split(":")[0], float(a.split(":")[1])) for a in sys.argv[1:]]
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.1; s.open()
try:
    for cmd, secs in steps:
        print(time.strftime("%H:%M:%S"), "send", cmd, "for", secs, "s", flush=True)
        t0 = time.time()
        while time.time() - t0 < secs:
            s.write((cmd + "\n").encode()); time.sleep(0.05)
finally:
    for _ in range(5): s.write(b"s0l0r0\n"); time.sleep(0.05)
    print(time.strftime("%H:%M:%S"), "stop sent", flush=True); s.close()
