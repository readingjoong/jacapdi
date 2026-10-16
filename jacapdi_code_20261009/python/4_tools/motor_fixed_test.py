"""고정 명령 모터 시험 (벤치용). 사용: python3 motor_fixed_test.py <steering> <pwm> <seconds>
s<steering>l<pwm>r<pwm> 을 20Hz 로 <seconds> 동안 보내고, 끝나면 s0l0r0 을 5번 보낸다. Ctrl+C 에도 정지 명령."""
import sys, time, serial
st, pwm, secs = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.1; s.open()
cmd = ("s%dl%dr%d\n" % (st, pwm, pwm)).encode()
print(time.strftime("%H:%M:%S"), "send", cmd.strip().decode(), "for", secs, "s", flush=True)
t0 = time.time(); n = 0
try:
    while time.time() - t0 < secs:
        s.write(cmd); n += 1; time.sleep(0.05)
finally:
    for _ in range(5): s.write(b"s0l0r0\n"); time.sleep(0.05)
    print(time.strftime("%H:%M:%S"), "stop sent after %d commands (%.1fs)" % (n, time.time() - t0), flush=True)
    s.close()
