# -*- coding: utf-8 -*-
"""수동 주행 + 학습 사진 수집 (호스트에서 실행, root 필요 없음).
보드에 manual_drive/driving 펌웨어(9600 baud, 조향 ±7단계)를 올린 뒤:  python3 ~/jacapdi-docker/manual_drive/data_collection.py
주행 파이프라인(car_prep.sh)이 켜져 있으면 포트를 같이 써서 안 되니 먼저 끈다.

키 (이 터미널 창에서, 엔터 없이):
   w / s   앞으로 200 / 뒤로 200 (누르면 그 속도로 계속 감, r 로 정지)
   a / d   누르고 있는 동안 조향 왼쪽 -5 / 오른쪽 +5 단계. 둘 다 안 누르면 0
           (터미널은 키를 뗀 걸 못 알려줘서 키 반복이 끊기면 뗀 것으로 봄: 짧게 톡 누르면 0.7 초, 누르고 있다 떼면 0.2 초 안에 0.
            키 반복은 마지막에 누른 키만 되므로 a 를 누른 채 다른 키를 누르면 a 를 다시 눌러야 함)
   r       정지
   c       사진 저장 → manual_drive/Collected_Datasets/<시작 시각>/<번호>_steer:<조향>_left_speed:<왼쪽>_right_speed:<오른쪽>.png
   f       종료 (Ctrl+C 도 같음). 끝날 때 정지 명령을 보낸다
펌웨어는 마지막 명령을 계속 유지하므로(시간 초과 정지 없음) 이 창을 강제로 죽이면 차가 계속 달린다. 꼭 f 나 Ctrl+C 로 끈다."""
import glob, os, select, sys, termios, time, tty
from datetime import datetime

import cv2
import serial

HERE = os.path.dirname(os.path.realpath(__file__))
BAUD = 9600                 # driving.ino 의 Serial.begin(9600)
SPEED = 200                 # w / s 속도 (펌웨어 최대 250)
STEER = 5                   # a / d 조향 단계 (펌웨어 최대 ±7)
PERIOD = 0.1                # 키 확인·명령 전송 주기 (s)
KEY_FIRST = 0.7             # 처음 누른 뒤 이 시간 안에 키 반복이 오면 누르고 있는 것 (키 반복 시작 0.5 초)
KEY_REPEAT = 0.15           # 키 반복 중엔 이 시간 동안 안 들어오면 뗀 것 (반복 간격 0.03 초)


def is_down(t, now):
    """t = (처음 누른 시각, 마지막으로 들어온 시각) → 아직 누르고 있나."""
    first, last = t
    return now - last < (KEY_FIRST if last == first else KEY_REPEAT)


def find_camera():
    """C920 의 영상 장치 번호 (없으면 0)."""
    for p in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            if open(p + "/index").read().strip() == "0" and "C920" in open(p + "/name").read():
                return int(p.rsplit("video", 1)[1])
        except (OSError, ValueError):
            pass
    return 0


def main():
    port = (sorted(glob.glob("/dev/ttyACM*")) or ["/dev/ttyACM0"])[0]
    ser = serial.Serial(port, BAUD, timeout=1)
    time.sleep(1)                                   # 포트를 열면 보드가 리셋됨
    send = lambda s, l, r: ser.write(("s%dl%dr%d\n" % (s, l, r)).encode())
    cam = find_camera()
    cap = cv2.VideoCapture(cam, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1); cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    out = os.path.join(HERE, "Collected_Datasets", datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    print("보드 %s, 카메라 /dev/video%d%s" % (port, cam, "" if cap.isOpened() else " (못 열음: 사진 저장 안 됨)"))
    print("키: w/s 속도, a/d 조향, r 정지, c 사진, f 종료")
    steer = speed = n = 0
    held = {}                                       # a / d → (처음 누른 시각, 마지막으로 들어온 시각)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())               # 엔터 없이 한 글자씩 (Ctrl+C 는 그대로)
    try:
        while True:
            t0 = time.time()
            keys = set()
            while select.select([sys.stdin], [], [], 0)[0]:
                keys.add(sys.stdin.read(1).lower())
            cap.grab()                              # 늘 최신 사진을 들고 있게
            if "f" in keys:
                print("f: 종료")
                break
            before = (steer, speed)
            if "w" in keys: speed = SPEED
            if "s" in keys: speed = -SPEED
            if "r" in keys: speed = 0
            for k in keys & {"a", "d"}:
                held[k] = (held[k][0], t0) if k in held and is_down(held[k], t0) else (t0, t0)
            down = [k for k in held if is_down(held[k], t0)]
            k = max(down, key=lambda k: held[k][1], default=None)   # 둘 다면 나중에 누른 쪽
            steer = {"a": -STEER, "d": STEER}.get(k, 0)
            if "c" in keys:
                ok, frame = cap.retrieve()
                if ok:
                    os.makedirs(out, exist_ok=True)
                    name = "%s/%d_steer:%d_left_speed:%d_right_speed:%d.png" % (out, n, steer, speed, speed)
                    cv2.imwrite(name, frame); n += 1
                    print("사진 저장 %d: %s" % (n, os.path.basename(name)))
                else:
                    print("사진 실패 (카메라 확인)")
            send(steer, speed, speed)
            if (steer, speed) != before:
                print("조향 %+d  속도 %+d" % (steer, speed))
            time.sleep(max(0.0, PERIOD - (time.time() - t0)))
    except KeyboardInterrupt:
        print("Ctrl+C: 종료")
    finally:
        for _ in range(3):
            send(0, 0, 0); time.sleep(0.05)
        ser.close(); cap.release()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("정지 명령 보냄. 사진 %d장%s" % (n, (" → " + out) if n else ""))


if __name__ == "__main__":
    main()
