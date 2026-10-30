# -*- coding: utf-8 -*-
"""주행 기록기 (구독만): 10 Hz 로 <기록폴더>/timeline.csv 에 한 줄씩.
경로(valid, a, b, c), 명령(조향, 속도), 실선/점선 검출 수, YOLO 결과 Hz, 그리고 느려짐 원인 구분용
cam_hz·cam_fail(webcam_pub 이 /tmp/pipeline/cam_hz 에 씀), cpu_temp_c, cpu_mhz(열 제한이면 뚝 떨어짐), yolo_cpu(%, 100 = 코어 하나).
cam_hz 가 떨어지면 웹캠 쪽, cam_hz 는 그대로인데 det_hz 와 cpu_mhz 가 떨어지면 CPU 열 제한.
사용: python3 run_monitor.py <기록폴더>   (pipeline.sh 가 띄움)"""
import csv, glob, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand


def read(path):
    with open(path) as f:
        return f.read()


def read_cam():
    try:
        hz, fail, ts = read("/tmp/pipeline/cam_hz").split()
        return (hz, fail) if time.time() - float(ts) < 3 else ("0.0", "")   # 3 초 넘게 안 바뀜 = 웹캠 노드 멈춤
    except (OSError, ValueError):
        return "", ""


def find_temp_zone():
    for z in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            if read(z + "/type").strip() == "x86_pkg_temp":
                return z + "/temp"
        except OSError:
            pass
    return None


def read_mhz():
    try:
        v = [float(l.split(":")[1]) for l in read("/proc/cpuinfo").splitlines() if l.startswith("cpu MHz")]
        return "%.0f" % (sum(v) / len(v)) if v else ""
    except (OSError, ValueError):
        return ""


def find_pid(pattern):
    for p in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            if pattern in open(p, "rb").read().decode(errors="replace"):
                return p.split("/")[2]
        except OSError:
            pass
    return None


def proc_ticks(pid):
    try:
        f = read("/proc/%s/stat" % pid).rsplit(")", 1)[1].split()
        return int(f[11]) + int(f[12])          # utime + stime (clock ticks)
    except (OSError, IndexError, ValueError, TypeError):
        return None


def main():
    out = sys.argv[1]
    rclpy.init(); n = Node("ab_run_monitor")
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    st = {"traj": None, "cmd": None, "det": (0, 0), "n_det": 0}
    n.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: st.__setitem__("traj", m), q)
    n.create_subscription(MotionCommand, "/topic_control_signal", lambda m: st.__setitem__("cmd", m), q)

    def on_det(m):
        st["det"] = (sum(d.class_name == "solid_line" for d in m.detections), sum(d.class_name == "dashed_line" for d in m.detections))
        st["n_det"] += 1
    n.create_subscription(DetectionArray, "/detections", on_det, q)
    f = open(out + "/timeline.csv", "w", newline=""); w = csv.writer(f)
    w.writerow(["t", "traj_valid", "a", "b", "c", "steer", "speed", "n_solid", "n_dashed", "det_hz",
                "cam_hz", "cam_fail", "cpu_temp_c", "cpu_mhz", "yolo_cpu"])
    t0 = last = time.time(); last_n = 0; hz = 0.0
    tick = os.sysconf("SC_CLK_TCK"); zone = find_temp_zone()
    ypid = None; y_last = None; y_t = t0; ycpu = ""; sys_t = 0.0; temp = mhz = ""
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            if now - last < 0.1:
                continue
            hz = 0.8 * hz + 0.2 * ((st["n_det"] - last_n) / (now - last)); last_n = st["n_det"]; last = now
            if now - sys_t >= 1.0:                  # 시스템 값은 1 초에 한 번 (기록기 자체 부하 최소)
                try:
                    temp = "%.0f" % (int(read(zone)) / 1000) if zone else ""
                except (OSError, ValueError):
                    temp = ""
                mhz = read_mhz()
                if ypid is None or proc_ticks(ypid) is None:
                    ypid = find_pid("nodes/yolov8_node.py"); y_last = None
                yt = proc_ticks(ypid) if ypid else None
                ycpu = "%.0f" % (100.0 * (yt - y_last) / tick / (now - y_t)) if (yt is not None and y_last is not None) else ""
                y_last, y_t, sys_t = yt, now, now
            t, c = st["traj"], st["cmd"]; cam = read_cam()
            w.writerow(["%.2f" % (now - t0), int(bool(t and t.valid)),
                        "%.4f" % t.a if t else "", "%.4f" % t.b if t else "", "%.4f" % t.c if t else "",
                        c.steering if c else "", c.left_speed if c else "", st["det"][0], st["det"][1], "%.1f" % hz,
                        cam[0], cam[1], temp, mhz, ycpu])
            f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
