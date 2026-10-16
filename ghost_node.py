# -*- coding: utf-8 -*-
"""
=====================================================================================
 고스트 노드 - 그림자 모드 (ghost_node.py, 2026-10-16)
=====================================================================================

지금 주행(H → tunable_controller)은 그대로 둔다. 같은 입력을 받아 고스트 결과를 '기록만' 한다. 조향에 안 쓴다.

   /detections (YOLO)           → 경계 점 (ghost_view.boundaries, ground_model: 실측 + f 540.4 + yaw 4.96°)
   board.log 의 ADC (조향 실제 위치) + /topic_control_signal 의 PWM (달리는 중인지)
                                → 고스트 이동 (vehicle.yaml motion: 조향 실측, 언더스티어 표, 속도 0.71 m/s)
   lane_map.LaneMap             → 사진마다 스캔 정합으로 고스트 위치 보정, 앞바퀴 축 기준 차로 오차

사진은 YOLO 지연(중앙값 약 80 ms)만큼 늦게 온다. 그래서 고스트 위치 기록을 들고 있다가 사진이 찍힌 시각의
위치로 점을 쌓고, 그때 고친 양을 지금 위치에도 똑같이 적용한다.

출력
   /ghost/trajectory (LaneTrajectory) : 원점 = 앞바퀴 축 가운데. c = 차로 중앙이 왼쪽에 있는 거리 (m),
                                        b = tan(차로 방향), a = 곡률/2, valid = 답이 있을 때. fit_error_m = 쓴 점의 최근성(초)
   <기록폴더>/ghost.csv : 20 Hz 한 줄씩 (시각, 고스트 위치, 앞바퀴 오차, ADC, 보정량, 계산 시간)

실행 (car_prep.sh 뒤, 컨테이너 안):  python3 ~/jacapdi-docker/ghost_node.py <기록폴더>   (ghost_start.sh 가 대신 해 줌)
"""
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, D)
from ghost_view import boundaries, REAR_AXLE, FRONT_AXLE   # noqa: E402
from lane_map import LaneMap                               # noqa: E402
from vehicle_motion import delta_of_adc, effective_delta  # noqa: E402

RUN = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "/tmp"
M = yaml.safe_load(open(os.path.join(D, "vehicle.yaml")))["motion"]
WB = 0.56
FA = FRONT_AXLE - REAR_AXLE                               # 앞바퀴 축 = 뒷바퀴 축 기준 0.56 m


def compose(p, q):
    """2D 자세 합성 p∘q (p 좌표계에서 q)."""
    x, y, t = p; c, s = math.cos(t), math.sin(t)
    return (x + c * q[0] - s * q[1], y + s * q[0] + c * q[1], t + q[2])


def inverse(p):
    x, y, t = p; c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


class GhostNode(Node):
    def __init__(self):
        super().__init__("ghost_node")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(DetectionArray, "/detections", self.on_det, q)
        self.create_subscription(MotionCommand, "/topic_control_signal", self.on_cmd, q)
        self.pub = self.create_publisher(LaneTrajectory, "/ghost/trajectory", q)
        self.map = LaneMap(wheelbase=WB)
        self.pwm = 0; self.adc = float(M["adc_center"]); self.adc_age = None
        self.board = os.path.join(RUN, "board.log")
        self.hist = []                                     # (시각, X, Y, TH) 최근 2 초
        self.t_last = None; self.last_match = None; self.det_ms = 0.0; self.n_det = 0
        self.f = open(os.path.join(RUN, "ghost.csv"), "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(["t", "X", "Y", "TH_deg", "adc", "pwm", "valid", "e_front_m", "psi_deg", "kap", "sides",
                         "newest_age_s", "n_pts", "match_dx", "match_dy", "match_dth_deg", "det_ms"])
        self.create_timer(0.05, self.tick)
        self.get_logger().info("ghost (shadow) start: motion %s, log %s" % (M, RUN))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.pwm = max(m.left_speed, m.right_speed)

    def read_adc(self):
        """board.log 마지막 ADC (펌웨어가 보내는 실제 조향 위치)."""
        try:
            with open(self.board, "rb") as f:
                f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 400)); lines = f.read().decode(errors="replace").splitlines()
        except OSError:
            return
        for l in reversed(lines):
            if "ADC=" in l:
                try:
                    self.adc = float(l.split("ADC=")[1].split()[0]); return
                except ValueError:
                    return

    # --- 20 Hz: 고스트 이동, 답 내보내기, 기록 ---
    def tick(self):
        t = self.now(); self.read_adc()
        if self.t_last is not None:
            v = M["speed_mps"] * min(1.0, self.pwm / 255.0)   # PWM → 속도: 255 에서만 잼 (그 아래는 비례로 둠, 미검증)
            d_eff = effective_delta(delta_of_adc(self.adc))     # 언더스티어 표 반영 (vehicle.yaml)
            self.map.move(v, d_eff, t - self.t_last)
        self.t_last = t
        self.hist.append((t, self.map.X, self.map.Y, self.map.TH))
        self.hist = [h for h in self.hist if t - h[0] <= 2.0]
        res = self.map.errors_at(FA)
        msg = LaneTrajectory(); msg.header.stamp = self.get_clock().now().to_msg(); msg.valid = res is not None
        if res:
            msg.c = res["e"]; msg.b = math.tan(res["psi"]); msg.a = 0.5 * res["kap"]
            msg.x_min_m, msg.x_max_m = -0.4, 0.8; msg.fit_error_m = res["newest_age"] or 0.0; msg.num_points = res["n"]
        self.pub.publish(msg)
        mt = self.last_match or (None, None, None)
        self.w.writerow(["%.3f" % t, "%.4f" % self.map.X, "%.4f" % self.map.Y, "%.2f" % math.degrees(self.map.TH),
                         "%.0f" % self.adc, self.pwm, int(res is not None),
                         *(("%.4f" % res["e"], "%.2f" % math.degrees(res["psi"]), "%.3f" % res["kap"], res["sides"],
                            "%.2f" % (res["newest_age"] or 0), res["n"]) if res else ("",) * 6),
                         *(("%.4f" % mt[0], "%.4f" % mt[1], "%.2f" % math.degrees(mt[2])) if mt[0] is not None else ("",) * 3),
                         "%.0f" % self.det_ms])
        self.last_match = None
        if int(t * 20) % 40 == 0:
            self.f.flush()

    # --- 사진(YOLO 결과): 찍힌 시각의 고스트 위치로 정합 + 쌓기, 고친 양을 지금 위치에도 ---
    def on_det(self, msg):
        t0 = time.monotonic()
        ts = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        shape = next(((d.mask.height, d.mask.width) for d in msg.detections if d.mask.height > 0), None)
        if shape is None or not self.hist:
            return
        h, w = shape
        dets = [(d.class_name, d.score, [[p.x, p.y] for p in d.mask.data]) for d in msg.detections]
        Dp, Sp = boundaries(dets, h, w)
        if not len(Dp) and not len(Sp):
            return
        # 사진 시각의 고스트 위치 (기록에서 가장 가까운 것)
        i = int(np.argmin([abs(hh[0] - ts) for hh in self.hist])); _, X, Y, TH = self.hist[i]
        cur = (self.map.X, self.map.Y, self.map.TH); old = (X, Y, TH)
        self.map.X, self.map.Y, self.map.TH = old
        self.map.add(Dp + [-REAR_AXLE, 0] if len(Dp) else None, Sp + [-REAR_AXLE, 0] if len(Sp) else None)
        new = (self.map.X, self.map.Y, self.map.TH)
        corr = compose(new, inverse(old))                      # 세계 좌표에서의 보정 (사진 시각 기준)
        self.map.X, self.map.Y, self.map.TH = compose(corr, cur)
        self.hist = [(hh[0], *compose(corr, hh[1:])) if hh[0] >= ts else hh for hh in self.hist]
        self.last_match = self.map.last_match
        self.det_ms = 1000 * (time.monotonic() - t0); self.n_det += 1


def main():
    rclpy.init(); n = GhostNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.f.close(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
