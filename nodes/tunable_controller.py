# -*- coding: utf-8 -*-
"""
조정형 Stanley 제어기:  /lane_trajectory (y = a·x² + b·x + c)  →  /topic_control_signal (조향 -100~100, 왼쪽/오른쪽 PWM)

   δ = heading_gain·atan(y'(x0)) + atan(stanley_gain·y(x0) / (vehicle_speed_mps + softening_speed_mps))
       + curvature_gain·κ(x0) + i_gain·∫y(x0)dt                                   (x0 = 평가 지점, 아래)
   조향 = -100 · δ / max_steer_deg   (펌웨어: + 오른쪽. 경로 각도는 + 왼쪽이라 부호가 반대)

   - 평가 지점 x0: 2차선 front_axle_offset_m, 1차선 front_axle_offset_m_lane1 (없으면 front_axle_offset_m). 차선 = tune/lane.
     차선을 바꾸면 새 차선에 자리 잡을 때까지 두 값 중 큰 쪽, 그 뒤 lane_change_sec 동안 새 값으로 서서히 옮김. 11/05 가제보: 1차선 0.36 이면 반경 1.5~2.4 m 코너를 질러가 안쪽 실선 이탈,
     0.1 이면 이탈 없음 (2차선은 0.36 이 가장 나음). docs/gazebo_1105
   - 내륜차 보상: 경로를 0.5·축간거리²·κ 만큼 커브 바깥으로 옮겨서 계산 (뒷바퀴가 안쪽 선을 밟지 않게)
   - I 항: 직선(|κ| < i_curv_max)이고 끝까지 꺾지 않았을 때만 쌓고 ±i_max_deg 에서 자름. 경로를 0.5 초 넘게 잃으면 0
   - 경로가 lane_timeout_sec 넘게 없으면 정지 명령
   - 신호 정지: /detections 에서 빨간·노란불을 보고 횡단보도 줄무늬의 먼 쪽 끝(줄무늬마다 잰 것의 중앙값)이 앞범퍼에서
     light_stop_dist_m 안에 들어오면 정지(PWM 0). 횡단보도 가까운 끝은 앞범퍼 0.65 m 부터 화면 밖이라 기준으로 못 씀,
     먼 끝은 횡단보도 바로 앞에 서도 보임. 멈춘 뒤엔 초록불이 보일 때까지 유지.
     초록불로 출발한 뒤 LIGHT_COOLDOWN_SEC 동안은 다시 멈추지 않음 (같은 횡단보도). 0 = 끔

tune/tune.json (tune_gui.py 가 저장) 이 바뀌면 다음 주기(0.05 초)에 반영한다.
tune/go_until (car_go.sh 가 쓰는 주행 끝 시각) 전에만 명령을 보낸다. 꺼지면 0.5 초 동안 정지 명령, 그 뒤엔 아무것도 안 보냄.
tune/status.json 에 0.2 초마다 상태를 쓴다 (tune_gui.py 표시, car_go.sh 가 "go" 키로 제어기 확인).
사용: python3 nodes/tunable_controller.py [기록폴더]   (pipeline.sh 가 띄움. 기록폴더/tune_changes.log 에 설정 변경 기록)
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

from ipm import Ipm

D =os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tune")
TUNE, STATUS, GO, LANE = D + "/tune.json", D + "/status.json", D + "/go_until", D + "/lane"
STEER_MAX = 100          # 펌웨어 조향 명령 범위 ±100 (최대 조향의 1% 단위)
WHEELBASE = 0.56         # 축간거리 m (앞축 -0.23, 뒷축 -0.79 실측)
DT = 0.05                # 명령 주기 s
LIGHT_STOP_DIST_M = 1.3  # tune.json 에 light_stop_dist_m 이 없을 때 (11/04 횡단보도 바로 앞에 선 장면에서 먼 끝 1.27 m)
LIGHT_MIN_SCORE = 0.5    # 신호등·정지선 최소 확신도
LIGHT_FRAMES = 3         # 같은 신호가 이만큼 연달아 보여야 바뀐 것으로 봄 (한 장 잘못 잡은 것 무시)
LIGHT_FORGET_SEC = 1.0   # 신호등이 이만큼 안 보이면 색을 잊음 (멈춰 있을 때는 초록불이 보일 때까지 유지)
LIGHT_COOLDOWN_SEC = 3.0
CROSS_MAX_M = 3.0        # 횡단보도 먼 끝으로 믿는 최대 거리 (앞범퍼 m). 이 밖이나 음수는 버림 (11/04 트랙 밖 오검출이 -15.8 m)
SETTLE_C_M = 0.10        # 차선 변경 뒤 '자리 잡음' = 경로 옆 오차 |c| 이 안
SETTLE_B = 0.15          # 그리고 경로 방향 |b| 이 안
SETTLE_SEC = 0.5         # 이 시간 이어지면
SETTLE_MAX_SEC = 6.0     # 차선을 바꾼 뒤 이 시간이 지나면 자리 잡았다고 봄
BUMPER_X_M = -0.30       # 앞범퍼의 경로 x (lane_path_node 와 같음: x 0 = 카메라 바로 아래에서 0.84 m 앞, 렌즈는 범퍼 0.544 m 뒤)


class Tunable(Node):
    def __init__(self, run=""):
        super().__init__("tunable_controller")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE, depth=1)
        self.pub = self.create_publisher(MotionCommand, "/topic_control_signal", q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.create_subscription(DetectionArray, "/detections", self.on_det, q)
        self.ipm = Ipm()
        self.light = None; self.light_t = 0.0     # 확정된 신호 ("stop" = 빨강·노랑, "go" = 초록) 와 마지막으로 본 시각
        self.light_cand = (None, 0)               # (지금 보이는 신호, 연달아 본 횟수)
        self.cross_far = None                     # 횡단보도 먼 끝까지 앞범퍼 거리 m (안 보이면 None)
        self.hold = False                         # 신호 정지 중
        self.cool_until = 0.0
        self.go = False                           # 출발 스위치 (tick 이 갱신, on_det 은 주행 중일 때만 판단)
        self.run = run
        self.p = None; self.mtime = 0.0
        self.traj = None; self.traj_t = None      # 마지막 정상 경로와 받은 시각
        self.prev_steer = 0
        self.i_acc = 0.0                          # 옆오차 적분 (m·s)
        self.i_lost = 0.0                         # 경로 없던 시간 (s)
        self.zero_until = 0.0                     # 출발 스위치가 꺼진 뒤 이 시각까지 정지 명령
        self.n = 0
        self.lane = 2; self.lane_mt = None         # 달릴 차선 (tune/lane, 경로 노드와 같은 파일)
        self.lane_t = -1e9                        # 차선을 바꾼 시각 (monotonic)
        self.settled = True; self.settle_n = 0     # 새 차선에 자리 잡았나 (그 전엔 평가 지점을 줄이지 않음)
        self.x0 = None                            # 지금 평가 지점 (차선을 바꾸면 lane_change_sec 동안 새 값으로 옮김)
        self.reload(force=True)
        self.create_timer(DT, self.tick)

    def reload(self, force=False):
        try:
            mt = os.path.getmtime(TUNE)
            if not force and mt == self.mtime:
                return
            p = json.load(open(TUNE))
        except (OSError, ValueError):
            return                                # 슬라이더 창이 쓰는 도중이면 다음 주기에
        self.mtime = mt; self.p = p
        self.get_logger().info("tune loaded: %s" % json.dumps(p, ensure_ascii=False))
        if self.run:
            with open(os.path.join(self.run, "tune_changes.log"), "a") as f:
                f.write("%s %s\n" % (time.strftime("%H:%M:%S"), json.dumps(p, ensure_ascii=False)))

    def on_traj(self, m):
        self.traj, self.traj_t = (m, time.monotonic()) if m.valid else (None, None)

    def on_det(self, msg):
        """신호등 색을 정하고 횡단보도 먼 끝까지 거리를 재서 신호 정지를 판단한다."""
        now = time.monotonic()
        names = {d.class_name for d in msg.detections if d.score >= LIGHT_MIN_SCORE}
        seen = "stop" if names & {"traffic_light_red", "traffic_light_yellow"} else "go" if "traffic_light_green" in names else None
        if seen:
            n = self.light_cand[1] + 1 if self.light_cand[0] == seen else 1
            self.light_cand = (seen, n)
            if n >= LIGHT_FRAMES:
                self.light, self.light_t = seen, now
        elif not self.hold and now - self.light_t > LIGHT_FORGET_SEC:
            self.light = None
        far = []                                                  # 줄무늬마다 먼 쪽 끝 (사진에서 가장 위 점)
        for d in msg.detections:
            if d.class_name == "crosswalk_marking" and d.score >= LIGHT_MIN_SCORE and d.mask.data:
                self.ipm.setup(d.mask.height, d.mask.width)
                P = np.array([[q.x, q.y] for q in d.mask.data], np.float32)
                u, v = P[np.argmin(P[:, 1])]
                dist = float(self.ipm.to_m(np.array([u]), np.array([v]))[0][0]) - BUMPER_X_M
                if 0.0 < dist < CROSS_MAX_M:                      # 지평선 위·트랙 밖 오검출은 거리가 음수·엉뚱하게 나옴
                    far.append(dist)
        self.cross_far = float(np.median(far)) if far else None   # 한두 줄 잘못 잡혀도 괜찮게 중앙값
        limit = float(self.p.get("light_stop_dist_m", LIGHT_STOP_DIST_M))
        if not self.go:                                           # 주행 중일 때만 판단 (서 있을 땐 매 프레임 다시 걸리지 않게)
            return
        if self.hold:
            if self.light == "go":
                self.hold = False; self.cool_until = now + LIGHT_COOLDOWN_SEC
                self.get_logger().info("signal: green -> go")
        elif (limit > 0 and self.light == "stop" and now > self.cool_until
              and self.cross_far is not None and self.cross_far <= limit):
            self.hold = True
            self.get_logger().info("signal: red/yellow, crosswalk far end %.2f m ahead -> stop" % self.cross_far)

    def eval_x(self, p):
        """이 주기의 평가 지점. 차선별 값으로, 차선을 바꾸면 새 차선에 자리 잡을 때까지(경로 옆 오차 SETTLE_C_M, 방향 SETTLE_B 안이
        SETTLE_SEC 이어짐, 최대 SETTLE_MAX_SEC) 두 값 중 큰 쪽(덜 민감)을 쓰고, 그 뒤 lane_change_sec 동안 일정한 빠르기로 새 값으로 옮긴다.
        (11/05 가제보: 차가 아직 옮겨 가는 중에 0.1 로 줄이면 새 차선을 지나쳐 안쪽 실선을 넘음)"""
        sec = max(float(p.get("lane_change_sec", 2.0)), 0.1)
        try:
            mt = os.path.getmtime(LANE)
            if mt != self.lane_mt:
                first = self.lane_mt is None
                self.lane_mt = mt; v = int(open(LANE).read().strip())
                if v in (1, 2) and v != self.lane:
                    self.lane = v
                    if not first: self.lane_t = time.monotonic(); self.settled = False; self.settle_n = 0
        except (OSError, ValueError):
            pass
        x2 = float(p["front_axle_offset_m"]); x1 = float(p.get("front_axle_offset_m_lane1", x2))
        want = x1 if self.lane == 1 else x2
        if self.x0 is None:
            self.x0 = want
        now = time.monotonic()
        if now - self.lane_t < SETTLE_MAX_SEC and not self.settled:
            t = self.traj
            ok = now - self.lane_t >= sec and t is not None and abs(t.c) < SETTLE_C_M and abs(t.b) < SETTLE_B
            self.settle_n = self.settle_n + 1 if ok else 0
            self.settled = self.settle_n * DT >= SETTLE_SEC
            if not self.settled:
                want = max(want, self.x0)
        rate = abs(x2 - x1) / sec
        self.x0 += float(np.clip(want - self.x0, -rate * DT, rate * DT)) if rate > 0 else want - self.x0
        return self.x0

    def steer(self, p, t):
        """정상 경로 t → (조향 -100~100, 조향각 rad, 내륜차 보상으로 옮긴 거리 m)."""
        x = self.eval_x(p)
        a, b = float(t.a), float(t.b)
        shift = -0.5 * WHEELBASE ** 2 * (2.0 * a / (1.0 + (2.0 * a * x + b) ** 2) ** 1.5)
        c = float(t.c + shift)
        dy = 2.0 * a * x + b
        cte = a * x * x + b * x + c                                   # x0 에서 경로가 차보다 왼쪽에 있는 거리
        curvature = 2.0 * a / np.power(1.0 + dy * dy, 1.5)
        delta = float(float(p["heading_gain"]) * np.arctan(dy)
                      + np.arctan(float(p["stanley_gain"]) * cte / (abs(float(p["vehicle_speed_mps"])) + float(p["softening_speed_mps"])))
                      + float(p["curvature_gain"]) * curvature)
        ki = float(p["i_gain"]); imax = math.radians(float(p["i_max_deg"]))
        self.i_lost = 0.0
        if ki > 0:
            if abs(curvature) < p["i_curv_max"] and abs(self.prev_steer) < STEER_MAX - 1:
                self.i_acc += cte * DT
            self.i_acc = float(np.clip(self.i_acc, -imax / ki, imax / ki))
            delta += ki * self.i_acc
        else:
            self.i_acc = 0.0
        norm = float(np.clip(delta / max(math.radians(p["max_steer_deg"]), 1e-6), -1.0, 1.0))
        return int(round(-norm * STEER_MAX)), delta, shift

    def tick(self):
        self.reload()
        p = self.p
        stale = self.traj is None or time.monotonic() - self.traj_t > p["lane_timeout_sec"]
        try:
            go = time.time() < float(open(GO).read())
        except (OSError, ValueError):
            go = False
        self.go = go
        if not go:
            self.hold = False                     # 주행이 끝나면 신호 정지도 풂 (다음 출발에 남지 않게)
        if stale or self.hold:
            self.i_lost += DT
            if self.i_lost > 0.5:
                self.i_acc = 0.0
            steer, pwm, delta, shift = 0, 0, None, 0.0
        else:
            steer, delta, shift = self.steer(p, self.traj)
            pwm = int(np.clip(round(p["pwm"]), 0, 255))
        self.prev_steer = steer
        if go:
            self.zero_until = time.monotonic() + 0.5
            m = MotionCommand(); m.steering, m.left_speed, m.right_speed = steer, pwm, pwm
            self.pub.publish(m)
        elif time.monotonic() < self.zero_until:
            self.pub.publish(MotionCommand())     # 기본값 0, 0, 0 = 정지
        self.n += 1
        if self.n % 4 == 0:
            st = {"time": time.time(), "steer": steer, "pwm": pwm, "valid": not stale,
                  "c": None if stale else round(self.traj.c, 3), "b": None if stale else round(self.traj.b, 3),
                  "shift": round(shift, 3), "delta_deg": None if delta is None else round(math.degrees(delta), 1),
                  "i_deg": round(math.degrees(float(p["i_gain"]) * self.i_acc), 2), "go": go,
                  "light": self.light, "hold": self.hold, "cross_far": None if self.cross_far is None else round(self.cross_far, 2),
                  "lane": self.lane, "x0": None if self.x0 is None else round(self.x0, 3)}
            try:
                with open(STATUS + ".tmp", "w") as f:
                    json.dump(st, f)
                os.replace(STATUS + ".tmp", STATUS)
            except OSError:
                pass


def main():
    rclpy.init()
    n = Tunable(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():                            # 주행 중 오류는 그대로, 끄는 중(Ctrl+C 로 ROS 가 먼저 닫힘)에 난 것은 무시
            raise
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
