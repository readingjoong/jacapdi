# -*- coding: utf-8 -*-
"""
=====================================================================================
 조정형 Stanley 제어기  (tunable_controller.py, 2026-10-09)
 "경로 곡선 → 핸들을 몇 단계 꺾을지 + 뒷바퀴를 얼마나 세게 돌릴지"
=====================================================================================

   /lane_trajectory (경로 a,b,c)  ──▶  이 노드  ──▶  /topic_control_signal (조향 단계, 왼쪽 PWM, 오른쪽 PWM)
                                         ▲
                     tune/tune.json (슬라이더 창 tune_gui.py 가 저장) 을 0.05 초마다 확인해서 바뀌면 바로 반영

계산식은 팀 코드(decision_making_pkg 의 LaneKeepingController = Stanley 제어)를 그대로 가져다 쓴다.
기본값이면 팀 stanley_controller_node 와 완전히 같은 명령이 나온다 (무작위 경로 1만 개로 확인).

Stanley 제어 한 줄 요약
   조향각 δ = heading_gain × (경로 방향 오차)  +  atan( stanley_gain × (옆으로 벗어난 거리) / (속도 + 저속완화) )
            + curvature_gain × (경로 곡률)
   - "옆으로 벗어난 거리" = 평가 지점(front_axle_offset_m)에서 경로가 차보다 얼마나 옆에 있나 (y)
   - "경로 방향 오차"     = 그 지점에서 경로가 차 앞 방향과 이루는 각도
   - "곡률"               = 경로가 얼마나 휘었나 (커브 미리 대응)
   그 다음 δ 를 -100 ~ +100 (최대 조향의 1% 단위) 정수로 바꾼다 (펌웨어 1010f 규칙: + 오른쪽, - 왼쪽).
   [10/10] 예전엔 -7 ~ +7 단계였다. 슬라이더 값(max_steering_step, steer_trim, steer_rate, lane_guard_push, pwm_by_step)은
   지금도 예전 '단계' 단위 그대로 쓰고, 여기서 1 단계 = 100/7 ≈ 14.3 으로 바꿔서 보낸다 (소수 단계도 됨).

이 노드에서 추가한 것
   - path_offset_m  : 경로 전체를 옆으로 옮기기 (+ 왼쪽).  오른쪽 실선을 자꾸 밟을 때
   - inside_bias_m  : [인코스] 커브에서만 경로를 커브 안쪽으로 당기기. 많이 휠수록 많이(최대 이 값).
                      바깥으로 크게 돌지 않고 안쪽 라인을 타서 랩타임 단축. 너무 크면 점선/실선을 밟는다.
   - steer_trim     : 직진이 한쪽으로 흐르면 조향 단계를 일정하게 보정
   - pwm_by_step    : 조향 단계(0~7)마다 뒷바퀴 PWM 따로 (커브에서 감속)
   - steer_rate     : [급발진 방지] 0.05 초마다 조향 단계를 최대 몇 단계까지 바꿀지 (0 = 제한 없음)
                      1 이면 0 → 7 까지 0.35 초. 조향이 확 튀지 않고 부드럽게 따라간다.
   - motor_balance  : [좌우 모터 보정] 직선에서 한쪽으로 흐를 때. + 면 왼쪽 뒷바퀴 PWM 을 그만큼 줄임(오른쪽 쏠림 보정),
                      - 면 오른쪽 뒷바퀴를 줄임(왼쪽 쏠림 보정). 0 = 둘 다 같은 PWM
   - lane_guard_left_m: [선 넘기 방지 - 왼쪽(점선) 쪽 기준] 따로 정하고 싶을 때 (0 = lane_guard_m 과 같게).
                      인코스로 커브 안쪽(왼쪽)에 붙을 때 너무 일찍 막지 않도록 오른쪽보다 크게 둔다.
   - inside_bias_right_m: [인코스 - 오른쪽 커브만] 0 이 아니면 오른쪽 커브에서는 inside_bias_m 대신 이 값.
                      - 면 오른쪽 커브를 바깥(왼쪽)으로 크게 돈다 = 늦게 꺾음 (왼쪽 커브 직후 오른쪽 커브에서 실선 안쪽을 질러가는 것 방지)
   - lane_guard_push: [선 넘기 방지 - 되돌리기] 선 넘기 방지 기준을 넘으면 반대쪽으로 최소 이 단계만큼은 꺾는다.
                      0 = 막기만 함(예전). 2 = 실선 쪽으로 치우치면 최소 왼쪽 2단계.
   - stall_boost_pwm: [멈춤 탈출] 경로가 1초 동안 거의 안 변하면(= 차가 멈춤, 예: 끝까지 꺾고 낮은 PWM)
                      0.5 초 동안 이 PWM 으로 세게 민다. 0 = 끔. (움직일 땐 경로가 1초에 6 cm 넘게 변함, 멈추면 0.3 cm 미만)
   - i_gain         : [10/16 Stanley I 항] 차로 중심에서 한쪽으로 계속 벗어나 있으면(옆오차 적분) 그만큼 더 꺾는다.
                      조향각 += i_gain × ∫(옆오차 m) dt   (rad).  0 = 끔 (예전과 같음). steer_trim/path_offset 을 자동으로 맞추는 역할.
                      [windup 방지] 직선(|곡률| < i_curv_max)이고 끝까지 안 꺾었을 때만 쌓는다. 커브/경로 없음이면 그대로 두고,
                      경로를 0.5초 넘게 잃으면 0 으로. I 항 크기는 ±i_max_deg 도에서 자른다.
   - lane_guard_m   : [선 넘기 방지] 차가 차로 가운데에서 이 거리(m)보다 왼쪽에 있으면 왼쪽으로 더 못 꺾게,
                      오른쪽에 있으면 오른쪽으로 더 못 꺾게 막는다 (0 = 끔). 점선/실선 넘는 것 방지.

사용 (car_prep.sh 가 대신 실행):  python3 tunable_controller.py [기록폴더]
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
from interfaces_pkg.msg import LaneTrajectory, MotionCommand
from decision_making_pkg.lib.lane_keeping_controller import LaneKeepingController   # 팀 Stanley 계산식

TUNE = os.path.expanduser("~/jacapdi-docker/tune/tune.json")       # 슬라이더 값 (사람이 바꿈)
STATUS = os.path.expanduser("~/jacapdi-docker/tune/status.json")   # 지금 상태 (슬라이더 창에 표시)
GO = os.path.expanduser("~/jacapdi-docker/tune/go_until")          # [1017] 출발 스위치: 주행 끝 시각(epoch 초). car_go.sh 가 쓰고 지움
RUN = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else ""   # 기록 폴더
FW_FULL = 100            # 펌웨어(1010f)가 받는 조향 최대값 (s-100 ~ s100, 1% 단위)
OLD_STEPS = 7            # 슬라이더는 예전 단계(0~7) 단위
K = FW_FULL / OLD_STEPS  # 예전 1 단계 = 14.3
CURV_FULL = 0.6          # 인코스: 곡률이 이 값(1/m, 반경 약 1.7 m) 이상이면 inside_bias_m 를 다 씀

# 새로 생긴 설정이 예전 tune.json 에 없을 때 쓸 기본값 (기본값이면 동작이 예전과 같음)
DEFAULTS = {"inside_bias_m": 0.0, "steer_rate": 0, "lane_guard_m": 0.0, "motor_balance": 0, "stall_boost_pwm": 0, "lane_guard_left_m": 0.0, "lane_guard_push": 0, "inside_bias_right_m": 0.0, "steering_sign": -1.0,
            "i_gain": 0.0, "i_max_deg": 3.0, "i_curv_max": 0.3}


class Tunable(Node):
    def __init__(self):
        super().__init__("tunable_controller")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE, depth=1)
        self.pub = self.create_publisher(MotionCommand, "/topic_control_signal", q)   # 시리얼 송신기가 받는 통로
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.p = None; self.mtime = 0.0; self.ctrl = None
        self.traj = None; self.traj_t = None      # 마지막으로 받은 정상 경로와 받은 시각
        self.n = 0
        self.prev_steer = 0                       # 직전에 보낸 조향 단계 (급발진 방지용)
        self.i_acc = 0.0                          # [I 항] 옆오차 적분 (m·s)
        self.i_lost = 0.0                         # [I 항] 경로 없던 시간 (s)
        self.hist = []                            # 최근 1초 경로 (시각, b, c) - 멈춤 감지용
        self.boost_until = 0.0                    # 이 시각까지 멈춤 탈출 PWM 사용
        self.go = False
        self.zero_until = 0.0                     # [1017] 출발 스위치 꺼진 뒤 이 시각까지 정지 명령 보냄
        self.reload(force=True)
        self.create_timer(0.05, self.tick)        # 1초에 20번 명령 계산 (팀 노드와 같은 주기)

    # --- 슬라이더 값 다시 읽기 (파일이 바뀌었을 때만) ---
    def reload(self, force=False):
        try:
            mt = os.path.getmtime(TUNE)
            if not force and mt == self.mtime:
                return
            p = dict(DEFAULTS); p.update(json.load(open(TUNE)))
        except (OSError, ValueError):
            return                                # 슬라이더 창이 쓰는 도중이면 다음 주기에 다시
        self.mtime = mt; self.p = p
        # Stanley 계산기를 새 게인으로 다시 만든다
        self.ctrl = LaneKeepingController(stanley_gain=p["stanley_gain"], curvature_gain=p["curvature_gain"],
                                          softening_speed_mps=p["softening_speed_mps"],
                                          front_axle_offset_m=p["front_axle_offset_m"], heading_gain=p["heading_gain"])
        self.get_logger().info("tune loaded: %s" % json.dumps(p, ensure_ascii=False))
        if RUN:                                   # 언제 무엇으로 바꿨는지 기록 (나중에 비교용)
            with open(os.path.join(RUN, "tune_changes.log"), "a") as f:
                f.write("%s %s\n" % (time.strftime("%H:%M:%S"), json.dumps(p, ensure_ascii=False)))

    # --- 경로가 올 때마다: 정상 경로면 기억, 아니면 지움 ---
    def on_traj(self, m):
        if m.valid:
            self.traj, self.traj_t = m, time.monotonic()
        else:
            self.traj, self.traj_t = None, None

    # --- 1초에 20번: 명령 계산해서 보내기 ---
    def tick(self):
        self.reload()
        p = self.p
        # (1) 경로가 없거나 너무 오래됐으면 정지 명령 (안전)
        stale = self.traj is None or (time.monotonic() - self.traj_t) > p["lane_timeout_sec"]
        delta = None; shift = 0.0
        if stale:
            self.i_lost += 0.05
            if self.i_lost > 0.5:
                self.i_acc = 0.0                  # [I 항] 경로를 0.5초 넘게 잃으면 초기화
            steer, l, r = 0, 0, 0
        else:
            t = self.traj
            '''
            # (2) 경로를 옆으로 옮기기: 고정 이동(path_offset) + 커브 안쪽으로 당기기(인코스)
            x0 = p["front_axle_offset_m"]
            curv = 2.0 * t.a / (1.0 + (2.0 * t.a * x0 + t.b) ** 2) ** 1.5    # 평가 지점의 곡률 (+ = 왼쪽 커브)
            bias = p["inside_bias_m"]
            if curv < 0 and p["inside_bias_right_m"] != 0:      # 오른쪽 커브(곡률 -)는 따로 정한 값
                bias = p["inside_bias_right_m"]
            shift = p["path_offset_m"] + bias * float(np.clip(curv / CURV_FULL, -1.0, 1.0))
            '''
            # -------------------- [기존 122~128번 줄 7줄 삭제 후 교체] --------------------
            # (2) 내륜차 물리 보상 (우회전 시 뒷바퀴가 코너 안쪽 실선 밟는 것 방지)
            x0 = p["front_axle_offset_m"]
            curv = 2.0 * t.a / (1.0 + (2.0 * t.a * x0 + t.b) ** 2) ** 1.5    # 곡률 (+: 좌회전, -: 우회전)
            WHEELBASE = 0.56       # 전동차 축간거리(앞바퀴~뒷바퀴 거리 약 0.65m)
            off_tracking = 0.5 * (WHEELBASE ** 2) * curv   # 우회전이면 curv<0 이므로 shift는 왼쪽(+)으로
            shift = p["path_offset_m"] - off_tracking
            

            # (3) Stanley 로 조향각(라디안) 계산
            delta, dbg = self.ctrl.compute_delta((t.a, t.b, t.c + shift), p["vehicle_speed_mps"])
            # (3-1) [I 항] 직선에서 한쪽으로 계속 벗어나 있으면 조금씩 더 꺾기. 끝까지 꺾고 있으면(=포화) 쌓지 않음
            ki = float(p["i_gain"]); imax = math.radians(float(p["i_max_deg"]))
            self.i_lost = 0.0
            if ki > 0:
                saturated = abs(self.prev_steer) >= FW_FULL - 1
                if abs(dbg.curvature) < p["i_curv_max"] and not saturated:
                    self.i_acc += dbg.cte_m * 0.05
                self.i_acc = float(np.clip(self.i_acc, -imax / ki, imax / ki))
                delta += ki * self.i_acc
            else:
                self.i_acc = 0.0
            # (4) 조향각 → 조향 명령(-100~+100).  최대 조향각(max_steer_deg)이 100 (끝까지).
            #     max_steering_step 은 예전 단계 단위 상한 (7 = 100, 5 = 71)
            mx = int(round(max(1.0, min(OLD_STEPS, p["max_steering_step"])) * K))
            norm = float(np.clip(delta / max(math.radians(p["max_steer_deg"]), 1e-6), -1.0, 1.0))
            steer = int(round(p["steering_sign"] * norm * FW_FULL + p["steer_trim"] * K))   # sign=-1: 왼쪽 각도 → 음수
            steer = int(np.clip(steer, -mx, mx))
            # (4-1) [선 넘기 방지] 차 앞(x=0, 약 0.84 m 앞)에서 차로 가운데가 차보다 어디 있나 = t.c
            #       t.c < -guard : 가운데가 오른쪽 = 차가 왼쪽(점선 쪽)으로 치우침 → 왼쪽(음수) 조향 금지
            #       t.c > +guard : 가운데가 왼쪽   = 차가 오른쪽(실선 쪽)으로 치우침 → 오른쪽(양수) 조향 금지
            g = p["lane_guard_m"]                                  # 오른쪽(실선) 쪽 기준
            gl = p["lane_guard_left_m"] if p["lane_guard_left_m"] > 0 else g   # 왼쪽(점선) 쪽 기준
            push = int(round(p["lane_guard_push"] * K))              # 넘으면 반대쪽으로 최소 이만큼 (단계 → 1% 단위)
            ''' 휘청
            if gl > 0 and t.c < -gl: steer = max(steer, push)       # 점선 쪽으로 치우침 → 최소 오른쪽 push 단계
            if g > 0 and t.c > g: steer = min(steer, -push)         # 실선 쪽으로 치우침 → 최소 왼쪽 push 단계
            '''
            
            # (4-2) [급발진 방지] 직전 단계에서 steer_rate 단계 넘게 한 번에 바뀌지 않게
            rate = int(round(p["steer_rate"] * K))                  # 단계/0.05초 → 1% 단위 (1 단계 = 14)
            if rate > 0:
                steer = int(np.clip(steer, self.prev_steer - rate, self.prev_steer + rate))
            # (5) 조향 단계에 맞는 뒷바퀴 PWM (많이 꺾을수록 느리게 설정 가능)
            #     예전 단계 사이 값은 앞뒤 단계 PWM 을 이어서 계산 (예: 3.5 단계 = 3단계와 4단계 PWM 의 가운데)
            pos = min(abs(steer) / K, OLD_STEPS)
            i0 = int(pos); i1 = min(i0 + 1, OLD_STEPS)
            pw = p["pwm_by_step"][i0] + (p["pwm_by_step"][i1] - p["pwm_by_step"][i0]) * (pos - i0)
            l = r = int(np.clip(round(pw), 0, 255))
            # (5-0) [멈춤 탈출] 최근 1초 동안 경로(b, c)가 1 cm 도 안 변했으면 차가 서 있는 것 → 0.5 초 세게
            now = time.monotonic()
            self.hist = [h for h in self.hist if now - h[0] <= 1.0] + [(now, t.b, t.c)]
            boost = int(p["stall_boost_pwm"])
            if boost > 0 and now >= self.boost_until and now - self.hist[0][0] >= 0.9:
                if max(max(h[1] for h in self.hist) - min(h[1] for h in self.hist),
                       max(h[2] for h in self.hist) - min(h[2] for h in self.hist)) < 0.01:
                    self.boost_until = now + 0.5
                    self.hist = []
                    self.get_logger().info("stall detected -> boost PWM %d for 0.5 s" % boost)
            if now < self.boost_until:
                l = r = max(l, boost)
            # (5-1) [좌우 모터 보정] 두 모터 힘이 달라 한쪽으로 흐르면 센 쪽을 줄인다
            bal = int(round(p["motor_balance"]))
            if bal > 0: l = max(0, l - bal)
            if bal < 0: r = max(0, r + bal)
        self.prev_steer = steer
        msg = MotionCommand(); msg.steering, msg.left_speed, msg.right_speed = steer, l, r
        # (5-2) [1017 출발 스위치] serial_sender 가 car_prep 때부터 켜져 있으므로 여기서 막는다.
        #       go_until 시각 전에만 주행 명령. 꺼지면 0.5 초 동안 정지 명령, 그 뒤엔 아무것도 안 보냄 (보드는 IDLE 로 대기)
        try:
            go = time.time() < float(open(GO).read())
        except (OSError, ValueError):
            go = False
        self.go = go
        if go:
            self.zero_until = time.monotonic() + 0.5
            self.pub.publish(msg)
        elif time.monotonic() < self.zero_until:
            m0 = MotionCommand(); m0.steering = m0.left_speed = m0.right_speed = 0
            self.pub.publish(m0)

        # (6) 0.2 초마다 슬라이더 창에 상태 알림
        self.n += 1
        if self.n % 4 == 0:
            st = {"time": time.time(), "steer": steer, "pwm": l, "valid": not stale,
                  "c": None if stale else round(self.traj.c, 3), "b": None if stale else round(self.traj.b, 3),
                  "shift": round(shift, 3), "delta_deg": None if delta is None else round(math.degrees(delta), 1),
                  "i_deg": round(math.degrees(float(p["i_gain"]) * self.i_acc), 2),
                  "go": self.go}                     # [1017] car_go.sh 가 이 키로 출발 스위치 있는 제어기인지 확인
            try:
                with open(STATUS + ".tmp", "w") as f:
                    json.dump(st, f)
                os.replace(STATUS + ".tmp", STATUS)
            except OSError:
                pass


rclpy.init()
n = Tunable()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    m = MotionCommand(); m.steering = m.left_speed = m.right_speed = 0    # 끝날 때 정지 명령
    n.pub.publish(m); n.destroy_node(); rclpy.try_shutdown()
