# -*- coding: utf-8 -*-
"""
조정형 Stanley 제어기:  /lane_trajectory (y = a·x² + b·x + c)  →  /topic_control_signal (조향 -100~100, 왼쪽/오른쪽 PWM)

base(HEAD 동작, 플래그 공집합) — tests/golden/base_ctrl.json 으로 연산 순서까지 동일함을 증명한다:
   δ = heading_gain·atan(y'(x0)) + atan(stanley_gain·y(x0) / (vehicle_speed_mps + softening_speed_mps))
       + curvature_gain·κ(x0) + i_gain·∫y(x0)dt                                   (x0 = front_axle_offset_m)
   조향 = -100 · δ / max_steer_deg   (펌웨어: + 오른쪽. 경로 각도는 + 왼쪽이라 부호가 반대)
   - 내륜차 보상: 경로를 0.5·축간거리²·κ 만큼 커브 바깥으로 옮겨서 계산 (뒷바퀴가 안쪽 선을 밟지 않게)
   - I 항: 직선(|κ| < i_curv_max)이고 끝까지 꺾지 않았을 때만 쌓고 ±i_max_deg 에서 자름. 경로를 0.5 초 넘게 잃으면 0
   - 경로가 lane_timeout_sec 넘게 없으면 정지 명령. pwm 은 tune.json pwm 고정
   - κ(x0) 는 내륜차 항(python 거듭제곱)과 δ 의 curvature 항(np.power) 두 식으로 계산한다. 같은 값이지만 비트 동일하지
     않아(40만 표본 중 106개 1 ULP 차이) 하나로 합치면 base 골든이 깨진다 → 두 식 유지.

변형 플래그 (nodes/variant.py, tune/algo 프로필 → 플래그 집합. 상수는 var.p 그룹 한 곳):
   corner_slowdown  속도 스케줄러 SpeedScheduler: |조향 %| 히스테리시스(진입 corner.steer_on_pct, 해제 corner.steer_off_pct)
                    로 목표 pwm 을 직선 tune pwm / 코너 min(corner.pwm, tune pwm) 두 단계로 정하고 1차 저역통과(corner.tau_sec) 로 따라간다.
                    코너 pwm 이 직선 pwm 을 넘지 않으므로 '감속' 만 한다(tune pwm < corner.pwm 이어도 가속 없음).
                    스케줄러 상태는 출발 스위치가 켜진 동안만 유효: go 가 꺼진 주기마다 reset (정차 중 카메라가 큰 옆오차를 봐도
                    출발 첫 명령은 tune pwm 에서 시작). 경로 상실 0.5 s 뒤에도 reset.
                    (이전 v2 의 조향 비례 선형 감속은 20 Hz 로 255↔190 을 왕복해 폐기)
   speed_schedule   Stanley 분모 v_eff = v255·pwm_f/255 (v255 = config/vehicle.yaml pwm_speed_mps[255], pwm_f = 스케줄러의 직전
                    주기 pwm). 꺼지면 base 와 같이 tune vehicle_speed_mps. corner_slowdown 이 필요(variant.REQUIRES).
                    주의: vehicle.yaml 의 pwm→속도 표가 measured: false(시뮬 눈금 255 → 1.05 m/s) 인 동안은 분모가 base 의
                    tune vehicle_speed_mps(0.5) 와 다른 눈금이라 같은 stanley_gain 이 ~1/1.9 로 희석된다. 기동 시 경고 로그 1회.
                    실측(사양 5.1 ③) 뒤 표를 바꾸거나 눈금 결정을 다시 한다(README 실차 프로토콜).
   arc_comp         원호 보정: 평가점이 뒷축 앞 PATH_ORIGIN_X_M + x0 라 커브에서 중심을 잘 따라도 경로가 d²/(2R) 만큼 안쪽에
                    보여야 한다. 0.5·(PATH_ORIGIN_X_M + x0)²·κ_f 를 빼서 계산 (내륜차 항 포함·대체). κ_f = tan(직전 주기 실제
                    조향각)/축간거리 의 저역통과(arc.kappa_tau_sec). 보정량 ±arc.shift_max_m. 경로 점수 num_points ≤ arc.min_points
                    인 프레임(탐색·되돌림 경로)은 shift 0. 부호 보존 게이트는 없다(사각지대 제거). 경로를 0.5 초 넘게 잃으면 κ_f 리셋.
   controller_pp    Pure Pursuit(arc_comp·front_axle_ref 와 배타): x_t = pp.lookahead_x_m, y_t = a x_t² + b x_t + c,
                    D² = (PATH_ORIGIN_X_M + x_t)² + y_t², κ = 2 y_t / D², δ = atan(축간거리·κ) + I 항.
                    I 항은 base 와 같은 누적기(직선 |κ_path(x_t)| < i_curv_max, 끝까지 꺾지 않았을 때, 게인 pp.i_gain, 상한 i_max_deg).
                    heading/curvature/내륜차/arc 미사용. 원호 위 점열이면 κ = 1/R 이 정확히 나온다.
   front_axle_ref   앞쪽 평가 Stanley(arc_comp·controller_pp 와 배타): 평가점을 IPM 창 뒤(앞축 쪽)로 외삽. x_e = 축간거리 − PATH_ORIGIN_X_M
                    + front.ahead_m (IPM 좌표, 앞축 = −1.07). 채택값(코드 기본) ahead_m 0.85 = 앞축 앞 0.85 m = 뒷축 앞 1.41 m (IPM x_e −0.22, base 평가점 1.73 m).
                    바깥 정지 여유가 좁다: x_e 를 −0.26 이하로 뒤로 보내면 정지 급증(10/31 1차선 벤치 −0.22 정지 0/20, −0.26 1/3, −0.30 4/5).
                    e = y(x_e), ψ = 경로 접선각(x_e). 외삽 front.extrap: poly = 2차식 그대로,
                    arc(기본) = anchor 점 front.anchor_x_m 의 위치·접선 θ_a·곡률 κ_a 로 정한 원을 따라 x_e 까지:
                    sin ψ = sin θ_a + κ_a·(x_e − x_a), e = y_a + (x_e − x_a)·tan((θ_a + ψ)/2)  (현의 방향 = 두 접선각의 평균),
                    δ = front.heading_gain·ψ + atan(front.k_scale·stanley_gain·e / (v_eff + softening_speed_mps)) + I 항(base 규칙, 게인 i_gain).
                    고전 정의(ahead_m 0, heading_gain 1)는 원호에서 ψ = atan(L/R_뒷축) = 필요한 조향이라 정상 옆오차 0 이지만, 앞축 값은 외삽뿐이라
                    곡률에 음의 가중이 걸려 코너 진입에서 바깥으로 꺾는다(10/31 1차선: 첫 코너 이탈·정지). 내륜차·arc 보정·curvature 항 미사용
                    (평가점을 당겨 안쪽 평형을 줄이는 것으로 대체). v_eff 는 speed_schedule 이면 스케줄러 pwm 기준.
   cte_raw          모든 분기에서 보정 전 옆오차 a x0² + b x0 + c (status.json 표시용).
   차선 범위         var.flags 는 Variant 가 tune/lane_eff(플래그 차선: 경로 노드가 Lane 실제 상태로 정해 원자적으로 씀)를 mtime 으로 읽어
                    고른 집합(v3: 1차선 = V3_LANE1 = v2f, 2차선 = base). 부재·잘못된 값이면 tune/lane(요청 차선, variant.parse_lane), 시작 때 둘 다 부재 → 2.
                    경로 노드가 쓴 다음 tick(0.05 s) 부터 새 차선 플래그 → 어긋남 최대 1프레임. scope_hold 프로필은 S자 ramp(CHANGING) 전체가
                    1차선 플래그(front_axle_ref), lane_settle 보류 중에는 지금 달리는 차선의 플래그(규칙·시각 판단은 경로 노드 한 곳).
                    상태 리셋은 상태를 쓰는 플래그가 켜지거나 꺼진 tick 에만: corner_slowdown 유무가 바뀌면 스케줄러, arc_comp 유무가 바뀌면 κ_f
                    (꺼져 있던 동안 멈춘 상태로 다시 시작하지 않게. 유지되는 플래그의 상태는 이어감 — 예 v2→v2a 는 스케줄러 그대로).
                    front_axle_ref 는 자체 필터 상태가 없다(매 주기 κ_f·kappa_cmd = None, 입력은 그 주기 경로와 스케줄러 pwm 뿐).
                    I 누적기 i_acc 는 리셋하지 않는다: base·arc_comp·front_axle_ref 가 같은 게인 i_gain·상한 i_max_deg 로 공유 → 전환 순간 I 항 연속.
                    base·두 차선 같은 프로필은 플래그가 바뀌지 않으므로 이 리셋이 일어나지 않는다(골든 불변).

차량 기하(축간거리 0.56, 경로 원점 1.63 = 0.79 + 0.84)는 config/vehicle.yaml → nodes/vehicle.py 에서 읽는다.
tune/tune.json (tune_gui.py 가 저장) 이 바뀌면 다음 주기(0.05 초)에 반영한다.
tune/go_until (car_go.sh 가 쓰는 주행 끝 시각) 전에만 명령을 보낸다. 꺼지면 0.5 초 동안 정지 명령, 그 뒤엔 아무것도 안 보냄.
tune/status.json 에 0.2 초마다 상태를 쓴다 (tune_gui.py 표시, car_go.sh 가 "go" 키로 제어기 확인).
  키: HEAD 10키 time steer pwm valid c b shift delta_deg i_deg go 그대로 + 추가 7키 kappa_cmd cte_raw variant v_target arc_shift v_eff_mps
  (기존 키 삭제 없음, 추가만. car_go.sh 는 go·time, tune_gui 는 표시용으로 읽음)
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
from interfaces_pkg.msg import LaneTrajectory, MotionCommand

import vehicle
from variant import Variant

D = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tune")   # tune.json / status.json / go_until 폴더
STEER_MAX = 100                                  # 펌웨어 조향 명령 범위 ±100 (최대 조향의 1% 단위)
WHEELBASE = vehicle.wheelbase_m()                # 축간거리 m (config/vehicle.yaml)
PATH_ORIGIN_X_M = vehicle.path_origin_x_m()      # IPM 원점 = 뒷축 앞 m (cam_to_rear_axle + ipm_x_near). arc_comp·controller_pp·front_axle_ref 만 사용
DT = 0.05                                        # 명령 주기 s


class SpeedScheduler:
    """코너 감속(corner_slowdown) 속도 스케줄러. |조향|(%) 히스테리시스 → 목표 pwm 두 단계 → 1차 저역통과.

    engaged: 코너 상태. |조향| > steer_on_pct 에서 진입, |조향| < steer_off_pct 에서 해제.
    목표: 코너 min(corner.pwm, pwm_max), 직선 pwm_max — 코너가 직선보다 빨라지지 않는다.
    pwm: 저역통과된 pwm (float). None = 다음 update 에서 pwm_max 로 초기화 (출발 스위치 OFF·경로 상실 0.5 s 뒤 리셋)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.engaged = False
        self.pwm = None

    def update(self, abs_steer, pwm_max, corner):
        """한 주기(DT) 갱신. corner = variant.Corner (steer_on_pct/steer_off_pct 조향 %, pwm, tau_sec). 반환: pwm (float)."""
        if self.pwm is None:
            self.pwm = float(pwm_max)
        if self.engaged:
            if abs_steer < float(corner.steer_off_pct):
                self.engaged = False
        elif abs_steer > float(corner.steer_on_pct):
            self.engaged = True
        target = min(float(corner.pwm), float(pwm_max)) if self.engaged else float(pwm_max)
        tau = max(float(corner.tau_sec), 0.0)
        self.pwm += (target - self.pwm) * DT / (tau + DT)
        return self.pwm


class Tunable(Node):
    def __init__(self, run="", tune_dir=None, now=None):
        super().__init__("tunable_controller")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE, depth=1)
        self.pub = self.create_publisher(MotionCommand, "/topic_control_signal", q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.run = run
        self.tune_dir = tune_dir or D
        self.tune_path, self.status_path, self.go_path = (os.path.join(self.tune_dir, n) for n in ("tune.json", "status.json", "go_until"))
        self.now = now or time.monotonic           # 단조 시계 (경로 나이·정지 명령 유지). 테스트는 주입
        self.p = None; self.mtime = 0.0
        self.traj = None; self.traj_t = None      # 마지막 정상 경로와 받은 시각
        self.prev_steer = 0
        self.i_acc = 0.0                          # 옆오차 적분 (m·s)
        self.i_lost = 0.0                         # 경로 없던 시간 (s)
        self.kappa_f = None                       # arc_comp: 저역통과 κ (None = 다음 경로로 초기화)
        self.kappa_cmd = None                     # status 표시: arc_comp 는 kappa_f, controller_pp 는 PP κ, base·front_axle_ref 는 None
        self.cte_raw = None                       # 보정 전 옆오차 m (모든 분기 공통)
        self.arc_shift = 0.0                      # arc_comp 보정량 m (꺼지면 0)
        self.sched = SpeedScheduler()             # corner_slowdown 속도 스케줄러
        self.zero_until = 0.0                     # 출발 스위치가 꺼진 뒤 이 시각까지 정지 명령
        self.n = 0
        self.var = Variant(self.tune_dir, log=self.get_logger().info)
        if not vehicle.load().get("measured", False):
            self.get_logger().warn("vehicle.yaml measured=false: speed_schedule 분모 눈금(v255 %.2f m/s) 은 미실측 시뮬 값 (사양 5.1 ③)" % vehicle.v255_mps())
        self.reload(force=True)
        self.create_timer(DT, self.tick)

    def reload(self, force=False):
        try:
            mt = os.path.getmtime(self.tune_path)
            if not force and mt == self.mtime:
                return
            p = json.load(open(self.tune_path))
        except (OSError, ValueError):
            return                                # 슬라이더 창이 쓰는 도중이면 다음 주기에
        self.mtime = mt; self.p = p
        self.get_logger().info("tune loaded: %s" % json.dumps(p, ensure_ascii=False))
        if self.run:
            with open(os.path.join(self.run, "tune_changes.log"), "a") as f:
                f.write("%s %s\n" % (time.strftime("%H:%M:%S"), json.dumps(p, ensure_ascii=False)))

    def on_traj(self, m):
        self.traj, self.traj_t = (m, self.now()) if m.valid else (None, None)

    # ---- 속도 ----
    def corner_pwm(self, p, steer):
        """이 주기의 pwm. corner_slowdown 이면 스케줄러(|조향| 히스테리시스 + 저역통과), 아니면 tune pwm 고정(base)."""
        if 'corner_slowdown' in self.var.flags:
            pwm = self.sched.update(abs(steer), float(p["pwm"]), self.var.p.corner)
            return int(np.clip(round(pwm), 0, 255))
        return int(np.clip(round(p["pwm"]), 0, 255))

    def v_eff(self, p):
        """Stanley 분모 속도 m/s. speed_schedule 이면 v255·pwm_f/255 (pwm_f = 스케줄러 직전 주기 값), 아니면 tune vehicle_speed_mps."""
        if 'speed_schedule' in self.var.flags:
            pwm_f = float(p["pwm"]) if self.sched.pwm is None else self.sched.pwm
            return vehicle.v255_mps() * pwm_f / 255.0
        return abs(float(p["vehicle_speed_mps"]))

    # ---- 조향 ----
    def _i_term(self, p, ki, cte, curvature):
        """I 항 (base 규칙): 직선(|κ| < i_curv_max)이고 끝까지 꺾지 않았을 때만 쌓고 ±i_max_deg 에서 자름. 반환: 더할 각 rad."""
        imax = math.radians(float(p["i_max_deg"]))
        self.i_lost = 0.0
        if ki > 0:
            if abs(curvature) < p["i_curv_max"] and abs(self.prev_steer) < STEER_MAX - 1:
                self.i_acc += cte * DT
            self.i_acc = float(np.clip(self.i_acc, -imax / ki, imax / ki))
            return ki * self.i_acc
        self.i_acc = 0.0
        return 0.0

    def _norm(self, p, delta):
        """조향각 rad → 조향 명령 -100~100 (부호 반대, 클립)."""
        norm = float(np.clip(delta / max(math.radians(p["max_steer_deg"]), 1e-6), -1.0, 1.0))
        return int(round(-norm * STEER_MAX))

    def _arc_shift(self, p, x, n_pts):
        """arc_comp: 직전 조향 명령 → κ(·arc.kappa_gain) → 저역통과 κ_f → 평가점 원호 보정량 m. num_points ≤ arc.min_points 면 0.
        제한기 arc.limiter (variant.ARC_LIMITERS, 증상별 분기 없이 규칙 하나):
          none   ±shift_max_m 상한만
          cancel 상쇄만: shift 는 cte_raw 와 반대 부호일 때만, |shift| ≤ |cte_raw| (보정이 옆오차 부호를 뒤집어 반대쪽으로 밀지 못함)
          geo    κ 를 조향 대신 경로에서 역산 κ_geo = 2·cte_raw/d² (원호 위 c = d²/2R) → shift = −cte_raw (heading 항만 남음)
          conf   경로 점수 신뢰도 가중: num_points ≥ conf_points 전량, min_points+1 에서 conf_min_weight, 사이 선형 (상한 뒤 가중)
        self.cte_raw 는 steer() 가 이 호출 전에 둔다."""
        arc = self.var.p.arc
        delta_prev = -self.prev_steer / STEER_MAX * math.radians(float(p["max_steer_deg"]))   # 클립 뒤 실제 조향각 (조향 % → rad)
        kappa_src = float(arc.kappa_gain) * math.tan(delta_prev) / WHEELBASE
        tau = max(float(arc.kappa_tau_sec), 0.0)
        self.kappa_f = kappa_src if self.kappa_f is None else self.kappa_f + (kappa_src - self.kappa_f) * DT / (tau + DT)
        self.kappa_cmd = self.kappa_f
        n_min = int(arc.min_points)
        if n_pts <= n_min:
            shift = 0.0
        else:
            lim = abs(float(arc.shift_max_m))
            cte_raw = float(self.cte_raw if self.cte_raw is not None else 0.0)
            kappa = 2.0 * cte_raw / (PATH_ORIGIN_X_M + x) ** 2 if arc.limiter == 'geo' else self.kappa_f
            shift = float(np.clip(-0.5 * (PATH_ORIGIN_X_M + x) ** 2 * kappa, -lim, lim))
            if arc.limiter == 'cancel':
                shift = 0.0 if shift * cte_raw >= 0.0 else float(np.sign(shift) * min(abs(shift), abs(cte_raw)))
            elif arc.limiter == 'conf':
                w0 = float(arc.conf_min_weight); n_full = int(arc.conf_points)
                w = w0 + (1.0 - w0) * (n_pts - n_min - 1) / max(n_full - n_min - 1, 1)
                shift *= float(np.clip(w, w0, 1.0))
        self.arc_shift = shift
        return shift

    def _steer_pp(self, p, a, b, c):
        """controller_pp: Pure Pursuit. 반환 (조향, δ rad, shift 0.0)."""
        pp = self.var.p.pp
        x_t = float(pp.lookahead_x_m)
        y_t = a * x_t * x_t + b * x_t + c
        X = PATH_ORIGIN_X_M + x_t                                   # 뒷축 → 목표점 전방 거리 (≈1.73 m: 룩어헤드는 IPM 창에 묶여 길다)
        d2 = X * X + y_t * y_t                                      # 뒷축 → 목표점 거리²
        kappa_pp = 2.0 * y_t / d2                                   # 순수 PP: 원호 위 목표점이면 1/R 정확
        dy = 2.0 * a * x_t + b
        # 접선 잔차 보정 (pp.heading_gain): 원호 위라면 목표점 접선각 θ_t = 2·시선각 α (원주각) 이라 잔차 0 → 원호 정확성 유지.
        # 직선에서 변위 e·헤딩 ψ 가 있으면 잔차 ≈ ψ + 2e/X 라 −잔차/D 가 PP 의 e 강성을 (1+gain)배로 올린다
        # (PP 단독은 2/D² ≈ 0.67 /m² 로 Stanley 의 1/4, 경로 |c| 상한 0.7 m 에 묶여 κ ≤ 0.42 → R 2.2 코너에서 바깥으로 밀림).
        # 코너 진입(경로가 앞에서 굽기 시작)에서는 잔차가 변위와 반대 뜻이라 PP 와 부호가 다르면 0 (|κ| 를 키우는 쪽만 허용).
        theta_t = math.atan(dy); alpha = math.atan2(y_t, X)
        kappa_h = -float(pp.heading_gain) * (theta_t - 2.0 * alpha) / math.sqrt(d2)
        if kappa_h * kappa_pp < 0.0:
            kappa_h = 0.0
        kappa = float(pp.gain) * kappa_pp + kappa_h
        delta = math.atan(WHEELBASE * kappa)
        curv_path = 2.0 * a / np.power(1.0 + dy * dy, 1.5)          # I 항 직선 판정용 경로 κ(x_t)
        delta += self._i_term(p, float(pp.i_gain), y_t, curv_path)
        self.kappa_f = None; self.kappa_cmd = kappa; self.arc_shift = 0.0
        return self._norm(p, delta), delta, 0.0

    def _front_ref(self, a, b, c, x_e):
        """front_axle_ref 평가점 x_e 의 (e, ψ, κ). poly: 2차식 값·기울기. arc: anchor 원을 따라 되돌아감 (원이 x_e 에 닿지 않으면 ψ = ±90°)."""
        fr = self.var.p.front
        if fr.extrap == 'poly':
            dy = 2.0 * a * x_e + b
            return a * x_e * x_e + b * x_e + c, math.atan(dy), 2.0 * a / (1.0 + dy * dy) ** 1.5
        x_a = float(fr.anchor_x_m)
        d = 2.0 * a * x_a + b
        th_a = math.atan(d)
        kappa = 2.0 * a / (1.0 + d * d) ** 1.5                      # anchor 곡률 (원 모델의 일정 κ)
        psi = math.asin(float(np.clip(math.sin(th_a) + kappa * (x_e - x_a), -1.0, 1.0)))
        e = a * x_a * x_a + b * x_a + c + (x_e - x_a) * math.tan(0.5 * (th_a + psi))   # κ = 0 이면 직선 외삽과 같음
        return e, psi, kappa

    def _steer_front(self, p, a, b, c):
        """front_axle_ref: 앞쪽 평가 Stanley (front.ahead_m 0·heading_gain 1 이면 고전 앞축 Stanley). 반환 (조향, δ rad, shift 0.0)."""
        fr = self.var.p.front
        x_e = WHEELBASE - PATH_ORIGIN_X_M + float(fr.ahead_m)         # 평가점 (IPM 좌표, 앞축 −1.07 ~ IPM 원점 0: IPM 창 뒤로 외삽)
        e, psi, curv_path = self._front_ref(a, b, c, x_e)           # 평가점 옆오차(경로가 왼쪽 +)·접선각·경로 κ
        k = float(fr.k_scale) * float(p["stanley_gain"])
        delta = float(fr.heading_gain) * psi + math.atan(k * e / (self.v_eff(p) + float(p["softening_speed_mps"])))
        delta += self._i_term(p, float(p["i_gain"]), e, curv_path)  # I 항 직선 판정은 경로 κ
        self.kappa_f = None; self.kappa_cmd = None; self.arc_shift = 0.0
        return self._norm(p, delta), delta, 0.0

    def steer(self, p, t):
        """정상 경로 t → (조향 -100~100, 조향각 rad, 옮긴 거리 m). base 는 HEAD 와 연산 순서까지 동일."""
        flags = self.var.flags
        x = float(p["front_axle_offset_m"])
        a, b = float(t.a), float(t.b)
        self.cte_raw = a * x * x + b * x + float(t.c)               # 보정 전 옆오차 (status 표시용, 명령 계산에 안 씀)
        if 'controller_pp' in flags:
            return self._steer_pp(p, a, b, float(t.c))
        if 'front_axle_ref' in flags:
            return self._steer_front(p, a, b, float(t.c))
        kappa0 = 2.0 * a / (1.0 + (2.0 * a * x + b) ** 2) ** 1.5
        if 'arc_comp' in flags:
            shift = self._arc_shift(p, x, int(t.num_points))
            cte = self.cte_raw + shift
        else:                                                       # base: 내륜차 보상 식 그대로
            self.kappa_f = None; self.kappa_cmd = None; self.arc_shift = 0.0
            shift = -0.5 * WHEELBASE ** 2 * kappa0
            c = float(t.c + shift)
            cte = a * x * x + b * x + c                               # x0 에서 경로가 차보다 왼쪽에 있는 거리
        dy = 2.0 * a * x + b
        curvature = 2.0 * a / np.power(1.0 + dy * dy, 1.5)
        delta = float(float(p["heading_gain"]) * np.arctan(dy)
                      + np.arctan(float(p["stanley_gain"]) * cte / (self.v_eff(p) + float(p["softening_speed_mps"])))
                      + float(p["curvature_gain"]) * curvature)
        delta += self._i_term(p, float(p["i_gain"]), cte, curvature)
        return self._norm(p, delta), delta, shift

    def tick(self):
        flags0 = self.var.flags
        self.reload(); self.var.load(now=self.now())
        flags = self.var.flags                    # 상태를 쓰는 플래그가 켜지거나 꺼질 때만 그 상태 리셋 (프로필·플래그 차선 변경)
        if ('corner_slowdown' in flags) != ('corner_slowdown' in flags0):
            self.sched.reset()
        if ('arc_comp' in flags) != ('arc_comp' in flags0):
            self.kappa_f = None
        p = self.p
        stale = self.traj is None or self.now() - self.traj_t > p["lane_timeout_sec"]
        if stale:
            self.i_lost += DT
            if self.i_lost > 0.5:
                self.i_acc = 0.0; self.kappa_f = None
                if 'corner_slowdown' in self.var.flags:
                    self.sched.reset()
            steer, pwm, delta, shift = 0, 0, None, 0.0
        else:
            steer, delta, shift = self.steer(p, self.traj)
            pwm = self.corner_pwm(p, steer)
        self.prev_steer = steer
        try:
            go = time.time() < float(open(self.go_path).read())
        except (OSError, ValueError):
            go = False
        if go:
            self.zero_until = self.now() + 0.5
            m = MotionCommand(); m.steering, m.left_speed, m.right_speed = steer, pwm, pwm
            self.pub.publish(m)
        elif self.now() < self.zero_until:
            self.pub.publish(MotionCommand())     # 기본값 0, 0, 0 = 정지
        if not go and 'corner_slowdown' in self.var.flags:
            self.sched.reset()                    # 스케줄러 상태는 주행 중에만 유효 (정차 중 누적된 코너 상태로 출발하지 않음)
        self.n += 1
        if self.n % 4 == 0:
            self.write_status(p, stale, steer, pwm, delta, shift, go)

    def write_status(self, p, stale, steer, pwm, delta, shift, go):
        st = {"time": time.time(), "steer": steer, "pwm": pwm, "valid": not stale,
              "c": None if stale else round(self.traj.c, 3), "b": None if stale else round(self.traj.b, 3),
              "shift": round(shift, 3), "kappa_cmd": None if (stale or self.kappa_cmd is None) else round(self.kappa_cmd, 3),
              "delta_deg": None if delta is None else round(math.degrees(delta), 1),
              "cte_raw": None if (stale or self.cte_raw is None) else round(self.cte_raw, 3),
              "i_deg": round(math.degrees(float(p["i_gain"]) * self.i_acc), 2), "go": go,
              "variant": self.var.name,
              "v_target": None if self.sched.pwm is None else round(self.sched.pwm, 1),     # 스케줄러 pwm (저역통과 뒤)
              "arc_shift": 0.0 if stale else round(self.arc_shift, 3),
              "v_eff_mps": round(self.v_eff(p), 3)}
        try:
            with open(self.status_path + ".tmp", "w") as f:
                json.dump(st, f)
            os.replace(self.status_path + ".tmp", self.status_path)
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
