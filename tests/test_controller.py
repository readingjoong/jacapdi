# -*- coding: utf-8 -*-
"""제어기(nodes/tunable_controller.py) 단위 테스트 — 사양 U8 + base 보존 보조 검사.

ROS 없이 돈다: rclpy·interfaces_pkg 를 스텁으로 끼운 뒤 모듈을 파일 경로로 임포트한다 (실제 rclpy 가 있어도 DDS 를 열지 않음).

항목: arc_comp 직선 shift 0 · num_points 게이트 · 부호 게이트 없음 · kappa_f None 리셋 · PP 원호 κ 정확(<1e-6) ·
      속도 스케줄러 변화율 ≤ 65/0.3 s · 히스테리시스(진입 60/해제 45) · 코너 pwm ≤ 직선 pwm(tune pwm 150) · 출발 스위치 OFF 중 스케줄러 리셋 ·
      base 에서 변형 함수 미호출 · status.json 키 · base 조향 == HEAD(a41706e, git 있을 때) · v2 조향 == 리팩터 전 작업 트리(동결 사본 있을 때).
실행: cd ~/jacapdi-docker && python3 -m pytest -q tests/test_controller.py
"""
import importlib.util
import json
import math
import os
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
NODES = REPO / "nodes"
CTRL = NODES / "tunable_controller.py"
SNAPSHOT_CTRL = Path.home() / "claude/자캡디/sim/logs/matrix/repo_baseline_snapshot/nodes/tunable_controller.py"
HEAD_SHA = "a41706e"
HEAD_TUNE = {"stanley_gain": 1.0, "heading_gain": 0.2, "vehicle_speed_mps": 0.5, "curvature_gain": 0.2,
             "softening_speed_mps": 0.1, "front_axle_offset_m": 0.1, "max_steer_deg": 25.0, "i_gain": 0.04,
             "i_max_deg": 1.0, "i_curv_max": 0.3, "lane_timeout_sec": 0.6, "pwm": 255, "lane_change_sec": 2.0}
DT = 0.05


# ---------------- ROS 스텁 ----------------
class _Log:
    def __init__(self): self.lines = []
    def info(self, s): self.lines.append(s)
    warn = error = info


class _Pub:
    def __init__(self): self.sent = []
    def publish(self, m): self.sent.append(m)


class _Node:
    def __init__(self, name):
        self._log = _Log(); self.timers = []
    def create_publisher(self, *a, **k): return _Pub()
    def create_subscription(self, *a, **k): return None
    def create_timer(self, dt, cb): self.timers.append((dt, cb)); return None
    def get_logger(self): return self._log
    def destroy_node(self): pass


class _Traj:
    def __init__(self, a=0.0, b=0.0, c=0.0, num_points=40, valid=True, x_min_m=0.0, x_max_m=0.8, fit_error_m=0.0):
        self.a, self.b, self.c = float(a), float(b), float(c)
        self.num_points = int(num_points); self.valid = valid
        self.x_min_m, self.x_max_m, self.fit_error_m = x_min_m, x_max_m, fit_error_m


class _Motion:
    def __init__(self): self.steering = 0; self.left_speed = 0; self.right_speed = 0


def _stub_modules():
    """sys.modules 에 rclpy / interfaces_pkg 스텁을 넣고, 바꾸기 전 항목을 돌려준다."""
    saved = {k: sys.modules.get(k) for k in ("rclpy", "rclpy.node", "rclpy.qos", "interfaces_pkg", "interfaces_pkg.msg")}
    rclpy = types.ModuleType("rclpy"); rclpy.init = lambda *a, **k: None; rclpy.ok = lambda: True
    rclpy.spin = lambda n: None; rclpy.try_shutdown = lambda: None
    node = types.ModuleType("rclpy.node"); node.Node = _Node
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **k: k
    for name in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, name, types.SimpleNamespace(RELIABLE=1, KEEP_LAST=1, VOLATILE=1))
    rclpy.node = node; rclpy.qos = qos
    ipkg = types.ModuleType("interfaces_pkg"); msg = types.ModuleType("interfaces_pkg.msg")
    msg.LaneTrajectory = _Traj; msg.MotionCommand = _Motion; ipkg.msg = msg
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos, "interfaces_pkg": ipkg, "interfaces_pkg.msg": msg})
    return saved


def _restore_modules(saved):
    for k, v in saved.items():
        if v is None: sys.modules.pop(k, None)
        else: sys.modules[k] = v


def _import(path, name):
    """노드 모듈을 ROS 스텁 아래에서 파일 경로로 임포트."""
    saved = _stub_modules()
    if str(NODES) not in sys.path: sys.path.insert(0, str(NODES))
    try:
        spec = importlib.util.spec_from_file_location(name, str(path))
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    finally:
        _restore_modules(saved)
    return mod


@pytest.fixture(scope="module")
def tc():
    return _import(CTRL, "tc_w3")


class Clock:
    def __init__(self, t=1000.0): self.t = t
    def __call__(self): return self.t
    def step(self, dt=DT): self.t += dt


def make(tc, tmp_path, flags=(), name=None, tune=None, clock=None):
    """임시 tune 폴더에 tune.json(+variants.json 프로필 't') 을 쓰고 제어기를 만든다. flags 로 플래그 집합 직접 지정."""
    tdir = tmp_path / ("tune_%s" % (name or "_".join(sorted(flags)) or "base"))
    tdir.mkdir(parents=True, exist_ok=True)
    (tdir / "tune.json").write_text(json.dumps(tune or HEAD_TUNE))
    if flags and name is None:
        (tdir / "variants.json").write_text(json.dumps({"profiles": {"t": sorted(flags)}}))
        name = "t"
    if name is not None:
        (tdir / "algo").write_text(name + "\n")              # tick 의 var.load() 가 같은 이름을 다시 읽도록 파일로
    clock = clock or Clock()
    n = tc.Tunable(run="", tune_dir=str(tdir), now=clock)
    assert n.var.name == (name or "base")
    n.clock = clock
    return n


def feed(n, traj):
    """경로 수신 + 한 주기."""
    n.on_traj(traj); n.tick(); n.clock.step()


# ---------------- 테스트 ----------------
def test_base_flags_empty_and_no_variant_calls(tc, tmp_path, monkeypatch):
    n = make(tc, tmp_path)
    assert n.var.name == "base" and n.var.flags == frozenset()
    def boom(*a, **k): raise AssertionError("base 에서 변형 함수 호출")
    monkeypatch.setattr(tc.SpeedScheduler, "update", boom)
    monkeypatch.setattr(n, "_arc_shift", boom); monkeypatch.setattr(n, "_steer_pp", boom)
    for _ in range(30):
        feed(n, _Traj(0.05, 0.1, 0.2))
    assert n.kappa_f is None and n.kappa_cmd is None and n.arc_shift == 0.0 and n.sched.pwm is None
    n.i_acc = 0.0; n.prev_steer = 0
    s, d, sh = n.steer(n.p, _Traj(0.0, 0.0, 0.0, 40))
    assert s == 0 and sh == 0.0 and n.corner_pwm(n.p, 90) == 255


def test_constants_from_vehicle(tc):
    import vehicle
    assert tc.WHEELBASE == 0.56 and tc.PATH_ORIGIN_X_M == 1.63
    assert tc.PATH_ORIGIN_X_M == vehicle.cam_to_rear_axle_m() + vehicle.ipm_x_near_m()
    assert vehicle.max_steer_deg() == 25.0 and vehicle.v255_mps() > 0
    assert vehicle.speed_mps(0) == 0.0 and abs(vehicle.speed_mps(127.5) - vehicle.v255_mps() / 2) < 1e-12
    assert not hasattr(tc, "KAPPA_TAU_SEC") and not hasattr(tc, "ALGO_LEVELS")


def test_arc_straight_shift_zero(tc, tmp_path):
    n = make(tc, tmp_path, flags={"arc_comp"})
    b = make(tc, tmp_path)
    for _ in range(10):
        t = _Traj(0.0, 0.0, 0.0, 40)                            # 직선 중앙 → 조향 0 → κ_f 0
        feed(n, t); feed(b, t)
    assert n.arc_shift == 0.0 and n.kappa_f == 0.0 and n.kappa_cmd == 0.0
    assert n.prev_steer == b.prev_steer == 0                  # 직선 + 조향 0 → base 와 같은 명령
    n.prev_steer = 0
    s, d, sh = n.steer(n.p, _Traj(0.0, 0.0, 0.2, 40))            # 옆오차만 있어도 직전 조향 0 이면 보정 0 (cte = cte_raw)
    assert sh == 0.0 and n.cte_raw == 0.2


def test_arc_num_points_gate_and_no_sign_gate(tc, tmp_path):
    n = make(tc, tmp_path, flags={"arc_comp"})
    n.prev_steer = -80                                          # 직전에 왼쪽으로 크게 꺾음 → κ_f > 0 → shift < 0
    n.steer(n.p, _Traj(0.0, 0.0, 0.0, 6))
    assert n.arc_shift == 0.0 and n.kappa_f is not None and n.kappa_f > 0
    n.prev_steer = -80
    n.steer(n.p, _Traj(0.0, 0.0, -0.05, 7))                      # cte_raw(-) 와 shift(-) 같은 부호: 게이트 없이 그대로 적용
    assert n.arc_shift < 0 and abs(n.arc_shift) <= n.var.p.arc.shift_max_m
    assert n.cte_raw == -0.05
    # 평가점 보정이 cte 에 들어갔는지: base 식과 조향이 달라야 함
    n2 = make(tc, tmp_path, flags={"arc_comp"}); n2.prev_steer = -80
    s2, _, sh2 = n2.steer(n2.p, _Traj(0.0, 0.0, -0.05, 7))
    b = make(tc, tmp_path); b.prev_steer = -80
    sb, _, _ = b.steer(b.p, _Traj(0.0, 0.0, -0.05, 7))
    assert sh2 == n2.arc_shift and s2 != sb


def test_arc_shift_clipped(tc, tmp_path):
    n = make(tc, tmp_path, flags={"arc_comp"})
    n.prev_steer = -100
    for _ in range(200):
        n.steer(n.p, _Traj(0.0, 0.0, 0.0, 40)); n.prev_steer = -100
    assert n.arc_shift == -n.var.p.arc.shift_max_m


def test_kappa_f_reset_after_lost(tc, tmp_path):
    n = make(tc, tmp_path, flags={"arc_comp"})
    n.var.p.arc.kappa_gain = 1.0   # κ_f == tan δ / L 비교를 위해 게인 1
    t = _Traj(0.0, 0.0, 0.3, 40)
    for _ in range(20):
        feed(n, t)
    assert n.kappa_f is not None and n.kappa_f != 0.0
    kf = n.kappa_f
    n.traj = None                                               # 경로 상실
    for _ in range(int(0.5 / DT) + 2):
        n.tick(); n.clock.step()
    assert n.kappa_f is None and n.i_acc == 0.0
    n.prev_steer = 40
    feed(n, t)
    delta_prev = -40 / tc.STEER_MAX * math.radians(25.0)
    assert n.kappa_f == math.tan(delta_prev) / tc.WHEELBASE       # None 뒤 첫 프레임은 저역통과 기억 없이 초기화
    assert n.kappa_f != kf


def _circle_coefs(R, x_t, origin, sign=1.0):
    """반지름 R 원호(뒷축에서 x 축에 접함, sign>0 왼쪽) 위 3점으로 경로 좌표 2차식 (a,b,c)."""
    xs = np.array([x_t - 0.1, x_t, x_t + 0.1])
    xg = origin + xs
    ys = sign * (R - np.sqrt(R * R - xg * xg))
    a, b, c = np.polyfit(xs, ys, 2)
    return float(a), float(b), float(c)


@pytest.mark.parametrize("R,sign", [(3.0, 1.0), (2.0, -1.0), (5.0, 1.0), (10.0, -1.0)])
def test_pp_arc_kappa_exact(tc, tmp_path, R, sign):
    n = make(tc, tmp_path, flags={"controller_pp"})
    x_t = n.var.p.pp.lookahead_x_m
    a, b, c = _circle_coefs(R, x_t, tc.PATH_ORIGIN_X_M, sign)
    s, delta, sh = n.steer(n.p, _Traj(a, b, c, 40))
    assert abs(n.kappa_cmd - sign / R) < 2e-3                   # 접선 잔차 θ_t−2α 는 원호에서 0 (3점 2차 적합 오차만 남음)
    assert sh == 0.0 and n.arc_shift == 0.0 and n.kappa_f is None
    assert (s < 0) == (sign > 0)                                # 왼쪽 원호 → 왼쪽 조향(명령 음수)
    assert abs(delta - math.atan(tc.WHEELBASE * sign / R)) <= math.radians(1.0) + 2e-3   # Ackermann δ = atan(L/R) (I 항 ±1° 안)


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_pp_straight_offset_stiffer_than_pure(tc, tmp_path, sign):
    """직선 변위 e: 잔차 보정이 PP 와 같은 부호로 |κ| 를 (1+heading_gain)배 근처로 키우고, 방향은 경로 쪽."""
    n = make(tc, tmp_path, flags={"controller_pp"})
    pp = n.var.p.pp; e = 0.3 * sign; X = tc.PATH_ORIGIN_X_M + pp.lookahead_x_m
    s, delta, sh = n.steer(n.p, _Traj(0.0, 0.0, e, 40))
    k_pure = 2.0 * e / (X * X + e * e)
    assert (n.kappa_cmd > 0) == (e > 0) and (s < 0) == (e > 0)
    assert abs(n.kappa_cmd) > abs(k_pure) * (1.0 + 0.8 * pp.heading_gain)
    assert abs(delta - math.atan(tc.WHEELBASE * n.kappa_cmd)) <= math.radians(1.0) + 1e-9


def test_pp_heading_residual_never_flips_sign(tc, tmp_path):
    """코너 진입(경로가 앞에서만 굽음): 잔차가 PP 와 반대 부호면 0 → κ 부호는 PP 그대로."""
    n = make(tc, tmp_path, flags={"controller_pp"})
    x_t = n.var.p.pp.lookahead_x_m
    a, b = 0.6, 0.0                                             # y = 0.6 x² : x_t 에서 y 작고 접선은 크게 왼쪽
    s, delta, sh = n.steer(n.p, _Traj(a, b, 0.0, 40))
    X = tc.PATH_ORIGIN_X_M + x_t; y_t = a * x_t * x_t
    assert abs(n.kappa_cmd - 2.0 * y_t / (X * X + y_t * y_t)) < 1e-9 and s <= 0


def test_pp_straight_centered_zero(tc, tmp_path):
    n = make(tc, tmp_path, flags={"controller_pp"})
    s, delta, sh = n.steer(n.p, _Traj(0.0, 0.0, 0.0, 40))
    assert s == 0 and delta == 0.0 and n.kappa_cmd == 0.0 and n.cte_raw == 0.0


def test_pp_i_term_straight_only(tc, tmp_path):
    n = make(tc, tmp_path, flags={"controller_pp"})
    for _ in range(50):
        n.steer(n.p, _Traj(0.0, 0.0, 0.05, 40)); n.prev_steer = 0
    assert n.i_acc > 0                                          # 직선 옆오차 → 누적
    acc = n.i_acc
    a, b, c = _circle_coefs(2.0, n.var.p.pp.lookahead_x_m, tc.PATH_ORIGIN_X_M)   # κ_path 0.5 ≥ i_curv_max 0.3
    n.steer(n.p, _Traj(a, b, c, 40))
    assert n.i_acc == acc                                       # 곡선(|κ_path| ≥ i_curv_max)에선 안 쌓음


def test_scheduler_hysteresis(tc, tmp_path):
    n = make(tc, tmp_path, flags={"corner_slowdown"})
    c = n.var.p.corner
    assert (c.steer_on_pct, c.steer_off_pct) == (60.0, 45.0)
    s = n.sched
    s.update(50, 255, c); assert not s.engaged                  # 60 아래: 진입 안 함
    s.update(61, 255, c); assert s.engaged                      # 60 넘음: 진입
    s.update(50, 255, c); assert s.engaged                      # 45~60: 유지
    s.update(46, 255, c); assert s.engaged
    s.update(44, 255, c); assert not s.engaged                  # 45 아래: 해제
    s.update(59, 255, c); assert not s.engaged                  # 다시 60 넘기 전엔 안 들어감
    for _ in range(200): s.update(0, 255, c)
    assert abs(s.pwm - 255) < 1e-6
    for _ in range(200): s.update(100, 255, c)
    assert abs(s.pwm - c.pwm) < 1e-6


def test_scheduler_rate_limit_and_flip(tc, tmp_path):
    n = make(tc, tmp_path, flags={"corner_slowdown"})
    c = n.var.p.corner
    # ±50 1 Hz 뒤집힘: |조향| 50 < 60 → 감속 진입 없음 → pwm 255 고정
    pwms = []
    for k in range(200):
        st = 50 if (k // 10) % 2 == 0 else -50
        pwms.append(n.corner_pwm(n.p, st))
    assert set(pwms) == {255}
    # ±70 1 Hz 뒤집힘: 진입 후 유지, 변화율 ≤ 65 / 0.3 s (6주기), 왕복 없음
    n2 = make(tc, tmp_path, flags={"corner_slowdown"}, name=None)
    seq = []
    for k in range(200):
        st = 70 if (k // 10) % 2 == 0 else -70
        seq.append(n2.corner_pwm(n2.p, st))
    w = int(round(0.3 / DT))
    assert max(abs(seq[i + w] - seq[i]) for i in range(len(seq) - w)) <= 65
    assert all(seq[i + 1] <= seq[i] for i in range(len(seq) - 1))    # 단조 감소 (255 → 190)
    assert seq[-1] == c.pwm
    # 코너 → 직선 복귀도 같은 변화율
    seq2 = [n2.corner_pwm(n2.p, 0) for _ in range(60)]
    assert max(abs(seq2[i + w] - seq2[i]) for i in range(len(seq2) - w)) <= 65 and seq2[-1] == 255


def test_corner_pwm_never_above_tune_pwm(tc, tmp_path):
    """tune pwm 이 corner.pwm(190) 보다 낮으면(실차 보수적 PWM 150) 코너에서 pwm 이 올라가지 않는다 — 감속만."""
    for pmax in (150, 190, 255):
        tune = dict(HEAD_TUNE, pwm=pmax)
        n = make(tc, tmp_path / ("p%d" % pmax), flags={"corner_slowdown", "speed_schedule"}, tune=tune)
        assert n.var.flags == frozenset({"corner_slowdown", "speed_schedule"}) and n.p["pwm"] == pmax
        Path(n.go_path).write_text(repr(__import__("time").time() + 60))
        pwms = []
        for _ in range(100):
            feed(n, _Traj(0.0, 0.0, 0.6, 40))                   # 큰 옆오차 → |조향| > 60 → 코너
            pwms.append(n.pub.sent[-1].left_speed)
        assert max(pwms) <= pmax, (pmax, max(pwms))
        assert pwms[-1] == min(pmax, n.var.p.corner.pwm), (pmax, pwms[-1])
        assert n.v_eff(n.p) <= tc.vehicle.v255_mps() * pmax / 255.0 + 1e-9


def test_scheduler_reset_while_go_off(tc, tmp_path):
    """출발 스위치 OFF 동안 정차 중 카메라가 큰 옆오차를 봐도 코너 상태가 남지 않는다: go ON 첫 명령은 tune pwm 에서 시작."""
    n = make(tc, tmp_path, flags={"corner_slowdown", "speed_schedule"})
    for _ in range(60):
        feed(n, _Traj(0.0, 0.0, 0.6, 40))                       # go OFF 3 s: 조향 포화
    assert n.pub.sent == [] and n.sched.pwm is None and not n.sched.engaged
    st = json.load(open(n.status_path)); assert st["v_target"] is None
    Path(n.go_path).write_text(repr(__import__("time").time() + 60))
    feed(n, _Traj(0.0, 0.0, 0.6, 40))
    first = n.pub.sent[0].left_speed
    assert first >= 240, first                                  # 255 에서 저역통과로 내려가기 시작 (190 출발 아님)
    b = make(tc, tmp_path)                                      # base 는 상태가 없어 영향 없음
    for _ in range(5): feed(b, _Traj(0.0, 0.0, 0.6, 40))
    assert b.sched.pwm is None


def test_speed_schedule_v_eff(tc, tmp_path):
    import vehicle
    n = make(tc, tmp_path, flags={"corner_slowdown", "speed_schedule"})
    assert abs(n.v_eff(n.p) - vehicle.v255_mps()) < 1e-12        # 스케줄러 초기화 전: tune pwm 255 기준
    for _ in range(200): n.corner_pwm(n.p, 100)
    assert abs(n.v_eff(n.p) - vehicle.v255_mps() * n.var.p.corner.pwm / 255.0) < 1e-9
    b = make(tc, tmp_path)
    assert b.v_eff(b.p) == 0.5                                  # base: tune vehicle_speed_mps


def test_status_keys(tc, tmp_path):
    n = make(tc, tmp_path, flags={"corner_slowdown", "speed_schedule"})
    Path(n.go_path).write_text(repr(__import__("time").time() + 60))
    for _ in range(4):
        feed(n, _Traj(0.0, 0.0, 0.1, 40))
    st = json.load(open(n.status_path))
    head = {"time", "steer", "pwm", "valid", "c", "b", "shift", "delta_deg", "i_deg", "go"}          # HEAD a41706e 10키
    added = {"kappa_cmd", "cte_raw", "variant", "v_target", "arc_shift", "v_eff_mps"}
    assert set(st) == head | added, set(st) ^ (head | added)
    assert st["variant"] == "t" and st["valid"] is True and st["go"] is True
    assert st["cte_raw"] == 0.1 and st["v_target"] is not None and st["arc_shift"] == 0.0 and st["v_eff_mps"] > 0


def test_go_publishes_and_stop_after(tc, tmp_path):
    n = make(tc, tmp_path)
    Path(n.go_path).write_text(repr(__import__("time").time() + 60))
    feed(n, _Traj(0.0, 0.0, 0.0, 40))
    assert len(n.pub.sent) == 1 and n.pub.sent[0].left_speed == 255
    os.remove(n.go_path)
    feed(n, _Traj(0.0, 0.0, 0.0, 40))
    assert len(n.pub.sent) == 2 and n.pub.sent[1].left_speed == 0   # 꺼진 뒤 0.5 s 정지 명령
    n.clock.step(0.6); n.tick()
    assert len(n.pub.sent) == 2


# ---------------- base / v2 보존 (W0 골든의 보조) ----------------
GRID_ABC = [(a, b, c) for a in (-0.3, -0.05, 0.0, 0.05, 0.3) for b in (-0.6, -0.1, 0.0, 0.1, 0.6) for c in (-0.4, -0.1, 0.0, 0.1, 0.4)]
GRID_N = (0, 5, 6, 7, 40)
GRID_PREV = (-100, -60, 0, 60, 100)


def _grid(n, name_setter=None):
    out = []
    for (a, b, c) in GRID_ABC:
        for npts in GRID_N:
            for prev in GRID_PREV:
                n.prev_steer = prev; n.i_acc = 0.0123; n.kappa_f = None
                if hasattr(n, "sched"): n.sched.reset()          # 격자점마다 새 상태 (이전 선형 감속은 상태 없음)
                s, d, sh = n.steer(n.p, _Traj(a, b, c, npts))
                pwm = n.corner_pwm(n.p, s) if hasattr(n, "corner_pwm") else int(np.clip(round(n.p["pwm"]), 0, 255))  # HEAD 는 tick 안 인라인
                out.append((s, d, sh, n.i_acc, pwm))
    return out


def _head_source():
    try:
        return subprocess.run(["git", "-C", str(REPO), "show", "%s:nodes/tunable_controller.py" % HEAD_SHA],
                              capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None


def _legacy_ctrl(tmp_path, src, modname, algo):
    """HEAD/작업트리 사본 제어기를 스텁 아래에서 만들고 (tune 은 임시 폴더) 변형 이름을 지정."""
    p = tmp_path / (modname + ".py"); p.write_text(src)
    mod = _import(p, modname)
    class _N(mod.Tunable):
        def __init__(self):
            _Node.__init__(self, "x"); self.p = dict(HEAD_TUNE); self.prev_steer = 0; self.i_acc = 0.0; self.i_lost = 0.0
            self.kappa_f = None; self.cte_raw = None; self.algo = algo
    return _N()


def test_base_equals_head(tc, tmp_path):
    src = _head_source()
    if src is None:
        pytest.skip("git/HEAD 없음 — W0 골든(tests/golden/base_ctrl.json)이 대신 증명")
    assert "ALGO_LEVELS" not in src                              # 진짜 HEAD(변형 코드 이전)인지
    h = _legacy_ctrl(tmp_path, src, "tc_head", "base")
    n = make(tc, tmp_path)
    assert _grid(h) == _grid(n)                                 # (steer, delta, shift, i_acc, pwm) 전부 ==


def test_v2_steer_equals_snapshot(tc, tmp_path):
    if not SNAPSHOT_CTRL.is_file():
        pytest.skip("동결 사본 없음 — W0 v2 골든이 대신 증명")
    h = _legacy_ctrl(tmp_path, SNAPSHOT_CTRL.read_text(), "tc_snap", "v2")
    n = make(tc, tmp_path, name="v2")
    assert "speed_schedule" in n.var.flags
    n.var = type(n.var).from_name("v2", n.tune_dir)
    n.var.flags = frozenset(n.var.flags - {"speed_schedule"})        # 조향 비교: 분모는 base 와 같게(corner_slowdown 만)
    gh, gn = _grid(h), _grid(n)
    assert [r[:4] for r in gh] == [r[:4] for r in gn]            # steer, delta, shift, i_acc 동일
    # pwm: 이전 선형 감속 → 스케줄러(히스테리시스+저역통과) 로 바뀜(사양 1.2). |조향| ≤ 60 인 격자점은 둘 다 255
    for r_h, r_n in zip(gh, gn):
        if abs(r_h[0]) <= 60: assert r_h[4] == r_n[4] == 255


# ---------------- arc.limiter (원호 보정 제한기) ----------------
def _arc_n(tc, tmp_path, **arc):
    n = make(tc, tmp_path, flags={"arc_comp"})
    arc = {"kappa_gain": 1.0, "shift_max_m": 0.4, **arc}   # 제한기 단위 검사는 기본값(0.6/0.25)과 무관하게 명시 값으로
    for k, v in arc.items():
        setattr(n.var.p.arc, k, v)
    return n


def _shift_after_left_turn(n, c, n_pts=40, steps=200):
    """직전 조향 -100(왼쪽) 을 유지해 κ_f 를 포화시킨 뒤 옆오차 c 경로에서 shift 를 받는다."""
    for _ in range(steps):
        n.prev_steer = -100; n.steer(n.p, _Traj(0.0, 0.0, c, n_pts))
    return n.arc_shift


def test_arc_limiter_default_none_unchanged(tc, tmp_path):
    d = make(tc, tmp_path, flags={"arc_comp"}).var.p.arc              # 코드 기본값(10/31 벤치 채택값)
    assert (d.limiter, d.kappa_gain, d.shift_max_m) == ('conf', 0.6, 0.25)
    n = _arc_n(tc, tmp_path)
    assert _shift_after_left_turn(n, -0.05) == -n.var.p.arc.shift_max_m       # 같은 부호여도 상한까지 그대로 (게이트 없음)


def test_arc_limiter_cancel_never_flips_sign(tc, tmp_path):
    n = _arc_n(tc, tmp_path, limiter='cancel')
    assert _shift_after_left_turn(n, 0.5) == -0.4                              # 반대 부호·|cte_raw| 0.5 > 상한 0.4 → 상한
    assert _shift_after_left_turn(n, 0.1) == -0.1                              # |shift| ≤ |cte_raw|: cte 가 0 까지만 (부호 안 뒤집음)
    assert _shift_after_left_turn(n, -0.05) == 0.0                             # 같은 부호 → 0 (반대쪽으로 못 밈)
    assert _shift_after_left_turn(n, 0.0) == 0.0
    for c in (0.5, 0.1, -0.05, 0.3):
        sh = _shift_after_left_turn(n, c)
        assert (c + sh) * c >= 0.0 and abs(sh) <= abs(c) + 1e-12


def test_arc_limiter_geo_cancels_cte(tc, tmp_path):
    n = _arc_n(tc, tmp_path, limiter='geo')
    for c in (0.3, -0.2):
        sh = _shift_after_left_turn(n, c)
        assert abs(sh + c) < 1e-12 and abs(n.cte_raw + sh) < 1e-12               # κ_geo = 2c/d² → shift = −c
    assert _shift_after_left_turn(n, 0.9) == -n.var.p.arc.shift_max_m          # 상한은 유지


def test_arc_limiter_conf_weights_by_num_points(tc, tmp_path):
    n = _arc_n(tc, tmp_path, limiter='conf')
    full = _shift_after_left_turn(n, 0.5, n_pts=40)
    assert full == -n.var.p.arc.shift_max_m
    low = _shift_after_left_turn(n, 0.5, n_pts=7)                              # min_points+1 → conf_min_weight
    assert abs(low - full * n.var.p.arc.conf_min_weight) < 1e-12
    mid = _shift_after_left_turn(n, 0.5, n_pts=15)
    assert full < mid < low
    assert _shift_after_left_turn(n, 0.5, n_pts=6) == 0.0                     # num_points 게이트는 그대로


def test_arc_kappa_gain_scales_kappa(tc, tmp_path):
    n1 = _arc_n(tc, tmp_path); n2 = _arc_n(tc, tmp_path, kappa_gain=0.5)
    for _ in range(200):
        n1.prev_steer = n2.prev_steer = -60
        n1.steer(n1.p, _Traj(0.0, 0.0, 0.0, 40)); n2.steer(n2.p, _Traj(0.0, 0.0, 0.0, 40))
    assert abs(n2.kappa_f - 0.5 * n1.kappa_f) < 1e-9 and n2.arc_shift > n1.arc_shift


def test_arc_limiter_constraint_rejects_unknown(tc, tmp_path):
    import variant as V
    log = []
    _, p = V.parse_variants({"profiles": {}, "params": {"arc": {"limiter": "bogus"}}}, log=log.append)
    assert p.arc.limiter == 'conf' and any("arc" in s for s in log)
    _, p2 = V.parse_variants({"profiles": {}, "params": {"arc": {"limiter": "cancel", "kappa_gain": 0.6}}}, log=log.append)
    assert p2.arc.limiter == 'cancel' and p2.arc.kappa_gain == 0.6


# ---------------- front_axle_ref (고전 Stanley: 앞축 평가) ----------------
def _front_arc_coefs(R_r, xs, origin, sign=1.0):
    """뒷축 반경 R_r 원호 주행(원 중심 = 뒷축 옆 sign·R_r, 앞축이 원 위) 의 경로를 경로 좌표 xs 점에서 2차 적합 → (a,b,c).
    이 원 위에서는 앞축 접선각 = atan(L/R_r) = 필요한 조향, 앞축 옆오차 0."""
    import vehicle
    L = vehicle.wheelbase_m()
    R = math.hypot(R_r, L)
    xg = origin + np.asarray(xs, float)
    ys = sign * (R_r - np.sqrt(R * R - xg * xg))
    a, b, c = np.polyfit(np.asarray(xs, float), ys, 2)
    return float(a), float(b), float(c)


def _front_n(tc, tmp_path, tag, **front):
    """front_axle_ref 제어기 + params.front 덮어쓰기."""
    tdir = tmp_path / ("tune_front_%s" % tag); tdir.mkdir()
    (tdir / "tune.json").write_text(json.dumps(HEAD_TUNE))
    (tdir / "variants.json").write_text(json.dumps({"profiles": {"t": ["front_axle_ref"]}, "params": {"front": front}}))
    (tdir / "algo").write_text("t\n")
    return tc.Tunable(run="", tune_dir=str(tdir), now=Clock())


@pytest.mark.parametrize("R_r,sign", [(3.0, 1.0), (2.0, -1.0), (5.0, 1.0), (10.0, -1.0)])
def test_front_arc_steady_delta_exact(tc, tmp_path, R_r, sign):
    """정상 상태 δ = atan(L/R_r), 옆오차 0: arc 외삽은 anchor 근처 3점 적합, poly 외삽은 앞축 근처 3점 적합(2차식 = 원의 국소 근사)."""
    target = math.atan(tc.WHEELBASE * sign / R_r)
    x_e = tc.WHEELBASE - tc.PATH_ORIGIN_X_M
    n = _front_n(tc, tmp_path, "arc%g%g" % (R_r, sign), ahead_m=0.0, heading_gain=1.0)   # 고전 정의(앞축, 헤딩 1)
    x_a = n.var.p.front.anchor_x_m
    assert n.var.p.front.extrap == 'arc'
    a, b, c = _front_arc_coefs(R_r, [x_a - 0.05, x_a, x_a + 0.05], tc.PATH_ORIGIN_X_M, sign)
    e, psi, kappa = n._front_ref(a, b, c, x_e)
    assert abs(e) < 1e-2 and abs(psi - target) < 1e-2             # 3점 2차 적합의 곡률 오차만 (R 2: 5 mm)
    s, delta, sh = n.steer(n.p, _Traj(a, b, c, 40))
    assert abs(delta - target) < 2e-2 and sh == 0.0
    assert (s < 0) == (sign > 0) and n.kappa_f is None and n.arc_shift == 0.0
    m = _front_n(tc, tmp_path, "poly%g%g" % (R_r, sign), extrap="poly", ahead_m=0.0, heading_gain=1.0)
    a, b, c = _front_arc_coefs(R_r, [x_e - 0.1, x_e, x_e + 0.1], tc.PATH_ORIGIN_X_M, sign)
    assert abs(a * x_e * x_e + b * x_e + c) < 1e-4
    s, delta, sh = m.steer(m.p, _Traj(a, b, c, 40))
    assert abs(delta - target) < 2e-3


@pytest.mark.parametrize("R_r,sign", [(3.0, 1.0), (5.0, -1.0), (8.0, 1.0)])
def test_front_arc_steady_from_ipm_window(tc, tmp_path, R_r, sign):
    """IPM 창(경로 x 0~0.45 m) 적합을 앞축까지 외삽: arc 는 δ ≈ atan(L/R_r) (1.5° 안). poly 는 R 3 에서 접선각 부호가 뒤집힘."""
    n = _front_n(tc, tmp_path, "win%g%g" % (R_r, sign), ahead_m=0.0, heading_gain=1.0)
    a, b, c = _front_arc_coefs(R_r, np.linspace(0.0, 0.45, 10), tc.PATH_ORIGIN_X_M, sign)
    s, delta, sh = n.steer(n.p, _Traj(a, b, c, 40))
    target = math.atan(tc.WHEELBASE * sign / R_r)
    assert abs(delta - target) < math.radians(1.5)
    if R_r == 3.0:
        m = _front_n(tc, tmp_path, "poly_win", extrap="poly")
        e, psi, _ = m._front_ref(a, b, c, tc.WHEELBASE - tc.PATH_ORIGIN_X_M)
        assert psi * target < 0                                 # 2차식 외삽: 원의 고차항 누락 → 앞축 접선각 반대 부호


def test_front_arc_extrap_straight_matches_linear(tc, tmp_path):
    """κ = 0(a = 0) 이면 arc 외삽 = 직선 외삽 = poly."""
    n = make(tc, tmp_path, flags={"front_axle_ref"})
    x_e = tc.WHEELBASE - tc.PATH_ORIGIN_X_M
    e, psi, kappa = n._front_ref(0.0, 0.1, 0.05, x_e)
    assert kappa == 0.0 and abs(e - (0.1 * x_e + 0.05)) < 1e-12 and abs(psi - math.atan(0.1)) < 1e-12


def test_front_straight_offset_and_ahead(tc, tmp_path):
    """직선 옆오차 e: δ = atan(k·e/(v+v_s)) (헤딩 0). ahead_m = PATH_ORIGIN_X_M − 축간거리 면 평가점 = IPM 원점(e = c, ψ = atan b)."""
    n = make(tc, tmp_path, flags={"front_axle_ref"})
    p = n.p
    s, delta, sh = n.steer(p, _Traj(0.0, 0.0, 0.0, 40))
    assert s == 0 and delta == 0.0
    n.i_acc = 0.0
    s, delta, sh = n.steer(p, _Traj(0.0, 0.0, 0.1, 40))
    k = p["stanley_gain"]; v = p["vehicle_speed_mps"] + p["softening_speed_mps"]
    assert abs(delta - (math.atan(k * 0.1 / v) + p["i_gain"] * 0.1 * DT)) < 1e-12 and s < 0
    ahead = tc.PATH_ORIGIN_X_M - tc.WHEELBASE
    m = _front_n(tc, tmp_path, "ahead", ahead_m=ahead, extrap="poly", heading_gain=1.0)
    a, b, c = 0.4, 0.2, 0.05
    s, delta, sh = m.steer(m.p, _Traj(a, b, c, 40))
    x_e = tc.WHEELBASE - tc.PATH_ORIGIN_X_M + ahead
    e = a * x_e * x_e + b * x_e + c; dy = 2 * a * x_e + b
    assert abs(x_e) < 1e-12
    assert abs(delta - (math.atan(dy) + math.atan(k * e / v))) < 1e-9   # κ(x_e)=0.8 ≥ i_curv_max → I 항 0


def test_front_uses_speed_schedule_denominator(tc, tmp_path):
    """corner_slowdown·speed_schedule 과 조합: 분모 = v_eff(스케줄러 pwm) + v_s."""
    import vehicle
    n = make(tc, tmp_path, flags={"front_axle_ref", "corner_slowdown", "speed_schedule"})
    n.sched.pwm = 190.0
    s, delta, sh = n.steer(n.p, _Traj(0.0, 0.0, 0.1, 40))
    v = vehicle.v255_mps() * 190.0 / 255.0 + n.p["softening_speed_mps"]
    assert abs(delta - (math.atan(n.p["stanley_gain"] * 0.1 / v) + n.p["i_gain"] * 0.1 * DT)) < 1e-12
    assert n.corner_pwm(n.p, 90) < 255                         # 스케줄러도 같이 돈다 (|조향| 90 > 진입 60 → 감속 시작)


def test_front_exclusive_and_profile():
    import variant as V
    log = []
    prof, _ = V.parse_variants({"profiles": {"x": ["arc_comp", "front_axle_ref"], "y": ["front_axle_ref", "controller_pp"]}}, log=log.append)
    assert prof["x"] == frozenset({"front_axle_ref"}) and prof["y"] == frozenset({"controller_pp"})
    assert V.PROFILES["v2f"] == V.PROFILES["v2"] | {"front_axle_ref"}
    assert "front_axle_ref" not in V.PROFILES["base"]
    _, p = V.parse_variants({"params": {"front": {"ahead_m": -0.1}}}, log=log.append)
    assert p.front == V.FrontAxle() and any("front" in s for s in log)   # 범위 위반 → 그룹 코드 기본값


def test_front_not_called_in_base(tc, tmp_path, monkeypatch):
    n = make(tc, tmp_path)
    def boom(*a, **k): raise AssertionError("base 에서 front_axle_ref 호출")
    monkeypatch.setattr(n, "_steer_front", boom)
    for _ in range(10):
        feed(n, _Traj(0.05, 0.1, 0.2))


def test_front_defaults_bench_point(tc, tmp_path):
    """코드 기본값 = 10/31 벤치 채택값 (x_e −0.22, 헤딩 0.2, arc 외삽). 평가점은 vehicle.yaml 기하에서 계산."""
    n = make(tc, tmp_path, flags={"front_axle_ref"})
    f = n.var.p.front
    assert (f.ahead_m, f.heading_gain, f.k_scale, f.extrap, f.anchor_x_m) == (0.85, 0.2, 1.0, 'arc', 0.2)
    x_e = tc.WHEELBASE - tc.PATH_ORIGIN_X_M + f.ahead_m
    assert abs(x_e - (-0.22)) < 1e-9
    a, b, c = 0.3, 0.2, 0.1                                     # 왼쪽으로 굽는 경로 → 왼쪽 조향(명령 음수)
    s, delta, sh = n.steer(n.p, _Traj(a, b, c, 40))
    e, psi, kappa = n._front_ref(a, b, c, x_e)
    v = n.p["vehicle_speed_mps"] + n.p["softening_speed_mps"]
    assert abs(delta - (0.2 * psi + math.atan(n.p["stanley_gain"] * e / v))) < 1e-12 and s < 0   # κ 0.5 ≥ i_curv_max → I 항 0
