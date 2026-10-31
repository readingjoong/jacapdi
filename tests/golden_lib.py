# -*- coding: utf-8 -*-
"""골든 생성·비교 공용 도구: ROS 스텁, 주입 시계, 노드 모듈 로더, 프레임/시퀀스/제어기 격자 실행기.

- rclpy / interfaces_pkg 는 항상 스텁으로 바꿔 임포트한다 (실제 ROS 없이, 어떤 환경에서도 같은 결과).
- 시계: 모듈의 `time` 이름을 가짜 모듈로 바꿔 time.time()/time.monotonic() 이 CLOCK 을 돌려준다.
  (현재 노드는 time.time 직접 호출. 리팩터 뒤 self.now 콜러블이 생겨도 기본값이 time.time 이면 그대로 주입된다.)
- tune 파일: 모듈의 문자열 상수 중 <저장소>/tune 아래를 가리키는 것을 모두 임시 폴더로 바꾼다 (TUNE_DIR, LANE_FILE, ALGO_FILE, D, TUNE, STATUS, GO, ALGO ...).
- 노드 호출은 공개 진입점만: LanePathNode() 생성, setup(h,w), find_path(md,ms,header,mirror), on_detections(msg), stats, last_reason;
  Tunable() 생성, on_traj(msg), tick(), prev_steer, i_acc. 발행 메시지는 스텁 퍼블리셔가 토픽별로 모은다.
"""
import copy
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import types

import cv2
import numpy as np

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TESTS_DIR)
GOLDEN_DIR = os.path.join(TESTS_DIR, "golden")
H, W = 480, 640
DT = 0.05
T0 = 1000.0
STATS_BASE_KEYS = ["frames", "ok", "both", "one_side", "held", "search", "transverse_dropped",
                   "outlier_held", "outlier_accepted", "far_only_held", "cross_held", "dash_joined"]
STATS_V1_KEYS = ["c_clipped", "on_line_held", "outlier_stepped", "search_back", "solid_on_line_accept", "short_fit"]


# ---------------------------------------------------------------- 주입 시계
class FakeClock:
    def __init__(self, t0=T0):
        self.t = float(t0)

    def __call__(self):
        return self.t

    def set(self, t):
        self.t = float(t)

    def advance(self, dt):
        self.t += dt


CLOCK = FakeClock()


def fake_time_module():
    import time as real
    m = types.ModuleType("time")
    m.time = CLOCK; m.monotonic = CLOCK; m.perf_counter = CLOCK
    m.sleep = lambda s: None
    m.strftime = real.strftime; m.localtime = real.localtime; m.gmtime = real.gmtime
    return m


# ---------------------------------------------------------------- ROS 스텁
class _Msg:
    FIELDS = {}

    def __init__(self, **kw):
        for k, v in self.FIELDS.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)

    def __repr__(self):
        return "%s(%s)" % (type(self).__name__, ", ".join("%s=%r" % (k, getattr(self, k)) for k in self.FIELDS))


class Stamp(_Msg):
    FIELDS = {"sec": 0, "nanosec": 0}


class Header(_Msg):
    FIELDS = {"stamp": Stamp, "frame_id": ""}


class Point2D(_Msg):
    FIELDS = {"x": 0.0, "y": 0.0}


class Mask(_Msg):
    FIELDS = {"height": 0, "width": 0, "data": list}


class Detection(_Msg):
    FIELDS = {"class_name": "", "score": 0.0, "mask": Mask}


class DetectionArray(_Msg):
    FIELDS = {"header": Header, "detections": list}


class LaneTrajectory(_Msg):
    FIELDS = {"header": Header, "valid": False, "a": 0.0, "b": 0.0, "c": 0.0,
              "x_min_m": 0.0, "x_max_m": 0.0, "fit_error_m": 0.0, "num_points": 0}


class MotionCommand(_Msg):
    FIELDS = {"steering": 0, "left_speed": 0, "right_speed": 0}


class Duration:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = int(nanoseconds)


class Time:
    def __init__(self, nanoseconds=0, seconds=0, clock_type=None):
        self.nanoseconds = int(nanoseconds) + int(seconds * 1e9)

    @classmethod
    def from_msg(cls, st, clock_type=None):
        return cls(nanoseconds=int(st.sec) * 1_000_000_000 + int(st.nanosec))

    def to_msg(self):
        return Stamp(sec=self.nanoseconds // 1_000_000_000, nanosec=self.nanoseconds % 1_000_000_000)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)


def stamp_of(t):
    sec = int(t); return Stamp(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class Clock:
    def now(self):
        return Time(nanoseconds=int(round(CLOCK() * 1e9)))


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level):
        def f(msg, *a, **k):
            self.lines.append((level, str(msg)))
        return f

    def __getattr__(self, name):
        if name in ("info", "warn", "warning", "error", "debug", "fatal"):
            return self._log(name)
        raise AttributeError(name)


class Publisher:
    def __init__(self, topic):
        self.topic = topic; self.msgs = []

    def publish(self, m):
        self.msgs.append(copy.deepcopy(m))


class Parameter:
    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        return self


class Node:
    def __init__(self, name="node", **kw):
        self._name = name; self._pubs = {}; self._logger = Logger(); self._params = {}

    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def create_publisher(self, msg_type, topic, *a, **k):
        p = Publisher(topic); self._pubs[topic] = p; return p
    def get_logger(self): return self._logger
    def get_clock(self): return Clock()
    def destroy_node(self): pass
    def get_name(self): return self._name
    def declare_parameter(self, name, value=None, *a, **k):
        self._params.setdefault(name, Parameter(value)); return self._params[name]
    def get_parameter(self, name):
        return self._params.get(name, Parameter(None))


def _mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); return m


def install_ros_stubs():
    """sys.modules 에 rclpy / interfaces_pkg / std_msgs 스텁을 넣는다 (여러 번 불러도 같은 객체)."""
    if getattr(sys.modules.get("rclpy"), "__golden_stub__", False):
        return
    class _Enum:
        def __getattr__(self, n): return n
    rclpy = _mod("rclpy", init=lambda *a, **k: None, ok=lambda: True, spin=lambda n: None,
                 spin_once=lambda *a, **k: None, shutdown=lambda *a, **k: None, try_shutdown=lambda *a, **k: None)
    rclpy.__golden_stub__ = True
    rclpy.node = _mod("rclpy.node", Node=Node)
    qos = _mod("rclpy.qos", QoSProfile=lambda **k: dict(k))
    for n in ("QoSReliabilityPolicy", "QoSHistoryPolicy", "QoSDurabilityPolicy", "ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, n, _Enum())
    rclpy.qos = qos
    rclpy.time = _mod("rclpy.time", Time=Time, Duration=Duration)
    rclpy.duration = _mod("rclpy.duration", Duration=Duration)
    rclpy.clock = _mod("rclpy.clock", Clock=Clock)
    rclpy.parameter = _mod("rclpy.parameter", Parameter=Parameter)
    for k, v in (("rclpy", rclpy), ("rclpy.node", rclpy.node), ("rclpy.qos", qos), ("rclpy.time", rclpy.time),
                 ("rclpy.duration", rclpy.duration), ("rclpy.clock", rclpy.clock), ("rclpy.parameter", rclpy.parameter)):
        sys.modules[k] = v
    msg = _mod("interfaces_pkg.msg", DetectionArray=DetectionArray, Detection=Detection, Point2D=Point2D, Mask=Mask,
               LaneTrajectory=LaneTrajectory, MotionCommand=MotionCommand)
    pkg = _mod("interfaces_pkg"); pkg.msg = msg
    sys.modules["interfaces_pkg"] = pkg; sys.modules["interfaces_pkg.msg"] = msg
    std = _mod("std_msgs.msg", Header=Header); stdp = _mod("std_msgs"); stdp.msg = std
    sys.modules["std_msgs"] = stdp; sys.modules["std_msgs.msg"] = std
    bi = _mod("builtin_interfaces.msg", Time=Stamp); bip = _mod("builtin_interfaces"); bip.msg = bi
    sys.modules["builtin_interfaces"] = bip; sys.modules["builtin_interfaces.msg"] = bi


# ---------------------------------------------------------------- 모듈 로더
LOCAL_HELPERS = ("ipm", "variant", "vehicle", "tunefile")   # nodes/ 안에서 서로 임포트하는 보조 모듈 (모듈 폴더마다 새로 읽음)


def repo_tune_dir_of(module_path):
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(module_path))), "tune")


def set_tune_dir(M, dst):
    """모듈의 문자열 상수 중 tune 폴더 아래를 가리키는 것을 dst 로 바꾼다 (보조 모듈 ipm/variant/vehicle 도 같이)."""
    src = getattr(M, "__golden_tune_dir__", repo_tune_dir_of(M.__file__))
    targets = [M] + [getattr(M, n) for n in LOCAL_HELPERS if isinstance(getattr(M, n, None), types.ModuleType)]
    for T in targets:
        for k, v in list(vars(T).items()):
            if isinstance(v, str) and (v == src or v.startswith(src + os.sep)):
                setattr(T, k, dst + v[len(src):])
    M.__golden_tune_dir__ = dst


def load_module(path, name, tune_dir):
    """노드 모듈을 rclpy 없이 임포트하고 시계·tune 폴더를 주입한다. path 의 폴더를 sys.path 앞에 넣어 `from ipm import Ipm` 이 그 폴더 것을 본다."""
    install_ros_stubs()
    d = os.path.dirname(os.path.abspath(path))
    for k in list(sys.modules):
        if k in LOCAL_HELPERS or k.split(".")[0] in LOCAL_HELPERS:
            del sys.modules[k]
    sys.path.insert(0, d)
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        M = importlib.util.module_from_spec(spec)
        sys.modules[name] = M
        spec.loader.exec_module(M)
    finally:
        sys.path.remove(d)
    ft = fake_time_module()
    M.time = ft
    for n in LOCAL_HELPERS:
        sub = getattr(M, n, None)
        if isinstance(sub, types.ModuleType) and hasattr(sub, "time"):
            sub.time = ft
    set_tune_dir(M, tune_dir)
    return M


def make_tune_dir(path, tune, algo=None, lane=None, variants=None, go_until=1e12):
    """임시 tune 폴더. tune: dict → tune.json, algo: None 이면 파일 없음, variants: dict → variants.json."""
    os.makedirs(path, exist_ok=True)
    for f in ("algo", "lane", "lane_eff", "variants.json", "status.json", "tune.json", "go_until"):
        try: os.remove(os.path.join(path, f))
        except OSError: pass
    with open(os.path.join(path, "tune.json"), "w") as f:
        json.dump(tune, f, indent=1)
    if algo is not None:
        open(os.path.join(path, "algo"), "w").write(algo + "\n")
    if lane is not None:
        open(os.path.join(path, "lane"), "w").write("%d\n" % lane)
    if variants is not None:
        with open(os.path.join(path, "variants.json"), "w") as f:
            json.dump(variants, f, indent=1)
    open(os.path.join(path, "go_until"), "w").write("%r\n" % float(go_until))
    return path


def git_show(rev, rel, repo=REPO_DIR):
    return subprocess.check_output(["git", "-C", repo, "show", "%s:%s" % (rev, rel)])


def export_rev(rev, dst, repo=REPO_DIR):
    """git 리비전의 nodes/lane_path_node.py, tunable_controller.py, ipm.py, config/ipm_c920.yaml, tune/tune.json 을 dst 에 푼다."""
    for rel in ("nodes/lane_path_node.py", "nodes/tunable_controller.py", "nodes/ipm.py", "config/ipm_c920.yaml", "tune/tune.json"):
        p = os.path.join(dst, rel); os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, "wb").write(git_show(rev, rel, repo))
    return dst


def pubs(node):
    return node._pubs


def roundtrip(x):
    """JSON 왕복 (tuple→list, numpy 스칼라→python) 로 골든과 같은 꼴로 만든다."""
    return json.loads(json.dumps(x, default=_default))


def _default(o):
    if isinstance(o, (np.floating,)): return float(o)
    if isinstance(o, (np.integer,)): return int(o)
    if isinstance(o, np.ndarray): return o.tolist()
    raise TypeError(type(o))


# ---------------------------------------------------------------- 경로 노드
def new_lane_node(M):
    CLOCK.set(T0)
    n = M.LanePathNode()
    n.setup(H, W)
    return n


def paint_masks(node, dets, h=H, w=W):
    """on_detections 의 마스크 칠하기와 같은 규칙 (점수 1.0). (md, ms, dropped)"""
    md = np.zeros((h, w), np.uint8); ms = np.zeros((h, w), np.uint8); dropped = 0
    for name, poly in dets:
        if name not in ("solid_line", "dashed_line"):
            continue
        if name == "solid_line" and node.is_transverse(poly):
            dropped += 1; continue
        cv2.fillPoly(ms if name == "solid_line" else md,
                     [np.clip(np.round(poly), [0, 0], [w - 1, h - 1]).astype(np.int32)], 255)
    return md, ms, dropped


def fp_tuple(ret):
    """find_path 반환 → [reason, a, b, c, x_min, x_max, err, num_points, band]. (traj, band, reason) 튜플과 PathResult 객체 둘 다 받음."""
    if isinstance(ret, tuple) and len(ret) == 3:
        traj, band, reason = ret
    else:
        traj, band, reason = ret.traj, ret.band, ret.reason
        if callable(reason):
            reason = reason()
    if traj is None:
        return [reason]
    return [reason, traj.a, traj.b, traj.c, traj.x_min_m, traj.x_max_m, traj.fit_error_m, traj.num_points,
            None if band is None else [[float(a), float(b)] for a, b in band]]


def run_frames(M, node, frames):
    """합성 프레임마다 find_path 한 번 (시계 0.05 s 씩 전진). [[이름, lane, dropped, 튜플...]]"""
    out = []
    for name, lane, dets in frames:
        CLOCK.advance(DT)
        md, ms, dropped = paint_masks(node, dets)
        if not (md.any() or ms.any()):
            out.append([name, lane, dropped, ["no_line"]]); continue
        ret = node.find_path(md, ms, Header(stamp=stamp_of(CLOCK())), lane == 1)
        out.append([name, lane, dropped, fp_tuple(ret)])
    return roundtrip(out)


def detection_array(dets, t, score=1.0, shape=(H, W), stamp_age=0.0):
    msg = DetectionArray(header=Header(stamp=stamp_of(t - stamp_age)))
    for cls, poly in dets:
        d = Detection(class_name=cls, score=score)
        d.mask.height, d.mask.width = shape
        d.mask.data = [Point2D(x=float(x), y=float(y)) for x, y in poly]
        msg.detections.append(d)
    return msg


def traj_fields(m):
    return [bool(m.valid), m.a, m.b, m.c, m.x_min_m, m.x_max_m, m.fit_error_m, m.num_points]


def run_sequence(M, node, steps, tune_dir):
    """steps: [dict(t, dets, score, stamp_age, lane, shape)] 를 on_detections 로 넣고 발행물을 모은다.
    프레임 기록 = [t, last_reason, traj 필드 or None, band 점 or None]; 끝에 stats."""
    tp = pubs(node)["/lane_trajectory"]; bp = pubs(node)["/lane_detections"]
    out = []
    for st in steps:
        CLOCK.set(st["t"])
        if st.get("lane") is not None:
            open(os.path.join(tune_dir, "lane"), "w").write("%d\n" % st["lane"])
        n_t, n_b = len(tp.msgs), len(bp.msgs)
        msg = detection_array(st["dets"], st["t"], st.get("score", 1.0), st.get("shape", (H, W)), st.get("stamp_age", 0.0))
        node.on_detections(msg)
        traj = traj_fields(tp.msgs[-1]) if len(tp.msgs) > n_t else None
        band = None
        if len(bp.msgs) > n_b:
            da = bp.msgs[-1]
            band = [[p.x, p.y] for d in da.detections for p in d.mask.data] if da.detections else []
        out.append([round(st["t"] - T0, 3), getattr(node, "last_reason", None), traj, band])
    return roundtrip({"frames": out, "stats": dict(node.stats)})


# ---------------------------------------------------------------- 제어기
def new_controller(M):
    CLOCK.set(T0)
    return M.Tunable("")


def ctrl_tick(M, n, traj, prev_steer, i_acc=0.0):
    """경로 하나 → tick 한 번. (steer, pwm_l, pwm_r, shift(status.json), i_acc)"""
    n.on_traj(traj)
    n.prev_steer = prev_steer; n.i_acc = i_acc; n.kappa_f = None
    n.n = 3                                   # 이번 tick 에 status.json 을 쓰게 함 (n % 4 == 0)
    cp = pubs(n)["/topic_control_signal"]; k = len(cp.msgs)
    n.tick()
    status_path = os.path.join(M.__golden_tune_dir__, "status.json")
    st = json.load(open(status_path))
    m = cp.msgs[-1] if len(cp.msgs) > k else None
    return [None if m is None else m.steering, None if m is None else m.left_speed, None if m is None else m.right_speed,
            st.get("shift"), n.i_acc]


GRID_A = [-0.5, -0.2, 0.0, 0.2, 0.5]
GRID_B = [-0.6, -0.3, 0.0, 0.3, 0.6]
GRID_C = [-0.7, -0.3, 0.0, 0.3, 0.7]
GRID_NPTS = [0, 5, 6, 7, 40]
GRID_PREV = [-100, -60, 0, 60, 100]


def run_ctrl_grid(M, n):
    out = []
    for a in GRID_A:
        for b in GRID_B:
            for c in GRID_C:
                for npts in GRID_NPTS:
                    for prev in GRID_PREV:
                        CLOCK.advance(DT)
                        t = LaneTrajectory(valid=True, a=a, b=b, c=c, x_min_m=0.0, x_max_m=0.7, fit_error_m=0.02, num_points=npts)
                        out.append([a, b, c, npts, prev] + ctrl_tick(M, n, t, prev))
    return roundtrip(out)


def run_ctrl_seq(M, n):
    """I 항 누적·포화 → 경로 상실 0.7 s (i_acc 0, kappa_f None) → 복귀 → 곡선. 틱마다 (steer, pwm, shift, i_acc)."""
    out = []
    plan = [(60, dict(a=0.0, b=0.0, c=0.2, num_points=40)), (14, None), (20, dict(a=0.0, b=0.05, c=-0.1, num_points=40)),
            (30, dict(a=0.3, b=0.1, c=0.05, num_points=12)), (10, dict(a=0.0, b=0.0, c=0.0, num_points=0))]
    n.prev_steer = 0; n.i_acc = 0.0
    cp = pubs(n)["/topic_control_signal"]
    for count, tr in plan:
        for _ in range(count):
            CLOCK.advance(DT)
            if tr is None:
                n.on_traj(LaneTrajectory(valid=False))
            else:
                n.on_traj(LaneTrajectory(valid=True, x_min_m=0.0, x_max_m=0.7, fit_error_m=0.02, **tr))
            n.n = 3; k = len(cp.msgs)
            n.tick()
            st = json.load(open(os.path.join(M.__golden_tune_dir__, "status.json")))
            m = cp.msgs[-1] if len(cp.msgs) > k else None
            out.append([None if m is None else m.steering, None if m is None else m.left_speed, st.get("shift"), n.i_acc,
                        st.get("kappa_cmd")])
    return roundtrip(out)


TUNE_B_OVERRIDE = {"stanley_gain": 1.5, "heading_gain": 0.5, "i_gain": 0.0, "max_steer_deg": 20.0, "pwm": 220,
                   "curvature_gain": 0.0, "corner_steer_pct": 40, "pwm_corner": 150}


def ctrl_tune_sets(tune):
    """격자에 쓰는 tune.json 두 벌: 그대로 / 게인·pwm 바꾼 것 (i_gain 0 분기 포함)."""
    b = dict(tune); b.update(TUNE_B_OVERRIDE)
    return {"A": dict(tune), "B": b}


# ---------------------------------------------------------------- 30 s 시퀀스 대본
def sequence_steps(ipm):
    """on_detections 30 s (20 fps = 600 프레임). 설계: outlier 0.5 s → far_only → cross_hold 1.5 → search 2.0 타임아웃 → fail 1.0
    → stale / no_mask / no_line → 횡단보도 실선 오인 → 커브 → 선 하나 → 차선 변경(2→1) → 1차선 실선 넘음 → 탐색 타임아웃."""
    import synth
    steps = []

    def seg(t0, t1, make, **kw):
        t = t0
        while t < t1 - 1e-9:
            dets = make(t)
            steps.append(dict(t=T0 + round(t, 3), dets=dets, **kw))
            t = round(t + DT, 3)

    def lf(lane, **k):
        return lambda t: synth.lane_frame(ipm, lane, **k)

    far_left = lambda t: synth.lane_frame(ipm, 2, dy=-1.0, dyaw=0, drop=(1, 2))              # 점선이 후보 창 밖(+1.425 m, 먼 곳만 보임) → 경계 없음 → search
    far_left_cw = lambda t: far_left(t) + synth.lane_frame(ipm, 2, dy=0, dyaw=0, drop=(0, 1, 2), extra=synth.crosswalk("crosswalk_marking", 0.4))

    seg(0.0, 3.0, lf(2, dy=0.0, dyaw=0, phase=0.0))                       # 믿은 경로 확립
    seg(3.0, 4.0, lf(2, dy=0.3, dyaw=0))                                  # 튐 0.3 m → outlier_hold 0.5 s → accepted / step
    seg(4.0, 5.2, lf(2, dy=0.3, dyaw=0, x_start=0.5))                     # 먼 곳만 → far_only_hold, ref 1 s 뒤 만료
    seg(5.2, 6.0, lf(2, dy=0.0, dyaw=0))
    seg(6.0, 9.5, far_left_cw)                                             # 횡단보도 + 경계 없음: cross_hold 1.5 → search 0.5 → timeout → held 1 s → invalid
    seg(9.5, 11.0, lf(2, dy=0.0, dyaw=0))
    seg(11.0, 11.5, lf(2, dy=0.0, dyaw=0), stamp_age=0.5)                 # stale
    seg(11.5, 12.0, lambda t: [])                                          # no_mask (빈 detections)
    seg(12.0, 13.2, lf(2, dy=0.0, dyaw=0), score=0.2)                      # 점수 미달 → no_line → held → invalid
    seg(13.2, 15.0, lf(2, dy=0.0, dyaw=0, extra=synth.crosswalk("solid_line", 0.5)))   # 횡단보도를 실선으로 → transverse_dropped
    seg(15.0, 16.0, lf(2, dy=0.0, dyaw=0, kind="arc", R=3.0, turn=1))
    seg(16.0, 17.0, lf(2, dy=0.1, dyaw=5, kind="arc", R=2.0, turn=-1))
    seg(17.0, 18.0, lf(2, dy=0.4, dyaw=0, drop=(0, 2)))                    # solid_only, |c| 큼 → v1 c_clip
    seg(18.0, 19.0, lf(2, dy=0.42, dyaw=0, drop=(1, 2)))                   # dashed_only, 점선 밟음 → v1 on_line_hold
    seg(19.0, 19.05, lf(2, dy=0.0, dyaw=0), lane=1)                        # 차선 변경 요청 → 2 s S 자 → 1차선 규칙
    seg(19.05, 22.0, lf(2, dy=0.0, dyaw=0))
    seg(22.0, 24.0, lf(1, dy=0.0, dyaw=0))
    seg(24.0, 25.0, lf(1, dy=0.55, dyaw=0))                                 # 1차선 실선 밟음 (mirror +0.125) → v1 (g) 즉시 수용
    seg(25.0, 26.0, lf(1, dy=0.75, dyaw=0, drop=(1, 2), extra=synth.parallel_dashes(1.0, phase=0.3)))   # 실선 넘음(+0.325) + 바깥 점선(주차 구획) → v1 (f)
    seg(26.0, 27.0, lambda t: synth.lane_frame(ipm, 1, dy=1.0, dyaw=0, drop=(0, 2)))                    # 경계 없음 → v1 (e) search_back / base search_left
    seg(27.0, 28.5, lambda t: synth.lane_frame(ipm, 1, dy=1.0, dyaw=0, drop=(0, 2)))                    # 탐색 타임아웃 → held → invalid
    seg(28.5, 30.0, lf(1, dy=0.0, dyaw=0))
    return steps
