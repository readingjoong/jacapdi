# -*- coding: utf-8 -*-
"""주행 변형 선택 모듈 (ROS 임포트 없음). 경로 노드·제어기·셸·GUI 가 공유하는 단일 출처.

변형 = 이름 있는 프로필 → 기능 플래그 집합(frozenset). 플래그가 공집합(base)이면 어떤 변형 분기도 실행되지 않는다.
변형 상수 기본값은 아래 Params dataclass 한 곳(단일 출처). tune/variants.json(git 추적)은 프로필 추가와 파라미터 override 만
담는다 — 코드 기본값과 같은 항목은 적지 않는다(저장소 파일은 비어 있고, 테스트가 '기본값 복제 없음' 을 검사한다). 부재·빈 파일 = 코드 기본값.
유효값 = 코드 기본값 + override 는 `--show NAME` 으로 본다. 운전자 설정(Stanley 게인·PWM 등 13키)은 tune/tune.json 그대로.

파일:
  tune/algo           프로필 이름 한 줄. 부재 → 'base', 모르는 이름 → 'base' + 로그 1회(같은 내용이 유지되는 동안)
  tune/variants.json  {"profiles": {이름: [플래그...]}, "params": {"cross": {...}, "guard": {...}, ...}}
                      알 수 없는 키·플래그·그룹 → 오류 로그 + 무시. 값 범위 위반(CONSTRAINTS) → 로그 + 그 그룹은 코드 기본값.
                      UTF-8 이 아닌 바이트는 U+FFFD 로 읽어 '모르는 이름'/'파싱 실패' 경로로 보낸다(노드가 죽지 않음). 부재 → 코드 기본값.

사용(노드):
    from variant import Variant
    var = Variant(TUNE_DIR, log=self.get_logger().info)
    ...매 프레임:  var.load(now=now)      # mtime 이 바뀐 파일만 다시 읽음
    if 'one_side_clip' in var.flags: ...  # 상수는 var.p.guard.one_side_max_c_m
두 노드가 같은 로더를 쓰므로 노드 간 어긋남은 "mtime 기준 최대 1프레임"(각자 다음 주기에 같은 파일을 읽음).
Variant.from_name(NAME) 은 파일과 무관하게 고정된 변형(벤치·테스트·CLI): load() 가 파일을 읽지 않는다.

CLI:
    python3 nodes/variant.py --list          운전자 노출 프로필(DRIVER_PROFILES) 한 줄씩
    python3 nodes/variant.py --list-all      모든 프로필(variants.json 포함)
    python3 nodes/variant.py --check NAME    exit 0 = 아는 프로필, 1 = 모름
    python3 nodes/variant.py --show NAME     플래그·파라미터 덤프
    (--tune-dir DIR 로 tune 폴더 지정 가능, 기본 저장소 tune/)
"""
import dataclasses
import json
import os
import sys
import time
from dataclasses import dataclass, field, fields

TUNE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tune")
ALGO_NAME = "algo"            # tune/algo
VARIANTS_NAME = "variants.json"

# ---- 기능 플래그 (경로 노드·제어기가 `in var.flags` 로만 분기) ----
FLAGS = frozenset({
    'one_side_clip',        # (a) 선 하나일 때 |c| 상한
    'on_line_hold',         # (b) 선이 x=0 ±0.25 안이면 ref 유지
    'step_accept',          # (c) outlier 를 accept_step_m 씩 점진 수용
    'short_fit_guard',      # (d) 짧은 적합 폐기  — 기본 OFF(5절 벤치 뒤 결정)
    'solid_cross_recovery', # (e)(f)(g) 실선 넘음 복구: search_back, 후보 창, 즉시 수용
    'corner_slowdown',      # 제어기 속도 스케줄러
    'speed_schedule',       # Stanley 분모에 v_target 공급
    'arc_comp',             # 제어기 평가점 원호 보정(조향 κ)
    'controller_pp',        # Pure Pursuit 분기(arc_comp 와 배타)
    'lane_settle',          # 차선 변경 상태기계: ramp 끝 → SETTLING(pre-offset |c| < lane.settle_c_m 가 settle_sec 지속 또는 max_sec) → IDLE.
                            #   CHANGING/SETTLING 중 새 요청은 되돌리기(CHANGING 중, 원래 차선으로)만 즉시, 나머지는 IDLE 까지 보류
    'change_slope',         # 변경 중 S자 오프셋의 기울기 d(off)/ds 를 경로 b 에 더함 (ds = lane.assumed_speed_mps·dt, Stanley 헤딩 항이 S자를 봄)
})
EXCLUSIVE = (('arc_comp', 'controller_pp'),)   # 같은 프로필에 동시 허용 안 함 (뒤쪽 이름 유지, 앞쪽 제거)
REQUIRES = (('speed_schedule', 'corner_slowdown'),)   # 앞쪽은 뒤쪽이 있어야 함 (분모 v_eff 가 스케줄러 pwm 을 읽음). 없으면 앞쪽 제거

LANE_FLAGS = frozenset({'lane_settle', 'change_slope'})   # 차선 변경 플래그 (경로 노드 Lane). 골든 v1 은 이 둘이 없던 때의 출력 → test_variant_identity 가 뺀다
GUARDS = frozenset({'one_side_clip', 'on_line_hold', 'step_accept', 'solid_cross_recovery', 'lane_settle'})   # change_slope 는 기본 OFF(10/31 벤치에서 효과 미측정, 가정 속도 의존)
PROFILES = {
    'base': frozenset(),
    'v1':   GUARDS,
    'v2':   GUARDS | {'corner_slowdown', 'speed_schedule'},
    'v2a':  GUARDS | {'corner_slowdown', 'speed_schedule', 'arc_comp'},
    'v2p':  GUARDS | {'corner_slowdown', 'speed_schedule', 'controller_pp'},
}
DRIVER_PROFILES = ['base', 'v1', 'v2']   # 운전자 UI 노출 목록. v2a/v2p 는 벤치 뒤 승격
PROFILE_DESC = {                          # UI 한 줄 설명 (프로필 이름 옆)
    'base': '기존(중간평가 2차선)',
    'v1':   '1차선 경로 가드',
    'v2':   'v1 + 코너 감속',
    'v2a':  'v2 + 원호 보정',
    'v2p':  'v2 + Pure Pursuit',
}
DEFAULT_NAME = 'base'


# ---- 변형 상수 (단일 출처). 그룹.필드 이름은 variants.json "params" 와 1:1 ----
@dataclass
class Cross:
    """실선 넘음 복구(solid_cross_recovery) 상태기계 상수."""
    enter_m: float = -0.2             # (f) 직전 실선이 이 안(차 밑·반대쪽, mirror 좌표)에 있었을 때만 넘은 실선 창을 넓힘
    crossed_m: float = 0.2            # (f) 첫 줄 실선이 이보다 차 반대쪽이면 '넘은 실선' 로 보고 점선 규칙을 바꿈 m
    overlap_m: float = 0.3            # (f) 넘은 실선과 이 거리 안에 점선 끝이 있으면 실선 폐기(점선 오인) m
    search_back_m: float = 0.3        # (e) 넘은 뒤 경계 없을 때 실선 쪽으로 가정하는 중심 위치 m
    search_max_sec: float = 1.0       # (e) 탐색 최대 시간 s (초과 → fail)
    hold_last_sec: float = 0.5        # fail 시 직전 경로 유지 시간 s
    cand_max_cross_m: float = 0.6     # (f) 넘은 실선 첫 줄 후보 창 상한 m (base 0.2)
    c_max_cross_m: float = 0.7        # (f) 넘은 실선 하나로 만든 경로의 |c| 상한 m
    near_m: float = 0.25              # 선이 x=0 ±near_m 안 → STRADDLE
    max_age_sec: float = 1.5          # 마지막 실선 관측을 믿는 시간 s (>= search_max_sec 보장)
    confirm_frames: int = 2           # (g) 즉시 수용 전 연속 확인 프레임 수
    min_points: int = 15              # (g) 즉시 수용에 필요한 점 수 하한


@dataclass
class Guard:
    """경로 가드 (a)(b)(c)(d) 상수."""
    one_side_max_c_m: float = 0.45    # (a) 선 하나로 만든 경로의 |c| 상한 m
    on_line_half_m: float = 0.25      # (b) 선이 차 중심에서 이 안이면 "선을 밟고 있음" m
    on_line_hold_sec: float = 0.3     # (b) 그때 믿은 경로를 유지하는 최대 시간 s
    on_line_min_ref_c: float = 0.05   # (b) 믿은 경로가 선 반대쪽으로 이만큼은 가리킬 때만 유지 m
    accept_step_m: float = 0.3        # (c) outlier 한 번에 받아들이는 최대 변화 m
    ext_min_span_m: float = 0.0       # (d) 선 하나짜리 ext 적합의 x 범위 하한 m (0 = 사실상 꺼짐)
    ext_max_b: float = 9.0            # (d) 선 하나짜리 ext 적합의 |b| 상한 (9 = 사실상 꺼짐)


@dataclass
class Corner:
    """속도 스케줄러(corner_slowdown/speed_schedule) 상수. 판정 입력은 조향 명령 |%| (κ 가 아님: 25°·L 0.56 에서 60 % ↔ κ 0.48 1/m,
    환산은 tune.json max_steer_deg 에 따라 변한다)."""
    steer_on_pct: float = 60.0        # 감속 진입 |조향| %
    steer_off_pct: float = 45.0       # 감속 해제 |조향| % (히스테리시스, < steer_on_pct)
    pwm: int = 190                    # 코너 PWM 목표. 직선은 tune.json pwm 이며 코너 pwm 은 그 이상이 되지 않음(min)
    tau_sec: float = 0.3              # pwm 목표 저역통과 시정수 s


@dataclass
class Arc:
    """원호 보정(arc_comp) 상수."""
    kappa_tau_sec: float = 0.4        # 조향 κ 저역통과 시정수 s
    shift_max_m: float = 0.25         # 보정량 절대값 상한 m (10/31 벤치: 0.4 는 코너 출구 되돌림 과보정·위빙)
    min_points: int = 6               # num_points 가 이 값 이하면 보정 0 (탐색·되돌림 경로)
    kappa_gain: float = 0.6           # κ 모델 보정: κ_src = gain·tan(δ)/L (시뮬 실측 κ_act/κ_cmd ≈ 0.5~0.65 → 0.6. 실차는 요레이트/속도 대 tan δ/L 실측값으로)
    limiter: str = 'conf'             # 보정량 제한 규칙(기본 conf: 1차선 3/3·2차선 2/2, 10/31 벤치) ARC_LIMITERS: none(상한만) / cancel(상쇄만: cte_raw 와 반대 부호·|shift|≤|cte_raw|)
                                      #   / geo(κ 를 경로 c 에서 역산 2c/d² → shift=−c, heading 항만 남음) / conf(num_points 신뢰도 가중)
    conf_points: int = 24             # conf: num_points ≥ 이 값이면 전량, min_points+1 에서 conf_min_weight 로 선형 (두 선≈29, 선 하나 8~18)
    conf_min_weight: float = 0.5      # conf: num_points = min_points+1 일 때 가중치


ARC_LIMITERS = ('none', 'cancel', 'geo', 'conf')


@dataclass
class PP:
    """Pure Pursuit(controller_pp) 상수."""
    lookahead_x_m: float = 0.1        # 평가점 x_t (경로 좌표) m (뒷축 기준 1.73 m: IPM 창 안이라 더 줄일 수 없음)
    i_gain: float = 0.04              # 직선 I 항 게인 (base 누적기와 동일 규칙)
    gain: float = 1.0                 # κ_pp 배율 (1.0 = 원호 정확. >1 은 원호에서 (1−1/gain)·D²/2R 만큼 안쪽 치우침)
    heading_gain: float = 1.5         # 접선 잔차 보정 게인: κ += −gain·(θ_t − 2α)/D, PP 와 같은 부호일 때만 (원호 잔차 0 → 정확성 유지)


@dataclass
class LaneChange:
    """차선 변경 상태기계(lane_settle)·기울기 보상(change_slope) 상수. 변경 시간 자체는 운전자 키 tune.json lane_change_sec."""
    settle_c_m: float = 0.15          # SETTLING → IDLE: 오프셋 전 경로 |c| 가 이 안이면 "정착 중" m
    settle_sec: float = 0.5           # 그 상태가 이만큼 연속되면 IDLE s
    max_sec: float = 8.0              # SETTLING 상한 s (초과 → IDLE, 보류된 요청 실행)
    assumed_speed_mps: float = 0.85   # change_slope 의 ds = v·dt 에 쓰는 가정 속도 m/s (경로 노드는 실제 속도를 모름. 실차 PWM-속도 미실측 → 재확인)


@dataclass
class Params:
    cross: Cross = field(default_factory=Cross)
    guard: Guard = field(default_factory=Guard)
    corner: Corner = field(default_factory=Corner)
    arc: Arc = field(default_factory=Arc)
    pp: PP = field(default_factory=PP)
    lane: LaneChange = field(default_factory=LaneChange)

    def groups(self):
        """{그룹명: dataclass 인스턴스}."""
        return {f.name: getattr(self, f.name) for f in fields(self)}

    def to_dict(self):
        return {g: dataclasses.asdict(v) for g, v in self.groups().items()}


def param_keys():
    """{그룹명: 필드명 집합} — 테스트의 키 집합 검사용."""
    return {f.name: {ff.name for ff in fields(f.type)} for f in fields(Params)}


# 값 범위 제약 (그룹별). 위반 → 로그 + 그 그룹을 코드 기본값으로 되돌림. cross.max_age_sec ≥ search_max_sec 는 max() 보정(아래)
CONSTRAINTS = {
    'cross':  ((lambda c: c.search_max_sec > 0 and c.hold_last_sec >= 0 and c.max_age_sec > 0, "search_max_sec > 0, hold_last_sec >= 0, max_age_sec > 0"),
               (lambda c: c.near_m > 0 and c.cand_max_cross_m > 0 and c.c_max_cross_m > 0, "near_m, cand_max_cross_m, c_max_cross_m > 0"),
               (lambda c: c.confirm_frames >= 1 and c.min_points >= 0, "confirm_frames >= 1, min_points >= 0"),
               (lambda c: c.overlap_m >= 0 and c.crossed_m >= 0, "overlap_m, crossed_m >= 0")),
    'guard':  ((lambda g: g.one_side_max_c_m > 0 and g.on_line_half_m >= 0 and g.on_line_hold_sec >= 0, "one_side_max_c_m > 0, on_line_half_m/on_line_hold_sec >= 0"),
               (lambda g: g.accept_step_m > 0 and g.ext_min_span_m >= 0 and g.ext_max_b > 0, "accept_step_m > 0, ext_min_span_m >= 0, ext_max_b > 0")),
    'corner': ((lambda c: 0 <= c.steer_off_pct < c.steer_on_pct <= 100, "0 <= steer_off_pct < steer_on_pct <= 100"),
               (lambda c: 0 <= c.pwm <= 255 and c.tau_sec >= 0, "0 <= pwm <= 255, tau_sec >= 0")),
    'arc':    ((lambda a: a.kappa_tau_sec >= 0 and a.shift_max_m >= 0 and a.min_points >= 0, "kappa_tau_sec, shift_max_m, min_points >= 0"),
               (lambda a: a.limiter in ARC_LIMITERS and a.kappa_gain > 0 and a.conf_points > a.min_points and 0 <= a.conf_min_weight <= 1,
                "limiter in %s, kappa_gain > 0, conf_points > min_points, 0 <= conf_min_weight <= 1" % (ARC_LIMITERS,))),
    'pp':     ((lambda p: p.lookahead_x_m >= 0 and p.i_gain >= 0 and p.gain > 0 and p.heading_gain >= 0, "lookahead_x_m, i_gain, heading_gain >= 0, gain > 0"),),
    'lane':   ((lambda l: l.settle_c_m > 0 and l.settle_sec >= 0 and l.max_sec > 0 and l.assumed_speed_mps > 0,
                "settle_c_m > 0, settle_sec >= 0, max_sec > 0, assumed_speed_mps > 0"),),
}


def _coerce(ftype, v):
    if ftype is int:
        if isinstance(v, bool) or not isinstance(v, (int, float)): raise ValueError(v)
        if float(v) != int(v): raise ValueError(v)
        return int(v)
    if ftype is float:
        if isinstance(v, bool) or not isinstance(v, (int, float)): raise ValueError(v)
        return float(v)
    return v


SETTLE_NS = 50_000_000   # 커널 파일 시각 눈금(수 ms)보다 넉넉히. 이보다 최근에 바뀐 파일은 다음 주기에 한 번 더 확인


def _stat_key(path):
    """파일 정체 (mtime_ns, size). 부재 → None. 방금(SETTLE_NS 안) 바뀐 파일은 같은 눈금 안의 연속 쓰기를
    놓칠 수 있어 ("unsettled",) 를 돌려 다음 load() 에서 다시 읽게 한다(내용이 같으면 적용은 생략)."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    if 0 <= time.time_ns() - st.st_mtime_ns < SETTLE_NS:   # mtime 이 미래(시계 역행·복사된 파일)면 재읽기 없이 그대로 정체로 씀
        return ("unsettled",)
    return (st.st_mtime_ns, st.st_size)


def parse_variants(doc, log=None, where=VARIANTS_NAME):
    """variants.json 내용(dict) → (profiles dict, Params). 알 수 없는 키는 오류 로그 + 무시. 배타(EXCLUSIVE)·의존(REQUIRES)
    위반 플래그는 로그 + 제거. CONSTRAINTS 위반 그룹은 로그 + 코드 기본값. cross.max_age_sec < cross.search_max_sec 면 max() 로 올리고 경고."""
    log = log or (lambda s: None)
    profiles = dict(PROFILES); p = Params()
    if not isinstance(doc, dict):
        log("[variant] %s: 최상위가 객체가 아님 -> 무시" % where); return profiles, p
    for k in doc:
        if k not in ("profiles", "params"):
            log("[variant] %s: 알 수 없는 키 '%s' 무시" % (where, k))
    prof = doc.get("profiles", {})
    if not isinstance(prof, dict):
        log("[variant] %s: profiles 가 객체가 아님 -> 무시" % where); prof = {}
    for name, flist in prof.items():
        if not isinstance(name, str) or not name or not isinstance(flist, (list, tuple)) or not all(isinstance(f, str) for f in flist):
            log("[variant] %s: 프로필 '%s' 형식 오류(플래그 이름 목록이어야 함) 무시" % (where, name)); continue
        bad = [f for f in flist if f not in FLAGS]
        for f in bad:
            log("[variant] %s: 프로필 '%s' 의 알 수 없는 플래그 '%s' 무시" % (where, name, f))
        fl = set(f for f in flist if f in FLAGS)
        for a, b in EXCLUSIVE:
            if a in fl and b in fl:
                log("[variant] %s: 프로필 '%s' 에 배타 플래그 %s·%s 동시 지정 -> '%s' 제거" % (where, name, a, b, a)); fl.discard(a)
        for a, b in REQUIRES:
            if a in fl and b not in fl:
                log("[variant] %s: 프로필 '%s' 의 %s 는 %s 가 필요 -> '%s' 제거" % (where, name, a, b, a)); fl.discard(a)
        if name == DEFAULT_NAME and fl:
            log("[variant] %s: 프로필 'base' 는 플래그 공집합이어야 함 -> 무시" % where); continue
        profiles[name] = frozenset(fl)
    params = doc.get("params", {})
    if not isinstance(params, dict):
        log("[variant] %s: params 가 객체가 아님 -> 무시" % where); params = {}
    groups = p.groups()
    for g, kv in params.items():
        if g not in groups:
            log("[variant] %s: 알 수 없는 params 그룹 '%s' 무시" % (where, g)); continue
        if not isinstance(kv, dict):
            log("[variant] %s: params.%s 가 객체가 아님 -> 무시" % (where, g)); continue
        ftypes = {f.name: f.type for f in fields(groups[g])}
        for k, v in kv.items():
            if k not in ftypes:
                log("[variant] %s: 알 수 없는 키 params.%s.%s 무시" % (where, g, k)); continue
            try:
                setattr(groups[g], k, _coerce(ftypes[k], v))
            except (TypeError, ValueError):
                log("[variant] %s: params.%s.%s 값 %r 형식 오류 무시" % (where, g, k, v))
    for g, checks in CONSTRAINTS.items():
        bad = [msg for pred, msg in checks if not pred(groups[g])]
        if bad:
            log("[variant] %s: params.%s 범위 위반(%s) -> 그룹 코드 기본값" % (where, g, "; ".join(bad)))
            setattr(p, g, type(groups[g])())
    if p.cross.max_age_sec < p.cross.search_max_sec:
        log("[variant] cross.max_age_sec %.3g < search_max_sec %.3g -> %.3g 으로 올림" % (p.cross.max_age_sec, p.cross.search_max_sec, p.cross.search_max_sec))
        p.cross.max_age_sec = max(p.cross.max_age_sec, p.cross.search_max_sec)
    return profiles, p


def read_text(path):
    """tune 파일 원문(UTF-8, 깨진 바이트는 U+FFFD — 노드가 UnicodeDecodeError 로 죽지 않게). 부재 → None."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except FileNotFoundError:
        return None


def parse_variants_text(text, log=None, where=VARIANTS_NAME):
    """원문(None = 부재) → (profiles, Params). 파싱 실패 → 코드 기본값 + 로그."""
    log = log or (lambda s: None)
    if text is None:
        return dict(PROFILES), Params()
    try:
        doc = json.loads(text)
    except ValueError as e:
        log("[variant] %s 파싱 실패(%s) -> 코드 기본값" % (where, e)); return dict(PROFILES), Params()
    return parse_variants(doc, log, where)


def load_variants_file(path, log=None):
    """variants.json 파일 → (profiles, Params). 부재/파싱 실패 → 코드 기본값(실패는 로그)."""
    log = log or (lambda s: None)
    try:
        text = read_text(path)
    except OSError as e:
        log("[variant] %s 읽기 실패(%s) -> 코드 기본값" % (path, e)); return dict(PROFILES), Params()
    return parse_variants_text(text, log, os.path.basename(path))


class Variant:
    """현재 변형: .name(str), .flags(frozenset), .p(Params). load() 가 tune/algo·variants.json 을 mtime 기준으로 갱신.

    - tune/algo 부재 → 'base'. 모르는 이름 → 'base' + 로그 1회(파일 내용이 바뀔 때까지 반복 안 함).
    - variants.json 부재 → 코드 기본값. 변경 시 프로필 표·파라미터 재적용(현재 이름이 사라지면 'base').
    - 파일이 안 바뀐 주기에 load() 가 하는 일은 stat 두 번뿐(부수 효과 없음). 방금 바뀐 파일(SETTLE_NS 안)만 한 번 더 읽음.
    - 두 노드가 각자 인스턴스를 갖고 같은 파일을 읽으므로 노드 간 어긋남은 mtime 기준 최대 1프레임.
    - from_name() 으로 만든 고정 변형은 load() 가 파일을 읽지 않는다 (.fixed = True)."""

    def __init__(self, tune_dir=TUNE_DIR, log=None, now=None):
        self.tune_dir = tune_dir
        self.log = log or (lambda s: None)
        self.name = DEFAULT_NAME; self.flags = PROFILES[DEFAULT_NAME]; self.p = Params()
        self.profiles = dict(PROFILES)
        self.loaded_at = now; self.changed_at = now
        self._algo_id = self._var_id = ("unset",)   # 아직 안 읽음 (None 은 "부재 확인")
        self._algo_raw = None; self._var_raw = None; self._warned_raw = None
        self.fixed = False
        self.load(now=now)

    # -- 경로 --
    @property
    def algo_path(self):
        return os.path.join(self.tune_dir, ALGO_NAME)

    @property
    def variants_path(self):
        return os.path.join(self.tune_dir, VARIANTS_NAME)

    def load(self, tune_dir=None, now=None):
        """바뀐 파일만 다시 읽는다. 반환: 이름·플래그·파라미터 중 하나라도 바뀌면 True. 고정 변형(from_name)은 항상 False."""
        if self.fixed:
            return False
        if tune_dir is not None:
            if tune_dir != self.tune_dir:
                self._algo_id = self._var_id = ("unset",)
            self.tune_dir = tune_dir
        self.loaded_at = now
        before = (self.name, self.flags, self.p)
        first = self._var_id == ("unset",)
        vid = _stat_key(self.variants_path)
        if vid != self._var_id or vid == ("unsettled",):
            self._var_id = vid
            try:
                text = read_text(self.variants_path)
            except OSError:
                self._var_id = ("unset",); text = self._var_raw          # 쓰는 도중 → 다음 주기에 다시
            if first or text != self._var_raw:
                self._var_raw = text
                self.profiles, self.p = parse_variants_text(text, self.log)
                if text is not None: self.log("[variant] %s 읽음: 프로필 %s" % (VARIANTS_NAME, ",".join(sorted(self.profiles))))
                self._apply_name(self._algo_raw)
        first = self._algo_id == ("unset",)
        aid = _stat_key(self.algo_path)
        if aid != self._algo_id or aid == ("unsettled",):
            self._algo_id = aid
            raw = None
            if aid is not None:
                try:
                    raw = read_text(self.algo_path)
                    raw = None if raw is None else raw.strip()
                except OSError:
                    self._algo_id = ("unset",); raw = self._algo_raw   # 쓰는 도중 → 다음 주기에 다시
            if first or raw != self._algo_raw:
                self._algo_raw = raw
                self._apply_name(raw)
        changed = (self.name, self.flags) != before[:2] or self.p != before[2]
        if changed: self.changed_at = now
        return changed

    def _apply_name(self, raw):
        """tune/algo 내용 → name/flags. 부재(None)·빈 문자열 → base, 모름 → base + 로그 1회."""
        if raw is None or raw == "":
            name = DEFAULT_NAME
        elif raw in self.profiles:
            name = raw
        else:
            if raw != self._warned_raw:
                self.log("[variant] tune/algo '%s' 모름 -> base (아는 이름: %s)" % (raw, ",".join(sorted(self.profiles))))
                self._warned_raw = raw
            name = DEFAULT_NAME
        if raw in self.profiles: self._warned_raw = None
        if name != self.name or self.profiles[name] != self.flags:
            self.log("[variant] algo %s flags=%s" % (name, ",".join(sorted(self.profiles[name])) or "-"))
        self.name = name; self.flags = self.profiles[name]

    @classmethod
    def from_name(cls, name, tune_dir=TUNE_DIR, log=None):
        """이름으로 고정한 Variant (벤치·테스트·CLI). variants.json 은 생성 때 한 번 읽고, 이후 load() 는 파일을 보지 않는다.
        모르는 이름 → ValueError."""
        v = cls.__new__(cls)
        v.tune_dir = tune_dir; v.log = log or (lambda s: None)
        v.profiles, v.p = load_variants_file(os.path.join(tune_dir, VARIANTS_NAME), v.log)
        if name not in v.profiles: raise ValueError("모르는 변형 '%s' (아는 이름: %s)" % (name, ",".join(sorted(v.profiles))))
        v.name = name; v.flags = v.profiles[name]
        v.loaded_at = v.changed_at = None
        v._algo_id = v._var_id = ("fixed",); v._algo_raw = name; v._var_raw = None; v._warned_raw = None
        v.fixed = True
        return v

    def describe(self):
        """--show 출력 문자열."""
        lines = ["variant %s  (%s)" % (self.name, PROFILE_DESC.get(self.name, "variants.json 정의"))]
        lines.append("flags: %s" % (" ".join(sorted(self.flags)) or "(없음 = base)"))
        for g, obj in self.p.groups().items():
            for f in fields(obj):
                lines.append("%s.%s = %s" % (g, f.name, getattr(obj, f.name)))
        return "\n".join(lines)


def known_profiles(tune_dir=TUNE_DIR):
    """코드 + variants.json 프로필 이름 목록(정렬)."""
    prof, _ = load_variants_file(os.path.join(tune_dir, VARIANTS_NAME))
    return sorted(prof)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="주행 변형 프로필 조회 (셸·GUI·sim 은 이 출력만 믿는다)")
    ap.add_argument("--tune-dir", default=TUNE_DIR)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="운전자 노출 프로필 한 줄씩")
    g.add_argument("--list-all", action="store_true", help="모든 프로필(variants.json 포함)")
    g.add_argument("--check", metavar="NAME", help="아는 프로필이면 exit 0, 아니면 1")
    g.add_argument("--show", metavar="NAME", help="플래그·파라미터 덤프")
    a = ap.parse_args(argv)
    errs = []
    if a.list:
        print("\n".join(DRIVER_PROFILES)); return 0
    if a.list_all:
        print("\n".join(known_profiles(a.tune_dir))); return 0
    if a.check is not None:
        return 0 if a.check in known_profiles(a.tune_dir) else 1
    try:
        v = Variant.from_name(a.show, a.tune_dir, log=errs.append)
    except ValueError as e:
        print(e, file=sys.stderr); return 1
    for e in errs: print(e, file=sys.stderr)
    print(v.describe()); return 0


if __name__ == "__main__":
    sys.exit(main())
