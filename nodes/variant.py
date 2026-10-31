# -*- coding: utf-8 -*-
"""주행 변형 선택 모듈 (ROS 임포트 없음). 경로 노드·제어기·셸·GUI 가 공유하는 단일 출처.

변형 = 이름 있는 프로필 → 기능 플래그 집합(frozenset). 플래그가 공집합(base)이면 어떤 변형 분기도 실행되지 않는다.
차선 범위(10/31): 프로필 표 항목은 frozenset(두 차선 같은 플래그 — base/v1/v2/v2a/v2p/v2f) 또는 LaneScope(유효 차선별 플래그 — v3 = 2차선 base / 1차선 v2f).
  플래그 차선(단일 출처 = 경로 노드 Lane 의 실제 상태): 경로 노드가 매 프레임 lane.update() 다음에 정한다 —
  CHANGING(S자 ramp 중)이면 scope_hold 가 있으면 1, 없으면 목표 차선(change[3]), 그 밖(IDLE·SETTLING·보류 중)은 경계 규칙 lane.rules.
  경로 노드 Variant 는 set_lane(플래그 차선)으로 플래그를 풀고, 그 값이 바뀔 때와 시작할 때 tune/lane_eff 에 원자적으로 쓴다(tmp + os.replace).
  제어기 Variant 는 load() 가 tune/lane_eff 를 mtime 기준으로 읽는다(부재·잘못된 값 → tune/lane 요청 차선, 시작 때 둘 다 부재 → 2) → 어긋남 최대 1프레임.
  즉시 처리되는 요청: 2→1 은 S자 시작부터 1차선 플래그, 1→2 는 S자 시작부터 2차선 플래그(경로 노드는 ramp 끝까지 1차선 규칙으로 경계를 고름).
  scope_hold: 변경 구간(CHANGING) 전체를 1차선 플래그로 → 2→1·1→2 모두 S자는 1차선 알고리즘이 맡고 2차선 플래그는 2차선 규칙으로 달리는 동안만.
  lane_settle 이 새 요청을 보류하는 동안(SETTLING)은 지금 달리는 차선(rules)의 플래그를 유지하고, 보류가 풀려 ramp 가 시작되는 프레임부터 위 규칙.
  차선 변경 플래그(LANE_FLAGS)는 차선 범위와 무관한 프로필 수준: lane_scope() 가 어느 한 차선에 있으면 두 차선 모두에 둔다
  (변경 도중 차선이 바뀌어도 Lane 상태기계 규칙이 바뀌지 않게. 변경이 없으면 두 플래그는 출력에 영향 없음 → tests/test_lane_scope.py 골든).
변형 상수 기본값은 아래 Params dataclass 한 곳(단일 출처). tune/variants.json(git 추적)은 프로필 추가와 파라미터 override 만
담는다 — 코드 기본값과 같은 항목은 적지 않는다(저장소 파일은 비어 있고, 테스트가 '기본값 복제 없음' 을 검사한다). 부재·빈 파일 = 코드 기본값.
유효값 = 코드 기본값 + override 는 `--show NAME` 으로 본다. 운전자 설정(Stanley 게인·PWM 등 13키)은 tune/tune.json 그대로.

파일:
  tune/algo           프로필 이름 한 줄. 부재 → 'base', 모르는 이름 → 'base' + 로그 1회(같은 내용이 유지되는 동안)
  tune/lane           요청 차선 (1 또는 2). car.sh lane·tune_gui·트리거가 씀. 경로 노드 Lane 과 같은 규칙(parse_lane)
  tune/lane_eff       플래그 차선 (1 또는 2). 경로 노드만 씀(실행 중 파일, .gitignore). 제어기 Variant 가 읽음
  tune/variants.json  {"profiles": {이름: [플래그...] 또는 {"lane1": [...], "lane2": [...]}}, "params": {"cross": {...}, "guard": {...}, ...}}
                      알 수 없는 키·플래그·그룹 → 오류 로그 + 무시. 값 범위 위반(CONSTRAINTS) → 로그 + 그 그룹은 코드 기본값.
                      UTF-8 이 아닌 바이트는 U+FFFD 로 읽어 '모르는 이름'/'파싱 실패' 경로로 보낸다(노드가 죽지 않음). 부재 → 코드 기본값.

사용(노드):
    from variant import Variant
    var = Variant(TUNE_DIR, log=self.get_logger().info)
    ...매 프레임:  var.load(now=now)      # mtime 이 바뀐 파일만 다시 읽음
    if 'one_side_clip' in var.flags: ...  # 상수는 var.p.guard.one_side_max_c_m. flags 는 플래그 차선(var.eff_lane)으로 풀린 집합
두 노드가 같은 로더를 쓰므로 노드 간 어긋남은 "mtime 기준 최대 1프레임"(각자 다음 주기에 같은 파일을 읽음).
Variant.from_name(NAME) 은 파일과 무관하게 고정된 변형(벤치·테스트·CLI): load() 가 tune/algo·variants.json 을 읽지 않는다
(차선 범위 프로필이면 tune/lane_eff·tune/lane 만 읽어 차선별 플래그를 고른다 — 차선은 변형 선택이 아니라 주행 상태).

CLI:
    python3 nodes/variant.py --list          운전자 노출 프로필(DRIVER_PROFILES) 한 줄씩
    python3 nodes/variant.py --list-all      모든 프로필(variants.json 포함)
    python3 nodes/variant.py --check NAME    exit 0 = 아는 프로필, 1 = 모름
    python3 nodes/variant.py --show NAME     플래그(차선 범위 프로필은 차선별 줄 flags[1차선]/flags[2차선] 추가)·파라미터 덤프
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
LANE_NAME = "lane"            # tune/lane (요청 차선)
LANE_EFF_NAME = "lane_eff"    # tune/lane_eff (플래그 차선, 경로 노드가 씀)
LANES = (1, 2)
DEFAULT_LANE = 2              # tune/lane 부재로 시작할 때 (경로 노드 Lane 초기값과 같음)
DEFAULT_LANE_CHANGE_SEC = 2.0 # tune.json lane_change_sec 부재 시 S자 ramp 길이 (HEAD 값. 경로 노드 Lane·제어기 scope_hold 공용)
VARIANTS_NAME = "variants.json"

# ---- 기능 플래그 (경로 노드·제어기가 `in var.flags` 로만 분기) ----
FLAGS = frozenset({
    'one_side_clip',        # (a) 선 하나일 때 |c| 상한
    'on_line_hold',         # (b) 선이 x=0 ±0.25 안이면 ref 유지
    'step_accept',          # (c) outlier 를 accept_step_m 씩 점진 수용
    'short_fit_guard',      # (d) 짧은 적합 폐기  — 기본 OFF(5절 벤치 뒤 결정)
    'solid_cross_recovery', # (e)(f)(g) 실선 넘음 복구: search_back, 후보 창, 즉시 수용
    'cross_continuity',     # (f) 전제 보정: 직전 관측 없이 차 반대쪽에 처음 보이는 실선 = 옆 차로 바깥 실선 → 경계로 안 씀 (solid_cross_recovery 필요)
    'corner_slowdown',      # 제어기 속도 스케줄러
    'speed_schedule',       # Stanley 분모에 v_target 공급
    'arc_comp',             # 제어기 평가점 원호 보정(조향 κ)
    'controller_pp',        # Pure Pursuit 분기(arc_comp·front_axle_ref 와 배타)
    'front_axle_ref',       # 앞쪽 평가 Stanley: 평가점을 IPM 창 뒤(앞축 쪽)로 외삽(arc_comp·controller_pp 와 배타). 상수 front 그룹
    'lane_settle',          # 차선 변경 상태기계: ramp 끝 → SETTLING(pre-offset |c| < lane.settle_c_m 가 settle_sec 지속 또는 max_sec) → IDLE.
                            #   CHANGING/SETTLING 중 새 요청은 되돌리기(CHANGING 중, 원래 차선으로)만 즉시, 나머지는 IDLE 까지 보류
    'change_slope',         # 변경 중 S자 오프셋의 기울기 d(off)/ds 를 경로 b 에 더함 (ds = lane.assumed_speed_mps·dt, Stanley 헤딩 항이 S자를 봄)
    'scope_hold',           # 차선 범위 프로필 전용(경로 노드 플래그 차선): 변경 구간(Lane CHANGING, S자 ramp) 전체를 1차선 플래그로 → 양방향 S자를 1차선 알고리즘이 맡음
})
EXCLUSIVE = (('arc_comp', 'controller_pp'), ('arc_comp', 'front_axle_ref'), ('front_axle_ref', 'controller_pp'))   # 같은 프로필에 동시 허용 안 함 (뒤쪽 이름 유지, 앞쪽 제거)
REQUIRES = (('speed_schedule', 'corner_slowdown'), ('cross_continuity', 'solid_cross_recovery'))   # 앞쪽은 뒤쪽이 있어야 함 (분모 v_eff 가 스케줄러 pwm 을 읽음). 없으면 앞쪽 제거

LANE_FLAGS = frozenset({'lane_settle', 'change_slope', 'scope_hold'})   # 차선 변경 플래그 (프로필 수준). 골든 v1 은 이것들이 없던 때의 출력 → test_variant_identity 가 뺀다
GUARDS = frozenset({'one_side_clip', 'on_line_hold', 'step_accept', 'solid_cross_recovery', 'lane_settle'})   # change_slope 는 기본 OFF(10/31 벤치에서 효과 미측정, 가정 속도 의존)


@dataclass(frozen=True)
class LaneScope:
    """차선 범위 프로필 항목: 유효 차선별 최종 플래그 집합. 직접 만들지 말고 lane_scope() 로 (LANE_FLAGS 두 차선 동일 보장)."""
    lane1: frozenset
    lane2: frozenset

    def at(self, lane):
        return self.lane1 if lane == 1 else self.lane2


def lane_scope(lane1, lane2):
    """차선별 플래그 → LaneScope. LANE_FLAGS(차선 변경)는 프로필 수준이라 어느 한 차선에 있으면 두 차선 모두에 둔다."""
    lane1, lane2 = frozenset(lane1), frozenset(lane2)
    common = (lane1 | lane2) & LANE_FLAGS
    return LaneScope(lane1 | common, lane2 | common)


def flags_at(entry, lane):
    """프로필 표 항목(frozenset 또는 LaneScope) → 그 차선의 플래그 집합."""
    return entry.at(lane) if isinstance(entry, LaneScope) else entry


def parse_lane(raw):
    """tune/lane 원문 → 1/2, 그 밖(부재 None·빈 값·숫자 아님·범위 밖) → None (호출자는 직전 값 유지). 경로 노드 Lane.update 와 같은 규칙."""
    try:
        v = int(raw.strip())
    except (AttributeError, ValueError):
        return None
    return v if v in LANES else None


V3_LANE1 = GUARDS | {'corner_slowdown', 'speed_schedule', 'front_axle_ref', 'cross_continuity'}   # v3 의 1차선 주행 플래그 (= v2f + cross_continuity: 10/31 급좌코너 옆 차로 실선 오인 → 경로 뒤집힘 보정. v2f 근거: 10/31 벤치 1차선 20/20 완주·이탈 0 15/20, v2a 2/8. 바꿀 때는 이 한 줄만)
V3_CHANGE = frozenset({'lane_settle', 'scope_hold'})    # v3 차선 변경 플래그(프로필 수준, 두 차선). 10/31 슬롯5 1to2 정지: lane_settle 만 2/3 → +scope_hold 0/3.
                                                         #   change_slope 는 1to2 정지 0/3 이지만 2to1 에서 S 뒤 잔여 헤딩으로 코너2 정지 3/3 → 제외 (logs/matrix/D_hold_1to2, D_v3final)
PROFILES = {
    'base': frozenset(),
    'v1':   GUARDS,
    'v2':   GUARDS | {'corner_slowdown', 'speed_schedule'},
    'v2a':  GUARDS | {'corner_slowdown', 'speed_schedule', 'arc_comp'},
    'v2p':  GUARDS | {'corner_slowdown', 'speed_schedule', 'controller_pp'},
    'v2f':  GUARDS | {'corner_slowdown', 'speed_schedule', 'front_axle_ref'},
    'v3':   lane_scope(V3_LANE1 | V3_CHANGE, frozenset()),   # 1차선 = V3_LANE1, 2차선 = base(+ 프로필 수준 V3_CHANGE: 변경 없으면 출력 base 와 비트 동일)
}
DRIVER_PROFILES = ['base', 'v1', 'v2', 'v3']   # 운전자 UI 노출 목록. v2a/v2p/v2f 는 실험 프로필(PROFILES 에만, v2f 는 v3 의 1차선으로 노출)
PROFILE_DESC = {                          # UI 한 줄 설명 (프로필 이름 옆)
    'base': '기존(중간평가 2차선)',
    'v1':   '1차선 경로 가드',
    'v2':   'v1 + 코너 감속',
    'v2a':  'v2 + 원호 보정',
    'v2p':  'v2 + Pure Pursuit',
    'v2f':  'v2 + 앞쪽 평가 Stanley',
    'v3':   '2차선 기존 / 1차선 v2f + 실선 연속성',
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
class FrontAxle:
    """앞쪽 평가 Stanley(front_axle_ref) 상수. 평가점 x_e = 축간거리 − PATH_ORIGIN_X_M + ahead_m (IPM 좌표, 앞축 ≈ −1.07 m).
    고전 정의(ahead_m 0, heading_gain 1): 원호(뒷축 반경 R) 위에서 앞축이 경로에 있으면 앞축 접선각 = atan(L/R) = 필요한 조향 → 정상 옆오차 0.
    단 앞축은 IPM 창 첫 행보다 1.07 m 뒤라 값은 외삽뿐이고, 외삽은 창 안 (c, b, a) 의 재가중이다: 앞축 가중은 곡률 a 에 음의 가중
    (코너 진입에서 바깥으로 꺾음) → 10/31 벤치 1차선 x_e −1.07·−0.63·−0.40 모두 첫 코너 바깥 이탈·정지, −0.30 1/5·−0.26 2/3 완주.
    기본값 = 벤치 채택값 x_e −0.22 (뒷축 앞 1.41 m, base 1.73 m): 20/20 완주·엄격 15/20·이탈 평균 0.25 (v2a 8/8·2/8·1.9).
    −0.18 2/3 완주(정지 1: 코너3 경로 노드 c −0.7 오인), −0.15 는 안쪽(e_left 0.20, 이탈 4)."""
    ahead_m: float = 0.85             # 평가점의 앞축 앞 거리 m (0 = 앞축, PATH_ORIGIN_X_M − 축간거리(1.07) = IPM 원점). 0.85 → x_e −0.22.
                                      #   x_e 를 뒤로(작게) 할수록 원호 안쪽 평형↓ 바깥 이탈↑ (−0.26 정지 1/3, −0.30 정지 4/5: 여유 0.04~0.08 m)
    heading_gain: float = 0.2         # 헤딩 항 게인 (고전 정의 1 은 앞축에서만 성립). 벤치 채택 0.2 = tune.json heading_gain 과 같은 값(독립 상수).
                                      #   조향 미달(κ_실제/κ_명령 = g)이면 고전 정의에서는 1/g 로 올려야 정상 옆오차 0
    k_scale: float = 1.0              # tune.json stanley_gain 배율 (옆오차 항 atan(k_scale·k·e/(v+v_s)))
    extrap: str = 'arc'               # 평가점까지 외삽 규칙 FRONT_EXTRAPS: poly(2차식 그대로) / arc(anchor 점의 위치·접선·곡률 원을 따라 되돌아감).
                                      #   트랙 = 직선 + 일정 반경 원호. 2차식은 원의 고차항을 못 담아 IPM 창(뒷축 1.6~2.1 m) 적합을 1.07 m 뒤로 늘이면
                                      #   R 3 원호에서 앞축 접선각이 −4.9°(참 10.6°). arc 는 10.0° (10/31 b01: poly 는 첫 코너에서 바깥 이탈·정지)
    anchor_x_m: float = 0.2           # arc: 위치·접선·곡률을 읽는 점 (경로 좌표 m). 경로 노드 근거리 적합 창(0~0.45 m) 가운데라 곡률 추정이 가장 안정


FRONT_EXTRAPS = ('poly', 'arc')


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
    front: FrontAxle = field(default_factory=FrontAxle)
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
    'front':  ((lambda f: f.ahead_m >= 0 and f.heading_gain >= 0 and f.k_scale > 0, "ahead_m, heading_gain >= 0, k_scale > 0"),
               (lambda f: f.extrap in FRONT_EXTRAPS and f.anchor_x_m >= 0, "extrap in %s, anchor_x_m >= 0" % (FRONT_EXTRAPS,))),
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


def _is_flag_list(v):
    return isinstance(v, (list, tuple)) and all(isinstance(f, str) for f in v)


def _clean_flags(flist, label, log, where):
    """플래그 이름 목록 → 검증된 set. 모르는 플래그 무시, 배타(EXCLUSIVE)·의존(REQUIRES) 위반은 로그 + 제거. label = "이름'" 형식."""
    for f in flist:
        if f not in FLAGS:
            log("[variant] %s: 프로필 '%s 의 알 수 없는 플래그 '%s' 무시" % (where, label, f))
    fl = set(f for f in flist if f in FLAGS)
    for a, b in EXCLUSIVE:
        if a in fl and b in fl:
            log("[variant] %s: 프로필 '%s 에 배타 플래그 %s·%s 동시 지정 -> '%s' 제거" % (where, label, a, b, a)); fl.discard(a)
    for a, b in REQUIRES:
        if a in fl and b not in fl:
            log("[variant] %s: 프로필 '%s 의 %s 는 %s 가 필요 -> '%s' 제거" % (where, label, a, b, a)); fl.discard(a)
    return fl


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
        if not isinstance(name, str) or not name:
            log("[variant] %s: 프로필 '%s' 형식 오류(이름) 무시" % (where, name)); continue
        if isinstance(flist, dict):                                             # 차선 범위 {"lane1": [...], "lane2": [...]}
            extra = [k for k in flist if k not in ("lane1", "lane2")]
            if extra or not all(_is_flag_list(flist.get(k, [])) for k in ("lane1", "lane2")):
                log("[variant] %s: 프로필 '%s' 형식 오류(차선 범위는 lane1·lane2 플래그 목록만) 무시" % (where, name)); continue
            l1 = _clean_flags(flist.get("lane1", []), "%s'.lane1" % name, log, where)
            l2 = _clean_flags(flist.get("lane2", []), "%s'.lane2" % name, log, where)
            if (l1 ^ l2) & LANE_FLAGS:
                log("[variant] %s: 프로필 '%s' 차선 변경 플래그 %s 는 프로필 수준 -> 두 차선에 둠" % (where, name, ",".join(sorted((l1 ^ l2) & LANE_FLAGS))))
            entry = lane_scope(l1, l2)
        elif _is_flag_list(flist):
            entry = frozenset(_clean_flags(flist, name + "'", log, where))
        else:
            log("[variant] %s: 프로필 '%s' 형식 오류(플래그 이름 목록이어야 함) 무시" % (where, name)); continue
        if name == DEFAULT_NAME and entry != frozenset():
            log("[variant] %s: 프로필 'base' 는 플래그 공집합이어야 함 -> 무시" % where); continue
        profiles[name] = entry
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
    """현재 변형: .name(str), .flags(frozenset), .p(Params), .lane(요청 차선 tune/lane 1/2), .eff_lane(플래그를 고르는 차선).
    load() 가 tune/algo·variants.json·tune/lane·tune/lane_eff 를 mtime 기준으로 갱신. .flags = flags_at(프로필 항목, .eff_lane): 두 차선 같은 프로필은 차선과 무관.
    .eff_lane: set_lane() 을 부른 인스턴스(경로 노드)는 호출자가 준 값, 아니면(제어기) tune/lane_eff, 그것이 부재·잘못된 값이면 .lane.

    - tune/algo 부재 → 'base'. 모르는 이름 → 'base' + 로그 1회(파일 내용이 바뀔 때까지 반복 안 함).
    - variants.json 부재 → 코드 기본값. 변경 시 프로필 표·파라미터 재적용(현재 이름이 사라지면 'base').
    - 파일이 안 바뀐 주기에 load() 가 하는 일은 stat 네 번뿐(부수 효과 없음). 방금 바뀐 파일(SETTLE_NS 안)만 한 번 더 읽음.
    - 두 노드가 각자 인스턴스를 갖고 같은 파일을 읽으므로 노드 간 어긋남은 mtime 기준 최대 1프레임.
    - tune/lane: parse_lane 규칙(시작 때 부재 → 2, 이후 부재·잘못된 값 → 직전 값). tune/lane_eff: parse_lane 이 None 이면 .lane 으로 대체.
      플래그 차선이 바뀐 load() 는 True(플래그가 바뀐 경우).
    - from_name() 으로 만든 고정 변형은 load() 가 tune/algo·variants.json 을 읽지 않는다 (.fixed = True). 차선 범위 프로필이면 차선 파일만 읽는다."""

    def __init__(self, tune_dir=TUNE_DIR, log=None, now=None):
        self.tune_dir = tune_dir
        self.log = log or (lambda s: None)
        self.name = DEFAULT_NAME; self.flags = PROFILES[DEFAULT_NAME]; self.p = Params()
        self.profiles = dict(PROFILES)
        self.lane = self.eff_lane = DEFAULT_LANE
        self._file_eff = None; self._ext_lane = None                 # tune/lane_eff 값(None = 부재·잘못된 값), set_lane() 값(None = 안 부름)
        self.loaded_at = now; self.changed_at = now
        self._algo_id = self._var_id = self._lane_id = self._eff_id = ("unset",)   # 아직 안 읽음 (None 은 "부재 확인")
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

    @property
    def lane_path(self):
        return os.path.join(self.tune_dir, LANE_NAME)

    @property
    def lane_eff_path(self):
        return os.path.join(self.tune_dir, LANE_EFF_NAME)

    @property
    def scoped(self):
        """지금 프로필이 차선 범위(LaneScope) 인가."""
        return isinstance(self.profiles.get(self.name), LaneScope)

    def _eff(self):
        """플래그를 고르는 차선: set_lane() 값 > tune/lane_eff > 요청 차선 .lane."""
        if self._ext_lane is not None:
            return self._ext_lane
        return self._file_eff if self._file_eff is not None else self.lane

    def _read_lane_file(self, path, key):
        """mtime 이 바뀐 차선 파일 → ('same', None) 안 바뀜 / ('gone', None) 부재 / ('retry', None) 쓰는 도중 / ('read', parse_lane 값)."""
        lid = _stat_key(path)
        if lid == getattr(self, key) and lid != ("unsettled",):
            return "same", None
        setattr(self, key, lid)
        if lid is None:
            return "gone", None
        try:
            return "read", parse_lane(read_text(path))
        except OSError:
            setattr(self, key, ("unset",)); return "retry", None        # 쓰는 도중 → 다음 주기에 다시

    def _load_lane(self):
        """tune/lane(요청 차선: None 이면 직전 값)·tune/lane_eff(플래그 차선: 부재·None 이면 대체)를 바뀐 것만 다시 읽는다."""
        how, lane = self._read_lane_file(self.lane_path, "_lane_id")
        if how == "read" and lane is not None:
            self.lane = lane
        how, eff = self._read_lane_file(self.lane_eff_path, "_eff_id")
        if how in ("read", "gone"):
            self._file_eff = eff

    def set_lane(self, lane):
        """경로 노드 전용: 플래그 차선을 직접 준다(Lane 실제 상태에서 계산한 1/2). 이후 이 인스턴스는 tune/lane_eff 대신 이 값을 쓴다.
        반환: 플래그가 바뀌었는가."""
        self._ext_lane = lane
        if self.eff_lane == lane:
            return False
        before = self.flags; self.eff_lane = lane
        if self.scoped:
            self._resolve()
        return self.flags != before

    def _resolve(self):
        """지금 이름·차선 → .flags (차선 범위 프로필의 차선별 집합이 바뀌면 로그)."""
        fl = flags_at(self.profiles[self.name], self.eff_lane)
        if fl != self.flags and self.scoped:
            self.log("[variant] %s lane %d flags=%s" % (self.name, self.eff_lane, ",".join(sorted(fl)) or "-"))
        self.flags = fl

    def load(self, tune_dir=None, now=None):
        """바뀐 파일만 다시 읽는다. 반환: 이름·플래그·파라미터 중 하나라도 바뀌면 True. 고정 변형(from_name)은 차선 범위 프로필의
        플래그 차선 전환만 True."""
        if self.fixed:
            if not self.scoped:
                return False
            self._load_lane()
            eff = self._eff()
            if eff == self.eff_lane:
                return False
            before = self.flags; self.eff_lane = eff; self._resolve()
            return self.flags != before
        if tune_dir is not None:
            if tune_dir != self.tune_dir:
                self._algo_id = self._var_id = self._lane_id = self._eff_id = ("unset",)
            self.tune_dir = tune_dir
        self.loaded_at = now
        before = (self.name, self.flags, self.p)
        self._load_lane()                                                   # 차선 먼저: 아래 _apply_name 이 새 차선으로 플래그를 푼다
        eff = self._eff(); lane_changed = eff != self.eff_lane; self.eff_lane = eff
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
        if lane_changed and self.scoped:
            self._resolve()
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
        fl = flags_at(self.profiles[name], self.eff_lane)
        if name != self.name or fl != self.flags:
            self.log("[variant] algo %s%s flags=%s" % (name, " lane %d" % self.eff_lane if isinstance(self.profiles[name], LaneScope) else "",
                                                      ",".join(sorted(fl)) or "-"))
        self.name = name; self.flags = fl

    @classmethod
    def from_name(cls, name, tune_dir=TUNE_DIR, log=None):
        """이름으로 고정한 Variant (벤치·테스트·CLI). variants.json 은 생성 때 한 번 읽고, 이후 load() 는 파일을 보지 않는다.
        모르는 이름 → ValueError."""
        v = cls.__new__(cls)
        v.tune_dir = tune_dir; v.log = log or (lambda s: None)
        v.profiles, v.p = load_variants_file(os.path.join(tune_dir, VARIANTS_NAME), v.log)
        if name not in v.profiles: raise ValueError("모르는 변형 '%s' (아는 이름: %s)" % (name, ",".join(sorted(v.profiles))))
        v.name = name; v.lane = v.eff_lane = DEFAULT_LANE; v._lane_id = v._eff_id = ("unset",); v._file_eff = v._ext_lane = None
        if isinstance(v.profiles[name], LaneScope):
            v._load_lane(); v.eff_lane = v._eff()
        v.flags = flags_at(v.profiles[name], v.eff_lane)
        v.loaded_at = v.changed_at = None
        v._algo_id = v._var_id = ("fixed",); v._algo_raw = name; v._var_raw = None; v._warned_raw = None
        v.fixed = True
        return v

    def describe(self):
        """--show 출력 문자열."""
        lines = ["variant %s  (%s)" % (self.name, PROFILE_DESC.get(self.name, "variants.json 정의"))]
        fmt = lambda fl: " ".join(sorted(fl)) or "(없음 = base)"
        entry = self.profiles[self.name]
        if isinstance(entry, LaneScope):
            lines.append("flags: %s  (플래그 차선 %d = tune/lane_eff 또는 요청 차선, 요청 차선 tune/lane %d)" % (fmt(self.flags), self.eff_lane, self.lane))
            lines.append("flags[1차선]: %s" % fmt(entry.lane1))
            lines.append("flags[2차선]: %s" % fmt(entry.lane2))
        else:
            lines.append("flags: %s" % fmt(self.flags))
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
