# -*- coding: utf-8 -*-
"""
경로 노드:  /detections (YOLO 선 마스크)  →  /lane_trajectory (차로 중앙 y = a·x² + b·x + c)
                                          └→ /lane_detections (화면용 차로 띠, path_viewer)

좌표는 nodes/ipm.py (x 앞 m, y 왼쪽 m). c > 0 = 차로 중앙이 차보다 왼쪽.
트랙: 2차로, 차로 폭 0.85 m. 2차선 = 왼쪽 점선 + 오른쪽 실선, 1차선 = 왼쪽 실선 + 오른쪽 점선.
달릴 차선 = tune/lane 파일 (1 또는 2, 없으면 2). 주행 중에 바꾸면 (car.sh lane 1/2, tune_gui 버튼)
지금 차선 경로에 옆 이동을 0 → ±0.85 m 까지 lane_change_sec(tune.json, 없으면 2.0 초 = HEAD 값. 10/31 가제보: 2.0·3.0 완주, 4.0 은 왕복에서 코너와 겹쳐 정지, 사양 1.2) 동안 S자로 더하고,
다 옮기면 새 차선 규칙으로 바꾼다. 도중에 되돌리면 그 자리에서 다시 돌아온다.

base(HEAD 동작)  — 변형 플래그 공집합(tune/algo 부재 또는 'base'). 아래 단계만 실행되고 SolidCross 메서드는 호출되지 않는다.
 (아래 설명은 2차선 기준. 1차선은 좌우를 뒤집어서 같은 계산)
 1. 사진이 0.3 초 넘게 늦었거나 마스크가 없으면 실패 (6)
 2. 실선/점선 마스크를 칠한다. 확신도 0.30 미만, 차 진행 방향과 60° 넘게 누운 실선(횡단보도·정지선 앞 굵은 선)은 버린다
 3. find_path = _scan_rows → (_search_traj | fit_line ×2 → _fit_center → _apply_guards)
    - 사진 아래→위 2줄씩: 실선의 안쪽 끝 = 오른쪽 경계, 점선의 안쪽 끝 = 왼쪽 경계 (위치·연결성 규칙으로 고름)
    - 선마다 2차식. 점선 조각 사이 빈칸이 0.6 m 이하면 이어서 한 선으로 본다
    - 중심 = 선에서 수직 0.425 m (가로로는 0.425·√(1+기울기²)). 두 선의 폭이 맞으면 두 선의 가운데, 아니면 실선 기준
    - 경계가 하나도 없으면 "중심이 오른쪽 15 cm" 경로 (search_right). 2 초 넘게 계속되면 실패
    - 중심점들에 경로 곡선 (가까운 0.45 m 우선, 점이 부족하면 0.7 m 까지. 오차 15 cm 넘으면 실패)
 4. cross_hold: 횡단보도/정지선을 0.5 초 안에 봤는데 search_right 면 → 믿은 경로 (최대 1.5 초)
 5. accept_or_hold: 믿은 경로(마지막으로 받아들인 경로)와 비교
    - 경로 점이 모두 0.4 m 보다 먼 곳에만 있으면(차 앞이 안 보임) → 믿은 경로 (_far_only)
    - x = 0 / 0.4 / 0.8 m 에서 10 cm 넘게 튀면 → 0.5 초까지 믿은 경로 (_outlier_hold), 그 뒤엔 새 경로를 받아들임 (_accept)
 6. 실패하면 직전 경로를 1 초까지 유지, 그 뒤 valid=false (제어기가 차를 세움)
 base 의 연산 순서·상수는 HEAD(a41706e) 와 같다 (tests/test_base_identity.py 골든으로 확인).

변형 플래그 (nodes/variant.py: tune/algo 의 프로필 이름 → 플래그 집합. 상수는 Variant.p 한 곳)
 one_side_clip         (a) 선 하나(dashed_only/solid_only)로 만든 경로의 |c| 를 guard.one_side_max_c_m 로 제한 → reason ',c_clip'
 on_line_hold          (b) 점선만 보이는데 점선이 차 중심 ±guard.on_line_half_m 안(차가 점선을 밟고 있음)이고 믿은 경로가
                           점선 반대쪽(±guard.on_line_min_ref_c)을 가리키면 믿은 경로 유지 (최대 guard.on_line_hold_sec)
 step_accept           (c) outlier_accepted 때 x = 0/0.4/0.8 m 에서 변화량을 guard.accept_step_m 씩만 받아들임
 short_fit_guard       (d) 선 하나로 ext 적합한 경로가 x 범위 guard.ext_min_span_m 미만이거나 |b| > guard.ext_max_b 면 실패(short_fit). 기본 OFF
 solid_cross_recovery  (e)(f)(g) 실선 넘음 복구 = SolidCross 상태기계 (아래). 탐색 상한 cross.search_max_sec, 실패 유지 cross.hold_last_sec
    (e) 실선을 넘은 뒤(CROSSED 기억) 경계가 없으면 search_left/right 대신 실선 쪽으로 돌아가는 경로
        (믿은 경로가 그쪽이면 search_back_ref, 아니면 중심 = 실선 쪽 cross.search_back_m). 방향은 탐색 시작 때 스냅샷
    (f) 직전 실선이 차 밑·반대쪽(cross.enter_m 이상)까지 들어왔었으면 첫 줄 실선 후보 창을 +0.2 → cross.cand_max_cross_m 로 넓히고
        그 실선 하나로 돌아가는 경로의 |c| 상한은 cross.c_max_cross_m. 넘은 실선(y > cross.crossed_m) 바깥 점선(주차 구획선 오인) 무시,
        실선과 cross.overlap_m 안에서 겹치는 점선이면 실선 폐기
    (g) 실선을 밟고 있고(STRADDLE 연속 cross.confirm_frames 프레임, 점 수 ≥ cross.min_points) 실선 하나로 만든 새 경로가
        믿은 경로보다 차로 쪽을 더 가리키면 outlier_hold 없이 즉시 수용
 cross_continuity      (f) 의 전제 보정 (solid_cross_recovery 와 함께만 의미): 실선은 차 밑을 연속으로 지나야 반대쪽에 온다.
                       직전 관측이 차 밑·반대쪽(cross.enter_m 초과)이 아닌데 반대쪽(y0 > cross.near_m)에 처음 보이는 실선 = 옆 차로 바깥 실선
                       (1차선 급좌코너에서 안쪽 실선이 화면 밖으로 빠졌을 때) → 그 프레임은 실선 없이 점선만으로 다시 고르고(foreign_solid),
                       (f) 의 '넘은 실선 바깥 점선 무시' 규칙도 entered 일 때만 적용(docstring (f) 전제와 코드 일치). 기억(last)에도 남기지 않음
 lane_settle           Lane 상태기계 IDLE/CHANGING/SETTLING: ramp 끝에 규칙 전환 뒤 오프셋 전 경로 |c| < lane.settle_c_m 가 lane.settle_sec 지속
                       (또는 lane.max_sec) 해야 IDLE. 변경 중 새 요청은 되돌리기(CHANGING 중, 원래 차선)만 즉시, 그 밖은 IDLE 까지 보류.
                       OFF 면 HEAD 의 순수 시간 전환(Lane.update settle=False) 그대로
 change_slope          변경 중 publish 가 traj.c += off 에 더해 traj.b += d(off)/ds (ds = lane.assumed_speed_mps·dt) → S자 접선이 경로 기울기에 반영
 (corner_slowdown / speed_schedule / arc_comp / controller_pp / front_axle_ref 는 제어기 플래그. 이 노드는 보지 않음)
 차선 범위 프로필(variant.LaneScope, 예 v3): 플래그 차선의 단일 출처 = 이 노드의 Lane 실제 상태. lane.update() 다음에 flag_lane() 을 정해
                       var.set_lane() 으로 self.flags 를 풀고, 바뀔 때와 시작할 때 tune/lane_eff 에 원자적으로 쓴다(제어기 Variant 가 읽음).
                       flag_lane: CHANGING 이면 scope_hold 있으면 1·없으면 목표 차선 change[3], 그 밖(IDLE·SETTLING·보류 중)은 lane.rules.
                       → 즉시 처리되는 요청은 요청 프레임부터 새 차선 플래그(2→1 은 S자 시작부터 1차선, 1→2 는 S자 시작부터 2차선 —
                       경계 규칙 lane.rules 는 ramp 끝에 바뀜), scope_hold(v3)는 S자 전체(양방향) 1차선 알고리즘, lane_settle 보류 중에는
                       지금 차선 알고리즘 유지. lane.update 에 넘기는 lane_settle 판정은 직전 프레임 플래그로 하지만 차선 변경 플래그(LANE_FLAGS)는
                       프로필 수준이라 두 차선 같으므로 결과가 같다.

시계: self.now (기본 time.time, 파라미터 use_ros_clock=true 면 ROS 시계). 테스트는 생성자 인자 now 로 주입. 읽는 위치는 HEAD 와 같다
(on_detections 머리에서 1회, 탐색 분기 _search_traj 안에서 1회, cross_hold/accept_or_hold/fail 각각). base 는 find_path 안에서 시계를 읽지 않는다.
tune 파일: TuneFile 이 tune.json 을 mtime 기준으로 1회 읽어 Lane 에 넘긴다(lane_change_sec). 변형 파일은 Variant 가 읽는다.
상태: LaneState/SolidCross.state 는 관측용(테스트·진단) 파생값이고 제어 흐름은 SolidCross 의 질의 메서드(entered/cand_max/c_limit/search_dir/
accept_immediately)로만 분기한다. LOST 의 시간 상한은 base 와 공유하는 self.search_start 가 재고 cross.timeout() 으로 알린다.
가드 (b) 의 유지 시작 시각은 노드 필드 on_line_since (실선 넘음과 무관하므로 SolidCross 밖).
통계 로그 frames= 줄: HEAD 형식(frames … lane=) 뒤에 algo= 와 VARIANT_STATS_KEYS 를 순서대로 붙이고 last= 로 끝난다. 추가는 VARIANT_STATS_KEYS 끝에만.
"""
import copy
import json
import math
import os
import time
from dataclasses import dataclass, field
from enum import Enum

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory

from ipm import Ipm
from variant import DEFAULT_LANE_CHANGE_SEC, LANE_EFF_NAME, Variant

LANE_WIDTH_M = 0.85
HALF = LANE_WIDTH_M / 2.0
MIN_SCORE = 0.30           # 실선/점선 최소 확신도
TRANSVERSE_DEG = 60.0      # 이 각도보다 누운 실선은 버림
NEAR_RANGE_M = 0.45        # 경로 곡선에 평소 쓰는 거리
EXT_RANGE_M = 0.7          # 점이 부족할 때 넓혀 볼 최대 거리 (줄 훑기도 여기까지)
MAX_IMAGE_AGE_SEC = 0.3
HOLD_LAST_SEC = 1.0        # 실패 시 직전 경로 유지 시간 (base)
SEARCH_OFFSET_M = 0.15     # 경계가 없을 때 가정하는 중심 위치 (오른쪽)
SEARCH_MAX_SEC = 2.0       # 탐색 상한 (base)
SOLID_CAND_MAX_M = 0.2     # 첫 줄 실선 후보 창 상한 (base)
DASH_MAX_GAP_M = 0.6       # 조각 사이 빈칸이 이보다 길면 잇지 않음
DASH_SEG_GAP_M = 0.06      # 점 사이가 이보다 벌어지면 다른 조각
WIDTH_TOL_M = 0.25         # 두 선 사이 폭 허용 오차
LINE_MAX_ERR_M = 0.08      # 선 하나의 곡선 오차가 이보다 크면 그 선은 안 씀
MIN_KEEP = 8               # 중심점 수 하한
CHECK_X = np.array([0.0, 0.4, 0.8])   # 경로 튐을 비교하는 지점 (m)
JUMP_M = 0.10
OUTLIER_HOLD_SEC = 0.5
REF_MAX_AGE_SEC = 1.0      # 믿은 경로가 이보다 오래됐으면 비교하지 않음
FAR_ONLY_XMIN_M = 0.4
CROSS_CLASSES = ("crosswalk_marking", "stop_line")
CROSS_MIN_SCORE = 0.5
CROSS_RECENT_SEC = 0.5
CROSS_HOLD_MAX_SEC = 1.5
TUNE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tune")
LANE_FILE = TUNE_DIR + "/lane"            # 달릴 차선 1 또는 2 (car.sh lane, tune_gui 가 씀)
TUNE_FILE = TUNE_DIR + "/tune.json"       # lane_change_sec 만 읽음
LANE_CHANGE_SEC = DEFAULT_LANE_CHANGE_SEC  # tune.json 에 없을 때 2.0 (HEAD 값, variant.py 단일 출처. 10/31 벤치: 2.0·3.0 완주, 4.0 은 왕복 11.5 지점에서 코너와 겹쳐 정지)
DASH_MIN_FROM_SOLID_M = 0.4               # 점선 안쪽 끝은 실선 안쪽 끝보다 이만큼 이상 차로 쪽 (base·cross 공통)
BASE_STATS_KEYS = ["frames", "ok", "both", "one_side", "held", "search", "transverse_dropped",
                   "outlier_held", "outlier_accepted", "far_only_held", "cross_held", "dash_joined"]   # HEAD 12키
VARIANT_STATS_KEYS = ["c_clipped", "on_line_held", "outlier_stepped", "search_back", "solid_on_line_accept", "short_fit", "foreign_solid"]   # 플래그 통계. 추가는 끝에만
STATS_KEYS = BASE_STATS_KEYS + VARIANT_STATS_KEYS   # frames= 로그 줄 순서


def run_edges(mask, gap=3):
    """mask (줄, 열) bool → (시작, 끝) bool 표. 선 픽셀이 이어진 구간(gap 픽셀 이하 끊김은 같은 구간)의
    시작 = 왼쪽 gap+1 칸 안에 선 픽셀이 없는 선 픽셀, 끝 = 오른쪽 gap+1 칸 안에 없는 선 픽셀."""
    before = np.zeros_like(mask); after = np.zeros_like(mask)
    for k in range(1, gap + 2):
        before[:, k:] |= mask[:, :-k]; after[:, :-k] |= mask[:, k:]
    return mask & ~before, mask & ~after


class LineFit:
    """한 선의 점들에 맞춘 곡선 y = f(x) (점이 퍼진 길이가 0.3 m 미만이면 직선) 과 쓸 수 있는 x 구간들."""

    def __init__(self, P, ranges, joined):
        self.coef = np.polyfit(P[:, 0], P[:, 1], 2 if np.ptp(P[:, 0]) >= 0.3 else 1)
        self.dcoef = np.polyder(self.coef)
        self.err = float(np.sqrt(np.mean((np.polyval(self.coef, P[:, 0]) - P[:, 1]) ** 2)))
        self.ranges = ranges
        self.joined = joined

    def covers(self, x):
        return np.logical_or.reduce([(a - 1e-6 <= x) & (x <= b + 1e-6) for a, b in self.ranges])

    def y(self, x):
        return np.polyval(self.coef, x)

    def k(self, x):
        """수직 1 m 가 가로 몇 m 인가 = √(1+기울기²)"""
        return np.sqrt(1.0 + np.polyval(self.dcoef, x) ** 2)


def fit_line(pts):
    """경계 점들 → LineFit (점이 6 개 미만, 길이 6 cm 미만, 오차 8 cm 초과면 None)."""
    if len(pts) < 6:
        return None
    P = np.array(sorted(pts))
    if np.ptp(P[:, 0]) < 0.06:
        return None
    segs = np.split(np.arange(len(P)), np.flatnonzero(np.diff(P[:, 0]) > DASH_SEG_GAP_M) + 1)
    ranges = [(P[s[0], 0], P[s[-1], 0]) for s in segs]
    joined = len(segs) >= 2 and max(ranges[i + 1][0] - ranges[i][1] for i in range(len(ranges) - 1)) <= DASH_MAX_GAP_M
    if joined:
        ranges = [(ranges[0][0], ranges[-1][1])]
    f = LineFit(P, ranges, joined)
    return None if f.err > LINE_MAX_ERR_M else f


def line_at_origin(f):
    """선 적합의 x = 0 (차 위치) 에서의 옆 좌표."""
    return float(f.y(0.0))


def nearest(ys, target):
    return min(ys, key=lambda v: abs(v - target))


def read_file(path, mtime, cast):
    """파일이 바뀌었으면 (mtime, 값), 그대로이거나 못 읽으면 (mtime, None)."""
    try:
        mt = os.path.getmtime(path)
        return (mt, None) if mt == mtime else (mt, cast(open(path).read()))
    except (OSError, ValueError):
        return mtime, None


class TuneFile:
    """tune.json 을 mtime 기준으로 한 번만 읽는다. update() → 바뀐 주기에 dict, 아니면 None. data = 마지막으로 읽은 dict."""

    def __init__(self, path):
        self.path = path; self.mt = None; self.data = {}

    def update(self):
        self.mt, v = read_file(self.path, self.mt, json.loads)
        if isinstance(v, dict):
            self.data = v
            return v
        return None


class Lane:
    """달릴 차선과 차선 변경 중 옆 이동. 시계는 호출자가 now 로 넘긴다.

    상태(관측용 .state): IDLE / CHANGING(S자 ramp 중, change != None) / SETTLING(규칙 전환 뒤 정착 대기, settle_t0 != None).
    update(settle=False) = base·플래그 OFF: HEAD 와 같은 순수 시간 전환(ramp 끝 → rules, 새 want 는 즉시 반영, SETTLING 없음).
    update(settle=True)  = lane_settle: IDLE →(want≠rules) CHANGING → ramp 끝에 rules 전환 + SETTLING → observe() 가
      오프셋 전 경로 |c| < p.settle_c_m 를 p.settle_sec 연속 보거나 p.max_sec 가 지나면 IDLE. CHANGING 중 원래 차선(rules)으로의
      되돌리기만 즉시, 그 밖의 새 want 는 IDLE 까지 보류(want 는 파일 값 그대로이므로 IDLE 진입 다음 update 가 실행).
    slope_at(now, v) = change_slope 가 경로 b 에 더하는 d(off)/ds (ds = v·dt). 1차선(왼쪽)으로 갈 때 off 증가 → 양(왼쪽)."""
    IDLE, CHANGING, SETTLING = "IDLE", "CHANGING", "SETTLING"

    def __init__(self, log, now, tune=None, lane_file=None):
        self.log = log
        self.rules = 2                            # 지금 경계를 고르는 규칙 (1 또는 2)
        self.want = 2                             # 요청된 차선
        self.change = None                        # 변경 중: (시작 시각, 시작 이동, 목표 이동, 목표 차선)
        self.lane_file = LANE_FILE if lane_file is None else lane_file
        self.lane_mt = None
        self.sec = LANE_CHANGE_SEC
        self.p = None                             # variant.LaneChange (settle=True 일 때 update 가 채움)
        self.settle_t0 = None                     # SETTLING 진입 시각 (None = SETTLING 아님)
        self.settle_ok_since = None               # |c| < settle_c_m 가 이어진 시작 시각
        self.held_want = None                     # 보류 로그 1회용 (보류 중인 want)
        self.update(now, tune)
        self.rules = self.want; self.change = None   # 시작할 때는 옮기지 않고 바로 그 차선

    @property
    def state(self):
        return self.CHANGING if self.change is not None else self.SETTLING if self.settle_t0 is not None else self.IDLE

    @property
    def pending(self):
        """보류된 요청 차선 (IDLE 이 아니고 want 가 지금 규칙·목표와 다를 때), 없으면 None."""
        goal = self.change[3] if self.change else self.rules
        return self.want if self.state != self.IDLE and self.want != goal else None

    def offset_at(self, now):
        if self.change is None:
            return 0.0
        t0, o0, o1, _ = self.change
        s = min(max((now - t0) / max(self.sec, 1e-3), 0.0), 1.0)
        return o0 + (o1 - o0) * s * s * (3 - 2 * s)

    def slope_at(self, now, speed):
        """d(off)/ds: smoothstep 미분 6s(1-s)/sec × (o1-o0) ÷ 속도. 변경 중이 아니면 0."""
        if self.change is None or speed <= 0:
            return 0.0
        t0, o0, o1, _ = self.change
        s = min(max((now - t0) / max(self.sec, 1e-3), 0.0), 1.0)
        return (o1 - o0) * 6.0 * s * (1.0 - s) / max(self.sec, 1e-3) / speed

    def _start(self, now, target):
        o1 = 0.0 if target == self.rules else (LANE_WIDTH_M if target == 1 else -LANE_WIDTH_M)
        self.change = (now, self.offset_at(now), o1, target)
        self.log("lane change -> %d (%.1fs)" % (target, self.sec))

    def update(self, now, tune=None, settle=False, p=None):
        """lane 파일과 (바뀐 주기의) tune dict 를 반영하고 지금 경로에 더할 옆 이동(m, + 왼쪽)을 돌려준다."""
        self.lane_mt, v = read_file(self.lane_file, self.lane_mt, lambda s: int(s.strip()))
        if v in (1, 2):
            self.want = v
        if isinstance(tune, dict):
            try:
                self.sec = float(tune.get("lane_change_sec", LANE_CHANGE_SEC))
            except (TypeError, ValueError):
                pass
        if not settle:                            # base / lane_settle OFF: HEAD 와 같은 순수 시간 전환
            goal = self.change[3] if self.change else self.rules
            if self.want != goal:                     # 새 요청 또는 변경 도중 되돌리기
                o1 = 0.0 if self.want == self.rules else (LANE_WIDTH_M if self.want == 1 else -LANE_WIDTH_M)
                self.change = (now, self.offset_at(now), o1, self.want)
                self.log("lane change -> %d (%.1fs)" % (self.want, self.sec))
            if self.change and now - self.change[0] >= self.sec:
                if self.change[3] != self.rules:      # 다 옮겼으면 새 차선 규칙으로
                    self.rules = self.change[3]
                    self.log("lane %d" % self.rules)
                self.change = None
            return self.offset_at(now)
        if p is not None:
            self.p = p
        if self.settle_t0 is not None and self.p is not None and now - self.settle_t0 >= self.p.max_sec:
            self._settled(now, "timeout %.1fs" % self.p.max_sec)
        st = self.state
        goal = self.change[3] if self.change else self.rules
        if self.want != goal:
            if st == self.IDLE or (st == self.CHANGING and self.want == self.rules):   # 새 요청 / 되돌리기
                self._start(now, self.want); self.held_want = None
            elif self.held_want != self.want:                                            # 그 밖은 IDLE 까지 보류
                self.held_want = self.want
                self.log("lane change -> %d held (%s)" % (self.want, st.lower()))
        if self.change and now - self.change[0] >= self.sec:                             # ramp 끝 → rules 전환 + SETTLING
            if self.change[3] != self.rules:
                self.rules = self.change[3]
                self.log("lane %d" % self.rules)
            self.change = None
            self.settle_t0 = now; self.settle_ok_since = None
            self.log("lane settling")
        return self.offset_at(now)

    def observe(self, now, c):
        """SETTLING 중 오프셋 전 경로 c 를 본다 (lane_settle 전용, 경로가 있는 프레임만)."""
        if self.settle_t0 is None or self.p is None:
            return
        if abs(c) < self.p.settle_c_m:
            self.settle_ok_since = now if self.settle_ok_since is None else self.settle_ok_since
            if now - self.settle_ok_since >= self.p.settle_sec:
                self._settled(now, "|c| < %.2f for %.1fs" % (self.p.settle_c_m, self.p.settle_sec))
        else:
            self.settle_ok_since = None

    def _settled(self, now, why):
        self.settle_t0 = None; self.settle_ok_since = None
        self.log("lane settled (%s)" % why)


class LaneState(Enum):
    """실선 기준 차 위치 상태 (mirror 좌표: 실선이 차로 바깥쪽이면 y0 < 0, 차 반대쪽이면 y0 > 0).
    SolidCross.state 가 관측(last·lost_since·failed)에서 파생해 돌려주는 관측용 값. 노드 분기는 SolidCross 질의 메서드로 한다."""
    NORMAL = "normal"       # 두 선 또는 선 하나. 실선이 제자리(y0 < -near_m)
    STRADDLE = "straddle"   # 실선이 x=0 ±near_m 안 (차가 실선을 밟고 있음)
    CROSSED = "crossed"     # 실선이 차 반대쪽 (y0 > near_m)
    LOST = "lost"           # 경계 없음 (탐색 중)
    FAILED = "failed"       # 탐색 상한 초과 → fail (hold_last_sec 뒤 valid=false)


class SolidCross:
    """실선 넘음 복구 상태기계 (플래그 solid_cross_recovery). base 에서는 생성만 되고 어떤 메서드도 호출되지 않는다.

    전이표 (입력 = 프레임마다 expire(now) 뒤 note_solid(now, y0) 또는 lose(now); 시간 상한은 상태마다 1개)
    | 상태     | 입력                              | 다음      | 효과 |
    |----------|-----------------------------------|-----------|------|
    | NORMAL   | 실선 적합 y0 ≤ -near_m             | NORMAL    | confirm 0 |
    | NORMAL   | 실선 적합 |y0| ≤ near_m             | STRADDLE  | confirm += 1 |
    | NORMAL   | 실선 적합 y0 > near_m               | CROSSED   | confirm 0 |
    | *        | 선은 있는데 실선 적합 없음           | 유지      | confirm 0 (기억 last 유지) |
    | STRADDLE | |y0| ≤ near_m 연속 ≥ confirm_frames  | STRADDLE  | accept_immediately 가 참이 될 수 있음 (g) |
    | *        | 경계 없음 lose(now)                 | LOST      | search_dir 스냅샷: 기억 y0 > 0 → +1(실선 쪽), 아니면 0(base 탐색) |
    | LOST     | 경계 다시 보임                      | NORMAL/STRADDLE/CROSSED (관측대로) | lost_since 해제 |
    | LOST     | now - lost_since > search_max_sec   | FAILED    | fail(hold_last_sec) |
    | *        | now - last[0] > max_age_sec (expire) | 기억 삭제 | prev None → (e)(f) 비활성. max_age_sec ≥ search_max_sec 는 variant 로더가 보장 |
    (f) 는 prev(이번 프레임 시작 시 유효한 직전 관측) > p.enter_m 일 때, (e) 는 LOST 진입 때 prev > 0 일 때.
    LOST 의 시간 상한(search_max_sec)은 노드가 base 와 같은 self.search_start 로 재고 timeout() 으로 알린다(lost_since 는 state 표시용)."""

    def __init__(self, p):
        self.p = p                     # variant.Cross
        self.last = None               # (시각, y0) 마지막 실선 적합 (mirror 좌표)
        self.prev = None               # 이번 프레임 시작 시 유효한 직전 관측 y0 (expire 가 채움)
        self.lost_since = None
        self._search_dir = 0
        self.confirm = 0               # STRADDLE 연속 프레임 수
        self.failed = False

    @property
    def state(self):
        if self.lost_since is not None:
            return LaneState.FAILED if self.failed else LaneState.LOST
        if self.last is None or self.last[1] < -self.p.near_m:
            return LaneState.NORMAL
        return LaneState.STRADDLE if abs(self.last[1]) <= self.p.near_m else LaneState.CROSSED

    def expire(self, now):
        """프레임 시작: 오래된 기억을 지우고 직전 관측 y0(또는 None)을 prev 로 고정해 돌려준다."""
        if self.last is not None and now - self.last[0] > self.p.max_age_sec:
            self.last = None
        self.prev = None if self.last is None else self.last[1]
        return self.prev

    def note_solid(self, now, y0):
        """경계를 봤다. y0 = 실선 적합의 x=0 위치 (mirror), 실선 적합이 없으면 None."""
        self.lost_since = None; self.failed = False
        if y0 is None:
            self.confirm = 0
            return
        self.last = (now, float(y0))
        self.confirm = self.confirm + 1 if abs(y0) <= self.p.near_m else 0

    def lose(self, now):
        """경계 없음 → LOST 진입 (처음 한 번 방향 스냅샷)."""
        if self.lost_since is None:
            self.lost_since = now
            self._search_dir = 1 if (self.prev is not None and self.prev > 0) else 0

    def timeout(self):
        self.failed = True

    def search_dir(self):
        """LOST 진입 때 고정한 방향: 1 = 실선 쪽으로 돌아감(search_back), 0 = base 탐색."""
        return self._search_dir

    def entered(self):
        return self.prev is not None and self.prev > self.p.enter_m

    def foreign(self, y0):
        """(f) 전제: 실선은 차 밑을 연속으로 지나야 반대쪽(y0 > near_m)에 온다. 직전 관측이 차 밑·반대쪽(entered)이 아닌데
        반대쪽에 처음 보이는 실선은 넘은 안쪽 실선이 아니라 옆 차로 바깥 실선(급코너에서 안쪽 실선이 화면 밖으로 빠졌을 때) → 경계로 쓰지 않음."""
        return y0 is not None and y0 > self.p.near_m and not self.entered()

    def cand_max(self, base):
        """(f) 첫 줄 실선 후보 창 상한."""
        return self.p.cand_max_cross_m if self.entered() else base

    def c_limit(self, base, solid_y0):
        """(f) 넘은 실선(차 반대쪽) 하나로 돌아가는 경로의 |c| 상한."""
        return max(base, self.p.c_max_cross_m) if (solid_y0 > 0 and self.entered()) else base

    def accept_immediately(self, traj, ref):
        """(g) 실선을 밟은 채 실선 하나로 만든 새 경로가 믿은 경로보다 차로 쪽을 더 가리키는가 (연속 확인·점 수 조건 포함)."""
        if self.confirm < self.p.confirm_frames or traj.num_points < self.p.min_points:
            return False
        return traj.c * ref.c <= 0 or abs(traj.c) > abs(ref.c)


@dataclass
class PathResult:
    """find_path 한 프레임의 결과. reason() 이 로그 문자열을 한 곳에서 조립한다 (기존 형식 유지)."""
    traj: object = None
    band: object = None
    kind: str = "fail"                      # solid+dashed | solid_only | dashed_only | search | fail
    used: str = ""                          # near | ext
    flags: set = field(default_factory=set)  # c_clip, on_line, solid_on_line
    note: str = ""                          # search/fail 의 사유 본문
    wraps: list = field(default_factory=list)   # [(fmt, args)] 바깥 단계(hold/accept) 접두, 안쪽부터

    FLAG_ORDER = ("c_clip", "on_line", "solid_on_line")

    @property
    def ok(self):
        return self.traj is not None

    def wrap(self, fmt, *args):
        """fmt 의 마지막 %s 에 안쪽 reason 이 들어간다."""
        self.wraps.append((fmt, args))
        return self

    def reason(self):
        if self.kind in ("search", "fail"):
            s = self.note
        else:
            s = "%s,%s%s" % (self.kind, self.used, "".join("," + f for f in self.FLAG_ORDER if f in self.flags))
        for fmt, args in self.wraps:
            s = fmt % (tuple(args) + (s,))
        return s

    def stat_key(self):
        """on_detections 통계 분류 (HEAD 규칙 그대로): 접두(hold/accept)가 붙은 결과는 종류와 무관하게 one_side.
        → 'search' | 'both' | 'one_side'."""
        if self.wraps:
            return "one_side"
        return "search" if self.kind == "search" else ("both" if self.kind == "solid+dashed" else "one_side")


def fail_result(note):
    return PathResult(kind="fail", note=note)


def search_traj(header, c):
    t = LaneTrajectory(); t.header = header; t.valid = True
    t.a = t.b = 0.0; t.c = float(c)
    t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
    return t


class LanePathNode(Node):
    def __init__(self, now=None, variant=None):
        super().__init__("line_path_node")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(DetectionArray, "/detections", self.on_detections, qos)
        self.traj_pub = self.create_publisher(LaneTrajectory, "/lane_trajectory", qos)
        self.band_pub = self.create_publisher(DetectionArray, "/lane_detections", qos)
        self.now = now if now is not None else time.time
        if now is None and self.declare_parameter("use_ros_clock", False).value:
            self.now = lambda: self.get_clock().now().nanoseconds * 1e-9
        self.init_state(self.get_logger().info, variant)
        self.create_timer(2.0, self.log_stats)

    def init_state(self, log, variant=None):
        """ROS 와 무관한 상태 (테스트는 이 부분만 쓴다)."""
        self.ipm = Ipm(); self.rows = None; self.row_idx = None
        self.tune = TuneFile(TUNE_FILE)
        self.lane = Lane(log, self.now(), self.tune.update()); self.off = 0.0; self.slope = 0.0
        self.var = variant if variant is not None else Variant(TUNE_DIR, log=log, now=self.now())
        self.lane_eff_path = os.path.join(os.path.dirname(self.lane.lane_file), LANE_EFF_NAME)   # 요청 차선 파일 옆 tune/lane_eff (제어기가 읽음)
        self.lane_eff_written = None                                            # 마지막으로 tune/lane_eff 에 쓴 플래그 차선
        self.sync_flag_lane()                                                   # 시작: Lane 초기 차선으로 플래그 + tune/lane_eff
        self.cross = SolidCross(self.var.p.cross)                               # base 는 생성만
        self.last_traj = None; self.last_band = None; self.last_ok_time = 0.0   # 직전 정상 경로 (실패 시 유지용)
        self.ref = None; self.ref_band = None; self.ref_time = 0.0              # 믿은 경로
        self.outlier_since = None; self.search_start = None
        self.on_line_since = None                                               # 가드 (b) 유지 시작 시각
        self.cross_seen = 0.0; self.cross_hold_since = None
        self.stats = dict.fromkeys(STATS_KEYS, 0)
        self.last_reason = ""

    @property
    def flags(self):
        return self.var.flags

    def flag_lane(self):
        """플래그를 고르는 차선 (Lane 실제 상태): CHANGING 이면 scope_hold → 1, 아니면 목표 차선. 그 밖(IDLE·SETTLING·보류)은 lane.rules."""
        if self.lane.change is not None:
            return 1 if "scope_hold" in self.flags else self.lane.change[3]
        return self.lane.rules

    def sync_flag_lane(self):
        """플래그 차선을 Variant 에 주고, 바뀌었으면 tune/lane_eff 에 원자적으로 쓴다(tmp + os.replace, 실패하면 다음 프레임에 다시)."""
        fl = self.flag_lane()
        self.var.set_lane(fl)
        if fl != self.lane_eff_written:
            try:
                with open(self.lane_eff_path + ".tmp", "w") as f:
                    f.write("%d\n" % fl)
                os.replace(self.lane_eff_path + ".tmp", self.lane_eff_path)
                self.lane_eff_written = fl
            except OSError:
                pass

    def setup(self, h, w):
        """사진 크기가 바뀌면: IPM 표, 훑을 줄 목록 (아래→위 2줄씩, 앞 거리가 줄어들거나 EXT_RANGE_M 를 넘으면 끝, x < -0.3 은 건너뜀)."""
        if self.ipm.shape == (h, w):
            return
        self.ipm.setup(h, w)
        self.rows = []; last = -1e9
        for row in range(h - 1, -1, -2):
            xf0 = float(self.ipm.xf[row])
            if xf0 < last - 1e-6 or xf0 > EXT_RANGE_M:
                break
            last = xf0
            if xf0 >= -0.3:
                self.rows.append((row, xf0))
        self.row_idx = np.array([r for r, _ in self.rows], int)

    def on_detections(self, msg):
        now = self.now()
        if any(d.class_name in CROSS_CLASSES and d.score >= CROSS_MIN_SCORE for d in msg.detections):
            self.cross_seen = now
        self.stats["frames"] += 1
        self.var.load(now=now)
        self.off = self.lane.update(now, self.tune.update(), "lane_settle" in self.flags, self.var.p.lane)
        self.sync_flag_lane()                                                  # 차선 범위 프로필: Lane 실제 상태로 플래그 차선 (tune/lane_eff)
        self.slope = self.lane.slope_at(now, self.var.p.lane.assumed_speed_mps) if "change_slope" in self.flags else 0.0
        if self.cross.p is not self.var.p.cross:                               # variants.json 이 바뀌면 상수 객체 교체
            self.cross.p = self.var.p.cross
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        shape = next(((d.mask.height, d.mask.width) for d in msg.detections if d.mask.height > 0), None)
        if age > MAX_IMAGE_AGE_SEC or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > MAX_IMAGE_AGE_SEC else "no_mask"
            return self.fail(msg.header)
        h, w = shape
        self.setup(h, w)
        md = np.zeros((h, w), np.uint8)    # 점선
        ms = np.zeros((h, w), np.uint8)    # 실선
        for d in msg.detections:
            if d.class_name not in ("solid_line", "dashed_line") or d.score < MIN_SCORE or not d.mask.data:
                continue
            poly = np.array([[p.x, p.y] for p in d.mask.data], np.float32)
            if d.class_name == "solid_line" and self.is_transverse(poly):
                self.stats["transverse_dropped"] += 1
                continue
            cv2.fillPoly(ms if d.class_name == "solid_line" else md,
                         [np.clip(np.round(poly), [0, 0], [w - 1, h - 1]).astype(np.int32)], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            return self.fail(msg.header)

        res = self.find_path(md, ms, msg.header, self.lane.rules == 1)
        res = self.cross_hold(res, msg.header)
        res = self.accept_or_hold(res, msg.header)
        reason = res.reason()
        self.last_reason = reason
        if res.traj is None:
            return self.fail(msg.header)
        k = res.stat_key()
        if k != "search":
            self.stats[k] += 1
        self.stats["ok"] += 1
        self.last_traj, self.last_band, self.last_ok_time = res.traj, res.band, self.now()
        if "lane_settle" in self.flags:
            self.lane.observe(now, res.traj.c)
        self.publish(res.traj, res.band, shape)

    def publish(self, traj, band, shape):
        """차선 변경 중이면 지금 옆 이동(과 change_slope 의 기울기)을 더해서 내보낸다 (계산·유지 판단은 모두 이동 없는 차선 기준)."""
        if self.off or self.slope:
            traj = copy.deepcopy(traj); traj.c += self.off; traj.b += self.slope
            band = self.band_of(traj) if band is not None else None
        self.traj_pub.publish(traj)
        self.publish_band(traj.header, band, shape)

    def band_of(self, t):
        """경로 ± 0.425 m 띠의 사진 좌표 (화면용)."""
        bx = np.linspace(t.x_min_m, t.x_max_m, 12); by = np.polyval([t.a, t.b, t.c], bx)
        return [(float(a), float(b)) for a, b in self.ipm.to_px(np.r_[bx, bx[::-1]], np.r_[by + HALF, by[::-1] - HALF])]

    def is_transverse(self, poly):
        """실선 테두리 점들이 가장 길게 퍼진 방향(주성분)이 앞쪽과 TRANSVERSE_DEG 넘게 벌어지면 가로선."""
        xf, yl = self.ipm.to_m(poly[:, 0], poly[:, 1])
        ok = (xf > -0.5) & (xf < 4.0)                    # 지평선 근처(변환이 이상해지는 점)는 제외
        if ok.sum() < 4:
            return False
        P = np.c_[xf[ok], yl[ok]]
        dx, dy = np.linalg.svd(P - P.mean(axis=0), full_matrices=False)[2][0]
        return math.degrees(math.atan2(abs(dy), abs(dx))) > TRANSVERSE_DEG

    # ---------------------------------------------------------------- find_path
    def find_path(self, md, ms, header, mirror=False):
        """mirror (1차선): 실선은 구간의 오른쪽 끝, 점선은 왼쪽 끝을 쓰고 옆 좌표 부호를 바꿔 2차선과 같은 계산 → 결과를 다시 뒤집음.
        → PathResult."""
        cross = self.cross if "solid_cross_recovery" in self.flags else None   # base: None (SolidCross 메서드 호출 없음)
        now = None if cross is None else self.now()                             # base 는 여기서 시계를 읽지 않음 (HEAD: 탐색 분기에서만)
        cand_max = SOLID_CAND_MAX_M
        if cross is not None:
            cross.expire(now)
            cand_max = cross.cand_max(SOLID_CAND_MAX_M)
        R, D = self._scan_rows(md, ms, mirror, cand_max, cross, cont=cross is not None and "cross_continuity" in self.flags)
        if not R and not D:
            return self._search_traj(header, mirror, cross)
        fs, fd = fit_line(R), fit_line(D)                                        # (fit_line 은 순수 함수: base 순서 영향 없음)
        if cross is not None and "cross_continuity" in self.flags and fs is not None and cross.foreign(line_at_origin(fs)):
            self.stats["foreign_solid"] += 1
            R, D = self._scan_rows(md, ms, mirror, cand_max, cross, drop_solid=True, cont=True)   # 옆 차로 실선 → 실선 없이 점선만 다시 고름
            fs, fd = None, fit_line(D)
            if not D:
                return self._search_traj(header, mirror, cross)                 # 경계 없음과 같음 (탐색 시계 유지)
        self.search_start = None
        if cross is not None:
            cross.note_solid(now, None if fs is None else line_at_origin(fs))
        fit, note = self._fit_center(fs, fd, R, D, mirror)
        if fit is None:
            return fail_result(note)
        c, sel, err, kind, used = fit
        flags = set()
        if self.flags:                                                           # base 는 여기로 안 들어옴
            c, flags, note = self._apply_guards(c, sel, kind, used, fs, fd, cross)
            if note is not None:
                return fail_result(note)

        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        bx = np.linspace(t.x_min_m, t.x_max_m, 12); by = np.polyval(c, bx)          # 화면용: 경로 ± 0.425 m 띠
        band = self.ipm.to_px(np.r_[bx, bx[::-1]], np.r_[by + HALF, by[::-1] - HALF])
        self.stats["dash_joined"] += int(fd is not None and fd.joined)
        return PathResult(t, [(float(a), float(b)) for a, b in band], kind, used, flags)

    def _scan_rows(self, md, ms, mirror, cand_max, cross, drop_solid=False, cont=False):
        """줄마다 실선 안쪽 끝(R)·점선 안쪽 끝(D) 하나씩 고른다 (mirror 좌표). cross(SolidCross, base 는 None): (f) 넘은 실선·바깥 점선 규칙."""
        yl = self.ipm.yl
        n = len(self.rows)
        sr, sc = np.nonzero(run_edges(ms[self.row_idx] > 0)[int(mirror)])       # 실선의 안쪽 끝 (줄 번호, 열)
        dr, dc = np.nonzero(run_edges(md[self.row_idx] > 0)[1 - int(mirror)])   # 점선의 안쪽 끝
        sk, dk = np.searchsorted(sr, np.arange(n + 1)), np.searchsorted(dr, np.arange(n + 1))   # 줄마다 시작 위치
        R, D = [], []                            # 실선 안쪽 끝 (x, y), 점선 안쪽 끝 (x, y)
        prev_r = prev_d = None
        for i, (row, xf0) in enumerate(self.rows):
            yr = yd = None
            starts = sc[sk[i]:sk[i + 1]]
            if starts.size and not drop_solid:                      # 오른쪽 경계 = 실선 구간의 왼쪽 끝
                ys = -yl[row, starts] if mirror else yl[row, starts]
                if prev_r is None:               # 처음: 차 오른쪽(-1.2 ~ cand_max) 중 차로 폭 절반에 가장 가까운 것
                    cand = [v for v in ys if -1.2 < v < cand_max]
                    yr = nearest(cand, -HALF) if cand else None
                else:                            # 이어서: 아랫줄과 15 cm 안쪽으로 이어지는 것만
                    v = nearest(ys, prev_r); yr = v if abs(v - prev_r) < 0.15 else None
            ends = dc[dk[i]:dk[i + 1]]
            if ends.size:                        # 왼쪽 경계 = 점선 구간의 오른쪽 끝
                ys = -yl[row, ends] if mirror else yl[row, ends]
                if cross is not None and yr is not None and yr > cross.p.crossed_m and (not cont or cross.entered()):   # (f) 넘은 실선: 겹치는 점선이면 실선 폐기(점선 오인), 실선 바깥 점선은 주차 구획선으로 무시
                    if any(abs(v - yr) < cross.p.overlap_m for v in ys):
                        yr = None
                    else:
                        ys = [v for v in ys if v > yr + DASH_MIN_FROM_SOLID_M]
                else:
                    if yr is not None and any(v < yr - 0.2 for v in ys):
                        yr = None                    # 점선보다 왼쪽의 실선 = 1차선 바깥 실선
                    if yr is not None:
                        ys = [v for v in ys if v > yr + DASH_MIN_FROM_SOLID_M]   # 점선은 실선보다 0.4 m 이상 왼쪽
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = nearest(cand, HALF) if cand else None
                elif len(ys):
                    v = nearest(ys, prev_d); yd = v if abs(v - prev_d) < 0.2 else None
            if yr is not None: prev_r = yr; R.append((xf0, float(yr)))
            if yd is not None: prev_d = yd; D.append((xf0, float(yd)))
        return R, D

    def _search_traj(self, header, mirror, cross):
        """경계가 하나도 없음: 탐색 경로 (num_points 0) 또는 search_timeout 실패. 시계는 여기서 읽는다 (HEAD 와 같은 위치)."""
        now = self.now()
        if self.search_start is None:
            self.search_start = now
            if cross is not None:
                cross.lose(now)
        search_max = cross.p.search_max_sec if cross is not None else SEARCH_MAX_SEC
        if now - self.search_start > search_max:
            if cross is not None:
                cross.timeout()
            return fail_result("search_timeout(%.1fs)" % (now - self.search_start))
        self.stats["search"] += 1
        el = now - self.search_start
        if cross is not None and cross.search_dir() > 0:       # (e) 실선을 넘은 상태에서 경계가 없어짐 → 실선 쪽(차로 쪽)으로 되돌아감
            self.stats["search_back"] += 1
            back = -1.0 if mirror else 1.0                     # 실제 좌표에서 실선 쪽 부호 (1차선 오른쪽, 2차선 왼쪽)
            if self.ref_ok(now) and self.ref.c * back > 0:     # 믿은 경로가 이미 그쪽을 가리키면 그 경로
                t = copy.deepcopy(self.ref); t.header = header
                return PathResult(t, self.ref_band, "search", note="search_back_ref(%.1fs)" % el)
            return PathResult(search_traj(header, back * cross.p.search_back_m), None, "search", note="search_back(%.1fs)" % el)
        c = float(SEARCH_OFFSET_M) if mirror else -float(SEARCH_OFFSET_M)
        return PathResult(search_traj(header, c), None, "search", note="search_%s(%.1fs)" % ("left" if mirror else "right", el))

    def _fit_center(self, fs, fd, R, D, mirror):
        """두 선 적합 → 중심점 → 경로 2차식. → ((coef, sel, err, kind, used), None) 또는 (None, 실패 사유)."""
        if fs is None and fd is None:
            return None, "too_few_points(s%d,d%d)" % (len(R), len(D))
        xs_all = [p[0] for p in (R if fs else [])] + [p[0] for p in (D if fd else [])]
        x = np.arange(min(xs_all), max(xs_all) + 1e-9, 0.02)
        no = np.zeros(len(x), bool)
        s_ok = fs.covers(x) if fs else no
        d_ok = fd.covers(x) if fd else no
        y_s = fs.y(x) + HALF * fs.k(x) if fs else 0.0            # 실선 기준 중심
        y_d = fd.y(x) - HALF * fd.k(x) if fd else 0.0            # 점선 기준 중심
        both = s_ok & d_ok
        if both.any():                                           # 두 선 다 있고 폭이 맞으면 두 선의 가운데
            both &= np.abs((fd.y(x) - fs.y(x)) - LANE_WIDTH_M * (0.5 * (fs.k(x) + fd.k(x)))) < WIDTH_TOL_M
        use_s = s_ok & ~both
        use_d = d_ok & ~s_ok
        yc = np.where(both, 0.5 * (fs.y(x) + fd.y(x)) if both.any() else 0.0, np.where(use_s, y_s, y_d))
        keep = both | use_s | use_d
        if keep.sum() < MIN_KEEP:
            return None, "too_few_points(%d)" % keep.sum()

        P = np.c_[x[keep], yc[keep]]
        sel = P[P[:, 0] <= NEAR_RANGE_M]
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:           # 가까운 점이 부족하면 더 멀리까지
            sel = P; used = "ext"
        if len(sel) < MIN_KEEP or np.ptp(sel[:, 0]) < 0.08:
            return None, "short_span"
        c = np.polyfit(sel[:, 0], sel[:, 1], 2 if np.ptp(sel[:, 0]) >= 0.3 else 1)
        if len(c) == 2: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, "bad_fit(%.2f)" % err
        if mirror:
            c = -c
        n_s, n_d = use_s.any(), use_d.any()
        kind = "solid+dashed" if (both.any() or (n_s and n_d)) else ("solid_only" if n_s else "dashed_only")
        return (c, sel, err, kind, used), None

    def _apply_guards(self, c, sel, kind, used, fs, fd, cross):
        """선 하나짜리 경로 가드 (d)(a)(b)(g). → (coef, 표식 집합, 실패 사유 또는 None). 플래그 공집합이면 호출되지 않는다."""
        flags = set(); g = self.var.p.guard
        if kind == "solid+dashed":
            return c, flags, None
        if "short_fit_guard" in self.flags:
            span = float(np.ptp(sel[:, 0]))
            if used == "ext" and (span < g.ext_min_span_m or abs(c[1]) > g.ext_max_b):
                self.stats["short_fit"] += 1                 # (d) 짧은 범위·급한 기울기의 선 하나짜리 ext 적합은 믿지 않음 → 직전 경로 유지
                return c, flags, "short_fit(%s,n%d,span%.2f,b%.2f)" % (kind, len(sel), span, c[1])
        if "one_side_clip" in self.flags:
            lim = g.one_side_max_c_m
            if cross is not None and kind == "solid_only" and fs is not None:
                lim = cross.c_limit(lim, line_at_origin(fs))     # (f) 넘은 실선 하나로 돌아가는 경로는 상한을 넓힘
            if abs(c[2]) > lim:                                  # (a) 선 하나짜리 경로의 |c| 상한
                c = c.copy(); c[2] = float(np.clip(c[2], -lim, lim)); flags.add("c_clip"); self.stats["c_clipped"] += 1
        if "on_line_hold" in self.flags and kind == "dashed_only" and fd is not None and abs(line_at_origin(fd)) <= g.on_line_half_m:
            flags.add("on_line")                                 # (b) 차가 점선을 밟고 있음 → accept_or_hold 가 믿은 경로 유지
        if cross is not None and kind == "solid_only" and fs is not None and abs(line_at_origin(fs)) <= cross.p.near_m:
            flags.add("solid_on_line")                           # (g) 차가 실선을 밟고 있음 → accept_or_hold 가 즉시 수용
        return c, flags, None

    # ---------------------------------------------------------------- hold / accept
    def ref_ok(self, now):
        return self.ref is not None and now - self.ref_time <= REF_MAX_AGE_SEC

    def hold_ref(self, res, header, key, fmt, *args):
        """결과를 믿은 경로로 바꾸고 reason 에 접두를 붙인다."""
        self.stats[key] += 1
        t = copy.deepcopy(self.ref); t.header = header
        res.traj, res.band = t, self.ref_band
        return res.wrap(fmt, *args)

    def cross_hold(self, res, header):
        """횡단보도/정지선 근처의 search_right → 믿은 경로."""
        now = self.now()
        if res.traj is None or res.kind != "search":
            self.cross_hold_since = None
            return res
        if now - self.cross_seen <= CROSS_RECENT_SEC and self.ref_ok(now):
            if self.cross_hold_since is None:
                self.cross_hold_since = now
            if now - self.cross_hold_since <= CROSS_HOLD_MAX_SEC:
                return self.hold_ref(res, header, "cross_held", "cross_hold(%.1fs,%s)", now - self.cross_hold_since)
        return res

    def accept_or_hold(self, res, header):
        """먼 곳 점만으로 만든 경로, 갑자기 튄 경로 → 믿은 경로. 받아들인 경로는 믿은 경로가 된다.
        술어 순서: _far_only → _on_line_hold(플래그) → _solid_on_line_accept(플래그) → _outlier_hold → _step_accept(플래그) → _accept."""
        if res.traj is None:
            return res
        now = self.now()
        if self.ref_ok(now):
            if self._far_only(res, header):
                return res
            if "on_line_hold" in self.flags and self._on_line_hold(res, header, now):
                return res
            new = np.polyval([res.traj.a, res.traj.b, res.traj.c], CHECK_X); old = np.polyval([self.ref.a, self.ref.b, self.ref.c], CHECK_X)
            jump = float(np.abs(new - old).max())
            if "solid_cross_recovery" in self.flags and self._solid_on_line_accept(res, jump, now):
                return res
            if jump > JUMP_M:
                if self._outlier_hold(res, header, jump, now):
                    return res
                if "step_accept" in self.flags and self._step_accept(res, jump, old, new, now):
                    return res
                self.stats["outlier_accepted"] += 1      # 0.5 초 넘게 계속 → 진짜 변화
                res.wrap("outlier_accepted(%.2fm),%s", jump)
        elif "on_line_hold" in self.flags:
            self.on_line_since = None
        return self._accept(res, now)

    def _far_only(self, res, header):
        if res.traj.x_min_m > FAR_ONLY_XMIN_M:
            self.hold_ref(res, header, "far_only_held", "far_only_hold(xmin=%.2f,%s)", res.traj.x_min_m)
            return True
        return False

    def _on_line_hold(self, res, header, now):
        """(b) 점선을 밟고 있고 믿은 경로가 점선 반대쪽을 가리키면 유지 (최대 on_line_hold_sec)."""
        g = self.var.p.guard
        away = self.ref.c > g.on_line_min_ref_c if self.lane.rules == 1 else self.ref.c < -g.on_line_min_ref_c
        if "on_line" in res.flags and away:
            if self.on_line_since is None:
                self.on_line_since = now
            if now - self.on_line_since < g.on_line_hold_sec:
                self.ref_time = now
                self.hold_ref(res, header, "on_line_held", "on_line_hold(%.1fs,%s)", now - self.on_line_since)
                return True
            return False
        self.on_line_since = None
        return False

    def _solid_on_line_accept(self, res, jump, now):
        """(g) 실선을 밟은 채 실선 하나로 만든 새 경로가 차로 쪽을 더 가리키면 붙잡지 않고 즉시 수용."""
        if jump > JUMP_M and "solid_on_line" in res.flags and self.cross.accept_immediately(res.traj, self.ref):
            self.stats["solid_on_line_accept"] += 1
            self.outlier_since = None
            self.ref, self.ref_band, self.ref_time = copy.deepcopy(res.traj), res.band, now
            res.wrap("solid_on_line_accept(%.2fm),%s", jump)
            return True
        return False

    def _outlier_hold(self, res, header, jump, now):
        if self.outlier_since is None:
            self.outlier_since = now
        if now - self.outlier_since < OUTLIER_HOLD_SEC:
            self.ref_time = now                  # 붙잡고 있는 동안 믿은 경로가 낡았다고 보지 않음
            self.hold_ref(res, header, "outlier_held", "outlier_hold(%.2fm,%s)", jump)
            return True
        return False

    def _step_accept(self, res, jump, old, new, now):
        """(c) 변화량을 accept_step_m 씩만 받아들임 (3 점 → 2차식 다시). outlier_since 는 유지 → 다음 프레임도 이어서 수용."""
        step = self.var.p.guard.accept_step_m
        if jump <= step:
            return False
        self.stats["outlier_stepped"] += 1
        coef = np.polyfit(CHECK_X, old + np.clip(new - old, -step, step), 2)
        traj = copy.deepcopy(res.traj); traj.a, traj.b, traj.c = float(coef[0]), float(coef[1]), float(coef[2])
        band = self.band_of(traj) if res.band is not None else None
        res.traj, res.band = traj, band
        self.ref, self.ref_band, self.ref_time = copy.deepcopy(traj), band, now
        res.wrap("outlier_step(%.2fm),%s", jump)
        return True

    def _accept(self, res, now):
        self.outlier_since = None
        self.ref, self.ref_band, self.ref_time = copy.deepcopy(res.traj), res.band, now
        return res

    # ---------------------------------------------------------------- fail / publish / log
    def fail(self, header):
        hold = self.var.p.cross.hold_last_sec if "solid_cross_recovery" in self.flags else HOLD_LAST_SEC
        held = self.last_traj is not None and self.now() - self.last_ok_time <= hold
        if held:
            self.stats["held"] += 1
            self.last_traj.header = header
            self.publish(self.last_traj, self.last_band, None)
        else:
            t = LaneTrajectory(); t.header = header; t.valid = False
            self.traj_pub.publish(t)
            self.publish_band(header, None, None)

    def publish_band(self, header, band, shape):
        out = DetectionArray(); out.header = header
        if band is not None:
            d = Detection(); d.class_name = "lane"; d.score = 1.0
            if shape: d.mask.height, d.mask.width = shape
            d.mask.data = [Point2D(x=x, y=y) for x, y in band]
            out.detections.append(d)
        self.band_pub.publish(out)

    def log_stats(self):
        """frames= 통계 줄 (car_prep.sh / summarize_run.py 가 'frames=' 로 찾음). HEAD 접두 형식 그대로 + algo= + VARIANT_STATS_KEYS 순서 + last=."""
        s = self.stats
        head = ("frames=%d ok=%d (both %d, one_side %d, search %d) held=%d transverse_dropped=%d "
                "outlier_held=%d outlier_accepted=%d far_only_held=%d cross_held=%d dash_joined=%d lane=%d"
                % (s["frames"], s["ok"], s["both"], s["one_side"], s["search"], s["held"], s["transverse_dropped"],
                   s["outlier_held"], s["outlier_accepted"], s["far_only_held"], s["cross_held"], s["dash_joined"], self.lane.rules))
        var = " ".join("%s=%d" % (k, s[k]) for k in VARIANT_STATS_KEYS)
        self.get_logger().info("%s algo=%s %s last=%s" % (head, self.var.name, var, self.last_reason))


def main():
    rclpy.init()
    n = LanePathNode()
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
