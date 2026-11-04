# -*- coding: utf-8 -*-
"""
경로 노드:  /detections (YOLO 선 마스크)  →  /lane_trajectory (차로 중앙 y = a·x² + b·x + c)
                                          └→ /lane_detections (화면용 차로 띠, path_viewer)

좌표는 nodes/ipm.py (x 앞 m, y 왼쪽 m). c > 0 = 차로 중앙이 차보다 왼쪽.
트랙: 2차로, 차로 폭 0.85 m. 2차선 = 왼쪽 점선 + 오른쪽 실선, 1차선 = 왼쪽 실선 + 오른쪽 점선.
달릴 차선 = tune/lane 파일 (1 또는 2, 없으면 2). 주행 중에 바꾸면 (car.sh lane 1/2, tune_gui 버튼)
지금 차선 경로에 옆 이동을 0 → ±0.85 m 까지 lane_change_sec(tune.json, 기본 2 초) 동안 S자로 더하고,
다 옮기면 새 차선 규칙으로 바꾼다. 도중에 되돌리면 그 자리에서 다시 돌아온다.
서 있는 장애물 차(YOLO "car")가 내 차선에서 앞범퍼부터 obstacle_dist_m(tune.json, 슬라이더) 안에 보이면 옆 차선을
같은 방식으로 요청한다 (tune/lane 에 씀). 몇 차선인지 = 차 박스 맨 아래 가운데가 근처 점선보다 왼쪽(1)인지 오른쪽(2)인지,
또는 내 차선 실선(2차선: 오른쪽, 1차선: 왼쪽 안쪽 실선)에서 차로 폭 안인지 (lane_of).

한 프레임 (아래 설명은 2차선 기준. 1차선은 좌우를 뒤집어서 같은 계산)
 1. 사진이 0.3 초 넘게 늦었거나 마스크가 없으면 실패 (6)
 2. 실선/점선 마스크를 칠한다. 확신도 0.30 미만은 버린다. 도로를 가로지르는 실선(정지선, 교차로의 얇은 선)도 버린다:
    - 실선 전체 방향이 차 진행 방향과 60° 넘게 벌어질 때
    - 점선에 닿아 있고(10 cm 이내) 닿은 곳에서 그 점선과 45° 넘게 벌어질 때 (진짜 경계는 점선과 0.85 m 떨어져 나란함.
      차가 이미 돌고 있으면 가로지르는 선이 차 기준으로는 덜 누워 보여서 첫 규칙만으로는 못 거름)
 3. find_path
    - 사진 아래→위 2줄씩: 실선의 안쪽 끝 = 오른쪽 경계, 점선의 안쪽 끝 = 왼쪽 경계 (위치·연결성 규칙으로 고름)
    - 실선이 여러 개면 이어지는 선끼리 묶어 가장 긴 선, 그보다 10~50 cm 안쪽에 나란한 선이 있으면 그 안쪽 선 (주차선 무시)
    - 선마다 2차식. 점선 조각 사이 빈칸이 0.6 m 이하면 이어서 한 선으로 본다
    - 중심 = 선에서 수직 0.425 m (가로로는 0.425·√(1+기울기²)). 두 선의 폭이 맞으면 두 선의 가운데, 아니면 실선 기준
    - 경계가 하나도 없으면 "중심이 오른쪽 15 cm" 경로 (search_right). 2 초 넘게 계속되면 실패
    - 중심점들에 경로 곡선 (가까운 0.8 m 우선, 점이 부족하면 1.05 m 까지. 오차 15 cm 넘으면 실패)
 4. cross_hold: 횡단보도/정지선을 0.5 초 안에 봤는데 search_right 면 → 믿은 경로 (최대 1 초)
 5. outlier_hold: 믿은 경로(마지막으로 받아들인 경로)와 비교
    - 경로 점이 모두 0.75 m 보다 먼 곳에만 있으면(차 앞이 안 보임) → 믿은 경로
    - 두 경로가 둘 다 점을 가진 거리 구간(10 cm 이상 겹칠 때)에서 10 cm 넘게 튀면 → 0.5 초까지 믿은 경로, 그 뒤엔 새 경로를 받아들임.
      안 겹치면(예: 믿은 경로는 차 바로 앞, 새 경로는 먼 점선 조각) 비교하지 않고 받아들임 (커브를 튐으로 잘못 보지 않게)
 6. 실패하면 직전 경로를 1 초까지 유지, 그 뒤 valid=false (제어기가 차를 세움)
"""
import copy
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory

from ipm import Ipm

LANE_WIDTH_M = 0.85
HALF = LANE_WIDTH_M / 2.0
MIN_SCORE = 0.30           # 실선/점선 최소 확신도
TRANSVERSE_DEG = 60.0      # 차 진행 방향과 이 각도보다 누운 실선은 버림
CROSS_DASH_DEG = 45.0      # 점선에 닿은 실선이 닿은 곳에서 그 점선과 이 각도보다 벌어지면 버림
TOUCH_M = 0.10             # 실선과 점선이 이 거리 안이면 닿은 것
LOCAL_M = 0.3              # 실선 방향은 닿은 점에서 이 거리 안의 점들로 잼
DASH_ELONG = 2.5           # 점선 조각이 이만큼 길쭉해야 방향 기준으로 씀 (화면 끝에 잘린 조각 제외)
MIN_CHAIN_ROWS = 4         # 실선 후보가 이 줄 수보다 적게 이어지면 선으로 안 봄
INNER_MIN_M = 0.10         # 고른 실선보다 이만큼 이상 안쪽에 나란한 실선이 있으면 그쪽을 경계로 (주차선 등은 바깥)
INNER_MAX_M = 0.50         # 이보다 더 안쪽이면 다른 차로의 선이라 안 바꿈
NEAR_RANGE_M = 0.9        # 경로 곡선에 평소 쓰는 거리. 아래 셋은 11/04 카메라를 올려 사진 맨 아래가 x 0.35 m 가 되면서 +0.35
EXT_RANGE_M = 1.6         # 점이 부족할 때 넓혀 볼 최대 거리 (줄 훑기도 여기까지)
MAX_IMAGE_AGE_SEC = 0.3
HOLD_LAST_SEC = 1.0        # 실패 시 직전 경로 유지 시간
SEARCH_OFFSET_M = 0.15     # 경계가 없을 때 가정하는 중심 위치 (오른쪽)
SEARCH_MAX_SEC = 2.0
DASH_MAX_GAP_M = 0.6       # 조각 사이 빈칸이 이보다 길면 잇지 않음
DASH_SEG_GAP_M = 0.06      # 점 사이가 이보다 벌어지면 다른 조각
WIDTH_TOL_M = 0.25         # 두 선 사이 폭 허용 오차
LINE_MAX_ERR_M = 0.08      # 선 하나의 곡선 오차가 이보다 크면 그 선은 안 씀
OVERLAP_MIN_M = 0.10       # 새 경로와 믿은 경로의 점 구간(x)이 이만큼 겹칠 때만 튐을 비교
JUMP_M = 0.10
OUTLIER_HOLD_SEC = 0.5
REF_MAX_AGE_SEC = 1.0      # 믿은 경로가 이보다 오래됐으면 비교하지 않음
FAR_ONLY_XMIN_M = 0.7
CROSS_CLASSES = ("crosswalk_marking", "stop_line")
CROSS_MIN_SCORE = 0.5
CROSS_RECENT_SEC = 0.5
CROSS_HOLD_MAX_SEC = 0.5
TUNE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tune")
LANE_FILE = TUNE_DIR + "/lane"            # 달릴 차선 1 또는 2 (car.sh lane, tune_gui 가 씀)
TUNE_FILE = TUNE_DIR + "/tune.json"       # lane_change_sec, obstacle_dist_m 만 읽음
LANE_CHANGE_SEC = 2.0                     # tune.json 에 없을 때
OBSTACLE_DIST_M = 2.0                     # tune.json 에 없을 때. 0 = 장애물 피하기 끔
CAR_MIN_SCORE = 0.5                       # 장애물 차 최소 확신도
BUMPER_X_M = -0.30                        # 앞범퍼의 경로 x (x 0 = 카메라 바로 아래에서 0.84 m 앞, 렌즈는 범퍼 0.544 m 뒤)
OBSTACLE_FRAMES = 3                       # 내 차선 장애물이 이만큼 연달아 보여야 차선 변경 (한 장 잘못 잡은 것 무시)
OBSTACLE_MARGIN_M = 0.10                  # 장애물이 경계에서 이보다 가까우면(선 위) 어느 차선인지 정하지 않음


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


def direction(P):
    """점들이 가장 길게 퍼진 방향 (주성분 단위벡터)."""
    return np.linalg.svd(P - P.mean(axis=0), full_matrices=False)[2][0]


def inner_gap(a, b):
    """선 b 가 선 a 보다 안쪽(차로 가운데 쪽)으로 얼마나 있나 (겹치는 줄의 중앙값). 겹치는 줄이 3 개 미만이면 None."""
    common = a.keys() & b.keys()
    return float(np.median([b[i] - a[i] for i in common])) if len(common) >= 3 else None


def nearest(ys, target):
    return min(ys, key=lambda v: abs(v - target))


def offset_from(O, P):
    """선 하나의 바닥 점들 P 를 주된 방향의 직선으로 보고 → (O 가 그 직선에서 왼쪽으로 떨어진 수직 거리 m (+ 왼쪽),
    선을 따라 잰 P 가운데 ~ O 거리 m)."""
    v = direction(P)
    v = -v if v[0] < 0 else v                    # 앞쪽을 향하게
    d = O - P.mean(axis=0)
    return float(v[0] * d[1] - v[1] * d[0]), float(abs(v @ d))


def lane_of(O, dashes, solids, rules):
    """바닥 점 O 가 몇 차선인가. 두 가지로 보고, 둘 다 답이 있는데 다르면 None.
    - 점선(1·2차선 경계) 조각 중 선을 따라 1 m 안에서 가장 가까운 것: 그 왼쪽 = 1, 오른쪽 = 2
    - 내 차선 실선(점선 반대쪽: 2차선이면 오른쪽, 1차선이면 왼쪽 안쪽 실선)의 O 근처(앞뒤 0.5 m) 부분: 거기서 차로 폭 안이면 내 차선, 넘으면 옆 차선
      (커브에서 점선이 화면 밖으로 나가거나 장애물에 가려도 이 실선은 장애물 옆에 보일 때가 많음)
    선에서 10 cm 안(선에 걸침)이면 그 판단은 버림."""
    votes = set()
    near = [(s, along) for s, along in (offset_from(O, P) for P in dashes) if along < 1.0 and abs(s) < 1.2]
    if near:
        s = min(near, key=lambda sa: sa[1])[0]
        if abs(s) >= OBSTACLE_MARGIN_M:
            votes.add(1 if s > 0 else 2)
    sg = 1 if rules == 2 else -1                 # 내 차선 실선에서 O 까지 (+ = 차로 안쪽)
    gaps = []
    for P in solids:
        P = P[np.abs(P[:, 0] - O[0]) < 0.5]
        if len(P) < 4:
            continue
        v = direction(P)
        if math.degrees(math.atan2(abs(v[1]), abs(v[0]))) > TRANSVERSE_DEG:
            continue                             # 가로선(정지선 등)
        g = sg * offset_from(O, P)[0]
        if 0 < g < 2 * LANE_WIDTH_M:
            gaps.append(g)
    if gaps and abs(min(gaps) - LANE_WIDTH_M) >= OBSTACLE_MARGIN_M:
        votes.add(rules if min(gaps) < LANE_WIDTH_M else 3 - rules)
    return votes.pop() if len(votes) == 1 else None


def read_file(path, mtime, cast):
    """파일이 바뀌었으면 (mtime, 값), 그대로이거나 못 읽으면 (mtime, None)."""
    try:
        mt = os.path.getmtime(path)
        return (mt, None) if mt == mtime else (mt, cast(open(path).read()))
    except (OSError, ValueError):
        return mtime, None


class Lane:
    """달릴 차선과 차선 변경 중 옆 이동."""

    def __init__(self, log):
        self.log = log
        self.rules = 2                            # 지금 경계를 고르는 규칙 (1 또는 2)
        self.want = 2                             # 요청된 차선
        self.change = None                        # 변경 중: (시작 시각, 시작 이동, 목표 이동, 목표 차선)
        self.lane_file, self.tune_file = LANE_FILE, TUNE_FILE
        self.lane_mt = self.tune_mt = None
        self.sec = LANE_CHANGE_SEC
        self.obstacle_dist = OBSTACLE_DIST_M
        self.update(time.time())
        self.rules = self.want; self.change = None   # 시작할 때는 옮기지 않고 바로 그 차선

    def offset_at(self, now):
        if self.change is None:
            return 0.0
        t0, o0, o1, _ = self.change
        s = min(max((now - t0) / max(self.sec, 1e-3), 0.0), 1.0)
        return o0 + (o1 - o0) * s * s * (3 - 2 * s)

    def update(self, now):
        """파일을 확인하고 지금 경로에 더할 옆 이동(m, + 왼쪽)을 돌려준다."""
        self.lane_mt, v = read_file(self.lane_file, self.lane_mt, lambda s: int(s.strip()))
        if v in (1, 2):
            self.want = v
        self.tune_mt, v = read_file(self.tune_file, self.tune_mt, json.loads)
        if v is not None:
            self.sec = float(v.get("lane_change_sec", LANE_CHANGE_SEC))
            self.obstacle_dist = float(v.get("obstacle_dist_m", OBSTACLE_DIST_M))
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

    def request(self, n):
        """다른 차선 요청: 키 1/2 처럼 차선 파일에 쓴다 (다음 프레임에 읽어서 변경 시작, tune_gui 표시도 바뀜)."""
        with open(self.lane_file + ".tmp", "w") as f:
            f.write("%d\n" % n)
        os.replace(self.lane_file + ".tmp", self.lane_file)


class LanePathNode(Node):
    def __init__(self):
        super().__init__("line_path_node")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(DetectionArray, "/detections", self.on_detections, qos)
        self.traj_pub = self.create_publisher(LaneTrajectory, "/lane_trajectory", qos)
        self.band_pub = self.create_publisher(DetectionArray, "/lane_detections", qos)
        self.ipm = Ipm(); self.rows = None; self.row_idx = None
        self.lane = Lane(self.get_logger().info); self.off = 0.0
        self.last_traj = None; self.last_band = None; self.last_ok_time = 0.0   # 직전 정상 경로 (실패 시 유지용)
        self.ref = None; self.ref_band = None; self.ref_time = 0.0              # 믿은 경로
        self.outlier_since = None; self.search_start = None
        self.cross_seen = 0.0; self.cross_hold_since = None
        self.obstacle_n = 0                                                     # 내 차선 장애물이 연달아 보인 프레임 수
        self.stats = dict.fromkeys(["frames", "ok", "both", "one_side", "held", "search", "transverse_dropped",
                                    "outlier_held", "outlier_accepted", "far_only_held", "cross_held", "dash_joined",
                                    "obstacle_change"], 0)
        self.last_reason = ""
        self.create_timer(2.0, self.log_stats)

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
        if any(d.class_name in CROSS_CLASSES and d.score >= CROSS_MIN_SCORE for d in msg.detections):
            self.cross_seen = time.time()
        self.stats["frames"] += 1
        rules = self.lane.rules
        self.off = self.lane.update(time.time())
        if self.lane.rules != rules:
            self.shift_held(LANE_WIDTH_M if self.lane.rules == 1 else -LANE_WIDTH_M)
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        shape = next(((d.mask.height, d.mask.width) for d in msg.detections if d.mask.height > 0), None)
        if age > MAX_IMAGE_AGE_SEC or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > MAX_IMAGE_AGE_SEC else "no_mask"
            return self.fail(msg.header)
        h, w = shape
        self.setup(h, w)
        md = np.zeros((h, w), np.uint8)    # 점선
        ms = np.zeros((h, w), np.uint8)    # 실선
        lines = [(d.class_name, np.array([[p.x, p.y] for p in d.mask.data], np.float32)) for d in msg.detections
                 if d.class_name in ("solid_line", "dashed_line") and d.score >= MIN_SCORE and d.mask.data]
        g_lines = [(c, g) for c, g in ((c, self.ground(poly)) for c, poly in lines) if g is not None]
        dashes = [g for c, g in g_lines if c == "dashed_line" and g[3] >= DASH_ELONG]
        self.check_obstacle(msg, [g[0] for c, g in g_lines if c == "dashed_line"], [g[0] for c, g in g_lines if c == "solid_line"])
        for c, poly in lines:
            if c == "solid_line" and self.is_transverse(poly, dashes):
                self.stats["transverse_dropped"] += 1
                continue
            cv2.fillPoly(ms if c == "solid_line" else md, [np.clip(np.round(poly), [0, 0], [w - 1, h - 1]).astype(np.int32)], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            return self.fail(msg.header)

        traj, band, reason = self.find_path(md, ms, msg.header, self.lane.rules == 1)
        traj, band, reason = self.cross_hold(traj, band, reason, msg.header)
        traj, band, reason = self.outlier_hold(traj, band, reason, msg.header)
        self.last_reason = reason
        if traj is None:
            return self.fail(msg.header)
        if not reason.startswith("search"):
            self.stats["both" if reason.startswith("solid+dashed") else "one_side"] += 1
        self.stats["ok"] += 1
        self.last_traj, self.last_band, self.last_ok_time = traj, band, time.time()
        self.publish(traj, band, shape)

    def check_obstacle(self, msg, dashes, solids):
        """서 있는 장애물 차가 내 차선에서 앞범퍼부터 obstacle_dist_m 안에 OBSTACLE_FRAMES 번 연달아 보이면 옆 차선 요청.
        차 위치 = 박스 맨 아래 가운데 (바퀴가 바닥에 닿는 곳). 차선 변경 중엔 보지 않음."""
        dist = self.lane.obstacle_dist
        if dist <= 0 or self.lane.change is not None:
            self.obstacle_n = 0
            return
        near = None
        for d in msg.detections:
            if d.class_name != "car" or d.score < CAR_MIN_SCORE or not d.mask.data:
                continue
            P = np.array([[p.x, p.y] for p in d.mask.data], np.float32)
            xs, ys = self.ipm.to_m(np.array([0.5 * (P[:, 0].min() + P[:, 0].max())]), np.array([P[:, 1].max()]))
            ahead = float(xs[0]) - BUMPER_X_M
            if ahead <= dist and lane_of(np.array([xs[0], ys[0]], float), dashes, solids, self.lane.rules) == self.lane.rules:
                near = ahead if near is None else min(near, ahead)
        self.obstacle_n = self.obstacle_n + 1 if near is not None else 0
        if self.obstacle_n >= OBSTACLE_FRAMES:
            to = 1 if self.lane.rules == 2 else 2
            self.get_logger().info("obstacle in lane %d, %.2f m ahead -> lane %d" % (self.lane.rules, near, to))
            self.lane.request(to)
            self.stats["obstacle_change"] += 1
            self.obstacle_n = 0

    def publish(self, traj, band, shape):
        """차선 변경 중이면 지금 옆 이동을 더해서 내보낸다 (계산·유지 판단은 모두 이동 없는 차선 기준)."""
        if self.off:
            traj = copy.deepcopy(traj); traj.c += self.off
            band = self.band_of(traj) if band is not None else None
        self.traj_pub.publish(traj)
        self.publish_band(traj.header, band, shape)

    def shift_held(self, dy):
        """차선 변경을 다 해서 규칙이 바뀐 순간: 붙잡아 둔 경로(믿은 경로, 직전 경로)를 새 차선 기준으로 옮긴다.
        안 옮기면 새 규칙 경로가 0.85 m 튄 것으로 보여 튐방지가 원래 차선 경로를 0.5 초 내보낸다."""
        for k in ("ref", "last_traj"):
            t = getattr(self, k)
            if t is not None:
                t = copy.deepcopy(t); t.c += dy; setattr(self, k, t)
        self.ref_band = self.band_of(self.ref) if self.ref_band is not None else None
        self.last_band = self.band_of(self.last_traj) if self.last_band is not None else None

    def band_of(self, t):
        """경로 ± 0.425 m 띠의 사진 좌표 (화면용)."""
        bx = np.linspace(t.x_min_m, t.x_max_m, 12); by = np.polyval([t.a, t.b, t.c], bx)
        return [(float(a), float(b)) for a, b in self.ipm.to_px(np.r_[bx, bx[::-1]], np.r_[by + HALF, by[::-1] - HALF])]

    def ground(self, poly):
        """마스크 테두리 → (바닥 점들 (N, 2), 가운데, 주된 방향 단위벡터, 길쭉한 정도). 점이 4 개 미만이면 None."""
        xf, yl = self.ipm.to_m(poly[:, 0], poly[:, 1])
        ok = (xf > -0.5) & (xf < 4.0)                    # 지평선 근처(변환이 이상해지는 점)는 제외
        if ok.sum() < 4:
            return None
        P = np.c_[xf[ok], yl[ok]]
        _, sv, vt = np.linalg.svd(P - P.mean(axis=0), full_matrices=False)
        return P, P.mean(axis=0), vt[0], sv[0] / max(sv[1], 1e-9)

    def is_transverse(self, poly, dashes):
        """도로를 가로지르는 실선인가 (차 진행 방향과 60° 넘음, 또는 점선에 닿아 그 점선과 45° 넘음)."""
        g = self.ground(poly)
        if g is None:
            return False
        P, _, v, _ = g
        if math.degrees(math.atan2(abs(v[1]), abs(v[0]))) > TRANSVERSE_DEG:
            return True
        lo, hi = P.min(axis=0) - TOUCH_M, P.max(axis=0) + TOUCH_M
        for Q, _, vd, _ in dashes:
            if np.any(Q.max(axis=0) < lo) or np.any(Q.min(axis=0) > hi):
                continue                                 # 감싸는 상자끼리 떨어져 있으면 닿을 수 없음
            d = np.hypot(P[:, None, 0] - Q[None, :, 0], P[:, None, 1] - Q[None, :, 1])   # 실선 점 × 점선 점 거리
            i = np.unravel_index(np.argmin(d), d.shape)
            if d[i] > TOUCH_M:
                continue
            local = P[np.hypot(*(P - P[i[0]]).T) < LOCAL_M]
            vs = direction(local) if len(local) >= 4 else v
            if math.degrees(math.acos(min(1.0, abs(float(np.dot(vs, vd)))))) > CROSS_DASH_DEG:
                return True
        return False

    def solid_rows(self, sc, sk, yl, mirror):
        """줄마다 실선 안쪽 끝 후보를 아랫줄과 15 cm 안으로 이어 선들로 묶고 하나를 고른다 → {줄 번호: y}.
        새 선은 차 오른쪽 -1.2 ~ +0.2 m 에서만 시작. 기본은 가장 긴 선, 그보다 10~50 cm 안쪽에 나란한 선이 있으면 그쪽
        (주차선처럼 차로 경계 바깥에 실선이 하나 더 보일 때. 안쪽 선이 몇 줄 끊겨도 바깥 선으로 옮겨 붙지 않음)."""
        chains = []                              # [마지막 y, {줄 번호: y}]
        for i, (row, _) in enumerate(self.rows):
            starts = sc[sk[i]:sk[i + 1]]
            if not starts.size:
                continue
            ys = -yl[row, starts] if mirror else yl[row, starts]
            pairs = sorted((abs(v - c[0]), j, k) for j, c in enumerate(chains) for k, v in enumerate(ys) if abs(v - c[0]) < 0.15)
            took_c, took_v = set(), set()
            for _, j, k in pairs:                # 가까운 짝부터: 선마다 후보 하나, 후보마다 선 하나
                if j not in took_c and k not in took_v:
                    took_c.add(j); took_v.add(k); chains[j][0] = ys[k]; chains[j][1][i] = ys[k]
            chains += [[v, {i: v}] for k, v in enumerate(ys) if k not in took_v and -1.2 < v < 0.2]
        chains = [c[1] for c in chains if len(c[1]) >= MIN_CHAIN_ROWS]
        if not chains:
            return {}
        best = max(chains, key=len)
        for _ in range(len(chains)):
            inner = [(g, c) for c in chains if c is not best and (g := inner_gap(best, c)) is not None and INNER_MIN_M <= g <= INNER_MAX_M]
            if not inner:
                break
            best = max(inner, key=lambda gc: gc[0])[1]
        return best

    def find_path(self, md, ms, header, mirror=False):
        """mirror (1차선): 실선은 구간의 오른쪽 끝, 점선은 왼쪽 끝을 쓰고 옆 좌표 부호를 바꿔 2차선과 같은 계산 → 결과를 다시 뒤집음."""
        yl = self.ipm.yl
        n = len(self.rows)
        sr, sc = np.nonzero(run_edges(ms[self.row_idx] > 0)[int(mirror)])       # 실선의 안쪽 끝 (줄 번호, 열)
        dr, dc = np.nonzero(run_edges(md[self.row_idx] > 0)[1 - int(mirror)])   # 점선의 안쪽 끝
        sk, dk = np.searchsorted(sr, np.arange(n + 1)), np.searchsorted(dr, np.arange(n + 1))   # 줄마다 시작 위치
        solid = self.solid_rows(sc, sk, yl, mirror)
        R, D = [], []                            # 실선 안쪽 끝 (x, y), 점선 안쪽 끝 (x, y)
        prev_d = None
        for i, (row, xf0) in enumerate(self.rows):
            yr, yd = solid.get(i), None
            ends = dc[dk[i]:dk[i + 1]]
            if ends.size:                        # 왼쪽 경계 = 점선 구간의 오른쪽 끝
                ys = -yl[row, ends] if mirror else yl[row, ends]
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None                    # 점선보다 왼쪽의 실선 = 1차선 바깥 실선
                if yr is not None:
                    ys = [v for v in ys if v > yr + 0.4]   # 점선은 실선보다 0.4 m 이상 왼쪽
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = nearest(cand, HALF) if cand else None
                elif len(ys):
                    v = nearest(ys, prev_d); yd = v if abs(v - prev_d) < 0.2 else None
            if yr is not None: R.append((xf0, float(yr)))
            if yd is not None: prev_d = yd; D.append((xf0, float(yd)))

        if not R and not D:
            now = time.time()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > SEARCH_MAX_SEC:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = float(SEARCH_OFFSET_M) if mirror else -float(SEARCH_OFFSET_M)
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_%s(%.1fs)" % ("left" if mirror else "right", now - self.search_start)
        self.search_start = None

        fs, fd = fit_line(R), fit_line(D)
        if fs is None and fd is None:
            return None, None, "too_few_points(s%d,d%d)" % (len(R), len(D))
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
        if keep.sum() < 8:
            return None, None, "too_few_points(%d)" % keep.sum()

        P = np.c_[x[keep], yc[keep]]
        sel = P[P[:, 0] <= NEAR_RANGE_M]
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:           # 가까운 점이 부족하면 더 멀리까지
            sel = P; used = "ext"
        if len(sel) < 8 or np.ptp(sel[:, 0]) < 0.08:
            return None, None, "short_span"
        c = np.polyfit(sel[:, 0], sel[:, 1], 2 if np.ptp(sel[:, 0]) >= 0.3 else 1)
        if len(c) == 2: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, None, "bad_fit(%.2f)" % err
        if mirror:
            c = -c

        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        bx = np.linspace(t.x_min_m, t.x_max_m, 12); by = np.polyval(c, bx)          # 화면용: 경로 ± 0.425 m 띠
        band = self.ipm.to_px(np.r_[bx, bx[::-1]], np.r_[by + HALF, by[::-1] - HALF])
        n_s, n_d = use_s.any(), use_d.any()
        kind = "solid+dashed" if (both.any() or (n_s and n_d)) else ("solid_only" if n_s else "dashed_only")
        self.stats["dash_joined"] += int(fd is not None and fd.joined)
        return t, [(float(a), float(b)) for a, b in band], "%s,%s" % (kind, used)

    def ref_ok(self, now):
        return self.ref is not None and now - self.ref_time <= REF_MAX_AGE_SEC

    def hold_ref(self, header, reason, key):
        self.stats[key] += 1
        t = copy.deepcopy(self.ref); t.header = header
        return t, self.ref_band, reason

    def cross_hold(self, traj, band, reason, header):
        """횡단보도/정지선 근처의 search_right → 믿은 경로."""
        now = time.time()
        if traj is None or not reason.startswith("search"):
            self.cross_hold_since = None
            return traj, band, reason
        if now - self.cross_seen <= CROSS_RECENT_SEC and self.ref_ok(now):
            if self.cross_hold_since is None:
                self.cross_hold_since = now
            if now - self.cross_hold_since <= CROSS_HOLD_MAX_SEC:
                return self.hold_ref(header, "cross_hold(%.1fs,%s)" % (now - self.cross_hold_since, reason), "cross_held")
        return traj, band, reason

    def outlier_hold(self, traj, band, reason, header):
        """먼 곳 점만으로 만든 경로, 갑자기 튄 경로 → 믿은 경로. 받아들인 경로는 믿은 경로가 된다."""
        if traj is None:
            return traj, band, reason
        now = time.time()
        if self.ref_ok(now):
            if traj.x_min_m > FAR_ONLY_XMIN_M:
                return self.hold_ref(header, "far_only_hold(xmin=%.2f,%s)" % (traj.x_min_m, reason), "far_only_held")
            lo, hi = max(traj.x_min_m, self.ref.x_min_m), min(traj.x_max_m, self.ref.x_max_m)
            jump = 0.0                                   # 두 경로가 둘 다 점을 가진 거리 구간에서만 비교. 안 겹치면 비교할 근거가 없어 받아들임
            if hi - lo >= OVERLAP_MIN_M:
                xs = np.linspace(lo, hi, 3)
                jump = float(np.abs(np.polyval([traj.a, traj.b, traj.c], xs) - np.polyval([self.ref.a, self.ref.b, self.ref.c], xs)).max())
            if jump > JUMP_M:
                if self.outlier_since is None:
                    self.outlier_since = now
                if now - self.outlier_since < OUTLIER_HOLD_SEC:
                    self.ref_time = now                  # 붙잡고 있는 동안 믿은 경로가 낡았다고 보지 않음
                    return self.hold_ref(header, "outlier_hold(%.2fm,%s)" % (jump, reason), "outlier_held")
                self.stats["outlier_accepted"] += 1      # 0.5 초 넘게 계속 → 진짜 변화
                reason = "outlier_accepted(%.2fm),%s" % (jump, reason)
        self.outlier_since = None
        self.ref, self.ref_band, self.ref_time = copy.deepcopy(traj), band, now
        return traj, band, reason

    def fail(self, header):
        held = self.last_traj is not None and time.time() - self.last_ok_time <= HOLD_LAST_SEC
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
        s = self.stats   # car_prep.sh / summarize_run.py 가 "frames=" 로 이 줄을 찾는다
        self.get_logger().info("frames=%d ok=%d (both %d, one_side %d, search %d) held=%d transverse_dropped=%d "
                               "outlier_held=%d outlier_accepted=%d far_only_held=%d cross_held=%d dash_joined=%d "
                               "obstacle_change=%d lane=%d last=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s["search"], s["held"],
                                  s["transverse_dropped"], s["outlier_held"], s["outlier_accepted"], s["far_only_held"],
                                  s["cross_held"], s["dash_joined"], s["obstacle_change"], self.lane.rules, self.last_reason))


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
