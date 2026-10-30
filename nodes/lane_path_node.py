# -*- coding: utf-8 -*-
"""
경로 노드:  /detections (YOLO 선 마스크)  →  /lane_trajectory (차로 중앙 y = a·x² + b·x + c)
                                          └→ /lane_detections (화면용 차로 띠, path_viewer)

좌표는 nodes/ipm.py (x 앞 m, y 왼쪽 m). c > 0 = 차로 중앙이 차보다 왼쪽.
트랙: 항상 2차선. 왼쪽 경계 = 점선, 오른쪽 경계 = 실선, 차로 폭 0.85 m.

한 프레임
 1. 사진이 0.3 초 넘게 늦었거나 마스크가 없으면 실패 (6)
 2. 실선/점선 마스크를 칠한다. 확신도 0.30 미만, 차 진행 방향과 60° 넘게 누운 실선(횡단보도·정지선 앞 굵은 선)은 버린다
 3. find_path
    - 사진 아래→위 2줄씩: 실선의 안쪽 끝 = 오른쪽 경계, 점선의 안쪽 끝 = 왼쪽 경계 (위치·연결성 규칙으로 고름)
    - 선마다 2차식. 점선 조각 사이 빈칸이 0.6 m 이하면 이어서 한 선으로 본다
    - 중심 = 선에서 수직 0.425 m (가로로는 0.425·√(1+기울기²)). 두 선의 폭이 맞으면 두 선의 가운데, 아니면 실선 기준
    - 경계가 하나도 없으면 "중심이 오른쪽 15 cm" 경로 (search_right). 2 초 넘게 계속되면 실패
    - 중심점들에 경로 곡선 (가까운 0.45 m 우선, 점이 부족하면 0.7 m 까지. 오차 15 cm 넘으면 실패)
 4. cross_hold: 횡단보도/정지선을 0.5 초 안에 봤는데 search_right 면 → 믿은 경로 (최대 1.5 초)
 5. outlier_hold: 믿은 경로(마지막으로 받아들인 경로)와 비교
    - 경로 점이 모두 0.4 m 보다 먼 곳에만 있으면(차 앞이 안 보임) → 믿은 경로
    - x = 0 / 0.4 / 0.8 m 에서 10 cm 넘게 튀면 → 0.5 초까지 믿은 경로, 그 뒤엔 새 경로를 받아들임
 6. 실패하면 직전 경로를 1 초까지 유지, 그 뒤 valid=false (제어기가 차를 세움)
"""
import copy
import math
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
TRANSVERSE_DEG = 60.0      # 이 각도보다 누운 실선은 버림
NEAR_RANGE_M = 0.45        # 경로 곡선에 평소 쓰는 거리
EXT_RANGE_M = 0.7          # 점이 부족할 때 넓혀 볼 최대 거리 (줄 훑기도 여기까지)
MAX_IMAGE_AGE_SEC = 0.3
HOLD_LAST_SEC = 1.0        # 실패 시 직전 경로 유지 시간
SEARCH_OFFSET_M = 0.15     # 경계가 없을 때 가정하는 중심 위치 (오른쪽)
SEARCH_MAX_SEC = 2.0
DASH_MAX_GAP_M = 0.6       # 조각 사이 빈칸이 이보다 길면 잇지 않음
DASH_SEG_GAP_M = 0.06      # 점 사이가 이보다 벌어지면 다른 조각
WIDTH_TOL_M = 0.25         # 두 선 사이 폭 허용 오차
LINE_MAX_ERR_M = 0.08      # 선 하나의 곡선 오차가 이보다 크면 그 선은 안 씀
CHECK_X = np.array([0.0, 0.4, 0.8])   # 경로 튐을 비교하는 지점 (m)
JUMP_M = 0.10
OUTLIER_HOLD_SEC = 0.5
REF_MAX_AGE_SEC = 1.0      # 믿은 경로가 이보다 오래됐으면 비교하지 않음
FAR_ONLY_XMIN_M = 0.4
CROSS_CLASSES = ("crosswalk_marking", "stop_line")
CROSS_MIN_SCORE = 0.5
CROSS_RECENT_SEC = 0.5
CROSS_HOLD_MAX_SEC = 1.5


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


def nearest(ys, target):
    return min(ys, key=lambda v: abs(v - target))


class LanePathNode(Node):
    def __init__(self):
        super().__init__("line_path_node")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(DetectionArray, "/detections", self.on_detections, qos)
        self.traj_pub = self.create_publisher(LaneTrajectory, "/lane_trajectory", qos)
        self.band_pub = self.create_publisher(DetectionArray, "/lane_detections", qos)
        self.ipm = Ipm(); self.rows = None; self.row_idx = None
        self.last_traj = None; self.last_band = None; self.last_ok_time = 0.0   # 직전 정상 경로 (실패 시 유지용)
        self.ref = None; self.ref_band = None; self.ref_time = 0.0              # 믿은 경로
        self.outlier_since = None; self.search_start = None
        self.cross_seen = 0.0; self.cross_hold_since = None
        self.stats = dict.fromkeys(["frames", "ok", "both", "one_side", "held", "search", "transverse_dropped",
                                    "outlier_held", "outlier_accepted", "far_only_held", "cross_held", "dash_joined"], 0)
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

        traj, band, reason = self.find_path(md, ms, msg.header)
        traj, band, reason = self.cross_hold(traj, band, reason, msg.header)
        traj, band, reason = self.outlier_hold(traj, band, reason, msg.header)
        self.last_reason = reason
        if traj is None:
            return self.fail(msg.header)
        if not reason.startswith("search"):
            self.stats["both" if reason.startswith("solid+dashed") else "one_side"] += 1
        self.stats["ok"] += 1
        self.last_traj, self.last_band, self.last_ok_time = traj, band, time.time()
        self.traj_pub.publish(traj)
        self.publish_band(msg.header, band, shape)

    def is_transverse(self, poly):
        """실선 테두리 점들이 가장 길게 퍼진 방향(주성분)이 앞쪽과 TRANSVERSE_DEG 넘게 벌어지면 가로선."""
        xf, yl = self.ipm.to_m(poly[:, 0], poly[:, 1])
        ok = (xf > -0.5) & (xf < 4.0)                    # 지평선 근처(변환이 이상해지는 점)는 제외
        if ok.sum() < 4:
            return False
        P = np.c_[xf[ok], yl[ok]]
        dx, dy = np.linalg.svd(P - P.mean(axis=0), full_matrices=False)[2][0]
        return math.degrees(math.atan2(abs(dy), abs(dx))) > TRANSVERSE_DEG

    def find_path(self, md, ms, header):
        yl = self.ipm.yl
        n = len(self.rows)
        sr, sc = np.nonzero(run_edges(ms[self.row_idx] > 0)[0])   # 실선 구간의 왼쪽 끝 (줄 번호, 열)
        dr, dc = np.nonzero(run_edges(md[self.row_idx] > 0)[1])   # 점선 구간의 오른쪽 끝
        sk, dk = np.searchsorted(sr, np.arange(n + 1)), np.searchsorted(dr, np.arange(n + 1))   # 줄마다 시작 위치
        R, D = [], []                            # 실선 안쪽 끝 (x, y), 점선 안쪽 끝 (x, y)
        prev_r = prev_d = None
        for i, (row, xf0) in enumerate(self.rows):
            yr = yd = None
            starts = sc[sk[i]:sk[i + 1]]
            if starts.size:                      # 오른쪽 경계 = 실선 구간의 왼쪽 끝
                ys = yl[row, starts]
                if prev_r is None:               # 처음: 차 오른쪽(-1.2 ~ +0.2 m) 중 차로 폭 절반에 가장 가까운 것
                    cand = [v for v in ys if -1.2 < v < 0.2]
                    yr = nearest(cand, -HALF) if cand else None
                else:                            # 이어서: 아랫줄과 15 cm 안쪽으로 이어지는 것만
                    v = nearest(ys, prev_r); yr = v if abs(v - prev_r) < 0.15 else None
            ends = dc[dk[i]:dk[i + 1]]
            if ends.size:                        # 왼쪽 경계 = 점선 구간의 오른쪽 끝
                ys = yl[row, ends]
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None                    # 점선보다 왼쪽의 실선 = 1차선 바깥 실선
                if yr is not None:
                    ys = [v for v in ys if v > yr + 0.4]   # 점선은 실선보다 0.4 m 이상 왼쪽
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = nearest(cand, HALF) if cand else None
                elif len(ys):
                    v = nearest(ys, prev_d); yd = v if abs(v - prev_d) < 0.2 else None
            if yr is not None: prev_r = yr; R.append((xf0, float(yr)))
            if yd is not None: prev_d = yd; D.append((xf0, float(yd)))

        if not R and not D:
            now = time.time()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > SEARCH_MAX_SEC:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = -float(SEARCH_OFFSET_M)
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_right(%.1fs)" % (now - self.search_start)
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
            jump = float(np.abs(np.polyval([traj.a, traj.b, traj.c], CHECK_X) - np.polyval([self.ref.a, self.ref.b, self.ref.c], CHECK_X)).max())
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
            t = self.last_traj; t.header = header
        else:
            t = LaneTrajectory(); t.header = header; t.valid = False
        self.traj_pub.publish(t)
        self.publish_band(header, self.last_band if held else None, None)

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
                               "outlier_held=%d outlier_accepted=%d far_only_held=%d cross_held=%d dash_joined=%d last=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s["search"], s["held"],
                                  s["transverse_dropped"], s["outlier_held"], s["outlier_accepted"], s["far_only_held"],
                                  s["cross_held"], s["dash_joined"], self.last_reason))


def main():
    rclpy.init()
    n = LanePathNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
