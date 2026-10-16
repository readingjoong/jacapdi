# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 H  (2026-10-10)  =  G  +  "점선을 이어서 한 선으로"  +  "커브에서 선에 수직으로 0.425 m"
=====================================================================================

왜 필요한가? (10/10 주행: 왼쪽 커브에서 경로가 실선 쪽(오른쪽)으로 쏠려 덜 꺾음)
   E/F/G 는 사진 한 줄마다 차로 중심을 따로 정했다.
   (1) 점선 사이 빈칸 줄에서는 실선만 보고 "실선 + 0.425 m" → 중심점 대부분이 실선 기준.
   (2) 0.425 m 는 선에 '수직' 인 거리인데 사진 가로줄 방향으로 그대로 더했다.
       선이 기울어진 커브에서는 가로 거리가 0.425 / cos(기울기) 여야 한다 → 30° 에서 6.6 cm, 45° 에서 17.6 cm 실선 쪽으로 붙음.
   (3) 두 선의 가로 거리가 0.85 ± 0.3 m 밖이면 실선만 썼다 → 급커브(45° 에서 가로 1.20 m)일수록 (1)(2) 가 심해짐.

어떻게 바꿨나?
   [H1] 줄마다 찾은 실선 점 / 점선 점을 따로 모아, 선마다 곡선(2차식)을 하나씩 맞춘다.
        점선은 여러 조각의 점을 한꺼번에 맞추므로 빈칸을 건너 '한 선' 이 된다.
        - 조각이 2 개 이상이고 조각 사이 빈칸이 dash_max_gap_m(0.6 m) 이하일 때만 가장 가까운 ~ 가장 먼 조각 사이를 잇는다.
        - 조각이 1 개뿐이거나 빈칸이 너무 길면 점이 있는 구간만 쓴다 (밖으로 늘려서 추측하지 않음).
   [H2] 각 선의 기울기 m 에서 수직 0.425 m = 가로 0.425 × √(1+m²) 만큼 옮겨 중심을 구한다 (C 와 같은 생각).
   [H3] 두 선이 다 있으면 두 선의 가운데. 폭 검사도 기울기 반영 폭 0.85 × √(1+m²) ± width_tol_m(0.25) 로.
        폭이 안 맞으면 (점선을 잘못 잡았을 수 있음) 실선 기준.
   나머지 (가까운 범위 우선, 곡선 맞추기 품질 검사, 횡단보도 유지, 이상치 무시, 오른쪽 탐색) 는 G/F/E 와 같다.

조정값 (-p 로 바꿀 수 있음)
   dash_max_gap_m   0.6    점선 조각 사이 빈칸이 이보다 길면 잇지 않음
   dash_seg_gap_m   0.06   점 사이가 이보다 벌어지면 다른 조각으로 봄
   width_tol_m      0.25   두 선 사이 폭 허용 오차
   line_max_err_m   0.08   선 하나의 곡선 맞추기 오차가 이보다 크면 그 선은 안 씀

실행:  ALGO=H CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from E_curve_cross import LinePathNodeE, LaneTrajectory, runs_in_row   # noqa: E402
from G_cross_hold import LinePathNodeG                                # noqa: E402


class _LineFit:
    """한 선(실선 또는 점선)의 점들에 맞춘 곡선 y = f(x) 와 쓸 수 있는 x 구간들."""

    def __init__(self, P, ranges, deg_span=0.3):
        span = np.ptp(P[:, 0])
        deg = 2 if span >= deg_span else 1
        self.coef = np.polyfit(P[:, 0], P[:, 1], deg)
        self.dcoef = np.polyder(self.coef)
        self.err = float(np.sqrt(np.mean((np.polyval(self.coef, P[:, 0]) - P[:, 1]) ** 2)))
        self.ranges = ranges

    def covers(self, x):
        return any(a - 1e-6 <= x <= b + 1e-6 for a, b in self.ranges)

    def y(self, x):
        return float(np.polyval(self.coef, x))

    def k(self, x):
        """기울기 m 인 선에서 수직 1 m 가 가로 몇 m 인가 = √(1+m²)"""
        return float(np.sqrt(1.0 + np.polyval(self.dcoef, x) ** 2))


class _BoundaryFit(LinePathNodeE):
    """E 의 find_path 를 바꾼다: 줄마다 중심 → 선마다 곡선 맞춘 뒤 중심."""

    def find_path(self, md, ms, h, w, header):
        self.homography(h, w)
        half = self.lane_width_m / 2.0
        R, D = [], []                            # 실선 안쪽 끝 (x, y), 점선 안쪽 끝 (x, y)
        prev_r = prev_d = None
        last_xf = -1e9
        # --- (1) 줄마다 경계 점 찾기: 선 고르는 규칙은 E 와 같다 (폭 검사만 뒤로 미룸) ---
        for row in range(h - 1, -1, -2):
            xf0 = float(self.to_m([self.ego], [row])[0][0])
            if xf0 < last_xf - 1e-6 or xf0 > self.ext_range_m:
                break
            last_xf = xf0
            if xf0 < -0.3:
                continue
            solid_runs = runs_in_row(ms[row])
            dash_runs = runs_in_row(md[row])
            yr = yd = None
            if solid_runs:
                _, ys = self.to_m([r[0] for r in solid_runs], [row] * len(solid_runs))
                if prev_r is None:
                    cand = [v for v in ys if -1.2 < v < 0.2]
                    yr = min(cand, key=lambda v: abs(v + half)) if cand else None
                else:
                    v = min(ys, key=lambda v: abs(v - prev_r)); yr = v if abs(v - prev_r) < 0.15 else None
            if dash_runs:
                _, ys = self.to_m([r[1] for r in dash_runs], [row] * len(dash_runs))
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None
                if yr is not None:
                    ys = [v for v in ys if v > yr + 0.4]
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = min(cand, key=lambda v: abs(v - half)) if cand else None
                elif len(ys):
                    v = min(ys, key=lambda v: abs(v - prev_d)); yd = v if abs(v - prev_d) < 0.2 else None
            if yr is not None: prev_r = yr; R.append((xf0, float(yr)))
            if yd is not None: prev_d = yd; D.append((xf0, float(yd)))

        # --- 선이 하나도 없으면: E 와 같이 오른쪽 탐색 (G 가 횡단보도 근처면 직전 경로로 바꿈) ---
        if not R and not D:
            now = time.time()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > self.search_max_sec:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = -float(self.search_offset_m)
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_right(%.1fs)" % (now - self.search_start)
        self.search_start = None

        # --- (2) [H1] 선마다 곡선 맞추기 ---
        fs = self.fit_line(R, joinable=True)     # 실선도 먼 곳에서 줄 사이가 벌어지거나 잠깐 끊기면 같은 규칙으로 잇는다
        fd = self.fit_line(D, joinable=True)
        if fs is None and fd is None:
            return None, None, "too_few_points(s%d,d%d)" % (len(R), len(D))

        # --- (3) [H2][H3] x 마다 중심 ---
        xs_all = [p[0] for p in (R if fs else [])] + [p[0] for p in (D if fd else [])]
        pts = []; n_both = n_s = n_d = 0
        for x in np.arange(min(xs_all), max(xs_all) + 1e-9, 0.02):
            s_ok = fs is not None and fs.covers(x)
            d_ok = fd is not None and fd.covers(x)
            if s_ok and d_ok:
                ys, yd = fs.y(x), fd.y(x)
                kk = 0.5 * (fs.k(x) + fd.k(x))
                if abs((yd - ys) - self.lane_width_m * kk) < self.width_tol_m:
                    pts.append((x, 0.5 * (ys + yd))); n_both += 1
                    continue
                d_ok = False                                   # 폭이 안 맞음 → 실선 기준
            if s_ok:
                pts.append((x, fs.y(x) + half * fs.k(x))); n_s += 1
            elif d_ok:
                pts.append((x, fd.y(x) - half * fd.k(x))); n_d += 1
        if len(pts) < 8:
            return None, None, "too_few_points(%d)" % len(pts)

        # --- (4) 중심점들에 경로 곡선 맞추기 (E 와 같음) ---
        P = np.array(pts)
        sel = P[P[:, 0] <= self.near_range_m]
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:
            sel = P; used = "ext"
        if len(sel) < 8 or np.ptp(sel[:, 0]) < 0.08:
            return None, None, "short_span"
        deg = 2 if np.ptp(sel[:, 0]) >= 0.3 else 1
        c = np.polyfit(sel[:, 0], sel[:, 1], deg)
        if deg == 1: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, None, "bad_fit(%.2f)" % err

        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        xs = np.linspace(t.x_min_m, t.x_max_m, 12); yc = np.polyval(c, xs)
        band = [(self.ego - (y + half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs, yc)] + \
               [(self.ego - (y - half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs[::-1], yc[::-1])]
        band = cv2.perspectiveTransform(np.float32(band).reshape(-1, 1, 2), self.Minv).reshape(-1, 2)
        if n_both or (n_s and n_d):
            kind = "solid+dashed"
        else:
            kind = "solid_only" if n_s else "dashed_only"
        self.stats["dash_joined"] += int(fd is not None and fd.joined)
        return t, [(float(x), float(y)) for x, y in band], "%s,%s" % (kind, used)

    def fit_line(self, pts, joinable):
        """점들 → _LineFit. 점이 적거나 곡선이 안 맞으면 None.
        joinable(점선): 조각 사이 빈칸이 dash_max_gap_m 이하이고 조각이 2개 이상이면 처음~끝을 한 구간으로 잇는다."""
        if len(pts) < 6:
            return None
        P = np.array(sorted(pts))
        if np.ptp(P[:, 0]) < 0.06:
            return None
        cut = np.flatnonzero(np.diff(P[:, 0]) > self.dash_seg_gap_m)
        segs = np.split(np.arange(len(P)), cut + 1)
        ranges = [(P[s[0], 0], P[s[-1], 0]) for s in segs]
        gaps = [ranges[i + 1][0] - ranges[i][1] for i in range(len(ranges) - 1)]
        joined = joinable and len(segs) >= 2 and max(gaps) <= self.dash_max_gap_m
        if joined:
            ranges = [(ranges[0][0], ranges[-1][1])]
        f = _LineFit(P, ranges)
        if f.err > self.line_max_err_m:
            return None
        f.joined = joined
        return f


class LinePathNodeH(LinePathNodeG, _BoundaryFit):
    """클래스 순서: H → G → F → (G 의 횡단보도 유지) → _BoundaryFit → E"""

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.dash_max_gap_m = p("dash_max_gap_m", 0.6).value
        self.dash_seg_gap_m = p("dash_seg_gap_m", 0.06).value
        self.width_tol_m = p("width_tol_m", 0.25).value
        self.line_max_err_m = p("line_max_err_m", 0.08).value
        self.stats["dash_joined"] = 0
        self.get_logger().info("algo H: boundary fit, dash join gap<=%.2fm, width tol %.2fm"
                               % (self.dash_max_gap_m, self.width_tol_m))

    def log_stats(self):
        super().log_stats()
        self.get_logger().info("dash_joined=%d" % self.stats["dash_joined"])


def main():
    rclpy.init()
    n = LinePathNodeH()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
