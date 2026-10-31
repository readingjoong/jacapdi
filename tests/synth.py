# -*- coding: utf-8 -*-
"""합성 차선 프레임 생성 (sim 의존 없음).

바닥 좌표(x 앞 m, y 왼쪽 m, 원점 = IPM 원점)에 선을 놓고 nodes/ipm.py 의 Ipm.to_px 로 640×480 사진 좌표에 투영,
cv2.fillPoly → findContours 로 YOLO 마스크 폴리곤 흉내 → [(class_name, poly(N,2) float32 px)] 목록.
스크래치 harness.py(GT 라벨 → 폴리곤) 의 포즈 목록(dy, dyaw 조합)을 이식한 것.

차로 규칙: 2차선 = 왼쪽 점선(+0.425) + 오른쪽 실선(-0.425) + 반대 차로 바깥 실선(+1.275)
           1차선 = 왼쪽 실선(+0.425) + 오른쪽 점선(-0.425) + 바깥 실선(-1.275)
포즈 (dy, dyaw): 차가 차로 중심에서 dy 만큼 왼콍(+), dyaw° 만큼 왼쪽으로 틀어진 상태.
"""
import math

import cv2
import numpy as np

LANE_W = 0.85
HALF = LANE_W / 2.0
LINE_W = 0.10            # 선 폭 m
DASH_ON, DASH_OFF = 0.55, 0.45
X_MIN, X_MAX = -0.3, 3.0  # 투영하는 앞 거리 범위 (IPM 유효 범위 안)
S_RANGE = (-1.0, 4.0, 0.01)
MIN_AREA = 20.0
H, W = 480, 640

DY = [0.0, 0.1, -0.1, 0.4, 0.6]
DYAW = [0, 8, -8, 10]
COMBOS7 = [(0.0, 0), (0.1, 0), (-0.1, 0), (0.0, 8), (0.0, -8), (0.4, 0), (0.6, 10)]


# ---------------------------------------------------------------- 중심선 (트랙 좌표, 차 중심선 기준)
def centerline(kind="straight", R=3.0, turn=1, x0=0.8):
    """s(호 길이) 격자 → (X, Y, heading). kind: straight | arc | entry(직선→x0 부터 호) | exit(호→x0 부터 직선)."""
    s = np.arange(*S_RANGE)
    if kind == "straight":
        return s, s.copy(), np.zeros_like(s), np.zeros_like(s)
    if kind == "arc":
        return s, R * np.sin(s / R), turn * R * (1 - np.cos(s / R)), turn * s / R
    if kind == "entry":
        th = np.where(s < x0, 0.0, turn * (s - x0) / R)
        X = np.where(s < x0, s, x0 + R * np.sin((s - x0) / R))
        Y = np.where(s < x0, 0.0, turn * R * (1 - np.cos((s - x0) / R)))
        return s, X, Y, th
    if kind == "exit":                      # s<x0 호(차는 호 위), 그 뒤 접선 직선
        th_e = turn * x0 / R
        Xe, Ye = R * np.sin(x0 / R), turn * R * (1 - np.cos(x0 / R))
        th = np.where(s < x0, turn * s / R, th_e)
        X = np.where(s < x0, R * np.sin(s / R), Xe + (s - x0) * np.cos(th_e))
        Y = np.where(s < x0, turn * R * (1 - np.cos(s / R)), Ye + (s - x0) * np.sin(th_e))
        return s, X, Y, th
    raise ValueError(kind)


def offset_line(X, Y, th, d):
    """중심선에서 왼쪽으로 d 만큼 떨어진 선."""
    return X - d * np.sin(th), Y + d * np.cos(th)


def to_vehicle(X, Y, dy, dyaw_deg):
    """트랙 좌표 → 차 좌표. 차는 (0, dy) 에서 dyaw 만큼 왼쪽으로 틀어져 있음."""
    c, s = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))
    Yr = Y - dy
    return c * X + s * Yr, -s * X + c * Yr


# ---------------------------------------------------------------- 마스크 → 폴리곤
class Scene:
    """한 프레임의 클래스별 마스크. add_* 로 선을 칠하고 detections() 로 폴리곤 목록을 얻는다."""

    def __init__(self, ipm, h=H, w=W):
        self.ipm = ipm; self.h, self.w = h, w
        self.masks = {}

    def _mask(self, cls):
        if cls not in self.masks:
            self.masks[cls] = np.zeros((self.h, self.w), np.uint8)
        return self.masks[cls]

    def fill_ground_poly(self, cls, xs, ys):
        """차 좌표 폴리곤(x, y 배열) → 사진에 칠하기. 모든 점이 X 범위 밖이면 생략."""
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        ok = (xs >= X_MIN) & (xs <= X_MAX)
        if ok.sum() < 3:
            return
        px = self.ipm.to_px(xs[ok], ys[ok])
        if not np.all(np.isfinite(px)):
            return
        cv2.fillPoly(self._mask(cls), [np.round(px).astype(np.int32).reshape(-1, 1, 2)], 255)

    def add_line(self, cls, s, X, Y, th, dashed=False, phase=0.0, s_lo=None, s_hi=None, width=LINE_W):
        """중심선 위의 선(이미 offset 된 X, Y)을 폭 width 띠로 칠한다. dashed 면 DASH_ON/OFF 주기, phase 는 m 단위 위상."""
        lo = S_RANGE[0] if s_lo is None else s_lo
        hi = S_RANGE[1] if s_hi is None else s_hi
        if dashed:
            on = ((s + phase) % (DASH_ON + DASH_OFF)) < DASH_ON
        else:
            on = np.ones_like(s, bool)
        on &= (s >= lo) & (s <= hi)
        idx = np.flatnonzero(on)
        if idx.size == 0:
            return
        for seg in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
            if seg.size < 2:
                continue
            Xl, Yl = offset_line(X[seg], Y[seg], th[seg], width / 2)
            Xr, Yr = offset_line(X[seg], Y[seg], th[seg], -width / 2)
            self.fill_ground_poly(cls, np.r_[Xl, Xr[::-1]], np.r_[Yl, Yr[::-1]])

    def detections(self, min_area=MIN_AREA):
        out = []
        for cls in sorted(self.masks):
            cnts, _ = cv2.findContours(self.masks[cls], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < min_area or len(c) < 3:
                    continue
                out.append((cls, c.reshape(-1, 2).astype(np.float32)))
        return out


def lane_lines(lane):
    """(class, offset d, dashed) 목록. 차 중심선 기준 왼쪽 +."""
    if lane == 2:
        return [("dashed_line", HALF, True), ("solid_line", -HALF, False), ("solid_line", 3 * HALF, False)]
    return [("solid_line", HALF, False), ("dashed_line", -HALF, True), ("solid_line", -3 * HALF, False)]


def lane_frame(ipm, lane, dy, dyaw, kind="straight", R=3.0, turn=1, x0=0.8, phase=0.0,
               drop=(), x_start=None, extra=None):
    """차로 선 프레임. drop: 생략할 선 index 집합 (0 왼쪽, 1 오른콍, 2 바깥). x_start: 선이 보이기 시작하는 차 좌표 x (far_only 용).
    extra: Scene 에 더 그릴 콜백 extra(scene, s, X, Y, th)."""
    s, X, Y, th = centerline(kind, R, turn, x0)
    sc = Scene(ipm)
    for i, (cls, d, dashed) in enumerate(lane_lines(lane)):
        if i in drop:
            continue
        Xo, Yo = offset_line(X, Y, th, d)
        xv, yv = to_vehicle(Xo, Yo, dy, dyaw)
        thv = th - math.radians(dyaw)                            # 차 좌표 heading
        s_lo = None
        if x_start is not None:
            far = np.flatnonzero(xv >= x_start)
            if far.size == 0:
                continue
            s_lo = float(s[far[0]])
        sc.add_line(cls, s, xv, yv, thv, dashed=dashed, phase=phase, s_lo=s_lo)
    if extra is not None:
        xv, yv = to_vehicle(X, Y, dy, dyaw)
        extra(sc, s, xv, yv, th - math.radians(dyaw))
    return sc.detections()


# ---------------------------------------------------------------- 오인 케이스
def crosswalk(cls, xc, stripe_w=0.3, gap=0.3, depth=0.4, y_span=1.3):
    """차 좌표 xc 앞에 가로 줄무늬(횡단보도). cls 가 solid_line 이면 '횡단보도를 실선으로 오인' 케이스."""
    def draw(sc, s, X, Y, th):
        y = -y_span
        while y < y_span:
            xs = [xc, xc + depth, xc + depth, xc]; ys = [y, y, y + stripe_w, y + stripe_w]
            sc.fill_ground_poly(cls, xs, ys)
            y += stripe_w + gap
    return draw


def parking_stubs(side=-1, y0=1.3, length=0.5, every=0.6, cls="dashed_line"):
    """실선 바깥(side −1 오른쪽) 에 가로 짧은 구획선. 점선으로 오인."""
    def draw(sc, s, X, Y, th):
        x = -0.2
        while x < 2.5:
            xs = [x, x + LINE_W, x + LINE_W, x]; ys = [side * y0, side * y0, side * (y0 + length), side * (y0 + length)]
            sc.fill_ground_poly(cls, xs, ys)
            x += every
    return draw


def parallel_dashes(d, phase=0.0, cls="dashed_line"):
    """중심선에서 d 떨어진 곳에 나란한 점선 (주차 구획 긴 변 / 넘은 실선 바깥 점선)."""
    def draw(sc, s, X, Y, th):
        Xo, Yo = offset_line(X, Y, th, d)
        sc.add_line(cls, s, Xo, Yo, th, dashed=True, phase=phase)
    return draw


# ---------------------------------------------------------------- 프레임 목록
def frames(ipm):
    """[(이름, lane, dets)] ≥1000 프레임. 순서 고정(골든 비교 순서)."""
    out = []

    def add(name, lane, **kw):
        out.append((name, lane, lane_frame(ipm, lane, **kw)))

    for lane in (2, 1):
        for dy in DY:
            for dyaw in DYAW:
                for ph in (0.0, 0.5):
                    add("straight dy%+.2f yaw%+d ph%.1f" % (dy, dyaw, ph), lane, dy=dy, dyaw=dyaw, phase=ph)
        for R in (1.5, 2.0, 3.0, 5.0):
            for turn in (1, -1):
                for dy, dyaw in COMBOS7:
                    for ph in (0.0, 0.5):
                        add("arc R%.1f t%+d dy%+.2f yaw%+d ph%.1f" % (R, turn, dy, dyaw, ph), lane,
                            dy=dy, dyaw=dyaw, kind="arc", R=R, turn=turn, phase=ph)
        for x0 in (0.3, 0.8, 1.3):
            for R in (1.5, 3.0):
                for turn in (1, -1):
                    for dy, dyaw in COMBOS7:
                        add("entry x%.1f R%.1f t%+d dy%+.2f yaw%+d" % (x0, R, turn, dy, dyaw), lane,
                            dy=dy, dyaw=dyaw, kind="entry", R=R, turn=turn, x0=x0, phase=0.25)
        for x0 in (0.3, 0.8):
            for R in (1.5, 3.0):
                for turn in (1, -1):
                    for dy, dyaw in COMBOS7:
                        add("exit x%.1f R%.1f t%+d dy%+.2f yaw%+d" % (x0, R, turn, dy, dyaw), lane,
                            dy=dy, dyaw=dyaw, kind="exit", R=R, turn=turn, x0=x0, phase=0.75)
        # 실선을 넘은 포즈 (|dy| > 0.425)
        for dy in (0.5, -0.5, 0.7, -0.7, 0.9, -0.9):
            for dyaw in (0, 10, -10, 20):
                for kind, R in (("straight", 3.0), ("arc", 2.0)):
                    add("crossed %s dy%+.2f yaw%+d" % (kind, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=R, turn=1)
        # 선을 밟은 포즈
        for dy in (0.425, -0.425, 0.35, -0.35, 0.5, -0.5):
            for dyaw in (0, 8, -8):
                for kind, turn in (("straight", 1), ("arc", 1), ("arc", -1)):
                    add("on_line %s t%+d dy%+.3f yaw%+d" % (kind, turn, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=3.0, turn=turn)
        # 횡단보도: 올바른 클래스 / 실선 오인
        for xc in (0.3, 0.6, 1.0):
            for dy, dyaw in COMBOS7:
                for cls in ("crosswalk_marking", "solid_line"):
                    add("crosswalk %s x%.1f dy%+.2f yaw%+d" % (cls, xc, dy, dyaw), lane, dy=dy, dyaw=dyaw,
                        extra=crosswalk(cls, xc))
        # 주차 구획선 오인 (실선 바깥 가로 토막 / 나란한 점선)
        for kind in ("straight", "arc"):
            for dy, dyaw in COMBOS7:
                side = -1 if lane == 2 else 1
                add("parking stubs %s dy%+.2f yaw%+d" % (kind, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=3.0,
                    extra=parking_stubs(side=side))
                add("parking dashes %s dy%+.2f yaw%+d" % (kind, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=3.0,
                    extra=parallel_dashes(side * 1.0, phase=0.3))
        # 먼 곳만 보임 (far_only)
        for xs in (0.5, 0.8):
            for dy, dyaw in COMBOS7:
                add("far_only x%.1f dy%+.2f yaw%+d" % (xs, dy, dyaw), lane, dy=dy, dyaw=dyaw, x_start=xs)
        # 선 하나만 (dashed_only / solid_only)
        for kind, turn in (("straight", 1), ("arc", 1), ("arc", -1)):
            for dy, dyaw in COMBOS7:
                add("dashed_only %s t%+d dy%+.2f yaw%+d" % (kind, turn, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=2.0, turn=turn,
                    drop=(1, 2) if lane == 2 else (0, 2))
                add("solid_only %s t%+d dy%+.2f yaw%+d" % (kind, turn, dy, dyaw), lane, dy=dy, dyaw=dyaw, kind=kind, R=2.0, turn=turn,
                    drop=(0, 2) if lane == 2 else (1, 2))
    return out


if __name__ == "__main__":
    import os, sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "nodes"))
    from ipm import Ipm
    ipm = Ipm(); ipm.setup(H, W)
    fr = frames(ipm)
    print("frames", len(fr))
    if len(sys.argv) > 1:   # 미리보기 PNG 몇 장
        for i in range(0, len(fr), max(1, len(fr) // 12)):
            name, lane, dets = fr[i]
            img = np.zeros((H, W, 3), np.uint8)
            for cls, poly in dets:
                col = (0, 255, 0) if cls == "dashed_line" else ((0, 0, 255) if cls == "solid_line" else (255, 0, 0))
                cv2.fillPoly(img, [np.round(poly).astype(np.int32)], col)
            cv2.imwrite(os.path.join(sys.argv[1], "f%04d.png" % i), img)
            print(i, lane, name, len(dets))
