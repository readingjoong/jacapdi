# -*- coding: utf-8 -*-
"""
=====================================================================================
 고스트 정합 보기 (ghost_view.py, 2026-10-16)  -  차의 실제 위치(고스트)를 차로와 같은 바닥 좌표에 올린다
=====================================================================================

좌표계 = vehicle.yaml (원점 앞범퍼 아래 바닥, x 앞, y 왼쪽), 변환 = ground_model.py (실측 + f 540.4 + yaw 4.96°)

한 프레임마다
   1) YOLO 실선/점선 마스크 → 행마다 안쪽 끝 픽셀 → 바닥 좌표 (점선 = 왼쪽 경계, 실선 = 오른쪽 경계)
   2) 경계마다 2차식 x→y. 차로 중앙 = 두 경계 가운데 (한쪽만 보이면 그 선 ± 0.425 m)
   3) 앞바퀴 축(x=-0.23)에서: 옆 오차 e_y = 차로 중앙의 y (+ = 중앙이 왼쪽 = 차가 오른쪽에 있음),
      방향 오차 e_psi = 차로 중앙 접선 각도 (+ = 차로가 왼쪽으로 감)
      ※ 카메라에 보이는 가장 가까운 곳은 앞범퍼 앞 약 0.15 m. 앞바퀴 축까지 0.4 m 는 2차식을 늘린 값.
   4) H 의 경로(frames.csv a,b,c)를 예전 변환(ipm_c920.yaml)으로 픽셀에 되돌린 뒤 새 변환으로 옮겨 같이 그린다.

사용
   python3 ghost_view.py <dets.jsonl> <출력폴더>     (dets.jsonl = raw 프레임에 YOLO 돌린 결과, 줄마다 {"f","h","w","d"})
   → <출력폴더>/NNNN.png (왼쪽 카메라, 오른쪽 조감도),  ghost.csv (프레임별 e_y, e_psi, 차로 폭)
"""
import csv
import json
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_model import GroundModel   # noqa: E402

D = os.path.dirname(os.path.abspath(__file__))
VEH = yaml.safe_load(open(os.path.join(D, "vehicle.yaml")))
L = VEH["vehicle"]; FRONT_AXLE = VEH["points"]["front_axle"][0]; REAR_AXLE = VEH["points"]["rear_axle"][0]
HALF = 0.425
g = GroundModel()


# ---------------------------------------------------------------------------------
# 예전 변환 (H 경로를 새 좌표로 옮기기 위해서만)
# ---------------------------------------------------------------------------------
def old_ipm(w=640, h=480):
    ip = yaml.safe_load(open(os.path.join(D, "ipm_c920.yaml")))["lane_info_extractor_node"]["ros__parameters"]
    src = np.float32(ip["ipm_src_points"]).reshape(4, 2)
    src[:, 0] *= w / float(ip["ipm_src_base_width"]); src[:, 1] *= h / float(ip["ipm_src_base_height"])
    bh = int(round(h * float(ip["bev_height_scale"])))
    dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                      [w * ip["ipm_dst_right_ratio"], bh - 1], [w * ip["ipm_dst_left_ratio"], bh - 1]])
    Minv = np.linalg.inv(cv2.getPerspectiveTransform(src, dst))
    lat, lon, ego = float(ip["lateral_meter_per_pixel"]), float(ip["longitudinal_meter_per_pixel"]), (w - 1) * 0.5

    def path_to_pixels(a, b, c, x0, x1):
        xs = np.linspace(x0, x1, 25); ys = a * xs ** 2 + b * xs + c
        bev = np.float32(np.c_[ego - ys / lat, bh - 1 - xs / lon]).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(bev, Minv).reshape(-1, 2)
    return path_to_pixels


# ---------------------------------------------------------------------------------
# 한 프레임: 경계 점 → 경계 곡선 → 차로 중앙 → 앞바퀴 기준 오차
# ---------------------------------------------------------------------------------
def boundaries(dets, h, w):
    """아래(가까운 곳)부터 위로: 점선 안쪽 끝 = 왼쪽 경계, 실선 안쪽 끝 = 오른쪽 경계.
    처음은 기대 위치(±0.425 m)에 가장 가까운 것, 그다음은 바로 아랫줄과 이어지는 것만 (E 와 같은 생각)."""
    md = np.zeros((h, w), np.uint8); ms = np.zeros((h, w), np.uint8)
    for cls, s, poly in dets:
        if cls in ("solid_line", "dashed_line") and s >= 0.3 and len(poly) > 2:
            cv2.fillPoly(md if cls == "dashed_line" else ms, [np.int32(poly)], 255)
    Dp, Sp = [], []; prev_d = prev_s = None
    for v in range(h - 36, int(h * 0.35), -2):          # 맨 아래 36 행은 보닛
        for mask, side, P in ((md, +1, Dp), (ms, -1, Sp)):
            px = np.flatnonzero(mask[v])
            if not px.size:
                continue
            runs = np.split(px, np.flatnonzero(np.diff(px) > 1) + 1)
            edge = [r[-1] + 0.5 if side > 0 else r[0] - 0.5 for r in runs]          # 차로 쪽 끝
            xs, ys = g.to_ground(np.array(edge), np.full(len(edge), v))
            prev = prev_d if side > 0 else prev_s
            if prev is None:
                ok = (ys * side > -0.2) & (ys * side < 1.0)
                if not ok.any(): continue
                i = np.flatnonzero(ok)[np.argmin(np.abs(ys[ok] - side * HALF))]
            else:
                i = int(np.argmin(np.abs(ys - prev)))
                if abs(ys[i] - prev) > 0.10: continue
            P.append((xs[i], ys[i]))
            if side > 0: prev_d = ys[i]
            else: prev_s = ys[i]
    return np.array(Dp).reshape(-1, 2), np.array(Sp).reshape(-1, 2)


def fit(P):
    """경계 점 → 3차식 계수 (낮은 차수는 앞을 0 으로 채움). 차 가까운 1.6 m 안쪽만 쓴다."""
    P = P[P[:, 0] <= 1.6] if len(P) else P
    if len(P) < 8 or np.ptp(P[:, 0]) < 0.2:
        return None
    span = np.ptp(P[:, 0])
    for deg in ((1,) if span < 0.5 else (2, 3) if span > 0.9 else (2,)):
        c = np.polyfit(P[:, 0], P[:, 1], deg)
        rms = np.sqrt(np.mean((np.polyval(c, P[:, 0]) - P[:, 1]) ** 2))
        if rms <= 0.015:                                  # 급커브라 2차식이 안 맞으면 3차식
            break
    if rms > 0.03:
        return None
    return np.r_[np.zeros(4 - len(c)), c]


def near_x(Dp, Sp):
    """경계 점이 실제로 있는 가장 가까운 x."""
    return float(min(P[:, 0].min() for P in (Dp, Sp) if len(P)))


def lane_center(cd, cs):
    """경계 2차식 → 차로 중앙 2차식 (한쪽뿐이면 ±0.425 m 를 기울기 보정해 옮김)."""
    if cd is not None and cs is not None:
        return 0.5 * (cd + cs), "both"
    c = cd if cd is not None else cs
    if c is None:
        return None, "none"
    sign = -1.0 if cd is not None else 1.0
    x = np.linspace(0.0, 1.5, 16); m = np.polyval(np.polyder(c), x)
    y = np.polyval(c, x) + sign * HALF * np.sqrt(1 + m ** 2)
    return np.polyfit(x, y, 3), "dashed_only" if cd is not None else "solid_only"


# ---------------------------------------------------------------------------------
# 그리기
# ---------------------------------------------------------------------------------
S = 260.0                                   # 조감도: 1 m = 260 px
BW, BH = 420, 760
def bev(x, y):
    return np.c_[BW / 2 - np.asarray(y) * S, BH - 60 - (np.asarray(x) - (-1.0)) * S].astype(np.int32)


def draw(img, Dp, Sp, cd, cs, cc, hpath, info, xn):
    cam = img.copy()
    x_hi = max([P[:, 0].max() for P in (Dp, Sp) if len(P)] + [0.5])
    # 카메라 위: 차 폭 띠(바퀴 바깥면 연장선), 차로 경계/중앙(보이는 구간만), H 경로
    for y in (L["width_m"] / 2, -L["width_m"] / 2):
        u, v = g.to_pixel(np.linspace(0.12, 2.0, 30), np.full(30, y))
        cv2.polylines(cam, [np.int32(np.c_[u, v])], False, (255, 255, 255), 1, cv2.LINE_AA)
    xs = np.linspace(xn if xn is not None else 0.12, min(x_hi, 1.6), 40)
    for c, col in ((cd, (0, 220, 0)), (cs, (255, 120, 0)), (cc, (0, 255, 255))):
        if c is not None:
            u, v = g.to_pixel(xs, np.polyval(c, xs)); cv2.polylines(cam, [np.int32(np.c_[u, v])], False, col, 2, cv2.LINE_AA)
    if hpath is not None:
        cv2.polylines(cam, [np.int32(hpath[0])], False, (0, 0, 255), 2, cv2.LINE_AA)

    b = np.full((BH, BW, 3), 30, np.uint8)
    for x in np.arange(-1.0, 2.01, 0.5):                                      # 0.5 m 격자
        cv2.line(b, tuple(bev(x, 0.8)[0]), tuple(bev(x, -0.8)[0]), (60, 60, 60), 1)
        cv2.putText(b, "%.1f" % x, tuple(bev(x, 0.78)[0] + [0, -3]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
    fov = np.array([g.to_ground(u, 444) for u in (0, 639)] + [g.to_ground(u, 170) for u in (639, 0)])
    cv2.polylines(b, [bev(fov[:, 0], fov[:, 1])], True, (90, 90, 90), 1)    # 카메라에 보이는 바닥
    for P, col in ((Dp, (0, 160, 0)), (Sp, (200, 90, 0))):
        for p in bev(P[:, 0], P[:, 1]) if len(P) else []:
            cv2.circle(b, tuple(p), 1, col, -1)
    xs = np.linspace(xn if xn is not None else 0.12, min(x_hi, 1.6), 60)     # 보이는 구간만 실선
    for c, col, t in ((cd, (0, 220, 0), 2), (cs, (255, 120, 0), 2), (cc, (0, 255, 255), 2)):
        if c is not None:
            cv2.polylines(b, [bev(xs, np.polyval(c, xs))], False, col, t, cv2.LINE_AA)
    if cc is not None and xn is not None:                                   # 차 쪽은 접선으로만 늘림 = 점선 (추측 구간)
        m = np.polyval(np.polyder(cc), xn); y0 = np.polyval(cc, xn)
        for x in np.arange(xn, REAR_AXLE - 0.05, -0.06):
            p1 = bev(x, y0 + m * (x - xn))[0]; p2 = bev(x - 0.03, y0 + m * (x - 0.03 - xn))[0]
            cv2.line(b, tuple(p1), tuple(p2), (0, 255, 255), 1)
    if hpath is not None:
        cv2.polylines(b, [bev(hpath[1][:, 0], hpath[1][:, 1])], False, (0, 0, 255), 2, cv2.LINE_AA)
    # 고스트: 차체, 바퀴, 축, 렌즈
    hw = L["width_m"] / 2
    cv2.polylines(b, [bev([0, 0, -L["length_m"], -L["length_m"]], [hw, -hw, -hw, hw])], True, (255, 255, 255), 2)
    for ax in (FRONT_AXLE, REAR_AXLE):
        cv2.line(b, tuple(bev(ax, L["track_m"] / 2)[0]), tuple(bev(ax, -L["track_m"] / 2)[0]), (200, 200, 200), 1)
        for y in (L["track_m"] / 2, -L["track_m"] / 2):
            cv2.polylines(b, [bev([ax + 0.06, ax + 0.06, ax - 0.06, ax - 0.06], [y + 0.03, y - 0.03, y - 0.03, y + 0.03])], True, (200, 200, 200), 1)
    cv2.circle(b, tuple(bev(*VEH["camera"]["lens"][:2])[0]), 4, (255, 0, 255), -1)
    cv2.circle(b, tuple(bev(FRONT_AXLE, 0)[0]), 4, (0, 255, 255), -1)
    y0 = 18
    for line in info:
        cv2.putText(b, line, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1); y0 += 18
    cam = cv2.resize(cam, (int(640 * BH / 480), BH))
    return np.hstack([cam, b])


def main():
    dets_path, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    to_px = old_ipm()
    csv_cache = {}
    rows_out = []
    for k, line in enumerate(open(dets_path)):
        r = json.loads(line)
        img = cv2.imread(r["f"])
        if img is None:
            continue
        Dp, Sp = boundaries(r["d"], r["h"], r["w"])
        cd, cs = fit(Dp), fit(Sp)
        cc, kind = lane_center(cd, cs)
        # H 경로 (같은 파일 이름의 frames.csv 줄)
        run = os.path.dirname(os.path.dirname(r["f"]))
        if run not in csv_cache:
            csv_cache[run] = {x["file"]: x for x in csv.DictReader(open(os.path.join(run, "frames.csv")))}
        hp = None; x = csv_cache[run].get(os.path.basename(r["f"]))
        if x and x["traj_valid"] == "1":
            px = to_px(float(x["a"]), float(x["b"]), float(x["c"]), float(x["x_min_m"]), float(x["x_max_m"]))
            hp = (px, np.c_[g.to_ground(px[:, 0], px[:, 1])])
        info = [os.path.basename(r["f"])[:20], "lane: %s" % kind]
        ey = epsi = width = None
        xn = None
        if cc is not None:
            xn = near_x(Dp, Sp)                           # 보이는 가장 가까운 곳
            yn = float(np.polyval(cc, xn)); m = float(np.polyval(np.polyder(cc), xn))
            ey = yn + m * (FRONT_AXLE - xn); epsi = float(np.degrees(np.arctan(m)))
            info += ["seen from x=%.2f m (axle %.2f m behind)" % (xn, xn - FRONT_AXLE),
                     "front axle e_y %+.3f m (extrapolated)" % ey, "e_psi %+.1f deg" % epsi]
        if cd is not None and cs is not None:
            width = float(np.polyval(cd - cs, 0.6)); info.append("width @0.6m %.3f m" % width)
        if x:
            info.append("H: c=%+.2f b=%+.2f steer %s" % (float(x["c"] or 0), float(x["b"] or 0), x["steer"]))
        cv2.imwrite(os.path.join(out, "%04d.png" % k), draw(img, Dp, Sp, cd, cs, cc, hp, info, xn))
        rows_out.append([os.path.basename(r["f"]), kind, ey, epsi, width, x["steer"] if x else ""])
    with open(os.path.join(out, "ghost.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["file", "lane", "front_axle_e_y_m", "front_axle_e_psi_deg", "width_0.6m", "steer"])
        w.writerows(rows_out)
    print("저장:", out, len(rows_out), "프레임")


if __name__ == "__main__":
    main()
