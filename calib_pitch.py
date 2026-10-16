# -*- coding: utf-8 -*-
"""pitch 만 재보정 (f=560, h=0.607 은 직선구간 보정값 고정). 커브에서도 동작.
양쪽 선(왼쪽 dashed 안쪽 끝, 오른쪽 solid 안쪽 끝)이 같은 행에 보이는 곳의 차로폭이 0.85m 가 되는 pitch 를 찾는다.
IPM 범위: 전방 X_near(보닛 위 행) ~ X_far(기본 1.4m, 차 바로 앞만). 실패하면 yaml 변경 안 함."""
import os, sys, time
import cv2, numpy as np, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray
F, HGT, CX, CY, W, H, LANE, HALF = 560., 0.607, 320., 240., 640, 480, 0.85, 0.8
X_FAR = float(sys.argv[1]) if len(sys.argv) > 1 else 1.4
NEAR_ROW = int(sys.argv[2]) if len(sys.argv) > 2 else 440
OUT = os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")
rclpy.init(); n = Node("calib_pitch"); dets = []
n.create_subscription(DetectionArray, "/detections", dets.append, QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
rows = []
for d in dets[-30:]:
    md = np.zeros((H, W), np.uint8); ms = np.zeros((H, W), np.uint8)
    for x in d.detections:
        if x.mask.data and x.class_name in ("dashed_line", "solid_line"):
            cv2.fillPoly(md if x.class_name == "dashed_line" else ms, [np.array([[p.x, p.y] for p in x.mask.data], np.int32)], 255)
    for y in range(H // 3, H):
        a = np.flatnonzero(md[y]); b = np.flatnonzero(ms[y]); b = b[b > (a.max() if a.size else 0)]
        if a.size and b.size and b.min() > a.max() + 20: rows.append((y, a.max(), b.min()))
if len(rows) < 30:
    print("FAIL: 양쪽 선이 함께 보이는 행 부족 (%d)" % len(rows)); sys.exit(1)
rows = np.array(rows, float)
def ground(u, v, th):
    a = (u - CX) / F; b = (v - CY) / F
    Xd = np.cos(th) - np.sin(th) * b; Zd = -(np.sin(th) + np.cos(th) * b); t = HGT / -Zd
    return t * Xd, -t * a
best = None
for th_deg in np.arange(3, 50, 0.25):
    th = np.deg2rad(th_deg)
    _, YL = ground(rows[:, 1], rows[:, 0], th); _, YR = ground(rows[:, 2], rows[:, 0], th)
    wmed = float(np.median(YL - YR))
    if best is None or abs(wmed - LANE) < abs(best[1] - LANE): best = (th_deg, wmed)
th_deg, wmed = best; th = np.deg2rad(th_deg)
if abs(wmed - LANE) > 0.05:
    print("FAIL: pitch 탐색 실패 (best %.1fdeg width %.2f)" % best); sys.exit(1)
def proj(X, Y):
    xc = -Y; yc = -X * np.sin(th) + HGT * np.cos(th); zc = X * np.cos(th) + HGT * np.sin(th)
    return CX + F * xc / zc, CY + F * yc / zc
Xs = np.linspace(0.05, 10, 20000); _, vv = proj(Xs, 0 * Xs)
X_near = round(float(Xs[np.argmin(np.abs(vv - NEAR_ROW))]) + 0.02, 2)
if X_FAR <= X_near + 0.3:
    print("FAIL: X_far %.2f 가 X_near %.2f 에 너무 가까움" % (X_FAR, X_near)); sys.exit(1)
src = [float(round(c, 1)) for X, Y in [(X_FAR, HALF), (X_FAR, -HALF), (X_near, -HALF), (X_near, HALF)] for c in proj(X, Y)]
p = {"use_sim_time": False, "sub_detection_topic": "/lane_detections", "drivable_class_name": "lane",
     "ipm_src_points": src, "ipm_src_base_width": 640.0, "ipm_src_base_height": 480.0,
     "ipm_dst_left_ratio": 0.3, "ipm_dst_right_ratio": 0.7, "bev_height_scale": 4.0,
     "lateral_meter_per_pixel": round(2 * HALF / 256, 6), "longitudinal_meter_per_pixel": round((X_FAR - X_near) / 1919, 7), "ego_x_px": -1.0}
if os.path.isfile(OUT): os.replace(OUT, OUT + ".prev")
open(OUT, "w").write("# %s calib_pitch.py: f=560 h=0.607 pitch=%.2fdeg (rows %d, lane width %.3fm). 전방 %.2f~%.2fm, 좌우 +-%.1fm.\n"
                     % (time.strftime("%F %T"), th_deg, len(rows), wmed, X_near, X_FAR, HALF)
                     + yaml.safe_dump({"lane_info_extractor_node": {"ros__parameters": p}}, sort_keys=False))
print("OK pitch=%.2fdeg width=%.3f rows=%d X %.2f..%.2f m src=%s" % (th_deg, wmed, len(rows), X_near, X_FAR, src))
