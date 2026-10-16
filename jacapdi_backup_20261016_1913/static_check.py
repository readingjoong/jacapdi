# -*- coding: utf-8 -*-
"""정지 시험: 차를 직선 차로에 평행하게 세우고, 실시간 /detections 의 경계선을 ground_model 로 바닥에 옮겨
기울기(0 이어야 함)와 위치를 잰다. 컨테이너 안에서:  python3 ~/jacapdi-docker/static_check.py   (YOLO 켜져 있어야 함)"""
import os, sys, time
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray
sys.path.insert(0, os.path.expanduser("~/jacapdi-docker"))
from ground_model import GroundModel

g = GroundModel(); rclpy.init(); n = Node("static_check"); got = []
n.create_subscription(DetectionArray, "/detections", got.append,
                      QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5))
end = time.time() + 6
while time.time() < end and len(got) < 60:
    rclpy.spin_once(n, timeout_sec=0.1)
print("frames", len(got))
res = {"dashed": [], "solid": []}
for m in got:
    md = np.zeros((480, 640), np.uint8); ms = np.zeros((480, 640), np.uint8)
    for d in m.detections:
        if d.class_name in ("dashed_line", "solid_line") and d.score >= 0.3 and len(d.mask.data) > 2:
            cv2.fillPoly(md if d.class_name == "dashed_line" else ms, [np.int32([[p.x, p.y] for p in d.mask.data])], 255)
    D, S = [], []
    for v in range(200, 445, 2):
        dd = np.flatnonzero(md[v][:400])               # 점선 = 왼쪽 경계, 안쪽(오른쪽) 끝
        ss = np.flatnonzero(ms[v][300:]) + 300          # 실선 = 오른쪽 경계, 안쪽(왼쪽) 끝
        if dd.size: D.append(g.to_ground(dd.max() + 0.5, v))
        if ss.size: S.append(g.to_ground(ss.min() - 0.5, v))
    for k, P in (("dashed", D), ("solid", S)):
        if len(P) >= 8:
            P = np.array(P); res[k].append(list(np.polyfit(P[:, 0], P[:, 1], 1)) + [P[:, 0].min(), P[:, 0].max()])
for k, L in res.items():
    if not L:
        print(k, "없음"); continue
    L = np.array(L); m, q = np.median(L[:, 0]), np.median(L[:, 1])
    print("%s: n=%d 기울기 %.4f (%.2f°, 프레임간 sd %.4f) | y@앞축 %.3f  y@뒷축 %.3f | 보이는 x %.2f~%.2f m"
          % (k, len(L), m, np.degrees(np.arctan(m)), L[:, 0].std(), m * -0.23 + q, m * -0.79 + q, np.median(L[:, 2]), np.median(L[:, 3])))
