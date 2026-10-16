# -*- coding: utf-8 -*-
"""카메라 없이 A~D 경로 알고리즘 점검 (실차 노트북 컨테이너 안, 30초 이내).
실차 IPM(~/jacapdi-docker/ipm_c920.yaml)의 카메라 값으로 가상 차선 마스크를 그려 각 알고리즘의 metric_follow 에 넣는다.
  1 직선, 실선+점선      -> 모두 solid+dashed, c 약 0
  2 오른쪽 실선 안 보임  -> A 는 search_right (원본 규칙), B/C/D 는 dashed_only
  3 반경 3 m 우회전      -> 모두 유효 경로, c 음수(오른쪽)
결과가 다르면 설치나 경로(labeling-tools, ipm yaml)를 확인한다."""
import importlib.util
import math
import os
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from std_msgs.msg import Header

KIT = Path(__file__).resolve().parent
IPM = Path(os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml"))
W, H = 640, 480
FILES = {"A": "A_orig.py", "B": "B_dashed_fallback.py", "C": "C_curve_offset.py", "D": "D_stable.py"}


def cam_params():
    head = IPM.read_text().splitlines()[0]
    f = float(re.search(r"f=([\d.]+)", head).group(1)) if "f=" in head else 560.0
    h = float(re.search(r"h=([\d.]+)", head).group(1)) if "h=" in head else 0.607
    p = float(re.search(r"pitch=([\d.]+)", head).group(1)) if "pitch=" in head else 17.0
    return f, h, math.radians(p)


def proj(F, HGT, th, X, Y):
    xc = -Y; yc = -X * np.sin(th) + HGT * np.cos(th); zc = X * np.cos(th) + HGT * np.sin(th)
    return W / 2 + F * xc / zc, H / 2 + F * yc / zc


def draw_line(mask, cam, pts, width=0.05, dash=None):
    F, HGT, th = cam
    pts = np.asarray(pts, float)
    seg = np.hypot(*np.diff(pts, axis=0).T); s = np.concatenate([[0], np.cumsum(seg)])
    for i in range(len(pts) - 1):
        if dash and (s[i] % 1.0) > dash:
            continue
        a, b = pts[i], pts[i + 1]
        n = np.array([-(b - a)[1], (b - a)[0]]); n = n / (np.linalg.norm(n) + 1e-9) * width / 2
        quad = [a + n, b + n, b - n, a - n]
        uv = np.array([proj(F, HGT, th, q[0], q[1]) for q in quad])
        if np.all(np.isfinite(uv)):
            cv2.fillPoly(mask, [np.round(uv).astype(np.int32)], 255)


def scene(cam, kind):
    md = np.zeros((H, W), np.uint8); ms = np.zeros((H, W), np.uint8)
    X = np.linspace(0.3, 4.0, 200)
    if kind == "curve_right":                   # 반경 3 m 우회전: 차로 중심 y = -(R - sqrt(R^2 - x^2))
        R = 3.0
        yc = -(R - np.sqrt(np.maximum(R * R - X * X, 0)))
        ok = X < R * 0.95; X, yc = X[ok], yc[ok]
        right = np.c_[X, yc - 0.45 * (1 + 0 * X)]; left = np.c_[X, yc + 0.45]
    else:
        right = np.c_[X, -0.45 + 0 * X]; left = np.c_[X, 0.45 + 0 * X]
    if kind != "no_solid":
        draw_line(ms, cam, right - [0, 0.025])
    draw_line(md, cam, left + [0, 0.025], dash=0.45)
    return md, ms


def load(name, ipm):
    spec = importlib.util.spec_from_file_location("algo_" + name, KIT / "algo" / FILES[name])
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    n = m.LinePathNode.__new__(m.LinePathNode)
    n.ipm = ipm; n.M = n.Minv = None; n.lane_width_m = 0.85; n.near_range_m = -1.0; n.ext_range_m = 1.4
    n.search_start = None; n.search_max_sec = 2.0; n.search_offset_m = 0.15; n.near_top_row = -1
    n.stats = {"frames": 0, "ok": 0, "both": 0, "one_side": 0, "held": 0, "search": 0}; n.min_rows = 0.06; n.max_gap = 0.4
    return n


def main():
    ipm = yaml.safe_load(IPM.read_text())["lane_info_extractor_node"]["ros__parameters"]
    cam = cam_params()
    print("IPM %s  (f=%.0f h=%.3f pitch=%.1f)" % (IPM, cam[0], cam[1], math.degrees(cam[2])))
    expect = {"straight": {k: "solid+dashed" for k in FILES},
              "no_solid": {"A": "search_right", "B": "dashed_only", "C": "dashed_only", "D": "dashed_only"},
              "curve_right": {k: "" for k in FILES}}
    bad = 0
    for kind in ("straight", "no_solid", "curve_right"):
        md, ms = scene(cam, kind)
        for name in FILES:
            n = load(name, ipm)
            t, _, reason = n.metric_follow(md, ms, H, W, Header())
            ok = reason.startswith(expect[kind][name]) and t is not None and t.valid
            if kind == "curve_right":
                ok = ok and t.num_points > 0 and t.c < 0
            bad += not ok
            print("%-12s %s  %-28s %s" % (kind, name, reason, ("c=%+.3f b=%+.3f" % (t.c, t.b)) if t is not None else "-") + ("" if ok else "   <-- 예상과 다름"))
    print("\n결과: %s" % ("모두 정상" if bad == 0 else "%d 건 예상과 다름" % bad))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
