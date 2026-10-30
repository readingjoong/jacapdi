# -*- coding: utf-8 -*-
"""사진 픽셀 ↔ 바닥 미터 변환 (IPM). 경로 노드, frame_logger, path_viewer 가 같이 쓴다. 설정: config/ipm_c920.yaml

좌표: x = 차 앞쪽 거리 m (0 = 카메라 보정 구간의 가장 가까운 줄, 차 앞 약 0.84 m), y = 차 왼쪽 거리 m (+ 왼쪽).
사진 속 바닥 사다리꼴 4점 → 조감도(BEV) 직사각형 4점 변환 M 으로 옮긴 뒤, 조감도 픽셀 × (미터/픽셀)."""
import os

import cv2
import numpy as np
import yaml

IPM_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "ipm_c920.yaml")


class Ipm:
    def __init__(self, path=IPM_YAML):
        self.p = yaml.safe_load(open(path))
        self.lat = float(self.p["lateral_meter_per_pixel"])       # 조감도 가로 1픽셀 = 몇 m
        self.lon = float(self.p["longitudinal_meter_per_pixel"])  # 조감도 세로 1픽셀 = 몇 m
        self.shape = None

    def setup(self, h, w):
        """사진 크기 (h, w) 에 맞게 준비. 크기가 바뀔 때만 계산한다.
        xf[row] = 차 중심선 위 그 줄의 앞 거리, yl[row, col] = 그 픽셀의 옆 거리 (경로 노드가 줄마다 쓰는 표)."""
        if self.shape == (h, w):
            return
        p = self.p
        self.src = np.float32(p["ipm_src_points"]).reshape(4, 2)
        self.src[:, 0] *= w / float(p["ipm_src_base_width"]); self.src[:, 1] *= h / float(p["ipm_src_base_height"])
        self.bev_h = int(round(h * float(p["bev_height_scale"])))
        l, r = w * p["ipm_dst_left_ratio"], w * p["ipm_dst_right_ratio"]
        self.M = cv2.getPerspectiveTransform(self.src, np.float32([[l, 0], [r, 0], [r, self.bev_h - 1], [l, self.bev_h - 1]]))
        self.Minv = np.linalg.inv(self.M)
        self.ego = (w - 1) * 0.5                                        # 차 중심선 (조감도 가운데)
        self.shape = (h, w)
        rows = np.arange(h)
        self.xf = self.to_m(np.full(h, self.ego), rows)[0]
        cols, rr = np.meshgrid(np.arange(w), rows)
        self.yl = self.to_m(cols.ravel(), rr.ravel())[1].reshape(h, w)

    def to_m(self, xs, ys):
        """사진 픽셀 (xs[i], ys[i]) → 바닥 (x m, y m) 배열 두 개 (float32)."""
        pts = cv2.perspectiveTransform(np.float32(np.c_[xs, ys]).reshape(-1, 1, 2), self.M).reshape(-1, 2)
        return (self.bev_h - 1 - pts[:, 1]) * self.lon, (self.ego - pts[:, 0]) * self.lat

    def to_bev(self, xs, ys):
        """바닥 (x m, y m) → 조감도 픽셀 (N, 2) float32."""
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        return np.stack([self.ego - ys / self.lat, self.bev_h - 1 - xs / self.lon], 1).astype(np.float32)

    def to_px(self, xs, ys):
        """바닥 (x m, y m) → 사진 픽셀 (N, 2) float32."""
        return cv2.perspectiveTransform(self.to_bev(xs, ys).reshape(-1, 1, 2), self.Minv).reshape(-1, 2)
