# -*- coding: utf-8 -*-
"""
=====================================================================================
 바닥 변환 모델 (ground_model.py, 2026-10-16)  -  사진 픽셀 → 차 기준 바닥 좌표 (실측 기반)
=====================================================================================

좌표계 = vehicle.yaml:  원점 = 앞범퍼 아래 바닥, x 앞(+), y 왼쪽(+), 단위 m
   앞바퀴 축 x = -0.23, 뒷바퀴 축 x = -0.79, 렌즈 (-0.544, 0, 0.58)

값의 출처
   렌즈 위치, 높이 0.58 m, pitch 17.5°, 좌우 가운데, 정면(yaw 0) : 실측 (vehicle.yaml)
   초점거리 f = 540.4 px : 10/10 주행 원본 사진 326 장(직선 구간)에서 "차로 폭 0.85 m" 로 푼 값 (부트스트랩 ±0.3 px).
                          이 f 로 렌즈 앞 0.6~1.8 m 전 구간의 차로 폭이 0.850~0.854 m 로 일정 → 실측 pitch/높이와 모순 없음.
   광학 중심 (320, 240) : 사진 가운데로 둠. 좌우로 어긋난 만큼은 아래 yaw 에 함께 들어간다.
   yaw (카메라가 차체보다 왼쪽을 보는 각도) : 10/16 정지 시험 (static_check.py). 직선 차로에 차를 평행하게 세움
                         (왼쪽 앞/뒷바퀴 → 점선 둘 다 17 cm) → 두 경계선이 똑같이 -4.96° 로 보임 = 카메라가 4.96° 왼쪽.
                         주행 기록으로는 안 보인다 (제어기가 카메라 화면에서 차선을 똑바로 만들기 때문).
   렌즈 왜곡 : 무시 (calib_intrinsics.py 로 재면 반영 가능)

사용
   from ground_model import GroundModel
   g = GroundModel();  x, y = g.to_ground(u, v)      # 픽셀 배열 → 바닥 m (지평선 위 픽셀은 nan)
   python3 ground_model.py   → 지금 파이프라인(ipm_c920.yaml) 변환과 비교표
"""
import os

import numpy as np
import yaml

D = os.path.expanduser("~/jacapdi-docker")


class GroundModel:
    def __init__(self, f=540.4, cx=320.0, cy=240.0, vehicle_yaml=os.path.join(D, "vehicle.yaml")):
        cam = yaml.safe_load(open(vehicle_yaml))["camera"]
        self.f, self.cx, self.cy = f, cx, cy
        self.lx, self.ly, self.h = cam["lens"]
        self.th = np.radians(cam["pitch_deg"])
        self.psi = np.radians(cam.get("yaw_deg", 0.0))       # + = 카메라가 왼쪽을 봄
        self.cs, self.sn = np.cos(self.psi), np.sin(self.psi)

    def to_ground(self, u, v):
        """픽셀 (u 오른쪽, v 아래) → 바닥 (x 앞, y 왼쪽). 광선이 바닥에 안 닿으면 nan."""
        a = (np.asarray(u, float) - self.cx) / self.f
        b = (np.asarray(v, float) - self.cy) / self.f
        fwd = np.cos(self.th) - np.sin(self.th) * b          # 광선 방향: 앞
        down = np.sin(self.th) + np.cos(self.th) * b         # 광선 방향: 아래
        t = np.where(down > 1e-6, self.h / np.where(down > 1e-6, down, 1.0), np.nan)
        xc, yc = t * fwd, -t * a                             # 카메라가 보는 방향 기준 (렌즈 아래 바닥 원점)
        return self.lx + self.cs * xc - self.sn * yc, self.ly + self.sn * xc + self.cs * yc   # 차체 기준으로 yaw 만큼 돌림

    def to_pixel(self, x, y):
        """바닥 (x, y) → 픽셀 (u, v)."""
        dx = np.asarray(x, float) - self.lx; dy = np.asarray(y, float) - self.ly
        X = self.cs * dx + self.sn * dy; Y = -self.sn * dx + self.cs * dy   # 차체 → 카메라 방향 기준
        zc = X * np.cos(self.th) + self.h * np.sin(self.th)
        yc = -X * np.sin(self.th) + self.h * np.cos(self.th)
        return self.cx - self.f * Y / zc, self.cy + self.f * yc / zc


def compare_with_ipm():
    """ipm_c920.yaml (지금 주행 중인 변환) 이 같은 바닥 점을 어디로 보는지."""
    import cv2
    ip = yaml.safe_load(open(os.path.join(D, "ipm_c920.yaml")))["lane_info_extractor_node"]["ros__parameters"]
    w, h = 640, 480
    src = np.float32(ip["ipm_src_points"]).reshape(4, 2)
    bev_h = int(round(h * float(ip["bev_height_scale"])))
    dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                      [w * ip["ipm_dst_right_ratio"], bev_h - 1], [w * ip["ipm_dst_left_ratio"], bev_h - 1]])
    M = cv2.getPerspectiveTransform(src, dst)
    lat, lon, ego = float(ip["lateral_meter_per_pixel"]), float(ip["longitudinal_meter_per_pixel"]), (w - 1) * 0.5
    x_near_lens = float(open(os.path.join(D, "ipm_c920.yaml")).readline().split("전방 ")[1].split("~")[0])   # 헤더의 0.84
    g = GroundModel()
    print("실제 바닥 점(앞범퍼 기준)  →  지금 파이프라인이 보는 위치 (같은 좌표계로 바꿔서)")
    print("   x      y   |  ipm x    ipm y  |  앞뒤 오차  옆 오차")
    for x in (0.3, 0.7, 1.1, 1.5):
        for y in (0.425, -0.425):
            u, v = g.to_pixel(x, y)
            p = cv2.perspectiveTransform(np.float32([[[u, v]]]), M)[0, 0]
            xi = (bev_h - 1 - p[1]) * lon + x_near_lens + g.lx   # 파이프라인 x(=0.84 m 지점부터) → 앞범퍼 기준
            yi = (ego - p[0]) * lat
            print("  %.2f %+.3f  |  %.3f  %+.3f  |  %+5.1f cm  %+5.1f cm" % (x, y, xi, yi, 100 * (xi - x), 100 * (yi - y)))
    print("\n파이프라인 x=0 (경로 c 를 재는 곳) = 앞범퍼 %.2f m 앞 = 앞바퀴 축 %.2f m 앞" % (x_near_lens + g.lx, x_near_lens + g.lx + 0.23))
    print("Stanley 평가 지점 (x=0.1)        = 앞바퀴 축 %.2f m 앞" % (x_near_lens + g.lx + 0.23 + 0.1))


if __name__ == "__main__":
    compare_with_ipm()
