# -*- coding: utf-8 -*-
"""카메라를 다시 달았을 때 IPM(config/ipm_c920.yaml) 재보정. 토픽은 구독만 한다 (publish 없음).
차를 직선 구간 2차선 가운데에 두고 car_prep.sh 만 켠 상태(출발 X)에서:
   ~/jacapdi-docker/car.sh shell python3 ~/jacapdi-docker/nodes/calib_ipm.py <렌즈 높이 m> [--write]
(렌즈 높이 = 바닥 ~ 렌즈 중심. 10/16 실측 0.58 m)
카메라 모델 (초점 f, 렌즈 높이 h, 아래로 숙인 각 pitch) 에서 f 는 렌즈 고유값이라 그대로 두고, h 는 실측,
pitch 는 "왼쪽 점선 안쪽 끝 ~ 오른쪽 실선 안쪽 끝" 폭이 0.85 m 가 되는 값으로 푼다.
바닥 구간은 예전과 같이 카메라 바로 아래에서 앞 0.84~1.40 m, 좌우 ±0.8 m (경로 x = 0 위치와 미터/픽셀이 그대로 →
경로 노드·제어기의 거리 설정을 그대로 쓸 수 있음). 화면 밖 점이 나와도 변환 정의용이라 괜찮다.
--write 가 없으면 결과와 확인 사진(config/ipm_check.png)만. 있으면 기존 yaml 을 .prev 로 두고 새로 쓴다."""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray

F = 540.4                  # 초점 거리 px (640x480). 10/16 실측 높이 0.58 m·pitch 17.5° 에서 차로 폭으로 푼 값
CX, CY, W, H = 320.0, 240.0, 640, 480
LANE = 0.85                # 차로 폭 m
X_NEAR, X_FAR, HALF = 0.84, 1.40, 0.8   # 보정 바닥 구간 (카메라 바로 아래 기준 m). 예전과 같음
CFG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")


def ground(u, v, h, th):
    """사진 (u, v) → 바닥 (앞 X, 왼쪽 Y) m (카메라 바로 아래 기준). 지평선 위는 nan."""
    a, b = (u - CX) / F, (v - CY) / F
    zd = -(np.sin(th) + np.cos(th) * b)
    t = np.where(zd < 0, h / -np.where(zd < 0, zd, -1), np.nan)
    return t * (np.cos(th) - np.sin(th) * b), -t * a


def project(X, Y, h, th):
    """바닥 (X, Y) → 사진 (u, v)."""
    zc = X * np.cos(th) + h * np.sin(th)
    return CX + F * -Y / zc, CY + F * (-X * np.sin(th) + h * np.cos(th)) / zc


def main():
    h = float(sys.argv[1])
    rclpy.init(); n = Node("calib_ipm"); dets, imgs = [], []
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
    n.create_subscription(DetectionArray, "/detections", dets.append, q)
    n.create_subscription(Image, "/image_raw", imgs.append, 10)
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    rows = []                                    # (줄, 점선 안쪽 끝 열, 실선 안쪽 끝 열)
    for d in dets[-60:]:
        md = np.zeros((H, W), np.uint8); ms = np.zeros((H, W), np.uint8)
        for x in d.detections:
            if x.mask.data and x.class_name in ("dashed_line", "solid_line") and x.score >= 0.3:
                cv2.fillPoly(md if x.class_name == "dashed_line" else ms, [np.array([[p.x, p.y] for p in x.mask.data], np.int32)], 255)
        for y in range(H // 3, H):
            a = np.flatnonzero(md[y]); b = np.flatnonzero(ms[y])
            b = b[b > (a.max() if a.size else W)]
            if a.size and b.size and b.min() > a.max() + 20:
                rows.append((y, a.max(), b.min()))
    print("검출 %d 장, 양쪽 선이 같이 보인 줄 %d 개" % (len(dets), len(rows)))
    if len(rows) < 30:
        sys.exit("실패: 왼쪽 점선과 오른쪽 실선이 같이 보이는 줄이 부족. 차를 직선 2차선 가운데에 두고 다시")
    R = np.array(rows, float)

    def width(th):
        _, yl = ground(R[:, 1], R[:, 0], h, th); _, yr = ground(R[:, 2], R[:, 0], h, th)
        return yl - yr
    ths = np.deg2rad(np.arange(0.0, 45.0, 0.05))
    med = np.array([np.nanmedian(width(t)) for t in ths])
    th = ths[np.nanargmin(np.abs(med - LANE))]
    wd = width(th)
    print("pitch %.2f° (f %.1f, h %.3f): 차로 폭 중앙값 %.3f m" % (np.rad2deg(th), F, h, np.nanmedian(wd)))
    X, _ = ground(R[:, 1], R[:, 0], h, th)
    _, yl = ground(R[:, 1], R[:, 0], h, th); _, yr = ground(R[:, 2], R[:, 0], h, th)
    mid = (yl + yr) / 2                          # 차로 가운데 (+ = 카메라보다 왼쪽)
    for lo, hi in ((0, 1.2), (1.2, 1.6), (1.6, 2.2), (2.2, 3.0), (3.0, 9)):   # 거리마다 폭이 같아야 f·h·pitch 가 서로 맞는 것
        m = (X >= lo) & (X < hi)
        if m.sum() >= 5:
            print("   카메라 앞 %.1f~%.1f m: 폭 %.3f m, 차로 가운데 %+.3f m (%d 줄)" % (lo, hi, np.median(wd[m]), np.median(mid[m]), m.sum()))
    ok = np.isfinite(mid) & np.isfinite(X)
    k, c0 = np.polyfit(X[ok], mid[ok], 1)        # 가운데 = c0 + k·X. 거리에 따라 커지면 카메라가 돌아간 것, 일정하면 차가 비켜 선 것
    print("차로 가운데 = %+.3f m %+.3f·거리 → 카메라가 차로보다 %s 으로 %.1f° 돌아가 있고, 카메라 바로 아래에서 차로 가운데가 %s 으로 %.3f m"
          % (c0, k, "왼쪽" if k < 0 else "오른쪽", abs(np.degrees(np.arctan(k))), "왼쪽" if c0 > 0 else "오른쪽", abs(c0)))
    print("   (차를 차로와 나란히, 가운데에 뒀다면 둘 다 0 근처여야 함. 각도가 크면 카메라 좌우 방향을 고칠 것)")
    bottom = ground(np.array([CX]), np.array([H - 1.0]), h, th)[0][0]
    print("사진 맨 아래 줄 = 카메라 앞 %.2f m (경로 x = %.2f m). 보닛에 가린 줄은 더 앞" % (bottom, bottom - X_NEAR))

    src = [float(round(c, 1)) for X_, Y_ in ((X_FAR, HALF), (X_FAR, -HALF), (X_NEAR, -HALF), (X_NEAR, HALF)) for c in project(X_, Y_, h, th)]
    print("ipm_src_points:", src)
    if imgs:                                     # 확인 사진: 고른 끝점, 경로 x = 0 / 0.45 / 0.7 m 줄, 풀어낸 차로 경계
        m = imgs[-1]
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3].copy()
        if m.encoding == "rgb8":
            img = img[:, :, ::-1].copy()
        for y, a, b in rows[-400:]:
            img[int(y), int(a)] = (0, 255, 255); img[int(y), int(b)] = (255, 0, 255)
        ys = np.linspace(-1.0, 1.0, 50)
        for x in (0.0, 0.45, 0.7):
            u, v = project(X_NEAR + x + 0 * ys, ys, h, th)
            cv2.polylines(img, [np.int32(np.c_[u, v])], False, (0, 255, 0), 1)
            cv2.putText(img, "x=%.2f" % x, (int(u[-1]) + 5, int(v[-1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        xs = np.linspace(X_NEAR - 0.5, X_NEAR + 1.5, 50)
        for y0 in (np.nanmedian(yl), np.nanmedian(yr)):
            u, v = project(xs, y0 + 0 * xs, h, th)
            cv2.polylines(img, [np.int32(np.c_[u, v])], False, (255, 255, 255), 1)
        cv2.imwrite(os.path.join(CFG, "ipm_check.png"), img)
        print("확인 사진:", os.path.join(CFG, "ipm_check.png"), "(흰 선 = 풀어낸 차로 경계, 초록 = 경로 x 거리)")
    if "--write" not in sys.argv:
        print("yaml 은 그대로 (쓰려면 --write)")
        return
    path = os.path.join(CFG, "ipm_c920.yaml")
    if os.path.isfile(path):
        os.replace(path, path + ".prev")
    with open(path, "w") as f:
        f.write("# 카메라(C920) 사진 픽셀 → 바닥 미터 변환 (IPM). 읽는 곳: nodes/ipm.py\n"
                "# %s calib_ipm.py: f=%.1f h=%.3f pitch=%.2fdeg (rows %d, lane width %.3fm). 전방 %.2f~%.2fm, 좌우 +-%.1fm.\n"
                % (time.strftime("%F %T"), F, h, np.rad2deg(th), len(rows), np.nanmedian(wd), X_NEAR, X_FAR, HALF))
        f.write("ipm_src_points: [%s]   # 사진 속 바닥 사다리꼴 4점 (640x480 기준)\n" % ", ".join("%.1f" % c for c in src))
        f.write("ipm_src_base_width: 640.0\nipm_src_base_height: 480.0\nipm_dst_left_ratio: 0.3\nipm_dst_right_ratio: 0.7\n"
                "bev_height_scale: 4.0\nlateral_meter_per_pixel: %.6f\nlongitudinal_meter_per_pixel: %.7f\n"
                % (2 * HALF / 256, (X_FAR - X_NEAR) / 1919))
    print("썼음:", path, "(이전 것은 .prev). 파이프라인을 다시 켜야 반영됨")


if __name__ == "__main__":
    main()
