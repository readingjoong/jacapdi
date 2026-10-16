# -*- coding: utf-8 -*-
"""
=====================================================================================
 바닥 표식 보정 (calib_ground.py, 2026-10-16)  -  "사진 픽셀 → 차 기준 바닥 미터" 를 추측 없이
=====================================================================================

왜 필요한가?
   calib_pitch.py 는 f=560, h=0.607 을 고정하고 pitch 하나만 '차로 폭 0.85 m' 에 맞췄다.
   10/10 원본 사진 326 장으로 확인해 보니 차로 폭만으로는 f 와 pitch 를 따로 정할 수 없다
   (f 520~640 어느 값이든 pitch 만 바꾸면 폭 오차가 같음). 즉 앞뒤 거리는 맞는다는 보장이 없다.
   이 도구는 바닥에 붙인 테이프 표식(위치를 줄자로 잰 점)으로 변환 행렬 H 를 직접 푼다.
   f, pitch, roll, yaw, 카메라 높이를 가정하지 않는다.
   실측값(vehicle.yaml 의 렌즈 위치, pitch)은 H 에서 거꾸로 계산한 값과 비교하는 검증에만 쓴다.

좌표계 = vehicle.yaml:  원점 = 앞범퍼 아래 바닥, x 앞(+), y 왼쪽(+)

준비 (차는 세워 둔 채로, 카메라는 주행 때와 같은 상태)
   1) 평평한 바닥에 테이프 십자 표식 15 개를 아래 위치(m)에 붙인다. 줄자로 ±5 mm 안쪽.
         x = 0.30, 0.70, 1.10, 1.50, 1.90  (앞범퍼 끝에서 앞으로)
         y = +0.45 (왼쪽), 0 (차 폭 가운데), -0.45 (오른쪽)
      (시뮬레이션: 클릭이 1 px 틀려도 차로 경계 위치의 옆 오차 95% 가 5 mm 이내)
      다른 위치를 쓰려면 --points "x,y x,y ..." 로 준다 (4 점 이상, 넓게 퍼질수록 좋음).
   2) ./car_prep.sh 로 카메라가 켜진 상태에서:  python3 calib_ground.py
      (사진 파일이 이미 있으면:  python3 calib_ground.py --image 사진.png)

창 조작
   왼쪽 클릭 = 지금 점 찍기 (오른쪽 위 확대창으로 십자 가운데를 맞춘다)
   u = 직전 점 취소,  s = 이 점 건너뜀 (안 보일 때),  Enter = 계산,  q = 그만
결과
   ground_calib.yaml : H (픽셀 → 바닥 m), 점별 오차, H 로 거꾸로 구한 카메라 위치/각도/초점거리
   ground_calib_check.png : 구한 H 로 바닥 격자(0.2 m)와 차 중심선을 사진 위에 다시 그린 것
   주행 중인 파이프라인(ipm_c920.yaml)은 건드리지 않는다.

자체 시험 (가짜 카메라로 계산이 맞는지):  python3 calib_ground.py --selftest
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

D = os.path.expanduser("~/jacapdi-docker")
DEFAULT_POINTS = [(x, y) for x in (0.30, 0.70, 1.10, 1.50, 1.90) for y in (0.45, 0.0, -0.45)]   # 15 점


# ---------------------------------------------------------------------------------
# 1) 계산: 점 짝 → H,  H → 카메라 위치/각도
# ---------------------------------------------------------------------------------
def solve(img_pts, gnd_pts):
    """img_pts (N,2) 픽셀, gnd_pts (N,2) 바닥 m → H (픽셀 → 바닥), 점별 오차(m), 한 점 빼고 맞춘 오차(m)."""
    img_pts = np.float64(img_pts); gnd_pts = np.float64(gnd_pts)
    H, _ = cv2.findHomography(img_pts, gnd_pts, 0)
    err = np.linalg.norm(apply(H, img_pts) - gnd_pts, axis=1)
    loo = []                                         # 그 점을 빼고 구한 H 로 그 점을 예측 = 정직한 정확도
    if len(img_pts) >= 5:
        for i in range(len(img_pts)):
            k = np.arange(len(img_pts)) != i
            Hi, _ = cv2.findHomography(img_pts[k], gnd_pts[k], 0)
            loo.append(float(np.linalg.norm(apply(Hi, img_pts[i:i + 1])[0] - gnd_pts[i])))
    return H, err, np.array(loo)


def apply(H, pts):
    p = np.c_[np.float64(pts), np.ones(len(pts))] @ H.T
    return p[:, :2] / p[:, 2:3]


def decompose(H, w, h):
    """바닥 → 픽셀 행렬 G = H⁻¹ = K [r1 r2 t] 에서 초점거리 f, 카메라 위치, pitch/yaw/roll 을 꺼낸다.
    광학 중심은 사진 가운데라고 둔다 (검증용 값이라 이 가정은 결과 H 에는 영향 없음)."""
    G = np.linalg.inv(H)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    g1, g2 = G[:, 0], G[:, 1]
    a1 = g1[:2] - np.array([cx, cy]) * g1[2]; a2 = g2[:2] - np.array([cx, cy]) * g2[2]
    # u = 1/f² 에 대한 두 식: r1 ⟂ r2,  |r1| = |r2|  (yaw≈0 이면 첫 식은 0=0 이라 둘을 같이 푼다)
    A = np.array([a1 @ a2, a1 @ a1 - a2 @ a2]); b = np.array([g1[2] * g2[2], g1[2] ** 2 - g2[2] ** 2])
    u = -(A @ b) / (A @ A)
    if not np.isfinite(u) or u <= 0:
        return None
    f = float(1.0 / np.sqrt(u))
    Ki = np.array([[1 / f, 0, -cx / f], [0, 1 / f, -cy / f], [0, 0, 1]])
    B = Ki @ G
    lam = 2.0 / (np.linalg.norm(B[:, 0]) + np.linalg.norm(B[:, 1]))
    if B[2, 2] < 0:                                  # 바닥 원점이 카메라 앞(깊이 +)에 오도록 부호 선택
        lam = -lam
    r1, r2, t = lam * B[:, 0], lam * B[:, 1], lam * B[:, 2]
    R = np.c_[r1, r2, np.cross(r1, r2)]
    U, _, Vt = np.linalg.svd(R); R = U @ Vt          # 가장 가까운 회전 행렬로
    C = -R.T @ t                                     # 카메라 위치 (바닥 좌표, z 위 +)
    axis = R.T @ np.array([0, 0, 1.0])               # 광축 방향 (바닥 좌표)
    right = R.T @ np.array([1.0, 0, 0])              # 사진 오른쪽 방향
    return {"f_px": f, "cam_x_m": float(C[0]), "cam_y_m": float(C[1]), "cam_z_m": float(C[2]),
            "pitch_deg": float(np.degrees(np.arcsin(-axis[2]))),
            "yaw_deg": float(np.degrees(np.arctan2(axis[1], axis[0]))),     # + = 왼쪽을 봄
            "roll_deg": float(np.degrees(np.arcsin(right[2])))}             # + = 오른쪽이 올라감


# ---------------------------------------------------------------------------------
# 2) 사진 한 장 받기 (컨테이너의 /image_raw 에서)
# ---------------------------------------------------------------------------------
def grab(out):
    code = ("import rclpy,cv2,time;from rclpy.node import Node;from sensor_msgs.msg import Image;"
            "from cv_bridge import CvBridge;rclpy.init();n=Node('calib_ground_grab');b=CvBridge();g=[]\n"
            "n.create_subscription(Image,'/image_raw',lambda m:g.append(m),5)\n"
            "e=time.time()+3\nwhile time.time()<e and len(g)<10: rclpy.spin_once(n,timeout_sec=0.1)\n"
            "cv2.imwrite('%s',b.imgmsg_to_cv2(g[-1],'bgr8')) if g else exit(1)" % out)
    r = subprocess.run(["docker", "exec", "jacapdi-humble", "bash", "-ic", "python3 -c \"%s\"" % code],
                       capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(out):
        sys.exit("사진을 못 받음. car_prep.sh 로 카메라를 켰는지 확인 (또는 --image 사진.png)")


# ---------------------------------------------------------------------------------
# 3) 클릭 창
# ---------------------------------------------------------------------------------
def click_points(img, gnd):
    picked = [None] * len(gnd); i = [0]; mouse = [0, 0]
    win = "calib_ground  (click / u undo / s skip / Enter solve / q quit)"

    def on_mouse(ev, x, y, *_):
        mouse[:] = [x, y]
        if ev == cv2.EVENT_LBUTTONDOWN and i[0] < len(gnd):
            picked[i[0]] = (float(x), float(y)); i[0] += 1

    cv2.namedWindow(win); cv2.setMouseCallback(win, on_mouse)
    while True:
        vis = img.copy()
        for k, p in enumerate(picked):
            if p is not None:
                cv2.drawMarker(vis, (int(p[0]), int(p[1])), (0, 255, 255), cv2.MARKER_CROSS, 14, 1)
                cv2.putText(vis, "%d" % (k + 1), (int(p[0]) + 6, int(p[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        msg = ("%d/%d  x=%.2f y=%+.2f m" % (i[0] + 1, len(gnd), *gnd[i[0]])) if i[0] < len(gnd) else "Enter = solve"
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 24), (0, 0, 0), -1)
        cv2.putText(vis, msg, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        # 확대창 (커서 주변 40x40 → 160x160)
        x, y = mouse; r = 20
        pad = cv2.copyMakeBorder(img, r, r, r, r, cv2.BORDER_CONSTANT)
        z = cv2.resize(pad[y:y + 2 * r, x:x + 2 * r], (160, 160), interpolation=cv2.INTER_NEAREST)
        cv2.line(z, (80, 0), (80, 159), (0, 0, 255), 1); cv2.line(z, (0, 80), (159, 80), (0, 0, 255), 1)
        vis[28:188, vis.shape[1] - 164:vis.shape[1] - 4] = z
        cv2.imshow(win, vis)
        k = cv2.waitKey(20) & 0xFF
        if k == ord("q"):
            sys.exit("그만둠")
        if k == ord("u") and i[0] > 0:
            i[0] -= 1; picked[i[0]] = None
        if k == ord("s") and i[0] < len(gnd):
            i[0] += 1
        if k in (13, 10) and sum(p is not None for p in picked) >= 4:
            break
    cv2.destroyWindow(win)
    return picked


def draw_check(img, H, out):
    """구한 H 로 바닥 격자(0.2 m)와 차 중심선(y=0)을 사진 위에 다시 그린다. 테이프·차선과 겹치면 맞은 것."""
    G = np.linalg.inv(H); vis = img.copy()
    proj = lambda P: apply(G, P).astype(np.int32)
    for x in np.arange(0.2, 2.41, 0.2):
        cv2.polylines(vis, [proj(np.c_[np.full(41, x), np.linspace(-0.8, 0.8, 41)])], False, (0, 255, 255), 1)
    for y in np.arange(-0.8, 0.81, 0.2):
        c = (0, 0, 255) if abs(y) < 1e-6 else (0, 255, 255)
        cv2.polylines(vis, [proj(np.c_[np.linspace(0.1, 2.4, 47), np.full(47, y)])], False, c, 2 if c[2] == 255 else 1)
    cv2.imwrite(out, vis)


# ---------------------------------------------------------------------------------
# 4) 자체 시험: 실측값과 같은 가짜 카메라로 점을 만들고, 거꾸로 풀어서 같은 값이 나오는지
# ---------------------------------------------------------------------------------
def selftest():
    f, th, C, w, h = 575.0, np.radians(17.5), np.array([-0.544, 0.0, 0.58]), 640, 480
    gnd = np.array(DEFAULT_POINTS)
    def project(P):
        X = P[:, 0] - C[0]; Y = P[:, 1] - C[1]; Z = -C[2]
        xc = -Y; yc = -X * np.sin(th) - Z * np.cos(th); zc = X * np.cos(th) - Z * np.sin(th)
        return np.c_[(w - 1) / 2 + f * xc / zc, (h - 1) / 2 + f * yc / zc]
    rng = np.random.default_rng(1)
    for noise in (0.0, 1.0):
        img_pts = project(gnd) + rng.normal(0, noise, (len(gnd), 2))
        H, err, loo = solve(img_pts, gnd); d = decompose(H, w, h)
        print("픽셀 잡음 %.1f px: 점 오차 최대 %.1f mm, 한 점 빼고 최대 %.1f mm | f %.1f, 렌즈 (%.3f, %.3f, %.3f), pitch %.2f, yaw %.2f, roll %.2f"
              % (noise, 1000 * err.max(), 1000 * loo.max(), d["f_px"], d["cam_x_m"], d["cam_y_m"], d["cam_z_m"],
                 d["pitch_deg"], d["yaw_deg"], d["roll_deg"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image"); ap.add_argument("--points"); ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    gnd = [tuple(map(float, s.split(","))) for s in a.points.split()] if a.points else DEFAULT_POINTS
    path = a.image or os.path.join(D, "ground_calib_frame.png")
    if not a.image:
        grab(path)
    img = cv2.imread(path)
    picked = click_points(img, gnd)
    use = [k for k, p in enumerate(picked) if p is not None]
    ip = np.array([picked[k] for k in use]); gp = np.array([gnd[k] for k in use])
    H, err, loo = solve(ip, gp)
    d = decompose(H, img.shape[1], img.shape[0])
    veh = yaml.safe_load(open(os.path.join(D, "vehicle.yaml")))
    print("\n점별 오차 (맞춘 H 로 다시 계산한 바닥 위치 - 줄자 위치)")
    for j, k in enumerate(use):
        print("  x=%.2f y=%+.2f : %4.1f mm%s" % (*gnd[k], 1000 * err[j], ("   (한 점 빼고 맞추면 %.1f mm)" % (1000 * loo[j])) if len(loo) else ""))
    if d:
        lens = veh["camera"]["lens"]
        print("\nH 에서 거꾸로 구한 카메라 (실측과 비교)")
        print("  렌즈 x %.3f m (실측 %.3f)  y %+.3f m (실측 %+.3f)  높이 %.3f m (실측 %.3f)"
              % (d["cam_x_m"], lens[0], d["cam_y_m"], lens[1], d["cam_z_m"], lens[2]))
        print("  pitch %.2f° (실측 %.1f°)  yaw %+.2f°  roll %+.2f°  f %.0f px"
              % (d["pitch_deg"], veh["camera"]["pitch_deg"], d["yaw_deg"], d["roll_deg"], d["f_px"]))
    out = {"date": time.strftime("%F %T"), "image": path, "image_size": [img.shape[1], img.shape[0]],
           "frame": "vehicle.yaml (원점 = 앞범퍼 아래 바닥, x 앞, y 왼쪽, m)",
           "H_img_to_ground": [[float(v) for v in row] for row in H],
           "points": [{"ground": list(gnd[k]), "pixel": list(picked[k]), "err_mm": round(1000 * float(err[j]), 1)}
                      for j, k in enumerate(use)],
           "leave_one_out_max_mm": round(1000 * float(loo.max()), 1) if len(loo) else None,
           "decomposed": d}
    yaml.safe_dump(out, open(os.path.join(D, "ground_calib.yaml"), "w"), sort_keys=False, allow_unicode=True)
    draw_check(img, H, os.path.join(D, "ground_calib_check.png"))
    print("\n저장: ground_calib.yaml, ground_calib_check.png (격자가 테이프·차선과 겹치는지 눈으로 확인)")


if __name__ == "__main__":
    main()
