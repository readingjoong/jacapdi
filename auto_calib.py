# -*- coding: utf-8 -*-
"""C920 IPM 자동 보정 (컨테이너 안에서, 파이프라인 실행 중에).

조건: 카메라 고정, 차는 직선 구간 차로 안(가운데일 필요 없음), 왼쪽 dashed_line + 오른쪽 solid_line 이 보임.
절차: /image_raw + /detections 30프레임 수집 -> 각 프레임 안쪽 가장자리 직선 적합
      -> 카메라 모델(f 후보 x 높이/pitch/yaw/차로 위치) 최소제곱 -> 점선 주기 1.0m 에 가장 맞는 f 선택
      -> 차량 기준 IPM (전방 X_near~X_far, 좌우 +-0.8m) -> 검증(BEV 차로폭) -> ipm_c920.yaml 저장.
검증 실패(선 부족, 적합 오차 큼, 차로폭 0.85m 에서 15% 이상 벗어남)면 yaml 을 바꾸지 않고 종료 코드 1.
"""
import json, os, sys, time
import cv2, numpy as np, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.optimize import least_squares
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray

LANE, PERIOD, HALF, X_FAR = 0.85, 1.0, 0.8, 2.0
OUT = os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")
W, H, CX, CY = 640, 480, 320.0, 240.0

rclpy.init(); n = Node("auto_calib"); br = CvBridge()
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
imgs, dets = {}, []
n.create_subscription(Image, "/image_raw", lambda m: imgs.__setitem__(st(m.header), br.imgmsg_to_cv2(m, "bgr8")), 30)
n.create_subscription(DetectionArray, "/detections", lambda m: dets.append(m), q)
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)

def masks(d):
    md = np.zeros((H, W), np.uint8); ms = np.zeros((H, W), np.uint8)
    for x in d.detections:
        if x.mask.data and x.class_name in ("dashed_line", "solid_line"):
            cv2.fillPoly(md if x.class_name == "dashed_line" else ms, [np.array([[p.x, p.y] for p in x.mask.data], np.int32)], 255)
    return md, ms

# 여러 프레임의 안쪽 가장자리 점 모으기 (점선: 화면 왼쪽 절반의 오른쪽 끝, 실선: 오른쪽 절반의 왼쪽 끝)
lp, rp, dash_ends, frame = [], [], [], None
for d in dets[-30:]:
    md, ms = masks(d)
    for y in range(int(H * 0.35), H):
        x = np.flatnonzero(md[y]); x = x[x < W * 0.6]
        if x.size: lp.append((y, x.max()))
        x = np.flatnonzero(ms[y]); x = x[x > W * 0.4]
        if x.size: rp.append((y, x.min()))
    if frame is None and st(d.header) in imgs: frame = imgs[st(d.header)]
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(md)
    for i in range(1, nlab):
        ys = np.flatnonzero(lab.any(axis=1) & (lab == i).any(axis=1))
        if stats[i, cv2.CC_STAT_HEIGHT] > 8: dash_ends.append((ys.max(), ys.min()))
lp, rp = np.array(lp, float), np.array(rp, float)
if len(lp) < 80 or len(rp) < 80:
    print("FAIL: 선 점 부족 (dashed %d, solid %d). 양쪽 선이 화면 아래 절반에 보여야 함" % (len(lp), len(rp))); sys.exit(1)
fl = np.polyfit(lp[:, 0], lp[:, 1], 1); fr = np.polyfit(rp[:, 0], rp[:, 1], 1)
res_l = np.std(lp[:, 1] - np.polyval(fl, lp[:, 0])); res_r = np.std(rp[:, 1] - np.polyval(fr, rp[:, 0]))
print("edge fit: left x=%.3fy+%.1f (%.1fpx), right x=%.3fy+%.1f (%.1fpx)" % (fl[0], fl[1], res_l, fr[0], fr[1], res_r))
if res_l > 8 or res_r > 8:
    print("FAIL: 가장자리가 직선이 아님 (커브?). 직선 구간에서 다시"); sys.exit(1)
ys_fit = np.linspace(max(lp[:, 0].min(), rp[:, 0].min()), H - 1, 20)

def proj(X, Y, h, th, f):
    xc = -Y; yc = -X * np.sin(th) + h * np.cos(th); zc = X * np.cos(th) + h * np.sin(th)
    return CX + f * xc / zc, CY + f * yc / zc
def row_to_X(v, Yfun, h, th, f):
    Xs = np.linspace(0.02, 12, 6000); u, vv = proj(Xs, Yfun(Xs), h, th, f)
    i = np.argmin(np.abs(vv - v)); return Xs[i], u[i]
def resid(qq, f):
    h, th, psi, yl = qq; r = []
    for fit, off in ((fl, yl), (fr, yl - LANE / np.cos(psi))):
        for v in ys_fit:
            _, u = row_to_X(v, lambda X: off + X * np.tan(psi), h, th, f); r.append(u - np.polyval(fit, v))
    return np.array(r)
best = None
for f in np.arange(460, 720, 20.0):
    r = least_squares(resid, [0.4, np.deg2rad(20), 0.0, 0.4], args=(f,), bounds=([0.05, 0.0, -0.5, 0.0], [1.5, 1.3, 0.5, 0.85]))
    h, th, psi, yl = r.x; rms = np.sqrt(np.mean(r.fun ** 2))
    # 점선 주기 검증: 점선 조각 아래끝 사이 거리의 대표값
    bots = sorted({int(b) for b, t in dash_ends}, reverse=True)
    Xb = sorted({round(row_to_X(b, lambda X: yl + X * np.tan(psi), h, th, f)[0], 2) for b in bots})
    gaps = [b - a for a, b in zip(Xb, Xb[1:]) if 0.6 < b - a < 1.6]
    per = float(np.median(gaps)) if gaps else float("nan")
    score = rms + (0 if np.isnan(per) else 20 * abs(per - PERIOD))
    print("f=%3d h=%.2f pitch=%.1f yaw=%.1f rms=%.1fpx period=%.2f" % (f, h, np.rad2deg(th), np.rad2deg(psi), rms, per))
    if rms < 4 and (best is None or score < best[0]): best = (score, f, h, th, psi, yl, rms, per)
if best is None:
    print("FAIL: 카메라 모델 적합 실패"); sys.exit(1)
_, f, h, th, psi, yl, rms, per = best
horizon = CY - f * np.tan(th)
near_row = min(H - 5, int(max(lp[:, 0].max(), rp[:, 0].max())))
X_near = round(float(row_to_X(near_row, lambda X: 0 * X, h, th, f)[0]) + 0.02, 2)
x_far = min(X_FAR, X_near + 1.5)
pts = [(x_far, HALF), (x_far, -HALF), (X_near, -HALF), (X_near, HALF)]
src = [float(round(c, 1)) for X, Y in pts for c in proj(X, Y, h, th, f)]
bev_h = H * 4
lat = 2 * HALF / (0.4 * W); lon = (x_far - X_near) / (bev_h - 1)
# 검증: BEV 에서 차로 폭
md, ms = masks(dets[-1])
M = cv2.getPerspectiveTransform(np.float32(src).reshape(4, 2), np.float32([[192, 0], [448, 0], [448, bev_h - 1], [192, bev_h - 1]]))
bl = cv2.warpPerspective(md, M, (W, bev_h), flags=cv2.INTER_NEAREST); brr = cv2.warpPerspective(ms, M, (W, bev_h), flags=cv2.INTER_NEAREST)
widths = []
for y in range(0, bev_h, 8):
    a = np.flatnonzero(bl[y]); b = np.flatnonzero(brr[y])
    if a.size and b.size and b.min() > a.max(): widths.append((b.min() - a.max()) * lat)
wmed = float(np.median(widths)) if widths else float("nan")
print("chosen f=%d h=%.3f pitch=%.1fdeg yaw=%.1fdeg horizon_y=%.0f rms=%.1fpx period=%.2fm | IPM X %.2f..%.2f m | BEV lane width %.2f m (spec %.2f, rows %d)"
      % (f, h, np.rad2deg(th), np.rad2deg(psi), horizon, rms, per, X_near, x_far, wmed, LANE, len(widths)))
if not widths or abs(wmed - LANE) / LANE > 0.15:
    print("FAIL: BEV 차로폭 검증 실패, yaml 변경 안 함"); sys.exit(1)
params = {"use_sim_time": False, "sub_detection_topic": "/lane_detections", "drivable_class_name": "lane",
          "ipm_src_points": src, "ipm_src_base_width": 640.0, "ipm_src_base_height": 480.0,
          "ipm_dst_left_ratio": 0.3, "ipm_dst_right_ratio": 0.7, "bev_height_scale": 4.0,
          "lateral_meter_per_pixel": round(lat, 6), "longitudinal_meter_per_pixel": round(lon, 7), "ego_x_px": -1.0}
hdr = ("# auto_calib.py %s: f=%d h=%.3f pitch=%.1f yaw=%.1f rms=%.1fpx period=%.2fm, BEV lane width %.2fm\n"
       "# 차량 기준 전방 %.2f~%.2fm, 좌우 +-%.1fm. x_forward 0 = 차량 전방 %.2fm. 카메라를 움직이면 다시 실행.\n"
       % (time.strftime("%F %T"), f, h, np.rad2deg(th), np.rad2deg(psi), rms, per, wmed, X_near, x_far, HALF, X_near))
if os.path.isfile(OUT): os.replace(OUT, OUT + ".prev")
open(OUT, "w").write(hdr + yaml.safe_dump({"lane_info_extractor_node": {"ros__parameters": params}}, sort_keys=False))
if frame is not None:
    vis = frame.copy(); cv2.polylines(vis, [np.float32(src).reshape(4, 2).astype(np.int32)], True, (0, 255, 255), 2)
    cv2.imwrite(os.path.expanduser("~/jacapdi-docker/ipm_c920_check.jpg"), vis)
print("OK: saved", OUT)
