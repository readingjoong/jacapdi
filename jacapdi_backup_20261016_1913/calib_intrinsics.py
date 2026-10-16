# -*- coding: utf-8 -*-
"""
=====================================================================================
 카메라 내부 보정 (calib_intrinsics.py, 2026-10-16)  -  초점거리 f, 광학 중심, 렌즈 왜곡을 실측
=====================================================================================

왜 필요한가?
   차로 폭만으로는 f 와 pitch 를 따로 정할 수 없다 (10/10 사진으로 확인). f 를 재 두면
   높이(실측 0.58 m) + f 로 pitch 는 주행 사진에서 저절로 풀린다. 바닥 테이프가 필요 없다.

준비 (1 분)
   1) python3 calib_intrinsics.py --make-board   → checkerboard.png (칸 10 x 7, 안쪽 모서리 9 x 6)
   2) 그 그림을 폰 화면에 꽉 차게 띄우고(밝기 최대, 자동 회전 끔), 줄자로 칸 한 변 길이(mm)를 잰다.
      (A4 에 인쇄해서 평평한 판에 붙여도 된다)
실행 (2~3 분)  - 카메라를 직접 열므로 파이프라인은 꺼 둔다 (./car_stop.sh 또는 car_prep 전)
   python3 calib_intrinsics.py --square-mm 12.5
   폰을 카메라 앞 30~80 cm 에서 화면 여기저기(가운데, 네 귀퉁이)로 옮기고 이리저리 기울인다.
   체커보드가 잡히고 앞서 찍은 자세와 충분히 다르면 자동으로 찍힌다 (초록 깜빡임). 25 장이면 끝.
   q = 그만 (15 장 이상이면 그때까지로 계산)
결과
   camera_intrinsics.yaml : K (fx, fy, cx, cy), 왜곡 계수, 재투영 오차 (0.5 px 아래면 좋음)
"""
import argparse
import os
import time

import cv2
import numpy as np
import yaml

D = os.path.expanduser("~/jacapdi-docker")
COLS, ROWS = 9, 6          # 안쪽 모서리 개수


def make_board(path):
    sq = 120
    img = np.full(((ROWS + 1) * sq + 2 * sq, (COLS + 1) * sq + 2 * sq), 255, np.uint8)   # 둘레 흰 여백 1 칸
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            if (r + c) % 2 == 0:
                img[sq + r * sq:sq + (r + 1) * sq, sq + c * sq:sq + (c + 1) * sq] = 0
    cv2.imwrite(path, img)
    print("저장:", path, "(폰에 띄우고 칸 한 변을 mm 로 재서 --square-mm 로 준다)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-board", action="store_true")
    ap.add_argument("--square-mm", type=float)
    ap.add_argument("--dev", default="/dev/video4")
    ap.add_argument("--n", type=int, default=25)
    a = ap.parse_args()
    if a.make_board:
        return make_board(os.path.join(D, "checkerboard.png"))
    if not a.square_mm:
        raise SystemExit("--square-mm 필요 (폰 화면 위 칸 한 변 길이)")

    cap = cv2.VideoCapture(a.dev, cv2.CAP_V4L2)          # 주행 때와 같은 설정 (webcam_pub.py)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit("카메라 못 엶: %s (꽂혀 있는지, 파이프라인이 잡고 있지 않은지 확인)" % a.dev)
    obj = np.zeros((COLS * ROWS, 3), np.float32)
    obj[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * a.square_mm / 1000.0
    objs, imgs, poses = [], [], []
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    flash = 0; size = None
    while len(imgs) < a.n:
        ok, f = cap.read()
        if not ok:
            continue
        size = f.shape[1::-1]
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        found, c = cv2.findChessboardCorners(g, (COLS, ROWS), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        vis = f.copy()
        if found:
            c = cv2.cornerSubPix(g, c, (5, 5), (-1, -1), crit)
            cv2.drawChessboardCorners(vis, (COLS, ROWS), c, found)
            # 자세 요약 = (중심 x, 중심 y, 크기, 기울기 2개). 앞서 찍은 것과 충분히 다를 때만 찍는다
            p = c.reshape(-1, 2)
            q = np.r_[p.mean(0) / size, np.ptp(p, 0) / size,
                      np.linalg.norm(p[COLS - 1] - p[0]) / max(np.linalg.norm(p[-1] - p[-COLS]), 1e-6)]
            if not poses or min(np.abs(np.array(poses) - q).max(1)) > 0.08:
                objs.append(obj); imgs.append(c); poses.append(q); flash = 4
        if flash:
            cv2.rectangle(vis, (0, 0), (size[0] - 1, size[1] - 1), (0, 255, 0), 8); flash -= 1
        cv2.putText(vis, "%d/%d  (q = stop)" % (len(imgs), a.n), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("calib_intrinsics", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release(); cv2.destroyAllWindows()
    if len(imgs) < 15:
        raise SystemExit("사진 %d 장뿐. 15 장 이상 필요" % len(imgs))

    rms, K, dist, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None)
    # 정직한 정확도: 사진을 반씩 나눠 따로 맞췄을 때 f 가 얼마나 다른가
    h1 = cv2.calibrateCamera(objs[0::2], imgs[0::2], size, None, None)[1]
    h2 = cv2.calibrateCamera(objs[1::2], imgs[1::2], size, None, None)[1]
    out = {"date": time.strftime("%F %T"), "device": a.dev, "image_size": list(size), "n_images": len(imgs),
           "square_mm": a.square_mm, "rms_px": round(float(rms), 3),
           "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
           "dist_k1_k2_p1_p2_k3": [float(v) for v in dist.ravel()],
           "split_half_fx": [round(float(h1[0, 0]), 1), round(float(h2[0, 0]), 1)]}
    yaml.safe_dump(out, open(os.path.join(D, "camera_intrinsics.yaml"), "w"), sort_keys=False)
    print("재투영 오차 %.3f px (%d 장)" % (rms, len(imgs)))
    print("fx %.1f  fy %.1f  cx %.1f  cy %.1f   (반씩 나눠 맞춘 fx: %.1f / %.1f)"
          % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], h1[0, 0], h2[0, 0]))
    print("왜곡", np.round(dist.ravel(), 4), "\n저장: camera_intrinsics.yaml")


if __name__ == "__main__":
    main()
