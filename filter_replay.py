# -*- coding: utf-8 -*-
"""
=====================================================================================
 기록 재생: 고스트 + 기억 지도 (filter_replay.py, 2026-10-16)
=====================================================================================

frames.csv (10 Hz: 조향 ADC) 로 고스트를 움직이고, 새 사진이 올 때마다 YOLO 경계 점을 기억 지도에 쌓는다.
앞바퀴 축 기준 옆 오차 / 방향을 '기억한 점' 으로 구해서 사진 한 장 방식(ghost_view.py, 접선으로 늘림)과 같이 그린다.

차 움직임 값 (vehicle.yaml 의 motion)
   ADC → 조향각: 가운데 462 (펌웨어 실측), 왼쪽 0.285°/칸, 오른쪽 0.304°/칸 (10/16 회전 원 실측)
   실제 회전 = 언더스티어 표(조향 3°: 0.82, 9.5°: 0.48, 18°: 0.45) × v·tanδ/L : 10/10 사진 쌍을 겹쳐 맞춘 값
   속도 0.71 m/s : 같은 사진 쌍에서 (PWM 255)

사용:  python3 filter_replay.py <dets.jsonl (연속 프레임)> <frames.csv> <출력폴더>
"""
import csv
import json
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ghost_view import boundaries, fit, lane_center, near_x, bev, BW, BH, g, L as VL, FRONT_AXLE, REAR_AXLE   # noqa: E402
from lane_map import LaneMap   # noqa: E402
from vehicle_motion import delta_of_adc, effective_delta   # noqa: E402

D = os.path.dirname(os.path.abspath(__file__))
M = yaml.safe_load(open(os.path.join(D, "vehicle.yaml")))["motion"]
WB = 0.56; FA = FRONT_AXLE - REAR_AXLE            # 앞바퀴 축 = 뒷바퀴 축 기준 0.56 m


def main():
    dets_path, csv_path, out = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    rows = list(csv.DictReader(open(csv_path)))
    tw = np.array([float(r["wall_time"]) for r in rows]); adc = np.array([float(r["board_adc"] or M["adc_center"]) for r in rows])
    active = np.array([r["board_state"] == "ACTIVE" for r in rows])
    by_file = {r["file"]: r for r in rows}
    imgs = []
    for line in open(dets_path):
        if line.startswith("{"):
            d = json.loads(line); r = by_file.get(os.path.basename(d["f"]))
            if r: imgs.append((float(r["img_stamp"]), d, r))
    imgs.sort(key=lambda x: x[0])
    mp = LaneMap(wheelbase=WB); t_now = None; last_stamp = None; out_rows = []
    for k, (ts, d, r) in enumerate(imgs):
        if ts == last_stamp:
            continue                                   # 같은 사진이 여러 줄 기록된 것
        last_stamp = ts
        # 고스트를 사진 시각까지 움직인다 (10 Hz ADC, 멈춤/IDLE 이면 안 움직임)
        if t_now is None:
            t_now = ts
        while t_now < ts:
            i = max(0, np.searchsorted(tw, t_now) - 1); step = min(0.02, ts - t_now)
            v = M["speed_mps"] if active[i] else 0.0
            mp.move(v, effective_delta(delta_of_adc(adc[i])), step)
            t_now += step
        Dp, Sp = boundaries(d["d"], d["h"], d["w"])
        mp.add(Dp + [-REAR_AXLE, 0] if len(Dp) else None, Sp + [-REAR_AXLE, 0] if len(Sp) else None)
        res = mp.errors_at(FA)
        # 비교: 사진 한 장 방식 (보이는 가장 가까운 곳에서 접선으로 늘림)
        cd, cs = fit(Dp), fit(Sp); cc, kind = lane_center(cd, cs); single = None
        if cc is not None:
            xn = near_x(Dp, Sp); m = float(np.polyval(np.polyder(cc), xn))
            single = (float(np.polyval(cc, xn)) + m * (FRONT_AXLE - xn), np.degrees(np.arctan(m)))
        img = cv2.imread(d["f"])
        cv2.imwrite(os.path.join(out, "%04d.png" % len(out_rows)), draw(img, mp, res, single, Dp, Sp, r))
        out_rows.append([os.path.basename(d["f"]), r["board_adc"], r["steer"],
                         *(("%.4f" % res["e"], "%.2f" % np.degrees(res["psi"]), "%.3f" % res["kap"], res["sides"]) if res else ("",) * 4),
                         *(("%.4f" % single[0], "%.2f" % single[1]) if single else ("", ""))])
    with open(os.path.join(out, "replay.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "adc", "steer", "map_e_front_m", "map_psi_deg", "map_kap", "map_sides", "single_e_front_m", "single_psi_deg"])
        w.writerows(out_rows)
    print("저장:", out, len(out_rows), "장")


def draw(img, mp, res, single, Dp, Sp, r):
    cam = cv2.resize(img, (int(640 * BH / 480), BH))
    b = np.full((BH, BW, 3), 30, np.uint8)
    off = np.array([REAR_AXLE, 0.0])                   # 뒷바퀴 축 기준 → vehicle.yaml(앞범퍼) 기준으로 그림
    for x in np.arange(-1.0, 2.01, 0.5):
        cv2.line(b, tuple(bev(x, 0.8)[0]), tuple(bev(x, -0.8)[0]), (60, 60, 60), 1)
    if len(mp.pts):
        C = mp.to_car(mp.pts[:, :2]) + off; age = mp.t - mp.pts[:, 3]
        for (x, y), side, a in zip(C, mp.pts[:, 2], age):
            if a < 1e-6: continue
            f = max(0.25, 1 - a / mp.keep_sec)
            col = (0, int(160 * f), 0) if side > 0 else (int(200 * f), int(90 * f), 0)
            cv2.circle(b, tuple(bev(x, y)[0]), 1, col, -1)            # 기억한 점 (흐릴수록 오래됨)
    for P, col in ((Dp, (0, 255, 0)), (Sp, (255, 140, 0))):
        for p in bev(P[:, 0], P[:, 1]) if len(P) else []:
            cv2.circle(b, tuple(p), 1, col, -1)                        # 이번 사진의 점
    hw = VL["width_m"] / 2
    cv2.polylines(b, [bev([0, 0, -VL["length_m"], -VL["length_m"]], [hw, -hw, -hw, hw])], True, (255, 255, 255), 2)
    for ax in (FRONT_AXLE, REAR_AXLE):
        cv2.line(b, tuple(bev(ax, VL["track_m"] / 2)[0]), tuple(bev(ax, -VL["track_m"] / 2)[0]), (200, 200, 200), 1)
    cv2.circle(b, tuple(bev(FRONT_AXLE, 0)[0]), 4, (0, 255, 255), -1)
    info = [r["file"][:20], "steer %s adc %s" % (r["steer"], r["board_adc"])]
    if res:
        e, psi = res["e"], res["psi"]
        xs = np.linspace(-0.4, 0.8, 20); ys = e + np.tan(psi) * xs + 0.5 * res["kap"] * xs ** 2
        cv2.polylines(b, [bev(xs + FRONT_AXLE, ys)], False, (0, 255, 255), 2, cv2.LINE_AA)
        info += ["MAP  e_front %+.3f m  psi %+.1f deg" % (e, np.degrees(psi)), "     sides %d  newest pt %.2f s" % (res["sides"], res["newest_age"])]
    else:
        info.append("MAP  none")
    if single:
        info.append("1-img e_front %+.3f m psi %+.1f deg" % single)
    y0 = 18
    for line in info:
        cv2.putText(b, line, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1); y0 += 18
    return np.hstack([cam, b])


if __name__ == "__main__":
    main()
