# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 E  (2026-10-09)  -  "YOLO 가 찾은 선 → 차가 따라갈 경로(곡선)"
=====================================================================================

이 프로그램은 ROS 2 노드 하나입니다. 하는 일은 딱 하나:
   카메라 사진에서 YOLO 가 찾아 준 "실선(solid_line)" 과 "점선(dashed_line)" 을 보고,
   우리 차로(2차선)의 한가운데를 지나는 곡선  y = a·x² + b·x + c  를 계산해서 보낸다.

   ┌──────────────┐   /detections    ┌───────────────┐  /lane_trajectory  ┌────────────┐
   │ yolov8_node  │ ───────────────▶ │  이 노드 (E)   │ ─────────────────▶ │  제어기     │ → 조향/속도
   └──────────────┘ (선 모양 + 확신도) └───────────────┘   (경로 곡선 a,b,c) └────────────┘

좌표 약속 (중요!)
   x = 차 앞쪽 거리 (미터).  0 = 카메라 보정 구간의 가장 가까운 줄(차 앞 약 0.84 m)
   y = 차 왼쪽 거리 (미터).  + 는 왼쪽, - 는 오른쪽
   그래서 c > 0 이면 "차로 한가운데가 차보다 왼쪽에 있다" = 왼쪽으로 꺾어야 함.

트랙 규칙 (예선): 항상 2차선을 달린다.
   왼쪽 경계 = 점선(dashed_line),  오른쪽 경계 = 실선(solid_line),  차로 폭 0.85 m
   → 차로 한가운데 = 실선에서 0.425 m 왼쪽 = 점선에서 0.425 m 오른쪽

B 와 달라진 점 (나머지는 B 와 같음)
   [E1] 가로로 누운 실선은 버린다.
        횡단보도/정지선 앞의 굵은 가로선이 solid_line 으로 잡히면, B 는 그것을 "오른쪽 실선" 으로 착각해서
        경로를 순간적으로 오른쪽으로 꺾었다 (횡단보도에서 멈칫 + 오른쪽으로 가던 현상).
        E 는 실선 모양을 바닥 좌표로 바꿔 방향을 재고, 차 진행 방향과 60° 넘게 기울어 있으면 무시한다.
   [E2] 선 종류마다 다른 확신도 기준.
        점선은 흐려서 YOLO 확신도가 낮게 나오는 경우가 많다 → 0.30 이상이면 인정 (실선은 0.50 그대로).
        (YOLO 자체 threshold 는 0.3 으로 낮춰서 실행: CONF=0.3)
   [E3] 경로를 더 멀리까지 본다.
        B 는 차 앞 0.84~1.40 m 만 써서 커브를 늦게 알아챘다. E 는 기본 1.0 m 범위(약 0.84~1.84 m)까지 쓴다.
        → 커브가 시작되는 것을 더 일찍 보고 더 일찍 꺾는다.

실행 (start_ab.sh 가 대신 해 줌):  ALGO=E CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import math
import os
import sys
import time

import cv2                      # 사진/도형 계산 라이브러리 (OpenCV)
import numpy as np              # 숫자 배열 계산 라이브러리
import rclpy                    # ROS 2 파이썬 라이브러리
import yaml                     # 설정 파일(.yaml) 읽기
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory   # 팀이 정의한 메시지 양식

# 팀 labeling-tools 의 함수 하나를 빌려 쓴다: 사진 한 줄(row)에서 선이 칠해진 구간들을 찾는 함수
sys.path.insert(0, os.path.join(os.path.expanduser("~/jacapdi-live-car-2026-09-24"), "labeling-tools"))
from lane_from_lines import runs_in_row   # noqa: E402   (예: [0,0,1,1,1,0,1,1] → [(2,4), (6,7)])


class LinePathNodeE(Node):
    # ---------------------------------------------------------------------------------
    # 1) 시작할 때 한 번: 설정값(파라미터) 읽기, 통로(토픽) 연결
    # ---------------------------------------------------------------------------------
    def __init__(self):
        super().__init__("line_path_node")          # 노드 이름 (다른 프로그램이 이 이름으로 우리를 확인함)
        p = self.declare_parameter                  # "실행할 때 바꿀 수 있는 값" 을 선언하는 함수 (줄여 쓰기)

        # --- 통로(토픽) 이름 ---
        self.det_topic = p("sub_detection_topic", "/detections").value       # YOLO 결과가 들어오는 통로
        self.traj_topic = p("pub_trajectory_topic", "/lane_trajectory").value # 우리가 경로를 내보내는 통로
        self.band_topic = p("pub_topic", "/lane_detections").value           # 화면(path_viewer)용 차로 띠

        # --- 트랙/선 관련 ---
        self.lane_width_m = p("lane_width_m", 0.85).value      # 트랙 차로 폭 (규격)
        self.solid_min_score = p("solid_min_score", 0.50).value   # [E2] 실선 최소 확신도
        self.dashed_min_score = p("dashed_min_score", 0.30).value # [E2] 점선 최소 확신도
        self.transverse_deg = p("transverse_deg", 60.0).value     # [E1] 이 각도보다 누운 실선은 버림

        # --- 얼마나 멀리 볼지 ---
        self.near_range_m = p("near_range_m", 1.0).value        # [E3] 평소 쓰는 거리 (x=0 부터)
        self.ext_range_m = p("ext_range_m", 1.4).value          # 점이 부족할 때 넓혀 볼 최대 거리

        # --- 안전 관련 ---
        self.max_image_age = p("max_image_age_sec", 0.3).value  # 사진이 이보다 오래됐으면 쓰지 않음
        self.hold_last_sec = p("hold_last_sec", 1.0).value      # 선을 잃으면 직전 경로를 이 시간까지만 유지
        self.search_offset_m = p("search_offset_m", 0.15).value # 선이 전혀 없으면 오른쪽으로 찾는 정도
        self.search_max_sec = p("search_max_sec", 2.0).value    # 그렇게 찾는 최대 시간 (넘으면 정지)

        # --- 카메라 보정값 (사진 픽셀 → 바닥 미터 변환에 필요) ---
        ipm_path = p("ipm_yaml", os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")).value
        self.ipm = yaml.safe_load(open(ipm_path))["lane_info_extractor_node"]["ros__parameters"]
        self.M = None                                            # 변환 행렬 (첫 사진 크기를 보고 만듦)

        # --- 통로 연결 ---
        # RELIABLE = 메시지를 잃어버리지 않도록 확인하며 보냄, depth=1 = 가장 최신 것 하나만 보관
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(DetectionArray, self.det_topic, self.on_detections, qos)  # 받기
        self.traj_pub = self.create_publisher(LaneTrajectory, self.traj_topic, qos)       # 보내기 (경로)
        self.band_pub = self.create_publisher(DetectionArray, self.band_topic, qos)       # 보내기 (화면용)

        # --- 상태 기억용 변수 ---
        self.last_traj = None; self.last_band = None; self.last_ok_time = 0.0   # 직전 정상 경로
        self.search_start = None                                                 # 오른쪽 탐색 시작 시각
        self.stats = {"frames": 0, "ok": 0, "both": 0, "one_side": 0, "held": 0, "search": 0, "transverse_dropped": 0}
        self.last_reason = ""
        self.create_timer(2.0, self.log_stats)      # 2초마다 통계를 로그에 남김 (line_path.log)
        self.get_logger().info("algo E: transverse<%.0fdeg drop, solid>=%.2f dashed>=%.2f, near %.2fm"
                               % (self.transverse_deg, self.solid_min_score, self.dashed_min_score, self.near_range_m))

    # ---------------------------------------------------------------------------------
    # 2) 사진 픽셀 → 바닥 미터 변환 준비 (카메라 보정 = IPM)
    # ---------------------------------------------------------------------------------
    def homography(self, h, w):
        """사진 속 사다리꼴(바닥의 직사각형) 4점 ↔ 조감도 직사각형 4점 을 잇는 변환 행렬 M 을 만든다.
        M 으로 사진 점을 '하늘에서 내려다본 그림(BEV)' 의 점으로 옮기고, 거기서 픽셀 × (미터/픽셀) 로 미터를 얻는다."""
        if self.M is None or self.M_shape != (h, w):
            ip = self.ipm
            src = np.float32(ip["ipm_src_points"]).reshape(4, 2)            # 사진 속 4점 (640x480 기준)
            src[:, 0] *= w / float(ip["ipm_src_base_width"]); src[:, 1] *= h / float(ip["ipm_src_base_height"])
            self.bev_h = int(round(h * float(ip["bev_height_scale"])))      # 조감도 세로 픽셀 수
            dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                              [w * ip["ipm_dst_right_ratio"], self.bev_h - 1], [w * ip["ipm_dst_left_ratio"], self.bev_h - 1]])
            self.M = cv2.getPerspectiveTransform(src, dst); self.Minv = np.linalg.inv(self.M)
            self.M_shape = (h, w)
            self.lat = float(ip["lateral_meter_per_pixel"])       # 조감도 가로 1픽셀 = 몇 미터
            self.lon = float(ip["longitudinal_meter_per_pixel"])  # 조감도 세로 1픽셀 = 몇 미터
            self.ego = (w - 1) * 0.5                              # 차 중심선 (조감도 가운데)
        return self.M

    def to_m(self, xs, ys):
        """사진 픽셀 좌표들 (xs[i], ys[i]) → 바닥 좌표 (x 앞 m, y 왼쪽 m) 배열 두 개."""
        pts = cv2.perspectiveTransform(np.float32(np.c_[xs, ys]).reshape(-1, 1, 2), self.M).reshape(-1, 2)
        return (self.bev_h - 1 - pts[:, 1]) * self.lon, (self.ego - pts[:, 0]) * self.lat

    # ---------------------------------------------------------------------------------
    # 3) [E1] 이 실선이 '가로로 누운 선' 인가?  (횡단보도/정지선 앞 굵은 선)
    # ---------------------------------------------------------------------------------
    def is_transverse(self, poly):
        """실선 테두리 점들을 바닥 좌표로 바꾼 뒤, 점들이 퍼진 주된 방향(주성분)을 구한다.
        차로 경계선이라면 대체로 차 앞쪽(x 방향)으로 길게 뻗어 있고,
        정지선/횡단보도 앞 선은 옆(y 방향)으로 길게 누워 있다.
        주된 방향이 x 축과 transverse_deg(기본 60°) 넘게 벌어지면 '가로선' 으로 판단."""
        xf, yl = self.to_m(poly[:, 0], poly[:, 1])
        ok = (xf > -0.5) & (xf < 4.0)                    # 지평선 근처(변환이 이상해지는 점)는 제외
        if ok.sum() < 4:
            return False                                 # 점이 너무 적으면 판단 보류 (버리지 않음)
        P = np.c_[xf[ok], yl[ok]]
        P = P - P.mean(axis=0)                           # 가운데로 옮기고
        _, _, vt = np.linalg.svd(P, full_matrices=False) # 가장 길게 퍼진 방향 = vt[0]
        dx, dy = vt[0]
        angle = math.degrees(math.atan2(abs(dy), abs(dx)))   # 0° = 앞쪽 방향, 90° = 옆 방향
        return angle > self.transverse_deg

    # ---------------------------------------------------------------------------------
    # 4) YOLO 결과가 올 때마다 (1초에 약 30번) 실행되는 함수
    # ---------------------------------------------------------------------------------
    def on_detections(self, msg):
        self.stats["frames"] += 1
        # (a) 사진이 너무 오래됐으면 쓰지 않는다 (지연된 정보로 운전하면 위험)
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        shape = next(((d.mask.height, d.mask.width) for d in msg.detections if d.mask.height > 0), None)
        if age > self.max_image_age or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > self.max_image_age else "no_mask"
            return self.fail(msg.header)
        h, w = shape
        self.homography(h, w)

        # (b) 실선 / 점선을 각각 흑백 그림(마스크)으로 칠한다. 칠할지 말지는 [E1][E2] 규칙으로 거른다.
        md = np.zeros((h, w), np.uint8)    # 점선 마스크 (255 = 점선이 있는 픽셀)
        ms = np.zeros((h, w), np.uint8)    # 실선 마스크
        for d in msg.detections:
            if not d.mask.data:
                continue
            is_solid = d.class_name == "solid_line"
            is_dashed = d.class_name == "dashed_line"
            if not (is_solid or is_dashed):
                continue                                                   # 다른 클래스(정지선 등)는 무시
            if is_solid and d.score < self.solid_min_score:  continue      # [E2] 실선 확신도 기준
            if is_dashed and d.score < self.dashed_min_score: continue     # [E2] 점선 확신도 기준 (더 관대)
            poly = np.array([[p.x, p.y] for p in d.mask.data], np.float32)
            if is_solid and self.is_transverse(poly):                      # [E1] 가로로 누운 실선 버림
                self.stats["transverse_dropped"] += 1
                continue
            pts = np.clip(np.round(poly), [0, 0], [w - 1, h - 1]).astype(np.int32)
            cv2.fillPoly(md if is_dashed else ms, [pts], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            return self.fail(msg.header)

        # (c) 마스크로 경로 계산
        traj, band, reason = self.find_path(md, ms, h, w, msg.header)
        self.last_reason = reason
        if traj is None:
            return self.fail(msg.header)
        if not reason.startswith("search"):
            self.stats["both" if reason.startswith("solid+dashed") else "one_side"] += 1
        self.stats["ok"] += 1
        self.last_traj, self.last_band, self.last_ok_time = traj, band, time.time()
        self.traj_pub.publish(traj)
        self.publish_band(msg.header, band, shape)

    # ---------------------------------------------------------------------------------
    # 5) 경로 계산의 핵심: 아래(차 바로 앞)부터 위로 한 줄씩 훑으며 차로 한가운데 점을 모은다
    # ---------------------------------------------------------------------------------
    def find_path(self, md, ms, h, w, header):
        self.homography(h, w)                   # 픽셀→미터 변환 준비 (이미 했으면 그대로 씀)
        half = self.lane_width_m / 2.0          # 0.425 m
        pts = []                                # 모은 (x, y) 중심점들
        n_s = n_d = 0                           # 실선 / 점선을 본 줄 수
        prev_r = prev_d = None                  # 바로 아랫줄에서 찾은 실선 / 점선 위치 (연결성 확인용)
        last_xf = -1e9
        for row in range(h - 1, -1, -2):        # 사진 맨 아래(가까운 곳) → 위(먼 곳), 2줄씩
            xf0 = float(self.to_m([self.ego], [row])[0][0])      # 이 줄이 차 앞 몇 m 인가
            if xf0 < last_xf - 1e-6 or xf0 > self.ext_range_m:  # 지평선에 가까워지거나 너무 멀면 멈춤
                break
            last_xf = xf0
            if xf0 < -0.3:
                continue
            solid_runs = runs_in_row(ms[row])                   # 이 줄에서 실선이 칠해진 구간들
            dash_runs = runs_in_row(md[row])                    # 이 줄에서 점선이 칠해진 구간들
            yr = yd = None                                      # 이번 줄의 오른쪽 경계 / 왼쪽 경계 (m)

            # --- 오른쪽 경계 = 실선의 '안쪽(왼쪽) 끝' ---
            if solid_runs:
                _, ys = self.to_m([r[0] for r in solid_runs], [row] * len(solid_runs))
                if prev_r is None:   # 처음 찾을 때: 차 오른쪽(-1.2 ~ +0.2 m)에 있는 것 중 차로 폭 절반에 가장 가까운 것
                    cand = [v for v in ys if -1.2 < v < 0.2]
                    yr = min(cand, key=lambda v: abs(v + half)) if cand else None
                else:                # 이어서 찾을 때: 아랫줄 위치와 15 cm 안쪽으로 이어지는 것만 (튀는 것 방지)
                    v = min(ys, key=lambda v: abs(v - prev_r)); yr = v if abs(v - prev_r) < 0.15 else None

            # --- 왼쪽 경계 = 점선의 '안쪽(오른쪽) 끝' ---
            if dash_runs:
                _, ys = self.to_m([r[1] for r in dash_runs], [row] * len(dash_runs))
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None        # 점선보다 왼쪽에 있는 실선 = 1차선 바깥 실선 → 우리 경계 아님
                if yr is not None:
                    ys = [v for v in ys if v > yr + 0.4]   # 2차선 규칙: 점선은 실선보다 0.4 m 이상 왼쪽
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = min(cand, key=lambda v: abs(v - half)) if cand else None
                elif len(ys):
                    v = min(ys, key=lambda v: abs(v - prev_d)); yd = v if abs(v - prev_d) < 0.2 else None

            # --- 이 줄의 차로 한가운데 ---
            if yr is not None and yd is not None and abs((yd - yr) - self.lane_width_m) < 0.3:
                yc = 0.5 * ((yr + half) + (yd - half)); n_s += 1; n_d += 1   # 둘 다 보이고 폭이 맞으면 평균
            elif yr is not None:
                yc = yr + half; n_s += 1                                      # 실선만: 실선 + 0.425 m
            elif yd is not None:
                yc = yd - half; n_d += 1                                      # 점선만: 점선 - 0.425 m
            else:
                yc = None
            if yr is not None: prev_r = yr
            if yd is not None: prev_d = yd
            if yc is not None:
                pts.append((xf0, yc))

        # --- 선이 하나도 없으면: 잠깐 오른쪽으로 찾아보고, 2초 넘으면 정지 ---
        if n_s == 0 and n_d == 0:
            now = time.time()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > self.search_max_sec:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = -float(self.search_offset_m)               # 중심이 오른쪽 15 cm 에 있다고 가정
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_right(%.1fs)" % (now - self.search_start)
        self.search_start = None
        if len(pts) < 8:
            return None, None, "too_few_points(%d)" % len(pts)

        # --- 모은 점들에 곡선 y = a·x² + b·x + c 맞추기 ---
        P = np.array(pts)
        sel = P[P[:, 0] <= self.near_range_m]                 # [E3] 평소 범위 (기본 1.0 m)
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:         # 점이 부족하면 더 멀리까지
            sel = P; used = "ext"
        if len(sel) < 8 or np.ptp(sel[:, 0]) < 0.08:
            return None, None, "short_span"
        deg = 2 if np.ptp(sel[:, 0]) >= 0.3 else 1            # 점이 퍼진 길이가 짧으면 직선(1차)만
        c = np.polyfit(sel[:, 0], sel[:, 1], deg)
        if deg == 1: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))   # 곡선이 점과 평균 몇 m 어긋나나
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, None, "bad_fit(%.2f)" % err          # 15 cm 넘게 어긋나면 믿지 않음

        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        # 화면용: 경로 ± 0.425 m 띠를 다시 사진 좌표로
        xs = np.linspace(t.x_min_m, t.x_max_m, 12); yc = np.polyval(c, xs)
        band = [(self.ego - (y + half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs, yc)] + \
               [(self.ego - (y - half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs[::-1], yc[::-1])]
        band = cv2.perspectiveTransform(np.float32(band).reshape(-1, 1, 2), self.Minv).reshape(-1, 2)
        kind = "solid+dashed" if n_s and n_d else ("solid_only" if n_s else "dashed_only")
        return t, [(float(x), float(y)) for x, y in band], "%s,%s" % (kind, used)

    # ---------------------------------------------------------------------------------
    # 6) 실패했을 때: 1초까지는 직전 경로를 계속 쓰고, 그 뒤엔 valid=false (제어기가 차를 세움)
    # ---------------------------------------------------------------------------------
    def fail(self, header):
        held = self.last_traj is not None and time.time() - self.last_ok_time <= self.hold_last_sec
        if held:
            self.stats["held"] += 1
            t = self.last_traj; t.header = header
        else:
            t = LaneTrajectory(); t.header = header; t.valid = False
        self.traj_pub.publish(t)
        self.publish_band(header, self.last_band if held else None, None)

    def publish_band(self, header, band, shape):
        out = DetectionArray(); out.header = header
        if band is not None:
            d = Detection(); d.class_name = "lane"; d.score = 1.0
            if shape: d.mask.height, d.mask.width = shape
            for x, y in band:
                pt = Point2D(); pt.x = float(x); pt.y = float(y); d.mask.data.append(pt)
            out.detections.append(d)
        self.band_pub.publish(out)

    def log_stats(self):
        s = self.stats
        self.get_logger().info("frames=%d ok=%d (both %d, one_side %d, search %d) held=%d transverse_dropped=%d last=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s["search"], s["held"],
                                  s["transverse_dropped"], self.last_reason))


def main():
    rclpy.init()
    n = LinePathNodeE()
    try:
        rclpy.spin(n)          # 메시지가 올 때마다 on_detections 를 불러 주며 계속 실행
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
