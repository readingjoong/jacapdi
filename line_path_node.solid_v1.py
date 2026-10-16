# -*- coding: utf-8 -*-
"""선 라벨(solid_line/dashed_line) YOLO 결과 -> lane 폴리곤 -> /lane_detections (실시간 ROS 연결부).

START_HERE_KO.md 2B 계약을 따른다.
  - /detections 의 mask.data 는 픽셀 좌표이므로 polygon_to_mask(정규화용)를 쓰지 않고 직접 fillPoly 한다.
  - 경계 추출은 팀 labeling-tools 원본 함수를 그대로 호출한다.
      width_model 파라미터가 있으면: lane_path.find_path (한쪽 선 보완 포함)
      없으면: lane_from_lines.scan_lane / interpolate_edges / lane_polygon (양쪽 선이 관측된 구간만)
  - 결과 lane 폴리곤을 class_name='lane' 픽셀 Detection 으로 /lane_detections 에 발행한다.
    미터 변환(IPM)과 LaneTrajectory 발행은 팀 lane_info_extractor_node 가
    sub_detection_topic:=/lane_detections 로 재사용한다.
  - 실패/영상 지연(max_image_age_sec 초과)/원본 영상 없음은 빈 DetectionArray 를 발행해
    lane_info_extractor 가 valid=false 를 내게 한다. 마지막 정상 경로를 재발행하지 않는다.
  - header 는 원본 프레임 header 를 유지한다.

관대 모드 (사용자 요청, 2026-09-24):
  - online_width:=true  양쪽 선이 보인 행에서 차로 폭 표본을 모아 lane_path.fit_width 로 폭 모델을 온라인 학습.
                         양쪽 스캔이 실패하면 그 모델로 lane_path.find_path (한쪽 선 보완)를 시도.
  - hold_last_sec:=T    둘 다 실패하면 마지막 정상 lane 폴리곤을 최대 T초 동안 새 header 로 재발행.
                         T초가 지나면 빈 결과(= valid=false, Stanley 정지).
  - 엄격 모드 원본: line_path_node.strict.py

solid_follow 모드 (mode:=solid_follow, 2026-09-24 예선용):
  - 2차선 고정: 오른쪽 solid_line 안쪽 가장자리 = 차로 오른쪽 경계. dashed_line 은 solid 왼쪽에 있을 때만 왼쪽 경계.
  - 왼쪽 경계가 없거나 폭이 맞지 않으면 IPM(ipm_yaml)으로 solid 에서 lane_width_m(0.85m) 왼쪽 점을 계산해 경계로 쓴다.
  - solid 가 안 보이면 dashed 오른쪽 lane_width_m 을 차로로 본다(여전히 2차선).
  - 차 바로 앞(이미지 아래쪽 near_top_row 아래) 행만 사용해 먼 곳 커브에 미리 조향하지 않게 한다.
  - 관대 모드 원본: line_path_node.lenient.py
"""
import collections
import time
import json
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, Detection, Point2D

PACK = os.environ.get("JACAPDI_LIVE_PACK", os.path.expanduser("~/jacapdi-live-car-2026-09-24"))
sys.path.insert(0, os.path.join(PACK, "labeling-tools"))
from lane_from_lines import scan_lane, interpolate_edges, lane_polygon, runs_in_row  # noqa: E402
from lane_path import find_path, fit_width  # noqa: E402


class LinePathNode(Node):
    def __init__(self):
        super().__init__("line_path_node")
        p = self.declare_parameter
        self.det_topic = p("sub_detection_topic", "/detections").value
        self.img_topic = p("image_topic", "/image_raw").value
        self.pub_topic = p("pub_topic", "/lane_detections").value
        self.solid = p("solid_class", "solid_line").value
        self.dashed = p("dashed_class", "dashed_line").value
        self.width_model_path = p("width_model", "").value
        self.lane_side = p("lane", "right").value          # lane_path: 점선 기준 우리 차로 방향
        self.min_width = p("min_width", 0.10).value        # lane_from_lines 기본값
        self.max_width = p("max_width", 0.90).value
        self.max_gap = p("max_gap", 0.25).value
        self.min_rows = p("min_rows", 0.15).value
        self.max_image_age = p("max_image_age_sec", 0.3).value
        self.online_width = p("online_width", False).value
        self.hold_last_sec = p("hold_last_sec", 0.0).value
        self.mode = p("mode", "lines").value
        self.lane_width_m = p("lane_width_m", 0.85).value
        self.near_top_row = p("near_top_row", -1).value      # 이 행보다 아래만 사용 (-1: IPM 사다리꼴 윗변)
        self.ipm_yaml = p("ipm_yaml", os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")).value
        self.M = self.Minv = None
        if self.mode == "solid_follow":
            import yaml
            ip = yaml.safe_load(open(self.ipm_yaml))["lane_info_extractor_node"]["ros__parameters"]
            self.ipm = ip
            self.get_logger().info("solid_follow mode (lane 2, dashed left / solid right), ipm=%s" % self.ipm_yaml)
        self.width_samples = collections.deque(maxlen=4000)
        self.online_model, self.last_fit = None, 0.0
        self.last_poly, self.last_poly_time = None, 0.0

        self.width_model = None
        if self.width_model_path:
            models = json.load(open(self.width_model_path, encoding="utf-8"))
            self.width_model = models.get("_global") if "_global" in models else models
            self.get_logger().info("find_path mode, width model: %s" % self.width_model_path)
        else:
            self.get_logger().info("both-sides mode (no width model): lane_from_lines scan_lane")

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.bridge = CvBridge()
        self.images = {}  # stamp_ns -> bgr (최근 몇 장)
        self.create_subscription(Image, self.img_topic, self.on_image, 10)
        self.create_subscription(DetectionArray, self.det_topic, self.on_detections, qos)
        self.pub = self.create_publisher(DetectionArray, self.pub_topic, qos)
        self.stats = {"frames": 0, "ok": 0, "both": 0, "one_side": 0, "held": 0}
        self.last_reason = ""
        self.create_timer(2.0, self.log_stats)

    @staticmethod
    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.images[self.stamp_ns(msg.header)] = img
        if len(self.images) > 10:
            for k in sorted(self.images)[:-10]:
                del self.images[k]

    def publish(self, header, poly):
        out = DetectionArray()
        out.header = header
        if poly is not None:
            d = Detection()
            d.class_name = "lane"
            d.score = 1.0
            d.mask.height, d.mask.width = self.shape
            for x, y in poly:
                pt = Point2D(); pt.x = float(x); pt.y = float(y)
                d.mask.data.append(pt)
            out.detections.append(d)
        self.pub.publish(out)

    def on_detections(self, msg):
        self.stats["frames"] += 1
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        img = self.images.get(self.stamp_ns(msg.header))
        shape = None
        for d in msg.detections:
            if d.mask.height > 0 and d.mask.width > 0:
                shape = (d.mask.height, d.mask.width); break
        if shape is None and img is not None:
            shape = img.shape[:2]
        if age > self.max_image_age or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > self.max_image_age else "no_mask"
            self.fail(msg.header); return
        self.shape = shape
        h, w = shape
        md = np.zeros((h, w), np.uint8); ms = np.zeros((h, w), np.uint8)
        for d in msg.detections:
            if not d.mask.data or d.class_name not in (self.solid, self.dashed):
                continue
            pts = np.array([[int(round(p.x)), int(round(p.y))] for p in d.mask.data], np.int32)
            pts[:, 0] = np.clip(pts[:, 0], 0, w - 1); pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            cv2.fillPoly(md if d.class_name == self.dashed else ms, [pts], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            self.fail(msg.header); return

        poly = None
        if self.mode == "solid_follow":
            poly, reason = self.solid_follow(md, ms, h, w)
            self.last_reason = reason
            if poly is not None:
                self.stats["both" if reason == "solid+dashed" else "one_side"] += 1
        elif self.width_model is not None:
            if img is None:
                self.last_reason = "no_image_for_frame"
                self.publish(msg.header, None); return
            args = SimpleNamespace(lane=self.lane_side, max_gap=self.max_gap, min_rows=self.min_rows)
            res = find_path(img, md, ms, self.width_model, args)
            poly = res["lane_poly"]
            self.last_reason = res["status"] + ("," + ",".join(res["flags"]) if res["flags"] else "")
        else:
            mask = cv2.bitwise_or(md, ms)
            L, R, stat = scan_lane(mask, int(self.min_width * w), int(self.max_width * w), int(self.max_gap * h))
            if self.online_width:
                both = np.flatnonzero(~np.isnan(L) & ~np.isnan(R))
                self.width_samples.extend((float(y), float(R[y] - L[y])) for y in both[::2])
                if time.time() - self.last_fit > 1.0 and len(self.width_samples) >= 50:
                    self.last_fit = time.time()
                    m = fit_width(list(self.width_samples), h)
                    if m is not None: self.online_model = m
            L, R = interpolate_edges(L, R, int(self.max_gap * h))
            res = lane_polygon(L, R, min_rows=int(self.min_rows * h))
            if res is not None:
                poly = res[0]; self.last_reason = "ok"; self.stats["both"] += 1
            elif self.online_model is not None and img is not None:
                args = SimpleNamespace(lane=self.lane_side, max_gap=self.max_gap, min_rows=self.min_rows,
                                       min_width=self.min_width, max_width=self.max_width)
                r2 = find_path(img, md, ms, self.online_model, args)
                if r2["lane_poly"] and r2["status"] != "too_short":
                    poly = r2["lane_poly"]; self.last_reason = "find_path:" + r2["status"]; self.stats["one_side"] += 1
                else:
                    self.last_reason = "find_path_failed:" + r2["status"]
            else:
                self.last_reason = "one_side_only" if (stat["rows_left_only"] + stat["rows_right_only"]) > stat["rows_both"] else "too_short"
        if poly is None:
            self.fail(msg.header); return
        self.stats["ok"] += 1
        self.last_poly, self.last_poly_time = poly, time.time()
        self.publish(msg.header, poly)

    def _homography(self, h, w):
        if self.M is None or self.M_shape != (h, w):
            ip = self.ipm
            src = np.float32(ip["ipm_src_points"]).reshape(4, 2)
            src[:, 0] *= w / float(ip["ipm_src_base_width"]); src[:, 1] *= h / float(ip["ipm_src_base_height"])
            bev_h = int(round(h * float(ip["bev_height_scale"])))
            dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                              [w * ip["ipm_dst_right_ratio"], bev_h - 1], [w * ip["ipm_dst_left_ratio"], bev_h - 1]])
            self.M = cv2.getPerspectiveTransform(src, dst); self.Minv = np.linalg.inv(self.M)
            self.M_shape = (h, w); self.top_row = int(src[:, 1].min())
            self.lat_px = self.lane_width_m / float(ip["lateral_meter_per_pixel"])  # 차로폭 BEV 픽셀
        return self.M

    def _shift_in_image(self, x, y, bev_dx):
        """이미지 점 (x, y) 를 BEV 에서 가로로 bev_dx 픽셀 옮긴 점의 이미지 x (같은 행 근사)."""
        b = cv2.perspectiveTransform(np.float32([[[x, y]]]), self.M)[0, 0]
        q = cv2.perspectiveTransform(np.float32([[[b[0] + bev_dx, b[1]]]]), self.Minv)[0, 0]
        return float(q[0])

    def solid_follow(self, md, ms, h, w):
        self._homography(h, w)
        top = self.near_top_row if self.near_top_row >= 0 else self.top_row
        L = np.full(h, np.nan); R = np.full(h, np.nan); kind = np.zeros(h, np.int8)
        prev_r = None; prev_d = None
        for y in range(h - 1, max(top, 0) - 1, -1):
            sr = runs_in_row(ms[y]); dr = runs_in_row(md[y])
            # 오른쪽 solid: 처음엔 화면 가운데보다 오른쪽 조각 중 가장 왼쪽, 이후엔 직전 행과 연속된 조각
            r = None
            if sr:
                if prev_r is None:
                    c = [s_ for s_ in sr if s_[0] > w * 0.4]
                    r = min(c, key=lambda s_: s_[0])[0] if c else None
                else:
                    s_ = min(sr, key=lambda s_: abs(s_[0] - prev_r))
                    r = s_[0] if abs(s_[0] - prev_r) < 40 else None
            # 왼쪽 dashed: solid 보다 왼쪽(2차선 = dashed 왼쪽)
            d = None
            if dr:
                ref = r if r is not None else (prev_r if prev_r is not None else w)
                c = [d_ for d_ in dr if d_[1] < ref - 10]
                if c:
                    d = max(c, key=lambda d_: d_[1])[1]
                    if prev_d is not None and abs(d - prev_d) > 60: d = None
            if r is not None:
                exp_l = self._shift_in_image(r, y, -self.lat_px)
                if d is not None and abs((r - d) - (r - exp_l)) < 0.35 * (r - exp_l):
                    L[y], R[y], kind[y] = d, r, 1
                else:
                    L[y], R[y], kind[y] = exp_l, r, 2
                prev_r = r
                if d is not None: prev_d = d
            elif d is not None:
                L[y], R[y], kind[y] = d, self._shift_in_image(d, y, self.lat_px), 3
                prev_d = d
        have = np.flatnonzero(kind > 0)
        if have.size < max(10, int(self.min_rows * h)):
            return None, "too_few_rows(%d)" % have.size
        # 아래에서부터 끊김(max_gap) 없는 구간만
        seg = [int(have[-1])]
        for y in have[::-1][1:]:
            if seg[-1] - y <= int(self.max_gap * h): seg.append(int(y))
            else: break
        idx = np.array(sorted(seg)); ys = np.arange(idx[0], idx[-1] + 1)
        Lf = np.interp(ys, idx, L[idx]); Rf = np.interp(ys, idx, R[idx])
        pts = np.array([(x, y) for x, y in zip(Lf[::-1], ys[::-1])] + [(x, y) for x, y in zip(Rf, ys)], np.float32)
        approx = cv2.approxPolyDP(pts.reshape(-1, 1, 2), 2.0, True).reshape(-1, 2)
        k = collections.Counter(int(v) for v in kind[idx])
        reason = "solid+dashed" if k.get(1, 0) >= len(idx) * 0.5 else ("solid_only" if k.get(2, 0) >= k.get(3, 0) else "dashed_only")
        return [(float(x), float(y)) for x, y in approx], reason

    def fail(self, header):
        """실패 시: hold_last_sec 이내면 마지막 폴리곤 재발행, 아니면 빈 결과(정지)."""
        if self.last_poly is not None and time.time() - self.last_poly_time <= self.hold_last_sec:
            self.stats["held"] += 1
            self.publish(header, self.last_poly)
        else:
            self.publish(header, None)

    def log_stats(self):
        s = self.stats
        wm = "none" if self.online_model is None else "a=%.3f b=%.1f n=%d" % (self.online_model["a"], self.online_model["b"], self.online_model["n"])
        self.get_logger().info("frames=%d ok=%d (both %d, one_side %d) held=%d last=%s width_model=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s["held"], self.last_reason, wm))


def main():
    rclpy.init()
    n = LinePathNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
