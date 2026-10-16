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
"""
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
from lane_from_lines import scan_lane, interpolate_edges, lane_polygon  # noqa: E402
from lane_path import find_path  # noqa: E402


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
        self.stats = {"frames": 0, "ok": 0}
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
            self.publish(msg.header, None); return
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
            self.publish(msg.header, None); return

        poly = None
        if self.width_model is not None:
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
            L, R = interpolate_edges(L, R, int(self.max_gap * h))
            res = lane_polygon(L, R, min_rows=int(self.min_rows * h))
            if res is None:
                self.last_reason = "one_side_only" if (stat["rows_left_only"] + stat["rows_right_only"]) > stat["rows_both"] else "too_short"
            else:
                poly = res[0]; self.last_reason = "ok"
        if poly is not None:
            self.stats["ok"] += 1
        self.publish(msg.header, poly)

    def log_stats(self):
        self.get_logger().info("frames=%d lane_ok=%d last=%s" % (self.stats["frames"], self.stats["ok"], self.last_reason))


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
