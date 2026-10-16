# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 G  (2026-10-10)  =  F  +  "횡단보도에서 오른쪽 탐색 대신 직전 경로 유지"  +  실선 확신도 0.30
=====================================================================================

왜 필요한가?
   10/10 13:42 주행 횡단보도 (drive_134207 frame 210~226):
   오른쪽 파란 실선이 사진에는 잘 보이는데 YOLO 확신도가 0.30~0.35 로 떨어졌다 (빨간 줄무늬 옆이라 헷갈림).
   E 는 실선 0.50 미만을 버리고, 점선은 횡단보도 너머(보는 범위 밖)에만 있어서 "선 0 줄" 이 되었다.
   → E 의 search_right (차로 중심이 오른쪽 15 cm 에 있다고 가정하는 고정 경로) 가 나왔고,
     F 가 0.5 초 막은 뒤 받아들여서 경로가 오른쪽으로 45 cm 튀었다 (조향 -100 → +41). 실선이 돌아오자 다시 45 cm 튐.

무엇을 바꿨나?
   [G1] 횡단보도(crosswalk_marking) 나 정지선(stop_line) 이 최근 cross_recent_sec(0.5 초) 안에 보였으면,
        search_right 대신 마지막으로 믿은 경로(F 의 ref)를 그대로 보낸다. 최대 cross_hold_max_sec(1.5 초).
        그보다 길어지면 원래대로 search_right → E 의 2 초 탐색 제한 → 정지 (안전 장치는 그대로).
   [G2] 실선 최소 확신도 solid_min_score 0.50 → 0.30 (점선과 같게).
        10/10 기록에서 0.3~0.5 실선 711 프레임 중 12 장을 보니 11 장이 진짜 오른쪽 실선 (주로 먼 쪽).
        가짜 1 장은 트랙 밖에 세워 둔 차가 창틀을 잡은 것. 가로 실선은 E1, 위치/연결성은 E 규칙이 그대로 거른다.
        예전 기준으로 돌리려면: -p solid_min_score:=0.5

조정값 (start 할 때 -p 로 바꿀 수 있음)
   cross_recent_sec    0.5   횡단보도/정지선이 이 시간 안에 보였으면 "횡단보도 근처"
   cross_min_score     0.5   횡단보도/정지선 인정 확신도
   cross_hold_max_sec  1.5   직전 경로를 최대 몇 초 유지할지

실행:  ALGO=G CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import copy
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from E_curve_cross import LinePathNodeE    # noqa: E402
from F_outlier_hold import LinePathNodeF   # noqa: E402

CROSS_CLASSES = ("crosswalk_marking", "stop_line")


class _CrossSearchGuard(LinePathNodeE):
    """E 의 find_path 바로 위에 끼워서, 횡단보도 근처의 search_right 를 직전 경로로 바꾼다.
       (클래스 순서 G → F → 이것 → E 라서 F 의 이상치 검사는 바뀐 결과를 받는다)"""

    def find_path(self, md, ms, h, w, header):
        traj, band, reason = super().find_path(md, ms, h, w, header)
        now = time.time()
        if traj is None or not reason.startswith("search"):
            self.cross_hold_since = None
            return traj, band, reason
        near_cross = now - self.cross_seen <= self.cross_recent_sec
        ref_ok = self.ref is not None and now - self.ref_time <= self.ref_max_age_sec
        if near_cross and ref_ok:
            if self.cross_hold_since is None:
                self.cross_hold_since = now
            if now - self.cross_hold_since <= self.cross_hold_max_sec:
                self.stats["cross_held"] += 1
                t = copy.deepcopy(self.ref); t.header = header
                return t, self.ref_band, "cross_hold(%.1fs,%s)" % (now - self.cross_hold_since, reason)
        return traj, band, reason


class LinePathNodeG(LinePathNodeF, _CrossSearchGuard):
    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.cross_recent_sec = p("cross_recent_sec", 0.5).value
        self.cross_min_score = p("cross_min_score", 0.5).value
        self.cross_hold_max_sec = p("cross_hold_max_sec", 1.5).value
        self.cross_seen = 0.0; self.cross_hold_since = None
        self.stats["cross_held"] = 0
        # [G2] E 가 선언한 파라미터의 기본값만 바꾼다 (-p solid_min_score:=... 로 직접 주면 그 값)
        if not self.get_parameter_overrides_has("solid_min_score"):
            self.solid_min_score = 0.30
        self.get_logger().info("algo G: cross hold<=%.1fs (seen<=%.1fs), solid_min_score=%.2f"
                               % (self.cross_hold_max_sec, self.cross_recent_sec, self.solid_min_score))

    def get_parameter_overrides_has(self, name):
        return name in self._parameter_overrides

    def on_detections(self, msg):
        if any(d.class_name in CROSS_CLASSES and d.score >= self.cross_min_score for d in msg.detections):
            self.cross_seen = time.time()
        super().on_detections(msg)

    def log_stats(self):
        super().log_stats()
        self.get_logger().info("cross_held=%d" % self.stats["cross_held"])


def main():
    rclpy.init()
    n = LinePathNodeG()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
