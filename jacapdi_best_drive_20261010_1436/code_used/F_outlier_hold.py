# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 F  (2026-10-09)  =  E  +  "경로가 갑자기 튀면 무시하고 직전 경로 유지"
=====================================================================================

왜 필요한가?
   10/09 주행 횡단보도에서: 한 프레임 만에 경로가 c=+0.16 (왼쪽) → c=-0.15 (오른쪽) 로 바뀌었고
   조향 명령이 s-4 → s+4 로 뒤집혔다 (멈칫 + 오른쪽으로 가는 현상).
   횡단보도/정지선/주차구역 등 선이 갑자기 많이 보이거나 끊기는 곳에서 경로 계산이 잠깐 엉뚱해진다.

어떻게 하나? (이상치 = outlier)
   1) 새로 계산한 경로와 마지막으로 "믿은" 경로를 비교한다.
      차 앞 x = 0.0 / 0.4 / 0.8 m 세 지점에서 두 경로의 옆 위치(y) 차이 중 가장 큰 값을 본다.
   2) 차이가 jump_m (기본 0.10 m = 10 cm) 보다 크면 "이상치" → 새 경로는 버리고 믿은 경로를 그대로 보낸다.
      (정상 주행에서 1/30 초 사이 경로 변화는 보통 1 cm 미만. 10/09 E 주행 기록 10 Hz 기준 90% 가 4 cm 이하)
   3) 그런데 이상치가 outlier_hold_sec (기본 0.5 초) 넘게 계속되면 → "진짜로 길이 바뀐 것" 으로 보고 새 경로를 받아들인다.
      (영원히 옛 경로에 갇히지 않도록)
   4) 선을 아예 잃었을 때 E 는 "오른쪽으로 찾기(search_right)" 경로를 냈는데, 이것도 같은 규칙을 거친다.
      → 횡단보도에서 선이 잠깐 끊겨도 바로 오른쪽으로 꺾지 않고 0.5 초는 직전 경로로 간다.

조정값 (start 할 때 -p 로 바꿀 수 있음)
   jump_m            0.10   이보다 크게 튀면 이상치.  크게 하면 덜 막음, 작게 하면 커브 진입도 막을 수 있음
   outlier_hold_sec  0.5    이상치를 최대 몇 초 무시할지. 길게 하면 진짜 커브에 늦게 반응
   ref_max_age_sec   1.0    믿은 경로가 이보다 오래되면(선을 오래 잃었으면) 비교하지 않고 새 경로를 바로 믿음
   far_only_xmin_m   0.4    [10/09 추가] 경로를 만든 점들이 이 거리(m)보다 먼 곳에만 있으면(차 앞이 안 보임, 예: 횡단보도)
                            그 경로를 차 앞까지 늘린 값은 믿을 수 없다 → 이상치로 보고 믿은 경로를 계속 쓴다 (시간 제한 없이, ref_max_age 까지)

실행:  ALGO=F CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import copy
import os
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from E_curve_cross import LinePathNodeE   # noqa: E402   E 의 모든 기능을 그대로 물려받는다


class LinePathNodeF(LinePathNodeE):
    CHECK_X = np.array([0.0, 0.4, 0.8])      # 경로를 비교할 차 앞 지점들 (m, x=0 은 차 앞 약 0.84 m)

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.jump_m = p("jump_m", 0.10).value
        self.outlier_hold_sec = p("outlier_hold_sec", 0.5).value
        self.ref_max_age_sec = p("ref_max_age_sec", 1.0).value
        self.far_only_xmin_m = p("far_only_xmin_m", 0.4).value
        self.stats["far_only_held"] = 0
        self.ref = None; self.ref_band = None; self.ref_time = 0.0   # 마지막으로 "믿은" 경로
        self.outlier_since = None                                    # 이상치가 시작된 시각
        self.stats["outlier_held"] = 0
        self.stats["outlier_accepted"] = 0
        self.get_logger().info("algo F: outlier jump>%.2fm hold<=%.2fs" % (self.jump_m, self.outlier_hold_sec))

    # E 의 find_path 결과를 한 번 더 검사한다
    def find_path(self, md, ms, h, w, header):
        traj, band, reason = super().find_path(md, ms, h, w, header)
        if traj is None:
            return traj, band, reason                    # 실패는 E 와 같이 처리 (직전 경로 1초 유지 후 정지)
        now = time.time()
        ref_ok = self.ref is not None and now - self.ref_time <= self.ref_max_age_sec
        # [10/09] 먼 곳 점만으로 만든 경로 (차 앞 x_min 이 멀다) → 믿은 경로 유지
        if ref_ok and traj.x_min_m > self.far_only_xmin_m:
            self.stats["far_only_held"] += 1
            t = copy.deepcopy(self.ref); t.header = header
            return t, self.ref_band, "far_only_hold(xmin=%.2f,%s)" % (traj.x_min_m, reason)
        if ref_ok:
            y_new = np.polyval([traj.a, traj.b, traj.c], self.CHECK_X)
            y_ref = np.polyval([self.ref.a, self.ref.b, self.ref.c], self.CHECK_X)
            jump = float(np.abs(y_new - y_ref).max())    # 두 경로가 옆으로 가장 많이 벌어진 거리 (m)
            if jump > self.jump_m:
                if self.outlier_since is None:
                    self.outlier_since = now
                if now - self.outlier_since < self.outlier_hold_sec:
                    # 이상치 → 새 경로 버리고, 믿은 경로를 복사해서 보낸다 (시각만 지금으로)
                    self.stats["outlier_held"] += 1
                    t = copy.deepcopy(self.ref); t.header = header
                    self.ref_time = now                  # 붙잡고 있는 동안 믿은 경로가 낡았다고 보지 않음
                    return t, self.ref_band, "outlier_hold(%.2fm,%s)" % (jump, reason)
                self.stats["outlier_accepted"] += 1      # 0.5 초 넘게 계속 → 진짜 변화로 인정
                reason = "outlier_accepted(%.2fm),%s" % (jump, reason)
        # 정상 경로 → 믿은 경로로 저장
        self.outlier_since = None
        self.ref, self.ref_band, self.ref_time = copy.deepcopy(traj), band, now
        return traj, band, reason

    def log_stats(self):
        super().log_stats()
        self.get_logger().info("outlier_held=%d outlier_accepted=%d far_only_held=%d"
                               % (self.stats["outlier_held"], self.stats["outlier_accepted"], self.stats["far_only_held"]))


def main():
    rclpy.init()
    n = LinePathNodeF()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
