# -*- coding: utf-8 -*-
"""
차 움직임 식 (vehicle_motion.py, 2026-10-16) - ghost_node / filter_replay / tunable_controller(고스트 모드) 가 같이 쓴다.
값은 전부 vehicle.yaml 의 motion (실측 또는 10/10 주행 데이터).

   조향 ADC → 바퀴 등가 각도 δ (기하학)
   실제로 도는 곡률 = ratio(|δ|) · tan(δ) / L      ratio = understeer_table 을 직선으로 이은 값 (언더스티어)
   거꾸로: 원하는 곡률 → 내야 할 δ → 명령 % (실측 끝 각도 기준)
"""
import math
import os

import numpy as np
import yaml

D = os.path.dirname(os.path.abspath(__file__))
_V = yaml.safe_load(open(os.path.join(D, "vehicle.yaml")))
M = _V["motion"]
L = float(_V["vehicle"]["wheelbase_m"])
_TAB = np.array(M["understeer_table"], float)


def ratio(delta):
    """기하학 조향 δ(rad) 에서 실제로 먹히는 비율."""
    return float(np.interp(abs(math.degrees(delta)), _TAB[:, 0], _TAB[:, 1]))


def delta_of_adc(adc):
    a = adc - M["adc_center"]
    return math.radians(M["deg_per_count_left"] if a > 0 else M["deg_per_count_right"]) * a


def effective_delta(delta):
    """기하학 δ → 자전거 모델에 넣을 실제 등가 δ (고스트 이동용)."""
    return math.atan(ratio(delta) * math.tan(delta))


def command_delta(delta_kin):
    """원하는 실제 등가 δ (Stanley 결과) → 내야 할 기하학 δ. 끝 각도를 넘으면 끝 각도."""
    want = math.tan(abs(delta_kin))
    lock = math.radians(M["lock_deg_left"] if delta_kin > 0 else M["lock_deg_right"])
    if ratio(lock) * math.tan(lock) <= want:
        return math.copysign(lock, delta_kin)
    lo, hi = 0.0, lock
    for _ in range(40):                              # ratio·tan 은 δ 에 대해 커지기만 하므로 이분법
        mid = 0.5 * (lo + hi)
        if ratio(mid) * math.tan(mid) < want: lo = mid
        else: hi = mid
    return math.copysign(0.5 * (lo + hi), delta_kin)


def steer_percent(delta_geo):
    """기하학 δ(rad, + 왼쪽) → 펌웨어 명령 크기 0~100 (부호는 제어기의 steering_sign 이 정함)."""
    lock = math.radians(M["lock_deg_left"] if delta_geo > 0 else M["lock_deg_right"])
    return min(1.0, abs(delta_geo) / lock) * 100.0
