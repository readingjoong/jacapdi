# -*- coding: utf-8 -*-
"""U1~U3: 현재 작업 트리 모듈을 algo=base 로 돌려 HEAD(a41706e) 골든과 == 비교.

base 조건 두 가지(tune/algo 부재, "base")로 모두 돌리고, tune.json 에 변형 키(arc_comp 2 포함)·variants.json 에 arc_comp 켜진
프로필이 있어도 base 는 무시함을 같은 비교로 확인한다. 어느 하나라도 다르면 base 비트 보존 실패.

골든 JSON 은 생성 환경(numpy·cv2 버전, meta) 에 묶여 있다 — 다른 numpy/cv2(예: 컨테이너 시스템 python 1.21.5) 에선 polyfit·래스터가
달라 저장 골든과 어긋날 수 있다. identity_lib.load_golden 은 환경이 다르면 같은 환경에서 git a41706e 모듈로 기준을 즉석 생성해
비교한다(git 필요, 없으면 skip 사유 출력). 즉 어느 환경에서든 'HEAD 모듈 == 현재 모듈' 을 같은 인터프리터로 증명한다."""
import pytest

import conftest as C
import golden_lib as G
import identity_lib as I

# base 가 무시해야 하는 것들: 옛 tune.json 변형 키 + variants.json 의 arc_comp 켜진 프로필
POISON_TUNE = {"arc_comp": 2, "arc_kappa_tau_sec": 0.4, "arc_shift_max_m": 0.4, "arc_min_points": 6,
               "one_side_max_c_m": 0.45, "on_line_hold_sec": 0.3, "accept_step_m": 0.3, "corner_steer_pct": 60, "pwm_corner": 190,
               "search_back_m": 0.3, "search_max_sec": 1.0, "hold_last_sec": 0.5, "one_side_max_c_cross_m": 0.7,
               "ext_min_span_m": 0.0, "ext_max_b": 9.0, "kappa_tau_sec": 0.15}
POISON_VARIANTS = {"profiles": {"v2a": ["one_side_clip", "on_line_hold", "step_accept", "solid_cross_recovery",
                                        "corner_slowdown", "speed_schedule", "arc_comp"]},
                   "params": {"arc": {"shift_max_m": 0.4, "kappa_tau_sec": 0.4}, "corner": {"pwm": 190}}}
BASE_CASES = [("absent", None), ("named", "base")]


@pytest.mark.parametrize("tag,algo", BASE_CASES)
def test_find_path_frames(tag, algo):
    g = I.load_golden("base_path.json")
    n = I.check_path(C.LANE_MODULE, g, algo, "base_" + tag, variants=POISON_VARIANTS, tune_override=POISON_TUNE)
    assert n >= 1000


@pytest.mark.parametrize("tag,algo", BASE_CASES)
def test_sequence_30s(tag, algo):
    g = I.load_golden("base_seq.json")
    cur = I.check_seq(C.LANE_MODULE, g, algo, "base_" + tag, G.STATS_BASE_KEYS, variants=POISON_VARIANTS, tune_override=POISON_TUNE)
    for k in G.STATS_V1_KEYS:                           # base 에서는 변형 통계가 하나도 오르면 안 됨
        assert cur["stats"].get(k, 0) == 0, (k, cur["stats"].get(k))
    assert "inner_biased" not in cur["stats"]             # 삭제된 v3 잔재 없음


@pytest.mark.parametrize("tag,algo", BASE_CASES)
def test_controller_grid(tag, algo):
    g = I.load_golden("base_ctrl.json")
    I.check_ctrl(C.CTRL_MODULE, g, algo, "base_" + tag, variants=POISON_VARIANTS, tune_override=POISON_TUNE)


def test_sequence_covers_branches():
    """골든 시퀀스 자체가 설계한 분기를 모두 지나는지 (골든 파일의 stats·reason 으로 확인)."""
    g = I.load_golden("base_seq.json")
    st = g["data"]["stats"]
    for k in ("outlier_held", "outlier_accepted", "far_only_held", "cross_held", "search", "held", "transverse_dropped", "both", "one_side"):
        assert st[k] > 0, (k, st)
    reasons = " ".join(str(f[1]) for f in g["data"]["frames"])
    for s in ("search_timeout", "stale(", "no_mask", "no_line", "search_right", "search_left", "dashed_only", "solid_only"):
        assert s in reasons, s
    invalid = [f for f in g["data"]["frames"] if f[2] is not None and f[2][0] is False]
    assert invalid, "valid=false 발행이 없음"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-q"]))
