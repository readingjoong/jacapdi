# -*- coding: utf-8 -*-
"""U4: 리팩터가 벤치된 변형 동작을 보존하는지. 동결 사본(리팩터 전 작업 트리)으로 만든 v1 경로 골든·v2 제어기 골든과 현재 모듈 비교.

- v1: 가드 (a)(b)(c)(e)(f)(g) 그대로. 리팩터 뒤 confirm_frames 기본 2 는 골든(즉시 수용 1프레임)과 달라지므로
  variants.json 으로 cross.confirm_frames=1 을 준다 (V1_VARIANTS). 현재 코드는 이 파일을 읽지 않아 영향 없음.
  차선 변경 플래그(variant.LANE_FLAGS: lane_settle·change_slope·scope_hold, 10/31 추가) 는 골든 생성 때 없던 동작이므로 프로필 v1_legacy
  (= GUARDS − LANE_FLAGS) 로 뺀다. 그 플래그들 자체는 tests/test_lane_change.py(lane_settle·change_slope)·test_lane_scope.py(scope_hold) 가 검사한다.
- v2 제어기: 골든은 리팩터 전 v2(조향 비례 선형 감속, Stanley 분모 = tune vehicle_speed_mps). 사양 1.1/1.2 가 코너 감속을
  히스테리시스+저역통과 속도 스케줄러로 바꾸고(corner_slowdown) 그 속도를 Stanley 분모에 넣기로(speed_schedule) 결정했으므로
  pwm 열은 설계상 달라진다. 따라서 (1) speed_schedule 을 뺀 프로필 v2_legacy(가드 + corner_slowdown) 로 조향·shift·i_acc·kappa 열이
  골든과 == 인지 — 리팩터가 Stanley 경로를 바꾸지 않았음, (2) pwm 열은 새 스케줄러 규칙(tune pwm 이하, 코너 감속 존재)만 검사한다.
  inner_biased(삭제된 v3) 는 골든에만 있고 현재 stats 에는 없다."""
import os

import pytest

import conftest as C
import golden_lib as G
import identity_lib as I

def _variant_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location("variant_for_identity", os.path.join(C.NODES_DIR, "variant.py"))
    V = importlib.util.module_from_spec(spec); spec.loader.exec_module(V)
    return V


def _v1_legacy_variants():
    """골든 v1 과 같은 플래그 집합(가드만, 차선 변경 플래그 없음) + confirm_frames 1."""
    V = _variant_module()
    return {"profiles": {"v1_legacy": sorted(V.GUARDS - V.LANE_FLAGS)}, "params": {"cross": {"confirm_frames": 1}}}


def test_v1_find_path_frames():
    g = I.load_golden("v1_path.json")
    assert I.check_path(C.LANE_MODULE, g, "v1_legacy", "v1", variants=_v1_legacy_variants()) >= 1000


def test_v1_sequence_30s():
    g = I.load_golden("v1_seq.json")
    cur = I.check_seq(C.LANE_MODULE, g, "v1_legacy", "v1", G.STATS_BASE_KEYS + G.STATS_V1_KEYS, variants=_v1_legacy_variants())
    st = g["data"]["stats"]
    assert any(st[k] > 0 for k in ("c_clipped", "on_line_held", "outlier_stepped", "search_back", "solid_on_line_accept")), st
    assert "inner_biased" not in cur["stats"]


GRID_PWM_COLS = (6, 7)      # run_ctrl_grid 행: a b c num_points prev_steer | steer pwm_l pwm_r shift i_acc
SEQ_PWM_COLS = (1,)         # run_ctrl_seq 행: steer pwm shift i_acc kappa_cmd


def _v2_legacy_variants():
    """리팩터 전 v2 와 같은 플래그 집합(가드 + corner_slowdown, speed_schedule 없음) 을 variants.json 프로필로."""
    V = _variant_module()
    return {"profiles": {"v2_legacy": sorted((V.GUARDS - V.LANE_FLAGS) | {"corner_slowdown"})}}


def test_v2_controller_grid():
    g = I.load_golden("v2_ctrl.json")
    cur = I.check_ctrl(C.CTRL_MODULE, g, "v2_legacy", "v2", variants=_v2_legacy_variants(),
                       skip_grid_cols=GRID_PWM_COLS, skip_seq_cols=SEQ_PWM_COLS)
    # 감속이 실제로 골든에 들어 있는지 (pwm 이 tune 의 pwm 보다 낮은 격자점 존재)
    pwm = g["meta"]["tune"]["pwm"]
    assert any(row[6] is not None and row[6] < pwm for row in g["data"]["A"]["grid"])
    # 새 스케줄러(사양 1.2): 명령 pwm 은 tune pwm 이하, 코너(|조향| 큰 격자점)에서 감속이 존재, 좌우 동일
    for name, (grid, seq) in cur.items():
        pmax = g["meta"]["tune"]["pwm"] if name == "A" else G.TUNE_B_OVERRIDE["pwm"]
        rows = [r for r in grid if r[6] is not None]
        assert rows and all(r[6] == r[7] and 0 <= r[6] <= pmax for r in rows), name
        assert any(r[6] < pmax for r in rows), name
        assert all(r[1] is None or 0 <= r[1] <= pmax for r in seq), name


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-q"]))
