# -*- coding: utf-8 -*-
"""U7: 플래그 독립성 — 플래그 하나만 켠 프로필로 같은 합성 장면을 돌리면 그 플래그의 stats 키만 오른다. base 는 전부 0.
덧붙여 'frames=' 통계 로그 줄의 키 순서가 리팩터 전과 같은지(car_prep.sh / summarize_run.py 파싱) 확인.

실행: python3 -m pytest -q tests/test_flags_independent.py"""
import os
import re
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)
from test_solid_cross import (DT, VARIANT_KEYS, G, L, feed, frame, make_node, set_lane, variant)  # noqa: E402

# 플래그 → 그 플래그만 올릴 수 있는 stats 키
OWNS = {
    "one_side_clip": {"c_clipped"},
    "on_line_hold": {"on_line_held"},
    "step_accept": {"outlier_stepped"},
    "short_fit_guard": {"short_fit"},
    "solid_cross_recovery": {"search_back", "solid_on_line_accept"},
}
# short_fit 은 기본값(0.0 / 9.0)으로는 켜도 안 걸리므로 장면이 걸리게 span 하한을 준다
OVERRIDE = {"short_fit_guard": {"guard_ext_min_span_m": 0.5}}


def scenario(n, clock):
    """모든 플래그가 한 번씩 걸릴 수 있는 장면 (2차선 → 1차선). 총 7.5 s."""
    set_lane(n, 2)
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 1.0)                       # ref 확립
    feed(n, clock, frame(n, 2, dy=0.5, dyaw=0), 1.0)                       # 튐 0.5 m → outlier_hold 0.5 s → accepted / step
    feed(n, clock, frame(n, 2, dy=0.6, dyaw=0, drop=(0, 2)), 1.0)          # solid_only |c| 0.6 > 0.45 → c_clip
    feed(n, clock, frame(n, 2, dy=0.15, dyaw=0), 0.5)                      # ref 가 점선 반대쪽(c -0.15) 을 가리킴
    feed(n, clock, frame(n, 2, dy=0.42, dyaw=0, drop=(1, 2)), 0.5)         # dashed_only, 점선 밟음 → on_line_hold
    set_lane(n, 1)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 1.0)                       # 1차선 ref
    feed(n, clock, frame(n, 1, dy=0.5, dyaw=0, drop=(1, 2)), 0.5)          # 실선 밟음(STRADDLE) → solid_on_line_accept
    feed(n, clock, frame(n, 1, dy=0.8, dyaw=0, drop=(1, 2)), 0.5)          # 실선 넘음(CROSSED)
    feed(n, clock, frame(n, 1, dy=1.0, dyaw=0, drop=(0, 2)), 0.5)          # 경계 없음 → search_back / search_left
    set_lane(n, 2)
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0, drop=(0, 2), x_start=0.3), 0.5)   # 짧은 ext 적합 (span 0.4) → short_fit(상한 0.5 일 때)
    return dict(n.stats)


def run(tmp_path, flags, **over):
    n, clock = make_node(tmp_path, flags, cross_confirm_frames=1, cross_min_points=8, **over)
    return scenario(n, clock), n


@pytest.mark.parametrize("flag", sorted(OWNS))
def test_single_flag_only_raises_its_stats(tmp_path, flag):
    st, n = run(tmp_path, (flag,), **OVERRIDE.get(flag, {}))
    rose = {k for k in VARIANT_KEYS if st[k] > 0}
    assert rose, (flag, st)                                    # 장면이 그 플래그를 실제로 건드림
    assert rose <= OWNS[flag], (flag, rose, st)                # 다른 플래그의 통계는 0
    assert st["ok"] > 0


def test_base_raises_none(tmp_path):
    st, n = run(tmp_path, ())
    assert all(st[k] == 0 for k in VARIANT_KEYS), st
    assert st["search"] > 0 and st["outlier_accepted"] > 0 and st["ok"] > 0


def test_all_guards_raise_all(tmp_path):
    """v1 프로필(GUARDS) 로는 네 플래그의 통계가 모두 오른다 (장면 유효성)."""
    st, n = run(tmp_path, variant.GUARDS)
    for k in ("c_clipped", "on_line_held", "outlier_stepped", "search_back", "solid_on_line_accept"):
        assert st[k] > 0, (k, st)
    assert st["short_fit"] == 0                                # v1 에는 short_fit_guard 없음


def test_flag_sets_match_variant_module():
    """노드가 보는 플래그 이름이 variant.FLAGS 안에 있고, 경로 노드 플래그 5개가 모두 노드 소스에 쓰인다."""
    src = open(L.__file__, encoding="utf-8").read()
    for f in OWNS:
        assert f in variant.FLAGS and ('"%s"' % f) in src, f
    for k in VARIANT_KEYS:
        assert k in L.STATS_KEYS
    assert "inner_gain" not in src and "ALGO_LEVELS" not in src and "level" not in re.sub(r"#.*|\"\"\"[\s\S]*?\"\"\"", "", src)


HEAD_LINE_KEYS = ["frames", "ok", "held", "transverse_dropped", "outlier_held", "outlier_accepted", "far_only_held", "cross_held",
                  "dash_joined", "lane"]                       # HEAD a41706e 의 frames= 줄 (괄호 안 both/one_side/search 제외)
STATS_LINE_KEYS = HEAD_LINE_KEYS + ["algo"] + L.VARIANT_STATS_KEYS + ["last"]


def test_stats_log_line_format(tmp_path):
    """'frames=' 줄: HEAD 접두 그대로 + algo= + VARIANT_STATS_KEYS 순서 + last=. inner_biased(삭제된 v3) 없음. summarize_run.py 와 같은 방식으로 자른다."""
    n, clock = make_node(tmp_path, ())
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 0.2)
    n.log_stats()
    lines = [m for lv, m in n.get_logger().lines if "frames=" in m]
    assert lines, n.get_logger().lines
    line = lines[-1]
    assert re.match(r"^frames=\d+ ok=\d+ \(both \d+, one_side \d+, search \d+\) held=\d+ ", line), line
    keys = re.findall(r"(\w+)=", line.split(" last=")[0]) + ["last"]
    assert keys == STATS_LINE_KEYS, keys
    assert "inner_biased" not in line and set(L.STATS_KEYS) == set(L.BASE_STATS_KEYS) | set(L.VARIANT_STATS_KEYS)
    assert G.STATS_BASE_KEYS == L.BASE_STATS_KEYS and set(G.STATS_V1_KEYS) <= set(L.VARIANT_STATS_KEYS)   # 골든 키 목록은 노드 키의 부분집합
    assert " algo=test " in line and line.endswith("last=" + n.last_reason)
    assert ("]: " + line).split("]: ", 1)[-1] == line            # summarize_run.py: stats[-1].split("]: ", 1)[-1]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
