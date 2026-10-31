# -*- coding: utf-8 -*-
"""U6: SolidCross 상태기계 — 전이 (i)~(iv), confirm_frames, 1프레임 오검출 → ref 불변, 방향 스냅샷(search_max_sec 2.5 에서도 유지).

노드는 ROS 스텁(golden_lib.install_ros_stubs) + 주입 시계 + 고정 Variant(FixedVariant) 로 만든다. 합성 프레임은 tests/synth.py.
실행: python3 -m pytest -q tests/test_solid_cross.py   (rclpy 없이도 됨)"""
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)
import golden_lib as G  # noqa: E402

G.install_ros_stubs()
NODES_DIR = os.environ.get("JACAPDI_NODES_DIR", os.path.join(G.REPO_DIR, "nodes"))
if NODES_DIR not in sys.path:
    sys.path.insert(0, NODES_DIR)
import synth  # noqa: E402
import variant  # noqa: E402
import lane_path_node as L  # noqa: E402

DT = 0.05
VARIANT_KEYS = tuple(L.VARIANT_STATS_KEYS)   # 노드가 단일 출처


class FixedVariant:
    """파일을 읽지 않는 Variant 대역: flags 와 Params 를 고정. load() 는 아무것도 안 함."""

    def __init__(self, flags=(), name="test", **over):
        self.name = name; self.flags = frozenset(flags); self.p = variant.Params()
        for k, v in over.items():                  # 예: cross_confirm_frames=1 → p.cross.confirm_frames
            grp, fld = k.split("_", 1)
            setattr(getattr(self.p, grp), fld, v)

    def load(self, tune_dir=None, now=None):
        return False

    def set_lane(self, lane):
        return False


class Clock:
    """주입 시계. 스텁 Node.get_clock() 이 보는 golden_lib.CLOCK 도 같이 맞춘다 (메시지 stamp 나이 계산)."""
    def __init__(self, t0=1000.0): self.t = t0; G.CLOCK.set(t0)
    def __call__(self): return self.t
    def step(self, dt=DT): self.t += dt; G.CLOCK.set(self.t); return self.t


def make_node(tmp_path, flags=(), lane=2, **over):
    """주입 시계·고정 변형으로 만든 노드. (node, clock). tune 폴더는 빈 임시 폴더 (lane 파일 없음 → 2차선)."""
    td = str(tmp_path / "tune"); os.makedirs(td, exist_ok=True)
    L.LANE_FILE = os.path.join(td, "lane"); L.TUNE_FILE = os.path.join(td, "tune.json")
    clock = Clock()
    n = L.LanePathNode(now=clock, variant=FixedVariant(flags, **over))
    n.setup(G.H, G.W)
    set_lane(n, lane)
    return n, clock


def set_lane(n, lane):
    """차선 변경 S 자 없이 규칙만 바꾼다 (테스트 전용)."""
    n.lane.rules = n.lane.want = lane; n.lane.change = None


def feed(n, clock, dets, seconds, lane=None):
    """dets 프레임을 seconds 동안 20 fps 로 on_detections 에 넣고 마지막 reason 을 돌려준다."""
    reason = None
    for _ in range(int(round(seconds / DT))):
        t = clock.step()
        n.on_detections(G.detection_array(dets, t))
        reason = n.last_reason
    return reason


def frame(n, lane, **kw):
    return synth.lane_frame(n.ipm, lane, **kw)


def last_traj(n):
    return G.pubs(n)["/lane_trajectory"].msgs[-1]


# ---------------------------------------------------------------- SolidCross 단위 (노드 없이)
def cross(**over):
    p = variant.Cross()
    for k, v in over.items():
        setattr(p, k, v)
    return L.SolidCross(p)


def test_i_states_from_observation():
    """(i) NORMAL → STRADDLE → CROSSED 는 실선 y0 (mirror) 만으로 결정."""
    sc = cross()
    assert sc.state is L.LaneState.NORMAL
    sc.expire(0.0); sc.note_solid(0.0, -0.45); assert sc.state is L.LaneState.NORMAL and sc.confirm == 0
    sc.expire(0.05); sc.note_solid(0.05, 0.1); assert sc.state is L.LaneState.STRADDLE and sc.confirm == 1
    sc.expire(0.10); sc.note_solid(0.10, -0.2); assert sc.state is L.LaneState.STRADDLE and sc.confirm == 2   # |y0| ≤ near_m 0.25
    sc.expire(0.15); sc.note_solid(0.15, 0.4); assert sc.state is L.LaneState.CROSSED and sc.confirm == 0
    sc.expire(0.20); sc.note_solid(0.20, None); assert sc.state is L.LaneState.CROSSED   # 선은 있는데 실선 적합 없음 → 기억 유지
    assert sc.confirm == 0


def test_ii_expire_and_f_windows():
    """(ii) max_age_sec 지나면 기억 삭제 → (f) 후보 창·|c| 상한이 base 로 돌아온다."""
    sc = cross(max_age_sec=1.5)
    sc.expire(0.0); sc.note_solid(0.0, 0.1)
    assert sc.expire(1.0) == 0.1 and sc.entered()
    assert sc.cand_max(0.2) == sc.p.cand_max_cross_m == 0.6
    assert sc.c_limit(0.45, 0.3) == sc.p.c_max_cross_m == 0.7     # 넘은 실선(y0 > 0) 하나
    assert sc.c_limit(0.45, -0.3) == 0.45                          # 제자리 실선은 base 상한
    assert sc.expire(1.6) is None and not sc.entered()
    assert sc.cand_max(0.2) == 0.2 and sc.c_limit(0.45, 0.3) == 0.45
    assert sc.state is L.LaneState.NORMAL
    sc.expire(2.0); sc.note_solid(2.0, -0.1)                       # 차 밑(-0.2 < y0 ≤ 0) 도 '들어옴'
    sc.expire(2.05); assert sc.entered() and sc.cand_max(0.2) == 0.6
    sc.expire(2.10); sc.note_solid(2.10, -0.3); sc.expire(2.15); assert not sc.entered()


def test_iii_lost_direction_snapshot_survives_expiry():
    """(iii) LOST 진입 때 방향 스냅샷. search_max_sec 2.5 > max_age_sec 1.5 로 두어 기억이 지워져도 방향 유지, 상한 초과 → FAILED."""
    sc = cross(search_max_sec=2.5, max_age_sec=1.5)
    sc.expire(0.0); sc.note_solid(0.0, 0.3)                        # CROSSED
    sc.expire(0.5); sc.lose(0.5)
    assert sc.state is L.LaneState.LOST and sc.search_dir() == 1
    for t in (1.0, 2.0, 2.5, 2.9):                                 # 기억은 2.0 s 뒤 사라지지만 방향은 그대로
        sc.expire(t); sc.lose(t)
        assert sc.search_dir() == 1, t
    assert sc.expire(2.9) is None
    sc.timeout(); assert sc.state is L.LaneState.FAILED
    sc.expire(3.0); sc.note_solid(3.0, -0.4)                       # 경계 다시 보임 → 관측대로
    assert sc.state is L.LaneState.NORMAL and sc.lost_since is None
    # 넘지 않은 상태에서 잃으면 base 탐색 방향
    sc2 = cross(); sc2.expire(0.0); sc2.note_solid(0.0, -0.4); sc2.expire(0.1); sc2.lose(0.1)
    assert sc2.search_dir() == 0
    sc3 = cross(); sc3.expire(0.0); sc3.lose(0.0)                  # 기억 없음
    assert sc3.search_dir() == 0


def test_iv_accept_immediately_confirm_and_min_points():
    """(iv) 즉시 수용 술어: confirm_frames 연속 + num_points ≥ min_points + 차로 쪽 조건."""
    sc = cross(confirm_frames=2, min_points=15)
    T = L.LaneTrajectory
    new = T(c=-0.5, num_points=20); ref = T(c=0.0, num_points=20)
    sc.expire(0.0); sc.note_solid(0.0, 0.1)
    assert not sc.accept_immediately(new, ref)                     # 1 프레임째
    sc.expire(0.05); sc.note_solid(0.05, 0.12)
    assert sc.accept_immediately(new, ref)                         # 2 프레임 연속
    assert not sc.accept_immediately(T(c=-0.5, num_points=10), ref)             # 점 부족
    assert not sc.accept_immediately(T(c=0.2, num_points=20), T(c=0.3))         # 믿은 경로보다 덜 가리킴
    assert sc.accept_immediately(T(c=-0.2, num_points=20), T(c=0.3))            # 부호 반대 → 수용
    sc.expire(0.10); sc.note_solid(0.10, 0.4)                      # CROSSED → 연속 끊김
    assert sc.confirm == 0 and not sc.accept_immediately(new, ref)
    sc1 = cross(confirm_frames=1, min_points=0); sc1.expire(0.0); sc1.note_solid(0.0, 0.1)
    assert sc1.accept_immediately(T(c=-0.5, num_points=8), ref)    # v1 골든 조건 (confirm 1) 은 1 프레임에 수용


# ---------------------------------------------------------------- 노드 통합 (합성 프레임)
GUARDS = variant.GUARDS


def test_node_one_frame_false_detection_keeps_ref(tmp_path):
    """1차선, 실선을 밟은 1 프레임 오검출: confirm_frames 2 면 ref 불변(outlier_hold), 2 프레임 연속이면 solid_on_line_accept."""
    n, clock = make_node(tmp_path, GUARDS, lane=1, cross_confirm_frames=2, cross_min_points=8)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 1.0)
    ref_before = (n.ref.a, n.ref.b, n.ref.c)
    straddle = frame(n, 1, dy=0.5, dyaw=0, drop=(1, 2))           # 실선 안쪽 끝 mirror +0.125 → STRADDLE, solid_only
    r1 = feed(n, clock, straddle, DT)
    assert "solid_only" in r1 and "solid_on_line" in r1 and r1.startswith("outlier_hold"), r1
    assert (n.ref.a, n.ref.b, n.ref.c) == ref_before               # 1 프레임 오검출은 ref 를 못 바꿈
    assert n.stats["solid_on_line_accept"] == 0 and n.cross.state is L.LaneState.STRADDLE and n.cross.confirm == 1
    r2 = feed(n, clock, straddle, DT)
    assert r2.startswith("solid_on_line_accept"), r2
    assert n.stats["solid_on_line_accept"] == 1 and (n.ref.a, n.ref.b, n.ref.c) != ref_before
    # 같은 장면을 confirm_frames 1 로: 첫 프레임에 수용 (리팩터 전 v1 동작)
    n1, c1 = make_node(tmp_path, GUARDS, lane=1, cross_confirm_frames=1, cross_min_points=8)
    feed(n1, c1, frame(n1, 1, dy=0.0, dyaw=0), 1.0)
    assert feed(n1, c1, frame(n1, 1, dy=0.5, dyaw=0, drop=(1, 2)), DT).startswith("solid_on_line_accept")


def test_node_crossed_then_lost_search_back(tmp_path):
    """1차선: 실선 밟음 → 넘음(CROSSED, 후보 창 0.6) → 경계 없음 → search_back (실선 쪽), 탐색 상한 뒤 search_timeout."""
    n, clock = make_node(tmp_path, GUARDS, lane=1, cross_confirm_frames=1, cross_min_points=8, cross_search_max_sec=1.0)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 1.0)
    feed(n, clock, frame(n, 1, dy=0.5, dyaw=0, drop=(1, 2)), 0.3)   # STRADDLE
    assert n.cross.state is L.LaneState.STRADDLE
    r = feed(n, clock, frame(n, 1, dy=0.8, dyaw=0, drop=(1, 2)), 0.3)   # 실선 안쪽 끝 mirror +0.425: 넓힌 창(0.6) 안
    assert n.cross.state is L.LaneState.CROSSED, (n.cross.state, r)
    assert "solid_only" in r, r
    lost = frame(n, 1, dy=1.0, dyaw=0, drop=(0, 2))                # 점선만 mirror +1.425 → 후보 창 밖 → 경계 없음
    r = feed(n, clock, lost, 0.5)
    assert r.startswith("search_back"), r
    assert n.cross.state is L.LaneState.LOST and n.cross.search_dir() == 1
    assert n.stats["search_back"] >= 9 and last_traj(n).valid
    t = last_traj(n)
    assert t.c < 0                                                # 1차선: 실선(왼쪽)을 넘었으니 오른쪽(차로 쪽)으로 되돌아감
    if r.startswith("search_back_ref"):
        assert t.num_points > 0 and (t.a, t.b, t.c) == (n.ref.a, n.ref.b, n.ref.c)   # 믿은 경로가 이미 그쪽 → 그 경로
    else:
        assert t.num_points == 0 and t.c == -n.var.p.cross.search_back_m
    r = feed(n, clock, lost, 1.0)
    assert r.startswith("search_timeout"), r
    assert n.cross.state is L.LaneState.FAILED
    r = feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 0.3)           # 경계 복귀 → 정상
    assert r.startswith("solid+dashed") or r.startswith("outlier"), r
    assert n.cross.state is L.LaneState.NORMAL


def test_node_base_never_calls_solidcross(tmp_path):
    """base(플래그 공집합): 같은 장면을 돌려도 SolidCross 메서드 호출 0, 변형 통계 0, 탐색은 search_left."""
    n, clock = make_node(tmp_path, ())
    calls = []
    for name in ("expire", "note_solid", "lose", "timeout", "search_dir", "cand_max", "c_limit", "accept_immediately", "entered"):
        orig = getattr(n.cross, name)
        setattr(n.cross, name, (lambda nm, f: (lambda *a, **k: (calls.append(nm), f(*a, **k))[1]))(name, orig))
    set_lane(n, 1)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 1.0)
    feed(n, clock, frame(n, 1, dy=0.5, dyaw=0, drop=(1, 2)), 0.3)
    feed(n, clock, frame(n, 1, dy=0.8, dyaw=0, drop=(1, 2)), 0.3)
    r = feed(n, clock, frame(n, 1, dy=1.0, dyaw=0, drop=(0, 2)), 0.5)
    assert calls == [], calls
    assert r.startswith("search_left"), r
    assert all(n.stats[k] == 0 for k in VARIANT_KEYS) and "inner_biased" not in n.stats
    assert n.cross.state is L.LaneState.NORMAL                      # 아무 입력도 받지 않음


def test_path_result_reason_format():
    """reason() 문자열이 기존 접미·접두 형식을 그대로 만든다."""
    r = L.PathResult(object(), None, "dashed_only", "near", {"on_line", "c_clip"})
    assert r.reason() == "dashed_only,near,c_clip,on_line"
    r.wrap("outlier_hold(%.2fm,%s)", 0.123)
    assert r.reason() == "outlier_hold(0.12m,dashed_only,near,c_clip,on_line)"
    r.wrap("cross_hold(%.1fs,%s)", 0.5)
    assert r.reason() == "cross_hold(0.5s,outlier_hold(0.12m,dashed_only,near,c_clip,on_line))"
    assert L.PathResult(object(), None, "search", note="search_right(0.3s)").reason() == "search_right(0.3s)"
    assert L.fail_result("short_span").reason() == "short_span" and not L.fail_result("x").ok
    s = L.PathResult(object(), None, "solid_only", "ext", set()).wrap("solid_on_line_accept(%.2fm),%s", 0.5)
    assert s.reason() == "solid_on_line_accept(0.50m),solid_only,ext"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))


def _outer_solid_only(n, y0=-0.6, k=1.0):
    """1차선 급좌코너 재현(10/31 v3g_l1_r4 stamp 41.1~42.8): 안쪽 실선·점선은 화면 밖, 옆 차로 바깥 실선만 오른쪽에서 왼쪽 앞으로 휘어 보임.
    실제 좌표 y = y0 + k·x (x = IPM 창 좌표). mirror 로는 가까운 줄 +0.6, 먼 줄 < +0.2 → 먼 줄부터 첫 줄 후보 창(+0.2)에 걸림."""
    import math
    import numpy as np
    s = np.arange(-1.0, 3.0, 0.01); sc = synth.Scene(n.ipm)
    sc.add_line("solid_line", s, s, y0 + k * s, np.full_like(s, math.atan(k)), dashed=False)
    return sc.detections()


@pytest.mark.parametrize("cont", [False, True])
def test_node_foreign_outer_solid_not_crossed(tmp_path, cont):
    """cross_continuity: 실선이 차 밑을 지난 관측 없이 반대쪽에 처음 보이면 넘은 안쪽 실선이 아니다(옆 차로 바깥 실선).
    끄면(v2f) 그 실선을 '넘은 실선'으로 보고 경로가 오른쪽 c = -cross.c_max_cross_m(-0.70) 까지 간다 = 기록의 경로 뒤집힘.
    켜면 실선을 버리고(foreign_solid) 경계 없음과 같은 1차선 탐색(왼쪽)으로 간다."""
    flags = variant.V3_LANE1 if cont else variant.PROFILES["v2f"]
    n, clock = make_node(tmp_path, flags, lane=1)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 1.0)
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0, drop=(0,)), 2.0)     # 안쪽 실선이 화면 밖 (점선만) > cross.max_age_sec → 기억 없음
    assert n.cross.prev is None or n.cross.prev < n.var.p.cross.enter_m
    rs = [feed(n, clock, _outer_solid_only(n), DT) for _ in range(15)]
    t = last_traj(n)
    if not cont:
        assert any("solid_only" in r for r in rs) and t.c == pytest.approx(-n.var.p.cross.c_max_cross_m), (rs[-1], t.c)
        assert n.stats["foreign_solid"] == 0
    else:
        assert not any("solid_only" in r for r in rs), rs
        assert rs[-1].startswith("search_left") and t.c > 0, (rs[-1], t.c)   # 1차선 탐색 = 왼쪽(안쪽)
        assert n.stats["foreign_solid"] == 15 and n.cross.last is None    # 기억에도 안 남김 → (f) 창 확장 없음
