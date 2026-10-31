# -*- coding: utf-8 -*-
"""차선 범위 프로필(variant.LaneScope, v3) 검사. 사용자 요구: 한 프로필로 2차선은 기존(base) 그대로, 1차선은 개선 알고리즘, 그 사이 차선 변경.

- 정적: v3 = lane_scope(V3_LANE1 | V3_CHANGE, ∅). 1차선 = V3_LANE1(= v2f) + V3_CHANGE, 2차선 = 프로필 수준 차선 변경 플래그(V3_CHANGE: lane_settle·scope_hold)만.
  LANE_FLAGS 는 두 차선이 같다(lane_scope 가 끌어올림). 기존 프로필(base/v1/v2/v2a/v2p/v2f)은 frozenset 그대로(두 차선 동일).
- 요청 차선 = tune/lane (parse_lane: 경로 노드 Lane.update 와 같은 규칙). Variant.load() 가 mtime 으로 읽고, 고정 변형(from_name)도
  차선 범위 프로필이면 차선 파일만 읽는다. variants.json {"lane1": [...], "lane2": [...]} 형식.
- 골든 증명(== 비교): v3 로 2차선만 달리면(변경 없음) 경로 노드 find_path 1000+ 프레임·30 s 시퀀스·제어기 격자/시퀀스가 base 골든과 비트 동일.
  30 s 골든 시퀀스(19 s 에 2→1 요청) 는 요청 전 프레임 전부가 골든과 같고, 요청 프레임부터 1차선 플래그. 1차선에서는 v2f 와 비트 동일.
- 플래그 차선(단일 출처 = 경로 노드 Lane 실제 상태): CHANGING 이면 scope_hold → 1·없으면 목표 차선, 그 밖(보류 중 포함)은 lane.rules.
  경로 노드 Variant 는 set_lane(), 제어기 Variant 는 경로 노드가 쓴 tune/lane_eff(부재·잘못된 값 → tune/lane). 실제 Lane+Variant 를 같은
  가짜 시계로 돌려 SETTLING 중 보류된 요청(2→1→2, 1→2→1, 정착 성공·max_sec 타임아웃)에서 S자 전 구간 1차선 플래그를 확인.
- 제어기: 플래그 차선이 바뀐 다음 tick 에 플래그 전환. 상태 리셋은 상태를 쓰는 플래그가 켜지거나 꺼질 때만
  (corner_slowdown → 스케줄러, arc_comp → κ_f). v2→v2a 처럼 유지되는 플래그의 상태는 이어감."""
import json
import os
import subprocess
import sys

import pytest

import conftest as C
import golden_lib as G
import identity_lib as I

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODES = os.path.join(ROOT, "nodes")
if NODES not in sys.path:
    sys.path.insert(0, NODES)
import variant  # noqa: E402
from variant import GUARDS, LANE_FLAGS, PROFILES, LaneScope, Variant, flags_at, lane_scope, parse_lane  # noqa: E402


def write(path, text):
    with open(path + ".tmp", "w") as f:
        f.write(text)
    os.replace(path + ".tmp", path)


# ---- 정적 ----
def test_v3_static():
    v3 = PROFILES["v3"]
    assert isinstance(v3, LaneScope)
    assert variant.V3_LANE1 == PROFILES["v2f"] | {"cross_continuity"}       # 1차선 자리 = v2f + (f) 전제 보정 (교체는 V3_LANE1 한 줄)
    assert v3.lane1 == variant.V3_LANE1 | variant.V3_CHANGE and variant.V3_CHANGE <= LANE_FLAGS
    assert v3.lane2 == variant.V3_CHANGE == frozenset({"lane_settle", "scope_hold"})   # 2차선 = base + 프로필 수준 변경 플래그
    assert v3.lane1 & LANE_FLAGS == v3.lane2 & LANE_FLAGS
    assert flags_at(v3, 1) == v3.lane1 and flags_at(v3, 2) == v3.lane2
    for name in ("base", "v1", "v2", "v2a", "v2p", "v2f"):                # 기존 프로필: 두 차선 같은 집합
        assert isinstance(PROFILES[name], frozenset) and flags_at(PROFILES[name], 1) == flags_at(PROFILES[name], 2)
    assert "v3" in variant.DRIVER_PROFILES and "v3" in variant.PROFILE_DESC
    assert "v2f" in PROFILES and "v2f" not in variant.DRIVER_PROFILES      # v2f 는 실험 프로필 (운전자에게는 v3 의 1차선으로만)


def test_lane_scope_lifts_lane_flags():
    s = lane_scope({"one_side_clip", "lane_settle"}, {"change_slope"})
    assert s.lane1 == {"one_side_clip", "lane_settle", "change_slope"} and s.lane2 == {"lane_settle", "change_slope"}


def test_parse_lane_rule():
    assert [parse_lane(x) for x in ("1", "2\n", " 1 ", "+2")] == [1, 2, 1, 2]
    assert [parse_lane(x) for x in (None, "", "3", "0", "x", "1.0")] == [None] * 6


# ---- Variant 가 tune/lane 을 읽음 ----
def test_variant_follows_lane_file(tmp_path):
    td = str(tmp_path); logs = []
    write(os.path.join(td, "algo"), "v3\n")
    v = Variant(td, log=logs.append, now=0.0)
    assert (v.name, v.lane, v.flags) == ("v3", 2, PROFILES["v3"].lane2)        # 부재 → 2
    write(os.path.join(td, "lane"), "1\n")
    assert v.load(now=1.0) is True and v.lane == 1 and v.flags == PROFILES["v3"].lane1
    assert any("v3 lane 1" in s for s in logs)
    for bad in ("x\n", "3\n", ""):                                              # 잘못된 값 → 직전 값
        write(os.path.join(td, "lane"), bad)
        assert v.load(now=2.0) is False and v.lane == 1
    os.remove(os.path.join(td, "lane"))                                         # 주행 중 부재 → 직전 값
    assert v.load(now=3.0) is False and v.lane == 1
    write(os.path.join(td, "lane"), "2\n")
    assert v.load(now=4.0) is True and v.flags == PROFILES["v3"].lane2
    write(os.path.join(td, "algo"), "v2a\n")                                    # 두 차선 같은 프로필: 차선이 바뀌어도 플래그 불변
    assert v.load(now=5.0) is True and v.flags == PROFILES["v2a"]
    write(os.path.join(td, "lane"), "1\n")
    assert v.load(now=6.0) is False and v.flags == PROFILES["v2a"] and v.lane == 1


def test_variant_start_on_lane1(tmp_path):
    td = str(tmp_path)
    write(os.path.join(td, "algo"), "v3\n"); write(os.path.join(td, "lane"), "1\n")
    v = Variant(td, now=0.0)
    assert v.lane == 1 and v.flags == PROFILES["v3"].lane1


def test_fixed_variant_reads_lane_only_when_scoped(tmp_path):
    td = str(tmp_path)
    write(os.path.join(td, "lane"), "1\n")
    v = Variant.from_name("v3", td)
    assert v.fixed and v.flags == PROFILES["v3"].lane1
    write(os.path.join(td, "algo"), "base\n")                                   # 고정 변형은 tune/algo 를 안 읽음
    write(os.path.join(td, "lane"), "2\n")
    assert v.load(now=1.0) is True and v.name == "v3" and v.flags == PROFILES["v3"].lane2
    w = Variant.from_name("v2a", td)
    write(os.path.join(td, "lane"), "1\n")
    assert w.load(now=2.0) is False and w.flags == PROFILES["v2a"]


def test_variants_json_scoped_profile(tmp_path):
    td = str(tmp_path); logs = []
    write(os.path.join(td, "variants.json"), json.dumps({"profiles": {
        "x": {"lane1": ["one_side_clip", "lane_settle", "bogus"], "lane2": []},
        "y": {"lane1": [], "lanes": []},
        "z": {"lane1": ["speed_schedule"]},
        "base": {"lane1": ["one_side_clip"]}}}))
    write(os.path.join(td, "algo"), "x\n")
    v = Variant(td, log=logs.append, now=0.0)
    assert v.profiles["x"] == LaneScope(frozenset({"one_side_clip", "lane_settle"}), frozenset({"lane_settle"}))
    assert "y" not in v.profiles and v.profiles["base"] == frozenset()
    assert v.profiles["z"] == LaneScope(frozenset(), frozenset())                 # REQUIRES 위반 제거 (차선별 검사)
    joined = "\n".join(logs)
    assert "bogus" in joined and "프로필 수준" in joined and "corner_slowdown 가 필요" in joined
    assert v.flags == frozenset({"lane_settle"})


def test_cli_show_and_check_v3():
    r = subprocess.run([sys.executable, os.path.join(NODES, "variant.py"), "--show", "v3"], capture_output=True, text=True)
    assert r.returncode == 0
    lines = r.stdout.splitlines()
    assert any(l.startswith("flags[1차선]: ") and "front_axle_ref" in l and "arc_comp" not in l for l in lines)
    assert "flags[2차선]: lane_settle scope_hold" in lines
    assert subprocess.run([sys.executable, os.path.join(NODES, "variant.py"), "--check", "v3"]).returncode == 0


# ---- 플래그 차선: 경로 노드는 set_lane(Lane 실제 상태), 제어기는 tune/lane_eff ----
def _hold_profile(td, name="h"):
    write(os.path.join(td, "variants.json"), json.dumps({"profiles": {name: {"lane1": sorted(variant.V3_LANE1 | {"scope_hold"}), "lane2": []}}}))
    return LaneScope(variant.V3_LANE1 | {"scope_hold"}, frozenset({"lane_settle", "scope_hold"}))


def _request_profile(td, name="r", lane1=None):
    """scope_hold 없는 차선 범위 프로필 (변경 중 플래그 = 목표 차선). lane1 기본 = V3_LANE1."""
    lane1 = variant.V3_LANE1 if lane1 is None else frozenset(lane1)
    write(os.path.join(td, "variants.json"), json.dumps({"profiles": {name: {"lane1": sorted(lane1), "lane2": []}}}))
    return lane_scope(lane1, frozenset())


def test_set_lane_overrides_lane_files(tmp_path):
    """경로 노드 쪽: set_lane() 을 부르면 tune/lane·tune/lane_eff 와 무관하게 그 차선으로 플래그를 푼다."""
    td = str(tmp_path); want = _hold_profile(td)
    write(os.path.join(td, "algo"), "h\n"); write(os.path.join(td, "lane"), "1\n")
    v = Variant(td, now=0.0)
    assert v.flags == want.lane1
    assert v.set_lane(2) is True and (v.lane, v.eff_lane, v.flags) == (1, 2, want.lane2)
    write(os.path.join(td, "lane_eff"), "1\n"); write(os.path.join(td, "lane"), "1\n")
    assert v.load(now=1.0) is False and v.flags == want.lane2                    # 파일은 플래그 차선을 바꾸지 않음
    assert v.set_lane(2) is False and v.set_lane(1) is True and v.flags == want.lane1
    w = Variant.from_name("v2a", td)                                            # 두 차선 같은 프로필: 차선과 무관
    assert w.set_lane(1) is False and w.flags == PROFILES["v2a"]


def test_controller_variant_follows_lane_eff(tmp_path):
    """(d) 제어기 쪽 Variant: tune/lane_eff 를 따르고, 부재·잘못된 값이면 tune/lane(요청 차선), 시작 때 둘 다 부재면 2."""
    td = str(tmp_path); want = _hold_profile(td); L = os.path.join(td, "lane"); E = os.path.join(td, "lane_eff")
    write(os.path.join(td, "algo"), "h\n")
    v = Variant(td, now=0.0)
    assert (v.lane, v.eff_lane, v.flags) == (2, 2, want.lane2)                 # 둘 다 부재 → 2
    write(E, "1\n")
    assert v.load(now=1.0) is True and (v.lane, v.eff_lane, v.flags) == (2, 1, want.lane1)   # 요청 2 여도 플래그 차선 1
    write(L, "1\n"); write(E, "2\n")
    assert v.load(now=2.0) is True and (v.lane, v.eff_lane, v.flags) == (1, 2, want.lane2)
    for bad in ("x\n", "3\n", ""):                                             # 잘못된 값 → 요청 차선 1
        write(E, bad)
        v.load(now=3.0)
        assert (v.eff_lane, v.flags) == (1, want.lane1)
    write(E, "2\n"); v.load(now=4.0)
    os.remove(E)                                                                # 부재 → 요청 차선
    assert v.load(now=5.0) is True and (v.eff_lane, v.flags) == (1, want.lane1)
    write(E, "2\n")
    u = Variant(td, now=6.0)                                                    # 시작 때 lane_eff 가 있으면 그 값
    assert (u.lane, u.eff_lane, u.flags) == (1, 2, want.lane2)
    f = Variant.from_name("h", td)                                              # 고정 변형도 같은 규칙
    assert f.flags == want.lane2
    write(E, "1\n")
    assert f.load(now=7.0) is True and f.flags == want.lane1


def _settle_run(tag, algo, start, first, second, dy_until5, dy_after, variants=None):
    """실제 경로 노드(Lane + Variant)와 제어기 쪽 Variant(같은 tune 폴더, tune/lane_eff 를 읽음)를 같은 가짜 시계로 20 s 돌린다.
    1 s 에 start→first 요청(즉시 S자), 4 s(첫 변경의 SETTLING 중)에 first→second 요청(보류). 3 s 부터 first 차선 프레임:
    5 s 전 옆오차 dy_until5, 뒤 dy_after (0 → |c| < settle_c_m 로 정착 성공, 0.3 → lane.max_sec 타임아웃)."""
    import synth
    tune = dict(I.load_golden("base_seq.json")["meta"]["tune"]); tune["lane_change_sec"] = 2.0
    td = G.make_tune_dir(os.path.join(I.workdir(), "tune_ls_settle_" + tag), tune, algo=algo, lane=start, variants=variants)
    M = G.load_module(C.LANE_MODULE, "cur_lane_settle_" + tag, td)
    node = G.new_lane_node(M)
    ctrl = Variant(td)                                                          # 제어기와 같은 규칙 (set_lane 안 부름)
    rows = []; t = 0.0
    while t < 20.0 - 1e-9:
        if t < 3.0:
            dets = synth.lane_frame(node.ipm, start, dy=0.0, dyaw=0)
        else:
            dets = synth.lane_frame(node.ipm, first, dy=dy_until5 if t < 5.0 else dy_after, dyaw=0)
        G.CLOCK.set(G.T0 + t)
        req = first if abs(t - 1.0) < 1e-6 else second if abs(t - 4.0) < 1e-6 else None
        if req is not None:
            write(os.path.join(td, "lane"), "%d\n" % req)
        node.on_detections(G.detection_array(dets, G.T0 + t))
        ctrl.load(now=G.T0 + t)
        ln = node.lane
        rows.append(dict(t=t, st=ln.state, rules=ln.rules, goal=ln.change[3] if ln.change else None, pending=ln.pending,
                         flags=node.var.flags, ctrl=ctrl.flags, eff=read_lane_eff(td)))
        t = round(t + 0.05, 3)
    return rows


def read_lane_eff(td):
    with open(os.path.join(td, "lane_eff")) as f:
        return parse_lane(f.read())


@pytest.mark.parametrize("start,first,second", [(2, 1, 2), (1, 2, 1)])        # (a) 피했다 바로 복귀 2→1→2, (b) 반대 1→2→1
@pytest.mark.parametrize("settle", ["ok", "timeout"])                            # (c) 정착 성공 / max_sec 타임아웃
def test_scope_hold_follows_lane_state_through_held_request(start, first, second, settle):
    """v3(scope_hold): 플래그 = CHANGING 이면 1차선, 그 밖은 lane.rules. 보류 중에는 지금 차선, ramp 전 구간은 1차선 플래그.
    제어기 쪽 Variant 는 tune/lane_eff 로 같은 프레임 같은 플래그."""
    dy_after = 0.0 if settle == "ok" else 0.3
    rows = _settle_run("v3_%d%d%d_%s" % (start, first, second, settle), "v3", start, first, second, 0.3, dy_after)
    v3 = PROFILES["v3"]
    for r in rows:
        lane = 1 if r["goal"] is not None else r["rules"]
        assert r["flags"] == v3.at(lane), r
        assert r["ctrl"] == r["flags"] and r["eff"] == lane, r
    held = [r for r in rows if r["pending"] == second]
    assert held and all(r["st"] == "SETTLING" and r["rules"] == first and r["flags"] == v3.at(first) for r in held)   # 보류 중 = 지금 차선 플래그
    ramp2 = [r for r in rows if r["goal"] == second]
    t2 = ramp2[0]["t"]
    assert all(r["flags"] == v3.lane1 for r in ramp2)                           # 두 번째 S자 전 구간 1차선 플래그
    assert t2 >= 5.0 and ramp2[-1]["t"] >= t2 + 2.0 - 0.1                       # 요청(4 s) + ramp(2 s) 보다 늦게 끝나는 S자 (옛 요청 시각 기준 창의 결함 구간)
    if settle == "ok":
        assert t2 < 3.0 + variant.LaneChange().max_sec - 1.0
    else:
        assert abs(t2 - (3.0 + variant.LaneChange().max_sec)) <= 0.1
    after = [r for r in rows if r["t"] > ramp2[-1]["t"]]
    assert after and all(r["rules"] == second for r in after)
    l2 = [r for r in rows if r["flags"] == v3.lane2 and r["t"] > 1.0]
    assert all(r["rules"] == 2 and r["goal"] is None for r in l2)              # 2차선 플래그는 rules 가 2 인 뒤에만
    assert l2                                                                   # (a)는 복귀 뒤, (b)는 첫 변경 뒤


def test_no_scope_hold_flags_follow_change_target(tmp_path):
    """scope_hold 없는 차선 범위 프로필: CHANGING 이면 목표 차선 플래그(즉시 처리되는 요청 = 요청 프레임부터), 보류 중은 지금 차선."""
    lane1 = variant.V3_LANE1
    prof = {"profiles": {"r": {"lane1": sorted(lane1), "lane2": []}}}
    want = lane_scope(lane1, frozenset())
    rows = _settle_run("r_212", "r", 2, 1, 2, 0.3, 0.3, variants=prof)
    for r in rows:
        lane = r["goal"] if r["goal"] is not None else r["rules"]
        assert r["flags"] == want.at(lane) and r["ctrl"] == r["flags"], r
    first_req = next(r for r in rows if r["t"] >= 1.0 - 1e-9)
    assert first_req["goal"] == 1 and first_req["flags"] == want.lane1         # 요청 프레임부터 1차선 플래그
    assert any(r["pending"] == 2 and r["flags"] == want.lane1 for r in rows)  # 보류 중 1차선 유지
    ramp2 = [r for r in rows if r["goal"] == 2]
    assert ramp2 and all(r["flags"] == want.lane2 for r in ramp2)             # 1→2 S자 시작부터 2차선 플래그


def test_controller_follows_lane_eff_and_resets_on_toggle(tmp_path):
    """제어기: 플래그 차선은 경로 노드가 쓴 tune/lane_eff (요청 차선 tune/lane 과 달라도). corner_slowdown 이 꺼지면 스케줄러 리셋."""
    import test_controller as TC
    tc = TC._import(TC.CTRL, "tc_lane_scope_eff")
    tdir = tmp_path / "tune_h"; tdir.mkdir()
    want = _hold_profile(str(tdir))
    write(str(tdir / "lane"), "2\n"); write(str(tdir / "lane_eff"), "1\n"); write(str(tdir / "go_until"), "1e12\n")
    n = TC.make(tc, tmp_path, name="h")
    assert n.var.lane == 2 and n.var.flags == want.lane1                        # 1→2 S자 중(경로 노드가 1 을 씀)
    for _ in range(19):
        TC.feed(n, TC._Traj(a=0.3, c=0.1, num_points=40))
    assert n.var.flags == want.lane1 and n.sched.pwm is not None and "front_axle_ref" in n.var.flags
    write(str(tdir / "lane_eff"), "2\n")                                        # ramp 끝: 경로 노드가 2 를 씀 → 다음 tick
    TC.feed(n, TC._Traj(a=0.0, c=0.0, num_points=40))
    assert n.var.flags == want.lane2 and n.sched.pwm is None and n.kappa_f is None


# ---- 골든: 2차선만 달리면 base 와 비트 동일 ----
@pytest.mark.parametrize("lane", [None, 2])
def test_v3_lane2_find_path_equals_base_golden(lane):
    g = I.load_golden("base_path.json")
    assert I.check_path(C.LANE_MODULE, g, "v3", "v3_l%s" % lane, lane=lane) >= 1000


@pytest.mark.parametrize("lane", [None, 2])
def test_v3_lane2_controller_equals_base_golden(lane):
    g = I.load_golden("base_ctrl.json")
    I.check_ctrl(C.CTRL_MODULE, g, "v3", "v3_l%s" % lane, lane=lane)


def _run_seq(algo, lane, tag, drop_lane_requests):
    """30 s 골든 대본을 algo·시작 차선으로 돌린다. drop_lane_requests: 19 s 의 2→1 요청을 뺀 '변경 없음' 대본."""
    g = I.load_golden("base_seq.json")
    td = G.make_tune_dir(os.path.join(I.workdir(), "tune_ls_" + tag), dict(g["meta"]["tune"]), algo=algo, lane=lane)
    M = G.load_module(C.LANE_MODULE, "cur_lane_ls_" + tag, td)
    node = G.new_lane_node(M)
    steps = G.sequence_steps(node.ipm)
    if drop_lane_requests:
        steps = [dict(s, lane=None) for s in steps]
    return G.run_sequence(M, node, steps, td), node, g


def test_v3_lane2_sequence_no_change_equals_base():
    """변경 없는 30 s 대본(2차선 출발): v3 == 현재 base (현재 base == HEAD 골든은 test_base_identity 가 증명)."""
    cur, node, _ = _run_seq("v3", 2, "v3_nochg", True)
    ref, _, _ = _run_seq("base", 2, "base_nochg", True)
    I.assert_same(ref, cur, "v3 2차선 변경 없음 vs base")
    assert node.var.flags == PROFILES["v3"].lane2


def test_v3_golden_sequence_prefix_and_switch():
    """골든 대본 그대로(19 s 에 2→1): 요청 전 프레임은 base 골든과 ==, 요청 프레임부터 1차선 플래그."""
    cur, node, g = _run_seq("v3", None, "v3_gold", False)
    n19 = next(i for i, f in enumerate(g["data"]["frames"]) if f[0] >= 19.0 - 1e-9)
    I.assert_same(g["data"]["frames"][:n19], cur["frames"][:n19], "v3 요청 전 프레임 vs base 골든")
    assert node.var.lane == 1 and node.var.flags == PROFILES["v3"].lane1
    assert cur["frames"][n19:] != g["data"]["frames"][n19:]                   # 1차선 플래그(가드)가 실제로 켜짐


def test_v3_lane1_sequence_equals_v2f():
    """1차선 출발·변경 없음: v3 경로 노드 출력 == v2f (경로 노드는 제어기 플래그를 보지 않으므로 1차선 집합 = v2f 경로 동작)."""
    cur, node, _ = _run_seq("v3", 1, "v3_l1", True)
    ref, _, _ = _run_seq("v2f", 1, "v2f_l1", True)
    I.assert_same(ref, cur, "v3 1차선 vs v2f")
    assert node.var.flags == PROFILES["v2f"] | variant.V3_CHANGE | {"cross_continuity"}   # 변경 플래그는 변경이 없으면 출력에 영향 없음. cross_continuity 는 이 대본(옆 차로 실선 없음)에선 출력 불변


def test_v3_lane1_controller_equals_v2f():
    g = I.load_golden("base_ctrl.json")
    out = {}
    for algo in ("v3", "v2f"):
        tj = dict(g["meta"]["tune"])
        td = G.make_tune_dir(os.path.join(I.workdir(), "tune_ls_ctrl_" + algo), tj, algo=algo, lane=1)
        M = G.load_module(C.CTRL_MODULE, "cur_ctrl_ls_" + algo, td)
        out[algo] = (G.run_ctrl_grid(M, G.new_controller(M)), G.run_ctrl_seq(M, G.new_controller(M)))
    I.assert_same(out["v2f"], out["v3"], "제어기 v3 1차선 vs v2f")


# ---- 제어기: 차선 전환 시점·상태 리셋 ----
@pytest.mark.parametrize("lane1,has_kappa_f", [("v2f", False), ("v2a", True)])   # v2f(= V3_LANE1): front_axle_ref 는 κ_f 를 쓰지 않음(매 주기 None)
def test_controller_switches_flags_next_tick_and_resets_state(tmp_path, lane1, has_kappa_f):
    import test_controller as TC
    tc = TC._import(TC.CTRL, "tc_lane_scope")
    tdir = tmp_path / "tune_r"; tdir.mkdir()
    want = _request_profile(str(tdir), lane1=PROFILES[lane1])                  # 경로 노드 없음(lane_eff 부재) → 요청 차선 tune/lane 으로 대체, 다음 tick 에 전환
    n = TC.make(tc, tmp_path, name="r")
    lane_file = os.path.join(n.tune_dir, "lane")
    write(os.path.join(n.tune_dir, "go_until"), "1e12\n")                      # 출발 스위치 ON (OFF 면 스케줄러가 매 tick 리셋)
    assert n.var.flags == want.lane2
    write(lane_file, "1\n")
    TC.feed(n, TC._Traj(a=0.3, c=0.1, num_points=40))                           # 다음 tick 부터 1차선 플래그
    assert n.var.lane == 1 and n.var.flags == want.lane1
    assert n.sched.pwm is not None and (n.kappa_f is not None) == has_kappa_f
    for _ in range(20):
        TC.feed(n, TC._Traj(a=0.3, c=0.1, num_points=40))
    write(lane_file, "2\n")
    TC.feed(n, TC._Traj(a=0.0, c=0.0, num_points=40))
    assert n.var.flags == want.lane2 and n.sched.pwm is None and n.kappa_f is None   # corner_slowdown 꺼짐 → 스케줄러 리셋, arc_comp 꺼짐(v2a) → κ_f 리셋
    write(lane_file, "1\n")
    TC.feed(n, TC._Traj(a=0.0, c=0.0, num_points=40))
    assert n.sched.pwm is not None and abs(n.sched.pwm - n.p["pwm"]) < 1e-6 and not n.sched.engaged   # 멈춘 상태가 아니라 새로 시작


def test_controller_keeps_state_when_stateful_flags_kept(tmp_path):
    """상태를 쓰는 플래그가 유지되면 리셋 없음: v2→v2a 는 corner_slowdown 유지 → 스케줄러 이어감, arc_comp 켜짐 → κ_f 새로 시작.
    v2a→v2f 는 corner_slowdown 유지 → 스케줄러 이어감, arc_comp 꺼짐 → κ_f 리셋."""
    import test_controller as TC
    tc = TC._import(TC.CTRL, "tc_lane_scope_keep")
    n = TC.make(tc, tmp_path, name="v2")
    write(os.path.join(n.tune_dir, "go_until"), "1e12\n")
    for _ in range(40):                                                         # 큰 조향으로 코너 상태 진입·감속
        TC.feed(n, TC._Traj(a=1.0, c=0.6, num_points=40))
    assert n.sched.engaged and n.sched.pwm < n.p["pwm"] - 1.0 and n.kappa_f is None
    pwm0 = n.sched.pwm
    write(os.path.join(n.tune_dir, "algo"), "v2a\n")
    n.var.load(); n.tick(); n.clock.step()                                       # 플래그 전환 tick (경로는 직전 것)
    assert n.var.flags == PROFILES["v2a"] and n.sched.engaged and abs(n.sched.pwm - pwm0) < 5.0   # 스케줄러 이어감 (리셋이면 pwm = tune pwm)
    assert n.kappa_f is not None
    k_hist = []
    for _ in range(5):
        TC.feed(n, TC._Traj(a=1.0, c=0.6, num_points=40)); k_hist.append(n.kappa_f)
    write(os.path.join(n.tune_dir, "algo"), "v2f\n")
    TC.feed(n, TC._Traj(a=1.0, c=0.6, num_points=40))
    assert n.var.flags == PROFILES["v2f"] and n.sched.engaged and n.kappa_f is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
