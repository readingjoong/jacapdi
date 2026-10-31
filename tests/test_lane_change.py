# -*- coding: utf-8 -*-
"""U8: 차선 변경 Lane 상태기계(lane_settle) 와 기울기 보상(change_slope).

- 상태 전이 IDLE → CHANGING → SETTLING → IDLE (시계 주입, lane 파일로 want 지정)
- 보류: CHANGING(되돌리기 아님)·SETTLING 중 새 want 는 IDLE 까지 보류, IDLE 진입 다음 update 에서 실행
- 되돌리기: CHANGING 중 원래 차선으로의 want 는 즉시 (그 자리에서 되돌아옴)
- 정착 타임아웃 lane.max_sec
- base 동일: settle=False 의 Lane 출력(offset, rules, change)이 HEAD 의 순수 시간 전환과 같다 (참조 구현을 여기 두고 비교)
- slope 부호·크기: 2→1 이면 b 양(왼쪽), 1→2 이면 음, 최대 = 0.85·1.5/(sec·v). 노드 publish 가 b 에 더하고 c 에는 오프셋만
실행: python3 -m pytest -q tests/test_lane_change.py"""
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)
import golden_lib as G  # noqa: E402

G.install_ros_stubs()
from test_solid_cross import DT, FixedVariant, Clock, feed, frame, last_traj, make_node, variant, L  # noqa: E402

W = L.LANE_WIDTH_M


def write_lane(path, lane):
    with open(path + ".tmp", "w") as f:
        f.write("%d\n" % lane)
    os.replace(path + ".tmp", path)


def bump_mtime(path, step):
    """같은 초 안의 연속 쓰기도 mtime 이 바뀐 것으로 보이게 (read_file 은 mtime 비교)."""
    st = os.stat(path); os.utime(path, (st.st_atime + step, st.st_mtime + step))


class LaneRig:
    """Lane 하나 + lane 파일 + 시계. set(lane) 는 파일을 쓰고, run(sec, c) 는 update(+observe) 를 DT 간격으로 돈다."""

    def __init__(self, tmp_path, settle=True, sec=2.0, **over):
        self.path = str(tmp_path / "lane"); write_lane(self.path, 2)
        self.t = 1000.0; self.logs = []; self.settle = settle; self.n = 0
        self.p = variant.LaneChange(**over)
        self.lane = L.Lane(self.logs.append, self.t, {"lane_change_sec": sec}, lane_file=self.path)
        self.lane.update(self.t, None, settle, self.p)

    def set(self, lane):
        self.n += 1; write_lane(self.path, lane); bump_mtime(self.path, self.n)

    def step(self, c=None):
        self.t += DT
        off = self.lane.update(self.t, None, self.settle, self.p)
        if c is not None and self.settle:
            self.lane.observe(self.t, c)
        return off

    def run(self, sec, c=None):
        out = []
        for _ in range(int(round(sec / DT))):
            out.append(self.step(c))
        return out


# ---------------------------------------------------------------- 상태 전이
def test_idle_changing_settling_idle(tmp_path):
    r = LaneRig(tmp_path, sec=2.0)
    L0 = r.lane
    assert (L0.state, L0.rules, L0.want) == (L0.IDLE, 2, 2)
    r.set(1); r.step(c=0.0)
    assert L0.state == L0.CHANGING and L0.rules == 2 and L0.change[3] == 1
    offs = r.run(1.9, c=0.0)
    assert L0.state == L0.CHANGING and L0.rules == 2                   # ramp 중엔 규칙 그대로 (순수 시간 기준은 유지)
    assert all(offs[i] <= offs[i + 1] + 1e-12 for i in range(len(offs) - 1)) and 0 < offs[-1] <= W
    r.run(0.2, c=-0.4)                                                  # ramp 끝: rules 1 + SETTLING, 차는 아직 못 옮김(|c| 0.4)
    assert (L0.state, L0.rules, L0.change) == (L0.SETTLING, 1, None)
    assert r.lane.offset_at(r.t) == 0.0
    r.run(1.0, c=-0.4)
    assert L0.state == L0.SETTLING                                      # |c| 가 크면 정착 아님
    r.run(0.3, c=0.05)
    assert L0.state == L0.SETTLING                                      # 0.5 s 연속이어야 함
    r.run(0.3, c=0.05)
    assert L0.state == L0.IDLE
    assert any("lane settled" in s for s in r.logs) and any("lane settling" in s for s in r.logs)


def test_settle_resets_on_excursion(tmp_path):
    r = LaneRig(tmp_path, sec=2.0)
    r.set(1); r.run(2.1, c=0.5)
    assert r.lane.state == r.lane.SETTLING
    r.run(0.4, c=0.05); r.run(0.1, c=0.3); r.run(0.4, c=0.05)          # 중간에 튀면 연속 카운트 리셋
    assert r.lane.state == r.lane.SETTLING
    r.run(0.2, c=0.05)
    assert r.lane.state == r.lane.IDLE


def test_settle_timeout(tmp_path):
    r = LaneRig(tmp_path, sec=2.0, max_sec=3.0)
    r.set(1); r.run(2.1, c=0.0)
    assert r.lane.state == r.lane.SETTLING
    r.run(2.8, c=-0.5)
    assert r.lane.state == r.lane.SETTLING
    r.run(0.3, c=-0.5)
    assert r.lane.state == r.lane.IDLE and any("timeout" in s for s in r.logs)


def test_settle_without_path_frames_still_times_out(tmp_path):
    """경로가 없는 프레임(observe 안 불림)만 이어져도 max_sec 뒤 IDLE (update 가 상한을 본다)."""
    r = LaneRig(tmp_path, sec=2.0, max_sec=1.0)
    r.set(1); r.run(2.1, c=0.0)
    r.run(1.2)                                                          # c=None → observe 안 함
    assert r.lane.state == r.lane.IDLE


# ---------------------------------------------------------------- 보류 / 되돌리기
def test_new_want_during_settling_is_held_then_executed(tmp_path):
    r = LaneRig(tmp_path, sec=2.0)
    r.set(1); r.run(2.1, c=0.0)
    assert r.lane.state == r.lane.SETTLING and r.lane.rules == 1
    r.set(2); r.run(0.5, c=-0.4)                                        # 정착 전 되돌아가기 요청 → 보류
    assert r.lane.state == r.lane.SETTLING and r.lane.change is None and r.lane.pending == 2
    assert sum("held" in s for s in r.logs) == 1
    r.run(1.0, c=-0.4); assert r.lane.pending == 2 and sum("held" in s for s in r.logs) == 1   # 로그 1회
    r.run(0.6, c=0.0)                                                   # 정착 → IDLE
    assert r.lane.state == r.lane.IDLE and r.lane.pending is None
    r.step(c=0.0)                                                       # 다음 update 에서 보류 실행
    assert r.lane.state == r.lane.CHANGING and r.lane.change[3] == 2 and r.lane.rules == 1
    assert r.lane.change[2] == -W                                       # 1→2: 오른쪽
    assert r.lane.pending is None


def test_revert_during_changing_is_immediate(tmp_path):
    r = LaneRig(tmp_path, sec=4.0)
    r.set(1); r.run(1.0, c=0.0)
    o_mid = r.lane.offset_at(r.t); assert 0 < o_mid < W
    r.set(2); r.step(c=0.0)                                             # 원래 차선(rules 2)으로 → 즉시 되돌리기
    assert r.lane.state == r.lane.CHANGING and r.lane.change[3] == 2 and r.lane.change[2] == 0.0
    assert abs(r.lane.change[1] - o_mid) < 0.05                         # 그 자리에서
    assert r.lane.pending is None and not any("held" in s for s in r.logs)
    offs = r.run(4.1, c=0.0)
    assert offs[-1] == 0.0 and r.lane.rules == 2 and r.lane.state == r.lane.SETTLING   # 되돌림 뒤에도 정착 확인


def test_rechange_during_revert_is_held(tmp_path):
    """되돌리기(목표 = rules) 중 다시 다른 차선 요청 → 보류, 되돌리기 끝 + 정착 뒤 실행."""
    r = LaneRig(tmp_path, sec=2.0)
    r.set(1); r.run(1.0, c=0.0); r.set(2); r.step(c=0.0)
    assert r.lane.change[3] == 2
    r.set(1); r.run(0.5, c=0.0)
    assert r.lane.change[3] == 2 and r.lane.pending == 1
    r.run(1.6, c=0.0)                                                   # 되돌리기 끝 → SETTLING
    assert r.lane.state == r.lane.SETTLING and r.lane.rules == 2 and r.lane.pending == 1
    r.run(0.6, c=0.0)
    assert r.lane.state == r.lane.CHANGING and r.lane.change[3] == 1


# ---------------------------------------------------------------- base 동일 (settle=False)
class HeadLane:
    """HEAD(a41706e) Lane.update 의 참조 구현 (파일 읽기 제외, want 를 직접 넣음)."""

    def __init__(self, sec):
        self.rules = 2; self.want = 2; self.change = None; self.sec = sec

    def offset_at(self, now):
        if self.change is None:
            return 0.0
        t0, o0, o1, _ = self.change
        s = min(max((now - t0) / max(self.sec, 1e-3), 0.0), 1.0)
        return o0 + (o1 - o0) * s * s * (3 - 2 * s)

    def update(self, now):
        goal = self.change[3] if self.change else self.rules
        if self.want != goal:
            o1 = 0.0 if self.want == self.rules else (W if self.want == 1 else -W)
            self.change = (now, self.offset_at(now), o1, self.want)
        if self.change and now - self.change[0] >= self.sec:
            if self.change[3] != self.rules:
                self.rules = self.change[3]
            self.change = None
        return self.offset_at(now)


@pytest.mark.parametrize("sec", [2.0, 4.0])
def test_flag_off_matches_head(tmp_path, sec):
    """settle=False: 2→1, 도중 되돌리기, 정착 전 재요청(HEAD 는 즉시 반영) 모두 HEAD 참조와 == ."""
    r = LaneRig(tmp_path, settle=False, sec=sec)
    ref = HeadLane(sec)
    script = {0.5: 1, 0.5 + sec * 0.4: 2, 0.5 + sec * 0.4 + 0.3: 1, 0.5 + sec * 0.4 + 0.3 + sec + 0.2: 2}
    t = 0.0
    for _ in range(int(round((sec * 3 + 3) / DT))):
        t = round(t + DT, 3)
        for when, lane in script.items():
            if abs(t - round(when, 3)) < 1e-9:
                r.set(lane); ref.want = lane
        off = r.step(c=0.3)
        assert off == ref.update(r.t)
        assert (r.lane.rules, r.lane.change) == (ref.rules, ref.change)
        assert r.lane.settle_t0 is None and r.lane.state in (r.lane.IDLE, r.lane.CHANGING)
    assert r.lane.rules == 2 and ref.rules == 2


def test_flag_off_node_publishes_no_slope(tmp_path):
    n, clock = make_node(tmp_path, ())
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 0.5)
    b0 = last_traj(n).b
    write_lane(L.LANE_FILE, 1)
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 1.0)
    assert n.slope == 0.0 and n.off > 0 and abs(last_traj(n).b - b0) < 1e-9 and n.lane.state == n.lane.CHANGING


# ---------------------------------------------------------------- slope
def test_slope_sign_and_magnitude(tmp_path):
    sec, v = 4.0, 0.85
    r = LaneRig(tmp_path, sec=sec)
    assert r.lane.slope_at(r.t, v) == 0.0
    r.set(1); r.step(c=0.0)
    slopes = [r.lane.slope_at(r.t, v) for _ in range(int(round(sec / DT))) if r.step(c=0.0) is not None]
    assert all(s >= 0 for s in slopes) and max(slopes) == pytest.approx(W * 1.5 / (sec * v), rel=0.02)   # 2→1: 왼쪽(+), 최대 6·¼·Δ/(sec·v)
    assert abs(slopes[-1]) < 1e-9                                       # ramp 끝 → 0
    r.run(0.6, c=0.0); assert r.lane.state == r.lane.IDLE
    r.set(2); r.step(c=0.0)
    s_back = [r.lane.slope_at(r.t, v) for _ in range(int(round(sec / DT))) if r.step(c=0.0) is not None]
    assert min(s_back) == pytest.approx(-W * 1.5 / (sec * v), rel=0.02) and all(s <= 0 for s in s_back)   # 1→2: 오른쪽(−)
    # 속도 가정이 느리면 기울기가 커진다 (ds 가 짧음)
    r.set(1); r.step(c=0.0); r.run(sec / 2, c=0.0)
    assert r.lane.slope_at(r.t, 0.5) > r.lane.slope_at(r.t, 1.0) > 0


def test_node_adds_slope_to_b_and_offset_to_c(tmp_path):
    n, clock = make_node(tmp_path, ("lane_settle", "change_slope"), lane_assumed_speed_mps=0.85)
    write_tune = os.path.join(os.path.dirname(L.LANE_FILE), "tune.json")
    with open(write_tune, "w") as f:
        f.write('{"lane_change_sec": 2.0}')
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 0.5)
    t0 = last_traj(n); assert n.off == 0.0 and n.slope == 0.0
    write_lane(L.LANE_FILE, 1)
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 1.0)                    # ramp 중간: off ≈ W/2, slope 최대
    t1 = last_traj(n)
    assert n.lane.state == n.lane.CHANGING and n.off == pytest.approx(W / 2, abs=0.05)   # 첫 프레임에 파일을 읽어 0.95 s 경과
    assert n.slope == pytest.approx(W * 1.5 / (2.0 * 0.85), rel=0.05) and n.slope > 0
    assert t1.c - t0.c == pytest.approx(n.off, abs=1e-6) and t1.b - t0.b == pytest.approx(n.slope, abs=1e-6)
    assert t1.a == t0.a
    feed(n, clock, frame(n, 2, dy=0.0, dyaw=0), 1.15)                   # ramp 끝(2.0 s + 여유) → rules 1, SETTLING, b 원복
    assert n.lane.rules == 1 and n.lane.state == n.lane.SETTLING and n.slope == 0.0 and n.off == 0.0
    feed(n, clock, frame(n, 1, dy=0.0, dyaw=0), 0.7)                    # 1차선 중앙 프레임 → |c| 작음 → 정착
    assert n.lane.state == n.lane.IDLE and n.slope == 0.0 and n.off == 0.0   # 정착 뒤 발행 경로에 보상 없음


def test_flags_in_variant_module():
    assert {"lane_settle", "change_slope"} <= variant.FLAGS and "lane_settle" in variant.PROFILES["v1"] and "change_slope" not in variant.PROFILES["v1"]
    assert "lane_settle" not in variant.PROFILES["base"]
    src = open(L.__file__, encoding="utf-8").read()
    assert '"lane_settle"' in src and '"change_slope"' in src


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
