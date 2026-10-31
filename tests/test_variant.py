# -*- coding: utf-8 -*-
"""U5: 변형 로더(nodes/variant.py). ROS 없이 돈다: python3 -m pytest tests/test_variant.py
항목: 부재→base, 모름→base+로그 1회, 파일 삭제 시 두 노드 동일 전이, 프로필 포함 관계,
variants.json 키 ⊆ dataclass + 기본값 복제 없음(override 전용), DRIVER_PROFILES ⊆ PROFILES, max_age_sec ≥ search_max_sec,
REQUIRES/CONSTRAINTS, 비 UTF-8 바이트에서 죽지 않음, 고정 변형(from_name) 은 파일을 안 읽음, 미래 mtime 재읽기 없음, CLI."""
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODES = os.path.join(ROOT, "nodes")
if NODES not in sys.path:
    sys.path.insert(0, NODES)
import variant  # noqa: E402
from variant import FLAGS, GUARDS, PROFILES, DRIVER_PROFILES, Params, Variant  # noqa: E402

REPO_VARIANTS = os.path.join(ROOT, "tune", "variants.json")
REPO_TUNE = os.path.join(ROOT, "tune", "tune.json")
HEAD_DRIVER_KEYS = {"stanley_gain", "heading_gain", "vehicle_speed_mps", "curvature_gain", "softening_speed_mps",
                    "front_axle_offset_m", "max_steer_deg", "i_gain", "i_max_deg", "i_curv_max", "lane_timeout_sec",
                    "pwm", "lane_change_sec"}


def write(path, text):
    with open(path + ".tmp", "w") as f:
        f.write(text)
    os.replace(path + ".tmp", path)


@pytest.fixture
def tdir(tmp_path):
    return str(tmp_path)


# ---- 정적 계약 ----
def test_profiles_static_contract():
    assert set(DRIVER_PROFILES) <= set(PROFILES)
    assert GUARDS <= FLAGS
    for name, fl in PROFILES.items():
        assert isinstance(fl, frozenset) and fl <= FLAGS, name
    assert PROFILES["base"] == frozenset()
    assert PROFILES["v1"] == GUARDS
    assert "lane_settle" in GUARDS and "change_slope" not in GUARDS and variant.LANE_FLAGS == {"lane_settle", "change_slope"}
    assert PROFILES["base"] < PROFILES["v1"] < PROFILES["v2"] < PROFILES["v2a"]
    assert PROFILES["v2"] < PROFILES["v2p"]
    assert {"arc_comp", "controller_pp"} - PROFILES["v2"] == {"arc_comp", "controller_pp"}
    for name, fl in PROFILES.items():
        assert not ({"arc_comp", "controller_pp"} <= fl), "%s: arc_comp 와 controller_pp 는 배타" % name
    assert DRIVER_PROFILES[0] == "base"


def test_params_defaults_single_source():
    p = Params()
    assert (p.cross.search_back_m, p.cross.search_max_sec, p.cross.hold_last_sec) == (0.3, 1.0, 0.5)
    assert (p.cross.cand_max_cross_m, p.cross.c_max_cross_m, p.cross.near_m, p.cross.max_age_sec) == (0.6, 0.7, 0.25, 1.5)
    assert (p.cross.confirm_frames, p.cross.min_points) == (2, 15)
    assert (p.guard.one_side_max_c_m, p.guard.on_line_half_m, p.guard.on_line_hold_sec, p.guard.on_line_min_ref_c) == (0.45, 0.25, 0.3, 0.05)
    assert (p.guard.accept_step_m, p.guard.ext_min_span_m, p.guard.ext_max_b) == (0.3, 0.0, 9.0)   # 벤치가 돌아간 디스크 값
    assert (p.corner.steer_on_pct, p.corner.steer_off_pct, p.corner.pwm, p.corner.tau_sec) == (60.0, 45.0, 190, 0.3)
    assert (p.cross.enter_m, p.cross.crossed_m, p.cross.overlap_m) == (-0.2, 0.2, 0.3)
    assert (p.arc.kappa_tau_sec, p.arc.shift_max_m, p.arc.min_points) == (0.4, 0.25, 6)
    assert p.pp.lookahead_x_m == 0.1
    assert p.cross.max_age_sec >= p.cross.search_max_sec
    assert (p.lane.settle_c_m, p.lane.settle_sec, p.lane.max_sec, p.lane.assumed_speed_mps) == (0.15, 0.5, 8.0, 0.85)
    assert set(variant.param_keys()) == {"cross", "guard", "corner", "arc", "pp", "lane"}


def test_repo_variants_json_is_override_only():
    """저장소 tune/variants.json: 키 ⊆ Params 필드, 코드 기본값과 같은 항목(복제) 없음, 코드 PROFILES 와 같은 프로필 재정의 없음,
    오류 없이 읽힘. 단일 출처는 코드이고 파일은 override 만 담는다."""
    doc = json.load(open(REPO_VARIANTS))
    assert set(doc) <= {"profiles", "params"}
    keys = variant.param_keys(); defaults = Params().to_dict()
    for g, kv in doc.get("params", {}).items():
        assert g in keys, g
        assert set(kv) <= keys[g], (g, set(kv) - keys[g])
        dup = {k for k, v in kv.items() if v == defaults[g][k]}
        assert not dup, "variants.json params.%s 가 코드 기본값을 복제함: %s" % (g, sorted(dup))
    for name, fl in doc.get("profiles", {}).items():
        assert set(fl) <= FLAGS, (name, set(fl) - FLAGS)
        assert not (name in PROFILES and frozenset(fl) == PROFILES[name]), "프로필 %s 가 코드 PROFILES 를 복제함" % name
    errs = []
    prof, p = variant.load_variants_file(REPO_VARIANTS, errs.append)
    assert errs == [], errs
    assert set(PROFILES) <= set(prof)


def test_repo_tune_json_is_driver_keys_only():
    """tune.json·tune_default.json 은 운전자 13키만 (변형 상수는 variants.json)."""
    for name in ("tune.json", "tune_default.json"):
        d = json.load(open(os.path.join(ROOT, "tune", name)))
        assert set(d) == HEAD_DRIVER_KEYS, (name, set(d) ^ HEAD_DRIVER_KEYS)


# ---- 로더 동작 ----
def test_absent_algo_is_base(tdir):
    logs = []
    v = Variant(tdir, log=logs.append, now=0.0)
    assert (v.name, v.flags) == ("base", frozenset())
    assert v.p == Params()
    assert v.load(now=1.0) is False
    assert not any("모름" in s for s in logs)


def test_unknown_name_is_base_logged_once(tdir):
    logs = []
    v = Variant(tdir, log=logs.append, now=0.0)
    write(os.path.join(tdir, "algo"), "v9\n")
    assert v.load(now=1.0) is False          # base → base: 상태 변화 없음
    assert (v.name, v.flags) == ("base", frozenset())
    for t in range(2, 12):
        v.load(now=float(t))
    assert sum("모름" in s for s in logs) == 1, logs
    # 아는 이름으로 바꾸면 반영, 다시 모르는 이름이면 다시 1회
    write(os.path.join(tdir, "algo"), "v1\n")
    assert v.load(now=20.0) is True and v.name == "v1" and v.flags == GUARDS
    write(os.path.join(tdir, "algo"), "nope\n")
    assert v.load(now=21.0) is True and v.name == "base"
    v.load(now=22.0); v.load(now=23.0)
    assert sum("모름" in s for s in logs) == 2, logs


def test_delete_algo_both_nodes_return_to_base(tdir):
    """두 노드(두 인스턴스)가 같은 파일을 읽으면 선택·삭제 전이가 동일."""
    a = Variant(tdir, now=0.0); b = Variant(tdir, now=0.0)
    write(os.path.join(tdir, "algo"), "v2\n")
    a.load(now=1.0); b.load(now=1.0)
    assert (a.name, a.flags) == (b.name, b.flags) == ("v2", PROFILES["v2"])
    os.remove(os.path.join(tdir, "algo"))
    assert a.load(now=2.0) is True and b.load(now=2.0) is True
    assert (a.name, a.flags) == (b.name, b.flags) == ("base", frozenset())
    assert a.p == b.p == Params()
    # 다시 생기면 둘 다 같은 프레임(load 호출)에 반영
    write(os.path.join(tdir, "algo"), "v2a\n")
    a.load(now=3.0); b.load(now=3.0)
    assert a.flags == b.flags == PROFILES["v2a"] and "arc_comp" in a.flags


def test_empty_and_whitespace_algo_is_base(tdir):
    v = Variant(tdir, now=0.0)
    write(os.path.join(tdir, "algo"), "  v2  \n")
    v.load(now=1.0); assert v.name == "v2"
    write(os.path.join(tdir, "algo"), "\n")
    v.load(now=2.0); assert v.name == "base"


def test_variants_json_override_and_unknown_keys(tdir):
    logs = []
    doc = {"profiles": {"exp": ["one_side_clip", "arc_comp", "no_such_flag"], "base": ["arc_comp"]},
           "params": {"cross": {"search_back_m": 0.5, "confirm_frames": 3, "nope": 1},
                      "guard": {"accept_step_m": "x"},
                      "ghost": {"a": 1}},
           "extra": {}}
    write(os.path.join(tdir, "variants.json"), json.dumps(doc))
    v = Variant(tdir, log=logs.append, now=0.0)
    assert v.name == "base" and v.flags == frozenset()                      # base 는 공집합 유지(파일의 base 재정의 무시)
    assert v.profiles["exp"] == frozenset({"one_side_clip", "arc_comp"})
    assert v.p.cross.search_back_m == 0.5 and v.p.cross.confirm_frames == 3
    assert v.p.guard.accept_step_m == 0.3                                   # 형식 오류 → 기본값 유지
    assert v.p.cross.search_max_sec == 1.0                                  # 안 쓴 키는 기본값
    joined = "\n".join(logs)
    for frag in ("no_such_flag", "params.cross.nope", "ghost", "'extra'", "accept_step_m", "'base'"):
        assert frag in joined, (frag, joined)
    write(os.path.join(tdir, "algo"), "exp\n")
    assert v.load(now=1.0) is True and v.name == "exp" and v.flags == frozenset({"one_side_clip", "arc_comp"})
    # variants.json 삭제 → 코드 기본값, 'exp' 는 사라져 base
    os.remove(os.path.join(tdir, "variants.json"))
    assert v.load(now=2.0) is True and v.name == "base" and v.p == Params() and v.profiles == PROFILES


def test_max_age_ge_search_max(tdir):
    logs = []
    write(os.path.join(tdir, "variants.json"), json.dumps({"params": {"cross": {"search_max_sec": 2.5}}}))
    v = Variant(tdir, log=logs.append, now=0.0)
    assert v.p.cross.search_max_sec == 2.5 and v.p.cross.max_age_sec == 2.5
    assert any("max_age_sec" in s for s in logs)
    logs.clear()
    write(os.path.join(tdir, "variants.json"), json.dumps({"params": {"cross": {"search_max_sec": 0.8, "max_age_sec": 1.2}}}))
    assert v.load(now=1.0) is True and (v.p.cross.search_max_sec, v.p.cross.max_age_sec) == (0.8, 1.2)
    assert not any("max_age_sec" in s for s in logs)


def test_exclusive_flags_in_file_profile(tdir):
    logs = []
    write(os.path.join(tdir, "variants.json"), json.dumps({"profiles": {"both": ["arc_comp", "controller_pp"]}}))
    v = Variant(tdir, log=logs.append, now=0.0)
    assert v.profiles["both"] == frozenset({"controller_pp"})
    assert any("배타" in s for s in logs)


def test_broken_json_falls_back_to_defaults(tdir):
    logs = []
    write(os.path.join(tdir, "variants.json"), "{ not json")
    v = Variant(tdir, log=logs.append, now=0.0)
    assert v.p == Params() and v.profiles == PROFILES
    assert any("파싱 실패" in s for s in logs)


def test_non_utf8_bytes_do_not_crash(tdir):
    """tune/algo·variants.json 에 UTF-8 이 아닌 바이트: 노드가 매 프레임 load() 를 부르므로 예외 없이 base/기본값 + 로그 1회."""
    logs = []
    v = Variant(tdir, log=logs.append, now=0.0)
    with open(os.path.join(tdir, "algo.tmp"), "wb") as f: f.write(b"\xff\xfev2\n")
    os.replace(os.path.join(tdir, "algo.tmp"), os.path.join(tdir, "algo"))
    for t in range(1, 10):
        v.load(now=float(t))                                  # 예외가 나면 테스트 실패
    assert v.name == "base" and sum("모름" in s for s in logs) == 1, logs
    with open(os.path.join(tdir, "variants.json.tmp"), "wb") as f: f.write(b"\xff\xfe{\x00")
    os.replace(os.path.join(tdir, "variants.json.tmp"), os.path.join(tdir, "variants.json"))
    for t in range(10, 20):
        v.load(now=float(t))
    assert v.p == Params() and v.profiles == PROFILES
    assert sum("파싱 실패" in s for s in logs) == 1, logs


def test_requires_and_constraints(tdir):
    """speed_schedule 은 corner_slowdown 이 필요(제거 + 로그). 범위 위반 그룹은 코드 기본값 + 로그."""
    logs = []
    doc = {"profiles": {"sched_only": ["speed_schedule"], "okp": ["corner_slowdown", "speed_schedule"]},
           "params": {"corner": {"steer_on_pct": 40, "steer_off_pct": 60}, "arc": {"shift_max_m": -1}, "pp": {"lookahead_x_m": 0.3}}}
    write(os.path.join(tdir, "variants.json"), json.dumps(doc))
    v = Variant(tdir, log=logs.append, now=0.0)
    assert v.profiles["sched_only"] == frozenset() and v.profiles["okp"] == frozenset({"corner_slowdown", "speed_schedule"})
    assert v.p.corner == variant.Corner() and v.p.arc == variant.Arc()          # 위반 그룹은 기본값
    assert v.p.pp.lookahead_x_m == 0.3                                          # 정상 그룹은 적용
    joined = "\n".join(logs)
    assert "corner_slowdown 가 필요" in joined and "params.corner 범위 위반" in joined and "params.arc 범위 위반" in joined
    for a, b in variant.REQUIRES:
        for name, fl in PROFILES.items():
            assert not (a in fl and b not in fl), name


def test_fixed_variant_ignores_files(tdir):
    """from_name 으로 고정한 변형은 load() 가 tune/algo 를 읽지 않는다 (벤치·테스트 주입이 조용히 덮이지 않음)."""
    write(os.path.join(tdir, "algo"), "v2\n")
    v = Variant.from_name("v1", tdir)
    assert v.fixed and v.load(now=1.0) is False and v.name == "v1" and v.flags == GUARDS


def test_future_mtime_not_reread_every_cycle(tdir, monkeypatch):
    """파일 mtime 이 미래(시계 역행·복사)여도 매 주기 다시 읽지 않는다."""
    import time
    p = os.path.join(tdir, "algo"); write(p, "v1\n")
    future = time.time_ns() + 3_600 * 10**9
    os.utime(p, ns=(future, future))
    v = Variant(tdir, now=0.0)
    assert v.name == "v1"
    calls = []
    real = variant.read_text
    monkeypatch.setattr(variant, "read_text", lambda path: (calls.append(path), real(path))[1])
    for t in range(1, 20):
        v.load(now=float(t))
    assert calls == [], calls


def test_unchanged_files_no_relog(tdir):
    """파일이 안 바뀌면 load() 는 로그를 더 내지 않는다 (매 프레임 호출 안전)."""
    import time
    logs = []
    write(os.path.join(tdir, "variants.json"), json.dumps({"params": {"cross": {"search_back_m": 0.4}}}))
    write(os.path.join(tdir, "algo"), "v1\n")
    v = Variant(tdir, log=logs.append, now=0.0)
    time.sleep(variant.SETTLE_NS / 1e9 * 1.2)   # 쓰기 직후 재확인 창을 지나서
    v.load(now=1.0); n = len(logs)
    for t in range(2, 50):
        assert v.load(now=float(t)) is False
    assert len(logs) == n


def test_from_name():
    v = Variant.from_name("v2", os.path.join(ROOT, "tune"))
    assert v.flags == PROFILES["v2"] and v.p == Params()
    with pytest.raises(ValueError):
        Variant.from_name("bogus", os.path.join(ROOT, "tune"))
    assert "flags:" in v.describe()


# ---- CLI ----
def run_cli(*args):
    r = subprocess.run([sys.executable, os.path.join(NODES, "variant.py"), *args], capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr


def test_cli_list_check_show(tdir):
    rc, out, _ = run_cli("--list")
    assert rc == 0 and out.split() == DRIVER_PROFILES
    rc, out, _ = run_cli("--list-all")
    assert rc == 0 and set(out.split()) == set(PROFILES)
    for name in PROFILES:
        assert run_cli("--check", name)[0] == 0
    assert run_cli("--check", "bogus")[0] == 1
    assert run_cli("--check", "")[0] == 1
    rc, out, _ = run_cli("--show", "v2a")
    assert rc == 0
    assert "arc_comp" in out and "cross.search_max_sec = 1.0" in out and "corner.pwm = 190" in out
    assert run_cli("--show", "bogus")[0] == 1
    # variants.json 의 추가 프로필도 CLI 가 안다
    write(os.path.join(tdir, "variants.json"), json.dumps({"profiles": {"exp": ["one_side_clip"]}}))
    assert run_cli("--tune-dir", tdir, "--check", "exp")[0] == 0
    rc, out, _ = run_cli("--tune-dir", tdir, "--list-all")
    assert "exp" in out.split()
    rc, out, _ = run_cli("--tune-dir", tdir, "--list")
    assert out.split() == DRIVER_PROFILES              # --list 는 운전자 목록 고정


def test_tune_gui_uses_driver_profiles_only():
    """tune_gui.py 가 변형 이름을 하드코딩하지 않고 variant.DRIVER_PROFILES 를 쓴다 (정적)."""
    src = open(os.path.join(ROOT, "tune_gui.py"), encoding="utf-8").read()
    assert "variant.DRIVER_PROFILES" in src
    assert "ALGOS" not in src and '"v3"' not in src and "'v3'" not in src
    assert "def collect(self):\n        return {k: int(round(v.get())) if k == \"pwm\" else round(float(v.get()), 3) for k, (v, _, _) in self.vars.items()}" in src
