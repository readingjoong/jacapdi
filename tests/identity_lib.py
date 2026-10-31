# -*- coding: utf-8 -*-
"""골든 == 비교 공용 (test_base_identity / test_variant_identity 가 공유). isclose 금지, 전부 == .

골든 파일은 생성 환경에 묶인다(meta.numpy·meta.cv2). 현재 numpy/cv2 가 meta 와 다르면 load_golden 이 같은 환경에서 기준을 즉석 생성한다:
base_* 는 git a41706e 모듈(meta.git_rev), v1/v2 는 meta.module 의 동결 사본. 둘 다 없으면 pytest.skip (사유에 환경 차이 명시).
컨테이너(시스템 python numpy 1.21.5) 에서 돌릴 때가 이 경우다."""
import json
import os
import sys
import tempfile

import cv2
import numpy as np

import golden_lib as G

_work = None
_live = {}
HEAD_REV = "a41706e"


def workdir():
    global _work
    if _work is None:
        _work = tempfile.mkdtemp(prefix="w0_identity_")
    return _work


def env_matches(meta):
    """골든 생성 환경과 같은가 (numpy 필수, cv2 는 meta 에 있을 때만 비교)."""
    return meta.get("numpy") == np.__version__ and meta.get("cv2", cv2.__version__) == cv2.__version__


def load_golden(name):
    g = json.load(open(os.path.join(G.GOLDEN_DIR, name)))
    if env_matches(g["meta"]):
        return g
    return live_reference(name, g)


def live_reference(name, g):
    """다른 환경: 같은 인터프리터로 기준 모듈을 돌려 data 를 다시 만든다 (meta.tune 그대로). 결과는 세션 안에서 캐시."""
    if name in _live:
        return _live[name]
    import pytest
    sys.path.insert(0, G.TESTS_DIR)
    import make_golden as MG
    m = g["meta"]; why = "골든 %s 는 numpy %s/cv2 %s 로 생성, 지금은 numpy %s/cv2 %s" % (name, m.get("numpy"), m.get("cv2", "?"), np.__version__, cv2.__version__)
    if m["algo"] == "base":
        try:
            root = G.export_rev(m.get("git_rev") or HEAD_REV, os.path.join(workdir(), "head_live"))
        except Exception as e:                                 # git 없음 (컨테이너에 저장소 .git 이 없을 때)
            pytest.skip("%s; git show %s 실패(%s) → 같은 환경 기준을 만들 수 없음" % (why, HEAD_REV, e))
        module = os.path.join(root, "nodes", "lane_path_node.py" if m["kind"] in ("path", "seq") else "tunable_controller.py")
        algo = None
    else:
        module = m["module"]; algo = m["algo"]
        if not os.path.isfile(module):
            pytest.skip("%s; 동결 사본 %s 없음" % (why, module))
    gen = {"path": MG.gen_path, "seq": MG.gen_seq, "ctrl": MG.gen_ctrl}[m["kind"]]
    data, _ = gen(module, algo, dict(m["tune"]), os.path.join(workdir(), "live_" + name.replace(".json", "")))
    _live[name] = {"meta": dict(m, live=why), "data": data}
    return _live[name]


def first_diff(a, b, path="", limit=5, acc=None):
    """두 JSON 값의 다른 곳을 path 문자열로 모은다 (최대 limit 개)."""
    acc = [] if acc is None else acc
    if len(acc) >= limit:
        return acc
    if type(a) != type(b) and not (isinstance(a, (int, float)) and isinstance(b, (int, float))):
        acc.append("%s: type %s != %s (%r vs %r)" % (path, type(a).__name__, type(b).__name__, a, b)); return acc
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                acc.append("%s.%s: missing (%s)" % (path, k, "golden" if k not in a else "current"))
                if len(acc) >= limit: return acc
                continue
            first_diff(a[k], b[k], "%s.%s" % (path, k), limit, acc)
    elif isinstance(a, list):
        if len(a) != len(b):
            acc.append("%s: len %d != %d" % (path, len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            first_diff(x, y, "%s[%d]" % (path, i), limit, acc)
            if len(acc) >= limit: return acc
    elif a != b:
        acc.append("%s: %r != %r" % (path, a, b))
    return acc


def assert_same(golden, current, what):
    if golden == current:
        return
    diffs = first_diff(golden, current)
    raise AssertionError("%s: 골든과 다름 (%d 곳 중 앞부분)\n  %s" % (what, len(diffs), "\n  ".join(diffs)))


def check_path(module_path, golden, algo, tag, variants=None, tune_override=None):
    """단일 프레임 find_path 골든 비교."""
    import synth
    tune = dict(golden["meta"]["tune"]); tune.update(tune_override or {})
    td = G.make_tune_dir(os.path.join(workdir(), "tune_path_" + tag), tune, algo=algo, variants=variants)
    M = G.load_module(module_path, "cur_lane_path_" + tag, td)
    node = G.new_lane_node(M)
    frames = synth.frames(node.ipm)
    assert len(frames) >= 1000, len(frames)
    cur = G.run_frames(M, node, frames)
    assert_same(golden["data"], cur, "find_path 프레임 (%s)" % tag)
    return len(frames)


def check_seq(module_path, golden, algo, tag, stats_keys, variants=None, tune_override=None):
    tune = dict(golden["meta"]["tune"]); tune.update(tune_override or {})
    td = G.make_tune_dir(os.path.join(workdir(), "tune_seq_" + tag), tune, algo=algo, variants=variants)
    M = G.load_module(module_path, "cur_lane_seq_" + tag, td)
    node = G.new_lane_node(M)
    steps = G.sequence_steps(node.ipm)
    assert steps[-1]["t"] - steps[0]["t"] >= 30.0 - G.DT - 1e-6
    cur = G.run_sequence(M, node, steps, td)
    assert_same(golden["data"]["frames"], cur["frames"], "30 s 시퀀스 (%s)" % tag)
    g_stats = {k: golden["data"]["stats"][k] for k in stats_keys}
    c_stats = {k: cur["stats"].get(k, "<없음>") for k in stats_keys}
    assert_same(g_stats, c_stats, "stats (%s)" % tag)
    return cur


def mask_cols(rows, cols):
    """행 목록에서 cols 열을 None 으로 (비교에서 빼는 열)."""
    if not cols:
        return rows
    return [[None if i in cols else v for i, v in enumerate(r)] for r in rows]


def check_ctrl(module_path, golden, algo, tag, variants=None, tune_override=None, skip_grid_cols=(), skip_seq_cols=()):
    """제어기 격자·시퀀스 골든 비교. skip_*_cols: 비교에서 빼는 열(사양으로 동작이 바뀐 열만; 호출 쪽이 이유를 적는다).
    반환: {tune 이름: (grid, seq)} 현재 출력 (추가 검사용)."""
    M = None; out = {}
    for name, tj in G.ctrl_tune_sets(golden["meta"]["tune"]).items():
        tj = dict(tj); tj.update(tune_override or {})
        td = G.make_tune_dir(os.path.join(workdir(), "tune_ctrl_%s_%s" % (tag, name)), tj, algo=algo, variants=variants)
        if M is None:
            M = G.load_module(module_path, "cur_ctrl_" + tag, td)
        else:
            G.set_tune_dir(M, td)
        grid = G.run_ctrl_grid(M, G.new_controller(M))
        assert_same(mask_cols(golden["data"][name]["grid"], skip_grid_cols), mask_cols(grid, skip_grid_cols),
                    "제어기 격자 tune %s (%s)" % (name, tag))
        seq = G.run_ctrl_seq(M, G.new_controller(M))
        assert_same(mask_cols(golden["data"][name]["seq"], skip_seq_cols), mask_cols(seq, skip_seq_cols),
                    "제어기 시퀀스 tune %s (%s)" % (name, tag))
        out[name] = (grid, seq)
    return out
