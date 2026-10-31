#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""골든 생성기.

  단일:  make_golden.py --kind path|seq --module <lane_path_node.py> --algo base|v1|v2 --tune <tune.json> --out <json>
         make_golden.py --kind ctrl --ctrl <tunable_controller.py> --algo base|v2 --tune <tune.json> --out <json>
  전체:  make_golden.py --regen-all [--head a41706e] [--snapshot <동결 사본 폴더>]
         base_* : git show <head> 의 모듈 + HEAD tune.json, tune/algo 없음
         v1_path / v1_seq : 동결 사본 모듈, algo=v1, 사본 tune.json
         v2_ctrl : 동결 사본 제어기, algo=v2, 사본 tune.json (arc_comp 0 → corner_slowdown 만)

골든 JSON = {"meta": {kind, algo, module, tune(dict), numpy, cv2, python, git_rev}, "data": ...}
테스트는 meta.tune 을 그대로 임시 tune 폴더에 써서 돌린다 (저장소 tune.json 상태와 무관).
골든은 생성 환경(numpy·cv2)에 묶인다. 다른 환경에서는 identity_lib.load_golden 이 같은 기준 모듈로 즉석 생성해 비교한다.
"""
import argparse
import json
import os
import platform
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden_lib as G
import cv2
import numpy as np

SNAPSHOT_DEFAULT = os.path.expanduser("~/claude/자캡디/sim/logs/matrix/repo_baseline_snapshot")


def meta(kind, algo, module, tune, note=""):
    try:
        rev = subprocess.check_output(["git", "-C", G.REPO_DIR, "rev-parse", "--short", "HEAD"]).decode().strip()
    except Exception:
        rev = "?"
    return {"kind": kind, "algo": algo, "module": os.path.abspath(module), "tune": tune, "numpy": np.__version__,
            "cv2": cv2.__version__, "python": platform.python_version(), "git_rev": rev, "note": note}


def gen_path(module, algo, tune, workdir):
    import synth
    td = G.make_tune_dir(os.path.join(workdir, "tune_path"), tune, algo=algo)
    M = G.load_module(module, "golden_lane_%s_path" % algo, td)
    node = G.new_lane_node(M)
    frames = synth.frames(node.ipm)
    data = G.run_frames(M, node, frames)
    return data, len(frames)


def gen_seq(module, algo, tune, workdir):
    td = G.make_tune_dir(os.path.join(workdir, "tune_seq"), tune, algo=algo)
    M = G.load_module(module, "golden_lane_%s_seq" % algo, td)
    node = G.new_lane_node(M)
    steps = G.sequence_steps(node.ipm)
    return G.run_sequence(M, node, steps, td), len(steps)


def gen_ctrl(module, algo, tune, workdir):
    data = {}
    M = None
    for name, tj in G.ctrl_tune_sets(tune).items():
        td = G.make_tune_dir(os.path.join(workdir, "tune_ctrl_" + name), tj, algo=algo)
        if M is None:
            M = G.load_module(module, "golden_ctrl_%s" % algo, td)
        else:
            G.set_tune_dir(M, td)
        n = G.new_controller(M)
        data[name] = {"grid": G.run_ctrl_grid(M, n), "seq": G.run_ctrl_seq(M, G.new_controller(M))}
    return data, sum(len(v["grid"]) for v in data.values())


def write(out, kind, algo, module, tune, data, note=""):
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        json.dump({"meta": meta(kind, algo, module, tune, note), "data": data}, f, separators=(",", ":"))
    print("wrote", out, os.path.getsize(out) // 1024, "KB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=["path", "seq", "ctrl"])
    ap.add_argument("--module"); ap.add_argument("--ctrl"); ap.add_argument("--algo", default="base")
    ap.add_argument("--tune"); ap.add_argument("--out")
    ap.add_argument("--regen-all", action="store_true")
    ap.add_argument("--head", default="a41706e"); ap.add_argument("--snapshot", default=SNAPSHOT_DEFAULT)
    ap.add_argument("--golden-dir", default=G.GOLDEN_DIR)
    a = ap.parse_args()
    work = tempfile.mkdtemp(prefix="golden_")
    if a.regen_all:
        head = G.export_rev(a.head, os.path.join(work, "head"))
        head_tune = json.load(open(os.path.join(head, "tune/tune.json")))
        snap_tune = json.load(open(os.path.join(a.snapshot, "tune/tune.json")))
        gd = a.golden_dir
        d, n = gen_path(os.path.join(head, "nodes/lane_path_node.py"), None, head_tune, work)
        write(os.path.join(gd, "base_path.json"), "path", "base", os.path.join(head, "nodes/lane_path_node.py"), head_tune, d, "git %s, tune/algo 없음, %d frames" % (a.head, n))
        d, n = gen_seq(os.path.join(head, "nodes/lane_path_node.py"), None, head_tune, work)
        write(os.path.join(gd, "base_seq.json"), "seq", "base", os.path.join(head, "nodes/lane_path_node.py"), head_tune, d, "git %s, %d steps" % (a.head, n))
        d, n = gen_ctrl(os.path.join(head, "nodes/tunable_controller.py"), None, head_tune, work)
        write(os.path.join(gd, "base_ctrl.json"), "ctrl", "base", os.path.join(head, "nodes/tunable_controller.py"), head_tune, d, "git %s, %d grid" % (a.head, n))
        sm = os.path.join(a.snapshot, "nodes/lane_path_node.py"); sc = os.path.join(a.snapshot, "nodes/tunable_controller.py")
        d, n = gen_path(sm, "v1", snap_tune, work)
        write(os.path.join(gd, "v1_path.json"), "path", "v1", sm, snap_tune, d, "동결 사본, algo=v1, %d frames" % n)
        d, n = gen_seq(sm, "v1", snap_tune, work)
        write(os.path.join(gd, "v1_seq.json"), "seq", "v1", sm, snap_tune, d, "동결 사본, algo=v1, %d steps" % n)
        d, n = gen_ctrl(sc, "v2", snap_tune, work)
        write(os.path.join(gd, "v2_ctrl.json"), "ctrl", "v2", sc, snap_tune, d, "동결 사본, algo=v2 (arc_comp 0 → corner_slowdown 만), %d grid" % n)
        return
    if not (a.kind and a.out and a.tune):
        ap.error("--kind/--tune/--out 필요 (또는 --regen-all)")
    tune = json.load(open(a.tune))
    algo = None if a.algo == "base" else a.algo
    if a.kind == "path":
        d, n = gen_path(a.module, algo, tune, work); write(a.out, "path", a.algo, a.module, tune, d, "%d frames" % n)
    elif a.kind == "seq":
        d, n = gen_seq(a.module, algo, tune, work); write(a.out, "seq", a.algo, a.module, tune, d, "%d steps" % n)
    else:
        d, n = gen_ctrl(a.ctrl, algo, tune, work); write(a.out, "ctrl", a.algo, a.ctrl, tune, d, "%d grid" % n)


if __name__ == "__main__":
    main()
