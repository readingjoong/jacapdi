# -*- coding: utf-8 -*-
"""pytest 공통: tests/ 를 import 경로에 넣고 ROS 스텁을 설치한다. 노드 폴더는 환경변수 JACAPDI_NODES_DIR 로 바꿀 수 있다
(기본 <저장소>/nodes; 동결 사본으로 '골든·테스트 자체가 맞는지' 확인할 때 사용)."""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)

import golden_lib as G  # noqa: E402

G.install_ros_stubs()

NODES_DIR = os.environ.get("JACAPDI_NODES_DIR", os.path.join(G.REPO_DIR, "nodes"))
LANE_MODULE = os.path.join(NODES_DIR, "lane_path_node.py")
CTRL_MODULE = os.path.join(NODES_DIR, "tunable_controller.py")

try:
    import pytest

    @pytest.fixture(scope="session")
    def work_dir(tmp_path_factory):
        return str(tmp_path_factory.mktemp("w0"))
except ImportError:   # unittest 로 돌릴 때
    pass
