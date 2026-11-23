# pytest -q controller/mpc/test — tools/check_localization.py 의 첫 최적화 감지 (2026-11-23)
import importlib.util
import os

import pytest

cm = pytest.importorskip("cartographer_ros_msgs.msg")
spec = importlib.util.spec_from_file_location("check_localization", os.path.join(os.path.dirname(__file__), "..", "tools", "check_localization.py"))
cl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cl)


def msg(entries):
    m = cm.SubmapList()
    for traj, idx, x, y in entries:
        e = cm.SubmapEntry(); e.trajectory_id, e.submap_index = traj, idx
        e.pose.position.x, e.pose.position.y, e.pose.orientation.w = x, y, 1.0
        m.submap.append(e)
    return m


def test_detects_moved_submap_of_new_trajectory_only():
    """옛 trajectory (0·1) 이 옮겨져도, 새 trajectory 2 의 이미 본 서브맵이 옮겨질 때만 최적화로 본다. 새로 생긴 서브맵은 기록만."""
    prev = {}
    assert cl.optimization_moved(prev, msg([(0, 0, 0, 0), (1, 0, 5, 5)]), 2) == (0.0, 0.0)    # 새 trajectory 아직 없음
    assert cl.optimization_moved(prev, msg([(1, 0, 6, 6), (2, 0, 1.0, 2.0)]), 2) == (0.0, 0.0)  # 처음 봄 → 기록만
    assert cl.optimization_moved(prev, msg([(2, 0, 1.0, 2.0), (2, 1, 1.5, 2.0)]), 2) == (0.0, 0.0)  # 그대로 + 새 서브맵
    moved, turned = cl.optimization_moved(prev, msg([(2, 0, 1.06, 2.08), (2, 1, 1.5, 2.0)]), 2)  # 0 번이 (6, 8) cm 옮겨짐
    assert moved == pytest.approx(0.10) and turned == pytest.approx(0.0)


def test_tiny_correction_still_counts():
    """초기 포즈가 정확하면 첫 최적화 보정이 0.1 mm 미만 (11-23 151038) — 1 µm 도 최적화로 본다 (wait_optimization 문턱 1e-6)."""
    prev = {}
    cl.optimization_moved(prev, msg([(2, 0, 1.0, 2.0)]), 2)
    moved, _ = cl.optimization_moved(prev, msg([(2, 0, 1.0 + 3e-6, 2.0)]), 2)
    assert 1e-6 < moved < 1e-4
