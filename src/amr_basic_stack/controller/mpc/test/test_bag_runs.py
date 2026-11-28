# pytest -q controller/mpc/test/test_bag_runs.py   (ROS 불필요) — 2026-11-28 pitwall bag 판 나누기
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from bag_runs import split_runs  # noqa: E402

RC, ST, MC, AR = "/figure8/run_config", "/figure8/status", "/torque_mpc/config", "/mpc/armed"


def mpc(event, W0):
    return {"event": event, "live": {"W": [W0]}, "launch_only": {"limit_mode": "friction"}}


def test_three_runs_finished_discarded_and_bag_end():
    """판 1 완주 → 판 2 도중 멈춤에서 값 변경 (버림) → 판 3 은 bag 끝까지. MPC 값은 각 판 무장 때 값."""
    ev = [
        (0.0, MC, mpc("startup", 80)),
        (10.0, AR, True), (10.0, MC, mpc("arm", 80)), (10.01, RC, {"run": 1}), (10.02, ST, {"run": 1, "state": "running"}),
        (15.0, MC, mpc("pending", 80)),
        (30.0, ST, {"run": 1, "state": "finished"}),
        (32.0, AR, False), (32.0, MC, mpc("applied", 40)), (32.01, ST, {"run": 1, "state": "idle"}),
        (40.0, AR, True), (40.0, MC, mpc("arm", 40)), (40.01, RC, {"run": 2}),
        (45.0, AR, False), (47.0, ST, {"run": 2, "state": "idle"}),
        (50.0, AR, True), (50.0, MC, mpc("arm", 40)), (50.01, RC, {"run": 3}),
    ]
    rs = split_runs(sorted(ev, key=lambda e: e[0]), 0.0, 70.0)
    assert [r["run"] for r in rs] == [1, 2, 3]
    assert rs[0]["t1"] == 30.0 and "finished" in rs[0]["end"]
    assert rs[1]["t1"] == 47.0 and "버림" in rs[1]["end"]
    assert rs[2]["t1"] == 70.0 and rs[2]["end"] == "bag 끝"
    assert [r["mpc"]["W"][0] for r in rs] == [80, 40, 40]
    assert rs[0]["armed"] == [(10.01, 30.0)]
    assert rs[1]["armed"] == [(40.01, 45.0)]


def test_old_bag_without_run_config_is_one_run():
    ev = [(5.0, AR, True), (20.0, AR, False), (25.0, AR, True), (40.0, AR, False)]
    rs = split_runs(ev, 0.0, 50.0)
    assert len(rs) == 1 and rs[0]["t0"] == 5.0 and rs[0]["t1"] == 40.0 and rs[0]["ref"] is None


def test_controller_and_seq_leg_per_run():
    """controller:=all bag: 판마다 출발 직전 /ctrl/active 와 amrseq arm 사건을 붙인다 (2026-11-28)."""
    CA, SQ = "/ctrl/active", "/seq/status"
    ev = [
        (1.0, CA, {"active": "mpc", "event": "start"}),
        (9.0, SQ, {"event": "arm", "i": 1, "kind": "data", "ctrl": "mpc", "round": 1, "dir": "fwd", "data_i": 1}),
        (10.0, AR, True), (10.0, CA, {"active": "mpc", "event": "arm"}), (10.01, RC, {"run": 1}),
        (20.0, ST, {"run": 1, "state": "finished"}), (21.0, AR, False),
        (22.0, CA, {"active": "pp", "event": "select"}),
        (29.0, SQ, {"event": "arm", "i": 2, "kind": "data", "ctrl": "pp", "round": 1, "dir": "rev", "data_i": 2}),
        (30.0, AR, True), (30.01, RC, {"run": 2}),
    ]
    rs = split_runs(sorted(ev, key=lambda e: e[0]), 0.0, 40.0)
    assert [r["controller"] for r in rs] == ["mpc", "pp"]
    assert [r["seq"]["dir"] for r in rs] == ["fwd", "rev"] and rs[1]["seq"]["ctrl"] == "pp"


def test_bag_without_select_has_no_controller():
    ev = [(10.0, AR, True), (10.01, RC, {"run": 1}), (20.0, ST, {"run": 1, "state": "finished"})]
    r = split_runs(ev, 0.0, 30.0)[0]
    assert r["controller"] is None and r["seq"] is None
