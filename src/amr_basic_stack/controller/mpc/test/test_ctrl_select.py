# pytest -q controller/mpc/test/test_ctrl_select.py   (ROS 불필요) — 2026-11-28 controller:=all 제어기 선택
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from amr_mpc.core.ctrl_select import TopicSelect, family, parse_active, parse_map, route  # noqa: E402

TORQUE = ["mpc=/torque_mpc/cmd_raw", "pp=/wheel_pi/cmd_raw", "adpp=/wheel_pi/cmd_raw"]


def test_family_and_route():
    assert family("mpc") == "mpc" and family("pp") == "pp" and family(" ADPP ") == "pp"
    with pytest.raises(ValueError):
        family("ramp")
    assert route("mpc", True) == {"mpc": True, "pp": False}
    assert route("adpp", True) == {"mpc": False, "pp": True}
    for c in ("mpc", "pp", "adpp"):
        assert route(c, False) == {"mpc": False, "pp": False}      # 정지면 둘 다 false


def test_parse_active():
    assert parse_active(json.dumps({"active": "adpp", "event": "select"})) == "adpp"
    for bad in ("{}", "not json", json.dumps({"active": "step"})):
        with pytest.raises(ValueError):
            parse_active(bad)


def test_parse_map_needs_all_three():
    assert parse_map(TORQUE)["pp"] == "/wheel_pi/cmd_raw"
    with pytest.raises(ValueError):
        parse_map(TORQUE[:2])
    with pytest.raises(ValueError):
        parse_map(["mpc/torque_mpc/cmd_raw", *TORQUE[1:]])


def test_topic_select_passes_only_active():
    s = TopicSelect(TORQUE)
    assert s.topics() == ["/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw"]
    assert not s.passes("/torque_mpc/cmd_raw") and not s.passes("/wheel_pi/cmd_raw")    # 고르기 전에는 아무것도
    s.select("pp")
    assert s.passes("/wheel_pi/cmd_raw") and not s.passes("/torque_mpc/cmd_raw")       # MPC 의 DAMPING 을 막는다
    s.select("mpc")
    assert s.passes("/torque_mpc/cmd_raw") and not s.passes("/wheel_pi/cmd_raw")
