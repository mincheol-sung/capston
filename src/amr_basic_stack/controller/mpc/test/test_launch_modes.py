"""torque_experiment.launch.py 를 모드마다 만들어 보고 뜨는 노드를 확인한다 (R10 리뷰 2026-11-14).

실제로 띄우지 않는다 — launch 의 _nodes() 를 인자만 주고 불러, 오류 없이 노드 목록이 나오는지 본다.
plant:=real 은 이 PC 에서 한 번도 돌아본 적 없는 경로라, 인자 이름 오타·빠진 import 같은 것을 여기서 잡는다.
"""
import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch", "torque_experiment.launch.py")


@pytest.fixture
def launch_mod():
    spec = importlib.util.spec_from_file_location("torque_experiment_launch", LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def build(m, monkeypatch=None, **args):
    """기본 인자 + args 로 _nodes() 를 부른다. 반환 (실행 파일 목록, include 수, 전체 항목)."""
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    for k, v in args.items():
        ctx.launch_configurations[k] = str(v)
    out = m._nodes(ctx)
    exes = []
    for n in out:
        ex = getattr(n, "_Node__node_executable", None)
        if ex:
            exes.append(ex if isinstance(ex, str) else "".join(s.perform(ctx) for s in ex))
    return exes, sum(isinstance(n, IncludeLaunchDescription) for n in out), out


COMMON = ["torque_output_node", "slip_logger_node", "keyboard_command_gate"]


def test_sim_default(launch_mod):
    exes, inc, _ = build(launch_mod)
    assert exes == ["sim_plant_torque_node", "state_estimator_node", "figure8_reference_node", "torque_mpc_node",
                    "torque_output_node", "fault_monitor_node", "slip_logger_node", "keyboard_command_gate"]
    assert inc == 0


def test_sim_group_a(launch_mod):
    exes, _, _ = build(launch_mod, controller="pure_pursuit")
    assert "pure_pursuit_node" in exes and "wheel_pi_node" in exes and "torque_mpc_node" not in exes


def test_sim_truth_has_no_estimator_or_monitor(launch_mod):
    exes, _, _ = build(launch_mod, state_source="truth")
    assert "state_estimator_node" not in exes and "fault_monitor_node" not in exes


def test_sim_can(launch_mod):
    exes, _, _ = build(launch_mod, plant="sim_can")
    assert exes[:2] == ["sim_plant_torque_node", "ak40_can_node"]


def test_wheel_odom_switch(launch_mod):
    exes, _, _ = build(launch_mod, wheel_odom="true")
    assert exes[0] == "wheel_odometry_node"


def test_plant_only(launch_mod):
    exes, _, _ = build(launch_mod, use_mpc="false")
    assert exes == ["sim_plant_torque_node", "state_estimator_node"]


@pytest.mark.parametrize("controller", ["mpc", "pure_pursuit"])
def test_real_robot_path(launch_mod, monkeypatch, controller):
    """plant:=real — 모터·IMU 회전·robot_state(TF) 가 뜨고, 시뮬 플랜트·참값은 없다. CAN 자동 준비(sudo)는 끈다."""
    real_share = launch_mod.get_package_share_directory
    monkeypatch.setattr(launch_mod, "get_package_share_directory",
                        lambda p: "/nonexistent/amr_description" if p == "amr_description" else real_share(p))
    exes, inc, out = build(launch_mod, plant="real", auto_setup_can="false", controller=controller)
    assert "sim_plant_torque_node" not in exes
    assert exes[:2] == ["ak40_can_node", "imu_to_base_node"]
    assert inc == 1                                                     # robot_state.launch.py
    assert "fault_monitor_node" in exes and all(c in exes for c in COMMON)
    slip = [n for n in out if getattr(n, "_Node__node_executable", None) == "slip_logger_node"][0]
    assert "/sim/ground_truth" not in str(slip._Node__parameters)        # 실차에는 참값이 없다


def test_real_robot_without_robot_state(launch_mod):
    _, inc, _ = build(launch_mod, plant="real", auto_setup_can="false", robot_state="false")
    assert inc == 0


def test_real_robot_refuses_armed_at_start(launch_mod):
    with pytest.raises(RuntimeError, match="armed:=true"):
        build(launch_mod, plant="real", auto_setup_can="false", armed="true")


def test_real_robot_refuses_truth(launch_mod):
    with pytest.raises(RuntimeError):
        build(launch_mod, plant="real", auto_setup_can="false", state_source="truth")


def _torque_output_params(out):
    """torque_output_node 파라미터 {이름: 값} — 키는 TextSubstitution 튜플이라 글자로 푼다."""
    n = [n for n in out if getattr(n, "_Node__node_executable", None) == "torque_output_node"][0]
    return {"".join(getattr(s, "text", str(s)) for s in k): v for d in n._Node__parameters for k, v in d.items()}


@pytest.mark.parametrize("arg, expect", [("auto", False), ("true", True), ("false", False)])
def test_stiction_comp_sim(launch_mod, arg, expect):
    """정지 마찰 보상 (2026-11-16): auto 면 시뮬에서 끈다 (시뮬 플랜트에 정지 마찰이 없다). true/false 는 그대로."""
    _, _, out = build(launch_mod, stiction_comp=arg)
    assert _torque_output_params(out)["stiction_comp"] is expect


@pytest.mark.parametrize("arg, expect", [("auto", True), ("false", False)])
def test_stiction_comp_real(launch_mod, monkeypatch, arg, expect):
    """auto 면 실차에서 켠다. false 로 끌 수 있다 (A/B 비교)."""
    real_share = launch_mod.get_package_share_directory
    monkeypatch.setattr(launch_mod, "get_package_share_directory",
                        lambda p: "/nonexistent/amr_description" if p == "amr_description" else real_share(p))
    _, _, out = build(launch_mod, plant="real", auto_setup_can="false", stiction_comp=arg)
    assert _torque_output_params(out)["stiction_comp"] is expect


def _node_params(out, exe):
    """실행 파일 exe 노드의 파라미터 {이름: 값} (키는 TextSubstitution 튜플이라 글자로 푼다)."""
    n = [n for n in out if getattr(n, "_Node__node_executable", None) == exe][0]
    return {"".join(getattr(s, "text", str(s)) for s in k): v for d in n._Node__parameters for k, v in d.items()}


@pytest.mark.parametrize("args, plant_expect, ctrl_expect", [
    ({}, None, None),                                                       # 비우면 둘 다 YAML (model.use_wheel_inertia)
    ({"use_wheel_inertia": "true"}, True, True),                            # 둘 다
    ({"use_wheel_inertia": "true", "ctrl_wheel_inertia": "false"}, True, False),   # 지금 실차 흉내 (2026-11-16)
    ({"ctrl_wheel_inertia": "true"}, None, True),                           # 제어기만
])
def test_wheel_inertia_plant_and_controller(launch_mod, args, plant_expect, ctrl_expect):
    """바퀴 등가 질량 스위치 (2026-11-16): use_wheel_inertia 는 플랜트·제어기 둘 다, ctrl_wheel_inertia 는 제어기만."""
    _, _, out = build(launch_mod, **args)
    assert _node_params(out, "sim_plant_torque_node").get("use_wheel_inertia") is plant_expect
    assert _node_params(out, "torque_mpc_node").get("use_wheel_inertia") is ctrl_expect


def test_real_all_passes_model_switches():
    """amrall(real_all.launch.py) 이 정지 마찰·바퀴 관성 스위치를 torque_experiment 로 넘긴다."""
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "use_wheel_inertia" in m._PASS and "stiction_comp" in m._PASS


def test_track_error_stops_before_figure8_abort(launch_mod):
    """fault_monitor 정지 문턱(launch 기본 1.5 m, 2026-11-16)이 figure8 경성 중단 abort_error(torque_track.yaml 1.6 m)보다 작다."""
    import yaml
    _, _, out = build(launch_mod)
    mte = _node_params(out, "fault_monitor_node")["max_track_error"]
    with open(os.path.join(HERE, "..", "config", "torque_track.yaml")) as f:
        abort = float(yaml.safe_load(f)["track"]["abort_error"])
    assert mte == pytest.approx(1.5) and abort == pytest.approx(1.6)
    assert mte < abort
