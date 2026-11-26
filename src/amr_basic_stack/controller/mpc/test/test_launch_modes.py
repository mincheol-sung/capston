"""torque_experiment.launch.py 를 모드마다 만들어 보고 뜨는 노드를 확인한다 (R10 리뷰 2026-11-14).

실제로 띄우지 않는다 — launch 의 _nodes() 를 인자만 주고 불러, 오류 없이 노드 목록이 나오는지 본다.
plant:=real 은 이 PC 에서 한 번도 돌아본 적 없는 경로라, 인자 이름 오타·빠진 import 같은 것을 여기서 잡는다.
"""
import importlib.util
import math
import re
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


@pytest.mark.parametrize("controller", ["mpc", "pp", "adpp", "pure_pursuit"])
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


@pytest.mark.parametrize("args, plant_expect, use_w_expect", [
    ({}, None, None),                                   # 비우면 YAML (플랜트 model.use_wheel_inertia, 제어기 model_terms.use_w)
    ({"use_wheel_inertia": "true"}, True, None),        # 시뮬 플랜트 물리만
    ({"use_w": "true"}, None, True),                    # 제어기 w 항만
    ({"use_wheel_inertia": "true", "use_w": "false"}, True, False),
])
def test_wheel_inertia_plant_and_w_term(launch_mod, args, plant_expect, use_w_expect):
    """2026-11-16 w 항·외란 추정치 지시서: use_wheel_inertia 는 시뮬 플랜트만, 제어기는 M 에 m_w 를 늘 넣고 use_w 로 w 항만."""
    _, _, out = build(launch_mod, **args)
    assert _node_params(out, "sim_plant_torque_node").get("use_wheel_inertia") is plant_expect
    ctrl = _node_params(out, "torque_mpc_node")
    assert "use_wheel_inertia" not in ctrl
    assert ctrl.get("use_w") is use_w_expect


def test_real_all_passes_model_switches():
    """amrall(real_all.launch.py) 이 정지 마찰·w 항 스위치를 torque_experiment 로 넘긴다."""
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "use_w" in m._PASS and "stiction_comp" in m._PASS and "disturbance" in m._PASS
    assert "use_wheel_inertia" not in m._PASS                # 실차에는 플랜트가 없다
    assert "controller" in m._PASS                           # 비교군 A (2026-11-16 전에는 빠져 있어 늘 MPC 가 떴다)


def test_real_all_controller_reaches_torque_experiment():
    """amrall controller:=pure_pursuit 가 include 한 torque_experiment 의 launch_arguments 로 실제로 넘어간다."""
    from launch.actions import GroupAction
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    ctx.launch_configurations.update({"sensors": "false", "localization": "false", "viz": "false", "auto_localize": "false",
                                      "use_mpc": "true", "controller": "pure_pursuit", "yaw_mode": "tangent"})
    incs = [g for g in m._nodes(ctx) if isinstance(g, GroupAction)]
    assert len(incs) == 1                                    # torque_experiment 하나
    inc = [e for e in incs[0].get_sub_entities() if isinstance(e, IncludeLaunchDescription)][0]
    txt = lambda x: x if isinstance(x, str) else "".join(s.perform(ctx) for s in x)
    args = {txt(k): txt(v) for k, v in inc.launch_arguments}
    assert args["controller"] == "pure_pursuit" and args["plant"] == "real" and args["yaw_mode"] == "tangent"


@pytest.mark.parametrize("given, expect", [({}, {}),
                                            ({"imu_accel_fps": "400", "imu_gyro_fps": "400", "imu_unite": "1"},
                                             {"depth_camera_accel_fps": "400", "depth_camera_gyro_fps": "400",
                                              "depth_camera_unite_imu_method": "1"})])
def test_real_all_passes_imu_settings_to_sensors(given, expect):
    """2026-11-19 가속도계 지연 시험: amrall imu_accel_fps·imu_gyro_fps·imu_unite 가 sensors.launch.py 의 depth_camera_* 로 간다 (비우면 안 넘김)."""
    from launch.actions import GroupAction
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    ctx.launch_configurations.update({"sensors": "true", "localization": "false", "viz": "false", "auto_localize": "false",
                                      "use_mpc": "false", **given})
    txt = lambda x: x if isinstance(x, str) else "".join(s.perform(ctx) for s in x)
    incs = []
    for g in m._nodes(ctx):
        if isinstance(g, GroupAction):
            inc = [e for e in g.get_sub_entities() if isinstance(e, IncludeLaunchDescription)][0]
            incs.append({txt(k): txt(v) for k, v in inc.launch_arguments})
    assert len(incs) == 2                                    # sensors + torque_experiment (나머지는 끔)
    sens = [a for a in incs if "plant" not in a]             # torque_experiment 에는 plant:=real 이 늘 있다
    assert sens == [expect]


@pytest.mark.parametrize("given, expect", [({}, None), ({"est_pose_period": "0.05"}, 0.05)])
def test_estimator_pose_period(launch_mod, given, expect):
    """2026-11-19: est_pose_period 가 state_estimator_node pose_period 로 (비우면 노드 기본 0.1)."""
    _, _, out = build(launch_mod, **given)
    assert _node_params(out, "state_estimator_node").get("pose_period") == expect


@pytest.mark.parametrize("lua, given, expect", [("localization_torque_2d.lua", {}, None),
                                                ("localization_torque_acc1_2d.lua", {}, "0.05"),
                                                ("localization_torque_acc1_2d.lua", {"est_pose_period": "0.08"}, "0.08")])
def test_real_all_acc1_lua_sets_pose_period(lua, given, expect):
    """스캔 하나씩 매칭하는 lua 면 real_all 이 est_pose_period 0.05 를 자동으로 넘긴다 (직접 주면 그 값)."""
    from launch.actions import GroupAction
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    ctx.launch_configurations.update({"sensors": "false", "localization": "false", "viz": "false", "auto_localize": "false",
                                      "use_mpc": "false", "lua": lua, **given})
    txt = lambda x: x if isinstance(x, str) else "".join(s.perform(ctx) for s in x)
    incs = [g for g in m._nodes(ctx) if isinstance(g, GroupAction)]
    inc = [e for e in incs[0].get_sub_entities() if isinstance(e, IncludeLaunchDescription)][0]
    assert {txt(k): txt(v) for k, v in inc.launch_arguments}.get("est_pose_period") == expect


@pytest.mark.parametrize("given, expect", [({}, {}), ({"pp_speed_source": "estimator", "pp_kp": "0.015"},
                                                        {"speed_source": "estimator", "kp": 0.015})])
def test_pure_pursuit_speed_source(launch_mod, given, expect):
    """2026-11-19: 비교군 A 바퀴 PI 의 속도 되먹임 출처·게인 덮어쓰기. speed_topic 은 늘 MPC 와 같은 상태 토픽."""
    _, _, out = build(launch_mod, controller="pure_pursuit", **given)
    import yaml                                     # 글자 파라미터는 YAML 로 적힌 Substitution 튜플 ("값\n...\n")
    txt = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) if isinstance(v, (tuple, list)) else v
    pi = {k: txt(v) for k, v in _node_params(out, "wheel_pi_node").items()}
    assert pi["speed_topic"] == "/state_estimator/odom"
    for k, v in expect.items():
        assert pi[k] == v
    if not given:
        assert "speed_source" not in pi and "kp" not in pi


@pytest.mark.parametrize("given, expect", [({}, {}), ({"mpc_dist_frame": "world", "mpc_dist_freeze": "false", "mpc_dist_acc_max": "0.3",
                                                         "mpc_dist_yaw_acc_max": "-1"},
                                                        {"disturbance_frame": "world", "disturbance_freeze_on_saturation": False,
                                                         "disturbance_acc_max": 0.3, "disturbance_yaw_acc_max": -1.0})])
def test_disturbance_frame_args(launch_mod, given, expect):
    """2026-11-21: 외란 추정치 좌표·포화 중 멈춤·상한 덮어쓰기 (비우면 torque_mpc.yaml disturbance 블록을 노드가 읽는다)."""
    import yaml                                     # 글자 파라미터는 YAML 로 적힌 Substitution 튜플 (test_pure_pursuit_speed_source 와 같음)
    _, _, out = build(launch_mod, **given)
    txt = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) if isinstance(v, (tuple, list)) else v
    p = {k: txt(v) for k, v in _node_params(out, "torque_mpc_node").items()}
    for k, v in expect.items():
        assert p[k] == (pytest.approx(v) if isinstance(v, float) else v)
    if not given:
        assert not any(k in p for k in ("disturbance_frame", "disturbance_freeze_on_saturation", "disturbance_acc_max"))


def test_disturbance_frame_rejects_unknown(launch_mod):
    with pytest.raises(Exception):
        build(launch_mod, mpc_dist_frame="map")


@pytest.mark.parametrize("given, expect", [({}, {}), ({"mpc_dist_observer": "kalman", "mpc_target_projection": "true", "mpc_ff_normal_scale": "0.9",
                                                         "mpc_ff_normal_estimate": "true", "mpc_ff_normal_q": "0.05"},
                                                        {"disturbance_observer": "kalman", "target_projection": True, "ff_normal_scale": 0.9,
                                                         "ff_normal_estimate": True, "ff_normal_q": 0.05})])
def test_offset_free_args(launch_mod, given, expect):
    """2026-11-21: 관측기·목표 입력 사영·구심 feedforward 계수 ρ 덮어쓰기 (비우면 torque_mpc.yaml 을 노드가 읽는다)."""
    import yaml
    _, _, out = build(launch_mod, **given)
    txt = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) if isinstance(v, (tuple, list)) else v
    p = {k: txt(v) for k, v in _node_params(out, "torque_mpc_node").items()}
    for k, v in expect.items():
        assert p[k] == (pytest.approx(v) if isinstance(v, float) else v)
    if not given:
        assert not any(k in p for k in ("disturbance_observer", "target_projection", "ff_normal_scale", "ff_normal_estimate", "ff_normal_q"))


def test_offset_free_rejects_unknown_observer(launch_mod):
    with pytest.raises(Exception):
        build(launch_mod, mpc_dist_observer="luenberger")


@pytest.mark.parametrize("given, expect", [({}, {}), ({"mpc_Nc": "25"}, {"Nc": 25}), ({"mpc_Np": "100", "mpc_Nc": "50"}, {"Np": 100, "Nc": 50})])
def test_mpc_horizon_args(launch_mod, given, expect):
    """2026-11-23: 예측·제어 지평 Np·Nc 덮어쓰기 (비우면 torque_mpc.yaml tuning 을 노드가 읽는다)."""
    import yaml
    _, _, out = build(launch_mod, **given)
    txt = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) if isinstance(v, (tuple, list)) else v
    p = {k: txt(v) for k, v in _node_params(out, "torque_mpc_node").items()}
    for k in ("Np", "Nc"):
        if k in expect:
            assert p[k] == expect[k]
        else:
            assert k not in p


@pytest.mark.parametrize("bad", [{"mpc_Nc": "400"}, {"mpc_Np": "400"}, {"mpc_Np": "50", "mpc_Nc": "60"}])
def test_mpc_horizon_rejects_out_of_range(launch_mod, bad):
    with pytest.raises(Exception):
        build(launch_mod, **bad)


def test_offset_free_args_pass_through_real_all():
    """real_all 이 새 인자를 torque_experiment 로 넘긴다 (_PASS)."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "launch", "real_all.launch.py")).read()
    for a in ("mpc_dist_observer", "mpc_target_projection", "mpc_ff_normal_scale", "mpc_ff_normal_estimate", "mpc_ff_normal_q", "mpc_Nc", "mpc_Np"):
        assert f'"{a}"' in src


@pytest.mark.parametrize("given, expect", [({}, {}), ({"pp_lookahead_min": "0.15", "pp_lookahead_max": "0.60"},
                                                        {"lookahead_min": 0.15, "lookahead_max": 0.60})])
def test_pure_pursuit_lookahead_override(launch_mod, given, expect):
    """2026-11-21: 룩어헤드 하한·상한 덮어쓰기 (비우면 torque_baseline_a.yaml 을 노드가 읽는다)."""
    _, _, out = build(launch_mod, controller="pure_pursuit", **given)
    pp = _node_params(out, "pure_pursuit_node")
    for k, v in expect.items():
        assert pp[k] == pytest.approx(v)
    if not given:
        assert "lookahead_min" not in pp and "lookahead_max" not in pp


def test_real_all_passes_pp_lookahead():
    """real_all 이 pp_lookahead_min·max 를 torque_experiment 로 넘긴다."""
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    assert {"pp_lookahead_min", "pp_lookahead_max"} <= set(m._PASS)


# ── 2026-11-23 비교군 세 개 (교수님 회의): 룩어헤드 고정 PP · 어댑티브 PP · MPC ──
LOOKAHEAD = ("lookahead_time", "lookahead_min", "lookahead_max")


def test_adpp_nodes(launch_mod):
    exes, _, _ = build(launch_mod, controller="adpp")
    assert "pure_pursuit_node" in exes and "wheel_pi_node" in exes and "torque_mpc_node" not in exes


@pytest.mark.parametrize("given, expect", [
    ({}, {"lookahead_time": 0.5, "lookahead_min": 0.15, "lookahead_max": 0.60}),
    ({"pp_lookahead_max": "0.45"}, {"lookahead_time": 0.5, "lookahead_min": 0.15, "lookahead_max": 0.45}),   # 인자가 우선
])
def test_adpp_lookahead(launch_mod, given, expect):
    """어댑티브는 torque_baseline_a.yaml pure_pursuit_adaptive 세 값을 노드에 넘긴다 (11-20 전 속도 비례 룩어헤드)."""
    _, _, out = build(launch_mod, controller="adpp", **given)
    pp = _node_params(out, "pure_pursuit_node")
    for k, v in expect.items():
        assert pp[k] == pytest.approx(v)


def test_pp_fixed_vs_adaptive_yaml():
    """고정 PP 는 min = max (상수), 어댑티브는 min < max — 두 비교군이 실제로 다르다."""
    import yaml
    ya = yaml.safe_load(open(os.path.join(HERE, "..", "config", "torque_baseline_a.yaml")))
    fx, ad = ya["pure_pursuit"], ya["pure_pursuit_adaptive"]
    assert fx["lookahead_min"] == fx["lookahead_max"]
    assert ad["lookahead_min"] < ad["lookahead_max"] and ad["lookahead_time"] > 0


def test_pp_modes_differ_only_in_lookahead(launch_mod):
    """두 PP 모드는 룩어헤드 세 값 말고 같다: 퓨어 퍼슛 나머지 파라미터, 바퀴 PI, torque_output 입력, fault_monitor status."""
    _, _, fx = build(launch_mod, controller="pp")
    _, _, ad = build(launch_mod, controller="adpp")
    txt = lambda v: "".join(getattr(s, "text", str(s)) for s in v) if isinstance(v, (tuple, list)) else v   # 글자 = Substitution 튜플
    plain = lambda p, drop=(): {k: txt(v) for k, v in p.items() if k not in drop}
    assert plain(_node_params(fx, "pure_pursuit_node"), LOOKAHEAD) == plain(_node_params(ad, "pure_pursuit_node"), LOOKAHEAD)
    for exe in ("wheel_pi_node", "fault_monitor_node"):
        assert plain(_node_params(fx, exe)) == plain(_node_params(ad, exe))
    assert plain(_torque_output_params(fx)) == plain(_torque_output_params(ad))
    import yaml                                     # 글자 파라미터는 YAML 로 적힌 Substitution 튜플
    assert yaml.safe_load(plain(_node_params(ad, "fault_monitor_node"))["track_status_topic"]) == "/pure_pursuit/status"


def test_pure_pursuit_is_old_name_of_pp(launch_mod):
    """controller:=pure_pursuit (예전 이름, 옛 스크립트·bag 인자) 은 pp 와 같은 노드·파라미터."""
    ex_old, _, old = build(launch_mod, controller="pure_pursuit")
    ex_new, _, new = build(launch_mod, controller="pp")
    assert ex_old == ex_new
    txt = lambda v: "".join(getattr(s, "text", str(s)) for s in v) if isinstance(v, (tuple, list)) else v
    for exe in ("pure_pursuit_node", "wheel_pi_node", "fault_monitor_node"):
        assert {k: txt(v) for k, v in _node_params(old, exe).items()} == {k: txt(v) for k, v in _node_params(new, exe).items()}


def test_track_error_stops_before_figure8_abort(launch_mod):
    """fault_monitor 정지 문턱(launch 기본 1.5 m, 2026-11-16)이 figure8 경성 중단 abort_error(torque_track.yaml 1.6 m)보다 작다."""
    import yaml
    _, _, out = build(launch_mod)
    mte = _node_params(out, "fault_monitor_node")["max_track_error"]
    with open(os.path.join(HERE, "..", "config", "torque_track.yaml")) as f:
        abort = float(yaml.safe_load(f)["track"]["abort_error"])
    assert mte == pytest.approx(1.5) and abort == pytest.approx(1.6)
    assert mte < abort


@pytest.mark.parametrize("args", [{"yaw_mode": "true"}, {"shape": "circle"}, {"origin_mode": "map"}])
def test_rejects_invalid_track_strings(launch_mod, args):
    """2026-11-16: yaw_mode:=true 로 띄우면 figure8_reference_node 가 조용히 죽어 기준 없이 서 있었다 → launch 에서 바로 거부."""
    with pytest.raises(RuntimeError, match="허용값"):
        build(launch_mod, **args)


@pytest.mark.parametrize("args", [{"yaw_mode": "tangent"}, {"yaw_mode": "fixed", "shape": "circles8"}, {"shape": "line"}])
def test_accepts_valid_track_strings(launch_mod, args):
    exes, _, _ = build(launch_mod, **args)
    assert "figure8_reference_node" in exes


def test_invalid_track_rejected_before_can_setup_on_real(launch_mod, monkeypatch):
    """실차에서도 CAN 준비(sudo)·노드보다 먼저 거부한다."""
    called = []
    monkeypatch.setattr(launch_mod, "prepare_can", lambda *a, **k: called.append(1))   # 진짜 sudo 를 부르지 않게
    with pytest.raises(RuntimeError, match="허용값"):
        build(launch_mod, plant="real", auto_setup_can="true", yaw_mode="true")
    assert not called


@pytest.mark.parametrize("given, expect", [
    ("", None),                                          # 비우면 넘기지 않는다 → 노드가 torque_plant.yaml body.wheel_load_share
    ("0.2388,0.2392,0.2608,0.2612", [0.2388, 0.2392, 0.2608, 0.2612]),
    ("[0.25, 0.25, 0.25, 0.25]", [0.25, 0.25, 0.25, 0.25]),
])
def test_wheel_load_share_reaches_mpc_and_sim_plant(launch_mod, given, expect):
    """2026-11-19 바퀴별 수직력 몫 → torque_mpc_node (friction 상한) 와 sim_plant_torque_node (슬립 모델 N_i) 둘 다."""
    _, _, out = build(launch_mod, limit_mode="friction", wheel_load_share=given)
    for exe in ("torque_mpc_node", "sim_plant_torque_node"):
        got = _node_params(out, exe).get("wheel_load_share")   # launch_ros 는 리스트를 튜플로 바꾼다
        assert (list(got) if got is not None else None) == expect, exe


@pytest.mark.parametrize("given", ["0.25,0.25,0.5", "0.05,0.3,0.3,0.35", "a,b,c,d", "0.3,0.3,0.3,0.3"])
def test_wheel_load_share_rejected_before_can_setup(launch_mod, monkeypatch, given):
    """개수·범위(0.1~0.4)·합(1 ± 0.05)이 틀리면 실차에서도 CAN 준비(sudo)·노드보다 먼저 멈춘다."""
    called = []
    monkeypatch.setattr(launch_mod, "prepare_can", lambda *a, **k: called.append(1))
    with pytest.raises(RuntimeError, match="wheel_load_share"):
        build(launch_mod, plant="real", auto_setup_can="true", wheel_load_share=given)
    assert not called


def test_wheel_load_share_warns_without_friction_mode(launch_mod):
    """YAML 기본 상한 모드는 motor_only — MPC 상한에는 안 쓰인다는 것을 launch 로그로 알린다."""
    from launch.actions import LogInfo
    def msgs(**a):
        _, _, out = build(launch_mod, **a)
        return ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    assert any("wheel_load_share" in t and "friction 이 아니면" in t for t in msgs(wheel_load_share="0.24,0.24,0.26,0.26"))
    assert not any("friction 이 아니면" in t for t in msgs(limit_mode="friction", wheel_load_share="0.24,0.24,0.26,0.26"))


def test_real_all_passes_wheel_load_share():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "wheel_load_share" in m._PASS


@pytest.mark.parametrize("given, expect", [({}, None), ({"est_mode": "slam"}, "slam"), ({"est_mode": "kf"}, "kf")])
def test_estimator_mode_reaches_node(launch_mod, given, expect):
    """2026-11-19: est_mode 가 state_estimator_node est_mode 로 (비우면 torque_estimator.yaml mode, 기본 kf)."""
    _, _, out = build(launch_mod, **given)
    v = _node_params(out, "state_estimator_node").get("est_mode")
    if isinstance(v, (tuple, list)):                   # 문자열 파라미터는 YAML 문서('slam\n...\n')를 담은 TextSubstitution 으로 온다
        import yaml
        v = yaml.safe_load("".join(s.text for s in v))
    assert v == expect


@pytest.mark.parametrize("given, expect", [({}, None), ({"trip_on_pose_jump": "false"}, False), ({"trip_on_pose_jump": "true"}, True)])
def test_trip_on_pose_jump_reaches_fault_monitor(launch_mod, given, expect):
    """2026-11-19: trip_on_pose_jump 가 fault_monitor_node 로 (비우면 노드 기본 true)."""
    _, _, out = build(launch_mod, **given)
    assert _node_params(out, "fault_monitor_node").get("trip_on_pose_jump") == expect


def test_real_all_passes_estimator_mode():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "est_mode" in m._PASS and "trip_on_pose_jump" in m._PASS
    assert "lap_speeds" in m._PASS                           # 랩별 속도 사다리 (2026-11-21)



@pytest.mark.parametrize("given, expect", [
    ("", None),                                          # 비우면 넘기지 않는다 → 노드가 torque_plant.yaml surface.mu_hat_wheels
    ("0.432,0.285,0.456,0.350", [0.432, 0.285, 0.456, 0.350]),
    ("off", [-1.0, -1.0, -1.0, -1.0]),                   # 끄기 → 네 바퀴 모두 mu_hat × 하중 몫
    ("0.4,-1,0.45,0.5", [0.4, -1.0, 0.45, 0.5]),         # 0 이하인 바퀴만 mu_hat
])
def test_mu_hat_wheels_reaches_mpc(launch_mod, given, expect):
    """2026-11-19 (11) 바퀴별 μ̂ (2026-11-22 부터 N_i 기준) → torque_mpc_node. 시뮬 플랜트에는 안 넘어간다 (물리는 surface_mu·하중 몫)."""
    _, _, out = build(launch_mod, limit_mode="friction", mu_hat_wheels=given)
    got = _node_params(out, "torque_mpc_node").get("mu_hat_wheels")
    assert (list(got) if got is not None else None) == expect
    assert "mu_hat_wheels" not in _node_params(out, "sim_plant_torque_node")


@pytest.mark.parametrize("given", ["0.4,0.3,0.45", "0.4,0.3,0.45,2.5", "a,b,c,d"])
def test_mu_hat_wheels_rejected_before_can_setup(launch_mod, monkeypatch, given):
    called = []
    monkeypatch.setattr(launch_mod, "prepare_can", lambda *a, **k: called.append(1))
    with pytest.raises(RuntimeError, match="mu_hat_wheels"):
        build(launch_mod, plant="real", auto_setup_can="true", mu_hat_wheels=given)
    assert not called


def test_mu_hat_ignored_warning_when_wheels_set(launch_mod):
    """바퀴별 μ̂ 가 있으면 mu_hat:= 는 MPC 상한에 안 쓰인다 — launch 로그로 알리고, 없으면 (off) 안 알린다.
    2026-11-25 사용자 결정: YAML 기본이 바퀴별 값 끔 (−1 × 4) · mu_hat 0.33 · cap_on ground — 바퀴별 값은 인자로 줄 때만 있다."""
    from launch.actions import LogInfo
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_plant.yaml")) as f:
        surf = yaml.safe_load(f)["surface"]
    assert surf["mu_hat"] == 0.33 and surf["mu_hat_wheels"] == [-1.0] * 4 and surf["c_s"] == 0.8
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        assert yaml.safe_load(f)["torque_limit"]["cap_on"] == "ground"
    def msgs(**a):
        _, _, out = build(launch_mod, **a)
        return ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    old_wheels = "0.598,0.489,0.547,0.703"
    assert any("안 쓰인다" in t and "mu_hat:=0.6" in t for t in msgs(limit_mode="friction", mu_hat="0.6", mu_hat_wheels=old_wheels))
    assert not any("mu_hat:=0.6" in t for t in msgs(limit_mode="friction", mu_hat="0.6"))                       # YAML 기본 (끔)
    assert not any("mu_hat:=0.6" in t for t in msgs(limit_mode="friction", mu_hat="0.6", mu_hat_wheels="off"))


def test_real_all_passes_mu_hat_wheels():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "mu_hat_wheels" in m._PASS and "wheel_load_share" in m._PASS



@pytest.mark.parametrize("given, expect", [("", None), ("0.9", 0.9), ("1.0", 1.0)])
def test_load_budget_reaches_figure8(launch_mod, given, expect):
    """2026-11-19 (12) 부하 예산 → figure8_reference_node. 비우면 넘기지 않는다 (노드 기본 = plant.yaml, 2026-11-20 부터 1.0)."""
    _, _, out = build(launch_mod, shape="circles8", width="2.0", yaw_mode="tangent", load_budget=given)
    assert _node_params(out, "figure8_reference_node").get("load_budget") == expect


@pytest.mark.parametrize("given", ["0", "1.2", "-0.5", "abc"])
def test_load_budget_rejected_before_can_setup(launch_mod, monkeypatch, given):
    called = []
    monkeypatch.setattr(launch_mod, "prepare_can", lambda *a, **k: called.append(1))
    with pytest.raises(RuntimeError, match="load_budget"):
        build(launch_mod, plant="real", auto_setup_can="true", load_budget=given)
    assert not called


def test_real_all_passes_load_budget():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "load_budget" in m._PASS


def test_square_params_reach_figure8(launch_mod):
    """2026-11-20 정사각형: shape·square_* 가 figure8_reference_node 로 간다. 비우면 torque_track.yaml 값."""
    _, _, out = build(launch_mod, shape="square", yaw_mode="tangent", square_side="2.0", square_corner_speed="0.05")
    import yaml                                     # 위 시험과 같은 방식 (글자 파라미터는 YAML 로 적힌 튜플)
    p = _node_params(out, "figure8_reference_node")
    text = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v))   # 문자열 값은 YAML 로 적힌 TextSubstitution 튜플
    assert text(p["shape"]) == "square" and p["square_side"] == 2.0 and p["square_corner_speed"] == 0.05
    assert p["square_v_max"] == 0.0 and p["square_yaw_blend"] == pytest.approx(0.3)


def test_square_rejects_spin_before_nodes(launch_mod):
    with pytest.raises(RuntimeError, match="square"):
        build(launch_mod, shape="square", yaw_mode="spin")


def test_real_all_passes_square_keys():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert {"square_side", "square_corner_speed", "square_v_max", "square_yaw_blend"} <= set(m._PASS)


def test_square_corner_stop_reaches_figure8(launch_mod):
    """2026-11-21 꼭짓점 정지: square_corner_mode·rot_s·dwell_s 가 노드로. 비우면 yaml (pass, 1.0, 0.1)."""
    import yaml
    text = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v))
    _, _, out = build(launch_mod, shape="square", square_corner_mode="stop", square_rot_s="0.8")
    p = _node_params(out, "figure8_reference_node")
    assert text(p["square_corner_mode"]) == "stop" and p["square_rot_s"] == 0.8 and p["square_dwell_s"] == pytest.approx(0.1)
    _, _, out = build(launch_mod, shape="square")
    assert text(_node_params(out, "figure8_reference_node")["square_corner_mode"]) == "pass"


def test_square_corner_mode_checked_before_nodes(launch_mod):
    with pytest.raises(RuntimeError, match="square_corner_mode"):
        build(launch_mod, shape="square", square_corner_mode="halt")


def test_real_all_passes_square_corner_keys():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert {"square_corner_mode", "square_rot_s", "square_dwell_s"} <= set(m._PASS)


def test_zigzag_params_reach_figure8(launch_mod):
    """2026-11-24 지그재그: zigzag_* 가 노드로 (zigzag_n 은 정수). 비우면 yaml. 전용 zigzag_accel·zigzag_v_max 는 없다 ((8))."""
    import yaml
    text = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v))
    _, _, out = build(launch_mod, shape="zigzag", zigzag_n="3", zigzag_angle_deg="-20", zigzag_profile="square")
    p = _node_params(out, "figure8_reference_node")
    assert text(p["shape"]) == "zigzag" and text(p["zigzag_profile"]) == "square"
    assert p["zigzag_n"] == 3 and isinstance(p["zigzag_n"], int) and p["zigzag_angle_deg"] == -20.0
    assert p["zigzag_length"] == 1.0 and "zigzag_accel" not in p and "zigzag_v_max" not in p
    assert 0.5 < p["accel"] < 3.0                     # 비우면 max 가 숫자로 (지금 설정 1.70)
    _, _, out = build(launch_mod, shape="zigzag", accel="0.8")
    p = _node_params(out, "figure8_reference_node")
    assert p["accel"] == 0.8 and text(p["zigzag_profile"]) == "sine" and p["zigzag_n"] == 5


def test_zigzag_accel_max_matches_square_edge(launch_mod):
    """지그재그 직선과 정사각형 변은 같은 직선 가속 한계 (fig8_limits.line_accel_max)."""
    _, _, out = build(launch_mod, shape="zigzag")
    a_z = _node_params(out, "figure8_reference_node")["accel"]
    _, _, out = build(launch_mod, shape="square", accel="max")
    assert a_z == pytest.approx(_node_params(out, "figure8_reference_node")["accel"])


@pytest.mark.parametrize("args, match", [
    ({"zigzag_profile": "triangle"}, "zigzag_profile"),
    ({"zigzag_n": "2.5"}, "zigzag_n"), ({"zigzag_n": "0"}, "zigzag_n"),
])
def test_zigzag_rejected_before_nodes(launch_mod, args, match):
    with pytest.raises(RuntimeError, match=match):
        build(launch_mod, shape="zigzag", **args)


@pytest.mark.parametrize("args, accel, speed, msg", [
    ({}, "max", 0.0, "speed:=max · accel:=max"),                  # 비우면 둘 다 max (yaml 공용 speed 0.5·accel 0.4 안 씀)
    ({"speed": "max", "accel": "max"}, "max", 0.0, "speed:=max · accel:=max"),
    ({"accel": "1.0"}, 1.0, 0.0, "speed:=max ("),
    ({"speed": "0.6"}, "max", 0.6, "accel:=max ("),                # speed = 직선 최고 속도 상한
    ({"speed": "0.6", "accel": "1.0"}, 1.0, 0.6, None),            # max 없음 → 계산·로그 없이 그대로
])
def test_zigzag_speed_accel(launch_mod, args, accel, speed, msg):
    """2026-11-24 (8): 지그재그 가속도·속도는 공용 accel·speed — accel = 최대 가속도, speed = 직선 최고 속도 상한 (노드에 0 = 부하 예산)."""
    _, _, out = build(launch_mod, shape="zigzag", **args)
    p = _node_params(out, "figure8_reference_node")
    _, _, ref = build(launch_mod, shape="square", accel="max")
    a_max = _node_params(ref, "figure8_reference_node")["accel"]
    assert p["accel"] == pytest.approx(a_max if accel == "max" else accel) and p["speed"] == speed
    logs = ["".join(getattr(x, "text", str(x)) for x in o.msg) for o in out if type(o).__name__ == "LogInfo" and "zigzag" in "".join(getattr(x, "text", str(x)) for x in o.msg)]
    if msg is None:
        assert not logs
    else:
        assert len(logs) == 1 and logs[0].startswith(msg)


@pytest.mark.parametrize("yaw", ["tangent", "fixed", "spin"])
def test_zigzag_yaw_modes_reach_figure8(launch_mod, yaw):
    """2026-11-24 (9): 지그재그도 yaw_mode tangent | fixed | spin, spin_rate·align_s 가 노드로. fixed·spin 의 accel max 는 경로 전체 앞먹임으로."""
    import yaml
    from amr_mpc.core import fig8_limits
    text = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v))
    _, _, out = build(launch_mod, shape="zigzag", yaw_mode=yaw, spin_rate="2.0", align_s="1.0")
    p = _node_params(out, "figure8_reference_node")
    assert text(p["yaw_mode"]) == yaw and p["spin_rate"] == 2.0 and p["align_s"] == 1.0
    logs = ["".join(getattr(x, "text", str(x)) for x in o.msg) for o in out if type(o).__name__ == "LogInfo" and "zigzag" in "".join(getattr(x, "text", str(x)) for x in o.msg)]
    assert len(logs) == 1 and f"yaw {yaw}" in logs[0] and (("직선 앞먹임" in logs[0]) == (yaw == "tangent"))
    m = re.search(r"앞먹임 ([0-9.]+) ≤", logs[0])
    assert m and float(m.group(1)) <= 1.0 + 1e-9


@pytest.mark.parametrize("args, rot_max", [
    ({"zigzag_reverse": "false"}, True),                          # 늘 전진: zigzag_rot_s 비우면 max
    ({"zigzag_reverse": "false", "zigzag_rot_s": "2.0"}, False),  # 주면 그 값
    ({"zigzag_rot_s": "max"}, True),                              # 전진·후진에서도 max 를 줄 수 있다
    ({}, False),                                                  # 기본 (전진·후진) 은 yaml 0.5
])
def test_zigzag_reverse_and_rot_max(launch_mod, args, rot_max):
    """2026-11-24 (10): zigzag_reverse 가 노드로 (기본 true), zigzag_rot_s:=max 는 경로 전체 앞먹임 ≤ 1 인 가장 짧은 회전 시간으로."""
    from amr_mpc.core import fig8_limits
    from amr_mpc.core.zigzag import Zigzag
    _, _, out = build(launch_mod, shape="zigzag", **args)
    p = _node_params(out, "figure8_reference_node")
    assert p["zigzag_reverse"] is (args.get("zigzag_reverse") != "false")
    rot = p["zigzag_rot_s"]
    assert isinstance(rot, float)
    if not rot_max:
        assert rot == float(args.get("zigzag_rot_s", 0.5))
        return
    logs = ["".join(getattr(x, "text", str(x)) for x in o.msg) for o in out if type(o).__name__ == "LogInfo" and "zigzag" in "".join(getattr(x, "text", str(x)) for x in o.msg)]
    assert len(logs) == 1 and "zigzag_rot_s:=max" in logs[0] and f"꼭짓점 회전 {rot:.2f} s" in logs[0]
    c = fig8_limits.plant_constants(os.path.join(HERE, "..", "config"))
    z = Zigzag(accel=p["accel"], reverse=p["zigzag_reverse"], rot_s=rot)
    assert fig8_limits.ff_ratio(z, c) <= 1.0 + 1e-9
    if not p["zigzag_reverse"]:                                   # 한 칸 (0.05 s) 짧으면 상한을 넘는다 (가장 짧은 값)
        assert fig8_limits.ff_ratio(Zigzag(accel=p["accel"], reverse=False, rot_s=rot - 0.05), c) > 1.0


def test_real_all_passes_zigzag_reverse():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "zigzag_reverse" in m._PASS


def test_zigzag_speed_cap_reaches_peak():
    """speed 상한이 직선 최고 속도를 자른다 (Zigzag v_max) — sine 5 × 1 m, accel 1.7: 상한 없음 0.92, 0.6 이면 0.6."""
    from amr_mpc.core.zigzag import Zigzag
    assert Zigzag(accel=1.7).V == pytest.approx(math.sqrt(1.7 / 2.0))
    assert Zigzag(accel=1.7, v_max=0.6).V == pytest.approx(0.6)


def test_real_all_passes_zigzag_keys():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert {"zigzag_n", "zigzag_length", "zigzag_angle_deg", "zigzag_profile", "zigzag_rot_s", "zigzag_dwell_s", "speed", "accel"} <= set(m._PASS)
    assert not {"zigzag_accel", "zigzag_v_max"} & set(m._PASS)


@pytest.mark.parametrize("arg, expect", [("", False), ("true", True), ("false", False)])
def test_slip_guard_passed_to_torque_output(launch_mod, arg, expect):
    # 2026-11-21 바퀴별 슬립 가드 (amr_mpc/torque/slip_guard.py) — 기본 끔
    _, _, out = build(launch_mod, slip_guard=arg)
    assert _torque_output_params(out)["slip_guard"] is expect


def test_real_all_passes_slip_guard():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "slip_guard" in m._PASS


@pytest.mark.parametrize("arg", ["ground", "total"])
def test_mpc_cap_on_passed(launch_mod, arg):
    # 2026-11-22 friction 상한을 거는 곳 (torque_mpc.yaml torque_limit.cap_on)
    _, _, out = build(launch_mod, limit_mode="friction", mpc_cap_on=arg)
    v = _node_params(out, "torque_mpc_node")["cap_on"]
    import yaml
    assert yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) == arg   # 문자열 값은 YAML 로 실린 TextSubstitution 튜플


def test_mpc_cap_on_rejects_unknown(launch_mod):
    with pytest.raises(RuntimeError):
        build(launch_mod, limit_mode="friction", mpc_cap_on="wheel")


def test_real_all_passes_mpc_cap_on():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "mpc_cap_on" in m._PASS


def _real_all_ctx(**over):
    from launch.actions import GroupAction  # noqa: F401
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    ctx.launch_configurations.update(over)
    return m, ctx


def test_real_all_torque_include_first():
    """2026-11-23: torque_experiment (CAN 준비·인자 검사에서 예외가 날 수 있다) 가 프로세스를 띄우는 어떤 동작보다 먼저 —
    예외가 나도 고아가 안 남는다 (11-23 11:36 CAN 실패가 cartographer 묶음 12 개를 남겼다)."""
    from launch.actions import ExecuteProcess, GroupAction, TimerAction
    m, ctx = _real_all_ctx(sensors="true", localization="true", bag="true", viz="false", auto_localize="false", use_mpc="true")
    m._leftover_nodes = lambda: []
    ents = [e for e in m._nodes(ctx) if isinstance(e, (GroupAction, ExecuteProcess, TimerAction))]
    assert len(ents) >= 3                                    # torque · bag · sensors · cartographer
    first = ents[0]
    assert isinstance(first, GroupAction)
    inc = [e for e in first.get_sub_entities() if isinstance(e, IncludeLaunchDescription)][0]
    txt = lambda x: x if isinstance(x, str) else "".join(s.perform(ctx) for s in x)
    assert {txt(k): txt(v) for k, v in inc.launch_arguments}.get("plant") == "real"


def test_real_all_refuses_with_leftover_nodes():
    """2026-11-23: 고아 노드가 남아 있으면 아무것도 띄우기 전에 멈추고 정리 명령을 알려 준다."""
    m, ctx = _real_all_ctx(sensors="false", localization="false", viz="false", auto_localize="false")
    m._leftover_nodes = lambda: [(4321, "/x/install/amr_laser_tools/lib/amr_laser_tools/scan_merger")]
    with pytest.raises(RuntimeError, match="kill -INT 4321"):
        m._nodes(ctx)


def test_leftover_nodes_finds_real_orphan(monkeypatch):
    """실제 고아 (이중 fork 로 부모를 잃은 프로세스, 같은 ROS_DOMAIN_ID) 를 찾고, 다른 도메인은 안 찾는다."""
    import subprocess, time
    m, _ = _real_all_ctx()
    install = os.path.realpath(os.path.join(m.get_package_share_directory("amr_mpc"), "..", "..", ".."))
    tag = f"{install}/amr_orphan_test_{os.getpid()}"
    monkeypatch.setenv("ROS_DOMAIN_ID", "79")
    subprocess.run(["bash", "-c", f"(ROS_DOMAIN_ID=79 exec -a {tag} sleep 30 &)"], check=True)
    subprocess.run(["bash", "-c", f"(ROS_DOMAIN_ID=78 exec -a {tag}_other sleep 30 &)"], check=True)
    time.sleep(0.3)
    try:
        pids = {p for p, c in m._leftover_nodes() if tag in c}
        mine = subprocess.run(["pgrep", "-f", tag], capture_output=True, text=True).stdout.split()
        if not mine:
            pytest.skip("고아를 못 만들었다")
        parent = int(open(f"/proc/{mine[0]}/stat").read().rsplit(")", 1)[1].split()[1])
        pcomm = open(f"/proc/{parent}/comm").read().strip()
        if parent != 1 and pcomm not in ("systemd", "init"):
            pytest.skip(f"이 환경의 하위 수거자는 {pcomm} — 검사 대상 밖")
        names = {c for p, c in m._leftover_nodes() if tag in c}
        assert any(c.startswith(tag + " ") or c == f"{tag} 30" for c in names)   # 도메인 79 것
        assert not any("_other" in c for c in names)                              # 도메인 78 은 아님
        assert pids
    finally:
        subprocess.run(["pkill", "-f", f"amr_orphan_test_{os.getpid()}"])


def test_speed_max_reaches_figure8_as_numbers(launch_mod):
    """2026-11-23 speed:=max·accel:=max → figure8_reference 에 숫자 (core/fig8_limits.py), 기동 로그 한 줄. 내부 키는 노드로 안 간다."""
    from launch.actions import LogInfo
    _, _, out = build(launch_mod, shape="circles8", width="2.0", yaw_mode="tangent", laps="2", speed="max", accel="max",
                      speed_mod_amp="0.5", speed_mod_cycles="2", speed_mod_phase_deg="-90")
    p = _node_params(out, "figure8_reference_node")
    assert isinstance(p["speed"], float) and isinstance(p["accel"], float) and "_max_msg" not in p
    assert 0.3 < p["speed"] < 0.75 and 0.5 <= p["accel"] <= 3.0
    msgs = ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    assert any("speed:=max" in t and f"{p['speed']:.2f}" in t for t in msgs)


def test_speed_max_rejected_for_line_before_nodes(launch_mod):
    with pytest.raises(RuntimeError, match="speed:=max"):
        build(launch_mod, shape="line", speed="max")


def test_square_speed_max_sets_v_max_zero_and_edge_accel(launch_mod):
    """2026-11-23 shape:=square speed:=max accel:=max → square_v_max 0 (부하 예산에서 자름), accel 은 변 직선 가속 한계."""
    from launch.actions import LogInfo
    _, _, out = build(launch_mod, shape="square", yaw_mode="tangent", square_v_max="1.5", speed="max", accel="max")
    p = _node_params(out, "figure8_reference_node")
    assert p["square_v_max"] == 0.0 and isinstance(p["accel"], float) and 1.0 <= p["accel"] <= 3.0
    msgs = ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    assert any("(square)" in t and f"{p['accel']:.2f}" in t for t in msgs)


def test_real_all_passes_max_ff_ratio():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "max_ff_ratio" in m._PASS


# ── 2026-11-23 구동계 방향 보정 launch 인자 (torque_plant.yaml drive) ──
@pytest.mark.parametrize("controller, exe", [("mpc", "torque_mpc_node"), ("pure_pursuit", "wheel_pi_node"),
                                             ("pp", "wheel_pi_node"), ("adpp", "wheel_pi_node")])
def test_drive_args_default_not_passed(launch_mod, controller, exe):
    """인자를 안 주면 어느 노드에도 drive_* 를 넘기지 않는다 (노드가 torque_plant.yaml drive 기본 0 을 읽는다)."""
    _, _, out = build(launch_mod, controller=controller)
    assert not any(k.startswith("drive_") for k in _node_params(out, exe))
    assert not any(k.startswith("drive_") for k in _torque_output_params(out))


@pytest.mark.parametrize("controller, exe", [("mpc", "torque_mpc_node"), ("pure_pursuit", "wheel_pi_node"),
                                             ("pp", "wheel_pi_node"), ("adpp", "wheel_pi_node")])
def test_drive_args_pass_to_controller_and_output(launch_mod, controller, exe):
    """drive_yaw_deg·drive_slip_coef → 제어기 (MPC·바퀴 PI) 에 둘 다, torque_output 에는 δ0 만."""
    _, _, out = build(launch_mod, controller=controller, drive_yaw_deg="2.4", drive_slip_coef="-2.5")
    p = _node_params(out, exe)
    assert p["drive_yaw_offset_deg"] == 2.4 and p["drive_slip_coef_deg"] == -2.5
    t = _torque_output_params(out)
    assert t["drive_yaw_offset_deg"] == 2.4 and "drive_slip_coef_deg" not in t


def test_drive_args_reject_non_number(launch_mod):
    with pytest.raises(RuntimeError, match="drive_yaw_deg"):
        build(launch_mod, drive_yaw_deg="abc")


def test_real_all_passes_drive_args():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    assert {"drive_yaw_deg", "drive_slip_coef"} <= set(m._PASS)


# ── 2026-11-23 path following (figure8_reference path_follow_lead_s) ──
@pytest.mark.parametrize("given, lead, rate", [("", None, None), ("0.05", 0.05, 50.0), ("0", 0.0, 50.0), ("off", -1.0, 50.0)])
def test_path_follow_lead_reaches_figure8(launch_mod, given, lead, rate):
    """비우면 넘기지 않는다 (노드 기본 −1 = 시간 추종). 주면 lead 와 발행 50 Hz (발행 사이 외삽 몫을 줄인다)."""
    _, _, out = build(launch_mod, shape="circles8", width="1.6", yaw_mode="tangent", path_follow_lead=given)
    p = _node_params(out, "figure8_reference_node")
    assert p.get("path_follow_lead_s") == lead and p.get("rate_hz") == rate


@pytest.mark.parametrize("given", ["1.5", "-0.1", "abc"])
def test_path_follow_lead_rejected(launch_mod, given):
    with pytest.raises((RuntimeError, ValueError)):
        build(launch_mod, shape="circles8", width="1.6", yaw_mode="tangent", path_follow_lead=given)


def test_real_all_passes_path_follow_and_max_com_height():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "path_follow_lead" in m._PASS and "max_com_height" in m._PASS


# ── 2026-11-23 friction_comp (교수님: 구동계 마찰은 d̂ 가 흡수할 외란 — 보상을 끄고 시험) ──
@pytest.mark.parametrize("controller", ["mpc", "pp", "adpp", "pure_pursuit"])
@pytest.mark.parametrize("given, expect", [("", 0.0), ("true", None), ("false", 0.0)])
def test_friction_comp_switch(launch_mod, controller, given, expect):
    """2026-11-24 기본 끔: 비우거나 false 면 torque_output compensation_scale 0 (coulomb·viscous·정지 모두 이 값을 곱한다). true 면 넘기지 않는다 (yaml 0.9)."""
    _, _, out = build(launch_mod, controller=controller, friction_comp=given)
    assert _torque_output_params(out).get("compensation_scale") == expect


def test_real_all_passes_friction_comp():
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "friction_comp" in m._PASS


@pytest.mark.parametrize("given, expect", [
    ({}, {}),                                                                   # 비우면 torque_mpc.yaml slip_eta (끔)
    ({"slip_eta": "true"}, {"slip_eta_enable": True}),
    ({"slip_eta": "false"}, {"slip_eta_enable": False}),
    ({"slip_eta": "true", "slip_eta_lambda_c": "0.2", "slip_eta_s": "0.0167"},
     {"slip_eta_enable": True, "slip_eta_lambda_c": 0.2, "slip_eta_s": 0.0167}),
])
def test_slip_eta_args(launch_mod, given, expect):
    """2026-11-23 슬립 계수 η_s 켬/끔·곡선 덮어쓰기. amrall 도 같은 인자를 넘긴다."""
    import yaml
    _, _, out = build(launch_mod, **given)
    txt = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v)) if isinstance(v, (tuple, list)) else v
    p = {k: txt(v) for k, v in _node_params(out, "torque_mpc_node").items()}
    for k, v in expect.items():
        assert p[k] == (pytest.approx(v) if isinstance(v, float) else v)
    if not given:
        assert not any(k.startswith("slip_eta") for k in p)
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert {"slip_eta", "slip_eta_lambda_c", "slip_eta_s"} <= set(m._PASS)


# ── 2026-11-24 마찰 상한 옮김 (11-23 회의 후속 지시서 T1 7단계 (B), torque_mpc.yaml torque_limit.friction_shift) ──
@pytest.mark.parametrize("given, expect", [
    ({}, None),                                                                                  # 비우면 yaml (끔) — 파라미터를 넘기지 않는다
    ({"mpc_cap_friction_shift": "false"}, False),
    ({"mpc_cap_friction_shift": "false", "friction_comp": "true"}, False),                       # 끈 채면 보상과 상관없다
    ({"mpc_cap_friction_shift": "true", "friction_comp": "false", "limit_mode": "friction"}, True),
    ({"mpc_cap_friction_shift": "true", "friction_comp": "false", "limit_mode": "friction", "mpc_cap_on": "ground"}, True),
])
def test_cap_friction_shift_args(launch_mod, given, expect):
    _, _, out = build(launch_mod, **given)
    p = _node_params(out, "torque_mpc_node")
    if expect is None:
        assert "cap_friction_shift" not in p
    else:
        assert p["cap_friction_shift"] is expect
    if expect:                                                # 켠 판은 torque_output 보상이 0 이어야 한다
        assert _torque_output_params(out).get("compensation_scale") == 0.0


def test_cap_friction_shift_needs_comp_off(launch_mod):
    """보상을 켠 채 (friction_comp:=true) 상한을 옮기면 구동계 마찰을 두 번 센다 → 기동 거부. 2026-11-24 부터 비움 = 끔이라 그냥 켜진다."""
    with pytest.raises(RuntimeError, match="friction_comp:=false"):
        build(launch_mod, mpc_cap_friction_shift="true", friction_comp="true", limit_mode="friction")
    _, _, out = build(launch_mod, mpc_cap_friction_shift="true", limit_mode="friction")
    assert _node_params(out, "torque_mpc_node")["cap_friction_shift"] is True
    assert _torque_output_params(out).get("compensation_scale") == 0.0


@pytest.mark.parametrize("controller", ["pp", "adpp"])
def test_cap_friction_shift_ignored_by_pp(launch_mod, controller):
    """PP 분기는 MPC 상한을 안 쓴다 — 인자를 줘도 거부하지 않고 안내만."""
    exes, _, _ = build(launch_mod, controller=controller, mpc_cap_friction_shift="true")
    assert "torque_mpc_node" not in exes and "wheel_pi_node" in exes


def test_cap_friction_shift_yaml_default_off_and_passed_by_real_all():
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        tl = yaml.safe_load(f)["torque_limit"]
    assert tl["friction_shift"] is True and tl["friction_shift_scale"] == 1.0          # 2026-11-24 사용자 결정: 기본 켬
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "mpc_cap_friction_shift" in m._PASS


# ── 2026-11-24 마찰 모델 (지시서 T11b 앞당김, torque_mpc.yaml model_terms.use_fric_model) ──
@pytest.mark.parametrize("given, expect", [
    ({}, None),
    ({"use_fric_model": "false"}, False),
    ({"use_fric_model": "false", "friction_comp": "true"}, False),
    ({"use_fric_model": "true", "friction_comp": "false", "limit_mode": "friction", "mpc_cap_friction_shift": "true"}, True),
    ({"use_fric_model": "true", "friction_comp": "false"}, True),                       # 상한 옮김 없이도 켤 수는 있다 (안내만)
])
def test_use_fric_model_args(launch_mod, given, expect):
    _, _, out = build(launch_mod, **given)
    p = _node_params(out, "torque_mpc_node")
    if expect is None:
        assert "use_fric_model" not in p
    else:
        assert p["use_fric_model"] is expect
    if expect:
        assert _torque_output_params(out).get("compensation_scale") == 0.0


def test_use_fric_model_needs_comp_off(launch_mod):
    with pytest.raises(RuntimeError, match="friction_comp:=false"):
        build(launch_mod, use_fric_model="true", friction_comp="true", limit_mode="friction")
    _, _, out = build(launch_mod, use_fric_model="true", limit_mode="friction")          # 2026-11-24 비움 = 보상 끔
    assert _node_params(out, "torque_mpc_node")["use_fric_model"] is True


def test_use_fric_model_yaml_default_off_and_passed_by_real_all():
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        assert yaml.safe_load(f)["model_terms"]["use_fric_model"] is True               # 2026-11-24 사용자 결정: 기본 켬
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "use_fric_model" in m._PASS


# ── 2026-11-24 구름 손실 (torque_mpc.yaml model_terms.fric_rolling_Nm, launch mpc_fric_rolling · sim_rolling) ──
def test_fric_rolling_args(launch_mod):
    on = {"use_fric_model": "true", "friction_comp": "false", "limit_mode": "friction", "mpc_cap_friction_shift": "true"}
    _, _, out = build(launch_mod, **on)
    assert "fric_rolling" not in _node_params(out, "torque_mpc_node")                 # 안 주면 노드가 yaml 기본 (0) 을 쓴다
    assert _node_params(out, "sim_plant_torque_node")["rolling_Nm"] == 0.0
    _, _, out = build(launch_mod, mpc_fric_rolling="0.0027", sim_rolling="0.0051", **on)
    assert _node_params(out, "torque_mpc_node")["fric_rolling"] == pytest.approx(0.0027)
    assert _node_params(out, "sim_plant_torque_node")["rolling_Nm"] == pytest.approx(0.0051)
    _, _, out = build(launch_mod, mpc_fric_rolling="0.0027")                           # 2026-11-24 마찰 모델 yaml 기본이 켬 — 인자만 줘도 넘긴다
    assert _node_params(out, "torque_mpc_node")["fric_rolling"] == pytest.approx(0.0027)
    for off in ({"use_fric_model": "false"}, {"friction_comp": "true"}):               # 마찰 모델이 꺼진 판 (보상을 켠 판 포함) 에는 넘기지 않는다
        _, _, out = build(launch_mod, mpc_fric_rolling="0.0027", **off)
        assert "fric_rolling" not in _node_params(out, "torque_mpc_node")


def test_fric_rolling_yaml_default_zero_and_passed_by_real_all():
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        assert yaml.safe_load(f)["model_terms"]["fric_rolling_Nm"] == 0.0
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "mpc_fric_rolling" in m._PASS


# ── 2026-11-24 사용자 결정 "기본 꺼": 주행 launch 의 torque_output 마찰 보상은 인자를 안 줘도 꺼진다 ──
def test_friction_comp_default_is_off(launch_mod):
    from launch.actions import LogInfo
    msgs = lambda out: ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    arg = next(e for e in launch_mod.generate_launch_description().entities
               if isinstance(e, DeclareLaunchArgument) and e.name == "friction_comp")
    assert "".join(s.perform(LaunchContext()) for s in arg.default_value) == "false"
    _, _, out = build(launch_mod)                                                       # 기본 (motor_only): 보상 끔, 안내 없음
    assert _torque_output_params(out).get("compensation_scale") == 0.0
    assert not any("마찰 상한 옮김도 꺼져 있다" in m for m in msgs(out))
    _, _, out = build(launch_mod, limit_mode="friction", mpc_cap_friction_shift="false")   # friction 상한인데 상한 옮김을 끄면 알린다
    assert any("마찰 상한 옮김도 꺼져 있다" in m for m in msgs(out))
    _, _, out = build(launch_mod, limit_mode="friction")                                # yaml 기본 (옮김 켬) 이면 안내 없음
    assert not any("마찰 상한 옮김도 꺼져 있다" in m for m in msgs(out))
    _, _, out = build(launch_mod, limit_mode="friction", mpc_cap_friction_shift="true")
    assert not any("마찰 상한 옮김도 꺼져 있다" in m for m in msgs(out))
    _, _, out = build(launch_mod, limit_mode="friction", friction_comp="true")          # 보상을 켠 판: yaml 0.9, 상한 옮김·마찰 모델은 알아서 꺼진다
    assert "compensation_scale" not in _torque_output_params(out)
    assert not any("마찰 상한 옮김도 꺼져 있다" in m for m in msgs(out))
    p = _node_params(out, "torque_mpc_node")
    assert p["cap_friction_shift"] is False and p["use_fric_model"] is False


def test_mpc_yaml_defaults_2026_11_24():
    """사용자 결정 "둘다 키고 외란도 켜": 마찰 상한 옮김 · 마찰 모델 · 외란 추정의 yaml 기본이 켬. 구름 손실 값은 0 그대로."""
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        y = yaml.safe_load(f)
    assert y["torque_limit"]["friction_shift"] is True and y["model_terms"]["use_fric_model"] is True
    assert y["disturbance"]["enable"] is True and y["model_terms"]["fric_rolling_Nm"] == 0.0


@pytest.mark.parametrize("name", ["torque_step.launch.py", "torque_ramp.launch.py"])
def test_step_and_ramp_keep_compensation_on(name):
    """amrstep · amrramp 는 "바닥 토크 τ_g 를 준다" 가 정의 — 보상을 명시로 켠 채 real_all 에 넘긴다 (friction_comp:=false 로 끌 수 있다)."""
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), os.path.join(HERE, "..", "launch", name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert m._ARGS["friction_comp"] == ("friction_comp", "true")


def test_zigzag_corner_pass_reaches_figure8(launch_mod):
    """2026-11-25 zigzag_corner:=pass·zigzag_corner_speed·zigzag_turn_m 가 노드로. pass 면 zigzag_rot_s 를 max 로 찾지 않는다. real_all 도 넘긴다."""
    import yaml
    text = lambda v: yaml.safe_load("".join(getattr(x, "text", str(x)) for x in v))
    logs_of = lambda out: ["".join(getattr(x, "text", str(x)) for x in o.msg) for o in out
                           if type(o).__name__ == "LogInfo" and "zigzag" in "".join(getattr(x, "text", str(x)) for x in o.msg)]
    base = dict(shape="zigzag", zigzag_profile="square", zigzag_reverse="false", zigzag_corner="pass")
    _, _, out = build(launch_mod, zigzag_corner_speed="0.5", zigzag_turn_m="0", **base)      # 꺾인 채
    p = _node_params(out, "figure8_reference_node")
    assert text(p["zigzag_corner"]) == "pass" and p["zigzag_corner_speed"] == 0.5 and p["zigzag_turn_m"] == 0.0 and p["zigzag_rot_s"] == 0.5
    logs = logs_of(out)
    assert len(logs) == 1 and "꼭짓점 정지 없이 0.50 m/s (꺾인 채)" in logs[0]
    with pytest.raises(RuntimeError, match="zigzag_turn_m > 0"):
        build(launch_mod, zigzag_turn_m="0", **base)                     # 꺾인 채 + corner_speed max (yaml 기본)
    with pytest.raises(RuntimeError, match="zigzag_corner"):
        build(launch_mod, shape="zigzag", zigzag_corner="glide")
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert {"zigzag_corner", "zigzag_corner_speed", "zigzag_turn_m"} <= set(m._PASS)


def test_zigzag_corner_pass_round_speed_max(launch_mod):
    """turn_m > 0 (yaml 0.5) + zigzag_corner_speed 비움 (yaml max) → C 앞먹임 ≤ 1 안의 최대 v_c 가 숫자로, 로그에 turn 곡률."""
    import re
    _, _, out = build(launch_mod, shape="zigzag", zigzag_profile="square", zigzag_reverse="false", zigzag_corner="pass",
                      zigzag_n="4", zigzag_angle_deg="30")
    p = _node_params(out, "figure8_reference_node")
    assert p["zigzag_turn_m"] == 0.5 and 0.05 <= p["zigzag_corner_speed"] < 1.0
    logs = ["".join(getattr(x, "text", str(x)) for x in o.msg) for o in out
            if type(o).__name__ == "LogInfo" and "zigzag" in "".join(getattr(x, "text", str(x)) for x in o.msg)]
    assert len(logs) == 1 and "zigzag_corner_speed:=max" in logs[0] and "κ_max" in logs[0]
    m = re.search(r"turn 포함 최대 ([0-9.]+)", logs[0])
    assert m and float(m.group(1)) <= 1.0 + 1e-9


# ── 2026-11-25 꺾은선 shape:=polyline (core/polyline.py) ──
def test_polyline_shape_and_path_reach_node(launch_mod):
    import yaml
    txt = lambda v: v if isinstance(v, str) else yaml.safe_load("".join(getattr(s, "text", str(s)) for s in v))
    _, _, out = build(launch_mod, shape="polyline")
    p = {k: txt(v) for k, v in _node_params(out, "figure8_reference_node").items() if k in ("shape", "poly_path", "square_corner_mode")}
    assert p["shape"] == "polyline" and p["poly_path"] == "R1,L1,L2,R1,R1,R2"        # torque_track.yaml 기본
    _, _, out = build(launch_mod, shape="polyline", poly_path="F1,L1,L2,R1,R1,R2", square_corner_mode="stop")
    p = {k: txt(v) for k, v in _node_params(out, "figure8_reference_node").items() if k in ("shape", "poly_path", "square_corner_mode")}
    assert p["poly_path"] == "F1,L1,L2,R1,R1,R2" and p["square_corner_mode"] == "stop"
    _, _, out = build(launch_mod, shape="polyline")                                   # 2026-11-25 오후: 꺾은선도 yaml 기본 (pass) 그대로
    assert txt(_node_params(out, "figure8_reference_node")["square_corner_mode"]) == "pass"
    _, _, out = build(launch_mod, shape="polyline", square_corner_mode="stop")
    assert txt(_node_params(out, "figure8_reference_node")["square_corner_mode"]) == "stop"
    _, _, out = build(launch_mod, shape="square")                                     # 정사각형은 yaml 기본 (pass) 그대로
    assert txt(_node_params(out, "figure8_reference_node")["square_corner_mode"]) == "pass"
    with pytest.raises(RuntimeError, match="poly_path"):
        build(launch_mod, shape="polyline", poly_path="X1")
    with pytest.raises(RuntimeError, match="spin"):
        build(launch_mod, shape="polyline", yaw_mode="spin")
    spec = importlib.util.spec_from_file_location("real_all_launch", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "poly_path" in m._PASS


def test_step_rotation_args_2026_11_25(launch_mod, monkeypatch):
    """제자리 회전 단계: step_dir:=ccw step_angle_deg 가 노드 angle_deg 로. real_all·amrstep 도 넘긴다. amrstep 은 회전이면 angle 을 꼭 받는다."""
    monkeypatch.setattr(os.path, "exists", lambda p: True)
    _, _, out = build(launch_mod, plant="real", use_mpc="true", controller="step", step_dir="ccw", step_tau="0.08", step_angle_deg="90")
    p = _node_params(out, "torque_step_node")
    import yaml
    assert p["angle_deg"] == 90.0 and yaml.safe_load("".join(getattr(x, "text", str(x)) for x in p["direction"])) == "ccw"
    spec = importlib.util.spec_from_file_location("real_all_rot", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "step_angle_deg" in m._PASS
    spec = importlib.util.spec_from_file_location("torque_step_launch_rot", os.path.join(HERE, "..", "launch", "torque_step.launch.py"))
    s = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(s)
    assert s._ARGS["angle"] == ("step_angle_deg", "")


def test_roller_and_inertia_args_2026_11_25(launch_mod, monkeypatch):
    """mpc_roller_rr·mpc_inertia_z 는 MPC 노드 roller_rr·inertia_z 로, sim_roller_rr 는 시뮬 플랜트 roller_rr_N 으로. 비우면 안 넘긴다 (yaml 0·0.06). real_all 도 넘긴다."""
    _, _, out = build(launch_mod, mpc_roller_rr="0.88", mpc_inertia_z="0.03", sim_roller_rr="0.88", sim_inertia_z="0.03")
    p = _node_params(out, "torque_mpc_node")
    assert p["roller_rr"] == 0.88 and p["inertia_z"] == 0.03
    sp = _node_params(out, "sim_plant_torque_node")
    assert sp["roller_rr_N"] == 0.88 and sp["inertia_z"] == 0.03
    _, _, out = build(launch_mod)
    p = _node_params(out, "torque_mpc_node")
    assert "roller_rr" not in p and "inertia_z" not in p
    assert _node_params(out, "sim_plant_torque_node")["roller_rr_N"] == 0.0 and _node_params(out, "sim_plant_torque_node")["inertia_z"] == -1.0
    spec = importlib.util.spec_from_file_location("real_all_roller", os.path.join(HERE, "..", "launch", "real_all.launch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert "mpc_roller_rr" in m._PASS and "mpc_inertia_z" in m._PASS
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_mpc.yaml")) as f:
        y = yaml.safe_load(f)
    assert y["model_terms"]["roller_rr_N"] == 0.0 and y["model_terms"]["roller_s0_mps"] == 0.02
