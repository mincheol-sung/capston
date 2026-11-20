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



@pytest.mark.parametrize("given, expect", [
    ("", None),                                          # 비우면 넘기지 않는다 → 노드가 torque_plant.yaml surface.mu_hat_wheels
    ("0.432,0.285,0.456,0.350", [0.432, 0.285, 0.456, 0.350]),
    ("off", [-1.0, -1.0, -1.0, -1.0]),                   # 끄기 → 네 바퀴 모두 mu_hat × 하중 몫
    ("0.4,-1,0.45,0.5", [0.4, -1.0, 0.45, 0.5]),         # 0 이하인 바퀴만 mu_hat
])
def test_mu_hat_wheels_reaches_mpc(launch_mod, given, expect):
    """2026-11-19 (11) 바퀴별 μ̂ (mg/4 기준) → torque_mpc_node. 시뮬 플랜트에는 안 넘어간다 (물리는 surface_mu·하중 몫)."""
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
    """YAML 에 바퀴별 μ̂ 가 있으면 mu_hat:= 는 MPC 상한에 안 쓰인다 — launch 로그로 알리고, off 면 안 알린다."""
    from launch.actions import LogInfo
    import yaml
    with open(os.path.join(HERE, "..", "config", "torque_plant.yaml")) as f:
        assert yaml.safe_load(f)["surface"]["mu_hat_wheels"]          # 지금 YAML 에 값이 있다
    def msgs(**a):
        _, _, out = build(launch_mod, **a)
        return ["".join(getattr(s, "text", str(s)) for s in n.msg) for n in out if isinstance(n, LogInfo)]
    assert any("안 쓰인다" in t and "mu_hat:=0.6" in t for t in msgs(limit_mode="friction", mu_hat="0.6"))
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
