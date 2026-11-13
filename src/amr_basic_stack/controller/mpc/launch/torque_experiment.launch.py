#!/usr/bin/env python3
"""토크 MPC 실험 launch (구조도 v3) — 기존 autonomy.launch.py 와 별개다.

기존 스택(RoboClaw, EKF, DRA-MPPI, 인지)은 띄우지 않는다. 기존 경로는 그대로 남겨 둔다.
단계별로 노드가 늘어난다 (amr/notes_amr/stack_changelog.md, torque_mpc_todo.md).

  지금 (6단계) — 시뮬 폐루프:
    figure8_reference_node ─기준열─▶ torque_mpc_node ─▶ torque_output_node ─▶ 게이트('a' 무장) ─▶ sim_plant_torque_node
                                          ▲                                                          │ 가짜 IMU 200 Hz,
                                          └── /state_estimator/odom ◀── state_estimator_node ◀───────┘ 가짜 SLAM 포즈 10 Hz (0.13 s 늦게)
    controller:=pure_pursuit (비교군 A) 이면 torque_mpc_node 자리에
        pure_pursuit_node ─(v_x, ω_z)─▶ wheel_pi_node (+ /wheel_states) ─▶ torque_output_node ─▶ … (뒤는 같다)
  7단계:  plant:=sim_can → 시뮬 플랜트가 가짜 AK40 모터(vcan0)가 되고 ak40_can_node 가 토픽과 잇는다 (CAN 전 구간 시험)
          plant:=real    → ak40_can_node(can0) + IMU 회전 노드. 센서·Cartographer 는 따로 띄운다

  state_source:=truth       추정기 없이 시뮬 참값(/sim/ground_truth)을 바로 쓴다 (3단계 방식, 제어기만 볼 때)
  slam_mode:=extrapolated   지금 실차 Cartographer 설정 흉내 (외삽 켬, 100 Hz). 기본 scan_time = 외삽 끔
  sensor_noise_scale:=0     센서 잡음 없이 (바이어스·지연은 그대로)
  wheel_odom:=true          휠 오돔(/wheel_odom)을 만든다 — Cartographer 초기 짐작용 선택 입력. 기본은
                            config/torque_estimator.yaml slam.use_wheel_odom (false). 시뮬에는 Cartographer 가 없어
                            노드만 뜬다. 실차(7단계)에서는 Cartographer lua 도 휠 오돔 켠 판으로 고른다

사용 (새 터미널에서 amr 을 먼저)
  ros2 launch amr_mpc torque_experiment.launch.py                       # 8자 한 바퀴. 이 터미널에서 a = 무장, s/SPACE = 정지
                                                                        #   무장해야 기준 시간이 흐른다. 정지하면 기준도 멈추고, 다시 a 로 이어 간다
  ros2 launch amr_mpc torque_experiment.launch.py armed:=true           # 띄우자마자 무장 (시뮬에서만)
  ros2 launch amr_mpc torque_experiment.launch.py limit_mode:=friction  # 비교군 C (마찰 토크 상한). 기본 motor_only = B
  ros2 launch amr_mpc torque_experiment.launch.py controller:=pure_pursuit   # 비교군 A (퓨어 퍼슛 + 바퀴별 PI).
                                                                        #   게인은 config/torque_baseline_a.yaml
  ros2 launch amr_mpc torque_experiment.launch.py disturbance:=true     # 외란 추정치 d̂ 켬 (기본 끔)
  ros2 launch amr_mpc torque_experiment.launch.py payload_kg:=5.0       # 적재 +5 kg (플랜트와 제어기 둘 다)
  ros2 launch amr_mpc torque_experiment.launch.py payload_kg:=5.0 ctrl_payload_kg:=0.0   # 제어기는 모름 (질량 오차 시험)
  ros2 launch amr_mpc torque_experiment.launch.py use_mpc:=false        # 플랜트·RViz 만 (손으로 토크 주기, tools/torque_demo_drive.py)
  8자 위치·모양·속도는 config/torque_track.yaml (지도 고정, 2026-11-11). 한 번만 바꿀 때: speed:=0.8 laps:=2 origin_x:=1.0
    로봇을 출발점(origin) 근처에 놓고 'a' — 30 cm·30° 안이면 천천히 출발점으로 들어간 뒤 8자, 밖이면 출발 거부.
    예전처럼 로봇이 있는 곳을 교차점으로: origin_mode:=robot

비워 두는 인자는 config/*.yaml 값을 쓴다 (torque_plant, torque_mpc, torque_track).
"""
import yaml
import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amr_mpc.ak40.setup import prepare_can


def _truthy(s):
    return str(s).strip().lower() in ("true", "1", "yes", "on")


# config/torque_track.yaml 의 키. launch 인자 이름이 같고, 비워 두면 YAML 값을 쓴다
_TRACK_KEYS = ["origin_mode", "origin_x", "origin_y", "origin_yaw_deg", "start_tolerance_xy",
               "start_tolerance_yaw_deg", "lead_speed", "lead_yaw_rate", "width", "height", "yaw_mode",
               "speed", "accel", "laps", "align_s", "start_delay", "abort_error", "hold_s", "lap_speeds",
               "dt", "horizon_steps"]


def _track(context, share):
    with open(os.path.join(share, "config", "torque_track.yaml")) as f:
        track = yaml.safe_load(f)["track"]
    for k in _TRACK_KEYS:
        v = LaunchConfiguration(k).perform(context).strip()
        if v:
            track[k] = v
    out = {}
    for k in _TRACK_KEYS:                             # 노드 파라미터 타입에 맞춘다
        if k == "lap_speeds":                         # 목록. launch 인자는 "[0.3, 0.5]" 꼴 문자열
            v = yaml.safe_load(track[k]) if isinstance(track[k], str) else track[k]
            out[k] = [float(x) for x in (v or [])] or [0.0]
        elif k == "horizon_steps":
            out[k] = int(float(track[k]))
        else:
            out[k] = str(track[k]) if k in ("origin_mode", "yaw_mode") else float(track[k])
    return out


def _nodes(context):
    g = lambda n: LaunchConfiguration(n).perform(context).strip()
    share = get_package_share_directory("amr_mpc")
    plant = g("plant")
    source, slam_mode = g("state_source"), g("slam_mode")
    state_topic = g("state_topic") or ("/state_estimator/odom" if source == "estimator" else "/sim/ground_truth")
    nodes = []
    # Run before returning any motor/controller actions. Never runs for simulation.
    if plant == "real":
        if source != "estimator":
            raise RuntimeError("plant:=real requires state_source:=estimator")
        if _truthy(g("auto_setup_can")):
            with open(os.path.join(share, "config", "ak40.yaml")) as f:
                can_config = yaml.safe_load(f)["ak40"]
            interface = g("can_interface") or str(can_config["interface"])
            result = prepare_can(os.path.join(share, "tools", "setup_can.sh"), interface, g("can_device"))
            nodes.append(LogInfo(msg=f"CAN setup: {result}"))

    # 측위 입력 선택 (휠 오돔 스위치) — config/torque_estimator.yaml slam 절, launch 인자 wheel_odom 이 덮어쓴다
    with open(os.path.join(share, "config", "torque_estimator.yaml")) as f:
        slam = yaml.safe_load(f)["slam"]
    use_wheel_odom = _truthy(g("wheel_odom")) if g("wheel_odom") else bool(slam["use_wheel_odom"])
    carto_lua = slam["lua_with_wheel_odom"] if use_wheel_odom else slam["lua_without_wheel_odom"]
    if use_wheel_odom:
        nodes.append(Node(
            package="amr_mpc", executable="wheel_odometry_node", name="wheel_odometry_node", output="screen",
            parameters=[{"output_topic": slam["wheel_odom_topic"]}]))

    # 플랜트와 제어기가 같이 쓰는 모델 설정. 비어 있으면 YAML 값
    shared = {}
    if g("use_wheel_inertia"):
        shared["use_wheel_inertia"] = _truthy(g("use_wheel_inertia"))
    payload = float(g("payload_kg"))

    # ── 플랜트
    if plant in ("sim", "sim_can"):
        # sim_can: 시뮬 플랜트가 가짜 AK40 모터가 되어 CAN 으로만 대화하고, ak40_can_node 가 토픽과 잇는다 (7단계)
        can_if = g("can_interface") or "vcan0"
        nodes.append(Node(
            package="amr_mpc", executable="sim_plant_torque_node", name="sim_plant_torque_node", output="screen",
            parameters=[{**shared, "payload_kg": payload,
                         "x0": float(g("x0")), "y0": float(g("y0")), "yaw0": float(g("yaw0")),
                         "sensors": source == "estimator", "slam_mode": slam_mode,
                         "sensor_noise_scale": float(g("sensor_noise_scale")),
                         "surface_mu": float(g("surface_mu")) if g("surface_mu") else -1.0,
                         "friction_scale": float(g("friction_scale")),
                         "can_interface": can_if if plant == "sim_can" else ""}]))
        if plant == "sim_can":
            nodes.append(Node(
                package="amr_mpc", executable="ak40_can_node", name="ak40_can_node", output="screen",
                parameters=[{"interface": can_if}]))
    elif plant == "real":
        if source != "estimator":
            raise RuntimeError("plant:=real 에는 참값이 없다 — state_source:=estimator 로")
        # CAN 준비 성공 후에만 모터 시작 (hardware_checked=false이면 토크 제한).
        nodes.append(Node(
            package="amr_mpc", executable="ak40_can_node", name="ak40_can_node", output="screen",
            parameters=[{"interface": g("can_interface")}] if g("can_interface") else []))
        # IMU: D455 원본(/camera/imu, optical 프레임) → 몸체 프레임 /imu/base_link (localization.launch.py 와 같은 노드)
        nodes.append(Node(
            package="amr_perception_bringup", executable="imu_to_base_node", name="imu_to_base_node", output="screen",
            parameters=[{"target_frame": "base_footprint"}],
            remappings=[("imu_in", "/camera/imu"), ("imu_out", "/imu/base_link")]))
        nodes.append(LogInfo(msg=(
            "plant:=real — 이 launch 는 모터(ak40_can_node)와 IMU 회전 노드만 띄운다. 다른 터미널에서 먼저:\n"
            "  ① 센서: ros2 launch amr_bringup sensors.launch.py   (라이다·D455 IMU·robot_state_publisher)\n"
            f"  ② 측위: ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=<지도 이름> lua_basename:={carto_lua}\n"
            "     (저장된 지도 pbstream 위에서, 외삽 끔 → /tracked_pose 스캔 시각 stamp)\n"
            "  ★ 실차 센서 묶음은 아직 실차로 확인하지 않았다 (7단계 기록 참고). 인자 이름은 각 launch 파일에서 확인할 것")))
    else:
        raise RuntimeError(f"plant:={plant!r} — sim, sim_can 또는 real")

    # ── 상태 추정기 (SLAM 포즈 + IMU → 100 Hz 상태)
    if source == "estimator":
        nodes.append(Node(
            package="amr_mpc", executable="state_estimator_node", name="state_estimator_node", output="screen",
            parameters=[{"pose_source": "stamped" if slam_mode == "scan_time" else "after_scan_match"}]))

    if not _truthy(g("use_mpc")):
        return nodes

    # ── 기준궤적: 8자 시간 기준 (기동 포즈가 교차점)
    if _truthy(g("use_figure8")):
        nodes.append(Node(
            package="amr_mpc", executable="figure8_reference_node", name="figure8_reference_node", output="screen",
            parameters=[{
                **_track(context, share),
                "frame_id": "map", "odom_topic": state_topic,
                "wait_for_arm": True, "armed_topic": "/mpc/armed",   # 무장된 동안만 기준 시간이 흐른다
                # 기준 부하 검사: 정격 속도 370 rpm ≈ 38.746 rad/s를 기본 운용 상한으로 사용.
                "max_wheel_rad_per_sec": float(g("max_wheel_rad_per_sec")),
            }]))

    # ── 제어기. controller:=mpc (비교군 B·C, 기본) | pure_pursuit (비교군 A, 6단계)
    #    둘 다 같은 기준열·같은 상태를 받고, 바퀴 토크를 같은 torque_output → 게이트 → 플랜트 경로로 낸다.
    controller = g("controller")
    if controller == "mpc":
        # payload_kg 는 "제어기가 믿는" 적재 (비우면 플랜트와 같게)
        ctrl = {**shared, "state_topic": state_topic, "armed_topic": "/mpc/armed",
                "payload_kg": float(g("ctrl_payload_kg")) if g("ctrl_payload_kg") else payload,
                "log_prefix": g("log_prefix")}
        if g("limit_mode"):
            ctrl["limit_mode"] = g("limit_mode")
        if g("disturbance"):
            ctrl["disturbance_enable"] = _truthy(g("disturbance"))
        if g("mu_hat"):
            ctrl["mu_hat"] = float(g("mu_hat"))
        nodes.append(Node(
            package="amr_mpc_cpp", executable="torque_mpc_node", name="torque_mpc_node", output="screen",
            parameters=[ctrl]))
        torque_in, track_status = "/torque_mpc/cmd_raw", "/torque_mpc/status"
    elif controller == "pure_pursuit":
        # 비교군 A: 퓨어 퍼슛 (v_x, ω_z) → 바퀴별 PI → 토크. 게인은 config/torque_baseline_a.yaml
        ignored = [k for k in ("limit_mode", "disturbance", "mu_hat", "ctrl_payload_kg", "log_prefix") if g(k)]
        if ignored:
            nodes.append(LogInfo(msg=f"controller:=pure_pursuit — MPC 인자 {ignored} 는 쓰지 않는다 "
                                     f"(log_prefix 는 slip_logger 에만 쓴다)"))
        nodes.append(Node(
            package="amr_mpc", executable="pure_pursuit_node", name="pure_pursuit_node", output="screen",
            parameters=[{"state_topic": state_topic, "armed_topic": "/mpc/armed"}]))
        nodes.append(Node(
            package="amr_mpc", executable="wheel_pi_node", name="wheel_pi_node", output="screen",
            parameters=[{"armed_topic": "/mpc/armed"}]))
        torque_in, track_status = "/wheel_pi/cmd_raw", "/pure_pursuit/status"
    else:
        raise RuntimeError(f"controller:={controller!r} — mpc 또는 pure_pursuit")

    nodes.append(Node(
        package="amr_mpc", executable="torque_output_node", name="torque_output_node", output="screen",
        parameters=[{"input_topic": torque_in, "max_wheel_rad_per_sec": float(g("max_wheel_rad_per_sec"))}]))

    # 위험 감시: 무장 중 포즈 튐·추종 오차·모터 에러·센서 끊김이면 /mpc/arm false → DAMPING
    if source == "estimator":                         # 참값 모드에는 추정기 출력이 없어 끊김으로 오판한다
        nodes.append(Node(
            package="amr_mpc", executable="fault_monitor_node", name="fault_monitor_node", output="screen",
            parameters=[{"max_track_error": float(g("max_track_error")), "track_status_topic": track_status}]))

    # 슬립률 기록 (로깅 전용). 시뮬이면 참값으로 구한 슬립률도 같이 남긴다
    nodes.append(Node(
        package="amr_mpc", executable="slip_logger_node", name="slip_logger_node", output="screen",
        parameters=[{"body_topic": state_topic,
                     "reference_topic": "/sim/ground_truth" if plant in ("sim", "sim_can") else "",
                     "log_prefix": (g("log_prefix") + "_slip") if g("log_prefix") else ""}]))

    # ── 게이트: 무장 전에는 DAMPING 만 나간다. 받는 즉시 중계 (100 Hz 명령을 재샘플하지 않는다)
    nodes.append(Node(
        package="amr_mpc", executable="keyboard_command_gate", name="torque_command_gate", output="screen",
        parameters=[{
            "command_type": "torques",
            "input_topic": "/torque_output/cmd_raw", "output_topic": "/wheel_torque_cmd",
            "arm_topic": "/mpc/arm", "initially_armed": _truthy(g("armed")),
            "keyboard_enabled": True, "command_timeout": 0.20, "publish_frequency": 50.0,
            "forward_on_receive": True,
        }]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("amr_mpc")
    rviz_cfg = os.path.join(share, "rviz", "torque_experiment.rviz")
    with open(os.path.join(share, "config", "torque_plant.yaml")) as f:
        rated_rpm = float(yaml.safe_load(f)["motor"]["rated_speed_rpm"])
    rated_rad_s = rated_rpm * 2.0 * math.pi / 60.0
    A = DeclareLaunchArgument
    return LaunchDescription([
        A("plant", default_value="sim", choices=["sim", "sim_can", "real"],
          description="sim = 시뮬 플랜트(토픽), sim_can = 시뮬 플랜트가 가짜 AK40 + ak40_can_node (CAN 전 구간), "
                      "real = ak40_can_node + IMU 회전 (센서·Cartographer 는 따로)"),
        A("can_interface", default_value="", description="CAN 장치. 비우면 sim_can 은 vcan0, real 은 config/ak40.yaml (can0)"),
        A("auto_setup_can", default_value="true", choices=["true", "false"],
          description="plant:=real 에서 모터 시작 전 CAN 자동 준비. sudo는 비대화형; sim에는 적용 안 함"),
        A("can_device", default_value="", description="slcan USB 장치 경로. 비우면 CANable by-id 자동 선택"),
        A("state_source", default_value="estimator", choices=["estimator", "truth"],
          description="estimator = state_estimator_node (SLAM 포즈 + IMU), truth = 시뮬 참값을 바로"),
        A("slam_mode", default_value="scan_time", choices=["scan_time", "extrapolated"],
          description="시뮬 SLAM 꼴. scan_time = 스캔 시각 포즈를 늦게 (외삽 끔), extrapolated = 100 Hz 외삽 스트림 (지금 실차 설정)"),
        A("sensor_noise_scale", default_value="1.0", description="시뮬 센서 잡음 배율 (1 = bag 실측 수준, 0 = 없음)"),
        A("state_topic", default_value="", description="상태 토픽을 직접 줄 때만 (비우면 state_source 에 따라 정한다)"),
        A("max_track_error", default_value="0.30", description="fault_monitor: 기준과 이만큼 벌어지면 정지 [m]"),
        A("surface_mu", default_value="", description="시뮬 바닥 마찰계수 (비우면 슬립 없음). 제어기의 mu_hat 과 따로 준다"),
        A("friction_scale", default_value="1.0",
          description="시뮬 구동계 마찰 = torque_plant.yaml friction × 이 값 (0 = 마찰 없음, 1.3 = 보상보다 30 % 큼)"),
        A("wheel_odom", default_value="", description="휠 오돔 true/false (비우면 torque_estimator.yaml slam.use_wheel_odom, 기본 false)"),
        A("use_mpc", default_value="true", description="false 면 플랜트(+RViz)만 띄운다"),
        A("use_figure8", default_value="true", description="8자 기준궤적 노드. false 면 기준열을 밖에서 준다"),
        A("armed", default_value="false", description="게이트 초기 상태. false 면 터미널에서 'a' 로 무장"),
        # 모델·비교군. 비우면 config/torque_plant.yaml, torque_mpc.yaml 값
        A("payload_kg", default_value="0.0", description="적재 [kg]. 2차 발표 조건 0 / 5"),
        A("ctrl_payload_kg", default_value="", description="제어기가 믿는 적재. 비우면 payload_kg 와 같게"),
        A("use_wheel_inertia", default_value="", description="모델 바퀴 관성 스위치 (YAML 기본 false)"),
        A("controller", default_value="mpc", choices=["mpc", "pure_pursuit"],
          description="mpc = torque_mpc_node (비교군 B·C), pure_pursuit = 퓨어 퍼슛 + 바퀴별 PI (비교군 A)"),
        A("limit_mode", default_value="", description="motor_only(B) | friction(C) (YAML 기본 motor_only)"),
        A("disturbance", default_value="", description="외란 추정치 d̂ true/false (YAML 기본 false)"),
        A("mu_hat", default_value="", description="제어기의 노면 마찰계수 (YAML 기본 0.3)"),
        A("log_prefix", default_value="", description="MPC CSV 접두. 비면 저장 안 함"),
        # 8자 트랙: 비우면 config/torque_track.yaml
        *[A(k, default_value="", description=f"트랙 {k} (비우면 torque_track.yaml)") for k in _TRACK_KEYS],
        A("max_wheel_rad_per_sec", default_value=str(rated_rad_s), description="기준 궤적 부하 검사 상한 [rad/s]. 기본은 torque_plant.yaml 정격 370 rpm; 모터 과속 차단 기능은 아님"),
        A("x0", default_value="0.0"), A("y0", default_value="0.0"), A("yaw0", default_value="0.0"),
        A("use_rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", rviz_cfg], condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])
