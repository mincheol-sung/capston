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
  ros2 launch amr_mpc torque_experiment.launch.py stiction_comp:=false  # torque_output 정지 마찰 보상 끔. 기본 auto = plant:=real 에서만 켬
                                                                        #   (torque_plant.yaml friction.static_Nm, 기준 바퀴속도 방향. 2026-11-16)
  ros2 launch amr_mpc torque_experiment.launch.py linearization:=current_state   # 선형화를 현재 추정 상태 기준으로.
                                                                        #   기본 reference = 1판 방식(기준궤적)
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amr_mpc.ak40.setup import prepare_can
from amr_mpc.core import surfaces


def _truthy(s):
    return str(s).strip().lower() in ("true", "1", "yes", "on")


def _load_share(text):
    """wheel_load_share:="0.2388,0.2392,0.2608,0.2612" → [FL, FR, RL, RR]. 개수·범위가 틀리면 시작 전에 멈춘다 (2026-11-19).
    검사는 model.load_share 와 같다 (값 0.1~0.4, 합 1 ± 0.05). 넘기는 값은 적은 그대로 — 노드가 합 1 로 맞춘다."""
    from amr_mpc.torque.model import load_share
    try:
        vals = [float(v) for v in str(text).strip("[] ").split(",")]
        load_share(vals)
    except ValueError as e:
        raise RuntimeError(f"wheel_load_share 는 4 개 \"FL,FR,RL,RR\", 값 0.1~0.4, 합 1 ± 0.05 — {text!r} ({e})")
    return vals


def _mu_wheels(text):
    """mu_hat_wheels:="0.432,0.285,0.456,0.350" → [FL, FR, RL, RR], "off" → [−1]*4 (네 바퀴 모두 mu_hat × 하중 몫).
    4 개가 아니거나 2 를 넘으면 시작 전에 멈춘다. 0 이하는 그 바퀴만 mu_hat × 하중 몫 (2026-11-19 (11))."""
    if str(text).strip().lower() in ("off", "false", "none"):
        return [-1.0] * 4
    try:
        vals = [float(v) for v in str(text).strip("[] ").split(",")]
    except ValueError:
        raise RuntimeError(f"mu_hat_wheels 는 숫자 4 개 \"FL,FR,RL,RR\" 또는 off — {text!r}")
    if len(vals) != 4 or any(v > 2.0 for v in vals):
        raise RuntimeError(f"mu_hat_wheels 는 4 개 [FL, FR, RL, RR], 값 ≤ 2 (0 이하 = 그 바퀴 mu_hat) 또는 off — {text!r}")
    return vals


def _budget(text):
    """load_budget:=0.9 → 0.9. 기준 바퀴 부하 |vx|+|vy|+k|ω| 의 허용 비율 (× V_LIMIT = r × max_wheel_rad_per_sec).
    0 < 값 ≤ 1 이 아니면 시작 전에 멈춘다 — 1 을 넘으면 기준 자체가 정격 바퀴 속도를 넘는다 (2026-11-19 (12))."""
    try:
        v = float(text)
    except ValueError:
        raise RuntimeError(f"load_budget 은 0 < 값 ≤ 1 — {text!r}")
    if not 0.0 < v <= 1.0:
        raise RuntimeError(f"load_budget 은 0 < 값 ≤ 1 (1 이면 보정 여유 0) — {text!r}")
    return v


# config/torque_track.yaml 의 키. launch 인자 이름이 같고, 비워 두면 YAML 값을 쓴다
_TRACK_KEYS = ["origin_mode", "origin_x", "origin_y", "origin_yaw_deg", "start_tolerance_xy",
               "start_tolerance_yaw_deg", "lead_speed", "lead_yaw_rate", "width", "height", "yaw_mode", "crossing_blend_m",
               "spin_rate",                                  # 2026-11-19 yaw_mode:=spin 의 순항 요레이트 [rad/s]
               "speed", "accel", "laps", "align_s", "start_delay", "abort_error", "hold_s", "lap_speeds",
               "dt", "horizon_steps",
               # 5-3 단일 축 기준 (2026-11-16): shape:=line|rotate 와 seg_* (figure8_reference.py)
               "shape", "seg_distance", "seg_dir_deg", "seg_angle_deg", "seg_yaw_rate", "seg_yaw_accel",
               "seg_out_and_back", "seg_pause_s",
               # 정사각형 (2026-11-20): shape:=square 와 square_* (core/square.py)
               "square_side", "square_corner_speed", "square_v_max", "square_yaw_blend"]


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
        elif k == "seg_out_and_back":
            out[k] = str(track[k]).strip().lower() in ("1", "true", "yes", "on")
        else:
            out[k] = str(track[k]) if k in ("origin_mode", "yaw_mode", "shape") else float(track[k])
    # 문자열 인자는 여기서 허용값을 검사한다 (2026-11-16): yaw_mode:=true 로 띄운 판이 6 개 있었다 — figure8_reference_node 가
    # 시작하자마자 ValueError 로 죽는데 오류가 화면에만 나오고 /rosout·bag 에는 안 남아, 무장해도 기준 없이 그냥 서 있었다.
    for k, allowed in _TRACK_CHOICES.items():
        if out[k] not in allowed:
            raise RuntimeError(f"{k}:={out[k]!r} — 허용값은 {' | '.join(allowed)} (torque_track.yaml 또는 launch 인자)")
    if out["shape"] == "square" and out["yaw_mode"] == "spin":        # 2026-11-20: 노드 기동 오류는 bag 에 안 남는다 (위 주석)
        raise RuntimeError("shape:=square 는 yaw_mode:=tangent | fixed (spin 은 figure8·circles8 만)")
    return out


_TRACK_CHOICES = {"yaw_mode": ("tangent", "fixed", "spin"), "shape": ("figure8", "circles8", "line", "rotate", "square"),
                  "origin_mode": ("robot", "fixed")}


def _nodes(context):
    g = lambda n: LaunchConfiguration(n).perform(context).strip()
    share = get_package_share_directory("amr_mpc")
    plant = g("plant")
    source, slam_mode = g("state_source"), g("slam_mode")
    state_topic = g("state_topic") or ("/state_estimator/odom" if source == "estimator" else "/sim/ground_truth")
    nodes = []
    use_ref = _truthy(g("use_figure8")) and g("controller") not in ("ramp", "step")   # 램프·스텝은 기준궤적 없이 (2026-11-17)
    if _truthy(g("use_mpc")) and use_ref:
        _track(context, share)                        # 잘못된 yaw_mode·shape·origin_mode 면 CAN 준비·노드보다 먼저 멈춘다
    if g("wheel_load_share"):
        _load_share(g("wheel_load_share"))              # 잘못된 바퀴 하중 몫도 CAN 준비 전에 (2026-11-19)
    if g("mu_hat_wheels"):
        _mu_wheels(g("mu_hat_wheels"))                  # 잘못된 바퀴별 μ̂ 도 CAN 준비 전에 (2026-11-19 (11))
    if g("load_budget"):
        _budget(g("load_budget"))                       # 잘못된 부하 예산도 CAN 준비 전에 (2026-11-19 (12))
    # Run before returning any motor/controller actions. Never runs for simulation.
    if plant == "real":
        if source != "estimator":
            raise RuntimeError("plant:=real requires state_source:=estimator")
        # 실차는 띄우자마자 무장하지 않는다 (R10 리뷰 2026-11-14). 센서·추정기 준비 전이라 fault_monitor 가 곧 세우겠지만,
        # 그 사이 모터가 토크를 받을 수 있다. 무장은 "준비 끝" 로그와 /wheel_states error 0 을 보고 키보드 'a' 로.
        if _truthy(g("armed")):
            raise RuntimeError("plant:=real 에서는 armed:=true 를 쓸 수 없다 — 띄운 뒤 준비를 보고 'a' 로 무장")
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

    # 시뮬 플랜트의 바퀴 관성 (물리). 비어 있으면 YAML 값. 2026-11-16 부터 제어기에는 안 간다 —
    # 제어기는 M 에 m_w 를 늘 넣고 w 항만 use_w 로 켜고 끈다 (w 항·외란 추정치 지시서)
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
                         "slam_jump": g("sim_slam_jump"),                       # 2026-11-19 포즈 튐 흉내 (비우면 끔)
                         "surface_mu": float(g("surface_mu")) if g("surface_mu") else -1.0,
                         "friction_scale": float(g("friction_scale")),
                         "wheel_efficiency": [float(v) for v in g("wheel_eff").split(",")] if g("wheel_eff") else [1.0, 1.0, 1.0, 1.0],
                         "floor_efficiency": [float(v) for v in g("floor_eff").split(",")] if g("floor_eff") else [1.0, 1.0, 1.0],
                         "can_interface": can_if if plant == "sim_can" else "",
                         "can_reply_delay": float(g("can_reply_delay")),
                         # 2026-11-19 바퀴별 수직력 몫 (슬립 모델 N_i). 비우면 노드가 torque_plant.yaml body.wheel_load_share
                         **({"wheel_load_share": _load_share(g("wheel_load_share"))} if g("wheel_load_share") else {})}]))
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
        # 로봇 모델 TF (robot_state_publisher). imu_to_base_node 는 base_footprint ← camera_imu_optical_frame TF 가 있어야
        # IMU 를 내고, Cartographer 도 라이다·카메라 위치를 TF 로 안다. sensors.launch.py 는 이것을 띄우지 않는다
        # (bringup.launch.py 가 띄우지만 그쪽은 RoboClaw 구동 노드까지 띄운다 — AK40 실차에서는 쓰지 않는다). R10 리뷰 2026-11-14
        if _truthy(g("robot_state")):
            try:
                desc = get_package_share_directory("amr_description")
            except Exception as e:
                raise RuntimeError(f"amr_description 패키지를 찾을 수 없다 ({e}) — NUC 에서 빌드·source 했는지 확인. "
                                   f"robot_state_publisher 를 따로 띄웠다면 robot_state:=false") from None
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(desc, "launch", "robot_state.launch.py"))))
        # IMU: D455 원본(/camera/imu, optical 프레임) → 몸체 프레임 /imu/base_link (localization.launch.py 와 같은 노드)
        nodes.append(Node(
            package="amr_perception_bringup", executable="imu_to_base_node", name="imu_to_base_node", output="screen",
            # bias_calib_s 0: 회전만 한다. 자이로 바이어스는 state_estimator 가 기동 정지 구간에서 잡는다.
            #   이 노드도 바이어스를 빼면 (기본 3 s) 추정기가 1 s 만에 잡은 바이어스가 3 s 째에 한 번 더 빠져
            #   자이로가 그만큼 계단처럼 어긋난다. 시뮬에는 이 노드가 없어 드러나지 않는다 (실차 점검 2026-11-13).
            parameters=[{"target_frame": "base_footprint", "bias_calib_s": 0.0}],
            remappings=[("imu_in", "/camera/imu"), ("imu_out", "/imu/base_link")]))
        nodes.append(LogInfo(msg=(
            "plant:=real — 이 launch 는 모터(ak40_can_node), IMU 회전 노드, "
            f"{'robot_state_publisher(로봇 모델 TF)' if _truthy(g('robot_state')) else '(robot_state:=false — TF 는 따로)'} 를 띄운다. "
            "다른 터미널에서 먼저:\n"
            "  ① 센서: ros2 launch amr_bringup sensors.launch.py   (라이다·D455 IMU)\n"
            f"  ② 측위: ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=<지도 이름> lua_basename:={carto_lua}\n"
            "     (저장된 지도 pbstream 위에서, 외삽 끔 → /tracked_pose 스캔 시각 stamp)\n"
            "  ✗ 같이 띄우지 말 것: bringup.launch.py (RoboClaw 구동 노드), localization.launch.py (IMU 회전 노드가 하나 더 떠 IMU 가 두 배로 온다)\n"
            "  NUC 에서는 use_rviz:=false 권장 (화면 없음·CPU 를 MPC 와 나눠 쓴다)\n"
            "  실차 확인 순서: amr/notes_amr/real_robot_checklist.md")))
    else:
        raise RuntimeError(f"plant:={plant!r} — sim, sim_can 또는 real")

    # ── 상태 추정기 (SLAM 포즈 + IMU → 100 Hz 상태)
    if source == "estimator":
        nodes.append(Node(
            package="amr_mpc", executable="state_estimator_node", name="state_estimator_node", output="screen",
            parameters=[{"pose_source": "stamped" if slam_mode == "scan_time" else "after_scan_match",
                         # 2026-11-19: 게인 설계의 포즈 주기. 비우면 노드 기본 0.1 s (스캔 두 개 누적, 10 Hz)
                         **({"pose_period": float(g("est_pose_period"))} if g("est_pose_period") else {}),
                         # 2026-11-19: 추정기 모드 kf | slam. 비우면 torque_estimator.yaml estimator.mode (기본 kf)
                         **({"est_mode": g("est_mode")} if g("est_mode") else {})}]))

    if not _truthy(g("use_mpc")):
        return nodes

    # ── 기준궤적: 8자 시간 기준 (기동 포즈가 교차점)
    if use_ref:
        nodes.append(Node(
            package="amr_mpc", executable="figure8_reference_node", name="figure8_reference_node", output="screen",
            parameters=[{
                **_track(context, share),
                "frame_id": "map", "odom_topic": state_topic,
                "wait_for_arm": True, "armed_topic": "/mpc/armed",   # 무장된 동안만 기준 시간이 흐른다
                # 기준 부하 검사: 정격 속도 370 rpm ≈ 38.746 rad/s를 기본 운용 상한으로 사용.
                "max_wheel_rad_per_sec": float(g("max_wheel_rad_per_sec")),
                # 2026-11-19 (12) 부하 예산. 비우면 노드 기본 (plant.yaml limits.load_budget — 2026-11-20 0.65 → 1.0)
                **({"load_budget": _budget(g("load_budget"))} if g("load_budget") else {}),
            }]))

    # ── 노면 μ̂ (2026-11-17): mu_hat:=값 이 있으면 그 값, 없고 surface:=이름 이면 config/surfaces.yaml 의 값,
    #    둘 다 없으면 비워 둬서 torque_plant.yaml surface.mu_hat. 표에 없거나 미측정이면 여기서 멈춘다(조용히 기본값으로 안 감)
    mu_arg = g("mu_hat")
    if not mu_arg and g("surface"):
        mu_arg = str(surfaces.lookup(g("surface")))
        nodes.append(LogInfo(msg=f"surface:={g('surface')} → μ̂ {mu_arg} (config/surfaces.yaml)"))

    # ── 제어기. controller:=mpc (비교군 B·C, 기본) | pure_pursuit (비교군 A, 6단계)
    #    둘 다 같은 기준열·같은 상태를 받고, 바퀴 토크를 같은 torque_output → 게이트 → 플랜트 경로로 낸다.
    controller = g("controller")
    if controller == "mpc":
        # payload_kg 는 "제어기가 믿는" 적재 (비우면 플랜트와 같게)
        ctrl = {"state_topic": state_topic, "armed_topic": "/mpc/armed",
                "payload_kg": float(g("ctrl_payload_kg")) if g("ctrl_payload_kg") else payload,
                "log_prefix": g("log_prefix")}
        if g("use_w"):                                # w 항 (식-6). 비우면 torque_mpc.yaml model_terms.use_w (기본 false)
            ctrl["use_w"] = _truthy(g("use_w"))
        if g("limit_mode"):
            ctrl["limit_mode"] = g("limit_mode")
        if g("disturbance"):
            ctrl["disturbance_enable"] = _truthy(g("disturbance"))
        if g("linearization"):
            ctrl["linearization"] = g("linearization")
        if mu_arg:                                   # mu_hat:= 또는 surface:= (config/surfaces.yaml)
            ctrl["mu_hat"] = float(mu_arg)
        # 2026-11-19 (11) 바퀴별 μ̂ (mg/4 기준). 비우면 torque_plant.yaml surface.mu_hat_wheels, off 면 끈다
        if g("mu_hat_wheels"):
            ctrl["mu_hat_wheels"] = _mu_wheels(g("mu_hat_wheels"))
        with open(os.path.join(share, "config", "torque_plant.yaml")) as f:
            yaml_muw = (yaml.safe_load(f).get("surface") or {}).get("mu_hat_wheels") or []
        muw = ctrl.get("mu_hat_wheels", yaml_muw)
        if muw and all(v > 0 for v in muw) and g("mu_hat") and (g("limit_mode") or "") == "friction":
            nodes.append(LogInfo(msg=f"mu_hat:={g('mu_hat')} 는 MPC 상한에 안 쓰인다 — 바퀴별 μ̂ {list(muw)} 가 있다 "
                                     "(torque_plant.yaml surface.mu_hat_wheels). mu_hat 로 돌리려면 mu_hat_wheels:=off"))
        if g("wheel_load_share"):                     # 2026-11-19 바퀴별 수직력 몫 — limit_mode friction 상한에만 쓰인다
            ctrl["wheel_load_share"] = _load_share(g("wheel_load_share"))
            if (g("limit_mode") or "") != "friction":
                nodes.append(LogInfo(msg=f"wheel_load_share 를 줬지만 limit_mode:={g('limit_mode') or '(YAML 기본)'} — "
                                         "friction 이 아니면 MPC 상한에는 안 쓰인다 (시뮬 슬립 모델에는 쓰인다)"))
        # 바퀴별 마찰 상한 배율 (2026-11-18): wheel_limit_scale:="0.85,1,1.1,1" [FL, FR, RL, RR]. 비우면 torque_plant.yaml
        if g("wheel_limit_scale"):
            ws = [float(v) for v in g("wheel_limit_scale").strip("[] ").split(",")]
            if len(ws) != 4 or not all(0.0 < v <= 1.5 for v in ws):
                raise RuntimeError(f"wheel_limit_scale 는 4 개, 각 0 < s ≤ 1.5 [FL, FR, RL, RR] — {g('wheel_limit_scale')!r}")
            ctrl["wheel_limit_scale"] = ws
        # 튜닝 덮어쓰기 (2026-11-16, 5-3 감쇠 시험): mpc_W:="100,100,50,1,1,0.5"  mpc_w_delta:=10
        if g("mpc_W"):
            W = [float(v) for v in g("mpc_W").strip("[] ").split(",")]
            if len(W) != 6:
                raise RuntimeError(f"mpc_W 는 6 개 [x, y, θ, ẋ, ẏ, θ̇] — {g('mpc_W')!r}")
            ctrl["W"] = W
        if g("mpc_w_delta"):                          # 스텝 단위 r_Δ [(N·m)⁻²] — 주면 w_dudt 보다 우선 (torque_mpc_node)
            ctrl["w_delta"] = float(g("mpc_w_delta"))
        if g("mpc_w_dudt"):                           # 2026-11-18 교수님 지시: 시간 미분 단위 r_τ̇ [(N·m/s)⁻²] = r_Δ·Ts²
            ctrl["w_dudt"] = float(g("mpc_w_dudt"))
        if g("mpc_r_u"):
            ctrl["r_u"] = float(g("mpc_r_u"))
        if g("mpc_q_null"):
            ctrl["q_null"] = float(g("mpc_q_null"))
        if g("mpc_dist_alpha"):
            ctrl["disturbance_alpha"] = float(g("mpc_dist_alpha"))
        if g("velocity_source"):                      # 2026-11-18: estimator | wheels (torque_mpc.yaml state_input)
            ctrl["velocity_source"] = g("velocity_source")
        if g("mpc_dist_accel_only"):                  # 2026-11-18 시험용 (torque_mpc.yaml disturbance.accel_only)
            ctrl["disturbance_accel_only"] = _truthy(g("mpc_dist_accel_only"))
        nodes.append(Node(
            package="amr_mpc_cpp", executable="torque_mpc_node", name="torque_mpc_node", output="screen",
            parameters=[ctrl]))
        torque_in, track_status = "/torque_mpc/cmd_raw", "/torque_mpc/status"
    elif controller == "pure_pursuit":
        # 비교군 A: 퓨어 퍼슛 (v_x, ω_z) → 바퀴별 PI → 토크. 게인은 config/torque_baseline_a.yaml
        ignored = [k for k in ("limit_mode", "disturbance", "linearization", "mu_hat", "mu_hat_wheels", "wheel_limit_scale", "ctrl_payload_kg",
                               "log_prefix")
                   if g(k)]
        if ignored:
            nodes.append(LogInfo(msg=f"controller:=pure_pursuit — MPC 인자 {ignored} 는 쓰지 않는다 "
                                     f"(log_prefix 는 slip_logger 에만 쓴다)"))
        nodes.append(Node(
            package="amr_mpc", executable="pure_pursuit_node", name="pure_pursuit_node", output="screen",
            parameters=[{"state_topic": state_topic, "armed_topic": "/mpc/armed"}]))
        # 2026-11-19: pp_speed_source:=estimator 면 PI 되먹임이 추정기 속도 (MPC 와 같은 정보). pp_kp·pp_ki 로 게인 덮어쓰기
        pi = {"armed_topic": "/mpc/armed", "speed_topic": state_topic}
        if g("pp_speed_source"):
            pi["speed_source"] = g("pp_speed_source")
        for arg, par in (("pp_kp", "kp"), ("pp_ki", "ki")):
            if g(arg):
                pi[par] = float(g(arg))
        nodes.append(Node(
            package="amr_mpc", executable="wheel_pi_node", name="wheel_pi_node", output="screen",
            parameters=[pi]))
        torque_in, track_status = "/wheel_pi/cmd_raw", "/pure_pursuit/status"
    elif controller == "ramp":
        # 바닥 토크 램프 시험 (2026-11-17, torque_ramp.py): MPC 자리에서 τ* = τ_g + 관성 ff 를 /torque_mpc/cmd_raw 로.
        # 뒤(torque_output 마찰 보상·게이트·fault_monitor)는 MPC 와 같다
        ramp = {"state_topic": state_topic, "direction": g("ramp_dir") or "fwd"}
        for arg, par in (("ramp_tau_max", "tau_max"), ("ramp_rate", "rate"), ("ramp_distance", "distance"),
                         ("ramp_stop_distance", "stop_distance")):
            if g(arg):
                ramp[par] = float(g(arg))
        if mu_arg:
            ramp["mu_hat"] = float(mu_arg)
        if g("ramp_inertia_ff"):
            ramp["inertia_ff"] = _truthy(g("ramp_inertia_ff"))
        if g("ramp_log_dir"):
            ramp["log_dir"] = g("ramp_log_dir")
        nodes.append(Node(
            package="amr_mpc", executable="torque_ramp_node", name="torque_ramp_node", output="screen",
            parameters=[ramp]))
        torque_in, track_status = "/torque_mpc/cmd_raw", "/torque_mpc/status"
    elif controller == "step":
        # 바닥 토크 스텝 시험 (2026-11-17, torque_step.py): τ* = (I + K)·tau·pattern 일정, distance 가면 정지
        step = {"state_topic": state_topic, "direction": g("step_dir") or "fwd"}
        for arg, par in (("step_tau", "tau"), ("step_distance", "distance"), ("step_max_time", "max_time")):
            if g(arg):
                step[par] = float(g(arg))
        if g("step_inertia_ff"):
            step["inertia_ff"] = _truthy(g("step_inertia_ff"))
        if g("step_log_dir"):
            step["log_dir"] = g("step_log_dir")
        nodes.append(Node(
            package="amr_mpc", executable="torque_step_node", name="torque_step_node", output="screen",
            parameters=[step]))
        torque_in, track_status = "/torque_mpc/cmd_raw", "/torque_mpc/status"
    else:
        raise RuntimeError(f"controller:={controller!r} — mpc, pure_pursuit, ramp 또는 step")

    # 정지 마찰 보상 (2026-11-16): auto = 실차에서만. 시뮬 플랜트에는 정지 마찰이 없어 켜면 없는 마찰을 보상하게 된다
    stiction = g("stiction_comp")
    stiction_on = plant == "real" if stiction == "auto" else _truthy(stiction)
    nodes.append(Node(
        package="amr_mpc", executable="torque_output_node", name="torque_output_node", output="screen",
        parameters=[{"input_topic": torque_in, "max_wheel_rad_per_sec": float(g("max_wheel_rad_per_sec")),
                     "stiction_comp": stiction_on}]))

    # 위험 감시: 무장 중 포즈 튐·추종 오차·모터 에러·센서 끊김이면 /mpc/arm false → DAMPING
    if source == "estimator":                         # 참값 모드에는 추정기 출력이 없어 끊김으로 오판한다
        nodes.append(Node(
            package="amr_mpc", executable="fault_monitor_node", name="fault_monitor_node", output="screen",
            parameters=[{"max_track_error": float(g("max_track_error")), "track_status_topic": track_status,
                         # 2026-11-19: 포즈 튐 정지 스위치 (est_mode:=slam 과 같이 시뮬로 정하려고). 비우면 노드 기본 true
                         **({"trip_on_pose_jump": _truthy(g("trip_on_pose_jump"))} if g("trip_on_pose_jump") else {})}]))

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
        A("robot_state", default_value="true", choices=["true", "false"],
          description="plant:=real 에서 robot_state_publisher(로봇 모델 TF)를 같이 띄운다. 따로 띄웠으면 false"),
        A("can_reply_delay", default_value="0.0",
          description="plant:=sim_can 가짜 모터 응답 지연 [s]. ak40 reply_wait(0.004)보다 크게 주면 늘 늦는 실차 버스를 흉내 낸다"),
        A("can_device", default_value="", description="slcan USB 장치 경로. 비우면 CANable by-id 자동 선택"),
        A("state_source", default_value="estimator", choices=["estimator", "truth"],
          description="estimator = state_estimator_node (SLAM 포즈 + IMU), truth = 시뮬 참값을 바로"),
        A("slam_mode", default_value="scan_time", choices=["scan_time", "extrapolated"],
          description="시뮬 SLAM 꼴. scan_time = 스캔 시각 포즈를 늦게 (외삽 끔), extrapolated = 100 Hz 외삽 스트림 (지금 실차 설정)"),
        A("sensor_noise_scale", default_value="1.0", description="시뮬 센서 잡음 배율 (1 = bag 실측 수준, 0 = 없음)"),
        A("state_topic", default_value="", description="상태 토픽을 직접 줄 때만 (비우면 state_source 에 따라 정한다)"),
        A("max_track_error", default_value="1.5",
          description="fault_monitor: 기준과 이만큼 벌어지면 정지 [m]. 2026-11-16 0.30 → 1.5 (figure8 abort_error 1.6 보다 작게)"),
        A("surface_mu", default_value="", description="시뮬 바닥 마찰계수 (비우면 슬립 없음). 제어기의 mu_hat 과 따로 준다"),
        A("friction_scale", default_value="1.0",
          description="시뮬 구동계 마찰 = torque_plant.yaml friction × 이 값 (0 = 마찰 없음, 1.3 = 보상보다 30 % 큼)"),
        A("wheel_eff", default_value="", description="시뮬 바퀴별 효율 'FL,FR,RL,RR' (비우면 1). 접지력이 약한 바퀴 흉내 (2026-11-16)"),
        A("floor_eff", default_value="",
          description="시뮬 바닥 효율 'η_x,η_y,η_ψ' (비우면 1,1,1). 실차 5-2 맞춤값 0.65,0.40,0.40 (0.15 N·m 가속, 2026-11-16)"),
        A("wheel_odom", default_value="", description="휠 오돔 true/false (비우면 torque_estimator.yaml slam.use_wheel_odom, 기본 false)"),
        A("stiction_comp", default_value="auto", choices=["auto", "true", "false"],
          description="torque_output 정지 마찰 보상 (torque_plant.yaml friction.static_Nm). auto = plant:=real 에서만 켬"),
        A("use_mpc", default_value="true", description="false 면 플랜트(+RViz)만 띄운다"),
        A("use_figure8", default_value="true", description="8자 기준궤적 노드. false 면 기준열을 밖에서 준다"),
        A("armed", default_value="false", description="게이트 초기 상태. false 면 터미널에서 'a' 로 무장. plant:=real 에서는 true 금지"),
        # 모델·비교군. 비우면 config/torque_plant.yaml, torque_mpc.yaml 값
        A("payload_kg", default_value="0.0", description="적재 [kg]. 2차 발표 조건 0 / 5"),
        A("ctrl_payload_kg", default_value="", description="제어기가 믿는 적재. 비우면 payload_kg 와 같게"),
        A("use_wheel_inertia", default_value="",
          description="시뮬 플랜트의 바퀴 관성 (torque_plant.yaml model.use_wheel_inertia). 2026-11-16 부터 플랜트만 — "
                      "제어기는 M 에 m_w 를 늘 넣는다"),
        A("use_w", default_value="",
          description="제어기 w 항 (식-6) true/false. 비우면 torque_mpc.yaml model_terms.use_w (기본 false, 2026-11-16 지시서)"),
        A("controller", default_value="mpc", choices=["mpc", "pure_pursuit", "ramp", "step"],
          description="mpc = torque_mpc_node (비교군 B·C), pure_pursuit = 퓨어 퍼슛 + 바퀴별 PI (비교군 A), "
                      "ramp = 바닥 토크 램프 시험 torque_ramp_node, step = 스텝 시험 torque_step_node (8자 기준 없음)"),
        A("ramp_dir", default_value="", description="ramp: fwd (직진) | left (왼쪽 90°). 비우면 fwd"),
        A("ramp_tau_max", default_value="", description="ramp: 바퀴–바닥 토크 끝값 [N·m] (기본 1.0)"),
        A("ramp_rate", default_value="", description="ramp: 기울기 [N·m/s]. 비우면 ramp_distance·mu_hat 로 자동"),
        A("ramp_distance", default_value="", description="ramp: 자동 기울기의 목표 이동 거리 [m] (기본 3.0)"),
        A("ramp_stop_distance", default_value="", description="ramp: 이만큼 가면 끊음 [m] (기본 1.15 × ramp_distance)"),
        A("ramp_inertia_ff", default_value="", description="ramp: 바퀴 관성 feedforward true/false (기본 true)"),
        A("step_dir", default_value="", description="step: fwd | left (비우면 fwd)"),
        A("step_tau", default_value="", description="step: 바퀴–바닥 토크 τ_g [N·m] (꼭 준다)"),
        A("step_distance", default_value="", description="step: 끊는 이동 거리 [m] (꼭 준다)"),
        A("step_max_time", default_value="", description="step: 최대 시간 [s] (기본 5)"),
        A("step_inertia_ff", default_value="", description="step: 바퀴 관성 feedforward true/false (기본 true)"),
        A("step_log_dir", default_value="", description="step: 판마다 CSV 폴더 (비우면 안 남김)"),
        A("ramp_log_dir", default_value="", description="ramp: 판마다 CSV 폴더 (비우면 안 남김)"),
        A("limit_mode", default_value="", description="motor_only(B) | friction(C) (YAML 기본 motor_only)"),
        A("disturbance", default_value="", description="외란 추정치 d̂ true/false (YAML 기본 false)"),
        A("linearization", default_value="", description="reference | current_state (YAML 기본 reference)"),
        A("mpc_W", default_value="", description="MPC 상태 가중 'x,y,θ,ẋ,ẏ,θ̇' (비우면 torque_mpc.yaml tuning.W)"),
        A("mpc_w_delta", default_value="", description="MPC 토크 변화 벌점 r_Δ 스텝 단위 [(N·m)⁻²] (주면 w_dudt 보다 우선)"),
        A("mpc_w_dudt", default_value="", description="MPC 토크 변화율 벌점 r_τ̇ [(N·m/s)⁻²] = r_Δ·Ts² (비우면 torque_mpc.yaml tuning.w_dudt)"),
        A("mpc_r_u", default_value="", description="MPC 입력 편차 벌점 r_U (비우면 torque_mpc.yaml tuning.r_u)"),
        A("mpc_dist_alpha", default_value="", description="외란 추정치 저역통과 α (비우면 torque_mpc.yaml disturbance.alpha)"),
        A("pp_speed_source", default_value="", description="비교군 A 바퀴 PI 속도 되먹임 wheels(엔코더, 기본) | estimator(추정기, MPC 와 같은 정보)"),
        A("pp_kp", default_value="", description="비교군 A 바퀴 PI K_p (비우면 torque_baseline_a.yaml)"),
        A("pp_ki", default_value="", description="비교군 A 바퀴 PI K_i (비우면 torque_baseline_a.yaml)"),
        A("est_pose_period", default_value="", description="추정기 게인 설계의 SLAM 포즈 주기 [s] (비우면 0.1. 스캔 하나씩 매칭하는 lua 면 0.05)"),
        A("est_mode", default_value="", description="추정기 모드 kf (고정 게인 칼만) | slam (SLAM 으로 덮어쓰기, 원래 설계). 비우면 torque_estimator.yaml (기본 kf)"),
        A("trip_on_pose_jump", default_value="", description="fault_monitor 가 무장 중 SLAM 포즈 튐에 정지 true/false (비우면 true)"),
        A("sim_slam_jump", default_value="", description="시뮬 포즈 튐 흉내 \"t,dx,dy[,dyaw_deg[,once]]\": 출발 t 초 뒤부터 SLAM 포즈를 옮긴다 (once 1 = 스캔 하나만)"),
        A("velocity_source", default_value="", description="MPC 속도 상태 estimator | wheels (바퀴 기구학 v_x·v_y). 비우면 torque_mpc.yaml state_input"),
        A("mpc_dist_accel_only", default_value="", description="외란 추정치 위치 잔차를 ½Ts·속도 잔차로 true/false (비우면 torque_mpc.yaml disturbance.accel_only)"),
        A("mpc_q_null", default_value="", description="MPC 영공간 벌점 (비우면 torque_mpc.yaml tuning.q_null)"),
        A("mu_hat", default_value="", description="제어기의 노면 마찰계수 (YAML 기본 0.3). surface:= 보다 우선"),
        A("mu_hat_wheels", default_value="",
          description="바퀴별 μ̂ \"FL,FR,RL,RR\" (mg/4 기준, MPC friction 상한) 또는 off. 비우면 torque_plant.yaml surface.mu_hat_wheels"),
        A("wheel_load_share", default_value="",
          description="바퀴별 수직력 몫 \"FL,FR,RL,RR\" (합 1). 비우면 torque_plant.yaml body.wheel_load_share. MPC friction 상한·시뮬 슬립 모델"),
        A("surface", default_value="", description="노면 이름 → config/surfaces.yaml 의 μ̂ (mu_hat 이 비었을 때). 미측정이면 실행 거부"),
        A("wheel_limit_scale", default_value="",
          description="바퀴별 마찰 상한 배율 FL,FR,RL,RR (예 0.85,1,1.1,1). C 에서만. 비우면 torque_plant.yaml surface.wheel_limit_scale"),
        A("log_prefix", default_value="", description="MPC CSV 접두. 비면 저장 안 함"),
        # 8자 트랙: 비우면 config/torque_track.yaml
        *[A(k, default_value="", description=f"트랙 {k} (비우면 torque_track.yaml)") for k in _TRACK_KEYS],
        A("load_budget", default_value="",
          description="기준 바퀴 부하 예산 (× r·max_wheel_rad_per_sec). 비우면 plant.yaml 1.0 (정격까지, 2026-11-20). circles8 tangent 최고 W2.0 1.11 · W3.0 1.23 m/s"),
        A("max_wheel_rad_per_sec", default_value=str(rated_rad_s), description="기준 궤적 부하 검사 상한 [rad/s]. 기본은 torque_plant.yaml 정격 370 rpm; 모터 과속 차단 기능은 아님"),
        A("x0", default_value="0.0"), A("y0", default_value="0.0"), A("yaw0", default_value="0.0"),
        A("use_rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", rviz_cfg], condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])
