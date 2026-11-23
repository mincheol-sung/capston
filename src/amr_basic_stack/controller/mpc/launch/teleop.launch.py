#!/usr/bin/env python3
"""AK40 저수준 + 게임패드 텔레옵만 (2026-11-23). MPC·기준궤적·추정기·센서·Cartographer 는 띄우지 않는다.

  joy_node ─/joy─▶ joy_teleop ─/teleop/cmd_vel (TwistStamped: vx, vy, wz)─▶ wheel_pi_node ─▶ torque_output_node
      ─▶ 게이트(torque_command_gate, 무장해야 통과) ─/wheel_torque_cmd─▶ ak40_can_node ─CAN─▶ AK40 × 4
                                        ▲                                                  │
                                        └──────────────── /wheel_states (100 Hz) ◀─────────┘
  wheel_pi_node: ω_ref = T⁻¹·v (메카넘 역기구학) → 바퀴별 PI → 토크. 비교군 A 가 쓰는 노드·게인 그대로
                 (config/torque_baseline_a.yaml). speed_source = wheels (엔코더) — 추정기가 없어도 돈다.
  fault_monitor: 모터 에러·/wheel_states 끊김을 보면 무장 해제. 추정기 끊김 검사는 끈다 (check_state_timeout false).

사용 (새 터미널에서 amr 을 먼저)
  ros2 launch amr_mpc teleop.launch.py                    # Xbox 패드 (config/joy_teleop_xbox.yaml), 실차 AK40
  ros2 launch amr_mpc teleop.launch.py gamepad:=f710      # F710 (joy_teleop.yaml) — 패드 무장 버튼 없음, 키보드 'a'
  ros2 launch amr_mpc teleop.launch.py plant:=sim         # 모터 없이 시뮬 플랜트로 전 경로 확인

무장·정지
  Xbox: Menu(☰) = 무장, B = 즉시 해제(DAMPING).   이 터미널: 'a' = 무장, 's'/SPACE = 해제.
  무장 전·해제 뒤·스틱을 놓은 뒤 0.25 s 가 지나면 바퀴는 DAMPING (토크 0 이 아니라 감쇠 — 밀면 버틴다).

조작 (Xbox, joy_teleop_xbox.yaml)
  왼쪽 스틱 위/아래 = 전진/후진 (vx),  오른쪽 스틱 좌/우 = 왼쪽/오른쪽 횡이동 (vy),  LB/RB = 반시계/시계 회전 (wz)
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_mpc.ak40.setup import prepare_can

GAMEPAD_CONFIG = {"xbox": "joy_teleop_xbox.yaml", "f710": "joy_teleop.yaml"}


def _truthy(s):
    return str(s).strip().lower() in ("true", "1", "yes", "on")


def _nodes(context):
    g = lambda n: LaunchConfiguration(n).perform(context).strip()
    share = get_package_share_directory("amr_mpc")
    plant, gamepad = g("plant"), g("gamepad")
    nodes = []

    # ── 플랜트
    if plant == "real":
        if _truthy(g("armed")):
            raise RuntimeError("plant:=real 에서는 armed:=true 를 쓸 수 없다 — 띄운 뒤 /wheel_states 를 보고 무장")
        if _truthy(g("auto_setup_can")):
            with open(os.path.join(share, "config", "ak40.yaml")) as f:
                interface = g("can_interface") or str(yaml.safe_load(f)["ak40"]["interface"])
            result = prepare_can(os.path.join(share, "tools", "setup_can.sh"), interface, g("can_device"))
            nodes.append(LogInfo(msg=f"CAN setup: {result}"))
        nodes.append(Node(
            package="amr_mpc", executable="ak40_can_node", name="ak40_can_node", output="screen",
            parameters=[{"interface": g("can_interface")}] if g("can_interface") else []))
    else:                                             # sim: 시뮬 플랜트가 /wheel_torque_cmd → /wheel_states
        nodes.append(Node(
            package="amr_mpc", executable="sim_plant_torque_node", name="sim_plant_torque_node", output="screen",
            parameters=[{"sensors": False}]))

    # ── 속도 → 토크 (바퀴별 PI)
    # cmd_timeout: joy_teleop 은 20 Hz(0.05 s)로 내서 비교군 A 기본 0.05 s 로는 주기마다 끊김 판정이 날 수 있다.
    #   0.25 s = 5 주기. 스틱을 놓으면 joy_teleop 이 0 을 한 번 보내고 멈추므로, 그 0.25 s 뒤 DAMPING + PI 리셋.
    nodes.append(Node(
        package="amr_mpc", executable="wheel_pi_node", name="wheel_pi_node", output="screen",
        parameters=[{"cmd_topic": "/teleop/cmd_vel", "armed_topic": "/mpc/armed",
                     "speed_source": "wheels", "cmd_timeout": float(g("cmd_timeout"))}]))
    nodes.append(Node(
        package="amr_mpc", executable="torque_output_node", name="torque_output_node", output="screen",
        parameters=[{"input_topic": "/wheel_pi/cmd_raw", "max_wheel_rad_per_sec": float(g("max_wheel_rad_per_sec")),
                     # 정지 마찰 보상은 기준열(/dra_mppi/reference_trajectory)의 바퀴 방향을 쓴다 — 텔레옵에는 기준열이 없다
                     "stiction_comp": False, "slip_guard": False}]))
    nodes.append(Node(
        package="amr_mpc", executable="fault_monitor_node", name="fault_monitor_node", output="screen",
        parameters=[{"check_state_timeout": False, "trip_on_pose_jump": False,
                     "track_status_topic": "/teleop/track_status"}]))      # 발행자 없음 → 추종 오차 검사 안 걸림
    nodes.append(Node(
        package="amr_mpc", executable="keyboard_command_gate", name="torque_command_gate", output="screen",
        parameters=[{
            "command_type": "torques",
            "input_topic": "/torque_output/cmd_raw", "output_topic": "/wheel_torque_cmd",
            "arm_topic": "/mpc/arm", "initially_armed": _truthy(g("armed")),
            "keyboard_enabled": True, "command_timeout": 0.20, "publish_frequency": 50.0,
            "forward_on_receive": True,
        }]))

    # ── 게임패드
    joy_cfg = os.path.join(get_package_share_directory("amr_bringup"), "config", GAMEPAD_CONFIG[gamepad])
    nodes.append(Node(package="joy", executable="joy_node", name="joy_node", output="screen", parameters=[joy_cfg]))
    nodes.append(Node(
        package="amr_base", executable="joy_teleop_node", name="joy_teleop", output="screen",
        parameters=[joy_cfg, {"stamped_output": True}],
        remappings=[("cmd_vel", "/teleop/cmd_vel")]))       # /cmd_vel(Twist, RoboClaw 경로)과 섞이지 않게
    nodes.append(LogInfo(msg=f"teleop: plant={plant}, gamepad={gamepad} ({GAMEPAD_CONFIG[gamepad]}). "
                             + ("Xbox Menu = 무장, B = 해제. " if gamepad == "xbox" else "")
                             + "이 터미널 'a' = 무장, 's'/SPACE = 해제"))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("amr_mpc")
    with open(os.path.join(share, "config", "torque_plant.yaml")) as f:
        rated_rad_s = float(yaml.safe_load(f)["motor"]["rated_speed_rpm"]) * 2.0 * math.pi / 60.0
    A = DeclareLaunchArgument
    return LaunchDescription([
        A("plant", default_value="real", choices=["real", "sim"],
          description="real = ak40_can_node(can0), sim = 시뮬 플랜트 (모터 없이 경로 확인)"),
        A("gamepad", default_value="xbox", choices=list(GAMEPAD_CONFIG),
          description="xbox = amr_bringup/config/joy_teleop_xbox.yaml, f710 = joy_teleop.yaml"),
        A("armed", default_value="false", choices=["true", "false"], description="띄우자마자 무장 (plant:=sim 에서만)"),
        A("cmd_timeout", default_value="0.25", description="wheel_pi: 텔레옵 명령이 이만큼 끊기면 DAMPING [s]"),
        A("max_wheel_rad_per_sec", default_value=str(rated_rad_s),
          description="torque_output 마찰 여유 경고용 바퀴 속도 [rad/s] — 속도를 자르지는 않는다. 기본 torque_plant.yaml 정격 370 rpm"),
        A("auto_setup_can", default_value="true", choices=["true", "false"],
          description="plant:=real 에서 모터 시작 전 CAN 자동 준비 (torque_experiment 와 같다)"),
        A("can_interface", default_value="", description="비우면 config/ak40.yaml (can0)"),
        A("can_device", default_value="", description="slcan USB 장치 경로. 비우면 CANable by-id 자동 선택"),
        OpaqueFunction(function=_nodes),
    ])
