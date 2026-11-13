#!/usr/bin/env python3
"""전체 시스템 통합 launch — 실제 로봇 스택 + 인지/측위.

amr_jazzy_basic_stack(로봇) + amr_ws(인지·측위)를 한 번에 실행한다.
두 워크스페이스를 모두 source 한 상태에서 실행할 것.

구성:
  robot_state (URDF→TF)            [amr_description]
  base (roboclaw, /odom)           [amr_bringup]  publish_tf=false (EKF가 TF 소유)
  sensors (라이다2+병합+realsense)  [amr_bringup]  align_depth on
  localization (EKF)               [amr_perception_bringup]  odom→base_link TF
  perception (sensor_fusion 등)    [amr_perception_bringup]

TF 소유권: URDF(base_link→센서) · EKF(odom→base_link) · SLAM(map→odom, 추후).
roboclaw publish_tf=false 로 EKF와의 odom→base_link TF 충돌 방지.

사전 준비(새 NUC/재설치 시 1회):
  scripts/setup_nuc_system.sh 실행  → 커널 rmem_max + USB autosuspend 설정
  (없으면 CycloneDDS가 큰 이미지를 못 날라 /detections 안 나옴. CYCLONEDDS_URI 는 이 launch가 자동 지정.)

실행:
  ros2 launch amr_perception_bringup full_system.launch.py            # mock(하드웨어 없이)
  ros2 launch amr_perception_bringup full_system.launch.py mock_mode:=false
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, args=None):
    src = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(pkg), "launch", rel]))
    return IncludeLaunchDescription(src, launch_arguments=args.items() if args else None)


def generate_launch_description():
    mock_mode = LaunchConfiguration("mock_mode")

    # CycloneDDS: 큰 이미지(RGB 1280x720≈2.7MB) 전달용 소켓버퍼 확대.
    # 2026-07-27 /detections 안 나오던 근본원인 → config/cyclonedds.xml 로 해결.
    # 여기서 CYCLONEDDS_URI 를 설치된 설정으로 자동 지정(모든 하위 노드가 상속).
    # ※ 커널 net.core.rmem_max 상향은 launch 불가(root) → scripts/setup_nuc_system.sh 로 1회 설정.
    set_cyclonedds_uri = SetEnvironmentVariable(
        "CYCLONEDDS_URI",
        ["file://", PathJoinSubstitution([
            FindPackageShare("amr_perception_bringup"), "config", "cyclonedds.xml"])])

    robot_state = _inc("amr_description", "robot_state.launch.py")

    base = _inc("amr_bringup", "base.launch.py",
                {"mock_mode": mock_mode, "publish_tf": "false"})

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amr_bringup"), "launch", "sensors.launch.py"])),
        condition=IfCondition(LaunchConfiguration("use_sensors")))

    localization = _inc("amr_perception_bringup", "localization.launch.py")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amr_perception_bringup"),
                                  "launch", "perception_stack.launch.py"])),
        launch_arguments={
            "yolo_model_path": LaunchConfiguration("yolo_model_path"),
            "use_object_tracker": LaunchConfiguration("use_object_tracker"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_perception")))

    # 조이스틱 텔레옵 (amr_base joy_teleop -> /cmd_vel -> roboclaw)
    joy_cfg = PathJoinSubstitution([
        FindPackageShare("amr_bringup"), "config", "joy_teleop.yaml"])
    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen",
        parameters=[joy_cfg],
        condition=IfCondition(LaunchConfiguration("use_joystick")))
    joy_teleop = Node(
        package="amr_base", executable="joy_teleop_node", name="joy_teleop",
        output="screen", parameters=[joy_cfg],
        condition=IfCondition(LaunchConfiguration("use_joystick")))

    return LaunchDescription([
        DeclareLaunchArgument("mock_mode", default_value="true",
                              description="true=하드웨어 없이 ROS만, false=실제 RoboClaw"),
        DeclareLaunchArgument("use_sensors", default_value="true",
                              description="라이다/카메라 드라이버 실행 (하드웨어 연결 시)"),
        DeclareLaunchArgument("use_perception", default_value="true",
                              description="인지 스택(stub 포함) 실행. slam_toolbox 테스트 시 false로."),
        DeclareLaunchArgument("use_object_tracker", default_value="true",
                              description="object_tracker_node 실행 여부"),
        DeclareLaunchArgument("use_joystick", default_value="false",
                              description="게임패드 텔레옵(joy_node+joy_teleop) 실행"),
        DeclareLaunchArgument("yolo_model_path", default_value="",
                              description="YOLO OpenVINO 모델 폴더. 비면 sensor_fusion stub."),
        set_cyclonedds_uri,   # 하위 노드 spawn 전에 CYCLONEDDS_URI 설정
        robot_state,
        base,
        sensors,
        localization,
        perception,
        joy_node,
        joy_teleop,
    ])
