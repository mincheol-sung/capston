#!/usr/bin/env python3
"""Bring up RViz-goal navigation with Theta*, DRA-MPPI, and MPC.

    ros2 launch stack_master autonomy.launch.py map:=<맵이름>

RViz 가 rviz/autonomy.rviz 로 같이 뜬다. 그 설정에만 2D Pose Estimate /
2D Goal Pose 도구가 있다 (amr_perception.rviz 에는 없어서 목표를 못 찍는다).

로봇을 움직이는 순서:
  1) RViz "2D Pose Estimate" 로 현재 위치·방향   -> /initialpose
  2) RViz "2D Goal Pose" 로 목적지               -> /goal_pose
     Theta* 가 /global_path 를 낸다 (초록 선)
  3) 이 launch 를 띄운 터미널에서 'a'            -> 게이트 무장, 로봇이 움직인다
     's' 또는 SPACE 로 즉시 정지

use_mpc:=true 로 띄우면 3) 없이 목표 즉시 주행한다. 처음에는 쓰지 말 것.

초기 위치를 AMCL 로 자동으로 잡으려면 (RViz 2D Pose Estimate 없이):
  ros2 launch stack_master autonomy.launch.py map:=<맵> \
      use_amcl_bootstrap:=true use_joystick:=true use_command_gate:=false
  -> 조이스틱으로 몇 미터 몰면 AMCL 이 수렴해 초기 위치가 잡힌다.
     그 뒤 이 launch 를 끄고 위 기본 명령으로 다시 띄운다.
  ※ use_command_gate:=false 가 필요한 이유: 게이트는 disarmed 여도 50 Hz 로
     0 을 발행해서 조이스틱과 /cmd_vel 에서 섞인다.

use_perception:=true(기본)면 장애물 인지→IMM tracking→prediction도 함께 띄운다.

객체 의미 지도·접근 경로 시험(기본은 미리보기, 실제 목표 발행은 별도):
  ros2 launch stack_master autonomy.launch.py map:=<맵> use_semantic_navigation:=true

A ``nav2_map_server`` publishes the saved map on ``/map`` for RViz. Cartographer
in localization mode only emits the three freshly built submaps (on
``/map_carto``), so without this node RViz has no full map to align against and
the robot looks mislocalized even when it is not. Set the RViz Fixed Frame to
``map``.
"""

import hashlib
from pathlib import Path
import uuid

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def include_launch(package_name, launch_file, launch_arguments=None, condition=None):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
    )
    kwargs = {}
    if launch_arguments:
        kwargs["launch_arguments"] = launch_arguments.items()
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(source, **kwargs)


def _configure_semantic(context):
    """Resolve map/session identity once before any robot processes are launched."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    if not IfCondition(LaunchConfiguration("use_perception")).evaluate(context):
        raise RuntimeError("use_semantic_navigation:=true requires use_perception:=true "
                           "to reuse the existing RGB-D detector")
    map_id = value("semantic_map_id")
    if not map_id:
        name = value("map")
        folder = Path(value("maps_dir")).expanduser() / name
        yaml_path = folder / f"{name}.yaml"
        try:
            metadata = yaml.safe_load(yaml_path.read_text())
            image = Path(metadata["image"])
            if not image.is_absolute():
                image = folder / image
            digest = hashlib.sha256()
            for tag, path in (("yaml", yaml_path), ("image", image),
                              ("pbstream", folder / f"{name}.pbstream")):
                digest.update(tag.encode())
                with path.open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(block)
            map_id = f"{name}-{digest.hexdigest()[:16]}"
        except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            raise RuntimeError("Cannot identify semantic map from YAML/image/pbstream; "
                               "check maps_dir/map or provide semantic_map_id") from exc
    correction = IfCondition(LaunchConfiguration("semantic_cartographer_correction")).evaluate(
        context)
    graph_id = (value("semantic_graph_id") or f"carto-{uuid.uuid4().hex}") if correction else ""
    database = value("semantic_database_path")
    if not database:
        identity = hashlib.sha256(f"{map_id}\0{graph_id}".encode()).hexdigest()[:24]
        database = str(Path.home() / ".ros" / "amr" / "semantic" / f"{identity}.json")
    return [
        SetLaunchConfiguration("_semantic_map_id", map_id),
        SetLaunchConfiguration("_semantic_graph_id", graph_id),
        SetLaunchConfiguration("_semantic_database_path", database),
        LogInfo(msg=f"Semantic objects: map_id={map_id}, graph_id={graph_id or 'fixed'}, "
                    f"database={database}"),
    ]


def generate_launch_description():
    map_name = LaunchConfiguration("map")
    maps_dir = LaunchConfiguration("maps_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_base = LaunchConfiguration("use_base")
    use_sensors = LaunchConfiguration("use_sensors")
    use_localization = LaunchConfiguration("use_localization")
    use_perception = LaunchConfiguration("use_perception")
    use_object_tracker = LaunchConfiguration("use_object_tracker")
    use_map_localization = LaunchConfiguration("use_map_localization")
    use_map_server = LaunchConfiguration("use_map_server")
    use_global_path = LaunchConfiguration("use_global_path")
    use_traversability = LaunchConfiguration("use_traversability")
    use_voxel_3d = LaunchConfiguration("use_voxel_3d")
    use_rviz = LaunchConfiguration("use_rviz")
    use_amcl_bootstrap = LaunchConfiguration("use_amcl_bootstrap")
    use_joystick = LaunchConfiguration("use_joystick")
    use_command_gate = LaunchConfiguration("use_command_gate")
    use_semantic = LaunchConfiguration("use_semantic_navigation")

    # 구조 A(body: 몸체 twist → /cmd_vel) 와 구조 B(wheel: 바퀴 각속도 → /wheel_cmd) 중 택일.
    # LaunchConfigurationEquals 는 Jazzy 에서 deprecated 경고를 내므로 PythonExpression 을 쓴다.
    def _structure_is(name, also=None):
        parts = ["'", LaunchConfiguration("mpc_structure"), f"' == '{name}'"]
        if also is not None:
            parts = ["'", also, "'.lower() in ('true','1') and "] + parts
        return IfCondition(PythonExpression(parts))

    use_dra_mppi = LaunchConfiguration("use_dra_mppi")
    start_mpc = LaunchConfiguration("start_mpc")

    default_maps_dir = PathJoinSubstitution([
        FindPackageShare("global_planner"),
        "maps",
    ])
    default_theta_star_config = PathJoinSubstitution([
        FindPackageShare("global_planner"),
        "config",
        "theta_star.yaml",
    ])
    default_dra_mppi_config = PathJoinSubstitution([
        FindPackageShare("dra_mppi"),
        "config",
        "dra_mppi.yaml",
    ])
    default_mpc_config = PathJoinSubstitution([
        FindPackageShare("amr_mpc"),
        "config",
        "mpc.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("stack_master"),
        "rviz",
        "autonomy.rviz",
    ])
    default_traversability_config = PathJoinSubstitution([
        FindPackageShare("traversability"),
        "config",
        "traversability.yaml",
    ])

    # GroupAction(scoped=True) 로 감싼다. IncludeLaunchDescription 의
    # launch_arguments 는 부모 스코프로 새어 나간다. 아래에서 넘기는
    # "use_rviz": "false" 가 이 파일 자신의 use_rviz 를 덮어써서 RViz 가
    # 조용히 안 뜨는 문제가 있었다(2026-09-09 실측).
    robot_stack = GroupAction([include_launch(
        "stack_master",
        "amr_system.launch.py",
        {
            "mock_mode": LaunchConfiguration("mock_mode"),
            "use_sim_time": use_sim_time,
            "use_robot_state": "true",
            "use_base": use_base,
            "use_sensors": use_sensors,
            "use_localization": use_localization,
            "use_perception": use_perception,
            "use_object_tracker": use_object_tracker,
            "use_semantic_slam": "false",
            "publish_semantic": IfElseSubstitution(
                use_semantic, "true", LaunchConfiguration("publish_semantic")),
            "inference_hz": LaunchConfiguration("inference_hz"),
            "use_slam": "false",
            "use_rviz": "false",
            # 기본은 false. MPC 게이트가 /cmd_vel 을 독점하기 때문이다.
            # AMCL 부트스트랩처럼 손으로 몰아야 할 때만 켠다 — 그때는
            # use_command_gate:=false 도 같이 줘야 한다(아래 참조).
            "use_joystick": use_joystick,
            # The EKF owns odom -> base_footprint.
            "base_publish_tf": "false",
            # use_voxel_3d 를 켜면 D455 점군을 함께 켠다. 이게 없으면
            # depth_preprocess_node 가 구독할 /camera/depth/color/points 자체가
            # 없어서 복셀이 한 개도 안 나온다(sensors.launch.py 기본 off).
            "depth_camera_pointcloud_enable": IfElseSubstitution(
                use_semantic, "true", use_voxel_3d),
            # RealSense decimation can reduce aligned-depth resolution relative to RGB.
            # Semantic projection requires matching RGB/depth/CameraInfo dimensions.
            "depth_camera_decimation_enable": IfElseSubstitution(
                use_semantic, "false", use_voxel_3d),
        },
    )], scoped=True)

    map_localization = include_launch(
        "amr_perception_bringup",
        "cartographer.launch.py",
        {
            "mode": "localization",
            "map": map_name,
            "maps_dir": maps_dir,
            "use_sim_time": use_sim_time,
            # localization_2d.lua 와 일치. IMU 위치에 있으면서 base_link 와
            # 축이 정렬되어 RViz 2D Pose의 yaw가 90도 돌아가지 않는다.
            "tracking_frame": "camera_gyro_frame",
            # 시뮬레이터 지도는 localization_sim_2d.lua (max_range 30) 로 측위한다. 비면 기본.
            "lua_basename": LaunchConfiguration("carto_lua"),
            "use_dynamic_filter": LaunchConfiguration("use_dynamic_filter"),
            "dynamic_filter_config": LaunchConfiguration("dynamic_filter_config"),
            "scan_front_topic": LaunchConfiguration("scan_front_topic"),
            "scan_rear_topic": LaunchConfiguration("scan_rear_topic"),
        },
        condition=IfCondition(use_map_localization),
    )

    # D455 점군 -> depth_preprocess -> octomap_server(3D voxel).
    # traversability_node 의 2D 격자와 별개다 — 이쪽은 odom 프레임에 누적한다.
    voxel_3d = include_launch(
        "traversability",
        "voxel_3d.launch.py",
        {
            "use_sim_time": use_sim_time,
            "use_octomap": "true",
            "use_stvl": "false",
        },
        condition=IfCondition(use_voxel_3d),
    )

    # The semantic map freshness gate also requires depth-band sensor input.
    # voxel_3d already owns this node when enabled; do not start a second copy.
    semantic_depth = Node(
        package="traversability", executable="depth_preprocess_node",
        name="depth_preprocess_node", output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("traversability"), "config",
                                  "depth_preprocess.yaml"]),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_semantic, "'.lower() in ('true','1') and '", use_voxel_3d,
            "'.lower() not in ('true','1')",
        ])),
    )
    semantic_navigation = GroupAction([
        include_launch("amr_perception_bringup", "semantic_navigation.launch.py", {
            "map_id": LaunchConfiguration("_semantic_map_id"),
            "graph_id": LaunchConfiguration("_semantic_graph_id"),
            "database_path": LaunchConfiguration("_semantic_database_path"),
            "config": LaunchConfiguration("semantic_config"),
            "use_sim_time": use_sim_time,
            "start_detector": "false",
            "enable_cartographer_correction": LaunchConfiguration(
                "semantic_cartographer_correction"),
            "enable_goal_dispatch": LaunchConfiguration("semantic_goal_dispatch"),
        }),
    ], scoped=True, condition=IfCondition(use_semantic))

    global_path_planner = Node(
        package="global_planner",
        executable="rviz_theta_star_node",
        name="rviz_theta_star_node",
        output="screen",
        parameters=[
            LaunchConfiguration("theta_star_config"),
            {
                "use_sim_time": use_sim_time,
                "map": map_name,
                "maps_dir": maps_dir,
            },
        ],
        condition=IfCondition(use_global_path),
    )

    # <maps_dir>/<map>/<map>.yaml — global_planner 의 find_ros_map_yaml 과 같은 규칙.
    map_yaml_file = [maps_dir, "/", map_name, "/", map_name, ".yaml"]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": ParameterValue(map_yaml_file, value_type=str),
            "topic_name": "map",
            "frame_id": "map",
        }],
        condition=IfCondition(use_map_server),
    )

    # map_server 는 라이프사이클 노드다. configure/activate 를 해주지 않으면
    # 프로세스는 떠 있는데 /map 은 한 번도 안 나간다.
    map_server_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
        condition=IfCondition(use_map_server),
    )

    # /map (map_server) + /scan 을 3-state traversable map 으로 합쳐
    # /traversable_map 으로 낸다. Theta* 는 신선한 이 지도를 저장 PGM 대신
    # 현재 계획용 기준 costmap 으로 쓴다.
    # map_server 가 꺼져 있으면 /map 이 없어 이 노드는 기다리기만 한다.
    traversability = Node(
        package="traversability",
        executable="traversability_node",
        name="traversability_node",
        output="screen",
        parameters=[
            LaunchConfiguration("traversability_config"),
            {"use_sim_time": use_sim_time,
             "use_depth": ParameterValue(IfElseSubstitution(
                 use_semantic, "true", use_voxel_3d), value_type=bool)},
        ],
        condition=IfCondition(use_traversability),
    )

    # AMCL 전역 측위로 cartographer 초기 위치를 자동으로 잡는다. RViz
    # "2D Pose Estimate" 를 손으로 찍는 대신 쓴다. AMCL 은 map->odom TF 를
    # 내지 않고(/amcl_pose 만) 수렴하면 /initialpose 로 넘긴다.
    # 로봇을 몇 미터 주행시켜야 수렴한다.
    # use_amcl_wiggle 을 켜면 amcl_wiggle_node 가 수렴에 필요한 최소 이동을
    # 자동으로 만든다 (20 cm 사각형 두 바퀴 = 경로 1.6 m). 조이스틱 없이
    # 초기위치를 잡는 경로다. 게이트 입력 토픽에 쓰고 /mpc/arm 으로 arm 하므로
    # use_command_gate 는 true 여야 한다 — 위 조이스틱 절차와 반대다.
    amcl_bootstrap = GroupAction([include_launch(
        "amr_perception_bringup",
        "amcl_bootstrap.launch.py",
        {
            "use_sim_time": use_sim_time,
            "use_wiggle": LaunchConfiguration("use_amcl_wiggle"),
        },
        condition=IfCondition(use_amcl_bootstrap),
    )], scoped=True)

    dra_mppi = include_launch(
        "dra_mppi",
        "dra_mppi.launch.py",
        {"config_file": LaunchConfiguration("dra_mppi_config"),
         "use_sim_time": use_sim_time},
        condition=IfCondition(use_dra_mppi),
    )

    # 구조 A. 출력은 게이트가 단독 소유하는 /cmd_vel 이 아니라 raw 토픽으로 낸다.
    mpc = include_launch(
        "amr_mpc",
        "mpc.launch.py",
        {
            "config_file": LaunchConfiguration("mpc_config"),
            "odom_topic": "/odometry/filtered",
            "cmd_vel_topic": "/mpc/cmd_vel_raw",
            "use_sim_time": use_sim_time,
            "log_prefix": LaunchConfiguration("mpc_log_prefix"),
            "publish_debug": LaunchConfiguration("mpc_publish_debug"),
        },
        condition=_structure_is("body", start_mpc),
    )

    # 구조 B. odom_topic 을 /odometry/filtered 로 덮어쓴다 — 노드 기본값은 /odom 이지만
    # 플래너(dra_mppi.yaml odometry_topic)와 같은 원천이어야 오차 프레임이 맞는다.
    # wheel_tracking.py 헤더 주석대로, 상태 원천을 바꿨으니 θ_d 재측정 대상이다.
    wheel_mpc = include_launch(
        "amr_mpc",
        "wheel.launch.py",
        {
            "config_file": LaunchConfiguration("wheel_mpc_config"),
            "odom_topic": "/odometry/filtered",
            "wheel_cmd_topic": "/wheel_mpc/cmd_raw",
            "use_sim_time": use_sim_time,
            "log_prefix": LaunchConfiguration("mpc_log_prefix"),
            "publish_debug": LaunchConfiguration("mpc_publish_debug"),
        },
        condition=_structure_is("wheel", start_mpc),
    )

    # 구조 B 의 C++ 판 (amr_mpc_cpp, 100 Hz·DAQP·d 단 dead time). 인자·토픽이 위 Python 판과 같다.
    # 2026-10-22 추가. 실차 검증 전이라 기본값은 Python 판(wheel) 그대로다.
    wheel_mpc_cpp = include_launch(
        "amr_mpc_cpp",
        "wheel_mpc.launch.py",
        {
            "config_file": LaunchConfiguration("wheel_mpc_config"),
            "odom_topic": "/odometry/filtered",
            "wheel_cmd_topic": "/wheel_mpc/cmd_raw",
            "use_sim_time": use_sim_time,
            "log_prefix": LaunchConfiguration("mpc_log_prefix"),
            "publish_debug": LaunchConfiguration("mpc_publish_debug"),
        },
        condition=_structure_is("wheel_cpp", start_mpc),
    )

    # RViz. 이 설정에는 2D Pose Estimate / 2D Goal Pose 도구가 들어 있다.
    # amr_perception.rviz 에는 없어서 목표를 못 찍는다.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_autonomy",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # ★ 이 게이트는 disarmed 상태에서도 50 Hz 로 0 을 계속 발행한다(적극적
    #   정지 명령). 그래서 게이트가 떠 있는 동안에는 조이스틱으로 몰 수 없다 —
    #   두 발행자가 /cmd_vel 에서 섞여 명령의 일부만 실행된다.
    #   손으로 몰아야 하면 use_command_gate:=false 로 끄고, 다 몰았으면 다시
    #   켠 구성으로 띄운다.
    #   구조 B 도 반드시 게이트를 거친다. /wheel_cmd 는 Float32MultiArray 라 예전 게이트가
    #   중계하지 못했는데(Twist 하드코딩), keyboard_command_gate 에 command_type 을 붙여
    #   같은 arming·freshness·정지 논리를 그대로 쓴다. 게이트 없이 /wheel_cmd 를 드라이버에
    #   직결하면 기준열이 들어오는 즉시 arming 없이 바퀴가 돈다.
    #   forward_on_receive: 받는 즉시 중계 (keyboard_command_gate 주석). 100 Hz MPC 를 50 Hz 재샘플하면
    #   명령 절반이 사라지고 0~20 ms 무작위 지연이 붙으므로 wheel_cpp 에서만 켠다.
    def _gate(name, command_type, input_topic, output_topic, condition, forward_on_receive=False):
        return Node(
            package="amr_mpc",
            executable="keyboard_command_gate",
            name=name,
            output="screen",
            parameters=[{
                "command_type": command_type,
                "input_topic": input_topic,
                "output_topic": output_topic,
                "arm_topic": "/mpc/arm",
                "initially_armed": ParameterValue(
                    LaunchConfiguration("use_mpc"),
                    value_type=bool,
                ),
                "keyboard_enabled": True,
                "command_timeout": 0.20,
                "publish_frequency": 50.0,
                "forward_on_receive": forward_on_receive,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=condition,
        )

    command_gate = _gate(
        "mpc_command_gate", "twist", "/mpc/cmd_vel_raw", "/cmd_vel",
        _structure_is("body", use_command_gate),
    )
    wheel_command_gate = _gate(
        "wheel_mpc_command_gate", "wheels", "/wheel_mpc/cmd_raw", "/wheel_cmd",
        _structure_is("wheel", use_command_gate),
    )
    wheel_cpp_command_gate = _gate(
        "wheel_mpc_command_gate", "wheels", "/wheel_mpc/cmd_raw", "/wheel_cmd",
        _structure_is("wheel_cpp", use_command_gate), forward_on_receive=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description=(
                "Required map folder/name containing <map>.yaml and "
                "<map>.pbstream. Pass map:=<map_name>."
            ),
        ),
        DeclareLaunchArgument(
            "maps_dir",
            default_value=default_maps_dir,
            description="Directory containing the installed global-planner maps.",
        ),
        DeclareLaunchArgument("mock_mode", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_dynamic_filter", default_value="false"),
        DeclareLaunchArgument("dynamic_filter_config", default_value=PathJoinSubstitution([
            FindPackageShare("dynamic_filter"), "config", "dynamic_filter.yaml"])),
        DeclareLaunchArgument("scan_front_topic", default_value="/scan_front_carto"),
        DeclareLaunchArgument("scan_rear_topic", default_value="/scan_rear_carto"),
        DeclareLaunchArgument(
            "carto_lua", default_value="",
            description="cartographer 측위 lua 파일명 override. Isaac 창고 지도면 localization_sim_2d.lua.",
        ),
        # ★ use_sim_time 은 include 마다 명시로 넘긴다. launch_ros SetParameter(전역) 를 쓰면
        #   -p 로 붙는데, rcl 은 -p 가 params 파일보다 앞에 오면 그 뒤 /** 와일드카드
        #   파일(launch 의 dict 파라미터)이 노드 전용 yaml 항목을 못 덮는다 — 2026-09-14 실측:
        #   wheel_mpc_node 의 wheel_cmd_topic override 가 사라져 게이트를 우회해 /wheel_cmd 에
        #   직접 발행했다 (disarmed 인데 로봇이 6.5 m 주행). 전역 SetParameter 금지.
        DeclareLaunchArgument("use_base", default_value="true"),
        DeclareLaunchArgument("use_sensors", default_value="true"),
        DeclareLaunchArgument("use_localization", default_value="true"),
        DeclareLaunchArgument(
            "use_perception",
            default_value="true",
            description=(
                "Start detection fusion, CV/CT IMM tracker, and multimodal "
                "motion predictor required by DRA-MPPI dynamic risk."
            ),
        ),
        DeclareLaunchArgument(
            "use_object_tracker",
            default_value="true",
            description="Start object_tracker_node inside the perception stack.",
        ),
        DeclareLaunchArgument("use_map_localization", default_value="true"),
        DeclareLaunchArgument(
            "use_semantic_navigation", default_value="false",
            description="Start object memory and object-goal preview; reuse RGB-D inference.",
        ),
        DeclareLaunchArgument(
            "semantic_cartographer_correction", default_value="true",
            description="Reproject retained observations using optimized Cartographer nodes.",
        ),
        DeclareLaunchArgument(
            "semantic_goal_dispatch", default_value="false",
            description="Allow object actions to publish /goal_pose; still requires manual arm.",
        ),
        DeclareLaunchArgument("semantic_map_id", default_value="",
                              description="Optional identity override; default hashes map files."),
        DeclareLaunchArgument("semantic_graph_id", default_value="",
                              description="Optional saved-session override; default new run ID."),
        DeclareLaunchArgument(
            "semantic_database_path", default_value="",
            description="Optional DB override; default per-map/per-session JSON.",
        ),
        DeclareLaunchArgument(
            "semantic_config", default_value=IfElseSubstitution(
                use_semantic, PathJoinSubstitution([
                    FindPackageShare("semantic_slam"), "config", "semantic.yaml"]), ""),
            description="Object mapper and navigation parameters.",
        ),
        DeclareLaunchArgument("publish_semantic", default_value="false"),
        DeclareLaunchArgument("inference_hz", default_value=IfElseSubstitution(
            use_semantic, "5.0", "0.0")),
        DeclareLaunchArgument(
            "use_map_server",
            default_value="true",
            description=(
                "Publish the saved map on /map for RViz. Turn off if another "
                "map_server or slam_toolbox already owns that topic."
            ),
        ),
        DeclareLaunchArgument("use_global_path", default_value="true"),
        DeclareLaunchArgument(
            "use_voxel_3d",
            default_value="false",
            description=(
                "D455 점군 -> depth_preprocess_node -> octomap_server 를 함께 띄우고 "
                "카메라 점군을 켠다. RViz 의 'Octomap Voxels' 가 이때만 나온다. "
                "실측 대역폭: 점군 6.06 MB/s, 복셀 마커 14.14 MB/s (decimation 3). "
                "octomap_server 는 apt 정식 설치다(2026-09-15). /opt/ros/jazzy 소싱이면 된다."
            ),
        ),
        DeclareLaunchArgument(
            "use_traversability",
            default_value="true",
            description=(
                "Publish /traversable_map (0 passable / 100 blocked / -1 "
                "unknown) from /map and /scan. Requires use_map_server:=true."
            ),
        ),
        DeclareLaunchArgument("use_dra_mppi", default_value="true"),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description=(
                "게임패드 teleop. 기본 false — MPC 게이트가 /cmd_vel 을 "
                "독점한다. 손으로 몰려면 use_command_gate:=false 와 같이 켤 것."
            ),
        ),
        DeclareLaunchArgument(
            "use_command_gate",
            default_value="true",
            description=(
                "MPC 명령 게이트(/mpc/cmd_vel_raw -> /cmd_vel). false 면 자율 "
                "주행이 불가능하다. 조이스틱으로 몰 때만 끈다."
            ),
        ),
        DeclareLaunchArgument(
            "use_amcl_wiggle",
            default_value="false",
            description=(
                "AMCL 수렴용 최소 이동을 자동으로 만든다 (20 cm 사각형 두 바퀴 "
                "= 경로 1.6 m). use_amcl_bootstrap 과 use_command_gate 가 둘 다 "
                "true 여야 하고, 로봇 주변 반경 30 cm 가 비어 있어야 한다. "
                "조이스틱 없이 초기위치를 잡는 경로다."
            ),
        ),
        DeclareLaunchArgument(
            "use_amcl_bootstrap",
            default_value="false",
            description=(
                "AMCL 전역 측위로 cartographer 초기 위치를 자동으로 잡는다. "
                "켜면 RViz 2D Pose Estimate 를 찍을 필요가 없지만, 로봇을 "
                "몇 미터 주행시켜야 수렴한다. use_map_server 가 필요하다."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description=(
                "2D Pose Estimate / 2D Goal Pose 도구가 있는 RViz 를 띄운다. "
                "이게 없으면 목표를 찍을 수 없어 로봇이 움직이지 않는다."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=IfElseSubstitution(use_semantic, PathJoinSubstitution([
                FindPackageShare("semantic_slam"), "rviz", "semantic.rviz"]),
                default_rviz_config),
            description="자율주행 확인용 RViz 설정 파일.",
        ),
        DeclareLaunchArgument(
            "start_mpc",
            default_value="true",
            description="Start the MPC computation node.",
        ),
        DeclareLaunchArgument(
            "mpc_structure",
            default_value="wheel",
            choices=["body", "wheel", "wheel_cpp"],
            description=(
                "어느 MPC 를 띄울까. "
                "body = 몸체 twist 출력(mpc_tracking_node → /mpc/cmd_vel_raw → 게이트 → /cmd_vel, "
                "드라이버가 T⁻¹ 로 바퀴를 푼다). "
                "wheel = 바퀴 각속도 출력(wheel_tracking_node → /wheel_mpc/cmd_raw → 게이트 → "
                "/wheel_cmd, 기구학 변환이 MPC 안이라 바퀴 제약을 box 로 직접 건다). "
                "wheel_cpp = wheel 의 C++ 판(amr_mpc_cpp, 100 Hz, 게이트는 받는 즉시 중계). "
                "둘은 배타적이다 — 동시에 띄우면 드라이버가 두 명령을 받는다."
            ),
        ),
        DeclareLaunchArgument(
            "wheel_mpc_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("amr_mpc"), "config", "wheel.yaml"]
            ),
            description="구조 B(wheel) 파라미터 YAML. mpc_structure:=wheel 일 때만 쓰인다.",
        ),
        DeclareLaunchArgument(
            "use_mpc",
            default_value="false",
            description=(
                "Initial MPC command gate state. false waits for keyboard 'a'; "
                "true permits motion immediately after a valid goal."
            ),
        ),
        DeclareLaunchArgument(
            "theta_star_config",
            default_value=default_theta_star_config,
            description="Theta* global planner parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "traversability_config",
            default_value=default_traversability_config,
            description="traversability_node parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "dra_mppi_config",
            default_value=default_dra_mppi_config,
        ),
        DeclareLaunchArgument(
            "mpc_config",
            default_value=default_mpc_config,
        ),
        DeclareLaunchArgument(
            "mpc_log_prefix",
            default_value="",
            description="Optional MPC CSV log prefix; empty disables logging.",
        ),
        DeclareLaunchArgument(
            "mpc_publish_debug",
            default_value="true",
            description=(
                "Publish /mpc/reference_horizon and /mpc/status. Left on by "
                "default so stack_master/scripts/capture.sh can bag the whole "
                "Theta* -> MPPI -> MPC -> gate chain without a relaunch."
            ),
        ),
        OpaqueFunction(function=_configure_semantic, condition=IfCondition(use_semantic)),
        robot_stack,
        map_localization,
        map_server,
        map_server_lifecycle,
        amcl_bootstrap,
        traversability,
        voxel_3d,
        semantic_depth,
        semantic_navigation,
        global_path_planner,
        dra_mppi,
        mpc,
        wheel_mpc,
        wheel_mpc_cpp,
        command_gate,
        wheel_command_gate,
        wheel_cpp_command_gate,
        rviz,
    ])
