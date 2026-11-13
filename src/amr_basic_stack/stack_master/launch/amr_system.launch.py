#!/usr/bin/env python3
"""Top-level launch for the 2026 AMR stack.

This launch ties together the low-level robot stack, sensors, localization, and
perception pipeline from one entry point.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_perception_model_path():
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        sibling_model = (
            parent
            / "perception"
            / "models"
            / "yolo11n_openvino_model"
        )
        if sibling_model.is_dir():
            return str(sibling_model)

        workspace_model = (
            parent
            / "src"
            / "amr_basic_stack"
            / "perception"
            / "models"
            / "yolo11n_openvino_model"
        )
        if workspace_model.is_dir():
            return str(workspace_model)

    return (
        "/home/smoc/2026amr_ws/src/amr_basic_stack/perception/models/"
        "yolo11n_openvino_model"
    )


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


def generate_launch_description():
    mock_mode = LaunchConfiguration("mock_mode")
    use_robot_state = LaunchConfiguration("use_robot_state")
    use_base = LaunchConfiguration("use_base")
    use_sensors = LaunchConfiguration("use_sensors")
    use_localization = LaunchConfiguration("use_localization")
    use_perception = LaunchConfiguration("use_perception")
    use_object_tracker = LaunchConfiguration("use_object_tracker")
    use_semantic_slam = LaunchConfiguration("use_semantic_slam")
    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joystick = LaunchConfiguration("use_joystick")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    default_yolo_model_path = get_perception_model_path()

    joy_config = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "joy_teleop.yaml",
    ])

    set_cyclonedds_uri = SetEnvironmentVariable(
        "CYCLONEDDS_URI",
        [
            "file://",
            PathJoinSubstitution([
                FindPackageShare("stack_master"),
                "config",
                "cyclonedds.xml",
            ]),
        ],
    )

    robot_state = include_launch(
        "amr_description",
        "robot_state.launch.py",
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
        condition=IfCondition(use_robot_state),
    )

    base = include_launch(
        "amr_bringup",
        "base.launch.py",
        {
            "mock_mode": mock_mode,
            "publish_tf": LaunchConfiguration("base_publish_tf"),
        },
        condition=IfCondition(use_base),
    )

    sensors = include_launch(
        "amr_bringup",
        "sensors.launch.py",
        {
            "use_front_lidar": LaunchConfiguration("use_front_lidar"),
            "use_rear_lidar": LaunchConfiguration("use_rear_lidar"),
            "use_scan_merge": LaunchConfiguration("use_scan_merge"),
            "use_depth_camera": LaunchConfiguration("use_depth_camera"),
            "front_lidar_serial_port": LaunchConfiguration("front_lidar_serial_port"),
            "rear_lidar_serial_port": LaunchConfiguration("rear_lidar_serial_port"),
            "depth_camera_color_profile": LaunchConfiguration("depth_camera_color_profile"),
            "depth_camera_depth_profile": LaunchConfiguration("depth_camera_depth_profile"),
            # 점군은 sensors.launch.py 에서 기본 off 다(대역폭). traversability 의
            # 3D 갱신(depth_preprocess -> octomap/STVL)을 쓸 때만 켠다.
            "depth_camera_pointcloud_enable": LaunchConfiguration(
                "depth_camera_pointcloud_enable"),
            "depth_camera_decimation_enable": LaunchConfiguration(
                "depth_camera_decimation_enable"),
            "depth_camera_decimation_magnitude": LaunchConfiguration(
                "depth_camera_decimation_magnitude"),
        },
        condition=IfCondition(use_sensors),
    )

    localization = include_launch(
        "amr_perception_bringup",
        "localization.launch.py",
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
        condition=IfCondition(use_localization),
    )

    perception = include_launch(
        "amr_perception_bringup",
        "perception_stack.launch.py",
        {
            "observation_timing_config": LaunchConfiguration("observation_timing_config"),
            "yolo_model_path": yolo_model_path,
            "use_semantic_slam": use_semantic_slam,
            "use_object_tracker": use_object_tracker,
            "yolo_conf": LaunchConfiguration("yolo_conf"),
            "publish_semantic": LaunchConfiguration("publish_semantic"),
            "inference_hz": LaunchConfiguration("inference_hz"),
            "semantic_map_id": LaunchConfiguration("semantic_map_id"),
            "camera_position_method": LaunchConfiguration("camera_position_method"),
            "depth_roi_ratio": LaunchConfiguration("depth_roi_ratio"),
            "min_valid_depth_pixels": LaunchConfiguration("min_valid_depth_pixels"),
            "cluster_position_method": LaunchConfiguration("cluster_position_method"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        },
        condition=IfCondition(use_perception),
    )

    slam = include_launch(
        "amr_perception_bringup",
        "slam.launch.py",
        condition=IfCondition(use_slam),
    )

    rviz = include_launch(
        "amr_perception_bringup",
        "rviz.launch.py",
        condition=IfCondition(use_rviz),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[joy_config],
        condition=IfCondition(use_joystick),
    )

    joy_teleop = Node(
        package="amr_base",
        executable="joy_teleop_node",
        name="joy_teleop",
        output="screen",
        parameters=[joy_config],
        condition=IfCondition(use_joystick),
    )

    return LaunchDescription([
        DeclareLaunchArgument("mock_mode", default_value="false"),
        # 시뮬레이터 연결용. robot_state / localization(EKF) / perception include 에 명시로 넘긴다.
        # (전역 SetParameter 는 쓰지 말 것 — autonomy.launch.py 의 주석 참조.)
        # 이게 없으면 EKF 가 wall clock 으로 돌아 odom->base_footprint TF 스탬프가 /clock 과
        # 어긋나고 /odometry/filtered 가 2 Hz 대로 떨어진다 (2026-09-14 Isaac 매핑에서 실측).
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_robot_state", default_value="true"),
        DeclareLaunchArgument("use_base", default_value="true"),
        DeclareLaunchArgument(
            "use_sensors",
            default_value="true",
            description="Start LiDAR and depth camera drivers.",
        ),
        DeclareLaunchArgument("use_localization", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("observation_timing_config", default_value=PathJoinSubstitution([
            FindPackageShare("amr_perception_bringup"), "config", "observation_timing.yaml"])),
        DeclareLaunchArgument("use_object_tracker", default_value="true"),
        DeclareLaunchArgument("use_semantic_slam", default_value="false"),
        DeclareLaunchArgument("publish_semantic", default_value="false"),
        DeclareLaunchArgument("inference_hz", default_value="0.0"),
        DeclareLaunchArgument("semantic_map_id", default_value=""),
        DeclareLaunchArgument("yolo_conf", default_value="0.4"),
        # C단계 실험 정의 파라미터(기본값 = 노드 기본값 = C2 고정값).
        DeclareLaunchArgument("camera_position_method", default_value="roi_median_3d"),
        DeclareLaunchArgument("depth_roi_ratio", default_value="0.5"),
        DeclareLaunchArgument("min_valid_depth_pixels", default_value="20"),
        DeclareLaunchArgument("cluster_position_method", default_value="mean_centroid"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("base_publish_tf", default_value="false"),
        DeclareLaunchArgument("use_front_lidar", default_value="true"),
        DeclareLaunchArgument("use_rear_lidar", default_value="true"),
        DeclareLaunchArgument("use_scan_merge", default_value="true"),
        DeclareLaunchArgument("use_depth_camera", default_value="true"),
        DeclareLaunchArgument("front_lidar_serial_port", default_value="/dev/lidar_front"),
        DeclareLaunchArgument("rear_lidar_serial_port", default_value="/dev/lidar_rear"),
        DeclareLaunchArgument("depth_camera_color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_camera_depth_profile", default_value="640x480x15"),
        # 실측(2026-09-09): decimation 3 에서 /camera/depth/color/points 6.06 MB/s,
        # 끄면 307200 pt x 20 B x 15 Hz = 92 MB/s. 기본 off 를 유지한다.
        DeclareLaunchArgument("depth_camera_pointcloud_enable", default_value="false"),
        DeclareLaunchArgument("depth_camera_decimation_enable", default_value="false"),
        DeclareLaunchArgument("depth_camera_decimation_magnitude", default_value="3"),
        DeclareLaunchArgument(
            "yolo_model_path",
            default_value=default_yolo_model_path,
            description="OpenVINO YOLO model directory. Empty keeps sensor_fusion in stub mode.",
        ),
        set_cyclonedds_uri,
        robot_state,
        base,
        sensors,
        localization,
        perception,
        slam,
        rviz,
        joy_node,
        joy_teleop,
    ])
