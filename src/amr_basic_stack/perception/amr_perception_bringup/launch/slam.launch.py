#!/usr/bin/env python3
"""slam_toolbox 매핑 launch (2D LiDAR SLAM 백본).

입력: /scan (dual_scan_merger), odom->base_footprint TF (robot_localization EKF)
출력: /map (nav_msgs/OccupancyGrid), map->odom TF (드리프트 보정)

전제: full_system이 실행 중이어야 함(/scan + odom TF 제공).
     단, full_system의 semantic_slam stub과 /map 충돌 방지 위해
     full_system은 use_perception:=false 로 띄우거나 stub을 끌 것.

실행:
  ros2 launch amr_perception_bringup slam.launch.py
저장:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params_file = PathJoinSubstitution([
        FindPackageShare("amr_perception_bringup"),
        "config",
        "slam_toolbox.yaml",
    ])

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params_file),
        slam_toolbox,
    ])
