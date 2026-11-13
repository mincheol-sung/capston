#!/usr/bin/env python3
"""traversability_node 단독 실행.

    ros2 launch traversability traversability.launch.py

/map (nav2_map_server) 과 /scan 이 이미 떠 있어야 한다. autonomy.launch.py 에서
use_map_server:=true 로 띄우는 map_server 가 그 /map 이다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('traversability'), 'config', 'traversability.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='traversability',
            executable='traversability_node',
            name='traversability_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
