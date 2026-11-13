#!/usr/bin/env python3
"""두 라이다 스캔 병합 launch.

/scan_front + /scan_rear -> 단일 /scan (base_link 기준 360°).
slam_toolbox / sensor_fusion 가 이 /scan 을 소비한다.

전제: robot_state_publisher가 URDF에서 base_link<-laser_front/rear TF를 발행 중이어야 함.

실행:
  ros2 launch amr_laser_tools scan_merge.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amr_laser_tools'), 'config', 'scan_merge.yaml')

    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        # 시뮬레이터 연결 시 true. 세 노드에 명시로 넘긴다 (전역 SetParameter 금지 — autonomy.launch.py 주석).
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='amr_laser_tools',
            executable='scan_sector_filter',
            name='front_cartographer_scan_filter',
            output='screen',
            parameters=[config, sim],
            remappings=[
                ('scan_in', '/scan_front'),
                ('scan_out', '/scan_front_carto'),
            ],
        ),
        Node(
            package='amr_laser_tools',
            executable='scan_sector_filter',
            name='rear_cartographer_scan_filter',
            output='screen',
            parameters=[config, sim],
            remappings=[
                ('scan_in', '/scan_rear'),
                ('scan_out', '/scan_rear_carto'),
            ],
        ),
        Node(
            package='amr_laser_tools',
            executable='scan_merger',
            name='dual_scan_merger',
            output='screen',
            parameters=[config, sim],
        ),
    ])
