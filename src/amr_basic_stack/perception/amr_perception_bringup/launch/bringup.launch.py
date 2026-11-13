#!/usr/bin/env python3
"""전체 시스템 통합 launch.

측위(localization) + 인지 스택(perception_stack)을 한 번에 실행한다.
실행 순서 원칙(이슈 C): odom/IMU EKF로 임시 pose 부트스트랩 -> 인지 파이프라인.

실행:
  ros2 launch amr_perception_bringup bringup.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('amr_perception_bringup'), 'launch')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')))

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'perception_stack.launch.py')),
        launch_arguments={
            'use_object_tracker': LaunchConfiguration('use_object_tracker'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('use_object_tracker', default_value='true'),
        localization,
        perception,
    ])
