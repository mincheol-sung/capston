#!/usr/bin/env python3
"""RViz 시각화 launch — 융합/SLAM 결과를 한 화면에.

표시: 로봇모델(CAD) + TF + /scan(병합 라이다) + /map(SLAM) +
      /detection_markers(융합 검출) + /camera/color(영상).

실행:
  ros2 launch amr_perception_bringup rviz.launch.py
  (SLAM 지도를 볼 땐 RViz 좌상단 Fixed Frame을 map 으로)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('amr_perception_bringup'),
        'rviz', 'amr_perception.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_cfg),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
