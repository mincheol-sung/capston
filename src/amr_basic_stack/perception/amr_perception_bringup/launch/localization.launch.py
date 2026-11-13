#!/usr/bin/env python3
"""측위(Localization) launch — 이슈 C 보강.

robot_localization의 ekf_node를 config/ekf.yaml 설정으로 실행한다.
IMU(/imu/base_link) + 휠 오도메트리(/odom)를 융합 -> /odometry/filtered 발행 +
odom -> base_footprint TF 발행(부트스트랩 pose). base_footprint 가 URDF 루트다.

실행:
  ros2 launch amr_perception_bringup localization.launch.py
확인:
  ros2 topic echo /odometry/filtered
  ros2 run tf2_ros tf2_echo odom base_link
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('amr_perception_bringup'), 'config', 'ekf.yaml')

    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # D455 IMU 를 로봇 body 프레임으로 회전시켜 재발행한다.
        # robot_localization 은 imu0_config 를 센서 프레임 기준으로 해석하므로,
        # optical 프레임 원본을 그대로 주면 자이로 yaw 가 vroll 로 옮겨가
        # two_d_mode 에 눌려 통째로 버려진다. 자세한 내용은 노드 docstring 참조.
        # (cartographer 는 원본 /camera/imu 를 그대로 쓴다.)
        Node(
            package='amr_perception_bringup',
            executable='imu_to_base_node',
            name='imu_to_base_node',
            output='screen',
            parameters=[{'target_frame': 'base_footprint'}, sim],
            remappings=[('imu_in', '/camera/imu'),
                        ('imu_out', '/imu/base_link')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, sim],
        ),
    ])
