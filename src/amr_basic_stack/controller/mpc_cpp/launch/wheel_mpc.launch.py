#!/usr/bin/env python3
"""구조 B 바퀴 MPC — C++ 판 (100 Hz, DAQP).  amr_mpc/launch/wheel.launch.py 와 인자·토픽이 같고 실행파일만 다르다.

  ros2 launch amr_mpc_cpp wheel_mpc.launch.py
  ros2 launch amr_mpc_cpp wheel_mpc.launch.py tuning_section:=tuning      # 25 Hz 설정으로 (Python 과 같은 값)

배선(토픽)은 amr_mpc 의 config/wheel.yaml 을 그대로 쓴다 — 노드 이름이 같은 wheel_mpc_node 라서.
물리값·튜닝은 amr_mpc 의 config/plant_wheel.yaml (튜닝 절은 tuning_section 으로 고른다).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    default_config = os.path.join(get_package_share_directory('amr_mpc'), 'config', 'wheel.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config, description='배선 YAML (amr_mpc/config/wheel.yaml).'),
        DeclareLaunchArgument('pose_extrapolation', default_value='true',
                              description='묵은 포즈를 현재 틱으로 외삽 (노드 머리주석). false 는 비교 시험용.'),
        DeclareLaunchArgument('model_section', default_value='model_100hz', description='plant_wheel.yaml 의 모델 절.'),
        DeclareLaunchArgument('tuning_section', default_value='tuning_100hz', description='plant_wheel.yaml 의 튜닝 절.'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        # 엔코더 카운트 차분 속도 (드라이버 encoder_wheel_speeds). model_100hz 1차 모델이 실제 바퀴 속도 기준이라
        # cmd 18 기반 /measured_wheel_speeds (91~94 ms 늦음, base.yaml 주석) 를 쓰면 모델과 어긋난다
        DeclareLaunchArgument('wheel_topic', default_value='/encoder_wheel_speeds'),
        DeclareLaunchArgument('wheel_cmd_topic', default_value='/wheel_cmd'),
        DeclareLaunchArgument('log_prefix', default_value=''),
        DeclareLaunchArgument('publish_debug', default_value='true'),
        Node(
            package='amr_mpc_cpp', executable='wheel_mpc_node', name='wheel_mpc_node', output='screen',
            parameters=[lc('config_file'),
                        {'tuning_section': lc('tuning_section'), 'model_section': lc('model_section'),
                          'pose_extrapolation': ParameterValue(lc('pose_extrapolation'), value_type=bool),
                         'odom_topic': lc('odom_topic'), 'wheel_topic': lc('wheel_topic'),
                         'wheel_cmd_topic': lc('wheel_cmd_topic'), 'log_prefix': lc('log_prefix'),
                         'publish_debug': ParameterValue(lc('publish_debug'), value_type=bool),
                         'use_sim_time': ParameterValue(lc('use_sim_time'), value_type=bool)}],
        ),
    ])
