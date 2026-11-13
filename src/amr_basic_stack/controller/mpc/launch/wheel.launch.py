#!/usr/bin/env python3
"""구조 B — 바퀴 각속도 MPC 노드.  /dra_mppi/reference_trajectory + odom + /measured_wheel_speeds → /wheel_cmd

구조 A 의 mpc.launch.py 와 같은 형태다. 다른 점만 적으면:
  · 출력이 Twist(/cmd_vel) 가 아니라 Float32MultiArray[4](/wheel_cmd) 다.
  · 바퀴 속도 피드백(/measured_wheel_speeds)이 추가로 필요하다 — 끊기면 노드가 0 을 낸다.
  · 기구학 변환 T 가 MPC 안에 있어서 바퀴 제약을 box 로 직접 건다 (구조 A 는 L1 다면체 대리식).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('amr_mpc'), 'config', 'wheel.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이터 /clock 사용.'),
        DeclareLaunchArgument('config_file', default_value=default_config, description='Wheel MPC parameter YAML.'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='Pose source.'),
        DeclareLaunchArgument('wheel_topic', default_value='/measured_wheel_speeds', description='Encoder wheel speeds.'),
        DeclareLaunchArgument('wheel_cmd_topic', default_value='/wheel_cmd', description='Wheel MPC command output.'),
        DeclareLaunchArgument('log_prefix', default_value='', description='CSV log prefix; empty = no log.'),
        DeclareLaunchArgument('publish_debug', default_value='true',
                              description='/wheel_mpc/reference_horizon, /wheel_mpc/status publication.'),
        Node(
            package='amr_mpc',
            executable='wheel_tracking_node',
            # 구조 A(mpc.launch.py)와 같은 이유로 BLAS 스레드를 1 로 묶는다. 결정변수 4·Nc=20,
            # 제약 80 행이라 스레드 동기화가 계산보다 비싸고, 제한이 없으면 코어 수만큼 스레드를
            # 띄워 다른 노드의 코어를 잠식한다.
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                            'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'},
            name='wheel_mpc_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'),
                        {'odom_topic': LaunchConfiguration('odom_topic'),
                         'wheel_topic': LaunchConfiguration('wheel_topic'),
                         'wheel_cmd_topic': LaunchConfiguration('wheel_cmd_topic'),
                         'log_prefix': LaunchConfiguration('log_prefix'),
                         'publish_debug': ParameterValue(LaunchConfiguration('publish_debug'),
                                                         value_type=bool),
                         'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        ),
    ])
