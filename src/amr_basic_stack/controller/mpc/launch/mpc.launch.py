#!/usr/bin/env python3
"""MPC 궤적추종 노드.  /dra_mppi/reference_trajectory + /odometry/filtered → /cmd_vel"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('amr_mpc'), 'config', 'mpc.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이터 /clock 사용.'),
        DeclareLaunchArgument('config_file', default_value=default_config, description='MPC parameter YAML.'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='State source (EKF).'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='MPC command output.'),
        DeclareLaunchArgument('log_prefix', default_value='', description='CSV log prefix; empty = no log.'),
        # capture.sh 가 담는 /mpc/reference_horizon, /mpc/status 의 스위치.
        # 기본 true — 디버깅에 항상 필요하고 비용은 25 Hz Path 하나 + 9-원소 배열뿐이다.
        DeclareLaunchArgument('publish_debug', default_value='true',
                              description='/mpc/reference_horizon, /mpc/status publication.'),
        Node(
            package='amr_mpc',
            executable='mpc_tracking_node',
             # numpy/OpenBLAS 스레드를 1 로 묶는다. 결정변수 15~25, 행렬 최대 100×25 라
             # 스레드 동기화가 계산보다 비싸고, 제한이 없으면 코어 수(12)만큼 스레드를 띄워
             # 다른 노드의 코어를 잠식한다.
             #   실측 (8자 기준궤적, step 600 회):
             #     제한 없음  p50 1.243  p99 1.291  max 2.324 ms   OS 스레드 12
             #     1 스레드   p50 0.664  p99 0.719  max 0.772 ms   OS 스레드  1
             #   1.9 배 빠르면서 코어를 1 개만 쓴다 — 이 크기에서는 순수 이득이다.
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                             'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'},
            name='mpc_tracking_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'),
                        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                         'odom_topic': LaunchConfiguration('odom_topic'),
                         'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                         'log_prefix': LaunchConfiguration('log_prefix'),
                         'publish_debug': ParameterValue(LaunchConfiguration('publish_debug'),
                                                         value_type=bool)}],
        ),
    ])
