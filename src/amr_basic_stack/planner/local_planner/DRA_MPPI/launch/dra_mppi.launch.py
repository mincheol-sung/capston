#!/usr/bin/env python3
"""Launch the DRA-MPPI reference generator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    default_config = os.path.join(
        get_package_share_directory('dra_mppi'),
        'config',
        'dra_mppi.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='시뮬레이터 /clock 사용.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='DRA-MPPI parameter YAML.',
        ),
        Node(
            package='dra_mppi',
            executable='dra_mppi_node',
            name='dra_mppi_node',
            output='screen',
            # Guide-local MC already owns explicit parallelism. Prevent BLAS
            # kernels inside each worker from multiplying the thread count.
            additional_env={
                'OPENBLAS_NUM_THREADS': '1',
                'OMP_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
                'NUMEXPR_NUM_THREADS': '1',
            },
            parameters=[config_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            respawn=True,
            respawn_delay=2.0,
            respawn_max_retries=3,
        ),
    ])
