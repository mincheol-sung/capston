#!/usr/bin/env python3
"""구조 B (바퀴 각속도 입력 MPC) 원궤적 시험.
   ros2 launch amr_mpc wheel_circle.launch.py sim:=true speed:=0.5 duration:=20.0     # 로봇 없이
   ros2 launch amr_mpc wheel_circle.launch.py sim:=false speed:=0.3 duration:=20.0    # 실차: 드라이버가 /wheel_cmd 를 받아야 한다
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BLAS1 = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('speed', default_value='0.3'),
        DeclareLaunchArgument('duration', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('log_prefix', default_value='wheel_sim'),
        Node(package='amr_mpc', executable='sim_plant_wheel_node', name='sim_plant_wheel_node', output='screen',
             condition=IfCondition(lc('sim')), parameters=[{'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc', executable='fake_reference_node', name='fake_reference_node', output='screen',
             parameters=[{'radius': lc('radius'), 'speed': lc('speed'), 'duration': lc('duration'),
                          'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc', executable='wheel_tracking_node', name='wheel_mpc_node', output='screen',
             additional_env=BLAS1,
             parameters=[{'odom_topic': lc('odom_topic'), 'log_prefix': lc('log_prefix')}]),
    ])
