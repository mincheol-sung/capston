#!/usr/bin/env python3
"""C++ 바퀴 MPC 원궤적 시험 — amr_mpc/launch/wheel_circle.launch.py 와 같고 MPC 노드만 C++ 판.
   ros2 launch amr_mpc_cpp wheel_circle.launch.py sim:=true speed:=0.5 duration:=20.0      # 로봇 없이
   ros2 launch amr_mpc_cpp wheel_circle.launch.py sim:=false speed:=0.3 duration:=20.0     # 실차
시뮬 플랜트(sim_plant_wheel_node)는 드라이버처럼 50 Hz 로 /odom, /measured_wheel_speeds 를 낸다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('speed', default_value='0.3'),
        DeclareLaunchArgument('duration', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('pose_extrapolation', default_value='true',
                              description='묵은 포즈를 현재 틱으로 외삽 (노드 머리주석). false 는 비교 시험용.'),
        DeclareLaunchArgument('model_section', default_value='model_100hz', description='plant_wheel.yaml 의 모델 절.'),
        DeclareLaunchArgument('tuning_section', default_value='tuning_100hz'),
        DeclareLaunchArgument('log_prefix', default_value='wheel_cpp_sim'),
        Node(package='amr_mpc', executable='sim_plant_wheel_node', name='sim_plant_wheel_node', output='screen',
             condition=IfCondition(lc('sim')), parameters=[{'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc', executable='fake_reference_node', name='fake_reference_node', output='screen',
             parameters=[{'radius': lc('radius'), 'speed': lc('speed'), 'duration': lc('duration'),
                          'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc_cpp', executable='wheel_mpc_node', name='wheel_mpc_node', output='screen',
             parameters=[{'odom_topic': lc('odom_topic'), 'log_prefix': lc('log_prefix'),
                          'wheel_topic': '/encoder_wheel_speeds',
                          'tuning_section': lc('tuning_section'), 'model_section': lc('model_section'),
                          'pose_extrapolation': ParameterValue(lc('pose_extrapolation'), value_type=bool)}]),
    ])
