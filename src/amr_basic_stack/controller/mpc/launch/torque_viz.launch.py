#!/usr/bin/env python3
"""실차 토크 실험 RViz 화면 (랩 PC 에서 띄운다, 2026-11-16).

  ros2 launch amr_mpc torque_viz.launch.py                (= tools/amr_aliases.sh 의 amrviz)

  띄우는 것
    torque_viz_node : /state_estimator/odom → TF map → base_footprint, 지나간 경로 /torque_viz/trail (무장 때 비움)
    rviz2           : rviz/torque_real.rviz
                      지도 /map_carto · 로봇 모델 · 라이다 /scan_front /scan_rear · 8자 기준 /figure8/path ·
                      MPC 기준 지평 · 지나간 경로 · 도구 "2D Pose Estimate" → /initialpose
                      (측위 launch 의 set_initial_pose_node 가 받아 Cartographer 궤적을 그 자리에서 다시 시작한다)

  전제 — NUC 에서 A(센서)·B(측위)·C(CAN + 추정기, 필요하면 MPC)가 떠 있고, 두 컴퓨터가 서로 보인다:
    같은 ROS_DOMAIN_ID(81), 같은 RMW(rmw_cyclonedds_cpp), 서로의 IP 가 ROS_STATIC_PEERS 에 (NUC 은 LOCALHOST 범위).
    로봇 모델의 메시(package://amr_description/...)는 이 PC 에 amr_description 이 빌드돼 있어야 보인다.

  인자
    tf:=false    TF 를 내지 않는다 — 시뮬(plant:=sim)은 시뮬 플랜트가 이미 map → base_footprint 를 낸다
    rviz:=false  표시 노드만 (RViz 를 따로 띄울 때)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("amr_mpc")
    return LaunchDescription([
        DeclareLaunchArgument("tf", default_value="true",
                              description="map → base_footprint TF 발행. 시뮬에서는 false (시뮬 플랜트가 낸다)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="amr_mpc", executable="torque_viz_node", name="torque_viz_node", output="screen",
             parameters=[{"publish_tf": ParameterValue(LaunchConfiguration("tf"), value_type=bool)}]),
        Node(package="rviz2", executable="rviz2", name="rviz2_torque_real", output="log",
             arguments=["-d", os.path.join(share, "rviz", "torque_real.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
