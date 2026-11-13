#!/usr/bin/env python3
"""AMCL 전역 측위로 cartographer 초기 위치를 자동으로 잡는다.

RViz "2D Pose Estimate" 를 손으로 찍는 대신 쓴다.

    ros2 launch amr_perception_bringup amcl_bootstrap.launch.py

전제 (autonomy.launch.py 를 쓰면 다 갖춰진다):
  /map                    nav2_map_server. cartographer 매핑 때 저장한 PGM.
  /scan                   앞뒤 라이다 병합
  odom -> base_footprint  EKF
  set_initial_pose_node   cartographer.launch.py 의 localization 모드가 띄운다

동작:
  1) amcl 이 뜨고 lifecycle_manager 가 activate
  2) amcl_bootstrap_node 가 /reinitialize_global_localization 호출
     -> 파티클이 자유공간 전체에 뿌려진다
  3) 로봇을 움직인다
       use_wiggle:=true  -> amcl_wiggle_node 가 20 cm 사각형을 두 바퀴 돈다
       use_wiggle:=false -> 조이스틱/텔레옵으로 직접 몇 미터 주행
  4) 파티클이 수렴하면 /initialpose 발행
  5) set_initial_pose_node 가 cartographer 궤적을 그 위치에서 재시작

제자리 회전만으로는 부족하다. 대칭적인 복도는 서로 다른 지점에서 본 관측이
쌓여야 모호성이 풀린다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('amcl_config')

    default_config = PathJoinSubstitution([
        FindPackageShare('amr_perception_bringup'), 'config', 'amcl.yaml',
    ])

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # amcl 은 lifecycle 노드다. activate 를 안 하면 프로세스는 떠 있는데
    # /amcl_pose 가 한 번도 안 나간다 (map_server 와 같은 함정).
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='amcl_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }],
    )

    # 수렴에 필요한 최소 이동을 자동으로 만든다. 게이트 입력 토픽에 바퀴
    # 명령을 쏘고 /mpc/arm 으로 arm 하며, /initialpose 가 나오면 물러난다.
    # ★ wheel_tracking 이 같은 입력 토픽에 0 을 계속 내므로 명령이 섞여
    #   덜컹거린다. publish_frequency 로 duty 를 확보하는 구성이다.
    wiggle = Node(
        package='amr_perception_bringup',
        executable='amcl_wiggle_node',
        name='amcl_wiggle_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_wiggle')),
    )

    bootstrap = Node(
        package='amr_perception_bringup',
        executable='amcl_bootstrap_node',
        name='amcl_bootstrap_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_bootstrap')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('amcl_config', default_value=default_config),
        DeclareLaunchArgument(
            'use_bootstrap', default_value='true',
            description=(
                'AMCL 수렴을 /initialpose 로 넘기는 노드. false 면 AMCL 만 '
                '띄우고 초기 위치는 직접 찍는다.'),
        ),
        DeclareLaunchArgument(
            'use_wiggle', default_value='false',
            description=(
                'AMCL 수렴용 최소 이동(20 cm 사각형 두 바퀴 = 경로 1.6 m)을 '
                '자동으로 만든다. 로봇 주변 반경 30 cm 가 비어 있어야 한다.'),
        ),
        amcl,
        lifecycle,
        wiggle,
        bootstrap,
    ])
