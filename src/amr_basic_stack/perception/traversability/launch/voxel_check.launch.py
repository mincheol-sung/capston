#!/usr/bin/env python3
"""내일 아침 확인용 — D455 z-band voxel 이 RViz2 에 제대로 나오는지 본다.

    source /opt/ros/jazzy/setup.bash
    source ~/2026amr_ws/install/setup.bash
    ros2 launch traversability voxel_check.launch.py

띄우는 것:
  robot_state (URDF -> TF)          base_footprint -> camera_* 가 필요하다
  base + EKF                        odom -> base_footprint TF
  sensors                           라이다 2 + D455 (점군 on, decimation on)
  depth_preprocess_node             자기관측 제거 -> 거리 -> 다운샘플 -> z-band
  octomap_server                    3D voxel + /occupied_cells_vis_array
  traversability_node               2D 3-state 격자 (LiDAR + D455 합집합)
  rviz2                             rviz/voxel_check.rviz

무엇을 보면 되나:
  1. "Cloud z-band [0.033, 0.31]" 초록 점이 바닥에 깔리지 **않아야** 한다.
     깔린다면 z_min 이 낮거나 카메라 장착 높이가 URDF 와 다른 것이다.
  2. "Octomap Voxels (z 색)" 이 물체 높이에 따라 다른 색으로 쌓여야 한다.
     RViz 를 옆에서(Pitch 낮게) 봐야 z 분포가 보인다.
  3. 상자를 앞에 놓으면 그 높이만큼 voxel 기둥이 서고, 치우면 사라져야 한다.
     (사라지는 데 시간이 걸린다 — 설계문서 §3.3 의 log-odds 표대로 관통 miss
      관측이 약 9회 필요하다.)

기본은 mock_mode:=false (실차). 하드웨어 없이 배관만 보려면
  ros2 launch traversability voxel_check.launch.py use_robot:=false
로 띄우고 tools/fake_depth_cloud.py 를 돌리면 된다.

STVL 은 기본 off 다. 설계문서 §3.3 은 CPU·메모리를 한 방식씩 실행해 측정하라고
정했다. 비교하려면 use_octomap:=false use_stvl:=true 로 한 번 더 띄운다.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, relative, arguments=None, condition=None):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', relative]))
    kwargs = {}
    if arguments:
        kwargs['launch_arguments'] = arguments.items()
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(source, **kwargs)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot = LaunchConfiguration('use_robot')
    mock_mode = LaunchConfiguration('mock_mode')

    robot_state = _include('amr_description', 'robot_state.launch.py',
                           condition=IfCondition(use_robot))

    # EKF 가 odom -> base_footprint TF 를 소유한다. roboclaw 의 TF 는 끈다.
    base = _include('amr_bringup', 'base.launch.py',
                    {'mock_mode': mock_mode, 'publish_tf': 'false'},
                    condition=IfCondition(use_robot))

    localization = _include('amr_perception_bringup', 'localization.launch.py',
                            condition=IfCondition(use_robot))

    sensors = _include(
        'amr_bringup', 'sensors.launch.py',
        {
            # 점군을 켜야 D455 3D 가 나온다. 대역폭 때문에 decimation 도 같이.
            'depth_camera_pointcloud_enable': 'true',
            'depth_camera_decimation_enable': 'true',
            'depth_camera_decimation_magnitude': LaunchConfiguration(
                'decimation_magnitude'),
            'use_depth_camera': 'true',
        },
        condition=IfCondition(use_robot))

    voxel_3d = _include(
        'traversability', 'voxel_3d.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_octomap': LaunchConfiguration('use_octomap'),
            'use_stvl': LaunchConfiguration('use_stvl'),
        })

    # 2D 3-state 격자. /map 이 없어도 뜨고, 있으면 기준 맵을 받아 쓴다.
    traversability = Node(
        package='traversability',
        executable='traversability_node',
        name='traversability_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('traversability'),
                                  'config', 'traversability.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('use_traversability')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_voxel_check',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('traversability'), 'rviz', 'voxel_check.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_robot', default_value='true',
            description='로봇 하드웨어 스택(URDF·base·EKF·센서) 기동'),
        DeclareLaunchArgument(
            'mock_mode', default_value='false',
            description='true 면 roboclaw 없이 가짜 오도메트리'),
        DeclareLaunchArgument('use_traversability', default_value='true'),
        DeclareLaunchArgument('use_octomap', default_value='true'),
        DeclareLaunchArgument('use_stvl', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'decimation_magnitude', default_value='3',
            description='depth 다운샘플 배율. 대역폭이 남으면 2 로 낮춰도 된다'),
        robot_state,
        base,
        localization,
        sensors,
        voxel_3d,
        traversability,
        rviz,
    ])
