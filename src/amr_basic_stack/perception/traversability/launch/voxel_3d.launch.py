#!/usr/bin/env python3
"""3D voxel 파이프라인 — D455 전처리 + OctoMap + STVL (설계문서 §3.3).

    D455 /camera/depth/color/points
      -> depth_preprocess_node
           /traversability/cloud_filtered  (전체 높이, odom)
           /traversability/cloud_band      (z-band 만, base_footprint)
      -> octomap_server        /octomap_full, /occupied_cells_vis_array, /projected_map
      -> nav2_costmap_2d+STVL  /costmap/costmap, /costmap/voxel_grid

두 3D 표현은 **같은 점군**을 받는다. §3.3 의 비교 실험 조건이다. 한 번에 하나만
띄워 CPU·메모리를 측정하려면 use_octomap / use_stvl 로 끄면 된다.

전제:
  * octomap_server 와 STVL 이 설치돼 있을 것 (2026-09-15 부터 apt 정식 설치라
    /opt/ros/jazzy 소싱으로 충분하다. 그 전에는 vendor/ 를 따로 소싱해야 했다)
  * odom -> base_footprint TF (EKF), base_footprint -> camera_* TF (URDF)
  * /camera/depth/color/points 발행 (pointcloud.enable:=true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _config(name):
    return PathJoinSubstitution(
        [FindPackageShare('traversability'), 'config', name])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_octomap = LaunchConfiguration('use_octomap')
    use_stvl = LaunchConfiguration('use_stvl')

    depth_preprocess = Node(
        package='traversability',
        executable='depth_preprocess_node',
        name='depth_preprocess_node',
        output='screen',
        parameters=[
            LaunchConfiguration('depth_config'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('use_depth_preprocess')),
    )

    # 공식 octomap_server 바이너리. /cloud_in 을 전처리 출력으로 remap 한다.
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            LaunchConfiguration('octomap_config'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', LaunchConfiguration('filtered_cloud_topic'))],
        condition=IfCondition(use_octomap),
    )

    # STVL 은 Layer 플러그인이라 nav2_costmap_2d 독립 노드 안에서 돈다.
    # 경로 계획기는 띄우지 않는다.
    stvl_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[
            LaunchConfiguration('stvl_config'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_stvl),
    )

    # costmap 은 lifecycle 노드다. configure/activate 를 해주지 않으면
    # 프로세스는 떠 있는데 아무것도 발행하지 않는다.
    stvl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='costmap_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['costmap'],
            'bond_timeout': 0.0,
        }],
        condition=IfCondition(use_stvl),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_depth_preprocess', default_value='true'),
        DeclareLaunchArgument(
            'use_octomap', default_value='true',
            description='OctoMap 3D 점유 표현 (설계문서 §3.3 후보 A)'),
        DeclareLaunchArgument(
            'use_stvl', default_value='false',
            description=(
                'STVL 3D 점유 표현 (설계문서 §3.3 후보 B). 기본 off — '
                '§3.3 은 CPU·메모리를 한 방식씩 실행해 측정하라고 정했다.')),
        DeclareLaunchArgument(
            'filtered_cloud_topic',
            default_value='/traversability/cloud_filtered'),
        DeclareLaunchArgument(
            'depth_config', default_value=_config('depth_preprocess.yaml')),
        DeclareLaunchArgument(
            'octomap_config', default_value=_config('octomap.yaml')),
        DeclareLaunchArgument(
            'stvl_config', default_value=_config('stvl_costmap.yaml')),
        depth_preprocess,
        octomap,
        stvl_costmap,
        stvl_lifecycle,
    ])
