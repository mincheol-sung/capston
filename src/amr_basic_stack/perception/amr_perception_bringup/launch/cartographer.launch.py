#!/usr/bin/env python3
"""Cartographer 2D — 매핑 / IMU+scan 매핑 / 측위 / 동시(SLAM) 통합 launch.

  ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping      map:=factory1
  ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping_imu_scan map:=factory1_imu
  ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=factory1
  ros2 launch amr_perception_bringup cartographer.launch.py mode:=slam         map:=factory1

┌ 모드 ────────────┬ lua ──────────────────┬ 하는 일 ─────────────────────────┐
│ mapping          │ mapping_hq_2d.lua     │ 맵 작성 전용. 맵 품질 우선.      │
│                  │                       │ RViz "2D Goal Pose" 로 저장.     │
│ mapping_imu_scan │ mapping_imu_scan_2d.lua│ scan+IMU만으로 맵 작성.          │
│                  │                       │ Cartographer가 odom을 구독 안 함.│
│ localization     │ localization_2d.lua   │ 저장된 pbstream 위 순수 측위.    │
│                  │                       │ RViz "2D Pose Estimate" 로 초기화│
│ slam             │ slam_2d.lua           │ 맵을 만들며 그 pose 로 주행.     │
│                  │                       │ 실시간성 우선. 저장도 가능.      │
└──────────────────┴───────────────────────┴──────────────────────────────────┘

TF 소유권
  map -> odom        ekf_carto (이 launch 가 띄우는 robot_localization)
  odom -> base_footprint  ekf_filter_node (localization.launch.py, 별도로 띄울 것)
  base_link -> 센서   URDF (robot_state_publisher)

cartographer 자체는 TF 를 발행하지 않는다(lua 의 publish_to_tf = false).
/tracked_pose -> pose_to_odom_node(공분산 주입) -> /tracked_pose/odom -> ekf_carto.

측위 모드의 재측위 경로
  /initialpose -> set_initial_pose_node -> Finish/StartTrajectory (cartographer)
                                        -> /ekf_carto/set_pose (EKF 상태 리셋)
  EKF 리셋이 빠지면 마할라노비스 게이트가 재측위 점프를 이상치로 버려서
  map -> odom 이 옛 위치에 남는다. set_initial_pose_node docstring 참조.

전제
  full_system 또는 bringup 이 실행 중이어야 한다
  (/scan_front_carto, /scan_rear_carto, /odom, /camera/imu,
   odom->base_footprint TF).
  slam_toolbox(slam.launch.py) 와 semantic_slam stub 은 /map 이 겹치므로 끌 것.

진단
  ros2 run amr_perception_bringup carto_debug --watch 2
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


MODES = {
    # mode          : (lua basename,          ekf yaml,                      기본 /map 토픽)
    # 2026-09-16: 기본을 mapping_hq_2d.lua 로 바꿨다. 같은 bag 을 두 lua 로
    # 재매핑한 A/B 에서 hq 가 모든 지표에서 이겼다(근거는 그 파일 주석).
    # mapping_2d.lua 는 이제 '매핑 프로파일' 이 아니라 6 개 lua 의 공통
    # 베이스다 — 직접 mode 로 쓰지 않는다.
    'mapping':       ('mapping_hq_2d.lua',    'ekf_carto_mapping.yaml',      '/map'),
    'mapping_imu_scan': ('mapping_imu_scan_2d.lua',
                         'ekf_carto_mapping.yaml', '/map'),
    'localization':  ('localization_2d.lua',  'ekf_carto_localization.yaml', '/map_carto'),
    'slam':          ('slam_2d.lua',          'ekf_carto_slam.yaml',         '/map'),
}


def default_maps_dir():
    """소스 트리의 planner/global_planner/maps 를 찾는다.

    finish_map 이 여기에 저장하면 global_planner 가 그 .yaml/.png 를 그대로
    읽으므로, 매핑 직후 경로 생성으로 넘어갈 수 있다.
    (global_line_generator.launch.py 와 같은 방식.)
    """
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        for candidate in (parent / 'planner' / 'global_planner' / 'maps',
                          parent / 'src' / 'amr_basic_stack' / 'planner'
                          / 'global_planner' / 'maps'):
            if candidate.is_dir():
                return str(candidate)
    return ('/home/smoc/2026amr_ws/src/amr_basic_stack/planner/'
            'global_planner/maps')


def launch_setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    mode = cfg('mode')
    if mode not in MODES:
        raise RuntimeError(
            f"mode 는 {sorted(MODES)} 중 하나여야 합니다 (받은 값: '{mode}')")
    lua_basename, ekf_yaml, default_map_topic = MODES[mode]
    # lua 를 통째로 바꿔 끼울 때 (예: 시뮬레이터용 mapping_sim_2d.lua). 비면 모드 기본값.
    if cfg('lua_basename'):
        lua_basename = cfg('lua_basename')

    config_dir = os.path.join(
        get_package_share_directory('amr_perception_bringup'), 'config')
    map_name = cfg('map')
    maps_dir = cfg('maps_dir')
    map_dir = os.path.join(maps_dir, map_name)
    use_sim_time = cfg('use_sim_time').lower() in ('true', '1', 'yes')
    use_dynamic_filter = IfCondition(LaunchConfiguration('use_dynamic_filter')).evaluate(context)
    scan_front = cfg('scan_front_topic')
    scan_rear = cfg('scan_rear_topic')
    filters = []
    if use_dynamic_filter:
        outputs = ('/scan_front_static', '/scan_rear_static')
        if any('/' + topic.lstrip('/') in outputs for topic in (scan_front, scan_rear)):
            raise RuntimeError('Dynamic filter source topics must differ from static outputs')
        for side, source, target in zip(('front', 'rear'), (scan_front, scan_rear), outputs):
            filters.append(Node(
                package='dynamic_filter', executable='dynamic_filter_node',
                name=f'dynamic_filter_{side}', output='screen',
                # Launch owns scan routing even if a custom YAML contains topic parameters.
                parameters=[cfg('dynamic_filter_config'), {'use_sim_time': use_sim_time,
                            'scan_input_topic': 'scan_in', 'scan_output_topic': 'scan_out'}],
                remappings=[('scan_in', source), ('scan_out', target)],
            ))
        scan_front, scan_rear = outputs

    map_topic = cfg('map_topic') or default_map_topic

    # ── cartographer_node 인자 ──
    carto_args = [
        '-configuration_directory', config_dir,
        '-configuration_basename', lua_basename,
        '-collect_metrics',          # carto_debug 의 /read_metrics 용
    ]

    pbstream = cfg('pbstream') or os.path.join(map_dir, f'{map_name}.pbstream')
    if mode == 'localization':
        if not os.path.exists(pbstream):
            raise RuntimeError(
                f'측위 모드에는 저장된 맵이 필요합니다. 없음: {pbstream}\n'
                f'  먼저 mode:=mapping 으로 맵을 만드세요.')
        carto_args += ['-load_state_filename', pbstream, '-load_frozen_state', 'true']
    elif mode == 'slam' and cfg('pbstream'):
        # slam 모드는 기본적으로 빈 맵에서 시작한다. pbstream 을 명시하면
        # 그 맵을 이어서 확장한다(lifelong SLAM).
        carto_args += ['-load_state_filename', pbstream, '-load_frozen_state', 'false']

    nodes = filters + [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=carto_args,
            # num_laser_scans=2이면 고정 구독명이 scan_1, scan_2가 된다.
            # 두 원본의 stamp/time_increment를 보존한 섹터 필터 출력을 사용한다.
            remappings=[
                ('scan_1', scan_front),
                ('scan_2', scan_rear),
                ('odom', cfg('odom_topic')),
                ('imu', cfg('imu_topic')),
            ],
        ),

        # 서브맵을 OccupancyGrid 로 변환. cartographer_node 자체는 /map 을 안 낸다.
        # 측위 모드에서도 frozen 궤적의 서브맵이 그대로 남아 사실상 전체 맵이
        # 나온다 — pure_localization_trimmer 는 새 궤적만 3 개로 제한한다.
        # (2026-09-15 실측: /map_carto 자유셀 92 m², map_server /map 은 78 m².)
        # 그래도 토픽을 /map_carto 로 분리하는 이유는 크기·origin 이 매 갱신마다
        # 바뀌어 map_server 의 고정 격자를 대체할 수 없기 때문이다.
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', cfg('resolution'),
                '-publish_period_sec', cfg('publish_period_sec'),
            ],
            remappings=[('map', map_topic)],
        ),

        # /tracked_pose (map -> tracking_frame) 를 base_link 기준으로 변환하고
        # 공분산을 채워 Odometry 로 발행. cartographer 는 공분산을 안 준다.
        Node(
            package='amr_perception_bringup',
            executable='pose_to_odom_node',
            name='pose_to_odom_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tracking_frame': cfg('tracking_frame'),
                'target_frame': cfg('base_frame'),
                'cov_x': float(cfg('cov_x')),
                'cov_y': float(cfg('cov_y')),
                'cov_yaw': float(cfg('cov_yaw')),
            }],
            remappings=[
                ('pose_in', '/tracked_pose'),
                ('odom_out', '/tracked_pose/odom'),
            ],
        ),

        # map -> odom TF 를 발행하는 상위 EKF. 모드별로 yaml 이 다르다.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_carto',
            output='screen',
            parameters=[os.path.join(config_dir, ekf_yaml),
                        {'use_sim_time': use_sim_time}],
            # set_pose 는 robot_localization 의 상태 강제 리셋 창구다. 리맵하지
            # 않으면 /set_pose 가 되어 odom 레벨 ekf_filter_node 와 이름이
            # 겹친다(둘 다 루트 네임스페이스). 측위 모드에서 set_initial_pose_node
            # 가 재측위 직후 이 토픽으로 초기 pose 를 밀어넣는다.
            remappings=[('odometry/filtered', '/odometry/map'),
                        ('set_pose', '/ekf_carto/set_pose')],
        ),
    ]

    if mode in ('mapping', 'mapping_imu_scan', 'slam'):
        nodes.append(Node(
            package='amr_perception_bringup',
            executable='finish_map',
            name='finish_map_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_name': map_name,
                'maps_dir': map_dir,
                'resolution': float(cfg('resolution')),
                'rebuild_package': cfg('rebuild_package'),
            }],
        ))

    if mode == 'localization':
        nodes.append(Node(
            package='amr_perception_bringup',
            executable='set_initial_pose_node',
            name='set_initial_pose_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_dir': config_dir,
                'config_base': lua_basename,
                'ekf_set_pose_topic': '/ekf_carto/set_pose',
                'map_frame': 'map',
            }],
        ))

    return nodes


NOTICE_ODOM_PRIOR = """
────────────────────────────────────────────────────────────────────────────
TODO(확인) — cartographer 의 오도메트리 prior 를 무엇으로 줄 것인가   (2026-09-03)

  지금 /odom (휠 원본) 을 쓴다. 이 선택을 실측으로 검증한 적이 없다.

  배경: 2026-09-03 에 ekf.yaml 이 휠의 절대 위치 x,y 를 융합하던 것을 고쳐
        (속도만 받도록) /odometry/filtered 가 자기일관적이 됐다. 이제
        ekf_carto_*.yaml 세 개가 쓰는 패턴과 같다.
        같은 날 실측:  휠 yaw 는 실제 회전의 65 % 만 본다 (총회전 66° vs 자이로 114°).
        /odom 의 pose 델타에는 그 축소된 회전이 실려 있다.

  괜찮을 수도 있는 이유: lua 가 use_imu_data = true 이고 tracking_frame 이 IMU
        프레임이라, cartographer 는 자세를 IMU 에서 직접 받는다. 오도메트리는
        주로 병진 prior 다. "회전은 IMU, 병진은 휠" 은 cartographer 정석 설정이다.

  판단 방법 (코드 수정 없이 A/B):
        ros2 launch amr_perception_bringup cartographer.launch.py \\
             mode:=localization map:=<맵> odom_topic:=/odometry/filtered
        같은 경로를 두 토픽으로 주행하고 비교:
          · ros2 run amr_perception_bringup carto_debug --watch 2
          · map -> odom 이 튀는 정도 (특히 제자리 회전·급선회 구간)
          · 재측위 수렴 시간

  끝나면: 더 나은 쪽을 default_value 로 박고 근거를 한 줄 남긴 뒤
          **이 알림 블록과 위의 TODO(확인) 주석을 지워라.**
          /odom 이 이기면 그것도 결론이다 — 지우고 이유를 적으면 된다.
────────────────────────────────────────────────────────────────────────────
"""


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='slam',
            description='mapping | mapping_imu_scan | localization | slam'),
        DeclareLaunchArgument(
            'map', default_value='my_map',
            description='맵 이름. <maps_dir>/<map>/<map>.{pbstream,png,yaml}'),
        DeclareLaunchArgument(
            'maps_dir', default_value=default_maps_dir(),
            description='맵 폴더들이 있는 상위 디렉터리(소스 트리)'),
        DeclareLaunchArgument(
            'pbstream', default_value='',
            description='불러올 pbstream 경로를 직접 지정(비우면 map 이름으로 유추). '
                        'slam 모드에서는 지정 시에만 기존 맵을 이어서 확장한다.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_dynamic_filter', default_value='false',
                              description='Mask confirmed tracks for Cartographer only.'),
        DeclareLaunchArgument('dynamic_filter_config', default_value=os.path.join(
            get_package_share_directory('dynamic_filter'), 'config', 'dynamic_filter.yaml'),
            description='Filter policy/radius, track topic, age and bounded TF wait limits.'),
        DeclareLaunchArgument(
            'lua_basename', default_value='',
            description='모드 기본 lua 대신 쓸 config/*.lua 파일명. 예: mapping_sim_2d.lua / '
                        'localization_sim_2d.lua (Isaac 창고, max_range 30). 비면 모드 기본값.'),

        DeclareLaunchArgument(
            'scan_front_topic', default_value='/scan_front_carto',
            description='시간 정보를 보존하고 섹터 마스킹한 전방 LaserScan.'),
        DeclareLaunchArgument(
            'scan_rear_topic', default_value='/scan_rear_carto',
            description='시간 정보를 보존하고 섹터 마스킹한 후방 LaserScan.'),
        # TODO(확인): 이 기본값이 최선인지 아직 안 재봤다. 아래 LogInfo 참조.
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='cartographer pose extrapolator 의 오도메트리 prior. '
                                          '기본은 휠 원본(/odom). /odometry/filtered 도 후보다 — '
                                          'TODO(확인) 참조.'),
        # 선언 뒤에 와야 한다 — launch 는 위에서부터 순서대로 실행하고,
        # 아직 선언 안 된 LaunchConfiguration 을 읽으면 그 자리에서 죽는다.
        # odom_topic 을 기본값이 아닌 값으로 주면 이 알림은 자동으로 안 나온다.
        LogInfo(
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('odom_topic'), "' == '/odom'"])),
            msg=NOTICE_ODOM_PRIOR),

        DeclareLaunchArgument('imu_topic', default_value='/camera/imu'),
        DeclareLaunchArgument('tracking_frame',
                              default_value='camera_imu_optical_frame',
                              description='lua 의 tracking_frame 과 반드시 일치'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='ekf_carto_*.yaml 의 base_link_frame 과 반드시 일치. '
                                          'URDF 루트가 base_footprint 다.'),

        DeclareLaunchArgument('map_topic', default_value='',
                              description='비우면 모드별 기본값(매핑·slam=/map, 측위=/map_carto)'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='lua 의 submaps.grid_options_2d.resolution 과 맞출 것'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),

        DeclareLaunchArgument('cov_x', default_value='0.01',
                              description='cartographer pose 공분산 x [m^2]. 작을수록 EKF 가 더 믿는다.'),
        DeclareLaunchArgument('cov_y', default_value='0.01'),
        DeclareLaunchArgument('cov_yaw', default_value='0.01',
                              description='cartographer pose 공분산 yaw [rad^2]'),

        DeclareLaunchArgument('rebuild_package', default_value='global_planner',
                              description='맵 저장 후 재설치할 패키지. 비우면 건너뜀.'),

        OpaqueFunction(function=launch_setup),
    ])
