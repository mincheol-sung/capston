#!/usr/bin/env python3
"""전체 인지 스택 통합 launch.

파이프라인:
  (센서) -> rgbd_detection -> /detections      ─┐
           lidar_cluster   -> /lidar_clusters  ─┴> camera_lidar_fusion
                                     ├-> /detections_fused -> detection_markers -> RViz
                                     ├-> /detections_tracking -> object_tracker
                                     └-> /lidar_obstacles (unmatched geometry)
    -> object_tracker -> /tracks + /tracked_object_modes (odom)
  SLAM-only dynamic filters are owned by cartographer.launch.py.
    -> rgbd_detection -> /semantic/observations -> semantic_slam -> /semantic/objects
    -> motion_predictor -> /predictions (CV/CT mixture)
       -> risk_model -> /risk

실행:
  ros2 launch amr_perception_bringup perception_stack.launch.py
연결성 확인:
  rqt_graph  (또는 ros2 topic list / ros2 node list)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_semantic_identity(context):
    if not LaunchConfiguration('semantic_map_id').perform(context).strip():
        raise RuntimeError('use_semantic_slam=true requires non-empty semantic_map_id')
    if (IfCondition(LaunchConfiguration('enable_cartographer_correction')).evaluate(context)
            and not LaunchConfiguration('semantic_graph_id').perform(context).strip()):
        raise RuntimeError(
            'enable_cartographer_correction=true requires non-empty semantic_graph_id')


def generate_launch_description():
    # yolo_model_path: OpenVINO 모델 폴더. 비면 sensor_fusion이 stub(빈 검출)로 동작.
    yolo_model = LaunchConfiguration('yolo_model_path')
    use_semantic_slam = LaunchConfiguration('use_semantic_slam')
    use_object_tracker = LaunchConfiguration('use_object_tracker')
    # 노드가 double 로 선언하므로 문자열 그대로 넘기면 타입 불일치로 죽는다.
    yolo_conf = ParameterValue(LaunchConfiguration('yolo_conf'), value_type=float)
    # C단계 실험 정의 파라미터. 기본값 = 노드 기본값이라 기존 실행은 그대로 동작한다.
    # launch 로 노출하는 이유: 실험 설정을 시작 시점에 못 박아야 재현이 된다.
    cam_pos_method = LaunchConfiguration('camera_position_method')
    depth_roi_ratio = ParameterValue(
        LaunchConfiguration('depth_roi_ratio'), value_type=float)
    min_valid_px = ParameterValue(
        LaunchConfiguration('min_valid_depth_pixels'), value_type=int)
    lid_pos_method = LaunchConfiguration('cluster_position_method')
    # 라이다 yaw 장착 오차 보정(rad). C2 실측 -2.212도. URDF 를 고치면 0.0 으로.
    lidar_yaw = ParameterValue(
        LaunchConfiguration('lidar_yaw_correction'), value_type=float)
    forward_unmatched_lidar = ParameterValue(
        LaunchConfiguration('forward_unmatched_lidar_to_fused'), value_type=bool)

    # 트래커 프로세스 잡음 sigma_a [m/s^1.5]. 유일한 미실측 설계값이라 launch 로
    # 노출한다 — 나머지 문턱은 chi^2 분위수라 실험으로 정할 것이 없다.
    tracker_sigma_a = ParameterValue(
        LaunchConfiguration('tracker_sigma_a'), value_type=float)
    tracker_frame = LaunchConfiguration('tracker_frame')

    other_modules = [
        ('risk_model', 'risk_model_node'),
    ]
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    tracker_config = PathJoinSubstitution([
        FindPackageShare('object_tracker'), 'config', 'tracker.yaml'])
    predictor_config = PathJoinSubstitution([
        FindPackageShare('motion_predictor'), 'config', 'predictor.yaml'])
    timing_config = LaunchConfiguration('observation_timing_config')
    return LaunchDescription([
        DeclareLaunchArgument('observation_timing_config', default_value=PathJoinSubstitution([
            FindPackageShare('amr_perception_bringup'), 'config', 'observation_timing.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('yolo_model_path', default_value='',
                              description='YOLO OpenVINO 모델 폴더 경로. 비면 stub.'),
        DeclareLaunchArgument('use_semantic_slam', default_value='false',
                              description='Start persistent object mapper; requires semantic_map_id.'),
        DeclareLaunchArgument('semantic_map_id', default_value=''),
        DeclareLaunchArgument('semantic_config', default_value=IfElseSubstitution(
            use_semantic_slam, PathJoinSubstitution([
                FindPackageShare('semantic_slam'), 'config', 'semantic.yaml']), '')),
        DeclareLaunchArgument('semantic_database_path',
                              default_value='~/.ros/amr/semantic_objects.json'),
        DeclareLaunchArgument('semantic_graph_id', default_value=''),
        DeclareLaunchArgument('enable_cartographer_correction', default_value='false'),
        DeclareLaunchArgument('publish_semantic', default_value='false',
                              description='Fan out the existing inference to semantic observations.'),
        DeclareLaunchArgument('inference_hz', default_value='0.0',
                              description='0 = unchanged rate; semantic trial recommends 5 Hz.'),
        DeclareLaunchArgument('use_object_tracker', default_value='true',
                              description='Start object_tracker_node.'),
        # 거리별 신뢰도 측정 시 낮춰서(예 0.05) 절단 없는 confidence 분포를 받는다.
        DeclareLaunchArgument('yolo_conf', default_value='0.4',
                              description='YOLO score threshold.'),
        # 카메라 객체 대표점(object representative position) 산출 방식.
        # roi_median_3d = bbox 중앙 ROI 유효 depth 전체의 좌표별 median (C2 실험 정의)
        # center_patch  = bbox 중심 창의 depth median 1점 (C1 실험 방식, 비교용)
        DeclareLaunchArgument('camera_position_method', default_value='roi_median_3d',
                              description='roi_median_3d | center_patch'),
        DeclareLaunchArgument('depth_roi_ratio', default_value='0.5',
                              description='bbox 중앙 ROI 비율 (0, 1].'),
        DeclareLaunchArgument('min_valid_depth_pixels', default_value='20',
                              description='ROI 내 유효 depth 픽셀 하한.'),
        DeclareLaunchArgument('lidar_yaw_correction', default_value='-0.043670',
                              description='라이다 yaw 보정(rad). URDF 수정 후 0.0.'),
        DeclareLaunchArgument(
            'forward_unmatched_lidar_to_fused', default_value='false',
            description='Legacy/debug: also include anonymous LiDAR in /detections_fused.'),
        DeclareLaunchArgument('cluster_position_method', default_value='mean_centroid',
                              description='mean_centroid | median | nearest_point'),
        DeclareLaunchArgument('tracker_sigma_a', default_value='1.0',
                              description='트래커 CWNA 가속도 세기 [m/s^1.5]. 미실측 임시값.'),
        DeclareLaunchArgument('tracker_frame', default_value='odom',
                              description='추적 프레임. map 은 AMCL 점프가 속도로 샌다.'),
        OpaqueFunction(function=_validate_semantic_identity,
                       condition=IfCondition(use_semantic_slam)),
        Node(package='sensor_fusion', executable='rgbd_detection_node',
             name='rgbd_detection_node', output='screen',
             parameters=[sim, timing_config, {
                 'yolo_model_path': yolo_model, 'yolo_conf': yolo_conf,
                 'camera_position_method': cam_pos_method,
                 'depth_roi_ratio': depth_roi_ratio,
                 'min_valid_depth_pixels': min_valid_px,
                 'publish_semantic': ParameterValue(
                     LaunchConfiguration('publish_semantic'), value_type=bool),
                 'inference_hz': ParameterValue(
                     LaunchConfiguration('inference_hz'), value_type=float)}],
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        # RViz 마커. 입력은 트래커와 같은 semantic /detections_fused 다.
        Node(package='sensor_fusion', executable='detection_markers',
             name='detection_markers', output='screen',
             parameters=[sim, {'detections_topic': '/detections_fused'}]),
        # 라이다 단독 객체 검출(ABD 클러스터링) -> /lidar_clusters
        # 융합의 LiDAR 측 입력. 없으면 Camera X + LiDAR O 케이스가 불가능.
        Node(package='sensor_fusion', executable='lidar_cluster_node',
             name='lidar_cluster_node', output='screen',
             parameters=[sim, timing_config, {'cluster_position_method': lid_pos_method}],
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        # association/fusion -> semantic /detections_fused. 미매칭 LiDAR는
        # /lidar_obstacles로 분리되어 semantic tracker를 오염시키지 않는다.
        # 기본 파라미터는 C2 실측값(EXPERIMENT_C2.md). /detections 는 순수 카메라
        # 관측으로 남는다 — measure_c 가 두 센서를 독립으로 봐야 하기 때문.
        Node(package='sensor_fusion', executable='camera_lidar_fusion_node',
             name='camera_lidar_fusion_node', output='screen',
             parameters=[sim, timing_config, {
                 'lidar_yaw_correction': lidar_yaw,
                 'forward_unmatched_lidar_to_fused': forward_unmatched_lidar,
             }],
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        # 동기화된 카메라/융합/LiDAR-only 검출 -> 트랙. semantic-only
        # /detections_fused 계약은 기존 소비자를 위해 유지한다.
        # 트래커가 예상치 못하게 종료되면 prediction 사슬 전체가 멈추므로
        # 최대 3회만 재시작한다.
        Node(package='object_tracker', executable='object_tracker_node',
             name='object_tracker_node', output='screen',
             parameters=[sim, tracker_config, timing_config,
                         {'detections_topic': '/detections_tracking',
                          'track_frame': tracker_frame,
                          'sigma_a': tracker_sigma_a}],
             condition=IfCondition(use_object_tracker),
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        # IMM posterior를 measurement update 없이 CV/CT별로 미래 전파한다.
        Node(package='motion_predictor', executable='motion_predictor_node',
             name='motion_predictor_node', output='screen',
             parameters=[sim, predictor_config, {'sigma_a': tracker_sigma_a}],
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        # CV/CT 평균 경로와 95% 위치 공분산 타원 -> RViz MarkerArray.
        Node(package='motion_predictor', executable='prediction_markers_node',
             name='prediction_markers_node', output='screen',
             parameters=[sim, predictor_config],
             respawn=True, respawn_delay=2.0, respawn_max_retries=3),
        Node(package='semantic_slam', executable='semantic_slam_node',
             name='semantic_slam_node', output='screen',
             parameters=[LaunchConfiguration('semantic_config'), sim, {
                 'map_id': ParameterValue(LaunchConfiguration('semantic_map_id'), value_type=str),
                 'database_path': ParameterValue(
                     LaunchConfiguration('semantic_database_path'), value_type=str),
                 'cartographer_graph_id': ParameterValue(
                     LaunchConfiguration('semantic_graph_id'), value_type=str),
                 'enable_cartographer_correction': ParameterValue(
                     LaunchConfiguration('enable_cartographer_correction'), value_type=bool)}],
             condition=IfCondition(use_semantic_slam)),
    ] + [
        Node(package=pkg, executable=exe, name=exe, output='screen', parameters=[sim])
        for pkg, exe in other_modules
    ])
