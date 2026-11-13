-- ============================================================================
-- mapping_2d.lua — Cartographer 2D **공통 베이스**. 직접 쓰지 않는다.
--
-- ★ 2026-09-16: 이 파일은 더 이상 매핑 프로파일이 아니다.
--   mode:=mapping 의 기본값은 mapping_hq_2d.lua 다. 같은 bag 을 두 lua 로
--   재매핑한 A/B 에서 hq 가 흐릿/점유 -9.6 %, 조각 -12 %, constraint +42.6 %
--   로 이겼다(근거 표는 mapping_hq_2d.lua 주석).
--   여기는 아래 다섯 파일이 include 하는 센서·프레임·튜닝의 공통분모로만
--   남긴다 — 그래서 지우면 안 된다:
--     mapping_hq_2d.lua  localization_2d.lua  slam_2d.lua
--     mapping_imu_scan_2d.lua  mapping_sim_2d.lua
--   (localization_sim_2d.lua 는 localization_2d.lua 를 거쳐 간접 의존)
--
--   값을 바꾸면 매핑·측위·SLAM 이 **전부** 같이 바뀐다. 한 모드만 바꾸려면
--   그 모드의 파일에서 덮어쓸 것.
--
-- ── 센서 입력 ───────────────────────────────────────────────────────────
--   /scan_front_carto, /scan_rear_carto
--                원본 LaserScan의 stamp/scan_time/time_increment를 보존하고
--                자기관측 섹터만 마스킹한 Cartographer 전용 입력
--   /odom        roboclaw_mecanum_node 휠 오도메트리(raw), 50Hz
--   /camera/imu  D455 내장 IMU(gyro+accel). frame=camera_imu_optical_frame, 200Hz
--
-- ★ 두 LaserScan을 따로 넣는 이유
--   회전식 라이다 두 대의 비동기 포인트 시각은 하나의 LaserScan
--   time_increment로 표현할 수 없다. 기존 dual_scan_merger는 100ms에 걸쳐
--   취득한 포인트를 time_increment=0으로 만들어 주행 중 motion distortion을
--   만들었다. scan_sector_filter가 각 원본의 시간/빔 순서를 그대로 보존하고
--   front [-95,95] / rear [85,-85] 섹터 밖만 inf로 마스킹한다.
--   cartographer는 scan_1, scan_2 두 센서를 직접 collate/deskew한다.
--
-- ── TF 소유권 ───────────────────────────────────────────────────────────
--   map -> odom        ekf_carto (robot_localization, ekf_carto_*.yaml)
--   odom -> base_footprint  ekf_filter_node (기존 localization.launch.py)
--   base_link -> 센서   URDF (robot_state_publisher)
--
--   cartographer 는 TF 를 직접 발행하지 않는다(publish_to_tf = false).
--   대신 /tracked_pose 를 내고, pose_to_odom_node 가 공분산을 채워
--   /tracked_pose/odom 으로 바꾸면 ekf_carto 가 그걸 융합해 map->odom 을 낸다.
--   이렇게 하면 (1) 공분산을 사람이 직접 넣을 수 있고 (2) 루프클로저 점프가
--   EKF 를 거치며 완만해진다.
-- ============================================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",

  -- IMU 를 쓰면 tracking_frame 은 IMU 가 실제로 놓인 프레임이어야 한다.
  -- cartographer 는 IMU 의 회전 오프셋만 TF 로 보정하고 병진 오프셋은
  -- 보정하지 못한다(레버암 가속도 미모델링).
  tracking_frame = "camera_imu_optical_frame",

  -- ── TF 발행 ──
  -- publish_to_tf = false 면 TF 를 하나도 내보내지 않는다. 다만 published_frame
  -- 이 무의미해지는 건 아니다. /tracked_pose 발행을 포함한 publish 블록 전체가
  -- published_frame -> tracking_frame TF 조회 성공에 걸려 있으므로, 반드시
  -- 실재하는 프레임이어야 한다.
  publish_to_tf = false,
  published_frame = "base_link",
  odom_frame = "odom",          -- provide_odom_frame = true 일 때만 의미 있음
  provide_odom_frame = false,

  publish_tracked_pose = true,  -- 기본값 false. 이 스택의 핵심 출력이다.
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,   -- 평면 주행 AMR

  -- use_odometry = true 로 두되, 포즈그래프의 오도메트리 잔차 가중치는 아래에서
  -- 0 으로 만든다. 이러면 휠 오도메트리는 pose extrapolator 의 속도 예측
  -- (스캔매칭 초기값)에만 쓰이고, 백엔드 최적화는 오염시키지 않는다.
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 2,                   -- timing 보존 front/rear 두 입력
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,   -- 포인트별 time_increment는 그대로 유지됨
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- ─────────────────────────────────────────────────────────────────────────
-- 프론트엔드: 스캔 -> 서브맵
-- ─────────────────────────────────────────────────────────────────────────
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- RPLiDAR/scan_merge.yaml(range_min 0.15 / range_max 30)과 정합.
-- max_range 는 20m 로 제한한다. S2 는 30m 까지 보지만 원거리 반사는 각도
-- 해상도가 벌어져 노이즈 비중이 커지고 서브맵을 흐린다.
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- 앞/뒤가 각각 한 바퀴를 보내며 range data collator가 같은 시간대로 묶는다.
-- 2 = 앞+뒤 두 장을 합쳐 360° 로 scan match 한다 (1 이면 한 장 = 반쪽 시야,
-- 한 match 의 시야 median 145°). 2026-09-18 map_v2_20260916_005042 offline
-- 2x2 (mapping_hq_2d.lua): 겹벽 21.4 → 11.7 %, 흐릿/점유 1.419 → 0.602,
-- constraint 26.6만 → 3.0만, RSS 2.0 → 0.39 GB, 19 → 3 분.
-- 이 파일을 include 하는 모든 lua(mapping_hq/localization/slam/imu_scan/sim)에 적용된다.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 20.

-- ★ 메카넘 대응 1
-- 휠 오도메트리 yaw 는 롤러 슬립으로 부정확하다(ekf.yaml 도 같은 이유로 odom 의
-- yaw/vyaw 를 제외했다). 초기 추정이 어긋난 채 ceres 로 넘어가면 국소최적에
-- 빠지므로, 실시간 상관 스캔매처로 넓은 각도 구간을 먼저 훑어 초기값을 잡는다.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ceres 기본값(1 / 10 / 40) 유지. 매핑 중에는 IMU+휠 예측을 어느 정도 믿는 편이
-- 서브맵이 매끄럽다. 자유도를 더 주고 싶으면 rotation_weight 를 낮춘다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- 매핑은 노드를 촘촘히 남길수록 맵이 좋다(대신 CPU/메모리를 더 쓴다).
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45   -- 12Hz x 45 ≈ 약 4초 분량
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- ─────────────────────────────────────────────────────────────────────────
-- 백엔드: 포즈그래프 / 루프클로저
-- ─────────────────────────────────────────────────────────────────────────
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- 탐색창을 기본값(7m)보다 좁힌다. 공장처럼 통로가 반복되는 환경에서는 넓은
-- 창이 엉뚱한 곳과 매칭돼 맵이 접히는 사고를 낸다. 드리프트가 커서 루프가
-- 안 닫히면 이 값을 키울 것.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- ★ 메카넘 대응 2
-- 휠 오도메트리를 포즈그래프 구속에서 완전히 뺀다. 롤러 슬립 때문에 노드 간
-- 상대 구속으로 쓰기에는 신뢰도가 너무 낮고, 강하게 묶이면 루프클로저 보정을
-- 방해한다. use_odometry = true 이므로 pose extrapolator 에는 계속 쓰인다.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.

return options
