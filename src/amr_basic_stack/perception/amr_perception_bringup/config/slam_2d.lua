-- ============================================================================
-- slam_2d.lua — 매핑과 측위를 동시에 (온라인 SLAM).
--
-- mapping_2d.lua 를 베이스로 하고 달라지는 것만 덮어쓴다.
--
-- 매핑 모드와 뭐가 다른가:
--   매핑 모드는 "좋은 맵을 만드는 것"이 목적이라 노드를 촘촘히 남기고
--   최적화를 크게 몰아서 한다(optimize_every_n_nodes = 100). 그 사이 pose 는
--   가끔 크게 튄다. 주행에 쓰기엔 부담스럽다.
--
--   이 모드는 "지금 이 pose 로 주행하면서 맵도 같이 만드는 것"이 목적이라
--   최적화를 자주·조금씩 돌려 pose 를 빨리 수렴시키고, 노드는 성기게 남겨
--   CPU 를 아낀다. 맵 정밀도는 매핑 모드보다 조금 떨어진다.
--
-- 사전 맵 없이 빈 상태로 시작하는 게 기본이다. -load_state_filename 을 주면
-- 기존 맵을 이어서 확장한다(lifelong SLAM). finish_map 으로 저장 가능.
-- ============================================================================

include "mapping_2d.lua"

-- 측위 지연을 줄인다.
options.pose_publish_period_sec = 5e-3
options.trajectory_publish_period_sec = 30e-3

MAP_BUILDER.num_background_threads = 6

-- ── 프론트엔드 ──
-- 노드를 성기게 남겨 백엔드 부하를 줄인다(매핑: 0.1m / 0.2도).
-- 주행 중에는 맵 밀도보다 실시간성이 우선이다.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- 스캔 정합 자유도를 매핑(40)과 측위(1) 중간에 둔다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.

-- ── 백엔드 ──
-- 자주 조금씩 최적화한다(매핑: 100). 한 번에 크게 튀지 않아 주행이 안정된다.
POSE_GRAPH.optimize_every_n_nodes = 30

-- 루프클로저 판정을 매핑보다 보수적으로. 주행 중 잘못 닫히면 로봇이 순간이동한다.
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

-- -load_state_filename 으로 기존 맵을 얹었을 때만 의미 있다. 반복 통로에서의
-- 순간이동을 막는다(localization_2d.lua 와 같은 이유).
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.global_constraint_search_after_n_seconds = 1000.

return options
