-- ============================================================================
-- mapping_sim_2d.lua — Isaac Sim 창고용 매핑. mapping_2d.lua 위에 범위만 넓힌다.
--
-- 왜 따로 두나 (2026-09-14 실측, isaac_warehouse):
--   창고는 열린 바닥이라 /scan 의 거리 분포가 min 9.3 / median 17 / p90 25 m 다.
--   max_range 20 이면 tour 중 열린 구간의 노드에 점이 1~30개만 남고(로그의
--   "Node (0, 980) with 5 points"), 측위 때 그 희박한 옛 노드가 새 submap 에
--   맞춰지며 정지 상태에서도 57 cm / 21 cm 점프를 만들었다.
--   Isaac RTX lidar 는 range_max 30 m 를 실제로 돌려주므로 30 으로 올린다.
--   실차(RPLidar S2, 실내 건물)는 mapping_2d.lua 의 20 을 그대로 쓴다.
-- ============================================================================
include "mapping_2d.lua"

TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.
-- 먼 벽까지 free space 를 새기면 지도가 커지지만 열린 바닥에서 위치 구속이 생긴다.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.

return options
