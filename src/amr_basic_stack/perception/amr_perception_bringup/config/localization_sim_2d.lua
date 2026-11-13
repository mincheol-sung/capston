-- ============================================================================
-- localization_sim_2d.lua — Isaac Sim 창고용 측위. localization_2d.lua 위에 덮어쓴다.
-- 지도는 mapping_sim_2d.lua(max_range 30) 로 만든 것이어야 한다. 근거는 그 파일 머리주석.
-- ============================================================================
include "localization_2d.lua"

TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.

-- 잘못된 전역 constraint 를 거르는 문턱. mapping_2d.lua 는 0.62 / 0.75.
-- 열린 공간에서 점이 적은 노드는 낮은 점수로도 붙으므로 두 단계 올린다.
POSE_GRAPH.constraint_builder.min_score = 0.75
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85

return options
