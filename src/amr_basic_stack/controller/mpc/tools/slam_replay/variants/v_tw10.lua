-- 2026-11-15 이전 설정 (Cartographer 기본 translation_weight 10) — 빠를 때 넘치던 판과 비교할 때
include "localization_torque_2d.lua"
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
return options
