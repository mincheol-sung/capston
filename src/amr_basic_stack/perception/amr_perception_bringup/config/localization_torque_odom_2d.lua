-- ============================================================================
-- localization_torque_odom_2d.lua — 토크 MPC 실험용 측위, 휠 오돔을 **켠** 판.
--
-- localization_torque_2d.lua(외삽 끔, 휠 오돔 끔)와 다른 것은 use_odometry 하나뿐이다.
-- 언제 쓰나: 기준물을 둔 8자 트랙에서도 휠 오돔 없이 측위가 미끄러질 때만.
--   기본은 끈 판을 쓴다 (controller/mpc/config/torque_estimator.yaml 의 slam.use_wheel_odom: false).
--
-- 휠 오돔은 AK40 바퀴 속도로 만든 /wheel_odom (amr_mpc wheel_odometry_node) 이다.
--   cartographer.launch.py 에 odom_topic:=/wheel_odom 으로 준다.
-- 휠 오돔이 쓰이는 곳은 로컬 스캔 매칭의 초기 짐작뿐이다 (포즈 그래프의 오돔 가중은
-- mapping_2d.lua 에서 이미 0). 2026-11-11 bag replay: 끄면 통로꼴 구간에서 30 s 에 약 30 cm
-- 미끄러졌고, 켜면 5 cm 이내였다.
--
-- ★ 주의 (미검증): Ceres 정합이 초기 짐작 쪽으로 끌리는 가중이 크다
--   (ceres_scan_matcher.translation_weight 10, rotation_weight 10). 슬립 중에는 휠 오돔
--   짐작이 틀리므로 결과 포즈가 휠 오돔 쪽으로 일부 끌려 슬립이 작게 측정될 수 있다.
--   이 판을 쓰게 되면 translation_weight 를 낮추는 것을 함께 보고, 일부러 슬립을 낸
--   실차 시험으로 확인한다 (torque_mpc_todo.md "Cartographer에 휠 오돔을 넣을 것인가").
-- ============================================================================

include "localization_torque_2d.lua"

options.use_odometry = true

return options
