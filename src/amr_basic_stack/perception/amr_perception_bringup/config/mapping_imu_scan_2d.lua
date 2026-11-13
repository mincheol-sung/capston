-- ============================================================================
-- mapping_imu_scan_2d.lua — 휠 오도메트리를 사용하지 않는 2D 매핑.
--
-- mapping_2d.lua 의 스캔 매칭·IMU·포즈그래프 튜닝은 그대로 재사용하되,
-- Cartographer 입력에서 /odom 을 완전히 제외한다. 따라서 이 모드의 trajectory는
--   /scan_front_carto, /scan_rear_carto
--               : timing을 보존하고 자기관측 섹터만 가린 전/후방 RPLIDAR S2
--   /camera/imu : D455 원본 IMU (camera_imu_optical_frame)
-- 두 센서만으로 추정된다.
--
-- 주의: full_system 의 하위 EKF가 odom -> base_footprint TF를 만들기 위해
-- /odom 을 사용하는 것은 별개다. 여기서 끄는 것은 cartographer_node의
-- OdometryData 구독과 내부 pose extrapolator/pose graph 입력뿐이다.
-- ============================================================================

include "mapping_2d.lua"

-- ComputeExpectedSensorIds 에서 ODOMETRY 센서가 빠지므로 cartographer_node가
-- /odom 을 구독하지 않는다. 이동량은 scan matching, 회전·중력 정렬은 IMU가
-- 담당한다.
options.use_odometry = false

return options
