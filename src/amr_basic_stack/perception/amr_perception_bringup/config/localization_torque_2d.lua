-- ============================================================================
-- localization_torque_2d.lua — 토크 MPC 실험용 측위 (구조도 v3).
--
-- localization_2d.lua 를 그대로 쓰고 두 가지만 바꾼다. 근거·결정은
-- amr/notes_amr/torque_mpc_todo.md "포즈 입력"(2026-11-11)과 stack_changelog.md.
--
-- 1) use_pose_extrapolator = false — Cartographer 외삽을 끈다.
--    /tracked_pose 는 스캔 매칭 결과(local_pose)만, 그 스캔 시각 stamp 로, 새 스캔마다
--    한 번씩 나온다 (cartographer_ros 2.0.9003 src/node.cpp PublishLocalTrajectoryData:
--    stamp = local_slam_data->time, pose = local_slam_data->local_pose, 같은 stamp 는
--    다시 내지 않음). 이 옵션은 발행용 외삽기만 바꾸고 스캔 매칭(로컬 SLAM 내부의
--    외삽기·IMU 사용)에는 영향이 없다.
--    스캔 사이 구간과 약 132 ms 의 지연은 state_estimator 가 IMU 가속도 적분으로
--    메운다 (config/torque_estimator.yaml pose_source: stamped).
--    이유: 외삽된 /tracked_pose 를 참값으로 쓰면 가속할수록 위치가 뒤처지고, 그 값으로
--    MPC 를 닫으면 시뮬에서 발산했다 (2026-11-11).
--    ※ TF(map → base_link)도 스캔 시각으로 늦게 나간다. RViz 의 로봇이 약 0.13 s 늦게
--      보인다. 제어는 /state_estimator/odom 을 쓰므로 무관하다.
--
-- 2) use_odometry = false — 휠 오돔을 쓰지 않는다.
--    토크 모터(AK40)로 바꾸면 ak40_can_node 는 /odom 을 내지 않는다 (구조도 v3).
--    슬립을 보려는 실험이라 휠 오돔이 측위에 섞이면 안 된다.
--    ★ 2026-11-11 bag replay(135325, 30 s, smoc_lab 지도): 휠 오돔을 빼면 측위가 시간에 따라
--      미끄러져 실차 기록과 30 s 에 약 30 cm 벌어지고 돌아오지 않았다 (켜면 5 cm 이내).
--      외삽만 끈 것은 영향 없음 (실차 설정 replay 와 같은 2.5 cm). → 결정 대기, torque_mpc_todo.md
--
-- 발행 주기는 localization_2d 값(10 ms)을 그대로 쓴다. 5 ms 로 줄였을 때 0.5 배속 replay 에서
-- cartographer_ros 발행용 외삽기의 IMU 시각 검사(pose_extrapolator.cc:92)에 걸려 멈춘 적이 있다.
-- ============================================================================

include "localization_2d.lua"

options.use_pose_extrapolator = false
options.use_odometry = false

return options
