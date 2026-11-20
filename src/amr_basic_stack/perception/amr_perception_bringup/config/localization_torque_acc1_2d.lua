-- 토크 MPC 실험용 측위 — 라이다 스캔을 하나씩 매칭 (2026-11-19, 시험용)
--
-- localization_torque_2d.lua 와 같고 num_accumulated_range_data 만 2 → 1.
-- 왜: 2026-11-18 bag 175240 에서 /tracked_pose 가 stamp 보다 131 ms 늦게 왔다.
--   라이다 한 바퀴 100 ms (stamp = 첫 점, 다 돌아야 메시지) + 앞·뒤 두 스캔 누적 약 25 ms + Cartographer 계산 5.9 ms.
--   하나씩 매칭하면 누적 몫이 빠져 약 106 ms, 포즈 20 Hz (앞·뒤 번갈아) 로 기대한다. 스캔 하나에 점 3240 개 (DenseBoost).
-- 같이 바꿀 것: 추정기 게인 설계의 포즈 주기 0.1 → 0.05 s (real_all 은 lua 이름에 acc1 이 있으면 est_pose_period 0.05 를 자동으로 넘긴다).
-- 확인: amrlag (추정기 v_x 지연), amrcmp (추종 오차), /tracked_pose 주기·수신 − stamp, 측위 튐 (/state_estimator/status).
--
--   amrall ... lua:=localization_torque_acc1_2d.lua

include "localization_torque_2d.lua"

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

return options
