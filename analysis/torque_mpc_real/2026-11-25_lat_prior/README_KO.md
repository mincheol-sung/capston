# 2026-11-25 — 옛 옆걸음 bag (11-15 lat1) 으로 옆 저항 미리 맞춤 · 롤러 구름 저항 가설 확인 1 차

명령: `/usr/bin/python3 src/amr_basic_stack/controller/mpc/tools/floor_wheel_tests.py lat bags/2026-11-15_lidar/lat1 --out analysis/torque_mpc_real/2026-11-25_lat_prior` → `floor_lat.md`
bag: basic_motion_test left/right 0.10·0.15·0.20 N·m × 0.5 s, 모터 35 °C, 판마다 한 회 (6 회).
예측 (회전 시험에서, 이 bag 을 보기 전에 적음): 롤러 구름 저항이면 옆 저항 = 회전 저항 0.40 N·m / l_x 0.0862 m = 4.6 N (범위 4.3~5.1).
결과: m_eff 6.785 고정 회마다 0.10 → 5.50 · 0.15 → 4.87 N (IMU), 회들끼리 맞춤 F_c 5.0~5.4 ± 1.2~2.0 N, m 7.6~8.4 ± 1.2~2.2 kg.
0.20 N·m 는 바퀴/몸체 옆 속도 1.53 (헛돎) 이라 저항이 크게 나온다 (7.4 N) — 빼고 본다. 0.10 판 끝 바퀴 속도가 고르지 않다 (RL 0 rad/s 인 회 있음).
안 쓴 bag: 11-17 step_left (0.5·0.8 N·m, 바퀴가 전압 한계 40 rad/s 에 붙음), 11-16 5-3 left1 (MPC 닫힌 고리라 단계 구간이 아님).
