# 2026-11-25 — 옛 제자리 회전 bag 으로 회전 관성·바닥 회전 저항 미리 맞춤

명령: `/usr/bin/python3 src/amr_basic_stack/controller/mpc/tools/floor_wheel_tests.py rot bags/2026-11-15_lidar/rot1 bags/2026-11-16_rot/rot_slow --out analysis/torque_mpc_real/2026-11-25_rot_prior`
bag: 11-15 rot1 (basic_motion_test ccw/cw 0.10·0.15·0.20 N·m × 0.25 s, 모터 37 °C), 11-16 rot_slow (0.10 N·m × 0.15·0.2 s, 23 °C). 결과 표는 `floor_rot.md`.
새 시험 (`tools/floor_tests_w4_t7.sh rot`) 은 낮은 토크 (0.06~0.10)·긴 구간 (90°) 으로 MPC 가 실제 회전에 쓰는 범위를 잰다.
