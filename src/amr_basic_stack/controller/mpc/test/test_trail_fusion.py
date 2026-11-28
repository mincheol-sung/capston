"""core/trail_fusion.py — RViz 지나간 경로 (SLAM → base_footprint, 점프 없는 경로) 2026-11-26."""
import math

import numpy as np

from amr_mpc.core.trail_fusion import SmoothTrail, mecanum_fwd, slam_to_base

R_IMU = (0.1093, 0.017)
T_FWD = mecanum_fwd(0.04, 0.1724, 0.2140)
T_INV = np.linalg.pinv(T_FWD)


def test_slam_to_base_removes_lever_arm():
    for psi in (0.0, 0.7, -2.5):
        c, s = math.cos(psi), math.sin(psi)
        tx, ty = 1.0 + c * R_IMU[0] - s * R_IMU[1], 2.0 + s * R_IMU[0] + c * R_IMU[1]
        bx, by, bp = slam_to_base(tx, ty, psi, *R_IMU)
        assert abs(bx - 1.0) < 1e-12 and abs(by - 2.0) < 1e-12 and bp == psi


def run(slam_bias=(0.0, 0.0), slam_noise=0.0, delay=0.11, seed=0):
    """원 위를 0.5 m/s 로 도는 로봇. 바퀴·자이로는 정확, SLAM 은 10 Hz·delay 늦게·잡음·위치 편향."""
    rng = np.random.default_rng(seed)
    st = SmoothTrail(T_FWD, R_IMU, tau=0.5)
    v, wz, dt = 0.5, 0.5, 0.01
    out, truth, pend = [], [], []
    for i in range(1500):
        t = i * dt
        psi = wz * t
        x, y = math.sin(psi) / wz * v, (1 - math.cos(psi)) / wz * v
        if i % 10 == 0:
            c, s = math.cos(psi), math.sin(psi)
            pend.append((t + delay, t, x + c * R_IMU[0] - s * R_IMU[1] + slam_bias[0] + rng.normal(0, slam_noise),
                         y + s * R_IMU[0] + c * R_IMU[1] + slam_bias[1] + rng.normal(0, slam_noise), psi))
        while pend and pend[0][0] <= t:
            _, ts, sx, sy, sp = pend.pop(0)
            st.on_slam(ts, sx, sy, sp)
        st.on_gyro(wz)
        p = st.on_wheels(t, T_INV @ np.array([v, 0.0, wz]))
        if p is not None:
            out.append(p)
            truth.append((x, y, psi))
    return np.array(out), np.array(truth)


def test_tracks_truth_with_perfect_wheels():
    out, truth = run()
    err = np.hypot(out[50:, 0] - truth[50:, 0], out[50:, 1] - truth[50:, 1])
    assert err.max() < 2e-3


def test_slam_offset_is_blended_without_jump():
    # SLAM 이 5 cm 편향되어 있으면 경로는 그쪽으로 옮겨 가되, 한 표본(10 ms)에 바퀴 이동 밖으로 2 mm 넘게 뛰지 않는다
    out, truth = run(slam_bias=(0.05, 0.0))
    step = np.hypot(*np.diff(out[:, :2], axis=0).T) - 0.5 * 0.01
    assert step[5:].max() < 2e-3
    assert abs((out[-1, 0] - truth[-1, 0]) - 0.05) < 3e-3


def test_slam_noise_is_attenuated():
    out, truth = run(slam_noise=0.01, seed=1)
    err = np.hypot(out[100:, 0] - truth[100:, 0], out[100:, 1] - truth[100:, 1])
    # SLAM 잡음 σ 1 cm (축당) → 위치 RMS √2 cm. 그 60 % 아래로
    assert np.sqrt(np.mean(err ** 2)) < 0.6 * math.sqrt(2) * 0.01
