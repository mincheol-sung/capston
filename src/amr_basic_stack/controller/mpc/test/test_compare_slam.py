"""tools/compare_slam.py 의 계산부 (2026-11-19) — IMU 점 보정, 경로 좌표 분해, 교차점 창, 무장 구간."""
import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("compare_slam", os.path.join(HERE, "..", "tools", "compare_slam.py"))
cs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cs)

OFF = (0.098, 0.017)       # torque_estimator.yaml imu_offset


def test_imu_point_back_to_center():
    """중심 (1, 2), 헤딩 90° 이면 IMU 점 = 중심 + R(90°) r = (1 − 0.017, 2 + 0.098) → 보정하면 중심."""
    yaw = np.array([math.pi / 2])
    x, y = cs.imu_to_center(np.array([1 - 0.017]), np.array([2 + 0.098]), yaw, OFF)
    assert abs(x[0] - 1) < 1e-12 and abs(y[0] - 2) < 1e-12


def _circle(t, R=0.5, v=0.7):
    """반시계 원 (중심 원점), tangent 헤딩. 기준 속도는 몸체 [v, 0], ω = v/R."""
    w = v / R
    a = w * t
    return R * np.cos(a), R * np.sin(a), a + math.pi / 2, np.full_like(t, v), np.zeros_like(t), np.full_like(t, w)


def test_inward_offset_is_positive_cross_track():
    t = np.linspace(0, 2, 201)
    xr, yr, thr, vx, vy, om = _circle(t)
    x, y = 0.99 * xr, 0.99 * yr              # 반지름 1 cm 안쪽 (R 0.5 의 1 %)
    et, ec, en, ey, sp = cs.path_errors(x, y, thr, xr, yr, thr, vx, vy, om)
    assert np.allclose(ec, 0.005, atol=1e-9) and np.allclose(et, 0, atol=1e-9)    # 0.5 × 1 % = 5 mm 안쪽
    assert np.allclose(sp, 0.7) and np.allclose(ey, 0)


def test_time_lag_is_negative_along_track():
    """0.1 s 늦게 따라가면 e_t = −R sin(ωΔt) ≈ −69.8 mm (0.7 m/s), e_c = R(1 − cos ωΔt) 쪽으로 안."""
    t = np.linspace(1, 2, 101)
    xr, yr, thr, vx, vy, om = _circle(t)
    x, y, th, *_ = _circle(t - 0.1)
    et, ec, en, ey, sp = cs.path_errors(x, y, th, xr, yr, thr, vx, vy, om)
    assert np.allclose(et, -0.5 * math.sin(0.14), atol=1e-9)
    assert np.allclose(ec, 0.5 * (1 - math.cos(0.14)), atol=1e-9)
    assert np.allclose(np.degrees(np.abs(ey)), math.degrees(0.14), atol=1e-9)


def test_clockwise_inward_is_still_positive():
    """시계 방향 원(ω < 0)에서도 곡률 중심 쪽이 +."""
    t = np.linspace(0, 1, 101)
    w = -1.4
    xr, yr = 0.5 * np.cos(w * t), 0.5 * np.sin(w * t)
    thr = w * t - math.pi / 2
    et, ec, *_ = cs.path_errors(0.99 * xr, 0.99 * yr, thr, xr, yr, thr, np.full_like(t, 0.7), np.zeros_like(t), np.full_like(t, w))
    assert np.allclose(ec, 0.005, atol=1e-9)


def test_crossing_window_and_armed_mask():
    t = np.arange(0, 10, 0.01)
    om = np.where(t < 5, 1.4, -1.4)
    on = cs.armed_mask(t, [(0.5, False), (1.0, True), (9.0, False)])
    assert on[(t >= 1.0) & (t <= 9.0)].all() and not on[t < 1.0].any() and not on[t > 9.0].any()
    win, n = cs.crossing_mask(t, om, on)
    assert n == 1
    assert win[(t > 4.5 + 1e-6) & (t < 6.5 - 1e-6)].all() and not win[(t < 4.49) | (t > 6.51)].any()
