"""휠 오도메트리 계산부(amr_mpc/torque/wheel_odometry.py)."""
import math

import numpy as np
import pytest

from amr_mpc.torque.model import TorquePlant
from amr_mpc.torque.wheel_odometry import WheelOdometry, forward_matrix, integrate_se2

P = TorquePlant.from_yaml()


def test_forward_is_inverse_of_driver_kinematics():
    T = forward_matrix(P.r, P.k)
    assert np.allclose(T @ P.Tinv, np.eye(3))          # 드라이버 twist_to_wheels 의 역
    n = np.array([1.0, 1.0, -1.0, -1.0])                # 영공간 (앞뒤가 서로 미는 조합) 은 몸체 속도 0
    assert np.allclose(T @ n, 0.0)


@pytest.mark.parametrize("v", [(0.5, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 1.0), (0.4, -0.2, 0.6)])
def test_constant_twist_matches_closed_form(v):
    odo = WheelOdometry(P.r, P.k)
    w = P.Tinv @ np.array(v)
    for i in range(201):                                 # 2 s, 100 Hz
        x, y, th, vx, vy, wz = odo.update(i * 0.01, w)
    assert (vx, vy, wz) == pytest.approx(v, abs=1e-12)
    xe, ye, the = integrate_se2(0.0, 0.0, 0.0, *v, 2.0)   # 한 번에 정확 적분한 값과 같아야 한다
    assert (x, y) == pytest.approx((xe, ye), abs=1e-9)
    assert th == pytest.approx(the, abs=1e-12)


def test_circle_closes():
    # 1 rad/s 로 2π 초 돌면서 앞으로 0.5 m/s → 반지름 0.5 m 원을 돌아 제자리
    odo = WheelOdometry(P.r, P.k)
    w = P.Tinv @ np.array([0.5, 0.0, 1.0])
    n = 2000
    for i in range(n + 1):
        x, y, th, *_ = odo.update(i * (2 * math.pi / n), w)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert abs(th) < 1e-9


def test_gap_in_samples_is_not_integrated():
    odo = WheelOdometry(P.r, P.k, max_dt=0.1)
    w = P.Tinv @ np.array([1.0, 0.0, 0.0])
    odo.update(0.0, w)
    x, *_ = odo.update(1.0, w)                           # 1 s 끊김 → 그 사이는 모른다, 적분하지 않는다
    assert x == 0.0
    x, *_ = odo.update(1.01, w)
    assert x == pytest.approx(0.01)
