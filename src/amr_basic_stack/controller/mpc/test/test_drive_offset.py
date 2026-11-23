"""구동계 방향 보정 (amr_mpc/torque/model.py drive_*, 2026-11-23) — C++ TorqueModel::correct_wheel_velocity·force_map 과 같은 식.
실물 가정: v_실제,xy = R(δ)·(T ω)_xy,  δ = δ0 + c·a_c.  근거 analysis/torque_mpc_real/2026-11-23_horizon_real/RESULTS_KO.md 5·6 절."""
import math

import numpy as np
import pytest

from amr_mpc.core import config
from amr_mpc.torque.model import TorquePlant, drive_angle, drive_params, drive_Tinv

TINV = TorquePlant.from_yaml().Tinv
T = np.linalg.pinv(TINV)


def real_motion(omega, angle):
    """바퀴 ω 로 실제 나가는 몸체 속도 (굴러가는 축이 angle 반시계)."""
    v = T @ omega
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def test_zero_is_nominal():
    assert np.array_equal(drive_Tinv(TINV, 0.0), TINV)
    assert drive_angle(0.0, 0.0, [0.9, 0.0, 1.8], 2.0) == 0.0


@pytest.mark.parametrize("deg", [2.4, -1.65, 7.4])
def test_corrected_command_gives_desired_motion(deg):
    a = math.radians(deg)
    for v in ([0.6, 0, 0], [0, 0.3, 0], [0.5, -0.2, 1.1], [0, 0, 1.5]):
        assert np.allclose(real_motion(drive_Tinv(TINV, a) @ v, a), v, atol=1e-12)


def test_nominal_command_drifts_left():
    """보정 없이 앞 0.6 m/s 를 명령하면 왼쪽으로 0.6·sin 2.4° = 25 mm/s 새어 나간다 (11-23 분석의 몸체 왼쪽 치우침)."""
    a = math.radians(2.4)
    out = real_motion(TINV @ [0.6, 0, 0], a)
    assert out[1] == pytest.approx(0.6 * math.sin(a), abs=1e-12) and out[1] > 0.02


def test_sideways_wheel_ratio_is_tan_45_minus_delta():
    """보정 모델로 순수 옆걸음에 필요한 바퀴 비율 (|ω_FL|+|ω_RR|)/(|ω_FR|+|ω_RL|) = tan(45° − δ) — 옆걸음 bag 실측 0.886~0.952 와 비교한 식."""
    a = math.radians(2.67)
    w = np.abs(drive_Tinv(TINV, a) @ [0, 0.3, 0])
    assert (w[0] + w[3]) / (w[1] + w[2]) == pytest.approx(math.tan(math.radians(45 - 2.67)), rel=1e-12)


def test_drive_angle_slip_and_clip():
    y, c = math.radians(2.4), math.radians(-2.5)
    assert drive_angle(y, c, [0.9, 0.0, 1.8], 2.0) == pytest.approx(math.radians(2.4 - 2.5 * 1.62))   # P1 최고속 CCW
    assert drive_angle(y, c, [1.0, 0.0, -3.0], 2.0) == pytest.approx(math.radians(2.4 + 2.5 * 2.0))   # a_c −3 → −2 로 자름


def test_drive_params_default_and_yaml():
    assert drive_params({}) == (0.0, 0.0, 2.0)
    yaw, coef, acm = drive_params(config.load("torque_plant.yaml"))
    assert (yaw, coef) == (0.0, 0.0) and acm == 2.0          # 실차 A/B 전까지 기본 0 (보정 없음)
