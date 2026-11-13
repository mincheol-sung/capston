"""토크 모델(amr_mpc/torque/model.py) 검산 — 1판 설계의 수치 예와 기구학 부호."""
import math

import numpy as np
import pytest

from amr_mpc.torque.model import NULL_DIR, TorquePlant


@pytest.fixture(params=[False, True], ids=["no_Jw", "with_Jw"])
def plant(request):
    return TorquePlant.from_yaml(use_wheel_inertia=request.param)


def test_null_space_makes_no_motion(plant):
    a = plant.accel(np.zeros(3), np.zeros(3), 0.3 * NULL_DIR)
    assert np.allclose(a, 0.0)


def test_equal_torque_drives_forward(plant):
    tau = 0.1
    a = plant.accel(np.zeros(3), np.zeros(3), [tau] * 4)
    m_eff = plant.M_eff[0, 0]
    assert a[0] == pytest.approx(4 * tau / (plant.r * m_eff))
    assert a[1] == pytest.approx(0.0, abs=1e-12)
    assert a[2] == pytest.approx(0.0, abs=1e-12)


def test_forward_push_rotates_with_heading(plant):
    # θ = 90° 로 북쪽을 보고 있으면 "앞으로" 가 map 의 +y 다
    a = plant.accel(np.array([0.0, 0.0, math.pi / 2]), np.zeros(3), [0.1] * 4)
    assert a[0] == pytest.approx(0.0, abs=1e-12)
    assert a[1] > 0.0


def test_left_and_yaw_patterns_match_driver_kinematics(plant):
    # 드라이버 twist_to_wheels: FL=(vx−vy−kω)/r, FR=(vx+vy+kω)/r, RL=(vx+vy−kω)/r, RR=(vx−vy+kω)/r
    left = plant.accel(np.zeros(3), np.zeros(3), [-0.1, 0.1, 0.1, -0.1])
    assert left[1] > 0 and abs(left[0]) < 1e-12 and abs(left[2]) < 1e-12
    ccw = plant.accel(np.zeros(3), np.zeros(3), [-0.1, 0.1, -0.1, 0.1])
    assert ccw[2] > 0 and abs(ccw[0]) < 1e-12 and abs(ccw[1]) < 1e-12


def test_wheel_speeds_no_slip(plant):
    w = plant.wheel_speeds(np.zeros(3), np.array([0.5, 0.0, 0.0]))
    assert np.allclose(w, 0.5 / plant.r)


def test_wheel_inertia_switch():
    off = TorquePlant.from_yaml(use_wheel_inertia=False)
    on = TorquePlant.from_yaml(use_wheel_inertia=True)
    assert off.mu == 0.0
    assert on.mu == pytest.approx(4 * on.J_w / on.r ** 2)      # J_w 9.7e-4 이면 2.425 kg (1판 식-3)
    assert np.allclose(off.w(np.array([1.0, 0.0, 1.0])), 0.0)
    assert not np.allclose(on.w(np.array([1.0, 0.0, 1.0])), 0.0)


def test_friction_limit_matches_design_example():
    # 1판 식-21 수치 예: μ̂ 0.3, c_s 0.8, m 4 kg → 0.067 N·m
    p = TorquePlant.from_yaml(mass=4.0, mu_hat=0.3, c_s=0.8)
    assert p.tau_friction_limit() == pytest.approx(0.0666, abs=5e-4)


def test_constant_torque_integration_matches_closed_form(plant):
    X, Xd = np.zeros(3), np.zeros(3)
    tau, dt, n = 0.05, 0.001, 1000
    for _ in range(n):
        X, Xd = plant.step(X, Xd, [tau] * 4, dt)
    a = 4 * tau / (plant.r * plant.M_eff[0, 0])
    assert X[0] == pytest.approx(0.5 * a * (n * dt) ** 2, rel=1e-9)
    assert Xd[0] == pytest.approx(a * n * dt, rel=1e-9)


# ── 슬립 모델 (5단계) ─────────────────────────────────────────────────────────
from amr_mpc.torque.model import SlipPlant


def run_slip(sp, tau, T=1.0, dt=None):
    X, Xd, w = np.zeros(3), np.zeros(3), np.zeros(4)
    dt = dt or min(1e-3, sp.stable_dt())
    for _ in range(int(T / dt)):
        X, Xd, w = sp.step(X, Xd, w, tau, dt)
    return X, Xd, w


def test_slip_model_matches_no_slip_model_under_small_torque():
    # 마찰 한계보다 훨씬 작은 토크면 무슬립 모델(바퀴 관성 켬)과 같은 가속을 낸다
    p = TorquePlant.from_yaml(use_wheel_inertia=True)
    sp = SlipPlant(p, mu=1.0)
    tau = [0.01] * 4
    X, Xd, w = run_slip(sp, tau, T=1.0)
    a_ref = p.accel(np.zeros(3), np.zeros(3), tau)[0]
    assert Xd[0] == pytest.approx(a_ref * 1.0, rel=0.02)
    assert np.allclose(w, p.wheel_speeds(X, Xd), rtol=0.02)          # 바퀴와 몸체가 맞물려 돈다


def test_slip_model_saturates_at_friction_limit():
    # 마찰 한계를 넘는 토크: 몸체 가속은 μ g 로 묶이고 바퀴는 헛돈다
    p = TorquePlant.from_yaml()
    sp = SlipPlant(p, mu=0.3)
    tau_lim = p.r / math.sqrt(2.0) * sp.Fmax                          # 1판 식-21 (c_s 없이)
    X, Xd, w = run_slip(sp, [3.0 * tau_lim] * 4, T=0.5)
    a_max = 4 * sp.Fmax / math.sqrt(2.0) / p.mass                    # 네 바퀴 힘의 전방 성분 합 / m
    assert Xd[0] == pytest.approx(a_max * 0.5, rel=0.05)
    assert a_max == pytest.approx(0.3 * 9.81 / math.sqrt(2.0), rel=1e-6)   # 메카넘 전진 최대 가속 = μg/√2 (힘이 45° 로 나간다)
    v_nom = p.wheel_speeds(X, Xd)
    assert np.all(w > 1.5 * v_nom)                                    # 바퀴가 몸체보다 훨씬 빨리 돈다


def test_slip_model_rotation_and_strafe_signs():
    p = TorquePlant.from_yaml()
    sp = SlipPlant(p, mu=1.0)
    _, Xd, _ = run_slip(sp, [-0.01, 0.01, 0.01, -0.01], T=0.3)       # 왼쪽 게걸음
    assert Xd[1] > 0 and abs(Xd[0]) < 1e-6 and abs(Xd[2]) < 1e-6
    _, Xd, _ = run_slip(sp, [-0.002, 0.002, -0.002, 0.002], T=0.3)   # 반시계 회전
    assert Xd[2] > 0 and abs(Xd[0]) < 1e-6 and abs(Xd[1]) < 1e-6
