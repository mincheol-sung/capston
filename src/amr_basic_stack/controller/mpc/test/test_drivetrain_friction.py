"""구동계 마찰 (R2 리뷰 2026-11-12) — 시뮬 플랜트 전용 DrivetrainFriction 과 보상(torque_output)의 맞물림."""
import math

import numpy as np
import pytest

from amr_mpc.nodes.torque_output import motor_command
from amr_mpc.torque.model import DrivetrainFriction, SlipPlant, TorquePlant

TC, BV = 0.026, 0.0016


def plant():
    return TorquePlant.from_yaml(use_wheel_inertia=False, fric_coulomb=TC, fric_viscous=BV, fric_max=0.1)


def run(p, tau_fn, T, fric, X=None, Xd=None):
    """무슬립 플랜트를 마찰의 안정 간격으로 T 초 적분."""
    X = np.zeros(3) if X is None else X
    Xd = np.zeros(3) if Xd is None else Xd
    h = min(0.001, fric.stable_dt(p) if fric else 0.001)
    for _ in range(int(round(T / h))):
        X, Xd = p.step(X, Xd, tau_fn(X, Xd), h, fric=fric)
    return X, Xd


def test_friction_shape():
    f = DrivetrainFriction(TC, BV, smooth=0.05)
    out = f([10.0, -10.0, 0.0, 0.01])
    assert out[0] == pytest.approx(TC + BV * 10.0, rel=1e-6)          # 빠르면 쿨롱이 다 걸린다
    assert out[1] == pytest.approx(-(TC + BV * 10.0), rel=1e-6)       # 방향 반대
    assert out[2] == 0.0
    assert 0.0 < out[3] < TC                                          # 0 근처는 부드럽게
    assert f.stiffness == pytest.approx(TC / 0.05 + BV)


def test_small_torque_barely_moves():
    """쿨롱보다 작은 토크는 로봇을 거의 못 민다 (ω_s 수준으로 기어가는 정도)."""
    p, f = plant(), DrivetrainFriction(TC, BV, 0.05)
    _, Xd = run(p, lambda X, Xd: np.full(4, 0.5 * TC), 1.0, f)
    w = p.wheel_speeds(np.zeros(3), Xd)
    assert np.all(np.abs(w) < 0.05)                                    # ω_s 아래 (바퀴 2 mm/s 아래)


def test_straight_acceleration_matches_net_torque():
    """직진에서 가속 = 4(τ − τc − bv·ω)/(r m). 마찰을 빼면 그만큼 덜 가속한다."""
    p, f = plant(), DrivetrainFriction(TC, BV, 0.05)
    tau = 0.08
    X, Xd = run(p, lambda X, Xd: np.full(4, tau), 0.2, f)
    w = p.wheel_speeds(X, Xd)[0]
    a_expected = 4.0 * (tau - TC - BV * w) / (p.r * p.mass)
    X2, Xd2 = p.step(X, Xd, np.full(4, tau), 1e-4, fric=f)
    a = (Xd2[0] - Xd[0]) / 1e-4
    assert a == pytest.approx(a_expected, rel=1e-3)
    assert Xd[0] < 4.0 * tau / (p.r * p.mass) * 0.2                    # 마찰 없을 때보다 느리다


def test_compensation_cancels_friction_at_speed():
    """torque_output 의 보상(ω_s 0.5) + 플랜트 마찰(ω_s 0.05): 바퀴가 충분히 빠르면 순토크가 MPC 의도와 같다."""
    p, f = plant(), DrivetrainFriction(TC, BV, 0.05)
    Xd0 = np.array([0.4, 0.0, 0.0])                                    # 0.4 m/s 직진 → 바퀴 10 rad/s
    tau_net = np.full(4, 0.03)

    def comp(X, Xd):
        return motor_command(tau_net, p.wheel_speeds(X, Xd), TC, BV, 0.5, 1.3)[0]

    _, Xd_f = run(p, comp, 0.2, f, Xd=Xd0.copy())
    _, Xd_0 = run(p, lambda X, Xd: tau_net, 0.2, None, Xd=Xd0.copy())  # 마찰도 보상도 없는 이상 플랜트
    assert Xd_f[0] == pytest.approx(Xd_0[0], abs=2e-4)


def test_rest_is_stable_with_stable_dt():
    """정지·토크 0 에서 안정 간격으로 적분해도 튀지 않는다 (마찰이 뻣뻣해도 발산 없음)."""
    p, f = plant(), DrivetrainFriction(TC, BV, 0.05)
    X, Xd = run(p, lambda X, Xd: np.zeros(4), 0.5, f, Xd=np.array([0.05, -0.02, 0.3]))
    assert np.all(np.isfinite(Xd)) and np.linalg.norm(Xd) < 1e-3     # 마찰이 세운다


def test_slip_plant_free_wheel_spin_down():
    """바닥 마찰이 거의 없을 때(바퀴 띄움 흉내) 바퀴만 구동계 마찰로 선다: J ω̇ = −(τc + bv ω)."""
    p, f = plant(), DrivetrainFriction(TC, BV, 0.05)
    sp = SlipPlant(p, mu=1e-6, fric=f)
    X, Xd, w = np.zeros(3), np.zeros(3), np.full(4, 10.0)
    h = sp.stable_dt()
    assert h < f.stable_dt(p, slip=True)                               # 두 마찰이 합쳐져 간격이 더 짧다
    X2, Xd2, w2 = sp.step(X, Xd, w, np.zeros(4), 1e-4)
    wd = (w2[0] - w[0]) / 1e-4
    assert wd == pytest.approx(-(TC + BV * 10.0) / p.J_w, rel=1e-3)
    for _ in range(int(0.5 / h)):
        X, Xd, w = sp.step(X, Xd, w, np.zeros(4), h)
    assert np.all(np.abs(w) < 0.1)                                     # 10 rad/s 에서 약 0.3 s 안에 선다
    assert math.isfinite(w.sum())
