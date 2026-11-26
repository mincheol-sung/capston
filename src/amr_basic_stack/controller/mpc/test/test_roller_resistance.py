"""롤러 구름 저항 (2026-11-25, model.py RollerResistance) — 시뮬 플랜트 전용 항.

RP1 식: 앞 0, 옆걸음 −4√2 F, 제자리 회전 −4√2 l_x F, 옆이 우세하면 회전 저항 0. 일률 P 의 유한차분과 같다
RP2 0 이면 플랜트 적분이 비트까지 같다 (무슬립·슬립 둘 다)
RP3 실차 재현: F 0.88 이면 회전 단계 토크 (0.06·0.08·0.10 N·m, 띄운 마찰 보상 없이) 의 가속이 실차와 같은 쪽 — 모델 가속 대비 0.45·0.70·0.81 (± 0.1)
    옆 단계 (0.10 N·m, 띄운 마찰 뺀 비교) 는 옆 저항 4.98 N 만큼 덜 가속
"""
import math

import numpy as np
import pytest

from amr_mpc.torque.model import DrivetrainFriction, RollerResistance, SlipPlant, TorquePlant

P = TorquePlant.from_yaml()
LX = 0.5 * P.wheel_base


def test_formula_and_power_gradient():
    rr = RollerResistance(P, 0.88, 1e-9)
    assert np.allclose(rr.force([1.0, 0.0, 0.0]), 0.0)
    assert rr.force([0.0, 0.3, 0.0])[1] == pytest.approx(-4 * math.sqrt(2) * 0.88)
    assert rr.force([0.0, 0.0, 1.0])[2] == pytest.approx(-4 * math.sqrt(2) * LX * 0.88)
    assert abs(rr.force([0.0, 0.3, 1.0])[2]) < 1e-9 and abs(rr.force([0.0, 0.05, 2.0])[1]) < 1e-9
    rr = RollerResistance(P, 0.88, 0.02)
    Pw = lambda v: 0.88 * np.sum(np.sqrt(np.sum(rr.rel(v) ** 2, axis=1) + 0.02 ** 2))
    for v in ([0.4, 0.1, 0.5], [-0.2, 0.05, -2.0], [0.0, -0.01, 0.1]):
        v = np.array(v)
        fd = np.array([-(Pw(v + 1e-7 * e) - Pw(v - 1e-7 * e)) / 2e-7 for e in np.eye(3)])
        assert np.allclose(rr.force(v), fd, atol=1e-6)
    with pytest.raises(ValueError):
        RollerResistance(P, -0.1)


def test_zero_is_identical():
    rr0 = RollerResistance(P, 0.0)
    X, Xd, tau = np.array([0.1, 0.2, 0.3]), np.array([0.3, -0.2, 0.8]), np.array([0.1, -0.05, 0.08, 0.02])
    fr = DrivetrainFriction(P.fric_coulomb, P.fric_viscous)
    a = P.step(X, Xd, tau, 0.002, fric=fr)
    b = P.step(X, Xd, tau, 0.002, fric=fr, roller=rr0)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
    sp0, sp1 = SlipPlant(P, 0.4, fric=fr), SlipPlant(P, 0.4, fric=fr, roller=rr0)
    w = P.Tinv @ P.body_velocity(X, Xd)
    a, b = sp0.step(X, Xd, w, tau, 0.0005), sp1.step(X, Xd, w, tau, 0.0005)
    assert all(np.array_equal(x, y) for x, y in zip(a, b))


def _accel(pattern, tau, rr, T=0.6, fric=True):
    """바퀴당 τ 를 패턴으로 T 초 (보상 없이 띄운 마찰 그대로) — 평균 가속 (몸체 축)."""
    fr = DrivetrainFriction(P.fric_coulomb, P.fric_viscous, smooth=0.5) if fric else None
    X, Xd = np.zeros(3), np.zeros(3)
    ax = 2 if pattern[0] == -1 and pattern[2] == -1 else 1
    for _ in range(int(T / 0.002)):
        X, Xd = P.step(X, Xd, tau * np.array(pattern, float), 0.002, fric=fr, roller=rr)
    return P.body_velocity(X, Xd)[ax] / T


def test_reproduces_real_rotation_and_lateral():
    rr = RollerResistance(P, 0.88)
    real = {0.06: 0.45, 0.08: 0.70, 0.10: 0.81}
    for tau, ratio in real.items():
        r = _accel([-1, 1, -1, 1], tau, rr) / _accel([-1, 1, -1, 1], tau, None)
        assert r == pytest.approx(ratio, abs=0.12), tau
    m_eff = P.M_eff[1, 1]
    # 옆: 띄운 마찰을 빼고 보면 (속도가 달라 점성 마찰 몫이 달라지는 것을 지운다) 가속 차 × m_eff = 옆 저항 4√2 F
    da = _accel([-1, 1, 1, -1], 0.10, None, 0.3, fric=False) - _accel([-1, 1, 1, -1], 0.10, rr, 0.3, fric=False)
    assert da * m_eff == pytest.approx(4 * math.sqrt(2) * 0.88, rel=0.06)        # 출발 직후 롤러 속도 < s0 (0.02 m/s) 동안 덜 걸려 4.75 N
